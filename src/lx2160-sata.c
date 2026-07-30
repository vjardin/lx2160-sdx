/*
 * lx2160-sata — LX2160A SATA lane + AHCI controller state reader.
 *
 * Run (root required, /dev/mem access):
 *   ./lx2160-sata                 # scan + report
 *   ./lx2160-sata --quiet         # one line per SATA lane/controller
 *
 * WARNING: it was not properly tested.
 */

#define _DEFAULT_SOURCE

#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#define EXIT_USAGE 2

#define DCFG_BASE          0x01E00000UL
#define DCFG_MAP_LEN       0x1000UL
#define OFF_RCWSR29        0x170

#define SD_MAP_LEN         0x1000UL
static const uint64_t sd_base[4] = { 0, 0x01EA0000UL, 0x01EB0000UL, 0x01EC0000UL };

#define OFF_PLLFRSTCTL     0x400
#define OFF_PLLFCR0        0x404
#define OFF_PLLFCR1        0x408
#define OFF_PLLSRSTCTL     0x500
#define OFF_PLLSCR0        0x504
#define OFF_PLLSCR1        0x508
#define OFF_LANE_GCR0      0x800
#define OFF_LANE_TRSTCTL   0x820
#define OFF_LANE_RRSTCTL   0x840
#define LANE_STRIDE        0x100
#define NUM_LANES          8

#define PLL_RSTCTL_DIS      24
#define PLL_RSTCTL_LOCK     23
#define LN_RST_DONE         30
#define LN_DIS              24
#define LN_RRSTCTL_CDR_LOCK 12

#define PROTO_SATA          0x02

/* AHCI controllers, one port each (fsl,lx2160a-ahci). */
#define AHCI_MAP_LEN       0x1000UL
static const uint64_t ahci_base[4] = {
    0x03200000UL, 0x03210000UL, 0x03220000UL, 0x03230000UL,
};

#define AHCI_CAP           0x000
#define AHCI_PI            0x00C
#define AHCI_VS            0x010
#define AHCI_PXCMD         0x118
#define AHCI_PXSIG         0x124
#define AHCI_PXSSTS        0x128
#define AHCI_PXSERR        0x130

enum mode { MODE_HUMAN, MODE_QUIET };

static inline uint32_t rd32(const volatile uint32_t *m, uint32_t off)
{
    return m[off / 4];
}

static inline uint32_t bit(uint32_t v, unsigned b) { return (v >> b) & 1u; }

static volatile uint32_t *map_phys(int fd, uint64_t base, size_t len)
{
    void *p = mmap(NULL, len, PROT_READ, MAP_SHARED, fd, (off_t)base);
    if (p == MAP_FAILED) {
        warn("mmap(0x%" PRIx64 ")", base);
        return NULL;
    }
    return (volatile uint32_t *)p;
}

static const char *refclk_mhz(uint32_t cr0)
{
    switch ((cr0 >> 16) & 0x1fu) {
    case 0x00: return "100 MHz";
    case 0x01: return "125 MHz";
    case 0x02: return "156.25 MHz";
    case 0x03: return "150 MHz";
    case 0x04: return "161.1328125 MHz";
    default:   return "reserved";
    }
}

/* SATA runs from the 6 GHz clock-net (FRATE_SEL=0b10010) */
static bool pll_is_sata(uint32_t cr1)
{
    return ((cr1 >> 24) & 0x1fu) == 0x12;
}

static const char *det_str(uint32_t d)
{
    switch (d & 0xfu) {
    case 0: return "no device detected";
    case 1: return "device presence detected, no phy communication";
    case 3: return "device presence detected, phy communication established";
    case 4: return "phy offline (disabled or loopback)";
    default: return "reserved";
    }
}

static const char *spd_str(uint32_t s)
{
    switch (s & 0xfu) {
    case 0: return "-";
    case 1: return "1.5 Gbps (Gen1)";
    case 2: return "3 Gbps (Gen2)";
    case 3: return "6 Gbps (Gen3)";
    default: return "?";
    }
}

static const char *ipm_str(uint32_t i)
{
    switch (i & 0xfu) {
    case 0: return "not present";
    case 1: return "active";
    case 2: return "partial";
    case 6: return "slumber";
    case 8: return "devsleep";
    default: return "?";
    }
}

static const char *sig_str(uint32_t sig)
{
    switch (sig) {
    case 0x00000101: return "ATA disk";
    case 0xEB140101: return "ATAPI";
    case 0xC33C0101: return "enclosure (SEMB)";
    case 0x96690101: return "port multiplier";
    case 0xFFFFFFFF: return "none";
    default:         return "unknown";
    }
}

static void
usage(const char *argv0)
{
    fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "Options:\n"
        "  -q, --quiet      One line per SATA lane / AHCI controller\n"
        "  -h, --help       Show this help and exit\n"
        "\n"
        "Exit codes: 0 no SATA configured, or every SATA link healthy;\n"
        "            1 a SATA lane/port is unhealthy; 2 usage error\n",
        argv0);
}

int
main(int argc, char **argv)
{
    enum mode m = MODE_HUMAN;

    static const struct option longopts[] = {
        { "quiet", no_argument, NULL, 'q' },
        { "help",  no_argument, NULL, 'h' },
        { NULL, 0, NULL, 0 },
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "qh", longopts, NULL)) != -1) {
        switch (opt) {
        case 'q': m = MODE_QUIET; break;
        case 'h': usage(argv[0]); return EXIT_SUCCESS;
        default:  usage(argv[0]); return EXIT_USAGE;
        }
    }

    int fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (fd < 0)
        err(EXIT_FAILURE, "open /dev/mem (need root)");

    volatile uint32_t *dcfg = map_phys(fd, DCFG_BASE, DCFG_MAP_LEN);
    if (!dcfg)
        return EXIT_FAILURE;

    uint32_t rcwsr29 = rd32(dcfg, OFF_RCWSR29);
    uint32_t proto[4];
    proto[1] = (rcwsr29 >> 16) & 0x1fu;
    proto[2] = (rcwsr29 >> 21) & 0x1fu;
    proto[3] = (rcwsr29 >> 26) & 0x1fu;

    if (m == MODE_HUMAN)
        printf("Latched RCW SerDes protocols (RCWSR29=0x%08x): S1=%u S2=%u S3=%u\n",
               rcwsr29, proto[1], proto[2], proto[3]);

    int failures = 0;
    int sata_lanes = 0;

    /* 1+2: silicon scan for SATA-mode lanes */
    for (int s = 1; s <= 3; s++) {
        volatile uint32_t *sd = map_phys(fd, sd_base[s], SD_MAP_LEN);
        if (!sd) {
            failures++;
            continue;
        }

        uint32_t frst = rd32(sd, OFF_PLLFRSTCTL), fcr0 = rd32(sd, OFF_PLLFCR0),
                 fcr1 = rd32(sd, OFF_PLLFCR1);
        uint32_t srst = rd32(sd, OFF_PLLSRSTCTL), scr0 = rd32(sd, OFF_PLLSCR0),
                 scr1 = rd32(sd, OFF_PLLSCR1);

        for (int l = 0; l < NUM_LANES; l++) {
            uint32_t gcr0 = rd32(sd, OFF_LANE_GCR0 + (uint32_t)l * LANE_STRIDE);
            if (((gcr0 >> 3) & 0x1fu) != PROTO_SATA)
                continue;
            sata_lanes++;

            uint32_t trst = rd32(sd, OFF_LANE_TRSTCTL + (uint32_t)l * LANE_STRIDE);
            uint32_t rrst = rd32(sd, OFF_LANE_RRSTCTL + (uint32_t)l * LANE_STRIDE);
            bool cdr = bit(rrst, LN_RRSTCTL_CDR_LOCK);

            /* the PLL with the 6 GHz clock-net serves SATA */
            const char *pll = "?";
            uint32_t pcr0 = 0;
            bool lock = false;
            if (!bit(frst, PLL_RSTCTL_DIS) && pll_is_sata(fcr1)) {
                pll = "PLLF"; pcr0 = fcr0; lock = bit(frst, PLL_RSTCTL_LOCK);
            } else if (!bit(srst, PLL_RSTCTL_DIS) && pll_is_sata(scr1)) {
                pll = "PLLS"; pcr0 = scr0; lock = bit(srst, PLL_RSTCTL_LOCK);
            }
            if (!cdr || !lock)
                failures++;

            if (m == MODE_QUIET)
                printf("SD%d LN%c SATA CDR=%u PLL=%s%s refclk=%s rate=sw-select(1.5/3/6G, see AHCI)\n",
                       s, 'A' + l, cdr, pll,
                       lock ? "(LOCKED)" : "(NOT-locked)", refclk_mhz(pcr0));
            else
                printf("\nSD%d LN%c  mode=SATA (LNmGCR0)  TX_RST_DONE=%u RX_RST_DONE=%u CDR_LOCK=%u\n"
                       "  clock = %s %s, refclk=%s on SD%d_%s_REF_CLK_P/N (external), 6 GHz clknet / 24 GHz VCO\n"
                       "  rate  = 1.5/3/6 Gbps, software-selected -- negotiated value is in AHCI PxSSTS below\n",
                       s, 'A' + l, bit(trst, LN_RST_DONE), bit(rrst, LN_RST_DONE), cdr,
                       pll, lock ? "LOCKED" : "*** NOT LOCKED ***",
                       refclk_mhz(pcr0), s, pll);
        }
    }

    if (sata_lanes == 0) {
        printf("No SerDes lane is muxed to SATA by the latched RCW "
               "(S1=%u S2=%u S3=%u): nothing to report.\n"
               "The AHCI controllers are deliberately not probed (likely "
               "clock-gated without SATA lanes).\n",
               proto[1], proto[2], proto[3]);
        return failures ? EXIT_FAILURE : EXIT_SUCCESS;
    }

    /* 3: AHCI controllers: only reached when SATA lanes exist */
    if (m == MODE_HUMAN)
        printf("\nAHCI controllers (sata0..3, one port each):\n");

    for (int c = 0; c < 4; c++) {
        volatile uint32_t *hba = map_phys(fd, ahci_base[c], AHCI_MAP_LEN);
        if (!hba) {
            failures++;
            continue;
        }

        uint32_t cap  = rd32(hba, AHCI_CAP);
        uint32_t pi   = rd32(hba, AHCI_PI);
        uint32_t ssts = rd32(hba, AHCI_PXSSTS);
        uint32_t sig  = rd32(hba, AHCI_PXSIG);
        uint32_t serr = rd32(hba, AHCI_PXSERR);
        uint32_t det  = ssts & 0xfu;
        uint32_t spd  = (ssts >> 4) & 0xfu;
        uint32_t ipm  = (ssts >> 8) & 0xfu;

        if (cap == 0xffffffffu || (cap == 0 && pi == 0)) {
            printf("%ssata%d @0x%08" PRIx64 ": not accessible (clock-gated?)\n",
                   m == MODE_HUMAN ? "  " : "", c, ahci_base[c]);
            continue;
        }

        if (det != 3)
            failures++;

        if (m == MODE_QUIET) {
            printf("sata%d PxSSTS=0x%08x DET=%u(%s) SPD=%s IPM=%s SIG=%s SERR=0x%08x\n",
                   c, ssts, det, det_str(det), spd_str(spd), ipm_str(ipm),
                   sig_str(sig), serr);
        } else {
            printf("  sata%d @0x%08" PRIx64 "  CAP=0x%08x PI=0x%08x\n",
                   c, ahci_base[c], cap, pi);
            printf("         PxSSTS=0x%08x  DET=%u (%s)\n", ssts, det, det_str(det));
            printf("         speed=%s  power=%s  PxSIG=0x%08x (%s)  PxSERR=0x%08x\n",
                   spd_str(spd), ipm_str(ipm), sig, sig_str(sig), serr);
        }
    }

    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
