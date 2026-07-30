/*
 * lx2160-pcie — LX2160A PCIe controller/slot state reader (Linux userland).
 *
 * For every PCIe controller (PEX1..PEX6) the loaded RCW enables, decode
 * over /dev/mem:
 *   - which SerDes block + lanes carry it (from the latched RCWSR
 *     protocol fields, not from source files),
 *   - the DWC controller's live LTSSM training state (PORT_DEBUG0/1)
 *     with a plain-language verdict (slot empty / endpoint powered but
 *     silent / training / link up),
 *   - the root port's config-space link capability vs. status
 *     (LNKCAP / LNKSTA: negotiated speed + width),
 *   - the SerDes PLL that clocks it: LOCK, reference-clock frequency
 *     and source pins, clock-net rate (PLLnRSTCTL / CR0 / CR1),
 *   - per-lane CDR lock (is the endpoint transmitting at all?),
 *   - the RCW's PCIe rate cap (SRDS_DIV_PEX_Sn).
 *
 * What this tool can NOT see: slot 3.3 V power rails and the PERST#
 * level at the connector are board signals with no LX2160A-side
 * register (on Nodebox v3 they are owned by the carrier MCU). The
 * LTSSM verdict is the closest SoC-side inference: reaching Polling
 * requires the endpoint's receiver terminations to be detected, which
 * requires endpoint PHY power.
 *
 * Run (root required, /dev/mem access):
 *   ./lx2160-pcie                 # every RCW-enabled controller
 *   ./lx2160-pcie --pex 6         # only PEX6
 *   ./lx2160-pcie --watch 10      # + re-sample LTSSM 10x at 1 Hz
 *   ./lx2160-pcie --quiet         # one line per controller
 *
 * References
 *   LX2160ARM.pdf §26 SerDes Module (26.4.1.2/.3/.4 PLL regs,
 *     26.4.1.12 LNmGCR0, 26.4.1.17 LNmRRSTCTL CDR_LOCK)
 *   LX2160ARM.pdf RCWSR29/30 SRDS_PRTCL_Sn / SRDS_DIV_PEX_Sn /
 *     SRDS_PLL_REF_CLK_SEL_Sn (DCFG 0x1E00170/0x1E00174)
 *   Synopsys DWC PCIe PORT_DEBUG0 (0x728) LTSSM encoding — mirrored
 *     by linux/drivers/pci/controller/dwc/pcie-designware.h
 *   PCIe Base Spec: LTSSM Detect requires receiver detection before
 *     Polling may be entered.
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

/* DCFG: RCW status registers (RCWSR1 at 0x1E00100, 32-bit each). */
#define DCFG_BASE          0x01E00000UL
#define DCFG_MAP_LEN       0x1000UL
#define OFF_RCWSR29        0x170    /* SRDS_PLL_PD [5:0], PRTCL S1 [20:16] S2 [25:21] S3 [30:26] */
#define OFF_RCWSR30        0x174    /* PLL_REF_CLK_SEL S1 [5:4] S2 [7:6] S3 [9:8];
                                     * DIV_PEX S1 [17:16] S2 [19:18] S3 [21:20] */

/* SerDes blocks */
#define SD_MAP_LEN         0x1000UL
static const uint64_t sd_base[4] = { 0, 0x01EA0000UL, 0x01EB0000UL, 0x01EC0000UL };

#define OFF_PLLFRSTCTL     0x400
#define OFF_PLLFCR0        0x404
#define OFF_PLLFCR1        0x408
#define OFF_PLLSRSTCTL     0x500
#define OFF_PLLSCR0        0x504
#define OFF_PLLSCR1        0x508
#define OFF_LANE_TRSTCTL   0x820
#define OFF_LANE_RRSTCTL   0x840
#define LANE_STRIDE        0x100
#define PLL_RSTCTL_RST_DONE 30
#define PLL_RSTCTL_DIS      24
#define PLL_RSTCTL_LOCK     23
#define LN_RRSTCTL_CDR_LOCK 12
#define LN_DIS              24

/* PEX controllers: DBI/CCSR bases. PEX1/2 exist only on SD1 protocols. */
#define PEX_MAP_LEN        0x1000UL
static const uint64_t pex_base[7] = {
    0, 0x03400000UL, 0x03500000UL, 0x03600000UL,
       0x03700000UL, 0x03800000UL, 0x03900000UL,
};

/* DWC debug registers in the DBI space */
#define PORT_DEBUG0        0x728    /* [5:0] LTSSM state */
#define PORT_DEBUG1        0x72C    /* bit4 LINK_UP, bit29 LINK_IN_TRAINING */

/*
 * Which PEX a given (SerDes block, protocol) pair enables, and on which
 * lanes. Only protocols this tool knows are listed; an active protocol
 * with no entry is reported as such. (LX2160ARM SerDes protocol tables.)
 */
struct pcie_map {
    int      sd;        /* SerDes block 1..3 */
    uint32_t proto;     /* SRDS_PRTCL_Sn */
    int      pex;       /* controller number 1..6 */
    int      first_lane;
    int      nlanes;
};

static const struct pcie_map pcie_maps[] = {
    { 2,  7, 3, 0, 1 },   /* SD2 proto 7: PCIe.3 x1 on lane A */
    { 2,  7, 4, 4, 1 },   /* SD2 proto 7: PCIe.4 x1 on lane E */
    { 3,  2, 5, 0, 8 },   /* SD3 proto 2: PCIe.5 x8 */
    { 3,  3, 5, 0, 4 },   /* SD3 proto 3: PCIe.5 x4 on lanes A-D */
    { 3,  3, 6, 4, 4 },   /* SD3 proto 3: PCIe.6 x4 on lanes E-H */
};

enum mode { MODE_HUMAN, MODE_QUIET };

/* Helpers */
static inline uint32_t rd32(const volatile uint32_t *m, uint32_t off)
{
    return m[off / 4];
}

static inline uint32_t rd16(const volatile uint32_t *m, uint32_t off)
{
    return (rd32(m, off & ~3u) >> ((off & 3u) * 8)) & 0xffffu;
}

static inline uint32_t rd8(const volatile uint32_t *m, uint32_t off)
{
    return (rd32(m, off & ~3u) >> ((off & 3u) * 8)) & 0xffu;
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

/* PLLnCR0[REFCLK_SEL] (bits 20:16) -> refclk frequency, RM 26.4.1.3. */
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

/* PLLnCR1[FRATE_SEL] (bits 28:24) -> clock-net / VCO rate, RM 26.4.1.4. */
static const char *frate(uint32_t cr1)
{
    switch ((cr1 >> 24) & 0x1fu) {
    case 0x00: return "5 GHz clknet / 20 GHz VCO";
    case 0x06: return "10.3125 GHz clknet / 20.625 GHz VCO";
    case 0x10: return "5 GHz clknet / 25 GHz VCO";
    case 0x12: return "6 GHz clknet / 24 GHz VCO";
    case 0x16: return "12.890625 GHz clknet / 25.78125 GHz VCO";
    case 0x17: return "8 GHz clknet / 24 GHz VCO";
    case 0x19: return "8 GHz clknet / 16 GHz VCO";
    default:   return "reserved";
    }
}

/* Is this PLL configured with a 5 GHz clock-net (the PCIe rate family)? */
static bool pll_is_pcie(uint32_t cr1)
{
    uint32_t f = (cr1 >> 24) & 0x1fu;
    return f == 0x00 || f == 0x10;
}

/*
 * Is this PLL an armed PCIe Gen3 standby (RM 26.10.3.1)? Gen1/gen2
 * train from the 5 GHz PLL; a PLL parked by the RCW but preset with
 * an 8 GHz clock-net is what the PCIe hardware switches the lanes to
 * on the fly at the Gen3 speed change.
 */
static bool pll_is_gen3_standby(uint32_t rst, uint32_t cr1)
{
    uint32_t f = (cr1 >> 24) & 0x1fu;
    return bit(rst, PLL_RSTCTL_DIS) && (f == 0x17 || f == 0x19);
}

/* SRDS_DIV_PEX_Sn -> max trainable rate (RM: 01=8G, 10=5G, 11=2.5G). */
static const char *div_pex_str(uint32_t v)
{
    switch (v & 3u) {
    case 1:  return "8 GT/s (Gen3)";
    case 2:  return "5 GT/s (Gen2)";
    case 3:  return "2.5 GT/s (Gen1)";
    default: return "reserved(0)";
    }
}

/* PCIe cap Link speed code -> string */
static const char *speed_str(uint32_t s)
{
    switch (s & 0xfu) {
    case 1:  return "2.5 GT/s (Gen1)";
    case 2:  return "5 GT/s (Gen2)";
    case 3:  return "8 GT/s (Gen3)";
    case 4:  return "16 GT/s (Gen4)";
    default: return "?";
    }
}

/* DWC PORT_DEBUG0[5:0] LTSSM state names (pcie-designware.h). */
static const char *ltssm_name(uint32_t s)
{
    static const char *tab[] = {
        [0x00] = "Detect.Quiet",     [0x01] = "Detect.Active",
        [0x02] = "Polling.Active",   [0x03] = "Polling.Compliance",
        [0x04] = "Polling.Config",   [0x05] = "Pre.Detect.Quiet",
        [0x06] = "Detect.Wait",      [0x07] = "Cfg.Linkwidth.Start",
        [0x08] = "Cfg.Linkwidth.Accept", [0x09] = "Cfg.Lanenum.Wait",
        [0x0a] = "Cfg.Lanenum.Accept", [0x0b] = "Cfg.Complete",
        [0x0c] = "Cfg.Idle",         [0x0d] = "Recovery.RcvrLock",
        [0x0e] = "Recovery.Speed",   [0x0f] = "Recovery.RcvrCfg",
        [0x10] = "Recovery.Idle",    [0x11] = "L0",
        [0x12] = "L0s",              [0x13] = "L123.SendEIdle",
        [0x14] = "L1.Idle",          [0x15] = "L2.Idle",
        [0x16] = "L2.Wake",          [0x17] = "Disabled.Entry",
        [0x18] = "Disabled.Idle",    [0x19] = "Disabled",
        [0x1a] = "Loopback.Entry",   [0x1b] = "Loopback.Active",
        [0x1c] = "Loopback.Exit",    [0x1d] = "Loopback.Exit.Timeout",
        [0x1e] = "HotReset.Entry",   [0x1f] = "HotReset",
        [0x20] = "Recovery.Eq0",     [0x21] = "Recovery.Eq1",
        [0x22] = "Recovery.Eq2",     [0x23] = "Recovery.Eq3",
    };
    if (s < sizeof(tab) / sizeof(tab[0]) && tab[s])
        return tab[s];
    return "unknown";
}

/*
 * Plain-language reading of the LTSSM state. The Detect->Polling
 * transition happens only after receiver detection succeeded, so
 * Polling.* is positive proof the endpoint's RX terminations are
 * present (= endpoint PHY has power).
 */
static const char *ltssm_verdict(uint32_t s)
{
    if (s <= 0x01 || s == 0x05 || s == 0x06)
        return "no receiver termination detected -- slot empty or endpoint unpowered";
    if (s >= 0x02 && s <= 0x04)
        return "endpoint termination DETECTED (endpoint PHY powered) but endpoint "
               "not transmitting -- check PERST# release and endpoint refclk";
    if (s >= 0x07 && s <= 0x0c)
        return "link width/lane negotiation in progress";
    if ((s >= 0x0d && s <= 0x10) || (s >= 0x20 && s <= 0x23))
        return "recovery / equalization in progress";
    if (s == 0x11)
        return "link up (L0)";
    if (s >= 0x12 && s <= 0x16)
        return "link up, power-save substate";
    if (s >= 0x17 && s <= 0x19)
        return "port disabled";
    if (s >= 0x1a && s <= 0x1d)
        return "loopback";
    if (s >= 0x1e && s <= 0x1f)
        return "hot reset";
    return "unknown state";
}

/* Walk the config-space capability list for the PCIe cap (id 0x10). */
static int find_pcie_cap(const volatile uint32_t *dbi)
{
    if (!(rd16(dbi, 0x06) & 0x0010))   /* Status.CapList */
        return -1;
    uint32_t ptr = rd8(dbi, 0x34) & 0xfcu;
    for (int guard = 0; guard < 48 && ptr >= 0x40; guard++) {
        uint32_t id = rd8(dbi, ptr);
        if (id == 0x10)
            return (int)ptr;
        if (id == 0xff)
            break;
        ptr = rd8(dbi, ptr + 1) & 0xfcu;
    }
    return -1;
}

static void
usage(const char *argv0)
{
    fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "Options:\n"
        "  -p, --pex N      Only report PCIe controller N (1..6)\n"
        "  -w, --watch N    After the report, re-sample each LTSSM N times at 1 Hz\n"
        "  -q, --quiet      One-line summary per controller\n"
        "  -h, --help       Show this help and exit\n"
        "\n"
        "Exit codes: 0 all reported links up or intentionally absent;\n"
        "            1 at least one enabled link not in L0; 2 usage error\n",
        argv0);
}

int
main(int argc, char **argv)
{
    int  filter_pex = -1;
    int  watch = 0;
    enum mode m = MODE_HUMAN;

    static const struct option longopts[] = {
        { "pex",   required_argument, NULL, 'p' },
        { "watch", required_argument, NULL, 'w' },
        { "quiet", no_argument,       NULL, 'q' },
        { "help",  no_argument,       NULL, 'h' },
        { NULL, 0, NULL, 0 },
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "p:w:qh", longopts, NULL)) != -1) {
        switch (opt) {
        case 'p':
            filter_pex = atoi(optarg);
            if (filter_pex < 1 || filter_pex > 6)
                errx(EXIT_USAGE, "pex must be 1..6 (got '%s')", optarg);
            break;
        case 'w':
            watch = atoi(optarg);
            if (watch < 1)
                errx(EXIT_USAGE, "watch count must be >= 1");
            break;
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
    uint32_t rcwsr30 = rd32(dcfg, OFF_RCWSR30);
    uint32_t proto[4];
    proto[1] = (rcwsr29 >> 16) & 0x1fu;
    proto[2] = (rcwsr29 >> 21) & 0x1fu;
    proto[3] = (rcwsr29 >> 26) & 0x1fu;
    uint32_t pll_pd = rcwsr29 & 0x3fu;
    uint32_t div_pex[4];
    div_pex[1] = (rcwsr30 >> 16) & 3u;
    div_pex[2] = (rcwsr30 >> 18) & 3u;
    div_pex[3] = (rcwsr30 >> 20) & 3u;

    volatile uint32_t *sd_map[4] = { NULL, NULL, NULL, NULL };
    for (int s = 1; s <= 3; s++)
        sd_map[s] = map_phys(fd, sd_base[s], SD_MAP_LEN);

    if (m == MODE_HUMAN) {
        printf("Latched RCW (RCWSR29=0x%08x RCWSR30=0x%08x):\n",
               rcwsr29, rcwsr30);
        printf("  SRDS_PRTCL   S1=%u  S2=%u  S3=%u   SRDS_PLL_PD=0x%02x (PLL1..6 bitmap)\n",
               proto[1], proto[2], proto[3], pll_pd);
        printf("  SRDS_DIV_PEX S1=%u (%s)  S2=%u (%s)  S3=%u (%s)\n",
               div_pex[1], div_pex_str(div_pex[1]),
               div_pex[2], div_pex_str(div_pex[2]),
               div_pex[3], div_pex_str(div_pex[3]));
    }

    int failures = 0;
    int domain = 0;   /* Linux PCI domains follow DT probe order of enabled PEX */
    bool any = false;

    for (size_t i = 0; i < sizeof(pcie_maps) / sizeof(pcie_maps[0]); i++) {
        const struct pcie_map *pm = &pcie_maps[i];

        if (proto[pm->sd] != pm->proto)
            continue;
        int this_domain = domain++;
        if (filter_pex > 0 && pm->pex != filter_pex)
            continue;
        any = true;

        volatile uint32_t *dbi = map_phys(fd, pex_base[pm->pex], PEX_MAP_LEN);
        volatile uint32_t *sd  = sd_map[pm->sd];
        if (!dbi || !sd) {
            failures++;
            continue;
        }

        uint32_t id     = rd32(dbi, 0x00);
        uint32_t class3 = rd32(dbi, 0x08) >> 8;
        uint32_t dbg0   = rd32(dbi, PORT_DEBUG0);
        uint32_t dbg1   = rd32(dbi, PORT_DEBUG1);
        uint32_t ltssm  = dbg0 & 0x3fu;
        bool link_up    = bit(dbg1, 4);
        bool training   = bit(dbg1, 29);

        /* the PLL clocking PCIe on this block: 5 GHz clock-net, not disabled */
        uint32_t frst = rd32(sd, OFF_PLLFRSTCTL), fcr0 = rd32(sd, OFF_PLLFCR0),
                 fcr1 = rd32(sd, OFF_PLLFCR1);
        uint32_t srst = rd32(sd, OFF_PLLSRSTCTL), scr0 = rd32(sd, OFF_PLLSCR0),
                 scr1 = rd32(sd, OFF_PLLSCR1);
        const char *pll_name = "?";
        uint32_t prst = 0, pcr0 = 0, pcr1 = 0;
        if (!bit(frst, PLL_RSTCTL_DIS) && pll_is_pcie(fcr1)) {
            pll_name = "PLLF"; prst = frst; pcr0 = fcr0; pcr1 = fcr1;
        } else if (!bit(srst, PLL_RSTCTL_DIS) && pll_is_pcie(scr1)) {
            pll_name = "PLLS"; prst = srst; pcr0 = scr0; pcr1 = scr1;
        }
        bool pll_lock = prst ? bit(prst, PLL_RSTCTL_LOCK) : false;

        /* the OTHER PLL may be the armed Gen3 clock source */
        const char *g3_name = NULL;
        uint32_t g3cr0 = 0;
        if (pll_name[3] == 'S' && pll_is_gen3_standby(frst, fcr1)) {
            g3_name = "PLLF"; g3cr0 = fcr0;
        } else if (pll_name[3] == 'F' && pll_is_gen3_standby(srst, scr1)) {
            g3_name = "PLLS"; g3cr0 = scr0;
        }

        char cdr[9], lanes[16];
        for (int l = 0; l < pm->nlanes; l++)
            cdr[l] = bit(rd32(sd, OFF_LANE_RRSTCTL +
                                  (uint32_t)(pm->first_lane + l) * LANE_STRIDE),
                         LN_RRSTCTL_CDR_LOCK) ? '1' : '0';
        cdr[pm->nlanes] = '\0';
        if (pm->nlanes == 1)
            snprintf(lanes, sizeof(lanes), "LN%c", 'A' + pm->first_lane);
        else
            snprintf(lanes, sizeof(lanes), "LN%c..LN%c",
                     'A' + pm->first_lane, 'A' + pm->first_lane + pm->nlanes - 1);

        int cap = find_pcie_cap(dbi);
        uint32_t lnkcap = cap > 0 ? rd32(dbi, (uint32_t)cap + 0x0c) : 0;
        uint32_t lnksta = cap > 0 ? rd16(dbi, (uint32_t)cap + 0x12) : 0;

        if (!link_up)
            failures++;

        if (m == MODE_QUIET) {
            printf("PEX%d dom=%04x SD%d %s x%d LTSSM=%s link=%s cap=%s-x%u cur=%s-x%u "
                   "CDR=%s PLL=%s%s refclk=%s divpex=%s -- %s\n",
                   pm->pex, this_domain, pm->sd, lanes, pm->nlanes,
                   ltssm_name(ltssm), link_up ? "UP" : "down",
                   speed_str(lnkcap), (lnkcap >> 4) & 0x3fu,
                   speed_str(lnksta), (lnksta >> 4) & 0x3fu,
                   cdr, pll_name, pll_lock ? "(LOCKED)" : "(NOT locked)",
                   refclk_mhz(pcr0), div_pex_str(div_pex[pm->sd]),
                   ltssm_verdict(ltssm));
            if (g3_name)
                printf("PEX%d gen3=%s-armed (8 GHz clknet preset, parked; hardware switch at Gen3)\n",
                       pm->pex, g3_name);
        } else {
            printf("\nPEX%d @0x%08" PRIx64 "  (likely PCI domain %04x)  --  SD%d %s, x%d  (SRDS_PRTCL_S%d=%u)\n",
                   pm->pex, pex_base[pm->pex], this_domain,
                   pm->sd, lanes, pm->nlanes, pm->sd, pm->proto);
            printf("  id       = %04x:%04x  class=0x%06x%s\n",
                   id & 0xffffu, id >> 16, class3,
                   class3 == 0x060400 ? " (PCI-PCI bridge / root port)" : "");
            printf("  RCW cap  = max %s (SRDS_DIV_PEX_S%d=%u)\n",
                   div_pex_str(div_pex[pm->sd]), pm->sd, div_pex[pm->sd]);
            if (cap > 0)
                printf("  LNKCAP   = %s x%u    LNKSTA = %s x%u%s%s\n",
                       speed_str(lnkcap), (lnkcap >> 4) & 0x3fu,
                       speed_str(lnksta), (lnksta >> 4) & 0x3fu,
                       bit(lnksta, 11) ? "  [LT active]" : "",
                       link_up ? "" : "  (idle value, link not up)");
            printf("  LTSSM    = 0x%02x %-22s  [DEBUG0=0x%08x DEBUG1=0x%08x]\n",
                   ltssm, ltssm_name(ltssm), dbg0, dbg1);
            printf("             link_up=%s  in_training=%s\n",
                   link_up ? "yes" : "no", training ? "yes" : "no");
            printf("  SerDes   = %s %s  refclk=%s on SD%d_%s_REF_CLK_P/N (external)\n",
                   pll_name, pll_lock ? "LOCKED" : "*** NOT LOCKED ***",
                   refclk_mhz(pcr0), pm->sd, pll_name);
            printf("             clknet=%s   lane CDR_LOCK[%s]=%s\n",
                   frate(pcr1), lanes, cdr);
            if (g3_name)
                printf("  gen3     = %s ARMED as standby: refclk=%s, 8 GHz clknet preset, parked by RCW --\n"
                       "             the PCIe hardware switches the lanes to it on the fly at the Gen3\n"
                       "             speed change (RM 26.10.3.1; gen1/gen2 train from %s)\n",
                       g3_name, refclk_mhz(g3cr0), pll_name);
            printf("  verdict  = %s\n", ltssm_verdict(ltssm));
        }

        if (watch > 0) {
            for (int w = 0; w < watch; w++) {
                uint32_t d0 = rd32(dbi, PORT_DEBUG0) & 0x3fu;
                printf("  watch[%02d] PEX%d LTSSM=0x%02x %s\n",
                       w, pm->pex, d0, ltssm_name(d0));
                if (w + 1 < watch)
                    usleep(1000 * 1000);
            }
        }

        munmap((void *)(uintptr_t)dbi, PEX_MAP_LEN);
    }

    if (!any) {
        printf("No PCIe controller is enabled by the latched SerDes protocols "
               "(S1=%u S2=%u S3=%u) known to this tool.\n",
               proto[1], proto[2], proto[3]);
    }

    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
