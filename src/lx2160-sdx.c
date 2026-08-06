/*
 * lx2160-sdx — read LX2160A SerDes block per-lane status from Linux userland.
 *
 * Walks the LX2160A's three SerDes blocks (SD1 / SD2 / SD3) over /dev/mem,
 * decodes PLL lock state, per-lane TX/RX state-machine status, RX CDR lock,
 * and TX/RX equalization coefficients, then prints a human-readable summary.
 *
 * Run (root required, /dev/mem access):
 *   ./lx2160-sdx                       # full report on all three blocks
 *   ./lx2160-sdx --block 2             # only SD2
 *   ./lx2160-sdx --lane 6              # only lane 6 of each block
 *   ./lx2160-sdx --raw                 # raw hex dump of every known register
 *   ./lx2160-sdx --quiet               # one-line "OK / FAIL" summary per lane
 *   ./lx2160-sdx -h                    # full usage
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
#include <sys/stat.h>
#include <unistd.h>

/*
 * In addition to the standard EXIT_SUCCESS / EXIT_FAILURE from <stdlib.h>.
 * Matches the BSD convention used by getopt-driven CLIs.
 */
#define EXIT_USAGE 2

/* SerDes block bases and sizes */
#define SD1_BASE        0x01EA0000UL
#define SD2_BASE        0x01EB0000UL
#define SD3_BASE        0x01EC0000UL
#define SD_BLOCK_MAP_LEN 0x1000UL   /* 4 KiB covers all decoded registers (highest = lane 7 RECR4 @ 0xF54) */

/* Block-level register offsets */
#define OFF_RSTCTL          0x000
#define OFF_PLLFRSTCTL      0x400
#define OFF_PLLFCR0         0x404
#define OFF_PLLFCR1         0x408
#define OFF_PLLFCR3         0x410
#define OFF_PLLFCR4         0x414
#define OFF_PLLFCR9         0x428
#define OFF_PLLFSSCR0       0x42C
#define OFF_PLLFSSCR1       0x430
#define OFF_PLLFSSCR2       0x434
#define OFF_PLLSRSTCTL      0x500
#define OFF_PLLSCR0         0x504
#define OFF_PLLSCR1         0x508
#define OFF_PLLSCR3         0x510
#define OFF_PLLSCR4         0x514
#define OFF_PLLSCR9         0x528
#define OFF_PLLSSSCR0       0x52C
#define OFF_PLLSSSCR1       0x530
#define OFF_PLLSSSCR2       0x534

/* Per-lane register offsets (applied as base + a*0x100)  */
#define OFF_LANE_GCR0       0x800   /* General Control 0 */
#define OFF_LANE_TRSTCTL    0x820   /* TX Reset Control */
#define OFF_LANE_TGCR0      0x824   /* TX General Control 0 */
#define OFF_LANE_TECR0      0x830   /* TX Equalization 0 (FIR taps) */
#define OFF_LANE_TECR1      0x834   /* TX Equalization 1 */
#define OFF_LANE_RRSTCTL    0x840   /* RX Reset Control (CDR_LOCK at bit 12) */
#define OFF_LANE_RGCR0      0x844   /* RX General Control 0 */
#define OFF_LANE_RGCR1      0x848   /* RX General Control 1 */

#define OFF_LANE_RECR2      0x858   /* RX Equalization 2 */
#define OFF_LANE_RECR3      0x85C   /* RX Equalization 3 */
#define OFF_LANE_RECR4      0x860   /* RX Equalization 4 */

#define LANE_STRIDE         0x100
#define NUM_LANES           8
#define LANE_OFF(off, a)    ((off) + (a) * LANE_STRIDE)

/* Bit positions in PLL[F/S]RSTCTL */
#define PLL_RSTCTL_RST_REQ      31
#define PLL_RSTCTL_RST_DONE     30
#define PLL_RSTCTL_RST_ERR      29
#define PLL_RSTCTL_HLT_REQ      27
#define PLL_RSTCTL_STP_REQ      26
#define PLL_RSTCTL_DIS          24
/*
 * LOCK lives in PLL[F/S]RSTCTL (offset 0x400/0x500) bit 23: NOT in
 * PLL[F/S]CR0.
 * bit 23  LOCK ("Indicates PLLn has calibrated and locked"),
 * bits 22-0 Reserved.
 */
#define PLL_RSTCTL_LOCK         23

/*
 * PLL[F/S]CR0 REFCLK_SEL (bits 20:16): reference-clock frequency
 * select, the frequency expected on the external
 * SDn_PLL{F,S}_REF_CLK_P/N pin pair (the SerDes has no internal
 * refclk source).
 * PLL[F/S]CR1 FRATE_SEL (bits 28:24): clock-net / VCO rate select.
 * Both default from the RCW.
 */
#define PLL_CR0_REFCLK_SEL_SHIFT 16
#define PLL_CR0_REFCLK_SEL_MASK  0x1fu
#define PLL_CR1_FRATE_SEL_SHIFT  24
#define PLL_CR1_FRATE_SEL_MASK   0x1fu

/*
 * LNmGCR0: PROTO_SEL bits [7:3] = what the lane is muxed to in
 * silicon (PCIe/SGMII/SATA/10G-ETH/25G-ETH)
 *  IF_WIDTH bits [2:0] = PCS parallel width
 *  PORT_LN0_B bit 16 = 0 on the port's master clock lane
 */
#define LN_GCR0_PROTO_SEL_SHIFT 3
#define LN_GCR0_PROTO_SEL_MASK  0x1fu
#define LN_GCR0_IF_WIDTH_MASK   0x7u
#define LN_GCR0_PORT_LN0_B      16
#define LN_GCR0_PORT_RST_LEFT   17

/* Bit positions in LNaTRSTCTL */
#define LN_TRSTCTL_RST_REQ      31
#define LN_TRSTCTL_RST_DONE     30
#define LN_TRSTCTL_HLT_REQ      27
#define LN_TRSTCTL_STP_REQ      26
#define LN_TRSTCTL_DIS          24

/* Bit positions in LNaRRSTCTL */
#define LN_RRSTCTL_RST_REQ      31
#define LN_RRSTCTL_RST_DONE     30
#define LN_RRSTCTL_HLT_REQ      27
#define LN_RRSTCTL_STP_REQ      26
#define LN_RRSTCTL_DIS          24
#define LN_RRSTCTL_CDR_LOCK     12

/* Output format */
enum mode {
    MODE_HUMAN,   /* full decoded report (default) */
    MODE_QUIET,   /* one summary line per lane: SDx LNa  CDR_LOCK=N  RST_DONE=N */
    MODE_RAW,     /* hex dump of every known register, one per line */
};

/* Per-block descriptive metadata (my test case is RCW = 13 / 7 / 3) 
 * Update if you build a different RCW. The text is informational only;
 * the register reads are universal.
 */
struct block_info {
    int      idx;               /* 1, 2, 3 */
    uint64_t base;              /* CCSR base */
    const char *summary;        /* one-line description per the production RCW */
    const char *lane_labels[NUM_LANES]; /* per-lane protocol assignment per RCW */
};

static const struct block_info blocks[3] = {
    {
        .idx     = 1,
        .base    = SD1_BASE,
        .summary = "SD1 (RCW SRDS_PRTCL_S1=13: 2x 100GE); awaits 100GE-capable carrier",
        .lane_labels = {
            "100GE.1 lane 0", "100GE.1 lane 1", "100GE.1 lane 2", "100GE.1 lane 3",
            "100GE.2 lane 0", "100GE.2 lane 1", "100GE.2 lane 2", "100GE.2 lane 3",
        },
    },
    {
        .idx     = 2,
        .base    = SD2_BASE,
        .summary = "SD2 (RCW SRDS_PRTCL_S2=7: PCIe.3/4 + SGMII + USXGMII)",
        .lane_labels = {
            "PCIe.3 x1 (Gen2)", "SGMII.12", "SGMII.17", "SGMII.18",
            "PCIe.4 x1 (Gen2)", "SGMII.16", "USXGMII13", "USXGMII14",
        },
    },
    {
        .idx     = 3,
        .base    = SD3_BASE,
        .summary = "SD3 (RCW SRDS_PRTCL_S3=3: PCIe.5 x4 + PCIe.6 x4)",
        .lane_labels = {
            "PCIe.5 lane 0", "PCIe.5 lane 1", "PCIe.5 lane 2", "PCIe.5 lane 3",
            "PCIe.6 lane 0", "PCIe.6 lane 1", "PCIe.6 lane 2", "PCIe.6 lane 3",
        },
    },
};

/* Helpers */
static inline uint32_t bit(uint32_t v, unsigned b) {
    return (v >> b) & 1u;
}

static inline uint32_t read_reg(volatile const uint32_t *map, uint32_t off) {
    /* Word offset into the mmap region. Memory barrier is unnecessary for read-only diagnostics. */
    return map[off / 4];
}

static const char *yn(uint32_t b) { return b ? "yes" : "no"; }

static const char *checkmark(bool ok) { return ok ? "[OK]" : "[!!]"; }

/* PLLnCR0[REFCLK_SEL] -> refclk frequency */
static const char *refclk_mhz(uint32_t cr0)
{
    switch ((cr0 >> PLL_CR0_REFCLK_SEL_SHIFT) & PLL_CR0_REFCLK_SEL_MASK) {
    case 0x00: return "100 MHz";
    case 0x01: return "125 MHz";
    case 0x02: return "156.25 MHz";
    case 0x03: return "150 MHz";
    case 0x04: return "161.1328125 MHz";
    default:   return "reserved";
    }
}

/* PLLnCR1[FRATE_SEL] -> clock-net / VCO rate */
static const char *frate(uint32_t cr1)
{
    switch ((cr1 >> PLL_CR1_FRATE_SEL_SHIFT) & PLL_CR1_FRATE_SEL_MASK) {
    case 0x00: return "5 GHz clknet / 20 GHz VCO (PLLS only)";
    case 0x06: return "10.3125 GHz clknet / 20.625 GHz VCO (PLLS only)";
    case 0x10: return "5 GHz clknet / 25 GHz VCO";
    case 0x12: return "6 GHz clknet / 24 GHz VCO";
    case 0x16: return "12.890625 GHz clknet / 25.78125 GHz VCO";
    case 0x17: return "8 GHz clknet / 24 GHz VCO";
    case 0x19: return "8 GHz clknet / 16 GHz VCO (PLLF only)";
    default:   return "reserved";
    }
}

/* LNmGCR0[PROTO_SEL] -> the mode the lane is muxed to in silicon. */
static const char *lane_mode(uint32_t gcr0)
{
    switch ((gcr0 >> LN_GCR0_PROTO_SEL_SHIFT) & LN_GCR0_PROTO_SEL_MASK) {
    case 0x00: return "PCIe";
    case 0x01: return "ethernet-1G (SGMII/1000Base-KX)";
    case 0x02: return "SATA";
    case 0x0a: return "ethernet-10G (XFI/KR/USXGMII/40G)";
    case 0x1a: return "ethernet-25G (25G/50G/100G lanes)";
    default:   return "reserved";
    }
}

/* Short mode token for the one-line quiet format. */
static const char *lane_mode_short(uint32_t gcr0)
{
    switch ((gcr0 >> LN_GCR0_PROTO_SEL_SHIFT) & LN_GCR0_PROTO_SEL_MASK) {
    case 0x00: return "PCIe";
    case 0x01: return "ETH-1G";
    case 0x02: return "SATA";
    case 0x0a: return "ETH-10G";
    case 0x1a: return "ETH-25G";
    default:   return "rsvd";
    }
}

/* Nominal per-lane line rate for the silicon mode (baud, not payload). */
static const char *lane_rate(uint32_t gcr0)
{
    switch ((gcr0 >> LN_GCR0_PROTO_SEL_SHIFT) & LN_GCR0_PROTO_SEL_MASK) {
    case 0x00: return "2.5/5/8 GT/s (per LTSSM)";
    case 0x01: return "1.25 Gbd";
    case 0x02: return "1.5/3/6 Gbd";
    case 0x0a: return "10.3125 Gbd";
    case 0x1a: return "25.78125 Gbd";
    default:   return "?";
    }
}

/* LNmGCR0[IF_WIDTH] -> PCS parallel interface width. */
static const char *if_width(uint32_t gcr0)
{
    switch (gcr0 & LN_GCR0_IF_WIDTH_MASK) {
    case 0: return "10-bit";
    case 1: return "16-bit";
    case 2: return "20-bit";
    case 3: return "32-bit";
    case 4: return "40-bit";
    default: return "?";
    }
}

/* Print a PLL block (PLLF or PLLS) */
static void
print_pll(const volatile uint32_t *map,
          int blk_idx,
          const char *name,
          uint32_t off_rstctl,
          uint32_t off_cr0,
          uint32_t off_cr1)
{
    uint32_t rst = read_reg(map, off_rstctl);
    uint32_t cr0 = read_reg(map, off_cr0);
    uint32_t cr1 = read_reg(map, off_cr1);

    bool dis      = bit(rst, PLL_RSTCTL_DIS);
    bool rst_done = bit(rst, PLL_RSTCTL_RST_DONE);
    bool rst_err  = bit(rst, PLL_RSTCTL_RST_ERR);
    bool lock     = bit(rst, PLL_RSTCTL_LOCK);

    bool ok = dis ? true : (rst_done && lock && !rst_err);

    printf("  PLL%-2s  %s  RSTCTL=0x%08x  CR0=0x%08x  CR1=0x%08x\n",
           name, checkmark(ok), rst, cr0, cr1);
    printf("        DIS=%s  RST_DONE=%s  RST_ERR=%s  LOCK=%s",
           yn(dis), yn(rst_done), yn(rst_err), yn(lock));
    /*
     * A PLL parked by the RCW but pre-configured with an 8 GHz
     * clock-net is not dead weight: it is the documented PCIe Gen3
     * standby configuration; gen1/gen2 train from the 5 GHz PLL
     * and the PCIe hardware hands the lanes* over to this PLL on
     * the fly at the Gen3 speed switch (the protocol tables' "PLL
     * mapping after Gen3 speed switch" column).
     */
    uint32_t fr = (cr1 >> PLL_CR1_FRATE_SEL_SHIFT) & PLL_CR1_FRATE_SEL_MASK;
    bool gen3_standby = dis && (fr == 0x17 || fr == 0x19);

    if (gen3_standby)   printf("  --> parked by RCW: PCIe Gen3 STANDBY (8 GHz clknet preset; hardware wakes it at the Gen3 speed switch)");
    else if (dis)       printf("  --> powered down (RCW SRDS_PLL_PD_*)");
    else if (rst_err)   printf("  --> *** PLL reset error ***");
    else if (!rst_done) printf("  --> still in reset");
    else if (!lock)     printf("  --> *** PLL not locked ***");
    else                printf("  --> healthy");
    printf("\n");
    /*
     * Clock information: the refclk arrives on the external
     * SDn_PLL{F,S}_REF_CLK_P/N ball pair (there is no on-die refclk
     * source); who drives that pair is a per board topic.
     * REFCLK_SEL/FRATE_SEL are what the RCW programmed into the PLL.
     */
    printf("        refclk = %s on SD%d_PLL%s_REF_CLK_P/N (external)   clknet = %s\n",
           refclk_mhz(cr0), blk_idx, name, frate(cr1));
}

/*  Print one lane  */

static int
print_lane(const struct block_info *blk,
           const volatile uint32_t *map,
           int a,
           enum mode m)
{
    uint32_t gcr0   = read_reg(map, LANE_OFF(OFF_LANE_GCR0,    a));
    uint32_t trst   = read_reg(map, LANE_OFF(OFF_LANE_TRSTCTL, a));
    uint32_t tgcr0  = read_reg(map, LANE_OFF(OFF_LANE_TGCR0,   a));
    uint32_t tecr0  = read_reg(map, LANE_OFF(OFF_LANE_TECR0,   a));
    uint32_t tecr1  = read_reg(map, LANE_OFF(OFF_LANE_TECR1,   a));
    uint32_t rrst   = read_reg(map, LANE_OFF(OFF_LANE_RRSTCTL, a));
    uint32_t rgcr0  = read_reg(map, LANE_OFF(OFF_LANE_RGCR0,   a));
    uint32_t rgcr1  = read_reg(map, LANE_OFF(OFF_LANE_RGCR1,   a));
    uint32_t recr2  = read_reg(map, LANE_OFF(OFF_LANE_RECR2,   a));
    uint32_t recr3  = read_reg(map, LANE_OFF(OFF_LANE_RECR3,   a));
    uint32_t recr4  = read_reg(map, LANE_OFF(OFF_LANE_RECR4,   a));

    bool tx_dis      = bit(trst, LN_TRSTCTL_DIS);
    bool tx_rst_done = bit(trst, LN_TRSTCTL_RST_DONE);
    bool rx_dis      = bit(rrst, LN_RRSTCTL_DIS);
    bool rx_rst_done = bit(rrst, LN_RRSTCTL_RST_DONE);
    bool cdr_lock    = bit(rrst, LN_RRSTCTL_CDR_LOCK);

    /* Healthy means: enabled, both state machines done, CDR locked.
     * If both DIS bits are set the lane is intentionally off, then count as OK.
     */
    bool fully_off = tx_dis && rx_dis;
    bool ok = fully_off ? true : (tx_rst_done && rx_rst_done && cdr_lock);

    if (m == MODE_QUIET) {
        printf("SD%d LN%c  %s  CDR_LOCK=%u  TX_DIS=%u  RX_DIS=%u  TX_RST_DONE=%u  RX_RST_DONE=%u  mode=%s rate=%s   %s\n",
               blk->idx, 'A' + a, checkmark(ok),
               cdr_lock, tx_dis, rx_dis, tx_rst_done, rx_rst_done,
               fully_off ? "disabled" : lane_mode_short(gcr0),
               fully_off ? "-" : lane_rate(gcr0),
               blk->lane_labels[a]);
        return ok ? 0 : 1;
    }

    if (m == MODE_RAW) {
        printf("SD%d LN%c (lane %d, %s):\n", blk->idx, 'A' + a, a, blk->lane_labels[a]);
        printf("  +0x%03x GCR0    = 0x%08x\n", LANE_OFF(OFF_LANE_GCR0,    a), gcr0);
        printf("  +0x%03x TRSTCTL = 0x%08x\n", LANE_OFF(OFF_LANE_TRSTCTL, a), trst);
        printf("  +0x%03x TGCR0   = 0x%08x\n", LANE_OFF(OFF_LANE_TGCR0,   a), tgcr0);
        printf("  +0x%03x TECR0   = 0x%08x\n", LANE_OFF(OFF_LANE_TECR0,   a), tecr0);
        printf("  +0x%03x TECR1   = 0x%08x\n", LANE_OFF(OFF_LANE_TECR1,   a), tecr1);
        printf("  +0x%03x RRSTCTL = 0x%08x\n", LANE_OFF(OFF_LANE_RRSTCTL, a), rrst);
        printf("  +0x%03x RGCR0   = 0x%08x\n", LANE_OFF(OFF_LANE_RGCR0,   a), rgcr0);
        printf("  +0x%03x RGCR1   = 0x%08x\n", LANE_OFF(OFF_LANE_RGCR1,   a), rgcr1);
        printf("  +0x%03x RECR2   = 0x%08x\n", LANE_OFF(OFF_LANE_RECR2,   a), recr2);
        printf("  +0x%03x RECR3   = 0x%08x\n", LANE_OFF(OFF_LANE_RECR3,   a), recr3);
        printf("  +0x%03x RECR4   = 0x%08x\n", LANE_OFF(OFF_LANE_RECR4,   a), recr4);
        return ok ? 0 : 1;
    }

    /* Default human-readable view */
    printf("  Lane %d (LN%c)  %s  --  %s\n",
           a, 'A' + a, checkmark(ok), blk->lane_labels[a]);
    if (fully_off)
        printf("    mode    = disabled (TX+RX DIS)\n");
    else
        printf("    mode    = %s   IF_WIDTH=%s   rate=%s%s\n",
               lane_mode(gcr0), if_width(gcr0), lane_rate(gcr0),
               bit(gcr0, LN_GCR0_PORT_LN0_B) == 0 ? "   [port master lane]" : "");
    printf("    GCR0    = 0x%08x\n", gcr0);
    printf("    TRSTCTL = 0x%08x   RST_DONE=%u  DIS=%u\n",
           trst, tx_rst_done, tx_dis);
    printf("    RRSTCTL = 0x%08x   RST_DONE=%u  DIS=%u  CDR_LOCK=%u   ",
           rrst, rx_rst_done, rx_dis, cdr_lock);
    if (rx_dis)             printf("(RX disabled)\n");
    else if (!rx_rst_done)  printf("*** RX still in reset ***\n");
    else if (!cdr_lock)     printf("*** no CDR lock — RX not seeing valid signal ***\n");
    else                    printf("--> RX healthy\n");
    printf("    TX equ  = TECR0=0x%08x  TECR1=0x%08x   (FIR taps)\n",
           tecr0, tecr1);
    printf("    RX equ  = RECR2=0x%08x  RECR3=0x%08x  RECR4=0x%08x\n",
           recr2, recr3, recr4);
    printf("    RX gen  = RGCR0=0x%08x  RGCR1=0x%08x\n",
           rgcr0, rgcr1);

    return ok ? 0 : 1;
}

/*
 * Multi-lane port grouping
 *
 * LNmGCR0[PORT_LN0_B]=0 marks the master clock lane of a port (it is
 * also 0 on single-lane ports and on disabled lanes), the other lanes
 * of a multi-lane port carry PORT_LN0_B=1 and the same PROTO_SEL.
 * Scan lanes A..H: each PORT_LN0_B=0 lane opens a group, and the
 * following PORT_LN0_B=1 lanes with the same PROTO_SEL join it (the
 * RCW protocols on this SoC place the master left-most,
 * PORT_RST_LEFT=1: e.g. PCIe x4 = master LNA + members LNB..LND).
 *
 * Per group: lane span, silicon mode, xN width, IF_WIDTH, per-lane
 * CDR bitmap, and a settings-consistency verdict (all member lanes
 * must share PROTO_SEL + IF_WIDTH).
 */
static void
print_ports(const struct block_info *blk,
            const volatile uint32_t *map,
            enum mode m)
{
    uint32_t gcr0[NUM_LANES], trst[NUM_LANES], rrst[NUM_LANES];

    for (int i = 0; i < NUM_LANES; i++) {
        gcr0[i] = read_reg(map, LANE_OFF(OFF_LANE_GCR0,    i));
        trst[i] = read_reg(map, LANE_OFF(OFF_LANE_TRSTCTL, i));
        rrst[i] = read_reg(map, LANE_OFF(OFF_LANE_RRSTCTL, i));
    }

    if (m == MODE_HUMAN)
        printf("\nPorts (grouping from LNmGCR0[PORT_LN0_B]):\n");

    int a = 0;
    while (a < NUM_LANES) {
        int first = a;
        int n = 1;

        /*
         * Direction-aware walk: with PORT_RST_LEFT=1
         * the master (PORT_LN0_B=0) is the LEFT-most lane and members
         * follow it (PCIe x4); with PORT_RST_LEFT=0 the master is the
         * RIGHT-most lane and members precede it (the 100GE CAUI4
         * groups use this orientation).
         */
        bool left = bit(gcr0[first], LN_GCR0_PORT_RST_LEFT) == 1;
        if (left) {
            while (first + n < NUM_LANES &&
                   bit(gcr0[first + n], LN_GCR0_PORT_LN0_B) == 1 &&
                   (((gcr0[first + n] ^ gcr0[first])
                     >> LN_GCR0_PROTO_SEL_SHIFT) & LN_GCR0_PROTO_SEL_MASK) == 0)
                n++;
        } else {
            while (bit(gcr0[first + n - 1], LN_GCR0_PORT_LN0_B) == 1 &&
                   first + n < NUM_LANES &&
                   (((gcr0[first + n] ^ gcr0[first])
                     >> LN_GCR0_PROTO_SEL_SHIFT) & LN_GCR0_PROTO_SEL_MASK) == 0)
                n++;
        }
        a = first + n;
        int master = left ? first : first + n - 1;

        bool disabled = bit(trst[first], LN_TRSTCTL_DIS) &&
                        bit(rrst[first], LN_RRSTCTL_DIS);

        bool consistent = true;
        char cdr[NUM_LANES + 1];
        for (int i = 0; i < n; i++) {
            if (((gcr0[first + i] ^ gcr0[first]) & 0xffu) != 0)
                consistent = false;   /* PROTO_SEL or IF_WIDTH differs */
            cdr[i] = bit(rrst[first + i], LN_RRSTCTL_CDR_LOCK) ? '1' : '0';
        }
        cdr[n] = '\0';

        if (m == MODE_QUIET) {
            if (n > 1)   /* per-lane quiet lines already cover x1 */
                printf("SD%d PORT LN%c..LN%c x%d mode=%s IF=%s rate=%s/lane CDR=%s%s\n",
                       blk->idx, 'A' + first, 'A' + first + n - 1, n,
                       lane_mode_short(gcr0[first]), if_width(gcr0[first]),
                       lane_rate(gcr0[first]), cdr,
                       consistent ? "" : " *** SETTINGS MISMATCH ***");
            continue;
        }

        if (disabled && n == 1) {
            printf("  LN%c        x1  disabled\n", 'A' + first);
        } else if (n == 1) {
            printf("  LN%c        x1  %-34s IF=%s  rate=%s  CDR=%s\n",
                   'A' + first, lane_mode(gcr0[first]),
                   if_width(gcr0[first]), lane_rate(gcr0[first]), cdr);
        } else {
            printf("  LN%c..LN%c   x%d  %-34s IF=%s  rate=%s/lane  master=LN%c  CDR=%s  %s\n",
                   'A' + first, 'A' + first + n - 1, n,
                   lane_mode(gcr0[first]), if_width(gcr0[first]),
                   lane_rate(gcr0[first]), 'A' + master, cdr,
                   consistent ? "settings-consistent"
                              : "*** SETTINGS MISMATCH across lanes ***");
        }
    }
}

static int
print_block(const struct block_info *blk,
            const volatile uint32_t *map,
            int filter_lane,
            enum mode m)
{
    int failures = 0;

    if (m != MODE_QUIET) {
        printf("\nSerDes block SD%d  (CCSR base 0x%08" PRIx64 ")\n",
               blk->idx, blk->base);
        printf("  %s\n", blk->summary);

        printf("\nBlock-level:\n");
        printf("  RSTCTL = 0x%08x\n", read_reg(map, OFF_RSTCTL));

        printf("\nPLL state:\n");
        print_pll(map, blk->idx, "F", OFF_PLLFRSTCTL, OFF_PLLFCR0, OFF_PLLFCR1);
        print_pll(map, blk->idx, "S", OFF_PLLSRSTCTL, OFF_PLLSCR0, OFF_PLLSCR1);

        if (m == MODE_HUMAN && filter_lane < 0)
            print_ports(blk, map, m);

        printf("\nPer-lane state:\n");
    }

    for (int a = 0; a < NUM_LANES; a++) {
        if (filter_lane >= 0 && filter_lane != a)
            continue;
        failures += print_lane(blk, map, a, m);
        if (m == MODE_HUMAN)
            printf("\n");
    }

    if (m == MODE_QUIET && filter_lane < 0)
        print_ports(blk, map, m);

    return failures;
}

/*  Map one block via /dev/mem  */
static volatile uint32_t *
mmap_block(int fd, uint64_t base)
{
    void *p = mmap(NULL, SD_BLOCK_MAP_LEN, PROT_READ, MAP_SHARED, fd, (off_t)base);
    if (p == MAP_FAILED) {
        warn("mmap(0x%" PRIx64 ", %lu)", base, SD_BLOCK_MAP_LEN);
        return NULL;
    }
    return (volatile uint32_t *)p;
}

static void
usage(const char *argv0)
{
    fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "Options:\n"
        "  -b, --block N        Only report SerDes block N (1, 2, or 3)\n"
        "  -l, --lane N         Only report lane N (0..7) of each reported block\n"
        "  -q, --quiet          One-line summary per lane (suitable for parsing)\n"
        "  -r, --raw            Raw hex dump of every known register, one per line\n"
        "  -h, --help           Show this help and exit\n"
        "\n"
        "Exit codes:\n"
        "  0  EXIT_SUCCESS — all reported lanes healthy (or intentionally\n"
        "                    disabled per the loaded RCW)\n"
        "  1  EXIT_FAILURE — at least one reported lane unhealthy\n"
        "                    (RX state machine not done, or CDR_LOCK=0),\n"
        "                    or a runtime error (e.g. /dev/mem inaccessible)\n"
        "  2  EXIT_USAGE   — argument error (bad block/lane index)\n"
        "\n",
        argv0);
}

int
main(int argc, char **argv)
{
    int filter_block = -1;
    int filter_lane  = -1;
    enum mode m = MODE_HUMAN;

    static const struct option longopts[] = {
        { "block", required_argument, NULL, 'b' },
        { "lane",  required_argument, NULL, 'l' },
        { "quiet", no_argument,       NULL, 'q' },
        { "raw",   no_argument,       NULL, 'r' },
        { "help",  no_argument,       NULL, 'h' },
        { NULL, 0, NULL, 0 },
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "b:l:qrh", longopts, NULL)) != -1) {
        switch (opt) {
        case 'b':
            filter_block = atoi(optarg);
            if (filter_block < 1 || filter_block > 3)
                errx(EXIT_USAGE, "block must be 1, 2, or 3 (got '%s')", optarg);
            break;
        case 'l':
            filter_lane = atoi(optarg);
            if (filter_lane < 0 || filter_lane >= NUM_LANES)
                errx(EXIT_USAGE, "lane must be 0..%d (got '%s')",
                     NUM_LANES - 1, optarg);
            break;
        case 'q': m = MODE_QUIET; break;
        case 'r': m = MODE_RAW;   break;
        case 'h': usage(argv[0]); return EXIT_SUCCESS;
        default:  usage(argv[0]); return EXIT_USAGE;
        }
    }

    int fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (fd < 0)
        err(EXIT_FAILURE, "open /dev/mem (need root, or CAP_SYS_RAWIO)");

    int total_failures = 0;
    int blocks_reported = 0;

    for (size_t i = 0; i < sizeof(blocks) / sizeof(blocks[0]); i++) {
        const struct block_info *blk = &blocks[i];
        if (filter_block != -1 && filter_block != blk->idx)
            continue;

        volatile uint32_t *map = mmap_block(fd, blk->base);
        if (!map) {
            /* mmap_block() already warned via warn(); count as a failure
             * but keep going so the user sees what is reachable. */
            total_failures++;
            continue;
        }

        total_failures += print_block(blk, map, filter_lane, m);
        blocks_reported++;

        munmap((void *)map, SD_BLOCK_MAP_LEN);
    }

    close(fd);

    if (m == MODE_HUMAN && blocks_reported > 0) {
        if (total_failures == 0)
            printf("\nSummary: all reported lanes healthy.\n");
        else
            printf("\nSummary: %d lane%s unhealthy (RX not ready, or CDR_LOCK=0).\n",
                   total_failures, total_failures == 1 ? "" : "s");
    }

    return total_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
