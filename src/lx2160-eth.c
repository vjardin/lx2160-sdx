/*
 * lx2160-eth — LX2160A Ethernet-mode SerDes lane reader (Linux userland).
 *
 * WRIOP MAC numbers printed for known protocols are the DPAA2 dpmac
 * ids (fixed lane->MAC binding in silicon). RGMII Ethernet (EC1/EC2
 * pin-mux MACs 17/18) is not SerDes and out of scope here.
 * Note that SGMII 17 and 18 SerDes are exclusive with RGMII 17 and 18.
 *
 * Run (root required, /dev/mem access):
 *   ./lx2160-eth                 # all Ethernet lanes of SD1..SD3
 *   ./lx2160-eth --block 2       # only SD2's Ethernet lanes
 *   ./lx2160-eth --quiet         # one line per lane
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
#define OFF_LANE_TECR0     0x830
#define OFF_LANE_TECR1     0x834
#define OFF_LANE_RRSTCTL   0x840
#define OFF_LANE_RECR3     0x85C   /* [31] EQ_SNAP_START, [30] EQ_SNAP_DN */
#define OFF_LANE_RECR4     0x860   /* [15:12] EQ_BIN_DATA_SEL, [8:0] EQ_BIN_DATA */
#define OFF_LANE_TCSR0     0x8A0   /* [31] SD_STAT_OBS_EN */
#define OFF_LANE_TCSR1     0x8A4   /* [27:24] SD_TST_SEL, [8:0] RX_DATA_SM */
#define OFF_LANE_TCSR2     0x8A8   /* [15:0] RX_TST_DAT */
#define LANE_STRIDE        0x100
#define NUM_LANES          8

#define RECR3_EQ_SNAP_START (1u << 31)
#define RECR3_EQ_SNAP_DN    (1u << 30)
#define TCSR0_SD_STAT_OBS_EN (1u << 31)

/* TCSR1[27:24] SD_TST_SEL lane test modes */
#define TST_APP       0x0
#define TST_SAMPLER   0x2
#define TST_WALKTAP   0x3
#define TST_JITSCOPE  0x4
#define TST_BISTCHK   0x5
#define TST_PIJITTER  0x6

#define PLL_RSTCTL_RST_DONE 30
#define PLL_RSTCTL_DIS      24
#define PLL_RSTCTL_LOCK     23
#define LN_RST_DONE         30
#define LN_DIS              24
#define LN_RRSTCTL_CDR_LOCK 12
#define LN_GCR0_PORT_LN0_B  16
#define LN_GCR0_PORT_RST_LEFT 17

/* LNmGCR0[PROTO_SEL] values */
#define PROTO_PCIE   0x00
#define PROTO_SGMII  0x01
#define PROTO_SATA   0x02
#define PROTO_10G    0x0a   /* XFI/SFI/10GBase-R, 10GBase-KR, 10G-SXGMII, 40G */
#define PROTO_25G    0x1a   /* 25GBase-R/KR, 50G, 100G (CAUI4) lanes */

/*
 * Ethernet lane expectations per known (SerDes block, SRDS_PRTCL)
 * pairs. label/mac are informational; the silicon PROTO_SEL is always
 * read back and cross-checked. Unknown protocols fall back to a pure
 * silicon-decode (any lane whose PROTO_SEL is an Ethernet class).
 */
struct eth_lane {
    int      lane;
    uint32_t expect;        /* expected PROTO_SEL class */
    const char *label;      /* per the protocol table */
    const char *mac;        /* WRIOP MAC / dpmac id */
};

struct eth_map {
    int      sd;
    uint32_t proto;
    int      nlanes;
    struct eth_lane lanes[NUM_LANES];
};

static const struct eth_map eth_maps[] = {
    { 1, 13, 8, {
        { 0, PROTO_25G, "100GE.1 lane 0", "MAC1 (CAUI4)" },
        { 1, PROTO_25G, "100GE.1 lane 1", "MAC1 (CAUI4)" },
        { 2, PROTO_25G, "100GE.1 lane 2", "MAC1 (CAUI4)" },
        { 3, PROTO_25G, "100GE.1 lane 3", "MAC1 (CAUI4)" },
        { 4, PROTO_25G, "100GE.2 lane 0", "MAC2 (CAUI4)" },
        { 5, PROTO_25G, "100GE.2 lane 1", "MAC2 (CAUI4)" },
        { 6, PROTO_25G, "100GE.2 lane 2", "MAC2 (CAUI4)" },
        { 7, PROTO_25G, "100GE.2 lane 3", "MAC2 (CAUI4)" },
    } },
    { 2, 7, 6, {
        { 1, PROTO_SGMII, "SGMII.12",   "MAC12" },
        { 2, PROTO_SGMII, "SGMII.17",   "MAC17" },
        { 3, PROTO_SGMII, "SGMII.18",   "MAC18" },
        { 5, PROTO_SGMII, "SGMII.16",   "MAC16" },
        { 6, PROTO_10G,   "USXGMII.13", "MAC13" },
        { 7, PROTO_10G,   "USXGMII.14", "MAC14" },
    } },
};

enum mode { MODE_HUMAN, MODE_QUIET };

static inline uint32_t rd32(const volatile uint32_t *m, uint32_t off)
{
    return m[off / 4];
}

static inline uint32_t bit(uint32_t v, unsigned b) { return (v >> b) & 1u; }

static inline uint32_t lane_reg(const volatile uint32_t *sd, uint32_t off, int lane)
{
    return rd32(sd, off + (uint32_t)lane * LANE_STRIDE);
}

static volatile uint32_t *map_phys_prot(int fd, uint64_t base, size_t len, int prot)
{
    void *p = mmap(NULL, len, prot, MAP_SHARED, fd, (off_t)base);
    if (p == MAP_FAILED) {
        warn("mmap(0x%" PRIx64 ")", base);
        return NULL;
    }
    return (volatile uint32_t *)p;
}

static volatile uint32_t *map_phys(int fd, uint64_t base, size_t len)
{
    return map_phys_prot(fd, base, len, PROT_READ);
}

static inline void wr32(volatile uint32_t *m, uint32_t off, uint32_t val)
{
    m[off / 4] = val;
}

static inline void lane_wr(volatile uint32_t *sd, uint32_t off, int lane,
                           uint32_t val)
{
    wr32(sd, off + (uint32_t)lane * LANE_STRIDE, val);
}

static uint32_t proto_sel(uint32_t gcr0) { return (gcr0 >> 3) & 0x1fu; }

static bool is_eth(uint32_t ps)
{
    return ps == PROTO_SGMII || ps == PROTO_10G || ps == PROTO_25G;
}

static const char *mode_str(uint32_t ps)
{
    switch (ps) {
    case PROTO_PCIE:  return "PCIe";
    case PROTO_SGMII: return "ethernet-1G (SGMII/1000Base-KX)";
    case PROTO_SATA:  return "SATA";
    case PROTO_10G:   return "ethernet-10G (XFI/KR/USXGMII)";
    case PROTO_25G:   return "ethernet-25G (25G/50G/100G lane)";
    default:          return "reserved";
    }
}

static const char *mode_short(uint32_t ps)
{
    switch (ps) {
    case PROTO_SGMII: return "ETH-1G";
    case PROTO_10G:   return "ETH-10G";
    case PROTO_25G:   return "ETH-25G";
    default:          return "non-eth";
    }
}

static const char *rate_str(uint32_t ps)
{
    switch (ps) {
    case PROTO_SGMII: return "1.25 Gbd";
    case PROTO_10G:   return "10.3125 Gbd";
    case PROTO_25G:   return "25.78125 Gbd";
    default:          return "?";
    }
}

static const char *if_width(uint32_t gcr0)
{
    switch (gcr0 & 0x7u) {
    case 0: return "10-bit";
    case 1: return "16-bit";
    case 2: return "20-bit";
    case 3: return "32-bit";
    case 4: return "40-bit";
    default: return "?";
    }
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

/*
 * Does this PLL's clock-net serve the given Ethernet mode?
 *   SGMII 1.25 Gbd    <- 5 GHz clock-net (FRATE 0x00 or 0x10)
 *   10.3125 Gbd class <- 10.3125 GHz clock-net (FRATE 0x06)
 *   25.78125 Gbd class<- 12.890625 GHz clock-net (FRATE 0x16)
 */
static bool pll_serves(uint32_t cr1, uint32_t ps)
{
    uint32_t f = (cr1 >> 24) & 0x1fu;
    switch (ps) {
    case PROTO_SGMII: return f == 0x00 || f == 0x10;
    case PROTO_10G:   return f == 0x06;
    case PROTO_25G:   return f == 0x16;
    default:          return false;
    }
}

struct pll_view {
    const char *name;     /* "PLLF" / "PLLS" */
    uint32_t rst, cr0, cr1;
    bool dis, lock;
};

static void load_pll(const volatile uint32_t *sd, struct pll_view *f,
                     struct pll_view *s)
{
    f->name = "PLLF";
    f->rst  = rd32(sd, OFF_PLLFRSTCTL);
    f->cr0  = rd32(sd, OFF_PLLFCR0);
    f->cr1  = rd32(sd, OFF_PLLFCR1);
    f->dis  = bit(f->rst, PLL_RSTCTL_DIS);
    f->lock = bit(f->rst, PLL_RSTCTL_LOCK);
    s->name = "PLLS";
    s->rst  = rd32(sd, OFF_PLLSRSTCTL);
    s->cr0  = rd32(sd, OFF_PLLSCR0);
    s->cr1  = rd32(sd, OFF_PLLSCR1);
    s->dis  = bit(s->rst, PLL_RSTCTL_DIS);
    s->lock = bit(s->rst, PLL_RSTCTL_LOCK);
}

/* Pick the enabled PLL whose clock-net serves the lane's mode. */
static const struct pll_view *pick_pll(const struct pll_view *f,
                                       const struct pll_view *s, uint32_t ps)
{
    if (!f->dis && pll_serves(f->cr1, ps))
        return f;
    if (!s->dis && pll_serves(s->cr1, ps))
        return s;
    return NULL;
}

/* One Ethernet lane report. Returns 0 healthy / 1 unhealthy. */
static int
print_eth_lane(int sd_idx, const volatile uint32_t *sd, int lane,
               uint32_t expect, const char *label, const char *mac,
               const struct pll_view *pf, const struct pll_view *ps_,
               enum mode m)
{
    uint32_t gcr0 = lane_reg(sd, OFF_LANE_GCR0,    lane);
    uint32_t trst = lane_reg(sd, OFF_LANE_TRSTCTL, lane);
    uint32_t rrst = lane_reg(sd, OFF_LANE_RRSTCTL, lane);
    uint32_t tecr0 = lane_reg(sd, OFF_LANE_TECR0,  lane);
    uint32_t tecr1 = lane_reg(sd, OFF_LANE_TECR1,  lane);

    uint32_t ps   = proto_sel(gcr0);
    bool tx_dis   = bit(trst, LN_DIS);
    bool rx_dis   = bit(rrst, LN_DIS);
    bool cdr      = bit(rrst, LN_RRSTCTL_CDR_LOCK);
    bool tx_done  = bit(trst, LN_RST_DONE);
    bool rx_done  = bit(rrst, LN_RST_DONE);
    bool off      = tx_dis && rx_dis;
    bool match    = (expect == 0xff) || (ps == expect);

    const struct pll_view *pll = pick_pll(pf, ps_, ps);
    bool ok = off ? true : (tx_done && rx_done && cdr && match &&
                            pll && pll->lock);

    if (m == MODE_QUIET) {
        printf("SD%d LN%c %s mode=%s rate=%s CDR=%u PLL=%s%s refclk=%s %s%s\n",
               sd_idx, 'A' + lane, ok ? "[OK]" : "[!!]",
               off ? "disabled" : mode_short(ps), rate_str(ps), cdr,
               pll ? pll->name : "?",
               pll ? (pll->lock ? "(LOCKED)" : "(NOT-locked)") : "",
               pll ? refclk_mhz(pll->cr0) : "?",
               label, match ? "" : " *** MODE != protocol table ***");
        return ok ? 0 : 1;
    }

    printf("  LN%c  %s  %s  (%s)\n", 'A' + lane, ok ? "[OK]" : "[!!]",
           label, mac);
    if (off) {
        printf("       mode = disabled (TX+RX DIS)\n");
        return 0;
    }
    printf("       mode    = %s   IF_WIDTH=%s   line rate=%s%s\n",
           mode_str(ps), if_width(gcr0), rate_str(ps),
           bit(gcr0, LN_GCR0_PORT_LN0_B) == 0 ? "   [port master lane]" : "");
    if (!match)
        printf("       *** silicon PROTO_SEL (%s) differs from the protocol-table "
               "expectation (%s) ***\n", mode_str(ps), mode_str(expect));
    if (pll)
        printf("       clock   = %s %s, refclk=%s on SD%d_%s_REF_CLK_P/N (external), %s\n",
               pll->name, pll->lock ? "LOCKED" : "*** NOT LOCKED ***",
               refclk_mhz(pll->cr0), sd_idx,
               pll->name, frate(pll->cr1));
    else
        printf("       clock   = *** no enabled PLL provides this mode's clock-net ***\n");
    printf("       state   = TX_RST_DONE=%u RX_RST_DONE=%u CDR_LOCK=%u  %s\n",
           tx_done, rx_done, cdr,
           cdr ? "(link partner transmitting)"
               : "*** no CDR lock -- no signal from link partner ***");
    printf("       TX equ  = TECR0=0x%08x TECR1=0x%08x\n", tecr0, tecr1);
    return ok ? 0 : 1;
}

/* Multi-lane grouping header, same PORT_LN0_B walk as lx2160-sdx. */
static void
print_groups(int sd_idx, const volatile uint32_t *sd, enum mode m)
{
    uint32_t gcr0[NUM_LANES], rrst[NUM_LANES];

    for (int i = 0; i < NUM_LANES; i++) {
        gcr0[i] = lane_reg(sd, OFF_LANE_GCR0,    i);
        rrst[i] = lane_reg(sd, OFF_LANE_RRSTCTL, i);
    }

    int a = 0;
    while (a < NUM_LANES) {
        int first = a, n = 1;
        /*
         * Direction-aware walk: PORT_RST_LEFT=1 -> master left-most,
         * members follow; PORT_RST_LEFT=0 -> master right-most,
         * members precede (100GE CAUI4 groups use this orientation).
         */
        bool left = bit(gcr0[first], LN_GCR0_PORT_RST_LEFT) == 1;
        if (left) {
            while (first + n < NUM_LANES &&
                   bit(gcr0[first + n], LN_GCR0_PORT_LN0_B) == 1 &&
                   proto_sel(gcr0[first + n]) == proto_sel(gcr0[first]))
                n++;
        } else {
            while (bit(gcr0[first + n - 1], LN_GCR0_PORT_LN0_B) == 1 &&
                   first + n < NUM_LANES &&
                   proto_sel(gcr0[first + n]) == proto_sel(gcr0[first]))
                n++;
        }
        a = first + n;
        int master = left ? first : first + n - 1;

        if (n < 2 || !is_eth(proto_sel(gcr0[first])))
            continue;

        bool consistent = true;
        char cdr[NUM_LANES + 1];
        for (int i = 0; i < n; i++) {
            if (((gcr0[first + i] ^ gcr0[first]) & 0xffu) != 0)
                consistent = false;
            cdr[i] = bit(rrst[first + i], LN_RRSTCTL_CDR_LOCK) ? '1' : '0';
        }
        cdr[n] = '\0';

        if (m == MODE_QUIET)
            printf("SD%d GROUP LN%c..LN%c x%d %s %s/lane CDR=%s%s\n",
                   sd_idx, 'A' + first, 'A' + first + n - 1, n,
                   mode_short(proto_sel(gcr0[first])),
                   rate_str(proto_sel(gcr0[first])), cdr,
                   consistent ? "" : " *** SETTINGS MISMATCH ***");
        else
            printf("  group LN%c..LN%c  x%d  %s  %s/lane  master=LN%c  CDR=%s  %s\n",
                   'A' + first, 'A' + first + n - 1, n,
                   mode_str(proto_sel(gcr0[first])),
                   rate_str(proto_sel(gcr0[first])), 'A' + master, cdr,
                   consistent ? "settings-consistent"
                              : "*** SETTINGS MISMATCH across lanes ***");
    }
}

/*
 * Lynx-28G RX diagnostics
 *
 * eq-bins: LNmRECR3[31] EQ_SNAP_START snapshots the receiver's
 * equalization control/binning state; when RECR3[30] EQ_SNAP_DN sets,
 * LNmRECR4[15:12] selects one of nine bins read at RECR4[8:0].
 * TCSR0[31] SD_STAT_OBS_EN gates status observation ("used in receiver
 * adaptive equalization snapshot algorithm"): temporarily set if
 * clear, restored afterwards.
 *
 * jitter-scope (EXPERIMENTAL): TCSR1[27:24] SD_TST_SEL selects a lane
 * test mode (sampler / walking-tap / jitter-scope / BIST-checker /
 * pattern-independent jitter-scope) and TCSR1[8:0] RX_DATA_SM sets the
 * "offset between samplers within a bit"; TCSR2[15:0] RX_TST_DAT is
 * the raw readout. It dumps raw values for empirical decoding. May
 * disturb the live lane; TCSR1 is restored at the end.
 */

/*
 * Snapshot procedure and interpretation per the NXP lf-6.18.y BSP KR
 * link-training implementation (phy-fsl-lynx-28g.c + phy-fsl-lynx-core.c),
 * which documents:
 *  - one FRESH snapshot per bin read: wait EQ_SNAP_DN clear, select the
 *    bin in RECR4[15:12], set EQ_SNAP_START, poll EQ_SNAP_DN, read,
 *    clear EQ_SNAP_START;
 *  - RECR4[8:0] EQ_BIN_DATA is 9-bit two's complement (-256..255);
 *  - the snapshot also latches CTLE gains in RECR3 (GAINK2_HF[28:24],
 *    GAINK3_MF[20:16], GAINK4_LF[4:0]) and RECR4[21:16] EQ_OFFSET_STAT
 *    (OSESTAT, 0x00..0x3F) + RECR4[28:24] BLW_STAT;
 *  - health ("RX happy") criteria, 10 samples per metric: the Offset
 *    bin must NOT be 10x the same value; OSESTAT must sit in the
 *    mid-range 0x10..0x2F with spread <= 4; Bin1/Bin2/Bin3 averages
 *    must be in the TOGGLE band (-150..150; below = EARLY, above = LATE).
 */

#define BIN_NSAMP          10
#define BIN_THR            150
#define OSESTAT_MID_LO     0x10
#define OSESTAT_MID_HI     0x2F

#define SEL_BIN1     0x0
#define SEL_BIN2     0x1
#define SEL_BIN3     0x2
#define SEL_OFFSET   0x4
#define SEL_BLW      0x8
#define SEL_DATA_AVG 0x9
#define SEL_BIN_M1   0xC
#define SEL_BIN_LONG 0xD

/* one full snapshot with bin_sel selected; returns 0 and the raw
 * RECR3/RECR4 latched values, or -1 on timeout */
static int lynx_snapshot(volatile uint32_t *sd, int lane, uint32_t bin_sel,
                         uint32_t *recr3, uint32_t *recr4)
{
    int i;

    for (i = 0; i < 1000; i++) {
        if (!(lane_reg(sd, OFF_LANE_RECR3, lane) & RECR3_EQ_SNAP_DN))
            break;
        usleep(1);
    }
    if (i == 1000)
        return -1;

    uint32_t r4 = lane_reg(sd, OFF_LANE_RECR4, lane);
    lane_wr(sd, OFF_LANE_RECR4, lane,
            (r4 & ~(0xFu << 12)) | ((bin_sel & 0xFu) << 12));
    lane_wr(sd, OFF_LANE_RECR3, lane,
            lane_reg(sd, OFF_LANE_RECR3, lane) | RECR3_EQ_SNAP_START);

    for (i = 0; i < 1000; i++) {
        if (lane_reg(sd, OFF_LANE_RECR3, lane) & RECR3_EQ_SNAP_DN)
            break;
        usleep(1);
    }
    if (i == 1000) {
        lane_wr(sd, OFF_LANE_RECR3, lane,
                lane_reg(sd, OFF_LANE_RECR3, lane) & ~RECR3_EQ_SNAP_START);
        return -1;
    }

    *recr3 = lane_reg(sd, OFF_LANE_RECR3, lane);
    *recr4 = lane_reg(sd, OFF_LANE_RECR4, lane);
    lane_wr(sd, OFF_LANE_RECR3, lane,
            lane_reg(sd, OFF_LANE_RECR3, lane) & ~RECR3_EQ_SNAP_START);
    return 0;
}

static int bin_signed(uint32_t recr4)
{
    int v = (int)(recr4 & 0x1FFu);
    if (v & 0x100)
        v -= 512;                     /* 9-bit two's complement */
    return v;
}

struct bin_stat { int min, max, avg; int valid; };

static struct bin_stat collect_bin(volatile uint32_t *sd, int lane,
                                   uint32_t sel)
{
    struct bin_stat st = { 0, 0, 0, 0 };
    int sum = 0;
    uint32_t r3, r4;

    for (int i = 0; i < BIN_NSAMP; i++) {
        if (lynx_snapshot(sd, lane, sel, &r3, &r4) < 0)
            return st;
        int v = bin_signed(r4);
        if (!i || v < st.min) st.min = v;
        if (!i || v > st.max) st.max = v;
        sum += v;
    }
    st.avg = sum / BIN_NSAMP;
    st.valid = 1;
    return st;
}

static const char *bin_state(const struct bin_stat *st)
{
    if (!st->valid)          return "TIMEOUT";
    if (st->avg < -BIN_THR)  return "EARLY";
    if (st->avg > BIN_THR)   return "LATE";
    return "TOGGLE";
}

static int eq_bins_lane(volatile uint32_t *sd, int s, int lane, enum mode m)
{
    uint32_t gcr0 = lane_reg(sd, OFF_LANE_GCR0, lane);
    uint32_t ps = proto_sel(gcr0);

    if (!is_eth(ps)) {
        printf("SD%d LN%c: not an Ethernet lane (PROTO_SEL=0x%02x): skipped\n",
               s, 'A' + lane, ps);
        return 0;
    }
    if (!bit(lane_reg(sd, OFF_LANE_RRSTCTL, lane), LN_RRSTCTL_CDR_LOCK)) {
        printf("SD%d LN%c  cdr=0: no CDR lock, RX EQ snapshot not possible\n",
               s, 'A' + lane);
        return 1;
    }

    uint32_t tcsr0 = lane_reg(sd, OFF_LANE_TCSR0, lane);
    bool obs_forced = false;
    if (!(tcsr0 & TCSR0_SD_STAT_OBS_EN)) {
        lane_wr(sd, OFF_LANE_TCSR0, lane, tcsr0 | TCSR0_SD_STAT_OBS_EN);
        obs_forced = true;
    }

    /* timing bins, 10 fresh snapshots each */
    struct bin_stat b1 = collect_bin(sd, lane, SEL_BIN1);
    struct bin_stat b2 = collect_bin(sd, lane, SEL_BIN2);
    struct bin_stat b3 = collect_bin(sd, lane, SEL_BIN3);
    struct bin_stat bm1 = collect_bin(sd, lane, SEL_BIN_M1);
    struct bin_stat blong = collect_bin(sd, lane, SEL_BIN_LONG);

    /* offset bin: must not be 10x identical */
    int off_smp[BIN_NSAMP];
    int off_ok = 1, off_dithers = 0;
    uint32_t r3 = 0, r4 = 0;
    for (int i = 0; i < BIN_NSAMP; i++) {
        if (lynx_snapshot(sd, lane, SEL_OFFSET, &r3, &r4) < 0) {
            off_ok = 0;
            break;
        }
        off_smp[i] = bin_signed(r4);
        if (i && off_smp[i] != off_smp[0])
            off_dithers = 1;
    }

    /* gains + OSESTAT, latched by every snapshot */
    unsigned k2 = 0, k3 = 0, k4 = 0, ose_min = 0x3F, ose_max = 0;
    int gains_ok = 1, ose_mid = 1;
    for (int i = 0; i < BIN_NSAMP; i++) {
        if (lynx_snapshot(sd, lane, SEL_BIN1, &r3, &r4) < 0) {
            gains_ok = 0;
            break;
        }
        unsigned ose = (r4 >> 16) & 0x3Fu;
        k2 = (r3 >> 24) & 0x1Fu;
        k3 = (r3 >> 16) & 0x1Fu;
        k4 = r3 & 0x1Fu;
        if (ose < ose_min) ose_min = ose;
        if (ose > ose_max) ose_max = ose;
        if (ose < OSESTAT_MID_LO || ose > OSESTAT_MID_HI)
            ose_mid = 0;
    }
    unsigned blw = (r4 >> 24) & 0x1Fu;

    if (obs_forced)
        lane_wr(sd, OFF_LANE_TCSR0, lane, tcsr0);

    /* NXP "RX happy" verdict */
    const char *why = NULL;
    if (!b1.valid || !b2.valid || !b3.valid || !off_ok || !gains_ok)
        why = "snapshot timeout";
    else if (!off_dithers)
        why = "offset bin frozen (10x same value)";
    else if (!ose_mid)
        why = "OSESTAT outside mid-range 0x10..0x2F";
    else if (ose_max - ose_min > 4)
        why = "OSESTAT dither > +/-2";
    else if (strcmp(bin_state(&b1), "TOGGLE") || strcmp(bin_state(&b2), "TOGGLE") ||
             strcmp(bin_state(&b3), "TOGGLE"))
        why = "Bin1/2/3 not all TOGGLE";

    printf("SD%d LN%c  cdr=1  %s%s%s\n", s, 'A' + lane,
           why ? "RX-UNHAPPY (" : "RX-HAPPY", why ? why : "", why ? ")" : "");
    printf("        gains: K2_HF=%-2u K3_MF=%-2u K4_LF=%-2u  OSESTAT=0x%02x..0x%02x%s  BLW_STAT=%u\n",
           k2, k3, k4, ose_min, ose_max, ose_mid ? " (mid-range)" : "", blw);
    printf("        Bin1 %+4d [%+4d..%+4d] %-6s  Bin2 %+4d [%+4d..%+4d] %-6s  Bin3 %+4d [%+4d..%+4d] %-6s\n",
           b1.avg, b1.min, b1.max, bin_state(&b1),
           b2.avg, b2.min, b2.max, bin_state(&b2),
           b3.avg, b3.min, b3.max, bin_state(&b3));
    printf("        BinM1 %+4d [%+4d..%+4d] %-6s (pre-cursor)   BinLong %+4d [%+4d..%+4d] %-6s (post-cursor)  offset-bin dithers=%s\n",
           bm1.avg, bm1.min, bm1.max, bin_state(&bm1),
           blong.avg, blong.min, blong.max, bin_state(&blong),
           off_dithers ? "yes" : "NO");
    (void)m;
    return why ? 1 : 0;
}

static int jitter_scope_lane(volatile uint32_t *sd, int s, int lane,
                             uint32_t tstmode, unsigned step, unsigned dwell_us)
{
    uint32_t cdr0 = bit(lane_reg(sd, OFF_LANE_RRSTCTL, lane), LN_RRSTCTL_CDR_LOCK);
    uint32_t tcsr1_orig = lane_reg(sd, OFF_LANE_TCSR1, lane);
    uint32_t tcsr0_orig = lane_reg(sd, OFF_LANE_TCSR0, lane);
    uint32_t nonzero = 0;

    printf("SD%d LN%c jitter-scope [EXPERIMENTAL]: mode=0x%x step=%u dwell=%uus "
           "cdr_before=%u  (raw RX_TST_DAT dump; encoding undocumented in RM)\n",
           s, 'A' + lane, tstmode, step, dwell_us, cdr0);
    printf("  TCSR1 before: 0x%08x\n", tcsr1_orig);

    /* status observation gate on during the sweep (as for eq-bins) */
    if (!(tcsr0_orig & TCSR0_SD_STAT_OBS_EN))
        lane_wr(sd, OFF_LANE_TCSR0, lane, tcsr0_orig | TCSR0_SD_STAT_OBS_EN);

    for (unsigned off = 0; off < 512; off += step) {
        uint32_t v = (tcsr1_orig & ~((0xFu << 24) | 0x1FFu))
                     | (tstmode << 24) | (off & 0x1FFu);
        lane_wr(sd, OFF_LANE_TCSR1, lane, v);
        usleep(dwell_us);
        uint32_t d = lane_reg(sd, OFF_LANE_TCSR2, lane) & 0xFFFFu;
        nonzero |= d;
        if ((off / step) % 8 == 0)
            printf("  sm=%3u:", off);
        printf(" %04x", d);
        if ((off / step) % 8 == 7)
            printf("\n");
    }
    printf("\n");

    /* back to application mode with the original settings */
    lane_wr(sd, OFF_LANE_TCSR1, lane, tcsr1_orig);
    lane_wr(sd, OFF_LANE_TCSR0, lane, tcsr0_orig);
    usleep(10000);
    uint32_t cdr1 = bit(lane_reg(sd, OFF_LANE_RRSTCTL, lane), LN_RRSTCTL_CDR_LOCK);
    printf("  TCSR1 restored: 0x%08x   cdr_after=%u%s\n",
           lane_reg(sd, OFF_LANE_TCSR1, lane), cdr1,
           (cdr0 && !cdr1) ? "  *** lane lost lock:  give it a moment ***" : "");
    if (!nonzero)
        printf("  NOTE: RX_TST_DAT stayed 0 for the whole sweep (tested with and\n"
               "  without SD_STAT_OBS_EN). TBC with NXP.\n");
    return 0;
}

static int parse_lane(const char *a)
{
    if (!a || !*a)
        return -1;
    if (a[0] == 'L' || a[0] == 'l')       /* "LNA".."LNH" */
        a += 2;
    if (a[0] >= 'A' && a[0] <= 'H' && !a[1])
        return a[0] - 'A';
    if (a[0] >= 'a' && a[0] <= 'h' && !a[1])
        return a[0] - 'a';
    if (a[0] >= '0' && a[0] <= '7' && !a[1])
        return a[0] - '0';
    return -1;
}

static void
usage(const char *argv0)
{
    fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "Options:\n"
        "  -b, --block N    Only report SerDes block N (1..3)\n"
        "  -q, --quiet      One line per Ethernet lane (parse-friendly)\n"
        "  -h, --help       Show this help and exit\n"
        "\n"
        "Lynx-28G RX diagnostics (default block 1):\n"
        "      --eq-bins            RX equalization snapshot: 9 bins per lane\n"
        "                           (all Ethernet lanes, or one with --lane)\n"
        "      --jitter-scope       EXPERIMENTAL raw margining sweep (needs --lane;\n"
        "                           may disturb the lane; TCSR1 restored after)\n"
        "      --lane X             lane A..H / LNA..LNH / 0..7\n"
        "      --js-mode M          sampler|walktap|jitscope|bist|pijitter (default pijitter)\n"
        "      --js-step N          RX_DATA_SM sweep step 1..256 (default 8)\n"
        "      --js-dwell US        settle time per point in us (default 2000)\n"
        "\n"
        "Exit codes: 0 all Ethernet lanes healthy; 1 otherwise; 2 usage error\n",
        argv0);
}

int
main(int argc, char **argv)
{
    int filter_block = -1;
    enum mode m = MODE_HUMAN;
    bool do_eqbins = false, do_js = false;
    int diag_lane = -1;
    uint32_t js_mode = TST_PIJITTER;
    unsigned js_step = 8, js_dwell = 2000;

    enum { OPT_EQBINS = 1000, OPT_JS, OPT_LANE, OPT_JSMODE, OPT_JSSTEP,
           OPT_JSDWELL };
    static const struct option longopts[] = {
        { "block",        required_argument, NULL, 'b' },
        { "quiet",        no_argument,       NULL, 'q' },
        { "help",         no_argument,       NULL, 'h' },
        { "eq-bins",      no_argument,       NULL, OPT_EQBINS },
        { "jitter-scope", no_argument,       NULL, OPT_JS },
        { "lane",         required_argument, NULL, OPT_LANE },
        { "js-mode",      required_argument, NULL, OPT_JSMODE },
        { "js-step",      required_argument, NULL, OPT_JSSTEP },
        { "js-dwell",     required_argument, NULL, OPT_JSDWELL },
        { NULL, 0, NULL, 0 },
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "b:qh", longopts, NULL)) != -1) {
        switch (opt) {
        case 'b':
            filter_block = atoi(optarg);
            if (filter_block < 1 || filter_block > 3)
                errx(EXIT_USAGE, "block must be 1..3 (got '%s')", optarg);
            break;
        case 'q': m = MODE_QUIET; break;
        case OPT_EQBINS: do_eqbins = true; break;
        case OPT_JS: do_js = true; break;
        case OPT_LANE:
            diag_lane = parse_lane(optarg);
            if (diag_lane < 0)
                errx(EXIT_USAGE, "bad lane '%s' (A..H, LNA..LNH or 0..7)", optarg);
            break;
        case OPT_JSMODE:
            if      (!strcmp(optarg, "sampler"))  js_mode = TST_SAMPLER;
            else if (!strcmp(optarg, "walktap"))  js_mode = TST_WALKTAP;
            else if (!strcmp(optarg, "jitscope")) js_mode = TST_JITSCOPE;
            else if (!strcmp(optarg, "bist"))     js_mode = TST_BISTCHK;
            else if (!strcmp(optarg, "pijitter")) js_mode = TST_PIJITTER;
            else errx(EXIT_USAGE, "bad --js-mode '%s'", optarg);
            break;
        case OPT_JSSTEP:
            js_step = (unsigned)atoi(optarg);
            if (js_step < 1 || js_step > 256)
                errx(EXIT_USAGE, "js-step must be 1..256");
            break;
        case OPT_JSDWELL:
            js_dwell = (unsigned)atoi(optarg);
            break;
        case 'h': usage(argv[0]); return EXIT_SUCCESS;
        default:  usage(argv[0]); return EXIT_USAGE;
        }
    }

    bool diag = do_eqbins || do_js;
    if (do_js && diag_lane < 0)
        errx(EXIT_USAGE, "--jitter-scope needs --lane");

    int fd = open("/dev/mem", (diag ? O_RDWR : O_RDONLY) | O_SYNC);
    if (fd < 0)
        err(EXIT_FAILURE, "open /dev/mem (need root)");

    if (diag) {
        int s = filter_block > 0 ? filter_block : 1;
        volatile uint32_t *sd = map_phys_prot(fd, sd_base[s], SD_MAP_LEN,
                                              PROT_READ | PROT_WRITE);
        if (!sd)
            return EXIT_FAILURE;
        int rc = 0;
        if (do_eqbins) {
            printf("SD%d RX equalization snapshot bins (RECR3/RECR4; 9-bit raw values):\n", s);
            if (diag_lane >= 0)
                rc |= eq_bins_lane(sd, s, diag_lane, m);
            else
                for (int l = 0; l < NUM_LANES; l++)
                    if (is_eth(proto_sel(lane_reg(sd, OFF_LANE_GCR0, l))))
                        rc |= eq_bins_lane(sd, s, l, m);
        }
        if (do_js)
            rc |= jitter_scope_lane(sd, s, diag_lane, js_mode, js_step, js_dwell);
        return rc ? EXIT_FAILURE : EXIT_SUCCESS;
    }

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

    for (int s = 1; s <= 3; s++) {
        if (filter_block > 0 && s != filter_block)
            continue;

        volatile uint32_t *sd = map_phys(fd, sd_base[s], SD_MAP_LEN);
        if (!sd) {
            failures++;
            continue;
        }

        struct pll_view pf, ps_;
        load_pll(sd, &pf, &ps_);

        /* known protocol map? */
        const struct eth_map *em = NULL;
        for (size_t i = 0; i < sizeof(eth_maps) / sizeof(eth_maps[0]); i++)
            if (eth_maps[i].sd == s && eth_maps[i].proto == proto[s])
                em = &eth_maps[i];

        /* does this block carry any Ethernet lane at all? */
        bool any = em != NULL;
        if (!em)
            for (int l = 0; l < NUM_LANES; l++)
                if (is_eth(proto_sel(lane_reg(sd, OFF_LANE_GCR0, l))))
                    any = true;
        if (!any) {
            if (m == MODE_HUMAN)
                printf("\nSD%d (SRDS_PRTCL_S%d=%u): no Ethernet lanes\n",
                       s, s, proto[s]);
            continue;
        }

        if (m == MODE_HUMAN) {
            printf("\nSD%d (SRDS_PRTCL_S%d=%u)%s:\n", s, s, proto[s],
                   em ? "" : "  [protocol unknown to the table -- pure silicon decode]");
            printf("  PLLF: %s refclk=%s %s   PLLS: %s refclk=%s %s\n",
                   pf.dis ? "off" : (pf.lock ? "LOCKED" : "NOT-locked"),
                   refclk_mhz(pf.cr0), frate(pf.cr1),
                   ps_.dis ? "off" : (ps_.lock ? "LOCKED" : "NOT-locked"),
                   refclk_mhz(ps_.cr0), frate(ps_.cr1));
            print_groups(s, sd, m);
        }

        if (em) {
            for (int i = 0; i < em->nlanes; i++) {
                const struct eth_lane *el = &em->lanes[i];
                failures += print_eth_lane(s, sd, el->lane, el->expect,
                                           el->label, el->mac, &pf, &ps_, m);
                if (m == MODE_HUMAN)
                    printf("\n");
            }
        } else {
            for (int l = 0; l < NUM_LANES; l++) {
                if (!is_eth(proto_sel(lane_reg(sd, OFF_LANE_GCR0, l))))
                    continue;
                failures += print_eth_lane(s, sd, l, 0xff,
                                           "(silicon decode)", "MAC ?", &pf, &ps_, m);
                if (m == MODE_HUMAN)
                    printf("\n");
            }
        }

        if (m == MODE_QUIET)
            print_groups(s, sd, m);
    }

    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
