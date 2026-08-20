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
 *   ./lx2160-eth --quiet --stages # + the per-port link-stage ladder
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

#define SD_MAP_LEN         0x2000UL
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
#define LN_RST_REQ          31
#define LN_GCR0_PORT_LN0_B  16
#define LN_GCR0_PORT_RST_LEFT 17

/* LNmGCR0[PROTO_SEL] values */
#define PROTO_PCIE   0x00
#define PROTO_SGMII  0x01
#define PROTO_SATA   0x02
#define PROTO_10G    0x0a   /* XFI/SFI/10GBase-R, 10GBase-KR, 10G-SXGMII, 40G */
#define PROTO_25G    0x1a   /* 25GBase-R/KR, 50G, 100G (CAUI4) lanes */

/*
 * WRIOP MEMAC internal MDIO, used by --fec-mac to reach the RS-FEC
 * codeword counters. Core-side WRIOP port space is 0x08C0_0000 (RM: the
 * MC's own 0x28C0_0000 view is offset by 0x2000_0000); MEMAC n sits at
 * 0x08C07000 + (n-1)*0x4000, and the MDIO block is at +0x30 within it.
 */
#define MEMAC_BASE(n)       (0x08C07000UL + ((uint64_t)(n) - 1) * 0x4000UL)
#define MEMAC_MAP_LEN       0x1000UL
#define OFF_MDIO_CFG        0x30
#define OFF_MDIO_CTL        0x34
#define OFF_MDIO_DATA       0x38
#define OFF_MDIO_ADDR       0x3C
#define MDIO_CTL_READ       0x8000u
#define MDIO_CFG_BSY        0x1u
#define MDIO_CFG_RD_ER      0x2u

/* MDIO device addresses, RM Table 297 */
#define MDD_PCS             3
#define MDD_RSFEC           30

/* RS-FEC registers, RM SS26.5.3.2.1.2.x -- CCW_LO must be read first, and
 * the counters clear on read, so a reading is "since the last read". */
#define RSFEC_STATUS        1
#define RSFEC_CCW_LO        2
#define RSFEC_CCW_HI        3
#define RSFEC_NCCW_LO       4
#define RSFEC_NCCW_HI       5
#define RSFEC_STATUS_ALIGN  14      /* FEC_ALIGN_S: locked + deskew done */

/* Ethernet protocol status registers */
#define OFF_SXGMIICR3(l)    (0x1A80 + (l) * 0x10 + 0xC)
#define OFF_E25GCR3(l)      (0x1B00 + (l) * 0x10 + 0xC)
#define OFF_E40GCR3(p)      ((p) ? 0x1C4C : 0x1C0C)
#define OFF_E50GCR3(p)      ((p) ? 0x1DCC : 0x1DAC)
#define OFF_E100GCR3(p)     ((p) ? 0x1E2C : 0x1E0C)

/* SXGMIIaCR3 -- 10G single lane (RM SS26.4.1.61) */
#define SXGMII_CR3_BLOCK_LK     7
#define SXGMII_CR3_FEC_LK       6
#define SXGMII_CR3_HI_BER       4

#define OFF_E25GCR2(l)          (0x1B00 + (l) * 0x10 + 0x8)
#define E25G_CR2_FEC91_ENA      0x00100000u   /* RS-FEC (CL91) */
#define E25G_CR2_FEC_ENA        0x00800000u   /* FC-FEC (CL74) */

/* E25GaCR3 -- 25G single lane (RM SS26.4.1.64) */
#define E25G_CR3_FEC_LK         16
#define E25G_CR3_RSFEC_ALN      12
#define E25G_CR3_AMPS_LK        8
#define E25G_CR3_HI_BER         4
#define E25G_CR3_LINK_ST        0

/* E40GaCR3 -- 4 x 10G (RM SS26.4.1.67); the RM table lists no LINK_ST */
#define E40G_CR3_ALIGN_DN       8
#define E40G_CR3_HI_BER         4

/* E50GaCR3 -- 2 x 25G (RM SS26.4.1.69) */
#define E50G_CR3_HI_BER         4
#define E50G_CR3_ALIGN_DN       1
#define E50G_CR3_LINK_ST        0

#define OFF_E100GCR2(p)             ((p) ? 0x1E28 : 0x1E08)
#define E100G_CR2_FEC91_ENA         0x00F00000u
#define E100G_CR2_BLOCK_LK_MASK     0x000FFFFFu

#define E100G_CR3_RSFEC_ALN_SHIFT   12
#define E100G_CR3_AMPS_LK_SHIFT     8
#define E100G_CR3_LANE_MASK         0xFu
#define E100G_CR3_HI_BER            4
#define E100G_CR3_ALIGN_LK          1
#define E100G_CR3_LINK_ST           0

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
    const char *mac;        /* WRIOP MAC / dpmac id, for display */
    int      mac_id;        /* same, numeric, for --fec */
};

struct eth_map {
    int      sd;
    uint32_t proto;
    int      nlanes;
    struct eth_lane lanes[NUM_LANES];
};

static const struct eth_map eth_maps[] = {
    { .sd = 1, .proto = 13, .nlanes = 8, .lanes = {
        { 0, PROTO_25G, "100GE.1 lane 0", "MAC2 (CAUI4)", 2 },
        { 1, PROTO_25G, "100GE.1 lane 1", "MAC2 (CAUI4)", 2 },
        { 2, PROTO_25G, "100GE.1 lane 2", "MAC2 (CAUI4)", 2 },
        { 3, PROTO_25G, "100GE.1 lane 3", "MAC2 (CAUI4)", 2 },
        { 4, PROTO_25G, "100GE.2 lane 0", "MAC1 (CAUI4)", 1 },
        { 5, PROTO_25G, "100GE.2 lane 1", "MAC1 (CAUI4)", 1 },
        { 6, PROTO_25G, "100GE.2 lane 2", "MAC1 (CAUI4)", 1 },
        { 7, PROTO_25G, "100GE.2 lane 3", "MAC1 (CAUI4)", 1 },
    } },
    { .sd = 2, .proto = 7, .nlanes = 6, .lanes = {
        { 1, PROTO_SGMII, "SGMII.12",   "MAC12", 12 },
        { 2, PROTO_SGMII, "SGMII.17",   "MAC17", 17 },
        { 3, PROTO_SGMII, "SGMII.18",   "MAC18", 18 },
        { 5, PROTO_SGMII, "SGMII.16",   "MAC16", 16 },
        { 6, PROTO_10G,   "USXGMII.13", "MAC13", 13 },
        { 7, PROTO_10G,   "USXGMII.14", "MAC14", 14 },
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

/*
 * Link bring-up ladder for one Ethernet port.
 */

#define MAX_STAGES 9

struct link_stage {
    const char *name;
    const char *note;      /* RM field name, for cross-reference */
    uint32_t    val;       /* per-lane bitmap, or 0/1 for a single bit */
    int         lanes;     /* >1 => render val as a per-lane bitmap */
    bool        ok;

    bool        info;
    const char *text;      /* optional rendering override for info rows */
};

static void
add_stage(struct link_stage *st, int *ns, const char *name, const char *note,
          uint32_t val, int lanes, bool ok)
{
    st[*ns] = (struct link_stage){ .name = name, .note = note, .val = val,
                                   .lanes = lanes, .ok = ok };
    (*ns)++;
}

/* Printed, but never eligible as "the stage we stalled at". */
static void
add_info(struct link_stage *st, int *ns, const char *name, const char *note,
         uint32_t val, int lanes, bool ok, const char *text)
{
    st[*ns] = (struct link_stage){ .name = name, .note = note, .val = val,
                                   .lanes = lanes, .ok = ok, .info = true,
                                   .text = text };
    (*ns)++;
}

/* Render a per-lane field lane-0-first: field 0b0011 over 4 lanes -> "1100". */
static void
lane_bits(uint32_t v, int n, char *out)
{
    int i;
    for (i = 0; i < n; i++)
        out[i] = ((v >> i) & 1u) ? '1' : '0';
    out[i] = '\0';
}

static int
proto_instance(int sd_idx, int first, int n)
{
    if (sd_idx == 1)
        return (NUM_LANES - first - n) / n;
    return first / n;
}

/* Which protocol status register serves this port, and which layout it uses. */
enum ps_kind { PSK_NONE, PSK_E100G, PSK_E50G, PSK_E25G, PSK_E40G, PSK_SXGMII };

static enum ps_kind
port_status_reg(int sd_idx, uint32_t ps, int first, int n,
                uint32_t *off, char *name, size_t namesz)
{
    int inst = proto_instance(sd_idx, first, n);

    if      (ps == PROTO_25G && n == 4) { if (inst > 1) return PSK_NONE;
        *off = OFF_E100GCR3(inst); snprintf(name, namesz, "E100G%cCR3", 'a'+inst); return PSK_E100G; }
    else if (ps == PROTO_25G && n == 2) { if (inst > 1) return PSK_NONE;
        *off = OFF_E50GCR3(inst);  snprintf(name, namesz, "E50G%cCR3",  'a'+inst); return PSK_E50G; }
    else if (ps == PROTO_25G && n == 1) {
        *off = OFF_E25GCR3(inst);  snprintf(name, namesz, "E25G%cCR3",  'a'+inst); return PSK_E25G; }
    else if (ps == PROTO_10G && n == 4) { if (inst > 1) return PSK_NONE;
        *off = OFF_E40GCR3(inst);  snprintf(name, namesz, "E40G%cCR3",  'a'+inst); return PSK_E40G; }
    else if (ps == PROTO_10G && n == 1) {
        *off = OFF_SXGMIICR3(inst);snprintf(name, namesz, "SXGMII%cCR3",'a'+inst); return PSK_SXGMII; }
    return PSK_NONE;
}

/*
 * Does this port show the "stuck PCS" signature that a lane reset clears?
 *
 *   every lane CDR-locked + LINK_ST == 0 + HI_BER == 0
 *
 * i.e. signal is arriving and clocked, the port is down, and it is not a
 * noise problem. That is what MC leaves behind when its link state machine
 * latches on a grouped port: `ip link down/up` does not clear it
 *
 * Caution: this signature is also what a link partner that is not emitting
 * valid framing looks like. The two are indistinguishable from here, which
 * is why the kick is the cheap thing to try FIRST
 *
 * A lane with CDR=0 is excluded: there is no signal to re-lock onto.
 * HI_BER=1 is excluded: that is signal integrity, and a reset only hides it.
 */
static bool
port_is_stuck(int sd_idx, const volatile uint32_t *sd, int first, int n,
              uint32_t gcr0, const char *cdr, const char **why)
{
    uint32_t off = 0, reg;
    char nm[16];
    enum ps_kind k = port_status_reg(sd_idx, proto_sel(gcr0), first, n,
                                     &off, nm, sizeof nm);
    int link_bit, hiber_bit;

    for (int i = 0; i < n; i++)
        if (cdr[i] != '1') { *why = "not every lane has CDR lock -- no signal to re-lock"; return false; }

    switch (k) {
    case PSK_E100G: link_bit = E100G_CR3_LINK_ST; hiber_bit = E100G_CR3_HI_BER; break;
    case PSK_E50G:  link_bit = E50G_CR3_LINK_ST;  hiber_bit = E50G_CR3_HI_BER;  break;
    case PSK_E25G:  link_bit = E25G_CR3_LINK_ST;  hiber_bit = E25G_CR3_HI_BER;  break;
    default:        *why = "no LINK_ST for this port type"; return false;
    }

    reg = rd32(sd, off);
    if (bit(reg, link_bit))  { *why = "already up (LINK_ST=1)"; return false; }
    if (bit(reg, hiber_bit)) { *why = "HI_BER set -- signal integrity, not a latch"; return false; }
    *why = "CDR locked on all lanes, LINK_ST=0, HI_BER=0";
    return true;
}

static void
print_link_stages(int sd_idx, const volatile uint32_t *sd,
                  int first, int n, uint32_t gcr0, const char *cdr,
                  enum mode m)
{
    uint32_t ps  = proto_sel(gcr0);
    int      inst = proto_instance(sd_idx, first, n);
    struct link_stage st[MAX_STAGES];
    int      ns = 0;
    uint32_t reg, off;
    const char *regname;
    bool cdr_all = true;

    for (int i = 0; i < n; i++)
        if (cdr[i] != '1')
            cdr_all = false;

    static char rn[16];

    if (port_status_reg(sd_idx, ps, first, n, &off, rn, sizeof rn) == PSK_NONE)
        return;                /* 1G/PCIe/SATA have their own status paths */
    regname = rn;

    reg = rd32(sd, off);

    add_stage(st, &ns, "CDR lock",
                  "LNaRRSTCTL[12]",
                  0, n, cdr_all);

    if (ps == PROTO_25G && n == 4) {
        uint32_t amps = (reg >> E100G_CR3_AMPS_LK_SHIFT)   & E100G_CR3_LANE_MASK;
        uint32_t aln  = (reg >> E100G_CR3_RSFEC_ALN_SHIFT) & E100G_CR3_LANE_MASK;
        uint32_t all  = (1u << n) - 1u;
        uint32_t cr2  = rd32(sd, OFF_E100GCR2(inst));
        uint32_t fec  = (cr2 & E100G_CR2_FEC91_ENA) == E100G_CR2_FEC91_ENA;
        uint32_t vlbl = cr2 & E100G_CR2_BLOCK_LK_MASK;
        static char vlbuf[16], fecnote[40], blknote[48];

        snprintf(fecnote, sizeof fecnote, "E100G%cCR2 FEC91_ENA", 'a' + inst);
        snprintf(blknote, sizeof blknote,
                 "E100G%cCR2[19:0], n/a under RS-FEC", 'a' + inst);
        add_info(st, &ns, "RS-FEC enabled", fecnote,
                 fec, 1, true, fec ? "yes" : "no");
        add_stage(st, &ns, "RS-FEC codeword align",
                  "AMPS_LK (pre-deskew)",
                  amps, n, amps == all);
        add_info(st, &ns, "RS-FEC deskew+reorder", "RSFEC_ALN",
                 aln, n, aln == all, NULL);
        /* 20 virtual lanes; the RM marks this not relevant in RS-FEC mode. */
        snprintf(vlbuf, sizeof vlbuf, "%d/20", __builtin_popcount(vlbl));
        add_info(st, &ns, "64b/66b block lock", blknote,
                 vlbl, 1, vlbl == E100G_CR2_BLOCK_LK_MASK, vlbuf);
        add_stage(st, &ns, "AM lock (all VLs)",
                  "ALIGN_LK",
                  bit(reg, E100G_CR3_ALIGN_LK), 1, bit(reg, E100G_CR3_ALIGN_LK));
        add_stage(st, &ns, "no high BER",
                  "HI_BER",
                  bit(reg, E100G_CR3_HI_BER), 1, !bit(reg, E100G_CR3_HI_BER));
        add_stage(st, &ns, "LINK_ST (MC polls this)",
                  "ALIGN_LK && !HI_BER",
                  bit(reg, E100G_CR3_LINK_ST), 1, bit(reg, E100G_CR3_LINK_ST));
    } else if (ps == PROTO_25G && n == 2) {
        add_stage(st, &ns, "AM lock (all VLs)",
                  "ALIGN_DN",
                  bit(reg, E50G_CR3_ALIGN_DN), 1, bit(reg, E50G_CR3_ALIGN_DN));
        add_stage(st, &ns, "no high BER",
                  "HI_BER",
                  bit(reg, E50G_CR3_HI_BER), 1, !bit(reg, E50G_CR3_HI_BER));
        add_stage(st, &ns, "LINK_ST (MC polls this)",
                  "ALIGN_DN && !HI_BER",
                  bit(reg, E50G_CR3_LINK_ST), 1, bit(reg, E50G_CR3_LINK_ST));
    } else if (ps == PROTO_25G && n == 1) {
        uint32_t cr2_25 = rd32(sd, OFF_E25GCR2(inst));
        static char fecnote25[40];
        const char *fecstr = (cr2_25 & E25G_CR2_FEC91_ENA) ? "rs" :
                             (cr2_25 & E25G_CR2_FEC_ENA)   ? "fc" : "none";
        snprintf(fecnote25, sizeof fecnote25, "E25G%cCR2 FEC_ENA", 'a' + inst);
        add_info(st, &ns, "FEC enabled", fecnote25,
                 cr2_25 & (E25G_CR2_FEC91_ENA | E25G_CR2_FEC_ENA), 1, true, fecstr);
        add_info(st, &ns, "RS-FEC lock", "FEC_LK (never seen set)",
                 bit(reg, E25G_CR3_FEC_LK), 1, bit(reg, E25G_CR3_FEC_LK), NULL);
        add_stage(st, &ns, "RS-FEC codeword align",
                  "AMPS_LK",
                  bit(reg, E25G_CR3_AMPS_LK), 1, bit(reg, E25G_CR3_AMPS_LK));
        add_stage(st, &ns, "RS-FEC alignment",
                  "RSFEC_ALN",
                  bit(reg, E25G_CR3_RSFEC_ALN), 1, bit(reg, E25G_CR3_RSFEC_ALN));
        add_stage(st, &ns, "no high BER",
                  "HI_BER",
                  bit(reg, E25G_CR3_HI_BER), 1, !bit(reg, E25G_CR3_HI_BER));
        add_stage(st, &ns, "LINK_ST (MC polls this)",
                  "",
                  bit(reg, E25G_CR3_LINK_ST), 1, bit(reg, E25G_CR3_LINK_ST));
    } else if (ps == PROTO_10G && n == 4) {
        add_stage(st, &ns, "AM lock (all VLs)",
                  "ALIGN_DN",
                  bit(reg, E40G_CR3_ALIGN_DN), 1, bit(reg, E40G_CR3_ALIGN_DN));
        add_stage(st, &ns, "no high BER",
                  "HI_BER",
                  bit(reg, E40G_CR3_HI_BER), 1, !bit(reg, E40G_CR3_HI_BER));
    } else {   /* SXGMII / XFI, single lane */
        /* SXGMIIaCR2 exposes no FEC-enable bit, so a clear FEC_LK cannot be
         * distinguished from "this link does not use FEC". Informational:
         * BLOCK_LK is the gating stage here. */
        add_info(st, &ns, "FEC lock",
                 "FEC_LK (only if FEC in use)",
                 bit(reg, SXGMII_CR3_FEC_LK), 1, bit(reg, SXGMII_CR3_FEC_LK), NULL);
        add_stage(st, &ns, "64b/66b block lock",
                  "BLOCK_LK",
                  bit(reg, SXGMII_CR3_BLOCK_LK), 1, bit(reg, SXGMII_CR3_BLOCK_LK));
        add_stage(st, &ns, "no high BER",
                  "HI_BER",
                  bit(reg, SXGMII_CR3_HI_BER), 1, !bit(reg, SXGMII_CR3_HI_BER));
    }

    const char *stalled = NULL;
    for (int i = 0; i < ns; i++)
        if (!st[i].info && !st[i].ok) { stalled = st[i].name; break; }

    char buf[24];

    if (m == MODE_QUIET) {
        printf("SD%d STAGES LN%c..LN%c %s=0x%08x", sd_idx,
               'A' + first, 'A' + first + n - 1, regname, reg);
        for (int i = 0; i < ns; i++) {
            if (i == 0)                 snprintf(buf, sizeof buf, "%s", cdr);
            else if (st[i].text)        snprintf(buf, sizeof buf, "%s", st[i].text);
            else if (st[i].lanes > 1)   lane_bits(st[i].val, st[i].lanes, buf);
            else                        snprintf(buf, sizeof buf, "%u", st[i].val);
            printf(" %s=%s", st[i].note[0] ? st[i].note : st[i].name, buf);
        }
        printf(" -> %s\n", stalled ? stalled : "LINK UP");
        return;
    }

    printf("       link stages (%s = 0x%08x):\n", regname, reg);
    for (int i = 0; i < ns; i++) {
        if (i == 0)                 snprintf(buf, sizeof buf, "%s", cdr);
        else if (st[i].text)        snprintf(buf, sizeof buf, "%s", st[i].text);
        else if (st[i].lanes > 1)   lane_bits(st[i].val, st[i].lanes, buf);
        else                        snprintf(buf, sizeof buf, "%u", st[i].val);
        printf("         %-24s %-6s %s%s%s\n",
               st[i].name, buf,
               st[i].info ? "(i) " : (st[i].ok ? "[OK]" : "[!!]"),
               st[i].note[0] ? "   " : "", st[i].note);
    }
    printf("         => %s\n",
           stalled ? stalled : "link up");
}

/*
 * Un-halted RX reset on one port's lanes: LNaRRSTCTL[31] RST_REQ, no
 * preceding HLT_REQ.
 *
 * It bounces the link. Gated on port_is_stuck() unless --force.
 */
static void
rx_kick_port(volatile uint32_t *sd, int first, int n)
{
    for (int i = 0; i < n; i++) {
        uint32_t off = OFF_LANE_RRSTCTL + (uint32_t)(first + i) * LANE_STRIDE;
        wr32(sd, off, rd32(sd, off) | (1u << LN_RST_REQ));
    }
}

/*
 * --rx-kick: find ports showing the stuck-PCS signature and reset their
 * lanes. Reports what it did and, after settling, whether it worked.
 */
static int
rx_kick_block(int sd_idx, volatile uint32_t *sd, bool force)
{
    uint32_t gcr0[NUM_LANES], rrst[NUM_LANES];
    int kicked = 0, recovered = 0, a = 0;
    struct { int first, n; } hit[NUM_LANES];

    for (int i = 0; i < NUM_LANES; i++) {
        gcr0[i] = lane_reg(sd, OFF_LANE_GCR0,    i);
        rrst[i] = lane_reg(sd, OFF_LANE_RRSTCTL, i);
    }

    while (a < NUM_LANES) {
        int first = a, n = 1;
        bool left = bit(gcr0[first], LN_GCR0_PORT_RST_LEFT) == 1;
        char cdr[NUM_LANES + 1];
        const char *why = "";

        if (left) {
            while (first + n < NUM_LANES && bit(gcr0[first+n], LN_GCR0_PORT_LN0_B) == 1 &&
                   proto_sel(gcr0[first+n]) == proto_sel(gcr0[first])) n++;
        } else {
            while (bit(gcr0[first+n-1], LN_GCR0_PORT_LN0_B) == 1 && first + n < NUM_LANES &&
                   proto_sel(gcr0[first+n]) == proto_sel(gcr0[first])) n++;
        }
        a = first + n;
        if (!is_eth(proto_sel(gcr0[first])))
            continue;

        for (int i = 0; i < n; i++)
            cdr[i] = bit(rrst[first+i], LN_RRSTCTL_CDR_LOCK) ? '1' : '0';
        cdr[n] = '\0';

        bool stuck = port_is_stuck(sd_idx, sd, first, n, gcr0[first], cdr, &why);
        printf("  SD%d LN%c..LN%c  CDR=%-8s %s\n", sd_idx, 'A'+first,
               'A'+first+n-1, cdr, stuck ? "STUCK -- kicking" :
               (force ? "not stuck, but --force" : why));
        if (!stuck && !force)
            continue;
        rx_kick_port(sd, first, n);
        hit[kicked].first = first; hit[kicked].n = n; kicked++;
    }

    if (!kicked) {
        printf("  nothing to do\n");
        return 0;
    }
    /* MC polls FIXED links at 1 Hz and can take well over 10 s to
     * re-assert; do not judge the result too early. */
    printf("  waiting 20 s for MC to re-assert link...\n");
    fflush(stdout);
    sleep(20);
    for (int i = 0; i < kicked; i++) {
        uint32_t off = 0, g = lane_reg(sd, OFF_LANE_GCR0, hit[i].first);
        char nm[16];
        enum ps_kind k = port_status_reg(sd_idx, proto_sel(g), hit[i].first,
                                         hit[i].n, &off, nm, sizeof nm);
        int lb = (k == PSK_E100G) ? E100G_CR3_LINK_ST :
                 (k == PSK_E50G)  ? E50G_CR3_LINK_ST  : E25G_CR3_LINK_ST;
        bool up = (k != PSK_NONE) && bit(rd32(sd, off), lb);
        printf("  SD%d LN%c..LN%c  %s = 0x%08x  -> %s\n", sd_idx,
               'A'+hit[i].first, 'A'+hit[i].first+hit[i].n-1, nm,
               k == PSK_NONE ? 0 : rd32(sd, off),
               up ? "LINK UP" : "still down -- not a latch; look at the link partner");
        if (up) recovered++;
    }
    return kicked - recovered;      /* non-zero => something did not recover */
}

/* Multi-lane grouping header, same PORT_LN0_B walk as lx2160-sdx. */
static void
print_groups(int sd_idx, const volatile uint32_t *sd, enum mode m,
             bool want_stages)
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

        if (!is_eth(proto_sel(gcr0[first])))
            continue;

        bool consistent = true;
        char cdr[NUM_LANES + 1];
        for (int i = 0; i < n; i++) {
            if (((gcr0[first + i] ^ gcr0[first]) & 0xffu) != 0)
                consistent = false;
            cdr[i] = bit(rrst[first + i], LN_RRSTCTL_CDR_LOCK) ? '1' : '0';
        }
        cdr[n] = '\0';

        /* The group header stays multi-lane only; single-lane ports are
         * already covered by the per-lane report above it. The stage
         * ladder applies to both, so it is emitted for every port. */
        if (n >= 2) {
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

        if (m == MODE_HUMAN || want_stages)
            print_link_stages(sd_idx, sd, first, n, gcr0[first], cdr, m);
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

/* One Clause-45 read on the MEMAC internal MDIO */
static int mdio_wait(volatile uint32_t *mac)
{
    /* ~100 ms at any plausible MDC divider; a Clause-45 frame is ~50 us. */
    for (int i = 0; i < 200000; i++)
        if (!(rd32(mac, OFF_MDIO_CFG) & MDIO_CFG_BSY))
            return 0;
    return -1;
}

static int mdio_c45_read(volatile uint32_t *mac, int dev, uint16_t reg,
                         uint16_t *out)
{
    uint32_t ctl = (uint32_t)dev;          /* port address 0 */

    /*
     * Every step has to wait for BSY. Omitting the waits appears to work
     * when each access is a separate process (spawn latency covers the
     * frame time) and fails outright in a tight loop, returning whatever
     * MDIO_DATA held from the previous transaction.
     */
    if (mdio_wait(mac) < 0)
        return -1;
    wr32(mac, OFF_MDIO_CTL,  ctl);
    wr32(mac, OFF_MDIO_ADDR, reg);         /* address cycle */
    if (mdio_wait(mac) < 0)
        return -1;
    wr32(mac, OFF_MDIO_CTL,  ctl | MDIO_CTL_READ);
    if (mdio_wait(mac) < 0)
        return -1;
    if (rd32(mac, OFF_MDIO_CFG) & MDIO_CFG_RD_ER)
        return -1;                         /* device did not respond */
    *out = (uint16_t)rd32(mac, OFF_MDIO_DATA);
    return 0;
}

/*
 * RS-FEC corrected / uncorrected codeword counters for one WRIOP MAC.
 *
 * This is the pre-FEC margin instrument: HI_BER only tells you the link
 * has crossed a threshold, while these say how hard FEC was working
 * before it did. Zero corrected AND zero uncorrected means the channel is
 * error-free, not merely linked.
 */
static int fec_counters(volatile uint32_t *mac, int mac_id, enum mode m)
{
    uint32_t saved = rd32(mac, OFF_MDIO_CTL);
    uint32_t ccw, nccw;
    uint16_t lo = 0, hi = 0, st = 0;
    int err = mdio_c45_read(mac, MDD_RSFEC, RSFEC_STATUS, &st);

    if (err < 0 || st == 0) {
        /* Cross-check that the MDIO itself works before blaming the MMD. */
        uint16_t pcs = 0;
        (void)mdio_c45_read(mac, MDD_PCS, 0x20, &pcs);
        wr32(mac, OFF_MDIO_CTL, saved);
        if (m == MODE_QUIET)
            printf("MAC%d FEC no-rsfec-mmd pcs_baser_status1=0x%04x\n", mac_id, pcs);
        else
            printf("  MAC%-2d  no RS-FEC MMD response (device %d silent; "
                   "PCS device %d reads 0x%04x)\n"
                   "         RS-FEC is configured in the PCS vendor window on this MAC;\n"
                   "         codeword counters are not exposed there.\n",
                   mac_id, MDD_RSFEC, MDD_PCS, pcs);
        return 2;
    }

    err  = mdio_c45_read(mac, MDD_RSFEC, RSFEC_CCW_LO, &lo);  /* LO first */
    err |= mdio_c45_read(mac, MDD_RSFEC, RSFEC_CCW_HI, &hi);
    ccw = ((uint32_t)hi << 16) | lo;
    err |= mdio_c45_read(mac, MDD_RSFEC, RSFEC_NCCW_LO, &lo);
    err |= mdio_c45_read(mac, MDD_RSFEC, RSFEC_NCCW_HI, &hi);
    nccw = ((uint32_t)hi << 16) | lo;
    if (err < 0) {
        wr32(mac, OFF_MDIO_CTL, saved);
        warnx("MAC%d: MDIO read error while fetching counters", mac_id);
        return 1;
    }

    wr32(mac, OFF_MDIO_CTL, saved);

    if (m == MODE_QUIET) {
        printf("MAC%d FEC status=0x%04x align=%u corrected=%u uncorrected=%u\n",
               mac_id, st, bit(st, RSFEC_STATUS_ALIGN), ccw, nccw);
    } else {
        printf("  MAC%-2d  RS_FEC_STATUS=0x%04x  FEC_ALIGN_S=%u (%s)\n",
               mac_id, st, bit(st, RSFEC_STATUS_ALIGN),
               bit(st, RSFEC_STATUS_ALIGN)
                   ? "locked, deskew complete"
                   : "not aligned -- no link, or RS-FEC not in use here");
        printf("         corrected codewords   = %-10u  (CCW)\n", ccw);
        printf("         uncorrected codewords = %-10u  (NCCW)\n", nccw);
        printf("         counters cleared by this read -- next read is the delta\n");
    }
    return (nccw != 0) ? 1 : 0;
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
        "      --stages     Also emit the per-port link-stage ladder in --quiet\n"
        "      --rx-kick    Reset the lanes of any port showing the stuck-PCS\n"
        "                   signature (every lane CDR-locked, LINK_ST=0,\n"
        "                   HI_BER=0), then report whether it recovered.\n"
        "                   BOUNCES THE LINK. --force skips the gate\n"
        "      --fec        RS-FEC corrected/uncorrected codeword counters for\n"
        "                   every MAC of the selected block(s), MACs derived\n"
        "                   from the protocol table. Counters clear on read\n"
        "      --fec-mac N  Same, for WRIOP MAC N (1..18) explicitly. Needed\n"
        "                   when a PBI re-shaped the lanes, so the protocol\n"
        "                   table does not describe the board\n"
        "                   (always shown in the default human report)\n"
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
    bool do_eqbins = false, do_js = false, want_stages = false;
    bool do_kick = false, kick_force = false;
    int  fec_mac = -1;
    bool do_fec  = false;
    int diag_lane = -1;
    uint32_t js_mode = TST_PIJITTER;
    unsigned js_step = 8, js_dwell = 2000;

    enum { OPT_EQBINS = 1000, OPT_JS, OPT_LANE, OPT_JSMODE, OPT_JSSTEP,
           OPT_JSDWELL, OPT_STAGES, OPT_FECMAC, OPT_FEC, OPT_KICK, OPT_FORCE };
    static const struct option longopts[] = {
        { "block",        required_argument, NULL, 'b' },
        { "quiet",        no_argument,       NULL, 'q' },
        { "help",         no_argument,       NULL, 'h' },
        { "stages",       no_argument,       NULL, OPT_STAGES },
        { "rx-kick",      no_argument,       NULL, OPT_KICK },
        { "force",        no_argument,       NULL, OPT_FORCE },
        { "fec",          no_argument,       NULL, OPT_FEC },
        { "fec-mac",      required_argument, NULL, OPT_FECMAC },
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
        case OPT_STAGES: want_stages = true; break;
        case OPT_KICK:  do_kick = true; break;
        case OPT_FORCE: kick_force = true; break;
        case OPT_FEC: do_fec = true; break;
        case OPT_FECMAC:
            fec_mac = atoi(optarg);
            if (fec_mac < 1 || fec_mac > 18)
                errx(EXIT_USAGE, "--fec-mac must be a WRIOP MAC id 1..18");
            break;
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

    bool diag = do_eqbins || do_js || fec_mac > 0 || do_fec || do_kick;
    if (do_js && diag_lane < 0)
        errx(EXIT_USAGE, "--jitter-scope needs --lane");

    int fd = open("/dev/mem", (diag ? O_RDWR : O_RDONLY) | O_SYNC);
    if (fd < 0)
        err(EXIT_FAILURE, "open /dev/mem (need root)");

    if (do_kick) {
        int rc = 0;
        printf("RX lane reset (LNaRRSTCTL[31] RST_REQ, un-halted). This bounces the link.\n");
        for (int b = 1; b <= 3; b++) {
            volatile uint32_t *sd;
            if (filter_block > 0 && b != filter_block)
                continue;
            sd = map_phys_prot(fd, sd_base[b], SD_MAP_LEN, PROT_READ | PROT_WRITE);
            if (!sd) { rc = 1; continue; }
            rc |= rx_kick_block(b, sd, kick_force);
        }
        close(fd);
        return rc ? EXIT_FAILURE : EXIT_SUCCESS;
    }

    if (do_fec && fec_mac < 0) {
        /*
         * Derive the MACs from the protocol table rather than making the
         * caller name them. Only possible where the (block, protocol)
         * pair is known: a PBI that re-shapes lanes after reset - the
         * NBV32 100G split, say - has no table entry, and guessing a MAC
         * id there reads a MEMAC that may not belong to any port at all.
         * Say so and let --fec-mac override.
         */
        int seen[19] = { 0 };
        int found = 0, rc_fec = 0;
        uint32_t proto[4] = { 0 };
        volatile uint32_t *dcfg0 = map_phys(fd, DCFG_BASE, DCFG_MAP_LEN);

        if (!dcfg0)
            return EXIT_FAILURE;
        {
            uint32_t r29 = rd32(dcfg0, OFF_RCWSR29);
            proto[1] = (r29 >> 16) & 0x1fu;
            proto[2] = (r29 >> 21) & 0x1fu;
            proto[3] = (r29 >> 26) & 0x1fu;
        }

        for (int s = 1; s <= 3; s++) {
            const struct eth_map *em = NULL;

            if (filter_block > 0 && s != filter_block)
                continue;
            for (size_t i = 0; i < sizeof(eth_maps) / sizeof(eth_maps[0]); i++)
                if (eth_maps[i].sd == s && eth_maps[i].proto == proto[s])
                    em = &eth_maps[i];
            if (!em)
                continue;

            for (int i = 0; i < em->nlanes; i++) {
                int id = em->lanes[i].mac_id;

                if (id < 1 || id > 18 || seen[id])
                    continue;         /* one read per MAC, not per lane */
                seen[id] = 1;
                found++;
                volatile uint32_t *mac = map_phys_prot(fd, MEMAC_BASE(id),
                                                       MEMAC_MAP_LEN,
                                                       PROT_READ | PROT_WRITE);
                if (!mac)
                    continue;
                if (m == MODE_HUMAN)
                    printf("SD%d %s:\n", s, em->lanes[i].label);
                if (fec_counters(mac, id, m) == 1)
                    rc_fec = 1;
            }
        }
        if (!found)
            warnx("no known protocol map for the selected block(s); "
                  "name the MAC explicitly with --fec-mac N");
        close(fd);
        if (!found)
            return EXIT_FAILURE;
        return rc_fec ? EXIT_FAILURE : EXIT_SUCCESS;
    }

    if (fec_mac > 0) {
        /* WRIOP MEMAC, not a SerDes block -- its own mapping. */
        volatile uint32_t *mac = map_phys_prot(fd, MEMAC_BASE(fec_mac),
                                               MEMAC_MAP_LEN,
                                               PROT_READ | PROT_WRITE);
        if (!mac)
            return EXIT_FAILURE;
        if (m == MODE_HUMAN)
            printf("RS-FEC codeword counters (MDIO device %d via MEMAC 0x%08lx):\n",
                   MDD_RSFEC, (unsigned long)MEMAC_BASE(fec_mac));
        int rc = fec_counters(mac, fec_mac, m);
        close(fd);
        return rc ? EXIT_FAILURE : EXIT_SUCCESS;   /* asked for it, so absence is an error */
    }

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
            print_groups(s, sd, m, want_stages);
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
            print_groups(s, sd, m, want_stages);
    }

    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
