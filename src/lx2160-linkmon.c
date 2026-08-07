/*
 * lx2160-linkmon — LX2160A Ethernet link-flap monitor with SerDes
 *
 * Run (root required, /dev/mem access):
 *   ./lx2160-linkmon                       # monitor
 *   ./lx2160-linkmon --log /run/linkmon.log
 *   ./lx2160-linkmon --interval-us 200     # hunt fast CDR blips (costs more CPU)
 *   ./lx2160-linkmon --on-event /sbin/capture.sh
 */

#define _DEFAULT_SOURCE
#define _GNU_SOURCE

#include <dirent.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <linux/if.h>
#include <linux/if_link.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>

#define EXIT_USAGE 2

#define SD_MAP_LEN         0x1000UL
static const uint64_t sd_base[4] = { 0, 0x01EA0000UL, 0x01EB0000UL, 0x01EC0000UL };

#define OFF_PLLFRSTCTL     0x400
#define OFF_PLLSRSTCTL     0x500
#define OFF_LANE_GCR0      0x800
#define OFF_LANE_TRSTCTL   0x820
#define OFF_LANE_TECR0     0x830
#define OFF_LANE_RRSTCTL   0x840
#define LANE_STRIDE        0x100
#define NUM_LANES          8
#define NUM_SD             3          /* SD1..SD3 */

/* LNaTRSTCTL / LNaRRSTCTL, per LX2160ARM Rev.1 SS26. */
#define RST_REQ            (1u << 31)
#define RST_DONE           (1u << 30)
#define HLT_REQ            (1u << 27)
#define STP_REQ            (1u << 26)
#define LN_DIS             (1u << 24)
#define CDR_LOCK           (1u << 12)   /* RRSTCTL only */

/* PLLnRSTCTL */
#define PLL_LOCK           (1u << 23)
#define PLL_RST_DONE       (1u << 30)

/*
 * Bits whose change is worth an event. Deliberately excludes the noisy
 * low-order status bits: we want state transitions, not chatter.
 */
#define RRST_WATCH  (RST_REQ | RST_DONE | HLT_REQ | STP_REQ | LN_DIS | CDR_LOCK)
#define TRST_WATCH  (RST_REQ | RST_DONE | HLT_REQ | STP_REQ | LN_DIS)

#define PROTO_SGMII  0x01
#define PROTO_10G    0x0a
#define PROTO_25G    0x1a

static inline uint32_t rd32(const volatile uint32_t *m, uint32_t off)
{
    return *(const volatile uint32_t *)((const volatile uint8_t *)m + off);
}

static inline uint32_t lane_rd(const volatile uint32_t *sd, uint32_t off, int lane)
{
    return rd32(sd, off + (uint32_t)lane * LANE_STRIDE);
}

static inline uint32_t proto_sel(uint32_t gcr0) { return (gcr0 >> 3) & 0x1fu; }

static bool is_eth_proto(uint32_t ps)
{
    return ps == PROTO_SGMII || ps == PROTO_10G || ps == PROTO_25G;
}

struct lane_snap {
    uint32_t gcr0;
    uint32_t trstctl;
    uint32_t rrstctl;
    uint32_t tecr0;
};

struct sd_block {
    volatile uint32_t *map;
    bool     present;
    bool     eth_lane[NUM_LANES];
    struct lane_snap snap[NUM_LANES];
    uint32_t pllf, plls;
};

static struct sd_block blk[NUM_SD + 1];   /* 1-based: blk[1]..blk[3] */

#define RING_SZ 4096

enum ev_kind { EV_LANE, EV_PLL };

struct phy_ev {
    uint64_t t_ns;              /* CLOCK_BOOTTIME, matches dmesg */
    enum ev_kind kind;
    int      sd;
    int      lane;              /* EV_LANE only */
    struct lane_snap old, new;  /* EV_LANE only */
    uint32_t old_pllf, new_pllf, old_plls, new_plls;  /* EV_PLL only */
};

static struct phy_ev ring[RING_SZ];
static unsigned ring_head;      /* next slot to write */
static unsigned ring_count;     /* total ever written */

static uint64_t last_reset_ns[NUM_SD + 1][NUM_LANES];

#define RESET_ATTRIB_HORIZON_MS_DEFAULT  30000
static uint64_t reset_horizon_ns = (uint64_t)RESET_ATTRIB_HORIZON_MS_DEFAULT
                                   * 1000000ull;


static struct phy_ev *ring_push(void)
{
    struct phy_ev *e = &ring[ring_head % RING_SZ];
    ring_head++;
    ring_count++;
    memset(e, 0, sizeof(*e));
    return e;
}

/*
 * dpni -> the SerDes lanes physically behind it.
 *
 * Override with --map dpni.N=SD:lanes , e.g. --map dpni.2=1:0-3
 */
#define MAX_PORTS 16
#define MAX_PORT_LANES 8

struct port {
    int  dpni;
    int  sd;
    int  nlanes;
    int  lane[MAX_PORT_LANES];
    char ifname[IFNAMSIZ];
    bool have_if;
    /* last known netdev state, for edge detection */
    bool carrier;
    bool admin_up;
    bool seen;
    unsigned flaps;              /* carrier edges WE observed via netlink */
    unsigned cc_start;           /* /sys/.../carrier_changes when we primed */
    unsigned cc_offset;          /* known constant skew, latched at first edge */
    bool     cc_offset_set;
};

/*
 * The kernel's own carrier-transition counter.
 *
 */
static unsigned read_carrier_changes(const char *ifname);

static struct port ports[MAX_PORTS];
static int nports;

static const struct port boards_profile[] = {
    { .dpni = 2,  .sd = 1, .nlanes = 4, .lane = { 0, 1, 2, 3 } },  /* dpmac.2 100G CAUI-4 */
    { .dpni = 10, .sd = 1, .nlanes = 1, .lane = { 4 } },           /* LNE -> dpmac.6 */
    { .dpni = 9,  .sd = 1, .nlanes = 1, .lane = { 5 } },           /* LNF -> dpmac.5 */
    { .dpni = 8,  .sd = 1, .nlanes = 1, .lane = { 6 } },           /* LNG -> dpmac.4 */
    { .dpni = 1,  .sd = 1, .nlanes = 1, .lane = { 7 } },           /* LNH -> dpmac.3 */
    { .dpni = 6,  .sd = 2, .nlanes = 1, .lane = { 1 } },           /* SGMII.12 -> dpmac.12 */
    { .dpni = 7,  .sd = 2, .nlanes = 1, .lane = { 3 } },           /* SGMII.18 -> dpmac.18 */
    { .dpni = 5,  .sd = 2, .nlanes = 1, .lane = { 5 } },           /* SGMII.16 -> dpmac.16 */
    { .dpni = 3,  .sd = 2, .nlanes = 1, .lane = { 6 } },           /* USXGMII.13 -> dpmac.13 */
    { .dpni = 4,  .sd = 2, .nlanes = 1, .lane = { 7 } },           /* USXGMII.14 -> dpmac.14 */
};

static FILE *out;
static FILE *logfp;
static bool  color;

static uint64_t now_ns(clockid_t clk)
{
    struct timespec ts;
    clock_gettime(clk, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static void emit(uint64_t t_ns, const char *fmt, ...)
{
    char buf[1024];
    char head[80];
    time_t wall = time(NULL);
    struct tm tm;
    va_list ap;

    localtime_r(&wall, &tm);
    snprintf(head, sizeof head, "[%6" PRIu64 ".%06" PRIu64 "] %02d:%02d:%02d ",
             (uint64_t)(t_ns / 1000000000ull),
             (uint64_t)((t_ns % 1000000000ull) / 1000ull),
             tm.tm_hour, tm.tm_min, tm.tm_sec);

    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);

    fprintf(out, "%s%s\n", head, buf);
    fflush(out);
    if (logfp) {
        fprintf(logfp, "%s%s\n", head, buf);
        fflush(logfp);
    }
}

static void emit_cont(const char *fmt, ...)
{
    char buf[1024];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);

    fprintf(out, "                            %s\n", buf);
    fflush(out);
    if (logfp) {
        fprintf(logfp, "                            %s\n", buf);
        fflush(logfp);
    }
}

static const char *lane_name(int lane) /* LNA..LNH */
{
    static const char *n[NUM_LANES] = { "LNA", "LNB", "LNC", "LND",
                                        "LNE", "LNF", "LNG", "LNH" };
    return (lane >= 0 && lane < NUM_LANES) ? n[lane] : "LN?";
}

/* Render the changed watched bits of a reset-control register as text. */
static int decode_rst(char *dst, size_t n, uint32_t old, uint32_t new, bool rx)
{
    static const struct { uint32_t bit; const char *name; } tbl[] = {
        { RST_REQ,  "RST_REQ"  },
        { RST_DONE, "RST_DONE" },
        { HLT_REQ,  "HLT_REQ"  },
        { STP_REQ,  "STP_REQ"  },
        { LN_DIS,   "DIS"      },
        { CDR_LOCK, "CDR_LOCK" },
    };
    uint32_t diff = old ^ new;
    size_t used = 0;
    unsigned i;

    dst[0] = '\0';
    for (i = 0; i < sizeof tbl / sizeof tbl[0]; i++) {
        if (tbl[i].bit == CDR_LOCK && !rx)
            continue;
        if (!(diff & tbl[i].bit))
            continue;
        used += (size_t)snprintf(dst + used, used < n ? n - used : 0,
                                 "%s%s %d->%d", used ? " " : "", tbl[i].name,
                                 !!(old & tbl[i].bit), !!(new & tbl[i].bit));
    }
    return dst[0] != '\0';
}

enum verdict {
    V_SW_ADMIN,    /* netdev was administratively downed (a script did it) */
    V_PHY_PLL,     /* the block PLL lost lock - every lane behind it dies */
    V_SW_RESET,    /* RST_REQ/HLT_REQ asserted - lane was reconfigured */
    V_PHY_CDR,     /* CDR_LOCK dropped - real signal-integrity / link-partner loss */
    V_MAC_LATCH,   /* carrier dropped with the PHY perfectly healthy */
    V_UNKNOWN,
};

static const char *verdict_str(enum verdict v)
{
    switch (v) {
    case V_SW_ADMIN:  return "SW-ADMIN";
    case V_PHY_PLL:   return "PHY-PLL";
    case V_SW_RESET:  return "SW-RESET";
    case V_PHY_CDR:   return "PHY-CDR";
    case V_MAC_LATCH: return "MAC-LATCH";
    default:          return "UNKNOWN";
    }
}

static const char *verdict_help(enum verdict v)
{
    switch (v) {
    case V_SW_ADMIN:
        return "netdev was downed from userspace (ip link set down) -- self-inflicted, not a fault";
    case V_PHY_PLL:
        return "SerDes block PLL lost lock -- suspect refclk or the block supply, NOT the channel";
    case V_SW_RESET:
        return "a lane reset/halt was requested -- software reconfigured the SerDes";
    case V_PHY_CDR:
        return "the receiver lost CDR lock -- real physical layer: channel, retimer, or link partner";
    case V_MAC_LATCH:
        return "PHY stayed locked and no lane reset is recent (age above) -- MC/PCS "
               "link state machine, NOT signal integrity";
    default:
        return "no PHY activity in the lookback window -- widen --window or lower --interval-us";
    }
}

/* Most recent reset among a port's lanes; 0 if none was ever observed. */
static uint64_t port_last_reset(const struct port *p, int *lane_out)
{
    uint64_t best = 0;
    int i;

    for (i = 0; i < p->nlanes; i++) {
        uint64_t v = last_reset_ns[p->sd][p->lane[i]];
        if (v > best) {
            best = v;
            if (lane_out)
                *lane_out = p->lane[i];
        }
    }
    return best;
}

static bool port_owns_lane(const struct port *p, int sd, int lane)
{
    int i;
    if (p->sd != sd)
        return false;
    for (i = 0; i < p->nlanes; i++)
        if (p->lane[i] == lane)
            return true;
    return false;
}

static enum verdict classify(const struct port *p, uint64_t t_ns, uint64_t window_ns,
                             bool admin_down, char *detail, size_t dn)
{
    unsigned start = ring_count > RING_SZ ? ring_count - RING_SZ : 0;
    unsigned i;
    bool pll = false, sw_reset = false, cdr = false;
    unsigned lanes_cdr = 0, lanes_rst = 0;
    uint32_t cdr_mask = 0, rst_mask = 0;   /* distinct lanes, not event counts */
    uint64_t first_cdr = 0, first_rst = 0, first_pll = 0;
    char lanelist[128] = "";
    int nrelated = 0;

    detail[0] = '\0';

    for (i = ring_count; i > start; i--) {
        const struct phy_ev *e = &ring[(i - 1) % RING_SZ];

        if (e->t_ns + window_ns < t_ns)
            break;
        if (e->t_ns > t_ns)
            continue;              /* strictly a lookback */

        if (e->kind == EV_PLL) {
            if (e->sd != p->sd)
                continue;
            if (((e->old_pllf & PLL_LOCK) && !(e->new_pllf & PLL_LOCK)) ||
                ((e->old_plls & PLL_LOCK) && !(e->new_plls & PLL_LOCK))) {
                pll = true;
                first_pll = e->t_ns;
            }
            continue;
        }

        if (!port_owns_lane(p, e->sd, e->lane))
            continue;
        nrelated++;

        if (((e->old.rrstctl & RST_DONE) && !(e->new.rrstctl & RST_DONE)) ||
            ((e->old.trstctl & RST_DONE) && !(e->new.trstctl & RST_DONE)) ||
            (e->new.rrstctl & (RST_REQ | HLT_REQ | STP_REQ)) ||
            (e->new.trstctl & (RST_REQ | HLT_REQ | STP_REQ))) {
            if (!(rst_mask & (1u << e->lane)))
                lanes_rst++;
            rst_mask |= 1u << e->lane;
            sw_reset = true;
            first_rst = e->t_ns;
        }
        if ((e->old.rrstctl & CDR_LOCK) && !(e->new.rrstctl & CDR_LOCK)) {
            cdr = true;
            first_cdr = e->t_ns;
            if (!(cdr_mask & (1u << e->lane))) {
                lanes_cdr++;
                snprintf(lanelist + strlen(lanelist),
                         sizeof lanelist - strlen(lanelist),
                         "%s%s", lanelist[0] ? "," : "", lane_name(e->lane));
            }
            cdr_mask |= 1u << e->lane;
        }
    }

    if (admin_down) {
        snprintf(detail, dn, "IFF_UP cleared by userspace");
        return V_SW_ADMIN;
    }
    if (pll) {
        snprintf(detail, dn, "SD%d PLL lock lost %" PRIu64 " ms before carrier",
                 p->sd, (uint64_t)((t_ns - first_pll) / 1000000ull));
        return V_PHY_PLL;
    }
    if (sw_reset) {
        snprintf(detail, dn,
                 "%u of %d lane(s) went through a reset (RST_DONE dropped), first %"
                 PRIu64 " ms before carrier%s",
                 lanes_rst, p->nlanes, (uint64_t)((t_ns - first_rst) / 1000000ull),
                 cdr ? "; the CDR loss that followed is a consequence, not a cause" : "");
        return V_SW_RESET;
    }
    {
        int rlane = -1;
        uint64_t rst = port_last_reset(p, &rlane);

        if (rst && rst <= t_ns && (t_ns - rst) <= reset_horizon_ns) {
            snprintf(detail, dn,
                     "no reset inside the %" PRIu64 " ms window, but SD%d %s was reset %"
                     PRIu64 ".%03" PRIu64 " s ago%s -- MC can take >10 s to react, "
                     "so this is attributed to that reset, not to the PHY",
                     (uint64_t)(window_ns / 1000000ull), p->sd, lane_name(rlane),
                     (uint64_t)((t_ns - rst) / 1000000000ull),
                     (uint64_t)(((t_ns - rst) % 1000000000ull) / 1000000ull),
                     cdr ? " (the CDR loss in-window is its consequence)" : "");
            return V_SW_RESET;
        }
    }
    if (cdr) {
        snprintf(detail, dn,
                 "CDR_LOCK lost on %u of %d lane(s) [%s], first %" PRIu64 " ms before carrier -- %s",
                 lanes_cdr, p->nlanes, lanelist,
                 (uint64_t)((t_ns - first_cdr) / 1000000ull),
                 (int)lanes_cdr == p->nlanes && p->nlanes > 1
                     ? "ALL lanes together => common-mode, look upstream of the channel"
                     : "per-lane => that channel/retimer segment");
        return V_PHY_CDR;
    }
    if (nrelated == 0) {
        int rlane = -1;
        uint64_t rst = port_last_reset(p, &rlane);

        /* Always state the basis: "no reset recently" is the whole claim. */
        if (rst)
            snprintf(detail, dn,
                     "no register change on SD%d lane(s) in the last %" PRIu64
                     " ms; nearest lane reset was SD%d %s, %" PRIu64 " s ago",
                     p->sd, (uint64_t)(window_ns / 1000000ull), p->sd,
                     lane_name(rlane), (uint64_t)((t_ns - rst) / 1000000000ull));
        else
            snprintf(detail, dn,
                     "no register change on SD%d lane(s) in the last %" PRIu64
                     " ms, and no lane reset has been observed since start",
                     p->sd, (uint64_t)(window_ns / 1000000ull));
        return V_MAC_LATCH;
    }
    snprintf(detail, dn, "%d PHY event(s) seen but none explains a link loss", nrelated);
    return V_UNKNOWN;
}

/* Print the live PHY state of a port's lanes, so the event block is self-contained. */
static void dump_port_lanes(const struct port *p)
{
    char line[256];
    size_t used = 0;
    int i;

    for (i = 0; i < p->nlanes; i++) {
        const struct lane_snap *s = &blk[p->sd].snap[p->lane[i]];
        used += (size_t)snprintf(line + used, used < sizeof line ? sizeof line - used : 0,
                                 "%s%s:CDR=%d,TXRST_DONE=%d,RXRST_DONE=%d",
                                 used ? " " : "", lane_name(p->lane[i]),
                                 !!(s->rrstctl & CDR_LOCK),
                                 !!(s->trstctl & RST_DONE),
                                 !!(s->rrstctl & RST_DONE));
    }
    emit_cont("now: SD%d %s", p->sd, line);
}

/* Which lanes the hot loop touches, and how often it reads everything  */
static uint8_t mon_lane[NUM_SD + 1];      /* bitmask of lanes to watch per block */
static unsigned slow_div = 64;            /* full read every Nth sample */
static unsigned sample_seq;

static void sample_once(uint64_t t_ns, bool quiet_first)
{
    int s, l;
    bool full = (sample_seq++ % slow_div) == 0;

    for (s = 1; s <= NUM_SD; s++) {
        struct sd_block *b = &blk[s];
        uint32_t pllf, plls;

        if (!b->present || !mon_lane[s])
            continue;

        if (!full)
            goto lanes;

        pllf = rd32(b->map, OFF_PLLFRSTCTL);
        plls = rd32(b->map, OFF_PLLSRSTCTL);
        if (((pllf ^ b->pllf) & (PLL_LOCK | PLL_RST_DONE)) ||
            ((plls ^ b->plls) & (PLL_LOCK | PLL_RST_DONE))) {
            struct phy_ev *e = ring_push();
            e->t_ns = t_ns;
            e->kind = EV_PLL;
            e->sd = s;
            e->old_pllf = b->pllf; e->new_pllf = pllf;
            e->old_plls = b->plls; e->new_plls = plls;
            if (!quiet_first)
                emit(t_ns, "SD%d PLL  PLLF lock %d->%d  PLLS lock %d->%d", s,
                     !!(b->pllf & PLL_LOCK), !!(pllf & PLL_LOCK),
                     !!(b->plls & PLL_LOCK), !!(plls & PLL_LOCK));
            b->pllf = pllf;
            b->plls = plls;
        }

lanes:
        for (l = 0; l < NUM_LANES; l++) {
            struct lane_snap cur;
            struct lane_snap *old = &b->snap[l];
            char what[256];

            if (!(mon_lane[s] & (1u << l)))
                continue;

            /*
             * Fast path: one read. RRSTCTL alone decides whether anything
             * interesting happened; if not, we are done with this lane.
             */
            cur = *old;
            cur.rrstctl = lane_rd(b->map, OFF_LANE_RRSTCTL, l);
            if (!full && !((cur.rrstctl ^ old->rrstctl) & RRST_WATCH)) {
                old->rrstctl = cur.rrstctl;
                continue;
            }

            cur.gcr0    = lane_rd(b->map, OFF_LANE_GCR0, l);
            cur.trstctl = lane_rd(b->map, OFF_LANE_TRSTCTL, l);
            cur.tecr0   = lane_rd(b->map, OFF_LANE_TECR0, l);
            b->eth_lane[l] = is_eth_proto(proto_sel(cur.gcr0));

            if (!((cur.rrstctl ^ old->rrstctl) & RRST_WATCH) &&
                !((cur.trstctl ^ old->trstctl) & TRST_WATCH) &&
                cur.gcr0 == old->gcr0 && cur.tecr0 == old->tecr0) {
                *old = cur;
                continue;
            }

            {
                struct phy_ev *e = ring_push();
                e->t_ns = t_ns;
                e->kind = EV_LANE;
                e->sd = s;
                e->lane = l;
                e->old = *old;
                e->new = cur;
            }

            /* Same signature classify() uses: RST_DONE falling is the durable
             * one; RST_REQ/HLT_REQ only if a sample happened to land inside. */
            if (((old->rrstctl & RST_DONE) && !(cur.rrstctl & RST_DONE)) ||
                ((old->trstctl & RST_DONE) && !(cur.trstctl & RST_DONE)) ||
                (cur.rrstctl & (RST_REQ | HLT_REQ | STP_REQ)) ||
                (cur.trstctl & (RST_REQ | HLT_REQ | STP_REQ)))
                last_reset_ns[s][l] = t_ns;

            if (!quiet_first) {
                char rx[192], tx[192];
                int nrx = decode_rst(rx, sizeof rx, old->rrstctl, cur.rrstctl, true);
                int ntx = decode_rst(tx, sizeof tx, old->trstctl, cur.trstctl, false);

                what[0] = '\0';
                if (nrx)
                    snprintf(what + strlen(what), sizeof what - strlen(what),
                             "RX[%s] ", rx);
                if (ntx)
                    snprintf(what + strlen(what), sizeof what - strlen(what),
                             "TX[%s] ", tx);
                if (cur.gcr0 != old->gcr0)
                    snprintf(what + strlen(what), sizeof what - strlen(what),
                             "GCR0 %08x->%08x ", old->gcr0, cur.gcr0);
                if (cur.tecr0 != old->tecr0)
                    snprintf(what + strlen(what), sizeof what - strlen(what),
                             "TECR0 %08x->%08x (TX FIR rewritten) ",
                             old->tecr0, cur.tecr0);
                emit(t_ns, "SD%d %s  PHY  %s", s, lane_name(l), what);
            }
            *old = cur;
        }
    }
}

static int nl_open(void)
{
    struct sockaddr_nl sa;
    int fd, sz = 1 << 20;

    fd = socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_ROUTE);
    if (fd < 0)
        err(EXIT_FAILURE, "netlink socket");

    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &sz, sizeof sz);

    memset(&sa, 0, sizeof sa);
    sa.nl_family = AF_NETLINK;
    sa.nl_groups = RTMGRP_LINK;
    if (bind(fd, (struct sockaddr *)&sa, sizeof sa) < 0)
        err(EXIT_FAILURE, "netlink bind");
    return fd;
}

static struct port *port_by_ifname(const char *name)
{
    int i;
    for (i = 0; i < nports; i++)
        if (ports[i].have_if && strcmp(ports[i].ifname, name) == 0)
            return &ports[i];
    return NULL;
}

static char *on_event_cmd;
static uint64_t window_ns = 2000ull * 1000000ull;

static void run_hook(const char *ifname, const char *state, const char *verdict)
{
    pid_t pid;

    if (!on_event_cmd)
        return;
    pid = fork();
    if (pid == 0) {
        execl(on_event_cmd, on_event_cmd, ifname, state, verdict, (char *)NULL);
        _exit(127);
    }
    /* SIGCHLD is SIG_IGN, so the child is auto-reaped; never block the poll loop. */
}

static void handle_link_msg(const struct nlmsghdr *nh, uint64_t t_ns)
{
    const struct ifinfomsg *ifi = NLMSG_DATA(nh);
    const struct rtattr *rta;
    int len = (int)(nh->nlmsg_len - NLMSG_LENGTH(sizeof *ifi));
    const char *name = NULL;
    int carrier = -1;
    struct port *p;
    bool up, admin_down;
    char detail[320];
    enum verdict v;

    for (rta = IFLA_RTA(ifi); RTA_OK(rta, len); rta = RTA_NEXT(rta, len)) {
        switch (rta->rta_type) {
        case IFLA_IFNAME:   name = (const char *)RTA_DATA(rta); break;
        case IFLA_CARRIER:  carrier = *(const uint8_t *)RTA_DATA(rta); break;
        default: break;
        }
    }
    if (!name)
        return;
    p = port_by_ifname(name);
    if (!p)
        return;

    up = carrier >= 0 ? carrier != 0 : (ifi->ifi_flags & IFF_LOWER_UP) != 0;
    admin_down = (ifi->ifi_flags & IFF_UP) == 0;

    /* Track the admin edge separately -- it is the SW-ADMIN discriminator. */
    if (admin_down != !p->admin_up) {
        p->admin_up = !admin_down;
        if (admin_down) {
            emit(t_ns, "%-6s (dpni.%d)  ADMIN DOWN            (ip link set down)",
                 name, p->dpni);

            if (p->carrier) {
                p->carrier = false;
                p->flaps++;
                emit(t_ns, "%-6s (dpni.%d)  CARRIER DOWN  (#%u)  verdict=%s",
                     name, p->dpni, p->flaps, verdict_str(V_SW_ADMIN));
                emit_cont("IFF_UP cleared by userspace; kernel suppresses further "
                          "carrier events while admin-down");
                emit_cont("-> %s", verdict_help(V_SW_ADMIN));
                dump_port_lanes(p);
                run_hook(name, "down", verdict_str(V_SW_ADMIN));
            }
        } else {
            emit(t_ns, "%-6s (dpni.%d)  ADMIN UP              (ip link set up)",
                 name, p->dpni);
        }
    }

    if (p->seen && up == p->carrier)
        return;                       /* no carrier edge */

    if (!p->seen) {
        p->seen = true;
        p->carrier = up;
        emit(t_ns, "%-6s (dpni.%d)  baseline carrier=%d", name, p->dpni, up);
        return;
    }

    p->carrier = up;
    p->flaps++;

    {
        unsigned kern = read_carrier_changes(name) - p->cc_start;

        if (!p->cc_offset_set && p->flaps > 0) {
            p->cc_offset = kern > p->flaps ? kern - p->flaps : 0;
            p->cc_offset_set = true;
        } else if (p->cc_offset_set && kern > p->flaps + p->cc_offset) {
            emit(t_ns, "%-6s (dpni.%d)  NOTE: kernel counted %u carrier change(s) "
                 "since start, we observed %u (+%u known skew) -- %u edge(s) "
                 "never reached netlink",
                 name, p->dpni, kern, p->flaps, p->cc_offset,
                 kern - p->flaps - p->cc_offset);
        }
    }

    if (up) {
        emit(t_ns, "%-6s (dpni.%d)  CARRIER UP    (#%u)", name, p->dpni, p->flaps);
        dump_port_lanes(p);
        run_hook(name, "up", "-");
        return;
    }

    v = classify(p, t_ns, window_ns, admin_down, detail, sizeof detail);
    emit(t_ns, "%-6s (dpni.%d)  CARRIER DOWN  (#%u)  verdict=%s",
         name, p->dpni, p->flaps, verdict_str(v));
    emit_cont("%s", detail);
    emit_cont("-> %s", verdict_help(v));
    dump_port_lanes(p);
    run_hook(name, "down", verdict_str(v));
}

static void nl_drain(int fd, uint64_t t_ns)
{
    char buf[16384];
    ssize_t n;

    for (;;) {
        n = recv(fd, buf, sizeof buf, MSG_DONTWAIT);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                return;
            if (errno == EINTR)
                continue;
            if (errno == ENOBUFS) {
                emit(t_ns, "WARNING: netlink receive overrun -- link events were LOST");
                continue;
            }
            warn("netlink recv");
            return;
        }
        if (n == 0)
            return;

        {
            struct nlmsghdr *nh = (struct nlmsghdr *)buf;
            for (; NLMSG_OK(nh, (unsigned)n); nh = NLMSG_NEXT(nh, n)) {
                if (nh->nlmsg_type == NLMSG_DONE)
                    break;
                if (nh->nlmsg_type == RTM_NEWLINK || nh->nlmsg_type == RTM_DELLINK)
                    handle_link_msg(nh, t_ns);
            }
        }
    }
}

static bool resolve_dpni_ifname(int dpni, char *dst, size_t n)
{
    char path[128];
    DIR *d;
    struct dirent *de;
    bool ok = false;

    snprintf(path, sizeof path, "/sys/bus/fsl-mc/devices/dpni.%d/net", dpni);
    d = opendir(path);
    if (!d)
        return false;
    while ((de = readdir(d))) {
        if (de->d_name[0] == '.')
            continue;
        if (strlen(de->d_name) >= n)
            continue;               /* cannot match netlink IFLA_IFNAME anyway */
        memcpy(dst, de->d_name, strlen(de->d_name) + 1);
        ok = true;
        break;
    }
    closedir(d);
    return ok;
}


static bool read_sysfs_net(const char *ifname, const char *attr,
                           char *buf, size_t n)
{
    char path[160];
    int fd;
    ssize_t r;

    snprintf(path, sizeof path, "/sys/class/net/%s/%s", ifname, attr);
    fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0)
        return false;
    r = read(fd, buf, n - 1);
    close(fd);
    if (r <= 0)
        return false;
    buf[r] = '\0';
    return true;
}

static unsigned read_carrier_changes(const char *ifname)
{
    char buf[32];

    if (!read_sysfs_net(ifname, "carrier_changes", buf, sizeof buf))
        return 0;
    return (unsigned)strtoul(buf, NULL, 0);
}

static bool prime_port_state(struct port *p)
{
    char buf[32];

    if (!read_sysfs_net(p->ifname, "flags", buf, sizeof buf))
        return false;
    p->admin_up = (strtoul(buf, NULL, 0) & IFF_UP) != 0;

    /* EINVAL here means admin-down, which implies no carrier. */
    p->carrier = read_sysfs_net(p->ifname, "carrier", buf, sizeof buf)
                 && strtoul(buf, NULL, 0) != 0;

    p->cc_start = read_carrier_changes(p->ifname);
    p->seen = true;
    return true;
}

static volatile uint32_t *map_sd(int fd, int s)
{
    void *m = mmap(NULL, SD_MAP_LEN, PROT_READ, MAP_SHARED, fd,
                   (off_t)sd_base[s]);
    return m == MAP_FAILED ? NULL : (volatile uint32_t *)m;
}

/* --map dpni.2=1:0-3  or  --map dpni.9=1:5 */
static int parse_map(const char *arg)
{
    struct port p;
    const char *q;
    char *end;
    long v;

    memset(&p, 0, sizeof p);
    if (strncmp(arg, "dpni.", 5) != 0)
        return -1;
    v = strtol(arg + 5, &end, 10);
    if (end == arg + 5 || *end != '=')
        return -1;
    p.dpni = (int)v;

    q = end + 1;
    v = strtol(q, &end, 10);
    if (end == q || *end != ':' || v < 1 || v > NUM_SD)
        return -1;
    p.sd = (int)v;

    q = end + 1;
    while (*q) {
        long a, b;
        a = strtol(q, &end, 10);
        if (end == q)
            return -1;
        q = end;
        b = a;
        if (*q == '-') {
            q++;
            b = strtol(q, &end, 10);
            if (end == q)
                return -1;
            q = end;
        }
        if (a < 0 || b >= NUM_LANES || a > b)
            return -1;
        for (; a <= b; a++) {
            if (p.nlanes >= MAX_PORT_LANES)
                return -1;
            p.lane[p.nlanes++] = (int)a;
        }
        if (*q == ',')
            q++;
        else if (*q)
            return -1;
    }
    if (p.nlanes == 0)
        return -1;
    if (nports >= MAX_PORTS)
        return -1;
    ports[nports++] = p;
    return 0;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "Monitor LX2160A Ethernet links and classify every carrier loss against\n"
        "the SerDes lane state that preceded it. Read-only; never writes SerDes.\n"
        "\n"
        "  -i, --interval-us N  lane sampling period in us (default 1000).\n"
        "                       Measured cost on board (13 lanes, one core):\n"
        "                         10000us = 2%%   1000us = 14%%   100us = 98%%\n"
        "                       Use 10000 for an always-on monitor, 1000 to\n"
        "                       hunt, 100 only for a short targeted capture.\n"
        "                       Events shorter than the period can be MISSED:\n"
        "                       a lane reset completes in well under 1 ms and\n"
        "                       is invisible even at 100us. Absence of a PHY\n"
        "                       event is therefore not proof the PHY was idle.\n"
        "  -w, --window-ms N    correlation lookback for a carrier loss (default 2000)\n"
        "      --reset-horizon-ms N  how long a lane reset still counts as the\n"
        "                       likely cause of a later carrier drop (default\n"
        "                       30000, from MC's own \">well over 10 s\" reaction\n"
        "                       time). The age is printed either way, so this\n"
        "                       only moves where SW-RESET stops and MAC-LATCH\n"
        "                       starts - it never hides the evidence.\n"
        "  -l, --log FILE       append every line to FILE as well as stdout\n"
        "  -m, --map SPEC       dpni.N=SD:lanes (repeatable)\n"
        "  -e, --on-event CMD   run CMD <ifname> <up|down> <verdict> on each event\n"
        "  -1, --once           print the baseline snapshot and exit\n"
        "  -h, --help           show this help and exit\n"
        "\n"
        "Verdicts: SW-ADMIN, PHY-PLL, SW-RESET, PHY-CDR, MAC-LATCH, UNKNOWN.\n"
        "MAC-LATCH means every lane stayed locked -- the SerDes is not at fault.\n",
        argv0);
}

static bool stop;
static struct timespec sample_ts;
static void on_sig(int sig) { (void)sig; stop = true; }

int main(int argc, char **argv)
{
    int memfd, nlfd, opt, s, i;
    long interval_us = 1000;
    bool once = false, user_map = false;
    const char *logpath = NULL;
    uint64_t t;

    static const struct option longopts[] = {
        { "interval-us", required_argument, NULL, 'i' },
        { "window-ms",   required_argument, NULL, 'w' },
        { "reset-horizon-ms", required_argument, NULL, 'R' },
        { "log",         required_argument, NULL, 'l' },
        { "map",         required_argument, NULL, 'm' },
        { "on-event",    required_argument, NULL, 'e' },
        { "once",        no_argument,       NULL, '1' },
        { "help",        no_argument,       NULL, 'h' },
        { NULL, 0, NULL, 0 },
    };

    out = stdout;
    color = isatty(STDOUT_FILENO);
    (void)color;

    while ((opt = getopt_long(argc, argv, "i:w:R:l:m:e:1h", longopts, NULL)) != -1) {
        switch (opt) {
        case 'i':
            interval_us = strtol(optarg, NULL, 0);
            if (interval_us < 50 || interval_us > 1000000)
                errx(EXIT_USAGE, "--interval-us out of range (50..1000000)");
            break;
        case 'w':
            window_ns = (uint64_t)strtoll(optarg, NULL, 0) * 1000000ull;
            break;
        case 'R':
            reset_horizon_ns = (uint64_t)strtoll(optarg, NULL, 0) * 1000000ull;
            break;
        case 'l': logpath = optarg; break;
        case 'm':
            if (!user_map) { nports = 0; user_map = true; }
            if (parse_map(optarg) < 0)
                errx(EXIT_USAGE, "bad --map '%s' (want dpni.N=SD:lanes)", optarg);
            break;
        case 'e': on_event_cmd = optarg; break;
        case '1': once = true; break;
        case 'h': usage(argv[0]); return EXIT_SUCCESS;
        default:  usage(argv[0]); return EXIT_USAGE;
        }
    }

    /* XXX */
    if (!user_map) {
        nports = (int)(sizeof boards_profile / sizeof boards_profile[0]);
        memcpy(ports, boards_profile, sizeof boards_profile);
    }

    if (logpath) {
        logfp = fopen(logpath, "ae");
        if (!logfp)
            err(EXIT_FAILURE, "open %s", logpath);
    }

    memfd = open("/dev/mem", O_RDONLY | O_SYNC | O_CLOEXEC);
    if (memfd < 0)
        err(EXIT_FAILURE, "open /dev/mem (need root)");

    for (s = 1; s <= NUM_SD; s++) {
        blk[s].map = map_sd(memfd, s);
        blk[s].present = blk[s].map != NULL;
        if (!blk[s].present)
            warn("mmap SD%d at 0x%08" PRIx64, s, sd_base[s]);
    }

    /* Resolve dpni -> netdev once; names are discovery-ordered, dpni is not. */
    for (i = 0; i < nports; i++) {
        ports[i].have_if = resolve_dpni_ifname(ports[i].dpni, ports[i].ifname,
                                               sizeof ports[i].ifname);
        if (ports[i].have_if)
            prime_port_state(&ports[i]);
    }

    signal(SIGCHLD, SIG_IGN);
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    /* The hot loop only touches lanes some port actually sits on. */
    for (i = 0; i < nports; i++) {
        int j;
        for (j = 0; j < ports[i].nlanes; j++)
            mon_lane[ports[i].sd] |= (uint8_t)(1u << ports[i].lane[j]);
    }

    /* Baseline: prime the snapshots without emitting a wall of fake events. */
    t = now_ns(CLOCK_BOOTTIME);
    sample_once(t, true);
    ring_head = ring_count = 0;

    {
        int nl = 0;
        for (s = 1; s <= NUM_SD; s++)
            for (i = 0; i < NUM_LANES; i++)
                if (mon_lane[s] & (1u << i))
                    nl++;
        emit(t, "lx2160-linkmon start: interval=%ldus window=%" PRIu64 "ms ports=%d lanes=%d",
             interval_us, (uint64_t)(window_ns / 1000000ull), nports, nl);
    }
    for (i = 0; i < nports; i++) {
        const struct port *p = &ports[i];
        char lanes[64] = "";
        int j;

        for (j = 0; j < p->nlanes; j++)
            snprintf(lanes + strlen(lanes), sizeof lanes - strlen(lanes),
                     "%s%s", j ? "," : "", lane_name(p->lane[j]));
        if (!p->have_if) {
            emit_cont("dpni.%-2d  SD%d %-16s  (no netdev -- not instantiated)",
                      p->dpni, p->sd, lanes);
            continue;
        }
        emit_cont("dpni.%-2d  SD%d %-16s  %-6s  carrier=%d admin_up=%d",
                  p->dpni, p->sd, lanes, p->ifname, p->carrier, p->admin_up);
        dump_port_lanes(p);
    }

    if (once)
        return EXIT_SUCCESS;

    nlfd = nl_open();

    {
        struct timespec ts = {
            .tv_sec  = interval_us / 1000000,
            .tv_nsec = (interval_us % 1000000) * 1000,
        };
        sample_ts = ts;
    }

    unsigned reslv_tick = 0;

    while (!stop) {
        struct pollfd pfd = { .fd = nlfd, .events = POLLIN };
        int r = ppoll(&pfd, 1, &sample_ts, NULL);

        if ((reslv_tick++ % 512) == 0) {
            for (i = 0; i < nports; i++) {
                if (ports[i].have_if)
                    continue;
                if (!resolve_dpni_ifname(ports[i].dpni, ports[i].ifname,
                                         sizeof ports[i].ifname))
                    continue;
                ports[i].have_if = true;
                prime_port_state(&ports[i]);
                emit(now_ns(CLOCK_BOOTTIME),
                     "%-6s (dpni.%d)  appeared: carrier=%d admin_up=%d",
                     ports[i].ifname, ports[i].dpni,
                     ports[i].carrier, ports[i].admin_up);
            }
        }

        t = now_ns(CLOCK_BOOTTIME);
        if (r > 0 && (pfd.revents & POLLIN)) {
            sample_once(t, false);
            nl_drain(nlfd, t);
            continue;
        }
        sample_once(t, false);
    }

    emit(now_ns(CLOCK_BOOTTIME), "lx2160-linkmon stop");
    for (i = 0; i < nports; i++)
        if (ports[i].have_if)
        {
            unsigned kern = read_carrier_changes(ports[i].ifname) -
                            ports[i].cc_start;

            unsigned skew = ports[i].cc_offset;

            emit_cont("%-6s (dpni.%-2d) carrier edges: %u observed, %u counted "
                      "by the kernel (+%u known skew)%s",
                      ports[i].ifname, ports[i].dpni, ports[i].flaps, kern, skew,
                      kern > ports[i].flaps + skew
                          ? "  <- DIVERGENCE: edges the kernel saw and we did not"
                          : "");
        }
    return EXIT_SUCCESS;
}
