# lx2160-sdx

Read LX2160A SerDes block per-lane status from Linux userland.

## What it is

A single-file, dependency-free tool that walks the LX2160A's three SerDes
blocks (SD1/SD2/SD3) over `/dev/mem`, decodes:

- block reset state (`RSTCTL`),
- PLL F and PLL S lock + reset state (`PLL{F,S}{RSTCTL,CR0,CR1}`),
- per-lane TX and RX state-machine status (`LNaTRSTCTL`, `LNaRRSTCTL`),
- the canonical "is RX healthy?" bit `LNaRRSTCTL[12] CDR_LOCK`,
- TX FIR equalization taps (`LNaTECR0/1`),
- RX equalization adapt status (`LNaRECR2/3/4`),

and prints either a human-readable report, a one-line-per-lane summary
(parseable from shell), or a raw hex dump of every known register.

The Nodebox v3 production RCW (`SRDS_PRTCL_S1=13 / S2=7 / S3=3`) is baked
in as informational lane labels (e.g. "USXGMII13 (LAN8023)" for SD2 lane 6,
"PCIe.5 lane 0" for SD3 lane 0). Edit the `blocks[]` table in
`src/lx2160-sdx.c` if you build a different RCW.

## Build

Native (e.g. on the LX2160A target itself):

```sh
meson setup build
meson compile -C build
```

Cross from an x86_64 host, using the workspace's aarch64 cross-file:

```sh
meson setup build-aarch64 --cross-file ../aarch64-cross.txt
meson compile -C build-aarch64
```

Statically linked (handy for dropping into a busybox initramfs):

```sh
meson setup build --cross-file ../aarch64-cross.txt -Dstatic=true
meson compile -C build
```

The binary lands at `build*/lx2160-sdx`.

## Run

Requires root (or `CAP_SYS_RAWIO`) for `/dev/mem` access.

```sh
./lx2160-sdx                  # full report on all three SerDes blocks
./lx2160-sdx --block 2        # only SD2
./lx2160-sdx --lane 6         # only lane 6 of each reported block
./lx2160-sdx --quiet          # one line per lane, OK/!! markers — easy to grep
./lx2160-sdx --raw            # raw hex dump of every known register
./lx2160-sdx -h               # full usage
```

## Example output

Healthy run on Nodebox v3 with the production RCW (`13 / 7 / 3`), one
LAN8023 active on USXGMII13/14, no carrier-side endpoints on SGMII, no
PCIe devices plugged into PCIE3/4/5/6:

```
$ sudo ./lx2160-sdx --quiet
SD1 LNA  [!!]  CDR_LOCK=0  TX_DIS=0  RX_DIS=0  TX_RST_DONE=1  RX_RST_DONE=1   100GE.1 lane 0
SD1 LNB  [!!]  CDR_LOCK=0  TX_DIS=0  RX_DIS=0  TX_RST_DONE=1  RX_RST_DONE=1   100GE.1 lane 1
SD1 LNC  [!!]  CDR_LOCK=0  TX_DIS=0  RX_DIS=0  TX_RST_DONE=1  RX_RST_DONE=1   100GE.1 lane 2
SD1 LND  [!!]  CDR_LOCK=0  TX_DIS=0  RX_DIS=0  TX_RST_DONE=1  RX_RST_DONE=1   100GE.1 lane 3
SD1 LNE  [!!]  CDR_LOCK=0  TX_DIS=0  RX_DIS=0  TX_RST_DONE=1  RX_RST_DONE=1   100GE.2 lane 0
SD1 LNF  [!!]  CDR_LOCK=0  TX_DIS=0  RX_DIS=0  TX_RST_DONE=1  RX_RST_DONE=1   100GE.2 lane 1
SD1 LNG  [!!]  CDR_LOCK=0  TX_DIS=0  RX_DIS=0  TX_RST_DONE=1  RX_RST_DONE=1   100GE.2 lane 2
SD1 LNH  [!!]  CDR_LOCK=0  TX_DIS=0  RX_DIS=0  TX_RST_DONE=1  RX_RST_DONE=1   100GE.2 lane 3
SD2 LNA  [!!]  CDR_LOCK=0  TX_DIS=0  RX_DIS=0  TX_RST_DONE=1  RX_RST_DONE=1   PCIe.3 x1 (Gen2)
SD2 LNB  [!!]  CDR_LOCK=0  TX_DIS=0  RX_DIS=0  TX_RST_DONE=1  RX_RST_DONE=1   SGMII.12
SD2 LNC  [!!]  CDR_LOCK=0  TX_DIS=0  RX_DIS=0  TX_RST_DONE=1  RX_RST_DONE=1   SGMII.17
SD2 LND  [!!]  CDR_LOCK=0  TX_DIS=0  RX_DIS=0  TX_RST_DONE=1  RX_RST_DONE=1   SGMII.18
SD2 LNE  [!!]  CDR_LOCK=0  TX_DIS=0  RX_DIS=0  TX_RST_DONE=1  RX_RST_DONE=1   PCIe.4 x1 (Gen2)
SD2 LNF  [!!]  CDR_LOCK=0  TX_DIS=0  RX_DIS=0  TX_RST_DONE=1  RX_RST_DONE=1   SGMII.16
SD2 LNG  [OK]  CDR_LOCK=1  TX_DIS=0  RX_DIS=0  TX_RST_DONE=1  RX_RST_DONE=1   USXGMII13 (LAN8023)
SD2 LNH  [OK]  CDR_LOCK=1  TX_DIS=0  RX_DIS=0  TX_RST_DONE=1  RX_RST_DONE=1   USXGMII14 (LAN8023)
SD3 LNA  [!!]  CDR_LOCK=0  TX_DIS=0  RX_DIS=0  TX_RST_DONE=1  RX_RST_DONE=1   PCIe.5 lane 0
…
```

`[!!]` flags every lane whose CDR loop hasn't latched onto an incoming
signal — but **on this carrier** that's the *expected* state for SD1 (no
100GE endpoints), SGMII (no carrier PHYs attached), and PCIe (no
endpoints plugged in). Only USXGMII13/14 → LAN8023 reliably comes up
green out of the box. Cross-reference against
`../doc/lx2160a-serdes-list.md` § "Expected pattern on this board"
before triaging any `[!!]` as a real fault.

The default human-readable mode adds bit-field decoding, equalizer
register dumps, and a final summary line:

```
$ sudo ./lx2160-sdx --block 2 --lane 6

SerDes block SD2  (CCSR base 0x01eb0000)
  SD2 (RCW SRDS_PRTCL_S2=7: PCIe.3/4 + SGMII + USXGMII)

Block-level:
  RSTCTL = 0x80000000

PLL state:
  PLL F   [OK]  RSTCTL=0xc0000000  CR0=0x80000000  CR1=0x00000000
        DIS=no  RST_DONE=yes  RST_ERR=no  LOCK=yes  --> healthy
  PLL S   [OK]  RSTCTL=0xc0000000  CR0=0x80000000  CR1=0x00000000
        DIS=no  RST_DONE=yes  RST_ERR=no  LOCK=yes  --> healthy

Per-lane state:
  Lane 6 (LNG)  [OK]  --  USXGMII13 (LAN8023)
    GCR0    = 0x00000001
    TRSTCTL = 0xc0000000   RST_DONE=1  DIS=0
    RRSTCTL = 0xc0001000   RST_DONE=1  DIS=0  CDR_LOCK=1   --> RX healthy
    TX equ  = TECR0=0x...  TECR1=0x...   (FIR taps; refer to RM §26.4.1.15)
    RX equ  = RECR2=0x...  RECR3=0x...  RECR4=0x...
    RX gen  = RGCR0=0x...  RGCR1=0x...

Summary: all reported lanes healthy.
```
