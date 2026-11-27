/* ka10_sa10.c: Systems Concepts SA-10 subsystem adapter

   Copyright (c) 2026, aap

   Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation
   the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
   THE AUTHOR BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   The SA-10 (Systems Concepts, 1973-75) connects IBM System/360 control
   units to a PDP-10: device 274 on the I/O bus, four selector channels
   ("subchannels") that run channel programs out of PDP-10 memory.  BBN
   TENEX 1.34 (System C) ran its Calcomp CD230 (= IBM 3330-1) disks on
   channel 0 in block multiplexor mode and its STC tapes on channel 1.

   Sources: SA-10 User's Manual (rev. 1 Mar 1975) and Maintenance Manual;
   TENEX 1.34 SSASRV.MAC, DSKPAK.MAC, PROLOG.MAC; BBN's diagnostic
   SA10D.MAC.  The status-type code follows the Maintenance Manual and the
   software (0 ending, 1 initial selection, 2 asynchronous, 3 dummy); the
   User's Manual has 0 and 1 swapped.

   Device SA: the adapter, four channel units (events only), BASE register
   (low-core block, 160 on BBN System C: base+4n = channel n start word,
   +1/+2 = status words, +3 = used by channel).  No control units yet.
*/

#include "kx10_defs.h"

#ifndef NUM_DEVS_SA
#define NUM_DEVS_SA 0
#endif

#if NUM_DEVS_SA > 0

#define SA_DEVNUM   0274
#define SA_NCHAN    4

#define DEBUG_STATUS  0x0001000     /* status stores */
#define DEBUG_BMX     0x0002000     /* block multiplexor device list */

/* CONO bits */
#define CONO_RESET   0400000        /* 18: reset SA-10 */
#define CONO_CME     0200000        /* 19: clear memory errors */
#define CONO_CLOCK   0001000        /* 26: clock control */
#define CONO_FUNC(x) (((x) >> 6) & 7) /* 27-29 */
#define CONO_SET     0000040        /* 30 */
#define CONO_CHAN(x) (((x) >> 3) & 3) /* 31-32 */
#define CONO_DIAG(x) (((x) >> 3) & 7) /* 30-32 with function 0 */
#define CONO_PIA     0000007

/* Per-channel flags, and their CONI positions */
#define F_GO    1
#define F_SREQ  2
#define F_SF    4
#define F_IE    8
#define CONI_CONN  01000000         /* 17: connected */
#define CONI_PIR   0400000          /* 18 */
#define CONI_PAR   0200000          /* 19 */
#define CONI_NXM   0100000          /* 20 */
#define CONI_IE(n) (040000 >> (n))  /* 21-24 */
#define CONI_GO(n) (02000 >> (n))   /* 25-28 */
#define CONI_SF(n) (0100 >> (n))    /* 29-32 */

/* Channel word fields */
#define W_BIT(n)   (1ULL << (35 - (n)))
#define DCW_NMX    W_BIT(0)
#define DCW_MODE1  W_BIT(1)
#define DCW_XCT    W_BIT(2)
#define DCW_CHN    W_BIT(3)
#define DCW_BYTE   W_BIT(4)
#define DCW_PCI    W_BIT(5)
#define DCW_ILE    W_BIT(6)
#define DCW_B7     W_BIT(7)
#define DCW_CMD(w) ((int)(((w) >> 20) & 0377))
#define DCW_DEV(w) ((int)(((w) >> 12) & 0377))
#define DCH_LAST   W_BIT(0)
#define DCH_CNT(w) ((int)(((w) >> 24) & 07777))
#define ADDR(w)    ((t_addr)((w) & RMASK))

/* Status word 1 */
#define S_TYPE(t)  ((uint64)(t) << 34)
#define T_END      0
#define T_INIT     1
#define T_ASYNC    2
#define T_DUMMY    3
#define S_SEL      W_BIT(2)
#define S_BIP      W_BIT(3)
#define S_CTL      W_BIT(4)
#define S_LEN      W_BIT(6)
#define S_PCI      W_BIT(7)

/* IBM unit status byte */
#define US_ATN  0200
#define US_SM   0100
#define US_CUE  0040
#define US_BUSY 0020
#define US_CE   0010
#define US_DE   0004
#define US_UC   0002
#define US_UE   0001
#define US_INIT 01000   /* (internal) status arose at initial selection */

/* Packing modes: DCW bits 1 and 4 */
#define M_WORD    0
#define M_BYTE    1
#define M_NATURAL 2
#define M_TAPE    3

/* BMX device list entry */
#define BMX_STATE(w) ((int)(((w) >> 24) & 017))
#define BMX_DEV(w)   ((int)(((w) >> 28) & 0377))
#define BMX_START    015
#define BMX_WAIT     016
#define BMX_TERM     017
#define BMX_END      000

/* Channel states */
#define CS_IDLE    0
#define CS_RUN     1    /* fetch and execute the word at pc */
#define CS_WAITDE  2    /* selector mode: chained command, waiting for DE */
#define CS_STORE   3    /* status to store, waiting for the status flag to clear */
#define CS_STORED  4    /* status stored, waiting for the status flag to clear */
#define CS_HANG    5    /* memory error */

#define BUFSIZE    16384

struct sa_chan {
    int     flags;      /* F_GO etc. */
    int     state;
    int     next;       /* state after CS_STORE / CS_STORED */
    t_addr  pc;         /* next channel word */
    int     bmx;        /* running in block multiplexor mode */
    t_addr  list;       /* BMX device list */
    t_addr  entry;      /* BMX entry of the device being run */
    int     dev;        /* current device address */
    int     ustat;      /* last unit status */
    int     resid;      /* residual count of the last DCH */
    uint64  w1, w2;     /* status to store */
    int     scan;       /* BMX: look for start entries when idle */
    int     nowait;     /* BMX halt with sign bit: do not wait for the flag */
};

static struct sa_chan sa_ch[SA_NCHAN];
static int sa_pia;
static int sa_merr;             /* CONI_PAR | CONI_NXM */
static int sa_diag;             /* diagnostic word selected for DATAI */
static t_addr sa_errad;         /* address of the memory error */
static int sa_errch;            /* channel of the memory error */
static int32 sa_base = 0160;
static int32 sa_wait = 5;       /* instructions per channel word */
static int32 sa_xfer = 50;      /* instructions per data transfer */
static uint8 sa_buf[BUFSIZE];

static t_stat sa_devio(uint32 dev, uint64 *data);
static t_stat sa_svc(UNIT *);
static t_stat sa_reset(DEVICE *);
static t_stat sa_help(FILE *, DEVICE *, UNIT *, int32, const char *);
static const char *sa_description(DEVICE *);
static void sa_irq(void);
static void sa_kick(int ch, int32 delay);
static int sa_dev_async(int ch, int dev, int stat);
static int sa_idle(int ch);

/* The control units on the channels (at the end of the file). */
static void sa_cu_reset(int ch);
static int sa_cu_async(int ch, int dev, int *devp);
static UNIT *sa_find(int ch, int dev);
static int sa_cu_cmd(UNIT *uptr, int cmd, uint8 *buf, int *len, int write);

UNIT sa_unit[SA_NCHAN] = {
    { UDATA (&sa_svc, UNIT_DIS, 0) },
    { UDATA (&sa_svc, UNIT_DIS, 0) },
    { UDATA (&sa_svc, UNIT_DIS, 0) },
    { UDATA (&sa_svc, UNIT_DIS, 0) },
};

DIB sa_dib = { SA_DEVNUM, 1, &sa_devio, NULL };

MTAB sa_mod[] = {
    { 0 }
};

DEBTAB sa_debug[] = {
    {"IRQ", DEBUG_IRQ, "Interrupt requests"},
    {"CMD", DEBUG_CMD, "Channel words and device commands"},
    {"DATA", DEBUG_DATA, "Data transferred"},
    {"DETAIL", DEBUG_DETAIL, "Details"},
    {"EXP", DEBUG_EXP, "Errors"},
    {"CONI", DEBUG_CONI, "CONI instructions"},
    {"CONO", DEBUG_CONO, "CONO instructions"},
    {"DATAIO", DEBUG_DATAIO, "DATAI and DATAO instructions"},
    {"STATUS", DEBUG_STATUS, "Status stores"},
    {"BMX", DEBUG_BMX, "Block multiplexor device list"},
    {0, 0}
};

REG sa_reg[] = {
    { ORDATAD (BASE, sa_base, 18, "low-core control block address") },
    { DRDATAD (WAIT, sa_wait, 24, "instructions per channel word") },
    { DRDATAD (XFER, sa_xfer, 24, "instructions per data transfer") },
    { ORDATAD (PIA, sa_pia, 3, "priority interrupt assignment") },
    { ORDATAD (MERR, sa_merr, 18, "memory error flags") },
    { ORDATAD (FLAGS0, sa_ch[0].flags, 4, "channel 0 flags: 1 GO, 2 SREQ, 4 SF, 10 IE") },
    { ORDATAD (FLAGS1, sa_ch[1].flags, 4, "channel 1 flags") },
    { ORDATAD (FLAGS2, sa_ch[2].flags, 4, "channel 2 flags") },
    { ORDATAD (FLAGS3, sa_ch[3].flags, 4, "channel 3 flags") },
    { ORDATAD (STATE0, sa_ch[0].state, 4, "channel 0 state") },
    { ORDATAD (STATE1, sa_ch[1].state, 4, "channel 1 state") },
    { ORDATAD (PC0, sa_ch[0].pc, 18, "channel 0 program counter") },
    { ORDATAD (PC1, sa_ch[1].pc, 18, "channel 1 program counter") },
    { 0 }
};

DEVICE sa_dev = {
    "SA", sa_unit, sa_reg, sa_mod,
    SA_NCHAN, 8, 18, 1, 8, 36,
    NULL, NULL, &sa_reset, NULL, NULL, NULL,
    &sa_dib, DEV_DISABLE | DEV_DEBUG | DEV_DIS, 0, sa_debug,
    NULL, NULL, &sa_help, NULL, NULL, &sa_description
};

/* ---- the adapter ---- */

static void sa_irq(void)
{
    int n, req = sa_merr != 0;
    for (n = 0; n < SA_NCHAN; n++)
        if ((sa_ch[n].flags & (F_SF|F_IE)) == (F_SF|F_IE))
            req = 1;
    if (req && sa_pia) {
        sim_debug(DEBUG_IRQ, &sa_dev, "interrupt request, PIA %o\n", sa_pia);
        set_interrupt(SA_DEVNUM, sa_pia);
    } else
        clr_interrupt(SA_DEVNUM);
}

static uint64 sa_coni(void)
{
    uint64 r = CONI_CONN | sa_merr | sa_pia;
    int n;
    for (n = 0; n < SA_NCHAN; n++) {
        struct sa_chan *c = &sa_ch[n];
        if (c->flags & F_IE) r |= CONI_IE(n);
        if (c->flags & F_GO) r |= CONI_GO(n);
        if (c->flags & F_SF) r |= CONI_SF(n);
        if ((c->flags & (F_SF|F_IE)) == (F_SF|F_IE)) r |= CONI_PIR;
    }
    if (sa_merr) r |= CONI_PIR;
    return r;
}

static void sa_kick(int ch, int32 delay)
{
    if (!sim_is_active(&sa_unit[ch]))
        sim_activate(&sa_unit[ch], delay);
}

static void sa_chan_reset(int ch)
{
    struct sa_chan *c = &sa_ch[ch];
    sim_cancel(&sa_unit[ch]);
    memset(c, 0, sizeof *c);
    c->state = CS_IDLE;
    /* System Reset to the control units on this channel. */
    sa_cu_reset(ch);
}

static int sa_rd(int ch, t_addr a, uint64 *w)
{
    if (Mem_read_word(a, w, 0)) {
        sim_debug(DEBUG_EXP, &sa_dev, "ch%d NXM reading %06o\n", ch, a);
        sa_merr |= CONI_NXM; sa_errad = a; sa_errch = ch;
        sa_ch[ch].state = CS_HANG;
        sa_irq();
        return 1;
    }
    return 0;
}

static int sa_wr(int ch, t_addr a, uint64 *w)
{
    if (Mem_write_word(a, w, 0)) {
        sim_debug(DEBUG_EXP, &sa_dev, "ch%d NXM writing %06o\n", ch, a);
        sa_merr |= CONI_NXM; sa_errad = a; sa_errch = ch;
        sa_ch[ch].state = CS_HANG;
        sa_irq();
        return 1;
    }
    return 0;
}

/* Store status (w1, w2) for channel ch, then go to state nxt (CS_IDLE,
   or CS_RUN after a PCI).  If the status flag is still set, wait for it. */
static void sa_status(int ch, uint64 w1, uint64 w2, int nxt)
{
    struct sa_chan *c = &sa_ch[ch];
    c->w1 = w1; c->w2 = w2; c->next = nxt;
    if (c->flags & F_SF) {
        c->state = CS_STORE;
        return;
    }
    if (sa_wr(ch, sa_base + 4 * ch + 1, &w1) || sa_wr(ch, sa_base + 4 * ch + 2, &w2))
        return;
    sim_debug(DEBUG_STATUS, &sa_dev,
              "ch%d status %012llo %012llo (type %d unit %03o dev %03o)\n",
              ch, w1, w2, (int)(w1 >> 34), (int)(w1 >> 20) & 0377,
              (int)(w1 >> 12) & 0377);
    c->flags |= F_SF;
    c->flags &= ~F_SREQ;
    /* After a PCI the channel goes on; otherwise it waits for the flag. */
    c->state = (nxt == CS_RUN || c->nowait) ? nxt : CS_STORED;
    c->nowait = 0;
    sa_irq();
}

static uint64 sa_w1(int type, int ustat, int dev, uint64 flags)
{
    return S_TYPE(type) | flags | ((uint64)(ustat & 0377) << 20) |
           ((uint64)(dev & 0377) << 12);
}

static uint64 sa_w2(int resid, t_addr a)
{
    return ((uint64)(resid & 07777) << 24) | (a & RMASK);
}

/* BMX: rewrite the device list entry of the device being run. */
static void sa_bmx_set(int ch, int state, t_addr a)
{
    struct sa_chan *c = &sa_ch[ch];
    uint64 w;
    if (!c->bmx || c->entry == 0)
        return;
    w = ((uint64)(c->dev & 0377) << 28) | ((uint64)state << 24) | (a & RMASK);
    sim_debug(DEBUG_BMX, &sa_dev, "ch%d entry %06o := %012llo\n", ch, c->entry, w);
    sa_wr(ch, c->entry, &w);
}

/* End the program with status; in BMX mode the entry is terminated. */
static void sa_stop(int ch, uint64 w1, uint64 w2, int wait)
{
    struct sa_chan *c = &sa_ch[ch];
    sa_bmx_set(ch, BMX_TERM, ADDR(w2));
    c->entry = 0;
    if (c->bmx)
        c->scan = 1;
    /* BMX halt with the sign bit: store, but do not wait for the flag. */
    c->nowait = !wait;
    sa_status(ch, w1, w2, CS_IDLE);
}

/* ---- data transfer ---- */

struct seg { t_addr addr; int n; };

static int sa_mode(uint64 dcw)
{
    return ((dcw & DCW_MODE1) ? 2 : 0) | ((dcw & DCW_BYTE) ? 1 : 0);
}

/* Bytes that n units (words, or bytes in BYTE mode) hold. */
static int sa_cap(int mode, int n)
{
    switch (mode) {
    case M_BYTE: return n;
    case M_TAPE: return 5 * n;
    default:     return (9 * n + 1) / 2;
    }
}

/* Units needed for b bytes. */
static int sa_units(int mode, int b)
{
    switch (mode) {
    case M_BYTE: return b;
    case M_TAPE: return (b + 4) / 5;
    default:     return (2 * b + 8) / 9;
    }
}

/* Read the data chain words that follow the DCW at pc. */
static int sa_segs(int ch, t_addr pc, struct seg *s, int max, t_addr *after)
{
    int k = 0;
    uint64 w;
    for (;;) {
        if (sa_rd(ch, pc, &w))
            return -1;
        if (k < max) {
            s[k].addr = ADDR(w);
            s[k].n = 04000 - (DCH_CNT(w) & 03777);
            k++;
        }
        pc++;
        if (w & DCH_LAST)
            break;
        if (k >= max)
            break;
    }
    *after = pc;
    return k;
}

/* Memory -> bytes (write direction).  Returns number of bytes, -1 on NXM. */
static int sa_gather(int ch, int mode, int off, struct seg *s, int ns, uint8 *b, int max)
{
    int i, j, g = 0, wi = 0;       /* g = byte index, wi = word index in stream */
    uint64 w;
    memset(b, 0, max);
    for (i = 0; i < ns; i++) {
        if (mode == M_BYTE) {
            int o = (i == 0) ? off : 0;
            for (j = 0; j < s[i].n && g < max; j++, g++) {
                int p = o + j;
                if ((p & 3) == 0 || j == 0)
                    if (sa_rd(ch, s[i].addr + p / 4, &w))
                        return -1;
                b[g] = (uint8)(w >> (28 - 8 * (p & 3)));
            }
            continue;
        }
        for (j = 0; j < s[i].n; j++, wi++) {
            int base;
            if (sa_rd(ch, s[i].addr + j, &w))
                return -1;
            if (mode == M_TAPE) {
                base = 5 * wi;
                if (base + 5 > max) return g;
                b[base] = (uint8)(w >> 28); b[base+1] = (uint8)(w >> 20);
                b[base+2] = (uint8)(w >> 12); b[base+3] = (uint8)(w >> 4);
                b[base+4] = (uint8)(w & 017);
                continue;
            }
            base = 9 * (wi / 2);
            if (base + 9 > max) return g;
            if ((wi & 1) == 0) {
                b[base] = (uint8)(w >> 28); b[base+1] = (uint8)(w >> 20);
                b[base+2] = (uint8)(w >> 12); b[base+3] = (uint8)(w >> 4);
                b[base+4] = (uint8)((w & 017) << 4);
            } else if (mode == M_WORD) {
                b[base+4] |= (uint8)(w & 017);
                b[base+5] = (uint8)(w >> 28); b[base+6] = (uint8)(w >> 20);
                b[base+7] = (uint8)(w >> 12); b[base+8] = (uint8)(w >> 4);
            } else {                /* NATURAL */
                b[base+4] |= (uint8)(w >> 32);
                b[base+5] = (uint8)(w >> 24); b[base+6] = (uint8)(w >> 16);
                b[base+7] = (uint8)(w >> 8); b[base+8] = (uint8)w;
            }
        }
        g = (mode == M_TAPE) ? 5 * wi : (9 * wi + 1) / 2;
    }
    return g > max ? max : g;
}

/* Bytes -> memory (read direction): store len bytes.  Address 0 = skip. */
static int sa_scatter(int ch, int mode, struct seg *s, int ns, uint8 *b, int len)
{
    int i, j, g = 0, wi = 0;
    uint64 w;
#define BY(k) ((uint64)((k) < len ? b[k] : 0))
    for (i = 0; i < ns && g < len; i++) {
        if (mode == M_BYTE) {
            for (j = 0; j < s[i].n && g < len; j += 4) {
                int m = s[i].n - j < 4 ? s[i].n - j : 4, q;
                w = 0;
                for (q = 0; q < m && g < len; q++, g++)
                    w |= (uint64)b[g] << (28 - 8 * q);
                if (s[i].addr != 0 && sa_wr(ch, s[i].addr + j / 4, &w))
                    return -1;
            }
            continue;
        }
        for (j = 0; j < s[i].n; j++, wi++) {
            int base;
            if (mode == M_TAPE) {
                base = 5 * wi;
                if (base >= len) return 0;
                w = (BY(base) << 28) | (BY(base+1) << 20) | (BY(base+2) << 12) |
                    (BY(base+3) << 4) | (BY(base+4) & 017);
            } else {
                base = 9 * (wi / 2);
                if ((wi & 1) == 0) {
                    if (base >= len) return 0;
                    w = (BY(base) << 28) | (BY(base+1) << 20) | (BY(base+2) << 12) |
                        (BY(base+3) << 4) | (BY(base+4) >> 4);
                } else if (mode == M_WORD) {
                    if (base + 4 >= len) return 0;
                    w = ((BY(base+4) & 017)) | (BY(base+5) << 28) | (BY(base+6) << 20) |
                        (BY(base+7) << 12) | (BY(base+8) << 4);
                } else {
                    if (base + 4 >= len) return 0;
                    w = ((BY(base+4) & 017) << 32) | (BY(base+5) << 24) |
                        (BY(base+6) << 16) | (BY(base+7) << 8) | BY(base+8);
                }
            }
            if (s[i].addr != 0 && sa_wr(ch, s[i].addr + j, &w))
                return -1;
        }
        g = (mode == M_TAPE) ? 5 * wi : (9 * wi + 1) / 2;
    }
#undef BY
    return 0;
}

/* Residual count after b bytes were transferred through segments s. */
static int sa_resid(int mode, struct seg *s, int ns, int b, int *last)
{
    int u = sa_units(mode, b), i;
    for (i = 0; i < ns; i++) {
        if (u < s[i].n || i == ns - 1) {
            *last = i;
            return u >= s[i].n ? 0 : -(s[i].n - u);
        }
        u -= s[i].n;
    }
    *last = 0;
    return 0;
}

static void sa_dump(int ch, const char *what, uint8 *b, int n)
{
    char line[100];
    int i, k = 0;
    if (!sim_deb || !(sa_dev.dctrl & DEBUG_DATA))
        return;
    sim_debug(DEBUG_DATA, &sa_dev, "ch%d %s %d bytes\n", ch, what, n);
    for (i = 0; i < n && i < 64; i++) {
        k += sprintf(line + k, " %02x", b[i]);
        if ((i & 15) == 15 || i == n - 1 || i == 63) {
            sim_debug(DEBUG_DATA, &sa_dev, "  %s\n", line);
            k = 0;
        }
    }
}

/* Execute the device command word at c->pc. */
static void sa_dcw(int ch, uint64 dcw)
{
    struct sa_chan *c = &sa_ch[ch];
    struct seg s[256];
    int ns = 0, mode = sa_mode(dcw), cmd = DCW_CMD(dcw), dev = DCW_DEV(dcw);
    int write = cmd & 1, len, have, st, lenerr = 0, last = 0;
    t_addr after = c->pc + 1;
    UNIT *uptr;

    c->dev = dev;
    sim_debug(DEBUG_CMD, &sa_dev,
              "ch%d %06o: DCW %012llo cmd %02x dev %03o%s%s%s%s%s mode %d\n",
              ch, c->pc, dcw, cmd, dev, (dcw & DCW_NMX) ? " NMX" : "",
              (dcw & DCW_CHN) ? " CHN" : "", (dcw & DCW_PCI) ? " PCI" : "",
              (dcw & DCW_ILE) ? " ILE" : "", (dcw & DCW_B7) ? " B7" : "", mode);
    if ((dcw & DCW_NMX) == 0) {
        ns = sa_segs(ch, c->pc + 1, s, 256, &after);
        if (ns < 0)
            return;
    }
    uptr = sa_find(ch, dev);
    if (uptr == NULL) {
        sim_debug(DEBUG_EXP, &sa_dev, "ch%d select error, dev %03o\n", ch, dev);
        c->ustat = 0;
        sa_stop(ch, sa_w1(T_INIT, 0, dev, S_SEL), sa_w2(0, after), 1);
        return;
    }

    /* The data, both ways, a record at a time. */
    have = 0;
    if (write && ns > 0) {
        int cap = 0, i;
        for (i = 0; i < ns; i++)
            cap += sa_cap(mode, s[i].n);
        have = sa_gather(ch, mode, (mode == M_BYTE && (dcw & DCW_B7)) ? 2 : 0,
                         s, ns, sa_buf, BUFSIZE);
        if (have > cap)
            have = cap;
        if (have < 0)
            return;
        sa_dump(ch, "out", sa_buf, have);
    }
    len = have;
    st = sa_cu_cmd(uptr, cmd, sa_buf, &len, write);
    if (st & US_INIT) {
        /* Rejected at initial selection: no data moved. */
        c->ustat = st & 0377;
        sa_stop(ch, sa_w1(T_INIT, st, dev, 0), sa_w2(ns ? -s[0].n : 0, after), 1);
        return;
    }
    if (write) {
        /* len = bytes the device wanted */
        if (have != len)
            lenerr = 1;
        c->resid = ns ? sa_resid(mode, s, ns, have < len ? have : len, &last) : 0;
    } else {
        int cap = 0, i, n;
        for (i = 0; i < ns; i++)
            cap += sa_cap(mode, s[i].n);
        n = len < cap ? len : cap;
        if (len != cap)
            lenerr = 1;
        sa_dump(ch, "in", sa_buf, len);
        if (ns && sa_scatter(ch, mode, s, ns, sa_buf, n) < 0)
            return;
        c->resid = ns ? sa_resid(mode, s, ns, n, &last) : 0;
    }
    if (ns == 0 && len != 0)
        lenerr = 1;
    c->ustat = st & 0377;
    c->pc = after;
    sim_debug(DEBUG_CMD, &sa_dev, "ch%d   unit status %03o%s resid %d next %06o\n",
              ch, st & 0377, lenerr ? " length error" : "", c->resid, after);

    if (st & (US_ATN|US_CUE|US_BUSY|US_UC|US_UE)) {
        sa_stop(ch, sa_w1(T_END, st, dev, lenerr ? S_LEN : 0),
                sa_w2(c->resid, after), 1);
        return;
    }
    if (lenerr && !(dcw & DCW_ILE)) {
        sa_stop(ch, sa_w1(T_END, st, dev, S_LEN), sa_w2(c->resid, after), 1);
        return;
    }
    if (dcw & DCW_CHN) {
        if (!(st & US_DE)) {
            /* Channel end without device end: the device disconnects. */
            if (c->bmx) {
                sa_bmx_set(ch, BMX_WAIT, after);
                c->entry = 0;
                c->scan = 1;
                c->state = CS_IDLE;
            } else
                c->state = CS_WAITDE;
            if (dcw & DCW_PCI)
                sa_status(ch, sa_w1(T_END, st, dev, S_PCI | (lenerr ? S_LEN : 0)),
                          sa_w2(c->resid, after), c->state);
            return;
        }
        if (st & US_SM)
            c->pc++;
    }
    if (dcw & DCW_PCI)
        sa_status(ch, sa_w1(T_END, st, dev, S_PCI | (lenerr ? S_LEN : 0)),
                  sa_w2(c->resid, after), CS_RUN);
}

/* Fetch and execute channel words until something has to wait. */
static void sa_run(int ch)
{
    struct sa_chan *c = &sa_ch[ch];
    uint64 w;
    int steps;

    for (steps = 0; steps < 64 && c->state == CS_RUN; steps++) {
        if (sa_rd(ch, c->pc, &w))
            return;
        if ((w & DCW_XCT) == 0) {
            if (w & DCW_MODE1) {
                sim_debug(DEBUG_CMD, &sa_dev, "ch%d %06o: TIC %06o\n", ch, c->pc, ADDR(w));
                c->pc = ADDR(w);
                continue;
            }
            sim_debug(DEBUG_CMD, &sa_dev, "ch%d %06o: halt%s\n", ch, c->pc,
                      (w & W_BIT(0)) ? " (no wait)" : "");
            sa_stop(ch, sa_w1(T_END, c->ustat, c->dev, 0),
                    sa_w2(c->resid, c->pc + 1), !(c->bmx && (w & W_BIT(0))));
            return;
        }
        sa_dcw(ch, w);
        if (c->state == CS_RUN || (c->state == CS_IDLE && c->scan)) {
            /* One command per event, so the CPU runs meanwhile; after a
               disconnect the scan of the device list goes on. */
            sa_kick(ch, c->state == CS_RUN ? sa_xfer : sa_wait);
            return;
        }
    }
    if (c->state == CS_RUN)
        sa_kick(ch, sa_wait);
}

/* GO: pick up the start word. */
static void sa_go(int ch)
{
    struct sa_chan *c = &sa_ch[ch];
    uint64 w;
    c->flags &= ~F_GO;
    if (sa_rd(ch, sa_base + 4 * ch, &w))
        return;
    sim_debug(DEBUG_CMD, &sa_dev, "ch%d GO, start word %012llo\n", ch, w);
    c->ustat = 0; c->resid = 0;
    if (w & W_BIT(0)) {
        c->bmx = 1;
        c->list = ADDR(w);
        c->scan = 1;
        c->state = CS_IDLE;
        return;
    }
    c->bmx = 0;
    c->entry = 0;
    c->state = CS_RUN;
    if (w & DCW_XCT)
        c->pc = sa_base + 4 * ch;   /* a command right in the start word */
    else if (w & DCW_MODE1)
        c->pc = ADDR(w);
    else {
        /* Halt as the start word. */
        sa_stop(ch, sa_w1(T_END, 0, 0, 0), sa_w2(0, sa_base + 4 * ch + 1), 1);
    }
}

/* BMX: look for a start entry.  Returns 1 if one was started. */
static int sa_bmx_scan(int ch)
{
    struct sa_chan *c = &sa_ch[ch];
    uint64 w;
    t_addr a;
    int n;
    c->scan = 0;
    for (a = c->list, n = 0; n < 256; a++, n++) {
        if (sa_rd(ch, a, &w))
            return 1;
        if (BMX_STATE(w) == BMX_END)
            break;
        if (BMX_STATE(w) == BMX_START) {
            sim_debug(DEBUG_BMX, &sa_dev, "ch%d start entry %06o: %012llo\n", ch, a, w);
            c->entry = a;
            c->dev = BMX_DEV(w);
            c->pc = ADDR(w);
            c->ustat = 0; c->resid = 0;
            c->state = CS_RUN;
            return 1;
        }
    }
    return 0;
}

/* BMX: device end for a waiting entry? */
static int sa_bmx_resume(int ch, int dev, int stat)
{
    struct sa_chan *c = &sa_ch[ch];
    uint64 w;
    t_addr a;
    int n;
    if (c->list == 0)
        return 0;
    for (a = c->list, n = 0; n < 256; a++, n++) {
        if (sa_rd(ch, a, &w))
            return 1;
        if (BMX_STATE(w) == BMX_END)
            break;
        if (BMX_STATE(w) == BMX_WAIT && BMX_DEV(w) == dev) {
            sim_debug(DEBUG_BMX, &sa_dev, "ch%d resume entry %06o: %012llo\n", ch, a, w);
            c->bmx = 1;
            c->entry = a;
            c->dev = dev;
            c->pc = ADDR(w);
            if (stat & US_SM)
                c->pc++;
            c->ustat = stat;
            c->state = CS_RUN;
            return 1;
        }
    }
    return 0;
}

/* Idle channel: GO, then status request, then asynchronous status, then
   (block multiplexor mode) start entries.  Returns 1 if it did something. */
static int sa_idle(int ch)
{
    struct sa_chan *c = &sa_ch[ch];
    int dev, st;

    if (c->flags & F_SF)
        return 0;
    if (c->flags & F_GO) {
        sa_go(ch);
        if (c->state == CS_IDLE && c->bmx && c->scan)
            sa_bmx_scan(ch);
        return 1;
    }
    if (c->flags & F_SREQ) {
        sa_status(ch, sa_w1(T_DUMMY, 0, 0, 0), 0, CS_IDLE);
        return 1;
    }
    st = sa_cu_async(ch, -1, &dev);
    if (st) {
        sa_dev_async(ch, dev, st);
        return 1;
    }
    if (c->bmx && c->scan) {
        sa_bmx_scan(ch);
        return 1;
    }
    return 0;
}

/* Asynchronous status from a device, channel idle.  Returns 1 if the
   channel has something to do now. */
static int sa_dev_async(int ch, int dev, int st)
{
    struct sa_chan *c = &sa_ch[ch];
    sim_debug(DEBUG_DETAIL, &sa_dev, "ch%d async status %03o from %03o\n", ch, st, dev);
    if (c->state == CS_WAITDE && c->dev == dev) {
        c->ustat = st;
        c->state = CS_RUN;
        if (st & (US_ATN|US_CUE|US_BUSY|US_UC|US_UE)) {
            sa_stop(ch, sa_w1(T_END, st, dev, 0), sa_w2(c->resid, c->pc), 1);
            return 1;
        }
        if (st & US_SM)
            c->pc++;
        return 1;
    }
    if ((st & ~(US_DE|US_SM)) == 0 && (st & US_DE) && sa_bmx_resume(ch, dev, st))
        return 1;
    sa_status(ch, sa_w1(T_ASYNC, st, dev, 0), 0, CS_IDLE);
    return 1;
}

static t_stat sa_svc(UNIT *uptr)
{
    int ch = (int)(uptr - sa_unit);
    struct sa_chan *c = &sa_ch[ch];
    int loops;

    for (loops = 0; loops < 16; loops++) {
        switch (c->state) {
        case CS_HANG:
            return SCPE_OK;
        case CS_STORE:
            if (c->flags & F_SF)
                return SCPE_OK;
            sa_status(ch, c->w1, c->w2, c->next);
            continue;
        case CS_STORED:
            if (c->flags & F_SF)
                return SCPE_OK;
            c->state = c->next;
            continue;
        case CS_WAITDE:
            {
                int dev, st = sa_cu_async(ch, c->dev, &dev);
                if (st) {
                    sa_dev_async(ch, dev, st);
                    continue;
                }
            }
            return SCPE_OK;
        case CS_RUN:
            sa_run(ch);
            return SCPE_OK;
        case CS_IDLE:
            if (!sa_idle(ch))
                return SCPE_OK;
            continue;
        }
    }
    sa_kick(ch, sa_wait);
    return SCPE_OK;
}

static t_stat sa_devio(uint32 dev, uint64 *data)
{
    int ch, fn, set;
    switch (dev & 3) {
    case CONI:
        *data = sa_coni();
        sim_debug(DEBUG_CONI, &sa_dev, "CONI %012llo PC=%06o\n", *data, PC);
        break;
    case CONO:
        sim_debug(DEBUG_CONO, &sa_dev, "CONO %06llo PC=%06o\n", *data & RMASK, PC);
        sa_pia = (int)(*data & CONO_PIA);
        if (*data & CONO_RESET) {
            for (ch = 0; ch < SA_NCHAN; ch++)
                sa_chan_reset(ch);
            sa_merr = 0;
        }
        if (*data & CONO_CME) {
            if (sa_merr)
                for (ch = 0; ch < SA_NCHAN; ch++)
                    if (sa_ch[ch].state == CS_HANG) {
                        /* the erring channel gives up its program */
                        sa_ch[ch].state = CS_IDLE;
                        sa_ch[ch].entry = 0;
                    }
            sa_merr = 0;
        }
        fn = CONO_FUNC(*data);
        set = (*data & CONO_SET) != 0;
        ch = CONO_CHAN(*data);
        switch ((*data & CONO_CLOCK) ? -1 : fn) {
        case 0:
            sa_diag = CONO_DIAG(*data);
            break;
        case 2:     /* with bit 30: HALT I/O (stores no status) */
        case 3:     /* reset channel */
            sim_debug(DEBUG_CMD, &sa_dev, "ch%d %s\n", ch,
                      fn == 2 && set ? "halt I/O" : "reset");
            {
                int ie = sa_ch[ch].flags & F_IE;
                sa_chan_reset(ch);
                sa_ch[ch].flags |= ie;
            }
            break;
        case 4: case 5: case 6: case 7: {
            static const int fl[4] = { F_GO, F_SREQ, F_SF, F_IE };
            int f = fl[fn - 4];
            if (set)
                sa_ch[ch].flags |= f;
            else
                sa_ch[ch].flags &= ~f;
            if (f != F_IE)
                sa_kick(ch, sa_wait);
            break;
        }
        }
        sa_irq();
        break;
    case DATAI:
        switch (sa_diag) {
        case 0: *data = (uint64)(sa_errch & 3) << 18; break;
        case 1: *data = sa_errad; break;
        case 4: *data = 0400000000000LL >> 16 | (sa_coni() & RMASK); break;
        default: *data = 0; break;
        }
        sim_debug(DEBUG_DATAIO, &sa_dev, "DATAI %o: %012llo PC=%06o\n", sa_diag, *data, PC);
        break;
    case DATAO:
        sim_debug(DEBUG_DATAIO, &sa_dev, "DATAO %012llo (ignored) PC=%06o\n", *data, PC);
        break;
    }
    return SCPE_OK;
}

static t_stat sa_reset(DEVICE *dptr)
{
    int ch;
    for (ch = 0; ch < SA_NCHAN; ch++)
        sa_chan_reset(ch);
    sa_merr = 0;
    sa_pia = 0;
    clr_interrupt(SA_DEVNUM);
    return SCPE_OK;
}

static t_stat sa_help(FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, const char *cptr)
{
    fprintf(st, "%s\n\n", sa_description(dptr));
    fprintf(st, "Device 274.  Four channels; the low-core block is at BASE (160):\n");
    fprintf(st, "BASE+4n is channel n's start word, +1 and +2 its status words.\n");
    fprintf(st, "Channels run selector or block-multiplexor programs; WORD, BYTE,\n");
    fprintf(st, "NATURAL and TAPE packing.  Disks: device SAD.\n\n");
    fprint_reg_help(st, dptr);
    return SCPE_OK;
}

static const char *sa_description(DEVICE *dptr)
{
    return "Systems Concepts SA-10 subsystem adapter";
}

/* ---- control unit interface ---- */

/* No control units yet: every device address gives a select error. */
static void sa_cu_reset(int ch)
{
}

static int sa_cu_async(int ch, int dev, int *devp)
{
    return 0;
}

static UNIT *sa_find(int ch, int dev)
{
    return NULL;
}

static int sa_cu_cmd(UNIT *uptr, int cmd, uint8 *buf, int *len, int write)
{
    *len = 0;
    return US_UC|US_INIT;
}

#endif
