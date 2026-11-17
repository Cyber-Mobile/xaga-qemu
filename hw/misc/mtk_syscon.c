/*
 * MediaTek syscon / register-block stub
 *
 * Many MTK peripherals are small register files that the boot chain pokes at
 * during bring-up. Where the exact register semantics are not needed to get
 * further -- because another device already provides the real service -- the
 * register file itself is the honest model: reads return what was last
 * written, writes are accepted, and nothing is invented.
 *
 * The systimer is the first user: the MTK system timer registers as a
 * clockevent device alongside the architected timer, and its shutdown path
 * writes a control register. On real hardware that write lands in the timer
 * block; here it must land somewhere rather than fault, while the architected
 * timer continues to provide timekeeping.
 *
 * This is deliberately NOT a catch-all: instances are created per address,
 * so a peripheral that needs real behaviour is still visible as a missing
 * model rather than silently absorbed.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "hw/core/cpu.h"
#include "trace.h"


#define TYPE_MTK_SYSCON "mtk-syscon"

/*
 * The DRAM PHY "ch_rsv" per-rank completion word (offset 0x70b0) carries
 * three 2-bit fields, one bit per rank: [15:14], [19:18] and [21:20]. Only
 * the rank-0 halves are set, because the emulated part is one 8 GiB rank.
 */
#define MTK_SYSCON_DRAM_PHY_CHAR_RANK_STATUS  0x00144000

OBJECT_DECLARE_SIMPLE_TYPE(MtkSysconState, MTK_SYSCON)

struct MtkSysconState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t *regs;
    uint32_t size;
    char *name;
    /* Bus-protection request masks; see mtk_syscon_bp_decode(). */
    uint32_t bprot[16];
    /* Hardware-voter clock-gate votes / done acks; see mtk_hwv_cg_decode(). */
    uint32_t hwv_vote[256];
    uint32_t hwv_done[256];
};

static bool mtk_infracfg_b4_ctl;
static bool mtk_infracfg_b0_ctl;
static bool mtk_efuse_cmd_seen;

/* MTK_HWVLOG: log hwv/apmixedsys accesses, collapsing repeated reads. */
static void mtk_hwvlog(const char *name, char rw, hwaddr off, uint64_t v)
{
    static int n;
    static hwaddr loff;
    static uint64_t lv;
    static char lrw;
    static const char *lname;
    static unsigned rep;

    if (!name || !getenv("MTK_HWVLOG") ||
        !(strstr(name, "hwv") || strstr(name, "apmixedsys"))) {
        return;
    }
    if (rw == 'R' && lrw == 'R' && name == lname && off == loff && v == lv) {
        rep++;
        return;
    }
    if (rep) {
        fprintf(stderr, "HWV   (x%u repeats)\n", rep);
        rep = 0;
    }
    if (n++ > 3000) {
        return;
    }
    fprintf(stderr, "HWV %c %s+%#x = %#" PRIx64 "\n", rw, name,
            (unsigned)off, v);
    lname = name; loff = off; lv = v; lrw = rw;
}

/*
 * MTK_USBDEVTRACE=1: log firmware accesses to the USB windows
 * (names containing usb/xhci/ippc).  MTK_USBDEVTRACE_MAX caps the output
 * (default 6000 lines).  Used to reverse-engineer LK's mtu3/QMU UDC
 * register contract for fastboot; no effect when the env is unset.
 */
static uint64_t mtk_usbdev_pc(void)
{
    CPUState *cs = qemu_get_cpu(0);

    return cs ? (uint64_t)CPU_GET_CLASS(cs)->get_pc(cs) : 0;
}

static void mtk_usbdevtrace(const char *name, char rw, hwaddr off,
                            unsigned size, uint64_t val)
{
    static int on = -1;
    static int n;
    static int max;

    if (on < 0) {
        const char *m = getenv("MTK_USBDEVTRACE_MAX");

        on = getenv("MTK_USBDEVTRACE") != NULL;
        max = m ? atoi(m) : 6000;
    }
    if (!on || !name ||
        !(strstr(name, "usb") || strstr(name, "xhci") ||
          strstr(name, "ippc"))) {
        return;
    }
    if (n >= max) {
        if (n++ == max) {
            fprintf(stderr, "USBT ... (%d line limit reached)\n", max);
        }
        return;
    }
    n++;
    fprintf(stderr, "USBT %c %s+%#x size=%u val=%#" PRIx64 " pc=%#" PRIx64 "\n",
            rw, name, (unsigned)off, size, val, mtk_usbdev_pc());
}

/* PLLs currently voted on through the HW voter (hwv 0x190/0x194). */
static uint32_t mtk_hwv_pll_on;

/*
 * Bus-protection request/status windows.
 *
 * The vendor mtk-scpsys driver (drivers/soc/mediatek/mtk-scpsys.c) sequences
 * each power-domain transition behind the SoC AXI bus protection: it writes
 * the domain's mask to the SET register and polls the matching status
 * register until the block reports the same mask (before powering a domain
 * down), and writes the CLR register and polls the status until the mask
 * reads back 0 (when releasing protection after powering up).  Both polls
 * time out with -ETIMEDOUT when the status word does not follow the request;
 * the power-on sweep in mtk_register_power_domains() and the
 * genpd_power_off_unused() sweep then report failures that pd_chk_mt6895
 * turns into a BUG().
 *
 * Register layout, from the BUS_PROT_IGN() tables in
 * mtk-scpsys-mt6895.c (type, set_ofs, clr_ofs, en_ofs, sta_ofs):
 *
 *   infracfg_ao : 12 groups of 16 bytes from +0xc10: EN, SET, CLR, STA
 *   vlpcfg      : EN 0x210, SET 0x214, CLR 0x218, STA 0x220
 *   gpu_eb_rpc  : EN/SET 0x40, CLR 0x44, STA 0x48
 *   nemi/semi   : SET 0x84, CLR 0x88, EN/STA 0x8c
 *
 * The status word reports the OR of the masks currently requested, so derive
 * it from SET/CLR writes instead of returning a constant.  MTK_NO_BUSPROT=1
 * falls back to plain storage.
 */
enum {
    MTK_BP_NONE,
    MTK_BP_INFRACFG,
    MTK_BP_VLP,
    MTK_BP_MFGRPC,
    MTK_BP_NEMI,
    MTK_BP_SEMI,
};

static int mtk_syscon_bp_kind(const char *name)
{
    if (!name) {
        return MTK_BP_NONE;
    }
    if (!strcmp(name, "infracfg_ao_clk")) {
        return MTK_BP_INFRACFG;
    }
    if (!strcmp(name, "vlpcfg_bus_clk")) {
        return MTK_BP_VLP;
    }
    if (!strcmp(name, "gpu_eb_rpc_bus_clk")) {
        return MTK_BP_MFGRPC;
    }
    if (!strcmp(name, "nemi_bus")) {
        return MTK_BP_NEMI;
    }
    if (!strcmp(name, "semi_bus")) {
        return MTK_BP_SEMI;
    }
    return MTK_BP_NONE;
}

/* Decode one register offset; false when it is not a bus-protect word. */
static bool mtk_syscon_bp_decode(MtkSysconState *s, hwaddr off,
                                 bool *is_set, bool *is_clr, bool *is_sta,
                                 bool *is_en, int *grp)
{
    *is_set = *is_clr = *is_sta = *is_en = false;
    *grp = 0;

    /*
     * Bus protection belongs to the MTK_PWR_STATE model (see mtk_scpsys.c):
     * enable it together with the real PWR_CON state machine, otherwise the
     * legacy constant-ack overlays in mt6895.c stay in charge.
     */
    if (!getenv("MTK_PWR_STATE")) {
        return false;
    }

    switch (mtk_syscon_bp_kind(s->name)) {
    case MTK_BP_INFRACFG:
        if (off >= 0xc10 && off < 0xcd0) {
            unsigned rel = off - 0xc10;

            *grp = rel / 0x10;
            rel %= 0x10;
            *is_en = rel == 0x0;
            *is_set = rel == 0x4;
            *is_clr = rel == 0x8;
            *is_sta = rel == 0xc;
            return true;
        }
        break;
    case MTK_BP_VLP:
        switch (off) {
        case 0x210: *is_en = true; return true;
        case 0x214: *is_set = true; return true;
        case 0x218: *is_clr = true; return true;
        case 0x220: *is_sta = true; return true;
        default: break;
        }
        break;
    case MTK_BP_MFGRPC:
        switch (off) {
        case 0x40: *is_en = *is_set = true; return true;
        case 0x44: *is_clr = true; return true;
        case 0x48: *is_sta = true; return true;
        default: break;
        }
        break;
    case MTK_BP_NEMI:
    case MTK_BP_SEMI:
        switch (off) {
        case 0x84: *is_set = true; return true;
        case 0x88: *is_clr = true; return true;
        case 0x8c: *is_en = *is_sta = true; return true;
        default: break;
        }
        break;
    default:
        break;
    }
    return false;
}

/*
 * Hardware-voter clock gates (the "hwv" syscon at 0x10320000).
 *
 * The vendor clock framework routes every gated clock and every HWV mux
 * through the hardware voter instead of touching the gate register itself:
 * it writes a SET vote (or a CLR request), waits until the SET register
 * reads back the vote it just wrote, and then waits for the voter's done
 * bit.  mtk_clk_hwv_mux_disable()/__cg_enable_hwv() (clk-mux.c, clk-gate.c)
 * describe the protocol; the register map is in clkchk-mt6895.c
 * (HWV_CG_SET(id) = id*8, HWV_CG_SET_STA = 0x1a00 + id*4,
 * HWV_CG_DONE(id) = 0x1c00 + id*4, HWV_CG_STA/EN at 0x1800/0x1900).
 *
 * A voter that acknowledges votes within microseconds is modelled as a
 * state machine: the SET register reflects the votes software has driven
 * (so a CLR request removes them), and the done word reports the transition
 * as complete once the vote was written.  Without this the SET readback
 * keeps the stale vote and clk-mux.c times out, which is exactly the
 * CLK_EVT_HWV_CG_TIMEOUT that makes clkchk-mt6895.c BUG().
 *
 * MTK_PWR_STATE=1 selects the state machine, matching mtk-scpsys.c.
 */
enum {
    MTK_HWV_CG_SET = 0,
    MTK_HWV_CG_CLR,
    MTK_HWV_CG_STA,
    MTK_HWV_CG_EN,
    MTK_HWV_CG_SET_STA,
    MTK_HWV_CG_CLR_STA,
    MTK_HWV_CG_DONE,
};

static bool mtk_hwv_cg_decode(hwaddr off, int *id, int *kind)
{
    unsigned idv;

    if (off < 0x1000) {
        idv = off / 8;
        if (idv < 256) {
            *id = idv;
            *kind = (off & 4) ? MTK_HWV_CG_CLR : MTK_HWV_CG_SET;
            return true;
        }
        return false;
    }
    if (off >= 0x1800 && off < 0x1d00 && ((off - 0x1800) & 3) == 0) {
        idv = ((off - 0x1800) % 0x100) / 4;
        if (idv < 256) {
            *id = idv;
            *kind = MTK_HWV_CG_STA + (off - 0x1800) / 0x100;
            return true;
        }
    }
    return false;
}

static bool mtk_hwv_cg_active(MtkSysconState *s)
{
    return s->name && !strcmp(s->name, "hwv") && getenv("MTK_PWR_STATE");
}

static uint64_t mtk_syscon_read(void *opaque, hwaddr offset, unsigned size)
{
    {
        MtkSysconState *sp = MTK_SYSCON(opaque);

        if (getenv("MTK_PINCTRL_RD") && sp->name &&
            strstr(sp->name, "pinctrl")) {
            static int np;

            if (np < 60) {
                np++;
                fprintf(stderr, "PIN-RD off=%#x size=%u val=%#x\n",
                        (unsigned)offset, size,
                        sp->regs[offset / 4]);
            }
        }
    }
    {
        MtkSysconState *sd = MTK_SYSCON(opaque);
        static const char *lname;
        static hwaddr loff;
        static unsigned long cnt;

        if (getenv("MTK_SPIN") && sd->name) {
            if (sd->name == lname && offset == loff) {
                cnt++;
                if (cnt == 20000 || cnt == 50000 || cnt == 100000 ||
                    cnt == 200000) {
                    fprintf(stderr, "SPIN %s off=%#x x%lu\n", sd->name,
                            (unsigned)offset, cnt);
                }
            } else {
                lname = sd->name;
                loff = offset;
                cnt = 1;
            }
        }
    }
    {
        MtkSysconState *s0 = MTK_SYSCON(opaque);

        /*
         * infracfg_ao (0x10001000) + 0xb0/0xb4 acknowledge a request the same
         * way the other MTK handshake windows do: the stage writes the
         * control word, then spins until the block reports the transfer as
         * complete.  The window has no engine behind it, so report the ack
         * once the request word has actually been written -- a word that was
         * never driven still reads as not-acknowledged.
         */
        if (s0->name && strstr(s0->name, "infracfg-ao") &&
            (offset == 0xb0 || offset == 0xb4)) {
            /*
             * Control/status pair: the stage programs the mask with writes
             * (which may be cleared again right away) and then polls bit 0 for
             * "operation complete".  The status is maintained by the block,
             * not by the last value written, so latch the acknowledgement once
             * a control word was actually driven.
             */
            if (offset == 0xb4 && mtk_infracfg_b4_ctl) {
                static int n;

                if (getenv("MTK_OFFB4") && n < 6) {
                    n++;
                    fprintf(stderr, "INFRA-B4-ACK %s\n", s0->name);
                }
                return s0->regs[offset / 4] | 1u;
            }
            if (offset == 0xb0 && mtk_infracfg_b0_ctl) {
                return s0->regs[offset / 4] | 1u;
            }
        }
        /*
         * EFUSE controller +0x00: keep it a plain register.  The read-done
         * bit is raised by the command write below and cleared by the next
         * command, so the data words read back unchanged -- an earlier
         * version forced bit 0 on every read, which silently corrupted the
         * OTP words BL2 pulls its key material from.
         */
        /*
         * GPIO input register file.  Tracing the preloader's fastmeta probe
         * shows it reads the pad's mode (0x000), data-out (0x200), pull-enable
         * (0x260, read twice), pull-select and the input level (0x300); one
         * nibble per pin, eight pins per word.
         *
         * The input level of a pad that is configured as an input with its
         * pull-up enabled, and that nothing on the board drives, is high.  This
         * is the physical behaviour that decides the fastmeta strap.
         */
        /*
         * GPIO/pinctrl register file.  Tracing the preloader's fastmeta probe
         * shows it reads 0x000, 0x200, 0x260 (twice) and 0x300; a write probe
         * confirmed 0x300 is the pad MODE word.  Forcing any of the other
         * words high breaks early boot, so they are left as plain storage --
         * the pad input level is not served from this block.
         */
        if (s0->name && strstr(s0->name, "efuse") && offset == 0) {
            return s0->regs[0] | (mtk_efuse_cmd_seen ? 1u : 0u);
        }
        /*
         * EXPERIMENT (env-gated): secure-boot fuse at EFUSE +0x60 bit 1.
         *
         * LK reads 0xFFFF000011F10060 and extracts bit 1 as "sbc_en".  On a
         * production unit the secure-boot fuse is burned, so the bit reads 1
         * and the image path goes through the (fenrir-patched) cert parser
         * instead of the raw signature memcmp.  The model currently returns 0
         * for every OTP word because the fuse map is not known; gate the
         * experiment so it can be evaluated and reverted.
         */
        if (s0->name && strstr(s0->name, "efuse") && offset == 0x60 &&
            getenv("MTK_EFUSE_SBC")) {
            return s0->regs[offset / 4] | 0x2u;
        }
    }
    MtkSysconState *s = MTK_SYSCON(opaque);
    uint64_t val = 0;
    unsigned int i;

    for (i = 0; i < size; i += 4) {
        hwaddr o = offset + i;

        if (o + 4 <= s->size) {
            val |= (uint64_t)s->regs[o / 4] << (8 * i);
        }
    }
    if (size == 4) {
        bool is_set, is_clr, is_sta, is_en;
        int grp;

        if (mtk_syscon_bp_decode(s, offset, &is_set, &is_clr, &is_sta,
                                 &is_en, &grp) && (is_sta || is_en)) {
            val |= s->bprot[grp];
            if (getenv("MTK_BPTRACE") && s->name &&
                !strcmp(s->name, "infracfg_ao_clk") &&
                offset >= 0xc10 && offset < 0xc30) {
                static int nb;
                if (nb++ < 200) {
                    fprintf(stderr, "BPT-R %s off=%#x val=%#" PRIx64
                            " bp=%#x\n", s->name, (unsigned)offset, val,
                            s->bprot[grp]);
                }
            }
        }
    }
    if (size == 4 && mtk_hwv_cg_active(s)) {
        int id, kind;

        if (mtk_hwv_cg_decode(offset, &id, &kind)) {
            switch (kind) {
            case MTK_HWV_CG_SET:
            case MTK_HWV_CG_STA:
            case MTK_HWV_CG_EN:
            case MTK_HWV_CG_SET_STA:
                val = s->hwv_vote[id];
                break;
            case MTK_HWV_CG_CLR:
            case MTK_HWV_CG_CLR_STA:
                val = 0;
                break;
            case MTK_HWV_CG_DONE:
                val = s->hwv_done[id];
                break;
            default:
                break;
            }
        }
    }
    /*
     * DRAM PHY "ch_rsv" per-rank completion word, offset 0x70b0.
     *
     * The preloader's DRAM PHY function polls this register in three
     * sequential loops (0x02015796 >>18, 0x02015a88 >>14, 0x02015b4e >>20);
     * each extracts a 2-bit field, one bit per rank, and waits until both
     * bits are set. A byte-level scan of the image shows these are the
     * only three rank-status extractions in that function, so the word
     * carries three per-rank 2-bit fields at bits [15:14], [19:18] and
     * [21:20].
     *
     * With the register reading zero every loop spins forever: an
     * instrumented run saw 2,955,109 reads at off=0x70b0, all returning
     * 0, before a single character reached the serial port. The
     * preloader's own calibration report already shows every channel
     * passing TA2 (err_cnt:0), so the phases are done and the word
     * reports the rank-0 bits of all three fields: 0x00144000.
     *
     * The register lives in the windows the machine names
     * mt6895.dramc-20..dramc-27; the emulated part is one 8 GiB rank, so
     * only the rank-0 halves of the fields are set.
     */
    if (offset == 0x70b0 && s->name &&
        !strncmp(s->name, "mt6895.dramc-2", 14)) {
        val = MTK_SYSCON_DRAM_PHY_CHAR_RANK_STATUS;
    }
    /*
     * DSI command/status register (window 0x14017000, offset 0x0C).
     *
     * LK's display bring-up polls this register for the "done" bits of the
     * DSI command interface (DSI_PS_Control waits for bit 15 after the
     * sleep-in sequence, DSI_polling() waits for caller-supplied masks).
     * There is no real panel or PHY behind the controller here, so every
     * command completes immediately and the status reads back all ones.
     * Without this the sleep-in loop spins forever printing
     * "polling SLEEPIN_DONE <count>".
     */
    /*
     * MCUPM bring-up handshake (0x1c001000 block, offset 0xeb4): LK writes a
     * command word at +0x000, sets bits 2/3 of +0xeb4 and then polls bits
     * [31:30] until they read 0b11. There is no MCUPM core behind the
     * register block here, so report the handshake as complete.
     */
    if (offset == 0xeb4) {
        static int n;
        if (n < 6) { n++; fprintf(stderr, "MT6895-DBG off 0xeb4 window=%s\n", s->name ? s->name : "(null)"); }
    }
    /*
     * MCUPM bring-up handshake (0x1c001000 block, offset 0xeb4).
     *
     * The preloader writes a command word at +0x000, sets bit 9 of +0xeb4 and
     * then polls bit 13 until it reads 1; a second phase sets bit 6 and polls
     * bit 14.  There is no MCUPM core behind this register block, so report
     * the handshake as complete: bits 30/31 (state reached) plus the two
     * per-phase acknowledge bits the flows spin on.
     *
     * The window is registered as "mt6895.dev-mcupm-cfg" (hyphen), so match
     * that spelling -- the underscore form never matched and the whole
     * special case was dead code.
     */
    if (offset == 0xeb4 && s->name && strstr(s->name, "mcupm-cfg")) {
        val |= 0xc0006000u;
    }
    if (offset == 0x0c && s->name && strstr(s->name, "dev-dsi")) {
        /*
         * Only the two "done" bits are reported: bit 15 is SLEEPIN_DONE
         * (DSI_PS_Control polls it after the sleep-in sequence) and bit 6
         * is SLEEPOUT_DONE (DSI_exit_ULPS polls it). Everything else must
         * read as "not busy", or the WaitForNotBusy loops -- which wait
         * for a busy bit to clear -- hang.
         */
        val |= 0x8040;
    }
    /* report registers that are being polled (same offset read repeatedly) */
    {
        static const char *last_name;
        static hwaddr last_off;
        static int rep, npoll;
        if (s->name && last_name && !strcmp(s->name, last_name) &&
            offset == last_off) {
            rep++;
            if (rep == 40 && npoll < 24) {
                npoll++;
                fprintf(stderr, "MT6895-DBG poll %s off=%#x val=%#x\n",
                        s->name, (unsigned)offset, (unsigned)val);
            }
        } else {
            last_name = s->name;
            last_off = offset;
            rep = 0;
        }
    }
    if (s->name && strstr(s->name, "sej")) {
        static int nsejr;

        if (nsejr < 40) {
            nsejr++;
            fprintf(stderr, "MTK-SEJ-R off=%#x val=%#" PRIx64 "\n",
                    (unsigned)offset, val);
        }
    }
    if (offset == 0x140c && s->name && !strcmp(s->name, "hwv") &&
        !getenv("MTK_NO_HWVDONE")) {
        /*
         * HW-voter PLL "done" word: mtk_hwv_pll_prepare spins on its PLL's
         * bit before voting (univpll timed out at 100us on 0). The voter is
         * never busy here, so every PLL reports done.
         */
        val = 0xffffffffu;
    }
    if (s->name && !strcmp(s->name, "hwv") && !getenv("MTK_NO_HWVPLLEN")) {
        /*
         * HW-voter PLL vote SET (0x190) / CLR (0x194).
         *
         * These two are driven by the voter's vote bitmap, not by the last
         * word written: SET reads back the votes currently registered, CLR
         * reads back the clear requests still pending.  mtk_hwv_pll_prepare
         * writes SET(BIT(bit)) and spins until the bit shows up in the 0x190
         * readback; mtk_hwv_pll_unprepare writes CLR(BIT(bit)) and spins
         * until the bit is gone from the 0x194 readback before it polls done
         * (0x140c) and pending (0x1464).  With plain storage the CLR readback
         * kept the bit set, so the unvote timed out at kernel 291.07 s
         * ("mmpll pll unvote timeout(1010us)(0x80)") and clkchk BUGged in
         * dump_hwv_pll_reg.  The write path maintains mtk_hwv_pll_on; reads
         * only report it.
         */
        if (offset == 0x190) {
            val = mtk_hwv_pll_on;
        } else if (offset == 0x194) {
            val = 0;
        }
    }
    if (s->name && !strcmp(s->name, "apmixedsys_clk") && mtk_hwv_pll_on &&
        !getenv("MTK_NO_HWVPLLEN")) {
        /*
         * HW-voter PLLs (struct mtk_pll_data in clk-mt6895.ko, entries whose
         * hwv set/clr are 0x190/0x194): a PLL voted on through the voter has
         * its CON0 PLL_EN (bit 0) on, as mtk_hwv_pll_is_prepared_done checks.
         */
        static const struct { uint16_t con0; uint8_t bit; } hwv_plls[] = {
            { 0x248, 9 },   /* tvdpll */
            { 0x308, 5 },   /* univpll */
            { 0x370, 13 },  /* imgpll */
            { 0x3a0, 7 },   /* mmpll */
        };
        for (unsigned k = 0; k < ARRAY_SIZE(hwv_plls); k++) {
            if (offset == hwv_plls[k].con0 &&
                (mtk_hwv_pll_on & (1u << hwv_plls[k].bit))) {
                val |= 1;
            }
        }
    }
    if (getenv("MTK_BPTRACE") && size == 4 && offset >= 0xc10 && offset < 0xc30) {
        static int n2r;
        if (n2r++ < 150) {
            fprintf(stderr, "BPT2-R name=%s off=%#x val=%#" PRIx64 "\n",
                    s->name ? s->name : "(null)", (unsigned)offset, val);
        }
    }
    mtk_usbdevtrace(MTK_SYSCON(opaque)->name, 'R', offset, size, val);
    mtk_hwvlog(s->name, 'R', offset, val);
    trace_mtk_syscon_read(s->name, offset, val);
    return val;
}

static void mtk_syscon_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    MtkSysconState *s = MTK_SYSCON(opaque);


    if (s->name && strstr(s->name, "infracfg-ao")) {
        if (offset == 0xb4 && value) {
            mtk_infracfg_b4_ctl = true;
        }
        if (offset == 0xb0 && value) {
            mtk_infracfg_b0_ctl = true;
        }
    }
    if (s->name && strstr(s->name, "efuse") && offset == 0 && value) {
        mtk_efuse_cmd_seen = true;
    }
    if (s->name && strstr(s->name, "efuse") && getenv("MTK_EFUSE")) {
        static int ne;

        if (ne < 30) {
            ne++;
            fprintf(stderr, "EFUSE-W off=%#x val=%#" PRIx64 "\n",
                    (unsigned)offset, value);
        }
    }
    if (s->name && strstr(s->name, "infracfg-ao") &&
        getenv("MTK_INFRA96")) {
        static int nw;

        if (nw < 30) {
            nw++;
            fprintf(stderr, "INFRA96-W off=%#x val=%#" PRIx64 " size=%u\n",
                    (unsigned)offset, value, size);
        }
    }
    if (s->name && strstr(s->name, "adsp_core")) {
        static int nadspw;

        if (nadspw < 40) {
            nadspw++;
            fprintf(stderr, "MTK-ADSPCORE-W %s off=%#x val=%#" PRIx64
                    " size=%u\n", s->name, (unsigned)offset, value, size);
        }
    }
    if (s->name && strstr(s->name, "dev-dsi")) {
        static int ndsiw;
        if (ndsiw < 80) {
            ndsiw++;
            fprintf(stderr, "MT6895-DBG dsi W off=%#x val=%#" PRIx64 "\n",
                    (unsigned)offset, value);
        }
    }
    if (s->name && strstr(s->name, "sej")) {
        static int nsejw;

        if (nsejw < 40) {
            nsejw++;
            fprintf(stderr, "MTK-SEJ-W off=%#x val=%#" PRIx64 "\n",
                    (unsigned)offset, value);
        }
    }
    unsigned int i;

    for (i = 0; i < size; i += 4) {
        hwaddr o = offset + i;

        if (o + 4 <= s->size) {
            s->regs[o / 4] = (value >> (8 * i)) & 0xffffffff;
        }
    }
    if (size == 4) {
        bool is_set, is_clr, is_sta, is_en;
        int grp;

        if (mtk_syscon_bp_decode(s, offset, &is_set, &is_clr, &is_sta,
                                 &is_en, &grp)) {
            if (is_set) {
                s->bprot[grp] |= (uint32_t)value;
            } else if (is_clr) {
                s->bprot[grp] &= ~(uint32_t)value;
            } else if (is_en && !is_sta) {
                s->bprot[grp] = (uint32_t)value;
            }
            if (getenv("MTK_BPTRACE") && s->name &&
                !strcmp(s->name, "infracfg_ao_clk") &&
                offset >= 0xc10 && offset < 0xc30) {
                static int nw;
                if (nw++ < 200) {
                    fprintf(stderr, "BPT-W %s off=%#x val=%#" PRIx64
                            " set=%d clr=%d en=%d sta=%d bp=%#x\n",
                            s->name, (unsigned)offset, value, is_set, is_clr,
                            is_en, is_sta, s->bprot[grp]);
                }
            }
        }
    }
    if (size == 4 && mtk_hwv_cg_active(s)) {
        int id, kind;

        if (mtk_hwv_cg_decode(offset, &id, &kind)) {
            switch (kind) {
            case MTK_HWV_CG_SET:
                s->hwv_vote[id] |= (uint32_t)value;
                s->hwv_done[id] |= (uint32_t)value;
                break;
            case MTK_HWV_CG_CLR:
                s->hwv_vote[id] &= ~(uint32_t)value;
                s->hwv_done[id] |= (uint32_t)value;
                break;
            case MTK_HWV_CG_EN:
                s->hwv_vote[id] = (uint32_t)value;
                s->hwv_done[id] |= (uint32_t)value;
                break;
            default:
                break;
            }
        }
    }
    if (s->name && !strcmp(s->name, "hwv") && !getenv("MTK_NO_HWVPLLEN") &&
        (offset == 0x190 || offset == 0x194) && 0x1468 <= s->size) {
        /*
         * HW-voter PLL vote SET (0x190) / CLR (0x194): the voter turns the
         * PLL on/off at once. mtk_hwv_pll_is_prepared_done (clk-common.ko)
         * wants done (0x140c) set, pending (0x1464) clear and the PLL's
         * CON0 EN bit set; 0x1464 stays 0, CON0 follows mtk_hwv_pll_on.
         */
        if (offset == 0x190) {
            mtk_hwv_pll_on |= (uint32_t)value;
        } else {
            mtk_hwv_pll_on &= ~(uint32_t)value;
        }
    }
    if (getenv("MTK_BPTRACE") && size == 4 && offset >= 0xc10 && offset < 0xc30) {
        static int n2w;
        if (n2w++ < 150) {
            fprintf(stderr, "BPT2-W name=%s off=%#x val=%#" PRIx64 "\n",
                    s->name ? s->name : "(null)", (unsigned)offset, value);
        }
    }
    mtk_usbdevtrace(s->name, 'W', offset, size, value);
    mtk_hwvlog(s->name, 'W', offset, value);
    trace_mtk_syscon_write(s->name, offset, value);
}

static const MemoryRegionOps mtk_syscon_ops = {
    .read = mtk_syscon_read,
    .write = mtk_syscon_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    /*
     * Some firmware stores are 64-bit (for example the pattern writes the
     * thermal block does at 0x190e1700); let the core split them into
     * 4-byte accesses instead of raising an external abort.
     */
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/*
 * Bus-protection state left enabled when the boot chain hands over.
 *
 * The preloader/LK clkmgr flows poll these status bits without always writing
 * the matching SET request first (they are the same bits the legacy
 * constant-ack overlays used to report).  The kernel's
 * clear_bus_protection() writes CLR before it polls, so a power-on transition
 * that owns a mask clears the corresponding preset by itself.
 *
 *   infracfg_ao group 0xc40/0xc50/0xc60: MD protect masks (bit28 / bit9 /
 *   bits 16,17 = MT6895_TOP_AXI_PROT_EN_INFRASYS0_MD / INFRASYS1_MD /
 *   EMISYS0_MD);
 *   vlpcfg: the VLP ack bits 10..13 the clkmgr flow polls at 0x1c00c220.
 */
static void mtk_syscon_bp_presets(MtkSysconState *s)
{
    memset(s->bprot, 0, sizeof(s->bprot));
    if (!getenv("MTK_PWR_STATE") || !s->name) {
        return;
    }
    if (!strcmp(s->name, "infracfg_ao_clk")) {
        s->bprot[3] = 0x10000000u;
        s->bprot[4] = 0x00000200u;
        s->bprot[5] = 0x00030000u;
    } else if (!strcmp(s->name, "vlpcfg_bus_clk")) {
        s->bprot[0] = 0x00003c00u;
    }
}

static void mtk_syscon_reset_hold(Object *obj, ResetType type)
{
    MtkSysconState *s = MTK_SYSCON(obj);

    memset(s->regs, 0, s->size);
    memset(s->hwv_vote, 0, sizeof(s->hwv_vote));
    memset(s->hwv_done, 0, sizeof(s->hwv_done));
    mtk_syscon_bp_presets(s);
}

/*
 * The region and its backing store are built in realize, not instance_init:
 * qdev properties (including "size") are only applied after instance_init
 * has run, so sizing anything there would use the zeroed struct instead of
 * the requested size -- a zero-length region plus a zero-byte allocation
 * that the reset handler then memsets with the real size.
 */
static void mtk_syscon_realize(DeviceState *dev, Error **errp)
{
    MtkSysconState *s = MTK_SYSCON(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &mtk_syscon_ops, s,
                          TYPE_MTK_SYSCON, s->size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    s->regs = g_new0(uint32_t, s->size / 4);
    mtk_syscon_bp_presets(s);
}

static void mtk_syscon_finalize(Object *obj)
{
    MtkSysconState *s = MTK_SYSCON(obj);

    g_free(s->regs);
}

static const Property mtk_syscon_props[] = {
    DEFINE_PROP_UINT32("size", MtkSysconState, size, 0x1000),
    /*
     * The trace events already carry a name; without one every instance
     * reports "(null)", which makes a trace covering several windows
     * impossible to attribute. The machine sets this per window.
     */
    DEFINE_PROP_STRING("name", MtkSysconState, name),
};

static void mtk_syscon_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "MediaTek syscon register block";
    dc->realize = mtk_syscon_realize;
    device_class_set_props(dc, mtk_syscon_props);
    rc->phases.hold = mtk_syscon_reset_hold;
}

static const TypeInfo mtk_syscon_info = {
    .name = TYPE_MTK_SYSCON,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkSysconState),
    .instance_finalize = mtk_syscon_finalize,
    .class_init = mtk_syscon_class_init,
};

static void mtk_syscon_register_types(void)
{
    type_register_static(&mtk_syscon_info);
}

type_init(mtk_syscon_register_types);
