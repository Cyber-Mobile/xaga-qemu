/*
 * MediaTek SPMI PMIC Arbiter (PMIF) host controller
 *
 * The SPMI core (drivers/spmi/spmi.c) and the MTK host controller driver
 * (drivers/spmi/spmi-mtk-pmif.c) are both built into the ROM kernel. The host
 * controller drives the PMIF register block, which in turn speaks to the PMICs
 * sitting on the SPMI bus. Nothing about that bus exists on the QEMU side, so
 * every PMIC register read fails and the whole PMIC stack stays down -- which
 * in turn keeps every consumer of a PMIC regulator (UFS, charger, ...) from
 * probing.
 *
 * This device models the part that is actually specified:
 *
 *   - the PMIF register block, whose software-interface (SWINF) channel state
 *     machine gates every transaction;
 *   - the SPMI master block, used for the arbiter's own commands;
 *   - a flat register space per SPMI slave id, which is what a PMIC presents
 *     over the bus.
 *
 * The channel handshake is taken from the mainline driver (spmi-mtk-pmif.c),
 * so the sequence below is the one the guest actually issues:
 *
 *   read:   poll ch_sta until GET_SWINF(sta) == SWINF_IDLE
 *           write cmd to ch_send
 *           poll ch_sta until GET_SWINF(sta) == SWINF_WFVLDCLR
 *           read ch_rdata
 *           write ch_rdy to clear the valid flag
 *
 *   write:  poll ch_sta until GET_SWINF(sta) == SWINF_IDLE
 *           write data to ch_wdata
 *           write cmd to ch_send
 *
 *   cmd = (opc << 30) | (rw << 29) | (sid << 24) | ((len - 1) << 16) | addr
 *
 * Register offsets are from the same driver; the module in this firmware
 * (spmi-mtk-pmif.ko) carries one table only, identical to the mt6873 one, so
 * MT6895 uses this layout too.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/mtk-env.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/misc/mtk_spmi_pmif.h"
#include "qom/object.h"
#include "ui/input.h"

/*
 * PMIF register block.
 *
 * These are this firmware's offsets, read out of spmi-mtk-pmif.ko's own
 * register table (the module carries one table, and it is not the mt6873 one
 * the mainline driver ships: the SWINF block sits at 0x800 rather than 0xc00).
 * The values were confirmed against the accesses the driver actually makes.
 */
#define PMIF_INIT_DONE          0x0000
#define PMIF_INF_EN             0x0024
#define PMIF_ARB_EN             0x0150
#define PMIF_CMDISSUE_EN        0x03b8
#define PMIF_TIMER_CTRL         0x03e4
#define PMIF_SPI_MODE_CTRL      0x0408
#define PMIF_IRQ_EVENT_EN_0     0x0420
#define PMIF_IRQ_FLAG_0         0x0428
#define PMIF_IRQ_CLR_0          0x042c
#define PMIF_WDT_EVENT_EN_0     0x0474
#define PMIF_WDT_FLAG_0         0x0478

/* Per-channel block, stride 0x40 */
#define PMIF_SWINF_STRIDE       0x40
#define PMIF_SWINF_ACC(n)       (0x0800 + (n) * PMIF_SWINF_STRIDE)
#define PMIF_SWINF_WDATA(n)     (0x0804 + (n) * PMIF_SWINF_STRIDE)
#define PMIF_SWINF_RDATA(n)     (0x0814 + (n) * PMIF_SWINF_STRIDE)
#define PMIF_SWINF_VLD_CLR(n)   (0x0824 + (n) * PMIF_SWINF_STRIDE)
#define PMIF_SWINF_STA(n)       (0x0828 + (n) * PMIF_SWINF_STRIDE)

/* SPMI master block */
#define SPMI_OP_ST_CTRL         0x0000
#define SPMI_GRP_ID_EN          0x0004
#define SPMI_OP_ST_STA          0x0008
#define SPMI_MST_SAMPL          0x000c
#define SPMI_MST_REQ_EN         0x0010
#define SPMI_REC_CTRL           0x0040
#define SPMI_REC0               0x0044
#define SPMI_MST_DBG            0x00fc

/*
 * Channel state as the driver reads it: GET_SWINF() extracts these, so the
 * raw register value carries the state in bits 1..3.
 */
#define SWINF_IDLE_VAL          0x00
#define SWINF_WFVLDCLR_VAL      0x0c

#define PMIF_NUM_CHANS          4
#define PMIF_AP_CHAN            2       /* swinf_ch_start = 0xa = 2 channels in */

/* SPMI command opcodes, already collapsed by the driver */
#define PMIF_CMD_REG            1
#define PMIF_CMD_EXT_REG        2
#define PMIF_CMD_EXT_REG_LONG   3

#define PMIF_CMD_REG_0          0

#define SPMI_MAX_SID            16
#define SPMI_PMIC_REG_SPACE     0x10000     /* 16-bit register address */
#define SPMI_PMIC_CHIP_ID       0x0b        /* MTK_SPMI_PMIC_REG_CHIP_ID */
#define SPMI_PMIC_RCS_IRQ_DONE  0x41b

/* MT6363 chip id, as the driver would read it back */
#define MT6363_CHIP_ID          0x63

/*
 * MT6363 main PMIC interrupt state used by the power key.
 *
 * The key driver (drivers/input/keyboard/mtk-pmic-keys.c) reports the press
 * from the PWRKEY interrupt and the release from PWRKEY_R; it does not sample
 * a register.  The PMIC mfd driver (drivers/mfd/mtk-spmi-pmic.c) walks
 * TOP_INT_STATUS1 -> PSC_TOP_INT_STATUS0/1 and acknowledges by writing the
 * status value back (write-1-to-clear).  The PMIC raises its RCS interrupt
 * line, which on this board is GPIO/EINT pin 243 (spmi@1c804000
 * "rcs_irq" = <&pio 243 IRQ_TYPE_LEVEL_HIGH>).
 */
#define MT6363_PMIC_SID             4
#define MT6363_TOP_INT_STATUS1      0x4e
#define MT6363_PSC_TOP_INT_STATUS0  0x91b
#define MT6363_PSC_TOP_INT_STATUS1  0x91c
#define MT6363_TOP_PSC_BIT          0x04
#define MT6363_PSC_PWRKEY           0x01
#define MT6363_PSC_PWRKEY_R         0x08

/* SPMI master block: per-slave interrupt status, one byte per slave id. */
#define SPMI_SLV_EINT_BASE          0x20
#define SPMI_SLV_EINT_REGS          (SPMI_MAX_SID / 4)

struct MtkSpmiPmifState {
    SysBusDevice parent_obj;

    MemoryRegion pmif;
    MemoryRegion mst;

    uint32_t pmif_regs[0x1000 / 4];
    uint32_t mst_regs[0x100 / 4];

    /* One register space per SPMI slave id: 16-bit address, 8-bit value */
    uint8_t pmic[SPMI_MAX_SID][SPMI_PMIC_REG_SPACE];

    /* SPMI RCS interrupt output (level, to the EINT controller). */
    qemu_irq rcs_irq;
    bool trace;
};

void mtk_spmi_pmif_set_pmic_reg(DeviceState *dev, unsigned sid,
                                unsigned reg, uint8_t val)
{
    MtkSpmiPmifState *s = MTK_SPMI_PMIF(dev);

    if (sid < SPMI_MAX_SID && reg < SPMI_PMIC_REG_SPACE) {
        s->pmic[sid][reg] = val;
    }
}

/*
 * SPMI RCS interrupt line: asserted while any slave has a pending interrupt
 * byte in the SPMI master block. The guest clears a slave's byte by writing
 * the value back (write-1-to-clear).
 */
static void mtk_spmi_pmif_update_rcs(MtkSpmiPmifState *s)
{
    bool level = false;
    unsigned i;

    for (i = 0; i < SPMI_SLV_EINT_REGS; i++) {
        if (s->mst_regs[(SPMI_SLV_EINT_BASE / 4) + i]) {
            level = true;
            break;
        }
    }
    qemu_set_irq(s->rcs_irq, level);
}

/*
 * Emulate one MT6363 power key edge: latch the PSC group status bit for the
 * press or release interrupt and raise the PMIC RCS line.  The PMIC interrupt
 * handler then reads the status registers back through SPMI.
 */
void mtk_spmi_pmif_power_key(DeviceState *dev, bool pressed)
{
    MtkSpmiPmifState *s = MTK_SPMI_PMIF(dev);
    unsigned reg = SPMI_SLV_EINT_BASE + (MT6363_PMIC_SID / 4) * 4;
    unsigned shift = (MT6363_PMIC_SID % 4) * 8;

    s->pmic[MT6363_PMIC_SID][MT6363_TOP_INT_STATUS1] |= MT6363_TOP_PSC_BIT;
    s->pmic[MT6363_PMIC_SID][MT6363_PSC_TOP_INT_STATUS0] |=
        pressed ? MT6363_PSC_PWRKEY : MT6363_PSC_PWRKEY_R;
    s->mst_regs[reg / 4] |= 1u << shift;
    if (s->trace) {
        fprintf(stderr, "MTK-PMIC power key %s: TOP_INT_STATUS1=%#x"
                " PSC_TOP_INT_STATUS0=%#x rcs=%#x\n",
                pressed ? "press" : "release",
                s->pmic[MT6363_PMIC_SID][MT6363_TOP_INT_STATUS1],
                s->pmic[MT6363_PMIC_SID][MT6363_PSC_TOP_INT_STATUS0],
                s->mst_regs[reg / 4]);
    }
    mtk_spmi_pmif_update_rcs(s);
}


static void mtk_spmi_pmif_reset(DeviceState *dev)
{
    MtkSpmiPmifState *s = MTK_SPMI_PMIF(dev);
    int i;

    memset(s->pmif_regs, 0, sizeof(s->pmif_regs));
    memset(s->mst_regs, 0, sizeof(s->mst_regs));
    mtk_spmi_pmif_update_rcs(s);

    /* All channels idle */
    for (i = 0; i < PMIF_NUM_CHANS; i++) {
        s->pmif_regs[PMIF_SWINF_STA(i) / 4] = SWINF_IDLE_VAL;
    }

    /*
     * INIT_DONE reads as 1: the driver only gates on the channel handshake,
     * but a zero here would look like "the arbiter never came out of reset".
     */
    s->pmif_regs[PMIF_INIT_DONE / 4] = 1;

    memset(s->pmic, 0, sizeof(s->pmic));

    /*
     * SPMI calibration check table values (Preloader platform/mt6895/src/drivers/spmi.c):
     * Check 0: slvid 4 (MT6363 / main PMIC), reg 0x09: exp 0x63, mask 0xf0
     * Check 1: slvid 9 (sub PMIC),           reg 0x09: exp 0x85, mask 0xf0
     * Check 2: slvid 6 (PMIC),               reg 0x09: exp 0x15, mask 0xf0
     * Check 3: slvid 7 (PMIC),               reg 0x09: exp 0x15, mask 0xf0
     * Check 4: slvid 5 (MT6368 PMIC),        reg 0x09: exp 0x68, mask 0xf0
     * Check 5: slvid 5 (MT6368 PMIC),        reg 0x240: exp 0x0c, mask 0x0c
     */
    /* Slave 4: MT6363 main PMIC */
    /*
     * MTK_PMIC_VDD2L: give the buck-core register window a plausible voltage
     * so the boot chain's "get vdd2l" step succeeds.  The preloader reads
     * PMIC SID-4 registers 0x1a1b..0x1a4c to obtain the VDD2L setting; with
     * them at zero it reports
     *   "Need reset vdd2l for fist time!" / "get vdd2l fail"
     * and the DRAM description it builds is invalid ("dram info unset"),
     * which makes every boot look cold and loops the DRAM retrain.
     */
    if (getenv("MTK_PMIC_VDD2L")) {
        unsigned r;

        for (r = 0x1a00; r < 0x1b00; r++) {
            s->pmic[4][r] = 0x50;
        }
        for (r = 0x1400; r < 0x1500; r++) {
            s->pmic[4][r] = 0x50;
        }
    }

    s->pmic[4][0x0008] = MT6363_CHIP_ID;
    s->pmic[4][0x0009] = MT6363_CHIP_ID;
    s->pmic[4][0x000a] = 0x00;
    s->pmic[4][0x000b] = MT6363_CHIP_ID;
    s->pmic[4][0x001e] = 0x06;

    /* Slave 9: sub PMIC */
    s->pmic[9][0x0008] = 0x85;
    s->pmic[9][0x0009] = 0x85;
    s->pmic[9][0x000b] = 0x85;

    /* Slave 6 */
    s->pmic[6][0x0008] = 0x15;
    s->pmic[6][0x0009] = 0x15;
    s->pmic[6][0x000b] = 0x15;

    /* Slave 7 */
    s->pmic[7][0x0008] = 0x15;
    s->pmic[7][0x0009] = 0x15;
    s->pmic[7][0x000b] = 0x15;

    /* Slave 5: MT6368 PMIC */
    s->pmic[5][0x0008] = 0x68;
    s->pmic[5][0x0009] = 0x68;
    s->pmic[5][0x000b] = 0x68;
    s->pmic[5][0x0240] = 0x0c;
}

/*
 * Execute one transaction against a PMIC's register space.
 *
 * The driver already normalised the opcode, so what is left is the payload
 * length and the direction. A PMIC register is one byte wide, so a
 * multi-byte access walks consecutive addresses.
 */
/* Ring of recent PMIC register accesses, dumped by the UART on demand. */
#define MTK_SPMI_RING_LEN 64
uint32_t mtk_spmi_ring[MTK_SPMI_RING_LEN];
unsigned mtk_spmi_ring_idx;

static void mtk_spmi_ring_add(uint32_t sid, uint32_t addr, uint32_t val,
                              uint32_t rw)
{
    mtk_spmi_ring[mtk_spmi_ring_idx % MTK_SPMI_RING_LEN] =
        (sid << 24) | (addr << 8) | (rw << 16) | (val & 0xff);
    mtk_spmi_ring_idx++;
}

/*
 * MT6685 (SPMI slave 9) RTC frequency meter (FQMTR).
 *
 * Register usage taken from the preloader's own get_frequency_meter()
 * (LUA_1 VA 0x0203e1ac) and its callers:
 *   0x0111[2:0]  RG_FQMTR_CKSEL  reference ("fixed") clock
 *   0x0522 bit6  FQMTR reset pulse
 *   0x0546[2:0]  RG_FQMTR_TCKSEL clock under test
 *   0x0546 bit3  FQMTR_BUSY      (guest polls it, 1000 x 1 us)
 *   0x0546 bit4  DCXO26M_EN
 *   0x0547 bit7  FQMTR_EN        (CON0 bit15)
 *   0x0548/9     FQMTR_WINSET    window, in reference-clock cycles - 1
 *   0x054a/b     FQMTR_DATA      result
 *   0x05ae[4:0]  RTC_OSC32CON    EOSC trim written by eosc_cali()
 * The count is the number of test-clock edges inside a window of
 * (WINSET + 1) reference-clock cycles:
 *     DATA = f(TCKSEL) * (WINSET + 1) / f(CKSEL)
 * which is the only reading consistent with all four acceptance windows
 * the preloader checks (eosc check: 3..7, 3..8, 301..10403; eosc_cali
 * target 0x318..0x31c = 26 MHz / 32.768 kHz).  The measurement completes
 * before the guest's first BUSY poll.  MTK_NO_FQMTR=1 restores the old
 * "zero CON0 on every write" behaviour (DATA stays 0).
 */
#define MT6685_SID              9
#define MT6685_FQMTR_CKSEL      0x0111
#define MT6685_FQMTR_RST        0x0522
#define MT6685_FQMTR_CON0       0x0546
#define MT6685_FQMTR_CON0_H     0x0547
#define MT6685_FQMTR_WINSET     0x0548
#define MT6685_FQMTR_DATA       0x054a
#define MT6685_RTC_OSC32CON     0x05ae

static bool mt6685_fqmtr_on(void)
{
    static int on = -1;

    if (on < 0) {
        on = getenv("MTK_NO_FQMTR") == NULL;
    }
    return on;
}

/*
 * EOSC (internal 32 kHz RC oscillator).  No datasheet slope is public;
 * assume code 16 is nominal 32.768 kHz and each trim LSB moves it by
 * ~0.4 %, higher code = slower (the direction eosc_cali()'s binary
 * search assumes: count > 0x31c => lower the code).
 */
static double mt6685_eosc_hz(const uint8_t *r)
{
    unsigned cal = r[MT6685_RTC_OSC32CON] & 0x1f;

    return 32768.0 * (1.0 + 0.004 * (16.0 - (double)cal));
}

static double mt6685_fqmtr_tck_hz(const uint8_t *r, unsigned sel)
{
    switch (sel) {
    case 0: return 26000000.0;            /* FQM26M_CK */
    case 1: return 32768.0;               /* XOSC32_CK (DCXO 32k in 32k-less) */
    case 2: return 26000000.0 / 793.4570; /* DCXO_F32K_CK */
    case 3: return mt6685_eosc_hz(r);     /* EOSC32_CK */
    case 4: return 26000000.0;            /* XOSCDET_CK (26 MHz domain) */
    default: return 0.0;
    }
}

static double mt6685_fqmtr_fix_hz(const uint8_t *r, unsigned sel)
{
    switch (sel) {
    case 0: return 26000000.0;            /* FIX_CLK_26M */
    case 1: return 32768.0;               /* FIX_CLK_XOSC_32K_DET */
    case 2: return mt6685_eosc_hz(r);     /* FIX_CLK_EOSC_32K */
    case 3: return 32768.0;               /* FIX_CLK_RTC_32K */
    case 5: return 1.0;                   /* FIX_CLK_TCK_SEC */
    case 6: return 75000.0;               /* FIX_CLK_PMU_75K */
    default: return 0.0;
    }
}

static void mt6685_fqmtr_write(MtkSpmiPmifState *s, unsigned addr,
                               unsigned len, uint8_t en_before)
{
    uint8_t *r = s->pmic[MT6685_SID];

    if (addr <= MT6685_FQMTR_RST && MT6685_FQMTR_RST < addr + len &&
        (r[MT6685_FQMTR_RST] & 0x40)) {
        r[MT6685_FQMTR_DATA] = 0;
        r[MT6685_FQMTR_DATA + 1] = 0;
    }
    if (addr <= MT6685_FQMTR_CON0_H && MT6685_FQMTR_CON0_H < addr + len &&
        (r[MT6685_FQMTR_CON0_H] & 0x80) && !(en_before & 0x80)) {
        unsigned tsel = r[MT6685_FQMTR_CON0] & 7;
        unsigned fsel = r[MT6685_FQMTR_CKSEL] & 7;
        unsigned win = r[MT6685_FQMTR_WINSET] |
                       (r[MT6685_FQMTR_WINSET + 1] << 8);
        double tck = mt6685_fqmtr_tck_hz(r, tsel);
        double fix = mt6685_fqmtr_fix_hz(r, fsel);
        double cnt = fix > 0.0 ? tck * (win + 1) / fix : 0.0;
        unsigned data = cnt >= 65535.0 ? 0xffff : (unsigned)cnt;

        r[MT6685_FQMTR_DATA] = data & 0xff;
        r[MT6685_FQMTR_DATA + 1] = data >> 8;
        if (getenv("MTK_FQMTR_LOG")) {
            fprintf(stderr, "MTK-FQMTR tcksel=%u cksel=%u win=%u cal=%u"
                    " -> %u\n", tsel, fsel, win,
                    r[MT6685_RTC_OSC32CON] & 0x1f, data);
        }
    }
    /* BUSY: the measurement is over before the guest's first poll */
    r[MT6685_FQMTR_CON0] &= ~0x08;
}

static void mtk_spmi_pmif_xfer(MtkSpmiPmifState *s, unsigned chan, uint32_t cmd)
{
    unsigned rw = (cmd >> 29) & 0x1;
    unsigned sid = (cmd >> 24) & 0xf;
    unsigned len = ((cmd >> 16) & 0xff) + 1;
    unsigned addr = cmd & 0xffff;
    unsigned i;

    if (MTK_ENV_CACHED("MTK_SPMITRACE") && !rw && sid == 4) {
        static int nr;

        if (nr < 260) {
            nr++;
            fprintf(stderr, "MTK-SPMIRD addr=%#x len=%u val=%#x\n",
                    addr, len, s->pmic[sid][addr]);
        }
    }
    if (sid >= SPMI_MAX_SID || len > 4 || addr + len > SPMI_PMIC_REG_SPACE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad transaction cmd=%#x (sid=%u len=%u addr=%#x)\n",
                      TYPE_MTK_SPMI_PMIF, cmd, sid, len, addr);
        s->pmif_regs[PMIF_SWINF_STA(chan) / 4] = SWINF_IDLE_VAL;
        return;
    }

    if (rw) {
        uint32_t data = s->pmif_regs[PMIF_SWINF_WDATA(chan) / 4];
        uint8_t fq_en_before = s->pmic[sid][MT6685_FQMTR_CON0_H];

        for (i = 0; i < len; i++) {
            s->pmic[sid][addr + i] = (data >> (8 * i)) & 0xff;
            mtk_spmi_ring_add(sid, addr + i, (data >> (8 * i)) & 0xff, 0);
        }

        /* RTC auto-clearing trigger / busy bits */
        if (sid == 9) {
            s->pmic[9][0x0588] &= ~0x78; /* clear bits 3,4,5,6 */
            /*
             * RTC_BBPU bit2 (RESET_SPAR) is a self-clearing request like
             * bit3 (RESET_ALARM): rtc_hw_init() (LUA_1 0x0203e4ac) and the
             * "spar/alarm clear" step write 0x430c/0x4320 and wait up to
             * 500 ms for both bits to drop.  MTK_NO_RTCRSTSPAR=1 keeps it set.
             */
            if (!MTK_ENV_CACHED("MTK_NO_RTCRSTSPAR")) {
                s->pmic[9][0x0588] &= ~0x04;
            }
            if (mt6685_fqmtr_on()) {
                mt6685_fqmtr_write(s, addr, len, fq_en_before);
            } else {
                s->pmic[9][0x0546] = 0;
                s->pmic[9][0x0547] = 0;
            }
            s->pmic[9][0x05c2] = 0;
        }

        /*
         * MT6363 status registers are write-1-to-clear (the PMIC interrupt
         * handler acknowledges a group by writing the status value back).
         * The top level PSC bit mirrors the group status.
         */
        if (sid == MT6363_PMIC_SID) {
            for (i = 0; i < len; i++) {
                unsigned r = addr + i;

                if (r == MT6363_TOP_INT_STATUS1 ||
                    r == MT6363_PSC_TOP_INT_STATUS0 ||
                    r == MT6363_PSC_TOP_INT_STATUS1) {
                    s->pmic[sid][r] &= ~((data >> (8 * i)) & 0xff);
                }
            }
            if (!s->pmic[sid][MT6363_PSC_TOP_INT_STATUS0] &&
                !s->pmic[sid][MT6363_PSC_TOP_INT_STATUS1]) {
                s->pmic[sid][MT6363_TOP_INT_STATUS1] &= ~MT6363_TOP_PSC_BIT;
            }
        }

        /*
         * A write has no payload for the guest to collect, so the channel is
         * done as soon as the command is accepted. Only a read parks in
         * WFVLDCLR, waiting for the guest to clear the valid flag.
         */
        s->pmif_regs[PMIF_SWINF_STA(chan) / 4] = SWINF_IDLE_VAL;
    } else {
        uint32_t data = 0;

        for (i = 0; i < len; i++) {
            data |= (uint32_t)s->pmic[sid][addr + i] << (8 * i);
            mtk_spmi_ring_add(sid, addr + i, s->pmic[sid][addr + i], 1);
        }
        s->pmif_regs[PMIF_SWINF_RDATA(chan) / 4] = data;
        s->pmif_regs[PMIF_SWINF_STA(chan) / 4] = SWINF_WFVLDCLR_VAL;
    }
}

static uint64_t mtk_spmi_pmif_read(void *opaque, hwaddr offset, unsigned size)
{
    MtkSpmiPmifState *s = MTK_SPMI_PMIF(opaque);

    if (offset + 4 > sizeof(s->pmif_regs)) {
        return 0;
    }

    /*
     * Keep the per-access UNIMP log opt-in: it fires tens of thousands of
     * times during boot and drowns -d unimp, which is what the UFS bring-up
     * investigation needs to see.
     */
    if (MTK_ENV_CACHED("MTK_SPMI_LOG")) {
        qemu_log_mask(LOG_UNIMP, "%s: read  off=%#x -> %#x\n",
                      TYPE_MTK_SPMI_PMIF, (unsigned)offset,
                      s->pmif_regs[offset / 4]);
    }

    return s->pmif_regs[offset / 4];
}

static void mtk_spmi_pmif_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    MtkSpmiPmifState *s = MTK_SPMI_PMIF(opaque);
    unsigned chan;

    if (offset + 4 > sizeof(s->pmif_regs)) {
        return;
    }

    if (MTK_ENV_CACHED("MTK_SPMI_LOG")) {
        qemu_log_mask(LOG_UNIMP, "%s: write off=%#x val=%#x\n",
                  TYPE_MTK_SPMI_PMIF, (unsigned)offset, (unsigned)value);
    }

    for (chan = 0; chan < PMIF_NUM_CHANS; chan++) {
        if (offset == PMIF_SWINF_ACC(chan)) {
            s->pmif_regs[offset / 4] = value;
            mtk_spmi_pmif_xfer(s, chan, value);
            return;
        }
        if (offset == PMIF_SWINF_VLD_CLR(chan)) {
            /* Guest acknowledges the payload: channel returns to idle */
            s->pmif_regs[PMIF_SWINF_STA(chan) / 4] = SWINF_IDLE_VAL;
            return;
        }
    }

    s->pmif_regs[offset / 4] = value;
}

static const MemoryRegionOps mtk_spmi_pmif_ops = {
    .read = mtk_spmi_pmif_read,
    .write = mtk_spmi_pmif_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/*
 * SPMI master block. The arbiter's own commands (reset, sleep, wakeup) are
 * issued here rather than through a channel; the driver writes the command and
 * then waits for the busy bit to come back set, so completing the command is
 * all this side has to do.
 */
static uint64_t mtk_spmi_mst_read(void *opaque, hwaddr offset, unsigned size)
{
    MtkSpmiPmifState *s = MTK_SPMI_PMIF(opaque);

    if (offset + 4 > sizeof(s->mst_regs)) {
        return 0;
    }
    return s->mst_regs[offset / 4];
}

static void mtk_spmi_mst_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    MtkSpmiPmifState *s = MTK_SPMI_PMIF(opaque);

    if (offset + 4 > sizeof(s->mst_regs)) {
        return;
    }

    /*
     * The RCS interrupt status bytes are write-1-to-clear and drive the
     * level-sensitive RCS line to the EINT controller.
     */
    if (offset >= SPMI_SLV_EINT_BASE &&
        offset < SPMI_SLV_EINT_BASE + SPMI_SLV_EINT_REGS * 4) {
        s->mst_regs[offset / 4] &= ~(uint32_t)value;
        mtk_spmi_pmif_update_rcs(s);
        return;
    }

    s->mst_regs[offset / 4] = value;

    if (offset == SPMI_OP_ST_CTRL) {
        /* Command accepted: report it done on the status register */
        s->mst_regs[SPMI_OP_ST_STA / 4] |= 1;
    }
}

static const MemoryRegionOps mtk_spmi_mst_ops = {
    .read = mtk_spmi_mst_read,
    .write = mtk_spmi_mst_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void mtk_spmi_pmif_realize(DeviceState *dev, Error **errp)
{
    MtkSpmiPmifState *s = MTK_SPMI_PMIF(dev);

    memory_region_init_io(&s->pmif, OBJECT(s), &mtk_spmi_pmif_ops, s,
                          TYPE_MTK_SPMI_PMIF ".pmif", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->pmif);

    memory_region_init_io(&s->mst, OBJECT(s), &mtk_spmi_mst_ops, s,
                          TYPE_MTK_SPMI_PMIF ".mst", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mst);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->rcs_irq);
    s->trace = getenv("MTK_PMICLOG") != NULL;
}

static void mtk_spmi_pmif_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mtk_spmi_pmif_realize;
    device_class_set_legacy_reset(dc, mtk_spmi_pmif_reset);
    dc->desc = "MediaTek SPMI PMIC arbiter";
}

static const TypeInfo mtk_spmi_pmif_info = {
    .name = TYPE_MTK_SPMI_PMIF,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkSpmiPmifState),
    .class_init = mtk_spmi_pmif_class_init,
};

static void mtk_spmi_pmif_register_types(void)
{
    type_register_static(&mtk_spmi_pmif_info);
}

type_init(mtk_spmi_pmif_register_types)
