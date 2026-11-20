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

/*
 * Latest-command recorder.
 *
 * The preloader dumps the arbiter's recent commands ("[PMIF] Latest cmd
 * lists before reboot") by reading a block of PMIF registers.  Its own read
 * sequence (observable with MTK_PMIF_TRACE) is one header pair at 0x4f8/0x4fc
 * followed by PMIF_CMDQ_NUM entries at 0x50c + 0x14*i, each read as two words:
 * the command word at +0 and the data word at +4.  The command word packs
 * exactly the fields the preloader prints:
 *
 *   swinf(4) cmd(4) is_write(1) slvid(4) bytecnt(3) addr(16)
 *
 * (the printed cmd is the normalised SPMI opcode the driver put in the ACC
 * register, and is_write is that ACC's rw bit).  The layout is reproduced here
 * from that read sequence; the exact hardware packing of the "swinf" field is
 * not established yet, so this model records the physical channel index and
 * documents it as unresolved rather than inventing a remap.
 */
#define PMIF_CMDQ_HDR0          0x04f8
#define PMIF_CMDQ_HDR1          0x04fc
#define PMIF_CMDQ_BASE          0x0500
#define PMIF_CMDQ_ENTRY         0x0014
#define PMIF_CMDQ_NUM           31

/*
 * Observational tracing, opt-in through the environment.  MTK_PMIF_TRACE
 * logs every PMIF/SPMI-master register access; MTK_SPMI_XTRACE logs every
 * SPMI transaction as it is executed.  Both are diagnostics only: they never
 * change what the guest sees.
 */
static bool mtk_pmif_trace_enabled(void)
{
    static int on = -1;

    if (on < 0) {
        on = getenv("MTK_PMIF_TRACE") != NULL;
    }
    return on;
}

static bool mtk_spmi_xtrace_enabled(void)
{
    static int on = -1;

    if (on < 0) {
        on = getenv("MTK_SPMI_XTRACE") != NULL;
    }
    return on;
}

#define MTK_PMIF_TRACE_BUDGET 200000

/*
 * PMIC parts behind the SPMI arbiter.  Which part sits at which slave id is a
 * machine (board) property, not a controller property: the machine passes it
 * through the "slave-map" property as a list of "sid=part" pairs.
 *
 * The power-on register contents of each part come from the real unit's own
 * boot log (artifacts/r1302/lineage/real_device_boot.txt): the preloader prints
 * every status register it reads together with the register address and the
 * value the part returned, e.g.
 *
 *   [PMIC]PONSTS[0xC]=0x0   [SECOND_PMIC]POFFSTS0_1[0xD]=0x4
 *   [MT6319]S6 RG_SLV_ID[0x136]=0xB6   [MT6685]STS_PDN_MODE0_1[0x15]=0xFFFF
 *
 * Those values are this part's latched power-on / power-down state, so they
 * are modelled as the register file's reset contents.
 *
 * Provenance of every entry below is one of:
 *   DIRECT  - the device's own log prints both the register address and the
 *             value it read back (e.g. "[PMIC]POFFSTS[0xD]=0x400").
 *   INFERRED- the value is printed by the device but the byte address inside a
 *             multi-byte or otherwise implicit access is inferred from the SPMI
 *             transaction trace of the same boot (see the entries marked
 *             INFERRED below).  Nothing here is a plausible-number guess.
 *
 * The rail "latch" reads are INFERRED in exactly that sense: the preloader
 * prints the rail name and the raw code it read
 *
 *   [MT6363] latch VBUCK5 1050000 uV(0xFC)
 *
 * and the SPMI trace of the same boot shows which register that latch reads
 * (VBUCK5 -> SID4 0x1431), so 0x1431 resets to 0xFC.
 *
 * Registers that the device never prints are deliberately left at zero rather
 * than invented; the one value inherited from the previous flat model
 * (MT6363 0x001e) is marked LEGACY.
 */
typedef enum {
    PMIC_PART_NONE = 0,
    PMIC_PART_MT6363,           /* main PMIC, SPMI slave 4 */
    PMIC_PART_MT6368,           /* second PMIC, SPMI slave 5 */
    PMIC_PART_MT6319,           /* buck, SPMI slaves 6 and 7 */
    PMIC_PART_MT6685,           /* RTC / auxadc PMIC, SPMI slave 9 */
} PmicPart;

typedef struct {
    uint16_t reg;
    uint8_t val;
} PmicPowerOnReg;

/*
 * MT6363 (slave 4), from the real unit's boot log.  Registers 0x000a/0x000b
 * form the 2-byte chip code the preloader prints as
 *   [mt6363_check_cid] CHIP Code=0x6330
 * (the preloader reads SID4 0x000a..0x000b as one 2-byte access).
 */
static const PmicPowerOnReg mt6363_power_on[] = {
    /* Reg 0x09 is DIRECT ("slvid:4 rdata = 0x63"); 0x08/0x0b INFERRED equal. */
    { 0x0008, 0x63 }, { 0x0009, 0x63 },
    /*
     * The preloader reads SID4 0x000a as one 2-byte access; the device prints
     * the result as CHIP Code=0x6330, so the bytes are 0x30, 0x63.
     */
    { 0x000a, 0x30 }, { 0x000b, 0x63 },   /* CHIP Code = 0x6330 */
    { 0x000c, 0x00 },                     /* PONSTS[0xC] = 0x0 */
    { 0x000d, 0x00 }, { 0x000e, 0x04 },   /* POFFSTS[0xD] = 0x400 */
    { 0x000f, 0x00 }, { 0x0010, 0x00 },   /* POFFSTS2_3[0xF] = 0x0 */
    { 0x0011, 0xff }, { 0x0012, 0xff },   /* POFFSTS4_5[0x11] = 0xFFFF */
    { 0x0013, 0x0f },                     /* POFFSTS6[0x13] = 0xF */
    /*
     * MT6363 TOPSTATUS (0x1e).  Bits 1/3/4 are the active-low debounced
     * power/home/home2 key levels: is_pmic_key_pressed() returns
     * "pressed" when the bit is 0.  On the board no key is held at boot,
     * so all three inputs are pulled high and read 1.
     *
     * The old value 0x06 had bit 3 (HOMEKEY_DEB) clear, so LK's
     * detect_key(BOOT_MENU_KEY)->is_pmic_key_pressed(HOME_KEY) reported
     * the home key as pressed and boot_mode_check() selected
     * RECOVERY_BOOT ("lk boot mode = 2") on every normal power-on.
     * Setting bit 3 gives 0x0e and lets the device boot Android.
     *
     * MTK_NO_HOMEKEY_STRAP=1 keeps the historical 0x06 value for A/B.
     */
    { 0x001e, 0x0e },                     /* TOPSTATUS: PWR/HOME/HOME2 de-bounced = released */
    { 0x020b, 0xff }, { 0x020c, 0xff },   /* PG_SDN_STS0_1[0x20B] = 0xFFFF */
    { 0x020d, 0xff }, { 0x020e, 0xff },   /* PG_SDN_STS2_3[0x20D] = 0xFFFF */
    { 0x020f, 0x80 },                     /* PG_SDN_STS4[0x20F] = 0x80 */
    { 0x0210, 0x00 }, { 0x0211, 0x00 },   /* OC_SDN_STS0_1[0x210] = 0x0 */
    { 0x0212, 0x00 }, { 0x0213, 0x00 },   /* OC_SDN_STS2_3[0x212] = 0x0 */
    { 0x0214, 0x00 }, { 0x0215, 0x00 },   /* VR_SDN_MODE0_1[0x214] = 0x0 */
    { 0x0216, 0x00 }, { 0x0217, 0x00 },   /* VR_SDN_MODE2_3[0x216] = 0x0 */
    { 0x0218, 0x00 }, { 0x0219, 0x00 },   /* VR_SDN_MODE4_5[0x218] = 0x0 */
    { 0x021a, 0x00 }, { 0x021b, 0x00 },   /* VR_SDN_MODE6_7[0x21A] = 0x0 */
    { 0x0a1a, 0x02 },                     /* STRUP_CON4[0xA1A] = 0x2 */
    { 0x0a0f, 0x01 },                     /* STRUP_CON12[0xA0F] = 0x1 */
    { 0x0139, 0x04 },                     /* WDTRSTB[0x139] = 0x4 */
    /*
     * Power-on reset defaults for two rail registers the get-voltage phase
     * reads before anything writes them (verified against the SPMI trace: their
     * first access is the read at serial "get volt", no earlier write exists in
     * preloader/BL2_EXT/LK).
     *   SID4 0x24e : reset default, value 0x78 from device get-volt line
     *                "[MT6363] get volt 1, 120, 0, 750000", address from trace
     *                15602 -> INFERRED.
     *   SID4 0x258 : reset default, value 0x48 from device get-volt line
     *                "[MT6363] get volt 6, 72, 0, 850000", address from trace
     *                15596 -> INFERRED.
     * If a stage we do not model would have written them on the device, these
     * seeds would mask that gap; that is recorded as an INFERRED debt item.
     * BROM is not modelled, so a BROM write to these addresses cannot be
     * excluded (the trace proof covers preloader onward only).
     */
    { 0x024e, 0x78 },                     /* vcore readback 120 (750000 uV) */
    { 0x0258, 0x48 },                     /* vsram_digrf readback 72 (850000 uV) */
    /*
     * Rail VSEL contents latched at boot: values DIRECT from the
     * "[MT6363] latch <rail> <uV>(<code>)" lines, addresses INFERRED from the
     * SPMI trace of the same boot (the latch loop reads these in rail order).
     */
    { 0x142d, 0x00 },                     /* VBUCK1  0 uV (0x0) */
    { 0x142e, 0x74 },                     /* VBUCK2  550000 uV (0x74) */
    { 0x142f, 0x00 },                     /* VBUCK3  0 uV (0x0) */
    { 0x1430, 0x44 },                     /* VBUCK4  750000 uV (0x44) */
    { 0x1431, 0xfc },                     /* VBUCK5  1050000 uV (0xFC) */
    { 0x1432, 0x00 },                     /* VBUCK6  0 uV (0x0) */
    { 0x1d24, 0x00 },                     /* VSRAM_DIGRF 400000 uV (0x0) */
    { 0x1da9, 0x60 },                     /* VSRAM_MODEM 800000 uV (0x60) */
    { 0x1e0e, 0x2c },                     /* VSRAM_CPUB 743750 uV (0x2C) */
    { 0x1e24, 0x24 },                     /* VSRAM_CPUM 750000 uV (0x24) */
    { 0x1e8e, 0x24 },                     /* VSRAM_CPUL 750000 uV (0x24) */
};

/*
 * MT6368 (slave 5): status values DIRECT from the "[SECOND_PMIC]..." lines,
 * rail latch values DIRECT from "[MT6368] latch ...", latch addresses INFERRED
 * from the SPMI trace (0x1423..0x1429 read in rail order).
 */
static const PmicPowerOnReg mt6368_power_on[] = {
    /* 0x09 DIRECT (spmi_read_check slvid 5); 0x08/0x0b INFERRED equal. */
    { 0x0008, 0x68 }, { 0x0009, 0x68 },
    /* 2-byte read at 0x000a; device prints CHIP Code=0x6830. */
    { 0x000a, 0x30 }, { 0x000b, 0x68 },   /* CHIP Code = 0x6830 */
    { 0x000c, 0x00 },                     /* PONSTS[0xC] = 0x0 */
    { 0x000d, 0x04 }, { 0x000e, 0x00 },   /* POFFSTS0_1[0xD] = 0x4 */
    { 0x000f, 0x00 }, { 0x0010, 0x00 },   /* POFFSTS2_3[0xF] = 0x0 */
    { 0x0011, 0xff }, { 0x0012, 0xff },   /* POFFSTS4_5[0x11] = 0xFFFF */
    { 0x0013, 0x0f },                     /* POFFSTS6[0x13] = 0xF */
    { 0x0209, 0xff }, { 0x020a, 0xf8 },   /* PG_SDN_STS0_1[0x209] = 0xF8FF */
    { 0x020b, 0x00 }, { 0x020c, 0x00 },   /* OC_SDN_STS0_1[0x20B] = 0x0 */
    { 0x020d, 0x00 }, { 0x020e, 0x00 },   /* VR_SDN_MODE0_1[0x20D] = 0x0 */
    { 0x020f, 0x00 }, { 0x0210, 0x00 },   /* VR_SDN_MODE2_3[0x20F] = 0x0 */
    { 0x0211, 0x00 }, { 0x0212, 0x00 },   /* VR_SDN_MODE4_5[0x211] = 0x0 */
    { 0x0213, 0x00 },                     /* VR_SDN_MODE6[0x213] = 0x0 */
    { 0x0139, 0x04 },                     /* WDTRSTB[0x139] = 0x4 */
    { 0x0240, 0x0c },
    /*
     * Power-on reset default for the vapu rail register: the get-voltage phase
     * reads SID5 0x250 before anything writes it (verified in the SPMI trace:
     * first access is the read at trace 15632, and the only write in the whole
     * boot is much later at trace 41390).  Value 0x78 from device get-volt line
     * "[MT6368] get volt 20, 120, 0, 750000", address from trace -> INFERRED
     * reset-default seed.  BROM is not modelled, so a BROM write to this
     * address cannot be excluded (the trace proof covers preloader onward only).
     */
    { 0x0250, 0x78 },
    /* Rail VSEL latches: VBUCK1..6 at 0x1423..0x1428, VPA at 0x1429. */
    { 0x1423, 0x00 },                     /* VBUCK1 0 uV (0x0) */
    { 0x1424, 0x00 },                     /* VBUCK2 0 uV (0x0) */
    { 0x1425, 0x00 },                     /* VBUCK3 0 uV (0x0) */
    { 0x1426, 0x00 },                     /* VBUCK4 0 uV (0x0) */
    { 0x1427, 0x78 },                     /* VBUCK5 500000 uV (0x78) */
    { 0x1428, 0x5c },                     /* VBUCK6 650000 uV (0x5C) */
    { 0x1429, 0x03 },                     /* VPA 600000 uV (0x3) */
};

/* MT6319 buck (slaves 6 and 7): status + the two latched rails of each die. */
static const PmicPowerOnReg mt6319_s6_power_on[] = {
    /* 0x09 DIRECT (spmi_read_check slvid 6); 0x0b INFERRED equal. */
    { 0x0009, 0x15 }, { 0x000b, 0x15 },
    { 0x0136, 0xb6 },                     /* RG_SLV_ID[0x136] = 0xB6 */
    { 0x000c, 0x00 },                     /* POFFSTS[0xC] = 0x0 */
    { 0x0a0f, 0x00 },                     /* RGS_POFFSTS[0xA0F] = 0x0 */
    { 0x000f, 0x0f },                     /* PG_SDN_STS0[0xF] = 0xF */
    { 0x0010, 0x00 },                     /* OC_SDN_STS0[0x10] = 0x0 */
    { 0x1416, 0x00 },                     /* BUCK_OC_SDN_STS[0x1416] = 0x0 */
    { 0x1451, 0x1f },                     /* BUCK_OC_SDN_EN[0x1451] = 0x1F */
    { 0x0126, 0x04 },                     /* TOP_RST_MISC[0x126] = 0x4 */
    /*
     * Rail VSEL registers of this die.  The payload uses two aliases for the
     * same register (0x1419/0x1449 and 0x141d/0x144d) and reads the monitor
     * aliases 0x1499/0x1599 for the current voltage; see the MT6319 mirror
     * handling in mtk_spmi_pmif_xfer().  Values are the device's own latched
     * codes ("[MT6319]latch vcpub 493750 uV(0x68)", "... vmm 0 uV(0x0)").
     */
    { 0x1419, 0x68 }, { 0x1449, 0x68 },   /* vcpub 493750 uV (0x68) */
    { 0x141d, 0x00 }, { 0x144d, 0x00 },   /* vmm 0 uV (0x0) */
};

static const PmicPowerOnReg mt6319_s7_power_on[] = {
    /* 0x09 DIRECT (spmi_read_check slvid 7); 0x0b INFERRED equal. */
    { 0x0009, 0x15 }, { 0x000b, 0x15 },
    { 0x0136, 0xb7 },                     /* RG_SLV_ID[0x136] = 0xB7 */
    { 0x000c, 0x00 },
    { 0x0a0f, 0x00 },
    { 0x000f, 0x0f },
    { 0x0010, 0x00 },
    { 0x1416, 0x00 },
    { 0x1451, 0x1f },
    { 0x0126, 0x04 },
    /*
     * VSEL/monitor aliases for this die; values are the device's latches
     * ("[MT6319]latch vcpum 500000 uV(0x78)", "... vcpul 550000 uV(0x74)").
     */
    { 0x1419, 0x78 }, { 0x1449, 0x78 },   /* vcpum 500000 uV (0x78) */
    { 0x141d, 0x74 }, { 0x144d, 0x74 },   /* vcpul 550000 uV (0x74) */
};

/* MT6685 RTC/auxadc PMIC (slave 9), from the "[MT6685]..." lines. */
static const PmicPowerOnReg mt6685_power_on[] = {
    /* 0x09 DIRECT (spmi_read_check slvid 9); 0x08/0x0b INFERRED equal. */
    { 0x0008, 0x85 }, { 0x0009, 0x85 }, { 0x000b, 0x85 },
    { 0x000c, 0x00 },                     /* PONSTS[0xC] = 0x0 */
    { 0x000d, 0x00 }, { 0x000e, 0x00 },   /* POFFSTS0_1[0xD] = 0x0 */
    { 0x0013, 0x7f },                     /* PG_OC_SDN_STS0[0x13] = 0x7F */
    { 0x0015, 0xff }, { 0x0016, 0xff },   /* STS_PDN_MODE0_1[0x15] = 0xFFFF */
};

struct MtkSpmiPmifState {
    SysBusDevice parent_obj;

    MemoryRegion pmif;
    MemoryRegion mst;

    uint32_t pmif_regs[0x1000 / 4];
    uint32_t mst_regs[0x100 / 4];

    /* One register space per SPMI slave id: 16-bit address, 8-bit value */
    uint8_t pmic[SPMI_MAX_SID][SPMI_PMIC_REG_SPACE];

    /* Board description: which PMIC part responds at each slave id. */
    char *slave_map;
    PmicPart slave_part[SPMI_MAX_SID];

    /* Latest-command log: ring of the last PMIF_CMDQ_NUM commands. */
    unsigned cmdq_head;
    unsigned cmdq_count;

    /* Realisation index; only used to tell the two PMIF instances apart in the
     * MTK_PMIF_TRACE diagnostic. */
    unsigned inst;

    /* PMIC OTP/efuse windows: word index latches (see xfer()). */
    uint8_t otp_addr[SPMI_MAX_SID];      /* 0x42d latch: MT6363 / MT6368 OTP */
    uint8_t mt6319_efuse_idx[SPMI_MAX_SID]; /* 0x38c latch: MT6319 efuse */

    /*
     * MT6368 (SID5) internal EFUSE controller: 0x39e key unlock, 0x38b
     * address latch, 0x395 trigger, 0x39b busy, 0x399 16-bit data.  Protocol
     * from LUA_1 helper 0x0203b744 (notes_pmic_tail_recon.md §3.3).
     */
    bool mt6368_efuse_unlocked[SPMI_MAX_SID];
    uint8_t mt6368_efuse_addr[SPMI_MAX_SID];

    /* SPMI RCS interrupt output (level, to the EINT controller). */
    qemu_irq rcs_irq;
    bool trace;
};

static void mtk_spmi_pmif_seed(MtkSpmiPmifState *s, unsigned sid,
                               const PmicPowerOnReg *regs, unsigned n)
{
    unsigned i;

    for (i = 0; i < n; i++) {
        s->pmic[sid][regs[i].reg] = regs[i].val;
    }
}

/*
 * Load each described part's power-on register contents.
 */
static void mtk_spmi_pmif_seed_parts(MtkSpmiPmifState *s)
{
    unsigned sid;

    for (sid = 0; sid < SPMI_MAX_SID; sid++) {
        switch (s->slave_part[sid]) {
        case PMIC_PART_MT6363:
            mtk_spmi_pmif_seed(s, sid, mt6363_power_on,
                               ARRAY_SIZE(mt6363_power_on));
            if (MTK_ENV_CACHED("MTK_NO_HOMEKEY_STRAP")) {
                s->pmic[sid][0x1e] = 0x06;   /* legacy: HOMEKEY_DEB low */
            }
            break;
        case PMIC_PART_MT6368:
            mtk_spmi_pmif_seed(s, sid, mt6368_power_on,
                               ARRAY_SIZE(mt6368_power_on));
            break;
        case PMIC_PART_MT6319:
            /*
             * The two MT6319 dies are different instances of the same part;
             * the board identifies them by slave id, and each die's latched
             * register contents differ (real log S6/S7 sections).
             */
            if (sid == 6) {
                mtk_spmi_pmif_seed(s, sid, mt6319_s6_power_on,
                                   ARRAY_SIZE(mt6319_s6_power_on));
            } else {
                mtk_spmi_pmif_seed(s, sid, mt6319_s7_power_on,
                                   ARRAY_SIZE(mt6319_s7_power_on));
            }
            break;
        case PMIC_PART_MT6685:
            mtk_spmi_pmif_seed(s, sid, mt6685_power_on,
                               ARRAY_SIZE(mt6685_power_on));
            break;
        default:
            break;
        }
    }
}

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
     * The arbiter reports "not initialised" until software runs its enable
     * sequence; the real device's first is_pmif_spmi_init_done() call reads 0
     * and only the post-enable call reads 1
     *   ([PMIF] is_pmif_spmi_init_done ret = 0 -> ...enable steps... -> ret = 1).
     * mtk_spmi_pmif_write() completes the handshake when the enable registers
     * are written.
     */
    s->pmif_regs[PMIF_INIT_DONE / 4] = 0;

    memset(s->pmic, 0, sizeof(s->pmic));
    s->cmdq_head = 0;
    s->cmdq_count = 0;

    /*
     * Load the power-on register contents of the PMIC parts the board puts on
     * this SPMI bus.  Besides the latched status/rail values, this supplies the
     * part ids that the preloader's SPMI calibration check table validates:
     *   slvid 4 reg 0x09 = 0x63, slvid 9 = 0x85, slvid 6/7 = 0x15,
     *   slvid 5 = 0x68 and reg 0x240 = 0x0c,
     * and the 2-byte chip codes the PMIC drivers print ([mt6363_check_cid]
     * CHIP Code=0x6330, [second_pmic_check_cid] CHIP Code=0x6830).
     */
    mtk_spmi_pmif_seed_parts(s);
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

/*
 * Append one completed transaction to the latest-command recorder.
 */
static void mtk_spmi_pmif_record_cmd(MtkSpmiPmifState *s, unsigned chan,
                                     unsigned opc, unsigned rw, unsigned sid,
                                     unsigned len, unsigned addr, uint32_t data)
{
    unsigned slot = PMIF_CMDQ_BASE + s->cmdq_head * PMIF_CMDQ_ENTRY;
    uint32_t word;

    word = ((chan & 0xf) << 28) |
           ((opc & 0xf) << 24) |
           ((rw & 0x1) << 23) |
           ((sid & 0xf) << 19) |
           (((len - 1) & 0x7) << 16) |
           (addr & 0xffff);

    s->pmif_regs[(slot + 0xc) / 4] = word;
    s->pmif_regs[(slot + 0x10) / 4] = data;

    s->cmdq_head = (s->cmdq_head + 1) % PMIF_CMDQ_NUM;
    s->cmdq_count++;
}

/*
 * PMIC OTP/efuse word read.
 *
 * MT6363 and MT6368 expose their OTP through a banked window: the word address
 * is written to 0x42d and the 16-bit word is read back at 0x42b (the preloader
 * walks words 0x00..0x1f with a second bank at 0x10+n).  However no raw word
 * contents for these two parts exist in any device artefact we have (the log
 * only carries derived values such as VCORE VB_INFO1), so every word reads 0
 * for them - same behaviour as before, but through a real address latch.
 *
 * The MT6319 dies use a different interface: word index written to 0x38c and
 * data read at 0x39b; only their eight efuse words are device-sourced.
 */
static uint16_t mtk_spmi_pmif_otp_word(MtkSpmiPmifState *s, unsigned sid)
{
    return 0;   /* MT6363/MT6368: contents not available from the device */
}

/*
 * MT6368 internal EFUSE words, addressed through the controller registers
 * (0x38b latch / 0x395 trigger / 0x399 data).  Word 0x7a bits[4:0] select the
 * second-PMIC variant on the real device: (word & 0x1f) == 1 marks the
 * mt6368DW build -- LUA_1 helper 0x0203bb9a gates both the RSV4 bit 29 in
 * dvfsrc_opp_level_mapping (0x30500020 vs 0x10500020) and the 15-entry
 * mt6368DW init table on exactly that comparison
 * (notes_pmic_tail_recon.md §1-§3).  Only that field is documented by the
 * device artefacts; every other word reads 0.
 */
static uint16_t mtk_spmi_mt6368_efuse_word(unsigned word)
{
    if (word == 0x7a) {
        return 0x0001;
    }
    return 0;
}

/*
 * MT6319 efuse data at 0x39b, selected by the 0x38c word-index latch.
 *
 * The real unit's log prints eight decoded words per die (S6:
 * 0xFFFF:0xFFFF:0xFFFF:0xFFFF:0xF5A4:0x8CFF:0xFA67:0xE678, S7: ...), but those
 * are the payload's DECODED output, not raw efuse contents (feeding them back
 * double-applies the decode: 0xFFFF in -> 0xFFFD out).  Raw contents are not
 * available from any device artefact, so every word reads 0 until the payload's
 * decode is inverted by disassembly (notes_pmic_round1.md section 16).
 */
static uint16_t mtk_spmi_pmif_mt6319_efuse(MtkSpmiPmifState *s, unsigned sid)
{
    return 0;
}

/*
 * MT6319 rail register aliases.
 *
 * The payload reaches each MT6319 buck's voltage-select register under two
 * addresses (0x1419/0x1449 for buck A, 0x141d/0x144d for buck B on both dies)
 * and reads the current setting through separate monitor addresses (0x1499 for
 * buck A, 0x1599 for buck B).  Addresses come from the preloader's own SPMI
 * sequence: it writes 0x1449/0x144d for the set phase and reads 0x1499/0x1599
 * in the get-voltage phase; the device's get lines (vcpub 120=0x78,
 * vcpum 95=0x5f, vcpul 97=0x61) are the codes that must appear there.  A write
 * through either alias therefore has to be visible through all of them.
 */
static unsigned mt6319_vsel_alias(unsigned reg)
{
    switch (reg) {
    case 0x1419:
        return 0x1449;
    case 0x1449:
        return 0x1419;
    case 0x141d:
        return 0x144d;
    case 0x144d:
        return 0x141d;
    default:
        return 0;
    }
}

/* Monitor address -> VSEL register it reports. */
static unsigned mt6319_vsel_monitor(unsigned reg)
{
    switch (reg) {
    case 0x1499:
        return 0x1449;
    case 0x1599:
        return 0x144d;
    default:
        return 0;
    }
}

/*
 * MT6363 SDMADC (sigma-delta ADC) conversion window.
 *
 * The preloader reads the two board-identification voltages through the
 * "main_pmic_sdmadc" auxadc device (registered in LUA_1 by mt6363_auxadc_init;
 * the reader is LUA_1 VA 0x020384A0 and prints "project id volt"/"build id
 * volt").  The channel descriptor table at LUA_1 0x0206C54C gives the access
 * protocol (read op at 0x0203AFA8):
 *
 *   write 0x11C4 = 0x80 | mux | (len << 3)   channel select / enable
 *   write 0x1109 = 0x10                      start conversion
 *   poll 0x10D2 bit15 until set, then read 0x10D2 as a 16-bit little-endian
 *                                            value: bit15 = done, [14:0] = code
 *
 * and the code-to-millivolt conversion (LUA_1 0x0203B05A) is
 *
 *   mV = (code * 0x730) >> 15                (0x730 = 1840)
 *
 * The board-id reader asks for channels 18 and 19, which the driver's channel
 * table maps to mux 1 and mux 2.  The analog inputs are board resistors that no
 * artefact exposes, so the two codes are seeded from the unit's own boot log
 * (real_device_boot.txt:2075-2076):
 *
 *   project id volt = 475 mv, projectId_channel = 18
 *   build  id volt =  71 mv, buildId_channel  = 19
 *
 * mux 1 therefore returns the smallest code that converts to 475 mV
 * (ceil(475 * 32768 / 1840) = 8460) and mux 2 the one for 71 mV (1265); every
 * other mux reads code 0.  The conversion is reported complete on the first
 * poll, as on the unit, where the whole auxadc read is far below the retry
 * budget.
 */
#define MT6363_SDMADC_CFG    0x11c4
#define MT6363_SDMADC_START  0x1109
#define MT6363_SDMADC_DATA   0x10d2
#define MT6363_SDMADC_DONE   0x8000

static uint16_t mtk_spmi_pmif_sdmadc(MtkSpmiPmifState *s, unsigned sid)
{
    unsigned mux = s->pmic[sid][MT6363_SDMADC_CFG] & 0x7;
    uint16_t code;

    switch (mux) {
    case 1:     /* project id channel 18: 475 mV on the unit */
        code = 8460;
        break;
    case 2:     /* build id channel 19: 71 mV on the unit */
        code = 1265;
        break;
    default:
        code = 0;
        break;
    }
    return MT6363_SDMADC_DONE | code;
}

static void mtk_spmi_pmif_xfer(MtkSpmiPmifState *s, unsigned chan, uint32_t cmd)
{
    unsigned opc = (cmd >> 30) & 0x3;
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
            unsigned r = addr + i;
            uint8_t v = (data >> (8 * i)) & 0xff;

            s->pmic[sid][r] = v;
            if (s->slave_part[sid] == PMIC_PART_MT6319) {
                unsigned alias = mt6319_vsel_alias(r);

                if (alias) {
                    s->pmic[sid][alias] = v;
                }
                if (r == 0x38c) {
                    /* MT6319 efuse word-index latch (data read at 0x39b). */
                    s->mt6319_efuse_idx[sid] = v;
                }
            }
            if (r == 0x42d) {
                /* MT6363/MT6368 OTP word-address latch (data read at 0x42b). */
                s->otp_addr[sid] = v;
            }
            if (s->slave_part[sid] == PMIC_PART_MT6368) {
                /*
                 * MT6368 internal EFUSE controller (LUA_1 0x0203b744):
                 * unlock 0x39e with the 16-bit key 0x9c97, latch the word's
                 * byte address (word_index << 1) & 0xff in 0x38b, then write
                 * 0x395 to trigger; the driver waits for 0x39b != 1 and reads
                 * the 16-bit word at 0x399 little-endian.
                 */
                if (addr == 0x39e && len >= 2) {
                    s->mt6368_efuse_unlocked[sid] =
                        (data & 0xffffu) == 0x9c97u;
                }
                if (addr == 0x38b) {
                    s->mt6368_efuse_addr[sid] = v;
                }
                if (addr == 0x395) {
                    uint16_t w = mtk_spmi_mt6368_efuse_word(
                        s->mt6368_efuse_addr[sid] >> 1);

                    if (s->mt6368_efuse_unlocked[sid]) {
                        s->pmic[sid][0x399] = (uint8_t)(w & 0xff);
                        s->pmic[sid][0x39a] = (uint8_t)(w >> 8);
                    }
                    s->pmic[sid][0x39b] = 0;    /* BUSY clear: ready */
                }
            }
            mtk_spmi_ring_add(sid, r, v, 0);
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

        if (mtk_spmi_xtrace_enabled()) {
            fprintf(stderr, "MTK-SPMI[%u] wr ch=%u sid=%u addr=%#x len=%u data=%#x\n",
                    s->inst, chan, sid, addr, len, data);
        }
        mtk_spmi_pmif_record_cmd(s, chan, opc, rw, sid, len, addr, data);
        /*
         * A write has no payload for the guest to collect, so the channel is
         * done as soon as the command is accepted. Only a read parks in
         * WFVLDCLR, waiting for the guest to clear the valid flag.
         */
        s->pmif_regs[PMIF_SWINF_STA(chan) / 4] = SWINF_IDLE_VAL;
    } else {
        uint32_t data = 0;

        for (i = 0; i < len; i++) {
            unsigned r = addr + i;
            uint8_t v;

            if (addr == 0x42b && len == 2) {
                /*
                 * MT6363/MT6368 OTP data window: 16-bit word selected by the
                 * 0x42d latch, returned little-endian over the two bytes.
                 */
                uint16_t w = mtk_spmi_pmif_otp_word(s, sid);

                v = (w >> (8 * i)) & 0xff;
            } else if (addr == 0x39b && len == 2 &&
                       s->slave_part[sid] == PMIC_PART_MT6319) {
                /*
                 * MT6319 efuse data window: 16-bit word selected by the 0x38c
                 * latch.  Contents are the device's own eight words.
                 */
                uint16_t w = mtk_spmi_pmif_mt6319_efuse(s, sid);

                v = (w >> (8 * i)) & 0xff;
            } else if (addr == MT6363_SDMADC_DATA && len == 2 &&
                       s->slave_part[sid] == PMIC_PART_MT6363 &&
                       (s->pmic[sid][MT6363_SDMADC_START] & 0x10)) {
                /*
                 * Board-identification SDMADC data window: conversion is
                 * already complete when the payload polls it, and the mux was
                 * selected by the last 0x11C4 write (see the block comment
                 * above).  The start register gates the window so unrelated
                 * reads of this address keep their old all-zero behaviour.
                 */
                uint16_t w = mtk_spmi_pmif_sdmadc(s, sid);

                v = (w >> (8 * i)) & 0xff;
            } else {
                v = s->pmic[sid][r];
                if (s->slave_part[sid] == PMIC_PART_MT6319) {
                    unsigned src = mt6319_vsel_monitor(r);

                    if (src) {
                        v = s->pmic[sid][src];
                    }
                }
            }
            data |= (uint32_t)v << (8 * i);
            mtk_spmi_ring_add(sid, r, v, 1);
        }
        s->pmif_regs[PMIF_SWINF_RDATA(chan) / 4] = data;
        s->pmif_regs[PMIF_SWINF_STA(chan) / 4] = SWINF_WFVLDCLR_VAL;
        if (mtk_spmi_xtrace_enabled()) {
            fprintf(stderr, "MTK-SPMI[%u] rd ch=%u sid=%u addr=%#x len=%u data=%#x\n",
                    s->inst, chan, sid, addr, len, data);
        }
        mtk_spmi_pmif_record_cmd(s, chan, opc, rw, sid, len, addr, data);
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
    if (mtk_pmif_trace_enabled()) {
        static int budget;

        if (budget < MTK_PMIF_TRACE_BUDGET) {
            budget++;
            fprintf(stderr, "MTK-PMIF[%u] rd off=%#x size=%u -> %#x\n",
                    s->inst, (unsigned)offset, size,
                    s->pmif_regs[offset / 4]);
        }
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
    if (mtk_pmif_trace_enabled()) {
        static int budget;

        if (budget < MTK_PMIF_TRACE_BUDGET) {
            budget++;
            fprintf(stderr, "MTK-PMIF[%u] wr off=%#x size=%u val=%#x\n",
                    s->inst, (unsigned)offset, size, (unsigned)value);
        }
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

    /*
     * Init handshake: the enable sequence (force normal mode, enable the
     * software interfaces, the command issue path and the arbiter) is what
     * brings the block out of reset; complete INIT_DONE once those registers
     * have been driven, the way the real arbiter does.
     */
    if ((offset == PMIF_INF_EN || offset == PMIF_ARB_EN ||
         offset == PMIF_CMDISSUE_EN || offset == PMIF_SPI_MODE_CTRL) &&
        value) {
        s->pmif_regs[PMIF_INIT_DONE / 4] |= 1;
    }
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
    if (mtk_pmif_trace_enabled()) {
        static int budget;

        if (budget < MTK_PMIF_TRACE_BUDGET) {
            budget++;
            fprintf(stderr, "MTK-SPMIMST rd off=%#x size=%u -> %#x\n",
                    (unsigned)offset, size, s->mst_regs[offset / 4]);
        }
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
    if (mtk_pmif_trace_enabled()) {
        static int budget;

        if (budget < MTK_PMIF_TRACE_BUDGET) {
            budget++;
            fprintf(stderr, "MTK-SPMIMST wr off=%#x size=%u val=%#x\n",
                    (unsigned)offset, size, (unsigned)value);
        }
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

static PmicPart mtk_spmi_pmif_part_by_name(const char *name, size_t len)
{
    static const struct {
        const char *name;
        PmicPart part;
    } names[] = {
        { "mt6363", PMIC_PART_MT6363 },
        { "mt6368", PMIC_PART_MT6368 },
        { "mt6319", PMIC_PART_MT6319 },
        { "mt6685", PMIC_PART_MT6685 },
    };
    size_t i;

    for (i = 0; i < ARRAY_SIZE(names); i++) {
        if (strlen(names[i].name) == len &&
            memcmp(names[i].name, name, len) == 0) {
            return names[i].part;
        }
    }
    return PMIC_PART_NONE;
}

/*
 * "slave-map" is the board's description of which PMIC part responds at which
 * SPMI slave id, e.g. "4=mt6363,5=mt6368,6=mt6319,7=mt6319,9=mt6685".
 */
static void mtk_spmi_pmif_parse_slave_map(MtkSpmiPmifState *s)
{
    const char *p = s->slave_map;

    memset(s->slave_part, 0, sizeof(s->slave_part));
    while (p && *p) {
        char *end;
        char name[16];
        unsigned sid;
        size_t n = 0;

        sid = strtoul(p, &end, 10);
        if (end == p || *end != '=') {
            break;
        }
        p = end + 1;
        while (*p && *p != ',' && n < sizeof(name) - 1) {
            name[n++] = *p++;
        }
        name[n] = 0;
        if (sid < SPMI_MAX_SID) {
            s->slave_part[sid] = mtk_spmi_pmif_part_by_name(name, n);
        }
        if (*p == ',') {
            p++;
        }
    }
}

static void mtk_spmi_pmif_realize(DeviceState *dev, Error **errp)
{
    MtkSpmiPmifState *s = MTK_SPMI_PMIF(dev);
    static unsigned next_inst;

    s->inst = next_inst++;
    mtk_spmi_pmif_parse_slave_map(s);

    memory_region_init_io(&s->pmif, OBJECT(s), &mtk_spmi_pmif_ops, s,
                          TYPE_MTK_SPMI_PMIF ".pmif", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->pmif);

    memory_region_init_io(&s->mst, OBJECT(s), &mtk_spmi_mst_ops, s,
                          TYPE_MTK_SPMI_PMIF ".mst", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mst);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->rcs_irq);
    s->trace = getenv("MTK_PMICLOG") != NULL;
}

static const Property mtk_spmi_pmif_props[] = {
    DEFINE_PROP_STRING("slave-map", MtkSpmiPmifState, slave_map),
};

static void mtk_spmi_pmif_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mtk_spmi_pmif_realize;
    device_class_set_props(dc, mtk_spmi_pmif_props);
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
