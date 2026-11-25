/*
 * MediaTek MT6895 GPUEB (GPU Embedded Controller) boot & mailbox device
 *
 * Models:
 *   - MMIO 0 (0x13c5fd24, size 0x1000): LK RV33 boot handshake (+0x8dc -> +0x00
 *     reports 0x55667788) and GPUEB mbox0 slot SRAM (0x13c5fd80..0x13c5ffff,
 *     160 x 32-bit slots at region offset 0x5c..0x2db).
 *   - MMIO 1 (0x13c62000, size 0x100): mbox0 IRQ control registers
 *     (+0x00 mbox0_send, +0x04 mbox0_set, +0x74 mbox0_clr, +0x78 mbox0_recv).
 *   - IRQ 0: GIC_SPI 516 level-high interrupt (asserted while recv_status != 0).
 *   - Pin 1 (IPI_ID_GPUFREQ) synchronous IPI responder backing
 *     mtk_gpufreq_wrapper.ko -> mtk_gpufreq_mt6895.ko -> ged.ko (/proc/ged).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/misc/mtk_gpueb.h"
#include "system/address-spaces.h"
#include "system/memory.h"

#define MTK_GPUEB_BOOT_MAGIC    0x55667788u
#define MTK_GPUEB_START_OFS     0x8dc
#define MTK_GPUEB_MBOX0_OFS     (MTK_GPUEB_MBOX0_BASE - MTK_GPUEB_BOOT_BASE)

#define MBOX0_OFF_SEND          0x00
#define MBOX0_OFF_SET           0x04
#define MBOX0_OFF_CLR           0x74
#define MBOX0_OFF_RECV          0x78

#define GPUEB_NUM_PINS          8
#define GPUEB_PIN_GPUFREQ       1

/* struct gpufreq_ipi_data::cmd from mtk_gpufreq_wrapper.ko */
enum {
    GPUFREQ_CMD_INIT_SHARED_MEM     = 0,
    GPUFREQ_CMD_GET_FREQ_BY_IDX     = 1,
    GPUFREQ_CMD_GET_POWER_BY_IDX    = 2,
    GPUFREQ_CMD_GET_OPPIDX_BY_FREQ  = 3,
    GPUFREQ_CMD_GET_LEAKAGE_POWER   = 4,
    GPUFREQ_CMD_SET_LIMIT           = 5,
    GPUFREQ_CMD_POWER_CONTROL       = 6,
    GPUFREQ_CMD_COMMIT              = 7,
    GPUFREQ_CMD_GET_DEBUG_OPP_INFO  = 8,
    GPUFREQ_CMD_GET_DEBUG_LIMIT_INFO = 9,
    GPUFREQ_CMD_GET_WORKING_TABLE   = 10,
    GPUFREQ_CMD_GET_SIGNED_TABLE    = 11,
    GPUFREQ_CMD_GET_LIMIT_TABLE     = 12,
    GPUFREQ_CMD_SWITCH_LIMIT        = 13,
    GPUFREQ_CMD_FIX_TARGET_OPPIDX   = 14,
    GPUFREQ_CMD_FIX_CUSTOM_FREQ_VOLT = 15,
    GPUFREQ_CMD_SET_STRESS_TEST     = 16,
    GPUFREQ_CMD_SET_GPM_MODE        = 18,
    GPUFREQ_CMD_SET_TEST_MODE       = 19,
};

typedef struct GpuebPinCfg {
    uint8_t send_slot;
    uint8_t send_len;
    uint8_t recv_slot;
    uint8_t recv_len;
    bool recv_ack;
} GpuebPinCfg;

/* Matches vendor-mt6895.dts send_table / recv_table and gpueb.ko pin layout */
static const GpuebPinCfg gpueb_pins[GPUEB_NUM_PINS] = {
    [0] = {  0, 4, 43, 4, false }, /* IPI_ID_FAST_DVFS_EVENT */
    [1] = {  4, 8, 47, 8, true  }, /* IPI_ID_GPUFREQ */
    [2] = { 12, 3, 55, 1, false }, /* IPI_ID_SLEEP */
    [3] = { 15, 3, 56, 1, false }, /* IPI_ID_TIMER */
    [4] = { 18, 9, 57, 1, true  }, /* IPI_ID_FHCTL */
    [5] = { 27, 4, 58, 4, true  }, /* IPI_ID_CCF */
    [6] = { 31, 6, 62, 1, true  }, /* IPI_ID_GPUMPU */
    [7] = { 37, 6, 63, 6, true  }, /* IPI_ID_FAST_DVFS */
};

typedef struct GpufreqOppEntry {
    uint32_t freq;
    uint32_t volt;
    uint32_t vsram;
    uint32_t posdiv;
    uint32_t vaging;
    uint32_t power;
} GpufreqOppEntry;

#define GPUFREQ_NUM_OPPS 8

/* Nominal MT6895 Mali-G610 MC6 OPP table (freq in kHz, volt in 10uV) */
static const GpufreqOppEntry gpufreq_opps[GPUFREQ_NUM_OPPS] = {
    { 886000, 75000, 75000, 2, 0, 2200 },
    { 800000, 72500, 75000, 2, 0, 1900 },
    { 700000, 70000, 75000, 2, 0, 1600 },
    { 600000, 67500, 75000, 4, 0, 1300 },
    { 500000, 65000, 75000, 4, 0, 1000 },
    { 400000, 62500, 75000, 4, 0,  750 },
    { 300000, 60000, 75000, 4, 0,  500 },
    { 218000, 57500, 75000, 8, 0,  300 },
};

/* Relevant offsets in struct gpufreq_shared_status (MEM_ID_GPUFREQ + 0x0) */
#define SHSTAT_CUR_OPPIDX_GPU       0x00
#define SHSTAT_CUR_OPPIDX_STACK     0x04
#define SHSTAT_OPP_NUM_GPU          0x08
#define SHSTAT_OPP_NUM_STACK        0x0c
#define SHSTAT_CUR_FGPU             0x10
#define SHSTAT_CUR_FSTACK           0x14
#define SHSTAT_CUR_VGPU             0x18
#define SHSTAT_CUR_VSTACK           0x1c
#define SHSTAT_CUR_VSRAM_GPU        0x20
#define SHSTAT_CUR_VSRAM_STACK      0x24
#define SHSTAT_CUR_POWER_GPU        0x28
#define SHSTAT_CUR_POWER_STACK      0x2c
#define SHSTAT_MAX_POWER_GPU        0x30
#define SHSTAT_MAX_POWER_STACK      0x34
#define SHSTAT_MIN_POWER_GPU        0x38
#define SHSTAT_MIN_POWER_STACK      0x3c
#define SHSTAT_CUR_CEIL_GPU         0x40
#define SHSTAT_CUR_FLOOR_GPU        0x44
#define SHSTAT_CUR_CEIL_STACK       0x48
#define SHSTAT_CUR_FLOOR_STACK      0x4c
#define SHSTAT_POWER_CONTROL        0x60
#define SHSTAT_DVFS_STATE           0x64
#define SHSTAT_SHADER_PRESENT       0x68
#define SHSTAT_POWER_COUNT          0x6c
#define SHSTAT_DEBUG_CEIL_GPU       0x80
#define SHSTAT_DEBUG_FLOOR_GPU      0x84
#define SHSTAT_DEBUG_LIM_GPU        0x88
#define SHSTAT_DEBUG_SIZE           0x100

struct MtkGpuebState {
    SysBusDevice parent_obj;

    MemoryRegion iomem_boot;
    MemoryRegion iomem_irq;
    qemu_irq irq;

    uint8_t boot_mem[MTK_GPUEB_BOOT_SIZE];
    bool started;
    uint32_t recv_status;

    uint64_t status_base;
    uint64_t debug_base;
    uint32_t status_size;
    uint32_t debug_size;

    uint32_t cur_oppidx;
    uint32_t ceil_idx;
    uint32_t floor_idx;
    uint32_t power_count;
    bool dbg_enabled;
};

static bool mtk_gpueb_dbg_enabled(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *v = getenv("MTK_GPUEB_DBG");
        cached = (v && v[0] && strcmp(v, "0") != 0) ? 1 : 0;
    }
    return cached != 0;
}

static uint32_t mtk_gpueb_slot_read(const MtkGpuebState *s, uint32_t slot)
{
    hwaddr off = MTK_GPUEB_MBOX0_OFS + (hwaddr)slot * 4;

    if (slot >= MTK_GPUEB_MBOX0_SLOTS || off + 4 > sizeof(s->boot_mem)) {
        return 0;
    }
    return ldl_le_p(s->boot_mem + off);
}

static void mtk_gpueb_slot_write(MtkGpuebState *s, uint32_t slot, uint32_t val)
{
    hwaddr off = MTK_GPUEB_MBOX0_OFS + (hwaddr)slot * 4;

    if (slot >= MTK_GPUEB_MBOX0_SLOTS || off + 4 > sizeof(s->boot_mem)) {
        return;
    }
    stl_le_p(s->boot_mem + off, val);
}

static uint32_t mtk_gpueb_clamp_opp(uint32_t idx)
{
    return idx < GPUFREQ_NUM_OPPS ? idx : (GPUFREQ_NUM_OPPS - 1);
}

static uint32_t mtk_gpueb_find_oppidx_by_freq(uint32_t freq_khz)
{
    uint32_t i;

    for (i = 0; i < GPUFREQ_NUM_OPPS; i++) {
        if (gpufreq_opps[i].freq <= freq_khz) {
            return i;
        }
    }
    return GPUFREQ_NUM_OPPS - 1;
}

static void mtk_gpueb_sync_shared_status(MtkGpuebState *s)
{
    uint8_t buf[SHSTAT_DEBUG_SIZE];
    const GpufreqOppEntry *opp;
    hwaddr len;

    if (!s->status_base) {
        return;
    }

    opp = &gpufreq_opps[mtk_gpueb_clamp_opp(s->cur_oppidx)];
    memset(buf, 0, sizeof(buf));

    stl_le_p(buf + SHSTAT_CUR_OPPIDX_GPU, s->cur_oppidx);
    stl_le_p(buf + SHSTAT_CUR_OPPIDX_STACK, s->cur_oppidx);
    stl_le_p(buf + SHSTAT_OPP_NUM_GPU, GPUFREQ_NUM_OPPS);
    stl_le_p(buf + SHSTAT_OPP_NUM_STACK, GPUFREQ_NUM_OPPS);
    stl_le_p(buf + SHSTAT_CUR_FGPU, opp->freq);
    stl_le_p(buf + SHSTAT_CUR_FSTACK, opp->freq);
    stl_le_p(buf + SHSTAT_CUR_VGPU, opp->volt);
    stl_le_p(buf + SHSTAT_CUR_VSTACK, opp->volt);
    stl_le_p(buf + SHSTAT_CUR_VSRAM_GPU, opp->vsram);
    stl_le_p(buf + SHSTAT_CUR_VSRAM_STACK, opp->vsram);
    stl_le_p(buf + SHSTAT_CUR_POWER_GPU, opp->power);
    stl_le_p(buf + SHSTAT_CUR_POWER_STACK, opp->power);
    stl_le_p(buf + SHSTAT_MAX_POWER_GPU, gpufreq_opps[0].power);
    stl_le_p(buf + SHSTAT_MAX_POWER_STACK, gpufreq_opps[0].power);
    stl_le_p(buf + SHSTAT_MIN_POWER_GPU,
             gpufreq_opps[GPUFREQ_NUM_OPPS - 1].power);
    stl_le_p(buf + SHSTAT_MIN_POWER_STACK,
             gpufreq_opps[GPUFREQ_NUM_OPPS - 1].power);
    stl_le_p(buf + SHSTAT_CUR_CEIL_GPU, s->ceil_idx);
    stl_le_p(buf + SHSTAT_CUR_FLOOR_GPU, s->floor_idx);
    stl_le_p(buf + SHSTAT_CUR_CEIL_STACK, s->ceil_idx);
    stl_le_p(buf + SHSTAT_CUR_FLOOR_STACK, s->floor_idx);
    stl_le_p(buf + SHSTAT_POWER_CONTROL, 1);
    stl_le_p(buf + SHSTAT_DVFS_STATE, 0);
    stl_le_p(buf + SHSTAT_SHADER_PRESENT, 0x3f);
    stl_le_p(buf + SHSTAT_POWER_COUNT, s->power_count);
    stl_le_p(buf + SHSTAT_DEBUG_CEIL_GPU, s->ceil_idx);
    stl_le_p(buf + SHSTAT_DEBUG_FLOOR_GPU, s->floor_idx);
    stl_le_p(buf + SHSTAT_DEBUG_LIM_GPU, 0);

    len = sizeof(buf);
    if (s->status_size && s->status_size < len) {
        len = s->status_size;
    }
    address_space_rw(&address_space_memory, s->status_base,
                     MEMTXATTRS_UNSPECIFIED, buf, len, true);
}

static void mtk_gpueb_write_opp_table(MtkGpuebState *s)
{
    uint8_t buf[GPUFREQ_NUM_OPPS * sizeof(GpufreqOppEntry)];
    hwaddr len = sizeof(buf);
    uint32_t i;

    if (!s->debug_base) {
        return;
    }

    for (i = 0; i < GPUFREQ_NUM_OPPS; i++) {
        uint8_t *p = buf + i * 24;

        stl_le_p(p + 0x00, gpufreq_opps[i].freq);
        stl_le_p(p + 0x04, gpufreq_opps[i].volt);
        stl_le_p(p + 0x08, gpufreq_opps[i].vsram);
        stl_le_p(p + 0x0c, gpufreq_opps[i].posdiv);
        stl_le_p(p + 0x10, gpufreq_opps[i].vaging);
        stl_le_p(p + 0x14, gpufreq_opps[i].power);
    }

    if (s->debug_size && s->debug_size < len) {
        len = s->debug_size;
    }
    address_space_rw(&address_space_memory, s->debug_base,
                     MEMTXATTRS_UNSPECIFIED, buf, len, true);
}

static void mtk_gpueb_zero_debug_mem(MtkGpuebState *s, hwaddr req_len)
{
    uint8_t zeros[0x400];
    hwaddr len = MIN(req_len, (hwaddr)sizeof(zeros));

    if (!s->debug_base || len == 0) {
        return;
    }
    if (s->debug_size && s->debug_size < len) {
        len = s->debug_size;
    }
    memset(zeros, 0, len);
    address_space_rw(&address_space_memory, s->debug_base,
                     MEMTXATTRS_UNSPECIFIED, zeros, len, true);
}

static void mtk_gpueb_handle_gpufreq(MtkGpuebState *s)
{
    const GpuebPinCfg *pin = &gpueb_pins[GPUEB_PIN_GPUFREQ];
    uint32_t req[8] = { 0 };
    uint32_t ack[8] = { 0 };
    uint32_t i;
    uint32_t ret = 0;

    for (i = 0; i < pin->send_len; i++) {
        req[i] = mtk_gpueb_slot_read(s, pin->send_slot + i);
    }

    switch (req[0]) {
    case GPUFREQ_CMD_INIT_SHARED_MEM:
        s->status_base = ((uint64_t)req[3] << 32) | req[2];
        s->debug_base = ((uint64_t)req[5] << 32) | req[4];
        s->status_size = req[6];
        s->debug_size = req[7];
        mtk_gpueb_sync_shared_status(s);
        ret = 0;
        break;
    case GPUFREQ_CMD_GET_FREQ_BY_IDX:
        ret = gpufreq_opps[mtk_gpueb_clamp_opp(req[2])].freq;
        break;
    case GPUFREQ_CMD_GET_POWER_BY_IDX:
        ret = gpufreq_opps[mtk_gpueb_clamp_opp(req[2])].power;
        break;
    case GPUFREQ_CMD_GET_OPPIDX_BY_FREQ:
        ret = mtk_gpueb_find_oppidx_by_freq(req[2]);
        break;
    case GPUFREQ_CMD_GET_LEAKAGE_POWER:
        ret = 50;
        break;
    case GPUFREQ_CMD_SET_LIMIT:
        s->ceil_idx = mtk_gpueb_clamp_opp(req[3]);
        s->floor_idx = mtk_gpueb_clamp_opp(req[4]);
        if (s->ceil_idx > s->floor_idx) {
            s->ceil_idx = s->floor_idx;
        }
        mtk_gpueb_sync_shared_status(s);
        ret = 0;
        break;
    case GPUFREQ_CMD_POWER_CONTROL:
        if (req[2]) {
            s->power_count++;
        } else if (s->power_count > 0) {
            s->power_count--;
        }
        mtk_gpueb_sync_shared_status(s);
        ret = s->power_count;
        break;
    case GPUFREQ_CMD_COMMIT:
        s->cur_oppidx = mtk_gpueb_clamp_opp(req[2]);
        mtk_gpueb_sync_shared_status(s);
        ret = 0;
        break;
    case GPUFREQ_CMD_GET_DEBUG_OPP_INFO:
    case GPUFREQ_CMD_GET_DEBUG_LIMIT_INFO:
        mtk_gpueb_sync_shared_status(s);
        ret = 0;
        break;
    case GPUFREQ_CMD_GET_WORKING_TABLE:
    case GPUFREQ_CMD_GET_SIGNED_TABLE:
        mtk_gpueb_write_opp_table(s);
        ret = 0;
        break;
    case GPUFREQ_CMD_GET_LIMIT_TABLE:
        mtk_gpueb_zero_debug_mem(s, 0x400);
        ret = 0;
        break;
    default:
        ret = 0;
        break;
    }

    ack[0] = req[0];
    ack[1] = req[1];
    ack[2] = ret;
    for (i = 0; i < pin->recv_len; i++) {
        mtk_gpueb_slot_write(s, pin->recv_slot + i, ack[i]);
    }

    if (s->dbg_enabled) {
        fprintf(stderr,
                "MTK-GPUEB cmd=%u target=%u arg2=%#x -> ret=%#x "
                "(status=%#" PRIx64 " debug=%#" PRIx64 ")\n",
                req[0], req[1], req[2], ret, s->status_base, s->debug_base);
    }
}

static void mtk_gpueb_dispatch_pin(MtkGpuebState *s, unsigned pin_id)
{
    const GpuebPinCfg *pin;
    unsigned i;

    if (pin_id >= GPUEB_NUM_PINS) {
        return;
    }

    pin = &gpueb_pins[pin_id];
    if (pin_id == GPUEB_PIN_GPUFREQ) {
        mtk_gpueb_handle_gpufreq(s);
    } else if (pin->recv_ack) {
        for (i = 0; i < pin->recv_len; i++) {
            mtk_gpueb_slot_write(s, pin->recv_slot + i, 0);
        }
    }

    if (pin->recv_ack) {
        s->recv_status |= (1u << pin_id);
        qemu_set_irq(s->irq, 1);
    }
}

static uint64_t mtk_gpueb_boot_read(void *opaque, hwaddr off, unsigned size)
{
    MtkGpuebState *s = opaque;
    uint64_t val = 0;
    unsigned i;

    for (i = 0; i < size && (off + i) < sizeof(s->boot_mem); i++) {
        uint8_t byte;

        if (s->started && (off + i) < 4) {
            byte = (uint8_t)(MTK_GPUEB_BOOT_MAGIC >> (8 * (off + i)));
        } else {
            byte = s->boot_mem[off + i];
        }
        val |= (uint64_t)byte << (8 * i);
    }
    return val;
}

static void mtk_gpueb_boot_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    MtkGpuebState *s = opaque;
    unsigned i;

    for (i = 0; i < size && (off + i) < sizeof(s->boot_mem); i++) {
        s->boot_mem[off + i] = (uint8_t)(val >> (8 * i));
    }
    if (off <= MTK_GPUEB_START_OFS &&
        (off + size) > MTK_GPUEB_START_OFS && val != 0) {
        s->started = true;
    }
}

static const MemoryRegionOps mtk_gpueb_boot_ops = {
    .read = mtk_gpueb_boot_read,
    .write = mtk_gpueb_boot_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    /*
     * MTK_GPUEB_BOOT_BASE (0x13c5fd24) is 4-byte aligned, so 8-byte aligned
     * guest stores (such as LK's memset over 0x13c40000..0x13c60000) arrive
     * at region offsets with (off & 7) == 4.
     */
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
        .unaligned = true,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
        .unaligned = true,
    },
};

static uint64_t mtk_gpueb_irq_read(void *opaque, hwaddr off, unsigned size)
{
    MtkGpuebState *s = opaque;
    uint32_t val = 0;

    (void)size;
    switch (off) {
    case MBOX0_OFF_SEND:
        val = 0; /* all AP->GPUEB send pins free */
        break;
    case MBOX0_OFF_RECV:
        val = s->recv_status;
        break;
    default:
        break;
    }
    return val;
}

static void mtk_gpueb_irq_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    MtkGpuebState *s = opaque;
    unsigned pin;

    (void)size;
    switch (off) {
    case MBOX0_OFF_SET:
        for (pin = 0; pin < GPUEB_NUM_PINS; pin++) {
            if (val & (1u << pin)) {
                mtk_gpueb_dispatch_pin(s, pin);
            }
        }
        break;
    case MBOX0_OFF_CLR:
        s->recv_status &= ~(uint32_t)val;
        qemu_set_irq(s->irq, s->recv_status != 0);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps mtk_gpueb_irq_ops = {
    .read = mtk_gpueb_irq_read,
    .write = mtk_gpueb_irq_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void mtk_gpueb_reset(DeviceState *dev)
{
    MtkGpuebState *s = MTK_GPUEB(dev);

    memset(s->boot_mem, 0, sizeof(s->boot_mem));
    s->started = false;
    s->recv_status = 0;
    s->status_base = 0;
    s->debug_base = 0;
    s->status_size = 0;
    s->debug_size = 0;
    s->cur_oppidx = 0;
    s->ceil_idx = 0;
    s->floor_idx = GPUFREQ_NUM_OPPS - 1;
    s->power_count = 1;
    s->dbg_enabled = mtk_gpueb_dbg_enabled();
    qemu_irq_lower(s->irq);
}

static void mtk_gpueb_realize(DeviceState *dev, Error **errp)
{
    MtkGpuebState *s = MTK_GPUEB(dev);

    (void)errp;
    memory_region_init_io(&s->iomem_boot, OBJECT(dev), &mtk_gpueb_boot_ops, s,
                          TYPE_MTK_GPUEB ".boot_mbox", MTK_GPUEB_BOOT_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem_boot);

    memory_region_init_io(&s->iomem_irq, OBJECT(dev), &mtk_gpueb_irq_ops, s,
                          TYPE_MTK_GPUEB ".mbox_irq", MTK_GPUEB_IRQ_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem_irq);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
}

static void mtk_gpueb_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    (void)data;
    dc->realize = mtk_gpueb_realize;
    dc->desc = "MediaTek MT6895 GPUEB boot and mbox0 IPI controller";
    device_class_set_legacy_reset(dc, mtk_gpueb_reset);
}

static const TypeInfo mtk_gpueb_info = {
    .name = TYPE_MTK_GPUEB,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkGpuebState),
    .class_init = mtk_gpueb_class_init,
};

static void mtk_gpueb_register_types(void)
{
    type_register_static(&mtk_gpueb_info);
}

type_init(mtk_gpueb_register_types)
