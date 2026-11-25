/*
 * MediaTek MT6895 ADSP subsystem and mailbox device
 *
 * Models the ADSP CFG/CFG2 control windows, 4-channel ADSP mailbox block,
 * and per-core DTCM backing store so ATF boot/resume handshakes and kernel
 * ADSP/audio IPI exchanges succeed without timing out.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/misc/mtk_adsp.h"
#include "hw/core/sysbus.h"
#include "system/memory.h"

#define MTK_ADSP_NUM_CORES      2
#define MTK_ADSP_NUM_MBOX       4
#define MTK_ADSP_CFG_SIZE       0x6000
#define MTK_ADSP_CFG2_SIZE      0x5000
#define MTK_ADSP_MBOX_SIZE      0x1000
#define MTK_ADSP_MBOX_DATA_SIZE 0x100
#define MTK_ADSP_RX_FIFO_DEPTH  16

/* ADSP CFG offsets driven by ATF mt6895 ADSP SMC handler and Linux adsp.ko */
#define ADSP_CFG_SW_INT         0x0000
#define ADSP_CFG_HIFI_IO_CONFIG 0x000c
#define ADSP_CFG_CORE0_MEM_PWR  0x0044
#define ADSP_CFG_CORE1_MEM_PWR  0x0048
#define ADSP_CFG_MEM_PWR_ACK    0x004c
#define ADSP_CFG_SPM_WAKEUP0    0x005c
#define ADSP_CFG_SPM_WAKEUP1    0x0060
#define ADSP_CFG_SLEEP_STATUS   0x0158

#define ADSP_HIFI_RUNSTALL_C0   (1u << 31)
#define ADSP_HIFI_RUNSTALL_C1   (1u << 30)
#define ADSP_SLEEP_C0_BITS      0x5u
#define ADSP_SLEEP_C1_BITS      0x2u

/* ADSP CFG2 register offsets */
#define ADSP_CFG2_BOOT_LOADER_CTRL 0x000c
#define ADSP_CFG2_LOADER_MODE_BIT  (1u << 1)

/* ADSP MBOX register offsets within each 0x1000 window */
#define ADSP_MBOX_OFF_SET       0x0100
#define ADSP_MBOX_OFF_CLR       0x010c

/* ADSP DTCM shared wakelock flag checked by adsp_core{0,1}_suspend() */
#define ADSP_DTCM_WAKELOCK_OFF  (MTK_ADSP_DTCM_SIZE - 8)

/* ADSP IPI packet layout & IDs (16-byte header + up to 240-byte payload) */
#define ADSP_IPI_HDR_SIZE       16
#define ADSP_IPI_MAGIC          0xdeadbeefU
#define ADSP_IPI_OPT_DEFAULT    0xffff0000U
#define ADSP_IPI_MAX_PAYLOAD    (MTK_ADSP_MBOX_DATA_SIZE - ADSP_IPI_HDR_SIZE)

#define ADSP_IPI_TEST1          1
#define ADSP_IPI_AUDIO          9
#define ADSP_IPI_ADSP_A_READY   13
#define ADSP_IPI_ADSP_A_SUSPEND 21

#define AUDIO_IPI_MAGIC         0x8888U
#define AUDIO_IPI_MSG_NEED_ACK  1
#define AUDIO_IPI_MSG_ACK_BACK  2

typedef struct MtkAdspMboxCtx {
    MtkAdspState *s;
    unsigned idx;
} MtkAdspMboxCtx;

struct MtkAdspState {
    SysBusDevice parent_obj;

    MemoryRegion cfg_mr;
    MemoryRegion mbox_mr[MTK_ADSP_NUM_MBOX];
    MemoryRegion cfg2_mr;
    qemu_irq irq[MTK_ADSP_NUM_MBOX];

    MtkAdspMboxCtx mbox_ctx[MTK_ADSP_NUM_MBOX];

    uint8_t cfg[MTK_ADSP_CFG_SIZE];
    uint8_t cfg2[MTK_ADSP_CFG2_SIZE];
    uint8_t mbox_data[MTK_ADSP_NUM_MBOX][MTK_ADSP_MBOX_DATA_SIZE];
    uint8_t mbox_regs[MTK_ADSP_NUM_MBOX][MTK_ADSP_MBOX_SIZE -
                                         MTK_ADSP_MBOX_DATA_SIZE];
    uint32_t mbox_irq_set[MTK_ADSP_NUM_MBOX];
    uint32_t mbox_irq_clr[MTK_ADSP_NUM_MBOX];

    uint8_t rx_fifo[MTK_ADSP_NUM_CORES][MTK_ADSP_RX_FIFO_DEPTH]
                   [MTK_ADSP_MBOX_DATA_SIZE];
    uint32_t rx_head[MTK_ADSP_NUM_CORES];
    uint32_t rx_count[MTK_ADSP_NUM_CORES];

    uint8_t dtcm[MTK_ADSP_NUM_CORES][MTK_ADSP_DTCM_SIZE];

    bool responder_enabled;
    bool trace_enabled;
};

static bool mtk_adsp_env_enabled(const char *name)
{
    const char *v = getenv(name);

    return v && v[0] && strcmp(v, "0") != 0;
}

static uint64_t mtk_adsp_read_bytes(const uint8_t *buf, hwaddr off,
                                    unsigned size)
{
    uint64_t val = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        val |= (uint64_t)buf[off + i] << (8 * i);
    }
    return val;
}

static void mtk_adsp_write_bytes(uint8_t *buf, hwaddr off, uint64_t val,
                                 unsigned size)
{
    unsigned i;

    for (i = 0; i < size; i++) {
        buf[off + i] = (uint8_t)(val >> (8 * i));
    }
}

uint64_t mtk_adsp_dtcm_read(DeviceState *dev, unsigned core, hwaddr off,
                            unsigned size)
{
    MtkAdspState *s = MTK_ADSP(dev);

    if (core >= MTK_ADSP_NUM_CORES || size > 8 ||
        off + size > MTK_ADSP_DTCM_SIZE) {
        return 0;
    }
    return mtk_adsp_read_bytes(s->dtcm[core], off, size);
}

void mtk_adsp_dtcm_write(DeviceState *dev, unsigned core, hwaddr off,
                         uint64_t val, unsigned size)
{
    MtkAdspState *s = MTK_ADSP(dev);

    if (core >= MTK_ADSP_NUM_CORES || size > 8 ||
        off + size > MTK_ADSP_DTCM_SIZE) {
        return;
    }
    mtk_adsp_write_bytes(s->dtcm[core], off, val, size);
}

static void mtk_adsp_deliver_rx(MtkAdspState *s, unsigned core,
                                const uint8_t *msg)
{
    unsigned rx_mbox = core * 2 + 1;

    if (s->mbox_irq_clr[rx_mbox] == 0) {
        memcpy(s->mbox_data[rx_mbox], msg, MTK_ADSP_MBOX_DATA_SIZE);
        s->mbox_irq_clr[rx_mbox] = 1;
        stl_le_p(&s->mbox_regs[rx_mbox][ADSP_MBOX_OFF_CLR -
                                        MTK_ADSP_MBOX_DATA_SIZE], 1);
        if (s->trace_enabled) {
            fprintf(stderr, "MTK-ADSP rx core=%u mbox=%u id=%u len=%u\n",
                    core, rx_mbox, ldl_le_p(msg), ldl_le_p(msg + 4));
        }
        qemu_set_irq(s->irq[rx_mbox], 1);
        return;
    }

    if (s->rx_count[core] < MTK_ADSP_RX_FIFO_DEPTH) {
        uint32_t tail = (s->rx_head[core] + s->rx_count[core]) %
                        MTK_ADSP_RX_FIFO_DEPTH;
        memcpy(s->rx_fifo[core][tail], msg, MTK_ADSP_MBOX_DATA_SIZE);
        s->rx_count[core]++;
    }
}

static void mtk_adsp_post_ipi(MtkAdspState *s, unsigned core, uint32_t id,
                              const void *payload, uint32_t len)
{
    uint8_t msg[MTK_ADSP_MBOX_DATA_SIZE];
    uint32_t copy_len = MIN(len, ADSP_IPI_MAX_PAYLOAD);

    memset(msg, 0, sizeof(msg));
    stl_le_p(msg + 0x00, id);
    stl_le_p(msg + 0x04, copy_len);
    stl_le_p(msg + 0x08, ADSP_IPI_OPT_DEFAULT);
    stl_le_p(msg + 0x0c, ADSP_IPI_MAGIC);
    if (payload && copy_len > 0) {
        memcpy(msg + ADSP_IPI_HDR_SIZE, payload, copy_len);
    }
    mtk_adsp_deliver_rx(s, core, msg);
}

static void mtk_adsp_start_core(MtkAdspState *s, unsigned core)
{
    uint32_t sleep = ldl_le_p(&s->cfg[ADSP_CFG_SLEEP_STATUS]);
    uint32_t c_le = core;

    if (s->trace_enabled) {
        fprintf(stderr, "MTK-ADSP start core=%u\n", core);
    }
    sleep &= ~(core == 0 ? ADSP_SLEEP_C0_BITS : ADSP_SLEEP_C1_BITS);
    stl_le_p(&s->cfg[ADSP_CFG_SLEEP_STATUS], sleep);
    stl_le_p(&s->dtcm[core][ADSP_DTCM_WAKELOCK_OFF], 0);
    mtk_adsp_post_ipi(s, core, ADSP_IPI_ADSP_A_READY, &c_le, sizeof(c_le));
}

static void mtk_adsp_handle_tx(MtkAdspState *s, unsigned mbox)
{
    unsigned core = mbox / 2;
    const uint8_t *msg = s->mbox_data[mbox];
    uint32_t id = ldl_le_p(msg + 0x00);
    uint32_t len = ldl_le_p(msg + 0x04);
    const uint8_t *payload = msg + ADSP_IPI_HDR_SIZE;

    if (s->trace_enabled) {
        fprintf(stderr, "MTK-ADSP tx core=%u mbox=%u id=%u len=%u\n",
                core, mbox, id, len);
    }

    switch (id) {
    case ADSP_IPI_ADSP_A_SUSPEND: {
        uint32_t sleep = ldl_le_p(&s->cfg[ADSP_CFG_SLEEP_STATUS]);
        uint32_t c_le = core;

        sleep |= (core == 0 ? ADSP_SLEEP_C0_BITS : ADSP_SLEEP_C1_BITS);
        stl_le_p(&s->cfg[ADSP_CFG_SLEEP_STATUS], sleep);
        stl_le_p(&s->dtcm[core][ADSP_DTCM_WAKELOCK_OFF], 1);
        mtk_adsp_post_ipi(s, core, ADSP_IPI_ADSP_A_SUSPEND, &c_le,
                          sizeof(c_le));
        break;
    }
    case ADSP_IPI_AUDIO:
        if (len >= 16 && len <= ADSP_IPI_MAX_PAYLOAD &&
            lduw_le_p(payload) == AUDIO_IPI_MAGIC &&
            payload[5] == AUDIO_IPI_MSG_NEED_ACK) {
            uint8_t reply[ADSP_IPI_MAX_PAYLOAD];
            uint8_t layer = payload[3];

            memcpy(reply, payload, len);
            reply[3] = (uint8_t)(((layer & 0x0f) << 4) |
                                 ((layer >> 4) & 0x0f));
            reply[5] = AUDIO_IPI_MSG_ACK_BACK;
            mtk_adsp_post_ipi(s, core, ADSP_IPI_AUDIO, reply, len);
        }
        break;
    case ADSP_IPI_TEST1:
        mtk_adsp_post_ipi(s, core, ADSP_IPI_TEST1, payload,
                          MIN(len, ADSP_IPI_MAX_PAYLOAD));
        break;
    default:
        break;
    }
}

static uint64_t mtk_adsp_cfg_read(void *opaque, hwaddr off, unsigned size)
{
    MtkAdspState *s = opaque;

    if (off + size > MTK_ADSP_CFG_SIZE) {
        return 0;
    }
    return mtk_adsp_read_bytes(s->cfg, off, size);
}

static void mtk_adsp_cfg_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    MtkAdspState *s = opaque;
    uint32_t old_val = 0;

    if (off + size > MTK_ADSP_CFG_SIZE) {
        return;
    }
    if (off == ADSP_CFG_HIFI_IO_CONFIG && size == 4) {
        old_val = ldl_le_p(&s->cfg[ADSP_CFG_HIFI_IO_CONFIG]);
    }

    mtk_adsp_write_bytes(s->cfg, off, val, size);
    if (s->trace_enabled && size == 4 &&
        (off == ADSP_CFG_SW_INT || off == ADSP_CFG_HIFI_IO_CONFIG ||
         off == ADSP_CFG_CORE0_MEM_PWR || off == ADSP_CFG_CORE1_MEM_PWR ||
         off == ADSP_CFG_MEM_PWR_ACK || off == ADSP_CFG_SLEEP_STATUS)) {
        fprintf(stderr,
                "MTK-ADSP cfg W off=%#x old=%#x val=%#x ldr=%#x pwr0=%#x pwr1=%#x\n",
                (unsigned)off, old_val, (uint32_t)val,
                (uint32_t)ldl_le_p(&s->cfg2[ADSP_CFG2_BOOT_LOADER_CTRL]),
                (uint32_t)ldl_le_p(&s->cfg[ADSP_CFG_CORE0_MEM_PWR]),
                (uint32_t)ldl_le_p(&s->cfg[ADSP_CFG_CORE1_MEM_PWR]));
    }

    if (!s->responder_enabled || size != 4) {
        return;
    }

    if (off == ADSP_CFG_CORE0_MEM_PWR || off == ADSP_CFG_CORE1_MEM_PWR) {
        uint32_t p0 = ldl_le_p(&s->cfg[ADSP_CFG_CORE0_MEM_PWR]);
        uint32_t p1 = ldl_le_p(&s->cfg[ADSP_CFG_CORE1_MEM_PWR]);

        stl_le_p(&s->cfg[ADSP_CFG_MEM_PWR_ACK], (p0 | p1) & 0x1fu);
    } else if (off == ADSP_CFG_HIFI_IO_CONFIG) {
        uint32_t new_val = (uint32_t)val;
        /*
         * LK's mt_adsp_load_image briefly releases RUNSTALL with CFG2 +0x0c
         * bit 1 (loader mode) set and CORE*_MEM_PWR == 0. Only boot cores on
         * Linux runtime un-stalls where loader mode is clear and mem pwr acks
         * are armed (0x1f).
         */
        bool loader_mode = (ldl_le_p(&s->cfg2[ADSP_CFG2_BOOT_LOADER_CTRL]) &
                            ADSP_CFG2_LOADER_MODE_BIT) != 0;
        uint32_t pwr0 = ldl_le_p(&s->cfg[ADSP_CFG_CORE0_MEM_PWR]);
        uint32_t pwr1 = ldl_le_p(&s->cfg[ADSP_CFG_CORE1_MEM_PWR]);

        if ((old_val & ADSP_HIFI_RUNSTALL_C0) &&
            !(new_val & ADSP_HIFI_RUNSTALL_C0) &&
            !loader_mode && (pwr0 & 0x1fu) == 0x1fu) {
            mtk_adsp_start_core(s, 0);
        }
        if ((old_val & ADSP_HIFI_RUNSTALL_C1) &&
            !(new_val & ADSP_HIFI_RUNSTALL_C1) &&
            !loader_mode && (pwr1 & 0x1fu) == 0x1fu) {
            mtk_adsp_start_core(s, 1);
        }
    }
}

static const MemoryRegionOps mtk_adsp_cfg_ops = {
    .read = mtk_adsp_cfg_read,
    .write = mtk_adsp_cfg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static uint64_t mtk_adsp_cfg2_read(void *opaque, hwaddr off, unsigned size)
{
    MtkAdspState *s = opaque;

    if (off + size > MTK_ADSP_CFG2_SIZE) {
        return 0;
    }
    return mtk_adsp_read_bytes(s->cfg2, off, size);
}

static void mtk_adsp_cfg2_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    MtkAdspState *s = opaque;

    if (off + size > MTK_ADSP_CFG2_SIZE) {
        return;
    }
    mtk_adsp_write_bytes(s->cfg2, off, val, size);
    if (s->trace_enabled && size == 4 &&
        (off == 0x0004 || off == ADSP_CFG2_BOOT_LOADER_CTRL ||
         off == 0x4a00)) {
        fprintf(stderr, "MTK-ADSP cfg2 W off=%#x val=%#x\n",
                (unsigned)off, (uint32_t)val);
    }
}

static const MemoryRegionOps mtk_adsp_cfg2_ops = {
    .read = mtk_adsp_cfg2_read,
    .write = mtk_adsp_cfg2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static uint64_t mtk_adsp_mbox_read(void *opaque, hwaddr off, unsigned size)
{
    MtkAdspMboxCtx *ctx = opaque;
    MtkAdspState *s = ctx->s;
    unsigned m = ctx->idx;

    if (off + size <= MTK_ADSP_MBOX_DATA_SIZE) {
        return mtk_adsp_read_bytes(s->mbox_data[m], off, size);
    }
    if (off == ADSP_MBOX_OFF_SET && size == 4) {
        return s->mbox_irq_set[m];
    }
    if (off == ADSP_MBOX_OFF_CLR && size == 4) {
        return s->mbox_irq_clr[m];
    }
    if (off >= MTK_ADSP_MBOX_DATA_SIZE && off + size <= MTK_ADSP_MBOX_SIZE) {
        return mtk_adsp_read_bytes(s->mbox_regs[m],
                                   off - MTK_ADSP_MBOX_DATA_SIZE, size);
    }
    return 0;
}

static void mtk_adsp_mbox_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    MtkAdspMboxCtx *ctx = opaque;
    MtkAdspState *s = ctx->s;
    unsigned m = ctx->idx;

    if (off + size <= MTK_ADSP_MBOX_DATA_SIZE) {
        mtk_adsp_write_bytes(s->mbox_data[m], off, val, size);
        return;
    }
    if (off < MTK_ADSP_MBOX_DATA_SIZE || off + size > MTK_ADSP_MBOX_SIZE) {
        return;
    }

    mtk_adsp_write_bytes(s->mbox_regs[m], off - MTK_ADSP_MBOX_DATA_SIZE,
                         val, size);

    if (size != 4) {
        return;
    }

    if (off == ADSP_MBOX_OFF_SET) {
        if (s->trace_enabled) {
            fprintf(stderr, "MTK-ADSP mbox%u SET val=%#x\n",
                    m, (uint32_t)val);
        }
        if (!s->responder_enabled) {
            s->mbox_irq_set[m] |= (uint32_t)val;
            stl_le_p(&s->mbox_regs[m][ADSP_MBOX_OFF_SET -
                                      MTK_ADSP_MBOX_DATA_SIZE],
                     s->mbox_irq_set[m]);
            return;
        }
        if ((m == 0 || m == 2) && (val & 1u)) {
            s->mbox_irq_set[m] = 0;
            stl_le_p(&s->mbox_regs[m][ADSP_MBOX_OFF_SET -
                                      MTK_ADSP_MBOX_DATA_SIZE], 0);
            mtk_adsp_handle_tx(s, m);
        }
    } else if (off == ADSP_MBOX_OFF_CLR) {
        s->mbox_irq_clr[m] &= ~(uint32_t)val;
        if (s->trace_enabled) {
            fprintf(stderr, "MTK-ADSP mbox%u CLR val=%#x rem=%#x\n",
                    m, (uint32_t)val, s->mbox_irq_clr[m]);
        }
        stl_le_p(&s->mbox_regs[m][ADSP_MBOX_OFF_CLR -
                                  MTK_ADSP_MBOX_DATA_SIZE],
                 s->mbox_irq_clr[m]);
        if (s->mbox_irq_clr[m] == 0) {
            qemu_set_irq(s->irq[m], 0);
            if (s->responder_enabled && (m == 1 || m == 3)) {
                unsigned core = m / 2;

                if (s->rx_count[core] > 0) {
                    uint8_t next[MTK_ADSP_MBOX_DATA_SIZE];

                    memcpy(next, s->rx_fifo[core][s->rx_head[core]],
                           sizeof(next));
                    s->rx_head[core] = (s->rx_head[core] + 1) %
                                       MTK_ADSP_RX_FIFO_DEPTH;
                    s->rx_count[core]--;
                    mtk_adsp_deliver_rx(s, core, next);
                }
            }
        }
    }
}

static const MemoryRegionOps mtk_adsp_mbox_ops = {
    .read = mtk_adsp_mbox_read,
    .write = mtk_adsp_mbox_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static void mtk_adsp_reset(DeviceState *dev)
{
    MtkAdspState *s = MTK_ADSP(dev);
    unsigned i;

    memset(s->cfg, 0, sizeof(s->cfg));
    memset(s->cfg2, 0, sizeof(s->cfg2));
    memset(s->mbox_data, 0, sizeof(s->mbox_data));
    memset(s->mbox_regs, 0, sizeof(s->mbox_regs));
    memset(s->mbox_irq_set, 0, sizeof(s->mbox_irq_set));
    memset(s->mbox_irq_clr, 0, sizeof(s->mbox_irq_clr));
    memset(s->rx_fifo, 0, sizeof(s->rx_fifo));
    memset(s->rx_head, 0, sizeof(s->rx_head));
    memset(s->rx_count, 0, sizeof(s->rx_count));
    memset(s->dtcm, 0, sizeof(s->dtcm));

    for (i = 0; i < MTK_ADSP_NUM_MBOX; i++) {
        qemu_set_irq(s->irq[i], 0);
    }
}

static void mtk_adsp_realize(DeviceState *dev, Error **errp)
{
    MtkAdspState *s = MTK_ADSP(dev);
    unsigned i;

    s->responder_enabled = !mtk_adsp_env_enabled("MTK_NO_ADSP");
    s->trace_enabled = mtk_adsp_env_enabled("MTK_ADSP_LOG");

    memory_region_init_io(&s->cfg_mr, OBJECT(dev), &mtk_adsp_cfg_ops, s,
                          TYPE_MTK_ADSP ".cfg", MTK_ADSP_CFG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->cfg_mr);

    for (i = 0; i < MTK_ADSP_NUM_MBOX; i++) {
        g_autofree char *name = g_strdup_printf(TYPE_MTK_ADSP ".mbox%u", i);

        s->mbox_ctx[i].s = s;
        s->mbox_ctx[i].idx = i;
        memory_region_init_io(&s->mbox_mr[i], OBJECT(dev), &mtk_adsp_mbox_ops,
                              &s->mbox_ctx[i], name, MTK_ADSP_MBOX_SIZE);
        sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mbox_mr[i]);
        sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq[i]);
    }

    memory_region_init_io(&s->cfg2_mr, OBJECT(dev), &mtk_adsp_cfg2_ops, s,
                          TYPE_MTK_ADSP ".cfg2", MTK_ADSP_CFG2_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->cfg2_mr);
}

static void mtk_adsp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mtk_adsp_realize;
    dc->desc = "MediaTek MT6895 ADSP subsystem";
    device_class_set_legacy_reset(dc, mtk_adsp_reset);
}

static const TypeInfo mtk_adsp_info = {
    .name = TYPE_MTK_ADSP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkAdspState),
    .class_init = mtk_adsp_class_init,
};

static void mtk_adsp_register_types(void)
{
    type_register_static(&mtk_adsp_info);
}

type_init(mtk_adsp_register_types)
