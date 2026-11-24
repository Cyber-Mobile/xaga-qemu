/*
 * MediaTek tinysys mailbox
 *
 * SCMI on this SoC does not use an SMC conduit -- the device tree gives it a
 * mailbox and two shared-memory windows:
 *
 *   scmi: scmi {
 *       mboxes = <&tinysys_mbox 0>, <&tinysys_mbox 1>;
 *       shmem  = <&scmi_tx_shmem>, <&scmi_rx_shmem>;
 *   };
 *   tinysys_mbox: tinysys_mbox@1C351000 {
 *       reg = <0x1c351000 0x1000>, <0x1c361000 0x1000>;
 *       interrupts = <0 0x123 4 0>, <0 0x124 4 0>;
 *   };
 *
 * Neither window was mapped, so the first thing to do is make them exist and
 * observe what the driver does with them -- an unmapped window aborts rather
 * than telling us anything. The registers are a plain read/write store for
 * now, with a trace, and the two interrupt lines are wired but never raised:
 * answering SCMI needs the protocol side (the responder that reads the shared
 * memory and signals completion), which is the next layer.
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
#include "trace.h"
#include "exec/cpu-common.h"
#include "system/address-spaces.h"
#include "system/memory.h"

#define TYPE_MTK_TINYSYS_MBOX "mtk-tinysys-mbox"
OBJECT_DECLARE_SIMPLE_TYPE(MtkTinysysMboxState, MTK_TINYSYS_MBOX)

#define MTK_TINYSYS_MBOX_NUM_IRQ    2

struct MtkTinysysMboxState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion iomem_rx;
    qemu_irq irq[MTK_TINYSYS_MBOX_NUM_IRQ];
    uint32_t *regs;
    uint32_t size;
    uint32_t *regs_rx;
    uint32_t size_rx;
    uint64_t shmem_base;        /* SCMI TX shared memory (agent -> platform) */
    uint64_t shmem_rx_base;     /* SCMI RX shared memory (platform -> agent) */
    uint32_t shmem_size;
    bool scmi_agent;
    uint32_t scmi_replies;
};

/*
 * Minimal SCMI platform responder.
 *
 * Firmware side (dts: tinysys_mbox@1C351000): tinysys_mbox has two register
 * windows (0x1c351000 for TX/A2P chan[0] / GIC_SPI 291, 0x1c361000 for
 * RX/P2A chan[1] / GIC_SPI 292) and two 0x80-byte shared-memory windows
 * (0x1c350000 for TX/A2P, 0x1c360000 for RX/P2A). mtk-mbox-mailbox.ko
 * implements the transport: tinysys_mbox_send_data writes profiling timestamps
 * at window+0x70/+0x74 and rings the doorbell by writing 1 to window0+0x00,
 * while tinysys_mbox_rx_interrupt tests window+0x04 and clears it (W1C) with 1
 * before forwarding to mbox_chan_received_data -> scmi_rx_callback.
 *
 * Per ARM DEN0056 and Linux arm_scmi/shmem.c:
 *   +0x00 reserved        +0x04 channel_status (bit0 FREE, bit1 ERROR)
 *   +0x10 flags (bit0 INTR) +0x14 length (4-byte header + payload bytes)
 *   +0x18 message header  +0x1c payload[0] = return status, then return values
 *
 * Message header layout (DEN0056):
 *   [7:0] message_id, [9:8] message_type, [17:10] protocol_id, [27:18] token.
 *
 * Synchronous A2P replies are written back in place into the TX shared memory
 * window (shmem_base): set CHANNEL_FREE, clear message_type [9:8] to 0 while
 * preserving protocol_id/msg_id/token, status=0 (SCMI_SUCCESS), and payload.
 * Raise chan[0] IRQ (irq[0], window0+0x04) only when flags bit0 (INTR) is set;
 * polling transfers inspect CHANNEL_FREE in shmem_base directly.
 * MTK_NO_SCMI=1 disables the responder; MTK_SCMILOG=1 logs every reply.
 */
#define MTK_SCMI_OFF_STATUS          0x04
#define MTK_SCMI_OFF_FLAGS           0x10
#define MTK_SCMI_OFF_LENGTH          0x14
#define MTK_SCMI_OFF_HDR             0x18
#define MTK_SCMI_OFF_PAYLOAD         0x1c

#define MTK_SCMI_CHAN_FREE           0x1
#define MTK_SCMI_FLAG_INTR           0x1
#define MTK_SCMI_PROTO_BASE          0x10
#define MTK_SCMI_PROTO_TINYSYS       0x80
#define MTK_SCMI_TINYSYS_NUM_SOURCES 20

static bool mtk_scmi_off(void)
{
    static int cached = -1;

    if (cached < 0) {
        cached = getenv("MTK_NO_SCMI") ? 1 : 0;
    }
    return cached;
}

static bool mtk_scmi_verbose(void)
{
    static int cached = -1;

    if (cached < 0) {
        cached = getenv("MTK_SCMILOG") ? 1 : 0;
    }
    return cached;
}

static void mtk_scmi_shmem_read(MtkTinysysMboxState *s, uint64_t addr,
                                void *buf, hwaddr len)
{
    address_space_rw(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                     buf, len, false);
}

static void mtk_scmi_shmem_write(MtkTinysysMboxState *s, uint64_t addr,
                                 const void *buf, hwaddr len)
{
    address_space_rw(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                     (void *)buf, len, true);
}

static unsigned mtk_scmi_fill_payload(MtkTinysysMboxState *s,
                                      const uint8_t *tx, uint32_t protocol,
                                      uint32_t msg_id, uint8_t *pl)
{
    uint32_t v;

    switch (protocol) {
    case MTK_SCMI_PROTO_BASE:
        switch (msg_id) {
        case 0x0:   /* PROTOCOL_VERSION */
            stl_le_p(pl, 0x00020000);
            return 4;
        case 0x1:   /* PROTOCOL_ATTRIBUTES: u8 num_protocols, u8 num_agents */
            stl_le_p(pl, 1 | (1 << 8));
            return 4;
        case 0x2:   /* PROTOCOL_MESSAGE_ATTRIBUTES */
            stl_le_p(pl, 0);
            return 4;
        case 0x3:   /* DISCOVER_VENDOR */
        case 0x4:   /* DISCOVER_SUB_VENDOR */
            memset(pl, 0, 16);
            memcpy(pl, "MTK", 3);
            return 16;
        case 0x5:   /* DISCOVER_IMPLEMENTATION_VERSION */
            stl_le_p(pl, 0x00010000);
            return 4;
        case 0x6:   /* DISCOVER_LIST_PROTOCOLS: u32 num_ret, u8 list[] */
            v = ldl_le_p(tx + MTK_SCMI_OFF_PAYLOAD);   /* num_skip */
            if (v == 0) {
                stl_le_p(pl, 1);
                pl[4] = MTK_SCMI_PROTO_TINYSYS;
                return 8;
            }
            stl_le_p(pl, 0);
            return 4;
        case 0x7:   /* DISCOVER_AGENT */
            memset(pl, 0, 20);
            stl_le_p(pl, 0);
            memcpy(pl + 4, "OSPM", 4);
            return 20;
        default:    /* NOTIFY_ERRORS / SET_DEVICE_PERMISSIONS / ... */
            return 0;
        }
    case MTK_SCMI_PROTO_TINYSYS:
        /*
         * Vendor protocol 0x80 (tinysys-scmi.ko):
         *   msg 0: PROTOCOL_VERSION (4B)
         *   msg 1: PROTOCOL_ATTRIBUTES -> num_sources (4B, cb_array[20])
         *   msg 2: PROTOCOL_MESSAGE_ATTRIBUTES (4B)
         *   msg 3: TINYSYS_COMMON_SET (status only)
         *   msg 4: TINYSYS_COMMON_GET (12B state response)
         *   msg 5: TINYSYS_EVENT_NOTIFY (status only)
         */
        switch (msg_id) {
        case 0x0:
            stl_le_p(pl, 0x00010000);
            return 4;
        case 0x1:
            stl_le_p(pl, MTK_SCMI_TINYSYS_NUM_SOURCES);
            return 4;
        case 0x2:
            stl_le_p(pl, 0);
            return 4;
        case 0x4:
            memset(pl, 0, 12);
            return 12;
        case 0x3:
        case 0x5:
        default:
            return 0;
        }
    default:
        return 0;
    }
}

static void mtk_scmi_handle(MtkTinysysMboxState *s)
{
    uint8_t tx[0x70], reply[0x70], pl[0x50];
    uint32_t flags, hdr, protocol, msg_id, token, plen;

    memset(reply, 0, sizeof(reply));
    memset(pl, 0, sizeof(pl));
    mtk_scmi_shmem_read(s, s->shmem_base, tx, sizeof(tx));

    flags = ldl_le_p(tx + MTK_SCMI_OFF_FLAGS);
    hdr = ldl_le_p(tx + MTK_SCMI_OFF_HDR);
    msg_id = hdr & 0xff;
    protocol = (hdr >> 10) & 0xff;
    token = (hdr >> 18) & 0x3ff;

    plen = mtk_scmi_fill_payload(s, tx, protocol, msg_id, pl);
    if (plen > sizeof(pl)) {
        plen = sizeof(pl);
    }

    stl_le_p(reply + MTK_SCMI_OFF_STATUS, MTK_SCMI_CHAN_FREE);
    stl_le_p(reply + MTK_SCMI_OFF_LENGTH, 8 + plen);
    stl_le_p(reply + MTK_SCMI_OFF_HDR, hdr & ~(3u << 8));
    stl_le_p(reply + MTK_SCMI_OFF_PAYLOAD, 0);         /* status: success */
    memcpy(reply + MTK_SCMI_OFF_PAYLOAD + 4, pl, plen);
    mtk_scmi_shmem_write(s, s->shmem_base, reply, sizeof(reply));

    if (flags & MTK_SCMI_FLAG_INTR) {
        s->regs[MTK_SCMI_OFF_STATUS / 4] = 1;
        qemu_set_irq(s->irq[0], 1);
    }
    s->scmi_replies++;

    if (mtk_scmi_verbose() || s->scmi_replies <= 24) {
        fprintf(stderr, "MTK-SCMI reply %u: proto=%#x msg=%u token=%u"
                " flags=%#x len=%u req=%08x %08x %08x\n",
                s->scmi_replies, protocol, msg_id, token, flags, 8 + plen,
                ldl_le_p(tx + MTK_SCMI_OFF_PAYLOAD),
                ldl_le_p(tx + MTK_SCMI_OFF_PAYLOAD + 4),
                ldl_le_p(tx + MTK_SCMI_OFF_PAYLOAD + 8));
    }
}

static uint64_t mtk_tinysys_mbox_read(void *opaque, hwaddr offset, unsigned size)
{
    MtkTinysysMboxState *s = MTK_TINYSYS_MBOX(opaque);
    uint64_t val = 0;
    unsigned int i;

    for (i = 0; i < size; i += 4) {
        hwaddr o = offset + i;

        if (o + 4 <= s->size) {
            uint32_t r = s->regs[o / 4];
            if (o == 0x14) {
                static int nack;

                if (nack < 10) {
                    nack++;
                    fprintf(stderr, "MTK-MBOX-ACK read off=%#x\n", (unsigned)o);
                }
                r |= 0x20; /* mailbox always has ACK bit set */
            }
            val |= (uint64_t)r << (8 * i);
        }
    }
    trace_mtk_tinysys_mbox_read(offset, val);
    return val;
}

/*
 * DIAGNOSTIC (2026-11-08): ATF's console backend list (head 0x48253b50) has a
 * node whose handler is the tinysys-mailbox writer 0x48204cfc: it polls the
 * ack bit at +0x14, then stores one character at +0x00. With no tinysys
 * firmware on the other side, everything ATF prints -- including its EL3
 * panic dump (fault PC + ESR_EL3 + all sysregs) -- was being written into
 * this window and discarded. Echo the characters so the panic text reaches
 * the QEMU stderr log.
 */
static void mtk_tinysys_mbox_echo_char(uint64_t value)
{
    static char line[256];
    static unsigned len;
    char c = (char)(value & 0xff);

    if (c == '\0') {
        return;
    }
    if (c == '\n' || len == sizeof(line) - 1) {
        line[len] = '\0';
        if (len) {
            fprintf(stderr, "ATF-CONSOLE: %s\n", line);
        }
        len = 0;
        return;
    }
    line[len++] = c;
}

static void mtk_tinysys_mbox_write(void *opaque, hwaddr offset, uint64_t value,
                                   unsigned size)
{
    MtkTinysysMboxState *s = MTK_TINYSYS_MBOX(opaque);
    unsigned int i;

    trace_mtk_tinysys_mbox_write(offset, value);

    if (offset == 0x00) {
        /*
         * The transport driver's doorbell is "write 1 to window0+0x00" and the
         * register reads back 0 once the transfer is queued.  Every other value
         * at this offset is ATF's console character path.
         */
        if (s->scmi_agent && value == 1) {
            mtk_scmi_handle(s);
            s->regs[0] = 0;
            return;
        }
        mtk_tinysys_mbox_echo_char(value);
    }

    if (offset == MTK_SCMI_OFF_STATUS) {
        s->regs[MTK_SCMI_OFF_STATUS / 4] = 0;
        qemu_set_irq(s->irq[0], 0);
        return;
    }

    for (i = 0; i < size; i += 4) {
        hwaddr o = offset + i;

        if (o + 4 <= s->size) {
            s->regs[o / 4] = (value >> (8 * i)) & 0xffffffff;
        }
    }
}

static const MemoryRegionOps mtk_tinysys_mbox_ops = {
    .read = mtk_tinysys_mbox_read,
    .write = mtk_tinysys_mbox_write,
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
 * Receive-status window (0x1c361000).  tinysys_mbox_rx_interrupt reads +0x04,
 * and a non-zero value means "a reply is waiting"; writing 1 clears it.
 */
static uint64_t mtk_tinysys_mbox_rx_read(void *opaque, hwaddr offset,
                                         unsigned size)
{
    MtkTinysysMboxState *s = MTK_TINYSYS_MBOX(opaque);
    uint64_t val = 0;
    unsigned int i;

    for (i = 0; i < size; i += 4) {
        hwaddr o = offset + i;

        if (o + 4 <= s->size_rx) {
            val |= (uint64_t)s->regs_rx[o / 4] << (8 * i);
        }
    }
    return val;
}

static void mtk_tinysys_mbox_rx_write(void *opaque, hwaddr offset,
                                      uint64_t value, unsigned size)
{
    MtkTinysysMboxState *s = MTK_TINYSYS_MBOX(opaque);

    if (offset == MTK_SCMI_OFF_STATUS) {
        s->regs_rx[MTK_SCMI_OFF_STATUS / 4] = 0;
        qemu_set_irq(s->irq[1], 0);
        return;
    }
    if (offset + 4 <= s->size_rx) {
        s->regs_rx[offset / 4] = (uint32_t)value;
    }
}

static const MemoryRegionOps mtk_tinysys_mbox_rx_ops = {
    .read = mtk_tinysys_mbox_rx_read,
    .write = mtk_tinysys_mbox_rx_write,
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

static void mtk_tinysys_mbox_reset(DeviceState *dev)
{
    MtkTinysysMboxState *s = MTK_TINYSYS_MBOX(dev);
    int i;

    memset(s->regs, 0, s->size);
    memset(s->regs_rx, 0, s->size_rx);
    s->scmi_replies = 0;
    for (i = 0; i < MTK_TINYSYS_MBOX_NUM_IRQ; i++) {
        qemu_set_irq(s->irq[i], 0);
    }
}

static void mtk_tinysys_mbox_realize(DeviceState *dev, Error **errp)
{
    MtkTinysysMboxState *s = MTK_TINYSYS_MBOX(dev);
    int i;

    memory_region_init_io(&s->iomem, OBJECT(dev), &mtk_tinysys_mbox_ops, s,
                          TYPE_MTK_TINYSYS_MBOX, s->size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    memory_region_init_io(&s->iomem_rx, OBJECT(dev),
                          &mtk_tinysys_mbox_rx_ops, s,
                          TYPE_MTK_TINYSYS_MBOX ".rx", s->size_rx);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem_rx);

    /*
     * The SCMI shared windows are separate RAM regions owned by the machine
     * (mt6895.scmi-tx-shmem / mt6895.scmi-rx-shmem); this device only reads and
     * writes them through the address space.  Mapping them here as RAM shadowed
     * the SPM/SRAMP area and broke SCPSYS power domains , so
     * the windows stay with the machine and the transport is modelled here.
     */
    s->scmi_agent = !mtk_scmi_off();
    for (i = 0; i < MTK_TINYSYS_MBOX_NUM_IRQ; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq[i]);
    }
    s->regs = g_new0(uint32_t, s->size / 4);
    s->regs_rx = g_new0(uint32_t, s->size_rx / 4);
}

static void mtk_tinysys_mbox_finalize(Object *obj)
{
    MtkTinysysMboxState *s = MTK_TINYSYS_MBOX(obj);

    g_free(s->regs);
    g_free(s->regs_rx);
}

static const Property mtk_tinysys_mbox_props[] = {
    DEFINE_PROP_UINT32("size", MtkTinysysMboxState, size, 0x1000),
    DEFINE_PROP_UINT32("rx-size", MtkTinysysMboxState, size_rx, 0x1000),
    DEFINE_PROP_UINT64("shmem-base", MtkTinysysMboxState, shmem_base, 0x1c350000),
    DEFINE_PROP_UINT64("shmem-rx-base", MtkTinysysMboxState, shmem_rx_base,
                       0x1c360000),
    DEFINE_PROP_UINT32("shmem-size", MtkTinysysMboxState, shmem_size, 0x1000),
};

static void mtk_tinysys_mbox_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mtk_tinysys_mbox_realize;
    dc->desc = "MediaTek tinysys mailbox";
    device_class_set_legacy_reset(dc, mtk_tinysys_mbox_reset);
    device_class_set_props(dc, mtk_tinysys_mbox_props);
}

static const TypeInfo mtk_tinysys_mbox_info = {
    .name = TYPE_MTK_TINYSYS_MBOX,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkTinysysMboxState),
    .instance_finalize = mtk_tinysys_mbox_finalize,
    .class_init = mtk_tinysys_mbox_class_init,
};

static void mtk_tinysys_mbox_register_types(void)
{
    type_register_static(&mtk_tinysys_mbox_info);
}

type_init(mtk_tinysys_mbox_register_types)
