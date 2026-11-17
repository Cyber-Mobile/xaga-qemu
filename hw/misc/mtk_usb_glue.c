/*
 * MediaTek USB IPPC glue (MTK_USB_IPPC)
 *
 * mtu3's ssusb_check_clocks() polls the IPPC block (device tree "ippc" reg:
 * 0x11203e00 + 0x100) until the per-port PHY clock-stable bits report ready:
 *
 *     value = readl(ibase + 0x10);
 *     if ((value & (check_clk | 0x501)) != (check_clk | 0x501))  -> retry
 *     value = readl(ibase + 0x14);
 *     if (!(value & BIT(0)))                                     -> retry
 *
 * On the emulated SoC there is no PHY clock tree behind that window, the
 * poll times out and mtu3 probe fails with -ETIMEDOUT:
 *
 *     mtu3 11201000.usb0: clks of sts1 are not stable!
 *     mtu3 11201000.usb0: device enable failed -110
 *
 * This model keeps the whole window as a plain read/write register file --
 * identical to the syscon stub that used to serve it, so the clock-gate
 * driver using the same block keeps working -- and reports the two status
 * words as "everything stable".  All other IPPC registers keep their stored
 * values; the 0x100-byte DT window is installed as an overlay (priority 1)
 * on top of the generic syscon window in hw/arm/mt6895.c.
 *
 * MTK_NO_USBGLUE=1 reverts to the plain syscon window.
 * MTK_USBLOG=1 traces accesses here.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "hw/core/cpu.h"

#define TYPE_MTK_USB_IPPC "mtk-usb-ippc"
OBJECT_DECLARE_SIMPLE_TYPE(MtkUsbIppcState, MTK_USB_IPPC)

#define MTK_USB_IPPC_SIZE 0x100

/* Offsets mtu3's ssusb_check_clocks() polls, see mtu3_host.c/mtu3_core.c. */
#define MTK_USB_IPPC_CLK_STS0 0x10
#define MTK_USB_IPPC_CLK_STS1 0x14

struct MtkUsbIppcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[MTK_USB_IPPC_SIZE / 4];
};

static bool mtk_usb_ippc_log(void)
{
    static int cached = -1;

    if (cached < 0) {
        cached = getenv("MTK_USBLOG") != NULL;
    }
    return cached;
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

static uint64_t mtk_usb_ippc_read(void *opaque, hwaddr off, unsigned size)
{
    MtkUsbIppcState *s = MTK_USB_IPPC(opaque);
    uint32_t v;

    off &= ~0x3ULL;
    v = s->regs[(off & (MTK_USB_IPPC_SIZE - 1)) / 4];

    switch (off) {
    case MTK_USB_IPPC_CLK_STS0:
        /*
         * Clock/enable status of the enabled U2/U3 ports.  The driver ORs
         * 0x501 into its enabled-port mask and waits until every bit is
         * set, so all-ones means "stable" for any port combination.
         */
        v = 0xffffffffu;
        break;
    case MTK_USB_IPPC_CLK_STS1:
        /* bit0: the USB clock tree is stable */
        v |= 0x1u;
        break;
    default:
        break;
    }

    mtk_usbdevtrace("mtk-usb-ippc@11203e00", 'R', off, size, v);
    if (mtk_usb_ippc_log()) {
        static unsigned n;

        if (n++ < 256) {
            fprintf(stderr, "MTK-USBIPP R off=%#04x = %#08x\n",
                    (unsigned)off, v);
        }
    }
    return v;
}

static void mtk_usb_ippc_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    MtkUsbIppcState *s = MTK_USB_IPPC(opaque);

    off &= ~0x3ULL;
    s->regs[(off & (MTK_USB_IPPC_SIZE - 1)) / 4] = (uint32_t)val;

    mtk_usbdevtrace("mtk-usb-ippc@11203e00", 'W', off, size, val);
    if (mtk_usb_ippc_log()) {
        static unsigned n;

        if (n++ < 256) {
            fprintf(stderr, "MTK-USBIPP W off=%#04x = %#08x\n",
                    (unsigned)off, (uint32_t)val);
        }
    }
}

static const MemoryRegionOps mtk_usb_ippc_ops = {
    .read = mtk_usb_ippc_read,
    .write = mtk_usb_ippc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void mtk_usb_ippc_realize(DeviceState *dev, Error **errp)
{
    MtkUsbIppcState *s = MTK_USB_IPPC(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &mtk_usb_ippc_ops, s,
                          "mtk-usb-ippc", MTK_USB_IPPC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void mtk_usb_ippc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mtk_usb_ippc_realize;
    dc->desc = "MediaTek USB IPPC clock/port status glue";
}

static const TypeInfo mtk_usb_ippc_info = {
    .name = TYPE_MTK_USB_IPPC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkUsbIppcState),
    .class_init = mtk_usb_ippc_class_init,
};

static void mtk_usb_ippc_register_types(void)
{
    type_register_static(&mtk_usb_ippc_info);
}

type_init(mtk_usb_ippc_register_types);
