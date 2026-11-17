/*
 * MediaTek mtu3 USB device-controller (UDC) minimal model for LK fastboot.
 *
 * LK's fastboot UDC driver (see docs/xaga/architecture.md) talks to
 * the mtu3 IP at 0x11201000 (U3D device regs), 0x11202400 (USB3 MAC CSR),
 * 0x11203400 (USB2 CSR) and 0x11203e00 (IPPC, modelled by mtk-usb-ippc glue).
 * This device renders the subset needed to enumerate EP0 (control) and to let
 * LK's QMU init/start writes succeed (shadow registers), so fastboot's
 * "usb_online" event can be reached.  A host-side Python "virtual USB host"
 * talks to it over a chardev socket with a tiny framed protocol:
 *
 *   host -> dev:  u8 type, u16le len, payload
 *                 0x01 SETUP (8-byte USB setup packet)
 *                 0x10 HOST_CONNECT (bus reset)
 *                 0x11 HOST_DISCONNECT
 *                 0x03 EP0 IN ACK (optional)
 *   dev -> host:  0x81 EP0 IN data, 0x90 LOG (nul-terminated string)
 *
 * Only used when the machine is started with MTK_USBDEV=1; by default the
 * device is not created, so kernel/USB-host scenarios are unaffected.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "qapi/error.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define TYPE_MTK_USBDEV "mtk-usbdev"
OBJECT_DECLARE_SIMPLE_TYPE(MtkUsbDevState, MTK_USBDEV)

#define MTK_USBDEV_SIZE        0x2e00

/* U3D device registers (base 0x11201000) */
#define R_LV1ISR               0x0000
#define R_LV1IER               0x0004
#define R_LV1IESR              0x0008
#define R_LV1IECR              0x000c
#define R_EPISR                0x0080
#define R_EPIER                0x0084
#define R_EPIESR               0x0088
#define R_EPIECR               0x008c
#define R_EP0CSR               0x0100
#define R_RXCOUNT0             0x0108
#define R_FIFO0                0x0300
#define R_QCR0                 0x0400
#define R_QCR1                 0x0404
#define R_QCR2                 0x0408
#define R_QGCSR                0x0410
#define R_QISAR0               0x0700
#define R_QIER0                0x0704
#define R_QIESR0               0x0708
#define R_QIECR0               0x070c
#define R_QISAR1               0x0710
#define R_QIER1                0x0714
#define R_QEMIR                0x0740
#define R_QCR3                 0x040c
#define R_TXQCSR1              0x0510
#define R_TXQSAR1              0x0514
#define R_TXQCPR1              0x0518
#define R_RXQCSR1              0x0610
#define R_RXQSAR1              0x0614
#define R_RXQCPR1              0x0618
#define R_RXQLDPR1             0x061c
#define R_TXQHIAR1             0x0484
#define R_RXQHIAR1             0x04c4
#define QMU_Q_START            0x00000001
#define QMU_Q_RESUME           0x00000002
#define QMU_Q_STOP             0x00000004
#define QMU_Q_ACTIVE           0x00008000
#define QMU_TX_DONE_1          (1u << 1)
#define QMU_RX_DONE_1          (1u << 17)
#define GPD_FLAG_HWO           (1u << 0)
#define QMU_GPD_DEFAULT_STRIDE 16
/* USB3 MAC CSR (base 0x11202400) -> offset 0x1400 */
#define R_USB3_BASE            0x1400
#define R_USB3_CONFIG          0x141c
/* EPCTL CSR (base 0x11201800) -> offset 0x0800 */
#define R_DEVICE_CONF          0x0800
#define R_EP_RST               0x0804
#define R_MISC_CTRL            0x0c84
/* Capability registers (read-only).  FIFO sizes are in units of
 * MTU3_EP_FIFO_UNIT (512 B); EPINFO packs the TX/RX non-EP0 endpoint counts
 * as CAP_TX_EP_NUM = bits[4:0], CAP_RX_EP_NUM = bits[12:8].  LK fastboot and
 * the kernel gadget share one FIFO layout: 64 B fixed for EP0 plus a modeled
 * non-EP0 FIFO split evenly between TX and RX. */
#define R_CAP_EP0FFSZ          0x0c04
#define R_CAP_EPNTXFFSZ        0x0c08
#define R_CAP_EPNRXFFSZ        0x0c0c
#define R_CAP_EPINFO           0x0c10
#define MTK_USBDEV_EP_PAIRS    7        /* non-EP0 IN/OUT endpoint pairs */
#define MTK_USBDEV_EP_FIFO_SZ  0x2000   /* total non-EP0 FIFO, 4 KiB per direction */
#define R_DEV_LINK_INTR_EN     0x0850
#define R_DEV_LINK_INTR        0x0854
/* USB2 CSR (base 0x11203400) -> offset 0x2400 */
#define R_USB2_BASE            0x2400
#define R_POWER_MGMT           0x2404
#define R_COMMON_INTR_EN       0x2418
#define R_COMMON_INTR          0x241c
#define R_LINK_RESET_INFO      0x2424

/* register bits */
#define LV1_EP_INTR            (1u << 0)    /* EPISR/EP aggregate */
#define LV1_QMU_INTR           (1u << 1)
#define LV1_MAC2_INTR          (1u << 4)
#define LV1_LINK_INTR          (1u << 5)
#define EP0_ISR                (1u << 0)    /* EPISR[0]: EP0 TX done  */
#define EP0_SETUP_ISR          (1u << 16)   /* EPISR[16]: setup/RX   */
#define EP0CSR_RXPKTRDY        (1u << 16)
#define EP0CSR_DPHTX           (1u << 20)
#define EP0CSR_SETUPPKTRDY     (1u << 17)
#define EP0CSR_TXPKTRDY        (1u << 18)
#define EP0CSR_DATAEND         (1u << 19)
#define EP0CSR_SENTSTALL       (1u << 22)
#define EP0CSR_SENDSTALL       (1u << 25)
#define PM_SOFT_CONN           (1u << 6)   /* U3D_POWER_MANAGEMENT.SOFT_CONN */
#define COMMON_INTR_RESET      (1u << 2)
/* U3D_DEV_LINK_INTR (EPCTL CSR +0x54) */
#define DEV_LINK_SPEED_CHG     (1u << 0)
/*
 * SSUSB_DEV_SPEED(U3D_DEVICE_CONF) encoding of the three USB 2.0 speeds:
 * 1 = low, 2 = full, 3 = high.  The kernel's enum mtu3_speed uses the same
 * values (MTU3_SPEED_FULL = 1, MTU3_SPEED_HIGH = 3) and calls
 * mtu3_ep0_setup() only from the high/full-speed cases of mtu3_link_isr(),
 * so reporting 2 for high speed left the driver in its UNKNOWN branch
 * (gadget disconnect, EP0 interrupts never enabled).
 */
#define DEVICE_CONF_SPEED_HS   3
/*
 * U3D_MISC_CTRL.DMA_ADDR_36BIT is a hardwired capability of the controller
 * (the mtu3 driver selects its DMA mask from it).  Without it every gadget
 * buffer above 4 GiB fails to map and the adb function cannot queue requests.
 */
#define MISC_CTRL_DMA_ADDR_36BIT (1u << 31)
#define COMMON_INTR_SUSPEND    (1u << 0)
#define COMMON_INTR_RESUME     (1u << 1)

/* chardev framing */
#define FB_HOST_SETUP          0x01
#define FB_HOST_EPOUT          0x02
#define FB_HOST_EP0_ACK        0x03
#define FB_HOST_CONNECT        0x10
#define FB_HOST_DISCONNECT     0x11
#define FB_DEV_EP0_IN          0x81
#define FB_DEV_EP1_IN          0x82
#define FB_DEV_LOG             0x90

struct MtkUsbDevState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    CharFrontend chr_be;
    bool have_chr;
    bool trace;

    uint32_t regs[MTK_USBDEV_SIZE / 4];
    uint32_t lv1_isr;
    uint32_t lv1_ier;
    uint32_t ep_isr;
    uint32_t ep_ier;
    uint32_t common_intr;
    uint32_t common_intr_en;
    uint32_t dev_link_intr;
    uint32_t dev_link_intr_en;

    uint8_t pending_setup[8];
    bool setup_pending;
    bool ep0_in_flight;
    uint8_t fifo_out[64];        /* device -> host (setup + IN payloads) */
    unsigned fifo_out_len;
    unsigned fifo_out_pos;
    uint8_t fifo_in[1024];       /* host <- device (EP0 IN payloads) */
    unsigned fifo_in_len;

    GByteArray *rxraw;           /* unframed chardev input */
    QEMUTimer *pc_timer;
    GByteArray *rxq;
    QEMUTimer *rx_timer;
    uint64_t rx_last_frame_ms;
    uint64_t rx_bytes_done;
    uint64_t rx_gpd_done;
    bool rx_wait_ack;
    uint64_t rx_wait_since_ms;
    uint64_t rx_armed_addr;
    uint64_t rx_armed_ms;
    uint64_t start_ms;
    uint32_t soft_conn_min_ms;
    bool connected;
    bool soft_conn_applied;
    uint32_t gpd_stride;
    uint32_t ring_size;
    uint64_t tx_gpd_last;   /* next TX GPD to send (36-bit address) */
    uint64_t rx_gpd_next;   /* first RX GPD of the ring (36-bit address) */
    bool qmu_tx_on;
    bool qmu_rx_on;
    bool qmu_ready;
};

static const char *rname(uint32_t off)
{
    switch (off) {
    case R_LV1ISR: return "LV1ISR";
    case R_EPISR: return "EPISR";
    case R_EP0CSR: return "EP0CSR";
    case R_RXCOUNT0: return "RXCOUNT0";
    case R_FIFO0: return "FIFO0";
    case R_DEVICE_CONF: return "DEVICE_CONF";
    case R_POWER_MGMT: return "POWER_MGMT";
    case R_COMMON_INTR_EN: return "COMMON_INTR_EN";
    case R_COMMON_INTR: return "COMMON_INTR";
    default: return "REG";
    }
}

static void usbdev_trace(MtkUsbDevState *s, const char *dir, uint32_t off,
                         uint64_t val)
{
    if (!s->trace) {
        return;
    }
    {
        CPUState *cs = qemu_get_cpu(0);
        uint64_t pc = cs ? (uint64_t)CPU_GET_CLASS(cs)->get_pc(cs) : 0;

        fprintf(stderr, "USBDEV %s %s+%#x val=%#" PRIx64 " pc=%#" PRIx64 "\n",
                dir, rname(off), off, val, pc);
    }
}

static uint32_t usbdev_lv1(MtkUsbDevState *s)
{
    uint32_t v = s->lv1_isr;

    /* LK bit layout (from its ISR disassembly at 0x50f31a74):
     * bit0 = EPISR/EP aggregate, bit1 = QMU, bit4 = MAC2/common-USB
     * (reset/suspend/resume), bit5 = DEV_LINK. */
    if ((s->ep_isr & s->ep_ier) != 0) {
        v |= LV1_EP_INTR;
    }
    if ((s->common_intr & s->common_intr_en) != 0) {
        v |= LV1_MAC2_INTR;
    }
    if ((s->regs[R_QISAR0 / 4] & s->regs[R_QIER0 / 4]) != 0 ||
        (s->regs[R_QISAR1 / 4] & s->regs[R_QIER1 / 4]) != 0) {
        v |= LV1_QMU_INTR;
    }
    if ((s->dev_link_intr & s->dev_link_intr_en) != 0) {
        v |= LV1_LINK_INTR;
    }
    return v;
}

static void usbdev_update_irq(MtkUsbDevState *s)
{
    uint32_t lv1 = usbdev_lv1(s);

    if (s->trace && lv1) {
        fprintf(stderr, "USBDEV IRQ lv1=%#x ier=%#x\n", lv1, s->lv1_ier);
    }
    qemu_set_irq(s->irq, (lv1 & s->lv1_ier) != 0);
}

static void usbdev_send(MtkUsbDevState *s, uint8_t type,
                        const uint8_t *data, unsigned len)
{
    uint8_t hdr[3];

    if (!s->have_chr) {
        return;
    }
    hdr[0] = type;
    hdr[1] = len & 0xff;
    hdr[2] = (len >> 8) & 0xff;
    qemu_chr_fe_write_all(&s->chr_be, hdr, 3);
    if (len) {
        qemu_chr_fe_write_all(&s->chr_be, data, len);
    }
}

static void usbdev_log(MtkUsbDevState *s, const char *msg)
{
    usbdev_send(s, FB_DEV_LOG, (const uint8_t *)msg, strlen(msg));
}

/*
 * QMU (endpoint queue) support for EP1.  The GPD ring lives in guest DRAM
 * with a 16-byte stride; the controller DMA engine supports 36-bit addresses
 * (U3D_MISC_CTRL.DMA_ADDR_36BIT), so each GPD carries the upper four address
 * bits in its extension nibbles.
 *
 * Legacy U2 GPD layout ("OG" macros in the kernel driver):
 *   dw0: TX -> { buf_hi@16-19, next_hi@20-23 } | IOC | HWO
 *        RX -> { rx_buf_len@16-31 }
 *   dw1: next_gpd (low 32 bits)
 *   dw2: buffer (low 32 bits)
 *   dw3: TX -> { data_len@0-15 }
 *        RX -> { buf_hi@16-19, next_hi@20-23 }
 *   byte 0: flags (bit0 = HWO, bit7 = IOC)
 * QISAR0 bit17 = RX done / bit1 = TX done for EP1.
 */
#define QMU_GPD_MAX_BUF        65532

static uint64_t usbdev_qmu_start_addr(MtkUsbDevState *s, bool is_in)
{
    uint32_t sar = s->regs[(is_in ? R_TXQSAR1 : R_RXQSAR1) / 4];
    uint32_t hiar = s->regs[(is_in ? R_TXQHIAR1 : R_RXQHIAR1) / 4];

    return sar | ((uint64_t)(hiar & 0xf) << 32);
}

static uint64_t usbdev_qmu_cur_gpd(MtkUsbDevState *s, bool is_in)
{
    uint32_t cpr = s->regs[(is_in ? R_TXQCPR1 : R_RXQCPR1) / 4];
    uint32_t hiar = s->regs[(is_in ? R_TXQHIAR1 : R_RXQHIAR1) / 4];

    return cpr | ((uint64_t)((hiar >> 8) & 0xf) << 32);
}

static void usbdev_qmu_set_cpr(MtkUsbDevState *s, bool is_in, uint64_t cpr)
{
    uint32_t creg = (is_in ? R_TXQCPR1 : R_RXQCPR1) / 4;
    uint32_t hreg = (is_in ? R_TXQHIAR1 : R_RXQHIAR1) / 4;

    s->regs[creg] = (uint32_t)cpr;
    s->regs[hreg] = (s->regs[hreg] & ~(0xfu << 8)) |
                    ((uint32_t)((cpr >> 32) & 0xf) << 8);
}
static bool usbdev_dma_read(MtkUsbDevState *s, uint64_t addr, void *buf,
                            unsigned len)
{
    if (!addr) {
        return false;
    }
    return dma_memory_read(&address_space_memory, addr, buf, len,
                           MEMTXATTRS_UNSPECIFIED) == MEMTX_OK;
}

static bool usbdev_dma_write(MtkUsbDevState *s, uint64_t addr, const void *buf,
                             unsigned len)
{
    if (!addr) {
        return false;
    }
    return dma_memory_write(&address_space_memory, addr, buf, len,
                            MEMTXATTRS_UNSPECIFIED) == MEMTX_OK;
}

static void usbdev_gpd_parse(const uint8_t *g, bool is_rx, uint32_t *flag,
                             uint64_t *baddr, uint64_t *next, uint16_t *len)
{
    uint32_t lo_next = (uint32_t)g[4] | ((uint32_t)g[5] << 8) |
                       ((uint32_t)g[6] << 16) | ((uint32_t)g[7] << 24);
    uint32_t lo_buf = (uint32_t)g[8] | ((uint32_t)g[9] << 8) |
                      ((uint32_t)g[10] << 16) | ((uint32_t)g[11] << 24);
    unsigned hi = is_rx ? g[14] : g[2];

    *flag = g[0];
    *len = (uint16_t)(g[12] | (g[13] << 8));
    *baddr = lo_buf | ((uint64_t)(hi & 0xf) << 32);
    *next = lo_next | ((uint64_t)((hi >> 4) & 0xf) << 32);
}

/*
 * EP1 RX (host -> LK fastboot data) path.
 *
 * Host payloads are queued in a model FIFO and drained one GPD at a time only
 * when LK has (re)armed the next GPD (HWO=1).  This mirrors the hardware:
 * one GPD completes per interrupt, RXQCPR advances to the next GPD, and a new
 * completion only happens after LK has processed the previous request and
 * armed the next one.  Large fastboot downloads therefore never write into an
 * unarmed GPD (which made LK report "no correct RX req is found" and panic).
 */
static void usbdev_rx_arm_timer(MtkUsbDevState *s, int ms)
{
    if (!s->rx_timer) {
        return;
    }
    timer_mod(s->rx_timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + ms);
}

static void usbdev_rx_try_deliver(MtkUsbDevState *s, bool flush_short)
{
    uint8_t gpd[16];
    uint64_t base = usbdev_qmu_start_addr(s, false);
    uint32_t stride = s->gpd_stride ? s->gpd_stride : 16;
    uint64_t ring = s->ring_size ? s->ring_size : 0x40;
    uint64_t gaddr = usbdev_qmu_cur_gpd(s, false);
    uint32_t flag;
    uint64_t baddr, next;
    uint16_t done_len, buflen;
    unsigned n, avail;

    if (!s->qmu_rx_on || !s->rxq || !s->rxq->len) {
        return;
    }
    if (!gaddr) {
        gaddr = base;
    }
    if (!gaddr) {
        return;
    }
    /* Hardware semantics: RXQCPR is the physical address of the GPD being
     * filled.  It is only safe to complete it when LK has armed it (HWO=1);
     * LK arms the next GPD only after processing this completion. */
    if (!usbdev_dma_read(s, gaddr, gpd, sizeof(gpd))) {
        return;
    }
    usbdev_gpd_parse(gpd, true, &flag, &baddr, &next, &done_len);
    if (!(flag & GPD_FLAG_HWO)) {
        if (s->trace && s->rx_gpd_done < 64) {
            static int nw;
            if (nw++ < 16) {
                fprintf(stderr, "USBDEV RXWAIT cpr=%#" PRIx64
                        " flag=%#x q=%u\n", gaddr, flag, s->rxq->len);
            }
        }
        usbdev_rx_arm_timer(s, 3);
        return;
    }
    buflen = gpd[2] | (gpd[3] << 8);
    if (buflen < 64) {
        buflen = 64;
    }
    if (!baddr) {
        usbdev_rx_arm_timer(s, 3);
        return;
    }
    avail = s->rxq->len;
    if (avail < buflen && !flush_short) {
        usbdev_rx_arm_timer(s, 4);
        return;
    }
    n = avail < buflen ? avail : buflen;
    /* LK sets GPD HWO=1 a few instructions before it stores req->gpd; if we
     * complete the GPD immediately the interrupt can be taken inside that
     * window and LK reports "no correct RX req".  Wait one timer tick after
     * first seeing HWO so the prepare sequence has finished. */
    {
        uint64_t now = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);

        if (s->rx_armed_addr != gaddr || s->rx_armed_ms == 0 ||
            now - s->rx_armed_ms < 1) {
            s->rx_armed_addr = gaddr;
            s->rx_armed_ms = now;
            usbdev_rx_arm_timer(s, 1);
            return;
        }
    }
    if (s->trace && s->rx_gpd_done < 128) {
        unsigned i;
        int mask = 0;

        fprintf(stderr, "USBDEV RXDONE cpr=%#" PRIx64
                " flag=%#x len=%u buf=%u baddr=%#" PRIx64 " src=%s\n",
                gaddr, flag, n, buflen, baddr,
                (avail < buflen) ? "short" : "full");
        for (i = 0; i < 8 && base; i++) {
            uint8_t g2[16];

            if (!usbdev_dma_read(s, base + i * stride, g2, sizeof(g2))) {
                break;
            }
            mask |= (g2[0] & 1) << i;
        }
        fprintf(stderr, "USBDEV RXARM base=%#" PRIx64 " mask=%#x cpr=%#"
                PRIx64 "\n", base, mask, gaddr);
    }
    usbdev_dma_write(s, baddr, s->rxq->data, n);
    g_byte_array_remove_range(s->rxq, 0, n);
    /* Complete exactly this GPD: actual length at +0xc, only HWO cleared. */
    gpd[12] = n & 0xff;
    gpd[13] = (n >> 8) & 0xff;
    gpd[0] &= ~GPD_FLAG_HWO;
    usbdev_dma_write(s, gaddr, gpd, sizeof(gpd));
    {
        /* The GPD carries the DMA address of the next descriptor and the
         * hardware walks that link, so follow it; the kernel uses 144-entry
         * rings while LK used a 4-entry one.  Fall back to a linear walk with
         * the modelled ring size only when a descriptor has no next link. */
        uint64_t cpr = next;

        if (!cpr) {
            cpr = gaddr + stride;
            if (base && (cpr - base) >= ring) {
                cpr = base;
            }
        }
        usbdev_qmu_set_cpr(s, false, cpr);
        if (s->trace && s->rx_gpd_done < 128) {
            fprintf(stderr, "USBDEV RXCPR %#" PRIx64 " -> %#" PRIx64 "\n",
                    gaddr, cpr);
        }
    }
    s->regs[R_QISAR0 / 4] |= QMU_RX_DONE_1;
    s->ep_isr |= (1u << 1);          /* EPISR bit1 = EP1 */
    s->rx_bytes_done += n;
    s->rx_gpd_done++;
    s->rx_armed_addr = 0;
    s->rx_armed_ms = 0;
    if (s->trace && (s->rx_gpd_done % 1024) == 1) {
        fprintf(stderr, "USBDEV RX gpd=%llu gaddr=%#" PRIx64
                " n=%u buf=%u total=%llu q=%u\n",
                (unsigned long long)s->rx_gpd_done, gaddr, n, buflen,
                (unsigned long long)s->rx_bytes_done, s->rxq->len);
    }
    usbdev_update_irq(s);
    /* Give LK a chance to process the completion and arm the next GPD. */
    if (s->rxq->len) {
        usbdev_rx_arm_timer(s, 1);
    }
}

static void usbdev_ep1_out(MtkUsbDevState *s, const uint8_t *data, unsigned len)
{
    if (!len) {
        return;
    }
    if (!s->rxq) {
        s->rxq = g_byte_array_new();
    }
    g_byte_array_append(s->rxq, data, len);
    s->rx_last_frame_ms = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
    usbdev_rx_try_deliver(s, false);
}

static void usbdev_rx_timer(void *opaque)
{
    MtkUsbDevState *s = opaque;
    uint64_t now;

    if (!s->rxq || !s->rxq->len) {
        return;
    }
    now = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
    usbdev_rx_try_deliver(s, now - s->rx_last_frame_ms >= 20);
}

static void usbdev_tx_kick(MtkUsbDevState *s, uint32_t cpr)
{
    uint64_t base = usbdev_qmu_start_addr(s, true);
    uint32_t stride = s->gpd_stride ? s->gpd_stride : 16;
    uint64_t gaddr = s->tx_gpd_last ? s->tx_gpd_last : base;
    uint8_t *buf;
    int guard = 0;

    if (!s->qmu_tx_on || !gaddr) {
        return;
    }
    buf = g_malloc(QMU_GPD_MAX_BUF + 1);
    while (gaddr && guard++ < 64) {
        uint8_t gpd[16];
        uint32_t flag;
        uint64_t baddr, next;
        uint16_t blen;

        if (!usbdev_dma_read(s, gaddr, gpd, sizeof(gpd))) {
            break;
        }
        usbdev_gpd_parse(gpd, false, &flag, &baddr, &next, &blen);
        if (!(flag & GPD_FLAG_HWO)) {
            break;
        }
        if (blen > QMU_GPD_MAX_BUF) {
            blen = QMU_GPD_MAX_BUF;
        }
        if (blen && baddr && usbdev_dma_read(s, baddr, buf, blen)) {
            usbdev_send(s, FB_DEV_EP1_IN, buf, blen);
        } else if (!blen) {
            /* zero-length packet: hand an empty frame to the host bridge */
            usbdev_send(s, FB_DEV_EP1_IN, buf, 0);
        }
        gpd[0] &= ~GPD_FLAG_HWO;
        usbdev_dma_write(s, gaddr, gpd, sizeof(gpd));
        if (s->trace) {
            fprintf(stderr, "USBDEV EP1 IN len=%u gpd=%#" PRIx64
                    " baddr=%#" PRIx64 " next=%#" PRIx64 "\n",
                    blen, gaddr, baddr, next);
        }
        /* Follow the GPD next-descriptor link like the hardware does; fall
         * back to a linear walk with a ring wrap only if it has no link. */
        if (next) {
            gaddr = next;
        } else {
            uint64_t ring = s->ring_size ? s->ring_size : 0x400;

            if (base && (gaddr - base + stride) >= ring) {
                gaddr = base;
            } else {
                gaddr += stride;
            }
        }
    }
    g_free(buf);
    s->tx_gpd_last = gaddr;
    usbdev_qmu_set_cpr(s, true, gaddr);
    s->regs[R_QISAR0 / 4] |= QMU_TX_DONE_1;
    s->ep_isr |= (1u << 1);          /* EPISR bit1 = EP1 */
    usbdev_update_irq(s);
}

/* Complete a control-IN packet: send the current fifo_in payload to host. */
static void usbdev_deliver_setup(MtkUsbDevState *s)
{
    memcpy(s->fifo_out, s->pending_setup, 8);
    s->fifo_out_len = 8;
    s->fifo_out_pos = 0;
    s->regs[R_RXCOUNT0 / 4] = 8;
    s->regs[R_EP0CSR / 4] |= EP0CSR_SETUPPKTRDY | EP0CSR_RXPKTRDY;
    /*
     * SETUPENDISR is what the Linux mtu3 driver uses to return its EP0 state
     * machine to "waiting for SETUP"; it enables both this bit and EP0ISR.
     */
    s->ep_isr |= EP0_ISR | EP0_SETUP_ISR;
    usbdev_update_irq(s);
}

static void usbdev_ep0_in_flush(MtkUsbDevState *s)
{
    usbdev_trace(s, "TX", R_FIFO0, s->fifo_in_len);
    usbdev_send(s, FB_DEV_EP0_IN, s->fifo_in, s->fifo_in_len);
    s->fifo_in_len = 0;
    s->ep0_in_flight = true;
    s->ep_isr |= EP0_ISR;
}

static void usbdev_host_setup(MtkUsbDevState *s, const uint8_t *setup,
                              unsigned len)
{
    if (len != 8) {
        return;
    }
    if (s->trace) {
        fprintf(stderr, "USBDEV SETUP %02x %02x %02x %02x %02x %02x %02x %02x%s\n",
                setup[0], setup[1], setup[2], setup[3], setup[4], setup[5],
                setup[6], setup[7],
                (s->ep0_in_flight || s->setup_pending) ? " (queued)" : "");
    }
    /* Do not clobber a control transfer that is still finishing: LK may send
     * the DATAEND/status write for the previous request after the next SETUP
     * arrived, which would otherwise clear the new SETUP flags (the ~50%
     * SET_CONFIG stall).  Queue it and deliver when the IN transfer ends. */
    if (s->ep0_in_flight || s->setup_pending) {
        memcpy(s->pending_setup, setup, 8);
        s->setup_pending = true;
        return;
    }
    memcpy(s->pending_setup, setup, 8);
    usbdev_deliver_setup(s);
}

static void usbdev_host_connect(MtkUsbDevState *s)
{
    s->connected = true;
    s->common_intr |= COMMON_INTR_RESET;
    /*
     * A connect is also a link-speed change for the device controller: the
     * Linux mtu3 driver only runs mtu3_ep0_setup() (which programs EP0
     * maxpacket and enables the EP0/SETUPEND interrupts) from its device-link
     * interrupt handler, so the speed-change interrupt has to be raised here
     * just like on real hardware.
     */
    s->regs[R_DEVICE_CONF / 4] =
        (s->regs[R_DEVICE_CONF / 4] & ~0x7u) | DEVICE_CONF_SPEED_HS;
    s->dev_link_intr |= DEV_LINK_SPEED_CHG;
    s->fifo_in_len = 0;
    s->ep0_in_flight = false;
    s->setup_pending = false;
    s->regs[R_EP0CSR / 4] = 0;
    s->ep_isr = 0;
    usbdev_log(s, "CONNECTED");
    usbdev_update_irq(s);
}

static void usbdev_frame(MtkUsbDevState *s, uint8_t type,
                         const uint8_t *payload, unsigned len)
{
    if (!s->soft_conn_applied && (type == FB_HOST_SETUP ||
                                    type == FB_HOST_EPOUT)) {
        /* LK has not raised the U2 pullup yet: its UDC driver is not ready,
         * so buffering control traffic would only confuse it. */
        usbdev_log(s, "NOT_READY");
        return;
    }
    switch (type) {
    case FB_HOST_SETUP:
        usbdev_host_setup(s, payload, len);
        break;
    case FB_HOST_CONNECT:
        usbdev_host_connect(s);
        break;
    case FB_HOST_DISCONNECT:
        s->connected = false;
        s->common_intr |= COMMON_INTR_RESET;
        usbdev_update_irq(s);
        break;
    case FB_HOST_EP0_ACK:
        break;
    case FB_HOST_EPOUT:
        usbdev_ep1_out(s, payload, len);
        break;
    default:
        if (s->trace) {
            fprintf(stderr, "USBDEV unknown frame %#x len=%u\n", type, len);
        }
        break;
    }
}

static int usbdev_chr_can_read(void *opaque)
{
    MtkUsbDevState *s = opaque;

    /* Backpressure: stop reading the socket while the RX FIFO is full. */
    if (s->rxq && s->rxq->len > (1u << 20)) {
        return 0;
    }
    return 4096;
}

static void usbdev_chr_read(void *opaque, const uint8_t *buf, int size)
{
    MtkUsbDevState *s = opaque;

    if (!s->rxraw) {
        s->rxraw = g_byte_array_new();
    }
    g_byte_array_append(s->rxraw, buf, size);
    while (s->rxraw->len >= 3) {
        uint8_t type = s->rxraw->data[0];
        unsigned len = s->rxraw->data[1] | (s->rxraw->data[2] << 8);
        unsigned need = 3 + len;

        if (len > (8u << 20)) {          /* bogus frame: resync */
            g_byte_array_set_size(s->rxraw, 0);
            return;
        }
        if (s->rxraw->len < need) {
            return;
        }
        usbdev_frame(s, type, s->rxraw->data + 3, len);
        g_byte_array_remove_range(s->rxraw, 0, need);
    }
}

static void usbdev_chr_event(void *opaque, QEMUChrEvent event)
{
    MtkUsbDevState *s = opaque;

    if (event == CHR_EVENT_OPENED) {
        if (s->rxraw) {
            g_byte_array_set_size(s->rxraw, 0);
        }
        usbdev_log(s, "READY");
    } else if (event == CHR_EVENT_CLOSED) {
        if (s->rxraw) {
            g_byte_array_set_size(s->rxraw, 0);
        }
    }
}

static uint64_t usbdev_read(void *opaque, hwaddr off, unsigned size)
{
    MtkUsbDevState *s = opaque;
    uint32_t idx = off / 4;
    uint64_t val = 0;

    if (idx >= ARRAY_SIZE(s->regs)) {
        return 0;
    }
    if (off >= R_FIFO0 && off < R_FIFO0 + 0x10) {
        unsigned i;

        for (i = 0; i < 4; i++) {
            unsigned p = s->fifo_out_pos + i;

            if (p < s->fifo_out_len) {
                val |= (uint64_t)s->fifo_out[p] << (8 * i);
            }
        }
        s->fifo_out_pos += 4;
        return val;
    }
    switch (off) {
    case R_LV1ISR:
        val = usbdev_lv1(s);
        break;
    case R_LV1IER:
        val = s->lv1_ier;
        break;
    case R_EPISR:
        val = s->ep_isr;
        break;
    case R_EPIER:
        val = s->ep_ier;
        break;
    case R_COMMON_INTR:
        val = s->common_intr;
        break;
    case R_COMMON_INTR_EN:
        val = s->common_intr_en;
        break;
    case R_DEV_LINK_INTR:
        val = s->dev_link_intr;
        break;
    case R_DEV_LINK_INTR_EN:
        val = s->dev_link_intr_en;
        break;
    case R_CAP_EP0FFSZ:
        val = 64;                       /* EP0 FIFO is fixed at 64 B */
        break;
    case R_CAP_EPNTXFFSZ:
    case R_CAP_EPNRXFFSZ:
        val = MTK_USBDEV_EP_FIFO_SZ;
        break;
    case R_CAP_EPINFO:
        val = ((uint32_t)MTK_USBDEV_EP_PAIRS << 8) | MTK_USBDEV_EP_PAIRS;
        break;
    case R_MISC_CTRL:
        /* DMA_ADDR_36BIT is a hardwired capability of the controller. */
        val = s->regs[idx] | MISC_CTRL_DMA_ADDR_36BIT;
        break;
    default:
        val = s->regs[idx];
        break;
    }
    if (size < 4) {
        val &= (1ULL << (size * 8)) - 1;
    }
    usbdev_trace(s, "R", off, val);
    return val;
}

static void usbdev_write(void *opaque, hwaddr off, uint64_t value,
                         unsigned size)
{
    MtkUsbDevState *s = opaque;
    uint32_t idx = off / 4;
    uint32_t v = (uint32_t)value;

    if (idx >= ARRAY_SIZE(s->regs)) {
        return;
    }
    usbdev_trace(s, "W", off, value);

    if (off >= R_FIFO0 && off < R_FIFO0 + 0x10) {
        unsigned i;

        for (i = 0; i < size && s->fifo_in_len < sizeof(s->fifo_in); i++) {
            s->fifo_in[s->fifo_in_len++] = (value >> (8 * i)) & 0xff;
        }
        return;
    }
    switch (off) {
    case R_LV1ISR:
        s->lv1_isr &= ~v;
        usbdev_update_irq(s);
        return;
    case R_LV1IER:
        s->lv1_ier = v;
        usbdev_update_irq(s);
        return;
    case R_LV1IESR:
        s->lv1_ier |= v;
        usbdev_update_irq(s);
        return;
    case R_LV1IECR:
        s->lv1_ier &= ~v;
        usbdev_update_irq(s);
        return;
    case R_EPISR:
        {
            uint32_t clr = v;

            /* Race guard: LK reads EPISR then writes the same value back to
             * acknowledge it.  If a new SETUP/data packet arrived in between,
             * that bit must stay pending or the interrupt is lost and LK
             * stays idle (the ~50% SET_CONFIG stall). */
            if ((clr & EP0_ISR) &&
                (s->regs[R_EP0CSR / 4] &
                 (EP0CSR_SETUPPKTRDY | EP0CSR_RXPKTRDY)) &&
                s->fifo_out_pos == 0) {
                clr &= ~EP0_ISR;
            }
            if ((clr & (1u << 1)) && s->regs[R_QISAR0 / 4] != 0) {
                clr &= ~(1u << 1);
            }
            s->ep_isr &= ~clr;
            usbdev_update_irq(s);
        }
        return;
    case R_EPIER:
        s->ep_ier = v;
        usbdev_update_irq(s);
        return;
    case R_EPIESR:
        s->ep_ier |= v;
        usbdev_update_irq(s);
        return;
    case R_EPIECR:
        s->ep_ier &= ~v;
        usbdev_update_irq(s);
        return;
    case R_EP0CSR:
        {
            uint32_t csr = s->regs[idx];

            /* W1C status bits */
            csr &= ~(v & (EP0CSR_RXPKTRDY | EP0CSR_SETUPPKTRDY |
                          EP0CSR_SENTSTALL));
            csr = (csr & ~0x3ffu) | (v & 0x3ffu);   /* MAXPKTSZ0 */
            csr |= (v & EP0CSR_SENDSTALL);
            if (v & EP0CSR_SENDSTALL) {
                csr |= EP0CSR_SENTSTALL;
            }
            if (v & EP0CSR_TXPKTRDY) {
                usbdev_ep0_in_flush(s);
                csr &= ~EP0CSR_TXPKTRDY;
            }
            if (v & EP0CSR_DATAEND) {
                csr &= ~(EP0CSR_RXPKTRDY | EP0CSR_SETUPPKTRDY);
                /* LK completes on EP0ISR; the kernel gadget driver also needs
                 * SETUPENDISR to move its ep0 state machine back to SETUP. */
                s->ep_isr |= EP0_ISR | EP0_SETUP_ISR;
                s->ep0_in_flight = false;
                if (s->setup_pending) {
                    s->setup_pending = false;
                    usbdev_deliver_setup(s);
                }
            }
            /* DPHTX is the status-phase bit and is cleared by hardware once
             * the IN packet has been sent; never keep it sticky. */
            csr &= ~EP0CSR_DPHTX;
            s->regs[idx] = csr;
            usbdev_update_irq(s);
        }
        return;
    case R_RXCOUNT0:
        s->regs[idx] = v;
        return;
    case R_QISAR0:
    case R_QISAR1:
    case R_QEMIR:
    case 0x0744:            /* QEMIR companion */
        s->regs[idx] &= ~v;
        usbdev_update_irq(s);
        return;
    case R_QIER0:
        s->regs[idx] = v;
        return;
    case R_QIESR0:
        s->regs[R_QIER0 / 4] |= v;
        return;
    case R_QIECR0:
        s->regs[R_QIER0 / 4] &= ~v;
        return;
    case R_POWER_MGMT:
        s->regs[idx] = v;
        if ((v & PM_SOFT_CONN) && !s->soft_conn_applied) {
            if (s->soft_conn_min_ms &&
                qemu_clock_get_ms(QEMU_CLOCK_REALTIME) - s->start_ms <
                    s->soft_conn_min_ms) {
                return;
            }
            s->soft_conn_applied = true;
            usbdev_log(s, "SOFT_CONN");
        } else if (!(v & PM_SOFT_CONN) && s->soft_conn_applied) {
            s->soft_conn_applied = false;
            usbdev_log(s, "SOFT_DISCONNECT");
        } else if ((v & PM_SOFT_CONN) && s->soft_conn_applied) {
            /* LK re-inits the UDC (second pullup): tell the host again so it
             * can restart enumeration after the final init pass. */
            static int n;
            if (n++ < 4) {
                usbdev_log(s, "SOFT_CONN_AGAIN");
            }
        }
        return;
    case R_COMMON_INTR:
        s->common_intr &= ~v;
        usbdev_update_irq(s);
        return;
    case R_COMMON_INTR_EN:
        s->common_intr_en = v;
        usbdev_update_irq(s);
        return;
    case R_DEV_LINK_INTR:
        s->dev_link_intr &= ~v;
        return;
    case R_DEV_LINK_INTR_EN:
        s->dev_link_intr_en = v;
        return;
    case R_DEVICE_CONF:
        s->regs[idx] = v & 0x7f;
        return;
    case R_TXQSAR1:
        s->regs[idx] = v;
        return;
    case R_TXQCSR1:
        /* Report the queue as ACTIVE so the driver's start_qmu() poll
         * succeeds. */
        s->regs[idx] = (v | QMU_Q_ACTIVE) & ~QMU_Q_STOP;
        if (v & QMU_Q_START) {
            s->tx_gpd_last = usbdev_qmu_start_addr(s, true);
            s->qmu_tx_on = true;
            usbdev_tx_kick(s, v);
        } else if (v & QMU_Q_RESUME) {
            if (!s->tx_gpd_last) {
                s->tx_gpd_last = usbdev_qmu_start_addr(s, true);
            }
            s->qmu_tx_on = true;
            /* The driver re-arms the queue (RESUME) after it has processed
             * the previous completions and queued new GPDs. */
            usbdev_tx_kick(s, v);
        }
        if (v & QMU_Q_STOP) {
            s->qmu_tx_on = false;
        }
        return;
    case R_TXQCPR1:
        s->regs[idx] = v;
        return;
    case R_RXQSAR1:
        s->regs[idx] = v;
        return;
    case R_RXQCSR1:
        s->regs[idx] = (v | QMU_Q_ACTIVE) & ~QMU_Q_STOP;
        if (v & QMU_Q_START) {
            /* Hardware restarts its current-GPD pointer at the ring base. */
            s->rx_gpd_next = usbdev_qmu_start_addr(s, false);
            usbdev_qmu_set_cpr(s, false, s->rx_gpd_next);
        }
        if (v & (QMU_Q_START | QMU_Q_RESUME)) {
            s->qmu_rx_on = true;
            if (!s->qmu_ready) {
                s->qmu_ready = true;
                usbdev_log(s, "QMU_READY");
            }
            /* The driver writes RESUME/START after it has processed the
             * previous RX request and re-armed GPDs; only then is it safe to
             * complete the next GPD. */
            if (s->rxq && s->rxq->len) {
                usbdev_rx_arm_timer(s, 1);
            }
        }
        if (v & QMU_Q_STOP) {
            s->qmu_rx_on = false;
        }
        return;
    case R_RXQCPR1:
    case R_RXQLDPR1:
        s->regs[idx] = v;
        return;
    case R_EP_RST:
    case 0x1804:
        s->regs[idx] = 0;   /* self-clearing */
        return;
    default:
        s->regs[idx] = v;
        return;
    }
}

static void usbdev_pc_sample(void *opaque)
{
    MtkUsbDevState *s = opaque;
    CPUState *cs = qemu_get_cpu(0);

    if (s->trace && cs) {
        fprintf(stderr, "USBDEV PC %#" PRIx64 "\n",
                (uint64_t)CPU_GET_CLASS(cs)->get_pc(cs));
    }
    if (s->pc_timer) {
        timer_mod(s->pc_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 2000);
    }
}

static const MemoryRegionOps usbdev_ops = {
    .read = usbdev_read,
    .write = usbdev_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void usbdev_realize(DeviceState *dev, Error **errp)
{
    MtkUsbDevState *s = MTK_USBDEV(dev);

    s->regs[R_QIER0 / 4] = 0xffffffffu;
    s->regs[R_QIER1 / 4] = 0xffffffffu;
    s->trace = getenv("MTK_USBDEV_TRACE") != NULL;
    {
        const char *gs = getenv("MTK_USBDEV_GPD_STRIDE");

        s->gpd_stride = (gs && *gs) ? (uint32_t)strtoul(gs, NULL, 0) :
                                                     QMU_GPD_DEFAULT_STRIDE;
    }
    {
        const char *rs = getenv("MTK_USBDEV_GPD_RING");

        s->ring_size = (rs && *rs) ? (uint32_t)strtoul(rs, NULL, 0) : 0x40;
    }

    memory_region_init_io(&s->iomem, OBJECT(dev), &usbdev_ops, s,
                          "mtk-usbdev", MTK_USBDEV_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    if (s->trace) {
        s->pc_timer = timer_new_ms(QEMU_CLOCK_REALTIME, usbdev_pc_sample, s);
        timer_mod(s->pc_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 2000);
    }
    s->rxq = g_byte_array_new();
    s->rxraw = g_byte_array_new();
    s->start_ms = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
    {
        const char *m = getenv("MTK_USBDEV_SOFT_CONN_MIN_MS");

        s->soft_conn_min_ms = (m && *m) ? (uint32_t)strtoul(m, NULL, 0) : 0;
    }
    s->rx_timer = timer_new_ms(QEMU_CLOCK_REALTIME, usbdev_rx_timer, s);
    s->have_chr = qemu_chr_fe_backend_connected(&s->chr_be);
    if (s->have_chr) {
        qemu_chr_fe_set_handlers(&s->chr_be, usbdev_chr_can_read,
                                 usbdev_chr_read, usbdev_chr_event, NULL, s,
                                 NULL, true);
    }
}

static const Property usbdev_props[] = {
    DEFINE_PROP_CHR("chardev", MtkUsbDevState, chr_be),
};

static void usbdev_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = usbdev_realize;
    dc->desc = "MediaTek mtu3 UDC (LK fastboot) minimal model";
    device_class_set_props(dc, usbdev_props);
}

static const TypeInfo usbdev_info = {
    .name = TYPE_MTK_USBDEV,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkUsbDevState),
    .class_init = usbdev_class_init,
};

static void usbdev_register_types(void)
{
    type_register_static(&usbdev_info);
}

type_init(usbdev_register_types);
