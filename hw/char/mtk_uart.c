/*
 * MediaTek MT6577-compatible UART
 *
 * The MTK UART is a 16550 whose registers are laid out 4 bytes apart. Linux
 * drives it through 8250_mtk.c with:
 *
 *     uart.port.type    = PORT_16550;
 *     uart.port.flags   = UPF_BOOT_AUTOCONF | UPF_FIXED_PORT;
 *     uart.port.iotype  = UPIO_MEM32;
 *     uart.port.regshift = 2;
 *
 * so every register index N used by the driver lives at byte offset N * 4.
 * Because UPF_FIXED_PORT is set, 8250 never probes the part type, and the
 * baud clock comes from the "baud" clock in the device tree, not from any
 * register we expose. That means the model only has to be truthful about the
 * data path: LSR/IIR/IER/FCR/DLL/DLM and the MTK extension registers that
 * 8250_mtk.c reads back (HIGHS, FEATURE_SEL/EFR, DEBUG0).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "system/address-spaces.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "trace.h"
#include "chardev/char-fe.h"
#include "hw/char/mtk_uart.h"
#include "hw/core/irq.h"
#include "hw/core/cpu.h"
#include "target/arm/cpu.h"
#include "hw/core/qdev-properties-system.h"

/* Byte offsets: register index * 4 (regshift = 2). */
#define MTK_RBR      0x00  /* RBR / THR, DLL when DLAB = 1 */
#define MTK_IER      0x04  /* IER,       DLM when DLAB = 1 */
#define MTK_IIR      0x08  /* IIR (r) / FCR (w) */
#define MTK_LCR      0x0c
#define MTK_MCR      0x10
#define MTK_LSR      0x14
#define MTK_MSR      0x18
#define MTK_SCR      0x1c

/* MTK extension registers, as named by drivers/tty/serial/8250/8250_mtk.c. */
#define MTK_HIGHS        0x24  /* index 0x09 */
#define MTK_SAMPLE_COUNT 0x28  /* index 0x0a */
#define MTK_SAMPLE_POINT 0x2c  /* index 0x0b */
#define MTK_RATE_FIX     0x34  /* index 0x0d */
#define MTK_ESCAPE_DAT   0x40  /* index 0x10 */
#define MTK_ESCAPE_EN    0x44  /* index 0x11 */
#define MTK_DMA_EN       0x4c  /* index 0x13 */
#define MTK_RXTRI_AD     0x50  /* index 0x14 */
#define MTK_FRACDIV_L    0x54  /* index 0x15 */
#define MTK_FRACDIV_M    0x58  /* index 0x16 */
#define MTK_DEBUG0       0x60  /* index 0x18 */
#define MTK_DLL2         0x90  /* index 0x24 */
#define MTK_DLH2         0x94  /* index 0x25 */
#define MTK_EFR          0x98  /* index 0x26 */
#define MTK_FEATURE_SEL  0x9c  /* index 0x27 */

/* Line status register bits. */
#define MTK_LSR_DR    0x01
#define MTK_LSR_OE    0x02
#define MTK_LSR_PE    0x04
#define MTK_LSR_FE    0x08
#define MTK_LSR_BI    0x10
#define MTK_LSR_THRE  0x20
#define MTK_LSR_TEMT  0x40
#define MTK_LSR_FIFO  0x80

/* Interrupt enable register bits. */
#define MTK_IER_RDI   0x01
#define MTK_IER_THRI  0x02
#define MTK_IER_RLSI  0x04
#define MTK_IER_MSI   0x08

/* Interrupt identification register values. */
#define MTK_IIR_NO_INT 0x01
#define MTK_IIR_THRI   0x02
#define MTK_IIR_RDI    0x04
#define MTK_IIR_RLSI   0x06
#define MTK_IIR_FIFO   0xc0

#define MTK_FCR_ENABLE_FIFO 0x01
#define MTK_FCR_CLEAR_RCVR  0x02
#define MTK_FCR_CLEAR_XMIT  0x04

/* Line control register bits. */
#define MTK_LCR_DLAB 0x80

static void mtk_uart_reset_regs(MtkUartState *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    s->rx_head = 0;
    s->rx_count = 0;
    s->break_pending = false;
    s->lcr = 0;
    s->fcr = 0;
    s->ier = 0;
    s->mcr = 0;
    s->thre_ip = false;
}

static void mtk_uart_update_irq(MtkUartState *s)
{
    bool level = false;

    /*
     * Priority follows the 16550: receiver data available outranks a
     * transmitter holding register empty condition. Asserting THRE only
     * while ier_thre_pending is set keeps the line edge-like, which is what
     * the 8250 core expects; it acknowledges by reading IIR.
     */
    if ((s->ier & MTK_IER_RLSI) && s->break_pending) {
        /* Receiver line status outranks everything, as in the 16550. */
        level = true;
    } else if ((s->ier & MTK_IER_RDI) && s->rx_count) {
        level = true;
    } else if ((s->ier & MTK_IER_THRI) && s->thre_ip) {
        level = true;
    }

    qemu_set_irq(s->irq, level);
}

static uint8_t mtk_uart_iir(MtkUartState *s)
{
    uint8_t iir = MTK_IIR_NO_INT;

    if ((s->ier & MTK_IER_RLSI) && s->break_pending) {
        iir = MTK_IIR_RLSI;
    } else if ((s->ier & MTK_IER_RDI) && s->rx_count) {
        iir = MTK_IIR_RDI;
    } else if ((s->ier & MTK_IER_THRI) && s->thre_ip) {
        iir = MTK_IIR_THRI;
    }

    if (s->fcr & MTK_FCR_ENABLE_FIFO) {
        iir |= MTK_IIR_FIFO;
    }
    return iir;
}

static uint8_t mtk_uart_lsr(MtkUartState *s)
{
    uint8_t lsr = MTK_LSR_THRE | MTK_LSR_TEMT;

    if (s->break_pending) {
        lsr |= MTK_LSR_BI | MTK_LSR_DR;
    }

    if (s->rx_count) {
        lsr |= MTK_LSR_DR;
    }
    if (s->fcr & MTK_FCR_ENABLE_FIFO) {
        lsr |= MTK_LSR_FIFO;
    }
    return lsr;
}

static void mtk_uart_rx_push(MtkUartState *s, uint8_t ch)
{
    if (s->rx_count >= MTK_UART_RX_FIFO) {
        /* Overrun: drop, as the hardware would without flow control. */
        return;
    }
    s->rx_fifo[(s->rx_head + s->rx_count) % MTK_UART_RX_FIFO] = ch;
    s->rx_count++;
}

static int mtk_uart_can_receive(void *opaque)
{
    MtkUartState *s = opaque;

    return MTK_UART_RX_FIFO - s->rx_count;
}

static void mtk_uart_event(void *opaque, QEMUChrEvent event)
{
    MtkUartState *s = opaque;

    if (event == CHR_EVENT_BREAK) {
        trace_mtk_uart_break();
        s->break_pending = true;
        mtk_uart_update_irq(s);
    }
}

static void mtk_uart_receive(void *opaque, const uint8_t *buf, int size)
{
    MtkUartState *s = opaque;

    for (int i = 0; i < size; i++) {
        mtk_uart_rx_push(s, buf[i]);
    }
    mtk_uart_update_irq(s);
    qemu_chr_fe_accept_input(&s->chr);
}

static uint64_t mtk_uart_read(void *opaque, hwaddr offset, unsigned size)
{
    MtkUartState *s = opaque;
    uint64_t val;

    switch (offset) {
    case MTK_RBR:
        if (s->lcr & MTK_LCR_DLAB) {
            val = s->regs[MTK_RBR / 4] & 0xff;
            break;
        }
        if (s->rx_count) {
            val = s->rx_fifo[s->rx_head];
            s->rx_head = (s->rx_head + 1) % MTK_UART_RX_FIFO;
            s->rx_count--;
        } else {
            val = 0;
        }
        mtk_uart_update_irq(s);
        break;
    case MTK_IER:
        val = s->regs[MTK_IER / 4] & 0xff;
        break;
    case MTK_IIR:
        val = mtk_uart_iir(s);
        /* Reading IIR acknowledges a pending THRE interrupt. */
        s->thre_ip = false;
        mtk_uart_update_irq(s);
        break;
    case MTK_LSR:
        val = mtk_uart_lsr(s);
        /* Reading LSR clears the latched break, as on real hardware. */
        s->break_pending = false;
        mtk_uart_update_irq(s);
        break;
    default:
        val = s->regs[offset / 4] & 0xff;
        break;
    }

    return val;
}

/* Recorded by hw/arm/mt6895.c. */
extern uint64_t mtk_win_ring[];
extern uint64_t mtk_win_pc_ring[];
extern unsigned mtk_win_ring_idx;

/* Recorded by hw/misc/mtk_spmi_pmif.c. */
extern uint32_t mtk_spmi_ring[];
extern unsigned mtk_spmi_ring_idx;

/* Recorded by accel/tcg/cpu-exec.c when MTK_PC_HIST is set. */
extern uint64_t mtk_pc_hist[];
extern unsigned mtk_pc_hist_idx;

static void mtk_uart_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    MtkUartState *s = opaque;
    uint8_t ch = value & 0xff;

    if (getenv("MTK_UART_PC")) {
        static int ncalls;

        if (ncalls < 5) {
            ncalls++;
            fprintf(stderr, "UARTWRITE-CALLED off=%#x val=%#x\n",
                    (unsigned)offset, (unsigned)value);
        }
    }

    switch (offset) {
    case MTK_RBR:
        if (s->lcr & MTK_LCR_DLAB) {
            s->regs[MTK_RBR / 4] = ch;
            break;
        }
        /*
         * The transmitter holding register is 8 bits wide. A guest that
         * drives the port with 32-bit accesses (Linux does, via
         * earlycon=uart8250,mmio32) still only transfers the low byte; the
         * upper bytes are not part of the character.
         */
        /*
         * MTK_UART_PC: emit one diagnostic line per serial line stating the
         * guest PC that produced it.  Correlating the firmware's own prints
         * with the code that ran is the only reliable way to locate routines
         * whose entry points are not separately observable.
         */
        if (getenv("MTK_UART_PC")) {
            static char line[192];
            static unsigned linelen;
            static int nlines;

            if (ch == '\n') {
                CPUState *cs = qemu_get_cpu(0);

                line[linelen] = 0;
                /*
                 * MTK_UART_FILTER: only report lines that contain one of the
                 * given substrings (comma separated), so a quiet early-boot
                 * window is not spent on unrelated output.
                 */
                {
                    const char *flt = getenv("MTK_UART_FILTER");
                    bool want = !flt || !*flt;

                    while (!want && flt && *flt) {
                        const char *comma = strchr(flt, ',');
                        size_t n = comma ? (size_t)(comma - flt) : strlen(flt);

                        if (n) {
                            size_t k = 0;

                            for (k = 0; line[k] && k + n <= strlen(line); k++) {
                                if (strncmp(line + k, flt, n) == 0) {
                                    want = true;
                                    break;
                                }
                            }
                        }
                        flt = comma ? comma + 1 : NULL;
                    }
                    if (want && nlines < 400) {
                        nlines++;
                        uint64_t pcv = 0;
                        uint64_t lrv = 0;
                        uint64_t spv = 0;

                        if (cs) {
                            CPUARMState *env = cpu_env(cs);

                            pcv = env->aarch64 ? env->pc
                                               : (uint64_t)env->regs[15];
                            lrv = env->aarch64 ? 0
                                               : (uint64_t)env->regs[14];
                            spv = env->aarch64 ? 0
                                               : (uint64_t)env->regs[13];
                        }
                        {
                            unsigned pi;

                            fprintf(stderr, "PCHIST | %s\n", line);
                            for (pi = 0; pi < 96; pi++) {
                                uint64_t pv =
                                    mtk_pc_hist[(mtk_pc_hist_idx + pi) % 96];

                                if (pv >= 0x02000000ULL && pv < 0x02400000ULL) {
                                    fprintf(stderr, "  PCHIST-ARM %#" PRIx64
                                            "\n", pv);
                                }
                            }
                        }
                        {
                            uint32_t stk[6] = { 0 };

                            if (cs && !cpu_env(cs)->aarch64 && spv) {
                                address_space_read(&address_space_memory,
                                                   spv, MEMTXATTRS_UNSPECIFIED,
                                                   stk, sizeof(stk));
                            }
                            fprintf(stderr,
                                    "UARTPC pc=%#" PRIx64 " lr=%#" PRIx64
                                    " sp=%#" PRIx64
                                    " stk=%08x,%08x,%08x,%08x,%08x,%08x"
                                    " | %s\n",
                                    pcv, lrv, spv,
                                    stk[0], stk[1], stk[2],
                                    stk[3], stk[4], stk[5], line);
                        }
                    }
                }
                if (line[0] && strstr(line, "mem read")) {
                    unsigned wi;

                    fprintf(stderr, "MEMREAD-WIN-DUMP idx=%u\n",
                            mtk_win_ring_idx);
                    for (wi = 0; wi < 64; wi++) {
                        uint64_t e =
                            mtk_win_ring[(mtk_win_ring_idx + wi) % 64];

                        if (e) {
                            fprintf(stderr, "  WIN off=%#" PRIx64
                                    " size=%u pc=%#" PRIx64 "\n",
                                    e >> 8, (unsigned)(e & 0xff),
                                    mtk_win_pc_ring[(mtk_win_ring_idx + wi)
                                                    % 64]);
                        }
                    }
                }
                if (line[0] && strstr(line, "vdd2l")) {
                    unsigned ri;

                    fprintf(stderr, "VDD2L-SPMI-DUMP idx=%u\n",
                            mtk_spmi_ring_idx);
                    for (ri = 0; ri < 64; ri++) {
                        uint32_t e =
                            mtk_spmi_ring[(mtk_spmi_ring_idx + ri) % 64];

                        if (e) {
                            fprintf(stderr,
                                    "  SPMI sid=%u addr=%#x %s val=%#x\n",
                                    e >> 24, (e >> 8) & 0xffff,
                                    (e >> 16) & 1 ? "R" : "W", e & 0xff);
                        }
                    }
                }
                linelen = 0;
            } else if (ch >= 0x20 && ch < 0x7f) {
                if (linelen + 1 < sizeof(line)) {
                    line[linelen++] = ch;
                }
            } else if (ch == '\r') {
                /* ignore */
            } else {
                linelen = 0;
            }
        }
        qemu_chr_fe_write_all(&s->chr, &ch, 1);
        /*
         * The byte leaves immediately, so THRE is set again and a THRE
         * interrupt becomes pending if the guest asked for one.
         */
        s->thre_ip = true;
        mtk_uart_update_irq(s);
        break;
    case MTK_IER:
        if (s->lcr & MTK_LCR_DLAB) {
            s->regs[MTK_IER / 4] = ch;
            break;
        }
        s->ier = ch;
        s->regs[MTK_IER / 4] = ch;
        /*
         * Enabling THRE interrupts while the transmitter is empty must raise
         * one immediately, otherwise the 8250 core would wait forever for a
         * transmit interrupt that never comes.
         */
        if (s->ier & MTK_IER_THRI) {
            s->thre_ip = true;
        }
        mtk_uart_update_irq(s);
        break;
    case MTK_IIR:
        if (ch & MTK_FCR_ENABLE_FIFO) {
            s->fcr |= MTK_FCR_ENABLE_FIFO;
        } else {
            s->fcr &= ~MTK_FCR_ENABLE_FIFO;
        }
        if (ch & MTK_FCR_CLEAR_RCVR) {
            s->rx_head = 0;
            s->rx_count = 0;
        }
        s->regs[MTK_IIR / 4] = ch & ~(MTK_FCR_CLEAR_RCVR |
                                      MTK_FCR_CLEAR_XMIT);
        mtk_uart_update_irq(s);
        break;
    case MTK_LCR:
        s->lcr = ch;
        break;
    case MTK_MCR:
        s->mcr = ch;
        break;
    default:
        s->regs[offset / 4] = ch;
        break;
    }
}

static const MemoryRegionOps mtk_uart_ops = {
    .read = mtk_uart_read,
    .write = mtk_uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void mtk_uart_reset_hold(Object *obj, ResetType type)
{
    MtkUartState *s = MTK_UART(obj);

    mtk_uart_reset_regs(s);
    qemu_set_irq(s->irq, 0);
}

static void mtk_uart_realize(DeviceState *dev, Error **errp)
{
    MtkUartState *s = MTK_UART(dev);

    qemu_chr_fe_set_handlers(&s->chr, mtk_uart_can_receive,
                             mtk_uart_receive, mtk_uart_event, NULL, s, NULL,
                             true);
}

static void mtk_uart_init(Object *obj)
{
    MtkUartState *s = MTK_UART(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    memory_region_init_io(&s->iomem, obj, &mtk_uart_ops, s,
                          TYPE_MTK_UART, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const Property mtk_uart_props[] = {
    DEFINE_PROP_CHR("chardev", MtkUartState, chr),
};

static void mtk_uart_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = mtk_uart_realize;
    device_class_set_props(dc, mtk_uart_props);
    dc->desc = "MediaTek MT6577 UART";
    rc->phases.hold = mtk_uart_reset_hold;
}

static const TypeInfo mtk_uart_info = {
    .name = TYPE_MTK_UART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkUartState),
    .instance_init = mtk_uart_init,
    .class_init = mtk_uart_class_init,
};

static void mtk_uart_register_types(void)
{
    type_register_static(&mtk_uart_info);
}

type_init(mtk_uart_register_types);
