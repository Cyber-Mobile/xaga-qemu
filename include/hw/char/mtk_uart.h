/*
 * MediaTek MT6577-compatible UART
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_CHAR_MTK_UART_H
#define HW_CHAR_MTK_UART_H

#include "chardev/char-fe.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_MTK_UART "mtk-uart"
OBJECT_DECLARE_SIMPLE_TYPE(MtkUartState, MTK_UART)

/* Number of bytes of RX FIFO we model. */
#define MTK_UART_RX_FIFO 64

struct MtkUartState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    CharFrontend chr;
    qemu_irq irq;

    /*
     * Register file. The MTK UART is a 16550 whose registers are spaced
     * 4 bytes apart (Linux drives it with UPIO_MEM32 + regshift = 2), so a
     * raw byte-addressed array indexed by offset/4 is the natural model.
     */
    uint32_t regs[0x100 / 4];

    /*
     * A break condition is latched until the guest reads LSR, mirroring the
     * 16550's BI bit. Linux's SysRq over serial is delivered this way, which
     * is what makes an external 'dump all task states' possible.
     */
    bool break_pending;

    uint8_t rx_fifo[MTK_UART_RX_FIFO];
    unsigned int rx_head;
    unsigned int rx_count;

    /* Cached state mirrored from the register file, kept for clarity. */
    uint8_t lcr;
    uint8_t fcr;
    uint8_t ier;
    uint8_t mcr;

    /*
     * Transmitter-holding-register-empty interrupt is pending. It is raised
     * when the guest enables THRE interrupts or writes a byte (which the
     * model transmits immediately), and acknowledged by reading IIR.
     */
    bool thre_ip;

    /*
     * Transmit staging buffer.  The guest writes the console one byte per
     * register write; handing every byte to the chardev costs one host write
     * syscall per byte (several microseconds on the Windows build host).  A
     * line at a time is written instead.  Nothing guest-visible depends on
     * it: LSR.THRE is raised as soon as the byte is accepted.
     */
    uint8_t tx_buf[256];
    unsigned int tx_len;
};

#endif /* HW_CHAR_MTK_UART_H */
