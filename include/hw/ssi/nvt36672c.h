/*
 * Novatek NT36672C-family touch controller (SPI client), xaga / MT6895.
 *
 * Protocol verified from the vendor kernel driver
 * (firmware/ref/xaga-kernel/drivers/input/touchscreen/NVT36672C/nt36xxx.c):
 *  - page select: write {0xFF, addr>>15, addr>>7}
 *  - write:  first byte = addr | 0x80, then data bytes (address increments)
 *  - read:   first byte = addr & 0x7F, then the response stream is
 *            [2 leading bytes][payload...]; the driver copies from rx+2
 *  - chip id: 7-byte read at 0x3F004; the bytes after the first match the
 *            trim_id_table entry {0x0B,-,-,0x72,0x66,0x03} (NT36672A map,
 *            hw_crc=1), which is what this model presents
 *  - touch reports: 109-byte read at page 0x21C00 on IRQ; finger i record at
 *    1+6*i, byte = (id<<3)|status (1 enter, 2 moving), X=(p1<<8)|p2,
 *    Y=(p3<<8)|p4, checksum at byte 65 = two's complement of bytes 1..64
 *  - firmware download always runs at probe: 64512+1-byte SRAM writes, CRC
 *    bank programming, then the driver reads partition*4 bytes at 0x3F120
 *    and compares them with the image's own partition CRCs.  The model
 *    returns the genuine CRCs of the stock novatek_nt36672e_l16_fw01 image
 *    (the part is flashed with that firmware).
 *
 * Host input: a QEMU absolute pointer device; a VNC click/drag becomes a
 * touch coordinate which is synthesised into a real report packet.
 * Single finger.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SSI_NVT36672C_H
#define HW_SSI_NVT36672C_H

#include "hw/ssi/ssi.h"
#include "ui/input.h"
#include "qom/object.h"

#define TYPE_NVT36672C "nvt36672c"
OBJECT_DECLARE_SIMPLE_TYPE(Nvt36672cState, NVT36672C)

#define NVT_MEM_SIZE      (1u << 24)   /* 24-bit page/offset address space */
#define NVT_REPORT_ADDR   0x021c00u
#define NVT_REPORT_LEN    109
#define NVT_CRC_ADDR      0x03f120u
#define NVT_CRC_NUM       17

struct Nvt36672cState {
    SSIPeripheral parent_obj;

    qemu_irq irq;          /* touch interrupt, wired to the EINT block */
    QEMUTimer *irq_timer;  /* pulse/fallback deassert for edge-configured INT */
    QemuInputHandlerState *input;

    uint8_t *mem;

    /* SPI framing state. */
    bool in_frame;
    bool write_mode;
    bool page_cmd;
    unsigned page_bytes;
    uint8_t page_hi;
    uint32_t page_cur;
    uint32_t write_addr;
    uint32_t read_addr;
    int read_idx;
    unsigned clocks;
    bool frame_done;

    /* Model state. */
    uint16_t abs_x_max;
    uint16_t abs_y_max;
    uint16_t super_res;
    uint32_t report_base;   /* page the driver reads its report from */
    uint32_t touch_x, touch_y;
    bool touch_down;
    bool report_read_pending;
    bool trace;
};

#endif /* HW_SSI_NVT36672C_H */
