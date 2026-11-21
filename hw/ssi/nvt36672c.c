/*
 * Novatek NT36672C touch controller (SPI client).
 *
 * See the header for the verified protocol and the modelled device state.
 * This is a real SPI client on the MTK SPI controller's SSI bus: its chip ID
 * makes the vendor driver bind, its firmware-download path accepts the image
 * and its CRC readback matches that image, and touch reports are synthesised
 * from real host input events.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "qemu/timer.h"
#include "hw/core/qdev-properties.h"
#include "hw/ssi/nvt36672c.h"

#define NVT_LOG(s, ...)                             \
    do {                                            \
        if ((s)->trace) {                           \
            fprintf(stderr, "NVT36672C: ");         \
            fprintf(stderr, __VA_ARGS__);           \
        }                                           \
    } while (0)

/*
 * Partition CRCs of the stock firmware image the part is flashed with
 * (novatek_nt36672e_l16_fw01.bin, 139264 bytes, parsed from its NVT bin
 * header: ILM, DLM, 13 info sections, 2 overlay sections).  The driver reads
 * these back from 0x3F120 and compares them with the same image, so the
 * check passes because the image matches, not because a convenient value was
 * invented.
 */
static const uint32_t nvt_crcs[NVT_CRC_NUM] = {
    0x351cb070, 0x3854206b, 0x61826962, 0x434b996d, 0x59cdd4df,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x646c0677,
    0xdcb3f996, 0x1625d520,
};

/*
 * The driver reads a 109-byte report from the sticky page at offset 0 (the
 * first byte of the SPI read is 0x00, so the address is the page base).  The
 * page it uses is tracked at runtime (report_base) instead of assuming one:
 * the real chip likewise serves the report wherever the host's page pointer
 * points.  Layout as the driver consumes it: point_data[1] = mem[base],
 * so finger records start at base + 6*i, pressure at base + 98, and the
 * checksum covers point_data[1..64] = mem[base..base+63] and is stored at
 * point_data[65] = mem[base+64].
 */
/* Fallback deassert: a real touch INT pin is released once the host has
 * drained the report.  If the host never reads it (e.g. the edge was lost),
 * drop the line anyway so the next event produces a fresh rising edge. */
static void nvt_irq_fallback(void *opaque)
{
    Nvt36672cState *s = opaque;

    qemu_irq_lower(s->irq);
}

static void nvt_build_idle_report(Nvt36672cState *s)
{
    uint8_t *p = &s->mem[s->report_base];
    uint8_t sum = 0;
    unsigned i;

    memset(p, 0, NVT_REPORT_LEN);
    for (i = 0; i < 64; i++) {
        sum += p[i];
    }
    p[64] = (uint8_t)(0xFF - sum + 1);
}

static uint8_t nvt_mem_read(Nvt36672cState *s, uint32_t addr)
{
    static const uint8_t trim_id[6] = {
        0x0B, 0x00, 0x00, 0x72, 0x66, 0x03
    };

    if (addr >= 0x03f004 && addr < 0x03f004 + sizeof(trim_id)) {
        return trim_id[addr - 0x03f004];
    }
    if (addr >= 0x01f64e && addr < 0x01f64e + sizeof(trim_id)) {
        return trim_id[addr - 0x01f64e];
    }
    if (addr >= NVT_CRC_ADDR && addr < NVT_CRC_ADDR + NVT_CRC_NUM * 4) {
        unsigned idx = (addr - NVT_CRC_ADDR) / 4;

        return (nvt_crcs[idx] >> (((addr - NVT_CRC_ADDR) % 4) * 8)) & 0xff;
    }
    /* Firmware reset state / handshake: the running firmware is healthy. */
    if (addr == 0x021c60) {
        return 0xA3;
    }
    if (addr == 0x021c51) {
        return 0x00;
    }
    return addr < NVT_MEM_SIZE ? s->mem[addr] : 0;
}

static void nvt_frame_start(Nvt36672cState *s)
{
    s->in_frame = true;
    s->page_cmd = false;
    s->page_bytes = 0;
    /*
     * CTP_SPI_READ() copies the payload from rxbuf+2: the SPI response has
     * two header bytes (the command echo and one dummy), so the first data
     * byte is the third transfer.  read_idx = -1 serves exactly that:
     * clock 1 -> 0 (dummy), clock 2 -> mem[read_addr] (first data byte).
     * (read_idx = -2 serves one byte too late; the driver then reads
     * buf[1] = 0x00 and reports "chip is not identified".)
     */
    s->read_idx = -1;
    s->write_mode = false;
    s->clocks = 0;
    s->frame_done = false;
}

static void nvt_frame_end(Nvt36672cState *s)
{
    s->in_frame = false;
    if (s->report_read_pending) {
        s->report_read_pending = false;
        if (s->irq_timer) {
            timer_del(s->irq_timer);
        }
        qemu_irq_lower(s->irq);
        NVT_LOG(s, "report read, irq lower\n");
        if (getenv("MTK_NVTCHAIN")) {
            fprintf(stderr, "NVT-CHAIN reportread ts=%lld\n",
                    (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        }
    }
    if (!s->touch_down) {
        nvt_build_idle_report(s);
    }
}

static void nvt_frame_gpio(void *opaque, int n, int level)
{
    Nvt36672cState *s = NVT36672C(opaque);

    if (level) {
        nvt_frame_start(s);
    } else {
        nvt_frame_end(s);
    }
}

static uint32_t nvt_transfer(SSIPeripheral *dev, uint32_t val)
{
    Nvt36672cState *s = NVT36672C(dev);
    uint8_t b = val & 0xff;

    if (!s->in_frame) {
        return 0;
    }
    if (s->clocks++ == 0) {
        /* first byte of the frame: command/address */
        if (s->trace) {
            static unsigned nframe;

            if (nframe++ < 60) {
                fprintf(stderr, "NVT36672C: frame cmd=%#04x\n", b);
            }
            if (getenv("MTK_NVTCHAIN") && nframe < 4000) {
                fprintf(stderr, "NVT-CHAIN frame ts=%lld cmd=%#04x\n",
                        (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), b);
            }
        }
        s->page_cmd = (b == 0xFF);
        s->write_mode = (b & 0x80) != 0;
        if (!s->page_cmd) {
            uint32_t addr = s->page_cur | (b & 0x7f);

            s->write_addr = addr;
            s->read_addr = addr;
            /*
             * A read at page offset 0 is the touch-report fetch; remember the
             * page so reports are built where the host will read them.
             */
            if (!s->write_mode && (b & 0x7f) == 0x00) {
                s->report_base = s->page_cur;
                s->report_read_pending = true;
                if (getenv("MTK_NVTCHAIN")) {
                    fprintf(stderr, "NVT-CHAIN readreq ts=%lld page=%#x\n",
                            (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                            s->page_cur);
                }
            }
        }
        return 0;
    }

    if (s->frame_done) {
        /* Word padding at the end of a short transfer: ignore. */
        return 0;
    }
    if (s->page_cmd) {
        if (s->page_bytes == 0) {
            s->page_hi = b;
        } else {
            s->page_cur = ((uint32_t)s->page_hi << 15) | ((uint32_t)b << 7);
            s->page_cmd = false;
            s->frame_done = true;   /* page select is exactly 3 bytes */
            NVT_LOG(s, "page %#x\n", s->page_cur);
        }
        s->page_bytes++;
        return 0;
    }

    if (s->write_mode) {
        if (s->write_addr < NVT_MEM_SIZE) {
            s->mem[s->write_addr] = b;
        }
        s->write_addr++;
        return 0;
    }

    /* Read: two leading bytes are discarded by the driver, then payload. */
    if (s->read_idx < 0) {
        s->read_idx++;
        return 0;
    }
    {
        uint8_t out = nvt_mem_read(s, s->read_addr + (uint32_t)s->read_idx);

        s->read_idx++;
        return out;
    }
}

static void nvt_touch_report(Nvt36672cState *s, uint8_t status)
{
    uint8_t *p = &s->mem[s->report_base];
    uint8_t sum = 0;
    uint32_t x = s->touch_x;
    uint32_t y = s->touch_y;
    unsigned i;

    memset(p, 0, NVT_REPORT_LEN);
    p[0] = (uint8_t)((1u << 3) | (status & 0x07));
    p[1] = (x >> 8) & 0xff;
    p[2] = x & 0xff;
    p[3] = (y >> 8) & 0xff;
    p[4] = y & 0xff;
    p[5] = 1;
    p[98] = 1;
    for (i = 0; i < 64; i++) {
        sum += p[i];
    }
    p[64] = (uint8_t)(0xFF - sum + 1);
    NVT_LOG(s, "touch report x=%u y=%u status=%u\n", x, y, status);
    if (getenv("MTK_NVTCHAIN")) {
        fprintf(stderr, "NVT-CHAIN pack ts=%lld p0=%02x p1=%02x p2=%02x p3=%02x p4=%02x p5=%02x base=%#x\n",
                (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                p[0], p[1], p[2], p[3], p[4], p[5], s->report_base);
    }
    if (getenv("MTK_NVTCHAIN")) {
        fprintf(stderr, "NVT-CHAIN report ts=%lld x=%u y=%u status=%u pending=%d\n",
                (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), x, y, status,
                (int)s->report_read_pending);
    }
    /*
     * Assert the INT line while the report is unread, but do not hold it
     * forever: the guest configures the line edge-triggered (trigger type 1),
     * so a stuck-high line would swallow later edges.  The frame-end path
     * lowers it as soon as the report has been read; the timer is the
     * fallback for a read that never happens.
     */
    qemu_irq_raise(s->irq);
    if (s->irq_timer) {
        /*
         * Hold INT asserted while the report is unread (real controllers do
         * this); the 2 ms pulse was too short: a level-pending SPI that is
         * rejected behind a same-priority live PPI is lost if the line drops
         * before the next full rescan.  Keep a generous safety timeout only so
         * a missed read cannot wedge the line forever.
         */
        timer_mod(s->irq_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 50000000);
    }
}

static void nvt_input_event(DeviceState *dev, QemuConsole *src,
                            QemuInputEvent *evt)
{
    Nvt36672cState *s = NVT36672C(dev);

    if (getenv("MTK_NVTCHAIN")) {
        if (evt->type == INPUT_EVENT_KIND_ABS) {
            fprintf(stderr, "NVT-CHAIN input abs axis=%d val=%d tx=%u ty=%u\n",
                    (int)evt->abs.axis, (int)evt->abs.value, s->touch_x, s->touch_y);
        } else if (evt->type == INPUT_EVENT_KIND_BTN) {
            fprintf(stderr, "NVT-CHAIN input btn button=%d down=%d tx=%u ty=%u\n",
                    (int)evt->btn.button, (int)evt->btn.down, s->touch_x, s->touch_y);
        } else {
            fprintf(stderr, "NVT-CHAIN input kind=%d tx=%u ty=%u\n",
                    (int)evt->type, s->touch_x, s->touch_y);
        }
    }
    switch (evt->type) {
    case INPUT_EVENT_KIND_REL:
    {
        /*
         * REL sources are VNC clients that never negotiated absolute mode:
         * their events arrive as pointer deltas and used to be dropped by the
         * handler mask, leaving the report stuck at (0,0).  Accumulate the
         * deltas as console-pixel moves of the emulated finger so an
         * interactive drag works end-to-end; ABS injectors keep the exact
         * 0..0x7fff scaling path above.
         */
        int64_t max_x = (int64_t)s->abs_x_max * s->super_res - 1;
        int64_t max_y = (int64_t)s->abs_y_max * s->super_res - 1;

        if (!s->rel_seeded) {
            s->touch_x = (uint32_t)s->abs_x_max * s->super_res / 2;
            s->touch_y = (uint32_t)s->abs_y_max * s->super_res / 2;
            s->rel_seeded = true;
        }
        if (evt->rel.axis == INPUT_AXIS_X) {
            int64_t v = (int64_t)s->touch_x + evt->rel.value;

            s->touch_x = (uint32_t)(v < 0 ? 0 : (v > max_x ? max_x : v));
        } else if (evt->rel.axis == INPUT_AXIS_Y) {
            int64_t v = (int64_t)s->touch_y + evt->rel.value;

            s->touch_y = (uint32_t)(v < 0 ? 0 : (v > max_y ? max_y : v));
        }
        if (s->touch_down) {
            nvt_touch_report(s, 2);
        }
        break;
    }
    case INPUT_EVENT_KIND_ABS:
        if (evt->abs.axis == INPUT_AXIS_X) {
            s->touch_x = (uint32_t)((uint64_t)evt->abs.value *
                                    (s->abs_x_max * s->super_res - 1) / 0x7fff);
        } else if (evt->abs.axis == INPUT_AXIS_Y) {
            s->touch_y = (uint32_t)((uint64_t)evt->abs.value *
                                    (s->abs_y_max * s->super_res - 1) / 0x7fff);
        }
        if (s->touch_down) {
            nvt_touch_report(s, 2);
        }
        break;
    case INPUT_EVENT_KIND_BTN:
        if (evt->btn.button == INPUT_BUTTON_LEFT) {
            if (evt->btn.down && !s->touch_down) {
                s->touch_down = true;
                nvt_touch_report(s, 1);
            } else if (!evt->btn.down && s->touch_down) {
                s->touch_down = false;
                nvt_build_idle_report(s);
                qemu_irq_raise(s->irq);
            }
        }
        break;
    default:
        break;
    }
}

static const QemuInputHandler nvt_input_handler = {
    .name = "nvt36672c-touch",
    .mask = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS |
            INPUT_EVENT_MASK_REL,
    .event = nvt_input_event,
};

static void nvt_ssi_realize(SSIPeripheral *dev, Error **errp)
{
    Nvt36672cState *s = NVT36672C(dev);

    s->mem = g_malloc0(NVT_MEM_SIZE);
    s->trace = getenv("MTK_NVT_TRACE") != NULL;
    s->abs_x_max = 1080;
    s->abs_y_max = 2460;
    s->super_res = 10;
    s->report_base = NVT_REPORT_ADDR;
    /*
     * Firmware info block read by nvt_get_fw_info(): the driver reads 17
     * bytes at 0x21C78 and uses buf[1] as fw_ver, so the payload starts at
     * mem[0x21C78] (buf[k] = mem[0x21C77+k]).
     */
    s->mem[0x021c78] = 0x43;         /* fw_ver (0x43 + 0xBC = 0xFF) */
    s->mem[0x021c79] = 0xBC;
    s->mem[0x021c7a] = 18;           /* x_num */
    s->mem[0x021c7b] = 32;           /* y_num */
    s->mem[0x021c7c] = 0x04;         /* abs_x_max = 1080 */
    s->mem[0x021c7d] = 0x38;
    s->mem[0x021c7e] = 0x09;         /* abs_y_max = 2460 */
    s->mem[0x021c7f] = 0x9c;
    nvt_build_idle_report(s);
    qdev_init_gpio_in(DEVICE(dev), nvt_frame_gpio, 1);
    qdev_init_gpio_out(DEVICE(dev), &s->irq, 1);
    s->irq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, nvt_irq_fallback, s);
    s->input = qemu_input_handler_register(DEVICE(dev), &nvt_input_handler);
    fprintf(stderr, "NVT36672C: model at spi2/CS0, trim id 0x0B/0x72/0x66/0x03\n");
}

static void nvt_unrealize(DeviceState *dev)
{
    Nvt36672cState *s = NVT36672C(dev);

    qemu_input_handler_unregister(s->input);
    timer_free(s->irq_timer);
    g_free(s->mem);
}

static void nvt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    dc->unrealize = nvt_unrealize;
    dc->desc = "Novatek NT36672C touch controller (SPI)";
    k->realize = nvt_ssi_realize;
    k->transfer = nvt_transfer;
    k->cs_polarity = SSI_CS_NONE;
}

static const TypeInfo nvt_info = {
    .name = TYPE_NVT36672C,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(Nvt36672cState),
    .class_init = nvt_class_init,
};

static void nvt_register_types(void)
{
    type_register_static(&nvt_info);
}

type_init(nvt_register_types)
