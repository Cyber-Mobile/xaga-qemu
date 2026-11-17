/*
 * MediaTek MT6895 AFE (audio front end) model.
 *
 * Hardware described by the vendor device tree (vendor_boot_a.dtb):
 *
 *   /mt6895-afe-pcm@1e100000
 *       compatible = "mediatek,mt6895-sound"
 *       reg        = <0 0x1e100000 0 0x2000>
 *       interrupts = <0 416 4 0>              -- GIC SPI 416, level high
 *       power-domains = <&scpsys 4>           -- audio
 *       clocks     = 54 entries (20 x &afe_clk @1e100000, 34 x topckgen)
 *
 * The vendor audio stack (AFE PCM + machine driver + MT6368 codec glue) is a
 * single out-of-tree module, mtk-afe-external.ko, which is NOT present in any
 * image we currently have (see docs/xaga/architecture.md).  This model is
 * therefore written so that it is useful in two stages:
 *
 *   1. Tone self-test: with MTK_AFE_TONE=<hz> (or -device mtk-afe,tone=<hz>)
 *      the device synthesises PCM into the QEMU audio backend.  No guest
 *      driver is needed; this verifies the host audio path end to end.
 *
 *   2. Driver bring-up: once mtk-afe-external.ko can be insmod'ed, the full
 *      register file is kept (writes read back) and the model watches for a
 *      memif ring buffer: writes of two guest-physical DRAM addresses
 *      (base < end, 0x1000..0x400000 apart) are treated as the ring bounds,
 *      PCM is then streamed from guest memory to the audio backend at 48 kHz
 *      S16 stereo.  MTK_AFETRACE=1 logs every access so the real memif register
 *      layout can be filled in from the driver trace afterwards.
 *
 * Deliberately not modelled yet: I2S/TDM/ETDM links, the ADSP/SCP/DSP task
 * graph (snd_audio_dsp), BT CVSD, and codec semantics (MT6368 lives on SPMI
 * slave 5 and is served by hw/misc/mtk_spmi_pmif.c's flat register space).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/audio.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/audio/mtk_afe.h"
#include "system/address-spaces.h"
#include "system/memory.h"

#define MTK_AFE_RATE            48000
#define MTK_AFE_CHANNELS        2
#define MTK_AFE_FRAME_BYTES     (MTK_AFE_CHANNELS * 2)   /* S16 */
#define MTK_AFE_MIXBUF_SIZE     8192
#define MTK_AFE_TICK_NS         (1 * NANOSECONDS_PER_SECOND)

/* Guest DRAM window (include/hw/arm/mt6895.h: MT6895_DRAM_BASE). */
#define MTK_AFE_DRAM_LO         0x40000000ULL
#define MTK_AFE_DRAM_HI         0xc0000000ULL
#define MTK_AFE_RING_MIN        0x1000u
#define MTK_AFE_RING_MAX        0x400000u
#define MTK_AFE_NUM_CAND        8

/*
 * MediaTek clock-gate convention: per 0x100 block, +0x00 CLR, +0x04 SET,
 * +0x08 STA.  The AFE window 0x000..0xfff doubles as the afe_clk provider
 * (DT /soc/syscon@1e100000, compatible "mediatek,mt6895-afe","syscon"), so the
 * clock framework may poll the STA words.  This generic emulation keeps such
 * polls stable; disable with MTK_NO_AFE_CG=1.
 */
#define MTK_AFE_CG_BLOCKS       (0x1000 / 0x100)

struct MtkAfeState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    AudioBackend *audio_be;
    SWVoiceOut *voice;
    QEMUTimer *timer;

    uint32_t regs[MTK_AFE_MMIO_SIZE / 4];
    uint8_t *mixbuf;
    size_t mixbuf_size;

    /* Tone self-test. */
    uint32_t tone_hz;
    uint32_t phase;

    /* Logging / debug. */
    bool trace;
    bool irq_test;
    uint64_t bytes_out;
    unsigned trace_quota;

    /* Auto-detected memif ring (prototype). */
    uint32_t ring_base;
    uint32_t ring_end;
    uint32_t ring_read;
    uint32_t cand[MTK_AFE_NUM_CAND];
    unsigned ncand;
    unsigned nrings;
};

static bool mtk_afe_env_on(const char *name)
{
    const char *v = getenv(name);

    return v && *v && strcmp(v, "0") != 0;
}

static void mtk_afe_trace(MtkAfeState *s, const char *rw, hwaddr off,
                          uint32_t val)
{
    if (!s->trace || s->trace_quota == 0) {
        return;
    }
    s->trace_quota--;
    fprintf(stderr, "MTK-AFE %s %#" HWADDR_PRIx " = %#x\n", rw, off, val);
}

/* ---- memif ring handling (prototype) ------------------------------------ */

static size_t mtk_afe_ring_fill(MtkAfeState *s, uint8_t *dst, size_t len)
{
    uint32_t ring_sz;
    size_t done = 0;

    if (!s->ring_base || s->ring_end <= s->ring_base) {
        return 0;
    }
    ring_sz = s->ring_end - s->ring_base;
    len = MIN(len, (size_t)ring_sz);
    len -= len % MTK_AFE_FRAME_BYTES;
    while (done < len) {
        uint32_t pos = s->ring_read % ring_sz;
        uint32_t chunk = MIN(len - done, (size_t)(ring_sz - pos));

        address_space_read(&address_space_memory, s->ring_base + pos,
                           MEMTXATTRS_UNSPECIFIED, dst + done, chunk);
        done += chunk;
        s->ring_read += chunk;
    }
    return done;
}

/*
 * Watch register writes for a plausible memif ring: two aligned guest DRAM
 * addresses whose distance looks like a PCM ring.  This is intentionally
 * heuristic - the exact memif register offsets come from the vendor module and
 * are filled in after the first MTK_AFETRACE capture.
 */
static void mtk_afe_note_candidate(MtkAfeState *s, hwaddr off, uint32_t val)
{
    unsigned i;

    if (val < MTK_AFE_DRAM_LO || val >= MTK_AFE_DRAM_HI || (val & 3)) {
        return;
    }
    if (s->ncand == MTK_AFE_NUM_CAND) {
        memmove(s->cand, s->cand + 1,
                (MTK_AFE_NUM_CAND - 1) * sizeof(s->cand[0]));
        s->ncand--;
    }
    s->cand[s->ncand++] = val;

    for (i = 0; i + 1 < s->ncand; i++) {
        uint32_t lo = s->cand[i], hi = s->cand[i + 1];

        if (hi <= lo) {
            continue;
        }
        if (hi - lo < MTK_AFE_RING_MIN || hi - lo > MTK_AFE_RING_MAX) {
            continue;
        }
        if ((lo & 0xfff) || (hi & 0xfff)) {
            continue;
        }
        s->ring_base = lo;
        s->ring_end = hi;
        s->ring_read = 0;
        s->nrings++;
        fprintf(stderr, "MTK-AFE ring detected base=%#" PRIx32
                " end=%#" PRIx32 " size=%u (write off=%#" HWADDR_PRIx ")\n",
                lo, hi, hi - lo, off);
        return;
    }
}

/* ---- MMIO -------------------------------------------------------------- */

static uint64_t mtk_afe_read(void *opaque, hwaddr off, unsigned size)
{
    MtkAfeState *s = opaque;
    uint32_t word;
    uint64_t ret;

    if (off + 4 > MTK_AFE_MMIO_SIZE) {
        return 0;
    }
    word = s->regs[off / 4];
    switch (size) {
    case 1:
        ret = (word >> ((off & 3) * 8)) & 0xff;
        break;
    case 2:
        ret = (word >> ((off & 3) * 8)) & 0xffff;
        break;
    default:
        ret = word;
        break;
    }
    if (s->trace && s->trace_quota) {
        s->trace_quota--;
        fprintf(stderr, "MTK-AFE R %#" HWADDR_PRIx " -> %#" PRIx64 "\n",
                off, ret);
    }
    return ret;
}

static void mtk_afe_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    MtkAfeState *s = opaque;
    uint32_t v32 = (uint32_t)val;

    if (off + 4 > MTK_AFE_MMIO_SIZE) {
        return;
    }
    s->regs[off / 4] = v32;
    mtk_afe_trace(s, "W", off, v32);
    mtk_afe_note_candidate(s, off, v32);

    /* Generic clock-gate CLR/SET/STA emulation for the afe_clk sub-window. */
    if (off < 0x1000 && !mtk_afe_env_on("MTK_NO_AFE_CG")) {
        hwaddr grp = off & ~(hwaddr)0xff;
        hwaddr sub = off & 0xff;
        hwaddr sta = grp + 0x08;

        if (sub == 0x04 && sta + 4 <= 0x1000) {
            s->regs[sta / 4] |= v32;
        } else if (sub == 0x00 && sta + 4 <= 0x1000) {
            s->regs[sta / 4] &= ~v32;
        }
    }
}

static const MemoryRegionOps mtk_afe_ops = {
    .read = mtk_afe_read,
    .write = mtk_afe_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* ---- audio backend ----------------------------------------------------- */

static void mtk_afe_tone_fill(MtkAfeState *s, uint8_t *dst, size_t len)
{
    int16_t *p = (int16_t *)dst;
    size_t frames = len / MTK_AFE_FRAME_BYTES;
    uint32_t step = (uint32_t)(((uint64_t)s->tone_hz << 32) / MTK_AFE_RATE);
    size_t i;

    for (i = 0; i < frames; i++) {
        int16_t sample;

        s->phase += step;
        sample = (s->phase & 0x80000000u) ? 6000 : -6000;
        p[2 * i] = sample;
        p[2 * i + 1] = sample;
    }
}

static void mtk_afe_out_cb(void *opaque, int avail)
{
    MtkAfeState *s = opaque;
    size_t len, filled;

    if (!s->voice || avail <= 0) {
        return;
    }
    len = MIN((size_t)avail, s->mixbuf_size);
    len -= len % MTK_AFE_FRAME_BYTES;
    if (!len) {
        return;
    }
    if (s->tone_hz) {
        mtk_afe_tone_fill(s, s->mixbuf, len);
    } else {
        filled = mtk_afe_ring_fill(s, s->mixbuf, len);
        if (filled < len) {
            memset(s->mixbuf + filled, 0, len - filled);
        }
    }
    audio_be_write(s->audio_be, s->voice, s->mixbuf, len);
    s->bytes_out += len;
}

/* ---- device ------------------------------------------------------------ */

static void mtk_afe_tick(void *opaque);

static void mtk_afe_reset(DeviceState *dev)
{
    MtkAfeState *s = MTK_AFE(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->ring_base = s->ring_end = s->ring_read = 0;
    s->ncand = 0;
    s->phase = 0;
}

static void mtk_afe_realize(DeviceState *dev, Error **errp)
{
    MtkAfeState *s = MTK_AFE(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    struct audsettings as = {
        .freq = MTK_AFE_RATE,
        .nchannels = MTK_AFE_CHANNELS,
        .fmt = AUDIO_FORMAT_S16,
        .big_endian = false,
    };
    Error *local_err = NULL;
    const char *env;

    /* Environment fallbacks so the machine wiring can stay minimal. */
    if (!s->trace) {
        s->trace = mtk_afe_env_on("MTK_AFETRACE");
    }
    s->irq_test = mtk_afe_env_on("MTK_AFE_IRQ_TEST");
    if (!s->tone_hz) {
        env = getenv("MTK_AFE_TONE");
        if (env && *env) {
            s->tone_hz = strtoul(env, NULL, 0);
        }
    }
    if (!s->audio_be) {
        env = getenv("MTK_AFE_AUDIODEV");
        if (env && *env) {
            qdev_prop_set_string(dev, "audiodev", env);
        }
    }
    /*
     * Audio is optional: never fail machine creation just because the host
     * has no audio backend configured (the register window must still be
     * usable for driver bring-up).
     */
    if (!audio_be_check(&s->audio_be, &local_err)) {
        warn_report("mtk-afe: no audio backend (%s); running silent",
                    error_get_pretty(local_err));
        error_free(local_err);
        local_err = NULL;
        s->audio_be = NULL;
    }

    s->mixbuf_size = MTK_AFE_MIXBUF_SIZE;
    s->mixbuf = g_malloc0(s->mixbuf_size);
    if (s->audio_be) {
        s->voice = audio_be_open_out(s->audio_be, NULL, "mtk-afe", s,
                                     mtk_afe_out_cb, &as);
    }
    if (s->voice) {
        audio_be_set_active_out(s->audio_be, s->voice, true);
    } else {
        fprintf(stderr, "MTK-AFE: no output voice (silent model)\n");
    }

    memory_region_init_io(&s->iomem, OBJECT(s), &mtk_afe_ops, s,
                          "mtk-afe", MTK_AFE_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->trace_quota = s->trace ? 4096 : 0;
    s->timer = timer_new_ns(QEMU_CLOCK_REALTIME, mtk_afe_tick, s);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) +
                        MTK_AFE_TICK_NS);
    fprintf(stderr, "MTK-AFE: realize size=%#x rate=%d ch=%d tone=%u"
            " audiodev=%s trace=%d\n", MTK_AFE_MMIO_SIZE, MTK_AFE_RATE,
            MTK_AFE_CHANNELS, s->tone_hz,
            s->audio_be ? audio_be_get_id(s->audio_be) : "none",
            s->trace);
}

static void mtk_afe_tick(void *opaque)
{
    MtkAfeState *s = opaque;

    if (s->trace) {
        fprintf(stderr, "MTK-AFE: bytes_out=%" PRIu64 " rings=%u base=%#" PRIx32
                " end=%#" PRIx32 "\n", s->bytes_out, s->nrings,
                s->ring_base, s->ring_end);
    }
    if (s->irq_test) {
        qemu_irq_pulse(s->irq);
    }
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) +
                        MTK_AFE_TICK_NS);
}

static const Property mtk_afe_props[] = {
    DEFINE_AUDIO_PROPERTIES(MtkAfeState, audio_be),
    DEFINE_PROP_UINT32("tone", MtkAfeState, tone_hz, 0),
    DEFINE_PROP_BOOL("trace", MtkAfeState, trace, false),
};

static void mtk_afe_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mtk_afe_realize;
    device_class_set_legacy_reset(dc, mtk_afe_reset);
    device_class_set_props(dc, mtk_afe_props);
    dc->desc = "MediaTek MT6895 AFE (register file + memif ring + audio voice)";
    /* Allow "-device mtk-afe,audiodev=snd0,tone=440" for the tone self-test. */
    dc->user_creatable = true;
}

/*
 * Development scaffolding: a user-creatable wrapper device that instantiates
 * the sysbus AFE, maps its register window and pokes it from a timer, so the
 * tone self-test and the register file can be exercised before the device is
 * wired into the MT6895 machine:
 *
 *   qemu-system-aarch64 -M none -device mtk-afe-selftest,audiodev=snd0,tone=440 \
 *       -audiodev wav,id=snd0,path=tone.wav
 *
 * It is a plain TYPE_DEVICE (not sysbus) so that "-device" works on any
 * machine, and it is not referenced by the MT6895 machine at all.
 */
#define TYPE_MTK_AFE_SELFTEST   "mtk-afe-selftest"
#define MTK_AFE_SELFTEST_BASE   0x1e100000ULL
#define MTK_AFE_SELFTEST_NS     250000000LL   /* 250 ms */

typedef struct MtkAfeSelftestState {
    DeviceState parent_obj;

    AudioBackend *audio_be;
    uint32_t tone_hz;
    bool trace;

    QEMUTimer *timer;
    unsigned step;
} MtkAfeSelftestState;

OBJECT_DECLARE_SIMPLE_TYPE(MtkAfeSelftestState, MTK_AFE_SELFTEST)

static void mtk_afe_selftest_poke(void *opaque)
{
    MtkAfeSelftestState *s = opaque;
    uint32_t w, r;
    bool ok = true;

    /* Plain register file: write/read at an arbitrary offset. */
    w = 0x5a5a0000u | s->step;
    address_space_write(&address_space_memory, MTK_AFE_SELFTEST_BASE + 0x1234,
                        MEMTXATTRS_UNSPECIFIED, &w, 4);
    r = 0;
    address_space_read(&address_space_memory, MTK_AFE_SELFTEST_BASE + 0x1234,
                       MEMTXATTRS_UNSPECIFIED, &r, 4);
    if (r != w) {
        fprintf(stderr, "MTK-AFE-SELFTEST: shadow readback FAIL (%#x != %#x)\n",
                r, w);
        ok = false;
    }

    /* Clock-gate SET/STA and CLR/STA semantics of block 0. */
    w = 0x10;
    address_space_write(&address_space_memory, MTK_AFE_SELFTEST_BASE + 0x04,
                        MEMTXATTRS_UNSPECIFIED, &w, 4);
    r = 0;
    address_space_read(&address_space_memory, MTK_AFE_SELFTEST_BASE + 0x08,
                       MEMTXATTRS_UNSPECIFIED, &r, 4);
    if (r & 0x10) {
        address_space_write(&address_space_memory, MTK_AFE_SELFTEST_BASE + 0x00,
                            MEMTXATTRS_UNSPECIFIED, &w, 4);
        r = 0xffffffff;
        address_space_read(&address_space_memory, MTK_AFE_SELFTEST_BASE + 0x08,
                           MEMTXATTRS_UNSPECIFIED, &r, 4);
        if (r & 0x10) {
            fprintf(stderr, "MTK-AFE-SELFTEST: clock-gate CLR FAIL (%#x)\n", r);
            ok = false;
        }
    } else {
        fprintf(stderr, "MTK-AFE-SELFTEST: clock-gate SET FAIL\n");
        ok = false;
    }

    /* Last word of the window must be readable too. */
    r = 0;
    address_space_read(&address_space_memory, MTK_AFE_SELFTEST_BASE + 0x1ffc,
                       MEMTXATTRS_UNSPECIFIED, &r, 4);

    fprintf(stderr, "MTK-AFE-SELFTEST: %s step=%u (shadow=%#x tail=%#x)\n",
            ok ? "PASS" : "FAIL", s->step, w, r);
    if (s->step++ < 3) {
        timer_mod(s->timer,
                  qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + MTK_AFE_SELFTEST_NS);
    }
}

static void mtk_afe_selftest_realize(DeviceState *dev, Error **errp)
{
    MtkAfeSelftestState *s = MTK_AFE_SELFTEST(dev);
    DeviceState *afe = qdev_new(TYPE_MTK_AFE);

    if (s->audio_be) {
        qdev_prop_set_string(afe, "audiodev", audio_be_get_id(s->audio_be));
    }
    if (s->tone_hz) {
        qdev_prop_set_uint32(afe, "tone", s->tone_hz);
    }
    if (s->trace) {
        qdev_prop_set_bit(afe, "trace", true);
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(afe), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(afe), 0, MTK_AFE_SELFTEST_BASE);

    s->timer = timer_new_ns(QEMU_CLOCK_REALTIME, mtk_afe_selftest_poke, s);
    timer_mod(s->timer,
              qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + MTK_AFE_SELFTEST_NS);
}

static const Property mtk_afe_selftest_props[] = {
    DEFINE_AUDIO_PROPERTIES(MtkAfeSelftestState, audio_be),
    DEFINE_PROP_UINT32("tone", MtkAfeSelftestState, tone_hz, 0),
    DEFINE_PROP_BOOL("trace", MtkAfeSelftestState, trace, false),
};

static void mtk_afe_selftest_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mtk_afe_selftest_realize;
    device_class_set_props(dc, mtk_afe_selftest_props);
    dc->desc = "MTK AFE tone/MMIO self-test wrapper (development scaffolding)";
    dc->user_creatable = true;
}

static const TypeInfo mtk_afe_types[] = {
    {
        .name          = TYPE_MTK_AFE,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(MtkAfeState),
        .class_init    = mtk_afe_class_init,
    },
    {
        .name          = TYPE_MTK_AFE_SELFTEST,
        .parent        = TYPE_DEVICE,
        .instance_size = sizeof(MtkAfeSelftestState),
        .class_init    = mtk_afe_selftest_class_init,
    },
};

DEFINE_TYPES(mtk_afe_types)

