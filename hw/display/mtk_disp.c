/*
 * MediaTek display scan-out model.
 *
 * Models the pixel path OVL -> ... -> DSI only as far as "which memory is on
 * screen": at every refresh the OVL layer registers (read through the system
 * address space, i.e. whatever the guest programmed into the OVL windows) are
 * decoded and the enabled layers are composed into a width x height canvas.
 *
 * OVL register layout (upstream drivers/gpu/drm/mediatek/mtk_disp_ovl.c and
 * the downstream mtk_disp_ovl.c):
 *   0x020 ROI_SIZE (h<<16|w)    0x02c SRC_CON (bit n = layer n enable)
 *   0x030+0x20n L_CON  [15:12] CLRFMT, bit24 BYTE_SWAP, bit25 RGB_SWAP
 *   0x038+0x20n L_SRC_SIZE (h<<16|w)   0x03c+0x20n L_OFFSET (y<<16|x)
 *   0x044+0x20n L_PITCH [15:0] bytes   0xf40+0x20n L_ADDR
 * CLRFMT: 0 RGB565, 1 RGB888, 2 RGBA8888 (memory = DRM XRGB8888),
 *         3 ARGB8888 (memory = DRM RGBX8888), 4 UYVY, 5 YUYV.
 *
 * If no OVL layer is enabled, property "fb-fallback" (e.g. LK's fb_base) is
 * scanned as XRGB8888 with pitch ALIGN(width,32)*4, when non-zero.
 *
 * Output: a QEMU graphic console (works with HMP "screendump") and, when
 * "dump-path" is set, a binary PPM written every dump-ms milliseconds of host
 * time whenever the picture changed (path is overwritten, and a numbered copy
 * path.N.ppm is kept for the first 16 distinct frames).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qemu/cutils.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/mtk_disp.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "hw/misc/mtk_iommu.h"
#include "ui/console.h"
#include <zlib.h>

OBJECT_DECLARE_SIMPLE_TYPE(MtkDispState, MTK_DISP)

#define MTK_DISP_MAX_OVL 6

struct MtkDispState {
    DeviceState parent_obj;

    QemuConsole *con;
    QEMUTimer *timer;
    uint32_t width, height;
    uint64_t ovl_base[MTK_DISP_MAX_OVL];
    uint64_t fb_fallback;
    uint32_t fb_pitch;          /* simplefb stride in bytes, 0 = auto   */
    uint32_t fb_fmt;            /* simplefb CLRFMT code, default RGBA8888 */
    uint64_t fb_right;          /* optional right-die buffer (dual pipe) */
    uint32_t fb_right_pitch;
    uint32_t pipe_split;        /* first OVL index of the right-half pipe */
    char *dump_path;
    uint32_t dump_ms;
    bool trace;

    unsigned n_xlate_fail;      /* IOMMU translation failures */
    unsigned n_xlate_ok;        /* IOMMU translated pages read */
    unsigned xlate_logged;
    uint32_t *canvas;           /* xRGB8888, width*height */
    uint8_t *line;
    uint32_t last_crc;
    unsigned dumps;
    char last_desc[160];
};

static uint32_t rd32(hwaddr a)
{
    return address_space_ldl_le(&address_space_memory, a,
                                MEMTXATTRS_UNSPECIFIED, NULL);
}

/*
 * Read scan-out data.  With MTK_IOMMU=1 the OVL layer address is an IOVA and
 * every 4 KiB page has to be translated through the disp_iommu page table;
 * without IOMMU translation the address is a physical address and is read
 * directly (the pre-IOMMU bring-up configuration).
 */
static bool mtk_disp_read(MtkDispState *s, uint64_t addr, void *buf,
                          size_t len, bool iommu)
{
    if (!iommu) {
        return address_space_read(&address_space_memory, addr,
                                  MEMTXATTRS_UNSPECIFIED, buf, len) == MEMTX_OK;
    }
    while (len) {
        hwaddr pa;
        size_t chunk = MIN(len, (size_t)(0x1000 - (addr & 0xfff)));

        if (!mtk_iommu_translate_or_identity(MTK_IOMMU_DEV_DISP, addr, &pa) ||
            address_space_read(&address_space_memory, pa,
                               MEMTXATTRS_UNSPECIFIED, buf, chunk) != MEMTX_OK) {
            s->n_xlate_fail++;
            return false;
        }
        s->n_xlate_ok++;
        buf = (uint8_t *)buf + chunk;
        addr += chunk;
        len -= chunk;
    }
    return true;
}

static uint32_t mtk_disp_px(const uint8_t *p, unsigned fmt, bool bswap,
                            bool rgbswap)
{
    uint32_t r, g, b, v;

    switch (fmt) {
    case 0: /* RGB565 */
        v = lduw_le_p(p);
        if (bswap) {
            v = bswap16(v);
        }
        r = ((v >> 11) & 0x1f) << 3;
        g = ((v >> 5) & 0x3f) << 2;
        b = (v & 0x1f) << 3;
        break;
    case 1: /* RGB888: memory B,G,R */
        b = p[bswap ? 2 : 0];
        g = p[1];
        r = p[bswap ? 0 : 2];
        break;
    case 2: /* RGBA8888 hw code: le32 = A<<24|R<<16|G<<8|B */
        v = ldl_le_p(p);
        if (bswap) {
            v = bswap32(v);
        }
        r = (v >> 16) & 0xff;
        g = (v >> 8) & 0xff;
        b = v & 0xff;
        break;
    case 3: /* ARGB8888 hw code: le32 = R<<24|G<<16|B<<8|A */
        v = ldl_le_p(p);
        if (bswap) {
            v = bswap32(v);
        }
        r = v >> 24;
        g = (v >> 16) & 0xff;
        b = (v >> 8) & 0xff;
        break;
    default: /* YUV: show luma only */
        r = g = b = p[(fmt == 4) ? 1 : 0];
        break;
    }
    if (rgbswap) {
        uint32_t t = r;
        r = b;
        b = t;
    }
    return (r << 16) | (g << 8) | b;
}

static unsigned mtk_disp_bpp(unsigned fmt)
{
    switch (fmt) {
    case 0: return 2;
    case 1: return 3;
    case 2: case 3: return 4;
    default: return 2;
    }
}

static void mtk_disp_blit(MtkDispState *s, uint64_t addr, uint32_t pitch,
                          unsigned fmt, bool bswap, bool rgbswap,
                          uint32_t x0, uint32_t y0, uint32_t w, uint32_t h,
                          bool iommu)
{
    unsigned bpp = mtk_disp_bpp(fmt);
    uint32_t y, x;

    if (x0 >= s->width || y0 >= s->height) {
        return;
    }
    w = MIN(w, s->width - x0);
    h = MIN(h, s->height - y0);
    for (y = 0; y < h; y++) {
        if (!mtk_disp_read(s, addr + (uint64_t)y * pitch, s->line,
                           (size_t)w * bpp, iommu)) {
            memset(s->line, 0, (size_t)w * bpp);
            if (s->xlate_logged < 8) {
                s->xlate_logged++;
                fprintf(stderr, "MTK-DISP: no IOMMU mapping for %#" PRIx64
                        " (layer %ux%u fmt %u)\n", addr, w, h, fmt);
            }
        }
        for (x = 0; x < w; x++) {
            s->canvas[(y0 + y) * s->width + x0 + x] =
                mtk_disp_px(s->line + x * bpp, fmt, bswap, rgbswap);
        }
    }
}

/*
 * Two scan-out sources, selectable through MTK_DISP_SRC:
 *
 *   simplefb  the flat framebuffer at "fb-fallback" (the bootloader's fb
 *             hand-off).  This is the simple path: it needs no OVL registers
 *             and no CMDQ, so it keeps showing whatever the guest (LK, early
 *             kernel, or anything else that writes that buffer) puts there.
 *   drm       the OVL layer registers, i.e. what the kernel's mediatek-drm
 *             programs through CMDQ.  This is the real display pipeline.
 *   auto      DRM when any OVL layer is enabled, simplefb otherwise
 *             (default, keeps every earlier run behaving as before).
 */
enum {
    MTK_DISP_SRC_AUTO,
    MTK_DISP_SRC_SIMPLEFB,
    MTK_DISP_SRC_DRM,
};

static int mtk_disp_src_mode(void)
{
    static int mode = -1;

    if (mode < 0) {
        const char *e = getenv("MTK_DISP_SRC");

        mode = MTK_DISP_SRC_AUTO;
        if (e && !g_strcmp0(e, "simplefb")) {
            mode = MTK_DISP_SRC_SIMPLEFB;
        } else if (e && !g_strcmp0(e, "drm")) {
            mode = MTK_DISP_SRC_DRM;
        }
    }
    return mode;
}

/* Compose one frame; returns false when nothing scannable was found. */
extern uint32_t mtk_gce_disp_epoch;   /* hw/misc/mtk_gce.c */

static bool mtk_disp_compose(MtkDispState *s, char *desc, size_t dlen)
{
    bool any = false;
    int o, n, pass;
    int mode = mtk_disp_src_mode();
    bool iommu = mtk_iommu_enabled();

    memset(s->canvas, 0, (size_t)s->width * s->height * 4);
    desc[0] = 0;
    if (mode == MTK_DISP_SRC_SIMPLEFB) {
        if (!s->fb_fallback) {
            return false;
        }
        mtk_disp_blit(s, s->fb_fallback,
                      s->fb_pitch ? s->fb_pitch : ROUND_UP(s->width, 32) * 4,
                      s->fb_fmt, false, false, 0, 0,
                      s->fb_right ? s->width / 2 : s->width, s->height, false);
        if (s->fb_right) {
            mtk_disp_blit(s, s->fb_right,
                          s->fb_right_pitch ? s->fb_right_pitch
                                            : ROUND_UP(s->width / 2, 32) * 4,
                          s->fb_fmt, false, false, s->width / 2, 0,
                          s->width - s->width / 2, s->height, false);
        }
        snprintf(desc, dlen, "simplefb fb=%#" PRIx64 " pitch=%u fmt=%u%s%#" PRIx64,
                 s->fb_fallback,
                 s->fb_pitch ? s->fb_pitch : ROUND_UP(s->width, 32) * 4,
                 s->fb_fmt, s->fb_right ? " right=" : "",
                 s->fb_right);
        return true;
    }
    if (iommu) {
        snprintf(desc + strlen(desc), dlen - strlen(desc),
                 " iommu_ok=%u iommu_fail=%u", s->n_xlate_ok, s->n_xlate_fail);
    }
    /*
     * Two passes:
     *   pass 0 - normal OVL layers (enabled + CON/SIZE/ADDR valid);
     *   pass 1 - ADDR-only fallback layers, drawn on top.  The kernel's
     *            recovery minui commit only programs ADDR (and PITCH) of
     *            OVL2 L1 (addr = 0xc0900870 / 0xc1400870 in the CMA pool)
     *            and leaves CON/SIZE/SRC_CON at zero; scanning that buffer
     *            out full-panel is what the hardware effectively does for
     *            the recovery UI (verified in round 1,298: the dumped PPM
     *            contains the Xiaomi HyperOS Recovery menu).
     *            MTK_DISP_NO_ADDR_FALLBACK=1 disables the fallback.
     *            MTK_DISP_ADDR_ONLY=1 forces it for any CMA-address layer
     *            (diagnostic mode used to identify the minui frame).
     */
    for (pass = 0; pass < 2; pass++) {
        for (o = 0; o < MTK_DISP_MAX_OVL; o++) {
            hwaddr base = s->ovl_base[o];
            uint32_t src_con, xbase = 0;

            if (!base) {
                continue;
            }
            /*
             * Dual-pipe (left/right split) panels: each pipe's OVL chain
             * renders its own ROI and the DSC/DSI merge places the right
             * pipe after the left pipe's ROI width (OVL0 ROI_SIZE[15:0]).
             */
            if (s->pipe_split && o >= s->pipe_split && s->ovl_base[0]) {
                xbase = rd32(s->ovl_base[0] + 0x20) & 0xffff;
            }
            src_con = rd32(base + 0x2c);
            for (n = 0; n < 4; n++) {
                hwaddr lb = base + 0x20 * n;
                uint32_t con, size, off, pitch, addr;
                bool enabled, normal, fb_addr, cma;

                addr = rd32(base + 0xf40 + 0x20 * n);
                con = rd32(lb + 0x30);
                size = rd32(lb + 0x38);
                off = rd32(lb + 0x3c);
                pitch = rd32(lb + 0x44) & 0xffff;
                enabled = (src_con & (1u << n)) != 0 ||
                          getenv("MTK_DISP_IGN_SRC_CON") != NULL;
                normal = enabled && addr != 0 && (size & 0xffff) != 0 &&
                         (size >> 16) != 0;
                if (iommu) {
                    /*
                     * The layer address is an IOVA: recovery's minui commits
                     * only ADDR/PITCH of OVL2 L1, so an address that maps
                     * through disp_iommu is the recovery framebuffer.
                     */
                    hwaddr pa;

                    cma = addr != 0 &&
                          mtk_iommu_translate_or_identity(MTK_IOMMU_DEV_DISP,
                                                          addr, &pa);
                } else {
                    cma = addr >= 0xc0000000u && addr < 0xd0000000u;
                }
                fb_addr = cma && !getenv("MTK_DISP_NO_ADDR_FALLBACK") &&
                          (getenv("MTK_DISP_ADDR_ONLY") != NULL || !normal);
                if (pass == 0) {
                    if (!normal || fb_addr) {
                        continue;
                    }
                    if (!pitch) {
                        pitch = (size & 0xffff) *
                                mtk_disp_bpp((con >> 12) & 0xf);
                    }
                    mtk_disp_blit(s, addr, pitch, (con >> 12) & 0xf,
                                  con & (1u << 24), con & (1u << 25),
                                  xbase + (off & 0xffff), off >> 16,
                                  size & 0xffff, size >> 16, true);
                    snprintf(desc + strlen(desc), dlen - strlen(desc),
                             "ovl%d.L%d(en=bit%d src_con=%#x) addr=%#x "
                             "con=%#x size=%ux%u off=%u,%u pitch=%u; ",
                             o, n, n, src_con, addr, con, size & 0xffff,
                             size >> 16, off & 0xffff, off >> 16, pitch);
                    any = true;
                } else {
                    uint32_t fp;

                    if (!fb_addr || normal) {
                        continue;
                    }
                    fp = pitch ? pitch : (uint32_t)s->width * 4;
                    /*
                     * Dual-pipe panels store the two 540-column halves
                     * swapped inside one framebuffer row (verified in round
                     * 1,298: swapping the halves turns the split recovery
                     * splash into a perfect "Xiaomi HyperOS Recovery"
                     * screen).  MTK_DISP_NO_FB_SWAP=1 restores the plain
                     * linear scan-out.
                     */
                    if (s->pipe_split && !getenv("MTK_DISP_NO_FB_SWAP")) {
                        uint32_t hw2 = (uint32_t)s->width / 2;

                        mtk_disp_blit(s, addr + hw2 * 4, fp, 2, false, false,
                                      0, 0, hw2, s->height, true);
                        mtk_disp_blit(s, addr, fp, 2, false, false,
                                      hw2, 0, s->width - hw2, s->height, true);
                    } else {
                        mtk_disp_blit(s, addr, fp, 2, false, false, 0, 0,
                                      s->width, s->height, true);
                    }
                    snprintf(desc + strlen(desc), dlen - strlen(desc),
                             "ovl%d.L%d(ADDR-FB) addr=%#x pitch=%u; ",
                             o, n, addr, fp);
                    any = true;
                }
            }
        }
    }
    if (!any && s->fb_fallback && mode != MTK_DISP_SRC_DRM) {
        mtk_disp_blit(s, s->fb_fallback, ROUND_UP(s->width, 32) * 4, 2,
                      false, false, 0, 0, s->width, s->height, false);
        snprintf(desc, dlen, "fallback fb=%#" PRIx64, s->fb_fallback);
        any = true;
    }
    return any;
}

/*
 * MTK_DISP_DBG=1: dump the raw OVL register state of every instance/layer so
 * the vendor register map can be matched against what the kernel actually
 * programmed (recovery minui writes its framebuffer through CMDQ; the layer
 * enable bits and the ADDR/PITCH/OFFSET registers have to be paired up).
 */
static void mtk_disp_dump_ovl(MtkDispState *s, const char *why)
{
    static int ndump;
    int o, n;

    if (!getenv("MTK_DISP_DBG") || ndump >= 400) {
        return;
    }
    ndump++;
    for (o = 0; o < MTK_DISP_MAX_OVL; o++) {
        hwaddr base = s->ovl_base[o];
        uint32_t src_con, dpcon, roi;

        if (!base) {
            continue;
        }
        src_con = rd32(base + 0x2c);
        dpcon = rd32(base + 0x24);
        roi = rd32(base + 0x20);
        fprintf(stderr,
                "MTK-DISP-DBG %s ovl%d base=%#" PRIx64 " roi=%#x dp=%#x "
                "src_con=%#x\n", why, o, (uint64_t)base, roi, dpcon, src_con);
        for (n = 0; n < 4; n++) {
            hwaddr lb = base + 0x20 * n;
            uint32_t addr = rd32(base + 0xf40 + 0x20 * n);
            char pabuf[48] = "";

            if (mtk_iommu_enabled() && addr) {
                hwaddr pa;

                if (mtk_iommu_translate_or_identity(MTK_IOMMU_DEV_DISP,
                                                     addr, &pa)) {
                    snprintf(pabuf, sizeof(pabuf), " pa=%#" HWADDR_PRIx, pa);
                } else {
                    snprintf(pabuf, sizeof(pabuf), " pa=UNMAPPED");
                }
            }
            fprintf(stderr,
                    "MTK-DISP-DBG   L%d con=%#x size=%#x off=%#x pitch=%#x "
                    "addr=%#x addr_ext=%#x%s\n",
                    n, rd32(lb + 0x30), rd32(lb + 0x38), rd32(lb + 0x3c),
                    rd32(lb + 0x44), addr,
                    rd32(base + 0xf48 + 0x20 * n), pabuf);
        }
    }
}

static void mtk_disp_to_surface(MtkDispState *s)
{
    DisplaySurface *surf = qemu_console_surface(s->con);
    uint32_t y;

    if (!surf || surface_width(surf) != s->width ||
        surface_height(surf) != s->height) {
        qemu_console_resize(s->con, s->width, s->height);
        surf = qemu_console_surface(s->con);
    }
    for (y = 0; y < s->height; y++) {
        memcpy((uint8_t *)surface_data(surf) + y * surface_stride(surf),
               s->canvas + y * s->width, s->width * 4);
    }
    qemu_console_update_full(s->con);
}

static bool mtk_disp_gfx_update(void *opaque)
{
    MtkDispState *s = opaque;
    char desc[512];

    mtk_disp_compose(s, desc, sizeof(desc));
    mtk_disp_to_surface(s);
    return true;
}

static void mtk_disp_invalidate(void *opaque)
{
}

static const GraphicHwOps mtk_disp_gfx_ops = {
    .invalidate = mtk_disp_invalidate,
    .gfx_update = mtk_disp_gfx_update,
};

static void mtk_disp_write_ppm(MtkDispState *s, const char *path)
{
    FILE *f = fopen(path, "wb");
    uint32_t i, n = s->width * s->height;
    uint8_t *row;

    if (!f) {
        warn_report("mtk-disp: cannot open %s", path);
        return;
    }
    fprintf(f, "P6\n%u %u\n255\n", s->width, s->height);
    row = g_malloc(s->width * 3);
    for (i = 0; i < n; i += s->width) {
        uint32_t x;

        for (x = 0; x < s->width; x++) {
            uint32_t p = s->canvas[i + x];

            row[x * 3] = p >> 16;
            row[x * 3 + 1] = p >> 8;
            row[x * 3 + 2] = p;
        }
        if (fwrite(row, 1, s->width * 3, f) != s->width * 3) {
            break;
        }
    }
    g_free(row);
    fclose(f);
}

static void mtk_disp_tick(void *opaque)
{
    MtkDispState *s = opaque;
    char desc[512];
    bool any = mtk_disp_compose(s, desc, sizeof(desc));

    if (s->trace && strncmp(desc, s->last_desc, sizeof(s->last_desc) - 1)) {
        fprintf(stderr, "MTK-DISP: %s\n", any ? desc : "no enabled OVL layer");
        pstrcpy(s->last_desc, sizeof(s->last_desc), desc);
    }
    {
        static uint32_t last_epoch;

        if (mtk_gce_disp_epoch != last_epoch) {
            last_epoch = mtk_gce_disp_epoch;
            mtk_disp_dump_ovl(s, "epoch");
        }
    }
    if (any) {
        uint32_t crc = crc32(0, (const Bytef *)s->canvas,
                             s->width * s->height * 4);
        bool nonzero = false;
        uint32_t i;

        for (i = 0; i < s->width * s->height; i++) {
            if (s->canvas[i]) {
                nonzero = true;
                break;
            }
        }
        if (crc != s->last_crc) {
            s->last_crc = crc;
            mtk_disp_dump_ovl(s, "frame");
            if (s->dump_path) {
                mtk_disp_write_ppm(s, s->dump_path);
                if (s->dumps < 16) {
                    g_autofree char *p = g_strdup_printf("%s.%u.ppm",
                                                        s->dump_path, s->dumps);
                    mtk_disp_write_ppm(s, p);
                }
                fprintf(stderr,
                        "MTK-DISP: frame %u t=%" PRId64 "ms crc=%08x %s -> %s (%s)\n",
                        s->dumps,
                        qemu_clock_get_ms(QEMU_CLOCK_REALTIME),
                        crc, nonzero ? "non-zero" : "all-black",
                        s->dump_path, desc);
                s->dumps++;
            }
        }
    }
    timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + s->dump_ms);
}

static void mtk_disp_realize(DeviceState *dev, Error **errp)
{
    MtkDispState *s = MTK_DISP(dev);

    if (!s->width || !s->height || s->width > 8192 || s->height > 8192) {
        error_setg(errp, "mtk-disp: bad size %ux%u", s->width, s->height);
        return;
    }
    s->canvas = g_new0(uint32_t, (size_t)s->width * s->height);
    s->line = g_malloc((size_t)s->width * 4);
    s->con = qemu_graphic_console_create(dev, 0, &mtk_disp_gfx_ops, s);
    qemu_console_resize(s->con, s->width, s->height);
    if (!s->dump_ms) {
        s->dump_ms = 1000;
    }
    if (s->dump_path || s->trace) {
        s->timer = timer_new_ms(QEMU_CLOCK_REALTIME, mtk_disp_tick, s);
        timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + s->dump_ms);
    }
}

static const Property mtk_disp_props[] = {
    DEFINE_PROP_UINT32("width", MtkDispState, width, 1080),
    DEFINE_PROP_UINT32("height", MtkDispState, height, 2460),
    DEFINE_PROP_UINT64("ovl0", MtkDispState, ovl_base[0], 0),
    DEFINE_PROP_UINT64("ovl1", MtkDispState, ovl_base[1], 0),
    DEFINE_PROP_UINT64("ovl2", MtkDispState, ovl_base[2], 0),
    DEFINE_PROP_UINT64("ovl3", MtkDispState, ovl_base[3], 0),
    DEFINE_PROP_UINT64("ovl4", MtkDispState, ovl_base[4], 0),
    DEFINE_PROP_UINT64("ovl5", MtkDispState, ovl_base[5], 0),
    DEFINE_PROP_UINT32("pipe-split", MtkDispState, pipe_split, 0),
    DEFINE_PROP_UINT64("fb-fallback", MtkDispState, fb_fallback, 0),
    DEFINE_PROP_UINT32("fb-pitch", MtkDispState, fb_pitch, 0),
    DEFINE_PROP_UINT32("fb-fmt", MtkDispState, fb_fmt, 2),
    DEFINE_PROP_UINT64("fb-right", MtkDispState, fb_right, 0),
    DEFINE_PROP_UINT32("fb-right-pitch", MtkDispState, fb_right_pitch, 0),
    DEFINE_PROP_STRING("dump-path", MtkDispState, dump_path),
    DEFINE_PROP_UINT32("dump-ms", MtkDispState, dump_ms, 1000),
    DEFINE_PROP_BOOL("trace", MtkDispState, trace, false),
};

static void mtk_disp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mtk_disp_realize;
    device_class_set_props(dc, mtk_disp_props);
    dc->desc = "MediaTek OVL/DSI scan-out (console + PPM dump)";
    dc->user_creatable = false;
}

static const TypeInfo mtk_disp_types[] = {
    {
        .name = TYPE_MTK_DISP,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(MtkDispState),
        .class_init = mtk_disp_class_init,
    },
};

DEFINE_TYPES(mtk_disp_types)
