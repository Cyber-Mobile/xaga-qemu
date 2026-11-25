/*
 * MT6895 ARM DynamIQ Shared Unit (DSU) PMU bare-metal system register test.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <glib/gstdio.h>
#include "qemu/bswap.h"
#include "libqtest.h"

/*
 * Assembled AArch64 Image payload (verified via llvm-mc + capstone):
 * - Enters at NS EL2 via arm_load_kernel, checks CLUSTERPMMDCR_EL1 (PL2_RW).
 * - Drops to NS EL1h via HCR_EL2.RW=1 / ERET and verifies CurrentEL == EL1.
 * - Exercises all 13 PL1 CLUSTERPM* registers: CLUSTERPMCR_EL1 (RO bits, E|X,
 *   P|C reset of counters), CLUSTERPMCEID0/1_EL1, W1S/W1C masks (CNTEN/OVS/
 *   INTEN), CLUSTERPMSELR_EL1 (valid idx 2 and out-of-range idx 6 RAZ/WI),
 *   CLUSTERPMXEVTYPER_EL1 (10-bit mask), CLUSTERPMXEVCNTR_EL1, and 64-bit
 *   CLUSTERPMCCNTR_EL1.
 * - Writes "OK\n" (or "F<tag>\n" on mismatch) to MT6895 UART0 (0x11001000).
 */
static const uint32_t dsu_pmu_image_words[] = {
    0x14000010, /* 0x000: b #0x40 */
    0x00000000, /* 0x004: hdr */
    0x00080000, /* 0x008: hdr (text_offset = 0x80000) */
    0x00000000, /* 0x00c: hdr */
    0x00010000, /* 0x010: hdr (image_size = 0x10000) */
    0x00000000, /* 0x014: hdr */
    0x00000000, /* 0x018: hdr */
    0x00000000, /* 0x01c: hdr */
    0x00000000, /* 0x020: hdr */
    0x00000000, /* 0x024: hdr */
    0x00000000, /* 0x028: hdr */
    0x00000000, /* 0x02c: hdr */
    0x00000000, /* 0x030: hdr */
    0x00000000, /* 0x034: hdr */
    0x644d5241, /* 0x038: hdr ("ARM\x64") */
    0x00000000, /* 0x03c: hdr */
    0xd282001c, /* 0x040: mov x28, #0x1000 */
    0xf2a2201c, /* 0x044: movk x28, #0x1100, lsl #16 */
    0xd5384240, /* 0x048: mrs x0, currentel */
    0xd2800101, /* 0x04c: mov x1, #8 */
    0xeb01001f, /* 0x050: cmp x0, x1 */
    0x54000060, /* 0x054: b.eq #0x60 */
    0x52800820, /* 0x058: mov w0, #0x41 */
    0x14000131, /* 0x05c: b #0x520 */
    0xd538f660, /* 0x060: mrs x0, s3_0_c15_c6_3 */
    0xd2800001, /* 0x064: mov x1, #0 */
    0xeb01001f, /* 0x068: cmp x0, x1 */
    0x54000060, /* 0x06c: b.eq #0x78 */
    0x52800840, /* 0x070: mov w0, #0x42 */
    0x1400012b, /* 0x074: b #0x520 */
    0xd2800aa0, /* 0x078: mov x0, #0x55 */
    0xd518f660, /* 0x07c: msr s3_0_c15_c6_3, x0 */
    0xd5033fdf, /* 0x080: isb  */
    0xd538f660, /* 0x084: mrs x0, s3_0_c15_c6_3 */
    0xd2800aa1, /* 0x088: mov x1, #0x55 */
    0xeb01001f, /* 0x08c: cmp x0, x1 */
    0x54000060, /* 0x090: b.eq #0x9c */
    0x52800860, /* 0x094: mov w0, #0x43 */
    0x14000122, /* 0x098: b #0x520 */
    0xd518f67f, /* 0x09c: msr s3_0_c15_c6_3, xzr */
    0xd5033fdf, /* 0x0a0: isb  */
    0xd538f660, /* 0x0a4: mrs x0, s3_0_c15_c6_3 */
    0xd2800001, /* 0x0a8: mov x1, #0 */
    0xeb01001f, /* 0x0ac: cmp x0, x1 */
    0x54000060, /* 0x0b0: b.eq #0xbc */
    0x52800880, /* 0x0b4: mov w0, #0x44 */
    0x1400011a, /* 0x0b8: b #0x520 */
    0xd53c1100, /* 0x0bc: mrs x0, hcr_el2 */
    0x9264f800, /* 0x0c0: and x0, x0, #0xfffffffff7ffffff */
    0xb2610000, /* 0x0c4: orr x0, x0, #0x80000000 */
    0xd51c1100, /* 0x0c8: msr hcr_el2, x0 */
    0xd28078a0, /* 0x0cc: mov x0, #0x3c5 */
    0xd51c4000, /* 0x0d0: msr spsr_el2, x0 */
    0x10000080, /* 0x0d4: adr x0, #0xe4 */
    0xd51c4020, /* 0x0d8: msr elr_el2, x0 */
    0xd5033fdf, /* 0x0dc: isb  */
    0xd69f03e0, /* 0x0e0: eret  */
    0xd5384240, /* 0x0e4: mrs x0, currentel */
    0xd2800081, /* 0x0e8: mov x1, #4 */
    0xeb01001f, /* 0x0ec: cmp x0, x1 */
    0x54000060, /* 0x0f0: b.eq #0xfc */
    0x528008a0, /* 0x0f4: mov w0, #0x45 */
    0x1400010a, /* 0x0f8: b #0x520 */
    0xd538f500, /* 0x0fc: mrs x0, s3_0_c15_c5_0 */
    0xd2860001, /* 0x100: mov x1, #0x3000 */
    0xf2a82001, /* 0x104: movk x1, #0x4100, lsl #16 */
    0xeb01001f, /* 0x108: cmp x0, x1 */
    0x54000060, /* 0x10c: b.eq #0x118 */
    0x528008c0, /* 0x110: mov w0, #0x46 */
    0x14000103, /* 0x114: b #0x520 */
    0xd29fffe0, /* 0x118: mov x0, #0xffff */
    0xf2bfffe0, /* 0x11c: movk x0, #0xffff, lsl #16 */
    0xd518f500, /* 0x120: msr s3_0_c15_c5_0, x0 */
    0xd5033fdf, /* 0x124: isb  */
    0xd538f500, /* 0x128: mrs x0, s3_0_c15_c5_0 */
    0xd2860221, /* 0x12c: mov x1, #0x3011 */
    0xf2a82001, /* 0x130: movk x1, #0x4100, lsl #16 */
    0xeb01001f, /* 0x134: cmp x0, x1 */
    0x54000060, /* 0x138: b.eq #0x144 */
    0x528008e0, /* 0x13c: mov w0, #0x47 */
    0x140000f8, /* 0x140: b #0x520 */
    0xd518f51f, /* 0x144: msr s3_0_c15_c5_0, xzr */
    0xd5033fdf, /* 0x148: isb  */
    0xd538f500, /* 0x14c: mrs x0, s3_0_c15_c5_0 */
    0xd2860001, /* 0x150: mov x1, #0x3000 */
    0xf2a82001, /* 0x154: movk x1, #0x4100, lsl #16 */
    0xeb01001f, /* 0x158: cmp x0, x1 */
    0x54000060, /* 0x15c: b.eq #0x168 */
    0x52800900, /* 0x160: mov w0, #0x48 */
    0x140000ef, /* 0x164: b #0x520 */
    0xd538f680, /* 0x168: mrs x0, s3_0_c15_c6_4 */
    0xd2800001, /* 0x16c: mov x1, #0 */
    0xf2a44441, /* 0x170: movk x1, #0x2222, lsl #16 */
    0xeb01001f, /* 0x174: cmp x0, x1 */
    0x54000060, /* 0x178: b.eq #0x184 */
    0x52800920, /* 0x17c: mov w0, #0x49 */
    0x140000e8, /* 0x180: b #0x520 */
    0xd538f6a0, /* 0x184: mrs x0, s3_0_c15_c6_5 */
    0xd281c001, /* 0x188: mov x1, #0xe00 */
    0xeb01001f, /* 0x18c: cmp x0, x1 */
    0x54000060, /* 0x190: b.eq #0x19c */
    0x52800940, /* 0x194: mov w0, #0x4a */
    0x140000e2, /* 0x198: b #0x520 */
    0xd29fffe0, /* 0x19c: mov x0, #0xffff */
    0xf2bfffe0, /* 0x1a0: movk x0, #0xffff, lsl #16 */
    0xd518f520, /* 0x1a4: msr s3_0_c15_c5_1, x0 */
    0xd5033fdf, /* 0x1a8: isb  */
    0xd538f520, /* 0x1ac: mrs x0, s3_0_c15_c5_1 */
    0xd28007e1, /* 0x1b0: mov x1, #0x3f */
    0xf2b00001, /* 0x1b4: movk x1, #0x8000, lsl #16 */
    0xeb01001f, /* 0x1b8: cmp x0, x1 */
    0x54000060, /* 0x1bc: b.eq #0x1c8 */
    0x52800960, /* 0x1c0: mov w0, #0x4b */
    0x140000d7, /* 0x1c4: b #0x520 */
    0xd538f540, /* 0x1c8: mrs x0, s3_0_c15_c5_2 */
    0xd28007e1, /* 0x1cc: mov x1, #0x3f */
    0xf2b00001, /* 0x1d0: movk x1, #0x8000, lsl #16 */
    0xeb01001f, /* 0x1d4: cmp x0, x1 */
    0x54000060, /* 0x1d8: b.eq #0x1e4 */
    0x52800980, /* 0x1dc: mov w0, #0x4c */
    0x140000d0, /* 0x1e0: b #0x520 */
    0xd28000a0, /* 0x1e4: mov x0, #5 */
    0xd518f540, /* 0x1e8: msr s3_0_c15_c5_2, x0 */
    0xd5033fdf, /* 0x1ec: isb  */
    0xd538f520, /* 0x1f0: mrs x0, s3_0_c15_c5_1 */
    0xd2800741, /* 0x1f4: mov x1, #0x3a */
    0xf2b00001, /* 0x1f8: movk x1, #0x8000, lsl #16 */
    0xeb01001f, /* 0x1fc: cmp x0, x1 */
    0x54000060, /* 0x200: b.eq #0x20c */
    0x528009a0, /* 0x204: mov w0, #0x4d */
    0x140000c6, /* 0x208: b #0x520 */
    0xd29fffe0, /* 0x20c: mov x0, #0xffff */
    0xf2bfffe0, /* 0x210: movk x0, #0xffff, lsl #16 */
    0xd518f540, /* 0x214: msr s3_0_c15_c5_2, x0 */
    0xd5033fdf, /* 0x218: isb  */
    0xd538f540, /* 0x21c: mrs x0, s3_0_c15_c5_2 */
    0xd2800001, /* 0x220: mov x1, #0 */
    0xeb01001f, /* 0x224: cmp x0, x1 */
    0x54000060, /* 0x228: b.eq #0x234 */
    0x528009c0, /* 0x22c: mov w0, #0x4e */
    0x140000bc, /* 0x230: b #0x520 */
    0xd29fffe0, /* 0x234: mov x0, #0xffff */
    0xf2bfffe0, /* 0x238: movk x0, #0xffff, lsl #16 */
    0xd518f560, /* 0x23c: msr s3_0_c15_c5_3, x0 */
    0xd5033fdf, /* 0x240: isb  */
    0xd538f560, /* 0x244: mrs x0, s3_0_c15_c5_3 */
    0xd28007e1, /* 0x248: mov x1, #0x3f */
    0xf2b00001, /* 0x24c: movk x1, #0x8000, lsl #16 */
    0xeb01001f, /* 0x250: cmp x0, x1 */
    0x54000060, /* 0x254: b.eq #0x260 */
    0x528009e0, /* 0x258: mov w0, #0x4f */
    0x140000b1, /* 0x25c: b #0x520 */
    0xd538f580, /* 0x260: mrs x0, s3_0_c15_c5_4 */
    0xd28007e1, /* 0x264: mov x1, #0x3f */
    0xf2b00001, /* 0x268: movk x1, #0x8000, lsl #16 */
    0xeb01001f, /* 0x26c: cmp x0, x1 */
    0x54000060, /* 0x270: b.eq #0x27c */
    0x52800a00, /* 0x274: mov w0, #0x50 */
    0x140000aa, /* 0x278: b #0x520 */
    0xd28000a0, /* 0x27c: mov x0, #5 */
    0xd518f580, /* 0x280: msr s3_0_c15_c5_4, x0 */
    0xd5033fdf, /* 0x284: isb  */
    0xd538f560, /* 0x288: mrs x0, s3_0_c15_c5_3 */
    0xd2800741, /* 0x28c: mov x1, #0x3a */
    0xf2b00001, /* 0x290: movk x1, #0x8000, lsl #16 */
    0xeb01001f, /* 0x294: cmp x0, x1 */
    0x54000060, /* 0x298: b.eq #0x2a4 */
    0x52800a20, /* 0x29c: mov w0, #0x51 */
    0x140000a0, /* 0x2a0: b #0x520 */
    0xd29fffe0, /* 0x2a4: mov x0, #0xffff */
    0xf2bfffe0, /* 0x2a8: movk x0, #0xffff, lsl #16 */
    0xd518f580, /* 0x2ac: msr s3_0_c15_c5_4, x0 */
    0xd5033fdf, /* 0x2b0: isb  */
    0xd538f580, /* 0x2b4: mrs x0, s3_0_c15_c5_4 */
    0xd2800001, /* 0x2b8: mov x1, #0 */
    0xeb01001f, /* 0x2bc: cmp x0, x1 */
    0x54000060, /* 0x2c0: b.eq #0x2cc */
    0x52800a40, /* 0x2c4: mov w0, #0x52 */
    0x14000096, /* 0x2c8: b #0x520 */
    0xd29fffe0, /* 0x2cc: mov x0, #0xffff */
    0xf2bfffe0, /* 0x2d0: movk x0, #0xffff, lsl #16 */
    0xd518f5c0, /* 0x2d4: msr s3_0_c15_c5_6, x0 */
    0xd5033fdf, /* 0x2d8: isb  */
    0xd538f5c0, /* 0x2dc: mrs x0, s3_0_c15_c5_6 */
    0xd28007e1, /* 0x2e0: mov x1, #0x3f */
    0xf2b00001, /* 0x2e4: movk x1, #0x8000, lsl #16 */
    0xeb01001f, /* 0x2e8: cmp x0, x1 */
    0x54000060, /* 0x2ec: b.eq #0x2f8 */
    0x52800a60, /* 0x2f0: mov w0, #0x53 */
    0x1400008b, /* 0x2f4: b #0x520 */
    0xd538f5e0, /* 0x2f8: mrs x0, s3_0_c15_c5_7 */
    0xd28007e1, /* 0x2fc: mov x1, #0x3f */
    0xf2b00001, /* 0x300: movk x1, #0x8000, lsl #16 */
    0xeb01001f, /* 0x304: cmp x0, x1 */
    0x54000060, /* 0x308: b.eq #0x314 */
    0x52800a80, /* 0x30c: mov w0, #0x54 */
    0x14000084, /* 0x310: b #0x520 */
    0xd28000a0, /* 0x314: mov x0, #5 */
    0xd518f5e0, /* 0x318: msr s3_0_c15_c5_7, x0 */
    0xd5033fdf, /* 0x31c: isb  */
    0xd538f5c0, /* 0x320: mrs x0, s3_0_c15_c5_6 */
    0xd2800741, /* 0x324: mov x1, #0x3a */
    0xf2b00001, /* 0x328: movk x1, #0x8000, lsl #16 */
    0xeb01001f, /* 0x32c: cmp x0, x1 */
    0x54000060, /* 0x330: b.eq #0x33c */
    0x52800aa0, /* 0x334: mov w0, #0x55 */
    0x1400007a, /* 0x338: b #0x520 */
    0xd29fffe0, /* 0x33c: mov x0, #0xffff */
    0xf2bfffe0, /* 0x340: movk x0, #0xffff, lsl #16 */
    0xd518f5e0, /* 0x344: msr s3_0_c15_c5_7, x0 */
    0xd5033fdf, /* 0x348: isb  */
    0xd538f5e0, /* 0x34c: mrs x0, s3_0_c15_c5_7 */
    0xd2800001, /* 0x350: mov x1, #0 */
    0xeb01001f, /* 0x354: cmp x0, x1 */
    0x54000060, /* 0x358: b.eq #0x364 */
    0x52800ac0, /* 0x35c: mov w0, #0x56 */
    0x14000070, /* 0x360: b #0x520 */
    0xd2800040, /* 0x364: mov x0, #2 */
    0xd518f5a0, /* 0x368: msr s3_0_c15_c5_5, x0 */
    0xd5033fdf, /* 0x36c: isb  */
    0xd538f5a0, /* 0x370: mrs x0, s3_0_c15_c5_5 */
    0xd2800041, /* 0x374: mov x1, #2 */
    0xeb01001f, /* 0x378: cmp x0, x1 */
    0x54000060, /* 0x37c: b.eq #0x388 */
    0x52800ae0, /* 0x380: mov w0, #0x57 */
    0x14000067, /* 0x384: b #0x520 */
    0xd297dde0, /* 0x388: mov x0, #0xbeef */
    0xf2bbd5a0, /* 0x38c: movk x0, #0xdead, lsl #16 */
    0xd518f620, /* 0x390: msr s3_0_c15_c6_1, x0 */
    0xd5033fdf, /* 0x394: isb  */
    0xd538f620, /* 0x398: mrs x0, s3_0_c15_c6_1 */
    0xd2805de1, /* 0x39c: mov x1, #0x2ef */
    0xeb01001f, /* 0x3a0: cmp x0, x1 */
    0x54000060, /* 0x3a4: b.eq #0x3b0 */
    0x52800b00, /* 0x3a8: mov w0, #0x58 */
    0x1400005d, /* 0x3ac: b #0x520 */
    0xd28acf00, /* 0x3b0: mov x0, #0x5678 */
    0xf2a24680, /* 0x3b4: movk x0, #0x1234, lsl #16 */
    0xd518f640, /* 0x3b8: msr s3_0_c15_c6_2, x0 */
    0xd5033fdf, /* 0x3bc: isb  */
    0xd538f640, /* 0x3c0: mrs x0, s3_0_c15_c6_2 */
    0xd28acf01, /* 0x3c4: mov x1, #0x5678 */
    0xf2a24681, /* 0x3c8: movk x1, #0x1234, lsl #16 */
    0xeb01001f, /* 0x3cc: cmp x0, x1 */
    0x54000060, /* 0x3d0: b.eq #0x3dc */
    0x52800b20, /* 0x3d4: mov w0, #0x59 */
    0x14000052, /* 0x3d8: b #0x520 */
    0xd28000c0, /* 0x3dc: mov x0, #6 */
    0xd518f5a0, /* 0x3e0: msr s3_0_c15_c5_5, x0 */
    0xd5033fdf, /* 0x3e4: isb  */
    0xd538f5a0, /* 0x3e8: mrs x0, s3_0_c15_c5_5 */
    0xd28000c1, /* 0x3ec: mov x1, #6 */
    0xeb01001f, /* 0x3f0: cmp x0, x1 */
    0x54000060, /* 0x3f4: b.eq #0x400 */
    0x52800b40, /* 0x3f8: mov w0, #0x5a */
    0x14000049, /* 0x3fc: b #0x520 */
    0xd29de020, /* 0x400: mov x0, #0xef01 */
    0xf2b579a0, /* 0x404: movk x0, #0xabcd, lsl #16 */
    0xd518f620, /* 0x408: msr s3_0_c15_c6_1, x0 */
    0xd518f640, /* 0x40c: msr s3_0_c15_c6_2, x0 */
    0xd5033fdf, /* 0x410: isb  */
    0xd538f620, /* 0x414: mrs x0, s3_0_c15_c6_1 */
    0xd2800001, /* 0x418: mov x1, #0 */
    0xeb01001f, /* 0x41c: cmp x0, x1 */
    0x54000060, /* 0x420: b.eq #0x42c */
    0x52800600, /* 0x424: mov w0, #0x30 */
    0x1400003e, /* 0x428: b #0x520 */
    0xd538f640, /* 0x42c: mrs x0, s3_0_c15_c6_2 */
    0xd2800001, /* 0x430: mov x1, #0 */
    0xeb01001f, /* 0x434: cmp x0, x1 */
    0x54000060, /* 0x438: b.eq #0x444 */
    0x52800620, /* 0x43c: mov w0, #0x31 */
    0x14000038, /* 0x440: b #0x520 */
    0xd2800040, /* 0x444: mov x0, #2 */
    0xd518f5a0, /* 0x448: msr s3_0_c15_c5_5, x0 */
    0xd5033fdf, /* 0x44c: isb  */
    0xd538f640, /* 0x450: mrs x0, s3_0_c15_c6_2 */
    0xd28acf01, /* 0x454: mov x1, #0x5678 */
    0xf2a24681, /* 0x458: movk x1, #0x1234, lsl #16 */
    0xeb01001f, /* 0x45c: cmp x0, x1 */
    0x54000060, /* 0x460: b.eq #0x46c */
    0x52800640, /* 0x464: mov w0, #0x32 */
    0x1400002e, /* 0x468: b #0x520 */
    0xd299bde0, /* 0x46c: mov x0, #0xcdef */
    0xf2b13560, /* 0x470: movk x0, #0x89ab, lsl #16 */
    0xf2c8ace0, /* 0x474: movk x0, #0x4567, lsl #32 */
    0xf2e02460, /* 0x478: movk x0, #0x123, lsl #48 */
    0xd518f600, /* 0x47c: msr s3_0_c15_c6_0, x0 */
    0xd5033fdf, /* 0x480: isb  */
    0xd538f600, /* 0x484: mrs x0, s3_0_c15_c6_0 */
    0xd299bde1, /* 0x488: mov x1, #0xcdef */
    0xf2b13561, /* 0x48c: movk x1, #0x89ab, lsl #16 */
    0xf2c8ace1, /* 0x490: movk x1, #0x4567, lsl #32 */
    0xf2e02461, /* 0x494: movk x1, #0x123, lsl #48 */
    0xeb01001f, /* 0x498: cmp x0, x1 */
    0x54000060, /* 0x49c: b.eq #0x4a8 */
    0x52800660, /* 0x4a0: mov w0, #0x33 */
    0x1400001f, /* 0x4a4: b #0x520 */
    0xd28000c0, /* 0x4a8: mov x0, #6 */
    0xd518f500, /* 0x4ac: msr s3_0_c15_c5_0, x0 */
    0xd5033fdf, /* 0x4b0: isb  */
    0xd538f640, /* 0x4b4: mrs x0, s3_0_c15_c6_2 */
    0xd2800001, /* 0x4b8: mov x1, #0 */
    0xeb01001f, /* 0x4bc: cmp x0, x1 */
    0x54000060, /* 0x4c0: b.eq #0x4cc */
    0x52800680, /* 0x4c4: mov w0, #0x34 */
    0x14000016, /* 0x4c8: b #0x520 */
    0xd538f600, /* 0x4cc: mrs x0, s3_0_c15_c6_0 */
    0xd2800001, /* 0x4d0: mov x1, #0 */
    0xeb01001f, /* 0x4d4: cmp x0, x1 */
    0x54000060, /* 0x4d8: b.eq #0x4e4 */
    0x528006a0, /* 0x4dc: mov w0, #0x35 */
    0x14000010, /* 0x4e0: b #0x520 */
    0xd538f500, /* 0x4e4: mrs x0, s3_0_c15_c5_0 */
    0xd2860001, /* 0x4e8: mov x1, #0x3000 */
    0xf2a82001, /* 0x4ec: movk x1, #0x4100, lsl #16 */
    0xeb01001f, /* 0x4f0: cmp x0, x1 */
    0x54000060, /* 0x4f4: b.eq #0x500 */
    0x528006c0, /* 0x4f8: mov w0, #0x36 */
    0x14000009, /* 0x4fc: b #0x520 */
    0x528009e1, /* 0x500: mov w1, #0x4f */
    0x39000381, /* 0x504: strb w1, [x28] */
    0x52800961, /* 0x508: mov w1, #0x4b */
    0x39000381, /* 0x50c: strb w1, [x28] */
    0x52800141, /* 0x510: mov w1, #0xa */
    0x39000381, /* 0x514: strb w1, [x28] */
    0xd503207f, /* 0x518: wfi  */
    0x17ffffff, /* 0x51c: b #0x518 */
    0x528008c1, /* 0x520: mov w1, #0x46 */
    0x39000381, /* 0x524: strb w1, [x28] */
    0x39000380, /* 0x528: strb w0, [x28] */
    0x52800141, /* 0x52c: mov w1, #0xa */
    0x39000381, /* 0x530: strb w1, [x28] */
    0x17fffff9, /* 0x534: b #0x518 */
};

static void write_full(int fd, const void *buf, size_t len)
{
    const uint8_t *p = buf;
    size_t off = 0;

    while (off < len) {
        ssize_t n = write(fd, p + off, len - off);

        g_assert(n > 0);
        off += (size_t)n;
    }
}

static bool read_serial_line(QTestState *qts, int fd, char *buf, size_t cap)
{
    time_t start = time(NULL);
    size_t pos = 0;

    while (pos + 1 < cap) {
        char ch;
        ssize_t n = read(fd, &ch, 1);

        if (n == 1) {
            if (ch == '\r') {
                continue;
            }
            buf[pos++] = ch;
            buf[pos] = '\0';
            if (ch == '\n') {
                return true;
            }
            continue;
        }
        g_assert(n == 0);
        if (!qtest_probe_child(qts)) {
            break;
        }
        if (time(NULL) - start >= 30) {
            break;
        }
        g_usleep(10000);
    }

    return false;
}

static void test_dsu_pmu(void)
{
    uint32_t img[ARRAY_SIZE(dsu_pmu_image_words)];
    g_autofree char *serialtmp = NULL;
    g_autofree char *kerntmp = NULL;
    QTestState *qts;
    char line[64] = { 0 };
    int ser_fd, kern_fd;
    size_t i;
    bool got_line;

    if (!qtest_has_accel("tcg")) {
        g_test_skip("TCG accelerator not available");
        return;
    }
    if (!qtest_has_machine("mt6895")) {
        g_test_skip("mt6895 machine not available");
        return;
    }

    for (i = 0; i < ARRAY_SIZE(dsu_pmu_image_words); i++) {
        img[i] = cpu_to_le32(dsu_pmu_image_words[i]);
    }

    ser_fd = g_file_open_tmp("qtest-mtk-dsu-pmu-sXXXXXX", &serialtmp, NULL);
    g_assert_cmpint(ser_fd, !=, -1);
    close(ser_fd);

    kern_fd = g_file_open_tmp("qtest-mtk-dsu-pmu-kXXXXXX", &kerntmp, NULL);
    g_assert_cmpint(kern_fd, !=, -1);
    write_full(kern_fd, img, sizeof(img));
    close(kern_fd);

    qts = qtest_initf("-machine mt6895 -m 256M -smp 1 -accel tcg -no-shutdown "
                      "-kernel %s -chardev file,id=serial0,path=%s "
                      "-serial chardev:serial0",
                      kerntmp, serialtmp);

    ser_fd = open(serialtmp, O_RDONLY);
    g_assert_cmpint(ser_fd, !=, -1);
    got_line = read_serial_line(qts, ser_fd, line, sizeof(line));
    close(ser_fd);

    qtest_quit(qts);
    g_unlink(serialtmp);
    g_unlink(kerntmp);

    g_assert_true(got_line);
    g_assert_cmpstr(line, ==, "OK\n");
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/mt6895/dsu-pmu/sysregs", test_dsu_pmu);
    return g_test_run();
}
