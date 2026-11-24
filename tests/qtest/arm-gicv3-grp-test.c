/*
 * ARM GICv3 per-group HPPI shadowing QTest.
 *
 * Verifies that pending interrupts in a group disabled at the CPU interface
 * (ICC_IGRPEN* == 0) never shadow pending interrupts in an enabled group,
 * regardless of relative priority or INTID ordering, and remain preserved
 * until their group is re-enabled.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <glib/gstdio.h>
#include "qemu/bswap.h"
#include "libqtest.h"

/*
 * Bare-metal AArch64 EL3 Secure ROM executed on -M virt,secure=on,gic-version=3.
 * Tests 7 cases via GICD (0x08000000), GICR (0x080a0000 / 0x080b0000) and
 * PL011 UART0 (0x09000000), emitting "OK\n" or "FAIL<code>\n".
 */
static const uint32_t rom_main[] = {
    /* 000 */ 0xd50041bf, /* msr SPSel, #0x1 */
    /* 004 */ 0xd5034fdf, /* msr DAIFSet, #0xf */
    /* 008 */ 0xd5033fdf, /* isb */
    /* 00c */ 0xd2810000, /* mov x0, #0x800 (VBAR_EL3 aligned vec_base) */
    /* 010 */ 0xd51ec000, /* msr VBAR_EL3, x0 */
    /* 014 */ 0xd28080c0, /* mov x0, #0x406 (SCR_EL3: RW | FIQ | IRQ, NS=0) */
    /* 018 */ 0xd51e1100, /* msr SCR_EL3, x0 */
    /* 01c */ 0xd5033fdf, /* isb */
    /* 020 */ 0xd2a1201c, /* mov x28, #0x9000000 (PL011 UART0) */
    /* 024 */ 0x52802020, /* mov w0, #0x101 (UARTEN | TXE) */
    /* 028 */ 0x79006380, /* strh w0, [x28, #0x30] */
    /* 02c */ 0xd2a1001b, /* mov x27, #0x8000000 (GICD) */
    /* 030 */ 0xd2a1015a, /* mov x26, #0x80a0000 (GICR RD_base) */
    /* 034 */ 0xd2a10179, /* mov x25, #0x80b0000 (GICR SGI_base) */
    /* 038 */ 0xb9401740, /* ldr w0, [x26, #0x14] (GICR_WAKER) */
    /* 03c */ 0x121e7800, /* and w0, w0, #0xfffffffd (clear ProcessorSleep) */
    /* 040 */ 0xb9001740, /* str w0, [x26, #0x14] */
    /* 044 */ 0xb9401740, /* ldr w0, [x26, #0x14] */
    /* 048 */ 0x3717ffe0, /* tbnz w0, #0x2, 0x44 (wait ChildrenAsleep == 0) */
    /* 04c */ 0x528006e0, /* mov w0, #0x37 (GICD_CTLR enables + ARE_S|ARE_NS) */
    /* 050 */ 0xb9000360, /* str w0, [x27] */
    /* 054 */ 0xd5033f9f, /* dsb sy */
    /* 058 */ 0xd28001e0, /* mov x0, #0xf */
    /* 05c */ 0xd51ecca0, /* msr ICC_SRE_EL3, x0 */
    /* 060 */ 0xd5033fdf, /* isb */
    /* 064 */ 0xd2800020, /* mov x0, #0x1 */
    /* 068 */ 0xd518cca0, /* msr ICC_SRE_EL1, x0 */
    /* 06c */ 0xd5033fdf, /* isb */
    /* 070 */ 0xd2801e00, /* mov x0, #0xf0 */
    /* 074 */ 0xd5184600, /* msr ICC_PMR_EL1, x0 */
    /* 078 */ 0xd518ccdf, /* msr ICC_IGRPEN0_EL1, xzr */
    /* 07c */ 0xd2800040, /* mov x0, #0x2 (G1S=1, G1NS=0) */
    /* 080 */ 0xd51ecce0, /* msr ICC_IGRPEN1_EL3, x0 */
    /* 084 */ 0xd5033fdf, /* isb */
    /* 088 */ 0x52800020, /* mov w0, #0x1 */
    /* 08c */ 0xb9008320, /* str w0, [x25, #0x80] (GICR_IGROUPR0: SGI0=NS) */
    /* 090 */ 0x52a40000, /* mov w0, #0x20000000 */
    /* 094 */ 0xb90d0320, /* str w0, [x25, #0xd00] (GICR_IGRPMODR0: PPI29=G1S) */
    /* 098 */ 0x52801a00, /* mov w0, #0xd0 */
    /* 09c */ 0xb9040320, /* str w0, [x25, #0x400] (SGI0 prio 0xd0) */
    /* 0a0 */ 0x529a0000, /* mov w0, #0xd000 */
    /* 0a4 */ 0xb9041f20, /* str w0, [x25, #0x41c] (PPI29 prio 0xd0) */
    /* 0a8 */ 0x52800038, /* mov w24, #0x1 */
    /* 0ac */ 0x32030318, /* orr w24, w24, #0x20000000 (bit0 | bit29) */
    /* 0b0 */ 0xb9010338, /* str w24, [x25, #0x100] (GICR_ISENABLER0) */
    /* 0b4 */ 0xd5033f9f, /* dsb sy */
    /* 0b8 */ 0xb9020338, /* str w24, [x25, #0x200] (Test 1: pend SGI0+PPI29) */
    /* 0bc */ 0xd5033f9f, /* dsb sy */
    /* 0c0 */ 0xd5033fdf, /* isb */
    /* 0c4 */ 0xd538cc41, /* mrs x1, ICC_HPPIR1_EL1 */
    /* 0c8 */ 0xf100743f, /* cmp x1, #0x1d (expect PPI 29) */
    /* 0cc */ 0x52800622, /* mov w2, #'1' */
    /* 0d0 */ 0x54000ec1, /* b.ne 0x2a8 <fail> */
    /* 0d4 */ 0xd538c841, /* mrs x1, ICC_HPPIR0_EL1 */
    /* 0d8 */ 0xf10ff03f, /* cmp x1, #0x3fc (expect INTID_SECURE 1020) */
    /* 0dc */ 0x52800822, /* mov w2, #'A' */
    /* 0e0 */ 0x54000e41, /* b.ne 0x2a8 <fail> */
    /* 0e4 */ 0x52801200, /* mov w0, #0x90 (Test 2: SGI0 higher prio 0x90) */
    /* 0e8 */ 0xb9040320, /* str w0, [x25, #0x400] */
    /* 0ec */ 0xd5033f9f, /* dsb sy */
    /* 0f0 */ 0xd5033fdf, /* isb */
    /* 0f4 */ 0xd538cc41, /* mrs x1, ICC_HPPIR1_EL1 */
    /* 0f8 */ 0xf100743f, /* cmp x1, #0x1d (still PPI 29 while G1NS off) */
    /* 0fc */ 0x52800642, /* mov w2, #'2' */
    /* 100 */ 0x54000d41, /* b.ne 0x2a8 <fail> */
    /* 104 */ 0xd2800060, /* mov x0, #0x3 (Test 3: enable G1NS as well) */
    /* 108 */ 0xd51ecce0, /* msr ICC_IGRPEN1_EL3, x0 */
    /* 10c */ 0xd5033fdf, /* isb */
    /* 110 */ 0xd538cc41, /* mrs x1, ICC_HPPIR1_EL1 */
    /* 114 */ 0xf100003f, /* cmp x1, #0x0 (now SGI 0 wins) */
    /* 118 */ 0x52800662, /* mov w2, #'3' */
    /* 11c */ 0x54000c61, /* b.ne 0x2a8 <fail> */
    /* 120 */ 0xd2800040, /* mov x0, #0x2 (Test 4: disable G1NS, ack PPI 29) */
    /* 124 */ 0xd51ecce0, /* msr ICC_IGRPEN1_EL3, x0 */
    /* 128 */ 0xd5033fdf, /* isb */
    /* 12c */ 0xd538cc01, /* mrs x1, ICC_IAR1_EL1 */
    /* 130 */ 0xf100743f, /* cmp x1, #0x1d */
    /* 134 */ 0x52800682, /* mov w2, #'4' */
    /* 138 */ 0x54000b81, /* b.ne 0x2a8 <fail> */
    /* 13c */ 0xd518cc21, /* msr ICC_EOIR1_EL1, x1 */
    /* 140 */ 0xd5033fdf, /* isb */
    /* 144 */ 0xd538cc41, /* mrs x1, ICC_HPPIR1_EL1 */
    /* 148 */ 0xf10ffc3f, /* cmp x1, #0x3ff (INTID_SPURIOUS 1023) */
    /* 14c */ 0x52800842, /* mov w2, #'B' */
    /* 150 */ 0x54000ac1, /* b.ne 0x2a8 <fail> */
    /* 154 */ 0xd2800060, /* mov x0, #0x3 (Test 5: re-enable G1NS, ack SGI 0) */
    /* 158 */ 0xd51ecce0, /* msr ICC_IGRPEN1_EL3, x0 */
    /* 15c */ 0xd5033fdf, /* isb */
    /* 160 */ 0xd538cc01, /* mrs x1, ICC_IAR1_EL1 */
    /* 164 */ 0xf100003f, /* cmp x1, #0x0 */
    /* 168 */ 0x528006a2, /* mov w2, #'5' */
    /* 16c */ 0x540009e1, /* b.ne 0x2a8 <fail> */
    /* 170 */ 0xd518cc21, /* msr ICC_EOIR1_EL1, x1 */
    /* 174 */ 0xd5033fdf, /* isb */
    /* 178 */ 0xd538cc41, /* mrs x1, ICC_HPPIR1_EL1 */
    /* 17c */ 0xf10ffc3f, /* cmp x1, #0x3ff */
    /* 180 */ 0x52800862, /* mov w2, #'C' */
    /* 184 */ 0x54000921, /* b.ne 0x2a8 <fail> */
    /* 188 */ 0xb900877f, /* str wzr, [x27, #0x84] (Test 6: SPI 32 G1S) */
    /* 18c */ 0x52800020, /* mov w0, #0x1 */
    /* 190 */ 0xb90d0760, /* str w0, [x27, #0xd04] (GICD_IGRPMODR1 bit0=1) */
    /* 194 */ 0x52801a00, /* mov w0, #0xd0 */
    /* 198 */ 0xb9042360, /* str w0, [x27, #0x420] (SPI 32 prio 0xd0) */
    /* 19c */ 0xd28c2000, /* mov x0, #0x6100 */
    /* 1a0 */ 0xf8206b7f, /* str xzr, [x27, x0] (GICD_IROUTER32 = CPU 0) */
    /* 1a4 */ 0x52800020, /* mov w0, #0x1 */
    /* 1a8 */ 0xb9010760, /* str w0, [x27, #0x104] (GICD_ISENABLER1) */
    /* 1ac */ 0xd2800040, /* mov x0, #0x2 */
    /* 1b0 */ 0xd51ecce0, /* msr ICC_IGRPEN1_EL3, x0 */
    /* 1b4 */ 0xd5033fdf, /* isb */
    /* 1b8 */ 0x52800020, /* mov w0, #0x1 */
    /* 1bc */ 0xb9020320, /* str w0, [x25, #0x200] (pend SGI 0 0x90) */
    /* 1c0 */ 0xb9020760, /* str w0, [x27, #0x204] (pend SPI 32 0xd0) */
    /* 1c4 */ 0xd5033f9f, /* dsb sy */
    /* 1c8 */ 0xd5033fdf, /* isb */
    /* 1cc */ 0xd538cc41, /* mrs x1, ICC_HPPIR1_EL1 */
    /* 1d0 */ 0xf100803f, /* cmp x1, #0x20 (expect SPI 32) */
    /* 1d4 */ 0x528006c2, /* mov w2, #'6' */
    /* 1d8 */ 0x54000681, /* b.ne 0x2a8 <fail> */
    /* 1dc */ 0xd538cc01, /* mrs x1, ICC_IAR1_EL1 */
    /* 1e0 */ 0xf100803f, /* cmp x1, #0x20 */
    /* 1e4 */ 0x52800882, /* mov w2, #'D' */
    /* 1e8 */ 0x54000601, /* b.ne 0x2a8 <fail> */
    /* 1ec */ 0xd518cc21, /* msr ICC_EOIR1_EL1, x1 */
    /* 1f0 */ 0xd5033fdf, /* isb */
    /* 1f4 */ 0x52800020, /* mov w0, #0x1 */
    /* 1f8 */ 0xb9028760, /* str w0, [x27, #0x284] (GICD_ICPENDR1) */
    /* 1fc */ 0xd2800060, /* mov x0, #0x3 */
    /* 200 */ 0xd51ecce0, /* msr ICC_IGRPEN1_EL3, x0 */
    /* 204 */ 0xd5033fdf, /* isb */
    /* 208 */ 0xd538cc01, /* mrs x1, ICC_IAR1_EL1 */
    /* 20c */ 0xf100003f, /* cmp x1, #0x0 */
    /* 210 */ 0x528008a2, /* mov w2, #'E' */
    /* 214 */ 0x540004a1, /* b.ne 0x2a8 <fail> */
    /* 218 */ 0xd518cc21, /* msr ICC_EOIR1_EL1, x1 */
    /* 21c */ 0xd5033fdf, /* isb */
    /* 220 */ 0xd2800013, /* mov x19, #0x0 (Test 7: async FIQ at EL3) */
    /* 224 */ 0xd2800040, /* mov x0, #0x2 */
    /* 228 */ 0xd51ecce0, /* msr ICC_IGRPEN1_EL3, x0 */
    /* 22c */ 0xd5033fdf, /* isb */
    /* 230 */ 0xb9020338, /* str w24, [x25, #0x200] (pend SGI0 + PPI29) */
    /* 234 */ 0xd5033f9f, /* dsb sy */
    /* 238 */ 0xd50341ff, /* msr DAIFClr, #0x1 (unmask FIQ) */
    /* 23c */ 0xd5033fdf, /* isb */
    /* 240 */ 0x52820014, /* mov w20, #0x1000 */
    /* 244 */ 0xb5000073, /* cbnz x19, 0x250 */
    /* 248 */ 0x71000694, /* subs w20, w20, #0x1 */
    /* 24c */ 0x54ffffc1, /* b.ne 0x244 */
    /* 250 */ 0xd50341df, /* msr DAIFSet, #0x1 */
    /* 254 */ 0xd5033fdf, /* isb */
    /* 258 */ 0xf100767f, /* cmp x19, #0x1d (FIQ handler acked PPI 29) */
    /* 25c */ 0x528006e2, /* mov w2, #'7' */
    /* 260 */ 0x54000241, /* b.ne 0x2a8 <fail> */
    /* 264 */ 0xd2800060, /* mov x0, #0x3 */
    /* 268 */ 0xd51ecce0, /* msr ICC_IGRPEN1_EL3, x0 */
    /* 26c */ 0xd5033fdf, /* isb */
    /* 270 */ 0xd538cc01, /* mrs x1, ICC_IAR1_EL1 */
    /* 274 */ 0xf100003f, /* cmp x1, #0x0 */
    /* 278 */ 0x528008c2, /* mov w2, #'F' */
    /* 27c */ 0x54000161, /* b.ne 0x2a8 <fail> */
    /* 280 */ 0xd518cc21, /* msr ICC_EOIR1_EL1, x1 */
    /* 284 */ 0xd5033fdf, /* isb */
    /* 288 */ 0x528009e0, /* mov w0, #'O' */
    /* 28c */ 0x39000380, /* strb w0, [x28] */
    /* 290 */ 0x52800960, /* mov w0, #'K' */
    /* 294 */ 0x39000380, /* strb w0, [x28] */
    /* 298 */ 0x52800140, /* mov w0, #'\n' */
    /* 29c */ 0x39000380, /* strb w0, [x28] */
    /* 2a0 */ 0xd503207f, /* halt: wfi */
    /* 2a4 */ 0x17ffffff, /* b halt */
    /* 2a8 */ 0x528008c0, /* fail: mov w0, #'F' */
    /* 2ac */ 0x39000380, /* strb w0, [x28] */
    /* 2b0 */ 0x52800820, /* mov w0, #'A' */
    /* 2b4 */ 0x39000380, /* strb w0, [x28] */
    /* 2b8 */ 0x52800920, /* mov w0, #'I' */
    /* 2bc */ 0x39000380, /* strb w0, [x28] */
    /* 2c0 */ 0x52800980, /* mov w0, #'L' */
    /* 2c4 */ 0x39000380, /* strb w0, [x28] */
    /* 2c8 */ 0x39000382, /* strb w2, [x28] */
    /* 2cc */ 0x52800140, /* mov w0, #'\n' */
    /* 2d0 */ 0x39000380, /* strb w0, [x28] */
    /* 2d4 */ 0x17fffff3, /* b halt */
};

static const struct {
    uint32_t offset;
    uint32_t insn;
} rom_vectors[] = {
    /* Current EL SP0 vectors (+0x000 .. +0x180 from VBAR_EL3 = 0x800) */
    { 0x800, 0x52800ac2 }, /* mov w2, #'V' */
    { 0x804, 0x17fffea9 }, /* b 0x2a8 <fail> */
    { 0x880, 0x52800922 }, /* mov w2, #'I' */
    { 0x884, 0x17fffe89 }, /* b 0x2a8 <fail> */
    { 0x900, 0xd538cc13 }, /* mrs x19, ICC_IAR1_EL1 */
    { 0x904, 0xd518cc33 }, /* msr ICC_EOIR1_EL1, x19 */
    { 0x908, 0xd5033fdf }, /* isb */
    { 0x90c, 0xd69f03e0 }, /* eret */
    { 0x980, 0x52800a62 }, /* mov w2, #'S' */
    { 0x984, 0x17fffe49 }, /* b 0x2a8 <fail> */
    /* Current EL SPx vectors (+0x200 .. +0x380 from VBAR_EL3 = 0x800) */
    { 0xa00, 0x52800ec2 }, /* mov w2, #'v' */
    { 0xa04, 0x17fffe29 }, /* b 0x2a8 <fail> */
    { 0xa80, 0x52800d22 }, /* mov w2, #'i' */
    { 0xa84, 0x17fffe09 }, /* b 0x2a8 <fail> */
    { 0xb00, 0xd538cc13 }, /* mrs x19, ICC_IAR1_EL1 */
    { 0xb04, 0xd518cc33 }, /* msr ICC_EOIR1_EL1, x19 */
    { 0xb08, 0xd5033fdf }, /* isb */
    { 0xb0c, 0xd69f03e0 }, /* eret */
    { 0xb80, 0x52800e62 }, /* mov w2, #'s' */
    { 0xb84, 0x17fffdc9 }, /* b 0x2a8 <fail> */
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

static void test_gicv3_group_enable_hppi(void)
{
    uint32_t rom[0xc00 / sizeof(uint32_t)];
    g_autofree char *serialtmp = NULL;
    g_autofree char *romtmp = NULL;
    QTestState *qts;
    char line[64] = { 0 };
    int ser_fd, rom_fd;
    size_t i;
    bool got_line;

    if (!qtest_has_accel("tcg")) {
        g_test_skip("TCG accelerator not available");
        return;
    }
    if (!qtest_has_machine("virt")) {
        g_test_skip("virt machine not available");
        return;
    }

    for (i = 0; i < ARRAY_SIZE(rom); i++) {
        rom[i] = cpu_to_le32(0xd503201f); /* nop */
    }
    for (i = 0; i < ARRAY_SIZE(rom_main); i++) {
        rom[i] = cpu_to_le32(rom_main[i]);
    }
    for (i = 0; i < ARRAY_SIZE(rom_vectors); i++) {
        rom[rom_vectors[i].offset / 4] = cpu_to_le32(rom_vectors[i].insn);
    }

    ser_fd = g_file_open_tmp("qtest-gicv3-grp-sXXXXXX", &serialtmp, NULL);
    g_assert_cmpint(ser_fd, !=, -1);
    close(ser_fd);

    rom_fd = g_file_open_tmp("qtest-gicv3-grp-rXXXXXX", &romtmp, NULL);
    g_assert_cmpint(rom_fd, !=, -1);
    write_full(rom_fd, rom, sizeof(rom));
    close(rom_fd);

    qts = qtest_initf("-machine virt,secure=on,gic-version=3 "
                      "-cpu cortex-a55 -accel tcg -no-shutdown "
                      "-bios %s -chardev file,id=serial0,path=%s "
                      "-serial chardev:serial0",
                      romtmp, serialtmp);

    ser_fd = open(serialtmp, O_RDONLY);
    g_assert_cmpint(ser_fd, !=, -1);
    got_line = read_serial_line(qts, ser_fd, line, sizeof(line));
    close(ser_fd);

    qtest_quit(qts);
    g_unlink(serialtmp);
    g_unlink(romtmp);

    g_assert_true(got_line);
    g_assert_cmpstr(line, ==, "OK\n");
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/arm/gicv3/group-enable-hppi",
                   test_gicv3_group_enable_hppi);
    return g_test_run();
}
