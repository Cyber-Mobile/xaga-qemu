/*
 * MT6895 MIU range readback tests.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define MIU_BASE            0x10351000
#define MIU_SECOND_BANK     0x10355000
#define SYS_CIRQ_BASE       0x10312000
#define SR01_START_LO       (MIU_BASE + 0x000)
#define SR01_START_HI       (MIU_BASE + 0x004)
#define SR01_END_LO         (MIU_BASE + 0x008)
#define SR01_END_HI         (MIU_BASE + 0x00c)
#define SR01_ENABLE         (MIU_BASE + 0x1d8)
#define SR47_END_HI         (MIU_BASE + 0x4ec)
#define SPECIAL_END_HI      (MIU_BASE + 0x0fc)
#define MIU_PROGRAM_FLAG    (1u << 15)

static void test_miu_range_readback(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");
    uint32_t end_lo, end_hi;
    uint64_t end;

    g_assert_cmphex(qtest_readl(qts, SR01_END_LO), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SR01_END_HI), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SR01_ENABLE), ==, 0);

    /* The device BL31 programs SR01 before setting its separate enable bit. */
    qtest_writel(qts, SR01_START_LO, 0x8200);
    qtest_writel(qts, SR01_START_HI, 0);
    qtest_writel(qts, SR01_END_LO, 0x8400);
    qtest_writel(qts, SR01_END_HI, MIU_PROGRAM_FLAG);
    g_assert_cmphex(qtest_readl(qts, SR01_ENABLE), ==, 0);
    qtest_writel(qts, SR01_ENABLE, 1);

    end_lo = qtest_readl(qts, SR01_END_LO);
    end_hi = qtest_readl(qts, SR01_END_HI);
    g_assert_cmphex(end_lo, ==, 0x8400);
    g_assert_cmphex(end_hi, ==, 0);
    end = 0x40000000 + ((uint64_t)end_lo << 12) + ((uint64_t)end_hi << 28);
    g_assert_cmphex(end, ==, 0x48400000);
    g_assert_cmphex(qtest_readl(qts, SR01_START_LO), ==, 0x8200);
    g_assert_cmphex(qtest_readl(qts, SR01_ENABLE), ==, 1);

    /* The high address bits in an ordinary third-group descriptor survive. */
    qtest_writel(qts, SR47_END_HI, MIU_PROGRAM_FLAG | 3);
    g_assert_cmphex(qtest_readl(qts, SR47_END_HI), ==, 3);

    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, SR01_START_LO), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SR01_END_LO), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SR01_END_HI), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SR01_ENABLE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SR47_END_HI), ==, 0);
    qtest_quit(qts);
}


/* Region 32 lives in the last slot of the second group (0x2fc). */
static void test_miu_last_slot_end_readback(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");
    const uint64_t d = MIU_BASE + 0x2f0;
    uint32_t end_lo, end_hi;
    uint64_t end;

    /* Same words the vendor CCCI leaves for 0xD0000000..0xD2710000. */
    qtest_writel(qts, d + 0x0, 0x0000);
    qtest_writel(qts, d + 0x4, 0x0009);
    qtest_writel(qts, d + 0x8, 0x2710);
    qtest_writel(qts, d + 0xc, 0x8009);

    g_assert_cmphex(qtest_readl(qts, d + 0xc), ==, 0x0009);
    end_lo = qtest_readl(qts, d + 0x8);
    end_hi = qtest_readl(qts, d + 0xc);
    end = 0x40000000 + ((uint64_t)end_lo << 12) + ((uint64_t)end_hi << 28);
    g_assert_cmphex(end, ==, 0xD2710000);
    g_assert_cmphex(end & (1ull << 43), ==, 0);

    /* Every other slot keeps the flag behaviour; 0x1fc is not a descriptor. */
    qtest_writel(qts, MIU_BASE + 0x00c, MIU_PROGRAM_FLAG);
    g_assert_cmphex(qtest_readl(qts, MIU_BASE + 0x00c), ==, 0);
    qtest_writel(qts, MIU_BASE + 0x1fc, MIU_PROGRAM_FLAG);
    g_assert_cmphex(qtest_readl(qts, MIU_BASE + 0x1fc), ==, MIU_PROGRAM_FLAG);
    qtest_quit(qts);
}

static void test_miu_aperture_scope(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");

    /* Preserve the existing register-file behavior outside the known family. */
    qtest_writel(qts, MIU_SECOND_BANK + 0x00c, MIU_PROGRAM_FLAG);
    qtest_writel(qts, SYS_CIRQ_BASE + 0x00c, MIU_PROGRAM_FLAG);
    qtest_writel(qts, SPECIAL_END_HI, MIU_PROGRAM_FLAG);
    qtest_writel(qts, SR01_ENABLE, MIU_PROGRAM_FLAG | 1);
    /*
     * The last descriptor slot of a group is a normal range descriptor too
     * (region 16, 32 and 48 land there), so its end high word must also lose
     * the programming flag.
     */
    g_assert_cmphex(qtest_readl(qts, MIU_SECOND_BANK + 0x00c), ==,
                    MIU_PROGRAM_FLAG);
    g_assert_cmphex(qtest_readl(qts, SYS_CIRQ_BASE + 0x00c), ==,
                    MIU_PROGRAM_FLAG);
    g_assert_cmphex(qtest_readl(qts, SPECIAL_END_HI), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SR01_ENABLE), ==, MIU_PROGRAM_FLAG | 1);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/mtk-miu/range-readback", test_miu_range_readback);
    qtest_add_func("/mtk-miu/aperture-scope", test_miu_aperture_scope);
    qtest_add_func("/mtk-miu/last-slot-end-readback",
                   test_miu_last_slot_end_readback);
    return g_test_run();
}
