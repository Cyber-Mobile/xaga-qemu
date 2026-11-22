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

static void test_miu_aperture_scope(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");

    /* Preserve the existing register-file behavior outside the known family. */
    qtest_writel(qts, MIU_SECOND_BANK + 0x00c, MIU_PROGRAM_FLAG);
    qtest_writel(qts, SYS_CIRQ_BASE + 0x00c, MIU_PROGRAM_FLAG);
    qtest_writel(qts, SPECIAL_END_HI, MIU_PROGRAM_FLAG);
    qtest_writel(qts, SR01_ENABLE, MIU_PROGRAM_FLAG | 1);
    g_assert_cmphex(qtest_readl(qts, MIU_SECOND_BANK + 0x00c), ==,
                    MIU_PROGRAM_FLAG);
    g_assert_cmphex(qtest_readl(qts, SYS_CIRQ_BASE + 0x00c), ==,
                    MIU_PROGRAM_FLAG);
    g_assert_cmphex(qtest_readl(qts, SPECIAL_END_HI), ==, MIU_PROGRAM_FLAG);
    g_assert_cmphex(qtest_readl(qts, SR01_ENABLE), ==, MIU_PROGRAM_FLAG | 1);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/mtk-miu/range-readback", test_miu_range_readback);
    qtest_add_func("/mtk-miu/aperture-scope", test_miu_aperture_scope);
    return g_test_run();
}
