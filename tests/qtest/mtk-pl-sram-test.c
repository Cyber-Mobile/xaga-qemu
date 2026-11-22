/*
 * MT6895 preloader SRAM boundary regression test.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define I2C_COMPAT_ID5  0x020f36ec
#define LOG_ALLOC_GATE  0x020f3734
#define LOG_INIT_GUARD  0x020f3735

static void test_pl_sram_neighbor(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");

    /* Preserve the legacy I2C entry used by the preloader. */
    g_assert_cmphex(qtest_readb(qts, I2C_COMPAT_ID5), ==, 1);
    g_assert_cmphex(qtest_readb(qts, I2C_COMPAT_ID5 + 1), ==, 1);

    /* Reproduce only the verified device STRB, without running firmware. */
    qtest_writel(qts, LOG_ALLOC_GATE, 0);
    qtest_writeb(qts, LOG_INIT_GUARD, 1);
    g_assert_cmphex(qtest_readb(qts, LOG_ALLOC_GATE), ==, 0);
    g_assert_cmphex(qtest_readb(qts, LOG_INIT_GUARD), ==, 1);
    g_assert_cmphex(qtest_readl(qts, LOG_ALLOC_GATE), ==, 0x100);

    /* A write to the gate must likewise leave the adjacent guard alone. */
    qtest_writeb(qts, LOG_ALLOC_GATE, 0xa5);
    g_assert_cmphex(qtest_readb(qts, LOG_INIT_GUARD), ==, 1);
    g_assert_cmphex(qtest_readl(qts, LOG_ALLOC_GATE), ==, 0x1a5);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/mt6895/pl-sram-neighbor", test_pl_sram_neighbor);
    return g_test_run();
}
