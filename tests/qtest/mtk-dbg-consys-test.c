/*
 * MT6895 DebugTop DEM/tracer and CONSYS WF/BGF MMIO window regression test.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

static void check_mmio_rw32(QTestState *qts, uint64_t addr, uint32_t pattern)
{
    g_assert_cmphex(qtest_readl(qts, addr), ==, 0);
    qtest_writel(qts, addr, pattern);
    g_assert_cmphex(qtest_readl(qts, addr), ==, pattern);
    qtest_writel(qts, addr, 0);
    g_assert_cmphex(qtest_readl(qts, addr), ==, 0);
}

static void test_dbg_dem_consys_mmio(void)
{
    static const uint64_t addrs[] = {
        0x0d01a000ULL, /* bus_tracer dbgao */
        0x0d020000ULL, /* dbg_cti */
        0x0d030000ULL, /* dbg_etr */
        0x0d041000ULL, /* bus_tracer etr */
        0x0d044000ULL, /* bus_tracer etf */
        0x0d0a0088ULL, /* dbg_dem DEBUGTOP_MON */
        0x0d100000ULL, /* dbg_mdsys1 */
        0x18400120ULL, /* consys wf top */
        0x184b0010ULL, /* consys wf cfg */
        0x184c0800ULL, /* consys wf mcu */
        0x184f0440ULL, /* consys wf misc */
        0x18500000ULL, /* consys wf misc upper */
        0x18820284ULL, /* consys bgf cfg */
        0x18830000ULL, /* consys bgf sram */
        0x18900000ULL, /* consys bgf top */
    };
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");

    for (size_t i = 0; i < ARRAY_SIZE(addrs); i++) {
        check_mmio_rw32(qts, addrs[i], 0x5a000000U + (uint32_t)i);
    }

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/mt6895/dbg-dem-consys-mmio", test_dbg_dem_consys_mmio);
    return g_test_run();
}
