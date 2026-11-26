/*
 * MT6895 cpufreq-hw, DPMAIF v3, and Arm Mali-G610 control register QTests.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define CPUFREQ_BASE        0x0011bc10ULL
#define CPUFREQ_DOM_STRIDE  0x120ULL
#define CPUFREQ_ENABLE      0x084
#define CPUFREQ_PERF_STATE  0x088
#define CPUFREQ_HW_STATE    0x08c
#define CPUFREQ_EM_POWER0   0x090
#define CPUFREQ_LATENCY     0x114
#define CPUFREQ_EAS_LIMITS  0x0011ce08ULL

#define DPMAIF_AO_BASE      0x10014000ULL
#define DPMAIF_PD_BASE      0x1022d000ULL

#define MALI_BASE           0x13000000ULL

static void test_cpufreq_hw(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");
    unsigned dom, idx;

    for (dom = 0; dom < 3; dom++) {
        uint64_t base = CPUFREQ_BASE + dom * CPUFREQ_DOM_STRIDE;
        uint32_t prev_freq = qtest_readl(qts, base) & 0xfffu;
        uint32_t prev_pwr = qtest_readl(qts, base + CPUFREQ_EM_POWER0);

        /* Start at the highest OPP and repeat the lowest OPP after slot 7. */
        g_assert_cmpuint(qtest_readl(qts, base + CPUFREQ_PERF_STATE), ==, 0u);
        for (idx = 1; idx < 8; idx++) {
            uint32_t mhz = qtest_readl(qts, base + idx * 4) & 0xfffu;
            uint32_t uw = qtest_readl(qts, base + CPUFREQ_EM_POWER0 + idx * 4);

            g_assert_cmpuint(mhz, <, prev_freq);
            g_assert_cmpuint(uw, <, prev_pwr);
            prev_freq = mhz;
            prev_pwr = uw;
        }
        g_assert_cmpuint(qtest_readl(qts, base + 8 * 4) & 0xfffu, ==, prev_freq);
        g_assert_cmpuint(qtest_readl(qts, base + CPUFREQ_EM_POWER0 + 8 * 4),
                         ==, prev_pwr);

        /* Latency is 1000 us and REG_FREQ_ENABLE=1 yields HW+SVS enabled */
        g_assert_cmpuint(qtest_readl(qts, base + CPUFREQ_LATENCY), ==, 1000u);
        qtest_writel(qts, base + CPUFREQ_ENABLE, 1u);
        g_assert_cmphex(qtest_readl(qts, base + CPUFREQ_HW_STATE) & 0x3u,
                        ==, 0x3u);

        /* REG_FREQ_PERF_STATE accepts valid OPP index */
        qtest_writel(qts, base + CPUFREQ_PERF_STATE, 3u);
        g_assert_cmpuint(qtest_readl(qts, base + CPUFREQ_PERF_STATE), ==, 3u);
    }

    /* The thermal controller's CSRAM words hold separate per-domain limits. */
    for (dom = 0; dom < 4; dom++) {
        uint64_t limit = CPUFREQ_EAS_LIMITS + dom * 4;

        g_assert_cmphex(qtest_readl(qts, limit), ==, 0u);
        qtest_writel(qts, limit, dom + 1);
    }
    for (dom = 0; dom < 4; dom++) {
        g_assert_cmphex(qtest_readl(qts, CPUFREQ_EAS_LIMITS + dom * 4),
                        ==, dom + 1);
    }
    qtest_system_reset(qts);
    for (dom = 0; dom < 4; dom++) {
        g_assert_cmphex(qtest_readl(qts, CPUFREQ_EAS_LIMITS + dom * 4), ==, 0u);
    }

    qtest_quit(qts);
}

static void test_dpmaif(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");
    static const uint32_t sc_bit31_offs[] = { 0x0b0, 0x0b4, 0x0b8, 0x0bc,
                                              0x100, 0x104, 0x120, 0x124 };
    size_t i;

    /* Match the paired load in the modem driver's register dump. */
    qtest_writel(qts, DPMAIF_PD_BASE, 0x11223344u);
    qtest_writel(qts, DPMAIF_PD_BASE + 4, 0x55667788u);
    g_assert_cmphex(qtest_readq(qts, DPMAIF_PD_BASE), ==, 0x5566778811223344ULL);

    /* AO_UL +0x84/+0x94 CLR and +0x88/+0x98 SET mirror hardware masks */
    qtest_writel(qts, DPMAIF_AO_BASE + 0x084, 0x01fffc1fu);
    qtest_writel(qts, DPMAIF_AO_BASE + 0x088, 0xfe0003e0u);
    g_assert_cmphex(qtest_readl(qts, DPMAIF_AO_BASE + 0x080), ==, 0xfe0003e0u);

    qtest_writel(qts, DPMAIF_AO_BASE + 0x094, 0x00000043u);
    qtest_writel(qts, DPMAIF_AO_BASE + 0x098, 0xffffffbcu);
    g_assert_cmphex(qtest_readl(qts, DPMAIF_AO_BASE + 0x090), ==, 0xffffffbcu);

    /* A byte write to CLR must not replay a previous command in other lanes. */
    qtest_writeb(qts, DPMAIF_AO_BASE + 0x084, 0x33);
    qtest_writel(qts, DPMAIF_AO_BASE + 0x080, 0xffffffffu);
    qtest_writeb(qts, DPMAIF_AO_BASE + 0x085, 0x80);
    g_assert_cmphex(qtest_readl(qts, DPMAIF_AO_BASE + 0x080), ==, 0xffff7fffu);

    /* PD_MISC +0x90 (hw_init_done) and +0x94 (sram_init) self-clear bit 0 */
    qtest_writel(qts, DPMAIF_PD_BASE + 0x494, 1u);
    g_assert_cmphex(qtest_readl(qts, DPMAIF_PD_BASE + 0x494) & 1u, ==, 0u);
    qtest_writel(qts, DPMAIF_PD_BASE + 0x490, 1u);
    g_assert_cmphex(qtest_readl(qts, DPMAIF_PD_BASE + 0x490) & 1u, ==, 0u);

    /* PD_MISC +0x98 (dl_restore) sets bits in +0x90 */
    qtest_writel(qts, DPMAIF_PD_BASE + 0x498, 0x20u);
    g_assert_cmphex(qtest_readl(qts, DPMAIF_PD_BASE + 0x490) & 0x20u,
                    ==, 0x20u);
    qtest_writel(qts, DPMAIF_PD_BASE + 0x490, 0);
    qtest_writeb(qts, DPMAIF_PD_BASE + 0x499, 0x41);
    g_assert_cmphex(qtest_readl(qts, DPMAIF_PD_BASE + 0x490), ==, 0x4100u);

    /* PD_UL / PD_DL BIT(31) busy-trigger registers self-clear */
    for (i = 0; i < G_N_ELEMENTS(sc_bit31_offs); i++) {
        uint64_t addr = DPMAIF_PD_BASE + sc_bit31_offs[i];

        qtest_writel(qts, addr, 0x80000005u);
        g_assert_cmphex(qtest_readl(qts, addr) & 0x80000000u, ==, 0u);
        g_assert_cmphex(qtest_readl(qts, addr), ==, 5u);
    }

    /* Idle checks: PD_UL +0x88 bit 11 and PD_DL +0xb4 (+0x1b4) bit 23 */
    g_assert_cmphex(qtest_readl(qts, DPMAIF_PD_BASE + 0x088) & (1u << 11),
                    ==, 1u << 11);
    g_assert_cmphex(qtest_readl(qts, DPMAIF_PD_BASE + 0x1b4) & (1u << 23),
                    ==, 1u << 23);

    qtest_quit(qts);
}

static void test_mali_gpu(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");

    /* GPU_ID (Mali-G610 MC6) and core/feature masks */
    g_assert_cmphex(qtest_readl(qts, MALI_BASE + 0x000), ==, 0xa8670000u);
    g_assert_cmphex(qtest_readl(qts, MALI_BASE + 0x014), ==, 0x00002830u);
    g_assert_cmphex(qtest_readl(qts, MALI_BASE + 0x100), ==, 0x0000003fu);

    /* Soft reset sets RESET_COMPLETED (bit 8); IRQ_CLEAR clears it */
    qtest_writel(qts, MALI_BASE + 0x030, 0x01u);
    g_assert_cmphex(qtest_readl(qts, MALI_BASE + 0x020) & (1u << 8),
                    ==, 1u << 8);
    qtest_writel(qts, MALI_BASE + 0x024, 1u << 8);
    g_assert_cmphex(qtest_readl(qts, MALI_BASE + 0x020) & (1u << 8), ==, 0u);

    /* CACHE_CLN_INV (0x103) sets CLEAN_CACHES_COMPLETED (bit 17) */
    qtest_writel(qts, MALI_BASE + 0x030, 0x103u);
    g_assert_cmphex(qtest_readl(qts, MALI_BASE + 0x020) & (1u << 17),
                    ==, 1u << 17);

    /* SHADER_PWRON_LO updates SHADER_READY_LO */
    qtest_writel(qts, MALI_BASE + 0x180, 0x3fu);
    g_assert_cmphex(qtest_readl(qts, MALI_BASE + 0x140), ==, 0x3fu);

    /* JOB/MMU IRQ banks and upper 0x480000 window do not abort */
    qtest_writel(qts, MALI_BASE + 0x1004, 0xffffffffu);
    qtest_writel(qts, MALI_BASE + 0x2004, 0xffffffffu);
    g_assert_cmphex(qtest_readl(qts, MALI_BASE + 0x47fffc), ==, 0u);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/mt6895/p127/cpufreq_hw", test_cpufreq_hw);
    qtest_add_func("/mt6895/p127/dpmaif", test_dpmaif);
    qtest_add_func("/mt6895/p127/mali_gpu", test_mali_gpu);
    return g_test_run();
}
