/*
 * MT6895 LVTS thermal controller, TIA NTC ADC, MD1 power sequencer, and
 * SPM MD_EXT_BUCK_ISO acknowledge QTests.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define LVTS_D0_BASE        0x10315000ULL
#define LVTS_D1_BASE        0x10316000ULL
#define LVTS_TC_STRIDE      0x100ULL

#define LVTS_REG_ID         0x4c
#define LVTS_REG_CONFIG0    0x50
#define LVTS_REG_MSR0       0x90
#define LVTS_REG_MSR3       0x9c
#define LVTS_REG_RDATA0     0xb0
#define LVTS_REG_SPARE0     0xf0

#define TIA_BASE            0x1c805000ULL
#define MD1_SEQ_BASE        0x1c803000ULL
#define SPM_BASE            0x1c001000ULL
#define SPM_MD_EXT_BUCK_ISO 0x0404

static void test_lvts(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");
    unsigned tc;
    uint64_t off;

    /* Domain 0: TC3..7 report IDs 0x85..0x89 */
    for (tc = 0; tc < 5; tc++) {
        uint64_t base = LVTS_D0_BASE + tc * LVTS_TC_STRIDE;

        g_assert_cmphex(qtest_readl(qts, base + LVTS_REG_ID), ==, 0x85u + tc);
    }

    /* Domain 1: TC0..2 report IDs 0x81..0x83 */
    for (tc = 0; tc < 3; tc++) {
        uint64_t base = LVTS_D1_BASE + tc * LVTS_TC_STRIDE;

        g_assert_cmphex(qtest_readl(qts, base + LVTS_REG_ID), ==, 0x81u + tc);
    }

    /* Writing +0x50 clears bits 24/25 immediately and latches +0xb0 on 0xc1xx */
    qtest_writel(qts, LVTS_D0_BASE + LVTS_REG_CONFIG0, 0x03000000u);
    g_assert_cmphex(qtest_readl(qts, LVTS_D0_BASE + LVTS_REG_CONFIG0) &
                    (0x3u << 24), ==, 0);

    qtest_writel(qts, LVTS_D1_BASE + LVTS_REG_CONFIG0, 0xc1020000u);
    g_assert_cmphex(qtest_readl(qts, LVTS_D1_BASE + LVTS_REG_RDATA0), ==,
                    0x4000u);

    /* MSR0..MSR3 report valid nominal readings across all Domain 0/1 TCs */
    for (off = LVTS_REG_MSR0; off <= LVTS_REG_MSR3; off += 4) {
        g_assert_cmphex(qtest_readl(qts, LVTS_D0_BASE + off), ==, 0x00014000u);
        g_assert_cmphex(qtest_readl(qts, LVTS_D1_BASE + off), ==, 0x00014000u);
    }

    /* +0xf0 (LVTSSPARE0) preserves RW values for lvts_lk_init_check */
    qtest_writel(qts, LVTS_D0_BASE + LVTS_REG_SPARE0, 0x555u);
    g_assert_cmphex(qtest_readl(qts, LVTS_D0_BASE + LVTS_REG_SPARE0), ==,
                    0x555u);
    qtest_writel(qts, LVTS_D0_BASE + LVTS_REG_SPARE0, 0u);
    g_assert_cmphex(qtest_readl(qts, LVTS_D0_BASE + LVTS_REG_SPARE0), ==, 0u);

    qtest_quit(qts);
}

static void test_tia(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");
    static const uint32_t adc_offs[] = { 0x0cc, 0x0d4, 0x0d8, 0x0dc, 0x0f0 };
    size_t i;

    for (i = 0; i < G_N_ELEMENTS(adc_offs); i++) {
        g_assert_cmphex(qtest_readl(qts, TIA_BASE + adc_offs[i]), ==,
                        0x0000c000u);
    }

    /* Non-ADC control slots (+0x28 / +0x30) retain RW storage */
    qtest_writel(qts, TIA_BASE + 0x28, 0x12345678u);
    qtest_writel(qts, TIA_BASE + 0x30, 0x9abcdef0u);
    g_assert_cmphex(qtest_readl(qts, TIA_BASE + 0x28), ==, 0x12345678u);
    g_assert_cmphex(qtest_readl(qts, TIA_BASE + 0x30), ==, 0x9abcdef0u);

    qtest_quit(qts);
}

static void test_mdseq_and_spm(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");

    /* MD1 power sequencer (0x1c803000) */
    g_assert_cmphex(qtest_readl(qts, MD1_SEQ_BASE + 0x310), ==, 0x01010001u);
    qtest_writel(qts, MD1_SEQ_BASE + 0x204, 1u);
    g_assert_cmphex(qtest_readl(qts, MD1_SEQ_BASE + 0x310), ==, 0x04040040u);
    qtest_writel(qts, MD1_SEQ_BASE + 0x204, 0u);
    g_assert_cmphex(qtest_readl(qts, MD1_SEQ_BASE + 0x310), ==, 0x01010001u);

    /* SPM +0x0404 (MD_EXT_BUCK_ISO) bit 4 mirrors bit 0 */
    qtest_writel(qts, SPM_BASE + SPM_MD_EXT_BUCK_ISO, 1u);
    g_assert_cmphex(qtest_readl(qts, SPM_BASE + SPM_MD_EXT_BUCK_ISO) & 0x10u,
                    ==, 0x10u);
    qtest_writel(qts, SPM_BASE + SPM_MD_EXT_BUCK_ISO, 0u);
    g_assert_cmphex(qtest_readl(qts, SPM_BASE + SPM_MD_EXT_BUCK_ISO) & 0x10u,
                    ==, 0u);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/mt6895/thermal/lvts", test_lvts);
    qtest_add_func("/mt6895/thermal/tia", test_tia);
    qtest_add_func("/mt6895/thermal/mdseq_and_spm", test_mdseq_and_spm);
    return g_test_run();
}
