/*
 * MT6895 GPIO controller (pinctrl window) register-bank tests.
 *
 * The block decodes each register bank as a 0x10-byte group: data, SET, CLR
 * and, for the 4-bit MODE banks, the MWR write-enable alias.  These tests
 * drive the registers the boot chain drives and check the resulting data
 * words; see mtk_syscon.c for the evidence behind each rule.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define PIO_BASE    0x10005000ULL
#define PIO_DIR     0x000
#define PIO_DOUT    0x100
#define PIO_MODE    0x300
#define PIO_SET     0x04
#define PIO_CLR     0x08
#define PIO_MWR     0x0c

/* GPIO197: the vendor dtbo defaults it to mode 1 (bank 24, nibble 5). */
#define GPIO197_BANK    (PIO_MODE + 24 * 0x10)
#define GPIO197_SHIFT   ((197 % 8) * 4)

static void test_mode_mwr(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");
    const uint32_t mode197 = 1u << GPIO197_SHIFT;
    const uint32_t mwr197 = (1u << (GPIO197_SHIFT + 3)) | mode197;

    g_assert_cmphex(qtest_readl(qts, PIO_BASE + GPIO197_BANK), ==, 0);

    /*
     * bl2_ext writes ((mode & 7) << shift) | (1 << (shift + 3)) to the MWR
     * alias; the enabled nibble must land in the data word.
     */
    qtest_writel(qts, PIO_BASE + GPIO197_BANK + PIO_MWR, mwr197);
    g_assert_cmphex(qtest_readl(qts, PIO_BASE + GPIO197_BANK), ==, mode197);

    /*
     * The check scp.ko performs on the vendor DT gpio-vreq entry: the pad
     * reads back as mode 1 at bits [22:20] of the mode word.
     */
    g_assert_cmphex((qtest_readl(qts, PIO_BASE + GPIO197_BANK) >> 20) & 7, ==, 1);

    /* A nibble without its write-enable bit must not be modified. */
    qtest_writel(qts, PIO_BASE + GPIO197_BANK + PIO_MWR, 0xffffffffu);
    g_assert_cmphex(qtest_readl(qts, PIO_BASE + GPIO197_BANK), ==,
                    0x77777777u);

    /* Write-enable with mode 0 clears the field and leaves the rest alone. */
    qtest_writel(qts, PIO_BASE + GPIO197_BANK + PIO_MWR, 0x80000000u);
    g_assert_cmphex(qtest_readl(qts, PIO_BASE + GPIO197_BANK), ==,
                    0x07777777u);

    /* Alias words are write-only; only the data word reads back. */
    g_assert_cmphex(qtest_readl(qts, PIO_BASE + GPIO197_BANK + PIO_MWR), ==,
                    0);
    g_assert_cmphex(qtest_readl(qts, PIO_BASE + GPIO197_BANK + PIO_SET), ==,
                    0);

    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, PIO_BASE + GPIO197_BANK), ==, 0);
    qtest_quit(qts);
}

static void test_mode_set_clr(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");

    /* Pin 2 of bank 0 lives in nibble 2; SET and CLR touch one mode bit. */
    qtest_writel(qts, PIO_BASE + PIO_MODE + PIO_SET, 1u << 8);
    g_assert_cmphex(qtest_readl(qts, PIO_BASE + PIO_MODE), ==, 0x00000100u);
    qtest_writel(qts, PIO_BASE + PIO_MODE + PIO_CLR, 1u << 8);
    g_assert_cmphex(qtest_readl(qts, PIO_BASE + PIO_MODE), ==, 0);
    qtest_writel(qts, PIO_BASE + PIO_MODE + PIO_SET, 0x3u << 8);
    g_assert_cmphex(qtest_readl(qts, PIO_BASE + PIO_MODE), ==, 0x00000300u);

    /* The data word keeps three mode bits per pin, not four. */
    qtest_writel(qts, PIO_BASE + PIO_MODE, 0xffffffffu);
    g_assert_cmphex(qtest_readl(qts, PIO_BASE + PIO_MODE), ==, 0x77777777u);
    qtest_quit(qts);
}

static void test_dir_dout_set_clr(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");

    /* DIR and DOUT hold one bit per pin and share the SET/CLR decoding. */
    qtest_writel(qts, PIO_BASE + PIO_DIR + PIO_SET, 1u << 10);
    g_assert_cmphex(qtest_readl(qts, PIO_BASE + PIO_DIR), ==, 1u << 10);
    qtest_writel(qts, PIO_BASE + PIO_DOUT + PIO_SET, 1u << 10);
    g_assert_cmphex(qtest_readl(qts, PIO_BASE + PIO_DOUT), ==, 1u << 10);
    qtest_writel(qts, PIO_BASE + PIO_DOUT + PIO_CLR, 1u << 10);
    g_assert_cmphex(qtest_readl(qts, PIO_BASE + PIO_DOUT), ==, 0);
    qtest_writel(qts, PIO_BASE + PIO_DIR + PIO_CLR, 1u << 10);
    g_assert_cmphex(qtest_readl(qts, PIO_BASE + PIO_DIR), ==, 0);

    /* A plain data write still replaces the whole word. */
    qtest_writel(qts, PIO_BASE + PIO_DIR, 0x80000000u);
    g_assert_cmphex(qtest_readl(qts, PIO_BASE + PIO_DIR), ==, 0x80000000u);
    qtest_quit(qts);
}

static void test_bank_scope(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");

    /*
     * The vendor DT declares 227 pads: MODE covers 0x300..0x4cf and the
     * one-bit banks cover 0x000..0x07f / 0x100..0x17f.  Everything else in
     * the window stays a plain register file.
     */
    qtest_writel(qts, PIO_BASE + 0x4d0, 0xffffffffu);
    g_assert_cmphex(qtest_readl(qts, PIO_BASE + 0x4d0), ==, 0xffffffffu);
    qtest_writel(qts, PIO_BASE + 0x090, 0xffffffffu);
    g_assert_cmphex(qtest_readl(qts, PIO_BASE + 0x090), ==, 0xffffffffu);

    /* The last modelled MODE bank holds pins 224..231. */
    qtest_writel(qts, PIO_BASE + PIO_MODE + 28 * 0x10 + PIO_MWR, 0x8000000fu);
    g_assert_cmphex(qtest_readl(qts, PIO_BASE + PIO_MODE + 28 * 0x10), ==, 7u);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/mtk-pinctrl/mode-mwr", test_mode_mwr);
    qtest_add_func("/mtk-pinctrl/mode-set-clr", test_mode_set_clr);
    qtest_add_func("/mtk-pinctrl/dir-dout-set-clr", test_dir_dout_set_clr);
    qtest_add_func("/mtk-pinctrl/bank-scope", test_bank_scope);
    return g_test_run();
}
