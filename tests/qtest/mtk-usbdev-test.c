/*
 * MT6895 virtual UDC (mtk-usbdev) QMU stop/restart tests.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define USBDEV_BASE         0x11201000ULL
#define R_TXQHIAR1          0x0484
#define R_RXQHIAR1          0x04c4
#define R_TXQCSR1           0x0510
#define R_TXQSAR1           0x0514
#define R_TXQCPR1           0x0518
#define R_RXQCSR1           0x0610
#define R_RXQSAR1           0x0614
#define R_RXQCPR1           0x0618
#define R_QISAR0            0x0700

#define QMU_Q_START         (1u << 0)
#define QMU_Q_STOP          (1u << 2)
#define QMU_Q_ACTIVE        (1u << 15)
#define QMU_TX_DONE_1       (1u << 1)

#define TX_RING_1           0x40100000ULL
#define RX_RING_1           0x40101000ULL
#define TX_RING_2           0x40200000ULL
#define RX_RING_2           0x40201000ULL
#define TX_BUF_2            0x40202000ULL

static void zero_mem(QTestState *qts, uint64_t addr, size_t len)
{
    uint8_t zeros[32] = { 0 };

    while (len > 0) {
        size_t step = MIN(len, sizeof(zeros));

        qtest_memwrite(qts, addr, zeros, step);
        addr += step;
        len -= step;
    }
}

static void write_tx_gpd(QTestState *qts, uint64_t gaddr, uint8_t flags,
                         uint64_t next, uint64_t buf, uint16_t len)
{
    uint8_t gpd[16] = { 0 };

    gpd[0] = flags;
    gpd[2] = ((buf >> 32) & 0xf) | (((next >> 32) & 0xf) << 4);
    gpd[4] = next & 0xff;
    gpd[5] = (next >> 8) & 0xff;
    gpd[6] = (next >> 16) & 0xff;
    gpd[7] = (next >> 24) & 0xff;
    gpd[8] = buf & 0xff;
    gpd[9] = (buf >> 8) & 0xff;
    gpd[10] = (buf >> 16) & 0xff;
    gpd[11] = (buf >> 24) & 0xff;
    gpd[12] = len & 0xff;
    gpd[13] = (len >> 8) & 0xff;
    qtest_memwrite(qts, gaddr, gpd, sizeof(gpd));
}

static void test_usbdev_qmu_stop_restart(void)
{
    QTestState *qts;
    const uint8_t payload[8] = { 'A', 'D', 'B', 'T', 'E', 'S', 'T', '2' };

    g_setenv("MTK_USBDEV", "1", true);
    g_setenv("MTK_USBDEV_CHR", "qtestusb", true);
    qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults "
                     "-chardev null,id=qtestusb");

    g_assert_cmphex(qtest_readl(qts, USBDEV_BASE + R_TXQCSR1) & QMU_Q_ACTIVE,
                    ==, 0u);
    g_assert_cmphex(qtest_readl(qts, USBDEV_BASE + R_RXQCSR1) & QMU_Q_ACTIVE,
                    ==, 0u);

    /* First gadget enable phase: start TX and RX QMU rings. */
    zero_mem(qts, TX_RING_1, 32);
    zero_mem(qts, RX_RING_1, 32);
    qtest_writel(qts, USBDEV_BASE + R_TXQHIAR1, 0);
    qtest_writel(qts, USBDEV_BASE + R_TXQSAR1, (uint32_t)TX_RING_1);
    qtest_writel(qts, USBDEV_BASE + R_RXQHIAR1, 0);
    qtest_writel(qts, USBDEV_BASE + R_RXQSAR1, (uint32_t)RX_RING_1);
    qtest_writel(qts, USBDEV_BASE + R_TXQCSR1, QMU_Q_START);
    qtest_writel(qts, USBDEV_BASE + R_RXQCSR1, QMU_Q_START);

    g_assert_cmphex(qtest_readl(qts, USBDEV_BASE + R_TXQCSR1) & QMU_Q_ACTIVE,
                    ==, QMU_Q_ACTIVE);
    g_assert_cmphex(qtest_readl(qts, USBDEV_BASE + R_RXQCSR1) & QMU_Q_ACTIVE,
                    ==, QMU_Q_ACTIVE);
    g_assert_cmphex(qtest_readl(qts, USBDEV_BASE + R_RXQCPR1), ==,
                    (uint32_t)RX_RING_1);

    /* Gadget disable / role switch: QMU_Q_STOP must clear QMU_Q_ACTIVE. */
    qtest_writel(qts, USBDEV_BASE + R_TXQCSR1, QMU_Q_STOP);
    qtest_writel(qts, USBDEV_BASE + R_RXQCSR1, QMU_Q_STOP);
    g_assert_cmphex(qtest_readl(qts, USBDEV_BASE + R_TXQCSR1) &
                    (QMU_Q_ACTIVE | QMU_Q_STOP), ==, 0u);
    g_assert_cmphex(qtest_readl(qts, USBDEV_BASE + R_RXQCSR1) &
                    (QMU_Q_ACTIVE | QMU_Q_STOP), ==, 0u);

    /* Acknowledge interrupts and arm a fresh ring pair for second gadget bind. */
    qtest_writel(qts, USBDEV_BASE + R_QISAR0, 0xffffffffu);
    zero_mem(qts, TX_RING_2, 32);
    zero_mem(qts, RX_RING_2, 32);
    qtest_memwrite(qts, TX_BUF_2, payload, sizeof(payload));
    write_tx_gpd(qts, TX_RING_2, 0x81, TX_RING_2 + 16, TX_BUF_2,
                 sizeof(payload));

    qtest_writel(qts, USBDEV_BASE + R_TXQHIAR1, 0);
    qtest_writel(qts, USBDEV_BASE + R_TXQSAR1, (uint32_t)TX_RING_2);
    qtest_writel(qts, USBDEV_BASE + R_RXQHIAR1, 0);
    qtest_writel(qts, USBDEV_BASE + R_RXQSAR1, (uint32_t)RX_RING_2);
    g_assert_cmphex(qtest_readl(qts, USBDEV_BASE + R_TXQCSR1) & QMU_Q_ACTIVE,
                    ==, 0u);
    g_assert_cmphex(qtest_readl(qts, USBDEV_BASE + R_RXQCSR1) & QMU_Q_ACTIVE,
                    ==, 0u);

    qtest_writel(qts, USBDEV_BASE + R_TXQCSR1, QMU_Q_START);
    qtest_writel(qts, USBDEV_BASE + R_RXQCSR1, QMU_Q_START);

    g_assert_cmphex(qtest_readl(qts, USBDEV_BASE + R_TXQCSR1) & QMU_Q_ACTIVE,
                    ==, QMU_Q_ACTIVE);
    g_assert_cmphex(qtest_readl(qts, USBDEV_BASE + R_RXQCSR1) & QMU_Q_ACTIVE,
                    ==, QMU_Q_ACTIVE);
    g_assert_cmpuint(qtest_readb(qts, TX_RING_2) & 1u, ==, 0u);
    g_assert_cmphex(qtest_readl(qts, USBDEV_BASE + R_TXQCPR1), ==,
                    (uint32_t)(TX_RING_2 + 16));
    g_assert_cmphex(qtest_readl(qts, USBDEV_BASE + R_QISAR0) & QMU_TX_DONE_1,
                    ==, QMU_TX_DONE_1);
    g_assert_cmphex(qtest_readl(qts, USBDEV_BASE + R_RXQCPR1), ==,
                    (uint32_t)RX_RING_2);

    qtest_quit(qts);
    g_unsetenv("MTK_USBDEV");
    g_unsetenv("MTK_USBDEV_CHR");
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/mt6895/usbdev-qmu-stop-restart",
                   test_usbdev_qmu_stop_restart);
    return g_test_run();
}
