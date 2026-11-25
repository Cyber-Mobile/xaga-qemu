/*
 * MT6895 ADSP subsystem and mailbox QTests.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define ADSP_CFG_BASE           0x1e000000ULL
#define ADSP_MBOX0_BASE         0x1e006000ULL
#define ADSP_MBOX1_BASE         0x1e007000ULL
#define ADSP_MBOX2_BASE         0x1e008000ULL
#define ADSP_MBOX3_BASE         0x1e009000ULL
#define ADSP_CFG2_BASE          0x1e00b000ULL
#define ADSP_C0_DTCM_BASE       0x1e020000ULL
#define ADSP_C1_DTCM_BASE       0x1e090000ULL

#define CFG_HIFI_IO_CONFIG      0x000c
#define CFG_CORE0_MEM_PWR       0x0044
#define CFG_CORE1_MEM_PWR       0x0048
#define CFG_MEM_PWR_ACK         0x004c
#define CFG_SLEEP_STATUS        0x0158
#define CFG2_BOOT_LOADER_CTRL   0x000c

#define MBOX_OFF_SET            0x0100
#define MBOX_OFF_CLR            0x010c
#define MBOX_HDR_SIZE           16
#define DTCM_WAKELOCK_OFF       0x7ff8

#define ADSP_IPI_MAGIC          0xdeadbeefU
#define ADSP_IPI_OPT_DEFAULT    0xffff0000U
#define ADSP_IPI_AUDIO          9
#define ADSP_IPI_ADSP_A_READY   13
#define ADSP_IPI_ADSP_A_SUSPEND 21

static void adsp_send_ipi(QTestState *qts, uint64_t mbox_base,
                          uint32_t id, const uint8_t *payload, uint32_t len)
{
    uint32_t i;

    qtest_writel(qts, mbox_base + 0x00, id);
    qtest_writel(qts, mbox_base + 0x04, len);
    qtest_writel(qts, mbox_base + 0x08, ADSP_IPI_OPT_DEFAULT);
    qtest_writel(qts, mbox_base + 0x0c, ADSP_IPI_MAGIC);
    for (i = 0; i < len; i += 4) {
        uint32_t w = 0;
        uint32_t j;

        for (j = 0; j < 4 && (i + j) < len; j++) {
            w |= (uint32_t)payload[i + j] << (8 * j);
        }
        qtest_writel(qts, mbox_base + MBOX_HDR_SIZE + i, w);
    }
    qtest_writel(qts, mbox_base + MBOX_OFF_SET, 1);
}

static void check_rx_header(QTestState *qts, uint64_t mbox_base,
                            uint32_t exp_id, uint32_t exp_len)
{
    g_assert_cmphex(qtest_readl(qts, mbox_base + MBOX_OFF_CLR), ==, 1);
    g_assert_cmpuint(qtest_readl(qts, mbox_base + 0x00), ==, exp_id);
    g_assert_cmpuint(qtest_readl(qts, mbox_base + 0x04), ==, exp_len);
    g_assert_cmphex(qtest_readl(qts, mbox_base + 0x08), ==,
                    ADSP_IPI_OPT_DEFAULT);
    g_assert_cmphex(qtest_readl(qts, mbox_base + 0x0c), ==, ADSP_IPI_MAGIC);
}

static void test_adsp_bootup(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");

    /*
     * LK image load clears RUNSTALL with CFG2 +0x0c bit 1 (loader mode) and
     * without arming CORE*_MEM_PWR: neither should emit a spurious READY IRQ.
     */
    qtest_writel(qts, ADSP_CFG2_BASE + CFG2_BOOT_LOADER_CTRL, 0x2);
    qtest_writel(qts, ADSP_CFG_BASE + CFG_HIFI_IO_CONFIG, 0xc0001002U);
    qtest_writel(qts, ADSP_CFG_BASE + CFG_HIFI_IO_CONFIG, 0x40001002U);
    g_assert_cmphex(qtest_readl(qts, ADSP_MBOX1_BASE + MBOX_OFF_CLR), ==, 0);
    qtest_writel(qts, ADSP_CFG2_BASE + CFG2_BOOT_LOADER_CTRL, 0x0);

    qtest_writel(qts, ADSP_CFG_BASE + CFG_CORE0_MEM_PWR, 0x0);
    qtest_writel(qts, ADSP_CFG_BASE + CFG_HIFI_IO_CONFIG, 0xc0001002U);
    qtest_writel(qts, ADSP_CFG_BASE + CFG_HIFI_IO_CONFIG, 0x40001002U);
    g_assert_cmphex(qtest_readl(qts, ADSP_MBOX1_BASE + MBOX_OFF_CLR), ==, 0);

    /* Core mem power control acks in CFG +0x4c, CFG2 retains writes. */
    qtest_writel(qts, ADSP_CFG_BASE + CFG_CORE0_MEM_PWR, 0x1f);
    g_assert_cmphex(qtest_readl(qts, ADSP_CFG_BASE + CFG_MEM_PWR_ACK), ==,
                    0x1f);
    qtest_writel(qts, ADSP_CFG_BASE + CFG_CORE1_MEM_PWR, 0x1f);
    g_assert_cmphex(qtest_readl(qts, ADSP_CFG_BASE + CFG_MEM_PWR_ACK), ==,
                    0x1f);
    qtest_writel(qts, ADSP_CFG2_BASE + 0x2118, 0xa5a55a5aU);
    g_assert_cmphex(qtest_readl(qts, ADSP_CFG2_BASE + 0x2118), ==, 0xa5a55a5aU);

    /* Stall both cores (bits 31 & 30), no ready IRQ yet. */
    qtest_writel(qts, ADSP_CFG_BASE + CFG_HIFI_IO_CONFIG, 0xc0001002U);
    g_assert_cmphex(qtest_readl(qts, ADSP_MBOX1_BASE + MBOX_OFF_CLR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ADSP_MBOX3_BASE + MBOX_OFF_CLR), ==, 0);

    /* Release Core 0 RUNSTALL (bit 31: 1 -> 0) -> READY(0) on MBOX1. */
    qtest_writel(qts, ADSP_CFG_BASE + CFG_HIFI_IO_CONFIG, 0x40001002U);
    check_rx_header(qts, ADSP_MBOX1_BASE, ADSP_IPI_ADSP_A_READY, 4);
    g_assert_cmpuint(qtest_readl(qts, ADSP_MBOX1_BASE + MBOX_HDR_SIZE), ==, 0);
    qtest_writel(qts, ADSP_MBOX1_BASE + MBOX_OFF_CLR, 1);
    g_assert_cmphex(qtest_readl(qts, ADSP_MBOX1_BASE + MBOX_OFF_CLR), ==, 0);

    /* Release Core 1 RUNSTALL (bit 30: 1 -> 0) -> READY(1) on MBOX3. */
    qtest_writel(qts, ADSP_CFG_BASE + CFG_HIFI_IO_CONFIG, 0x00001002U);
    check_rx_header(qts, ADSP_MBOX3_BASE, ADSP_IPI_ADSP_A_READY, 4);
    g_assert_cmpuint(qtest_readl(qts, ADSP_MBOX3_BASE + MBOX_HDR_SIZE), ==, 1);
    qtest_writel(qts, ADSP_MBOX3_BASE + MBOX_OFF_CLR, 1);
    g_assert_cmphex(qtest_readl(qts, ADSP_MBOX3_BASE + MBOX_OFF_CLR), ==, 0);

    qtest_quit(qts);
}

static void build_audio_pkt(uint8_t pkt[16], uint8_t scene, uint8_t layer,
                            uint8_t ack_type, uint16_t msg_id,
                            uint32_t p1, uint32_t p2)
{
    memset(pkt, 0, 16);
    pkt[0] = 0x88;
    pkt[1] = 0x88;
    pkt[2] = scene;
    pkt[3] = layer;
    pkt[4] = 0; /* data_type = MSG */
    pkt[5] = ack_type;
    pkt[6] = (uint8_t)(msg_id & 0xff);
    pkt[7] = (uint8_t)(msg_id >> 8);
    memcpy(pkt + 8, &p1, 4);
    memcpy(pkt + 12, &p2, 4);
}

static void test_adsp_audio_ipi_ack(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");
    uint8_t pkt1[16], pkt2[16];
    uint32_t w0, w1;

    /* Audio IPI with NEED_ACK (1) returns ACK_BACK (2) with swapped layer. */
    build_audio_pkt(pkt1, 0x05, 0x21, 1, 0x1234, 0xabcdef01U, 0x0badcafeU);
    adsp_send_ipi(qts, ADSP_MBOX0_BASE, ADSP_IPI_AUDIO, pkt1, sizeof(pkt1));
    g_assert_cmphex(qtest_readl(qts, ADSP_MBOX0_BASE + MBOX_OFF_SET), ==, 0);

    check_rx_header(qts, ADSP_MBOX1_BASE, ADSP_IPI_AUDIO, 16);
    w0 = qtest_readl(qts, ADSP_MBOX1_BASE + MBOX_HDR_SIZE + 0);
    w1 = qtest_readl(qts, ADSP_MBOX1_BASE + MBOX_HDR_SIZE + 4);
    g_assert_cmphex(w0, ==, 0x12058888U);
    g_assert_cmphex(w1, ==, 0x12340200U);
    g_assert_cmphex(qtest_readl(qts, ADSP_MBOX1_BASE + MBOX_HDR_SIZE + 8), ==,
                    0xabcdef01U);
    g_assert_cmphex(qtest_readl(qts, ADSP_MBOX1_BASE + MBOX_HDR_SIZE + 12), ==,
                    0x0badcafeU);
    qtest_writel(qts, ADSP_MBOX1_BASE + MBOX_OFF_CLR, 1);
    g_assert_cmphex(qtest_readl(qts, ADSP_MBOX1_BASE + MBOX_OFF_CLR), ==, 0);

    /* Back-to-back burst queues second ACK in FIFO until first is W1C. */
    build_audio_pkt(pkt1, 0x03, 0x21, 1, 0x1111, 1, 2);
    build_audio_pkt(pkt2, 0x04, 0x21, 1, 0x2222, 3, 4);
    adsp_send_ipi(qts, ADSP_MBOX0_BASE, ADSP_IPI_AUDIO, pkt1, sizeof(pkt1));
    adsp_send_ipi(qts, ADSP_MBOX0_BASE, ADSP_IPI_AUDIO, pkt2, sizeof(pkt2));

    check_rx_header(qts, ADSP_MBOX1_BASE, ADSP_IPI_AUDIO, 16);
    g_assert_cmphex(qtest_readl(qts, ADSP_MBOX1_BASE + MBOX_HDR_SIZE + 4), ==,
                    0x11110200U);
    qtest_writel(qts, ADSP_MBOX1_BASE + MBOX_OFF_CLR, 1);

    check_rx_header(qts, ADSP_MBOX1_BASE, ADSP_IPI_AUDIO, 16);
    g_assert_cmphex(qtest_readl(qts, ADSP_MBOX1_BASE + MBOX_HDR_SIZE + 4), ==,
                    0x22220200U);
    qtest_writel(qts, ADSP_MBOX1_BASE + MBOX_OFF_CLR, 1);
    g_assert_cmphex(qtest_readl(qts, ADSP_MBOX1_BASE + MBOX_OFF_CLR), ==, 0);

    /* Audio IPI with BYPASS_ACK (0) produces no RX interrupt or packet. */
    build_audio_pkt(pkt1, 0x05, 0x21, 0, 0x3333, 0, 0);
    adsp_send_ipi(qts, ADSP_MBOX0_BASE, ADSP_IPI_AUDIO, pkt1, sizeof(pkt1));
    g_assert_cmphex(qtest_readl(qts, ADSP_MBOX1_BASE + MBOX_OFF_CLR), ==, 0);

    qtest_quit(qts);
}

static void test_adsp_suspend_resume(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");
    uint8_t dummy[4] = { 0 };

    /* Initial boot of both cores */
    qtest_writel(qts, ADSP_CFG_BASE + CFG_CORE0_MEM_PWR, 0x1f);
    qtest_writel(qts, ADSP_CFG_BASE + CFG_CORE1_MEM_PWR, 0x1f);
    qtest_writel(qts, ADSP_CFG_BASE + CFG_HIFI_IO_CONFIG, 0xc0001002U);
    qtest_writel(qts, ADSP_CFG_BASE + CFG_HIFI_IO_CONFIG, 0x00001002U);
    check_rx_header(qts, ADSP_MBOX1_BASE, ADSP_IPI_ADSP_A_READY, 4);
    check_rx_header(qts, ADSP_MBOX3_BASE, ADSP_IPI_ADSP_A_READY, 4);
    qtest_writel(qts, ADSP_MBOX1_BASE + MBOX_OFF_CLR, 1);
    qtest_writel(qts, ADSP_MBOX3_BASE + MBOX_OFF_CLR, 1);

    /* Suspend Core 0 and Core 1 */
    adsp_send_ipi(qts, ADSP_MBOX0_BASE, ADSP_IPI_ADSP_A_SUSPEND, dummy, 4);
    adsp_send_ipi(qts, ADSP_MBOX2_BASE, ADSP_IPI_ADSP_A_SUSPEND, dummy, 4);

    check_rx_header(qts, ADSP_MBOX1_BASE, ADSP_IPI_ADSP_A_SUSPEND, 4);
    g_assert_cmpuint(qtest_readl(qts, ADSP_MBOX1_BASE + MBOX_HDR_SIZE), ==, 0);
    check_rx_header(qts, ADSP_MBOX3_BASE, ADSP_IPI_ADSP_A_SUSPEND, 4);
    g_assert_cmpuint(qtest_readl(qts, ADSP_MBOX3_BASE + MBOX_HDR_SIZE), ==, 1);
    qtest_writel(qts, ADSP_MBOX1_BASE + MBOX_OFF_CLR, 1);
    qtest_writel(qts, ADSP_MBOX3_BASE + MBOX_OFF_CLR, 1);

    g_assert_cmphex(qtest_readl(qts, ADSP_CFG_BASE + CFG_SLEEP_STATUS), ==,
                    0x7);
    g_assert_cmpuint(qtest_readl(qts, ADSP_C0_DTCM_BASE + DTCM_WAKELOCK_OFF),
                     ==, 1);
    g_assert_cmpuint(qtest_readl(qts, ADSP_C1_DTCM_BASE + DTCM_WAKELOCK_OFF),
                     ==, 1);

    /* Re-stall & release resumes both cores cleanly */
    qtest_writel(qts, ADSP_CFG_BASE + CFG_HIFI_IO_CONFIG, 0xc0001002U);
    qtest_writel(qts, ADSP_CFG_BASE + CFG_HIFI_IO_CONFIG, 0x00001002U);

    g_assert_cmphex(qtest_readl(qts, ADSP_CFG_BASE + CFG_SLEEP_STATUS), ==,
                    0x0);
    g_assert_cmpuint(qtest_readl(qts, ADSP_C0_DTCM_BASE + DTCM_WAKELOCK_OFF),
                     ==, 0);
    g_assert_cmpuint(qtest_readl(qts, ADSP_C1_DTCM_BASE + DTCM_WAKELOCK_OFF),
                     ==, 0);

    check_rx_header(qts, ADSP_MBOX1_BASE, ADSP_IPI_ADSP_A_READY, 4);
    g_assert_cmpuint(qtest_readl(qts, ADSP_MBOX1_BASE + MBOX_HDR_SIZE), ==, 0);
    check_rx_header(qts, ADSP_MBOX3_BASE, ADSP_IPI_ADSP_A_READY, 4);
    g_assert_cmpuint(qtest_readl(qts, ADSP_MBOX3_BASE + MBOX_HDR_SIZE), ==, 1);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/mt6895/adsp/bootup", test_adsp_bootup);
    qtest_add_func("/mt6895/adsp/audio-ipi-ack", test_adsp_audio_ipi_ack);
    qtest_add_func("/mt6895/adsp/suspend-resume", test_adsp_suspend_resume);
    return g_test_run();
}
