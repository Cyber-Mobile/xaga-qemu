/*
 * MT6895 GPUEB boot handshake and mbox0 IPI responder QTests.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define GPUEB_BOOT_BASE     0x13c5fd24ULL
#define GPUEB_START_OFS     0x8dc
#define GPUEB_BOOT_MAGIC    0x55667788u
#define GPUEB_MBOX0_BASE    0x13c5fd80ULL
#define GPUEB_IRQ_BASE      0x13c62000ULL

#define MBOX0_SEND_OFF      0x00
#define MBOX0_SET_OFF       0x04
#define MBOX0_CLR_OFF       0x74
#define MBOX0_RECV_OFF      0x78

/* Pin 1 (IPI_ID_GPUFREQ): send_slot=4 (+0x10), recv_slot=47 (+0xbc) */
#define GPUFREQ_SEND_BASE   (GPUEB_MBOX0_BASE + 4 * 4)
#define GPUFREQ_RECV_BASE   (GPUEB_MBOX0_BASE + 47 * 4)
/* Pin 7 (IPI_ID_FAST_DVFS): recv_slot=63 (+0xfc) */
#define FASTDVFS_RECV_BASE  (GPUEB_MBOX0_BASE + 63 * 4)

#define SHMEM_STATUS_BASE   0x42000000ULL
#define SHMEM_DEBUG_BASE    0x42000400ULL

static void send_gpufreq_cmd(QTestState *qts, uint32_t cmd, uint32_t target,
                             uint32_t arg2, uint32_t arg3, uint32_t arg4,
                             uint32_t arg5, uint32_t arg6, uint32_t arg7)
{
    qtest_writel(qts, GPUFREQ_SEND_BASE + 0x00, cmd);
    qtest_writel(qts, GPUFREQ_SEND_BASE + 0x04, target);
    qtest_writel(qts, GPUFREQ_SEND_BASE + 0x08, arg2);
    qtest_writel(qts, GPUFREQ_SEND_BASE + 0x0c, arg3);
    qtest_writel(qts, GPUFREQ_SEND_BASE + 0x10, arg4);
    qtest_writel(qts, GPUFREQ_SEND_BASE + 0x14, arg5);
    qtest_writel(qts, GPUFREQ_SEND_BASE + 0x18, arg6);
    qtest_writel(qts, GPUFREQ_SEND_BASE + 0x1c, arg7);
    qtest_writel(qts, GPUEB_IRQ_BASE + MBOX0_SET_OFF, 1u << 1);
}

static void test_gpueb_boot_and_ipi(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");
    uint32_t ack_ret;

    /* 1. LK boot magic handshake + 64-bit store at +0x4 (8B-aligned PA 0x13c5fd28) */
    qtest_writeq(qts, GPUEB_BOOT_BASE + 4, 0x1122334455667788ULL);
    g_assert_cmphex(qtest_readq(qts, GPUEB_BOOT_BASE + 4), ==,
                    0x1122334455667788ULL);
    qtest_writeq(qts, GPUEB_BOOT_BASE + 4, 0);
    g_assert_cmphex(qtest_readl(qts, GPUEB_BOOT_BASE), !=, GPUEB_BOOT_MAGIC);
    qtest_writel(qts, GPUEB_BOOT_BASE + GPUEB_START_OFS, 3);
    g_assert_cmphex(qtest_readl(qts, GPUEB_BOOT_BASE), ==, GPUEB_BOOT_MAGIC);

    /* 2. CMD_INIT_SHARED_MEM (0): populates shared_status in guest RAM */
    send_gpufreq_cmd(qts, 0, 0,
                     (uint32_t)SHMEM_STATUS_BASE, 0,
                     (uint32_t)SHMEM_DEBUG_BASE, 0,
                     0x400, 0xc00);
    g_assert_cmphex(qtest_readl(qts, GPUEB_IRQ_BASE + MBOX0_SEND_OFF), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPUEB_IRQ_BASE + MBOX0_RECV_OFF), ==,
                    1u << 1);
    g_assert_cmpuint(qtest_readl(qts, GPUFREQ_RECV_BASE + 0x00), ==, 0);
    g_assert_cmpuint(qtest_readl(qts, GPUFREQ_RECV_BASE + 0x08), ==, 0);

    g_assert_cmpuint(qtest_readl(qts, SHMEM_STATUS_BASE + 0x08), ==, 8);
    g_assert_cmpuint(qtest_readl(qts, SHMEM_STATUS_BASE + 0x0c), ==, 8);
    g_assert_cmpuint(qtest_readl(qts, SHMEM_STATUS_BASE + 0x10), ==, 886000);
    g_assert_cmphex(qtest_readl(qts, SHMEM_STATUS_BASE + 0x68), ==, 0x3f);
    g_assert_cmpuint(qtest_readl(qts, SHMEM_STATUS_BASE + 0x6c), ==, 1);

    /* W1C clear recv IRQ */
    qtest_writel(qts, GPUEB_IRQ_BASE + MBOX0_CLR_OFF, 1u << 1);
    g_assert_cmphex(qtest_readl(qts, GPUEB_IRQ_BASE + MBOX0_RECV_OFF), ==, 0);

    /* 3. CMD_GET_FREQ_BY_IDX (1) & CMD_GET_WORKING_TABLE (10) */
    send_gpufreq_cmd(qts, 1, 1, 2, 0, 0, 0, 0, 0);
    ack_ret = qtest_readl(qts, GPUFREQ_RECV_BASE + 0x08);
    g_assert_cmpuint(ack_ret, ==, 700000);
    qtest_writel(qts, GPUEB_IRQ_BASE + MBOX0_CLR_OFF, 1u << 1);

    send_gpufreq_cmd(qts, 10, 1, 0, 0, 0, 0, 0, 0);
    g_assert_cmpuint(qtest_readl(qts, GPUFREQ_RECV_BASE + 0x08), ==, 0);
    g_assert_cmpuint(qtest_readl(qts, SHMEM_DEBUG_BASE + 0x00), ==, 886000);
    g_assert_cmpuint(qtest_readl(qts, SHMEM_DEBUG_BASE + 0x14), ==, 2200);
    g_assert_cmpuint(qtest_readl(qts, SHMEM_DEBUG_BASE + 7 * 24 + 0x00), ==,
                     218000);
    qtest_writel(qts, GPUEB_IRQ_BASE + MBOX0_CLR_OFF, 1u << 1);

    /* 4. CMD_COMMIT (7) & CMD_POWER_CONTROL (6) update shared status */
    send_gpufreq_cmd(qts, 7, 1, 4, 0, 0, 0, 0, 0);
    g_assert_cmpuint(qtest_readl(qts, GPUFREQ_RECV_BASE + 0x08), ==, 0);
    g_assert_cmpuint(qtest_readl(qts, SHMEM_STATUS_BASE + 0x00), ==, 4);
    g_assert_cmpuint(qtest_readl(qts, SHMEM_STATUS_BASE + 0x10), ==, 500000);
    qtest_writel(qts, GPUEB_IRQ_BASE + MBOX0_CLR_OFF, 1u << 1);

    send_gpufreq_cmd(qts, 6, 0, 1, 0, 0, 0, 0, 0);
    g_assert_cmpuint(qtest_readl(qts, GPUFREQ_RECV_BASE + 0x08), ==, 2);
    g_assert_cmpuint(qtest_readl(qts, SHMEM_STATUS_BASE + 0x6c), ==, 2);
    qtest_writel(qts, GPUEB_IRQ_BASE + MBOX0_CLR_OFF, 1u << 1);

    /* 5. Generic ACK pin 7 (IPI_ID_FAST_DVFS) */
    qtest_writel(qts, FASTDVFS_RECV_BASE, 0x12345678u);
    qtest_writel(qts, GPUEB_IRQ_BASE + MBOX0_SET_OFF, 1u << 7);
    g_assert_cmphex(qtest_readl(qts, GPUEB_IRQ_BASE + MBOX0_RECV_OFF), ==,
                    1u << 7);
    g_assert_cmpuint(qtest_readl(qts, FASTDVFS_RECV_BASE), ==, 0);
    qtest_writel(qts, GPUEB_IRQ_BASE + MBOX0_CLR_OFF, 1u << 7);
    g_assert_cmphex(qtest_readl(qts, GPUEB_IRQ_BASE + MBOX0_RECV_OFF), ==, 0);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/mt6895/gpueb/boot_and_ipi", test_gpueb_boot_and_ipi);
    return g_test_run();
}
