/*
 * MT6895 tinysys mailbox SCMI protocol tests.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define SCMI_TX_SHMEM       0x1c350000
#define SCMI_MBOX_TX        0x1c351000
#define SCMI_MBOX_RX        0x1c361000

#define OFF_STATUS          0x04
#define OFF_FLAGS           0x10
#define OFF_LENGTH          0x14
#define OFF_HDR             0x18
#define OFF_PAYLOAD         0x1c

#define SCMI_PROTO_BASE     0x10
#define SCMI_PROTO_TINYSYS  0x80

static void scmi_xfer(QTestState *qts, uint32_t proto, uint32_t msg_id,
                      uint32_t token, uint32_t flags,
                      const uint32_t *req_words, size_t nwords)
{
    uint32_t hdr = (msg_id & 0xff) | ((proto & 0xff) << 10) |
                   ((token & 0x3ff) << 18);
    size_t i;

    qtest_writel(qts, SCMI_TX_SHMEM + OFF_STATUS, 0);
    qtest_writel(qts, SCMI_TX_SHMEM + OFF_FLAGS, flags);
    qtest_writel(qts, SCMI_TX_SHMEM + OFF_LENGTH, 4 + 4 * (uint32_t)nwords);
    qtest_writel(qts, SCMI_TX_SHMEM + OFF_HDR, hdr);
    for (i = 0; i < nwords; i++) {
        qtest_writel(qts, SCMI_TX_SHMEM + OFF_PAYLOAD + 4 * i, req_words[i]);
    }
    qtest_writel(qts, SCMI_MBOX_TX + 0x00, 1);
}

static void test_scmi_base_protocol(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");
    uint32_t hdr, len, status, skip;
    char vendor[5] = { 0 };
    uint32_t vword;

    g_assert_cmphex(qtest_readl(qts, SCMI_TX_SHMEM + OFF_STATUS), ==, 1);

    /* Keep driver profiling area at +0x70/+0x74 intact across replies. */
    qtest_writel(qts, SCMI_TX_SHMEM + 0x70, 0x11223344);
    qtest_writel(qts, SCMI_TX_SHMEM + 0x74, 0x55667788);

    /* Base PROTOCOL_VERSION (msg 0) */
    scmi_xfer(qts, SCMI_PROTO_BASE, 0x0, 7, 1, NULL, 0);
    g_assert_cmphex(qtest_readl(qts, SCMI_MBOX_TX + 0x00), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SCMI_MBOX_TX + OFF_STATUS), ==, 1);
    qtest_writel(qts, SCMI_MBOX_TX + OFF_STATUS, 1);
    g_assert_cmphex(qtest_readl(qts, SCMI_MBOX_TX + OFF_STATUS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SCMI_MBOX_RX + OFF_STATUS), ==, 0);

    g_assert_cmphex(qtest_readl(qts, SCMI_TX_SHMEM + OFF_STATUS), ==, 1);
    len = qtest_readl(qts, SCMI_TX_SHMEM + OFF_LENGTH);
    hdr = qtest_readl(qts, SCMI_TX_SHMEM + OFF_HDR);
    status = qtest_readl(qts, SCMI_TX_SHMEM + OFF_PAYLOAD);
    g_assert_cmpuint(len, ==, 12);
    g_assert_cmphex(hdr, ==, (SCMI_PROTO_BASE << 10) | (7u << 18));
    g_assert_cmphex(status, ==, 0);
    g_assert_cmphex(qtest_readl(qts, SCMI_TX_SHMEM + OFF_PAYLOAD + 4), ==,
                    0x00020000);
    g_assert_cmphex(qtest_readl(qts, SCMI_TX_SHMEM + 0x70), ==, 0x11223344);
    g_assert_cmphex(qtest_readl(qts, SCMI_TX_SHMEM + 0x74), ==, 0x55667788);

    /* Base PROTOCOL_ATTRIBUTES (msg 1): 1 non-base protocol, 1 agent */
    scmi_xfer(qts, SCMI_PROTO_BASE, 0x1, 8, 1, NULL, 0);
    qtest_writel(qts, SCMI_MBOX_TX + OFF_STATUS, 1);
    g_assert_cmphex(qtest_readl(qts, SCMI_TX_SHMEM + OFF_PAYLOAD), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SCMI_TX_SHMEM + OFF_PAYLOAD + 4), ==,
                    1 | (1u << 8));

    /* Base DISCOVER_VENDOR (msg 3) */
    scmi_xfer(qts, SCMI_PROTO_BASE, 0x3, 9, 1, NULL, 0);
    qtest_writel(qts, SCMI_MBOX_TX + OFF_STATUS, 1);
    g_assert_cmpuint(qtest_readl(qts, SCMI_TX_SHMEM + OFF_LENGTH), ==, 24);
    vword = qtest_readl(qts, SCMI_TX_SHMEM + OFF_PAYLOAD + 4);
    memcpy(vendor, &vword, 4);
    g_assert_cmpstr(vendor, ==, "MTK");

    /* Base DISCOVER_LIST_PROTOCOLS (msg 6) */
    skip = 0;
    scmi_xfer(qts, SCMI_PROTO_BASE, 0x6, 10, 1, &skip, 1);
    qtest_writel(qts, SCMI_MBOX_TX + OFF_STATUS, 1);
    g_assert_cmpuint(qtest_readl(qts, SCMI_TX_SHMEM + OFF_LENGTH), ==, 16);
    g_assert_cmphex(qtest_readl(qts, SCMI_TX_SHMEM + OFF_PAYLOAD), ==, 0);
    g_assert_cmpuint(qtest_readl(qts, SCMI_TX_SHMEM + OFF_PAYLOAD + 4), ==, 1);
    g_assert_cmphex(qtest_readl(qts, SCMI_TX_SHMEM + OFF_PAYLOAD + 8) & 0xff,
                    ==, SCMI_PROTO_TINYSYS);

    skip = 1;
    scmi_xfer(qts, SCMI_PROTO_BASE, 0x6, 11, 1, &skip, 1);
    qtest_writel(qts, SCMI_MBOX_TX + OFF_STATUS, 1);
    g_assert_cmpuint(qtest_readl(qts, SCMI_TX_SHMEM + OFF_LENGTH), ==, 12);
    g_assert_cmpuint(qtest_readl(qts, SCMI_TX_SHMEM + OFF_PAYLOAD + 4), ==, 0);

    qtest_quit(qts);
}

static void test_scmi_tinysys_protocol(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");
    uint32_t hdr, len;
    uint32_t set_req[7] = { 7, 0x504c5401, 0x7c4f0000, 0x10000, 0, 0, 0 };
    uint32_t get_req[3] = { 7, 0x504c5401, 1 };

    /* Tinysys PROTOCOL_VERSION (msg 0): bit 17 of 0x80 must survive header */
    scmi_xfer(qts, SCMI_PROTO_TINYSYS, 0x0, 12, 1, NULL, 0);
    g_assert_cmphex(qtest_readl(qts, SCMI_MBOX_TX + OFF_STATUS), ==, 1);
    qtest_writel(qts, SCMI_MBOX_TX + OFF_STATUS, 1);
    len = qtest_readl(qts, SCMI_TX_SHMEM + OFF_LENGTH);
    hdr = qtest_readl(qts, SCMI_TX_SHMEM + OFF_HDR);
    g_assert_cmpuint(len, ==, 12);
    g_assert_cmphex((hdr >> 10) & 0xff, ==, SCMI_PROTO_TINYSYS);
    g_assert_cmpuint((hdr >> 18) & 0x3ff, ==, 12);
    g_assert_cmpuint((hdr >> 8) & 0x3, ==, 0);
    g_assert_cmphex(qtest_readl(qts, SCMI_TX_SHMEM + OFF_PAYLOAD), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SCMI_TX_SHMEM + OFF_PAYLOAD + 4), ==,
                    0x00010000);

    /* Tinysys PROTOCOL_ATTRIBUTES (msg 1) in polling mode (flags == 0) */
    scmi_xfer(qts, SCMI_PROTO_TINYSYS, 0x1, 13, 0, NULL, 0);
    g_assert_cmphex(qtest_readl(qts, SCMI_MBOX_TX + OFF_STATUS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SCMI_TX_SHMEM + OFF_STATUS), ==, 1);
    g_assert_cmpuint(qtest_readl(qts, SCMI_TX_SHMEM + OFF_LENGTH), ==, 12);
    g_assert_cmpuint(qtest_readl(qts, SCMI_TX_SHMEM + OFF_PAYLOAD + 4), ==, 20);

    /* Tinysys COMMON_SET (msg 3) */
    scmi_xfer(qts, SCMI_PROTO_TINYSYS, 0x3, 14, 1, set_req, 7);
    qtest_writel(qts, SCMI_MBOX_TX + OFF_STATUS, 1);
    g_assert_cmpuint(qtest_readl(qts, SCMI_TX_SHMEM + OFF_LENGTH), ==, 8);
    g_assert_cmphex(qtest_readl(qts, SCMI_TX_SHMEM + OFF_PAYLOAD), ==, 0);

    /* Tinysys COMMON_GET (msg 4) */
    scmi_xfer(qts, SCMI_PROTO_TINYSYS, 0x4, 15, 1, get_req, 3);
    qtest_writel(qts, SCMI_MBOX_TX + OFF_STATUS, 1);
    g_assert_cmpuint(qtest_readl(qts, SCMI_TX_SHMEM + OFF_LENGTH), ==, 20);
    g_assert_cmphex(qtest_readl(qts, SCMI_TX_SHMEM + OFF_PAYLOAD), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SCMI_TX_SHMEM + OFF_PAYLOAD + 4), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SCMI_TX_SHMEM + OFF_PAYLOAD + 8), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SCMI_TX_SHMEM + OFF_PAYLOAD + 12), ==, 0);

    /* Tinysys EVENT_NOTIFY (msg 5) */
    scmi_xfer(qts, SCMI_PROTO_TINYSYS, 0x5, 16, 1, get_req, 3);
    qtest_writel(qts, SCMI_MBOX_TX + OFF_STATUS, 1);
    g_assert_cmpuint(qtest_readl(qts, SCMI_TX_SHMEM + OFF_LENGTH), ==, 8);
    g_assert_cmphex(qtest_readl(qts, SCMI_TX_SHMEM + OFF_PAYLOAD), ==, 0);

    qtest_quit(qts);
}

static void test_scmi_reset(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");

    scmi_xfer(qts, SCMI_PROTO_BASE, 0x0, 1, 1, NULL, 0);
    g_assert_cmphex(qtest_readl(qts, SCMI_MBOX_TX + OFF_STATUS), ==, 1);

    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, SCMI_MBOX_TX + OFF_STATUS), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/mt6895/scmi/base-protocol", test_scmi_base_protocol);
    qtest_add_func("/mt6895/scmi/tinysys-protocol", test_scmi_tinysys_protocol);
    qtest_add_func("/mt6895/scmi/reset", test_scmi_reset);
    return g_test_run();
}
