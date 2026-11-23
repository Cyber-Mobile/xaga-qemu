/*
 * MT6895 MT6375 sub-PMIC / TCPCI / ADC, I2C APDMA, and USB1 IPPC tests.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define I2C5_BASE           0x11280000ULL
#define I2C5_DMA_BASE       0x11300400ULL
#define USB1_IPPC_BASE      0x11213e00ULL
#define DRAM_TX_BUF         0x40100000ULL
#define DRAM_RX_BUF         0x40101000ULL

#define R_DATA_PORT         0x00
#define R_INTR_STAT         0x0c
#define R_CONTROL           0x10
#define R_TRANSFER_LEN      0x14
#define R_TRANSAC_LEN       0x18
#define R_START             0x24
#define R_FIFO_ADDR_CLR     0x38
#define R_TRANSFER_LEN_AUX  0x44
#define R_SLAVE_ADDR        0x94

#define INTR_COMP           (1u << 0)
#define INTR_ACKERR         (1u << 1)
#define CTL_DMA_EN          (1u << 2)
#define CTL_DIR_CHANGE      (1u << 4)

#define DMA_INT_FLAG        0x00
#define DMA_EN              0x08
#define DMA_CON             0x18
#define DMA_TX_ADDR         0x1c
#define DMA_RX_ADDR         0x20
#define DMA_TX_LEN          0x24
#define DMA_RX_LEN          0x28
#define DMA_TX_4G           0x54
#define DMA_RX_4G           0x58

#define IPPC_CLK_STS0       0x10
#define IPPC_CLK_STS1       0x14

static void i2c5_fifo_wr(QTestState *qts, uint8_t addr7,
                         const uint8_t *tx, unsigned txlen)
{
    unsigned i;

    qtest_writel(qts, I2C5_BASE + R_FIFO_ADDR_CLR, 1);
    qtest_writel(qts, I2C5_BASE + R_INTR_STAT, 0xffff);
    qtest_writel(qts, I2C5_BASE + R_SLAVE_ADDR, (uint32_t)addr7 << 1);
    qtest_writel(qts, I2C5_BASE + R_CONTROL, 0);
    qtest_writel(qts, I2C5_BASE + R_TRANSFER_LEN, txlen);
    qtest_writel(qts, I2C5_BASE + R_TRANSAC_LEN, 1);
    for (i = 0; i < txlen; i++) {
        qtest_writel(qts, I2C5_BASE + R_DATA_PORT, tx[i]);
    }
    qtest_writel(qts, I2C5_BASE + R_START, 1);
    g_assert_cmphex(qtest_readl(qts, I2C5_BASE + R_INTR_STAT) &
                    (INTR_COMP | INTR_ACKERR), ==, INTR_COMP);
    qtest_writel(qts, I2C5_BASE + R_INTR_STAT, 0xffff);
}

static void i2c5_fifo_wr_rd(QTestState *qts, uint8_t addr7,
                            const uint8_t *tx, unsigned txlen,
                            uint8_t *rx, unsigned rxlen)
{
    unsigned i;

    qtest_writel(qts, I2C5_BASE + R_FIFO_ADDR_CLR, 1);
    qtest_writel(qts, I2C5_BASE + R_INTR_STAT, 0xffff);
    qtest_writel(qts, I2C5_BASE + R_SLAVE_ADDR, (uint32_t)addr7 << 1);
    qtest_writel(qts, I2C5_BASE + R_CONTROL, CTL_DIR_CHANGE);
    qtest_writel(qts, I2C5_BASE + R_TRANSFER_LEN, txlen);
    qtest_writel(qts, I2C5_BASE + R_TRANSFER_LEN_AUX, rxlen);
    qtest_writel(qts, I2C5_BASE + R_TRANSAC_LEN, 2);
    for (i = 0; i < txlen; i++) {
        qtest_writel(qts, I2C5_BASE + R_DATA_PORT, tx[i]);
    }
    qtest_writel(qts, I2C5_BASE + R_START, 1);
    g_assert_cmphex(qtest_readl(qts, I2C5_BASE + R_INTR_STAT) &
                    (INTR_COMP | INTR_ACKERR), ==, INTR_COMP);
    for (i = 0; i < rxlen; i++) {
        rx[i] = (uint8_t)qtest_readl(qts, I2C5_BASE + R_DATA_PORT);
    }
    qtest_writel(qts, I2C5_BASE + R_INTR_STAT, 0xffff);
}

static void test_mt6375_fifo_and_regs(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");
    uint8_t reg;
    uint8_t buf[6];
    uint8_t ext_wr[3] = { 0xf2, 0x37, 0xa5 };
    uint8_t ext_rd[2] = { 0xf2, 0x37 };
    uint8_t adc_cmd[2] = { 0xa6, 0x10 };
    uint8_t sbu_pd[2] = { 0xd2, 0x03 };
    uint8_t sbu_pu[2] = { 0xd2, 0x05 };

    /* Bank 1 (0x34): DEV_INFO = 0x72, STAT0 F_ST_PWR_RDY = 1 */
    reg = 0x00;
    i2c5_fifo_wr_rd(qts, 0x34, &reg, 1, buf, 1);
    g_assert_cmphex(buf[0], ==, 0x72);
    reg = 0xe0;
    i2c5_fifo_wr_rd(qts, 0x34, &reg, 1, buf, 1);
    g_assert_cmphex(buf[0] & 1, ==, 1);

    /* Bank 0 (0x4e): TCPCI VID 0x29cf, PID 0x6375, CC_STATUS, POWER_STATUS */
    reg = 0x00;
    i2c5_fifo_wr_rd(qts, 0x4e, &reg, 1, buf, 6);
    g_assert_cmphex(buf[0] | (buf[1] << 8), ==, 0x29cf);
    g_assert_cmphex(buf[2] | (buf[3] << 8), ==, 0x6375);
    reg = 0x1d;
    i2c5_fifo_wr_rd(qts, 0x4e, &reg, 1, buf, 2);
    g_assert_cmphex(buf[0], ==, 0x11);
    g_assert_cmphex(buf[1], ==, 0x0c);
    reg = 0x9f;
    i2c5_fifo_wr_rd(qts, 0x4e, &reg, 1, buf, 1);
    g_assert_cmphex(buf[0], ==, 0x00);

    /* Extended [0xf2, subreg] page isolates from normal Bank 0 reg[0x37] */
    i2c5_fifo_wr(qts, 0x4e, ext_wr, sizeof(ext_wr));
    i2c5_fifo_wr_rd(qts, 0x4e, ext_rd, sizeof(ext_rd), buf, 1);
    g_assert_cmphex(buf[0], ==, 0xa5);
    reg = 0x37;
    i2c5_fifo_wr_rd(qts, 0x4e, &reg, 1, buf, 1);
    g_assert_cmphex(buf[0], ==, 0x00);

    /* One-shot ADC trigger self-clears and reports BE16 ch 1 VBUS (800) */
    i2c5_fifo_wr(qts, 0x34, adc_cmd, sizeof(adc_cmd));
    reg = 0xa6;
    i2c5_fifo_wr_rd(qts, 0x34, &reg, 1, buf, 1);
    g_assert_cmphex(buf[0], ==, 0x00);
    reg = 0xaa;
    i2c5_fifo_wr_rd(qts, 0x34, &reg, 1, buf, 2);
    g_assert_cmphex(((uint16_t)buf[0] << 8) | buf[1], ==, 800);

    /* Water-detect SBU1 pull-down vs pull-up (0 vs 480 BE16) */
    i2c5_fifo_wr(qts, 0x4e, sbu_pd, sizeof(sbu_pd));
    reg = 0xc4;
    i2c5_fifo_wr_rd(qts, 0x34, &reg, 1, buf, 2);
    g_assert_cmphex(((uint16_t)buf[0] << 8) | buf[1], ==, 0);
    i2c5_fifo_wr(qts, 0x4e, sbu_pu, sizeof(sbu_pu));
    i2c5_fifo_wr_rd(qts, 0x34, &reg, 1, buf, 2);
    g_assert_cmphex(((uint16_t)buf[0] << 8) | buf[1], ==, 480);

    qtest_quit(qts);
}

static void test_mt6375_apdma_and_usb1_ippc(void)
{
    QTestState *qts = qtest_init("-machine mt6895 -m 256M -smp 1 -nodefaults");
    uint8_t unmask_evt[17];
    uint8_t evt_rx[16];
    uint8_t clr_evt[17];
    uint8_t reg = 0xd0;
    unsigned i;

    /* Trigger adc_donei first while masked, then unmask via 16-byte APDMA */
    unmask_evt[0] = 0xa6;
    unmask_evt[1] = 0x10;
    i2c5_fifo_wr(qts, 0x34, unmask_evt, 2);

    memset(unmask_evt, 0, sizeof(unmask_evt));
    unmask_evt[0] = 0xf0;
    qtest_memwrite(qts, DRAM_TX_BUF, unmask_evt, sizeof(unmask_evt));

    qtest_writel(qts, I2C5_BASE + R_INTR_STAT, 0xffff);
    qtest_writel(qts, I2C5_BASE + R_SLAVE_ADDR, 0x34 << 1);
    qtest_writel(qts, I2C5_BASE + R_CONTROL, CTL_DMA_EN);
    qtest_writel(qts, I2C5_DMA_BASE + DMA_INT_FLAG, 0);
    qtest_writel(qts, I2C5_DMA_BASE + DMA_CON, 0);
    qtest_writel(qts, I2C5_DMA_BASE + DMA_TX_ADDR, (uint32_t)DRAM_TX_BUF);
    qtest_writel(qts, I2C5_DMA_BASE + DMA_TX_4G, 0);
    qtest_writel(qts, I2C5_DMA_BASE + DMA_TX_LEN, sizeof(unmask_evt));
    qtest_writel(qts, I2C5_DMA_BASE + DMA_EN, 1);
    qtest_writel(qts, I2C5_BASE + R_START, 1);

    g_assert_cmphex(qtest_readl(qts, I2C5_DMA_BASE + DMA_EN) & 1, ==, 0);
    g_assert_cmphex(qtest_readl(qts, I2C5_BASE + R_INTR_STAT) &
                    (INTR_COMP | INTR_ACKERR), ==, INTR_COMP);

    /* Bulk read 16 IRQ_EVT bytes (0xd0..0xdf) with APDMA DIR_CHANGE */
    memset(evt_rx, 0, sizeof(evt_rx));
    qtest_memwrite(qts, DRAM_RX_BUF, evt_rx, sizeof(evt_rx));
    qtest_memwrite(qts, DRAM_TX_BUF, &reg, 1);
    qtest_writel(qts, I2C5_BASE + R_INTR_STAT, 0xffff);
    qtest_writel(qts, I2C5_BASE + R_SLAVE_ADDR, 0x34 << 1);
    qtest_writel(qts, I2C5_BASE + R_CONTROL, CTL_DMA_EN | CTL_DIR_CHANGE);
    qtest_writel(qts, I2C5_DMA_BASE + DMA_TX_ADDR, (uint32_t)DRAM_TX_BUF);
    qtest_writel(qts, I2C5_DMA_BASE + DMA_RX_ADDR, (uint32_t)DRAM_RX_BUF);
    qtest_writel(qts, I2C5_DMA_BASE + DMA_TX_LEN, 1);
    qtest_writel(qts, I2C5_DMA_BASE + DMA_RX_LEN, sizeof(evt_rx));
    qtest_writel(qts, I2C5_DMA_BASE + DMA_EN, 1);
    qtest_writel(qts, I2C5_BASE + R_START, 1);

    g_assert_cmphex(qtest_readl(qts, I2C5_DMA_BASE + DMA_EN) & 1, ==, 0);
    g_assert_cmphex(qtest_readl(qts, I2C5_BASE + R_INTR_STAT) &
                    (INTR_COMP | INTR_ACKERR), ==, INTR_COMP);
    qtest_memread(qts, DRAM_RX_BUF, evt_rx, sizeof(evt_rx));
    g_assert_cmphex(evt_rx[0] & 0x01, ==, 0x01); /* fl_pwr_rdy */
    g_assert_cmphex(evt_rx[5] & 0x10, ==, 0x10); /* adc_donei */

    /* W1C clear all events via APDMA */
    clr_evt[0] = 0xd0;
    for (i = 0; i < 16; i++) {
        clr_evt[1 + i] = 0xff;
    }
    qtest_memwrite(qts, DRAM_TX_BUF, clr_evt, sizeof(clr_evt));
    qtest_writel(qts, I2C5_BASE + R_CONTROL, CTL_DMA_EN);
    qtest_writel(qts, I2C5_DMA_BASE + DMA_TX_LEN, sizeof(clr_evt));
    qtest_writel(qts, I2C5_DMA_BASE + DMA_EN, 1);
    qtest_writel(qts, I2C5_BASE + R_START, 1);

    qtest_memwrite(qts, DRAM_TX_BUF, &reg, 1);
    qtest_writel(qts, I2C5_BASE + R_CONTROL, CTL_DMA_EN | CTL_DIR_CHANGE);
    qtest_writel(qts, I2C5_DMA_BASE + DMA_TX_LEN, 1);
    qtest_writel(qts, I2C5_DMA_BASE + DMA_RX_LEN, sizeof(evt_rx));
    qtest_writel(qts, I2C5_DMA_BASE + DMA_EN, 1);
    qtest_writel(qts, I2C5_BASE + R_START, 1);
    qtest_memread(qts, DRAM_RX_BUF, evt_rx, sizeof(evt_rx));
    for (i = 0; i < 16; i++) {
        g_assert_cmphex(evt_rx[i], ==, 0x00);
    }

    /* USB1 IPPC clock-stable glue */
    g_assert_cmphex(qtest_readl(qts, USB1_IPPC_BASE + IPPC_CLK_STS0), ==,
                    0xffffffffu);
    g_assert_cmphex(qtest_readl(qts, USB1_IPPC_BASE + IPPC_CLK_STS1) & 1u, ==,
                    1u);

    /*
     * TCPM_CMD_LOOK_CONNECTION (0x23 = 0x99) + EINT-E (0x11ce0000) level
     * latching on bit 18 across ACK / MASK_CLR, cleared by 0x1df W1C.
     */
    {
        const uint64_t i2c5 = 0x11280000ULL;
        const uint64_t einte = 0x11ce0000ULL;
        const uint32_t bit18 = 1u << 18;
        const struct {
            uint8_t addr;
            uint8_t reg;
            uint8_t val;
            uint8_t mask;
            bool is_read;
        } seq[] = {
            /* Ensure TCPC ALERT (bank 0 @ 0x4e) and PMU IRQ (bank 1 @ 0x34) are unmasked and cleared. */
            { 0x4e, 0x12, 0xff, 0x00, false },
            { 0x34, 0xff, 0x00, 0x00, false },
            { 0x4e, 0x10, 0xff, 0x00, false },
            { 0x34, 0xdf, 0xff, 0x00, false },
            /* Configure EINT-E bit 18 as level-sensitive and verify clean. */
            { 0x00, 0x00, 0x00, 0x00, false }, /* marker: setup EINT */
            /* Trigger TCPM_CMD_LOOK_CONNECTION -> re-latch ALERT & 0x1df. */
            { 0x4e, 0x23, 0x99, 0x00, false },
            { 0x4e, 0x10, 0x03, 0x03, true },
            { 0x34, 0xdf, 0x01, 0x01, true },
            /* Marker: verify EINT bit 18 stays latched across ACK/MASK_CLR. */
            { 0x00, 0x01, 0x00, 0x00, false },
            /* W1C clear TCPC ALERT & PMU 0x1df and confirm deassertion. */
            { 0x4e, 0x10, 0x03, 0x00, false },
            { 0x34, 0xdf, 0x01, 0x00, false },
            { 0x34, 0xdf, 0x00, 0x01, true },
        };

        for (size_t step = 0; step < G_N_ELEMENTS(seq); step++) {
            if (seq[step].addr == 0x00 && seq[step].reg == 0x00) {
                qtest_writel(qts, einte + 0x180, bit18); /* SENS_SET */
                g_assert_cmphex(qtest_readl(qts, einte + 0x140) & bit18, ==,
                                bit18);
                qtest_writel(qts, einte + 0x100, bit18); /* MASK_CLR */
                qtest_writel(qts, einte + 0x040, bit18); /* ACK */
                g_assert_cmphex(qtest_readl(qts, einte + 0x000) & bit18, ==,
                                0u);
                continue;
            }
            if (seq[step].addr == 0x00 && seq[step].reg == 0x01) {
                g_assert_cmphex(qtest_readl(qts, einte + 0x000) & bit18, ==,
                                bit18);
                qtest_writel(qts, einte + 0x040, bit18); /* ACK while active */
                g_assert_cmphex(qtest_readl(qts, einte + 0x000) & bit18, ==,
                                bit18);
                qtest_writel(qts, einte + 0x0c0, bit18); /* MASK_SET */
                qtest_writel(qts, einte + 0x040, bit18); /* ACK while masked */
                g_assert_cmphex(qtest_readl(qts, einte + 0x000) & bit18, ==,
                                0u);
                g_assert_cmphex(qtest_readl(qts, einte + 0xa00) & bit18, ==,
                                bit18);
                qtest_writel(qts, einte + 0x100, bit18); /* MASK_CLR */
                g_assert_cmphex(qtest_readl(qts, einte + 0x000) & bit18, ==,
                                bit18);
                continue;
            }

            qtest_writel(qts, i2c5 + 0x0c, 0x7);
            qtest_writel(qts, i2c5 + 0x38, 0x1);
            qtest_writel(qts, i2c5 + 0x94, (uint32_t)seq[step].addr << 1);
            if (seq[step].is_read) {
                uint32_t v;
                qtest_writel(qts, i2c5 + 0x10, 0x12);
                qtest_writel(qts, i2c5 + 0x14, 1);
                qtest_writel(qts, i2c5 + 0x44, 1);
                qtest_writel(qts, i2c5 + 0x18, 2);
                qtest_writel(qts, i2c5 + 0x00, seq[step].reg);
                qtest_writel(qts, i2c5 + 0x24, 1);
                g_assert_cmphex(qtest_readl(qts, i2c5 + 0x0c) & 0x7, ==, 0x1);
                v = qtest_readl(qts, i2c5 + 0x00) & 0xff;
                g_assert_cmphex(v & seq[step].mask, ==, seq[step].val);
            } else {
                qtest_writel(qts, i2c5 + 0x10, 0x0);
                qtest_writel(qts, i2c5 + 0x14, 2);
                qtest_writel(qts, i2c5 + 0x18, 1);
                qtest_writel(qts, i2c5 + 0x00, seq[step].reg);
                qtest_writel(qts, i2c5 + 0x00, seq[step].val);
                qtest_writel(qts, i2c5 + 0x24, 1);
                g_assert_cmphex(qtest_readl(qts, i2c5 + 0x0c) & 0x7, ==, 0x1);
            }
        }

        qtest_writel(qts, einte + 0x040, bit18); /* ACK after W1C */
        g_assert_cmphex(qtest_readl(qts, einte + 0x000) & bit18, ==, 0u);
    }

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/mt6895/mt6375-fifo-and-regs", test_mt6375_fifo_and_regs);
    qtest_add_func("/mt6895/mt6375-apdma-and-usb1-ippc",
                   test_mt6375_apdma_and_usb1_ippc);
    return g_test_run();
}
