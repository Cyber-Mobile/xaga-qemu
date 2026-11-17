/*
 * MediaTek MSDC (MultiMediaCard/SD Controller)
 *
 * The xaga ROM has no virtio, no NVMe, no loop device and no generic SDHCI
 * platform glue; its only block storage paths are this controller and the
 * MTK UFS host. Android's first-stage init needs /dev/block/by-name/<name>
 * partitions, so a working MSDC is what turns the ROM's own kernel into a
 * bootable phone.
 *
 * The register map, command encoding, DMA descriptor layout and interrupt
 * bits below are taken from the vendor driver, drivers/mmc/host/mtk-sd.c of
 * the Xiaomi MT6895 kernel tree, not guessed:
 *
 *   rawcmd = vol_swt << 30 | auto_cmd << 28 | blklen << 16 | go_irq << 15 |
 *            stop << 14 | rw << 13 | dtype << 11 | rsptyp << 7 | brk << 6 |
 *            opcode   (see RAWCMD_* below)
 *
 * Data moves through a General Purpose Descriptor chained to Buffer
 * Descriptors, both in guest memory.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "trace.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/sd/sd.h"
#include "system/address-spaces.h"
#include "qom/object.h"

#define TYPE_MTK_MSDC "mtk-msdc"
OBJECT_DECLARE_SIMPLE_TYPE(MtkMsdcState, MTK_MSDC)

/* Register offsets (mtk-sd.c). */
#define MSDC_CFG        0x000
#define MSDC_IOCON      0x004
#define MSDC_PS         0x008
#define MSDC_INT        0x00c
#define MSDC_INTEN      0x010
#define MSDC_FIFOCS     0x014
#define MSDC_TXDATA     0x018
#define MSDC_RXDATA     0x01c
#define SDC_CFG         0x030
#define SDC_CMD         0x034
#define SDC_ARG         0x038
#define SDC_STS         0x03c
#define SDC_RESP0       0x040
#define SDC_RESP1       0x044
#define SDC_RESP2       0x048
#define SDC_RESP3       0x04c
#define SDC_BLK_NUM     0x050
#define SDC_ADV_CFG0    0x064
#define SDC_ACMD_RESP   0x080
#define MSDC_DMA_SA     0x090
#define MSDC_DMA_CTRL   0x098
#define MSDC_DMA_GPD    0x090  /* GPD/BD list address (see note in transfer) */
#define MSDC_DMA_CFG    0x09c
#define MSDC_DMA_SA_H4  0x0a0
#define MSDC_PATCH_BIT  0x0b0
#define MSDC_PATCH_BIT1 0x0b4
#define MSDC_PATCH_BIT2 0x0b8
#define MSDC_PAD_TUNE   0x0ec
#define MSDC_PAD_TUNE0  0x0f0
#define EMMC50_CFG0     0x208
#define SDC_FIFO_CFG    0x228

#define MSDC_CFG_CKSTB  (1u << 7)
#define MSDC_CFG_RST    (1u << 2)

#define MSDC_PS_CDSTS   (1u << 1)
#define MSDC_PS_DATA1   (1u << 17)

/* Interrupt bits (W1C). */
#define MSDC_INT_MMCIRQ     (1u << 0)
#define MSDC_INT_CDSC       (1u << 1)
#define MSDC_INT_ACMDRDY    (1u << 3)
#define MSDC_INT_ACMDTMO    (1u << 4)
#define MSDC_INT_ACMDCRCERR (1u << 5)
#define MSDC_INT_CMDRDY     (1u << 8)
#define MSDC_INT_CMDTMO     (1u << 9)
#define MSDC_INT_RSPCRCERR  (1u << 10)
#define MSDC_INT_XFER_COMPL (1u << 12)
#define MSDC_INT_DXFER_DONE (1u << 13)
#define MSDC_INT_DATTMO     (1u << 14)
#define MSDC_INT_DATCRCERR  (1u << 15)

#define MSDC_FIFOCS_CLR  (1u << 31)
#define SDC_STS_CMDBUSY (1u << 0)
#define SDC_STS_SDCBUSY (1u << 1)

#define MSDC_DMA_CTRL_START  (1u << 0)
#define MSDC_DMA_CTRL_STOP   (1u << 1)
#define MSDC_DMA_CFG_STS     (1u << 0)

/* GPD / BD descriptor bits (mtk-sd.c). */
#define GPDMA_DESC_HWO  (1u << 0)
#define GPDMA_DESC_BDP  (1u << 1)
#define BDMA_DESC_EOL   (1u << 0)
#define BDMA_DESC_BUFLEN 0xffff

/* rawcmd fields */
#define RAWCMD_OPCODE(r)   ((r) & 0x3f)
#define RAWCMD_RSPTYP(r)   (((r) >> 7) & 0x7)
#define RAWCMD_DTYPE(r)    (((r) >> 11) & 0x3)
#define RAWCMD_RW(r)       (((r) >> 13) & 0x1)
#define RAWCMD_BLKLEN(r)   (((r) >> 16) & 0xfff)

#define MTK_MSDC_NREGS  0x1000

struct MtkMsdcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    SDBus sdbus;

    uint32_t regs[MTK_MSDC_NREGS / 4];

    /* Cached copies the model needs to interpret. */
    uint32_t int_status;
    uint32_t int_enable;
    uint32_t resp[4];

    /*
     * A data command records how much to move; the transfer itself runs when
     * the driver starts the DMA, because the descriptor list is programmed
     * after the command word is written.
     */
    uint32_t pending_total;
    bool pending_read;
};

static void mtk_msdc_update_irq(MtkMsdcState *s)
{
    int level = (s->int_status & s->int_enable) != 0;

    trace_mtk_msdc_irq(s->int_status, s->int_enable, level);
    qemu_set_irq(s->irq, level);
}

static void mtk_msdc_raise(MtkMsdcState *s, uint32_t bits)
{
    s->int_status |= bits;
    s->regs[MSDC_INT / 4] = s->int_status;
    mtk_msdc_update_irq(s);
}

/*
 * Walk the GPD/BD chain and move one block's worth of data between the card
 * and guest memory.
 */
static void mtk_msdc_dma_transfer(MtkMsdcState *s, bool read, uint32_t total)
{
    AddressSpace *as = &address_space_memory;
    /*
     * MSDC_DMA_SA is 0x090, which is where the driver puts the descriptor
     * list address. MSDC_DMA_GPD is kept as an alias for readability; the
     * fallback below only matters if a future register-map change moves it.
     */
    uint32_t gpd_addr = s->regs[MSDC_DMA_GPD / 4];
    if (!gpd_addr) {
        gpd_addr = s->regs[MSDC_DMA_SA / 4];
    }
    uint32_t bd_addr;
    uint32_t done = 0;
    int guard = 0;

    bd_addr = address_space_ldl_le(as, gpd_addr + 8, MEMTXATTRS_UNSPECIFIED,
                                   NULL);
    if (!(address_space_ldl_le(as, gpd_addr, MEMTXATTRS_UNSPECIFIED, NULL) &
          GPDMA_DESC_BDP)) {
        /* No buffer descriptor list: nothing to move. */
        bd_addr = 0;
    }

    while (done < total && bd_addr && guard++ < 1024) {
        uint32_t bd_info = address_space_ldl_le(as, bd_addr,
                                                MEMTXATTRS_UNSPECIFIED, NULL);
        uint32_t buf = address_space_ldl_le(as, bd_addr + 8,
                                            MEMTXATTRS_UNSPECIFIED, NULL);
        uint32_t len = address_space_ldl_le(as, bd_addr + 12,
                                            MEMTXATTRS_UNSPECIFIED, NULL) &
                       BDMA_DESC_BUFLEN;
        uint32_t n = MIN(len, total - done);

        if (n) {
            if (read) {
                uint8_t *tmp = g_malloc(n);

                sdbus_read_data(&s->sdbus, tmp, n);
                address_space_write(as, buf, MEMTXATTRS_UNSPECIFIED, tmp, n);
                g_free(tmp);
            } else {
                uint8_t *tmp = g_malloc(n);

                address_space_read(as, buf, MEMTXATTRS_UNSPECIFIED, tmp, n);
                sdbus_write_data(&s->sdbus, tmp, n);
                g_free(tmp);
            }
        }
        done += n;

        if (bd_info & BDMA_DESC_EOL) {
            break;
        }
        bd_addr = address_space_ldl_le(as, bd_addr + 4, MEMTXATTRS_UNSPECIFIED,
                                       NULL);
    }

    /*
     * Clear the busy bit. The driver writes MSDC_DMA_CFG with the start/busy
     * bit set and then spins on that register until the bit drops; on
     * hardware it self-clears once the transfer finishes. Leaving it set
     * spins the driver forever -- measured as 57 million reads of 0x09c in a
     * single run.
     */
    s->regs[MSDC_DMA_CFG / 4] &= ~MSDC_DMA_CFG_STS;

    trace_mtk_msdc_dma(gpd_addr, s->regs[MSDC_DMA_GPD / 4] ? address_space_ldl_le(as, gpd_addr, MEMTXATTRS_UNSPECIFIED, NULL) : 0,
                       bd_addr, total, done);
    mtk_msdc_raise(s, MSDC_INT_XFER_COMPL | MSDC_INT_DXFER_DONE);
}

static void mtk_msdc_do_command(MtkMsdcState *s, uint32_t rawcmd)
{
    SDRequest req;
    uint8_t resp[16];
    size_t rlen;
    uint32_t dtype = RAWCMD_DTYPE(rawcmd);
    bool data_read = !RAWCMD_RW(rawcmd);
    uint32_t total = 0;

    req.cmd = RAWCMD_OPCODE(rawcmd);
    req.arg = s->regs[SDC_ARG / 4];
    req.crc = 0;

    trace_mtk_msdc_cmd(rawcmd, req.arg);

    if (!sdbus_get_inserted(&s->sdbus)) {
        mtk_msdc_raise(s, MSDC_INT_CMDTMO);
        return;
    }

    rlen = sdbus_do_command(&s->sdbus, &req, resp, sizeof(resp));

    if (RAWCMD_RSPTYP(rawcmd) == 0) {
        /* No response expected. */
        s->resp[0] = s->resp[1] = s->resp[2] = s->resp[3] = 0;
    } else if (rlen == 16) {
        /*
         * 136-bit response (CID / CSD), read as four big-endian words with
         * resp[0] the most significant -- matching the mmc core's own
         * convention, where cmd->resp[0] is the leading word.
         *
         * Getting this backwards garbles the CID (the card's name printed as
         * junk) and makes the CSD decode to the wrong capacity, because the
         * core takes both straight from these words.
         */
        s->resp[0] = ldl_be_p(&resp[0]);
        s->resp[1] = ldl_be_p(&resp[4]);
        s->resp[2] = ldl_be_p(&resp[8]);
        s->resp[3] = ldl_be_p(&resp[12]);
    } else if (rlen == 4) {
        s->resp[0] = ldl_be_p(resp);
        s->resp[1] = s->resp[2] = s->resp[3] = 0;
    } else {
        mtk_msdc_raise(s, MSDC_INT_CMDTMO);
        return;
    }

    /*
     * Register order differs with the response length.
     *
     * For a 136-bit R2 response the driver reads RESP3 first and stores it as
     * resp[0], so RESP3 carries the most significant word and RESP0 the
     * least -- an identity mapping onto s->resp[].
     *
     * For a 32-bit R1 response the driver reads RESP0; putting it in RESP3
     * instead makes every R1 look like zero, so e.g. the APP_CMD bit never
     * appears and the card-init sequence retries four times and gives up.
     */
    if (rlen == 16) {
        s->regs[SDC_RESP0 / 4] = s->resp[3];
        s->regs[SDC_RESP1 / 4] = s->resp[2];
        s->regs[SDC_RESP2 / 4] = s->resp[1];
        s->regs[SDC_RESP3 / 4] = s->resp[0];
    } else {
        s->regs[SDC_RESP0 / 4] = s->resp[0];
        s->regs[SDC_RESP1 / 4] = 0;
        s->regs[SDC_RESP2 / 4] = 0;
        s->regs[SDC_RESP3 / 4] = 0;
    }

    trace_mtk_msdc_resp(req.cmd, s->resp[0]);
    mtk_msdc_raise(s, MSDC_INT_CMDRDY);

    if (dtype == 0) {
        return;
    }

    /*
     * Do not move data here. The driver writes SDC_CMD first and only then
     * programs the DMA (GPD address into 0x090, then START into
     * MSDC_DMA_CTRL), so transferring at command time would walk a descriptor
     * list that has not been set up yet -- which is exactly why some
     * transfers saw gpd=0 and moved nothing. Record what is pending and let
     * the DMA start trigger do the work.
     */
    total = RAWCMD_BLKLEN(rawcmd) * s->regs[SDC_BLK_NUM / 4];
    s->pending_total = total;
    s->pending_read = data_read;
}

static uint64_t mtk_msdc_read(void *opaque, hwaddr offset, unsigned size)
{
    MtkMsdcState *s = MTK_MSDC(opaque);
    uint32_t val;

    switch (offset) {
    case MSDC_CFG:
        /*
         * Two bits need modelling rather than plain storage.
         *
         * CKSTB: the driver spins on it after every clock change, so the
         * clock is always reported stable.
         *
         * RST: software reset is self-clearing on hardware. The driver sets
         * it and then waits for it to drop (msdc_reset_hw), so a register
         * file that echoes the write back spins there forever.
         */
        s->regs[MSDC_CFG / 4] &= ~MSDC_CFG_RST;
        val = s->regs[MSDC_CFG / 4] | MSDC_CFG_CKSTB;
        break;
    case MSDC_PS:
        val = s->regs[MSDC_PS / 4];
        if (sdbus_get_inserted(&s->sdbus)) {
            val |= MSDC_PS_CDSTS;
            /*
             * A card at rest holds its data lines high. The preloader's
             * own driver waits for exactly that: it spins on MSDC_PS
             * bit 15 (DAT1) at 0x0203919E before touching the controller,
             * so without these bits boot never leaves the storage layer.
             */
            val |= 0xF << 14;      /* DAT0..DAT3 idle-high */
        } else {
            val &= ~MSDC_PS_CDSTS;
            val &= ~(0xF << 14);
        }
        break;
    case MSDC_INT:
        val = s->int_status;
        break;
    case MSDC_INTEN:
        val = s->int_enable;
        break;
    case MSDC_FIFOCS:
        /*
         * FIFO state is entirely synthesised: transfers complete before the
         * write returns, so the FIFO is always empty and its clear bit has
         * always finished. MSDC_FIFOCS_CLR is self-clearing on hardware and
         * msdc_reset_hw spins on it, so echoing the write back would hang
         * the driver exactly like MSDC_CFG_RST did. The TXCNT/RXCNT fields
         * must read zero because msdc_irq reads them to decide the FIFO is
         * drained.
         */
        val = 0;
        break;
    case SDC_STS:
        /* Commands complete synchronously here, so the controller is never
         * busy. */
        val = 0;
        break;
    case MSDC_DMA_CFG:
        val = s->regs[MSDC_DMA_CFG / 4];
        break;
    default:
        val = s->regs[offset / 4];
        break;
    }

    trace_mtk_msdc_read(offset, val);
    return val;
}

static void mtk_msdc_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    MtkMsdcState *s = MTK_MSDC(opaque);
    uint32_t val = value;

    trace_mtk_msdc_write(offset, val);

    switch (offset) {
    case MSDC_INT:
        /* Write-one-to-clear. */
        s->int_status &= ~val;
        s->regs[MSDC_INT / 4] = s->int_status;
        mtk_msdc_update_irq(s);
        return;
    case MSDC_INTEN:
        s->int_enable = val;
        mtk_msdc_update_irq(s);
        return;
    case SDC_CMD:
        s->regs[SDC_CMD / 4] = val;
        mtk_msdc_do_command(s, val);
        return;
    case MSDC_DMA_CTRL:
        s->regs[MSDC_DMA_CTRL / 4] = val;
        if ((val & MSDC_DMA_CTRL_START) && s->pending_total) {
            uint32_t total = s->pending_total;
            bool read = s->pending_read;

            s->pending_total = 0;
            mtk_msdc_dma_transfer(s, read, total);
        }
        return;
    default:
        s->regs[offset / 4] = val;
        return;
    }
}

static const MemoryRegionOps mtk_msdc_ops = {
    .read = mtk_msdc_read,
    .write = mtk_msdc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void mtk_msdc_reset_hold(Object *obj, ResetType type)
{
    MtkMsdcState *s = MTK_MSDC(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->int_status = 0;
    s->int_enable = 0;
    memset(s->resp, 0, sizeof(s->resp));
    s->regs[MSDC_DMA_CFG / 4] = 0;
    qemu_set_irq(s->irq, 0);
}

static void mtk_msdc_realize(DeviceState *dev, Error **errp)
{
    /*
     * Nothing to sample here: whether a card is inserted has to be asked at
     * command time. The machine attaches the card after this controller is
     * realized, so caching the answer now would latch "no card" for the whole
     * run and every command would report a timeout.
     */
}

static void mtk_msdc_init(Object *obj)
{
    MtkMsdcState *s = MTK_MSDC(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    memory_region_init_io(&s->iomem, obj, &mtk_msdc_ops, s,
                          TYPE_MTK_MSDC, MTK_MSDC_NREGS);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_SD_BUS, DEVICE(obj),
              "sd-bus");
}

static void mtk_msdc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = mtk_msdc_realize;
    dc->desc = "MediaTek MSDC SD/eMMC controller";
    rc->phases.hold = mtk_msdc_reset_hold;
}

static const TypeInfo mtk_msdc_info = {
    .name = TYPE_MTK_MSDC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkMsdcState),
    .instance_init = mtk_msdc_init,
    .class_init = mtk_msdc_class_init,
};

static void mtk_msdc_register_types(void)
{
    type_register_static(&mtk_msdc_info);
}

type_init(mtk_msdc_register_types);
