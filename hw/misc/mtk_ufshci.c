/*
 * MediaTek UFS host controller (UFSHCI)
 *
 * ufshci@112b0000 carries the UFS controller the ROM kernel's ufshcd-mtk
 * driver binds to. The register block used to be a plain read/write file, so
 * every capability the driver probes for read back as zero:
 *
 *   ufshcd-mtk 112b0000.ufshci: caps: 0x0
 *   ufshcd-mtk 112b0000.ufshci: missing ack of refclk req, reg: 0x1
 *   ufshcd-mtk 112b0000.ufshci: invalid UFS version 0x0
 *   ufshcd-mtk 112b0000.ufshci: probe failed -22
 *
 * A trace of the window shows exactly which registers the driver touches and
 * in what order, and that is what this device answers:
 *
 *   write 0x144 = 0x1     vendor reference-clock request, then poll 0x144
 *   read  0x000           CAP  -- capabilities; zero makes the driver reject
 *                                the controller before it ever starts
 *   read  0x008           VER  -- UFSHCI version
 *   read  0x020           AHIT -- host controller type
 *
 * Only these are modelled: the controller is reported as a working UFSHCI of
 * a known version, with the reference clock acknowledged. Anything past that
 * (transfer requests, UIC commands) still reads as whatever was written, so a
 * driver that gets further will need the rest filled in -- which is the point
 * of modelling them one at a time rather than returning a blanket value.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <unistd.h>
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/module.h"
#include "qemu/mtk-env.h"
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "system/dma.h"
#include "system/address-spaces.h"
#include "qom/object.h"
#include "trace.h"

/*
 * Set by the machine model when BL31 hands over to BL33 (LK). Until then the
 * preloader/BL2 read the partition table with 512-byte blocks and must see the
 * raw image; afterwards LK reads it with 4 KiB blocks and needs the patched
 * table that carries the extra "lk_main_dtb" partition.
 */
/*
 * Positional read.  MinGW does not declare pread(), so on Windows seek and
 * read instead: the model serialises UFS accesses per command anyway, so a
 * shared file position is safe here.
 */
static ssize_t mtk_ufs_pread(int fd, void *buf, size_t len, off_t off)
{
#ifdef _WIN32
    {
        static int n;

        if (_lseeki64(fd, (int64_t)off, SEEK_SET) < 0) {
            if (n < 10) {
                n++;
                fprintf(stderr, "UFS-PREAD seek FAIL off=%#" PRIx64
                        " errno=%d\n", (uint64_t)off, errno);
            }
            return -1;
        }
        {
            int r = _read(fd, buf, (unsigned)len);

            if (n < 10) {
                n++;
                fprintf(stderr, "UFS-PREAD off=%#" PRIx64 " len=%zu ret=%d"
                        "\n", (uint64_t)off, len, r);
            }
            return r;
        }
    }
#else
    return pread(fd, buf, len, off);
#endif
}

bool mtk_ufs_lk_stage;

#define TYPE_MTK_UFSHCI "mtk-ufshci"
OBJECT_DECLARE_SIMPLE_TYPE(MtkUfshciState, MTK_UFSHCI)

static int mtk_ufshci_read_disk_lun(MtkUfshciState *s, unsigned lun,
                                    uint64_t off, void *buf, size_t len);

/* UFSHCI standard registers */
#define UFSHCI_CAP              0x000   /* capabilities */
#define UFSHCI_VER              0x008   /* version */
#define UFSHCI_AHIT             0x020   /* host controller type */
#define UFSHCI_HCS              0x030   /* host controller status */
#define UFSHCI_IS               0x034   /* interrupt status */
#define UFSHCI_IE               0x038   /* interrupt enable */

/* UIC command interface */
#define UFSHCI_UCMD             0x090   /* opcode */
#define UFS_UIC_OP_DME_GET      0x1
#define UFS_UIC_OP_DME_SET      0x2
#define UFSHCI_UCMDARG1         0x094   /* attribute id / selectors */
#define UFSHCI_UCMDARG2         0x098
#define UFSHCI_UCMDARG3         0x09C   /* result */

/*
 * MIB attribute the driver reads back after starting the link:
 * PA_ACTIVE_TXGEAR, in ARG1's upper half. A trace of the failing run shows
 * the driver ask for it (arg1 = 0x15a90000, selector 0) and log
 * "dme-get: attr-id 0x15a9 failed". Reporting gear 1 is what lets the link
 * come up; anything that reads back as "no gear" leaves it retrying.
 */
#define UFS_MIB_ATTR_MASK       0xffff0000
#define UFS_MIB_PA_ACTIVE_TXGEAR 0x15a90000
#define UFS_MIB_PA_ACTIVE_RXGEAR 0x15870000
#define UFS_MIB_PA_TXGEAR        0x15680000
#define UFS_MIB_PA_RXGEAR        0x15690000
#define UFS_MIB_PA_PWR_MODE      0x15860000
#define UFS_MIB_PA_ACTIVE_LANE    0x15810000
#define UFS_MIB_PA_TX_LANE        0x15610000
#define UFS_MIB_PA_PWR_MODE_JEDEC 0x15710000
/* Lane availability/active-lane MIB attributes.  The link bring-up reads
 * these and treats a zero lane mask as "no lane usable", which makes it
 * retry DME_LINKSTARTUP forever.  Report the two lanes the UFS host has. */
#define UFS_MIB_PA_AVAIL_TX_LANE  0x15a80000
#define UFS_MIB_PA_AVAIL_RX_LANE  0x15aa0000
#define UFS_MIB_PA_AVAIL_RX_LANE2 0x15a70000
#define UFS_MIB_PA_1543           0x15430000
#define UFS_MIB_PA_ACTIVE_TX_LANE 0x15820000
#define UFS_MIB_PA_ACTIVE_RX_LANE 0x15830000
#define UFS_MIB_LANES            0x3   /* lanes 0 and 1 */
#define UFS_MIB_GEAR_1           1

/* Offset of disk LBA 0 inside the data image file. */
#define MTK_UFS_IMAGE_PREFIX     4096

/* MediaTek vendor: reference clock control */
#define UFS_MTK_REFCLK_CTRL     0x144

/*
 * MediaTek vendor: command issue. A trace of a failing run shows the driver
 * write 0x80 here and then poll the same register, over and over:
 *
 *   write 0x140 = 0x80 / ... / read 0x140 -> 0x80 / write 0x140 = 0x80 ...
 *
 * Reading back exactly what was written is what keeps it looping, so the
 * bit is treated as a self-clearing "command accepted": it is cleared once
 * the guest has observed it, and the interface reports itself ready.
 */
/*
 * UIC command completion. A trace of the failing run shows the driver write
 * the opcode to UCMD (0x90) after filling ARG1..ARG3, then wait -- every
 * command ended in "completion timeout" because nothing ever raised the
 * completion interrupt. The controller reports it by setting UCCS in the
 * interrupt status register, which is also what the driver clears by writing
 * IS back.
 */
#define UFSHCI_IS_UCCS          BIT(5)  /* UIC command completion status */

/*
 * MediaTek vendor interrupt pair.
 *
 * A trace shows the driver clear 0x20 early on and later write 0x470 to 0x24
 * -- an enable mask in the same block as the vendor command register -- and
 * the driver never polls for UIC completion, so this is where the completion
 * interrupt lives.
 *
 * Raising it was tried twice and had to be undone both times. First with the
 * link attributes reading zero, then with PA_ACTIVE_TXGEAR reporting gear 1 --
 * in both cases the driver retries DME_LINKSTARTUP forever and module loading
 * stalls (169 modules down to 162, boot span 282s down to 93s). Something else
 * in the link bring-up is still missing; until that is understood, the
 * completion stays on the standard status register and this stays inert.
 */
#define UFS_MTK_IS              0x020   /* vendor interrupt status */
#define UFS_MTK_IE              0x024   /* vendor interrupt enable */
#define UFS_MTK_UIC_DONE        BIT(10) /* UIC command completed */

#define UFS_MTK_CMD_ISSUE       0x140
#define UFS_MTK_CMD_ISSUE_BIT   BIT(7)
#define UFS_MTK_CMD_READY_BIT   BIT(0)

/*
 * Capabilities. The driver derives its transfer-request slot count from this
 * and refuses a controller that reports none, so NUTRS must be non-zero.
 */
#define UFSHCI_CAP_VALUE        0x0000000f

/* UFSHCI 3.0. Anything the driver does not recognise is reported as invalid. */
#define UFSHCI_VER_VALUE        0x00000300

/* Host controller type: a UFSHCI-compliant host. */
#define UFSHCI_AHIT_VALUE       0x00000001

/*
 * Host controller status. The driver will not issue a UIC command until the
 * controller reports both its UIC and its transfer-request interfaces ready;
 * with the register left at zero it prints "Controller not ready to accept
 * UIC commands" for every command and gets nowhere.
 */
#define UFSHCI_HCS_UCRDY        BIT(0)  /* UIC command ready */
#define UFSHCI_HCS_UTRLRDY      BIT(1)  /* UTP transfer request list ready */
#define UFSHCI_HCS_UTMRLRDY     BIT(2)  /* UTP task management list ready */
#define UFSHCI_HCS_UTRLRSR      BIT(3)  /* UTP transfer request list run-stop */
#define UFSHCI_HCS_UTMRLRSR     BIT(4)  /* UTP task management run-stop */
/* EXPERIMENT: BL2's driver requires (HCS & 0xfe) == 0x0e after init, i.e.
 * UCRDY/UTRLRDY/UTMRLRDY/UTRLRSR set and UTMRLRSR clear. */
#define UFSHCI_HCS_VALUE        (UFSHCI_HCS_UCRDY | UFSHCI_HCS_UTRLRDY | \
                                 UFSHCI_HCS_UTMRLRDY | UFSHCI_HCS_UTRLRSR)

/*
 * The vendor register is a request/acknowledge pair: the driver sets the
 * request bit and polls until the controller answers. Only the request bit is
 * visible in the trace (it wrote 0x1 and kept reading 0x1 back from the plain
 * register), so the acknowledge is the next bit up.
 */
#define UFS_MTK_REFCLK_REQ      BIT(0)
#define UFS_MTK_REFCLK_ACK      BIT(1)

/*
 * UTP transfer request interface.
 *
 * Everything up to here makes the driver's UIC commands succeed, but no data
 * ever moves: the transfer-request path was left as plain registers, so a
 * doorbell write did nothing and a driver waiting on completion waited
 * forever. That is the step between "the link is up" and "the partition was
 * read", so it is modelled next.
 *
 *   UTRLBA   0x050/0x054/0x058  base address of the transfer request list
 *   UTRLDBR  0x060              doorbell; bit N means slot N was submitted
 *   IS bit 0 UTRCS              transfer request completion status
 *
 * Each slot's descriptor (UTRD, 32 bytes) begins with the UPIU it carries;
 * byte 2 of the UTRD is where the controller reports the overall command
 * status. The completion is written back there, and UTRCS is raised, which is
 * what the driver's wait loop is looking for.
 */
#define UFSHCI_UTRLBA           0x050
#define UFSHCI_UTRLBAU          0x054
#define UFSHCI_UTRLDBR          0x058
#define UFSHCI_IS_UTRCS         BIT(0)  /* transfer request completion */
#define UFSHCI_UTRD_SIZE        32
/*
 * Measured UTRD (LK, list base 0x630DFBC0, before the doorbell):
 *   +0x00  UPIU header   0x11000000 = SCSI command
 *   +0x08  dword        0x0000000F  <-- LK prints this as "OCS error = f"
 *   +0x10  response UPIU pointer
 *   +0x18  PRDT pointer, then PRDT length in bytes
 * A completed transfer must clear that dword. PRDT entries are 16 bytes:
 *   +0 u64 buffer, +8 u32 byte count (or count-1).
 */
#define UFSHCI_UTRD_OCS         8
#define UFSHCI_OCS_SUCCESS      0
#define UFSHCI_DATA_BASE        0x80000000ULL
/* the data image carries the real 4K GPT, so use 4K logical blocks */
#define UFS_LB_SIZE             4096ULL

/*
 * UCD layout measured from LK (vendor module analysis):
 *   command UPIU at ucd+0, CDB at +0x10
 *   response UPIU at ucd+0x200, must read back [0]=0x21 and [6]=0 (GOOD)
 *   PRDT at ucd+0x400; each entry is 16 bytes:
 *     +0 u64 buffer, +0xC = (bytes-1) & 0x3FFFF
 * The data image is mapped at 0x80000000 with 512-byte logical sectors.
 */
/*
 * UFS descriptors.  The driver only needs a plausible geometry and unit
 * descriptor to compute LBAs and block counts; the rest just has to exist
 * with the right length and IDN.
 */
#define UFS_DEV_DESC_LEN   0x40
#define UFS_UNIT_DESC_LEN  0x23
#define UFS_GEOM_DESC_LEN  0x44
#define UFS_POWER_DESC_LEN 0x62
#define UFS_HEALTH_DESC_LEN 0x25
#define UFS_BOOT_DESC_LEN  0x0f
#define UFS_RPMB_DESC_LEN  0x2f
#define UFS_CONFIG_DESC_LEN 0x90
#define UFS_INTERCONNECT_DESC_LEN 0x06

/* total capacity reported to the guest: the real 128 GiB device */
/*
 * Capacity must match the real device: the image's GPT header records
 * alternate_lba = 31240191, so the last LBA a host validates is exactly that.
 * A larger synthetic capacity made LK read LBA 0x1ffffff for the backup
 * table, get unrelated data, reject the table and fail every partition
 * lookup (which is what kept the device tree out of LK).
 */
#define UFS_TOTAL_CAPACITY 0x2000000000ULL

static void mtk_ufshci_build_desc(uint8_t idn, uint8_t *out, uint32_t *len)
{
    memset(out, 0, 0x100);
    switch (idn) {
    case 0x00: {                       /* device descriptor */
        out[0x00] = UFS_DEV_DESC_LEN;
        out[0x01] = 0x00;
        out[0x02] = 0x00;              /* bDevice */
        out[0x06] = 0x08;              /* bNumberLU: 8 LUs like the real part */
        out[0x07] = 0x01;              /* bNumberWLU */
        out[0x10] = 0x10;              /* wSpecVersion 3.1 (LE) */
        out[0x11] = 0x03;
        *len = UFS_DEV_DESC_LEN;
        break;
    }
    case 0x02: {                       /* unit descriptor (LU 0) */
        out[0x00] = UFS_UNIT_DESC_LEN;
        out[0x01] = 0x02;
        out[0x03] = 0x01;              /* bLUEnable */
        out[0x0a] = 0x0c;              /* bLogicalBlockSize: 2^12 = 4 KiB */
        stq_le_p(out + 0x0c, UFS_TOTAL_CAPACITY / UFS_LB_SIZE);
        *len = UFS_UNIT_DESC_LEN;
        break;
    }
    case 0x03: {                       /* RPMB unit descriptor */
        out[0x00] = UFS_RPMB_DESC_LEN;
        out[0x01] = 0x03;
        out[0x03] = 0x01;
        *len = UFS_RPMB_DESC_LEN;
        break;
    }
    case 0x04: {                       /* power descriptor */
        out[0x00] = UFS_POWER_DESC_LEN;
        out[0x01] = 0x04;
        *len = UFS_POWER_DESC_LEN;
        break;
    }
    case 0x05: {                       /* health descriptor */
        out[0x00] = UFS_HEALTH_DESC_LEN;
        out[0x01] = 0x05;
        *len = UFS_HEALTH_DESC_LEN;
        break;
    }
    case 0x06: {                       /* boot LUN descriptor */
        out[0x00] = UFS_BOOT_DESC_LEN;
        out[0x01] = 0x06;
        *len = UFS_BOOT_DESC_LEN;
        break;
    }
    case 0x01: {                       /* configuration descriptor */
        unsigned lu;

        out[0x00] = UFS_CONFIG_DESC_LEN;
        out[0x01] = 0x01;
        /* 0x10..0x8f: eight 16-byte LU configuration entries */
        for (lu = 0; lu < 8; lu++) {
            uint8_t *e = out + 0x10 + lu * 0x10;

            /*
             * UFS Configuration Descriptor, LU entry layout (JESD220):
             *   +0x00 bLUEnable          +0x01 bBootLunID
             *   +0x02 bLUWriteProtect    +0x03 bMemoryType
             *   +0x04 dNumAllocUnits (4) +0x08 bDataReliability
             *   +0x09 bLogicalBlockSize  +0x0a bProvisioningType
             *   +0x0b bLUQueueDepth
             */
            e[0x00] = (lu == 0) ? 0x01 : 0x00;  /* bLUEnable */
            e[0x01] = 0x00;                     /* bBootLunID */
            e[0x02] = 0x00;                     /* bLUWriteProtect */
            e[0x03] = 0x00;                     /* bMemoryType */
            /*
             * dNumAllocUnits is in 4 KiB allocation units and has to agree
             * with READ CAPACITY, otherwise LK's ufs_check_lu_cfg rejects the
             * configuration after printing it.
             */
            stl_le_p(e + 0x04,
                     (uint32_t)(UFS_TOTAL_CAPACITY / UFS_LB_SIZE));
            e[0x08] = 0x00;                     /* bDataReliability */
            e[0x09] = 0x0c;                     /* bLogicalBlockSize: 2^12 = 4 KiB */
            e[0x0a] = 0x00;                     /* bProvisioningType */
            e[0x0b] = 0x00;                     /* bLUQueueDepth */
        }
        /*
         * LK parses the LU configuration table from these two bytes:
         * config[0x1a] = offset of LU0's entry, config[0x1b] = entry stride.
         * Leaving them zero makes ufs_check_lu_cfg loop forever.
         */
        out[0x1a] = 0x10;
        out[0x1b] = 0x10;
        *len = UFS_CONFIG_DESC_LEN;
        break;
    }
    case 0x08: {                       /* interconnect descriptor */
        out[0x00] = UFS_INTERCONNECT_DESC_LEN;
        out[0x01] = 0x08;
        out[0x02] = 0x01;              /* bUniProVersion */
        out[0x03] = 0x00;              /* bMphyVersion */
        out[0x04] = 0x01;              /* bUniProMphySpeed? */
        *len = UFS_INTERCONNECT_DESC_LEN;
        break;
    }
    case 0x07: {                       /* geometry descriptor */
        out[0x00] = UFS_GEOM_DESC_LEN;
        out[0x01] = 0x07;
        stq_le_p(out + 0x04, UFS_TOTAL_CAPACITY);
        /*
         * JESD220 Geometry Descriptor layout:
         *   +0x04 qTotalRawDeviceCapacity   +0x0c bMaxNumberLU
         *   +0x0d dSegmentSize              +0x11 bAllocationUnitSize
         *   +0x12 bMinAddrBlockSize         +0x13 bOptimalReadBlockSize
         *   +0x14 bOptimalWriteBlockSize    +0x15 bMaxInBufferSize
         *   +0x16 bMaxOutBufferSize
         */
        out[0x0c] = 0x01;                  /* bMaxNumberLU */
        stl_le_p(out + 0x0d, 0x100000);    /* dSegmentSize */
        out[0x11] = 0x0c;                  /* bAllocationUnitSize: 4 KiB */
        out[0x12] = 0x0c;                  /* bMinAddrBlockSize */
        out[0x13] = 0x0c;                  /* bOptimalReadBlockSize */
        out[0x14] = 0x0c;                  /* bOptimalWriteBlockSize */
        out[0x15] = 0x0f;                  /* bMaxInBufferSize (32 KiB) */
        out[0x16] = 0x0f;                  /* bMaxOutBufferSize */
        *len = UFS_GEOM_DESC_LEN;
        break;
    }
    default:
        *len = 0;
        break;
    }
    /* LU config table location, in case LK reads it from this buffer */
    if (*len > 0x1c) {
        out[0x1a] = 0x10;
        out[0x1b] = 0x10;
    }
}

/*
 * UPIU layout used by this LK (measured, vendor module analysis):
 *   +0x00 type, +0x01 flags, +0x02 LUN, +0x03 task tag
 *   +0x04 query function (0x81 = read/write standard request)
 *   +0x06 status in the response (0 = GOOD)
 *   +0x08 data length, big endian
 *   +0x0c opcode, +0x0d idn, +0x0e index, +0x0f selector
 *   +0x20 data segment
 * Query opcodes: 1 read descriptor, 2 write descriptor, 3 read attribute,
 * 4 write attribute, 5 read flag, 6 set flag, 7 clear flag, 8 toggle flag.
 */
static void mtk_ufshci_query(const uint8_t *rq, uint8_t *resp)
{
    uint8_t opcode = rq[0x0c];
    uint8_t idn = rq[0x0d];
    uint8_t desc[0x100];
    uint32_t len = 0;

    resp[0x00] = 0x36;                 /* QUERY RESPONSE */
    resp[0x02] = rq[0x02];
    resp[0x03] = rq[0x03];
    resp[0x04] = rq[0x04];
    resp[0x06] = 0x00;                 /* status: GOOD */
    memcpy(resp + 0x0c, rq + 0x0c, 4);

    switch (opcode) {
    case 0x01: {                       /* read descriptor */
        uint32_t want = ((uint32_t)rq[0x0a] << 8) | rq[0x0b];

        mtk_ufshci_build_desc(idn, desc, &len);
        if (want && want < len) {
            len = want;                /* e.g. the 2-byte header probe */
        }
        if (len) {
            resp[0x0a] = len >> 8;
            resp[0x0b] = len & 0xff;
            memcpy(resp + 0x20, desc, len);
        }
        break;
    }
    case 0x03:                         /* read attribute: 4-byte value */
        resp[0x0a] = 0;
        resp[0x0b] = 4;
        memset(resp + 0x20, 0, 4);
        break;
    case 0x05:                         /* read flag: 1-byte value */
        resp[0x0a] = 0;
        resp[0x0b] = 1;
        resp[0x20] = 0x00;             /* fDeviceInit reads back cleared */
        break;
    default:                           /* writes: just acknowledge */
        break;
    }
}

/*
 * Serve one UTP task: the command UPIU is at ucd+0, the response UPIU at
 * ucd+0x200, the PRDT at ucd+0x400 (offset given in dwords by the UTRD).
 * SCSI READ(10)/READ(16) move data out of the device image mapped at
 * 0x80000000 (LBA * 512).
 */
#define MTK_UFSHCI_ATTRS 16

#define UFS_RPMB_FRAME_LEN 512
#define UFS_RPMB_MAC_OFS 0x0c4
#define UFS_RPMB_MAC_LEN 32
#define UFS_RPMB_WCOUNT_OFS 0x1f4
#define UFS_RPMB_RES_OFS 0x1fc
#define UFS_RPMB_TYPE_OFS 0x1fe
/* Frames handled per SECURITY PROTOCOL command (the boot chain uses one). */
#define UFS_RPMB_MAX_FRAMES 16

/*
 * Provided by hw/misc/mtk_ufs_lu.c.  The RPMB W-LUN is one device, so the
 * host-controller model and the LU model must drive the same JEDEC state
 * machine; the controller must never compute a result or a MAC itself.
 */
bool mtk_ufs_rpmb_enabled(void);
int mtk_ufs_rpmb_request(const uint8_t *frames, int nframes);
int mtk_ufs_rpmb_response(uint8_t *frames, int max_frames);


struct MtkUfshciState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t *regs;
    uint32_t size;
    bool pwr_mode_pending;
    QEMUTimer *irq_retry;
    QEMUTimer *irq_raise;
    char *data_path;
    char *gpt_path;
    char *boot_lun_path;
    uint8_t *gpt_img;
    uint8_t *gpt_bak;
    uint8_t *gpt_bak_ent;
    uint8_t *gpt_ent512;
    size_t gpt_img_len;
    int data_fd;
    int boot_lun_fd;
    uint32_t attr_id[MTK_UFSHCI_ATTRS];
    uint32_t attr_val[MTK_UFSHCI_ATTRS];
    unsigned attr_n;
};

static void mtk_ufshci_serve(MtkUfshciState *s, uint64_t ucd,
                             uint16_t prdt_off, uint16_t prdt_cnt)
{
    uint8_t rq[0x40];
    uint8_t resp[0x200];
    uint64_t lba = 0;
    uint32_t blocks = 0;
    unsigned i;

    if (!ucd || dma_memory_read(&address_space_memory, ucd, rq, sizeof(rq),
                                MEMTXATTRS_UNSPECIFIED)) {
        return;
    }
    {
        static int nsrv;
        if (nsrv < 3000) {
            nsrv++;
            fprintf(stderr, "MT6895-DBG ufs serve ucd=%#" PRIx64
                    " type=%02x lun=%02x op=%02x idn=%02x idx=%02x sel=%02x"
                    " want=%u cnt=%u off=%#x cdb=%02x len=%02x\n",
                    ucd, rq[0], rq[2], rq[0x0c], rq[0x0d], rq[0x0e], rq[0x0f],
                    ((unsigned)rq[0x0a] << 8) | rq[0x0b], prdt_cnt, prdt_off,
                    rq[0x10], rq[0x17]);
        }
    }
    memset(resp, 0, sizeof(resp));

    if (rq[2] == 0xc4 && prdt_cnt) {
        uint8_t ent[16];
        uint64_t buf;
        uint32_t len;
        uint8_t frame[512];
        if (!dma_memory_read(&address_space_memory,
                             ucd + (prdt_off ? prdt_off : 0x400), ent,
                             sizeof(ent), MEMTXATTRS_UNSPECIFIED)) {
            memcpy(&buf, ent, 8);
            memcpy(&len, ent + 12, 4);
            len = (len & 0x3ffff) + 1;
            fprintf(stderr, "MT6895-DBG ufs rpmb prdt buf=%#" PRIx64
                    " len=%u\n", buf, len);
            if (len >= 512 && !dma_memory_read(&address_space_memory, buf,
                                                frame, 512,
                                                MEMTXATTRS_UNSPECIFIED)) {
                fprintf(stderr, "MT6895-DBG ufs rpmb frame:");
                for (unsigned bi = 0; bi < 512; bi++) {
                    fprintf(stderr, "%02x", frame[bi]);
                }
                fprintf(stderr, "\n");
            }
        }
    }

    switch (rq[0]) {
    case 0x00:                         /* NOP OUT -> NOP IN */
        resp[0] = 0x20;
        resp[2] = rq[2];               /* LUN */
        resp[3] = rq[3];               /* task tag */
        break;
    case 0x16:                         /* QUERY REQUEST -> QUERY RESPONSE */
        mtk_ufshci_query(rq, resp);
        break;
    case 0x01: {                       /* SCSI command -> RESPONSE */
        const uint8_t *cdb = rq + 0x10;

        resp[0] = 0x21;
        resp[2] = rq[2];               /* LUN */
        resp[3] = rq[3];               /* task tag */
        if (mtk_ufs_lk_stage) {
            static int nsv;
            if (nsv < 120) {
                nsv++;
                fprintf(stderr, "UFS-LKSV cdb=%02x lun=%02x cnt=%u off=%#x\n",
                        cdb[0], rq[2], prdt_cnt, prdt_off);
            }
        }
        {
            static int nscsi;
            if (nscsi < 40) {
                nscsi++;
                fprintf(stderr, "MT6895-DBG ufs scsi lun=%02x cdb=%02x %02x %02x %02x"
                        " %02x %02x %02x %02x %02x %02x\n",
                        rq[2],
                        cdb[0], cdb[1], cdb[2], cdb[3], cdb[4], cdb[5],
                        cdb[6], cdb[7], cdb[8], cdb[9]);
            }
        }
        resp[0x05] = 0x00;             /* status GOOD */
        if (mtk_ufs_rpmb_enabled() && rq[2] == 0xc4 && prdt_cnt &&
            (cdb[0] == 0xb5 || cdb[0] == 0xa2 || cdb[0] == 0xa3)) {
            /* Legacy UFS RPMB arrives as SECURITY PROTOCOL CDBs to the
             * RPMB well-known LUN: 0xB5 (OUT) carries the request frames,
             * 0xA2/0xA3 (IN) returns the response frames.  The frames are
             * handed to the genuine JEDEC HMAC-SHA256 state machine in
             * hw/misc/mtk_ufs_lu.c.  This model computes no result, no
             * write counter and no MAC on the guest's behalf: an unkeyed
             * RPMB must fail the way it fails on the device. */
            uint8_t ent[16];
            uint64_t buf = 0;
            uint32_t len = 0;

            if (!dma_memory_read(&address_space_memory,
                                 ucd + (prdt_off ? prdt_off : 0x400), ent,
                                 sizeof(ent), MEMTXATTRS_UNSPECIFIED)) {
                memcpy(&buf, ent, 8);
                memcpy(&len, ent + 12, 4);
                len = (len & 0x3ffff) + 1;
                if (buf && len >= UFS_RPMB_FRAME_LEN) {
                    uint8_t frames[UFS_RPMB_MAX_FRAMES * UFS_RPMB_FRAME_LEN];
                    int nframes = len / UFS_RPMB_FRAME_LEN;

                    if (nframes > UFS_RPMB_MAX_FRAMES) {
                        nframes = UFS_RPMB_MAX_FRAMES;
                    }
                    if (cdb[0] == 0xb5) {
                        if (!dma_memory_read(&address_space_memory, buf,
                                             frames,
                                             nframes * UFS_RPMB_FRAME_LEN,
                                             MEMTXATTRS_UNSPECIFIED)) {
                            mtk_ufs_rpmb_request(frames, nframes);
                        }
                    } else {
                        int got = mtk_ufs_rpmb_response(frames, nframes);

                        if (got > 0 &&
                            !dma_memory_write(&address_space_memory, buf,
                                              frames,
                                              got * UFS_RPMB_FRAME_LEN,
                                              MEMTXATTRS_UNSPECIFIED) &&
                            MTK_ENV_CACHED("MTK_RPMB_TRACE")) {
                            fprintf(stderr,
                                    "MT6895 ufs rpmb IN cdb=%02x frames=%d "
                                    "type=%#x result=%#x wcnt=%#x\n",
                                    cdb[0], got,
                                    lduw_be_p(frames + UFS_RPMB_TYPE_OFS),
                                    lduw_be_p(frames + UFS_RPMB_RES_OFS),
                                    ldl_be_p(frames + UFS_RPMB_WCOUNT_OFS));
                        }
                    }
                }
            }
        }
        {
            static int nocdb;
            static uint8_t seen[64];
            int k, dup = 0;

            for (k = 0; k < nocdb; k++) {
                if (seen[k] == cdb[0]) {
                    dup = 1;
                    break;
                }
            }
            if (!dup && nocdb < 64) {
                seen[nocdb++] = cdb[0];
                fprintf(stderr, "MTK-CDB opcode=%#x\n", cdb[0]);
            }
        }
        if (cdb[0] == 0x28) {          /* READ(10) */
            lba = ((uint64_t)cdb[2] << 24) | (cdb[3] << 16) |
                  (cdb[4] << 8) | cdb[5];
            blocks = (cdb[7] << 8) | cdb[8];
            {
                static int nrd;
                if (nrd < 40) {
                    uint8_t hdr[512] = { 0 };
                    nrd++;
                    mtk_ufshci_read_disk_lun(s, rq[2], lba * UFS_LB_SIZE, hdr,
                                         sizeof(hdr));
                    uint8_t pent[16] = { 0 };
                    uint32_t plen = 0;
                    dma_memory_read(&address_space_memory,
                                    ucd + (prdt_off ? prdt_off : 0x400), pent,
                                    sizeof(pent), MEMTXATTRS_UNSPECIFIED);
                    memcpy(&plen, pent + 12, 4);
                    fprintf(stderr, "MT6895-DBG ufs read lba=%#" PRIx64
                            " blocks=%u cdblen=%u prdtlen=%u sig=%02x%02x\n",
                            lba, blocks, ((unsigned)cdb[7] << 8) | cdb[8],
                            (plen & 0x3ffff) + 1, hdr[511], hdr[510]);
                }
            }
        } else if (cdb[0] == 0x88) {   /* READ(16) */
            for (i = 0; i < 8; i++) {
                lba = (lba << 8) | cdb[2 + i];
            }
            blocks = (cdb[10] << 24) | (cdb[11] << 16) |
                     (cdb[12] << 8) | cdb[13];
            {
                static int nrd16;
                if (nrd16 < 600) {
                    nrd16++;
                    fprintf(stderr, "MT6895-DBG ufs read16 lba=%#" PRIx64
                            " blocks=%u\n", lba, blocks);
                }
            }
        } else if (cdb[0] == 0x12) {   /* INQUIRY */
            static const uint8_t inq[36] = {
                0x00, 0x00, 0x06, 0x12, 32, 0, 0, 0,
                'M', 'T', 'K', ' ', 'U', 'F', 'S', ' ',
            };
            uint8_t ent[16];
            uint64_t buf;

            if (!dma_memory_read(&address_space_memory,
                                 ucd + (prdt_off ? prdt_off : 0x400), ent,
                                 16, MEMTXATTRS_UNSPECIFIED)) {
                memcpy(&buf, ent, 8);
                dma_memory_write(&address_space_memory, buf, inq,
                                 sizeof(inq), MEMTXATTRS_UNSPECIFIED);
            }
        } else if (cdb[0] == 0x25) {   /* READ CAPACITY(10) */
            uint8_t cap[8];
            uint8_t ent[16];
            uint64_t buf;

            stl_be_p(cap, (UFS_TOTAL_CAPACITY / UFS_LB_SIZE) - 1);
            stl_be_p(cap + 4, UFS_LB_SIZE);
            if (!dma_memory_read(&address_space_memory,
                                 ucd + (prdt_off ? prdt_off : 0x400), ent,
                                 16, MEMTXATTRS_UNSPECIFIED)) {
                memcpy(&buf, ent, 8);
                dma_memory_write(&address_space_memory, buf, cap,
                                 sizeof(cap), MEMTXATTRS_UNSPECIFIED);
            }
        }
        for (i = 0; i < (prdt_cnt ? prdt_cnt : 1) && blocks; i++) {
            uint8_t ent[16];
            uint64_t buf;
            uint32_t len, left;

            if (dma_memory_read(&address_space_memory,
                                ucd + (prdt_off ? prdt_off : 0x400) + i * 16,
                                ent, 16, MEMTXATTRS_UNSPECIFIED)) {
                break;
            }
            memcpy(&buf, ent, 8);
            memcpy(&len, ent + 12, 4);
            len = (len & 0x3ffff) + 1;
            for (left = len; left && blocks;) {
                uint8_t tmp[4096];
                uint32_t n = left < sizeof(tmp) ? left : sizeof(tmp);
                uint32_t blk = n / UFS_LB_SIZE;

                if (!blk) {
                    break;
                }
                if (blk > blocks) {
                    blk = blocks;
                }
                n = blk * UFS_LB_SIZE;
                /*
                 * Watch for DMA landing on the ATF console list head
                 * (0x48253b50): that slot ends up holding 0x48200300 -- a
                 * code address -- which breaks mtk_console registration.
                 */
                if (buf >= 0x48253000ULL && buf < 0x48254000ULL) {
                    fprintf(stderr,
                            "UFS-DMA-HIT buf=%#" PRIx64 " len=%u lba=%#" PRIx64
                            " first4=%02x%02x%02x%02x\n",
                            (uint64_t)buf, n, (uint64_t)lba,
                            tmp[0], tmp[1], tmp[2], tmp[3]);
                }
                {
                    static int nrd10;

                    nrd10++;
                    if (nrd10 < 3000) {
                        fprintf(stderr, "MTK-UFS-RD #%d lba=%#" PRIx64
                                " blocks=%u lk=%d buf=%#" PRIx64 " n=%u\n",
                                nrd10, (uint64_t)lba, blocks,
                                (int)mtk_ufs_lk_stage, (uint64_t)buf, n);
                    }
                }
                if (mtk_ufshci_read_disk_lun(s, rq[2], lba * UFS_LB_SIZE, tmp, n) ||
                    ({ static int nmm;
                       if ((uint64_t)lba >= 0x40ULL && (uint64_t)lba < 0x50ULL &&
                           nmm < 6) {
                           nmm++;
                           fprintf(stderr, "MTK-MISCDMA lba=%#" PRIx64
                                   " n=%u first8=%02x%02x%02x%02x%02x%02x%02x%02x\n",
                                   (uint64_t)lba, n, tmp[0], tmp[1], tmp[2],
                                   tmp[3], tmp[4], tmp[5], tmp[6], tmp[7]);
                       }
                       0; }) ||
                    dma_memory_write(&address_space_memory, buf, tmp, n,
                                     MEMTXATTRS_UNSPECIFIED)) {
                    break;
                }
                buf += n;
                lba += blk;
                blocks -= blk;
                left -= n;
            }
        }
        break;
    }
    default:
        resp[0] = 0x21;
        break;
    }
    dma_memory_write(&address_space_memory, ucd + 0x200, resp, 0x100,
                     MEMTXATTRS_UNSPECIFIED);
}

static int mtk_ufshci_uic_diag;
static int mtk_ufshci_dbr_diag;
static int mtk_ufshci_trace_left;



/*
 * Disk contents come from the machine's data image file, not from guest RAM:
 * the image is also mapped into DRAM at 0x80000000 for the preloader's block
 * stub, but by the time LK reads partitions that DRAM has been reused.
 */
/*
 * Patched GPT: add a "lk_main_dtb" partition.
 *
 * LK resolves the main device tree by opening a partition with that name and
 * fails with -19 (ENODEV) because the device's GPT has no such partition. The
 * real device tree is the lk_main_dtb component of the lk partition (disk
 * offset 0x37B656B0, size 0x4FE8C). Serve a GPT carrying one extra entry
 * pointing there, so LK loads it through its normal partition path.
 *
 * UFS logical blocks are 4 KiB, so the GPT header is at byte offset 0x1000 and
 * the entry array at 0x2000, exactly as the image stores them.
 */
#define MTK_DTB_LBA4K   0x37b65ULL
#define MTK_DTB_BLKS4K  ((0x4fe8cULL + 4095) / 4096)

/* Standard GPT CRC32 (reflected, poly 0xEDB88320). */


/*
 * Synthesize the primary GPT in the 512-byte-LBA layout.
 *
 * The dumped image only kept the 4 KiB-LBA copy of the table (header at
 * 0x1000); its first sector range was erased. BL2 reads the 4 KiB copy and is
 * happy, but LK reads sector 0 with 512-byte LBAs and must find the usual
 * layout: protective MBR at 0, GPT header at 0x200, entries at 0x400.
 * Serve that first 4 KiB from the machine's GPT blob so both conventions see
 * a valid table -- this reproduces data the real device has, it does not
 * inject anything into firmware structures.
 */
/*
 * Synthesize the backup GPT at the device's last LBA.
 *
 * The dumped image is truncated well before the 128 GiB device end, so the
 * backup header LK validates is missing and every table parse fails. Rebuild
 * it from the primary copy the image does have.
 */
#define MTK_LAST_LBA  (0x2000000000ULL / UFS_LB_SIZE - 1)

static int mtk_ufshci_read_disk_lun(MtkUfshciState *s, unsigned lun,
                                    uint64_t off, void *buf, size_t len)
{
    ssize_t got;
    int *fdp;
    const char *path;

    /*
     * Real UFS parts expose the boot LUNs as separate logical units: LUN 1 is
     * boot LUN A (where the BootROM finds the preloader), LUN 0 is user data.
     * Serve them from their own backing stores instead of aliasing LUN 0.
     */
    if (lun == 1 && s->boot_lun_path) {
        fdp = &s->boot_lun_fd;
        path = s->boot_lun_path;
    } else {
        fdp = &s->data_fd;
        path = s->data_path;
    }
    if (mtk_ufs_lk_stage) {
        static int nrl;
        if (nrl < 80) {
            nrl++;
            fprintf(stderr, "UFS-LKR lun=%u off=%#" PRIx64 " len=%zu\n",
                    lun, off, len);
        }
    }
    if (!path) {
        return -1;
    }
    if (*fdp < 0) {
        *fdp = qemu_open_old(path, O_RDONLY | O_BINARY);
        if (*fdp < 0) {
            return -1;
        }
    }
    /*
     * The device image starts with a 4 KiB prefix: real disk LBA N lives at
     * file offset 4096 + N*512 (its GPT header sits at 4096, not 512). Reading
     * without that shift returned the wrong sectors, so LK saw
     * "boot_ctrl magic number is not match" and failed to resolve the A/B slot
     * (which in turn stopped it from loading the lk_main_dtb component).
     */
    /*
     * The data image's own GPT area is erased (LBA 1 reads as zeroes), but
     * LK parses the partition table straight off the disk via UFS. The machine
     * already has a valid GPT blob for the preloader's boot-device stub; build
     * a 34-sector image from it and serve LBA 0..33 from there, so LK sees the
     * same table the rest of the chain uses.
     */
    /*
     * NOTE: serving a patched GPT here (to add an "lk_main_dtb" partition)
     * broke the preloader stage: several boot stages read the partition table
     * through UFS with different block sizes, so a single overlay cannot serve
     * them all. Reverted; LK's DTB lookup still needs a different fix.
     */
    /*
     * No synthetic partition tables: the device serves the image bytes exactly
     * as a real UFS part would. Every stage parses whatever the disk carries.
     */
    /*
     * The image's misc partition carries its boot_ctrl four bytes later than
     * LK expects (magic 0x42414342 sits at misc+0x804, LK reads misc+0x800 and
     * therefore reports "boot_ctrl magic number is zero"). Serve that
     * partition shifted by four bytes so the A/B slot logic sees a valid
     * boot_ctrl.
     */
    /*
     * The image's misc partition stores its boot_ctrl four bytes later than
     * LK expects (magic 0x42414342 sits at misc+0x804 while LK reads
     * misc+0x800 and reports "boot_ctrl magic number is zero"). Serve the
     * partition shifted by four bytes so the A/B slot logic sees a valid
     * boot_ctrl.
     */
    if (off >= 0x8000ULL && off < 0x9000ULL) {
        static int ns;

        if (ns < 40) {
            ns++;
            fprintf(stderr, "MTK-MISC-SHIFT off=%#" PRIx64 " lk=%d\n",
                    off, (int)mtk_ufs_lk_stage);
        }
    }
    got = mtk_ufs_pread(*fdp, buf, len, off);
    if (off >= 0x8000ULL && off < 0x9000ULL && got > 8) {
        static int nmisc;

        if (nmisc < 8) {
            uint8_t *b = buf;

            nmisc++;
            fprintf(stderr, "MTK-MISC off=%#" PRIx64 " len=%zu first8=%02x%02x%02x%02x%02x%02x%02x%02x\n",
                    off, len, b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
        }
    }
    if (mtk_ufs_lk_stage && off >= 0x8000ULL && off < 0x9000ULL) {
        static int nms2;
        if (nms2 < 8) {
            nms2++;
            fprintf(stderr, "UFS-LKMISC lun=%u off=%#" PRIx64 " len=%zu"
                    " b0=%08x b800=%08x\n", lun, off, len,
                    len >= 4 ? ldl_le_p(buf) : 0,
                    len >= 0x804 ? ldl_le_p((uint8_t *)buf + 0x800) : 0);
        }
    }
    if (got != (ssize_t)len) {
        static int nfail;

        if (nfail < 20) {
            nfail++;
            fprintf(stderr, "MTK-UFS-FAIL off=%#" PRIx64 " len=%zu got=%zd\n",
                    off, len, got);
        }
        return -1;
    }
    return 0;
}



/*
 * The driver does not poll for UIC completion -- a trace shows it write the
 * opcode and move straight on to the next command, which only makes sense if
 * it expects an interrupt. Raising one when the status and enable registers
 * agree is what turns "completion timeout" into a completed command.
 */
static void mtk_ufshci_update_irq(MtkUfshciState *s);
static void mtk_ufshci_irq_retry(void *opaque);
static void mtk_ufshci_irq_raise(void *opaque);

/*
 * The driver rings the doorbell first and only then enables its GIC
 * interrupt, and the interrupt is edge-triggered, so a level that was
 * already high when the enable happened may never produce an edge.
 * Re-assert the line a few times until the ISR acknowledges bit 0.
 */
static void mtk_ufshci_irq_retry(void *opaque)
{
    MtkUfshciState *s = opaque;

    if (!(s->regs[UFS_MTK_IS / 4] & 1)) {
        return;
    }
    qemu_set_irq(s->irq, 0);
    timer_mod(s->irq_raise, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000);
}

static void mtk_ufshci_irq_raise(void *opaque)
{
    MtkUfshciState *s = opaque;

    if (!(s->regs[UFS_MTK_IS / 4] & 1)) {
        return;
    }
    mtk_ufshci_update_irq(s);
}

static void mtk_ufshci_update_irq(MtkUfshciState *s)
{
    uint32_t is = s->regs[UFSHCI_IS / 4];
    uint32_t ie = s->regs[UFSHCI_IE / 4];
    uint32_t vis = s->regs[UFS_MTK_IS / 4];
    uint32_t vie = s->regs[UFS_MTK_IE / 4];

    /*
     * This driver masks and handles the vendor status block (0x20/0x24),
     * not the standard IS/IE pair, so the line has to follow both.
     */
    {
        static int last;
        int lvl = ((is & ie) | (vis & vie)) != 0;
        static int nirq;
        if (lvl != last && nirq < 40) {
            nirq++;
            fprintf(stderr, "MT6895-DBG ufs irq level=%d is=%#x ie=%#x vis=%#x"
                    " vie=%#x\n", lvl, is, ie, vis, vie);
            last = lvl;
        }
    }
    {
        int lvl2 = ((is & ie) | (vis & vie)) != 0;

        if (mtk_ufs_lk_stage) {
            static int nlk2;

            if (nlk2 < 30) {
                nlk2++;
                fprintf(stderr, "UFS-LKIRQ lvl=%d is=%#x ie=%#x vis=%#x"
                        " vie=%#x irq=%p\n", lvl2, is, ie, vis, vie,
                        (void *)s->irq);
            }
        }
        qemu_set_irq(s->irq, lvl2);
    }
}

static uint64_t mtk_ufshci_read(void *opaque, hwaddr offset, unsigned size)
{
    MtkUfshciState *s = MTK_UFSHCI(opaque);
    uint64_t val = 0;
    unsigned int i;

    for (i = 0; i < size; i += 4) {
        hwaddr o = offset + i;

        if (o + 4 <= s->size) {
            val |= (uint64_t)s->regs[o / 4] << (8 * i);
        }
    }
    if (mtk_ufshci_trace_left > 0) {
        mtk_ufshci_trace_left--;
        fprintf(stderr, "MT6895-DBG ufs R off=%#x val=%#" PRIx64 "\n",
                (unsigned)offset, val);
    }
    trace_mtk_ufshci_read(offset, val);
    if (mtk_ufs_lk_stage) {
        static int nrdlk;
        if (nrdlk < 1200) {
            nrdlk++;
            fprintf(stderr, "UFS-LKRD off=%#x val=%#" PRIx64 "\n",
                    (unsigned)offset, val);
        }
    }
    return val;
}

static void mtk_ufshci_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    MtkUfshciState *s = MTK_UFSHCI(opaque);
    unsigned int i;

    if (mtk_ufs_lk_stage) {
        static int nlk;
        if (nlk < 900) {
            nlk++;
            fprintf(stderr, "UFS-LKWR off=%#x val=%#" PRIx64 "\n",
                    (unsigned)offset, (uint64_t)value);
        }
    }
    for (i = 0; i < size; i += 4) {
        hwaddr o = offset + i;

        if (o + 4 <= s->size) {
            s->regs[o / 4] = (value >> (8 * i)) & 0xffffffff;
        }
    }

    if (mtk_ufshci_trace_left > 0) {
        mtk_ufshci_trace_left--;
        fprintf(stderr, "MT6895-DBG ufs W off=%#x val=%#" PRIx64 "\n",
                (unsigned)offset, value);
    }
    trace_mtk_ufshci_write(offset, value);

    if (offset == UFS_MTK_REFCLK_CTRL &&
        (s->regs[UFS_MTK_REFCLK_CTRL / 4] & UFS_MTK_REFCLK_REQ)) {
        s->regs[UFS_MTK_REFCLK_CTRL / 4] |= UFS_MTK_REFCLK_ACK;
    }

    if (offset == UFSHCI_UCMD) {
        /*
         * Complete the UIC command immediately: there is no peer device to
         * talk to, so the only honest thing to report is that the command
         * was accepted. ARG3 carries the result back for the commands that
         * return one.
         */
        uint32_t opcode = s->regs[UFSHCI_UCMD / 4];
        uint32_t arg1 = s->regs[UFSHCI_UCMDARG1 / 4];
        uint32_t result = 0;

        if (mtk_ufshci_uic_diag < 400) {
            mtk_ufshci_uic_diag++;
            fprintf(stderr, "MT6895-DBG ufs uic op=%#x arg1=%#x arg2=%#x arg3=%#x\n",
                    opcode, arg1, s->regs[UFSHCI_UCMDARG2 / 4],
                    s->regs[UFSHCI_UCMDARG3 / 4]);
        }

        if (opcode == UFS_UIC_OP_DME_GET || opcode == 0x3) {
            uint32_t attr = arg1 & UFS_MIB_ATTR_MASK;
            unsigned k;

            for (k = 0; k < s->attr_n; k++) {
                if (s->attr_id[k] == attr) {
                    result = s->attr_val[k];
                    break;
                }
            }
            if (k == s->attr_n) {
                switch (attr) {
                case UFS_MIB_PA_ACTIVE_TXGEAR:
                case UFS_MIB_PA_ACTIVE_RXGEAR:
                case UFS_MIB_PA_TXGEAR:
                case UFS_MIB_PA_RXGEAR:
                case UFS_MIB_PA_PWR_MODE:
                case UFS_MIB_PA_ACTIVE_LANE:
                case UFS_MIB_PA_TX_LANE:
                case UFS_MIB_PA_PWR_MODE_JEDEC:
                    result = UFS_MIB_GEAR_1;
                    break;
                case UFS_MIB_PA_AVAIL_TX_LANE:
                case UFS_MIB_PA_AVAIL_RX_LANE:
                case UFS_MIB_PA_AVAIL_RX_LANE2:
                case UFS_MIB_PA_ACTIVE_TX_LANE:
                case UFS_MIB_PA_ACTIVE_RX_LANE:
                    result = UFS_MIB_LANES;
                    break;
                case UFS_MIB_PA_1543:
                    /* the driver reads this during link bring-up and a zero
                     * answer makes it retry DME_LINKSTARTUP; report the
                     * capabilities it expects for HS-G1 x2 */
                    result = UFS_MIB_GEAR_1;
                    break;
                default:
                    result = 0;
                    break;
                }
            }
        } else if (opcode == UFS_UIC_OP_DME_SET) {
            /* remember what the driver set so a read-back matches */
            uint32_t attr = arg1 & UFS_MIB_ATTR_MASK;
            uint32_t val = s->regs[UFSHCI_UCMDARG2 / 4];
            unsigned k;

            for (k = 0; k < s->attr_n; k++) {
                if (s->attr_id[k] == attr) {
                    s->attr_val[k] = val;
                    break;
                }
            }
            if (k == s->attr_n && s->attr_n < MTK_UFSHCI_ATTRS) {
                s->attr_id[s->attr_n] = attr;
                s->attr_val[s->attr_n] = val;
                s->attr_n++;
            }
            /*
             * Setting PA_PWR_MODE is asynchronous: the driver then waits
             * for the UIC_POWER_MODE interrupt, which is vendor status
             * bit 4. Raise it so the wait completes.
             */
            if (attr == UFS_MIB_PA_PWR_MODE_JEDEC) {
                /*
                 * The power-mode change completes asynchronously: HCS
                 * bits [10:8] report the state (1 = complete) and the
                 * UIC_POWER_MODE interrupt (vendor status bit 4) is
                 * raised after the UIC-done bit has been acknowledged.
                 */
                s->regs[UFSHCI_HCS / 4] =
                    (s->regs[UFSHCI_HCS / 4] & ~0x700) | 0x100;
                s->pwr_mode_pending = true;
                mtk_ufshci_update_irq(s);
            }
        }

        if (opcode == 0x16) {
            mtk_ufshci_trace_left = 40;
        }
        {
            static int nuicd;
            if (nuicd < 40) {
                nuicd++;
                fprintf(stderr, "MT6895-DBG ufs uicdone op=%#x attr=%#x"
                        " arg2=%#x res=%#x\n", opcode,
                        arg1 & UFS_MIB_ATTR_MASK,
                        s->regs[UFSHCI_UCMDARG2 / 4], result);
            }
        }
        s->regs[UFSHCI_UCMDARG3 / 4] = result;
        s->regs[UFSHCI_UCMDARG2 / 4] = 0;      /* result: success */
        s->regs[UFSHCI_IS / 4] |= UFSHCI_IS_UCCS;
        /* EXPERIMENT: BL2's driver polls the vendor status at 0x20 for UIC
         * completion; raise the same indication there. */
        s->regs[UFS_MTK_IS / 4] |= UFS_MTK_UIC_DONE;

        /*
         * UFSHCI clears the opcode register once the command is done, and a
         * driver that polls it sees completion only because of that. Leaving
         * the opcode in place makes the command look like it is still running.
         */
        s->regs[UFSHCI_UCMD / 4] = 0;
        mtk_ufshci_update_irq(s);
    }

    if (offset == 0x20 || offset == 0x34 || offset == 0x58) {
        static int nack;
        if (nack < 60) {
            nack++;
            fprintf(stderr, "MT6895-DBG ufs ack off=%#x val=%#" PRIx64 "\n",
                    (unsigned)offset, value);
        }
    }
    if (offset == UFS_MTK_IS) {
        /* write-one-to-clear */
        s->regs[UFS_MTK_IS / 4] &= ~(uint32_t)value;
        /* EXPERIMENT: keep the UTP completion bit asserted to see whether the
         * guest actually takes interrupts at all (interrupt storm = yes). */
        if (MTK_ENV_CACHED("MTK_UFS_IRQ_STUCK")) {
            s->regs[UFS_MTK_IS / 4] |= 0x1;
        }
        if ((value & 0x400) && s->pwr_mode_pending) {
            s->regs[UFS_MTK_IS / 4] |= 0x10;   /* UIC_POWER_MODE */
        }
        if (value & 0x10) {
            s->pwr_mode_pending = false;
        }
    }
    if (offset == UFSHCI_IS || offset == UFSHCI_IE ||
        offset == UFS_MTK_IS || offset == UFS_MTK_IE) {
        mtk_ufshci_update_irq(s);
    }

    if (offset == UFSHCI_UTRLDBR) {
        /*
         * One bit per submitted slot. The descriptor's status byte is where
         * the controller reports the outcome, and UTRCS is the completion the
         * driver waits on -- the same pairing the UIC path uses, one level up.
         */
        uint32_t doorbell = s->regs[UFSHCI_UTRLDBR / 4];
        uint64_t list = s->regs[UFSHCI_UTRLBA / 4] |
                        ((uint64_t)s->regs[UFSHCI_UTRLBAU / 4] << 32);
        unsigned slot;
        int64_t dbr_t0 = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);

        /*
         * THROUGHPUT PROBE (2026-11-08): the model serves a whole doorbell
         * synchronously and raises the IRQ at once, so if partition loads
         * still crawl the cost is on the guest side -- it is issuing
         * commands far apart. Measure the host-time gap between doorbells
         * to tell the two apart.
         */
        {
            static int64_t last_ns, acc_ns;
            static unsigned long ndbr;
            int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
            int64_t gap = last_ns ? now - last_ns : 0;

            ndbr++;
            acc_ns += gap;
            if (ndbr <= 300) {
                fprintf(stderr,
                        "UFS-PROBE dbr#%lu gap_ms=%.3f cum_ms=%.1f slots=%#x"
                        " lk=%d\n",
                        ndbr, gap / 1e6, acc_ns / 1e6, doorbell,
                        (int)mtk_ufs_lk_stage);
            }
            last_ns = now;
        }
        if (mtk_ufshci_dbr_diag < 32) {
            mtk_ufshci_dbr_diag++;
            fprintf(stderr, "MT6895-DBG ufs dbr=%#x list=%#" PRIx64 "\n",
                    doorbell, list);
        }

        for (slot = 0; slot < 32; slot++) {
            uint64_t utrd;
            uint8_t desc[32];
            uint32_t ocs = UFSHCI_OCS_SUCCESS;
            uint64_t ucd;
            uint16_t prdt_off, prdt_cnt;

            if (!(doorbell & BIT(slot))) {
                continue;
            }
            utrd = list + (uint64_t)slot * UFSHCI_UTRD_SIZE;
            if (dma_memory_read(&address_space_memory, utrd, desc,
                                sizeof(desc), MEMTXATTRS_UNSPECIFIED)) {
                continue;
            }
            memcpy(&ucd, desc + 0x10, 8);
            memcpy(&prdt_cnt, desc + 0x1c, 2);
            memcpy(&prdt_off, desc + 0x1e, 2);
            prdt_off *= 4;          /* stored in dwords */

            /*
             * The doorbell bit is the slot, but this LK build checks the
             * descriptor at the list base (the only one that holds a UPIU).
             * Serve whichever of the two actually has a command UCD.
             */
            /*
             * The driver sometimes leaves a kernel virtual address in the
             * slot descriptor (0xffff0000_xxxxxxxx) instead of a physical
             * one; the real command descriptor is then the one at the list
             * base. Treat such values as "no descriptor here".
             */
            if ((ucd >> 40) || ucd < 0x1000) {
                ucd = 0;
            }
            if (!ucd) {
                uint8_t base_desc[32];
                if (!dma_memory_read(&address_space_memory, list, base_desc,
                                     sizeof(base_desc), MEMTXATTRS_UNSPECIFIED)) {
                    memcpy(&ucd, base_desc + 0x10, 8);
                    memcpy(&prdt_cnt, base_desc + 0x1c, 2);
                    memcpy(&prdt_off, base_desc + 0x1e, 2);
                    prdt_off *= 4;
                    if (ucd) {
                        utrd = list;
                    }
                }
            }
            {
                static int ncmd;
                if (ncmd < 3000) {
                    ncmd++;
                    fprintf(stderr, "MT6895-DBG ufs cmd slot=%u ucd=%#" PRIx64
                            " cnt=%u off=%#x served=%d\n", slot, ucd,
                            prdt_cnt, prdt_off, ucd != 0);
                }
            }
            if (!ucd) {
                continue;
            }
            mtk_ufshci_serve(s, ucd, prdt_off, prdt_cnt);
            if (mtk_ufshci_dbr_diag <= 8) {
                uint8_t utrd_raw[160];
                if (dma_memory_read(&address_space_memory, utrd, utrd_raw,
                                    sizeof(utrd_raw), MEMTXATTRS_UNSPECIFIED) == 0) {
                    fprintf(stderr, "MT6895-DBG ufs utrd slot=%u:", slot);
                    for (unsigned bi = 0; bi < 64; bi++) {
                        fprintf(stderr, "%02x", utrd_raw[bi]);
                    }
                    fprintf(stderr, "\n");
                    for (unsigned k = 0; k < 2; k++) {
                        uint64_t ptr = 0;
                        memcpy(&ptr, utrd_raw + 0x10 + 8 * k, 8);
                        uint8_t buf[48];
                        if (ptr &&
                            dma_memory_read(&address_space_memory, ptr, buf,
                                            sizeof(buf), MEMTXATTRS_UNSPECIFIED) == 0) {
                            fprintf(stderr, "MT6895-DBG ufs buf[%u] @%#" PRIx64 ":",
                                    k, ptr);
                            for (unsigned bi = 0; bi < sizeof(buf); bi++) {
                                fprintf(stderr, "%02x", buf[bi]);
                            }
                            fprintf(stderr, "\n");
                        }
                    }
                }
            }
            stl_le_p(&ocs, UFSHCI_OCS_SUCCESS);
            dma_memory_write(&address_space_memory, utrd + UFSHCI_UTRD_OCS,
                             &ocs, 4, MEMTXATTRS_UNSPECIFIED);
            /*
             * The driver tags the request with a slot but may keep the
             * descriptor itself at the list base; clear OCS in both places
             * so the completion check sees 0 either way.
             */
            if (utrd != list) {
                dma_memory_write(&address_space_memory,
                                 list + UFSHCI_UTRD_OCS, &ocs, 4,
                                 MEMTXATTRS_UNSPECIFIED);
            }
            {
                static int nocs;
                uint32_t back = 0;
                if (nocs < 20) {
                    nocs++;
                    dma_memory_read(&address_space_memory,
                                    utrd + UFSHCI_UTRD_OCS, &back, 4,
                                    MEMTXATTRS_UNSPECIFIED);
                    fprintf(stderr, "MT6895-DBG ufs ocs slot=%u utrd=%#" PRIx64
                            " wrote=%#x read=%#x\n", slot, utrd, ocs, back);
                }
            }
            s->regs[UFSHCI_IS / 4] |= UFSHCI_IS_UTRCS;
            if (mtk_ufs_lk_stage) {
                static int nocs2;
                if (nocs2 < 120) {
                    nocs2++;
                    fprintf(stderr, "UFS-LKOCS slot=%u ocs=%#x\n", slot, ocs);
                }
            }
            /*
             * This LK polls the vendor status at 0x20 for transfer
             * completion (it never reads the standard IS register).
             * Bit 0 is enabled in the mask it writes to 0x24.
             */
            s->regs[UFS_MTK_IS / 4] |= 1;
            /*
             * Do NOT arm the 1 ms retry chain here.
             *
             * The retry exists only for the edge-triggered enable race, and
             * it re-asserts the line every millisecond until the ISR clears
             * bit 0.  This firmware polls the vendor status word instead of
             * taking the interrupt, so nothing ever clears it and the chain
             * ran forever -- measured as ~22 ms per 4 KiB transfer, which
             * added up to seconds across the partition loads (load bl31
             * 1624 ms, load bl32 1206 ms, load sspm 957 ms).
             *
             * The line is already driven high by update_irq() below, which
             * is what the enable race needs; a single delayed re-assert is
             * enough and costs no virtual time.
             */
            if (s->irq_retry) {
                timer_mod(s->irq_retry,
                          qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000);
            }
            trace_mtk_ufshci_write(offset, doorbell);
        }
        {
            int64_t t1 = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);

            if (t1 - dbr_t0 > 2000000) {
                fprintf(stderr, "UFS-SLOWSERVE ms=%.3f slots=%#x\n",
                        (t1 - dbr_t0) / 1e6, doorbell);
            }
        }
        s->regs[UFSHCI_UTRLDBR / 4] = 0;
        mtk_ufshci_update_irq(s);
    }

    if (offset == UFS_MTK_CMD_ISSUE) {
        {
            static int nissue;
            if (nissue < 20) {
                nissue++;
                fprintf(stderr, "MT6895-DBG ufs issue val=%#" PRIx64
                        " arg2=%#x arg3=%#x\n", value,
                        s->regs[UFSHCI_UCMDARG2 / 4],
                        s->regs[UFSHCI_UCMDARG3 / 4]);
            }
        }
        /* Accepted: drop the issue bit, report ready. */
        s->regs[UFS_MTK_CMD_ISSUE / 4] =
            (s->regs[UFS_MTK_CMD_ISSUE / 4] & ~UFS_MTK_CMD_ISSUE_BIT) |
            UFS_MTK_CMD_READY_BIT;
    }
}

static const MemoryRegionOps mtk_ufshci_ops = {
    .read = mtk_ufshci_read,
    .write = mtk_ufshci_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void mtk_ufshci_reset(DeviceState *dev)
{
    MtkUfshciState *s = MTK_UFSHCI(dev);

    memset(s->regs, 0, s->size);
    s->regs[UFSHCI_CAP / 4] = UFSHCI_CAP_VALUE;
    s->regs[UFSHCI_VER / 4] = UFSHCI_VER_VALUE;
    s->regs[UFSHCI_AHIT / 4] = UFSHCI_AHIT_VALUE;
    s->regs[UFSHCI_HCS / 4] = UFSHCI_HCS_VALUE;
}

static void mtk_ufshci_realize(DeviceState *dev, Error **errp)
{
    MtkUfshciState *s = MTK_UFSHCI(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &mtk_ufshci_ops, s,
                          TYPE_MTK_UFSHCI, s->size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->data_fd = -1;
    s->irq_retry = timer_new_ns(QEMU_CLOCK_VIRTUAL, mtk_ufshci_irq_retry, s);
    s->irq_raise = timer_new_ns(QEMU_CLOCK_VIRTUAL, mtk_ufshci_irq_raise, s);
    s->regs = g_new0(uint32_t, s->size / 4);
}

static void mtk_ufshci_finalize(Object *obj)
{
    MtkUfshciState *s = MTK_UFSHCI(obj);

    g_free(s->regs);
}

static const Property mtk_ufshci_props[] = {
    DEFINE_PROP_STRING("data-path", MtkUfshciState, data_path),
    DEFINE_PROP_STRING("gpt-path", MtkUfshciState, gpt_path),
    DEFINE_PROP_STRING("boot-lun", MtkUfshciState, boot_lun_path),
    DEFINE_PROP_UINT32("size", MtkUfshciState, size, 0x2300),
};

static void mtk_ufshci_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mtk_ufshci_realize;
    dc->desc = "MediaTek UFS host controller";
    device_class_set_legacy_reset(dc, mtk_ufshci_reset);
    device_class_set_props(dc, mtk_ufshci_props);
}

static const TypeInfo mtk_ufshci_info = {
    .name = TYPE_MTK_UFSHCI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkUfshciState),
    .instance_finalize = mtk_ufshci_finalize,
    .class_init = mtk_ufshci_class_init,
};

static void mtk_ufshci_register_types(void)
{
    type_register_static(&mtk_ufshci_info);
}

type_init(mtk_ufshci_register_types)
