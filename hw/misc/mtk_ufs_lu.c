/*
 * QEMU UFS Logical Unit
 *
 * Copyright (c) 2023 Samsung Electronics Co., Ltd. All rights reserved.
 *
 * Written by Jeuk Kim <jeuk20.kim@samsung.com>
 *
 * This code is licensed under the GNU GPL v2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/mtk-env.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qemu/memalign.h"
#include "hw/scsi/scsi.h"
#include "scsi/constants.h"
#include "crypto/hmac.h"
#include "qemu/bswap.h"
#include "system/dma.h"
#include "system/block-backend.h"
#include "system/dma.h"
#include "qemu/cutils.h"
#include "trace.h"
#include "mtk_ufs.h"
#include "block/block-common.h"

#define SCSI_COMMAND_FAIL (-1)

static void mtk_ufs_build_upiu_sense_data(UfsRequest *req, uint8_t *sense,
                                      uint32_t sense_len)
{
    req->rsp_upiu.sr.sense_data_len = cpu_to_be16(sense_len);
    assert(sense_len <= SCSI_SENSE_LEN);
    memcpy(req->rsp_upiu.sr.sense_data, sense, sense_len);
}

static void mtk_ufs_build_scsi_response_upiu(UfsRequest *req, uint8_t *sense,
                                         uint32_t sense_len,
                                         uint32_t transfered_len,
                                         int16_t status)
{
    uint32_t expected_len = be32_to_cpu(req->req_upiu.sc.exp_data_transfer_len);
    uint8_t flags = 0, response = UFS_COMMAND_RESULT_SUCCESS;
    uint16_t data_segment_length;

    if (expected_len > transfered_len) {
        req->rsp_upiu.sr.residual_transfer_count =
            cpu_to_be32(expected_len - transfered_len);
        flags |= UFS_UPIU_FLAG_UNDERFLOW;
    } else if (expected_len < transfered_len) {
        req->rsp_upiu.sr.residual_transfer_count =
            cpu_to_be32(transfered_len - expected_len);
        flags |= UFS_UPIU_FLAG_OVERFLOW;
    }

    if (status != 0) {
        mtk_ufs_build_upiu_sense_data(req, sense, sense_len);
        response = UFS_COMMAND_RESULT_FAIL;
    }

    data_segment_length = sense_len + sizeof(req->rsp_upiu.sr.sense_data_len);
    mtk_ufs_build_upiu_header(req, UFS_UPIU_TRANSACTION_RESPONSE, flags, response,
                          status, data_segment_length);
}

#define UFS_GROUP_NUMBER_MASK 0x1F
#define UFS_WB_GROUP_NUMBER_DEFAULT 0x00 /* 00000b */
#define UFS_WB_GROUP_NUMBER_PINNED 0x18 /* 11000b */
static bool mtk_ufs_wb_check_write_pinned(UfsHc *u, UfsRequest *req)
{
    uint8_t cmd = req->req_upiu.sc.cdb[0];
    uint8_t group_number = UFS_WB_GROUP_NUMBER_DEFAULT;

    if (u->attributes.wb_buffer_partial_flush_mode != UFS_WB_FLUSH_PINNED) {
        return false;
    }

    if (cmd == WRITE_16) {
        group_number = req->req_upiu.sc.cdb[14] & UFS_GROUP_NUMBER_MASK;

    } else if (cmd == WRITE_10) {
        group_number = req->req_upiu.sc.cdb[6] & UFS_GROUP_NUMBER_MASK;
    }

    return (group_number == UFS_WB_GROUP_NUMBER_PINNED);
}

static void mtk_ufs_wb_process_write_normal(UfsHc *u, uint32_t transfered_len)
{
    UfsWb *wb = &u->wb;
    uint64_t curr_bytes, used_bytes, remain_bytes;

    if (!wb->curr_bytes) {
        return;
    }

    curr_bytes = wb->curr_bytes - wb->pinned_curr_bytes;
    used_bytes = wb->used_bytes - wb->pinned_used_bytes;

    if (used_bytes >= curr_bytes) {
        return;
    }

    remain_bytes = curr_bytes - used_bytes;
    wb->used_bytes += MIN(remain_bytes, transfered_len);
}

#define UFS_WB_TOTAL_WRITTEN_DIV (10 * 1024 * 1024) /* 10MiB */
static void mtk_ufs_wb_process_write_pinned(UfsHc *u, uint32_t transfered_len)
{
    UfsWb *wb = &u->wb;
    uint64_t remain_bytes, remain_data;
    uint32_t total_written;

    if (!wb->pinned_curr_bytes) {
        mtk_ufs_wb_process_write_normal(u, transfered_len);
        return;
    }

    if (wb->pinned_used_bytes >= wb->pinned_curr_bytes) {
        mtk_ufs_wb_process_write_normal(u, transfered_len);
        return;
    }

    remain_bytes = wb->pinned_curr_bytes - wb->pinned_used_bytes;
    if (remain_bytes >= transfered_len) {
        wb->pinned_total_written_bytes += transfered_len;
        wb->pinned_used_bytes += transfered_len;
        wb->used_bytes += transfered_len;
        remain_data = 0;

    } else {
        wb->pinned_total_written_bytes += remain_bytes;
        wb->pinned_used_bytes += remain_bytes;
        wb->used_bytes += remain_bytes;
        remain_data = transfered_len - remain_bytes;
    }

    total_written = wb->pinned_total_written_bytes / UFS_WB_TOTAL_WRITTEN_DIV;
    u->attributes.pinned_wb_cumm_written_size = cpu_to_be32(total_written);

    mtk_ufs_wb_process_write_normal(u, remain_data);
}

static void mtk_ufs_wb_process_write_req(UfsRequest *req, uint32_t transfered_len)
{
    UfsHc *u = req->hc;

    if (!u->flags.wb_en || !mtk_ufs_is_write_req(req)) {
        return;
    }

    if (mtk_ufs_wb_check_write_pinned(u, req)) {
        mtk_ufs_wb_process_write_pinned(u, transfered_len);
    } else {
        mtk_ufs_wb_process_write_normal(u, transfered_len);
    }

    mtk_ufs_wb_update_avail_buffer(u);
}

#define UFS_HID_MAX_FRAGMENTS 1024
static void mtk_ufs_hid_count_fragments(UfsRequest *req)
{
    UfsHc *u = req->hc;

    if (!mtk_ufs_is_write_req(req)) {
        return;
    }

    u->hid_fragment_count =
        MIN(u->hid_fragment_count + 1, UFS_HID_MAX_FRAGMENTS);
}

static void mtk_ufs_scsi_command_complete(SCSIRequest *scsi_req, size_t resid)
{
    UfsRequest *req = scsi_req->hba_private;
    int16_t status = scsi_req->status;
    uint32_t transfered_len = scsi_req->cmd.xfer - resid;

    if (MTK_ENV_CACHED("MTK_UFS_CDB")) {
        static int ncc;

        if (ncc < 30) {
            ncc++;
            fprintf(stderr, "UFSCC[%02d] status=%d xfer=%u resid=%zu"
                    " done=%u\n", ncc, (int)status,
                    (unsigned)scsi_req->cmd.xfer, resid, transfered_len);
        }
    }

    /* WB / HID accounting should only happen for successful commands */
    if (status == GOOD) {
        mtk_ufs_wb_process_write_req(req, transfered_len);
        mtk_ufs_hid_count_fragments(req);
    }

    mtk_ufs_build_scsi_response_upiu(req, scsi_req->sense, scsi_req->sense_len,
                                 transfered_len, status);

    if (MTK_ENV_CACHED("MTK_UFS_CDB") && status == GOOD && req->sg &&
        req->sg->sg && req->sg->nsg) {
        static int ndump;

        if (ndump < 60) {
            uint8_t hdr[16];
            QEMUSGList *sg = req->sg;

            ndump++;
            if (!dma_memory_read(&address_space_memory, sg->sg[0].base,
                                 hdr, sizeof(hdr), MEMTXATTRS_UNSPECIFIED)) {
                fprintf(stderr, "UFSDUMP[%02d] addr=%#" HWADDR_PRIx
                        " len=%u data=%02x%02x%02x%02x %02x%02x%02x%02x"
                        " | %c%c%c%c\n", ndump, sg->sg[0].base, transfered_len,
                        hdr[0], hdr[1], hdr[2], hdr[3],
                        hdr[4], hdr[5], hdr[6], hdr[7],
                        (hdr[0] >= 0x20 && hdr[0] < 0x7f) ? hdr[0] : '.',
                        (hdr[1] >= 0x20 && hdr[1] < 0x7f) ? hdr[1] : '.',
                        (hdr[2] >= 0x20 && hdr[2] < 0x7f) ? hdr[2] : '.',
                        (hdr[3] >= 0x20 && hdr[3] < 0x7f) ? hdr[3] : '.');
            }
        }
    }

    mtk_ufs_complete_req(req, UFS_REQUEST_SUCCESS);

    scsi_req->hba_private = NULL;
    scsi_req_unref(scsi_req);
}

static QEMUSGList *mtk_ufs_get_sg_list(SCSIRequest *scsi_req)
{
    UfsRequest *req = scsi_req->hba_private;
    return req->sg;
}

static const struct SCSIBusInfo mtk_ufs_scsi_info = {
    .tcq = true,
    .max_target = 0,
    .max_lun = UFS_MAX_LUS,
    .max_channel = 0,

    .get_sg_list = mtk_ufs_get_sg_list,
    .complete = mtk_ufs_scsi_command_complete,
};

static int mtk_ufs_emulate_report_luns(UfsRequest *req, uint8_t *outbuf,
                                   uint32_t outbuf_len)
{
    UfsHc *u = req->hc;
    int len = 0;

    /* TODO: Support for cases where SELECT REPORT is 1 and 2 */
    if (req->req_upiu.sc.cdb[2] != 0) {
        return SCSI_COMMAND_FAIL;
    }

    if (outbuf_len < 8) {
        return SCSI_COMMAND_FAIL;
    }
    memset(outbuf, 0, 8);
    len += 8;

    for (uint8_t lun = 0; lun < UFS_MAX_LUS; ++lun) {
        if (u->lus[lun]) {
            if (len + 8 > outbuf_len) {
                break;
            }

            memset(outbuf + len, 0, 8);
            outbuf[len] = 0;
            outbuf[len + 1] = lun;
            len += 8;
        }
    }

    /* store the LUN list length */
    stl_be_p(outbuf, len - 8);

    return len;
}

static int mtk_ufs_scsi_emulate_vpd_page(UfsRequest *req, uint8_t *outbuf,
                                     uint32_t outbuf_len)
{
    uint8_t page_code = req->req_upiu.sc.cdb[2];
    int start, buflen = 0;

    outbuf[buflen++] = TYPE_WLUN;
    outbuf[buflen++] = page_code;
    outbuf[buflen++] = 0x00;
    outbuf[buflen++] = 0x00;
    start = buflen;

    switch (page_code) {
    case 0x00: /* Supported page codes, mandatory */
    {
        outbuf[buflen++] = 0x00; /* list of supported pages (this page) */
        outbuf[buflen++] = 0x87; /* mode page policy */
        break;
    }
    case 0x87: /* Mode Page Policy, mandatory */
    {
        outbuf[buflen++] = 0x3f; /* apply to all mode pages and subpages */
        outbuf[buflen++] = 0xff;
        outbuf[buflen++] = 0; /* shared */
        outbuf[buflen++] = 0;
        break;
    }
    default:
        return SCSI_COMMAND_FAIL;
    }
    /* done with EVPD */
    assert(buflen - start <= 255);
    outbuf[start - 1] = buflen - start;
    return buflen;
}

static int mtk_ufs_emulate_wlun_inquiry(UfsRequest *req, uint8_t *outbuf,
                                    uint32_t outbuf_len)
{
    if (outbuf_len < SCSI_INQUIRY_LEN) {
        return 0;
    }

    if (req->req_upiu.sc.cdb[1] & 0x1) {
        /* Vital product data */
        return mtk_ufs_scsi_emulate_vpd_page(req, outbuf, outbuf_len);
    }

    /* Standard INQUIRY data */
    if (req->req_upiu.sc.cdb[2] != 0) {
        return SCSI_COMMAND_FAIL;
    }

    outbuf[0] = TYPE_WLUN;
    outbuf[1] = 0;
    outbuf[2] = 0x6; /* SPC-4 */
    outbuf[3] = 0x2;
    outbuf[4] = 31;
    outbuf[5] = 0;
    outbuf[6] = 0;
    outbuf[7] = 0x2;
    strpadcpy((char *)&outbuf[8], 8, "QEMU", ' ');
    strpadcpy((char *)&outbuf[16], 16, "QEMU UFS", ' ');
    memset(&outbuf[32], 0, 4);

    return SCSI_INQUIRY_LEN;
}


/*
 * ===== RPMB well-known LU (W-LUN 0xC4) — rpmb-dev =====
 *
 * JEDEC UFS (JESD220) §12.4 / eMMC RPMB semantics: SECURITY PROTOCOL OUT
 * (0xB5) carries request frames, SECURITY PROTOCOL IN (0xA2) returns the
 * response frames, protocol 0xEC, protocol-specific 0x0001 (RPMB region 0).
 * Frame (512 B, big-endian): stuff[196] key/MAC[32]@196 data[256]@228
 * nonce[16]@484 write-counter@500 address@504 block-count@506 result@508
 * req/resp@510.  MAC = HMAC-SHA256(key, bytes 228..511 of every frame).
 *
 * State (key, write counter, data) lives in memory and, when MTK_RPMB_FILE
 * names a file, is persisted there (512-byte header + 256-byte blocks).
 * The device starts with no key programmed, like a factory-fresh part.
 * MTK_NO_RPMB=1 restores the previous INVALID OPCODE behaviour.
 */
#ifndef SECURITY_PROTOCOL_IN
#define SECURITY_PROTOCOL_IN  0xa2   /* SPC-4 */
#endif
#ifndef SECURITY_PROTOCOL_OUT
#define SECURITY_PROTOCOL_OUT 0xb5
#endif
#define RPMB_FRAME      512
#define RPMB_MAC_OFS    196
#define RPMB_DATA_OFS   228
#define RPMB_NONCE_OFS  484
#define RPMB_WCNT_OFS   500
#define RPMB_ADDR_OFS   504
#define RPMB_BCNT_OFS   506
#define RPMB_RES_OFS    508
#define RPMB_TYPE_OFS   510
#define RPMB_BLK        256
#define RPMB_MAX_FRAMES 64          /* geometry bRPMB_ReadWriteSize */

enum {
    RPMB_REQ_KEY = 1, RPMB_REQ_WCNT = 2, RPMB_REQ_WRITE = 3,
    RPMB_REQ_READ = 4, RPMB_REQ_RESULT = 5,
};
enum {
    RPMB_RES_OK = 0, RPMB_RES_GENERAL = 1, RPMB_RES_AUTH = 2,
    RPMB_RES_COUNTER = 3, RPMB_RES_ADDR = 4, RPMB_RES_WRITE = 5,
    RPMB_RES_READ = 6, RPMB_RES_NO_KEY = 7, RPMB_RES_WCNT_EXPIRED = 0x80,
};

typedef struct MtkUfsRpmb {
    bool key_set;
    uint8_t key[32];
    uint32_t wcnt;
    uint32_t nblk;
    uint8_t *data;
    char *path;
    /* last request, for the following SECURITY PROTOCOL IN */
    uint16_t pend_type;         /* response type to return, 0 = none */
    uint16_t last_result;
    uint16_t last_write_type;   /* 0x0100 or 0x0300 for RESULT READ */
    uint16_t last_addr;
    uint16_t rd_addr, rd_cnt;
    uint8_t nonce[16];
} MtkUfsRpmb;

static MtkUfsRpmb *mtk_rpmb;

bool mtk_ufs_rpmb_enabled(void)
{
    return getenv("MTK_NO_RPMB") == NULL;
}

uint64_t mtk_ufs_rpmb_blocks(void)
{
    /* xaga RPMB size from the unit's own UFS unit descriptor:
     * logical_blk_size 256 bytes * logical_blk_cnt 0x10000 = 16 MiB (the
     * device boot log records both).  MTK_RPMB_MIB overrides for tests. */
    const char *e = getenv("MTK_RPMB_MIB");
    uint64_t mib = e ? strtoull(e, NULL, 0) : 16;

    if (!mib || mib > 32) {
        mib = 16;
    }
    return mib * MiB / RPMB_BLK;
}

static void mtk_rpmb_save(MtkUfsRpmb *r, int blk, int cnt)
{
    uint8_t hdr[512];
    FILE *f;

    if (!r->path) {
        return;
    }
    f = fopen(r->path, "r+b");
    if (!f) {
        f = fopen(r->path, "w+b");
    }
    if (!f) {
        fprintf(stderr, "MT6895 rpmb: cannot write %s\n", r->path);
        return;
    }
    memset(hdr, 0, sizeof(hdr));
    memcpy(hdr, "MTKRPMB1", 8);
    hdr[8] = r->key_set;
    stl_be_p(hdr + 12, r->wcnt);
    stl_be_p(hdr + 16, r->nblk);
    memcpy(hdr + 32, r->key, 32);
    fseek(f, 0, SEEK_SET);
    fwrite(hdr, 1, sizeof(hdr), f);
    if (cnt > 0) {
        fseek(f, 512 + (long)blk * RPMB_BLK, SEEK_SET);
        fwrite(r->data + (size_t)blk * RPMB_BLK, 1, (size_t)cnt * RPMB_BLK, f);
    }
    fclose(f);
}

static MtkUfsRpmb *mtk_rpmb_get(void)
{
    MtkUfsRpmb *r = mtk_rpmb;
    const char *p;

    if (r) {
        return r;
    }
    r = g_new0(MtkUfsRpmb, 1);
    r->nblk = mtk_ufs_rpmb_blocks();
    r->data = g_malloc0((size_t)r->nblk * RPMB_BLK);
    p = getenv("MTK_RPMB_FILE");
    if (p && *p) {
        FILE *f = fopen(p, "rb");

        r->path = g_strdup(p);
        if (f) {
            uint8_t hdr[512];

            if (fread(hdr, 1, sizeof(hdr), f) == sizeof(hdr) &&
                !memcmp(hdr, "MTKRPMB1", 8) && ldl_be_p(hdr + 16) == r->nblk) {
                r->key_set = hdr[8] != 0;
                r->wcnt = ldl_be_p(hdr + 12);
                memcpy(r->key, hdr + 32, 32);
                if (fread(r->data, 1, (size_t)r->nblk * RPMB_BLK, f) == 0) {
                    /* data area not written yet: stays erased (zero) */
                }
            }
            fclose(f);
        }
    }
    fprintf(stderr, "MT6895 rpmb: %u blocks (%u KiB), key %s, wcnt %u, %s\n",
            r->nblk, r->nblk / 4, r->key_set ? "programmed" : "not programmed",
            r->wcnt, r->path ? r->path : "in-memory");
    mtk_rpmb = r;
    return r;
}

/* HMAC-SHA256 over bytes 228..511 of n consecutive frames */
static bool mtk_rpmb_mac(MtkUfsRpmb *r, const uint8_t *frames, int n,
                         uint8_t *mac)
{
    QCryptoHmac *h;
    struct iovec iov[RPMB_MAX_FRAMES];
    uint8_t *res = NULL;
    size_t reslen = 0;
    int i;
    bool ok = false;

    if (n < 1 || n > RPMB_MAX_FRAMES) {
        return false;
    }
    h = qcrypto_hmac_new(QCRYPTO_HASH_ALGO_SHA256, r->key, 32, NULL);
    if (!h) {
        return false;
    }
    for (i = 0; i < n; i++) {
        iov[i].iov_base = (void *)(frames + i * RPMB_FRAME + RPMB_DATA_OFS);
        iov[i].iov_len = RPMB_FRAME - RPMB_DATA_OFS;
    }
    if (qcrypto_hmac_bytesv(h, iov, n, &res, &reslen, NULL) == 0 &&
        reslen == 32) {
        memcpy(mac, res, 32);
        ok = true;
    }
    g_free(res);
    qcrypto_hmac_free(h);
    return ok;
}

static uint16_t mtk_rpmb_res(MtkUfsRpmb *r, uint16_t res)
{
    if (r->wcnt == 0xffffffffu) {
        res |= RPMB_RES_WCNT_EXPIRED;
    }
    return res;
}

/* SECURITY PROTOCOL OUT: n request frames */
static void mtk_rpmb_out(MtkUfsRpmb *r, const uint8_t *fr, int n)
{
    uint16_t type = lduw_be_p(fr + RPMB_TYPE_OFS);
    const uint8_t *last = fr + (n - 1) * RPMB_FRAME;

    switch (type) {
    case RPMB_REQ_KEY:
        r->last_write_type = 0x0100;
        r->pend_type = 0;
        if (r->key_set) {
            r->last_result = mtk_rpmb_res(r, RPMB_RES_GENERAL);
        } else {
            memcpy(r->key, fr + RPMB_MAC_OFS, 32);
            r->key_set = true;
            r->last_result = RPMB_RES_OK;
            mtk_rpmb_save(r, 0, 0);
        }
        break;
    case RPMB_REQ_WCNT:
        memcpy(r->nonce, fr + RPMB_NONCE_OFS, 16);
        r->pend_type = 0x0200;
        r->last_result = r->key_set ? mtk_rpmb_res(r, RPMB_RES_OK)
                                    : RPMB_RES_NO_KEY;
        break;
    case RPMB_REQ_WRITE: {
        uint16_t addr = lduw_be_p(fr + RPMB_ADDR_OFS);
        uint16_t cnt = lduw_be_p(fr + RPMB_BCNT_OFS);
        uint8_t mac[32];
        int i;

        r->last_write_type = 0x0300;
        r->last_addr = addr;
        r->pend_type = 0;
        if (cnt == 0 || cnt > n) {
            cnt = n;
        }
        if (!r->key_set) {
            r->last_result = RPMB_RES_NO_KEY;
        } else if (r->wcnt == 0xffffffffu) {
            r->last_result = mtk_rpmb_res(r, RPMB_RES_WRITE);
        } else if (!mtk_rpmb_mac(r, fr, cnt, mac) ||
                   memcmp(mac, fr + (cnt - 1) * RPMB_FRAME + RPMB_MAC_OFS,
                          32)) {
            r->last_result = mtk_rpmb_res(r, RPMB_RES_AUTH);
        } else if (ldl_be_p(last + RPMB_WCNT_OFS) != r->wcnt) {
            r->last_result = mtk_rpmb_res(r, RPMB_RES_COUNTER);
        } else if ((uint32_t)addr + cnt > r->nblk) {
            r->last_result = mtk_rpmb_res(r, RPMB_RES_ADDR);
        } else {
            for (i = 0; i < cnt; i++) {
                memcpy(r->data + ((size_t)addr + i) * RPMB_BLK,
                       fr + i * RPMB_FRAME + RPMB_DATA_OFS, RPMB_BLK);
            }
            r->wcnt++;
            r->last_result = mtk_rpmb_res(r, RPMB_RES_OK);
            mtk_rpmb_save(r, addr, cnt);
        }
        break;
    }
    case RPMB_REQ_READ:
        memcpy(r->nonce, fr + RPMB_NONCE_OFS, 16);
        r->rd_addr = lduw_be_p(fr + RPMB_ADDR_OFS);
        r->rd_cnt = lduw_be_p(fr + RPMB_BCNT_OFS);
        r->pend_type = 0x0400;
        break;
    case RPMB_REQ_RESULT:
        r->pend_type = r->last_write_type ? r->last_write_type : 0x0300;
        break;
    default:
        r->pend_type = 0;
        r->last_result = RPMB_RES_GENERAL;
        break;
    }
    if (getenv("MTK_RPMB_TRACE")) {
        fprintf(stderr, "MT6895 rpmb OUT type=%u frames=%d result=%u wcnt=%u\n",
                type, n, r->last_result, r->wcnt);
    }
}

/* SECURITY PROTOCOL IN: build up to max_frames response frames */
static int mtk_rpmb_in(MtkUfsRpmb *r, uint8_t *fr, int max_frames)
{
    int n = 1, i;
    uint16_t type = r->pend_type;

    memset(fr, 0, (size_t)max_frames * RPMB_FRAME);
    if (type == 0) {
        /* nothing requested: report a general failure response */
        type = r->last_write_type ? r->last_write_type : 0x0300;
        r->last_result = mtk_rpmb_res(r, RPMB_RES_GENERAL);
    }
    switch (type) {
    case 0x0200:
        memcpy(fr + RPMB_NONCE_OFS, r->nonce, 16);
        stl_be_p(fr + RPMB_WCNT_OFS, r->wcnt);
        stw_be_p(fr + RPMB_RES_OFS, r->last_result);
        break;
    case 0x0100:
    case 0x0300:
        stl_be_p(fr + RPMB_WCNT_OFS, r->wcnt);
        stw_be_p(fr + RPMB_ADDR_OFS, type == 0x0300 ? r->last_addr : 0);
        stw_be_p(fr + RPMB_RES_OFS, r->last_result);
        break;
    case 0x0400: {
        uint16_t res;

        n = r->rd_cnt ? r->rd_cnt : max_frames;
        if (n > max_frames) {
            n = max_frames;
        }
        if (!r->key_set) {
            res = RPMB_RES_NO_KEY;
        } else if ((uint32_t)r->rd_addr + n > r->nblk) {
            res = mtk_rpmb_res(r, RPMB_RES_ADDR);
        } else {
            res = mtk_rpmb_res(r, RPMB_RES_OK);
        }
        for (i = 0; i < n; i++) {
            uint8_t *f = fr + i * RPMB_FRAME;

            if (res == RPMB_RES_OK ||
                res == (RPMB_RES_OK | RPMB_RES_WCNT_EXPIRED)) {
                memcpy(f + RPMB_DATA_OFS,
                       r->data + ((size_t)r->rd_addr + i) * RPMB_BLK,
                       RPMB_BLK);
            }
            memcpy(f + RPMB_NONCE_OFS, r->nonce, 16);
            stw_be_p(f + RPMB_ADDR_OFS, r->rd_addr);
            stw_be_p(f + RPMB_BCNT_OFS, n);
            stw_be_p(f + RPMB_RES_OFS, res);
            stw_be_p(f + RPMB_TYPE_OFS, 0x0400);
        }
        break;
    }
    }
    for (i = 0; i < n; i++) {
        stw_be_p(fr + i * RPMB_FRAME + RPMB_TYPE_OFS, type);
    }
    if (r->key_set) {
        mtk_rpmb_mac(r, fr, n, fr + (n - 1) * RPMB_FRAME + RPMB_MAC_OFS);
    }
    if (getenv("MTK_RPMB_TRACE")) {
        fprintf(stderr, "MT6895 rpmb IN  type=0x%04x frames=%d result=%u\n",
                type, n, lduw_be_p(fr + RPMB_RES_OFS));
    }
    r->pend_type = 0;
    return n;
}

static bool mtk_ufs_rpmb_scsi(UfsLu *lu, UfsRequest *req, UfsReqResult *ret)
{
    const uint8_t *cdb = req->req_upiu.sc.cdb;
    uint8_t sense_buf[UFS_SENSE_SIZE];
    uint8_t scsi_status = GOOD;
    MtkUfsRpmb *r;
    uint32_t xfer;
    int len = 0;
    uint8_t *buf = NULL;

    if (cdb[0] != SECURITY_PROTOCOL_IN && cdb[0] != SECURITY_PROTOCOL_OUT &&
        cdb[0] != READ_CAPACITY_10) {
        return false;
    }
    r = mtk_rpmb_get();
    memset(sense_buf, 0, sizeof(sense_buf));
    if (cdb[0] == READ_CAPACITY_10) {
        uint8_t cap[8];

        stl_be_p(cap, r->nblk - 1);
        stl_be_p(cap + 4, RPMB_BLK);
        len = MIN(8, (int)req->data_len);
        if (len > 0 && dma_buf_read(cap, len, NULL, req->sg,
                                    MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            *ret = UFS_REQUEST_FAIL;
            return true;
        }
        goto done;
    }
    xfer = ldl_be_p(cdb + 6);
    if (cdb[4] & 0x80) {                         /* INC_512 */
        xfer *= 512;
    }
    if (cdb[1] != 0xEC || (lduw_be_p(cdb + 2) & 0xff00) != 0 ||
        xfer < RPMB_FRAME) {
        scsi_build_sense(sense_buf, SENSE_CODE(INVALID_FIELD));
        scsi_status = CHECK_CONDITION;
        goto done;
    }
    xfer = MIN(xfer, (uint32_t)req->data_len);
    xfer = MIN(xfer, (uint32_t)RPMB_MAX_FRAMES * RPMB_FRAME);
    xfer -= xfer % RPMB_FRAME;
    if (!xfer) {
        scsi_build_sense(sense_buf, SENSE_CODE(INVALID_FIELD));
        scsi_status = CHECK_CONDITION;
        goto done;
    }
    buf = g_malloc0(xfer);
    if (cdb[0] == SECURITY_PROTOCOL_OUT) {
        if (dma_buf_write(buf, xfer, NULL, req->sg,
                          MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            g_free(buf);
            *ret = UFS_REQUEST_FAIL;
            return true;
        }
        mtk_rpmb_out(r, buf, xfer / RPMB_FRAME);
        len = xfer;
    } else {
        len = mtk_rpmb_in(r, buf, xfer / RPMB_FRAME) * RPMB_FRAME;
        if (dma_buf_read(buf, len, NULL, req->sg,
                         MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            g_free(buf);
            *ret = UFS_REQUEST_FAIL;
            return true;
        }
    }
    g_free(buf);
done:
    mtk_ufs_build_scsi_response_upiu(req, sense_buf, sizeof(sense_buf),
                                     scsi_status == GOOD ? len : 0,
                                     scsi_status);
    *ret = UFS_REQUEST_SUCCESS;
    return true;
}
int mtk_ufs_rpmb_request(const uint8_t *frames, int nframes);
int mtk_ufs_rpmb_response(uint8_t *frames, int max_frames);

/*
 * Shared entry points for hw/misc/mtk_ufshci.c.
 *
 * The boot chain reaches the RPMB W-LUN through the host controller's UTP
 * path, not through this LU's SCSI emulation.  Both must drive the same
 * genuine JEDEC state machine, so the controller asks for frame processing
 * here instead of answering frames itself.  Every result code, write counter
 * and MAC is then computed by the same code on both paths -- nothing about
 * the outcome is decided by the host-controller model.
 */
int mtk_ufs_rpmb_request(const uint8_t *frames, int nframes)
{
    if (!mtk_ufs_rpmb_enabled() || nframes < 1 ||
        nframes > RPMB_MAX_FRAMES) {
        return -1;
    }
    mtk_rpmb_out(mtk_rpmb_get(), frames, nframes);
    return 0;
}

int mtk_ufs_rpmb_response(uint8_t *frames, int max_frames)
{
    if (!mtk_ufs_rpmb_enabled() || max_frames < 1) {
        return 0;
    }
    if (max_frames > RPMB_MAX_FRAMES) {
        max_frames = RPMB_MAX_FRAMES;
    }
    return mtk_rpmb_in(mtk_rpmb_get(), frames, max_frames);
}

/* ===== end RPMB ===== */

static UfsReqResult mtk_ufs_emulate_scsi_cmd(UfsLu *lu, UfsRequest *req)
{
    uint8_t lun = lu->lun;

    if (MTK_ENV_CACHED("MTK_UFS_CDB")) {
        static int nw;

        if (nw < 60) {
            nw++;
            fprintf(stderr, "UFSWLUN[%02d] lun=0x%02x cdb=%02x %02x %02x %02x"
                    " len=%u\n", nw, lun, req->req_upiu.sc.cdb[0],
                    req->req_upiu.sc.cdb[1], req->req_upiu.sc.cdb[2],
                    req->req_upiu.sc.cdb[3], req->data_len);
        }
    }
    QEMU_UNINITIALIZED uint8_t outbuf[4096];
    uint8_t sense_buf[UFS_SENSE_SIZE];
    uint8_t scsi_status;
    int len = 0;

    if (lun == UFS_UPIU_RPMB_WLUN && mtk_ufs_rpmb_enabled()) {
        UfsReqResult rr;

        if (mtk_ufs_rpmb_scsi(lu, req, &rr)) {
            return rr;
        }
    }

    switch (req->req_upiu.sc.cdb[0]) {
    case REPORT_LUNS:
        len = mtk_ufs_emulate_report_luns(req, outbuf, sizeof(outbuf));
        if (len == SCSI_COMMAND_FAIL) {
            scsi_build_sense(sense_buf, SENSE_CODE(INVALID_FIELD));
            scsi_status = CHECK_CONDITION;
        } else {
            scsi_status = GOOD;
        }
        break;
    case INQUIRY:
        len = mtk_ufs_emulate_wlun_inquiry(req, outbuf, sizeof(outbuf));
        if (len == SCSI_COMMAND_FAIL) {
            scsi_build_sense(sense_buf, SENSE_CODE(INVALID_FIELD));
            scsi_status = CHECK_CONDITION;
        } else {
            scsi_status = GOOD;
        }
        break;
    case REQUEST_SENSE:
        /* Just return no sense data */
        len = scsi_build_sense_buf(outbuf, sizeof(outbuf), SENSE_CODE(NO_SENSE),
                                   true);
        scsi_status = GOOD;
        break;
    case TEST_UNIT_READY:
        /*
         * Every well-known LUN must answer TEST UNIT READY with GOOD.
         * The RPMB WLUN is probed this way as part of the storage bring-up;
         * without this case it fell into the default branch, the firmware got
         * INVALID OPCODE (ASC/ASCQ 0x20/0x00) and retried forever, stalling
         * every later stage.
         */
        scsi_status = GOOD;
        break;
    case START_STOP:
        /* TODO: Revisit it when Power Management is implemented */
        if (lun == UFS_UPIU_UFS_DEVICE_WLUN) {
            scsi_status = GOOD;
            break;
        }
        /* fallthrough */
    default:
        scsi_build_sense(sense_buf, SENSE_CODE(INVALID_OPCODE));
        scsi_status = CHECK_CONDITION;
    }

    len = MIN(len, (int)req->data_len);
    if (scsi_status == GOOD && len > 0 &&
        dma_buf_read(outbuf, len, NULL, req->sg, MEMTXATTRS_UNSPECIFIED) !=
            MEMTX_OK) {
        return UFS_REQUEST_FAIL;
    }

    mtk_ufs_build_scsi_response_upiu(req, sense_buf, sizeof(sense_buf), len,
                                 scsi_status);
    return UFS_REQUEST_SUCCESS;
}

static UfsReqResult mtk_ufs_process_scsi_cmd(UfsLu *lu, UfsRequest *req)
{
    uint8_t task_tag = req->req_upiu.header.task_tag;

    if (MTK_ENV_CACHED("MTK_UFS_CDB")) {
        static int ncdb;
        static int ncdb_max = -1;

        /*
         * DIAGNOSTIC: the entry limit is configurable through MTK_UFS_CDB_MAX
         * because the boot stages alone consume the historical hard-coded 80
         * entries, leaving nothing for the kernel/init reads that we actually
         * want to see.  The default is unchanged.
         */
        if (ncdb_max < 0) {
            const char *m = getenv("MTK_UFS_CDB_MAX");

            ncdb_max = m ? atoi(m) : 80;
        }
        if (ncdb < ncdb_max) {
            uint64_t lba = 0;
            uint32_t nblk = 0;
            uint8_t op = req->req_upiu.sc.cdb[0];

            ncdb++;
            /* Decode the LBA/count of the block commands so the partition
             * the firmware is touching can be identified by its GPT LBA. */
            if (op == 0x28 || op == 0x2a || op == 0x88 || op == 0x8a) {
                lba = ((uint64_t)req->req_upiu.sc.cdb[2] << 24) |
                      ((uint64_t)req->req_upiu.sc.cdb[3] << 16) |
                      ((uint64_t)req->req_upiu.sc.cdb[4] << 8) |
                      (uint64_t)req->req_upiu.sc.cdb[5];
                nblk = ((uint32_t)req->req_upiu.sc.cdb[7] << 8) |
                       req->req_upiu.sc.cdb[8];
            } else if (op == 0xa8 || op == 0xaa) {
                lba = ((uint64_t)req->req_upiu.sc.cdb[2] << 56) |
                      ((uint64_t)req->req_upiu.sc.cdb[3] << 48) |
                      ((uint64_t)req->req_upiu.sc.cdb[4] << 40) |
                      ((uint64_t)req->req_upiu.sc.cdb[5] << 32);
                nblk = ((uint32_t)req->req_upiu.sc.cdb[10] << 24) |
                       ((uint32_t)req->req_upiu.sc.cdb[11] << 16) |
                       ((uint32_t)req->req_upiu.sc.cdb[12] << 8) |
                       req->req_upiu.sc.cdb[13];
            }
            fprintf(stderr, "UFSCDB[%02d] lun=%u cdb=%02x %02x %02x %02x"
                    " %02x %02x len=%u lba=%" PRIu64 " nblk=%u\n", ncdb,
                    lu->lun, op, req->req_upiu.sc.cdb[1],
                    req->req_upiu.sc.cdb[2], req->req_upiu.sc.cdb[3],
                    req->req_upiu.sc.cdb[4], req->req_upiu.sc.cdb[5],
                    req->data_len, lba, nblk);
        }
    }

    /*
     * Each ufs-lu has its own independent virtual SCSI bus. Therefore, we can't
     * use scsi_target_emulate_report_luns() which gets all lu information over
     * the SCSI bus. Therefore, we use mtk_ufs_emulate_scsi_cmd() like the
     * well-known lu.
     */
    if (req->req_upiu.sc.cdb[0] == REPORT_LUNS) {
        return mtk_ufs_emulate_scsi_cmd(lu, req);
    }

    /*
     * Clear any stashed unit attention before dispatching.
     *
     * The boot firmware does not implement the SCSI "unit attention, retry"
     * handshake: it turns the first CHECK CONDITION into a hard failure
     * (observed as "ufs_get_lu_size: ufshcd_queuecommand err" with
     * Sense Data ASC=0x29 right after the configuration descriptor read).
     * A real UFS logical unit has no removable medium, so no medium-change
     * or power-on attention should ever be pending.
     */
    lu->scsi_dev->unit_attention = SENSE_CODE(NO_SENSE);

    SCSIRequest *scsi_req =
        scsi_req_new(lu->scsi_dev, task_tag, lu->lun, req->req_upiu.sc.cdb,
                     UFS_CDB_SIZE, req);

    uint32_t len = scsi_req_enqueue(scsi_req);
    if (len) {
        scsi_req_continue(scsi_req);
    }

    return UFS_REQUEST_NO_COMPLETE;
}

static const Property mtk_ufs_lu_props[] = {
    DEFINE_PROP_DRIVE("drive", UfsLu, conf.blk),
    DEFINE_PROP_UINT8("lun", UfsLu, lun, 0),
};

static bool mtk_ufs_add_lu(UfsHc *u, UfsLu *lu, Error **errp)
{
    BlockBackend *blk = lu->conf.blk;
    int64_t brdv_len = blk_getlength(blk);
    uint64_t raw_dev_cap =
        be64_to_cpu(u->geometry_desc.total_raw_device_capacity);

    if (u->device_desc.number_lu >= UFS_MAX_LUS) {
        error_setg(errp, "ufs host controller has too many logical units.");
        return false;
    }

    if (u->lus[lu->lun] != NULL) {
        error_setg(errp, "ufs logical unit %d already exists.", lu->lun);
        return false;
    }

    u->lus[lu->lun] = lu;
    u->device_desc.number_lu++;
    raw_dev_cap += (brdv_len >> UFS_GEOMETRY_CAPACITY_SHIFT);
    u->geometry_desc.total_raw_device_capacity = cpu_to_be64(raw_dev_cap);
    return true;
}

void mtk_ufs_init_wlu(UfsLu *wlu, uint8_t wlun)
{
    wlu->lun = wlun;
    wlu->scsi_op = &mtk_ufs_emulate_scsi_cmd;
}

static void mtk_ufs_init_lu(UfsLu *lu)
{
    BlockBackend *blk = lu->conf.blk;
    int64_t brdv_len = blk_getlength(blk);

    memset(&lu->unit_desc, 0, sizeof(lu->unit_desc));
    lu->unit_desc.length = sizeof(UnitDescriptor);
    lu->unit_desc.descriptor_idn = UFS_QUERY_DESC_IDN_UNIT;
    lu->unit_desc.lu_enable = 0x01;
    lu->unit_desc.logical_block_size = UFS_BLOCK_SIZE_SHIFT;
    lu->unit_desc.unit_index = lu->lun;
    lu->unit_desc.logical_block_count =
        cpu_to_be64(brdv_len / (1 << lu->unit_desc.logical_block_size));

    lu->scsi_op = &mtk_ufs_process_scsi_cmd;
}

static bool mtk_ufs_lu_check_constraints(UfsLu *lu, Error **errp)
{
    if (!lu->conf.blk) {
        error_setg(errp, "drive property not set");
        return false;
    }

    if (lu->lun >= UFS_MAX_LUS) {
        error_setg(errp, "lun must be between 0 and %d", UFS_MAX_LUS - 1);
        return false;
    }

    return true;
}

static void mtk_ufs_init_scsi_device(UfsLu *lu, BlockBackend *blk, Error **errp)
{
    DeviceState *scsi_dev;

    scsi_bus_init(&lu->bus, sizeof(lu->bus), DEVICE(lu), &mtk_ufs_scsi_info);

    blk_ref(blk);
    blk_detach_dev(blk, DEVICE(lu));
    lu->conf.blk = NULL;
    /*
     * The scsi-hd below attaches through the node name and gets its own
     * BlockBackend; drop this one's claims (it still requires WRITE and does
     * not share it) so a writable LUN does not hit a permission conflict.
     */
    blk_set_perm(blk, 0, BLK_PERM_ALL, NULL);


    /*
     * The ufs-lu is the device that is wrapping the scsi-hd. It owns a virtual
     * SCSI bus that serves the scsi-hd.
     */
    scsi_dev = qdev_new("scsi-hd");
    object_property_add_child(OBJECT(&lu->bus), "ufs-scsi", OBJECT(scsi_dev));

    qdev_prop_set_uint32(scsi_dev, "physical_block_size", UFS_BLOCK_SIZE);
    qdev_prop_set_uint32(scsi_dev, "logical_block_size", UFS_BLOCK_SIZE);
    qdev_prop_set_uint32(scsi_dev, "scsi-id", 0);
    qdev_prop_set_uint32(scsi_dev, "lun", lu->lun);
    if (!qdev_prop_set_drive_err(scsi_dev, "drive", blk, errp)) {
        object_unparent(OBJECT(scsi_dev));
        return;
    }

    if (!qdev_realize_and_unref(scsi_dev, &lu->bus.qbus, errp)) {
        object_unparent(OBJECT(scsi_dev));
        return;
    }

    blk_unref(blk);
    lu->scsi_dev = SCSI_DEVICE(scsi_dev);

    /*
     * Drop the power-on unit attention SCSI stashes on a fresh device.
     *
     * The firmware brings the LU up and immediately issues its descriptor
     * reads; it does not honour the "unit attention, please retry" contract,
     * so the stashed ASC/ASCQ 0x29 turns the very first command into a
     * CHECK CONDITION.  The boot then aborts with
     *   "bug: active_num_lu shall not be 0"
     * and never reads a partition.  A real UFS device answers those
     * descriptor reads with data, so model that: no pending attention.
     */
    lu->scsi_dev->unit_attention = SENSE_CODE(NO_SENSE);
}

static void mtk_ufs_lu_realize(DeviceState *dev, Error **errp)
{
    UfsLu *lu = DO_UPCAST(UfsLu, qdev, dev);
    BusState *s = qdev_get_parent_bus(dev);
    UfsHc *u = UFS(s->parent);
    BlockBackend *blk = lu->conf.blk;

    if (!mtk_ufs_lu_check_constraints(lu, errp)) {
        return;
    }

    if (!blk) {
        error_setg(errp, "drive property not set");
        return;
    }

    if (!blkconf_blocksizes(&lu->conf, errp)) {
        return;
    }

    if (!blkconf_apply_backend_options(&lu->conf, !blk_supports_write_perm(blk),
                                       true, errp)) {
        return;
    }

    mtk_ufs_init_lu(lu);
    if (!mtk_ufs_add_lu(u, lu, errp)) {
        return;
    }

    mtk_ufs_init_scsi_device(lu, blk, errp);
}

static void mtk_ufs_lu_unrealize(DeviceState *dev)
{
    UfsLu *lu = DO_UPCAST(UfsLu, qdev, dev);

    lu->scsi_dev = NULL;
}

static void mtk_ufs_lu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = mtk_ufs_lu_realize;
    dc->unrealize = mtk_ufs_lu_unrealize;
    dc->bus_type = TYPE_UFS_BUS;
    device_class_set_props(dc, mtk_ufs_lu_props);
    dc->desc = "Virtual UFS logical unit";
}

static const TypeInfo mtk_ufs_lu_info = {
    .name = TYPE_UFS_LU,
    .parent = TYPE_DEVICE,
    .class_init = mtk_ufs_lu_class_init,
    .instance_size = sizeof(UfsLu),
};

static void mtk_ufs_lu_register_types(void)
{
    type_register_static(&mtk_ufs_lu_info);
}

type_init(mtk_ufs_lu_register_types)
