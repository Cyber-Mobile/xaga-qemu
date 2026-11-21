/*
 * MediaTek MT6895 (Dimensity 8100 / 8000) machine
 *
 * Emulates the boot path of an MT6895 handset (Xiaomi xaga / POCO X4 GT,
 * Redmi Note 11T Pro(+), Redmi K50i) far enough to start the vendor boot
 * chain:
 *
 *   BootROM  ->  preloader  ->  LK  ->  Linux
 *
 * The BootROM is modelled in two halves, which is how the real one is split
 * too:
 *
 *   - storage I/O: read the UFS boot LUN, verify the "UFS_BOOT" header, walk
 *     the MMM container and copy the payload to its load address. This is
 *     done in C here, because it is device access rather than CPU work.
 *   - hand-off: the actual "leave reset and enter the payload" step, which
 *     for a 32-bit payload means an AArch64 -> AArch32 transition and so has
 *     to be real guest code. That lives in mt6895_bootrom.S and is placed at
 *     the reset vector.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include <zlib.h>

#include "qapi/visitor.h"
#include "trace.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "qemu/bswap.h"
#include "net/net.h"
#include "hw/arm/boot.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "hw/arm/machines-qom.h"
#include "hw/arm/mt6895.h"
#include "target/arm/cpu.h"
#include "target/arm/cpregs.h"
#include "hw/char/mtk_uart.h"
#include "hw/misc/mtk_scpsys.h"
#include "hw/misc/mtk_tinysys_mbox.h"
#include "hw/timer/mtk_systimer.h"
#include "hw/misc/mtk_ufshci.h"
#include "hw/misc/mtk_ufs.h"
#include "system/block-backend-global-state.h"
#include "block/block-common.h"
#include "hw/misc/mtk_spmi_pmif.h"
#include "hw/misc/mtk_syscon.h"
#include "hw/misc/mtk_iommu.h"
#include "hw/misc/mtk_gce.h"
#include "hw/i2c/mtk_i2c.h"
#include "hw/i2c/bq28z610.h"
#include "hw/ssi/mtk_spi.h"
#include "hw/ssi/nvt36672c.h"

/* Set where the EINT controller is created; the touch IRQ is wired to it. */
static DeviceState *mt6895_eint_dev;
static void mt6895_create_spi(Mt6895MachineState *s, Error **errp);
#include "hw/misc/ktz8863a.h"
#include "hw/display/mtk_disp.h"
#include "chardev/char.h"
#include "hw/usb/xhci.h"
#include "hw/usb/hcd-xhci-sysbus.h"
#include "hw/sd/sd.h"
#include "hw/intc/arm_gicv3_common.h"
#include "hw/intc/arm_gicv3_its_common.h"
#include "hw/arm/bsa.h"
#include "system/reset.h"
#include "system/runstate.h"
#include "hw/core/resettable.h"
#include "target/arm/gtimer.h"
#include "target/arm/arm-powerctl.h"
#include "qemu/main-loop.h"
#include "hw/core/irq.h"
#include "exec/cpu-common.h"
#include "qemu/mtk-env.h"
#include "crypto/aes.h"

/*
 * Link-safety stub for arm_psci_set_prefer_el1(): the real (strong)
 * definition lives in target/arm/tcg/psci.c.  Keeping a weak one here means
 * that if this machine's source is uploaded/updated a few seconds before the
 * target/arm side (or a stale target/arm copy is in the tree), the build still
 * links; PSCI handled by ATF (the default) never calls the setter anyway.
 */
__attribute__((weak)) void arm_psci_set_prefer_el1(bool enable)
{
    (void)enable;
}
#include "hw/pci-host/gpex.h"
#include "hw/pci/pci.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "system/address-spaces.h"

/* Recorded by accel/tcg/cpu-exec.c when MTK_PC_HIST is set. */
/* Ring of recent data-window reads, dumped by the UART on demand. */
#define MTK_WIN_RING_LEN 64
uint64_t mtk_win_ring[MTK_WIN_RING_LEN];   /* (off << 8) | size */
uint64_t mtk_win_pc_ring[MTK_WIN_RING_LEN]; /* producer PC */
unsigned mtk_win_ring_idx;

extern uint64_t mtk_pc_hist[];
extern unsigned mtk_pc_hist_idx;
#include "qobject/qlist.h"
#include "system/system.h"
#include "target/arm/cpu.h"
#include "target/arm/cpregs.h"

/* ---- Boot LUN image format -------------------------------------------------
 *
 * Both structures below were recovered from the xaga boot LUN dump and are
 * documented, with the evidence, in docs/00-firmware-forensics.md.
 */

#define UFS_BOOT_MAGIC_LEN      8
#define UFS_BOOT_HDR_MIN        0x18
#define UFS_BOOT_IMG_OFF        0x10    /* u32: offset of the MMM container */

#define MMM_MAGIC               0x014D4D4D  /* "MMM\1" */
#define MMM_HDR_MIN             0x38
#define MMM_SIZE_OFF            0x20    /* u32: total image size */
#define MMM_ADDR_OFF            0x24    /* u32: load address */
#define MMM_ENTRY_OFF           0x30    /* u32: entry point, relative to load */

/*
 * SPSR_EL3 used to hand control to the preloader.
 *
 * For a return to AArch32 the SPSR_ELx value is the AArch32 CPSR value
 * itself: QEMU's el_from_spsr() tests bit 4 (PSTATE_nRW), which is set in
 * every AArch32 mode number, and then decodes CPSR_M directly. So this is
 *
 *   M[4:0] = 0x13  SVC mode, i.e. AArch32 EL1
 *   bit 6  = 1     FIQ masked
 *   bit 7  = 1     IRQ masked
 *   bit 8  = 1     SError masked
 *   bit 5  = 0     ARM state, not Thumb
 */
#define MT6895_PRELOADER_SPSR   0x1D3

/* AArch32 service entry inside the BootROM window (see the stub's array). */
#define MT6895_BOOTROM_A32_SERVICE  0x21ULL

/* The BootROM stub, assembled from hw/arm/mt6895_bootrom.S. */
/* Cached MTK_* environment probes (see qemu/mtk-env.h). */

/*
 * MTK_PCHIT=addr[,addr...] (hex): count how often each guest PC is reached and
 * print the counts from a machine timer (MTK_PCHIT_MS, default 14000 ms).
 * This is the cheap way to answer "did this firmware function ever execute" --
 * -d exec logging is orders of magnitude slower.
 */
extern unsigned long long mtk_pchit_cnt[8];
extern uint64_t mtk_pchit_tgt[8];
extern unsigned mtk_pchit_n;

static void mtk_pchit_dump(void *opaque)
{
    unsigned i;

    (void)opaque;
    for (i = 0; i < mtk_pchit_n && i < 8; i++) {
        fprintf(stderr, "MTK-PCHIT pc=%#" PRIx64 " count=%llu\n",
                mtk_pchit_tgt[i], mtk_pchit_cnt[i]);
    }
}

static const uint32_t mt6895_bootrom_code[] = {
    0xd2840003,     /* mov  x3, #0x2000          ; boot descriptor base   */
    0xf9400061,     /* ldr  x1, [x3]             ; payload entry          */
    0xf9400462,     /* ldr  x2, [x3, #8]         ; SPSR_EL3 for hand-off  */
    0xf9400864,     /* ldr  x4, [x3, #16]        ; boot info for payload  */
    0xd51e4021,     /* msr  elr_el3, x1                                     */
    0xd51e4002,     /* msr  spsr_el3, x2                                    */
    0xf9400c7e,     /* ldr  x30, [x3, #24]      ; LR for the payload         */
    0xd69f03e0,     /* eret                                                 */

    /*
     * AArch32 service entry, at MT6895_BOOTROM_A32_SERVICE (0x20).
     *
     * The payload stores the r4 it was handed, and its DRAM-mode stage later
     * calls it with "blx r4" -- a call, not a jump, so the service is expected
     * to return. It runs in AArch32/Thumb (the payload is a 32-bit image), so
     * the AArch64 stub above cannot serve it: one address cannot be both.
     *
     * This is a probe, not the final service: it only returns, so that the
     * payload's continuation becomes visible in the trace and its expectations
     * can be read off. r4's low bit is set to select Thumb on the call.
     */
    /*
     * Thumb: bx lr, preceded by nops so the entry stays at 0x20.
     *
     * An earlier variant set r4 to the boot-argument block before returning,
     * on the theory that the payload's DRAM stage expects a context pointer in
     * r4. It made no difference: the DRAM stage loads r4 from its own literal
     * pool (image offset 0x3658C -> 0x02045C75) and calls that, so the call
     * never reaches this service at all. Its "mov r0, r4" loop therefore hands
     * its own code address to its own function as a data pointer -- either the
     * image expects a data pointer at that literal, or the DRAM copy sits at
     * the wrong base. Both need image relocation analysis, not a register
     * tweak here.
     */
    0x46c046c0,     /* nop ; nop                                            */
    0x00004770,     /* bx   lr                                              */
};

/*
 * Micro-driver stub for para-virtualized block device reading.
 * Copies 512 bytes from 0x50000000 + LBA * 512 to dst buffer (r3).
 * Placed at offset 0x100 in BootROM (entry 0x00000101 Thumb).
 */
static const uint16_t mt6895_bdev_read_stub[] = {
    0xb530,             /* push     {r4, r5, lr}          */
    0x4c14,             /* ldr      r4, [pc, #20]         */  /* FIX: +4 offset to load 0x50000000 literal */
    0x024d,             /* lsls     r5, r1, #9            */
    0x1964,             /* adds     r4, r4, r5            */
    0x2280,             /* movs     r2, #128              */
    /* loop: */
    0xcc20,             /* ldm      r4!, {r5}             */
    0xc320,             /* stm      r3!, {r5}             */
    0x3a01,             /* subs     r2, #1                */
    0xd1fb,             /* bne      loop                  */
    0x2000,             /* movs     r0, #0                */
    0xbd30,             /* pop      {r4, r5, pc}          */
    0xbf00,             /* nop                            */
    0x0000, 0x5000,     /* .word    0x50000000            */
};



static void *mt6895_guest_ptr(Mt6895MachineState *s, uint64_t addr,
                              uint64_t size);

/*
 * Block-device command register: the payload's write callback latches
 * (LBA, block count, buffer) here and the machine performs the transfer into
 * the image-backed window, so writes are coherent with reads and survive a
 * warm reset.  This is what makes the DRAM calibration block persist instead
 * of being dropped.
 */
static uint64_t mt6895_bdev_cmd_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    switch (off) {
    case 0:
        return s->bdev_cmd_lba;
    case 4:
        return s->bdev_cmd_cnt;
    default:
        return 0;
    }
}

static void mt6895_bdev_cmd_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    Mt6895MachineState *s = opaque;

    switch (off) {
    case 0:
        s->bdev_cmd_lba = val;
        if (MTK_ENV_CACHED("MTK_BDEVTRACE")) {
            static int n1;
            if (n1++ < 40) fprintf(stderr, "MTK-BDEVARG lba=%u\n", (uint32_t)val);
        }
        break;
    case 4:
        s->bdev_cmd_cnt = val;
        if (MTK_ENV_CACHED("MTK_BDEVTRACE")) {
            static int n2;
            if (n2++ < 40) fprintf(stderr, "MTK-BDEVARG cnt=%u\n", (uint32_t)val);
        }
        break;
    case 8: {
        uint32_t lba = s->bdev_cmd_lba;
        uint32_t cnt = s->bdev_cmd_cnt;
        uint64_t boff = (uint64_t)lba * 512;
        uint64_t len = (uint64_t)cnt * 512;
        uint8_t *host;

        if (!cnt || len > (1u << 20) || boff + len > s->data_win_size) {
            break;
        }
        if (MTK_ENV_CACHED("MTK_BDEVTRACE")) {
            static int n3;
            if (n3++ < 40) {
                fprintf(stderr, "MTK-BDEVARG buf=%#x lba=%u cnt=%u\n",
                        (uint32_t)val, lba, cnt);
            }
        }
        host = mt6895_guest_ptr(s, val, len);
        if (host) {
            if (MTK_ENV_CACHED("MTK_BDEVTRACE")) {
                fprintf(stderr,
                        "MTK-BDEVDATA lba=%u cnt=%u bytes=%02x%02x%02x%02x"
                        "%02x%02x%02x%02x\n", lba, cnt,
                        host[0], host[1], host[2], host[3],
                        host[4], host[5], host[6], host[7]);
            }
            address_space_write(&address_space_memory,
                                MT6895_DATA_ADDR + boff,
                                MEMTXATTRS_UNSPECIFIED, host, len);
            if (MTK_ENV_CACHED("MTK_BDEVTRACE")) {
                fprintf(stderr, "MTK-BDEVWRITE lba=%u cnt=%u\n", lba, cnt);
            }
        }
        break;
    }
    default:
        break;
    }
}

static const MemoryRegionOps mt6895_bdev_cmd_ops = {
    .read = mt6895_bdev_cmd_read,
    .write = mt6895_bdev_cmd_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/*
 * ARM-mode block-write entry point.  The device object's +0x24 slot used to
 * point at the read stub, so every write the payload made came back as a read
 * and the data was dropped -- the DRAM calibration block never reached the
 * dram_para partition and the payload retrained DRAM on every boot.
 *
 * The stub latches (LBA, block count, buffer) into a model-owned command
 * register and the machine performs the transfer into the image-backed
 * window, so writes are coherent with reads and survive a warm reset.
 *
 * The bytes are copied into the payload's RAM (not served from an overlay)
 * because that address sits inside the payload's BSS clear range, and an
 * overlay returning non-zero bytes there derails the BSS clear.
 *
 *   push {r4, lr}        ldr  r4, [pc, #0x14]   ; command register
 *   str  r1, [r4]        str  r2, [r4, #4]
 *   str  r3, [r4, #8]    mov  r0, #0
 *   pop  {r4, pc}        nop
 *   .word MT6895_BDEV_CMD_BASE
 */
static const uint8_t mt6895_bdev_write_stub[] = {
    0x10, 0x40, 0x2d, 0xe9,     /* e92d4010 push {r4, lr}      */
    0x14, 0x40, 0x9f, 0xe5,     /* e59f4014 ldr r4, [pc,#0x14] */
    0x00, 0x10, 0x84, 0xe5,     /* e5841000 str r1, [r4]       */
    0x04, 0x20, 0x84, 0xe5,     /* e5842004 str r2, [r4, #4]   */
    0x08, 0x30, 0x84, 0xe5,     /* e5843008 str r3, [r4, #8]   */
    0x00, 0x00, 0xa0, 0xe3,     /* e3a00000 mov r0, #0         */
    0x10, 0x80, 0xbd, 0xe8,     /* e8bd8010 pop {r4, pc}       */
    0x00, 0x00, 0xa0, 0xe1,     /* e1a00000 nop                */
    0x80, 0xc0, 0x07, 0x02,     /* .word 0x0207c080            */
};

/* Copy the write stub into the payload's RAM once the BSS clear is done. */
/*
 * MTK_UFS_GATE: the preloader skips its UFS bring-up unless the byte at
 * 0x020f7cfc reads 1 (0x2041aa: ldr r0,[0x20f7cfc]; cmp r0,#1; bne skip).
 * On hardware an earlier stage sets that flag; the machine sets it here so
 * the payload's own ufshcd driver runs instead of falling back to the
 * synthetic block device.
 */
static void mt6895_set_ufs_gate(void *opaque)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, 0x020f7cfcULL, 4);

    if (p) {
        p[0] = 1;
        if (MTK_ENV_CACHED("MTK_UFS_GATE")) {
            fprintf(stderr, "mt6895: UFS init gate set at 0x020f7cfc\n");
        }
    }
}

static void mt6895_place_bdev_stub(void *opaque)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, MT6895_BDEV_WRITE_STUB,
                                  sizeof(mt6895_bdev_write_stub));

    if (p) {
        memcpy(p, mt6895_bdev_write_stub, sizeof(mt6895_bdev_write_stub));
        if (MTK_ENV_CACHED("MTK_BDEVTRACE")) {
            fprintf(stderr, "mt6895: block-write stub placed at %#llx\n",
                    (unsigned long long)MT6895_BDEV_WRITE_STUB);
        }
    }
}
/*
 * MTK_LOGBUFPC: watch the preloader's log buffer.  Lines are formatted into
 * DRAM at 0x7FFC0000 and only flushed to the UART later, so the PC at the
 * UART write belongs to the flusher, not to the code that produced the
 * message.  Observing the buffer writes instead gives the producer's PC.
 */
static uint64_t mt6895_logbuf_read(void *opaque, hwaddr off, unsigned size)
{
    return 0;
}

/*
 * MTK_UFS_PROBE: the preloader's UFS bring-up polls 0x20487cc, which reads a
 * readiness halfword at [driver+0x50] and spins while it is zero.  Report the
 * driver pointer and its readiness word so the model can tell whether the
 * probe stage ever ran.
 */
static void mt6895_ufs_probe(CPUState *cs)
{
    CPUARMState *env = cpu_env(cs);
    uint32_t pc = env->regs[15];

    if (pc == 0x20487d6) {
        static int np;
        uint32_t drv = env->regs[0];

        if (np < 10) {
            uint8_t rdy[2] = { 0 };

            np++;
            address_space_read(&address_space_memory, drv + 0x50ULL,
                               MEMTXATTRS_UNSPECIFIED, rdy, 2);
            fprintf(stderr, "UFS-PROBE drv=%#x rdymem[0x50]=%#x%02x\n",
                    drv, rdy[1], rdy[0]);
        }
    }
}

static void mt6895_logbuf_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    if (MTK_ENV_CACHED("MTK_UFS_PROBE")) {
        CPUState *pcs = qemu_get_cpu(0);

        if (pcs) {
            mt6895_ufs_probe(pcs);
        }
    }
    if (MTK_ENV_CACHED("MTK_LOGBUFPC")) {
        static int nb;
        CPUState *cs = qemu_get_cpu(0);
        uint64_t pc = 0;

        if (cs) {
            CPUARMState *env = cpu_env(cs);

            pc = env->aarch64 ? env->pc : (uint64_t)env->regs[15];
        }
        if (nb < 400) {
            uint64_t lr = 0, sp = 0;
            uint32_t stk[8] = { 0 };
            {
                unsigned hi;
                char hbuf[600];
                size_t ho = 0;

                hbuf[0] = 0;
                for (hi = 0; hi < 96 && ho < sizeof(hbuf) - 12; hi++) {
                    uint64_t hv =
                        mtk_pc_hist[(mtk_pc_hist_idx + hi) % 96];

                    if (hv >= 0x02000000ULL && hv < 0x02080000ULL) {
                        ho += snprintf(hbuf + ho, sizeof(hbuf) - ho,
                                       "%lx,", (unsigned long)hv);
                    }
                }
                fprintf(stderr, "LOGBUFHIST %s\n", hbuf);
            }

            if (cs && !cpu_env(cs)->aarch64) {
                CPUARMState *env = cpu_env(cs);

                lr = (uint64_t)env->regs[14];
                sp = (uint64_t)env->regs[13];
                if (sp) {
                    address_space_read(&address_space_memory, sp,
                                       MEMTXATTRS_UNSPECIFIED, stk,
                                       sizeof(stk));
                }
            }
            nb++;
            fprintf(stderr,
                    "LOGBUF pc=%#" PRIx64 " lr=%#" PRIx64 " sp=%#" PRIx64
                    " stk=%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x"
                    " | val=%#" PRIx64 "\n",
                    pc, lr, sp,
                    stk[0], stk[1], stk[2], stk[3],
                    stk[4], stk[5], stk[6], stk[7], val);
        }
    }
}

static const MemoryRegionOps mt6895_logbuf_ops = {
    .read = mt6895_logbuf_read,
    .write = mt6895_logbuf_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static const struct {
    const char *name;
    uint64_t base;
    uint64_t size;
} mt6895_sram[MT6895_NUM_SRAMS] = {
    { "mt6895.sram0", MT6895_SRAM0_BASE, MT6895_SRAM0_SIZE },
    { "mt6895.sram1", MT6895_SRAM1_BASE, MT6895_SRAM1_SIZE },
    { "mt6895.scmi-tx-shmem", MT6895_SCMI_TX_SHMEM_BASE, MT6895_SCMI_SHMEM_SIZE },
    { "mt6895.scmi-rx-shmem", MT6895_SCMI_RX_SHMEM_BASE, MT6895_SCMI_SHMEM_SIZE },
};

static uint32_t ld32(const uint8_t *p)
{
    return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

/*
 * Translate a guest physical address range into a host pointer, if it lands
 * in memory the BootROM can write to.
 */
static void *mt6895_guest_ptr(Mt6895MachineState *s, uint64_t addr,
                              uint64_t len)
{
    /*
     * Fold the boot chain's 0xffff0000_00000000 data window back to the
     * physical address it aliases; both spellings refer to the same DRAM.
     */
    if (addr >= MT6895_DRAM_ALIAS_BASE) {
        addr -= MT6895_DRAM_ALIAS_BASE;
    }
    if (addr >= MT6895_DRAM_BASE &&
        len <= MACHINE(s)->ram_size &&
        addr - MT6895_DRAM_BASE <= MACHINE(s)->ram_size - len) {
        return memory_region_get_ram_ptr(s->ddr) + (addr - MT6895_DRAM_BASE);
    }
    for (int i = 0; i < MT6895_NUM_SRAMS; i++) {
        uint64_t base = mt6895_sram[i].base;
        uint64_t size = mt6895_sram[i].size;

        if (addr >= base && len <= size && addr - base <= size - len) {
            return memory_region_get_ram_ptr(&s->sram[i]) + (addr - base);
        }
    }
    return NULL;
}

/*
 * Model of the BootROM's storage stage: pull the preloader out of the UFS
 * boot LUN image and place it where the payload header says it goes.
 */
/*
 * The SRAM log header window. It sits on top of the (read-only) BootROM region
 * so that guest accesses to the header can be traced: the ROM kernel's
 * log_store module reports reading the wrong bytes, and the first question is
 * whether the guest touches this address at all.
 */
static uint64_t mt6895_logstore_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint64_t val = 0;

    /*
     * A bounded probe: every overlay read anywhere in the machine passes
     * through here once per boot at most, recording the window and offset so
     * the regions the payload actually touches become visible without a
     * debugger. GDB cannot reach the rollback path inside one run, but this
     * costs nothing and survives the whole boot.
     */
    if (off + size <= MT6895_LOGSTORE_WIN_SIZE) {
        memcpy(&val, memory_region_get_ram_ptr(&s->bootrom) +
               MT6895_LOGSTORE_WIN_BASE + off, size);
    }
    trace_mt6895_logstore_read(off, size, val);
    return val;
}

/*
 * A BootROM-provided startup value that the payload's own BSS clear would
 * otherwise destroy.
 *
 * The DRAM stage reads a memory descriptor base pointer from 0x020F33BC and
 * then looks regions up in a table at base + 0x7CCF8. That address sits inside
 * the range the payload clears early on (0x020F1000..0x020FB870): a hardware
 * watchpoint shows the machine's write landing and the clear immediately
 * zeroing it, after which the lookup reads a count of zero and the stage
 * prints "md fail" through the routine at 0x02036098.
 *
 * Values the BootROM supplies are not writable by the payload on hardware, so
 * this exposes the words as read-only registers instead of RAM locations and
 * the clear can no longer erase them. Reads return the SRAM1 window base (the
 * origin the descriptor offsets are measured from) and an entry count; writes
 * are ignored.
 */
static int mt6895_brom_arg_diag_writes;

static uint64_t mt6895_brom_arg_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint64_t val;

    if (MTK_ENV_CACHED("MTK_BROMTRACE")) {
        static int nb;

        if (nb < 60) {
            nb++;
            fprintf(stderr, "MTK-BROMRD off=%#" PRIx64 " size=%u\n",
                    (uint64_t)off, size);
        }
    }

    /*
     * EXPERIMENT (2026-11-03): let guest writes win over the machine default.
     * The boot-to-LK path reads [0x020F33C0] as a pointer to the boot-tag
     * buffer; with the fixed default (1) it dereferences address 0x10 and
     * faults.  If the payload has stored a pointer there, honour it.
     */
    if (off / 4 < ARRAY_SIZE(s->brom_arg_store) && s->brom_arg_store[off / 4]) {
        return extract64(s->brom_arg_store[off / 4], 0, size * 8);
    }

    switch (off) {
    case 0:
        /*
         * The data-region base the payload indexes everything from: SRAM1.
         *
         * Three values have now been tested, and only this one works:
         *
         *   0x02000000  correct  (811 bytes of serial output)
         *   0x02001000  wrong    (731 bytes; power-state dump regresses)
         *   0x02000008  wrong    (731 bytes; same regression)
         *
         * So the payload wants the raw window base, not the container's own
         * address and not the address of the "FILE_INFO" string inside it.
         */
        val = MT6895_SRAM1_BASE;
        break;
    case 4:     /* entry count for that table */
        val = 1;
        break;
    default:
        val = 0;
        break;
    }

    return extract64(val, 0, size * 8);
}

static void mt6895_brom_arg_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    Mt6895MachineState *s = opaque;
    ARMCPU *cpu = ARM_CPU(current_cpu);
    uint32_t pc = cpu ? (uint32_t)cpu->env.regs[15] : 0;
    uint32_t lr = cpu ? (uint32_t)cpu->env.regs[14] : 0;

    (void)pc;
    (void)lr;

    (void)pc;
    (void)lr;

    s->brom_arg_store[off / 4] = val;
    if (mt6895_brom_arg_diag_writes < 64) {
        mt6895_brom_arg_diag_writes++;
    }
}

static const MemoryRegionOps mt6895_brom_arg_ops = {
    .read = mt6895_brom_arg_read,
    .write = mt6895_brom_arg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/*
 * The pointer the region-descriptor routine at 0x020345E8 tests first.
 *
 * Execution tracing of a working run shows exactly two blocks before the
 * "ign need to be power of 2" print at 0x02034688 -- 0x02034620
 * ("ldr.w r0, [r8]", r8 = 0x020F3720) and 0x02034624 ("cbz r0") -- while the
 * whole comparison chain that follows (0x0203462C..0x02034672) never runs. A
 * zero there is what takes the error path.
 *
 * Writing it from the machine does not survive: the address sits inside the
 * payload's BSS clear range (0x020F1000..0x020FB870), so the SRAM stage zeroes
 * it again -- a live read shows 0 even though the machine wrote it. Exposing
 * it read-only removes that race, the same way 0x020F33BC is handled.
 */
static uint64_t mt6895_region_ptr_read(void *opaque, hwaddr off, unsigned size)
{
    return extract64(MT6895_SRAM1_BASE, 0, size * 8);
}

static void mt6895_region_ptr_write(void *opaque, hwaddr off, uint64_t val,
                                    unsigned size)
{
    Mt6895MachineState *s = opaque;

    s->region_ptr_store[off / 4] = val;
}

static const MemoryRegionOps mt6895_region_ptr_ops = {
    .read = mt6895_region_ptr_read,
    .write = mt6895_region_ptr_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/*
 * A lookup-table head the RKP initialisation dereferences.
 *
 * The routine at 0x0200CFCC loads 0x020F130C and immediately branches past
 * the "+28" step ("b.n 0xa" at 0x0200CFD2 jumps to 0x0200CFE0), so the word
 * it dereferences is *(0x020F130C) itself. Empty means it returns zero, which
 * is what sends the caller at 0x0202C870 down its "ramconsole" error path
 * instead of running the RKP load. That address sits inside the payload's BSS
 * clear range, so a plain machine write does not survive; exposing the word
 * read-only does.
 *
 * The value handed back is the boot argument block, whose first word is the
 * pointer the payload itself stored there.
 */
static uint64_t mt6895_kinfo_flag_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off / 4 < ARRAY_SIZE(s->kinfo_flag_store) && s->kinfo_flag_store[off / 4]) {
        return extract64(s->kinfo_flag_store[off / 4], 0, size * 8);
    }

    /*
     * Enable the DRAM stage's name-table lookup; see MT6895_KINFO_FLAG_BASE.
     *
     * MTK_KINFO_FLAG0 reports zero instead, which selects the DRAM stage's
     * GPT path (0x0202C854) -- the route real hardware takes when it reads
     * the partition table from the boot device rather than from the copy the
     * BootROM leaves in SRAM.
     */
    if (MTK_ENV_CACHED("MTK_KINFO_FLAG0")) {
        return 0;
    }
    return extract64(1, 0, size * 8);
}

static void mt6895_kinfo_flag_write(void *opaque, hwaddr off, uint64_t val,
                                    unsigned size)
{
    Mt6895MachineState *s = opaque;

    s->kinfo_flag_store[off / 4] = val;
}

static const MemoryRegionOps mt6895_kinfo_flag_ops = {
    .read = mt6895_kinfo_flag_read,
    .write = mt6895_kinfo_flag_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

/*
 * MTK_CRYPTO_FLAG0: the BL2_EXT crypto engine keeps an "already initialised"
 * flag at 0x510a1b54.  Reading it as 1 makes the driver take the disable
 * path, so the HW_ENGINE_INIT SiP is never submitted and SHA-256 never
 * becomes available.  This window answers zero so the init path runs.
 */
static uint64_t mt6895_crypto_flag_read(void *opaque, hwaddr off, unsigned size)
{
    (void)opaque; (void)off; (void)size;
    return 0;
}

static void mt6895_crypto_flag_write(void *opaque, hwaddr off, uint64_t val,
                                     unsigned size)
{
    (void)opaque; (void)off; (void)val; (void)size;
}

static const MemoryRegionOps mt6895_crypto_flag_ops = {
    .read = mt6895_crypto_flag_read,
    .write = mt6895_crypto_flag_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static uint64_t mt6895_lba_source_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off / 4 < ARRAY_SIZE(s->lba_source_store) && s->lba_source_store[off / 4]) {
        return extract64(s->lba_source_store[off / 4], 0, size * 8);
    }

    /*
     * The partition parse compares the header's current-LBA against
     * *(0x020F4098) >> 9, so this value has to be that LBA shifted back up.
     */
    return extract64(MT6895_LBA_SOURCE_VALUE, 0, size * 8);
}

static void mt6895_lba_source_write(void *opaque, hwaddr off, uint64_t val,
                                    unsigned size)
{
    Mt6895MachineState *s = opaque;

    s->lba_source_store[off / 4] = val;
}

static const MemoryRegionOps mt6895_lba_source_ops = {
    .read = mt6895_lba_source_read,
    .write = mt6895_lba_source_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/*
 * The payload's partition name table at 0x02078EA4: pure pass-through
 * overlay. The payload builds and maintains the table it consumes (GPT path
 * selected by the flag at 0x020F4AC4); the machine seeds nothing.
 */

static int mt6895_tblprobe_n;
static int mt6895_geomprobe_n;
static int mt6895_bl2_name_write_diag_n;

static uint64_t mt6895_name_tbl_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint64_t value = 0;
    unsigned i;

    if (mt6895_tblprobe_n < 48) {
        mt6895_tblprobe_n++;
        trace_mt6895_tblprobe(mt6895_tblprobe_n, MT6895_NAME_TABLE_BASE + off, size);
    }
    if (mt6895_geomprobe_n < 48 && ((off % MT6895_NAME_TABLE_STRIDE) == 4)) {
        mt6895_geomprobe_n++;
        trace_mt6895_geomprobe(mt6895_geomprobe_n, MT6895_NAME_TABLE_BASE + off, size);
    }
    for (i = 0; i < size; i++) {
        hwaddr b = off + i;
        uint8_t byte = (b >= MT6895_NAME_TABLE_SIZE || !s->name_tbl_valid[b]) ? 0 :
            (uint8_t)(s->name_tbl_store[b / 4] >> ((b % 4) * 8));
        value |= (uint64_t)byte << (i * 8);
    }
    return value;
}

static void mt6895_name_tbl_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    Mt6895MachineState *s = opaque;
    ARMCPU *cpu = ARM_CPU(current_cpu);
    uint32_t pc = cpu ? (uint32_t)cpu->env.regs[15] : 0;
    uint32_t lr = cpu ? (uint32_t)cpu->env.regs[14] : 0;

    (void)pc;
    (void)lr;
    unsigned i;

    if (mt6895_bl2_name_write_diag_n < 128 && off >= 0xb4 && off < 0xd0) {
        mt6895_bl2_name_write_diag_n++;
        fprintf(stderr,
                "mt6895-dbg: name write off=0x%03x size=%u value=0x%08" PRIx64
                " pc=%08x lr=%08x\n",
                (unsigned)off, size, val, pc, lr);
    }

    /* Ignore only the confirmed early BSS clear, not GPT field writes. */
    if (pc == 0x020487c6u && lr == 0x020404f7u) {
        return;
    }

    for (i = 0; i < size; i++) {
        hwaddr byte_off = off + i;
        uint8_t byte;
        uint32_t mask;
        unsigned shift;

        if (byte_off >= MT6895_NAME_TABLE_SIZE) {
            continue;
        }
        byte = (val >> (i * 8)) & 0xff;
        shift = (byte_off % 4) * 8;
        mask = 0xffu << shift;
        s->name_tbl_store[byte_off / 4] =
            (s->name_tbl_store[byte_off / 4] & ~mask) | ((uint32_t)byte << shift);
        s->name_tbl_valid[byte_off] = 1;
    }
}

static const MemoryRegionOps mt6895_name_tbl_ops = {
    .read = mt6895_name_tbl_read,
    .write = mt6895_name_tbl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * The UART console base pointer at 0x02079E00, overlaid for the same reason
 * as the RKP object.
 *
 * A hardware watchpoint caught the payload's own BSS clear zeroing this
 * word: a byte-wise memset (0x020487BE, "strb r1,[r0,r3]") fills
 * 0x020798A4..0x0207C0A4 -- 0x2800 bytes -- and the slot sits inside it.
 * It is reached from the partition-lookup path (return address
 * 0x02040505), so it runs late, after the console has been switched
 * 0x11001000 -> 0x11002000 -> 0x11001000 and after the preloader has
 * printed several hundred lines. With the word zero the next serial write
 * polls THRE at 0x02048F32 against address 0x14, which is unmapped and
 * reads 0, so the transmitter never reports empty and the boot stops for
 * good at "[proinfo] ..." -- the first character is already in r0.
 *
 * The overlay is deliberately four bytes wide and deliberately not a plain
 * register file: while the store reads zero the read falls back to UART0,
 * the port the boot log actually comes out of. A wider overlay here was
 * tried and reverted (see MT6895_NAME_TABLE_SIZE) because it masked SRAM1
 * the payload uses.
 */
static uint64_t mt6895_conbase_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (s->conbase_store) {
        return extract64(s->conbase_store, 0, size * 8);
    }
    return extract64(MT6895_UART0_BASE, 0, size * 8);
}

static void mt6895_conbase_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off + size <= 4) {
        s->conbase_store = val;
    }
}

static const MemoryRegionOps mt6895_conbase_ops = {
    .read = mt6895_conbase_read,
    .write = mt6895_conbase_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};


/*
 * Overlay for one machine-planted structure that the payload's BSS clear
 * would otherwise wipe.
 *
 * The clear is a byte-wise memset (0x020487BE) that the partition-lookup
 * path runs twice -- 0x02078EA4 for 0xA00 bytes and 0x020798A4 for 0x2800
 * -- and a third call reaches 0x02080000 as well, so simply relocating the
 * planted block past the image does not save it (measured: the object's
 * type word reads 1 right after the machine plants it and 0 after the
 * clear, PC 0x020487C8).
 *
 * Only the two structures that must survive code execution are overlaid:
 * the device object (its type word is what get_bootdev(1) matches on) and
 * the I/O stub (the "blx" into it ran off to 0xB224D9C0 when it was
 * zeroed). The GPT header is deliberately NOT overlaid -- covering
 * 0x0207B200 cut the serial output from 119281 bytes to 30144, because
 * that range is BSS the payload uses at run time.
 */
typedef struct Mt6895Planted {
    Mt6895MachineState *s;
    hwaddr base;
    unsigned size;
    const uint8_t *def;
    uint32_t store[MT6895_PLANTED_WORDS];
} Mt6895Planted;


/*
 * MMIO cost probe: overlays are function-call-per-access, and the payload
 * polls some of them hard. Count every overlay access and report the rate so
 * "slow boot" can be attributed to MMIO rather than to guest compute.
 */
static uint64_t mt6895_io_total;
static int64_t mt6895_io_last_ns;
static void mt6895_io_probe_tick(void)
{
    int64_t now;

    mt6895_io_total++;
    if ((mt6895_io_total & 0xfffff) == 0) {
        now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
        fprintf(stderr, "IO-PROBE total=%" PRIu64 " last_1M_in_ms=%.1f\n",
                mt6895_io_total, (now - mt6895_io_last_ns) / 1e6);
        mt6895_io_last_ns = now;
    }
}

static uint64_t mt6895_planted_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895Planted *r = opaque;
    Mt6895MachineState *s = r->s;
    uint64_t v = 0;
    unsigned i;

    (void)s;

    mt6895_io_probe_tick();

    if (off / 4 < MT6895_PLANTED_WORDS && r->store[off / 4]) {
        v = extract64(r->store[off / 4], 0, size * 8);
    } else {
        for (i = 0; i < size; i++) {
            if (off + i < r->size) {
                v |= (uint64_t)r->def[off + i] << (8 * i);
            }
        }
    }

    return v;
}

static void mt6895_planted_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    mt6895_io_probe_tick();
    Mt6895Planted *r = opaque;

    if (off + size <= r->size && off / 4 < MT6895_PLANTED_WORDS) {
        r->store[off / 4] = val;
    }
}

static const MemoryRegionOps mt6895_planted_ops = {
    .read = mt6895_planted_read,
    .write = mt6895_planted_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};


/*
 * Platform tag area (GTA) for ATF.
 *
 * ATF's xlat context setup (0x4821443c) asks the tag parser (0x4821e488)
 * for the translation-table area it should use.  The parser walks the GTA
 * looking for the "SS\x12" tag and returns two values through the caller's
 * output pointers; when the tag is absent it leaves the outputs untouched and
 * the caller then reuses whatever the stack happened to hold, which makes the
 * new context share BL31's table area and wipe BL31's mappings.
 *
 * The tag layout, from the parser itself, is:
 *     +0x00  u32  magic 0x00535312 ("SS\x12")
 *     +0x08  u32  table base   (big endian)
 *     +0x0c  u32  table size   (big endian)
 * The area is platform configuration: this machine owns the memory layout, so
 * it publishes a tag describing a table region of its own rather than letting
 * the parser fall through to the stack.
 */
static void mt6895_atf_gta_plant(uint8_t *p)
{
    stl_le_p(p + 0x00, 0x00535312u);    /* magic "SS\x12" */
    stl_le_p(p + 0x04, 0x48280000u);    /* table base (LE, above BL31 tables) */
    stl_le_p(p + 0x08, 0x02000000u);    /* table size: 32 MiB (LE) */
}

/*
 * Shadow copy of ATF's GTA / platform tag area (0x4825ad1c).
 *
 * Same problem as the console node above: BL2 hands the platform tag list to
 * ATF by writing it into ATF's BSS, and ATF's cold-entry BSS clear (DC ZVA
 * over 0x4824b000..0x4825e000) wipes it.  ATF's xlat context setup then reads
 * an empty tag list, derives bogus page-table parameters and ends up
 * re-initialising the BL31 context, which clears the sys_cirq mapping that
 * APUSYS later needs.  Keep a shadow: ignore zero writes (the BSS clear) and
 * apply everything else.
 */
static uint8_t mt6895_atf_gta_shadow[0x100];
static bool mt6895_atf_gta_shadow_ready;

static uint64_t mt6895_atf_gta_read(void *opaque, hwaddr off, unsigned size)
{
    size_t i;
    uint64_t v = 0;

    (void)opaque;
    /*
     * Keep the platform tag present: the firmware zeroes (and later rewrites)
     * this area during its own setup, so re-seed it whenever the magic is
     * gone.  The tag is platform configuration this machine owns, so it is
     * never legitimately replaced by firmware data.
     */
    if (!mt6895_atf_gta_shadow_ready ||
        ldq_le_p(mt6895_atf_gta_shadow) != 0x00535312ULL) {
        mt6895_atf_gta_shadow_ready = true;
        mt6895_atf_gta_plant(mt6895_atf_gta_shadow);
        fprintf(stderr, "MTK-GTAPLANT: seeded tag area (on read)\n");
    }
    if (off + size > sizeof(mt6895_atf_gta_shadow)) {
        return 0;
    }
    for (i = 0; i < size; i++) {
        v |= (uint64_t)mt6895_atf_gta_shadow[off + i] << (8 * i);
    }
    if (MTK_ENV_CACHED("MTK_GTA_RDLOG")) {
        static int nrd;
        ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));

        if (nrd < 10) {
            nrd++;
            fprintf(stderr, "MTK-GTARD off=%#" HWADDR_PRIx " pc=%#" PRIx64
                    " val=%#" PRIx64 " size=%u sh=%02x%02x%02x%02x\n", off,
                    cpu ? (uint64_t)cpu->env.pc : 0, v, size,
                    mt6895_atf_gta_shadow[0], mt6895_atf_gta_shadow[1],
                    mt6895_atf_gta_shadow[2], mt6895_atf_gta_shadow[3]);
        }
    }
    return v;
}

static void mt6895_atf_gta_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    size_t i;

    (void)opaque;
    if (!mt6895_atf_gta_shadow_ready) {
        /* Seed the platform tag before the first firmware write lands. */
        mt6895_atf_gta_shadow_ready = true;
        mt6895_atf_gta_plant(mt6895_atf_gta_shadow);
        fprintf(stderr, "MTK-GTAPLANT: seeded tag area (on write) shadow=%02x%02x%02x%02x %02x%02x%02x%02x\n",
                mt6895_atf_gta_shadow[0], mt6895_atf_gta_shadow[1],
                mt6895_atf_gta_shadow[2], mt6895_atf_gta_shadow[3],
                mt6895_atf_gta_shadow[4], mt6895_atf_gta_shadow[5],
                mt6895_atf_gta_shadow[6], mt6895_atf_gta_shadow[7]);
    }
    if (off + size > sizeof(mt6895_atf_gta_shadow)) {
        return;
    }
    /* ignore the BSS clear: DC ZVA stores zero in 16-byte chunks */
    if (val == 0 && size >= 8) {
        return;
    }
    for (i = 0; i < size; i++) {
        mt6895_atf_gta_shadow[off + i] = (val >> (8 * i)) & 0xff;
    }
}

static const MemoryRegionOps mt6895_atf_gta_ops = {
    .read = mt6895_atf_gta_read,
    .write = mt6895_atf_gta_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/*
 * Points ATF's console driver at a usable UART object (see the registration
 * site in the machine init for why this global has to be supplied).
 */
static uint64_t mt6895_atf_uart_read(void *opaque, hwaddr off, unsigned size)
{
    (void)opaque;
    (void)size;
    (void)off;
    return 0x4824a100ULL;      /* pointer slot: ATF's UART object */
}

static uint64_t mt6895_atf_uartobj_read(void *opaque, hwaddr off, unsigned size)
{
    (void)opaque;
    (void)size;
    if (off == 0x14) {
        /*
         * TX-ready bit. The console writer loops on
         * "ldr w2,[uart,#0x14]; and w2,#0x20; cbz -> loop" (0x48204d1c) and
         * the object initialiser never sets this field, so without it ATF
         * spins forever on the first character.
         */
        return 0x20;
    }
    return 0x4825d000ULL;
}

static void mt6895_atf_uart_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    (void)opaque; (void)off; (void)val; (void)size;
}

static void mt6895_atf_uartobj_write(void *opaque, hwaddr off, uint64_t val,
                                     unsigned size)
{
    (void)opaque; (void)size;
    if (off == 0) {
        /*
         * The character itself: surface ATF's console on stderr, one line at
         * a time, so the firmware's own log is readable.
         */
        static char line[256];
        static size_t len;

        if (val == '\n' || len == sizeof(line) - 1) {
            line[len] = '\0';
            fprintf(stderr, "ATF-CONSOLE: %s (pc=%#" PRIx64 ")\n", line,
                    current_cpu ? (uint64_t)ARM_CPU(current_cpu)->env.regs[15] : 0);
            len = 0;
        } else if (val != '\r') {
            line[len++] = (char)val;
        }
    }
}



/*
 * MTK CPC/SMP power controller bridge (dcm-mp-cpusys-top block, 0x0C538000).
 *
 * ATF's CPU_ON path (plat/mediatek/drivers/cpu_pm/cpcv3_2/mt_smp.c) does, per
 * target CPU n:
 *   1. write the secondary entry point to CPC 0x0C53C900 + 8*n
 *      (0x48211fd8: str w3,[x8]); the value observed in the running boot is
 *      the BL31 warm boot entry 0x48200100;
 *   2. set the per-CPU power-on request bit 16+n in CPC 0x0C53C8E4
 *      (0x48212014);
 *   3. poll the per-CPU power-state bit n in CPC 0x0C53A840
 *      (0x482120d8 = the mt_smp.c:162 wait).  If that bit is already set it
 *      skips the whole power-on sequence and returns success.
 *
 * This machine used to answer every read of 0x0C53A840 with all ones, so
 * step 3 always saw the CPU as already powered: no core was ever started and
 * the Linux SMP bring-up timed out (5.32 guest seconds per CPU, then
 * "CPUx: failed to come online").
 *
 * Model the three registers for real and bridge step 2 into QEMU's power
 * control: when a request bit rises, start the matching vCPU with
 * Model the CPC power controller registers for real and bridge step 2 into
 * QEMU's power control: when a request bit rises, start the matching vCPU with
 * arm_set_cpu_on() at the entry point ATF wrote in step 1, and report the CPU
 * as powered for step 3. The bridge is default-ON; MTK_LEGACY_CPCOFF=1 preserves
 * the old all-ones legacy behavior for A/B testing; MTK_CPCLOG=1 prints the bridge
 * decisions.
 */
#define MT6895_CPC_CTRL_ADDR    0x0c53a814ULL
#define MT6895_CPC_STATUS_ADDR  0x0c53a840ULL
#define MT6895_CPC_ACK_ADDR     0x0c53a8a8ULL
#define MT6895_CPC_BIGCORE_ADDR 0x0c53ad8cULL
#define MT6895_CPC_REQ_ADDR     0x0c53c8e4ULL
#define MT6895_CPC_ENTRY_ADDR   0x0c53c900ULL
#define MT6895_CPC_ENTRY_COUNT  8
#define MT6895_CPC_REQ_BIT0     16

static uint64_t mt6895_cpc_entries[MT6895_CPC_ENTRY_COUNT];
static uint32_t mt6895_cpc_req_word;
static uint32_t mt6895_cpc_powered = 1;   /* CPU0 runs from reset */
static uint32_t mt6895_cpc_ack = 1;       /* CPU0 boot ack */
static uint32_t mt6895_cpc_ctrl;
static uint32_t mt6895_cpc_bigcore;
static uint32_t mt6895_cpc_pending;       /* cores with a queued power-up */
static QEMUTimer *mt6895_cpc_power_timer;

static void mt6895_cpc_state_reset(void)
{
    memset(mt6895_cpc_entries, 0, sizeof(mt6895_cpc_entries));
    mt6895_cpc_req_word = 0;
    mt6895_cpc_powered = 1;
    mt6895_cpc_ack = 1;
    mt6895_cpc_ctrl = 0;
    mt6895_cpc_bigcore = 0;
    mt6895_cpc_pending = 0;
    if (mt6895_cpc_power_timer) {
        timer_del(mt6895_cpc_power_timer);
    }
}

/*
 * DIAGNOSTIC (MTK_CPCTRACE=1): log every CPC register access with value and
 * the accessing PC.  Pure observation: nothing is written or faked; the trace
 * is how the CPU_ON handshake between the guest ATF and this device is verified.
 */
static void mt6895_cpc_trace(const char *op, uint64_t addr, uint64_t val,
                             unsigned size)
{
    ARMCPU *cpu = ARM_CPU(current_cpu);
    uint64_t pc = 0;

    if (cpu) {
        pc = cpu->env.aarch64 ? cpu->env.pc : (uint64_t)cpu->env.regs[15];
    }
    fprintf(stderr, "MTK-CPC-TRACE %s addr=%#" PRIx64 " val=%#" PRIx64
            " size=%u pc=%#" PRIx64 "\n", op, addr, val, size, pc);
}

static uint64_t mt6895_cpc_status_read(void *opaque, hwaddr off, unsigned size)
{
    uint64_t v;

    (void)opaque; (void)off;
    if (MTK_ENV_CACHED("MTK_LEGACY_CPCOFF")) {
        return (size >= 8) ? ~0ULL : ((1ULL << (size * 8)) - 1);
    }
    v = mt6895_cpc_powered;
    if (MTK_ENV_CACHED("MTK_CPCTRACE")) {
        mt6895_cpc_trace("rd-status", MT6895_CPC_STATUS_ADDR + off, v, size);
    }
    return v;
}

static void mt6895_cpc_status_write(void *opaque, hwaddr off, uint64_t val,
                                    unsigned size)
{
    (void)opaque; (void)off; (void)size;
    if (MTK_ENV_CACHED("MTK_CPCTRACE")) {
        mt6895_cpc_trace("wr-status", MT6895_CPC_STATUS_ADDR + off, val, size);
    }
}

static uint64_t mt6895_cpc_ack_read(void *opaque, hwaddr off, unsigned size)
{
    (void)opaque; (void)off;
    if (MTK_ENV_CACHED("MTK_CPCTRACE")) {
        mt6895_cpc_trace("rd-ack", MT6895_CPC_ACK_ADDR + off, mt6895_cpc_ack, size);
    }
    return mt6895_cpc_ack;
}

static void mt6895_cpc_ack_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    (void)opaque; (void)off; (void)size;
    if (MTK_ENV_CACHED("MTK_CPCTRACE")) {
        mt6895_cpc_trace("wr-ack", MT6895_CPC_ACK_ADDR + off, val, size);
    }
    mt6895_cpc_ack |= (uint32_t)val;
    mt6895_cpc_powered |= (uint32_t)val;
}

static uint64_t mt6895_cpc_ctrl_read(void *opaque, hwaddr off, unsigned size)
{
    (void)opaque; (void)off;
    if (MTK_ENV_CACHED("MTK_CPCTRACE")) {
        mt6895_cpc_trace("rd-ctrl", MT6895_CPC_CTRL_ADDR + off, mt6895_cpc_ctrl, size);
    }
    return mt6895_cpc_ctrl;
}

static void mt6895_cpc_ctrl_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    (void)opaque; (void)off; (void)size;
    if (MTK_ENV_CACHED("MTK_CPCTRACE")) {
        mt6895_cpc_trace("wr-ctrl", MT6895_CPC_CTRL_ADDR + off, val, size);
    }
    mt6895_cpc_ctrl = (uint32_t)val;
}

static uint64_t mt6895_cpc_bigcore_read(void *opaque, hwaddr off, unsigned size)
{
    (void)opaque; (void)off;
    if (MTK_ENV_CACHED("MTK_CPCTRACE")) {
        mt6895_cpc_trace("rd-bigcore", MT6895_CPC_BIGCORE_ADDR + off,
                         mt6895_cpc_bigcore, size);
    }
    return mt6895_cpc_bigcore;
}

static void mt6895_cpc_bigcore_write(void *opaque, hwaddr off, uint64_t val,
                                     unsigned size)
{
    (void)opaque; (void)off; (void)size;
    if (MTK_ENV_CACHED("MTK_CPCTRACE")) {
        mt6895_cpc_trace("wr-bigcore", MT6895_CPC_BIGCORE_ADDR + off, val, size);
    }
    mt6895_cpc_bigcore = (uint32_t)val;
}

static uint64_t mt6895_cpc_req_read(void *opaque, hwaddr off, unsigned size)
{
    (void)opaque; (void)off; (void)size;
    if (MTK_ENV_CACHED("MTK_CPCTRACE")) {
        mt6895_cpc_trace("rd-req", MT6895_CPC_REQ_ADDR + off,
                         mt6895_cpc_req_word, size);
    }
    return mt6895_cpc_req_word;
}

/*
 * Power-up completion for cores requested through the CPC bridge.
 *
 * Hardware does not finish a CPU_ON inside the primary's register write: the
 * request starts the power controller, the core is released, and the per-core
 * power-status bit only rises once that power-up has happened.  The guest's
 * ATF depends on exactly that ordering: mt_smp.c writes the request and then
 * polls bit n of 0x0C53A840, expecting it to be clear on the first poll and set
 * on a later iteration, while it drives the SPM power-up sequence.  Doing the
 * whole thing synchronously in the request write skipped ATF's own power-up
 * path and released the core before TF-A had committed the target's
 * non-secure context - that inverted causality is what this timer restores.
 *
 * 1 ms of virtual power-up latency stays well inside ATF's wait budget
 * (mt_smp polls 0xC351 times with a 1 us delay, i.e. about 50 ms).
 */
#define MT6895_CPC_POWER_ON_NS  1000000ULL

static void mt6895_cpc_mark_powered(CPUState *cs, run_on_cpu_data data)
{
    unsigned n = data.host_ulong;

    (void)cs;
    mt6895_cpc_powered |= 1u << n;
}

static void mt6895_cpc_power_on_tick(void *opaque)
{
    unsigned n;

    (void)opaque;
    for (n = 0; n < MT6895_CPC_ENTRY_COUNT; n++) {
        uint64_t entry = mt6895_cpc_entries[n];
        CPUState *target;
        int ret;

        if (!(mt6895_cpc_pending & (1u << n))) {
            continue;
        }
        mt6895_cpc_pending &= ~(1u << n);
        if (entry == 0) {
            continue;
        }
        ret = arm_set_cpu_on((uint64_t)n << 8, entry, 0, 3, true);
        if (MTK_ENV_CACHED("MTK_CPCLOG")) {
            fprintf(stderr, "MTK-CPC on cpu%u entry=%#" PRIx64 " ret=%d\n",
                    n, entry, ret);
        }
        if (ret != 0) {
            continue;
        }
        /*
         * arm_set_cpu_on() has queued the target's reset/start work on its own
         * vCPU thread; queue the status update behind it so the bit only rises
         * after the core has actually been reset and started.
         */
        target = arm_get_cpu_by_id((uint64_t)n << 8);
        if (target) {
            async_run_on_cpu(target, mt6895_cpc_mark_powered,
                             RUN_ON_CPU_HOST_ULONG(n));
        } else {
            mt6895_cpc_powered |= 1u << n;
        }
    }
}

static void mt6895_cpc_req_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    uint32_t raised;
    int n;

    (void)opaque; (void)off; (void)size;
    if (MTK_ENV_CACHED("MTK_CPCTRACE")) {
        mt6895_cpc_trace("wr-req", MT6895_CPC_REQ_ADDR + off, val, size);
    }
    raised = (uint32_t)val & ~mt6895_cpc_req_word;
    mt6895_cpc_req_word = val;
    if (MTK_ENV_CACHED("MTK_LEGACY_CPCOFF")) {
        return;
    }
    for (n = 0; n < MT6895_CPC_ENTRY_COUNT; n++) {
        uint32_t bit = 1u << (MT6895_CPC_REQ_BIT0 + n);

        if (!(raised & bit) || mt6895_cpc_entries[n] == 0) {
            continue;
        }
        /*
         * Do not touch the status bit or arm_set_cpu_on() here: the power-up
         * itself is a hardware sequence that completes after this write, and
         * ATF polls the status register expecting to see it still clear first.
         */
        mt6895_cpc_pending |= 1u << n;
        if (!mt6895_cpc_power_timer) {
            mt6895_cpc_power_timer =
                timer_new_ns(QEMU_CLOCK_VIRTUAL, mt6895_cpc_power_on_tick,
                             NULL);
        }
        timer_mod(mt6895_cpc_power_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  MT6895_CPC_POWER_ON_NS);
    }
}

static uint64_t mt6895_cpc_entry_read(void *opaque, hwaddr off, unsigned size)
{
    unsigned n = off / 8;
    bool hi = (off % 8) != 0;

    (void)opaque; (void)size;
    if (n >= MT6895_CPC_ENTRY_COUNT) {
        return 0;
    }
    {
        uint64_t v = hi ? (uint32_t)(mt6895_cpc_entries[n] >> 32)
                        : (uint32_t)mt6895_cpc_entries[n];

        if (MTK_ENV_CACHED("MTK_CPCTRACE")) {
            mt6895_cpc_trace("rd-entry", MT6895_CPC_ENTRY_ADDR + off, v, size);
        }
        return v;
    }
}

static void mt6895_cpc_entry_write(void *opaque, hwaddr off, uint64_t val,
                                   unsigned size)
{
    unsigned n = off / 8;
    bool hi = (off % 8) != 0;

    (void)opaque; (void)size;
    if (n >= MT6895_CPC_ENTRY_COUNT) {
        return;
    }
    if (MTK_ENV_CACHED("MTK_CPCTRACE")) {
        mt6895_cpc_trace("wr-entry", MT6895_CPC_ENTRY_ADDR + off, val, size);
    }
    if (hi) {
        mt6895_cpc_entries[n] = (mt6895_cpc_entries[n] & 0xffffffffULL) |
                                ((uint64_t)(uint32_t)val << 32);
    } else {
        mt6895_cpc_entries[n] = (mt6895_cpc_entries[n] & ~0xffffffffULL) |
                                (uint32_t)val;
    }
}

/*
 * Write watch on the pl-boottag mblock (0x48600000, 1 MiB).
 *
 * The preloader builds the boot tag LK later reads, and prints
 * "boot_tag size = 0x0", i.e. it adds no entries -- which is why LK cannot
 * find its main DTB. Log who writes here (PC + value), passing the write
 * through to DRAM, so the tag layout and the reason it stays empty become
 * visible in one run.
 */
static uint64_t mt6895_bt_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, 0x48600000ULL + off, size);
    uint64_t v = 0;
    unsigned i;
    static int nrd;

    if (MTK_ENV_CACHED("MTK_BTLOG") && nrd < 24) {
        nrd++;
        fprintf(stderr, "MTK-BT-RD off=%#x size=%u pc=%#" PRIx64 "\n",
                (unsigned)off, size,
                current_cpu ? (uint64_t)ARM_CPU(current_cpu)->env.regs[15] : 0);
    }
    if (!p) {
        return 0;
    }
    for (i = 0; i < size; i++) {
        v |= (uint64_t)p[i] << (8 * i);
    }
    /*
     * Experiment hook: MTK_BOOT_MODE=<n> forces the value of the boot tag's
     * BOOT_MODE entry (tag id 0x88610002, value 1 = normal boot), which is how
     * the preloader tells
     * LK which mode to enter (1 = normal boot).  Applied on the read path so
     * the tag bytes the guest wrote stay untouched; without the variable the
     * read returns exactly what is in DRAM.
     */
    {
        static int mode_cached = -2;
        static char mode_val[32];

        if (mode_cached == -2) {
            const char *e = getenv("MTK_BOOT_MODE");

            mode_cached = -1;
            if (e && *e) {
                snprintf(mode_val, sizeof(mode_val), "%s", e);
                mode_cached = 0;
                fprintf(stderr, "MTK-BT: forcing BOOT_MODE=%s\n", mode_val);
            }
        }
        if (mode_cached == 0) {
            unsigned base;

            for (base = 0; base + 16 <= 0x1000; base += 16) {
                uint8_t *q = mt6895_guest_ptr(s, 0x48600000ULL + base, 16);
                uint32_t id;
                uint64_t val, out = 0;
                unsigned a;

                if (!q) {
                    break;
                }
                id = q[4] | ((uint32_t)q[5] << 8) |
                     ((uint32_t)q[6] << 16) | ((uint32_t)q[7] << 24);
                if (id != 0x88610002 || off < base + 8 || off >= base + 16) {
                    continue;
                }
                val = strtoull(mode_val, NULL, 0);
                for (a = 0; a < size; a++) {
                    unsigned long pos = (unsigned long)off + a;
                    uint8_t b;

                    if (pos >= base + 8 && pos < base + 16) {
                        b = (val >> (8 * (pos - (base + 8)))) & 0xff;
                    } else {
                        b = p[a];
                    }
                    out |= (uint64_t)b << (8 * a);
                }
                return out;
            }
        }
    }
    return v;
}

static void mt6895_bt_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, 0x48600000ULL + off, size);
    ARMCPU *cpu = ARM_CPU(current_cpu);
    static int n;
    unsigned i;

    if (MTK_ENV_CACHED("MTK_BTLOG") && n < 40) {
        n++;
        fprintf(stderr, "MTK-BT-WR off=%#x val=%#" PRIx64 " size=%u pc=%#"
                PRIx64 " lr=%#" PRIx64 "\n", (unsigned)off, val, size,
                cpu ? (uint64_t)cpu->env.regs[15] : 0,
                cpu ? (uint64_t)cpu->env.regs[14] : 0);
    }
    if (!p) {
        return;
    }
    {
        static int nw;

        if (MTK_ENV_CACHED("MTK_BTLOG") && nw < 40) {
            nw++;
            fprintf(stderr, "MTK-ATFARG2-WR off=%#x size=%u val=%#" PRIx64
                    " pc=%#" PRIx64 "\n", (unsigned)off, size, val,
                    current_cpu ? (uint64_t)ARM_CPU(current_cpu)->env.pc : 0);
        }
    }
    for (i = 0; i < size; i++) {
        p[i] = (val >> (8 * i)) & 0xff;
    }
}

static const MemoryRegionOps mt6895_bt_ops = {
    .read = mt6895_bt_read,
    .write = mt6895_bt_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
    .impl = { .min_access_size = 1, .max_access_size = 8 },
};

/*
 * DIAGNOSTIC (MTK_BAGWATCH): transparent, read-through observation of the
 * regions that decide the LK boot-argument path.  Accesses are forwarded to
 * RAM unchanged; only logging is added.
 *
 * The 0x020F7D30 boot-parameter window is deliberately SPLIT around
 * 0x020F7D78..0x020F7D7B: that word is served by the lk_bootarg plant, and a
 * priority-4 overlay over it would shadow the plant, change boot behaviour and
 * produce the very abort this diagnostic is meant to observe.
 */
#define MT6895_BAG_BLK_BASE   0x020F3740ULL
#define MT6895_BAG_BLK_SIZE   0x0000093C
#define MT6895_BAG_BA_LO_BASE 0x020F7D30ULL
#define MT6895_BAG_BA_LO_SIZE 0x00000048
#define MT6895_BAG_BA_HI_BASE 0x020F7D7CULL
#define MT6895_BAG_BA_HI_SIZE 0x00000124

typedef struct Mt6895BagRegion {
    Mt6895MachineState *s;
    hwaddr base;
    unsigned nrd;
    unsigned nwr;
} Mt6895BagRegion;

static Mt6895BagRegion mt6895_bag_blk;
static Mt6895BagRegion mt6895_bag_ba_lo;
static Mt6895BagRegion mt6895_bag_ba_hi;
static Mt6895BagRegion mt6895_bag_gl1;
static Mt6895BagRegion mt6895_bag_gl2;

static void mt6895_bag_log(const char *tag, hwaddr base, hwaddr off,
                           uint64_t val, unsigned size, bool wr)
{
    ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
    uint64_t pc = 0, lr = 0;

    if (cpu) {
        pc = cpu->env.pc;
        lr = cpu->env.aarch64 ? cpu->env.xregs[30] : cpu->env.regs[14];
    }
    fprintf(stderr, "MTK-%s %s base=%#" HWADDR_PRIx " off=%#x abs=%#"
            HWADDR_PRIx " val=%#" PRIx64 " size=%u pc=%#" PRIx64
            " lr=%#" PRIx64 "\n", tag, wr ? "WR" : "RD", base,
            (unsigned)off, base + off, val, size, pc, lr);
}

static uint64_t mt6895_bag_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895BagRegion *r = opaque;
    uint8_t *p = mt6895_guest_ptr(r->s, r->base + off, size);
    uint64_t v = 0;
    unsigned i;

    if (!p) {
        return 0;
    }
    for (i = 0; i < size; i++) {
        v |= (uint64_t)p[i] << (8 * i);
    }
    if (r->nrd < 200) {
        r->nrd++;
        mt6895_bag_log("BAG", r->base, off, v, size, false);
    }
    return v;
}

static void mt6895_bag_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    Mt6895BagRegion *r = opaque;
    uint8_t *p = mt6895_guest_ptr(r->s, r->base + off, size);
    unsigned i;

    if (!p) {
        return;
    }
    if (r->nwr < 120) {
        r->nwr++;
        mt6895_bag_log("BAG", r->base, off, val, size, true);
    }
    for (i = 0; i < size; i++) {
        p[i] = (val >> (8 * i)) & 0xff;
    }
}

static const MemoryRegionOps mt6895_bag_ops = {
    .read = mt6895_bag_read,
    .write = mt6895_bag_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
    .impl = { .min_access_size = 1, .max_access_size = 8 },
};

static const MemoryRegionOps mt6895_cpc_status_ops = {
    .read = mt6895_cpc_status_read,
    .write = mt6895_cpc_status_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static const MemoryRegionOps mt6895_cpc_ack_ops = {
    .read = mt6895_cpc_ack_read,
    .write = mt6895_cpc_ack_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static const MemoryRegionOps mt6895_cpc_ctrl_ops = {
    .read = mt6895_cpc_ctrl_read,
    .write = mt6895_cpc_ctrl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static const MemoryRegionOps mt6895_cpc_bigcore_ops = {
    .read = mt6895_cpc_bigcore_read,
    .write = mt6895_cpc_bigcore_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static const MemoryRegionOps mt6895_cpc_req_ops = {
    .read = mt6895_cpc_req_read,
    .write = mt6895_cpc_req_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static const MemoryRegionOps mt6895_cpc_entry_ops = {
    .read = mt6895_cpc_entry_read,
    .write = mt6895_cpc_entry_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};


static const MemoryRegionOps mt6895_atf_uart_ops = {
    .read = mt6895_atf_uart_read,
    .write = mt6895_atf_uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static const MemoryRegionOps mt6895_atf_uartobj_ops = {
    .read = mt6895_atf_uartobj_read,
    .write = mt6895_atf_uartobj_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* The payload image's own vtable bytes at 0x02078DE8. */
static const uint32_t mt6895_vtbl_default[] = {
    0x0203b141,
    0x0203b231,
    0x0203b2f9,
    0x0203b349,
    0x0203b3b1,
    0x0203b419,
    0x00000000,
    0x00000000,
    0x0203b4a5,
};

/*
 * The payload .data vtable region, overlaid with the image's own bytes.
 * Read-only by construction: the payload never writes these slots after
 * start-up, it only loads function pointers out of them, so serving the
 * image bytes cannot mask anything the guest produces.
 */
static uint64_t mt6895_rodata_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off / 4 < ARRAY_SIZE(s->rodata_store) && s->rodata_store[off / 4]) {
        return extract64(s->rodata_store[off / 4], 0, size * 8);
    }
    /*
     * Fall back to the image's own bytes. Without this the guest's BSS
     * clear (which writes zero through this overlay) leaves the vtable
     * zeroed and the dispatch at 0x0204001c branches to garbage.
     */
    if (off + size <= MT6895_RODATA_SIZE) {
        return extract64(mt6895_vtbl_default[off / 4], 0, size * 8);
    }
    return 0;
}

static void mt6895_rodata_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off + size <= MT6895_RODATA_SIZE) {
        s->rodata_store[off / 4] = val;
    }
}

static const MemoryRegionOps mt6895_rodata_ops = {
    .read = mt6895_rodata_read,
    .write = mt6895_rodata_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/*
 * The machine used to overlay four payload PMIC structures here (device table,
 * PMIC ops vtable, auxadc registry, auxadc device object) with synthesised
 * copies.  With the SPMI bus now modelled the payload builds them itself and
 * the overlays poisoned the regulator table, so they are deleted: the guest
 * sees the payload's own data.  See the note in mt6895_init().
 */

/*
 * The MTK I2C device descriptor table at 0x020F369C: ten entries of 16 bytes,
 * indexed by the device id stored in the driver object at +4.
 *
 * The poll core at 0x02034414 refuses to start a transfer unless the entry's
 * first byte is non-zero and either byte +1 or byte +0xf is non-zero:
 *
 *   0x02034470  ldrb r2, [r1, r2]    ; entry + 0
 *   0x0203447A  ldrb r1, [r0, #1]    ; entry + 1
 *   0x0203447C  cbnz r1, ok
 *   0x0203447E  ldrb r0, [r0, #0xf]  ; entry + 0xf
 *   0x02034482  beq  fail
 *
 * The whole table reads zero in this machine (nothing in the payload image
 * writes it), so device id 5 -- the bus the MT6375 PMIC hangs off at slave
 * 0x4A, per the log line "[I2C] 339: id=5,addr: 4A, transfer error" -- is
 * rejected before any register is touched, and the boot later dies in a
 * strcmp data abort with an uninitialised auxadc device object.
 *
 * This overlay supplies an entry for id 5 only; the other ids stay zero,
 * which the same code path treats as "no such bus" rather than as an error.
 */
static const uint8_t mt6895_i2c_desc_default[] = {
    /* 0x020F369C: id 0..4 left zero */
    [0x50] = 0x01,   /* id 5 + 0x00: present */
    [0x51] = 0x01,   /* id 5 + 0x01: bus is usable */
    /* 0x020F36F8: id 5 + 0x0c left zero */
    /* 0x020F36FC: id 6..9 left zero */
};

static uint64_t mt6895_i2c_desc_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off / 4 < ARRAY_SIZE(s->i2c_desc_store) && s->i2c_desc_store[off / 4]) {
        return extract64(s->i2c_desc_store[off / 4], 0, size * 8);
    }
    if (off < ARRAY_SIZE(mt6895_i2c_desc_default) &&
        mt6895_i2c_desc_default[off]) {
        return mt6895_i2c_desc_default[off];
    }
    return 0;
}

static void mt6895_i2c_desc_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off / 4 < ARRAY_SIZE(s->i2c_desc_store)) {
        s->i2c_desc_store[off / 4] = val;
    }
}

static const MemoryRegionOps mt6895_i2c_desc_ops = {
    .read = mt6895_i2c_desc_read,
    .write = mt6895_i2c_desc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * EXPERIMENT: the [SEC_POLICY] table at 0x020793D8.
 *
 * The policy lookup walks 18 entries of 28 bytes from base+8, reading four
 * "name" pointers per entry. That range lands inside the partition name table
 * the payload rebuilds from the GPT, whose stride is 20 rather than 28, so the
 * walk drifts and eventually strcmps against an LBA or size field --
 * measured: r1 = 0x5800 (entry 67's LBA) and a data abort inside strcmp.
 *
 * Serving reads as zero and dropping writes makes every entry's four cbz
 * branches skip, so the walk runs to its end and falls back to the default
 * policy ("[SEC_POLICY] reached the end, use default policy"). Writes are
 * dropped, so the payload's rebuild of this range is lost -- acceptable only
 * because every partition the boot still needs (bl2_ext at entry 27, proinfo
 * at entry 9) sits below 0x020793D8. If a later lookup for a higher entry
 * fails, this is the thing to revisit.
 */
static uint64_t mt6895_secpol_read(void *opaque, hwaddr off, unsigned size)
{
    return 0;
}

static void mt6895_secpol_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    /* dropped: see the note above */
}

static const MemoryRegionOps mt6895_secpol_ops = {
    .read = mt6895_secpol_read,
    .write = mt6895_secpol_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * The PL log-store header the payload validates at 0x0011DF34.
 *
 * 0x02035048 checks three words and then two more:
 *
 *   0x02035054  ldr r0,[r4]; cmp r0,#0xCDAB3412     signature
 *   0x0203505C  ldr r0,[r4,#4]; cmp r0,#0x40000     buffer size
 *   0x02035064  cmpeq r0,#0x400                     third word
 *   0x020350DE  ([r4+0xc] + [r4+0x18] + 0x400) <= 0x40000
 *   0x020350E4  [r4+0x10] <= 0x7f  and  [r4+0x1c] >= 0x80
 *
 * and its diagnostics name the rest of the layout:
 *   +0x08 sz_pl  +0x0c pl_flag  +0x10 off_pl
 *   +0x14 sz_lk  +0x18 lk_flag  +0x1c off_lk
 *
 * The machine previously laid a header at 0x00000A00 with signature
 * 0xABCD1234, which is neither the address nor the value this routine reads,
 * so it always failed and the "Second Bootloader Load Failed" path followed.
 * This overlay serves the words it actually wants.
 */
static const uint32_t mt6895_loghdr_default[] = {
    [0x00 / 4] = 0xcdab3412,   /* +0x00 signature              */
    [0x04 / 4] = 0x00040000,   /* +0x04 buffer size            */
    [0x08 / 4] = 0x00000400,   /* +0x08 sz_pl                  */
    [0x0c / 4] = 0x00000000,   /* +0x0c pl_flag                */
    [0x10 / 4] = 0x00000000,   /* +0x10 off_pl  (<= 0x7f)      */
    [0x14 / 4] = 0x00000400,   /* +0x14 sz_lk                  */
    [0x18 / 4] = 0x00000000,   /* +0x18 lk_flag                */
    [0x1c / 4] = 0x00000080,   /* +0x1c off_lk  (>= 0x80)      */
};

static uint64_t mt6895_loghdr_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off / 4 < ARRAY_SIZE(s->loghdr_store) && s->loghdr_store[off / 4]) {
        return extract64(s->loghdr_store[off / 4], 0, size * 8);
    }
    if (off / 4 < ARRAY_SIZE(mt6895_loghdr_default)) {
        return extract64(mt6895_loghdr_default[off / 4], 0, size * 8);
    }
    return 0;
}

static void mt6895_loghdr_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off / 4 < ARRAY_SIZE(s->loghdr_store)) {
        s->loghdr_store[off / 4] = val;
    }
}

static const MemoryRegionOps mt6895_loghdr_ops = {
    .read = mt6895_loghdr_read,
    .write = mt6895_loghdr_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * The boot-control block the payload keeps at 0x020F2310.
 *
 * get_suffix (0x0200E3E4) calls a reader (0x0200E47C) whose first act on the
 * mode-0 path is to look at word +4 of this buffer: if it already holds the
 * magic 0x42414342 ("BCAB") the routine takes the branch at 0x0200E52A
 * straight to the success path and never touches the storage device.
 *
 * Otherwise it reads 32 bytes from the "misc" partition (0x020405E0), and on
 * this machine that read returns <= 0, which prints "[bootctrl] read
 * boot_control fail", makes get_suffix return -1, and ends the boot with
 * "[BLDR] Second Bootloader Load Failed".
 *
 * So the block is supplied here rather than through the partition, and the
 * layout matches what get_suffix reads afterwards: +0x0c and +0x0e are the
 * two 16-bit slot priorities, of which bits 4..6 hold the retry counts.
 * Slot 0 wins on priority, so the suffix resolves to "_a", which is the slot
 * the images in the device file occupy.
 */
static const uint32_t mt6895_bcb_default[] = {
    [0x00 / 4] = 0x00000000,             /* +0x00 (unused by the reader)   */
    [0x04 / 4] = 0x42414342,             /* +0x04 magic "BCAB"             */
    [0x08 / 4] = 0x00000000,
    [0x0c / 4] = 0x00fe00ff,             /* +0x0c: slot0 (0x00ff) success=1, tries=7, prio=15
                                          * +0x0e: slot1 (0x00fe) success=1, tries=7, prio=14 */
    [0x10 / 4] = 0x00000000,
    [0x14 / 4] = 0x00000000,
    [0x18 / 4] = 0x00000000,
    [0x1c / 4] = 0x00000000,             /* +0x1c crc (only checked after  */
                                          /*        a successful read)      */
    [0x20 / 4] = 0x00000000,
};

static uint64_t mt6895_bcb_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off / 4 < ARRAY_SIZE(s->bcb_store) && s->bcb_store[off / 4]) {
        return extract64(s->bcb_store[off / 4], 0, size * 8);
    }
    if (off / 4 < ARRAY_SIZE(mt6895_bcb_default)) {
        return extract64(mt6895_bcb_default[off / 4], 0, size * 8);
    }
    return 0;
}

static void mt6895_bcb_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off / 4 < ARRAY_SIZE(s->bcb_store)) {
        s->bcb_store[off / 4] = val;
    }
}

static const MemoryRegionOps mt6895_bcb_ops = {
    .read = mt6895_bcb_read,
    .write = mt6895_bcb_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};



/*
 * A one-node storage-device list for the lookup at 0x0200DEDC.
 *
 * That routine walks a singly linked list whose head pointer lives at
 * 0x020F130C, comparing each node's first word against the type it was asked
 * for. On this machine the head is zero, so the walk ends immediately and the
 * callers all inherit a NULL device:
 *
 *   - mt_get_part_sram  prints "%s%s fail, err(no dev)"
 *   - the rollback path  at 0x0200EA44 gets NULL, returns -1, and the boot
 *     ends on the assert at ab_bootcheck.c:54
 *
 * Serving a head pointer plus one node whose first word is 1 gives the walk a
 * match for the type the rollback path requests (0x0200EA48 passes 1), which
 * is the smallest change that lets it return a non-NULL device. Node words
 * beyond the first are zero: no caller in the paths that were failing reads
 * them, since they bail out before touching the device.
 */
static const uint32_t mt6895_bootdev_default[] = {
    [0x00 / 4] = 0x0207b000,   /* head pointer -> RKP node (protected) */ 
};

static const uint32_t mt6895_bootdev_node[] = {
    [0x00 / 4] = 0x00000001,   /* device type: matches the request   */
    /*
     * UFS LUNs report a 4096-byte logical block (the LU descriptor hands the
     * boot chain exactly that), so the boot-device node must advertise it too.
     * Advertising 512 makes the preloader convert partition LBAs with the
     * wrong shift: the secure-config partition ends up being read at
     * 0x3A00000 instead of 0x1D000000, so the seccfg magic never matches and
     * sec_lib_init() fails with 0x103000 for the rest of the boot.
     */
    [0x04 / 4] = 0x00001000,   /* 4096: block size in bytes          */
    [0x08 / 4] = 0x00001000,   /* alignment                          */
    [0x0c / 4] = 0x00000000,
    [0x10 / 4] = 0x00000000,
    [0x14 / 4] = 0x00000000,
    [0x18 / 4] = 0x00000000,
    [0x1c / 4] = 0x00000000,   /* next: end of list                  */
    [0x20 / 4] = 0x00000101,   /* read: 0x00000100 PV block driver   */
    [0x24 / 4] = 0x0200ded5,   /* write: movs r0, #0; bx lr          */
    [0x28 / 4] = 0x00000000,
    [0x2c / 4] = 0x0200ded5,   /* erase: movs r0, #0; bx lr          */
};

static uint64_t mt6895_bootdev_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    /*
     * MTK_BOOTDEV_ANY: the payload asks for device type 1 (main storage) and
     * type 2 (the "mem"/boot LUN device).  The machine only plants one node,
     * so the type-2 walk finds nothing and the DRAM-info read falls back to a
     * zero buffer ("dram info unset" -> cold-boot retrain loop).  Before the
     * walk reads the node's type word, publish the requested type into it so
     * every lookup matches; this is an experiment to confirm that the type-2
     * device is the missing "mem" one.
     */
    if (MTK_ENV_CACHED("MTK_BOOTDEV_ANY")) {
        CPUState *cs = qemu_get_cpu(0);
        uint32_t want = 0;

        if (cs) {
            want = cpu_env(cs)->regs[0];
        }
        if (want) {
            uint32_t t = want;

            address_space_write(&address_space_memory, 0x0207b000ULL,
                                MEMTXATTRS_UNSPECIFIED, &t, sizeof(t));
        }
    }
    if (MTK_ENV_CACHED("MTK_BOOTDEVTYPE")) {
        static int nt;
        CPUState *cs = qemu_get_cpu(0);
        uint32_t r0 = 0, r1 = 0;

        if (cs) {
            CPUARMState *env = cpu_env(cs);

            r0 = env->regs[0];
            r1 = env->regs[1];
        }
        if (nt < 60) {
            nt++;
            fprintf(stderr, "BOOTDEVTYPE off=%#x r0=%#" PRIx32 " r1=%#"
                    PRIx32 "\n", (unsigned)off, r0, r1);
        }
    }

    if (off / 4 < ARRAY_SIZE(s->bootdev_store) && s->bootdev_store[off / 4]) {
        return extract64(s->bootdev_store[off / 4], 0, size * 8);
    }
    if (off / 4 < ARRAY_SIZE(mt6895_bootdev_default)) {
        return extract64(mt6895_bootdev_default[off / 4], 0, size * 8);
    }
    return 0;
}

static void mt6895_bootdev_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off / 4 < ARRAY_SIZE(s->bootdev_store)) {
        s->bootdev_store[off / 4] = val;
    }
}

static const MemoryRegionOps mt6895_bootdev_ops = {
    .read = mt6895_bootdev_read,
    .write = mt6895_bootdev_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static uint64_t mt6895_bootdev_node_read(void *opaque, hwaddr off,
                                         unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off / 4 < ARRAY_SIZE(s->bootdev_node_store) &&
        s->bootdev_node_store[off / 4]) {
        return extract64(s->bootdev_node_store[off / 4], 0, size * 8);
    }
    if (off / 4 < ARRAY_SIZE(mt6895_bootdev_node)) {
        return extract64(mt6895_bootdev_node[off / 4], 0, size * 8);
    }
    return 0;
}

static void mt6895_bootdev_node_write(void *opaque, hwaddr off,
                                      uint64_t val, unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off / 4 < ARRAY_SIZE(s->bootdev_node_store)) {
        s->bootdev_node_store[off / 4] = val;
    }
}

static const MemoryRegionOps mt6895_bootdev_node_ops = {
    .read = mt6895_bootdev_node_read,
    .write = mt6895_bootdev_node_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};


/*
 * Two bytes of the slot table the payload keeps at 0x020F41B0.
 *
 * bl 0x02038834 -- the precondition both get_boot_part (0x02038614) and the
 * rollback (0x02038678) must pass -- starts with
 *
 *     ldrb r0, [r1, #0x269] ; cmp r0, #4 ; blo  <fail>
 *
 * where r1 is the table entry for slot 1, so the byte at
 * 0x020F41B0 + 0x2E4 + 0x269 = 0x020F471D has to be at least 4. The other
 * one, at entry + 0xFF = 0x020F4593, carries the slot in its bits 3..5 and is
 * what actually selects _a or _b.
 *
 * Both were measured as zero for the whole run, and a phase-swept scan of the
 * payload found no fixed-offset store to either, so nothing ever fills them in:
 * the precondition always fails, the rollback always fails, and the boot ends
 * on the assert at ab_bootcheck.c:54. The slot still resolves because
 * get_boot_part falls back to a default, which is why "[AB] ab_suffix: _a"
 * appears before the failure.
 *
 * The window is placed on a four-byte boundary and sized to cover both bytes:
 * an earlier attempt registered exactly 0x020F4593, which is not naturally
 * aligned, and the machine wedged before printing anything.
 */
static const uint8_t mt6895_slottbl_default[] = {
    [0x003] = 0x88,                       /* 0x020F4593: bit7 + bits3..5 = 1  */
    [0x18d] = 0x0f,                       /* 0x020F471D: 15 (>= 4)            */
};

static int mt6895_slotprobe_n;

static uint64_t mt6895_slottbl_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    /*
     * A bounded probe on the two bytes the rollback reads. Recording how many
     * times each is read -- and in what order -- shows whether the rollback
     * reaches the slot check at all before it gives up, which is the evidence
     * needed to place the next change. GDB cannot reach this path in one run,
     * so a trace is the only affordable way to see it.
     */
    if (mt6895_slotprobe_n < 48 &&
        (off == 0x3 || off == 0x18d)) {
        mt6895_slotprobe_n++;
        trace_mt6895_slotprobe(mt6895_slotprobe_n,
                               MT6895_SLOTTBL_BASE + off, size);
    }
    if (off / 4 < ARRAY_SIZE(s->slottbl_store) && s->slottbl_store[off / 4]) {
        return extract64(s->slottbl_store[off / 4], 0, size * 8);
    }
    if (off < MT6895_SLOTTBL_SIZE) {
        return mt6895_slottbl_default[off];
    }
    return 0;
}

static void mt6895_slottbl_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off + size <= MT6895_SLOTTBL_SIZE &&
        off / 4 < ARRAY_SIZE(s->slottbl_store)) {
        s->slottbl_store[off / 4] = val;
    }
}

static const MemoryRegionOps mt6895_slottbl_ops = {
    .read = mt6895_slottbl_read,
    .write = mt6895_slottbl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * The partition object the finder fills at 0x020F4AC8.
 *
 * The finder copies a name-table entry here and points +0x10 at a copy of the
 * name. This overlay survives the payload's BSS clear, so it must preserve
 * the guest's byte writes and return them to later range checks. The first
 * implementation only recorded writes and returned zero for every read,
 * which erased the size word at +4 and made every partition write fail.
 */
static int mt6895_objw_n;
static int mt6895_objw_diag_n;

static void mt6895_objw_diag(const char *kind, hwaddr off, unsigned size,
                             uint64_t val)
{
    ARMCPU *cpu = ARM_CPU(current_cpu);
    uint32_t pc = cpu ? (uint32_t)cpu->env.regs[15] : 0;
    uint32_t lr = cpu ? (uint32_t)cpu->env.regs[14] : 0;

    (void)pc;
    (void)lr;

    if (mt6895_objw_diag_n < 4) {
        mt6895_objw_diag_n++;
        fprintf(stderr,
                "mt6895-dbg: objw %s off=0x%02x size=%u value=0x%08" PRIx64
                " pc=%08x lr=%08x\n",
                kind, (unsigned)off, size, val, pc, lr);
    }
}

static uint64_t mt6895_objw_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint64_t val = 0;

    for (unsigned i = 0; i < size; i++) {
        if (off + i < sizeof(s->objw_store)) {
            val |= (uint64_t)s->objw_store[off + i] << (i * 8);
        }
    }
    mt6895_objw_diag("READ ", off, size, val);
    return val;
}

static void mt6895_objw_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    Mt6895MachineState *s = opaque;

    for (unsigned i = 0; i < size; i++) {
        if (off + i < sizeof(s->objw_store)) {
            s->objw_store[off + i] = (val >> (i * 8)) & 0xff;
        }
    }
    mt6895_objw_diag("WRITE", off, size, val);
    if (mt6895_objw_n < 200) {
        mt6895_objw_n++;
        trace_mt6895_objw(mt6895_objw_n, 0x020f4ac8ULL + off, size, val);
    }
}

static const MemoryRegionOps mt6895_objw_ops = {
    .read = mt6895_objw_read,
    .write = mt6895_objw_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static uint64_t mt6895_rkp_head_read(void *opaque, hwaddr off, unsigned size)
{
    /*
     * The caller passes 1 (0x0202C86A: "movs r0, #1") and the lookup compares
     * it against the first word of whatever this points at, so the pointer has
     * to address a table whose first entry is that value. The table lives at
     * 0x0207B000 -- past the payload and, more importantly, outside the range
     * the payload clears (0x020F1000..0x020FB870) -- and the machine builds it
     * alongside the other BootROM data.
     */
    return extract64(MT6895_RKP_TABLE_BASE, 0, size * 8);
}

static void mt6895_rkp_head_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    Mt6895MachineState *s = opaque;

    s->rkp_head_store[off / 4] = val;
}

static const MemoryRegionOps mt6895_rkp_head_ops = {
    .read = mt6895_rkp_head_read,
    .write = mt6895_rkp_head_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static uint64_t mt6895_dev_type_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    /*
     * The preloader's platform code gates both the device-object builder
     * (0x0200C47C) and the block-device driver (0x0200C8B0) on this word
     * being 5, and the word sits in SRAM0 BSS that the payload clears at
     * start-up. Stand in for the BootROM and report UFS.
     */
    if (off / 4 < 1 && s->dev_type_store[off / 4]) {
        return extract64(s->dev_type_store[off / 4], 0, size * 8);
    }
    return extract64(MT6895_DEV_TYPE_VALUE, 0, size * 8);
}

static void mt6895_dev_type_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off + size <= MT6895_DEV_TYPE_SIZE) {
        s->dev_type_store[off / 4] = val;
    }
}

static const MemoryRegionOps mt6895_dev_type_ops = {
    .read = mt6895_dev_type_read,
    .write = mt6895_dev_type_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/*
 * The platform device table the preloader's storage layer consumes
 * (see MT6895_DEV_TBL_BASE). The table location is inside the payload's
 * BSS-clear range, so reads are served from this overlay rather than from
 * RAM; the structures it points at are written to MT6895_DEV_OBJ_BASE by
 * mt6895_place_device_table, which survives because it sits past the image.
 */
static const uint32_t mt6895_dev_tbl_default[] = {
    [0x00 / 4] = 10,                          /* shu step count (array capacity at +0x1c is 10) */
    [0x08 / 4] = MT6895_DEV_OBJ_BASE,         /* Q */
    [0x1c / 4] = MT6895_DEV_OBJ_BASE + 0x20,  /* P0 */
    [0x20 / 4] = MT6895_DEV_OBJ_BASE + 0x24,  /* P1 */
    [0x24 / 4] = MT6895_DEV_OBJ_BASE + 0x28,  /* P2 */
};

static uint64_t mt6895_dev_tbl_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off / 4 < ARRAY_SIZE(s->dev_tbl_store) && s->dev_tbl_store[off / 4]) {
        return extract64(s->dev_tbl_store[off / 4], 0, size * 8);
    }
    if (off / 4 < ARRAY_SIZE(mt6895_dev_tbl_default) &&
        mt6895_dev_tbl_default[off / 4]) {
        return extract64(mt6895_dev_tbl_default[off / 4], 0, size * 8);
    }
    return 0;
}

static void mt6895_dev_tbl_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off + size <= MT6895_DEV_TBL_SIZE) {
        s->dev_tbl_store[off / 4] = val;
    }
}

static const MemoryRegionOps mt6895_dev_tbl_ops = {
    .read = mt6895_dev_tbl_read,
    .write = mt6895_dev_tbl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/*
 * fp platform table (see MT6895_PLAT_TBL_BASE). Defaults supply the object
 * fields the storage layer needs; guest writes win where present.
 *
 * The entries that become obj[+0x24..+0x30] are pointers the storage
 * layer dereferences directly: the msdc wait loop reads [obj[+0x28] + 8]
 * and [obj[+0x28] + 0xC] -- the latter is MSDC_INT (0x00c), which the MTK
 * MSDC model drives -- so these have to be the MSDC controller base, not
 * crafted RAM.
 */
static const uint32_t mt6895_plat_tbl_default[] = {
    [0x28 / 4] = MT6895_MSDC_BASE,
    [0x2c / 4] = MT6895_MSDC_BASE,
    [0x30 / 4] = MT6895_MSDC_BASE,
    [0x34 / 4] = MT6895_MSDC_BASE,   /* -> obj[+0x28], polled by msdc */
    [0x48 / 4] = MT6895_MSDC_BASE,
    [0x4c / 4] = MT6895_MSDC_BASE,
    [0x50 / 4] = MT6895_MSDC_BASE,
    [0x54 / 4] = MT6895_MSDC_BASE,
};

static uint64_t mt6895_plat_tbl_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off / 4 < ARRAY_SIZE(s->plat_tbl_store) && s->plat_tbl_store[off / 4]) {
        return extract64(s->plat_tbl_store[off / 4], 0, size * 8);
    }
    if (off / 4 < ARRAY_SIZE(mt6895_plat_tbl_default) &&
        mt6895_plat_tbl_default[off / 4]) {
        return extract64(mt6895_plat_tbl_default[off / 4], 0, size * 8);
    }
    return 0;
}

static void mt6895_plat_tbl_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off + size <= MT6895_PLAT_TBL_SIZE) {
        s->plat_tbl_store[off / 4] = val;
    }
}

static const MemoryRegionOps mt6895_plat_tbl_ops = {
    .read = mt6895_plat_tbl_read,
    .write = mt6895_plat_tbl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/*
 * Device queue tables (see MT6895_DEVQ_BASE). Fill entry 1 of each table
 * with the fields the transaction path reads; guest writes win.
 */
static const struct { uint32_t off; uint32_t val; } mt6895_devq_default[] = {
    /* devA[1] = base + 0x88 */
    { 0x88 + 0x00, 0x00000037 },                 /* device id */
    { 0x88 + 0x04, 0x00000400 },                 /* transfer size */
    { 0x88 + 0x08, 0x00000008 },                 /* slot count */
    { 0x88 + 0x20, 0x00000001 },                 /* retry counter */
    { 0x88 + 0x28, MT6895_MSDC_BASE },           /* controller base */
    /* devB[1] = base + 0x3F4 */
    { 0x3f4 + 0x00, 0x00000037 },                /* device id */
    { 0x3f4 + 0x04, 0x00000400 },                /* transfer size */
    { 0x3f4 + 0x08, 0x00000008 },                /* slot count */
    { 0x3f4 + 0x20, 0x00000001 },
    { 0x3f4 + 0x28, MT6895_MSDC_BASE },
};

static uint64_t mt6895_devq_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off / 4 < ARRAY_SIZE(s->devq_store) && s->devq_store[off / 4]) {
        return extract64(s->devq_store[off / 4], 0, size * 8);
    }
    if (off + size <= MT6895_DEVQ_SIZE) {
        for (size_t i = 0; i < ARRAY_SIZE(mt6895_devq_default); i++) {
            if (mt6895_devq_default[i].off == off &&
                size == 4) {
                return mt6895_devq_default[i].val;
            }
        }
        /* a read that straddles a default field is served field-wise */
        for (size_t i = 0; i < ARRAY_SIZE(mt6895_devq_default); i++) {
            if (mt6895_devq_default[i].off >= off &&
                mt6895_devq_default[i].off + 4 <= off + size) {
                return mt6895_devq_default[i].val;
            }
        }
    }
    return 0;
}

static void mt6895_devq_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off + size <= MT6895_DEVQ_SIZE) {
        s->devq_store[off / 4] = val;
    }
}

static const MemoryRegionOps mt6895_devq_ops = {
    .read = mt6895_devq_read,
    .write = mt6895_devq_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/*
 * Boot-reason globals (see MT6895_BOOT_REASON_BASE): report a PMIC-style
 * power-on reason so the dispatcher selects NORMAL_BOOT.
 */
static uint64_t mt6895_boot_reason_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off / 4 < ARRAY_SIZE(s->boot_reason_store) &&
        s->boot_reason_store[off / 4]) {
        return extract64(s->boot_reason_store[off / 4], 0, size * 8);
    }
    switch (off) {
    case 0x00:                  /* flag byte -> 0x2043890 */
        return 1;
    case 0x04: {                /* reason word -> 0x204389C */
        /*
         * MTK boot reasons: NORMAL_BOOT is what a power-on reset reports and
         * is what the DRAM stage keys its "cold boot, always return 0" path
         * off.  MTK_BOOT_REASON sweeps the encoding so a watchdog reboot can
         * be presented without rebuilding.
         */
        static int reason = -1;

        if (reason < 0) {
            const char *e = getenv("MTK_BOOT_REASON");

            reason = e ? atoi(e) : 1;
        }
        return (uint32_t)reason;
    }
    default:
        return 0;
    }
}

static void mt6895_boot_reason_write(void *opaque, hwaddr off, uint64_t val,
                                     unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off + size <= MT6895_BOOT_REASON_SIZE) {
        s->boot_reason_store[off / 4] = val;
    }
}

static const MemoryRegionOps mt6895_boot_reason_ops = {
    .read = mt6895_boot_reason_read,
    .write = mt6895_boot_reason_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/*
 * ATF's early platform setup takes an "atfarg" structure in x0.  BL2 passes
 * the tee-secmem base there (normally filled by GZ, which this model does not
 * run), so the structure is all zeroes and ATF logs "atfarg is NULL!" and
 * skips its debug buffer and SiP service setup.  Give it the shape the real
 * firmware provides: atfarg+0x18 points at a 0x58-byte descriptor whose +0x10
 * is the log buffer.
 */
/*
 * BL2 clears the tee-secmem area (0x7c800000) while setting up TEE, wiping the
 * atfarg structure that ATF reads at entry.  On real hardware GZ has already
 * published the descriptor by the time BL2 runs, so re-apply the two pointers
 * whenever the firmware writes zero over them.
 */
/*
 * Model-side copy of ATF's atfarg blob. ATF clears its BSS with DC ZVA, which
 * writes the backing RAM directly and bypasses memory regions, so the blob has
 * to live in the model rather than in guest memory.
 */
/* Structure BL2 hands over in x0: +0x00 is a pointer, +0x18 the descriptor. */

/*
 * The ATF caches the mblock table pointer (0x482596a8).  The table lives in
 * the atfarg block this machine publishes; expose the (enlarged) block address
 * so every mblock entry stays inside the window.
 */
/*
 * GTA map descriptor the ATF reads when it maps the platform page-table area
 * (0x4821354c): base at 0x4825ac84, size at 0x48245840 and an "enabled" flag
 * at 0x48245844.  They normally come from the BL2 bl_params list; this
 * machine publishes the same descriptor directly so the table area gets
 * mapped writable before mtk_iommu_init_secure_pt_mem() memsets it.
 */

/*
 * GTA map descriptor window: reads publish the descriptor values, writes are
 * forwarded to the underlying DRAM so the ATF's BSS clear still lands there.
 */
;

;

/*
 * ADSP loader ack.
 *
 * ATF's mt_adsp_load_image writes the firmware load address to 0x7a000380 and
 * then polls 0x7a000384 for 1.  On hardware the HiFi3 DSP runs the firmware
 * and raises that ack; this model does not execute DSP firmware, so the ack
 * word alone is modelled -- it rises once a command has been posted, which
 * keeps the handshake meaningful.  Only these two words are intercepted; the
 * rest of 0x7a000000 stays real DRAM, because LK loads the ADSP image there
 * and later verifies it byte-for-byte.
 */
#define MT6895_ADSP_ACK_ADDR  0x7a000384ULL
#define MT6895_ADSP_CMD_ADDR  0x7a000380ULL

static bool mt6895_adsp_cmd_posted;

static uint64_t mt6895_adsp_ack_read(void *opaque, hwaddr off, unsigned size)
{
    (void)opaque; (void)off; (void)size;
    return mt6895_adsp_cmd_posted ? 1u : 0u;
}

static void mt6895_adsp_ack_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    (void)opaque; (void)off; (void)val; (void)size;
}

static const MemoryRegionOps mt6895_adsp_ack_ops = {
    .read = mt6895_adsp_ack_read,
    .write = mt6895_adsp_ack_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static void mt6895_adsp_cmd_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *ddr = memory_region_get_ram_ptr(s->ddr);

    (void)off; (void)size;
    ddr[MT6895_ADSP_CMD_ADDR - MT6895_DRAM_BASE] = (uint8_t)val;
    if (val) {
        mt6895_adsp_cmd_posted = true;
    }
}

static uint64_t mt6895_adsp_cmd_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *ddr = memory_region_get_ram_ptr(s->ddr);

    (void)off; (void)size;
    return ddr[MT6895_ADSP_CMD_ADDR - MT6895_DRAM_BASE];
}

static const MemoryRegionOps mt6895_adsp_cmd_ops = {
    .read = mt6895_adsp_cmd_read,
    .write = mt6895_adsp_cmd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * DIAGNOSTIC (MTK_SPY): catch-all behind everything else, so accesses that
 * fall through the map get logged instead of vanishing.  Used to locate where
 * LK's ADSP staging copies actually land.
 */
static uint64_t mt6895_spy_read(void *opaque, hwaddr off, unsigned size)
{
    static int nrd;

    (void)opaque; (void)size;
    if (nrd < 60) {
        nrd++;
        fprintf(stderr, "SPY-R addr=%#" HWADDR_PRIx " size=%u\n",
                (hwaddr)off, size);
    }
    return 0;
}

static void mt6895_spy_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    static int nwr;

    (void)opaque; (void)size;
    if (nwr < 60) {
        nwr++;
        fprintf(stderr, "SPY-W addr=%#" HWADDR_PRIx " val=%#" PRIx64
                " size=%u\n", (hwaddr)off, val, size);
    }
}

static const MemoryRegionOps mt6895_spy_ops = {
    .read = mt6895_spy_read,
    .write = mt6895_spy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/*
 * Core0 exposes two windows to the AP: 0x1e020000 carries the DRAM half of the
 * DSP image, 0x1e050000 the IRAM half.  Both mirror the staging copy LK built
 * at its own VAs (0x...04429000 and 0x...04420000 respectively).
 */
static const struct {
    hwaddr base;
    uint64_t stage_va;
} mt6895_adsp_windows[] = {
    /* core0: DRAM half at 0x1e020000, IRAM half at 0x1e050000 */
    { 0x1e020000, 0xffff000004429000ULL },
    { 0x1e050000, 0xffff000004420000ULL },
    /* core1: DRAM half at 0x1e090000, IRAM half at 0x1e0c0000 */
    { 0x1e090000, 0xffff00000443a000ULL },
    { 0x1e0c0000, 0xffff000004431000ULL },
};

typedef struct Mt6895AdspWin {
    uint64_t stage_va;
} Mt6895AdspWin;

static Mt6895AdspWin mt6895_adsp_win_ctx[ARRAY_SIZE(mt6895_adsp_windows)];

static uint64_t mt6895_adsp_sram_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895AdspWin *w = opaque;
    uint8_t buf[8] = { 0 };
    CPUState *cs = qemu_get_cpu(0);
    uint64_t v = 0;
    unsigned i;

    if (cs && size <= sizeof(buf)) {
        cpu_memory_rw_debug(cs, w->stage_va + off, buf, size, false);
        {
            static int nrd;

            if (nrd < 400) {
                nrd++;
                if ((off & 0xfff) == 0 || nrd < 20) {
                    fprintf(stderr, "ADSP-WIN va=%#" PRIx64
                            " off=%#" HWADDR_PRIx " v0=%02x%02x%02x%02x"
                            "\n", w->stage_va, off,
                            buf[0], buf[1], buf[2], buf[3]);
                }
            }
        }
    }
    for (i = 0; i < size; i++) {
        v |= (uint64_t)buf[i] << (8 * i);
    }
    return v;
}

/*
 * Writes are accepted and dropped: this window is the read-only SRAM view the
 * ack/verify path inspects after the loader has fetched the staged image.  The
 * staging copy stays authoritative, so a write here must never disturb it.
 */
static void mt6895_adsp_sram_write(void *opaque, hwaddr off, uint64_t val,
                                   unsigned size)
{
    (void)opaque; (void)off; (void)val; (void)size;
}

static const MemoryRegionOps mt6895_adsp_sram_ops = {
    .read = mt6895_adsp_sram_read,
    .write = mt6895_adsp_sram_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

/*
 * MFG0 / GPUEB power-status word (0x1c1f3c).
 *
 * LK's gpueb loader refuses to touch the partition until this word reports the
 * GPU domains as powered (bit 1 = MFG0 on); on hardware the preloader has
 * already brought those domains up by the time LK runs, which is exactly what
 * the other SPM power-status windows in this model report.  Mirror that
 * convention: the domains that the boot flow has powered read back as on.
 */
;

/*
 * GPUEB (RV33 executor) boot mailbox, 0x13c5fd24.
 *
 * LK stages the gpueb_a firmware, writes 3 to +0x8dc to start the core, then
 * polls +0x00 until the firmware reports 0x55667788 ("booted").  The RV33 is
 * not emulated, so model the handshake the way the firmware drives it: the
 * magic appears only after the start command was actually posted, which keeps
 * the protocol meaningful instead of hard-coding a ready status.
 */
#define MT6895_GPUEB_MBOX_BASE  0x13c5fd24ULL
#define MT6895_GPUEB_MBOX_SIZE  0x1000ULL
#define MT6895_GPUEB_BOOT_MAGIC 0x55667788u
#define MT6895_GPUEB_START_OFS  0x8dc

static uint32_t mt6895_gpueb_mbox[MT6895_GPUEB_MBOX_SIZE / 4];
static bool mt6895_gpueb_started;

static uint64_t mt6895_gpueb_mbox_read(void *opaque, hwaddr off, unsigned size)
{
    (void)opaque; (void)size;
    if (off == 0 && mt6895_gpueb_started) {
        return MT6895_GPUEB_BOOT_MAGIC;
    }
    return mt6895_gpueb_mbox[(off >> 2) & (MT6895_GPUEB_MBOX_SIZE / 4 - 1)];
}

static void mt6895_gpueb_mbox_write(void *opaque, hwaddr off, uint64_t val,
                                    unsigned size)
{
    (void)opaque; (void)size;
    mt6895_gpueb_mbox[(off >> 2) & (MT6895_GPUEB_MBOX_SIZE / 4 - 1)] =
        (uint32_t)val;
    if (off == MT6895_GPUEB_START_OFS && val) {
        /* The RV33 would fetch the staged image and then report booted. */
        mt6895_gpueb_started = true;
    }
}

static const MemoryRegionOps mt6895_gpueb_mbox_ops = {
    .read = mt6895_gpueb_mbox_read,
    .write = mt6895_gpueb_mbox_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/*
 * SPM power-down/power-up request window (0x1c001c40).
 *
 * LK's clkmgr MTCMOS flow drives this block for every MTCMOS zone:
 *
 *   [+0x00]  status   : bit 28 = request acknowledged
 *   [-0x08]  request  : writing a zone mask starts the flow
 *   [+0x08]  handshake: 0x200 rail-ready token
 *   [+0x10]  rail     : bit 9 = rails settled
 *
 * The SPM acknowledges a request it was actually given, which is the same
 * request/ack shape the other power blocks in this model use, so a flow that
 * never drove a request still reads as not-acknowledged.
 */
#define MT6895_SPM_REQ_BASE  0x1c001c40ULL
#define MT6895_SPM_REQ_SIZE  0x40ULL

static uint32_t mt6895_spm_req[MT6895_SPM_REQ_SIZE / 4];
static bool mt6895_spm_req_pending;

static uint64_t mt6895_spm_req_read(void *opaque, hwaddr off, unsigned size)
{
    uint32_t v;

    (void)opaque; (void)size;
    if (MTK_ENV_CACHED("MTK_SPM0")) {
        static int n;

        if (n < 24) {
            n++;
            fprintf(stderr, "SPM0-R off=%#lx\n", (unsigned long)off);
        }
    }
    v = mt6895_spm_req[(off >> 2) & (MT6895_SPM_REQ_SIZE / 4 - 1)];
    /* 0x1c001c4c: bit 28 = request acknowledged. */
    if (off == 0x0c && mt6895_spm_req_pending) {
        v |= 1u << 28;
    }
    /* 0x1c001c5c: bit 9 = rails settled (same step-0 handshake register). */
    if (off == 0x1c) {
        v |= 1u << 9;
    }
    return v;
}

static void mt6895_spm_req_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    (void)opaque; (void)size;
    if (MTK_ENV_CACHED("MTK_SPM0")) {
        static int nw;

        if (nw < 24) {
            nw++;
            fprintf(stderr, "SPM0-W off=%#lx val=%#lx\n",
                    (unsigned long)off, (unsigned long)val);
        }
    }
    mt6895_spm_req[(off >> 2) & (MT6895_SPM_REQ_SIZE / 4 - 1)] =
        (uint32_t)val;
    if ((off == 0x04 || off == 0x38) && val) {
        /* Zone-mask write at the request word starts the SPM flow. */
        mt6895_spm_req_pending = true;
    }
}

static const MemoryRegionOps mt6895_spm_req_ops = {
    .read = mt6895_spm_req_read,
    .write = mt6895_spm_req_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/*
 * SPM MTCMOS power-down request/acknowledge window (0x10001c30).
 *
 * LK's clkmgr pdn flow drives the zone mask and then spins until the
 * acknowledge word reports it back:
 *
 *   0x10001c34  request   (write zone mask)
 *   0x10001c3c  acknowledge
 *
 * The SPM acknowledges exactly the mask it was handed, so keep the pair
 * together instead of reporting a constant: a zone that was never requested
 * still reads as not-acknowledged and the flow only completes for the zones
 * it actually drove.
 */
#define MT6895_PDN_BASE   0x10001c30ULL
#define MT6895_PDN_SIZE   0x40ULL
#define MT6895_PDN_REQ    0x04        /* 0x10001c34 */
#define MT6895_PDN_ACK    0x0c        /* 0x10001c3c */

static uint32_t mt6895_pdn_regs[MT6895_PDN_SIZE / 4];

static uint64_t mt6895_pdn_read(void *opaque, hwaddr off, unsigned size)
{
    uint32_t v;

    (void)opaque; (void)size;
    v = mt6895_pdn_regs[(off >> 2) & (MT6895_PDN_SIZE / 4 - 1)];
    if (off == MT6895_PDN_ACK) {
        /* Acknowledge the zone mask the flow actually requested. */
        v |= mt6895_pdn_regs[MT6895_PDN_REQ / 4];
    } else if (!MTK_ENV_CACHED("MTK_NO_PDNACK") && (off & 0xc) == 0xc) {
        /*
         * The window is a run of infracfg_ao bus-protect groups, 16 bytes
         * each: STA0, SET, CLR, STA1.  LK's md1 pdn flow
         * (spm_mtcmos_ctrl_md1_disable) writes SET at 0x10001c54 / 0x44 /
         * 0x64 and spins on the matching STA1 at 0x5c / 0x4c / 0x6c, so give
         * every group the same acknowledge as the 0x3c pair above.  This
         * window also shadows the 0x10001c5c "rail ready" region, which is
         * why step 0 of that flow used to hang.
         */
        v |= mt6895_pdn_regs[(off - 8) >> 2];
    }
    return v;
}

static void mt6895_pdn_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    (void)opaque; (void)size;
    mt6895_pdn_regs[(off >> 2) & (MT6895_PDN_SIZE / 4 - 1)] = (uint32_t)val;
}

static const MemoryRegionOps mt6895_pdn_ops = {
    .read = mt6895_pdn_read,
    .write = mt6895_pdn_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

;

static uint64_t mt6895_atfctx_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint64_t val = s->atfctx_val ? s->atfctx_val : 0x4824a900ULL;

    if (size == 8 && off == 0) {
        return val;
    }
    return (off & 4) ? (val >> 32) : (val & 0xffffffffULL);
}

static void mt6895_atfctx_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint64_t cur = s->atfctx_val ? s->atfctx_val : 0x4824a900ULL;

    if (size == 8 && off == 0) {
        s->atfctx_val = val;
    } else if (size == 4) {
        if (off == 0) {
            s->atfctx_val = (cur & ~0xffffffffULL) | (val & 0xffffffffULL);
        } else if (off == 4) {
            s->atfctx_val = (cur & 0xffffffffULL) | (val << 32);
        }
    } else {
        s->atfctx_val = val;
    }
    uint8_t *p = mt6895_guest_ptr(s, 0x4825a110ULL + off, size);
    if (p) {
        for (unsigned i = 0; i < size; i++) {
            p[i] = (val >> (i * 8)) & 0xff;
        }
    }
    if (MTK_ENV_CACHED("MTK_WDTTRACE") || MTK_ENV_CACHED("MTK_RGUTRACE")) {
        fprintf(stderr, "mt6895: ATF rgu_impl (0x4825a110+%u) write val=%#" PRIx64
                " -> ctx=%#" PRIx64 "\n",
                (unsigned)off, val, s->atfctx_val);
    }
}

static const MemoryRegionOps mt6895_atfctx_ops = {
    .read = mt6895_atfctx_read,
    .write = mt6895_atfctx_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

;

/*
 * Write watch on ATF's SIP-dispatch mode flag (0x4825abd8). When it reads as 1
 * the dispatcher routes the crypto SiP call to a real handler; with 0 it falls
 * through to the "unhandled" path. Logging writes shows whether BL31's init
 * ever sets it.
 */
static uint64_t mt6895_atfflag_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, 0x4825abd8ULL + off, size);
    uint64_t v = 0;
    unsigned i;

    {
        static int n;

        if (n < 10) {
            n++;
            fprintf(stderr, "MTK-ATFFLAG-RD off=%#x size=%u\n",
                    (unsigned)off, size);
        }
    }
    /*
     * The SiP dispatch mode selects the 64-bit MTK SiP ID table. A real
     * device running a 64-bit BL33 (LK v2.0) has it set -- LK issues
     * 0xc2000xxx SiP calls and they are handled on hardware. The vendor
     * ATF sets it from platform configuration that this model does not
     * receive (no writer exists in the ATF image and the BL2 hand-off
     * arguments carry no such field), so report the real-device value.
     * MTK_DIAG_FLAG0 keeps the old behaviour for comparison.
     */
    if (!MTK_ENV_CACHED("MTK_DIAG_FLAG0")) {
        return 1;
    }
    if (!p) {
        return 0;
    }
    for (i = 0; i < size; i++) {
        v |= (uint64_t)p[i] << (8 * i);
    }
    return v;
}

static void mt6895_atfflag_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, 0x4825abd8ULL + off, size);
    static int n;
    unsigned i;

    if (n < 12) {
        n++;
        fprintf(stderr, "MTK-ATFFLAG-WR off=%#x val=%#" PRIx64 " size=%u pc=%#"
                PRIx64 "\n", (unsigned)off, val, size,
                current_cpu ? (uint64_t)ARM_CPU(current_cpu)->env.regs[15] : 0);
    }
    if (!p) {
        return;
    }
    for (i = 0; i < size; i++) {
        p[i] = (val >> (8 * i)) & 0xff;
    }
}

/*
 * DIAGNOSTIC (MTK_BL33_64BIT): BL31 picks the next image's SPSR from bit 4 of
 * the byte at 0x4825b823 (0x48227b38: w1 = 1 - bit4; w1==1 -> SPSR 0x1d3
 * (AArch32 SVC), w1==0 -> 0xc5 (AArch64 EL1h)). Force the bit so the BL33
 * hand-off is AArch64 and the SiP dispatcher selects the 64-bit ID table.
 */
static uint64_t mt6895_bl33flag_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, 0x4825b823ULL + off, size);
    uint64_t v = 0;
    unsigned i;

    for (i = 0; p && i < size; i++) {
        v |= (uint64_t)p[i] << (8 * i);
    }
    if (off == 0 && size >= 1) {
        v |= 0x10;
    }
    return v;
}

static void mt6895_bl33flag_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, 0x4825b823ULL + off, size);
    unsigned i;

    if (!p) {
        return;
    }
    for (i = 0; i < size; i++) {
        p[i] = (val >> (8 * i)) & 0xff;
    }
}

static const MemoryRegionOps mt6895_bl33flag_ops = {
    .read = mt6895_bl33flag_read,
    .write = mt6895_bl33flag_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static const MemoryRegionOps mt6895_atfflag_ops = {
    .read = mt6895_atfflag_read,
    .write = mt6895_atfflag_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
    .impl = { .min_access_size = 1, .max_access_size = 8 },
};

/* DIAGNOSTIC (temporary): force the ATF SIP-handler precondition global to
 * read non-zero, to check whether that makes BL31's cold init complete. */
static uint64_t mt6895_diagg_read(void *opaque, hwaddr off, unsigned size)
{
    (void)opaque; (void)off; (void)size;
    return 1;
}
static void mt6895_diagg_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    (void)opaque; (void)off; (void)val; (void)size;
}
static const MemoryRegionOps mt6895_diagg_ops = {
    .read = mt6895_diagg_read, .write = mt6895_diagg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * The DRAM part the machine presents follows the configured RAM size:
 *
 *  - 6 GiB: the genuine Xiaomi xaga part, one 6 GiB LPDDR5X rank.  The
 *    unit's own boot log reports dram_type 0xC, mr5 0xFF and rank0_size
 *    0x180000000, and this is the machine default.
 *  - any other size: the historical 8 GiB development configuration (QVL
 *    row 7, one 8 GiB LPDDR5 rank).  Runs that still pass -m 8G therefore
 *    keep exactly the DRAM behaviour they had before the LPDDR5X fix.
 *
 * Deriving the part from the configured RAM keeps the guest from ever being
 * told about a memory map that does not describe the part the DRAM
 * controller reports.
 */
#define MT6895_DRAM_SIZE        0x180000000ULL   /* 6 GiB, genuine part */

static bool mt6895_dram_is_6g(Mt6895MachineState *s)
{
    return MACHINE(s)->ram_size == MT6895_DRAM_SIZE;
}

/*
 * Mode-register answers for the configured part.
 *
 * MR5 is the manufacturer/die identity check_qvl compares against its QVL
 * table; MR8 bits [5:2] select a per-die density from the preloader's
 * density switch at 0x02027014 and bits [7:6] the die count.  The product
 * is scaled by the channel count (4) to give the rank size:
 *
 *   6 GiB LPDDR5X: MR5 0xff, MR8 0x14 -> 0x60000000 * 1 die * 4
 *   = 0x180000000, QVL row 5 (type C, rank0 6 GiB, rank1 0);
 *   8 GiB LPDDR5:  MR5 0x06, MR8 0x18 -> 0x80000000 * 1 die * 4
 *   = 0x200000000, QVL row 7 (type B, rank0 8 GiB, rank1 0).
 */
static uint32_t mt6895_dram_mr5(Mt6895MachineState *s)
{
    return mt6895_dram_is_6g(s) ? 0x00ff : 0x0006;
}

static uint32_t mt6895_dram_mr8(Mt6895MachineState *s)
{
    return mt6895_dram_is_6g(s) ? 0x0014 : 0x0018;
}

static void mt6895_place_device_table(Mt6895MachineState *s)
{
    /*
     * Targets for the platform device table: the Q structure is read for a
     * few identification bytes; the P words carry the ready flag whose bit
     * 15 the storage layer polls at 0x0203919E.
     */
    uint8_t area[0x60] = { 0 };
    uint32_t ready = 0x00008000;

    stl_le_p(area + 0x20, ready);
    stl_le_p(area + 0x24, ready);
    stl_le_p(area + 0x28, ready);
    /* the sub-object the storage layer polls: [sub + 8] bit 15 */
    stl_le_p(area + 0x10 + 8, ready);

    address_space_write(&address_space_memory, MT6895_DEV_OBJ_BASE,
                        MEMTXATTRS_UNSPECIFIED, area, sizeof area);
}

/*
 * The preloader's boot-partition name table (SRAM0 0x0011D800) is built at
 * boot time by 0x2040BF8 from three {code, offset} shells; the name
 * strings themselves are normally patched into the image by the factory
 * flasher. The dumped LUA_1 has plain code bytes in those slots, so every
 * lookup through 0x2040C7C returns a non-string and
 * mt_get_part_info_by_name fails ("[GPT_PL]mt_get_part_info_by_name fail")
 * right before the LK load. Reproduce the flasher's patch; the slots sit
 * in the SRAM0 copy (execution runs from SRAM1 0x02000F10), outside every
 * BSS-clear range, so the writes survive.
 */
static void mt6895_place_boot_part_names(Mt6895MachineState *s)
{
    static const struct {
        uint64_t addr;
        const char *name;
    } names[] = {
        { 0x0011d80c, "lk_a"   },   /* code 0xD8A3 */
        { 0x0011d880, "boot_a" },   /* code 0xE31C */
        { 0x0011d8ac, "dtbo_a" },   /* code 0xDB45 */
    };

    for (size_t i = 0; i < ARRAY_SIZE(names); i++) {
        address_space_write(&address_space_memory, names[i].addr,
                            MEMTXATTRS_UNSPECIFIED, names[i].name,
                            strlen(names[i].name) + 1);
        info_report("mt6895: boot part name @ %#" PRIx64 " = '%s'",
                    names[i].addr, names[i].name);
    }
}

static void mt6895_logstore_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    Mt6895MachineState *s = opaque;

    trace_mt6895_logstore_write(off, size, val);
    if (off + size <= MT6895_LOGSTORE_WIN_SIZE) {
        memcpy(memory_region_get_ram_ptr(&s->bootrom) +
               MT6895_LOGSTORE_WIN_BASE + off, &val, size);
    }
}

static const MemoryRegionOps mt6895_logstore_ops = {
    .read = mt6895_logstore_read,
    .write = mt6895_logstore_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static uint64_t mt6895_systimer_read(void *opaque, hwaddr off, unsigned size)
{
    /*
     * MediaTek System Timer (13 MHz = 13 ticks per microsecond).
     * 0x00: CNTCR (Control) - bit 0 = EN
     * 0x04: CNTSR (Status)
     * 0x08: CNTCV_L (Counter Value Low, 0x1C011008)
     * 0x0C: CNTCV_H (Counter Value High)
     */
    uint64_t now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    /*
     * 13 MHz means 13_000_000 ticks per second, i.e. 13 ticks per
     * microsecond -> ticks = ns * 13 / 1000.  The previous expression
     * (ns * 1300 / 1000) ran the counter 100x too fast, which made every
     * firmware-side duration measurement (the "[PROFILE] ... takes N ms"
     * and "[PART] load speed" lines) wildly inflated.
     */
    uint64_t ticks = (now_ns * 13) / 1000;

    switch (off) {
    case 0x00:
        return 1; /* Counter enabled */
    case 0x04:
        return 1;
    case 0x08:
        return (uint32_t)ticks;
    case 0x0c:
        return (uint32_t)(ticks >> 32);
    default:
        return 0;
    }
}

static void mt6895_systimer_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    /* Free-running counter */
}

static const MemoryRegionOps mt6895_systimer_ops = {
    .read = mt6895_systimer_read,
    .write = mt6895_systimer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static uint64_t mt6895_kp_read(void *opaque, hwaddr off, unsigned size)
{
    /* Keypad registers: active low (all keys released = 1) */
    return 0xffffffffULL;
}

static void mt6895_kp_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
}

static const MemoryRegionOps mt6895_kp_ops = {
    .read = mt6895_kp_read,
    .write = mt6895_kp_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static uint64_t mt6895_srclken_rc_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    /* Channels 0..13 status registers: 0x114 .. 0x148. */
    if (off >= 0x114 && off <= 0x148) {
        unsigned ch = (off - 0x114) / 4;
        /*
         * Channels 6..11 are switched to FPM mode (mask 0xfc0): expected ACK is 0.
         * Channels 0..5 and 12..13 are non-FPM: expected ACK is 2.
         */
        uint32_t ack = (ch >= 6 && ch <= 11) ? 0 : 2;
        return (s->srclken_rc_regs[off / 4] & ~0x2) | ack;
    }
    if (off < sizeof(s->srclken_rc_regs)) {
        return s->srclken_rc_regs[off / 4];
    }
    return 0;
}

static void mt6895_srclken_rc_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    Mt6895MachineState *s = opaque;
    if (off < sizeof(s->srclken_rc_regs)) {
        s->srclken_rc_regs[off / 4] = val;
    }
}

static const MemoryRegionOps mt6895_srclken_rc_ops = {
    .read = mt6895_srclken_rc_read,
    .write = mt6895_srclken_rc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

/*
 * dxcc_sec: the security engine's HRE (host ring engine) descriptor ring.
 *
 * The preloader submits work as a six-word descriptor at 0xe80..0xe94 and
 * gates every single submission on the ring-status register at 0xe9c:
 *
 *   0x0205314a:  ldr.w  r2, [r5, #0xe9c]
 *   0x0205314e:  lsls   r2, r2, #0x1c      ; isolate bits[3:0]
 *   0x02053150:  beq    #0x205314a         ; wait until (val & 0xf) != 0
 *
 * That low nibble is the ring's free-descriptor indicator: the guest may
 * write a descriptor while it is non-zero. Nothing here executes the
 * descriptors, so the engine consumes each submission immediately and the
 * ring always reports a free slot. The guest's writes to the upper bits are
 * kept, so the register still behaves as a register file everywhere else.
 *
 * After submitting, the driver waits for completion and reads the engine's
 * verdict (0x020533fe):
 *
 *   0x02053446:  bl     #0x2053230         ; submit descriptor
 *   0x0205344a:  bl     #0x2053496         ; returns [engine + 0xa00]
 *   0x02053450:  lsls   r0, r0, #0x1d
 *   0x02053452:  bpl    #0x205344a         ; until (val & 0x4) != 0  (done)
 *   0x02053454:  ldr.w  r0, [r4, #0xba0]
 *   0x0205345a:  beq    #0x2053454         ; until [0xba0] != 0
 *   0x0205345e:  bne    #0x205346e         ; [0xba0] == 1 means success
 *
 * So 0xa00 reports "descriptor done" (bit 2, the very bit the driver enables
 * by writing 0xa08 = 0x4) and 0xba0 reports the operation result, where 1 is
 * success. Since descriptors are consumed without being executed, both read
 * back as "finished, succeeded".
 *
 * LIMITATION: the engine performs no actual crypto. A caller that consumes
 * the descriptor's output buffer -- rather than only the verdict -- will see
 * whatever was already in memory.
 */
static uint64_t mt6895_dxcc_sec_read_impl(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    {
        static int n;

        if (MTK_ENV_CACHED("MTK_DXCCLOG") && n < 300) {
            n++;
            fprintf(stderr, "MTK-DXCC-RD off=%#" PRIx64 " pc=%#" PRIx64 "\n",
                    (uint64_t)off,
                    current_cpu ? (uint64_t)ARM_CPU(current_cpu)->env.regs[15] : 0);
        }
    }

    if (off + 4 > MT6895_DXCC_SEC_SIZE) {
        return 0;
    }

    switch (off) {
    case MT6895_DXCC_SEC_RING_STA:
        return (s->dxcc_sec_regs[off / 4] & ~0xfU) | 0x1;
    case 0xb0:
    case 0xb4:
        /*
         * HRE command interface: these two words report "engine ready to
         * accept a command".  The submit path waits for bit 0 on them, then
         * writes the command to +0xd4 and its payload to +0xd8.  The stub
         * consumes each submission immediately (nothing executes), so the
         * interface is always ready -- the same convention the ring-status
         * register at +0xe9c follows.
         */
        return (s->dxcc_sec_regs[off / 4] & ~0x1U) | 0x1;
    case MT6895_DXCC_SEC_INT_STA:
        return (s->dxcc_sec_regs[off / 4] & ~0x4U) | 0x4;
    case MT6895_DXCC_SEC_RESULT:
        return 1;
    case MT6895_DXCC_SEC_GZ_STA:
        /* EXPERIMENT (2026-11-03): the GZ init path polls bit0 here
         * (0x02051C74: ldr.w r0,[r4,#0xabc]; lsls #31; beq). */
        return (s->dxcc_sec_regs[off / 4] & ~0x1U) | 0x1;
    case 0xad4:
        /* BL2_EXT's bl32_map_bl_params reads [dxcc_sec+0xad4] (via
         * 0x62f56e7c) and requires the low byte to be 1 or 5
         * (the caller in 0x62f50378 checks `w1|4 == 5`). Returning 1
         * reports the engine ready without taking the 0xaa4/0xab4
         * command path, which our stub cannot implement. 0xad4 is
         * the status byte the firmware polls; the preloader ignores it. */
        return 1;
    default:
        return s->dxcc_sec_regs[off / 4];
    }
}

/*
 * Compact SHA-256 used by the DXCC-SEC model.  The BL2_EXT image runs at EL3
 * and drives this engine directly through its register window, so the digest
 * has to be produced by the emulator: the firmware compares it against the
 * value stored in the secure-config partition and refuses to boot on mismatch.
 */
typedef struct MtkSha256Ctx {
    uint32_t h[8];
    uint64_t total;
    uint8_t buf[64];
    unsigned buflen;
} MtkSha256Ctx;

static const uint32_t mtk_sha_k[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

static inline uint32_t mtk_ror32(uint32_t x, unsigned n)
{
    return (x >> n) | (x << (32 - n));
}

static void mtk_sha_block(MtkSha256Ctx *c, const uint8_t *p)
{
    uint32_t w[64], a, b, cc, d, e, f, g, h;
    unsigned i;

    for (i = 0; i < 16; i++) {
        w[i] = ((uint32_t)p[i * 4] << 24) | ((uint32_t)p[i * 4 + 1] << 16) |
               ((uint32_t)p[i * 4 + 2] << 8) | (uint32_t)p[i * 4 + 3];
    }
    for (i = 16; i < 64; i++) {
        uint32_t s0 = mtk_ror32(w[i - 15], 7) ^ mtk_ror32(w[i - 15], 18) ^
                      (w[i - 15] >> 3);
        uint32_t s1 = mtk_ror32(w[i - 2], 17) ^ mtk_ror32(w[i - 2], 19) ^
                      (w[i - 2] >> 10);

        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    a = c->h[0]; b = c->h[1]; cc = c->h[2]; d = c->h[3];
    e = c->h[4]; f = c->h[5]; g = c->h[6]; h = c->h[7];
    for (i = 0; i < 64; i++) {
        uint32_t S1 = mtk_ror32(e, 6) ^ mtk_ror32(e, 11) ^ mtk_ror32(e, 25);
        uint32_t ch = (e & f) ^ (~e & g);
        uint32_t t1 = h + S1 + ch + mtk_sha_k[i] + w[i];
        uint32_t S0 = mtk_ror32(a, 2) ^ mtk_ror32(a, 13) ^ mtk_ror32(a, 22);
        uint32_t maj = (a & b) ^ (a & cc) ^ (b & cc);
        uint32_t t2 = S0 + maj;

        h = g; g = f; f = e; e = d + t1;
        d = cc; cc = b; b = a; a = t1 + t2;
    }
    c->h[0] += a; c->h[1] += b; c->h[2] += cc; c->h[3] += d;
    c->h[4] += e; c->h[5] += f; c->h[6] += g; c->h[7] += h;
}

static void mtk_sha_init(MtkSha256Ctx *c)
{
    c->h[0] = 0x6a09e667; c->h[1] = 0xbb67ae85;
    c->h[2] = 0x3c6ef372; c->h[3] = 0xa54ff53a;
    c->h[4] = 0x510e527f; c->h[5] = 0x9b05688c;
    c->h[6] = 0x1f83d9ab; c->h[7] = 0x5be0cd19;
    c->total = 0;
    c->buflen = 0;
}

static void mtk_sha_update(MtkSha256Ctx *c, const uint8_t *p, size_t len)
{
    c->total += len;
    if (c->buflen) {
        size_t want = 64 - c->buflen;

        if (len < want) {
            memcpy(c->buf + c->buflen, p, len);
            c->buflen += len;
            return;
        }
        memcpy(c->buf + c->buflen, p, want);
        mtk_sha_block(c, c->buf);
        c->buflen = 0;
        p += want;
        len -= want;
    }
    while (len >= 64) {
        mtk_sha_block(c, p);
        p += 64;
        len -= 64;
    }
    if (len) {
        memcpy(c->buf, p, len);
        c->buflen = len;
    }
}

static void mtk_sha_final(MtkSha256Ctx *c, uint8_t out[32])
{
    uint64_t bits = c->total * 8;
    uint8_t pad[72];
    size_t padlen;
    unsigned i;

    pad[0] = 0x80;
    padlen = (c->buflen < 56) ? (56 - c->buflen) : (120 - c->buflen);
    memset(pad + 1, 0, padlen - 1);
    for (i = 0; i < 8; i++) {
        pad[padlen + i] = (uint8_t)(bits >> (56 - 8 * i));
    }
    mtk_sha_update(c, pad, padlen + 8);
    for (i = 0; i < 8; i++) {
        out[i * 4]     = (uint8_t)(c->h[i] >> 24);
        out[i * 4 + 1] = (uint8_t)(c->h[i] >> 16);
        out[i * 4 + 2] = (uint8_t)(c->h[i] >> 8);
        out[i * 4 + 3] = (uint8_t)(c->h[i]);
    }
}

/*
 * DXCC-SEC cipher engine: AES-128-CTR.
 *
 * Before this block the descriptor ring was consumed without being executed.
 * The GZ images the preloader loads (gz_a/gz_b) are AES-128-CTR scrambled, so
 * the preloader hands this engine the key and the initial counter through
 * setup descriptors and then streams each image through one DIN_AES_DOUT
 * descriptor.  The six descriptor words are:
 *
 *   +0xE80  DIN address low 32
 *   +0xE84  DIN_DMA_MODE[1:0] | DIN_SIZE[25:2] | NS_BIT[26] |
 *           DIN_CONST[27] | NOT_LAST[28] | LOCK_QUEUE[29]
 *   +0xE88  DOUT address low 32
 *   +0xE8C  DOUT_DMA_MODE[1:0] | DOUT_SIZE[25:2] | NS_BIT[26] |
 *           DOUT_LAST_IND[27] | HASH_XOR_BIT[29]
 *   +0xE90  DATA_FLOW_MODE[5:0] | AES_SEL_N_HASH[6] | ACK_NEEDED[9:8] |
 *           CIPHER_MODE[13:10] | CIPHER_DO[16:15] | KEY_SIZE[23:22] |
 *           SETUP_OPERATION[27:24] | endian/swap bits[31:28]
 *   +0xE94  DIN_ADDR_HIGH[15:0] | DOUT_ADDR_HIGH[31:16]
 *
 * Field layout from mtkclient's hwcrypto_dxcc.py (DSCRPTR field table), and
 * confirmed against our own preloader's submissions
 * (artifacts/r1302/dxcc_aes/RUN_AES.err):
 *
 *   setup=LOAD_STATE1(2), flow=S_DIN_to_AES(32), cipher=CTR(2):
 *       DIN = initial 128-bit counter, 0x020FFEAC, bytes A0B09246..DC93458A
 *   setup=LOAD_KEY0(4),   flow=S_DIN_to_AES(32), cipher=CTR(2):
 *       DIN = 16-byte key, 0x020FFEBC, bytes 5C0E349A..A378BD17
 *   flow=DIN_AES_DOUT(1):
 *       in-place stream (gz_a: 0x7F800000/0x1CB4A0, gz_b:
 *       0x7F400000/0x23D90)
 *
 * The engine keeps key and counter across descriptors; every LOAD_STATE1
 * reloads the counter.  CTR is a 128-bit big-endian counter incremented once
 * per 16-byte block, i.e. exactly mtkclient's
 * Counter.new(128, initial_value=bytes_to_long(iv)).
 */
typedef struct MtkDxccAesState {
    uint8_t key[32];
    unsigned key_len;
    uint8_t ctr[16];
    unsigned cipher_mode;
    bool key_valid;
    bool state_valid;
} MtkDxccAesState;

static MtkDxccAesState mtk_dxcc_aes_state;
static int mtk_dxcc_aes_logs;

/*
 * DXCC hash engine: the real hardware keeps one SHA-256 chaining state
 * across descriptor submissions (LOAD_STATE1 loads it, S_DIN_to_HASH feeds
 * it, DIN_HASH finalizes it), so the model has to keep the context too.
 */
typedef struct MtkDxccHashState {
    MtkSha256Ctx ctx;
    bool active;
} MtkDxccHashState;

static MtkDxccHashState mtk_dxcc_hash_state;

/* Optional one-shot dump of a decrypted GZ buffer (MTK_GZ_DUMP=<prefix>). */
static void mtk_dxcc_aes_dump(uint64_t addr, uint64_t len)
{
    const char *base = getenv("MTK_GZ_DUMP");
    char path[1024];
    uint8_t *buf;
    FILE *fp;

    if (!base || !base[0] || !len) {
        return;
    }
    snprintf(path, sizeof(path), "%s_%08" PRIx64 ".bin", base, addr);
    buf = g_malloc(len);
    if (address_space_read(&address_space_memory, addr,
                           MEMTXATTRS_UNSPECIFIED, buf, len) == MEMTX_OK) {
        fp = fopen(path, "wb");
        if (fp) {
            fwrite(buf, 1, len, fp);
            fclose(fp);
            fprintf(stderr, "MTK-DXCC-AES dump %s addr=%#" PRIx64
                    " len=%#" PRIx64 "\n", path, addr, len);
        }
    }
    g_free(buf);
}

static void mtk_dxcc_aes_ctr(uint64_t din, uint64_t dout, uint64_t len)
{
    MtkDxccAesState *st = &mtk_dxcc_aes_state;
    AES_KEY aeskey;
    uint8_t *buf;
    uint8_t ks[16];
    uint64_t off;
    unsigned i;

    if (!st->key_valid || !st->state_valid) {
        if (mtk_dxcc_aes_logs < 40) {
            mtk_dxcc_aes_logs++;
            fprintf(stderr, "MTK-DXCC-AES skip din=%#" PRIx64 " len=%#" PRIx64
                    " (key=%d state=%d)\n", din, len, st->key_valid,
                    st->state_valid);
        }
        return;
    }
    if (!len || len > 0x4000000ULL) {
        return;
    }
    buf = g_malloc(len);
    if (address_space_read(&address_space_memory, din,
                           MEMTXATTRS_UNSPECIFIED, buf, len) != MEMTX_OK) {
        g_free(buf);
        return;
    }
    AES_set_encrypt_key(st->key, (int)(st->key_len * 8), &aeskey);
    for (off = 0; off < len; off += 16) {
        unsigned n = (len - off) < 16 ? (unsigned)(len - off) : 16;

        AES_encrypt(st->ctr, ks, &aeskey);
        for (i = 0; i < n; i++) {
            buf[off + i] ^= ks[i];
        }
        for (i = 16; i-- > 0; ) {
            if (++st->ctr[i] != 0) {
                break;
            }
        }
    }
    if (dout &&
        address_space_write(&address_space_memory, dout,
                            MEMTXATTRS_UNSPECIFIED, buf, len) != MEMTX_OK) {
        fprintf(stderr, "MTK-DXCC-AES write failed dout=%#" PRIx64 "\n", dout);
    }
    if (dout >= 0x7F000000ULL && dout < 0x80000000ULL) {
        mtk_dxcc_aes_dump(dout, len);
    }
    if (MTK_ENV_CACHED("MTK_DXCC_AESLOG")) {
        unsigned bi;

        fprintf(stderr, "MTK-DXCC-AES ctr din=%#" PRIx64 " dout=%#" PRIx64
                " len=%#" PRIx64 " key_len=%u head=", din, dout, len,
                st->key_len);
        for (bi = 0; bi < 32 && bi < len; bi++) {
            fprintf(stderr, "%02x", buf[bi]);
        }
        fprintf(stderr, "\n");
    }
    g_free(buf);
}

/*
 * Execute one descriptor.  WORD5 (+0xE94) is written last by the driver's
 * sasi_sb_adddescsequence(), so a write there marks a complete submission.
 */
static void mtk_dxcc_aes_submit(void *opaque)
{
    Mt6895MachineState *s = opaque;
    uint32_t *r = s->dxcc_sec_regs;
    uint32_t w0 = r[0xe80 / 4];
    uint32_t w1 = r[0xe84 / 4];
    uint32_t w2 = r[0xe88 / 4];
    uint32_t w3 = r[0xe8c / 4];
    uint32_t w4 = r[0xe90 / 4];
    uint32_t w5 = r[0xe94 / 4];
    uint64_t din = w0 | ((uint64_t)(w5 & 0xffffU) << 32);
    uint64_t dout = w2 | ((uint64_t)((w5 >> 16) & 0xffffU) << 32);
    uint64_t din_size = (w1 >> 2) & 0xffffffU;
    uint64_t dout_size = (w3 >> 2) & 0xffffffU;
    unsigned flow = w4 & 0x3fU;
    unsigned cipher = (w4 >> 10) & 0xfU;
    unsigned setup = (w4 >> 24) & 0xfU;
    MtkDxccAesState *st = &mtk_dxcc_aes_state;

    if (MTK_ENV_CACHED("MTK_DXCC_AESLOG") && mtk_dxcc_aes_logs < 60) {
        mtk_dxcc_aes_logs++;
        fprintf(stderr, "MTK-DXCC-AES desc din=%#" PRIx64 " dinsz=%#" PRIx64
                " dout=%#" PRIx64 " doutsz=%#" PRIx64 " flow=%u cipher=%u"
                " setup=%u din_const=%u\n", din, din_size, dout, dout_size,
                flow, cipher, setup, (w1 >> 27) & 1U);
    }

    if (flow == 32) {           /* S_DIN_to_AES: load key or counter */
        st->cipher_mode = cipher;
        if (setup == 2 && din && din_size == 16) {          /* LOAD_STATE1 */
            if (address_space_read(&address_space_memory, din,
                                   MEMTXATTRS_UNSPECIFIED, st->ctr,
                                   sizeof(st->ctr)) == MEMTX_OK) {
                st->state_valid = true;
            }
        } else if (setup == 4 && din &&            /* LOAD_KEY0 */
                   (din_size == 16 || din_size == 32)) {
            if (address_space_read(&address_space_memory, din,
                                   MEMTXATTRS_UNSPECIFIED, st->key,
                                   (size_t)din_size) == MEMTX_OK) {
                st->key_len = (unsigned)din_size;
                st->key_valid = true;
            }
        } else if (setup == 4) {
            /*
             * No DIN: the engine would use an internal (hardware) key that
             * software cannot supply.  Do not pretend the previous key is
             * still selected.
             */
            st->key_valid = false;
        }
    } else if (flow == 1) {     /* DIN_AES_DOUT: stream data */
        /*
         * A data descriptor with no output buffer only updates the cipher's
         * internal state (the preloader uses those to derive keys); nothing
         * guest-visible is produced.
         */
        if (dout && dout_size && st->cipher_mode == 2) {   /* CTR */
            mtk_dxcc_aes_ctr(din, dout, din_size ? din_size : dout_size);
        }
    }
    /*
     * The engine completes synchronously: the transformation above has
     * already run by the time the guest reads the verdict at +0xBA0
     * (1 = success).  Nothing may report success before the data moved.
     */
    r[MT6895_DXCC_SEC_RESULT / 4] = 1;
}

static void mt6895_dxcc_sec_write_impl(void *opaque, hwaddr off, uint64_t val,
                                       unsigned size)
{
    Mt6895MachineState *s = opaque;
    {
        static int n;
        static int cap = -1;

        /*
         * The historical hard cap of 300 lines was consumed by the BL2_EXT
         * phase alone, hiding every LK/BL31-phase descriptor submission.  The
         * limit is now env-configurable (MTK_DXCC_WR_MAX); the default keeps
         * the old behaviour.
         */
        if (cap < 0) {
            const char *m = getenv("MTK_DXCC_WR_MAX");

            cap = m ? atoi(m) : 300;
        }
        if (n < cap) {
            n++;
            fprintf(stderr, "MTK-DXCC-WR off=%#" PRIx64 " val=%#" PRIx64
                    " pc=%#" PRIx64 "\n", (uint64_t)off, val,
                    current_cpu ? (uint64_t)ARM_CPU(current_cpu)->env.regs[15] : 0);
        }
    }
    if (off == 0xe90 && MTK_ENV_CACHED("MTK_DXCC_SUBMIT")) {
        /*
         * DIAGNOSTIC (MTK_DXCC_SUBMIT): one decoded line per descriptor
         * submission, whatever the data-flow mode, so the exact sequence the
         * firmware drives the engine with can be read straight from the run
         * log (src/len/dst/opcode/flow/setup).
         */
        static int nsub;
        uint32_t din_lo = s->dxcc_sec_regs[0xe80 / 4];
        uint32_t din_cfg = s->dxcc_sec_regs[0xe84 / 4];
        uint32_t dout_lo = s->dxcc_sec_regs[0xe88 / 4];
        uint32_t dout_cfg = s->dxcc_sec_regs[0xe8c / 4];
        uint32_t hi = s->dxcc_sec_regs[0xe94 / 4];
        uint64_t din = ((uint64_t)(hi & 0xffffu) << 32) | din_lo;
        uint64_t dout = ((uint64_t)(hi >> 16) << 32) | dout_lo;
        uint64_t din_sz = (din_cfg >> 2) & 0x00ffffffu;
        uint64_t dout_sz = (dout_cfg >> 2) & 0x00ffffffu;

        if (nsub < 4000) {
            nsub++;
            fprintf(stderr, "MTK-DXCC-SUBMIT op=%#" PRIx64 " flow=%u %s"
                    " setup=%u din=%#" PRIx64 " din_sz=%" PRIu64
                    " dout=%#" PRIx64 " dout_sz=%" PRIu64 "\n",
                    val, (unsigned)(val & 0x3fu),
                    (val & 0x40u) ? "AES" : "HASH",
                    (unsigned)((val >> 24) & 0xfu),
                    din, din_sz, dout, dout_sz);
        }
    }

    if (off + 4 > MT6895_DXCC_SEC_SIZE) {
        return;
    }

    s->dxcc_sec_regs[off / 4] = val;

    /*
     * DIAGNOSTIC (MTK_DXCC_HASH): when the driver submits a descriptor whose
     * opcode word (+0xe90) is the hashing operation, dump the input buffer so
     * the exact bytes the firmware authenticates can be compared against the
     * stored digest.  One-shot and env gated, so it costs nothing normally.
     */
    /*
     * Execute the hash descriptors the engine is handed.  The BL2_EXT image
     * runs at EL3 and drives this block directly, so the emulator is the only
     * thing that can produce a correct digest; a no-op engine makes every
     * secure-config hash comparison fail and the boot stops before LK.
     */
    if (off == 0xe90) {
        /*
         * Execute the hash descriptors the engine is handed.  The real DXCC
         * hash engine keeps one SHA-256 chaining state across descriptors:
         * LOAD_STATE1 (flow=37 setup=1) loads the chaining state, S_DIN_to_HASH
         * (flow=7) streams update data, and the DIN_HASH descriptor with an
         * empty DIN (flow=43 setup=8) finalizes into DOUT.  Both the BL2_EXT
         * image (EL3) and BL31's SiP SHA service drive it this way -- the
         * reference LK computes the vbmeta digest for libfs_avb through
         * exactly such an update/final sequence (r1314 RUN_SUBMIT.err).
         * The old executor only recognised the two one-shot opcodes and, for
         * an empty DIN, hashed the DOUT buffer itself, so every streaming
         * digest was deterministic garbage; self-comparisons still matched
         * (wrong vs. the same wrong) until libfs_avb checked one against a
         * real software SHA-256.
         */
        unsigned flow = (unsigned)(val & 0x3f);
        unsigned setup = (unsigned)((val >> 24) & 0xf);
        uint32_t din_lo = s->dxcc_sec_regs[0xe80 / 4];
        uint32_t din_cfg = s->dxcc_sec_regs[0xe84 / 4];
        uint32_t dout_lo = s->dxcc_sec_regs[0xe88 / 4];
        uint32_t dout_cfg = s->dxcc_sec_regs[0xe8c / 4];
        uint32_t adhi = s->dxcc_sec_regs[0xe94 / 4];
        uint64_t din = ((uint64_t)(adhi & 0xffffu) << 32) | din_lo;
        uint64_t dout = ((uint64_t)(adhi >> 16) << 32) | dout_lo;
        uint64_t din_sz = (din_cfg >> 2) & 0x00ffffffu;
        uint64_t dout_sz = (dout_cfg >> 2) & 0x00ffffffu;
        MtkDxccHashState *hs = &mtk_dxcc_hash_state;

        if (flow == 37 && setup == 1) {
            /* LOAD_STATE1: load the SHA-256 chaining state (32 bytes) or
             * start from the standard IV when no state is supplied. */
            mtk_sha_init(&hs->ctx);
            hs->active = true;
            if (din && din_sz == 32 && din <= 0x100000000ULL) {
                uint8_t iv[32];

                if (address_space_read(&address_space_memory, din,
                                       MEMTXATTRS_UNSPECIFIED, iv,
                                       sizeof(iv)) == MEMTX_OK) {
                    unsigned i;

                    /*
                     * The firmware serialises the chaining state
                     * byte-reversed (observed: the standard SHA-256 IV
                     * appears fully reversed), so word i lives at
                     * bytes [4*(7-i), +4) little-endian.
                     */
                    for (i = 0; i < 8; i++) {
                        const uint8_t *p = iv + 4 * (7 - i);

                        hs->ctx.h[i] = (uint32_t)p[0] |
                                       ((uint32_t)p[1] << 8) |
                                       ((uint32_t)p[2] << 16) |
                                       ((uint32_t)p[3] << 24);
                    }
                }
                if (MTK_ENV_CACHED("MTK_SHARUN")) {
                    static int nstate;
                    unsigned bi;

                    if (nstate < 4) {
                        nstate++;
                        fprintf(stderr, "MTK-SHARUN LOAD_STATE din=%#" PRIx64
                                " iv=", din);
                        for (bi = 0; bi < 32; bi++) {
                            fprintf(stderr, "%02x", iv[bi]);
                        }
                        fprintf(stderr, "\n");
                    }
                }
            }
        } else if (flow == 7) {
            /* S_DIN_to_HASH: stream data into the chaining state. */
            if (din && din_sz && din_sz <= 0x4000000ULL) {
                uint8_t *in = g_malloc(din_sz);

                if (address_space_read(&address_space_memory, din,
                                       MEMTXATTRS_UNSPECIFIED, in,
                                       din_sz) == MEMTX_OK) {
                    mtk_sha_update(&hs->ctx, in, (size_t)din_sz);
                    hs->active = true;
                }
                g_free(in);
            }
        } else if (flow == 43 && setup == 8) {
            /* DIN_HASH: one-shot over DIN when supplied, otherwise finalize
             * the accumulated streaming state into DOUT. */
            uint8_t digest[32];
            bool have = false;

            if (din && din_sz && din_sz <= 0x100000ULL) {
                uint8_t *in = g_malloc(din_sz);

                if (address_space_read(&address_space_memory, din,
                                       MEMTXATTRS_UNSPECIFIED, in,
                                       din_sz) == MEMTX_OK) {
                    MtkSha256Ctx one;

                    mtk_sha_init(&one);
                    mtk_sha_update(&one, in, (size_t)din_sz);
                    mtk_sha_final(&one, digest);
                    have = true;
                }
                g_free(in);
            } else if (hs->active) {
                mtk_sha_final(&hs->ctx, digest);
                hs->active = false;
                have = true;
            }
            if (have) {
                /*
                 * DOUT receives the digest (DXCC field table), and the
                 * crypto mailbox at 0x48401000 is the second place the LK
                 * crypto_hw SHA front-end reads the result from through its
                 * global pointer.
                 */
                if (dout && dout_sz) {
                    uint64_t n = dout_sz < 32 ? dout_sz : 32;
                    uint8_t *out = mt6895_guest_ptr(s, dout, n);

                    if (out) {
                        memcpy(out, digest, (size_t)n);
                    }
                }
                {
                    uint8_t *mb = mt6895_guest_ptr(s, 0x48401000ULL,
                                                   sizeof(digest));

                    if (mb) {
                        memcpy(mb, digest, sizeof(digest));
                    }
                }
                if (MTK_ENV_CACHED("MTK_SHARUN")) {
                    static int nrun;
                    unsigned bi;

                    if (nrun < 8) {
                        nrun++;
                        fprintf(stderr, "MTK-SHARUN FINAL din=%#" PRIx64
                                " din_sz=%" PRIu64 " dout=%#" PRIx64
                                " digest=", din, din_sz, dout);
                        for (bi = 0; bi < 32; bi++) {
                            fprintf(stderr, "%02x", digest[bi]);
                        }
                        fprintf(stderr, "\n");
                    }
                }
            }
        }
        /* Report the operation as finished and successful. */
        s->dxcc_sec_regs[MT6895_DXCC_SEC_RESULT / 4] = 1;
    }

    if (MTK_ENV_CACHED("MTK_DXCC_HASH") && off == 0xe90 &&
        (val == 0x908082bULL || val == 0x80c082bULL)) {
        static int nhash;
        uint64_t src = s->dxcc_sec_regs[0xe88 / 4];
        uint64_t len = s->dxcc_sec_regs[0xe8c / 4];
        uint64_t dst = s->dxcc_sec_regs[0xe80 / 4];
        uint8_t *p = src ? mt6895_guest_ptr(s, src, len ? len : 1) : NULL;

        if (nhash < 8) {
            unsigned bi;

            nhash++;
            fprintf(stderr, "MTK-DXHASH op=%" PRIx64 " src=%#" PRIx64
                    " len=%" PRIu64 " dst=%#" PRIx64 " data=",
                    val, src, len, dst);
            if (p) {
                for (bi = 0; bi < len && bi < 96; bi++) {
                    fprintf(stderr, "%02x", p[bi]);
                }
            }
            fprintf(stderr, "\n");
        }
    }

    /* WORD5 completes a descriptor: execute it (AES key/IV/data). */
    if (off == 0xe94) {
        mtk_dxcc_aes_submit(s);
    }
}

static int dec_dumped_a, dec_dumped_b;
static uint64_t dec_e80, dec_e88;
static int dec_aes_n, dec_gz_done;
static int dec_aes_n2;

static void mtk_dec_file_dump(const char *suffix, uint64_t addr, size_t len)
{
    const char *base = getenv("MTK_DUMP_SRAM");
    char path[1024];
    uint8_t *buf;
    FILE *fp;

    if (!base || !base[0] || !len) {
        return;
    }
    snprintf(path, sizeof(path), "%s_%s.bin", base, suffix);
    buf = g_malloc(len);
    if (address_space_read(&address_space_memory, addr,
                           MEMTXATTRS_UNSPECIFIED, buf, len) != MEMTX_OK) {
        fprintf(stderr, "DEC-FILE %s FAILED addr=%016" PRIx64 " len=%zu\n",
                path, addr, len);
        g_free(buf);
        return;
    }
    fp = fopen(path, "wb");
    if (!fp) {
        fprintf(stderr, "DEC-FILE %s open-failed errno=%d\n", path, errno);
        g_free(buf);
        return;
    }
    fwrite(buf, 1, len, fp);
    fclose(fp);
    fprintf(stderr, "DEC-FILE %s moment=%s addr=%016" PRIx64 " len=%zu\n",
            path, suffix, addr, len);
    g_free(buf);
}

static bool mtk_env_on(const char *name)
{
    const char *v = getenv(name);

    /* Unset or "0" is OFF; only an explicit "1" turns the gate ON. */
    return v && strcmp(v, "1") == 0;
}

static bool mtk_dec_on(void)
{
    static int on = -1;

    if (on < 0) {
        on = mtk_env_on("MTK_DXCC_DEC") ? 1 : 0;
    }
    return on > 0;
}

static void mtk_dec_memdump(const char *tag, uint64_t addr, unsigned len)
{
    uint8_t buf[256];
    unsigned i;

    if (len > sizeof(buf)) {
        len = sizeof(buf);
    }
    if (address_space_read(&address_space_memory, addr,
                           MEMTXATTRS_UNSPECIFIED, buf, len) != MEMTX_OK) {
        fprintf(stderr, "DEC-MEM %s @%016" PRIx64 " read-failed\n", tag, addr);
        return;
    }
    fprintf(stderr, "DEC-MEM %s @%016" PRIx64 ":", tag, addr);
    for (i = 0; i < len; i++) {
        fprintf(stderr, " %02x", buf[i]);
    }
    fprintf(stderr, "\n");
}

static void mtk_dec_dump(const char *why)
{
    fprintf(stderr, "DEC-DUMP %s\n", why);
    mtk_dec_memdump("gz-hdr", 0x7F800000ULL, 64);
    mtk_dec_memdump("key-e780", 0x0206E780ULL, 384);
    mtk_dec_memdump("key-ffc00", 0x020FFC00ULL, 896);
    mtk_dec_memdump("ram-fc", 0x20FC000ULL, 128);
    mtk_dec_memdump("ram-fd", 0x20FD000ULL, 128);
    mtk_dec_memdump("ram-fe", 0x20FE000ULL, 128);
    mtk_dec_memdump("ram-ffd", 0x20FFD00ULL, 128);
    mtk_dec_memdump("ram-ffe", 0x20FFE00ULL, 128);
    mtk_dec_memdump("ram-fff", 0x20FFF00ULL, 64);
}

static uint64_t mt6895_dxcc_sec_read(void *opaque, hwaddr off, unsigned size)
{
    uint64_t v = mt6895_dxcc_sec_read_impl(opaque, off, size);
    static int nrd, dstat;
    CPUState *cs = qemu_get_cpu(0);

    if (mtk_dec_on() && nrd < 600) {
        nrd++;
        fprintf(stderr, "DEC-DXCC-R off=%#" PRIx64 " size=%u val=%#" PRIx64
                " pc=%#" PRIx64 "\n", (uint64_t)off, size, v,
                cs ? (uint64_t)cpu_env(cs)->pc : 0);
    }
    if (mtk_dec_on() && off == 0xa00 && v && !dstat) {
        dstat = 1;
        mtk_dec_dump("a00-status");
    }
    if (mtk_dec_on() && off == 0xba0 && v == 1 && dec_dumped_a && !dec_dumped_b) {
        dec_dumped_b = 1;
        mtk_dec_file_dump("b_sram", 0x02000000ULL, 0x100000);
        mtk_dec_file_dump("b_gz", 0x7F800000ULL, 0x200000);
    }
    return v;
}

static void mt6895_dxcc_sec_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    static int nwr, groups, cmd4;

    if (mtk_dec_on() && nwr < 600) {
        nwr++;
        fprintf(stderr, "DEC-DXCC-W off=%#" PRIx64 " size=%u val=%#" PRIx64
                " pc=%#" PRIx64 " g=%d\n", (uint64_t)off, size, val,
                (uint64_t)(qemu_get_cpu(0) ? cpu_env(qemu_get_cpu(0))->pc : 0),
                groups);
    }
    if (mtk_dec_on() && off == 0xa08 && val == 4 && !cmd4) {
        cmd4 = 1;
        mtk_dec_dump("cmd4");
    }
    if (mtk_dec_on() && off == 0xe94 && groups < 6) {
        groups++;
        mtk_dec_dump("desc-group");
    }
    if (mtk_dec_on() && off == 0xe90 && !dec_dumped_a &&
        (((val & 0x825ULL) == 0x825ULL) || ((val & 0x82bULL) == 0x82bULL))) {
        dec_dumped_a = 1;
        mtk_dec_file_dump("a_sram", 0x02000000ULL, 0x100000);
        mtk_dec_file_dump("a_gz", 0x7F800000ULL, 0x200000);
    }
    if (mtk_dec_on() && (off == 0xe80 || off == 0xe88) && nwr < 40) {
        mtk_dec_memdump(off == 0xe80 ? "e80-buf" : "e88-buf",
                        val & ~0xFULL, 32);
    }
    if (mtk_dec_on()) {
        if (off == 0xe80) {
            dec_e80 = val;
        }
        if (off == 0xe88) {
            dec_e88 = val;
        }
        if (off == 0xe84) {
            unsigned cmd = (unsigned)(val & 0xffff);

            if ((cmd == 0x11 || cmd == 0x12 || cmd == 0x41 || cmd == 0x42) &&
                dec_aes_n < 40) {
                dec_aes_n++;
                fprintf(stderr, "DEC-AES cmd=%#x e80=%016" PRIx64
                        " e88=%016" PRIx64 "\n", cmd, dec_e80, dec_e88);
                mtk_dec_memdump("aes-e80", dec_e80 & ~0xFULL, 64);
                mtk_dec_memdump("aes-e88", dec_e88 & ~0xFULL, 64);
            }
        }
    }
    if (mtk_dec_on() && off == 0xe84 && (val & 0xffff) == 0x42 &&
        dec_e80 >= 0x7F800000ULL && dec_e80 < 0x7FA00000ULL && !dec_gz_done) {
        dec_gz_done = 1;
        mtk_dec_file_dump("gz_in", 0x7F800000ULL, 0x200000);
        mtk_dec_file_dump("sram_in", 0x02000000ULL, 0x100000);
    }
    if (mtk_dec_on() && off == 0xe84) {
        unsigned cmd2 = (unsigned)(val & 0xffff);
        bool gzref = (dec_e80 >= 0x7F000000ULL && dec_e80 < 0x80000000ULL) ||
                     (dec_e88 >= 0x7F000000ULL && dec_e88 < 0x80000000ULL);

        if ((cmd2 == 0x11 || cmd2 == 0x12 || cmd2 == 0x41 || cmd2 == 0x42) &&
            gzref && dec_aes_n2 < 400) {
            dec_aes_n2++;
            fprintf(stderr, "DEC-AES2 cmd=%#x e80=%016" PRIx64
                    " e88=%016" PRIx64 "\n", cmd2, dec_e80, dec_e88);
            mtk_dec_memdump("gz-aes-e80", dec_e80 & ~0xFULL, 64);
            mtk_dec_memdump("gz-aes-e88", dec_e88 & ~0xFULL, 64);
            if (!dec_gz_done) {
                dec_gz_done = 1;
                mtk_dec_file_dump("gzref_in", 0x7F800000ULL, 0x200000);
                mtk_dec_file_dump("sramref_in", 0x02000000ULL, 0x100000);
            }
        }
    }
    mt6895_dxcc_sec_write_impl(opaque, off, val, size);
    if (mtk_dec_on() && dec_gz_done == 1 && off == 0xe84 &&
        (val & 0xffff) == 0x42) {
        mtk_dec_file_dump("gz_out", 0x7F800000ULL, 0x200000);
        mtk_dec_file_dump("sram_out", 0x02000000ULL, 0x100000);
        dec_gz_done = 2;
    }
}

static const MemoryRegionOps mt6895_dxcc_sec_ops = {
    .read = mt6895_dxcc_sec_read,
    .write = mt6895_dxcc_sec_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * masp: the secure/crypto accelerator the seclib drives.
 *
 * Two independent drivers use the command register at +8, each waiting on its
 * own completion bit:
 *
 *   (a) size-select path (0x0205237c) -- start = bit 30, done = bit 31
 *       0x020523f2:  ldr  r0, [r6, #4]
 *       0x020523f4:  orr  r0, r0, #0x40000000
 *       0x020523f8:  str  r0, [r6, #4]
 *       0x020523fa:  ldr  r0, [r6, #4]
 *       0x020523fc:  cmp.w r0, #-1
 *       0x02052400:  bgt  #0x20523fa       ; until bit 31 comes back
 *
 *   (b) block path (0x02052466) -- start = bits 1 and 0, done = bit 15
 *       0x0205249a:  orr  r5, r5, #2
 *       0x0205249e:  str  r5, [ip]         ; ip = 0x1c009008
 *       0x020524ea:  orr  r6, r6, #1
 *       0x020524ee:  str  r6, [ip]
 *       0x020524f2:  ldr  r0, [ip]
 *       0x020524f6:  lsls r0, r0, #0x10
 *       0x020524f8:  bpl  #0x20524f2       ; until bit 15 comes back
 *
 * Both completion bits are therefore raised as soon as the register is read --
 * an accelerator that finishes instantly -- while the start bits the driver
 * wrote are preserved.
 *
 * LIMITATION: no transformation is performed. A caller that consumes the
 * accelerator's output data, rather than only these completion flags, will
 * see whatever was already in memory.
 */
#define MT6895_MASP_DONE_BITS   0x80008000U     /* bit 31 and bit 15 */

/*
 * rnd-dev (round 1,290): second HACC AES bank at masp+0x100, driven by BL31's
 * plat_trng_prng (tee_a ATF 0x48216a34, poll helper 0x482171f0) after it has
 * read four TRNG words:
 *   +0x110..0x11c  IN0..3   <- TRNG seed
 *   +0x100 CON     = 2 (clear), then 5 (enable | mode)
 *   +0x104 START   = 1, then polled until bit 15 (ready)
 *   +0x120..0x12c  OUT0..3  -> returned as MTK_SIP_*_GET_RND data
 *   +0x100 CON     = 0 at the end
 * Previously +0x104 was plain RAM, bit 15 never rose and BL31 logged
 * "plat_trng_prng: hacc fail" -> LK "get_rnd failed".
 * The engine is an AES-128 block cipher keyed by a per-chip hardware key that
 * software cannot read; the model uses a fixed model-internal key (the real
 * HUK is unknown), i.e. OUT = AES-128-ECB(HUK_model, IN).  Since IN comes from
 * the TRNG the result is unpredictable, which is all the callers rely on.
 * MTK_NO_HACC=1 restores the old RAM behaviour.
 */

#define MT6895_HACC2_CON    0x100
#define MT6895_HACC2_START  0x104
#define MT6895_HACC2_IN     0x110
#define MT6895_HACC2_OUT    0x120
#define MT6895_HACC2_RDY    0x8000u

static bool mt6895_hacc2_done;

static void mt6895_hacc2_run(Mt6895MachineState *s)
{
    static const uint8_t huk_model[16] = {
        0x4d, 0x54, 0x36, 0x38, 0x39, 0x35, 0x2d, 0x48,  /* "MT6895-HACC-HUK!" */
        0x41, 0x43, 0x43, 0x2d, 0x48, 0x55, 0x4b, 0x21,
    };
    AES_KEY key;
    uint8_t in[16], out[16];
    int i;

    for (i = 0; i < 4; i++) {
        stl_le_p(in + 4 * i, s->masp_regs[(MT6895_HACC2_IN >> 2) + i]);
    }
    AES_set_encrypt_key(huk_model, 128, &key);
    AES_encrypt(in, out, &key);
    for (i = 0; i < 4; i++) {
        s->masp_regs[(MT6895_HACC2_OUT >> 2) + i] = ldl_le_p(out + 4 * i);
    }
    mt6895_hacc2_done = true;
    if (getenv("MTK_RPMB_TRACE")) {
        fprintf(stderr, "MTK-HACC2 aes in=%08x.. out=%08x..\n",
                s->masp_regs[MT6895_HACC2_IN >> 2],
                s->masp_regs[MT6895_HACC2_OUT >> 2]);
    }
}

static uint64_t mt6895_masp_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off + 4 > MT6895_MASP_SIZE) {
        return 0;
    }

    if (off == MT6895_MASP_CMD) {
        return s->masp_regs[off / 4] | MT6895_MASP_DONE_BITS;
    }
    if (off == MT6895_HACC2_START && mt6895_hacc2_done &&
        !MTK_ENV_CACHED("MTK_NO_HACC")) {
        return s->masp_regs[off / 4] | MT6895_HACC2_RDY;
    }

    return s->masp_regs[off / 4];
}

static void mt6895_masp_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off + 4 > MT6895_MASP_SIZE) {
        return;
    }

    s->masp_regs[off / 4] = val;

    if (MTK_ENV_CACHED("MTK_NO_HACC")) {
        return;
    }
    if (off == MT6895_HACC2_CON && !(val & 1)) {
        mt6895_hacc2_done = false;          /* clear / disable */
    } else if (off == MT6895_HACC2_START && (val & 1) &&
               (s->masp_regs[MT6895_HACC2_CON >> 2] & 1)) {
        mt6895_hacc2_run(s);
    }
}

static const MemoryRegionOps mt6895_masp_ops = {
    .read = mt6895_masp_read,
    .write = mt6895_masp_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};


#define MT6895_DRAMC_TOP2_SIZE 0x1000
#define MT6895_DEVAPC_SIZE     0x1000
#define MT6895_DRAMC_CMD_SIZE  0x2000
#define MT6895_DRAMC_TOP3_SIZE 0x1000

/*
 * Which part is fitted follows the configured RAM size, so the mode-register
 * answers live next to MT6895_DRAM_SIZE (mt6895_dram_mr5/mt6895_dram_mr8)
 * rather than in compile-time constants here.
 */

/*
 * Nothing answers at rank 1: an MRR there reads back all ones, which puts
 * MR8's density code out of range and leaves that rank's size zero.
 */
#define MT6895_DRAM_MR_ABSENT   0xffff

/* MRR command register, controller offset 0x130. */
#define MT6895_DRAMC_CMD_REG        0x130
#define MT6895_DRAMC_CMD_RANK(v)    (((v) >> 10) & 0x3)
#define MT6895_DRAMC_CMD_MR(v)      (((v) >> 12) & 0xff)

static uint64_t mt6895_dramc_top2_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895DramcWin *w = opaque;
    uint32_t *regs = w->regs;

    if (MTK_ENV_CACHED("MTK_DRAMCALL")) {
        static int na;

        if (na < 4000) {
            na++;
            fprintf(stderr, "MTK-DRAMCALL off=%#x\n", (unsigned)off);
        }
    }

    if (off + 4 > MT6895_DRAMC_TOP2_SIZE) {
        return 0;
    }

    /*
     * DIAGNOSTIC (MTK_DRAMC_OVR="off:val,off:val"): override arbitrary DRAMC
     * registers so the cold/warm-boot indication can be located by sweep.
     */
    if (MTK_ENV_CACHED("MTK_DRAMC_OVR")) {
        const char *e = getenv("MTK_DRAMC_OVR");
        const char *p = e;

        while (p && *p) {
            unsigned o = 0, v = 0;
            char *end;

            o = strtoul(p, &end, 16);
            if (*end != ':') {
                break;
            }
            p = end + 1;
            v = strtoul(p, &end, 16);
            if (o == (unsigned)off) {
                return v;
            }
            p = (*end == ',') ? end + 1 : NULL;
        }
    }

    /*
     * DIAGNOSTIC (MTK_DRAMC_RES=<hex>): force the un-modeled calibration
     * result block (0x200..0x240) so it can be established which register
     * family the RX/TX window scan consumes.
     */
    if (off >= 0x200 && off <= 0x240) {
        /*
         * RX/TX window-scan result block.
         *
         * The training code walks every delay step and asks this block
         * whether that step sampled a valid window; the payload's bring-up
         * reads it 3264 times.  The emulated DRAM is ideal -- its timing,
         * impedance and duty cycle never drift -- so every step samples a
         * valid window and the faithful answer is "all bits set".  With the
         * block left un-modeled it read back as zero and bring-up reported
         * "Error! Probably miss pass window!" / "Error! TX auto K is fail!"
         * on every step and then threw the calibration away.
         */
        /*
         * MTK_DRAMC_RES=<hex>: force a different RX/TX window-scan answer.
         * The 2026-11-06 reference run trained DRAM with this block reading
         * zero (it reported "TX auto K is fail" / "miss pass window" yet went
         * on to reach LK), so the all-ones default is not the only path the
         * firmware can take -- both are worth being able to select.
         */
        if (MTK_ENV_CACHED("MTK_DRAMC_RES")) {
            return strtoull(getenv("MTK_DRAMC_RES"), NULL, 0);
        }
        return 0xffffffffu;
    }

    switch (off) {
    case 0x10:
        /* Bit 0: DRAM channel calibration done (polled by 0x02014c3e) */
        return regs[off / 4] | 0x1;
    case 0x20:
        /*
         * Mode-register read-back. The controller rewrites the command
         * register of this channel (offset 0x130, command window) before
         * every MRR, and the MRR executor copies this register's low
         * halfword into the caller's result (0x0202ce7a). Nothing behind
         * the controller can answer an MRR, so the emulated part does.
         */
        {
            uint32_t cmd = w->s->dramc_mrr_cmd[w->ch];
            uint32_t res;

            if (MT6895_DRAMC_CMD_RANK(cmd) != 0) {
                res = MT6895_DRAM_MR_ABSENT;
            } else {
                switch (MT6895_DRAMC_CMD_MR(cmd)) {
                case 5:
                    res = mt6895_dram_mr5(w->s);
                    break;
                case 6:
                case 7:
                    res = 0;
                    break;
                case 8:
                    res = mt6895_dram_mr8(w->s);
                    break;
                default:
                    res = (MT6895_DRAMC_CMD_RANK(cmd) == 0)
                              ? mt6895_dram_mr8(w->s) : MT6895_DRAM_MR_ABSENT;
                    break;
                }
            }
            return res;
        }
    case 0x50:
        /* Bit 0: RTSW command done (polled by 0x0201e.../DramcTriggerRTSWCMD) */
        return regs[off / 4] | 0x1;
    case 0x80:
        /* Bit 2: RWOFOEN queue empty (polled by 0x0202c23e) */
        return regs[off / 4] | 0x4;
    case 0x88:
        /* Bit 0: DRAM channel calibration done */
        return regs[off / 4] | 0x1;
    case 0xc8:
        /*
         * Bit 0: DramcTriggerAndWait completion.
         *
         * Bit 27: command-bus training (cbt_autok) response. The sequence at
         * 0x02024DA6..0x02024DC4 polls this register 0xF00 times, delaying
         * 1ms between reads, and leaves the loop only while
         *   ands r0, r5, #0x8000000   ; bit 27
         *   beq  <retry>
         * is non-zero. With only bit 0 set the response never appears and
         * bring-up prints
         *   "[cbt_autok] Resp fail (time out) for CATrain / CSTrain"
         * then trips the DRAM calibration assert, so report both flags.
         */
        return regs[off / 4] | 0x1 | 0x08000000;
    case 0x120:
        /*
         * DRAM PHY calibration status.
         *
         * Bits [1:0] and [22:20] are the waveform-measurement
         * result-ready flags. Bit 13 is the impedance/DLL step flag: the
         * sequence at 0x0202ae18 writes 0x10c = 0x40000000 and
         * 0x110 = 0x80000 / 0x40, then polls this register in a loop with
         * no retry limit (0x0202ae48, `b 0x0202ae48` at 0x0202ae5a),
         * leaving only when the value reads negative after
         * `lsls r0, r0, #0x12` -- that is, only when bit 13 is set. The
         * preloader's own calibration report shows every channel passing
         * TA2, so the step is done and the flag reads as set.
         */
        return regs[off / 4] | 0x00702003;
    case 0x124: {
        /*
         * MTK_DRAMC_R124: the training code reads this register 195 times per
         * bring-up and the model had no description for it, so it always
         * returned the last write (usually zero) and the calibration loops
         * could never see a result.  The encoding is swept with
         * MTK_DRAMC_124=<hex> so the value the training algorithm accepts can
         * be identified, then hard-coded with a documented rationale.
         */
        static int v = -1;

        if (v < 0) {
            const char *e = getenv("MTK_DRAMC_124");

            v = e ? (int)strtoul(e, NULL, 16) : 0;
        }
        if (v) {
            return (uint32_t)v;
        }
        return regs[off / 4];
    }
    case 0x230:
        /*
         * Bit 31: "TX auto K" training done.
         *
         * 0x0202035c polls this register once per millisecond, up to
         * 100001 times, and leaves the loop only when the value reads
         * negative (0x02020374 `bmi`). On timeout it prints "Error! TX
         * auto K is not done!" and carries a failed result into the rest
         * of bring-up. The polled register is dramc-top2 offset 0x230
         * (the preloader passes 0x00800230 through u4RegBaseAddrTranslate,
         * whose group-1 entry plus the +0x10000000 bias lands on this
         * window). TX auto K is the controller's own TX equalisation, and
         * the preloader's calibration report already shows TX DQ/DQS as
         * resolved, so the step reports done.
         */
        return regs[off / 4] | 0x80000000;
    case 0x170:
        /* Waveform measurement progress: 8, or 2 once 0x100 bit 25 is set */
        return (regs[0x100 / 4] & 0x2000000) ? 2 : 8;
    default:
        /*
         * MTK_DRAMC_ONES: diagnostic -- report all un-modeled DRAMC
         * registers as set, to find which one carries the DRAM-exception
         * latch that distinguishes a cold boot from a reboot.
         */
        if (MTK_ENV_CACHED("MTK_DRAMC_ONES")) {
            return 0xffffffffu;
        }
        /*
         * MTK_DRAMCTRACE: report reads that hit registers the model does not
         * describe yet.  These are exactly the calibration results the DRAM
         * training code consumes, so listing them shows which registers still
         * need a faithful model.
         */
        if (MTK_ENV_CACHED("MTK_DRAMCTRACE")) {
            static int nd;

            if (nd < 200) {
                nd++;
                fprintf(stderr, "MTK-DRAMCRD-UNMODELED off=%#x val=%#x\n",
                        (unsigned)off, regs[off / 4]);
            }
        }
        return regs[off / 4];
    }
}

static void mt6895_dramc_top2_write(void *opaque, hwaddr off, uint64_t val,
                                    unsigned size)
{
    if (MTK_ENV_CACHED("MTK_DRAMCWR2")) {
        static int nw;

        if (nw < 200) {
            nw++;
            fprintf(stderr, "DRAMCWR2 off=%#x val=%#x\n",
                    (unsigned)off, (unsigned)val);
        }
    }

    Mt6895DramcWin *w = opaque;
    uint32_t *regs = w->regs;

    if (off + 4 > MT6895_DRAMC_TOP2_SIZE) {
        return;
    }

    regs[off / 4] = val;
}

static const MemoryRegionOps mt6895_dramc_top2_ops = {
    .read = mt6895_dramc_top2_read,
    .write = mt6895_dramc_top2_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * DEVAPC violation-clear handshake.
 *
 * 0x0200dd3a writes 1 to +0xdc on the INFRA_DEVAPC, INFRA_DEVAPC1 and
 * PERI_DEVAPC windows, then spins at 0x0200dd46 / 0x0200dd54 / 0x0200dd62
 * until +0x1fc reads 1 on each, then writes 0 back to +0xdc. Nothing behind
 * the controller can complete that clear in emulation, so the window models
 * it: the trigger arms a pending flag and +0x1fc reports it.
 */
static uint64_t mt6895_devapc_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895DevapcWin *w = opaque;

    if (off + 4 > MT6895_DEVAPC_SIZE) {
        return 0;
    }
    if (off == 0x1fc) {
        return w->clear_pending ? 1 : 0;
    }
    return w->regs[off / 4];
}

static void mt6895_devapc_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    Mt6895DevapcWin *w = opaque;

    if (off + 4 > MT6895_DEVAPC_SIZE) {
        return;
    }
    w->regs[off / 4] = val;
    if (off == 0xdc) {
        w->clear_pending = (val & 1) != 0;
    }
}

static const MemoryRegionOps mt6895_devapc_ops = {
    .read = mt6895_devapc_read,
    .write = mt6895_devapc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static uint64_t mt6895_dramc_top3_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895DramcWin *w = opaque;
    uint32_t *regs = w->regs;

    if (off + 4 > MT6895_DRAMC_TOP3_SIZE) {
        return 0;
    }

    if (off == 0x510 || off == 0x5c8) {
        static unsigned top3_probe_count[2];
        unsigned which = off == 0x510 ? 0 : 1;

        if (top3_probe_count[which] < 128) {
            ARMCPU *cpu = ARM_CPU(current_cpu);
            uint32_t pc = cpu ? (uint32_t)cpu->env.regs[15] : 0;
            uint32_t lr = cpu ? (uint32_t)cpu->env.regs[14] : 0;

    (void)pc;
    (void)lr;
            fprintf(stderr,
                    "mt6895-dbg: top3 read ch=%d off=0x%03x raw=0x%08x "
                    "ret=0x%08x pc=%08x lr=%08x\n",
                    w->ch, (unsigned)off, regs[off / 4],
                    regs[off / 4] | (off == 0x510 ? 0x8 : 0x80000000u), pc, lr);
            top3_probe_count[which]++;
        }
    }

    switch (off) {
    case 0x10:
        /* Bits 14..15: calibration FSM status polled by 0x020153c4 */
        return regs[off / 4] | 0xc000;
    case 0x940:
        /*
         * Calibration status of this channel. Bit 0 and bit 1 are the RX
         * window results the training code waits for at 0x02023b28..88,
         * and bit 2 is the "done" flag the EMI-driven second pass spins on
         * at 0x0202370e..18; report all three so each poll completes.
         */
        return regs[off / 4] | 0x7;
    case 0x510:
        /*
         * The helper first uses its r2=0x100 argument (bit 0) to trigger the
         * command, but its final status poll uses the fifth argument 0x103:
         * that encodes bit 3. 0x1800510 translates through group 3 of the
         * table at 0x02069BE4 to 0x10236510, this window's offset 0x510.
         * Return the completion bit used by the status poll; returning only
         * bit 0 reduced some waits but left the actual completion condition
         * unsatisfied.
         */
        return regs[off / 4] | 0x8;
    case 0x5c8:
        /*
         * The post-trigger status poll passes 0x11f to 0x0201BF4C:
         * (0x11f >> 8) selects a 31-bit field and 0x11f & 0xff selects
         * bit 31. 0x18005C8 maps to this top3 window's offset 0x5c8.
         */
        return regs[off / 4] | 0x80000000u;
    default:
        return regs[off / 4];
    }
}

static void mt6895_dramc_top3_write(void *opaque, hwaddr off, uint64_t val,
                                    unsigned size)
{
    Mt6895DramcWin *w = opaque;
    uint32_t *regs = w->regs;

    if (off + 4 > MT6895_DRAMC_TOP3_SIZE) {
        return;
    }

    regs[off / 4] = val;
}

static const MemoryRegionOps mt6895_dramc_top3_ops = {
    .read = mt6895_dramc_top3_read,
    .write = mt6895_dramc_top3_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static uint64_t mt6895_dramc_cmd_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895DramcWin *w = opaque;

    if (off + 4 > MT6895_DRAMC_CMD_SIZE) {
        return 0;
    }

    if (off == MT6895_DRAMC_CMD_REG) {
        return w->s->dramc_mrr_cmd[w->ch];
    }
    return w->regs[off / 4];
}

static void mt6895_dramc_cmd_write(void *opaque, hwaddr off, uint64_t val,
                                   unsigned size)
{
    Mt6895DramcWin *w = opaque;

    if (off + 4 > MT6895_DRAMC_CMD_SIZE) {
        return;
    }

    if (off == MT6895_DRAMC_CMD_REG) {
        /* Keep the MRR request where the data-register read can find it. */
        w->s->dramc_mrr_cmd[w->ch] = val;
        return;
    }
    w->regs[off / 4] = val;
}

static const MemoryRegionOps mt6895_dramc_cmd_ops = {
    .read = mt6895_dramc_cmd_read,
    .write = mt6895_dramc_cmd_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};


#define MT6895_TOPCKGEN_SIZE 0x1000

/*
 * Topckgen clock block.  Besides the plain register file it carries the
 * frequency-meter (PLL/oscillator measurement) pair used by the preloader's
 * PMIF clock-manager calibration: 0x220 starts a measurement, 0x224 reads the
 * pulse count.
 *
 * MTK_TOPCLK_TRACE=1 logs every access; it is observational only.
 */
/* Last window/trigger word written to the frequency meter (topckgen 0x224). */
static uint32_t mt6895_topckgen_meter_cfg;
/*
 * Set when the PMIF clock-manager starts a ULPOSC measurement (write to 0x220
 * with bit 4 set) after programming the ULPOSC window word (0x400) into 0x224.
 * Cleared on any 0x224 write: the preloader writes the previous RESULT back
 * into 0x224 after each measurement, which is exactly what must end one armed
 * measurement.
 */
static bool mt6895_ulposc_armed;

/*
 * ULPOSC1 frequency meter (topckgen meter channel 63).
 *
 * The preloader's PMIF clock-manager calibration trims the ULPOSC1 RC
 * oscillator through the VLP clock-sys block (syscon "vlp_cksys_clk" + 0x200,
 * ULPOSC_CON0; trim code in bits [6:0]) and measures it with this meter.
 * The measured frequency of each trim code, from the real unit's own boot log
 * ("[PMIF] cali_val=... , ... KHz"):
 *
 *   code 0x3f -> 245476 kHz      code 0x49 -> 261420 kHz
 *   code 0x47 -> 258168 kHz      code 0x4b -> 264976 kHz
 *   code 0x48 -> 260304 kHz      code 0x4f -> 271880 kHz
 *                                code 0x5f -> 304176 kHz
 *                                code 0x7f -> 394772/394872 kHz
 *
 * The device's search walks exactly these codes and settles on 0x48
 * ("[PMIF] ULPOSC1 K done: 260M"), so reproducing the curve reproduces the
 * calibration.  In between the points are linearly interpolated and below
 * 0x3f the first segment's slope is extrapolated.
 *
 * The meter counts the test clock during a window of 256 reference cycles of
 * 26 MHz, i.e. kHz = count * 26000 / 256.  That is the same arithmetic the old
 * fixed count 1024 reproduced as 104000 kHz.
 *
 * Trim-field and arming rule, both derived from the preloader's own trace
 * (SPMI-syscon CON0 writes paired with the serial "cali_val=... KHz" prints):
 *
 *   - Every calibration write to ULPOSC_CON0 is 0x2829200 | (0x100 | cali[6:0]),
 *     i.e. the applied value is 0x100 | (cali & 0x7F).  Bit 8 is a constant
 *     band bit during the search, so the effective trim is CON0[6:0] and the
 *     curve table above (keyed 0x3f..0x7f) is exactly the device's curve.
 *   - A measurement is started by writing 0x220 with bit 4 set (the trace shows
 *     rds 0x220, wr 0x220=0x8000, wr 0x220=0x9010, rd 0x220, rd 0x224).  The
 *     driver then writes the measured value back into 0x224, so a read of 0x224
 *     only reports a measurement while the block is armed; the old check
 *     (window word still == 0x400 on read) stopped working after the first
 *     write-back.
 *   - The later 135-iteration PLL-meter block programs 0x224 = 0x3ff0400, so
 *     arming additionally requires the last 0x224 window word to be 0x400 and
 *     the PLL path is untouched.
 */
static const struct {
    uint32_t code;
    uint32_t khz;
} mt6895_ulposc_curve[] = {
    { 0x3f, 245476 },
    { 0x47, 258168 },
    { 0x48, 260304 },
    { 0x49, 261420 },
    { 0x4b, 264976 },
    { 0x4f, 271880 },
    { 0x5f, 304176 },
    { 0x7f, 394872 },
};

static uint32_t mt6895_ulposc_khz(uint32_t code)
{
    unsigned i;
    const unsigned n = ARRAY_SIZE(mt6895_ulposc_curve);

    if (code <= mt6895_ulposc_curve[0].code) {
        int32_t slope = (int32_t)(mt6895_ulposc_curve[1].khz -
                                  mt6895_ulposc_curve[0].khz) /
                        (int32_t)(mt6895_ulposc_curve[1].code -
                                  mt6895_ulposc_curve[0].code);
        return (uint32_t)((int32_t)mt6895_ulposc_curve[0].khz +
                          slope * ((int32_t)code -
                                   (int32_t)mt6895_ulposc_curve[0].code));
    }
    for (i = 1; i < n; i++) {
        if (code <= mt6895_ulposc_curve[i].code) {
            uint32_t lo = mt6895_ulposc_curve[i - 1].khz;
            uint32_t hi = mt6895_ulposc_curve[i].khz;
            uint32_t c0 = mt6895_ulposc_curve[i - 1].code;
            uint32_t c1 = mt6895_ulposc_curve[i].code;

            return lo + (hi - lo) * (code - c0) / (c1 - c0);
        }
    }
    return mt6895_ulposc_curve[n - 1].khz;
}

/* Meter result: ULPOSC1 frequency for the trim code currently programmed. */
static uint32_t mt6895_ulposc_meter_count(void)
{
    /* Applied CON0 = 0x100 | cali[6:0]; bit 8 is the constant band bit. */
    uint32_t code = mtk_vlp_ulposc_con0 & 0x7f;
    uint32_t khz = mt6895_ulposc_khz(code);
    uint32_t count = (khz * 256u + 13000u) / 26000u;

    if (MTK_ENV_CACHED("MTK_ULPOSC_LOG")) {
        fprintf(stderr, "MTK-ULPOSC con0=%#x code=%#x khz=%u count=%u\n",
                mtk_vlp_ulposc_con0, code, khz, count);
    }
    return count;   /* round to nearest count */
}

static uint64_t mt6895_topckgen_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint32_t v;

    if (off + 4 > MT6895_TOPCKGEN_SIZE) {
        return 0;
    }

    switch (off) {
    case 0x220:
        /* Bit 4: frequency meter trigger/busy. Cleared when measurement done. */
        v = s->topckgen_regs[off / 4] & ~0x10U;
        break;
    case 0x224:
        /*
         * Armed ULPOSC measurement -> report the measured ULPOSC1 frequency.
         * The PLL-meter path programs a different window word and never arms,
         * so it is left to the plain register model until the PLL tree is
         * modelled.
         */
        if (mt6895_ulposc_armed) {
            v = mt6895_ulposc_meter_count();
        } else {
            v = s->topckgen_regs[off / 4] == 0 ? 1024 :
                s->topckgen_regs[off / 4];
        }
        break;
    default:
        v = s->topckgen_regs[off / 4];
        break;
    }
    if (MTK_ENV_CACHED("MTK_TOPCLK_TRACE")) {
        static int budget;

        if (budget < 200000) {
            budget++;
            fprintf(stderr, "MTK-TOPCK rd off=%#x -> %#x\n",
                    (unsigned)off, v);
        }
    }
    return v;
}

static void mt6895_topckgen_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off + 4 > MT6895_TOPCKGEN_SIZE) {
        return;
    }
    if (MTK_ENV_CACHED("MTK_TOPCLK_TRACE")) {
        static int budget;

        if (budget < 200000) {
            budget++;
            fprintf(stderr, "MTK-TOPCK wr off=%#x val=%#x\n",
                    (unsigned)off, (unsigned)val);
        }
    }

    if (off == 0x224) {
        mt6895_topckgen_meter_cfg = (uint32_t)val;
        /* Any window/result write ends the previous measurement. */
        mt6895_ulposc_armed = false;
    } else if (off == 0x220 && (val & 0x10) &&
               mt6895_topckgen_meter_cfg == 0x400) {
        /*
         * bit 4 = start measurement.  Only the ULPOSC calibration path (window
         * word 0x400) is armed; the PLL-meter path (0x3ff0400) is untouched.
         */
        mt6895_ulposc_armed = true;
    }

    s->topckgen_regs[off / 4] = val;
}

/*
 * Catch-all for the peripheral register window. LK's monitor / debug_ctrl_ao
 * drivers (from the device tree) touch registers in 0x1000_0000-0x1003_ffff
 * that this model does not implement; an unmapped write faults and LK dies.
 * Model them as a low-priority fill region so the real device models (higher
 * priority) still take precedence.
 */
/*
 * DIAGNOSTIC (MTK_ATF_WATCH): read-watch on the ATF log format strings. The
 * vendor ATF logs through table-driven helpers, so static cross references do
 * not reveal the caller; trapping the read of a specific format string gives
 * the PC that printed it. Reads are forwarded to the underlying RAM.
 */
static MemoryRegion mt6895_watch_mr;
#define MT6895_WATCH_BASE 0x48236700ULL
#define MT6895_WATCH_SIZE 0x100ULL

/* Base of the currently installed generic watch window (MTK_WATCH_ADDR or the
 * fixed ATF probe address).  The ops forward to the real RAM/DRAM underneath,
 * so installing a window never changes guest-visible behaviour. */
static uint64_t mt6895_watch_cur_base = MT6895_WATCH_BASE;

static uint64_t mt6895_watch_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
    static int n;

    if (n < 200) {
        uint64_t pc = 0;
        uint64_t pcmin = 0;
        const char *pmin = getenv("MTK_WATCH_PCMIN");
        bool show = true;

        n++;
        if (cpu) {
            /* AArch32 keeps the PC in regs[15]; AArch64 uses env.pc. */
            pc = cpu->env.aarch64 ? cpu->env.pc : (uint64_t)cpu->env.regs[15];
        }
        if (pmin) {
            pcmin = strtoull(pmin, NULL, 0);
            show = pc >= pcmin;
        }
        if (show) {
            /* ATF runs at EL3: the active stack pointer is SP_EL3. */
            uint64_t sp = cpu ? (uint64_t)cpu->env.sp_el[3] : 0;
            fprintf(stderr, "MT6895-WATCH read off=%#" HWADDR_PRIx
                    " pc=%#" PRIx64 " sp=%#" PRIx64 "\n",
                    mt6895_watch_cur_base + off, pc, sp);
            if (sp) {
                unsigned wi;

                fprintf(stderr, "  stack:");
                for (wi = 0; wi < 32; wi++) {
                    fprintf(stderr, " %#" PRIx64,
                            (uint64_t)address_space_ldq_le(
                                &address_space_memory, sp + wi * 8,
                                MEMTXATTRS_UNSPECIFIED, NULL));
                }
                fprintf(stderr, "\n");
            }
        }
    }
    {
        uint8_t *p = mt6895_guest_ptr(s, mt6895_watch_cur_base + off, size);
        uint64_t v = 0;
        unsigned i;

        if (!p) {
            return 0;
        }
        for (i = 0; i < size; i++) {
            v |= (uint64_t)p[i] << (8 * i);
        }
        return v;
    }
}

static void mt6895_watch_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, mt6895_watch_cur_base + off, size);
    unsigned i;

    if (MTK_ENV_CACHED("MTK_WATCH_ADDR") && mt6895_watch_cur_base != MT6895_WATCH_BASE) {
        static int nw;
        CPUState *cs = qemu_get_cpu(0);
        uint64_t pc = 0;

        if (cs) {
            CPUARMState *env = cpu_env(cs);
            pc = env->aarch64 ? env->pc : (uint64_t)env->regs[15];
        }
        {
            const char *wm = getenv("MTK_WWMAX");
            int wmax = wm ? atoi(wm) : 60;
            bool nz = getenv("MTK_WNONZERO") != NULL;

            if ((!nz || val) && nw < wmax) {
                nw++;
                fprintf(stderr, "MTK-WATCH-WR addr=%#" PRIx64
                        " val=%#" PRIx64 " size=%u pc=%#" PRIx64 "\n",
                        mt6895_watch_cur_base + off, val, size, pc);
            }
        }
    }
    if (!p) {
        return;
    }
    for (i = 0; i < size; i++) {
        p[i] = (val >> (8 * i)) & 0xff;
    }
}

/*
 * DIAGNOSTIC (MTK_PTWATCH): write-watch on the ATF xlat L3 entry that maps
 * sys_cirq (L3 @0x48264000, index 0x151 -> 0x48264a88). The ATF clears this
 * mapping right before it faults, so log who writes it.
 */
static MemoryRegion mt6895_ptwatch_mr;
#define MT6895_PTWATCH_BASE 0x48264000ULL

static uint64_t mt6895_ptwatch_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, MT6895_PTWATCH_BASE + off, size);
    uint64_t v = 0;
    unsigned i;

    if (!p) {
        return 0;
    }
    for (i = 0; i < size; i++) {
        v |= (uint64_t)p[i] << (8 * i);
    }
    return v;
}

static void mt6895_ptwatch_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    Mt6895MachineState *s = opaque;
    ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
    uint8_t *p = mt6895_guest_ptr(s, MT6895_PTWATCH_BASE + off, size);
    static int n;
    unsigned i;

    if (n < 40 && off >= 0xa80 && off < 0xa90) {
        n++;
        {
            uint64_t sp = cpu ? (uint64_t)cpu->env.sp_el[3] : 0;

            fprintf(stderr, "MTK-PTWATCH wr off=%#" HWADDR_PRIx " val=%#" PRIx64
                    " pc=%#" PRIx64 " lr=%#" PRIx64 " sp=%#" PRIx64
                    " x0=%#" PRIx64 " x1=%#" PRIx64 " x2=%#" PRIx64 "\n",
                    MT6895_PTWATCH_BASE + off, val,
                    cpu ? (uint64_t)cpu->env.pc : 0,
                    cpu ? (uint64_t)(cpu->env.aarch64 ? cpu->env.pc : cpu->env.regs[14]) : 0, sp,
                    cpu ? (uint64_t)cpu->env.regs[0] : 0,
                    cpu ? (uint64_t)cpu->env.regs[1] : 0,
                    cpu ? (uint64_t)cpu->env.regs[2] : 0);
            if (sp) {
                unsigned wi;

                fprintf(stderr, "  pstk:");
                for (wi = 0; wi < 24; wi++) {
                    fprintf(stderr, " %#" PRIx64,
                            (uint64_t)address_space_ldq_le(
                                &address_space_memory, sp + wi * 8,
                                MEMTXATTRS_UNSPECIFIED, NULL));
                }
                fprintf(stderr, "\n");
            }
        }
    }
    if (!p) {
        return;
    }
    for (i = 0; i < size; i++) {
        p[i] = (val >> (8 * i)) & 0xff;
    }
}

/*
 * DIAGNOSTIC (MTK_BTREAD): read-watch on BL2's boot tag area
 * (0x63002940..0x63002a40) to see whether ATF reads it.
 */
static MemoryRegion mt6895_btread_mr;
#define MT6895_BTREAD_BASE 0x63002940ULL
#define MT6895_BTREAD_SIZE 0x100ULL

static uint64_t mt6895_btread_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
    uint8_t *p = mt6895_guest_ptr(s, MT6895_BTREAD_BASE + off, size);
    uint64_t v = 0;
    unsigned i;
    static int n;

    if (n < 30) {
        n++;
        fprintf(stderr, "MTK-BTREAD rd off=%#" HWADDR_PRIx " size=%u pc=%#"
                PRIx64 "\n", MT6895_BTREAD_BASE + off, size,
                cpu ? (uint64_t)cpu->env.pc : 0);
    }
    if (!p) {
        return 0;
    }
    for (i = 0; i < size; i++) {
        v |= (uint64_t)p[i] << (8 * i);
    }
    return v;
}

static void mt6895_btread_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, MT6895_BTREAD_BASE + off, size);
    unsigned i;

    if (!p) {
        return;
    }
    for (i = 0; i < size; i++) {
        p[i] = (val >> (8 * i)) & 0xff;
    }
}

/*
 * DIAGNOSTIC (MTK_GWATCH): write-watch on the ATF global that holds the region
 * mapped by bl31_early_platform_setup (0x4825ac84 = base, 0x48245840 = size).
 */
static MemoryRegion mt6895_gwatch_mr;
#define MT6895_GWATCH_BASE 0x4825ac80ULL
#define MT6895_GWATCH_SIZE 0x10ULL

static uint64_t mt6895_gwatch_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, MT6895_GWATCH_BASE + off, size);
    uint64_t v = 0;
    unsigned i;

    if (!p) {
        return 0;
    }
    for (i = 0; i < size; i++) {
        v |= (uint64_t)p[i] << (8 * i);
    }
    return v;
}

static void mt6895_gwatch_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    Mt6895MachineState *s = opaque;
    ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
    uint8_t *p = mt6895_guest_ptr(s, MT6895_GWATCH_BASE + off, size);
    static int n;
    unsigned i;

    if (n < 16) {
        n++;
        fprintf(stderr, "MTK-GWATCH wr off=%#" HWADDR_PRIx " val=%#" PRIx64
                " pc=%#" PRIx64 " lr=%#" PRIx64 "\n",
                MT6895_GWATCH_BASE + off, val,
                cpu ? (uint64_t)cpu->env.pc : 0,
                cpu ? (uint64_t)(cpu->env.aarch64 ? cpu->env.xregs[30] : cpu->env.regs[14]) : 0);
    }
    if (!p) {
        return;
    }
    for (i = 0; i < size; i++) {
        p[i] = (val >> (8 * i)) & 0xff;
    }
}

/*
 * MTK_TZFIX: mirror BL2's TZ tag block into ATF's GTA area as soon as BL2
 * publishes it.  The real BL2 hands the tag list to ATF through the handoff
 * parameters; here we observe the published copy and re-apply it to the
 * address ATF's xlat setup reads, right after it appears.
 */
static MemoryRegion mt6895_tzfix_mr;
#define MT6895_TZFIX_BASE 0x7c800200ULL
#define MT6895_TZFIX_SIZE 0x100ULL

static uint64_t mt6895_tzfix_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, MT6895_TZFIX_BASE + off, size);
    uint64_t v = 0;
    unsigned i;

    if (!p) {
        return 0;
    }
    for (i = 0; i < size; i++) {
        v |= (uint64_t)p[i] << (8 * i);
    }
    return v;
}

static void mt6895_tzfix_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, MT6895_TZFIX_BASE + off, size);
    uint8_t *g = mt6895_guest_ptr(s, 0x4825ad1cULL, 0x100);
    static int nfix;
    unsigned i;

    if (!p) {
        return;
    }
    for (i = 0; i < size; i++) {
        p[i] = (val >> (8 * i)) & 0xff;
    }
    if (g && p[0] == 'T' && p[1] == 'A' && p[2] == 'G') {
        memcpy(g, p, 0x100);
        if (nfix < 4) {
            nfix++;
            fprintf(stderr, "MTK-TZFIX mirrored tag block (#%d)\n", nfix);
        }
    }
}

static const MemoryRegionOps mt6895_tzfix_ops = {
    .read = mt6895_tzfix_read,
    .write = mt6895_tzfix_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* DIAGNOSTIC (MTK_GTAWATCH): write-watch on ATF's GTA area (0x4825ad1c). */
static MemoryRegion mt6895_gtawatch_mr;
#define MT6895_GTAWATCH_BASE 0x4824a900ULL
#define MT6895_GTAWATCH_SIZE 0x100ULL

static uint64_t mt6895_gtawatch_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, MT6895_GTAWATCH_BASE + off, size);
    uint64_t v = 0;
    unsigned i;

    if (!p) {
        return 0;
    }
    for (i = 0; i < size; i++) {
        v |= (uint64_t)p[i] << (8 * i);
    }
    return v;
}

static void mt6895_gtawatch_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    Mt6895MachineState *s = opaque;
    ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
    uint8_t *p = mt6895_guest_ptr(s, MT6895_GTAWATCH_BASE + off, size);
    static int n;
    unsigned i;

    if (n < 24) {
        n++;
        fprintf(stderr, "MTK-GTAWATCH wr off=%#" HWADDR_PRIx " val=%#" PRIx64
                " pc=%#" PRIx64 " lr=%#" PRIx64 "\n",
                MT6895_GTAWATCH_BASE + off, val,
                cpu ? (uint64_t)cpu->env.pc : 0,
                cpu ? (uint64_t)(cpu->env.aarch64 ? cpu->env.xregs[30] : cpu->env.regs[14]) : 0);
    }
    if (!p) {
        return;
    }
    for (i = 0; i < size; i++) {
        p[i] = (val >> (8 * i)) & 0xff;
    }
}

static const MemoryRegionOps mt6895_gtawatch_ops = {
    .read = mt6895_gtawatch_read,
    .write = mt6895_gtawatch_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static const MemoryRegionOps mt6895_gwatch_ops = {
    .read = mt6895_gwatch_read,
    .write = mt6895_gwatch_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static const MemoryRegionOps mt6895_btread_ops = {
    .read = mt6895_btread_read,
    .write = mt6895_btread_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static const MemoryRegionOps mt6895_ptwatch_ops = {
    .read = mt6895_ptwatch_read,
    .write = mt6895_ptwatch_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static const MemoryRegionOps mt6895_watch_ops = {
    .read = mt6895_watch_read,
    .write = mt6895_watch_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/*
 * MTK_USBDEVTRACE=1: log accesses to the 0x11200000-0x11220000 USB window
 * that fall through to the peri-fill stub (unmodelled USB sub-blocks).
 * Companion of the same env in mtk_syscon.c / mtk_usb_glue.c.
 */
static void mt6895_usbdevtrace(char rw, uint64_t abs, unsigned size,
                               uint64_t val)
{
    static int on = -1;
    static int n;
    static int max;

    if (on < 0) {
        const char *m = getenv("MTK_USBDEVTRACE_MAX");

        on = getenv("MTK_USBDEVTRACE") != NULL;
        max = m ? atoi(m) : 6000;
    }
    if (!on ||
        !((abs >= 0x11200000ULL && abs < 0x11220000ULL) ||
          (abs >= 0x1c800000ULL && abs < 0x1c802000ULL) ||
          (abs >= 0x0c530000ULL && abs < 0x0c532000ULL))) {
        return;
    }
    if (n >= max) {
        if (n++ == max) {
            fprintf(stderr, "USBT ... (%d line limit reached)\n", max);
        }
        return;
    }
    n++;
    {
        CPUState *cs = qemu_get_cpu(0);

        fprintf(stderr, "USBT %c peri-fill+%#" PRIx64 " size=%u val=%#" PRIx64
                " pc=%#" PRIx64 "\n", rw, abs, size, val,
                (uint64_t)(cs ? cpu_env(cs)->pc : 0));
    }
}

/*
 * MTK_BL2PC=1: bounded PC sampler for the BL2_EXT->BL31 hand-off.
 *
 * The zero-injection boot stops after BL2_EXT frees "system_bl2-ext" with all
 * next images already staged, so the failure is a spin, a bad jump or an idle
 * wait at the transfer.  This diagnostic samples the PC at 1 kHz (virtual
 * clock) and keeps the last 256 PCs seen in 0x62F00000..0x64F00000 together
 * with LR/SP/DAIF; it dumps once when the same 64-byte PC window repeats for
 * more than 500 samples, and always dumps the last 32 samples at process exit.
 * The last BL2_EXT MMIO access (pa/value/read-write) is printed with either
 * dump, so "spinning on a missing register" is directly visible.  Diagnostic
 * only: without the env nothing is allocated and no timer runs.
 */
typedef struct MtkBl2PcSample {
    uint64_t pc, lr, sp, daif;
} MtkBl2PcSample;

#define MTK_BL2PC_RING 256
#define MTK_BL2PC_ANY 64
static struct {
    bool enabled;
    QEMUTimer *timer;
    unsigned head, count;
    uint64_t stuck_key;
    unsigned stuck;
    bool dumped;
    unsigned last_head, idle_ticks;
    bool dumped_noprogress;
    uint64_t vbar_last;
    unsigned vbar_changes;
    unsigned walker_path;
    bool blr_seen;
    uint64_t blr_pc, blr_x0, blr_x1, blr_x2, blr_x3, blr_x4, blr_x25;
    uint64_t blr_esr, blr_elr, blr_spsr, blr_vbar, blr_sctlr, blr_tcr;
    bool pc200_seen;
    unsigned pc200_count;
    bool stub_seen;
    uint64_t stub_pc, stub_x0, stub_x1, stub_x2, stub_x3, stub_x4, stub_x25;
    bool atf_seen, brom_seen, atf_left;
    uint64_t atf_first_ns, atf_last_ns, brom_first_ns, pc200_first_ns;
    uint64_t atf_left_ns, atf_left_last_pc;
    uint64_t atf_left_esr, atf_left_elr, atf_left_spsr;
    uint64_t atf_left_vbar, atf_left_sctlr, atf_left_tcr, atf_left_pc;
    uint64_t atf_hist_key[64];
    unsigned atf_hist_cnt[64];
    bool was_in_any, win2_seen, left_all, halt_seen;
    bool prev_halted;
    unsigned halt_transitions;
    uint64_t ticks_total, last_tick_ns, last_dump_ns, last_dump_ticks;
    unsigned cs_null;
    unsigned resume_count;
    uint64_t last_resume_ns, last_resume_pc, last_resume_irq, last_resume_delta;
    unsigned bl2_halt_count;
    unsigned dump_count;
    uint64_t last_progress_ns, last_report_ns;
    int64_t gt_at_halt[4], gt_at_resume[4];
    uint64_t win3_pc[64];
    unsigned win3_count;
    uint64_t escr, eelr3, evbar3, eesr3, evbar2, eelr2, eesr2, efar2, epstate;
    uint64_t fmt_pc, fscr, felr3, fvbar3, fesr3, fvbar2, felr2, fesr2, ffar2, fpstate;
    bool gz_seen, loop_seen;
    uint64_t gz_first_pc, gz_cur_el, gz_scr, gz_elr3, gz_pstate;
    uint64_t gz_vbar3, gz_esr3, loop_last_ns;
    uint64_t loop_x5, loop_x7, loop_x8;
    uint64_t win4_first_ns, win4_last_ns;
    unsigned win4_count;
    uint64_t win4_hist_key[32];
    unsigned win4_hist_cnt[32];
    bool smc_seen;
    unsigned smc_count;
    uint64_t smc_x0, smc_x1, smc_x2, smc_x3, smc_elr3, smc_esr3, smc_pstate;
    unsigned mmio70_count;
    uint64_t mmio70_pa[8], mmio70_val[8];
    unsigned mmio70_wr[8];
    unsigned resets;
    uint64_t any_pc[32];
    unsigned any_pc_count;
    unsigned atf_mmio_count;
    uint64_t atfmm_pa[8], atfmm_val[8];
    unsigned atfmm_wr[8];
    bool smcret_seen, out_seen;
    uint64_t win5_pc[64];
    unsigned win5_count, win5_el;
    uint64_t smcret_pc, smcret_x0, smcret_x1, smcret_x2, smcret_x3;
    uint64_t out_pc, out_elr3, out_esr3, out_pstate;
    uint64_t bl2_halt_first_ns, bl2_halt_last_ns, bl2_halt_last_pc;
    uint64_t last_halt_ns, last_halt_pc, last_halt_irq;
    uint64_t last_halt_esr, last_halt_elr, last_halt_spsr;
    uint64_t last_halt_vbar, last_halt_sctlr, last_halt_tcr;
    uint64_t win2_first_ns, win2_last_ns;
    uint64_t atf2_hist_key[64];
    unsigned atf2_hist_cnt[64];
    uint64_t left_all_ns, left_all_last_pc;
    uint64_t left_all_esr, left_all_elr, left_all_spsr;
    uint64_t left_all_vbar, left_all_sctlr, left_all_tcr;
    uint64_t halt_ns, halt_pc, halt_irq;
    uint64_t halt_esr, halt_elr, halt_spsr;
    uint64_t halt_vbar, halt_sctlr, halt_tcr;
    unsigned halt_count;
    bool have_atf_mmio;
    uint64_t atf_mmio_pa, atf_mmio_val;
    unsigned atf_mmio_write;
    bool have_mmio;
    uint64_t mmio_pa, mmio_val;
    unsigned mmio_write;
    MtkBl2PcSample ring[MTK_BL2PC_RING];
    MtkBl2PcSample any[MTK_BL2PC_ANY];
    unsigned any_head, any_count;
} mtk_bl2pc;

static bool mtk_bl2pc_pc_in_range(uint64_t pc)
{
    /*
     * BL2_EXT starts at physical 0x62f00000 but runs at the secure alias
     * 0xffff0000_62f00000 once its MMU is on (seen as pc=0xffff000062f4a3d4
     * in MTK-PCSTALL).  Accept both by comparing the low 48 bits.
     */
    uint64_t p = pc & 0x0000ffffffffffffULL;

    return p >= 0x62f00000ULL && p < 0x64f00000ULL;
}

static void mtk_bl2pc_memdump(const char *tag, uint64_t addr, unsigned len)
{
    uint8_t buf[64];
    unsigned i;

    if (len > sizeof(buf)) {
        len = sizeof(buf);
    }
    if (address_space_read(&address_space_memory, addr,
                           MEMTXATTRS_UNSPECIFIED, buf, len) != MEMTX_OK) {
        fprintf(stderr, "  mem %s @%016" PRIx64 " read-failed\n", tag, addr);
        return;
    }
    fprintf(stderr, "  mem %s @%016" PRIx64 ":", tag, addr);
    for (i = 0; i < len; i++) {
        fprintf(stderr, " %02x", buf[i]);
    }
    fprintf(stderr, "\n");
}

static void mtk_bl2pc_print(unsigned n, const char *why)
{
    unsigned i, start;

    if (!mtk_bl2pc.enabled || !mtk_bl2pc.count) {
        return;
    }
    if (n > mtk_bl2pc.count) {
        n = mtk_bl2pc.count;
    }
    fprintf(stderr, "MTK-BL2PC dump (%s): %u of %u samples\n",
            why, n, mtk_bl2pc.count);
    start = (mtk_bl2pc.head + MTK_BL2PC_RING - n) % MTK_BL2PC_RING;
    for (i = 0; i < n; i++) {
        MtkBl2PcSample *p = &mtk_bl2pc.ring[(start + i) % MTK_BL2PC_RING];

        fprintf(stderr,
                "  pc=%016" PRIx64 " lr=%016" PRIx64 " sp=%016" PRIx64
                " daif=%#" PRIx64 "\n", p->pc, p->lr, p->sp, p->daif);
    }
    if (mtk_bl2pc.have_mmio) {
        fprintf(stderr, "  last-bl2-mmio %c pa=%#" PRIx64 " val=%#" PRIx64
                "\n", mtk_bl2pc.mmio_write ? 'W' : 'R',
                mtk_bl2pc.mmio_pa, mtk_bl2pc.mmio_val);
    }
    {
        CPUState *cs = qemu_get_cpu(0);
        CPUARMState *env = cs ? cpu_env(cs) : NULL;

        if (env) {
            fprintf(stderr, "  now-el3 esr=%#" PRIx64 " elr=%016" PRIx64
                    " spsr=%#" PRIx64 " vbar=%016" PRIx64 " sctlr=%016"
                    PRIx64 " tcr=%016" PRIx64 " pc=%016" PRIx64 "\n",
                    env->cp15.esr_el[3], env->elr_el[3],
                    (uint64_t)(env->pstate | env->daif),
                    env->cp15.vbar_el[3], env->cp15.sctlr_el[3],
                    env->cp15.tcr_el[3], (uint64_t)env->pc);
            {
                CPUState *nc = qemu_get_cpu(0);
                fprintf(stderr, "  halted_now=%u cur_irq=%#" PRIx64 "\n",
                        nc ? (unsigned)nc->halted : 0,
                        nc ? (uint64_t)nc->interrupt_request : 0);
            }
        }
    }
    if (mtk_bl2pc.halt_seen) {
        fprintf(stderr, "  last-halt#%u at_ns=%" PRId64 " pc=%016" PRIx64
                " irq=%#" PRIx64 " esr=%#" PRIx64 " elr=%016" PRIx64
                " spsr=%#" PRIx64 " vbar=%016" PRIx64 " sctlr=%016" PRIx64
                " tcr=%016" PRIx64 "\n", mtk_bl2pc.halt_transitions,
                mtk_bl2pc.last_halt_ns, mtk_bl2pc.last_halt_pc,
                mtk_bl2pc.last_halt_irq, mtk_bl2pc.last_halt_esr,
                mtk_bl2pc.last_halt_elr, mtk_bl2pc.last_halt_spsr,
                mtk_bl2pc.last_halt_vbar, mtk_bl2pc.last_halt_sctlr,
                mtk_bl2pc.last_halt_tcr);
    }
    fprintf(stderr, "  markers pc200=%s(%u) stub=%s"
            " walker_path=%s\n",
            mtk_bl2pc.pc200_seen ? "yes" : "no", mtk_bl2pc.pc200_count,
            mtk_bl2pc.stub_seen ? "yes" : "no",
            mtk_bl2pc.walker_path == 1 ? "fail(returned0)" :
            mtk_bl2pc.walker_path == 2 ? "ok" : "unseen");
    if (mtk_bl2pc.stub_seen) {
        fprintf(stderr, "  stub pc=%016" PRIx64 " x0=%016" PRIx64
                " x1=%016" PRIx64 " x2=%016" PRIx64 " x3=%016" PRIx64
                " x4=%016" PRIx64 " x25=%016" PRIx64 "\n",
                mtk_bl2pc.stub_pc, mtk_bl2pc.stub_x0, mtk_bl2pc.stub_x1,
                mtk_bl2pc.stub_x2, mtk_bl2pc.stub_x3, mtk_bl2pc.stub_x4,
                mtk_bl2pc.stub_x25);
    }
    fprintf(stderr, "  el3 vbar=%016" PRIx64 " vbar_changes=%u"
            " walker_path=%s\n", mtk_bl2pc.vbar_last,
            mtk_bl2pc.vbar_changes,
            mtk_bl2pc.walker_path == 1 ? "fail(returned0)" :
            mtk_bl2pc.walker_path == 2 ? "ok" : "unseen");
    if (mtk_bl2pc.blr_seen) {
        fprintf(stderr, "  blr pc=%016" PRIx64 " x0=%016" PRIx64
                " x1=%016" PRIx64 " x2=%016" PRIx64 " x3=%016" PRIx64
                " x4=%016" PRIx64 " x25=%016" PRIx64 "\n",
                mtk_bl2pc.blr_pc, mtk_bl2pc.blr_x0, mtk_bl2pc.blr_x1,
                mtk_bl2pc.blr_x2, mtk_bl2pc.blr_x3, mtk_bl2pc.blr_x4,
                mtk_bl2pc.blr_x25);
        fprintf(stderr, "  blr-el3 esr=%#" PRIx64 " elr=%016" PRIx64
                " spsr=%#" PRIx64 " vbar=%016" PRIx64 " sctlr=%016" PRIx64
                " tcr=%016" PRIx64 "\n", mtk_bl2pc.blr_esr,
                mtk_bl2pc.blr_elr, mtk_bl2pc.blr_spsr, mtk_bl2pc.blr_vbar,
                mtk_bl2pc.blr_sctlr, mtk_bl2pc.blr_tcr);
    }
    fprintf(stderr, "  timeline atf_seen=%u first_ns=%" PRId64
            " last_ns=%" PRId64 " brom_seen=%u brom_first_ns=%" PRId64
            " pc200_first_ns=%" PRId64 "\n",
            mtk_bl2pc.atf_seen, mtk_bl2pc.atf_first_ns, mtk_bl2pc.atf_last_ns,
            mtk_bl2pc.brom_seen, mtk_bl2pc.brom_first_ns,
            mtk_bl2pc.pc200_first_ns);
    if (mtk_bl2pc.atf_left) {
        fprintf(stderr, "  atf-left at_ns=%" PRId64 " last_pc=%016" PRIx64
                " esr=%#" PRIx64 " elr=%016" PRIx64 " spsr=%#" PRIx64
                " vbar=%016" PRIx64 " sctlr=%016" PRIx64
                " tcr=%016" PRIx64 "\n", mtk_bl2pc.atf_left_ns,
                mtk_bl2pc.atf_left_last_pc, mtk_bl2pc.atf_left_esr,
                mtk_bl2pc.atf_left_elr, mtk_bl2pc.atf_left_spsr,
                mtk_bl2pc.atf_left_vbar, mtk_bl2pc.atf_left_sctlr,
                mtk_bl2pc.atf_left_tcr);
    }
    {
        uint64_t keys[64];
        unsigned cnts[64], hi, hj;

        memcpy(keys, mtk_bl2pc.atf_hist_key, sizeof(keys));
        memcpy(cnts, mtk_bl2pc.atf_hist_cnt, sizeof(cnts));
        for (hi = 0; hi < 64; hi++) {
            for (hj = hi + 1; hj < 64; hj++) {
                if (cnts[hj] > cnts[hi]) {
                    uint64_t tk = keys[hi]; unsigned tc = cnts[hi];
                    keys[hi] = keys[hj]; cnts[hi] = cnts[hj];
                    keys[hj] = tk; cnts[hj] = tc;
                }
            }
        }
        fprintf(stderr, "  atf-hist top:");
        for (hi = 0; hi < 8 && cnts[hi]; hi++) {
            fprintf(stderr, " %016" PRIx64 "x%u", keys[hi], cnts[hi]);
        }
        fprintf(stderr, "\n");
    }
    fprintf(stderr, "  timeline2 win2_seen=%u first_ns=%" PRId64
            " last_ns=%" PRId64 " left_all=%u left_ns=%" PRId64
            " left_pc=%016" PRIx64 "\n", mtk_bl2pc.win2_seen,
            mtk_bl2pc.win2_first_ns, mtk_bl2pc.win2_last_ns,
            mtk_bl2pc.left_all, mtk_bl2pc.left_all_ns,
            mtk_bl2pc.left_all_last_pc);
    if (mtk_bl2pc.left_all) {
        fprintf(stderr, "  left-all esr=%#" PRIx64 " elr=%016" PRIx64
                " spsr=%#" PRIx64 " vbar=%016" PRIx64 " sctlr=%016" PRIx64
                " tcr=%016" PRIx64 "\n", mtk_bl2pc.left_all_esr,
                mtk_bl2pc.left_all_elr, mtk_bl2pc.left_all_spsr,
                mtk_bl2pc.left_all_vbar, mtk_bl2pc.left_all_sctlr,
                mtk_bl2pc.left_all_tcr);
    }
    if (mtk_bl2pc.halt_seen) {
        fprintf(stderr, "  halt-seen at_ns=%" PRId64 " pc=%016" PRIx64
                " irq=%#" PRIx64 " count=%u\n", mtk_bl2pc.halt_ns,
                mtk_bl2pc.halt_pc, mtk_bl2pc.halt_irq,
                mtk_bl2pc.halt_count);
    }
    if (mtk_bl2pc.have_atf_mmio) {
        fprintf(stderr, "  last-atf-mmio %c pa=%#" PRIx64 " val=%#" PRIx64
                "\n", mtk_bl2pc.atf_mmio_write ? 'W' : 'R',
                mtk_bl2pc.atf_mmio_pa, mtk_bl2pc.atf_mmio_val);
    }
    {
        uint64_t keys2[64];
        unsigned cnts2[64], pi, pj;

        memcpy(keys2, mtk_bl2pc.atf2_hist_key, sizeof(keys2));
        memcpy(cnts2, mtk_bl2pc.atf2_hist_cnt, sizeof(cnts2));
        for (pi = 0; pi < 64; pi++) {
            for (pj = pi + 1; pj < 64; pj++) {
                if (cnts2[pj] > cnts2[pi]) {
                    uint64_t tk = keys2[pi]; unsigned tc = cnts2[pi];
                    keys2[pi] = keys2[pj]; cnts2[pi] = cnts2[pj];
                    keys2[pj] = tk; cnts2[pj] = tc;
                }
            }
        }
        fprintf(stderr, "  atf2-hist top:");
        for (pi = 0; pi < 8 && cnts2[pi]; pi++) {
            fprintf(stderr, " %016" PRIx64 "x%u", keys2[pi], cnts2[pi]);
        }
        fprintf(stderr, "\n");
    }
    {
        uint64_t nowp = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

        fprintf(stderr, "  ticks total=%" PRIu64 " cs_null=%u"
                " since_dump=%" PRIu64 " ns_since_dump=%" PRId64
                " last_tick_ns=%" PRId64 "\n", mtk_bl2pc.ticks_total,
                mtk_bl2pc.cs_null,
                mtk_bl2pc.ticks_total - mtk_bl2pc.last_dump_ticks,
                mtk_bl2pc.last_dump_ns ?
                    (int64_t)(nowp - mtk_bl2pc.last_dump_ns) : 0,
                (int64_t)mtk_bl2pc.last_tick_ns);
        mtk_bl2pc.last_dump_ns = nowp;
        mtk_bl2pc.last_dump_ticks = mtk_bl2pc.ticks_total;
    }
    if (mtk_bl2pc.resume_count) {
        fprintf(stderr, "  last-resume#%u at_ns=%" PRId64 " pc=%016" PRIx64
                " irq=%#" PRIx64 " delta_ns=%" PRId64 "\n",
                mtk_bl2pc.resume_count, mtk_bl2pc.last_resume_ns,
                mtk_bl2pc.last_resume_pc, mtk_bl2pc.last_resume_irq,
                mtk_bl2pc.last_resume_delta);
    }
    if (mtk_bl2pc.gz_seen) {
        fprintf(stderr, "  gz-enter pc=%016" PRIx64 " cur_el=%" PRIu64
                " scr=%#" PRIx64 " elr3=%016" PRIx64 " vbar3=%016" PRIx64
                " pstate=%#" PRIx64 " loop_seen=%u x5=%016" PRIx64
                " x7=%016" PRIx64 " x8=%016" PRIx64 "\n",
                mtk_bl2pc.gz_first_pc, mtk_bl2pc.gz_cur_el,
                mtk_bl2pc.gz_scr, mtk_bl2pc.gz_elr3, mtk_bl2pc.gz_vbar3,
                mtk_bl2pc.gz_pstate, (unsigned)mtk_bl2pc.loop_seen,
                mtk_bl2pc.loop_x5, mtk_bl2pc.loop_x7, mtk_bl2pc.loop_x8);
    }
    if (mtk_bl2pc.win4_count) {
        uint64_t k4[32];
        unsigned c4[32], i4, j4;

        memcpy(k4, mtk_bl2pc.win4_hist_key, sizeof(k4));
        memcpy(c4, mtk_bl2pc.win4_hist_cnt, sizeof(c4));
        for (i4 = 0; i4 < 32; i4++) {
            for (j4 = i4 + 1; j4 < 32; j4++) {
                if (c4[j4] > c4[i4]) {
                    uint64_t tk = k4[i4]; unsigned tc = c4[i4];
                    k4[i4] = k4[j4]; c4[i4] = c4[j4];
                    k4[j4] = tk; c4[j4] = tc;
                }
            }
        }
        fprintf(stderr, "  win4 first_ns=%" PRId64 " last_ns=%" PRId64
                " count=%u top:", mtk_bl2pc.win4_first_ns,
                mtk_bl2pc.win4_last_ns, mtk_bl2pc.win4_count);
        for (i4 = 0; i4 < 8 && c4[i4]; i4++) {
            fprintf(stderr, " %016" PRIx64 "x%u", k4[i4], c4[i4]);
        }
        fprintf(stderr, "\n");
    }
    if (mtk_bl2pc.smc_seen) {
        fprintf(stderr, "  smc x0=%#" PRIx64 " x1=%#" PRIx64 " x2=%#"
                PRIx64 " x3=%#" PRIx64 " elr3=%016" PRIx64
                " esr3=%#" PRIx64 " pstate=%#" PRIx64 "\n",
                mtk_bl2pc.smc_x0, mtk_bl2pc.smc_x1, mtk_bl2pc.smc_x2,
                mtk_bl2pc.smc_x3, mtk_bl2pc.smc_elr3, mtk_bl2pc.smc_esr3,
                mtk_bl2pc.smc_pstate);
    }
    fprintf(stderr, "  resets=%u mmio70=%u\n", mtk_bl2pc.resets,
            mtk_bl2pc.mmio70_count);
    if (mtk_bl2pc.atf_mmio_count) {
        unsigned km;

        fprintf(stderr, "  atf-mmio(%u):", mtk_bl2pc.atf_mmio_count);
        for (km = 0; km < mtk_bl2pc.atf_mmio_count; km++) {
            fprintf(stderr, " %c%#" PRIx64 "=%#" PRIx64,
                    mtk_bl2pc.atfmm_wr[km] ? 'W' : 'R',
                    mtk_bl2pc.atfmm_pa[km], mtk_bl2pc.atfmm_val[km]);
        }
        fprintf(stderr, "\n");
    }
    if (mtk_bl2pc.smcret_seen || mtk_bl2pc.out_seen) {
        fprintf(stderr, "  smc-ret_seen=%u x0=%#" PRIx64 " x1=%#" PRIx64
                " x2=%#" PRIx64 " x3=%#" PRIx64 " out_seen=%u out_pc=%016"
                PRIx64 " out_elr3=%016" PRIx64 " out_pstate=%#" PRIx64
                "\n", (unsigned)mtk_bl2pc.smcret_seen,
                mtk_bl2pc.smcret_x0, mtk_bl2pc.smcret_x1,
                mtk_bl2pc.smcret_x2, mtk_bl2pc.smcret_x3,
                (unsigned)mtk_bl2pc.out_seen, mtk_bl2pc.out_pc,
                mtk_bl2pc.out_elr3, mtk_bl2pc.out_pstate);
    }
    if (mtk_bl2pc.win5_count) {
        unsigned w5;

        fprintf(stderr, "  win5(%u) el=%u pcs:", mtk_bl2pc.win5_count,
                mtk_bl2pc.win5_el);
        for (w5 = 0; w5 < mtk_bl2pc.win5_count && w5 < 32; w5++) {
            fprintf(stderr, " %016" PRIx64, mtk_bl2pc.win5_pc[w5]);
        }
        fprintf(stderr, "\n");
        mtk_bl2pc_memdump("gz-phys", 0x7F800000ULL, 48);
        mtk_bl2pc_memdump("gz-alias", 0x87F800000ULL, 48);
    }
    if (mtk_bl2pc.win3_count) {
        unsigned wi;

        fprintf(stderr, "  win3-pcs(%u):", mtk_bl2pc.win3_count);
        for (wi = 0; wi < mtk_bl2pc.win3_count; wi++) {
            fprintf(stderr, " %016" PRIx64, mtk_bl2pc.win3_pc[wi]);
        }
        fprintf(stderr, "\n");
        mtk_bl2pc_memdump("gz-base", 0x70000000ULL, 48);
        mtk_bl2pc_memdump("gz-entry", 0x700000E0ULL, 48);
    }
    if (mtk_bl2pc.left_all) {
        mtk_bl2pc_memdump("left-pc", mtk_bl2pc.left_all_last_pc, 32);
    }
    if (mtk_bl2pc.pc200_first_ns) {
        mtk_bl2pc_memdump("fault-pc", mtk_bl2pc.fmt_pc, 32);
        fprintf(stderr, "  fault-el scr=%#" PRIx64 " elr3=%016" PRIx64
                " vbar3=%016" PRIx64 " esr3=%#" PRIx64
                " vbar2=%016" PRIx64 " elr2=%016" PRIx64
                " esr2=%#" PRIx64 " far2=%#" PRIx64 " pstate=%#" PRIx64
                "\n", mtk_bl2pc.fscr, mtk_bl2pc.felr3, mtk_bl2pc.fvbar3,
                mtk_bl2pc.fesr3, mtk_bl2pc.fvbar2, mtk_bl2pc.felr2,
                mtk_bl2pc.fesr2, mtk_bl2pc.ffar2, mtk_bl2pc.fpstate);
        fprintf(stderr, "  left-el scr=%#" PRIx64 " elr3=%016" PRIx64
                " vbar3=%016" PRIx64 " esr3=%#" PRIx64
                " vbar2=%016" PRIx64 " elr2=%016" PRIx64
                " esr2=%#" PRIx64 " far2=%#" PRIx64 " pstate=%#" PRIx64
                "\n", mtk_bl2pc.escr, mtk_bl2pc.eelr3, mtk_bl2pc.evbar3,
                mtk_bl2pc.eesr3, mtk_bl2pc.evbar2, mtk_bl2pc.eelr2,
                mtk_bl2pc.eesr2, mtk_bl2pc.efar2, mtk_bl2pc.epstate);
    }
    if (mtk_bl2pc.bl2_halt_count) {
        fprintf(stderr, "  bl2-halts count=%u first_ns=%" PRId64
                " last_ns=%" PRId64 " last_pc=%016" PRIx64 "\n",
                mtk_bl2pc.bl2_halt_count, mtk_bl2pc.bl2_halt_first_ns,
                mtk_bl2pc.bl2_halt_last_ns, mtk_bl2pc.bl2_halt_last_pc);
    }
}

static void mtk_bl2pc_exit_dump(void)
{
    mtk_bl2pc_print(32, "exit");
}

static void mtk_bl2pc_tick(void *opaque)
{
    CPUState *cs = qemu_get_cpu(0);
    CPUARMState *env;
    uint64_t pc, key;

    /*
     * Re-arm before any early return so a transient cs==NULL (or any other
     * early exit) can never stop the sampler.  The later timer_mod remains
     * as a second, harmless re-arm.
     */
    {
        mtk_bl2pc.ticks_total++;
        mtk_bl2pc.last_tick_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        mtk_bl2pc.any_pc[mtk_bl2pc.any_pc_count++ % 32] =
            (uint64_t)((ARM_CPU(cs)->env.pc) & 0x0000ffffffffffffULL);
        timer_mod(mtk_bl2pc.timer,
                  qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 100000);
    }
    if (!cs) {
        mtk_bl2pc.cs_null++;
        return;
    }
    env = cpu_env(cs);
    timer_mod(mtk_bl2pc.timer,
              qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 100000);
    pc = env->pc;
    {
        uint64_t lopc = pc & 0x0000ffffffffffffULL;
        uint64_t vb = env->cp15.vbar_el[3];

        if (vb != mtk_bl2pc.vbar_last) {
            mtk_bl2pc.vbar_last = vb;
            mtk_bl2pc.vbar_changes++;
        }
        if (lopc == 0x200ULL && !mtk_bl2pc.pc200_seen) {
            mtk_bl2pc.pc200_seen = true;
            mtk_bl2pc.pc200_count = mtk_bl2pc.any_count;
        }
        if (lopc >= 0x62f05c00ULL && lopc < 0x62f05d00ULL) {
            mtk_bl2pc.stub_seen = true;
            mtk_bl2pc.stub_pc = pc;
            mtk_bl2pc.stub_x0 = env->xregs[0];
            mtk_bl2pc.stub_x1 = env->xregs[1];
            mtk_bl2pc.stub_x2 = env->xregs[2];
            mtk_bl2pc.stub_x3 = env->xregs[3];
            mtk_bl2pc.stub_x4 = env->xregs[4];
            mtk_bl2pc.stub_x25 = env->xregs[25];
        }
        if (lopc >= 0x62f0c5b8ULL && lopc < 0x62f0c5dcULL) {
            mtk_bl2pc.walker_path = 1;
        } else if (lopc >= 0x62f0c5dcULL && lopc < 0x62f0c5f8ULL) {
            mtk_bl2pc.walker_path = 2;
        }
        if (lopc >= 0x62f0c610ULL && lopc <= 0x62f0c628ULL) {
            mtk_bl2pc.blr_seen = true;
            mtk_bl2pc.blr_pc = pc;
            mtk_bl2pc.blr_x0 = env->xregs[0];
            mtk_bl2pc.blr_x1 = env->xregs[1];
            mtk_bl2pc.blr_x2 = env->xregs[2];
            mtk_bl2pc.blr_x3 = env->xregs[3];
            mtk_bl2pc.blr_x4 = env->xregs[4];
            mtk_bl2pc.blr_x25 = env->xregs[25];
            mtk_bl2pc.blr_esr = env->cp15.esr_el[3];
            mtk_bl2pc.blr_elr = env->elr_el[3];
            mtk_bl2pc.blr_spsr = (env->pstate | env->daif);
            mtk_bl2pc.blr_vbar = vb;
            mtk_bl2pc.blr_sctlr = env->cp15.sctlr_el[3];
            mtk_bl2pc.blr_tcr = env->cp15.tcr_el[3];
        }
    }
    {
        uint64_t lopc = pc & 0x0000ffffffffffffULL;
        uint64_t now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        bool in_atf = lopc >= 0x48200000ULL && lopc < 0x48300000ULL;

        if (in_atf) {
            unsigned idx = (unsigned)(((lopc >> 4) * 2654435761u) & 63);

            if (!mtk_bl2pc.atf_seen) {
                mtk_bl2pc.atf_seen = true;
                mtk_bl2pc.atf_first_ns = now_ns;
            }
            mtk_bl2pc.atf_last_ns = now_ns;
            if (mtk_bl2pc.atf_hist_cnt[idx] == 0) {
                mtk_bl2pc.atf_hist_key[idx] = lopc;
            }
            mtk_bl2pc.atf_hist_cnt[idx]++;
        } else if (mtk_bl2pc.atf_seen && !mtk_bl2pc.atf_left &&
                   lopc < 0x48200000ULL) {
            mtk_bl2pc.atf_left = true;
            mtk_bl2pc.atf_left_ns = now_ns;
            mtk_bl2pc.atf_left_last_pc = lopc;
            mtk_bl2pc.atf_left_esr = env->cp15.esr_el[3];
            mtk_bl2pc.atf_left_elr = env->elr_el[3];
            mtk_bl2pc.atf_left_spsr = env->pstate | env->daif;
            mtk_bl2pc.atf_left_vbar = env->cp15.vbar_el[3];
            mtk_bl2pc.atf_left_sctlr = env->cp15.sctlr_el[3];
            mtk_bl2pc.atf_left_tcr = env->cp15.tcr_el[3];
            mtk_bl2pc.atf_left_pc = lopc;
        }
        if (lopc < 0x100000ULL && !mtk_bl2pc.brom_seen) {
            mtk_bl2pc.brom_seen = true;
            mtk_bl2pc.brom_first_ns = now_ns;
        }
        if (lopc == 0x200ULL && !mtk_bl2pc.pc200_first_ns) {
            mtk_bl2pc.pc200_first_ns = now_ns ? now_ns : 1;
        }
    }
    {
        uint64_t lopc = pc & 0x0000ffffffffffffULL;
        uint64_t now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        bool in1 = lopc >= 0x48200000ULL && lopc < 0x48300000ULL;
        bool in2 = lopc >= 0x48600000ULL && lopc < 0x48800000ULL;

        if (in2) {
            unsigned idx = (unsigned)(((lopc >> 4) * 2654435761u) & 63);

            if (!mtk_bl2pc.win2_seen) {
                mtk_bl2pc.win2_seen = true;
                mtk_bl2pc.win2_first_ns = now_ns;
            }
            mtk_bl2pc.win2_last_ns = now_ns;
            if (mtk_bl2pc.atf2_hist_cnt[idx] == 0) {
                mtk_bl2pc.atf2_hist_key[idx] = lopc;
            }
            mtk_bl2pc.atf2_hist_cnt[idx]++;
        }
        if (in1 || in2) {
            mtk_bl2pc.was_in_any = true;
        } else if (mtk_bl2pc.was_in_any && !mtk_bl2pc.left_all) {
            mtk_bl2pc.left_all = true;
            mtk_bl2pc.left_all_ns = now_ns;
            mtk_bl2pc.left_all_last_pc = lopc;
            mtk_bl2pc.left_all_esr = env->cp15.esr_el[3];
            mtk_bl2pc.left_all_elr = env->elr_el[3];
            mtk_bl2pc.left_all_spsr = env->pstate | env->daif;
            mtk_bl2pc.left_all_vbar = env->cp15.vbar_el[3];
            mtk_bl2pc.left_all_sctlr = env->cp15.sctlr_el[3];
            mtk_bl2pc.left_all_tcr = env->cp15.tcr_el[3];
        }
        if (cs->halted && !mtk_bl2pc.prev_halted) {
            unsigned ci;

            mtk_bl2pc.halt_transitions++;
            mtk_bl2pc.last_halt_ns = now_ns;
            mtk_bl2pc.last_halt_pc = lopc;
            mtk_bl2pc.last_halt_irq = cs->interrupt_request;
            mtk_bl2pc.last_halt_esr = env->cp15.esr_el[3];
            mtk_bl2pc.last_halt_elr = env->elr_el[3];
            mtk_bl2pc.last_halt_spsr = env->pstate | env->daif;
            mtk_bl2pc.last_halt_vbar = env->cp15.vbar_el[3];
            mtk_bl2pc.last_halt_sctlr = env->cp15.sctlr_el[3];
            mtk_bl2pc.last_halt_tcr = env->cp15.tcr_el[3];
            {
                ARMCPU *acpu = ARM_CPU(cs);
                unsigned gi;

                for (gi = 0; gi < NUM_GTIMERS && gi < 4; gi++) {
                    if (acpu->gt_timer[gi]) {
                        mtk_bl2pc.gt_at_halt[gi] =
                            timer_expire_time_ns(acpu->gt_timer[gi]) -
                            (int64_t)now_ns;
                    } else {
                        mtk_bl2pc.gt_at_halt[gi] = -1;
                    }
                }
            }
            if (lopc >= 0x62f00000ULL && lopc < 0x64f00000ULL) {
                if (!mtk_bl2pc.bl2_halt_count) {
                    mtk_bl2pc.bl2_halt_first_ns = now_ns;
                }
                mtk_bl2pc.bl2_halt_count++;
                mtk_bl2pc.bl2_halt_last_ns = now_ns;
                mtk_bl2pc.bl2_halt_last_pc = lopc;
            }
            if (!mtk_bl2pc.halt_seen) {
                mtk_bl2pc.halt_seen = true;
                mtk_bl2pc.halt_ns = mtk_bl2pc.last_halt_ns;
                mtk_bl2pc.halt_pc = mtk_bl2pc.last_halt_pc;
                mtk_bl2pc.halt_irq = mtk_bl2pc.last_halt_irq;
                mtk_bl2pc.halt_esr = mtk_bl2pc.last_halt_esr;
                mtk_bl2pc.halt_elr = mtk_bl2pc.last_halt_elr;
                mtk_bl2pc.halt_spsr = mtk_bl2pc.last_halt_spsr;
                mtk_bl2pc.halt_vbar = mtk_bl2pc.last_halt_vbar;
                mtk_bl2pc.halt_sctlr = mtk_bl2pc.last_halt_sctlr;
                mtk_bl2pc.halt_tcr = mtk_bl2pc.last_halt_tcr;
            }
            if (mtk_bl2pc.halt_transitions <= 8) {
                fprintf(stderr, "  HALT#%u at_ns=%" PRId64
                        " pc=%016" PRIx64 " cpu_halted=1 irq=%#" PRIx64
                        " esr=%#" PRIx64 " elr=%016" PRIx64
                        " spsr=%#" PRIx64 " vbar=%016" PRIx64
                        " sctlr=%016" PRIx64 " tcr=%016" PRIx64 "\n",
                        mtk_bl2pc.halt_transitions, now_ns, lopc,
                        mtk_bl2pc.last_halt_irq, mtk_bl2pc.last_halt_esr,
                        mtk_bl2pc.last_halt_elr, mtk_bl2pc.last_halt_spsr,
                        mtk_bl2pc.last_halt_vbar, mtk_bl2pc.last_halt_sctlr,
                        mtk_bl2pc.last_halt_tcr);
                fprintf(stderr, "    gt-halt ns_delta[0..3]=%lld,%lld,%lld,%lld\n",
                        (long long)mtk_bl2pc.gt_at_halt[0],
                        (long long)mtk_bl2pc.gt_at_halt[1],
                        (long long)mtk_bl2pc.gt_at_halt[2],
                        (long long)mtk_bl2pc.gt_at_halt[3]);
                for (ci = 0; ci < 8; ci++) {
                    CPUState *oc = qemu_get_cpu(ci);

                    if (!oc) {
                        break;
                    }
                    fprintf(stderr, "    cpu%u halted=%u pc=%016" PRIx64
                            " irq=%#" PRIx64 "\n", ci,
                            (unsigned)oc->halted,
                            (uint64_t)cpu_env(oc)->pc,
                            (uint64_t)oc->interrupt_request);
                }
            }
            {
                Mt6895MachineState *ms =
                    MT6895_MACHINE(OBJECT(qdev_get_machine()));
                GICv3State *g = (GICv3State *)ms->gic;

                if (g && g->cpu) {
                    for (ci = 0; ci < g->num_cpu && ci < 8; ci++) {
                        GICv3CPUState *gc = &g->cpu[ci];

                        fprintf(stderr,
                                "    gic%u hppi_irq=%d hppi_prio=%u"
                                " hppi_grp=%d level=%u"
                                " ienable=%08x ipend=%08x igroup=%08x"
                                " pend_en=%08x pmr=%#llx igrpen1=%#llx\n",
                                ci, gc->hppi.irq, (unsigned)gc->hppi.prio,
                                gc->hppi.grp, gc->level,
                                (unsigned)gc->gicr_ienabler0,
                                (unsigned)gc->gicr_ipendr0,
                                (unsigned)gc->gicr_igroupr0,
                                (unsigned)(gc->gicr_ipendr0 &
                                           gc->gicr_ienabler0),
                                (unsigned long long)gc->icc_pmr_el1,
                                (unsigned long long)gc->icc_igrpen[1]);
                    }
                }
            }
        }
        if (mtk_bl2pc.prev_halted && !cs->halted) {
            mtk_bl2pc.resume_count++;
            mtk_bl2pc.last_resume_ns = now_ns;
            mtk_bl2pc.last_resume_pc = lopc;
            mtk_bl2pc.last_resume_irq = cs->interrupt_request;
            mtk_bl2pc.last_resume_delta = (now_ns >= mtk_bl2pc.last_halt_ns)
                ? now_ns - mtk_bl2pc.last_halt_ns : 0;
            {
                ARMCPU *acpu = ARM_CPU(cs);
                unsigned gi;

                for (gi = 0; gi < NUM_GTIMERS && gi < 4; gi++) {
                    if (acpu->gt_timer[gi]) {
                        mtk_bl2pc.gt_at_resume[gi] =
                            timer_expire_time_ns(acpu->gt_timer[gi]) -
                            (int64_t)now_ns;
                    } else {
                        mtk_bl2pc.gt_at_resume[gi] = -1;
                    }
                }
            }
            if (mtk_bl2pc.resume_count <= 8) {
                fprintf(stderr, "  RESUME#%u at_ns=%" PRId64
                        " pc=%016" PRIx64 " irq=%#" PRIx64
                        " delta_ns=%" PRId64 " cause=%s\n",
                        mtk_bl2pc.resume_count, now_ns, lopc,
                        mtk_bl2pc.last_resume_irq,
                        mtk_bl2pc.last_resume_delta,
                        mtk_bl2pc.last_resume_irq ? "irq-pending" :
                        "no-irq (timer/other)");
                fprintf(stderr, "    gt-resume ns_delta[0..3]=%lld,%lld,%lld,%lld\n",
                        (long long)mtk_bl2pc.gt_at_resume[0],
                        (long long)mtk_bl2pc.gt_at_resume[1],
                        (long long)mtk_bl2pc.gt_at_resume[2],
                        (long long)mtk_bl2pc.gt_at_resume[3]);
            }
        }
        mtk_bl2pc.prev_halted = cs->halted;
    }
    {
        uint64_t lopc = pc & 0x0000ffffffffffffULL;
        uint64_t now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        bool in1 = lopc >= 0x48200000ULL && lopc < 0x48300000ULL;
        bool in2 = lopc >= 0x48600000ULL && lopc < 0x48800000ULL;
        bool in3 = lopc >= 0x70000000ULL && lopc < 0x71000000ULL;

        if (in3 && mtk_bl2pc.win3_count < 64) {
            mtk_bl2pc.win3_pc[mtk_bl2pc.win3_count++] = lopc;
        }
        if (in3 && !mtk_bl2pc.gz_seen) {
            mtk_bl2pc.gz_seen = true;
            mtk_bl2pc.gz_first_pc = lopc;
            mtk_bl2pc.gz_cur_el = env->pstate & 0xf;
            mtk_bl2pc.gz_scr = env->cp15.scr_el3;
            mtk_bl2pc.gz_elr3 = env->elr_el[3];
            mtk_bl2pc.gz_pstate = env->pstate | env->daif;
            mtk_bl2pc.gz_vbar3 = env->cp15.vbar_el[3];
            mtk_bl2pc.gz_esr3 = env->cp15.esr_el[3];
            fprintf(stderr, "  GZ-ENTER pc=%016" PRIx64 " cur_el=%" PRIu64
                    " scr=%#" PRIx64 " elr3=%016" PRIx64
                    " vbar3=%016" PRIx64 " esr3=%#" PRIx64
                    " pstate=%#" PRIx64 "\n", lopc, mtk_bl2pc.gz_cur_el,
                    mtk_bl2pc.gz_scr, mtk_bl2pc.gz_elr3, mtk_bl2pc.gz_vbar3,
                    mtk_bl2pc.gz_esr3, mtk_bl2pc.gz_pstate);
        }
        if (lopc >= 0x700000D8ULL && lopc <= 0x700000E4ULL) {
            if (!mtk_bl2pc.loop_seen) {
                mtk_bl2pc.loop_seen = true;
                mtk_bl2pc.loop_last_ns = now_ns;
                mtk_bl2pc.loop_x5 = env->xregs[5];
                mtk_bl2pc.loop_x7 = env->xregs[7];
                mtk_bl2pc.loop_x8 = env->xregs[8];
                fprintf(stderr, "  COPYLOOP-first pc=%016" PRIx64
                        " x5=%016" PRIx64 " x7=%016" PRIx64
                        " x8=%016" PRIx64 "\n", lopc, mtk_bl2pc.loop_x5,
                        mtk_bl2pc.loop_x7, mtk_bl2pc.loop_x8);
            } else if (now_ns - mtk_bl2pc.loop_last_ns >= 1000000000LL) {
                mtk_bl2pc.loop_last_ns = now_ns;
                mtk_bl2pc.loop_x5 = env->xregs[5];
                mtk_bl2pc.loop_x7 = env->xregs[7];
                mtk_bl2pc.loop_x8 = env->xregs[8];
                fprintf(stderr, "  COPYLOOP-1s at_ns=%" PRId64
                        " pc=%016" PRIx64 " x5=%016" PRIx64
                        " x7=%016" PRIx64 " x8=%016" PRIx64 "\n",
                        now_ns, lopc, mtk_bl2pc.loop_x5, mtk_bl2pc.loop_x7,
                        mtk_bl2pc.loop_x8);
            }
        }
        if (lopc >= 0x87F800000ULL && lopc < 0x880600000ULL) {
            if (!mtk_bl2pc.win5_count) {
                mtk_bl2pc.win5_el = env->pstate & 0xf;
            }
            if (mtk_bl2pc.win5_count < 64) {
                mtk_bl2pc.win5_pc[mtk_bl2pc.win5_count++] = lopc;
            }
        }
        if (lopc >= 0x71000000ULL && lopc < 0x74000000ULL) {
            unsigned idx = (unsigned)(((lopc >> 4) * 2654435761u) & 31);

            if (!mtk_bl2pc.win4_count) {
                mtk_bl2pc.win4_first_ns = now_ns;
            }
            mtk_bl2pc.win4_count++;
            mtk_bl2pc.win4_last_ns = now_ns;
            if (mtk_bl2pc.win4_hist_cnt[idx] == 0) {
                mtk_bl2pc.win4_hist_key[idx] = lopc;
            }
            mtk_bl2pc.win4_hist_cnt[idx]++;
        }
        if (mtk_bl2pc.gz_seen && in1 && !mtk_bl2pc.smc_seen) {
            mtk_bl2pc.smc_seen = true;
            mtk_bl2pc.smc_count++;
            mtk_bl2pc.smc_x0 = env->xregs[0];
            mtk_bl2pc.smc_x1 = env->xregs[1];
            mtk_bl2pc.smc_x2 = env->xregs[2];
            mtk_bl2pc.smc_x3 = env->xregs[3];
            mtk_bl2pc.smc_elr3 = env->elr_el[3];
            mtk_bl2pc.smc_esr3 = env->cp15.esr_el[3];
            mtk_bl2pc.smc_pstate = env->pstate | env->daif;
            fprintf(stderr, "  ATF-REENTRY pc=%016" PRIx64 " x0=%#" PRIx64
                    " x1=%#" PRIx64 " x2=%#" PRIx64 " x3=%#" PRIx64
                    " elr3=%016" PRIx64 " esr3=%#" PRIx64
                    " pstate=%#" PRIx64 "\n", lopc, mtk_bl2pc.smc_x0,
                    mtk_bl2pc.smc_x1, mtk_bl2pc.smc_x2, mtk_bl2pc.smc_x3,
                    mtk_bl2pc.smc_elr3, mtk_bl2pc.smc_esr3,
                    mtk_bl2pc.smc_pstate);
        }
        if (mtk_bl2pc.smc_seen && !mtk_bl2pc.smcret_seen &&
            lopc >= 0x71000000ULL && lopc < 0x74000000ULL) {
            mtk_bl2pc.smcret_seen = true;
            mtk_bl2pc.smcret_pc = lopc;
            mtk_bl2pc.smcret_x0 = env->xregs[0];
            mtk_bl2pc.smcret_x1 = env->xregs[1];
            mtk_bl2pc.smcret_x2 = env->xregs[2];
            mtk_bl2pc.smcret_x3 = env->xregs[3];
            fprintf(stderr, "  SMC-RET pc=%016" PRIx64 " x0=%#" PRIx64
                    " x1=%#" PRIx64 " x2=%#" PRIx64 " x3=%#" PRIx64
                    "\n", lopc, mtk_bl2pc.smcret_x0, mtk_bl2pc.smcret_x1,
                    mtk_bl2pc.smcret_x2, mtk_bl2pc.smcret_x3);
        }
        {
            bool known = in1 || in2 || in3 ||
                (lopc >= 0x62f00000ULL && lopc < 0x64f00000ULL) ||
                (lopc >= 0x48000000ULL && lopc < 0x49000000ULL) ||
                (lopc >= 0x71000000ULL && lopc < 0x74000000ULL);

            if (mtk_bl2pc.gz_seen && mtk_bl2pc.smc_seen && !known &&
                !mtk_bl2pc.out_seen && lopc != 0x200) {
                mtk_bl2pc.out_seen = true;
                mtk_bl2pc.out_pc = lopc;
                mtk_bl2pc.out_elr3 = env->elr_el[3];
                mtk_bl2pc.out_esr3 = env->cp15.esr_el[3];
                mtk_bl2pc.out_pstate = env->pstate | env->daif;
                fprintf(stderr, "  OUT-OF-WINDOW pc=%016" PRIx64
                        " elr3=%016" PRIx64 " esr3=%#" PRIx64
                        " pstate=%#" PRIx64 "\n", lopc,
                        mtk_bl2pc.out_elr3, mtk_bl2pc.out_esr3,
                        mtk_bl2pc.out_pstate);
            }
        }
        if (in1 || in2 || in3) {
            mtk_bl2pc.was_in_any = true;
        } else if (mtk_bl2pc.was_in_any && !mtk_bl2pc.left_all) {
            mtk_bl2pc.left_all = true;
            mtk_bl2pc.left_all_ns = now_ns;
            mtk_bl2pc.left_all_last_pc = lopc;
            mtk_bl2pc.left_all_esr = env->cp15.esr_el[3];
            mtk_bl2pc.left_all_elr = env->elr_el[3];
            mtk_bl2pc.left_all_spsr = env->pstate | env->daif;
            mtk_bl2pc.left_all_vbar = env->cp15.vbar_el[3];
            mtk_bl2pc.left_all_sctlr = env->cp15.sctlr_el[3];
            mtk_bl2pc.left_all_tcr = env->cp15.tcr_el[3];
            mtk_bl2pc.escr = env->cp15.scr_el3;
            mtk_bl2pc.eelr3 = env->elr_el[3];
            mtk_bl2pc.evbar3 = env->cp15.vbar_el[3];
            mtk_bl2pc.eesr3 = env->cp15.esr_el[3];
            mtk_bl2pc.evbar2 = env->cp15.vbar_el[2];
            mtk_bl2pc.eelr2 = env->elr_el[2];
            mtk_bl2pc.eesr2 = env->cp15.esr_el[2];
            mtk_bl2pc.efar2 = env->cp15.far_el[2];
            mtk_bl2pc.epstate = env->pstate | env->daif;
            fprintf(stderr, "  LEFT-ALL pc=%016" PRIx64 " scr=%#" PRIx64
                    " elr3=%016" PRIx64 " vbar3=%016" PRIx64
                    " esr3=%#" PRIx64 " vbar2=%016" PRIx64
                    " elr2=%016" PRIx64 " esr2=%#" PRIx64
                    " far2=%#" PRIx64 " pstate=%#" PRIx64 "\n", lopc,
                    mtk_bl2pc.escr, mtk_bl2pc.eelr3, mtk_bl2pc.evbar3,
                    mtk_bl2pc.eesr3, mtk_bl2pc.evbar2, mtk_bl2pc.eelr2,
                    mtk_bl2pc.eesr2, mtk_bl2pc.efar2, mtk_bl2pc.epstate);
        }
        if (lopc == 0x200ULL && !mtk_bl2pc.pc200_first_ns) {
            mtk_bl2pc.pc200_first_ns = now_ns ? now_ns : 1;
            mtk_bl2pc.fmt_pc = 0x200;
            mtk_bl2pc.fscr = env->cp15.scr_el3;
            mtk_bl2pc.felr3 = env->elr_el[3];
            mtk_bl2pc.fvbar3 = env->cp15.vbar_el[3];
            mtk_bl2pc.fesr3 = env->cp15.esr_el[3];
            mtk_bl2pc.fvbar2 = env->cp15.vbar_el[2];
            mtk_bl2pc.felr2 = env->elr_el[2];
            mtk_bl2pc.fesr2 = env->cp15.esr_el[2];
            mtk_bl2pc.ffar2 = env->cp15.far_el[2];
            mtk_bl2pc.fpstate = env->pstate | env->daif;
            fprintf(stderr, "  PC200 scr=%#" PRIx64 " elr3=%016" PRIx64
                    " vbar3=%016" PRIx64 " esr3=%#" PRIx64
                    " vbar2=%016" PRIx64 " elr2=%016" PRIx64
                    " esr2=%#" PRIx64 " far2=%#" PRIx64 " pstate=%#"
                    PRIx64 "\n", mtk_bl2pc.fscr, mtk_bl2pc.felr3,
                    mtk_bl2pc.fvbar3, mtk_bl2pc.fesr3, mtk_bl2pc.fvbar2,
                    mtk_bl2pc.felr2, mtk_bl2pc.fesr2, mtk_bl2pc.ffar2,
                    mtk_bl2pc.fpstate);
        }
    }
    mtk_bl2pc.any[mtk_bl2pc.any_head] = (MtkBl2PcSample){ pc, env->xregs[30],
                                                        env->xregs[31],
                                                        env->daif };
    mtk_bl2pc.any_head = (mtk_bl2pc.any_head + 1) % MTK_BL2PC_ANY;
    if (mtk_bl2pc.any_count < MTK_BL2PC_ANY) {
        mtk_bl2pc.any_count++;
    }
    if (mtk_bl2pc_pc_in_range(pc)) {
        mtk_bl2pc.ring[mtk_bl2pc.head] = (MtkBl2PcSample){ pc, env->xregs[30],
                                                          env->xregs[31],
                                                          env->daif };
        mtk_bl2pc.head = (mtk_bl2pc.head + 1) % MTK_BL2PC_RING;
        if (mtk_bl2pc.count < MTK_BL2PC_RING) {
            mtk_bl2pc.count++;
        }
    }
    /*
     * No-progress watchdog: if BL2_EXT stops executing for 2 s (bad jump to
     * an address outside its range, or an idle/wait it never returns from),
     * dump before the harness kills QEMU -- the atexit dump would not run on
     * a forced kill.  Labelled separately from the stall dump.
     */
    {
        int64_t now_r = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

        if (mtk_bl2pc.head == mtk_bl2pc.last_head) {
            mtk_bl2pc.idle_ticks++;
        } else {
            mtk_bl2pc.last_head = mtk_bl2pc.head;
            mtk_bl2pc.idle_ticks = 0;
            mtk_bl2pc.last_progress_ns = (uint64_t)now_r;
        }
        /*
         * Time-based reporting: every 2 s of virtual time, at most 4 dumps;
         * labelled no-progress only when no sample arrived for 2 s.  The
         * atexit dump remains an unconditional fallback.
         */
        if (mtk_bl2pc.count && mtk_bl2pc.dump_count < 4 && now_r > 0 &&
            now_r - (int64_t)mtk_bl2pc.last_report_ns >= 2000000000LL) {
            mtk_bl2pc.dump_count++;
            mtk_bl2pc.last_report_ns = (uint64_t)now_r;
            mtk_bl2pc_print(32, (mtk_bl2pc.last_progress_ns &&
                                now_r - (int64_t)mtk_bl2pc.last_progress_ns >=
                                2000000000LL) ? "no-progress" : "periodic");
            fprintf(stderr, "  current pc=%016" PRIx64 " lr=%016" PRIx64
                    " sp=%016" PRIx64 " daif=%#" PRIx64 "\n", pc,
                    env->xregs[30], env->xregs[31], env->daif);
            if (mtk_bl2pc.any_count) {
                unsigned k, st = (mtk_bl2pc.any_head + MTK_BL2PC_ANY -
                                  mtk_bl2pc.any_count) % MTK_BL2PC_ANY;

                for (k = 0; k < mtk_bl2pc.any_count; k++) {
                    MtkBl2PcSample *a =
                        &mtk_bl2pc.any[(st + k) % MTK_BL2PC_ANY];

                    fprintf(stderr, "  after[%u] pc=%016" PRIx64
                            " lr=%016" PRIx64 " sp=%016" PRIx64
                            " daif=%#" PRIx64 "\n", k, a->pc, a->lr,
                            a->sp, a->daif);
                }
            }
        }
    }
    key = (pc & 0x0000ffffffffffffULL) & ~0x3fULL;
    if (key == mtk_bl2pc.stuck_key) {
        mtk_bl2pc.stuck++;
    } else {
        mtk_bl2pc.stuck_key = key;
        mtk_bl2pc.stuck = 1;
    }
    if (mtk_bl2pc.stuck == 500 && !mtk_bl2pc.dumped) {
        mtk_bl2pc.dumped = true;
        mtk_bl2pc_print(64, "stall");
    }
}

static void mtk_bl2pc_mmio(hwaddr pa, uint64_t val, bool is_write)
{
    CPUState *cs;

    if (!mtk_bl2pc.enabled) {
        return;
    }
    cs = qemu_get_cpu(0);
    if (!cs || !mtk_bl2pc_pc_in_range(cpu_env(cs)->pc)) {
        return;
    }
    mtk_bl2pc.have_mmio = true;
    mtk_bl2pc.mmio_pa = pa;
    mtk_bl2pc.mmio_val = val;
    mtk_bl2pc.mmio_write = is_write;
}

static void mtk_bl2pc_atf_mmio(hwaddr pa, uint64_t val, bool is_write)
{
    CPUState *cs;
    uint64_t lopc;

    if (!mtk_bl2pc.enabled) {
        return;
    }
    cs = qemu_get_cpu(0);
    if (!cs) {
        return;
    }
    lopc = cpu_env(cs)->pc & 0x0000ffffffffffffULL;
    if ((lopc >= 0x48200000ULL && lopc < 0x48300000ULL) ||
        (lopc >= 0x48600000ULL && lopc < 0x48800000ULL) ||
        (lopc >= 0x70000000ULL && lopc < 0x74000000ULL) ||
        (lopc >= 0x7F800000ULL && lopc < 0x800600000ULL) ||
        (lopc >= 0x87F800000ULL && lopc < 0x880600000ULL)) {
        mtk_bl2pc.have_atf_mmio = true;
        mtk_bl2pc.atf_mmio_pa = pa;
        mtk_bl2pc.atf_mmio_val = val;
        mtk_bl2pc.atf_mmio_write = is_write;
        if (((lopc >= 0x48200000ULL && lopc < 0x48300000ULL) ||
             (lopc >= 0x48600000ULL && lopc < 0x48800000ULL)) &&
            mtk_bl2pc.atf_mmio_count < 8) {
            unsigned k = mtk_bl2pc.atf_mmio_count++;

            mtk_bl2pc.atfmm_pa[k] = pa;
            mtk_bl2pc.atfmm_val[k] = val;
            mtk_bl2pc.atfmm_wr[k] = is_write;
            fprintf(stderr, "  ATF-MMIO pc=%016" PRIx64 " %c pa=%#"
                    PRIx64 " val=%#" PRIx64 "\n", lopc,
                    is_write ? 'W' : 'R', pa, val);
        }
        if (lopc >= 0x70000000ULL && lopc < 0x74000000ULL &&
            mtk_bl2pc.mmio70_count < 8) {
            unsigned k = mtk_bl2pc.mmio70_count++;

            mtk_bl2pc.mmio70_pa[k] = pa;
            mtk_bl2pc.mmio70_val[k] = val;
            mtk_bl2pc.mmio70_wr[k] = is_write;
            fprintf(stderr, "  MMIO70 pc=%016" PRIx64 " %c pa=%#" PRIx64
                    " val=%#" PRIx64 "\n", lopc, is_write ? 'W' : 'R',
                    pa, val);
        }
    }
}

static void mtk_bl2pc_reset(void *opaque)
{
    (void)opaque;
    mtk_bl2pc.resets++;
    if (mtk_bl2pc.resets <= 4) {
        uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        CPUState *cs = qemu_get_cpu(0);
        CPUARMState *env = cs ? cpu_env(cs) : NULL;
        unsigned k, st = (mtk_bl2pc.any_pc_count > 32) ?
            (mtk_bl2pc.any_pc_count & 31) : 0;

        fprintf(stderr, "  RESET#%u at_ns=%" PRId64 " pc=%016" PRIx64
                " elr3=%016" PRIx64 " esr3=%#" PRIx64 " scr=%#" PRIx64
                " pstate=%#" PRIx64 "\n", mtk_bl2pc.resets, now,
                env ? (uint64_t)env->pc : 0,
                env ? env->elr_el[3] : 0,
                env ? env->cp15.esr_el[3] : 0,
                env ? env->cp15.scr_el3 : 0,
                env ? (env->pstate | env->daif) : 0);
        fprintf(stderr, "    RESET-last-pcs:");
        for (k = 0; k < 32 && k < mtk_bl2pc.any_pc_count; k++) {
            fprintf(stderr, " %016" PRIx64,
                    mtk_bl2pc.any_pc[(st + k) & 31]);
        }
        fprintf(stderr, "\n");
    }
}

static void mtk_bl2pc_init(void)
{
    if (mtk_bl2pc.enabled || !MTK_ENV_CACHED("MTK_BL2PC")) {
        return;
    }
    mtk_bl2pc.enabled = true;
    qemu_register_reset(mtk_bl2pc_reset, NULL);
    mtk_bl2pc.timer = timer_new_ns(QEMU_CLOCK_REALTIME, mtk_bl2pc_tick, NULL);
    timer_mod(mtk_bl2pc.timer,
              qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 100000);
    atexit(mtk_bl2pc_exit_dump);
    fprintf(stderr, "mt6895: MTK_BL2PC sampler enabled "
            "(BL2_EXT 0x62f00000..0x64efffff)\n");
}

static uint64_t mt6895_peri_fill_read(void *opaque, hwaddr off, unsigned size)
{
    (void)opaque;
    (void)size;
    mt6895_usbdevtrace('R', 0x10000000ULL + off, size, 0);
    mtk_bl2pc_mmio(0x10000000ULL + off, 0, false);
    mtk_bl2pc_atf_mmio(0x10000000ULL + off, 0, false);
    if (MTK_ENV_CACHED("MTK_SPIN2")) {
        static hwaddr last;
        static unsigned long cnt;

        if (off == last) {
            cnt++;
            if (cnt == 20000 || cnt == 100000) {
                fprintf(stderr, "SPIN-PERI off=%#lx x%lu\n",
                        (unsigned long)off, cnt);
            }
        } else {
            last = off;
            cnt = 1;
        }
    }
    if (MTK_ENV_CACHED("MTK_MFGTRACE") &&
        (0x10000000ULL + off) >= 0x1c1f00ULL &&
        (0x10000000ULL + off) < 0x1c2000ULL) {
        CPUState *cs = qemu_get_cpu(0);

        fprintf(stderr, "MFGTRACE R %#lx pc=%#lx\n",
                (unsigned long)(0x10000000ULL + off),
                (unsigned long)(cs ? cpu_env(cs)->pc : 0));
    }
    if (MTK_ENV_CACHED("MTK_PERITRACE") && off >= 0x350000 && off < 0x360000) {
        fprintf(stderr, "MTK-PERI read addr=%#lx\n",
                (unsigned long)(0x10000000ULL + off));
    }
    /*
     * MTK_BOOTDEVTRACE: the preloader decides which boot device exists by
     * reading a bit out of this window (the id -> word/bit mapping lives in
     * 0x0203026c).  Trace the reads that land near it so the register the
     * firmware expects can be identified and modelled precisely.
     */
    if (MTK_ENV_CACHED("MTK_BOOTDEVTRACE") && off >= 0x40 && off < 0xc0) {
        CPUState *cs = qemu_get_cpu(0);

        fprintf(stderr, "BOOTDEVTRACE R addr=%#lx pc=%#lx size=%u\n",
                (unsigned long)(0x10000000ULL + off),
                (unsigned long)(cs ? cpu_env(cs)->pc : 0), size);
    }
    if (MTK_ENV_CACHED("MTK_PERI_ONES")) {
        return 0xffffffffULL;
    }
    if (MTK_ENV_CACHED("MTK_POLLTRACE")) {
        static hwaddr last;
        static unsigned rep;
        static unsigned nprinted;

        if (off == last) {
            rep++;
            if (rep == 5000 && nprinted < 40) {
                nprinted++;
                fprintf(stderr, "POLLTRACE %#lx x%u\n",
                        (unsigned long)(0x10000000ULL + off), rep);
            }
        } else {
            last = off;
            rep = 0;
        }
    }
    return 0;
}

static void mt6895_peri_fill_write(void *opaque, hwaddr off, uint64_t val,
                                   unsigned size)
{
    (void)opaque; (void)size;
    mt6895_usbdevtrace('W', 0x10000000ULL + off, size, val);
    mtk_bl2pc_mmio(0x10000000ULL + off, val, true);
    mtk_bl2pc_atf_mmio(0x10000000ULL + off, val, true);
    if (MTK_ENV_CACHED("MTK_PERI_WTRACE")) {
        static int budget;

        if (budget < 100000) {
            budget++;
            fprintf(stderr, "MTK-PERI wr addr=%#" PRIx64 " val=%#" PRIx64
                    " size=%u\n", (uint64_t)(0x10000000ULL + off),
                    (uint64_t)val, size);
        }
    }
    if (MTK_ENV_CACHED("MTK_MFGTRACE") &&
        (0x10000000ULL + off) >= 0x1c1f00ULL &&
        (0x10000000ULL + off) < 0x1c2000ULL) {
        CPUState *cs = qemu_get_cpu(0);

        fprintf(stderr, "MFGTRACE W %#lx val=%#lx pc=%#lx\n",
                (unsigned long)(0x10000000ULL + off), (unsigned long)val,
                (unsigned long)(cs ? cpu_env(cs)->pc : 0));
    }
    if (MTK_ENV_CACHED("MTK_ADSPTRACE")) {
        uint64_t a = 0x10000000ULL + off;

        if (a >= 0x1e000000ULL && a < 0x1f020000ULL) {
            fprintf(stderr, "ADSPTRACE W %#lx val=%#lx\n",
                    (unsigned long)a, (unsigned long)val);
        }
    }
}

/*
 * SPM power-status window: LK's clkmgr power-on flow polls PWR_STATUS and
 * PWR_STATUS_2ND until the expected domains report "on".  Report all power
 * domains as already on so the flow completes instead of spinning.
 */
/* MTCMOS pwron step-0 handshake register (infracfg_ao + 0x5c). */
static uint64_t mt6895_infra_ready_read(void *opaque, hwaddr off, unsigned size)
{
    (void)opaque; (void)off; (void)size;
    if (MTK_ENV_CACHED("MTK_PWRTRACE")) {
        fprintf(stderr, "MTK-PWR read 0x10001c5c\n");
    }
    return 0x00000200ULL;      /* bit 9: rail ready */
}

static void mt6895_infra_ready_write(void *opaque, hwaddr off, uint64_t val,
                                     unsigned size)
{
    (void)opaque; (void)off; (void)val; (void)size;
}

static const MemoryRegionOps mt6895_infra_ready_ops = {
    .read = mt6895_infra_ready_read,
    .write = mt6895_infra_ready_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/*
 * VLP power-domain register block (0x1c001e00).
 *
 * The clkmgr power flows drive a control word and spin on the matching
 * acknowledgement in the status word at +0x134:
 *
 *   pdn   : [+0x00] |= 1 ;  wait until ([+0x134] & 1)
 *   pwron :           ;  skip when ([+0x134] & 1) is clear
 *
 * The block is a plain register file on hardware, so keep the written state
 * and derive the acknowledgement from the control word instead of inventing
 * a constant: that keeps the preloader (which never raises the request) and
 * the LK flows (which do) both consistent with the same state.
 */
static uint32_t mt6895_vlp_pwr_regs[0x400];

static uint64_t mt6895_vlp_pwr_read(void *opaque, hwaddr off, unsigned size)
{
    uint32_t v;

    (void)opaque; (void)size;
    if (MTK_ENV_CACHED("MTK_VLPDBG") && (off == 0x134 || off == 0x138)) {
        fprintf(stderr, "VLPDBG R off=%#lx\n", (unsigned long)off);
    }
    v = mt6895_vlp_pwr_regs[(off >> 2) & 0x3ff];
    if (MTK_ENV_CACHED("MTK_VLP70") && off == 0x70) {
        static int n;

        if (n < 20) {
            n++;
            fprintf(stderr, "VLP70-R v=%#x\n", v);
        }
    }
    if (MTK_ENV_CACHED("MTK_VLP0") && off == 0x134) {
        fprintf(stderr, "VLP0-R134 regs0=%#x ret=%#x\n",
                mt6895_vlp_pwr_regs[0],
                (unsigned)((v & ~1u) | (mt6895_vlp_pwr_regs[0] & 1u)));
    }
    if (off == 0x134) {
        /*
         * Power-status word.  bit0 follows the power-down request (the pdn
         * flow clears it); bit27 tracks the GPU/MFG0 domain, which the same
         * pdn/pwron service owns: it reads "powered" while the domain was
         * never powered down and clears as soon as the request is raised, so
         * the LK power-down flow can actually complete instead of spinning on
         * a status that never changes.
         */
        uint32_t req = mt6895_vlp_pwr_regs[0];

        v = (v & ~1u) | (req & 1u);
        if (!(req & 1u)) {
            v |= 0x08000000u;
        } else {
            v &= ~0x08000000u;
        }
        return v;
    }
    if (off == 0x13c) {
        /*
         * GPU domain power status.  LK's gpueb loader refuses to touch the
         * partition until bit 1 (MFG0) reports powered, and the preloader has
         * already brought the GPU domains up by the time LK runs -- the same
         * convention the other power-status words in this file follow.
         */
        return v | 0x2u;
    }
    if (off == 0x138) {
        /*
         * Per-domain power status.  The clkmgr flow drives the control word
         * at +0x18 (bits 2/3 = acknowledge-clear / request-clear) and then
         * waits for bit 6 here to report the domain as settled.  Derive the
         * status from that request instead of hard-coding it, so a domain
         * that was never driven still reads as not-settled.
         */
        if (mt6895_vlp_pwr_regs[0x18 >> 2] & 0xcu) {
            v |= 0x40u;
        }
        return v;
    }
    if (off == 0xb4) {
        /*
         * MCUPM bring-up handshake.  The preloader sets bit 9 here and polls
         * bit 13, then sets bit 6 and polls bit 14; there is no MCUPM core
         * behind this window, so report both phases complete.
         */
        return v | 0xc0006000u;
    }
    /*
     * Bits 30/31 of a power-control word are the "state reached"
     * acknowledgement.  A flow raises the bits it wants to see by writing a
     * control word that carries the low request bits, and clears them by
     * writing the word back without bit 0; follow that written state rather
     * than a global flag so both the power-on and power-down sequences can
     * reach the condition they wait for.
     */
    if (v & 0x1f) {
        v |= 0xc0000000u;
    }
    return v;
}

static void mt6895_vlp_pwr_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    (void)opaque; (void)size;
    if (MTK_ENV_CACHED("MTK_VLP0") && off == 0) {
        fprintf(stderr, "VLP0-W val=%#lx size=%u\n",
                (unsigned long)val, size);
    }
    if (MTK_ENV_CACHED("MTK_VLPTRACE")) {
        fprintf(stderr, "VLPTRACE W off=%#04lx val=%#lx\n",
                (unsigned long)off, (unsigned long)val);
    }
    mt6895_vlp_pwr_regs[(off >> 2) & 0x3ff] = (uint32_t)val;
}

static const MemoryRegionOps mt6895_vlp_pwr_ops = {
    .read = mt6895_vlp_pwr_read,
    .write = mt6895_vlp_pwr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/*
 * ADSP core handshake window (0x1e980000).
 *
 * ATF's adsp_bl_handler drives a per-core mailbox and waits for the core to
 * acknowledge:
 *
 *   write +0x108 = 1   ->  +0x10c bit1  (loader accepted)
 *   write +0x100 = 1   ->  +0x100 bit0  (loader finished)
 *   write +0x104 = 0   ->  release
 *
 * There is no ADSP firmware running behind this window, so the handshake is
 * modelled as a register file whose status bits follow the triggers the
 * handler writes, rather than as constants.
 */

;

;

static const MemoryRegionOps mt6895_peri_fill_ops = {
    .read = mt6895_peri_fill_read,
    .write = mt6895_peri_fill_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static const MemoryRegionOps mt6895_topckgen_ops = {
    .read = mt6895_topckgen_read,
    .write = mt6895_topckgen_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * A minimal but self-consistent GPT header, built for the partition parse to
 * read through the block-device object.
 *
 * Fields follow the UEFI specification; the two CRC32 values are computed the
 * way the specification requires -- the header CRC over the 92 header bytes
 * with its own field zeroed, and the entry-array CRC over the (all-zero) array
 * that follows.
 */
static void mt6895_build_gpt_header(uint8_t *gpt)
{
    const uint32_t entries = 128;
    const uint32_t entry_size = 128;
    uint32_t crc;
    uint8_t *array = gpt + MT6895_GPT_HEADER_SIZE;

    memset(gpt, 0, MT6895_GPT_HEADER_SIZE + entries * entry_size);

    memcpy(gpt, "EFI PART", 8);                       /* signature        */
    stl_le_p(gpt + 8, 0x00010000);                    /* revision 1.0     */
    stl_le_p(gpt + 12, MT6895_GPT_HEADER_SIZE);       /* header size = 92 */
    stl_le_p(gpt + 20, 0);                            /* reserved         */
    stq_le_p(gpt + 24, MT6895_GPT_CURRENT_LBA);       /* current LBA      */
    stq_le_p(gpt + 32, 0);                            /* backup LBA       */
    stq_le_p(gpt + 40, 2);                            /* first usable LBA */
    stq_le_p(gpt + 48, 0);                            /* last usable LBA  */
    stq_le_p(gpt + 72, 1);                            /* entry array LBA  */
    stl_le_p(gpt + 80, entries);
    stl_le_p(gpt + 84, entry_size);

    crc = crc32(0, array, entries * entry_size);
    stl_le_p(gpt + 88, crc);                          /* array CRC        */

    crc = crc32(0, gpt, MT6895_GPT_HEADER_SIZE);
    stl_le_p(gpt + 16, crc);                          /* header CRC       */
}

/*
 * Place the device's GPT where the payload reads it.
 *
 * The payload's block-I/O callback is handed a buffer that two independent
 * runs measured as 0x00102180, and a read-only probe showed that buffer is
 * empty when the callback runs -- so the data has to be there beforehand.
 * The file is the device's real GPT, taken verbatim from the storage image
 * (header plus 128 partition entries), which keeps both CRC32 fields correct
 * by construction rather than by recomputation.
 *
 * The header goes to the image start and the entry array to the LBA-2 offset
 * (2 * 512), because that is where the header's own "entry array LBA" field
 * says it lives. The callback copies both into the buffer it is handed.
 */
/*
 * Place a data image in DRAM and point the block-I/O object at it.
 *
 * The object's +0x14 field is what 0x0200D026 reads as the source of the
 * device's bytes, so that is where the machine has to put partition content.
 */
/*
 * Populate the DRAM stage's name table.
 *
 * The payload looks "aee_debug_kinfo" up in a table at 0x02078EA4 before it
 * falls back to searching the GPT through the RKP entry. That table is empty
 * in the image, so the lookup always fails; giving it one valid entry lets the
 * first path succeed.
 */
/*
 * Inject the real device tree for LK.
 *
 * The preloader builds the mblock table in the pl-boottag block but leaves
 * the "lk_main_dtb" entry out (its DTB load never happens in this model), so
 * LK's platform_load_device_tree() returns -19 and it panics with
 * "lk driver fdt is NULL!". The table layout was measured on the running
 * machine:
 *   entry base 0x48601050, stride 0x98, +0x10 start, +0x18 size, +0x24 name
 *   (verified against the aee_debug_kinfo entry), 23 entries used.
 * So the next free slot is 0x48601D40. Copy the DTB out of the vendor_boot_b
 * partition (first LBA 0x368800, DTB at +0x28C5040, size 0x4FE8C) into DRAM
 * and add the entry.
 */
#define MTK_DTB_DISK_OFF   0x37b66450ULL   /* lk_a region: FDT payload (found by magic scan) */
#define MTK_DTB_SIZE       0x4fe8cULL
#define MTK_DTB_LOAD_ADDR  0x511656b0ULL   /* LK image embedded FDT */
#define MTK_MB_ENTRY_ADDR  0x4857d4a8ULL   /* LK's own mblock table */
#define MTK_MB_ENTRY_ADDR2 0x486010e8ULL   /* preloader copy, for reference */

/*
 * Debug helper: locate every copy of the mblock table by its "aee_lk" name.
 * LK prints mblock entries but does not read the copy the preloader builds at
 * 0x48601050, so it must use another copy; this scan finds it.
 */
/*
 * Read-only overlay for one slot of LK's mblock table.
 *
 * LK prints mblock entries from the copy at 0x4857d410 (found by scanning for
 * the "aee_lk" name), not from the preloader's copy at 0x48601050. That table
 * has no "lk_main_dtb" entry, so platform_load_device_tree() fails with -19
 * and LK panics with "lk driver fdt is NULL!". A timer-based injection was
 * racy (the table is rebuilt), so serve the slot from a synthetic entry that
 * cannot be overwritten:
 *   +0x10 start, +0x18 size, +0x24 name   (layout measured on aee_lk entry)
 */
/* Entry count of LK's mblock table (0x51004F98 + 0x08). */
;


;

/*
 * Inject the lk_main_dtb mblock entry the moment LK publishes its table.
 *
 * LK builds the mblock table in its own BSS (base 0x51004F98, magic
 * 0x99999999) and copies it from the boot args before it prints/looks
 * anything up, so timer-based injection always lands too late. Watching the
 * entry-count field (base+8) lets us append the entry right after LK writes
 * the count.
 */
#define MTK_MB_BASE   0x51004f98ULL
#define MTK_MB_COUNT  (MTK_MB_BASE + 8)

;

/*
 * Diagnostic: find where the firmware leaves the overlaid device tree.
 *
 * BL2 reports "after overlay, fdt size(408549)" but LK still ends up with a
 * NULL fdt, so the model needs to know where BL2 actually parked the tree.
 * Scanning from inside the model is reliable (mt6895_guest_ptr() is the same
 * path the preloader stub uses), unlike QMP xp/pmemsave on this build.
 */
/*
 * Read-only diagnostic: dump the mblock tables LK uses.
 *
 * The mblock allocator hands out memory inside the regions listed in the
 * firmware's main array, so seeing both arrays is the only way to tell why a
 * request (e.g. crypto_hw at 0x48401000) cannot be satisfied.
 */
/*
 * Write watch on LK's crypto-initialised flag (0x510a1b54).
 *
 * LK's crypto_hw_tfa init skips the mblock allocation when this byte is 1,
 * which is what real hardware ends up with; logging who writes it (or that
 * nobody does) tells us which firmware step our model is missing.
 */
static uint64_t mt6895_cflag_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, 0x510a1b54ULL + off, size);
    uint64_t v = 0;
    unsigned i;

    if (!p) {
        return 0;
    }
    for (i = 0; i < size; i++) {
        v |= (uint64_t)p[i] << (8 * i);
    }
    return v;
}

static void mt6895_cflag_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, 0x510a1b54ULL + off, size);
    static int n;
    unsigned i;

    if (n < 12) {
        n++;
        fprintf(stderr, "MTK-CFLAG-WR off=%#x val=%#" PRIx64 " size=%u pc=%#"
                PRIx64 "\n", (unsigned)off, val, size,
                current_cpu ? (uint64_t)ARM_CPU(current_cpu)->env.regs[15] : 0);
    }
    if (!p) {
        return;
    }
    for (i = 0; i < size; i++) {
        p[i] = (val >> (8 * i)) & 0xff;
    }
}

static const MemoryRegionOps mt6895_cflag_ops = {
    .read = mt6895_cflag_read,
    .write = mt6895_cflag_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * Write watch on ATF's SMC owner table (0x4825b67c).
 *
 * ATF routes a SiP call by looking up owner[fnid >> 24 & 0x3f | bit31 << 6];
 * the crypto service only works if its registration filled owner[66]. Logging
 * the writes shows whether that registration ran at all.
 */
static uint64_t mt6895_own_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, 0x4825b67cULL + off, size);
    uint64_t v = 0;
    unsigned i;

    if (!p) {
        return 0;
    }
    for (i = 0; i < size; i++) {
        v |= (uint64_t)p[i] << (8 * i);
    }
    return v;
}

static void mt6895_own_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *p = mt6895_guest_ptr(s, 0x4825b67cULL + off, size);
    static int n;
    unsigned i;

    if (n < 40) {
        n++;
        fprintf(stderr, "MTK-OWN-WR off=%#x val=%#" PRIx64 " size=%u pc=%#"
                PRIx64 "\n", (unsigned)off, val, size,
                current_cpu ? (uint64_t)ARM_CPU(current_cpu)->env.regs[15] : 0);
    }
    if (!p) {
        return;
    }
    for (i = 0; i < size; i++) {
        p[i] = (val >> (8 * i)) & 0xff;
    }
}

static const MemoryRegionOps mt6895_own_ops = {
    .read = mt6895_own_read,
    .write = mt6895_own_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * The scan timer only exists to feed the boot-chain diagnostics implemented
 * below.  When none of them is requested, skip the timer entirely instead of
 * dumping the ATF/mblock state every 200 ms of guest time for the whole boot.
 */
static bool mt6895_scan_diag_wanted(void)
{
    return MTK_ENV_CACHED("MTK_TICK") || MTK_ENV_CACHED("MTK_MBDUMP") ||
           MTK_ENV_CACHED("MTK_NAMETBL_DUMP") ||
           MTK_ENV_CACHED("MTK_CALMARK") || MTK_ENV_CACHED("MTK_GTA_FIX") ||
           MTK_ENV_CACHED("MTK_CRYPTOBUF") || MTK_ENV_CACHED("MTK_PARTLIST") ||
           MTK_ENV_CACHED("MTK_MBSCAN") || MTK_ENV_CACHED("MTK_ATFSCAN");
}

static void mt6895_clear_crypto_mb(void *opaque)
{
    Mt6895MachineState *s = opaque;
    uint8_t *t;
    unsigned k;

    if (MTK_ENV_CACHED("MTK_TICK")) {
        static int ntk;

        if (ntk < 20) {
            ntk++;
            fprintf(stderr, "MTK-TICK %d vclock=%" PRId64 "\n", ntk,
                    qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        }
    }

    /*
     * DIAGNOSTIC: dump the preloader's mblock table so the crypto_hw
     * allocation can be traced.  The preloader builds it in the pl-boottag
     * block: entry base 0x48601050, stride 0x98, +0x10 start, +0x18 size,
     * +0x24 name pointer; the count lives at base-0x40+0x08 style slots and
     * the table ends at the first entry whose size is zero.
     */
    if (MTK_ENV_CACHED("MTK_MBDUMP")) {
        static int nmb;
        uint8_t *base = mt6895_guest_ptr(s, 0x48601050ULL, 0x98 * 32);

        if (base && nmb < 3) {
            unsigned ei;

            nmb++;
            fprintf(stderr, "MTK-MB[%d]:\n", nmb);
            for (ei = 0; ei < 32; ei++) {
                uint8_t *e = base + ei * 0x98;
                uint64_t st = ldq_le_p(e + 0x10);
                uint64_t sz = ldq_le_p(e + 0x18);
                uint64_t np = ldq_le_p(e + 0x24);
                char nm[24] = { 0 };
                uint8_t *npp;

                if (!sz) {
                    break;
                }
                npp = np ? mt6895_guest_ptr(s, np, sizeof(nm) - 1) : NULL;
                if (npp) {
                    memcpy(nm, npp, sizeof(nm) - 1);
                }
                fprintf(stderr, "  [%2u] %010" PRIx64 " +%012" PRIx64
                        "  %s\n", ei, st, sz, nm);
            }
        }
    }
    /*
     * DIAGNOSTIC (MTK_NAMETBL_DUMP): dump the preloader's partition name
     * table.  The DRAM stage keeps it at 0x02078ea4 with a 20-byte stride
     * {size, size2, ..., name pointer}; if the table is empty the BL2_EXT
     * side never receives any partition names either.
     */
    if (MTK_ENV_CACHED("MTK_NAMETBL_DUMP")) {
        static int nnt;
        uint8_t *tbl;
        unsigned ti;

        tbl = mt6895_guest_ptr(s, 0x02078ea4ULL, 20 * 16);
        if (tbl && nnt < 3) {
            nnt++;
            fprintf(stderr, "MTK-NAMETBL @0x02078ea4:\n");
            for (ti = 0; ti < 16; ti++) {
                uint8_t *e = tbl + ti * 20;
                uint32_t sz = ldl_le_p(e);
                uint32_t np = ldl_le_p(e + 16);
                char nm[24] = { 0 };
                uint8_t *npp = np ? mt6895_guest_ptr(s, np, 23) : NULL;

                if (!sz && !np) {
                    break;
                }
                if (npp) {
                    memcpy(nm, npp, 23);
                }
                fprintf(stderr, "  [%2u] sz=%#x nameptr=%#x name='%s'\n",
                        ti, sz, np, nm);
            }
        }
    }
    /*
     * DIAGNOSTIC (MTK_PARTLIST): dump BL2_EXT's block-device list.  The boot
     * loader keeps a circular list of partition nodes at 0x62fca250; each
     * node carries {next +0x08, refcount +0x10, size +0x20, name +0x40,
     * read +0x68}.  The secure-config read fails with 0x100008 when a name
     * lookup finds nothing, so seeing whether the list is populated -- and
     * what names it holds -- pins down the layer that is missing.
     */
    /*
     * DIAGNOSTIC (MTK_CALMARK): the DRAM calibration "initialised" marker.
     * The preloader clears its BSS at start-up, so a value planted by the
     * machine before the run can be erased before the check at 0x02034B86;
     * watching it over time shows whether that is what happens.
     */
    if (MTK_ENV_CACHED("MTK_CALMARK")) {
        static int ncm;
        uint8_t *m = mt6895_guest_ptr(s, 0x02075dc8ULL, 4);

        if (m && ncm < 6) {
            ncm++;
            fprintf(stderr, "MTK-CALMARK @0x02075dc8 = %#x\n", ldl_le_p(m));
        }
    }
    if (MTK_ENV_CACHED("MTK_PARTLIST")) {
        static int npl;
        uint8_t *head;
        uint64_t first;
        unsigned ni;

        head = mt6895_guest_ptr(s, 0x62fca250ULL, 0x18);
        if (head && npl < 4) {
            unsigned wi;

            npl++;
            fprintf(stderr, "MTK-PARTLIST raw @0x62fca240:\n");
            for (wi = 0; wi < 8; wi++) {
                uint8_t *w = mt6895_guest_ptr(s, 0x62fca240ULL + wi * 8, 8);

                fprintf(stderr, "  +%03x: %016" PRIx64 "\n", 0x240 + wi * 8,
                        w ? ldq_le_p(w) : 0);
            }
            {
                uint8_t *alias = mt6895_guest_ptr(s, 0xffff000062fca250ULL, 8);

                fprintf(stderr, "MTK-PARTLIST alias 0xffff000062fca250 -> %s"
                        " (direct 0x62fca250 -> %s)\n",
                        alias ? "MAPPED" : "unmapped", head ? "MAPPED" : "unmapped");
            }
            first = ldq_le_p(head + 8);
            fprintf(stderr, "MTK-PARTLIST head=%#" PRIx64 " first=%#" PRIx64
                    "\n", 0x62fca250ULL, first);
            for (ni = 0; ni < 24 && first; ni++) {
                uint64_t flat = (first >= MT6895_DRAM_ALIAS_BASE) ?
                                first - MT6895_DRAM_ALIAS_BASE : first;
                uint8_t *node = mt6895_guest_ptr(s, first, 0x80);
                uint64_t next, sz, np, rd;

                if (flat == 0x62fca250ULL) {
                    fprintf(stderr, "  (head reached after %u nodes)\n", ni);
                    break;
                }
                if (!node) {
                    fprintf(stderr, "  [%2u] node=%#" PRIx64 " UNMAPPED\n",
                            ni, first);
                    break;
                }
                next = ldq_le_p(node + 0x08);
                sz = ldq_le_p(node + 0x20);
                np = ldq_le_p(node + 0x40);
                rd = ldq_le_p(node + 0x68);
                {
                    char nm[32] = { 0 };
                    uint8_t *npp = np ? mt6895_guest_ptr(s, np, 31) : NULL;

                    if (npp) {
                        memcpy(nm, npp, 31);
                    }
                    fprintf(stderr, "  [%2u] node=%#" PRIx64 " sz=%#" PRIx64
                            " rd=%#" PRIx64 " name='%s'\n", ni, first, sz, rd, nm);
                }
                first = next;
            }
        }
    }
    /*
     * DIAGNOSTIC: locate the preloader's mblock MAIN array (the allocatable
     * regions the allocator hands memory out of).  Its first entry is
     * {start 0x40000000, size 0x08080000}, so scan the pl-boottag window and
     * the SRAM BSS for that pair.
     */
    if (MTK_ENV_CACHED("MTK_MBSCAN")) {
        static int nsc;
        static const struct { uint64_t base; uint32_t len; const char *what; }
        areas[] = {
            { 0x48600000ULL, 0x40000, "pl-boottag" },
            { 0x020f0000ULL, 0x10000, "sram-bss" },
        };
        unsigned ai;

        if (nsc < 2) {
            unsigned mi;

            nsc++;
            for (ai = 0; ai < ARRAY_SIZE(areas); ai++) {
                uint8_t *p = mt6895_guest_ptr(s, areas[ai].base,
                                              areas[ai].len);
                uint32_t o;

                if (!p) {
                    continue;
                }
                for (o = 0; o + 24 <= areas[ai].len; o += 8) {
                    if (ldq_le_p(p + o) == 0x40000000ULL &&
                        ldq_le_p(p + o + 8) == 0x08080000ULL) {
                        fprintf(stderr, "MTK-MBSCAN %s+0x%x: main array @ %#"
                                PRIx64 "\n", areas[ai].what, o,
                                areas[ai].base + o);
                    }
                }
            }
            {
                uint8_t *mp = mt6895_guest_ptr(s, 0x486000c0ULL, 16 * 24);

                fprintf(stderr, "MTK-MBMAIN ptr=%p\n", (void *)mp);
                for (mi = 0; mp && mi < 16; mi++) {
                    uint64_t st = ldq_le_p(mp + mi * 24);
                    uint64_t sz = ldq_le_p(mp + mi * 24 + 8);

                    fprintf(stderr, "MTK-MBMAIN[%2u] %010" PRIx64 " +%012"
                            PRIx64 "  end=%010" PRIx64 "\n",
                            mi, st, sz, st + sz);
                }
            }
        }
    }


    /*
     * DIAGNOSTIC: watch ATF's crypto shared buffer. LK hashes the seccfg
     * through it (SMC 0xc200010e/0xf), so the digest it validates against
     * appears here; logging changes lets us capture the exact value the
     * hardware crypto engine produces for a given input.
     */
    {
        static uint8_t last[0x40];
        static int ncb;
        uint8_t *cb = mt6895_guest_ptr(s, 0x48401000ULL, 0x40);

        if (mtk_ufs_lk_stage && cb && memcmp(cb, last, 0x40)) {
            if (ncb < 12) {
                unsigned bi;

                ncb++;
                fprintf(stderr, "MTK-CRYPTOBUF:");
                for (bi = 0; bi < 0x40; bi++) {
                    fprintf(stderr, "%02x", cb[bi]);
                }
                fprintf(stderr, "\n");
            }
            memcpy(last, cb, 0x40);
        }
    }
    /*
     * Restore the BL2 -> ATF platform tag area (GTA at 0x4825ad1c).
     *
     * BL2 publishes its TZ/secmem tags (magic "TAG") in the tee-secmem
     * window at 0x7c800200; the same tag list is what ATF's xlat context
     * setup reads through its GTA pointer. ATF clears its BSS with DC ZVA,
     * which bypasses memory regions, so the copy BL2 made into ATF's BSS is
     * lost and the xlat setup then reads an empty tag list and derives
     * bogus table-area parameters. Re-apply the published tags (idempotent)
     * so ATF sees the platform configuration the real BL2 hands over.
     */
    if (MTK_ENV_CACHED("MTK_GTA_FIX")) {
        uint8_t *src = mt6895_guest_ptr(s, 0x7c800200ULL, 0x100);
        uint8_t *dst = mt6895_guest_ptr(s, 0x4825ad1cULL, 0x100);

        if (src && dst && ldq_le_p(src) != 0 &&
            memcmp(dst, src, 0x40) != 0) {
            static int nfix;

            memcpy(dst, src, 0x100);
            if (nfix < 6) {
                nfix++;
                fprintf(stderr, "MTK-GTAFIX copied %d bytes (#%d)\n", 0x100,
                        nfix);
            }
        }
    }
    /* Re-arm: keep watching for the rest of the boot. */
    timer_mod(s->scan_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 200000000LL);

    /*
     * Full ATF/mblock state dump.  This used to run on every 200 ms of guest
     * time for the whole boot, putting several megabytes on stderr with no
     * guest-visible effect.  MTK_ATFSCAN=1 restores it.
     */
    if (MTK_ENV_CACHED("MTK_ATFSCAN")) {
    {
        uint8_t *a = mt6895_guest_ptr(s, 0x7c800000ULL, 0x20);
        uint8_t *d = mt6895_guest_ptr(s, 0x7c800800ULL, 0x58);

        if (a && d) {
            fprintf(stderr, "MTK-ATFARG atfarg+0x18=%016" PRIx64
                    " desc+0x10=%016" PRIx64 "\n",
                    ldq_le_p(a + 0x18), ldq_le_p(d + 0x10));
        }
    }
    {
        /* Locate ATF's console log and print it verbatim. */
        uint8_t *base = mt6895_guest_ptr(s, 0x48240000ULL, 0x20000);
        unsigned ak;

        if (base) {
            for (ak = 0; ak + 8 < 0x20000; ak++) {
                if (!memcmp(base + ak, "NOTICE", 6) ||
                    !memcmp(base + ak, "WARNING", 7) ||
                    !memcmp(base + ak, "ERROR", 5) ||
                    !memcmp(base + ak, "INFO", 4)) {
                    unsigned j = ak;

                    fprintf(stderr, "MTK-ATFLOG[%#x]: ", ak);
                    while (j < 0x20000 && j < ak + 0x120) {
                        unsigned char c = base[j];

                        if (c == 0) {
                            break;
                        }
                        fputc((c >= 0x20 && c < 0x7f) ? c : '.', stderr);
                        j++;
                    }
                    fprintf(stderr, "\n");
                    ak = j;
                }
            }
        }
    }
    {
        uint8_t *lg = mt6895_guest_ptr(s, 0x48253930ULL, 0x400);

        if (lg) {
            char line[0x100];
            unsigned bk;

            fprintf(stderr, "MTK-ATFLOG:");
            for (bk = 0; bk < 0x400; bk++) {
                unsigned char c = lg[bk];

                if (c == 0) {
                    break;
                }
                fputc((c >= 0x20 && c < 0x7f) ? c : '.', stderr);
            }
            fprintf(stderr, "\n");
            (void)line;
        }
    }
    {
        uint8_t *own = mt6895_guest_ptr(s, 0x4825b67cULL + 66, 4);
        uint8_t *ht = mt6895_guest_ptr(s, 0x4823eee0ULL, 0x40);

        if (own && ht) {
            {
            uint8_t *fl = mt6895_guest_ptr(s, 0x4825abd8ULL, 4);
            uint8_t *fl2 = mt6895_guest_ptr(s, 0x48245000ULL + 0xf08, 4);

            uint8_t *fp = mt6895_guest_ptr(s, 0x48250f40ULL, 8);

            uint8_t *g2 = mt6895_guest_ptr(s, 0x4825a8ccULL, 4);

            fprintf(stderr, "MTK-ATF-FLAGS 0x4825abd8=%#x 0x48245f08=%#x fp=%#" PRIx64 " g8cc=%#x\n",
                    fl ? ldl_le_p(fl) : 0xdead, fl2 ? ldl_le_p(fl2) : 0xdead,
                    fp ? ldq_le_p(fp) : 0xdeadbeefULL,
                    g2 ? ldl_le_p(g2) : 0xdead);
        }
        fprintf(stderr, "MTK-ATF-OWN owner[66]=%#x handler0=%016" PRIx64
                    " handler1=%016" PRIx64 "\n", own[0],
                    ldq_le_p(ht), ldq_le_p(ht + 0x20));
        }
    }
    {
        uint8_t *fl = mt6895_guest_ptr(s, 0x510a1b54ULL, 4);

        fprintf(stderr, "MTK-CRYPTOFLAG 0x510a1b54 = %#x\n",
                fl ? ldl_le_p(fl) : 0xffffffffu);
    }
    /* Reserved (named) array: base, count at +8, entries stride 0x98. */
    t = mt6895_guest_ptr(s, 0x51004f98ULL, 0x10);
    if (t) {
        fprintf(stderr, "MTK-MB rsv cnt=%u\n", ldl_le_p(t + 8));
    }
    t = mt6895_guest_ptr(s, 0x51004f98ULL + 0x10, 24 * 0x98);
    if (t) {
        for (k = 0; k < 24; k++) {
            uint8_t *e = t + k * 0x98;
            uint64_t st = ldq_le_p(e);
            uint64_t sz = ldq_le_p(e + 8);

            if (!st && !sz) {
                continue;
            }
            fprintf(stderr, "MTK-MB rsv[%u] start=%#" PRIx64 " size=%#" PRIx64
                    " name=%.12s\n", k, st, sz, (char *)e + 0x14);
        }
    }
    /* Main array: entries of {start, size, pad} at 0x51004398, stride 0x18. */
    t = mt6895_guest_ptr(s, 0x51004398ULL, 9 * 0x18);
    if (t) {
        for (k = 0; k < 9; k++) {
            uint8_t *e = t + k * 0x18;

            fprintf(stderr, "MTK-MB main[%u] start=%#" PRIx64 " size=%#" PRIx64
                    "\n", k, ldq_le_p(e), ldq_le_p(e + 8));
        }
    }
    } /* MTK_ATFSCAN */
    {
        static int pass;
        static const int64_t at[] = { 7000000000LL, 9000000000LL,
                                      10800000000LL, 11300000000LL,
                                      11600000000LL };

        pass++;
        if (pass < ARRAY_SIZE(at)) {
            timer_mod(s->scan_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      (at[pass] - at[pass - 1]));
        }
    }
}

/*
 * Append lk_main_dtb to the mblock table the preloader/BL2 leaves in DRAM
 * (base 0x4857CCE0: count at +8, entries at +0x10, stride 0x98, name at +0x24)
 * before LK copies it. LK then resolves the device tree through its normal
 * mboot_get_mem() path.
 */
#define MTK_BA_BASE   0x4857ccf0ULL   /* magic 0x99999999 found here */
#define MTK_BA_COUNT  (MTK_BA_BASE + 8)

/*
 * Positional read.  MinGW has no pread(), so seek and read on Windows; the
 * model reads the DTB once during machine init, so a shared file position is
 * fine here.
 */
static ssize_t mt6895_pread(int fd, void *buf, size_t len, off_t off)
{
#ifdef _WIN32
    {
        static int n;

        if (_lseeki64(fd, (int64_t)off, SEEK_SET) < 0) {
            int e = errno;

            if (n < 8) {
                n++;
                fprintf(stderr, "MT6895-PREAD seek FAIL off=%#" PRIx64
                        " errno=%d\n", (uint64_t)off, e);
            }
            return -1;
        }
        {
            int r = _read(fd, buf, (unsigned)len);

            if (n < 8) {
                n++;
                fprintf(stderr, "MT6895-PREAD off=%#" PRIx64 " len=%zu"
                        " ret=%d errno=%d\n", (uint64_t)off, len, r,
                        r < 0 ? errno : 0);
            }
            return r;
        }
    }
#else
    return pread(fd, buf, len, off);
#endif
}

/* Positional write, the mirror of mt6895_pread().  Used to persist guest
 * writes into the image when MTK_LU2_RW is enabled. */
static ssize_t mt6895_pwrite(int fd, const void *buf, size_t len, uint64_t off)
{
#ifdef _WIN32
    if (_lseeki64(fd, (int64_t)off, SEEK_SET) < 0) {
        return -1;
    }
    return _write(fd, buf, (unsigned)len);
#else
    return pwrite(fd, buf, len, off);
#endif
}

/*
 * The data image is NOT copied into guest RAM any more.
 *
 * It used to be streamed in 8 MiB chunks (8.3 GB for a real device dump),
 * which cost tens of seconds of machine-init time and pinned gigabytes of
 * host RSS, and forced the machine to be launched with -m 12G purely so the
 * image would fit above 0x80000000. The UFS model already serves the same
 * file through pread(data-path), and the only remaining consumer is the
 * preloader's block-read stub, which computes "0x80000000 + byte_offset" in
 * 32-bit arithmetic and therefore cannot address past 2 GiB anyway.
 *
 * So expose just that first 2 GiB as a read-only, pread-backed window over
 * the image file. Guest reads are served from the host page cache on
 * demand; nothing is materialised at reset.
 */
/*
 * The data window is plain guest RAM, but only 2 GiB of it.
 *
 * Two constraints meet here. The preloader's block-read stub forms its
 * source address as "0x80000000 + byte_offset" in 32-bit arithmetic, so it
 * can never address past 2 GiB -- anything beyond that is unreachable
 * through it. And the window must be RAM, not an IO region: the stub copies
 * byte by byte, and an MMIO access costs ~20 us in TCG, which turned a
 * 1.8 MB partition load into ~40 s.
 *
 * So back the window with real RAM and fill only the first 2 GiB from the
 * image file, skipping all-zero chunks. That is 1-2 s of page-cache I/O
 * instead of the ~8.3 GB the old whole-image copy moved, and every guest
 * read afterwards is an inline RAM access.
 */
/*
 * The data window: served on demand from the image file, never copied.
 *
 * Copying even 1 GiB into guest RAM at reset cost ~10 s of machine init
 * (the first GiB of the image is fully dense, so there is nothing sparse to
 * skip). Instead this is an IO region over the file with a 1 MiB read cache
 * and a 4 KiB copy-on-write overlay for the guest's own writes: the
 * preloader uses the same address both to read device bytes and as its
 * bounce buffer, and it clears that buffer at start-up.
 *
 * The read path is only touched every 24 bytes now (the block-read stub
 * moves data with ldm/stm), so the per-access MMIO cost that made an earlier
 * IO-based attempt slower than the copy no longer applies.
 */
#define MT6895_DATA_CACHE_BITS  20
#define MT6895_DATA_CACHE_SIZE  (1u << MT6895_DATA_CACHE_BITS)
#define MT6895_DATA_PAGE_BITS   12
#define MT6895_DATA_PAGE_SIZE   (1u << MT6895_DATA_PAGE_BITS)

typedef struct Mt6895DataPage {
    uint64_t idx;
    uint8_t data[MT6895_DATA_PAGE_SIZE];
} Mt6895DataPage;

static uint64_t mt6895_data_win_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t buf[8];
    {
        static int nwin;

        if (MTK_ENV_CACHED("MTK_STUBRD") &&
            ((off >= 0x8000ULL && off < 0x10000ULL) ||
             (off >= 0x63900000ULL && off < 0x63a00000ULL)) && nwin < 20) {
            nwin++;
            fprintf(stderr, "MTK-STUBRD off=%#" PRIx64 " size=%u\n", off, size);
        }
    }
    Mt6895DataPage *page;
    uint64_t pidx;
    unsigned i;

    /*
     * MTK_MEMINFO: the preloader's "mem" device reads 11 bytes from offset 0
     * of its block device to obtain the BootROM-provided DRAM description.
     * On hardware the BootROM owns that device; here the read lands on the
     * data window, whose offset 0 holds the image's protective MBR (zeros),
     * so the payload reports "dram info unset" and retrains DRAM forever.
     * Serve a description instead, with the byte pattern selectable so the
     * expected encoding can be found by sweep.
     */
    if (off < 11 && MTK_ENV_CACHED("MTK_MEMINFO")) {
        static int hitn;

        if (hitn < 30) {
            hitn++;
            fprintf(stderr, "MTK-MEMINFO-HIT off=%#" PRIx64 " size=%u\n",
                    (uint64_t)off, size);
        }
        static uint8_t info[11];
        static int inited;
        unsigned bi = 0;

        if (!inited) {
            const char *e2 = getenv("MTK_MEMINFO");
            const char *q = e2;

            while (q && *q && bi < sizeof(info)) {
                char *end;

                info[bi++] = (uint8_t)strtoul(q, &end, 16);
                q = (*end == ',') ? end + 1 : NULL;
            }
            inited = 1;
        }
        return info[off];
    }

    mtk_win_ring[mtk_win_ring_idx % MTK_WIN_RING_LEN] =
        ((uint64_t)off << 8) | (size & 0xff);
    {
        CPUState *wcs = qemu_get_cpu(0);
        uint64_t wpc = 0;

        if (wcs) {
            CPUARMState *wenv = cpu_env(wcs);

            wpc = wenv->aarch64 ? wenv->pc : (uint64_t)wenv->regs[15];
        }
        mtk_win_pc_ring[mtk_win_ring_idx % MTK_WIN_RING_LEN] = wpc;
    }
    mtk_win_ring_idx++;

    if (MTK_ENV_CACHED("MTK_WINREAD2")) {
        static int nw2;

        if (nw2 < 400) {
            nw2++;
            fprintf(stderr, "MTK-WINREAD2 off=%#" PRIx64 " size=%u\n",
                    (uint64_t)off, size);
        }
    }
    if (MTK_ENV_CACHED("MTK_WINTRACE")) {
        static int nw;
        static uint64_t last_mb;

        /* Report only when the 1 MiB bucket changes, so the trace stays short. */
        if ((off >> 20) != last_mb) {
            last_mb = off >> 20;
            if (nw < 200) {
                nw++;
                fprintf(stderr, "MTK-WINREGION off=%#" PRIx64 " size=%u\n",
                        (uint64_t)off, size);
            }
        }
    }

    if (size > sizeof(buf) || off + size > s->data_win_size) {
        return 0;
    }

    pidx = off >> MT6895_DATA_PAGE_BITS;
    if (s->data_page_valid && s->data_page_idx == pidx) {
        page = s->data_page_cur;
    } else {
        page = g_hash_table_lookup(s->data_pages, &pidx);
        s->data_page_idx = pidx;
        s->data_page_cur = page;
        s->data_page_valid = true;
    }

    if (page) {
        memcpy(buf, page->data + (off & (MT6895_DATA_PAGE_SIZE - 1)), size);
    } else {
        uint64_t cidx = off >> MT6895_DATA_CACHE_BITS;

        if (!s->data_cache_valid || s->data_cache_idx != cidx) {
            uint64_t base = cidx << MT6895_DATA_CACHE_BITS;
            size_t want = MIN((uint64_t)MT6895_DATA_CACHE_SIZE,
                              s->data_win_size - base);
            ssize_t n = mt6895_pread(s->data_fd, s->data_cache, want, base);

            memset(s->data_cache + (n > 0 ? n : 0), 0,
                   want - (n > 0 ? (size_t)n : 0));
            s->data_cache_idx = cidx;
            s->data_cache_valid = true;
        }
        memcpy(buf, s->data_cache + (off & (MT6895_DATA_CACHE_SIZE - 1)), size);
    }

    for (i = 0; i < size; i++) {
        /* little-endian assembly below */
        (void)i;
    }
    switch (size) {
    case 1:
        return buf[0];
    case 2:
        return (uint64_t)buf[0] | ((uint64_t)buf[1] << 8);
    case 4:
        return ldl_le_p(buf);
    default:
        return ldq_le_p(buf);
    }
}

static void mt6895_data_win_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint64_t pidx = off >> MT6895_DATA_PAGE_BITS;
    Mt6895DataPage *page;
    unsigned i;

    if (off + size > s->data_win_size) {
        return;
    }

    if (MTK_ENV_CACHED("MTK_WINTRACE")) {
        static int nww;
        static uint64_t last_mb_w;

        if ((off >> 20) != last_mb_w) {
            last_mb_w = off >> 20;
            if (nww < 200) {
                nww++;
                fprintf(stderr, "MTK-WINWRITE off=%#" PRIx64 " size=%u\n",
                        (uint64_t)off, size);
            }
        }
    }

    page = g_hash_table_lookup(s->data_pages, &pidx);
    if (!page) {
        uint64_t base = pidx << MT6895_DATA_PAGE_BITS;
        size_t want = MIN((uint64_t)MT6895_DATA_PAGE_SIZE,
                          s->data_win_size - base);
        ssize_t n;

        page = g_new0(Mt6895DataPage, 1);
        page->idx = pidx;
        n = mt6895_pread(s->data_fd, page->data, want, base);
        if (n < 0) {
            memset(page->data, 0, sizeof(page->data));
        }
        g_hash_table_insert(s->data_pages, &page->idx, page);
    }

    for (i = 0; i < size; i++) {
        page->data[(off & (MT6895_DATA_PAGE_SIZE - 1)) + i] =
            (val >> (8 * i)) & 0xff;
    }

    /*
     * MTK_LU2_RW: persist the page into the image file as well.
     *
     * A real UFS logical unit is writable, and the preloader relies on that:
     * its DRAM-calibration stage saves the freshly trained block into the
     * "dram_para" partition and a later boot reads it back and verifies it.
     * With the image opened read-only the write only lands in s->data_pages,
     * which dies with the process -- so every QEMU run looks like a
     * never-booted device (calibration invalid -> retrain -> warm reset ->
     * two boot passes).  Persisting the page makes a second QEMU run start
     * from the state a real device is in, which is what the boot chain after
     * BL2_EXT assumes.
     *
     * The low 64 KiB of the window doubles as the payload's DMA bounce
     * buffer (the block-read callback is handed dst = MT6895_DATA_ADDR), so
     * writes there are transient and must never reach the image.
     */
    if (s->data_writable && off >= 0x10000) {
        mt6895_pwrite(s->data_fd, page->data, MT6895_DATA_PAGE_SIZE,
                      (uint64_t)pidx << MT6895_DATA_PAGE_BITS);
    }

    s->data_page_idx = pidx;
    s->data_page_cur = page;
    s->data_page_valid = true;
}

/*
 * Copy `len` bytes of block-device bytes at image offset `off` into `dst`.
 *
 * Unlike mt6895_data_win_read() this is not limited to the 2 GiB guest
 * window: it addresses the whole image, which is what lets the payload read
 * partitions that live above 2 GiB (the "mem" partition at byte offset
 * 0x299000000 is the one that matters here).
 *
 * Bytes the payload itself wrote earlier live in s->data_pages (see
 * mt6895_data_win_write), so those are consulted first; everything else comes
 * straight from the image file.
 */
static void mt6895_data_fetch(Mt6895MachineState *s, uint64_t off,
                              uint8_t *dst, size_t len)
{
    while (len) {
        uint64_t pidx = off >> MT6895_DATA_PAGE_BITS;
        unsigned poff = off & (MT6895_DATA_PAGE_SIZE - 1);
        uint64_t n = MIN(len, (uint64_t)(MT6895_DATA_PAGE_SIZE - poff));
        Mt6895DataPage *page = s->data_pages
            ? g_hash_table_lookup(s->data_pages, &pidx) : NULL;

        if (page) {
            memcpy(dst, page->data + poff, n);
        } else {
            ssize_t got = mt6895_pread(s->data_fd, dst, n, off);

            if (got < (ssize_t)n) {
                memset(dst + (got > 0 ? (size_t)got : 0), 0,
                       n - (got > 0 ? (size_t)got : 0));
            }
        }
        dst += n;
        off += n;
        len -= n;
    }
}

static uint64_t mt6895_bdev_rcmd_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    switch (off) {
    case 0:
        return s->bdev_cmd_lba;
    case 4:
        return s->bdev_cmd_cnt;
    default:
        return 0;
    }
}

static void mt6895_bdev_rcmd_write(void *opaque, hwaddr off, uint64_t val,
                                   unsigned size)
{
    Mt6895MachineState *s = opaque;

    switch (off) {
    case 0:
        s->bdev_cmd_lba = (uint32_t)val;
        break;
    case 4:
        s->bdev_cmd_cnt = (uint32_t)val;
        break;
    case 8: {
        uint32_t lba = s->bdev_cmd_lba;
        uint32_t cnt = s->bdev_cmd_cnt;
        uint32_t dst = (uint32_t)val;
        uint64_t boff = (uint64_t)lba * 512;
        uint64_t len = (uint64_t)cnt * 512;
        uint8_t *buf;

        if (!cnt || len > (16u << 20)) {
            if (MTK_ENV_CACHED("MTK_BDEVTRACE")) {
                fprintf(stderr, "MTK-BDEVREAD-REJECT lba=%u cnt=%u\n",
                        lba, cnt);
            }
            break;
        }
        if (s->data_fd < 0 || boff >= s->data_file_size) {
            if (MTK_ENV_CACHED("MTK_BDEVTRACE")) {
                fprintf(stderr, "MTK-BDEVREAD-EOF lba=%u cnt=%u\n", lba, cnt);
            }
            break;
        }
        if (boff + len > s->data_file_size) {
            len = s->data_file_size - boff;
        }
        buf = g_malloc(len);
        if (boff < 0x4400) {
            /*
             * LBAs 0..33 come from the table the machine synthesises at
             * MT6895_GPT_SOURCE (protective MBR at LBA 0, header at LBA 1,
             * entry array from LBA 2), not from the image, whose own GPT is
             * laid out in 4 KiB units.
             */
            uint64_t n = MIN(len, (uint64_t)(0x4400 - boff));
            uint8_t *gp = mt6895_guest_ptr(s, MT6895_GPT_SOURCE + boff, n);

            if (gp) {
                memcpy(buf, gp, n);
            } else {
                memset(buf, 0, n);
            }
            if (len > n) {
                mt6895_data_fetch(s, boff + n, buf + n, len - n);
            }
        } else {
            mt6895_data_fetch(s, boff, buf, len);
        }
        if (MTK_ENV_CACHED("MTK_BDEVTRACE")) {
            static int nrd;

            if (nrd < 60) {
                nrd++;
                fprintf(stderr, "MTK-BDEVREAD lba=%u cnt=%u dst=%#x "
                        "bytes=%02x%02x%02x%02x %02x%02x%02x%02x\n",
                        lba, cnt, dst,
                        buf[0], buf[1], buf[2], buf[3],
                        buf[4], buf[5], buf[6], buf[7]);
            }
        }
        address_space_write(&address_space_memory, dst,
                            MEMTXATTRS_UNSPECIFIED, buf, len);
        g_free(buf);
        break;
    }
    default:
        break;
    }
}

static const MemoryRegionOps mt6895_bdev_rcmd_ops = {
    .read = mt6895_bdev_rcmd_read,
    .write = mt6895_bdev_rcmd_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static const MemoryRegionOps mt6895_data_win_ops = {
    .read = mt6895_data_win_read,
    .write = mt6895_data_win_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static void mt6895_place_data(Mt6895MachineState *s, Error **errp)
{
    struct stat st;
    uint64_t len;

    if (!s->data_path) {
        return;
    }

    /*
     * MTK_LU2_RW: open the data LUN read-write so that what the firmware
     * stores (notably the trained DRAM-calibration block in "dram_para")
     * survives the process.  A real UFS LUN is writable; read-only made every
     * run look like a never-booted device.
     */
    if (MTK_ENV_CACHED("MTK_LU2_RW")) {
        s->data_fd = qemu_open_old(s->data_path, O_RDWR | O_BINARY);
        s->data_writable = s->data_fd >= 0;
        if (s->data_fd < 0) {
            s->data_fd = qemu_open_old(s->data_path, O_RDONLY | O_BINARY);
        }
    } else {
        s->data_fd = qemu_open_old(s->data_path, O_RDONLY | O_BINARY);
    }
    if (s->data_fd < 0) {
        error_setg(errp, "mt6895: cannot open data image '%s': %s",
                   s->data_path, strerror(errno));
        return;
    }
    if (fstat(s->data_fd, &st) < 0) {
        error_setg(errp, "mt6895: cannot stat data image '%s': %s",
                   s->data_path, strerror(errno));
        close(s->data_fd);
        s->data_fd = -1;
        return;
    }

    /*
     * 2 GiB is the ceiling of the preloader stub's 32-bit source arithmetic;
     * nothing above it can be reached through the callback.
     */
    len = MIN((uint64_t)st.st_size, 0x80000000ULL);
    s->data_win_size = len;
    s->data_file_size = (uint64_t)st.st_size;

    if (!s->data_pages) {
        s->data_pages = g_hash_table_new_full(g_int64_hash, g_int64_equal,
                                              NULL, g_free);
    }
    if (!s->data_cache) {
        s->data_cache = g_malloc(MT6895_DATA_CACHE_SIZE);
    }

    memory_region_init_io(&s->data_win, OBJECT(s), &mt6895_data_win_ops, s,
                          "mt6895.data-image", len);
    memory_region_add_subregion_overlap(get_system_memory(), MT6895_DATA_ADDR,
                                        &s->data_win, 1);

    info_report("mt6895: data image '%s' served on demand (%" PRIu64
                " bytes at %#" PRIx64 "), no start-up copy",
                s->data_path, len, (uint64_t)MT6895_DATA_ADDR);
}

/*
 * WDT / RGU window.  The boot chain arms the watchdog here and, when the
 * DRAM calibration has to be redone, asks the SoC to reset.  The generic
 * register-file window only stores the writes, so the reset request is
 * invisible; a dedicated window lets the model observe (and later honour)
 * that request.
 */
/*
 * GenieZone domain remap control, SECURITY_AO + 0x690.
 *
 * The preloader enables the EMI ("DDR") and infra-IO domain remaps before it
 * hands over, then prints them ("[GZ_REMAP] GZ Domain Remap of EMI: enabled",
 * "DDR remap offset: VMID0 0x0, VMID1 0x800000000").  Its print routine
 * (0x2032648) tests bit 5 of +0x0 for the EMI remap and bit 5 of +0x4 for the
 * infra-IO remap, and the per-VMID offset is fixed at VMID << 35 (0x20325e8).
 *
 * With the EMI remap on, an access at (VMID1 << 35) + PA reaches DRAM at PA.
 * That is how BL31's BL33 entry -- bl33_start_addr = 0x87F800000 in the
 * preloader's mtk_bl_param_t -- reaches GenieZone, which the preloader loaded
 * at DRAM 0x7F800000 ("load gz_a ... to 0x7F800000").  The infra-IO remap is
 * recorded but not modelled yet.
 */
#define MT6895_GZ_REMAP_CTRL    0x1c00b690ULL
#define MT6895_GZ_VMID1_OFFSET  (1ULL << 35)

static uint64_t mt6895_gz_remap_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off / 4 >= ARRAY_SIZE(s->gz_remap_regs)) {
        return 0;
    }
    return s->gz_remap_regs[off / 4];
}

static void mt6895_gz_remap_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    Mt6895MachineState *s = opaque;
    bool emi;

    if (off / 4 >= ARRAY_SIZE(s->gz_remap_regs)) {
        return;
    }
    s->gz_remap_regs[off / 4] = val;
    emi = s->gz_remap_regs[0] & (1u << 5);
    memory_region_set_enabled(&s->gz_ddr_remap, emi);
    {
        static int n;

        if (n < 8) {
            n++;
            fprintf(stderr, "mt6895: GZ remap ctrl +%#x = %#" PRIx64
                    " (EMI remap %s, infra-IO remap %s)\n", (unsigned)off,
                    val, emi ? "on" : "off",
                    (s->gz_remap_regs[1] & (1u << 5)) ? "on" : "off");
        }
    }
}

static const MemoryRegionOps mt6895_gz_remap_ops = {
    .read = mt6895_gz_remap_read,
    .write = mt6895_gz_remap_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * Set when the guest asks for mtk_arch_reset().  A warm reset restarts the
 * CPU but leaves the peripheral domain alone, which is what the boot chain
 * expects: after the DRAM calibration it resets and then reads the storage
 * again without re-initialising anything.
 */
static bool mt6895_warm_reset_requested;
/*
 * The preloader reads MTK_WDT_STATUS (0x0C) early to learn why the chip
 * restarted and prints "[RGU] rst from: <kernel|BL2EXT>".  A fresh boot reads
 * zero; after a software reset the watchdog should report the reason so the
 * loader knows this is a reset boot and can reuse the saved DRAM calibration
 * instead of retraining.  The encoding is swept with MTK_WDT_STATUS_VAL.
 */
static bool mt6895_swreset_seen;

static int mt6895_wdt_status_val(void)
{
    static int v = -1;

    if (v < 0) {
        const char *e = getenv("MTK_WDT_STATUS_VAL");

        v = e ? atoi(e) : 0;
    }
    return v;
}

/*
 * Fastboot-key latch.  LK's recovery_check() (0x50f0b350 -> 0x50f3b9f0)
 * reads RGU+0x24 once: while the cached "special key" byte is the 0xF
 * sentinel it takes the register's low nibble, clears it in hardware and
 * caches the value.  A cached value of 3 makes LK select boot mode 99
 * (FASTBOOT), notify_enter_fastboot() and run the fastboot gadget instead
 * of booting Linux.  The real preloader sets that nibble from the latched
 * key state; MTK_RGU_KEY=<n> lets the model inject it.
 */
static int mt6895_rgu_key_val(void)
{
    static int v = -1;

    if (v < 0) {
        const char *e = getenv("MTK_RGU_KEY");

        v = e ? (int)strtol(e, NULL, 0) : -1;
    }
    /*
     * MTK_RGU_KEY_ONCE=1: only the first boot of this QEMU process sees the
     * latched fastboot key.  The flash-and-reboot workflow needs that: the
     * first boot enters LK fastboot so the host can flash, and the reboot
     * after it must proceed to the freshly flashed image instead of landing
     * in fastboot again.
     */
    if (v >= 0) {
        const char *once = getenv("MTK_RGU_KEY_ONCE");

        if (once) {
            /*
             * Allow the latched key for the first N reads (N defaults to the
             * four reads a normal LK boot performs), so the first boot lands
             * in fastboot; any later boot of the same QEMU process sees 0
             * and proceeds to the freshly flashed image.
             */
            static unsigned reads;
            unsigned allowed = 4;

            if (*once) {
                int n = (int)strtol(once, NULL, 0);

                if (n > 0) {
                    allowed = (unsigned)n;
                }
            }
            if (reads >= allowed) {
                return 0;
            }
            reads++;
        }
    }
    return v;
}

static uint64_t mt6895_wdt_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint64_t v = 0;

    if (off / 4 < ARRAY_SIZE(s->wdt_store)) {
        v = extract64(s->wdt_store[off / 4], 0, size * 8);
    }
    if (off == 0x0c && mt6895_swreset_seen) {
        v = (uint32_t)mt6895_wdt_status_val();
    }
    if (off == 0x24) {
        int k = mt6895_rgu_key_val();

        if (k >= 0) {
            v = (v & ~0xfULL) | ((unsigned)k & 0xf);
            if (MTK_ENV_CACHED("MTK_RGUTRACE")) {
                fprintf(stderr, "mt6895: RGU+0x24 read -> %#" PRIx64
                        " (fastboot key latch %d)\n", v, k);
            }
        }
    }
    if (MTK_ENV_CACHED("MTK_WDTTRACE")) {
        static int nr;

        if (nr < 80) {
            nr++;
            fprintf(stderr, "MTK-WDTRD off=%#" PRIx64 " val=%#" PRIx64
                    " size=%u\n", (uint64_t)off, v, size);
        }
    }
    return v;
}

static void mt6895_wdt_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off / 4 >= ARRAY_SIZE(s->wdt_store)) {
        return;
    }
    s->wdt_store[off / 4] = val;

    /*
     * MTK_WDT_RESTART: writing the 0x1209 key asks the SoC for an immediate
     * software reset -- this is how mtk_arch_reset() restarts the chip after
     * the DRAM calibration has to be redone.  Until now the model stored the
     * write and did nothing, so the preloader carried on executing past the
     * reset request instead of the machine coming back up.
     */
    if (off == 0x14 && (val & 0xffff) == 0x1209) {
        /*
         * mtk_arch_reset() asks for a WARM reset: the SoC restarts from the
         * BootROM but the peripheral domain (DRAM, UFS, UART, PMIC) keeps its
         * state.  QEMU always performs a cold reset, so record the request
         * and let mt6895_machine_reset() skip the device reset.
         */
        if (MTK_ENV_CACHED("MTK_NO_SWRESET")) {
            return;
        }
        /*
         * Record where the guest asked for the reset: the preloader has
         * several mtk_arch_reset() call sites (DRAM calibration failure,
         * vdd2l decision, boot-mode handling) and the PC/LR is the only
         * cheap way to tell which one fired.  AArch32 keeps the PC in
         * regs[15] and LR in regs[14].
         */
        {
            CPUState *cs = qemu_get_cpu(0);

            if (cs) {
                CPUARMState *env = cpu_env(cs);
                uint32_t stk[8] = { 0 };
                /* NB: regs[] is uint32_t[16]; AArch64 state uses xregs[]. */
                uint32_t lr = env->aarch64 ? (uint32_t)env->xregs[30]
                                           : env->regs[14];
                uint32_t sp = env->aarch64 ? (uint32_t)env->xregs[31]
                                           : env->regs[13];

                address_space_read(&address_space_memory, sp,
                                   MEMTXATTRS_UNSPECIFIED, stk, sizeof(stk));
                fprintf(stderr, "mt6895: WDT software reset (arch_reset, warm) "
                        "pc=%#lx lr=%#x sp=%#x stk=%08x %08x %08x %08x "
                        "%08x %08x %08x %08x\n",
                        (unsigned long)(env->aarch64 ? env->pc
                                                     : env->regs[15]),
                        lr, sp, stk[0], stk[1], stk[2], stk[3],
                        stk[4], stk[5], stk[6], stk[7]);
            } else {
                fprintf(stderr,
                        "mt6895: WDT software reset (arch_reset, warm)\n");
            }
        }
        mt6895_swreset_seen = true;
        mt6895_warm_reset_requested = true;
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        return;
    }

    if (MTK_ENV_CACHED("MTK_WDTTRACE")) {
        static int n;

        if (n < 200) {
            n++;
            fprintf(stderr, "MTK-WDTWR off=%#" PRIx64 " val=%#" PRIx64
                    " size=%u\n", (uint64_t)off, val, size);
        }
    }
}

static const MemoryRegionOps mt6895_wdt_ops = {
    .read = mt6895_wdt_read,
    .write = mt6895_wdt_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * Transparent read probe for the DRAM calibration "initialised" marker.
 * The preloader checks 0x02075DC8 against 256; the machine plants 256 there
 * before the guest starts, so observing the value the guest actually reads
 * tells whether something (the payload's own BSS clear) erases it first.
 */
static uint64_t mt6895_calmark_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *m = mt6895_guest_ptr(s, 0x02075dc8ULL, 4);
    uint32_t v = m ? ldl_le_p(m) : 0;

    if (MTK_ENV_CACHED("MTK_CALMARK")) {
        static int n;

        if (n < 12) {
            n++;
            fprintf(stderr, "MTK-CALREAD off=%#" PRIx64 " val=%#x\n",
                    (uint64_t)off, v);
        }
    }
    return v;
}

static void mt6895_calmark_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint8_t *m = mt6895_guest_ptr(s, 0x02075dc8ULL, 4);

    if (m) {
        stl_le_p(m, (uint32_t)val);
    }
    if (MTK_ENV_CACHED("MTK_CALMARK")) {
        static int nw;

        if (nw < 12) {
            nw++;
            fprintf(stderr, "MTK-CALWRITE off=%#" PRIx64 " val=%#" PRIx64 "\n",
                    (uint64_t)off, val);
        }
    }
}

static const MemoryRegionOps mt6895_calmark_ops = {
    .read = mt6895_calmark_read,
    .write = mt6895_calmark_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mt6895_place_gpt(Mt6895MachineState *s, Error **errp)
{
    g_autofree char *buf = NULL;
    gsize len = 0;
    g_autoptr(GError) gerr = NULL;
    uint8_t *image;

    if (!s->gpt_path) {
        return;
    }

    if (!g_file_get_contents(s->gpt_path, &buf, &len, &gerr)) {
        error_setg(errp, "mt6895: cannot read GPT image '%s': %s",
                   s->gpt_path, gerr->message);
        return;
    }

    if (len < MT6895_GPT_HEADER_SIZE +
              MT6895_GPT_ENTRIES * MT6895_GPT_ENTRY_SIZE) {
        error_setg(errp, "mt6895: '%s' is too small for a GPT header and %d "
                   "partition entries", s->gpt_path, MT6895_GPT_ENTRIES);
        return;
    }

    /*
     * The header sits at the buffer start and the entries at the LBA-2 offset,
     * so the image spans 2 * 512 bytes before the entry array -- not just the
     * header size.
     */
    image = g_malloc0(2 * 512 +
                      MT6895_GPT_ENTRIES * MT6895_GPT_ENTRY_SIZE);

    /*
     * Standard GPT disk geometry, matching the LBAs the firmware asks the
     * block callback for (r1 = 1 for the header; r1 = 2, 10, 18, 26 for the
     * four consecutive 4 KiB windows of the 16 KiB entry array):
     *   LBA 0 (offset 0): Protective MBR (zeros)
     *   LBA 1 (offset 512): GPT Header (signature "EFI PART")
     *   LBA 2 (offset 1024): GPT Partition Entries
     */
    memcpy(image + 1 * 512, buf, MT6895_GPT_HEADER_SIZE);
    memcpy(image + 2 * 512, buf + MT6895_GPT_HEADER_SIZE,
           MT6895_GPT_ENTRIES * MT6895_GPT_ENTRY_SIZE);

    address_space_write(&address_space_memory, MT6895_GPT_SOURCE,
                        MEMTXATTRS_UNSPECIFIED, image,
                        2 * 512 +
                        MT6895_GPT_ENTRIES * MT6895_GPT_ENTRY_SIZE);

    g_free(image);

    /*
     * Publish the entry-copy length where the callback can read it, so the
     * boundary can be swept without rebuilding the stub each time.
     */
    {
        uint32_t n = s->entry_bytes;

        address_space_write(&address_space_memory, MT6895_ENTRY_BYTES_ADDR,
                            MEMTXATTRS_UNSPECIFIED, &n, sizeof n);
    }

    info_report("mt6895: placed GPT from '%s' at %#" PRIx64
                " (entry copy %u bytes)",
                s->gpt_path, (uint64_t)MT6895_GPT_SOURCE, s->entry_bytes);
}


/*
 * Genuine BootROM boot-argument hand-off block.
 *
 * The BROM hands the preloader a pointer in r4; the SRAM stage stores it at
 * [0x02001000] and the DRAM stage copies 0x93C bytes from it into 0x020F3740,
 * where the fields are consumed as fixed offsets (notes section 18): the block
 * is a descriptor, not the TLV list it was first assumed to be.
 *
 * Real BootROM does not zero the whole window: fields it does not write keep
 * whatever SRAM already held (stale/non-zero content), and the preloader copes
 * with that.  The prime test showed a fully zeroed window stalls before LK
 * prints "Booting Linux" while stale content takes a benign path, so this
 * builder never memsets: in "control" mode it writes nothing at all, and in
 * fill mode it writes only fields whose value has a genuine modelled source
 * (BOOT_TAG_VERSION/REASON/MODE/EFLAG/SRAM start/size).
 *
 * This is a MODEL-SIDE experiment about how the preloader reacts to zeroed
 * versus stale descriptor fields; the window address is not claimed to be the
 * hardware layout.
 *
 * The window deliberately sits inside the preloader payload's string/rodata
 * table (image spans 0x02000F10..0x0207AFC0), which is non-zero and not
 * executed: 0x02040000 was rejected after disassembly showed it is live Thumb
 * code, so writing justified fields there would crash the preloader instead of
 * testing the descriptor-content hypothesis.
 *
 * The hand-off block also carries a genuine BROM field the preloader's MMM
 * parser consumes at +0x808 (a pointer to an MMM container).  Our genuine
 * boot-LUN container matches that parser's root-component path exactly
 * (masked magic 0x4D4D4D, type 0, +0x1B=5, size 0x7A0B0, +0x2C=0x66C and the
 * 0xE291F358 trailer signature at trailer+4, which the parser turns into
 * cache+0xC=1 and a cache+0x10 value), but the parser's type-8 metadata
 * (magic 0xC975E033 at component+0x2C, the flag at 0x020FB720) is present in
 * none of the genuine images this model holds (fw/LUA_1 and every image in
 * firmware/xaga/parts were scanned).  Pointing +0x808 at our own container
 * regressed the boot: the parser walked the chain and cached the metadata,
 * then spun reading cache+0x8 and the boot stalled at LK FGADC (+60 s).
 * The field is therefore deliberately left stale rather than invented, and
 * the builder stays default OFF until the genuine container is available.
 */
#define MT6895_BROM_BOOTARGS_PA   0x02065000ULL
#define MT6895_BROM_BOOTARGS_SIZE 0x00001000ULL
#define MT6895_BROM_TLV_TYPE      0x779f0000U

static void mt6895_build_brom_bootargs(Mt6895MachineState *s, bool fill)
{
    uint8_t *p = mt6895_guest_ptr(s, MT6895_BROM_BOOTARGS_PA,
                                  MT6895_BROM_BOOTARGS_SIZE);
    const char *e;
    uint32_t reason = 1;
    size_t off = 0;

    if (!p) {
        return;
    }
    /*
     * No memset (see the block comment): control mode writes nothing and lets
     * the pre-existing bytes stand; fill mode writes only justified fields.
     */
    if (!fill) {
        return;
    }

    e = getenv("MTK_BOOT_REASON");
    if (e) {
        reason = (uint32_t)atoi(e);
    }

    /* id 0x00: BOOT_TAG_VERSION (format version of this boot-argument ABI). */
    stl_le_p(p + off, 0x0c);
    stl_le_p(p + off + 4, MT6895_BROM_TLV_TYPE | 0x00);
    stl_le_p(p + off + 8, 1);
    off += 0x0c;

    /* id 0x01: BOOT_REASON, same value the boot dispatcher is given. */
    stl_le_p(p + off, 0x0c);
    stl_le_p(p + off + 4, MT6895_BROM_TLV_TYPE | 0x01);
    stl_le_p(p + off + 8, reason);
    off += 0x0c;

    /* id 0x02: BOOT_MODE, cold boot from power-on. */
    stl_le_p(p + off, 0x0c);
    stl_le_p(p + off + 4, MT6895_BROM_TLV_TYPE | 0x02);
    stl_le_p(p + off + 8, 0);
    off += 0x0c;

    /* id 0x0c: EFLAG, cold boot (no early-boot flag). */
    stl_le_p(p + off, 0x0c);
    stl_le_p(p + off + 4, MT6895_BROM_TLV_TYPE | 0x0c);
    stl_le_p(p + off + 8, 0);
    off += 0x0c;

    /* id 0x12: SRAM window the BROM reports to the preloader. */
    stl_le_p(p + off, 0x10);
    stl_le_p(p + off + 4, MT6895_BROM_TLV_TYPE | 0x12);
    stl_le_p(p + off + 8, 0x0010dc00);
    stl_le_p(p + off + 0xc, 0x00012400);
    off += 0x10;

    /* size==0 terminates the record list. */
    stl_le_p(p + off, 0);
}

static void mt6895_load_boot_lun(Mt6895MachineState *s, Error **errp,
                                 bool on_reset)
{
    g_autofree char *buf = NULL;
    gsize len = 0;
    GError *gerr = NULL;
    uint32_t img_off, size, addr, entry_off;
    uint64_t load_end;
    void *dest, *dest2, *dest2_f10;

    if (!s->preloader_path) {
        info_report("mt6895: no -machine preloader=<image> given; "
                    "the BootROM will start without a payload");
        return;
    }

    if (!g_file_get_contents(s->preloader_path, &buf, &len, &gerr)) {
        error_setg(errp, "mt6895: cannot read boot LUN image '%s': %s",
                   s->preloader_path, gerr->message);
        g_error_free(gerr);
        return;
    }

    if (len < MT6895_BOOTROM_SIZE + UFS_BOOT_HDR_MIN) {
        error_setg(errp, "mt6895: '%s' is too small to hold a UFS boot header",
                   s->preloader_path);
        return;
    }

    if (memcmp(buf, "UFS_BOOT", UFS_BOOT_MAGIC_LEN) != 0) {
        error_setg(errp, "mt6895: '%s' does not start with the UFS_BOOT "
                   "magic", s->preloader_path);
        return;
    }

    img_off = ld32((const uint8_t *)buf + UFS_BOOT_IMG_OFF);
    if (img_off > len || len - img_off < MMM_HDR_MIN) {
        error_setg(errp, "mt6895: boot LUN image offset 0x%x is outside '%s'",
                   img_off, s->preloader_path);
        return;
    }

    if (ld32((const uint8_t *)buf + img_off) != MMM_MAGIC) {
        error_setg(errp, "mt6895: no MMM container at offset 0x%x of '%s'",
                   img_off, s->preloader_path);
        return;
    }

    size = ld32((const uint8_t *)buf + img_off + MMM_SIZE_OFF);
    addr = ld32((const uint8_t *)buf + img_off + MMM_ADDR_OFF);
    entry_off = ld32((const uint8_t *)buf + img_off + MMM_ENTRY_OFF);

    if (size < MMM_HDR_MIN || size > len - img_off) {
        error_setg(errp, "mt6895: MMM payload size 0x%x does not fit in '%s'",
                   size, s->preloader_path);
        return;
    }
    if (entry_off >= size) {
        error_setg(errp, "mt6895: MMM entry offset 0x%x is outside the "
                   "payload (size 0x%x)", entry_off, size);
        return;
    }

    load_end = (uint64_t)addr + size;
    dest = mt6895_guest_ptr(s, addr, size);
    if (!dest) {
        error_setg(errp, "mt6895: payload [0x%x, 0x%" PRIx64 ") does not land "
                   "in DRAM [0x%" PRIx64 ", 0x%" PRIx64 ") or any SRAM window",
                   addr, load_end, (uint64_t)MT6895_DRAM_BASE,
                   (uint64_t)MT6895_DRAM_BASE + MACHINE(s)->ram_size);
        return;
    }

    /*
     * Payload literals the machine may override (properties "stage-arg" and
     * "stage-entry-patch").  They are applied to the host buffer here, before
     * the copies below, so that the one copy pass -- and the MTK_RESET_KEEP_*
     * gates in it -- decides what lands in SRAM.  They used to be applied
     * after the copies and followed by a second, unconditional copy of the
     * whole image into SRAM0, SRAM1 and SRAM1+0xF10, which silently undid
     * MTK_RESET_KEEP_SRAM0 and MTK_RESET_KEEP_AEE on every reset.
     *
     * The SRAM stage ends by loading the literal at image offset 0x304
     * (0x020360B9) into pc.  In the layout the DRAM stage actually runs from
     * -- the SRAM1+0xF10 copy below, i.e. VA = LUA_1 file offset - 0xF0 +
     * 0x02000000 -- 0x020360B8 is the first instruction of a function
     * (push {r4-r7,lr}; add r7, sp, #12; push.w {r8-fp}; sub sp, #0x74).
     * An older note called it a shared tail and moved the entry to 0x02035FF9,
     * but it read the image as if it started at 0x02000000; in the live layout
     * 0x02035FF8 is mid-function.  MT6895_DRAM_STAGE_ENTRY is therefore the
     * literal's own value and the default patch is an identity.
     */
    {
        uint8_t *img = (uint8_t *)buf + img_off;

        if (s->stage_arg) {
            stl_le_p(img + MT6895_STAGE_ARG_LIT_OFF, s->stage_arg);
        }
        if (s->stage_entry_patch &&
            ld32(img + MT6895_DRAM_STAGE_ENTRY_LIT_OFF) ==
            MT6895_DRAM_STAGE_ENTRY_LIT_OLD) {
            stl_le_p(img + MT6895_DRAM_STAGE_ENTRY_LIT_OFF,
                     MT6895_DRAM_STAGE_ENTRY);
        }
    }

    /*
     * MTK_RESET_KEEP_SRAM0: on a warm reset, leave the payload's SRAM0 image
     * (and the SRAM1+0xF10 alias) untouched so whatever state the previous
     * boot left in SRAM survives -- notably the DRAM-exception latch the
     * preloader sets up to tell a cold boot from a reboot.  Only the SRAM1
     * code copy is refreshed, which is what the preloader needs to run.
     */
    if (!(on_reset && MTK_ENV_CACHED("MTK_RESET_KEEP_SRAM0"))) {
        /*
         * MTK_RESET_KEEP_AEE: keep only the AEE ram_console page
         * (0x11D000..0x11E000) across the reload.  The real AEE console is a
         * small SRAM area that is not part of the payload image and does not
         * get cleared by an AP reset, but in this model it lives inside the
         * SRAM0 window the payload is copied into, so a plain reload wipes the
         * AEE memory-info block the BL2_EXT stage validates.  This is the
         * narrow, hardware-shaped variant of MTK_RESET_KEEP_SRAM0.
         */
        if (on_reset && MTK_ENV_CACHED("MTK_RESET_KEEP_AEE") &&
            (uint64_t)addr <= MT6895_AEE_CONSOLE_BASE &&
            (uint64_t)addr + size >=
                MT6895_AEE_CONSOLE_BASE + MT6895_AEE_CONSOLE_SIZE) {
            static uint8_t aee_save[MT6895_AEE_CONSOLE_SIZE];
            uint64_t rel = MT6895_AEE_CONSOLE_BASE - (uint64_t)addr;

            memcpy(aee_save, dest + rel, sizeof(aee_save));
            memcpy(dest, buf + img_off, size);
            memcpy(dest + rel, aee_save, sizeof(aee_save));
            if (MTK_ENV_CACHED("MTK_BDEVTRACE")) {
                fprintf(stderr, "mt6895: preserved AEE console page at %#llx\n",
                        (unsigned long long)MT6895_AEE_CONSOLE_BASE);
            }
        } else {
            memcpy(dest, buf + img_off, size);
        }
    }

    /*
     * MTK preloaders exist in two build flavours -- an SRAM build based at
     * 0x01000000 and a DRAM build based at 0x02000000 (see the upstream
     * link_sram_descriptor.ld / link_descriptor.ld). The image here carries
     * absolute addresses from both worlds: it zeroes a BSS at 0x00102180
     * and another at 0x020F1000, and it jumps to 0x020360B9, which is the
     * Thumb code living at image offset 0x360B8. So the SRAM stage hands
     * over to a second copy of the same payload placed in the 0x02000000
     * window, and the BootROM is what puts it there.
     */
    dest2 = mt6895_guest_ptr(s, MT6895_SRAM1_BASE, size);
    if (dest2) {
        memcpy(dest2, buf + img_off, size);
    }
    dest2_f10 = mt6895_guest_ptr(s, MT6895_SRAM1_BASE + 0xF10, size);
    if (dest2_f10 && !(on_reset && MTK_ENV_CACHED("MTK_RESET_KEEP_SRAM0"))) {
        memcpy(dest2_f10, buf + img_off, size);
    }


    /*
     * The BootROM hands the DRAM-mode stage a few parameters in its data
     * area. One of them is the stack size: the payload's SRAM stage computes
     *
     *     sp = 0x02000000 + (*(0x0206D308) - 4)
     *
     * and with the image's own bytes still sitting there (that offset lands
     * inside the payload's string table, and holds "ess ") the computed stack
     * pointer is 0x22737361 -- outside SRAM1 entirely, so the first "push"
     * faults. Writing a real stack size here is what makes sp land inside the
     * DRAM window, exactly as the BootROM arranges on hardware.
     */

    /*
     * REVERTED the paired image probe, which established what the two offsets
     * actually are:
     *
     *   0x3658C  IS a code pointer. Replacing 0x02045C75 with the
     *            boot-argument block (0x02001000) made "blx r4" jump into the
     *            block, and the trace filled with 11.2M undefined-instruction
     *            exceptions at 0x02001000/0x02001004/0x02001008 -- it was
     *            executing data as code. So the payload is right and the
     *            "it should be a data pointer" hypothesis is wrong.
     *
     *   0x304    retargeting it to 0x020360A1 does let the r4 load run, and
     *            the DRAM stage then reads r4 = 0x21 out of the boot-argument
     *            block. But the loop at 0x02045C72 passes that same r4 to its
     *            own code as a data pointer and faults on the odd address --
     *            so in the real flow that loop is not reached, and r4's
     *            contract is not something to be reverse-engineered by
     *            patching the image.
     *
     * Both probes are gone. Reaching LK needs the real preloader's register
     * contract, which means either documenting it from the preloader's own
     * code or leaving this stage to a proper BootROM model.
     */

    /*
     * The DRAM stage's stack size, written after every copy above on purpose.
     *
     * The SRAM stage computes sp = 0x02000000 + (*(0x0206D308) - 4) from this
     * location, and the payload's string table lands on it -- left alone it
     * reads back "ess " (0x20737365), which sends sp to 0x22737361, outside
     * SRAM1, so the first push faults with DFAR 0x22737351. Putting this write
     * before the memcpy above reintroduced exactly that fault three times.
     */
    if (dest2) {
        stl_le_p((uint8_t *)dest2 + MT6895_DRAM_STAGE_STACK_SIZE_OFF,
                 MT6895_DRAM_STAGE_STACK_SIZE);

        /*
         * The DRAM stage's debug UART base, which it keeps in a variable rather
         * than a literal: its character-output routine does
         *
         *     ldr r1, [pc, #24]     ; r1 = 0x02079E00 (a variable)
         *     ldr r2, [r1]          ; r2 = the UART base stored there
         *     ldr r2, [r2, #20]     ; UART line status
         *     lsls r2, r2, #26      ; test bit 5, TX holding register empty
         *     bpl  <loop>           ; wait for it forever
         *
         * With that variable left at zero the poll reads address 0x14, bit 5
         * never sets, and the preloader spins before printing its first
         * character -- which is why not one byte came out of the serial port
         * and why nothing after this point (partition reads, loading LK) ever
         * ran. On hardware the BootROM supplies it, the same way it supplies
         * the stack size above.
         */
        /*
         * The UART base variable is addressed by the payload as an ABSOLUTE
         * address (the literal 0x02079E00 appears in the image), so this one
         * belongs to the 0x02000000 copy, unlike the patch offsets below.
         */
        stl_le_p((uint8_t *)dest2 + MT6895_DRAM_STAGE_UART_BASE_OFF,
                 MT6895_UART0_BASE);

        /*
         * The offsets below are IMAGE offsets -- locations of code and of
         * literal-pool slots inside the payload -- and the copy the CPU
         * actually executes is the one based at 0x02000F10, so these have to
         * be applied to dest2_f10. Applying them to dest2 as well used to land
         * them 0xF10 bytes low inside the executing copy, silently corrupting
         * whatever lived there: the DCM stub, for instance, was written over
         * img[0x3DFC4 - 0xF10] = the literal-pool slot holding printf's
         * address, so the RTC reporter at 0x0203DD1A loaded 0x47702000 (the
         * stub's own encoding) as a function pointer and branched into empty
         * DRAM.
         *
         * Measured confirmation that dest2_f10 is the executing copy: the
         * GPIO stub planted at dest2_f10 + 0x305C4 (absolute 0x020314D4) is
         * reached and executed, while the dest2 equivalent at 0x020305C4 is
         * never executed at all.
         */
        if (dest2_f10) {
            /*
             * Virtual GPIO pad-control bypass for UART0 pins.
             * The hardware IOCFG pull-up/pull-down and Schmitt-trigger helper
             * routines (image offsets 0x305C4 and 0x305D4) attempt to write to
             * unmapped low address offsets in emulation. In QEMU, UART0 is a
             * virtual software FIFO and requires no analog pad tuning. Shorting
             * these two void helpers with "bx lr" (0x4770) allows
             * bldr_pre_process to complete UART initialization cleanly.
             */
            /*
             * The pad-control helpers now reach a modelled pinctrl block, so
             * they no longer need to be shorted out: leave the firmware code
             * intact and let it program the pull-up on GPIO5 (fastmeta pin).
             */
            if (!MTK_ENV_CACHED("MTK_KEEP_GPIO_STUB")) {
                /* nothing: run the real helpers */
            }
            stw_le_p((uint8_t *)dest2_f10 + MT6895_DCM_DEVAPC_INIT_OFF, 0x2000);
            stw_le_p((uint8_t *)dest2_f10 + MT6895_DCM_DEVAPC_INIT_OFF + 2, 0x4770);

            /*
             * Redirect PLL settling delay callbacks from 0x020489BD
             * to a clean "bx lr" stub (the GPIO_SET_PULL stub above).
             * In QEMU, PLL registers lock instantaneously without physical
             * delay.
             */
            stl_le_p((uint8_t *)dest2_f10 + MT6895_PLL_DELAY_LIT_OFF_1,
                     (uint32_t)(MT6895_SRAM1_BASE + 0xF10 +
                                MT6895_GPIO_SET_PULL_OFF + 1));
            stl_le_p((uint8_t *)dest2_f10 + MT6895_PLL_DELAY_LIT_OFF_2,
                     (uint32_t)(MT6895_SRAM1_BASE + 0xF10 +
                                MT6895_GPIO_SET_PULL_OFF + 1));

            /*
             * Redirect boot profiler get_timer callbacks from 0x0204897D
             * to a clean "movs r0, #0; bx lr" stub
             * (the DCM_DEVAPC_INIT stub above).
             */
            stl_le_p((uint8_t *)dest2_f10 + MT6895_PROFILER_TIME_LIT_OFF_1,
                     (uint32_t)(MT6895_SRAM1_BASE + 0xF10 +
                                MT6895_DCM_DEVAPC_INIT_OFF + 1));
            stl_le_p((uint8_t *)dest2_f10 + MT6895_PROFILER_TIME_LIT_OFF_2,
                     (uint32_t)(MT6895_SRAM1_BASE + 0xF10 +
                                MT6895_DCM_DEVAPC_INIT_OFF + 1));
            stl_le_p((uint8_t *)dest2_f10 + MT6895_PROFILER_TIME_LIT_OFF_3,
                     (uint32_t)(MT6895_SRAM1_BASE + 0xF10 +
                                MT6895_DCM_DEVAPC_INIT_OFF + 1));
            stl_le_p((uint8_t *)dest2_f10 + MT6895_PROFILER_TIME_LIT_OFF_4,
                     (uint32_t)(MT6895_SRAM1_BASE + 0xF10 +
                                MT6895_DCM_DEVAPC_INIT_OFF + 1));
        }

        /*
         * Redirect PLL frequency query callbacks from 0x02042D99
         * to a clean "movs r0, #0; bx lr" stub (MT6895_DCM_DEVAPC_INIT_OFF | 1).
         */
        stl_le_p((uint8_t *)dest2_f10 + MT6895_PLL_FREQ_LIT_OFF_1,
                 (uint32_t)(MT6895_SRAM1_BASE + 0xF10 +
                            MT6895_DCM_DEVAPC_INIT_OFF + 1));
        stl_le_p((uint8_t *)dest2_f10 + MT6895_PLL_FREQ_LIT_OFF_2,
                 (uint32_t)(MT6895_SRAM1_BASE + 0xF10 +
                            MT6895_DCM_DEVAPC_INIT_OFF + 1));

        /*
         * The DRAM stage's first argument, which its entry tests before doing
         * anything else: "ldr.w r8, [r7, #36]" / "cmp.w r8, #0" / "beq" sends
         * a zero down a path that prints two messages and returns, and
         * execution tracing confirms the rest of the stage never runs.
         *
         * r7 comes from the prologue, which turns the stack the SRAM stage
         * left at 0x020FFFFC into r7 = 0x020FFFF4:
         *
         *   0x020FFFFC - 20 (five-register push) = 0x020FFFE8
         *   0x020FFFE8 + 12                     = 0x020FFFF4
         *
         * so the argument is read from 0x020FFFF4 + 36 = 0x02100018, i.e.
         * SRAM1 offset 0x100018. The caller that fills this on hardware is not
         * the SRAM stage (it jumps rather than calls, leaving an empty stack),
         * so the BootROM provides it, as it does the stack size and UART base.
         */
        stl_le_p((uint8_t *)dest2 + MT6895_DRAM_STAGE_ARG_OFF,
                 MT6895_BOOT_ARG_BASE);

        /*
         * The DRAM calibration block's "initialised" marker.
         *
         * A routine at 0x02034B86 loads 0x02075DC8 and compares it against
         * 256; when it differs, control goes to the path that memsets 256
         * bytes, writes the 0xABCD1234 / 0x5678EF90 magic pair, and prints
         * "dram fatal exception found -> clear calibration data". The image
         * ships 0xDF2 there, so that is the path taken. On hardware the
         * BootROM leaves the marker set once it has a calibration block.
         */
        stl_le_p((uint8_t *)dest2 + MT6895_DRAM_STAGE_CAL_MARKER_OFF, 256);

        /*
         * The pointer a region-descriptor check dereferences.
         *
         * The routine at 0x020345E8 begins by loading 0x020F3720 and testing
         * it; execution tracing of a working run shows exactly two blocks
         * before the "ign need to be power of 2" print at 0x02034688 --
         * 0x02034620 ("ldr.w r0, [r8]" with r8 = 0x020F3720) and 0x02034624
         * ("cbz r0") -- while the whole comparison chain after it
         * (0x0203462C..0x02034672) never runs. So a zero there is what takes
         * the error path. It sits inside the payload's BSS clear range, so it
         * is written here, after the copies above.
         */
        stl_le_p((uint8_t *)dest2 + MT6895_DRAM_STAGE_REGION_PTR_OFF,
                 MT6895_SRAM1_BASE);

        /*
         * The object 0x0200CFCC hands back as the thing to operate on.
         *
         * That routine compares two registers for a {1, 2} pairing and never
         * reads this memory, so a non-zero pointer is all the lookup needs --
         * the first word here is the value the caller passes (1).
         *
         * Filling in a guessed layout on top of that (type at +0, handle at
         * +4, a page count at +8) broke the boot badly: serial output fell
         * from 1111 bytes to 156, and the number of executed blocks from 1158
         * to 648. The consumers at 0x0202C882 and 0x0202CAD0 do read those
         * offsets, but they evidently expect values this machine cannot guess
         * -- so the object is left as the minimal thing that works rather than
         * something invented.
         */
        /*
         * EXPERIMENT (MTK_NO_FAKE_BDEV=1): do not plant the fake RKP block
         * device. Real hardware has the preloader's own UFS driver create it,
         * so this flag lets us check whether our UFS model can serve that path.
         */
        /*
         * Permanently disabled: the preloader builds its block device from its
         * own UFS driver (real-hardware behaviour) and the boot reaches the
         * same point without this fake object.
         */
        if (0) {
            uint8_t *obj = (uint8_t *)dest2 + MT6895_RKP_TABLE_OFF;
            uint8_t *stub = (uint8_t *)dest2 + MT6895_RKP_STUB_OFF;

            /*
             * The object the block I/O routine at 0x0200D010 works on.
             *
             *   +0x00  type   -- 0x0202CAD0 requires 1 or 2
             *   +0x04  block size -- read into sl and rejected when zero
             *                       ("cmp.w sl, #0" / "beq" to the failure
             *                       exit); the release path's ">> 9" says 512
             *   +0x20  I/O callback -- loaded and reached with "blx ip", its
             *                       return value tested with "cmp r0, #0" and
             *                       "bne" to the failure exit, i.e. **zero
             *                       means success**
             *
             * Two earlier attempts filled the size in without the callback and
             * both collapsed the boot (1111 -> 156 bytes): they reached the
             * "blx" with a zero pointer. The callback is what was missing, and
             * the payload has no function this machine may point at, so a
             * two-instruction stub is placed in SRAM1 instead:
             *
             *     movs r0, #0     ; success
             *     bx   lr
             */
            memset(obj, 0, MT6895_RKP_TABLE_ENTRIES * 8);
            stl_le_p(obj + 0x00, MT6895_RKP_OBJ_TYPE);
            stl_le_p(obj + 0x04, MT6895_RKP_BLOCK_SIZE);
            /*
             * +0x14 is the data source the I/O routine reads. With a
             * -machine data= image it points at that; otherwise it keeps
             * pointing at the GPT the callback copies from.
             */
            stl_le_p(obj + 0x14, s->data_path ? MT6895_DATA_ADDR
                                              : MT6895_RKP_GPT_ADDR);
            /*
             * +0x20 and +0x24 are both function pointers the I/O path calls:
             *
             *   0x0200D412  ldr r6, [r4, #32] / 0x0200D424 blx r6
             *   0x0200D432  ldr r6, [r4, #36] / 0x0200D440 blx r6
             *
             * With only +0x20 set, the second call went through a null pointer
             * and the CPU jumped to 0. Both get the stub.
             */
            stl_le_p(obj + 0x20, MT6895_RKP_STUB_ADDR); /* +32: callback */
            stl_le_p(obj + 0x24, MT6895_RKP_STUB_ADDR); /* +36: callback */

            /*
             * A complete GPT header, so the partition parse has consistent
             * fields to work with rather than just the signature.
             */
            mt6895_build_gpt_header((uint8_t *)dest2 + MT6895_RKP_GPT_OFF);

            /*
             * The callback only has to report success.
             *
             * Two mistakes were made here before, both worth recording. First
             * the callback was left null, which crashed on "blx ip"; then the
             * stub was made to write the GPT signature into [r1] -- but r1 is
             * not the buffer. Re-reading the call sequence shows what is:
             *
             *   0x0200D026  ldr r0, [r0, #20]     ; the object's own pointer
             *   0x0200D0DA  mov r0, r8            ; r0 = object
             *   0x0200D0DE  blx ip                ; r1 holds a stale value
             *
             * so the data lives wherever *(obj+20) points, and the machine can
             * simply place the GPT there. The stub just returns zero.
             */
            /*
             * EVIDENCE STUB (temporary): the callback records the arguments
             * it is handed, so the run can be inspected afterwards instead of
             * guessing which register carries the buffer.
             *
             *   ldr.w ip, [pc, #32]   ; ip = 0x0207B300
             *   str.w r0..r3, [ip]    ; the four register arguments
             *   mov   r0, sp
             *   str.w r0, [ip, #16]   ; the stack pointer
             *   ldr   r0, [r0]
             *   str.w r0, [ip, #20]   ; the first stacked word
             *   movs  r0, #0
             *   bx    lr
             */
            /*
             * The stub copies the GPT header to the pointer the callback is
             * handed in r3 and reports success.
             *
             * An evidence stub that recorded its arguments settled which
             * register that is: r0 is the object, r1 is 0xFFFFFFFF (a stale
             * value, not a buffer), r2 is 1 and **r3 is 0x00102180** -- an
             * SRAM0 address, and the very start of the first range the SRAM
             * stage clears. Writing to [r1] earlier therefore corrupted
             * unrelated memory, which is why the boot collapsed.
             *
             * Assembled (28 bytes, Thumb):
             *   mov   r1, r3        ; destination
             *   ldr   r2, [pc, #20] ; source = 0x0207B200
             *   movs  r3, #92
             *   ldrb/strb/adds/adds/subs/bne
             *   movs  r0, #0 / bx lr
             */
            /*
             * The stub copies the GPT header to the buffer the callback is
             * handed in r3, and writes the caller's r2 into the header's
             * current-LBA field (+24).
             *
             * The parse runs twice. The first run compares +24 against the
             * value its caller passed and succeeds; the second run fails at
             * the same comparison, which means the two runs do not expect the
             * same LBA. Rather than guess a second constant, the stub takes
             * the value from the callback's own r2 -- whichever LBA the caller
             * is asking for is the one that is written.
             *
             * Assembled (36 bytes, Thumb):
             *   push {r4,r5,lr}
             *   mov  r4, r2        ; save the caller's value
             *   mov  r5, r3        ; destination
             *   ... copy 92 bytes from 0x0207B200 ...
             *   str  r4, [r5, #24] ; current LBA
             *   movs r0, #0 / pop {r4,r5,pc}
             */
            /*
             * EVIDENCE STUB (temporary): records every callback invocation.
             *
             * A counter at 0x0207B400 is incremented, and each call's four
             * register arguments, its stack pointer and the first stacked word
             * are written to a 32-byte slot at 0x0207B300 + n*32. Reading that
             * region after a run shows exactly what each invocation was given,
             * which is what the two different parse runs need to be told apart.
             *
             * Assembled (36 bytes, Thumb):
             *   push {r4,r5,lr}
             *   ldr  r4, =0x0207B400 / ldr r5,[r4] / adds r5,#1 / str r5,[r4]
             *   lsls r5, r5, #5 / ldr r4, =0x0207B300 / add r4, r5
             *   stmia r4!, {r0,r1,r2,r3}
             *   mov r0, sp / str r0,[r4] / ldr r0,[r0] / str r0,[r4,#4]
             *   pop {r4,r5,pc}
             */
            /*
             * Minimal stub: report success and do nothing else.
             *
             * Three stub shapes have now been measured, and the least
             * intrusive one gives the most serial output:
             *
             *   copy the GPT header into the buffer  1096 bytes
             *   record the arguments (no data write) 1206 bytes
             *   return zero only                     (this run)
             *
             * so the callback is left as just "movs r0,#0 / bx lr" until there
             * is evidence that writing anything helps.
             */
            /*
             * Read-only evidence stub: records the first two words of the
             * buffer it is handed in r3 (destination 0x0207B500) and reports
             * success. It writes nothing into the buffer, so the run stays
             * comparable to the minimal stub while showing whether the data
             * is already there when the callback runs.
             *
             * Assembled (20 bytes, Thumb):
             *   push {r4,r5,lr}
             *   ldr  r4, =0x0207B500
             *   ldr  r5, [r3, #0] / str r5, [r4, #0]
             *   ldr  r5, [r3, #4] / str r5, [r4, #4]
             *   movs r0, #0 / pop {r4,r5,pc}
             */
            /*
             * Copy the machine's GPT (header plus 128 real partition entries)
             * into the buffer the callback is handed in r3, then report
             * success.
             *
             * Earlier attempts wrote into [r1] (a stale register, not the
             * buffer) and pre-filled the buffer at machine init (which the
             * payload's own SRAM clear wipes). This one copies from
             * MT6895_GPT_SOURCE, a region a probe confirmed survives.
             *
             * Assembled (32 bytes, Thumb):
             *   push {r4,r5,lr} / mov r4, r3 / ldr r5, =SRC
             *   movs r3, #0x5c (92) / byte loop -> header
             *   movs r3, #0 (low) ... second loop -> entries at +1024
             */
            /*
             * Copy the machine's GPT into the buffer, then stamp a marker pair
             * over the first eight bytes of the entry area.
             *
             * The marker is what makes the timing observable: the entry copy
             * demonstrably runs, yet the area reads back as zeros afterwards,
             * so either the payload clears it later or the copy never lands.
             * Writing a recognisable value *after* the copy tells the two
             * apart -- if the marker survives the run, nothing wiped the area
             * behind the callback; if it is gone too, the wipe happens later.
             */
            /*
             * EVIDENCE STUB (temporary): records, on every invocation, what
             * the buffer held *before* the callback touched anything.
             *
             * It copies nothing, so whatever the payload left in the buffer is
             * what gets sampled -- that is the point. Earlier experiments
             * could not tell "the payload had already cleared it" from "the
             * callback never wrote it" because the callback wrote first. This
             * one only looks, and records a per-call time series:
             *
             *   counter  0x0207F600
             *   slot     n*16 at 0x0207F630: {r3, entry[0], entry[1], hdr[0]}
             *
             * Assembled (48 bytes, Thumb).
             */
            /*
             * Copy the machine's GPT into the buffer the callback is handed in
             * r3, then report success.
             *
             * A read-only probe settled the data question (see the machine's
             * gpt= property): the buffer is empty on every one of the ten
             * callback invocations, so the callback is the only thing that can
             * put the partition table there.
             *
             * Assembled (56 bytes, Thumb): header (92 bytes) to [r3], entry
             * array (16384 bytes) to [r3 + 1024].
             */
            /*
             * Copy the GPT, then record the header's CRC-field word for this
             * invocation: counter at 0x0207F600, the word for call n at
             * 0x0207F700 + n*4.
             *
             * The parse is entered twice. The first entry's CRC comparison
             * passes and the second one fails, even though the callback writes
             * the same bytes both times -- so the question is what the header
             * looks like on each call. The parser clears +16..+19 while it
             * validates and only restores them on the way out, so a call that
             * observes a zero there is a call that ran mid-validation.
             *
             * Assembled (80 bytes, Thumb).
             */
            /*
             * RAM copy of the block-read callback, kept byte-for-byte in sync
             * with stub_def above. The CPU actually executes the overlay at
             * MT6895_RKP_STUB_ADDR (which shadows this RAM copy), but the two
             * must not diverge or the next reader will be misled.
             *
             * Full block-device semantics: dst = r3, len = r2 * 512,
             * src = 0x50000000 + r1 * 512 for LBA 0..33 (GPT source window),
             * src = 0x80000000 + r1 * 512 for LBA 34+ (data= image at real
             * disk LBAs) while that offset still fits in 32 bits; offsets
             * >= 2 GiB are skipped (a 32-bit guest cannot address them).
             * Returns 0.  Assembled (54 bytes, Thumb) -- see the stub_def
             * comment above for the instruction listing.
             */
            static const uint8_t copy_stub[] = {
                0x10, 0xb5,              /* push {r4, lr}               */
                0x1c, 0x46,              /* mov  r4, r3                 */
                0x49, 0x02,              /* lsls r1, r1, #9             */
                0x52, 0x02,              /* lsls r2, r2, #9             */
                0x44, 0xf2, 0x00, 0x43,  /* movw r3, #0x4400            */
                0x99, 0x42,              /* cmp  r1, r3                 */
                0x02, 0xd2,              /* bhs  @data                  */
                0x4f, 0xf0, 0xa0, 0x40,  /* mov.w r0, #0x50000000       */
                0x05, 0xe0,              /* b    @copy                  */
                0x4f, 0xf0, 0x00, 0x43,  /* @data: mov.w r3, #0x80000000 */
                0x99, 0x42,              /* cmp  r1, r3                 */
                0x09, 0xd2,              /* bhs  @done                  */
                0x4f, 0xf0, 0x00, 0x40,  /* mov.w r0, #0x80000000       */
                0x08, 0x44,              /* @copy: add  r0, r1          */
                0x2a, 0xb1,              /* cbz  r2, @done              */
                0x10, 0xf8, 0x01, 0x1b,  /* loop: ldrb r1, [r0], #1     */
                0x04, 0xf8, 0x01, 0x1b,  /* strb r1, [r4], #1           */
                0x52, 0x1e,              /* subs r2, r2, #1             */
                0xf9, 0xd1,              /* bne  loop                   */
                0x00, 0x20,              /* @done: movs r0, #0          */
                0x10, 0xbd,              /* pop  {r4, pc}               */
            };

            memcpy(stub, copy_stub, sizeof copy_stub);





        }


        /*
         * The memory descriptor table the DRAM stage looks regions up in.
         *
         * Its lookup routine (0x02036E30) reads a base pointer from 0x020F33BC
         * and then, at base + 0x7CCF8, an entry count; entries follow at
         * base + 0x7CD00, 0x98 bytes each, matched on a 64-bit key. With the
         * base pointer left at zero (0x020F33BC falls inside the payload's own
         * BSS clear, 0x020F1000..0x020FB870) the count reads zero, the lookup
         * fails and the stage prints "md fail" through the routine at
         * 0x02036098, whose r0 literal is exactly that string.
         *
         * The descriptor layout comes from the format string the stage prints
         * with: "=start: 0x%llx, sz: 0x%llx map:%d name:%s". One entry
         * describing the DRAM window is the minimum that lets the lookup
         * succeed; the real BootROM fills this from the DRAM topology.
         */
        {
            uint32_t mbase = MT6895_SRAM1_BASE;
            uint8_t *tbl;

            stl_le_p((uint8_t *)dest2 + MT6895_DRAM_STAGE_MEMDESC_PTR_OFF,
                     mbase);

            tbl = (uint8_t *)dest2 + MT6895_DRAM_STAGE_MEMDESC_COUNT_OFF;
            stl_le_p(tbl, 2);                       /* two entries */

            /*
             * Entry layout, from the lookup and the format string the stage
             * prints with ("=start: 0x%llx, sz: 0x%llx map:%d name:%s"):
             *
             *   +0x00  start  (64-bit; the lookup compares r0/r1 against this)
             *   +0x08  size   (64-bit)
             *   +0x10  map    (32-bit)
             *
             * There is no separate key field: inserting one shifted every value
             * the stage printed, start landing where size had been. The two
             * callers differ in how they treat a zero argument -- 0x02030D44
             * skips the lookup when r0|r1 is zero, 0x02030F6A calls anyway with
             * the key it read from 0x02075D30 (zero in the image) -- so what a
             * zero key selects is still open.
             */
            /*
             * Two entries: one for the DRAM window itself, and one under key
             * zero. The caller at 0x02030F6A reads its key from 0x02075D30 --
             * zero in the image -- and calls the lookup unconditionally, so a
             * table without a zero-start entry fails that lookup and the stage
             * takes its "md fail" path and returns to the BootROM without
             * reaching the rest of its work.
             */
            /*
             * Entries are {start, size, map}; the driver rewrites them as it
             * works (see the transfer routine at 0x02036390), so this is
             * only a starting state.
             *
             * Sizes were briefly changed to a power of two (0x40000000) on
             * the theory that the "align need to be power of 2" check reads
             * them: serial output stayed at exactly 811 bytes with the
             * message still present, so that theory is wrong and the check
             * takes its length from somewhere else.
             */
            tbl = (uint8_t *)dest2 + MT6895_DRAM_STAGE_MEMDESC_ENTRY_OFF;
            stq_le_p(tbl + 0x00, 0);                     /* start: matches key 0 */
            stq_le_p(tbl + 0x08, MACHINE(s)->ram_size);  /* size  */
            stl_le_p(tbl + 0x10, 0);                     /* map   */

            tbl += MT6895_DRAM_STAGE_MEMDESC_STRIDE;
            stq_le_p(tbl + 0x00, MT6895_DRAM_BASE);      /* start: DRAM window */
            stq_le_p(tbl + 0x08, MACHINE(s)->ram_size);  /* size  */
            stl_le_p(tbl + 0x10, 0);                     /* map   */

            /*
             * Table 1: DRAM memory descriptor table looked up by 0x02036120.
             * Count at base + 0x7C0E8, entries at base + 0x7C0F0, stride 24 bytes.
             * Entry layout: { uint64_t start, uint64_t size, uint32_t map, uint32_t flags }.
             */
            tbl = (uint8_t *)dest2 + MT6895_DRAM_STAGE_DRAM_DESC_COUNT_OFF;
            stl_le_p(tbl, 2);

            tbl = (uint8_t *)dest2 + MT6895_DRAM_STAGE_DRAM_DESC_ENTRY_OFF;
            stq_le_p(tbl + 0x00, MT6895_DRAM_BASE);      /* start: 0x80000000 */
            stq_le_p(tbl + 0x08, MACHINE(s)->ram_size);  /* size */
            stl_le_p(tbl + 0x10, 0);                     /* map */
            stl_le_p(tbl + 0x14, 0);                     /* flags */

            tbl += MT6895_DRAM_STAGE_DRAM_DESC_STRIDE;
            stq_le_p(tbl + 0x00, 0);                     /* start: 0 */
            stq_le_p(tbl + 0x08, MACHINE(s)->ram_size);  /* size */
            stl_le_p(tbl + 0x10, 0);                     /* map */
            stl_le_p(tbl + 0x14, 0);                     /* flags */

            info_report("mt6895: BootROM: memory descriptor table at 0x%x "
                        "(base 0x%x, 1 entry: 0x%" PRIx64 " + 0x%" PRIx64 ")",
                        (unsigned)(mbase + MT6895_DRAM_STAGE_MEMDESC_ENTRY_OFF),
                        (unsigned)mbase,
                        (uint64_t)MT6895_DRAM_BASE,
                        (uint64_t)MACHINE(s)->ram_size);
        }
    }

    s->boot_entry = (uint64_t)addr + entry_off;
    s->boot_spsr = MT6895_PRELOADER_SPSR;
    if (s->boot_r4 == 0) {
        /*
         * Default to the AArch32 service entry with its low bit set, so the
         * payload's "blx r4" enters it in Thumb state. Overridable with
         * -machine boot-r4=<value>.
         */
        s->boot_r4 = MT6895_BOOTROM_A32_SERVICE;
    }
    /*
     * Model-side experiment switch (default OFF keeps the tree bootable):
     *   unset        -> r4 = A32 service entry, builder writes nothing;
     *   "control"    -> r4 = descriptor window, builder writes NOTHING
     *                   (isolates zero-vs-stale descriptor content);
     *   "1"/"fill"   -> r4 = descriptor window, builder writes the justified
     *                   fields only (never a memset).
     */
    {
        const char *m = getenv("MTK_BROMARGS_BLOCK");

        if (m && !strcmp(m, "control")) {
            mt6895_build_brom_bootargs(s, false);
            s->boot_r4 = MT6895_BROM_BOOTARGS_PA;
        } else if (m && *m && strcmp(m, "0")) {
            mt6895_build_brom_bootargs(s, true);
            s->boot_r4 = MT6895_BROM_BOOTARGS_PA;
        } else {
            mt6895_build_brom_bootargs(s, false);
            s->boot_r4 = MT6895_BOOTROM_A32_SERVICE;
        }
    }
    s->boot_valid = true;

    info_report("mt6895: preloader 0x%x bytes loaded at 0x%x, entry 0x%"
                PRIx64 " (from '%s')",
                size, addr, s->boot_entry, s->preloader_path);
}

static void mt6895_write_boot_desc(Mt6895MachineState *s)
{
    uint8_t *p = memory_region_get_ram_ptr(&s->boot_desc);

    if (!s->boot_valid) {
        return;
    }

    stq_le_p(p + MT6895_BOOT_DESC_ENTRY, s->boot_entry);
    stq_le_p(p + MT6895_BOOT_DESC_SPSR, s->boot_spsr);
    stq_le_p(p + MT6895_BOOT_DESC_R4, s->boot_r4);
    stq_le_p(p + MT6895_BOOT_DESC_LR, s->boot_lr);
}

/*
 * EXPERIMENT (2026-11-03): the preloader requests a warm reset via RMR to
 * hand over to the AArch64 BL2_EXT image it loaded at MT6895_BL2_EXT_ADDR.
 * After such a reset, point the BootROM stub at that image in AArch64 EL3.
 */
static void mt6895_gic_bootstrap(Mt6895MachineState *s);

/*
 * ATF per-CPU state for cores QEMU starts directly (MTK_DT_HVC_PSCI /
 * MTK_PSCI_HOST): such cores never run ATF's warm-boot entry at 0x48200100,
 * which would normally set TPIDR_EL3, the EL3 stack and the per-CPU
 * platform-data pointer.  Without them, the first SiP SMC issued on that core
 * (Linux kworkers issue UFS/GCE/mtee/hwrng SMCs from any CPU) faults inside
 * ATF; EL3 then spins/panics and every following SMC can wedge as well.  The
 * visible symptom was init stalling while loading /lib/modules/sec-rng.ko
 * once 3+ CPUs were online.
 *
 * Values taken from the ATF image disassembly:
 *   TPIDR_EL3 per-CPU data = 0x4824b000 + n * 0x80
 *   SP_EL3 (platform_stack) = 0x482469c0 + n * 0x800
 *   per-CPU platform data pointer [+0x10] = MIDR-table entry
 *       0x4823ef68 (Cortex-A55, CPUs 0..3) / 0x4823efb0 (Cortex-A78, 4..7)
 */
/*
 * Invoked by arm_psci CPU_ON in the target vCPU's own thread, right after its
 * reset work.  See the comment above for why this state is needed.
 */
static void mtk_secondary_atf_init(ARMCPU *cpu, unsigned n)
{
    CPUState *cs = CPU(cpu);
    CPUState *cs0 = qemu_get_cpu(0);
    uint64_t tp = 0x4824b000ULL + (uint64_t)n * 0x80;

    /*
     * The warm-boot entry also leaves the EL3 exception/MMU world set up
     * (VBAR_EL3 -> ATF runtime vectors at 0x4822b800, SCTLR_EL3.M=1 with
     * TTBR0_EL3/TCR_EL3/MAIR_EL3 describing the ATF tables).  A core that
     * QEMU reset directly has none of that, so the very first SMC raised to
     * EL3 would vector through VBAR_EL3=0.  Clone CPU0's (cold-booted) EL3
     * state instead of guessing.
     */
    if (cs0 && cs0 != cs) {
        ARMCPU *cpu0 = ARM_CPU(cs0);

        cpu->env.cp15.vbar_el[3] = cpu0->env.cp15.vbar_el[3];
        cpu->env.cp15.ttbr0_el[3] = cpu0->env.cp15.ttbr0_el[3];
        cpu->env.cp15.tcr_el[3] = cpu0->env.cp15.tcr_el[3];
        cpu->env.cp15.mair_el[3] = cpu0->env.cp15.mair_el[3];
        cpu->env.cp15.sctlr_el[3] = cpu0->env.cp15.sctlr_el[3];
        cpu->env.cp15.scr_el3 = cpu0->env.cp15.scr_el3;
        cpu->env.cp15.cptr_el[3] = cpu0->env.cp15.cptr_el[3];
        cpu->env.cp15.mdcr_el3 = cpu0->env.cp15.mdcr_el3;
    }
    cpu->env.cp15.tpidr_el[3] = tp;
    cpu->env.sp_el[3] = 0x482469c0ULL + (uint64_t)n * 0x800;
    /*
     * ATF's sync-exception handler switches to the "lower EL stack" it finds
     * at [SP_EL3 + 0x110] (set by el3_exit on a core that has already run
     * ATF).  A QEMU-started core never did, so the slot is zero and the first
     * SMC does "stp x0, x1, [sp, #-0x60]!" with sp = 0 -> EL3 data abort at
     * 0xffffffffffffffa0 (far_el3 seen in the pl5 panic).  Point it at this
     * core's own EL3 stack scratch area; ATF overwrites it with the real
     * value after the first el3_exit.
     */
    address_space_stq_le(&address_space_memory,
                         cpu->env.sp_el[3] + 0x110,
                         cpu->env.sp_el[3] + 0x700,
                         MEMTXATTRS_UNSPECIFIED, NULL);
    /* Per-CPU platform-data pointer, selected by the core's MIDR class. */
    address_space_stq_le(&address_space_memory, tp + 0x10,
                         n < MT6895_NUM_A55 ? 0x4823ef68ULL
                                            : 0x4823efb0ULL,
                         MEMTXATTRS_UNSPECIFIED, NULL);
    if (MTK_ENV_CACHED("MTK_ATFDBG")) {
        uint64_t slot = address_space_ldq_le(&address_space_memory,
                                             cpu->env.sp_el[3] + 0x110,
                                             MEMTXATTRS_UNSPECIFIED, NULL);
        fprintf(stderr, "MTK-ATFPLANT cpu%u tp=%#" PRIx64 " sp=%#" PRIx64
                " vbar=%#" PRIx64 " sctlr=%#" PRIx64 " stack[+110]=%#"
                PRIx64 "\n",
                n, tp, cpu->env.sp_el[3], cpu->env.cp15.vbar_el[3],
                cpu->env.cp15.sctlr_el[3], slot);
    }
}

extern uint64_t mtk_bl_params_ptr;

static void mt6895_machine_reset(MachineState *machine, ResetType type)
{
    Mt6895MachineState *s = MT6895_MACHINE(machine);

    {
        CPUState *cs = first_cpu;
        ARMCPU *ac = cs ? ARM_CPU(cs) : NULL;
        fprintf(stderr, "MT6895-DBG machine_reset type=%d warm=%d pc_before=%#x\n",
                (int)type, (int)arm_mt6895_warm_reset,
                ac ? (unsigned)ac->env.regs[15] : 0);
    }
    /*
     * On MT6895 hardware, a Watchdog Software Reset (WDT SWRST key 0x1209)
     * asserts system reset to all digital peripheral controllers, GIC,
     * and CPU cores. Preserved across warm reset are DRAM contents (retained
     * in self-refresh), RGU NONRST registers (in s->wdt_store), PMIC/RTC
     * always-on registers, and SRAM. Skipping device reset left Linux's GIC
     * redistributor PPI enables (0x880207f) and active timers running, and left
     * mtk_ufs_lk_stage true, which routed UFS completion interrupts into
     * BL2_EXT as Group 1 NS FIQs and triggered the platform_fiq panic.
     * Always reset devices and reset mtk_ufs_lk_stage to false.
     * Also reset load-bearing model-side state across warm reset:
     * bl_params pointer, GZ skip flag, DSI streaming engine and timers,
     * HACC2 crypto done flag, GPUEB and SPM request flags.
     */
    qemu_devices_reset(type);
    s->atfctx_val = 0;
    mtk_ufs_lk_stage = false;
    mtk_bl_params_ptr = 0;
    if (s->dsi_frame_timer) {
        timer_del(s->dsi_frame_timer);
    }
    s->dsi_streaming = false;
    s->dsi_irq_level = false;
    if (s->dsi_irq) {
        qemu_irq_lower(s->dsi_irq);
    }
    memset(s->dsi_regs, 0, sizeof(s->dsi_regs));
    memset(s->dsi_rx, 0, sizeof(s->dsi_rx));
    s->dsi_ulps_done = false;
    s->dsi_sleep_out = false;
    s->dsi_display_on = false;
    s->dsi_cmdq_ready = false;
    mt6895_hacc2_done = false;
    mt6895_gpueb_started = false;
    mt6895_spm_req_pending = false;
    if (mt6895_warm_reset_requested) {
        fprintf(stderr, "mt6895: WDT software reset -- devices reset, PMIC/DRAM preserved\n");
    }
    /*
     * The GZ EMI/infra-IO remap registers live inside the hypervisor's own
     * register block and reset with it: carrying boot#1's remap selection
     * into boot#2 gave the second preloader pass an environment the firmware
     * never set up (r1315 ISO2/ISO1 boot#2 preloader data abort).
     */
    memset(s->gz_remap_regs, 0, sizeof(s->gz_remap_regs));
    memory_region_set_enabled(&s->gz_ddr_remap, false);
    mt6895_cpc_state_reset();
    {
        bool was_warm = mt6895_warm_reset_requested;

        mt6895_warm_reset_requested = false;

    /*
     * Latch the watchdog reboot in the PMIC, the way the real part does.
     * pmic_check_rst() classifies the boot from these status registers and
     * falls back to "Cold Reset" when they read zero, which makes the DRAM
     * stage throw away the saved calibration on every reset.
     */
    if (s->spmi) {
        /*
         * Latch reset-cause status in the PMIC the way the real part does.
         * pmic_check_rst() classifies the boot from these registers
         * (PONSTS/POFFSTS/WDTRSTB) and falls back to "Cold Reset" when they
         * read zero, which is what makes the DRAM stage drop the saved
         * calibration on every reset.
         *
         * MTK_PMIC_REGS takes a comma list of "sid:reg:val" hex triples so
         * the encoding can be swept without rebuilding.
         */
        const char *e = getenv("MTK_PMIC_REGS");
        const char *p = e;
        int applied = 0;

        while (p && *p) {
            unsigned sid = 0, reg = 0, val = 0;
            char *end;

            sid = strtoul(p, &end, 16);
            if (*end != ':') {
                break;
            }
            p = end + 1;
            reg = strtoul(p, &end, 16);
            if (*end != ':') {
                break;
            }
            p = end + 1;
            val = strtoul(p, &end, 16);
            mtk_spmi_pmif_set_pmic_reg(s->spmi, sid, reg, (uint8_t)val);
            applied++;
            p = (*end == ',') ? end + 1 : NULL;
        }
        if (!applied && !e && was_warm) {
            /*
             * Default after a warm reset: POFFSTS records how the previous
             * boot ended.  pmic_check_rst() reads it and, on the bits that
             * mean "watchdog", reports BWDT -- which is what the DRAM stage
             * needs to treat this as a reboot instead of a cold boot.
             */
            mtk_spmi_pmif_set_pmic_reg(s->spmi, 4, 0x0d, 0xff);
            mtk_spmi_pmif_set_pmic_reg(s->spmi, 4, 0x139, 1);
        }
    }
    }
    {
        CPUState *cs;
        int n = 0;
        CPU_FOREACH(cs) {
            resettable_reset(OBJECT(cs), type);
            ARM_CPU(cs)->env.cp15.tpidr_el[3] = 0x4824b000ULL + (uint64_t)n * 0x80;
            n++;
        }
    }
    /*
     * The BootROM reloads the payload from the boot LUN on every reset.  The
     * machine used to copy it once at start-up, so a warm reset restarted the
     * CPU on an SRAM image the previous boot had already overwritten -- the
     * second boot then executed stale code and took a wild write (observed as
     * a permission fault at 0x3F74).  Re-run the load so every reset begins
     * from the same image the power-on boot used.
     *
     * Not on an RMR warm reset, though.  That is how the AArch32 preloader
     * hands over to the AArch64 BL2_EXT (RMR.AA64 + RMR.RR, see
     * mtk_rmr_write()): only the core restarts, at its reset vector, so the
     * BootROM loads nothing and SRAM keeps what the preloader left in it.
     * BL2_EXT validates two of those things first: the AEE memory-info block
     * at 0x11DEC0 ("addr"/"size" tags) and the log-store header at 0x11DF00,
     * both inside the SRAM0 window the payload is copied into.  The preloader
     * builds them correctly (it prints "magic1:0x61646472, magic2:0x73697A65"
     * after reading them back), but reloading here copied the payload's own
     * bytes over them, so BL2_EXT failed with "LOG_STORE_LK: log store init
     * failed" and "[MBOOT_PARAMS]: illegal magic number:[0xf10bd106,
     * 0x21004620]" -- the image words at file offsets 0x1EEC0 / 0x1EEFC.
     */
    if (!arm_mt6895_warm_reset) {
        Error *rerr = NULL;

        /*
         * on_reset=true, not false: the MTK_RESET_KEEP_SRAM0 gate inside
         * mt6895_load_boot_lun() only applies when the caller says this is a
         * reset.  It was passed false here, so the gate was dead code and the
         * whole payload image was re-copied into SRAM0 on every reset --
         * which also erased anything the previous boot had stored in the
         * boot-args SRAM (e.g. the mboot params BL2_EXT later validates at
         * physical 0x11d000).  With true, the default behaviour is unchanged
         * (the env is off by default) and the gate actually works.
         */
        mt6895_load_boot_lun(s, &rerr, true);
        if (rerr) {
            error_reportf_err(rerr, "mt6895: reload on reset: ");
            error_free(rerr);
        } else {
            mt6895_write_boot_desc(s);
            fprintf(stderr, "mt6895: payload reloaded for this reset\n");
        }
    } else {
        fprintf(stderr, "mt6895: RMR warm reset -- payload not reloaded, "
                "SRAM contents kept\n");
    }
    /*
     * The device reset above puts the GIC back to all-zero, which would undo
     * the BL2-style bring-up. Re-apply it here, after the reset.
     */
    mt6895_gic_bootstrap(s);
    if (s->gic_group_timer) {
        timer_mod(s->gic_group_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 2000000);
    }
    {
        CPUState *cs = first_cpu;
        ARMCPU *ac = cs ? ARM_CPU(cs) : NULL;
        fprintf(stderr, "MT6895-DBG machine_reset after devices_reset pc=%#x "
                "aarch64=%d pstate=%#x\n",
                ac ? (unsigned)ac->env.regs[15] : 0,
                ac ? (int)ac->env.aarch64 : -1,
                ac ? (unsigned)ac->env.pstate : 0);
    }

    if (!s->boot_valid) {
        return;
    }
    if (arm_mt6895_warm_reset) {
        /*
         * RMR warm reset into AArch64.  Only the core restarts: it comes up at
         * RVBAR -- the BL2_EXT entry the preloader programmed ("[TZ_INIT] Jump
         * to BL2_EXT: 0x62F00000") -- in EL3h with DAIF masked, which is the
         * state the CPU reset above has just produced.  The BootROM does not
         * run, so its stub and boot descriptor are bypassed (the descriptor
         * keeps the preloader entry for any later SoC reset).
         *
         * The register file survives, so x0..x3 are r0..r3 as they stood at
         * the WFI where the reset was taken (HELPER(wfi)).  The preloader's
         * jumparch64 leaves r0 = its mtk_bl_param_t pointer {bootarg_loc,
         * bootarg_size, bl33_start_addr, atf_arg_addr} and r1 = 0 there.
         * BL2_EXT saves x0..x3 at 0x62ff2098 on entry and later hands x0 on to
         * BL31 as its boot parameters.  This path used to go through the stub
         * with x0 = 0 (and x1/x2 = the stub's entry/SPSR scratch values), so
         * BL31 never saw any parameters.
         */
        CPUState *cs0 = qemu_get_cpu(0);

        arm_mt6895_warm_reset = false;
        if (cs0) {
            CPUARMState *env0 = cpu_env(cs0);
            unsigned i;

            env0->pc = MT6895_BL2_EXT_ADDR;
            for (i = 0; i < ARRAY_SIZE(arm_mt6895_rmr_regs); i++) {
                env0->xregs[i] = (uint32_t)arm_mt6895_rmr_regs[i];
            }
            fprintf(stderr, "mt6895: RMR warm reset -> BL2_EXT %#llx "
                    "x0=%#" PRIx64 " x1=%#" PRIx64 "\n",
                    (unsigned long long)MT6895_BL2_EXT_ADDR,
                    env0->xregs[0], env0->xregs[1]);
        }
        return;
    }
    mt6895_write_boot_desc(s);
}

static const char *mt6895_cpu_type(int cpu)
{
    return cpu < MT6895_NUM_A55 ? ARM_CPU_TYPE_NAME("cortex-a55")
                                : ARM_CPU_TYPE_NAME("cortex-a78");
}

static const CPUArchIdList *mt6895_possible_cpu_arch_ids(MachineState *ms)
{
    int n;

    if (ms->possible_cpus) {
        assert(ms->possible_cpus->len == MT6895_NUM_CPUS);
        return ms->possible_cpus;
    }

    ms->possible_cpus = g_malloc0(sizeof(CPUArchIdList) +
                                  sizeof(CPUArchId) * MT6895_NUM_CPUS);
    ms->possible_cpus->len = MT6895_NUM_CPUS;

    for (n = 0; n < MT6895_NUM_CPUS; n++) {
    /*
         * MPIDR affinity from the vendor device tree: cpu@0..cpu@300 are the
         * four A55 cores, cpu@400..cpu@700 the four A78 cores.
         */
        ms->possible_cpus->cpus[n].type = mt6895_cpu_type(n);
        ms->possible_cpus->cpus[n].arch_id = (uint64_t)n << 8;
    }

    return ms->possible_cpus;
}

/*
 * PSCI routing:
 * Native ATF PSCI via SMC conduit is the default for all configurations.
 * Secondary CPUs boot cleanly through BL31's warm-boot vector at 0x48200100
 * when requested via the CPC power controller bridge.
 * MTK_PSCI_HOST=1 is preserved purely for legacy A/B comparison.
 */
static bool mt6895_host_psci_mode(MachineState *ms)
{
    (void)ms;
    if (MTK_ENV_CACHED("MTK_PSCI_HOST")) {
        return true;
    }
    return false;
}

/*
 * MTK's Cortex-A55/A78 integration implements registers in the
 * implementation-defined C15 space that the stock QEMU CPU models lack.
 * BL31's per-core bring-up writes S3_0_C15_C3_6 right after arming its
 * per-CPU state (tee_a.img 0x4820448c); without the register the MSR is
 * undefined, ATF panics before finishing the warm boot, and the RGU turns
 * that into a machine reset (r1315 ISO1/ISO2/R3 evidence).  Model it as
 * plain per-CPU RW storage: the firmware uses it as configuration state.
 */
static uint64_t mtk_impdef_c15_c3_6[MT6895_NUM_CPUS];
static uint64_t mtk_impdef_c15_c2_0[MT6895_NUM_CPUS];

static uint64_t mt6895_impdef_c15_c3_6_read(CPUARMState *env,
                                            const ARMCPRegInfo *ri)
{
    return mtk_impdef_c15_c3_6[env_cpu(env)->cpu_index];
}

static void mt6895_impdef_c15_c3_6_write(CPUARMState *env,
                                         const ARMCPRegInfo *ri, uint64_t v)
{
    mtk_impdef_c15_c3_6[env_cpu(env)->cpu_index] = v;
}

static uint64_t mt6895_impdef_c15_c2_0_read(CPUARMState *env,
                                            const ARMCPRegInfo *ri)
{
    return mtk_impdef_c15_c2_0[env_cpu(env)->cpu_index];
}

static void mt6895_impdef_c15_c2_0_write(CPUARMState *env,
                                         const ARMCPRegInfo *ri, uint64_t v)
{
    mtk_impdef_c15_c2_0[env_cpu(env)->cpu_index] = v;
}

static void mt6895_add_impdef_regs(ARMCPU *cpu)
{
    /*
     * s3_0_c15_c3_6: written by BL31's per-core bring-up on every core
     * (tee_a.img 0x4820448c).  s3_6_c15_c2_0: A78-only (guarded by the MIDR
     * part-number compare at tee_a.img 0x482166e0), read-modify-written with
     * bit 54 set during the warm entry.  Both live in the MTK integration's
     * implementation-defined C15 space that the stock QEMU Cortex models
     * lack; the MSR/MRS is undefined without them and ATF panics mid
     * warm-boot, which the RGU turns into a machine reset.
     */
    static const ARMCPRegInfo mtk_impdef_c15_c3_6 = {
        .name = "mtk_impdef_c15_c3_6",
        .crn = 15, .crm = 3, .opc0 = 3, .opc1 = 0, .opc2 = 6,
        .access = PL3_RW,
        .state = ARM_CP_STATE_AA64,
        .type = ARM_CP_NO_RAW,
        .readfn = mt6895_impdef_c15_c3_6_read,
        .writefn = mt6895_impdef_c15_c3_6_write,
    };
    static const ARMCPRegInfo mtk_impdef_c15_c2_0 = {
        .name = "mtk_impdef_c15_c2_0",
        .crn = 15, .crm = 2, .opc0 = 3, .opc1 = 6, .opc2 = 0,
        .access = PL3_RW,
        .state = ARM_CP_STATE_AA64,
        .type = ARM_CP_NO_RAW,
        .readfn = mt6895_impdef_c15_c2_0_read,
        .writefn = mt6895_impdef_c15_c2_0_write,
    };

    define_one_arm_cp_reg(cpu, &mtk_impdef_c15_c3_6);
    define_one_arm_cp_reg(cpu, &mtk_impdef_c15_c2_0);
}

static void mt6895_create_cpus(Mt6895MachineState *s, Error **errp)
{
    MachineState *ms = MACHINE(s);

    for (int n = 0; n < ms->smp.cpus; n++) {
        Object *cpuobj = object_new(ms->possible_cpus->cpus[n].type);
        CPUState *cs = CPU(cpuobj);

        cs->cpu_index = n;
        object_property_set_int(cpuobj, "mp-affinity",
                                ms->possible_cpus->cpus[n].arch_id, &error_fatal);

    /*
         * The BootROM runs at EL3, which is also where QEMU leaves an ARMv8
         * core after reset. EL2 exists on MT6895 but nothing in the boot
         * chain uses it, and SCR_EL3.RW stays 0 so EL2/EL1 are AArch32 --
         * which is what the 32-bit preloader needs.
         */
        object_property_set_bool(cpuobj, "has_el3", true, &error_fatal);
        object_property_set_bool(cpuobj, "has_el2", true, &error_fatal);
        /*
         * The MTK ATF initialises the RAS error-record registers
         * (ERRSELR_EL1/ERXCTLR_EL1 at 0x48205030) as part of its platform
         * setup. Real Cortex-A55/A78 implement ARMv8.2-RAS, so that is legal
         * there; without the feature here the MSR is undefined and ATF takes
         * an "Unhandled Exception in EL3" (esr_el3=0x02000000, EC=0) at
         * elr_el3=0x48205034 and spins in its panic loop.
         */
        /* ATF SMC handler assumes TPIDR_EL3 = 0x4824b000 + cpu*0x80 (set by
         * ATF entry at 0x482056ac). Plant it so the first SMC (before any
         * ATF entry run) does not walk into unmapped space. */
        ARM_CPU(cpuobj)->env.cp15.tpidr_el[3] = 0x4824b000ULL + (uint64_t)n * 0x80;
        /*
         * Let QEMU answer the guest's PSCI calls (SMC conduit) whenever the
         * multi-vCPU bring-up needs it; see mt6895_host_psci_mode().
         */
        object_property_set_uint(cpuobj, "psci-conduit",
                                 mt6895_host_psci_mode(ms)
                                     ? QEMU_PSCI_CONDUIT_SMC
                                     : QEMU_PSCI_CONDUIT_HVC,
                                 &error_fatal);

    /*
         * The MT6895 device tree declares the architected timer at 13 MHz
         * (clock-frequency = <0xc65d40>). QEMU defaults CNTFRQ to 62.5 MHz,
         * and the kernel takes the register in preference to the device tree,
         * so leaving the default would make the guest's timekeeping wrong by
         * a factor of ~4.8.
         */
        object_property_set_uint(cpuobj, "cntfrq", MT6895_CNTFRQ_HZ,
                                 &error_fatal);

    /*
         * Only the boot core leaves reset. The others wait for PSCI
         * CPU_ON, exactly as they do on hardware.
         */
        if (n != 0) {
            object_property_set_bool(cpuobj, "start-powered-off", true,
                                     &error_fatal);
        }
        mt6895_add_impdef_regs(ARM_CPU(cpuobj));

        qdev_realize(DEVICE(cpuobj), NULL, errp);
        object_unref(cpuobj);
    }
}

/*
 * Wire the GIC to the CPUs.
 *
 * Two directions matter. Outwards, each CPU's generic timer drives its own
 * PPIs, which is what wakes a core parked in WFI -- without it the boot CPU
 * waits forever for a secondary's hotplug callback, and any task that sleeps
 * is never woken. Inwards, the GIC drives the CPU's IRQ/FIQ/VIRQ/VFIQ (and
 * NMI/VINMI) inputs.
 *
 * The PPI block for CPU i begins at MT6895_NUM_SPIS + i * GIC_INTERNAL, not at
 * GIC_INTERNAL: the GIC's input array is [0..N-1] SPI followed by the per-CPU
 * PPI blocks, where N = num-irq - GIC_INTERNAL. See the comment in the body.
 */

/*
 * NOTE: the GIC's GPIO input array is [0..N-1] = SPIs (input i is INTID
 * 32+i), then the per-CPU PPI blocks. The *_SPI constants are therefore
 * GIC input indices (INTID - 32): LK registers its UFS handler on IRQ 169,
 * which is input 137, i.e. MT6895_UFSHCI_SPI as passed below.
 */

/*
 * The firmware's LK only programs the GICv3 CPU interface (ICC_SRE_EL1,
 * ICC_PMR_EL1, ICC_IGRPEN1_EL1); it never touches the distributor or the
 * redistributor, because on a real device BL2 leaves those configured.
 *
 * With a reset GIC nothing is enabled, so no interrupt is ever delivered:
 * the architected timer never fires and any thread that sleeps or waits for
 * an interrupt waits forever (LK ends up in its idle wfi loop while the UFS
 * driver's completion is pending).
 *
 * Reproduce BL2's hand-off here: enable Group 1 NS, mark every SPI as
 * Group 1, enable every SPI, and wake the redistributor with its PPIs
 * (including the architected timer) enabled.
 */
/*
 * LK's own GIC init reprograms the SPIs as Group 0 (it runs secure), which
 * turns the UFS interrupt into a secure Group-0 interrupt that its
 * non-secure ICC_IAR1_EL1 path can never see. On a real device BL2 leaves
 * the peripheral SPIs in Group 1, so keep that part of the hand-off pinned.
 */
static void mt6895_gic_group_timer(void *opaque)
{
    Mt6895MachineState *s = opaque;
    AddressSpace *as = &address_space_memory;
    MemTxAttrs attrs = { .secure = 1 };
    unsigned n;

    /*
     * With a security-enabled GIC, an SPI is Group 1 Non-Secure -- the group
     * a non-secure EL1 driver acknowledges through ICC_IAR1_EL1 -- only when
     * IGROUPR bit = 1 and IGRPMODR bit = 0 (gicv3_irq_group()). LK's own GIC
     * init clears IGROUPR, which moves these SPIs to Group 0 and makes the
     * UFS completion interrupt invisible to it; re-pin the BL2 hand-off value
     * (Group 1 NS) so the UFS driver's wait completes.
     *
     * The switch has to wait for the BL33 hand-off: while BL2_EXT (secure) is
     * still running it polls the UFS controller and has FIQ unmasked, so a
     * Group 1 NS completion is taken as an FIQ and panics in platform_fiq().
     */
    {
        uint32_t grp = mtk_ufs_lk_stage ? 0xffffffff : 0x00000000;
        uint32_t mod = mtk_ufs_lk_stage ? 0x00000000 : 0xffffffff;

        for (n = 4; n < 6; n++) {           /* INTIDs 128..191 */
            address_space_stl_le(as, MT6895_GICD_BASE + 0x0080 + 4 * n,
                                 grp, attrs, NULL);
            address_space_stl_le(as, MT6895_GICD_BASE + 0x0d00 + 4 * n,
                                 mod, attrs, NULL);
        }
    }
    timer_mod(s->gic_group_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 2000000);
}

static void mt6895_gic_bootstrap(Mt6895MachineState *s)
{
    AddressSpace *as = &address_space_memory;
    MemTxAttrs attrs = { .secure = 1 };   /* BL2 configures the GIC securely */
    uint32_t val;
    hwaddr gicd = MT6895_GICD_BASE;
    hwaddr gicr = MT6895_GICR_BASE;
    unsigned n;

    /* distributor: EnableGrp0 | EnableGrp1NS | EnableGrp1S | ARE_NS | ARE_S */
    val = 0x37;
    address_space_stl_le(as, gicd + 0x0000, val, attrs, NULL);
    {
        uint32_t back = address_space_ldl_le(as, gicd + 0x0000,
                                             attrs, NULL);
        fprintf(stderr, "MT6895-DBG gic bootstrap: ctlr readback=%#x\n", back);
    }

    /* every SPI in Group 1 (non-secure) */
    for (n = 0; n < (MT6895_NUM_SPIS + 31) / 32; n++) {
        address_space_stl_le(as, gicd + 0x0080 + 4 * n, 0xffffffff,
                             attrs, NULL);
    }
    /* default priority so nothing is masked by the PMR threshold */
    for (n = 0; n < MT6895_NUM_SPIS; n += 4) {
        address_space_stl_le(as, gicd + 0x0400 + n, 0xa0a0a0a0,
                             attrs, NULL);
    }
    /* enable every SPI */
    for (n = 0; n < (MT6895_NUM_SPIS + 31) / 32; n++) {
        address_space_stl_le(as, gicd + 0x0100 + 4 * n, 0xffffffff,
                             attrs, NULL);
    }

    {
        uint32_t ctlr = address_space_ldl_le(as, gicd + 0x0000,
                                             attrs, NULL);
        uint32_t en4 = address_space_ldl_le(as, gicd + 0x0110,
                                            attrs, NULL);
        uint32_t grp4 = address_space_ldl_le(as, gicd + 0x0090,
                                             attrs, NULL);
        uint32_t gicr_en = address_space_ldl_le(as, gicr + 0x10100,
                                                attrs, NULL);
        uint32_t waker = address_space_ldl_le(as, gicr + 0x0014,
                                              attrs, NULL);
        uint32_t typer = address_space_ldl_le(as, gicr + 0x0008,
                                              attrs, NULL);
        uint32_t gicr_ctlr = address_space_ldl_le(as, gicr + 0x0000,
                                                  attrs, NULL);
        fprintf(stderr, "MT6895-DBG gic bootstrap: ctlr=%#x isen169w=%#x"
                " grp169w=%#x gicr_en=%#x waker=%#x typer=%#x rctlr=%#x\n",
                ctlr, en4, grp4, gicr_en, waker, typer, gicr_ctlr);
    }

    /*
     * Redistributors for every CPU, not just CPU0. RD_base holds
     * CTLR/WAKER/TYPER; the SGI frame (PPIs and SGIs) lives at
     * RD_base + 0x10000, RD frames are 0x20000 apart.
     *
     * With a security-enabled GIC, GICR_IGROUPR0/IGRPMODR0 are RAZ/WI to
     * Non-secure accesses, so the NS kernel cannot move a redistributor's
     * PPIs (architected timer!) from their reset state (Group 0) to
     * Group 1 NS. BL2 must do it for every core it hands over, exactly like
     * the SPIs above. Bootstrapping only CPU0 left every secondary's timer
     * PPI as a secure Group-0 interrupt, so secondary CPUs came online but
     * never got a tick (RCU expedited stalls, then the SMP bring-up of the
     * next CPU wedged).
     */
    {
        unsigned int ncpu = MACHINE(s)->smp.cpus;

        for (n = 0; n < ncpu; n++) {
            hwaddr rd = gicr + (hwaddr)n * 0x20000;

            address_space_stl_le(as, rd + 0x0014, 0x0, attrs, NULL);
            address_space_stl_le(as, rd + 0x10080, 0xffffffff, attrs, NULL);
            address_space_stl_le(as, rd + 0x10d00, 0x00000000, attrs, NULL);
            address_space_stl_le(as, rd + 0x10100, 0xffffffff, attrs, NULL);
            address_space_stl_le(as, rd + 0x10400, 0xa0a0a0a0, attrs, NULL);
        }
    }
}

static void mt6895_connect_gic_cpus(Mt6895MachineState *s)
{
    MachineState *ms = MACHINE(s);
    SysBusDevice *gicbusdev = SYS_BUS_DEVICE(s->gic);
    unsigned int smp_cpus = ms->smp.cpus;

    for (unsigned i = 0; i < smp_cpus; i++) {
        DeviceState *cpudev = DEVICE(qemu_get_cpu(i));
    /*
         * The GIC's GPIO input array is laid out [0..N-1] = SPI, then the
         * per-CPU PPI blocks, where N = num-irq - GIC_INTERNAL. This machine
         * asks for MT6895_NUM_SPIS + 32 interrupts, so the PPI blocks begin
         * at MT6895_NUM_SPIS -- not at GIC_INTERNAL, which is only correct
         * for a GIC with 32 SPIs.
         *
         * Wiring the timer outputs to GIC_INTERNAL + ... put every PPI on a
         * line the distributor reads as an SPI, so the architected timer's
         * interrupt went nowhere: /proc/interrupts showed arch_timer with
         * zero counts, and any task that slept -- mmc_power_up()'s mmc_delay()
         * included -- was never woken.
         */
        int intidbase = MT6895_NUM_SPIS + i * GIC_INTERNAL;
        const int timer_irq[] = {
            [GTIMER_PHYS]       = ARCH_TIMER_NS_EL1_IRQ,
            [GTIMER_VIRT]       = ARCH_TIMER_VIRT_IRQ,
            [GTIMER_HYP]        = ARCH_TIMER_NS_EL2_IRQ,
            [GTIMER_SEC]        = ARCH_TIMER_S_EL1_IRQ,
            [GTIMER_HYPVIRT]    = ARCH_TIMER_NS_EL2_VIRT_IRQ,
            [GTIMER_S_EL2_PHYS] = ARCH_TIMER_S_EL2_IRQ,
            [GTIMER_S_EL2_VIRT] = ARCH_TIMER_S_EL2_VIRT_IRQ,
        };

        for (unsigned irq = 0; irq < ARRAY_SIZE(timer_irq); irq++) {
            qdev_connect_gpio_out(cpudev, irq,
                                  qdev_get_gpio_in(s->gic,
                                                   intidbase + timer_irq[irq]));
        }

    /*
         * CPU interrupt lines, in the order the GIC exposes them. The
         * per-CPU block is contiguous, so the strides are 0, 1, 2, 3, ...
         * times smp_cpus -- matching hw/arm/virt.c. Skipping a stride (as
         * this machine did for VIRQ/VFIQ) puts them on the NMI/VINMI lines
         * instead and leaves the real ones unconnected.
         */
        sysbus_connect_irq(gicbusdev, i,
                           qdev_get_gpio_in(cpudev, ARM_CPU_IRQ));
        sysbus_connect_irq(gicbusdev, i + smp_cpus,
                           qdev_get_gpio_in(cpudev, ARM_CPU_FIQ));
        sysbus_connect_irq(gicbusdev, i + 2 * smp_cpus,
                           qdev_get_gpio_in(cpudev, ARM_CPU_VIRQ));
        sysbus_connect_irq(gicbusdev, i + 3 * smp_cpus,
                           qdev_get_gpio_in(cpudev, ARM_CPU_VFIQ));
        sysbus_connect_irq(gicbusdev, i + 4 * smp_cpus,
                           qdev_get_gpio_in(cpudev, ARM_CPU_NMI));
        sysbus_connect_irq(gicbusdev, i + 5 * smp_cpus,
                           qdev_get_gpio_in(cpudev, ARM_CPU_VINMI));
    }
}

static void mt6895_kstall_dump(void *opaque);   /* diagnostic block */

static void mt6895_create_gic(Mt6895MachineState *s, Error **errp)
{
    MachineState *ms = MACHINE(s);
    QList *redist_region_count = qlist_new();

    s->gic = qdev_new(gicv3_class_name());
    qdev_prop_set_uint32(s->gic, "revision", 3);
    qdev_prop_set_uint32(s->gic, "num-cpu", ms->smp.cpus);
    qdev_prop_set_uint32(s->gic, "num-irq", MT6895_NUM_SPIS + 32);
    qdev_prop_set_bit(s->gic, "has-security-extensions", true);

    qlist_append_int(redist_region_count, ms->smp.cpus);
    qdev_prop_set_array(s->gic, "redist-region-count", redist_region_count);

    /*
     * PCIe MSI needs the GICv3 ITS, and the ITS refuses to realize unless
     * every redistributor advertises physical LPI support (GICR_TYPER.PLPIS).
     * That bit is driven by has-lpi, and the LPI pending tables live in
     * guest RAM, hence the sysmem link.
     */
    object_property_set_link(OBJECT(s->gic), "sysmem",
                             OBJECT(get_system_memory()), &error_fatal);
    qdev_prop_set_bit(s->gic, "has-lpi", true);

    if (!sysbus_realize(SYS_BUS_DEVICE(s->gic), errp)) {
        return;
    }

    sysbus_mmio_map(SYS_BUS_DEVICE(s->gic), 0, MT6895_GICD_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->gic), 1, MT6895_GICR_BASE);
    s->gic_group_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                      mt6895_gic_group_timer, s);

    mt6895_connect_gic_cpus(s);
    mt6895_gic_bootstrap(s);
    if (s->gic_group_timer) {
        timer_mod(s->gic_group_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 2000000);
    }
    mt6895_kstall_dump(s);      /* diagnostic: no-op unless MTK_KSTALL_DUMP */
}

/*
 * ---- -dev block: kernel-stall diagnostics -------------------------
 * MTK_KSTALL_DUMP=<ms>: every <ms> of host (REALTIME) time print CPU0's PC,
 * PSTATE, the architected timer registers and the GICv3 redistributor /
 * CPU-interface state. Uses the realtime clock so it keeps reporting when
 * the guest has stopped receiving its own timer interrupts. Debug only.
 */
static void mt6895_kstall_dump(void *opaque)
{
    Mt6895MachineState *s = opaque;
    static QEMUTimer *t;
    static int64_t period_ms;
    CPUState *cs = qemu_get_cpu(0);
    ARMCPU *cpu = ARM_CPU(cs);
    CPUARMState *env = &cpu->env;
    GICv3State *gs = ARM_GICV3_COMMON(s->gic);
    GICv3CPUState *c = &gs->cpu[0];
    int64_t vns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!t) {
        const char *e = getenv("MTK_KSTALL_DUMP");

        period_ms = e ? strtoll(e, NULL, 0) : 0;
        if (period_ms <= 0) {
            return;
        }
        t = timer_new_ms(QEMU_CLOCK_REALTIME, mt6895_kstall_dump, s);
        timer_mod(t, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + period_ms);
        return;
    }
    fprintf(stderr, "MTK-KSTALL vns=%" PRId64 " cnt=%#" PRIx64
            " pc=%#" PRIx64 " pstate=%#x daif=%#" PRIx64
            " ireq=%#x halted=%u\n",
            vns, (uint64_t)(vns / gt_cntfrq_period_ns(cpu)), env->pc,
            (unsigned)pstate_read(env), (uint64_t)env->daif,
            (unsigned)cs->interrupt_request, (unsigned)cs->halted);
    fprintf(stderr, "MTK-KSTALL  vctl=%#" PRIx64 " vcval=%#" PRIx64
            " voff=%#" PRIx64 " vexp=%" PRId64 " pctl=%#" PRIx64
            " pcval=%#" PRIx64 " pexp=%" PRId64 "\n",
            env->cp15.c14_timer[GTIMER_VIRT].ctl,
            env->cp15.c14_timer[GTIMER_VIRT].cval,
            env->cp15.cntvoff_el2,
            timer_expire_time_ns(cpu->gt_timer[GTIMER_VIRT]),
            env->cp15.c14_timer[GTIMER_PHYS].ctl,
            env->cp15.c14_timer[GTIMER_PHYS].cval,
            timer_expire_time_ns(cpu->gt_timer[GTIMER_PHYS]));
    fprintf(stderr, "MTK-KSTALL  gicr pend=%#x en=%#x act=%#x lvl=%#x"
            " grp=%#x mod=%#x waker=%#x hppi=%d/%#x/%d pmr=%#" PRIx64
            " apr0=%#" PRIx64 " apr1s=%#" PRIx64 " apr1ns=%#" PRIx64
            " igrpen1ns=%#" PRIx64 " gicd_ctlr=%#x\n",
            c->gicr_ipendr0, c->gicr_ienabler0, c->gicr_iactiver0, c->level,
            c->gicr_igroupr0, c->gicr_igrpmodr0, c->gicr_waker,
            (int)c->hppi.irq, (unsigned)c->hppi.prio, (int)c->hppi.grp,
            c->icc_pmr_el1, c->icc_apr[GICV3_G0][0], c->icc_apr[GICV3_G1][0],
            c->icc_apr[GICV3_G1NS][0], c->icc_igrpen[GICV3_G1NS],
            gs->gicd_ctlr);
    timer_mod(t, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + period_ms);
}
/* ---- end -dev block ---------------------------------------------- */


/*
 * ===== TRNG @0x1020f000 — rpmb-dev block =====
 * MediaTek TRNG (same layout as Linux drivers/char/hw_random/mtk-rng.c and
 * the BL31 plat_get_rnd path disassembled from tee_a.img): CTRL+0x00 bit0
 * START, bit31 READY (RO); TIME+0x04; DATA+0x08 (32 random bits per read);
 * CONF+0x0c. BL31 sets CONF|=0x20, CTRL|=1, polls READY and reads DATA four
 * times for MTK_SIP_*_GET_RND (LK's ufs_rpmb_pre_nonce nonce source).
 * Was an unimplemented region (READY never set -> "get_rnd is not ready").
 * MTK_NO_TRNG=1 keeps the old unimplemented behaviour.
 */
#include "qemu/guest-random.h"

static uint32_t mt6895_trng_regs[4];
static void mt6895_infra_cg_init(void);

static uint64_t mt6895_trng_read(void *opaque, hwaddr off, unsigned size)
{
    uint32_t v = 0;

    switch (off) {
    case 0x00:
        v = mt6895_trng_regs[0] & 0x7fffffffu;
        if (v & 1) {
            v |= 0x80000000u;               /* READY while started */
        }
        break;
    case 0x04:
    case 0x0c:
        v = mt6895_trng_regs[off >> 2];
        break;
    case 0x08:
        if (mt6895_trng_regs[0] & 1) {
            qemu_guest_getrandom_nofail(&v, sizeof(v));
        }
        break;
    }
    return v;
}

static void mt6895_trng_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    switch (off) {
    case 0x00:
        mt6895_trng_regs[0] = (uint32_t)val & 0x7fffffffu;
        break;
    case 0x04:
    case 0x0c:
        mt6895_trng_regs[off >> 2] = (uint32_t)val;
        break;
    }
}

static const MemoryRegionOps mt6895_trng_ops = {
    .read = mt6895_trng_read,
    .write = mt6895_trng_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void mt6895_trng_init(void)
{
    static MemoryRegion mr;

    if (getenv("MTK_NO_TRNG")) {
        return;
    }
    memory_region_init_io(&mr, NULL, &mt6895_trng_ops, NULL, "mt6895.trng",
                          0x1000);
    memory_region_add_subregion_overlap(get_system_memory(), 0x1020f000ULL,
                                        &mr, 1000);
    mt6895_infra_cg_init();
}

/*
 * infracfg_ao MODULE_SW_CG_0/1 SET/CLR/STA (0x10001080..0x10001097), same
 * bank layout as Linux clk-mt6895 infra_ao cg_regs_0/1: SET0 0x80, CLR0 0x84,
 * SET1 0x88, CLR1 0x8c, STA0 0x90, STA1 0x94 (STA bit=1 -> clock gated).
 * BL31 plat_trng_prng (tee_a.img +0x16a40) ungates the TRNG clock by writing
 * CG1_CLR=0x200 and polling STA1&0x200==0, then re-gates it with
 * CG1_SET=0x200 and polls STA1&0x200==0x200; a RAM-backed STA never latched
 * the SET, so BL31 logged "plat_trng_prng: gate fail" and GET_RND failed.
 * Reset value 0 (all ungated) keeps the previous observable state.
 * MTK_NO_INFRA_CG=1 (or MTK_NO_TRNG=1) disables this overlay.
 */
static uint32_t mt6895_infra_cg_sta[2];

static uint64_t mt6895_infra_cg_read(void *opaque, hwaddr off, unsigned size)
{
    switch (off) {
    case 0x00: case 0x04: case 0x10:
        return mt6895_infra_cg_sta[0];
    case 0x08: case 0x0c: case 0x14:
        return mt6895_infra_cg_sta[1];
    }
    return 0;
}

static void mt6895_infra_cg_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    switch (off) {
    case 0x00: mt6895_infra_cg_sta[0] |= (uint32_t)val; break;
    case 0x04: mt6895_infra_cg_sta[0] &= ~(uint32_t)val; break;
    case 0x08: mt6895_infra_cg_sta[1] |= (uint32_t)val; break;
    case 0x0c: mt6895_infra_cg_sta[1] &= ~(uint32_t)val; break;
    default: break;                         /* STA is read-only */
    }
}

static const MemoryRegionOps mt6895_infra_cg_ops = {
    .read = mt6895_infra_cg_read,
    .write = mt6895_infra_cg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void mt6895_infra_cg_init(void)
{
    static MemoryRegion mr;

    if (getenv("MTK_NO_INFRA_CG")) {
        return;
    }
    memory_region_init_io(&mr, NULL, &mt6895_infra_cg_ops, NULL,
                          "mt6895.infra-cg01", 0x18);
    memory_region_add_subregion_overlap(get_system_memory(), 0x10001080ULL,
                                        &mr, 1000);
}
/* ===== ADCC (CPU DCDC calibration) ===== */
/*
 * The preloader's adcc_pre_init() is LUA_1 VA 0x0203F608; the failing print is
 * at 0x0203F84E ("adcc_pre_init():868"), reached from the calibration poll
 * loop at 0x0203F820..0x0203F84C:
 *
 *   for (cpu = 0; cpu < 8; cpu++) {
 *       timeout = 2000002;                       // literal 0x001E8482
 *       while (--timeout)
 *           if (readl(0x00115000 + cpu * 0x10) & 0x10000)
 *               goto calib_done;
 *       printf("ADCC CPU%d Auto-calibation fail.", cpu);
 *   }
 *
 * i.e. the block is eight per-CPU register sets 0x10 bytes apart at
 * 0x00115000, and bit 16 of each set's word 0 is the "calibration complete"
 * status the firmware waits for (the dump at 0x0203F510 prints the same bit
 * as SW_nFlag).  The unit's log has no failure line and completes the dump
 * immediately (real_device_boot.txt:2187 "#T#adcc_pre_init=1"), so hardware
 * sets the bit before the first poll.
 *
 * SRAM0 is modelled as 4 MiB of plain RAM starting at 0x00100000, which covers
 * 0x00115000: the firmware's configuration writes landed in RAM, the status
 * bit read back 0 and the poll timed out, so cpu_type stayed 0 and GZ panicked
 * at mt_smcall.c:626 (cpu_type != 0).  Model the register file here instead
 * and report the calibration as already complete.
 *
 * MTK_NO_ADCC=1 disables the overlay and restores the plain-RAM behaviour.
 */
#define MT6895_ADCC_BASE 0x00115000ULL
#define MT6895_ADCC_SIZE 0x00000080ULL      /* 8 CPUs x 0x10 */

static uint32_t mt6895_adcc_regs[MT6895_ADCC_SIZE / 4];

static uint64_t mt6895_adcc_read(void *opaque, hwaddr off, unsigned size)
{
    uint32_t v;

    if (off + size > MT6895_ADCC_SIZE) {
        return 0;
    }
    v = mt6895_adcc_regs[off >> 2];
    if ((off & 0xf) == 0) {
        /* word 0 bit 16: calibration complete (see the block comment) */
        v |= 0x00010000u;
    }
    return extract64(v, (off & 3) * 8, size * 8);
}

static void mt6895_adcc_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    unsigned shift, bits;
    uint32_t mask;

    if (off + size > MT6895_ADCC_SIZE) {
        return;
    }
    shift = (off & 3) * 8;
    bits = size * 8;
    mask = (bits >= 32) ? 0xffffffffu : ((1u << bits) - 1);
    mt6895_adcc_regs[off >> 2] =
        (mt6895_adcc_regs[off >> 2] & ~(mask << shift)) |
        (((uint32_t)val & mask) << shift);
}

static const MemoryRegionOps mt6895_adcc_ops = {
    .read = mt6895_adcc_read,
    .write = mt6895_adcc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mt6895_adcc_init(void)
{
    static MemoryRegion mr;

    if (getenv("MTK_NO_ADCC")) {
        return;
    }
    memory_region_init_io(&mr, NULL, &mt6895_adcc_ops, NULL, "mt6895.adcc",
                          MT6895_ADCC_SIZE);
    memory_region_add_subregion_overlap(get_system_memory(), MT6895_ADCC_BASE,
                                        &mr, 1000);
}
/* ===== end ADCC ===== */


/* ===== GZ hw code (low halfword at 0x08000000) ===== */
/*
 * The preloader's GZ platform-config builder (LUA_1 VA 0x020323BA..0x020323C8)
 * fills the GZ atag "hw code" / "hw sub code" fields from the low halfwords of
 * the words at 0x08000000 and 0x08000004:
 *
 *   0x020323BA  mov.w r0, #0x8000000
 *   0x020323BE  ldr   r0, [r0]
 *   0x020323C0  strh.w r0, [r4, #0x70]     ; GZ atag hw code
 *   0x020323C4  ldr   r0, =0x08000004
 *   0x020323C6  ldr   r0, [r0]
 *   0x020323C8  strh.w r0, [r4, #0x72]     ; GZ atag hw sub code
 *
 * GZ copies that atag verbatim (VA 0x7F804A04, 0x100 bytes) and uses the hw
 * code halfword as cpu_type: the getter at VA 0x7F8047AC returns it and
 * mtee_sapu_init (VA 0x7F81B630) prints "mtee_sapu_init cpu_type=0x%x" and
 * stores it.  On the unit cpu_type is 0x2, i.e. the low halfword at 0x08000000
 * reads 2.  Our 0x08000000 is an unimplemented device returning 0, so cpu_type
 * stayed 0 and GZ asserted at mt_smcall.c:626 (cpu_type != 0).
 *
 * Model the documented device state: low halfword 2.  Only that halfword is
 * documented; the upper halfword and 0x08000004 (hw sub code) are left 0.
 * MTK_NO_HWCODE=1 restores the old all-zero behaviour.
 */
static uint64_t mt6895_hwcode_read(void *opaque, hwaddr off, unsigned size)
{
    uint32_t v = 0;

    if (off < 4) {
        v = 0x00000002u;
    }
    return extract64(v, (off & 3) * 8, size * 8);
}

static void mt6895_hwcode_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    /* Read-only device-state register. */
}

static const MemoryRegionOps mt6895_hwcode_ops = {
    .read = mt6895_hwcode_read,
    .write = mt6895_hwcode_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mt6895_hwcode_init(void)
{
    static MemoryRegion mr;

    if (getenv("MTK_NO_HWCODE")) {
        return;
    }
    memory_region_init_io(&mr, NULL, &mt6895_hwcode_ops, NULL, "mt6895.hwcode",
                          4);
    memory_region_add_subregion_overlap(get_system_memory(), 0x08000000ULL,
                                        &mr, 1000);
}
/* ===== end GZ hw code ===== */


static void mt6895_create_msdc(Mt6895MachineState *s, Error **errp)
{
    DriveInfo *dinfo;
    BusState *sdbus;
    DeviceState *card;

    s->msdc = qdev_new("mtk-msdc");
    if (!sysbus_realize(SYS_BUS_DEVICE(s->msdc), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(s->msdc), 0, MT6895_MSDC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->msdc), 0,
                       qdev_get_gpio_in(s->gic, MT6895_MSDC_SPI));

    /* The MMC PHY block is referenced by the controller's device tree node. */
    {
        DeviceState *phy = qdev_new(TYPE_MTK_SYSCON);

        qdev_prop_set_uint32(phy, "size", MT6895_MSDC_PHY_SIZE);
        if (!sysbus_realize(SYS_BUS_DEVICE(phy), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(phy), 0, MT6895_MSDC_PHY_BASE);
    }

    /*
     * tinysys mailbox -- the SCMI transport. Mapped so the driver can reach
     * it at all; the protocol side is not modelled yet, so SCMI still times
     * out, but the trace now shows what the driver expects.
     */
    s->tinysys_mbox = qdev_new(TYPE_MTK_TINYSYS_MBOX);
    qdev_prop_set_uint32(s->tinysys_mbox, "size", MT6895_TINYSYS_MBOX_SIZE);
    qdev_prop_set_uint32(s->tinysys_mbox, "rx-size", MT6895_TINYSYS_MBOX_SIZE);
    qdev_prop_set_uint64(s->tinysys_mbox, "shmem-base",
                         MT6895_SCMI_TX_SHMEM_BASE);
    qdev_prop_set_uint64(s->tinysys_mbox, "shmem-rx-base",
                         MT6895_SCMI_RX_SHMEM_BASE);
    if (!sysbus_realize(SYS_BUS_DEVICE(s->tinysys_mbox), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(s->tinysys_mbox), 0, MT6895_TINYSYS_MBOX_BASE0);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->tinysys_mbox), 1, MT6895_TINYSYS_MBOX_BASE1);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->tinysys_mbox), 0,
                       qdev_get_gpio_in(s->gic, MT6895_TINYSYS_MBOX_SPI0));
    sysbus_connect_irq(SYS_BUS_DEVICE(s->tinysys_mbox), 1,
                       qdev_get_gpio_in(s->gic, MT6895_TINYSYS_MBOX_SPI1));

    /*
     * UFS host controller. The driver probes CAP/VER/AHIT and the vendor
     * reference-clock register before it will touch the controller at all.
     */
    /*
     * Two UFS models are available:
     *   - mtk_ufshci.c  : the hand-written UFSHCI that grew around this bring-up
     *   - QEMU's own UFS core (hw/ufs), ported to a platform device
     * The ported core follows the UFSHCI/UFS specs much more closely, so it is
     * the reference model; MTK_LEGACY_UFS keeps the old one selectable.
     */
    /*
     * Default to the spec-following reference model (QEMU's hw/ufs ported to a
     * platform device); MTK_LEGACY_UFS=1 selects the hand-written UFSHCI that
     * grew around this bring-up for comparison.
     */
    mt6895_trng_init();   /* rpmb-dev: TRNG block */
    if (!MTK_ENV_CACHED("MTK_LEGACY_UFS")) {
        UfsHc *u;
        int lun;

        s->ufshci = qdev_new("mtk-ufs-platform");
        if (!sysbus_realize(SYS_BUS_DEVICE(s->ufshci), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(s->ufshci), 0, MT6895_UFSHCI_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(s->ufshci), 0,
                           qdev_get_gpio_in(s->gic, MT6895_UFSHCI_SPI));
        u = (UfsHc *)s->ufshci;
        /*
         * MTK_UFS_BROM_BRINGUP: model the BootROM's storage bring-up.
         *
         * On hardware the BootROM initialises the UFS controller before it
         * loads the preloader, and leaves it running: HCE is set, the UIC
         * command interface reports ready.  This machine used to skip that
         * and instead plant a synthetic block device for the payload, which
         * is why the payload's own UFS driver never ran.  Enabling HCE here
         * reproduces what the BootROM leaves behind, without touching any
         * firmware-visible structure.
         */
        if (MTK_ENV_CACHED("MTK_UFS_BROM_BRINGUP")) {
            uint32_t hce = 1;

            address_space_stl_le(&address_space_memory,
                                 MT6895_UFSHCI_BASE + 0x34, hce,
                                 MEMTXATTRS_UNSPECIFIED, NULL);
            fprintf(stderr, "mt6895: UFS brought up before payload (HCE=1)\n");
        }
        /*
         * MediaTek's UFS layout puts the two boot LUs first and user data
         * last:
         *
         *   LU0 = "preloader1"  boot LUN A (preloader image)
         *   LU1 = "preloader2"  boot LUN B (same preloader image)
         *   LU2 = "OT_LUN_EN"   user data (carries the GPT)
         *
         * That ordering is visible in LK: its block-device registration hands
         * 1/2/3 to the three LUs and the read path converts them back with
         * "lun - 1" (0x50f34898), so "preloader1" ends up as LU0.  LK's
         * sec_func_init() then scans LU0 from offset 0x1000 for the
         * "AND_ROMINFO_v" marker of the preloader image, so LU0 has to hold
         * the preloader rather than user data.
         */
        for (lun = 0; lun < 3; lun++) {
            const char *path = (lun < 2) ? s->preloader_path : s->data_path;
            BlockBackend *blk;
            DeviceState *lu;
            Error *lerr = NULL;
            /*
             * The two boot LUNs are read-only on the real part, but user data
             * is writable and the secure-config partition lives there: the
             * bootloader rewrites its header to persist the lock state.  Model
             * that difference instead of write-protecting every LUN.
             */
            uint64_t bdrv_flags = 0;

            if (lun >= 2 && !MTK_ENV_CACHED("MTK_LU2_RO")) {
                bdrv_flags = BDRV_O_RDWR | BDRV_O_NO_FLUSH;
            }
            /*
             * MTK_LU2_SNAP=1: writable user-data LUN whose writes go to a
             * throw-away temporary overlay (QEMU -snapshot semantics), so LK
             * can persist seccfg without touching the shared lu2.img.
             */
            if (lun >= 2 && MTK_ENV_CACHED("MTK_LU2_SNAP")) {
                bdrv_flags = BDRV_O_RDWR | BDRV_O_NO_FLUSH | BDRV_O_SNAPSHOT;
            }
            /*
             * MTK_FLASH_RW=1: let the flashing flows (boot/vendor_boot/dtbo/
             * vbmeta writes) write LUN0.  QEMU -snapshot semantics keep the
             * writes in a temporary overlay, so the shared image file is never
             * modified.  LUN 1/2 behaviour is unchanged.
             */
            if (lun == 0 && MTK_ENV_CACHED("MTK_FLASH_RW")) {
                bdrv_flags = BDRV_O_RDWR | BDRV_O_NO_FLUSH | BDRV_O_SNAPSHOT;
                fprintf(stderr, "mt6895: MTK_FLASH_RW: LUN0 writable (snapshot)\n");
            }

            if (!path) {
                continue;
            }
            blk = blk_new_open(path, NULL, NULL, bdrv_flags, &lerr);
            if (!blk) {
                error_reportf_err(lerr, "mt6895: UFS LUN %d: ", lun);
                return;
            }
            /*
             * This BlockBackend only keeps the node alive: the LU and its
             * scsi-hd attach through the node name and get BlockBackends of
             * their own, which do not share WRITE.  Holding WRITE here made a
             * writable LUN fail with a permission conflict, which aborted the
             * rest of the SoC setup (the old "RW hangs in PMIF/ULPOSC").
             */
            if ((bdrv_flags & BDRV_O_RDWR) &&
                blk_set_perm(blk, 0, BLK_PERM_ALL, &lerr) < 0) {
                error_reportf_err(lerr, "mt6895: UFS LUN %d: ", lun);
                return;
            }
            lu = qdev_new("mtk-ufs-lu");
            qdev_prop_set_uint8(lu, "lun", lun);
            if (!qdev_prop_set_drive_err(lu, "drive", blk, &lerr)) {
                error_reportf_err(lerr, "mt6895: UFS LUN %d: ", lun);
                return;
            }
            if (!qdev_realize(lu, BUS(&u->bus), &lerr)) {
                error_reportf_err(lerr, "mt6895: UFS LUN %d: ", lun);
                return;
            }
        }
        goto ufs_done;
    }
    s->ufshci = qdev_new(TYPE_MTK_UFSHCI);
    qdev_prop_set_uint32(s->ufshci, "size", MT6895_UFSHCI_SIZE);
    if (s->data_path) {
        qdev_prop_set_string(s->ufshci, "data-path", s->data_path);
        /*
         * Real UFS parts keep the preloader in boot LUN A (LUN 1); the BootROM
         * reads it from there. Give the device that backing store so the whole
         * chain sees one storage device, like hardware.
         */
        if (s->preloader_path) {
            qdev_prop_set_string(s->ufshci, "boot-lun", s->preloader_path);
        }
    }
    /*
     * NOTE: do NOT feed the machine's GPT blob to the UFS device.
     *
     * UFS logical blocks are 4096 bytes, so LK reads the GPT header at
     * 4 KiB-LBA 1 (file offset 0x1000) and the entries at LBA 2 (0x2000) --
     * which is exactly where device_real.img already carries them. Its LBAs
     * are in 4 KiB units too (lk_a = 227584 -> 227584*4096 = 0x37900000,
     * the lk container). Overlaying xaga_gpt_full.bin (a 512-byte-unit table)
     * shifted every partition, which is what made boot_ctrl read as zero.
     */
    if (!sysbus_realize(SYS_BUS_DEVICE(s->ufshci), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(s->ufshci), 0, MT6895_UFSHCI_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->ufshci), 0,
                       qdev_get_gpio_in(s->gic, MT6895_UFSHCI_SPI));
ufs_done:

    /*
     * SCPSYS power-domain controller. The domains are gated here and the
     * driver polls this block for the power-on status; UFS needs its
     * ufs0_shutdown domain from it.
     */
    s->scpsys = qdev_new(TYPE_MTK_SCPSYS);
    qdev_prop_set_uint32(s->scpsys, "size", MT6895_SCPSYS_SIZE);
    if (!sysbus_realize(SYS_BUS_DEVICE(s->scpsys), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(s->scpsys), 0, MT6895_SCPSYS_BASE);

    /*
     * SPMI PMIC arbiter. Both the PMIF and the SPMI master block belong to it;
     * with the channel handshake modelled, the PMIC stack comes up and the
     * drivers that hang off a PMIC regulator can probe.
     */
    s->spmi = qdev_new(TYPE_MTK_SPMI_PMIF);
    /*
     * The board's PMIC population, as wired in the xaga device tree /
     * preloader SPMI check table: MT6363 main PMIC at slave 4, MT6368 second
     * PMIC at slave 5, the two MT6319 buck dies at slaves 6 and 7 and the
     * MT6685 RTC/auxadc PMIC at slave 9.  The arbiter itself carries no part
     * knowledge; it gets it from this machine description.
     */
    qdev_prop_set_string(s->spmi, "slave-map",
                         "4=mt6363,5=mt6368,6=mt6319,7=mt6319,9=mt6685");
    if (!sysbus_realize(SYS_BUS_DEVICE(s->spmi), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(s->spmi), 0, MT6895_SPMI_PMIF_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->spmi), 1, MT6895_SPMI_MST_BASE);

    s->spmi2 = qdev_new(TYPE_MTK_SPMI_PMIF);
    /*
     * Alternate SPMI master of the same bus (spmi@1c804000 has two master
     * windows); the board population is identical.
     */
    qdev_prop_set_string(s->spmi2, "slave-map",
                         "4=mt6363,5=mt6368,6=mt6319,7=mt6319,9=mt6685");
    if (!sysbus_realize(SYS_BUS_DEVICE(s->spmi2), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(s->spmi2), 0, MT6895_SPMI2_PMIF_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->spmi2), 1, MT6895_SPMI2_MST_BASE);

    /*
     * GPIO/EINT controller (apirq@11e50000, "mediatek,mt6983-eint").
     *
     * The five instances (w/e/s/n/c) are separate register banks but share
     * one GIC line, SPI 259. The device tree routes the SPMI PMIC RCS
     * interrupt to pin 243, which maps to instance 4 (eint_c) index 16; that
     * is the line the PMIF's RCS output drives, so the MT6363 power key
     * interrupt is delivered through the same hardware path as on the phone.
     */
    {
        static const uint64_t eint_bases[] = {
            0x11e50000ULL,      /* eint-w */
            0x11ce0000ULL,      /* eint-e */
            0x11de0000ULL,      /* eint-s */
            0x11fe0000ULL,      /* eint-n */
            0x1c01e000ULL,      /* eint-c */
        };
        DeviceState *eint = qdev_new("mtk-eint");
        /* Board wiring: the touch IRQ line maps to this EINT pin. */
        /* Touch INT: EINT instance 1, index 31 (discovered from the guest own request_irq programming). */
        qdev_prop_set_uint32(eint, "touch-instance", 1);
        qdev_prop_set_uint32(eint, "touch-index", 31);

        mt6895_eint_dev = eint;
        size_t ei;

        if (!sysbus_realize(SYS_BUS_DEVICE(eint), errp)) {
            return;
        }
        for (ei = 0; ei < ARRAY_SIZE(eint_bases); ei++) {
            sysbus_mmio_map(SYS_BUS_DEVICE(eint), ei, eint_bases[ei]);
        }
        sysbus_connect_irq(SYS_BUS_DEVICE(eint), 0,
                           qdev_get_gpio_in(s->gic, 259));
        sysbus_connect_irq(SYS_BUS_DEVICE(s->spmi), 0,
                           qdev_get_gpio_in(eint, 0));
        /*
         * Touch stack: SPI controllers + Novatek NT36672C client.  Opt-in via
         * MTK_TOUCH=1; default OFF because binding the NVT driver currently
         * exposes a pre-existing display/CMDQ model defect (GCE-D "aee not
         * ready" -> cmdq_task_handle_error -> smi_dbg_hang_detect -> emidbg
         * flood, no recovery).  See artifacts/r1302/phase2/touch_landing/
         * DISPLAY_CMDQ_DEFECT.md.  Unset means the machine creates no touch
         * devices and behaves exactly like the committed base.
         */
        if (mtk_env_on("MTK_TOUCH")) {
            mt6895_create_spi(s, errp);
        }
    }

    /*
     * Attach the card the user supplied with -drive if=sd. The controller's
     * device tree node is patched to non-removable, so no card-detect GPIO
     * needs modelling.
     */
    dinfo = drive_get(IF_SD, 0, 0);
    if (!dinfo) {
        return;
    }

    sdbus = qdev_get_child_bus(s->msdc, "sd-bus");
    card = qdev_new(TYPE_SD_CARD);
    qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(dinfo),
                            &error_fatal);
    qdev_realize_and_unref(card, sdbus, &error_fatal);
}


/*
 * EXPERIMENT (2026-11-03): diagnostic MTK I2C controller window.
 *
 * The battery/FG loop reads the sub-PMIC (mt6375) over the I2C controller at
 * 0x11F40000, which was a zero-filled syscon window: the driver polls a status
 * register that never completes and every transfer returns -121.  This first
 * version only RECORDS the access pattern (offset/size/value/pc/lr) and stores
 * written values, so the real register offsets can be read off a single run.
 * Replace the read path with a proper state machine afterwards.
 */
#define MT6895_I2C_DIAG_MAX 600
static uint32_t mt6895_i2c_regs[0x1000 / 4];
static int mt6895_i2c_diag_reads;
static int mt6895_i2c_diag_writes;

/*
 * EXPERIMENT (2026-11-03): transaction tracking for a fake mt6375 register
 * file.  START (0x24 bit0) snapshots the bytes queued on DATA_PORT into
 * tx_xfer; a DATA_PORT read returns the fake value for (slave, tx[0], index).
 */
static uint8_t mt6895_i2c_tx_work[16];
static int mt6895_i2c_tx_work_n;
static uint8_t mt6895_i2c_tx_xfer[16];
static int mt6895_i2c_tx_xfer_n;
static uint8_t mt6895_i2c_slave_xfer;
static int mt6895_i2c_rx_idx;
static int mt6895_i2c_xfer_diag;

/* EXPERIMENT: fake mt6375 register file (slave, register, bytes). */
static const struct {
    uint8_t slave;
    uint8_t reg;
    uint8_t val[4];
} mt6895_i2c_fake[] = {
    /*
     * MT6375 identity: regmap 0x100 = bank 1 offset 0x00, 1 byte.  Verified
     * driver semantics (notes_peri_spec.md §8.2): (value & 0xF0) must be 0x70
     * (MT6375_VID) and bits[3:0] are the chip revision, accepted as-is.  The
     * real unit's preloader prints "mt6375_check_devid rev=0x2", so the byte is
     * 0x72.  Bank 1's I2C address is 0x34 (7-bit) = 0x68 on the wire, from the
     * payload bank table at preloader 0x206d048 ("4e 34 1a 4a 64 3f").
     */
    { 0x68, 0x00, { 0x72 } },
    { 0x34, 0x58, { 0x12, 0x34 } },   /* gauge efuse word (first guess) */
    { 0x34, 0xae, { 0x01 } },
    { 0x7e, 0x03, { 0x00 } },
};

/*
 * mt6375 BC1.2 charger-type detection.
 *
 * The preloader's mtk_ext_chgdet (0x0203c344) toggles BC12_EN = reg 0x150
 * bit7 (0 then 1), waits 100 ms and polls BC12_STAT = reg 0x151 [3:0]
 * (PORT_STAT) every 25 ms for up to 2.5 s.  Register 0x1xx lives behind
 * slave 0x34 (bank table at preloader 0x206d048: 4e 34 1a 4a 64 3f), sub-
 * address = reg & 0xff.  Field layout and PORT_STAT encoding follow Linux
 * mt6375-charger / mt6370-charger: 0 = no info, 8 APPLE_10W, 9 SAMSUNG_10W,
 * 10 APPLE_5W, 11 APPLE_12W, 12 UNKNOWN_TA, 13 SDP, 14 CDP, 15 DCP.
 *
 * Model: BC12_FUNC is a plain R/W register.  On a 0->1 edge of BC12_EN with
 * VBUS present the detection sequence (DCD + primary + secondary detection,
 * ~130 ms of virtual time) starts; once it has elapsed PORT_STAT reads SDP
 * (the board is attached to a USB host).  Clearing BC12_EN clears PORT_STAT.
 * MTK_MT6375_NO_VBUS=1 models an unplugged cable (PORT_STAT stays 0, as on
 * silicon); MTK_NO_BC12=1 restores the old constant-0 behaviour;
 * MTK_BC12_LOG=1 traces to stderr.
 */
/*
 * The controller's SLAVE_ADDR register holds the 8-bit address (7-bit << 1):
 * MTK_I2CDIAG_LOG=1 shows the 0x151 poll as slave=0x68 tx=51 (the entries
 * of mt6895_i2c_fake[] tagged 0x34 are therefore 7-bit 0x1a = bank 2).
 */
#define MT6375_BC12_SLAVE       (0x34 << 1)
#define MT6375_BC12_FUNC        0x50    /* 0x150 */
#define MT6375_BC12_STAT        0x51    /* 0x151 */
#define MT6375_BC12_EN          0x80
#define MT6375_PORT_STAT_SDP    0x0d
#define MT6375_BC12_DET_MS      130
static uint8_t mt6375_bc12_func;
static int64_t mt6375_bc12_start_ms = -1;

static bool mt6375_bc12_model(void)
{
    return !MTK_ENV_CACHED("MTK_NO_BC12");
}

static uint8_t mt6375_bc12_stat(void)
{
    int64_t now;
    uint8_t v = 0;

    if (!(mt6375_bc12_func & MT6375_BC12_EN) || mt6375_bc12_start_ms < 0) {
        return 0;
    }
    now = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
    if (now - mt6375_bc12_start_ms >= MT6375_BC12_DET_MS) {
        v = MT6375_PORT_STAT_SDP;
    }
    if (MTK_ENV_CACHED("MTK_BC12_LOG")) {
        fprintf(stderr, "MTK-BC12 stat read t=+%" PRId64 "ms -> %#x\n",
                now - mt6375_bc12_start_ms, v);
    }
    return v;
}

static void mt6375_bc12_write_func(uint8_t val)
{
    uint8_t old = mt6375_bc12_func;

    mt6375_bc12_func = val;
    if (!(old & MT6375_BC12_EN) && (val & MT6375_BC12_EN)) {
        /* detection only runs with VBUS present */
        mt6375_bc12_start_ms = MTK_ENV_CACHED("MTK_MT6375_NO_VBUS") ? -1 :
                               qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
    } else if (!(val & MT6375_BC12_EN)) {
        mt6375_bc12_start_ms = -1;
    }
    if (MTK_ENV_CACHED("MTK_BC12_LOG")) {
        fprintf(stderr, "MTK-BC12 func <- %#x (start=%" PRId64 ")\n", val,
                mt6375_bc12_start_ms);
    }
}

static uint64_t mt6895_i2c_read(void *opaque, hwaddr off, unsigned size)
{
    uint64_t v = 0;
    unsigned i;
    ARMCPU *cpu = ARM_CPU(current_cpu);
    uint32_t pc = cpu ? (uint32_t)cpu->env.regs[15] : 0;
    uint32_t lr = cpu ? (uint32_t)cpu->env.regs[14] : 0;

    (void)pc;
    (void)lr;

    /*
     * EXPERIMENT E1 (per static analysis of 0x020345AC): the driver polls
     * INTR_STAT (offset 0x0c) for bit0 = TRANSAC_DONE and treats bit1/bit2 as
     * ACK/HS-NACK errors.  It first writes 7 to clear interrupts; a plain
     * store-and-readback model then reads 7 back and every transfer fails.
     * Report DONE only (0x1) and hand back the DATA_PORT value (0x00) for
     * reads.  Everything else keeps the readback behaviour.
     */
    if (off == 0x0c) {
        v = 0x1;
    } else if (off == 0x00) {
        uint8_t slave = mt6895_i2c_slave_xfer;
        uint8_t reg = mt6895_i2c_tx_xfer_n > 0 ? mt6895_i2c_tx_xfer[0] : 0xff;
        size_t k;

        v = 0x00;                       /* fake register file default */
        if (slave == MT6375_BC12_SLAVE && mt6375_bc12_model() &&
            (reg == MT6375_BC12_FUNC || reg == MT6375_BC12_STAT)) {
            v = (reg == MT6375_BC12_FUNC) ? mt6375_bc12_func
                                          : mt6375_bc12_stat();
            mt6895_i2c_rx_idx++;
            goto out;
        }
        for (k = 0; k < ARRAY_SIZE(mt6895_i2c_fake); k++) {
            if (mt6895_i2c_fake[k].slave == slave &&
                mt6895_i2c_fake[k].reg == reg) {
                v = mt6895_i2c_fake[k].val[mt6895_i2c_rx_idx % 4];
                break;
            }
        }
        if (mt6895_i2c_xfer_diag < 2000) {
            mt6895_i2c_xfer_diag++;
        }
        mt6895_i2c_rx_idx++;
    } else {
        for (i = 0; i < size; i++) {
            if (off + i < sizeof(mt6895_i2c_regs)) {
                v |= (uint64_t)((mt6895_i2c_regs[(off + i) / 4] >>
                                 (((off + i) % 4) * 8)) & 0xff) << (8 * i);
            }
        }
    }
out:
    if (mt6895_i2c_diag_reads < MT6895_I2C_DIAG_MAX) {
        mt6895_i2c_diag_reads++;
    }
    return v;
}

static void mt6895_i2c_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    unsigned i;
    ARMCPU *cpu = ARM_CPU(current_cpu);
    uint32_t pc = cpu ? (uint32_t)cpu->env.regs[15] : 0;
    uint32_t lr = cpu ? (uint32_t)cpu->env.regs[14] : 0;

    (void)pc;
    (void)lr;

    if (off == 0x24 && (val & 1)) {
        /* START: snapshot the queued TX bytes as this transaction's request */
        memcpy(mt6895_i2c_tx_xfer, mt6895_i2c_tx_work, sizeof(mt6895_i2c_tx_xfer));
        mt6895_i2c_tx_xfer_n = mt6895_i2c_tx_work_n;
        mt6895_i2c_tx_work_n = 0;
        mt6895_i2c_slave_xfer = mt6895_i2c_regs[0x94 / 4] & 0xff;
        mt6895_i2c_rx_idx = 0;
        if (MTK_ENV_CACHED("MTK_I2CDIAG_LOG")) {
            fprintf(stderr, "MTK-I2CDIAG start off=%#" HWADDR_PRIx
                    " slave=%#x n=%d tx=%02x %02x\n", off,
                    mt6895_i2c_slave_xfer, mt6895_i2c_tx_xfer_n,
                    mt6895_i2c_tx_xfer[0], mt6895_i2c_tx_xfer[1]);
        }
        /* mt6375 BC12_FUNC register write: [sub-address, value] */
        if (mt6895_i2c_slave_xfer == MT6375_BC12_SLAVE &&
            mt6375_bc12_model() && mt6895_i2c_tx_xfer_n >= 2 &&
            mt6895_i2c_tx_xfer[0] == MT6375_BC12_FUNC) {
            mt6375_bc12_write_func(mt6895_i2c_tx_xfer[1]);
        }
    } else if (off == 0x00) {
        if (mt6895_i2c_tx_work_n < (int)sizeof(mt6895_i2c_tx_work)) {
            mt6895_i2c_tx_work[mt6895_i2c_tx_work_n++] = (uint8_t)val;
        }
    }

    if (off == 0x0c) {
        /* INTR_STAT is write-1-to-clear. */
        mt6895_i2c_regs[0x0c / 4] &= ~(uint32_t)val;
    } else if (off == 0x00) {
        /* DATA_PORT TX byte: no slave model, drop it. */
    } else {
        for (i = 0; i < size; i++) {
            unsigned bo = off + i;
            if (bo < sizeof(mt6895_i2c_regs)) {
                uint32_t *w = &mt6895_i2c_regs[bo / 4];
                unsigned sh = (bo % 4) * 8;
                *w = (*w & ~(0xffu << sh)) |
                     (((uint32_t)(val >> (8 * i)) & 0xff) << sh);
            }
        }
    }
    if (mt6895_i2c_diag_writes < MT6895_I2C_DIAG_MAX) {
        mt6895_i2c_diag_writes++;
    }
}

static const MemoryRegionOps mt6895_i2c_ops = {
    .read = mt6895_i2c_read,
    .write = mt6895_i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static const struct {
    uint64_t base;
    uint32_t size;
} mt6895_i2c_windows[] = {
    { 0x11280000ULL, 0x2000 },   /* imp_iic_wrap_c: controllers 0/1 */
    { 0x11300000ULL, 0x0800 },   /* imp_iic_wrap_c sentinel windows */
    { 0x11d00000ULL, 0x7000 },   /* imp_iic_wrap_s: controllers 0..6 */
    { 0x11f40000ULL, 0x1000 },   /* imp_iic_wrap_w: controller used by PMIC */
};

/*
 * DIAGNOSTIC (MTK_BL2LIST): BL2 keeps a registry list at 0x62fca250 whose
 * entries carry names at +0x18 and +0x40.  sec_func_init looks up
 * "preloader1" there; when that fails the whole security init returns
 * 0x109000.  Dump the list once so the actual contents become visible.
 */
/*
 * DIAGNOSTIC (MTK_CODEGEN): the preloader generates Thumb code into a buffer
 * at 0x020f9e2c at runtime (see the writer at preloader VA 0x204d444).  Dump
 * that buffer so the generated sequence -- and therefore what the entry
 * writer is asked to fill -- becomes readable.
 */
static void mt6895_dump_codegen(Mt6895MachineState *s)
{
    uint8_t *p = mt6895_guest_ptr(s, 0x020f9e00ULL, 0x180);
    int i;

    if (!p) {
        return;
    }
    for (i = 0; i < 0x180; i += 0x10) {
        fprintf(stderr, "CODEGEN +%03x: %08x %08x %08x %08x\n",
                i, ldl_le_p(p + i), ldl_le_p(p + i + 4),
                ldl_le_p(p + i + 8), ldl_le_p(p + i + 12));
    }
}

static void mt6895_dump_bl2_list(Mt6895MachineState *s)
{
    hwaddr head = 0x62fca250;
    uint8_t *hp = mt6895_guest_ptr(s, head, 0x10);
    uint64_t first;
    int i;

    if (!hp) {
        return;
    }
    /*
     * Look for the exact string "preloader1" (NUL terminated) anywhere in the
     * low DRAM window BL2 allocates from, then report every word that points
     * at it.  A node that names itself exactly "preloader1" is what
     * sec_func_init searches for; the list above only carries
     * "preloader1pNN" variants, so knowing whether the bare name exists at all
     * tells us whether the node was never created or merely unlinked.
     */
    {
        static const char want[] = "preloader1";
        hwaddr scan;

        for (scan = 0x60000000ULL; scan < 0x64000000ULL; scan += 0x100000) {
            uint8_t *blk = mt6895_guest_ptr(s, scan, 0x100000);
            uint32_t off2;

            if (!blk) {
                continue;
            }
            for (off2 = 0; off2 + 16 < 0x100000; off2++) {
                if (memcmp(blk + off2, want, sizeof(want)) == 0) {
                    fprintf(stderr, "BL2LIST str 'preloader1' @ PA %#" PRIx64
                            " (next bytes: %02x %02x)\n",
                            (uint64_t)(scan + off2),
                            blk[off2 + 10], blk[off2 + 11]);
                }
            }
        }
    }
    /*
     * Report every word that points at the bare "preloader1" string found at
     * 0x630dfe40 (both plain and with the 0xffff0000_00000000 tag that BL2's
     * user-mode VAs carry).
     */
    {
        hwaddr scan;
        const uint64_t targets[2] = { 0x630dfe40ULL, 0xffff0000630dfe40ULL };

        for (scan = 0x60000000ULL; scan < 0x64000000ULL; scan += 0x100000) {
            uint8_t *blk = mt6895_guest_ptr(s, scan, 0x100000);
            uint32_t off2;
            int t;

            if (!blk) {
                continue;
            }
            for (off2 = 0; off2 + 8 <= 0x100000; off2 += 8) {
                uint64_t w = ldq_le_p(blk + off2);

                for (t = 0; t < 2; t++) {
                    if (w == targets[t]) {
                        fprintf(stderr, "BL2LIST ptr->preloader1 @ PA %#" PRIx64
                                " (value %#" PRIx64 ")\n",
                                (uint64_t)(scan + off2), w);
                    }
                }
            }
        }
    }
    /*
     * Dump the 0x120 bytes around the node that names itself "preloader1"
     * (the pointer to the string sits at 0x630dfd98), to see whether it is a
     * complete registry entry that simply never got linked.
     */
    {
        uint64_t base = 0x630dfd40ULL;
        uint8_t *nb = mt6895_guest_ptr(s, base, 0x140);
        int d;

        if (nb) {
            for (d = 0; d < 0x140; d += 0x10) {
                fprintf(stderr, "BL2NODE +%03x: %016" PRIx64 " %016" PRIx64 "\n",
                        d, ldq_le_p(nb + d), ldq_le_p(nb + d + 8));
            }
        }
    }
    first = ldq_le_p(hp) & 0xffffffffULL;
    {
        uint64_t n2 = first;
        int cnt = 0;

        while (n2 && n2 != head && cnt < 200) {
            uint8_t *nn = mt6895_guest_ptr(s, n2 & 0xffffffffULL, 0x60);

            if (!nn) {
                break;
            }
            n2 = ldq_le_p(nn) & 0xffffffffULL;
            cnt++;
        }
        fprintf(stderr, "BL2LIST nodes=%d first=%#" PRIx64 "\n", cnt, first);
    }
    for (i = 0; i < 80 && first && first != head; i++) {
        uint8_t *n = mt6895_guest_ptr(s, first & 0xffffffffULL, 0x60);
        uint64_t next;
        char n18[40], n40[40];
        int k;

        if (!n) {
            break;
        }
        next = ldq_le_p(n) & 0xffffffffULL;
        memset(n18, 0, sizeof(n18));
        memset(n40, 0, sizeof(n40));
        for (k = 0; k < 2; k++) {
            uint64_t ptr = ldq_le_p(n + (k ? 0x40 : 0x18)) & 0xffffffffULL;
            uint8_t *sp = ptr ? mt6895_guest_ptr(s, ptr, 0x28) : NULL;
            char *out = k ? n40 : n18;
            int j;

            if (!sp) {
                snprintf(out, 40, "<bad %#" PRIx64 ">", ptr);
                continue;
            }
            for (j = 0; j < 32; j++) {
                uint8_t c = sp[j];

                if (c < 0x20 || c >= 0x7f) {
                    break;
                }
                out[j] = (char)c;
            }
        }
        fprintf(stderr, "BL2LIST [%d] node=%#" PRIx64 " next=%#" PRIx64
                " n+18='%s' n+40='%s'\n", i, first, next, n18, n40);
        first = next;
    }
}

/* BEGIN MTK TOUCH WIRING (touch_wiring.py) */
/*
 * SPI / touch stack: eight spi-mt65xx controllers at 0x11010000..0x11017000
 * (GIC SPI 203..210), with the Novatek NT36672C touch controller as the SSI
 * client of spi2/CS0 (DT: novatek@0 on spi2; its IRQ is DT GPIO 400, which
 * the vendor pinctrl driver maps onto an EINT pin described by the
 * touch-instance/touch-index properties of the mtk-eint device).
 */
static void mt6895_create_spi(Mt6895MachineState *s, Error **errp)
{
    static const struct {
        uint64_t addr;
        int spi;
    } ctl[] = {
        { 0x11010000ULL, 203 }, { 0x11011000ULL, 204 },
        { 0x11012000ULL, 205 }, { 0x11013000ULL, 206 },
        { 0x11014000ULL, 207 }, { 0x11015000ULL, 208 },
        { 0x11016000ULL, 209 }, { 0x11017000ULL, 210 },
    };
    const int nvt_spi = 2;
    int i;

    if (!mt6895_eint_dev) {
        return;
    }
    for (i = 0; i < ARRAY_SIZE(ctl); i++) {
        DeviceState *dev = qdev_new(TYPE_MTK_SPI);
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

        qdev_prop_set_uint32(dev, "clock-hz", 13000000);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(s->gic, ctl[i].spi));
        memory_region_add_subregion_overlap(get_system_memory(), ctl[i].addr,
                                            sysbus_mmio_get_region(sbd, 0), 1);
        if (i == nvt_spi) {
            SSIBus *bus = (SSIBus *)qdev_get_child_bus(dev, "spi");
            DeviceState *nvt = qdev_new(TYPE_NVT36672C);

            if (!ssi_realize_and_unref(nvt, bus, errp)) {
                return;
            }
            qdev_connect_gpio_out(nvt, 0,
                                  qdev_get_gpio_in(mt6895_eint_dev, 1));
            sysbus_connect_irq(sbd, 1, qdev_get_gpio_in(nvt, 0));
        }
    }
}
/* END MTK TOUCH WIRING */
static void mt6895_create_i2c(Mt6895MachineState *s, Error **errp)
{
    MemoryRegion *sysmem = get_system_memory();
    size_t i;

    for (i = 0; i < ARRAY_SIZE(mt6895_i2c_windows); i++) {
        memory_region_init_io(&s->i2c_ctrl[i], OBJECT(s), &mt6895_i2c_ops, s,
                              "mt6895.i2c-ctrl", mt6895_i2c_windows[i].size);
        memory_region_add_subregion(sysmem, mt6895_i2c_windows[i].base,
                                    &s->i2c_ctrl[i]);
    }

    /*
     * real MTK I2C controllers for LK / kernel.
     *
     * The diagnostic windows above decode INTR_STAT only at window offset
     * 0x0c, i.e. for the first controller of each wrap.  LK's backlight/bias
     * chip KTZ8863A sits on I2C id=6 = 0x11281000 (window offset 0x100c), so
     * its poll of INTR_STAT never saw TRANSAC_COMP and every write timed out
     * (-13).  Here every controller of imp_iic_wrap_c / imp_iic_wrap_s gets a
     * TYPE_MTK_I2C instance (one I2CBus each) layered over the diagnostic
     * window; transfers run on the QEMU I2C bus, so a missing slave yields
     * ACKERR as on silicon.  KTZ8863A (0x11) is attached to 0x11281000.
     * MTK_NO_I2CMODEL=1 restores the old diagnostic-only behaviour;
     * MTK_I2CTRACE=1 prints every transfer.  The wrap_w controller at
     * 0x11f40000 (preloader's mt6375 path) is left untouched.
     */
    if (!MTK_ENV_CACHED("MTK_NO_I2CMODEL")) {
        /*
         * 0x11280000 (id=5) is NOT modelled here: the preloader's battery
         * path talks to the sub-PMIC mt6375 (0x34/0x4a) on it and the only
         * mt6375 we have is the fake register file inside the diagnostic
         * window.  With a real bus and no mt6375 slave every access ACKERRs
         * and the preloader loops on "bat NOT exist" (OUT_disp3_1.log).
         */
        /*
         * Address, DT bus number and GIC SPI of every modelled controller.
         * The SPI numbers are the ones from the i2c nodes' interrupts
         * property ("interrupts = <0 spi irq-type>"): i2c6@11281000 ->
         * SPI 150, i2c1..i2c4@11d00000..11d03000 -> 145..148 and
         * i2c7..i2c9@11d04000..11d06000 -> 151..153.  QEMU GICv3 input lines
         * take the SPI number unchanged (see include/hw/arm/mt6895.h).
         *
         * The kernel driver (drivers/i2c/busses/i2c-mt65xx.c) waits for
         * msg_complete on that IRQ: with the line unconnected the controller
         * still ACKs the transfer (INTR_STAT bit0 gets set) but the caller
         * times out after 2 s - which made every ktz8863a backlight write
         * stall for ~2 s while holding mtk_crtc->lock.
         */
        static const struct {
            uint64_t addr;
            uint32_t id;
            int spi;
        } ctl[] = {
            { 0x11281000ULL, 6, 150 },
            { 0x11d00000ULL, 1, 145 },
            { 0x11d01000ULL, 2, 146 },
            { 0x11d02000ULL, 3, 147 },
            { 0x11d03000ULL, 4, 148 },
            { 0x11d04000ULL, 7, 151 },
            { 0x11d05000ULL, 8, 152 },
            { 0x11d06000ULL, 9, 153 },
        };

        for (i = 0; i < ARRAY_SIZE(ctl); i++) {
            DeviceState *dev = qdev_new(TYPE_MTK_I2C);
            SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

            qdev_prop_set_uint32(dev, "id", ctl[i].id);
            qdev_prop_set_bit(dev, "trace", !!MTK_ENV_CACHED("MTK_I2CTRACE"));
            sysbus_realize_and_unref(sbd, &error_fatal);
            sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(s->gic, ctl[i].spi));
            memory_region_add_subregion_overlap(sysmem, ctl[i].addr,
                                                sysbus_mmio_get_region(sbd, 0),
                                                1);
            if (ctl[i].addr == 0x11281000ULL) {
                /*
                 * KTZ8863A backlight/bias controller at 7-bit address 0x11.
                 * TYPE_KTZ8863A_I2C is the standalone chip model; the older
                 * inline stub (TYPE_KTZ8863A, hw/i2c/mtk_i2c.c) stays
                 * registered but is not instantiated anywhere, so only one
                 * slave answers on this bus/address.
                 */
                DeviceState *ktz =
                    DEVICE(i2c_slave_new(TYPE_KTZ8863A_I2C, 0x11));

                qdev_prop_set_bit(ktz, "trace",
                                  !!MTK_ENV_CACHED("MTK_I2CTRACE"));
                i2c_slave_realize_and_unref(I2C_SLAVE(ktz), MTK_I2C(dev)->bus,
                                            &error_fatal);
            }
            if (ctl[i].addr == 0x11d04000ULL) {
                DeviceState *fg = DEVICE(i2c_slave_new(TYPE_BQ28Z610, 0x55));

                i2c_slave_realize_and_unref(I2C_SLAVE(fg), MTK_I2C(dev)->bus,
                                            &error_fatal);
            }

        }
    }

    /*
     * display scan-out.  Composes the enabled OVL
     * layers (disp_ovl@14002000/3000/4000, 14402000/3000/4000 per the device
     * tree) into a 1080x2460 graphic console (HMP screendump works with
     * -display none).  MTK_FBDUMP=<path> additionally writes a PPM whenever
     * the picture changes (checked every MTK_FBDUMP_MS ms of host time,
     * default 1000; numbered copies <path>.N.ppm for the first 16 frames).
     * MTK_FB_FALLBACK=<hex PA> scans that buffer as XRGB8888 when no OVL
     * layer is enabled.  MTK_DISPTRACE=1 logs the decoded layers.
     * MTK_NO_DISP=1 disables the whole model.
     */
    if (!MTK_ENV_CACHED("MTK_NO_DISP")) {
        DeviceState *dev = qdev_new(TYPE_MTK_DISP);
        const char *dump = getenv("MTK_FBDUMP");
        const char *ms = getenv("MTK_FBDUMP_MS");
        const char *fb = getenv("MTK_FB_FALLBACK");

        qdev_prop_set_uint64(dev, "ovl0", 0x14002000ULL);
        qdev_prop_set_uint64(dev, "ovl1", 0x14003000ULL);
        qdev_prop_set_uint64(dev, "ovl2", 0x14004000ULL);
        qdev_prop_set_uint64(dev, "ovl3", 0x14402000ULL);
        qdev_prop_set_uint64(dev, "ovl4", 0x14403000ULL);
        qdev_prop_set_uint64(dev, "ovl5", 0x14404000ULL);
        /* disp0 pipe = OVL 0x1400xxxx (left), disp1 = 0x1440xxxx (right) */
        qdev_prop_set_uint32(dev, "pipe-split", 3);
        if (dump && *dump) {
            qdev_prop_set_string(dev, "dump-path", dump);
        }
        if (ms && *ms) {
            qdev_prop_set_uint32(dev, "dump-ms", strtoul(ms, NULL, 0));
        }
        if (fb && *fb) {
            const char *fp = getenv("MTK_SIMPLEFB_PITCH");
            const char *ff = getenv("MTK_SIMPLEFB_FMT");
            const char *fw = getenv("MTK_SIMPLEFB_W");
            const char *fh = getenv("MTK_SIMPLEFB_H");
            const char *fr = getenv("MTK_SIMPLEFB_RIGHT");
            const char *frp = getenv("MTK_SIMPLEFB_RIGHT_PITCH");

            qdev_prop_set_uint64(dev, "fb-fallback", strtoull(fb, NULL, 16));
            if (fp && *fp) {
                qdev_prop_set_uint32(dev, "fb-pitch", strtoul(fp, NULL, 0));
            }
            if (ff && *ff) {
                qdev_prop_set_uint32(dev, "fb-fmt", strtoul(ff, NULL, 0));
            }
            if (fw && *fw) {
                qdev_prop_set_uint32(dev, "width", strtoul(fw, NULL, 0));
            }
            if (fh && *fh) {
                qdev_prop_set_uint32(dev, "height", strtoul(fh, NULL, 0));
            }
            if (fr && *fr) {
                qdev_prop_set_uint64(dev, "fb-right", strtoull(fr, NULL, 16));
            }
            if (frp && *frp) {
                qdev_prop_set_uint32(dev, "fb-right-pitch",
                                     strtoul(frp, NULL, 0));
            }
        }
        qdev_prop_set_bit(dev, "trace", !!MTK_ENV_CACHED("MTK_DISPTRACE"));
        qdev_realize_and_unref(dev, NULL, &error_fatal);
    }
}

/*
 * ===== USB host-mode DT injection (env MTK_USB_HOSTMODE=1) =====
 *
 * The vendor DT declares mtu3 as a dual-role (otg) controller with
 * "usb-role-switch".  Its late probe then registers a usb_role_switch and
 * gets a deferred-probe error, which tears the freshly registered xhci host
 * down again - the role-switch graph peer (extcon_usb) and the typec/tcpc
 * chain behind the mt6375 PMIC only appear much later in init (or not at
 * all, see docs/xaga/architecture.md).  To get a stable host controller
 * we do not need to model that whole PMIC chain:
 *
 *   - model-level only, no firmware file is changed - the machine rewrites
 *     the LK-produced FDT in guest RAM (mblock "mb_dtb", physical
 *     0x47c80000, 4 MiB; guest log: "mblock_alloc_range ... name:mb_dtb" /
 *     "DT overlay ok, fdt size(4194304)") and turns the zero-length
 *     "usb-role-switch" property of the usb0 node into an FDT_NOP
 *     (token+len+nameoff = 12 bytes -> 3x 0x00000004).
 *   - mtu3 then parses without a role switch, ssusb_otg_switch_init() takes
 *     the "no role-switch, no extcon" path (returns 0) and the xhci host
 *     stays registered.
 *
 * Knobs:
 *   MTK_USB_HOSTMODE=1       enable the injection
 *   MTK_HOSTMODE_ADDR=0x..   override the mb_dtb physical address
 *   MTK_HOSTMODE_MS=<n>      poll period, default 20 ms
 *   MTK_USBLOG=1             trace FDT discovery
 */
static QEMUTimer *mtk_usb_hostmode_timer;
static int mtk_usb_hostmode_state;          /* 0 search, 1 patched, 2 stop */
static int64_t mtk_usb_hostmode_period = 20;
static int64_t mtk_usb_hostmode_deadline;

static const char *mtk_usbhm_str(const uint8_t *fdt, uint32_t total,
                                 uint32_t stroff, uint32_t nameoff)
{
    uint32_t off = stroff + nameoff;

    if (off >= total) {
        return NULL;
    }
    return (const char *)fdt + off;
}

/* Walks the FDT structure block and NOPs usb0's "usb-role-switch" property. */
static int mtk_usbhm_patch_fdt(uint8_t *fdt, uint32_t avail)
{
    uint32_t total = ldl_be_p(fdt + 4);
    uint32_t off_struct = ldl_be_p(fdt + 8);
    uint32_t off_strings = ldl_be_p(fdt + 12);
    uint32_t size_struct = ldl_be_p(fdt + 36);
    uint32_t end = off_struct + size_struct;
    const char *node = "";
    uint32_t i;

    if (ldl_be_p(fdt) != 0xd00dfeed || total > avail || total < 0x100 ||
        off_struct < 0x28 || off_struct >= total || end > total ||
        off_strings >= total) {
        return 0;
    }

    for (i = off_struct; i + 4 <= end; ) {
        uint32_t tok = ldl_be_p(fdt + i);

        if (tok == 1) {                     /* FDT_BEGIN_NODE */
            const char *nm = (const char *)fdt + i + 4;
            size_t len = strnlen(nm, end - (i + 4));

            node = nm;
            i = (i + 4 + (uint32_t)len + 1 + 3) & ~3u;
        } else if (tok == 3) {              /* FDT_PROP */
            uint32_t len = ldl_be_p(fdt + i + 4);
            uint32_t nameoff = ldl_be_p(fdt + i + 8);
            const char *pn = mtk_usbhm_str(fdt, total, off_strings, nameoff);
            uint32_t next = (i + 12 + len + 3) & ~3u;

            if (pn && len == 0 && next + 4 <= end &&
                (!strncmp(node, "usb0@", 5) || !strcmp(node, "usb0")) &&
                !strcmp(pn, "usb-role-switch")) {
                uint32_t after = ldl_be_p(fdt + next);

                if (after != 1 && after != 2 && after != 3 &&
                    after != 4 && after != 9) {
                    return 0;
                }
                stl_be_p(fdt + i, 4);       /* FDT_NOP */
                stl_be_p(fdt + i + 4, 4);
                stl_be_p(fdt + i + 8, 4);
                return 1;
            }
            i = next;
        } else if (tok == 2) {              /* FDT_END_NODE */
            i += 4;
        } else if (tok == 4) {              /* FDT_NOP */
            i += 4;
        } else if (tok == 9) {              /* FDT_END */
            break;
        } else {
            return 0;
        }
    }
    return 0;
}

static void mtk_usb_hostmode_tick(void *opaque)
{
    Mt6895MachineState *s = opaque;
    const char *as = getenv("MTK_HOSTMODE_ADDR");
    const uint64_t win = 4 * MiB;
    uint64_t phys = as ? strtoull(as, NULL, 0) : 0x47c80000ULL;
    int64_t now = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
    uint8_t *ram;
    uint32_t off;

    if (mtk_usb_hostmode_state) {
        return;
    }
    if (phys < MT6895_DRAM_BASE ||
        phys - MT6895_DRAM_BASE > MACHINE(s)->ram_size - win) {
        mtk_usb_hostmode_state = 2;
        fprintf(stderr, "mt6895: MTK_USB_HOSTMODE: mb_dtb %#" PRIx64
                " outside guest RAM\n", phys);
        return;
    }
    ram = memory_region_get_ram_ptr(s->ddr) + (phys - MT6895_DRAM_BASE);

    for (off = 0; off + 0x40 <= win; off += 8) {
        uint32_t total;

        if (ldl_be_p(ram + off) != 0xd00dfeed) {
            continue;
        }
        total = ldl_be_p(ram + off + 4);
        if (total < 0x1000 || total > win - off) {
            continue;
        }
        if (getenv("MTK_USBLOG")) {
            fprintf(stderr, "MTK-USBHM: FDT at %#" PRIx64 " (size %u)\n",
                    phys + off, total);
        }
        if (mtk_usbhm_patch_fdt(ram + off, total)) {
            mtk_usb_hostmode_state = 1;
            fprintf(stderr, "mt6895: MTK_USB_HOSTMODE: NOP'ed usb0 "
                    "usb-role-switch in FDT at %#" PRIx64 "\n", phys + off);
            return;
        }
    }
    if (now > mtk_usb_hostmode_deadline) {
        mtk_usb_hostmode_state = 2;
        fprintf(stderr, "mt6895: MTK_USB_HOSTMODE: gave up, no patchable "
                "FDT at %#" PRIx64 "\n", phys);
        return;
    }
    timer_mod(mtk_usb_hostmode_timer, now + mtk_usb_hostmode_period);
}

static void mtk_usb_hostmode_init(Mt6895MachineState *s)
{
    const char *e = getenv("MTK_USB_HOSTMODE");
    const char *ms = getenv("MTK_HOSTMODE_MS");

    if (!e || !*e || !strcmp(e, "0")) {
        return;
    }
    if (ms) {
        mtk_usb_hostmode_period = strtoll(ms, NULL, 0);
        if (mtk_usb_hostmode_period < 2) {
            mtk_usb_hostmode_period = 2;
        }
        if (mtk_usb_hostmode_period > 1000) {
            mtk_usb_hostmode_period = 1000;
        }
    }
    mtk_usb_hostmode_deadline =
        qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 180000;
    mtk_usb_hostmode_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                          mtk_usb_hostmode_tick, s);
    timer_mod(mtk_usb_hostmode_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 100);
    fprintf(stderr, "mt6895: MTK_USB_HOSTMODE: watching mb_dtb for the "
            "usb0 usb-role-switch property\n");
}

/*
 * ===== USB gadget mode DT injection (env MTK_USB_GADGET=1) =====
 *
 * The vendor DT declares mtu3 as dual-role ("dr_mode = otg") with a
 * "usb-role-switch"; without the typec/tcpc chain the driver ends up in
 * host mode (or defers) and no UDC is registered, so recovery's adbd cannot
 * bind.  For the kernel gadget path we rewrite the usb0 node's dr_mode
 * string in the LK-produced FDT to "peripheral".  The new string does not
 * fit in place (4 -> 11 bytes), so the structure block is grown by 8 bytes
 * (12-byte aligned property slot), with totalsize / off_dt_strings /
 * size_dt_struct patched accordingly.  The 4 MiB mb_dtb mblock leaves
 * plenty of room.  Model-level only; firmware files are untouched.
 *
 * Knobs: MTK_USB_GADGET=1, MTK_GADGET_ADDR=<mb_dtb phys>, MTK_GADGET_MS=<poll>.
 */
static QEMUTimer *mtk_usb_gadget_timer;
static int mtk_usb_gadget_state;
static int64_t mtk_usb_gadget_period = 20;
static int64_t mtk_usb_gadget_deadline;

static int mtk_usbg_patch_fdt(uint8_t *fdt, uint32_t avail)
{
    uint32_t total = ldl_be_p(fdt + 4);
    uint32_t off_struct = ldl_be_p(fdt + 8);
    uint32_t off_strings = ldl_be_p(fdt + 12);
    uint32_t size_struct = ldl_be_p(fdt + 36);
    uint32_t end = off_struct + size_struct;
    const char *node = "";
    uint32_t i;

    if (ldl_be_p(fdt) != 0xd00dfeed || total > avail || total < 0x100 ||
        off_struct < 0x28 || off_struct >= total || end > total ||
        off_strings >= total) {
        return 0;
    }

    for (i = off_struct; i + 4 <= end; ) {
        uint32_t tok = ldl_be_p(fdt + i);

        if (tok == 1) {
            const char *nm = (const char *)fdt + i + 4;
            size_t len = strnlen(nm, end - (i + 4));

            node = nm;
            i = (i + 4 + (uint32_t)len + 1 + 3) & ~3u;
        } else if (tok == 3) {
            uint32_t len = ldl_be_p(fdt + i + 4);
            uint32_t nameoff = ldl_be_p(fdt + i + 8);
            const char *pn = mtk_usbhm_str(fdt, total, off_strings, nameoff);
            uint32_t next = (i + 12 + len + 3) & ~3u;

            if (pn && (!strncmp(node, "usb0@", 5) || !strcmp(node, "usb0")) &&
                !strcmp(pn, "dr_mode") && len == 4) {
                const char *v = (const char *)fdt + i + 12;
                uint32_t delta = 8;         /* 4 -> 12 padded bytes */

                if (!strcmp(v, "peripheral")) {
                    return 2;               /* already patched */
                }
                if (strcmp(v, "otg") != 0) {
                    return 0;               /* host/other: leave alone */
                }
                if (total + delta > avail) {
                    return 0;
                }
                memmove(fdt + next + delta, fdt + next, total - next);
                stl_be_p(fdt + 4, total + delta);
                stl_be_p(fdt + 36, size_struct + delta);
                if (off_strings >= next) {
                    stl_be_p(fdt + 12, off_strings + delta);
                }
                stl_be_p(fdt + i + 4, 11);  /* sizeof("peripheral") */
                memset(fdt + i + 12, 0, 12);
                memcpy(fdt + i + 12, "peripheral", 11);
                return 1;
            }
            i = next;
        } else if (tok == 2 || tok == 4) {
            i += 4;
        } else if (tok == 9) {
            break;
        } else {
            return 0;
        }
    }
    return 0;
}

static void mtk_usb_gadget_tick(void *opaque)
{
    Mt6895MachineState *s = opaque;
    const char *as = getenv("MTK_GADGET_ADDR");
    const uint64_t win = 4 * MiB;
    uint64_t phys = as ? strtoull(as, NULL, 0) : 0x47c80000ULL;
    int64_t now = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
    uint8_t *ram;
    uint32_t off;

    if (mtk_usb_gadget_state) {
        return;
    }
    if (phys < MT6895_DRAM_BASE ||
        phys - MT6895_DRAM_BASE > MACHINE(s)->ram_size - win) {
        mtk_usb_gadget_state = 2;
        fprintf(stderr, "mt6895: MTK_USB_GADGET: mb_dtb outside guest RAM\n");
        return;
    }
    ram = memory_region_get_ram_ptr(s->ddr) + (phys - MT6895_DRAM_BASE);
    for (off = 0; off + 0x40 <= win; off += 8) {
        uint32_t total;

        if (ldl_be_p(ram + off) != 0xd00dfeed) {
            continue;
        }
        total = ldl_be_p(ram + off + 4);
        if (total < 0x1000 || total > win - off) {
            continue;
        }
        {
            int r = mtk_usbg_patch_fdt(ram + off, win - off);

            if (r == 1) {
                if (mtk_usb_gadget_state == 0) {
                    fprintf(stderr, "mt6895: MTK_USB_GADGET: usb0 dr_mode -> "
                            "peripheral in FDT at %#" PRIx64 " (grown +8)\n",
                            phys + off);
                } else {
                    fprintf(stderr, "mt6895: MTK_USB_GADGET: re-patched FDT at "
                            "%#" PRIx64 "\n", phys + off);
                }
                mtk_usb_gadget_state = 1;
            } else if (r == 2 && mtk_usb_gadget_state == 0) {
                fprintf(stderr, "mt6895: MTK_USB_GADGET: usb0 dr_mode already "
                        "peripheral\n");
                mtk_usb_gadget_state = 1;
            }
        }
    }
    if (now > mtk_usb_gadget_deadline) {
        if (mtk_usb_gadget_state == 0) {
            mtk_usb_gadget_state = 2;
            fprintf(stderr, "mt6895: MTK_USB_GADGET: gave up\n");
        }
        return;
    }
    /* Keep watching for a while: LK may rewrite the final FDT after our first
     * successful patch, and a later copy must be patched as well. */
    timer_mod(mtk_usb_gadget_timer,
              now + (mtk_usb_gadget_state ? 100 : mtk_usb_gadget_period));
}

static void mtk_usb_gadget_init(Mt6895MachineState *s)
{
    const char *e = getenv("MTK_USB_GADGET");
    const char *ms = getenv("MTK_GADGET_MS");

    if (!e || !*e || !strcmp(e, "0")) {
        return;
    }
    if (ms) {
        mtk_usb_gadget_period = strtoll(ms, NULL, 0);
        if (mtk_usb_gadget_period < 2) {
            mtk_usb_gadget_period = 2;
        }
        if (mtk_usb_gadget_period > 1000) {
            mtk_usb_gadget_period = 1000;
        }
    }
    mtk_usb_gadget_deadline = qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 180000;
    mtk_usb_gadget_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                        mtk_usb_gadget_tick, s);
    timer_mod(mtk_usb_gadget_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 100);
    fprintf(stderr, "mt6895: MTK_USB_GADGET: watching mb_dtb for usb0 dr_mode\n");
}

/*
 * ===== DMA-address mode: strip the display/cmdq "iommus" properties =====
 *
 * (env MTK_DT_NOIOMMU=1)
 *
 * The vendor kernel puts the GCE and the whole display subsystem behind the
 * M4U (DT "iommus": mdp_iommu for gce, disp_iommu for the display blocks).
 * Every DMA address the cmdq packets carry is then an IOVA that only the M4U
 * page tables can translate - both the command buffer itself (the thread
 * CURR/END registers hold gce = (iova + mminfra) >> shift) and the OVL/RDMA
 * buffer addresses inside the packets.  This model does not implement the M4U
 * page tables, so the GCE would execute IOVA numbers as if they were physical
 * addresses (measured: thread 0 gce=0x87fff800 -> iova 0x3ffffc000, and the
 * auto-calibrated translation landed in the coherent page tables instead).
 *
 * This patch removes the "iommus" property from the display/GCE DT nodes while
 * the kernel boots.  Those devices then use plain physical DMA addresses: the
 * cmdq driver keeps its (shift, mminfra) conversion and mtk_gce.c's "post"
 * translation recovers the physical packet start, and the OVL layer addresses
 * in the packets are physical addresses the display model can scan.  No
 * firmware file is modified - the LK-produced FDT in guest RAM (mblock
 * "mb_dtb", physical 0x47c80000, 4 MiB) is edited in place, exactly like the
 * USB host-mode patch above (FDT_PROP tokens turned into FDT_NOP).
 *
 * Knobs:
 *   MTK_DT_NOIOMMU=1        enable the patch
 *   MTK_NOIOMMU_ADDR=0x..   override the mb_dtb physical address
 *   MTK_NOIOMMU_MS=<n>      poll period, default 20 ms
 *   MTK_NOIOMMU_LOG=1       trace the patched nodes/properties
 */
static QEMUTimer *mtk_dtnoiommu_timer;
static int mtk_dtnoiommu_state;             /* 0 search, 1 patched, 2 stop */
static int64_t mtk_dtnoiommu_period = 500;
static int64_t mtk_dtnoiommu_deadline;
static int mtk_dtnoiommu_count;
static int mtk_dtnoiommu_max_scans = 240;
static bool mtk_dt_hvc_psci_enabled;

/* Node names whose DMA addresses have to stay physical for the display model. */
static bool mtk_dtnoiommu_node_matches(const char *node)
{
    static const char *const pat[] = {
        "gce", "mmsys", "disp", "drm", "ovl", "rdma", "wdma", "dsi",
        "mutex", "mdp", "color", "ccorr", "dither", "gamma", "postmask",
        "dpi", "dsc",
    };
    size_t i;

    for (i = 0; i < ARRAY_SIZE(pat); i++) {
        if (strstr(node, pat[i])) {
            return true;
        }
    }
    return false;
}

/* Walks the FDT structure block and NOPs the iommus property of display nodes. */
static int mtk_dtnoiommu_patch_fdt(uint8_t *fdt, uint32_t avail)
{
    uint32_t total = ldl_be_p(fdt + 4);
    uint32_t off_struct = ldl_be_p(fdt + 8);
    uint32_t off_strings = ldl_be_p(fdt + 12);
    uint32_t size_struct = ldl_be_p(fdt + 36);
    uint32_t end = off_struct + size_struct;
    const char *node = "";
    uint32_t i;
    int nop = 0;
    int extra = 0;
    bool log = getenv("MTK_NOIOMMU_LOG") != NULL;
    bool drmdbg = getenv("MTK_DRM_DEBUG") != NULL;
    bool hvcpsci = mtk_dt_hvc_psci_enabled;
    bool prop_subst = getenv("MTK_FDT_PROP_SUBST") != NULL;

    if (ldl_be_p(fdt) != 0xd00dfeed || total > avail || total < 0x100 ||
        off_struct < 0x28 || off_struct >= total || end > total ||
        off_strings >= total) {
        return -1;
    }

    for (i = off_struct; i + 4 <= end; ) {
        uint32_t tok = ldl_be_p(fdt + i);

        if (tok == 1) {                     /* FDT_BEGIN_NODE */
            const char *nm = (const char *)fdt + i + 4;
            size_t len = strnlen(nm, end - (i + 4));

            node = nm;
            i = (i + 4 + (uint32_t)len + 1 + 3) & ~3u;
        } else if (tok == 3) {              /* FDT_PROP */
            uint32_t len = ldl_be_p(fdt + i + 4);
            uint32_t nameoff = ldl_be_p(fdt + i + 8);
            uint32_t next = (i + 12 + len + 3) & ~3u;
            const char *pn = NULL;

            if (off_strings + nameoff < total) {
                pn = (const char *)fdt + off_strings + nameoff;
            }
            if (pn && !strcmp(pn, "iommus") &&
                mtk_dtnoiommu_node_matches(node) && next + 4 <= end) {
                uint32_t after = ldl_be_p(fdt + next);

                if (after == 1 || after == 2 || after == 3 || after == 4 ||
                    after == 9) {
                    if (log) {
                        fprintf(stderr,
                                "MTK-NOIOMMU: NOP iommus of node \"%s\" "
                                "(len %u)\n", node, len);
                    }
                    /*
                     * Turn the whole property - token, length, name offset
                     * AND the value words - into FDT_NOP tokens: leaving the
                     * value behind desynchronises every later FDT walk (LK's
                     * mboot appends reserved-memory to this same tree and
                     * asserts out with "nodeoffset >= 0" otherwise).
                     */
                    for (uint32_t w = 0; w < ((12u + len + 3u) & ~3u);
                         w += 4) {
                        stl_be_p(fdt + i + w, 4);
                    }
                    nop++;
                }
            }

            /*
             * env MTK_DT_HVC_PSCI=1: make Linux use the HVC PSCI conduit
             * instead of SMC.  The vendor FDT's /psci/method is "smc" (a
             * 4-byte string); substituting "hvc" in place keeps both the
             * property length and the FDT layout untouched, so the tree
             * stays valid.  QEMU then answers the kernel's CPU_ON itself
             * through arm_set_cpu_on(), while every other SMC (the MTK
             * SiP calls) still traps to the real ATF - which is what the
             * firmware expects and what keeps the vendor kernel happy.
             */
            if (hvcpsci && pn && !strcmp(pn, "method") &&
                !strncmp(node, "psci", 4)) {
                uint8_t *val = fdt + i + 12;

                if (len == 4 && !memcmp(val, "hvc", 3)) {
                    /* already patched by an earlier scan */
                } else if (len == 4 && !memcmp(val, "smc", 3)) {
                    memcpy(val, "hvc\0", 4);
                    fprintf(stderr, "mt6895: MTK_DT_HVC_PSCI: /psci/method "
                            "\"smc\" -> \"hvc\"\n");
                    extra++;
                } else {
                    fprintf(stderr, "mt6895: MTK_DT_HVC_PSCI: unexpected "
                            "/psci/method length %u\n", len);
                }
            }
            /*
             * Diagnostic: turn the kernel cmdline on (drm.debug) so the DRM
             * atomic-commit path of the userspace (recovery/minui) becomes
             * visible in the serial log.  The cmdline lives in /chosen/bootargs
             * of the same FDT; substitute a same-length token, never change the
             * property length (the FDT would have to be rebuilt otherwise).
             */
            if (drmdbg && pn && !strcmp(pn, "bootargs") &&
                !strcmp(node, "chosen")) {
                static const char from[] = "usb2jtag_mode=0";
                static const char to[] = "drm.debug=0x1f";
                uint8_t *val = fdt + i + 12;
                uint32_t k;

                for (k = 0; k + sizeof(from) - 1 <= len; k++) {
                    uint32_t j;

                    if (memcmp(val + k, from, sizeof(from) - 1)) {
                        continue;
                    }
                    memcpy(val + k, to, sizeof(to) - 1);
                    for (j = sizeof(to) - 1; j < sizeof(from) - 1; j++) {
                        val[k + j] = ' ';
                    }
                    fprintf(stderr, "mt6895: MTK_DRM_DEBUG: substituted "
                            "\"%s\" for \"%s\" in /chosen/bootargs\n",
                            to, from);
                    extra++;
                    break;
                }
            }
            /*
             * Generic /chosen/bootargs substitution (env
             * MTK_FDT_BOOTARGS_SUBST="old:new"): same-length in-place edit,
             * padded with spaces, so the FDT layout never changes.  Used for
             * kernel-parameter experiments such as cma=... or drm.debug=...
             */
            if (pn && !strcmp(pn, "bootargs") && !strcmp(node, "chosen")) {
                const char *bs = getenv("MTK_FDT_BOOTARGS_SUBST");

                if (bs && *bs) {
                    const char *colon = strchr(bs, ':');

                    if (colon && colon != bs) {
                        size_t fl = (size_t)(colon - bs);
                        size_t tl = strlen(colon + 1);
                        uint8_t *val = fdt + i + 12;
                        uint32_t k;

                        if (tl <= fl && fl <= len) {
                            for (k = 0; k + fl <= len; k++) {
                                uint32_t j;

                                if (memcmp(val + k, bs, fl)) {
                                    continue;
                                }
                                memcpy(val + k, colon + 1, tl);
                                for (j = tl; j < fl; j++) {
                                    val[k + j] = ' ';
                                }
                                fprintf(stderr, "mt6895: "
                                        "MTK_FDT_BOOTARGS_SUBST: "
                                        "substituted \"%s\" for \"%s\"\n",
                                        colon + 1, bs);
                                extra++;
                                break;
                            }
                        }
                    }
                }
            }
            /*
             * Generic same-length property-value substitution (env
             * MTK_FDT_PROP_SUBST="old:new"): replaces every occurrence of
             * the ASCII token "old" inside any property value with "new"
             * (both tokens must have identical length; the property length
             * and the FDT layout stay untouched).  Lets vendor-tree policy
             * strings be flipped without touching firmware, e.g.
             * /hwrng "methods" = "smc" -> "hvc".
             */
            if (prop_subst && pn && len) {
                const char *ps = getenv("MTK_FDT_PROP_SUBST");
                const char *pcolon = ps ? strchr(ps, ':') : NULL;

                if (pcolon && pcolon != ps) {
                    size_t fl = (size_t)(pcolon - ps);
                    size_t tl = strlen(pcolon + 1);
                    uint8_t *pval = fdt + i + 12;
                    uint32_t pk;

                    if (fl && tl == fl && fl <= len) {
                        for (pk = 0; pk + fl <= len; pk++) {
                            if (memcmp(pval + pk, ps, fl)) {
                                continue;
                            }
                            memcpy(pval + pk, pcolon + 1, tl);
                            fprintf(stderr, "mt6895: "
                                    "MTK_FDT_PROP_SUBST: node \"%s\" "
                                    "prop \"%s\": \"%s\" -> \"%s\"\n",
                                    node, pn, ps, pcolon + 1);
                            extra++;
                            pk += (uint32_t)fl - 1;
                        }
                    }
                }
            }
            i = next;
        } else if (tok == 2) {              /* FDT_END_NODE */
            i += 4;
        } else if (tok == 4) {              /* FDT_NOP */
            i += 4;
        } else if (tok == 9) {              /* FDT_END */
            break;
        } else {
            return -1;
        }
    }
    return nop + extra;
}

static int mtk_dtnoiommu_scans;

static void mtk_dtnoiommu_tick(void *opaque)
{
    Mt6895MachineState *s = opaque;
    const char *as = getenv("MTK_NOIOMMU_ADDR");
    const uint64_t win = 4 * MiB;
    uint64_t phys = as ? strtoull(as, NULL, 0) : 0x47c80000ULL;
    int64_t now = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
    uint8_t *ram;
    uint32_t off;

    if (mtk_dtnoiommu_state == 2) {
        return;
    }
    if (phys < MT6895_DRAM_BASE ||
        phys - MT6895_DRAM_BASE > MACHINE(s)->ram_size - win) {
        mtk_dtnoiommu_state = 2;
        fprintf(stderr, "mt6895: MTK_DT_NOIOMMU: mb_dtb %#" PRIx64
                " outside guest RAM\n", phys);
        return;
    }
    ram = memory_region_get_ram_ptr(s->ddr) + (phys - MT6895_DRAM_BASE);

    for (off = 0; off + 0x40 <= win; off += 8) {
        uint32_t total;
        int nop;

        if (ldl_be_p(ram + off) != 0xd00dfeed) {
            continue;
        }
        total = ldl_be_p(ram + off + 4);
        if (total < 0x1000 || total > win - off) {
            continue;
        }
        nop = mtk_dtnoiommu_patch_fdt(ram + off, total);
        if (nop > 0) {
            if (!mtk_dtnoiommu_state || nop != mtk_dtnoiommu_count) {
                fprintf(stderr, "mt6895: MTK_DT_NOIOMMU: patched %d FDT "
                        "items (iommus/bootargs) at %#" PRIx64 "\n",
                        nop, phys + off);
            }
            mtk_dtnoiommu_state = 1;
            mtk_dtnoiommu_count = nop;
            break;
        }
    }
    mtk_dtnoiommu_scans++;
    if (mtk_dtnoiommu_scans >= mtk_dtnoiommu_max_scans ||
        now > mtk_dtnoiommu_deadline) {
        mtk_dtnoiommu_state = 2;
        if (!mtk_dtnoiommu_count) {
            fprintf(stderr, "mt6895: MTK_DT_NOIOMMU: gave up, no patchable "
                    "FDT at %#" PRIx64 "\n", phys);
        }
        return;
    }
    timer_mod(mtk_dtnoiommu_timer, now + mtk_dtnoiommu_period);
}

static void mtk_dtnoiommu_init(Mt6895MachineState *s)
{
    const char *e = getenv("MTK_DT_NOIOMMU");
    const char *hvc = getenv("MTK_DT_HVC_PSCI");
    const char *ms = getenv("MTK_NOIOMMU_MS");

    mtk_dt_hvc_psci_enabled = hvc && *hvc && strcmp(hvc, "0") != 0;
    if ((!e || !*e || !strcmp(e, "0")) && !mtk_dt_hvc_psci_enabled &&
        !getenv("MTK_FDT_BOOTARGS_SUBST") &&
        !getenv("MTK_FDT_PROP_SUBST")) {
        return;
    }
    if (ms) {
        mtk_dtnoiommu_period = strtoll(ms, NULL, 0);
        if (mtk_dtnoiommu_period < 2) {
            mtk_dtnoiommu_period = 2;
        }
        if (mtk_dtnoiommu_period > 1000) {
            mtk_dtnoiommu_period = 1000;
        }
    } else if (mtk_dt_hvc_psci_enabled) {
        /*
         * Linux probes /psci in its very first initcalls, so the FDT has
         * to be patched within milliseconds of LK publishing it: poll
         * fast, and keep scanning long enough for LK's slow boot.
         */
        mtk_dtnoiommu_period = 20;
    }
    if (mtk_dt_hvc_psci_enabled) {
        mtk_dtnoiommu_max_scans = 6000;
    }
    mtk_dtnoiommu_deadline =
        qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 430000;
    mtk_dtnoiommu_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                       mtk_dtnoiommu_tick, s);
    timer_mod(mtk_dtnoiommu_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
              (mtk_dt_hvc_psci_enabled ? 1 : 100));
    fprintf(stderr, "mt6895: %s: watching mb_dtb to %s\n",
            mtk_dt_hvc_psci_enabled ? "MTK_DT_HVC_PSCI" : "MTK_DT_NOIOMMU",
            mtk_dt_hvc_psci_enabled ? "switch /psci method to hvc"
                                    : "strip display/GCE iommus");
}

/*
 * DIAGNOSTIC (env MTK_SMPDBG=1): dump every CPU's PC/PSTATE/halted state on a
 * realtime timer.  Used to find out where a secondary core stops when Linux
 * hangs in "smp: Bringing up secondary CPUs".  100% read-only, off by
 * default, and it runs on the main loop so it also works while all vCPUs spin.
 */
static QEMUTimer *mtk_smpdbg_timer;

static uint64_t mtk_smpdbg_last[8];
static int mtk_smpdbg_seen[8];
static int mtk_smpdbg_ticks;

static void mtk_smpdbg_tick(void *opaque)
{
    MachineState *ms = MACHINE(opaque);
    int i;

    for (i = 0; i < ms->smp.cpus && i < 8; i++) {
        CPUState *cs = qemu_get_cpu(i);
        CPUARMState *env;

        if (!cs) {
            continue;
        }
        env = &ARM_CPU(cs)->env;
        if (!mtk_smpdbg_seen[i] && (env->pc || !cs->halted)) {
            mtk_smpdbg_seen[i] = 1;
            fprintf(stderr, "MTK-SMPDBG EVENT c%d started pc=%#llx scr3=%#llx"
                    " pstate=%#llx halted=%d\n",
                    i, (unsigned long long)env->pc,
                    (unsigned long long)env->cp15.scr_el3,
                    (unsigned long long)env->pstate, (int)cs->halted);
        }
        if (env->pc != mtk_smpdbg_last[i]) {
            mtk_smpdbg_last[i] = env->pc;
            fprintf(stderr, "MTK-SMPDBG PC c%d %#llx h=%d scr3=%#llx\n",
                    i, (unsigned long long)env->pc, (int)cs->halted,
                    (unsigned long long)env->cp15.scr_el3);
        }
    }
    mtk_smpdbg_ticks++;
    if (mtk_smpdbg_ticks >= 700) {
        int n = 0;

        for (i = 0; i < ms->smp.cpus && i < 8; i++) {
            n += mtk_smpdbg_seen[i] ? 1 : 0;
        }
        fprintf(stderr, "MTK-SMPDBG SUMMARY cpus_seen=%d\n", n);
        return;
    }
    timer_mod(mtk_smpdbg_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 200);
}

static void mtk_smpdbg_init(Mt6895MachineState *s)
{
    const char *e = getenv("MTK_SMPDBG");
    const char *n = getenv("MTK_SMPDBG_N");

    if (!e || !*e || !strcmp(e, "0")) {
        return;
    }
    (void)n;
    mtk_smpdbg_timer = timer_new_ms(QEMU_CLOCK_REALTIME, mtk_smpdbg_tick, s);
    timer_mod(mtk_smpdbg_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 500);
    fprintf(stderr, "mt6895: MTK_SMPDBG armed (200 ms boot-phase PC trail)\n");
}

/*
 * DIAGNOSTIC (env MTK_KMEM_SCAN="needle"): scan guest RAM for an ASCII needle
 * (typically a string known to be in the kernel printk ring buffer) and dump
 * the printable strings of the surrounding 512 KiB.  This is how the kernel
 * log is recovered on a run whose serial console dies: the ring buffer holds
 * every pr_info/pr_err even when the console chardev stops receiving bytes.
 *   MTK_KMEM_SCAN   needle to look for
 *   MTK_KMEM_MS     when to scan, default 120000 ms after machine init
 */
static QEMUTimer *mtk_kmem_timer;
static int64_t mtk_kmem_deadline;
static bool mtk_kmem_done;

static void mtk_kmem_dump(Mt6895MachineState *s, uint64_t off, uint64_t len)
{
    uint8_t *ram = memory_region_get_ram_ptr(s->ddr);
    uint64_t size = MACHINE(s)->ram_size;
    char line[200];
    unsigned n = 0;
    uint64_t i;
    bool instr = false;

    fprintf(stderr, "MTK-KLOG: dumping %#" PRIx64 "..%#" PRIx64 "\n",
            off, off + len);
    for (i = off; i < off + len && i < size; i++) {
        unsigned char c = ram[i];

        if (c >= 0x20 && c < 0x7f) {
            if (n < sizeof(line) - 1) {
                line[n++] = (char)c;
            }
            instr = true;
        } else {
            if (instr && n >= 8) {
                line[n] = 0;
                fprintf(stderr, "MTK-KLOG %s\n", line);
            }
            n = 0;
            instr = false;
        }
    }
}

static void mtk_kmem_tick(void *opaque)
{
    Mt6895MachineState *s = opaque;
    const char *needle = getenv("MTK_KMEM_SCAN");
    uint8_t *ram = memory_region_get_ram_ptr(s->ddr);
    uint64_t size = MACHINE(s)->ram_size;
    size_t nlen;
    uint64_t off;
    int hits = 0;
    int64_t now = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);

    if (mtk_kmem_done) {
        return;
    }
    if (now < mtk_kmem_deadline) {
        timer_mod(mtk_kmem_timer, now + 1000);
        return;
    }
    mtk_kmem_done = true;
    if (!needle || !*needle) {
        return;
    }
    nlen = strlen(needle);
    fprintf(stderr, "MTK-KLOG: scanning %#" PRIx64 " bytes for \"%s\"\n",
            size, needle);
    for (off = 0; off + nlen < size; off++) {
        if (ram[off] != (uint8_t)needle[0] ||
            memcmp(ram + off, needle, nlen)) {
            continue;
        }
        hits++;
        fprintf(stderr, "MTK-KLOG: hit #%d at %#" PRIx64 "\n", hits, off);
        mtk_kmem_dump(s, off > 0x40000 ? off - 0x40000 : 0, 0x80000);
        if (hits >= 3) {
            break;
        }
    }
    fprintf(stderr, "MTK-KLOG: scan done, hits=%d\n", hits);
}

static void mtk_kmem_init(Mt6895MachineState *s)
{
    const char *e = getenv("MTK_KMEM_SCAN");
    const char *ms = getenv("MTK_KMEM_MS");

    if (!e || !*e) {
        return;
    }
    mtk_kmem_deadline = qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
                        (ms ? strtoll(ms, NULL, 0) : 120000);
    mtk_kmem_timer = timer_new_ms(QEMU_CLOCK_REALTIME, mtk_kmem_tick, s);
    timer_mod(mtk_kmem_timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 1000);
    fprintf(stderr, "mt6895: MTK_KMEM_SCAN armed for \"%s\" at +%" PRId64
            " ms\n", e, mtk_kmem_deadline -
            qemu_clock_get_ms(QEMU_CLOCK_REALTIME));
}

/*
 * DIAGNOSTIC (env MTK_KCMDLINE_SUBST="old:new"): the kernel command line is
 * built by LK and passed to Linux through the FDT / boot tag; the FDT-based
 * patcher above can miss the copy the kernel actually reads (measured: patching
 * /chosen/bootargs did not change the "Kernel command line:" log).  This
 * scanner walks the low 256 MiB of DRAM a few times during the LK->kernel
 * hand-over and substitutes every raw copy of the token in place (same length,
 * padded with spaces), e.g. to inject drm.debug for a diagnostic run.
 */
static QEMUTimer *mtk_cmdline_timer;
static int mtk_cmdline_scans;

/*
 * DISP mutex frame interrupt (mutex block at 0x14001000 / 0x14401000).
 *
 * The vendor DRM driver requests the level-sensitive GIC SPI of the mutex
 * block (device tree disp_mutex@14001000: interrupts = <0 300 4>, <0 344 4>)
 * and enables the per-mutex SOF/EOF events in DISP_REG_MUTEX_INTEN (offset
 * 0x00, bits 0..5 = SOF of mutex 0..5, bits 16..21 = EOF of mutex 0..5;
 * DISP_MUTEX_TOTAL is 16 in the vendor kernel).  DISP_REG_MUTEX_INTSTA
 * (offset 0x04) latches those events and the IRQ line is asserted only while
 * (INTSTA & INTEN) != 0.  A bootloader that leaves INTEN at zero therefore
 * cannot make the kernel take display interrupts before it programs INTEN
 * itself; the previous time-based startup gate has been removed.
 *
 * The frame cadence itself is still generated by the model (a real panel
 * free-runs at ~60 Hz): every tick latches SOF/EOF for each mutex whose
 * hardware enable bit (EN at 0x20 + 0x20*n, bit 0) is set, then re-evaluates
 * the line.  The timer runs on QEMU_CLOCK_VIRTUAL: with -icount the guest
 * clock is derived from the executed instruction count, so a 16.666 ms virtual
 * period is exactly one 60 Hz frame of guest time (the earlier host-clock timer
 * ran at ~8 Hz in guest time).  The interrupt semantics (status latching,
 * INTEN gating, level line, acknowledge) follow hardware.
 *
 * Enabled by default; MTK_DISP_VBLANK=0 or MTK_NO_DISP_VBLANK=1 restores the
 * plain register window without the timer/IRQ wiring.  MTK_DISP_VBLANK_HZ
 * overrides the frame rate (default 60) and MTK_DISP_VBLANK_LOG traces the
 * interrupt traffic.
 *
 * INTSTA acknowledge: a status bit written as 1 keeps its state, a bit
 * written as 0 is cleared.  That is what the kernel drivers rely on: the
 * mutex handler does writel(~val, DISP_REG_MUTEX_INTSTA) on the latched bits
 * (mtk_drm_ddp.c, mtk_disp_mutex_irq_handler), the DSI/OVL/RDMA/WDMA
 * handlers use the same ~status pattern, and the upstream DSI driver clears
 * with mtk_dsi_mask(..., status, 0).  A write-1-to-clear register would leave
 * those bits set and the level line would never go low.
 */
#define MTK_DISP_MUTEX_INTEN_OFF    0x00
#define MTK_DISP_MUTEX_INTSTA_OFF   0x04
#define MTK_DISP_MUTEX_IRQ_MASK     0x003f003fu    /* SOF 0..5, EOF 16..21 */

static bool mtk_disp_vblank_enabled(void)
{
    const char *e = getenv("MTK_DISP_VBLANK");

    if (getenv("MTK_NO_DISP_VBLANK")) {
        return false;
    }
    /* Default on: the mutex interrupt block always exists in hardware.  Only
     * an explicit MTK_DISP_VBLANK=0 keeps the pre-model behaviour. */
    return !(e && (!*e || !strcmp(e, "0")));
}

static int64_t mtk_disp_vblank_period_ns = NANOSECONDS_PER_SECOND / 60;
static bool mtk_disp_vblank_log;
static uint64_t mtk_disp_vblank_frames;
static uint64_t mtk_disp_vblank_irqs;
static uint64_t mtk_disp_vblank_acks;
static uint64_t mtk_disp_vblank_report_frames = 3600;

/* Guest time in seconds, used by the diagnostic lines only. */
static double mtk_guest_seconds(void)
{
    return (double)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1e9;
}

/*
 * Re-evaluate the level-sensitive interrupt line: asserted only while a
 * latched INTSTA bit is also enabled in INTEN.  Called whenever INTSTA or
 * INTEN changes (frame tick, INTEN write, INTSTA acknowledge).
 */
static void mtk_disp_mutex_update_irq(Mt6895MachineState *s, int idx)
{
    uint32_t inten, intsta;
    bool level;

    if (!s->disp_mutex_irq[idx]) {
        return;
    }
    inten = s->disp_mutex_regs[idx][MTK_DISP_MUTEX_INTEN_OFF >> 2];
    intsta = s->disp_mutex_regs[idx][MTK_DISP_MUTEX_INTSTA_OFF >> 2];
    level = (inten & intsta & MTK_DISP_MUTEX_IRQ_MASK) != 0;

    if (level && !s->disp_mutex_irq_level[idx]) {
        qemu_irq_raise(s->disp_mutex_irq[idx]);
        s->disp_mutex_irq_level[idx] = true;
        mtk_disp_vblank_irqs++;
        if (mtk_disp_vblank_log && mtk_disp_vblank_irqs <= 8) {
            fprintf(stderr,
                    "MTK-DISP-VBLANK: irq deliver instance %d guest=%.3fs"
                    " INTEN=%#x INTSTA=%#x\n",
                    idx, mtk_guest_seconds(), inten, intsta);
        }
    } else if (!level && s->disp_mutex_irq_level[idx]) {
        qemu_irq_lower(s->disp_mutex_irq[idx]);
        s->disp_mutex_irq_level[idx] = false;
    }
}

/* DSI0 register offsets (vendor mtk_dsi.c) on the 0x14017000 window. */
#define MTK_DSI_START           0x00
#define MTK_DSI_INTEN           0x08
#define MTK_DSI_INTSTA          0x0c
#define MTK_DSI_CON_CTRL        0x10
#define MTK_DSI_MODE_CTRL       0x14
#define MTK_DSI_CMDQ_SIZE       0x60
#define MTK_DSI_RX_DATA         0x74
#define MTK_DSI_RACK            0x84
#define MTK_DSI_CMD_WORDS       0xd00

/* DSI_START bits. */
#define MTK_DSI_START_VDO       (1u << 0)   /* start video stream / transfer */
#define MTK_DSI_START_SLEEPOUT  (1u << 2)   /* start sleep-out (ULPS exit)   */
#define MTK_DSI_START_VM_CMD    (1u << 16)  /* video-mode command trigger    */

/* DSI_CON_CTRL bits. */
#define MTK_DSI_CON_DSI_EN      (1u << 1)

/* DSI_MODE_CTRL MODE field: 0 = command mode, 1..3 = video modes. */
#define MTK_DSI_MODE_MASK       0x3u

/* DSI_INTSTA status bits. */
#define MTK_DSI_INT_LPRX_RD_RDY      (1u << 0)
#define MTK_DSI_INT_CMD_DONE         (1u << 1)
#define MTK_DSI_INT_TE_RDY           (1u << 2)
#define MTK_DSI_INT_VM_DONE          (1u << 3)
#define MTK_DSI_INT_FRAME_DONE       (1u << 4)
#define MTK_DSI_INT_VM_CMD_DONE      (1u << 5)
#define MTK_DSI_INT_SLEEPOUT_DONE    (1u << 6)
#define MTK_DSI_INT_SLEEPIN_ULPS_DONE (1u << 15)
/* Status bits the level interrupt may follow.  Bits 12/14 (underrun) are
 * deliberately never latched by the model. */
#define MTK_DSI_IRQ_MASK             (0x7fu | MTK_DSI_INT_SLEEPIN_ULPS_DONE)

static int64_t mtk_dsi_frame_period_ns = NANOSECONDS_PER_SECOND / 60;
static bool mtk_dsi_trace;          /* MTK_DSITRACE: control writes + stats  */
static bool mtk_dsi_trace_all;      /* MTK_DSITRACE_ALL: every access        */

/*
 * DSI0 interrupt line (GIC SPI 323).
 *
 * INTSTA (0x0c) latches status independently of INTEN (0x08); the shared,
 * level-high interrupt is asserted while a latched status bit is also
 * enabled, and drops as soon as the handler acknowledges the status.  The
 * kernel's mtk_dsi_irq_status() reads INTSTA once, writes ~status to
 * acknowledge it (INTSTA is write-0-clears) and treats FRAME_DONE (bit4) as
 * the DRM vblank tick, so the line has to follow (INTSTA & INTEN).
 */
static void mtk_dsi_update_irq(Mt6895MachineState *s)
{
    uint32_t inten, intsta;
    bool level;

    if (!s->dsi_irq) {
        return;
    }
    inten = s->dsi_regs[MTK_DSI_INTEN >> 2];
    intsta = s->dsi_regs[MTK_DSI_INTSTA >> 2];
    level = (inten & intsta & MTK_DSI_IRQ_MASK) != 0;

    if (level && !s->dsi_irq_level) {
        qemu_irq_raise(s->dsi_irq);
        s->dsi_irq_level = true;
        s->dsi_irq_count++;
        if (mtk_dsi_trace &&
            (s->dsi_irq_count <= 8 || (s->dsi_irq_count % 600) == 0)) {
            fprintf(stderr,
                    "MTK-DSI: irq %u guest=%.3fs INTEN=%#x INTSTA=%#x\n",
                    s->dsi_irq_count, mtk_guest_seconds(), inten, intsta);
        }
    } else if (!level && s->dsi_irq_level) {
        qemu_irq_lower(s->dsi_irq);
        s->dsi_irq_level = false;
    }
}

/*
 * DSI video-mode frame engine.
 *
 * The xaga panel is a free-running 60 Hz video-mode panel: once the
 * controller is enabled (CON_CTRL.DSI_EN), a video mode is selected
 * (MODE_CTRL.MODE != 0) and the stream is started (DSI_START bit0), the
 * controller transmits one frame every 1/vrefresh with no further CPU or GCE
 * trigger.  Each frame end latches INTSTA.FRAME_DONE (status latches
 * regardless of INTEN; only the GIC line is gated by INTEN) and raises the
 * CMDQ event disp_dsi0_eof, which is what completes a display commit packet;
 * frame start raises disp_dsi0_sof0 for the VFP/msync waits.
 */
/*
 * Video streaming condition.
 *
 * LK hands the panel over in video mode with the frame stream already
 * running: its own register dump at the end of primary_display_trigger reads
 * back DSI_START = 1, MODE_CTRL = SYNC_PULSE_VDO_MODE (1) and CON_CTRL = 0
 * (see "DSI_DumpRegisters" / "dsi_analysis" in the LK serial log), and the
 * splash is on screen at that point.  The frame cadence is therefore gated by
 * MODE_CTRL.MODE != 0 plus DSI_START bit0; gating it on CON_CTRL.DSI_EN as
 * well would stop the stream at handoff and the first kernel commit, whose
 * wait-for-DSI0-EOF packet (GCE event 313) must complete, would never run.
 * CON_CTRL.DSI_EN is still stored and reported by the trace, and the kernel
 * sets it again in mtk_dsi_poweron().
 */
static bool mtk_dsi_wants_stream(Mt6895MachineState *s)
{
    return ((s->dsi_regs[MTK_DSI_MODE_CTRL >> 2] & MTK_DSI_MODE_MASK) != 0) &&
           (s->dsi_regs[MTK_DSI_START >> 2] & MTK_DSI_START_VDO);
}

static void mtk_dsi_frame_tick(void *opaque)
{
    Mt6895MachineState *s = opaque;

    if (!s->dsi_streaming) {
        return;
    }
    /* Frame end: FRAME_DONE status + CMDQ disp_dsi0_eof (313). */
    s->dsi_regs[MTK_DSI_INTSTA >> 2] |= MTK_DSI_INT_FRAME_DONE;
    s->dsi_frame_count++;
    mtk_dsi_update_irq(s);
    if (s->gce) {
        mtk_gce_event_signal(s->gce, MTK_GCE_EVENT_DSI0_EOF);
        /* The next frame starts at the same instant: disp_dsi0_sof0. */
        mtk_gce_event_signal(s->gce, MTK_GCE_EVENT_DSI0_SOF);
    }
    if (mtk_dsi_trace && (s->dsi_frame_count % 60) == 0) {
        fprintf(stderr,
                "MTK-DSI: frames=%" PRIu64 " irqs=%u guest=%.3fs"
                " INTEN=%#x INTSTA=%#x\n",
                s->dsi_frame_count, s->dsi_irq_count, mtk_guest_seconds(),
                s->dsi_regs[MTK_DSI_INTEN >> 2],
                s->dsi_regs[MTK_DSI_INTSTA >> 2]);
    }
    /* Guest time, not host time: see the mutex timer comment above. */
    timer_mod(s->dsi_frame_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              mtk_dsi_frame_period_ns);
}

/* Start/stop the frame engine after a write to START/CON_CTRL/MODE_CTRL. */
static void mtk_dsi_update_stream(Mt6895MachineState *s)
{
    bool want = mtk_dsi_wants_stream(s);

    if (want && !s->dsi_streaming) {
        s->dsi_streaming = true;
        /* Video mode entered: VM_DONE latches (legacy handler only). */
        s->dsi_regs[MTK_DSI_INTSTA >> 2] |= MTK_DSI_INT_VM_DONE;
        mtk_dsi_update_irq(s);
        if (s->dsi_frame_timer) {
            timer_mod(s->dsi_frame_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      mtk_dsi_frame_period_ns);
        }
        if (mtk_dsi_trace) {
            fprintf(stderr,
                    "MTK-DSI: stream start guest=%.3fs MODE_CTRL=%#x"
                    " START=%#x period=%" PRId64 "ns\n",
                    mtk_guest_seconds(),
                    s->dsi_regs[MTK_DSI_MODE_CTRL >> 2],
                    s->dsi_regs[MTK_DSI_START >> 2], mtk_dsi_frame_period_ns);
        }
    } else if (!want && s->dsi_streaming) {
        s->dsi_streaming = false;
        if (s->dsi_frame_timer) {
            timer_del(s->dsi_frame_timer);
        }
        if (mtk_dsi_trace) {
            fprintf(stderr,
                    "MTK-DSI: stream stop guest=%.3fs frames=%" PRIu64
                    " CON_CTRL=%#x MODE_CTRL=%#x START=%#x\n",
                    mtk_guest_seconds(), s->dsi_frame_count,
                    s->dsi_regs[MTK_DSI_CON_CTRL >> 2],
                    s->dsi_regs[MTK_DSI_MODE_CTRL >> 2],
                    s->dsi_regs[MTK_DSI_START >> 2]);
        }
    }
}

static void mtk_disp_vblank_tick(void *opaque)
{
    Mt6895MachineState *s = opaque;
    int idx, n;

    mtk_disp_vblank_frames++;
    for (idx = 0; idx < 2; idx++) {
        uint32_t mask = 0;

        for (n = 0; n < 6; n++) {
            if (s->disp_mutex_regs[idx][(0x20 + 0x20 * n) >> 2] & 1u) {
                /* mutex n SOF/EOF interrupt bits, see the handler mask. */
                mask |= (1u << n) | (1u << (16 + n));
            }
        }
        if (!mask) {
            continue;
        }
        /*
         * SOF/EOF status latches for every enabled mutex regardless of
         * INTEN; only the line follows (INTSTA & INTEN).
         */
        s->disp_mutex_regs[idx][MTK_DISP_MUTEX_INTSTA_OFF >> 2] |= mask;
        mtk_disp_mutex_update_irq(s, idx);
        if (idx == 0 && s->gce) {
            /* disp_mutex0_eof: the display commit packets wait on it.  The
             * block covers mutex ids 0..5; only mutex 0 is used on xaga. */
            mtk_gce_event_signal(s->gce, MTK_GCE_EVENT_MUTEX0_EOF);
        }
    }
    if (MTK_ENV_CACHED("MTK_NO_DSI_MODEL")) {
        /*
         * Fallback only: without the behavioural DSI controller there is no
         * frame engine, so keep the old vblank-driven FRAME_DONE latch.
         */
        if (s->dsi_regs[MTK_DSI_INTEN >> 2] & MTK_DSI_INT_FRAME_DONE) {
            s->dsi_regs[MTK_DSI_INTSTA >> 2] |= MTK_DSI_INT_FRAME_DONE;
            mtk_dsi_update_irq(s);
        }
    }
    if (mtk_disp_vblank_log && mtk_disp_vblank_report_frames &&
        (mtk_disp_vblank_frames % mtk_disp_vblank_report_frames) == 0) {
        fprintf(stderr,
                "MTK-DISP-VBLANK: stats frames=%" PRIu64 " irqs=%" PRIu64
                " acks=%" PRIu64 " inten0=%#x intsta0=%#x inten1=%#x"
                " intsta1=%#x\n",
                mtk_disp_vblank_frames, mtk_disp_vblank_irqs,
                mtk_disp_vblank_acks,
                s->disp_mutex_regs[0][0], s->disp_mutex_regs[0][1],
                s->disp_mutex_regs[1][0], s->disp_mutex_regs[1][1]);
    }
    timer_mod(s->disp_vblank_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              mtk_disp_vblank_period_ns);
}

static void mtk_disp_vblank_init(Mt6895MachineState *s)
{
    const char *hz_s;

    if (!mtk_disp_vblank_enabled()) {
        return;
    }
    mtk_disp_vblank_log = getenv("MTK_DISP_VBLANK_LOG") != NULL;
    mtk_disp_vblank_period_ns = NANOSECONDS_PER_SECOND / 60;
    hz_s = getenv("MTK_DISP_VBLANK_HZ");
    if (hz_s && *hz_s) {
        int64_t hz = strtoll(hz_s, NULL, 0);

        if (hz < 1) {
            hz = 1;
        } else if (hz > 1000) {
            hz = 1000;
        }
        mtk_disp_vblank_period_ns = NANOSECONDS_PER_SECOND / hz;
    }
    mtk_disp_vblank_report_frames =
        (uint64_t)(60 * NANOSECONDS_PER_SECOND / mtk_disp_vblank_period_ns);
    s->disp_mutex_irq[0] = qdev_get_gpio_in(s->gic, 300);
    s->disp_mutex_irq[1] = qdev_get_gpio_in(s->gic, 344);
    s->disp_vblank_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                        mtk_disp_vblank_tick, s);
    timer_mod(s->disp_vblank_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              mtk_disp_vblank_period_ns);
    fprintf(stderr,
            "mt6895: MTK-DISP-VBLANK: mutex frame IRQ model on"
            " (SPI 300/344, %" PRId64 " ns period)\n",
            mtk_disp_vblank_period_ns);
}

/*
 * DIAGNOSTIC (env MTK_PCSTALL=1): print the vCPU program counter once a
 * second.  A guest that hangs in an interrupt-disabled poll loop stops the
 * serial log with no backtrace; the last PC printed here identifies the
 * spinning function (match it against the kernel image / System.map).
 */
static QEMUTimer *mtk_pcstall_timer;
static int mtk_pcstall_prints;

static void mtk_pcstall_tick(void *opaque)
{
    CPUState *cs;

    (void)opaque;

    CPU_FOREACH(cs) {
        ARMCPU *cpu = ARM_CPU(cs);
        uint64_t pc = cpu->env.pc;
        uint64_t sp = cpu->env.xregs[31];
        uint8_t code[64];
        uint64_t stack[16];
        char hex[129];
        int i, nstack = 0;
        int ok;

        ok = cpu_memory_rw_debug(cs, pc, code, sizeof(code), false) == 0;
        cpu_memory_rw_debug(cs, sp, stack, sizeof(stack), false);
        for (i = 0; i < 16 && nstack < 4; i++) {
            if (stack[i] >= 0xffff800000000000ULL) {
                fprintf(stderr, "MTK-PCSTALL  stack[%d]=%#llx\n", i,
                        (unsigned long long)stack[i]);
                nstack++;
            }
        }
        if (ok) {
            for (i = 0; i < 64; i++) {
                static const char hx[] = "0123456789abcdef";

                hex[2 * i] = hx[code[i] >> 4];
                hex[2 * i + 1] = hx[code[i] & 0xf];
            }
            hex[128] = 0;
        } else {
            strcpy(hex, "unreadable");
        }
        fprintf(stderr,
                "MTK-PCSTALL cpu%ld pc=%#llx lr=%#llx sp=%#llx pstate=%#x "
                "code=%s\n",
                (long)cs->cpu_index, (unsigned long long)pc,
                (unsigned long long)cpu->env.xregs[30],
                (unsigned long long)sp,
                (unsigned)cpu->env.pstate, hex);
    }
    mtk_pcstall_prints++;
    if (mtk_pcstall_prints < 600) {
        timer_mod(mtk_pcstall_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 1000);
    }
}

static void mtk_pcstall_init(Mt6895MachineState *s)
{
    if (!getenv("MTK_PCSTALL")) {
        return;
    }
    mtk_pcstall_timer = timer_new_ms(QEMU_CLOCK_REALTIME, mtk_pcstall_tick, s);
    timer_mod(mtk_pcstall_timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 3000);
    fprintf(stderr, "mt6895: MTK_PCSTALL armed\n");
}

static void mtk_cmdline_tick(void *opaque)
{
    Mt6895MachineState *s = opaque;
    const char *spec = getenv("MTK_KCMDLINE_SUBST");
    const char *entry;
    char from[64], to[64];
    size_t flen, tlen;
    uint64_t win, off;
    uint8_t *base;
    int hits = 0;
    int64_t now = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);

    if (!spec || !*spec) {
        return;
    }
    /*
     * The spec is one or more "old:new" pairs separated by ';' so a single
     * run can patch several tokens (e.g. drm.debug and ignore_loglevel).
     */
    for (entry = spec; entry && *entry; ) {
        const char *colon = strchr(entry, ':');
        const char *semi = strchr(entry, ';');
        const char *next = semi ? semi + 1 : NULL;
        size_t elen;

        hits = 0;
        if (!colon || colon == entry) {
            return;
        }
        elen = semi ? (size_t)(semi - entry) : strlen(entry);
        flen = (size_t)(colon - entry);
        if (colon > entry + elen) {
            return;
        }
        tlen = elen - flen - 1;
        if (flen >= sizeof(from) || tlen > flen) {
            return;
        }
        memcpy(from, entry, flen);
        from[flen] = 0;
        memcpy(to, colon + 1, tlen);
        to[tlen] = 0;
        entry = next;

    win = 256 * MiB;
    if (win > MACHINE(s)->ram_size) {
        win = MACHINE(s)->ram_size;
    }
    base = memory_region_get_ram_ptr(s->ddr);
    {
        int pass;

        for (pass = 0; pass < 2; pass++) {
            uint64_t woff = 0;
            uint64_t wlen = win;

            if (pass == 1) {
                /*
                 * Second window: the mb_dtb / LK command-line copies live
                 * around 0x47c80000, i.e. far above the 256 MiB below.
                 * MTK_KCMDLINE_ADDR overrides the base address.
                 */
                const char *as = getenv("MTK_KCMDLINE_ADDR");
                uint64_t phys = as ? strtoull(as, NULL, 0)
                                   : 0x47c80000ULL;
                uint64_t wsize = 4 * MiB;

                if (phys < MT6895_DRAM_BASE ||
                    phys - MT6895_DRAM_BASE >
                        MACHINE(s)->ram_size - wsize) {
                    continue;
                }
                woff = phys - MT6895_DRAM_BASE;
                wlen = wsize;
            }
            /*
             * Scan every byte: LK writes the command line at unaligned
             * offsets, so a 4-byte stride can silently miss tokens (e.g.
             * 8250.nr_uarts=4 at an odd address).  Exact same-length
             * in-place overrides cannot corrupt other strings.
             */
            for (off = 0; off + flen <= wlen; off++) {
                uint8_t *p = base + woff + off;

                if (memcmp(p, from, flen)) {
                    continue;
                }
                memcpy(p, to, tlen);
                memset(p + tlen, ' ', flen - tlen);
                hits++;
            }
        }
    }
        if (hits && !mtk_cmdline_scans) {
            fprintf(stderr, "mt6895: MTK_KCMDLINE_SUBST: patched %d copy/copies \"%s\" -> \"%s\"\n",
                    hits, from, to);
        }
    } /* each "old:new" pair */
    mtk_cmdline_scans++;
    if (mtk_cmdline_scans >= 14) {
        return;
    }
    timer_mod(mtk_cmdline_timer, now + 5000);
}


/*
 * DIAGNOSTIC (env MTK_PCHISTDUMP=<ms>): dump the TB-start PC ring that
 * accel/tcg/cpu-exec.c records when MTK_PC_HIST is set.  Unlike PCSTALL
 * (which reads env->pc outside the TB loop and therefore samples epilogues),
 * this ring holds real translation-block entry PCs, so a hang in an
 * IRQ-disabled spin loop shows the loop's own PCs.
 */
static QEMUTimer *mtk_pchistdump_timer;
static int mtk_pchistdump_prints;

static void mtk_pchistdump_tick(void *opaque)
{
    uint64_t uniq[32];
    unsigned cnt[32] = { 0 };
    unsigned nu = 0, i, j;

    for (i = 0; i < 96; i++) {
        uint64_t pc = mtk_pc_hist[(mtk_pc_hist_idx + i) % 96];

        if (pc < 0xffff000000000000ULL) {
            continue;
        }
        for (j = 0; j < nu; j++) {
            if (uniq[j] == pc) {
                cnt[j]++;
                break;
            }
        }
        if (j == nu && nu < 32) {
            uniq[nu] = pc;
            cnt[nu] = 1;
            nu++;
        }
    }
    fprintf(stderr, "MTK-PCHIST-DUMP idx=%u uniq=%u", mtk_pc_hist_idx, nu);
    for (j = 0; j < nu; j++) {
        fprintf(stderr, " pc=%#llx x%u", (unsigned long long)uniq[j], cnt[j]);
    }
    fprintf(stderr, "\n");
    mtk_pchistdump_prints++;
    if (mtk_pchistdump_prints < 400) {
        timer_mod(mtk_pchistdump_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
                  (int64_t)strtoll(getenv("MTK_PCHISTDUMP"), NULL, 0));
    }
}

static void mtk_pchistdump_init(Mt6895MachineState *s)
{
    if (!getenv("MTK_PCHISTDUMP")) {
        return;
    }
    mtk_pchistdump_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                        mtk_pchistdump_tick, s);
    timer_mod(mtk_pchistdump_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 2000);
    fprintf(stderr, "mt6895: MTK_PCHISTDUMP armed\n");
}
static void mtk_cmdline_init(Mt6895MachineState *s)
{
    const char *spec = getenv("MTK_KCMDLINE_SUBST");

    if (!spec || !*spec) {
        return;
    }
    mtk_cmdline_timer = timer_new_ms(QEMU_CLOCK_REALTIME, mtk_cmdline_tick, s);
    timer_mod(mtk_cmdline_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 5000);
    fprintf(stderr, "mt6895: MTK_KCMDLINE_SUBST armed (%s)\n", spec);
}

/*
 * xHCI MMIO mapping is delayed by default: LK also owns the ssusb/xHCI block
 * during early boot (fastboot/udc init touches the same 0x11200000 window), so
 * the kernel-only sysbus xHCI is overlaid on top of the plain syscon window
 * only after LK has handed over to Linux.  MTK_XHCI_MAP_MS overrides the delay
 * in milliseconds; MTK_XHCI_MAP_MS=0 maps immediately (old behaviour, for
 * LK-side experiments); MTK_NO_XHCI=1 disables the device entirely.
 */
typedef struct MtkXhciMapState {
    DeviceState *dev;
    uint64_t base;
} MtkXhciMapState;

static void mtk_xhci_map_now(void *opaque)
{
    MtkXhciMapState *m = opaque;

    memory_region_add_subregion_overlap(get_system_memory(), m->base,
        sysbus_mmio_get_region(SYS_BUS_DEVICE(m->dev), 0), 1);
    fprintf(stderr, "mt6895: xHCI MMIO mapped at %#" PRIx64
            " (after LK, priority 1 over syscon)\n", m->base);
}

static void mtk_xhci_map_later(DeviceState *dev, uint64_t base,
                               int64_t delay_ms)
{
    MtkXhciMapState *m = g_new0(MtkXhciMapState, 1);
    QEMUTimer *t;

    m->dev = dev;
    m->base = base;
    t = timer_new_ms(QEMU_CLOCK_REALTIME, mtk_xhci_map_now, m);
    timer_mod(t, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + delay_ms);
}

/*
 * DISP mutex block (mt6895.dispsys_config + 0x1000, i.e. 0x14001000).
 *
 * The vendor DRM driver talks to the hardware mutex directly:
 *   mtk_disp_mutex_acquire(): write MUTEX_EN(n)=1 (0x20+0x20n), write
 *                             MUTEX(n)=1 (0x24+0x20n), then poll MUTEX(n)
 *                             until bit1 (HW-locked) is set, 10 ms budget;
 *   mtk_disp_mutex_release(): write MUTEX(n)=0.
 * A plain read/write syscon returns the written 1 in bit0 and never sets
 * bit1, so every acquire burns its 10 ms budget and the display commit path
 * stalls; recovery in particular ends up spinning on that timeout.  Model
 * the lock bit: a write of 1 to MUTEX(n) grants the mutex (bit1 reads back
 * set), a write of 0 releases it.  INTSTA (0x04) latches SOF/EOF and the
 * level line to the GIC follows INTEN (see the frame interrupt model above);
 * the driver also polls MUTEX(n) for the lock handshake.
 */
static uint64_t mt6895_disp_mutex_read_idx(Mt6895MachineState *s, int idx,
                                               hwaddr off, unsigned size)
{
    uint32_t v;

    if (off >= sizeof(s->disp_mutex_regs[0])) {
        return 0;
    }
    if (off < 0x20) {
        v = s->disp_mutex_regs[idx][off >> 2];
    } else {
        unsigned n = (off - 0x20) / 0x20;
        unsigned sub = (off - 0x20) % 0x20;

        if (sub == 0x04) {
            /* MUTEX(n): bit1 = hardware locked, bit0 mirrors the request. */
            v = (s->disp_mutex_locked[idx] & (1u << n)) ? 0x3u : 0x0u;
        } else {
            v = s->disp_mutex_regs[idx][off >> 2];
        }
    }
    return extract64(v, 0, size * 8);
}

static void mt6895_disp_mutex_write_idx(Mt6895MachineState *s, int idx,
                                        hwaddr off, uint64_t value)
{
    uint32_t val = (uint32_t)value;

    if (off >= sizeof(s->disp_mutex_regs[0])) {
        return;
    }
    if (off < 0x20) {
        if (off == MTK_DISP_MUTEX_INTSTA_OFF) {
            static int intsta_logs;
            uint32_t before = s->disp_mutex_regs[idx][off >> 2];
            uint32_t acked;

            /* INTSTA acknowledge: write 1 keeps a bit, write 0 clears it
             * (see the comment above mtk_disp_mutex_update_irq()). */
            s->disp_mutex_regs[idx][off >> 2] &= val;
            acked = before & ~s->disp_mutex_regs[idx][off >> 2];
            if (acked) {
                mtk_disp_vblank_acks++;
            }
            if (mtk_disp_vblank_log && intsta_logs < 8) {
                intsta_logs++;
                fprintf(stderr,
                        "MTK-DISP-MUTEX-IRQ: instance %d INTSTA write %#x"
                        " clears %#x -> %#x guest=%.3fs\n",
                        idx, val, acked,
                        s->disp_mutex_regs[idx][off >> 2],
                        mtk_guest_seconds());
            }
            mtk_disp_mutex_update_irq(s, idx);
        } else {
            uint32_t old = s->disp_mutex_regs[idx][off >> 2];

            s->disp_mutex_regs[idx][off >> 2] = val;
            if (off == MTK_DISP_MUTEX_INTEN_OFF) {
                static int inten_logs;

                /* An INTEN write must re-evaluate the line immediately. */
                if (mtk_disp_vblank_log && inten_logs < 16) {
                    if (val != old) {
                        inten_logs++;
                        fprintf(stderr,
                                "MTK-DISP-VBLANK: instance %d INTEN write"
                                " %#x -> %#x guest=%.3fs\n",
                                idx, old, val, mtk_guest_seconds());
                    }
                }
                mtk_disp_mutex_update_irq(s, idx);
            }
        }
        return;
    }
    {
        unsigned n = (off - 0x20) / 0x20;
        unsigned sub = (off - 0x20) % 0x20;

        s->disp_mutex_regs[idx][off >> 2] = val;
        if (n < 32u) {
            if (sub == 0x04) {
                if (val & 0x1u) {
                    s->disp_mutex_locked[idx] |= 1u << n;
                } else {
                    s->disp_mutex_locked[idx] &= ~(1u << n);
                }
            } else if (sub == 0x00 && !(val & 0x1u)) {
                /* Disabling the mutex also drops the lock. */
                s->disp_mutex_locked[idx] &= ~(1u << n);
            } else if (sub == 0x08 && (val & 0x1u)) {
                /* MUTEX_RST(n): reset drops the lock. */
                s->disp_mutex_locked[idx] &= ~(1u << n);
            }
        }
    }
}

static uint64_t mt6895_disp_mutex0_read(void *opaque, hwaddr off, unsigned size)
{
    return mt6895_disp_mutex_read_idx(opaque, 0, off, size);
}

static void mt6895_disp_mutex0_write(void *opaque, hwaddr off, uint64_t value,
                                     unsigned size)
{
    mt6895_disp_mutex_write_idx(opaque, 0, off, value);
}

static uint64_t mt6895_disp_mutex1_read(void *opaque, hwaddr off, unsigned size)
{
    return mt6895_disp_mutex_read_idx(opaque, 1, off, size);
}

static void mt6895_disp_mutex1_write(void *opaque, hwaddr off, uint64_t value,
                                     unsigned size)
{
    mt6895_disp_mutex_write_idx(opaque, 1, off, value);
}

static const MemoryRegionOps mt6895_disp_mutex0_ops = {
    .read = mt6895_disp_mutex0_read,
    .write = mt6895_disp_mutex0_write,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static const MemoryRegionOps mt6895_disp_mutex1_ops = {
    .read = mt6895_disp_mutex1_read,
    .write = mt6895_disp_mutex1_write,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/*
 * DSI0 interrupt block (0x14017000): INTEN 0x08 / INTSTA 0x0c.
 * The vendor DRM vblank handler (mtk_dsi_irq_status in mtk_dsi.c) reads
 * INTSTA, clears it with a write of ~status, and on FRAME_DONE_INT_FLAG
 * (BIT(4)) calls mtk_crtc_vblank_irq() -> drm_crtc_handle_vblank() and
 * drm_crtc_send_vblank_event(), i.e. the page-flip completion that minui's
 * DRM Flip() waits for in poll().
 */
static uint64_t mt6895_dsi_intr_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;

    if (off >= sizeof(s->dsi_regs)) {
        return 0;
    }
    return extract64(s->dsi_regs[off >> 2], 0, size * 8);
}

static void mt6895_dsi_intr_write(void *opaque, hwaddr off, uint64_t value,
                             unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint32_t val = (uint32_t)value;

    if (off >= sizeof(s->dsi_regs)) {
        return;
    }
    if (off == 0x0c) {
        /* INTSTA acknowledge: write 1 keeps, write 0 clears (the driver
         * writes ~status; upstream clears with mtk_dsi_mask(..., status, 0)). */
        s->dsi_regs[off >> 2] &= val;
        mtk_dsi_update_irq(s);
        return;
    }
    s->dsi_regs[off >> 2] = val;
    if (off == 0x08) {
        /* INTEN: re-evaluate the level line when FRAME_DONE is (un)masked. */
        mtk_dsi_update_irq(s);
    }
}

static const MemoryRegionOps mt6895_dsi_intr_ops = {
    .read = mt6895_dsi_intr_read,
    .write = mt6895_dsi_intr_write,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void mt6895_create_syscons(Mt6895MachineState *s, Error **errp)
{
    mtk_bl2pc_init();
    /*
     * Low-priority fill for unimplemented peripheral registers in the
     * 0x1000_0000 window. LK's monitor/debug_ctrl_ao drivers (taken from the
     * device tree) touch registers this model does not implement; an unmapped
     * write raises a data abort and LK dies. The real device models are added
     * later at priority 0, so they still take precedence.
     */
    memory_region_init_io(&s->peri_fill_mr, OBJECT(s), &mt6895_peri_fill_ops,
                          s, "mt6895.peri-fill", 0x10000000);
    memory_region_add_subregion_overlap(get_system_memory(), 0x10000000ULL,
                                        &s->peri_fill_mr, -1);

    mtk_usb_hostmode_init(s);
    mtk_usb_gadget_init(s);
    mtk_dtnoiommu_init(s);
    mtk_cmdline_init(s);
    mtk_pcstall_init(s);
    mtk_pchistdump_init(s);
    mtk_disp_vblank_init(s);
    /*
     * DSI0 interrupt line (GIC SPI 323, level-high).  It belongs to the DSI
     * controller model and not to the optional vblank models, so wire it
     * unconditionally; mtk_dsi_update_irq() drives it from (INTSTA & INTEN).
     */
    s->dsi_irq = qdev_get_gpio_in(s->gic, 323);
    mtk_smpdbg_init(s);
    mtk_kmem_init(s);

    /*
     * USB0 IPPC glue (hw/misc/mtk_usb_glue.c).  mtu3's ssusb_check_clocks()
     * polls IPPC+0x10 for the PHY clock-stable bits of the enabled U2/U3
     * ports and IPPC+0x14 bit0; without them the whole mtu3 probe fails with
     * "clks of sts1 are not stable!" / -110 (see
     * vendor module analysis).  The 0x100-byte DT
     * window (usb0 "ippc" reg) is overlaid on the generic syscon with
     * priority 1, so the clock gate registers around it keep their old plain
     * read/write behaviour.  MTK_NO_USBGLUE=1 keeps the plain syscon window.
     */
    if (!MTK_ENV_CACHED("MTK_NO_USBGLUE")) {
        DeviceState *ippc = qdev_new("mtk-usb-ippc");

        if (!sysbus_realize(SYS_BUS_DEVICE(ippc), errp)) {
            return;
        }
        memory_region_add_subregion_overlap(
            get_system_memory(), 0x11203e00ULL,
            sysbus_mmio_get_region(SYS_BUS_DEVICE(ippc), 0), 1);
    }
    /*
     * LK fastboot UDC (mtu3 device mode) minimal model.  Env MTK_USBDEV=1
     * creates it; default off so kernel/USB-host runs are unaffected.
     * Covers 0x11201000+0x2e00 (U3D + QMU + USB3 MAC CSR + USB2 CSR) with
     * priority 2 over the plain syscon/peri-fill.  IPPC glue at 0x11203e00 is
     * deliberately left untouched (it is what makes LK's clock check pass).
     * Host side talks over a chardev socket; env MTK_USBDEV_TCP=<port>
     * (default 4444) or MTK_USBDEV_CHR=<existing chardev id>.
     * IRQ: the guest FDT's usb0@11201000 node carries
     * "interrupts = <GIC_SPI 128 IRQ_TYPE_LEVEL_HIGH>" (SPI 128 / INTID 160),
     * which is the line both LK's fastboot UDC and the kernel mtu3 driver
     * request.  QEMU's GICv3 input lines are indexed by SPI number (see
     * mt6895.h), so one constant serves both paths.
     */
    {
        const char *ud = getenv("MTK_USBDEV");
        const char *gd = getenv("MTK_USB_GADGET");
        bool gadget_mode = gd && *gd && strcmp(gd, "0") != 0;

        if (gadget_mode || (ud && *ud && strcmp(ud, "0") != 0)) {
            Chardev *chr = NULL;
            const char *cid = getenv("MTK_USBDEV_CHR");

            if (cid && *cid) {
                chr = qemu_chr_find(cid);
                if (!chr) {
                    fprintf(stderr, "mt6895: MTK_USBDEV_CHR=%s not found\n",
                            cid);
                }
            }
            if (!chr) {
                const char *port = getenv("MTK_USBDEV_TCP");
                char spec[160];

                snprintf(spec, sizeof(spec),
                         "tcp:0.0.0.0:%s,server=on,wait=off",
                         (port && *port) ? port : "4444");
                chr = qemu_chr_new("mtk-usbdev", spec, NULL);
            }
            if (chr) {
                DeviceState *udev = qdev_new("mtk-usbdev");

                qdev_prop_set_chr(udev, "chardev", chr);
                if (!sysbus_realize(SYS_BUS_DEVICE(udev), errp)) {
                    return;
                }
                memory_region_add_subregion_overlap(get_system_memory(),
                    0x11201000ULL,
                    sysbus_mmio_get_region(SYS_BUS_DEVICE(udev), 0), 2);
                {
                    const char *ie = getenv("MTK_USBDEV_IRQ");
                    unsigned usb_irq = (ie && *ie) ? strtoul(ie, NULL, 0) :
                                       MT6895_SSUSB_SPI;

                    sysbus_connect_irq(SYS_BUS_DEVICE(udev), 0,
                                       qdev_get_gpio_in(s->gic, usb_irq));
                    fprintf(stderr, "mt6895: mtk-usbdev UDC @0x11201000, "
                            "IRQ intid%u/line%u, chardev=%s\n",
                            usb_irq + 32, usb_irq, chr->label);
                }
            }
        }
    }
    /*
     * ATF's platform tag area (GTA) at 0x4825ad1c is left as plain RAM by
     * default now that BL32/TEE really runs: ATF's secmem_init (0x4821e338)
     * copies the real TZ_TAGS from tee-secmem+0x200 into that address after
     * its BSS clear, so the tags ATF sees are the firmware's own.
     *
     * The old synthetic "SS\x12" tag (MTK_GTAPLANT=1 restores it) is wrong in
     * this configuration and is what hung the zero-injection boot: the tag
     * advertises a translation-table area at 0x48280000, which overlaps BL31's
     * own L3 tables, and its read hook re-plants the tag whenever the magic is
     * gone, overwriting the real TZ_TAGS.  With the tag in place, LK's CMDQ SMC
     * 0xc2000418 -> GZ-forwarded 0xc2004418 -> ATF cmdq_bl_handler (0x48207758)
     * -> mtk_iommu_init_secure_pt_mem (0x4821443c) memsets 0x48280000 for
     * 0x10000 bytes, data-aborts at 0x48280800 (translation fault, EL3), and
     * the nested instruction-abort loop at 0x4822b800 means LK never gets its
     * SMC return.  See DEBUG_NOTES round 1,285/1,286 and NEXT_STEPS 26/27.
     *
     * MTK_NO_GTAPLANT=1 is still accepted for older harnesses and now matches
     * the default.
     */
    if (MTK_ENV_CACHED("MTK_GTAPLANT") && !MTK_ENV_CACHED("MTK_NO_GTAPLANT")) {
        static MemoryRegion gta_mr;

        memory_region_init_io(&gta_mr, OBJECT(s), &mt6895_atf_gta_ops, s,
                              "mt6895.atf-gta", sizeof(mt6895_atf_gta_shadow));
        memory_region_add_subregion_overlap(get_system_memory(), 0x4825ad1cULL,
                                            &gta_mr, 1);
    }
    if (MTK_ENV_CACHED("MTK_TZFIX")) {
        memory_region_init_io(&mt6895_tzfix_mr, OBJECT(s),
                              &mt6895_tzfix_ops, s, "mt6895.tzfix",
                              MT6895_TZFIX_SIZE);
        memory_region_add_subregion_overlap(get_system_memory(),
                                            MT6895_TZFIX_BASE,
                                            &mt6895_tzfix_mr, 1);
    }
    if (MTK_ENV_CACHED("MTK_GTAWATCH")) {
        memory_region_init_io(&mt6895_gtawatch_mr, OBJECT(s),
                              &mt6895_gtawatch_ops, s, "mt6895.gtawatch",
                              MT6895_GTAWATCH_SIZE);
        memory_region_add_subregion_overlap(get_system_memory(),
                                            MT6895_GTAWATCH_BASE,
                                            &mt6895_gtawatch_mr, 1);
    }
    if (MTK_ENV_CACHED("MTK_GWATCH")) {
        memory_region_init_io(&mt6895_gwatch_mr, OBJECT(s),
                              &mt6895_gwatch_ops, s, "mt6895.gwatch",
                              MT6895_GWATCH_SIZE);
        memory_region_add_subregion_overlap(get_system_memory(),
                                            MT6895_GWATCH_BASE,
                                            &mt6895_gwatch_mr, 1);
    }
    if (MTK_ENV_CACHED("MTK_BTREAD")) {
        memory_region_init_io(&mt6895_btread_mr, OBJECT(s),
                              &mt6895_btread_ops, s, "mt6895.btread",
                              MT6895_BTREAD_SIZE);
        memory_region_add_subregion_overlap(get_system_memory(),
                                            MT6895_BTREAD_BASE,
                                            &mt6895_btread_mr, 1);
    }
    if (MTK_ENV_CACHED("MTK_PTWATCH")) {
        memory_region_init_io(&mt6895_ptwatch_mr, OBJECT(s),
                              &mt6895_ptwatch_ops, s, "mt6895.ptwatch", 0x1000);
        memory_region_add_subregion_overlap(get_system_memory(),
                                            MT6895_PTWATCH_BASE,
                                            &mt6895_ptwatch_mr, 1);
    }
    /*
     * DIAGNOSTIC (MTK_WATCH_ADDR / MTK_WATCH_SIZE): install the generic
     * read/write watch window at an arbitrary physical address.  Both accesses
     * are forwarded to the RAM/DRAM underneath, so guest behaviour is
     * unchanged; the write half logs (address, value, pc) so "who wrote this
     * byte" can be answered without guessing.  Used to find out what is
     * supposed to populate the AEE memory info at 0x11dec0.
     */
    if (MTK_ENV_CACHED("MTK_PCHIT")) {
        const char *e = getenv("MTK_PCHIT");
        const char *p = e;
        const char *ms = getenv("MTK_PCHIT_MS");
        int64_t when = ms ? strtoll(ms, NULL, 0) : 14000;
        QEMUTimer *t;

        mtk_pchit_n = 0;
        while (p && *p && mtk_pchit_n < 8) {
            char *end;
            uint64_t v = strtoull(p, &end, 0);

            mtk_pchit_tgt[mtk_pchit_n++] = v;
            p = (*end == ',') ? end + 1 : NULL;
        }
        for (unsigned i = 0; i < mtk_pchit_n; i++) {
            fprintf(stderr, "mt6895: PC hit counter %u = %#" PRIx64 "\n",
                    i, mtk_pchit_tgt[i]);
        }
        t = timer_new_ns(QEMU_CLOCK_VIRTUAL, mtk_pchit_dump, s);
        timer_mod(t, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  when * 1000000LL);
    }

    if (MTK_ENV_CACHED("MTK_WATCH_ADDR")) {
        static MemoryRegion any_mr;
        const char *sa = getenv("MTK_WATCH_ADDR");
        const char *ss = getenv("MTK_WATCH_SIZE");
        uint64_t wbase = strtoull(sa, NULL, 0);
        uint64_t wsize = ss ? strtoull(ss, NULL, 0) : 0x1000;

        if (wsize) {
            mt6895_watch_cur_base = wbase;
            memory_region_init_io(&any_mr, OBJECT(s), &mt6895_watch_ops, s,
                                  "mt6895.watch-any", wsize);
            memory_region_add_subregion_overlap(get_system_memory(), wbase,
                                                &any_mr, 4);
            fprintf(stderr, "mt6895: watch window %#" PRIx64 "..%#" PRIx64
                    " installed\n", wbase, wbase + wsize);
        }
    }
    if (MTK_ENV_CACHED("MTK_ATF_WATCH")) {
        memory_region_init_io(&mt6895_watch_mr, OBJECT(s), &mt6895_watch_ops,
                              s, "mt6895.watch", MT6895_WATCH_SIZE);
        memory_region_add_subregion_overlap(get_system_memory(),
                                            MT6895_WATCH_BASE,
                                            &mt6895_watch_mr, 1);
    }

    


static const struct {
        const char *name;
        uint64_t base;
        uint32_t size;
    } blocks[] = {
        /* topckgen_clk mapped as dedicated mt6895.topckgen_clk */
        { "infracfg_ao_clk", 0x10001000ULL, 0x1000 },
        { "apmixedsys_clk", 0x1000c000ULL, 0x1000 },
        { "nemi_bus", 0x10270000ULL, 0x1000 },
        { "semi_bus", 0x1030e000ULL, 0x1000 },
        { "hwv", 0x10320000ULL, 0x2000 },
        /*
         * The EMI cluster the DRAM/EMI bring-up walks: the fake engine's
         * register pair and the SLC block. Unmapped they abort the walk
         * (measured: DFAR 0x10342000, DFSR 0x8, ESR 0x9600003f -- a read).
         */
        { "mt6895.dev-emi-fake-eng", 0x1026c000ULL, 0x1000 },
        { "mt6895.dev-emi-fake-eng", 0x1026d000ULL, 0x1000 },
        { "mt6895.dev-emi-fake-eng", 0x10310000ULL, 0x1000 },
        { "mt6895.dev-emi-fake-eng", 0x10311000ULL, 0x1000 },
        { "mt6895.dev-emislb", 0x10342000ULL, 0x1000 },
        { "mt6895.dev-emislb", 0x10343000ULL, 0x1000 },
        { "pericfg_ao_clk", 0x11036000ULL, 0x1000 },
        { "ssusb_device_clk", 0x11201000ULL, 0x1000 },
        { "ssusb_sifslv_ippc_clk", 0x11203e00ULL, 0x1000 },
        { "ssusb_sifslv_ippc_p1_clk", 0x11213e00ULL, 0x1000 },
        { "imp_iic_wrap_c_clk", 0x11282000ULL, 0x1000 },
        { "ufs_ao_config_clk", 0x112b8000ULL, 0x1000 },
        { "ufs_pdn_cfg_clk", 0x112bc000ULL, 0x1000 },
        { "imp_iic_wrap_s_clk", 0x11d07000ULL, 0x1000 },
        { "imp_iic_wrap_w_clk", 0x11f41000ULL, 0x1000 },
        { "gpu_eb_rpc_bus_clk", 0x13f91000ULL, 0x1000 },
        { "mfgpll_pll_ctrl_clk", 0x13fa0000ULL, 0x400 },
        { "gpuebpll_pll_ctrl_clk", 0x13fa0800ULL, 0x400 },
        { "mfgscpll_pll_ctrl_clk", 0x13fa0c00ULL, 0x400 },
        { "mfg_top_config_clk", 0x13fbf000ULL, 0x1000 },
        { "mmsys0_config_clk", 0x14000000ULL, 0x1000 },
        { "mmsys1_config_clk", 0x14400000ULL, 0x1000 },
        { "imgsys_main_clk", 0x15000000ULL, 0x1000 },
        { "dip_top_dip1_clk", 0x15110000ULL, 0x1000 },
        { "dip_nr_dip1_clk", 0x15130000ULL, 0x1000 },
        { "wpe1_dip1_clk", 0x15220000ULL, 0x1000 },
        { "ipesys_clk", 0x15330000ULL, 0x1000 },
        { "wpe2_dip1_clk", 0x15520000ULL, 0x1000 },
        { "wpe3_dip1_clk", 0x15620000ULL, 0x1000 },
        { "vdec_soc_gcon_base_clk", 0x1600f000ULL, 0x1000 },
        { "vdec_gcon_base_clk", 0x1602f000ULL, 0x1000 },
        { "venc_gcon_clk", 0x17000000ULL, 0x1000 },
        { "venc_gcon_core1_clk", 0x17800000ULL, 0x1000 },
        { "apupll_pll_ctrl_clk", 0x190f3000ULL, 0x400 },
        { "npupll_pll_ctrl_clk", 0x190f3400ULL, 0x400 },
        { "apupll1_pll_ctrl_clk", 0x190f3800ULL, 0x400 },
        { "cam_main_r1a_clk", 0x1a000000ULL, 0x1000 },
        { "camsys_rawa_clk", 0x1a04f000ULL, 0x1000 },
        { "camsys_yuva_clk", 0x1a06f000ULL, 0x1000 },
        { "camsys_rawb_clk", 0x1a08f000ULL, 0x1000 },
        { "camsys_yuvb_clk", 0x1a0af000ULL, 0x1000 },
        { "camsys_rawc_clk", 0x1a0cf000ULL, 0x1000 },
        { "camsys_yuvc_clk", 0x1a0ef000ULL, 0x1000 },
        { "camsys_mraw_clk", 0x1a170000ULL, 0x1000 },
        { "ccu_main_clk", 0x1b200000ULL, 0x1000 },
        { "vlpcfg_bus_clk", 0x1c00c000ULL, 0x1000 },
        { "vlp_cksys_clk", 0x1c013000ULL, 0x1000 },
        { "afe_clk", 0x1e100000ULL, 0x1000 },
        { "mminfra_config_clk", 0x1e800000ULL, 0x1000 },
        { "mdpsys_config_clk", 0x1f000000ULL, 0x1000 },
        { "mdpsys1_config_clk", 0x1f800000ULL, 0x1000 },
        { "mt6895.wdt", 0x1c007000ULL, 0x1000 },
        { "mt6895.dvfsrc", MT6895_DVFSRC_BASE, MT6895_DVFSRC_SIZE },
    /*
         * The SPMI windows are not plain register files: the PMIF device
         * below owns them, because the channel handshake is what makes the
         * PMIC stack come up.
         */
        { "mt6895.dcm-infracfg-ao", MT6895_DCM_INFRACFG_AO, MT6895_DCM_WIN_SIZE },
        { "mt6895.dcm-infra-ao-bcrm", MT6895_DCM_INFRA_AO_BCRM, MT6895_DCM_WIN_SIZE },
        { "mt6895.dcm-infracfg-ao-mem", MT6895_DCM_INFRACFG_AO_MEM, MT6895_DCM_WIN_SIZE },
        { "mt6895.dcm-peri-ao-bcrm", MT6895_DCM_PERI_AO_BCRM, MT6895_DCM_WIN_SIZE },
        { "mt6895.dcm-vlp-ao-bcrm", MT6895_DCM_VLP_AO_BCRM, MT6895_DCM_WIN_SIZE },
        { "mt6895.mcusys-ao-cfg-lo", MT6895_MCUSYS_AO_CFG_LO_BASE, MT6895_MCUSYS_AO_CFG_LO_SIZE },
        { "mt6895.dcm-mp-cpusys-top", MT6895_DCM_MP_CPUSYS_TOP, MT6895_DCM_MP_CPUSYS_SIZE },
        { "mt6895.dcm-cpccfg-reg", MT6895_DCM_CPCCFG_REG, MT6895_DCM_WIN_SIZE },
        { "mt6895.mcusys-ao-cfg-hi", MT6895_MCUSYS_AO_CFG_HI_BASE, MT6895_MCUSYS_AO_CFG_HI_SIZE },
        { "mt6895.dev-mcupm-sram", MT6895_MCUPM_SRAM_BASE, MT6895_MCUPM_SRAM_SIZE },
        { "mt6895.dev-mcupm-cfg", MT6895_MCUPM_CORE_CFG_BASE, MT6895_MCUPM_CORE_CFG_SIZE },
        { "mt6895.dev-mcupm-log", MT6895_MCUPM_LOG_BASE, MT6895_MCUPM_LOG_SIZE },
        /* dramc@10230000's windows, generated from the device tree. */
        /* dramc-0..3 are mapped as dedicated mt6895.dramc-cmd regions */
        /* dramc-4..7 are mapped as dedicated mt6895.dramc-top2 regions */
        { "mt6895.dramc-8",  mt6895_dramc_wins[8].base,  mt6895_dramc_wins[8].size },
        { "mt6895.dramc-9",  mt6895_dramc_wins[9].base,  mt6895_dramc_wins[9].size },
        { "mt6895.dramc-10", mt6895_dramc_wins[10].base, mt6895_dramc_wins[10].size },
        { "mt6895.dramc-11", mt6895_dramc_wins[11].base, mt6895_dramc_wins[11].size },
        /* dramc-12..15 mapped as dedicated mt6895.dramc-top3 regions */
        { "mt6895.dramc-16", mt6895_dramc_wins[16].base, mt6895_dramc_wins[16].size },
        { "mt6895.dramc-17", mt6895_dramc_wins[17].base, mt6895_dramc_wins[17].size },
        { "mt6895.dramc-18", mt6895_dramc_wins[18].base, mt6895_dramc_wins[18].size },
        { "mt6895.dramc-19", mt6895_dramc_wins[19].base, mt6895_dramc_wins[19].size },
        { "mt6895.dramc-20", mt6895_dramc_wins[20].base, mt6895_dramc_wins[20].size },
        { "mt6895.dramc-21", mt6895_dramc_wins[21].base, mt6895_dramc_wins[21].size },
        { "mt6895.dramc-22", mt6895_dramc_wins[22].base, mt6895_dramc_wins[22].size },
        { "mt6895.dramc-23", mt6895_dramc_wins[23].base, mt6895_dramc_wins[23].size },
        { "mt6895.dramc-24", mt6895_dramc_wins[24].base, mt6895_dramc_wins[24].size },
        { "mt6895.dramc-25", mt6895_dramc_wins[25].base, mt6895_dramc_wins[25].size },
        { "mt6895.dramc-26", mt6895_dramc_wins[26].base, mt6895_dramc_wins[26].size },
        { "mt6895.dramc-27", mt6895_dramc_wins[27].base, mt6895_dramc_wins[27].size },
        /*
         * 0x0d040000 -- register base reached from the preloader's early
         * init at 0x02010970, which is called from the post-RAM_CONSOLE
         * bring-up sequence (0x02042550). That routine toggles bit 7 and
         * sets bit 16 on bus_dbg (0x10208000) and sys_cirq (0x10314000),
         * installs four address-range constants, then writes 0x6003 to
         * this base, 0x6003 to base+0x100 and 0x10000 to base+4. The
         * literal 0x0d0408fc appears twice in the image (0x020109e0 and
         * 0x020336b8), so it is a genuine peripheral base, not a stray
         * constant.
         *
         * Without a window here the store at 0x020109ba takes a
         * synchronous external abort (FSR 0x808). The abort vectors
         * through VBAR, which the preloader has not yet pointed at its own
         * table, so the CPU lands on the BootROM's AArch64 EL3 trampoline
         * at 0x00000010 and executes it as ARM32 -- the AArch64 word
         * 0xd51e4021 decodes as `ldrle r4, [lr, #-0x21]`, and with lr=0x18
         * that faults at 0xfffffff7 forever. The register file is stubbed
         * as a plain syscon: reads return what was last written.
         */
        { "mt6895.dev-0d040000", 0x0d040000ULL, 0x1000 },
        /*
         * The twelve HRE (hardware reset engine) latch blocks.
         *
         * 0x0200dbd6 walks a table at 0x02069c64 -- twelve {base, name}
         * pairs -- reading base+0x20c (latch3) and base+0x214 (latch5)
         * from each to build the watchdog flag word it prints as
         * "[AOC] HRE wdt_flags". The table supplies the instance names
         * used below, so these windows are not guesses. None of the
         * twelve appears in the extracted device-tree map; 0x10320000
         * (hwv) and 0x10342000/0x10343000 (emislb) are neighbours, not
         * these. The first entry, INFRA_BUS at 0x1032c000, is also reached
         * directly by the early init at 0x02033684, which writes 1 to
         * +0xd0, polls +0xd0 until it reads 1, polls +0x1ec until it
         * reads 0, then writes 0 to +0xd0.
         */
        { "mt6895.dev-infra-bus",     0x1032c000ULL, 0x1000 },
        /* The three DEVAPC windows are modelled separately, below. */
        { "mt6895.dev-north-emi",     0x10344000ULL, 0x1000 },
        { "mt6895.dev-north-emi-mpu", 0x10346000ULL, 0x1000 },
        { "mt6895.dev-north-slc",     0x10348000ULL, 0x1000 },
        { "mt6895.dev-south-emi",     0x10345000ULL, 0x1000 },
        { "mt6895.dev-south-emi-mpu", 0x10347000ULL, 0x1000 },
        { "mt6895.dev-south-slc",     0x10349000ULL, 0x1000 },
        { "mt6895.dev-north-smpu",    0x1034a000ULL, 0x1000 },
        { "mt6895.dev-south-smpu",    0x1034b000ULL, 0x1000 },
        { "mt6895.pinctrl", 0x10005000ULL, 0x1000 },
        /* EXPERIMENT (2026-11-03): the GZ RPMB error path prints
         * "[%s] gz insufficient RPMB size" with the %s argument = 0x20000;
         * on hardware that low SRAM window exists, here it was unmapped. */
        { "mt6895.dev-sram2", 0x00020000ULL, 0x1000 },
        /* EXPERIMENT (2026-11-03): boot-to-LK path reads 0x11E40318;
         * the window was unmapped and the read aborted. */
        { "mt6895.dev-iic-e", 0x11e40000ULL, 0x1000 },
        /*
         * The five EINT register banks are not plain register files: the
         * mtk-eint device below owns them because the interrupt status
         * registers must raise the shared GIC line (SPI 259).
         */
        { "mt6895.dev-apmixedsys", 0x08000000ULL, 0x1000 },
    /*
         * Register frames of the remaining device-tree peripherals, generated
         * mechanically.  Rules: label optional, indentation free, addresses
         * case-insensitive, and all 'reg' entries of every node are taken.
         * BootROM/SRAM ranges, ranges the machine already maps and regions with
         * dedicated models are excluded.  These blocks are plain register files
         * with no behaviour, but they must be mapped: drivers probe them, and an
         * unmapped access raises a synchronous external abort and panics.
         */
        { "mt6895.dev-infracfg_ao_mem", 0x10002000ULL, 0x1000 },
        { "mt6895.dev-pericfg", 0x10003000ULL, 0x1000 },
        { "mt6895.dev-scp_infra", 0x10006000ULL, 0x1000 },
        { "mt6895.dev-sej", 0x1000a000ULL, 0x1000 },
        { "mt6895.dev-drm", 0x1000d000ULL, 0x1000 },
        { "mt6895.dev-devapc_ao_infra_peri", 0x1000e000ULL, 0x1000 },
        /* srclken-rc@1c00d000 is mapped as dedicated mt6895.srclken_rc region */
        { "mt6895.dev-sleep_reg_md", 0x1000f000ULL, 0x1000 },
        { "mt6895.dev-topmisc", 0x10011000ULL, 0x1000 },
        { "mt6895.dev-mbist_ao", 0x10013000ULL, 0x1000 },
        { "mt6895.dev-dpmaif", 0x10014000ULL, 0x1000 },
        { "mt6895.dev-apcldmaout_ao", 0x10014400ULL, 0x400 },
        { "mt6895.dev-apcldmamisc_ao", 0x10014800ULL, 0x400 },
        { "mt6895.dev-apcldmamisc_ao", 0x10014c00ULL, 0x400 },
        { "mt6895.dev-devapc_mpu_ao", 0x10015000ULL, 0x1000 },
        { "mt6895.dev-aes_top0", 0x10016000ULL, 0x1000 },
        { "mt6895.dev-modem_temp_share", 0x10018000ULL, 0x1000 },
        { "mt6895.dev-devapc_ao_md", 0x10019000ULL, 0x1000 },
        { "mt6895.dev-topckgen_ao", 0x1001b000ULL, 0x1000 },
        { "mt6895.dev-devapc_ao_mm", 0x1001c000ULL, 0x1000 },
        { "mt6895.dev-sleep_sram", 0x1001e000ULL, 0x4000 },
        { "mt6895.dev-sleep_sram", 0x1001f000ULL, 0x1000 },
        { "mt6895.dev-sleep_sram", 0x10020000ULL, 0x1000 },
        { "mt6895.dev-sleep_sram", 0x10021000ULL, 0x1000 },
        { "mt6895.dev-devapc_ao_infra_peri", 0x10022000ULL, 0x1000 },
        { "mt6895.dev-devapc_ao_infra_peri", 0x10023000ULL, 0x1000 },
        { "mt6895.dev-devapc_ao_infra_peri", 0x10024000ULL, 0x1000 },
        { "mt6895.dev-devapc_ao_infra_peri", 0x10025000ULL, 0x1000 },
        { "mt6895.dev-devapc", 0x10030000ULL, 0x1000 },
        /* MT6895-FIX: 0x02010ABE reads/writes 0x10034F00, which fell in the
         * gap between this block and 0x10040000 and aborted 7.6M times. */
        { "mt6895.dev-infra_sys", 0x10032000ULL, 0x4000 },
        { "mt6895.dev-devapc", 0x10033000ULL, 0x1000 },
        { "mt6895.dev-mcucfg", 0x10200000ULL, 0x1000 },
        { "mt6895.dev-mcucfg", 0x10201000ULL, 0x1000 },
        { "mt6895.dev-mcucfg", 0x10202000ULL, 0x1000 },
        { "mt6895.dev-mcucfg", 0x10203000ULL, 0x1000 },
        { "mt6895.dev-sys_cirq", 0x10204000ULL, 0x1000 },
        { "mt6895.dev-devapc", 0x10207000ULL, 0x1000 },
        { "mt6895.dev-bus_dbg", 0x10208000ULL, 0x1000 },
        { "mt6895.dev-ccifdriver", 0x10209000ULL, 0x1000 },
        { "mt6895.dev-ccifdriver", 0x1020a000ULL, 0x1000 },
        { "mt6895.dev-ap_ccif1", 0x1020b000ULL, 0x1000 },
        { "mt6895.dev-md_ccif1", 0x1020c000ULL, 0x1000 },
        { "mt6895.dev-infra_mbist", 0x1020d000ULL, 0x1000 },
        { "mt6895.dev-devapc", 0x1020e000ULL, 0x1000 },
        { "mt6895.dev-trng", 0x1020f000ULL, 0x1000 },
        /* dxcc_sec@10210000 is mapped as dedicated mt6895.dxcc_sec region */
        { "mt6895.dev-md2md_md1_ccif0", 0x10211000ULL, 0x1000 },
        { "mt6895.dev-cq_dma", 0x10212000ULL, 0x1000 },
        { "mt6895.dev-cq_dma", 0x10212100ULL, 0x1000 },
        { "mt6895.dev-cq_dma", 0x10212200ULL, 0x1000 },
        { "mt6895.dev-cq_dma", 0x10212300ULL, 0x1000 },
        { "mt6895.dev-md2md_md2_ccif0", 0x10213000ULL, 0x1000 },
        { "mt6895.dev-sramrom", 0x10214000ULL, 0x1000 },
        { "mt6895.dev-infra_bcrm", 0x10215000ULL, 0x1000 },
        { "mt6895.dev-sub_infra_bcrm", 0x10216000ULL, 0x1000 },
        { "mt6895.dev-dbg_tracker2", 0x10218000ULL, 0x1000 },
        { "mt6895.dev-emicen", 0x10219000ULL, 0x1000 },
        { "mt6895.dev-infra_device_mpu", 0x1021a000ULL, 0x1000 },
        { "mt6895.dev-infra_device_mpu", 0x1021b000ULL, 0x1000 },
        { "mt6895.dev-apcldmaout", 0x1021b400ULL, 0x400 },
        { "mt6895.dev-apcldmamisc", 0x1021b800ULL, 0x400 },
        { "mt6895.dev-apcldmamisc", 0x1021bc00ULL, 0x400 },
        { "mt6895.dev-infracfg_mem", 0x1021c000ULL, 0x1000 },
        { "mt6895.dev-infra_device_mpu", 0x1021d000ULL, 0x1000 },
        { "mt6895.dev-infra_device_mpu", 0x1021e000ULL, 0x1000 },
        { "mt6895.dev-apcldmain", 0x1021f000ULL, 0x1000 },
        { "mt6895.dev-apdma", 0x10220000ULL, 0x4000 },
        { "mt6895.dev-bpi_bsi_slv2", 0x10225000ULL, 0x1000 },
        { "mt6895.dev-emimpu", 0x10226000ULL, 0x1000 },
        { "mt6895.dev-dpmaif", 0x1022c000ULL, 0x1000 },
        { "mt6895.dev-dpmaif", 0x1022d000ULL, 0x1000 },
        { "mt6895.dev-dpmaif", 0x1022e000ULL, 0x1000 },
        { "mt6895.dev-infra_dpmaif", 0x1022f000ULL, 0x1000 },
        /* dramc and emichn 4-channel holes from mt6895_dramc_wins */
        { "mt6895.dev-dramc-ch0-top1", 0x10232000ULL, 0x2000 },
        { "mt6895.dev-emichn0",        0x10235000ULL, 0x1000 },
        { "mt6895.dev-dramc-ch0-rsv",  0x10237000ULL, 0x1000 },
        { "mt6895.dev-dramc-ch0-rsv2", 0x1023a000ULL, 0x2000 },
        { "mt6895.dev-dramc-ch1-top1", 0x10242000ULL, 0x2000 },
        { "mt6895.dev-emichn1",        0x10245000ULL, 0x1000 },
        { "mt6895.dev-dramc-ch1-rsv",  0x10247000ULL, 0x1000 },
        { "mt6895.dev-dramc-ch2-top1", 0x10252000ULL, 0x2000 },
        { "mt6895.dev-emichn2",        0x10255000ULL, 0x1000 },
        { "mt6895.dev-dramc-ch2-rsv",  0x10257000ULL, 0x1000 },
        { "mt6895.dev-dramc-ch3-top1", 0x10262000ULL, 0x2000 },
        { "mt6895.dev-emichn3",        0x10265000ULL, 0x1000 },
        { "mt6895.dev-dramc-ch3-rsv",  0x10267000ULL, 0x1000 },
        { "mt6895.dev-ap_ccif2", 0x1023c000ULL, 0x1000 },
        { "mt6895.dev-md_ccif2", 0x1023d000ULL, 0x1000 },
        { "mt6895.dev-ap_ccif3", 0x1023e000ULL, 0x1000 },
        { "mt6895.dev-md_ccif3", 0x1023f000ULL, 0x1000 },
        { "mt6895.dev-ap_ccif4", 0x1024c000ULL, 0x1000 },
        { "mt6895.dev-md_ccif4", 0x1024d000ULL, 0x1000 },
        { "mt6895.dev-ap_ccif5", 0x1025c000ULL, 0x1000 },
        { "mt6895.dev-md_ccif5", 0x1025d000ULL, 0x1000 },
        { "mt6895.dev-mm_vpu_m0_sub_common", 0x1025e000ULL, 0x1000 },
        { "mt6895.dev-mm_vpu_m1_sub_common", 0x1025f000ULL, 0x1000 },
        { "mt6895.dev-devapc", 0x10274000ULL, 0x1000 },
        { "mt6895.dev-mm_vpu_m0_sub_common", 0x10309000ULL, 0x1000 },
        { "mt6895.dev-mm_vpu_m1_sub_common", 0x1030a000ULL, 0x1000 },
        { "mt6895.dev-mm_vpu_m1_sub_common", 0x1030b000ULL, 0x1000 },
        { "mt6895.dev-mm_vpu_m1_sub_common", 0x1030c000ULL, 0x1000 },
        { "mt6895.dev-mm_vpu_m1_sub_common", 0x1030d000ULL, 0x1000 },
        { "mt6895.dev-sys_cirq", 0x10312000ULL, 0x1000 },
        { "mt6895.dev-sys_cirq", 0x10313000ULL, 0x1000 },
        { "mt6895.dev-sys_cirq", 0x10314000ULL, 0x1000 },
        { "mt6895.dev-peri_ptp_therm", 0x10315000ULL, 0x1000 },
        { "mt6895.dev-peri_ptp_therm", 0x10316000ULL, 0x1000 },
        { "mt6895.dev-sys_cirq", 0x10350000ULL, 0x1000 },
        { "mt6895.dev-sys_cirq", 0x10351000ULL, 0x1000 },
        { "mt6895.dev-sys_cirq", 0x10352000ULL, 0x1000 },
        { "mt6895.dev-sys_cirq", 0x10354000ULL, 0x1000 },
        { "mt6895.dev-sys_cirq", 0x10355000ULL, 0x1000 },
        { "mt6895.dev-sys_cirq", 0x10356000ULL, 0x1000 },
        { "mt6895.dev-pwm", 0x11008000ULL, 0x1000 },
        { "mt6895.dev-btif", 0x1100c000ULL, 0x100 },
        { "mt6895.dev-disp_pwm0", 0x1100e000ULL, 0x1000 },
        { "mt6895.dev-spi0", 0x11010000ULL, 0x100 },
        { "mt6895.dev-spi1", 0x11011000ULL, 0x100 },
        { "mt6895.dev-spi2", 0x11012000ULL, 0x100 },
        { "mt6895.dev-spi3", 0x11013000ULL, 0x100 },
        { "mt6895.dev-spi4", 0x11014000ULL, 0x100 },
        { "mt6895.dev-spi5", 0x11015000ULL, 0x100 },
        { "mt6895.dev-spi6", 0x11016000ULL, 0x100 },
        { "mt6895.dev-spi7", 0x11017000ULL, 0x100 },
        /* MT6895-FIX: 0x02010AC6 touches r3+0x600 where r3=0x1103C900, i.e.
         * 0x1103CF00 -- also unmapped before this. */
        { "mt6895.dev-peri_bus", 0x1103c000ULL, 0x1000 },
        { "mt6895.dev-devapc", 0x11020000ULL, 0x1000 },
        { "mt6895.dev-xhci0", 0x11200000ULL, 0x1000 },
        { "mt6895.dev-xhci1", 0x11210000ULL, 0x1000 },
        { "mt6895.dev-usb1", 0x11211000ULL, 0x2e00 },
        /* EXPERIMENT (2026-11-03): ATF BL2 (BL2_EXT) programs the UFS
         * M-PHY at 0x112a1000/0x112a8000; unmapped reads aborted. */
        { "mt6895.dev-ufs-mphy", 0x112a0000ULL, 0x10000 },
        /* ufshci@112b0000 is its own device -- see below. */
        { "mt6895.dev-btif", 0x11300b00ULL, 0x80 },
        { "mt6895.dev-btif", 0x11300b80ULL, 0x80 },
        { "mt6895.dev-consys", 0x11c30000ULL, 0xa14 },
        { "mt6895.dev-seninf_top", 0x11c80000ULL, 0x20000 },
        { "mt6895.dev-consys", 0x11ea0000ULL, 0xa04 },
        { "mt6895.dev-efuse", 0x11f10000ULL, 0x10000 },
        { "mt6895.dev-mipi_tx_config", 0x11f70000ULL, 0x1000 },
        { "mt6895.dev-mali", 0x13000000ULL, 0x1000 },
        { "mt6895.dev-g3d_brisket", 0x13bfee00ULL, 0x200 },
        { "mt6895.dev-g3d_mpu", 0x13bff000ULL, 0x1000 },
        { "mt6895.dev-g3d_gpueb", 0x13c40000ULL, 0x20000 },
        { "mt6895.dev-gpueb", 0x13c5fd80ULL, 0x280 },
        { "mt6895.dev-g3d_gpueb", 0x13c60200ULL, 0x100 },
        { "mt6895.dev-g3d_gpueb", 0x13c60300ULL, 0x100 },
        { "mt6895.dev-g3d_gpueb", 0x13c60400ULL, 0x200 },
        { "mt6895.dev-g3d_gpueb", 0x13c60800ULL, 0x800 },
        { "mt6895.dev-g3d_gpueb", 0x13c62000ULL, 0x100 },
        { "mt6895.dev-gpueb", 0x13c62004ULL, 0x1000 },
        { "mt6895.dev-gpueb", 0x13c62074ULL, 0x1000 },
        { "mt6895.dev-gpueb", 0x13c62078ULL, 0x1000 },
        { "mt6895.dev-dfd", 0x13d00000ULL, 0x100000 },
        { "mt6895.dev-dfd", 0x13e00000ULL, 0x100000 },
        { "mt6895.dev-g3d_config", 0x13f90000ULL, 0x10000 },
        { "mt6895.dev-g3d_config", 0x13fa0400ULL, 0x400 },
        { "mt6895.dev-g3d_config", 0x13fb6000ULL, 0x1000 },
        { "mt6895.dev-g3d_config", 0x13fb7000ULL, 0x1000 },
        { "mt6895.dev-g3d_config", 0x13fb9c00ULL, 0x100 },
        { "mt6895.dev-g3d_config", 0x13fbb000ULL, 0x1000 },
        { "mt6895.dev-g3d_dvfs", 0x13fbc000ULL, 0x1000 },
        { "mt6895.dev-g3d_config", 0x13fbd000ULL, 0x1000 },
        { "mt6895.dev-g3d_config", 0x13fca000ULL, 0x1000 },
        { "mt6895.dev-g3d_config", 0x13fcb000ULL, 0x1000 },
        { "mt6895.dev-g3d_config", 0x13fcc000ULL, 0x1000 },
        { "mt6895.dev-g3d_config", 0x13fcd000ULL, 0x1000 },
        { "mt6895.dev-g3d_config", 0x13fce000ULL, 0x1000 },
        { "mt6895.dev-g3d_config", 0x13fcf000ULL, 0x1000 },
        { "mt6895.dev-g3d_config", 0x13fd9000ULL, 0x1000 },
        { "mt6895.dev-g3d_config", 0x13fda000ULL, 0x1000 },
        { "mt6895.dev-g3d_config", 0x13fdd000ULL, 0x1000 },
        { "mt6895.dev-g3d_config", 0x13fde000ULL, 0x1000 },
        { "mt6895.dev-g3d_config", 0x13fdf000ULL, 0x1000 },
        { "mt6895.dev-disp_mutex", 0x14001000ULL, 0x1000 },
        { "mt6895.dev-disp_ovl", 0x14002000ULL, 0x1000 },
        { "mt6895.dev-disp_ovl", 0x14003000ULL, 0x1000 },
        { "mt6895.dev-disp_ovl", 0x14004000ULL, 0x1000 },
        { "mt6895.dev-disp_rsz0", 0x14005000ULL, 0x1000 },
        { "mt6895.dev-disp_rdma", 0x14006000ULL, 0x1000 },
        { "mt6895.dev-disp_tdshp", 0x14007000ULL, 0x1000 },
        { "mt6895.dev-disp_c3d", 0x14008000ULL, 0x1000 },
        { "mt6895.dev-disp_color", 0x14009000ULL, 0x1000 },
        { "mt6895.dev-disp_ccorr", 0x1400a000ULL, 0x1000 },
        { "mt6895.dev-disp_ccorr", 0x1400b000ULL, 0x1000 },
        { "mt6895.dev-disp_mdp_aal", 0x1400c000ULL, 0x1000 },
        { "mt6895.dev-disp_aal", 0x1400d000ULL, 0x1000 },
        { "mt6895.dev-disp_gamma", 0x1400e000ULL, 0x1000 },
        { "mt6895.dev-disp_postmask0", 0x1400f000ULL, 0x1000 },
        { "mt6895.dev-disp_dither", 0x14010000ULL, 0x1000 },
        { "mt6895.dev-disp_chist", 0x14011000ULL, 0x1000 },
        { "mt6895.dev-disp_chist", 0x14012000ULL, 0x1000 },
        { "mt6895.dev-disp_cm", 0x14013000ULL, 0x1000 },
        { "mt6895.dev-disp_spr", 0x14014000ULL, 0x1000 },
        { "mt6895.dev-disp_dsc_wrap", 0x14015000ULL, 0x1000 },
        { "mt6895.dev-disp_merge", 0x14016000ULL, 0x1000 },
        { "mt6895.dev-dsi", 0x14017000ULL, 0x1000 },
        { "mt6895.dev-disp_wdma0", 0x14018000ULL, 0x1000 },
        { "mt6895.dev-disp_rdma", 0x1401c000ULL, 0x1000 },
        { "mt6895.dev-disp_wdma1", 0x1401f000ULL, 0x1000 },
        { "mt6895.dev-inlinerot", 0x14020000ULL, 0x1000 },
        { "mt6895.dev-smi_larb0", 0x14021000ULL, 0x1000 },
        { "mt6895.dev-smi_larb1", 0x14022000ULL, 0x1000 },
        { "mt6895.dev-smi_sub_comm0", 0x14023000ULL, 0x1000 },
        { "mt6895.dev-vcu", 0x14106000ULL, 0x1000 },
        { "mt6895.dev-disp_mutex", 0x14401000ULL, 0x1000 },
        { "mt6895.dev-disp_ovl", 0x14402000ULL, 0x1000 },
        { "mt6895.dev-disp_ovl", 0x14403000ULL, 0x1000 },
        { "mt6895.dev-disp_ovl", 0x14404000ULL, 0x1000 },
        { "mt6895.dev-disp_rsz0", 0x14405000ULL, 0x1000 },
        { "mt6895.dev-disp_rdma", 0x14406000ULL, 0x1000 },
        { "mt6895.dev-disp_tdshp", 0x14407000ULL, 0x1000 },
        { "mt6895.dev-disp_c3d", 0x14408000ULL, 0x1000 },
        { "mt6895.dev-disp_color", 0x14409000ULL, 0x1000 },
        { "mt6895.dev-disp_ccorr", 0x1440a000ULL, 0x1000 },
        { "mt6895.dev-disp_ccorr", 0x1440b000ULL, 0x1000 },
        { "mt6895.dev-disp_mdp_aal", 0x1440c000ULL, 0x1000 },
        { "mt6895.dev-disp_aal", 0x1440d000ULL, 0x1000 },
        { "mt6895.dev-disp_gamma", 0x1440e000ULL, 0x1000 },
        { "mt6895.dev-disp_postmask0", 0x1440f000ULL, 0x1000 },
        { "mt6895.dev-disp_dither", 0x14410000ULL, 0x1000 },
        { "mt6895.dev-disp_chist", 0x14411000ULL, 0x1000 },
        { "mt6895.dev-disp_chist", 0x14412000ULL, 0x1000 },
        { "mt6895.dev-disp_cm", 0x14413000ULL, 0x1000 },
        { "mt6895.dev-disp_spr", 0x14414000ULL, 0x1000 },
        { "mt6895.dev-disp_dsc1_wrap", 0x14415000ULL, 0x1000 },
        { "mt6895.dev-disp_merge", 0x14416000ULL, 0x1000 },
        { "mt6895.dev-disp_wdma0", 0x14418000ULL, 0x1000 },
        { "mt6895.dev-disp_ufbc_wdma0", 0x14419000ULL, 0x1000 },
        { "mt6895.dev-disp_rdma", 0x1441c000ULL, 0x1000 },
        { "mt6895.dev-disp_dp_intf0", 0x1441d000ULL, 0x1000 },
        { "mt6895.dev-disp_wdma1", 0x1441f000ULL, 0x1000 },
        { "mt6895.dev-inlinerot", 0x14420000ULL, 0x1000 },
        { "mt6895.dev-smi_larb20", 0x14421000ULL, 0x1000 },
        { "mt6895.dev-smi_larb21", 0x14422000ULL, 0x1000 },
        { "mt6895.dev-smi_sub_comm1", 0x14423000ULL, 0x1000 },
        { "mt6895.dev-dp_tx", 0x14800000ULL, 0x1000 },
        { "mt6895.dev-smi_larb9", 0x15001000ULL, 0x1000 },
        { "mt6895.dev-smi_img_sub_comm0", 0x15002000ULL, 0x1000 },
        { "mt6895.dev-smi_img_sub_comm1", 0x15003000ULL, 0x1000 },
        { "mt6895.dev-smi_larb18", 0x15004000ULL, 0x1000 },
        { "mt6895.dev-imgsys_fw", 0x15005000ULL, 0x1500 },
        { "mt6895.dev-imgsys_fw", 0x15007000ULL, 0x1500 },
        { "mt6895.dev-imgsys_fw", 0x15020000ULL, 0x10000 },
        { "mt6895.dev-imgsys_fw", 0x15040000ULL, 0x10000 },
        { "mt6895.dev-imgsys_fw", 0x15100000ULL, 0x10000 },
        { "mt6895.dev-smi_larb10", 0x15120000ULL, 0x1000 },
        { "mt6895.dev-smi_larb15", 0x15140000ULL, 0x1000 },
        { "mt6895.dev-imgsys_fw", 0x15150000ULL, 0x10000 },
        { "mt6895.dev-imgsys_fw", 0x15200000ULL, 0x10000 },
        { "mt6895.dev-imgsys_fw", 0x15210000ULL, 0x10000 },
        { "mt6895.dev-smi_larb11", 0x15230000ULL, 0x1000 },
        { "mt6895.dev-dvs", 0x15300000ULL, 0x1000 },
        { "mt6895.dev-dvp", 0x15300800ULL, 0x1000 },
        { "mt6895.dev-aie", 0x15310000ULL, 0x1000 },
        { "mt6895.dev-imgsys_fw", 0x15320000ULL, 0x10000 },
        { "mt6895.dev-smi_larb12", 0x15340000ULL, 0x1000 },
        { "mt6895.dev-imgsys_fw", 0x15500000ULL, 0x10000 },
        { "mt6895.dev-imgsys_fw", 0x15510000ULL, 0x10000 },
        { "mt6895.dev-smi_larb22", 0x15530000ULL, 0x1000 },
        { "mt6895.dev-imgsys_fw", 0x15600000ULL, 0x10000 },
        { "mt6895.dev-smi_larb23", 0x15630000ULL, 0x1000 },
        { "mt6895.dev-imgsys_fw", 0x15640000ULL, 0x10000 },
        { "mt6895.dev-vcu", 0x16000000ULL, 0x40000 },
        { "mt6895.dev-vdec", 0x16004000ULL, 0x1000 },
        { "mt6895.dev-vdec_fmt", 0x16005000ULL, 0x1000 },
        { "mt6895.dev-vdec_fmt", 0x16006000ULL, 0x1000 },
        { "mt6895.dev-vdec_fmt", 0x16007000ULL, 0x1000 },
        { "mt6895.dev-vdec_fmt", 0x16008000ULL, 0x1000 },
        { "mt6895.dev-smi_larb5", 0x1600d000ULL, 0x1000 },
        { "mt6895.dev-vdec", 0x16010000ULL, 0x1000 },
        { "mt6895.dev-vdec", 0x16011000ULL, 0x400 },
        { "mt6895.dev-vdec", 0x16020000ULL, 0x1000 },
        { "mt6895.dev-vdec", 0x16021000ULL, 0x1000 },
        { "mt6895.dev-vdec", 0x16023000ULL, 0x1000 },
        { "mt6895.dev-vdec", 0x16025000ULL, 0x4000 },
        { "mt6895.dev-smi_larb4", 0x1602e000ULL, 0x1000 },
        { "mt6895.dev-smi_larb7", 0x17010000ULL, 0x10000 },
        { "mt6895.dev-vcu", 0x17020000ULL, 0x10000 },
        { "mt6895.dev-jpgenc", 0x17030000ULL, 0x10000 },
        { "mt6895.dev-jpgdec0", 0x17040000ULL, 0x10000 },
        { "mt6895.dev-jpgdec0", 0x17050000ULL, 0x10000 },
        { "mt6895.dev-mbist", 0x17060000ULL, 0x10000 },
        { "mt6895.dev-smi_larb8", 0x17810000ULL, 0x10000 },
        { "mt6895.dev-vcu", 0x17820000ULL, 0x10000 },
        { "mt6895.dev-jpgenc", 0x17830000ULL, 0x10000 },
        { "mt6895.dev-mbist", 0x17860000ULL, 0x10000 },
        { "mt6895.dev-gps", 0x18000000ULL, 0x100000 },
        { "mt6895.dev-consys", 0x18001000ULL, 0x658 },
        { "mt6895.dev-consys", 0x18003000ULL, 0x204 },
        { "mt6895.dev-consys", 0x1800e000ULL, 0x124 },
        { "mt6895.dev-consys", 0x18011000ULL, 0x138 },
        { "mt6895.dev-consys", 0x18012000ULL, 0x98 },
        { "mt6895.dev-consys", 0x18020000ULL, 0x4c },
        { "mt6895.dev-consys", 0x18023000ULL, 0xe28 },
        { "mt6895.dev-consys", 0x1803b000ULL, 0x18 },
        { "mt6895.dev-consys", 0x18040000ULL, 0x2c },
        { "mt6895.dev-consys", 0x18041000ULL, 0x128 },
        { "mt6895.dev-consys", 0x18042000ULL, 0x324 },
        { "mt6895.dev-consys", 0x1804b000ULL, 0x414 },
        { "mt6895.dev-consys", 0x1804d000ULL, 0x41c },
        { "mt6895.dev-consys", 0x1804f000ULL, 0x148 },
        { "mt6895.dev-bt", 0x18051000ULL, 0x1000 },
        { "mt6895.dev-consys", 0x18053800ULL, 0x1000 },
        { "mt6895.dev-consys", 0x18060000ULL, 0xc08 },
        { "mt6895.dev-consys", 0x18070000ULL, 0x8004 },
        { "mt6895.dev-bt", 0x18800000ULL, 0x1000 },
        { "mt6895.dev-bt", 0x18812000ULL, 0x1000 },
        { "mt6895.dev-gps", 0x18c00000ULL, 0x100000 },
        { "mt6895.dev-apusys_reviser", 0x19001000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x19010000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x19011000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x19012000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x19013000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x19014000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x19015000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x19016000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x19017000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x19018000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x19019000ULL, 0x1000 },
        { "mt6895.dev-apusys_reviser", 0x1903c000ULL, 0x1000 },
        { "mt6895.dev-therm_intf", 0x190e1000ULL, 0x1000 },
        { "mt6895.dev-smi_larb13", 0x1a001000ULL, 0x1000 },
        { "mt6895.dev-smi_larb14", 0x1a002000ULL, 0x1000 },
        { "mt6895.dev-smi_larb25", 0x1a003000ULL, 0x1000 },
        { "mt6895.dev-smi_larb26", 0x1a004000ULL, 0x1000 },
        { "mt6895.dev-smi_5x1_sub_comm0", 0x1a005000ULL, 0x1000 },
        { "mt6895.dev-smi_8x1_sub_comm0", 0x1a006000ULL, 0x1000 },
        { "mt6895.dev-smi_5x1_sub_comm1", 0x1a007000ULL, 0x1000 },
        { "mt6895.dev-smi_larb16", 0x1a008000ULL, 0x1000 },
        { "mt6895.dev-smi_larb17", 0x1a009000ULL, 0x1000 },
        { "mt6895.dev-smi_larb27", 0x1a00a000ULL, 0x1000 },
        { "mt6895.dev-smi_larb29", 0x1a00b000ULL, 0x1000 },
        { "mt6895.dev-smi_larb28", 0x1a00c000ULL, 0x1000 },
        { "mt6895.dev-smi_larb30", 0x1a00d000ULL, 0x1000 },
        { "mt6895.dev-seninf_top", 0x1a010000ULL, 0x20000 },
        { "mt6895.dev-remoteproc_ccd", 0x1a030000ULL, 0x10000 },
        { "mt6895.dev-cam_raw_a", 0x1a038000ULL, 0x8000 },
        { "mt6895.dev-cam_yuv_a", 0x1a050000ULL, 0x8000 },
        { "mt6895.dev-cam_yuv_a", 0x1a058000ULL, 0x8000 },
        { "mt6895.dev-cam_raw_b", 0x1a070000ULL, 0x8000 },
        { "mt6895.dev-cam_raw_b", 0x1a078000ULL, 0x8000 },
        { "mt6895.dev-cam_yuv_b", 0x1a090000ULL, 0x8000 },
        { "mt6895.dev-cam_yuv_b", 0x1a098000ULL, 0x8000 },
        { "mt6895.dev-cam_raw_c", 0x1a0b0000ULL, 0x8000 },
        { "mt6895.dev-cam_raw_c", 0x1a0b8000ULL, 0x8000 },
        { "mt6895.dev-cam_yuv_c", 0x1a0d0000ULL, 0x8000 },
        { "mt6895.dev-cam_yuv_c", 0x1a0d8000ULL, 0x8000 },
        { "mt6895.dev-pda", 0x1a100000ULL, 0x1000 },
        { "mt6895.dev-camsv1", 0x1a110000ULL, 0x1000 },
        { "mt6895.dev-camsv2", 0x1a111000ULL, 0x1000 },
        { "mt6895.dev-camsv3", 0x1a112000ULL, 0x1000 },
        { "mt6895.dev-camsv4", 0x1a113000ULL, 0x1000 },
        { "mt6895.dev-camsv5", 0x1a114000ULL, 0x1000 },
        { "mt6895.dev-camsv6", 0x1a115000ULL, 0x1000 },
        { "mt6895.dev-camsv7", 0x1a116000ULL, 0x1000 },
        { "mt6895.dev-camsv8", 0x1a117000ULL, 0x1000 },
        { "mt6895.dev-camsv1", 0x1a118000ULL, 0x1000 },
        { "mt6895.dev-camsv2", 0x1a119000ULL, 0x1000 },
        { "mt6895.dev-camsv3", 0x1a11a000ULL, 0x1000 },
        { "mt6895.dev-camsv4", 0x1a11b000ULL, 0x1000 },
        { "mt6895.dev-camsv5", 0x1a11c000ULL, 0x1000 },
        { "mt6895.dev-camsv6", 0x1a11d000ULL, 0x1000 },
        { "mt6895.dev-camsv7", 0x1a11e000ULL, 0x1000 },
        { "mt6895.dev-camsv8", 0x1a11f000ULL, 0x1000 },
        { "mt6895.dev-mraw1", 0x1a130000ULL, 0x8000 },
        { "mt6895.dev-mraw1", 0x1a138000ULL, 0x8000 },
        { "mt6895.dev-mraw2", 0x1a140000ULL, 0x8000 },
        { "mt6895.dev-mraw2", 0x1a148000ULL, 0x8000 },
        { "mt6895.dev-mraw3", 0x1a150000ULL, 0x8000 },
        { "mt6895.dev-mraw3", 0x1a158000ULL, 0x8000 },
        { "mt6895.dev-mraw4", 0x1a160000ULL, 0x8000 },
        { "mt6895.dev-mraw4", 0x1a168000ULL, 0x8000 },
        { "mt6895.dev-camsv9", 0x1a180000ULL, 0x1000 },
        { "mt6895.dev-camsv10", 0x1a181000ULL, 0x1000 },
        { "mt6895.dev-camsv11", 0x1a182000ULL, 0x1000 },
        { "mt6895.dev-camsv12", 0x1a183000ULL, 0x1000 },
        { "mt6895.dev-camsv13", 0x1a184000ULL, 0x1000 },
        { "mt6895.dev-camsv14", 0x1a185000ULL, 0x1000 },
        { "mt6895.dev-camsv15", 0x1a186000ULL, 0x1000 },
        { "mt6895.dev-camsv16", 0x1a187000ULL, 0x1000 },
        { "mt6895.dev-camsv9", 0x1a188000ULL, 0x1000 },
        { "mt6895.dev-camsv10", 0x1a189000ULL, 0x1000 },
        { "mt6895.dev-camsv11", 0x1a18a000ULL, 0x1000 },
        { "mt6895.dev-camsv12", 0x1a18b000ULL, 0x1000 },
        { "mt6895.dev-camsv13", 0x1a18c000ULL, 0x1000 },
        { "mt6895.dev-camsv14", 0x1a18d000ULL, 0x1000 },
        { "mt6895.dev-camsv15", 0x1a18e000ULL, 0x1000 },
        { "mt6895.dev-camsv16", 0x1a18f000ULL, 0x1000 },
        { "mt6895.dev-ccu_rproc", 0x1b080000ULL, 0x9000 },
        { "mt6895.dev-smi_larb19", 0x1b201000ULL, 0x1000 },
        { "mt6895.dev-gps", 0x1c000000ULL, 0x1000 },
        /* scpsys@1c001000 is its own device -- see below. */
        /* masp@1c009000 is mapped as dedicated mt6895.masp region */
        { "mt6895.dev-security_ao", 0x1c00b000ULL, 0x1000 },
        { "mt6895.dev-devapc", 0x1c018000ULL, 0x1000 },
        { "mt6895.dev-devapc", 0x1c01c000ULL, 0x1000 },
        { "mt6895.dev-sspm", 0x1c300000ULL, 0x30000 },
        { "mt6895.dev-sspm", 0x1c340000ULL, 0x10000 },
        { "mt6895.dev-tinysys_mbox", 0x1c351000ULL, 0x1000 },
        { "mt6895.dev-tinysys_mbox", 0x1c361000ULL, 0x1000 },
        { "mt6895.dev-sspm", 0x1c380000ULL, 0x80 },
        { "mt6895.dev-scp", 0x1c400000ULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c721000ULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c724000ULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c730000ULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c740000ULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c752000ULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c753000ULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c760000ULL, 0x40000 },
        { "mt6895.dev-scp", 0x1c7a5000ULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c7a5020ULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c7a5024ULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c7a5028ULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c7a502cULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c7a5030ULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c7fb000ULL, 0x100 },
        { "mt6895.dev-scp", 0x1c7fb100ULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c7fb10cULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c7fc000ULL, 0x100 },
        { "mt6895.dev-scp", 0x1c7fc100ULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c7fc10cULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c7fd000ULL, 0x100 },
        { "mt6895.dev-scp", 0x1c7fd100ULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c7fd10cULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c7fe000ULL, 0x100 },
        { "mt6895.dev-scp", 0x1c7fe100ULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c7fe10cULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c7ff000ULL, 0x100 },
        { "mt6895.dev-scp", 0x1c7ff100ULL, 0x1000 },
        { "mt6895.dev-scp", 0x1c7ff10cULL, 0x1000 },
        { "mt6895.dev-gps", 0x1c805028ULL, 0x1000 },
        { "mt6895.dev-gps", 0x1c805030ULL, 0x1000 },
        { "mt6895.dev-gps", 0x1c8050ccULL, 0x28 },
        { "mt6895.dev-apusys_reviser", 0x1d900000ULL, 0x1000 },
        { "mt6895.dev-adspsys", 0x1e000000ULL, 0x6000 },
        { "mt6895.dev-adspsys", 0x1e006000ULL, 0x100 },
        { "mt6895.dev-adspsys", 0x1e006100ULL, 0x1000 },
        { "mt6895.dev-adspsys", 0x1e00610cULL, 0x1000 },
        { "mt6895.dev-adspsys", 0x1e007000ULL, 0x100 },
        { "mt6895.dev-adspsys", 0x1e007100ULL, 0x1000 },
        { "mt6895.dev-adspsys", 0x1e00710cULL, 0x1000 },
        { "mt6895.dev-adspsys", 0x1e008000ULL, 0x100 },
        { "mt6895.dev-adspsys", 0x1e008100ULL, 0x1000 },
        { "mt6895.dev-adspsys", 0x1e00810cULL, 0x1000 },
        { "mt6895.dev-adspsys", 0x1e009000ULL, 0x100 },
        { "mt6895.dev-adspsys", 0x1e009100ULL, 0x1000 },
        { "mt6895.dev-adspsys", 0x1e00910cULL, 0x1000 },
        { "mt6895.dev-adspsys", 0x1e00b000ULL, 0x5000 },
        { "mt6895.dev-adspsys", 0x1e010000ULL, 0x9000 },
        { "mt6895.dev-devapc", 0x1e019000ULL, 0x1000 },
        { "mt6895.dev-adspsys", 0x1e01a000ULL, 0x2000 },
        { "mt6895.dev-devapc", 0x1e01c000ULL, 0x1000 },
        { "mt6895.dev-adspsys", 0x1e01d000ULL, 0x3000 },
        { "mt6895.dev-adsp_core0", 0x1e020000ULL, 0x8000 },
        { "mt6895.dev-adsp_core0", 0x1e050000ULL, 0x9000 },
        { "mt6895.dev-adsp_core1", 0x1e090000ULL, 0x8000 },
        { "mt6895.dev-adsp_core1", 0x1e0c0000ULL, 0x9000 },
        { "mt6895.dev-audio_sram", 0x1e102000ULL, 0x18000 },
        { "mt6895.dev-mminfra_bus", 0x1e900000ULL, 0x1000 },
        { "mt6895.dev-smi_disp_comm", 0x1e801000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x1e802000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x1e803000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x1e804000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x1e805000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x1e806000ULL, 0x1000 },
        { "mt6895.dev-smi_mm_m0_rsi_regs", 0x1e807000ULL, 0x1000 },
        { "mt6895.dev-smi_mm_m1_rsi_regs", 0x1e808000ULL, 0x1000 },
        { "mt6895.dev-smi_infra_disp_subcomm0", 0x1e809000ULL, 0x1000 },
        { "mt6895.dev-smi_infra_disp_subcomm1", 0x1e80a000ULL, 0x1000 },
        { "mt6895.dev-smi_sysram_comm", 0x1e80b000ULL, 0x1000 },
        { "mt6895.dev-smi_sysram_subcomm2", 0x1e80c000ULL, 0x1000 },
        { "mt6895.dev-smi_sysram_subcomm3", 0x1e80d000ULL, 0x1000 },
        { "mt6895.dev-smi_sysram_subcomm4", 0x1e80e000ULL, 0x1000 },
        { "mt6895.dev-smi_mdp_comm", 0x1e80f000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x1e810000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x1e811000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x1e812000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x1e813000ULL, 0x1000 },
        { "mt6895.dev-iommu", 0x1e814000ULL, 0x1000 },
        { "mt6895.dev-smi_mdp_m0_rsi_regs", 0x1e815000ULL, 0x1000 },
        { "mt6895.dev-smi_mdp_m1_rsi_regs", 0x1e816000ULL, 0x1000 },
        { "mt6895.dev-smi_mdp_subcomm4", 0x1e817000ULL, 0x1000 },
        { "mt6895.dev-smi_mdp_subcomm5", 0x1e818000ULL, 0x1000 },
        { "mt6895.dev-devapc", 0x1e820000ULL, 0x1000 },
        { "mt6895.dev-mminfra_bcrm_ao", 0x1e824000ULL, 0x1000 },
        { "mt6895.dev-mminfra_debug_ctl", 0x1e825000ULL, 0x1000 },
        { "mt6895.dev-devapc", 0x1e826000ULL, 0x1000 },
        { "mt6895.dev-mminfra_bcrm_pdn", 0x1e827000ULL, 0x1000 },
        { "mt6895.dev-gce", 0x1e980000ULL, 0x4000 },
        { "mt6895.dev-gce", 0x1e990000ULL, 0x4000 },
        { "mt6895.dev-vcp", 0x1ea00000ULL, 0x40000 },
        { "mt6895.dev-vcp", 0x1ec24000ULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1ec30000ULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1ec40000ULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1ec52000ULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1ec60000ULL, 0x40000 },
        { "mt6895.dev-vcp", 0x1eca5000ULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1eca5020ULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1eca5024ULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1eca5028ULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1eca502cULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1eca5030ULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1ecfb000ULL, 0x100 },
        { "mt6895.dev-vcp", 0x1ecfb100ULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1ecfb10cULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1ecfc000ULL, 0x100 },
        { "mt6895.dev-vcp", 0x1ecfc100ULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1ecfc10cULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1ecfd000ULL, 0x100 },
        { "mt6895.dev-vcp", 0x1ecfd100ULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1ecfd10cULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1ecfe000ULL, 0x100 },
        { "mt6895.dev-vcp", 0x1ecfe100ULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1ecfe10cULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1ecff000ULL, 0x100 },
        { "mt6895.dev-vcp", 0x1ecff100ULL, 0x1000 },
        { "mt6895.dev-vcp", 0x1ecff10cULL, 0x1000 },
        { "mt6895.dev-mdp_mutex", 0x1f001000ULL, 0x1000 },
        { "mt6895.dev-smi_larb2", 0x1f002000ULL, 0x1000 },
        { "mt6895.dev-mdp_rdma", 0x1f003000ULL, 0x1000 },
        { "mt6895.dev-mdp_rdma", 0x1f004000ULL, 0x1000 },
        { "mt6895.dev-mdp_hdr", 0x1f005000ULL, 0x1000 },
        { "mt6895.dev-mdp_hdr", 0x1f006000ULL, 0x1000 },
        { "mt6895.dev-mdp_aal", 0x1f007000ULL, 0x1000 },
        { "mt6895.dev-mdp_aal", 0x1f008000ULL, 0x1000 },
        { "mt6895.dev-mdp_rsz", 0x1f009000ULL, 0x1000 },
        { "mt6895.dev-mdp_rsz", 0x1f00a000ULL, 0x1000 },
        { "mt6895.dev-mdp_tdshp", 0x1f00b000ULL, 0x1000 },
        { "mt6895.dev-mdp_tdshp", 0x1f00c000ULL, 0x1000 },
        { "mt6895.dev-mdp_color", 0x1f00d000ULL, 0x1000 },
        { "mt6895.dev-mdp_color", 0x1f00e000ULL, 0x1000 },
        { "mt6895.dev-mdp_wrot", 0x1f00f000ULL, 0x1000 },
        { "mt6895.dev-mdp_wrot", 0x1f010000ULL, 0x1000 },
        { "mt6895.dev-mdp_fg", 0x1f011000ULL, 0x1000 },
        { "mt6895.dev-mdp_fg", 0x1f012000ULL, 0x1000 },
        { "mt6895.dev-mml_mutex0", 0x1f801000ULL, 0x1000 },
        { "mt6895.dev-smi_larb3", 0x1f802000ULL, 0x1000 },
        { "mt6895.dev-mml_rdma0", 0x1f803000ULL, 0x1000 },
        { "mt6895.dev-mml_rdma1", 0x1f804000ULL, 0x1000 },
        { "mt6895.dev-mml_hdr0", 0x1f805000ULL, 0x1000 },
        { "mt6895.dev-mml_hdr1", 0x1f806000ULL, 0x1000 },
        { "mt6895.dev-mml_aal0", 0x1f807000ULL, 0x1000 },
        { "mt6895.dev-mml_aal1", 0x1f808000ULL, 0x1000 },
        { "mt6895.dev-mml_rsz0", 0x1f809000ULL, 0x1000 },
        { "mt6895.dev-mml_rsz1", 0x1f80a000ULL, 0x1000 },
        { "mt6895.dev-mml_tdshp0", 0x1f80b000ULL, 0x1000 },
        { "mt6895.dev-mml_tdshp1", 0x1f80c000ULL, 0x1000 },
        { "mt6895.dev-mml_color0", 0x1f80d000ULL, 0x1000 },
        { "mt6895.dev-mml_color1", 0x1f80e000ULL, 0x1000 },
        { "mt6895.dev-mml_wrot0", 0x1f80f000ULL, 0x1000 },
        { "mt6895.dev-mml_wrot1", 0x1f810000ULL, 0x1000 },
        { "mt6895.dev-mml_rsz2", 0x1f813000ULL, 0x1000 },
        { "mt6895.dev-mml_rsz3", 0x1f814000ULL, 0x1000 },
        { "mt6895.dev-mml_wrot2", 0x1f815000ULL, 0x1000 },
        { "mt6895.dev-mml_wrot3", 0x1f816000ULL, 0x1000 },
        { "mt6895.dev-hre_top_mdpsys", 0x1f817000ULL, 0x1000 },
    };

    /*
     * The thirteen iocfg windows from the pinctrl node's reg list, in the
     * order the device tree gives them.
     */
    static const uint64_t iocfg[] = {
        0x11c20000ULL, 0x11c30000ULL, 0x11c40000ULL,
        0x11d10000ULL, 0x11d30000ULL, 0x11d40000ULL, 0x11d50000ULL,
        0x11d60000ULL,
        0x11e10000ULL,
        0x11ea0000ULL, 0x11eb0000ULL, 0x11f20000ULL, 0x11f30000ULL,
    };

    for (size_t i = 0; i < ARRAY_SIZE(iocfg); i++) {
        DeviceState *dev = qdev_new(TYPE_MTK_SYSCON);

        qdev_prop_set_uint32(dev, "size", MT6895_PINCTRL_SIZE);
        if (!sysbus_realize(SYS_BUS_DEVICE(dev), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, iocfg[i]);
    }

    for (size_t i = 0; i < ARRAY_SIZE(blocks); i++) {
        /*
         * Two DCM entries map exactly the same windows as earlier entries:
         * mt6895.dcm-infracfg-ao == infracfg_ao_clk (0x10001000) and
         * mt6895.dcm-infracfg-ao-mem == nemi_bus (0x10270000).  With two
         * plain syscon regions on one address QEMU keeps the later one, so
         * the DCM entry used to shadow the window that owns the infracfg_ao
         * bus-protection/clock-gate registers (MTK_BPTRACE showed the
         * kernel's scpsys writes landing in mt6895.dcm-infracfg-ao).  The
         * DCM driver only needs MMIO backing, which the earlier window
         * already provides, so the duplicate is skipped.
         */
        if (!strcmp(blocks[i].name, "mt6895.dcm-infracfg-ao") ||
            !strcmp(blocks[i].name, "mt6895.dcm-infracfg-ao-mem")) {
            continue;
        }
        /*
         * M4U/IOMMU banks: real register semantics (TLB invalidate + CPE_DONE)
         * and, by default, real ARMv7 short-descriptor page table translation;
         * see hw/misc/mtk_iommu.c.  MTK_IOMMU=0 turns translation off (the
         * legacy bring-up path where clients program physical addresses).  On a
         * plain register window the kernel's mtk_iommu_tlb_flush_all polls
         * REG_MMU_CPE_DONE forever ("TLB flush All timed out").
         * MTK_NO_IOMMU=1 reverts to syscon.
         */
        /*
         * GCE/CMDQ: the CMDQ driver polls thread registers directly, so a
         * plain register window does not work: the warm-reset poll
         * (CMDQ_THR_WARM_RESET bit0 self-clear), the clear-event poll
         * (CMDQ_THR_CURR_ADDR == END_ADDR) and every mailbox task completion
         * time out ("reset GCE thread 0 failed",
         * "clear event instructions timeout pc:0x...", earlier bring-up notes,292 hwv).
         * hw/misc/mtk_gce.c models the thread state machine and the per-thread
         * completion IRQ.  The SPI numbers are the vendor device tree's:
         * gce@1e980000 -> GIC_SPI 401, gce_m@1e990000 -> GIC_SPI 400
         * (research/dts/vendor-mt6895.dts:5527, :5570).
         * MTK_NO_GCE=1 reverts to the plain syscon window.
         */
        if (!strcmp(blocks[i].name, "mt6895.dev-gce") &&
            !getenv("MTK_NO_GCE")) {
            DeviceState *gce = qdev_new("mtk-gce");
            unsigned spi = blocks[i].base == 0x1e990000ULL ? 400 : 401;

            if (!sysbus_realize_and_unref(SYS_BUS_DEVICE(gce), errp)) {
                return;
            }
            sysbus_mmio_map(SYS_BUS_DEVICE(gce), 0, blocks[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(gce), 0,
                               qdev_get_gpio_in(s->gic, spi));
            if (blocks[i].base == 0x1e980000ULL) {
                /* Main GCE: the display CMDQ threads run on this one. */
                s->gce = gce;
            }
            continue;
        }

        /*
         * tinysys mailbox (SCMI transport): the two register windows
         * 0x1c351000 / 0x1c361000 and the two 0x80-byte shared-memory windows
         * 0x1c350000 (tx) / 0x1c360000 (rx) have to belong to one device, so
         * both windows are taken out of the generic syscon list here and owned
         * by mtk-tinysys-mbox (see mt6895_create_msdc).  The device models the
         * doorbell, the receive-status register and a minimal SCMI reply agent;
         * MTK_NO_SCMI=1 only turns the agent off.
         */
        if (!strcmp(blocks[i].name, "mt6895.dev-tinysys_mbox")) {
            continue;
        }

        if (!strcmp(blocks[i].name, "mt6895.dev-iommu") &&
            !getenv("MTK_NO_IOMMU")) {
            DeviceState *idev = qdev_new("mtk-iommu");
            uint64_t base = blocks[i].base;
            unsigned iommu_id = MTK_IOMMU_DEV_NUM, bank = 0;

            /*
             * Tag every bank window with the M4U block it belongs to and the
             * bank index inside it (vendor DT: disp_iommu @ 0x1e802000 with
             * banks 1..4 at 0x1e803000..0x1e806000, mdp_iommu @ 0x1e810000
             * with 0x1e811000..0x1e814000, apu_iommu0 @ 0x19010000 and
             * apu_iommu1 @ 0x19015000, each bank 4 KiB).  Only bank 0 is
             * translated: banks 1..4 are the protected/secure domains owned by
             * TF-A, Linux never programs their page tables (hw/misc/mtk_iommu.c
             * still keeps their register file).
             */
            if (base >= 0x1e802000ULL && base <= 0x1e806000ULL) {
                iommu_id = MTK_IOMMU_DEV_DISP;
                bank = (base - 0x1e802000ULL) >> 12;
            } else if (base >= 0x1e810000ULL && base <= 0x1e814000ULL) {
                iommu_id = MTK_IOMMU_DEV_MDP;
                bank = (base - 0x1e810000ULL) >> 12;
            } else if (base >= 0x19010000ULL && base <= 0x19014000ULL) {
                iommu_id = MTK_IOMMU_DEV_APU0;
                bank = (base - 0x19010000ULL) >> 12;
            } else if (base >= 0x19015000ULL && base <= 0x19019000ULL) {
                iommu_id = MTK_IOMMU_DEV_APU1;
                bank = (base - 0x19015000ULL) >> 12;
            }
            qdev_prop_set_uint32(idev, "iommu-id", iommu_id);
            qdev_prop_set_uint32(idev, "bank", bank);
            if (!sysbus_realize_and_unref(SYS_BUS_DEVICE(idev), errp)) {
                return;
            }
            sysbus_mmio_map(SYS_BUS_DEVICE(idev), 0, base);
            continue;
        }

        /*
         * USB host controller 0.  The real vendor DT gives xhci0@11200000 a
         * 4 KiB register window ("mediatek,mtk-xhci", reg-names "mac", irq
         * GIC_SPI 129, see docs/xaga/architecture.md), and
         * that is exactly what QEMU's sysbus xHCI now provides (see the OFF_*
         * comments in hw/usb/hcd-xhci.c).  A usb-kbd is attached to its bus
         * so the guest enumerates a real HID device once xhci-mtk registers
         * the host.  MTK_USB_MOUSE=1 adds a mouse, MTK_USB_AUDIO=<audiodev>
         * adds usb-audio (run with -audiodev <backend>,id=<audiodev>).
         * MTK_NO_XHCI=1 restores the old plain syscon window.
         */
        if (!strcmp(blocks[i].name, "mt6895.dev-xhci0") &&
            !MTK_ENV_CACHED("MTK_NO_XHCI")) {
            DeviceState *xdev = qdev_new(TYPE_XHCI_SYSBUS);
            const char *audio_dev = getenv("MTK_USB_AUDIO");
            const char *netdev_id = getenv("MTK_USB_NET");

            const char *mapms = getenv("MTK_XHCI_MAP_MS");
            int64_t delay_ms = mapms ? strtoll(mapms, NULL, 0) : 20000;

            qdev_prop_set_uint32(xdev, "intrs", 1);
            qdev_prop_set_uint32(xdev, "slots", 64);
            if (!sysbus_realize_and_unref(SYS_BUS_DEVICE(xdev), errp)) {
                return;
            }
            fprintf(stderr, "MTK-XHCI: realize intrs=%u slots=%u numports=%u\n",
                    XHCI_SYSBUS(xdev)->xhci.numintrs,
                    XHCI_SYSBUS(xdev)->xhci.numslots,
                    XHCI_SYSBUS(xdev)->xhci.numports);
            sysbus_connect_irq(SYS_BUS_DEVICE(xdev), 0,
                               qdev_get_gpio_in(s->gic, 129));
            if (delay_ms > 0) {
                /*
                 * LK's own USB/fastboot init owns 0x11200000 before Linux is
                 * started; keep the plain syscon window there and overlay the
                 * xHCI only after LK has jumped to the kernel.
                 */
                DeviceState *sdev = qdev_new(TYPE_MTK_SYSCON);

                qdev_prop_set_uint32(sdev, "size", blocks[i].size);
                qdev_prop_set_string(sdev, "name", blocks[i].name);
                if (!sysbus_realize(SYS_BUS_DEVICE(sdev), errp)) {
                    return;
                }
                sysbus_mmio_map(SYS_BUS_DEVICE(sdev), 0, blocks[i].base);
                mtk_xhci_map_later(xdev, blocks[i].base, delay_ms);
            } else {
                sysbus_mmio_map(SYS_BUS_DEVICE(xdev), 0, blocks[i].base);
            }

            usb_create_simple(&XHCI_SYSBUS(xdev)->xhci.bus, "usb-kbd");
            if (MTK_ENV_CACHED("MTK_USB_MOUSE")) {
                usb_create_simple(&XHCI_SYSBUS(xdev)->xhci.bus, "usb-mouse");
            }
            if (audio_dev && *audio_dev) {
                DeviceState *adev = qdev_new("usb-audio");

                qdev_prop_set_string(adev, "audiodev", audio_dev);
                usb_realize_and_unref(USB_DEVICE(adev),
                                      &XHCI_SYSBUS(xdev)->xhci.bus,
                                      &error_fatal);
            }
            if (netdev_id && *netdev_id) {
                NetClientState *nc = qemu_find_netdev(netdev_id);

                if (!nc) {
                    fprintf(stderr, "mt6895: MTK_USB_NET=%s: no such "
                            "-netdev, usb-net not attached\n", netdev_id);
                } else {
                    DeviceState *ndev = qdev_new("usb-net");

                    qdev_prop_set_netdev(ndev, "netdev", nc);
                    usb_realize_and_unref(USB_DEVICE(ndev),
                                          &XHCI_SYSBUS(xdev)->xhci.bus,
                                          &error_fatal);
                }
            }
            continue;
        }

        /*
         * Fallback only: with the behavioural DSI controller disabled the
         * plain "dsi-intr" window still owns INTEN/INTSTA and drives the same
         * level interrupt, so MTK_NO_DSI_MODEL=1 keeps a working DSI IRQ.
         * With the controller installed this window is shadowed by the
         * priority-3 controller region, so it must not be added.
         */
        if (!strcmp(blocks[i].name, "mt6895.dev-dsi") &&
            MTK_ENV_CACHED("MTK_NO_DSI_MODEL")) {
            memory_region_init_io(&s->dsi_mr, OBJECT(s),
                                  &mt6895_dsi_intr_ops, s,
                                  "mt6895.dsi-intr", blocks[i].size);
            memory_region_add_subregion_overlap(get_system_memory(),
                                                blocks[i].base, &s->dsi_mr, 2);
            continue;
        }

        if (!strcmp(blocks[i].name, "mt6895.dev-disp_mutex") &&
            !MTK_ENV_CACHED("MTK_NO_DISPMUTEX")) {
            int idx = blocks[i].base == 0x14401000ULL ? 1 : 0;

            memory_region_init_io(&s->disp_mutex[idx], OBJECT(s),
                                  idx ? &mt6895_disp_mutex1_ops
                                      : &mt6895_disp_mutex0_ops, s,
                                  idx ? "mt6895.disp-mutex1"
                                      : "mt6895.disp-mutex0", blocks[i].size);
            memory_region_add_subregion_overlap(get_system_memory(),
                                                blocks[i].base,
                                                &s->disp_mutex[idx], 2);
            fprintf(stderr, "MTK-DISPMUTEX: state machine %d at %#llx\n",
                    idx, (unsigned long long)blocks[i].base);
            continue;
        }

        DeviceState *dev = qdev_new(TYPE_MTK_SYSCON);
        const char *n = blocks[i].name;

        qdev_prop_set_uint32(dev, "size", blocks[i].size);
        /* Names the window in the trace, so accesses can be attributed. */
        qdev_prop_set_string(dev, "name", n);
        if (!strcmp(n, "mt6895.pinctrl") && mt6895_dram_is_6g(s)) {
            /*
             * The xaga board pulls the pinctrl DRAM-generation strap high;
             * the preloader's dram-type detector reads it to select LPDDR5X
             * (dram_type 0xC).  Only the genuine 6 GiB configuration enables
             * it, so -m 8G keeps the historical LPDDR5 behaviour.
             */
            qdev_prop_set_bit(dev, "lpdd5x-strap", true);
        }

    /*
         * REVERTED all-ones PLL probe. Making the PLL windows read all-ones
         * does clear the "pll prepare timeout" -- so the unmodelled PLL status
         * really is that timeout's cause -- but it then trips a second check
         * in clk-chk-mt6895 (clkchk-mt6895.c:769 instead of :802) and the boot
         * panics earlier, 18.8s against 124.7s, with fewer modules loaded.
         * The probe is left out until the individual registers are modelled
         * with the values the checker expects.
         */
    /*
         * REVERTED. Three variants were tried on the PLL windows -- all-ones,
         * bit 31 only, and bit 31 plus the hwv block the trace showed
         * mtk_hwv_pll_prepare operating on. Each one does clear
         * "pll prepare timeout", so the unmodelled PLL status really is that
         * timeout's cause, but every variant also trips a different check
         * (clkchk-mt6895.c:769, dump_hwv_history) and the boot panics earlier
         * -- 14.5s against 116-125s, with fewer modules loaded. The baseline
         * of all-zero PLL registers remains the furthest this reaches, so the
         * probe is out until the 13.28s timeout that :769 reports is
         * understood; that one is not the 876ms PLL poll.
         */
        /*
         * REVERTED the narrowed PLL probe. Applying bit 31 to just
         * topckgen_clk offsets 0x10 and 0x20 -- the two registers the trace
         * showed the PLL poll reading -- did clear "pll prepare timeout"
         * without corrupting other windows, but the three whole-progress
         * metrics came out identical to the baseline (59 modules, 121.7s vs
         * 125s) and only the checkpoint moved, 802 -> 769. A lateral move, so
         * the probe is out.
         *
         * It did establish something useful: those two offsets are themselves
         * mux registers. The boot then reports
         * "disp0_sel mux prepare timeout(80808080)", and 0x80808080 is the
         * probe's own byte pattern -- so what mtk_hwv_pll_prepare polls is a
         * mux field, not a PLL status bit. Satisfying it needs the field's
         * real value, which is a clock-tree model rather than a status bit.
         */
        /*
         * REVERTED the topckgen status-mirror experiment. Mapping the polled
         * status registers (0x10 / 0x20) onto the mux config registers
         * (0x8 / 0xc) left the three whole-progress metrics unchanged (59
         * modules, 124.9s, checkpoint 802) and did not clear
         * "pll prepare timeout" either -- so that mirror is not the model
         * either. Six clock-model attempts have now been falsified the same
         * way (see the session notes): the driver's expected semantics are
         * not derivable from the register traffic alone.
         */
        (void)n;

        if (!sysbus_realize(SYS_BUS_DEVICE(dev), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, blocks[i].base);
    }

    /*
     * Tell the GCE which CMDQ events this machine actually generates, so a
     * WFE may block on them.  Every other event keeps the non-blocking
     * fallback: only the events below are raised by the frame engines.
     */
    if (s->gce) {
        if (!MTK_ENV_CACHED("MTK_NO_DSI_MODEL")) {
            mtk_gce_event_mark_generated(s->gce, MTK_GCE_EVENT_DSI0_SOF);
            mtk_gce_event_mark_generated(s->gce, MTK_GCE_EVENT_DSI0_EOF);
        }
        if (mtk_disp_vblank_enabled()) {
            /* Raised by mtk_disp_vblank_tick() for display mutex 0. */
            mtk_gce_event_mark_generated(s->gce, MTK_GCE_EVENT_MUTEX0_EOF);
        }
    }
}

static void mt6895_create_its(Mt6895MachineState *s, Error **errp)
{
    s->its = qdev_new(its_class_name());
    object_property_set_link(OBJECT(s->its), "parent-gicv3", OBJECT(s->gic),
                             &error_abort);

    if (!sysbus_realize(SYS_BUS_DEVICE(s->its), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(s->its), 0, MT6895_ITS_BASE);
}

static void mt6895_create_pcie(Mt6895MachineState *s, Error **errp)
{
    s->pcie = qdev_new(TYPE_GPEX_HOST);
    qdev_prop_set_uint32(s->pcie, "num-irqs", PCI_NUM_PINS);
    qdev_prop_set_uint64(s->pcie, PCI_HOST_ECAM_BASE, MT6895_PCIE_ECAM_BASE);
    qdev_prop_set_uint64(s->pcie, PCI_HOST_ECAM_SIZE, MT6895_PCIE_ECAM_SIZE);
    qdev_prop_set_uint64(s->pcie, PCI_HOST_PIO_BASE, MT6895_PCIE_PIO_BASE);
    qdev_prop_set_uint64(s->pcie, PCI_HOST_PIO_SIZE, MT6895_PCIE_PIO_SIZE);
    qdev_prop_set_uint64(s->pcie, PCI_HOST_BELOW_4G_MMIO_BASE,
                         MT6895_PCIE_MMIO_BASE);
    qdev_prop_set_uint64(s->pcie, PCI_HOST_BELOW_4G_MMIO_SIZE,
                         MT6895_PCIE_MMIO_SIZE);

    if (!sysbus_realize(SYS_BUS_DEVICE(s->pcie), errp)) {
        return;
    }

    /*
     * GPEX's ECAM region is a fixed 256 MiB (256 buses). Mapping all of it
     * would run over the syscon block at 0x10000000, and the guest is told
     * bus-range <0 0>, so expose only bus 0's 1 MiB through an alias.
     */
    memory_region_init_alias(&s->pcie_ecam_alias, OBJECT(s), "mt6895.pcie-ecam",
                             sysbus_mmio_get_region(SYS_BUS_DEVICE(s->pcie), 0),
                             0, MT6895_PCIE_ECAM_SIZE);
    memory_region_add_subregion(get_system_memory(), MT6895_PCIE_ECAM_BASE,
                                &s->pcie_ecam_alias);

    for (int i = 0; i < PCI_NUM_PINS; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(s->pcie), i,
                           qdev_get_gpio_in(s->gic, MT6895_PCIE_IRQ + i));
        gpex_set_irq_num(GPEX_HOST(s->pcie), i, MT6895_PCIE_IRQ + i);
    }
}

static void mt6895_create_uarts(Mt6895MachineState *s, Error **errp)
{
    static const hwaddr base[2] = { MT6895_UART0_BASE, MT6895_UART1_BASE };
    static const int spi[2] = { MT6895_UART0_SPI, MT6895_UART1_SPI };

    for (int i = 0; i < 2; i++) {
        s->uart[i] = qdev_new(TYPE_MTK_UART);
        qdev_prop_set_chr(s->uart[i], "chardev", serial_hd(i));

        if (!sysbus_realize(SYS_BUS_DEVICE(s->uart[i]), errp)) {
            return;
        }

        sysbus_mmio_map(SYS_BUS_DEVICE(s->uart[i]), 0, base[i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(s->uart[i]), 0,
                           qdev_get_gpio_in(s->gic, spi[i]));
    }
}

/*
 * DIAGNOSTIC: observe the preloader's "OE backup pending" flag byte.
 *
 * dramc_pi_calibration_api.c stores it at SRAM0 0x0010B3F4 -- set at
 * 0x02021B16 on the r1==0 entry, cleared at 0x02021A12 on the r1!=0 entry --
 * and asserts (line 17136) when the r1==0 path finds it already set. Logging
 * both accesses gives the exact set/clear ordering that the execution-count
 * traces cannot show (TB chaining hides repeated entries).
 */

/*
 * WORKAROUND helper for the probe above: true when the current OE-flag access
 * comes from one of the two backup call sites rather than the restore.
 */
#define MT6895_OE_BACKUP_LR_DIRECT  0x02022a03u  /* bl 0x0202305C @0x02029FE */
#define MT6895_OE_BACKUP_LR_WRAPPER 0x02021331u  /* via wrapper 0x0202131C  */

;

/* Temporary: sample the guest PC once a second to locate boot hotspots. */
static void mt6895_pc_sample(void *opaque)
{
    QEMUTimer *t = opaque;
    CPUState *cs = qemu_get_cpu(0);

    if (cs) {
        CPUARMState *env = cpu_env(cs);
        uint64_t pc = env->aarch64 ? env->pc : env->regs[15];

        /*
         * MTK_PCTIME (diagnostic, ): host wall-clock time series
         * with the guest PC. 5 ms period; t=0 at the first sample. Lets the
         * sample stream be aligned with the serial-byte timeline so a slow LK
         * phase can be split into guest spin/poll vs guest CPU work.
         */
        if (MTK_ENV_CACHED("MTK_PCTIME")) {
            static gint64 pct0;

            if (!pct0) {
                pct0 = g_get_monotonic_time();
            }
            uint64_t fp = env->aarch64 ? env->xregs[29] : env->regs[11];
            uint64_t caller = 0;

            if (fp) {
                cpu_memory_rw_debug(cs, fp + 8, (uint8_t *)&caller, 8, false);
            }
            fprintf(stderr,
                    "PCTIME %9.3f ep=%ld.%06ld pc=%#lx clr=%#lx"
                    " r19=%#lx r20=%#lx r21=%#lx r22=%#lx r29=%#lx a64=%d\n",
                    qemu_clock_get_ns(QEMU_CLOCK_REALTIME) / 1e9,
                    (long)(g_get_real_time() / 1000000),
                    (long)(g_get_real_time() % 1000000),
                    (unsigned long)pc, (unsigned long)caller,
                    (unsigned long)env->xregs[19],
                    (unsigned long)env->xregs[20],
                    (unsigned long)env->xregs[21],
                    (unsigned long)env->xregs[22],
                    (unsigned long)env->xregs[29],
                    (int)env->aarch64);
            fflush(stderr);
            /* t is a SCALE_MS timer: expire_time is in milliseconds. */
            timer_mod(t, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 5);
            return;
        }

        /*
         * MTK_PCFAST: sample far more often and only report the crypto
         * dispatch region, so a short-lived call can still be observed.
         * The virtual clock advances one tick per instruction under icount,
         * so a 50us period lands roughly every 780 instructions of guest
         * time.
         */
        if (MTK_ENV_CACHED("MTK_PCFAST")) {
            static int nfast;

            if (pc >= 0x0202C000ULL && pc < 0x0202D000ULL) {
                if (nfast < 600) {
                    nfast++;
                    fprintf(stderr, "PCFAST-GPT pc=%#lx\n", (unsigned long)pc);
                }
            }
            if (pc >= 0x0203F000ULL && pc < 0x02040000ULL) {
                if (nfast < 1200) {
                    nfast++;
                    fprintf(stderr, "PCFAST-PART pc=%#lx\n", (unsigned long)pc);
                }
            }
            if (pc >= 0x50f7d000ULL && pc < 0x50f7e000ULL) {
                if (nfast < 2000) {
                    nfast++;
                    fprintf(stderr, "PCFAST-SEC pc=%#lx\n", (unsigned long)pc);
                }
            }
            if (pc >= 0x50f78000ULL && pc < 0x50f7a000ULL) {
                if (nfast < 2600) {
                    nfast++;
                    fprintf(stderr, "PCFAST-CRYPTO pc=%#lx\n", (unsigned long)pc);
                }
            }
            if (pc >= 0x62f4a7c0ULL && pc < 0x62f4a900ULL) {
                if (nfast < 3200) {
                    nfast++;
                    fprintf(stderr, "PCFAST-PSCAN pc=%#lx\n", (unsigned long)pc);
                }
            }
            {
                /* Storage bring-up call chain: report which levels ever run. */
                static const struct { uint64_t lo, hi; const char *tag; } chain[] = {
                    { 0x0200c524ULL, 0x0200c600ULL, "REGTBL"  },
                    { 0x0200e794ULL, 0x0200e820ULL, "C1"      },
                    { 0x0203f8e0ULL, 0x0203f960ULL, "C2"      },
                    { 0x0204d630ULL, 0x0204d700ULL, "C3"      },
                    { 0x02041c08ULL, 0x02041c60ULL, "C4"      },
                    { 0x0204719cULL, 0x02047200ULL, "C5"      },
                };
                unsigned ci;

                for (ci = 0; ci < ARRAY_SIZE(chain); ci++) {
                    if (pc >= chain[ci].lo && pc < chain[ci].hi) {
                        if (nfast < 5000) {
                            unsigned ci2 = ci;

                            nfast++;
                            fprintf(stderr, "PCFAST-CHAIN-%s pc=%#lx\n",
                                    chain[ci2].tag, (unsigned long)pc);
                        }
                        break;
                    }
                }
            }
            if (pc >= 0x02045ac8ULL && pc < 0x02045b30ULL) {
                if (nfast < 4200) { nfast++; fprintf(stderr, "PCFAST-UFS1 pc=%#lx\n", (unsigned long)pc); }
            }
            if (pc >= 0x02045d40ULL && pc < 0x02045e40ULL) {
                if (nfast < 4400) { nfast++; fprintf(stderr, "PCFAST-UFS2 pc=%#lx\n", (unsigned long)pc); }
            }
            if (pc >= 0x02047114ULL && pc < 0x02047180ULL) {
                if (nfast < 4600) { nfast++; fprintf(stderr, "PCFAST-UFS3 pc=%#lx\n", (unsigned long)pc); }
            }
            if (pc >= 0x0200ded0ULL && pc < 0x0200dee0ULL) {
                if (nfast < 4200) {
                    nfast++;
                    fprintf(stderr, "PCFAST-WRSTUB pc=%#lx\n", (unsigned long)pc);
                }
            }
            if (pc >= 0x62f21a80ULL && pc < 0x62f21c90ULL) {
                if (nfast < 3800) {
                    nfast++;
                    fprintf(stderr, "PCFAST-BDEVREG pc=%#lx\n", (unsigned long)pc);
                }
            }
            /* 10us ~= 156 instructions under icount: fine enough to catch
             * functions that only run once for a few hundred instructions. */
            timer_mod(t, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) + 10);
            return;
        }
        fprintf(stderr, "PCSAMPLE pc=%#lx a64=%d\n",
                (unsigned long)pc, (int)env->aarch64);
    }
    timer_mod(t, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
}

/* Temporary: wall-clock timing of the machine init stages. */
static gint64 mt6895_t0;
static void mt6895_mark(const char *what)
{
    if (MTK_ENV_CACHED("MTK_INITTIME")) {
        gint64 now = g_get_monotonic_time();

        fprintf(stderr, "INITTIME %-24s %7.3f s\n", what,
                (now - mt6895_t0) / 1e6);
    }
}

/*
 * DRAM information register (0x1000d030).
 *
 * The preloader reads this register once to learn the DRAM configuration the
 * BootROM detected, caches it in a global (0x020f47c4) and prints
 * "dram_size=%d" / "dram info: ...".  Bit 20 of the value is the Vdd2l type
 * field.  With the register reading zero the payload reports
 * "dram info unset", classifies every boot as cold, retrains DRAM and resets
 * again -- an endless loop.
 *
 * The BootROM is what fills this register on hardware, so the machine models
 * the value it actually presents: DRAM size in MiB in the low bits plus the
 * normal Vdd2l type in bit 20.
 */
#define MT6895_DRAM_INFO_ADDR   0x1000d030ULL

static uint64_t mt6895_dram_info_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint64_t mib = (uint64_t)MACHINE(s)->ram_size / (1024 * 1024);
    uint64_t val = 0;

    switch (off) {
    case 0x00:
        /* Base word: DRAM size in MiB plus the normal Vdd2l type in bit 20 */
        val = (mib & 0xfffff) | (1ULL << 20);
        break;
    case 0x10:
        /* dram_size, printed verbatim by the preloader */
        val = mib;
        break;
    case 0x14:
        /* Vdd2l status word: bit 20 marks the normal type */
        val = 1ULL << 20;
        break;
    default:
        val = 0;
        break;
    }
    return extract64(val, 0, size * 8);
}

static void mt6895_dram_info_write(void *opaque, hwaddr off, uint64_t val,
                                   unsigned size)
{
    /* Read-only: the BootROM owns this word. */
}

static const MemoryRegionOps mt6895_dram_info_ops = {
    .read = mt6895_dram_info_read,
    .write = mt6895_dram_info_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/*
 * Minimal behavioural model of the MIPI DSI controller at 0x14017000.
 *
 * Why this exists: LK's display bring-up enters ULPS with
 *
 *   0x50f3f684  sub sp, sp, #0x70                 ; DSI_enter_ULPS
 *   ...
 *   ldr x23, [table, idx, lsl #3]                 ; DSI register base
 *   str/ldr [x23, #0x008] ...                     ; DSI_INTEN  : clear the status
 *   str/ldr [x23, #0x00c] ...                     ; DSI_INTSTA : clear the status
 *   orr/str [x23, #0x108], #8 / #2                ; PHY_LD0CON : ULPM entry
 *   orr/str [x23, #0x104], #2                     ; PHY_LCCON  : ULPM entry
 * loop:
 *   ldr w8, [x23, #0x00c]                         ; DSI_INTSTA
 *   tbnz w8, #0xf, done                           ; bit15 = SLEEPIN_DONE
 *   ... print "DSI_enter_ULPS:438: [LK_DDP/DSI]error:polling SLEEPIN_DONE" ...
 *
 * (found by disassembling parts/lk_a.img; the three string VAs are
 *  0x9bc35 "%s:%d: [LK_DDP/DSI]error:polling SLEEPIN_DONE %d",
 *  0x9bc67 "DSI_enter_ULPS" and the base table at payload offset 0xa0ba0 holds
 *  0x14017000 / 0x14017208 / 0x14017d00).
 *
 * With no model behind 0x14017000 the poll never sees bit15 and LK spins for
 * ever -- that is exactly where the 2026-11-06 golden log ends.
 *
 * The status bit is *derived from the guest's own register writes* (both ULPM
 * enables set -> PHY reports sleep-in done; a write clearing INTSTA bit15
 * clears it again), not pre-set, so no firmware data is fabricated.
 */
/* The register file itself lives in Mt6895MachineState (dsi_regs): the
 * behavioural controller, the INTEN/INTSTA interrupt model and the frame
 * engine all operate on that one array. */

/*
 * DCS read-back.
 *
 * LK's DSI_dcs_read_lcm_reg_v2 (payload offset 0x41d68 in parts/lk_a.img)
 * queues two CMDQ words, programs CMDQ_SIZE and then pulses DSI_START:
 *
 *   0x41f50  CMDQ[0] = 0x3700 | (max_return_size << 16)
 *   0x41f58  CMDQ[1] = 0x0600 | (dcs_opcode << 16) | 4
 *   0x41f60  DSI_CMDQ_SIZE = 0x8002            (two words, bit15 set)
 *   0x41f68  DSI_START = 0, 1, 0
 *   0x41f74  poll INTSTA bit0 (LPRX_RD_RDY),  udelay(1000) x 20
 *   0x420ac  write DSI_RACK |= 1
 *   0x420c4  poll INTSTA bit1 (CMD_DONE),    udelay(1000) x 20
 *   0x41ea8  read DSI_RX_DATA0..3, parse the response packet
 *
 * The two words above are the ones the firmware really writes: the
 * MTK-DSITRACE log of a boot (vendor module analysis
 * base_RUN_lkperf.err, lines 1279-1285) shows 0xd00 = 0x13700 and
 * 0xd04 = 0xa0604 with CMDQ_SIZE 0x8002 for the Get Power Mode (0x0A) read
 * issued by disp_lcm_init().
 *
 * So a started read command has to be answered: latch the panel response in
 * DSI_RX_DATA0..3 and report RD_RDY | CMD_DONE.  LK parses the packet as
 *   byte0 = MIPI response data type, bytes 1.. = parameters,
 * where 0x21/0x22 are the DCS short-read responses with 0/1 parameter
 * (0x11/0x12 are the generic equivalents).
 *
 * Panel state is kept minimal and is derived only from the DCS write
 * commands the guest itself queues (short writes, MIPI data type 0x05/0x15,
 * opcode in bits 23:16): 0x10 sleep in, 0x11 sleep out, 0x28 display off,
 * 0x29 display on.  The same boot trace shows LK sending 0x11 (0xd00 =
 * 0x110500, line 1171) and 0x29 (0xd00 = 0x290500, line 1183) before the
 * read, which is what makes Get Power Mode report 0x9C -- normal display
 * mode on, sleep out, display on.
 */


static uint32_t mt6895_dsi_word(Mt6895MachineState *s, unsigned off)
{
    return s->dsi_regs[off >> 2];
}

/* Run the command queue the guest just started (DSI_START bit0 or bit16). */
static void mt6895_dsi_cmdq_run(Mt6895MachineState *s)
{
    unsigned n = s->dsi_regs[MTK_DSI_CMDQ_SIZE >> 2] & 0x3f;
    uint32_t w0 = mt6895_dsi_word(s, MTK_DSI_CMD_WORDS);
    uint32_t w1 = n >= 2 ? mt6895_dsi_word(s, MTK_DSI_CMD_WORDS + 4) : 0;
    unsigned i;

    s->dsi_cmdq_ready = false;
    if (n == 0 || n > 16) {
        return;
    }

    if (n >= 2 && ((w1 >> 8) & 0xff) == 0x06) {
        /* Read request: the DCS opcode sits in CMDQ[1] bits 23:16. */
        unsigned opcode = (w1 >> 16) & 0xff;
        uint32_t type = 0x21;   /* DCS short read response, no parameter */
        uint32_t data = 0;

        if (opcode == 0x0a) {   /* Get Power Mode */
            type = 0x22;        /* ... with one parameter */
            if (s->dsi_sleep_out && s->dsi_display_on) {
                data = 0x9c;    /* normal display on, sleep out */
            } else if (s->dsi_sleep_out) {
                data = 0x08;    /* sleep out only */
            }
        }
        s->dsi_rx[0] = type | (data << 8);
        s->dsi_rx[1] = 0;
        s->dsi_rx[2] = 0;
        s->dsi_rx[3] = 0;
        /* Status latches regardless of INTEN; the driver polls the bits. */
        s->dsi_regs[MTK_DSI_INTSTA >> 2] |=
            MTK_DSI_INT_LPRX_RD_RDY | MTK_DSI_INT_CMD_DONE;
    } else {
        /* Write commands: follow the panel power state. */
        for (i = 0; i < n; i++) {
            uint32_t w = mt6895_dsi_word(s, MTK_DSI_CMD_WORDS + 4 * i);
            unsigned type = (w >> 8) & 0xff;
            unsigned opcode = (w >> 16) & 0xff;

            if (type != 0x05 && type != 0x15) {
                continue;
            }
            switch (opcode) {
            case 0x10:          /* enter sleep mode */
                s->dsi_sleep_out = false;
                break;
            case 0x11:          /* exit sleep mode */
                s->dsi_sleep_out = true;
                break;
            case 0x28:          /* display off */
                s->dsi_display_on = false;
                break;
            case 0x29:          /* display on */
                s->dsi_display_on = true;
                break;
            default:
                break;
            }
        }
        s->dsi_regs[MTK_DSI_INTSTA >> 2] |= MTK_DSI_INT_CMD_DONE;
    }
    mtk_dsi_update_irq(s);

    if (mtk_dsi_trace) {
        fprintf(stderr, "MTK-DSICMD n=%u w0=%#x w1=%#x rx0=%#x "
                "sleep_out=%d display_on=%d\n", n, w0, w1,
                s->dsi_rx[0], s->dsi_sleep_out, s->dsi_display_on);
    }
}

static uint64_t mt6895_dsi_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895MachineState *s = opaque;
    uint64_t out = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        unsigned a = off + i;
        uint8_t b = a < sizeof(s->dsi_regs)
                        ? ((const uint8_t *)s->dsi_regs)[a] : 0;

        /*
         * INTSTA bit31 is DSI_BUSY.  The modelled engine is never busy and
         * mtk_dsi_poll_for_idle() polls this bit (also through a CMDQ POLL
         * while other packets run), so it always reads 0.
         */
        if (a == MTK_DSI_INTSTA + 3) {
            b &= 0x7f;
        }

        /* DSI_RX_DATA0..3 (0x74..0x83): latched DCS read response. */
        if (a >= MTK_DSI_RX_DATA && a < MTK_DSI_RX_DATA + 16) {
            unsigned w = (a - MTK_DSI_RX_DATA) >> 2;

            b = (s->dsi_rx[w] >> (8 * ((a - MTK_DSI_RX_DATA) & 3))) & 0xff;
        }
        out |= (uint64_t)b << (8 * i);
    }
    if (mtk_dsi_trace_all) {
        fprintf(stderr, "MTK-DSIRD off=%#" PRIx64 " val=%#" PRIx64 "\n",
                (uint64_t)off, out);
    }
    return out;
}

static void mt6895_dsi_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    Mt6895MachineState *s = opaque;
    bool hit_start = off < MTK_DSI_START + 4 && off + size > MTK_DSI_START;
    bool hit_inten = off < MTK_DSI_INTEN + 4 && off + size > MTK_DSI_INTEN;
    bool hit_intsta = off < MTK_DSI_INTSTA + 4 && off + size > MTK_DSI_INTSTA;
    bool hit_con = off < MTK_DSI_CON_CTRL + 4 &&
                   off + size > MTK_DSI_CON_CTRL;
    bool hit_mode = off < MTK_DSI_MODE_CTRL + 4 &&
                    off + size > MTK_DSI_MODE_CTRL;
    uint32_t old_start = s->dsi_regs[MTK_DSI_START >> 2];
    unsigned i;

    if (mtk_dsi_trace && (hit_start || hit_inten || hit_con || hit_mode)) {
        fprintf(stderr,
                "MTK-DSI: wr off=%#" PRIx64 " val=%#" PRIx64 " size=%u"
                " guest=%.3fs INTEN=%#x START=%#x MODE_CTRL=%#x CON_CTRL=%#x\n",
                (uint64_t)off, val, size, mtk_guest_seconds(),
                s->dsi_regs[MTK_DSI_INTEN >> 2],
                s->dsi_regs[MTK_DSI_START >> 2],
                s->dsi_regs[MTK_DSI_MODE_CTRL >> 2],
                s->dsi_regs[MTK_DSI_CON_CTRL >> 2]);
    } else if (mtk_dsi_trace_all) {
        fprintf(stderr, "MTK-DSIWR off=%#" PRIx64 " val=%#" PRIx64 " size=%u\n",
                (uint64_t)off, val, size);
    }

    if (hit_intsta) {
        /*
         * INTSTA is write-0-clears, not write-1-to-clear: the driver
         * acknowledges with writel(~status, INTSTA) and clears everything
         * with writel(0, INTSTA), so the new value is the AND of the old
         * value and the bytes actually written.
         */
        uint32_t covered = 0;

        for (i = 0; i < size; i++) {
            unsigned a = off + i;

            if (a >= MTK_DSI_INTSTA && a < MTK_DSI_INTSTA + 4) {
                covered |= 0xffu << (8 * (a - MTK_DSI_INTSTA));
            }
        }
        s->dsi_regs[MTK_DSI_INTSTA >> 2] =
            (s->dsi_regs[MTK_DSI_INTSTA >> 2] & ~covered) |
            (s->dsi_regs[MTK_DSI_INTSTA >> 2] & (uint32_t)val & covered);
        if (!(s->dsi_regs[MTK_DSI_INTSTA >> 2] &
              MTK_DSI_INT_SLEEPIN_ULPS_DONE)) {
            s->dsi_ulps_done = false;
        }
        mtk_dsi_update_irq(s);
        return;
    }

    /* Every other register is plain RW storage at byte granularity. */
    for (i = 0; i < size; i++) {
        unsigned a = off + i;

        if (a < sizeof(s->dsi_regs)) {
            ((uint8_t *)s->dsi_regs)[a] = (val >> (8 * i)) & 0xff;
        }
    }

    if (hit_start) {
        uint32_t start = s->dsi_regs[MTK_DSI_START >> 2];

        if ((start & MTK_DSI_START_VDO) && !(old_start & MTK_DSI_START_VDO)) {
            /*
             * START pulse.  A programmed command queue is transmitted (DCS
             * read/write, as LK's disp_lcm_init does); in command mode the
             * transfer is a single frame, never a continuous stream.
             */
            if (s->dsi_cmdq_ready) {
                mt6895_dsi_cmdq_run(s);
            }
            if (!(s->dsi_regs[MTK_DSI_MODE_CTRL >> 2] & MTK_DSI_MODE_MASK)) {
                s->dsi_regs[MTK_DSI_INTSTA >> 2] |= MTK_DSI_INT_CMD_DONE;
                mtk_dsi_update_irq(s);
            }
        }
        if (start & MTK_DSI_START_VM_CMD) {
            /* Video-mode command: the driver polls VM_CMD_DONE (bit5). */
            if (s->dsi_cmdq_ready) {
                mt6895_dsi_cmdq_run(s);
            }
            s->dsi_regs[MTK_DSI_INTSTA >> 2] |= MTK_DSI_INT_VM_CMD_DONE;
            mtk_dsi_update_irq(s);
        }
        if (start & MTK_DSI_START_SLEEPOUT) {
            /* Sleep-out completed: latch SLEEPOUT_DONE (bit6). */
            s->dsi_regs[MTK_DSI_INTSTA >> 2] |= MTK_DSI_INT_SLEEPOUT_DONE;
            mtk_dsi_update_irq(s);
        }
    }

    /* A command queue is "programmed" until the START that consumes it. */
    if (off < MTK_DSI_CMDQ_SIZE + 4 && off + size > MTK_DSI_CMDQ_SIZE &&
        (s->dsi_regs[MTK_DSI_CMDQ_SIZE >> 2] & 0x3f)) {
        s->dsi_cmdq_ready = true;
    }
    if (off < MTK_DSI_CMD_WORDS + 0x20 && off + size > MTK_DSI_CMD_WORDS) {
        s->dsi_cmdq_ready = true;
    }

    /* DSI_RACK bit0 acknowledges the DCS read: hardware drops RD_RDY. */
    if (off < MTK_DSI_RACK + 4 && off + size > MTK_DSI_RACK &&
        (s->dsi_regs[MTK_DSI_RACK >> 2] & 0x01)) {
        s->dsi_regs[MTK_DSI_INTSTA >> 2] &= ~MTK_DSI_INT_LPRX_RD_RDY;
        mtk_dsi_update_irq(s);
    }

    /*
     * ULPS entry: the PHY reports sleep-in done once both the clock lane
     * (PHY_LCCON 0x104) and the data lane (PHY_LD0CON 0x108) are parked.
     */
    if (off < 0x10c && off + size > 0x104 &&
        (s->dsi_regs[0x104 >> 2] & 0x02) &&
        (s->dsi_regs[0x108 >> 2] & 0x02) &&
        !(s->dsi_regs[MTK_DSI_INTSTA >> 2] &
          MTK_DSI_INT_SLEEPIN_ULPS_DONE)) {
        s->dsi_ulps_done = true;
        s->dsi_regs[MTK_DSI_INTSTA >> 2] |= MTK_DSI_INT_SLEEPIN_ULPS_DONE;
        mtk_dsi_update_irq(s);
    }

    /* START/CON_CTRL/MODE_CTRL decide whether the frame engine runs. */
    mtk_dsi_update_stream(s);
}

static const MemoryRegionOps mt6895_dsi_ops = {
    .read = mt6895_dsi_read,
    .write = mt6895_dsi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/*
 * LK display RDMA frame-done status latch.
 *
 * Evidence (, 2026-11-16, MTK_PCTIME PC+register wall-clock sampling of
 * qemu-.exe at MTK_ICOUNT=off): LK spends ~1.0 s in one burst inside the
 * display RDMA wait path, disassembled from parts/lk_a.img as
 *
 *   0x50f4c36c  wait_rdma(id, mask, timeout):
 *       combined = rdma_base(id) + 0x04;              // INT_STATUS
 *       for (i = 0; i < 10 * timeout; i++) {
 *           if (ldr [combined] & mask)                // poll
 *               break;
 *           udelay(100);                              // ldr cntpct loop
 *       }
 *       str ~mask -> [combined];                      // W1C acknowledge
 *
 * with rdma_base(0xa) = 0x14006000, rdma_base(0xf) = 0x1401c000 and
 * rdma_base(0x53) = 0x14406000; the 32 PC samples of that burst have
 * caller 0x50f4c3f0 (the udelay call inside this loop) and the surrounding
 * code is platform/mediatek/mt6895/disp/ddp_rdma.c.  The three windows are
 * plain mtk_syscon storage in the model, so INT_STATUS always reads 0 and the
 * wait always burns its full timeout -- 190 x 5 ms samples = ~0.95 s of wall
 * clock for a frame that the model completes instantly.
 *
 * Model: INT_STATUS latches when the guest programs the frame start
 * registers (GLOBAL_CON 0x10 .. MEM_CON 0x24), and the driver's W1C write
 * clears the acknowledged bits again.  No firmware value is fabricated: a
 * status bit only exists after the guest itself started a frame.
 * MTK_NO_RDMA_MODEL=1 reverts to the plain syscon windows; MTK_RDMADBG=1
 * traces the status register traffic.
 */
#define MT6895_RDMA_WIN_SIZE   0x1000
#define MT6895_RDMA_INT_STATUS 0x04

typedef struct Mt6895RdmaWin {
    uint8_t regs[MT6895_RDMA_WIN_SIZE];
    uint32_t int_status;
} Mt6895RdmaWin;

static Mt6895RdmaWin mt6895_rdma_wins[4];

static uint64_t mt6895_rdma_read(void *opaque, hwaddr off, unsigned size)
{
    Mt6895RdmaWin *w = opaque;
    uint64_t v = 0;
    unsigned i;

    if (off + size > MT6895_RDMA_WIN_SIZE) {
        return 0;
    }
    if (off <= MT6895_RDMA_INT_STATUS &&
        off + size > MT6895_RDMA_INT_STATUS) {
        if (MTK_ENV_CACHED("MTK_RDMADBG")) {
            fprintf(stderr, "RDMA-IS R %#x -> %#x\n",
                    (unsigned)off, w->int_status);
        }
        return w->int_status;
    }
    for (i = 0; i < size; i++) {
        v |= (uint64_t)w->regs[off + i] << (8 * i);
    }
    return v;
}

static void mt6895_rdma_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    Mt6895RdmaWin *w = opaque;
    unsigned i;

    if (off + size > MT6895_RDMA_WIN_SIZE) {
        return;
    }
    if (off <= MT6895_RDMA_INT_STATUS && off + size > MT6895_RDMA_INT_STATUS) {
        if (MTK_ENV_CACHED("MTK_RDMADBG")) {
            fprintf(stderr, "RDMA-IS W1C %#x (ack mask %#x)\n",
                    (uint32_t)val, ~(uint32_t)val);
        }
        w->int_status &= ~(uint32_t)val;
        return;
    }
    for (i = 0; i < size; i++) {
        w->regs[off + i] = (val >> (8 * i)) & 0xff;
    }
    /*
     * A programmed frame completes immediately in the model.  Only BIT(2) is
     * reported: the register sampling caught LK's wait loop at
     * 0x50f4c3dc with r19 (mask) = 0x4 and r20 (register) = 0x14006004, i.e.
     * RDMA0 INT_STATUS bit 2 (frame done).  Reporting every bit (an earlier
     * experiment) also set the underrun/error bits, which stalled the *kernel*
     * display driver, so the status word is kept to the one bit LK polls.
     */
    if (off <= 0x24 && off + size > 0x10) {
        w->int_status = 1u << 2;
    }
}

static const MemoryRegionOps mt6895_rdma_ops = {
    .read = mt6895_rdma_read,
    .write = mt6895_rdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void mt6895_init(MachineState *machine)
{
    mt6895_t0 = g_get_monotonic_time();
    mt6895_mark("enter");
    Mt6895MachineState *s = MT6895_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    Error *err = NULL;

    if (machine->ram_size < 256 * MiB) {
        error_report("mt6895: at least 256 MiB of RAM is required");
        exit(EXIT_FAILURE);
    }

    /*
     * The machine default is the genuine 6 GiB LPDDR5X part, and both the
     * DRAM controller registers and the BootROM's DRAM-size word are derived
     * from the configured RAM size.  -m 8G still selects the historical 8 GiB
     * LPDDR5 development part (so old runs stay bit-identical), but that is
     * not the unit this model reproduces; say so once so a stray 8 GiB run is
     * visible.
     */
    if (machine->ram_size != MT6895_DRAM_SIZE) {
        warn_report("mt6895: -m %" PRIu64 " MiB selects the historical 8 GiB "
                    "LPDDR5 part; the genuine unit is 6 GiB LPDDR5X "
                    "(use -m 6G)", machine->ram_size / MiB);
    }

    /*
     * DRAM is the machine's default RAM backend (mc->default_ram_id), so use
     * the region QEMU already created rather than allocating a second one.
     */
    s->ddr = machine->ram;
    memory_region_add_subregion(sysmem, MT6895_DRAM_BASE, s->ddr);

    /*
     * Mirror DRAM into the 0xffff0000_00000000 window the boot chain uses for
     * its own data pointers.  Static structures (the block-device list at
     * 0x62fca250 among them) are initialised with aliased self-pointers, so
     * the alias must resolve or every list walk reads unmapped memory.
     */
    memory_region_init_alias(&s->ddr_alias, OBJECT(s), "mt6895.ddr-alias",
                             s->ddr, 0, machine->ram_size);
    memory_region_add_subregion(sysmem,
                                MT6895_DRAM_ALIAS_BASE + MT6895_DRAM_BASE,
                                &s->ddr_alias);
    /* Real-hardware secure PA alias: 0x800000000 + PA views the same DRAM. */
    {
        static MemoryRegion dram_alias_mr;
        memory_region_init_alias(&dram_alias_mr, OBJECT(s),
                                 "mt6895.dram-secure-alias", s->ddr, 0,
                                 MACHINE(s)->ram_size);
        memory_region_add_subregion(sysmem,
            0x800000000 + MT6895_DRAM_BASE, &dram_alias_mr);
    }


    /*
     * GenieZone EMI domain remap: DRAM as the VMID1 domain addresses it, at
     * VMID1 << 35.  Off until the preloader enables it through SECURITY_AO
     * (see mt6895_gz_remap_write()).
     */
    memory_region_init_alias(&s->gz_ddr_remap, OBJECT(s), "mt6895.gz-ddr-remap",
                             s->ddr, 0, machine->ram_size);
    memory_region_set_enabled(&s->gz_ddr_remap, false);
    memory_region_add_subregion(sysmem,
                                MT6895_GZ_VMID1_OFFSET + MT6895_DRAM_BASE,
                                &s->gz_ddr_remap);
    memory_region_init_io(&s->gz_remap_mr, OBJECT(s), &mt6895_gz_remap_ops, s,
                          "mt6895.gz-remap", sizeof(s->gz_remap_regs));
    memory_region_add_subregion_overlap(sysmem, MT6895_GZ_REMAP_CTRL,
                                        &s->gz_remap_mr, 1);

    /*
     * WDT/RGU sits on top of the generic register-file window so the reset
     * request can be observed.
     */
    memory_region_init_io(&s->wdt, OBJECT(s), &mt6895_wdt_ops, s,
                          "mt6895.wdt-ctrl", 0x1000);
    memory_region_add_subregion_overlap(sysmem, 0x1c007000ULL, &s->wdt, 2);

    /* Calibration-marker read probe (transparent). */
    memory_region_init_io(&s->calmark, OBJECT(s), &mt6895_calmark_ops, s,
                          "mt6895.calmark", 4);
    memory_region_add_subregion_overlap(sysmem, 0x02075dc8ULL, &s->calmark, 3);

    for (int i = 0; i < MT6895_NUM_SRAMS; i++) {
        memory_region_init_ram(&s->sram[i], NULL, mt6895_sram[i].name,
                               mt6895_sram[i].size, &error_fatal);
        memory_region_add_subregion(sysmem, mt6895_sram[i].base, &s->sram[i]);
    }

    /* ADCC register block inside the SRAM0 window (see mt6895_adcc_init). */
    mt6895_adcc_init();
    /* GZ hw code register at 0x08000000 (see mt6895_hwcode_init). */
    mt6895_hwcode_init();

    /*
     * Initial state of the SCMI shared memory.
     *
     * In the arm,scmi shmem layout channel_status sits at +4 and bit0 is
     * CHANNEL_FREE.  On real silicon the platform side (tinysys) sets that bit
     * when the channel is idle; this model has no platform side, so unless it is
     * pre-set the kernel reads 0 from shmem_channel_free(), decides the channel
     * is busy and spins forever in shmem_tx_prepare() (observed PC stuck at
     * shmem_tx_prepare+0x28c with X09 always 0).
     *
     * With CHANNEL_FREE pre-set the first transaction goes out; the kernel then
     * times out waiting for the platform reply and lets probe fail instead of
     * hanging the whole boot chain.
     */
    for (int i = 2; i < MT6895_NUM_SRAMS; i++) {
        uint8_t *p = memory_region_get_ram_ptr(&s->sram[i]);

        stl_le_p(p + 4, 1);     /* channel_status: CHANNEL_FREE */
    }

    memory_region_init_ram(&s->boot_desc, NULL, "mt6895.boot-desc",
                           MT6895_BOOT_DESC_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, MT6895_BOOT_DESC_BASE, &s->boot_desc);

    /*
     * The hand-off stub sits at the reset vector. RVBAR is 0 on MT6895, so
     * every core starts executing it, but only the boot core is released.
     *
     * This is a real ROM memory region rather than rom_add_blob_fixed_as():
     * the latter only maps a region when the machine sets rom_file_has_mr,
     * and otherwise tries to copy the blob into memory that already exists
     * at that address -- of which there is none below the GIC.
     */
    memory_region_init_rom(&s->bootrom, NULL, "mt6895.bootrom",
                           MT6895_BOOTROM_SIZE, &error_fatal);
    memcpy(memory_region_get_ram_ptr(&s->bootrom), mt6895_bootrom_code,
           sizeof(mt6895_bootrom_code));
    /*
     * Preloader ROM-info page.
     *
     * LK's sec_func_init() walks virtual addresses starting at 0x1000 looking
     * for the "AND_ROMINFO_v" signature of the preloader image (the caller
     * loads w0 = 0x1000 and passes it straight down; the search advances in
     * 0x3000 steps up to 0x9D000).  On hardware that low window resolves to
     * the preloader image, whose signature lives at file offset 0x131C.
     * Expose the matching page here: physical 0x1000..0x2000 carries
     * preloader file bytes 0x1000..0x2000, so VA 0x131C lands exactly on the
     * signature.  The page is otherwise unused (BootROM is 0..0x1000 and the
     * boot descriptor starts at 0x2000).
     */
    if (s->preloader_path) {
        gsize plen = 0;
        gchar *pdata = NULL;
        GError *perr = NULL;

        if (g_file_get_contents(s->preloader_path, &pdata, &plen, &perr)) {
            if (plen >= MT6895_ROMPAGE_END) {
                memory_region_init_rom(&s->preloader_page, NULL,
                                       "mt6895.preloader-page",
                                       MT6895_ROMPAGE_SIZE, &error_fatal);
                memcpy(memory_region_get_ram_ptr(&s->preloader_page),
                       pdata + MT6895_ROMPAGE_BASE, MT6895_ROMPAGE_SIZE);
                memory_region_add_subregion(sysmem, MT6895_ROMPAGE_BASE,
                                            &s->preloader_page);
            }
            g_free(pdata);
        } else {
            error_report("mt6895: preloader page: %s", perr->message);
            g_error_free(perr);
        }
    }

    memcpy(memory_region_get_ram_ptr(&s->bootrom) + 0x100,
           mt6895_bdev_read_stub, sizeof(mt6895_bdev_read_stub));
    memory_region_add_subregion(sysmem, MT6895_BOOTROM_BASE, &s->bootrom);

    /*
     * EXPERIMENT: the payload's DRAM stage loads a service pointer from its
     * own literal pool and calls it; the value observed at the fault is
     * 0x8000, which our 0x1000-byte BootROM does not cover (the real MTK
     * BootROM is far larger, so 0x8000 is plausibly a BootROM service entry).
     * Map a window there holding an ARM "bx lr" so a call can return.
     */
    {
        static const uint32_t brom_svc_code[] = {
            0xe12fff1e,     /* bx lr (ARM: 0x8000 has bit0 clear) */
            0xe1a00000,     /* nop */
        };
        memory_region_init_rom(&s->brom_svc, NULL, "mt6895.brom-svc",
                               0x1000, &error_fatal);
        memcpy(memory_region_get_ram_ptr(&s->brom_svc), brom_svc_code,
               sizeof(brom_svc_code));
        memory_region_add_subregion(sysmem, 0x8000, &s->brom_svc);
    }

    /*
     * The BootROM argument words, mapped read-only over the payload's BSS so
     * the payload's own clear cannot erase them. See mt6895_brom_arg_read.
     */
    memory_region_init_io(&s->brom_arg, OBJECT(s), &mt6895_brom_arg_ops, s,
                          "mt6895.brom-arg", MT6895_BROM_ARG_SIZE);
    memory_region_add_subregion(sysmem, MT6895_BROM_ARG_BASE, &s->brom_arg);

    memory_region_init_io(&s->region_ptr, OBJECT(s), &mt6895_region_ptr_ops, s,
                          "mt6895.region-ptr", MT6895_REGION_PTR_SIZE);
    memory_region_add_subregion(sysmem, MT6895_REGION_PTR_BASE, &s->region_ptr);

    memory_region_init_io(&s->kinfo_flag, OBJECT(s), &mt6895_kinfo_flag_ops, s,
                          "mt6895.kinfo_flag", MT6895_KINFO_FLAG_SIZE);
    memory_region_add_subregion(sysmem, MT6895_KINFO_FLAG_BASE, &s->kinfo_flag);

    if (MTK_ENV_CACHED("MTK_CRYPTO_FLAG0")) {
        memory_region_init_io(&s->crypto_flag, OBJECT(s),
                              &mt6895_crypto_flag_ops, s,
                              "mt6895.crypto-flag", 8);
        memory_region_add_subregion(sysmem, 0x510a1b54, &s->crypto_flag);
    }

    memory_region_init_io(&s->lba_source, OBJECT(s), &mt6895_lba_source_ops, s,
                          "mt6895.lba_source", MT6895_LBA_SOURCE_SIZE);
    memory_region_add_subregion(sysmem, MT6895_LBA_SOURCE_BASE, &s->lba_source);

    memory_region_init_io(&s->dev_type, OBJECT(s), &mt6895_dev_type_ops, s,
                          "mt6895.dev-type", MT6895_DEV_TYPE_SIZE);
    memory_region_add_subregion(sysmem, MT6895_DEV_TYPE_BASE, &s->dev_type);

    memory_region_init_io(&s->dev_tbl, OBJECT(s), &mt6895_dev_tbl_ops, s,
                          "mt6895.dev-tbl", MT6895_DEV_TBL_SIZE);
    memory_region_add_subregion(sysmem, MT6895_DEV_TBL_BASE, &s->dev_tbl);

    memory_region_init_io(&s->plat_tbl, OBJECT(s), &mt6895_plat_tbl_ops, s,
                          "mt6895.plat-tbl", MT6895_PLAT_TBL_SIZE);
    memory_region_add_subregion(sysmem, MT6895_PLAT_TBL_BASE, &s->plat_tbl);

    memory_region_init_io(&s->devq, OBJECT(s), &mt6895_devq_ops, s,
                          "mt6895.devq", MT6895_DEVQ_SIZE);
    memory_region_add_subregion(sysmem, MT6895_DEVQ_BASE, &s->devq);

    memory_region_init_io(&s->boot_reason, OBJECT(s), &mt6895_boot_reason_ops,
                          s, "mt6895.boot-reason", MT6895_BOOT_REASON_SIZE);
    memory_region_add_subregion(sysmem, MT6895_BOOT_REASON_BASE,
                                &s->boot_reason);

    memory_region_init_io(&s->rkp_head, OBJECT(s), &mt6895_rkp_head_ops, s,
                          "mt6895.rkp-head", MT6895_RKP_HEAD_SIZE);
    memory_region_add_subregion(sysmem, MT6895_RKP_HEAD_BASE, &s->rkp_head);

    {
        static Mt6895Planted planted[8];
        static const uint8_t obj_def[0x40] = {
            [0x00] = MT6895_RKP_OBJ_TYPE,       /* type: 1 or 2 expected */
            [0x04] = MT6895_RKP_BLOCK_SIZE & 0xff,
            [0x05] = (MT6895_RKP_BLOCK_SIZE >> 8) & 0xff,   /* 512 */
            [0x14] = MT6895_DATA_ADDR & 0xff,
            [0x15] = (MT6895_DATA_ADDR >> 8) & 0xff,
            [0x16] = (MT6895_DATA_ADDR >> 16) & 0xff,
            [0x17] = (MT6895_DATA_ADDR >> 24) & 0xff,
            [0x20] = MT6895_RKP_STUB_ADDR & 0xff,
            [0x21] = (MT6895_RKP_STUB_ADDR >> 8) & 0xff,
            [0x22] = (MT6895_RKP_STUB_ADDR >> 16) & 0xff,
            [0x23] = (MT6895_RKP_STUB_ADDR >> 24) & 0xff,
            /* +0x24: write callback -> the DRAM write stub (see above). */
            [0x24] = MT6895_BDEV_WRITE_STUB & 0xff,
            [0x25] = (MT6895_BDEV_WRITE_STUB >> 8) & 0xff,
            [0x26] = (MT6895_BDEV_WRITE_STUB >> 16) & 0xff,
            [0x27] = (MT6895_BDEV_WRITE_STUB >> 24) & 0xff,
        };
        /*
         * The I/O callback the payload calls as its block read; r3 is the
         * destination buffer. The measured ABI is r1 = LBA, r2 = block count
         * (512-byte blocks).
         *
         * This callback is the payload's *entire* block device: the read
         * helper at 0x0200DF20 dispatches every read through [dev+0x20], and
         * that is this stub. Partition reads (bl2_ext at LBA 0x31C800, lk,
         * super, ...) therefore arrive here too, not just the GPT reads.
         * It must reproduce the two backing windows the machine builds:
         *
         *   offset < 0x4400 (LBA 0..33) -> MT6895_GPT_SOURCE 0x50000000:
         *       LBA 0 zero (protective MBR), LBA 1 GPT header,
         *       LBA 2..33 the 16 KiB partition entry array
         *   offset >= 0x4400 (LBA 34+)  -> MT6895_DATA_ADDR 0x80000000
         *       plus 512 * LBA: the data= image at real disk LBAs
         *
         * History: the stub used to ignore r1/r2 and always serve the same
         * window from 0x50000400. That is exactly why the entries CRC failed
         * (the firmware chains four consecutive 4 KiB windows of the entry
         * array and expects each at the buffer start), and it also broke
         * partition reads once the chain got past the GPT parse. An earlier
         * stub in this project had the full block semantics; restoring them
         * is what lets the chain continue past the GPT parse.
         *
         * The dst buffer is the payload's DMA bounce buffer (dev[+0x14],
         * which the machine points at 0x80000000); the payload copies out of
         * it after the callback returns. One consequence: for partition data
         * at byte offsets >= 2 GiB (the "mem" partition sits at LBA 0x14C8000
         * -> byte offset 0x299000000) a 32-bit guest cannot form the source
         * address at all. The old model read garbage there and carried on, so
         * this stub skips the copy in that case instead of faulting; the
         * caller then falls back to whatever the buffer already held, which
         * is the same degradation the old model had.
         *
         * 54 bytes, Thumb:
         *
         *   push {r4,lr}              mov  r4, r3
         *   lsls r1, r1, #9           lsls r2, r2, #9
         *   movw r3, #0x4400          cmp  r1, r3          ; GPT window end
         *   bhs  @data
         *   mov.w r0, #0x50000000     b    @copy
         * @data: mov.w r3, #0x80000000  cmp r1, r3
         *   bhs  @done                                  ; offset >= 2 GiB
         *   mov.w r0, #0x80000000
         * @copy: add  r0, r1         cbz  r2, @done
         *   loop: ldrb r1, [r0], #1   strb r1, [r4], #1
         *         subs r2, r2, #1     bne  loop
         * @done: movs r0, #0          pop  {r4, pc}
         *
         * MT6895_GPT_SOURCE must therefore hold the standard LBA layout:
         * header at +512, entry array at +1024 (see mt6895_place_gpt()).
         */
        /*
 * The payload's block-read callback, re-implemented for speed.
 *
 * It used to be a byte-at-a-time Thumb loop (ldrb/strb), which is what made
 * partition loads crawl: an 831 KiB load spent tens of seconds here because
 * every byte cost four guest instructions. This version runs in ARM state
 * and moves 24 bytes per iteration with ldm/stm, and it honours AAPCS by
 * saving r4-r11. Semantics are unchanged:
 *
 *   r1 = LBA, r2 = block count (512-byte blocks), r3 = destination
 *   offset < 0x4400  -> source is the GPT window at 0x50000000
 *   offset >= 2 GiB  -> nothing to copy (32-bit source arithmetic cannot
 *                       reach past the 2 GiB data window)
 *   otherwise        -> source is 0x80000000 + offset
 *
 * Returns 0 in r0.
 */
static const uint8_t stub_def[0x80] = {
            0xf0, 0x4f, 0x2d, 0xe9, 0x03, 0x40, 0xa0, 0xe1, 0x81, 0x14, 0xa0, 0xe1,
            0x82, 0x24, 0xa0, 0xe1, 0x44, 0x0c, 0x51, 0xe3, 0x01, 0x00, 0x00, 0x2a,
            0x54, 0x00, 0x9f, 0xe5, 0x00, 0x00, 0x00, 0xea, 0x50, 0x00, 0x9f, 0xe5,
            0x01, 0x00, 0x90, 0xe0, 0x4c, 0x50, 0x9f, 0xe5, 0x05, 0x00, 0x51, 0xe1,
            0x0d, 0x00, 0x00, 0x2a, 0x03, 0x00, 0x14, 0xe3, 0x05, 0x00, 0x00, 0x1a,
            0x18, 0x00, 0x52, 0xe3, 0x03, 0x00, 0x00, 0x3a, 0xe0, 0x07, 0xb0, 0xe8,
            0xe0, 0x07, 0xa4, 0xe8, 0x18, 0x20, 0x52, 0xe2, 0xf9, 0xff, 0xff, 0xea,
            0x00, 0x00, 0x52, 0xe3, 0x03, 0x00, 0x00, 0x0a, 0x01, 0x10, 0xd0, 0xe4,
            0x01, 0x10, 0xc4, 0xe4, 0x01, 0x20, 0x52, 0xe2, 0xfb, 0xff, 0xff, 0x1a,
            0x00, 0x00, 0xa0, 0xe3, 0xf0, 0x8f, 0xbd, 0xe8, 0x00, 0x00, 0x00, 0x50,
            0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80,
};

/*
 * Same callback, but paravirtualised the way the write stub already is:
 * latch the request into MT6895_BDEV_RCMD_BASE and let the machine move the
 * bytes.  This is what removes the 2 GiB ceiling of the guest-side copy
 * above -- the payload can then read partitions anywhere in the image,
 * including "mem" at byte offset 0x299000000, which is where its DRAM
 * description lives.  Without it that read returns zeros, so the payload
 * reports "dram info unset", retrains DRAM and resets on every boot.
 *
 *   push {r4-r11, lr}     ldr  r4, [pc, #0x34]   ; command register
 *   str  r1, [r4]         str  r2, [r4, #4]      ; LBA, block count
 *   str  r3, [r4, #8]     mov  r0, #0            ; destination, then trigger
 *   pop  {r4-r11, pc}     .word MT6895_BDEV_RCMD_BASE
 */
static const uint8_t stub_cmd_def[0x80] = {
    0xf0, 0x4f, 0x2d, 0xe9,     /* e92d4ff0 push {r4-r11, lr}      */
    0x34, 0x40, 0x9f, 0xe5,     /* e59f4034 ldr r4, [pc, #0x34]    */
    0x00, 0x10, 0x84, 0xe5,     /* e5841000 str r1, [r4]           */
    0x04, 0x20, 0x84, 0xe5,     /* e5842004 str r2, [r4, #4]       */
    0x08, 0x30, 0x84, 0xe5,     /* e5843008 str r3, [r4, #8]       */
    0x00, 0x00, 0xa0, 0xe3,     /* e3a00000 mov r0, #0             */
    0xf0, 0x8f, 0xbd, 0xe8,     /* e8bd8ff0 pop {r4-r11, pc}       */
    0x00, 0x00, 0xa0, 0xe1,     /* nop x9, up to the literal at 0x40 */
    0x00, 0x00, 0xa0, 0xe1,
    0x00, 0x00, 0xa0, 0xe1,
    0x00, 0x00, 0xa0, 0xe1,
    0x00, 0x00, 0xa0, 0xe1,
    0x00, 0x00, 0xa0, 0xe1,
    0x00, 0x00, 0xa0, 0xe1,
    0x00, 0x00, 0xa0, 0xe1,
    0x00, 0x00, 0xa0, 0xe1,
    0xa0, 0xc0, 0x07, 0x02,     /* .word 0x0207c0a0                */
};
       /*
         * The platform device object the bl2_ext offset path hands to
         * 0x02040B00 as its "dev" argument. That routine computes
         *     product = dev[+4] * part[+4]
         * and rejects the partition unless product reaches the caller's
         * budget; with dev[+4] left zero the product is zero and every
         * lookup fails with "fail to get bl2_ext part_offset".
         *
         * MT6895_DEV_OBJ_BASE sits inside the payload's BSS clear range
         * (0x020798A4 .. 0x0207C0A4), so the machine's plain writes there
         * are wiped before the lookup runs. Serve it from an overlay with a
         * valid block size and the ready flags the storage layer polls.
         */
        static const uint8_t dev_obj_def[0x60] = {
            [0x04] = MT6895_RKP_BLOCK_SIZE & 0xff,
            [0x05] = (MT6895_RKP_BLOCK_SIZE >> 8) & 0xff,   /* 512 */
            [0x18] = 0x00, [0x19] = 0x80,                   /* 0x8000 ready */
            [0x20] = 0x00, [0x21] = 0x80,
            [0x24] = 0x00, [0x25] = 0x80,
            [0x28] = 0x00, [0x29] = 0x80,
        };

        /*
         * EXPERIMENT (2026-11-02): security version structure.
         *
         * The IMG_VER/ARB path (0x02051952 -> 0x02050160) reads a version
         * structure at 0x020FB71C:
         *   {+0 loaded, +4 ok1, +8 ver, +0xc ok2, +0x10 ver_max}
         * and treats "not initialised" as "pl ver not available" -> ARB Fail
         * (0x104008) -> ASSERT platform.c:2010. In this model the structure
         * is never populated because its loader (a devinfo/efuse version
         * blob with magic "MMM" and 0xE291F358 sub-images) has no backing
         * store. This overlay emulates "version info present and current":
         * loaded=1, ok1=1, ver=0, ok2=1, ver_max=0.  Experiment only; see
         * docs/52 and vendor module analysis
         */
        static const uint8_t sec_ver_def[0x14] = {
            [0x00] = 0x01,                          /* loaded */
            [0x04] = 0x01,                          /* ok1 */
            /* [0x08..0x0b] ver = 0 */
            [0x0c] = 0x01,                          /* ok2 */
            /* [0x10..0x13] ver_max = 0 */
        };
        planted[0] = (Mt6895Planted){ s, MT6895_RKP_TABLE_BASE, 0x40, obj_def };
        planted[1] = (Mt6895Planted){ s, MT6895_RKP_STUB_ADDR & ~1ULL, 0x80,
                                      MTK_ENV_CACHED("MTK_LEGACY_RDSTUB")
                                          ? stub_def : stub_cmd_def };
        planted[2] = (Mt6895Planted){ s, MT6895_DEV_OBJ_BASE, 0x60,
                                      dev_obj_def };
        planted[3] = (Mt6895Planted){ s, 0x020fb71c, 0x14, sec_ver_def };
        planted[7] = (Mt6895Planted){ s,
                                      MT6895_BDEV_WRITE_STUB & ~1ULL,
                                      0x40,
                                      (const uint8_t *)mt6895_bdev_write_stub };

        /*
         * REMAINING INJECTION (B2): the boot-args struct at 0x020F7D30 is not
         * initialised by the real preloader/BL2 in this model; field +0x48
         * (0x020F7D78) is read as a pointer by the boot-to-LK path
         * (0x02049AF4) and dereferenced at 0x020497FE.  Kept until the
         * genuine boot-chain step that initialises it is modelled.
         */
        static const uint8_t lk_bootarg_def[4] = { 0x00, 0x00, 0x70, 0x50 };
        planted[4] = (Mt6895Planted){ s, 0x020f7d78, 4, lk_bootarg_def };


        /*
         * NOTE: 0x48253b50.. (ATF bss) live inside 12G DRAM, so a sysmem
         *     subregion at priority 0 does NOT reliably shadow DRAM reads:
         *     CPU still sees DRAM leftovers. Head/node/ack are therefore
         *     written directly to RAM in mt6895_machine_reset() below,
         *     not here. This planted slot is left as a zero-size trap.
         */
        static const char *const planted_names[8] = {
            "mt6895.rkp-obj", "mt6895.rkp-stub", "mt6895.dev-obj",
            "mt6895.sec-ver", "mt6895.lk-bootarg", "mt6895.tee-base",
            "mt6895.atf-list", "mt6895.bdev-write-stub",
        };

        if (MTK_ENV_CACHED("MTK_MEMINFO_AT")) {
            uint8_t *dp = mt6895_guest_ptr(s, 0x40000000ULL, 16);

            if (dp) {
                const char *e3 = getenv("MTK_MEMINFO");
                unsigned bi2 = 0;
                const char *q3 = e3;

                memset(dp, 0, 16);
                while (q3 && *q3 && bi2 < 11) {
                    char *end;

                    dp[bi2++] = (uint8_t)strtoul(q3, &end, 16);
                    q3 = (*end == ',') ? end + 1 : NULL;
                }
                fprintf(stderr, "mt6895: DRAM info pattern written at 0x40000000\n");
            }
        }

        if (!MTK_ENV_CACHED("MTK_BDEV_NOREGION")) {
            {
            static MemoryRegion dram_info;

            memory_region_init_io(&dram_info, OBJECT(s),
                                  &mt6895_dram_info_ops, s,
                                  "mt6895.dram-info", 0x40);
            memory_region_add_subregion_overlap(sysmem, MT6895_DRAM_INFO_ADDR,
                                                &dram_info, 3);
        }

        /*
         * Behavioural MIPI DSI controller at 0x14017000: answers LK's DCS
         * reads (DSI_RX_DATA0..3 + RD_RDY/CMD_DONE) and keeps the minimal
         * panel power state, so disp_lcm_init()'s Get Power Mode read sees a
         * live panel instead of zero and stops re-initialising it.  Installed
         * unconditionally; MTK_NO_DSI_MODEL=1 falls back to the plain
         * mtk_syscon "dev-dsi" register window.  See the comment above
         * mt6895_dsi_cmdq_run() for the register evidence.
         */
        if (!MTK_ENV_CACHED("MTK_NO_DSI_MODEL")) {
            static MemoryRegion dsi_mr;

            const char *hz_s = getenv("MTK_DISP_VBLANK_HZ");

            memset(s->dsi_regs, 0, sizeof(s->dsi_regs));
            memset(s->dsi_rx, 0, sizeof(s->dsi_rx));
            s->dsi_ulps_done = false;
            s->dsi_sleep_out = false;
            s->dsi_display_on = false;
            s->dsi_streaming = false;
            s->dsi_cmdq_ready = false;
            s->dsi_frame_count = 0;
            s->dsi_irq_count = 0;
            mtk_dsi_trace = MTK_ENV_CACHED("MTK_DSITRACE");
            mtk_dsi_trace_all = MTK_ENV_CACHED("MTK_DSITRACE_ALL");
            mtk_dsi_frame_period_ns = NANOSECONDS_PER_SECOND / 60;
            if (hz_s && *hz_s) {
                int64_t hz = strtoll(hz_s, NULL, 0);

                if (hz < 1) {
                    hz = 1;
                } else if (hz > 1000) {
                    hz = 1000;
                }
                mtk_dsi_frame_period_ns = NANOSECONDS_PER_SECOND / hz;
            }
            s->dsi_frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                              mtk_dsi_frame_tick, s);
            memory_region_init_io(&dsi_mr, OBJECT(s), &mt6895_dsi_ops, s,
                                  "mt6895.dsi", 0x1000);
            memory_region_add_subregion_overlap(sysmem, 0x14017000ULL,
                                                &dsi_mr, 3);
        }

        /*
         * LK's ddp_rdma frame-done wait polls RDMA+0x04 (INT_STATUS) with
         * udelay(100) and runs out a ~1 s timeout because these windows are
         * plain syscon storage.  Latch the status when the guest programs the
         * frame start registers so the wait completes at once.
         * MTK_NO_RDMA_MODEL=1 reverts to the syscon windows.
         */
        if (!MTK_ENV_CACHED("MTK_NO_RDMA_MODEL")) {
            static const hwaddr rdma_bases[] = {
                0x14006000ULL, 0x1401c000ULL, 0x14406000ULL, 0x1441c000ULL,
            };
            static MemoryRegion rdma_mr[ARRAY_SIZE(rdma_bases)];
            unsigned ri;

            memset(mt6895_rdma_wins, 0, sizeof(mt6895_rdma_wins));
            for (ri = 0; ri < ARRAY_SIZE(rdma_bases); ri++) {
                memory_region_init_io(&rdma_mr[ri], OBJECT(s),
                                      &mt6895_rdma_ops,
                                      &mt6895_rdma_wins[ri],
                                      "mt6895.disp-rdma",
                                      MT6895_RDMA_WIN_SIZE);
                memory_region_add_subregion_overlap(sysmem, rdma_bases[ri],
                                                    &rdma_mr[ri], 3);
            }
        }

        if (MTK_ENV_CACHED("MTK_LOGBUFPC")) {
            static MemoryRegion logbuf;

            memory_region_init_io(&logbuf, OBJECT(s), &mt6895_logbuf_ops,
                                  s, "mt6895.logbuf-probe", 0x1000);
            memory_region_add_subregion_overlap(sysmem, 0x7FFC0000ULL,
                                                &logbuf, 4);
        }

        memory_region_init_io(&s->bdev_cmd, OBJECT(s),
                                  &mt6895_bdev_cmd_ops, s,
                                  "mt6895.bdev-cmd", 0x10);
            memory_region_add_subregion_overlap(sysmem, MT6895_BDEV_CMD_BASE,
                                                &s->bdev_cmd, 2);
        memory_region_init_io(&s->bdev_rcmd, OBJECT(s),
                                  &mt6895_bdev_rcmd_ops, s,
                                  "mt6895.bdev-rcmd", 0x10);
            memory_region_add_subregion_overlap(sysmem, MT6895_BDEV_RCMD_BASE,
                                                &s->bdev_rcmd, 2);
        }

        for (int i = 0; i < 8; i++) {
            const MemoryRegionOps *ops = &mt6895_planted_ops;

            /*
             * MTK_NO_FAKE_DEVO: drop the planted block-device object and its
             * read/write stub, so the payload's storage lookups find nothing
             * and its own UFS driver has to come up the way real hardware
             * does.  Slots 1 (stub) and 2 (device object) are the pair that
             * together form the model's synthetic block device.
             */
            if (MTK_ENV_CACHED("MTK_NO_FAKE_DEVO") &&
                (i == 1 || i == 2 || i == 7)) {
                continue;
            }
            if (i == 7 && (!MTK_ENV_CACHED("MTK_BDEV_WRITE") ||
                           MTK_ENV_CACHED("MTK_BDEV_NOSTUB"))) {
                continue;
            }
            if (i == 5 || i == 6) {
                continue;
            }
            if (0) {
                continue;
            }
            memory_region_init_io(&s->planted[i], OBJECT(s),
                                  ops, &planted[i],
                                  planted_names[i],
                                  planted[i].size);
            memory_region_add_subregion(sysmem, planted[i].base, &s->planted[i]);
        }
        /*
         * ATF's UART object pointer.
         *
         * mtk_console_init loads it from 0x4825ac84 and passes it to the
         * console-node constructor, whose first check is "cbz x0 -> return 0"
         * (0x48204d98). That global is BSS and is normally filled in by ATF's
         * UART driver during cold boot, which this machine never runs -- so
         * it stays 0, the constructor fails, and mtk_console panics at
         * mtk_console.c:71. Hand it a scratch buffer inside ATF's own BSS so
         * the constructor can initialise a real object and the console comes
         * up.
         */
        memory_region_init_io(&s->atf_uart_mr, OBJECT(s), &mt6895_atf_uart_ops,
                              NULL, "mt6895.atf-uart", 4);
        memory_region_add_subregion(sysmem, 0x4825ac84ULL, &s->atf_uart_mr);
        memory_region_init_io(&s->atf_uartobj_mr, OBJECT(s),
                              &mt6895_atf_uartobj_ops, NULL,
                              "mt6895.atf-uartobj", 0x100);
        /*
         * Keep the fake UART object OUT of ATF's BSS: the translation tables
         * and the mmap array live there, and parking it at 0x4825d000 made
         * xlat report a bogus region (VA/PA 0x19020000) and panic. 0x48300000
         * is inside BL31's reserved window but beyond its BSS.
         */
        memory_region_add_subregion(sysmem, 0x4824a100ULL,
                                    &s->atf_uartobj_mr);
        /*
         * CPC/SMP power controller: the per-CPU power-state register (read by
         * ATF's mt_smp wait), the per-CPU power-on request register, the
         * per-CPU secondary entry points (written by ATF's CPU_ON path),
         * control/ack/bigcore registers (handshaked by ATF warm boot).
         * Priority 3 beats both the generic dcm-cpccfg-reg window
         * (0x0c53a800) and dcm-mp-cpusys-top (0x0c538000) that cover these
         * addresses. See mt6895_cpc_req_write() for the arm_set_cpu_on()
         * bridge. Default-ON; MTK_LEGACY_CPCOFF=1 preserves legacy behavior.
         */
        if (!MTK_ENV_CACHED("MTK_LEGACY_CPCOFF")) {
            static MemoryRegion cpc_ack_mr;
            static MemoryRegion cpc_ctrl_mr;
            static MemoryRegion cpc_bigcore_mr;
            static MemoryRegion cpc_req_mr;
            static MemoryRegion cpc_entry_mr;

            memory_region_init_io(&s->cpc_mr, OBJECT(s),
                                  &mt6895_cpc_status_ops, NULL,
                                  "mt6895.cpc-status", 4);
            memory_region_add_subregion_overlap(sysmem, MT6895_CPC_STATUS_ADDR,
                                                &s->cpc_mr, 3);

            memory_region_init_io(&cpc_ack_mr, OBJECT(s),
                                  &mt6895_cpc_ack_ops, NULL,
                                  "mt6895.cpc-ack", 4);
            memory_region_add_subregion_overlap(sysmem, MT6895_CPC_ACK_ADDR,
                                                &cpc_ack_mr, 3);

            memory_region_init_io(&cpc_ctrl_mr, OBJECT(s),
                                  &mt6895_cpc_ctrl_ops, NULL,
                                  "mt6895.cpc-ctrl", 4);
            memory_region_add_subregion_overlap(sysmem, MT6895_CPC_CTRL_ADDR,
                                                &cpc_ctrl_mr, 3);

            memory_region_init_io(&cpc_bigcore_mr, OBJECT(s),
                                  &mt6895_cpc_bigcore_ops, NULL,
                                  "mt6895.cpc-bigcore", 4);
            memory_region_add_subregion_overlap(sysmem, MT6895_CPC_BIGCORE_ADDR,
                                                &cpc_bigcore_mr, 3);

            memory_region_init_io(&cpc_req_mr, OBJECT(s),
                                  &mt6895_cpc_req_ops, NULL,
                                  "mt6895.cpc-req", 4);
            memory_region_add_subregion_overlap(sysmem, MT6895_CPC_REQ_ADDR,
                                                &cpc_req_mr, 3);

            memory_region_init_io(&cpc_entry_mr, OBJECT(s),
                                  &mt6895_cpc_entry_ops, NULL,
                                  "mt6895.cpc-entry",
                                  MT6895_CPC_ENTRY_COUNT * 8);
            memory_region_add_subregion_overlap(sysmem,
                                                MT6895_CPC_ENTRY_ADDR,
                                                &cpc_entry_mr, 3);
        } else {
            /*
             * Default: keep the pre-bridge footprint byte-for-byte -- only
             * the 4-byte power-state window, priority 1, answering all ones
             * (mt6895_cpc_status_read() does that while MTK_CPCON is off).
             * Not installing the request/entry overlays leaves those
             * addresses to the generic dcm-mp-cpusys-top syscon as before.
             */
            memory_region_init_io(&s->cpc_mr, OBJECT(s),
                                  &mt6895_cpc_status_ops, NULL,
                                  "mt6895.cpc", 4);
            memory_region_add_subregion_overlap(sysmem, MT6895_CPC_STATUS_ADDR,
                                                &s->cpc_mr, 1);
        }
        memory_region_init_io(&s->bt_mr, OBJECT(s), &mt6895_bt_ops, s,
                              "mt6895.bt-watch", 0x2000);
        if (MTK_ENV_CACHED("MTK_BAGWATCH")) {
            static MemoryRegion bag_blk_mr;
            static MemoryRegion bag_ba_lo_mr;
            static MemoryRegion bag_ba_hi_mr;

            mt6895_bag_blk.s = s;
            mt6895_bag_blk.base = MT6895_BAG_BLK_BASE;
            mt6895_bag_ba_lo.s = s;
            mt6895_bag_ba_lo.base = MT6895_BAG_BA_LO_BASE;
            mt6895_bag_ba_hi.s = s;
            mt6895_bag_ba_hi.base = MT6895_BAG_BA_HI_BASE;
            mt6895_bag_gl1.s = s;
            mt6895_bag_gl1.base = 0x020FA47CULL;
            mt6895_bag_gl2.s = s;
            mt6895_bag_gl2.base = 0x020FB718ULL;
            memory_region_init_io(&bag_blk_mr, OBJECT(s), &mt6895_bag_ops,
                                  &mt6895_bag_blk, "mt6895.bag-blk",
                                  MT6895_BAG_BLK_SIZE);
            memory_region_add_subregion_overlap(sysmem, MT6895_BAG_BLK_BASE,
                                                &bag_blk_mr, 4);
            memory_region_init_io(&bag_ba_lo_mr, OBJECT(s), &mt6895_bag_ops,
                                  &mt6895_bag_ba_lo, "mt6895.bag-ba-lo",
                                  MT6895_BAG_BA_LO_SIZE);
            memory_region_add_subregion_overlap(sysmem, MT6895_BAG_BA_LO_BASE,
                                                &bag_ba_lo_mr, 4);
            memory_region_init_io(&bag_ba_hi_mr, OBJECT(s), &mt6895_bag_ops,
                                  &mt6895_bag_ba_hi, "mt6895.bag-ba-hi",
                                  MT6895_BAG_BA_HI_SIZE);
            memory_region_add_subregion_overlap(sysmem, MT6895_BAG_BA_HI_BASE,
                                                &bag_ba_hi_mr, 4);
            {
                static MemoryRegion bag_gl1_mr;
                static MemoryRegion bag_gl2_mr;

                memory_region_init_io(&bag_gl1_mr, OBJECT(s), &mt6895_bag_ops,
                                      &mt6895_bag_gl1, "mt6895.bag-gl1",
                                      0x58);
                memory_region_add_subregion_overlap(sysmem, 0x020FA47CULL,
                                                    &bag_gl1_mr, 4);
                memory_region_init_io(&bag_gl2_mr, OBJECT(s), &mt6895_bag_ops,
                                      &mt6895_bag_gl2, "mt6895.bag-gl2",
                                      0x20);
                memory_region_add_subregion_overlap(sysmem, 0x020FB718ULL,
                                                    &bag_gl2_mr, 4);
            }
            fprintf(stderr, "mt6895: MTK_BAGWATCH active (plant-safe split: "
                    "block 0x93C, struct low 0x48 + high 0x124)\n");
        }
        memory_region_add_subregion_overlap(sysmem, 0x48600000ULL,
                                            &s->bt_mr, 1);

    }

    /*
     * The console base pointer. Four bytes only: 0x02079E00 is one word of a
     * region the payload memsets, and a wider overlay here would shadow SRAM1
     * it uses (the mistake recorded under MT6895_NAME_TABLE_SIZE).
     */
    memory_region_init_io(&s->conbase, OBJECT(s), &mt6895_conbase_ops, s,
                          "mt6895.conbase", MT6895_CONBASE_SIZE);
    memory_region_add_subregion(sysmem, MT6895_CONBASE_ADDR, &s->conbase);

    /*
     * The PMIC auxadc registry (payload 0x020F4BC8), the auxadc device object
     * (0x02078E94), the payload device table (0x020F48AC) and the PMIC ops
     * vtable (0x0207850C) used to be pre-filled with model-synthesised
     * structures because, with the SPMI/I2C buses unmodelled, the payload's
     * PMIC init built nothing and later walks dereferenced garbage.
     *
     * Now that the SPMI bus is modelled the payload builds these structures
     * itself, and the overlays were poisoning them (the regulator table came
     * out empty: "mtk_regulator_get failed.(0x7FFF)").  The overlays are gone
     * - this is a de-injection, not a switch: the payload's own data must be
     * what the guest sees.
     */

    memory_region_init_io(&s->loghdr, OBJECT(s), &mt6895_loghdr_ops, s,
                          "mt6895.pl-loghdr", 0x20);
    memory_region_add_subregion(sysmem, 0x0011df34, &s->loghdr);

    memory_region_init_io(&s->bcb, OBJECT(s), &mt6895_bcb_ops, s,
                          "mt6895.bootctrl-bcb", 0x30);
    memory_region_add_subregion(sysmem, 0x020f2310, &s->bcb);


    /*
     * MTK_NO_FAKE_BOOTDEV: leave the block-device list empty so the preloader's
     * own UFS driver has to register the device the way real hardware does.
     * The synthetic node below only exists to stop the rollback path from
     * dereferencing NULL; it also answers the partition-table lookup with a
     * placeholder read routine, which masks whether the real driver ran.
     */
    if (!MTK_ENV_CACHED("MTK_NO_FAKE_BOOTDEV")) {
        memory_region_init_io(&s->bootdev, OBJECT(s), &mt6895_bootdev_ops, s,
                              "mt6895.bootdev-head", 0x4);
        memory_region_add_subregion(sysmem, 0x020f130c, &s->bootdev);

        memory_region_init_io(&s->bootdev_node, OBJECT(s),
                              &mt6895_bootdev_node_ops, s,
                              "mt6895.bootdev-node", 0x30);
        memory_region_add_subregion(sysmem, 0x020f1310, &s->bootdev_node);
    }

    memory_region_init_io(&s->slottbl, OBJECT(s), &mt6895_slottbl_ops, s,
                          "mt6895.slot-table", MT6895_SLOTTBL_SIZE);
    memory_region_add_subregion(sysmem, MT6895_SLOTTBL_BASE, &s->slottbl);

    memory_region_init_io(&s->objw, OBJECT(s), &mt6895_objw_ops, s,
                          "mt6895.part-obj-w", 0x20);
    memory_region_add_subregion(sysmem, 0x020f4ac8, &s->objw);



    memory_region_init_io(&s->secpol, OBJECT(s), &mt6895_secpol_ops, s,
                          "mt6895.sec-policy", 0x1e8);
    memory_region_add_subregion(sysmem, 0x020793d8, &s->secpol);

    memory_region_init_io(&s->i2c_desc, OBJECT(s), &mt6895_i2c_desc_ops, s,
                          "mt6895.i2c-desc", 0xa0);
    memory_region_add_subregion(sysmem, 0x020f369c, &s->i2c_desc);

    memory_region_init_io(&s->name_tbl, OBJECT(s), &mt6895_name_tbl_ops, s,
                          "mt6895.name-tbl", MT6895_NAME_TABLE_SIZE);
    memory_region_add_subregion(sysmem, MT6895_NAME_TABLE_BASE, &s->name_tbl);

    /* Payload device table and PMIC ops vtable: see the de-injection note
     * above; the payload builds and maintains both itself. */

    {


        for (size_t i = 0; i < ARRAY_SIZE(mt6895_vtbl_default); i++) {
            s->rodata_store[i] = mt6895_vtbl_default[i];
        }
        memory_region_init_io(&s->rodata, OBJECT(s), &mt6895_rodata_ops, s,
                              "mt6895.rodata-vtbl", MT6895_RODATA_SIZE);
        memory_region_add_subregion(sysmem, MT6895_RODATA_ADDR, &s->rodata);
    }

    /*
     * NOTE: an MMIO name-table overlay at MT6895_NAME_TABLE_BASE was tried
     * and reverted: covering 0x02078EA4..0x02078FA4 cut the serial output
     * from 119141 bytes to 3217 (dead in the PMIF stage) because that range
     * is ordinary SRAM1 the payload itself uses.
     *
     * The synthetic name table itself, and the fabricated bl2_ext entry that
     * went with it, have since been removed outright. bl2_ext is not a
     * partition: it is the MMM container at lk_a+0xC48B0 (magic 0x58881688),
     * and the payload resolves it through its own container path. Verified:
     * with the table gone the tree still reaches recovery with the aid set,
     * and the containers still load.
     */

    memory_region_init_io(&s->logstore_win, OBJECT(s), &mt6895_logstore_ops,
                          s, "mt6895.logstore", MT6895_LOGSTORE_WIN_SIZE);
    memory_region_add_subregion(sysmem, MT6895_LOGSTORE_WIN_BASE,
                                &s->logstore_win);

    /*
     * MTK_NO_SYSTIRQ=1: legacy counter-only systimer (SYST_CON/SYST_VAL
     * writes dropped, no SPI 280).  Otherwise the mtk-systimer device is
     * created after the GIC (see "SYST clockevent" below).
     */
    if (MTK_ENV_CACHED("MTK_NO_SYSTIRQ")) {
        memory_region_init_io(&s->systimer, OBJECT(s), &mt6895_systimer_ops,
                              s, "mt6895.systimer", MT6895_SYSTIMER_SIZE);
        memory_region_add_subregion(sysmem, MT6895_SYSTIMER_BASE,
                                    &s->systimer);
    }

    /*
     * Keypad (kp@1c00e000, "mediatek,kp").  The real controller is modelled
     * by hw/misc/mtk_kpd.c; it needs the GIC's SPI line, so the device is
     * created below, after mt6895_create_gic().  MTK_NO_KPD=1 keeps this
     * plain register window for the whole run instead.
     */
    if (MTK_ENV_CACHED("MTK_NO_KPD")) {
        memory_region_init_io(&s->kp, OBJECT(s), &mt6895_kp_ops,
                              s, "mt6895.kp", MT6895_KP_SIZE);
        memory_region_add_subregion(sysmem, MT6895_KP_BASE, &s->kp);
    }

    memory_region_init_io(&s->srclken_rc, OBJECT(s), &mt6895_srclken_rc_ops,
                          s, "mt6895.srclken_rc", MT6895_SRCLKEN_RC_SIZE);
    memory_region_add_subregion(sysmem, MT6895_SRCLKEN_RC_BASE, &s->srclken_rc);

    memory_region_init_io(&s->dxcc_sec, OBJECT(s), &mt6895_dxcc_sec_ops,
                          s, "mt6895.dxcc_sec", MT6895_DXCC_SEC_SIZE);
    memory_region_add_subregion(sysmem, MT6895_DXCC_SEC_BASE, &s->dxcc_sec);

    memory_region_init_io(&s->masp, OBJECT(s), &mt6895_masp_ops,
                          s, "mt6895.masp", MT6895_MASP_SIZE);
    memory_region_add_subregion(sysmem, MT6895_MASP_BASE, &s->masp);

    for (int i = 0; i < 4; i++) {
        static const hwaddr dramc_top2_bases[4] = {
            0x10234000ULL, 0x10244000ULL, 0x10254000ULL, 0x10264000ULL
        };
        static const hwaddr dramc_cmd_bases[4] = {
            0x10230000ULL, 0x10240000ULL, 0x10250000ULL, 0x10260000ULL
        };
        char *name;

        s->dramc_top2_win[i].s = s;
        s->dramc_top2_win[i].ch = i;
        name = g_strdup_printf("mt6895.dramc-top2-%d", i);
        memory_region_init_io(&s->dramc_top2[i], OBJECT(s),
                              &mt6895_dramc_top2_ops, &s->dramc_top2_win[i],
                              name, MT6895_DRAMC_TOP2_SIZE);
        memory_region_add_subregion(sysmem, dramc_top2_bases[i], &s->dramc_top2[i]);

        s->dramc_cmd_win[i].s = s;
        s->dramc_cmd_win[i].ch = i;
        name = g_strdup_printf("mt6895.dramc-cmd-%d", i);
        memory_region_init_io(&s->dramc_cmd[i], OBJECT(s),
                              &mt6895_dramc_cmd_ops, &s->dramc_cmd_win[i],
                              name, MT6895_DRAMC_CMD_SIZE);
        memory_region_add_subregion(sysmem, dramc_cmd_bases[i], &s->dramc_cmd[i]);
    }

    /*
     * The three DEVAPC windows, in the order the preloader's HRE table at
     * 0x02069c64 lists them: INFRA_DEVAPC, INFRA_DEVAPC1, PERI_DEVAPC.
     */
    {
        static const struct { hwaddr base; const char *name; } devapc[] = {
            { 0x1c3b1000ULL, "mt6895.dev-infra-devapc"  },
            { 0x1c3b2000ULL, "mt6895.dev-infra-devapc1" },
            { 0x1c3b0000ULL, "mt6895.dev-peri-devapc"   },
        };

        for (size_t i = 0; i < ARRAY_SIZE(devapc); i++) {
            memory_region_init_io(&s->devapc[i], OBJECT(s), &mt6895_devapc_ops,
                                  &s->devapc_win[i], devapc[i].name,
                                  MT6895_DEVAPC_SIZE);
            memory_region_add_subregion(sysmem, devapc[i].base, &s->devapc[i]);
        }
    }


    for (int i = 0; i < 4; i++) {
        static const hwaddr dramc_top3_bases[4] = {
            0x10236000ULL, 0x10246000ULL, 0x10256000ULL, 0x10266000ULL
        };
        char *name = g_strdup_printf("mt6895.dramc-top3-%d", i);
        s->dramc_top3_win[i].s = s;
        s->dramc_top3_win[i].ch = i;
        memory_region_init_io(&s->dramc_top3[i], OBJECT(s),
                              &mt6895_dramc_top3_ops, &s->dramc_top3_win[i],
                              name, MT6895_DRAMC_TOP3_SIZE);
        memory_region_add_subregion(sysmem, dramc_top3_bases[i], &s->dramc_top3[i]);
    }

    memory_region_init_io(&s->topckgen_clk, OBJECT(s), &mt6895_topckgen_ops,
                          s, "mt6895.topckgen_clk", MT6895_TOPCKGEN_SIZE);
    memory_region_add_subregion(sysmem, 0x10000000ULL, &s->topckgen_clk);

    /*
     * Lay down the SRAM log header the ROM kernel's log_store module looks
     * for. It reads the address from the "logstore" device tree node and
     * validates two magic words; without them the module never returns from
     * its init, which blocks every later module load.
     */
    {
        uint8_t *rom = memory_region_get_ram_ptr(&s->bootrom);
        /*
         * The payload names the window it reads in its own log:
         * "PL_LOG_STORE: check once, sig value 0x%x, addr 0x120000".
         * The header it validates sits there, not at the smaller window the
         * BootROM mapping had assumed, so the signatures and the buffer sizes
         * are seeded at 0x120000 and the loghdr block is placed at +0x34,
         * which is the offset the PL module's own check reads.
         */
        uint32_t hdr[64] = {
            [0x00 / 4] = MT6895_LOGSTORE_SIG0,   /* +0x00 signature */
            [0x04 / 4] = 0x1000,                 /* +0x04 size      */
            [0x08 / 4] = 0x800,                  /* +0x08 pl size   */
            [0x0c / 4] = MT6895_LOGSTORE_SIG1,   /* +0x0c signature */
            [0x10 / 4] = 0x800,                  /* +0x10 lk size   */
            [0x14 / 4] = 0,                      /* +0x14 last step */
            /* 0x0200E522 validates this block: signature, buffer size, and
             * the slot priorities get_suffix reads afterwards. */
            [0x34 / 4] = 0xcdab3412,             /* +0x34 signature */
            [0x38 / 4] = 0x00040000,             /* +0x38 buf size  */
            [0x3c / 4] = 0x00000400,             /* +0x3c           */
            [0x40 / 4] = 0x00000000,             /* +0x40 pl_flag   */
            [0x44 / 4] = 0x00000000,             /* +0x44 off_pl    */
            [0x48 / 4] = 0x00000400,             /* +0x48 sz_lk     */
            [0x4c / 4] = 0x00000000,             /* +0x4c lk_flag   */
            [0x50 / 4] = 0x00000080,             /* +0x50 off_lk    */
        };

    /*
         * Place it at the window base -- which is exactly what the device
         * tree's log_store property carries. A trace of the window shows the
         * module reading offset 0 first (that is the signature check); the
         * 0x34 it adds to its __ioremap() result addresses a later field, not
         * this header.
         */
        if (MT6895_LOGSTORE_WIN_BASE + sizeof(hdr) <= MT6895_BOOTROM_SIZE) {
            memcpy(rom + MT6895_LOGSTORE_WIN_BASE, hdr, sizeof(hdr));
        }
    }

    /*
     * Two mutually exclusive bring-up paths share this machine:
     *
     *   -kernel <Image>   direct kernel boot: skip the MTK boot chain and
     *                     start the ROM's own kernel. Used to reach Android
     *                     userspace without depending on the preloader.
     *   -machine preloader=<boot LUN>
     *                     the BootROM path: load the preloader and hand over.
     *
     * They are additive; neither replaces the other.
     */
    s->direct_boot = machine->kernel_filename != NULL;

    if (!s->direct_boot) {
        mt6895_load_boot_lun(s, &err, false);
        if (err) {
            fprintf(stderr, "MT6895-INIT-FAIL boot_lun: %s\n",
                    error_get_pretty(err));
        }
        if (!err) {
            mt6895_place_gpt(s, &err);
        }
        if (err) {
            fprintf(stderr, "MT6895-INIT-FAIL place_gpt: %s\n",
                    error_get_pretty(err));
        }
        if (!err) {
            mt6895_place_data(s, &err);
        }
        if (err) {
            fprintf(stderr, "MT6895-INIT-FAIL place_data: %s\n",
                    error_get_pretty(err));
        }
        /*
         * No dynamic injection anywhere in this model: every stage must find
         * what it needs through the devices and memory the way real hardware
         * would. Anything that pokes firmware data structures is removed --
         * it can mask missing load steps and make later stages diverge.
         */

        memory_region_init_io(&s->own_mr, OBJECT(s), &mt6895_own_ops, s,
                              "mt6895.own", 0x80);
        memory_region_add_subregion_overlap(sysmem, 0x4825b67cULL,
                                            &s->own_mr, 1);
        memory_region_init_io(&s->cflag_mr, OBJECT(s), &mt6895_cflag_ops, s,
                              "mt6895.cflag", 4);
        memory_region_add_subregion_overlap(sysmem, 0x510a1b54ULL,
                                            &s->cflag_mr, 1);
        if (MTK_ENV_CACHED("MTK_CODEGEN")) {
            static QEMUTimer *ct1, *ct2;

            ct1 = timer_new_ns(QEMU_CLOCK_VIRTUAL, (QEMUTimerCB *)mt6895_dump_codegen, s);
            ct2 = timer_new_ns(QEMU_CLOCK_VIRTUAL, (QEMUTimerCB *)mt6895_dump_codegen, s);
            if (ct1) {
                timer_mod(ct1, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                          (int64_t)8000000000LL);
            }
            if (ct2) {
                timer_mod(ct2, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                          (int64_t)10000000000LL);
            }
        }
        if (MTK_ENV_CACHED("MTK_BL2LIST")) {
            static QEMUTimer *ltm[8];
            int k;

            for (k = 0; k < 8; k++) {
                ltm[k] = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                      (QEMUTimerCB *)mt6895_dump_bl2_list, s);
                if (ltm[k]) {
                    timer_mod(ltm[k], qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                              (int64_t)((k + 12) * 1000000000LL));
                }
            }
        }
        /* Put the block-write stub into free DRAM (the payload's DRAM use
         * ends at 0x48080000) and let dev[+0x24] point at it.  DRAM is real
         * RAM, so no overlay sits inside the payload's BSS clear range. */
        mt6895_place_bdev_stub(s);

        if (MTK_ENV_CACHED("MTK_UFS_GATE")) {
            s->bdev_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                         mt6895_set_ufs_gate, s);
            {
                const char *ge = getenv("MTK_UFS_GATE_MS");
                int64_t gms = ge ? atoi(ge) : 100;

                timer_mod(s->bdev_timer,
                          qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                          + gms * 1000000LL);
            }
        }
        if (mt6895_scan_diag_wanted()) {
            s->scan_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                         mt6895_clear_crypto_mb, s);
            timer_mod(s->scan_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 7000000000LL);
        }

        mt6895_place_device_table(s);
        /*
         * APUSYS peri window: ATF's SMC handler stores per-op records at
         * 0x10355004 + op*0x20 (low/high addr and value), and LK polls the
         * same window. Back it with RAM so the writes survive and LK sees
         * the status it waits for.
         */
        {
            static MemoryRegion apusys_peri_mr;

            memory_region_init_ram(&apusys_peri_mr, NULL,
                                   "mt6895.apusys_peri", 0x1000, &error_abort);
            memory_region_add_subregion_overlap(sysmem, 0x10355000ULL,
                                                &apusys_peri_mr, 2);
        }
        /*
         * ATF reads a context pointer from 0x4825a110 (rgu_impl) and
         * dereferences +0x18 / +0x10. On real hardware platform setup fills
         * it in (mtk_rgu_driver_init -> mtk_rgu_driver_register). Default to
         * a zeroed scratch page until BL31's driver init writes &rgu_impl_mt6895
         * so early panics see NULL callbacks, but preserve guest writes so
         * rgu_sw_reset works at PSCI SYSTEM_RESET.
         */
        if (MTK_ENV_CACHED("MTK_DIAG_G8CC")) {
            memory_region_init_io(&s->diagg_mr, OBJECT(s), &mt6895_diagg_ops, s,
                                  "mt6895.diagg", 4);
            memory_region_add_subregion_overlap(sysmem, 0x4825a8ccULL,
                                                &s->diagg_mr, 3);
        }
        memory_region_init_io(&s->atfflag_mr, OBJECT(s), &mt6895_atfflag_ops,
                              s, "mt6895.atfflag", 4);
        memory_region_add_subregion_overlap(sysmem, 0x4825abd8ULL,
                                            &s->atfflag_mr, 2);
        if (MTK_ENV_CACHED("MTK_BL33_64BIT")) {
            memory_region_init_io(&s->bl33flag_mr, OBJECT(s),
                                  &mt6895_bl33flag_ops, s,
                                  "mt6895.bl33flag", 1);
            memory_region_add_subregion_overlap(sysmem, 0x4825b823ULL,
                                                &s->bl33flag_mr, 3);
        }
        memory_region_init_io(&s->atfctx_mr, OBJECT(s), &mt6895_atfctx_ops,
                              s, "mt6895.atfctx", 8);
        memory_region_add_subregion_overlap(sysmem, 0x4825a110ULL,
                                            &s->atfctx_mr, 2);
        mt6895_place_boot_part_names(s);
        if (err) {
            error_report_err(err);
            exit(EXIT_FAILURE);
        }
        mt6895_write_boot_desc(s);
    }

    /*
     * Order matters: the GIC binds each redistributor to a CPU object at
     * realize time, so the CPUs have to exist first. The UARTs need the GIC
     * to connect their interrupt lines, so they come last.
     */
    /*
     * MTK_PSCI_HOST=1: QEMU handles the guest's PSCI (CPU_ON etc.) instead of
     * ATF, and this machine's non-secure world runs at EL1 (ATF only ever
     * enters EL1), so secondaries have to be started at EL1 as well.
     */
    /*
     * MTK_DT_HVC_PSCI=1 uses the same "non-secure world runs at EL1" rule,
     * but the switch is made in the dtb (/psci/method hvc) instead of
     * intercepting the guest's SMCs, so ATF keeps serving every non-PSCI
     * SMC (MTK SiP calls) exactly as before.
     */
    if (mt6895_host_psci_mode(machine) ||
        MTK_ENV_CACHED("MTK_DT_HVC_PSCI")) {
        arm_psci_set_prefer_el1(true);
        /*
         * Cores that QEMU starts directly never execute ATF's warm-boot entry,
         * so their EL3 per-CPU state is left at reset.  The first SiP SMC
         * issued on such a core (e.g. the hwrng read behind sec-rng.ko, SMC
         * 0x26a, from a kworker that can land on any CPU) then faults inside
         * ATF and wedges EL3 -- which is what made init stall at
         * /lib/modules/sec-rng.ko with 3+ CPUs.  Plant the ATF warm-boot
         * state from the CPU_ON hook, in the target vCPU's thread.
         */
        if (MACHINE(s)->smp.cpus > 1 &&
            MTK_ENV_CACHED("MTK_ATF_PERCPU_PLANT")) {
            arm_psci_set_secondary_hook(mtk_secondary_atf_init);
            arm_psci_set_eager_plant(true);
        }
        if (MACHINE(s)->smp.cpus > 1 &&
            (MTK_ENV_CACHED("MTK_ATF_LAZY_PLANT") ||
             (mt6895_host_psci_mode(machine) &&
              !MTK_ENV_CACHED("MTK_NO_ATF_PLANT")))) {
            arm_psci_set_secondary_hook(mtk_secondary_atf_init);
            arm_psci_set_lazy_plant(true);
        }
        /*
         * Secondary CPUs started by QEMU never execute ATF's warm-boot entry;
         * a SiP SMC from them would fault in EL3 and wedge the system.  Until
         * the EL3 per-CPU state is fully reproduced, answer vendor SMCs from
         * non-boot cores with NOT_SUPPORTED so drivers fail gracefully.
         * MTK_SEC_SMC_PASSTHROUGH=1 restores the old behaviour.
         */
        if (MTK_ENV_CACHED("MTK_SEC_SMC_SKIP")) {
            arm_psci_set_skip_secondary_smc(true);
        }
        if (MTK_ENV_CACHED("MTK_SEC_SMC_DEBUG")) {
            arm_psci_set_smc_skip_debug(true);
        }
    }
    mt6895_possible_cpu_arch_ids(machine);

    mt6895_create_cpus(s, &err);
    if (err) {
        error_report_err(err);
        exit(EXIT_FAILURE);
    }

    mt6895_create_gic(s, &err);
    if (err) {
        error_report_err(err);
        exit(EXIT_FAILURE);
    }

    mt6895_create_its(s, &err);
    if (err) {
        error_report_err(err);
        exit(EXIT_FAILURE);
    }

    /*
     * SYST clockevent (DT /soc/systimer@1c011000, "mediatek,mt6765-timer",
     * GIC_SPI 280 level-high).  The kernel uses it as the tick broadcast
     * device when the arch timer stops in PSCI idle (local-timer-stop).
     */
    if (!MTK_ENV_CACHED("MTK_NO_SYSTIRQ")) {
        DeviceState *syst = qdev_new(TYPE_MTK_SYSTIMER);

        qdev_prop_set_uint32(syst, "size", MT6895_SYSTIMER_SIZE);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(syst), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(syst), 0, MT6895_SYSTIMER_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(syst), 0,
                           qdev_get_gpio_in(s->gic, 280));
    }

    mt6895_create_pcie(s, &err);
    if (err) {
        error_report_err(err);
        exit(EXIT_FAILURE);
    }

    mt6895_create_uarts(s, &err);
    if (err) {
        error_report_err(err);
        exit(EXIT_FAILURE);
    }

    mt6895_mark("before syscons");
    mt6895_create_syscons(s, &err);
    mt6895_mark("after syscons");
    /*
     * MTCMOS power-domain register files.
     *
     * LK's clkmgr power-on/power-down flows drive a request register and then
     * spin on the matching status bit until the PMIC/SPM acknowledges.  The
     * blocks below are plain register files on hardware (writes stick, the
     * state machine sets the status bits), so model them as RAM and pre-set
     * the acknowledgement bits the flows wait for.
     *
     *  0x1c001e00 block (VLP/XPU power control):
     *      +0x12c  status/protect word  (bits 0,1 cleared by the pdn flow)
     *      +0x134  power-state ACK      (bit 0 = domain reports powered)
     *  0x10001c00 block (infracfg_ao MTCMOS control):
     *      +0x04c  bit 28  ready
     *      +0x05c  bit 9   ready
     *      +0x06c  bits 16,17 ready
     */
    {
        static MemoryRegion vlp_pwr_mr;
        static MemoryRegion infra_pwr_mr;
        uint8_t *ram;

        /*
         * 0x1c001e00 is the SPM PWR_CON/PWR_STATUS half of the scpsys
         * window (ctls at 0xE00..0xEE8, status at 0xF34/0xF38).  The real
         * mtk-scpsys PWR_CON/PWR_STATUS state machine owns that window by
         * default (the vendor kernel's scpsys driver polls its SRAM sleep
         * acknowledge and per-domain status bits).  The old constant-ack
         * overlay is kept only for MTK_LEGACY_PWR_STATE=1 A/B runs.
         */
        if (MTK_ENV_CACHED("MTK_LEGACY_PWR_STATE")) {
            static MemoryRegion vlp_mr;

            memory_region_init_io(&vlp_mr, OBJECT(s), &mt6895_vlp_pwr_ops,
                                  s, "mt6895.vlp_pwr", 0x1000);
            memory_region_add_subregion_overlap(sysmem, 0x1c001e00ULL,
                                                &vlp_mr, 2);
        }
        if (0) {

        memory_region_init_ram(&vlp_pwr_mr, NULL, "mt6895.vlp_pwr",
                               0x1000, &error_abort);
        ram = memory_region_get_ram_ptr(&vlp_pwr_mr);
        /*
         * Power-domain control/status block.  Bits 30/31 of a power-control
         * word are the "state reached" acknowledge the clkmgr flows spin on;
         * the status words at +0x134/+0x138 carry the per-domain ready bits.
         * The flows run in sequence, so report every domain already settled.
         */
        stl_le_p(ram + 0x018, 0xc0000000u);   /* domain ctrl: ack bits 30/31 */
        stl_le_p(ram + 0x01c, 0xc0000000u);
        stl_le_p(ram + 0x020, 0xc0000000u);
        stl_le_p(ram + 0x070, 0xc0000000u);
        stl_le_p(ram + 0x134, 0x10000001u);   /* bit0 powered, bit28 ready  */
        stl_le_p(ram + 0x138, 0x10000040u);   /* bit6  ready,   bit28 ready */
        memory_region_add_subregion_overlap(sysmem, 0x1c001e00ULL,
                                            &vlp_pwr_mr, 2);
        }

        /*
         * VLP PLL/ack block the clkmgr flows poll at 0x1c00c210/0x1c00c220.
         * The 0x1c00c000 window is served by mtk-syscon's VLP bus-protection
         * state machine by default; this constant-ack overlay is legacy-only
         * (MTK_LEGACY_PWR_STATE=1, MTK_NO_VLPPLL=1 still disables it).
         */
        if (!MTK_ENV_CACHED("MTK_NO_VLPPLL") &&
            MTK_ENV_CACHED("MTK_LEGACY_PWR_STATE")) {
            static MemoryRegion vlp_pll_mr;
            uint8_t *p2;

            memory_region_init_ram(&vlp_pll_mr, NULL, "mt6895.vlp_pll",
                                   0x1000, &error_abort);
            p2 = memory_region_get_ram_ptr(&vlp_pll_mr);
            stl_le_p(p2 + 0x220, 0x00003c00u);   /* bits 10..13 ack           */
            memory_region_add_subregion_overlap(sysmem, 0x1c00c000ULL,
                                                &vlp_pll_mr, 2);
        }

        if (!MTK_ENV_CACHED("MTK_NO_INFRAPWR") &&
            MTK_ENV_CACHED("MTK_LEGACY_PWR_STATE")) {
        memory_region_init_ram(&infra_pwr_mr, NULL, "mt6895.infra_pwr",
                               0x100, &error_abort);
        ram = memory_region_get_ram_ptr(&infra_pwr_mr);
        stl_le_p(ram + 0x4c, 0x10000000u);    /* bit 28 ready               */
        stl_le_p(ram + 0x5c, 0x00000200u);    /* bit 9  ready               */
        stl_le_p(ram + 0x6c, 0x00030000u);    /* bits 16,17 ready           */
        memory_region_add_subregion_overlap(sysmem, 0x10001c00ULL,
                                            &infra_pwr_mr, 2);
        }
        if (!MTK_ENV_CACHED("MTK_NO_READY") &&
            MTK_ENV_CACHED("MTK_LEGACY_PWR_STATE")) {
            static MemoryRegion ready_mr;

            memory_region_init_io(&ready_mr, OBJECT(s),
                                  &mt6895_infra_ready_ops, s,
                                  "mt6895.infra_ready", 0x10);
            memory_region_add_subregion_overlap(sysmem, 0x10001c5cULL,
                                                &ready_mr, 5);
        }
    }
    /*
     * Modem-side window at 0x20000000.
     *
     * ATF's SMC path writes 0x20000024 while programming the modem's
     * bus-protection state and then reads the word back.  With no region
     * behind it the write raised a synchronous external abort
     * (ESR_EL3 = 0x96000050, FAR_EL3 = 0x20000024), which ATF turned into a
     * panic right after the modem power-on completed.  Back it with RAM.
     */
    {
        static MemoryRegion mdwin_mr;

        memory_region_init_ram(&mdwin_mr, NULL, "mt6895.md_window",
                               0x1000000, &error_abort);
        memory_region_add_subregion_overlap(sysmem, 0x20000000ULL,
                                            &mdwin_mr, 2);
    }
    /*
     * ADSP loader command/ack words (see mt6895_adsp_ack_ops).  Two 4-byte
     * windows only, so the ADSP image at 0x7a000000 stays real DRAM.
     */
    {
        static MemoryRegion adsp_cmd_mr;
        static MemoryRegion adsp_ack_mr;

        memory_region_init_io(&adsp_cmd_mr, OBJECT(s), &mt6895_adsp_cmd_ops,
                              s, "mt6895.adsp_cmd", 4);
        memory_region_add_subregion_overlap(sysmem, MT6895_ADSP_CMD_ADDR,
                                            &adsp_cmd_mr, 5);
        memory_region_init_io(&adsp_ack_mr, OBJECT(s), &mt6895_adsp_ack_ops,
                              s, "mt6895.adsp_ack", 4);
        memory_region_add_subregion_overlap(sysmem, MT6895_ADSP_ACK_ADDR,
                                            &adsp_ack_mr, 5);
    }
    /*
     * ADSP firmware landing zone.
     *
     * LK stages the audio_dsp_a sections at LK virtual addresses
     * 0xffff0000_03220000 .. 0xffff0000_0443a000; on this part the low 32
     * bits are the physical address (LK's identity window), so the images
     * land in 0x03220000..0x04440000.  Back that span with RAM so the staging
     * copies and the subsequent adsp_image_checker comparison see the data.
     */
    {
        static MemoryRegion adsp_stage_mr;

        memory_region_init_ram(&adsp_stage_mr, NULL, "mt6895.adsp_stage",
                               0x01300000, &error_abort);
        memory_region_add_subregion_overlap(sysmem, 0x03200000ULL,
                                            &adsp_stage_mr, 2);
    }
    if (MTK_ENV_CACHED("MTK_SPY")) {
        static MemoryRegion spy_mr;

        memory_region_init_io(&spy_mr, OBJECT(s), &mt6895_spy_ops, s,
                              "mt6895.spy", 0x100000000ULL);
        memory_region_add_subregion_overlap(sysmem, 0, &spy_mr, -1000);
    }
    /*
     * ADSP core0 SRAM view (0x1e050000).
     *
     * LK stages the audio_dsp_a sections into its own DRAM staging window
     * (LK VA 0xffff0000_04420000 and up) and then, through this core window,
     * checks that the firmware really landed in ADSP SRAM: adsp_image_checker
     * walks core0's SRAM view word by word against the staged copy.  Hardware
     * has the ADSP loader push the staged image into SRAM, so both sides read
     * the same bytes.  The model cannot model the DSP core itself, so this
     * window resolves the staging VA through the guest's own page tables and
     * serves the same bytes -- the window behaves as the SRAM would once the
     * staged image has been fetched.
     */
    {
        static MemoryRegion adsp_sram_mr[ARRAY_SIZE(mt6895_adsp_windows)];
        unsigned i;

        for (i = 0; i < ARRAY_SIZE(mt6895_adsp_windows); i++) {
            mt6895_adsp_win_ctx[i].stage_va =
                mt6895_adsp_windows[i].stage_va;
            memory_region_init_io(&adsp_sram_mr[i], OBJECT(s),
                                  &mt6895_adsp_sram_ops,
                                  &mt6895_adsp_win_ctx[i],
                                  "mt6895.adsp_core0_sram", 0x9000);
            memory_region_add_subregion_overlap(
                sysmem, mt6895_adsp_windows[i].base, &adsp_sram_mr[i], 6);
            fprintf(stderr, "ADSPWIN reg base=%#" HWADDR_PRIx
                    " stage=%#" PRIx64 "\n",
                    mt6895_adsp_windows[i].base,
                    mt6895_adsp_windows[i].stage_va);
        }
    }
    {
        static MemoryRegion gpueb_mbox_mr;

        memory_region_init_io(&gpueb_mbox_mr, OBJECT(s),
                              &mt6895_gpueb_mbox_ops, s,
                              "mt6895.gpueb_mbox", MT6895_GPUEB_MBOX_SIZE);
        memory_region_add_subregion_overlap(sysmem, MT6895_GPUEB_MBOX_BASE,
                                            &gpueb_mbox_mr, 4);
    }
    {
        static MemoryRegion spm_req_mr;

        memory_region_init_io(&spm_req_mr, OBJECT(s), &mt6895_spm_req_ops, s,
                              "mt6895.spm_req", MT6895_SPM_REQ_SIZE);
        memory_region_add_subregion_overlap(sysmem, MT6895_SPM_REQ_BASE,
                                            &spm_req_mr, 5);
    }
    {
        static MemoryRegion pdn_mr;

        memory_region_init_io(&pdn_mr, OBJECT(s), &mt6895_pdn_ops, s,
                              "mt6895.pdn", MT6895_PDN_SIZE);
        memory_region_add_subregion_overlap(sysmem, MT6895_PDN_BASE,
                                            &pdn_mr, 5);
    }
    mt6895_create_i2c(s, &err);
    mt6895_mark("after i2c");
    if (err) {
        error_report_err(err);
        exit(EXIT_FAILURE);
    }

    mt6895_create_msdc(s, &err);
    mt6895_mark("after msdc");
    if (err) {
        error_report_err(err);
        exit(EXIT_FAILURE);
    }

    /*
     * Keypad (kp@1c00e000, "mediatek,kp", GIC SPI 106 edge-rising).  The
     * controller is modelled by hw/misc/mtk_kpd.c: five active-low 16-bit
     * key-state words at +0x04..+0x14, an interrupt on every key-state
     * change, and a QEMU input handler so "sendkey volumedown" / a VNC
     * keyboard reaches the guest.  Created after the SPMI arbiter so the
     * power key can be linked to the PMIC key block: on this board the power
     * key is wired to the MT6363, not to the keypad matrix.
     * MTK_NO_KPD=1 keeps the plain register window installed earlier.
     */
    if (!MTK_ENV_CACHED("MTK_NO_KPD")) {
        DeviceState *kpd = qdev_new("mtk-kpd");

        /*
         * The main PMIC (SPMI slave 4) carries the phone's power key. The
         * keypad model forwards power key edges to it; its PWRKEY/PWRKEY_R
         * interrupts then reach the CPU through the PMIC RCS line, which is
         * EINT pin 243 (see the mtk-eint device created with the SPMI
         * arbiter).
         */
        object_property_set_link(OBJECT(kpd), "pmic", OBJECT(s->spmi),
                                 &error_abort);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(kpd), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(kpd), 0, MT6895_KP_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(kpd), 0,
                           qdev_get_gpio_in(s->gic, MT6895_KP_SPI));
    }

    if (MTK_ENV_CACHED("MTK_PCSAMPLE") || MTK_ENV_CACHED("MTK_PCTIME")) {
        QEMUTimer *t = timer_new_ms(QEMU_CLOCK_VIRTUAL, mt6895_pc_sample,
                                    NULL);

        t->opaque = t;
        timer_mod(t, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) + 1);
    }
    if (s->direct_boot) {
        s->binfo.ram_size = machine->ram_size;
        s->binfo.loader_start = MT6895_DRAM_BASE;
        s->binfo.psci_conduit = mt6895_host_psci_mode(machine)
                                    ? QEMU_PSCI_CONDUIT_SMC
                                    : QEMU_PSCI_CONDUIT_HVC;
        arm_load_kernel(ARM_CPU(first_cpu), machine, &s->binfo);
    }
}

static char *mt6895_get_preloader(Object *obj, Error **errp)
{
    Mt6895MachineState *s = MT6895_MACHINE(obj);

    return g_strdup(s->preloader_path);
}

static void mt6895_set_preloader(Object *obj, const char *value, Error **errp)
{
    Mt6895MachineState *s = MT6895_MACHINE(obj);

    g_free(s->preloader_path);
    s->preloader_path = g_strdup(value);
}

static void mt6895_get_entry_bytes(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    Mt6895MachineState *s = MT6895_MACHINE(obj);
    uint32_t value = s->entry_bytes;

    visit_type_uint32(v, name, &value, errp);
}

static void mt6895_set_entry_bytes(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    Mt6895MachineState *s = MT6895_MACHINE(obj);
    uint32_t value = 0;

    if (!visit_type_uint32(v, name, &value, errp)) {
        return;
    }
    s->entry_bytes = value;
}

static void mt6895_get_stage_entry_patch(Object *obj, Visitor *v,
                                          const char *name, void *opaque,
                                          Error **errp)
{
    Mt6895MachineState *s = MT6895_MACHINE(obj);
    uint32_t value = s->stage_entry_patch;

    visit_type_uint32(v, name, &value, errp);
}

static void mt6895_set_stage_entry_patch(Object *obj, Visitor *v,
                                         const char *name, void *opaque,
                                         Error **errp)
{
    Mt6895MachineState *s = MT6895_MACHINE(obj);
    uint32_t value = 0;

    if (!visit_type_uint32(v, name, &value, errp)) {
        return;
    }
    s->stage_entry_patch = value ? 1 : 0;
}

static void mt6895_get_stage_arg(Object *obj, Visitor *v, const char *name,
                                 void *opaque, Error **errp)
{
    Mt6895MachineState *s = MT6895_MACHINE(obj);
    uint32_t value = s->stage_arg;

    visit_type_uint32(v, name, &value, errp);
}

static void mt6895_set_stage_arg(Object *obj, Visitor *v, const char *name,
                                 void *opaque, Error **errp)
{
    Mt6895MachineState *s = MT6895_MACHINE(obj);
    uint32_t value = 0;

    if (!visit_type_uint32(v, name, &value, errp)) {
        return;
    }
    s->stage_arg = value;
}

static char *mt6895_get_data(Object *obj, Error **errp)
{
    Mt6895MachineState *s = MT6895_MACHINE(obj);

    return g_strdup(s->data_path);
}

static void mt6895_set_data(Object *obj, const char *value, Error **errp)
{
    Mt6895MachineState *s = MT6895_MACHINE(obj);

    g_free(s->data_path);
    s->data_path = g_strdup(value);
}

static char *mt6895_get_gpt(Object *obj, Error **errp)
{
    Mt6895MachineState *s = MT6895_MACHINE(obj);

    return g_strdup(s->gpt_path);
}

static void mt6895_set_gpt(Object *obj, const char *value, Error **errp)
{
    Mt6895MachineState *s = MT6895_MACHINE(obj);

    g_free(s->gpt_path);
    s->gpt_path = g_strdup(value);
}

static void mt6895_machine_instance_init(Object *obj)
{
    Mt6895MachineState *s = MT6895_MACHINE(obj);

    object_property_add_str(obj, "preloader", mt6895_get_preloader,
                            mt6895_set_preloader);
    object_property_set_description(obj, "preloader",
        "Path to a UFS boot LUN image (e.g. LUA_0) for the BootROM to start");

    object_property_add(obj, "entry-bytes", "uint32",
                        mt6895_get_entry_bytes, mt6895_set_entry_bytes,
                        NULL, NULL);
    object_property_add(obj, "entry-patch", "uint32",
                        mt6895_get_stage_entry_patch,
                        mt6895_set_stage_entry_patch, NULL, NULL);
    object_property_set_description(obj, "entry-patch",
        "1 (the default) redirects the DRAM stage's jump to the function's "
        "first instruction, as this machine has always done. 0 keeps the "
        "value the image itself carries (0x020360B9), which enters the shared "
        "tail instead of the prologue -- useful for re-testing that decision "
        "now that the other machine-side gaps are closed.");

    object_property_add(obj, "stage-arg", "uint32",
                        mt6895_get_stage_arg, mt6895_set_stage_arg,
                        NULL, NULL);
    object_property_set_description(obj, "stage-arg",
        "WORKAROUND: value to store at image offset 0x2EC, the literal the SRAM "
        "stage loads into the DRAM stage's second argument. 0 (the default) "
        "leaves the image untouched; a 4 KB-aligned value such as 0x02001000 "
        "lets the entry's alignment test pass so the work path can be observed.");
    object_property_set_description(obj, "entry-bytes",
        "How many bytes of the partition-entry array the payload's block-I/O "
        "callback copies after the 92-byte header (default: all 16384)");

    object_property_add_str(obj, "data", mt6895_get_data, mt6895_set_data);
    object_property_set_description(obj, "data",
        "Image to place in DRAM and point the block-I/O object's data-source "
        "field (+0x14) at");

    object_property_add_str(obj, "gpt", mt6895_get_gpt, mt6895_set_gpt);
    object_property_set_description(obj, "gpt",
        "Path to the device's GPT (header + 128 entries, taken verbatim from "
        "the storage image) to place in the buffer the payload's block-I/O "
        "callback is handed");

    object_property_add_uint64_ptr(obj, "boot-r4", &s->boot_r4,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint64_ptr(obj, "boot-lr", &s->boot_lr,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "boot-r4",
        "Value the BootROM hands to the payload in r4. The payload's first "
        "instruction stores it, and the DRAM-mode stage entered via the "
        "SRAM build's literal pool dereferences it as a code pointer.");

    s->preloader_path = NULL;
    s->gpt_path = NULL;
    s->entry_bytes = MT6895_GPT_ENTRIES * MT6895_GPT_ENTRY_SIZE;
    s->stage_arg = 0;
    s->stage_entry_patch = 1;
    s->boot_lr = 0;
    s->data_path = NULL;
}

static void mt6895_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "MediaTek MT6895 (Dimensity 8100 / 8000)";
    mc->init = mt6895_init;
    mc->max_cpus = MT6895_NUM_CPUS;
    mc->default_cpus = MT6895_NUM_CPUS;
    /* Default to the genuine 6 GiB LPDDR5X part (see MT6895_DRAM_SIZE). */
    mc->default_ram_size = 6 * GiB;
    mc->default_ram_id = "mt6895.ddr";
    mc->possible_cpu_arch_ids = mt6895_possible_cpu_arch_ids;
    mc->reset = mt6895_machine_reset;
}

static const TypeInfo mt6895_machine_typeinfo = {
    .name = TYPE_MT6895_MACHINE,
    .parent = TYPE_MACHINE,
    .class_init = mt6895_machine_class_init,
    .instance_init = mt6895_machine_instance_init,
    .instance_size = sizeof(Mt6895MachineState),
    /*
     * QEMU 11 enumerates and accepts machines through a per-target QOM
     * interface rather than through TYPE_MACHINE alone; without this the
     * machine registers but is not reachable from -M.
     */
    .interfaces = aarch64_machine_interfaces,
};

static void mt6895_machine_register_types(void)
{
    type_register_static(&mt6895_machine_typeinfo);
}

type_init(mt6895_machine_register_types);
