#include "qemu/timer.h"
/*
 * MediaTek MT6895 (Dimensity 8100 / 8000) machine definitions
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_MT6895_H
#define HW_ARM_MT6895_H

#include "hw/arm/boot.h"
#include "hw/core/boards.h"
#include "hw/core/qdev.h"
#include "qom/object.h"

#define TYPE_MT6895_MACHINE MACHINE_TYPE_NAME("mt6895")
OBJECT_DECLARE_SIMPLE_TYPE(Mt6895MachineState, MT6895_MACHINE)

/* Clusters, as described by the MT6895 device tree. */
#define MT6895_NUM_A55 4
#define MT6895_NUM_A78 4
#define MT6895_NUM_CPUS (MT6895_NUM_A55 + MT6895_NUM_A78)

/*
 * Physical map. Taken from the vendor device tree
 * (arch/arm64/boot/dts/vendor/mediatek/mt6895.dts of the Xiaomi/OnePlus
 * MT6895 kernel trees) and cross-checked against the mainline MT6895 port.
 */
#define MT6895_DRAM_BASE        0x40000000ULL
/*
 * The boot chain builds 64-bit pointers of the form
 * 0xffff0000_00000000 | physical_address for its own data.  That window is a
 * plain alias of DRAM on the real part, so partition lists and other static
 * structures hold aliased self-pointers that the loader dereferences while it
 * walks them.  Without the alias every walk stops on an unmapped read and the
 * bootloader reports "partition does not exist".
 */
#define MT6895_DRAM_ALIAS_BASE  0xffff000000000000ULL

#define MT6895_GICD_BASE        0x0c000000ULL
#define MT6895_GICD_SIZE        0x00040000ULL
#define MT6895_GICR_BASE        0x0c040000ULL
#define MT6895_GICR_SIZE        0x00200000ULL

#define MT6895_UART0_BASE       0x11001000ULL
#define MT6895_UART1_BASE       0x11002000ULL

/* The console base pointer slot, four bytes, overlaid (see mt6895_conbase_ops). */
#define MT6895_CONBASE_ADDR      0x02079e00ULL
#define MT6895_CONBASE_SIZE      0x00000004ULL

/*
 * A read-only vtable array that lives in the payload image's .data, at
 * 0x02078DE8..0x02078E08 -- just the function-pointer table itself.
 * The payload's own BSS clear covers it (the same memset that starts at
 * 0x02078EA4 for 0xA00 bytes starts 0xBC earlier here), which zeroes every
 * function pointer: the vibrator dispatch at
 * 0x02040014 then does "ldr r3,[obj+8]; ldr r3,[r3]; bx r3" with slot 0
 * zero and jumps to 0x04, where the CPU loops on undefined instructions.
 * On hardware the clear runs before .data is copied out of flash; here the
 * image is placed straight into RAM, so the region is served from an
 * overlay holding the image's own bytes.
 */
#define MT6895_RODATA_ADDR       0x02078de8ULL
#define MT6895_RODATA_SIZE       0x00000024ULL

/* Store capacity for the RKP planted overlays (each region is <= 0x40). */
#define MT6895_PLANTED_WORDS     64
#define MT6895_UART_SIZE        0x00001000ULL
/*
 * The values below are SPI numbers as they appear in the MT6895 device tree,
 * and QEMU's GICv3 GPIO input lines are laid out as [0..N-1] SPI followed by
 * the per-CPU PPI blocks, where N = num-irq - GIC_INTERNAL
 * (arm_gicv3_common.c). With num-irq = MT6895_NUM_SPIS + 32 that means:
 *
 *     line  0 .. 959   SPI 0 .. 959        (INTID = line + 32)
 *     line 960 .. 991  CPU0 PPIs           (INTID = line - 960 + 16)
 *     line 992 .. 1023 CPU1 PPIs
 *     ...
 *
 * So an SPI connection passes the SPI number unchanged, while a PPI
 * connection passes MT6895_NUM_SPIS + 32 * cpu + (INTID - 16).
 *
 * An earlier version of this comment claimed the PPI blocks began at line 0,
 * which is only true for a GIC with 32 SPIs. Acting on it put every timer PPI
 * on a line the distributor reads as an SPI, and no timer interrupt was ever
 * delivered.
 */
#define MT6895_UART0_SPI        141
#define MT6895_UART1_SPI        142

/*
 * Number of SPIs the interrupt controller exposes. The MT6895 device tree
 * references SPIs up to 775, so 960 is ample; it is also the largest
 * multiple of 32 that keeps the total INTID count (SPIs + 32 SGIs/PPIs)
 * within the GICv3 architectural limit of 1020.
 */
#define MT6895_NUM_SPIS         960

/*
 * On-chip SRAM windows.
 *
 * SRAM0 is where the preloader is loaded and runs: DRAM has not been
 * initialised yet at that point, and bringing it up is precisely what the
 * preloader does. The boot LUN header names 0x00100000 as the load address.
 *
 * SRAM1 was discovered by running the firmware: the preloader's very first
 * basic block loads a pointer from its literal pool and stores through it,
 * and that pointer is 0x02001000. No driver in the MT6895 device tree claims
 * anything in 0x02000000..0x02100000, which is consistent with a further
 * on-chip SRAM bank rather than a peripheral register block. Its exact size
 * is not documented publicly; 1 MiB is provisionally modelled and should be
 * corrected if the firmware ever addresses beyond it.
 */
#define MT6895_SRAM0_BASE       0x00100000ULL

#define MT6895_SRAM0_SIZE       0x00400000ULL   /* 4 MiB */

/*
 * AEE ram_console window inside SRAM0.
 *
 * The preloader sets this up (it prints "ram_console info sram_addr: 0x11D000 /
 * sram_size: 0x800 / def_type: 0x2 / memory_info_offset: 0xEC0"), so the AEE
 * memory-info block lives at 0x11D000 + 0xEC0 = 0x11DEC0 and is exactly what
 * BL2_EXT validates ("addr" tag at +0, "size" tag at +0x3C) before it hands
 * over.  BL2_EXT then puts its own log store at 0x11DF00 (0x100 bytes), so the
 * whole 4 KiB page from 0x11D000 belongs to this non-volatile SRAM area.
 */
#define MT6895_AEE_CONSOLE_BASE 0x0011D000ULL
#define MT6895_AEE_CONSOLE_SIZE 0x00001000ULL
#define MT6895_SRAM1_BASE       0x02000000ULL
#define MT6895_SRAM1_SIZE       0x00200000ULL   /* 2 MiB */

/*
 * Boot argument block. The preloader's SRAM stage writes the BROM-supplied
 * value here and then passes this address as the first argument to the
 * DRAM-mode payload, matching MTK's
 * jump(addr, BOOT_ARGUMENT_ADDR, sizeof(boot_arg_t)).
 */
#define MT6895_BOOT_ARG_BASE    0x02001000ULL
#define MT6895_BOOT_ARG_SIZE    0x00001000ULL

/*
 * SCMI shared memory (ssram1/ssram2).
 *
 * arm,scmi talks to tinysys through a mailbox and both sides exchange message
 * headers, status words and payloads over these two SRAM blocks.  They must be
 * mapped as writable memory rather than as a register file: the kernel writes
 * the CHANNEL_FREE bit of channel_status (plus the message header and length)
 * and the platform side reads it and writes it back.  When mapped as a plain
 * register file the reads always return 0, shmem_channel_free() considers the
 * channel busy forever and shmem_tx_prepare() spins on ldr/dmb (observed PC
 * stuck at shmem_tx_prepare+0x28c).
 *
 * The device tree gives a length of 0x80; map 4 KiB-aligned to leave headroom.
 */
#define MT6895_SCMI_TX_SHMEM_BASE   0x1c350000ULL
#define MT6895_SCMI_RX_SHMEM_BASE   0x1c360000ULL
#define MT6895_SCMI_SHMEM_SIZE      0x00001000ULL

#define MT6895_NUM_SRAMS        4

/*
 * Register blocks the boot chain touches whose behaviour is not needed to
 * make progress. Each entry is instantiated as an mtk-syscon register file
 * at its real address, so anything else stays visibly unmodelled.
 *
 * systimer@1c011000 -- "mediatek,mt6895-timer": registers as a clockevent
 * device next to the architected timer; its shutdown path writes a control
 * register, and that write must land somewhere rather than fault.
 */
#define MT6895_SYSTIMER_BASE    0x1c011000ULL
#define MT6895_SYSTIMER_SIZE    0x00001000

/*
 * watchdog@1c007000 -- "mediatek,mt6895-wdt". MTK's aee_hangdet module reads
 * the watchdog's stage register at module init; without a model that read
 * faults and takes the kernel down before init gets anywhere.
 */
/*
 * dvfsrc@1c00f000 -- "mediatek,mt6895-dvfsrc". The MTK DVFS resource
 * controller backs the vcore regulator and the interconnect providers the
 * MMC driver asks for; its query path reads this block directly.
 */
#define MT6895_DVFSRC_BASE      0x1c00f000ULL
#define MT6895_DVFSRC_SIZE      0x00001000

/*
 * spmi@1c804000 -- "mediatek,mt6895-spmi". Two windows: the PMIF register
 * block and the SPMI master block. The SPMI core is built into the ROM
 * kernel (spmi_controller_alloc/add are present), so only the MTK host
 * controller has to exist for the bus to come up.
 */
/*
 * scpsys@1c001000 -- "mediatek,mt6895-scpsys". Gates the SoC power domains
 * (mtcmos); UFS needs its ufs0_shutdown domain from here.
 */
/*
 * ufshci@112b0000 -- "mediatek,mt8183-ufshci". The UFS controller; its
 * registers must answer before ufshcd-mtk will probe.
 */
/* Model-owned block-device command register (see the write stub). */
#define MT6895_BDEV_CMD_BASE    0x0207c080ULL
#define MT6895_BDEV_WRITE_STUB  0x49000000ULL
/*
 * Model-owned block-device *read* command register.
 *
 * The read callback used to be guest code that copied from a memory window,
 * which caps every transfer at 2 GiB of image offset: a 32-bit guest cannot
 * form the source address of anything above that, and the "mem" partition
 * (LBA 0x14C8000 -> byte 0x299000000) sits far above it.  The result was that
 * the payload's DRAM-description read came back as zeros, so every boot
 * retrained DRAM and reset.
 *
 * The stub now only latches (LBA, block count, destination) here and the
 * machine performs the transfer, exactly like the write stub above.  That
 * removes the 32-bit ceiling and keeps reads coherent with the writes the
 * payload stores in the same image.
 */
#define MT6895_BDEV_RCMD_BASE   0x0207c0a0ULL

#define MT6895_UFSHCI_BASE    0x112b0000ULL
#define MT6895_UFSHCI_SIZE      0x00002300
#define MT6895_UFSHCI_SPI       137     /* dt: interrupts = <0 0x89 4 0> */

/*
 * tinysys mailbox, the transport SCMI uses on this SoC (the device tree gives
 * scmi a mbox plus two shared-memory windows rather than an SMC conduit).
 */
#define MT6895_TINYSYS_MBOX_BASE0   0x1c351000ULL
#define MT6895_TINYSYS_MBOX_BASE1   0x1c361000ULL
#define MT6895_TINYSYS_MBOX_SIZE    0x00001000
#define MT6895_TINYSYS_MBOX_SPI0    0x123   /* dt: interrupts = <0 0x123 4 0> */
#define MT6895_TINYSYS_MBOX_SPI1    0x124

/* SCMI shared-memory windows (dt: ssram1@1C350000 / ssram2@1C360000) */
#define MT6895_SCMI_SHMEM_TX_BASE   0x1c350000ULL
#define MT6895_SCMI_SHMEM_RX_BASE   0x1c360000ULL

/* ADSP subsystem CFG, CFG2, and 4-channel mailbox (dt: adspsys@1e000000) */
#define MT6895_ADSP_CFG_BASE        0x1e000000ULL
#define MT6895_ADSP_CFG_SIZE        0x00006000
#define MT6895_ADSP_MBOX0_BASE      0x1e006000ULL
#define MT6895_ADSP_MBOX1_BASE      0x1e007000ULL
#define MT6895_ADSP_MBOX2_BASE      0x1e008000ULL
#define MT6895_ADSP_MBOX3_BASE      0x1e009000ULL
#define MT6895_ADSP_MBOX_SIZE       0x00001000
#define MT6895_ADSP_CFG2_BASE       0x1e00b000ULL
#define MT6895_ADSP_CFG2_SIZE       0x00005000
#define MT6895_ADSP_MBOX_SPI0       644     /* GIC SPI 644..647 */

#define MT6895_SCPSYS_BASE      0x1c001000ULL
#define MT6895_SCPSYS_SIZE      0x00001000

#define MT6895_SPMI_PMIF_BASE   0x1c804000ULL
#define MT6895_SPMI_PMIF_SIZE   0x00001000
#define MT6895_SPMI_MST_BASE    0x1c801000ULL
#define MT6895_SPMI_MST_SIZE    0x00000100

#define MT6895_SRCLKEN_RC_BASE  0x1c00d000ULL
#define MT6895_SRCLKEN_RC_SIZE  0x00000800

/*
 * dxcc_sec@10210000 -- "mediatek,dxcc_sec". The security engine's HRE
 * (host ring engine). The preloader drives it through a six-word descriptor
 * ring and gates every submission on the ring-status register.
 */
#define MT6895_DXCC_SEC_BASE    0x10210000ULL
#define MT6895_DXCC_SEC_SIZE    0x00001000
#define MT6895_DXCC_SEC_INT_STA  0x00000a00  /* bit2 = descriptor done */
#define MT6895_DXCC_SEC_RESULT   0x00000ba0  /* 1 = operation succeeded */
#define MT6895_DXCC_SEC_RING_STA 0x00000e9c  /* bits[3:0] = free descriptors */
#define MT6895_DXCC_SEC_GZ_STA   0x00000abc  /* bit0 = GZ engine ready (EXPERIMENT) */

/*
 * masp@1c009000 -- "mediatek,masp". The secure/crypto accelerator seclib
 * drives. A command is started by setting bit 30 of the command register and
 * the result is signalled with bit 31.
 */
#define MT6895_MASP_BASE        0x1c009000ULL
#define MT6895_MASP_SIZE        0x00001000
#define MT6895_MASP_CMD         0x00000008

#define MT6895_SPMI2_PMIF_BASE  0x1c014000ULL
#define MT6895_SPMI2_PMIF_SIZE  0x00001000
#define MT6895_SPMI2_MST_BASE   0x1c801800ULL
#define MT6895_SPMI2_MST_SIZE   0x00000100

/*
 * SRAM log header.
 *
 * The address must stay inside the BootROM window (MT6895_BOOTROM_SIZE),
 * otherwise the write below is skipped by its bounds check.
 *
 * log_store.ko's dt_get_log_store() reads the "pmic_register" property of the
 * device tree's "logstore" node (the real firmware has 0xa0d there), treats it
 * as a physical address, ioremaps 256 bytes and validates two magic words at
 * offset 0 and offset 0xc. On hardware the preloader writes that header into
 * SRAM; here nothing runs before Linux, so the machine lays it down.
 */
/*
 * Offset of the SRAM log header inside the logstore window. log_store's
 * init_module does "add x8, x0, #0x34" on its __ioremap() result and then
 * validates two magic words there, so the window base handed to it through
 * the device tree must have the header at exactly this offset.
 */
#define MT6895_LOGSTORE_HDR_OFF 0x00000034ULL
/* Aligned window covering the two slot-table bytes the rollback needs:
 * 0x020F4593 (slot in bits 3..5) and 0x020F471D (>= 4 for the precondition). */
#define MT6895_SLOTTBL_BASE     0x020f4590ULL
#define MT6895_SLOTTBL_SIZE     0x00000190ULL

#define MT6895_LOGSTORE_SIG0    0xabcd1234
#define MT6895_LOGSTORE_SIG1    0x5678ef90

/*
 * A 256-byte window that sits on top of the BootROM region and serves the SRAM
 * log header. It exists so that accesses to the header can be traced: the ROM
 * kernel's log_store module reports reading the wrong bytes, and the first
 * question is whether the guest touches this address at all.
 */
#define MT6895_LOGSTORE_WIN_BASE 0x00000a00ULL
#define MT6895_LOGSTORE_WIN_SIZE 0x00000100ULL

/*
 * dcm@10001000 -- "mediatek,mt6895-dcm". Seven register windows; the DCM
 * driver ioremaps them and walks a register table. Before these existed the
 * walk hit an unmapped address and the kernel died with a synchronous
 * external abort while loading mt6895_dcm.ko, which rebooted the machine.
 */
#define MT6895_DCM_INFRACFG_AO      0x10001000ULL
#define MT6895_DCM_INFRA_AO_BCRM    0x10022000ULL
#define MT6895_DCM_INFRACFG_AO_MEM  0x10270000ULL
#define MT6895_DCM_PERI_AO_BCRM     0x11035000ULL
#define MT6895_DCM_VLP_AO_BCRM      0x1c017000ULL
#define MT6895_MCUSYS_AO_CFG_LO_BASE 0x0c530000ULL
#define MT6895_MCUSYS_AO_CFG_LO_SIZE 0x00008000
#define MT6895_DCM_MP_CPUSYS_TOP    0x0c538000ULL
#define MT6895_DCM_MP_CPUSYS_SIZE   0x00005000
#define MT6895_DCM_CPCCFG_REG       0x0c53a800ULL
#define MT6895_DCM_WIN_SIZE         0x00001000
#define MT6895_MCUSYS_AO_CFG_HI_BASE 0x0c53d000ULL
#define MT6895_MCUSYS_AO_CFG_HI_SIZE 0x00003000
#define MT6895_MCUPM_SRAM_BASE      0x0c540000ULL
#define MT6895_MCUPM_SRAM_SIZE      0x00022000
#define MT6895_MCUPM_CORE_CFG_BASE  0x0c580000ULL
#define MT6895_MCUPM_CORE_CFG_SIZE  0x00004000
#define MT6895_MCUPM_LOG_BASE       0x0c56f000ULL
#define MT6895_MCUPM_LOG_SIZE       0x00001000

/*
 * dramc@10230000 -- "mediatek,mt6895-dramc". Seventeen register windows.
 * mtk_dramc_get_ddr_type() reads one of them during probe; without the
 * window the read faults and the kernel panics while loading mmqos_mt6895.
 */
#define MT6895_DRAMC_WIN_COUNT  28
static const struct { hwaddr base; hwaddr size; } mt6895_dramc_wins[] = {
    { 0x10230000ULL, 0x2000 },
    { 0x10240000ULL, 0x2000 },
    { 0x10250000ULL, 0x2000 },
    { 0x10260000ULL, 0x2000 },
    { 0x10234000ULL, 0x1000 },
    { 0x10244000ULL, 0x1000 },
    { 0x10254000ULL, 0x1000 },
    { 0x10264000ULL, 0x1000 },
    { 0x10238000ULL, 0x2000 },
    { 0x10248000ULL, 0x2000 },
    { 0x10258000ULL, 0x2000 },
    { 0x10268000ULL, 0x2000 },
    { 0x10236000ULL, 0x1000 },
    { 0x10246000ULL, 0x1000 },
    { 0x10256000ULL, 0x1000 },
    { 0x10266000ULL, 0x1000 },
    { 0x10006000ULL, 0x1000 },
    /*
     * The remaining "dramc_ch1_rsv" windows from the same device tree node
     * list. The preloader's DDR-reserve code reaches them through
     * u4RegBaseAddrTranslate(), whose group-4 base table is
     * {0x00900000, 0x00a00000, 0x00b00000, 0x00c00000}: those plus the
     * translator's fixed +0x10000000 bias are exactly these addresses.
     * Without them the very first register of the walk aborts
     * (measured: DFAR 0x10947004, DFSR 0x8 synchronous external abort).
     */
    { 0x1024a000ULL, 0x2000 },
    { 0x1025a000ULL, 0x2000 },
    { 0x1026a000ULL, 0x2000 },
    { 0x10900000ULL, 0x40000 },
    { 0x10940000ULL, 0xc0000 },
    { 0x10a00000ULL, 0x40000 },
    { 0x10a40000ULL, 0xc0000 },
    { 0x10b00000ULL, 0x40000 },
    { 0x10b40000ULL, 0xc0000 },
    { 0x10c00000ULL, 0x40000 },
    { 0x10c40000ULL, 0xc0000 },
};

#define MT6895_WDT_BASE         0x1c007000ULL
#define MT6895_WDT_SIZE         0x00000100

/*
 * pinctrl@10005000 -- "mediatek,mt6895-pinctrl". Its reg list is the GPIO
 * block plus thirteen per-bank iocfg windows; the EINT registers the driver
 * touches during probe live in the first one. The iocfg windows are NOT
 * evenly strided, so they are listed explicitly. Without them the pinctrl
 * module faults on load, which is early enough to stop the module chain.
 */
#define MT6895_PINCTRL_BASE     0x10005000ULL
#define MT6895_PINCTRL_SIZE     0x00001000ULL
#define MT6895_IOCFG_BASE       0x11c20000ULL

/*
 * apirq@11e50000 -- "mediatek,mt6983-eint", referenced from the pinctrl
 * node as mediatek,eint. Five banks, each with its own 4 KiB window; the
 * pinctrl driver maps them while initialising EINT.
 */
#define MT6895_EINT_BASE0       0x11e50000ULL
#define MT6895_EINT_BASE1       0x11ce0000ULL
#define MT6895_EINT_BASE2       0x11de0000ULL
#define MT6895_EINT_BASE3       0x11fe0000ULL
#define MT6895_EINT_BASE4       0x1c01e000ULL
#define MT6895_EINT_SIZE        0x00001000ULL

/*
 * PCIe / MSI windows.
 *
 * The ROM kernel has pci-host-ecam-generic and nvme built in but no virtio,
 * so PCIe + NVMe is the storage path that needs no MTK controller emulation.
 * These addresses sit in the gap between the debug block at 0x0d000000 and
 * the syscon block at 0x10000000, which the MT6895 device tree leaves free.
 */
#define MT6895_ITS_BASE         0x0c800000ULL
#define MT6895_ITS_SIZE         0x00020000ULL
#define MT6895_PCIE_ECAM_BASE   0x0e000000ULL
#define MT6895_PCIE_ECAM_SIZE   0x00100000ULL
#define MT6895_PCIE_MMIO_BASE   0x0e100000ULL
#define MT6895_PCIE_MMIO_SIZE   0x01e00000ULL
#define MT6895_PCIE_PIO_BASE    0x0ff00000ULL
#define MT6895_PCIE_PIO_SIZE    0x00010000ULL

/* PCIe legacy INTx lines, mapped onto free SPIs. */
#define MT6895_PCIE_IRQ         900

/*
 * MTK MSDC (SD/eMMC) controller, and the MMC PHY its device tree node
 * references. The ROM has no virtio, no NVMe and no generic SDHCI glue, so
 * this is the only route to /dev/block/by-name/<name> for Android.
 */
#define MT6895_MSDC_BASE        0x11240000ULL
#define MT6895_MSDC_SIZE        0x00001000ULL
#define MT6895_MSDC_PHY_BASE    0x11e70000ULL
#define MT6895_MSDC_PHY_SIZE    0x00001000ULL
#define MT6895_MSDC_SPI         135

/* Architected timer frequency, from the MT6895 device tree. */
#define MT6895_CNTFRQ_HZ        13000000

/*
 * BootROM window. The MT6895 memory map leaves 0x0..0x0c000000 unused, which
 * is where the on-chip boot ROM and its parameter SRAM live on real silicon.
 */
#define MT6895_BOOTROM_BASE     0x00000000ULL
#define MT6895_BOOTROM_SIZE     0x00001000ULL
/*
 * LK's sec_func_init() searches for "AND_ROMINFO_v" from virtual address
 * 0x1000 upwards.  Expose that page with the matching bytes of the
 * preloader image (file 0x1000..0x2000) so VA 0x131C hits the signature.
 */
#define MT6895_ROMPAGE_BASE     0x00001000ULL
#define MT6895_ROMPAGE_SIZE     0x00001000ULL
#define MT6895_ROMPAGE_END      (MT6895_ROMPAGE_BASE + MT6895_ROMPAGE_SIZE)
/*
 * Parameters the BootROM leaves for the preloader's DRAM-mode stage (the copy
 * placed in the 0x02000000 window). The stack size is read by the SRAM stage
 * to compute its stack pointer.
 */
#define MT6895_DRAM_STAGE_STACK_SIZE_OFF   0x0006d308ULL
#define MT6895_DRAM_STAGE_STACK_SIZE       0x00100000U     /* 1 MiB */

/*
 * The DRAM stage keeps its debug UART base in a variable instead of a
 * literal; its character-output routine polls that variable for the line
 * status register. Left at zero it spins before printing anything.
 */
#define MT6895_DRAM_STAGE_UART_BASE_OFF    0x00079e00ULL

/*
 * The DRAM stage's first argument slot. The SRAM stage leaves sp at
 * 0x020FFFFC; the prologue pushes five registers (0x020FFFE8) and does
 * "add r7, sp, #12" (0x020FFFF4); the entry reads *(r7 + 36), i.e.
 * 0x02100018 -- SRAM1 offset 0x100000 - 4 - 20 + 12 + 36 = 0x100018.
 */
#define MT6895_DRAM_STAGE_ARG_OFF          0x00100018ULL

/*
 * The DRAM calibration block's "initialised" marker. The routine at
 * 0x02034B86 compares *(0x02075DC8) against 256 and treats anything else as
 * an uninitialised block. SRAM1 offset: 0x75DC8.
 */
#define MT6895_DRAM_STAGE_CAL_MARKER_OFF   0x00075dc8ULL

/*
 * A pointer the region-descriptor routine at 0x020345E8 loads and tests
 * before anything else: zero sends it straight to the "power of 2" error
 * print. The address falls inside the payload's BSS clear range.
 */
#define MT6895_DRAM_STAGE_REGION_PTR_OFF   0x000f3720ULL

/*
 * The same pointer as a read-only register, because the payload's own BSS
 * clear otherwise zeroes whatever the machine writes there.
 */
#define MT6895_REGION_PTR_BASE   0x020f3720ULL
#define MT6895_REGION_PTR_SIZE   0x00000008ULL

/* Legacy I2C compatibility view, ending before the PL log-store globals. */
#define MT6895_I2C_DESC_BASE     0x020f369cULL
#define MT6895_I2C_DESC_SIZE     0x94

/*
 * The pointer the RKP path dereferences at 0x0200CFCC. The routine branches
 * past its "+28" step, so this is the word that must be non-zero. Read-only
 * for the same reason as the two above: the payload clears its own BSS.
 */
#define MT6895_RKP_HEAD_BASE     0x020f130cULL

/*
 * The 64-bit value the release-size routine at 0x0202CAD0 reads and shifts
 * right by 9 before handing it to the partition parse as the current-LBA it
 * expects:
 *
 *   0x0202CAEA  ldr  r0, [pc, #52]   ; -> 0x020F4098
 *   0x0202CAEC  ldrd r0, r1, [r0]
 *   0x0202CAF0  lsrs r0, r0, #9 / orr r0, r0, r1, lsl #23 / lsrs r1, r1, #9
 *
 * It sits inside the payload's BSS clear (0x020F1000..0x020FB870), so like the
 * other BootROM windows it needs a writable overlay that survives the clear.
 * With the real GPT the header's current-LBA field is 1, so the value here is
 * 1 << 9 = 512.
 */
#define MT6895_LBA_SOURCE_BASE   0x020f4098ULL
#define MT6895_LBA_SOURCE_SIZE   8
#define MT6895_LBA_SOURCE_VALUE  (MT6895_GPT_CURRENT_LBA << 9)

/*
 * The preloader's boot-device-type global, read by the platform object
 * builder and the block-device driver alike:
 *
 *   0x0200C8CA  cmp  r0, #5   /  beq        ; driver, expects UFS
 *   0x0200C50E  cmp  r0, #5   /  bne -> assert("wrong memory type")
 *
 * The word lives at SRAM0 offset 0xA8FC, inside the range the payload
 * itself clears (0x00102180 onward), so a plain RAM write is wiped before
 * it is ever read. On hardware the BootROM supplies it after the clear;
 * here a read-only overlay stands in for that, exactly like the other
 * BootROM windows.
 */
#define MT6895_DEV_TYPE_BASE    0x0010a8fcULL
#define MT6895_DEV_TYPE_SIZE    0x00000004ULL
#define MT6895_DEV_TYPE_VALUE   5U              /* UFS */

/*
 * The preloader's platform device table (sl), read by the device-object
 * builder 0x0200C47C and by the storage layer:
 *
 *   +0x00  device id          (0x0203916A compares it with 0x28 / 0x37)
 *   +0x08  pointer Q          (byte [Q+0] feeds the object; helpers 0x02008CCC)
 *   +0x1C  pointer P0..P2     (*P drives obj[+8], whose bit 15 is polled at
 *                              0x0203919E until set)
 *
 * The table lives at SRAM1 offset 0xF1128, inside the range the payload
 * clears (0x020F1000..0x020FB870), so a plain RAM write is wiped; the
 * targets themselves are placed past the payload at MT6895_DEV_OBJ_BASE,
 * which survives (the payload's image ends at 0x0207BAC0).
 */
#define MT6895_DEV_TBL_BASE     0x020f1128ULL
#define MT6895_DEV_TBL_SIZE     0x00000030ULL
#define MT6895_DEV_OBJ_BASE     0x0207c000ULL

/*
 * The preloader's platform table fp (SRAM0 0x0010A900, also inside a BSS
 * clear range). The device-object builder takes most of the object layout
 * from it:
 *
 *   fp[+0x30]/[+0x34] and [+0x50]/[+0x54]  -> obj[+0x24] / obj[+0x28]
 *   fp[+0x28]/[+0x2C]                       -> obj[+0x2C] / obj[+0x30]
 *   fp[+0x80]/[+0x81] byte, fp[+0xA2] byte, fp[+0x00] word
 *
 * The storage layer then polls bit 15 of [obj[+0x28] + 8]
 * (0x0203919E) until it is set, so obj[+0x28] must point at a structure
 * whose eighth byte is 0x00008000.
 */
#define MT6895_PLAT_TBL_BASE    0x0010a900ULL
#define MT6895_PLAT_TBL_SIZE    0x000000a4ULL
#define MT6895_DEV_SUBOBJ_BASE  0x0207c040ULL

/*
 * The storage layer's per-device queue tables (SRAM1 BSS, inside the
 * payload's clear range):
 *
 *   devA = 0x020F40A0  stride 0x88    (0x0200C47C builder call site uses it;
 *                                      mapA at 0x020386EC)
 *   devB = 0x020F41B0  stride 0x2E4   (mapB at 0x020386FC)
 *
 * The write path transacts on entry 1 of each: obj = devA[1],
 * and 0x0203919E polls bit 15 of [obj[+0x28] + 8] until set, while
 * devB[1] supplies the device id (0x37 / 0x28 checks at 0x0203916A,
 * 0x020391B0), size (+4) and a count (+8).
 */
#define MT6895_DEVQ_BASE        0x020f40a0ULL
#define MT6895_DEVQ_SIZE        0x00000400ULL   /* covers devA[1] and devB[1] */

/*
 * Boot-reason globals read by the boot-mode dispatcher in bldr_pre_process
 * (0x02042718 / 0x02042736). On hardware the PMIC reports a power-on reason
 * (power key etc.); our PMIC model reads 0 for those registers, so this
 * lift stands in for it. Either global being non-zero selects NORMAL_BOOT:
 *
 *   0x02042712  bl 0x204389C  ; [0x020F4BA8], cbnz -> mode 0
 *   0x02042718  bl 0x2043890  ; [0x020F4BA4], cbnz -> mode 0
 *   0x02042736  ...           ; otherwise "Unknown boot!" / power off.
 */
#define MT6895_BOOT_REASON_BASE 0x020f4ba4ULL
#define MT6895_BOOT_REASON_SIZE 0x00000008ULL

/*
 * Where a -machine data= image is placed. Well inside the 3 GiB the machine
 * maps from 0x40000000, and high enough not to collide with the payload's own
 * use of low DRAM.
 */
#define MT6895_DATA_ADDR        0x80000000ULL

/*
 * The DRAM stage looks a name up in a table it addresses as
 * *(0x020F33BC) + 0x78EA4 -- and *(0x020F33BC) is this machine's to set, so
 * with the base it already supplies the table lands at 0x02078EA4.
 *
 *   0x0203F63E  ldr r0, [r5, #4]   ; non-zero entry is valid
 *   0x0203F642  ldr r1, [r5, #16]  ; the name
 *   0x0203F648  bl  strcmp         ; against the name it is looking for
 *   0x0203F64E  adds r5, #20       ; 20-byte stride
 *
 * The image's table is empty there (its first entry has +4 = 0), so the lookup
 * falls through to the RKP entry and the GPT. Filling it in is what lets the
 * first path succeed. 0x02078EA4 is above the payload's SRAM1 clear range
 * (0x020F1000..0x020FB870), so a value written here survives.
 */
#define MT6895_NAME_TABLE_BASE  0x02078ea4ULL
#define MT6895_NAME_TABLE_STRIDE 20
/*
 * Overlaid for the same reason as MT6895_RKP_OBJ_SIZE: the payload's BSS
 * clear is wider than the range previously recorded and wipes a plain
 * address_space_write here (measured: still zero 45 s into the run). The
 * entries mt6895_place_name_table builds are served from an overlay so the
 * partition-by-name lookup at 0x0204053E survives.
 */
#define MT6895_NAME_TABLE_SIZE   0x000000e0ULL

/*
 * The device table the payload's get-device-by-name routine reads
 * (0x0203FF7C: base 0x020F48AC, 12-byte entries, 0x210 bytes = 44 slots).
 * Entry layout: +0 name pointer, +4 selector, +8 ops pointer. Nothing in
 * the payload image ever writes this table -- only 0x0203FF7C and
 * 0x0203FFD6 read it -- so the machine has to supply it. Without an entry
 * the routine returns -1, the caller still passes its stack buffer to the
 * generic dispatcher (0x0204362E: mov r0, sp), and the dispatcher branches
 * through garbage to 0x04 where the CPU loops on undefined instructions
 * (measured: 11.6 million of them).
 */
#define MT6895_DEVTBL_ADDR       0x020f48acULL
#define MT6895_DEVTBL_SIZE       0x00000210ULL
#define MT6895_DEVTBL_MAX        44

/*
 * The registry pmic_auxadc_device_register walks. The routine at
 * 0x020445C4 reads an array of ten 8-byte slots here, and for every
 * non-zero slot loads the descriptor it names and strcmps the
 * descriptor's first word against the device being registered
 * ('main_pmic' at 0x0205F365, 'subpmic' at 0x0205FE42). Nothing in
 * the payload image writes this array, so an empty one makes the
 * strcmp read a garbage pointer -- measured: R0=0x020F4B2C,
 * R1=0x5800, a data abort inside strcmp at 0x02048786, then the
 * CPU spins in the abort vector. The same root cause shows up in
 * the log as "[xiaomi]: [pl] read build id failed! get auxadc pmic
 * failed and quit!".
 *
 * Layout: 0x50 bytes of slots followed by two 8-byte descriptors.
 * A wider overlay than 0x78 bytes would shadow payload BSS.
 */
#define MT6895_AUXADC_ADDR       0x020f4bc8ULL
#define MT6895_AUXADC_SIZE       0x00000078ULL

/*
 * The name the DRAM stage looks up is the pointer its caller passes, and that
 * pointer is 0x02062341 -- two bytes into the string "aee_debug_kinfo", so the
 * C string it compares against is "e_debug_kinfo". Pointing the table entry at
 * the very same address is what makes the comparison succeed; writing an equal
 * string of our own would work too, but reusing the payload's own pointer is
 * exact by construction.
 */
/*
 * Whatever the DRAM stage hands the lookup as its key -- tracing the call at
 * 0x020349E8 shows it is the literal 0x0205CB2D, which lands inside the
 * "lca_reserved_mem ..." format string, not a partition name. Pointing the
 * table entry at that very address makes strcmp compare a string with itself,
 * so the lookup succeeds by construction and the code after it can be seen.
 */
#define MT6895_NAME_STR_BASE    0x0205cb2dULL

/*
 * The DRAM stage only consults that table when the byte at 0x020F4AC4 is 1:
 *
 *   0x0203F62E  ldr  r0, [pc, #92]   ; -> 0x020F4AC4
 *   0x0203F630  ldrb r0, [r0, #0]
 *   0x0203F632  cmp  r0, #1
 *   0x0203F634  bne  0x52            ; skip the whole lookup
 *
 * The address is inside the payload's BSS clear (0x020F1000..0x020FB870), so
 * it reads back as zero and the table path is never taken. A writable overlay
 * like the other BootROM windows is what lets the table be used at all.
 */
#define MT6895_KINFO_FLAG_BASE  0x020f4ac4ULL
#define MT6895_KINFO_FLAG_SIZE  1
#define MT6895_RKP_HEAD_SIZE     0x00000004ULL

/*
 * The table the RKP lookup resolves to, and the entry the caller looks up
 * (it passes 1). Placed past the payload and outside the payload's clear
 * range, so a plain write survives.
 */
#define MT6895_RKP_TABLE_OFF     0x0007b000ULL
#define MT6895_RKP_TABLE_BASE    (MT6895_SRAM1_BASE + MT6895_RKP_TABLE_OFF)
#define MT6895_RKP_TABLE_ENTRIES 16

/*
 * The RKP device object, overlaid at MT6895_RKP_TABLE_BASE so the payload's
 * BSS clear cannot wipe the fields the storage layer dereferences. See
 * mt6895_rkp_obj_ops in mt6895.c for the measured timeline.
 */
/*
 * The whole machine-planted block at MT6895_RKP_TABLE_BASE: the device object
 * at +0x000, the success stub at +0x100 and the GPT header at +0x200. All of
 * it sits past the payload image (which ends at 0x0207A0C0) and inside the
 * 0x2800-byte BSS clear the payload memsets from 0x020798A4, so it is served
 * from an overlay with a default image behind it.
 */

/*
 * The block-device object the I/O routine at 0x0200D010 drives, and the stub
 * it calls back into. Layout from the instructions that read it:
 *   +0x00  type        -- 0x0202CAD0 requires 1 or 2
 *   +0x04  block size  -- must be non-zero; 512, from the release path's >> 9
 *   +0x20  I/O callback-- reached by "blx ip"; zero return means success, so
 *                         a two-instruction stub that returns zero is enough
 */
#define MT6895_RKP_OBJ_TYPE     1
#define MT6895_RKP_BLOCK_SIZE   512

/*
 * The GPT header the stub copies into the buffer the I/O routine provides.
 * SRAM1 offset 0x7B200, and the stub's literal names that address directly.
 */
#define MT6895_RKP_GPT_OFF      0x0007b200ULL
#define MT6895_GPT_HEADER_SIZE  92
#define MT6895_GPT_ENTRIES      128
#define MT6895_GPT_ENTRY_SIZE   128

/*
 * Where the payload hands its block-I/O callback the buffer (measured twice
 * as 0x00102180), and where the machine keeps the source image it is copied
 * from.
 *
 * The buffer itself cannot be pre-filled: 0x00102180 is the start of the
 * first range the payload's SRAM stage clears, so anything written there at
 * machine init is wiped before it is ever read. The source lives at SRAM1 +
 * 0x7B200 instead, which a probe confirmed survives the run, and the callback
 * copies from it.
 */
#define MT6895_GPT_BUFFER       0x00102180ULL

/* Where the callback reads how many entry bytes to copy (see entry-bytes). */
#define MT6895_ENTRY_BYTES_ADDR 0x0207f800ULL
/*
 * The GPT source lives in DRAM, not in SRAM1.
 *
 * It used to sit at SRAM1 + 0x7B200, which is where the payload's memory
 * descriptor table also lives: the table's count is at +0x7CCF8 and its
 * entries start at +0x7CD00, both inside the 17408 bytes a GPT occupies. The
 * machine was therefore overwriting the table it had just built, and the
 * descriptor-dump loop at 0x02036EF0 ran for whatever count the GPT's bytes
 * happened to spell -- which is what made one earlier run print far more than
 * the others.
 */
#define MT6895_GPT_SOURCE       0x50000000ULL
#define MT6895_RKP_GPT_ADDR     (MT6895_SRAM1_BASE + MT6895_RKP_GPT_OFF)

/*
 * The current-LBA field the partition parse validates against the value its
 * caller passes in r2/r3 ("eor r1,r3,r9" / "eor r0,r2,sl" / "bne").
 */
#define MT6895_GPT_CURRENT_LBA  1ULL

#define MT6895_RKP_STUB_OFF     0x0007b100ULL
/*
 * The stub is ARM state now (it uses ldm/stm for 24-byte copies), so the
 * address keeps bit 0 clear: the caller reaches it with blx, which selects
 * ARM when bit 0 is 0.
 */
#define MT6895_RKP_STUB_ADDR    (MT6895_SRAM1_BASE + MT6895_RKP_STUB_OFF)
#define MT6895_RKP_STUB_IMM     0x2000u     /* movs r0, #0 */
#define MT6895_RKP_STUB_BXLR    0x4770u     /* bx   lr     */

/* The rest of the hand-assembled stub: it writes the GPT signature. */
#define MT6895_RKP_STUB_LDR8    0x4b02u     /* ldr r3, [pc, #8]  */
#define MT6895_RKP_STUB_STR4    0x604bu     /* str r3, [r1, #4]  */
#define MT6895_RKP_STUB_LDR4    0x4b01u     /* ldr r3, [pc, #4]  */
#define MT6895_RKP_STUB_STR0    0x600bu     /* str r3, [r1, #0]  */
#define MT6895_RKP_STUB_MAGIC_PART  0x54524150u  /* "PART" */
#define MT6895_RKP_STUB_MAGIC_EFI   0x20494645u  /* "EFI " */






/*
 * The DRAM stage's memory descriptor table: a base pointer variable, then an
 * entry count and an array of {start, size, map, name} entries at fixed
 * offsets from that base (entry stride 0x98).
 */
/*
 * The SRAM stage's hand-off literal (image offset 0x304) and the value the
 * machine writes there (property "stage-entry-patch").  In the live SRAM1
 * layout (image at 0x02000F10) 0x020360B8 is a real function entry, so the
 * literal is correct as built and ENTRY equals LIT_OLD; see
 * mt6895_load_boot_lun().
 */
#define MT6895_DRAM_STAGE_ENTRY_LIT_OFF    0x00000304ULL
#define MT6895_DRAM_STAGE_ENTRY_LIT_OLD    0x020360b9U
#define MT6895_DRAM_STAGE_ENTRY            0x020360b9U

/* Virtual GPIO pad-control bypass offsets in payload (Thumb: bx lr = 0x4770) */
#define MT6895_GPIO_SET_PULL_OFF           0x000305c4ULL
#define MT6895_GPIO_SET_SMT_OFF            0x000305d4ULL
#define MT6895_DCM_DEVAPC_INIT_OFF         0x0003dfc4ULL

/* PLL settling delay literal offsets in payload */
#define MT6895_PLL_DELAY_LIT_OFF_1         0x00042614ULL
#define MT6895_PLL_DELAY_LIT_OFF_2         0x00047540ULL

/* Boot profiler get_timer callback literal offsets in payload */
#define MT6895_PROFILER_TIME_LIT_OFF_1     0x00035880ULL
#define MT6895_PROFILER_TIME_LIT_OFF_2     0x00041350ULL
#define MT6895_PROFILER_TIME_LIT_OFF_3     0x00041884ULL
#define MT6895_PROFILER_TIME_LIT_OFF_4     0x00041bf4ULL

/* PLL frequency query callback literal offsets in payload */
#define MT6895_PLL_FREQ_LIT_OFF_1          0x0003e9ccULL
#define MT6895_PLL_FREQ_LIT_OFF_2          0x0004204cULL

/*
 * WORKAROUND (diagnostic only, off unless -machine stage-arg=<value> is given)
 *
 * The DRAM stage's entry tests its second argument for 4 KB alignment:
 *
 *   0x0203600E  orrs r0, r2
 *   0x02036012  lsls r0, r0, #20      ; keeps the low 12 bits
 *   0x02036014  beq  0x0203604A       ; aligned -> the work path
 *
 * The SRAM stage loads that argument from its own literal at image offset
 * 0x2EC, which holds 0xDEADBEFF -- a poison value -- and nothing rewrites it,
 * so the test always fails and the stage takes its unconfigured branch. This
 * option replaces that word so the work path can be observed at all.
 *
 * Side effects: the stage then runs code it would not otherwise run, which can
 * mask the real question of where a valid argument is meant to come from.
 * Removing the option restores the image exactly.
 */
#define MT6895_STAGE_ARG_LIT_OFF           0x000002ecULL

#define MT6895_SYSTIMER_BASE    0x1c011000ULL

#define MT6895_KP_BASE          0x1c00e000ULL
#define MT6895_KP_SIZE          0x00001000ULL
/* DT: keypad@1c00e000, interrupts = <GIC_SPI 106 IRQ_TYPE_EDGE_RISING 0>. */
#define MT6895_KP_SPI           106

/*
 * DT: usb0@11201000, interrupts = <GIC_SPI 128 IRQ_TYPE_LEVEL_HIGH>.
 * This is the single device-mode interrupt line of the mtu3/ssusb block
 * (INTID 160); LK's fastboot UDC and the Linux mtu3 gadget driver both
 * request it.
 */
#define MT6895_SSUSB_SPI        128

/*
 * BootROM-supplied startup words, exposed read-only over the payload's BSS:
 * the memory descriptor base pointer and its entry count.
 */
#define MT6895_BROM_ARG_BASE    0x020f33bcULL
#define MT6895_BROM_ARG_SIZE    0x00000008ULL

#define MT6895_DRAM_STAGE_MEMDESC_PTR_OFF    0x000f33bcULL
#define MT6895_DRAM_STAGE_MEMDESC_COUNT_OFF  0x0007ccf8ULL
#define MT6895_DRAM_STAGE_MEMDESC_ENTRY_OFF  0x0007cd00ULL
#define MT6895_DRAM_STAGE_MEMDESC_STRIDE     0x00000098ULL

/* Table 1: DRAM memory descriptor table looked up at 0x02036120 */
#define MT6895_DRAM_STAGE_DRAM_DESC_COUNT_OFF 0x0007c0e8ULL
#define MT6895_DRAM_STAGE_DRAM_DESC_ENTRY_OFF 0x0007c0f0ULL
#define MT6895_DRAM_STAGE_DRAM_DESC_STRIDE    0x00000018ULL

#define MT6895_BOOT_DESC_BASE   0x00002000ULL
/*
 * EXPERIMENT (2026-11-03): DRAM address the preloader loads the AArch64
 * BL2_EXT image to (mblock "system_bl2-ext"); the preloader requests a warm
 * reset via RMR to hand over to it.  The machine model redirects the BootROM
 * stub there after that reset.
 */
#define MT6895_BL2_EXT_ADDR     0x62f00000ULL
#define MT6895_BOOT_DESC_SIZE   0x00001000ULL

/* Boot descriptor layout, consumed by hw/arm/mt6895_bootrom.S. */
#define MT6895_BOOT_DESC_ENTRY  0x00    /* u64: payload entry point */
#define MT6895_BOOT_DESC_SPSR   0x08    /* u64: SPSR_EL3 for the hand-off */
#define MT6895_BOOT_DESC_R4     0x10    /* u64: r4 handed to the payload */
#define MT6895_BOOT_DESC_LR     0x18    /* u64: LR handed to the payload (0 = leave as-is) */

/*
 * One DRAM controller register window and the channel it serves.
 *
 * The controller reaches the same block two ways: the command and control
 * registers are written with a small offset, which u4RegBaseAddrTranslate()
 * resolves through base-table group 0 to 0x10230000 + channel * 0x100000,
 * while the status and data registers are written with the encoded address
 * 0x00800000 | offset, which resolves through group 1 to
 * 0x10234000 + channel * 0x100000. Answering a mode-register read needs
 * both -- the MR index lives in the command window, the data comes back
 * through the other -- so every window carries its channel.
 */
typedef struct Mt6895DramcWin {
    struct Mt6895MachineState *s;
    unsigned ch;
    uint32_t regs[0x2000 / 4];
} Mt6895DramcWin;

/*
 * The DEVAPC (device access permission controller) windows carry a
 * violation-clear handshake: the preloader writes 1 to +0xdc, polls +0x1fc
 * until it reads 1, then writes 0 back to +0xdc. The register file is
 * otherwise a plain store-and-read-back block.
 */
typedef struct Mt6895DevapcWin {
    uint32_t regs[0x1000 / 4];
    bool clear_pending;
} Mt6895DevapcWin;

struct Mt6895MachineState {
    MachineState parent_obj;

    MemoryRegion bootrom;
    MemoryRegion preloader_page;
    MemoryRegion brom_svc;     /* BROM service entry window at 0x8000 */
    MemoryRegion boot_desc;    /* BROM parameter SRAM, at 0x2000 */
    MemoryRegion logstore_win;
    MemoryRegion brom_arg;
    MemoryRegion systimer;
    MemoryRegion kp;
    MemoryRegion srclken_rc;
    uint32_t srclken_rc_regs[MT6895_SRCLKEN_RC_SIZE / 4];
    MemoryRegion dxcc_sec;
    uint32_t dxcc_sec_regs[MT6895_DXCC_SEC_SIZE / 4];
    MemoryRegion masp;
    uint32_t masp_regs[MT6895_MASP_SIZE / 4];
    MemoryRegion dramc_top2[4];
    Mt6895DramcWin dramc_top2_win[4];
    MemoryRegion dramc_cmd[4];
    Mt6895DramcWin dramc_cmd_win[4];
    uint32_t dramc_mrr_cmd[4];   /* last MRR command per channel (offset 0x130) */
    MemoryRegion dramc_top3[4];
    Mt6895DramcWin dramc_top3_win[4];
    MemoryRegion devapc[3];
    Mt6895DevapcWin devapc_win[3];
    MemoryRegion topckgen_clk;
    uint32_t topckgen_regs[0x1000 / 4];
    uint32_t brom_arg_store[2];
    uint32_t region_ptr_store[2];
    uint32_t rkp_head_store[1];
    uint32_t name_tbl_store[MT6895_NAME_TABLE_SIZE / 4];
    uint8_t name_tbl_valid[MT6895_NAME_TABLE_SIZE];
    MemoryRegion name_tbl;       /* partition name table overlay */
    MemoryRegion peri_fill_mr;  /* low-priority fill for 0x1000_0000 window */
    MemoryRegion atf_head_mr;   /* write-protected ATF console list head */
    MemoryRegion atf_node_mr;
    MemoryRegion atf_arg_mr;
    MemoryRegion atf_log_mr;
    MemoryRegion cpc_mr;
    MemoryRegion bt_mr;
    MemoryRegion lkdtb_mr;
    MemoryRegion lkcount_mr;
    MemoryRegion cflag_mr;
    MemoryRegion own_mr;
    MemoryRegion atfarg_mr;
    MemoryRegion atfarg2_mr;
    MemoryRegion atfctx_mr;
    uint64_t atfctx_val;
    MemoryRegion bl33flag_mr;
    /* DISP mutex register block (0x14001000): EN/MUTEX/RST state machine so
     * the vendor DRM driver's hardware mutex acquire (poll MUTEXn bit1) and
     * release succeed instead of timing out. */
    MemoryRegion disp_mutex[2];          /* disp0 @0x14001000, disp1 @0x14401000 */
    uint32_t disp_mutex_regs[2][0x1000 / 4];
    uint32_t disp_mutex_locked[2];       /* one bit per mutex id, per instance */
    QEMUTimer *disp_vblank_timer;        /* MTK_DISP_VBLANK=1: fake frame EOF IRQ */
    qemu_irq disp_mutex_irq[2];          /* GIC SPI 300 (disp0) / 344 (disp1) */
    bool disp_mutex_irq_level[2];        /* line currently held high */
    /* DSI0 (0x14017000): behavioural MIPI DSI controller.
     *
     * One register file (dsi_regs) holds INTEN/INTSTA/CON_CTRL/MODE_CTRL/...
     * exactly as the guest writes it.  INTSTA latches status independently of
     * INTEN; GIC SPI 323 is level-high while (INTSTA & INTEN) != 0.  While
     * the controller streams (video mode, DSI_EN and START bit0) a frame
     * timer latches FRAME_DONE once per frame and raises the GCE events
     * disp_dsi0_eof (313) / disp_dsi0_sof0 (278). */
    MemoryRegion dsi_mr;
    uint32_t dsi_regs[0x1000 / 4];
    qemu_irq dsi_irq;
    bool dsi_irq_level;
    QEMUTimer *dsi_frame_timer;  /* video-mode frame cadence (60 Hz default) */
    bool dsi_streaming;          /* frame timer currently armed             */
    bool dsi_cmdq_ready;         /* CMDQ words programmed since last START  */
    uint64_t dsi_frame_count;    /* frames transmitted since reset          */
    uint32_t dsi_irq_count;      /* SPI 323 line asserts                    */
    /* Panel state, derived only from the DCS writes the guest queues. */
    bool dsi_ulps_done;          /* PHY in ULPS (INTSTA bit15)              */
    bool dsi_sleep_out;
    bool dsi_display_on;
    uint32_t dsi_rx[4];          /* DCS read response words (RX_DATA0..3)   */
    DeviceState *gce;            /* main GCE: CMDQ event signalling         */
    MemoryRegion diagg_mr;
    QEMUTimer *dtb_timer;
    QEMUTimer *scan_timer;   /* write-protected fake console node     */
    bool atf_win_ready;
    MemoryRegion planted[7];     /* RKP obj/stub, dev obj, sec ver, LK bootarg, BL32 tee base, ATF SMC */
    MemoryRegion i2c_ctrl[4];    /* EXPERIMENT: diagnostic I2C controller windows */
    MemoryRegion conbase;        /* UART console base pointer overlay */
    uint32_t conbase_store;
    MemoryRegion auxadc;         /* pmic auxadc registry (0x020F4BC8) */
    uint32_t auxadc_store[MT6895_AUXADC_SIZE / 4];
    MemoryRegion auxdev;        /* auxadc device object (0x02078E94) */
    uint32_t auxdev_store[0x10 / 4];
    MemoryRegion secpol;        /* SEC_POLICY table (0x020793D8) */
    MemoryRegion loghdr;        /* PL log-store header (0x0011DF34) */
    uint32_t loghdr_store[0x20 / 4];
    MemoryRegion bcb;           /* boot-control block (0x020F2310) */
    uint32_t bcb_store[0x30 / 4];
    MemoryRegion bootdev;       /* device list head (0x020F130C) */
    uint32_t bootdev_store[0x4 / 4];
    MemoryRegion bootdev_node;  /* device list node (0x020F1310) */
    uint32_t bootdev_node_store[0x30 / 4];
    MemoryRegion slottbl;      /* slot table bytes (0x020F4590) */
    uint32_t slottbl_store[MT6895_SLOTTBL_SIZE / 4];
    MemoryRegion objw;
     uint8_t objw_store[0x20];         /* partition object writes (0x020F4AC8) */
    MemoryRegion i2c_desc;       /* I2C descriptor compatibility view */
    uint32_t i2c_desc_store[MT6895_I2C_DESC_SIZE / 4];
    MemoryRegion devtbl;         /* payload device table (0x020F48AC) */
    uint32_t devtbl_store[0x210 / 4];
    MemoryRegion rodata;         /* payload .data vtable region overlay */
    uint32_t rodata_store[9];
    uint32_t rodata_store2[6];
    MemoryRegion rodata2;        /* PMIC ops vtable overlay */
    uint32_t lba_source_store[2];
    uint32_t dev_type_store[1];
    MemoryRegion dev_tbl;      /* platform device table overlay (sl) */
    uint32_t dev_tbl_store[MT6895_DEV_TBL_SIZE / 4];
    MemoryRegion plat_tbl;     /* platform table fp overlay */
    uint32_t plat_tbl_store[MT6895_PLAT_TBL_SIZE / 4];
    MemoryRegion devq;         /* device queue table overlay */
    uint32_t devq_store[MT6895_DEVQ_SIZE / 4];
    MemoryRegion boot_reason;    /* boot-reason globals overlay */
    MemoryRegion bdev_cmd;       /* block-device command register */
    MemoryRegion bdev_rcmd;      /* block-device read command register */
    QEMUTimer *bdev_timer;
    uint32_t bdev_cmd_lba;
    uint32_t bdev_cmd_cnt;
    uint32_t boot_reason_store[MT6895_BOOT_REASON_SIZE / 4];
    uint32_t kinfo_flag_store[1];
    uint32_t stage_arg; /* WORKAROUND: replace the DRAM stage argument literal */
    uint32_t stage_entry_patch; /* 1 = redirect the stage jump (default), 0 = keep the image value */
    MemoryRegion region_ptr;
    MemoryRegion rkp_head; /* traced SRAM log header window */
    MemoryRegion lba_source; /* current-LBA source window */
    MemoryRegion dev_type;   /* boot-device-type overlay (SRAM0 BSS) */
    MemoryRegion kinfo_flag;
    MemoryRegion crypto_flag; /* name-table enable flag window */
    MemoryRegion *ddr;   /* aliases MachineState.ram */
    MemoryRegion ddr_alias;  /* 0xffff0000_00000000 window onto DRAM */
    MemoryRegion gz_ddr_remap; /* GZ EMI domain remap: VMID1 view of DRAM */
    MemoryRegion gz_remap_mr;  /* SECURITY_AO GZ remap control (+0x690) */
    uint32_t gz_remap_regs[4];
    MemoryRegion wdt;        /* WDT/RGU control window */
    MemoryRegion calmark;    /* read probe on the DRAM cal marker */
    uint32_t wdt_store[0x1000 / 4];
    struct arm_boot_info binfo;   /* direct kernel boot (-kernel) only */
    bool direct_boot;
    MemoryRegion sram[MT6895_NUM_SRAMS];

    DeviceState *gic;
    QEMUTimer *gic_group_timer;
    DeviceState *its;
    DeviceState *pcie;
    DeviceState *msdc;
    DeviceState *spmi;
    DeviceState *spmi2;
    DeviceState *scpsys;
    DeviceState *ufshci;
    DeviceState *tinysys_mbox;
    DeviceState *adsp;
    DeviceState *gpueb;
    MemoryRegion pcie_ecam_alias;
    DeviceState *uart[2];

    /* -machine preloader=<path to a UFS boot LUN image> */
    char *preloader_path;

    /*
     * -machine gpt=<file>: the device's real GPT (header + 128 partition
     * entries, taken verbatim from the storage image so both CRC32 fields are
     * correct by construction). It is placed at the address the payload hands
     * its I/O callback, which two independent runs measured as 0x00102180.
     */
    char *gpt_path;

    /*
     * -machine entry-bytes=N: how many bytes of the partition-entry array the
     * I/O callback copies after the 92-byte header. The full array is 16384
     * bytes (128 entries), but writing all of it correlates with the parse
     * failing its CRC comparison while writing only the header lets it pass,
     * so the boundary is what needs measuring.
     */
    uint32_t entry_bytes;

    /*
     * -machine data=<file>: an image to place in DRAM and point the block-I/O
     * object's data-source field (+0x14) at.
     *
     * That field is the only handle the payload gives the machine on "where
     * the device's bytes live": 0x0200D026 reads it and passes it down. The
     * callback in +0x20 is called ten times with a ~4 KiB buffer, which is far
     * too little to move an 8 MiB LK image, so partition content must come
     * from memory the object points at rather than through the callback.
     */
    char *data_path;

    /*
     * The data image is served on demand from the host file through this
     * read-only window, instead of being copied into guest RAM at reset.
     * The preloader's block-read stub forms its source address as
     * "0x80000000 + byte_offset" in 32-bit arithmetic, so it can only ever
     * reach the first 2 GiB of the image -- which is exactly the window
     * size used here. Everything above stays ordinary RAM.
     */
    MemoryRegion data_win;
    int data_fd;
    bool data_writable;        /* image opened O_RDWR (MTK_LU2_RW) */
    uint64_t data_win_size;
    uint64_t data_file_size;   /* full image size, not the guest window */
    uint8_t *data_cache;
    uint64_t data_cache_idx;
    bool data_cache_valid;
    GHashTable *data_pages;
    struct Mt6895DataPage *data_page_cur;
    uint64_t data_page_idx;
    bool data_page_valid;

    /*
     * -machine boot-r4=<value>
     *
     * The BROM hands the preloader a value in r4; the payload's first
     * instruction stores it at the boot-argument block, and the DRAM-mode
     * stage -- entered from the SRAM build's literal pool at payload offset
     * 0x360B8 -- dereferences it as a code pointer and calls through it.
     *
     * Evidence for the value: the DRAM-mode code at 0x020360B8 issues
     * `blx r4`, and the only place in that function where r4 is set loads
     * the literal 0x02045C75. The SRAM build zeroes r4 at entry and never
     * sets it again before the jump, so the value has to arrive from the
     * BootROM.
     */
    uint64_t boot_r4;
    uint64_t boot_lr; /* LR handed to the payload; 0 leaves it alone */

    /* Resolved from the image during realize(). */
    bool boot_valid;
    uint64_t boot_entry;
    uint64_t boot_spsr;
};

#endif /* HW_ARM_MT6895_H */
