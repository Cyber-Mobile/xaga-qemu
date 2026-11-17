/*
 * MediaTek MT6895 BootROM hand-off stub
 *
 * The emulated BootROM performs, in hardware terms, exactly what the real
 * MT6895 BROM does before it starts the preloader:
 *
 *   1. read the boot LUN, verify the "UFS_BOOT" header and locate the MMM
 *      container (done by the machine model in C, because it is storage I/O);
 *   2. copy the MMM payload to its load address (also the machine model);
 *   3. drop the core out of its reset state and enter the payload.
 *
 * Step 3 is the only part that has to be real guest code, because the
 * AArch64 -> AArch32 register-width change can only happen through an
 * exception return. Doing it here means QEMU's normal ERET path performs the
 * transition (aarch64_sync_64_to_32, hflags rebuild, ...) instead of the
 * machine poking CPU state directly.
 *
 * Register conventions at reset: EL3, AArch64, MMU off, PC = RVBAR = 0x0.
 *
 * The entry address is not baked into this stub; the machine model writes a
 * boot descriptor at MT6895_BOOT_DESC_BASE before the CPU is released, so the
 * same stub works for any preloader image. Three fields are read from it:
 *
 *   +0x00  payload entry point
 *   +0x08  SPSR_EL3 for the hand-off
 *   +0x10  the value the payload expects to find in r4
 *
 * The SPSR is taken from the descriptor rather than hardcoded, and r4 is
 * loaded here because the AArch32 payload runs with r4 already holding it --
 * its first instruction stores r4 into the boot-argument block, and the
 * DRAM-mode stage later dereferences that. x4 is the same register as the
 * AArch32 r4, so setting it before the ERET is what the payload sees.
 *
 * SPSR_EL3 value 0x1C3 decodes as:
 *   M[4]   = 0    -> return to AArch32 (nRW clear)
 *   M[3:0] = 0x3  -> SVC mode, i.e. AArch32 EL1
 *   bit 5  = 0    -> ARM (not Thumb) state
 *   bit 6  = 1    -> FIQ masked
 *   bit 7  = 1    -> IRQ masked
 *   bit 8  = 1    -> SError/abort masked
 *   bit 9  = 0    -> little endian
 *
 * Regeneration (verified byte-for-byte against the array in mt6895.c):
 *   clang --target=aarch64-none-elf -march=armv8-a -c mt6895_bootrom.S -o b.o
 *   llvm-objcopy -O binary --only-section=.text b.o mt6895_bootrom.bin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

    .text
    .global mt6895_bootrom_start

mt6895_bootrom_start:
    movz    x3, #0x2000             /* boot descriptor base address */
    ldr     x1, [x3]                /* x1 = payload entry point */
    ldr     x2, [x3, #8]            /* x2 = SPSR_EL3 for the hand-off */
    ldr     x4, [x3, #16]           /* x4 = value the payload wants in r4 */
    msr     elr_el3, x1
    msr     spsr_el3, x2
    eret
