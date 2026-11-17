/*
 * QEMU support -- ARM Power Control specific functions.
 *
 * Copyright (c) 2016 Jean-Christophe Dubois
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#ifndef QEMU_ARM_POWERCTL_H
#define QEMU_ARM_POWERCTL_H

#include "kvm-consts.h"

#define QEMU_ARM_POWERCTL_RET_SUCCESS QEMU_PSCI_RET_SUCCESS
#define QEMU_ARM_POWERCTL_INVALID_PARAM QEMU_PSCI_RET_INVALID_PARAMS
#define QEMU_ARM_POWERCTL_ALREADY_ON QEMU_PSCI_RET_ALREADY_ON
#define QEMU_ARM_POWERCTL_IS_OFF QEMU_PSCI_RET_DENIED
#define QEMU_ARM_POWERCTL_ON_PENDING QEMU_PSCI_RET_ON_PENDING

/*
 * arm_get_cpu_by_id:
 * @cpuid: the id of the CPU we want to retrieve the state
 *
 * Retrieve a CPUState object from its CPU ID provided in @cpuid.
 *
 * Returns: a pointer to the CPUState structure of the requested CPU.
 */
CPUState *arm_get_cpu_by_id(uint64_t cpuid);

/*
 * arm_psci_set_prefer_el1:
 * @enable: if true, PSCI CPU_ON starts the target at EL1 instead of the
 *          highest non-secure exception level this emulator implements.
 *
 * For machines that run a real EL3 firmware (so the guest's non-secure world
 * lives at EL1), while QEMU also provides a PSCI conduit for CPU_ON.
 */
void arm_psci_set_prefer_el1(bool enable);

/*
 * arm_psci_set_secondary_hook:
 * @fn: callback invoked in the target vCPU's thread right after CPU_ON has
 *      reset and started a secondary CPU (or NULL to disable)
 *
 * Lets a machine plant firmware (EL3) per-CPU state for cores whose guest is
 * started directly by QEMU instead of through the firmware's warm-boot entry.
 */
void arm_psci_set_secondary_hook(void (*fn)(struct ArchCPU *cpu,
                                               unsigned core));

/*
 * arm_psci_set_skip_secondary_smc:
 * @enable: answer non-PSCI SMCs from non-boot CPUs with NOT_SUPPORTED instead
 *          of entering the guest EL3 firmware (which those cores never
 *          initialised because QEMU started them directly).
 */
void arm_psci_set_skip_secondary_smc(bool enable);

/* Log each SMC answered by arm_psci_set_skip_secondary_smc() (diagnostics). */
void arm_psci_set_smc_skip_debug(bool enable);

/*
 * arm_psci_set_lazy_plant:
 * @enable: run the secondary hook right before a non-boot CPU's first non-PSCI
 *          SMC is delivered to EL3, instead of when the CPU is started.
 */
void arm_psci_set_lazy_plant(bool enable);

/* Run the secondary hook from the CPU_ON path (debug/experimental). */
void arm_psci_set_eager_plant(bool enable);

/*
 * arm_set_cpu_on:
 * @cpuid: the id of the CPU we want to start/wake up.
 * @entry: the address the CPU shall start from.
 * @context_id: the value to put in r0/x0.
 * @target_el: The desired exception level.
 * @target_aa64: 1 if the requested mode is AArch64. 0 otherwise.
 *
 * Start the cpu designated by @cpuid in @target_el exception level. The mode
 * shall be AArch64 if @target_aa64 is set to 1. Otherwise the mode is
 * AArch32. The CPU shall start at @entry with @context_id in r0/x0.
 *
 * Returns: QEMU_ARM_POWERCTL_RET_SUCCESS on success.
 * QEMU_ARM_POWERCTL_INVALID_PARAM if bad parameters are provided.
 * QEMU_ARM_POWERCTL_ALREADY_ON if the CPU was already started.
 * QEMU_ARM_POWERCTL_ON_PENDING if the CPU is still powering up
 */
int arm_set_cpu_on(uint64_t cpuid, uint64_t entry, uint64_t context_id,
                   uint32_t target_el, bool target_aa64);

/*
 * arm_set_cpu_off:
 * @cpuid: the id of the CPU we want to stop/shut down.
 *
 * Stop the cpu designated by @cpuid.
 *
 * Returns: QEMU_ARM_POWERCTL_RET_SUCCESS on success.
 * QEMU_ARM_POWERCTL_INVALID_PARAM if bad parameters are provided.
 * QEMU_ARM_POWERCTL_IS_OFF if CPU is already off
 */

int arm_set_cpu_off(uint64_t cpuid);

/*
 * arm_reset_cpu:
 * @cpuid: the id of the CPU we want to reset.
 *
 * Reset the cpu designated by @cpuid.
 *
 * Returns: QEMU_ARM_POWERCTL_RET_SUCCESS on success.
 * QEMU_ARM_POWERCTL_INVALID_PARAM if bad parameters are provided.
 * QEMU_ARM_POWERCTL_IS_OFF if CPU is off
 */
int arm_reset_cpu(uint64_t cpuid);

/*
 * arm_set_cpu_on_and_reset:
 * @cpuid: the id of the CPU we want to star
 *
 * Start the cpu designated by @cpuid and put it through its normal
 * CPU reset process. The CPU will start in the way it is architected
 * to start after a power-on reset.
 *
 * Returns: QEMU_ARM_POWERCTL_RET_SUCCESS on success.
 * QEMU_ARM_POWERCTL_INVALID_PARAM if there is no CPU with that ID.
 * QEMU_ARM_POWERCTL_ALREADY_ON if the CPU is already on.
 * QEMU_ARM_POWERCTL_ON_PENDING if the CPU is already partway through
 * powering on.
 */
int arm_set_cpu_on_and_reset(uint64_t cpuid);

#endif
