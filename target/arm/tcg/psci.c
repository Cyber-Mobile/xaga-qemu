/*
 * Copyright (C) 2014 - Linaro
 * Author: Rob Herring <rob.herring@linaro.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "helper.h"
#include "kvm-consts.h"
#include "qemu/main-loop.h"
#include "system/runstate.h"
#include "internals.h"
#include "arm-powerctl.h"
#include "target/arm/multiprocessing.h"
#include "target/arm/trace.h"

/*
 * Set by a machine that virtualises EL3 itself (the guest runs its own
 * firmware) and whose non-secure world is entered at EL1: the guest's PSCI
 * CPU_ON then has to start secondaries at EL1, not at the highest NS EL this
 * emulator implements (EL2).  Off by default so all other machines keep the
 * upstream behaviour.
 */
static bool arm_psci_prefer_el1;
void arm_psci_set_prefer_el1(bool enable)
{
    arm_psci_prefer_el1 = enable;
}

/*
 * Optional machine hook run on a CPU right after QEMU has started it with
 * CPU_ON.  Machines that hand their secondaries to a guest while non-PSCI
 * SMCs are still served by an EL3 firmware use it to plant the EL3 world the
 * core would otherwise have set up in the firmware's warm-boot entry.
 * Runs in the target vCPU's thread, after its reset work.
 */
static void (*arm_psci_secondary_hook)(ARMCPU *cpu, unsigned core);

void arm_psci_set_secondary_hook(void (*fn)(ARMCPU *cpu, unsigned core))
{
    arm_psci_secondary_hook = fn;
}

/*
 * Machines whose non-PSCI SiP SMCs are served by a real EL3 firmware but
 * whose secondary CPUs QEMU starts directly cannot have those SMCs serviced:
 * the cores never ran the EL3 warm-boot entry, so EL3 faults on the first
 * one and wedges.  Answer such SMCs from non-boot cores with the
 * architectural NOT_SUPPORTED value so the guest driver degrades instead.
 */
bool arm_psci_skip_secondary_smc;
bool arm_psci_smc_skip_debug;

void arm_psci_set_skip_secondary_smc(bool enable)
{
    arm_psci_skip_secondary_smc = enable;
}

void arm_psci_set_smc_skip_debug(bool enable)
{
    arm_psci_smc_skip_debug = enable;
}

/*
 * Lazy plant: instead of preparing EL3 state when a secondary is started
 * (which perturbs its Linux startup), plant it right before its first
 * non-PSCI SMC is delivered to EL3 -- the first moment it is actually needed.
 */
bool arm_psci_lazy_plant;
bool arm_psci_eager_plant;

void arm_psci_set_lazy_plant(bool enable)
{
    arm_psci_lazy_plant = enable;
}

void arm_psci_set_eager_plant(bool enable)
{
    arm_psci_eager_plant = enable;
}

void arm_psci_plant_secondary(ARMCPU *cpu, unsigned core)
{
    static bool planted[64];
    if (core >= ARRAY_SIZE(planted) || planted[core]) {
        return;
    }
    planted[core] = true;
    if (arm_psci_secondary_hook) {
        arm_psci_secondary_hook(cpu, core);
        arm_rebuild_hflags(&cpu->env);
    }
}

static void arm_psci_secondary_apply(CPUState *cs, run_on_cpu_data data)
{
    ARMCPU *cpu = ARM_CPU(cs);

    if (arm_psci_secondary_hook) {
        arm_psci_secondary_hook(cpu, data.host_ulong);
        arm_rebuild_hflags(&cpu->env);
    }
}

bool arm_is_psci_call(ARMCPU *cpu, int excp_type)
{
    /*
     * Return true if the exception type matches the configured PSCI conduit.
     * This is called before the SMC/HVC instruction is executed, to decide
     * whether we should treat it as a PSCI call or with the architecturally
     * defined behaviour for an SMC or HVC (which might be UNDEF or trap
     * to EL2 or to EL3).
     */

    switch (excp_type) {
    case EXCP_HVC:
        /*
         * Host-PSCI mode also keeps swallowing HVCs even though the conduit
         * is SMC: this machine's LK (BL33) itself issues PSCI-style HVCs
         * (e.g. x0=0x84000003 with a vendor CPU id) while booting Linux.
         * Upstream/QEMU behaviour on a HVC conduit is to hand every HVC to
         * the PSCI dispatcher; doing the same here means those LK calls keep
         * getting a PSCI error return instead of trapping to an EL2 that has
         * no handler (which hangs right after "lk finished --> jump to linux
         * kernel 64Bit").
         */
        if (cpu->psci_conduit != QEMU_PSCI_CONDUIT_HVC &&
            !arm_psci_prefer_el1) {
            return false;
        }
        /*
         * Upstream behaviour on purpose: with a HVC conduit every HVC is
         * handed to the PSCI dispatcher.  A real EL3 firmware (mt6895/ATF)
         * shares that conduit with vendor "hypervisor calls" that are not
         * PSCI at all; those used to come back as PSCI_RET_NOT_SUPPORTED
         * (e.g. the guest's 0x820..../0xffffffff82000060 HVCs).  Filtering
         * them here makes them trap to the (unimplemented) EL2 instead and
         * hangs the vendor kernel at module load, so keep swallowing them
         * unless host PSCI mode below explicitly changes the contract.
         */
        break;
    case EXCP_SMC:
        if (cpu->psci_conduit != QEMU_PSCI_CONDUIT_SMC) {
            return false;
        }
        /*
         * Host-PSCI mode only (a machine that asked QEMU to answer PSCI over
         * SMC): the guest's EL1 SMCs are shared between PSCI and MTK SiP
         * calls, so only the PSCI function-ID windows (0x8400_00xx /
         * 0xC400_00xx) belong to QEMU; everything else must keep trapping to
         * EL3 (ATF).  Outside that mode the upstream behaviour (any SMC on
         * the conduit) is preserved exactly.
         */
        if (arm_psci_prefer_el1) {
            uint64_t fid = is_a64(&cpu->env) ? cpu->env.xregs[0]
                                             : cpu->env.regs[0];

            if ((fid & 0xFF000000ULL) != 0x84000000ULL &&
                (fid & 0xFF000000ULL) != 0xC4000000ULL) {
                return false;
            }
        }
        break;
    default:
        return false;
    }

    return true;
}

void arm_handle_psci_call(ARMCPU *cpu)
{
    /*
     * This function partially implements the logic for dispatching Power State
     * Coordination Interface (PSCI) calls (as described in ARM DEN 0022D.b),
     * to the extent required for bringing up and taking down secondary cores,
     * and for handling reset and poweroff requests.
     * Additional information about the calling convention used is available in
     * the document 'SMC Calling Convention' (ARM DEN 0028)
     */
    CPUARMState *env = &cpu->env;
    uint64_t param[4];
    uint64_t context_id, mpidr;
    uint64_t entry;
    int32_t ret = 0;
    int i;

    for (i = 0; i < 4; i++) {
        /*
         * All PSCI functions take explicit 32-bit or native int sized
         * arguments so we can simply zero-extend all arguments regardless
         * of which exact function we are about to call.
         */
        param[i] = is_a64(env) ? env->xregs[i] : env->regs[i];
    }
    trace_arm_psci_call(param[0], param[1], param[2], param[3],
                        arm_cpu_mp_affinity(cpu));

    if ((param[0] & QEMU_PSCI_0_2_64BIT) && !is_a64(env)) {
        ret = QEMU_PSCI_RET_NOT_SUPPORTED;
        goto err;
    }

    switch (param[0]) {
        CPUState *target_cpu_state;
        ARMCPU *target_cpu;

    case QEMU_PSCI_0_2_FN_PSCI_VERSION:
        ret = QEMU_PSCI_VERSION_1_1;
        break;
    case QEMU_PSCI_0_2_FN_MIGRATE_INFO_TYPE:
        ret = QEMU_PSCI_0_2_RET_TOS_MIGRATION_NOT_REQUIRED; /* No trusted OS */
        break;
    case QEMU_PSCI_0_2_FN_AFFINITY_INFO:
    case QEMU_PSCI_0_2_FN64_AFFINITY_INFO:
        mpidr = param[1];

        switch (param[2]) {
        case 0:
            target_cpu_state = arm_get_cpu_by_id(mpidr);
            if (!target_cpu_state) {
                ret = QEMU_PSCI_RET_INVALID_PARAMS;
                break;
            }
            target_cpu = ARM_CPU(target_cpu_state);

            g_assert(bql_locked());
            ret = target_cpu->power_state;
            break;
        default:
            /* Everything above affinity level 0 is always on. */
            ret = 0;
        }
        break;
    case QEMU_PSCI_0_2_FN_SYSTEM_RESET:
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        /* QEMU reset and shutdown are async requests, but PSCI
         * mandates that we never return from the reset/shutdown
         * call, so power the CPU off now so it doesn't execute
         * anything further.
         */
        goto cpu_off;
    case QEMU_PSCI_0_2_FN_SYSTEM_OFF:
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        goto cpu_off;
    case QEMU_PSCI_0_1_FN_CPU_ON:
    case QEMU_PSCI_0_2_FN_CPU_ON:
    case QEMU_PSCI_0_2_FN64_CPU_ON:
    {
        /* The PSCI spec mandates that newly brought up CPUs start
         * in the highest exception level which exists and is enabled
         * on the calling CPU. Since the QEMU PSCI implementation is
         * acting as a "fake EL3" or "fake EL2" firmware, this for us
         * means that we want to start at the highest NS exception level
         * that we are providing to the guest.
         * The execution mode should be that which is currently in use
         * by the same exception level on the calling CPU.
         * The CPU should be started with the context_id value
         * in x0 (if AArch64) or r0 (if AArch32).
         */
        int target_el = (arm_feature(env, ARM_FEATURE_EL2) &&
                         !arm_psci_prefer_el1) ? 2 : 1;
        bool target_aarch64 = arm_el_is_aa64(env, target_el);

        mpidr = param[1];
        entry = param[2];
        context_id = param[3];
        ret = arm_set_cpu_on(mpidr, entry, context_id,
                             target_el, target_aarch64);
        if (ret == QEMU_ARM_POWERCTL_RET_SUCCESS &&
            arm_psci_secondary_hook && arm_psci_eager_plant) {
            CPUState *tcpu = arm_get_cpu_by_id(mpidr);

            if (tcpu) {
                async_run_on_cpu(tcpu, arm_psci_secondary_apply,
                                 RUN_ON_CPU_HOST_ULONG((mpidr >> 8) & 0xff));
            }
        }
        break;
    }
    case QEMU_PSCI_0_1_FN_CPU_OFF:
    case QEMU_PSCI_0_2_FN_CPU_OFF:
        goto cpu_off;
    case QEMU_PSCI_0_1_FN_CPU_SUSPEND:
    case QEMU_PSCI_0_2_FN_CPU_SUSPEND:
    case QEMU_PSCI_0_2_FN64_CPU_SUSPEND:
        /* Affinity levels are not supported in QEMU */
        if (param[1] & 0xfffe0000) {
            ret = QEMU_PSCI_RET_INVALID_PARAMS;
            break;
        }
        /* Powerdown is not supported, we always go into WFI */
        if (is_a64(env)) {
            env->xregs[0] = 0;
        } else {
            env->regs[0] = 0;
        }
        helper_wfi(env, 4);
        break;
    case QEMU_PSCI_1_0_FN_PSCI_FEATURES:
        switch (param[1]) {
        case QEMU_PSCI_0_2_FN_PSCI_VERSION:
        case QEMU_PSCI_0_2_FN_MIGRATE_INFO_TYPE:
        case QEMU_PSCI_0_2_FN_AFFINITY_INFO:
        case QEMU_PSCI_0_2_FN64_AFFINITY_INFO:
        case QEMU_PSCI_0_2_FN_SYSTEM_RESET:
        case QEMU_PSCI_0_2_FN_SYSTEM_OFF:
        case QEMU_PSCI_0_1_FN_CPU_ON:
        case QEMU_PSCI_0_2_FN_CPU_ON:
        case QEMU_PSCI_0_2_FN64_CPU_ON:
        case QEMU_PSCI_0_1_FN_CPU_OFF:
        case QEMU_PSCI_0_2_FN_CPU_OFF:
        case QEMU_PSCI_0_1_FN_CPU_SUSPEND:
        case QEMU_PSCI_0_2_FN_CPU_SUSPEND:
        case QEMU_PSCI_0_2_FN64_CPU_SUSPEND:
        case QEMU_PSCI_1_0_FN_PSCI_FEATURES:
            if (!(param[1] & QEMU_PSCI_0_2_64BIT) || is_a64(env)) {
                ret = 0;
                break;
            }
            /* fallthrough */
        case QEMU_PSCI_0_1_FN_MIGRATE:
        case QEMU_PSCI_0_2_FN_MIGRATE:
        default:
            ret = QEMU_PSCI_RET_NOT_SUPPORTED;
            break;
        }
        break;
    case QEMU_PSCI_0_1_FN_MIGRATE:
    case QEMU_PSCI_0_2_FN_MIGRATE:
    default:
        ret = QEMU_PSCI_RET_NOT_SUPPORTED;
        break;
    }

err:
    if (is_a64(env)) {
        env->xregs[0] = ret;
    } else {
        env->regs[0] = ret;
    }
    return;

cpu_off:
    ret = arm_set_cpu_off(arm_cpu_mp_affinity(cpu));
    /* notreached */
    /* sanity check in case something failed */
    assert(ret == QEMU_ARM_POWERCTL_RET_SUCCESS);
}
