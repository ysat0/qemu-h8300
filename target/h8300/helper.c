/*
 *  H8/300 emulation
 *
 *  Copyright (c) 2019 Yoshinori Sato
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "cpu.h"
#include "exec/log.h"
#include "exec/cpu_ldst.h"
#include "sysemu/sysemu.h"

void h8300_cpu_unpack_ccr(CPUH8300State *env, uint32_t ccr)
{
    env->ccr_i = FIELD_EX8(ccr, CCR, I);
    env->ccr_ui = FIELD_EX8(ccr, CCR, UI);
    env->ccr_h = FIELD_EX8(ccr, CCR, H);
    env->ccr_u = FIELD_EX8(ccr, CCR, U);
    env->ccr_n = FIELD_EX8(ccr, CCR, N) << 31;
    env->ccr_z = 1 - FIELD_EX8(ccr, CCR, Z);
    env->ccr_v = FIELD_EX8(ccr, CCR, V) << 31;
    env->ccr_c = FIELD_EX8(ccr, CCR, C);
}

void h8300_cpu_do_interrupt(CPUState *cs)
{
    H8300CPU *cpu = H8300CPU(cs);
    CPUH8300State *env = &cpu->env;
    int do_irq = cs->interrupt_request & CPU_INTERRUPT_HARD;
    uint32_t save_ccr_pc;

    env->in_sleep = 0;

    save_ccr_pc = deposit32(0, 24, 8, h8300_cpu_pack_ccr(env));
    save_ccr_pc = deposit32(save_ccr_pc, 0, 24, env->pc);
    env->ccr_i = 1;
    if (!FIELD_EX8(env->syscr, SYSCR, UE)) {
        env->ccr_ui = 1;
    }

    if (do_irq) {
        env->regs[7] -= 4;
        cpu_stl_all(env, env->regs[7], save_ccr_pc);
        env->pc = cpu_ldl_all(env, env->ack_irq * 4);
        cs->interrupt_request &= ~CPU_INTERRUPT_HARD;
        qemu_set_irq(env->ack, env->ack_irq);
        qemu_log_mask(CPU_LOG_INT,
                      "interrupt 0x%02x raised\n", env->ack_irq);
    }
}

bool h8300_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    H8300CPU *cpu = H8300CPU(cs);
    CPUH8300State *env = &cpu->env;
    int pri;

    if (!FIELD_EX8(env->syscr, SYSCR, UE)) {
        pri = (env->ccr_i << 1 | env->ccr_ui);
    } else {
        pri = 3 * env->ccr_i;
    }
    if ((interrupt_request & CPU_INTERRUPT_HARD) &&
        pri <= env->req_pri) {
        env->ack_irq = env->req_irq;
        h8300_cpu_do_interrupt(cs);
        return true;
    }
    return false;
}

hwaddr h8300_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    return addr;
}
