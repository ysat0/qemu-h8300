/*
 *  H8/300 helper functions
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
#include "exec/exec-all.h"
#include "exec/helper-proto.h"
#include "exec/cpu_ldst.h"
#include "fpu/softfloat.h"

void helper_set_ccr(CPUH8300State *env, uint32_t ccr)
{
    h8300_cpu_unpack_ccr(env, ccr);
}

uint32_t helper_get_ccr(CPUH8300State *env)
{
    return h8300_cpu_pack_ccr(env);
}

/* div */
uint32_t helper_div(CPUH8300State *env, uint32_t num, uint32_t den)
{
    uint32_t q = num, r = 0, ret;
    if (!((num == INT_MIN && den == -1) || den == 0)) {
        q = (int32_t)num / (int32_t)den;
        r = (int32_t)num % (int32_t)den;
    }
    ret = deposit32(0, 0, 16, q);
    ret = deposit32(ret, 16, 16, r);
    return ret;
}

uint32_t helper_divu(CPUH8300State *env, uint32_t num, uint32_t den)
{
    uint32_t q = num, r = 0, ret;
    if (den != 0) {
        q = num / den;
        r = num % den;
    }
    ret = deposit32(0, 0, 16, q);
    ret = deposit32(ret, 16, 16, r);
    return ret;
}

uint32_t helper_daa(CPUH8300State *env, uint32_t num)
{
    int h, l, added;
    h = num >> 4;
    l = num & 0x0f;
    added = -1;
    if (!env->ccr_c && h < 10 && !env->ccr_h && l < 10) {
        added = 0;
        env->ccr_c = 0;
    } else if (!env->ccr_c && h < 9 && !env->ccr_h && l >= 10) {
        added = 0x06;
        env->ccr_c = 0;
    } else if (!env->ccr_c && h < 10 && env->ccr_h && l < 4) {
        added = 0x06;
        env->ccr_c = 0;
    } else if (!env->ccr_c && h >= 10 && !env->ccr_h && l < 10) {
        added = 0x60;
        env->ccr_c = 1;
    } else if (!env->ccr_c && h >= 9 && !env->ccr_h && l >= 10) {
        added = 0x66;
        env->ccr_c = 1;
    } else if (env->ccr_c && (h > 0 && h < 3) && !env->ccr_h && l < 10) {
        added = 0x60;
        env->ccr_c = 1;
    } else if (env->ccr_c && (h > 0 && h < 3) && !env->ccr_h && l >= 10) {
        added = 0x66;
        env->ccr_c = 1;
    } else if (env->ccr_c && (h > 0 && h < 4) && env->ccr_h && l < 4) {
        added = 0x66;
        env->ccr_c = 1;
    }
    if (added == -1) {
        qemu_log_mask(LOG_GUEST_ERROR, "Invalid DAA value");
        return num;
    } else {
        return num + added;
    }
}

uint32_t helper_das(CPUH8300State *env, uint32_t num)
{
    int h, l, added;
    h = num >> 4;
    l = num & 0x0f;
    added = -1;
    if (!env->ccr_c && h < 10 && !env->ccr_h && l < 10) {
        added = 0;
        env->ccr_c = 0;
    } else if (!env->ccr_c && h < 9 && env->ccr_h && l >= 6) {
        added = 0xfa;
        env->ccr_c = 0;
    } else if (env->ccr_c && h >= 7 && !env->ccr_h && l < 10) {
        added = 0xa0;
        env->ccr_c = 1;
    } else if (env->ccr_c && h >= 6 && env->ccr_h && l >= 6) {
        added = 0x9a;
        env->ccr_c = 1;
    }
    if (added == -1) {
        qemu_log_mask(LOG_GUEST_ERROR, "Invalid DAS value");
        return num;
    } else {
        return num + added;
    }
}

void helper_eepmovb(CPUH8300State *env)
{
    int cnt;
    uint8_t tmp;
    cnt = extract32(env->regs[4], 0, 8);
    while(cnt > 0) {
        tmp = cpu_ldub_data_ra(env, env->regs[5], GETPC());
        cpu_stb_data_ra(env, env->regs[6], tmp, GETPC());
        env->regs[5]++;
        env->regs[6]++;
        cnt--;
    }
    env->regs[4] = deposit32(env->regs[4], cnt, 0, 8);
}

void helper_eepmovw(CPUH8300State *env)
{
    int cnt;
    uint8_t tmp;
    cnt = extract32(env->regs[4], 0, 16);
    while(cnt > 0) {
        tmp = cpu_ldub_data_ra(env, env->regs[5], GETPC());
        cpu_stb_data_ra(env, env->regs[6], tmp, GETPC());
        env->regs[5]++;
        env->regs[6]++;
        cnt--;
    }
    env->regs[4] = deposit32(env->regs[4], cnt, 0, 16);
}

/* exception */
static inline void QEMU_NORETURN raise_exception(CPUH8300State *env, int index,
                                                 uintptr_t retaddr)
{
    CPUState *cs = CPU(h8300_env_get_cpu(env));

    cs->exception_index = index;
    cpu_loop_exit_restore(cs, retaddr);
}

void QEMU_NORETURN helper_sleep(CPUH8300State *env)
{
    CPUState *cs = CPU(h8300_env_get_cpu(env));

    cs->halted = 1;
    env->in_sleep = 1;
    raise_exception(env, EXCP_HLT, 0);
}

void QEMU_NORETURN helper_debug(CPUH8300State *env)
{
    CPUState *cs = CPU(h8300_env_get_cpu(env));

    cs->exception_index = EXCP_DEBUG;
    cpu_loop_exit(cs);
}

void QEMU_NORETURN helper_trapa(CPUH8300State *env, uint32_t vec)
{
    raise_exception(env, 0x100 + vec, 0);
}
