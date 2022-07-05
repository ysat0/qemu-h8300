/*
 * H8/300 gdb server stub
 *
 * Copyright (c) 2019 Yoshinori Sato
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
#include "qemu-common.h"
#include "cpu.h"
#include "exec/gdbstub.h"

int h8300_cpu_gdb_read_register(CPUState *cs, GByteArray *mem_buf, int n)
{
    H8300CPU *cpu = H8300_CPU(cs);
    CPUH8300State *env = &cpu->env;

    switch (n) {
    case 0 ... 7:
        return gdb_get_regl(mem_buf, env->regs[n]);
    case 8:
        return gdb_get_regl(mem_buf, h8300_cpu_pack_ccr(env));
    case 9:
        return gdb_get_regl(mem_buf, env->pc);
    }
    return 0;
}

int h8300_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    H8300CPU *cpu = H8300_CPU(cs);
    CPUH8300State *env = &cpu->env;
    uint32_t ccr;
    switch (n) {
    case 0 ... 7:
        env->regs[n] = ldl_p(mem_buf);
        break;
    case 8:
        ccr = ldl_p(mem_buf);
        h8300_cpu_unpack_ccr(env, ccr);
        break;
    case 9:
        env->pc = ldl_p(mem_buf);
        break;
    default:
        return 0;
    }

    return 4;
}
