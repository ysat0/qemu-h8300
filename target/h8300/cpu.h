/*
 *  H8/300 emulation definition
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

#ifndef H8300_CPU_H
#define H8300_CPU_H

#include "qemu/bitops.h"
#include "hw/registerfields.h"
#include "cpu-qom.h"

#include "exec/cpu-defs.h"

/* CCR define */
REG8(CCR, 0)
FIELD(CCR, C, 0, 1)
FIELD(CCR, V, 1, 1)
FIELD(CCR, Z, 2, 1)
FIELD(CCR, N, 3, 1)
FIELD(CCR, U, 4, 1)
FIELD(CCR, H, 5, 1)
FIELD(CCR, UI, 6, 1)
FIELD(CCR, I,  7, 1)

REG8(EXR, 0)
FIELD(EXR, I0, 0, 1)
FIELD(EXR, I1, 1, 1)
FIELD(EXR, I2, 2, 1)
FIELD(EXR, I, 0, 3)
FIELD(EXR, T, 7, 1)

/* SYSCR */
REG8(SYSCR, 0)
FIELD(SYSCR, RAME,  0, 1)
FIELD(SYSCR, SSOE,  1, 1)
FIELD(SYSCR, NMIEG, 2, 1)
FIELD(SYSCR, UE,    3, 1)
FIELD(SYSCR, STS,   4, 3)
FIELD(SYSCR, SSBY,  7, 1)

enum {
    NUM_REGS = 8,
};

typedef struct CPUArchState {
    /* CPU registers */
    uint32_t regs[NUM_REGS];    /* general registers */
    uint32_t ccr_c;             /* C bit of status register */
    uint32_t ccr_v;             /* V bit of status register */
    uint32_t ccr_z;             /* Z bit of status register */
    uint32_t ccr_n;             /* N bit of status register */
    uint32_t ccr_u;
    uint32_t ccr_h;
    uint32_t ccr_ui;
    uint32_t ccr_i;
    uint32_t exr_i;
    uint32_t exr_t;
    uint32_t pc;                /* program counter */
    uint64_t mac;
    uint64_t mult_z;
    uint32_t mult_n;
    uint32_t mult_v;
    
    /* Fields up to this point are cleared by a CPU reset */
    struct {} end_reset_fields;

    /* Internal use */
    uint32_t in_sleep;
    uint32_t req_irq;           /* Requested interrupt no (hard) */
    uint32_t ack_irq;           /* execute irq */
    uint32_t req_pri;
    uint32_t im;
    qemu_irq ack;		/* Interrupt acknowledge */
} CPUH8300State;

/*
 * H8300CPU:
 * @env: #CPUH8300State
 *
 * A H8300 CPU
 */
/*
 * H8300CPU:
 * @env: #CPUH8300State
 *
 * A H8300 CPU
 */
struct ArchCPU {
    /*< private >*/
    CPUState parent_obj;
    /*< public >*/

    CPUH8300State env;
};

/*
 * H8300CPUClass:
 * @parent_realize: The parent class' realize handler.
 * @parent_phases: The parent class' reset phase handlers.
 *
 * A H8300 CPU model.
 */
struct H8300CPUClass {
    CPUClass parent_class;

    DeviceRealize parent_realize;
    ResettablePhases parent_phases;
};

#define CPU_RESOLVING_TYPE TYPE_H8300_CPU

void h8300_cpu_do_interrupt(CPUState *cpu);
bool h8300_cpu_exec_interrupt(CPUState *cpu, int int_req);
void h8300_cpu_dump_state(CPUState *cpu, FILE *f, int flags);
int h8300_cpu_gdb_read_register(CPUState *cpu, GByteArray *buf, int reg);
int h8300_cpu_gdb_write_register(CPUState *cpu, uint8_t *buf, int reg);
hwaddr h8300_cpu_get_phys_page_debug(CPUState *cpu, vaddr addr);

void h8300_translate_init(void);
int cpu_h8300_signal_handler(int host_signum, void *pinfo,
                           void *puc);

void h8300_cpu_list(void);
void h8300_load_image(H8300CPU *cpu, const char *filename,
                   uint32_t start, uint32_t size);
void h8300_cpu_unpack_ccr(CPUH8300State *env, uint32_t ccr);
void h8300_cpu_unpack_exr(CPUH8300State *env, uint32_t exr);
void h8300_cpu_setim(int im);

#define cpu_signal_handler cpu_h8300_signal_handler
#define cpu_list h8300_cpu_list

#include "exec/cpu-all.h"

#define CPU_INTERRUPT_SOFT CPU_INTERRUPT_TGT_INT_0

#define H8300_CPU_IRQ 0

static inline void cpu_get_tb_cpu_state(CPUH8300State *env, vaddr *pc,
                                        uint64_t *cs_base, uint32_t *flags)
{
    *pc = env->pc;
    *cs_base = 0;
    *flags = env->im;
}

static inline int cpu_mmu_index(CPUH8300State *env, bool ifetch)
{
    return 0;
}

static inline uint32_t h8300_cpu_pack_ccr(CPUH8300State *env)
{
    uint32_t ccr = 0;
    ccr = FIELD_DP32(ccr, CCR, I,  env->ccr_i);
    ccr = FIELD_DP32(ccr, CCR, UI, env->ccr_ui);
    ccr = FIELD_DP32(ccr, CCR, H,   env->ccr_h);
    ccr = FIELD_DP32(ccr, CCR, U,   env->ccr_u);
    ccr = FIELD_DP32(ccr, CCR, N,   env->ccr_n >> 31);
    ccr = FIELD_DP32(ccr, CCR, Z,   env->ccr_z == 0);
    ccr = FIELD_DP32(ccr, CCR, V,   env->ccr_v >> 31);
    ccr = FIELD_DP32(ccr, CCR, C,   env->ccr_c);
    return ccr;
}

static inline uint32_t h8300_cpu_pack_exr(CPUH8300State *env)
{
    uint32_t exr = 0;
    exr = FIELD_DP32(exr, EXR, I,  env->exr_i);
    exr = FIELD_DP32(exr, EXR, T,  env->exr_t);
    return exr;
}

#endif /* H8300_CPU_H */
