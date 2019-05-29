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
#include "qemu-common.h"
#include "hw/registerfields.h"
#include "qom/cpu.h"

#define TYPE_RXCPU "h8300cpu"

#define RXCPU_CLASS(klass)                                     \
    OBJECT_CLASS_CHECK(RXCPUClass, (klass), TYPE_RXCPU)
#define RXCPU(obj) \
    OBJECT_CHECK(RXCPU, (obj), TYPE_RXCPU)
#define RXCPU_GET_CLASS(obj) \
    OBJECT_GET_CLASS(RXCPUClass, (obj), TYPE_RXCPU)

/*
 * H8300CPUClass:
 * @parent_realize: The parent class' realize handler.
 * @parent_reset: The parent class' reset handler.
 *
 * A H8300 CPU model.
 */
typedef struct H8300CPUClass {
    /*< private >*/
    CPUClass parent_class;
    /*< public >*/

    DeviceRealize parent_realize;
    void (*parent_reset)(CPUState *cpu);

} H8300CPUClass;

#define TARGET_LONG_BITS 32
#define TARGET_PAGE_BITS 12

#define CPUArchState struct CPUH8300State

#include "exec/cpu-defs.h"

#define TARGET_PHYS_ADDR_SPACE_BITS 32
#define TARGET_VIRT_ADDR_SPACE_BITS 32

/* CCR define */
REG8(CCR, 0)
FIELD(PSW, C, 0, 1)
FIELD(PSW, V, 1, 1)
FIELD(PSW, Z, 2, 1)
FIELD(PSW, N, 3, 1)
FIELD(PSW, U, 4, 1)
FIELD(PSW, H, 5, 1)
FIELD(PSW, UI, 6, 1)
FIELD(PSW, I,  7, 4)


#define NB_MMU_MODES 1
#define MMU_MODE0_SUFFIX _all

enum {
    NUM_REGS = 8,
};

typedef struct CPUH8300State {
    /* CPU registers */
    uint32_t regs[NUM_REGS];    /* general registers */
    uint32_t psw_c;             /* O bit of status register */
    uint32_t psw_v;             /* S bit of status register */
    uint32_t psw_z;             /* Z bit of status register */
    uint32_t psw_n;             /* C bit of status register */
    uint32_t psw_u;
    uint32_t psw_h;
    uint32_t psw_ui;
    uint32_t psw_i;
    uint32_t pc;                /* program counter */

    /* Fields up to this point are cleared by a CPU reset */
    struct {} end_reset_fields;

    /* Internal use */
    uint32_t in_sleep;
    uint32_t req_irq;           /* Requested interrupt no (hard) */
    uint32_t req_ipl;           /* Requested interrupt level */
    uint32_t ack_irq;           /* execute irq */
    uint32_t ack_ipl;           /* execute ipl */
    qemu_irq ack;		/* Interrupt acknowledge */

    CPU_COMMON
} CPUH8300State;

/*
 * H8300CPU:
 * @env: #CPUH8300State
 *
 * A H8300 CPU
 */
struct H830CPU {
    /*< private >*/
    CPUState parent_obj;
    /*< public >*/

    CPUH8300State env;
};

typedef struct H8300CPU H8300CPU;

static inline H8300CPU *h8300_env_get_cpu(CPUH8300State *env)
{
    return container_of(env, H8300CPU, env);
}

#define ENV_GET_CPU(e) CPU(h8300_env_get_cpu(e))

#define ENV_OFFSET offsetof(H8300CPU, env)

#define RX_CPU_TYPE_SUFFIX "-" TYPE_H8300CPU
#define RX_CPU_TYPE_NAME(model) model H8300_CPU_TYPE_SUFFIX
#define CPU_RESOLVING_TYPE TYPE_H8300CPU

void h8300_cpu_do_interrupt(CPUState *cpu);
bool h8300_cpu_exec_interrupt(CPUState *cpu, int int_req);
void h8300_cpu_dump_state(CPUState *cpu, FILE *f, int flags);
int h8300_cpu_gdb_read_register(CPUState *cpu, uint8_t *buf, int reg);
int h8300_cpu_gdb_write_register(CPUState *cpu, uint8_t *buf, int reg);
hwaddr h8300_cpu_get_phys_page_debug(CPUState *cpu, vaddr addr);

void h8300_translate_init(void);
int cpu_h8300_signal_handler(int host_signum, void *pinfo,
                           void *puc);

void h8300_cpu_list(void);
void h8300_load_image(RXCPU *cpu, const char *filename,
                   uint32_t start, uint32_t size);
void h8300_cpu_unpack_psw(CPURXState *env, uint32_t psw, int rte);

#define cpu_signal_handler cpu_h8300_signal_handler
#define cpu_list h8300_cpu_list

#include "exec/cpu-all.h"

#define CPU_INTERRUPT_SOFT CPU_INTERRUPT_TGT_INT_0

#define H8300_CPU_IRQ 0

static inline void cpu_get_tb_cpu_state(CPURXState *env, target_ulong *pc,
                                        target_ulong *cs_base, uint32_t *flags)
{
    *pc = env->pc;
    *cs_base = 0;
    *flags = 0;
}

static inline int cpu_mmu_index(CPURXState *env, bool ifetch)
{
    return 0;
}

static inline uint32_t h8300_cpu_pack_ccr(CPURXState *env)
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

#endif /* H8300_CPU_H */
