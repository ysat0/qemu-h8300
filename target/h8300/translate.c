/*
 *  H8/300 translation
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
#include "qemu/bswap.h"
#include "qemu/qemu-print.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "tcg/tcg-op.h"
#include "exec/cpu_ldst.h"
#include "exec/helper-proto.h"
#include "exec/helper-gen.h"
#include "exec/translator.h"
#include "exec/log.h"

typedef struct DisasContext {
    DisasContextBase base;
    CPUH8300State *env;
    uint32_t pc;
} DisasContext;

typedef struct DisasCompare {
    TCGv value;
    TCGv temp;
    TCGCond cond;
} DisasCompare;

/* Target-specific values for dc->base.is_jmp.  */
#define DISAS_JUMP    DISAS_TARGET_0
#define DISAS_UPDATE  DISAS_TARGET_1
#define DISAS_EXIT    DISAS_TARGET_2

/* global register indexes */
static TCGv cpu_regs[8];
static TCGv cpu_ccr_c, cpu_ccr_v, cpu_ccr_z, cpu_ccr_n;
static TCGv cpu_ccr_u, cpu_ccr_h, cpu_ccr_ui, cpu_ccr_i;
static TCGv cpu_pc;

#define cpu_sp cpu_regs[7]

#include "exec/gen-icount.h"

/* decoder helper */
static uint32_t decode_load_bytes(DisasContext *ctx, uint32_t insn,
                           int i, int n)
{
    while (++i <= n) {
        uint8_t b = cpu_ldub_code(ctx->env, ctx->base.pc_next++);
        insn |= b << (32 - i * 8);
    }
    return insn;
}

static uint32_t imm32(DisasContext *ctx, int dummy)
{
    CPUH8300State *env = ctx->env;
    uint32_t addr = ctx->base.pc_next;

    ctx->base.pc_next += 4;
    return cpu_ldl_code(env, addr);
}

static uint32_t dsp16(DisasContext *ctx, int dummy)
{
    CPUH8300State *env = ctx->env;
    uint32_t addr = ctx->base.pc_next;

    ctx->base.pc_next += 2;
    return cpu_ldsw_code(env, addr);
}

static uint32_t dsp32_4(DisasContext *ctx, int dummy)
{
    CPUH8300State *env = ctx->env;
    uint32_t dsp32;
    uint32_t addr = ctx->pc + 4;

    ctx->base.pc_next = ctx->pc + 8;
    dsp32 = cpu_ldl_code(env, addr);
    return dsp32;
}

static uint32_t dsp32_6(DisasContext *ctx, int dummy)
{
    CPUH8300State *env = ctx->env;
    uint32_t dsp32;
    uint32_t addr = ctx->pc + 6;

    ctx->base.pc_next = ctx->pc + 10;
    dsp32 = cpu_ldl_code(env, addr);
    return dsp32;
}

static uint32_t abs16(DisasContext *ctx, int dummy)
{
    CPUH8300State *env = ctx->env;
    uint32_t addr = ctx->base.pc_next;

    ctx->base.pc_next += 2;
    return cpu_ldsw_code(env, addr);
}

static uint32_t abs32(DisasContext *ctx, int dummy)
{
    CPUH8300State *env = ctx->env;
    uint32_t addr = ctx->base.pc_next;

    ctx->base.pc_next += 4;
    return cpu_ldl_code(env, addr);
}

static uint32_t b10_er(DisasContext *ctx, int dummy)
{
    CPUH8300State *env = ctx->env;
    uint32_t addr = ctx->pc + 5;

    return extract32(cpu_ldub_code(env, addr), 0, 3);
}

static uint32_t b10_ldst(DisasContext *ctx, int dummy)
{
    CPUH8300State *env = ctx->env;
    uint32_t addr = ctx->pc + 5;

    return extract32(cpu_ldub_code(env, addr), 4, 4);
}

#if 0
static int dummy(uint32_t insn, int pos, int len)
{
    return 0;
}
#endif

static int adds_imm(DisasContext *ctx, int imm)
{
    switch(imm) {
    case 0:
        return 1;
    case 2:
        return 2;
    case 3:
        return 4;
    default:
        return 0;
    }
}

static int incdec(DisasContext *ctx, int imm)
{
    return imm + 1;
}

static int sz013(DisasContext *ctx, int imm)
{
    switch(imm) {
    case 0:
    case 1:
        return imm;
    case 2:
    case 3:
        return 2;
    default:
        return -1;
    }
}

static int b6_bop_ir(DisasContext *ctx, int imm)
{
    CPUH8300State *env = ctx->env;
    uint32_t addr = ctx->pc + 5;

    return extract32(cpu_ldub_code(env, addr), 4, 3);
}

static int b6_bop_op(DisasContext *ctx, int imm)
{
    CPUH8300State *env = ctx->env;
    uint32_t addr = ctx->pc + 4;
    uint32_t inv = extract32(cpu_ldub_code(env, addr + 1), 7, 1);

    ctx->base.pc_next = ctx->pc + 6;
    return (inv << 8) | cpu_ldub_code(env, addr);
}

static int b8_bop_ir(DisasContext *ctx, int imm)
{
    CPUH8300State *env = ctx->env;
    uint32_t addr = ctx->pc + 7;

    return extract32(cpu_ldub_code(env, addr), 4, 3);
}

static int b8_bop_op(DisasContext *ctx, int imm)
{
    CPUH8300State *env = ctx->env;
    uint32_t addr = ctx->pc + 6;

    uint32_t inv = extract32(cpu_ldub_code(env, addr + 1), 7, 1);
    ctx->base.pc_next = ctx->pc + 8;
    return (inv << 8) | cpu_ldub_code(env, addr);
}

static int b8_bop_abs(DisasContext *ctx, int imm)
{
    CPUH8300State *env = ctx->env;
    uint32_t addr = ctx->pc + 2;

    return cpu_ldl_code(env, addr);
}

static int b4_bop_abs(DisasContext *ctx, int abs)
{
    return 0xffff00 | (uint32_t)abs;
}

/* Include the auto-generated decoder. */
#include "decode-insns.c.inc"

void h8300_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    H8300CPU *cpu = H8300_CPU(cs);
    CPUH8300State *env = &cpu->env;
    int i;
    uint32_t ccr;

    ccr = h8300_cpu_pack_ccr(env);
    qemu_fprintf(f, "pc=0x%08x ccr=0x%02x\n",
                 env->pc, ccr);
    for (i = 0; i < 8; i += 4) {
        qemu_fprintf(f, "er%d=0x%08x er%d=0x%08x er%d=0x%08x er%d=0x%08x\n",
                     i, env->regs[i], i + 1, env->regs[i + 1],
                     i + 2, env->regs[i + 2], i + 3, env->regs[i + 3]);
    }
}

static bool use_goto_tb(DisasContext *dc, target_ulong dest)
{
    if (unlikely(dc->base.singlestep_enabled)) {
        return false;
    } else {
        return true;
    }
}

static void gen_goto_tb(DisasContext *dc, int n, target_ulong dest)
{
    if (use_goto_tb(dc, dest)) {
        tcg_gen_goto_tb(n);
        tcg_gen_movi_i32(cpu_pc, dest);
        tcg_gen_exit_tb(dc->base.tb, n);
    } else {
        tcg_gen_movi_i32(cpu_pc, dest);
        if (dc->base.singlestep_enabled) {
            gen_helper_debug(cpu_env);
        } else {
            tcg_gen_lookup_and_goto_ptr();
        }
    }
    dc->base.is_jmp = DISAS_NORETURN;
}

/* generate QEMU condition */
static void ccr_cond(DisasCompare *dc, uint32_t cond)
{
    tcg_debug_assert(cond < 16);
    switch (cond) {
    case 2: /* !(c | z) */
    case 3: /* c | z */
        tcg_gen_setcondi_i32(TCG_COND_EQ, dc->temp, cpu_ccr_z, 0);
        tcg_gen_or_i32(dc->temp, dc->temp, cpu_ccr_c);
        dc->cond = (cond == 2) ? TCG_COND_EQ : TCG_COND_NE;
        dc->value = dc->temp;
        break;
    case 4: /* !c */
        dc->cond = TCG_COND_EQ;
        dc->value = cpu_ccr_c;
        break;
    case 5: /* c */
        dc->cond = TCG_COND_NE;
        dc->value = cpu_ccr_c;
        break;
    case 6: /* !z */
        dc->cond = TCG_COND_NE;
        dc->value = cpu_ccr_z;
        break;
    case 7: /* z */
        dc->cond = TCG_COND_EQ;
        dc->value = cpu_ccr_z;
        break;
    case 8: /* !v */
        dc->cond = TCG_COND_GE;
        dc->value = cpu_ccr_v;
        break;
    case 9: /* v */
        dc->cond = TCG_COND_LT;
        dc->value = cpu_ccr_v;
        break;
    case 10: /* !n */
        dc->cond = TCG_COND_GE;
        dc->value = cpu_ccr_n;
        break;
    case 11: /* n */
        dc->cond = TCG_COND_LT;
        dc->value = cpu_ccr_n;
        break;
    case 12: /* !(n^v) */
    case 13: /* n^v */
        tcg_gen_xor_i32(dc->temp, cpu_ccr_v, cpu_ccr_n);
        dc->cond = (cond == 12) ? TCG_COND_GE : TCG_COND_LT;
        dc->value = dc->temp;
        break;
    case 14: /* !((n^v) | z) */
    case 15: /*  ((n^v) | z) */
        tcg_gen_xor_i32(dc->temp, cpu_ccr_v, cpu_ccr_n);
        tcg_gen_sari_i32(dc->temp, dc->temp, 31);
        tcg_gen_andc_i32(dc->temp, cpu_ccr_z, dc->temp);
        dc->cond = (cond == 14) ? TCG_COND_NE : TCG_COND_EQ;
        dc->value = dc->temp;
        break;
    }
}

static inline void h8300_gen_reg_ldb(int rn, TCGv val, bool sign)
{
    int base;
    g_assert(rn < 16);
    if (rn < 8) {
        base = 8;
        tcg_gen_sextract_i32(val, cpu_regs[rn], 8, 8);
    } else {
        base = 0;
        rn &= 7;
        tcg_gen_sextract_i32(val, cpu_regs[rn & 7], 0, 8);
    }
    if (sign) {
        tcg_gen_sextract_i32(val, cpu_regs[rn], base, 8);
    } else {
        tcg_gen_extract_i32(val, cpu_regs[rn], base, 8);
    }
}

static inline void h8300_gen_reg_stb(int rn, TCGv val)
{
    g_assert(rn < 16);
    if (rn < 8) {
        tcg_gen_deposit_i32(cpu_regs[rn], cpu_regs[rn], val, 8, 8);
    } else {
        rn &= 7;
        tcg_gen_deposit_i32(cpu_regs[rn], cpu_regs[rn], val, 0, 8);
    }
}

static inline void h8300_gen_reg_ldw(int rn, TCGv val, bool sign)
{
    int base;
    g_assert(rn < 16);
    if (rn < 8) {
        base = 0;
    } else {
        base = 16;
        rn &= 7;
    }
    if (sign) {
        tcg_gen_sextract_i32(val, cpu_regs[rn], base, 16);
    } else {
        tcg_gen_extract_i32(val, cpu_regs[rn], base, 16);
    }
}

static inline void h8300_gen_reg_stw(int rn, TCGv val)
{
    g_assert(rn < 16);
    if (rn < 8) {
        tcg_gen_deposit_i32(cpu_regs[rn], cpu_regs[rn], val, 0, 16);
    } else {
        rn &= 7;
        tcg_gen_deposit_i32(cpu_regs[rn], cpu_regs[rn], val, 16, 16);
    }
}

enum {
    SZ_B, SZ_W, SZ_L,
};

static inline TCGv h8300_reg_ld(int sz, int rn, TCGv val, bool sign)
{
    switch(sz) {
    case SZ_B:
        h8300_gen_reg_ldb(rn, val, sign);
        return val;
    case SZ_W:
        h8300_gen_reg_ldw(rn, val, sign);
        return val;
    case SZ_L:
        return cpu_regs[rn & 7];
    default:
        g_assert_not_reached();
    }
}

static inline void h8300_reg_st(int sz, int rn, TCGv val)
{
    switch(sz) {
    case SZ_B:
        h8300_gen_reg_stb(rn, val);
        break;
    case SZ_W:
        h8300_gen_reg_stw(rn, val);
        break;
    case SZ_L:
        break;
    default:
        g_assert_not_reached();
    }
}

static bool trans_MOV_i(DisasContext *ctx, arg_MOV_i *a)
{
    TCGv imm = tcg_const_i32(a->imm);
    switch(a->sz) {
    case SZ_B:
        h8300_gen_reg_stb(a->rd, imm);
        break;
    case SZ_W:
        h8300_gen_reg_stw(a->rd, imm);
        break;
    case SZ_L:
        tcg_gen_mov_i32(cpu_regs[a->rd & 7], imm);
        break;
    }
    tcg_gen_mov_i32(cpu_ccr_z, imm);
    tcg_gen_mov_i32(cpu_ccr_n, imm);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(imm);
    return true;
}

static bool trans_MOV_r(DisasContext *ctx, arg_MOV_r *a)
{
    TCGv temp = tcg_temp_new();
    switch(a->sz) {
    case SZ_B:
        h8300_gen_reg_ldb(a->rs, temp, false);
        h8300_gen_reg_stb(a->rd, temp);
        break;
    case SZ_W:
        h8300_gen_reg_ldw(a->rs, temp, false);
        h8300_gen_reg_stw(a->rd, temp);
        break;
    case SZ_L:
        a->rs &= 7;
        a->rd &= 7;
        tcg_gen_mov_i32(cpu_regs[a->rd], cpu_regs[a->rs]);
        tcg_gen_mov_i32(temp, cpu_regs[a->rs]);
        break;
    }
    tcg_gen_mov_i32(cpu_ccr_z, temp);
    tcg_gen_mov_i32(cpu_ccr_n, temp);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(temp);
    return true;
}

static bool trans_MOV_mr(DisasContext *ctx, arg_MOV_mr *a)
{
    TCGv temp = tcg_temp_new();
    TCGv mem = tcg_temp_new();

    tcg_gen_addi_i32(mem, cpu_regs[a->er], a->dsp);
    tcg_gen_qemu_ld_i32(temp, mem, 0, a->sz | MO_SIGN | MO_TE);
    switch(a->sz) {
    case SZ_B:
        h8300_gen_reg_stb(a->r, temp);
        break;
    case SZ_W:
        h8300_gen_reg_stw(a->r, temp);
        break;
    case SZ_L:
        tcg_gen_mov_i32(cpu_regs[a->r & 7], temp);
        break;
    }
    tcg_gen_mov_i32(cpu_ccr_z, temp);
    tcg_gen_mov_i32(cpu_ccr_n, temp);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(mem);
    tcg_temp_free(temp);
    return true;
}

static bool trans_MOV_mpr(DisasContext *ctx, arg_MOV_mr *a)
{
    TCGv temp = tcg_temp_new();

    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->er], 0, a->sz | MO_SIGN | MO_TE);
    tcg_gen_addi_i32(cpu_regs[a->er], cpu_regs[a->er], 1 << a->sz);
    switch(a->sz) {
    case SZ_B:
        h8300_gen_reg_stb(a->r, temp);
        break;
    case SZ_W:
        h8300_gen_reg_stw(a->r, temp);
        break;
    case SZ_L:
        tcg_gen_mov_i32(cpu_regs[a->r & 7], temp);
        break;
    }
    tcg_gen_mov_i32(cpu_ccr_z, temp);
    tcg_gen_mov_i32(cpu_ccr_n, temp);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(temp);
    return true;
}

static bool trans_MOV_ar(DisasContext *ctx, arg_MOV_ar *a)
{
    TCGv temp = tcg_temp_new();
    TCGv mem;

    switch(a->a) {
    case 8:
        a->abs |= 0x00ffff00;
        break;
    case 16:
        a->abs = sextract32(a->abs, 0, 16) & 0x00ffffff;
        break;
    }
    mem = tcg_const_i32(a->abs);
    tcg_gen_qemu_ld_i32(temp, mem, 0, a->sz | MO_TE);
    switch(a->sz) {
    case SZ_B:
        h8300_gen_reg_stb(a->r, temp);
        break;
    case SZ_W:
        h8300_gen_reg_stw(a->r, temp);
        break;
    case SZ_L:
        tcg_gen_mov_i32(cpu_regs[a->r & 7], temp);
        break;
    }
    tcg_gen_mov_i32(cpu_ccr_z, temp);
    tcg_gen_mov_i32(cpu_ccr_n, temp);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(mem);
    tcg_temp_free(temp);
    return true;
}

static bool trans_MOV_rm(DisasContext *ctx, arg_MOV_mr *a)
{
    TCGv temp = tcg_temp_new();
    TCGv mem = tcg_temp_new();

    tcg_gen_addi_i32(mem, cpu_regs[a->er], a->dsp);
    switch(a->sz) {
    case SZ_B:
        h8300_gen_reg_ldb(a->r, temp, false);
        break;
    case SZ_W:
        h8300_gen_reg_ldw(a->r, temp, false);
        break;
    case SZ_L:
        tcg_gen_mov_i32(temp, cpu_regs[a->r & 7]);
        break;
    }
    tcg_gen_qemu_st_i32(temp, mem, 0, a->sz | MO_TE);
    tcg_gen_mov_i32(cpu_ccr_z, temp);
    tcg_gen_mov_i32(cpu_ccr_n, temp);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(mem);
    tcg_temp_free(temp);
    return true;
}

static bool trans_MOV_rmp(DisasContext *ctx, arg_MOV_mr *a)
{
    TCGv temp = tcg_temp_new();
    TCGv mem = tcg_temp_new();

    tcg_gen_subi_i32(cpu_regs[a->er], cpu_regs[a->er], 1 << a->sz);
    switch(a->sz) {
    case SZ_B:
        h8300_gen_reg_ldb(a->r, temp, false);
        break;
    case SZ_W:
        h8300_gen_reg_ldw(a->r, temp, false);
        break;
    case SZ_L:
        tcg_gen_mov_i32(temp, cpu_regs[a->r & 7]);
        break;
    }
    tcg_gen_qemu_st_i32(temp, cpu_regs[a->er], 0, a->sz | MO_TE);
    tcg_gen_mov_i32(cpu_ccr_z, temp);
    tcg_gen_mov_i32(cpu_ccr_n, temp);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(mem);
    tcg_temp_free(temp);
    return true;
}

static bool trans_MOV_ra(DisasContext *ctx, arg_MOV_ra *a)
{
    TCGv temp = tcg_temp_new();
    TCGv mem;

    switch(a->a) {
    case 8:
        a->abs |= 0x00ffff00;
        break;
    case 16:
        a->abs = sextract32(a->abs, 0, 16) & 0x00ffffff;
        break;
    }
    mem = tcg_const_i32(a->abs);
    switch(a->sz) {
    case SZ_B:
        h8300_gen_reg_ldb(a->r, temp, false);
        break;
    case SZ_W:
        h8300_gen_reg_ldw(a->r, temp, false);
        break;
    case SZ_L:
        tcg_gen_mov_i32(temp, cpu_regs[a->r & 7]);
        break;
    }
    tcg_gen_qemu_st_i32(temp, mem, 0, a->sz | MO_TE);
    tcg_gen_mov_i32(cpu_ccr_z, temp);
    tcg_gen_mov_i32(cpu_ccr_n, temp);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(mem);
    tcg_temp_free(temp);
    return true;
}

static bool trans_MOVFPE(DisasContext *ctx, arg_MOVFPE *a)
{
    TCGv mem, temp;

    a->abs = sextract32(a->abs, 0, 16) & 0x00ffffff;
    mem = tcg_const_i32(a->abs);
    temp = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 | MO_TE);
    h8300_gen_reg_stb(a->r, temp);
    tcg_temp_free(mem);
    tcg_temp_free(temp);
    return true;
}

static bool trans_MOVTPE(DisasContext *ctx, arg_MOVTPE *a)
{
    TCGv mem, temp;

    a->abs = sextract32(a->abs, 0, 16) & 0x00ffffff;
    mem = tcg_const_i32(a->abs);
    temp = tcg_temp_new();
    h8300_gen_reg_ldb(a->r, temp, false);
    tcg_gen_qemu_st_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_temp_free(mem);
    tcg_temp_free(temp);
    return true;
}

static bool trans_LDM(DisasContext *ctx, arg_LDM *a)
{
    int r;
    TCGv temp = tcg_temp_new();
    if ((a->num == 1 && (a->rn & 1) == 0) ||
        (a->num == 2 && (a->rn != 2 && a->rn != 6)) ||
        (a->num == 3 && (a->rn != 3 && a->rn != 7))) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Invalid ERn %d", a->rn);
    }
    for (r = a->rn; r >= (a->rn - a->num); r--) {
        tcg_gen_qemu_ld_i32(temp, cpu_regs[7], 0, MO_32 | MO_TE);
        tcg_gen_addi_i32(cpu_regs[7], cpu_regs[7], 4);
        tcg_gen_mov_i32(cpu_regs[r], temp);
    }
    tcg_temp_free(temp);
    return true;
}

static bool trans_STM(DisasContext *ctx, arg_STM *a)
{
    int r;
    if ((a->num == 1 && (a->rn & 1) == 0) ||
        (a->num == 2 && (a->rn != 2 && a->rn != 6)) ||
        (a->num == 3 && (a->rn != 3 && a->rn != 7))) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Invalid ERn %d", a->rn);
    }
    for (r = a->rn; r <= (a->rn + a->num); r++) {
        tcg_gen_subi_i32(cpu_regs[7], cpu_regs[7], 4);
        tcg_gen_qemu_st_i32(cpu_regs[r], cpu_regs[7], 0, MO_32 | MO_TE);
    }
    return true;
}

static inline void h8300_add(int sz, TCGv ret, TCGv arg1, TCGv arg2, bool c)
{
    TCGv z;
    z = tcg_const_i32(0);

    if (c) {
        tcg_gen_add2_i32(cpu_ccr_n, cpu_ccr_c, arg1, z, arg2, z);
    } else {
        tcg_gen_add_i32(cpu_ccr_n, arg1, arg2);
    }        
    tcg_gen_mov_i32(cpu_ccr_z, cpu_ccr_n);
    tcg_gen_xor_i32(cpu_ccr_v, cpu_ccr_n, arg1);
    switch(sz) {
    case SZ_B:
        tcg_gen_ext8s_i32(cpu_ccr_n, cpu_ccr_n);
        if (c) {
            tcg_gen_extract_i32(cpu_ccr_c, cpu_ccr_z, 8, 1);
        }
        tcg_gen_extract_i32(cpu_ccr_h, cpu_ccr_v, 4, 1);
        break;
    case SZ_W:
        tcg_gen_ext16s_i32(cpu_ccr_n, cpu_ccr_n);
        if (c) {
            tcg_gen_extract_i32(cpu_ccr_c, cpu_ccr_z, 16, 1);
        }
        tcg_gen_extract_i32(cpu_ccr_h, cpu_ccr_v, 12, 1);
        break;
    case SZ_L:
        tcg_gen_extract_i32(cpu_ccr_h, cpu_ccr_v, 28, 1);
        break;
    default:
        g_assert_not_reached();
    }
    tcg_gen_xor_i32(z, arg1, arg2);
    tcg_gen_andc_i32(cpu_ccr_v, cpu_ccr_v, z);
    tcg_gen_mov_i32(ret, cpu_ccr_n);
    tcg_temp_free(z);
}

static inline void h8300_addx(TCGv ret, TCGv arg1, TCGv arg2)
{
    TCGv z;
    z = tcg_const_i32(0);

    tcg_gen_add_i32(cpu_ccr_n, arg1, cpu_ccr_c);
    tcg_gen_add_i32(cpu_ccr_n, cpu_ccr_n, arg2);
    tcg_gen_mov_i32(cpu_ccr_z, cpu_ccr_n);
    tcg_gen_ext8s_i32(cpu_ccr_n, cpu_ccr_n);
    tcg_gen_xor_i32(cpu_ccr_v, cpu_ccr_n, arg1);
    tcg_gen_extract_i32(cpu_ccr_h, cpu_ccr_v, 4, 1);
    tcg_gen_extract_i32(cpu_ccr_c, cpu_ccr_v, 8, 1);
    tcg_gen_xor_i32(z, arg1, arg2);
    tcg_gen_andc_i32(cpu_ccr_v, cpu_ccr_v, z);
    tcg_gen_mov_i32(ret, cpu_ccr_n);
    tcg_temp_free(z);
}

static void h8300_sub(int sz, TCGv ret, TCGv arg1, TCGv arg2, bool c)
{
    TCGv temp;
    tcg_gen_sub_i32(cpu_ccr_n, arg1, arg2);
    tcg_gen_mov_i32(cpu_ccr_z, cpu_ccr_n);
    if (c) {
        tcg_gen_setcond_i32(TCG_COND_LTU, cpu_ccr_c, arg1, arg2);
    }
    tcg_gen_xor_i32(cpu_ccr_v, cpu_ccr_n, arg1);
    switch(sz) {
    case SZ_B:
        tcg_gen_ext8s_i32(cpu_ccr_n, cpu_ccr_n);
        if (c) {
            tcg_gen_extract_i32(cpu_ccr_c, cpu_ccr_v, 8, 1);
        }
        tcg_gen_extract_i32(cpu_ccr_h, cpu_ccr_v, 4, 1);
        break;
    case SZ_W:
        tcg_gen_ext16s_i32(cpu_ccr_n, cpu_ccr_n);
        if (c) {
            tcg_gen_extract_i32(cpu_ccr_c, cpu_ccr_v, 16, 1);
        }
        tcg_gen_extract_i32(cpu_ccr_h, cpu_ccr_v, 12, 1);
        break;
    case SZ_L:
        tcg_gen_extract_i32(cpu_ccr_h, cpu_ccr_v, 28, 1);
        break;
    default:
        g_assert_not_reached();
    }
    temp = tcg_temp_new_i32();
    tcg_gen_xor_i32(temp, arg1, arg2);
    tcg_gen_and_i32(cpu_ccr_v, cpu_ccr_v, temp);
    tcg_temp_free_i32(temp);
    /* CMP not requred return */
    if (ret) {
        tcg_gen_mov_i32(ret, cpu_ccr_n);
    }
}

static inline void h8300_ccr_adjust(int sz)
{
    switch(sz) {
    case SZ_B:
        tcg_gen_ext8s_i32(cpu_ccr_n, cpu_ccr_n);
        tcg_gen_ext8s_i32(cpu_ccr_z, cpu_ccr_z);
        break;
    case SZ_W:
        tcg_gen_ext16s_i32(cpu_ccr_n, cpu_ccr_n);
        tcg_gen_ext16s_i32(cpu_ccr_z, cpu_ccr_z);
        break;
    case SZ_L:
        break;
    default:
        g_assert_not_reached();
    }
}

static bool trans_ADD_i(DisasContext *ctx, arg_ADD_i *a)
{
    TCGv temp, imm, reg;
    imm = tcg_const_i32(a->imm);
    temp = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->rd, temp, false);
    h8300_add(a->sz, reg, reg, imm, true);
    h8300_reg_st(a->sz, a->rd, temp);
    tcg_temp_free(temp);
    tcg_temp_free(imm);
    return true;
}

static bool trans_ADD_r(DisasContext *ctx, arg_ADD_r *a)
{
    TCGv temp1, temp2, reg1, reg2;
    temp1 = tcg_temp_new();
    temp2 = tcg_temp_new();
    reg1 = h8300_reg_ld(a->sz, a->rd, temp1, false);
    reg2 = h8300_reg_ld(a->sz, a->rs, temp2, false);
    h8300_add(a->sz, reg1, reg1, reg2, true);
    h8300_reg_st(a->sz, a->rd, reg1);
    tcg_temp_free(temp1);
    tcg_temp_free(temp2);
    return true;
}

static bool trans_ADDS(DisasContext *ctx, arg_ADDS *a)
{
    if (a->imm > 0) {
        tcg_gen_addi_i32(cpu_regs[a->rd], cpu_regs[a->rd], a->imm);
        return true;
    } else {
        return false;
    }
}

static bool trans_ADDX_i(DisasContext *ctx, arg_ADDX_i *a)
{
    TCGv temp, reg, imm;
    imm = tcg_const_i32(a->imm);
    temp = tcg_temp_new();
    reg = h8300_reg_ld(SZ_B, a->rd, temp, false);
    h8300_addx(reg, reg, imm);
    h8300_reg_st(SZ_B, a->rd, temp);
    tcg_temp_free(temp);
    tcg_temp_free(imm);
    return true;
}

static bool trans_ADDX_r(DisasContext *ctx, arg_ADDX_r *a)
{
    TCGv temp1, temp2, reg1, reg2;
    temp1 = tcg_temp_new();
    temp2 = tcg_temp_new();
    reg1 = h8300_reg_ld(SZ_B, a->rd, temp1, false);
    reg2 = h8300_reg_ld(SZ_B, a->rs, temp2, false);
    h8300_addx(reg1, reg1, reg2);
    h8300_reg_st(SZ_B, a->rd, reg1);
    tcg_temp_free(temp1);
    tcg_temp_free(temp2);
    return true;
}

static bool trans_SUB_i(DisasContext *ctx, arg_SUB_i *a)
{
    TCGv temp, reg, imm;
    imm = tcg_const_i32(a->imm);
    temp = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->rd, temp, true);
    h8300_sub(a->sz, reg, reg, imm, true);
    h8300_reg_st(a->sz, a->rd, temp);
    tcg_temp_free(temp);
    tcg_temp_free(imm);
    return true;
}

static bool trans_SUB_r(DisasContext *ctx, arg_SUB_r *a)
{
    TCGv temp1, temp2, reg1, reg2;
    temp1 = tcg_temp_new();
    temp2 = tcg_temp_new();
    reg1 = h8300_reg_ld(a->sz, a->rd, temp1, true);
    reg2 = h8300_reg_ld(a->sz, a->rs, temp2, true);
    h8300_sub(a->sz, reg1, reg1, reg2, true);
    h8300_reg_st(a->sz, a->rd, reg1);
    tcg_temp_free(temp1);
    tcg_temp_free(temp2);
    return true;
}

static bool trans_SUBS(DisasContext *ctx, arg_SUBS *a)
{
    tcg_gen_subi_i32(cpu_regs[a->rd], cpu_regs[a->rd], a->imm);
    return true;
}

static bool trans_SUBX_i(DisasContext *ctx, arg_SUBX_i *a)
{
    TCGv temp, reg, imm;
    temp = tcg_temp_new();
    imm = tcg_temp_new();
    reg = h8300_reg_ld(SZ_B, a->rd, temp, true);
    tcg_gen_addi_i32(imm, cpu_ccr_c, a->imm);
    h8300_sub(SZ_B, reg, reg, imm, true);
    h8300_reg_st(SZ_B, a->rd, temp);
    tcg_temp_free(temp);
    tcg_temp_free(imm);
    return true;
}

static bool trans_SUBX_r(DisasContext *ctx, arg_SUBX_r *a)
{
    TCGv temp1, temp2, reg1, reg2;
    temp1 = tcg_temp_new();
    temp2 = tcg_temp_new();
    reg1 = h8300_reg_ld(SZ_B, a->rd, temp1, true);
    reg2 = h8300_reg_ld(SZ_B, a->rs, temp2, true);
    tcg_gen_add_i32(reg2, reg2, cpu_ccr_c);
    h8300_sub(SZ_B, reg1, reg1, reg2, true);
    h8300_reg_st(SZ_B, a->rd, reg1);
    tcg_temp_free(temp1);
    tcg_temp_free(temp2);
    return true;
}

static bool trans_CMP_i(DisasContext *ctx, arg_CMP_i *a)
{
    TCGv temp, reg, imm;
    imm = tcg_const_i32(a->imm);
    temp = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->rd, temp, true);
    h8300_sub(a->sz, NULL, reg, imm, true);
    tcg_temp_free(temp);
    tcg_temp_free(imm);
    return true;
}

static bool trans_CMP_r(DisasContext *ctx, arg_CMP_r *a)
{
    TCGv temp1, temp2, reg1, reg2;
    temp1 = tcg_temp_new();
    temp2 = tcg_temp_new();
    reg1 = h8300_reg_ld(a->sz, a->rd, temp1, true);
    reg2 = h8300_reg_ld(a->sz, a->rs, temp2, true);
    h8300_sub(a->sz, NULL, reg1, reg2, true);
    tcg_temp_free(temp1);
    tcg_temp_free(temp2);
    return true;
}

static bool trans_INC(DisasContext *ctx, arg_INC *a)
{
    TCGv temp, imm, reg;
    imm = tcg_const_i32(a->imm);
    temp = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->rd, temp, false);
    h8300_add(a->sz, reg, reg, imm, false);
    h8300_reg_st(a->sz, a->rd, temp);
    tcg_temp_free(temp);
    tcg_temp_free(imm);
    return true;
}    

static bool trans_DEC(DisasContext *ctx, arg_DEC *a)
{
    TCGv temp, imm, reg;
    imm = tcg_const_i32(a->imm);
    temp = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->rd, temp, true);
    h8300_sub(a->sz, reg, reg, imm, false);
    h8300_reg_st(a->sz, a->rd, temp);
    tcg_temp_free(temp);
    tcg_temp_free(imm);
    return true;
}    

static bool trans_DAA(DisasContext *ctx, arg_DAA *a)
{
    TCGv temp;
    temp = tcg_temp_new();
    h8300_reg_ld(SZ_B, a->r, temp, true);
    gen_helper_daa(temp, cpu_env, temp);
    h8300_reg_st(SZ_B, a->r, temp);
    return true;
}

static bool trans_DAS(DisasContext *ctx, arg_DAS *a)
{
    TCGv temp;
    temp = tcg_temp_new();
    h8300_reg_ld(SZ_B, a->r, temp, true);
    gen_helper_das(temp, cpu_env, temp);
    h8300_reg_st(SZ_B, a->r, temp);
    return true;
}

static bool trans_MULXU(DisasContext *ctx, arg_MULXU *a)
{
    TCGv temp1, temp2, reg1, reg2;
    temp1 = tcg_temp_new();
    temp2 = tcg_temp_new();
    reg1 = h8300_reg_ld(a->sz + 1, a->rd, temp1, false);
    switch(a->sz) {
    case SZ_B:
        tcg_gen_ext8u_i32(reg1, reg1);
        break;
    case SZ_W:
        tcg_gen_ext16u_i32(reg1, reg1);
        break;
    default:
        g_assert_not_reached();
    }
    reg2 = h8300_reg_ld(a->sz, a->rs, temp2, false);
    switch(a->sz) {
    case SZ_B:
        tcg_gen_ext8u_i32(reg1, reg1);
        tcg_gen_ext8u_i32(reg2, reg2);
        break;
    case SZ_W:
        tcg_gen_ext16u_i32(reg1, reg1);
        tcg_gen_ext16u_i32(reg2, reg2);
        break;
    default:
        g_assert_not_reached();
    }
    tcg_gen_mul_i32(reg1, reg1, reg2);
    h8300_reg_st(a->sz + 1, a->rd, reg1);
    tcg_temp_free(temp1);
    tcg_temp_free(temp2);
    return true;
}

static bool trans_MULXS(DisasContext *ctx, arg_MULXU *a)
{
    TCGv temp1, temp2, reg1, reg2;
    temp1 = tcg_temp_new();
    temp2 = tcg_temp_new();
    reg1 = h8300_reg_ld(a->sz + 1, a->rd, temp1, true);
    switch(a->sz) {
    case SZ_B:
        tcg_gen_ext8s_i32(reg1, reg1);
        break;
    case SZ_W:
        tcg_gen_ext16s_i32(reg1, reg1);
        break;
    default:
        g_assert_not_reached();
    }
    reg2 = h8300_reg_ld(a->sz, a->rs, temp2, true);
    tcg_gen_mul_i32(reg1, reg1, reg2);
    tcg_gen_mov_i32(cpu_ccr_n, reg1);
    tcg_gen_mov_i32(cpu_ccr_z, reg1);
    h8300_reg_st(a->sz + 1, a->rd, reg1);
    tcg_temp_free(temp1);
    tcg_temp_free(temp2);
    return true;
}

static bool trans_DIVXU(DisasContext *ctx, arg_DIVXU *a)
{
    TCGv temp1, temp2, reg1, reg2;
    temp1 = tcg_temp_new();
    temp2 = tcg_temp_new();
    reg1 = h8300_reg_ld(a->sz + 1, a->rd, temp1, false);
    reg2 = h8300_reg_ld(a->sz, a->rs, temp2, false);
    tcg_gen_mov_i32(cpu_ccr_z, reg2);
    tcg_gen_mov_i32(cpu_ccr_n, reg2);
    switch(a->sz) {
    case SZ_B:
        tcg_gen_ext16u_i32(reg1, reg1);
        tcg_gen_ext8u_i32(reg2, reg2);
        break;
    case SZ_W:
        tcg_gen_ext16u_i32(reg2, reg2);
        break;
    default:
        g_assert_not_reached();
    }
    gen_helper_divu(reg1, cpu_env, reg1, reg2);
    if (a->sz == SZ_B) {
        tcg_gen_extract_i32(reg2, reg1, 16, 8);
        tcg_gen_deposit_i32(reg1, reg1, reg2, 8, 8);
    }
    h8300_reg_st(a->sz + 1, a->rd, reg1);
    tcg_temp_free(temp1);
    tcg_temp_free(temp2);
    return true;
}

static bool trans_DIVXS(DisasContext *ctx, arg_DIVXU *a)
{
    TCGv temp1, temp2, reg1, reg2;
    temp1 = tcg_temp_new();
    temp2 = tcg_temp_new();
    reg1 = h8300_reg_ld(a->sz + 1, a->rd, temp1, true);
    reg2 = h8300_reg_ld(a->sz, a->rs, temp2, true);
    tcg_gen_mov_i32(cpu_ccr_z, reg2);
    tcg_gen_mov_i32(cpu_ccr_n, reg2);
    gen_helper_div(reg1, cpu_env, reg1, reg2);
    if (a->sz == SZ_B) {
        tcg_gen_extract_i32(reg2, reg1, 16, 8);
        tcg_gen_deposit_i32(reg1, reg1, reg2, 8, 8);
    }
    h8300_reg_st(a->sz + 1, a->rd, reg1);
    tcg_temp_free(temp1);
    tcg_temp_free(temp2);
    return true;
}

static bool trans_NEG(DisasContext *ctx, arg_NEG *a)
{
    TCGv temp, reg;
    uint32_t s;
    temp = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->r, temp, true);
    s = 8 * (1 << a->sz) - 1;
    s = 1 << s;
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_ccr_v, reg, s);
    tcg_gen_neg_i32(reg, reg);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, reg, 0);
    tcg_gen_mov_i32(cpu_ccr_z, reg);
    tcg_gen_mov_i32(cpu_ccr_n, reg);
    h8300_ccr_adjust(a->sz);
    h8300_reg_st(a->sz, a->r, reg);
    tcg_temp_free(temp);
    return true;
}

static bool trans_EXTU(DisasContext *ctx, arg_EXTU *a)
{
    TCGv temp, reg;
    temp = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->r, temp, false);
    switch(a->sz) {
    case SZ_W:
        tcg_gen_ext8u_i32(reg, reg);
        break;
    case SZ_L:
        tcg_gen_ext16u_i32(reg, reg);
        break;
    default:
        g_assert_not_reached();
    }
    h8300_reg_st(a->sz, a->r, reg);
    tcg_gen_mov_i32(cpu_ccr_z, reg);
    tcg_gen_mov_i32(cpu_ccr_n, reg);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(temp);
    return true;
}

static bool trans_EXTS(DisasContext *ctx, arg_EXTU *a)
{
    TCGv temp, reg;
    temp = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->r, temp, true);
    switch(a->sz) {
    case SZ_W:
        tcg_gen_ext8s_i32(reg, reg);
        break;
    case SZ_L:
        tcg_gen_ext16s_i32(reg, reg);
        break;
    default:
        g_assert_not_reached();
    }
    h8300_reg_st(a->sz, a->r, reg);
    tcg_gen_mov_i32(cpu_ccr_z, reg);
    tcg_gen_mov_i32(cpu_ccr_n, reg);
    h8300_ccr_adjust(a->sz);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(temp);
    return true;
}

static bool trans_AND_i(DisasContext *ctx, arg_AND_i *a)
{
    TCGv temp, reg;
    temp = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->rd, temp, false);
    tcg_gen_andi_i32(reg, reg, a->imm);
    h8300_reg_st(a->sz, a->rd, reg);
    tcg_gen_mov_i32(cpu_ccr_z, reg);
    tcg_gen_mov_i32(cpu_ccr_n, reg);
    h8300_ccr_adjust(a->sz);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(temp);
    return true;
}

static bool trans_AND_r(DisasContext *ctx, arg_AND_r *a)
{
    TCGv temp1, temp2, reg1, reg2;
    temp1 = tcg_temp_new();
    temp2 = tcg_temp_new();
    reg1 = h8300_reg_ld(a->sz, a->rd, temp1, false);
    reg2 = h8300_reg_ld(a->sz, a->rs, temp2, false);
    tcg_gen_and_i32(reg1, reg1, reg2);
    h8300_reg_st(a->sz, a->rd, reg1);
    tcg_gen_mov_i32(cpu_ccr_z, reg1);
    tcg_gen_mov_i32(cpu_ccr_n, reg1);
    h8300_ccr_adjust(a->sz);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(temp1);
    tcg_temp_free(temp2);
    return true;
}

static bool trans_OR_i(DisasContext *ctx, arg_OR_i *a)
{
    TCGv temp, reg;
    temp = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->rd, temp, false);
    tcg_gen_ori_i32(reg, reg, a->imm);
    h8300_reg_st(a->sz, a->rd, reg);
    tcg_gen_mov_i32(cpu_ccr_z, reg);
    tcg_gen_mov_i32(cpu_ccr_n, reg);
    h8300_ccr_adjust(a->sz);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(temp);
    return true;
}

static bool trans_OR_r(DisasContext *ctx, arg_OR_r *a)
{
    TCGv temp1, temp2, reg1, reg2;
    temp1 = tcg_temp_new();
    temp2 = tcg_temp_new();
    reg1 = h8300_reg_ld(a->sz, a->rd, temp1, false);
    reg2 = h8300_reg_ld(a->sz, a->rs, temp2, false);
    tcg_gen_or_i32(reg1, reg1, reg2);
    h8300_reg_st(a->sz, a->rd, reg1);
    tcg_gen_mov_i32(cpu_ccr_z, reg1);
    tcg_gen_mov_i32(cpu_ccr_n, reg1);
    h8300_ccr_adjust(a->sz);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(temp1);
    tcg_temp_free(temp2);
    return true;
}

static bool trans_XOR_i(DisasContext *ctx, arg_XOR_i *a)
{
    TCGv temp, reg;
    temp = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->rd, temp, false);
    tcg_gen_xori_i32(reg, reg, a->imm);
    h8300_reg_st(a->sz, a->rd, reg);
    tcg_gen_mov_i32(cpu_ccr_z, reg);
    tcg_gen_mov_i32(cpu_ccr_n, reg);
    h8300_ccr_adjust(a->sz);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(temp);
    return true;
}

static bool trans_XOR_r(DisasContext *ctx, arg_XOR_r *a)
{
    TCGv temp1, temp2, reg1, reg2;
    temp1 = tcg_temp_new();
    temp2 = tcg_temp_new();
    reg1 = h8300_reg_ld(a->sz, a->rd, temp1, false);
    reg2 = h8300_reg_ld(a->sz, a->rs, temp2, false);
    tcg_gen_xor_i32(reg1, reg1, reg2);
    h8300_reg_st(a->sz, a->rd, reg1);
    tcg_gen_mov_i32(cpu_ccr_z, reg1);
    tcg_gen_mov_i32(cpu_ccr_n, reg1);
    h8300_ccr_adjust(a->sz);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(temp1);
    tcg_temp_free(temp2);
    return true;
}

static bool trans_NOT(DisasContext *ctx, arg_NOT *a)
{
    TCGv temp, reg;
    temp = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->r, temp, false);
    tcg_gen_not_i32(reg, reg);
    h8300_reg_st(a->sz, a->r, reg);
    tcg_gen_mov_i32(cpu_ccr_z, reg);
    tcg_gen_mov_i32(cpu_ccr_n, reg);
    h8300_ccr_adjust(a->sz);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(temp);
    return true;
}

static bool trans_SHAL(DisasContext *ctx, arg_SHAL *a)
{
    TCGv temp, reg;
    int s;
    temp = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->r, temp, true);
    s = 8 * (1 << a->sz) - 1 ;
    tcg_gen_extract_i32(cpu_ccr_c, reg, s - a->s, 1);
    tcg_gen_shli_i32(reg, reg, a->s + 1);
    tcg_gen_xor_i32(cpu_ccr_v, cpu_ccr_v, reg);
    tcg_gen_extract_i32(cpu_ccr_v, cpu_ccr_v, s - a->s, 1);
    h8300_reg_st(a->sz, a->r, reg);
    tcg_gen_mov_i32(cpu_ccr_z, reg);
    tcg_gen_mov_i32(cpu_ccr_n, reg);
    h8300_ccr_adjust(a->sz);
    tcg_temp_free(temp);
    return true;
}

static bool trans_SHAR(DisasContext *ctx, arg_SHAR *a)
{
    TCGv temp, reg;
    int s;
    temp = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->r, temp, true);
    s = 8 * (1 << a->sz) - 1 ;
    tcg_gen_extract_i32(cpu_ccr_c, reg, a->s, 1);
    tcg_gen_extract_i32(cpu_ccr_v, reg, s, 1);
    tcg_gen_deposit_i32(cpu_ccr_v, cpu_ccr_v, cpu_ccr_v, s, 1);
    if (a->s == 1) {
        tcg_gen_deposit_i32(cpu_ccr_v, cpu_ccr_v, cpu_ccr_v, s - 1, 1);
    }
    tcg_gen_andi_i32(cpu_ccr_v, cpu_ccr_v, 0xfffffffe);
    tcg_gen_shri_i32(reg, reg, a->s + 1);
    tcg_gen_or_i32(reg, reg, cpu_ccr_v);
    h8300_reg_st(a->sz, a->r, reg);
    tcg_gen_mov_i32(cpu_ccr_z, reg);
    tcg_gen_mov_i32(cpu_ccr_n, reg);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    h8300_ccr_adjust(a->sz);
    tcg_temp_free(temp);
    return true;
}

static bool trans_SHLL(DisasContext *ctx, arg_SHLL *a)
{
    TCGv temp, reg;
    int s;
    temp = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->r, temp, false);
    s = 8 * (1 << a->sz) - 1 ;
    tcg_gen_extract_i32(cpu_ccr_c, reg, s - a->s, 1);
    tcg_gen_shli_i32(reg, reg, a->s + 1);
    h8300_reg_st(a->sz, a->r, reg);
    tcg_gen_mov_i32(cpu_ccr_z, reg);
    tcg_gen_mov_i32(cpu_ccr_n, reg);
    h8300_ccr_adjust(a->sz);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(temp);
    return true;
}

static bool trans_SHLR(DisasContext *ctx, arg_SHLR *a)
{
    TCGv temp, reg;
    temp = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->r, temp, false);
    tcg_gen_extract_i32(cpu_ccr_c, reg, a->s, 1);
    switch(a->sz) {
    case SZ_B:
        tcg_gen_ext8u_i32(reg, reg);
        break;
    case SZ_W:
        tcg_gen_ext16u_i32(reg, reg);
        break;
    }
    tcg_gen_shri_i32(reg, reg, a->s + 1);
    h8300_reg_st(a->sz, a->r, reg);
    tcg_gen_mov_i32(cpu_ccr_z, reg);
    tcg_gen_mov_i32(cpu_ccr_n, reg);
    h8300_ccr_adjust(a->sz);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(temp);
    return true;
}

static bool trans_ROTL(DisasContext *ctx, arg_ROTL *a)
{
    TCGv temp, reg;
    int s;
    temp = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->r, temp, false);
    s = 8 * (1 << a->sz) - 1 ;
    tcg_gen_extract_i32(cpu_ccr_c, reg, s - a->s, a->s + 1);
    tcg_gen_shli_i32(reg, reg, a->s + 1);
    tcg_gen_or_i32(reg, reg, cpu_ccr_c);
    h8300_reg_st(a->sz, a->r, reg);
    tcg_gen_mov_i32(cpu_ccr_z, reg);
    tcg_gen_mov_i32(cpu_ccr_n, reg);
    h8300_ccr_adjust(a->sz);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(temp);
    return true;
}

static bool trans_ROTR(DisasContext *ctx, arg_ROTR *a)
{
    TCGv temp, reg, c;
    temp = tcg_temp_new();
    c = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->r, temp, false);
    tcg_gen_extract_i32(cpu_ccr_c, reg, 0, a->s + 1);
    tcg_gen_shli_i32(c, cpu_ccr_c, 8 * (1 << a->sz) - 1 - a->s);
    tcg_gen_shri_i32(reg, reg, a->s + 1);
    tcg_gen_or_i32(reg, reg, c);
    tcg_gen_shri_i32(cpu_ccr_c, cpu_ccr_c, a->s);
    h8300_reg_st(a->sz, a->r, reg);
    tcg_gen_mov_i32(cpu_ccr_z, reg);
    tcg_gen_mov_i32(cpu_ccr_n, reg);
    h8300_ccr_adjust(a->sz);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(temp);
    tcg_temp_free(c);
    return true;
}

static bool trans_ROTXL(DisasContext *ctx, arg_ROTXL *a)
{
    TCGv temp, reg, c;
    int s;
    temp = tcg_temp_new();
    c = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->r, temp, false);
    s = 8 * (1 << a->sz) - 1 ;
    tcg_gen_extract_i32(c, reg, s, 1);
    tcg_gen_shli_i32(reg, reg, 1);
    tcg_gen_or_i32(reg, reg, cpu_ccr_c);
    tcg_gen_mov_i32(cpu_ccr_c, c);
    h8300_reg_st(a->sz, a->r, reg);
    tcg_gen_mov_i32(cpu_ccr_z, reg);
    tcg_gen_mov_i32(cpu_ccr_n, reg);
    h8300_ccr_adjust(a->sz);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(temp);
    tcg_temp_free(c);
    return true;
}

static bool trans_ROTXR(DisasContext *ctx, arg_ROTXR *a)
{
    TCGv temp, reg, c;
    temp = tcg_temp_new();
    c = tcg_temp_new();
    reg = h8300_reg_ld(a->sz, a->r, temp, false);
    switch(a->sz) {
    case SZ_B:
        tcg_gen_ext8u_i32(reg, reg);
        break;
    case SZ_W:
        tcg_gen_ext16u_i32(reg, reg);
        break;
    }
    tcg_gen_extract_i32(c, reg, 0, 1);
    tcg_gen_shri_i32(reg, reg, 1);
    tcg_gen_deposit_z_i32(cpu_ccr_c, cpu_ccr_c, 8 * (1 << a->sz) - 1, 1);
    tcg_gen_or_i32(reg, reg, cpu_ccr_c);
    h8300_reg_st(a->sz, a->r, reg);
    tcg_gen_mov_i32(cpu_ccr_c, c);
    tcg_gen_mov_i32(cpu_ccr_z, reg);
    tcg_gen_mov_i32(cpu_ccr_n, reg);
    h8300_ccr_adjust(a->sz);
    tcg_gen_movi_i32(cpu_ccr_v, 0);
    tcg_temp_free(temp);
    tcg_temp_free(c);
    return true;
}

static bool trans_BAND_r(DisasContext *ctx, arg_BAND_r *a)
{
    TCGv temp, mask;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    h8300_gen_reg_ldb(a->r, temp, false);
    tcg_gen_shli_i32(mask, cpu_ccr_c, a->imm);
    tcg_gen_and_i32(mask, temp, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, mask, 0);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    return true;
}
    
static bool trans_BAND_m(DisasContext *ctx, arg_BAND_m *a)
{
    TCGv temp, mask;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_gen_shli_i32(mask, cpu_ccr_c, a->imm);
    tcg_gen_and_i32(mask, temp, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, mask, 0);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    return true;
}
    
static bool trans_BAND_a(DisasContext *ctx, arg_BAND_a *a)
{
    TCGv temp, mask, mem;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_gen_shli_i32(mask, cpu_ccr_c, a->imm);
    tcg_gen_and_i32(mask, temp, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, mask, 0);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(mem);
    return true;
}

static bool trans_BIAND_r(DisasContext *ctx, arg_BIAND_r *a)
{
    TCGv temp, mask;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    h8300_gen_reg_ldb(a->r, temp, false);
    tcg_gen_not_i32(temp, temp);
    tcg_gen_shli_i32(mask, cpu_ccr_c, a->imm);
    tcg_gen_and_i32(mask, temp, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, mask, 0);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    return true;
}
    
static bool trans_BIAND_m(DisasContext *ctx, arg_BIAND_m *a)
{
    TCGv temp, mask;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_gen_not_i32(temp, temp);
    tcg_gen_shli_i32(mask, cpu_ccr_c, a->imm);
    tcg_gen_and_i32(mask, temp, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, mask, 0);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    return true;
}
    
static bool trans_BIAND_a(DisasContext *ctx, arg_BIAND_a *a)
{
    TCGv temp, mask, mem;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_gen_not_i32(temp, temp);
    tcg_gen_shli_i32(mask, cpu_ccr_c, a->imm);
    tcg_gen_and_i32(mask, temp, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, mask, 0);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(mem);
    return true;
}

static bool trans_BILD_r(DisasContext *ctx, arg_BILD_r *a)
{
    TCGv temp;
    temp = tcg_temp_new();
    h8300_gen_reg_ldb(a->r, temp, false);
    tcg_gen_not_i32(temp, temp);
    tcg_gen_extract_i32(cpu_ccr_c, temp, a->imm, 1);
    tcg_temp_free(temp);
    return true;
}
    
static bool trans_BILD_m(DisasContext *ctx, arg_BILD_m *a)
{
    TCGv temp;
    temp = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_gen_not_i32(temp, temp);
    tcg_gen_extract_i32(cpu_ccr_c, temp, a->imm, 1);
    tcg_temp_free(temp);
    return true;
}
    
static bool trans_BILD_a(DisasContext *ctx, arg_BILD_a *a)
{
    TCGv temp, mem;
    temp = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_gen_not_i32(temp, temp);
    tcg_gen_extract_i32(cpu_ccr_c, temp, a->imm, 1);
    tcg_temp_free(temp);
    tcg_temp_free(mem);
    return true;
}

static bool trans_BIOR_r(DisasContext *ctx, arg_BIOR_r *a)
{
    TCGv temp, mask;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    h8300_gen_reg_ldb(a->r, temp, false);
    tcg_gen_not_i32(temp, temp);
    tcg_gen_andi_i32(temp, temp, 1 << a->imm);
    tcg_gen_shli_i32(mask, cpu_ccr_c, a->imm);
    tcg_gen_or_i32(mask, temp, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, mask, 0);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    return true;
}
    
static bool trans_BIOR_m(DisasContext *ctx, arg_BIOR_m *a)
{
    TCGv temp, mask;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_gen_not_i32(temp, temp);
    tcg_gen_andi_i32(temp, temp, 1 << a->imm);
    tcg_gen_shli_i32(mask, cpu_ccr_c, a->imm);
    tcg_gen_or_i32(mask, temp, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, mask, 0);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    return true;
}
    
static bool trans_BIOR_a(DisasContext *ctx, arg_BIOR_a *a)
{
    TCGv temp, mask, mem;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_gen_not_i32(temp, temp);
    tcg_gen_andi_i32(temp, temp, 1 << a->imm);
    tcg_gen_shli_i32(mask, cpu_ccr_c, a->imm);
    tcg_gen_or_i32(mask, temp, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, mask, 0);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(mem);
    return true;
}

static bool trans_BIST_r(DisasContext *ctx, arg_BIST_r *a)
{
    TCGv temp, mask;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    h8300_gen_reg_ldb(a->r, temp, false);
    tcg_gen_andi_i32(temp, temp, ~(1 << a->imm));
    tcg_gen_not_i32(mask, cpu_ccr_c);
    tcg_gen_deposit_z_i32(mask, mask, a->imm, 1);
    tcg_gen_or_i32(temp, temp, mask);
    h8300_gen_reg_stb(a->r, temp);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    return true;
}
    
static bool trans_BIST_m(DisasContext *ctx, arg_BIST_m *a)
{
    TCGv temp, mask;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_gen_andi_i32(temp, temp, ~(1 << a->imm));
    tcg_gen_not_i32(mask, cpu_ccr_c);
    tcg_gen_deposit_z_i32(mask, mask, a->imm, 1);
    tcg_gen_or_i32(temp, temp, mask);
    tcg_gen_qemu_st_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    return true;
}
    
static bool trans_BIST_a(DisasContext *ctx, arg_BIST_a *a)
{
    TCGv temp, mask, mem;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_gen_andi_i32(temp, temp, ~(1 << a->imm));
    tcg_gen_not_i32(mask, cpu_ccr_c);
    tcg_gen_deposit_z_i32(mask, mask, a->imm, 1);
    tcg_gen_or_i32(temp, temp, mask);
    tcg_gen_qemu_st_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(mem);
    return true;
}

static bool trans_BIXOR_r(DisasContext *ctx, arg_BIXOR_r *a)
{
    TCGv temp, mask;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    h8300_gen_reg_ldb(a->r, temp, false);
    tcg_gen_not_i32(temp, temp);
    tcg_gen_andi_i32(temp, temp, 1 << a->imm);
    tcg_gen_shli_i32(mask, cpu_ccr_c, a->imm);
    tcg_gen_xor_i32(mask, temp, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, mask, 0);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    return true;
}
    
static bool trans_BIXOR_m(DisasContext *ctx, arg_BIXOR_m *a)
{
    TCGv temp, mask;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_gen_not_i32(temp, temp);
    tcg_gen_andi_i32(temp, temp, 1 << a->imm);
    tcg_gen_shli_i32(mask, cpu_ccr_c, a->imm);
    tcg_gen_xor_i32(mask, temp, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, mask, 0);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    return true;
}
    
static bool trans_BIXOR_a(DisasContext *ctx, arg_BIXOR_a *a)
{
    TCGv temp, mask, mem;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_gen_not_i32(temp, temp);
    tcg_gen_andi_i32(temp, temp, 1 << a->imm);
    tcg_gen_shli_i32(mask, cpu_ccr_c, a->imm);
    tcg_gen_xor_i32(mask, temp, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, mask, 0);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(mem);
    return true;
}

static bool trans_BLD_r(DisasContext *ctx, arg_BLD_r *a)
{
    TCGv temp;
    temp = tcg_temp_new();
    h8300_gen_reg_ldb(a->r, temp, false);
    tcg_gen_extract_i32(cpu_ccr_c, temp, a->imm, 1);
    tcg_temp_free(temp);
    return true;
}
    
static bool trans_BLD_m(DisasContext *ctx, arg_BLD_m *a)
{
    TCGv temp;
    temp = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_gen_extract_i32(cpu_ccr_c, temp, a->imm, 1);
    tcg_temp_free(temp);
    return true;
}
    
static bool trans_BLD_a(DisasContext *ctx, arg_BLD_a *a)
{
    TCGv temp, mem;
    temp = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_gen_extract_i32(cpu_ccr_c, temp, a->imm, 1);
    tcg_temp_free(temp);
    tcg_temp_free(mem);
    return true;
}

static bool trans_BOR_r(DisasContext *ctx, arg_BOR_r *a)
{
    TCGv temp, mask;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    h8300_gen_reg_ldb(a->r, temp, false);
    tcg_gen_andi_i32(temp, temp, 1 << a->imm);
    tcg_gen_shli_i32(mask, cpu_ccr_c, a->imm);
    tcg_gen_or_i32(mask, temp, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, mask, 0);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    return true;
}
    
static bool trans_BOR_m(DisasContext *ctx, arg_BOR_m *a)
{
    TCGv temp, mask;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_gen_andi_i32(temp, temp, 1 << a->imm);
    tcg_gen_shli_i32(mask, cpu_ccr_c, a->imm);
    tcg_gen_or_i32(mask, temp, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, mask, 0);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    return true;
}
    
static bool trans_BOR_a(DisasContext *ctx, arg_BOR_a *a)
{
    TCGv temp, mask, mem;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_gen_andi_i32(temp, temp, 1 << a->imm);
    tcg_gen_shli_i32(mask, cpu_ccr_c, a->imm);
    tcg_gen_or_i32(mask, temp, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, mask, 0);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(mem);
    return true;
}

static bool trans_BST_r(DisasContext *ctx, arg_BST_r *a)
{
    TCGv temp, mask;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    h8300_gen_reg_ldb(a->r, temp, false);
    tcg_gen_deposit_i32(temp, temp, cpu_ccr_c, a->imm, 1);
    h8300_gen_reg_stb(a->r, temp);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    return true;
}
    
static bool trans_BST_m(DisasContext *ctx, arg_BST_m *a)
{
    TCGv temp, mask;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_gen_deposit_i32(temp, temp, cpu_ccr_c, a->imm, 1);
    tcg_gen_qemu_st_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    return true;
}
    
static bool trans_BST_a(DisasContext *ctx, arg_BST_a *a)
{
    TCGv temp, mask, mem;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_gen_deposit_i32(temp, temp, cpu_ccr_c, a->imm, 1);
    tcg_gen_qemu_st_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(mem);
    return true;
}

static bool trans_BXOR_r(DisasContext *ctx, arg_BXOR_r *a)
{
    TCGv temp, mask;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    h8300_gen_reg_ldb(a->r, temp, false);
    tcg_gen_andi_i32(temp, temp, 1 << a->imm);
    tcg_gen_deposit_z_i32(mask, cpu_ccr_c, a->imm, 1);
    tcg_gen_xor_i32(mask, temp, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, mask, 0);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    return true;
}
    
static bool trans_BXOR_m(DisasContext *ctx, arg_BXOR_m *a)
{
    TCGv temp, mask;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_gen_andi_i32(temp, temp, 1 << a->imm);
    tcg_gen_deposit_z_i32(mask, cpu_ccr_c, a->imm, 1);
    tcg_gen_xor_i32(mask, temp, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, mask, 0);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    return true;
}
    
static bool trans_BXOR_a(DisasContext *ctx, arg_BXOR_a *a)
{
    TCGv temp, mask, mem;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 |  MO_TE);
    tcg_gen_andi_i32(temp, temp, 1 << a->imm);
    tcg_gen_deposit_z_i32(mask, cpu_ccr_c, a->imm, 1);
    tcg_gen_xor_i32(mask, temp, mask);
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_ccr_c, mask, 0);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(mem);
    return true;
}

static bool trans_BOP1(DisasContext *ctx, arg_BOP1 *a)
{
    arg_BCLR_ia	bclr_i;
    arg_BCLR_ra	bclr_r;
    arg_BNOT_ia	bnot_i;
    arg_BNOT_ra	bnot_r;
    arg_BTST_ia	btst_i;
    arg_BTST_ra	btst_r;
    arg_BSET_ia	bset_i;
    arg_BSET_ra	bset_r;
    arg_BAND_a  band_a;
    arg_BIAND_a biand_a;
    arg_BLD_a   bld_a;
    arg_BILD_a  bild_a;
    arg_BOR_a   bor_a;
    arg_BIOR_a  bior_a;
    arg_BST_a   bst_a;
    arg_BIST_a  bist_a;
    arg_BXOR_a  bxor_a;
    arg_BIXOR_a bixor_a;
    
    switch(a->op) {
    case 0x70:
        bset_i.imm = a->ir;
        bset_i.abs = a->abs;
        return trans_BSET_ia(ctx, &bset_i);
    case 0x60:
        bset_r.rn = a->ir;
        bset_r.abs = a->abs;
        return trans_BSET_ra(ctx, &bset_r);
    case 0x71:
        bnot_i.imm = a->ir;
        bnot_i.abs = a->abs;
        return trans_BNOT_ia(ctx, &bnot_i);
    case 0x61:
        bnot_r.rn = a->ir;
        bnot_r.abs = a->abs;
        return trans_BNOT_ra(ctx, &bnot_r);
    case 0x72:
        bclr_i.imm = a->ir;
        bclr_i.abs = a->abs;
        return trans_BCLR_ia(ctx, &bclr_i);
    case 0x62:
        bclr_r.rn = a->ir;
        bclr_r.abs = a->abs;
        return trans_BCLR_ra(ctx, &bclr_r);
    case 0x73:
        btst_i.imm = a->ir;
        btst_i.abs = a->abs;
        return trans_BTST_ia(ctx, &btst_i);
    case 0x63:
        btst_r.rn = a->ir;
        btst_r.abs = a->abs;
        return trans_BTST_ra(ctx, &btst_r);
    case 0x76:
        band_a.imm = a->ir;
        band_a.abs = a->abs;
        return trans_BAND_a(ctx, &band_a);
    case 0x176:
        biand_a.imm = a->ir;
        biand_a.abs = a->abs;
        return trans_BIAND_a(ctx, &biand_a);
    case 0x77:
        bld_a.imm = a->ir;
        bld_a.abs = a->abs;
        return trans_BLD_a(ctx, &bld_a);
    case 0x177:
        bild_a.imm = a->ir;
        bild_a.abs = a->abs;
        return trans_BILD_a(ctx, &bild_a);
    case 0x74:
        bor_a.imm = a->ir;
        bor_a.abs = a->abs;
        return trans_BOR_a(ctx, &bor_a);
    case 0x174:
        bior_a.imm = a->ir;
        bior_a.abs = a->abs;
        return trans_BIOR_a(ctx, &bior_a);
    case 0x67:
        bst_a.imm = a->ir;
        bst_a.abs = a->abs;
        return trans_BST_a(ctx, &bst_a);
    case 0x167:
        bist_a.imm = a->ir;
        bist_a.abs = a->abs;
        return trans_BIST_a(ctx, &bist_a);
    case 0x75:
        bxor_a.imm = a->ir;
        bxor_a.abs = a->abs;
        return trans_BXOR_a(ctx, &bxor_a);
    case 0x175:
        bixor_a.imm = a->ir;
        bixor_a.abs = a->abs;
        return trans_BIXOR_a(ctx, &bixor_a);
    default:
        return false;
    }
}

static bool trans_BCLR_ir(DisasContext *ctx, arg_BCLR_ir *a)
{
    TCGv temp;
    temp = tcg_temp_new();
    h8300_gen_reg_ldb(a->r, temp, false);
    tcg_gen_andi_i32(temp, temp, ~(1 << a->imm));
    h8300_gen_reg_stb(a->r, temp);
    tcg_temp_free(temp);
    return true;
}
    
static bool trans_BCLR_im(DisasContext *ctx, arg_BCLR_im *a)
{
    TCGv temp;
    temp = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_gen_andi_i32(temp, temp, ~(1 << a->imm));
    tcg_gen_qemu_st_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_temp_free(temp);
    return true;
}
    
static bool trans_BCLR_ia(DisasContext *ctx, arg_BCLR_ia *a)
{
    TCGv temp, mem;
    temp = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_gen_andi_i32(temp, temp, ~(1 << a->imm));
    tcg_gen_qemu_st_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_temp_free(temp);
    tcg_temp_free(mem);
    return true;
}

static bool trans_BCLR_rr(DisasContext *ctx, arg_BCLR_rr *a)
{
    TCGv temp, mask, shift;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    shift = tcg_temp_new();
    h8300_gen_reg_ldb(a->rd, temp, false);
    h8300_gen_reg_ldb(a->rs, shift, false);
    tcg_gen_movi_i32(mask, 1);
    tcg_gen_shl_i32(mask, mask, shift);
    tcg_gen_not_i32(mask, mask);
    tcg_gen_and_i32(temp, temp, mask);
    h8300_gen_reg_stb(a->rd, temp);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(shift);
    return true;
}
    
static bool trans_BCLR_rm(DisasContext *ctx, arg_BCLR_rm *a)
{
    TCGv temp, mask, shift;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    shift = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->rd], 0, MO_8 | MO_TE);
    h8300_gen_reg_ldb(a->rn, shift, false);
    tcg_gen_movi_i32(mask, 1);
    tcg_gen_shl_i32(mask, mask, shift);
    tcg_gen_not_i32(mask, mask);
    tcg_gen_and_i32(temp, temp, mask);
    tcg_gen_qemu_st_i32(temp, cpu_regs[a->rd], 0, MO_8 | MO_TE);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(shift);
    return true;
}
    
static bool trans_BCLR_ra(DisasContext *ctx, arg_BCLR_ra *a)
{
    TCGv temp, mask, mem, shift;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    shift = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 | MO_TE);
    h8300_gen_reg_ldb(a->rn, shift, false);
    tcg_gen_movi_i32(mask, 1);
    tcg_gen_shl_i32(mask, mask, shift);
    tcg_gen_not_i32(mask, mask);
    tcg_gen_and_i32(temp, temp, mask);
    tcg_gen_qemu_st_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(mem);
    tcg_temp_free(shift);
    return true;
}

static bool trans_BNOT_ir(DisasContext *ctx, arg_BNOT_ir *a)
{
    TCGv temp;
    temp = tcg_temp_new();
    h8300_gen_reg_ldb(a->r, temp, false);
    tcg_gen_xori_i32(temp, temp, 1 << a->imm);
    h8300_gen_reg_stb(a->r, temp);
    tcg_temp_free(temp);
    return true;
}
    
static bool trans_BNOT_im(DisasContext *ctx, arg_BNOT_im *a)
{
    TCGv temp;
    temp = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_gen_xori_i32(temp, temp, 1 << a->imm);
    tcg_gen_qemu_st_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_temp_free(temp);
    return true;
}
    
static bool trans_BNOT_ia(DisasContext *ctx, arg_BNOT_ia *a)
{
    TCGv temp, mem;
    temp = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_gen_xori_i32(temp, temp, 1 << a->imm);
    tcg_gen_qemu_st_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_temp_free(temp);
    tcg_temp_free(mem);
    return true;
}

static bool trans_BNOT_rr(DisasContext *ctx, arg_BNOT_rr *a)
{
    TCGv temp, mask, shift;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    shift = tcg_temp_new();
    h8300_gen_reg_ldb(a->rd, temp, false);
    h8300_gen_reg_ldb(a->rs, shift, false);
    tcg_gen_movi_i32(mask, 1);
    tcg_gen_shl_i32(mask, mask, shift);
    tcg_gen_xor_i32(temp, temp, mask);
    h8300_gen_reg_stb(a->rd, temp);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(shift);
    return true;
}

static bool trans_BNOT_rm(DisasContext *ctx, arg_BNOT_rm *a)
{
    TCGv temp, mask, shift;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    shift = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->rd], 0, MO_8 | MO_TE);
    h8300_gen_reg_ldb(a->rn, shift, false);
    tcg_gen_movi_i32(mask, 1);
    tcg_gen_shl_i32(mask, mask, shift);
    tcg_gen_xor_i32(temp, temp, mask);
    tcg_gen_qemu_st_i32(temp, cpu_regs[a->rd], 0, MO_8 | MO_TE);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    return true;
}
    
static bool trans_BNOT_ra(DisasContext *ctx, arg_BNOT_ra *a)
{
    TCGv temp, mask, mem, shift;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    shift = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 | MO_TE);
    h8300_gen_reg_ldb(a->rn, shift, false);
    tcg_gen_movi_i32(mask, 1);
    tcg_gen_shl_i32(mask, mask, shift);
    tcg_gen_xor_i32(temp, temp, mask);
    tcg_gen_qemu_st_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(mem);
    tcg_temp_free(shift);
    return true;
}

static bool trans_BSET_ir(DisasContext *ctx, arg_BSET_ir *a)
{
    TCGv temp;
    temp = tcg_temp_new();
    h8300_gen_reg_ldb(a->r, temp, false);
    tcg_gen_ori_i32(temp, temp, 1 << a->imm);
    h8300_gen_reg_stb(a->r, temp);
    tcg_temp_free(temp);
    return true;
}
    
static bool trans_BSET_im(DisasContext *ctx, arg_BSET_im *a)
{
    TCGv temp;
    temp = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_gen_ori_i32(temp, temp, 1 << a->imm);
    tcg_gen_qemu_st_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_temp_free(temp);
    return true;
}
    
static bool trans_BSET_ia(DisasContext *ctx, arg_BSET_ia *a)
{
    TCGv temp, mem;
    temp = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_gen_ori_i32(temp, temp, 1 << a->imm);
    tcg_gen_qemu_st_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_temp_free(temp);
    tcg_temp_free(mem);
    return true;
}

static bool trans_BSET_rr(DisasContext *ctx, arg_BSET_rr *a)
{
    TCGv temp, mask, shift;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    shift = tcg_temp_new();
    h8300_gen_reg_ldb(a->rd, temp, false);
    h8300_gen_reg_ldb(a->rs, shift, false);
    tcg_gen_movi_i32(mask, 1);
    tcg_gen_shl_i32(mask, mask, shift);
    tcg_gen_or_i32(temp, temp, mask);
    h8300_gen_reg_stb(a->rd, temp);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(shift);
    return true;
}

static bool trans_BSET_rm(DisasContext *ctx, arg_BSET_rm *a)
{
    TCGv temp, mask, shift;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    shift = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->rd], 0, MO_8 | MO_TE);
    h8300_gen_reg_ldb(a->rn, shift, false);
    tcg_gen_movi_i32(mask, 1);
    tcg_gen_shl_i32(mask, mask, shift);
    tcg_gen_or_i32(temp, temp, mask);
    tcg_gen_qemu_st_i32(temp, cpu_regs[a->rd], 0, MO_8 | MO_TE);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(shift);
    return true;
}
    
static bool trans_BSET_ra(DisasContext *ctx, arg_BSET_ra *a)
{
    TCGv temp, mask, mem, shift;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    shift = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 | MO_TE);
    h8300_gen_reg_ldb(a->rn, shift, false);
    tcg_gen_movi_i32(mask, 1);
    tcg_gen_shl_i32(mask, mask, shift);
    tcg_gen_or_i32(temp, temp, mask);
    tcg_gen_qemu_st_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(mem);
    tcg_temp_free(shift);
    return true;
}

static bool trans_BTST_ir(DisasContext *ctx, arg_BTST_ir *a)
{
    TCGv temp;
    temp = tcg_temp_new();
    h8300_gen_reg_ldb(a->r, temp, false);
    tcg_gen_andi_i32(cpu_ccr_z, temp, 1 << a->imm);
    tcg_temp_free(temp);
    return true;
}
    
static bool trans_BTST_im(DisasContext *ctx, arg_BTST_im *a)
{
    TCGv temp;
    temp = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->er], 0, MO_8 | MO_TE);
    tcg_gen_andi_i32(cpu_ccr_z, temp, 1 << a->imm);
    tcg_temp_free(temp);
    return true;
}
    
static bool trans_BTST_ia(DisasContext *ctx, arg_BTST_ia *a)
{
    TCGv temp, mem;
    temp = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 | MO_TE);
    tcg_gen_andi_i32(cpu_ccr_z, temp, 1 << a->imm);
    tcg_temp_free(temp);
    tcg_temp_free(mem);
    return true;
}

static bool trans_BTST_rr(DisasContext *ctx, arg_BTST_rr *a)
{
    TCGv temp, mask, shift;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    shift = tcg_temp_new();
    h8300_gen_reg_ldb(a->rd, temp, false);
    h8300_gen_reg_ldb(a->rs, shift, false);
    tcg_gen_movi_i32(mask, 1);
    tcg_gen_shl_i32(mask, mask, shift);
    tcg_gen_and_i32(cpu_ccr_z, temp, mask);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(shift);
    return true;
}

static bool trans_BTST_rm(DisasContext *ctx, arg_BTST_rm *a)
{
    TCGv temp, mask, shift;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    shift = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, cpu_regs[a->rd], 0, MO_8 | MO_TE);
    h8300_gen_reg_ldb(a->rn, shift, false);
    tcg_gen_movi_i32(mask, 1);
    tcg_gen_shl_i32(mask, mask, shift);
    tcg_gen_and_i32(cpu_ccr_z, temp, mask);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(shift);
    return true;
}
    
static bool trans_BTST_ra(DisasContext *ctx, arg_BTST_ra *a)
{
    TCGv temp, mask, mem, shift;
    temp = tcg_temp_new();
    mask = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    shift = tcg_temp_new();
    tcg_gen_qemu_ld_i32(temp, mem, 0, MO_8 | MO_TE);
    h8300_gen_reg_ldb(a->rn, shift, false);
    tcg_gen_movi_i32(mask, 1);
    tcg_gen_shl_i32(mask, mask, shift);
    tcg_gen_and_i32(cpu_ccr_z, temp, mask);
    tcg_temp_free(temp);
    tcg_temp_free(mask);
    tcg_temp_free(mem);
    tcg_temp_free(shift);
    return true;
}

static bool trans_BOP2(DisasContext *ctx, arg_BOP2 *a)
{
    arg_BCLR_ia bclr_ia;
    arg_BNOT_ia bnot_ia;
    arg_BSET_ia bset_ia;
    arg_BTST_ia btst_ia;
    arg_BCLR_ra bclr_ra;
    arg_BNOT_ra bnot_ra;
    arg_BSET_ra bset_ra;
    arg_BTST_ra btst_ra;

    switch(a->op) {
    case 0x72:
        bclr_ia.imm = a->ir;
        bclr_ia.abs = a->abs;
        return trans_BCLR_ia(ctx, &bclr_ia);
    case 0x71:
        bnot_ia.imm = a->ir;
        bnot_ia.abs = a->abs;
        return trans_BNOT_ia(ctx, &bnot_ia);
    case 0x70:
        bset_ia.imm = a->ir;
        bset_ia.abs = a->abs;
        return trans_BSET_ia(ctx, &bset_ia);
    case 0x73:
        btst_ia.imm = a->ir;
        btst_ia.abs = a->abs;
        return trans_BTST_ia(ctx, &btst_ia);
    case 0x62:
        bclr_ra.rn = a->ir;
        bclr_ra.abs = a->abs;
        return trans_BCLR_ra(ctx, &bclr_ra);
    case 0x61:
        bnot_ra.rn = a->ir;
        bnot_ra.abs = a->abs;
        return trans_BNOT_ra(ctx, &bnot_ra);
    case 0x60:
        bset_ra.rn = a->ir;
        bset_ra.abs = a->abs;
        return trans_BSET_ra(ctx, &bset_ra);
    case 0x63:
        btst_ra.rn = a->ir;
        btst_ra.abs = a->abs;
        return trans_BTST_ra(ctx, &btst_ra);
    default:
        return false;
    }
    return false;
}

static bool trans_Bcc(DisasContext *ctx, arg_Bcc *a)
{
    DisasCompare dc;
    TCGLabel *t, *done;

    switch (a->cd) {
    case 0:
        /* always true case */
        gen_goto_tb(ctx, 0, ctx->base.pc_next + a->dsp);
        break;
    case 1:
        /* always false case */
        /* Nothing do */
        break;
    case 2 ... 15:
        dc.temp = tcg_temp_new();
        ccr_cond(&dc, a->cd);
        t = gen_new_label();
        done = gen_new_label();
        tcg_gen_brcondi_i32(dc.cond, dc.value, 0, t);
        gen_goto_tb(ctx, 0, ctx->base.pc_next);
        tcg_gen_br(done);
        gen_set_label(t);
        gen_goto_tb(ctx, 1, ctx->base.pc_next + a->dsp);
        gen_set_label(done);
        tcg_temp_free(dc.temp);
        break;
    }
    return true;
}

static bool trans_JMP_r(DisasContext *ctx, arg_JMP_r *a)
{
    tcg_gen_andi_i32(cpu_pc, cpu_regs[a->rs], 0x00ffffff);
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

static bool trans_JMP_a24(DisasContext *ctx, arg_JMP_a24 *a)
{
    tcg_gen_movi_i32(cpu_pc, a->abs);
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

static bool trans_JMP_aa8(DisasContext *ctx, arg_JMP_aa8 *a)
{
    TCGv mem = tcg_const_i32(a->abs);
    tcg_gen_qemu_ld_i32(cpu_pc, mem, 0, MO_32 | MO_TE);
    ctx->base.is_jmp = DISAS_JUMP;
    tcg_temp_free(mem);
    return true;
}

static inline void save_pc(DisasContext *ctx)
{
    TCGv pc = tcg_const_i32(ctx->base.pc_next);
    tcg_gen_subi_i32(cpu_sp, cpu_sp, 4);
    tcg_gen_qemu_st_i32(pc, cpu_sp, 0, MO_32 | MO_TE);
    tcg_temp_free(pc);
}

static bool trans_JSR_r(DisasContext *ctx, arg_JMP_r *a)
{
    save_pc(ctx);
    tcg_gen_andi_i32(cpu_pc, cpu_regs[a->rs], 0x00ffffff);
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

static bool trans_JSR_a24(DisasContext *ctx, arg_JSR_a24 *a)
{
    save_pc(ctx);
    tcg_gen_movi_i32(cpu_pc, a->abs);
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

static bool trans_JSR_aa8(DisasContext *ctx, arg_JSR_aa8 *a)
{
    TCGv mem = tcg_const_i32(a->abs);
    save_pc(ctx);
    tcg_gen_qemu_ld_i32(cpu_pc, mem, 0, MO_32 | MO_TE);
    ctx->base.is_jmp = DISAS_JUMP;
    tcg_temp_free(mem);
    return true;
}

static bool trans_BSR(DisasContext *ctx, arg_BSR *a)
{
    save_pc(ctx);
    gen_goto_tb(ctx, 0, ctx->base.pc_next + a->dsp);
    return true;
}

static bool trans_RTS(DisasContext *ctx, arg_RTS *a)
{
    tcg_gen_qemu_ld_i32(cpu_pc, cpu_sp, 0, MO_32 | MO_TE);
    tcg_gen_addi_i32(cpu_sp, cpu_sp, 4);
    ctx->base.is_jmp = DISAS_JUMP;
    return true;
}

static bool trans_RTE(DisasContext *ctx, arg_RTE *a)
{
    TCGv temp1, temp2, reg;
    temp1 = tcg_temp_new();
    temp2 = tcg_temp_new();
    reg = tcg_temp_new();
    if (ctx->base.tb->flags == 2) {
        tcg_gen_qemu_ld_i32(temp1, cpu_sp, 0, MO_16 | MO_TE);
        tcg_gen_addi_i32(cpu_sp, cpu_sp, 2);
        tcg_gen_extract_i32(temp2, temp1, 8, 8);
        tcg_gen_movi_i32(reg, 1);
        gen_helper_set_ccr(cpu_env, reg, temp2);
    }
    tcg_gen_qemu_ld_i32(temp1, cpu_sp, 0, MO_32 | MO_TE);
    tcg_gen_addi_i32(cpu_sp, cpu_sp, 4);
    tcg_gen_extract_i32(cpu_pc, temp1, 0, 24);
    tcg_gen_extract_i32(temp2, temp1, 24, 8);
    tcg_gen_movi_i32(reg, 0);
    gen_helper_set_ccr(cpu_env, reg, temp2);
    ctx->base.is_jmp = DISAS_EXIT;
    tcg_temp_free(temp1);
    tcg_temp_free(temp2);
    tcg_temp_free(reg);
    return true;
}

static bool trans_TRAPA(DisasContext *ctx, arg_TRAPA *a)
{
    TCGv vec;

    tcg_debug_assert(a->imm < 4);
    vec = tcg_const_i32(a->imm);
    tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
    gen_helper_trapa(cpu_env, vec);
    tcg_temp_free(vec);
    ctx->base.is_jmp = DISAS_NORETURN;
    return true;
}

static bool trans_SLEEP(DisasContext *ctx, arg_SLEEP *a)
{
    tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
    gen_helper_sleep(cpu_env);
    return true;
}

static bool trans_LDC_i(DisasContext *ctx, arg_LDC_i *a)
{
    TCGv val, reg;
    val = tcg_const_i32(a->imm);
    reg = tcg_const_i32(a->c);
    gen_helper_set_ccr(cpu_env, reg, val);
    ctx->base.is_jmp = DISAS_UPDATE;
    tcg_temp_free(val);
    tcg_temp_free(reg);
    return true;
}

static bool trans_LDC_r(DisasContext *ctx, arg_LDC_r *a)
{
    TCGv val, reg;
    val = tcg_temp_new();
    h8300_gen_reg_ldb(a->r, val, false);
    reg = tcg_const_i32(a->sz);
    gen_helper_set_ccr(cpu_env, reg, val);
    ctx->base.is_jmp = DISAS_UPDATE;
    tcg_temp_free(val);
    tcg_temp_free(reg);
    return true;
}

static bool trans_LDC_m(DisasContext *ctx, arg_LDC_m *a)
{
    TCGv val, mem, reg;
    val = tcg_temp_new();
    mem = tcg_temp_new();
    reg = tcg_const_i32(a->c);
    tcg_gen_addi_i32(mem, cpu_regs[a->r], a->dsp);
    tcg_gen_qemu_ld_i32(val, mem, 0, MO_16 | MO_TE);
    tcg_gen_shri_i32(val, val, 8);
    gen_helper_set_ccr(cpu_env, reg, val);
    ctx->base.is_jmp = DISAS_UPDATE;
    tcg_temp_free(val);
    tcg_temp_free(mem);
    tcg_temp_free(reg);
    return true;
}

static bool trans_LDC_mp(DisasContext *ctx, arg_LDC_mp *a)
{
    TCGv val, reg;
    val = tcg_temp_new();
    reg = tcg_const_i32(a->c);
    tcg_gen_qemu_ld_i32(val, cpu_regs[a->r], 0, MO_16 | MO_TE);
    tcg_gen_addi_i32(cpu_regs[a->r], cpu_regs[a->r], 4);
    tcg_gen_shri_i32(val, val, 8);
    gen_helper_set_ccr(cpu_env, reg, val);
    ctx->base.is_jmp = DISAS_UPDATE;
    tcg_temp_free(val);
    tcg_temp_free(reg);
    return true;
}

static bool trans_LDC_a(DisasContext *ctx, arg_LDC_a *a)
{
    TCGv val, mem, reg;
    val = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    reg = tcg_const_i32(a->c);
    tcg_gen_qemu_ld_i32(val, mem, 0, MO_16 | MO_TE);
    tcg_gen_shri_i32(val, val, 8);
    gen_helper_set_ccr(cpu_env, reg, val);
    ctx->base.is_jmp = DISAS_UPDATE;
    tcg_temp_free(val);
    tcg_temp_free(mem);
    tcg_temp_free(reg);
    return true;
}

static bool trans_LDCSTC_m(DisasContext *ctx, arg_LDCSTC_m *a)
{
    TCGv val, mem, reg;
    val = tcg_temp_new();
    mem = tcg_temp_new();
    reg = tcg_const_i32(a->c);
    tcg_gen_addi_i32(mem, cpu_regs[a->r], a->dsp);
    if (a->ldst == 2) {
        tcg_gen_qemu_ld_i32(val, mem, 0, MO_16 | MO_TE);
        tcg_gen_shri_i32(val, val, 8);
        gen_helper_set_ccr(cpu_env, reg, val);
        ctx->base.is_jmp = DISAS_UPDATE;
    } else {
        gen_helper_get_ccr(val, cpu_env, reg);
        tcg_gen_shli_i32(val, val, 8);
        tcg_gen_qemu_st_i32(val, mem, 0, MO_16 | MO_TE);
    }
    tcg_temp_free(val);
    tcg_temp_free(mem);
    tcg_temp_free(reg);
    return true;
}

static bool trans_STC_r(DisasContext *ctx, arg_STC_r *a)
{
    TCGv val, reg;
    val = tcg_temp_new();
    reg = tcg_const_i32(a->sz);
    gen_helper_get_ccr(val, cpu_env, reg);
    h8300_gen_reg_stb(a->r, val);
    tcg_temp_free(val);
    tcg_temp_free(reg);
    return true;
}

static bool trans_STC_m(DisasContext *ctx, arg_STC_m *a)
{
    TCGv val, mem, reg;
    val = tcg_temp_new();
    mem = tcg_temp_new();
    reg = tcg_const_i32(a->c);
    tcg_gen_addi_i32(mem, cpu_regs[a->r], a->dsp);
    gen_helper_get_ccr(val, cpu_env, reg);
    tcg_gen_shli_i32(val, val, 8);
    tcg_gen_qemu_st_i32(val, mem, 0, MO_16 | MO_TE);
    tcg_temp_free(val);
    tcg_temp_free(mem);
    tcg_temp_free(reg);
    return true;
}

static bool trans_STC_mp(DisasContext *ctx, arg_STC_mp *a)
{
    TCGv val, reg;
    val = tcg_temp_new();
    reg = tcg_const_i32(a->c);
    gen_helper_get_ccr(val, cpu_env, reg);
    tcg_gen_shli_i32(val, val, 8);
    tcg_gen_subi_i32(cpu_regs[a->r], cpu_regs[a->r], 4);
    tcg_gen_qemu_st_i32(val, cpu_regs[a->r], 0, MO_16 | MO_TE);
    tcg_temp_free(val);
    tcg_temp_free(reg);
    return true;
}

static bool trans_STC_a(DisasContext *ctx, arg_STC_a *a)
{
    TCGv val, mem, reg;
    val = tcg_temp_new();
    mem = tcg_const_i32(a->abs);
    reg = tcg_const_i32(a->c);
    gen_helper_get_ccr(val, cpu_env, reg);
    tcg_gen_shli_i32(val, val, 8);
    tcg_gen_qemu_st_i32(val, mem, 0, MO_16 | MO_TE);
    tcg_temp_free(val);
    tcg_temp_free(mem);
    tcg_temp_free(reg);
    return true;
}

static bool trans_ANDC(DisasContext *ctx, arg_ANDC *a)
{
    TCGv val, reg;
    val = tcg_temp_new();
    reg = tcg_const_i32(a->c);
    gen_helper_get_ccr(val, cpu_env, reg);
    tcg_gen_andi_i32(val, val, a->imm);
    gen_helper_set_ccr(cpu_env, reg, val);
    ctx->base.is_jmp = DISAS_UPDATE;
    tcg_temp_free(val);
    tcg_temp_free(reg);
    return true;
}

static bool trans_ORC(DisasContext *ctx, arg_ORC *a)
{
    TCGv val, reg;
    val = tcg_temp_new();
    reg = tcg_const_i32(a->c);
    gen_helper_get_ccr(val, cpu_env, reg);
    tcg_gen_ori_i32(val, val, a->imm);
    gen_helper_set_ccr(cpu_env, reg, val);
    ctx->base.is_jmp = DISAS_UPDATE;
    tcg_temp_free(val);
    tcg_temp_free(reg);
    return true;
}

static bool trans_XORC(DisasContext *ctx, arg_XORC *a)
{
    TCGv val, reg;
    val = tcg_temp_new();
    reg = tcg_const_i32(a->c);
    gen_helper_get_ccr(val, cpu_env, reg);
    tcg_gen_xori_i32(val, val, a->imm);
    gen_helper_set_ccr(cpu_env, reg, val);
    ctx->base.is_jmp = DISAS_UPDATE;
    tcg_temp_free(val);
    tcg_temp_free(reg);
    return true;
}

static bool trans_NOP(DisasContext *ctx, arg_NOP *a)
{
    return true;
}

static bool trans_EEPMOV_B(DisasContext *ctx, arg_EEPMOV_B *a)
{
    gen_helper_eepmovb(cpu_env);
    return true;
}

static bool trans_EEPMOV_W(DisasContext *ctx, arg_EEPMOV_W *a)
{
    gen_helper_eepmovw(cpu_env);
    return true;
}

static bool trans_CLRMAC(DisasContext *ctx, arg_CLRMAC *a)
{
    return true;
}

static bool trans_LDMAC(DisasContext *ctx, arg_LDMAC *a)
{
    return true;
}

static bool trans_MAC(DisasContext *ctx, arg_MAC *a)
{
    return true;
}

static bool trans_STMAC(DisasContext *ctx, arg_STMAC *a)
{
    return true;
}

static bool trans_TAS(DisasContext *ctx, arg_TAS *a)
{
    return true;
}

static void h8300_tr_init_disas_context(DisasContextBase *dcbase, CPUState *cs)
{
    CPUH8300State *env = cs->env_ptr;
    DisasContext *ctx = container_of(dcbase, DisasContext, base);
    ctx->env = env;
}

static void h8300_tr_tb_start(DisasContextBase *dcbase, CPUState *cs)
{
}

static void h8300_tr_insn_start(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    tcg_gen_insn_start(ctx->base.pc_next);
}

#if 0
static bool h8300_tr_breakpoint_check(DisasContextBase *dcbase, CPUState *cs,
                                    const CPUBreakpoint *bp)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    /* We have hit a breakpoint - make sure PC is up-to-date */
    tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
    gen_helper_debug(cpu_env);
    ctx->base.is_jmp = DISAS_NORETURN;
    ctx->base.pc_next += 1;
    return true;
}
#endif

static void h8300_tr_translate_insn(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);
    uint32_t insn;

    ctx->pc = ctx->base.pc_next;
    insn = decode_load(ctx);
    if (!decode(ctx, insn)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Unknwon instruction at 0x%06x", ctx->pc);
    }
}

static void h8300_tr_tb_stop(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    switch (ctx->base.is_jmp) {
    case DISAS_NEXT:
    case DISAS_TOO_MANY:
        gen_goto_tb(ctx, 0, dcbase->pc_next);
        break;
    case DISAS_JUMP:
        if (ctx->base.singlestep_enabled) {
            gen_helper_debug(cpu_env);
        } else {
            tcg_gen_lookup_and_goto_ptr();
        }
        break;
    case DISAS_UPDATE:
        tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
        /* fall through */
    case DISAS_EXIT:
        tcg_gen_exit_tb(NULL, 0);
        break;
    case DISAS_NORETURN:
        break;
    default:
        g_assert_not_reached();
    }
}

static void h8300_tr_disas_log(const DisasContextBase *dcbase, CPUState *cs)
{
    qemu_log("IN:\n");  /* , lookup_symbol(dcbase->pc_first)); */
    log_target_disas(cs, dcbase->pc_first, dcbase->tb->size);
}

static const TranslatorOps h8300_tr_ops = {
    .init_disas_context = h8300_tr_init_disas_context,
    .tb_start           = h8300_tr_tb_start,
    .insn_start         = h8300_tr_insn_start,
    .translate_insn     = h8300_tr_translate_insn,
    .tb_stop            = h8300_tr_tb_stop,
    .disas_log          = h8300_tr_disas_log,
};

void gen_intermediate_code(CPUState *cs, TranslationBlock *tb, int max_insns)
{
    DisasContext dc;

    translator_loop(&h8300_tr_ops, &dc.base, cs, tb, max_insns);
}

void restore_state_to_opc(CPUH8300State *env, TranslationBlock *tb,
                          target_ulong *data)
{
    env->pc = data[0];
}

#define ALLOC_REGISTER(sym, name) \
    cpu_##sym = tcg_global_mem_new_i32(cpu_env, \
                                       offsetof(CPUH8300State, sym), name)

void h8300_translate_init(void)
{
    static const char * const regnames[NUM_REGS] = {
        "R0", "R1", "R2", "R3", "R4", "R5", "R6", "R7",
    };
    int i;

    for (i = 0; i < NUM_REGS; i++) {
        cpu_regs[i] = tcg_global_mem_new_i32(cpu_env,
                                              offsetof(CPUH8300State, regs[i]),
                                              regnames[i]);
    }
    ALLOC_REGISTER(pc, "PC");
    ALLOC_REGISTER(ccr_v, "CCR(V)");
    ALLOC_REGISTER(ccr_n, "CCR(N)");
    ALLOC_REGISTER(ccr_z, "CCR(Z)");
    ALLOC_REGISTER(ccr_c, "CCR(C)");
    ALLOC_REGISTER(ccr_u, "CCR(U)");
    ALLOC_REGISTER(ccr_h, "CCR(H)");
    ALLOC_REGISTER(ccr_ui, "CCR(UI)");
    ALLOC_REGISTER(ccr_i, "CCR(I)");
}
