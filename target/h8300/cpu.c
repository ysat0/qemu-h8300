/*
 * QEMU H8/300 CPU
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
#include "qemu/qemu-print.h"
#include "qapi/error.h"
#include "cpu.h"
#include "qemu-common.h"
#include "migration/vmstate.h"
#include "exec/exec-all.h"
#include "hw/loader.h"
#include "fpu/softfloat.h"

static void h8300_cpu_set_pc(CPUState *cs, vaddr value)
{
    H8300CPU *cpu = H8300CPU(cs);

    cpu->env.pc = value;
}

static void h8300_cpu_synchronize_from_tb(CPUState *cs, TranslationBlock *tb)
{
    H8300CPU *cpu = H8300CPU(cs);

    cpu->env.pc = tb->pc;
}

static bool h8300_cpu_has_work(CPUState *cs)
{
    return cs->interrupt_request & CPU_INTERRUPT_HARD;
}

static void h8300_cpu_reset(CPUState *s)
{
    H8300CPU *cpu = H8300CPU(s);
    H8300CPUClass *rcc = H8300CPU_GET_CLASS(cpu);
    CPUH8300State *env = &cpu->env;
    uint32_t *resetvec;

    rcc->parent_reset(s);

    memset(env, 0, offsetof(CPUH8300State, end_reset_fields));

    resetvec = rom_ptr(0x000000, 4);
    if (resetvec) {
        /* In the case of kernel, it is ignored because it is not set. */
        env->pc = ldl_p(resetvec);
    }
    h8300_cpu_unpack_ccr(env, 0x80);
}

static void h8300_cpu_list_entry(gpointer data, gpointer user_data)
{
    const char *typename = object_class_get_name(OBJECT_CLASS(data));
    int len = strlen(typename) - strlen(H8300_CPU_TYPE_SUFFIX);

    qemu_printf("%.*s\n", len, typename);
}

void h8300_cpu_list(void)
{
    GSList *list;
    list = object_class_get_list_sorted(TYPE_H8300CPU, false);
    g_slist_foreach(list, h8300_cpu_list_entry, NULL);
    g_slist_free(list);
}

static ObjectClass *h8300_cpu_class_by_name(const char *cpu_model)
{
    ObjectClass *oc;
    char *typename = NULL;

    typename = g_strdup_printf(H8300_CPU_TYPE_NAME(""));
    oc = object_class_by_name(typename);
    if (oc != NULL && object_class_is_abstract(oc)) {
        oc = NULL;
    }

    g_free(typename);
    return oc;
}

static void h8300_cpu_realize(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    H8300CPUClass *rcc = H8300CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    cpu_exec_realizefn(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }

    cpu_reset(cs);
    qemu_init_vcpu(cs);

    rcc->parent_realize(dev, errp);
}

static void h8300_cpu_set_irq(void *opaque, int no, int request)
{
    H8300CPU *cpu = opaque;
    CPUState *cs = CPU(cpu);
    int irq = request & 0xff;

    if (irq) {
        cpu->env.req_irq = irq;
        cpu->env.req_pri = (request >> 8) & 0x03;
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }
}

static void h8300_cpu_disas_set_info(CPUState *cpu, disassemble_info *info)
{
    info->mach = bfd_mach_h8300;
    info->print_insn = print_insn_h8300;
}

static void h8300_cpu_init(Object *obj)
{
    CPUState *cs = CPU(obj);
    H8300CPU *cpu = H8300CPU(obj);
    CPUH8300State *env = &cpu->env;

    cs->env_ptr = env;
    qdev_init_gpio_in(DEVICE(cpu), h8300_cpu_set_irq, 1);
}


static bool h8300_cpu_tlb_fill(CPUState *cs, vaddr addr, int size,
                            MMUAccessType access_type, int mmu_idx,
                            bool probe, uintptr_t retaddr)
{
    uint32_t address, physical, prot;

    /*
      RX has no-MMU
      Only linear mapping
    */
    address = physical = addr & TARGET_PAGE_MASK;
    prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
    tlb_set_page(cs, address, physical, prot, mmu_idx, TARGET_PAGE_SIZE);
    return true;
}

static void rxcpu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    CPUClass *cc = CPU_CLASS(klass);
    H8300CPUClass *rcc = H8300CPU_CLASS(klass);

    device_class_set_parent_realize(dc, h8300_cpu_realize,
                                    &rcc->parent_realize);

    rcc->parent_reset = cc->reset;
    cc->reset = h8300_cpu_reset;

    cc->class_by_name = h8300_cpu_class_by_name;
    cc->has_work = h8300_cpu_has_work;
    cc->do_interrupt = h8300_cpu_do_interrupt;
    cc->cpu_exec_interrupt = h8300_cpu_exec_interrupt;
    cc->dump_state = h8300_cpu_dump_state;
    cc->set_pc = h8300_cpu_set_pc;
    cc->synchronize_from_tb = h8300_cpu_synchronize_from_tb;
    cc->gdb_read_register = h8300_cpu_gdb_read_register;
    cc->gdb_write_register = h8300_cpu_gdb_write_register;
    cc->get_phys_page_debug = h8300_cpu_get_phys_page_debug;
    cc->disas_set_info = h8300_cpu_disas_set_info;
    cc->tcg_initialize = h8300_translate_init;
    cc->tlb_fill = h8300_cpu_tlb_fill;
    cc->gdb_num_core_regs = 26;
}

static const TypeInfo rxcpu_info = {
    .name = TYPE_H8300CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(H8300CPU),
    .instance_init = h8300_cpu_init,
    .abstract = false,
    .class_size = sizeof(H8300CPUClass),
    .class_init = rxcpu_class_init,
};

static void rxcpu_register_types(void)
{
    type_register_static(&rxcpu_info);
}

type_init(rxcpu_register_types)

static uint32_t extable[64];

#define IRAMTOP 0xffbf20

void h8300_load_image(H8300CPU *cpu, const char *filename,
                   uint32_t start, uint32_t size)
{
    long kernel_size;
    int i;

    kernel_size = load_image_targphys(filename, start, size);
    if (kernel_size < 0) {
        fprintf(stderr, "qemu: could not load kernel '%s'\n", filename);
        exit(1);
    }
    cpu->env.pc = start;

    /* setup exception trap trampoline */
    /* linux kernel only works little-endian mode */
    for (i = 0; i < 64; i++) {
        extable[i] = cpu_to_be32(IRAMTOP + (i * 4));
    }
    rom_add_blob_fixed("extable", extable, sizeof(extable), 0x000000);
}
