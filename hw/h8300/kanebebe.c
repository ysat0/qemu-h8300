/*
 * KaneBebe emulation
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
#include "qapi/error.h"
#include "qemu-common.h"
#include "cpu.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/loader.h"
#include "hw/h8300/h83069.h"
#include "hw/net/ne2000-local.h"
#include "sysemu/sysemu.h"
#include "sysemu/qtest.h"
#include "sysemu/device_tree.h"
#include "hw/boards.h"
#include "qom/object.h"

#define DRAM_BASE 0x00400000
#define DRAM_SIZE 4 * MiB

struct KaneBebeMachineClass {
    /*< private >*/
    MachineClass parent_class;
    /*< public >*/
};
typedef struct KaneBebeMachineClass KaneBebeMachineClass;

struct KaneBebeMachineState {
    /*< private >*/
    MachineState parent_obj;
    /*< public >*/
    H83069State mcu;
};
typedef struct KaneBebeMachineState KaneBebeMachineState;

#define TYPE_KANEBEBE_MACHINE MACHINE_TYPE_NAME("KaneBebe")

DECLARE_OBJ_CHECKERS(KaneBebeMachineState, KaneBebeMachineClass,
                     KANEBEBE_MACHINE, TYPE_KANEBEBE_MACHINE)

static void setup_vector(unsigned int base)
{
    uint32_t rom_vec[64];
    int i;

    for (i = 0; i < ARRAY_SIZE(rom_vec); i++) {
        rom_vec[i] = cpu_to_be32(base + i * 4);
    }
    rom_add_blob_fixed("vector", rom_vec, sizeof(rom_vec), 0x000000);
}

static void kanebebe_init(MachineState *machine)
{
    KaneBebeMachineState *s = KANEBEBE_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *dram = g_new(MemoryRegion, 1);
    const char *kernel_filename = machine->kernel_filename;
    const char *dtb_filename = machine->dtb;
    void *dtb = NULL;
    int dtb_size;

    /* Allocate memory space */
    memory_region_init_ram(dram, NULL, "kanebebe.dram",
                           DRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, DRAM_BASE, dram);

    if (!kernel_filename) {
        rom_add_file_fixed(machine->firmware, 0, 0);
    }

    /* Initalize CPU */
    object_initialize_child(OBJECT(machine), "mcu", &s->mcu, TYPE_H83069);
    object_property_set_link(OBJECT(&s->mcu), "memory", OBJECT(sysmem),
                             &error_abort);
    object_property_set_uint(OBJECT(&s->mcu), "clock-freq", 25000000, &error_abort);
    object_property_set_uint(OBJECT(&s->mcu), "console", 1, &error_abort);
    qdev_realize(DEVICE(&s->mcu), NULL, &error_abort);

    ne2000_init(&nd_table[0], 0x200000, s->mcu.irq[17]);

    /* Load kernel and dtb */
    if (kernel_filename) {
        h8300_load_image(H8300_CPU(first_cpu), kernel_filename,
                      DRAM_BASE + 0x280000, 0x180000);
        setup_vector(0xffff20 - 0x100);
        if (dtb_filename) {
            dtb = load_device_tree(dtb_filename, &dtb_size);
            if (dtb == NULL) {
                fprintf(stderr, "Couldn't open dtb file %s\n", dtb_filename);
                exit(1);
            }
            if (machine->kernel_cmdline &&
                qemu_fdt_setprop_string(dtb, "/chosen", "bootargs",
                                        machine->kernel_cmdline) < 0) {
                fprintf(stderr, "couldn't set /chosen/bootargs\n");
                exit(1);
            }
            rom_add_blob_fixed("dtb", dtb, dtb_size,
                               DRAM_BASE + 4 * MiB - dtb_size);
            /* Set dtb address to R0 */
            H8300_CPU(first_cpu)->env.regs[0] = DRAM_BASE + 4 * MiB - dtb_size;
        }
    }
}

static void kanebebe_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "KaneBebe";
    mc->init = kanebebe_init;
    mc->is_default = 1;
    mc->default_cpu_type = TYPE_H83069_CPU;
}

static const TypeInfo kanebebe_type[] = {
    {
        .name = MACHINE_TYPE_NAME("KaneBebe"),
        .parent = TYPE_MACHINE,
        .class_init = kanebebe_class_init,
        .instance_size = sizeof(KaneBebeMachineState),
        .class_size = sizeof(KaneBebeMachineClass),
    }
};

DEFINE_TYPES(kanebebe_type)
