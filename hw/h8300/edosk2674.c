/*
 * edosk2674 emulation
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
#include "hw/h8300/h8s2674.h"
#include "sysemu/sysemu.h"
#include "sysemu/qtest.h"
#include "sysemu/device_tree.h"
#include "hw/boards.h"

#define DRAM_BASE 0x00400000

static void edosk2674_init(MachineState *machine)
{
    H8S2674State *s = g_new(H8S2674State, 1);
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *sdram = g_new(MemoryRegion, 1);
    MemoryRegion *from = g_new(MemoryRegion, 1);
    const char *kernel_filename = machine->kernel_filename;
    const char *dtb_filename = machine->dtb;
    void *dtb = NULL;
    int dtb_size;

    /* Allocate memory space */
    memory_region_init_ram(sdram, NULL, "sdram", 8 * MiB,
                           &error_fatal);
    memory_region_add_subregion(sysmem, DRAM_BASE, sdram);
    memory_region_init_rom(from, NULL, "flash",
                           4 * MiB, &error_fatal);
    memory_region_add_subregion(sysmem, 0, from);

    if (!kernel_filename) {
        rom_add_file_fixed(bios_name, 0, 0);
    }

    /* Initalize CPU */
    object_initialize_child(OBJECT(machine), "mcu", s,
                            sizeof(H8S2674State), TYPE_H8S2674,
                            &error_fatal, NULL);
    object_property_set_link(OBJECT(s), OBJECT(get_system_memory()),
                             "memory", &error_abort);
    object_property_set_uint(OBJECT(s), 33333333,
                               "clock-freq", &error_abort);
    object_property_set_bool(OBJECT(s), true, "realized", &error_abort);

    /* Load kernel and dtb */
    if (kernel_filename) {
        h8300_load_image(H8300CPU(first_cpu), kernel_filename,
                      DRAM_BASE + 4 * MiB, 4 * MiB);
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
            H8300CPU(first_cpu)->env.regs[0] = DRAM_BASE + 4 * MiB - dtb_size;
        }
    }
}

static void edosk2674_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "EDOSK2674";
    mc->init = edosk2674_init;
    mc->is_default = 0;
    mc->default_cpu_type = TYPE_H8300CPU;
}

static const TypeInfo edosk2674_type = {
    .name = MACHINE_TYPE_NAME("edosk2674"),
    .parent = TYPE_MACHINE,
    .class_init = edosk2674_class_init,
};

static void edosk2674_machine_init(void)
{
    type_register_static(&edosk2674_type);
}

type_init(edosk2674_machine_init)
