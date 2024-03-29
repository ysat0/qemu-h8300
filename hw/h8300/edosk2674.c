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
#include "cpu.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/loader.h"
#include "hw/block/flash.h"
#include "hw/net/smc91c111.h"
#include "hw/h8300/h8s2674.h"
#include "sysemu/sysemu.h"
#include "sysemu/qtest.h"
#include "sysemu/device_tree.h"
#include "hw/boards.h"

#define DRAM_BASE 0x00400000

static void setup_vector(unsigned int base)
{
    uint32_t rom_vec[128];
    int i;

    for (i = 0; i < ARRAY_SIZE(rom_vec); i++) {
        rom_vec[i] = cpu_to_be32(base + i * 4);
    }
    rom_add_blob_fixed("vector", rom_vec, sizeof(rom_vec), 0x000000);
}


static void edosk2674_init(MachineState *machine)
{
    H8S2674State *s = g_new(H8S2674State, 1);
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *sdram = g_new(MemoryRegion, 1);
    const char *kernel_filename = machine->kernel_filename;
    const char *dtb_filename = machine->dtb;
    void *dtb = NULL;
    int dtb_size;
    DriveInfo *dinfo;

    /* Allocate memory space */
    memory_region_init_ram(sdram, NULL, "sdram", 8 * MiB,
                           &error_fatal);
    memory_region_add_subregion(sysmem, DRAM_BASE, sdram);
    dinfo = drive_get(IF_PFLASH, 0, 0);
    pflash_cfi01_register(0x0, "edosk2674.flash", 4 * MiB,
                          dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                          128 * KiB, 2, 0x0089, 0x0016, 0x0000, 0x0000,
                          0);

    if (!kernel_filename) {
        rom_add_file_fixed(machine->firmware, 0, 0);
    }

    /* Initalize CPU */
    object_initialize_child(OBJECT(machine), "mcu", &s->cpu, TYPE_H8S2674);
    object_property_set_link(OBJECT(&s->cpu), "main-bus", OBJECT(sysmem),
                             &error_abort);
    object_property_set_uint(OBJECT(s), "clock-freq", 33333333, &error_abort);
    object_property_set_uint(OBJECT(s), "console", 2, &error_abort);
    qdev_realize(DEVICE(&s->cpu), NULL, &error_abort);

    smc91c96_init(&nd_table[0], 0xf80000, s->irq[16]);

    /* Load kernel and dtb */
    if (kernel_filename) {
        h8300_load_image(H8300_CPU(first_cpu), kernel_filename,
                      DRAM_BASE + 4 * MiB, 4 * MiB);
        setup_vector(0xffc000 - 0x200);
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

static void edosk2674_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "EDOSK2674";
    mc->init = edosk2674_init;
    mc->is_default = 0;
    mc->default_cpu_type = TYPE_H8S2674_CPU;
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
