/*
 * RX QEMU virtual platform
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
#include "hw/rx/rx62n.h"
#include "sysemu/sysemu.h"
#include "sysemu/qtest.h"
#include "sysemu/device_tree.h"
#include "hw/boards.h"

/* Same address of GDB integrated simulator */
#define SDRAM_BASE 0x01000000

static void rxvirt_init(MachineState *machine)
{
    RX62NState *s = g_new(RX62NState, 1);
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *sdram = g_new(MemoryRegion, 1);
    const char *kernel_filename = machine->kernel_filename;
    const char *dtb_filename = machine->dtb;
    void *dtb = NULL;
    int dtb_size;

    /* Allocate memory space */
    memory_region_init_ram(sdram, NULL, "sdram", 16 * MiB,
                           &error_fatal);
    memory_region_add_subregion(sysmem, SDRAM_BASE, sdram);

    /* Initalize CPU */
    object_initialize_child(OBJECT(machine), "mcu", s,
                            sizeof(RX62NState), TYPE_RX62N,
                            &error_fatal, NULL);
    object_property_set_link(OBJECT(s), OBJECT(get_system_memory()),
                             "memory", &error_abort);
    object_property_set_bool(OBJECT(s), kernel_filename != NULL,
                             "load-kernel", &error_abort);
    object_property_set_bool(OBJECT(s), true, "realized", &error_abort);

    /* Load kernel and dtb */
    if (kernel_filename) {
        rx_load_image(RXCPU(first_cpu), kernel_filename,
                      SDRAM_BASE + 8 * MiB, 8 * MiB);
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
                               SDRAM_BASE + 16 * MiB - dtb_size);
            /* Set dtb address to R1 */
            RXCPU(first_cpu)->env.regs[1] = 0x02000000 - dtb_size;
        }
    }
}

static void rxvirt_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "RX QEMU Virtual Target";
    mc->init = rxvirt_init;
    mc->is_default = 1;
    mc->default_cpu_type = TYPE_RXCPU;
}

static const TypeInfo rxvirt_type = {
    .name = MACHINE_TYPE_NAME("rx-virt"),
    .parent = TYPE_MACHINE,
    .class_init = rxvirt_class_init,
};

static void rxvirt_machine_init(void)
{
    type_register_static(&rxvirt_type);
}

type_init(rxvirt_machine_init)
