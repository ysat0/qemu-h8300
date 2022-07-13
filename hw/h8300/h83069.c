/*
 * H83069 device
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
#include "hw/hw.h"
#include "hw/h8300/h83069.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "sysemu/sysemu.h"
#include "cpu.h"
#include "hw/qdev-properties.h"
#include "qom/object.h"

struct H83069Class {
    /*< private >*/
    DeviceClass parent_class;
    /*< public >*/
};
typedef struct H83069Class H83069Class;

DECLARE_CLASS_CHECKERS(H83069Class, H83069, TYPE_H83069)

static uint64_t syscr_read(void *opaque, hwaddr addr, unsigned size)
{
    H83069State *s = (H83069State *)opaque;
    
    return s->syscr_val;
}

static void syscr_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    H83069State *s = (H83069State *)opaque;

    s->syscr_val = val;
    s->cpu.env.im = extract32(val, 3, 1) ? 0 : 1;
}

static const MemoryRegionOps syscr_ops = {
    .write = syscr_write,
    .read  = syscr_read,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .max_access_size = 1,
    },
};

static void register_intc(H83069State *s)
{
    int i;
    SysBusDevice *intc;

    object_initialize_child(OBJECT(s), "intc", &s->intc, TYPE_H8300HINTC);

    intc = SYS_BUS_DEVICE(&s->intc);

    for (i = 0; i < NR_IRQS; i++) {
        s->irq[i] = qdev_get_gpio_in(DEVICE(intc), i);
    }

    sysbus_realize(intc, &error_abort);
    sysbus_connect_irq(intc, 0,
                       qdev_get_gpio_in(DEVICE(&s->cpu), H8300_CPU_IRQ));
    sysbus_mmio_map(intc, 0, H83069_INTCBASE);
}

static void register_tmr(H83069State *s, int unit)
{
    SysBusDevice *tmr;
    int i, irqbase;

    object_initialize_child(OBJECT(s), "tmr[*]",
                            &s->tmr[unit], TYPE_RENESAS_TMR);

    tmr = SYS_BUS_DEVICE(&s->tmr[unit]);
    qdev_prop_set_uint64(DEVICE(tmr), "input-freq", s->input_freq);
    qdev_prop_set_uint32(DEVICE(tmr), "timer-type", 1);
    sysbus_realize(tmr, &error_abort);

    irqbase = H83069_TMR_IRQBASE + TMR_NR_IRQ * unit;
    for (i = 0; i < TMR_NR_IRQ; i++) {
        sysbus_connect_irq(tmr, i, s->irq[irqbase + i]);
    }
    sysbus_mmio_map(tmr, 0, H83069_TMRBASE + unit * 0x10);
}

static void register_16tmr(H83069State *s)
{
    SysBusDevice *tmr;
    int i;


    object_initialize_child(OBJECT(s), "16timer",
                            &s->tmr16, TYPE_RENESAS_16TMR);
    tmr = SYS_BUS_DEVICE(&s->tmr16);
    qdev_prop_set_uint64(DEVICE(tmr), "input-freq", s->input_freq);
    sysbus_realize(tmr, &error_abort);

    for (i = 0; i < TMR16_NR_IRQ; i++) {
        sysbus_connect_irq(tmr, i, s->irq[H83069_16TIMER_IRQBASE + i]);
    }
    sysbus_mmio_map(tmr, 0, H83069_16TIMER_BASE);
}

static void register_sci(H83069State *s, int unit)
{
    SysBusDevice *sci;
    int i, irqbase;

    object_initialize_child(OBJECT(s), "sci[*]",
                            &s->sci[unit], TYPE_RENESAS_SCI);
    sci = SYS_BUS_DEVICE(&s->sci[unit]);
    qdev_prop_set_chr(DEVICE(sci), "chardev", serial_hd(0));
    qdev_prop_set_uint64(DEVICE(sci), "input-freq", s->input_freq);
    sysbus_realize(sci, &error_abort);

    irqbase = H83069_SCI_IRQBASE + SCI_NR_IRQ * unit;
    for (i = 0; i < SCI_NR_IRQ; i++) {
        sysbus_connect_irq(sci, i, s->irq[irqbase + i]);
    }
    sysbus_mmio_map(sci, 0, H83069_SCIBASE + unit * 0x08);
}

static void h83069_realize(DeviceState *dev, Error **errp)
{
    H83069State *s = H83069(dev);

    memory_region_init_ram(&s->iram, NULL, "iram", H83069_IRAM_SIZE, errp);
    memory_region_add_subregion(s->sysmem, H83069_IRAM_BASE, &s->iram);
    memory_region_init_ram(&s->flash, NULL, "flash",
                           H83069_FLASH_SIZE, errp);
    memory_region_add_subregion(s->sysmem, H83069_FLASH_BASE, &s->flash);
    memory_region_init_io(&s->syscr, OBJECT(dev), &syscr_ops,
                          s, "h83069-syscr", 0x1);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->syscr);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, H83069_SYSCR);

    object_initialize_child(OBJECT(s), "cpu", &s->cpu, TYPE_H8300_CPU);
    qdev_realize(DEVICE(&s->cpu), NULL, &error_abort);
    s->syscr_val = 0x09;
    register_intc(s);
    s->cpu.env.ack = qdev_get_gpio_in_named(DEVICE(&s->intc), "ack", 0);
    register_tmr(s, 0);
    register_tmr(s, 1);
    register_16tmr(s);
    register_sci(s, s->sci_con);
}

static Property h83069_properties[] = {
    DEFINE_PROP_LINK("memory", H83069State, sysmem, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_UINT64("clock-freq", H83069State, input_freq, 0),
    DEFINE_PROP_UINT32("console", H83069State, sci_con, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void h83069_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = h83069_realize;
    device_class_set_props(dc, h83069_properties);
}

static const TypeInfo h83069_info = {
    .name = TYPE_H83069,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(H83069State),
    .class_init = h83069_class_init,
    .class_size = sizeof(H83069Class),
};

static void h83069_register_types(void)
{
    type_register_static(&h83069_info);
}

type_init(h83069_register_types)
