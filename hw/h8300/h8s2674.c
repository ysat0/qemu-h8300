/*
 * H8S2674 device
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
#include "hw/h8300/h8s2674.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "sysemu/sysemu.h"
#include "cpu.h"

static uint64_t intcr_read(void *opaque, hwaddr addr, unsigned size)
{
    H8S2674State *s = (H8S2674State *)opaque;
    
    return deposit32(s->intcr_val, 4, 2, s->cpu.env.im);
}

static void intcr_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    H8S2674State *s = (H8S2674State *)opaque;

    s->intcr_val = val;
    s->cpu.env.im = extract32(val, 4, 2);
}

static const MemoryRegionOps intcr_ops = {
    .write = intcr_write,
    .read  = intcr_read,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .max_access_size = 1,
    },
};

static void register_intc(H8S2674State *s)
{
    int i;
    SysBusDevice *intc;

    object_initialize_child(OBJECT(s), "intc", &s->intc,
                            sizeof(H8SINTCState),
                            TYPE_H8SINTC, &error_abort, NULL);

    intc = SYS_BUS_DEVICE(&s->intc);
    sysbus_mmio_map(intc, 0, H8S2674_INTCBASE1);
    sysbus_mmio_map(intc, 1, H8S2674_INTCBASE2);

    for (i = 0; i < NR_IRQS; i++) {
        s->irq[i] = qdev_get_gpio_in(DEVICE(intc), i);
    }

    qdev_init_nofail(DEVICE(intc));
    sysbus_connect_irq(intc, 0,
                       qdev_get_gpio_in(DEVICE(&s->cpu), H8300_CPU_IRQ));
}

static void register_tmr(H8S2674State *s)
{
    SysBusDevice *tmr;
    int i;

    object_initialize_child(OBJECT(s), "tmr", &s->tmr,
                            sizeof(RTMRState), TYPE_RENESAS_TMR,
                            &error_abort, NULL);

    tmr = SYS_BUS_DEVICE(&s->tmr);
    sysbus_mmio_map(tmr, 0, H8S2674_TMRBASE);
    qdev_prop_set_uint64(DEVICE(tmr), "input-freq", s->input_freq);
    qdev_prop_set_uint32(DEVICE(tmr), "timer-type", 1);

    qdev_init_nofail(DEVICE(tmr));
    for (i = 0; i < TMR_NR_IRQ; i++) {
        sysbus_connect_irq(tmr, i, s->irq[H8S2674_TMR_IRQBASE + i]);
    }
}

static void register_tpu(H8S2674State *s)
{
    SysBusDevice *tpu;
    int i;

    object_initialize_child(OBJECT(s), "tpu", &s->tpu,
                            sizeof(RTPUState), TYPE_RENESAS_TPU,
                            &error_abort, NULL);

    tpu = SYS_BUS_DEVICE(&s->tpu);
    sysbus_mmio_map(tpu, 0, H8S2674_TPUBASE1);
    sysbus_mmio_map(tpu, 1, H8S2674_TPUBASE2);
    sysbus_mmio_map(tpu, 2, H8S2674_TPUBASE3);
    qdev_prop_set_uint64(DEVICE(tpu), "input-freq", s->input_freq);

    qdev_init_nofail(DEVICE(tpu));
    for (i = 0; i < TPU_NR_IRQ; i++) {
        sysbus_connect_irq(tpu, i, s->irq[H8S2674_TPU_IRQBASE + i]);
    }
}

static void register_sci(H8S2674State *s, int unit)
{
    SysBusDevice *sci;
    int i, irqbase;

    object_initialize_child(OBJECT(s), "sci[*]", &s->sci[unit],
                            sizeof(RSCIState), TYPE_RENESAS_SCI,
                            &error_abort, NULL);

    sci = SYS_BUS_DEVICE(&s->sci[unit]);
    sysbus_mmio_map(sci, 0, H8S2674_SCIBASE + unit * 0x08);
    qdev_prop_set_chr(DEVICE(sci), "chardev", serial_hd(unit));
    qdev_prop_set_uint64(DEVICE(sci), "input-freq", s->input_freq);
    qdev_prop_set_uint32(DEVICE(sci), "rev", 0);

    qdev_init_nofail(DEVICE(sci));
    irqbase = H8S2674_SCI_IRQBASE + SCI_NR_IRQ * unit;
    for (i = 0; i < SCI_NR_IRQ; i++) {
        sysbus_connect_irq(sci, i, s->irq[irqbase + i]);
    }
}

static void h8s2674_realize(DeviceState *dev, Error **errp)
{
    H8S2674State *s = H8S2674(dev);

    memory_region_init_ram(&s->iram, NULL, "iram", H8S2674_IRAM_SIZE, errp);
    memory_region_add_subregion(s->sysmem, H8S2674_IRAM_BASE, &s->iram);
    memory_region_init_io(&s->intcr, OBJECT(dev), &intcr_ops,
                          &s, "h8s2674-intcr", 0x1);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->intcr);

    object_initialize_child(OBJECT(s), "cpu", &s->cpu,
                            sizeof(H8300CPU), TYPE_H8300CPU,
                            errp, NULL);
    object_property_set_bool(OBJECT(&s->cpu), true, "realized", errp);

    register_intc(s);
    s->cpu.env.ack = qdev_get_gpio_in_named(DEVICE(&s->intc), "ack", 0);
    register_tmr(s);
    register_sci(s, 0);
    register_tpu(s);
}

static Property h8s2674_properties[] = {
    DEFINE_PROP_LINK("memory", H8S2674State, sysmem, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_UINT64("clock-freq", H8S2674State, input_freq, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void h8s2674_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = h8s2674_realize;
    dc->props = h8s2674_properties;
}

static const TypeInfo h8s2674_info = {
    .name = TYPE_H8S2674,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(H8S2674State),
    .class_init = h8s2674_class_init,
};

static void h8s2674_register_types(void)
{
    type_register_static(&h8s2674_info);
}

type_init(h8s2674_register_types)
