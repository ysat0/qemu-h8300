/*
 * H8/300H Interrupt Controller
 *
 * Copyright (c) 2019 Yoshinori Sato
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTINTCLAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/intc/h8300h_intc.h"
#include "qemu/error-report.h"
#include "qemu/bitops.h"

REG8(ISCR, 0)
  FIELD(ISCR, IRQSC, 0, 6)
REG8(IER, 1)
  FIELD(IER, IRQE, 0, 6)
REG8(ISR, 2)
  FIELD(ISR, IRQF, 0, 6)
REG8(IPRA, 4)
REG8(IPRB, 5)

static inline int pri(H8300HINTCState *intc, int irq)
{
    static const int primap[] = {
        -1, -1, -1, -1, -1, -1, -1, -1, 
        -1, -1, -1, -1,  7,  6,  5,  5,
         4,  4, -1, -1,  3,  3,  3,  3,
         2,  2,  2, -1,  1,  1,  1, -1,
         0,  0,  0, -1, 15, 15, 15, 15,
        14, 14, 14, 14, 13, 13, 13, 13,
        -1, -1, -1, -1, 11, 11, 11, 11,
        10, 10, 10, 10,  9,  9,  9,  9,
    };
    if (primap[irq] > 0) {
        return extract16(intc->ipr, primap[irq], 1);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "h8300h_intc: Undedind irq %d\n", irq);
        return -1;
    }
}

static inline int ext_no(int irq)
{
    if (irq < 7 || irq > 12) {
        return -1;
    }
    return irq - 7;
}

static void h8300hintc_set_irq(void *opaque, int n_IRQ, int level)
{
    H8300HINTCState *intc = opaque;
    bool enable = true;

    if (n_IRQ >= NR_IRQS) {
        error_report("%s: IRQ %d out of range", __func__, n_IRQ);
        return;
    }

    if (ext_no(n_IRQ) >= 0) {
        if (extract8(intc->iscr, ext_no(n_IRQ), 1) && 
            (extract8(intc->irqin, ext_no(n_IRQ), 1) ^ level) == 0) {
                /* No trigger */
                return;
            }
        intc->irqin = deposit8(intc->irqin, ext_no(n_IRQ), 1, level);
        if (extract8(intc->ier, ext_no(n_IRQ), 1) == 0) {
            enable = false;
        }
    }
    if (level) {
        if (enable) {
            intc->req = deposit64(intc->req, n_IRQ, 1, 1);
            if (atomic_read(&intc->req_irq) < 0) {
                atomic_set(&intc->req_irq, n_IRQ);
                qemu_set_irq(intc->irq, (pri(intc, n_IRQ) << 8) | n_IRQ);
            }
        }
    } else {
        intc->req = deposit64(intc->req, n_IRQ, 1, 0);
        if (atomic_read(&intc->req_irq) == n_IRQ) {
            atomic_set(&intc->req_irq, -1);
            qemu_set_irq(intc->irq, 0);
        }
    }
}

static void h8300hintc_ack_irq(void *opaque, int no, int level)
{
    H8300HINTCState *intc = opaque;
    int i;
    int n_IRQ;
    int max_pri;

    n_IRQ = atomic_read(&intc->req_irq);
    if (n_IRQ < 0) {
        return;
    }
    atomic_set(&intc->req_irq, -1);
    intc->req = deposit64(intc->req, n_IRQ, 1, 0);
    if (ext_no(n_IRQ) >= 0) {
        if (extract8(intc->iscr, ext_no(n_IRQ), 1)) {
            intc->isr = deposit8(intc->isr, ext_no(n_IRQ), 1, 0);
        } else {
            if (extract8(intc->irqin, ext_no(n_IRQ), 1) == 0) {
                intc->isr = deposit8(intc->isr, ext_no(n_IRQ), 1, 0);
            }
        }
    }
        
    max_pri = 0;
    n_IRQ = -1;
    i = 0;
    do {
        i =  find_next_bit(&intc->req, 64, i);
        if (i < 64 && pri(intc, i) > max_pri) {
            n_IRQ = i;
            max_pri = pri(intc, i);
        }
        i++;
    } while (i < 64);

    if (n_IRQ >= 0) {
        atomic_set(&intc->req_irq, n_IRQ);
        qemu_set_irq(intc->irq, (pri(intc, n_IRQ) << 8) | n_IRQ);
    }
}

static void clear_pend_irq(H8300HINTCState *intc)
{
    int i;
    for (i = 0; i < 6; i++) {
        if (extract8(intc->isr, i, 1) == 0 &&
            extract64(intc->req, i + 12, 1)) {
            intc->req = deposit64(intc->req, i + 12, 1, 0);
            if (atomic_read(&intc->req_irq) == i + 12) {
                h8300hintc_ack_irq(intc, i + 12, 0);
            }
        }
    }
}

static uint64_t intc_read(void *opaque, hwaddr addr, unsigned size)
{
    H8300HINTCState *intc = opaque;
    switch(addr) {
    case A_ISCR:
        return intc->iscr;
    case A_IER:
        return intc->ier;
    case A_ISR:
        return intc->isr;
    case A_IPRA:
        return extract16(intc->ipr, 8, 8);
        break;
    case A_IPRB:
        return extract16(intc->ipr, 0, 8);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "h8300h_intc: Register 0x%" HWADDR_PRIX
                      " not implemented\n", addr);
        break;
    }
    return UINT64_MAX;
}

static void intc_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    H8300HINTCState *intc = opaque;
    switch(addr) {
    case A_ISCR:
        intc->iscr = val;
        break;
    case A_IER:
        intc->ier = val;
        break;
    case A_ISR:
        intc->isr &= val;
        clear_pend_irq(intc);
        break;
    case A_IPRA:
        intc->ipr = deposit16(intc->ipr, 8, 8, val);
        break;
    case A_IPRB:
        intc->ipr = deposit16(intc->ipr, 0, 8, val);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "h8300h_intc: Register 0x%" HWADDR_PRIX
                      " not implemented\n", addr);
        break;
    }
}

static const MemoryRegionOps intc_ops = {
    .write = intc_write,
    .read  = intc_read,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .max_access_size = 2,
    },
};

static void h8300hintc_realize(DeviceState *dev, Error **errp)
{
    H8300HINTCState *intc = H8300HINTC(dev);

    intc->req_irq = -1;
}

static void h8300hintc_init(Object *obj)
{
    SysBusDevice *d = SYS_BUS_DEVICE(obj);
    H8300HINTCState *intc = H8300HINTC(obj);

    memory_region_init_io(&intc->memory, OBJECT(intc), &intc_ops,
                          intc, "h8300h-intc", 0x6);
    sysbus_init_mmio(d, &intc->memory);

    qdev_init_gpio_in(DEVICE(d), h8300hintc_set_irq, NR_IRQS);
    qdev_init_gpio_in_named(DEVICE(d), h8300hintc_ack_irq, "ack", 1);
    sysbus_init_irq(d, &intc->irq);
}

static const VMStateDescription vmstate_h8300hintc = {
    .name = "h8300h-intc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static void h8300hintc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = h8300hintc_realize;
    dc->vmsd = &vmstate_h8300hintc;
}

static const TypeInfo h8300hintc_info = {
    .name       = TYPE_H8300HINTC,
    .parent     = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(H8300HINTCState),
    .instance_init = h8300hintc_init,
    .class_init = h8300hintc_class_init,
};

static void h8300hintc_register_types(void)
{
    type_register_static(&h8300hintc_info);
}

type_init(h8300hintc_register_types)
