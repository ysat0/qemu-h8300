/*
 * H8S Interrupt Controller
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
#include "hw/intc/h8s_intc.h"
#include "qemu/error-report.h"
#include "qemu/bitops.h"

REG16(IPRA, 0)
REG16(IPRB, 2)
REG16(IPRC, 4)
REG16(IPRD, 6)
REG16(IPRE, 8)
REG16(IPRF, 10)
REG16(IPRG, 12)
REG16(IPRH, 14)
REG16(IPRI, 16)
REG16(IPRJ, 18)
REG16(IPRK, 20)
REG16(ITSR, 22)
REG16(SSIER, 24)
REG16(ISCRH, 26)
REG16(ISCRL, 28)
REG8(INTCR, 1)
REG16(IER, 2)
REG16(ISR, 4)

static inline int pri(H8SINTCState *intc, int irq)
{
    static const int primap[] = {
        -1, -1, -1, -1, -1, -1, -1, -1, 
        -1, -1, -1, -1, -1, -1, -1, -1,
         3,  2,  1,  0,  7,  6,  5,  4,
        11, 10,  9,  8, 15, 14, 13, 12,
        19, 18, 17, 16, 23, 23, 22, 22,
        21, 21, 21, 21, 21, 21, 21, 21,
        20, 20, 20, 20, 27, 27, 27, 27,
        26, 26, 26, 26, 26, 26, 26, 26,
        25, 25, 25, 25, 24, 24, 24, 24,
        31, 31, 31, 31, 30, 30, 30, 30,
        29, 29, 29, 29, 28, 35, 34, 33,
        32, 32, 32, 32, 39, 39, 39, 39,
        38, 38, 38, 38, 37, 37, 37, 37,
        36, 36, 36, 36, 43, 43, 43, 43,
        42, 42, 42, 42, 41, 41, 41, 41,
        40, 40, 40, 40, 40, 40, 40, 40,
    };
    if (primap[irq] > 0) {
        return extract16(intc->ipr[primap[irq] / 4], (primap[irq] % 4) * 4, 4);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "h8s_intc: Undedind irq %d\n", irq);
        return -1;
    }
}

static inline int ext_no(int irq)
{
    if (irq < 16 || irq >= 32) {
        return -1;
    }
    return irq - 16;
}

static inline void set_pending(H8SINTCState *s, int irq, int req)
{
    int index, offset;
    index = irq / 64;
    offset = irq % 64;
    s->req[index] = deposit64(s->req[index], offset, 1, req);
}

static inline int trigger_mode(H8SINTCState *intc, int no)
{
    switch (no) {
    case 16 ... 32:
        return extract32(intc->iscr, (no - 16) * 2, 2);
    case 72 ... 79:
        return 2;
    default:
        return 0;
    }
}

static void h8sintc_set_irq(void *opaque, int n_IRQ, int level)
{
    H8SINTCState *intc = opaque;
    bool enable = true;

    if (n_IRQ >= NR_IRQS) {
        error_report("%s: IRQ %d out of range", __func__, n_IRQ);
        return;
    }

    switch (trigger_mode(intc, n_IRQ)) {
    case 0: /* Level */
        enable = (level == 0);
        break;
    case 1: /* Neg */
        enable = (intc->last_level[n_IRQ] == 1 && level == 0);
        break;
    case 2: /* Pos */
        enable = (intc->last_level[n_IRQ] == 0 && level == 1);
        break;
    case 3: /* Both */
        enable = (intc->last_level[n_IRQ] != level);
        break;
    }
    intc->last_level[n_IRQ] = level;
    if (ext_no(n_IRQ) >= 0) {
        enable = enable && extract16(intc->ier, ext_no(n_IRQ), 1);
    }
    if (enable) {
        set_pending(intc, n_IRQ, 1);
        if (atomic_read(&intc->req_irq) < 0) {
            atomic_set(&intc->req_irq, n_IRQ);
            qemu_set_irq(intc->irq, (pri(intc, n_IRQ) << 8) | n_IRQ);
        }
    } else {
        if (trigger_mode(intc, n_IRQ) == 0) {
            set_pending(intc, n_IRQ, 0);
            if (atomic_read(&intc->req_irq) == n_IRQ) {
                atomic_set(&intc->req_irq, -1);
                qemu_set_irq(intc->irq, 0);
            }
        }
    }
}

static void h8sintc_ack_irq(void *opaque, int no, int level)
{
    H8SINTCState *intc = opaque;
    int i;
    int base;
    int n_IRQ;
    int max_pri;
    int ext;

    n_IRQ = atomic_read(&intc->req_irq);
    if (n_IRQ < 0) {
        return;
    }
    atomic_set(&intc->req_irq, -1);
    if (level == 0) {
        return;
    }
    set_pending(intc, n_IRQ, 0);
    ext = ext_no(n_IRQ);
    if (ext >= 0 && (extract32(intc->iscr, ext * 2, 2) > 0)) {
        intc->isr = deposit16(intc->isr, ext, 1, 0);
    }
        
    max_pri = 0;
    n_IRQ = -1;
    base = i = 0;
    do {
        i =  find_next_bit(&intc->req[base / 64], 64, i);
        if (i < 64) {
            if (pri(intc, base + i) > max_pri) {
                n_IRQ = base + i;
                max_pri = pri(intc, base + i);
            }
            i++;
        } else {
            base += 64;
            i = 0;
        }
    } while (base < NR_IRQS);

    if (n_IRQ >= 0) {
        atomic_set(&intc->req_irq, n_IRQ);
        qemu_set_irq(intc->irq, (pri(intc, n_IRQ) << 8) | n_IRQ);
    }
}

static void clear_pend_irq(H8SINTCState *intc)
{
    int i;
    for (i = 0; i < 16; i++) {
        if (extract16(intc->isr, i, 1) == 0 &&
            extract64(intc->req[0], i + 16, 1)) {
            intc->req[0] = deposit64(intc->req[0], i + 16, 1, 0);
            if (atomic_read(&intc->req_irq) == i + 16) {
                h8sintc_ack_irq(intc, i + 16, 0);
            }
        }
    }
}

static uint64_t intc_ipr_read(void *opaque, hwaddr addr, unsigned size)
{
    H8SINTCState *intc = opaque;
    switch(addr) {
    case A_IPRA ... A_IPRK:
        return intc->ipr[addr >> 1];
    case A_ITSR:
        return intc->itsr;
    case A_SSIER:
        return intc->ssier;
    case A_ISCRH:
        return extract32(intc->iscr, 16, 16);
    case A_ISCRL:
        return extract32(intc->iscr, 0, 16);
    default:
        qemu_log_mask(LOG_UNIMP, "h8s_intc: Register 0x%" HWADDR_PRIX
                      " not implemented\n", addr);
        break;
    }
    return UINT64_MAX;
}

static void intc_ipr_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    H8SINTCState *intc = opaque;
    switch(addr) {
    case A_IPRA ... A_IPRK:
        intc->ipr[addr >> 1] = val;
        break;
    case A_ITSR:
        intc->itsr = val;
        break;
    case A_SSIER:
        intc->ssier = val;
        break;
    case A_ISCRH:
        intc->iscr = deposit32(intc->iscr, 16, 16, val);
        break;
    case A_ISCRL:
        intc->iscr = deposit32(intc->iscr, 0, 16, val);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "h8s_intc: Register 0x%" HWADDR_PRIX
                      " not implemented\n", addr);
        break;
    }
}

static const MemoryRegionOps intc_ipr_ops = {
    .write = intc_ipr_write,
    .read  = intc_ipr_read,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .max_access_size = 2,
    },
};

static uint64_t intc_isr_read(void *opaque, hwaddr addr, unsigned size)
{
    H8SINTCState *intc = opaque;
    switch(addr) {
    case A_INTCR:
        return intc->intcr;
    case A_IER:
        return intc->ier;
    case A_ISR:
        return intc->isr;
    default:
        qemu_log_mask(LOG_UNIMP, "h8s_intc: Register 0x%" HWADDR_PRIX
                      " not implemented\n", addr);
        break;
    }
    return UINT64_MAX;
}

static void intc_isr_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    H8SINTCState *intc = opaque;
    switch(addr) {
    case A_INTCR:
        intc->intcr = val;
        *(uint32_t *)(intc->im) = extract32(val, 4, 2);
        if (*(uint32_t *)(intc->im) > 2) {
            qemu_log_mask(LOG_GUEST_ERROR, "h8s_intc: Invalid INTM %d\n",
                          *(uint32_t *)(intc->im));
        }
        break;
    case A_IER:
        intc->ier = val;
        break;
    case A_ISR:
        intc->isr &= val;
        clear_pend_irq(intc);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "h8s_intc: Register 0x%" HWADDR_PRIX
                      " not implemented\n", addr);
        break;
    }
}

static const MemoryRegionOps intc_isr_ops = {
    .write = intc_isr_write,
    .read  = intc_isr_read,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 2,
    },
};

static void h8sintc_realize(DeviceState *dev, Error **errp)
{
    H8SINTCState *intc = H8SINTC(dev);

    intc->req_irq = -1;
}

static void h8sintc_init(Object *obj)
{
    SysBusDevice *d = SYS_BUS_DEVICE(obj);
    H8SINTCState *intc = H8SINTC(obj);

    memory_region_init_io(&intc->memory[0], OBJECT(intc), &intc_ipr_ops,
                          intc, "h8s-intc-pri", 0x1e);
    sysbus_init_mmio(d, &intc->memory[0]);
    memory_region_init_io(&intc->memory[1], OBJECT(intc), &intc_isr_ops,
                          intc, "h8s-intc-ir", 0x4);
    sysbus_init_mmio(d, &intc->memory[1]);

    qdev_init_gpio_in(DEVICE(d), h8sintc_set_irq, NR_IRQS);
    qdev_init_gpio_in_named(DEVICE(d), h8sintc_ack_irq, "ack", 1);
    sysbus_init_irq(d, &intc->irq);
}

static const VMStateDescription vmstate_h8sintc = {
    .name = "h8s-intc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static Property h8sintc_properties[] = {
    DEFINE_PROP_PTR("cpu-im", H8SINTCState, im),
    DEFINE_PROP_END_OF_LIST(),
};

static void h8sintc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->props = h8sintc_properties;
    dc->realize = h8sintc_realize;
    dc->vmsd = &vmstate_h8sintc;
}

static const TypeInfo h8sintc_info = {
    .name       = TYPE_H8SINTC,
    .parent     = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(H8SINTCState),
    .instance_init = h8sintc_init,
    .class_init = h8sintc_class_init,
};

static void h8sintc_register_types(void)
{
    type_register_static(&h8sintc_info);
}

type_init(h8sintc_register_types)
