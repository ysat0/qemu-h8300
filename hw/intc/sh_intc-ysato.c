/*
 * SH7750(S/R) and SH7751(R)  Interrupt Controller
 *
 * Copyright (c) 2020 Yoshinori Sato
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
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
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "hw/irq.h"
#include "hw/registerfields.h"
#include "hw/qdev-properties.h"
#include "hw/intc/sh_intc.h"
#include "migration/vmstate.h"
#include "cpu.h"

REG16(ICR, 0)
  FIELD(ICR, IRLM, 7, 1)
  FIELD(ICR, NMIE, 8, 1)
  FIELD(ICR, NMIB, 9, 1)
  FIELD(ICR, MAI,  14, 1)
  FIELD(ICR, NMIL, 15, 1)
REG16(IPRA, 4)
REG16(IPRB, 8)
REG16(IPRC, 12)
REG16(IPRD, 16)

REG32(INTPRI00, 0)
REG32(INTREQ00, 32)
REG32(INTMSK00, 64)
REG32(INTMSKCLR00, 96)

enum {
    IPRA, IPRB, IPRC, IPRD, INTPRI00,
};

struct irq_info {
    uint16_t evt;
    int inipri;
    int iprno;
    int iproffset;
};

#define IRQINFO(src, code, pri, pr, offset) \
    [src] = { \
        .evt = code, \
        .inipri = pri, \
        .iprno = pr, \
        .iproffset = offset, \
    },

struct irq_info irq_list[] = {
    IRQINFO(IRL0, 0x240, 13, IPRD, 12)
    IRQINFO(IRL1, 0x2a0, 10, IPRD, 8)
    IRQINFO(IRL2, 0x300,  7, IPRD, 4)
    IRQINFO(IRL3, 0x360,  4, IPRD, 0)
    IRQINFO(HUDI, 0x600,  0, IPRC, 0)
    IRQINFO(GPIOI, 0x620,  0, IPRC, 12)
    IRQINFO(DMTE0, 0x640, 0, IPRC, 8)
    IRQINFO(DMTE1, 0x660, 0, IPRC, 8)
    IRQINFO(DMTE2, 0x680, 0, IPRC, 8)
    IRQINFO(DMTE3, 0x6a0, 0, IPRC, 8)
    IRQINFO(DMTE4, 0x780, 0, IPRC, 8)
    IRQINFO(DMTE5, 0x7a0, 0, IPRC, 8)
    IRQINFO(DMTE6, 0x7c0, 0, IPRC, 8)
    IRQINFO(DMTE7, 0x7e0, 0, IPRC, 8)
    IRQINFO(DMAE,  0x6c0, 0, IPRC, 8)
    IRQINFO(PCISERR, 0xa00, 0, INTPRI00, 0)
    IRQINFO(PCIERR, 0xae0, 0, INTPRI00, 4)
    IRQINFO(PCIPWDWN, 0xac0, 0, INTPRI00, 4)
    IRQINFO(PCIPWON, 0xaa0, 0, INTPRI00, 4)
    IRQINFO(PCIDMA0, 0xa80, 0, INTPRI00, 4)
    IRQINFO(PCIDMA1, 0xa60, 0, INTPRI00, 0)
    IRQINFO(PCIDMA2, 0xa40, 0, INTPRI00, 0)
    IRQINFO(PCIDMA3, 0xa20, 0, INTPRI00, 0)
    IRQINFO(TUNI3, 0xb00, 0, INTPRI00, 8)
    IRQINFO(TUNI4, 0xb80, 0, INTPRI00, 8)
    IRQINFO(TUNI0, 0x400, 0, IPRA, 12)
    IRQINFO(TUNI1, 0x420, 0, IPRA, 8)
    IRQINFO(TUNI2, 0x440, 0, IPRA, 4)
    IRQINFO(TICPI2, 0x460, 0, IPRA, 4)
    IRQINFO(RTC_ATI, 0x480, 0, IPRA, 0)
    IRQINFO(RTC_PRI, 0x4a0, 0, IPRA, 0)
    IRQINFO(RTC_CUI, 0x4c0, 0, IPRA, 0)
    IRQINFO(SCI_ERI, 0x4e0, 0, IPRB, 4)
    IRQINFO(SCI_RXI, 0x500, 0, IPRB, 4)
    IRQINFO(SCI_TXI, 0x520, 0, IPRB, 4)
    IRQINFO(SCI_TEI, 0x540, 0, IPRB, 4)
    IRQINFO(SCIF_ERI, 0x700, 0, IPRC, 4)
    IRQINFO(SCIF_RXI, 0x720, 0, IPRC, 4)
    IRQINFO(SCIF_BRI, 0x740, 0, IPRC, 4)
    IRQINFO(SCIF_TXI, 0x760, 0, IPRC, 4)
    IRQINFO(WDT_ITI, 0x560, 0, IPRB, 12)
    IRQINFO(REF_RCMI, 0x580, 0, IPRB, 8)
    IRQINFO(REF_ROVI, 0x5c0, 0, IPRB, 8)
};
#undef IRQINFO

typedef struct {
    SysBusDeviceClass parent;

    bool have_iprd;
    bool have_extint;
} SH4INTCClass;

#define SH4INTC_CLASS(klass)                                    \
    OBJECT_CLASS_CHECK(SH4INTCClass, (klass), TYPE_SH4_INTC)
#define SH4INTC_GET_CLASS(obj)                                  \
    OBJECT_GET_CLASS(SH4INTCClass, (obj), TYPE_SH4_INTC)

static bool is_single_request(SH4INTCState *s)
{
    return (s->req_src >= 0) && (s->req_src < NR_SINGLE_SOURCE);
}

static void clear_req(SH4INTCState *s)
{
    s->req_src = -1;
    s->req_pri = 0;
    s->pending_irl = 15;
}

static int int_priority(SH4INTCState *s, struct irq_info *info)
{
    int ipr = info->iprno;
    int offset = info->iproffset;
    if (ipr < INTPRI00) {
        return extract16(s->ipr[ipr], offset, 4);
    } else {
        return extract16(s->intpri00, offset, 4);
    }
}

static void request_pending_irq(SH4INTCState *s)
{
    int req = find_first_bit(&s->pend_map, 64);
    struct irq_info *info;
    int max_pri = 0;
    int priority;

    if (s->pending_irl < 15) {
        s->req_src = NR_SINGLE_SOURCE + s->pending_irl;
        max_pri = 15 - s->pending_irl;
    }
    while (req < ARRAY_SIZE(irq_list)) {
        info = &irq_list[req];
        priority = int_priority(s, info);
        if (priority > max_pri) {
            max_pri = priority;
            s->req_src = NR_SINGLE_SOURCE - req;
        }
        req = find_next_bit(&s->pend_map, 64, req + 1);
    }
    if (max_pri > 0) {
        s->req_pri = max_pri;
#if 0
        if (is_single_request(s)) {
            fprintf(stderr,"%s 0x%04x %d\n", __func__, irq_list[s->req_src].evt, s->req_pri);
        } else {
            fprintf(stderr,"%s 0x%04x %d\n", __func__, 0x200 + (s->req_src - NR_SINGLE_SOURCE) * 0x20 , s->req_pri);
        }
#endif
        cpu_interrupt(first_cpu, CPU_INTERRUPT_HARD);
    }
}

static void pending_current_request(SH4INTCState *s)
{
    if (is_single_request(s)) {
        s->pend_map = deposit64(s->pend_map,
                                NR_SINGLE_SOURCE - s->req_src, 1, 1);
    } else if (s->req_src >= NR_SINGLE_SOURCE) {
        s->pending_irl = s->req_src - NR_SINGLE_SOURCE;
    }
}

static void sh4intc_set_irq(void *opaque, int n_IRQ, int level)
{
    SH4INTCState *s = SH4_INTC(opaque);
    struct irq_info *info = &irq_list[n_IRQ];
    int priority = int_priority(s, info);
#if 0    
    fprintf(stderr, "1 %s %04x %d %d %d\n", __func__, info->evt, level, s->req_pri, priority);
#endif
    if (FIELD_EX16(s->icr, ICR, IRLM) == 0 &&
        n_IRQ <= IRL3) {
        /* IRL input is level. */
        return;
    }
    if (level) {
        if (s->req_pri < priority) {
            pending_current_request(s);
            s->req_pri = priority;
            s->req_src = n_IRQ;
#if 0
            fprintf(stderr, "2 %s %04x %d %d\n", __func__, info->evt, level, s->req_pri);
#endif
            cpu_interrupt(first_cpu, CPU_INTERRUPT_HARD);
        } else {
            s->pend_map = deposit64(s->pend_map,
                                    NR_SINGLE_SOURCE - n_IRQ, 1, 1);
        }
    } else {
        if (s->req_src == n_IRQ) {
#if 0
            fprintf(stderr, "%s %04x %d %d\n", __func__, info->evt, level, s->req_pri);
#endif
            if (s->pend_map || s->pending_irl < 15) {
                request_pending_irq(s);
            } else {
                clear_req(s);
                cpu_reset_interrupt(first_cpu, CPU_INTERRUPT_HARD);
            }
        } else {
            s->pend_map = deposit64(s->pend_map,
                                       NR_SINGLE_SOURCE - n_IRQ, 1, 0);
        }
    }
}

static void sh4intc_set_irl(void *opaque, int n_IRQ, int level)
{
    SH4INTCState *s = SH4_INTC(opaque);
    int priority = 15 - level;
 
    if (level < 15) {
        if (s->req_pri <= priority) {
            pending_current_request(s);
            s->req_src = NR_SINGLE_SOURCE + level;
            s->req_pri = priority;
            cpu_interrupt(first_cpu, CPU_INTERRUPT_HARD);
        } else {
            s->pending_irl = level;
        }
    } else {
        request_pending_irq(s);
    }
}

static uint64_t sh4intc_read(void *opaque, hwaddr addr, unsigned size)
{
    SH4INTCState *s = SH4_INTC(opaque);
    SH4INTCClass *ic = SH4INTC_GET_CLASS(s);
    int ipr;

    switch (addr) {
    case A_ICR:
        return s->icr;
    case A_IPRA:
    case A_IPRB:
    case A_IPRC:
    case A_IPRD:
        ipr = (addr - A_IPRA) / 4;
        if (ipr < 3 && ic->have_iprd) {
            return s->ipr[ipr];
        }
        /* Fail through */
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "sh_intc: Invalid address 0x%"
                      HWADDR_PRIX ".\n", addr);
        return UINT64_MAX;
    }
}

static void sh4intc_write(void *opaque, hwaddr addr,
                              uint64_t val, unsigned size)
{
    SH4INTCState *s = SH4_INTC(opaque);
    SH4INTCClass *ic = SH4INTC_GET_CLASS(s);
    int ipr;

    switch (addr) {
    case A_ICR:
        s->icr = FIELD_DP16(val, ICR, NMIL, 0);  /* NMIL is always low */
        if (FIELD_EX16(val, ICR, MAI) || FIELD_EX16(val, ICR, NMIB) ||
            FIELD_EX16(val, ICR, NMIE)) {
            qemu_log_mask(LOG_UNIMP, "sh_intc: NMI not supported.\n");
        }  
        break;
    case A_IPRA:
    case A_IPRB:
    case A_IPRC:
    case A_IPRD:
        ipr = (addr - A_IPRA) / 4;
        if (ipr < 3 || ic->have_iprd) {
            s->ipr[ipr] = val;
            request_pending_irq(s);
            break;
        }
        /* Fail through */
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "sh_intc: Invalid address 0x%"
                      HWADDR_PRIX ".\n", addr);
    }
}

static uint64_t sh4intc_ex_read(void *opaque, hwaddr addr, unsigned size)
{
    SH4INTCState *s = SH4_INTC(opaque);
    SH4INTCClass *ic = SH4INTC_GET_CLASS(s);

    if (!ic->have_extint) {
        g_assert_not_reached();
    }
    switch (addr) {
    case A_INTPRI00:
        return s->intpri00;
    case A_INTREQ00:
        return s->intreq00;
    case A_INTMSK00:
        return s->intmsk00;
    case A_INTMSKCLR00:
        qemu_log_mask(LOG_GUEST_ERROR, "sh_intc: INTMSK00 is write only.\n");
        return UINT64_MAX;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "sh_intc: Invalid address 0x%"
                      HWADDR_PRIX ".\n", addr);
        return UINT64_MAX;
    }
}

static void sh4intc_ex_write(void *opaque, hwaddr addr,
                                 uint64_t val, unsigned size)
{
    SH4INTCState *s = SH4_INTC(opaque);
    SH4INTCClass *ic = SH4INTC_GET_CLASS(s);

    if (!ic->have_extint) {
        g_assert_not_reached();
    }
    switch (addr) {
    case A_INTPRI00:
        s->intpri00 = val;
        request_pending_irq(s);
        break;
    case A_INTREQ00:
        qemu_log_mask(LOG_GUEST_ERROR, "sh_intc: INTREQ00 is read only.\n");
        break;
    case A_INTMSK00:
        s->intmsk00 |= val;
        break;
    case A_INTMSKCLR00:
        s->intmsk00 &= ~val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "sh_intc: Invalid address 0x%"
                      HWADDR_PRIX ".\n", addr);
    }
}

static const MemoryRegionOps sh4intc_ops = {
    .read = sh4intc_read,
    .write = sh4intc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 2,
        .max_access_size = 2,
    },
};

static const MemoryRegionOps sh4intc_ex_ops = {
    .read = sh4intc_ex_read,
    .write = sh4intc_ex_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

int sh_intc_get_pending_vector(SH4INTCState *state, uint32_t imask)
{
    SH4INTCState *s = SH4_INTC(state);
    int irl;
    int ret;

    if (imask != 0x0f) {
        if (is_single_request(s)) {
            ret =  irq_list[s->req_src].evt;
        } else {
            irl = s->req_src - NR_SINGLE_SOURCE;
            ret = 0x200 + irl * 0x20;
        }
    } else {
        ret = -1;
    }
    return ret;
}

static void sh4_intc_realize(DeviceState *dev, Error **errp)
{
    SH4INTCState *s = SH4_INTC(dev);
    struct irq_info *info = irq_list;
    int i;

    s->intmsk00 = 0x03ff;
    for (i = 0; i < NR_SINGLE_SOURCE; info++, i++) {
        s->ipr[info->iprno] = deposit32(s->ipr[info->iprno],
                                        info->iproffset, 4, info->inipri);
    }
    clear_req(s);
}

static void sh4intc_init(SH4INTCState *s)
{
    s->irqs = qemu_allocate_irqs(sh4intc_set_irq, s, NR_SINGLE_SOURCE);
    s->irl = qemu_allocate_irq(sh4intc_set_irl, s, 0);
}

static void sh7750_init(Object *obj)
{
    SH4INTCState *s = SH4_INTC(obj);

    memory_region_init_io(&s->memory_p4, OBJECT(s), &sh4intc_ops, s,
                          "sh7750-intc-p4", 0x10);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->memory_p4);
    memory_region_init_alias(&s->memory_a7, OBJECT(s),
                             "sh7750-intc-a7", &s->memory_p4, 0, 0x10);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->memory_a7);
    sh4intc_init(s);
}

static void sh7750s_init(Object *obj)
{
    SH4INTCState *s = SH4_INTC(obj);

    memory_region_init_io(&s->memory_p4, OBJECT(s), &sh4intc_ops, s,
                          "sh7750s-intc-p4", 0x14);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->memory_p4);
    memory_region_init_alias(&s->memory_a7, OBJECT(s),
                             "sh7750s-intc-a7", &s->memory_p4, 0, 0x14);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->memory_a7);
    sh4intc_init(s);
}

static void sh7750r_init(Object *obj)
{
    SH4INTCState *s = SH4_INTC(obj);

    memory_region_init_io(&s->memory_p4, OBJECT(s), &sh4intc_ops, s,
                          "sh7750-intc-p4", 0x14);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->memory_p4);
    memory_region_init_alias(&s->memory_a7, OBJECT(s),
                             "sh7750-intc-a7", &s->memory_p4, 0, 0x14);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->memory_a7);
    memory_region_init_io(&s->memex_p4, OBJECT(s), &sh4intc_ex_ops, s,
                          "sh7750-intc-ex-p4", 0x80);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->memex_p4);
    memory_region_init_alias(&s->memex_a7, OBJECT(s),
                             "sh7750-intc-ex-a7", &s->memory_p4, 0, 0x80);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->memex_a7);
    sh4intc_init(s);
}

static void sh4_intc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sh4_intc_realize;
}

static void sh7750_class_init(ObjectClass *klass, void *data)
{
    SH4INTCClass *ic = SH4INTC_CLASS(klass);
    ic->have_iprd = false;
    ic->have_extint = false;
}

static void sh7750s_class_init(ObjectClass *klass, void *data)
{
    SH4INTCClass *ic = SH4INTC_CLASS(klass);
    ic->have_iprd = true;
    ic->have_extint = false;
}

static void sh7750r_class_init(ObjectClass *klass, void *data)
{
    SH4INTCClass *ic = SH4INTC_CLASS(klass);
    ic->have_iprd = true;
    ic->have_extint = true;
}

static const TypeInfo sh_intc_types[] = {
    {
        .name           = TYPE_SH7750_INTC,
        .parent         = TYPE_SH4_INTC,
        .class_init     = sh7750_class_init,
        .instance_init  = sh7750_init,
    }, {
        .name           = TYPE_SH7750S_INTC,
        .parent         = TYPE_SH4_INTC,
        .class_init     = sh7750s_class_init,
        .instance_init  = sh7750s_init,
    }, {
        .name           = TYPE_SH7750R_INTC,
        .parent         = TYPE_SH4_INTC,
        .class_init     = sh7750r_class_init,
        .instance_init  = sh7750r_init,
    }, {
        .name           = TYPE_SH4_INTC,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(SH4INTCState),
        .class_size     = sizeof(SH4INTCClass),
        .class_init     = sh4_intc_class_init,
        .abstract       = true,
     }
};

DEFINE_TYPES(sh_intc_types)
