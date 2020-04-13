/*
 * Renesas 16bit Timer
 *
 * Copyright (c) 2020 Yoshinori Sato
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
#include "qemu-common.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "cpu.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/timer/renesas_16timer.h"
#include "qemu/error-report.h"

REG8(TSTR, 0)
    FIELD(TSTR, STR0, 0, 3)
REG8(TSNC, 1)
    FIELD(TSNC, SYNC, 0, 3)
REG8(TMDR, 2)
    FIELD(TMDR, PWM, 0, 3)
    FIELD(TMDR, FDIR, 5, 1)
    FIELD(TMDR, MDF, 6, 1)
REG8(TOLR, 3)
    FIELD(TOLR, TOA0, 0, 1)
    FIELD(TOLR, TOB0, 1, 1)
    FIELD(TOLR, TOA1, 2, 1)
    FIELD(TOLR, TOB1, 3, 1)
    FIELD(TOLR, TOA2, 4, 1)
    FIELD(TOLR, TOB2, 5, 1)
REG8(TISR, 4)
    FIELD(TISR, IMF, 0, 3)
    FIELD(TISR, IMIE, 4, 3)
REG8(TCR, 0)
    FIELD(TCR, TPSC, 0, 3)
    FIELD(TCR, CKEG, 3, 2)
    FIELD(TCR, CCLR, 5, 2)
REG8(TIOR, 1)
    FIELD(TIOR, IOA, 0, 3)
    FIELD(TIOR, IOB, 4, 3)
REG16(TCNT, 2)
REG16(GRA, 4)
REG16(GRB, 6)

static inline int clr_gr(R16State *s, int ch)
{
    int gr = -1;
    if (((FIELD_EX8(s->ch[ch].tcr, TCR, CCLR) & 3) > 0) &&
        ((FIELD_EX8(s->ch[ch].tcr, TCR, CCLR) & 3) < 3)) {
        gr = (FIELD_EX8(s->ch[ch].tcr, TCR, CCLR) & 3) -1;
    }
    return gr;
}

static void set_next_event(R16State *s)
{
    int ch;
    int t;
    uint32_t event_count;
    int gr;
    int64_t next_time = INT64_MAX;
    
    for (ch = 0; ch < 3; ch++) {
        if (extract32(s->tstr, ch, 1) == 0) {
            continue;
        }
        t = 1 << FIELD_EX8(s->ch[ch].tcr, TCR, TPSC);
        if (t >= 1 << 4) {
            continue;
        }
        gr = clr_gr(s, ch);
        if (gr == -1) {
            event_count = 0x10000;
        } else {
            event_count = s->ch[ch].gr[gr] + 1;
        }
        t *= (event_count - s->ch[ch].tcnt);
        next_time = MIN(next_time, t);
    }
    if (next_time < INT64_MAX) {
        next_time *= s->clk_per_nsec;
        next_time += qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        timer_mod(s->timer, next_time);
    }
}

static uint16_t read_tcnt(R16State *s, int ch)
{
    int64_t elapsed;
    int div;

    div = 1 << FIELD_EX8(s->ch[ch].tcr, TCR, TPSC);
    if (extract32(s->tstr, ch, 1) == 0 || div >= 1 << 4) {
        elapsed = 0;
    } else {
        elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->prev_time;
        elapsed /= s->clk_per_nsec;
        elapsed /= div;
    }
    return s->ch[ch].tcnt + elapsed;
}

static void update_tcnt(R16State *s)
{
    int64_t elapsed;
    int div;
    int ch;
    int64_t now;
    int gr;
    int sr;
    uint8_t ir;

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    elapsed = now - s->prev_time;
    s->prev_time = now;
    elapsed /= s->clk_per_nsec;

    for (ch = 2; ch >= 0; ch--) {
        div = 1 << FIELD_EX8(s->ch[ch].tcr, TCR, TPSC);
        if (extract32(s->tstr, ch, 1) == 0 || div >= 1 << 4) {
            continue;
        }

        s->ch[ch].tcnt += elapsed / div;
        
        if (s->ch[ch].tcnt >= 0x10000) {
            s->ch[ch].tcnt -= 0x10000;
            s->tisr[2] = deposit8(s->tisr[2], ch, 1, 1);
        }

        for (gr = 0; gr < 1; gr++) {
            if (s->ch[ch].tcnt >= s->ch[ch].gr[gr]) {
                s->tisr[gr] = deposit8(s->tisr[gr], ch, 1, 1);
            }
        }
                
        gr = clr_gr(s, ch);
        if (gr >= 0 && s->ch[ch].tcnt >= s->ch[ch].gr[gr]) {
            s->ch[ch].tcnt -= s->ch[ch].gr[gr];
        }
    }
    for (sr = 0; sr < 3; sr++) {
        ir = FIELD_EX8(s->tisr[sr], TISR, IMF) &
            FIELD_EX8(s->tisr[sr], TISR, IMIE);
        if (ir == 0) {
            continue;
        }
        for (ch = 0; ch < 3; ch++) {
            if (extract32(ir, ch, 1)) {
                qemu_set_irq(s->irq[ch * 4 + sr], 1);
            }
        }
    }
}

static void timer_event(void *opaqueue)
{
    R16State *s = R16(opaqueue);
    update_tcnt(s);
    set_next_event(s);
}

static uint64_t read_channel(void *opaque, int ch, hwaddr addr, unsigned size)
{
    R16State *tmr16 = opaque;
    switch (addr) {
    case A_TCR:
        return tmr16->ch[ch].tcr | 0x80;
    case A_TIOR:
        return tmr16->ch[ch].tior | 0x88;
    case A_TCNT:
        return read_tcnt(tmr16, ch);
    case A_GRA:
    case A_GRB:
        return tmr16->ch[ch].gr[addr - A_GRA];
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "renesas_tmr16: Unknown register %08lx\n",
                      addr);
        return -1;
    }
}

static void write_channel(void *opaque, int ch, hwaddr addr,
                          uint64_t val, unsigned size)
{
    R16State *tmr16 = opaque;

    switch (addr) {
    case A_TCR:
        if (extract32(tmr16->tstr, ch, 1)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "renesas_16timer: Timer now running.\n");
        }
        tmr16->ch[ch].tcr = val;
        set_next_event(tmr16);
        break;
    case A_TIOR:
        tmr16->ch[ch].tior = val;
        if (FIELD_EX8(tmr16->ch[ch].tior, TIOR, IOA) != 0) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "renesas_16timer: Support only normal operation.\n");
        }
        break;
    case A_TCNT:
        tmr16->ch[ch].tcnt = val;
        set_next_event(tmr16);
        break;
    case A_GRA:
    case A_GRB:
        tmr16->ch[ch].gr[addr - A_GRA] = val;
        set_next_event(tmr16);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "renesas_tmr16: Unknown register %08lx\n",
                      addr);
    }
}

static uint64_t read_common(void *opaque, hwaddr addr, unsigned size)
{
    R16State *tmr16 = opaque;

    switch(addr) {
    case A_TSTR:
        return tmr16->tstr | 0xf8;
    case A_TSNC:
        return tmr16->tsnc | 0xf8;
    case A_TMDR:
        return tmr16->tmdr | 0x98;
    case A_TOLR:
        return tmr16->tolr | 0xc0;
    case A_TISR:
    case A_TISR + 1:
    case A_TISR + 2:
        return tmr16->tisr[addr - A_TISR] | 0x88;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "renesas_tmr16: Unknown register %08lx\n",
                      addr);
        return 0;
    }
}

static void write_common(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    R16State *tmr16 = opaque;
    uint8_t isr;
    uint8_t ier;
    int reg;

    switch(addr) {
    case A_TSTR:
        tmr16->tstr = val;
        set_next_event(tmr16);
        break;
    case A_TSNC:
        tmr16->tsnc = val;
        if (tmr16->tsnc) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "renesas_tmr16: Sync mode not support.\n");
        }
        break;
    case A_TMDR:
        tmr16->tmdr = val;
        if (tmr16->tmdr) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "renesas_tmr16: Supported only normal operation.\n");
        }
        break;
    case A_TOLR:
        tmr16->tolr = val;
        qemu_log_mask(LOG_GUEST_ERROR,
                      "renesas_tmr16: TOLR not supported.\n");
        break;
    case A_TISR:
    case A_TISR + 1:
    case A_TISR + 2:
        reg = addr - A_TISR;
        isr = FIELD_EX8(val, TISR, IMF);
        ier = FIELD_EX8(val, TISR, IMIE);
        tmr16->tisr[reg] = FIELD_DP8(tmr16->tisr[reg], TISR, IMIE, ier);
        tmr16->tisr[reg] &= (0xf0 | isr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "renesas_tmr16: Unknown register %08lx\n",
                      addr);
    }
}

static uint64_t tmr16_read(void *opaque, hwaddr addr, unsigned size)
{
    int ch;
    if (addr < 8) {
        return read_common(opaque, addr & 7, size);
    } else {
        ch = (addr >> 3) - 1;
        return read_channel(opaque, ch, addr & 7, size);
    }
}

static void tmr16_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    int ch;
    if (addr < 8) {
        write_common(opaque, addr & 7, val, size);
    } else {
        ch = (addr >> 3) - 1;
        write_channel(opaque, ch, addr & 7, val, size);
    }
}

static const MemoryRegionOps tmr16_ops = {
    .write = tmr16_write,
    .read  = tmr16_read,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 2,
    },
};
static void r16_reset(DeviceState *dev)
{
    R16State *tmr16 = R16(dev);
    int i, j;
    for (i = 0; i < 3; i++) {
        memset(&tmr16->ch[i], 0, sizeof(tmr16->ch[i]));
        for (j = 0; j < 2; j++) {
            tmr16->ch[i].gr[j] = 0xffff;
        }
    }
}

static void r16_realize(DeviceState *dev, Error **errp)
{
    R16State *s = R16(dev);
    s->clk_per_nsec = NANOSECONDS_PER_SECOND / s->input_freq;
}

static void r16_init(Object *obj)
{
    SysBusDevice *d = SYS_BUS_DEVICE(obj);
    R16State *tmr16 = R16(obj);
    int i;

    memory_region_init_io(&tmr16->memory, OBJECT(tmr16), &tmr16_ops,
                          tmr16, "renesas-16timer", 0x20);
    sysbus_init_mmio(d, &tmr16->memory);

    for (i = 0; i < ARRAY_SIZE(tmr16->irq); i++) {
        sysbus_init_irq(d, &tmr16->irq[i]);
    }
    
    tmr16->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, timer_event, tmr16);
}

static const VMStateDescription vmstate_r16 = {
    .name = "renesas-16timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static Property r16_properties[] = {
    DEFINE_PROP_UINT64("input-freq", R16State, input_freq, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void r16_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->props = r16_properties;
    dc->vmsd = &vmstate_r16;
    dc->reset = r16_reset;
    dc->realize = r16_realize;
}

static const TypeInfo r16_info = {
    .name       = TYPE_RENESAS_16TMR,
    .parent     = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(R16State),
    .instance_init = r16_init,
    .class_init = r16_class_init,
};

static void r16_register_types(void)
{
    type_register_static(&r16_info);
}

type_init(r16_register_types)
