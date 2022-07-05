/*
 * Renesas Timer Pulse Unit
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
#include "qemu-common.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "hw/hw.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/qdev-properties.h"
#include "hw/timer/renesas_tpu.h"
#include "qemu/error-report.h"
#include "migration/vmstate.h"

REG8(TCR, 0)
FIELD(TCR, TPSC, 0, 3)
FIELD(TCR, CKEG, 3, 2)
FIELD(TCR, CCLR, 5, 3)
FIELD(TCR, CCLR2, 7, 1)
REG8(TMDR, 1)
FIELD(TMDR, MD, 0, 4)
FIELD(TMDR, BFA, 4, 1)
FIELD(TMDR, BFB, 5, 1)
REG16(TIOR, 2)
FIELD(TIOR, IOC,  0, 4)
FIELD(TIOR, IOD,  4, 4)
FIELD(TIOR, IOA,  8, 4)
FIELD(TIOR, IOB, 12, 4)
REG8(TIER, 4)
FIELD(TIER, TGIEA, 0, 1)
FIELD(TIER, TGIEB, 1, 1)
FIELD(TIER, TGIEC, 2, 1)
FIELD(TIER, TGIED, 3, 1)
FIELD(TIER, TGIE,  0, 4)
FIELD(TIER, TCIEV, 4, 1)
FIELD(TIER, TCIEU, 5, 1)
FIELD(TIER, TTGE,  7, 1)
REG8(TSR, 5)
FIELD(TSR, TGFA, 0, 1)
FIELD(TSR, TGFB, 1, 1)
FIELD(TSR, TGFC, 2, 1)
FIELD(TSR, TGFD, 3, 1)
FIELD(TSR, TGF,  0, 4)
FIELD(TSR, TCFV, 4, 1)
FIELD(TSR, TCFU, 5, 1)
FIELD(TSR, TCFD, 7, 1)
REG16(TCNT, 6)
REG16(TGRA, 8)
REG16(TGRB, 10)
REG16(TGRC, 12)
REG16(TGRD, 14)

static const int div_rate[6][8] = {
    [0] = {1, 4, 16, 64, 0, 0, 0, 0, },
    [1] = {1, 4, 16, 64, 0, 0, 256, -1, },
    [2] = {1, 4, 16, 64, 0, 0, 0, 1024, },
    [3] = {1, 4, 16, 64, 256, 1024, 0, 0, },
    [4] = {1, 4, 16, 64, 256, 1024, 0, -1, },
    [5] = {1, 4, 16, 64, 0, 0, 0, 0, },
};

static const int irq_map[6][5] = {
    [0] = {0, 1, 2, 3, 4, },
    [1] = {5, 6, 7, 8, -1 },
    [2] = {9, 10, 11, 12, -1 },
    [3] = {13, 14, 15, 16, 17 },
    [4] = {18, 19, 20, 21, -1 },
    [5] = {22, 23, 24, 25, -1 },
};

static inline int clr_gr(RTPUState *s, int ch)
{
    int gr = -1;
    if (((FIELD_EX8(s->ch[ch].tcr, TCR, CCLR) & 3) > 0) &&
        ((FIELD_EX8(s->ch[ch].tcr, TCR, CCLR) & 3) < 3)) {
        gr = (FIELD_EX8(s->ch[ch].tcr, TCR, CCLR) & 3) -1;
        if (ch == 0 || ch == 3) {
            gr = deposit32(gr, 2, 1, FIELD_EX8(s->ch[ch].tcr, TCR, CCLR2));
        }
    }
    return gr;
}

static void set_next_event(RTPUState *s)
{
    int ch;
    int t;
    uint32_t event_count;
    int gr;
    int64_t next_time = INT64_MAX;
    
    for (ch = 0; ch < 6; ch++) {
        if (extract32(s->tstr, ch, 1) == 0) {
            continue;
        }
        t = div_rate[ch][FIELD_EX8(s->ch[ch].tcr, TCR, TPSC)];
        if (t == 0) {
            continue;
        }
        gr = clr_gr(s, ch);
        if (gr == -1) {
            event_count = 0x10000;
        } else {
            event_count = s->ch[ch].tgr[gr] + 1;
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

static uint16_t read_tcnt(RTPUState *s, int ch)
{
    int64_t elapsed;
    int div;

    div = div_rate[ch][FIELD_EX8(s->ch[ch].tcr, TCR, TPSC)];
    if (extract32(s->tstr, ch, 1) == 0 || div <= 0) {
        elapsed = 0;
    } else {
        elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->prev_time;
        elapsed /= s->clk_per_nsec;
        elapsed /= div;
    }
    return s->ch[ch].tcnt + elapsed;
}

static void update_tcnt(RTPUState *s)
{
    int64_t elapsed;
    int div;
    int ch;
    int64_t now;
    uint32_t ir[6];
    int gr;
    int ib;

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    elapsed = now - s->prev_time;
    s->prev_time = now;
    elapsed /= s->clk_per_nsec;

    for (ch = 5; ch >= 0; ch--) {
        ir[ch] = 0;
        div = div_rate[ch][FIELD_EX8(s->ch[ch].tcr, TCR, TPSC)];
        if (extract32(s->tstr, ch, 1) == 0 || div == 0) {
            continue;
        }

        if (div > 0) {
            s->ch[ch].tcnt += elapsed / div;
        } else {
            s->ch[ch].tcnt += FIELD_EX32(ir[ch], TSR, TCFV);
        }
        
        if (s->ch[ch].tcnt >= 0x10000) {
            s->ch[ch].tcnt -= 0x10000;
            s->ch[ch].tsr = FIELD_DP8(s->ch[ch].tsr, TSR, TCFV, 1);
            ir[ch] = FIELD_DP32(ir[ch], TSR, TCFV, 1); 
        }

        for (gr = 0; gr < 4; gr++) {
            if (gr >= 2 && (ch != 0 && ch !=3)) {
                break;
            }
            if (s->ch[ch].tcnt >= s->ch[ch].tgr[gr]) {
                s->ch[ch].tsr = deposit8(s->ch[ch].tsr, gr, 1, 1);
                ir[ch] = deposit32(ir[ch], gr, 1, 1);
            }
        }
                
        gr = clr_gr(s, ch);
        if (gr >= 0 && s->ch[ch].tcnt >= s->ch[ch].tgr[gr]) {
            s->ch[ch].tcnt -= s->ch[ch].tgr[gr];
        }
    }

    for(ch = 0; ch < 6; ch++) {
        if (ir[ch]) {
            ir[ch] &= s->ch[ch].tier;
            for (ib = 0; ib < 5; ib++) {
                if (extract32(ir[ch], ib, 1)) {
                    qemu_set_irq(s->irq[irq_map[ch][ib]], 1);
                }
            }
        }
    }
}

static void timer_event(void *opaqueue)
{
    RTPUState *s = RTPU(opaqueue);
    update_tcnt(s);
    set_next_event(s);
}

static uint64_t tpu_ch_read(void *opaque, int ch, hwaddr addr, unsigned size)
{
    RTPUState *tpu = opaque;
    uint8_t ch_mask = 0xf3;
    uint8_t tmdr_mask = 0xc7;

    if (ch == 0 || ch == 3) {
        ch_mask = 0xff;
        tmdr_mask = 0xf7;
    }
    if ((size == 2 && (addr < A_TCNT && addr != A_TIOR)) ||
        ((size == 1) && (addr >= A_TCNT))) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "renesas_tpu: Invalid accress size %08lx\n",
                      addr);
        return 0;
    }        
    switch (addr) {
    case A_TCR:
        return tpu->ch[ch].tcr;
    case A_TMDR:
        return (tpu->ch[ch].tmdr | 0xc0) & tmdr_mask;
    case A_TIOR:
        if (size == 2) {
            return tpu->ch[ch].tior;
        } else {
            return extract32(tpu->ch[ch].tior, 8, 8);
        }
    case A_TIOR + 1:
        return extract32(tpu->ch[ch].tior, 0, 8);
    case A_TIER:
        return (tpu->ch[ch].tier & ch_mask) | 0x40;
    case A_TSR:
        return (tpu->ch[ch].tsr & ch_mask) | 0x40;
    case A_TCNT:
        return read_tcnt(tpu, ch);
    case A_TGRA:
    case A_TGRB:
        return tpu->ch[ch].tgr[addr - A_TGRA];
    case A_TGRC:
    case A_TGRD:
        if (ch == 0 || ch == 3) {
            return tpu->ch[ch].tgr[addr - A_TGRA];
        } else {
            return 0;
        }
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "renesas_tpu: Unknown register %08lx\n",
                      addr);
        return -1;
    }
}

static void tpu_ch_write(void *opaque, int ch, hwaddr addr,
                         uint64_t val, unsigned size)
{
    RTPUState *tpu = opaque;
    uint8_t old_tsr;
    int i;

    if ((size == 2 && (addr < A_TCNT && addr != A_TIOR)) ||
        ((size == 1) && (addr >= A_TCNT))) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "renesas_tpu: Invalid accress size %08lx\n",
                      addr);
        return ;
    }        
    switch (addr) {
    case A_TCR:
        if (extract32(tpu->tstr, ch, 1)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "renesas_tpu: Timer now running.\n");
        }
        tpu->ch[ch].tcr = val;
        set_next_event(tpu);
        break;
    case A_TMDR:
        tpu->ch[ch].tmdr = val;
        if (FIELD_EX8(tpu->ch[ch].tmdr, TMDR, MD) != 0) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "renesas_tpu: Support only normal operation.\n");
        }
        break;
    case A_TIOR:
        if (size == 2) {
            tpu->ch[ch].tior = val;
        } else {
            tpu->ch[ch].tior = deposit32(tpu->ch[ch].tior, 8, 8, val);
        }
        break;
    case A_TIOR + 1:
        tpu->ch[ch].tior = deposit32(tpu->ch[ch].tior, 0, 8, val);
        break;
    case A_TIER:
        tpu->ch[ch].tier = val;
        if (FIELD_EX8(tpu->ch[ch].tier, TIER, TTGE) != 0) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "renesas_tpu: A/D trigger not supported.\n");
        }
        break;
    case A_TSR:
        old_tsr = tpu->ch[ch].tsr;
        tpu->ch[ch].tsr &= (val & 0x3f);
        old_tsr ^= tpu->ch[ch].tsr;
        old_tsr &= tpu->ch[ch].tier;
        if (old_tsr) {
            for (i = 0; i < 5; i++) {
                if (extract32(old_tsr, i, 1)) {
                    qemu_set_irq(tpu->irq[irq_map[ch][i]], 0);
                }
            }
        }
        break;
    case A_TCNT:
        tpu->ch[ch].tcnt = val;
        set_next_event(tpu);
        break;
    case A_TGRA:
    case A_TGRB:
        tpu->ch[ch].tgr[addr - A_TGRA] = val;
        set_next_event(tpu);
        break;
    case A_TGRC:
    case A_TGRD:
        if (ch == 0 || ch == 3) {
            tpu->ch[ch].tgr[addr - A_TGRA] = val;
            set_next_event(tpu);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "renesas_tpu: Unknown register %08lx\n",
                      addr);
    }
}

static uint64_t tpu012_read(void *opaque, hwaddr addr, unsigned size)
{
    int ch;
    ch = addr >> 4;
    addr &= 0x0f;
    return tpu_ch_read(opaque, ch, addr, size);
}

static uint64_t tpu345_read(void *opaque, hwaddr addr, unsigned size)
{
    int ch;
    ch = (addr >> 4) + 3;
    addr &= 0x0f;
    return tpu_ch_read(opaque, ch, addr, size);
}

static void tpu012_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    int ch;
    ch = addr >> 4;
    addr &= 0x0f;
    tpu_ch_write(opaque, ch, addr, val, size);
}

static void tpu345_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    int ch;
    ch = (addr >> 4) + 3;
    addr &= 0x0f;
    tpu_ch_write(opaque, ch, addr, val, size);
}

static const MemoryRegionOps tpu012_ops = {
    .write = tpu012_write,
    .read  = tpu012_read,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 2,
    },
};

static const MemoryRegionOps tpu345_ops = {
    .write = tpu345_write,
    .read  = tpu345_read,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 2,
    },
};

static uint64_t tpuc_read(void *opaque, hwaddr addr, unsigned size)
{
    RTPUState *tpu = opaque;

    switch(addr) {
    case 0:
        return tpu->tstr;
    case 1:
        return tpu->tsyr;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "renesas_tpu: Unknown register %08lx\n",
                      addr);
        return 0;
    }
}

static void tpuc_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    RTPUState *tpu = opaque;

    switch(addr) {
    case 0:
        tpu->tstr = val;
        set_next_event(tpu);
        break;
    case 1:
        tpu->tsyr = val;
        if (tpu->tsyr) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "renesas_tpu: Sync mode not support.\n");
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "renesas_tpu: Unknown register %08lx\n",
                      addr);
    }
}

static const MemoryRegionOps tpuc_ops = {
    .write = tpuc_write,
    .read  = tpuc_read,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static void rtpu_reset(DeviceState *dev)
{
    RTPUState *tpu = RTPU(dev);
    int i, j;
    for (i = 0; i < 6; i++) {
        memset(&tpu->ch[i], 0, sizeof(tpu->ch[i]));
        tpu->ch[i].tsr = 0xc0;
        for (j = 0; j < 4; j++) {
            tpu->ch[i].tgr[j] = 0xffff;
        }
    }
}

static void rtpu_realize(DeviceState *dev, Error **errp)
{
    RTPUState *s = RTPU(dev);
    s->clk_per_nsec = NANOSECONDS_PER_SECOND / s->input_freq;
}

static void rtpu_init(Object *obj)
{
    SysBusDevice *d = SYS_BUS_DEVICE(obj);
    RTPUState *tpu = RTPU(obj);
    int i;

    memory_region_init_io(&tpu->memory[0], OBJECT(tpu), &tpu012_ops,
                          tpu, "renesas-tpu012", 0x30);
    sysbus_init_mmio(d, &tpu->memory[0]);
    memory_region_init_io(&tpu->memory[1], OBJECT(tpu), &tpu345_ops,
                          tpu, "renesas-tpu345", 0x30);
    sysbus_init_mmio(d, &tpu->memory[1]);
    memory_region_init_io(&tpu->memory[2], OBJECT(tpu), &tpuc_ops,
                          tpu, "renesas-tpuctl", 0x2);
    sysbus_init_mmio(d, &tpu->memory[2]);

    for (i = 0; i < ARRAY_SIZE(tpu->irq); i++) {
        sysbus_init_irq(d, &tpu->irq[i]);
    }
    
    tpu->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, timer_event, tpu);
}

static const VMStateDescription vmstate_rtpu = {
    .name = "renesas-tpu",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static Property rtpu_properties[] = {
    DEFINE_PROP_UINT64("input-freq", RTPUState, input_freq, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void rtpu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_rtpu;
    dc->reset = rtpu_reset;
    dc->realize = rtpu_realize;
    device_class_set_props(dc, rtpu_properties);
}

static const TypeInfo rtpu_info = {
    .name       = TYPE_RENESAS_TPU,
    .parent     = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RTPUState),
    .instance_init = rtpu_init,
    .class_init = rtpu_class_init,
};

static void rtpu_register_types(void)
{
    type_register_static(&rtpu_info);
}

type_init(rtpu_register_types)
