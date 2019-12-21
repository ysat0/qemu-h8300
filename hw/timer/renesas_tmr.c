/*
 * Renesas 8bit timer
 *
 * Datasheet: RX62N Group, RX621 Group User's Manual: Hardware
 * (Rev.1.40 R01UH0033EJ0140)
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
#include "qemu/bitops.h"
#include "cpu.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/timer/renesas_tmr.h"
#include "qemu/error-report.h"

REG8(TCR, 0)
  FIELD(TCR, CKS,  0, 3)
  FIELD(TCR, CCLR, 3, 2)
  FIELD(TCR, OVIE, 5, 1)
  FIELD(TCR, CMIEA, 6, 1)
  FIELD(TCR, CMIEB, 7, 1)
  FIELD(TCR, IE, 5, 3)
REG8(TCSR, 2)
  FIELD(TCSR, OSA, 0, 2)
  FIELD(TCSR, OSB, 2, 2)
  FIELD(TCSR, ADTE, 4, 1)
  FIELD(TCSR, OVF, 5, 1)
  FIELD(TCSR, CMFA, 6, 1)
  FIELD(TCSR, CMFB, 7, 1)
  FIELD(TCSR, F, 5, 3)
REG8(TCORA, 4)
REG8(TCORB, 6)
REG8(TCNT, 8)
REG8(TCCR, 10)
  FIELD(TCCR, CKS, 0, 3)
  FIELD(TCCR, CSS, 3, 2)
  FIELD(TCCR, TMRIS, 7, 1)

#define CASCADING0 0x03
#define CASCADING1 0x04
#define CCLR_A    0x01
#define CCLR_B    0x02

static inline int is_external(RTMRState *tmr, int ch)
{
    switch(tmr->type) {
    case 0:
        return FIELD_EX8(tmr->tccr[ch], TCCR, CSS) == 0;
    case 1:
        return FIELD_EX8(tmr->tcr[ch], TCR, CKS) >= 5;
    default:
        g_assert_not_reached();
    }
}

static inline int is_cascading(RTMRState *tmr, int ch)
{
    switch(tmr->type) {
    case 0:
        return FIELD_EX8(tmr->tccr[ch], TCCR, CSS) == CASCADING0;
    case 1:
        return FIELD_EX8(tmr->tcr[ch], TCR, CKS) == CASCADING1;
    default:
        g_assert_not_reached();
    }
}

static inline int divrate(RTMRState *tmr, int ch)
{
    static const int clkdiv0[] = {0, 1, 2, 8, 32, 64, 1024, 8192};
    static const int clkdiv1[] = {0, 8, 64, 8192};

    switch(tmr->type) {
    case 0:
        return clkdiv0[FIELD_EX8(tmr->tccr[ch], TCCR, CKS)];
    case 1:
        if (FIELD_EX8(tmr->tcr[ch], TCR, CKS) < 4) {
            return clkdiv1[FIELD_EX8(tmr->tcr[ch], TCR, CKS)];
        } else {
            return 0;
        }
    default:
        g_assert_not_reached();
    }
}

static int elapsed_count(RTMRState *tmr, int ch)
{
    int64_t now;
    int64_t elapsed;
    int div;

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    elapsed = now - tmr->last;
    elapsed /= NANOSECONDS_PER_SECOND / tmr->input_freq;
    if (ch == 0 && is_cascading(tmr, 0)) {
        div = divrate(tmr, 1);
    } else {
        div = divrate(tmr, ch);
    }
    if (div == 0) {
        return 0;
    } else {
        return elapsed / div;
    }
}

#define regcat(reg) ((reg[0] << 8) | reg[1])

static void set_next_event(RTMRState *tmr)
{
    int count;
    int ch;
    int i;
    int64_t next_event = INT64_MAX;

    /* Find the most recent event.　*/
    for (ch = 0; ch < 2; ch++) {
        int cmp[] = { 0x100,
                      tmr->tcora[ch],
                      tmr->tcorb[ch],
        };
        int64_t clk = divrate(tmr, ch) * NANOSECONDS_PER_SECOND / tmr->input_freq;
        count = 256;
        for (i = 0; i < 3; i++) {
            if (cmp[i] == 0) {
                continue;
            }
            count = MIN(count, cmp[i] - tmr->tcnt[ch]);
        }
        if (clk > 0) {
            next_event = MIN(next_event, count * clk);
        }
    }
    if (next_event == INT64_MAX) {
        /* No next event */
        return;
    }
    next_event += qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_mod(tmr->timer, next_event);
    tmr->last = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void update_tcsr(RTMRState *tmr, int ch, int tcnt)
{
    tmr->tcnt[ch] = tcnt & 0xff;
    if (tcnt == tmr->tcora[ch]) {
        tmr->tcsr[ch] = FIELD_DP8(tmr->tcsr[ch], TCSR, CMFA, 1);
        if (FIELD_EX8(tmr->tcr[ch], TCR, CCLR) & 0x01) {
            tmr->tcnt[ch] = 0;
        }
    }
    if (tcnt == tmr->tcorb[ch]) {
        tmr->tcsr[ch] = FIELD_DP8(tmr->tcsr[ch], TCSR, CMFB, 1);
        if (FIELD_EX8(tmr->tcr[ch], TCR, CCLR) & 0x02) {
            tmr->tcnt[ch] = 0;
        }
    }
    if (tcnt >= 0x100) {
        tmr->tcsr[ch] = FIELD_DP8(tmr->tcsr[ch], TCSR, OVF, 1);
    }
    set_next_event(tmr);
}

static void update_count(void *opaque)
{
    RTMRState *tmr = opaque;
    int ch;
    int ovf = 0;
    int tcnt;

    for (ch = 0; ch < 2; ch++) {
        int count = elapsed_count(tmr, ch);
        tcnt =  tmr->tcnt[ch];
        if (is_cascading(tmr, ch)) {
            continue;
        }
        tcnt += count;
        ovf = tcnt >> 8;
        update_tcsr(tmr, ch, tcnt);
    }
    if (is_cascading(tmr, 0)) {
        tcnt = tmr->tcnt[0];
        tcnt += ovf;
        update_tcsr(tmr, 0, tcnt);
    }
    if (is_cascading(tmr, 1) && tmr->tcnt[0] == tmr->tcora[0]) {
        tcnt = tmr->tcnt[1];
        tcnt++;
        update_tcsr(tmr, 1, tcnt);
    }
    for (ch = 0; ch < 2; ch++) {
        if (FIELD_EX8(tmr->tcr[ch], TCR, CMIEA) &
            FIELD_EX8(tmr->tcsr[ch], TCSR, CMFA)) {
            qemu_irq_pulse(tmr->cmia[ch]);
        }
        if (FIELD_EX8(tmr->tcr[ch], TCR, CMIEB) &
            FIELD_EX8(tmr->tcsr[ch], TCSR, CMFB)) {
            qemu_irq_pulse(tmr->cmib[ch]);
        }
        if (FIELD_EX8(tmr->tcr[ch], TCR, OVIE) &
            FIELD_EX8(tmr->tcsr[ch], TCSR, OVF)) {
            qemu_irq_pulse(tmr->ovi[ch]);
        }
    }
}

static int read_tcnt(RTMRState *tmr, int size, int ch)
{
    int cnt = elapsed_count(tmr, ch);
    if (size == 2) {
        return (regcat(tmr->tcnt) + cnt) & 0xffff;
    } else {
        return (tmr->tcnt[ch] + cnt) & 0xff;
    }
}

static inline uint8_t read_tccr(uint8_t r)
{
    uint8_t tccr = 0;
    tccr = FIELD_DP8(tccr, TCCR, TMRIS,
                     FIELD_EX8(r, TCCR, TMRIS));
    tccr = FIELD_DP8(tccr, TCCR, CSS,
                     FIELD_EX8(r, TCCR, CSS));
    tccr = FIELD_DP8(tccr, TCCR, CKS,
                     FIELD_EX8(r, TCCR, CKS));
    return tccr;
}

static uint64_t tmr_read(void *opaque, hwaddr addr, unsigned size)
{
    RTMRState *tmr = opaque;
    int ch = addr & 1;
    uint64_t ret;

    if (size == 2 && (ch != 0 || addr == A_TCR || addr == A_TCSR)) {
        qemu_log_mask(LOG_GUEST_ERROR, "renesas_tmr: Invalid read size 0x%"
                      HWADDR_PRIX "\n", addr);
        return UINT64_MAX;
    }
    switch (addr & 0x0e) {
    case A_TCR:
        ret = 0;
        ret = FIELD_DP8(ret, TCR, CKS,
                        FIELD_EX8(tmr->tcr[ch], TCR, CKS));
        ret = FIELD_DP8(ret, TCR, CCLR,
                        FIELD_EX8(tmr->tcr[ch], TCR, CCLR));
        ret = FIELD_DP8(ret, TCR, OVIE,
                        FIELD_EX8(tmr->tcr[ch], TCR, OVIE));
        ret = FIELD_DP8(ret, TCR, CMIEA,
                        FIELD_EX8(tmr->tcr[ch], TCR, CMIEA));
        ret = FIELD_DP8(ret, TCR, CMIEB,
                        FIELD_EX8(tmr->tcr[ch], TCR, CMIEB));
        return ret;
    case A_TCSR:
        ret = 0;
        ret = FIELD_DP8(ret, TCSR, OSA,
                        FIELD_EX8(tmr->tcsr[ch], TCSR, OSA));
        ret = FIELD_DP8(ret, TCSR, OSB,
                        FIELD_EX8(tmr->tcsr[ch], TCSR, OSB));
        switch (ch) {
        case 0:
            ret = FIELD_DP8(ret, TCSR, ADTE,
                            FIELD_EX8(tmr->tcsr[ch], TCSR, ADTE));
            break;
        case 1: /* CH1 ADTE unimplement always 1 */
            ret = FIELD_DP8(ret, TCSR, ADTE, 1);
            break;
        }
        return ret;
    case A_TCORA:
        if (size == 1) {
            return tmr->tcora[ch];
        } else if (ch == 0) {
            return regcat(tmr->tcora);
        }
    case A_TCORB:
        if (size == 1) {
            return tmr->tcorb[ch];
        } else {
            return regcat(tmr->tcorb);
        }
    case A_TCNT:
        return read_tcnt(tmr, size, ch);
    case A_TCCR:
        if (size == 1) {
            return read_tccr(tmr->tccr[ch]);
        } else {
            return read_tccr(tmr->tccr[0]) << 8 | read_tccr(tmr->tccr[1]);
        }
    default:
        qemu_log_mask(LOG_UNIMP, "renesas_tmr: Register 0x%" HWADDR_PRIX
                      " not implemented\n", addr);
        break;
    }
    return UINT64_MAX;
}

#define COUNT_WRITE(reg, val)                   \
    do {                                        \
        if (size == 1) {                        \
            tmr->reg[ch] = val;                 \
        } else {                                \
            tmr->reg[0] = extract32(val, 8, 8); \
            tmr->reg[1] = extract32(val, 0, 8); \
        }                                       \
        set_next_event(tmr);                    \
    } while (0)

static void tmr_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    RTMRState *tmr = opaque;
    int ch = addr & 1;

    if (size == 2 && ch != 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "renesas_tmr: Invalid write size 0x%" HWADDR_PRIX
                      "\n", addr);
        return;
    }
    switch (addr & 0x0e) {
    case A_TCR:
        if (size == 1) {
            tmr->tcr[ch] = val;
        } else {
            tmr->tcr[0] = extract32(val, 8, 8);
            tmr->tcr[1] = extract32(val, 0, 8);
        }
        if (tmr->type == 1) {
            set_next_event(tmr);
        }
        break;
    case A_TCSR:
        if (size == 1) {
            tmr->tcsr[ch] = val;
        } else {
            tmr->tcsr[0] = extract32(val, 8, 8);
            tmr->tcsr[1] = extract32(val, 0, 8);
        }
        break;
    case A_TCORA:
        COUNT_WRITE(tcora, val);
        break;
    case A_TCORB:
        COUNT_WRITE(tcorb, val);
        break;
    case A_TCNT:
        COUNT_WRITE(tcnt, val);
        break;
    case A_TCCR:
        COUNT_WRITE(tccr, val);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "renesas_tmr: Register 0x%" HWADDR_PRIX
                      " not implemented\n", addr);
        break;
    }
}

static const MemoryRegionOps tmr_ops = {
    .write = tmr_write,
    .read  = tmr_read,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 2,
    },
};

static void rtmr_reset(DeviceState *dev)
{
    RTMRState *tmr = RTMR(dev);
    tmr->tcr[0]   = tmr->tcr[1]   = 0x00;
    tmr->tcsr[0]  = 0x00;
    tmr->tcsr[1]  = 0x10;
    tmr->tcnt[0]  = tmr->tcnt[1]  = 0x00;
    tmr->tcora[0] = tmr->tcora[1] = 0xff;
    tmr->tcorb[0] = tmr->tcorb[1] = 0xff;
    tmr->tccr[0]  = tmr->tccr[1]  = 0x00;
}

static void rtmr_init(Object *obj)
{
    SysBusDevice *d = SYS_BUS_DEVICE(obj);
    RTMRState *tmr = RTMR(obj);
    int i;

    memory_region_init_io(&tmr->memory, OBJECT(tmr), &tmr_ops,
                          tmr, "renesas-tmr", 0x10);
    sysbus_init_mmio(d, &tmr->memory);

    for (i = 0; i < ARRAY_SIZE(tmr->ovi); i++) {
        sysbus_init_irq(d, &tmr->cmia[i]);
        sysbus_init_irq(d, &tmr->cmib[i]);
        sysbus_init_irq(d, &tmr->ovi[i]);
    }
    tmr->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, update_count, tmr);
}

static const VMStateDescription vmstate_rtmr = {
    .name = "rx-tmr",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static Property rtmr_properties[] = {
    DEFINE_PROP_INT64("input-freq", RTMRState, input_freq, 0),
    DEFINE_PROP_UINT32("timer-type", RTMRState, type, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void rtmr_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->props = rtmr_properties;
    dc->vmsd = &vmstate_rtmr;
    dc->reset = rtmr_reset;
}

static const TypeInfo rtmr_info = {
    .name       = TYPE_RENESAS_TMR,
    .parent     = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RTMRState),
    .instance_init = rtmr_init,
    .class_init = rtmr_class_init,
};

static void rtmr_register_types(void)
{
    type_register_static(&rtmr_info);
}

type_init(rtmr_register_types)
