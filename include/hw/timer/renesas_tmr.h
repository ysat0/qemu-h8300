/*
 * Renesas 8bit timer Object
 *
 * Copyright (c) 2018 Yoshinori Sato
 *
 * This code is licensed under the GPL version 2 or later.
 *
 */

#ifndef HW_RENESAS_TMR_H
#define HW_RENESAS_TMR_H

#include "hw/sysbus.h"

#define TYPE_RENESAS_TMR "renesas-tmr"
#define RTMR(obj) OBJECT_CHECK(RTMRState, (obj), TYPE_RENESAS_TMR)

enum {
    TMR_CH = 2,
    TMR_NR_IRQ = 3 * TMR_CH,
};

typedef struct RTMRState {
    SysBusDevice parent_obj;

    int64_t input_freq;
    MemoryRegion memory;

    uint8_t tcnt[TMR_CH];
    uint8_t tcora[TMR_CH];
    uint8_t tcorb[TMR_CH];
    uint8_t tcr[TMR_CH];
    uint8_t tccr[TMR_CH];
    uint8_t tcor[TMR_CH];
    uint8_t tcsr[TMR_CH];
    int64_t last;
    qemu_irq cmia[TMR_CH];
    qemu_irq cmib[TMR_CH];
    qemu_irq ovi[TMR_CH];
    QEMUTimer *timer;
    uint32_t type;
} RTMRState;

#endif
