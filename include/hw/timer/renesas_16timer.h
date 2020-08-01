/*
 * Renesas 16bit timer
 *
 * Copyright (c) 2020 Yoshinori Sato
 *
 * This code is licensed under the GPL version 2 or later.
 *
 */

#ifndef HW_RENESAS_TPU_H
#define HW_RENESAS_TPU_H

#include "hw/sysbus.h"

#define TYPE_RENESAS_16TMR "renesas-16tmr"
#define R16(obj) OBJECT_CHECK(R16State, (obj), TYPE_RENESAS_16TMR)

enum {
    TMR16_NR_IRQ = 12,
};

typedef struct R16State {
    SysBusDevice parent_obj;

    uint64_t input_freq;
    int clk_per_nsec;
    MemoryRegion memory;

    uint8_t tstr;
    uint8_t tsnc;
    uint8_t tmdr;
    uint8_t tolr;
    uint8_t tisr[3];
    struct {
        uint8_t tcr;
        uint8_t tior;
        uint32_t tcnt;
        uint16_t gr[2];
    } ch[3];

    qemu_irq irq[TMR16_NR_IRQ];
    QEMUTimer *timer;
    int64_t prev_time;
} R16State;

#endif
