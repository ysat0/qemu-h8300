/*
 * Renesas Timer Pulse Unit 
 *
 * Copyright (c) 2019 Yoshinori Sato
 *
 * This code is licensed under the GPL version 2 or later.
 *
 */

#ifndef HW_RENESAS_TPU_H
#define HW_RENESAS_TPU_H

#include "hw/sysbus.h"

#define TYPE_RENESAS_TPU "renesas-tpu"
#define RTPU(obj) OBJECT_CHECK(RTPUState, (obj), TYPE_RENESAS_TPU)

enum {
    TPU_NR_IRQ = 26,
};

typedef struct RTPUState {
    SysBusDevice parent_obj;

    uint64_t input_freq;
    int clk_per_nsec;
    MemoryRegion memory[3];

    uint8_t tstr;
    uint8_t tsyr;
    struct {
        uint8_t tcr;
        uint8_t tmdr;
        uint16_t tior;
        uint8_t tier;
        uint8_t tsr;
        uint32_t tcnt;
        uint16_t tgr[4];
    } ch[6];

    qemu_irq irq[TPU_NR_IRQ];
    QEMUTimer *timer;
    int64_t prev_time;
} RTPUState;

#endif
