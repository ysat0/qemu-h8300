/*
 * H83069 MCU Object
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

#ifndef HW_H8300_H83069_H
#define HW_H8300_H83069_H

#include "hw/sysbus.h"
#include "target/h8300/cpu.h"
#include "qemu/units.h"
#include "hw/timer/renesas_tmr.h"
#include "hw/timer/renesas_16timer.h"
#include "hw/char/renesas_sci.h"
#include "hw/intc/h8300h_intc.h"

#define TYPE_H83069 "h83069"
#define TYPE_H83069_CPU H8300_CPU_TYPE_NAME(TYPE_H83069)
#define H83069(obj) OBJECT_CHECK(H83069State, (obj), TYPE_H83069)

typedef struct H83069State {
    SysBusDevice parent_obj;

    H8300CPU cpu;
    RTMRState tmr[2];
    RSCIState sci[3];
    R16State tmr16;
    H8300HINTCState intc;
    uint8_t syscr_val;

    MemoryRegion *sysmem;

    MemoryRegion iram;
    MemoryRegion iomem1;
    MemoryRegion iomem2;
    MemoryRegion flash;
    MemoryRegion syscr;
    uint64_t input_freq;
    uint32_t sci_con;
    qemu_irq irq[NR_IRQS];
} H83069State;

/*
 * H83069 Internal Memory
 * It is the value of R5F562N8.
 * Please change the size for R5F562N7.
 */
#define H83069_IRAM_BASE 0x00ffbf20
#define H83069_IRAM_SIZE (16 * KiB)
#define H83069_FLASH_BASE 0
#define H83069_FLASH_SIZE (512 * KiB)

#define H83069_SCIBASE 0xffffb0
#define H83069_TMRBASE 0xffff80
#define H83069_INTCBASE 0xfee014
#define H83069_16TIMER_BASE 0xffff60
#define H83069_SYSCR 0xfee012

#define H83069_16TIMER_IRQBASE 24
#define H83069_TMR_IRQBASE 36
#define H83069_SCI_IRQBASE 52

#endif
