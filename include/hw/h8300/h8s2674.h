/*
 * H8S2674 MCU Object
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

#ifndef HW_H8300_H8S2674_H
#define HW_H8300_H8S2674_H

#include "hw/sysbus.h"
#include "target/h8300/cpu.h"
#include "qemu/units.h"
#include "hw/timer/renesas_tmr.h"
#include "hw/timer/renesas_tpu.h"
#include "hw/char/renesas_sci.h"
#include "hw/intc/h8s_intc.h"

#define TYPE_H8S2674 "h8s2674-mcu"
#define H8S2674(obj) OBJECT_CHECK(H8S2674State, (obj), TYPE_H8S2674)

typedef struct H8S2674State {
    /*< private >*/
    DeviceState parent_obj;
    /*< public >*/

    H8300CPU cpu;
    RTMRState tmr;
    RSCIState sci[3];
    RTPUState tpu;
    H8SINTCState intc;
    uint8_t intcr_val;

    MemoryRegion *sysmem;

    MemoryRegion iram;
    MemoryRegion iomem1;
    MemoryRegion iomem2;
    MemoryRegion flash;
    MemoryRegion intcr;
    uint64_t input_freq;
    uint32_t sci_con;
    qemu_irq irq[NR_IRQS];
} H8S2674State;

/*
 * H8S2674 Internal Memory
 */
#define H8S2674_IRAM_BASE 0x00ff4000
#define H8S2674_IRAM_SIZE (32 * KiB)

#define H8S2674_SCIBASE 0xffff78
#define H8S2674_TMRBASE 0xffffb0
#define H8S2674_TPUBASE1 0xffffd0
#define H8S2674_TPUBASE2 0xfffe80
#define H8S2674_TPUBASE3 0xffffc0
#define H8S2674_INTCBASE1 0xfffe00
#define H8S2674_INTCBASE2 0xffff30

#define H8S2674_TPU_IRQBASE 40
#define H8S2674_TMR_IRQBASE 72
#define H8S2674_SCI_IRQBASE 88

#endif
