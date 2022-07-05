/*
 * QEMU NE2000 emulation -- isa bus windup
 *
 * Copyright (c) 2003-2004 Fabrice Bellard
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#ifndef HW_NET_NE2000_LOCAL_H
#define HW_NET_NE2000_LOCAL_H

#include "hw/hw.h"
#include "hw/qdev-properties.h"
#include "net/net.h"

void ne2000_init(NICInfo *nd, uint32_t base, qemu_irq irq);

#endif
