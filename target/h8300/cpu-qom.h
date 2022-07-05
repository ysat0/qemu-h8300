/*
 * H8300 CPU
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

#ifndef H8300_CPU_QOM_H
#define H8300_CPU_QOM_H

#include "hw/core/cpu.h"
#include "qom/object.h"

#define TYPE_H8300_CPU "h8300cpu"

OBJECT_DECLARE_CPU_TYPE(H8300CPU, H8300CPUClass, H8300_CPU)

/*
 * H8300CPUClass:
 * @parent_realize: The parent class' realize handler.
 * @parent_reset: The parent class' reset handler.
 *
 * A H8300 CPU model.
 */
/*
 * H8300CPUClass:
 * @parent_realize: The parent class' realize handler.
 * @parent_reset: The parent class' reset handler.
 *
 * A H8300 CPU model.
 */
typedef struct H8300CPUClass {
    /*< private >*/
    CPUClass parent_class;
    /*< public >*/

    DeviceRealize parent_realize;
    void (*parent_reset)(DeviceState *dev);

} H8300CPUClass;

#endif
