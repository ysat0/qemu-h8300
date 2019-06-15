#ifndef H8300_INTC_H
#define H8300_INTC_H

#include "qemu-common.h"
#include "hw/irq.h"

enum {
    NR_IRQS = 64,
};

struct H8300HINTCState {
    SysBusDevice parent_obj;

    MemoryRegion memory;

    uint8_t iscr;
    uint8_t ier;
    uint8_t isr;
    uint16_t ipr;

    int req_irq;
    qemu_irq irq;
    uint64_t req;
    uint8_t irqin;
};
typedef struct H8300HINTCState H8300HINTCState;

#define TYPE_H8300HINTC "h8300h-intc"
#define H8300HINTC(obj) OBJECT_CHECK(H8300HINTCState, (obj), TYPE_H8300HINTC)

#endif /* H8300_INTC_H */
