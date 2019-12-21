#ifndef H8S_INTC_H
#define H8S_INTC_H

#include "qemu-common.h"
#include "hw/irq.h"
enum {
    NR_IRQS = 128,
};

struct H8SINTCState {
    SysBusDevice parent_obj;

    MemoryRegion memory[2];

    uint16_t ipr[11];
    uint16_t ier;
    uint32_t iscr;
    uint16_t isr;
    uint16_t itsr;
    uint16_t ssier;
    uint8_t intcr;

    int last_level[NR_IRQS];
    int req_irq;
    qemu_irq irq;
    uint64_t req[2];
    uint8_t irqin;
    void *im;
};
typedef struct H8SINTCState H8SINTCState;

#define TYPE_H8SINTC "h8s-intc"
#define H8SINTC(obj) OBJECT_CHECK(H8SINTCState, (obj), TYPE_H8SINTC)

#endif /* H8S_INTC_H */
