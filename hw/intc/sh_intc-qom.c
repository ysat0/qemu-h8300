/*
 * SuperH interrupt controller module
 *
 * Copyright (c) 2007 Magnus Damm
 * Based on sh_timer.c and arm_timer.c by Paul Brook
 * Copyright (c) 2005-2006 CodeSourcery.
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "hw/sh4/sh_intc.h"
#include "hw/irq.h"
#include "hw/sh4/sh.h"

//#define DEBUG_INTC
//#define DEBUG_INTC_SOURCES

/* sh775x interrupt controller tables for sh_intc.c
 * stolen from linux/arch/sh/kernel/cpu/sh4/setup-sh7750.c
 */

enum {
	UNUSED = 0,

	/* interrupt sources */
	IRL_0, IRL_1, IRL_2, IRL_3, IRL_4, IRL_5, IRL_6, IRL_7,
	IRL_8, IRL_9, IRL_A, IRL_B, IRL_C, IRL_D, IRL_E,
	IRL0, IRL1, IRL2, IRL3,
	HUDI, GPIOI,
	DMAC_DMTE0, DMAC_DMTE1, DMAC_DMTE2, DMAC_DMTE3,
	DMAC_DMTE4, DMAC_DMTE5, DMAC_DMTE6, DMAC_DMTE7,
	DMAC_DMAE,
	PCIC0_PCISERR, PCIC1_PCIERR, PCIC1_PCIPWDWN, PCIC1_PCIPWON,
	PCIC1_PCIDMA0, PCIC1_PCIDMA1, PCIC1_PCIDMA2, PCIC1_PCIDMA3,
	TMU3, TMU4, TMU0, TMU1, TMU2_TUNI, TMU2_TICPI,
	RTC_ATI, RTC_PRI, RTC_CUI,
	SCI1_ERI, SCI1_RXI, SCI1_TXI, SCI1_TEI,
	SCIF_ERI, SCIF_RXI, SCIF_BRI, SCIF_TXI,
	WDT,
	REF_RCMI, REF_ROVI,

	/* interrupt groups */
	DMAC, PCIC1, TMU2, RTC, SCI1, SCIF, REF,
	/* irl bundle */
	IRL,

	NR_SOURCES,
};

static struct intc_vect vectors[] = {
	INTC_VECT(HUDI, 0x600), INTC_VECT(GPIOI, 0x620),
	INTC_VECT(TMU0, 0x400), INTC_VECT(TMU1, 0x420),
	INTC_VECT(TMU2_TUNI, 0x440), INTC_VECT(TMU2_TICPI, 0x460),
	INTC_VECT(RTC_ATI, 0x480), INTC_VECT(RTC_PRI, 0x4a0),
	INTC_VECT(RTC_CUI, 0x4c0),
	INTC_VECT(SCI1_ERI, 0x4e0), INTC_VECT(SCI1_RXI, 0x500),
	INTC_VECT(SCI1_TXI, 0x520), INTC_VECT(SCI1_TEI, 0x540),
	INTC_VECT(SCIF_ERI, 0x700), INTC_VECT(SCIF_RXI, 0x720),
	INTC_VECT(SCIF_BRI, 0x740), INTC_VECT(SCIF_TXI, 0x760),
	INTC_VECT(WDT, 0x560),
	INTC_VECT(REF_RCMI, 0x580), INTC_VECT(REF_ROVI, 0x5a0),
};

static struct intc_group groups[] = {
	INTC_GROUP(TMU2, TMU2_TUNI, TMU2_TICPI),
	INTC_GROUP(RTC, RTC_ATI, RTC_PRI, RTC_CUI),
	INTC_GROUP(SCI1, SCI1_ERI, SCI1_RXI, SCI1_TXI, SCI1_TEI),
	INTC_GROUP(SCIF, SCIF_ERI, SCIF_RXI, SCIF_BRI, SCIF_TXI),
	INTC_GROUP(REF, REF_RCMI, REF_ROVI),
};

static struct intc_prio_reg prio_registers[] = {
	{ 0xffd00004, 0, 16, 4, /* IPRA */ { TMU0, TMU1, TMU2, RTC } },
	{ 0xffd00008, 0, 16, 4, /* IPRB */ { WDT, REF, SCI1, 0 } },
	{ 0xffd0000c, 0, 16, 4, /* IPRC */ { GPIOI, DMAC, SCIF, HUDI } },
	{ 0xffd00010, 0, 16, 4, /* IPRD */ { IRL0, IRL1, IRL2, IRL3 } },
	{ 0xfe080000, 0, 32, 4, /* INTPRI00 */ { 0, 0, 0, 0,
						 TMU4, TMU3,
						 PCIC1, PCIC0_PCISERR } },
};

/* SH7750, SH7750S, SH7751 and SH7091 all have 4-channel DMA controllers */

static struct intc_vect vectors_dma4[] = {
	INTC_VECT(DMAC_DMTE0, 0x640), INTC_VECT(DMAC_DMTE1, 0x660),
	INTC_VECT(DMAC_DMTE2, 0x680), INTC_VECT(DMAC_DMTE3, 0x6a0),
	INTC_VECT(DMAC_DMAE, 0x6c0),
};

static struct intc_group groups_dma4[] = {
	INTC_GROUP(DMAC, DMAC_DMTE0, DMAC_DMTE1, DMAC_DMTE2,
		   DMAC_DMTE3, DMAC_DMAE),
};

/* SH7750R and SH7751R both have 8-channel DMA controllers */

static struct intc_vect vectors_dma8[] = {
	INTC_VECT(DMAC_DMTE0, 0x640), INTC_VECT(DMAC_DMTE1, 0x660),
	INTC_VECT(DMAC_DMTE2, 0x680), INTC_VECT(DMAC_DMTE3, 0x6a0),
	INTC_VECT(DMAC_DMTE4, 0x780), INTC_VECT(DMAC_DMTE5, 0x7a0),
	INTC_VECT(DMAC_DMTE6, 0x7c0), INTC_VECT(DMAC_DMTE7, 0x7e0),
	INTC_VECT(DMAC_DMAE, 0x6c0),
};

static struct intc_group groups_dma8[] = {
	INTC_GROUP(DMAC, DMAC_DMTE0, DMAC_DMTE1, DMAC_DMTE2,
		   DMAC_DMTE3, DMAC_DMTE4, DMAC_DMTE5,
		   DMAC_DMTE6, DMAC_DMTE7, DMAC_DMAE),
};

/* SH7750R, SH7751 and SH7751R all have two extra timer channels */

static struct intc_vect vectors_tmu34[] = {
	INTC_VECT(TMU3, 0xb00), INTC_VECT(TMU4, 0xb80),
};

static struct intc_mask_reg mask_registers[] = {
	{ 0xfe080040, 0xfe080060, 32, /* INTMSK00 / INTMSKCLR00 */
	  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	    0, 0, 0, 0, 0, 0, TMU4, TMU3,
	    PCIC1_PCIERR, PCIC1_PCIPWDWN, PCIC1_PCIPWON,
	    PCIC1_PCIDMA0, PCIC1_PCIDMA1, PCIC1_PCIDMA2,
	    PCIC1_PCIDMA3, PCIC0_PCISERR } },
};

/* SH7750S, SH7750R, SH7751 and SH7751R all have IRLM priority registers */

static struct intc_vect vectors_irlm[] = {
	INTC_VECT(IRL0, 0x240), INTC_VECT(IRL1, 0x2a0),
	INTC_VECT(IRL2, 0x300), INTC_VECT(IRL3, 0x360),
};

/* SH7751 and SH7751R both have PCI */

static struct intc_vect vectors_pci[] = {
	INTC_VECT(PCIC0_PCISERR, 0xa00), INTC_VECT(PCIC1_PCIERR, 0xae0),
	INTC_VECT(PCIC1_PCIPWDWN, 0xac0), INTC_VECT(PCIC1_PCIPWON, 0xaa0),
	INTC_VECT(PCIC1_PCIDMA0, 0xa80), INTC_VECT(PCIC1_PCIDMA1, 0xa60),
	INTC_VECT(PCIC1_PCIDMA2, 0xa40), INTC_VECT(PCIC1_PCIDMA3, 0xa20),
};

static struct intc_group groups_pci[] = {
	INTC_GROUP(PCIC1, PCIC1_PCIERR, PCIC1_PCIPWDWN, PCIC1_PCIPWON,
		   PCIC1_PCIDMA0, PCIC1_PCIDMA1, PCIC1_PCIDMA2, PCIC1_PCIDMA3),
};

static struct intc_vect vectors_irl[] = {
	INTC_VECT(IRL_0, 0x200),
	INTC_VECT(IRL_1, 0x220),
	INTC_VECT(IRL_2, 0x240),
	INTC_VECT(IRL_3, 0x260),
	INTC_VECT(IRL_4, 0x280),
	INTC_VECT(IRL_5, 0x2a0),
	INTC_VECT(IRL_6, 0x2c0),
	INTC_VECT(IRL_7, 0x2e0),
	INTC_VECT(IRL_8, 0x300),
	INTC_VECT(IRL_9, 0x320),
	INTC_VECT(IRL_A, 0x340),
	INTC_VECT(IRL_B, 0x360),
	INTC_VECT(IRL_C, 0x380),
	INTC_VECT(IRL_D, 0x3a0),
	INTC_VECT(IRL_E, 0x3c0),
};

static struct intc_group groups_irl[] = {
	INTC_GROUP(IRL, IRL_0, IRL_1, IRL_2, IRL_3, IRL_4, IRL_5, IRL_6,
		IRL_7, IRL_8, IRL_9, IRL_A, IRL_B, IRL_C, IRL_D, IRL_E),
};

#define INTC_A7(x) ((x) & 0x1fffffff)

static void sh_intc_toggle_source(struct intc_source *source,
			   int enable_adj, int assert_adj)
{
    int enable_changed = 0;
    int pending_changed = 0;
    int old_pending;

    if ((source->enable_count == source->enable_max) && (enable_adj == -1))
        enable_changed = -1;

    source->enable_count += enable_adj;

    if (source->enable_count == source->enable_max)
        enable_changed = 1;

    source->asserted += assert_adj;

    old_pending = source->pending;
    source->pending = source->asserted &&
      (source->enable_count == source->enable_max);

    if (old_pending != source->pending)
        pending_changed = 1;

    if (pending_changed) {
        if (source->pending) {
            source->parent->pending++;
            if (source->parent->pending == 1) {
                cpu_interrupt(first_cpu, CPU_INTERRUPT_HARD);
            }
        } else {
            source->parent->pending--;
            if (source->parent->pending == 0) {
                cpu_reset_interrupt(first_cpu, CPU_INTERRUPT_HARD);
            }
	}
    }

  if (enable_changed || assert_adj || pending_changed) {
#ifdef DEBUG_INTC_SOURCES
            printf("sh_intc: (%d/%d/%d/%d) interrupt source 0x%x %s%s%s\n",
		   source->parent->pending,
		   source->asserted,
		   source->enable_count,
		   source->enable_max,
		   source->vect,
		   source->asserted ? "asserted " :
		   assert_adj ? "deasserted" : "",
		   enable_changed == 1 ? "enabled " :
		   enable_changed == -1 ? "disabled " : "",
		   source->pending ? "pending" : "");
#endif
  }
}

static void sh_intc_set_irq (void *opaque, int n, int level)
{
  struct intc_desc *desc = opaque;
  struct intc_source *source = &(desc->sources[n]);

  if (level && !source->asserted)
    sh_intc_toggle_source(source, 0, 1);
  else if (!level && source->asserted)
    sh_intc_toggle_source(source, 0, -1);
}

int sh_intc_get_pending_vector(struct intc_desc *desc, int imask)
{
    unsigned int i;

    /* slow: use a linked lists of pending sources instead */
    /* wrong: take interrupt priority into account (one list per priority) */

    if (imask == 0x0f) {
        return -1; /* FIXME, update code to include priority per source */
    }

    for (i = 0; i < desc->nr_sources; i++) {
        struct intc_source *source = desc->sources + i;

	if (source->pending) {
#ifdef DEBUG_INTC_SOURCES
            printf("sh_intc: (%d) returning interrupt source 0x%x\n",
		   desc->pending, source->vect);
#endif
            return source->vect;
	}
    }

    abort();
}

#define INTC_MODE_NONE       0
#define INTC_MODE_DUAL_SET   1
#define INTC_MODE_DUAL_CLR   2
#define INTC_MODE_ENABLE_REG 3
#define INTC_MODE_MASK_REG   4
#define INTC_MODE_IS_PRIO    8

static unsigned int sh_intc_mode(unsigned long address,
				 unsigned long set_reg, unsigned long clr_reg)
{
    if ((address != INTC_A7(set_reg)) &&
	(address != INTC_A7(clr_reg)))
        return INTC_MODE_NONE;

    if (set_reg && clr_reg) {
        if (address == INTC_A7(set_reg))
            return INTC_MODE_DUAL_SET;
	else
            return INTC_MODE_DUAL_CLR;
    }

    if (set_reg)
        return INTC_MODE_ENABLE_REG;
    else
        return INTC_MODE_MASK_REG;
}

static void sh_intc_locate(struct intc_desc *desc,
			   unsigned long address,
			   unsigned long **datap,
			   intc_enum **enums,
			   unsigned int *first,
			   unsigned int *width,
			   unsigned int *modep)
{
    unsigned int i, mode;

    /* this is slow but works for now */

    if (desc->mask_regs) {
        for (i = 0; i < desc->nr_mask_regs; i++) {
	    struct intc_mask_reg *mr = desc->mask_regs + i;

	    mode = sh_intc_mode(address, mr->set_reg, mr->clr_reg);
	    if (mode == INTC_MODE_NONE)
                continue;

	    *modep = mode;
	    *datap = &mr->value;
	    *enums = mr->enum_ids;
	    *first = mr->reg_width - 1;
	    *width = 1;
	    return;
	}
    }

    if (desc->prio_regs) {
        for (i = 0; i < desc->nr_prio_regs; i++) {
	    struct intc_prio_reg *pr = desc->prio_regs + i;

	    mode = sh_intc_mode(address, pr->set_reg, pr->clr_reg);
	    if (mode == INTC_MODE_NONE)
                continue;

	    *modep = mode | INTC_MODE_IS_PRIO;
	    *datap = &pr->value;
	    *enums = pr->enum_ids;
	    *first = (pr->reg_width / pr->field_width) - 1;
	    *width = pr->field_width;
	    return;
	}
    }

    abort();
}

static void sh_intc_toggle_mask(struct intc_desc *desc, intc_enum id,
				int enable, int is_group)
{
    struct intc_source *source = desc->sources + id;

    if (!id)
	return;

    if (!source->next_enum_id && (!source->enable_max || !source->vect)) {
#ifdef DEBUG_INTC_SOURCES
        printf("sh_intc: reserved interrupt source %d modified\n", id);
#endif
	return;
    }

    if (source->vect)
        sh_intc_toggle_source(source, enable ? 1 : -1, 0);

#ifdef DEBUG_INTC
    else {
        printf("setting interrupt group %d to %d\n", id, !!enable);
    }
#endif

    if ((is_group || !source->vect) && source->next_enum_id) {
        sh_intc_toggle_mask(desc, source->next_enum_id, enable, 1);
    }

#ifdef DEBUG_INTC
    if (!source->vect) {
        printf("setting interrupt group %d to %d - done\n", id, !!enable);
    }
#endif
}

static uint64_t sh_intc_read(void *opaque, hwaddr offset,
                             unsigned size)
{
    struct intc_desc *desc = opaque;
    intc_enum *enum_ids = NULL;
    unsigned int first = 0;
    unsigned int width = 0;
    unsigned int mode = 0;
    unsigned long *valuep;

#ifdef DEBUG_INTC
    printf("sh_intc_read 0x%lx\n", (unsigned long) offset);
#endif

    sh_intc_locate(desc, (unsigned long)offset, &valuep, 
		   &enum_ids, &first, &width, &mode);
    return *valuep;
}

static void sh_intc_write(void *opaque, hwaddr offset,
                          uint64_t value, unsigned size)
{
    struct intc_desc *desc = opaque;
    intc_enum *enum_ids = NULL;
    unsigned int first = 0;
    unsigned int width = 0;
    unsigned int mode = 0;
    unsigned int k;
    unsigned long *valuep;
    unsigned long mask;

#ifdef DEBUG_INTC
    printf("sh_intc_write 0x%lx 0x%08x\n", (unsigned long) offset, value);
#endif

    sh_intc_locate(desc, (unsigned long)offset, &valuep, 
		   &enum_ids, &first, &width, &mode);

    switch (mode) {
    case INTC_MODE_ENABLE_REG | INTC_MODE_IS_PRIO: break;
    case INTC_MODE_DUAL_SET: value |= *valuep; break;
    case INTC_MODE_DUAL_CLR: value = *valuep & ~value; break;
    default: abort();
    }

    for (k = 0; k <= first; k++) {
        mask = ((1 << width) - 1) << ((first - k) * width);

	if ((*valuep & mask) == (value & mask))
            continue;
#if 0
	printf("k = %d, first = %d, enum = %d, mask = 0x%08x\n", 
	       k, first, enum_ids[k], (unsigned int)mask);
#endif
        sh_intc_toggle_mask(desc, enum_ids[k], value & mask, 0);
    }

    *valuep = value;

#ifdef DEBUG_INTC
    printf("sh_intc_write 0x%lx -> 0x%08x\n", (unsigned long) offset, value);
#endif
}

static const MemoryRegionOps sh_intc_ops = {
    .read = sh_intc_read,
    .write = sh_intc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

struct intc_source *sh_intc_source(struct intc_desc *desc, intc_enum id)
{
    if (id)
        return desc->sources + id;

    return NULL;
}

static unsigned int sh_intc_register(MemoryRegion *sysmem,
                             struct intc_desc *desc,
                             const unsigned long address,
                             const char *type,
                             const char *action,
                             const unsigned int index)
{
    char name[60];
    MemoryRegion *iomem, *iomem_p4, *iomem_a7;

    if (!address) {
        return 0;
    }

    iomem = &desc->iomem;
    iomem_p4 = desc->iomem_aliases + index;
    iomem_a7 = iomem_p4 + 1;

#define SH_INTC_IOMEM_FORMAT "interrupt-controller-%s-%s-%s"
    snprintf(name, sizeof(name), SH_INTC_IOMEM_FORMAT, type, action, "p4");
    memory_region_init_alias(iomem_p4, NULL, name, iomem, INTC_A7(address), 4);
    memory_region_add_subregion(sysmem, P4ADDR(address), iomem_p4);

    snprintf(name, sizeof(name), SH_INTC_IOMEM_FORMAT, type, action, "a7");
    memory_region_init_alias(iomem_a7, NULL, name, iomem, INTC_A7(address), 4);
    memory_region_add_subregion(sysmem, A7ADDR(address), iomem_a7);
#undef SH_INTC_IOMEM_FORMAT

    /* used to increment aliases index */
    return 2;
}

static void sh_intc_register_source(struct intc_desc *desc,
				    intc_enum source,
				    struct intc_group *groups,
				    int nr_groups)
{
    unsigned int i, k;
    struct intc_source *s;

    if (desc->mask_regs) {
        for (i = 0; i < desc->nr_mask_regs; i++) {
	    struct intc_mask_reg *mr = desc->mask_regs + i;

	    for (k = 0; k < ARRAY_SIZE(mr->enum_ids); k++) {
                if (mr->enum_ids[k] != source)
                    continue;

		s = sh_intc_source(desc, mr->enum_ids[k]);
		if (s)
                    s->enable_max++;
	    }
	}
    }

    if (desc->prio_regs) {
        for (i = 0; i < desc->nr_prio_regs; i++) {
	    struct intc_prio_reg *pr = desc->prio_regs + i;

	    for (k = 0; k < ARRAY_SIZE(pr->enum_ids); k++) {
                if (pr->enum_ids[k] != source)
                    continue;

		s = sh_intc_source(desc, pr->enum_ids[k]);
		if (s)
                    s->enable_max++;
	    }
	}
    }

    if (groups) {
        for (i = 0; i < nr_groups; i++) {
	    struct intc_group *gr = groups + i;

	    for (k = 0; k < ARRAY_SIZE(gr->enum_ids); k++) {
                if (gr->enum_ids[k] != source)
                    continue;

		s = sh_intc_source(desc, gr->enum_ids[k]);
		if (s)
                    s->enable_max++;
	    }
	}
    }

}

void sh_intc_register_sources(struct intc_desc *desc,
			      struct intc_vect *vectors,
			      int nr_vectors,
			      struct intc_group *groups,
			      int nr_groups)
{
    unsigned int i, k;
    struct intc_source *s;

    for (i = 0; i < nr_vectors; i++) {
	struct intc_vect *vect = vectors + i;

	sh_intc_register_source(desc, vect->enum_id, groups, nr_groups);
	s = sh_intc_source(desc, vect->enum_id);
        if (s) {
            s->vect = vect->vect;

#ifdef DEBUG_INTC_SOURCES
            printf("sh_intc: registered source %d -> 0x%04x (%d/%d)\n",
                   vect->enum_id, s->vect, s->enable_count, s->enable_max);
#endif
        }
    }

    if (groups) {
        for (i = 0; i < nr_groups; i++) {
	    struct intc_group *gr = groups + i;

	    s = sh_intc_source(desc, gr->enum_id);
	    s->next_enum_id = gr->enum_ids[0];

	    for (k = 1; k < ARRAY_SIZE(gr->enum_ids); k++) {
                if (!gr->enum_ids[k])
                    continue;

		s = sh_intc_source(desc, gr->enum_ids[k - 1]);
		s->next_enum_id = gr->enum_ids[k];
	    }

#ifdef DEBUG_INTC_SOURCES
	    printf("sh_intc: registered group %d (%d/%d)\n",
		   gr->enum_id, s->enable_count, s->enable_max);
#endif
	}
    }
}

int sh_intc_init(MemoryRegion *sysmem,
         struct intc_desc *desc,
		 int nr_sources,
		 struct intc_mask_reg *mask_regs,
		 int nr_mask_regs,
		 struct intc_prio_reg *prio_regs,
		 int nr_prio_regs)
{
    unsigned int i, j;

    desc->pending = 0;
    desc->nr_sources = nr_sources;
    desc->mask_regs = mask_regs;
    desc->nr_mask_regs = nr_mask_regs;
    desc->prio_regs = prio_regs;
    desc->nr_prio_regs = nr_prio_regs;
    /* Allocate 4 MemoryRegions per register (2 actions * 2 aliases).
     **/
    desc->iomem_aliases = g_new0(MemoryRegion,
                                 (nr_mask_regs + nr_prio_regs) * 4);

    j = 0;
    i = sizeof(struct intc_source) * nr_sources;
    desc->sources = g_malloc0(i);

    for (i = 0; i < desc->nr_sources; i++) {
        struct intc_source *source = desc->sources + i;

        source->parent = desc;
    }

    desc->irqs = qemu_allocate_irqs(sh_intc_set_irq, desc, nr_sources);
 
    memory_region_init_io(&desc->iomem, NULL, &sh_intc_ops, desc,
                          "interrupt-controller", 0x100000000ULL);


    return 0;
}

/* Assert level <n> IRL interrupt. 
   0:deassert. 1:lowest priority,... 15:highest priority. */
void sh_intc_set_irl(void *opaque, int n, int level)
{
    struct intc_source *s = opaque;
    int i, irl = level ^ 15;
    for (i = 0; (s = sh_intc_source(s->parent, s->next_enum_id)); i++) {
	if (i == irl)
	    sh_intc_toggle_source(s, s->enable_count?0:1, s->asserted?0:1);
	else
	    if (s->asserted)
	        sh_intc_toggle_source(s, 0, -1);
    }
}
