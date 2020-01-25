// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Sunplus Plus1 SP7021 SoC interrupt controller
 *
 * Copyright (c) 2020 Andreas Färber
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#define REG_INTC_INTR_TYPE(i)		(0x0 + (i) * 4)
#define REG_INTC_INTR_POLARITY(i)	(0x1c + (i) * 4)
#define REG_INTC_INTR_PRIO(i)		(0x38 + (i) * 4)
#define REG_INTC_INTR_MASK(i)		(0x54 + (i) * 4)

#define REG_INTC_INTR_CLR(i)		(0x0 + (i) * 4)
#define REG_INTC_MASKED_FIQS(i)		(0x1c + (i) * 4)
#define REG_INTC_MASKED_IRQS(i)		(0x38 + (i) * 4)
#define REG_INTC_INTR_GROUP		0x7c

#define INTC_INTR_GROUP_FIQ	GENMASK(6, 0)
#define INTC_INTR_GROUP_IRQ	GENMASK(14, 8)

struct sp7021_intc_data {
	void __iomem		*base0;
	void __iomem		*base1;
	int			ext0_irq;
	int			ext1_irq;
	struct irq_chip		chip;
	struct irq_domain	*domain;
	raw_spinlock_t		lock;
};

static void sp7021_intc_ext0_irq_handle(struct irq_desc *desc)
{
	struct sp7021_intc_data *s = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 mask, masked;
	int i, j;

	chained_irq_enter(chip, desc);

	mask = readl_relaxed(s->base1 + REG_INTC_INTR_GROUP);
	mask = FIELD_GET(INTC_INTR_GROUP_IRQ, mask);
	while (mask) {
		i = fls(mask) - 1;
		mask &= ~BIT(i);

		masked = readl_relaxed(s->base1 + REG_INTC_MASKED_IRQS(i));
		while (masked) {
			j = fls(masked) - 1;
			masked &= ~BIT(j);

			generic_handle_irq(irq_find_mapping(s->domain,
							    i * 32 + j));
		}
	}

	chained_irq_exit(chip, desc);
}

static void sp7021_intc_ext1_irq_handle(struct irq_desc *desc)
{
	struct sp7021_intc_data *s = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 mask, masked;
	int i, j;

	chained_irq_enter(chip, desc);

	mask = readl_relaxed(s->base1 + REG_INTC_INTR_GROUP);
	mask = FIELD_GET(INTC_INTR_GROUP_FIQ, mask);
	while (mask) {
		i = fls(mask) - 1;
		mask &= ~BIT(i);

		masked = readl_relaxed(s->base1 + REG_INTC_MASKED_FIQS(i));
		while (masked) {
			j = fls(masked) - 1;
			masked &= ~BIT(j);

			generic_handle_irq(irq_find_mapping(s->domain,
							    i * 32 + j));
		}
	}

	chained_irq_exit(chip, desc);
}

static void sp7021_intc_ack_irq(struct irq_data *data)
{
	struct sp7021_intc_data *s = irq_data_get_irq_chip_data(data);
	unsigned int idx;
	u32 mask;

	idx = data->hwirq / 32;

	mask = BIT(data->hwirq % 32);
	writel_relaxed(mask, s->base1 + REG_INTC_INTR_CLR(idx));
}

static void sp7021_intc_mask_irq(struct irq_data *data)
{
	struct sp7021_intc_data *s = irq_data_get_irq_chip_data(data);
	unsigned long flags;
	unsigned int idx;
	u32 mask;

	idx = data->hwirq / 32;

	raw_spin_lock_irqsave(&s->lock, flags);

	mask = readl_relaxed(s->base0 + REG_INTC_INTR_MASK(idx));
	mask &= ~BIT(data->hwirq % 32);
	writel_relaxed(mask, s->base0 + REG_INTC_INTR_MASK(idx));

	raw_spin_unlock_irqrestore(&s->lock, flags);
}

static void sp7021_intc_unmask_irq(struct irq_data *data)
{
	struct sp7021_intc_data *s = irq_data_get_irq_chip_data(data);
	unsigned long flags;
	unsigned int idx;
	u32 mask;

	idx = data->hwirq / 32;

	raw_spin_lock_irqsave(&s->lock, flags);

	mask = readl_relaxed(s->base0 + REG_INTC_INTR_MASK(idx));
	mask |= BIT(data->hwirq % 32);
	writel_relaxed(mask, s->base0 + REG_INTC_INTR_MASK(idx));

	raw_spin_unlock_irqrestore(&s->lock, flags);
}

static int sp7021_intc_set_irq_type(struct irq_data *data, unsigned int flow_type)
{
	struct sp7021_intc_data *s = irq_data_get_irq_chip_data(data);
	unsigned long flags;
	unsigned int idx;
	u32 mask, type, polarity;

	idx = data->hwirq / 32;
	mask = BIT(data->hwirq % 32);

	if (flow_type & IRQ_TYPE_LEVEL_MASK)
		irq_set_chip_handler_name_locked(data, &s->chip, handle_level_irq, NULL);
	else
		irq_set_chip_handler_name_locked(data, &s->chip, handle_edge_irq, NULL);

	raw_spin_lock_irqsave(&s->lock, flags);

	type = readl_relaxed(s->base0 + REG_INTC_INTR_TYPE(idx));
	polarity = readl_relaxed(s->base0 + REG_INTC_INTR_POLARITY(idx));

	switch (flow_type) {
	case IRQ_TYPE_EDGE_RISING:
		type |= mask;
		polarity &= ~mask;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		type |= mask;
		polarity |= mask;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		type &= ~mask;
		polarity &= ~mask;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		type &= ~mask;
		polarity |= mask;
		break;
	default:
		raw_spin_unlock_irqrestore(&s->lock, flags);
		return -EBADR;
	}

	writel_relaxed(type, s->base0 + REG_INTC_INTR_TYPE(idx));
	writel_relaxed(polarity, s->base0 + REG_INTC_INTR_POLARITY(idx));

	raw_spin_unlock_irqrestore(&s->lock, flags);

	return IRQ_SET_MASK_OK;
}

static const struct irq_chip sp7021_intc_irq_chip = {
	.name			= "SP7021-A",
	.irq_ack		= sp7021_intc_ack_irq,
	.irq_mask		= sp7021_intc_mask_irq,
	.irq_unmask		= sp7021_intc_unmask_irq,
	.irq_set_type		= sp7021_intc_set_irq_type,
};

static int sp7021_intc_irq_domain_map(struct irq_domain *d,
		unsigned int irq, irq_hw_number_t hw)
{
	struct sp7021_intc_data *s = d->host_data;
	unsigned int idx;
	u32 mask, type;

	idx = hw / 32;
	mask = BIT(hw % 32);

	type = readl_relaxed(s->base0 + REG_INTC_INTR_TYPE(idx));

	irq_set_chip_and_handler(irq, &s->chip, (type & mask) ? handle_edge_irq : handle_level_irq);
	irq_set_chip_data(irq, s);
	irq_set_probe(irq);

	return 0;
}

static const struct irq_domain_ops sp7021_intc_domain_ops = {
	.xlate	= irq_domain_xlate_onecell,
	.map	= sp7021_intc_irq_domain_map,
};

int __init sp7021_intc_init(struct device_node *node, struct device_node *parent)
{
	struct sp7021_intc_data *s;
	void __iomem *base0, *base1;
	int i;

	base0 = of_iomap(node, 0);
	if (!base0)
		return -EIO;

	base1 = of_iomap(node, 1);
	if (!base1)
		return -EIO;

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->base0 = base0;
	s->base1 = base1;
	s->chip = sp7021_intc_irq_chip;

	s->ext0_irq = irq_of_parse_and_map(node, 0);
	if (s->ext0_irq <= 0) {
		kfree(s);
		return -EINVAL;
	}

	s->ext1_irq = irq_of_parse_and_map(node, 1);
	if (s->ext1_irq <= 0) {
		kfree(s);
		return -EINVAL;
	}

	raw_spin_lock_init(&s->lock);

	for (i = 0; i < 7; i++) {
		writel_relaxed(0, s->base0 + REG_INTC_INTR_MASK(i));
		writel_relaxed(~0, s->base0 + REG_INTC_INTR_TYPE(i));
		writel_relaxed(0, s->base0 + REG_INTC_INTR_POLARITY(i));

		/* irq, not fiq */
		writel_relaxed(~0, s->base0 + REG_INTC_INTR_PRIO(i));

		writel_relaxed(~0, s->base1 + REG_INTC_INTR_CLR(i));
	}

	s->domain = irq_domain_add_linear(node, 200, &sp7021_intc_domain_ops,
					  s);
	if (!s->domain) {
		kfree(s);
		return -ENOMEM;
	}

	irq_set_chained_handler_and_data(s->ext0_irq, sp7021_intc_ext0_irq_handle, s);
	irq_set_chained_handler_and_data(s->ext1_irq, sp7021_intc_ext1_irq_handle, s);

	return 0;
}
IRQCHIP_DECLARE(sp7021_intc, "sunplus,sp7021-intc", sp7021_intc_init);
