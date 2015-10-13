/*
 * Spansion FM4 interrupt handling
 *
 * Copyright (c) 2015 Andreas Färber
 *
 * License: GPL-2.0+
 */

#include <linux/interrupt.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_irq.h>

static struct irq_domain *fm4_irq_domain;

static int fm4_irq_domain_xlate(struct irq_domain *d,
				struct device_node *controller,
				const u32 *intspec, unsigned int intsize,
				unsigned long *out_hwirq,
				unsigned int *out_type)
{
	if (irq_domain_get_of_node(d) != controller)
		return -EINVAL;

	if (intsize < 2)
		return -EINVAL;

	*out_hwirq = intspec[0] * 32 + intspec[1];
	*out_type = 0;

	pr_info("%s: xlate %u %u -> %lu\n", irq_domain_get_of_node(d)->full_name, intspec[0], intspec[1], *out_hwirq);

	return 0;
}

static struct irq_chip fm4_chip = {
	.name			= "FM4",
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_mask		= irq_chip_mask_parent,
	.irq_unmask		= irq_chip_unmask_parent,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
	.irq_set_type		= irq_chip_set_type_parent,
};

static int fm4_alloc_nvic_irq(struct irq_domain *d, unsigned int virq, irq_hw_number_t hwirq)
{
	struct of_phandle_args args;

	args.np = irq_domain_get_of_node(d->parent);
	args.args_count = 1;
	args.args[0] = hwirq / 32;

	return irq_domain_alloc_irqs_parent(d, virq, 1, &args);
}

static int fm4_irq_domain_alloc(struct irq_domain *d, unsigned int virq,
				unsigned int nr_irqs, void *data)
{
	struct of_phandle_args *args = data;
	irq_hw_number_t hwirq;
	int i;

	if (args->args_count != 2)
		return -EINVAL;

	hwirq = args->args[0] * 32 + args->args[1];
	if (hwirq + nr_irqs > 128 * 32)
		return -EINVAL;

	for (i = 0; i < nr_irqs; i++) {
		int err = fm4_alloc_nvic_irq(d, virq + i, hwirq + i);
		if (err)
			return err;
		irq_domain_set_hwirq_and_chip(d, virq + i, hwirq + i, &fm4_chip, NULL);
		//irq_set_chained_handler_and_data(irq_find_mapping(d->parent, (hwirq + i) / 32));
	}
	pr_info("%s: alloc %u (%u) %u %u -> %lu\n", irq_domain_get_of_node(d)->full_name,
		virq, nr_irqs, args->args[0], args->args[1], hwirq);
	return 0;
}

static const struct irq_domain_ops fm4_irq_domain_ops = {
	.xlate = fm4_irq_domain_xlate,
	.alloc = fm4_irq_domain_alloc,
};

static int __init fm4_of_init(struct device_node *node, struct device_node *parent)
{
	struct irq_domain *parent_domain, *domain;

	if (!parent) {
		pr_err("%s: no parent\n", node->full_name);
		return -ENODEV;
	}

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		pr_err("%s: unable to obtain parent domain\n", node->full_name);
		return -ENXIO;
	}

	domain = irq_domain_add_hierarchy(parent_domain, 0, parent_domain->hwirq_max * 32, node, &fm4_irq_domain_ops, NULL);
	if (!domain)
		return -ENOMEM;
	fm4_irq_domain = domain;

	pr_info("%s: done (%lu)\n", node->full_name, domain->hwirq_max);

	return 0;
}
IRQCHIP_DECLARE(fm4_irq, "cypress,fm4-intc", fm4_of_init);
