/*
 * Copyright (c) 2017 Andreas Färber
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>

#define RDA_INTC_MASK_SET	0x08
#define RDA_INTC_MASK_CLR	0x0c

#define RDA_IRQ_MASK_ALL	0xFFFFFFFF

#define RDA_NR_IRQS 32

static void rda_intc_mask_irq(struct irq_data *d)
{
	void __iomem *base = (void __iomem *)irq_data_get_irq_chip_data(d);

	writel(BIT(d->irq), base + RDA_INTC_MASK_CLR);
}

static void rda_intc_unmask_irq(struct irq_data *d)
{
	void __iomem *base = (void __iomem *)irq_data_get_irq_chip_data(d);

	writel(BIT(d->irq), base + RDA_INTC_MASK_SET);
}

static int rda_intc_set_type(struct irq_data *data, unsigned int flow_type)
{
        if (flow_type & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)) {
                irq_set_handler(data->irq, handle_edge_irq);
        }
        if (flow_type & (IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW)) {
                irq_set_handler(data->irq, handle_level_irq);
        }
        return 0;
}

static struct irq_chip rda_irq_chip = {
	.name		= "rda-intc",
	.irq_ack 	= rda_intc_mask_irq,
	.irq_mask	= rda_intc_mask_irq,
	.irq_unmask 	= rda_intc_unmask_irq,
	.irq_set_type	= rda_intc_set_type,
	.irq_disable	= rda_intc_mask_irq,
};

static int rda_irq_map(struct irq_domain *d,
		       unsigned int virq, irq_hw_number_t hw)
{
	irq_set_status_flags(virq, IRQ_LEVEL);
	irq_set_chip_and_handler(virq, &rda_irq_chip, handle_level_irq);
	irq_set_chip_data(virq, d->host_data);
	irq_set_probe(virq);

	return 0;
}

static const struct irq_domain_ops rda_irq_domain_ops = {
	.map = rda_irq_map,
	.xlate = irq_domain_xlate_onecell,
};

static int __init rda8810_intc_init(struct device_node *np,
				    struct device_node *parent)
{
	void __iomem *base;

	base = of_io_request_and_map(np, 0, "rda-intc");
	if (!base)
		return -ENXIO;

	/*
	 * Mask, and invalid all interrupt sources
	 */
	writel(RDA_IRQ_MASK_ALL, base + RDA_INTC_MASK_CLR);

	irq_domain_add_simple(np, RDA_NR_IRQS, 0, &rda_irq_domain_ops, base);

	pr_info("RDA8810PL intc probed\n");

	return 0;
}
IRQCHIP_DECLARE(rda_intc, "rda,8810pl-intc", rda8810_intc_init);
