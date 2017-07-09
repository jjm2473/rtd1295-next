/*
 * Copyright (c) 2017 Andreas Färber
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
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

static int __init rda8810_intc_init(struct device_node *np,
				    struct device_node *parent)
{
	void __iomem *base;
	unsigned int i;

	base = of_io_request_and_map(np, 0, "rda-intc");
	if (!base)
		return -ENXIO;

	/*
	 * Mask, and invalid all interrupt sources
	 */
	writel(RDA_IRQ_MASK_ALL, base + RDA_INTC_MASK_CLR);

	for (i = 0; i < RDA_NR_IRQS; i++) {
		irq_set_chip_and_handler(i, &rda_irq_chip, handle_level_irq);
		irq_set_chip_data(i, base);
		irq_set_probe(i);
	}

	pr_info("RDA8810PL intc probed\n");

	return 0;
}
IRQCHIP_DECLARE(rda_intc, "rda,8810pl-intc", rda8810_intc_init);
