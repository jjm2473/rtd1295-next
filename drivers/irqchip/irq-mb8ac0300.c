/*
 * linux/arch/arm/mach-mb8ac0300/exiu.c
 *
 * Copyright (C) 2011 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/irqchip/irq-mb8ac0300.h>

#define DRIVER_NAME		"mb8ac0300-exiu"
#define DRIVER_DESC		"MB8AC0300 External Interrupt Unit"

/* two banks */
#define BANKLEN 16

static void __iomem *exiu_base;
static spinlock_t exiu_lock;
static int exiu_irq_base[2]; /* 0-15 and 16-31 base irq */
static u32 exiu_state[EXIU_REG_EXTENT / sizeof(u32)];

static int exiu_irq_to_shift(int irq)
{
	if ((irq < exiu_irq_base[0]) ||
			(irq >= (exiu_irq_base[1] + BANKLEN)) ||
			(((irq >= (exiu_irq_base[0] + BANKLEN))) &&
						(irq < exiu_irq_base[1])))
		return -1;

	if (irq < (exiu_irq_base[0] + BANKLEN))
		return irq - exiu_irq_base[0];

	return irq - exiu_irq_base[1] + EXTINT16_OFFSET;
}


static void exiu_irq_eoi(struct irq_data *d)
{
	int shift;

	shift = exiu_irq_to_shift(d->hwirq);
	if (shift < 0)
		return;

	__raw_writel(1 << shift, exiu_base + EXIU_REG_EIREQCLR);
}

static void exiu_irq_mask(struct irq_data *d)
{
	unsigned long val;
	int shift;

	shift = exiu_irq_to_shift(d->hwirq);
	if (shift < 0)
		return;

	/* disable interrupt request */
	val = __raw_readl(exiu_base + EXIU_REG_EIMASK);
	val |= (0x1 << shift);
	__raw_writel(val, exiu_base + EXIU_REG_EIMASK);
}

static void exiu_irq_unmask(struct irq_data *d)
{
	unsigned long val;
	int shift;

	shift = exiu_irq_to_shift(d->hwirq);
	if (shift < 0)
		return;

	/* enable an interrupt request */
	val = __raw_readl(exiu_base + EXIU_REG_EIMASK);
	val &= ~(0x1 << shift);
	__raw_writel(val, exiu_base + EXIU_REG_EIMASK);
}

int exiu_irq_set_type(unsigned long irq_num, unsigned long type)
{
	unsigned long eilvl, eiedg, flags;
	int shift;

	shift = exiu_irq_to_shift(irq_num);
	if (shift < 0) {
		pr_err("%s(): Bad exiu irq num %d.", __func__, (int)irq_num);
		return -EINVAL;
	}

	spin_lock_irqsave(&exiu_lock, flags);
	eilvl = __raw_readl(exiu_base + EXIU_REG_EILVL);
	eiedg = __raw_readl(exiu_base + EXIU_REG_EIEDG);
	eilvl &= ~(EXIU_EILVL_MASK << shift);
	eiedg &= ~(EXIU_EIEDG_MASK << shift);

	switch (type) {
	case IRQ_TYPE_LEVEL_HIGH:
		eilvl |= EXIU_EILVL_HIGH << shift;
		eiedg |= EXIU_EIEDG_LEVEL << shift;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		eilvl |= EXIU_EILVL_LOW << shift;
		eiedg |= EXIU_EIEDG_LEVEL << shift;
		break;
	case IRQ_TYPE_EDGE_RISING:
		eilvl |= EXIU_EILVL_HIGH << shift;
		eiedg |= EXIU_EIEDG_EDGE << shift;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		eilvl |= EXIU_EILVL_LOW << shift;
		eiedg |= EXIU_EIEDG_EDGE << shift;
		break;
	default:
		spin_unlock_irqrestore(&exiu_lock, flags);
		pr_err("%s(): Bad exiu irq type %lu.", __func__, type);
		return -EINVAL;
	}
	__raw_writel(eilvl, exiu_base + EXIU_REG_EILVL);
	__raw_writel(eiedg, exiu_base + EXIU_REG_EIEDG);
	spin_unlock_irqrestore(&exiu_lock, flags);

	return 0;
}
EXPORT_SYMBOL(exiu_irq_set_type);

#ifdef CONFIG_PM
static int exiu_irq_set_wake(struct irq_data *d, unsigned int on)
{
	return 0;
}
#endif

static int mb8ac0300_exiu_probe(struct platform_device *pdev)
{
	struct resource *res;

	spin_lock_init(&exiu_lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	exiu_base = ioremap(res->start, res->end - res->start + 1);
	if (!exiu_base) {
		dev_err(&pdev->dev, "unable to map mem region\n");
		return -EBUSY;
	}

	of_property_read_u32(pdev->dev.of_node, "exiu_irq_base0",
							&exiu_irq_base[0]);
	of_property_read_u32(pdev->dev.of_node, "exiu_irq_base1",
							&exiu_irq_base[1]);

	/* Disable all external interrupts */
	__raw_writel(EXIU_EIMASK_ALL, exiu_base + EXIU_REG_EIMASK);

	/* Set interrupt factor be external interrupt request */
	__raw_writel(EXIU_EISRCSEL_ALL_EXINT, exiu_base + EXIU_REG_EISRCSEL);

	/* Set all interrupts be edge triggered, active rising */
	__raw_writel(EXIU_EILVL_ALL_HIGH, exiu_base + EXIU_REG_EILVL);
	__raw_writel(EXIU_EIEDG_ALL_EDGE, exiu_base + EXIU_REG_EIEDG);

	/* Clear all interrupts */
	__raw_writel(EXIU_EIREQCLR_ALL, exiu_base + EXIU_REG_EIREQCLR);

	gic_arch_extn.irq_eoi = exiu_irq_eoi;
	gic_arch_extn.irq_mask = exiu_irq_mask;
	gic_arch_extn.irq_unmask = exiu_irq_unmask;
	/* use exiu_irq_set_type directly */
	gic_arch_extn.irq_set_type = NULL;
#ifdef CONFIG_PM
	gic_arch_extn.irq_set_wake = exiu_irq_set_wake;
#endif
	return 0;
}

static int mb8ac0300_exiu_remove(struct platform_device *pdev)
{
	__raw_writel(EXIU_EIMASK_ALL, exiu_base + EXIU_REG_EIMASK);

	iounmap(exiu_base);

	return 0;
}

#ifdef CONFIG_PM

int mb8ac0300_exiu_suspend_noirq(struct device *dev)
{
	int n;

	dev_info(dev, "saving exiu state\n");
	for (n = 0; n < EXIU_REG_EXTENT; n += 4)
		exiu_state[n >> 2] = readl(exiu_base + n);

        return 0;
}

int mb8ac0300_exiu_resume_noirq(struct device *dev)
{
	dev_info(dev, "restoring exiu state\n");

	writel(exiu_state[EXIU_REG_EILVL >> 2], exiu_base + EXIU_REG_EILVL);
	writel(exiu_state[EXIU_REG_EIEDG >> 2], exiu_base + EXIU_REG_EIEDG);
	writel(EXIU_EIREQCLR_ALL, exiu_base + EXIU_REG_EIREQCLR);
	writel(exiu_state[EXIU_REG_EIMASK >> 2], exiu_base + EXIU_REG_EIMASK);

        return 0;
}

static const struct dev_pm_ops mb8ac0300_exiu_pm_ops = {
	.suspend_noirq = mb8ac0300_exiu_suspend_noirq,
        .resume_noirq = mb8ac0300_exiu_resume_noirq,
};

#endif /* CONFIG_PM */

#ifdef CONFIG_OF
static const struct of_device_id mb8ac0300_exiu_dt_ids[] = {
	{ .compatible = "fujitsu,mb8ac0300-exiu" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, mb8ac0300_hdmac_dt_ids);
#else
#define mb8ac0300_hdmac_dt_ids NULL
#endif

static struct platform_driver mb8ac0300_exiu_driver = {
	.probe     = mb8ac0300_exiu_probe,
	.remove    = mb8ac0300_exiu_remove,
	.driver    = {
		.name  = DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &mb8ac0300_exiu_pm_ops,
#endif /* CONFIG_PM */
		.of_match_table = mb8ac0300_exiu_dt_ids,
	},
};

static int __init mb8ac0300_exiu_driver_init(void)
{
	return platform_driver_register(&mb8ac0300_exiu_driver);
}
subsys_initcall(mb8ac0300_exiu_driver_init);

MODULE_AUTHOR("Fujitsu Semiconductor Limited");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_LICENSE("GPL");
