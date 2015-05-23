/*
 * XMC4500 SCU RCU
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/spinlock.h>

struct xmc4500_reset_controller_dev {
	void __iomem *regs;
	spinlock_t lock;
	struct reset_controller_dev rcdev;
};

#define to_xmc_reset(_rcdev) \
	container_of(_rcdev, struct xmc4500_reset_controller_dev, rcdev)

static int xmc4500_reset_status(struct reset_controller_dev *rcdev,
			        unsigned long id)
{
	struct xmc4500_reset_controller_dev *s = to_xmc_reset(rcdev);
	int bank = id / BITS_PER_LONG;
	int offset = id % BITS_PER_LONG;
	u32 reg;

	reg = readl_relaxed(s->regs + 0x0C + bank * 3 * 4);

	return !!(reg & BIT(offset));
}

static int xmc4500_reset_assert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	struct xmc4500_reset_controller_dev *s = to_xmc_reset(rcdev);
	int bank = id / BITS_PER_LONG;
	int offset = id % BITS_PER_LONG;
	unsigned long flags;

	spin_lock_irqsave(&s->lock, flags);

	writel_relaxed(BIT(offset), s->regs + 0x10 + bank * 3 * 4);

	spin_unlock_irqrestore(&s->lock, flags);

	return 0;
}

static int xmc4500_reset_deassert(struct reset_controller_dev *rcdev,
				  unsigned long id)
{
	struct xmc4500_reset_controller_dev *s = to_xmc_reset(rcdev);
	int bank = id / BITS_PER_LONG;
	int offset = id % BITS_PER_LONG;
	unsigned long flags;

	spin_lock_irqsave(&s->lock, flags);

	writel_relaxed(BIT(offset), s->regs + 0x14 + bank * 3 * 4);

	spin_unlock_irqrestore(&s->lock, flags);

	return 0;
}

static int xmc4500_reset(struct reset_controller_dev *rcdev, unsigned long id)
{
	if (!xmc4500_reset_status(rcdev, id))
		xmc4500_reset_assert(rcdev, id);

	xmc4500_reset_deassert(rcdev, id);

	return 0;
}

static struct reset_control_ops xmc4500_reset_ops = {
	.assert		= xmc4500_reset_assert,
	.deassert	= xmc4500_reset_deassert,
	.status		= xmc4500_reset_status,
	.reset		= xmc4500_reset,
};

static int xmc4500_reset_probe(struct platform_device *pdev)
{
	struct xmc4500_reset_controller_dev *s;
	struct resource *res;

	s = devm_kzalloc(&pdev->dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	s->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(s->regs))
		return PTR_ERR(s->regs);

	spin_lock_init(&s->lock);

	s->rcdev.owner = THIS_MODULE;
	s->rcdev.nr_resets = 4 * BITS_PER_LONG;
	s->rcdev.ops = &xmc4500_reset_ops;
	s->rcdev.of_node = pdev->dev.of_node;
	reset_controller_register(&s->rcdev);

	return 0;
}

static const struct of_device_id xmc4500_reset_of_match_table[] = {
	{ .compatible = "infineon,xmc4500-scu-rcu" },
	{ },
};

static struct platform_driver xmc4500_reset_driver = {
	.probe = xmc4500_reset_probe,
	.driver = {
		.name = "xmc4500-scu-reset",
		.of_match_table = xmc4500_reset_of_match_table,
	},
};
module_platform_driver(xmc4500_reset_driver);
