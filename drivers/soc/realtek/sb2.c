#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#define SB2_DBG_ADDR_AUDIO	0x4b8
#define SB2_DBG_ADDR_SYSTEM	0x4c0
#define SB2_DBG_ADDR1		0x4c8
#define SB2_DBG_INT		0x4e0

#define SB2_DBG_INT_WRITE_DATA		BIT(0)
#define SB2_DBG_INT_SCPU_NEG_INT	BIT(4)
#define SB2_DBG_INT_ACPU_NEG_INT	BIT(6)
#define SB2_DBG_INT_SCPU_INT_EN		BIT(7)
#define SB2_DBG_INT_ACPU_INT_EN		BIT(9)
#define SB2_DBG_INT_SCPU_INT		BIT(10)
#define SB2_DBG_INT_ACPU_INT		BIT(12)

static irqreturn_t rtd_sb2_handle_irq(int irq, void *base)
{
	u32 val;

	val = readl_relaxed(base + SB2_DBG_INT);
	pr_info("sb2 interrupt 0x%0x08x\n", val);
	if (val & (SB2_DBG_INT_ACPU_INT | SB2_DBG_INT_SCPU_INT | SB2_DBG_INT_ACPU_NEG_INT | SB2_DBG_INT_SCPU_NEG_INT)) {
		u32 addr, cause;

		writel_relaxed(SB2_DBG_INT_ACPU_INT_EN | SB2_DBG_INT_SCPU_INT_EN | SB2_DBG_INT_WRITE_DATA, base + SB2_DBG_INT);

		cause = readl_relaxed(base + SB2_DBG_ADDR1);

		if (val & SB2_DBG_INT_SCPU_INT) {
			addr = readl_relaxed(base + SB2_DBG_ADDR_SYSTEM);
			cause = (cause >> 2) & 0x3;
		} else {
			addr = readl_relaxed(base + SB2_DBG_ADDR_AUDIO);
			cause = (cause >> 4) & 0x3;
		}

		pr_warn("Memory 0x%08x trashed by %s with %s %s\n", addr,
			(val & SB2_DBG_INT_SCPU_INT) ? "SCPU" : "ACPU",
			(cause & 1) ? "D" : "I",
			(cause & 2) ? "W" : "R");

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static const struct of_device_id rtd_sb2_dt_ids[] = {
	 { .compatible = "realtek,rtd1195-sb2" },
	 { .compatible = "realtek,rtd1295-sb2" },
	 { }
};

static int rtd_sb2_probe(struct platform_device *pdev)
{
	void __iomem *base;
	int irq, ret;

	base = of_iomap(pdev->dev.of_node, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -EINVAL;

	ret = request_irq(irq, rtd_sb2_handle_irq, IRQF_SHARED, "sb2", base);
	if (ret) {
		dev_err(&pdev->dev, "requesting irq %u failed\n", irq);
		return -EINVAL;
	}

	writel_relaxed(SB2_DBG_INT_ACPU_INT_EN | SB2_DBG_INT_SCPU_INT_EN | SB2_DBG_INT_WRITE_DATA, base + SB2_DBG_INT);

	dev_info(&pdev->dev, "probed\n");

	return 0;
}

static struct platform_driver rtd_sb2_driver = {
	.probe = rtd_sb2_probe,
	.driver = {
		.name = "rtd1295-sb2",
		.of_match_table	= rtd_sb2_dt_ids,
	},
};
builtin_platform_driver(rtd_sb2_driver);
