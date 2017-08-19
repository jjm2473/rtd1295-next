/*
 * Realtek RTD129x SCPU Wrapper
 *
 * Copyright (c) 2017 Andreas Färber
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#define DBG_START	0x200
#define DBG_END		0x210
#define DBG_CTRL	0x220
#define DBG_INT		0x230
#define DBG_ADDR	0x234
#define DBG_ADDR1	0x238

#define DBG_CTRL_DBG_EN_M1		BIT(0)
#define DBG_CTRL_WRITE_EN1		BIT(1)
#define DBG_CTRL_WRITE_EN2		BIT(4)

#define DBG_INT_WRITE_DATA		BIT(0)
#define DBG_INT_SCPU_NEG_INT_EN_M1	BIT(1)
#define DBG_INT_SCPU_NEG_INT_M1		BIT(2)
#define DBG_INT_SCPU_INT_EN_M1		BIT(3)
#define DBG_INT_SCPU_INT_M1		BIT(4)

#define DBG_ADDR1_SCPU_DBG_WRITE_M1	BIT(0)

static void rtd119x_scpu_dbg_disable_mem_monitor(void __iomem *base, int which)
{
	writel(BIT(13) | BIT(9) | DBG_CTRL_WRITE_EN1, base + DBG_CTRL + which * 4);
}

static void rtd119x_scpu_dbg_set_mem_monitor(void __iomem *base, int which, u32 start, u32 end, u32 flags)
{
	rtd119x_scpu_dbg_disable_mem_monitor(base, which);

	writel(start, base + DBG_START + which * 4);
	writel(end,   base + DBG_END + which * 4);
	writel(flags, base + DBG_CTRL + which * 4);
}

static void rtd119x_scpu_dbg_scpu_monitor(void __iomem *base, int which, u32 start, u32 end, u32 r_w)
{
	rtd119x_scpu_dbg_set_mem_monitor(base, which, start, end,
		(0x3 << 8) | r_w | DBG_CTRL_WRITE_EN1 | DBG_CTRL_DBG_EN_M1);
}

static irqreturn_t rtd119x_scpu_handle_irq(int irq, void *base)
{
	u32 dbg_int, dbg_addr, dbg_addr1;

	dbg_int = readl(base + DBG_INT);
	dbg_addr = readl(base + DBG_ADDR);
	dbg_addr1 = readl(base + DBG_ADDR1);
	pr_debug("%s: DBG_INT 0x%08x\n", __func__, dbg_int);
	if (dbg_int & (DBG_INT_SCPU_INT_M1 | DBG_INT_SCPU_NEG_INT_M1)) {
		writel(dbg_int & ~(DBG_INT_SCPU_INT_EN_M1 | DBG_INT_SCPU_NEG_INT_EN_M1), base + DBG_INT);
		pr_err("%s: SCPU addr 0x%08x mode %s", __func__,
			dbg_addr,
			(dbg_addr1 & DBG_ADDR1_SCPU_DBG_WRITE_M1) ? "W" : "R");
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static const struct of_device_id rtd119x_scpu_dt_ids[] = {
	 { .compatible = "realtek,rtd1195-scpu-wrapper" },
	 { .compatible = "realtek,rtd1295-scpu-wrapper" },
	 { }
};

static int rtd119x_scpu_probe(struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *base;
	unsigned long addr = 0;
	int irq, i, ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -EINVAL;

	ret = request_irq(irq, rtd119x_scpu_handle_irq, IRQF_SHARED, "scpu_wrapper", base);
	if (ret) {
		dev_err(&pdev->dev, "requesting irq %u failed\n", irq);
		return -EINVAL;
	}

	for (i = 0; i < 4; i++) {
		writel_relaxed(0, base + DBG_START + i * 4);
		writel_relaxed(0, base + DBG_END   + i * 4);
	}

	if (of_device_is_compatible(pdev->dev.of_node, "realtek,rtd1295-scpu-wrapper"))
		addr = 0x98000000;
	else if (of_device_is_compatible(pdev->dev.of_node, "realtek,rtd1195-scpu-wrapper"))
		addr = 0x18000000;

	for (i = 0; i < 4; i++) {
		writel_relaxed(addr,        base + DBG_START + i * 4);
		writel_relaxed(addr + 0x40, base + DBG_END   + i * 4);
		addr += 0x40;
	}

	writel(DBG_INT_SCPU_INT_EN_M1 | DBG_INT_WRITE_DATA, base + DBG_INT);

	if (of_device_is_compatible(pdev->dev.of_node, "realtek,rtd1295-scpu-wrapper"))
		rtd119x_scpu_dbg_scpu_monitor(base, 0, 0x98013b00, 0x98013c00, 0);

	dev_info(&pdev->dev, "probed\n");

	return 0;
}

static struct platform_driver rtd119x_scpu_driver = {
	.probe = rtd119x_scpu_probe,
	.driver = {
		.name = "rtd1295-scpu-wrapper",
		.of_match_table	= rtd119x_scpu_dt_ids,
	},
};
//builtin_platform_driver(rtd119x_scpu_driver);

static int __init rtd119x_scpu_init(void)
{
	return platform_driver_register(&rtd119x_scpu_driver);
}
late_initcall(rtd119x_scpu_init);
