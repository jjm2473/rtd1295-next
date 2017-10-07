// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Realtek RTD1195
 *
 * Copyright (c) 2017-2019 Andreas Färber
 */

#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <linux/io.h>
#include <linux/memblock.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <asm/mach/arch.h>

#define REG_WRAP_CTRL	0x000

#define WRAP_CTRL_WCMD_BUFABLE_SEL	GENMASK(13, 12)

#define REG_SB2_SYNC	0x020

static void __init rtd1195_memblock_remove(phys_addr_t base, phys_addr_t size)
{
	int ret;

	ret = memblock_remove(base, size);
	if (ret)
		pr_err("Failed to remove memblock %pa (%d)\n", &base, ret);
}

static void __init rtd1195_reserve(void)
{
	/* Exclude boot ROM from RAM */
	rtd1195_memblock_remove(0x00000000, 0x0000a800);

	/* Exclude peripheral register spaces from RAM */
	rtd1195_memblock_remove(0x18000000, 0x00070000);
	rtd1195_memblock_remove(0x18100000, 0x01000000);
}

static void __init rtd1195_init_time(void)
{
	void __iomem *base;

	base = ioremap(0xff018000, 4);
	writel_relaxed(0x1, base);
	iounmap(base);

	of_clk_init(NULL);
	timer_probe();
}

static void __init rtd1195_init_machine(void)
{
	struct device_node *node;
	void __iomem *base;
	u32 val;

	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);

	node = of_find_compatible_node(NULL, NULL, "realtek,rtd1195-scpu-wrapper");
	if (!node) {
		pr_err("%s: missing SCPU wrapper\n", __func__);
		return;
	}

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("%s: could not map SCPU wrapper registers\n", __func__);
		return;
	}

	val = readl(base + REG_WRAP_CTRL);
	val &= ~WRAP_CTRL_WCMD_BUFABLE_SEL;
	val |= FIELD_PREP(WRAP_CTRL_WCMD_BUFABLE_SEL, 0x1);
	writel(val, base + REG_WRAP_CTRL);

	iounmap(base);

	node = of_find_compatible_node(NULL, NULL, "realtek,rtd1195-sb2");
	if (!node) {
		pr_err("%s: missing SB2\n", __func__);
		return;
	}

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("%s: could not map SB2 registers\n", __func__);
		return;
	}

	writel(0x1234, base + REG_SB2_SYNC);

	iounmap(base);
}

static const char *const rtd1195_dt_compat[] __initconst = {
	"realtek,rtd1195",
	NULL
};

DT_MACHINE_START(rtd1195, "Realtek RTD1195")
	.dt_compat = rtd1195_dt_compat,
	.init_machine = rtd1195_init_machine,
	.init_time = rtd1195_init_time,
	.reserve = rtd1195_reserve,
	.l2c_aux_val = 0x0,
	.l2c_aux_mask = ~0x0,
MACHINE_END
