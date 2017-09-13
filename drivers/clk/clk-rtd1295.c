/*
 * Realtek RTD1295
 *
 * Copyright (c) 2017 Andreas Färber
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <dt-bindings/clock/realtek,rtd1295.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

struct rtd_pll_clk {
	struct clk_hw hw;
	void __iomem *base;
	bool gpu;
};

#define to_pll_clk(_hw) container_of(_hw, struct rtd_pll_clk, hw)

static unsigned long rtd_scpu_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct rtd_pll_clk *pll = to_pll_clk(hw);
	u32 reg1, reg2, reg3, reg4;
	unsigned f1, f2, f3, f4, f5;
	unsigned long rate, frac;

	reg1 = readl(pll->base + 0x4);
	reg2 = readl(pll->base - (0x500 - 0x30));
	reg3 = readl(pll->base + 0x0);
	reg4 = readl(pll->base + 0x1c);
	f1 = (reg1 >> 11) & 0xff;
	f2 = (reg1 >> 0) & 0x7ff;
	f3 = (reg2 >> 7) & 0x3;
	f4 = (reg3 >> 0) & 0x1;
	f5 = (reg4 >> 20) & 0x1;

	rate = parent_rate * (f1 + 3) / f3;
	frac = parent_rate / 2048 * f2 / BIT(f4);
	rate += frac;

	pr_info("%s 0x%08x n=%u f=%u 0x%08x x=%u 0x%08x y=%u 0x%08x z=%u rate=%lu\n",
		__clk_get_name(hw->clk),
		reg1, f1, f2,
		reg2, f3,
		reg3, f4,
		reg4, f5, rate);
	return rate;
}

static const struct clk_ops rtd_scpu_ops = {
	.recalc_rate = rtd_scpu_recalc_rate,
};

static struct clk *rtd_scpu(void __iomem *base, const char *name, struct clk *parent)
{
	struct rtd_pll_clk *pll;
	struct clk_init_data init;
	struct clk *clk;
	const char *parents[1];

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	if (parent)
		parents[0] = __clk_get_name(parent);
	init.name = name;
	init.ops = &rtd_scpu_ops;
	init.parent_names = parent ? parents : NULL;
	init.num_parents = parent ? 1 : 0;
	init.flags = CLK_IGNORE_UNUSED;

	pll->hw.init = &init;
	pll->base = base;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk)) {
		pr_err("%s: error registering clk", name);
		kfree(pll);
	}
	return clk;
}

static unsigned long rtd_nf_ssc_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct rtd_pll_clk *pll = to_pll_clk(hw);
	u32 reg1, reg2, reg3;
	unsigned f1, f2, f3, f4;
	unsigned long rate, frac;

	reg1 = readl(pll->base + 0x4);
	reg2 = readl(pll->base + 0x0);
	reg3 = readl(pll->base + 0x1c);
	f1 = (reg1 >> 11) & 0xff;
	f2 = (reg1 >> 0) & 0x7ff;
	f3 = (reg2 >> 0) & 0xf;
	f4 = (reg3 >> 20) & 0x1;

	rate = parent_rate * (f1 + 3);
	if (pll->gpu) {
		rate /= 2;
	}
	frac = parent_rate * 4 * f2 / BIT(f3);
	if (pll->gpu) {
		frac /= 2;
	}
	rate += frac;

	pr_info("%s 0x%08x n=%u f=%u 0x%08x d=%u 0x%08x x=%u rate=%lu\n", __clk_get_name(hw->clk),
		reg1, f1, f2,
		reg2, f3,
		reg3, f4, rate);
	return rate;
}

static const struct clk_ops rtd_nf_ssc_ops = {
	.recalc_rate = rtd_nf_ssc_recalc_rate,
};

static struct clk *rtd_nf_ssc(void __iomem *base, const char *name, struct clk *parent)
{
	struct rtd_pll_clk *pll;
	struct clk_init_data init;
	struct clk *clk;
	const char *parents[1];

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	if (parent)
		parents[0] = __clk_get_name(parent);
	init.name = name;
	init.ops = &rtd_nf_ssc_ops;
	init.parent_names = parent ? parents : NULL;
	init.num_parents = parent ? 1 : 0;
	init.flags = CLK_IGNORE_UNUSED;

	pll->hw.init = &init;
	pll->base = base;
	pll->gpu = (strcmp(name, "pll_gpu") == 0);

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk)) {
		pr_err("%s: error registering clk", name);
		kfree(pll);
	}
	return clk;
}

static unsigned long rtd_mno_ctrl_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct rtd_pll_clk *pll = to_pll_clk(hw);
	u32 reg1, reg2;
	unsigned f1, f2, f3;
	unsigned long rate;

	reg1 = readl(pll->base + 0x0);
	reg2 = readl(pll->base + 0x4);
	f1 = (reg1 >> 4) & 0xff;
	f2 = (reg1 >> 12) & 0x3;
	f3 = (reg1 >> 17) & 0x3;

	rate = parent_rate * (f1 + 2) / (f2 +1) / (f3 + 1);

	pr_info("%s 0x%08x m=%u n=%u o=%u 0x%08x rate=%lu\n", __clk_get_name(hw->clk),
		reg1, f1, f2, f3,
		reg2, rate);
	return rate;
}

static const struct clk_ops rtd_mno_ctrl_ops = {
	.recalc_rate = rtd_mno_ctrl_recalc_rate,
};

static struct clk *rtd_mno_ctrl(void __iomem *base, const char *name, struct clk *parent)
{
	struct rtd_pll_clk *pll;
	struct clk_init_data init;
	struct clk *clk;
	const char *parents[1];

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	if (parent)
		parents[0] = __clk_get_name(parent);
	init.name = name;
	init.ops = &rtd_mno_ctrl_ops;
	init.parent_names = parent ? parents : NULL;
	init.num_parents = parent ? 1 : 0;
	init.flags = CLK_IGNORE_UNUSED;

	pll->hw.init = &init;
	pll->base = base;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk)) {
		pr_err("%s: error registering clk", name);
		kfree(pll);
	}
	return clk;
}

static const char * const rtd1295_gates1[32] = {
	[ 0] = "clk_en_misc",
	[ 1] = "clk_en_pcie0",
	[ 2] = "clk_en_sata_0",
	[ 3] = "clk_en_gspi",
	[ 4] = "clk_en_usb",
	[ 5] = "clk_en_pcr",
	[ 6] = "clk_en_iso_misc",
	[ 7] = "clk_en_sata_alive_0",
	[ 8] = "clk_en_hdmi",
	[ 9] = "clk_en_etn",
	[10] = "clk_en_aio",
	/* "*clk_en_gpu", */
	/* "*clk_en_ve1", */
	/* "*clk_en_ve2", */
	[14] = "clk_en_tve",
	/* "*clk_en_vo", */
	[16] = "clk_en_lvds",
	[17] = "clk_en_se",
	[18] = "clk_en_dcu",
	[19] = "clk_en_cp",
	[20] = "clk_en_md",
	[21] = "clk_en_tp",
	[22] = "clk_en_rsa",
	[23] = "clk_en_nf",
	[24] = "clk_en_emmc",
	[25] = "clk_en_cr",
	[26] = "clk_en_sdio_ip",
	[27] = "clk_en_mipi",
	[28] = "clk_en_emmc_ip",
	/* "*clk_en_ve3", */
	[30] = "clk_en_sdio",
	[31] = "clk_en_sd_ip",
};

static const char * const rtd1295_gates2[32] = {
	[ 0] = "clk_en_nat",
	[ 1] = "clk_en_misc_i2c_5",
	/* "*clk_en_scpu", */
	[ 3] = "clk_en_jpeg",
	/* "*clk_en_apu", */
	[ 5] = "clk_en_pcie1",
	[ 6] = "clk_en_misc_sc",
	[ 7] = "clk_en_cbus_tx",
	/* "*rvd", */
	/* "*rvd", */
	[10] = "clk_en_misc_rtc",
	/* "*rvd", */
	/* "*rvd", */
	[13] = "clk_en_misc_i2c_4",
	[14] = "clk_en_misc_i2c_3",
	[15] = "clk_en_misc_i2c_2",
	[16] = "clk_en_misc_i2c_1",
	[17] = "clk_en_aio_au_codec",
	[18] = "clk_en_aio_mod",
	[19] = "clk_en_aio_da",
	[20] = "clk_en_aio_hdmi",
	[21] = "clk_en_aio_spdif",
	[22] = "clk_en_aio_i2s",
	[23] = "clk_en_aio_mclk",
	[24] = "clk_en_hdmirx",
	[25] = "clk_en_sata_1",
	[26] = "clk_en_sata_alive_1",
	[27] = "clk_en_ur2",
	[28] = "clk_en_ur1",
	[29] = "clk_en_fan",
	[30] = "clk_en_dcphy_0",
	[31] = "clk_en_dcphy_1",
};

static struct clk *clks[16 + 2 * 32] = {};

static struct clk_onecell_data rtd_clks = {
	.clks = clks,
	.clk_num = ARRAY_SIZE(clks),
};

static void __init rtd1295_clk_init(struct device_node *node)
{
	void __iomem *base;
	struct clk *osc;
	int i;
	static const char *clk_sys_parents[2] = { "pll_bus", "pll_bus_div2" };
	static const char *clk_ve_parents[4] = { "clk_sysh", "pll_ve1", "pll_ve2", "pll_ve2" };

	base = of_iomap(node, 0);

	osc = of_clk_get(node, 0);

	clks[RTD1295_CLK_PLL_SCPU] = rtd_scpu(base + 0x500, "pll_scpu", osc);
	clks[RTD1295_CLK_PLL_BUS] = rtd_nf_ssc(base + 0x520, "pll_bus", osc);
	clks[RTD1295_CLK_PLL_BUS_DIV2] = clk_register_fixed_factor(NULL, "pll_bus_div2", "pll_bus", 0, 1, 2);
	clks[RTD1295_CLK_SYS] = clk_register_mux(NULL, "clk_sys", clk_sys_parents, 2, 0, base + 0x30, 0, 1, CLK_MUX_READ_ONLY, NULL);
	clks[RTD1295_CLK_PLL_BUS_H] = rtd_nf_ssc(base + 0x540, "pll_bus_h", osc);
	clks[RTD1295_CLK_SYSH] = clk_register_fixed_factor(NULL, "clk_sysh", "pll_bus_h", 0, 1, 1);
	clks[RTD1295_CLK_PLL_DDSA] = rtd_nf_ssc(base + 0x560, "pll_ddsa", osc);
	clks[RTD1295_CLK_PLL_DDSB] = rtd_nf_ssc(base + 0x580, "pll_ddsb", osc);
	clks[RTD1295_CLK_PLL_VODMA] = rtd_mno_ctrl(base + 0x260, "pll_vodma", osc);
	clk_register_fixed_factor(NULL, "clk_vodma", "pll_vodma", 0, 1, 1);
	clks[RTD1295_CLK_EN_VO] = clk_register_gate(NULL, "clk_en_vo", "clk_vodma", CLK_IGNORE_UNUSED, base + 0xc, 15, 0, NULL);
	clks[RTD1295_CLK_PLL_VE1] = rtd_mno_ctrl(base + 0x114, "pll_ve1", osc);
	clks[RTD1295_CLK_PLL_VE2] = rtd_mno_ctrl(base + 0x1d0, "pll_ve2", osc);
	clk_register_mux(NULL, "clk_ve1", clk_ve_parents, 4, 0, base + 0x4c, 0, 2, CLK_MUX_READ_ONLY, NULL);
	clks[RTD1295_CLK_EN_VE1] = clk_register_gate(NULL, "clk_en_ve1", "clk_ve1", CLK_IGNORE_UNUSED, base + 0xc, 12, 0, NULL);
	clk_register_mux(NULL, "clk_ve2", clk_ve_parents, 4, 0, base + 0x4c, 2, 2, CLK_MUX_READ_ONLY, NULL);
	clks[RTD1295_CLK_EN_VE2] = clk_register_gate(NULL, "clk_en_ve2", "clk_ve2", CLK_IGNORE_UNUSED, base + 0xc, 13, 0, NULL);
	clk_register_mux(NULL, "clk_ve3", clk_ve_parents, 4, 0, base + 0x4c, 4, 2, CLK_MUX_READ_ONLY, NULL);
	clks[RTD1295_CLK_EN_VE3] = clk_register_gate(NULL, "clk_en_ve3", "clk_ve3", CLK_IGNORE_UNUSED, base + 0xc, 29, 0, NULL);
	clks[RTD1295_CLK_PLL_GPU] = rtd_nf_ssc(base + 0x5a0, "pll_gpu", osc);
	clk_register_fixed_factor(NULL, "clk_gpu", "pll_gpu", 0, 1, 1);
	clks[RTD1295_CLK_EN_GPU] = clk_register_gate(NULL, "clk_en_gpu", "clk_gpu", CLK_IGNORE_UNUSED, base + 0xc, 11, 0, NULL);
	clks[RTD1295_CLK_PLL_ACPU] = rtd_nf_ssc(base + 0x5c0, "pll_acpu", osc);

	for (i = 0; i < ARRAY_SIZE(rtd1295_gates1); i++) {
		if (!rtd1295_gates1[i])
			continue;
		clks[RTD1295_CLK_EN_BASE + i] = clk_register_gate(NULL, rtd1295_gates1[i], NULL, CLK_IGNORE_UNUSED, base + 0xc, i, 0, NULL);
	}

	for (i = 0; i < ARRAY_SIZE(rtd1295_gates2); i++) {
		if (!rtd1295_gates2[i])
			continue;
		clks[RTD1295_CLK_EN_BASE2 + i] = clk_register_gate(NULL, rtd1295_gates2[i], __clk_get_name(osc), CLK_IGNORE_UNUSED, base + 0x10, i, 0, NULL);
	}

	clk_put(osc);

	of_clk_add_provider(node, of_clk_src_onecell_get, &rtd_clks);
}
CLK_OF_DECLARE(rtd1295, "realtek,rtd1295-clk", rtd1295_clk_init);

static const char * const rtd1295_iso_gates[13] = {
	/* "*unused", */
	/* "*rvd", */
	[ 2] = "clk_en_misc_cec0",
	[ 3] = "clk_en_cbusrx_sys",
	[ 4] = "clk_en_cbustx_sys",
	[ 5] = "clk_en_cbus_sys",
	[ 6] = "clk_en_cbus_osc",
	[ 7] = "clk_en_misc_ir",
	[ 8] = "clk_en_misc_ur0",
	[ 9] = "clk_en_i2c0",
	[10] = "clk_en_i2c1",
	[11] = "clk_en_etn_250m",
	[12] = "clk_en_etn_sys",
};

static struct clk *iso_clks[13] = {};

static struct clk_onecell_data rtd_iso_clks = {
	.clks = iso_clks,
	.clk_num = ARRAY_SIZE(iso_clks),
};

static void __init rtd1295_iso_clk_init(struct device_node *node)
{
	void __iomem *base;
	struct clk *osc;
	int i;

	base = of_iomap(node, 0);

	osc = of_clk_get(node, 0);

	pr_info("I2C1_SDA_DEL %08x\n", readl_relaxed(base + 0x80));
	pr_info("I2C0_SDA_DEL %08x\n", readl_relaxed(base + 0x84));

	for (i = 0; i < ARRAY_SIZE(rtd1295_iso_gates); i++) {
		if (!rtd1295_iso_gates[i])
			continue;
		iso_clks[i] = clk_register_gate(NULL, rtd1295_iso_gates[i], __clk_get_name(osc), CLK_IGNORE_UNUSED, base + 0x8c, i, 0, NULL);
	}

	clk_put(osc);

	of_clk_add_provider(node, of_clk_src_onecell_get, &rtd_iso_clks);
}
CLK_OF_DECLARE(rtd1295_iso, "realtek,rtd1295-iso-clk", rtd1295_iso_clk_init);
