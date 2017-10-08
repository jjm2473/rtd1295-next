/*
 * Realtek RTD1195
 *
 * Copyright (c) 2017 Andreas Färber
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <dt-bindings/clock/realtek,rtd1195.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

static const char * const rtd1195_gates1[32] = {
	"clk_en_misc",
	"clk_en_hdmirx",
	NULL,
	"clk_en_gspi",
	"clk_en_usb",
	"clk_en_pcr",
	"clk_en_iso_misc",
	NULL,
	"clk_en_hdmi",
	"clk_en_etn",
	"clk_en_aio",
	"clk_en_gpu",
	"clk_en_ve_h264",
	"clk_en_ve_jpeg",
	"clk_en_tve",
	"clk_en_vo",
	"clk_en_lvds",
	"clk_en_se",
	"clk_en_dcu",
	"clk_en_cp",
	"clk_en_md",
	"clk_en_tp",
	NULL,
	"clk_en_nf",
	"clk_en_emmc",
	"clk_en_cr",
	NULL,
	"clk_en_mipi",
	NULL,
	"clk_en_ve_h265",
	"clk_en_sdio",
	NULL,
};

static const char * const rtd1195_gates2[32] = {
	/* reserved */
	[ 1] = "clk_en_misc_i2c_5",
	[ 2] = "clk_en_scpu",
	/* reserved */
	[ 4] = "clk_en_acpu",
	[ 5] = "clk_en_vcpu",
	/* reserved */
	[10] = "clk_en_misc_rtc",
	/* reserved */
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
	/* reserved */
	[28] = "clk_en_ur1",
	/* reserved */
};

static struct clk *clks[2 * 32] = {};

static struct clk_onecell_data rtd119x_clks = {
	.clks = clks,
	.clk_num = ARRAY_SIZE(clks),
};

static void __init rtd1195_clk_init(struct device_node *node)
{
	void __iomem *base;
	struct clk *osc;
	int i;

	base = of_iomap(node, 0);

	osc = of_clk_get(node, 0);

	for (i = 0; i < ARRAY_SIZE(rtd1195_gates1); i++) {
		if (!rtd1195_gates1[i])
			continue;
		clks[RTD1195_CLK_EN_BASE + i] = clk_register_gate(NULL, rtd1195_gates1[i], __clk_get_name(osc), CLK_IGNORE_UNUSED, base + 0xc, i, 0, NULL);
	}

	for (i = 0; i < ARRAY_SIZE(rtd1195_gates2); i++) {
		if (!rtd1195_gates2[i])
			continue;
		clks[RTD1195_CLK_EN_BASE2 + i] = clk_register_gate(NULL, rtd1195_gates2[i], __clk_get_name(osc), CLK_IGNORE_UNUSED, base + 0x10, i, 0, NULL);
	}

	clk_put(osc);

	of_clk_add_provider(node, of_clk_src_onecell_get, &rtd119x_clks);
}
CLK_OF_DECLARE(rtd1195, "realtek,rtd1195-clk", rtd1195_clk_init);

static const char * const rtd1195_iso_gates[13] = {
	"clk_en_misc_mix",
	"clk_en_misc_vfd",
	"clk_en_misc_cec0",
	"clk_en_cbusrx_sys",
	"clk_en_cbustx_sys",
	"clk_en_cbus_sys",
	"clk_en_cbus_osc",
	"clk_en_misc_ir",
	"clk_en_misc_ur0",
	"clk_en_i2c0",
	"clk_en_i2c6",
	"clk_en_etn_250m",
	"clk_en_etn_sys",
};

static struct clk *iso_clks[13] = {};

static struct clk_onecell_data rtd119x_iso_clks = {
	.clks = iso_clks,
	.clk_num = ARRAY_SIZE(iso_clks),
};

static void __init rtd1195_iso_clk_init(struct device_node *node)
{
	void __iomem *base;
	struct clk *osc;
	int i;

	base = of_iomap(node, 0);

	osc = of_clk_get(node, 0);

	for (i = 0; i < ARRAY_SIZE(rtd1195_iso_gates); i++) {
		if (!rtd1195_iso_gates[i])
			continue;
		iso_clks[i] = clk_register_gate(NULL, rtd1195_iso_gates[i], __clk_get_name(osc), CLK_IGNORE_UNUSED, base + 0x8c, i, 0, NULL);
	}

	clk_put(osc);

	of_clk_add_provider(node, of_clk_src_onecell_get, &rtd119x_iso_clks);
}
CLK_OF_DECLARE(rtd1195_iso, "realtek,rtd1195-iso-clk", rtd1195_iso_clk_init);
