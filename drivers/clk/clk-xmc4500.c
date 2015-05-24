/*
 * XMC4500
 */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

struct xmc4500_gate_clk {
	struct clk_hw hw;
	void __iomem *base;
	u8 bit_idx;
};

#define to_xmc_gate_clk(_hw) container_of(_hw, struct xmc4500_gate_clk, hw)

static int xmc4500_gate_clk_is_enabled(struct clk_hw *hw)
{
	struct xmc4500_gate_clk *xmcclk = to_xmc_gate_clk(hw);
	u32 reg;

	reg = clk_readl(xmcclk->base);

	return reg & BIT(xmcclk->bit_idx) ? 1 : 0;
}

static int xmc4500_gate_clk_enable(struct clk_hw *hw)
{
	struct xmc4500_gate_clk *xmcclk = to_xmc_gate_clk(hw);

	clk_writel(BIT(xmcclk->bit_idx), xmcclk->base + 0x4);

	return 0;
}

static void xmc4500_gate_clk_disable(struct clk_hw *hw)
{
	struct xmc4500_gate_clk *xmcclk = to_xmc_gate_clk(hw);

	clk_writel(BIT(xmcclk->bit_idx), xmcclk->base + 0x8);
}

static const struct clk_ops xmc4500_gate_clk_ops = {
	.enable		= xmc4500_gate_clk_enable,
	.disable	= xmc4500_gate_clk_disable,
	.is_enabled	= xmc4500_gate_clk_is_enabled,
};

static struct clk *xmc4500_register_gate_clk(const char *name, const char *parent_name,
	void __iomem *reg_base, u8 bit_idx)
{
	struct xmc4500_gate_clk *xmcclk;
	struct clk_init_data init;
	struct clk *clk;

	xmcclk = kzalloc(sizeof(*xmcclk), GFP_KERNEL);
	if (!xmcclk) {
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &xmc4500_gate_clk_ops;
	init.flags = 0;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	xmcclk->base = reg_base;
	xmcclk->bit_idx = bit_idx;
	xmcclk->hw.init = &init;

	clk = clk_register(NULL, &xmcclk->hw);
	if (IS_ERR(clk)) {
		pr_err("could not register clk %s\n", name);
		kfree(xmcclk);
		return clk;
	}

	return clk;
}

#define to_xmc_pllusb_clk(_hw) container_of(_hw, struct xmc4500_pllusb_clk, hw)

struct xmc4500_pllusb_clk {
	struct clk_hw hw;
	void __iomem *base;
};

static unsigned long xmc4500_pllusb_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct xmc4500_pllusb_clk *xmcclk = to_xmc_pllusb_clk(hw);
	u32 usbpllstat, usbpllcon;
	int ndiv, pdiv;
	unsigned long rate;

	usbpllstat = readl_relaxed(xmcclk->base + 0x120);
	usbpllcon = readl_relaxed(xmcclk->base + 0x124);

	ndiv = ((usbpllcon & 0x7f) >> 8) + 1;
	pdiv = ((usbpllcon & 0xf) >> 24) + 1;

	rate = parent_rate * ndiv / pdiv / 2;
	pr_info("PLLUSB normal mode: %lu (parent %lu)\n", rate, parent_rate);
	return rate;
}

static const struct clk_ops xmc4500_pllusb_clk_ops = {
	.recalc_rate	= xmc4500_pllusb_clk_recalc_rate,
};

static struct clk *xmc4500_register_pllusb_clk(const char *name, const char *parent_name,
	void __iomem *base)
{
	struct xmc4500_pllusb_clk *xmcclk;
	struct clk_init_data init;
	struct clk *clk;

	xmcclk = kzalloc(sizeof(*xmcclk), GFP_KERNEL);
	if (!xmcclk)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &xmc4500_pllusb_clk_ops;
	init.flags = 0;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	xmcclk->base = base;
	xmcclk->hw.init = &init;

	clk = clk_register(NULL, &xmcclk->hw);
	if (IS_ERR(clk)) {
		pr_err("could not register clk %s\n", name);
		kfree(xmcclk);
		return clk;
	}

	pr_info("%s: PLLUSB with parent %s\n", name, parent_name ? parent_name : "(null)");

	return clk;
}

#define to_xmc_pll_clk(_hw) container_of(_hw, struct xmc4500_pll_clk, hw)

struct xmc4500_pll_clk {
	struct clk_hw hw;
	void __iomem *base;
};

static unsigned long xmc4500_pll_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct xmc4500_pll_clk *xmcclk = to_xmc_pll_clk(hw);
	u32 pllstat, pllcon0, pllcon1, pllcon2;
	int k1div, ndiv, k2div, pdiv;
	unsigned long rate;

	pllstat = readl_relaxed(xmcclk->base + 0x110);
	pllcon0 = readl_relaxed(xmcclk->base + 0x114);
	pllcon1 = readl_relaxed(xmcclk->base + 0x118);
	pllcon2 = readl_relaxed(xmcclk->base + 0x11C);

	k1div = (pllcon1 & 0x7f) + 1;
	ndiv = ((pllcon1 >> 8) & 0x7f) + 1;
	k2div = ((pllcon1 >> 16) & 0x7f) + 1;
	pdiv = ((pllcon1 >> 24) & 0xf) + 1;

	/* assuming normal mode */
	if (1) {
		rate = parent_rate * ndiv / pdiv / k2div;
		pr_info("PLL normal mode: %lu (parent %lu)\n", rate, parent_rate);
		return rate;
	}
}

static const struct clk_ops xmc4500_pll_clk_ops = {
	.recalc_rate	= xmc4500_pll_clk_recalc_rate,
};

static struct clk *xmc4500_register_pll_clk(const char *name, const char *parent_name,
	void __iomem *base)
{
	struct xmc4500_pll_clk *xmcclk;
	struct clk_init_data init;
	struct clk *clk;

	xmcclk = kzalloc(sizeof(*xmcclk), GFP_KERNEL);
	if (!xmcclk)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &xmc4500_pll_clk_ops;
	init.flags = 0;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	xmcclk->base = base;
	xmcclk->hw.init = &init;

	clk = clk_register(NULL, &xmcclk->hw);
	if (IS_ERR(clk)) {
		pr_err("could not register clk %s\n", name);
		kfree(xmcclk);
		return clk;
	}

	pr_info("%s: PLL with parent %s\n", name, parent_name ? parent_name : "(null)");

	return clk;
}

enum {
	CLK_CPU = 0,
	CLK_PERIPH,
	CLK_CCU,

	CLK_USB,
	CLK_MMC,
	CLK_ETH0,
	CLK_EBU,
	CLK_WDT,

	CLK_EXT,

	CLK_NR_CLKS
};

static struct clk *clk[CLK_NR_CLKS];

static struct clk_onecell_data clk_data = {
	.clks = clk,
	.clk_num = ARRAY_SIZE(clk),
};

static const char *const xmc4500_sysclk_parents[] = {
	"OFI", "PLL"
};

static const char *const xmc4500_usb_parents[] = {
	"PLLUSB", "PLL"
};

static const char *const xmc4500_wdt_parents[] = {
	"OFI", "STDBY", "PLL"
};

static const char *const xmc4500_ext_parents[] = {
	"SYS", NULL, "USB", "PLL"
};

static __init void xmc4500_scu_ccu_init(struct device_node *node)
{
	void __iomem *base;
	struct clk *clk_xtal, *clk_xtal_rtc;
	int i;

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("Failed to map address range for CCU node\n");
		return;
	}

	pr_info("CCU @ 0x%p\n", base);

	for (i = 0; i < ARRAY_SIZE(clk); i++)
		clk[i] = ERR_PTR(-ENOENT);

	clk_xtal = of_clk_get_by_name(node, "xtal");
	if (IS_ERR(clk_xtal))
		pr_warn("Could not find xtal clock.\n");

	clk_xtal_rtc = of_clk_get_by_name(node, "xtal_rtc");
	if (IS_ERR(clk_xtal_rtc))
		pr_warn("Could not find xtal_rtc clock.\n");

	clk_register_fixed_factor(NULL, "OSCHP", __clk_get_name(clk_xtal), 0, 1, 1);
	clk_register_fixed_factor(NULL, "OSCULP", __clk_get_name(clk_xtal_rtc), 0, 1, 1);

	xmc4500_register_pll_clk("PLL", "OSCHP", base);
	xmc4500_register_pllusb_clk("PLLUSB", "OSCHP", base);
	clk_register_fixed_rate(NULL, "OFI", NULL, 0, 32768);
	clk_register_fixed_rate(NULL, "OSI", NULL, 0, 32768);

	clk_register_mux(NULL, "sysmux", xmc4500_sysclk_parents, 2, 0,
			base + 0xC, 16, 1, CLK_MUX_READ_ONLY, NULL);
	clk_register_divider(NULL, "SYS", "sysmux", 0,
			base + 0xC, 0, 8, 0, NULL);

	clk[CLK_CPU] = clk_register_divider(NULL, "CPU", "SYS", 0,
			base + 0x10, 0, 1, 0, NULL);

	clk[CLK_PERIPH] = clk_register_divider(NULL, "PERIPH", "CPU", 0,
			base + 0x14, 0, 1, 0, NULL);

	clk_register_mux(NULL, "usbmux", xmc4500_usb_parents, 2, 0,
			base + 0x18, 16, 1, 0, NULL);
	clk_register_divider(NULL, "USB", "usbmux", 0,
			base + 0x18, 0, 3, 0, NULL);

	clk_register_divider(NULL, "EBU", "PLL", 0,
			base + 0x1C, 0, 6, 0, NULL);

	clk_register_divider(NULL, "CCU", "SYS", 0,
			base + 0x20, 0, 1, 0, NULL);

	clk_register_mux(NULL, "wdtmux", xmc4500_wdt_parents, 3, 0,
			base + 0x24, 16, 2, 0, NULL);
	clk_register_divider(NULL, "WDT", "wdtmux", 0,
			base + 0x24, 0, 8, 0, NULL);

	clk_register_mux(NULL, "extmux", xmc4500_ext_parents, 4, 0,
			base + 0x28, 0, 2, 0, NULL);
	clk[CLK_EXT] = clk_register_divider(NULL, "EXT", "extmux", 0,
			base + 0x28, 16, 8, 0, NULL);

	/* CLK gates */
	clk[CLK_USB] = xmc4500_register_gate_clk("usb", "USB", base, 0);
	clk[CLK_MMC] = xmc4500_register_gate_clk("mmc", "USB", base, 1);
	clk[CLK_ETH0] = xmc4500_register_gate_clk("eth0", "ETH", base, 2);
	clk[CLK_EBU] = xmc4500_register_gate_clk("ebu", "EBU", base, 3);
	clk[CLK_CCU] = xmc4500_register_gate_clk("ccu", "CCU", base, 4);
	clk[CLK_WDT] = xmc4500_register_gate_clk("wdt", "WDT", base, 5);

	of_clk_add_provider(node, of_clk_src_onecell_get, &clk_data);

	for (i = 0; i < ARRAY_SIZE(clk); i++)
		if (!IS_ERR(clk[i]))
			pr_info("clk %s @ %lu\n", __clk_get_name(clk[i]), clk_get_rate(clk[i]));
}
CLK_OF_DECLARE(xmc4500scuccu, "infineon,xmc4500-scu-ccu", xmc4500_scu_ccu_init);
