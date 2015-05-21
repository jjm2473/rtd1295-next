/*
 * XMC4500
 */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

struct xmc4500_clk {
	struct clk_hw hw;
	void __iomem *base;
	u8 bit_idx;
};

#define to_xmc_clk(_hw) container_of(_hw, struct xmc4500_clk, hw)

static int xmc4500_clk_is_enabled(struct clk_hw *hw)
{
	struct xmc4500_clk *xmcclk = to_xmc_clk(hw);
	u32 reg;

	reg = clk_readl(xmcclk->base);

	return reg & BIT(xmcclk->bit_idx) ? 1 : 0;
}

static int xmc4500_clk_enable(struct clk_hw *hw)
{
	struct xmc4500_clk *xmcclk = to_xmc_clk(hw);

	clk_writel(BIT(xmcclk->bit_idx), xmcclk->base + 0x4);

	return 0;
}

static void xmc4500_clk_disable(struct clk_hw *hw)
{
	struct xmc4500_clk *xmcclk = to_xmc_clk(hw);

	clk_writel(BIT(xmcclk->bit_idx), xmcclk->base + 0x8);
}

static const struct clk_ops xmc4500_clk_ops = {
	.enable		= xmc4500_clk_enable,
	.disable	= xmc4500_clk_disable,
	.is_enabled	= xmc4500_clk_is_enabled,
};

static struct clk *xmc4500_register_gate_clk(const char *name, const char *parent_name,
	void __iomem *reg_base, u8 bit_idx)
{
	struct xmc4500_clk *xmcclk;
	struct clk_init_data init;
	struct clk *clk;

	xmcclk = kzalloc(sizeof(*xmcclk), GFP_KERNEL);
	if (!xmcclk) {
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &xmc4500_clk_ops;
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

enum {
	CLK_CPU = 0,
	CLK_PERIPH,
	CLK_CCU,
	CLK_EBU,
	CLK_NR_CLKS
};

static struct clk *clk[CLK_NR_CLKS];

static struct clk_onecell_data clk_data = {
	.clks = clk,
	.clk_num = ARRAY_SIZE(clk),
};

static const char *xmc4500_sysclk_parents[] = {
	"OFI", "PLL"
};

static __init void xmc4500_scu_ccu_init(struct device_node *node)
{
	void __iomem *base;
	int i;

	base = of_iomap(node, 0);
	if (!base) {
		pr_warn("Failed to map address range for CCU node\n");
		return;
	}

	pr_info("CCU @ 0x%p\n", base);

	for (i = 0; i < ARRAY_SIZE(clk); i++)
		clk[i] = ERR_PTR(-ENOENT);

	clk_register_fixed_rate(NULL, "PLL", NULL, CLK_IS_ROOT, 120000000ULL);
	clk_register_fixed_rate(NULL, "OFI", NULL, CLK_IS_ROOT, 32768);

	clk_register_mux(NULL, "sys", xmc4500_sysclk_parents, 2, 0,
		base + 0xC, 16, 1, CLK_MUX_READ_ONLY, NULL);
	clk_register_divider(NULL, "SYS", "sys", 0,
		base + 0xC, 0, 8, 0, NULL);

	clk[CLK_CPU] = clk_register_divider(NULL, "CPU", "SYS", 0,
		base + 0x10, 0, 1, 0, NULL);
	clk[CLK_PERIPH] = clk_register_divider(NULL, "PERIPH", "CPU", 0,
		base + 0x14, 0, 1, 0, NULL);
	clk_register_divider(NULL, "CCU", "SYS", 0,
		base + 0x20, 0, 1, 0, NULL);
	clk[CLK_CCU] = xmc4500_register_gate_clk("ccu", "CCU", base, 4);

	clk[CLK_EBU] = clk_register_divider(NULL, "EBU", "PLL", 0,
		base + 0x1C, 0, 6, 0, NULL);

	of_clk_add_provider(node, of_clk_src_onecell_get, &clk_data);

	for (i = 0; i < ARRAY_SIZE(clk); i++)
		if (!IS_ERR(clk[i]))
			pr_info("clk %s @ %lu\n", __clk_get_name(clk[i]), __clk_get_rate(clk[i]));
}
CLK_OF_DECLARE(xmc4500scuccu, "infineon,xmc4500-scu-ccu", xmc4500_scu_ccu_init);
