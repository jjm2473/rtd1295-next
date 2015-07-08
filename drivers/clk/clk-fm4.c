/*
 * FM4
 */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

enum {
	CLKPLL = 0,
	HCLK,
	PCLK0,
	PCLK1,
	PCLK2,
	CLK_NR_CLKS
};

static struct clk *clk[CLK_NR_CLKS];

static struct clk_onecell_data clk_data = {
	.clks = clk,
	.clk_num = ARRAY_SIZE(clk),
};

#define to_fm4_pll_clk(_hw) container_of(_hw, struct fm4_pll_clk, hw)

struct fm4_pll_clk {
	struct clk_hw hw;
	void __iomem *base;
};

static u8 fm4_pll_clk_get_parent(struct clk_hw *hw)
{
	struct fm4_pll_clk *fm4clk = to_fm4_pll_clk(hw);
	u8 psw_tmr;
	u8 parent;

	psw_tmr = readb_relaxed(fm4clk->base + 0x034);

	parent = (psw_tmr >> 4) & 0x1;
	if (parent >= clk_hw_get_num_parents(hw))
		return -EINVAL;

	return parent;
}

static unsigned long fm4_pll_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct fm4_pll_clk *fm4clk = to_fm4_pll_clk(hw);
	u8 pll_ctl1, pll_ctl2;
	int k, m, n;
	unsigned long rate;

	pll_ctl1 = readb_relaxed(fm4clk->base + 0x038);
	pll_ctl2 = readb_relaxed(fm4clk->base + 0x03C);

	k = ((pll_ctl1 >> 4) & 0xf) + 1;
	m = (pll_ctl1 & 0xf) + 1;
	n = (pll_ctl2 & 0x3f) + 1;

	rate = parent_rate / k;
	rate *= n;
	pr_info("PLL: %lu (parent %lu; K=%u, M=%u, N=%u)\n",
		rate, parent_rate, k, m, n);
	return rate;
}

static const struct clk_ops fm4_pll_clk_ops = {
	.get_parent	= fm4_pll_clk_get_parent,
	.recalc_rate	= fm4_pll_clk_recalc_rate,
};

static __init struct clk *fm4_register_pll_clk(const char *name,
	const char * const *parent_names, u8 num_parents, void __iomem *base)
{
	struct fm4_pll_clk *fm4clk;
	struct clk_init_data init;
	struct clk *clk;

	fm4clk = kzalloc(sizeof(*fm4clk), GFP_KERNEL);
	if (!fm4clk)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &fm4_pll_clk_ops;
	init.flags = 0;
	init.parent_names = parent_names;
	init.num_parents = num_parents;

	fm4clk->base = base;
	fm4clk->hw.init = &init;

	clk = clk_register(NULL, &fm4clk->hw);
	if (IS_ERR(clk)) {
		pr_err("could not register clk %s\n", name);
		kfree(fm4clk);
		return clk;
	}

	return clk;
}

static const char * pll_parent_names[] = {
	"CLKMO", "CLKHC",
};

static const char * master_parent_names[] = {
	"CLKHC", "CLKMO", "CLKPLL", "CLKLC", "CLKSO",
};

static u32 master_parent_values[] = {
	0, 1, 2, 4, 5,
};

static const struct clk_div_table hclk_dividers[] = {
	{ .val = 0, .div = 1 },
	{ .val = 1, .div = 2 },
	{ .val = 2, .div = 3 },
	{ .val = 3, .div = 4 },
	{ .val = 4, .div = 6 },
	{ .val = 5, .div = 8 },
	{ .val = 6, .div = 16 },
	{}
};

#define DUMP_CLK(_clk) if (!IS_ERR(_clk)) \
	pr_info("clk %s @ %lu\n", __clk_get_name(_clk), __clk_get_rate(_clk))

static __init void fm4_clk_init(struct device_node *node)
{
	void __iomem *base;
	int i;

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("Failed to map address range for CRG node\n");
		return;
	}

	pr_info("CRG @ 0x%p\n", base);

	pll_parent_names[0] = of_clk_get_parent_name(node, 0);
	master_parent_names[1] = of_clk_get_parent_name(node, 0);
	master_parent_names[4] = of_clk_get_parent_name(node, 1);

	for (i = 0; i < ARRAY_SIZE(clk); i++)
		clk[i] = ERR_PTR(-ENOENT);

	clk_register_fixed_rate(NULL, "CLKHC", NULL, 0, 4000000ULL);
	clk_register_fixed_rate(NULL, "CLKLC", NULL, 0, 100000ULL);

	clk[CLKPLL] = fm4_register_pll_clk("CLKPLL", pll_parent_names, 2, base);

	clk_register_mux_table(NULL, "master",
		master_parent_names, ARRAY_SIZE(master_parent_names), 0,
		base + 0x04, 5, 0x7, 0, master_parent_values, NULL);

	clk[HCLK] = clk_register_divider_table(NULL, "HCLK", "master", 0,
		base + 0x10, 0, 3, 0, hclk_dividers, NULL);

	clk[PCLK0] = clk_register_divider(NULL, "PCLK0", "HCLK", 0,
		base + 0x14, 0, 2, CLK_DIVIDER_POWER_OF_TWO, NULL);
	clk[PCLK1] = clk_register_divider(NULL, "PCLK1", "HCLK", 0,
		base + 0x18, 0, 2, CLK_DIVIDER_POWER_OF_TWO, NULL);
	clk[PCLK2] = clk_register_divider(NULL, "PCLK2", "HCLK", 0,
		base + 0x1C, 0, 2, CLK_DIVIDER_POWER_OF_TWO, NULL);

	of_clk_add_provider(node, of_clk_src_onecell_get, &clk_data);

	for (i = 0; i < ARRAY_SIZE(clk); i++)
		if (!IS_ERR(clk[i]))
			pr_info("clk %pCn @ %pCr\n", clk[i], clk[i]);
}
CLK_OF_DECLARE(fm4clk, "cypress,fm4-crg", fm4_clk_init);
