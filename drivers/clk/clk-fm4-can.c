/*
 * FM4
 */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#define to_fm4_can_clk(_hw) container_of(_hw, struct fm4_can_clk, hw)

struct fm4_can_clk {
	struct clk_hw hw;
	void __iomem *base;
};

static unsigned long fm4_can_prescaler_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct fm4_can_clk *fm4clk = to_fm4_can_clk(hw);
	u8 canpre;
	unsigned long rate;

	canpre = readb_relaxed(fm4clk->base + 0x0);
	switch (canpre & 0xf) {
	case 0x0:
		rate = parent_rate;
		break;
	case 0x1:
		rate = parent_rate / 2;
		break;
	case 0x2:
	case 0x3:
		rate = parent_rate / 4;
		break;
	case 0x4:
	case 0x5:
	case 0x6:
	case 0x7:
		rate = parent_rate / 8;
		break;
	case 0x8:
		rate = parent_rate * 2 / 3;
		break;
	case 0x9:
		rate = parent_rate / 3;
		break;
	case 0xa:
		rate = parent_rate / 6;
		break;
	case 0xb:
		rate = parent_rate / 12;
		break;
	case 0xc:
	case 0xd:
		rate = parent_rate / 5;
		break;
	case 0xe:
	case 0xf:
		rate = parent_rate / 10;
		break;
	}

	return rate;
}

/*static long fm4_can_prescaler_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *prate)
{
	long maxdiv = 12;

	
}

static int fm4_can_prescaler_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct fm4_can_clk *fm4clk = to_fm4_can_clk(hw);
	u8 reg;
	u8 canpre;

	if (rate == parent_rate)
		canpre = 0x0;
	else if (rate == parent_rate / 2)
		canpre = 0x1;
	else if (rate == parent_rate / 3)
		canpre = 0x9;
	else if (rate == parent_rate / 4)
		canpre = 0x2;
	else if (rate == parent_rate / 5)
		canpre = 0xc;
	else if (rate == parent_rate / 6)
		canpre = 0xa;
	else if (rate == parent_rate / 8)
		canpre = 0x4;
	else if (rate == parent_rate / 10)
		canpre = 0xe;
	else if (rate == parent_rate / 12)
		canpre = 0xb;
	else if (rate == parent_rate * 2 / 3)
		canpre = 0x8;
	else
		return -EINVAL;

	reg = readb_relaxed(fm4clk->base);
	reg &= ~0xf;
	writeb_relaxed(reg | canpre, fm4clk->base);
		
	return 0;
}*/

static const struct clk_ops fm4_can_prescaler_ops = {
	.recalc_rate	= fm4_can_prescaler_recalc_rate,
	//.round_rate	= fm4_can_prescaler_round_rate,
	//.set_rate	= fm4_can_prescaler_set_rate,
};

static __init void fm4_can_prescaler_init(struct device_node *node)
{
	void __iomem *base;
	struct fm4_can_clk *fm4clk;
	struct clk_init_data init;
	struct clk *clk;
	const char *parent_name = "CLKPLL";

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("Failed to map address range for CCU node\n");
		return;
	}

	pr_info("CAN prescaler @ 0x%p\n", base);

	fm4clk = kzalloc(sizeof(*fm4clk), GFP_KERNEL);
	if (!fm4clk)
		return;

	init.name = "can";
	init.ops = &fm4_can_prescaler_ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	fm4clk->base = base;
	fm4clk->hw.init = &init;

	clk = clk_register(NULL, &fm4clk->hw);
	if (IS_ERR(clk)) {
		pr_err("could not register CAN prescaler clk\n");
		kfree(fm4clk);
		return;
	}

	of_clk_add_provider(node, of_clk_src_simple_get, clk);

	pr_info("clk %pCn @ %pCr\n", clk, clk);
}
CLK_OF_DECLARE(fm4canprescaler, "cypress,fm4-can-prescaler", fm4_can_prescaler_init);
