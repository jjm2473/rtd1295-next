/*
 * XMC4500
 */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

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
	clk[CLK_CCU] = clk_register_divider(NULL, "CCU", "SYS", 0,
		base + 0x20, 0, 1, 0, NULL);

	clk[CLK_EBU] = clk_register_divider(NULL, "EBU", "PLL", 0,
		base + 0x1C, 0, 6, 0, NULL);

	of_clk_add_provider(node, of_clk_src_onecell_get, &clk_data);

	for (i = 0; i < ARRAY_SIZE(clk); i++)
		if (!IS_ERR(clk[i]))
			pr_info("clk %s @ %lu\n", __clk_get_name(clk[i]), __clk_get_rate(clk[i]));
}
CLK_OF_DECLARE(xmc4500scuccu, "infineon,xmc4500-scu-ccu", xmc4500_scu_ccu_init);
