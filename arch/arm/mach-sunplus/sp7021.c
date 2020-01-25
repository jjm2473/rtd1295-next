#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <linux/io.h>
#include <linux/memblock.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <asm/mach/arch.h>

static void __init sp7021_init_time(void)
{
	void __iomem *base;

	pr_info("%s\n", __func__);

	base = ioremap(0x9ed0a000, 4);
	writel(0x3, base);
	mb();
	iounmap(base);

	of_clk_init(NULL);
	timer_probe();
}

static const char *const sp7021_dt_compat[] __initconst = {
	"sunplus,sp7021",
	NULL
};

DT_MACHINE_START(sp7021, "Sunplus SP7021")
	.dt_compat = sp7021_dt_compat,
	.init_time = sp7021_init_time,
	.l2c_aux_val = 0x0,
	.l2c_aux_mask = ~0x0,
MACHINE_END
