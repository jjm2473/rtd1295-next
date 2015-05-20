/*
 * XMC4500 CCU4
 */
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define to_xmc_clockevent(_evt) container_of(_evt, struct xmc4500_clock_event_device, evtdev)

struct xmc4500_clock_event_device {
	struct clock_event_device evtdev;
	void __iomem *regs;
};

static struct xmc4500_clock_event_device xmc4500_clock_event_device = {
	.evtdev = {
		.name = "XMC4500 timer",
		.rating = 200,
		.features = CLOCK_EVT_FEAT_PERIODIC,
		//.set_mode = xmc4500_timer_set_mode,
		.cpumask = cpu_all_mask,
	},
};

static __init void xmc4500_ccu4_init(struct device_node *node)
{
	void __iomem *base;
	struct clk *clk;
	unsigned long rate;
	int ret;

	clk = of_clk_get_by_name(node, "mclk");
	if (IS_ERR(clk)) {
		pr_err("failed to get clock for CCU4 (%d)\n", ret);
		goto err_clk_get;
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		pr_err("failed to enable clock for CCU4 (%d)\n", ret);
		goto err_clk_enable;
	}
	rate = clk_get_rate(clk);

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("failed to map CCU4\n");
		goto err_iomap;
	}
	xmc4500_clock_event_device.regs = base;

	pr_info("CCU4 @ 0x%p (%lu)\n", base, rate);

	return;

err_iomap:
	clk_disable_unprepare(clk);
err_clk_enable:
	clk_put(clk);
err_clk_get:
	return;
}
CLOCKSOURCE_OF_DECLARE(xmc4500ccu4, "infineon,xmc4500-ccu4", xmc4500_ccu4_init);
