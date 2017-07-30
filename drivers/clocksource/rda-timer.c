/*
 * Copyright (c) 2017 Andreas Färber
 */

#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqreturn.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#define RDA_HWTIMER_LOCKVAL_LO 0x24
#define RDA_HWTIMER_LOCKVAL_HI 0x28

static void __iomem *rda_timer_base;

static u64 rda_hwtimer_read(struct clocksource *cs)
{
	u32 lo, hi;

	/* Always read low 32 bits first */
	lo = readl(rda_timer_base + RDA_HWTIMER_LOCKVAL_LO);
	hi = readl(rda_timer_base + RDA_HWTIMER_LOCKVAL_HI);

	return ((u64)hi << 32) | lo;
}

static struct clocksource rda_clocksource = {
	.name           = "rda-timer",
	.rating         = 400,
	.read           = rda_hwtimer_read,
	.mask           = CLOCKSOURCE_MASK(64),
	.flags          = CLOCK_SOURCE_IS_CONTINUOUS,
};

static int __init rda_timer_init(struct device_node *node)
{
	rda_timer_base = of_io_request_and_map(node, 0, "rda-timer");
	if (IS_ERR(rda_timer_base)) {
		pr_err("Can't map timer registers");
		return PTR_ERR(rda_timer_base);
	}

	clocksource_register_hz(&rda_clocksource, 2000000);

	return 0;
}
CLOCKSOURCE_OF_DECLARE(rda8810pl, "rda,8810pl-timer", rda_timer_init);
