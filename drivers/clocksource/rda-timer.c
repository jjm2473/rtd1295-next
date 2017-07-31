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

#define RDA_OSTIMER_LOADVAL_L	0x000
#define RDA_OSTIMER_CTRL	0x004
#define RDA_HWTIMER_LOCKVAL_L	0x024
#define RDA_HWTIMER_LOCKVAL_H	0x028
#define RDA_TIMER_IRQ_MASK_SET	0x02c
#define RDA_TIMER_IRQ_CLR	0x034

#define RDA_OSTIMER_CTRL_ENABLE		BIT(24)
#define RDA_OSTIMER_CTRL_REPEAT		BIT(28)
#define RDA_OSTIMER_CTRL_LOAD		BIT(30)

#define RDA_TIMER_IRQ_MASK_SET_OSTIMER	BIT(0)

#define RDA_TIMER_IRQ_CLR_OSTIMER	BIT(0)

static void __iomem *rda_timer_base;

static u64 rda_hwtimer_read(struct clocksource *cs)
{
	u32 lo, hi;

	/* Always read low 32 bits first */
	lo = readl(rda_timer_base + RDA_HWTIMER_LOCKVAL_L);
	hi = readl(rda_timer_base + RDA_HWTIMER_LOCKVAL_H);

	return ((u64)hi << 32) | lo;
}

static struct clocksource rda_clocksource = {
	.name           = "rda-timer",
	.rating         = 400,
	.read           = rda_hwtimer_read,
	.mask           = CLOCKSOURCE_MASK(64),
	.flags          = CLOCK_SOURCE_IS_CONTINUOUS,
};

static int rda_ostimer_start(bool periodic, u64 cycles)
{
	u32 ctrl, load_l;

	load_l = (u32)cycles;
	ctrl = ((cycles >> 32) & 0xffffff);
	ctrl |= RDA_OSTIMER_CTRL_LOAD | RDA_OSTIMER_CTRL_ENABLE;
	if (periodic)
		ctrl |= RDA_OSTIMER_CTRL_REPEAT;

	/* Enable ostimer interrupt first */
	writel(RDA_TIMER_IRQ_MASK_SET_OSTIMER, rda_timer_base + RDA_TIMER_IRQ_MASK_SET);

	/* Write low 32 bits first, high 24 bits are with ctrl */
	writel(load_l, rda_timer_base + RDA_OSTIMER_LOADVAL_L);
	writel(ctrl, rda_timer_base + RDA_OSTIMER_CTRL);

	return 0;
}

static int rda_ostimer_stop(void)
{
	/* Disable ostimer interrupt first */
	writel(0, rda_timer_base + RDA_TIMER_IRQ_MASK_SET);

	writel(0, rda_timer_base + RDA_OSTIMER_CTRL);

	return 0;
}

static int rda_ostimer_set_state_shutdown(struct clock_event_device *evt)
{
	pr_info("%s\n", __func__);

	rda_ostimer_stop();

	return 0;
}

static int rda_ostimer_set_state_oneshot(struct clock_event_device *evt)
{
	pr_info("%s\n", __func__);

	rda_ostimer_stop();

	return 0;
}

static int rda_ostimer_set_state_periodic(struct clock_event_device *evt)
{
	unsigned long cycles_per_jiffy;

	pr_info("%s\n", __func__);

	rda_ostimer_stop();

	cycles_per_jiffy =
		(((unsigned long long) NSEC_PER_SEC / HZ * evt->mult) >> evt->shift);
	rda_ostimer_start(true, cycles_per_jiffy);

	return 0;
}

static int rda_ostimer_tick_resume(struct clock_event_device *evt)
{
	pr_info("%s\n", __func__);

	return 0;
}

static int rda_ostimer_set_next_event(unsigned long evt,
				    struct clock_event_device *ev)
{
	pr_info("%s\n", __func__);

	rda_ostimer_start(false, evt);

	return 0;
}

static struct clock_event_device rda_clockevent = {
	.name			= "rda-ostimer",
	.rating			= 250,
	.features		= CLOCK_EVT_FEAT_PERIODIC |
				  CLOCK_EVT_FEAT_ONESHOT |
				  CLOCK_EVT_FEAT_DYNIRQ,
	.set_state_shutdown	= rda_ostimer_set_state_shutdown,
	.set_state_oneshot	= rda_ostimer_set_state_oneshot,
	.set_state_periodic	= rda_ostimer_set_state_periodic,
	.tick_resume		= rda_ostimer_tick_resume,
	.set_next_event		= rda_ostimer_set_next_event,
};

static irqreturn_t rda_ostimer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	pr_info("%s\n", __func__);

	/* clear timer int */
	writel(RDA_TIMER_IRQ_CLR_OSTIMER, rda_timer_base + RDA_TIMER_IRQ_CLR);

	if (evt->event_handler)
		evt->event_handler(evt);

	return IRQ_HANDLED;
}

static int __init rda_timer_init(struct device_node *node)
{
	unsigned long rate = 2000000;
	int ostimer_irq, ret;

	rda_timer_base = of_io_request_and_map(node, 0, "rda-timer");
	if (IS_ERR(rda_timer_base)) {
		pr_err("Can't map timer registers");
		return PTR_ERR(rda_timer_base);
	}

	ostimer_irq = of_irq_get_byname(node, "ostimer");
	if (ostimer_irq <= 0) {
		pr_err("Can't parse ostimer IRQ");
		return -EINVAL;
	}

	clocksource_register_hz(&rda_clocksource, rate);

	ret = request_irq(ostimer_irq, rda_ostimer_interrupt, IRQF_TIMER,
			  "rda-ostimer", &rda_clockevent);
	if (ret) {
		pr_err("failed to request irq %d\n", ostimer_irq);
		return ret;
	}

	irq_force_affinity(ostimer_irq, cpumask_of(0));

	rda_clockevent.cpumask = cpumask_of(0);
	rda_clockevent.irq = ostimer_irq;
	clockevents_config_and_register(&rda_clockevent, rate,
					0x2, 0xffffffff);

	pr_info("%s done\n", __func__);

	return 0;
}
CLOCKSOURCE_OF_DECLARE(rda8810pl, "rda,8810pl-timer", rda_timer_init);
