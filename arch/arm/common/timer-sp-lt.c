/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <asm/localtimer.h>
#include <asm/hardware/arm_timer.h>
#include <asm/hardware/timer-sp.h>

static int irq[4];
static int instances;
static void __iomem *base;
static unsigned long rate;
static struct clock_event_device *evtdev[4];

#define timer_base(c)	(base + ((c) > 1 ? 0x10000:0) + (c % 2) * TIMER_2_BASE)

static inline int to_cpu(struct clock_event_device *evt)
{
	int n;

	for (n = 0; n < instances; n++)
		if (evt == evtdev[n])
			return n;

	BUG();
}

static long __init sp804lt_get_clock_rate(struct clk *clk)
{
	int err;

	err = clk_prepare(clk);
	if (err) {
		pr_err("sp804lt: clock failed to prepare: %d\n", err);
		clk_put(clk);
		return err;
	}

	err = clk_enable(clk);
	if (err) {
		pr_err("sp804lt: clock failed to enable: %d\n", err);
		clk_unprepare(clk);
		clk_put(clk);
		return err;
	}

	rate = clk_get_rate(clk);
	if (rate < 0) {
		pr_err("sp804lt: clock failed to get rate: %ld\n", rate);
		clk_disable(clk);
		clk_unprepare(clk);
		clk_put(clk);
	}

	return rate;
}

static irqreturn_t sp804lt_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	unsigned int cpu = to_cpu(evt);

	/* clear the interrupt */
	writel(1, timer_base(cpu) + TIMER_INTCLR);

	if (cpu == smp_processor_id())
		evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction sp804lt_timer_irq[] = {
	[0] = {
		.name = "sp804_lt0",
		.flags = IRQF_TIMER | IRQF_NOBALANCING,
		.handler = sp804lt_timer_interrupt,
	},
	[1] = {
		.name = "sp804_lt1",
		.flags = IRQF_TIMER | IRQF_NOBALANCING,
		.handler = sp804lt_timer_interrupt,
	},
	[2] = {
		.name = "sp804_lt2",
		.flags = IRQF_TIMER | IRQF_NOBALANCING,
		.handler = sp804lt_timer_interrupt,
	},
	[3] = {
		.name = "sp804_lt3",
		.flags = IRQF_TIMER | IRQF_NOBALANCING,
		.handler = sp804lt_timer_interrupt,
	},
};

static void sp804lt_set_mode(enum clock_event_mode mode,
	struct clock_event_device *evt)
{
	unsigned int cpu = to_cpu(evt);
	unsigned long ctrl = TIMER_CTRL_32BIT | TIMER_CTRL_IE;

	writel(ctrl, timer_base(cpu) + TIMER_CTRL);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		writel(DIV_ROUND_CLOSEST(rate, HZ),
				timer_base(cpu) + TIMER_LOAD);
		ctrl |= TIMER_CTRL_PERIODIC | TIMER_CTRL_ENABLE;
		break;

	case CLOCK_EVT_MODE_ONESHOT:
		/* period set, and timer enabled in 'next_event' hook */
		ctrl |= TIMER_CTRL_ONESHOT;
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		ctrl = 0;
		break;
	}

	writel(ctrl, timer_base(cpu) + TIMER_CTRL);
}

static int sp804lt_set_next_event(unsigned long next,
	struct clock_event_device *evt)
{
	unsigned int cpu = to_cpu(evt);
	unsigned long ctrl = readl(timer_base(cpu) + TIMER_CTRL);

	writel(next, timer_base(cpu) + TIMER_LOAD);
	writel(ctrl | TIMER_CTRL_ENABLE, timer_base(cpu) + TIMER_CTRL);

	return 0;
}

static int __cpuinit sp804lt_local_timer_setup(struct clock_event_device *evt)
{
	unsigned int cpu = smp_processor_id();

	evtdev[cpu] = evt;

	evt->irq = irq[cpu];
	evt->name = "sp804_as_LT";
	evt->features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT;
	evt->rating = 200;
	evt->set_mode = sp804lt_set_mode;
	evt->set_next_event = sp804lt_set_next_event;
	clockevents_config_and_register(evt, rate, 0xff, 0xffffffff);

	sp804lt_timer_irq[cpu].dev_id = evt;
	setup_irq(evt->irq, &sp804lt_timer_irq[cpu]);
	irq_set_affinity(evt->irq, cpumask_of(cpu));

	return 0;
}

static void sp804lt_local_timer_stop(struct clock_event_device *evt)
{
	unsigned int cpu = smp_processor_id();

	remove_irq(evt->irq, &sp804lt_timer_irq[cpu]);
	evt->set_mode(CLOCK_EVT_MODE_UNUSED, evt);
}

static struct local_timer_ops sp804lt_tick_ops __cpuinitdata = {
	.setup	= sp804lt_local_timer_setup,
	.stop	= sp804lt_local_timer_stop,
};

static void __init sp804lt_of_init(struct device_node *np)
{
	struct clk *clk;
	int n;

	clk = of_clk_get(np, 0);
	if (WARN_ON(IS_ERR(clk)))
		return;

	if (sp804lt_get_clock_rate(clk) < 0)
		return;

	for (instances = 0; instances < ARRAY_SIZE(irq); instances++) {
		irq[instances] = irq_of_parse_and_map(np, instances);
		if (irq[instances] <= 0)
			break;
	}

	base = of_iomap(np, 0);
	if (WARN_ON(!base))
		return;

	/* Ensure timers are disabled */
	for(n = 0; n < instances; n++)
		writel(n, timer_base(n) + TIMER_CTRL);

	local_timer_register(&sp804lt_tick_ops);
}
CLOCKSOURCE_OF_DECLARE(sp804lt, "arm,sp804lt", sp804lt_of_init);
