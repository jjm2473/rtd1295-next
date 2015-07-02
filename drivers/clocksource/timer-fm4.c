/*
 * FM4 Dual Timer
 */
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/reset.h>

#define to_fm4_clockevent(_evt) container_of(_evt, struct fm4_clock_event_device, evtdev)

struct fm4_clock_event_device {
	struct clock_event_device evtdev;
	void __iomem *regs;
	u32 periodic_reload;
};

#define TIMERxLOAD	0x000

#define TIMERxCONTROL	0x008

#define TIMERxCONTROL_ONESHOT			BIT(0)
#define TIMERxCONTROL_TIMERSIZE_32		BIT(1)
#define TIMERxCONTROL_TIMERPRE_1		(0 << 2)
#define TIMERxCONTROL_TIMERPRE_16		(1 << 2)
#define TIMERxCONTROL_TIMERPRE_256		(2 << 2)
#define TIMERxCONTROL_INTENABLE			BIT(5)
#define TIMERxCONTROL_TIMERMODE_PERIODIC	BIT(6)
#define TIMERxCONTROL_TIMEREN			BIT(7)

#define TIMERxINTCLR	0x00C

#define TIMERxRIS	0x010

static int fm4_clock_event_shutdown(struct clock_event_device *evtdev)
{
	struct fm4_clock_event_device *fm4dev = to_fm4_clockevent(evtdev);
	void __iomem *base = fm4dev->regs;
	u32 val;

	val = TIMERxCONTROL_INTENABLE | TIMERxCONTROL_TIMERPRE_256 | TIMERxCONTROL_TIMERSIZE_32;
	writel_relaxed(val, base + TIMERxCONTROL);

	return 0;
}

static int fm4_clock_event_set_periodic(struct clock_event_device *evtdev)
{
	struct fm4_clock_event_device *fm4dev = to_fm4_clockevent(evtdev);
	void __iomem *base = fm4dev->regs;
	u32 val;

	val = TIMERxCONTROL_TIMERMODE_PERIODIC |
		TIMERxCONTROL_INTENABLE | TIMERxCONTROL_TIMERPRE_256 |
		TIMERxCONTROL_TIMERSIZE_32;
	writel_relaxed(val, base + TIMERxCONTROL);
	writel_relaxed(fm4dev->periodic_reload, base + TIMERxLOAD);
	writel_relaxed(val | TIMERxCONTROL_TIMEREN, base + TIMERxCONTROL);

	return 0;
}

static int fm4_clock_event_set_oneshot(struct clock_event_device *evtdev)
{
	struct fm4_clock_event_device *fm4dev = to_fm4_clockevent(evtdev);
	void __iomem *base = fm4dev->regs;
	u32 val;

	val = TIMERxCONTROL_INTENABLE | TIMERxCONTROL_TIMERPRE_256 |
		TIMERxCONTROL_TIMERSIZE_32 | TIMERxCONTROL_ONESHOT;
	writel_relaxed(val, base + TIMERxCONTROL);

	return 0;
}

#define IRQ047MON	0x2C0

static irqreturn_t fm4_clock_event_handler(int irq, void *opaque)
{
	struct fm4_clock_event_device *fm4dev = opaque;
	void __iomem *timer_base = fm4dev->regs;
	void __iomem *irq_base = (void __iomem *)0x40031000;
	u32 timint;
	u32 val;

	timint = readl_relaxed(irq_base + IRQ047MON) & 3;
	if (timint & BIT(0)) {
		val = readl_relaxed(timer_base + TIMERxRIS);
		if (val & 1) {
			writel_relaxed(0xffffffff, timer_base + TIMERxINTCLR);

			fm4dev->evtdev.event_handler(&fm4dev->evtdev);
		}
	}
	if (timint & BIT(1)) {
		timer_base += 0x20;
		val = readl_relaxed(timer_base + TIMERxRIS);
		if (val & 1) {
			writel_relaxed(0xffffffff, timer_base + TIMERxINTCLR);
		}
	}

	return IRQ_HANDLED;
}

static struct fm4_clock_event_device fm4_clock_event_device = {
	.evtdev = {
		.name = "FM4 Dual Timer",
		.rating = 200,
		.features = CLOCK_EVT_FEAT_PERIODIC,
		.set_state_shutdown = fm4_clock_event_shutdown,
		.set_state_periodic = fm4_clock_event_set_periodic,
		.set_state_oneshot = fm4_clock_event_set_oneshot,
		.cpumask = cpu_all_mask,
	},
};

static __init int fm4_dualtimer_init(struct device_node *node)
{
	void __iomem *base;
	struct clk *clk;
	unsigned long rate;
	int ret, irq;

	clk = of_clk_get(node, 0);
	if (IS_ERR(clk)) {
		pr_err("failed to get clock for Dual Timer\n");
		ret = PTR_ERR(clk);
		goto err_clk_get;
	}
	ret = clk_prepare_enable(clk);
	if (ret) {
		pr_err("failed to enable clock for Dual Timer (%d)\n", ret);
		goto err_clk_enable;
	}
	rate = clk_get_rate(clk);

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("failed to map Dual Timer\n");
		ret = -ENXIO;
		goto err_iomap;
	}
	fm4_clock_event_device.regs = base;

	irq = irq_of_parse_and_map(node, 0);
	if (!irq) {
		pr_err("failed to get irq for Dual Timer\n");
		ret = -EINVAL;
		goto err_get_irq;
	}

	writel_relaxed(0xffffffff, base + TIMERxINTCLR);
	writel_relaxed(0xffffffff, base + 0x020 + TIMERxINTCLR);

	fm4_clock_event_device.periodic_reload = DIV_ROUND_CLOSEST(rate, 1024 * HZ);

	clockevents_config_and_register(&fm4_clock_event_device.evtdev,
		DIV_ROUND_CLOSEST(rate, 1024), 0x1, 0x0000ffff);

	ret = request_irq(irq, fm4_clock_event_handler, IRQF_TIMER,
		"fm4 clockevent", &fm4_clock_event_device);
	if (ret) {
		pr_err("failed to request irq for Dual Timer\n");
		goto err_request_irq;
	}
	fm4_clock_event_device.evtdev.irq = irq;

	pr_info("Dual Timer @ %p (%lu)\n", base, rate);

	return 0;

err_request_irq:
err_get_irq:
	iounmap(base);
err_iomap:
	clk_disable_unprepare(clk);
err_clk_enable:
	clk_put(clk);
err_clk_get:
	return ret;
}
CLOCKSOURCE_OF_DECLARE(fm4dualtimer, "cypress,fm4-dual-timer", fm4_dualtimer_init);
