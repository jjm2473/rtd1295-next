/*
 * XMC4500 CCU4
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

#define to_xmc_clockevent(_evt) container_of(_evt, struct xmc4500_clock_event_device, evtdev)

struct xmc4500_clock_event_device {
	struct clock_event_device evtdev;
	void __iomem *regs;
	int slice;
	u16 periodic_reload;
};

#define CCU4_GCTRL	0x000

#define CCU4_GIDLC	0x00C
#define CCU4_GIDLC_SPRB		BIT(8)

#define CCU4_CC40TCSET	0x10C
#define CCU4_CC4yTCSET_TRBS	BIT(0)

#define CCU4_CC40TCCLR	0x110
#define CCU4_CC4yTCCLR_TRBC	BIT(0)

#define CCU4_CC40TC	0x114
#define CCU4_CC4yTC_TSSM	BIT(1)
#define CCU4_CC4yTC_ENDM	BIT(9)
#define CCU4_CC4yTC_STRM	BIT(10)

#define CCU4_CC40PSC	0x124

#define CCU4_CC40PRS	0x134

#define CCU4_CC40TIMER	0x170

#define CCU4_CC40INTE	0x1A4
#define CCU4_CC4yINTE_PME	BIT(0)
#define CCU4_CC4yINTE_OME	BIT(1)

#define CCU4_CC40SWR	0x1B0
#define CCU4_CC4ySWR_RPM	BIT(0)
#define CCU4_CC4ySWR_ROM	BIT(1)

static void xmc4500_clock_event_set_mode(enum clock_event_mode mode,
	struct clock_event_device *evtdev)
{
	struct xmc4500_clock_event_device *xmcdev = to_xmc_clockevent(evtdev);
	int slice_offset = xmcdev->slice * 0x100;
	u32 tc;

	tc = readl_relaxed(xmcdev->regs + CCU4_CC40TC + xmcdev->slice * 0x100);
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		writel_relaxed((tc & ~CCU4_CC4yTC_TSSM) | CCU4_CC4yTC_STRM, xmcdev->regs + CCU4_CC40TC + slice_offset);
		writel_relaxed(xmcdev->periodic_reload, xmcdev->regs + CCU4_CC40PRS + slice_offset);
		writel_relaxed(CCU4_CC4yINTE_OME, xmcdev->regs + CCU4_CC40INTE + slice_offset);
		writel_relaxed(CCU4_CC4yTCSET_TRBS, xmcdev->regs + CCU4_CC40TCSET + slice_offset);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
	default:
		writel_relaxed(CCU4_CC4yTCCLR_TRBC, xmcdev->regs + CCU4_CC40TCCLR + slice_offset);
		writel_relaxed(CCU4_CC4yINTE_OME, xmcdev->regs + CCU4_CC40INTE + slice_offset);
		writel_relaxed(tc | CCU4_CC4yTC_TSSM, xmcdev->regs + CCU4_CC40TC + slice_offset);
		break;
	}
}

static irqreturn_t xmc4500_clock_event_handler(int irq, void *opaque)
{
	struct xmc4500_clock_event_device *xmcdev = opaque;

	writel_relaxed(CCU4_CC4ySWR_ROM, xmcdev->regs + CCU4_CC40SWR + xmcdev->slice * 0x100);

	xmcdev->evtdev.event_handler(&xmcdev->evtdev);

	return IRQ_HANDLED;
}

static struct xmc4500_clock_event_device xmc4500_clock_event_device = {
	.evtdev = {
		.name = "XMC4500 CCU4",
		.rating = 200,
		.features = CLOCK_EVT_FEAT_PERIODIC,
		.set_mode = xmc4500_clock_event_set_mode,
		.cpumask = cpu_all_mask,
	},
};

static __init void xmc4500_ccu4_init(struct device_node *node)
{
	void __iomem *base;
	struct clk *clk;
	unsigned long rate;
	struct reset_control *reset;
	int ret, irq;

	reset = of_reset_control_get(node, NULL);
	if (!IS_ERR(reset)) {
		if (!reset_control_status(reset))
			reset_control_assert(reset);
		reset_control_deassert(reset);
		reset_control_put(reset);
	} else {
		writel_relaxed(BIT(2), (void *)(0x50004400 + 0x10));
		writel_relaxed(BIT(2), (void *)(0x50004400 + 0x14));
	}

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
	xmc4500_clock_event_device.slice = 0;

	irq = irq_of_parse_and_map(node, 0);
	if (!irq) {
		pr_err("failed to get irq for CCU4\n");
		goto err_get_irq;
	}

	pr_info("CCU4 @ 0x%p (%lu)\n", base, rate);

	writel_relaxed(CCU4_GIDLC_SPRB, base + CCU4_GIDLC);
	writel_relaxed(0, base + CCU4_GCTRL);

	writel_relaxed(0xa /* 1024 */, base + CCU4_CC40PSC + xmc4500_clock_event_device.slice * 0x100);
	writel_relaxed(BIT(xmc4500_clock_event_device.slice), base + CCU4_GIDLC);

	xmc4500_clock_event_device.periodic_reload = DIV_ROUND_CLOSEST(rate, 1024 * HZ);

	clockevents_config_and_register(&xmc4500_clock_event_device.evtdev,
		DIV_ROUND_CLOSEST(rate, 1024), 0x1, 0x0000ffff);

	ret = request_irq(irq, xmc4500_clock_event_handler, IRQF_TIMER,
		"xmc4500 clockevent", &xmc4500_clock_event_device);
	if (ret) {
		pr_err("failed to request irq for CCU4\n");
		goto err_request_irq;
	}
	xmc4500_clock_event_device.evtdev.irq = irq;

	return;

err_request_irq:
err_get_irq:
	iounmap(base);
err_iomap:
	clk_disable_unprepare(clk);
err_clk_enable:
	clk_put(clk);
err_clk_get:
	return;
}
CLOCKSOURCE_OF_DECLARE(xmc4500ccu4, "infineon,xmc4500-ccu4", xmc4500_ccu4_init);
