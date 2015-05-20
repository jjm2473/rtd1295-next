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

#define CCU4_GSTAT	0x004

#define CCU4_GIDLC	0x00C
#define CCU4_GIDLC_SPRB		BIT(8)

#define CCU4_GCSS	0x010
#define CCU4_GCSS_S0SE		BIT(0)

#define CCU4_GCST	0x018

#define CCU4_CC40TST	0x08

#define CCU4_CC40TCSET	0x0C
#define CCU4_CC4yTCSET_TRBS	BIT(0)

#define CCU4_CC40TCCLR	0x10
#define CCU4_CC4yTCCLR_TRBC	BIT(0)
#define CCU4_CC4yTCCLR_TCC	BIT(1)

#define CCU4_CC40TC	0x14
#define CCU4_CC4yTC_TCM		BIT(0)
#define CCU4_CC4yTC_TSSM	BIT(1)
#define CCU4_CC4yTC_CLST	BIT(2)
#define CCU4_CC4yTC_ENDM	(2UL << 8)
#define CCU4_CC4yTC_STRM	BIT(10)

#define CCU4_CC40PSC	0x24

#define CCU4_CC40PRS	0x34

#define CCU4_CC40TIMER	0x70

#define CCU4_CC40INTS	0xA0

#define CCU4_CC40INTE	0xA4
#define CCU4_CC4yINTE_PME	BIT(0)
#define CCU4_CC4yINTE_OME	BIT(1)

#define CCU4_CC40SWS	0xAC
#define CCU4_CC4ySWS_SPM	BIT(0)
#define CCU4_CC4ySWS_SOM	BIT(1)

#define CCU4_CC40SWR	0xB0
#define CCU4_CC4ySWR_RPM	BIT(0)
#define CCU4_CC4ySWR_ROM	BIT(1)

static int xmc4500_clock_event_shutdown(struct clock_event_device *evtdev)
{
	struct xmc4500_clock_event_device *xmcdev = to_xmc_clockevent(evtdev);
	int slice_offset = xmcdev->slice * 0x100;
	void __iomem *slice_base = xmcdev->regs + 0x100 + slice_offset;

	writel_relaxed(CCU4_CC4yTCCLR_TRBC, slice_base + CCU4_CC40TCCLR);

	return 0;
}

static int xmc4500_clock_event_set_periodic(struct clock_event_device *evtdev)
{
	struct xmc4500_clock_event_device *xmcdev = to_xmc_clockevent(evtdev);
	int slice_offset = xmcdev->slice * 0x100;
	void __iomem *slice_base = xmcdev->regs + 0x100 + slice_offset;
	u32 tc;

	tc = readl_relaxed(slice_base + CCU4_CC40TC);
	tc |= CCU4_CC4yTC_CLST;

	writel_relaxed(xmcdev->periodic_reload, slice_base + CCU4_CC40PRS);
	writel_relaxed(tc & ~CCU4_CC4yTC_TSSM, slice_base + CCU4_CC40TC);
	writel_relaxed(CCU4_GCSS_S0SE, xmcdev->regs + CCU4_GCSS);
	writel_relaxed(CCU4_CC4yTCCLR_TRBC | CCU4_CC4yTCCLR_TCC, slice_base + CCU4_CC40TCCLR);
	writel_relaxed(CCU4_CC4yTCSET_TRBS, slice_base + CCU4_CC40TCSET);

	return 0;
}

static int xmc4500_clock_event_set_oneshot(struct clock_event_device *evtdev)
{
	struct xmc4500_clock_event_device *xmcdev = to_xmc_clockevent(evtdev);
	int slice_offset = xmcdev->slice * 0x100;
	void __iomem *slice_base = xmcdev->regs + 0x100 + slice_offset;
	u32 tc;

	tc = readl_relaxed(slice_base + CCU4_CC40TC);
	tc |= CCU4_CC4yTC_CLST;

	writel_relaxed(tc | CCU4_CC4yTC_TSSM, slice_base + CCU4_CC40TC);
	writel_relaxed(CCU4_CC4yTCCLR_TRBC, slice_base + CCU4_CC40TCCLR);

	return 0;
}

static irqreturn_t xmc4500_clock_event_handler(int irq, void *opaque)
{
	struct xmc4500_clock_event_device *xmcdev = opaque;
	int slice_offset = xmcdev->slice * 0x100;
	void __iomem *slice_base = xmcdev->regs + 0x100 + slice_offset;

	writel_relaxed(CCU4_CC4ySWR_ROM | CCU4_CC4ySWR_RPM, slice_base + CCU4_CC40SWR);

	xmcdev->evtdev.event_handler(&xmcdev->evtdev);

	return IRQ_HANDLED;
}

static struct xmc4500_clock_event_device xmc4500_clock_event_device = {
	.evtdev = {
		.name = "XMC4500 CCU4",
		.rating = 200,
		.features = CLOCK_EVT_FEAT_PERIODIC,
		.set_state_shutdown = xmc4500_clock_event_shutdown,
		.set_state_periodic = xmc4500_clock_event_set_periodic,
		.set_state_oneshot = xmc4500_clock_event_set_oneshot,
		.tick_resume = xmc4500_clock_event_shutdown,
		.cpumask = cpu_all_mask,
	},
};

static __init int xmc4500_ccu4_init(struct device_node *node)
{
	void __iomem *base;
	void __iomem *slice_base;
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
	}

	clk = of_clk_get_by_name(node, "mclk");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		pr_err("failed to get mclk for CCU4 (%d)\n", ret);
		goto err_clk_get;
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		pr_err("failed to enable mclk for CCU4 (%d)\n", ret);
		goto err_clk_enable;
	}
	rate = clk_get_rate(clk);

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("failed to map CCU4\n");
		ret = -ENXIO;
		goto err_iomap;
	}
	xmc4500_clock_event_device.regs = base;
	xmc4500_clock_event_device.slice = 0;
	slice_base = base + 0x100 + xmc4500_clock_event_device.slice * 0x100;

	irq = irq_of_parse_and_map(node, 0);
	if (!irq) {
		pr_err("failed to get irq for CCU4\n");
		ret = -EINVAL;
		goto err_get_irq;
	}

	writel_relaxed(CCU4_GIDLC_SPRB, base + CCU4_GIDLC);
	writel_relaxed(0, base + CCU4_GCTRL);

	writel_relaxed(0xa /* 1024 */, slice_base + CCU4_CC40PSC);
	writel_relaxed(CCU4_CC4yINTE_OME | CCU4_CC4yINTE_PME, slice_base + CCU4_CC40INTE);

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
CLOCKSOURCE_OF_DECLARE(xmc4500ccu4, "infineon,xmc4500-ccu4", xmc4500_ccu4_init);
