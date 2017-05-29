/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mailbox_controller.h>
#include <linux/platform_device.h>

#define INTR_STAT_OFS	0x0
#define INTR_SET_OFS	0x8
#define INTR_CLR_OFS	0x10

#define MHU_SCFG	0x400

struct mhu_link {
	unsigned irq;
	spinlock_t lock;
	void __iomem *tx_reg;
	void __iomem *rx_reg;
	struct ipc_link link;
};

struct f_mhu {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct mhu_link mlink[3];
	struct ipc_controller ipc_con;
};

static char *ch_name[] = {"LP_NonSec", "HP_NonSec", "Secure"};

static void __iomem *__hackbase;

static inline struct mhu_link *to_mlink(struct ipc_link *l)
{
	if (!l)
		return NULL;

	return container_of(l, struct mhu_link, link);
}

static irqreturn_t mhu_rx_interrupt(int irq, void *p)
{
	struct mhu_link *mlink = to_mlink(p);
	u32 val;

	mbox_dbg("%s:%d\n", __func__, __LINE__);
	/* See NOTE_RX_DONE */
	val = __raw_readl(mlink->rx_reg + INTR_STAT_OFS);
	ipc_link_received_data(&mlink->link, (void *)val);

	/*
	 * It is agreed with the remote firmware that the receiver
	 * will clear the STAT register indicating it is ready to
	 * receive next data - NOTE_RX_DONE
	 */
	__raw_writel(val, mlink->rx_reg + INTR_CLR_OFS);

	return IRQ_HANDLED;
}

/*
 * MHU doesn't get "remote RTR" interrupt, so we can't call
 * ipc_link_txdone() instead we provide this callback for
 * the API to poll the controller for status
 */
static bool mhu_last_tx_done(struct ipc_link *link)
{
	struct mhu_link *mlink = to_mlink(link);
	unsigned long flags;
	u32 val;

	mbox_dbg("%s:%d\n", __func__, __LINE__);
	spin_lock_irqsave(&mlink->lock, flags);
	/* See NOTE_RX_DONE */
	val = __raw_readl(mlink->tx_reg + INTR_STAT_OFS);
	spin_unlock_irqrestore(&mlink->lock, flags);

	return (val == 0);
}

static int mhu_send_data(struct ipc_link *link, void *data)
{
	struct mhu_link *mlink = to_mlink(link);
	unsigned long flags;

	mbox_dbg("%s:%d\n", __func__, __LINE__);
	if (!mhu_last_tx_done(link)) {
		printk("%s:%d Shouldn't have seen the day!\n",
			__func__, __LINE__);
		return -EBUSY;
	}

	spin_lock_irqsave(&mlink->lock, flags);
	__raw_writel((u32)data, mlink->tx_reg + INTR_SET_OFS);
	spin_unlock_irqrestore(&mlink->lock, flags);

	return 0;
}

static int mhu_startup(struct ipc_link *link, void *ignored)
{
	struct mhu_link *mlink = to_mlink(link);
	unsigned long flags;
	u32 val;
	int ret;

	mbox_dbg("%s:%d\n", __func__, __LINE__);
	spin_lock_irqsave(&mlink->lock, flags);
	val = __raw_readl(mlink->tx_reg + INTR_STAT_OFS);
	__raw_writel(val, mlink->tx_reg + INTR_CLR_OFS);
	spin_unlock_irqrestore(&mlink->lock, flags);

	ret = request_irq(mlink->irq, mhu_rx_interrupt,
		IRQF_SHARED | IRQF_NO_THREAD, mlink->link.link_name, link);
	if (unlikely(ret)) {
		printk("Unable to aquire IRQ\n");
		return ret;
	}

	return 0;
}

static void mhu_shutdown(struct ipc_link *link)
{
	struct mhu_link *mlink = to_mlink(link);

	mbox_dbg("%s:%d\n", __func__, __LINE__);
	free_irq(mlink->irq, link);
}

static struct ipc_link_ops mhu_ops = {
	.send_data = mhu_send_data,
	.startup = mhu_startup,
	.shutdown = mhu_shutdown,
	.last_tx_done = mhu_last_tx_done,
};

static int f_mhu_probe(struct platform_device *pdev)
{
	int i, err;
	struct f_mhu *mhu;
	struct resource *res;
	struct ipc_link *l[4];
	struct mhu_link *mlink;
	int mhu_reg[3] = {0x0, 0x20, 0x200};

	/* Allocate memory for device */
	mhu = kzalloc(sizeof(struct f_mhu), GFP_KERNEL);
	if (!mhu) {
		dev_err(&pdev->dev, "failed to allocate memory.\n");
		return -EBUSY;
	}

	mhu->clk = clk_get(&pdev->dev, "clk");
	if (unlikely(IS_ERR(mhu->clk))) {
		dev_err(&pdev->dev, "unable to init clock\n");
		kfree(mhu);
		return -EINVAL;
	}
	clk_prepare_enable(mhu->clk);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mhu->base = ioremap(res->start, resource_size(res));
	if (!mhu->base) {
		dev_err(&pdev->dev, "ioremap failed.\n");
		kfree(mhu);
		return -EBUSY;
	}
	mhu->dev = &pdev->dev;
	__hackbase = mhu->base;

	/* Let UnTrustedOS's access violations don't bother us */
	__raw_writel(0, mhu->base + MHU_SCFG);

	for (i = 0; i < 3; i++) {
		mlink = &mhu->mlink[i];
		spin_lock_init(&mlink->lock);
		snprintf(mlink->link.link_name, 16, ch_name[i]);
		res = platform_get_resource(pdev, IORESOURCE_IRQ, i);
		mlink->irq = res->start;
		mlink->rx_reg = mhu->base + mhu_reg[i];
		mlink->tx_reg = mlink->rx_reg + 0x100;
		l[i] = &mlink->link;
		mbox_dbg("%s:%d link=%p\n", __func__, __LINE__, l[i]);
	}

	l[3] = NULL;
	mhu->ipc_con.links = l;
	mhu->ipc_con.ops = &mhu_ops;
	mhu->ipc_con.txdone_irq = false;
	mhu->ipc_con.txdone_poll = true;
	mhu->ipc_con.txpoll_period = 10;
	snprintf(mhu->ipc_con.controller_name, 16, "f_mhu");

	platform_set_drvdata(pdev, mhu);

	err = ipc_links_register(&mhu->ipc_con);
	if (err) {
		dev_err(&pdev->dev, "Failed to register mailboxes %d\n", err);
		iounmap(mhu->base);
		kfree(mhu);
	} else {
		dev_info(&pdev->dev, "Fujitsu MHU Mailbox registered\n");
	}

	return 0;
}

static const struct of_device_id f_mhu_dt_ids[] = {
	{ .compatible = "fujitsu,mhu" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, f_mhu_dt_ids);

static struct platform_driver f_mhu_driver = {
	.driver		= {
		.name	= "f_mhu",
		.owner = THIS_MODULE,
		.of_match_table = f_mhu_dt_ids,
	},
	.probe		= f_mhu_probe,
};

static int __init f_mhu_init(void)
{
	return platform_driver_register(&f_mhu_driver);
}
module_init(f_mhu_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Fujitsu MHU Driver");
MODULE_AUTHOR("Jassi Brar <jaswinder.singh@linaro.org>");
