/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/f_ipcu.h>
#include <linux/mailbox_controller.h>

#define MAR(i)		(0x80 + (i) * 4)
#define OFFSET_MBOX(m)	(0x100 + 0x80 * (m))
#define SRC_REG(m)	(OFFSET_MBOX(m) + 0x0)
#define MODE_REG(m)	(OFFSET_MBOX(m) + 0x4)
#define SEND_REG(m)	(OFFSET_MBOX(m) + 0x8)
#define DST_SET(m)	(OFFSET_MBOX(m) + 0x10)
#define DST_CLR(m)	(OFFSET_MBOX(m) + 0x14)
#define DST_STAT(m)	(OFFSET_MBOX(m) + 0x18)
#define ACK_SET(m)	(OFFSET_MBOX(m) + 0x30)
#define ACK_CLR(m)	(OFFSET_MBOX(m) + 0x34)
#define ACK_STAT(m)	(OFFSET_MBOX(m) + 0x38)
#define DAT_REG(m)	(OFFSET_MBOX(m) + 0x40)

#define IPCU_DSTMASK	0xffff /* 16 max cpu interfaces */

struct ipcu_mbox {
	/* The CPU i/f that owns this mailbox */
	int cpuif;
	/* Physical mailbox this structure represents */
	int id;
	/* Parent IPCU controller */
	struct ipcu *ipcu;
	struct ipc_link link;
};

struct ipcu {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	/* Number of mailboxes */
	int mboxes;
	/* Array of mailboxes */
	struct ipcu_mbox *mbox;
	/* Number of irq interfaces */
	int ifaces;
	/* Array of irqs */
	int *irq;
	struct ipc_controller ipc_con;
};

static inline struct ipcu_mbox *to_mbox(struct ipc_link *l)
{
	if (!l)
		return NULL;

	return container_of(l, struct ipcu_mbox, link);
}

static irqreturn_t ipcu_interrupt(int irq, void *data)
{
	struct ipcu_mbox *mbox = data;
	struct ipcu *ipcu = mbox->ipcu;
	u32 ack, mar;
	int i, src;

	mar = readl(ipcu->base + MAR(mbox->cpuif));
	src = ((mar & 0xfff) - 0x100) / 0x80;

	if (mar & (1 << 13)) { /* ACK */
		ack = readl(ipcu->base + ACK_STAT(src));
		writel(ack, ipcu->base + ACK_CLR(src));
		writel(0, ipcu->base + SEND_REG(src));
		ipc_link_txdone(&mbox->link, XFER_OK);

		return IRQ_HANDLED;
	}

	if (mar & (1 << 12)) { /* REQ */
		struct ipcu_mssg mssg;

		/* Read the data */
		for (i = 0; i < ARRAY_SIZE(mssg.data); i++)
			mssg.data[i] = readl(ipcu->base +
					DAT_REG(src) + i * 4);
		mssg.mask = readl(ipcu->base + SRC_REG(src));

		/* Handover the message to client */
		ipc_link_received_data(&mbox->link, (void *)&mssg);

		writel(1 << mbox->cpuif, ipcu->base + DST_CLR(src));
		writel(1 << mbox->cpuif, ipcu->base + ACK_SET(src));

		return IRQ_HANDLED;
	}

	mbox_dbg("%s:%d !!!\n", __func__, __LINE__);

	return IRQ_HANDLED;
}

static int ipcu_send_data(struct ipc_link *link, void *data)
{
	struct ipcu_mssg *mssg = data;
	struct ipcu_mbox *mbox = to_mbox(link);
	struct ipcu *ipcu = mbox->ipcu;
	int i;

	mbox_dbg("%s:%d\n", __func__, __LINE__);

	/* Reject if the mailbox is readonly */
	if (mbox->id == ipcu->mboxes)
		return -EIO;

	/* Reject if there is any invalid destination */
	if (mssg->mask & ~IPCU_DSTMASK)
		return -EINVAL;

	/* Reject if busy */
	if (readl(ipcu->base + SEND_REG(mbox->id)))
		return -EBUSY;

	writel(mssg->mask, ipcu->base + DST_SET(mbox->id));

	/* Fill the data */
	for (i = 0; i < 9; i++)
		writel(mssg->data[i],
				ipcu->base + DAT_REG(mbox->id) + i * 4);

	/* Trigger */
	writel(1, ipcu->base + SEND_REG(mbox->id));

	return 0;
}

static int ipcu_startup(struct ipc_link *link, void *params)
{
	struct ipcu_mbox *mbox = to_mbox(link);
	struct ipcu_client *cl = params;
	struct ipcu *ipcu = mbox->ipcu;
	int i, ret;
	u32 val;

	mbox_dbg("%s:%d\n", __func__, __LINE__);

	if (cl->iface >= ipcu->ifaces)
		return -EINVAL;

	/* Fail if the cpu i/f already owns some other mailbox */
	for (i = 0; i < ipcu->mboxes; i++)
		if (ipcu->mbox[i].cpuif == cl->iface)
			return -EBUSY;

	/* The mbox structure doesn't own a physical mbox yet */
	mbox->id = ipcu->mboxes;

	/* If the mailbox is not going to be used as read-only */
	if (!cl->ro) {
		/* Try to exclusively own a mailbox */
		for (i = 0; i < ipcu->mboxes; i++) {
			writel(1 << cl->iface, ipcu->base + SRC_REG(i));
			mb();
			val = readl(ipcu->base + SRC_REG(i));
			if (val == (1 << cl->iface)) {
				/* Set to Mode1 */
				writel(0, ipcu->base + MODE_REG(i));
				break;
			}
		}
		/* If no mailbox is available */
		if (i == ipcu->mboxes)
			return -EAGAIN;

		/* The mbox structure owns i'th physical mbox */
		mbox->id = i;
	}

	/* Assign the mailbox to the cpu i/f */
	mbox->cpuif = cl->iface;

	ret = request_irq(ipcu->irq[cl->iface], ipcu_interrupt,
				IRQF_SHARED, mbox->link.link_name, mbox);
	if (unlikely(ret)) {
		printk("Unable to aquire IRQ\n");
		if (mbox->id != ipcu->mboxes)
			writel(0, ipcu->base + SRC_REG(mbox->id));
		mbox->cpuif = -1;
		mbox->id = ipcu->mboxes;
	}

	return ret;
}

static void ipcu_shutdown(struct ipc_link *link)
{
	struct ipcu_mbox *mbox = to_mbox(link);
	struct ipcu *ipcu = mbox->ipcu;

	mbox_dbg("%s:%d\n", __func__, __LINE__);

	free_irq(ipcu->irq[mbox->cpuif], mbox);

	if (mbox->id != ipcu->mboxes)
		writel(0, ipcu->base + SRC_REG(mbox->id));

	mbox->id = ipcu->mboxes;
	mbox->cpuif = -1;
}

static struct ipc_link_ops ipcu_ops = {
	.send_data = ipcu_send_data,
	.startup = ipcu_startup,
	.shutdown = ipcu_shutdown,
};

static int f_ipcu_probe(struct platform_device *pdev)
{
	struct ipcu_mbox *mbox;
	int i, mboxes, ifaces;
	struct resource	*res;
	struct ipc_link **l;
	struct ipcu *ipcu;
	const int *p;
	int ret;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "Requires DT node\n");
		return -ENODEV;
	}

	p = of_get_property(pdev->dev.of_node, "mboxes", NULL);
	if (!p) {
		dev_err(&pdev->dev, "Requires DT \"mboxes\" property\n");
		return -ENODEV;
	}
	mboxes = be32_to_cpu(*p);

	/* Calculate number of local cpu interfaces */
	ifaces = 0;
	do {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, ifaces++);
	} while (res);
	ifaces--;

	if (!ifaces) {
		dev_err(&pdev->dev, "Requires DT \"interrupts\" property\n");
		return -ENODEV;
	}

	/* Allocate memory for device */
	ipcu = kzalloc(sizeof(struct ipcu), GFP_KERNEL);
	if (!ipcu)
		return -ENOMEM;

	ipcu->dev = &pdev->dev;
	ipcu->mboxes = mboxes;
	ipcu->ifaces = ifaces;
	ipcu->irq = kmalloc(sizeof(int) * ifaces, GFP_KERNEL);
	if (!ipcu->irq) {
		ret = -ENOMEM;
		goto fail1;
	}
	ipcu->mbox = kmalloc(sizeof(struct ipcu_mbox) * mboxes, GFP_KERNEL);
	if (!ipcu->mbox) {
		ret = -ENOMEM;
		goto fail2;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no mmio resource defined.\n");
		ret = -ENXIO;
		goto fail3;
	}
	ipcu->base = ioremap(res->start, resource_size(res));
	if (!ipcu->base) {
		dev_err(&pdev->dev, "ioremap failed.\n");
		ret = -ENXIO;
		goto fail3;
	}

	for (i = 0; i < ifaces; i++) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, i);
		ipcu->irq[i] = res->start;
	}

	l = kzalloc((mboxes + 1) * sizeof(struct ipc_link *), GFP_KERNEL);
	if (!l) {
		ret = -ENOMEM;
		goto l_fail;
	}

	for (i = 0; i < mboxes; i++) {
		mbox = &ipcu->mbox[i];
		mbox->id = ipcu->mboxes;
		mbox->cpuif = -1;
		mbox->ipcu = ipcu;
		l[i] = &mbox->link;
		snprintf(mbox->link.link_name,
			 sizeof(mbox->link.link_name), "mbox");
		mbox_dbg("%s:%d link=%p\n", __func__, __LINE__, l[i]);
	}

	l[mboxes] = NULL;
	ipcu->ipc_con.links = l;
	ipcu->ipc_con.ops = &ipcu_ops;
	ipcu->ipc_con.txdone_irq = true;
	/* Every mailbox is named "f_ipcu:mbox" */
	snprintf(ipcu->ipc_con.controller_name,
		 sizeof(ipcu->ipc_con.controller_name), "f_ipcu");

	platform_set_drvdata(pdev, ipcu);

	ret = ipc_links_register(&ipcu->ipc_con);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register mailboxes %d\n", ret);
		goto fail;
	}
	kfree(l);

	dev_info(&pdev->dev, "IPCU Mailbox registered with %d interfaces\n",
		 ifaces);

	return 0;

fail:
	kfree(l);
l_fail:
	iounmap(ipcu->base);
fail3:
	kfree(ipcu->mbox);
fail2:
	kfree(ipcu->irq);
fail1:
	kfree(ipcu);

	return ret;
}

static int __exit f_ipcu_remove(struct platform_device *pdev)
{
	struct ipcu *ipcu = platform_get_drvdata(pdev);

	ipc_links_unregister(&ipcu->ipc_con);
	iounmap(ipcu->base);
	kfree(ipcu->mbox);
	kfree(ipcu->irq);
	kfree(ipcu);

	return 0;
}

static const struct of_device_id f_ipcu_dt_ids[] = {
	{ .compatible = "fujitsu,ipcu" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, f_ipcu_dt_ids);

static struct platform_driver f_ipcu_driver = {
	.driver		= {
		.name	= "f_ipcu",
		.owner = THIS_MODULE,
		.of_match_table = f_ipcu_dt_ids,
	},
	.probe		= f_ipcu_probe,
	.remove		= __exit_p(f_ipcu_remove),
};

static int __init f_ipcu_init(void)
{
	return platform_driver_register(&f_ipcu_driver);
}
module_init(f_ipcu_init);

static void __exit f_ipcu_exit(void)
{
	platform_driver_unregister(&f_ipcu_driver);
}
module_exit(f_ipcu_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Fujitsu IPCU Driver");
MODULE_AUTHOR("Jassi Brar <jaswinder.singh@linaro.org>");
