#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/freezer.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/iommu.h>
#include <linux/sizes.h>
#include <linux/cache.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <asm/dma-iommu.h>


unsigned int channel = 0;
module_param(channel, uint, S_IRUGO);
MODULE_PARM_DESC(channel, "number of channels to use (default: 0)");


struct mb86s7x_dmatest {
	size_t test_length;
	struct completion completion;
	struct device *dev;
	struct dma_chan *chan;
	struct iommu_domain *domain;
	unsigned int *scr_cpu, *dst_cpu;
	dma_addr_t src_phy_addr, dst_phy_addr;
};

struct class class = {
	.name = "testclass",
	.owner = THIS_MODULE,
};

struct mb86s7x_dmatest _priv;
struct mb86s7x_dmatest *priv = &_priv;


static void dmatest_callback(void *dma_async_param)
{
	struct mb86s7x_dmatest *priv = dma_async_param;

	complete(&priv->completion);
}

void show_data(struct device *dev, unsigned int *src, unsigned int *dst)
{
	int n = 0;

	for (n = 0; n < 24; n += 4)
		dev_err(dev, "SRC->%02x %02x %02x %02x\n",
		     src[n], src[n + 1], src[n + 2], src[n + 3]);

	for (n = 0; n < 24; n += 4)
		dev_err(dev, "DST->%02x %02x %02x %02x\n",
		      dst[n], dst[n + 1], dst[n + 2], dst[n + 3]);
}


static int dmatest_do_copy_blocking(struct mb86s7x_dmatest *priv,
				dma_addr_t src, dma_addr_t dest, size_t len)
{
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
	int ret = 0;
	enum dma_status status;

	if (!is_dma_copy_aligned(priv->chan->device, src, dest, len))
		dev_err(priv->dev, "not aligned\n");

	/* prepare the transfer */
	desc = priv->chan->device->device_prep_dma_memcpy(
		priv->chan, dest, src, len, DMA_PREP_INTERRUPT);
	if (!desc) {
		ret = -EINVAL;
		goto bail;
	}

	desc->callback = dmatest_callback;
	desc->callback_param = priv;

	/* set the transfer going */

	init_completion(&priv->completion);
	cookie = dmaengine_submit(desc);
	dma_async_issue_pending(priv->chan);

	/* wait for it to complete */

	ret = wait_for_completion_timeout(&priv->completion,
							msecs_to_jiffies(500));
	if (ret <= 0) {
		dev_err(priv->dev, "timeout waiting completion\n");
		ret = -ETIME;
		goto bail;
	} else
		ret = 0;

	status = dma_async_is_tx_complete(priv->chan, cookie, NULL, NULL);

	switch (dma_async_is_tx_complete(priv->chan, cookie, NULL, NULL)) {
	case DMA_SUCCESS:
		break;
	case DMA_IN_PROGRESS:
		dev_err(priv->dev, "Status: DMA_IN_PROGRESS\n");
		break;
	case DMA_PAUSED:
		dev_err(priv->dev, "Status: DMA_PAUSED\n");
		break;
	case DMA_ERROR:
		dev_err(priv->dev, "Status: DMA_ERROR\n");
		break;
	}

	dev_err(priv->dev, "mb86s70-dmatest: completed\n");

bail:
	dev_err(priv->dev, "%s: %llX -> %llX len %llX returned %d\n", __func__,
		(u64)src, (u64)dest, (u64)len, ret);

	return ret;
}


static bool dma_filter(struct dma_chan *chan, void *param)
{
	printk(KERN_ERR "%s dma chan %d\n", __func__, chan->chan_id);
	if (chan->chan_id == channel)
		return true;
	else
		return false;
}


static int __init dmatest_init(void)
{
	dma_cap_mask_t mask;
	static u64 dma_mask = 0xffffffffffffffff;
	int ret = 0;

	/* init */
	priv->test_length = SZ_4K;

	/* fake up a device */

	ret = class_register(&class);
	if (ret)
		return -EINVAL;

	priv->dev = device_create(&class, NULL,
	    MKDEV(99, 99), &priv, "mb86s70-dmatest");
	if (IS_ERR(priv->dev)) {
		pr_err("mb86s70-dmatest: failed to create device: %d\n",
		    (int)priv->dev);
		ret = (int)priv->dev;
		goto bail_unreg_class;
	}

	priv->dev->dma_mask = &dma_mask;
	priv->dev->coherent_dma_mask = dma_mask;

	/* get a DMA channel that can do a memcpy */
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	priv->chan = dma_request_channel(mask, dma_filter, NULL);
	if (!priv->chan)
		goto bail_dev;

	dev_err(priv->dev, "Using DMA device %s\n",
				dev_name(priv->chan->device->dev));

	/* allocate coherent source and dest buffers */
	priv->scr_cpu = (unsigned int *)dma_alloc_coherent(priv->chan->device->dev,
	    priv->test_length, &priv->src_phy_addr, GFP_KERNEL);
	if (!priv->scr_cpu) {
		dev_err(priv->dev, "Unable to allocate test buffer\n");
		ret = -ENOMEM;
		goto bail_chan;
	}
	dev_err(priv->dev, "phys1 -> 0x%lx\n", (unsigned long) priv->src_phy_addr);

	priv->dst_cpu = (unsigned int *)dma_alloc_coherent(priv->chan->device->dev,
	    priv->test_length, &priv->dst_phy_addr, GFP_KERNEL);
	if (!priv->dst_cpu) {
		dev_err(priv->dev, "Unable to allocate test buffer\n");
		ret = -ENOMEM;
		goto bail_a1;
	}
	dev_err(priv->dev, "phys2 -> 0x%lx\n", (unsigned long) priv->dst_phy_addr);

	/* set phys1 contents VIR_SRC_LOW */
	/* set phys2 contents VIR_DST_LOW */
	memset(priv->scr_cpu, 0xcc, priv->test_length);
	memset(priv->dst_cpu, 0x88, priv->test_length);

	dmatest_do_copy_blocking(priv, priv->src_phy_addr,
	    priv->dst_phy_addr, priv->test_length);

	show_data(priv->dev, priv->scr_cpu, priv->dst_cpu);

	if (memcmp(priv->scr_cpu, priv->dst_cpu, priv->test_length) == 0)
		dev_err(priv->dev, "src 0x%llx dst 0x%llx memcmp ok!\n\n\n",
		    (u64)priv->src_phy_addr, (u64)priv->dst_phy_addr);
	else
		dev_err(priv->dev, "src 0x%llx dst 0x%llx memcmp fail!\n\n\n",
		    (u64)priv->src_phy_addr, (u64)priv->dst_phy_addr);

		return ret;

bail_a1:
	dma_free_coherent(priv->dev, priv->test_length, priv->scr_cpu, priv->src_phy_addr);

bail_chan:
	dma_release_channel(priv->chan);

bail_dev:
	device_destroy(NULL, MKDEV(99, 99));
bail_unreg_class:
	class_unregister(&class);
	return -1;
}
/* when compiled-in wait for drivers to load first */

module_init(dmatest_init);
static void __exit dmatest_exit(void)
{
	dev_err(priv->dev, "%s\n", __func__);
	dma_free_coherent(priv->dev, priv->test_length, priv->dst_cpu, priv->dst_phy_addr);
	dma_free_coherent(priv->dev, priv->test_length, priv->scr_cpu, priv->src_phy_addr);
	dma_release_channel(priv->chan);
	device_destroy(&class, MKDEV(99, 99));
	class_unregister(&class);
}
module_exit(dmatest_exit);

MODULE_AUTHOR("Andy Green <andy.green@linaro.org>");
MODULE_LICENSE("GPL v2");

