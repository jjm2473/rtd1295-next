
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

static unsigned int pattern = 0xdeadbabe;
module_param(pattern, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(pattern,"Pattern to set (default: 0xdeadbabe)");

static unsigned int bufsize = 32768;
module_param(bufsize, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(bufsize, "Size of buffer to set (default: 32768)");

#define SRC_BUF	128

static DECLARE_COMPLETION(memset_done);
static struct dma_interleaved_template *xt;
static struct dma_chan *chan;
static void *buf;

static void dma_callback(void *data)
{
	complete(&memset_done);
}

static int __init memsettest_init(void)
{
	struct dma_async_tx_descriptor *tx;
	struct dma_device *dmadev;
	dma_cookie_t cookie;
	dma_cap_mask_t mask;
	int ret;

	dma_cap_zero(mask);
	dma_cap_set(DMA_INTERLEAVE, mask);

	chan = dma_request_channel(mask, NULL, NULL);
	if (!chan) {
		printk("%s:%d dma_request_channel!\n", __func__, __LINE__);
		return -EINVAL;
	}
	dmadev = chan->device;

	if (!dma_has_cap(DMA_INTERLEAVE, dmadev->cap_mask)) {
		printk("%s:%d DMA_INTERLEAVE!\n", __func__, __LINE__);
		goto err1;
	}

	bufsize += SRC_BUF;
	buf = kmalloc(bufsize, GFP_KERNEL);
	if (!buf) {
		printk("%s:%d DMA_INTERLEAVE!\n", __func__, __LINE__);
		goto err1;
	}
	*(u32 *)buf = pattern;

	xt = kzalloc(sizeof(struct dma_interleaved_template) +
			sizeof(struct data_chunk), GFP_KERNEL);
	xt->src_inc = 0;
	xt->dst_inc = 1;
	xt->src_sgl = false;
	xt->dst_sgl = true;
	xt->frame_size = 1;
	xt->dir = DMA_MEM_TO_MEM;
	xt->sgl[0].size = 4;
	xt->sgl[0].icg = 0;
	xt->numf = (bufsize - SRC_BUF) / xt->sgl[0].size;
	xt->src_start = dma_map_single(dmadev->dev, buf,
				   bufsize, DMA_BIDIRECTIONAL);
	ret = dma_mapping_error(dmadev->dev, xt->src_start);
	if (ret) {
		printk("%s:%d dma_mapping_error!\n", __func__, __LINE__);
		goto err2;
	}
	xt->dst_start = xt->src_start + SRC_BUF;

	tx = dmadev->device_prep_interleaved_dma(chan, xt, 0);
	if (tx == NULL) {
		printk("%s:%d Error!\n", __func__, __LINE__);
		goto err3;
	}

	tx->callback = dma_callback;
	tx->callback_param = NULL;
	cookie = dmaengine_submit(tx);
	if (dma_submit_error(cookie)) {
		printk("%s:%d Error!\n", __func__, __LINE__);
		goto err3;
	}

	dma_async_issue_pending(chan);
	return 0;
err3:
	dma_unmap_single(dmadev->dev, xt->src_start,
				bufsize, DMA_BIDIRECTIONAL);
err2:
	kfree(xt);
	kfree(buf);
err1:
	dma_release_channel(chan);
	return -EINVAL;
}
late_initcall(memsettest_init);

static void __exit memsettest_exit(void)
{
	struct dma_device *dmadev = chan->device;
	int i, len = bufsize - SRC_BUF;
	u32 *dst = buf + SRC_BUF;

	wait_for_completion(&memset_done);

	printk("Destination pattern (%d bytes)\n", len);
	for (i = 0; i < len / 4; i++)
		printk("%x", *dst++);
	printk("\n");

	dma_unmap_single(dmadev->dev, xt->src_start,
				bufsize, DMA_BIDIRECTIONAL);
	kfree(xt);
	kfree(buf);
	dma_release_channel(chan);
}
module_exit(memsettest_exit);

MODULE_AUTHOR("Jassi");
MODULE_LICENSE("GPL v2");
