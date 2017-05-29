/*
 * Generic media shell driver for userland drivers
 * Copyright (C) 2013 Fujitsu Semiconductor, Ltd
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/firmware.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/pm_runtime.h>
#include <uapi/linux/f_media-shell.h>
#include <asm/dma-iommu.h>
#include <linux/iommu.h>

#define MS_PLATFORM_DEVICE_NAME		"media-shell"

#define MAX_MS_INSTANCES			4
/* #define MS_SUPPORT_RESERVED_VIDEO_MEMORY */
#ifdef MS_SUPPORT_RESERVED_VIDEO_MEMORY
#define MS_DRAM_PHYSICAL_BASE			0x70000000
#endif

/* REGISTER BASE */
#define BIT_BASE		0x0000
/* HARDWARE REGISTER */
#define BIT_CODE_RUN		(BIT_BASE + 0x000)
#define BIT_CODE_DOWN		(BIT_BASE + 0x004)
#define BIT_INT_CLEAR		(BIT_BASE + 0x00C)
#define BIT_CODE_RESET		(BIT_BASE + 0x014)
/* GLOBAL REGISTER */
#define BIT_BUSY_FLAG		(BIT_BASE + 0x160)
#define MS_BIT_CODE_HEAD	(1 * 1024 / 2) /* Write 2 bytes 1 time*/

#define WAVE320_FW_NAME		"wave320-firmware.bin"
/* #define SUPPORT_DRM_GEM */

MODULE_FIRMWARE(WAVE320_FW_NAME);

enum {
	IP_TYPE_F_JPEGTX,
	IP_TYPE_WAVE320,

	/* always last */
	IP_TYPE_COUNT
};

static char *ip_type_name[] = {
	"f_jpegtx",
	"f_wave320",
};

static char *ip_type_devname[] = {
	"jpu%d",
	"vpu%d",
};


#define MS_REG_STORE_NUM	64

struct ms {
	struct device *dev;
	struct cdev *cdev;
	struct fasync_struct *async_queue;
	void __iomem *base;
	int irq;
	u32 phy_base;
	struct clk *pclk;
	struct clk *aclk;
	bool clocks_enabled;
	u32 open_count;
	struct mutex lock;
	spinlock_t spinlock;
	struct list_head jbp_head;
	struct ms_buffer instance_pool;
	struct ms_buffer common_memory;
	int interrupt_flag;
	wait_queue_head_t interrupt_wait_q;
	int driver_type;
	u32 ms_reg_store[MS_REG_STORE_NUM];
	struct dma_attrs dma_attrs;
};

/* To track the allocated memory buffer */
struct ms_buffer_pool {
	struct list_head list;
	struct ms_buffer vb;
};

static dev_t dev_node[IP_TYPE_COUNT];

static const struct of_device_id media_shell_dt_ids[] = {
	{
		.compatible = "fujitsu,jpegtx",
		.data = (void *)IP_TYPE_F_JPEGTX,
	},
	{
		.compatible = "fujitsu,wave320",
		.data = (void *)IP_TYPE_WAVE320,
	},
	{ /* sentinel */ }
};

static struct class *ms_class;
static struct ms *mss[IP_TYPE_COUNT][MAX_MS_INSTANCES];
#define ms_reg_read(_ms, addr) __raw_readl(_ms->base + addr)
#define ms_reg_write(_ms, addr, val) __raw_writel(val, _ms->base + addr)

int ms_hw_reset(struct ms *ms)
{
	dev_info(ms->dev, "request ms reset from applciation.\n");
	return 0;
}


static int ms_alloc_dma_buffer(struct ms *ms, struct ms_buffer *vb)
{
	vb->base = dma_alloc_attrs(ms->dev,
		PAGE_ALIGN(vb->size),
		(dma_addr_t *)(&vb->phys_addr),
		GFP_DMA | GFP_KERNEL, &ms->dma_attrs);

	if (vb->base)
		return 0;
	dev_err(ms->dev, "Physmem alloc error size=%d\n", vb->size);
	return -ENOMEM;
}

static void ms_free_dma_buffer(struct ms_buffer *jb, struct ms *ms)
{
	if (!jb->base || !ms)
		return;

	dma_free_attrs(ms->dev, PAGE_ALIGN(jb->size),
		jb->base, jb->phys_addr, &ms->dma_attrs);
}

static int ms_free_buffers(struct ms *ms)
{
	struct ms_buffer_pool *pool, *n;
	struct ms_buffer vb;

	list_for_each_entry_safe(pool, n, &ms->jbp_head, list) {
		vb = pool->vb;
		if (!vb.base)
			continue;

		ms_free_dma_buffer(&vb, ms);
		list_del(&pool->list);
		kfree(pool);
	}
	return 0;
}

static irqreturn_t irq_handler(int irq, void *dev_id)
{
	struct ms *ms = dev_id;

	if (ms->driver_type == IP_TYPE_WAVE320)
		ms_reg_write(ms, BIT_INT_CLEAR, 0x1);

	if (ms->async_queue)
		kill_fasync(&ms->async_queue, SIGIO, POLL_IN);

	spin_lock(&ms->spinlock);
	ms->interrupt_flag = 1;

	dev_dbg(ms->dev, "irq_handler\n");
	wake_up_interruptible(&ms->interrupt_wait_q);

	disable_irq_nosync(ms->irq);
	spin_unlock(&ms->spinlock);
	return IRQ_HANDLED;
}

static int ms_open(struct inode *inode, struct file *filp)
{
	int minor;
	struct ms *ms;
	int n;

	minor = iminor(inode);
	if (minor > (MAX_MS_INSTANCES-1))
		return -ENODEV;

	for (n = 0; n < IP_TYPE_COUNT; n++)
		if (imajor(inode) == MAJOR(dev_node[n]))
			break;
	if (n == IP_TYPE_COUNT) {
		pr_err("Unknown dev mapping\n");
		return -ENODEV;
	}

	if (!mss[n][minor])
		return -ENODEV;

	ms = mss[n][minor];

	mutex_lock(&ms->lock);

	ms->open_count++;

	filp->private_data = ms;
	mutex_unlock(&ms->lock);

	pm_runtime_get_sync(ms->dev);

	return 0;
}

static long ms_ioctl(struct file *filp, u_int cmd, u_long arg)
{
	struct ms *ms = filp->private_data;
	int ret = 0;
	struct ms_buffer_pool *vbp = NULL, *n;
	struct ms_buffer vb;
	u32 timeout;
	u32 clkgate;
	unsigned long flags;

	switch (cmd) {
	case JDI_IOCTL_GET_RESERVED_VIDEO_MEMORY_INFO:
		mutex_lock(&ms->lock);
		if (ms->common_memory.base != 0) {
			ret = copy_to_user(
				(void __user *)arg, &ms->common_memory,
				sizeof(struct ms_buffer));

			if (ret != 0)
				ret = -EFAULT;
		} else
			ret = -EFAULT;

		mutex_unlock(&ms->lock);
		break;

	case JDI_IOCTL_ALLOCATE_PHYSICAL_MEMORY:
		vbp = kzalloc(sizeof(*vbp), GFP_KERNEL);
		if (!vbp)
			return -ENOMEM;
		ret = copy_from_user(&(vbp->vb), (struct ms_buffer *)arg,
						sizeof(struct ms_buffer));
		if (ret) {
			kfree(vbp);
			return -EFAULT;
		}

		ret = ms_alloc_dma_buffer(ms, &vbp->vb);
		if (ret == -1) {
			kfree(vbp);
			break;
		}

		ret = copy_to_user((void __user *)arg, &vbp->vb,
						sizeof(struct ms_buffer));

		if (ret) {
			kfree(vbp);
			ret = -EFAULT;
			break;
		}

		mutex_lock(&ms->lock);
		list_add(&vbp->list, &ms->jbp_head);
		mutex_unlock(&ms->lock);
		break;

	case JDI_IOCTL_FREE_PHYSICALMEMORY:
		ret = copy_from_user(&vb, (struct ms_buffer *)arg,
						sizeof(struct ms_buffer));

		if (ret)
			return -EACCES;

		if (vb.base)
			ms_free_dma_buffer(&vb, ms);

		mutex_lock(&ms->lock);
		list_for_each_entry_safe(vbp, n, &ms->jbp_head, list) {
			if (vbp->vb.base != vb.base)
				continue;
			list_del(&vbp->list);
			kfree(vbp);
			break;
		}
		mutex_unlock(&ms->lock);
		break;

	case JDI_IOCTL_WAIT_INTERRUPT:
		timeout = (u32)arg;
		if (!wait_event_interruptible_timeout(
			ms->interrupt_wait_q, ms->interrupt_flag != 0,
					msecs_to_jiffies(timeout))) {
			ret = -ETIME;
			break;
		}

		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}

		spin_lock_irqsave(&ms->spinlock, flags);
		ms->interrupt_flag = 0;
		enable_irq(ms->irq);
		spin_unlock_irqrestore(&ms->spinlock, flags);
		break;

	case JDI_IOCTL_SET_CLOCK_GATE:
		if (get_user(clkgate, (u32 __user *) arg))
			return -EFAULT;

		ms->clocks_enabled = clkgate;

		if (clkgate) {
			clk_prepare_enable(ms->pclk);
			clk_prepare_enable(ms->aclk);
		} else {
			clk_disable_unprepare(ms->pclk);
			clk_disable_unprepare(ms->aclk);
		}
		break;

	case JDI_IOCTL_GET_INSTANCE_POOL:
		mutex_lock(&ms->lock);

		if (ms->instance_pool.base) {
			ret = copy_to_user((void __user *)arg,
				&ms->instance_pool, sizeof(struct ms_buffer));
			if (ret)
				ret = -EFAULT;
			goto bail;
		}


		ret = copy_from_user(&ms->instance_pool,
			(struct ms_buffer *)arg, sizeof(struct ms_buffer));
		if (ret)
			goto bail;

		if (ms_alloc_dma_buffer(ms, &ms->instance_pool) == -1)
			goto bail;

		ret = copy_to_user((void __user *)arg,
					&ms->instance_pool,
					sizeof(struct ms_buffer));

		if (!ret) {
			/* success to get memory for instance pool */
			mutex_unlock(&ms->lock);
			break;
		}

		ret = -EFAULT;
bail:
		mutex_unlock(&ms->lock);
		break;

	case JDI_IOCTL_RESET:
		ms_hw_reset(ms);
		break;

	default:
		pr_err("Unknown IOCTL %d\n", cmd);
		break;
	}
	return ret;
}

static ssize_t ms_read(struct file *filp, char __user *buf,
						size_t len, loff_t *ppos)
{
	return -1;
}

static ssize_t ms_write(struct file *filp, const char __user *buf,
						size_t len, loff_t *ppos)
{
	return -1;
}

static int ms_release(struct inode *inode, struct file *filp)
{
	struct ms *ms = filp->private_data;

	mutex_lock(&ms->lock);
	ms->open_count--;

	if (ms->open_count <= 0)
		/* found and free the not handled buffer by user applications */
		ms_free_buffers(ms);

	mutex_unlock(&ms->lock);

	pm_runtime_put_sync_suspend(ms->dev);

	return 0;
}


static int ms_fasync(int fd, struct file *filp, int mode)
{
	struct ms *dev = filp->private_data;

	return fasync_helper(fd, filp, mode, &dev->async_queue);
}


static int ms_map_to_register(struct ms *ms, struct file *fp,
						struct vm_area_struct *vm)
{
	unsigned long pfn;

	vm->vm_flags |= VM_IO;
	vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
	pfn = (unsigned long)ms->phy_base >> PAGE_SHIFT;

	return remap_pfn_range(vm, vm->vm_start, pfn,
		vm->vm_end-vm->vm_start, vm->vm_page_prot) ? -EAGAIN : 0;
}

static int ms_map_to_physical_memory(struct file *fp,
					struct vm_area_struct *vm)
{
	struct ms *ms;
	struct dma_iommu_mapping *mapping;
	phys_addr_t physaddr;
	ms = fp->private_data;
	vm->vm_flags |= VM_IO;
	vm->vm_page_prot = pgprot_writecombine(vm->vm_page_prot);

	mapping = to_dma_iommu_mapping(ms->dev);

	if(mapping) {
		physaddr = iommu_iova_to_phys(
					mapping->domain, vm->vm_pgoff << PAGE_SHIFT);
		vm->vm_pgoff = physaddr >> PAGE_SHIFT;
	}

	return remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff,
		vm->vm_end-vm->vm_start, vm->vm_page_prot) ? -EAGAIN : 0;
}

static int ms_mmap(struct file *filp, struct vm_area_struct *vm)
{
	struct ms *ms = filp->private_data;

	if (vm->vm_pgoff)
		return ms_map_to_physical_memory(filp, vm);
	else
		return ms_map_to_register(ms, filp, vm);
}

static const struct file_operations ms_fops = {
	.owner = THIS_MODULE,
	.open = ms_open,
	.read = ms_read,
	.write = ms_write,
	.unlocked_ioctl = ms_ioctl,
	.release = ms_release,
	.fasync = ms_fasync,
	.mmap = ms_mmap,
};

static int ms_probe(struct platform_device *pdev)
{
	int err = 0;
	struct resource *res;
	struct ms *ms;
	const u32 *p;
	const struct of_device_id *of_id =
		of_match_device(media_shell_dt_ids, &pdev->dev);
	int id;
	struct dma_iommu_mapping *mapping;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "DT required\n");
		return -EINVAL;
	}

	ms = kzalloc(sizeof(*ms), GFP_KERNEL);
	if (!ms)
		return -ENOMEM;

	ms->driver_type = (int)of_id->data;
	ms->dev = &pdev->dev;
	mutex_init(&ms->lock);
	spin_lock_init(&ms->spinlock);
	INIT_LIST_HEAD(&ms->jbp_head);
	init_waitqueue_head(&ms->interrupt_wait_q);

	dev_set_drvdata(&pdev->dev, ms);
	init_dma_attrs(&ms->dma_attrs);
	mapping = to_dma_iommu_mapping(ms->dev);
	if (mapping) {
		dev_info(ms->dev,
			"Support IOMMU and get contiguous DMA memory\n");
		dma_set_attr(DMA_ATTR_FORCE_CONTIGUOUS, &ms->dma_attrs);
	}
	
        p = of_get_property(pdev->dev.of_node, "memory-size", NULL);
        if (!p) {
                dev_err(&pdev->dev, "Missing id property\n");
                return -EINVAL;
	}
        ms->common_memory.size = be32_to_cpu(*p);
	dev_info(&pdev->dev, "Reserved %d bytes\n", ms->common_memory.size);

#ifndef SUPPORT_DRM_GEM
#ifdef MS_SUPPORT_RESERVED_VIDEO_MEMORY
	ms->common_memory.phys_addr = MS_DRAM_PHYSICAL_BASE;
	ms->common_memory.base = ioremap(ms->common_memory.phys_addr,
					PAGE_ALIGN(ms->common_memory.size));
	if (!ms->common_memory.base) {
		err = -ENOMEM;
		goto bail_free;
	}
#else
	if (ms_alloc_dma_buffer(ms, &ms->common_memory) == -1) {
		err = -ENOMEM;
		goto bail_free;
	}
#endif
#endif /* SUPPORT_DRM_GEM */

	p = of_get_property(pdev->dev.of_node, "id", NULL);
	if (!p) {
		dev_err(&pdev->dev, "Missing id property\n");
		goto bail_dma_unalloc;
	}
	id = be32_to_cpu(*p);
	mss[ms->driver_type][id] = ms;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Missing resource\n");
		err = -EINVAL;
		goto bail_dma_unalloc;
	}

	ms->phy_base = res->start;
	ms->base = ioremap(res->start, res->end - res->start);
	if (!ms->base) {
		err = -ENOMEM;
		goto bail_dma_unalloc;
	}

	ms->pclk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(ms->pclk)) {
		dev_err(&pdev->dev, "Failed to get pclock");
		err = -EINVAL;
		goto bail_iounmap;
	}

	ms->aclk = of_clk_get(pdev->dev.of_node, 1);
	if (IS_ERR(ms->aclk)) {
		dev_err(&pdev->dev, "Failed to get aclock");
		err = -EINVAL;
		goto bail_clk_put;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "Missing resource\n");
		err = -EINVAL;
		goto bail_clk_put;
	}

	ms->irq = res->start;

	err = request_irq(ms->irq, irq_handler, 0, "MS_CODEC_IRQ", ms);
	if (err) {
		dev_err(&pdev->dev, "failed to register IRQ\n");
		goto bail_clk_put;
	}

	ms->cdev = cdev_alloc();
	cdev_init(ms->cdev, &ms_fops);

	if (cdev_add(ms->cdev, dev_node[ms->driver_type] + id, 1)) {
		dev_err(&pdev->dev, "failed to create char device\n");
		goto bail_irq;
	}
	if (IS_ERR(device_create(ms_class, NULL, dev_node[ms->driver_type] + id,
				 NULL, ip_type_devname[ms->driver_type], id))) {
		dev_err(&pdev->dev, "failed to create char device\n");
		goto bail_cdev;
	}

	pm_runtime_enable(&pdev->dev);

	dev_info(&pdev->dev, "%s at pa=0x%x, va=0x%p, IRQ %d, node: %d:%d\n",
			ip_type_name[ms->driver_type],
			ms->phy_base , ms->base, ms->irq,
				MAJOR(dev_node[ms->driver_type]),
				MINOR(dev_node[ms->driver_type]) + id);

	return 0;

bail_cdev:
	cdev_del(ms->cdev);
bail_irq:
	free_irq(ms->irq, ms);
bail_clk_put:
	clk_put(ms->pclk);
	clk_put(ms->aclk);
bail_iounmap:
	iounmap(ms->base);
bail_dma_unalloc:
#ifndef SUPPORT_DRM_GEM
	if (ms->common_memory.base) {
#ifdef MS_SUPPORT_RESERVED_VIDEO_MEMORY
			iounmap((void *)ms->common_memory.base);
#else
			ms_free_dma_buffer(&ms->common_memory, ms);
#endif
	}
#endif /* SUPPORT_DRM_GEM */
bail_free:
	kfree(ms);

	return err;
}

static int ms_remove(struct platform_device *pdev)
{
	struct ms *ms = dev_get_drvdata(&pdev->dev);

	if (ms->instance_pool.base)
		ms_free_dma_buffer(&ms->instance_pool, ms);

	pm_runtime_disable(&pdev->dev);

	cdev_del(ms->cdev);
	free_irq(ms->irq, ms);

#ifndef SUPPORT_DRM_GEM
	if (ms->common_memory.base) {
#ifdef MS_SUPPORT_RESERVED_VIDEO_MEMORY
			iounmap((void *)ms->common_memory.base);
#else
			ms_free_dma_buffer(&ms->common_memory, ms);
#endif
	}
#endif /* SUPPORT_DRM_GEM */

	clk_disable_unprepare(ms->pclk);
	clk_disable_unprepare(ms->aclk);
	clk_put(ms->pclk);
	clk_put(ms->aclk);
	iounmap(ms->base);

	return 0;
}

#ifdef CONFIG_PM

static int ms_device_runtime_idle(struct device *dev)
{
	return 1; /* no runtime_suspend */
}

static int ms_device_runtime_suspend(struct device *dev)
{
	struct ms *ms = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);

	/* disable_irq(ms->irq); */

	if (ms->clocks_enabled) {
		clk_disable_unprepare(ms->pclk);
		clk_disable_unprepare(ms->aclk);
	}

	return 0;
}

static int ms_device_runtime_resume(struct device *dev)
{
	struct  ms *ms = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);

	if (ms->clocks_enabled) {
		clk_prepare_enable(ms->pclk);
		clk_prepare_enable(ms->aclk);
	}

	/* enable_irq(ms->irq); */

	return 0;
}

static int ms_suspend(struct device *dev)
{
	return ms_device_runtime_suspend(dev);
}
static int ms_resume(struct device *dev)
{
	return ms_device_runtime_resume(dev);
}

static const struct dev_pm_ops media_shell_pm_ops = {
	.suspend = ms_suspend,
	.resume = ms_resume,
	.runtime_suspend = ms_device_runtime_suspend,
	.runtime_resume = ms_device_runtime_resume,
	.runtime_idle = ms_device_runtime_idle,
};
#endif				/* CONFIG_PM */


MODULE_DEVICE_TABLE(of, media_shell_dt_ids);

static struct platform_driver ms_driver = {
	.driver = {
			.name = MS_PLATFORM_DEVICE_NAME,
			.of_match_table = media_shell_dt_ids,
#ifdef CONFIG_PM
			.pm = &media_shell_pm_ops,
#endif
		   },
	.probe = ms_probe,
	.remove = ms_remove,
};

static int __init ms_init(void)
{
	int err;
	int n;

	ms_class = class_create(THIS_MODULE, "mediashell");

	for (n = 0; n < IP_TYPE_COUNT; n++) {
		err = alloc_chrdev_region(&dev_node[n], 0, MAX_MS_INSTANCES,
							      ip_type_name[n]);
		if (err < 0) {
			pr_err("%s: Failed to get chrdev region\n",
							      ip_type_name[n]);
			return -ENODEV;
		}

		pr_info(
		   "%s: allocated char region starting major %d, minor %d\n",
		      ip_type_name[n], MAJOR(dev_node[n]), MINOR(dev_node[n]));
	}
	err = platform_driver_register(&ms_driver);
	if (err < 0)
		goto bail;
	return err;

bail:
	while (--n >= 0)
		unregister_chrdev_region(dev_node[n], MAX_MS_INSTANCES);
	return err;
}

static void __exit ms_exit(void)
{
	int n;
	for (n = 0; n < IP_TYPE_COUNT; n++)
		unregister_chrdev_region(dev_node[n], MAX_MS_INSTANCES);

	class_destroy(ms_class);
	platform_driver_unregister(&ms_driver);
}

MODULE_AUTHOR("Fujitsu Semiconductor Ltd");
MODULE_DESCRIPTION("Driver for media-shell units");
MODULE_LICENSE("GPL");

module_init(ms_init);
module_exit(ms_exit);
