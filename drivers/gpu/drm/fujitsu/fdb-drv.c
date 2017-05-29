/* tdfx_drv.c -- tdfx driver -*- linux-c -*-
 * Created: Thu Oct  7 10:38:32 1999 by faith@precisioninsight.com
 *
 * Copyright 1999 Precision Insight, Inc., Cedar Park, Texas.
 * Copyright 2000 VA Linux Systems, Inc., Sunnyvale, California.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * PRECISION INSIGHT AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 *    Rickard E. (Rik) Faith <faith@valinux.com>
 *    Daryll Strauss <daryll@valinux.com>
 *    Gareth Hughes <gareth@valinux.com>
 */

#include <linux/module.h>
#include <video/fdb.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include "drm_fb_helper.h"

#include "fdb-drm.h"

#include <uapi/drm/fdb-drm.h>
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
#include <linux/kds.h>
#include <linux/dma-buf.h>
#endif

#define DRIVER_AUTHOR		"Linaro, Ltd"

#define DRIVER_NAME		"fdb-drm"
#define DRIVER_DESC		"Fujitsu Display Bus drm interface"
#define DRIVER_DATE		"20130422"

#define DRIVER_MAJOR		1
#define DRIVER_MINOR		0
#define DRIVER_PATCHLEVEL	0

static void fdb_fb_output_poll_changed(struct drm_device *dev)
{
	struct fdb_drm_private *priv = dev->dev_private;

	if (priv->fbdev)
		drm_fb_helper_hotplug_event(priv->fbdev);
}

static const struct drm_mode_config_funcs fdb_mode_config_funcs = {
	.fb_create = fdb_fb_create,
	.output_poll_changed = fdb_fb_output_poll_changed,
};


int callback_modetset_init(struct f_fdb_child *child, void *arg)
{
	struct drm_device *dev = arg;
	struct fdb_drm_private *priv = dev->dev_private;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	struct f_fdb_child *crtc_child;
	int n;
	int bound = 0;

	dev_info(child->dev, "callback_modetset_init\n");

	if (!child->ops->get_connector_type) {
		dev_info(child->dev, "(not a connector)\n");
		/* not a connector */
		return 0;
	}

	encoder = fdb_encoder_init(dev, child);
	if (!encoder) {
		dev_err(child->dev, "could not create encoder\n");
		return -ENOMEM;
	}

	connector = fdb_connector_init(dev,
			child->ops->get_connector_type(child), child, encoder);
	if (!connector) {
		dev_err(child->dev, "could not create connector\n");
		return -ENOMEM;
	}

	BUG_ON(priv->num_encoders >= ARRAY_SIZE(priv->encoders));
	BUG_ON(priv->num_connectors >= ARRAY_SIZE(priv->connectors));

	priv->encoders[priv->num_encoders++] = encoder;
	priv->connectors[priv->num_connectors++] = connector;

	drm_mode_connector_attach_encoder(connector, encoder);

	/* figure out which crtc's we can connect the encoder to: */
	encoder->possible_crtcs = child->ops->get_source_crtc_bitfield(child);

	dev_info(child->dev, "Connectivity: 0x%x\n", encoder->possible_crtcs);

	for (n = 0; n < priv->num_crtcs; n++)
		if (encoder->possible_crtcs & (1 << n)) {
			crtc_child = bound_child_from_drm_crtc(priv,
					priv->crtcs[n]);
			child->ops->bind_to_source(child, crtc_child);
			fdb_drm_crtc_set_head(priv->crtcs[n], child);
			bound = 1;
			break;
		}

	if (!bound) {
		dev_err(child->dev, "Failed to bind connector to crtc\n");
		return -1;
	}

	return 0;
}

static int fdb_modeset_init(struct drm_device *dev)
{
	struct fdb_drm_private *priv = dev->dev_private;
	unsigned int num_ovls = 1;
	int id;

	dev_info(dev->dev, "fdb_modeset_init\n");

	drm_mode_config_init(dev);

/*	omap_drm_irq_install(dev); */

	/*
	 * Create private planes and CRTCs for the last NUM_CRTCs overlay
	 * plus manager:
	 */
	for (id = 0; id < priv->num_crtcs; id++) {
		struct drm_plane *plane;
		struct drm_crtc *crtc;

		pr_info("***** initializing crtc %d\n", id);

		plane = fdb_plane_init(dev, id, true);
		crtc = fdb_crtc_init(dev, plane, /*pipe2chan(*/id, id);

		BUG_ON(priv->num_crtcs >= ARRAY_SIZE(priv->crtcs));
		priv->crtcs[id] = crtc;

		priv->planes[id] = plane;
		priv->num_planes++;
	}

	/*
	 * Create normal planes for the remaining overlays:
	 */
	for (; id < num_ovls; id++) {
		struct drm_plane *plane = fdb_plane_init(dev, id, false);

		BUG_ON(priv->num_planes >= ARRAY_SIZE(priv->planes));
		priv->planes[priv->num_planes++] = plane;
	}

	fdb_callback_each(dev->dev, callback_modetset_init, dev);

	dev->mode_config.min_width = 32;
	dev->mode_config.min_height = 32;

	/* note: eventually will need some cpu_is_omapXYZ() type stuff here
	 * to fill in these limits properly on different OMAP generations..
	 */
	dev->mode_config.max_width = 8192;
	dev->mode_config.max_height = 8192;

	dev->mode_config.funcs = &fdb_mode_config_funcs;

	return 0;
}

static void fdb_modeset_free(struct drm_device *dev)
{
	drm_mode_config_cleanup(dev);
}

static const struct file_operations fdb_drm_driver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.mmap = fdb_gem_mmap,
	.poll = drm_poll,
#ifdef CONFIG_COMPAT
	.compat_ioctl = drm_compat_ioctl,
#endif
	.llseek = noop_llseek,
	.read = drm_read,
	.llseek = noop_llseek,
};


static int fdb_drm_enable(struct f_fdb_child *fdb_child)
{
	return 0;
}
static void fdb_drm_disable(struct f_fdb_child *fdb_child)
{
}

int fdb_drm_sync(struct f_fdb_child *fdb_child, int crtc_id, u32 count)
{
	struct fdb_drm_private *priv =
			container_of(fdb_child, struct fdb_drm_private, child);

	dev_dbg(fdb_child->dev, "vsync crtc %d count %d\n", crtc_id, count);
	drm_handle_vblank(priv->dev, crtc_id);
	return 0;
}

struct f_fdb_ops fdb_drm_ops = {
	.enable = fdb_drm_enable,
	.disable = fdb_drm_disable,
	.sync = fdb_drm_sync,
};


int callback_drm_init(struct f_fdb_child *child, void *arg)
{
	struct drm_device *dev = arg;

	if (child->ops->initialized_by_drm)
		child->ops->initialized_by_drm(child, dev);

	return 0;
}

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
static void fdb_kds_sync_shared_buf_cb(void *cb1, void *cb2)
{
	wait_queue_head_t *wait = (wait_queue_head_t *) cb1;
	bool *cb_has_called = (bool *) cb2;

	*cb_has_called = true;
	wake_up(wait);
}
#endif

static int fdb_drm_load(struct drm_device *dev, unsigned long flags)
{
	struct fdb_drm_private *priv;
	int ret, n;
	struct device_node *node;
	struct f_fdb_child *child;

	pr_info("load: dev=%p", dev);

	/* did we get enough registrations? */
	if (!of_property_read_u32(dev->dev->of_node, "registrations", &n)) {
		dev_info(dev->dev, "of regs %d, actual %d\n", n,
						fdb_get_peer_count(dev->dev));
		if (n != fdb_get_peer_count(dev->dev))
			return -EPROBE_DEFER;
	}

	/* don't start the real probe and binding unless it's all there */

	do {
		node = of_parse_phandle(dev->dev->of_node, "bind", n);
		if (!node)
			continue;

		fdb_name_to_child(dev->dev, of_node_full_name(node), &child);
		if (!child) {
			dev_err(dev->dev, "Unable to find bind peer\n");
			return -EPROBE_DEFER;
		}
		n++;
	} while (node);



	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	atomic_set(&priv->nr_flips_in_flight, 0);
	init_waitqueue_head(&priv->wait_for_flips);

	ret = kds_callback_init(&priv->kds_cb, 1, fdb_crtc_page_flip_cb);
	if (ret) {
		dev_err(dev->dev, "kds_callback_init failed(%d)\n", ret);
		goto bail;
	}

	ret = kds_callback_init(&priv->kds_sync_shared_buf_cb, 1, fdb_kds_sync_shared_buf_cb);
	if (ret) {
		dev_err(dev->dev, "kds_callback_init failed(%d)\n", ret);
		kds_callback_term(&priv->kds_cb);
		goto bail;
	}

	priv->page_flip_slab =
		kmem_cache_create("page flip slab", sizeof(struct fdb_drm_flip_resource), 0, 0, NULL);
	if (!priv->page_flip_slab) {
		dev_err(dev->dev, "kmem_cache_create failed\n");
		ret = -ENOMEM;
		kds_callback_term(&priv->kds_cb);
		kds_callback_term(&priv->kds_sync_shared_buf_cb);
		goto bail;
	}
#endif

	dev->dev_private = priv;
	priv->dev = dev;

	priv->wq = alloc_ordered_workqueue("fdbdrm", 0);

	INIT_LIST_HEAD(&priv->obj_list);

	fdb_gem_init(dev);

	/* register our fdb_child with f_fdb bus */
	ret = fdb_register(dev->dev, &priv->child, &fdb_drm_ops);
	if (ret < 0) {
		dev_err(dev->dev, "framebuffer registration failed\n");
		goto bail;
	}

	do {
		node = of_parse_phandle(dev->dev->of_node, "bind",
							priv->num_crtcs);
		if (!node)
			continue;

		dev_info(dev->dev, "binding to %s\n", of_node_full_name(node));

		fdb_name_to_child(dev->dev, of_node_full_name(node),
				&priv->bound[priv->num_crtcs]);
		if (!priv->bound[priv->num_crtcs]) {
			dev_err(dev->dev, "Unable to find bind peer\n");
			return -EPROBE_DEFER;
		}

		/* tag the crtc as reporting to us */
		priv->bound[priv->num_crtcs]->ops->bind_to_source(
				priv->bound[priv->num_crtcs], &priv->child);

		priv->num_crtcs++;

	} while (node);

	dev_info(dev->dev, "CRTCs: %d\n", priv->num_crtcs);

	ret = fdb_modeset_init(dev);
	if (ret) {
		dev_err(dev->dev, "fdb_modeset_init failed: ret=%d\n", ret);
		goto bail1;
	}

	/*
	 * We don't use the drm_irq_install() helpers provided by the DRM
	 * core, so we need to set this manually in order to allow the
	 * DRM_IOCTL_WAIT_VBLANK to operate correctly.
	 */
	dev->irq_enabled = true;

	ret = drm_vblank_init(dev, priv->num_crtcs);
	if (ret)
		dev_warn(dev->dev, "could not init vblank\n");

	priv->fbdev = fdb_fbdev_init(dev);
	if (!priv->fbdev)
		dev_warn(dev->dev, "fdb_fbdev_init failed\n");

	dev_set_drvdata(dev->dev, dev);

	drm_kms_helper_poll_init(dev);

	/* let anyone registered who is interested know they're initialized */
	fdb_callback_each(dev->dev, callback_drm_init, dev);

	return 0;

bail1:
	fdb_unregister(dev->dev, &priv->child);
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	kmem_cache_destroy(priv->page_flip_slab);
	kds_callback_term(&priv->kds_cb);
	kds_callback_term(&priv->kds_sync_shared_buf_cb);
#endif
bail:
	dev->dev_private = NULL;
	kfree(priv);

return ret;
}

static int fdb_drm_unload(struct drm_device *dev)
{
	struct fdb_drm_private *priv = dev->dev_private;

	drm_kms_helper_poll_fini(dev);
	drm_vblank_cleanup(dev);
/*	fdb_drm_irq_uninstall(dev); */

	fdb_fbdev_free(dev);
	fdb_modeset_free(dev);
	fdb_gem_deinit(dev);

	flush_workqueue(priv->wq);
	destroy_workqueue(priv->wq);

	fdb_unregister(dev->dev, &priv->child);

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	kmem_cache_destroy(priv->page_flip_slab);
	kds_callback_term(&priv->kds_cb);
	kds_callback_term(&priv->kds_sync_shared_buf_cb);
#endif

	kfree(dev->dev_private);
	dev->dev_private = NULL;

	dev_set_drvdata(dev->dev, NULL);

	return 0;
}


/*
 * drm ioctl funcs
 */


static int ioctl_gem_cpu_prep(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct drm_fdb_gem_cpu_prep *args = data;
	struct drm_gem_object *obj;
	int ret;

	pr_err("%p:%p: handle=%d, op=%x", dev, file_priv,
						args->handle, args->op);

	obj = drm_gem_object_lookup(dev, file_priv, args->handle);
	if (!obj)
		return -ENOENT;

	ret = fdb_gem_op_sync(obj, args->op);

	if (!ret)
		ret = fdb_gem_op_start(obj, args->op);

	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

static int ioctl_gem_cpu_fini(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct drm_fdb_gem_cpu_fini *args = data;
	struct drm_gem_object *obj;
	int ret;

/*	pr_err("%p:%p: handle=%d", dev, file_priv, args->handle); */

	obj = drm_gem_object_lookup(dev, file_priv, args->handle);
	if (!obj)
		return -ENOENT;

	ret = 0;

	if (!ret)
		ret = fdb_gem_op_finish(obj, args->op);

	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

int ioctl_gem_alloc(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct drm_fdb_gem_bo *args = data;
	struct drm_gem_object *obj;
	struct drm_mode_create_dumb create_dumb;
	int ret = 0;

	dev_dbg(dev->dev, "%s: %p: wxh=%dx%d, bpp=%d", __func__,
			file_priv, args->width, args->height, args->bpp);

	if (!args->width || !args->height) {
		dev_err(dev->dev, "%s: zero size allocation\n", __func__);
		return -EINVAL;
	}

	create_dumb.height = args->height;
	create_dumb.width = args->width;
	create_dumb.bpp = args->bpp;
	create_dumb.flags = 0;

	ret = fdb_gem_dumb_create(file_priv, dev, &create_dumb);
	if (ret) {
		dev_err(dev->dev, "%s: Failed dumb_create: %d\n",
				__func__, ret);
		goto bail;
	}

	args->handle = create_dumb.handle;
	args->size = create_dumb.size;
	args->pitch = create_dumb.pitch;

	ret = fdb_gem_dumb_map_offset(file_priv, dev,
					args->handle, &args->offset);
	if (ret) {
		dev_err(dev->dev, "%s: Failed dumb_map_offset: %d\n",
				__func__, ret);
		goto bail;
	}

	obj = drm_gem_object_lookup(dev, file_priv, args->handle);
	if (!obj) {
		dev_err(dev->dev, "%s: Failed object_lookup\n", __func__);
		ret = -ENOENT;
		goto bail;
	}

	fdb_gem_put_width_height(obj, args->width, args->height);

	/* preset and cast because UAPI is u64, dma_addr_t is 64/32 vs LPAE */
	args->paddr = 0;
	ret = fdb_gem_get_paddr(obj, (phys_addr_t *)&args->paddr, true);
	if (ret) {
		dev_err(dev->dev, "%s: Failed get_paddr: %d\n", __func__, ret);
		goto bail1;
	}

	mutex_lock(&obj->dev->struct_mutex);
	args->base = fdb_gem_vaddr(obj);
	mutex_unlock(&obj->dev->struct_mutex);

bail1:
	drm_gem_object_unreference_unlocked(obj);
bail:
	return ret;
}

static int ioctl_iris_blit_blocking(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct blit_block *b = data;
	struct f_fdb *bus = f_fdb_from_dev(dev->dev);
	struct f_fdb_child *iris_fdb_child = NULL;
	struct drm_gem_object *obj_in = NULL, *obj_out = NULL;
	int n;
	struct fdb_blit b1;
	struct completion *comp = NULL;

	if (b->length != sizeof(struct blit_block)) {
		dev_err(dev->dev, "%s: blit struct size mismatch k:%d u:%d\n",
			__func__, sizeof(struct blit_block), b->length);
		return -EINVAL;
	}

	b1 = b->blit;

/*	dev_info(dev->dev, "%s: entry\n", __func__);*/

	for (n = 0; n < bus->count_fdb_children; n++)
		if (!strcmp(bus->child[n]->dev->driver->name, "f_iris_fb")) {
			iris_fdb_child = bus->child[n];
			break;
		}
	if (!iris_fdb_child) {
		dev_err(dev->dev, "%s: unable to find iris fb\n", __func__);
		return -1;
	}

	if (b->gem_handle_mask) {
		obj_in = drm_gem_object_lookup(dev, file_priv,
				b->gem_handle_mask);
		if (!obj_in) {
			dev_err(dev->dev,
					"Unable to get gem from mask handle\n");
			return -ENOENT;
		}
		/* cast because it's held in a u64 to be same LPAE or not */
		n = fdb_gem_get_paddr(obj_in, (phys_addr_t *)&b1.mask_addr,
				true);
		if (n < 0) {
			dev_err(dev->dev, "Unable to get mask paddr: %d\n", n);
			drm_gem_object_unreference_unlocked(obj_in);
			return n;
		}
		if (b1.stride_mask)
			b1.mask_addr += b->offset_mask;
		else {
			u32 __iomem *mask_va;

			mutex_lock(&obj_in->dev->struct_mutex);
			mask_va = fdb_gem_vaddr(obj_in);
			b1.mask_addr = 0;
			b1.fill_pixel = __raw_readl(mask_va);
			mutex_unlock(&obj_in->dev->struct_mutex);
		}
	}

	if (b->paddr_in) {
		b1.src_addr = b->paddr_in + b->offset_in;
		goto do_dest;
	}
	/*
	 * convert the gem handles to paddr, including the offsets now
	 */
	if (!b->paddr_in && b->gem_handle_in) {
		obj_in = drm_gem_object_lookup(dev, file_priv,
				b->gem_handle_in);
		if (!obj_in) {
			dev_err(dev->dev,
					"Unable to get gem from src handle\n");
			return -ENOENT;
		}

		fdb_gem_get_width_height(obj_in, &b1.src_size_w, &b1.src_size_h);

		/* cast because it's held in a u64 to be same LPAE or not */
		n = fdb_gem_get_paddr(obj_in, (phys_addr_t *)&b1.src_addr, true);
		if (n < 0) {
			dev_err(dev->dev, "Unable to get src paddr: %d\n", n);
			drm_gem_object_unreference_unlocked(obj_in);
			return n;
		}
		b1.src_addr += b->offset_in;
	} else
		if (b1.width_in || b1.height_in) {
			dev_err(dev->dev, "NULL handle_in\n");
			return -EINVAL;
		} else
			b1.src_addr = 0;
do_dest:
	if (b->paddr_out) {
		b1.dest_addr = b->paddr_out + b->offset_out;
		WARN_ONCE((b->blit.async == BLIT_ASYNC_NEED_COMP), "gem_handle_out is needed.\n");
		goto do_blit;
	}

	obj_out = drm_gem_object_lookup(dev, file_priv,	b->gem_handle_out);
	if (!obj_out) {
		dev_err(dev->dev, "Unable to get gem from dst handle\n");
		return -ENOENT;
	}
	fdb_gem_get_width_height(obj_out, &b1.dest_size_w, &b1.dest_size_h);
	/* cast because it's held in a u64 to be same LPAE or not */
	n = fdb_gem_get_paddr(obj_out, (phys_addr_t *)&b1.dest_addr, true);
	if (n < 0) {
		dev_err(dev->dev, "Unable to get destination paddr: %d\n", n);
		goto bail;
	}
	b1.dest_addr += b->offset_out;

	if (b->blit.async == BLIT_ASYNC_NEED_COMP)
		comp = fdb_gem_get_comp(obj_out);

	if (b1.dest_x_offset || b1.dest_y_offset) {
		unsigned long size, size_out;
		size_out = fdb_gem_mmap_size(obj_out);
		size = (b1.width_out + b1.dest_x_offset) *
		       (b1.height_out + b1.dest_y_offset) *
		       (b1.bits_per_pixel_out / 8);
		if (size > size_out) {
			dev_err(dev->dev, "Insufficient dest gem_obj size:"
				"0x%lx (0x%lx)\n", size_out, size);
			n = -EINVAL;
			goto bail;
		}
	}

do_blit:
	if (b->blit.async) {
		n = iris_fdb_child->ops->blit_blocking2(iris_fdb_child, &b1, comp, obj_in, obj_out);
	}
	else {
		n = iris_fdb_child->ops->blit_blocking(iris_fdb_child, &b1);
	}
	memcpy(b->blit.b_c_id, b1.b_c_id,
			sizeof(struct blit_completion_id) * BLIT_COMP_RING_LEN);
	b->blit.id = b1.id;
bail:
	if (obj_in)
		drm_gem_object_unreference_unlocked(obj_in);
	if (obj_out)
		drm_gem_object_unreference_unlocked(obj_out);

	return n;
}

static int ioctl_get_fb_paddr(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct drm_fdb_gem_bo *p = data;
	struct fdb_drm_private *priv = dev->dev_private;
	struct drm_framebuffer *fb;
	struct drm_gem_object *dgo;

	p->paddr = 0;
	fb = priv->fbdev->fb;

	dgo = fdb_fb_bo(fb, 0);
	/* dev_info(dev->dev, "dgo %p\n", dgo); */

	if (dgo)
		p->paddr = fdb_gem_paddr(dgo);

	return 0;
}

static int ioctl_get_blit_comp(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct f_fdb *bus = f_fdb_from_dev(dev->dev);
	struct f_fdb_child *iris_fdb_child = NULL;
	struct b_c_id_ring *b_c_id_ring = data;
	int n;

	for (n = 0; n < bus->count_fdb_children; n++)
		if (!strcmp(bus->child[n]->dev->driver->name, "f_iris_fb")) {
			iris_fdb_child = bus->child[n];
			break;
		}
	if (!iris_fdb_child) {
		dev_err(dev->dev, "%s: unable to find iris fb\n", __func__);
		return -1;
	}

	return iris_fdb_child->ops->get_blit_comp(iris_fdb_child, b_c_id_ring);
}

static int ioctl_get_blit_comp_one(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct drm_fdb_comp *fcomp = data;
	struct drm_gem_object *obj_out;
	struct completion *comp;
	long ret;

	if (!fcomp->gem_handle)
		return -EINVAL;

	obj_out = drm_gem_object_lookup(dev, file_priv,	fcomp->gem_handle);
	if (!obj_out)
		return -ENOENT;

	comp = fdb_gem_get_comp(obj_out);

	ret = wait_for_completion_interruptible_timeout(
				comp, msecs_to_jiffies(fcomp->wait_msec));
	if (!ret)
		ret = -ETIMEDOUT;
	else if (ret > 0)
		ret = 0;

	drm_gem_object_unreference_unlocked(obj_out);

	return (int)ret;
}

static int ioctl_sync_dma_shared_buf(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	int ret = 0;

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	struct drm_fdb_comp *fcomp = data;
	struct drm_gem_object *obj;

	if (!fcomp->gem_handle)
		return -EINVAL;

	obj = drm_gem_object_lookup(dev, file_priv,	fcomp->gem_handle);
	if (!obj)
		return -ENOENT;

	/*
	 * This function is called from ioctl(), so export_dma_buf would be referenced
	 * by callee process(export_dma_buf's refcount is > 0).
	 * It is not necessary to hold object_name_lock.
	 */
	if (obj->export_dma_buf) {
		struct dma_buf *buf = obj->export_dma_buf;
		struct fdb_drm_private *priv = dev->dev_private;
		unsigned long shared[1] = { 0 };
		struct kds_resource *resource_list[1] = {
				get_dma_buf_kds_resource(buf) };
		struct kds_resource_set *kds_res_set;
		bool cb_has_called = false;

		DECLARE_WAIT_QUEUE_HEAD_ONSTACK(wake);

		ret = kds_async_waitall(&kds_res_set,
					&priv->kds_sync_shared_buf_cb, &wake,
					&cb_has_called, 1, shared, resource_list);

		if (ret) {
			dev_err(dev->dev, "%s: unable to find iris fb\n", __func__);
		}
		else {
			ret = wait_event_interruptible_timeout(
						wake,
						(cb_has_called == true),
						msecs_to_jiffies(fcomp->wait_msec));
			/* Release the kds resource or cancel if error */
			kds_resource_set_release_sync(&kds_res_set);

			if (!ret)
				ret = -ETIMEDOUT;
			else if (ret > 0)
				ret = 0;
		}
	}

	drm_gem_object_unreference_unlocked(obj);
#endif

	return ret;
}

struct drm_ioctl_desc ioctls[DRM_COMMAND_END - DRM_COMMAND_BASE] = {
	DRM_IOCTL_DEF_DRV(FDB_GEM_CPU_PREP, ioctl_gem_cpu_prep,
							DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(FDB_GEM_CPU_FINI, ioctl_gem_cpu_fini,
							DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(FDB_GEM_ALLOC, ioctl_gem_alloc,
							DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(FDB_IRIS_BLIT_BLOCKING, ioctl_iris_blit_blocking,
							DRM_AUTH),
	DRM_IOCTL_DEF_DRV(FDB_GET_FB_PADDR, ioctl_get_fb_paddr, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(FDB_GET_BLIT_COMP, ioctl_get_blit_comp, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(FDB_GET_BLIT_COMP_ONE, ioctl_get_blit_comp_one, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(FDB_SYNC_SHARED_BUF, ioctl_sync_dma_shared_buf, DRM_AUTH)
};

static const struct vm_operations_struct fdb_gem_vm_ops = {
	.fault = fdb_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

#if defined(CONFIG_DEBUG_FS)
static int
fdb_debugfs_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct f_fdb *bus = f_fdb_from_dev(dev->dev);
	int n;

	seq_printf(m, "fdb-drm device: %s\n", dev_name(dev->dev));

	for (n = 0; n < bus->count_fdb_children; n++)
		if (bus->child[n]->ops->fdb_debugfs)
			bus->child[n]->ops->fdb_debugfs(bus->child[n], m,
					FDB_DEBUGFS_INFO);

	return 0;
}

static int
fdb_debugfs_dump(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct f_fdb *bus = f_fdb_from_dev(dev->dev);
	int n;

	seq_printf(m, "fdb-drm device: %s\n", dev_name(dev->dev));

	for (n = 0; n < bus->count_fdb_children; n++)
		if (bus->child[n]->ops->fdb_debugfs)
			bus->child[n]->ops->fdb_debugfs(bus->child[n], m,
					FDB_DEBUGFS_DUMP);

	return 0;
}

static struct drm_info_list fdb_debugfs_list[] = {
	{ "fdb-info", fdb_debugfs_info, 0, NULL },
	{ "fdb-dump", fdb_debugfs_dump, 0, NULL },
};

int
fdb_debugfs_init(struct drm_minor *minor)
{
	drm_debugfs_create_files(
		fdb_debugfs_list, ARRAY_SIZE(fdb_debugfs_list),
						minor->debugfs_root, minor);
	return 0;
}

void
fdb_debugfs_cleanup(struct drm_minor *minor)
{
	drm_debugfs_remove_files(fdb_debugfs_list, ARRAY_SIZE(fdb_debugfs_list),
				 minor);
}
#endif

static int fdb_irq_enable_vblank(struct drm_device *dev, int crtc)
{
	return 0;
}

static void fdb_irq_disable_vblank(struct drm_device *dev, int crtc)
{
}

static struct drm_driver driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_PRIME,
	.load = fdb_drm_load,
	.unload = fdb_drm_unload,

	.get_vblank_counter = drm_vblank_count,

	.enable_vblank = fdb_irq_enable_vblank,
	.disable_vblank = fdb_irq_disable_vblank,

	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_export = fdb_gem_prime_export,
	.gem_prime_import = fdb_gem_prime_import,

	.gem_free_object = fdb_gem_free_object,
	.gem_vm_ops = &fdb_gem_vm_ops,
	.dumb_create = fdb_gem_dumb_create,
	.dumb_map_offset = fdb_gem_dumb_map_offset,
	.dumb_destroy = fdb_gem_dumb_destroy,
	.ioctls = ioctls,
	.num_ioctls = DRM_FDB_NUM_IOCTLS,

#if defined(CONFIG_DEBUG_FS)
	.debugfs_init = fdb_debugfs_init,
	.debugfs_cleanup = fdb_debugfs_cleanup,
#endif

	.fops = &fdb_drm_driver_fops,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

static int fdb_drm_probe(struct platform_device *pdev)
{
	return drm_platform_init(&driver, pdev);
}

static int fdb_drm_remove(struct platform_device *pdev)
{
	drm_platform_exit(&driver, pdev);
	return 0;
}

static const struct of_device_id fdb_fb_drm_dt_ids[] = {
	{ .compatible = "fujitsu,fdb-drm" },
	{ /* sentinel */ }
};

struct platform_driver fdb_drm_pdev = {
		.driver = {
			.name = DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table = fdb_fb_drm_dt_ids,
/* #ifdef CONFIG_PM
			.pm = &omapdrm_pm_ops,
#endif */
		},
		.probe = fdb_drm_probe,
		.remove = fdb_drm_remove,
/*
		.suspend = pdev_suspend,
		.resume = pdev_resume,
		.shutdown = pdev_shutdown, */
};

MODULE_DEVICE_TABLE(of, fdb_fb_drm_dt_ids);

static int __init fdb_drm_init(void)
{
	return platform_driver_register(&fdb_drm_pdev);
}

static void __exit fdb_drm_exit(void)
{
	platform_driver_unregister(&fdb_drm_pdev);
}


module_init(fdb_drm_init);
module_exit(fdb_drm_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL and additional rights");
