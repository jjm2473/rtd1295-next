/*
 * drivers/gpu/drm/fujitsu/fdb_fbdev.c
 *
 * Copyright (C) 2013 Linaro, Ltd
 * Author: Andy Green <andy.green@linaro.org>
 *
 * based on omapdrm implementation from --->
 * Copyright (C) 2011 Texas Instruments
 * Author: Rob Clark <rob@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/dma-buf.h>
#include <linux/iommu.h>

#include <video/fdb.h>

#include <uapi/linux/fdb.h>
#include "fdb-drm.h"
#include "drm_crtc.h"
#include "drm_fb_helper.h"

#include <asm/dma-iommu.h>
#define MODULE_NAME "fdbdrm-fbdev"

#define NUM_BUFFERS 3

int reject_32bpp = 0;

static int __init param_reject_32bpp(char *str)
{
	reject_32bpp = 1;
	return 0;
}

early_param("reject_32bpp", param_reject_32bpp);


#define to_fdb_fbdev(x) container_of(x, struct fdb_fbdev, base)

struct fdb_fbdev {
	struct drm_fb_helper base;
	struct drm_framebuffer *fb;
	struct drm_gem_object *bo;
};

static int fdb_fb_ioctl(struct fb_info *info, unsigned int cmd,
			unsigned long arg)
{
	int ret = 0;
	struct drm_fb_helper *helper = info->par;
	struct drm_device *dev = helper->dev;
	struct fdb_drm_private *priv = dev->dev_private;
	struct fdb_fbdev *fbdev = to_fdb_fbdev(helper);
	u32 buf_fd;
	u32 __user *out_ptr = (u32 __user *)arg;
	struct f_fdb_child *fdb_child_fb;
	struct drm_connector *conn;
	struct drm_crtc *crtc;

	switch (cmd) {
	case IOCTL_GET_FB_DMA_BUF:
		buf_fd = fdb_fb_get_dma_buf(dev, fbdev->bo);

		if (buf_fd == -1) {
			ret = -ENOMEM;
			break;
		}
		ret = put_user(buf_fd, out_ptr);
		break;

	case FBIO_WAITFORVSYNC:
		if (!helper->connector_count)
			return -EINVAL;

		conn = helper->connector_info[0]->connector;

		if (!conn || !conn->encoder)
			return -EINVAL;

		crtc = conn->encoder->crtc;
		fdb_child_fb = bound_child_from_drm_crtc(priv, crtc);
		if (!fdb_child_fb)
			return -EINVAL;

		if (!fdb_child_fb->ops->fdb_wait_for_vsync)
			return -EINVAL;

		ret = fdb_child_fb->ops->fdb_wait_for_vsync(fdb_child_fb, 1);
		break;
	default:
		ret = -ENOTTY;
	}

	return ret;
}


static struct fb_ops fdb_fb_ops = {
	.owner = THIS_MODULE,

	.fb_fillrect = sys_fillrect,
	.fb_copyarea = sys_copyarea,
	.fb_imageblit = sys_imageblit,

	.fb_check_var = drm_fb_helper_check_var,
	.fb_set_par = drm_fb_helper_set_par,
	.fb_blank = drm_fb_helper_blank,
	.fb_setcmap = drm_fb_helper_setcmap,

	.fb_pan_display = drm_fb_helper_pan_display,
	.fb_ioctl	= fdb_fb_ioctl,
};

static int fdb_fbdev_create(struct drm_fb_helper *helper,
		struct drm_fb_helper_surface_size *sizes)
{
	struct fdb_fbdev *fbdev = to_fdb_fbdev(helper);
	struct drm_device *dev = helper->dev;
	struct drm_framebuffer *fb = NULL;
	u32 gsize;
	struct fb_info *fbi = NULL;
	struct drm_mode_fb_cmd2 mode_cmd = { 0 };
	phys_addr_t paddr;
	int ret;
	struct dma_iommu_mapping *mapping;

	mapping = to_dma_iommu_mapping(dev->dev);
	mode_cmd.pixel_format = drm_mode_legacy_fb_format(sizes->surface_bpp,
			sizes->surface_depth);

	dev_info(dev->dev, "create fbdev: %dx%d@%d (%dx%d) %c%c%c%c\n",
			sizes->surface_width,
			sizes->surface_height, sizes->surface_bpp,
			sizes->fb_width, sizes->fb_height,
			mode_cmd.pixel_format & 0xff,
			(mode_cmd.pixel_format >> 8) & 0xff,
			(mode_cmd.pixel_format >> 16) & 0xff,
			mode_cmd.pixel_format >> 24);

	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height * NUM_BUFFERS;

	mode_cmd.pitches[0] = align_pitch(
			mode_cmd.width * ((sizes->surface_bpp + 7) / 8),
			mode_cmd.width, sizes->surface_bpp);

	/* allocate backing bo */
	gsize = PAGE_ALIGN(mode_cmd.pitches[0] * mode_cmd.height);
	pr_info("allocating %d bytes for fb %d\n", gsize, dev->primary->index);
	fbdev->bo = fdb_gem_new(dev, gsize, FDB_BO_SCANOUT | FDB_BO_WC | FDB_BO_PREALLOC);
	if (!fbdev->bo) {
		dev_info(dev->dev, "failed to allocate buffer object\n");
		ret = -ENOMEM;
		goto fail;
	}

	fb = fdb_fb_init(dev, &mode_cmd, &fbdev->bo);
	if (IS_ERR(fb)) {
		dev_info(dev->dev, "failed to allocate fb\n");
		ret = PTR_ERR(fb);
		fb = NULL;
		goto fail;
	}

	/* note: this keeps the bo pinned.. which is perhaps not ideal,
	 * but is needed as long as we use fb_mmap() to mmap to userspace
	 * (since this happens using fix.smem_start).  Possibly we could
	 * implement our own mmap using GEM mmap support to avoid this
	 * (non-tiled buffer doesn't need to be pinned for fbcon to write
	 * to it).  Then we just need to be sure that we are able to re-
	 * pin it in case of an opps.
	 */
	ret = fdb_gem_get_paddr(fbdev->bo, &paddr, true);
	if (ret) {
		dev_info(dev->dev,
			"could not map (paddr)!  Skipping framebuffer alloc\n");
		ret = -ENOMEM;
		goto fail;
	}

	mutex_lock(&dev->struct_mutex);

	fbi = framebuffer_alloc(0, dev->dev);
	if (!fbi) {
		dev_info(dev->dev, "failed to allocate fb info\n");
		ret = -ENOMEM;
		goto fail_unlock;
	}

	fbdev->fb = fb;
	helper->fb = fb;
	helper->fbdev = fbi;

	fbi->par = helper;
	fbi->flags = FBINFO_DEFAULT;
	fbi->fbops = &fdb_fb_ops;

	strcpy(fbi->fix.id, "fdb-fbdev");

	ret = fb_alloc_cmap(&fbi->cmap, 256, 0);
	if (ret) {
		ret = -ENOMEM;
		goto fail_unlock;
	}
	fb_set_cmap(&fbi->cmap, fbi);

	drm_fb_helper_fill_fix(fbi, fb->pitches[0], fb->depth);
	drm_fb_helper_fill_var(fbi, helper, sizes->fb_width, sizes->fb_height);

	dev->mode_config.fb_base = paddr;

	fbi->screen_base = fdb_gem_vaddr(fbdev->bo);
	fbi->screen_size = fbdev->bo->size;
	fbi->fix.smem_start = paddr;
	if (mapping) {
		paddr = iommu_iova_to_phys(mapping->domain, paddr);
		fbi->fix.smem_start = paddr;
	}

	fbi->fix.smem_len = fbdev->bo->size;

	pr_info("par=%p, %dx%d\n", fbi->par, fbi->var.xres, fbi->var.yres);
	pr_info("allocated %dx%d fb at paddr 0x%llx\n",
				fbdev->fb->width, fbdev->fb->height, (u64)paddr);

	mutex_unlock(&dev->struct_mutex);

	return 0;

fail_unlock:
	mutex_unlock(&dev->struct_mutex);
fail:

	if (!ret)
		return 0;

	if (fbi)
		framebuffer_release(fbi);
	if (fb) {
		drm_framebuffer_unregister_private(fb);
		drm_framebuffer_remove(fb);
	}

	return ret;
}

static void fdb_crtc_fb_gamma_set(struct drm_crtc *crtc,
					u16 red, u16 green, u16 blue, int regno)
{
	struct fdb_drm_private *priv = crtc->dev->dev_private;
	struct f_fdb_child *fdb_child_fb;

	fdb_child_fb = bound_child_from_drm_crtc(priv, crtc);

	if (!fdb_child_fb)
		return;

	if (!fdb_child_fb->ops->set_gamma)
		return;

	fdb_child_fb->ops->set_gamma(fdb_child_fb, red, green, blue, regno);
}

static void fdb_crtc_fb_gamma_get(struct drm_crtc *crtc,
		u16 *red, u16 *green, u16 *blue, int regno)
{
	struct fdb_drm_private *priv = crtc->dev->dev_private;
	struct f_fdb_child *fdb_child_fb;

	fdb_child_fb = bound_child_from_drm_crtc(priv, crtc);

	if (!fdb_child_fb)
		return;
	if (!fdb_child_fb->ops->get_gamma)
		return;

	fdb_child_fb->ops->get_gamma(fdb_child_fb, red, green, blue, regno);

}

static struct drm_fb_helper_funcs fdb_fb_helper_funcs = {
	.gamma_set = fdb_crtc_fb_gamma_set,
	.gamma_get = fdb_crtc_fb_gamma_get,
	.fb_probe = fdb_fbdev_create,
};

struct f_fdb_child *bound_child_from_drm_crtc(
			struct fdb_drm_private *priv, struct drm_crtc *crtc)
{
	int n;

	for (n = 0; n < priv->num_crtcs; n++)
		if (priv->crtcs[n] == crtc)
			return priv->bound[n];

	pr_err("^^^^^ No bound child found for crtc\n");
	return NULL;
}

#if 0
static struct drm_fb_helper *get_fb(struct fb_info *fbi)
{
	if (!fbi || strcmp(fbi->fix.id, MODULE_NAME)) {
		/* these are not the fb's you're looking for */
		return NULL;
	}
	return fbi->par;
}
#endif

/* initialize fbdev helper */
struct drm_fb_helper *fdb_fbdev_init(struct drm_device *dev)
{
	struct fdb_drm_private *priv = dev->dev_private;
	struct fdb_fbdev *fbdev = NULL;
	struct drm_fb_helper *helper;
	int ret = 0;
	u32 bpp = 32; /* if DT doesn't override, prefer 32-bpp initial modes */

	if (reject_32bpp)
		bpp = 16;

	dev_info(dev->dev, "fdb_fbdev_init num_crtcs=%d num_connectors=%d\n",
					priv->num_crtcs, priv->num_connectors);

	fbdev = kzalloc(sizeof(*fbdev), GFP_KERNEL);
	if (!fbdev)
		goto fail;

	helper = &fbdev->base;
	helper->funcs = &fdb_fb_helper_funcs;

	ret = drm_fb_helper_init(dev, helper,
			priv->num_crtcs, priv->num_connectors);
	if (ret) {
		dev_err(dev->dev, "could not init fbdev: ret=%d\n", ret);
		goto fail;
	}

	if (!reject_32bpp)
		/* if given, prefer the default bpp from Device Tree */
		of_property_read_u32(dev->dev->of_node, "default-bpp", &bpp);


	drm_fb_helper_single_add_all_connectors(helper);
	/* disable all the possible outputs/crtcs before entering KMS mode */
	drm_helper_disable_unused_functions(dev);	
	drm_fb_helper_initial_config(helper, bpp);
	priv->fbdev = helper;

	return helper;

fail:
	kfree(fbdev);

	return NULL;
}

void fdb_fbdev_free(struct drm_device *dev)
{
	struct fdb_drm_private *priv = dev->dev_private;
	struct drm_fb_helper *helper = priv->fbdev;
	struct fdb_fbdev *fbdev;
	struct fb_info *fbi;

	pr_info("fdb_fbdev_free\n");

	fbi = helper->fbdev;

	/* only cleanup framebuffer if it is present */
	if (fbi) {
		unregister_framebuffer(fbi);
		framebuffer_release(fbi);
	}

	drm_fb_helper_fini(helper);

	fbdev = to_fdb_fbdev(priv->fbdev);

	/* this will free the backing object */
	if (fbdev->fb) {
		drm_framebuffer_unregister_private(fbdev->fb);
		drm_framebuffer_remove(fbdev->fb);
	}

	kfree(fbdev);

	priv->fbdev = NULL;
}
