/*
 * drivers/gpu/drm/omapdrm/fdb_crtc.c
 *
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

#include <video/fdb.h>
#include "fdb-drm.h"

#include <drm/drm_mode.h>
#include "drm_crtc.h"
#include "drm_crtc_helper.h"
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
#include <linux/kds.h>
#include <linux/dma-buf.h>
#endif

static void fdb_crtc_destroy(struct drm_crtc *crtc);

static void fdb_crtc_ref_fb_cb(void *arg)
{
	struct drm_framebuffer *fb = arg;

	drm_framebuffer_unreference(fb);
}

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
static void fdb_crtc_ref_fb_work(struct work_struct *work)
{
	struct fdb_drm_flip_resource *flip_res =
			container_of(work, struct fdb_drm_flip_resource, unref_work);
	struct fdb_drm_private *priv = flip_res->crtc->dev->dev_private;

	drm_framebuffer_unreference(flip_res->old_fb);
	kmem_cache_free(priv->page_flip_slab, flip_res);
}

static void fdb_crtc_ref_fb_cb_kds(void *arg)
{
	struct fdb_drm_flip_resource *flip_res = arg;
	struct fdb_crtc *fdb_crtc = to_fdb_crtc(flip_res->crtc);
	struct fdb_drm_private *priv = flip_res->crtc->dev->dev_private;
	unsigned long flags;
	int flips_in_flight;

	spin_lock_irqsave(&fdb_crtc->current_displaying_lock, flags);

	/* Release the previous buffer */
	if (fdb_crtc->old_kds_res_set) {
		kds_resource_set_release(&fdb_crtc->old_kds_res_set);
	}

	/* Record the current buffer, to release on the next buffer flip */
	fdb_crtc->old_kds_res_set = flip_res->kds_res_set;
	fdb_crtc->displaying_fb = flip_res->cur_fb;

	spin_unlock_irqrestore(&fdb_crtc->current_displaying_lock, flags);

	flips_in_flight = atomic_dec_return(&priv->nr_flips_in_flight);
	if (flips_in_flight == 0) {
		wake_up(&priv->wait_for_flips);
	}

	dma_buf_put(flip_res->dmabuf);

	if (flip_res->old_fb) {
		/* use workqueue to avoid deadlock with fdb_fb_destroy */
		INIT_WORK(&flip_res->unref_work, fdb_crtc_ref_fb_work);
		queue_work(priv->wq, &flip_res->unref_work);
	}
	else {
		kmem_cache_free(priv->page_flip_slab, flip_res);
	}
}
#endif

static int fdb_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
		struct drm_framebuffer *old_fb)
{
	struct fdb_crtc *fdb_crtc = to_fdb_crtc(crtc);
	struct drm_plane *plane = fdb_crtc->plane;
	struct drm_display_mode *mode = &crtc->mode;
	struct drm_device *dev = crtc->dev;
	struct fdb_drm_private *priv = dev->dev_private;
	struct f_fdb_child *fdb_child_fb;
	int bytes_per_pixel = (crtc->fb->bits_per_pixel + 7) >> 3;
	int stride = bytes_per_pixel * crtc->fb->width;

	fdb_child_fb = bound_child_from_drm_crtc(priv, crtc);

	if (!fdb_child_fb)
		return -EINVAL;
	//dev_info(fdb_child_fb->dev, "fdb_crtc_mode_set_base %d %d\n", x, y);

	//drm_crtc_vblank_on(crtc);

	if (old_fb && fdb_child_fb->ops->set_paddr_ref_cb) {
		int ret;

		drm_framebuffer_reference(old_fb);
		ret = fdb_child_fb->ops->set_paddr_ref_cb(fdb_child_fb,
				get_linear_addr(crtc->fb) + (x * bytes_per_pixel) +
						(y * stride), fdb_crtc_ref_fb_cb, old_fb);
		if (ret) {
			pr_err("%s - set_paddr failed(%d) fb id:%d\n",
					__func__, ret, crtc->fb->base.id);
			drm_framebuffer_unreference(old_fb);
			return ret;
		}
	}
	else {
		fdb_child_fb->ops->set_paddr(fdb_child_fb,
				get_linear_addr(crtc->fb) + (x * bytes_per_pixel) +
						(y * stride));
	}

	if (old_fb)
		return 0;

	return fdb_plane_mode_set(plane, crtc, crtc->fb,
			0, 0, mode->hdisplay, mode->vdisplay,
			x << 16, y << 16,
			mode->hdisplay << 16, mode->vdisplay << 16,
			NULL, NULL);
}

void fdb_set_hw_fb(struct drm_crtc *drmcrtc)
{
	struct fdb_crtc *fdbcrtc = to_fdb_crtc(drmcrtc);
	struct drm_device *drmdev = drmcrtc->dev;
	struct fdb_drm_private *priv = drmdev->dev_private;
	struct drm_framebuffer *fb = drmcrtc->fb;
	struct fdb_video_timings *timings = &fdbcrtc->timings;
	struct f_fdb_child *fdb_child_fb;

	fdb_child_fb = bound_child_from_drm_crtc(priv, drmcrtc);

	if (!fdb_child_fb)
		return;

	timings->stride_px = fb->width;

	fdb_child_fb->ops->set_timings(fdb_child_fb, timings);

	fdb_crtc_mode_set_base(drmcrtc, 0, 0, fb);
}

void fdb_drm_crtc_set_head(struct drm_crtc *crtc, struct f_fdb_child *child)
{
	struct fdb_crtc *fdbcrtc = to_fdb_crtc(crtc);

	fdbcrtc->head = child;
}

/*
 * CRTC funcs:
 */

static void fdb_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct fdb_drm_private *priv = crtc->dev->dev_private;
	struct fdb_crtc *fdb_crtc = to_fdb_crtc(crtc);
	struct f_fdb_child *fdb_child_fb;
	bool enabled = (mode == DRM_MODE_DPMS_ON);
	int i;

	if (enabled == fdb_crtc->enabled) {
//		WARN_ON(1);
		return;
	}

	dev_info(crtc->dev->dev, "dpms %s: %d\n", fdb_crtc->name, mode);

	fdb_crtc->enabled = enabled;
	fdb_crtc->full_update = true;
	fdb_crtc_mode_set_base(crtc, 0, 0, NULL);

	fdb_child_fb = bound_child_from_drm_crtc(priv, crtc);
	if (!fdb_child_fb) {
		dev_err(crtc->dev->dev, "Unable to find bound child\n");
		return;
	}

	if (enabled) {
		fdb_child_fb->ops->enable(fdb_child_fb);
		//drm_crtc_vblank_on(crtc);
	} else {
		//drm_crtc_vblank_off(crtc);
		fdb_child_fb->ops->disable(fdb_child_fb);
	}

	/* also enable our private plane: */
	WARN_ON(fdb_plane_dpms(fdb_crtc->plane, mode));

	/* and any attached overlay planes: */
	for (i = 0; i < priv->num_planes; i++) {
		struct drm_plane *plane = priv->planes[i];
		if (plane->crtc == crtc)
			WARN_ON(fdb_plane_dpms(plane, mode));
	}

	if (fdb_crtc->head &&fdb_crtc->head->ops->enable) {
		dev_dbg(fdb_crtc->head->dev,
				"%s: setting head enable state to %d\n",
							    __func__, enabled);
		if (enabled)
			fdb_crtc->head->ops->enable(fdb_crtc->head);
		else
			fdb_crtc->head->ops->disable(fdb_crtc->head);
	}
}

static bool fdb_crtc_mode_fixup(struct drm_crtc *crtc,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	return true;
}

static int fdb_crtc_mode_set(struct drm_crtc *crtc,
		struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode,
		int x, int y,
		struct drm_framebuffer *old_fb)
{
	struct fdb_crtc *fdb_crtc = to_fdb_crtc(crtc);

	mode = adjusted_mode;

	dev_dbg(crtc->dev->dev,
	 "fdb_crtc_mode_set: %d:\"%s\" %d %d %d %d %d %d %d %d %d %d 0x%x 0x%x",
			mode->base.id, mode->name,
			mode->vrefresh, mode->clock,
			mode->hdisplay, mode->hsync_start,
			mode->hsync_end, mode->htotal,
			mode->vdisplay, mode->vsync_start,
			mode->vsync_end, mode->vtotal,
			mode->type, mode->flags);

	copy_timings_drm_to_fdb(crtc, &fdb_crtc->timings, mode);
	fdb_crtc->full_update = true;

	fdb_set_hw_fb(crtc);

	return fdb_plane_mode_set(fdb_crtc->plane, crtc, crtc->fb,
			0, 0, mode->hdisplay, mode->vdisplay,
			x << 16, y << 16,
			mode->hdisplay << 16, mode->vdisplay << 16,
			NULL, NULL);
}

static void fdb_crtc_prepare(struct drm_crtc *crtc)
{
	struct fdb_crtc *fdb_crtc = to_fdb_crtc(crtc);
	pr_info("fdb_crtc_prepare: %s\n", fdb_crtc->name);
	fdb_crtc_dpms(crtc, DRM_MODE_DPMS_OFF);
}

static void fdb_crtc_commit(struct drm_crtc *crtc)
{
	struct fdb_crtc *fdb_crtc = to_fdb_crtc(crtc);
	pr_info("fdb_crtc_commit: %s\n", fdb_crtc->name);
	fdb_crtc_dpms(crtc, DRM_MODE_DPMS_ON);
}

static void fdb_crtc_load_lut(struct drm_crtc *crtc)
{
}
/*
 * sets the hardware to do the pageflip next vsync
 */

static int fdb_crtc_page_flip_locked(struct drm_crtc *crtc,
		 struct drm_framebuffer *fb,
		 struct drm_pending_vblank_event *event)
{
	struct fdb_crtc *fdb_crtc = to_fdb_crtc(crtc);
	struct fdb_drm_private *priv = crtc->dev->dev_private;
	struct f_fdb_child *fdb_child_fb = bound_child_from_drm_crtc(priv, crtc);
	u32 paddr = get_linear_addr(fb);
	struct drm_framebuffer *old_fb = crtc->fb;
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	struct drm_gem_object *bo = fdb_fb_bo(fb, 0);
#endif

	pr_debug("%d -> %d (event=%p)", crtc->fb ? crtc->fb->base.id : -1,
			fb->base.id, event);
	if (!fdb_child_fb)
		return -EINVAL;

//	pr_info("%d -> %d (event=%p) paddr=0x%x\n", crtc->primary->fb ? crtc->primary->fb->base.id : -1,
//			fb->base.id, event, paddr);

	fdb_crtc->event = event;
	crtc->fb = fb;

	/* fdb-drm does not support DRM_MODE_PAGE_FLIP_EVENT */
	WARN_ON_ONCE(event);

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	/*
	 * This function is called from ioctl(), so export_dma_buf would be referenced
	 * by callee process(export_dma_buf's refcount is > 0).
	 * It is not necessary to hold object_name_lock.
	 */
	if (bo->export_dma_buf) {
		struct dma_buf *buf = bo->export_dma_buf;
		unsigned long shared[1] = { 0 };
		struct kds_resource *resource_list[1] = {
				get_dma_buf_kds_resource(buf) };
		int err;
		struct fdb_drm_flip_resource *flip_res =
					kmem_cache_alloc(priv->page_flip_slab, GFP_KERNEL);
		if (!flip_res) {
			pr_err("kmem_cache_alloc failed to alloc - flip ignored\n");
			return -ENOMEM;
		}

		/*
		 * ARM Mali GPU driver is the unique component which use KDS resource,
		 * so we care about only ARM Mali Midgard DDK's use case on this sequence.
		 */

		get_dma_buf(buf);
		if (old_fb)
			drm_framebuffer_reference(old_fb);

		flip_res->crtc = crtc;
		flip_res->dmabuf = buf;
		flip_res->cur_fb = fb;
		flip_res->old_fb = old_fb;
		atomic_inc(&priv->nr_flips_in_flight);

		/* Wait for the KDS resource associated with this buffer */
		err = kds_async_waitall(&flip_res->kds_res_set,
					&priv->kds_cb, flip_res, fb, 1, shared,
					resource_list);
		if (err) {
			pr_err("kds_async_waitall failed(%d)\n", err);
			dma_buf_put(buf);
			if (old_fb)
				drm_framebuffer_unreference(old_fb);
			kmem_cache_free(priv->page_flip_slab, flip_res);
		}

		return err;
	}
#endif

	if (old_fb && fdb_child_fb->ops->set_paddr_ref_cb) {
		int ret;

		drm_framebuffer_reference(old_fb);
		ret = fdb_child_fb->ops->set_paddr_ref_cb(
					fdb_child_fb, paddr, fdb_crtc_ref_fb_cb, old_fb);
		if (ret) {
			pr_err("%s - set_paddr failed(%d) fb id:%d, paddr=%#x\n",
				__func__, ret, fb->base.id, paddr);
			drm_framebuffer_unreference(old_fb);
			return ret;
		}
	}
	else {
		fdb_child_fb->ops->set_paddr(fdb_child_fb, paddr);
	}

	return 0;
}

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
void fdb_crtc_page_flip_cb(void *cb1, void *cb2)
{
	struct fdb_drm_flip_resource *flip_res = cb1;
	struct drm_framebuffer *fb = cb2;
	struct fdb_drm_private *priv = flip_res->crtc->dev->dev_private;
	struct f_fdb_child *fdb_child_fb = bound_child_from_drm_crtc(priv, flip_res->crtc);
	u32 paddr = get_linear_addr(fb);
	int ret;

	pr_debug("%s - fb:%p(id:%d, bo:%p) paddr=%#x\n",
		__func__, fb, fb->base.id, fdb_fb_bo(fb, 0), paddr);

	if (fdb_child_fb->ops->set_paddr_ref_cb) {
		ret = fdb_child_fb->ops->set_paddr_ref_cb(
						fdb_child_fb, paddr, fdb_crtc_ref_fb_cb_kds, flip_res);
		if (!ret) {
			return;
		}
		else {
			pr_err("%s - set_paddr failed(%d) fb id:%d, paddr=%#x\n",
				__func__, ret, fb->base.id, paddr);
		}
	}
	else {
		fdb_child_fb->ops->set_paddr(fdb_child_fb, paddr);
	}

	fdb_crtc_ref_fb_cb_kds(flip_res);
}
#endif

static int fdb_crtc_set_property(struct drm_crtc *crtc,
		struct drm_property *property, uint64_t val)
{
	struct fdb_crtc *fdb_crtc = to_fdb_crtc(crtc);
	struct fdb_drm_private *priv = crtc->dev->dev_private;

	if (property == priv->rotation_prop)
		crtc->invert_dimensions =
		   !!(val & ((1LL << DRM_ROTATE_90) | (1LL << DRM_ROTATE_270)));

	return fdb_plane_set_property(fdb_crtc->plane, property, val);
}

static const struct drm_crtc_funcs fdb_crtc_funcs = {
	.set_config = drm_crtc_helper_set_config,
	.destroy = fdb_crtc_destroy,
	.page_flip = fdb_crtc_page_flip_locked,
	.set_property = fdb_crtc_set_property,
};

static const struct drm_crtc_helper_funcs fdb_crtc_helper_funcs = {
	.dpms = fdb_crtc_dpms,
	.mode_fixup = fdb_crtc_mode_fixup,
	.mode_set = fdb_crtc_mode_set,
	.prepare = fdb_crtc_prepare,
	.commit = fdb_crtc_commit,
	.mode_set_base = fdb_crtc_mode_set_base,
	.load_lut = fdb_crtc_load_lut,
};

const struct fdb_video_timings *fdb_crtc_timings(struct drm_crtc *crtc)
{
	struct fdb_crtc *fdb_crtc = to_fdb_crtc(crtc);
	return &fdb_crtc->timings;
}

void fdb_crtc_call_encoder_update(struct drm_crtc *crtc)
{
	struct fdb_crtc *fdb_crtc;
	struct drm_encoder *encoder = NULL;
	struct fdb_drm_private *priv;
	int i;

	if (!crtc)
		return;
	fdb_crtc = to_fdb_crtc(crtc);
	if (!fdb_crtc || !crtc->dev)
		return;
	priv = crtc->dev->dev_private;
	if (!priv)
		return;

	pr_debug(
		"fdb_crtc_call_encoder_update: %s: enabled=%d, full=%d",
				fdb_crtc->name, fdb_crtc->enabled,
							fdb_crtc->full_update);
	for (i = 0; i < priv->num_encoders; i++)
		if (priv->encoders[i]->crtc == crtc) {
			encoder = priv->encoders[i];
			break;
		}

	if (encoder) {
		if (!fdb_crtc->enabled)
			fdb_encoder_set_enabled(encoder, false);
		else {
			fdb_encoder_set_enabled(encoder, false);
			fdb_encoder_update(encoder, &fdb_crtc->timings);
			fdb_encoder_set_enabled(encoder, true);
		}
	} else
		dev_err(crtc->dev->dev, "fdb_crtc_call_encoder_update: unable to find encoder\n");
}

/* initialize crtc */
struct drm_crtc *fdb_crtc_init(struct drm_device *dev,
		struct drm_plane *plane, int channel, int id)
{
	struct drm_crtc *crtc = NULL;
	struct fdb_crtc *fdb_crtc;
/*	struct fdb_overlay_manager_info *info; */

	dev_dbg(dev->dev, "%d\n", channel);

	fdb_crtc = kzalloc(sizeof(*fdb_crtc), GFP_KERNEL);
	if (!fdb_crtc)
		goto fail;

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	spin_lock_init(&fdb_crtc->current_displaying_lock);
	fdb_crtc->old_kds_res_set = NULL;
	fdb_crtc->displaying_fb = NULL;
#endif

	crtc = &fdb_crtc->base;

	fdb_crtc->plane = plane;
	fdb_crtc->plane->crtc = crtc;
	sprintf(fdb_crtc->name, "crtc%d", id);
	fdb_crtc->pipe = id;

	drm_crtc_init(dev, crtc, &fdb_crtc_funcs);
	drm_crtc_helper_add(crtc, &fdb_crtc_helper_funcs);

	fdb_plane_install_properties(fdb_crtc->plane, &crtc->base);

	return crtc;

fail:
	if (crtc)
		fdb_crtc_destroy(crtc);

	return NULL;
}

static void fdb_crtc_destroy(struct drm_crtc *crtc)
{
	struct fdb_crtc *fdb_crtc = to_fdb_crtc(crtc);

	fdb_crtc->plane->funcs->destroy(fdb_crtc->plane);
	drm_crtc_cleanup(crtc);

	kfree(fdb_crtc);
}
