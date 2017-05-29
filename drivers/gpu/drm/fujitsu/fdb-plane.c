/*
 * drivers/gpu/drm/omapdrm/fdb_plane.c
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Rob Clark <rob.clark@linaro.org>
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

#include <linux/kfifo.h>

#include <video/fdb.h>

#include "fdb-drm.h"

/* some hackery because omapdss has an 'enum fdb_plane' (which would be
 * better named fdb_plane_id).. and compiler seems unhappy about having
 * both a 'struct fdb_plane' and 'enum fdb_plane'
 */
#define fdb_plane _fdb_plane
#define to_fdb_plane(x) container_of(x, struct fdb_plane, base)

struct fdb_plane {
	struct drm_plane base;
	int id;
	const char *name;
	struct fdb_overlay_info info;

	/* position/orientation of scanout within the fb: */
	struct fdb_drm_window win;
	bool enabled;

	/* last fb that we pinned: */
	struct drm_framebuffer *pinned_fb;

	uint32_t nformats;
	uint32_t formats[32];

	/* set of bo's pending unpin until next post_apply() */
	DECLARE_KFIFO_PTR(unpin_fifo, struct drm_gem_object *);
};

int fdb_plane_mode_set(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb,
		int crtc_x, int crtc_y,
		unsigned int crtc_w, unsigned int crtc_h,
		uint32_t src_x, uint32_t src_y,
		uint32_t src_w, uint32_t src_h,
		void (*fxn)(void *), void *arg)
{
	struct fdb_plane *fdb_plane = to_fdb_plane(plane);
	struct fdb_drm_window *win = &fdb_plane->win;
	struct drm_gem_object *bo = NULL;

	/*
	 * these settings are shadowed, setting the fb PA will trigger
	 * shadow load on next pipeline empty
	 *
	 * So there is no other need for synchronizing the setting action
	 */

	win->crtc_x = crtc_x;
	win->crtc_y = crtc_y;
	win->crtc_w = crtc_w;
	win->crtc_h = crtc_h;

	/* src values are in Q16 fixed point, convert to integer: */
	win->src_x = src_x >> 16;
	win->src_y = src_y >> 16;
	win->src_w = src_w >> 16;
	win->src_h = src_h >> 16;

	plane->fb = fb;
	plane->crtc = crtc;

	fdb_crtc_call_encoder_update(crtc);

	while (kfifo_get(&fdb_plane->unpin_fifo, &bo)) {
		fdb_gem_put_paddr(bo);
		drm_gem_object_unreference_unlocked(bo);
	}

	if (fxn)
		fxn(arg);

	return 0;
}

static int fdb_plane_update(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb,
		int crtc_x, int crtc_y,
		unsigned int crtc_w, unsigned int crtc_h,
		uint32_t src_x, uint32_t src_y,
		uint32_t src_w, uint32_t src_h)
{
	struct fdb_plane *fdb_plane = to_fdb_plane(plane);
	fdb_plane->enabled = true;
	return fdb_plane_mode_set(plane, crtc, fb,
			crtc_x, crtc_y, crtc_w, crtc_h,
			src_x, src_y, src_w, src_h,
			NULL, NULL);
}

static int fdb_plane_disable(struct drm_plane *plane)
{
	struct fdb_plane *fdb_plane = to_fdb_plane(plane);
	fdb_plane->win.rotation = BIT(DRM_ROTATE_0);
	return fdb_plane_dpms(plane, DRM_MODE_DPMS_OFF);
}

static void fdb_plane_destroy(struct drm_plane *plane)
{
	struct fdb_plane *fdb_plane = to_fdb_plane(plane);

	fdb_plane_disable(plane);
	drm_plane_cleanup(plane);

	WARN_ON(!kfifo_is_empty(&fdb_plane->unpin_fifo));
	kfifo_free(&fdb_plane->unpin_fifo);

	kfree(fdb_plane);
}

int fdb_plane_dpms(struct drm_plane *plane, int mode)
{
	struct fdb_plane *fdb_plane = to_fdb_plane(plane);
	bool enabled = (mode == DRM_MODE_DPMS_ON);
	int ret = 0;

	if (enabled != fdb_plane->enabled) {
		pr_debug("fdb_plane_dpms: changing to enabled= %d\n", enabled);
		fdb_plane->enabled = enabled;
//		ret = apply(plane);
	}

	return ret;
}

/* helper to install properties which are common to planes and crtcs */
void fdb_plane_install_properties(struct drm_plane *plane,
		struct drm_mode_object *obj)
{
	struct drm_device *dev = plane->dev;
	struct fdb_drm_private *priv = dev->dev_private;
	struct drm_property *prop;

	prop = priv->zorder_prop;
	if (!prop) {
		prop = drm_property_create_range(dev, 0, "zorder", 0, 3);
		if (prop == NULL)
			return;
		priv->zorder_prop = prop;
	}
	drm_object_attach_property(obj, prop, 0);
}

int fdb_plane_set_property(struct drm_plane *plane,
		struct drm_property *property, uint64_t val)
{
	struct fdb_plane *fdb_plane = to_fdb_plane(plane);
	struct fdb_drm_private *priv = plane->dev->dev_private;
	int ret = -EINVAL;

	if (property == priv->rotation_prop) {
		pr_err("%s: rotation: %02x", fdb_plane->name, (uint32_t)val);
		fdb_plane->win.rotation = val;
//		ret = apply(plane);
	} else if (property == priv->zorder_prop) {
		pr_err("%s: zorder: %02x", fdb_plane->name, (uint32_t)val);
		fdb_plane->info.zorder = val;
//		ret = apply(plane);
	}

	return ret;
}

static const struct drm_plane_funcs fdb_plane_funcs = {
		.update_plane = fdb_plane_update,
		.disable_plane = fdb_plane_disable,
		.destroy = fdb_plane_destroy,
		.set_property = fdb_plane_set_property,
};

#if 0
static void fdb_plane_error_irq(struct fdb_drm_irq *irq, uint32_t irqstatus)
{
	struct fdb_plane *fdb_plane =
			container_of(irq, struct fdb_plane, error_irq);
	DRM_ERROR("%s: errors: %08x\n", fdb_plane->name, irqstatus);
}
#endif

#if 0
static const char * const plane_names[] = {
		[FDB_DSS_GFX] = "gfx",
		[FDB_DSS_VIDEO1] = "vid1",
		[FDB_DSS_VIDEO2] = "vid2",
		[FDB_DSS_VIDEO3] = "vid3",
};

static const uint32_t error_irqs[] = {
		[FDB_DSS_GFX] = DISPC_IRQ_GFX_FIFO_UNDERFLOW,
		[FDB_DSS_VIDEO1] = DISPC_IRQ_VID1_FIFO_UNDERFLOW,
		[FDB_DSS_VIDEO2] = DISPC_IRQ_VID2_FIFO_UNDERFLOW,
		[FDB_DSS_VIDEO3] = DISPC_IRQ_VID3_FIFO_UNDERFLOW,
};
#endif


static int callback_collect_formats(struct f_fdb_child *child, void *arg)
{
	struct fdb_plane *fdb_plane = (struct fdb_plane *)arg;
	const struct fdb_format *format;
	int hit, len, n, m;

	if (!child->ops)
		return 0;

	if (!child->ops->get_fdb_formats)
		return 0;
	format = child->ops->get_fdb_formats(child, &len);
	if (!format)
		return 0;

	for (n = 0; n < len; n++) {
		hit = 0;
		for (m = 0; m < fdb_plane->nformats; m++)
			if (fdb_plane->formats[m] == format[n].pixel_format)
				hit = 1;
		if (hit)
			continue;

		if (fdb_plane->nformats < ARRAY_SIZE(fdb_plane->formats))
			fdb_plane->formats[fdb_plane->nformats++] =
							format[n].pixel_format;
	}

	return 0;
}

/* initialize plane */
struct drm_plane *fdb_plane_init(struct drm_device *dev,
		int id, bool private_plane)
{
	struct fdb_drm_private *priv = dev->dev_private;
	struct drm_plane *plane = NULL;
	struct fdb_plane *fdb_plane;
	struct fdb_overlay_info *info;
	int ret;

	dev_dbg(dev->dev, "fdb_plane_init\n");
/*	pr_err("%s: priv=%d", plane_names[id], private_plane); */

	fdb_plane = kzalloc(sizeof(*fdb_plane), GFP_KERNEL);
	if (!fdb_plane)
		goto fail;

	ret = kfifo_alloc(&fdb_plane->unpin_fifo, 16, GFP_KERNEL);
	if (ret) {
		dev_err(dev->dev, "could not allocate unpin FIFO\n");
		goto fail;
	}

	fdb_callback_each(dev->dev, callback_collect_formats, fdb_plane);

	fdb_plane->id = id;
	fdb_plane->name = "plane-name";

	plane = &fdb_plane->base;

	drm_plane_init(dev, plane, (1 << priv->num_crtcs) - 1, &fdb_plane_funcs,
			fdb_plane->formats, fdb_plane->nformats, private_plane);

	fdb_plane_install_properties(plane, &plane->base);

	/* get our starting configuration, set defaults for parameters
	 * we don't currently use, etc:
	 */
	info = &fdb_plane->info;
	info->rotation = FDB_ROT_0;
	info->global_alpha = 0xff;
	info->mirror = 0;

	/* Set defaults depending on whether we are a CRTC or overlay
	 * layer.
	 * TODO add ioctl to give userspace an API to change this.. this
	 * will come in a subsequent patch.
	 */
	if (private_plane)
		fdb_plane->info.zorder = 0;
	else
		fdb_plane->info.zorder = id;

	return plane;

fail:
	if (plane)
		fdb_plane_destroy(plane);

	return NULL;
}
