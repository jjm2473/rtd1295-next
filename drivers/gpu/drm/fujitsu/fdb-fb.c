/*
 * drivers/gpu/drm/fujitsu/mb8ac0300-fb.c
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

#include <video/fdb.h>

#include "fdb-drm.h"
#include "drm_crtc.h"
#include "drm_crtc_helper.h"

#include <drm/drmP.h>

/* per-plane info for the fb: */
struct plane {
	struct drm_gem_object *bo;
	uint32_t pitch;
	uint32_t offset;
	phys_addr_t paddr;
};

#define to_fdb_fb(x) \
		container_of(x, struct fdb_fb, base)

struct fdb_fb {
	struct drm_framebuffer base;
	const struct fdb_format *format;
	struct plane planes[4];
};

static int fdb_fb_create_handle(struct drm_framebuffer *fb,
		struct drm_file *file_priv,
		unsigned int *handle)
{
	struct fdb_fb *fdb_fb = to_fdb_fb(fb);

	return drm_gem_handle_create(file_priv,
			fdb_fb->planes[0].bo, handle);
}

static void fdb_fb_destroy(struct drm_framebuffer *fb)
{
	struct fdb_fb *fdb_fb = to_fdb_fb(fb);
	int i, n = drm_format_num_planes(fb->pixel_format);
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	struct fdb_drm_private *priv = fb->dev->dev_private;
	struct drm_crtc *crtc;
	unsigned long flags;
#endif

	pr_debug("destroy: FB ID: %d (%p)\n", fb->base.id, fb);

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	if (!wait_event_timeout(priv->wait_for_flips,
			(atomic_read(&priv->nr_flips_in_flight) == 0),
			msecs_to_jiffies(1000))) {
		pr_err("waiting for page flip is timeout\n");
	}

	list_for_each_entry(crtc, &fb->dev->mode_config.crtc_list, head) {
		struct fdb_crtc *fdb_crtc = to_fdb_crtc(crtc);

		spin_lock_irqsave(&fdb_crtc->current_displaying_lock, flags);

		if (fb == fdb_crtc->displaying_fb && fdb_crtc->old_kds_res_set) {
			kds_resource_set_release(&fdb_crtc->old_kds_res_set);
			fdb_crtc->old_kds_res_set = NULL;
		}

		spin_unlock_irqrestore(&fdb_crtc->current_displaying_lock, flags);
	}
#endif

	drm_framebuffer_cleanup(fb);

	for (i = 0; i < n; i++) {
		struct plane *plane = &fdb_fb->planes[i];
		if (plane->bo)
			drm_gem_object_unreference_unlocked(plane->bo);
	}

	kfree(fdb_fb);
}

phys_addr_t get_linear_addr(struct drm_framebuffer *fb)
{
	struct fdb_fb *fdb_fb = to_fdb_fb(fb);
	phys_addr_t paddr;

	fdb_gem_get_paddr(fdb_fb->planes[0].bo, &paddr, 0);

	return paddr;
}

static const struct drm_framebuffer_funcs fdb_fb_funcs = {
	.create_handle = fdb_fb_create_handle,
	.destroy = fdb_fb_destroy,
};
#if 0
static uint32_t get_linear_addr(struct plane *plane,
		const struct format *format, int n, int x, int y)
{
	uint32_t offset;

	offset = plane->offset +
			(x * format->planes[n].stride_bpp) +
			(y * plane->pitch / format->planes[n].sub_y);

	return plane->paddr + offset;
}

/* update ovl info for scanout, handles cases of multi-planar fb's, etc.
 */
void fdb_fb_update_scanout(struct drm_framebuffer *fb,
	  struct fdb_drm_window *win, struct fdb_overlay_info *info)
{
	struct fdb_fb *fdb_fb = to_fdb_fb(fb);
	const struct format *format = fdb_fb->format;
	struct plane *plane = &fdb_fb->planes[0];
	uint32_t x, y, orient = 0;

	info->color_mode = format->dss_format;

	info->pos_x      = win->crtc_x;
	info->pos_y      = win->crtc_y;
	info->out_width  = win->crtc_w;
	info->out_height = win->crtc_h;
	info->width      = win->src_w;
	info->height     = win->src_h;

	x = win->src_x;
	y = win->src_y;

	info->paddr         = get_linear_addr(plane, format, 0, x, y);
	info->rotation_type = FDB_DSS_ROT_DMA;
	info->screen_width  = plane->pitch;

	/* convert to pixels: */
	info->screen_width /= format->planes[0].stride_bpp;
	info->p_uv_addr = 0;
}
#endif

/* Call for unpin 'a' (if not NULL), and pin 'b' (if not NULL).  Although
 * buffers to unpin are just pushed to the unpin fifo so that the
 * caller can defer unpin until vblank.
 *
 * Note if this fails (ie. something went very wrong!), all buffers are
 * unpinned, and the caller disables the overlay.  We could have tried
 * to revert back to the previous set of pinned buffers but if things are
 * hosed there is no guarantee that would succeed.
 */
int fdb_fb_replace(struct drm_framebuffer *a,
		struct drm_framebuffer *b, void *arg,
		void (*unpin)(void *arg, struct drm_gem_object *bo))
{
	int ret = 0, i, na, nb;
	struct fdb_fb *ofba = to_fdb_fb(a);
	struct fdb_fb *ofbb = to_fdb_fb(b);
	uint32_t pinned_mask = 0;

	na = a ? drm_format_num_planes(a->pixel_format) : 0;
	nb = b ? drm_format_num_planes(b->pixel_format) : 0;

	for (i = 0; i < max(na, nb); i++) {
		struct plane *pa, *pb;

		pa = (i < na) ? &ofba->planes[i] : NULL;
		pb = (i < nb) ? &ofbb->planes[i] : NULL;

		if (pa)
			unpin(arg, pa->bo);

		if (!pb || ret)
			continue;

		ret = fdb_gem_get_paddr(pb->bo, &pb->paddr, true);
		if (ret)
			continue;

		fdb_gem_dma_sync(pb->bo, DMA_TO_DEVICE);
		pinned_mask |= (1 << i);
	}

	if (!ret)
		return 0;

	/* something went wrong.. unpin what has been pinned */
	for (i = 0; i < nb; i++)
		if (pinned_mask & (1 << i)) {
			struct plane *pb = &ofba->planes[i];
			unpin(arg, pb->bo);
		}

	return ret;
}

struct drm_gem_object *fdb_fb_bo(struct drm_framebuffer *fb, int p)
{
	struct fdb_fb *fdb_fb;

	if (!fb)
		return NULL;

	fdb_fb = to_fdb_fb(fb);

	if (p >= drm_format_num_planes(fb->pixel_format))
		return NULL;

	return fdb_fb->planes[p].bo;
}

/* iterate thru all the connectors, returning ones that are attached
 * to the same fb..
 */
struct drm_connector *fdb_fb_get_next_connector(
		struct drm_framebuffer *fb, struct drm_connector *from)
{
	struct drm_device *dev = fb->dev;
	struct list_head *connector_list = &dev->mode_config.connector_list;
	struct drm_connector *connector = from;
	struct drm_encoder *encoder;
/*	struct drm_crtc *crtc; */

	if (!from)
		return list_first_entry(connector_list, typeof(*from), head);

	list_for_each_entry_from(connector, connector_list, head) {
		if (connector == from)
			continue;

		encoder = connector->encoder;
		if (!encoder)
			continue;
		if (!encoder->crtc)
			continue;
		if (encoder->crtc->fb == fb)
			return connector;
	}

	return NULL;
}

#ifdef CONFIG_DEBUG_FS
void fdb_fb_describe(struct drm_framebuffer *fb, struct seq_file *m)
{
	struct fdb_fb *fdb_fb = to_fdb_fb(fb);
	int i, n = drm_format_num_planes(fb->pixel_format);

	seq_printf(m, "fb: %dx%d@%4.4s\n", fb->width, fb->height,
			(char *)&fb->pixel_format);

	for (i = 0; i < n; i++) {
		struct plane *plane = &fdb_fb->planes[i];
		seq_printf(m, "   %d: offset=%d pitch=%d, obj: ",
				i, plane->offset, plane->pitch);
		fdb_gem_describe(plane->bo, m);
	}
}
#endif

struct drm_framebuffer *fdb_fb_create(struct drm_device *dev,
		struct drm_file *file, struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct drm_gem_object *bos[4];
	struct drm_framebuffer *fb;
	int ret;

	ret = objects_lookup(dev, file, mode_cmd->pixel_format,
			bos, mode_cmd->handles);
	if (ret)
		return ERR_PTR(ret);

	fb = fdb_fb_init(dev, mode_cmd, bos);
	if (IS_ERR(fb)) {
		int i, n = drm_format_num_planes(mode_cmd->pixel_format);
		for (i = 0; i < n; i++)
			drm_gem_object_unreference_unlocked(bos[i]);
		return fb;
	}

	return fb;
}

struct fdb_find_format {
	const struct fdb_format *result;
	uint32_t pixel_format;
};

static int callback_find_format(struct f_fdb_child *child, void *arg)
{
	struct fdb_find_format *ff = (struct fdb_find_format *)arg;
	const struct fdb_format *format;
	int len, n;

	if (ff->result)
		return 0;

	if (!child->ops)
		return 0;

	if (!child->ops->get_fdb_formats)
		return 0;
	format = child->ops->get_fdb_formats(child, &len);
	if (!format)
		return 0;

	for (n = 0; n < len; n++)
		if (ff->pixel_format == format[n].pixel_format) {
			ff->result = &format[n];
			return 0;
		}

	return 0;
}

struct drm_framebuffer *fdb_fb_init(struct drm_device *dev,
		struct drm_mode_fb_cmd2 *mode_cmd, struct drm_gem_object **bos)
{
	struct fdb_fb *fdb_fb;
	struct drm_framebuffer *fb = NULL;
	struct fdb_find_format ff;
	int ret, i, n = drm_format_num_planes(mode_cmd->pixel_format);

	ff.pixel_format = mode_cmd->pixel_format;
	ff.result = NULL;

	dev_info(dev->dev,
		"create framebuffer: dev=%p, mode_cmd=%p (%dx%d@%4.4s)\n",
			dev, mode_cmd, mode_cmd->width, mode_cmd->height,
			(char *)&mode_cmd->pixel_format);

	fdb_callback_each(dev->dev, callback_find_format, &ff);
	if (!ff.result) {
		dev_info(dev->dev, "unsupported pixel format: %4.4s\n",
				(char *)&mode_cmd->pixel_format);
		ret = -EINVAL;
		goto fail;
	}

	fdb_fb = kzalloc(sizeof(*fdb_fb), GFP_KERNEL);
	if (!fdb_fb) {
		ret = -ENOMEM;
		goto fail;
	}

	fb = &fdb_fb->base;
	fdb_fb->format = ff.result;

	for (i = 0; i < n; i++) {
		struct plane *plane = &fdb_fb->planes[i];
		int size, pitch = mode_cmd->pitches[i];

		if (pitch < (mode_cmd->width *
				fdb_fb->format->planes[i].stride_bpp)) {
			dev_err(dev->dev,
				"provided buffer pitch is too small! %d < %d\n",
				pitch,
				mode_cmd->width *
					fdb_fb->format->planes[i].stride_bpp);
			ret = -EINVAL;
			goto fail;
		}

		size = pitch * mode_cmd->height /
					fdb_fb->format->planes[i].sub_y;

		if (size > (fdb_gem_mmap_size(bos[i]) - mode_cmd->offsets[i])) {
			dev_err(dev->dev,
			   "provided buffer object is too small! %d < %d\n",
				     bos[i]->size - mode_cmd->offsets[i], size);
			ret = -EINVAL;
			goto fail;
		}

		plane->bo     = bos[i];
		plane->offset = mode_cmd->offsets[i];
		plane->pitch  = pitch;
		plane->paddr  = 0;
	}

	drm_helper_mode_fill_fb_struct(fb, mode_cmd);

	ret = drm_framebuffer_init(dev, fb, &fdb_fb_funcs);
	if (ret) {
		dev_err(dev->dev, "framebuffer init failed: %d\n", ret);
		goto fail;
	}

	dev_dbg(dev->dev, "create: FB ID: %d (%p)\n", fb->base.id, fb);

	return fb;

fail:
	if (fb)
		fdb_fb_destroy(fb);

	return ERR_PTR(ret);
}
