/*
 * drivers/gpu/drm/fujitsu/fdb_connector.c
 *
 * Copyright (C) 2013 Linaro Ltd
 * Author Andy Green <andy.green@linaro.org>
 *
 * based on omap_connector.c -->
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

#include "fdb-drm.h"

#include <video/fdb.h>

#include "drm_crtc.h"
#include "drm_crtc_helper.h"

/*
 * connector funcs
 */

#define to_fdb_connector(x) container_of(x, struct fdb_connector, base)

enum private_ints {
	FDBDRM_EPI_BPP,
	FDBDRM_EPI_STRIDE_PX,
	FDBDRM_EPI_VIRTX,
	FDBDRM_EPI_VIRTY,
	FDBDRM_EPI_RED_OFFSET,
	FDBDRM_EPI_RED_LENGTH,
	FDBDRM_EPI_GREEN_OFFSET,
	FDBDRM_EPI_GREEN_LENGTH,
	FDBDRM_EPI_BLUE_OFFSET,
	FDBDRM_EPI_BLUE_LENGTH,
	FDBDRM_EPI_ALPHA_OFFSET,
	FDBDRM_EPI_ALPHA_LENGTH,
	FDBDRM_EPI_SCANOUT_ROTATION,

	/* always last */
	FDBDRM_EPI_COUNT
};

struct fdb_connector {
	struct drm_connector base;
	struct f_fdb_child *fdb;
	struct drm_encoder *encoder;
};

void copy_timings_fdb_to_drm(struct drm_display_mode *mode,
					struct fdb_video_timings *timings)
{
	if (!mode->private) {
		mode->private = kmalloc(FDBDRM_EPI_COUNT * sizeof(int), GFP_KERNEL);
		mode->private_size = FDBDRM_EPI_COUNT * sizeof(int);
	}

	if (!timings->bits_per_pixel) {
		pr_info("copy_timings_fdb_to_drm... zero bpp seen on incoming fdb_video_timings\n");
		//WARN_ON(1);
	}

	mode->private[FDBDRM_EPI_BPP] = timings->bits_per_pixel;
	mode->private[FDBDRM_EPI_STRIDE_PX] = timings->stride_px;
	mode->private[FDBDRM_EPI_VIRTX] = timings->xres_virtual;
	mode->private[FDBDRM_EPI_VIRTY] = timings->yres_virtual;
	mode->private[FDBDRM_EPI_RED_OFFSET] = timings->red_offset;
	mode->private[FDBDRM_EPI_RED_LENGTH] = timings->red_length;
	mode->private[FDBDRM_EPI_GREEN_OFFSET] = timings->green_offset;
	mode->private[FDBDRM_EPI_GREEN_LENGTH] = timings->green_length;
	mode->private[FDBDRM_EPI_BLUE_OFFSET] = timings->blue_offset;
	mode->private[FDBDRM_EPI_BLUE_LENGTH] = timings->blue_length;
	mode->private[FDBDRM_EPI_ALPHA_OFFSET] = timings->alpha_offset;
	mode->private[FDBDRM_EPI_ALPHA_LENGTH] = timings->alpha_length;
	mode->private[FDBDRM_EPI_SCANOUT_ROTATION] = timings->scanout_rotation;

	mode->clock = timings->dt.pixelclock.typ;

	mode->hdisplay = timings->dt.hactive.typ;
	mode->hsync_start = mode->hdisplay + timings->dt.hfront_porch.typ;
	mode->hsync_end = mode->hsync_start + timings->dt.hsync_len.typ;
	mode->htotal = mode->hsync_end + timings->dt.hback_porch.typ;

	mode->vdisplay = timings->dt.vactive.typ;
	mode->vsync_start = mode->vdisplay + timings->dt.vfront_porch.typ;
	mode->vsync_end = mode->vsync_start + timings->dt.vsync_len.typ;
	mode->vtotal = mode->vsync_end + timings->dt.vback_porch.typ;

	mode->flags = 0;

	if (timings->dt.flags & DISPLAY_FLAGS_INTERLACED)
		mode->flags |= DRM_MODE_FLAG_INTERLACE;

	if (timings->dt.flags & DISPLAY_FLAGS_HSYNC_HIGH)
		mode->flags |= DRM_MODE_FLAG_PHSYNC;
	else
		mode->flags |= DRM_MODE_FLAG_NHSYNC;

	if (timings->dt.flags & DISPLAY_FLAGS_VSYNC_HIGH)
		mode->flags |= DRM_MODE_FLAG_PVSYNC;
	else
		mode->flags |= DRM_MODE_FLAG_NVSYNC;
}

void copy_timings_drm_to_fdb(struct drm_crtc *crtc,
	struct fdb_video_timings *timings, struct drm_display_mode *mode)
{
	if (mode->private) {
		timings->bits_per_pixel = mode->private[FDBDRM_EPI_BPP];
		timings->stride_px = mode->private[FDBDRM_EPI_STRIDE_PX];
		timings->xres_virtual = mode->private[FDBDRM_EPI_VIRTX];
		timings->yres_virtual = mode->private[FDBDRM_EPI_VIRTY];
		timings->red_offset = mode->private[FDBDRM_EPI_RED_OFFSET];
		timings->red_length = mode->private[FDBDRM_EPI_RED_LENGTH];
		timings->green_offset = mode->private[FDBDRM_EPI_GREEN_OFFSET];
		timings->green_length = mode->private[FDBDRM_EPI_GREEN_LENGTH];
		timings->blue_offset = mode->private[FDBDRM_EPI_BLUE_OFFSET];
		timings->blue_length = mode->private[FDBDRM_EPI_BLUE_LENGTH];
		timings->alpha_offset = mode->private[FDBDRM_EPI_ALPHA_OFFSET];
		timings->alpha_length = mode->private[FDBDRM_EPI_ALPHA_LENGTH];
		timings->scanout_rotation = mode->private[FDBDRM_EPI_SCANOUT_ROTATION];
	}

	if (!timings->bits_per_pixel) {
		// pr_err("copy_timings_drm_to_fdb... zero bpp seen on outgoing fdb_video_timings\n");
		//WARN_ON(1);
		timings->bits_per_pixel = 32;
	}

	if (crtc)
		crtc->invert_dimensions = (timings->scanout_rotation == 90 ||
					timings->scanout_rotation == 270);

	timings->width_mm = mode->width_mm;
	timings->height_mm = mode->height_mm;

	timings->dt.pixelclock.typ = mode->clock;

	timings->dt.hactive.typ = mode->hdisplay;
	timings->dt.hfront_porch.typ = mode->hsync_start - mode->hdisplay;
	timings->dt.hsync_len.typ = mode->hsync_end - mode->hsync_start;
	timings->dt.hback_porch.typ = mode->htotal - mode->hsync_end;

	timings->dt.vactive.typ = mode->vdisplay;
	timings->dt.vfront_porch.typ = mode->vsync_start - mode->vdisplay;
	timings->dt.vsync_len.typ = mode->vsync_end - mode->vsync_start;
	timings->dt.vback_porch.typ = mode->vtotal - mode->vsync_end;

	if (crtc)
		timings->bits_per_pixel = crtc->fb->bits_per_pixel;
	else
		timings->bits_per_pixel = 0;

	timings->dt.flags = 0;

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		timings->dt.flags |= DISPLAY_FLAGS_INTERLACED;

	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		timings->dt.flags |= DISPLAY_FLAGS_HSYNC_HIGH;
	else
		timings->dt.flags |= DISPLAY_FLAGS_HSYNC_LOW;

	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		timings->dt.flags |= DISPLAY_FLAGS_VSYNC_HIGH;
	else
		timings->dt.flags |= DISPLAY_FLAGS_VSYNC_LOW;
}

static enum drm_connector_status fdb_connector_detect(
		struct drm_connector *connector, bool force)
{
	struct fdb_connector *fdb_connector =
					to_fdb_connector(connector);
	struct f_fdb_child *fdb = fdb_connector->fdb;
	enum drm_connector_status ret = connector_status_unknown;

	pr_debug("[Vivien] in %s\n", __func__);
	if (!fdb_connector->fdb->ops->detect)
		goto bail;

	if (fdb_connector->fdb->ops->detect(fdb))
		ret = connector_status_connected;
	else
		ret = connector_status_disconnected;
	pr_debug("[Vivien] find fdb->ops->detect, ret=%d \n", ret);
bail:
	/* dev_info(fdb_connector->fdb->dev, "%d (force=%d)", ret, force); */

	return ret;
}

static void fdb_connector_destroy(struct drm_connector *connector)
{
	struct fdb_connector *fdb_connector = to_fdb_connector(connector);
/*	struct f_fdb_child *fdb = fdb_connector->fdb; */

	dev_err(fdb_connector->fdb->dev, "\n");
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
	kfree(fdb_connector);

/*	fdb_dss_put_device(fdb); */
}

#define MAX_EDID  512

static int fdb_connector_get_modes(struct drm_connector *connector)
{
	struct fdb_connector *fdb_connector = to_fdb_connector(connector);
	struct f_fdb_child *fdb = fdb_connector->fdb;
/*	struct fdb_drm_private *priv = connector->dev->dev_private; */
	struct drm_device *dev = connector->dev;
	struct fdb_video_timings timings;
	struct drm_display_mode *mode;
	int n = 0;
	void *edid;

	memset(&timings, 0, sizeof(timings));

	pr_debug("[Vivien] in %s \n", __func__);
	/* if EDID possible, try to use it */
	if (fdb_connector->fdb->ops->read_edid) {
		edid = kzalloc(MAX_EDID, GFP_KERNEL);

		if ((fdb_connector->fdb->ops->read_edid(
					fdb, edid, MAX_EDID) > 0) &&
						      drm_edid_is_valid(edid)) {
			if (fdb->ops->set_hdmi_dvi_mode) {
				/* HDMI mode */
				if (drm_detect_hdmi_monitor(edid))
					fdb->ops->set_hdmi_dvi_mode(fdb, 1);
				else /* DVI mode */
					fdb->ops->set_hdmi_dvi_mode(fdb, 0);
			}

			drm_mode_connector_update_edid_property(
					connector, edid);
			n = drm_add_edid_modes(connector, edid);
		} else
			drm_mode_connector_update_edid_property(
					connector, NULL);

		kfree(edid);

		return n;
	}

	/* otherwise use our fixed timings */
	mode = drm_mode_create(dev);

	if (fdb_connector->fdb && fdb_connector->fdb->ops->get_timings)
		fdb_connector->fdb->ops->get_timings(fdb, &timings);
	copy_timings_fdb_to_drm(mode, &timings);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	return 1;
}

static int fdb_connector_mode_valid(struct drm_connector *connector,
				 struct drm_display_mode *mode)
{
	struct fdb_connector *fdb_connector =
					to_fdb_connector(connector);
	struct f_fdb_child *fdb = fdb_connector->fdb;
	struct fdb_video_timings timings;
	struct drm_display_mode *new_mode;
	int ret = MODE_BAD;
	struct drm_crtc *crtc = NULL;

	memset(&timings, 0, sizeof(timings));

	if (connector->encoder)
		crtc = connector->encoder->crtc;

	copy_timings_drm_to_fdb(crtc, &timings, mode);
	mode->vrefresh = drm_mode_vrefresh(mode);

	if (!fdb_connector->fdb)
		return 0;
	if (!fdb_connector->fdb->ops)
		return 0;
	if (!fdb_connector->fdb->ops->check_timings)
		return 0;

	if (!fdb_connector->fdb->ops->check_timings(fdb, &timings)) {
		/* check if vrefresh is still valid */
		new_mode = drm_mode_duplicate(connector->dev, mode);
		new_mode->clock = timings.dt.pixelclock.typ;
		new_mode->vrefresh = 0;
		if (mode->vrefresh == drm_mode_vrefresh(new_mode))
			ret = MODE_OK;
		drm_mode_destroy(connector->dev, new_mode);
	}

	dev_dbg(fdb->dev,
	"connector: mode %s: %d:\"%s\" %d %d %d %d %d %d %d %d %d %d 0x%x 0x%x\n",
			(ret == MODE_OK) ? "valid" : "invalid",
			mode->base.id, mode->name, mode->vrefresh, mode->clock,
			mode->hdisplay, mode->hsync_start,
			mode->hsync_end, mode->htotal,
			mode->vdisplay, mode->vsync_start,
			mode->vsync_end, mode->vtotal, mode->type, mode->flags);

	return ret;
}

struct drm_encoder *fdb_connector_attached_encoder(
		struct drm_connector *connector)
{
	struct fdb_connector *fdb_connector = to_fdb_connector(connector);
	return fdb_connector->encoder;
}

static const struct drm_connector_funcs fdb_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = fdb_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = fdb_connector_destroy,
};

static const struct drm_connector_helper_funcs fdb_connector_helper_funcs = {
	.get_modes = fdb_connector_get_modes,
	.mode_valid = fdb_connector_mode_valid,
	.best_encoder = fdb_connector_attached_encoder
};

/* flush an area of the framebuffer (in case of manual update display that
 * is not automatically flushed)
 */
void fdb_connector_flush(struct drm_connector *connector,
		int x, int y, int w, int h)
{
	struct fdb_connector *fdb_connector =
					to_fdb_connector(connector);

	/* TODO: enable when supported in dss */
	dev_err(fdb_connector->fdb->dev, "%d,%d, %dx%d", x, y, w, h);
}

/* initialize connector */
struct drm_connector *fdb_connector_init(struct drm_device *dev,
		int connector_type, struct f_fdb_child *fdb,
		struct drm_encoder *encoder)
{
	struct drm_connector *connector = NULL;
	struct fdb_connector *fdb_connector;

	dev_info(fdb->dev, "fdb_connector_init new fdb at %p\n", fdb);
	pr_debug("[Vivien] in %s, connector_type=%d \n", __func__, connector_type);

/*	fdb_dss_get_device(fdb); */

	fdb_connector = kzalloc(sizeof(struct fdb_connector), GFP_KERNEL);
	if (!fdb_connector)
		goto fail;

	fdb_connector->fdb = fdb;
	fdb_connector->encoder = encoder;

	connector = &fdb_connector->base;

	drm_connector_init(dev, connector, &fdb_connector_funcs,
				connector_type);
	drm_connector_helper_add(connector, &fdb_connector_helper_funcs);

#if 0 /* enable when fdb supports hotplug */
	if (fdb->caps & FDB_DSS_DISPLAY_CAP_HPD)
		connector->polled = 0;
	else
#endif
		connector->polled = DRM_CONNECTOR_POLL_CONNECT |
				DRM_CONNECTOR_POLL_DISCONNECT;

	connector->interlace_allowed = 1;
	connector->doublescan_allowed = 0;

	drm_sysfs_connector_add(connector);

	return connector;

fail:
	if (connector)
		fdb_connector_destroy(connector);

	return NULL;
}
