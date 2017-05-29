/*
 * drivers/gpu/drm/omapdrm/fdb_encoder.c
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

#include "fdb-drm.h"

#include "drm_crtc.h"
#include "drm_crtc_helper.h"

#include <linux/list.h>

#include <video/fdb.h>

/*
 * encoder funcs
 */

#define to_fdb_encoder(x) container_of(x, struct fdb_encoder, base)

/* The encoder and connector both map to same dssdev.. the encoder
 * handles the 'active' parts, ie. anything the modifies the state
 * of the hw, and the connector handles the 'read-only' parts, like
 * detecting connection and reading edid.
 */
struct fdb_encoder {
	struct drm_encoder base;
	struct f_fdb_child *child;
};

static void fdb_encoder_destroy(struct drm_encoder *encoder)
{
	struct fdb_encoder *fdb_encoder = to_fdb_encoder(encoder);
	drm_encoder_cleanup(encoder);
	kfree(fdb_encoder);
}

static const struct drm_encoder_funcs fdb_encoder_funcs = {
	.destroy = fdb_encoder_destroy,
};

/*
 * The CRTC drm_crtc_helper_set_mode() doesn't really give us the right
 * order.. the easiest way to work around this for now is to make all
 * the encoder-helper's no-op's and have the fdb_crtc code take care
 * of the sequencing and call us in the right points.
 *
 * Eventually to handle connecting CRTCs to different encoders properly,
 * either the CRTC helpers need to change or we need to replace
 * drm_crtc_helper_set_mode(), but lets wait until atomic-modeset for
 * that.
 */

static void fdb_encoder_dpms(struct drm_encoder *encoder, int mode)
{
}

static bool fdb_encoder_mode_fixup(struct drm_encoder *encoder,
				  const struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void fdb_encoder_mode_set(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
}

static void fdb_encoder_prepare(struct drm_encoder *encoder)
{
}

static void fdb_encoder_commit(struct drm_encoder *encoder)
{
}

static const struct drm_encoder_helper_funcs fdb_encoder_helper_funcs = {
	.dpms = fdb_encoder_dpms,
	.mode_fixup = fdb_encoder_mode_fixup,
	.mode_set = fdb_encoder_mode_set,
	.prepare = fdb_encoder_prepare,
	.commit = fdb_encoder_commit,
};

/*
 * Instead of relying on the helpers for modeset, the fdb_crtc code
 * calls these functions in the proper sequence.
 */

int fdb_encoder_set_enabled(struct drm_encoder *encoder, bool enabled)
{
	struct drm_device *dev = encoder->dev;
	struct fdb_encoder *fdb_encoder = to_fdb_encoder(encoder);
	struct f_fdb_child *child;

	dev_dbg(dev->dev, "fdb_encoder_set_enabled %d ----->>\n", enabled);

	child = fdb_encoder->child;
	if (enabled) {
		/* connector (HDMI) enable */
		if (child->ops->enable)
			return child->ops->enable(child);
	} else {
		/* connector (HDMI) disable */
		if (child->ops->disable)
			child->ops->disable(child);
	}
	dev_dbg(dev->dev, "<<----- fdb_encoder_set_enabled %d\n", enabled);

	return 0;
}

int fdb_encoder_update(struct drm_encoder *encoder,
		struct fdb_video_timings *timings)
{
	struct drm_device *dev = encoder->dev;
	struct fdb_drm_private *priv = dev->dev_private;
	struct fdb_encoder *fdb_encoder = to_fdb_encoder(encoder);
	struct f_fdb_child *child;
	struct f_fdb_child *fdb_child_fb;

	dev_dbg(dev->dev, "fdb_encoder_update ----->>\n");

	fdb_child_fb = bound_child_from_drm_crtc(priv, encoder->crtc);
	if (!fdb_child_fb) {
		dev_err(dev->dev, "fdb_encoder_update: no bound child found\n");
		return -EINVAL;
	}

	timings->stride_px = encoder->crtc->fb->width;

	/* without this it's workable in console, but the CRTC can end up in
	 * a different mode than the encoder */
#if 1
	/* framebuffer set timings */
	dev_dbg(fdb_child_fb->dev, "fdb_encoder_update: update fb ------->>\n");
	fdb_child_fb->ops->set_timings(fdb_child_fb, timings);
	dev_dbg(fdb_child_fb->dev,
			"<<---------- fdb_encoder_update: update fb\n");
#endif
	child = fdb_encoder->child;
	dev_dbg(child->dev,
		"fdb_encoder_update: update connector ------->>\n");

	/* connector set_timings */
	if (child->ops->set_timings)
		child->ops->set_timings(child, timings);

	dev_dbg(child->dev,
			"<<--------- fdb_encoder_update: update connector\n");
	dev_dbg(dev->dev, "<<--------- fdb_encoder_update\n");

	return 0;
}

/* initialize encoder */
struct drm_encoder *fdb_encoder_init(struct drm_device *dev,
		struct f_fdb_child *child)
{
	struct drm_encoder *encoder = NULL;
	struct fdb_encoder *fdb_encoder;

	fdb_encoder = kzalloc(sizeof(*fdb_encoder), GFP_KERNEL);
	if (!fdb_encoder)
		goto fail;

	fdb_encoder->child = child;

	encoder = &fdb_encoder->base;

	drm_encoder_init(dev, encoder, &fdb_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS);
	drm_encoder_helper_add(encoder, &fdb_encoder_helper_funcs);

	return encoder;

fail:
	if (encoder)
		fdb_encoder_destroy(encoder);

	return NULL;
}
