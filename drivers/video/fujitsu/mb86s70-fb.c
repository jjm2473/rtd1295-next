/*
 * drivers/video/fbdev/fujitsu/iris-fb.c
 *
 * Copyright (C) 2013 - 2014 Linaro, Ltd for Fujitsu Semiconductor, Ltd
 * Author: Andy Green <andy.green@linaro.org>
 *
 * This always instantiates an fdb child representing the framebuffer, if the
 * "simple" DT attribute is nonzero then it also registers as a simple Linux
 * framebuffer.  If !simple, the fdb child may be adopted by, eg, fdb-drm
 * driver and exposed that way.
 */

#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/module.h>

#include <video/mb86s70-fb.h>
#include <video/fdb.h>

#define CNVT_TOHW(val, width) ((((val) << (width)) + 0x7FFF - (val)) >> 16)

/* there's one logical blitter ring shared between n IRIS instances */
struct fdb_blitter_ring blits;
struct blit_completion_ring blit_comp;

static const struct fdb_format formats[] = {
	{ DRM_FORMAT_XRGB8888, { { 4, 1 } }, false },
	{ DRM_FORMAT_RGBX8888, { { 4, 1 } }, false },
	{ DRM_FORMAT_ARGB8888, { { 4, 1 } }, false },
	{ DRM_FORMAT_RGB888, { { 3, 1 } }, false },
	{ DRM_FORMAT_RGB565, { { 2, 1 } }, false }, /* RGB16-565 */
};

static const struct fdb_dumps dumps[] = {
	{ "GlobalCtrl", 4, 4 },
	{ "GlobalCtrl", 0x10, 0 },
	{ "GlobalCtrl", 0x1c, 0 },
	{ "Pixelbus", 0x800, 0x38 },
	{ "Pixelbus", 0x840, 4 },
	{ "Pixelbus", 0x84c, 0 },
	{ "Fetch0", 0xc00, 0x40 },
	{ "Fetch0", 0xc50, 8 },
	{ "Fetch1", 0x1000, 0x40 },
	{ "Fetch1", 0x104c, 8 },
	{ "Fetch2", 0x1400, 0x4c },
	{ "Fetch2", 0x145c, 8 },
	{ "Store0", 0x1800, 0x1c },
	{ "Store0", 0x1828, 0xc },
	{ "ROP0", 0x1c00, 0x10 },
	{ "BitBlend", 0x2000, 0x28 },
	{ "CLUT0", 0x2400, 0xc },
	{ "Matrix0", 0x2c00, 0x1c },
	{ "Hscaler", 0x3000, 0x8 },
	{ "Vscaler", 0x3400, 0x8 },
	{ "Fetch4", 0x3800, 0x40 },
	{ "Fetch4", 0x3c4c, 8 },
	{ "Fetch3", 0x3c00, 0x3c },
	{ "Fetch3", 0x3c60, 4 },
	{ "ExtDst0", 0x4000, 0 },
	{ "ExtDst0", 0x4008, 0x10 },
	{ "Layerblend0", 0x4400, 0xc },
	{ "Layerblend1", 0x4800, 0xc },
	{ "DispCfg", 0x5000, 0x8},
	{ "Framegen", 0x5404, 0x40},
	{ "Matrix1", 0x5800, 0x1c },
	{ "CLUT1", 0x5c00, 0xc },
	{ "Dither0", 0x6400, 0x8 },
	{ "Sig0", 0x6804, 0x40 },
	{ "Sig0", 0x6850, 0x1c },

};
#ifdef DEBUG
static void
iris_dump_regs(struct f_iris_par *par)
{
	void __iomem *base = par->base;
	int n, i;

	dev_dbg(par->dev, "====================iris_dump_regs %p\n", base);

	for (i = 0; i < ARRAY_SIZE(dumps); i++)
		for (n = 0; n <= dumps[i].len; n += 4)
			dev_dbg(par->dev, "%s: +0x%04x: 0x%08X\n",
				dumps[i].name, dumps[i].start + n,
					__raw_readl(base + dumps[i].start + n));
}
#else
#define iris_dump_reg(par)s
#endif

/*
 * This performs the necessary timings-related updates in NON-shadowed regs
 */

static void
_f_iris_fb_dynamic_reconfig(struct f_iris_par *par,
			    struct fdb_video_timings *timings)
{
	void __iomem *base = par->base;
	struct display_timing *dt = &timings->dt;

	writel(/* DIRECT */
	       fdb_const(PXENG_DISPCFG_POLARITY_CTRL, POLHS,
			 !!(timings->dt.flags & DISPLAY_FLAGS_HSYNC_HIGH)) |
	       fdb_const(PXENG_DISPCFG_POLARITY_CTRL, POLVS,
			 !!(timings->dt.flags & DISPLAY_FLAGS_VSYNC_HIGH)) |
	       fdb_const(PXENG_DISPCFG_POLARITY_CTRL, POLEN, 1),
	       base + PXENG_DISPCFG_POLARITY_CTRL_OFS);

	writel(/* DIRECT */
	       fdb_const(DISPENG_FRAMEGEN0_PKICKCONFIG, PKICKEN, 1) |
	       fdb_const(DISPENG_FRAMEGEN0_PKICKCONFIG,
			 PKICKROW, dt->vactive.typ) |
	       fdb_const(DISPENG_FRAMEGEN0_PKICKCONFIG,
			 PKICKINT0EN, 1) |
	       fdb_const(DISPENG_FRAMEGEN0_PKICKCONFIG, PKICKCOL,
			 dt->hactive.typ + par->h_pixels_fb_offset),
	       base + DISPENG_FRAMEGEN0_PKICKCONFIG_OFS);

	/* framegenerator setup */

	writel(/* DIRECT */
	       fdb_const(DISPENG_FRAMEGEN0_HTCFG1, HTOTAL,
			 dt->hactive.typ + par->h_pixels_fb_offset +
			 dt->hfront_porch.typ +
			 dt->hsync_len.typ + dt->hback_porch.typ - 1) |
	       fdb_const(DISPENG_FRAMEGEN0_HTCFG1, HACT,
			 dt->hactive.typ + par->h_pixels_fb_offset),
	       base + DISPENG_FRAMEGEN0_HTCFG1_OFS);
	writel(/* DIRECT */
	       fdb_const(DISPENG_FRAMEGEN0_HTCFG2, HSEN, 1) |
	       fdb_const(DISPENG_FRAMEGEN0_HTCFG2, HSBP,
			 dt->hsync_len.typ + dt->hback_porch.typ - 1) |
	       fdb_const(DISPENG_FRAMEGEN0_HTCFG2, HSYNC,
			 dt->hsync_len.typ - 1),
	       base + DISPENG_FRAMEGEN0_HTCFG2_OFS);

	writel(/* DIRECT */
	       fdb_const(DISPENG_FRAMEGEN0_VTCFG1, VTOTAL,
			 dt->vactive.typ + dt->vfront_porch.typ +
			 dt->vsync_len.typ + dt->vback_porch.typ - 1) |
	       fdb_const(DISPENG_FRAMEGEN0_VTCFG1, VACT, dt->vactive.typ),
	       base + DISPENG_FRAMEGEN0_VTCFG1_OFS);
	/* vsync */
	writel(/* DIRECT */
	       fdb_const(DISPENG_FRAMEGEN0_VTCFG2, VSEN, 1) |
	       fdb_const(DISPENG_FRAMEGEN0_VTCFG2, VSBP,
			 dt->vsync_len.typ + dt->vback_porch.typ - 1) |
	       fdb_const(DISPENG_FRAMEGEN0_VTCFG2, VSYNC,
			 dt->vsync_len.typ - 1),
	       base + DISPENG_FRAMEGEN0_VTCFG2_OFS);
}

static void
_f_iris_fb_set_timings(struct f_iris_par *par,
		       struct fdb_video_timings *timings)
{
	void __iomem *base = par->base;
	struct display_timing *dt  = &timings->dt;
	int bytes_pp = (timings->bits_per_pixel + 7) / 8;
	int n;
	int bufs;
	int buf_pool;

	par->h_bytes_fb_offset = par->h_pixels_fb_offset * bytes_pp;
	par->last_timings = *timings;

	dev_info(par->dev, "%s: (%d x %d, flags=0x%x)\n",
		__func__, timings->dt.hactive.typ, timings->dt.vactive.typ,
		timings->dt.flags);

	/* do the SHADOWED portion of the new mode change */

	writel(/* SHADOWED */
	       fdb_const(PXE_F3_SOURCEBUFFERSTRIDE, STRIDE,
			 timings->stride_px * bytes_pp - 1),
	       base + PXE_F3_SOURCEBUFFERSTRIDE_OFS);

	writel(/* SHADOWED */
	       fdb_const(PXE_F3_SOURCEBUFFERATTRIBUTES, LINECOUNT,
			 dt->vactive.typ - 1) |
	       fdb_const(PXE_F3_SOURCEBUFFERATTRIBUTES, LINEWIDTH,
			 dt->hactive.typ + par->h_pixels_fb_offset - 1),
	       base + PXE_F3_SOURCEBUFFERATTRIBUTES_OFS);

	writel(/* SHADOWED */
	       fdb_const(PXE_F3_FRAMEDIMENSIONS,
			 FRAMEHEIGHT, dt->vactive.typ - 1) |
	       fdb_const(PXE_F3_FRAMEDIMENSIONS, FRAMEWIDTH,
			 dt->hactive.typ + par->h_pixels_fb_offset - 1),
	       base + PXE_F3_FRAMEDIMENSIONS_OFS);

	/* if no pixel layout info is coming, assume ARGB 32 */
	if (!timings->red_length) {
		timings->red_length = 8;
		timings->green_length = 8;
		timings->blue_length = 8;
		timings->alpha_length = 0;

		timings->red_offset = 16;
		timings->green_offset = 8;
		timings->blue_offset = 0;
		timings->alpha_offset = 24;
	}

	dev_dbg(par->dev, "shifts: %d %d %d %d %d %d %d %d\n",
		timings->red_length, timings->green_length,
		timings->blue_length, timings->alpha_length,
		timings->red_offset, timings->green_offset,
		timings->blue_offset, timings->alpha_offset);
	/* component size */
	writel(/* SHADOWED */
	       fdb_const(PXE_F3_COLCOMPBITS,
			 COMPONENTBITSRED, timings->red_length) |
	       fdb_const(PXE_F3_COLCOMPBITS,
			 COMPONENTBITSGREEN, timings->green_length) |
	       fdb_const(PXE_F3_COLCOMPBITS,
			 COMPONENTBITSBLUE, timings->blue_length) |
	       fdb_const(PXE_F3_COLCOMPBITS,
			 COMPONENTBITSALPHA, timings->alpha_length),
	       base + PXE_F3_COLCOMPBITS_OFS);
	/* component shift */
	writel(/* SHADOWED */
	       fdb_const(PXE_F3_COLCOMPSHIFT,
			 COMPONENTSHIFTRED, timings->red_offset) |
	       fdb_const(PXE_F3_COLCOMPSHIFT,
			 COMPONENTSHIFTGREEN, timings->green_offset) |
	       fdb_const(PXE_F3_COLCOMPSHIFT,
			 COMPONENTSHIFTBLUE, timings->blue_offset) |
	       fdb_const(PXE_F3_COLCOMPSHIFT,
			 COMPONENTSHIFTALPHA, timings->alpha_offset),
	       base + PXE_F3_COLCOMPSHIFT_OFS);
	/* control */
	writel(/* SHADOWED */
	       fdb_const(PXE_F3_CTRL, BITSPERPIXEL, timings->bits_per_pixel),
	       base + PXE_F3_CTRL_OFS);

	/*
	 * burst buffers: default to 4 bufs len 8, max it out for higher res
	 * we can choose how to cut up the buffer ram but can't exceed reported
	 * buffer count or max depth at max buffer count.
	 * Also AXI burst limit is 16.
	 * Rotation affects the requirement so do that before setting burst
	 */

	buf_pool = par->max_burst_buffers * par->max_burst_len_at_max_buffers;
	n = 8;
	bufs = 4;
	if (dt->hactive.typ * bytes_pp >= (720 * 4)) {
#if 1
		n = 16; /* max AXI limit */
		bufs = buf_pool / n;
#else
		n = par->max_burst_len_at_max_buffers;
		bufs = par->max_burst_buffers;
#endif
	}

	/*
	 * do rotation and let that force burst buffers too
	 */

	dev_dbg(par->dev, "scanout_rotation: %d\n", timings->scanout_rotation);

	switch (timings->scanout_rotation) {
	case 90:
		/* swap */
		writel(fdb_const(PXE_F3_FRAMEXOFFSET, FRAMEXOFFSET, 0),
		       base + PXE_F3_FRAMEXOFFSET_OFS);
		writel(fdb_const(PXE_F3_FRAMEYOFFSET, FRAMEYOFFSET, 0),
		       base + PXE_F3_FRAMEYOFFSET_OFS);
		writel(fdb_const(PXE_F3_DELTAXX, DELTAXX, 0),
		       base + PXE_F3_DELTAXX_OFS);
		writel(fdb_const(PXE_F3_DELTAXY, DELTAXY, 1 << 18),
		       base + PXE_F3_DELTAXY_OFS);
		writel(fdb_const(PXE_F3_DELTAYX, DELTAYX, 1 << 18),
		       base + PXE_F3_DELTAYX_OFS);
		writel(fdb_const(PXE_F3_DELTAYY, DELTAYY, 0),
		       base + PXE_F3_DELTAYY_OFS);
		bufs = par->max_burst_buffers;
		n = 1;
		break;

	case 180:
		/* for horizontal flip */
		writel(fdb_const(PXE_F3_FRAMEXOFFSET, FRAMEXOFFSET,
				 dt->hactive.typ),
		       base + PXE_F3_FRAMEXOFFSET_OFS);
		writel(fdb_const(PXE_F3_DELTAXX, DELTAXX, 0x7f << 18),
		       base + PXE_F3_DELTAXX_OFS);
		writel(fdb_const(PXE_F3_DELTAXY, DELTAXY, 0),
		       base + PXE_F3_DELTAXY_OFS);

		/* for vertical flip */
		writel(fdb_const(PXE_F3_FRAMEYOFFSET, FRAMEYOFFSET,
				 dt->vactive.typ),
		       base + PXE_F3_FRAMEYOFFSET_OFS);
		writel(fdb_const(PXE_F3_DELTAYX, DELTAYX, 0),
		       base + PXE_F3_DELTAYX_OFS);
		writel(fdb_const(PXE_F3_DELTAYY, DELTAYY, 0x7f << 18),
		       base + PXE_F3_DELTAYY_OFS);
		break;

	case 270:
		/* swap */
		writel(fdb_const(PXE_F3_FRAMEXOFFSET, FRAMEXOFFSET, 0),
		       base + PXE_F3_FRAMEXOFFSET_OFS);
		writel(fdb_const(PXE_F3_FRAMEYOFFSET, FRAMEYOFFSET, 0),
		       base + PXE_F3_FRAMEYOFFSET_OFS);
		writel(fdb_const(PXE_F3_DELTAXX, DELTAXX, 0),
		       base + PXE_F3_DELTAXX_OFS);
		writel(fdb_const(PXE_F3_DELTAXY, DELTAXY, 1 << 18),
		       base + PXE_F3_DELTAXY_OFS);
		writel(fdb_const(PXE_F3_DELTAYX, DELTAYX, 1 << 18),
		       base + PXE_F3_DELTAYX_OFS);
		writel(fdb_const(PXE_F3_DELTAYY, DELTAYY, 0),
		       base + PXE_F3_DELTAYY_OFS);

		writel(fdb_const(PXE_F3_FRAMEYOFFSET, FRAMEYOFFSET,
				 dt->hactive.typ),
		       base + PXE_F3_FRAMEYOFFSET_OFS);
		writel(fdb_const(PXE_F3_DELTAXY, DELTAXY, 0x7f << 18),
		       base + PXE_F3_DELTAXY_OFS);
		writel(fdb_const(PXE_F3_DELTAXX, DELTAXX, 0),
		       base + PXE_F3_DELTAXX_OFS);

		bufs = par->max_burst_buffers;
		n = 1;
		break;

	default: /* normal */
		writel(fdb_const(PXE_F3_FRAMEXOFFSET, FRAMEXOFFSET, 0),
		       base + PXE_F3_FRAMEXOFFSET_OFS);
		writel(fdb_const(PXE_F3_FRAMEYOFFSET, FRAMEYOFFSET, 0),
		       base + PXE_F3_FRAMEYOFFSET_OFS);
		writel(fdb_const(PXE_F3_DELTAXX, DELTAXX, 1 << 18),
		       base + PXE_F3_DELTAXX_OFS);
		writel(fdb_const(PXE_F3_DELTAXY, DELTAXY, 0),
		       base + PXE_F3_DELTAXY_OFS);
		writel(fdb_const(PXE_F3_DELTAYX, DELTAYX, 0),
		       base + PXE_F3_DELTAYX_OFS);
		writel(fdb_const(PXE_F3_DELTAYY, DELTAYY, 1 << 18),
		       base + PXE_F3_DELTAYY_OFS);
		break;
	}

	/* sanity-check and set burst buffers */

	if (bufs > par->max_burst_buffers) /* IP limit */
		bufs = par->max_burst_buffers;

	if (n > 16) /* axi limit */
		n = 16;

	writel(/* SHADOWED */
		(n << 8) | bufs, base + PXE_F3_BRSTBUFMGT_OFS);

	dev_dbg(par->dev, "Burst buffers: %d buffers, depth %d\n", bufs, n);

	/* first time gets a free pass */
	if (!par->subsequent) {
		_f_iris_fb_dynamic_reconfig(par, timings);

		clk_set_rate(par->clocks[0], dt->pixelclock.typ * 1000);

		dev_dbg(par->dev, "%s: %d x %d, rate = %u (current = %lu)\n",
			__func__, dt->hactive.typ, dt->vactive.typ,
		      dt->pixelclock.typ * 1000, clk_get_rate(par->clocks[0]));
	}

	/*
	 *  Static Setup #6: generate shadow token for all streams
	 *                  (ShdTokGen <- 1 on fgen and extdst0)
	 */

	init_completion(&par->shadow_display_completion);

	/* must be together - without this == WSOD */
	writel(fdb_const(DISPENG_FRAMEGEN0_FGSLR, SHDTOKGEN, 1),
	       base + DISPENG_FRAMEGEN0_FGSLR_OFS);

	writel(fdb_const(PXE_PXB_SYNC_TRIGGER, EXTDST0_SYNC_TRIGGER, 1) |
	       fdb_const(PXE_PXB_SYNC_TRIGGER, STORE0_SYNC_TRIGGER, 0),
	       base + PXE_PXB_SYNC_TRIGGER_OFS);
	writel(fdb_const(DISPENG_FRAMEGEN0_FGENABLE, FGEN, 1),
	       base + DISPENG_FRAMEGEN0_FGENABLE_OFS);

	if (par->subsequent) {
		writel(/* DIRECT */
		       fdb_const(DISPENG_FRAMEGEN0_PKICKCONFIG, PKICKEN, 1) |
		       fdb_const(DISPENG_FRAMEGEN0_PKICKCONFIG, PKICKROW,
				 dt->vactive.typ) |
		       fdb_const(DISPENG_FRAMEGEN0_PKICKCONFIG,
				 PKICKINT0EN, 1) |
		       fdb_const(DISPENG_FRAMEGEN0_PKICKCONFIG, PKICKCOL,
				 dt->hactive.typ + par->h_pixels_fb_offset),
		       base + DISPENG_FRAMEGEN0_PKICKCONFIG_OFS);

		dev_dbg(par->dev, "%s: starting sync...\n", __func__);

		/*
		 * if it's not the first time, we have to wait for shadow
		 * load to go down... the ISR will do the dynamic part too
		 */
		n = wait_for_completion_timeout(
			&par->shadow_display_completion, msecs_to_jiffies(75));
		if (n <= 0)
			dev_info(par->dev, "%s: shadow sync timeout\n",
				 __func__);
		else {
			clk_set_rate(par->clocks[0],
				     dt->pixelclock.typ * 1000);
			_f_iris_fb_dynamic_reconfig(par, &par->last_timings);
			dev_dbg(par->dev,
				"%s: %d x %d, rate = %u (current = %lu)\n",
				__func__, dt->hactive.typ, dt->vactive.typ,
				dt->pixelclock.typ * 1000,
				clk_get_rate(par->clocks[0]));
		}
		dev_dbg(par->dev, "%s: sync completed...\n", __func__);
	}
	par->subsequent = true;

#ifdef DEBUG
	iris_dump_regs(par);
#endif

	dev_dbg(par->dev, "Done in %s\n", __func__);
}


static void
f_iris_fb_set_timings(struct f_fdb_child *fdb_child,
		      struct fdb_video_timings *timings)
{
	struct f_iris_par *par = f_par_from_fdb_child(fdb_child);

	mutex_lock(&par->lock);

	if (!memcmp(timings, &par->last_timings, sizeof(*timings)))
		goto bail;

	pm_runtime_get_sync(par->dev);

	if (__raw_readl(par->base + DISPENG_FRAMEGEN0_FGENABLE_OFS) &
			fdb_const(DISPENG_FRAMEGEN0_FGENABLE, FGEN, 1)) {
		init_completion(&par->dispeng_seq_completion);

		writel(fdb_const(DISPENG_FRAMEGEN0_FGENABLE, FGEN, 0),
		       par->base + DISPENG_FRAMEGEN0_FGENABLE_OFS);

		if (wait_for_completion_timeout(&par->dispeng_seq_completion,
						msecs_to_jiffies(60)) <= 0)
			dev_info(par->dev, "%s: dispeng stop timeout\n",
				 __func__);
	}

	_f_iris_fb_set_timings(par, timings);
	pm_runtime_mark_last_busy(par->dev);
	pm_runtime_put_autosuspend(par->dev);
bail:
	mutex_unlock(&par->lock);
}

static int
f_iris_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info);
static int __iris_wait_for_vsync(struct f_iris_par *par, int count);

static int
_f_iris_fb_hw_set_var(void __iomem *crtc,
		      struct fb_var_screeninfo *var, struct f_iris_par *par)
{
	struct fdb_video_timings timings;

	fdb_var_to_video_timings(var, &timings);

	if (timings.xres_virtual == par->last_timings.xres_virtual &&
	    timings.yres_virtual == par->last_timings.yres_virtual &&
	    timings.bits_per_pixel == par->last_timings.bits_per_pixel) {
		struct fb_info info;

		info.par = par;
		//f_iris_fb_pan_display(var, &info);

		return 0;
	}

	pm_runtime_get_sync(par->dev);
	_f_iris_fb_set_timings(par, &timings);
	pm_runtime_mark_last_busy(par->dev);
	pm_runtime_put_autosuspend(par->dev);

	return 0;
}

static void
_f_iris_fb_fdb_set_paddr(struct f_fdb_child *fdb_child, dma_addr_t pa)
{
	struct f_iris_par *par = f_par_from_fdb_child(fdb_child);
	void __iomem *crtc = par->base;

	writel(/* SHADOWED */
		pa - par->h_bytes_fb_offset,
			crtc + PXE_F3_BASEADDRESS_OFS);

	/* start the synchronizer */

	writel(/* DIRECT */
		fdb_const(PXE_PXB_SYNC_TRIGGER, EXTDST0_SYNC_TRIGGER, 1) |
		fdb_const(PXE_PXB_SYNC_TRIGGER, STORE0_SYNC_TRIGGER, 0),
				      par->base + PXE_PXB_SYNC_TRIGGER_OFS);

	par->fbpaddr = pa;
}

static void
f_iris_fb_fdb_set_paddr(struct f_fdb_child *fdb_child, dma_addr_t pa)
{
	struct f_iris_par *par = f_par_from_fdb_child(fdb_child);

	mutex_lock(&par->lock);
	_f_iris_fb_fdb_set_paddr(fdb_child, pa);
	mutex_unlock(&par->lock);
}

static int
f_iris_fb_fdb_set_paddr_ref_cb(
		struct f_fdb_child *fdb_child, dma_addr_t pa, void (*ref_cb)(void *), void *arg)
{
	struct f_iris_par *par = f_par_from_fdb_child(fdb_child);
	unsigned long flags;
	struct list_o_callback *list_cb = NULL;

	/*
	 * Iris Fetch#3 BaseAddres register is SHADOWED, so old framebuffer is used
	 * until shadow load irq is raised.
	 * We must hold framebuffer until shadow load irq is raised.
	 */

	if (ref_cb) {
		list_cb = kmalloc(sizeof(*list_cb), GFP_KERNEL);
		if (list_cb) {
			INIT_LIST_HEAD(&list_cb->list);
			list_cb->cb = ref_cb;
			list_cb->arg = arg;
			list_cb->retry = false;
		}
		else {
			dev_err(fdb_child->dev, "%s: memory alloc failed\n", __func__);
			return -ENOMEM;
		}
	}

	mutex_lock(&par->lock);
	spin_lock_irqsave(&par->spinlock, flags);

	_f_iris_fb_fdb_set_paddr(fdb_child, pa);

	if (list_cb) {
		list_cb->load_time = ktime_get();
		list_add_tail(&list_cb->list, &par->list_o_callback);
	}

	spin_unlock_irqrestore(&par->spinlock, flags);
	mutex_unlock(&par->lock);

	return 0;
}

static int
f_iris_fb_fdb_enable(struct f_fdb_child *fdb_child)
{
	struct f_iris_par *par = f_par_from_fdb_child(fdb_child);

	pm_runtime_get_sync(par->dev);

	mutex_lock(&par->lock);

	writel(fdb_const(DISPENG_FRAMEGEN0_FGENABLE, FGEN, 1),
	       par->base + DISPENG_FRAMEGEN0_FGENABLE_OFS);

	if (par->simple && fdb_child->reverse_binding)
		fdb_child->reverse_binding->ops->enable(fdb_child->reverse_binding);
	
	mutex_unlock(&par->lock);
	dev_info(par->dev, "%s ------ +++++\n", __func__);

	return 0;
}

static void
f_iris_fb_fdb_disable(struct f_fdb_child *fdb_child)
{
	struct f_iris_par *par = f_par_from_fdb_child(fdb_child);

	if (par->simple && fdb_child->reverse_binding)
		fdb_child->reverse_binding->ops->disable(fdb_child->reverse_binding);
	
	if (__raw_readl(par->base + DISPENG_FRAMEGEN0_FGENABLE_OFS) &
	    fdb_const(DISPENG_FRAMEGEN0_FGENABLE, FGEN, 1)) {
		init_completion(&par->dispeng_seq_completion);
		mutex_lock(&par->lock);
		writel(fdb_const(DISPENG_FRAMEGEN0_FGENABLE, FGEN, 0),
		       par->base + DISPENG_FRAMEGEN0_FGENABLE_OFS);

		mutex_unlock(&par->lock);

		if (wait_for_completion_timeout(&par->dispeng_seq_completion,
						msecs_to_jiffies(60)) <= 0)
			dev_info(par->dev, "%s: dispeng stop timeout\n",
				 __func__);
	}

	pm_runtime_mark_last_busy(par->dev);
	pm_runtime_put_autosuspend(par->dev);

	dev_info(par->dev, "%s ------ -------\n", __func__);
}


static int
f_iris_fb_setcolreg(unsigned regno, unsigned red, unsigned green,
		    unsigned blue, unsigned transp, struct fb_info *info)
{
	if (regno >= 16)
		return -EINVAL;

	if (info->var.grayscale)
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;

	red = CNVT_TOHW(red, info->var.red.length);
	green = CNVT_TOHW(green, info->var.green.length);
	blue = CNVT_TOHW(blue, info->var.blue.length);
	transp = CNVT_TOHW(transp, info->var.transp.length);

	((u32 *)(info->pseudo_palette))[regno] =
			(red << info->var.red.offset) |
			(green << info->var.green.offset) |
			(blue << info->var.blue.offset) |
					(transp << info->var.transp.offset);

	return 0;
}

/* only called for simple fb */

static int
f_iris_allocate_backing_set_var(struct fb_info *info, const char *mode)
{
	struct f_iris_par *par = info->par;
	int bytes_per_pixel;
	int ret = 0;

	if (!par->simple) {
		dev_err(info->dev, "%s: only for simple\n", __func__);
		return -EINVAL;
	}

	pm_runtime_get_sync(par->dev);
	mutex_lock(&par->lock);

	if (par->fb_va) {
		dma_free_writecombine(info->dev, par->framesize,
				      par->fb_va, par->fbpaddr);
		par->fb_va = NULL;
	}

	if (mode) {
		ret = fdb_get_of_var(mode, &info->var);
		if (ret) {
			dev_err(info->dev, "Failed to get of var %s\n", mode);
			ret = -ENOMEM;
			goto bail;
		}
	}

	bytes_per_pixel = (info->var.bits_per_pixel + 7) / 8;
	par->h_bytes_fb_offset = par->h_pixels_fb_offset * bytes_per_pixel;
	info->fix.line_length = info->var.xres * bytes_per_pixel;
	par->framesize = info->var.yres * info->fix.line_length * 2;
	info->var.yres_virtual = 2 * info->var.yres;
	
	par->fb_va = (char __force __iomem *)
		     dma_alloc_writecombine(info->dev, par->framesize,
					    &par->fbpaddr, GFP_KERNEL);

	if (!par->fb_va) {
		dev_err(info->dev, "Unable to allocate backing store\n");
		ret = -ENOMEM;
		goto bail;
	}
	info->screen_base = (void *)par->fb_va;
	dev_info(info->dev, "fb mem at va 0x%lX pa 0x%llX\n",
		 (unsigned long)info->screen_base, (u64)par->fbpaddr);
	info->fix.smem_start = par->fbpaddr;
	info->fix.smem_len = par->framesize;
	info->pseudo_palette = par->pseudo_palette;
	info->flags = FBINFO_DEFAULT;

	_f_iris_fb_fdb_set_paddr(&par->fdb_child, par->fbpaddr);
	_f_iris_fb_hw_set_var(par->base, &info->var, par);

bail:
	mutex_unlock(&par->lock);
	pm_runtime_mark_last_busy(par->dev);
	pm_runtime_put_autosuspend(par->dev);

	return ret;
}

static int
f_iris_fb_blank(int blank_mode, struct fb_info *info)
{
	struct f_iris_par *par = info->par;

	dev_dbg(info->dev, "f_iris_fb_blank: %d\n", blank_mode);

	if (blank_mode == FB_BLANK_UNBLANK)
		par->fdb_child.ops->enable(&par->fdb_child);
	else
		par->fdb_child.ops->disable(&par->fdb_child);

	return 0;
}

void
f_iris_fb_get_timings(struct f_fdb_child *fdb_child,
		      struct fdb_video_timings *timings)
{
	struct f_iris_par *par = f_par_from_fdb_child(fdb_child);
	struct fb_var_screeninfo var;
	const char *mode;
	int ret;

	mutex_lock(&par->lock);

	if (of_property_read_string(fdb_child->dev->of_node, "mode", &mode)) {
		dev_err(fdb_child->dev, "Missing mode\n");
		goto bail;
	}

	dev_dbg(fdb_child->dev, "fdb_fb_get_timings: mode = %s\n", mode);

	ret = fdb_get_of_var(mode, &var);
	if (ret) {
		dev_err(fdb_child->dev, "Failed to get of var for %s\n", mode);
		goto bail;
	}

	fdb_var_to_video_timings(&var, timings);
	dev_dbg(fdb_child->dev, "f_iris_fb_get_timings result ->\n");
	fdb_dump_video_timings(timings);

bail:
	mutex_unlock(&par->lock);
}

static void
f_iris_fb_set_gamma(struct f_fdb_child *fdb_child,
		    u16 red, u16 green, u16 blue, int regno)
{
}
static void
f_iris_fb_get_gamma(struct f_fdb_child *fdb_child,
		    u16 *red, u16 *green, u16 *blue, int regno)
{
}

static int
f_iris_fb_check_timings(struct f_fdb_child *fdb_child,
			struct fdb_video_timings *timings)
{
	return 0;
}

static const struct fdb_format *
f_iris_fb_get_fdb_formats(struct f_fdb_child *fdb_child, int *count)
{
	*count = ARRAY_SIZE(formats);
	return formats;
}

int
f_iris_fb_bind_to_source(struct f_fdb_child *fdb_child,
			 struct f_fdb_child *fdb_bound)
{
	struct f_iris_par *par = f_par_from_fdb_child(fdb_child);

	par->source = fdb_bound;

	return 0;
}

static int
f_iris_display_rotate(struct f_fdb_child *fdb_child, struct fdb_blit *blit)
{
	struct f_iris_par *par = f_par_from_fdb_child(fdb_child);

	dev_dbg(par->dev, "f_iris_display_rotate\n");

	switch (blit->rotate) {
	case 180:

		break;
	default:
		break;
	}

	return 0;
}

#if defined(CONFIG_DEBUG_FS)
static int
f_iris_fdb_debugfs(struct f_fdb_child *fdb_child, struct seq_file *m, int type)
{
	struct f_iris_par *par = f_par_from_fdb_child(fdb_child);
	struct f_iris_par *bpar;
	struct list_head *p;
	int i, n;

	switch (type) {
	case FDB_DEBUGFS_INFO:
		list_for_each(p, &par->blits->blitter_list) {
			bpar = container_of(p, struct f_iris_par, blitter_list);
			if (bpar == par)
				f_iris_blitter_debugfs(m, bpar);
		}
		break;
	case FDB_DEBUGFS_DUMP:
		pm_runtime_get_sync(par->dev);
		seq_printf(m, "struct dump dump_iris_0x%lx[] = {\n",
			   (unsigned long)par->base_pa);
		for (i = 0; i < ARRAY_SIZE(dumps); i++)
			for (n = 0; n <= dumps[i].len; n += 4)
				seq_printf(m,
					   "/* %s: +0x%04x */ "
					   "{ 0x%lx, 0x%08X },\n",
					   dumps[i].name,
					   dumps[i].start + n,
					   (unsigned long)par->base_pa +
					   dumps[i].start + n,
					   __raw_readl(par->base +
						       dumps[i].start + n));
		seq_puts(m, "};\n");
		pm_runtime_mark_last_busy(par->dev);
		pm_runtime_put_autosuspend(par->dev);
		break;
	}
	return 0;
}
#endif
static int __iris_wait_for_vsync(struct f_iris_par *par, int count)
{
	int ret;
	struct list_o_completions loc;
	unsigned long flags;

	init_completion(&loc.c);

	while (count--) {
		reinit_completion(&loc.c);

		spin_lock_irqsave(&par->spinlock, flags);
		INIT_LIST_HEAD(&loc.list);
		list_add(&loc.list, &par->list_o_completions);
		spin_unlock_irqrestore(&par->spinlock, flags);

		ret = wait_for_completion_timeout(&loc.c,
						  msecs_to_jiffies(50));

		spin_lock_irqsave(&par->spinlock, flags);
		list_del(&loc.list);
		spin_unlock_irqrestore(&par->spinlock, flags);

		if (ret <= 0) {
			dev_err(par->dev, "WAITFORVSYNC timeout\n");
			return -ETIME;
		}
	}

	return 0;
}

static int f_iris_fdb_wait_for_vsync(struct f_fdb_child *fdb_child, int count)
{
	struct f_iris_par *par = f_par_from_fdb_child(fdb_child);

	return __iris_wait_for_vsync(par, count);
}


struct f_fdb_ops f_iris_fb_fdb_ops = {
	.enable = f_iris_fb_fdb_enable,
	.disable = f_iris_fb_fdb_disable,
	.set_paddr = f_iris_fb_fdb_set_paddr,
	.set_paddr_ref_cb = f_iris_fb_fdb_set_paddr_ref_cb,
	.get_timings = f_iris_fb_get_timings,
	.set_timings = f_iris_fb_set_timings,
	.check_timings = f_iris_fb_check_timings,
	.get_gamma = f_iris_fb_get_gamma,
	.set_gamma = f_iris_fb_set_gamma,
	.get_fdb_formats = f_iris_fb_get_fdb_formats,
	.blit_blocking = f_iris_blit_blocking,
	.blit_blocking2 = f_iris_blit_blocking2,
	.get_blit_comp = f_iris_get_blit_comp,
	.set_rotate = f_iris_display_rotate,
	.bind_to_source = f_iris_fb_bind_to_source,
#if defined(CONFIG_DEBUG_FS)
	.fdb_debugfs = f_iris_fdb_debugfs,
#endif
	.fdb_wait_for_vsync = f_iris_fdb_wait_for_vsync,
};

static struct fb_fix_screeninfo f_iris_fb_fix = {
	.id =		"f_iris_fb",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.xpanstep =	1,
	.ypanstep =	1,
	.ywrapstep =	1,
	.accel =	FB_ACCEL_NONE,
};

static int
f_iris_fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct f_iris_par *par = info->par;

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		return __iris_wait_for_vsync(par, 1);
	}

	return -ENOTTY;
}

static int
f_iris_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct f_iris_par *par = info->par;
	int bytes_pp = (var->bits_per_pixel + 7) / 8;
	dma_addr_t paddr;

	paddr = par->fbpaddr + (var->xoffset * bytes_pp) +
				(var->yoffset * var->xres_virtual * bytes_pp);
	f_iris_fb_fdb_set_paddr(&par->fdb_child, par->fbpaddr);

	return 0;
}

static int f_iris_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct f_iris_par *par = info->par;
	
	var->red.offset= par->last_timings.red_offset;
	var->green.offset= par->last_timings.green_offset;
	var->blue.offset= par->last_timings.blue_offset;
	
	var->red.length = par->last_timings.red_length;
	var->green.length = par->last_timings.green_length;
	var->blue.length = par->last_timings.blue_length;
	
	return 0;
}

static int f_iris_set_par(struct fb_info *info)
{
	struct f_iris_par *par = info->par;

	_f_iris_fb_hw_set_var(par->base, &info->var, par);

	return 0;
}

static struct fb_ops f_iris_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= f_iris_fb_setcolreg,
	.fb_blank	= f_iris_fb_blank,
	.fb_pan_display = f_iris_fb_pan_display,
	.fb_ioctl	= f_iris_fb_ioctl,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_check_var   = f_iris_fb_check_var,
	.fb_set_par	= f_iris_set_par,

};

#define ASSUME_FORMER_SLOAD_USEC 1000
#define CB_DELAY_JIFFIES 0
/* vsync interval and a little bit more */
#define CB_RETRY_JIFFIES msecs_to_jiffies(45)

static void f_iris_callback_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct f_iris_par *par = container_of(dwork, struct f_iris_par, cb_work);
	struct list_head *cur, *next;
	struct list_o_callback *list_cb;
	struct list_head tmp_list;
	unsigned long flags;
	bool retry = false;

	INIT_LIST_HEAD(&tmp_list);

	spin_lock_irqsave(&par->spinlock, flags);

	list_for_each_safe(cur, next, &par->list_o_callback_wait) {
		list_cb = list_entry(cur, struct list_o_callback, list);
		if (!list_cb->retry) {
			if (retry) {
				/* guarantee a ordering */
				break;
			}

			list_del(&list_cb->list);
			list_add_tail(&list_cb->list, &tmp_list);
		}
		else {
			/*
			 * care about race condigion(see iris_isr)
			 */
			list_cb->retry = false;
			retry = true;
		}
	}

	if (retry)
		queue_delayed_work(system_wq, &par->cb_work, CB_RETRY_JIFFIES);

	spin_unlock_irqrestore(&par->spinlock, flags);

	/* callback without spinlock */
	if (!list_empty(&tmp_list)) {
		list_for_each_safe(cur, next, &tmp_list) {
			list_cb = list_entry(cur, struct list_o_callback, list);

			/* release framebuffer */
			list_cb->cb(list_cb->arg);
			kfree(list_cb);
		}
	}
}

/*
 * runtime_pm: the fact that these irqs were enabled to get this interrupt
 * means that it is OK to access the hardware
 */

static irqreturn_t iris_isr(int irq, void *arg)
{
	struct f_iris_par *par = arg;
	void __iomem *base = par->base;
	unsigned long flags;
	u32 status, temp;
#if defined(CONFIG_DEBUG_FS)
	struct timespec ts1;
#endif

	status = __raw_readl(base + IRIS_GBLC_IRQ_STATUS_OFS);
	writel(status, base + IRIS_GBLC_IRQ_CLEAR_OFS);

	/* vsync */
	if (fdb_getval(status, IRIS_GBLC_IRQ_STATUS, EXTDST0_FRAMECOMPLETE)) {
		struct list_head *pos = NULL;
		struct list_o_completions *loc = NULL;

		par->frame_index++;
		if (par->source && par->source->ops->sync)
			par->source->ops->sync(par->source, par->id,
							par->frame_index);

		spin_lock_irqsave(&par->spinlock, flags);
		list_for_each(pos, &par->list_o_completions) {
			loc = list_entry(pos, struct list_o_completions, list);
			complete(&loc->c);
		}
		spin_unlock_irqrestore(&par->spinlock, flags);
	}

	/* blitter shadow load complete (blitter start?) */
	if (fdb_getval(status, IRIS_GBLC_IRQ_STATUS, STORE0_SHADOWLOAD)) {
#if defined(CONFIG_DEBUG_FS)
		ts1 = current_kernel_time();
		temp = (u32)(timespec_to_ns(&ts1) -
				timespec_to_ns(&par->ts_entered_state)) / 1000;
		par->global.count_us_pipeline_setup += temp;
		par->global.count_ms_blitter_busy += temp / 1000;
		par->ts_entered_state = ts1;
#endif
	}

	/* blitter sequence complete */
/*	if (fdb_getval(status, IRIS_GBLC_IRQ_STATUS, STORE0_SEQCOMPLETE))
		complete(&par->blitter_seq_completion); */

	/* blitter complete */
	if (fdb_getval(status, IRIS_GBLC_IRQ_STATUS, STORE0_FRAMECOMPLETE))
		f_iris_blitter_complete_irq(par);

	/* display generator shadow load */
	if (fdb_getval(status, IRIS_GBLC_IRQ_STATUS, DISPGEN_SHADOW_LOAD))
		complete(&par->shadow_display_completion);

	/* frame generator (memory stream) shadow load */
	if (fdb_getval(status, IRIS_GBLC_IRQ_STATUS, FRAMEGEN_SHADOW_LOAD)) {
		struct list_head *cur, *next;
		struct list_o_callback *list_cb;
		ktime_t now = ktime_get();

		spin_lock_irqsave(&par->spinlock, flags);

		list_for_each_safe(cur, next, &par->list_o_callback) {
			list_cb = list_entry(cur, struct list_o_callback, list);
			/*
			 * We care about race condition between set_paddr and shadow load irq.
			 * If elapsed time from set_paddr is shorter than ASSUME_FORMER_SLOAD_USEC,
			 * we assume the irq is raised by former set_paddr.
			 *
			 * If the irq is truly raised by the set_paddr, next shadow load irq will
			 * not raise. We consider this case, we re-queue work if list_cb->retry is
			 * true in f_iris_callback_work.
			 */
			if (ktime_us_delta(now, list_cb->load_time) < ASSUME_FORMER_SLOAD_USEC)
				list_cb->retry = true;

			list_del(&list_cb->list);
			list_add_tail(&list_cb->list, &par->list_o_callback_wait);
		}

		/*
		 * If callbacks are pending although iris_isr does not add callbacks to the list,
		 * f_iris_callback_work might have re-queued work.
		 * In this case, iris_isr override delay timer.
		 */
		if (!list_empty(&par->list_o_callback_wait))
			mod_delayed_work(system_wq, &par->cb_work, CB_DELAY_JIFFIES);

		spin_unlock_irqrestore(&par->spinlock, flags);

		complete(&par->shadow_memory_completion);
	}

	/* display engine sequence complete (FGEN enable / disable) */
	if (fdb_getval(status, IRIS_GBLC_IRQ_STATUS, DISPENG_SEQCOMPLETE))
		complete(&par->dispeng_seq_completion);

	return IRQ_HANDLED;
}

static void
_f_iris_static_initialization(struct f_iris_par *par)
{
	void __iomem *base = par->base;
	int n;

	/*
	 * needs external runtime_pm coverage because also called in rpm resume
	 *
	 * one-time "static" initialization... display path consists of
	 *
	 *  - FRAMEGEN
	 *  - PXENG
	 *  - FETCH3
	 *  - EXTDST0
	 *
	 * everything than can have shadowed mode has it enabled now
	 */

	/* Unlock  */

	if (__raw_readl(base + DISPENG_FRAMEGEN0_LOCK_STATUS_OFS) & 1)
		/* only if locked... */
		writel(0xb4ac332b, base + DISPENG_FRAMEGEN0_LOCK_UNLOCK_OFS);

	if (__raw_readl(base + IRIS_GBLC_LOCK_STATUS_OFS) & 1)
		/* only if locked... */
		writel(0x553ccc93, base + IRIS_GBLC_LOCK_UNLOCK_OFS);

	dev_info(par->dev, "IRIS IP version 0x%x\n",
		 __raw_readl(base + IRIS_GBLC_IPID_OFS));

	writel(
	       fdb_const(PXE_F3_STATCTL, TOGGLEFIELD, 0) |
	       fdb_const(PXE_F3_STATCTL, SETFIELD, 0) |
	       fdb_const(PXE_F3_STATCTL, CLOCKDISABLE, 0) |
	       fdb_const(PXE_F3_STATCTL, SWRESET, 0) |
	       fdb_const(PXE_F3_STATCTL, SHDEN, 1),
	       base + PXE_F3_STATCTL_OFS);

	n = readl(base + PXE_F3_BURSTPROPS_OFS);
	par->max_burst_buffers =
			fdb_getval(n, PXE_F3_BURSTPROPS, MAXNUMBUFFERS);
	par->max_burst_len_at_max_buffers =
			fdb_getval(n, PXE_F3_BURSTPROPS, MAXBURSTLENGTH);
	dev_dbg(par->dev,
		"IRIS Fetch3 burst caps, max bufs: %d, len at max bufs: %d\n",
		par->max_burst_buffers, par->max_burst_len_at_max_buffers);

	/* Static Setup #1a: <pu>_shdw to SHADOWED for all processing units */

	writel(fdb_const(PXE_PXB_FETCH0_CFG, FETCH0_SHDW, 1),
	       base + PXE_PXB_FETCH0_CFG_OFS);
	writel(fdb_const(PXE_PXB_FETCH1_CFG, FETCH1_SHDW, 1),
	       base + PXE_PXB_FETCH1_CFG_OFS);
	writel(fdb_const(PXE_PXB_FETCH2_CFG, FETCH2_SHDW, 1),
	       base + PXE_PXB_FETCH2_CFG_OFS);
	writel(fdb_const(PXE_PXB_FETCH3_CFG, FETCH3_SHDW, 1) |
	       fdb_const(PXE_PXB_FETCH3_CFG, FETCH3_SRC_SEL, 0),
	       base + PXE_PXB_FETCH3_CFG_OFS);
	writel(fdb_const(PXE_PXB_FETCH4_CFG, FETCH4_SHDW, 1),
	       base + PXE_PXB_FETCH4_CFG_OFS);
	writel(fdb_const(PXE_PXB_ROP0_CFG, ROP0_SHDW, 1),
	       base + PXE_PXB_ROP0_CFG_OFS);
	writel(fdb_const(PXE_PXB_MATRIX0_CFG, MATRIX0_CLKEN, 1) |
	       fdb_const(PXE_PXB_MATRIX0_CFG, MATRIX0_SHDW, 1),
	       base + PXE_PXB_MATRIX0_CFG_OFS);
	writel(fdb_const(PXE_PXB_CLUT0_CFG, CLUT0_SHDW, 1),
	       base + PXE_PXB_CLUT0_CFG_OFS);

	writel(fdb_const(PXE_PXB_BLITBLEND0_CFG, BLITBLEND0_SHDW, 1),
	       base + PXE_PXB_BLITBLEND0_CFG_OFS);

	writel(fdb_const(PXE_PXB_EXTDST0_CFG, EXTDST0_SHDW, 1) |
	       fdb_const(PXE_PXB_EXTDST0_CFG, EXTDST0_SRC_SEL, 0xf),
	       base + PXE_PXB_EXTDST0_CFG_OFS);
	writel(fdb_const(PXE_PXB_LYRBLND1_CFG, LYRBLND1_SHDW, 1) |
	       fdb_const(PXE_PXB_LYRBLND1_CFG, LYRBLND1_CLKEN, 1) |
	       fdb_const(PXE_PXB_LYRBLND1_CFG, LYRBLND1_SEC_SEL, 0) |
	       fdb_const(PXE_PXB_LYRBLND1_CFG, LYRBLND1_PRIM_SEL, 0xe),
	       base + PXE_PXB_LYRBLND1_CFG_OFS);
	writel(fdb_const(PXE_PXB_LYRBLND0_CFG, LYRBLND0_SHDW, 1) |
	       fdb_const(PXE_PXB_LYRBLND0_CFG, LYRBLND0_CLKEN, 1) |
	       fdb_const(PXE_PXB_LYRBLND0_CFG, LYRBLND0_SEC_SEL, 0) |
	       fdb_const(PXE_PXB_LYRBLND0_CFG, LYRBLND0_PRIM_SEL, 0xc),
	       base + PXE_PXB_LYRBLND0_CFG_OFS);

	writel(fdb_const(PXE_PXB_HSCALER0_CFG, HSCALER0_SHDW, 1),
	       base + PXE_PXB_HSCALER0_CFG_OFS);
	writel(fdb_const(PXE_PXB_VSCALER0_CFG, VSCALER0_SHDW, 1),
	       base + PXE_PXB_VSCALER0_CFG_OFS);

	writel(fdb_const(PXE_PXB_STORE0_CFG, STORE0_SHDW, 1) |
	       fdb_const(PXE_PXB_STORE0_CFG, STORE0_SRC_SEL, 1),
	       base + PXE_PXB_STORE0_CFG_OFS);

	/* Static Setup #1b: ShdEn for above units also true */

	writel(fdb_const(PXE_F0_STATCTL, CLOCKDISABLE, 0) |
	       fdb_const(PXE_F0_STATCTL, SWRESET, 0) |
	       fdb_const(PXE_F0_STATCTL, SHDEN, 1),
	       base + PXE_F0_STATCTL_OFS);
	writel(fdb_const(PXE_S0_STATCTL, CLOCKDISABLE, 0) |
	       fdb_const(PXE_S0_STATCTL, SWRESET, 0) |
	       fdb_const(PXE_S0_STATCTL, SHDEN, 1),
	       base + PXE_S0_STATCTL_OFS);

	/* increase store burst to AXI limit */
	writel(fdb_const(PXE_S0_BURSTBUFFER_MGT, LENGTH, 16),
	       base + PXE_S0_BURSTBUFFER_MGT_OFS);

	writel(fdb_const(PXE_F3_STATCTL, SHDEN, 1),
	       base + PXE_F1_STATCTL_OFS);
	writel(fdb_const(PXE_F3_STATCTL, SHDEN, 1),
	       base + PXE_F2_STATCTL_OFS);
	writel(fdb_const(DISPENG_FRAMEGEN0_FGSTCTRL, SHDTOKEPPRIM, 1) |
	       fdb_const(DISPENG_FRAMEGEN0_FGSTCTRL, SHDLDSEL, 0) |
	       fdb_const(DISPENG_FRAMEGEN0_FGSTCTRL, SHDEN, 1),
	       base + DISPENG_FRAMEGEN0_FGSTCTRL_OFS);

	writel(fdb_const(PXE_F3_STATCTL, TOGGLEFIELD, 0) |
	       fdb_const(PXE_F3_STATCTL, SETFIELD, 0) |
	       fdb_const(PXE_F3_STATCTL, CLOCKDISABLE, 0) |
	       fdb_const(PXE_F3_STATCTL, SWRESET, 0) |
	       fdb_const(PXE_F3_STATCTL, SHDEN, 1),
	       base + PXE_F3_STATCTL_OFS);

	writel(fdb_const(PXENG_CLUT0_STATCTL, SHDEN, 1),
	       base + PXENG_CLUT0_STATCTL_OFS);
	writel(fdb_const(PXENG_MATRIX0_STATCTL, SHDEN, 1),
	       base + PXENG_MATRIX0_STATCTL_OFS);
	writel(fdb_const(PXENG_ROP0_STATCTL, SHDEN, 1),
	       base + PXENG_ROP0_STATCTL_OFS);
	writel(fdb_const(PXENG_HSCALER0_STATCTL, SHDEN, 1),
	       base + PXENG_HSCALER0_STATCTL_OFS);
	writel(fdb_const(PXENG_VSCALER0_STATCTL, SHDEN, 1),
	       base + PXENG_VSCALER0_STATCTL_OFS);
	writel(fdb_const(PXE_S0_STATCTL, SHDEN, 1),
	       base + PXE_S0_STATCTL_OFS);
	writel(fdb_const(PXENG_LYRBLND0_STATCTL, SHDEN, 1),
	       base + PXENG_LYRBLND0_STATCTL_OFS);
	writel(fdb_const(PXENG_LYRBLND1_STATCTL, SHDEN, 1),
	       base + PXENG_LYRBLND1_STATCTL_OFS);

	/* Static Setup #2: extdst0_sync_mode of pxeng to SYNC / whole pipe */

	writel(fdb_const(PXE_PXB_SYNC_MODE, EXTDST0_SYNC_MODE, 2),
	       base + PXE_PXB_SYNC_MODE_OFS);

	/*
	 *  Static Setup #3: disable shadow token LayerBlend endpoints
	 *                  (ShdTokEpec <- 0) - already done
	 */

	/*
	 *  Static Setup #4: enable shadow token endpoint at FrameGen input
	 *                  (ShdTokEpPrim <- 1 and ShdLdSel <- TRIGGER)
	 */

	writel(fdb_const(DISPENG_FRAMEGEN0_FGSTCTRL, SHDTOKEPPRIM, 1) |
	       fdb_const(DISPENG_FRAMEGEN0_FGSTCTRL, SHDLDSEL, 0) |
	       fdb_const(DISPENG_FRAMEGEN0_FGSTCTRL, SHDEN, 1),
	       base + DISPENG_FRAMEGEN0_FGSTCTRL_OFS);

	/*
	 *  Static Setup #5: set all static + initial dyn for Disp + Mem Stream
	 */

	/* enable clut access */
	writel(fdb_const(PXENG_CLUT0_UNSHADOWEDCONTROL, R_EN, 1) |
	       fdb_const(PXENG_CLUT0_UNSHADOWEDCONTROL, G_EN, 1) |
	       fdb_const(PXENG_CLUT0_UNSHADOWEDCONTROL, B_EN, 1),
	       base + PXENG_CLUT0_UNSHADOWEDCONTROL_OFS);
	/* bypass clut */
	writel(fdb_const(PXENG_CLUT0_CTRL, IDX_BITS, 0) |
	       fdb_const(PXENG_CLUT0_CTRL, ALPHAINVERT, 0) |
	       fdb_const(PXENG_CLUT0_CTRL, ALPHAMASK, 0) |
	       fdb_const(PXENG_CLUT0_CTRL, COL_8BIT, 0) |
	       fdb_const(PXENG_CLUT0_CTRL, MODE, 0),
	       base + PXENG_CLUT0_CTRL_OFS);

	/* bypass matrix */
	writel(fdb_const(PXENG_MATRIX0_CTRL, ALPHAINVERT, 0) |
	       fdb_const(PXENG_MATRIX0_CTRL, ALPHAMASK, 0) |
	       fdb_const(PXENG_MATRIX0_CTRL, MODE, 0),
	       base + PXENG_MATRIX0_CTRL_OFS);

	writel(fdb_const(PXENG_LYRBLND0_STATCTL, SHDLDSEL, 0) |
	       fdb_const(PXENG_LYRBLND0_STATCTL, SHDTOKEPSEC, 0) |
	       fdb_const(PXENG_LYRBLND0_STATCTL, SHDEN, 1),
	       base + PXENG_LYRBLND0_STATCTL_OFS);
	/* bypass */
	writel(fdb_const(PXENG_LYRBLND0_CTRL, ALPHA, 0) |
	       fdb_const(PXENG_LYRBLND0_CTRL, ALPHAMASKMODE, 0) |
	       fdb_const(PXENG_LYRBLND0_CTRL, SEC_A_BLD_FUNC, 0) |
	       fdb_const(PXENG_LYRBLND0_CTRL, PRIM_A_BLD_FUNC, 0) |
	       fdb_const(PXENG_LYRBLND0_CTRL, SEC_C_BLD_FUNC, 0) |
	       fdb_const(PXENG_LYRBLND0_CTRL, PRIM_C_BLD_FUNC, 0) |
	       fdb_const(PXENG_LYRBLND0_CTRL, ALPHAMASKENABLE, 0) |
	       fdb_const(PXENG_LYRBLND0_CTRL, MODE, 0),
	       base + PXENG_LYRBLND0_CTRL_OFS);
	writel(fdb_const(PXENG_LYRBLND1_STATCTL, SHDLDSEL, 0) |
	       fdb_const(PXENG_LYRBLND1_STATCTL, SHDTOKEPSEC, 0) |
	       fdb_const(PXENG_LYRBLND1_STATCTL, SHDEN, 1),
	       base + PXENG_LYRBLND1_STATCTL_OFS);
	writel(fdb_const(PXENG_LYRBLND1_CTRL, ALPHA, 0) |
	       fdb_const(PXENG_LYRBLND1_CTRL, ALPHAMASKMODE, 0) |
	       fdb_const(PXENG_LYRBLND1_CTRL, SEC_A_BLD_FUNC, 0) |
	       fdb_const(PXENG_LYRBLND1_CTRL, PRIM_A_BLD_FUNC, 0) |
	       fdb_const(PXENG_LYRBLND1_CTRL, SEC_C_BLD_FUNC, 0) |
	       fdb_const(PXENG_LYRBLND1_CTRL, PRIM_C_BLD_FUNC, 0) |
	       fdb_const(PXENG_LYRBLND1_CTRL, ALPHAMASKENABLE, 0) |
	       fdb_const(PXENG_LYRBLND1_CTRL, MODE, 0),
	       base + PXENG_LYRBLND1_CTRL_OFS);

	writel(fdb_const(PXE_F3_FRAMEXOFFSET, FRAMEXOFFSET, 0) |
	       fdb_const(PXE_F3_FRAMEXOFFSET, FRAMEXOFFSETDECIMALPLACES, 0),
	       base + PXE_F3_FRAMEXOFFSET_OFS);
	writel(fdb_const(PXE_F3_FRAMEYOFFSET, FRAMEYOFFSET, 0) |
	       fdb_const(PXE_F3_FRAMEYOFFSET, FRAMEYOFFSETDECIMALPLACES, 0),
	       base + PXE_F3_FRAMEYOFFSET_OFS);
	writel(fdb_const(PXE_F3_SKIPWINDOWOFFSET, SKIPWINDOWYOFFSET, 0) |
	       fdb_const(PXE_F3_SKIPWINDOWOFFSET, SKIPWINDOWXOFFSET, 0),
	       base + PXE_F3_SKIPWINDOWOFFSET_OFS);
	writel(fdb_const(PXE_F3_SKIPWINDOWDIMENSIONS, SKIPWINDOWHEIGHT, 0) |
	       fdb_const(PXE_F3_SKIPWINDOWDIMENSIONS, SKIPWINDOWWIDTH, 0),
	       base + PXE_F3_SKIPWINDOWDIMENSIONS_OFS);
	writel(fdb_const(PXE_F3_CONSTANTCOLOR, CONSTANTCOLORRED, 0xff) |
	       fdb_const(PXE_F3_CONSTANTCOLOR, CONSTANTCOLORGREEN, 0) |
	       fdb_const(PXE_F3_CONSTANTCOLOR, CONSTANTCOLORBLUE, 0) |
	       fdb_const(PXE_F3_CONSTANTCOLOR, CONSTANTCOLORALPHA, 0xff),
	       base + PXE_F3_CONSTANTCOLOR_OFS);
	writel(fdb_const(PXE_F3_TRANSPARENTCOLOR, TRANSPARENTCOLORRED, 0) |
	       fdb_const(PXE_F3_TRANSPARENTCOLOR, TRANSPARENTCOLORGREEN, 0) |
	       fdb_const(PXE_F3_TRANSPARENTCOLOR, TRANSPARENTCOLORBLUE, 0),
	       base + PXE_F3_TRANSPARENTCOLOR_OFS);
	writel(/* SHADOWED */
	       fdb_const(PXE_F3_FIRPOSITIONS, FIRPOSITION3, 0) |
	       fdb_const(PXE_F3_FIRPOSITIONS, FIRPOSITION2, 0) |
	       fdb_const(PXE_F3_FIRPOSITIONS, FIRPOSITION1, 0) |
	       fdb_const(PXE_F3_FIRPOSITIONS, FIRPOSITION0, 0),
	       base + PXE_F3_FIRPOSITIONS_OFS);
	writel(/* SHADOWED */
	       fdb_const(PXE_F3_FIRCOEFFICIENTS, FIRCOEFFICIENT3, 0) |
	       fdb_const(PXE_F3_FIRCOEFFICIENTS, FIRCOEFFICIENT2, 0) |
	       fdb_const(PXE_F3_FIRCOEFFICIENTS, FIRCOEFFICIENT1, 0) |
	       fdb_const(PXE_F3_FIRCOEFFICIENTS, FIRCOEFFICIENT0, 0),
	       base + PXE_F3_FIRCOEFFICIENTS_OFS);
	writel(/* SHADOWED */
	       fdb_const(PXE_F3_WARPCONTROL, WARPSYMMETRICOFFSET, 0) |
	       fdb_const(PXE_F3_WARPCONTROL, WARPCOORDINATEMODE, 0) |
	       fdb_const(PXE_F3_WARPCONTROL, WARPBITSPERPIXEL, 1),
	       base + PXE_F3_WARPCONTROL_OFS);

	/* set max mem bandwidth for non-display fetch */

	n = readl(base + PXE_F0_BURSTPROPS_OFS);
	n = fdb_getval(n, PXE_F0_BURSTPROPS, MAXBURSTLENGTH) *
	    fdb_getval(n, PXE_F0_BURSTPROPS, MAXNUMBUFFERS);
	writel((AXI_BURST_LIMIT << 8) | (n / AXI_BURST_LIMIT),
	       base + PXE_F0_BRSTBUFMGT_OFS);
	n = readl(base + PXE_F1_BURSTPROPS_OFS);
	n = fdb_getval(n, PXE_F1_BURSTPROPS, MAXBURSTLENGTH) *
	    fdb_getval(n, PXE_F1_BURSTPROPS, MAXNUMBUFFERS);
	writel((AXI_BURST_LIMIT << 8) | (n / AXI_BURST_LIMIT),
	       base + PXE_F1_BRSTBUFMGT_OFS);
	n = readl(base + PXE_F2_BURSTPROPS_OFS);
	n = fdb_getval(n, PXE_F2_BURSTPROPS, MAXBURSTLENGTH) *
	    fdb_getval(n, PXE_F2_BURSTPROPS, MAXNUMBUFFERS);
	writel((AXI_BURST_LIMIT << 8) | (n / AXI_BURST_LIMIT),
	       base + PXE_F2_BRSTBUFMGT_OFS);

	/* pixelbus */

	/* frame generator */
	writel(/* SHADOWED */
	       fdb_const(DISPENG_FRAMEGEN0_PACFG, PSTARTY, 1) |
	       fdb_const(DISPENG_FRAMEGEN0_PACFG, PSTARTX, 1),
	       base + DISPENG_FRAMEGEN0_PACFG_OFS);

	/* this is the colour issued for "illegal pixels" */
	writel(/* SHADOWED */
	       fdb_const(DISPENG_FRAMEGEN0_FGCCR, CCALPHA, 0) |
	       fdb_const(DISPENG_FRAMEGEN0_FGCCR, CCRED, 0) |
	       fdb_const(DISPENG_FRAMEGEN0_FGCCR, CCGREEN, 0) |
	       fdb_const(DISPENG_FRAMEGEN0_FGCCR, CCBLUE, 0xff),
	       base + DISPENG_FRAMEGEN0_FGCCR_OFS);
	writel(/* SHADOWED */
	       6, base + DISPENG_FRAMEGEN0_FGINPANIC_OFS);

	 /* 1 Constant color Primary input = 2, Test pattern = 6 */
	n = 2;
	if (par->colourbars)
		n = 6;
	writel(/* SHADOWED */
	       fdb_const(DISPENG_FRAMEGEN0_FGLNCTRL, FGDM, n),
	       base + DISPENG_FRAMEGEN0_FGLNCTRL_OFS);

	writel(/* DIRECT */
	       fdb_const(PXENG_EXTDST0_STATCTL, PERFCOUNTMODE, 0) |
	       fdb_const(PXENG_EXTDST0_STATCTL, KICK_MODE, 1),
	       base + PXENG_EXTDST0_STATCTL_OFS);

	/* any interrupts are interesting */

	writel(0xffff, base + IRIS_GBLC_IRQ_CLEAR_OFS);

	writel(/* DIRECT */
	       0xffff, par->base + IRIS_GBLC_IRQ_ENABLE_OFS);

	/* leave it like that until first set_timings */

	pm_runtime_mark_last_busy(par->dev);
}

static const char const *iris_irq_reason[] = {
	"S0-seq-complete",
	"ExtDst0-seq-complete",
	"S0-shadow-load",
	"S0-frame-complete",
	"ExtDst0-frame-complete",
	"lyrblnd0-shadow-load",
	"lyrblnd1-shadow-load",
	"DispEng-shadow-load",
	"DispEng-seq-complete",
	"id0",
	"id1",
	"id2",
	"id3",
	"fgen0-shdadow-load",
	"sig0-shadow-load",
	"sig0-meas-complete",
	"sig0-error",
};


static int
f_iris_fb_probe(struct platform_device *pdev)
{
	const char *mode = NULL;
	struct f_iris_par *par;
	struct resource *res;
	struct fb_info *info;
	unsigned long flags;
	char clkname[8];
	const int *p;
	int ret = 0;
	int n;

	info = framebuffer_alloc(sizeof(struct f_iris_par), &pdev->dev);
	if (!info) {
		dev_err(&pdev->dev, "Failed to alloc framebuffer\n");
		return -ENOMEM;
	}
	par = info->par;
	memset(par, 0, sizeof(*par));
	par->info = info;
	/* stored per IRIS instance to allow more flexible queues later */
	par->blits = &blits;
	par->blit_c = &blit_comp;
	mutex_init(&par->lock);
	INIT_LIST_HEAD(&par->list_o_completions);
	INIT_LIST_HEAD(&par->list_o_callback);
	INIT_LIST_HEAD(&par->list_o_callback_wait);
	INIT_DELAYED_WORK(&par->cb_work, f_iris_callback_work);
	init_completion(&par->shadow_display_completion);
	init_completion(&par->shadow_memory_completion);
	init_completion(&par->dispeng_seq_completion);
	INIT_LIST_HEAD(&par->blitter_list);

	INIT_WORK(&par->blitter_emergency_work, iris_emergency_reset_work);

	platform_set_drvdata(pdev, info);
	info->fbops = &f_iris_fb_ops;
	info->fix = f_iris_fb_fix;
	info->dev = &pdev->dev;
	par->dev = &pdev->dev;
	spin_lock_init(&par->spinlock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Missing base resource\n");
		return -EINVAL;
	}

	/* crtc register map */
	par->base = ioremap(res->start, res->end - res->start);
	par->base_pa = res->start;

	/* optional... mode will be NULL if not given */
	of_property_read_string(pdev->dev.of_node, "mode", &mode);

	p = of_get_property(pdev->dev.of_node, "id", NULL);
	if (p)
		par->id = be32_to_cpu(*p);

	p = of_get_property(pdev->dev.of_node, "simple", NULL);
	if (p)
		par->simple = !!be32_to_cpu(*p);

	p = of_get_property(pdev->dev.of_node, "colourbars", NULL);
	if (p)
		par->colourbars = !!be32_to_cpu(*p);

	p = of_get_property(pdev->dev.of_node, "perf-mode", NULL);
	if (p)
		par->perf_mode = !!be32_to_cpu(*p);

	if (par->simple) {
		if (!mode) {
			dev_err(&pdev->dev, "simple=1 requires mode\n");
			return -EINVAL;
		}
		dev_info(&pdev->dev, "Simple fb id=%d, mode=%s\n",
			 par->id, mode);
	} else {
		if (mode)
			dev_dbg(&pdev->dev, "DRM fb id=%d, mode=%s\n",
				par->id, mode);
		else
			dev_dbg(&pdev->dev, "DRM fb id=%d\n", par->id);
	}

	p = of_get_property(pdev->dev.of_node, "h-pixels-fb-offset", NULL);
	if (p) {
		par->h_pixels_fb_offset = be32_to_cpu(*p);
		dev_info(&pdev->dev, "Horizontal fb start offset by %dpx\n",
			 par->h_pixels_fb_offset);
	}

	for (n = 0; n < ARRAY_SIZE(par->irq); n++) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, n);
		if (!res) {
			dev_err(&pdev->dev, "Missing interrupt resource\n");
			ret = -EINVAL;
			goto bail0;
		}
		sprintf(par->irq_name[n], "%s:%s", dev_name(&pdev->dev),
			iris_irq_reason[n]);
		par->irq[n] = res->start;
		ret = request_irq(par->irq[n], iris_isr, IRQF_TRIGGER_RISING,
				  par->irq_name[n], par);
		if (ret) {
			dev_err(&pdev->dev, "failed to allocate irq\n");
			goto bail0;
		}
		disable_irq(par->irq[n]);
	}

	ret = 0;
	par->clocks_enabled = 0;
	while (!ret) {
		sprintf(clkname, "clk%d", par->clocks_enabled + 1);
		par->clocks[par->clocks_enabled] = clk_get(&pdev->dev, clkname);
		if (IS_ERR(par->clocks[par->clocks_enabled])) {
			ret = 1;
			continue;
		}
		par->clocks_enabled++;
	}

	/* allocate blitter overlap buffer */
	par->blit_temp_va = (char __force __iomem *)
			    dma_alloc_writecombine(info->dev,
						   BLITTER_TEMP_SIZE,
						   &par->blit_temp_paddr,
						   GFP_KERNEL);
	if (!par->blit_temp_va) {
		dev_err(&pdev->dev, "Failed to allocate blit temp buffer\n");
		goto bail1a;
	}

#if defined(CONFIG_DEBUG_FS)
	par->ts_entered_state = current_kernel_time();
#endif

	pm_runtime_set_autosuspend_delay(&pdev->dev, 2000); /* 2s delay */
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	/* runtime_pm coverage just for probe, enable/disable also cover it */
	pm_runtime_get_sync(&pdev->dev);

	_f_iris_static_initialization(par);

	info->pseudo_palette = par->pseudo_palette;
	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret) {
		dev_err(&pdev->dev, "Failed to alloc cmap\n");
		goto bail2;
	}
	fb_set_cmap(&info->cmap, info);

	if (par->simple) {
		f_iris_allocate_backing_set_var(info, mode);

		ret = register_framebuffer(info);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"framebuffer reg failed %d\n", ret);
			goto bail3;
		}
	}

	/* register our fdb_child with f_fdb bus */
	ret = fdb_register(&pdev->dev, &par->fdb_child, &f_iris_fb_fdb_ops);
	if (ret < 0) {
		dev_err(&pdev->dev, "fdb registration failed\n");
		goto bail4;
	}

	/* list our blitter as servicing the IRIS queue */
	spin_lock_irqsave(&par->blits->lock, flags);
	list_add(&par->blitter_list, &par->blits->blitter_list);
	_f_iris_blitter_poll_queue(par);
	spin_unlock_irqrestore(&par->blits->lock, flags);

	if (par->simple)
		dev_info(&pdev->dev, "fb%d: %s frame buffer device\n",
			 info->node, info->fix.id);
	else
		dev_info(&pdev->dev,
			 "f_iris fb%d hardware registered on fdb\n", par->id);

	pm_runtime_mark_last_busy(&pdev->dev);

/* HDMI is broken on S70 if we give up the power here
 * however same setup works on 3.16-rc4 so probably DRM changes
 *
 *	pm_runtime_put_autosuspend(&pdev->dev);
 */
	return 0;
bail4:
	if (par->simple)
		unregister_framebuffer(info);
bail3:
	if (par->simple)
		fb_dealloc_cmap(&info->cmap);
bail2:
	pm_runtime_put_sync_suspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	if (par->blit_temp_va)
		dma_free_writecombine(info->dev, BLITTER_TEMP_SIZE,
				      par->blit_temp_va, par->blit_temp_paddr);

bail1a:
	iounmap(par->base);

	while (--par->clocks_enabled >= 0)
		clk_put(par->clocks[par->clocks_enabled]);

bail0:
	for (n = 0; n < ARRAY_SIZE(par->irq); n++)
		if (par->irq[n])
			free_irq(par->irq[n], par);

	framebuffer_release(info);

	return ret;
}

static int
f_iris_fb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct f_iris_par *par;
	unsigned long flags;
	int timeout = 100;
	int n;

	if (!info)
		return 0;

	par = info->par;
	par->going_away = true;
	/* remove ourselves from servicing the blitter queue */
	spin_lock_irqsave(&par->blits->lock, flags);
	list_del(&par->blitter_list);
	_f_iris_blitter_poll_queue(par);
	spin_unlock_irqrestore(&par->blits->lock, flags);

	while (par->blitter_busy && --timeout)
		usleep_range(1000, 5000);

	if (!timeout)
		dev_err(par->dev, "blitter timed out going idle\n");

	fdb_unregister(&pdev->dev, &par->fdb_child);

	for (n = 0; n < ARRAY_SIZE(par->irq); n++)
		if (par->irq[n])
			free_irq(par->irq[n], par);

	if (par->simple) {
		unregister_framebuffer(info);
		fb_dealloc_cmap(&info->cmap);
	}

	pm_runtime_disable(&pdev->dev);
	if (par->fb_va)
		dma_free_writecombine(info->dev, par->framesize,
				      par->fb_va, par->fbpaddr);

	if (par->blit_temp_va)
		dma_free_writecombine(info->dev, BLITTER_TEMP_SIZE,
				      par->blit_temp_va, par->blit_temp_paddr);

	iounmap(par->base);
	framebuffer_release(info);

	while (--par->clocks_enabled >= 0)
		clk_put(par->clocks[par->clocks_enabled]);

	pr_err("[IRIS] Done in %s\n", __func__);
	return 0;
}

#ifdef CONFIG_PM
#ifdef CONFIG_PM_RUNTIME
static int
f_iris_runtime_suspend(struct device *dev)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct f_iris_par *par = info->par;
	int n;

	dev_info(dev, "%s\n", __func__);

	if (__raw_readl(par->base + DISPENG_FRAMEGEN0_FGENABLE_OFS) &
			fdb_const(DISPENG_FRAMEGEN0_FGENABLE, FGEN, 1)) {
		init_completion(&par->dispeng_seq_completion);

		writel(fdb_const(DISPENG_FRAMEGEN0_FGENABLE, FGEN, 0),
		       par->base + DISPENG_FRAMEGEN0_FGENABLE_OFS);

		if (wait_for_completion_timeout(&par->dispeng_seq_completion,
						msecs_to_jiffies(60)) <= 0)
			dev_info(par->dev, "%s: dispeng stop timeout\n",
				 __func__);
	}

	for (n = 0; n < ARRAY_SIZE(par->irq); n++)
		if (par->irq[n])
			disable_irq(par->irq[n]);

	for (n = par->clocks_enabled - 1; n >= 0; n--)
		clk_disable_unprepare(par->clocks[n]);

	return 0;
}

static int
f_iris_runtime_resume(struct device *dev)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct f_iris_par *par = info->par;
	int n;
	int timeout = 5000; /* approx 100us units */

	dev_info(dev, "%s\n", __func__);

	/* first let the clocks back on */

	for (n = 0; n < par->clocks_enabled; n++)
		clk_prepare_enable(par->clocks[n]);

	/* from probe, with splash, this could be true */

	if (__raw_readl(par->base + DISPENG_FRAMEGEN0_FGENABLE_OFS) &
	    fdb_const(DISPENG_FRAMEGEN0_FGENABLE, FGEN, 1)) {
		writel(fdb_const(DISPENG_FRAMEGEN0_FGENABLE, FGEN, 0),
		       par->base + DISPENG_FRAMEGEN0_FGENABLE_OFS);

		/* display engine sequence complete (FGEN enable / disable) */
		while (--timeout && !fdb_getval(__raw_readl(par->base +
					       IRIS_GBLC_IRQ_STATUS_OFS),
				   IRIS_GBLC_IRQ_STATUS, DISPENG_SEQCOMPLETE))
			udelay(100);
		if (!timeout) {
			dev_err(dev, "%s: framegen disable fail\n", __func__);
			return -ETIME;
		}
	}

	/* reset everything in the IRIS that has a soft reset */

	writel(fdb_const(PXE_PXB_SYNC_MODE, EXTDST0_SWRESET, 1) |
	       fdb_const(PXE_PXB_SYNC_MODE, STORE0_SWRESET, 1),
	       par->base + PXE_PXB_SYNC_MODE_OFS);
	writel(fdb_const(PXE_F0_STATCTL, SWRESET, 1),
	       par->base + PXE_F0_STATCTL_OFS);
	writel(fdb_const(PXE_F1_STATCTL, SWRESET, 1),
	       par->base + PXE_F1_STATCTL_OFS);
	writel(fdb_const(PXE_F2_STATCTL, SWRESET, 1),
	       par->base + PXE_F2_STATCTL_OFS);
	writel(fdb_const(PXE_F3_STATCTL, SWRESET, 1),
	       par->base + PXE_F3_STATCTL_OFS);
	writel(fdb_const(PXE_S0_STATCTL, SWRESET, 1),
	       par->base + PXE_S0_STATCTL_OFS);

	/* unlock things and set shadow modes, initialize it out of reset */

	_f_iris_static_initialization(par);

	/* let the interrupts back on */

	for (n = 0; n < ARRAY_SIZE(par->irq); n++)
		if (par->irq[n])
			enable_irq(par->irq[n]);

	/*
	 * although drm will re-set the mode if we are in Xorg, it won't do
	 * anything if we are in framebuffer console... so re-set the mode
	 * ourselves, if we ever had a mode set
	 */
	if (par->subsequent) {
		par->subsequent = 0;
		_f_iris_fb_set_timings(par, &par->last_timings);
	}

	if (par->fbpaddr)
		f_iris_fb_fdb_set_paddr(&par->fdb_child, par->fbpaddr);

	return 0;
}
#endif

static int
f_iris_pm_suspend(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	return f_iris_runtime_suspend(dev);
}

static int
f_iris_pm_resume(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	return f_iris_runtime_resume(dev);
}
#endif

static const struct of_device_id f_iris_fb_dt_ids[] = {
	{ .compatible = "fujitsu,f-iris-fb" },
	{ /* sentinel */ }
};

#ifdef CONFIG_PM
static const struct dev_pm_ops f_iris_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(f_iris_pm_suspend, f_iris_pm_resume)
	SET_RUNTIME_PM_OPS(f_iris_runtime_suspend, f_iris_runtime_resume, NULL)
};
#endif

static struct platform_driver f_iris_fb_driver = {
	.probe = f_iris_fb_probe,
	.remove = f_iris_fb_remove,
	.driver = {
		.name = "f_iris_fb",
		.of_match_table = f_iris_fb_dt_ids,
#ifdef CONFIG_PM
		.pm = &f_iris_pm_ops,
#endif
	},
};


MODULE_DEVICE_TABLE(of, f_iris_fb_dt_ids);

static int __init f_iris_fb_init(void)
{
	int ret;

	blits.ring = kmalloc_array(BLIT_RING_LEN, sizeof(*blits.ring), GFP_KERNEL);
	blits.length = BLIT_RING_LEN;
	spin_lock_init(&blits.lock);
	INIT_LIST_HEAD(&blits.blitter_list);

	blit_comp.ring = kmalloc_array(BLIT_COMP_RING_LEN,
			sizeof(*blit_comp.ring), GFP_KERNEL);
	blit_comp.length = BLIT_COMP_RING_LEN;
	blit_comp.entry = 0;
	memset(blit_comp.ring, 0, sizeof(*blit_comp.ring) * BLIT_COMP_RING_LEN);

	ret = platform_driver_register(&f_iris_fb_driver);

	if (ret) {
		kfree(blits.ring);
		kfree(blit_comp.ring);
	}

	return ret;
}

static void __exit f_iris_fb_exit(void)
{
	platform_driver_unregister(&f_iris_fb_driver);
	kfree(blits.ring);
	kfree(blit_comp.ring);
}

module_init(f_iris_fb_init);
module_exit(f_iris_fb_exit);

MODULE_LICENSE("GPL");
