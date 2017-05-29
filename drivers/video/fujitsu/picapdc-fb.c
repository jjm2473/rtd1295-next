/*
 * DMP Pica PDC Framebuffer driver, hardware part
 * Copyright (C) 2013 Linaro, Ltd for Fujitsu Semi
 * Author: Andy Green <andy.green@linaro.org>
 *
 * This always instantiates an fdb child representing the framebuffer, if the
 * "simple" DT attribute is nonzero then it also registers as a simple Linux
 * framebuffer.  If !simple, the fdb child may be adopted by, eg, fdb-drm
 * driver and exposed that way.
 *
 * based on -->
 *
 * linux/drivers/video/skeletonfb.c -- Skeleton for a frame buffer device
 *
 *  Modified to new api Jan 2001 by James Simmons (jsimmons@transvirtual.com)
 *
 *  Created 28 Dec 1997 by Geert Uytterhoeven
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/io.h>

#include <video/picapdc-fb.h>

#include <video/fdb.h>

#define CNVT_TOHW(val, width) ((((val) << (width)) + 0x7FFF - (val)) >> 16)

static const struct fdb_format formats[] = {
/*	{ DRM_FORMAT_XRGB8888, { { 4, 1 } }, false }, */
/*	{ DRM_FORMAT_RGBX8888, { { 4, 1 } }, false }, */
/*	{ DRM_FORMAT_ARGB8888, { { 4, 1 } }, false }, */
	{ DRM_FORMAT_RGB888, { { 3, 1 } }, false },
	{ DRM_FORMAT_RGB565, { { 2, 1 } }, false }, /* RGB16-565 */
};

static void picapdc_fb_set_timings(struct f_fdb_child *fdb_child,
	struct fdb_video_timings *timings)
{
	struct picapdc_par *par = f_par_from_fdb_child(fdb_child);
	void __iomem *crtc = &par->base[0];
	struct display_timing *dt = &timings->dt;
	int lm, leftm;
	u32 col;
	int bits_pp = timings->bits_per_pixel;
	int bytes_pp;
	int mode = 0;

	bytes_pp = (bits_pp + 7) / 8;

	dev_info(par->dev, "(stride_px = %d bpp = %d)\n",
					timings->stride_px, bits_pp);
	fdb_dump_video_timings(timings);

	switch (bytes_pp) {
	case 4:
		mode = PICAPDC_MODE_R8G8B8A8;
		break;
	case 3:
		mode = PICAPDC_MODE_R8G8B8;
		break;
	case 2:
		mode = PICAPDC_MODE_R5G6B5;
		break;
	}

	leftm = (dt->hback_porch.typ - dt->hfront_porch.typ) * 2;
	lm = dt->vactive.typ + dt->vfront_porch.typ + dt->vback_porch.typ +
							      dt->vsync_len.typ;

	fdb_setl(crtc, PICAPDC_H_CTR, SIZE,  dt->hactive.typ +
		dt->hfront_porch.typ + dt->hsync_len.typ + dt->hback_porch.typ);
	fdb_setl(crtc, PICAPDC_H_ADDR_TIME, START, leftm);
	fdb_setl(crtc, PICAPDC_H_PIC_BORDER, START, leftm);
	fdb_setl(crtc, PICAPDC_H_LEFT_BORDER, START, leftm);

	fdb_setl(crtc, PICAPDC_H_RIGHT_BORDER, START, leftm + dt->hactive.typ);
	fdb_setl(crtc, PICAPDC_H_PIC_BORDER, END, leftm + dt->hactive.typ);
	fdb_setl(crtc, PICAPDC_H_BLANK, START, leftm + dt->hactive.typ);

	fdb_setl(crtc, PICAPDC_H_SYNC, START, dt->hfront_porch.typ - 1);
	fdb_setl(crtc, PICAPDC_H_BACKPORCH, START, dt->hsync_len.typ - 1);
	fdb_setl(crtc, PICAPDC_H_INTERRUPT, START, dt->hfront_porch.typ - 1);
	fdb_setl(crtc, PICAPDC_H_DMA, START, dt->hfront_porch.typ - 1);

	fdb_setl(crtc, PICAPDC_H_INTERRUPT, END, dt->hsync_len.typ);
	fdb_setl(crtc, PICAPDC_H_DMA, END, dt->hfront_porch.typ);

	fdb_setl(crtc, PICAPDC_V_COUNTER, SIZE, lm);
	fdb_setl(crtc, PICAPDC_V_BOTTOM_BORDER, START, lm - 1);
	fdb_setl(crtc, PICAPDC_V_BLANK, START, lm - 1);
	fdb_setl(crtc, PICAPDC_V_SYNC, START, dt->vfront_porch.typ - 1);
	fdb_setl(crtc, PICAPDC_V_INTERRUPT, START, dt->vfront_porch.typ - 1);

	fdb_setl(crtc, PICAPDC_V_ADDR_TIME, START, dt->vback_porch.typ +
						dt->vfront_porch.typ + 1);
	fdb_setl(crtc, PICAPDC_V_TOP_BORDER, START, dt->vback_porch.typ +
						dt->vfront_porch.typ + 1);
	fdb_setl(crtc, PICAPDC_V_BACKPORCH, START, dt->vfront_porch.typ +
						dt->vsync_len.typ - 1);
	fdb_setl(crtc, PICAPDC_V_INTERRUPT, END, dt->vfront_porch.typ +
							dt->vsync_len.typ - 1);
	fdb_setl(crtc, PICAPDC_V_INCREMENT_H, VALUE, dt->hfront_porch.typ - 4);
	fdb_setl(crtc, PICAPDC_V_PIC_BORDER, START, dt->vback_porch.typ + 3);
	fdb_setl(crtc, PICAPDC_V_PIC_BORDER, END, lm - 1);

	fdb_setl(crtc, PICAPDC_SRC, WIDTH, dt->hactive.typ);
	fdb_setl(crtc, PICAPDC_SRC, HEIGHT, dt->vactive.typ);
	fdb_setl(crtc, PICAPDC_OUT, WIDTH, dt->hactive.typ);
	fdb_setl(crtc, PICAPDC_OUT, HEIGHT, dt->vactive.typ);
	fdb_setl(crtc, PICAPDC_DATA, SIZE, timings->stride_px * bytes_pp);
	fdb_setl(crtc, PICAPDC_SIGNAL_POL, HSYNC, 0);
	fdb_setl(crtc, PICAPDC_SIGNAL_POL, VSYNC, 0);

	__raw_writel(fdb_const(PICAPDC_MODE, FORMAT, mode) |
		     fdb_const(PICAPDC_MODE, VSYNC_START, 0) |
		     fdb_const(PICAPDC_MODE, BURST_LENGTH, 3) |
		     fdb_const(PICAPDC_MODE, REQ_INTERVAL, 8),
						crtc + PICAPDC_MODE_OFS);

	/* setup gamma LUT as linear */
	fdb_setl(crtc, PICAPDC_GAMMA_ADS, ADS, 0);
	lm = fdb_const(PICAPDC_GAMMA_DATA, RED, 1) |
	     fdb_const(PICAPDC_GAMMA_DATA, GREEN, 1) |
	     fdb_const(PICAPDC_GAMMA_DATA, BLUE, 1);

	for (col = 0; col < (256 * lm); col += lm)
		__raw_writel(col, crtc + PICAPDC_GAMMA_DATA_OFS);
}

static int _picapdc_fb_hw_set_var(void __iomem *crtc,
		       struct fb_var_screeninfo *var, struct picapdc_par *par)
{
	struct fdb_video_timings timings;

	fdb_var_to_video_timings(var, &timings);
	picapdc_fb_set_timings(&par->fdb_child, &timings);

	return 0;
}

static void
picapdc_fb_fdb_set_paddr(struct f_fdb_child *fdb_child, dma_addr_t pa)
{
	struct picapdc_par *par = f_par_from_fdb_child(fdb_child);
	void __iomem *crtc = &par->base[0];

	dev_dbg(fdb_child->dev, "picapdc_fb_fdb_set_paddr: %lx\n",
							(unsigned long)pa);

	__raw_writel(pa, crtc + PICAPDC_ADR_FB0_OFS);
	__raw_writel(pa, crtc + PICAPDC_ADR_FB1_OFS);
}

static int
picapdc_fb_fdb_enable(struct f_fdb_child *fdb_child)
{
	struct picapdc_par *par = f_par_from_fdb_child(fdb_child);
	void __iomem *crtc = &par->base[0];

	dev_info(fdb_child->dev, "picapdc_fb_fdb_enable\n");
	__raw_writel(
		fdb_const(PICAPDC_START, INT_MASK_H, 1) |
		fdb_const(PICAPDC_START, INT_MASK_V, 0) |
		fdb_const(PICAPDC_START, INT_MASK_ERR, 0) |
		fdb_const(PICAPDC_START, OUT_EN, 1) |
		fdb_const(PICAPDC_START, START, 1), crtc + PICAPDC_START_OFS);

	return 0;
}

static void
picapdc_fb_fdb_disable(struct f_fdb_child *fdb_child)
{
	struct picapdc_par *par = f_par_from_fdb_child(fdb_child);
	void __iomem *crtc = &par->base[0];

	dev_info(fdb_child->dev, "picapdc_fb_fdb_disable\n");
	__raw_writel(
		fdb_const(PICAPDC_START, INT_MASK_H, 1) |
		fdb_const(PICAPDC_START, INT_MASK_V, 1) |
		fdb_const(PICAPDC_START, INT_MASK_ERR, 1) |
		fdb_const(PICAPDC_START, OUT_EN, 0) |
		fdb_const(PICAPDC_START, START, 0), crtc + PICAPDC_START_OFS);
}


static int _picapdc_fb_hw_init(struct fb_info *info)
{
	struct picapdc_par *par = info->par;
	void __iomem *crtc = &par->base[0];

	picapdc_fb_fdb_disable(&par->fdb_child);

	picapdc_fb_fdb_set_paddr(&par->fdb_child, par->fbpaddr);
	_picapdc_fb_hw_set_var(crtc, &info->var, par);

	picapdc_fb_fdb_enable(&par->fdb_child);

	return 0;
}

static int picapdcfb_setcolreg(unsigned regno, unsigned red, unsigned green,
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

static int picapdc_allocate_backing_set_var(struct fb_info *info,
							const char *mode)
{
	struct picapdc_par *par = info->par;
	int ret = 0;
	int bytes_per_pixel;
	int line_length;

	mutex_lock(&par->lock);

	if (mode) {
		ret = fdb_get_of_var(mode, &info->var);
		if (ret) {
			dev_err(par->dev, "Ofvar for %s not found\n", mode);
			ret = -ENOMEM;
			goto bail;
		}
	}

	info->var.accel_flags = 0;

	bytes_per_pixel = (info->var.bits_per_pixel + 7) / 8;
	line_length = info->var.xres_virtual * bytes_per_pixel;

	if (par->fb_va &&
		       par->framesize == info->var.yres_virtual * line_length) {
		info->fix.line_length = line_length;
		info->screen_base = (void *)par->fb_va;
		info->fix.smem_start = par->fbpaddr;
		info->fix.smem_len = par->framesize;
		info->pseudo_palette = par->pseudo_palette;
		info->flags = FBINFO_DEFAULT;

		goto bail;
	}

	info->fix.line_length = line_length;
	par->framesize = info->var.yres_virtual * line_length;
	if (par->fb_va)
		dma_free_coherent(par->dev, par->framesize,
					par->fb_va, par->fbpaddr);
	par->fb_va = (char __force __iomem *)dma_alloc_coherent(
						par->dev, par->framesize,
						     &par->fbpaddr, GFP_KERNEL);
	if (!par->fb_va) {
		dev_err(par->dev, "Unable to allocate backing store for %d\n",
								par->framesize);
		ret = -ENOMEM;
		goto bail;
	}
	info->screen_base = (void *)par->fb_va;
	dev_info(par->dev, "virt(%d x %d) fb mem at va 0x%llX pa 0x%llX\n",
			info->var.xres_virtual, info->var.yres_virtual,
			(u64)(unsigned long)info->screen_base, (u64)par->fbpaddr);
	info->fix.smem_start = par->fbpaddr;
	info->fix.smem_len = par->framesize;
	info->pseudo_palette = par->pseudo_palette;
	info->flags = FBINFO_DEFAULT;

	ret = _picapdc_fb_hw_init(info);
bail:
	mutex_unlock(&par->lock);

	return ret;
}

static int picapdcfb_fb_set_par(struct fb_info *info)
{
	return picapdc_allocate_backing_set_var(info, NULL);
}

static int picapdcfb_blank(int blank_mode, struct fb_info *info)
{
	struct picapdc_par *par = info->par;

	dev_info(par->dev, "picapdcfb_blank %d\n", blank_mode);

	mutex_lock(&par->lock);

	if (blank_mode == FB_BLANK_UNBLANK)
		par->fdb_child.ops->enable(&par->fdb_child);
	else
		par->fdb_child.ops->disable(&par->fdb_child);

	mutex_unlock(&par->lock);

	return 0;
}

void picapdc_fb_get_timings(struct f_fdb_child *fdb_child,
					struct fdb_video_timings *timings)
{
	const char *mode;
	struct fb_var_screeninfo var;
	int ret;

	if (of_property_read_string(fdb_child->dev->of_node, "mode", &mode)) {
		dev_err(fdb_child->dev, "Missing mode\n");
		return;
	}

	dev_info(fdb_child->dev, "fdb_fb_get_timings: mode = %s\n", mode);

	ret = fdb_get_of_var(mode, &var);
	if (ret) {
		dev_err(fdb_child->dev, "Failed to get of var for %s\n", mode);
		return;
	}

	fdb_var_to_video_timings(&var, timings);
}

void picapdc_fb_set_gamma(struct f_fdb_child *fdb_child,
				u16 red, u16 green, u16 blue, int regno)
{
	struct picapdc_par *par = f_par_from_fdb_child(fdb_child);
	void __iomem *crtc = &par->base[0];

	mutex_lock(&par->lock);

	__raw_writel(regno, crtc + PICAPDC_GAMMA_ADS_OFS);
	__raw_writel(
		fdb_const(PICAPDC_GAMMA_DATA, RED, red) |
		fdb_const(PICAPDC_GAMMA_DATA, GREEN, green) |
		fdb_const(PICAPDC_GAMMA_DATA, BLUE, blue),
				      crtc + PICAPDC_GAMMA_DATA_OFS);

	mutex_unlock(&par->lock);
}
static void picapdc_fb_get_gamma(struct f_fdb_child *fdb_child,
				u16 *red, u16 *green, u16 *blue, int regno)
{
	struct picapdc_par *par = f_par_from_fdb_child(fdb_child);
	void __iomem *crtc = &par->base[0];
	u32 col;

	mutex_lock(&par->lock);

	__raw_writel(regno, crtc + PICAPDC_GAMMA_ADS_OFS);
	col = __raw_readl(crtc + PICAPDC_GAMMA_DATA_OFS);

	mutex_unlock(&par->lock);

	*red = fdb_getval(col, PICAPDC_GAMMA_DATA, RED);
	*green = fdb_getval(col, PICAPDC_GAMMA_DATA, GREEN);
	*blue = fdb_getval(col, PICAPDC_GAMMA_DATA, BLUE);
}

static int picapdc_fb_check_timings(struct f_fdb_child *fdb_child,
			struct fdb_video_timings *timings)
{
	return 0;
}

static const struct fdb_format *picapdc_get_fdb_formats(
			struct f_fdb_child *fdb_child, int *count)
{
	*count = ARRAY_SIZE(formats);
	return formats;
}

int picapdc_fb_bind_to_source(struct f_fdb_child *fdb_child,
						struct f_fdb_child *fdb_bound)
{
	struct picapdc_par *par = f_par_from_fdb_child(fdb_child);

	par->source = fdb_bound;

	return 0;
}

static irqreturn_t picapdcfb_isr(int irq, void *arg)
{
	struct picapdc_par *par = arg;
	void __iomem *crtc = &par->base[0];
	u32 status = __raw_readl(crtc + PICAPDC_STATUS_OFS);

	if (fdb_getval(status, PICAPDC_STATUS, V_INT)) {
		par->frame_index++;
		if (par->source && par->source->ops->sync)
			par->source->ops->sync(par->source, par->id,
							par->frame_index);
		if (par->completion_pending) {
			complete(&par->completion);
			par->completion_pending = false;
		}

		fdb_setl(crtc, PICAPDC_SWAP, INT_CLR_V, 1);
	}

	if (fdb_getval(status, PICAPDC_STATUS, ERROR)) {
		dev_info(par->dev, "ERR interrupt\n");
		fdb_setl(crtc, PICAPDC_SWAP, INT_CLR_ERR, 1);
	}

	return IRQ_HANDLED;
}

static int picapdcfb_fb_pan_display(struct fb_var_screeninfo *var,
							struct fb_info *info)
{
	struct picapdc_par *par = info->par;
	int bytes_pp = (var->bits_per_pixel + 7) / 8;
	void __iomem *crtc = &par->base[0];
	dma_addr_t paddr;
	int ret;

	if (par->completion_pending) {
		dev_err(par->dev, "flip overrun\n");
		return 0;
	}

	paddr = par->fbpaddr + (var->xoffset * bytes_pp) +
				(var->yoffset * var->xres_virtual * bytes_pp);
	__raw_writel(paddr, crtc + PICAPDC_ADR_FB0_OFS);
	__raw_writel(paddr, crtc + PICAPDC_ADR_FB1_OFS);

	init_completion(&par->completion);
	par->completion_pending = true;
	ret = wait_for_completion_timeout(&par->completion,
							msecs_to_jiffies(50));
	par->completion_pending = false;
	if (ret <= 0) {
		dev_err(par->dev, "fb_pan_display timeout\n");
		return -ETIME;
	}

	return 0;
}

static int picapdcfb_fb_check_var(struct fb_var_screeninfo *var,
							struct fb_info *info)
{
	return 0;
}

struct f_fdb_ops picapdc_fb_fdb_ops = {
	.enable = picapdc_fb_fdb_enable,
	.disable = picapdc_fb_fdb_disable,
	.set_paddr = picapdc_fb_fdb_set_paddr,
	.get_timings = picapdc_fb_get_timings,
	.set_timings = picapdc_fb_set_timings,
	.check_timings = picapdc_fb_check_timings,
	.get_gamma = picapdc_fb_get_gamma,
	.set_gamma = picapdc_fb_set_gamma,
	.get_fdb_formats = picapdc_get_fdb_formats,
	.bind_to_source = picapdc_fb_bind_to_source,
};

static struct fb_fix_screeninfo picapdcfb_fix = {
	.id =		"picapdcfb",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.xpanstep =	1,
	.ypanstep =	1,
	.ywrapstep =	1,
	.accel =	FB_ACCEL_NONE,
};

static int picapdcfb_fb_ioctl(struct fb_info *info, unsigned int cmd,
			unsigned long arg)
{
	struct picapdc_par *par = info->par;
	int ret;

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		if (par->completion_pending)
			return 0;
		init_completion(&par->completion);
		par->completion_pending = true;
		ret = wait_for_completion_timeout(&par->completion,
							msecs_to_jiffies(50));
		par->completion_pending = false;
		if (ret <= 0) {
			dev_err(par->dev, "WAITFORVSYNC timeout\n");
			return -ETIME;
		}
		return 0;
	}

	return -ENOTTY;
}

static struct fb_ops picapdcfb_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= picapdcfb_setcolreg,
	.fb_blank	= picapdcfb_blank,
	.fb_pan_display = picapdcfb_fb_pan_display,
	.fb_check_var	= picapdcfb_fb_check_var,
	.fb_set_par	= picapdcfb_fb_set_par,
	.fb_ioctl	= picapdcfb_fb_ioctl,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

static int picapdcfb_probe(struct platform_device *pdev)
{
	struct fb_info *info;
	struct picapdc_par *par;
	struct resource *res;
	int ret = 0;
	const char *mode;
	const int *p;

	info = framebuffer_alloc(sizeof(struct fb_info), &pdev->dev);
	if (!info) {
		dev_err(&pdev->dev, "Failed to alloc framebuffer\n");
		return -ENOMEM;
	}
	par = info->par;
	memset(par, 0, sizeof(*par));
	mutex_init(&par->lock);
	platform_set_drvdata(pdev, info);
	info->fbops = &picapdcfb_ops;
	info->fix = picapdcfb_fix;
	info->dev = &pdev->dev;
	par->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Missing base resource\n");
		return -EINVAL;
	}

	/* crtc register map */
	par->base = ioremap(res->start, res->end - res->start);

	if (of_property_read_string(pdev->dev.of_node, "mode", &mode)) {
		dev_err(&pdev->dev, "Missing mode\n");
		return -EINVAL;
	}

	p = of_get_property(pdev->dev.of_node, "id", NULL);
	if (p)
		par->id = be32_to_cpu(*p);
	else
		par->id = 0;

	p = of_get_property(pdev->dev.of_node, "simple", NULL);
	if (p)
		par->simple = !!be32_to_cpu(*p);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res)
		dev_info(&pdev->dev, "Missing interrupt resource\n");
	else
		par->irq = res->start;

	ret = request_irq(par->irq, picapdcfb_isr, IRQF_TRIGGER_RISING,
			pdev->dev.driver->name, par);
	if (ret) {
		dev_err(&pdev->dev, "failed to allocate irq.\n");
		goto bail2;
	}

	/* register our fdb_child with f_fdb bus */
	ret = fdb_register(&pdev->dev, &par->fdb_child, &picapdc_fb_fdb_ops);
	if (ret < 0) {
		dev_err(&pdev->dev, "fdb registration failed\n");
		goto bail3;
	}

	if (par->simple) {
		info->pseudo_palette = par->pseudo_palette;
		ret = fb_alloc_cmap(&info->cmap, 256, 0);
		if (ret) {
			dev_err(&pdev->dev, "Failed to alloc cmap\n");
			goto bail4;
		}
		fb_set_cmap(&info->cmap, info);

		picapdc_allocate_backing_set_var(info, mode);

		ret = register_framebuffer(info);
		if (ret < 0) {
			dev_err(&pdev->dev, "framebuffer reg failed %d\n", ret);
			goto bail5;
		}
	}

	if (par->simple)
		dev_info(&pdev->dev, "fb%d: %s frame buffer device\n",
						info->node, info->fix.id);
	else
		dev_info(&pdev->dev,
			"picapdc fb%d hardware registered on fdb\n", par->id);

	return 0;

bail5:
	if (par->simple)
		fb_dealloc_cmap(&info->cmap);
bail4:
	fdb_unregister(&pdev->dev, &par->fdb_child);
bail3:
	free_irq(par->irq, par);
bail2:
	iounmap(par->base);
	framebuffer_release(info);

	return ret;
}

static int picapdcfb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct picapdc_par *par = info->par;

	if (!info)
		return 0;

	free_irq(par->irq, par);

	fdb_unregister(&pdev->dev, &par->fdb_child);

	if (par->simple) {
		unregister_framebuffer(info);
		fb_dealloc_cmap(&info->cmap);
	}
	if (par->fb_va)
		dma_free_coherent(&pdev->dev, par->framesize,
						par->fb_va, par->fbpaddr);
	iounmap(par->base);
	framebuffer_release(info);

	return 0;
}

#ifdef CONFIG_PM
static int picapdcfb_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct picapdc_par *par = info->par;

	picapdc_fb_fdb_disable(&par->fdb_child);

	return 0;
}

static int picapdcfb_resume(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct picapdc_par *par = info->par;

	picapdc_fb_fdb_enable(&par->fdb_child);

	return 0;
}
#else
#define picapdcfb_suspend NULL
#define picapdcfb_resume NULL
#endif /* CONFIG_PM */

static const struct of_device_id picapdc_fb_dt_ids[] = {
	{ .compatible = "dmp,picapdc-fb" },
	{ /* sentinel */ }
};

static struct platform_driver picapdcfb_driver = {
	.probe = picapdcfb_probe,
	.remove = picapdcfb_remove,
	.suspend = picapdcfb_suspend,
	.resume = picapdcfb_resume,
	.driver = {
		.name = "picapdcfb",
		.of_match_table = picapdc_fb_dt_ids,
	},
};

MODULE_DEVICE_TABLE(of, picapdc_fb_dt_ids);

static int __init picapdcfb_init(void)
{
	return platform_driver_register(&picapdcfb_driver);
}

static void __exit picapdcfb_exit(void)
{
	platform_driver_unregister(&picapdcfb_driver);
}

module_init(picapdcfb_init);
module_exit(picapdcfb_exit);

MODULE_LICENSE("GPL");
