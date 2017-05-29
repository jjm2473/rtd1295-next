/*
 * drivers/video/fbdev/fujitsu/iris-blitter.c
 *
 * Copyright (C) 2013 - 2014 Linaro, Ltd for Fujitsu Semiconductor, Ltd
 * Author: Vivien Kuan <vivien.kuan@linaro.org>,
 *         Andy Green <andy.green@linaro.org>
 */

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
#include <linux/workqueue.h>

#include <video/mb86s70-fb.h>
#include <video/fdb.h>

/* #define DEBUG */
#ifdef DEBUG
static void _iris_dump_blit(const char *name,
			    struct fdb_blit *b, struct device *dev)
{
	dev_info(dev, "Blitter struct dump: %s\n", name);
	dev_info(dev, " stride_in: %u\n", b->stride_in);
	dev_info(dev, " stride_out: %u\n", b->stride_out);
	dev_info(dev, " width_in: %u x height_in: %u\n",
		 b->width_in, b->height_in);
	dev_info(dev, " width_out: %u x height_out: %u\n",
		 b->width_out, b->height_out);
	dev_info(dev, " bits_per_pixel_in: %u\n", b->bits_per_pixel_in);
	dev_info(dev, " bits_per_pixel_out: %u\n", b->bits_per_pixel_out);
	dev_info(dev, " fill_pixel: 0x%x\n", b->fill_pixel);
	dev_info(dev, " rotate: 0x%x\n", b->rotate);
	dev_info(dev, " src_x_offset: %u x src_y_offset: %u\n",
		 b->src_x_offset, b->src_y_offset);
	dev_info(dev, " dest_x_offset: %u x dest_y_offset: %u\n",
		 b->dest_x_offset, b->dest_y_offset);
	dev_info(dev, " format_in: %u\n", b->format_in);
	dev_info(dev, " format_out: %u\n", b->format_out);
	dev_info(dev, " blend: %u\n", b->blend);
	dev_info(dev, " src_addr: 0x%lx\n", (unsigned long)b->src_addr);
	dev_info(dev, " dest_addr: 0x%lx\n", (unsigned long)b->dest_addr);
}
#else
#define _iris_dump_blit(x, y, z)
#endif

enum {
	IRIS_BOC_NO_OVERLAP,
	IRIS_BOC_OVERLAPPING,
	IRIS_BOC_PERFECT_OVERLAP,
};

/* one quadrant of sine / cos, each covers 8 bits in 14-bit quadrant-space */

unsigned short sin_quad16_65[] = {
	0x0000,  0x0648,  0x0c8f,  0x12d5,  0x1917,  0x1f56,  0x258f,  0x2bc3,
	0x31f1,  0x3816,  0x3e33,  0x4447,  0x4a4f,  0x504d,  0x563e,  0x5c21,
	0x61f7,  0x67bd,  0x6d73,  0x7319,  0x78ac,  0x7e2e,  0x839b,  0x88f5,
	0x8e39,  0x9367,  0x987f,  0x9d7f,  0xa266,  0xa735,  0xabea,  0xb085,
	0xb504,  0xb967,  0xbdae,  0xc1d7,  0xc5e3,  0xc9d0,  0xcd9e,  0xd14c,
	0xd4da,  0xd847,  0xdb93,  0xdebd,  0xe1c4,  0xe4a9,  0xe76a,  0xea08,
	0xec82,  0xeed7,  0xf108,  0xf313,  0xf4f9,  0xf6b9,  0xf852,  0xf9c6,
	0xfb13,  0xfc3a,  0xfd39,  0xfe12,  0xfec3,  0xff4d,  0xffb0,  0xffeb,
	0xffff
};

static int32_t iris_sin(int16_t iris_angle)
{
	int quadrant = (iris_angle >> 14) & 3;
	int pos, res;

	if (quadrant & 1)
		iris_angle ^= 0xffff;

	pos = (iris_angle >> 8) & (64 - 1);
	res = sin_quad16_65[pos] + (((sin_quad16_65[pos + 1] -
			sin_quad16_65[pos]) * (iris_angle & 0xff)) / 0xff);

	if (quadrant & 2)
		return -res / 2;

	return res / 2;
}



#define IRIS_D90 0x4000
#define IRIS_D180 0x8000
#define IRIS_ANGLE_0DEGREES 0xffff8000

/* ie cos = sin retarded by 90 degrees */

#define iris_cos(_a) iris_sin((_a) - IRIS_D90)

static int iris_hyp(int x, int y)
{
	return int_sqrt((x * x) + (y * y));
}

/* convert fractional int to rounded integer part */

static int iris_trig_round(int n)
{
	return (n + (1 << 14)) >> 15;
}

static void minmax4(int a, int b, int c, int d, int *min, int *max)
{
	if (a < b)
		*min = a;
	else
		*min = b;
	if (c < *min)
		*min = c;
	if (d < *min)
		*min = d;

	if (a > b)
		*max = a;
	else
		*max = b;
	if (c > *max)
		*max = c;
	if (d > *max)
		*max = d;
}

struct rotator {
	int deltaXX_f16;
	int deltaXY_f16;
	int deltaYX_f16;
	int deltaYY_f16;
	int output_extent_x;
	int output_extent_y;
	int scan_offset_x_f16;
	int scan_offset_y_f16;
};

static void iris_rotator_init(int angle, int w, int h, struct rotator *ro)
{
	int radius = iris_hyp(w / 2, h / 2);
	int aspect_angle = (IRIS_D90 * w) / (w + h);
	int origin_angle = aspect_angle - angle;
	int rotx_from_c_f16 = radius * iris_sin(angle + aspect_angle);
	int roty_from_c_f16 = -radius * iris_cos(angle + aspect_angle);
	int tr_x_f16 = radius * iris_sin(angle - aspect_angle);
	int tr_y_f16 = -radius * iris_cos(angle - aspect_angle);
	int bl_x_f16 = radius * iris_sin(angle + aspect_angle +
		       2 * (IRIS_D90 - aspect_angle));
	int bl_y_f16 = -radius * iris_cos(angle + aspect_angle +
		       2 * (IRIS_D90 - aspect_angle));
	int opp_x_f16 = radius * iris_sin(angle + aspect_angle + IRIS_D180);
	int opp_y_f16 = -radius * iris_cos(angle + aspect_angle + IRIS_D180);
	int min_x, max_x, min_y, max_y;
	int bound_hyp;

	minmax4(tr_x_f16, tr_x_f16, rotx_from_c_f16, opp_x_f16, &min_x, &max_x);
	minmax4(tr_y_f16, tr_y_f16, roty_from_c_f16, opp_y_f16, &min_y, &max_y);

	ro->output_extent_x = iris_trig_round(max_x - min_x);
	ro->output_extent_y = iris_trig_round(max_y - min_y);
	bound_hyp = iris_hyp(ro->output_extent_x / 2, ro->output_extent_y / 2);

	ro->scan_offset_x_f16 = (radius * iris_sin(origin_angle) -
				 radius * iris_sin(-0x8000 + aspect_angle)) * 2;
	ro->scan_offset_y_f16 = (-radius * iris_cos(origin_angle) +
				 radius * iris_cos(-0x8000 + aspect_angle)) * 2;

	/* 2 x adjusts for 17.15-> 16.16 fixed computations */

	ro->deltaXX_f16 = (tr_x_f16 - rotx_from_c_f16) / (w >> 3);
	ro->deltaXY_f16 = -(tr_y_f16 - roty_from_c_f16) / (w >> 3);
	ro->deltaYX_f16 = -(bl_x_f16 - rotx_from_c_f16) / (h >> 3);
	ro->deltaYY_f16 = (bl_y_f16 - roty_from_c_f16) / (h >> 3);

	pr_debug("angle=%d, so = %d, %d XX=%d, XY=%d, YX=%d, YY=%d\n",
				(s16)angle,
				ro->scan_offset_x_f16 >> 16,
				ro->scan_offset_y_f16 >> 16,
				ro->deltaXX_f16, ro->deltaXY_f16,
				ro->deltaYX_f16, ro->deltaYY_f16);
}


static int
_iris_check_overlap(struct fdb_blit *b, struct device *dev)
{
	/*
	 * only detects overlap when src and dst PA are the
	 * same (eg, copy to and from same framebuffer like window drag)
	 */
	if (b->src_addr != b->dest_addr)
		return IRIS_BOC_NO_OVERLAP;

	if (b->src_x_offset == b->dest_x_offset &&
	    b->src_y_offset == b->dest_y_offset)
		return IRIS_BOC_PERFECT_OVERLAP; /* src == dest completely */

	/* source is entirely to the right of the dest */
	if (b->src_x_offset >= (b->dest_x_offset + b->width_out))
		return IRIS_BOC_NO_OVERLAP;

	/* source is entirely to the left of the dest */
	if ((b->src_x_offset + b->width_in) <= b->dest_x_offset)
		return IRIS_BOC_NO_OVERLAP;

	/* source is entirely below the dest */
	if (b->src_y_offset >= (b->dest_y_offset + b->height_out))
		return IRIS_BOC_NO_OVERLAP;

	/* source is entirely above the dest */
	if ((b->src_y_offset + b->height_in) <= b->dest_y_offset)
		return IRIS_BOC_NO_OVERLAP;

	return IRIS_BOC_OVERLAPPING;
}

/* pipeline settings & shadow */
static void
pipeline_shadow(struct fdb_blit *b, void __iomem *base, enum blitter_config pl)
{
	u32 val = 0;

	/* disposition of fetch2 */
	if (pl & (IBC_F2_INTO_ROP0 | IBC_F2_INTO_BITBLEND_SEC)) {
		writel(fdb_const(PXE_F2_STATCTL, CLOCKDISABLE, 0) |
		       fdb_const(PXE_F2_STATCTL, SWRESET, 0) |
		       fdb_const(PXE_F2_STATCTL, SHDEN, 1),
		       base + PXE_F2_STATCTL_OFS);
		writel(fdb_const(PXE_PXB_FETCH2_CFG, FETCH2_SHDW, 1),
		       base + PXE_PXB_FETCH2_CFG_OFS);
	} else
		writel(fdb_const(PXE_F2_STATCTL, CLOCKDISABLE, 1) |
		       fdb_const(PXE_F2_STATCTL, SWRESET, 1) |
		       fdb_const(PXE_F2_STATCTL, SHDEN, 0),
		       base + PXE_F2_STATCTL_OFS);

	/* disposition of ROP0 */

	writel(fdb_const(PXENG_ROP0_STATCTL, SHDEN, 1),
	       base + PXENG_ROP0_STATCTL_OFS);
	val = fdb_const(PXE_PXB_ROP0_CFG, ROP0_SHDW, 1) |
	      fdb_const(PXE_PXB_ROP0_CFG, ROP0_CLKEN, 1);

	switch (pl & (IBC_F0_INTO_CLUT0_ROP0 | IBC_F1_INTO_ROP0 |
		      IBC_F2_INTO_ROP0)) {
	case IBC_F0_INTO_CLUT0_ROP0:
		val |= fdb_const(PXE_PXB_ROP0_CFG,
				ROP0_TERT_SEL, IBCPS_DISABLED) |
		       fdb_const(PXE_PXB_ROP0_CFG,
				ROP0_SEC_SEL, IBCPS_DISABLED) |
		       fdb_const(PXE_PXB_ROP0_CFG,
				ROP0_PRIM_SEL, IBCPS_CLUT0);
		break;
	case IBC_F2_INTO_ROP0:
		val |= fdb_const(PXE_PXB_ROP0_CFG,
				ROP0_TERT_SEL, IBCPS_DISABLED) |
		       fdb_const(PXE_PXB_ROP0_CFG,
				ROP0_SEC_SEL, IBCPS_DISABLED) |
		       fdb_const(PXE_PXB_ROP0_CFG,
				ROP0_PRIM_SEL, IBCPS_F2);
		break;
	case IBC_F0_INTO_CLUT0_ROP0 | IBC_F1_INTO_ROP0 | IBC_F2_INTO_ROP0:
		val |= fdb_const(PXE_PXB_ROP0_CFG, ROP0_TERT_SEL, IBCPS_CLUT0) |
		       fdb_const(PXE_PXB_ROP0_CFG, ROP0_SEC_SEL, IBCPS_F1) |
		       fdb_const(PXE_PXB_ROP0_CFG, ROP0_PRIM_SEL, IBCPS_F2);
		break;
	case IBC_F0_INTO_CLUT0_ROP0 | IBC_F2_INTO_ROP0:
		val |= fdb_const(PXE_PXB_ROP0_CFG, ROP0_TERT_SEL, IBCPS_CLUT0) |
		       fdb_const(PXE_PXB_ROP0_CFG, ROP0_SEC_SEL,
							IBCPS_DISABLED) |
		       fdb_const(PXE_PXB_ROP0_CFG, ROP0_PRIM_SEL, IBCPS_F2);
		break;
	case 0:
		val = fdb_const(PXE_PXB_ROP0_CFG, ROP0_CLKEN, 0) |
		      fdb_const(PXE_PXB_ROP0_CFG, ROP0_TERT_SEL,
							IBCPS_DISABLED) |
		      fdb_const(PXE_PXB_ROP0_CFG, ROP0_SEC_SEL,
							IBCPS_DISABLED) |
		      fdb_const(PXE_PXB_ROP0_CFG, ROP0_PRIM_SEL,
							IBCPS_DISABLED);
		break;
	default:
		pr_err("unknown ROP0 setup\n");
		break;
	}
	writel(val, base + PXE_PXB_ROP0_CFG_OFS);

	/* H/V scaler */
	if (pl & IBC_ROP_INTO_SCALER) {
		writel(fdb_const(PXE_PXB_HSCALER0_CFG, HSCALER0_CLKEN, 1) |
		       fdb_const(PXE_PXB_HSCALER0_CFG, HSCALER0_SRC_SEL,
								IBCPS_ROP0),
			base + PXE_PXB_HSCALER0_CFG_OFS);
		writel(fdb_const(PXE_PXB_VSCALER0_CFG, VSCALER0_CLKEN, 1) |
		       fdb_const(PXE_PXB_VSCALER0_CFG, VSCALER0_SRC_SEL,
								IBCPS_HSCALER0),
			base + PXE_PXB_VSCALER0_CFG_OFS);
		writel(fdb_const(PXENG_HSCALER0_STATCTL, SHDEN, 1),
		       base + PXENG_HSCALER0_STATCTL_OFS);
		writel(fdb_const(PXENG_VSCALER0_STATCTL, SHDEN, 1),
		       base + PXENG_VSCALER0_STATCTL_OFS);
	} else {
		writel(fdb_const(PXE_PXB_HSCALER0_CFG, HSCALER0_CLKEN, 0) |
		       fdb_const(PXE_PXB_HSCALER0_CFG, HSCALER0_SRC_SEL, 0),
		       base + PXE_PXB_HSCALER0_CFG_OFS);
		writel(fdb_const(PXE_PXB_VSCALER0_CFG, VSCALER0_CLKEN, 0) |
		       fdb_const(PXE_PXB_VSCALER0_CFG, VSCALER0_SRC_SEL, 0),
		       base + PXE_PXB_VSCALER0_CFG_OFS);
	}

	/* BlitBlend#0 */
	switch (pl &
		(IBC_F2_INTO_BITBLEND_SEC | IBC_MATRIX_INTO_BITBLEND_PRI)) {
	case 0:
		writel(fdb_const(PXE_PXB_BLITBLEND0_CFG, BLITBLEND0_CLKEN, 0) |
		       fdb_const(PXE_PXB_BLITBLEND0_CFG,
							BLITBLEND0_SEC_SEL, 0) |
		       fdb_const(PXE_PXB_BLITBLEND0_CFG,
							BLITBLEND0_PRIM_SEL, 0),
		       base + PXE_PXB_BLITBLEND0_CFG_OFS);
		break;
	case IBC_F2_INTO_BITBLEND_SEC | IBC_MATRIX_INTO_BITBLEND_PRI:
		writel(fdb_const(PXENG_BLITBLEND0_STATCTL, SHDEN, 1),
		       base + PXENG_BLITBLEND0_STATCTL_OFS);
			writel(fdb_const(PXE_PXB_BLITBLEND0_CFG,
					 BLITBLEND0_SHDW, 1) |
			       fdb_const(PXE_PXB_BLITBLEND0_CFG,
					 BLITBLEND0_CLKEN, 1) |
			       fdb_const(PXE_PXB_BLITBLEND0_CFG,
					 BLITBLEND0_SEC_SEL, IBCPS_F2) |
			       fdb_const(PXE_PXB_BLITBLEND0_CFG,
					 BLITBLEND0_PRIM_SEL, IBCPS_MATRIX0),
			       base + PXE_PXB_BLITBLEND0_CFG_OFS);
		break;
	case IBC_MATRIX_INTO_BITBLEND_PRI:
		writel(fdb_const(PXENG_BLITBLEND0_STATCTL, SHDEN, 1),
				base + PXENG_BLITBLEND0_STATCTL_OFS);
		writel(fdb_const(PXE_PXB_BLITBLEND0_CFG, BLITBLEND0_SHDW, 1) |
		       fdb_const(PXE_PXB_BLITBLEND0_CFG, BLITBLEND0_CLKEN, 1) |
		       fdb_const(PXE_PXB_BLITBLEND0_CFG, BLITBLEND0_SEC_SEL,
							IBCPS_DISABLED) |
		       fdb_const(PXE_PXB_BLITBLEND0_CFG, BLITBLEND0_PRIM_SEL,
							IBCPS_MATRIX0),
		       base + PXE_PXB_BLITBLEND0_CFG_OFS);
		break;
	}

	/* CLuT#0 and Matrix#0 */
	if (pl & IBC_F0_INTO_CLUT0_ROP0) {
		writel(fdb_const(PXE_PXB_CLUT0_CFG, CLUT0_SHDW, 1) |
		       fdb_const(PXE_PXB_CLUT0_CFG, CLUT0_SRC_SEL, IBCPS_F0),
		       base + PXE_PXB_CLUT0_CFG_OFS);
		writel(fdb_const(PXENG_CLUT0_STATCTL, SHDEN, 1),
		       base + PXENG_CLUT0_STATCTL_OFS);
		writel(fdb_const(PXENG_MATRIX0_STATCTL, SHDEN, 1),
		       base + PXENG_MATRIX0_STATCTL_OFS);
	}

	if (pl & IBC_SCALER_INTO_MATRIX) {
		writel(fdb_const(PXE_PXB_MATRIX0_CFG, MATRIX0_SHDW, 1) |
		       fdb_const(PXE_PXB_MATRIX0_CFG,
				 MATRIX0_CLKEN, 1) |
		       fdb_const(PXE_PXB_MATRIX0_CFG,
				 MATRIX0_SRC_SEL, IBCPS_VSCALER0),
		       base + PXE_PXB_MATRIX0_CFG_OFS);
	} else
		writel(fdb_const(PXE_PXB_MATRIX0_CFG, MATRIX0_CLKEN, 0) |
		       fdb_const(PXE_PXB_MATRIX0_CFG, MATRIX0_SRC_SEL, 0),
		       base + PXE_PXB_MATRIX0_CFG_OFS);

	/* Fetch#0 */
	if (pl & (IBC_F0_INTO_CLUT0_ROP0 | IBC_F0_INTO_STORE0)) {
		writel(fdb_const(PXE_F0_STATCTL, CLOCKDISABLE, 0) |
		       fdb_const(PXE_F0_STATCTL, SWRESET, 0) |
		       fdb_const(PXE_F0_STATCTL, SHDEN, 1),
		       base + PXE_F0_STATCTL_OFS);
		writel(fdb_const(PXE_PXB_FETCH0_CFG, FETCH0_SHDW, 1),
		       base + PXE_PXB_FETCH0_CFG_OFS);
	} else
		writel(fdb_const(PXE_F0_STATCTL, CLOCKDISABLE, 1) |
		       fdb_const(PXE_F0_STATCTL, SWRESET, 1) |
		       fdb_const(PXE_F0_STATCTL, SHDEN, 0),
		       base + PXE_F0_STATCTL_OFS);

	/* Fetch#1 */
	if (pl & IBC_F1_INTO_ROP0) {
		writel(fdb_const(PXE_F1_STATCTL, CLOCKDISABLE, 0) |
		       fdb_const(PXE_F1_STATCTL, SWRESET, 0) |
		       fdb_const(PXE_F1_STATCTL, SHDEN, 1),
		       base + PXE_F1_STATCTL_OFS);
		writel(fdb_const(PXE_PXB_FETCH1_CFG, FETCH1_SHDW, 1),
		       base + PXE_PXB_FETCH1_CFG_OFS);
	} else
		writel(fdb_const(PXE_F1_STATCTL, CLOCKDISABLE, 1) |
		       fdb_const(PXE_F1_STATCTL, SWRESET, 1) |
		       fdb_const(PXE_F1_STATCTL, SHDEN, 0),
		       base + PXE_F1_STATCTL_OFS);

	/* Store#0 */
	switch (pl & (IBC_F0_INTO_STORE0 | IBC_BITBLEND_INTO_STORE0 |
		      IBC_SCALER_INTO_STORE0)) {
	case IBC_SCALER_INTO_STORE0:
		writel(fdb_const(PXE_PXB_STORE0_CFG, STORE0_SHDW, 1) |
		       fdb_const(PXE_PXB_STORE0_CFG, STORE0_SRC_SEL,
							IBCPS_VSCALER0),
		       base + PXE_PXB_STORE0_CFG_OFS);
		break;
	case IBC_F0_INTO_STORE0:
		writel(fdb_const(PXE_PXB_STORE0_CFG, STORE0_SHDW, 1) |
		       fdb_const(PXE_PXB_STORE0_CFG, STORE0_SRC_SEL, 1),
		       base + PXE_PXB_STORE0_CFG_OFS);
		break;
	case IBC_BITBLEND_INTO_STORE0:
		writel(fdb_const(PXE_PXB_STORE0_CFG, STORE0_SHDW, 1) |
		       fdb_const(PXE_PXB_STORE0_CFG, STORE0_SRC_SEL,
							IBCPS_BITBLEND0),
		       base + PXE_PXB_STORE0_CFG_OFS);
		break;
	default:
		writel(fdb_const(PXE_S0_STATCTL, CLOCKDISABLE, 0) |
			fdb_const(PXE_S0_STATCTL, SWRESET, 0) |
			fdb_const(PXE_S0_STATCTL, SHDEN, 1),
				base + PXE_S0_STATCTL_OFS);
		break;
	}
}

/* blit unit settings: format in/out with Fetch#0,1,2 */
static void blit_format(struct fdb_blit *b, void __iomem *base)
{
	u32 shift_r, shift_g, shift_b, shift_a;

	switch (b->format_in) {
	case FBF_ARGB8888:
	case FBF_ABGR8888:
	case FBF_BGRA8888:
	case FBF_RGBA8888:
		switch (b->format_in) {
		case FBF_ARGB8888:
			shift_r = 16;
			shift_g = 8;
			shift_b = 0;
			shift_a = 24;
			break;

		case FBF_ABGR8888:
			shift_r = 0;
			shift_g = 8;
			shift_b = 16;
			shift_a = 24;
			break;

		case FBF_BGRA8888:
			shift_r = 8;
			shift_g = 16;
			shift_b = 24;
			shift_a = 0;
			break;

		default: /* FBF_RGBA8888 */
			shift_r = 24;
			shift_g = 16;
			shift_b = 8;
			shift_a = 0;
			break;
		}

		writel(fdb_const(PXE_F0_COLCOMPBITS, BITSRED, 8) |
		       fdb_const(PXE_F0_COLCOMPBITS, BITSGREEN, 8) |
		       fdb_const(PXE_F0_COLCOMPBITS, BITSBLUE, 8) |
		       fdb_const(PXE_F0_COLCOMPBITS, BITSALPHA, 8),
		       base + PXE_F0_COLCOMPBITS_OFS);
		writel(fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTRED, shift_r) |
		       fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTGREEN, shift_g) |
		       fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTBLUE, shift_b) |
		       fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTALPHA, shift_a),
		       base + PXE_F0_COLCOMPSHIFT_OFS);
		writel(fdb_const(PXE_F2_COLCOMPBITS, BITSRED, 8) |
		       fdb_const(PXE_F2_COLCOMPBITS, BITSGREEN, 8) |
		       fdb_const(PXE_F2_COLCOMPBITS, BITSBLUE, 8) |
		       fdb_const(PXE_F2_COLCOMPBITS, BITSALPHA, 8),
		       base + PXE_F2_COLCOMPBITS_OFS);
		writel(fdb_const(PXE_F2_COLCOMPSHIFT, SHIFTRED, shift_r) |
		       fdb_const(PXE_F2_COLCOMPSHIFT, SHIFTGREEN, shift_g) |
		       fdb_const(PXE_F2_COLCOMPSHIFT, SHIFTBLUE, shift_b) |
		       fdb_const(PXE_F2_COLCOMPSHIFT, SHIFTALPHA, shift_a),
		       base + PXE_F2_COLCOMPSHIFT_OFS);
		break;
	case FBF_BGR888:
		writel(fdb_const(PXE_F0_COLCOMPBITS, BITSRED, 8) |
		       fdb_const(PXE_F0_COLCOMPBITS, BITSGREEN, 8) |
		       fdb_const(PXE_F0_COLCOMPBITS, BITSBLUE, 8) |
		       fdb_const(PXE_F0_COLCOMPBITS, BITSALPHA, 0),
		       base + PXE_F0_COLCOMPBITS_OFS);
		writel(fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTRED, 0) |
		       fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTGREEN, 8) |
		       fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTBLUE, 16) |
		       fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTALPHA, 0),
		       base + PXE_F0_COLCOMPSHIFT_OFS);
		writel(fdb_const(PXE_F2_COLCOMPBITS, BITSRED, 8) |
		       fdb_const(PXE_F2_COLCOMPBITS, BITSGREEN, 8) |
		       fdb_const(PXE_F2_COLCOMPBITS, BITSBLUE, 8) |
		       fdb_const(PXE_F2_COLCOMPBITS, BITSALPHA, 0),
		       base + PXE_F2_COLCOMPBITS_OFS);
		writel(fdb_const(PXE_F2_COLCOMPSHIFT, SHIFTRED, 0) |
		       fdb_const(PXE_F2_COLCOMPSHIFT, SHIFTGREEN, 8) |
		       fdb_const(PXE_F2_COLCOMPSHIFT, SHIFTBLUE, 16) |
		       fdb_const(PXE_F2_COLCOMPSHIFT, SHIFTALPHA, 0),
		       base + PXE_F2_COLCOMPSHIFT_OFS);
		break;
	case FBF_YUV420_SEPARATE:
	case FBF_YUV422_SEPARATE:
	case FBF_YUV444_SEPARATE:
		writel(fdb_const(PXE_F2_COLCOMPBITS, BITSRED, 8) |
		       fdb_const(PXE_F2_COLCOMPBITS, BITSGREEN, 0) |
		       fdb_const(PXE_F2_COLCOMPBITS, BITSBLUE, 0) |
		       fdb_const(PXE_F2_COLCOMPBITS, BITSALPHA, 0),
		       base + PXE_F2_COLCOMPBITS_OFS);
		writel(fdb_const(PXE_F2_COLCOMPSHIFT, SHIFTRED, 0) |
		       fdb_const(PXE_F2_COLCOMPSHIFT, SHIFTGREEN, 0) |
		       fdb_const(PXE_F2_COLCOMPSHIFT, SHIFTBLUE, 0) |
		       fdb_const(PXE_F2_COLCOMPSHIFT, SHIFTALPHA, 0),
		       base + PXE_F2_COLCOMPSHIFT_OFS);
		writel(fdb_const(PXE_F0_COLCOMPBITS, BITSRED, 0) |
		       fdb_const(PXE_F0_COLCOMPBITS, BITSGREEN, 8) |
		       fdb_const(PXE_F0_COLCOMPBITS, BITSBLUE, 0) |
		       fdb_const(PXE_F0_COLCOMPBITS, BITSALPHA, 0),
		       base + PXE_F0_COLCOMPBITS_OFS);
		writel(fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTRED, 0) |
		       fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTGREEN, 0) |
		       fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTBLUE, 0) |
		       fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTALPHA, 0),
		       base + PXE_F0_COLCOMPSHIFT_OFS);
		writel(fdb_const(PXE_F1_COLCOMPBITS, BITSRED, 0) |
		       fdb_const(PXE_F1_COLCOMPBITS, BITSGREEN, 0) |
		       fdb_const(PXE_F1_COLCOMPBITS, BITSBLUE, 8) |
		       fdb_const(PXE_F1_COLCOMPBITS, BITSALPHA, 0),
		       base + PXE_F1_COLCOMPBITS_OFS);
		writel(fdb_const(PXE_F1_COLCOMPSHIFT, SHIFTRED, 0) |
		       fdb_const(PXE_F1_COLCOMPSHIFT, SHIFTGREEN, 0) |
		       fdb_const(PXE_F1_COLCOMPSHIFT, SHIFTBLUE, 0) |
		       fdb_const(PXE_F1_COLCOMPSHIFT, SHIFTALPHA, 0),
		       base + PXE_F1_COLCOMPSHIFT_OFS);
		break;
	case FBF_YUV420_INTERLEAVE:
		writel(fdb_const(PXE_F2_COLCOMPBITS, BITSRED, 8) |
		       fdb_const(PXE_F2_COLCOMPBITS, BITSGREEN, 0) |
		       fdb_const(PXE_F2_COLCOMPBITS, BITSBLUE, 0) |
		       fdb_const(PXE_F2_COLCOMPBITS, BITSALPHA, 0),
		       base + PXE_F2_COLCOMPBITS_OFS);
		writel(fdb_const(PXE_F2_COLCOMPSHIFT, SHIFTRED, 0) |
		       fdb_const(PXE_F2_COLCOMPSHIFT, SHIFTGREEN, 0) |
		       fdb_const(PXE_F2_COLCOMPSHIFT, SHIFTBLUE, 0) |
		       fdb_const(PXE_F2_COLCOMPSHIFT, SHIFTALPHA, 0),
		       base + PXE_F2_COLCOMPSHIFT_OFS);
		writel(fdb_const(PXE_F0_COLCOMPBITS, BITSRED, 0) |
		       fdb_const(PXE_F0_COLCOMPBITS, BITSGREEN, 8) |
		       fdb_const(PXE_F0_COLCOMPBITS, BITSBLUE, 8) |
		       fdb_const(PXE_F0_COLCOMPBITS, BITSALPHA, 0),
		       base + PXE_F0_COLCOMPBITS_OFS);
		writel(fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTRED, 0) |
		       fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTGREEN, 8) |
		       fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTBLUE, 0) |
		       fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTALPHA, 0),
		       base + PXE_F0_COLCOMPSHIFT_OFS);
		break;
	}

	if (b->bits_per_pixel_in == 1) {
		/* format_in >= FBF_YUV420_INTERLEAVE */

		writel(fdb_const(PXE_F0_COLCOMPBITS, BITSRED, 0) |
		       fdb_const(PXE_F0_COLCOMPBITS, BITSGREEN, 0) |
		       fdb_const(PXE_F0_COLCOMPBITS, BITSBLUE, 0) |
		       fdb_const(PXE_F0_COLCOMPBITS, BITSALPHA, 1),
		       base + PXE_F0_COLCOMPBITS_OFS);
		writel(fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTRED, 0) |
		       fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTGREEN, 0) |
		       fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTBLUE, 0) |
		       fdb_const(PXE_F0_COLCOMPSHIFT, SHIFTALPHA, 0),
		       base + PXE_F0_COLCOMPSHIFT_OFS);
	}

	switch (b->format_out) {
	case FBF_ARGB8888:
		shift_r = 16;
		shift_g = 8;
		shift_b = 0;
		shift_a = 24;
		break;

	case FBF_ABGR8888:
		shift_r = 0;
		shift_g = 8;
		shift_b = 16;
		shift_a = 24;
		break;

	case FBF_BGRA8888:
		shift_r = 8;
		shift_g = 16;
		shift_b = 24;
		shift_a = 0;
		break;

	default: /* FBF_RGBA8888 */
		shift_r = 24;
		shift_g = 16;
		shift_b = 8;
		shift_a = 0;
		break;
	}

	/* out color format */
	writel(fdb_const(PXE_S0_COLCOMPBITS, BITSRED, 8) |
		   fdb_const(PXE_S0_COLCOMPBITS, BITSGREEN, 8) |
		   fdb_const(PXE_S0_COLCOMPBITS, BITSBLUE, 8) |
		   fdb_const(PXE_S0_COLCOMPBITS, BITSALPHA, 8),
		   base + PXE_S0_COLCOMPBITS_OFS);
	writel(fdb_const(PXE_S0_COLCOMPSHIFT, SHIFTRED, shift_r) |
		   fdb_const(PXE_S0_COLCOMPSHIFT, SHIFTGREEN, shift_g) |
		   fdb_const(PXE_S0_COLCOMPSHIFT, SHIFTBLUE, shift_b) |
		   fdb_const(PXE_S0_COLCOMPSHIFT, SHIFTALPHA, shift_a),
		   base + PXE_S0_COLCOMPSHIFT_OFS);
}

/* blit unit settings: Fetch#2 */
static void fetch2_other(struct fdb_blit *b, void __iomem *base,
						enum blitter_config pl)
{
	int h = b->height_in, w = b->width_in, s = b->stride_in,
		bpp = b->bits_per_pixel_in;
	struct rotator ro;
	u32 dest_addr = b->dest_addr + b->dest_y_offset * b->stride_out +
		b->dest_x_offset * ((b->bits_per_pixel_out + 7) / 8);
	u32 src_addr = b->src_addr + b->src_y_offset * b->stride_in +
		b->src_x_offset * ((b->bits_per_pixel_in + 7) / 8);

	/* adjust PAs for offsets of region starts */

	if (!(pl & (IBC_F2_INTO_ROP0 | IBC_F2_INTO_BITBLEND_SEC)))
		return;

	if (b->blend) {
		h = b->height_out;
		w = b->width_out;
		s = b->stride_out;
		bpp = b->bits_per_pixel_out;
		writel(dest_addr, base + PXE_F2_BASEADDRESS_OFS);
	} else
		writel(src_addr, base + PXE_F2_BASEADDRESS_OFS);

	/* fetch 2 is the Y of YUV */
	
	if (b->format_in >= FBF_YUV420_INTERLEAVE) {
		writel(fdb_const(PXE_F2_CTL, FILTERMODE, 0) |
		       fdb_const(PXE_F2_CTL, BITSPERPIXEL, 8),
		       base + PXE_F2_CTL_OFS);
		writel(b->src_addr + (b->src_y_offset * b->stride_in) + b->src_x_offset,
		       base + PXE_F2_BASEADDRESS_OFS);
	} else
		writel(fdb_const(PXE_F2_CTL, FILTERMODE, 0) |
		       fdb_const(PXE_F2_CTL, BITSPERPIXEL, bpp) |
		       fdb_const(PXE_F2_CTL, TILEMODE, 0), /* constant colour */
		       base + PXE_F2_CTL_OFS);

	writel(fdb_const(PXE_F2_FRAMEDIM, FRAMEHEIGHT, h - 1) |
	       fdb_const(PXE_F2_FRAMEDIM, FRAMEWIDTH, w - 1),
	       base + PXE_F2_FRAMEDIM_OFS);
	writel(fdb_const(PXE_F2_SRCBUFATTR, LINECOUNT, h - 1) |
	       fdb_const(PXE_F2_SRCBUFATTR, LINEWIDTH, w - 1),
	       base + PXE_F2_SRCBUFATTR_OFS);
	if (!b->blend && !(pl & IBC_YUV_SOURCE) && b->rotate) {
		iris_rotator_init(b->rotate - 0x8000, w, h, &ro);
		/* 7.18 fixed */
		writel(fdb_const(PXE_F2_DELTAXY, DELTAXY, ro.deltaXY_f16),
				base + PXE_F2_DELTAXY_OFS);
		writel(fdb_const(PXE_F2_DELTAYX, DELTAYX, ro.deltaYX_f16),
				base + PXE_F2_DELTAYX_OFS);
		writel(fdb_const(PXE_F2_DELTAXX, DELTAXX, ro.deltaXX_f16),
				base + PXE_F2_DELTAXX_OFS);
		writel(fdb_const(PXE_F2_DELTAYY, DELTAYY, ro.deltaYY_f16),
				base + PXE_F2_DELTAYY_OFS);
		/* 16.5 fixed in 16.16 layout register */
		writel(ro.scan_offset_x_f16, base + PXE_F2_FRAMEXOFFSET_OFS);
		writel(ro.scan_offset_y_f16, base + PXE_F2_FRAMEYOFFSET_OFS);
	} else {
		writel(fdb_const(PXE_F2_DELTAXY, DELTAXY, 0),
				base + PXE_F2_DELTAXY_OFS);
		writel(fdb_const(PXE_F2_DELTAYX, DELTAYX, 0),
				base + PXE_F2_DELTAYX_OFS);
		writel(fdb_const(PXE_F2_DELTAXX, DELTAXX, 0x40000),
				base + PXE_F2_DELTAXX_OFS);
		writel(fdb_const(PXE_F2_DELTAYY, DELTAYY, 0x40000),
				base + PXE_F2_DELTAYY_OFS);
		writel(0, base + PXE_F2_FRAMEXOFFSET_OFS);
		writel(0, base + PXE_F2_FRAMEYOFFSET_OFS);
	}
	writel(fdb_const(PXE_F2_SRCBUFSTRIDE, STRIDE, s - 1),
	       base + PXE_F2_SRCBUFSTRIDE_OFS);
}

/* blit unit settings: Fetch#0 */
static void fetch0_other(struct fdb_blit *b, void __iomem *base,
						enum blitter_config pl)
{
	int h = b->height_out, w = b->width_out, s = b->stride_out;
	u32 ctrl_mask = 0;
	u32 src_addr = b->src_addr + b->src_y_offset * b->stride_in +
		b->src_x_offset * ((b->bits_per_pixel_in + 7) / 8);
	u32 mask_addr = 0;
	u32 shift_r, shift_g, shift_b, shift_a;

	if (b->mask_addr)
		mask_addr = b->mask_addr + b->mask_y_offset * b->stride_mask +
			b->mask_x_offset * ((b->bits_per_pixel_in + 7) / 8);

	if (!(pl & (IBC_F0_INTO_CLUT0_ROP0 | IBC_F0_INTO_STORE0)))
		return;

	if (b->source_alpha && (b->fill_pixel >> 24) != 0xff)
		ctrl_mask = fdb_const(PXE_F0_CTL, ALPHA_MULTIPLY, 1);

	switch (b->format_in) {
	case FBF_ABGR8888:
		shift_r = 0;
		shift_g = 8;
		shift_b = 16;
		shift_a = 24;
		break;

	case FBF_BGRA8888:
		shift_r = 8;
		shift_g = 16;
		shift_b = 24;
		shift_a = 0;
		break;

	case FBF_RGBA8888:
		shift_r = 24;
		shift_g = 16;
		shift_b = 8;
		shift_a = 0;
		break;

	default: /* FBF_ARGB8888 */
	 /*
	  * Fill operation is used for 32bpp color.
	  * So, it is not necessary to care about FBF_YUV* and FBF_BGR888
	  */
		shift_r = 16;
		shift_g = 8;
		shift_b = 0;
		shift_a = 24;
		break;
	}

	writel(fdb_const(PXE_F0_CONSTANT_COLOR,
			 RED, (b->fill_pixel >> shift_r) & 0xff) |
	       fdb_const(PXE_F0_CONSTANT_COLOR,
			 GREEN, (b->fill_pixel >> shift_g) & 0xff) |
	       fdb_const(PXE_F0_CONSTANT_COLOR,
			 BLUE, (b->fill_pixel >> shift_b) & 0xff) |
	       fdb_const(PXE_F0_CONSTANT_COLOR,
			 ALPHA, (b->fill_pixel >> shift_a) & 0xff),
	       base + PXE_F0_CONSTANT_COLOR_OFS);

	if (!b->height_in || !b->width_in) { /* Buffer fill */
		writel(fdb_const(PXE_F0_FRAMEXOFS, XOFS_FP2, 0xffff << 2),
		       base + PXE_F0_FRAMEXOFS_OFS);
		writel(fdb_const(PXE_F0_FRAMEYOFS, YOFS_FP2, 0xffff << 2),
		       base + PXE_F0_FRAMEYOFS_OFS);
	} else {
		/* zero the frame offsets */
		writel(fdb_const(PXE_F0_FRAMEXOFS, XOFS_FP2, 0 << 2),
		       base + PXE_F0_FRAMEXOFS_OFS);
		writel(fdb_const(PXE_F0_FRAMEYOFS, YOFS_FP2, 0 << 2),
		       base + PXE_F0_FRAMEYOFS_OFS);
	}

	/* for YUV, F0 gets the U of YUV */
	
	if (b->format_in >= FBF_YUV420_INTERLEAVE) {
		writel(fdb_const(PXE_F0_FRAMEDIM,
				 FRAMEHEIGHT, b->height_in - 1) |
		       fdb_const(PXE_F0_FRAMEDIM,
				 FRAMEWIDTH, b->width_in - 1),
		       base + PXE_F0_FRAMEDIM_OFS);
		switch (b->format_in) {
		case FBF_YUV444_SEPARATE:
			writel(b->src_addr + (b->stride_in * b->height_in) +
			(b->src_y_offset * b->stride_in) + (b->src_x_offset),
				base + PXE_F0_BASEADDRESS_OFS);
			writel(fdb_const(PXE_F0_SRCBUFATTRIBUTES,
					 LINECOUNT, b->height_in - 1) |
			       fdb_const(PXE_F0_SRCBUFATTRIBUTES,
					 LINEWIDTH, b->width_in - 1),
			       base + PXE_F0_SRCBUFATTRIBUTES_OFS);
			writel(fdb_const(PXE_F0_DELTAYY, DELTAY, 1 << 2),
			       base + PXE_F0_DELTAYY_OFS);
			writel(fdb_const(PXE_F0_DELTAXX, DELTAX, 1 << 2),
			       base + PXE_F0_DELTAXX_OFS);
			writel(fdb_const(PXE_F0_CTL, TILEMODE, (!!b->rotate)) |
			       fdb_const(PXE_F0_CTL, BITSPERPIXEL, 8) |
			       ctrl_mask,
			       base + PXE_F0_CTL_OFS);
			break;
		case FBF_YUV422_SEPARATE:
			writel(b->src_addr + (b->stride_in * b->height_in) +
			(b->src_y_offset * b->stride_in / 2) + (b->src_x_offset / 2),
		       base + PXE_F0_BASEADDRESS_OFS);
			writel(fdb_const(PXE_F0_SRCBUFATTRIBUTES,
					 LINECOUNT, b->height_in - 1) |
			       fdb_const(PXE_F0_SRCBUFATTRIBUTES,
					 LINEWIDTH, b->width_in / 2 - 1),
			       base + PXE_F0_SRCBUFATTRIBUTES_OFS);
			writel(fdb_const(PXE_F0_DELTAYY, DELTAY, 1 << 2),
			       base + PXE_F0_DELTAYY_OFS);
			writel(fdb_const(PXE_F0_DELTAXX, DELTAX, 1 << 1),
			       base + PXE_F0_DELTAXX_OFS);
			writel(fdb_const(PXE_F0_CTL, TILEMODE, (!!b->rotate)) |
			       fdb_const(PXE_F0_CTL, BITSPERPIXEL, 8) |
			       ctrl_mask,
			       base + PXE_F0_CTL_OFS);
			break;
		case FBF_YUV420_SEPARATE:
			writel(b->src_addr + (b->stride_in * b->height_in) +
			(b->src_y_offset / 2 * b->stride_in / 2) + (b->src_x_offset / 2),
		       base + PXE_F0_BASEADDRESS_OFS);
			writel(fdb_const(PXE_F0_SRCBUFATTRIBUTES,
					 LINECOUNT, b->height_in / 2 - 1) |
			       fdb_const(PXE_F0_SRCBUFATTRIBUTES,
					 LINEWIDTH, b->width_in / 2 - 1),
			       base + PXE_F0_SRCBUFATTRIBUTES_OFS);
			writel(fdb_const(PXE_F0_DELTAYY, DELTAY, 1 << 1),
			       base + PXE_F0_DELTAYY_OFS);
			writel(fdb_const(PXE_F0_DELTAXX, DELTAX, 1 << 1),
			       base + PXE_F0_DELTAXX_OFS);
			writel(fdb_const(PXE_F0_CTL, TILEMODE, (!!b->rotate)) |
			       fdb_const(PXE_F0_CTL, BITSPERPIXEL, 8) |
			       ctrl_mask,
			       base + PXE_F0_CTL_OFS);
			break;
		default: /* FBF_YUV420_INTERLEAVE */
			writel(b->src_addr + (b->stride_in * b->height_in) +
			(b->src_y_offset * b->stride_in * 3 / 2) + (b->src_x_offset * 3 / 2),
		       base + PXE_F0_BASEADDRESS_OFS);
			writel(fdb_const(PXE_F0_SRCBUFATTRIBUTES,
					 LINECOUNT, b->height_in / 2 - 1) |
			       fdb_const(PXE_F0_SRCBUFATTRIBUTES,
					 LINEWIDTH, b->width_in / 2 - 1),
			       base + PXE_F0_SRCBUFATTRIBUTES_OFS);
			writel(fdb_const(PXE_F0_DELTAYY, DELTAY, 1 << 1),
			       base + PXE_F0_DELTAYY_OFS);
			writel(fdb_const(PXE_F0_DELTAXX, DELTAX, 1 << 1),
			       base + PXE_F0_DELTAXX_OFS);
			writel(fdb_const(PXE_F0_CTL, TILEMODE, (!!b->rotate)) |
			       fdb_const(PXE_F0_CTL, BITSPERPIXEL, 0x10) |
			       ctrl_mask,
			       base + PXE_F0_CTL_OFS);
			break;
		}
	} else { /* Not YUV */
		if (b->blend) {
			if (b->stride_mask)
				writel(mask_addr,
				       base + PXE_F0_BASEADDRESS_OFS);
			else {
				if (0x00FFFFFF == b->fill_pixel)
					b->op_blend = VG_BLEND_DARKEN;
				writel(src_addr,
				       base + PXE_F0_BASEADDRESS_OFS);
			}
			h = b->height_in;
			w = b->width_in;
			s = b->stride_in;
		} else
			writel(src_addr, base + PXE_F0_BASEADDRESS_OFS);

		writel(fdb_const(PXE_F0_FRAMEDIM, FRAMEHEIGHT, h - 1) |
			fdb_const(PXE_F0_FRAMEDIM, FRAMEWIDTH, w - 1),
			base + PXE_F0_FRAMEDIM_OFS);
		writel(fdb_const(PXE_F0_SRCBUFATTRIBUTES, LINECOUNT, h - 1) |
			fdb_const(PXE_F0_SRCBUFATTRIBUTES, LINEWIDTH, w - 1),
			base + PXE_F0_SRCBUFATTRIBUTES_OFS);
		if (!b->height_in || !b->width_in) { /* Buffer fill */
			writel(fdb_const(PXE_F0_DELTAXX, DELTAX, 0 << 2),
			       base + PXE_F0_DELTAXX_OFS);
			writel(fdb_const(PXE_F0_DELTAYY, DELTAY, 0 << 2),
			       base + PXE_F0_DELTAYY_OFS);
			writel(fdb_const(PXE_F0_CTL, TILEMODE, 1) |
			       fdb_const(PXE_F0_CTL, BITSPERPIXEL,
				         b->bits_per_pixel_in) | ctrl_mask,
			       base + PXE_F0_CTL_OFS);
		} else {
			/* h and v step is +1, starting from top-left px */
			writel(fdb_const(PXE_F0_DELTAXX, DELTAX, 1 << 2),
			       base + PXE_F0_DELTAXX_OFS);
			writel(fdb_const(PXE_F0_DELTAYY, DELTAY, 1 << 2),
			       base + PXE_F0_DELTAYY_OFS);
			writel(fdb_const(PXE_F0_CTL, TILEMODE, (!!b->rotate)) |
			       fdb_const(PXE_F0_CTL, BITSPERPIXEL,
				         b->bits_per_pixel_in) | ctrl_mask,
			       base + PXE_F0_CTL_OFS);
		}
	}

	if (b->format_in == FBF_YUV420_SEPARATE ||
	    b->format_in == FBF_YUV422_SEPARATE)
		writel(fdb_const(PXE_F0_SRCBUFSTRIDE, STRIDE,
				 b->stride_in / 2 - 1),
		       base + PXE_F0_SRCBUFSTRIDE_OFS);
	else if (b->blend && b->stride_mask)
		writel(fdb_const(PXE_F0_SRCBUFSTRIDE, STRIDE,
				 b->stride_mask - 1),
		       base + PXE_F0_SRCBUFSTRIDE_OFS);
	else if (!b->stride_in)
		__raw_writel(fdb_const(PXE_F0_SRCBUFSTRIDE,
					STRIDE, b->stride_out - 1),
			base + PXE_F0_SRCBUFSTRIDE_OFS);
	else
		writel(fdb_const(PXE_F0_SRCBUFSTRIDE, STRIDE, b->stride_in - 1),
		       base + PXE_F0_SRCBUFSTRIDE_OFS);
}

/* blit unit settings: Fetch#1 */
static void fetch1_other(struct fdb_blit *b, void __iomem *base,
						enum blitter_config pl)
{
	if (!(pl & IBC_F1_INTO_ROP0))
		return;

	/* fetch1 gets V of YUV */
	
	switch (b->format_in) {
	
	case FBF_YUV444_SEPARATE:
		writel(fdb_const(PXE_F1_SRCBUFSTRIDE, STRIDE,
				 b->stride_in - 1),
		       base + PXE_F1_SRCBUFSTRIDE_OFS);
		writel(fdb_const(PXE_F1_DELTAXX, DELTAX, 1 << 2),
		       base + PXE_F1_DELTAXX_OFS);
		writel(fdb_const(PXE_F1_DELTAYY, DELTAY, 1 << 2),
		       base + PXE_F1_DELTAYY_OFS);
	
		writel(b->src_addr + (b->stride_in * b->height_in * 2) + 
			(b->src_y_offset * b->stride_in) + b->src_x_offset,
		       base + PXE_F1_BASEADDRESS_OFS);
		writel(fdb_const(PXE_F1_SRCBUFATTR, LINECOUNT,
				 b->height_in - 1) |
		       fdb_const(PXE_F1_SRCBUFATTR, LINEWIDTH,
				 b->width_in - 1),
		       base + PXE_F1_SRCBUFATTR_OFS);
		break;
	default: /* FBF_YUV420_SEPARATE, FBF_YUV422_SEPARATE */
		writel(fdb_const(PXE_F1_SRCBUFSTRIDE, STRIDE,
				 b->stride_in / 2 - 1),
		       base + PXE_F1_SRCBUFSTRIDE_OFS);
		writel(fdb_const(PXE_F1_DELTAXX, DELTAX, 1 << 1),
		       base + PXE_F1_DELTAXX_OFS);
		if (b->format_in == FBF_YUV422_SEPARATE) {
			writel(fdb_const(PXE_F1_DELTAYY, DELTAY, 1 << 2),
			       base + PXE_F1_DELTAYY_OFS);
			writel(b->src_addr + (b->stride_in * b->height_in * 3 / 2) +
				(b->src_y_offset * b->stride_in / 2) + (b->src_x_offset / 2),
			       base + PXE_F1_BASEADDRESS_OFS);
			writel(fdb_const(PXE_F1_SRCBUFATTR, LINECOUNT,
					 b->height_in - 1) |
			       fdb_const(PXE_F1_SRCBUFATTR, LINEWIDTH,
					 b->width_in / 2 - 1),
			       base + PXE_F1_SRCBUFATTR_OFS);
		} else { /* FBF_YUV420_SEPARATE */
			writel(fdb_const(PXE_F1_DELTAYY, DELTAY, 1 << 1),
			       base + PXE_F1_DELTAYY_OFS);
			writel(b->src_addr + (b->stride_in * b->height_in * 5 / 4) +
				(b->src_y_offset / 2 * b->stride_in / 2) + (b->src_x_offset / 2),
			       base + PXE_F1_BASEADDRESS_OFS);
			writel(fdb_const(PXE_F1_SRCBUFATTR, LINECOUNT,
					 b->height_in / 2 - 1) |
			       fdb_const(PXE_F1_SRCBUFATTR, LINEWIDTH,
					 b->width_in / 2 - 1),
			       base + PXE_F1_SRCBUFATTR_OFS);
		}
		break;
	}

	writel(fdb_const(PXE_F1_FRAMEDIM, FRAMEHEIGHT, b->height_in - 1) |
	       fdb_const(PXE_F1_FRAMEDIM, FRAMEWIDTH, b->width_in - 1),
	       base + PXE_F1_FRAMEDIM_OFS);
	writel(fdb_const(PXE_F1_CTL, BITSPERPIXEL, 8), base + PXE_F1_CTL_OFS);
}

/* blit unit settings: CLuT#0 */
static void clut_other(void __iomem *base)
{
	writel(fdb_const(PXENG_CLUT0_CTRL, MODE, 0),
	       base + PXENG_CLUT0_CTRL_OFS);
}

/* blit unit settings: ROp#0 */
static void rop_other(struct fdb_blit *b, void __iomem *base,
						enum blitter_config pl)
{
	if (pl & IBC_YUV_SOURCE)
		writel(fdb_const(PXENG_ROP0_CTL, TERTDIV2, 0) |
		       fdb_const(PXENG_ROP0_CTL, SECDIV2, 0) |
		       fdb_const(PXENG_ROP0_CTL, PRIMDIV2, 0) |
		       fdb_const(PXENG_ROP0_CTL, REDMODE, 1) |
		       fdb_const(PXENG_ROP0_CTL, GREENMODE, 1) |
		       fdb_const(PXENG_ROP0_CTL, BLUEMODE, 1) |
		       fdb_const(PXENG_ROP0_CTL, ALPHAMODE, 1) |
		       fdb_const(PXENG_ROP0_CTL, MODE, 1),
		       base + PXENG_ROP0_CTL_OFS);
	else
		writel(fdb_const(PXENG_ROP0_CTL, MODE, 0),
		       base + PXENG_ROP0_CTL_OFS);
}

/* blit unit settings: HScaler#0 and VScaler#0 */
static void scaler_other(struct fdb_blit *b, void __iomem *base,
						enum blitter_config pl)
{
	int h_upscaler, v_upscaler;
	u32 h_factor, v_factor;

	if (!(pl & IBC_ROP_INTO_SCALER)) {
		writel(fdb_const(PXENG_VSCALER0_CTRL, MODE, 0),
		       base + PXENG_VSCALER0_CTRL_OFS);
		writel(fdb_const(PXENG_HSCALER0_CTRL, MODE, 0),
		       base + PXENG_HSCALER0_CTRL_OFS);

		return;
	}

	v_upscaler = b->height_in < b->height_out;
	h_upscaler = b->width_in < b->width_out;

	/* factors are in 1.15 floating */

	if (v_upscaler)
		v_factor = ((b->height_in - b->scale_adjust_h) << 15) / b->height_out;
	else
		v_factor = ((b->height_out + b->scale_adjust_h) << 15) / b->height_in;
	if (h_upscaler)
		h_factor = ((b->width_in - b->scale_adjust_w) << 15) / b->width_out;
	else
		h_factor = ((b->width_out + b->scale_adjust_w) << 15) / b->width_in;

	if ((h_factor & 0x8000) && h_factor != 0x8000)
		pr_err("h_factor 0x%x, %d %d in, %d %d out\n", h_factor, b->width_in, b->height_in, b->width_out, b->height_out);

	writel(fdb_const(PXENG_VSCALER0_SETUP1, SCALE_FACTOR, v_factor),
	       base + PXENG_VSCALER0_SETUP1_OFS);
	writel(fdb_const(PXENG_VSCALER0_CTRL, OUTPUT_SIZE,
			 b->height_out - 1) |
	       fdb_const(PXENG_VSCALER0_CTRL, FILTER_MODE, 1) |
	       fdb_const(PXENG_VSCALER0_CTRL, SCALE_MODE, v_upscaler) |
	       fdb_const(PXENG_VSCALER0_CTRL, MODE, 1),
	       base + PXENG_VSCALER0_CTRL_OFS);

	writel(fdb_const(PXENG_HSCALER0_SETUP1, SCALE_FACTOR, h_factor),
	       base + PXENG_HSCALER0_SETUP1_OFS);
	writel(fdb_const(PXENG_HSCALER0_CTRL, OUTPUT_SIZE,
			 b->width_out - 1) |

	       fdb_const(PXENG_HSCALER0_CTRL, FILTER_MODE, 1) |
	       fdb_const(PXENG_HSCALER0_CTRL, SCALE_MODE, h_upscaler) |
	       fdb_const(PXENG_HSCALER0_CTRL, MODE, 1),
	       base + PXENG_HSCALER0_CTRL_OFS);
}

/* blit unit settings: Matrix#0 */
static void matrix_other(struct fdb_blit *b, void __iomem *base,
						enum blitter_config pl)
{
	if (pl & IBC_YUV_SOURCE) {
		/* Coefficients of Matrix#0 are for BT.709 */
		writel(fdb_const(PXENG_MATRIX0_CTRL, ALPHAINVERT, 0) |
		       fdb_const(PXENG_MATRIX0_CTRL, ALPHAMASK, 0) |
		       fdb_const(PXENG_MATRIX0_CTRL, MODE, 1),
		       base + PXENG_MATRIX0_CTRL_OFS);
		writel(fdb_const(PXENG_MATRIX0_RED0, A12, 0) |
		       fdb_const(PXENG_MATRIX0_RED0, A11, 0x12a),
		       base + PXENG_MATRIX0_RED0_OFS);
		writel(fdb_const(PXENG_MATRIX0_RED1, C1, 0x708) |
		       fdb_const(PXENG_MATRIX0_RED1, A13, 0x1cb),
		       base + PXENG_MATRIX0_RED1_OFS);
		writel(fdb_const(PXENG_MATRIX0_GREEN0, A22, 0x7c9) |
		       fdb_const(PXENG_MATRIX0_GREEN0, A21, 0x12a),
		       base + PXENG_MATRIX0_GREEN0_OFS);
		writel(fdb_const(PXENG_MATRIX0_GREEN1, C2, 0x04c) |
		       fdb_const(PXENG_MATRIX0_GREEN1, A23, 0x778),
		       base + PXENG_MATRIX0_GREEN1_OFS);
		writel(fdb_const(PXENG_MATRIX0_BLUE0, A32, 0x21d) |
		       fdb_const(PXENG_MATRIX0_BLUE0, A31, 0x12a),
		       base + PXENG_MATRIX0_BLUE0_OFS);
		writel(fdb_const(PXENG_MATRIX0_BLUE1, C3, 0x6df) |
		       fdb_const(PXENG_MATRIX0_BLUE1, A33, 0x000),
		       base + PXENG_MATRIX0_BLUE1_OFS);
	} else
		writel(fdb_const(PXENG_MATRIX0_CTRL, MODE, 0),
		       base + PXENG_MATRIX0_CTRL_OFS);
}

/* blit unit settings: BlitBlend#0 */
static void blitblend_other(struct fdb_blit *b, void __iomem *base)
{
	u32 shift_r, shift_g, shift_b, shift_a;

	if (!b->blend) {
		writel(fdb_const(PXENG_BLITBLEND0_CTL, MODE, 0),
		       base + PXENG_BLITBLEND0_CTL_OFS);
		return;
	}

	switch (b->format_in) {
	case FBF_ABGR8888:
		shift_r = 0;
		shift_g = 8;
		shift_b = 16;
		shift_a = 24;
		break;

	case FBF_BGRA8888:
		shift_r = 8;
		shift_g = 16;
		shift_b = 24;
		shift_a = 0;
		break;

	case FBF_RGBA8888:
		shift_r = 24;
		shift_g = 16;
		shift_b = 8;
		shift_a = 0;
		break;

	default: /* FBF_ARGB8888 */
		shift_r = 16;
		shift_g = 8;
		shift_b = 0;
		shift_a = 24;
		break;
	}

	writel(fdb_const(PXENG_BLITBLEND0_CTL, MODE, 1),
	       base + PXENG_BLITBLEND0_CTL_OFS);
	writel(fdb_const(PXENG_BLITBLEND0_CONSTANTCOL,
			 RED, (b->fill_pixel >> shift_r) & 0xff) |
	       fdb_const(PXENG_BLITBLEND0_CONSTANTCOL,
			 GREEN, (b->fill_pixel >> shift_g) & 0xff) |
	       fdb_const(PXENG_BLITBLEND0_CONSTANTCOL,
			 BLUE, (b->fill_pixel >> shift_b) & 0xff) |
	       fdb_const(PXENG_BLITBLEND0_CONSTANTCOL,
			 ALPHA, (b->fill_pixel >> shift_a) & 0xff),
	       base + PXENG_BLITBLEND0_CONSTANTCOL_OFS);
	writel(fdb_const(PXENG_BLITBLEND0_COLREDBLEND,
			 DST, b->dst_blend) |
	       fdb_const(PXENG_BLITBLEND0_COLREDBLEND,
			 SRC, b->src_blend),
	       base + PXENG_BLITBLEND0_COLREDBLEND_OFS);
	writel(fdb_const(PXENG_BLITBLEND0_COLGREENBLEND,
			 DST, b->dst_blend) |
	       fdb_const(PXENG_BLITBLEND0_COLGREENBLEND,
			 SRC, b->src_blend),
		base + PXENG_BLITBLEND0_COLGREENBLEND_OFS);
	writel(fdb_const(PXENG_BLITBLEND0_COLBLUEBLEND,
			 DST, b->dst_blend) |
	       fdb_const(PXENG_BLITBLEND0_COLBLUEBLEND,
			 SRC, b->src_blend),
	       base + PXENG_BLITBLEND0_COLBLUEBLEND_OFS);
	writel(fdb_const(PXENG_BLITBLEND0_ALPHABLEND,
			 DST, b->dst_blend) |
	       fdb_const(PXENG_BLITBLEND0_ALPHABLEND,
			 SRC, b->src_blend),
	       base + PXENG_BLITBLEND0_ALPHABLEND_OFS);
	writel(fdb_const(PXENG_BLITBLEND0_MODE1, GREEN, b->op_blend) |
	       fdb_const(PXENG_BLITBLEND0_MODE1, RED, b->op_blend),
	       base + PXENG_BLITBLEND0_MODE1_OFS);
	writel(fdb_const(PXENG_BLITBLEND0_MODE2, ALPHA, b->op_blend) |
	       fdb_const(PXENG_BLITBLEND0_MODE2, BLUE, b->op_blend),
	       base + PXENG_BLITBLEND0_MODE2_OFS);
}

/* blit unit settings: Store#0 */
static void
store_other(struct fdb_blit *b, void __iomem *base)
{
	u32 dest_addr = b->dest_addr + b->dest_y_offset * b->stride_out +
		b->dest_x_offset * ((b->bits_per_pixel_out + 7) / 8);

	writel(fdb_const(PXE_S0_CTL, BITSPERPIXEL, b->bits_per_pixel_out),
	       base + PXE_S0_CTL_OFS);
	writel(fdb_const(PXE_S0_DESTBUFSTRIDE, STRIDE, b->stride_out - 1),
	       base + PXE_S0_DESTBUFSTRIDE_OFS);
	writel(dest_addr, base + PXE_S0_BASEADDRESS_OFS);
}

static int
_iris_trigger_pipeline_change(struct f_iris_par *par, int new)
{
	if (new != par->blitter_config) {
#if defined(CONFIG_DEBUG_FS)
		par->global.count_pipeline_flush++;
#endif
		/* expensive pipeline flush */
		writel(fdb_const(PXE_PXB_SYNC_TRIGGER, STORE0_SYNC_TRIGGER, 1),
		       par->base + PXE_PXB_SYNC_TRIGGER_OFS);
		/* for pipeline flush, we must wait until S0 shadow load irq */
	} else {
		/*
		 * if no pipeline change, "write 1 to shdtokgen in all selected
		 * fetch units"
		 */
		switch (new) {
		case IRISBC__F2_SCALER_S0:
			writel(fdb_const(PXE_F2_CTLTRIG, SHDTOKGEN, 1),
			       par->base + PXE_F2_CTLTRIG_OFS);
			break;
		case IRISBC__F0_S0:
			writel(fdb_const(PXE_F0_CTLTRIG, SHDTOKGEN, 1),
			       par->base + PXE_F0_CTLTRIG_OFS);
			break;

		case IRISBC__F2_F0_F1_YUV_S0:
			writel(fdb_const(PXE_F1_CTLTRIG, SHDTOKGEN, 1),
			       par->base + PXE_F1_CTLTRIG_OFS);
			/* fallthru */
		case IRISBC__F2_F0_BLITBLEND_S0:
		case IRISBC__F2_F0_YUV_S0:
			writel(fdb_const(PXE_F0_CTLTRIG, SHDTOKGEN, 1),
			       par->base + PXE_F0_CTLTRIG_OFS);

			writel(fdb_const(PXE_F2_CTLTRIG, SHDTOKGEN, 1),
			       par->base + PXE_F2_CTLTRIG_OFS);
			break;
		}
	}

	/* write a '1' to the start field of Store0 unit */
	writel(fdb_const(PXE_S0_START, START, 1), par->base + PXE_S0_START_OFS);

	/* next action will be shadow load interrupt of Store0 unit */
	par->blitter_config = new;

	return 0;
}

/* BLIT: (IRISBC__F0_S0)
 *	Fetch#0 -> Store#0
 *	Conditions: 1. fdb_blit->format_in != FBF_YUVxxx
 *		    2. fdb_blit->height_in == fdb_blit->height_out
 *		    3. fdb_blit->width_in == fdb_blit->width_out
 * SOLID: (IRISBC__F0_S0)
 *	Fetch#0 -> Store#0
 *	Conditions: 1. fdb_blit->format_in != FBF_YUVxxx
 *		    2. !fdb_blit->height_in
 *		    3. !fdb_blit->width_in
 *		    4. fdb_blit->fill_pixel
 * COMPOSITE: (IRISBC__F2_F0_BLITBLEND_S0)
 *	(src/src) Fetch#0 -> CLuT#0 -> ROp#0 -> [scaler0 ->] Matrix#0
 *								-> BitBlend#0,
 *	(mask/dest ) Fetch#2 -> BitBlend#0,
 *	and BitBlend#0 -> Store#0.
 *	Conditions: fdb_blit->blend
 * SCALER: (IRISBC__F2_SCALER_S0)
 *	Fetch#2 -> ROp#0 -> Scaler#0 -> Store#0
 *	Conditions: 1. fdb_blit->height_in != fdb_blit->height_out
 *		    2. fdb_blit->width_in != fdb_blit->width_out
 * YUV: (IRISBC__F2_F0_YUV_S0 for FBF_YUV420_INTERLEAVE,
 *	and IRISBC__F2_F0_F1_YUV_S0 for
 *	FBF_YUV420_SEPARATE/FBF_YUV422_SEPARATE/FBF_YUV444_SEPARATE)
 *	Coefficients of Matrix#0 are for BT.709
 *	Fetch#0 -> CLuT#0 -> ROp#0, Fetch#2 -> ROp#0, Fetch#1 -> ROp#0,
 *	and ROp#0 -> Scaler#0 -> Matrix#0 -> BlitBlend#0 -> Store#0.
 *	Conditions: fdb_blit->format_in = FBF_YUVxxx
 */
static int
_iris_blit_setup(struct f_iris_par *par, struct fdb_blit *b)
{
	void __iomem *base = par->base;
	int pipeline;

	if (b->blend)
		pipeline = IRISBC__F2_F0_BLITBLEND_S0;
	else if (b->format_in >= FBF_YUV420_SEPARATE)
		pipeline = IRISBC__F2_F0_F1_YUV_S0;
	else if (b->format_in == FBF_YUV420_INTERLEAVE)
		pipeline = IRISBC__F2_F0_YUV_S0;
	else if (!b->height_in || !b->width_in ||
		 (b->height_in == b->height_out &&
		  b->width_in == b->width_out)) {
		pipeline = IRISBC__F0_S0;
#if defined(CONFIG_DEBUG_FS)
		if (!b->height_in || !b->width_in)
			par->global.count_fill_blits++;
		else
			par->global.count_copy_blits++;
#endif
	} else if (b->height_in != b->height_out ||
			b->width_in != b->width_out) {
		pipeline = IRISBC__F2_SCALER_S0;
#if defined(CONFIG_DEBUG_FS)
		par->global.count_scaled_blits++;
#endif
	} else
		return -EINVAL;

	par->current_job = *b;
	par->ts_entered_state = current_kernel_time();
	par->blitter_busy = 1;

	/* pipeline settings & shadow */
	pipeline_shadow(b, base, pipeline);
	/* blit unit settings: format in/out with Fetch#0,1,2 */
	blit_format(b, base);
	/* blit unit settings: Fetch#2 */
	fetch2_other(b, base, pipeline);
	/* blit unit settings: Fetch#0 */
	fetch0_other(b, base, pipeline);
	/* blit unit settings: Fetch#1 */
	fetch1_other(b, base, pipeline);
	/* blit unit settings: CLuT#0 */
	clut_other(base);
	/* blit unit settings: ROp#0 */
	rop_other(b, base, pipeline);
	/* blit unit settings: HScaler#0 and VScaler#0 */
	scaler_other(b, base, pipeline);
	/* blit unit settings: Matrix#0 */
	matrix_other(b, base, pipeline);
	/* blit unit settings: BlitBlend#0 */
	blitblend_other(b, base);
	/* blit unit settings: Store#0 */
	store_other(b, base);

	return _iris_trigger_pipeline_change(par, pipeline);
}

static int f_iris_blit_check(struct f_iris_par *par, struct fdb_blit *b)
{
	/* skip doing anything if no output rectangle */
	if (!b->width_out || !b->height_out)
		return 1;

	/* only RGB32 target supported */
	if (b->format_out != FBF_ARGB8888 && b->format_out != FBF_ABGR8888 &&
		b->format_out != FBF_BGRA8888 && b->format_out != FBF_RGBA8888) {
		dev_err(par->dev, "bad format_out %d\n", b->format_out);
		return -EINVAL;
	}

	/* only reasonable values */
	if (b->width_in >= IRIS_BLITTER_AXIS_LIMIT ||
	    b->width_out >= IRIS_BLITTER_AXIS_LIMIT ||
	    b->height_in >= IRIS_BLITTER_AXIS_LIMIT ||
	    b->height_out >= IRIS_BLITTER_AXIS_LIMIT ||
	    b->src_x_offset >= IRIS_BLITTER_AXIS_LIMIT ||
	    b->src_y_offset >= IRIS_BLITTER_AXIS_LIMIT ||
	    b->mask_x_offset >= IRIS_BLITTER_AXIS_LIMIT ||
	    b->mask_y_offset >= IRIS_BLITTER_AXIS_LIMIT ||
	    b->dest_x_offset >= IRIS_BLITTER_AXIS_LIMIT ||
	    b->dest_y_offset >= IRIS_BLITTER_AXIS_LIMIT ||
	    b->width_in < 0 ||
	    b->width_out < 0 ||
	    b->height_in < 0 ||
	    b->height_out < 0 ||
	    b->src_x_offset < 0 ||
	    b->src_y_offset < 0 ||
	    b->dest_x_offset < 0 ||
	    b->dest_y_offset < 0
	) {
		_iris_dump_blit("rejected blit", b, par->dev);
		return -EINVAL;
	}

	if ((b->src_size_w && !b->src_size_h) ||
			(!b->src_size_w && b->src_size_h) ||
			(b->dest_size_w && !b->dest_size_h) ||
			(!b->dest_size_w && b->dest_size_h))
		return 1;

	if (b->src_size_w && b->src_size_h) {
		if ((b->src_size_w <= b->src_x_offset) ||
			(b->src_size_h <= b->src_y_offset))
			return 1;

		if (b->src_size_w < b->width_in + b->src_x_offset) {
			/* src width out of bounds */
			int new_w = b->src_size_w - b->src_x_offset;
			if (b->width_in != b->width_out) {
				b->width_out = b->width_out * new_w /
								b->width_in;
				b->width_in = new_w;
			} else {
				b->width_in = new_w;
				b->width_out = b->width_in;
			}
			dev_info(par->dev,
				"src width out of bounds, resize to %d\n",
								b->width_in);
		}

		if (b->src_size_h < b->height_in + b->src_y_offset) {
			/* src height out of bounds */
			int new_h = b->src_size_h - b->src_y_offset;
			if (b->height_in != b->height_out) {
				b->height_out = b->height_out * new_h /
								b->height_in;
				b->height_in = new_h;
			} else {
				b->height_in = new_h;
				b->height_out = b->height_in;
			}
			dev_info(par->dev,
				"src height out of bounds, resize to %d\n",
								b->height_in);
		}
	}

	if (b->dest_size_w && b->dest_size_h) {
		if ((b->dest_size_w <= b->dest_x_offset) ||
			(b->dest_size_h <= b->dest_y_offset))
			return 1;

		if (b->dest_size_w < b->width_out + b->dest_x_offset) {
			/* dest width out of bounds */
			int new_w = b->dest_size_w - b->dest_x_offset;
			if (b->width_in != b->width_out) {
				b->width_in = b->width_in * new_w /
								b->width_out;
				b->width_out = new_w;
			} else {
				b->width_out = new_w;
				b->width_in = b->width_out;
			}
			dev_info(par->dev,
				"dest width out of bounds, resize to %d\n",
								b->width_out);
		}

		if (b->dest_size_h < b->height_out + b->dest_y_offset) {
			/* dest height out of bounds */
			int new_h = b->dest_size_h - b->dest_y_offset;
			if (b->height_in != b->height_out) {
				b->height_in = b->height_in * new_h /
								b->height_out;
				b->height_out = new_h;
			} else {
				b->height_out = new_h;
				b->height_in = b->height_out;
			}
			dev_info(par->dev,
				"dest height out of bounds, resize to %d\n",
								b->height_out);
		}
	}

	return 0;
}

static int _f_irs_blitter_get_queue_space(struct f_iris_par *par)
{
	return (par->blits->length - 1) - (
	       (par->blits->head - par->blits->tail) & (par->blits->length - 1));
}

static void _iris_blit_comp_work(struct work_struct *work)
{
	struct fdb_obj_ref *obj =
		container_of(work, struct fdb_obj_ref, comp_work);
	struct drm_gem_object *obj_in = obj->obj_in;
	struct drm_gem_object *obj_out = obj->obj_out;

	if (obj_in)
		drm_gem_object_unreference_unlocked(obj_in);

	if (obj_out)
		drm_gem_object_unreference_unlocked(obj_out);

	kfree(obj);
}

static void _f_iris_blit_bump_tail(struct f_iris_par *par);

void iris_emergency_reset_work(struct work_struct *work)
{
	struct f_iris_par *par = container_of(work,
			struct f_iris_par, blitter_emergency_work);
	unsigned long flags;

	dev_err(par->dev, "%s\n", __func__);

	if (par->blitter_busy)
		_iris_dump_blit("failed job", &par->current_job, par->dev);

	par->dev->driver->pm->runtime_suspend(par->dev);
	par->dev->driver->pm->runtime_resume(par->dev);

	spin_lock_irqsave(&par->blits->lock, flags);
#if 1
	/* drop all jobs */
/*
	par->blits->head = 0;
	par->blits->tail = 0;
*/

	if (par->current_blit_work) {
		schedule_work(par->current_blit_work);
		par->current_blit_work = NULL;
	}

	while(par->blits->tail != par->blits->head) {
		struct fdb_blit_entry *entry =
					&par->blits->ring[par->blits->tail];

		if (entry->obj) {
			schedule_work(&(entry->obj->comp_work));
			entry->obj = NULL;
		}

		_f_iris_blit_bump_tail(par);
	}
#endif
	/* reset last pipeline state to force reissue */
	par->blitter_config = IRISBC__UNCONFIGURED;
	par->blitter_busy = 0;
	memset(&par->current_job, 0, sizeof(par->current_job));

	_f_iris_blitter_poll_queue(par);

#if defined(CONFIG_DEBUG_FS)
	par->global.errors++;
	par->since_last.errors++;
#endif
	spin_unlock_irqrestore(&par->blits->lock, flags);
}

static void _f_iris_blit_bump_head(struct f_iris_par *par)
{
	bool first = par->blits->tail == par->blits->head;
	struct timespec ts1 = current_kernel_time();
	struct f_iris_par *bpar;
	struct list_head *p;
	int delta_ms;

	par->blit_c->ring[par->blit_c->entry].c_id = 0;

	if (par->blits->head + 1 == par->blits->length)
		par->blits->head = 0;
	else
		par->blits->head++;

	list_for_each(p, &par->blits->blitter_list) {
		bpar = container_of(p, struct f_iris_par, blitter_list);

		if (bpar->blitter_busy) {
#if 1
			/* we've been doing this job > 200ms? */
			delta_ms = (timespec_to_ns(&ts1) -
				timespec_to_ns(&bpar->ts_entered_state)) >> 20;
			if (delta_ms > 5000) {
				dev_err(bpar->dev, "Blitter job hung (busy for %ums)\n", delta_ms);
				schedule_work(&bpar->blitter_emergency_work);
//				bpar->blitter_busy = 0;
				goto try;
			}
#endif
			continue;
		}
try:
		if (!first)
			continue;
		/*
		 * the queue was empty but for this, so
		 * if there's an idle blitter have it take
		 * the job straight away...
		 */

		_f_iris_blitter_poll_queue(bpar);
		break;
	}
}

static void _f_iris_blit_bump_tail(struct f_iris_par *par)
{
	if (par->blits->tail + 1 == par->blits->length)
		par->blits->tail = 0;
	else
		par->blits->tail++;

	if (par->blits->tail == par->blits->head)
		dev_dbg(par->dev, "blitter queue goes empty\n");
}

int _f_iris_blitter_poll_queue(struct f_iris_par *par)
{
	struct fdb_blit_entry *entry = &par->blits->ring[par->blits->tail];
	struct f_iris_par *bpar;
	struct list_head *p;
	u32 period;
	int ret = 0;

	par->blitter_busy = 0;

	if (par->blits->head == par->blits->tail)
		goto no_job;

	/*
	 * Is the destination for this job clear of other blitters using it?
	 * This enforces queued Z-order, so later queued blits always go
	 * cleanly on top of earlier queued blits
	 */

	list_for_each(p, &par->blits->blitter_list) {
		bpar = container_of(p, struct f_iris_par, blitter_list);

		/* don't check ourselves... we're idle if we look here for new work */

		if (bpar == par)
			continue;

		/* is he actually doing something? */

		if (!bpar->blitter_busy)
			continue;

		/* is this guy's current job writing to our source? */
		
		if (bpar->current_job.dest_addr == entry->b.src_addr) {
			/* OK... does his dest region conflict with this next job? */

			if (bpar->current_job.dest_x_offset >=
					entry->b.src_x_offset + entry->b.width_in)
				continue;
			if (bpar->current_job.dest_y_offset >=
					entry->b.src_y_offset + entry->b.height_in)
				continue;
			if (bpar->current_job.dest_x_offset + bpar->current_job.width_out <=
					entry->b.src_x_offset)
				continue;
			if (bpar->current_job.dest_y_offset + bpar->current_job.height_out <=
					entry->b.src_y_offset)
				continue;
				
			goto ban;
		}
		
		/* is this guy's current job in our same PA? */

		if (bpar->current_job.dest_addr != entry->b.dest_addr)
			continue;

		/* OK... does his dest region conflict with this next job? */

		if (bpar->current_job.dest_x_offset >=
				entry->b.dest_x_offset + entry->b.width_out)
			continue;
		if (bpar->current_job.dest_y_offset >=
				entry->b.dest_y_offset + entry->b.height_out)
			continue;
		if (bpar->current_job.dest_x_offset + bpar->current_job.width_out <=
				entry->b.dest_x_offset)
			continue;
		if (bpar->current_job.dest_y_offset + bpar->current_job.height_out <=
				entry->b.dest_y_offset)
			continue;

ban:
		/* ugh... yes... then we can't take the next job right now */

		dev_info(par->dev, "unable to take job\n");

#if defined(CONFIG_DEBUG_FS)
		par->global.count_blitter_conflicts++;
#endif
		ret = 1;
		break;
	}

	if (ret)
		goto no_job;

	par->current_blit_completion = entry->comp;
	par->current_blit_work = NULL;
	ret = _iris_blit_setup(par, &entry->b);
	/* we are busy now */
	par->ts_entered_state = current_kernel_time();

#if defined(CONFIG_DEBUG_FS)
	if (!ret) {
		period = (u32)(timespec_to_ns(&par->ts_entered_state) -
				timespec_to_ns(&entry->ts_queued)) / 1000000;
		if (period > par->global.worst_latency_ms)
			par->global.worst_latency_ms = period;
		if (period > par->since_last.worst_latency_ms)
			par->since_last.worst_latency_ms = period;
		par->global.count_blitter_ops++;
		if (entry->b.width_out * entry->b.height_out >
				entry->b.width_in * entry->b.height_in)
			par->global.count_pixels_blitted +=
				entry->b.width_out * entry->b.height_out;
		else
			par->global.count_pixels_blitted +=
				entry->b.width_in * entry->b.height_in;
	} else
		par->global.errors++;
#endif

	if (!ret) {
		if (par->blits->tail == par->blits->head)
			par->blit_c->ring[par->blit_c->entry].c_id = 1;

		if (par->blit_c->entry >= par->blit_c->length - 1)
			par->blit_c->entry = 0;
		else
			par->blit_c->entry++;
		
		if (entry->obj) {
			par->current_blit_work = &(entry->obj->comp_work);
			entry->obj = NULL;
		}

		/* remove the job from the queue */
		_f_iris_blit_bump_tail(par);
	} else
		dev_err(par->dev, "blit failed\n");
		
	return 1;

no_job:
	/* we ran out of jobs, so any new job later can run directly */
	par->blitter_busy = 0;

	return 0;
}


/* IRIS blitter has some restrictions that are in force when doing scaling */

static void _f_iris_blit_queue_via_restrictions(struct f_iris_par *par,
		struct fdb_blit *b, struct completion *comp, struct fdb_obj_ref *obj)
{
	struct fdb_blit_entry *entry;
	struct fdb_blit b1;
	int n;

	/* width limitation during scaling - chunk it in units of <= 1024 */
	if (b->width_in &&
		(b->width_in != b->width_out ||
		 b->height_in != b->height_out) &&
		(b->width_out > IRIS_SCALER_LIMIT ||
		 b->width_in > IRIS_SCALER_LIMIT)) {

		n = 0;
		b1 = *b;

#if defined(CONFIG_DEBUG_FS)
		par->global.count_h_restriction_blits++;
#endif
		while (n < b->width_out) {

			if (b->width_out - n > IRIS_SCALER_LIMIT)
				b1.width_out = IRIS_SCALER_LIMIT - 1;
			else
				b1.width_out = b->width_out - n;

			/* corresponding amount of width_in */
			b1.width_in = (b->width_in * b1.width_out) /
								b->width_out;
			if (b1.width_in > IRIS_SCALER_LIMIT) {
				/* 
				 * the computed matching input width blows
				 * the limit, clip input at the limit then
				 * and recompute out width to match
				 */
				b1.width_in = IRIS_SCALER_LIMIT - 1;
				b1.width_out = (b->width_out * b1.width_in) /
								b->width_in;
			} else if ((b1.width_out != (IRIS_SCALER_LIMIT - 1)) &&
					(n < (IRIS_SCALER_LIMIT - 1))) {
				/*
				 * remains, when input width blows the limit.
				 */
				b1.width_in = b->width_in %
							(IRIS_SCALER_LIMIT - 1);
			}

			n += b1.width_out;

			entry = &par->blits->ring[par->blits->head];
			/* only last one triggers the completion if any */
			if (n < b->width_out) {
				entry->comp = NULL;
				entry->obj = NULL;
			}
			else {
				entry->comp = comp;
				entry->obj = obj;
			}

			entry->ts_queued = current_kernel_time();
			entry->b = b1;

			_f_iris_blit_bump_head(par);

			b1.src_x_offset += b1.width_in;
			b1.dest_x_offset += b1.width_out;
		}
		return;
	}

	/* just normal */
	par->blits->ring[par->blits->head].obj = obj;
	par->blits->ring[par->blits->head].comp = comp;
	par->blits->ring[par->blits->head].ts_queued = current_kernel_time();
	par->blits->ring[par->blits->head].b = *b;
	_f_iris_blit_bump_head(par);
}

static int
f_iris_blit_queued(struct f_fdb_child *fdb_child, struct fdb_blit *b,
						struct completion *comp, struct fdb_obj_ref *obj)
{
	struct f_iris_par *par = f_par_from_fdb_child(fdb_child);
	int bytespp_in = (b->bits_per_pixel_in + 7) / 8;
	unsigned long flags;
	struct f_iris_par *bpar;
	struct list_head *p;
	struct fdb_blit *bp;
	int overlap;
	int ret = 0;
	int len;
	int n;

	ret = f_iris_blit_check(par, b);
	if (ret) {
		return ret;
	}

	/* do we have a problem that the dest will overwrite the source? */
	overlap = _iris_check_overlap(b, par->dev);

	/* if source IS the destination, no need to do anything */
	if (overlap == IRIS_BOC_PERFECT_OVERLAP)
		return 1; /* 1 means just ignore */

	spin_lock_irqsave(&par->blits->lock, flags);

	len = _f_irs_blitter_get_queue_space(par);
	if (len < 8) { /* need to allow for fragmentation of task */
		if (par->perf_mode) {
			for (n = 0; n < BLIT_RING_LEN >> 1; n++) {
				struct fdb_blit_entry *entry =
							&par->blits->ring[par->blits->tail];

				if (entry->obj) {
					schedule_work(&(entry->obj->comp_work));
					entry->obj = NULL;
				}

				_f_iris_blit_bump_tail(par);
			}
#if defined(CONFIG_DEBUG_FS)
			par->global.count_queue_full_events++;
#endif
		} else {
			dev_err(par->dev, "%s: queue full\n", __func__);

			list_for_each(p, &par->blits->blitter_list) {
				bpar = container_of(p, struct f_iris_par, blitter_list);

				schedule_work(&bpar->blitter_emergency_work);
			}
			goto bail;
		}
	}

#if defined(CONFIG_DEBUG_FS)
	n = (par->blits->head - par->blits->tail) & (par->blits->length - 1);
	if (n > par->global.worst_ring_depth)
		par->global.worst_ring_depth = n;
	if (n > par->since_last.worst_ring_depth)
		par->since_last.worst_ring_depth = n;
#endif

	b->id = par->blit_c->entry + 1;

	if (comp)
		init_completion(comp);

	/* is the destination actually overlapping with the source? */
	if (overlap == IRIS_BOC_OVERLAPPING) {
		/*
		 * we need to buffer the entire source blit first.  We can't
		 * tile because first tile destination may overlap later tile
		 * source.
		 */

		if (b->width_in * b->height_in * bytespp_in >
				BLITTER_TEMP_SIZE) {
			dev_err(fdb_child->dev, "blit src > cache %d vs %d\n",
				b->width_in * b->height_in * bytespp_in,
							BLITTER_TEMP_SIZE);
			ret = -EINVAL;
			goto bail;
		}

		par->blits->ring[par->blits->head].obj = NULL;
		par->blits->ring[par->blits->head].comp = NULL;
		par->blits->ring[par->blits->head].ts_queued =
							current_kernel_time();
		bp = &par->blits->ring[par->blits->head].b;
		*bp = *b;
		bp->dest_x_offset = 0;
		bp->dest_y_offset = 0;
		bp->stride_out = b->width_in * bytespp_in;
		bp->dest_addr = par->blit_temp_paddr;

		_f_iris_blit_bump_head(par);

		par->blits->ring[par->blits->head].obj = obj;
		par->blits->ring[par->blits->head].comp = comp;
		par->blits->ring[par->blits->head].ts_queued =
							current_kernel_time();
		bp = &par->blits->ring[par->blits->head].b;
		*bp = *b;
		bp->src_x_offset = 0;
		bp->src_y_offset = 0;
		bp->stride_in = b->width_in * bytespp_in;
		bp->src_addr = par->blit_temp_paddr;

		_f_iris_blit_bump_head(par);

#if defined(CONFIG_DEBUG_FS)
		par->global.count_blitter_overlaps++;
#endif

		goto bail;
	}

	_f_iris_blit_queue_via_restrictions(par, b, comp, obj);

bail:
	spin_unlock_irqrestore(&par->blits->lock, flags);

	return ret;
}

int f_iris_get_blit_comp(struct f_fdb_child *fdb_child,
		struct b_c_id_ring *b_c_id_ring)
{
	struct f_iris_par *par = f_par_from_fdb_child(fdb_child);
	int len;
	unsigned long timeout;

	memcpy(b_c_id_ring->b_c_id, par->blit_c->ring,
			sizeof(struct blit_completion_id) * BLIT_COMP_RING_LEN);
	len = _f_irs_blitter_get_queue_space(par);
	timeout = 100;
	while (len < (par->blits->length - 1) && timeout) {
		len = _f_irs_blitter_get_queue_space(par);
		timeout--;
		mdelay(100);
	}

	if (!timeout)
		dev_err(par->dev, "%s: timeout error\n", __func__);

	return 0;
}
EXPORT_SYMBOL_GPL(f_iris_get_blit_comp);

int
f_iris_blit_blocking2(struct f_fdb_child *fdb_child, struct fdb_blit *b,
				struct completion *comp_arg, void *obj_in, void *obj_out)
{
	struct f_iris_par *par = f_par_from_fdb_child(fdb_child);
	struct completion completion, *comp = NULL;
	struct fdb_obj_ref *obj = NULL;
	int ret;

	if (!b->async)
		comp = &completion;
	else if (b->async == BLIT_ASYNC_NEED_COMP && comp_arg)
		comp = comp_arg;

	if (obj_in || obj_out) {
		obj = kmalloc(sizeof(*obj), GFP_KERNEL);
		if (!obj) {
			dev_err(par->dev, "%s: memory alloc failed\n", __func__);
			return -ENOMEM;
		}

		/* get reference until blit completed */
		if (obj_in)
			drm_gem_object_reference(obj_in);

		if (obj_out)
			drm_gem_object_reference(obj_out);

		obj->obj_in = obj_in;
		obj->obj_out = obj_out;
		INIT_WORK(&obj->comp_work, _iris_blit_comp_work);
	}

	ret = f_iris_blit_queued(fdb_child, b, comp, obj);
	if (ret) {
		if (ret > 0) { /* 1 means just ignore */
			dev_dbg(par->dev, "%s: ignore blit\n", __func__);
			ret = 0;
		}

		if (obj) {
			if (obj_in)
				drm_gem_object_unreference_unlocked(obj_in);

			if (obj_out)
				drm_gem_object_unreference_unlocked(obj_out);

			kfree(obj);
		}

		return ret;
	}

	memcpy(b->b_c_id, par->blit_c->ring,
			sizeof(struct blit_completion_id) * BLIT_COMP_RING_LEN);

	if (b->async)
		return 0;

	ret = wait_for_completion_timeout(&completion, msecs_to_jiffies(1000));
	if (!ret) {
		dev_err(par->dev, "%s: timeout\n", __func__);
		return -ETIMEDOUT;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(f_iris_blit_blocking2);

int
f_iris_blit_blocking(struct f_fdb_child *fdb_child, struct fdb_blit *b)
{
	WARN_ON(b->async == BLIT_ASYNC_NEED_COMP);
	return f_iris_blit_blocking2(fdb_child, b, NULL, NULL, NULL);
}
EXPORT_SYMBOL_GPL(f_iris_blit_blocking);

/*
 * Called from IRQ handler when blitter complete interrupt appears
 */

void f_iris_blitter_complete_irq(struct f_iris_par *par)
{
#if defined(CONFIG_DEBUG_FS)
	struct timespec ts1;
#endif
	struct f_iris_par *bpar;
	struct list_head *p;

	if (par->current_blit_completion)
		complete(par->current_blit_completion);

	spin_lock(&par->blits->lock);
	if (par->current_blit_work) {
		schedule_work(par->current_blit_work);
		par->current_blit_work = NULL;
	}
	spin_unlock(&par->blits->lock);

#if defined(CONFIG_DEBUG_FS)
	ts1 = current_kernel_time();
	par->global.last_ms_in_blitter_wait = (u32)(timespec_to_ns(&ts1) -
		timespec_to_ns(&par->ts_entered_state)) / 1000000;
	par->global.count_ms_blitter_busy +=
		par->global.last_ms_in_blitter_wait;
	if (par->current_blit_completion)
		par->global.count_ms_in_blitter_wait +=
			par->global.last_ms_in_blitter_wait;
#endif

	if (par->going_away)
		return;

	par->blitter_busy = 0;

	/*
	 * protect entire queue processing with global blitter lock
	 *
	 * with multiple blitters and SMP, another blitter may complete
	 * while this interrupt on one core is meddling with the queue
	 * If another core comes here to service, block it until the
	 * other interrupt service completed
	 */
	spin_lock(&par->blits->lock);
	list_for_each(p, &par->blits->blitter_list) {
		bpar = container_of(p, struct f_iris_par, blitter_list);
		if (!bpar->blitter_busy)
			_f_iris_blitter_poll_queue(bpar);
	}
	spin_unlock(&par->blits->lock);
}

#if defined(CONFIG_DEBUG_FS)
void f_iris_blitter_debugfs(struct seq_file *m, struct f_iris_par *par)
{
	unsigned long flags;
	u64 ns = timespec_to_ns(&par->last);
	struct timespec ts;
	int n, n1, n2, dur;

	spin_lock_irqsave(&par->blits->lock, flags);

	ts = current_kernel_time();
	par->global.frame_index = par->frame_index;

	dur = (u32)((timespec_to_ns(&ts) - ns) >> 8) / (1000000 / 256);

	if (ns)
		dev_seq_printf(par->dev, m,
			       "\t\t\t\t\tTotal\t(in last %ums)\n", dur);
	dev_seq_printf(par->dev, m, "vsync count:\t\t\t\t%d\t%d\n",
		       par->global.frame_index,
		       par->global.frame_index - par->since_last.frame_index);
	n = par->global.count_blitter_ops - par->since_last.count_blitter_ops;
	n2 = par->global.frame_index - par->since_last.frame_index;
	if (n2)
		n1 = (n * 1000) / n2;
	else
		n1 = 0;
	dev_seq_printf(par->dev, m,
			"total blit_ops:\t\t\t%d\t%d (%d.%03d / vsync)\n",
		       par->global.count_blitter_ops,
		       n, n1 / 1000, n1 % 1000);
	dev_seq_printf(par->dev, m, "  pipe flushes:\t\t\t%d\t%d\n",
		       par->global.count_pipeline_flush,
		       par->global.count_pipeline_flush -
		       par->since_last.count_pipeline_flush);
	dev_seq_printf(par->dev, m, "    fill_blits:\t\t\t%d\t%d\n",
		       par->global.count_fill_blits,
		       par->global.count_fill_blits -
		       par->since_last.count_fill_blits);
	dev_seq_printf(par->dev, m, "    copy_blits:\t\t\t%d\t%d\n",
		       par->global.count_copy_blits,
		       par->global.count_copy_blits -
		       par->since_last.count_copy_blits);
	dev_seq_printf(par->dev, m, "  scaled_blits:\t\t\t%d\t%d\n",
		       par->global.count_scaled_blits,
		       par->global.count_scaled_blits -
				par->since_last.count_scaled_blits);
	dev_seq_printf(par->dev, m,
		       "  scaler restriction limit:\t\th%d v%d\th%d v%d\n",
		       par->global.count_h_restriction_blits,
		       par->global.count_v_restriction_blits,
		       par->global.count_h_restriction_blits -
		       par->since_last.count_h_restriction_blits,
		       par->global.count_v_restriction_blits -
		       par->since_last.count_v_restriction_blits);
	dev_seq_printf(par->dev, m, "  overlap_blits:\t\t\t%d\t%d\n",
		       par->global.count_blitter_overlaps,
		       par->global.count_blitter_overlaps -
				par->since_last.count_blitter_overlaps);
	dev_seq_printf(par->dev, m, "  multiblitter conflicts:\t\t%d\t%d\n",
		       par->global.count_blitter_conflicts,
		       par->global.count_blitter_conflicts -
				par->since_last.count_blitter_conflicts);

	n = par->global.count_ms_blitter_busy -
				par->since_last.count_ms_blitter_busy;
	if (dur)
		n1 = (n * 100) / dur;
	dev_seq_printf(par->dev, m, "blitter busy ms:\t\t\t%u\t%u (%u%%)\n",
		       par->global.count_ms_blitter_busy, n, n1);
	dev_seq_printf(par->dev, m, "ms last blitter wait:\t\t%u\n",
		       par->global.last_ms_in_blitter_wait);
	dev_seq_printf(par->dev, m, "total ms spent in blitter wait:\t%u\t%u\n",
		       par->global.count_ms_in_blitter_wait,
		       par->global.count_ms_in_blitter_wait -
		       par->since_last.count_ms_in_blitter_wait);
	dev_seq_printf(par->dev, m, "   ms pipeline setup:\t\t%u\t%u\n",
		       par->global.count_us_pipeline_setup / 1000,
		       (par->global.count_us_pipeline_setup -
			par->since_last.count_us_pipeline_setup) / 1000);
	if (dur)
		dev_seq_printf(par->dev, m,
			"total Mpx blitted:\t\t\t%lu\t%lu (%luMpx/sec)\n",
		       (unsigned long)(par->global.count_pixels_blitted >> 20),
		       (unsigned long)((par->global.count_pixels_blitted -
				par->since_last.count_pixels_blitted) >> 20),
		       (((unsigned long)((par->global.count_pixels_blitted -
				par->since_last.count_pixels_blitted)) / dur)
									>> 10));
	else
		dev_seq_printf(par->dev, m,
				"total Mpx blitted:\t\t0\t0 (%uMpx/sec)\n", 0);
	dev_seq_printf(par->dev, m, "errors:\t\t\t\t%u\t%u\n",
			par->global.errors,
			par->since_last.errors);
	dev_seq_printf(par->dev, m, "worst ring depth:\t\t\t%u\t%u\n",
			par->global.worst_ring_depth,
			par->since_last.worst_ring_depth);

	dev_seq_printf(par->dev, m, "worst queue latency ms:\t\t%u\t%u\n",
			par->global.worst_latency_ms,
			par->since_last.worst_latency_ms);

	dev_seq_printf(par->dev, m, "Instantaneous queue length:\t\t%u/%u\n", (par->blits->head - par->blits->tail) & (par->blits->length - 1), par->blits->length);
	dev_seq_printf(par->dev, m, "Instantaneous busy:\t\t\t%u\n", par->blitter_busy	);

	dev_seq_printf(par->dev, m, "queue full events:\t\t\t%d\t%d\n",
		       par->global.count_queue_full_events,
		       par->global.count_queue_full_events -
		       par->since_last.count_queue_full_events);


	par->since_last = par->global;
	par->since_last.worst_latency_ms = 0;
	par->since_last.worst_ring_depth = 0;
	par->since_last.errors = 0;
	par->last = ts;

	spin_unlock_irqrestore(&par->blits->lock, flags);
}
#endif
