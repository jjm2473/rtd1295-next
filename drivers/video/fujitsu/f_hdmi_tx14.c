/*
 * f_hdmi_tx14 HDMI 14 PHY driver
 * Copyright (C) 2013-2015 Linaro, Ltd for Fujitsu Semi
 * Author: Andy Green <andy.green@linaro.org>
 * Copyright (C) 2015-2016 Socionext Inc.
 */

// #define DEBUG_F_HDMI_TX14

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <sound/asound.h>
#include <sound/asoundef.h>
#include <video/f_hdmi_tx14.h>

#include <video/fdb.h>
#include <linux/pm_runtime.h>

struct f_hdmi_tx14 {
	struct f_fdb_child fdb_child;
	struct device *dev;
	void __iomem *base;
	int irq;
	struct drm_device *drm_dev;
	u32 source_crtc_bitfield;
	enum hdmi_dvi_mode mode;
	struct f_fdb_child *bound; /* fdb fb we are bound to */
	u8 edid[512];
	bool private_edid_valid;

	struct clk	*pllclk;
	struct clk	*pclk;
	struct clk	*dpiclk;

	struct mutex lock;
	struct f_hdmi_config cfg;
	struct f_hdmi_infoframe_avi avi_cfg;
};


#ifdef DEBUG_F_HDMI_TX14
struct dumps {
	const char *name;
	unsigned int start;
	unsigned int len;
};

static void hdmitx14_dump_regs(struct f_hdmi_tx14 *par)
{
	void __iomem *base = par->base;
	int n, i;
	struct dumps dumps[] = {
		{ "Base", 0, 0x0e },
		{ "Video", 0x32, 0x1c },
		{ "TDMS", 0x80, 0xf },
	};

	dev_info(par->dev, "============================HDMI TX_14_regs %x\n", (unsigned int)base);

	for (i = 0; i < ARRAY_SIZE(dumps); i++)
		for (n = 0; n <= dumps[i].len; n ++)
			dev_info(par->dev, "%s: +0x%04x: 0x%02X\n",
				dumps[i].name, dumps[i].start + n,
					__raw_readb(base + dumps[i].start + n));
}
#else
#define hdmitx14_dump_regs(_x)
#endif

/*
 * reference list of CEA timings we can use HDMI audio with
 */
struct f_video_timings {
	u16 x_res;
	u16 y_res;
	u32 pixel_clock;
	u16 hsw;        /* Horizontal synchronization pulse width */
	u16 hfp;        /* Horizontal front porch */
	u16 hbp;        /* Horizontal back porch */
	u16 vsw;        /* Vertical synchronization pulse width */
	u16 vfp;        /* Vertical front porch */
	u16 vbp;        /* Vertical back porch */
	bool vsync_level;
	bool hsync_level;
	bool interlace;
	unsigned char code;
	u16 mode;
};

static const struct f_video_timings cea_timings[] = {
	/* CEA (hdmi_group = 1) */
	{
		640, 480, 25200, 96, 16, 48, 2, 10, 33,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 1, HDMI_HDMI
	}, {
		720, 480, 27000, 62, 16, 60, 6, 9, 30,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 2, HDMI_HDMI
	}, {
		1280, 720, 74250, 40, 110, 220, 5, 5, 20,
			HDMI_TX14_ACTIVE_HIGH, HDMI_TX14_ACTIVE_HIGH,
			false, 4, HDMI_HDMI
/*
	}, {
		1920, 540, 74250, 44, 88, 148, 5, 2, 15,
			HDMI_TX14_ACTIVE_HIGH, HDMI_TX14_ACTIVE_HIGH,
			true, 5, HDMI_HDMI
	}, {
		1440, 240, 27027, 124, 38, 114, 3, 4, 15,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			true, 6, HDMI_HDMI
*/
	}, {
		1920, 1080, 148500, 44, 88, 148, 5, 4, 36,
			HDMI_TX14_ACTIVE_HIGH, HDMI_TX14_ACTIVE_HIGH,
			false, 16, HDMI_HDMI
	}, {
		720, 576, 27000, 64, 12, 68, 5, 5, 39,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 17, HDMI_HDMI
	}, {
		1280, 720, 74250, 40, 440, 220, 5, 5, 20,
			HDMI_TX14_ACTIVE_HIGH, HDMI_TX14_ACTIVE_HIGH,
			false, 19, HDMI_HDMI
/*
	}, {
		1920, 540, 74250, 44, 528, 148, 5, 2, 15,
			HDMI_TX14_ACTIVE_HIGH, HDMI_TX14_ACTIVE_HIGH,
			true, 20, HDMI_HDMI
	}, {
		1440, 288, 27000, 126, 24, 138, 3, 2, 19,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			true, 21, HDMI_HDMI
*/
	}, {
		1440, 576, 54000, 128, 24, 136, 5, 5, 39,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 29, HDMI_HDMI
	}, {
		1920, 1080, 148500, 44, 528, 148, 5, 4, 36,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 31, HDMI_HDMI
/*	}, {
		1920, 1080, 74250, 44, 638, 148, 5, 4, 36,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 32, HDMI_HDMI
*/
    },
	/* DMT (hdmi_group = 2) */
	{
		640, 480, 40000, 96, 16, 48, 2, 10, 33,
			HDMI_TX14_ACTIVE_HIGH, HDMI_TX14_ACTIVE_HIGH,
			false, 4, HDMI_DVI
	}, {
		800, 600, 40000, 128, 40, 88,    4, 1, 23,
			HDMI_TX14_ACTIVE_HIGH, HDMI_TX14_ACTIVE_HIGH,
			false, 9, HDMI_DVI
	}, {
		1024, 600, 32000, 48, 40, 40, 3, 13, 29,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 15, HDMI_DVI
	}, {
		1024, 768, 65000, 136, 24, 160, 6, 3, 29,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 16, HDMI_DVI
	}, {
		1280, 768, 79500, 128, 64, 192, 7, 3, 20,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 23, HDMI_DVI
	}, {
		1280, 800, 71000, 32, 48, 80, 6, 3, 14,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 28, HDMI_DVI
	}, {
		1360, 768, 85500, 112, 64, 256, 6, 3, 18,
			HDMI_TX14_ACTIVE_HIGH, HDMI_TX14_ACTIVE_HIGH,
			false, 39, HDMI_DVI
	}, {
		1400, 1050, 101000, 32, 48, 80, 4, 3, 23,
			HDMI_TX14_ACTIVE_HIGH, HDMI_TX14_ACTIVE_HIGH,
			false, 42, HDMI_DVI
	}, {
		1600, 1200, 162000, 192, 64, 304,    3, 1, 46,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 51, HDMI_DVI
	}, {
		1680, 1050, 146250, 176 , 104, 280, 6, 3, 30,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 58, HDMI_DVI
	}, {
		1680, 1050, 119000, 32 , 48, 80, 6, 3, 21,
			HDMI_TX14_ACTIVE_LOW, HDMI_TX14_ACTIVE_LOW,
			false, 58, HDMI_DVI
	},

};

static inline void f_hdmi_write_reg(void __iomem *base_addr,
				const u16 idx, u32 val)
{
	writeb(val, base_addr + idx);
}

static inline u8 f_hdmi_read_reg(void __iomem *base_addr,
				const u16 idx)
{
	return readb(base_addr + idx);
}

static void f_hdmi_core_powerdown_disable(struct f_hdmi_tx14 *priv)
{

	fdb_setb(priv->base, FHT14_ACR_DPD, NPOWERDOWN_TOTAL, 0);
}

#if 0
static void f_hdmi_core_swreset_release(struct f_hdmi_tx14 *priv)
{

	fdb_setb(priv->base, FHT14_BASE_SRST, SWRST, 0);
	fdb_setb(priv->base, FHT14_BASE_SRST, FIFORST, 0);
}

static void f_hdmi_core_swreset_assert(struct f_hdmi_tx14 *priv)
{

	fdb_setb(priv->base, FHT14_BASE_SRST, SWRST, 1);
	fdb_setb(priv->base, FHT14_BASE_SRST, FIFORST, 1);
}
#endif

static void f_hdmi_core_aux_infoframe_avi_config(struct f_hdmi_tx14 *priv)
{
	u32 val;
	char sum = 0, checksum = 0;
	void __iomem *base = priv->base;
	struct f_hdmi_infoframe_avi info_avi = priv->avi_cfg;

	sum += 0x82 + 0x02 + 0x0D;
	f_hdmi_write_reg(base, FHT14_PKT_AVI_TYPE_OFS, 0x82);
	f_hdmi_write_reg(base, FHT14_PKT_AVI_VERS_OFS, 0x02);
	f_hdmi_write_reg(base, FHT14_PKT_AVI_LEN_OFS, 0x0D);

	val = (info_avi.db1_format << 5) |
		(info_avi.db1_active_info << 4) |
		(info_avi.db1_bar_info_dv << 2) |
		(info_avi.db1_scan_info);
	f_hdmi_write_reg(base, FHT14_PKT_AVI_DBYTE1_OFS, val);
	sum += val;

	val = (info_avi.db2_colorimetry << 6) |
		(info_avi.db2_aspect_ratio << 4) |
		(info_avi.db2_active_fmt_ar);
	f_hdmi_write_reg(base, FHT14_PKT_AVI_DBYTE2_OFS, val);
	sum += val;

	val = (info_avi.db3_itc << 7) |
		(info_avi.db3_ec << 4) |
		(info_avi.db3_q_range << 2) |
		(info_avi.db3_nup_scaling);
	f_hdmi_write_reg(base, FHT14_PKT_AVI_DBYTE3_OFS, val);
	sum += val;

	f_hdmi_write_reg(base, FHT14_PKT_AVI_DBYTE4_OFS,
					info_avi.db4_videocode);
	sum += info_avi.db4_videocode;

	val = info_avi.db5_pixel_repeat;
	f_hdmi_write_reg(base, FHT14_PKT_AVI_DBYTE5_OFS, val);
	sum += val;

	val = info_avi.db6_7_line_eoftop & 0x00FF;
	f_hdmi_write_reg(base, FHT14_PKT_AVI_DBYTE6_OFS, val);
	sum += val;

	val = ((info_avi.db6_7_line_eoftop >> 8) & 0x00FF);
	f_hdmi_write_reg(base, FHT14_PKT_AVI_DBYTE7_OFS, val);
	sum += val;

	val = info_avi.db8_9_line_sofbottom & 0x00FF;
	f_hdmi_write_reg(base, FHT14_PKT_AVI_DBYTE8_OFS, val);
	sum += val;

	val = ((info_avi.db8_9_line_sofbottom >> 8) & 0x00FF);
	f_hdmi_write_reg(base, FHT14_PKT_AVI_DBYTE9_OFS, val);
	sum += val;

	val = info_avi.db10_11_pixel_eofleft & 0x00FF;
	f_hdmi_write_reg(base, FHT14_PKT_AVI_DBYTE10_OFS, val);
	sum += val;

	val = ((info_avi.db10_11_pixel_eofleft >> 8) & 0x00FF);
	f_hdmi_write_reg(base, FHT14_PKT_AVI_DBYTE11_OFS, val);
	sum += val;

	val = info_avi.db12_13_pixel_sofright & 0x00FF;
	f_hdmi_write_reg(base, FHT14_PKT_AVI_DBYTE12_OFS, val);
	sum += val;

	val = ((info_avi.db12_13_pixel_sofright >> 8) & 0x00FF);
	f_hdmi_write_reg(base, FHT14_PKT_AVI_DBYTE13_OFS, val);
	sum += val;

	checksum = 0x100 - sum;
	f_hdmi_write_reg(base, FHT14_PKT_AVI_CHSUM_OFS, checksum);
	dev_info(priv->dev, "[%s] Done!\n", __FUNCTION__);
}

static void f_hdmi_core_av_packet_config(struct f_hdmi_tx14 *priv,
		struct f_hdmi_packet_enable_repeat repeat_cfg)
{
	void __iomem *base = priv->base;

	/* enable/repeat the infoframe */
	f_hdmi_write_reg(base, FHT14_PKT_PB_CTRL1_OFS,
		(repeat_cfg.audio_pkt << 5) |
		(repeat_cfg.audio_pkt_repeat << 4) |
		(repeat_cfg.avi_infoframe << 1) |
		(repeat_cfg.avi_infoframe_repeat));
#if 0
	/* enable/repeat the packet */
	f_hdmi_write_reg(base, FHT14_PKT_PB_CTRL2_OFS,
		(repeat_cfg.gen_cntrl_pkt << 3) |
		(repeat_cfg.gen_cntrl_pkt_repeat << 2) |
		(repeat_cfg.generic_pkt << 1) |
		(repeat_cfg.generic_pkt_repeat));
#endif
}

int f_hdmi_wp_video_start(struct f_hdmi_tx14 *priv)
{
	fdb_setb(priv->base, FHT14_VDE_DE_CTRL, DE_GEN, 1);

	return 0;
}

void f_hdmi_wp_video_stop(struct f_hdmi_tx14 *priv)
{
	fdb_setb(priv->base, FHT14_VDE_DE_CTRL, DE_GEN, 0);
}

static void f_hdmi_wp_video_config_timing(struct f_hdmi_tx14 *priv)
{
	u32 resolution_h = priv->cfg.timings.dt.hactive.typ;
	u32 resolution_v = priv->cfg.timings.dt.vactive.typ;
	int flags = priv->cfg.timings.dt.flags;
	u32 timing_h = priv->cfg.timings.dt.hback_porch.typ + priv->cfg.timings.dt.hsync_len.typ;
	u32 timing_v = priv->cfg.timings.dt.vback_porch.typ + priv->cfg.timings.dt.vsync_len.typ;

	if (!priv->cfg.timings.dt.pixelclock.typ)
		return;

	dev_info(priv->dev, "%s: %d x %d\n",
			__func__, priv->cfg.timings.dt.hactive.typ,
					      priv->cfg.timings.dt.vactive.typ);

	fdb_setb(priv->base, FHT14_VDE_DE_DLY, DE_DLY_L, timing_h & 0xff);
	fdb_setb(priv->base, FHT14_VDE_DE_CTRL, DE_DLY_H, (timing_h >> 8) & 0xf);
	fdb_setb(priv->base, FHT14_VDE_DE_TOP, DE_TOP, timing_v & 0x7f);

	if (flags & DISPLAY_FLAGS_INTERLACED)
		resolution_v = (resolution_v >> 1);

	fdb_setb(priv->base, FHT14_VDE_DE_CNTL, DE_CNTL, (resolution_h & 0xff));
	fdb_setb(priv->base, FHT14_VDE_DE_CNTH, DE_CNTH,
			((resolution_h >> 8) & 0xf));
	fdb_setb(priv->base, FHT14_VDE_DE_LINL, DE_LINL, (resolution_v & 0xff));
	fdb_setb(priv->base, FHT14_VDE_DE_LINH, DE_LINH,
			((resolution_v >> 8) & 0x7));

	fdb_setb(priv->base, FHT14_VDE_DE_CTRL, HS_POL, !(flags & DISPLAY_FLAGS_HSYNC_HIGH));
	fdb_setb(priv->base, FHT14_VDE_DE_CTRL, VS_POL, !(flags & DISPLAY_FLAGS_VSYNC_HIGH));

	fdb_setb(priv->base, FHT14_VDE_HWIDTH_H, HIGH2, priv->cfg.timings.dt.hsync_len.typ >> 8);
	fdb_setb(priv->base, FHT14_VDE_HWIDTH_L, LOW, priv->cfg.timings.dt.hsync_len.typ & 0xff);
	fdb_setb(priv->base, FHT14_VDE_VWIDTH, WIDTH, priv->cfg.timings.dt.vsync_len.typ);
}

static irqreturn_t f_hdmi_tx14_interrupt(int irq, void *dev)
{
	struct f_hdmi_tx14 *priv = dev;
	u8 stat[4];
	int n;

	for (n = 0; n < 4; n++) {
		stat[n] = __raw_readb(priv->base + FHT14_INTR_INTR1_OFS + n);
		__raw_writeb(stat[n], priv->base + FHT14_INTR_INTR1_OFS + n);
	}

	if (fdb_getval(stat[0], FHT14_INTR_INTR1, SOFT))
		dev_info(priv->dev, "IRQ: SOFT\n");

	if (fdb_getval(stat[0], FHT14_INTR_INTR1, HPD)) {
		dev_info(priv->dev, "IRQ: HPD\n");
		n = fdb_getb(priv->base, FHT14_BASE_SYS_STAT, HPD);
		if (!n)
			priv->private_edid_valid = false;
		if (priv->drm_dev)
			drm_helper_hpd_irq_event(priv->drm_dev);
	}

	if (fdb_getval(stat[0], FHT14_INTR_INTR1, RSEN))
		dev_info(priv->dev, "IRQ: RSEN\n");

	if (fdb_getval(stat[0], FHT14_INTR_INTR1, DROP_SAMPLE))
		dev_info(priv->dev, "IRQ: DROP_SAMPLE\n");

	if (fdb_getval(stat[0], FHT14_INTR_INTR1, BIPHASE_ERROR))
		dev_info(priv->dev, "IRQ: BIPHASE_ERROR\n");

	if (fdb_getval(stat[0], FHT14_INTR_INTR1, RI_128))
		dev_info(priv->dev, "IRQ: RI_128\n");

	if (fdb_getval(stat[0], FHT14_INTR_INTR1, OVERRUN))
		dev_info(priv->dev, "IRQ: OVERRUN\n");

	if (fdb_getval(stat[0], FHT14_INTR_INTR1, UNDERRUN))
		dev_info(priv->dev, "IRQ: UNDERRUN\n");

	if (fdb_getval(stat[1], FHT14_INTR_INTR2, SPDIF_PAR))
		dev_info(priv->dev, "IRQ: SPDIF_PAR\n");

	if (fdb_getval(stat[1], FHT14_INTR_INTR2, ENC_DIS))
		dev_info(priv->dev, "IRQ: ENC_DIS\n");

	if (fdb_getval(stat[1], FHT14_INTR_INTR2, PREAM_ERR))
		dev_info(priv->dev, "IRQ: PREAM_ERR\n");

	if (fdb_getval(stat[1], FHT14_INTR_INTR2, CTS_CHG))
		dev_info(priv->dev, "IRQ: CTS_CHG\n");

	if (fdb_getval(stat[1], FHT14_INTR_INTR2, ACR_OVR))
		dev_info(priv->dev, "IRQ: ACR_OVR\n");

	if (fdb_getval(stat[1], FHT14_INTR_INTR2, TCLK_STBL))
		dev_info(priv->dev, "IRQ: TCLK_STBL\n");

	if (fdb_getval(stat[1], FHT14_INTR_INTR2, VSYNC_REC))
		dev_info(priv->dev, "IRQ: VSYNC_REC\n");

	if (fdb_getval(stat[2], FHT14_INTR_INTR3, RI_ERR3))
		dev_info(priv->dev, "IRQ: RI_ERR3\n");

	if (fdb_getval(stat[2], FHT14_INTR_INTR3, RI_ERR2))
		dev_info(priv->dev, "IRQ: RI_ERR2\n");

	if (fdb_getval(stat[2], FHT14_INTR_INTR3, RI_ERR1))
		dev_info(priv->dev, "IRQ: RI_ERR1\n");

	if (fdb_getval(stat[2], FHT14_INTR_INTR3, RI_ERR0))
		dev_info(priv->dev, "IRQ: RI_ERR0\n");

	if (fdb_getval(stat[2], FHT14_INTR_INTR3, DDC_CMD_DONE))
		dev_info(priv->dev, "IRQ: DDC_CMD_DONE\n");

	if (fdb_getval(stat[2], FHT14_INTR_INTR3, DDC_FIFO_HALF))
		dev_info(priv->dev, "IRQ: DDC_FIFO_HALF\n");

	if (fdb_getval(stat[2], FHT14_INTR_INTR3, DDC_FIFO_FULL))
		dev_info(priv->dev, "IRQ: DDC_FIDO_FULL\n");

	if (fdb_getval(stat[2], FHT14_INTR_INTR3, DDC_FIFO_EMPTY))
		dev_info(priv->dev, "IRQ: DDC_FIFO_EMPTY\n");

	if (fdb_getval(stat[3], FHT14_INTR_INTR4, INTR4_STAT3))
		dev_info(priv->dev, "IRQ: INTR4_STAT3\n");

	if (fdb_getval(stat[3], FHT14_INTR_INTR4, INTR4_STAT2))
		dev_info(priv->dev, "IRQ: INTR4_STAT2\n");

	if (fdb_getval(stat[3], FHT14_INTR_INTR4, INTR4_STAT1))
		dev_info(priv->dev, "IRQ: INTR4_STAT1\n");

	return IRQ_HANDLED;
}

static int f_hdmi_tx14_get_connector_type(struct f_fdb_child *fdb_child)
{
	return DRM_MODE_CONNECTOR_HDMIA;
}

static int f_hdmi_tx14_is_hdmi_mode(struct f_hdmi_tx14 *priv)
{
	u8 val;

	val = fdb_getb(priv->base, FHT14_ACR_HDMI_CTRL, HDMI_MODE);
	dev_info(priv->dev, "[%s] Is HDMI mode:%d\n", __FUNCTION__, val);
	return !!val;
}
#if 0
static void f_hdmi_tx14_mode_enable(struct f_hdmi_tx14 *priv, u8 enable)
{
	fdb_setb(priv->base, FHT14_ACR_HDMI_CTRL, HDMI_MODE, enable);
}
#endif
static void f_hdmi_tx14_sendcp_packet(struct f_hdmi_tx14 *priv, u8 on)
{
	u8 val;
	int timeout = 1280;
	/* Send CP packets only if in HDMI mode */
	if (!f_hdmi_tx14_is_hdmi_mode(priv))
		return;
	val = fdb_getb(priv->base, FHT14_PKT_PB_CTRL2, CP_EN);
	if (val & 0x08) { /* CP Enable */
		val = fdb_getb(priv->base, FHT14_PKT_GP_BYTE1, AVM);
		if (on) {
			if (val == 0x01) /* Already Mute set */
				return;
		} else {
			if (val == 0x10) /* Already Mute cleared */
				return;
		}
	}

	fdb_setb(priv->base, FHT14_PKT_PB_CTRL2, CP_EN, 0);
	fdb_setb(priv->base, FHT14_PKT_PB_CTRL2, CP_REPEAT, 0);
	while (timeout > 0) {
		val = fdb_getb(priv->base, FHT14_PKT_PB_CTRL2, CP_EN);
		if (!(val & 0x08))
			break;
		udelay(50);
		timeout--;
	}

	if (!timeout) {
		dev_err(priv->dev, "timeout INF_CTRL2 %x time %d\n",
							val, timeout);
		return;
	}

#if 0 /* It will destory some sensitive HDMI */
	if (on)
		fdb_setb(priv->base, FHT14_PKT_GP_BYTE1, SETAVM, 1);
	else
		fdb_setb(priv->base, FHT14_PKT_GP_BYTE1, CLRAVM, 1);
#endif

	fdb_setb(priv->base, FHT14_PKT_PB_CTRL2, CP_EN, 1);
	fdb_setb(priv->base, FHT14_PKT_PB_CTRL2, CP_REPEAT, 1);
	//dev_info(priv->dev, "write INF_CTRL2 CP_EN, CP_REPEAT done. time %d\n",
	//							timeout);
}

static void f_hdmi_tx14_phy_pwr_ctrl(struct f_hdmi_tx14 *priv, u8 power_state)
{
	/* Power Down the Phy(PD# assert) */
	if (power_state == 0) {
		dev_info(priv->dev, "mute on by phy power down\n");
		f_hdmi_tx14_sendcp_packet(priv, 1);
		fdb_setb(priv->base, FHT14_BASE_SYS_CTRL1, NPD, 0);
		dev_info(priv->dev, "PHY powered down\n");
	} else {
		fdb_setb(priv->base, FHT14_BASE_SYS_CTRL1, NPD, 1);
		dev_info(priv->dev, "mute off by phy power up\n");
		f_hdmi_tx14_sendcp_packet(priv, 0);
		dev_info(priv->dev, "PHY powered up\n");
	}
}

static void f_hdmi_tx14_pwr_ctrl(struct f_hdmi_tx14 *priv, u8 power_state)
{
	if (power_state == 0) {
		fdb_setb(priv->base, FHT14_ACR_DPD, NPOWERDOWN_TOTAL, 0);
		dev_info(priv->dev, "ip powered down\n");
	} else {
		fdb_setb(priv->base, FHT14_ACR_DPD, NPOWERDOWN_TOTAL, 1);
		dev_info(priv->dev, "ip powered up\n");
	}
}

static void f_hdmi_tx14_mddc_init(struct f_hdmi_tx14 *priv)
{
	dev_info(priv->dev, "Master DDC Init\n");
	fdb_setb(priv->base, FHT14_DDC_DDC_CMD, DDC_CMD,
						DDC_CMD__ABORT_TRANSACTION);
	msleep(1);
	fdb_setb(priv->base, FHT14_DDC_DDC_CMD, DDC_CMD, DDC_CMD__CLEAR_FIFO);
	msleep(1);
	fdb_setb(priv->base, FHT14_DDC_DDC_CMD, DDC_CMD, DDC_CMD__CLOCK_SCL);
	msleep(1);
}

static void f_hdmi_tx14_wakeup(struct f_hdmi_tx14 *priv)
{
	fdb_setb(priv->base, FHT14_BASE_SYS_CTRL1, NPD, 1);
	/* Interrupt pin polarity: Assertion HIGH */
	fdb_setb(priv->base, FHT14_INTR_INT_CTRL, NPOLARITY, 0);
}

static void f_hdmi_tx14_swreset(struct f_hdmi_tx14 *priv)
{
	int timeout = 255;
	int phy_pwr_on;
	u8 val;

	phy_pwr_on = fdb_getb(priv->base, FHT14_BASE_SYS_CTRL1, NPD);
	while (timeout > 0) { /* wait for input pixel clock to stabilized */
		val = fdb_getb(priv->base, FHT14_BASE_SYS_STAT, P_STABLE);
		if (val & 0x1)
			break;
		timeout--;
	}

	/* Assert SW Reset */
	fdb_setb(priv->base, FHT14_BASE_SRST, SWRST, 1);
	fdb_setb(priv->base, FHT14_BASE_SRST, FIFORST, 1);
	val = fdb_getb(priv->base, FHT14_BASE_SYS_STAT, P_STABLE);
	dev_info(priv->dev, "STAT after RST=1: %02x\n", val);
	/* Mask RSEN and HPD INT */
	fdb_setb(priv->base, FHT14_INTR_INT_UNMASK1, RSEN, 0);
	fdb_setb(priv->base, FHT14_INTR_INT_UNMASK1, HPD, 0);
	f_hdmi_tx14_phy_pwr_ctrl(priv, 0); /* Phy Power Down */
	msleep(1);
	if (phy_pwr_on) {
		dev_info(priv->dev, "swreset(), phy_pwr_on=1\n");
		f_hdmi_tx14_phy_pwr_ctrl(priv, 1); /* Phy Power On */
	}

	/* Release SW Reset */
	fdb_setb(priv->base, FHT14_BASE_SRST, SWRST, 0);
	fdb_setb(priv->base, FHT14_BASE_SRST, FIFORST, 0);
	val = fdb_getb(priv->base, FHT14_BASE_SYS_STAT, P_STABLE);
	dev_info(priv->dev, "STAT after RST=0: %02x\n", val);

	/* UnMask RSEN and HPD INT */
	fdb_setb(priv->base, FHT14_INTR_INTR1, RSEN, 1);
	fdb_setb(priv->base, FHT14_INTR_INTR1, HPD, 1);
	fdb_setb(priv->base, FHT14_INTR_INT_UNMASK1, RSEN, 1);
	fdb_setb(priv->base, FHT14_INTR_INT_UNMASK1, HPD, 1);

	/* allow TCLK (sent to Rx across the HDMS link) to stabilize*/
	msleep(64);
	dev_info(priv->dev, "swreset done!!\n");
}

static void f_hdmi_tx14_no_hdcp(struct f_hdmi_tx14 *priv)
{
	/* Send zero in audio packet */
	
	// !!! uh... don't support hdcp, so let the audio play...
	fdb_setb(priv->base, FHT14_BASE_SYS_DCTL, AUD_MUTE, 0);
	f_hdmi_tx14_sendcp_packet(priv, 0);
}

static void f_hdmi_tx14_hdmitx_init(struct f_hdmi_tx14 *priv)
{
	dev_info(priv->dev, "HDMI TX Init - PD# %02x\n",
			fdb_getb(priv->base, FHT14_BASE_SYS_STAT, P_STABLE));
	fdb_setb(priv->base, FHT14_TMDS_TMDS_CTRL, TCLKSEL, 1);
	f_hdmi_tx14_wakeup(priv);
	#if 1
	f_hdmi_tx14_swreset(priv);
	#endif
	dev_info(priv->dev, "[%s] Done!\n", __FUNCTION__);
}

static int f_hdmi_tx14_hw_init(struct f_hdmi_tx14 *priv)
{
	int n;
	u8 val;

	/* Check IDCK to TMDS Clock Stable */
	for (n = 0; n < 200; n++) {
		val = fdb_getb(priv->base, FHT14_BASE_SYS_STAT, P_STABLE);
		if (val & 0x1)
			break;
		udelay(1);
	}
//	dev_info(priv->dev, "Txwait %d times, reg: %02x\n", n, val);

	f_hdmi_tx14_phy_pwr_ctrl(priv, 1); /* Phy Power On */
	f_hdmi_tx14_pwr_ctrl(priv, 1); /* Power On */
	f_hdmi_tx14_mddc_init(priv);
	f_hdmi_tx14_hdmitx_init(priv);

	dev_info(priv->dev, "[%s] Done!\n", __FUNCTION__);
	return 0;
}

static void f_hdmi_tx14_basic_configure(struct f_hdmi_tx14 *priv)
{
	/* Before transmmit, basic setting for video timing and info-frame */
	struct f_hdmi_video_config v_core_cfg;
	struct f_hdmi_packet_enable_repeat repeat_cfg;
	struct f_hdmi_infoframe_avi avi_cfg = priv->avi_cfg;
	u8 regvalue_1 = 0;
	u8 regvalue_2 = 0;

	/* video config */
	v_core_cfg.tclk_sel_clkmult = HDMI_FPLL10IDCK;
	v_core_cfg.pkt_mode = HDMI_PACKETMODE24BITPERPIXEL;
	v_core_cfg.deep_color = HDMI_DEEPCOLORPACKECTDISABLE;
	v_core_cfg.hdmi_dvi = HDMI_HDMI;
	v_core_cfg.buswidth = HDMI_INPUT_8BIT;
	v_core_cfg.clipcs = HDMI_OUTPUT_RGB;
	v_core_cfg.rangclip = HDMI_RANGECLIP_DISABLE;
	v_core_cfg.rgb2ycbcr = HDMI_RBG2YCBCR_DISABLE;
	v_core_cfg.rangcmps = HDMI_RANGECMPS_DISABLE;
	v_core_cfg.downsmp = HDMI_DOWNSAMPLE_DISABLE;
	v_core_cfg.dithmode = HDMI_OUTPUTDITHER_8BIT;
	v_core_cfg.dither = HDMI_DITHER_DISABLE;
	v_core_cfg.range = HDMI_RANGE_DISABLE;
	v_core_cfg.csc = HDMI_YCBCR2RGB_DISABLE;
	v_core_cfg.upsmp = HDMI_UPSAMPLE_DISABLE;
	v_core_cfg.demux = HDMI_DEMUX_DISABLE;
	v_core_cfg.syncext = HDMI_SYNCEXTRAT_DISABLE;

	/* info frame */
	memset(&avi_cfg, 0, sizeof(avi_cfg));

	/* packet enable and repeat */
	memset(&repeat_cfg, 0, sizeof(repeat_cfg));

	f_hdmi_wp_video_config_timing(priv);

	/*
	 * configure core video part
	 * set software reset in the core
	 */
	v_core_cfg.pkt_mode = HDMI_PACKETMODE24BITPERPIXEL;
	v_core_cfg.hdmi_dvi = priv->cfg.cm.mode;

	regvalue_1 = (v_core_cfg.syncext | v_core_cfg.demux | v_core_cfg.upsmp | v_core_cfg.csc
				| v_core_cfg.range | v_core_cfg.dither | v_core_cfg.dithmode);
	fdb_setb(priv->base, FHT14_VDE_VID_MODE, VALUE, regvalue_1);
	/* Latch input on rising edge */
	fdb_setb(priv->base, FHT14_BASE_SYS_CTRL1, EDGE, 1);

	regvalue_2 = (v_core_cfg.downsmp | v_core_cfg.rangcmps | v_core_cfg.rgb2ycbcr
				| v_core_cfg.rangclip | v_core_cfg.clipcs | v_core_cfg.buswidth);
	fdb_setb(priv->base, FHT14_VDE_VID_ACEN, VALUE, regvalue_2);

	/* Color Space */
	fdb_setb(priv->base, FHT14_VDE_VID_CTRL, CSCSEL,
				priv->cfg.timings.dt.vactive.typ >= 720);
	/* Packet Mode */
	fdb_setb(priv->base, FHT14_ACR_HDMI_CTRL, PACKET_MODE, v_core_cfg.pkt_mode);
	/* Deep Color */
	fdb_setb(priv->base, FHT14_ACR_HDMI_CTRL, DC_EN, v_core_cfg.deep_color);
	/* HDMI Mode */
	fdb_setb(priv->base, FHT14_ACR_HDMI_CTRL, HDMI_MODE, v_core_cfg.hdmi_dvi);
	/* TMDS_CTRL */
	fdb_setb(priv->base, FHT14_TMDS_TMDS_CTRL, TCLKSEL, v_core_cfg.tclk_sel_clkmult);

	/* Must be done AFTER setting up audio and video paths
		 and BEFORE starting to send InfoFrames */
	f_hdmi_tx14_swreset(priv);

	/* TBD: If needed, set iclk/mclk for video and audio */

	/*
	 * configure packet
	 * info frame video see doc CEA861-D page 65
	 */
	avi_cfg.db1_format = HDMI_INFOFRAME_AVI_DB1Y_RGB;
	avi_cfg.db1_active_info =
		HDMI_INFOFRAME_AVI_DB1A_ACTIVE_FORMAT_OFF;
	avi_cfg.db1_bar_info_dv = HDMI_INFOFRAME_AVI_DB1B_NO;
	avi_cfg.db1_scan_info = HDMI_INFOFRAME_AVI_DB1S_0;
	avi_cfg.db2_colorimetry = HDMI_INFOFRAME_AVI_DB2C_NO;
	avi_cfg.db2_aspect_ratio = HDMI_INFOFRAME_AVI_DB2M_NO;
	avi_cfg.db2_active_fmt_ar = HDMI_INFOFRAME_AVI_DB2R_SAME;
	avi_cfg.db3_itc = HDMI_INFOFRAME_AVI_DB3ITC_NO;
	avi_cfg.db3_ec = HDMI_INFOFRAME_AVI_DB3EC_XVYUV601;
	avi_cfg.db3_q_range = HDMI_INFOFRAME_AVI_DB3Q_DEFAULT;
	avi_cfg.db3_nup_scaling = HDMI_INFOFRAME_AVI_DB3SC_NO;
	avi_cfg.db4_videocode = priv->cfg.cm.code;
	avi_cfg.db5_pixel_repeat = HDMI_INFOFRAME_AVI_DB5PR_NO;
	avi_cfg.db6_7_line_eoftop = 0;
	avi_cfg.db8_9_line_sofbottom = 0;
	avi_cfg.db10_11_pixel_eofleft = 0;
	avi_cfg.db12_13_pixel_sofright = 0;

	f_hdmi_core_aux_infoframe_avi_config(priv);

	/* enable/repeat the infoframe */
	repeat_cfg.avi_infoframe = HDMI_PACKETENABLE;
	repeat_cfg.avi_infoframe_repeat = HDMI_PACKETREPEATON;
	repeat_cfg.audio_pkt = HDMI_PACKETENABLE;
	repeat_cfg.audio_pkt_repeat = HDMI_PACKETREPEATON;
	f_hdmi_core_av_packet_config(priv, repeat_cfg);

	hdmitx14_dump_regs(priv);

	dev_info(priv->dev, "[%s] Done!\n", __FUNCTION__);
}

static struct f_hdmi_cm f_hdmi_get_code(struct fdb_video_timings *timing)
{
	struct f_hdmi_cm cm = { -1 , HDMI_DVI };

	u32 pixel_clk = timing->dt.pixelclock.typ; /* in kHz */
	int i;

	for (i = 0; i < ARRAY_SIZE(cea_timings); i++)
		/* pr_info("%d: %d/%d %d/%d %d/%d %d/%d\n", i,
		 timing->dt.hsync_len.typ, cea_timings[i].hsw,
		 timing->dt.hback_porch.typ, cea_timings[i].hbp,
		 timing->dt.hfront_porch.typ, cea_timings[i].hfp,
			    pixel_clk, cea_timings[i].pixel_clock); */

		if (
		 timing->dt.hsync_len.typ == cea_timings[i].hsw &&
		 timing->dt.hback_porch.typ == cea_timings[i].hbp &&
		 timing->dt.hfront_porch.typ == cea_timings[i].hfp &&
			    pixel_clk == cea_timings[i].pixel_clock) {
			cm.mode = cea_timings[i].mode;
			cm.code = cea_timings[i].code;
			pr_info("f_hdmi_get_code: MATCH , Code = %d , Mode = %s\n",
				cm.code, cm.mode ? "CEA" : "DMT");
			return cm;
		}

	pr_info("f_hdmi_get_code: REJECT, This Code/Mode is out of supported list.\n");
	return cm;
}

#if 0
static const struct f_hdmi_config *f_hdmi_get_timings(struct f_hdmi_tx14 *priv)
{
	int i;

	dev_info(priv->dev, "++++++++ f_hdmi_get_timings ++++++++++\n");

	for (i = 0; i < ARRAY_SIZE(f_cea_timings); i++) {
		if (f_cea_timings[i].cm.code == priv->cfg.cm.code)
				return &f_cea_timings[i];
	}

	return NULL;
}
#endif


static int f_hdmi_tx14_check_timings(struct f_fdb_child *fdb_child,
					struct fdb_video_timings *timings)
{
	struct f_hdmi_cm cm;
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);
	int ret = 0;

	mutex_lock(&priv->lock);

	/* with CEA timings we can use audio, so let's prefer them */
	cm = f_hdmi_get_code(timings);
	if (cm.code == -1) {
		ret = -EINVAL;
		goto bail;
	}
	if (!priv->bound)
		goto bail;
	if (!priv->bound->ops->check_timings)
		goto bail;

	ret = priv->bound->ops->check_timings(priv->bound, timings);

bail:
	mutex_unlock(&priv->lock);

	return ret;
}

static bool f_hdmi_tx14_detect(struct f_fdb_child *fdb_child)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);
	u8 val;

	val = fdb_getb(priv->base, FHT14_BASE_SYS_STAT, HPD);
	// dev_info(priv->dev, "hotplug detect = %d\n", val);

	return !!val;
}

static int f_hdmi_tx14_ddc_edid(struct f_fdb_child *fdb_child,
							u8 *edid, int ext)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);
	u32 i;
	int n;
	u8 csum = 0;
	u32 offset = 0;
	int timeout = 100;
	int tries = 5;

	while (tries--) {
	
	while (fdb_getb(priv->base, FHT14_DDC_DDC_STATUS, IN_PROG) && --timeout)
		udelay(100);

	if (!timeout)
		dev_info(priv->dev, "DDC problem status=0x%x\n",
			__raw_readb(priv->base + FHT14_DDC_DDC_STATUS_OFS));

	/* Abort Master DCC operation and Clear FIFO pointer */
	fdb_setb(priv->base, FHT14_DDC_DDC_CMD, DDC_CMD, DDC_CMD__CLEAR_FIFO);

	if (ext & 1)
		offset = 0x80;

	/* Set Slave Address Register */
	fdb_setb(priv->base, FHT14_DDC_DDC_ADDR, ADDR, EDID_SLV);

	/* Set Segment Address Register */
	fdb_setb(priv->base, FHT14_DDC_DDC_SEGM, SEGM, ext >> 1);

	/* Set Offset Address Register */
	fdb_setb(priv->base, FHT14_DDC_DDC_OFFSET, OFFSET, offset);

	/* Set Data Count Address Register */
	fdb_setb(priv->base, FHT14_DDC_DDC_COUNT_LOW, COUNT1, 0x80);
	fdb_setb(priv->base, FHT14_DDC_DDC_COUNT_UP, COUNT2, 0);

	/* Set DDC_CMD */
	if (ext)
		fdb_setb(priv->base, FHT14_DDC_DDC_CMD, DDC_CMD,
				DDC_CMD__ENHANCED_DDC_READ_NO_ACK_ON_LAST);
	else
		fdb_setb(priv->base, FHT14_DDC_DDC_CMD,
				DDC_CMD, DDC_CMD__SEQ_READ_NO_ACK_ON_LAST);

	/* DDC_STATUS_BUS_LOW */
	if (fdb_getb(priv->base, FHT14_DDC_DDC_STATUS, BUS_LOW)) {
		fdb_setb(priv->base, FHT14_DDC_DDC_CMD, DDC_CMD,
						DDC_CMD__ABORT_TRANSACTION);

		fdb_setb(priv->base, FHT14_DDC_DDC_CMD, DDC_CMD,
							DDC_CMD__CLOCK_SCL);

		fdb_setb(priv->base, FHT14_DDC_DDC_MAN, SCL, 0);
		fdb_setb(priv->base, FHT14_DDC_DDC_MAN, SDA, 0);
		if (!tries) {
			dev_err(priv->dev, "I2C Bus Low?\n");
			return -EIO;
		} else
			goto retry;
	}
	/* DDC_STATUS_NO_ACK */
	if (fdb_getb(priv->base, FHT14_DDC_DDC_STATUS, NO_ACK)) {
		if (!tries) {
			dev_err(priv->dev, "I2C No ACK\n");
			return -EIO;
		}
	}

	break;

retry:
		msleep(200);
	}

	if (tries < 0)
		return -EIO;

	timeout = 500;
	i = 0;
	while (timeout && i < 0x80) {
		n = __raw_readb(priv->base + FHT14_DDC_DDC_FIFOCNT_OFS);
		n = fdb_const(FHT14_DDC_DDC_FIFOCNT, FIFOCONT, n);
		if (n) {
			while (n-- && i != 0x80)
				edid[i++] = fdb_getb(priv->base,
						FHT14_DDC_DDC_DATA, DDC_DATA);
			continue;
		}
		udelay(100);
		timeout--;
	}

	if (!timeout) {
		dev_info(priv->dev, "DDC BUSY status=0x%xi, read %d\n",
			__raw_readb(priv->base + FHT14_DDC_DDC_STATUS_OFS), i);
		return -EIO;
	}

	print_hex_dump(KERN_INFO, "",
			DUMP_PREFIX_NONE, 16, 1, edid, 0x80, true);

	csum = 0;
	for (i = 0; i < 0x80; i++)
		csum += edid[i];

	if (!csum)
		return 0;

	dev_err(priv->dev, "E-EDID checksum failed!!\n");

	return -EIO;
}

static int f_hdmi_tx14_read_edid(struct f_fdb_child *fdb_child,
							u8 *buf, int len)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);
	int r, l, j = 0, valid_extensions = 0;
	u8 ext_num;
	int ret = 0;

	mutex_lock(&priv->lock);

	dev_info(priv->dev, "f_hdmi_tx14_read_edid: ---------->>\n");

	if (priv->private_edid_valid)
		goto done;

	if (len < 128) {
		ret = -EINVAL;
		goto bail;
	}

	f_hdmi_tx14_mddc_init(priv);
	r = f_hdmi_tx14_ddc_edid(fdb_child, priv->edid, 0);
	if (r) {
		ret = r;
		goto bail;
	}

	l = 128;
	ext_num = priv->edid[0x7e];

	if (len >= 128 * ext_num && ext_num > 0)
		for (j = 1; j <= ext_num; j++) {
			r = f_hdmi_tx14_ddc_edid(fdb_child,
				priv->edid + ((valid_extensions + 1) * 128), j);
			if (r) {
				ret = r;
				goto bail;
			}

			valid_extensions++;
			l += 128;
		}

	priv->private_edid_valid = true;
done:
	memcpy(buf, priv->edid, len);
	ret = len;
bail:

	dev_info(priv->dev, "<<----------- f_hdmi_tx14_read_edid\n");
	mutex_unlock(&priv->lock);

	return ret;
}

static u32 f_hdmi_tx14_get_source_crtc_bitfield(struct f_fdb_child *fdb_child)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	return priv->source_crtc_bitfield;
}

static int f_hdmi_tx14_bind_to_source(struct f_fdb_child *fdb_child,
						struct f_fdb_child *fdb_bound)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	priv->bound = fdb_bound;
	fdb_bound->reverse_binding = fdb_child;
	dev_info(fdb_child->dev, "****** binding to source %s\n",
					dev_name(fdb_bound->dev));

	return 0;
}

static int f_hdmi_tx14_hdmi_enable(struct f_fdb_child *fdb_child)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);
	int ret = 0;

	if (!priv->cfg.timings.dt.pixelclock.typ)
		return 0;

	mutex_lock(&priv->lock);

	dev_info(priv->dev, "f_hdmi_tx14_hdmi_enable --------------->>\n");

	ret = f_hdmi_tx14_hw_init(priv);
	if (ret)
		goto bail;

	f_hdmi_tx14_basic_configure(priv);
	f_hdmi_wp_video_start(priv);
	/* no HDCP authentication */
	f_hdmi_tx14_no_hdcp(priv);

bail:
	mutex_unlock(&priv->lock);
	dev_info(priv->dev, "<<------------ f_hdmi_tx14_hdmi_enable\n");

	return ret;
}

static void f_hdmi_tx14_hdmi_disable(struct f_fdb_child *fdb_child)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	mutex_lock(&priv->lock);
	dev_info(priv->dev, "f_hdmi_tx14_hdmi_disable --------------->>\n");

	f_hdmi_wp_video_stop(priv);
	f_hdmi_core_powerdown_disable(priv);

	mutex_unlock(&priv->lock);
	dev_info(priv->dev, "<<------------ f_hdmi_tx14_hdmi_disable\n");
}

static int f_hdmi_tx14_set_hdmi_dvi_mode(struct f_fdb_child *fdb_child, u8 mode)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	priv->mode = mode;

	return 0;
}

int f_hdmi_tx14_audio_enable(struct f_fdb_child *fdb_child)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);
	
	/* I2S channel0 enable */
	fdb_setb(priv->base, FHT14_ACRIN_AUD_MODE, SD0_EN, 1);	
	/* CTS Request enable */
	fdb_setb(priv->base, FHT14_ACR_ACR_CTRL, NCTS_PKT_EN, 1);

	return 0;
}

void f_hdmi_tx14_audio_disable(struct f_fdb_child *fdb_child)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	/* I2S channel0 disable */
	fdb_setb(priv->base, FHT14_ACRIN_AUD_MODE, SD0_EN, 0);
	/* CTS Request disable */
	fdb_setb(priv->base, FHT14_ACR_ACR_CTRL, NCTS_PKT_EN, 0);

}

static int f_hdmi_tx14_audio_start(struct f_fdb_child *fdb_child)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	/* Audio input stream enable */
	fdb_setb(priv->base, FHT14_ACRIN_AUD_MODE, AUD_EN, 1);

	return 0;
}

static void f_hdmi_tx14_audio_stop(struct f_fdb_child *fdb_child)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	/* Audio input stream disable */
	fdb_setb(priv->base, FHT14_ACRIN_AUD_MODE, AUD_EN, 0);
}

static void f_hdmi_tx14_audio_format_cfg(struct f_hdmi_tx14 *priv,
					struct f_hdmi_audio_format *aud_fmt,
					struct f_hdmi_audio_config *aud_cfg)
{
	void __iomem *base = priv->base;
	int n, N;
	u8 r2, chst4, chst5, r5, r6;

	f_hdmi_write_reg(base, FHT14_ACRIN_I2S_CHST3_OFS, 2);

	/* send forced N/CTS numbers */
	f_hdmi_write_reg(base, FHT14_ACR_ACR_CTRL_OFS, 1);
	r2 = f_hdmi_read_reg(base, FHT14_ACRIN_AUD_MODE_OFS);
	chst4 = f_hdmi_read_reg(base, FHT14_ACRIN_I2S_CHST4_OFS);
	chst5 = f_hdmi_read_reg(base, FHT14_ACRIN_I2S_CHST5_OFS);
	r5 = f_hdmi_read_reg(base, FHT14_ACRIN_I2S_IN_LEN_OFS);
	r6 = f_hdmi_read_reg(base, FHT14_ACRIN_ASRC_OFS);

	/* audio input mode */
	r2 |= aud_cfg->i2s_cfg.active_sds;
	/* sampling freq. */
	chst4 = (0 << 4) | aud_fmt->sample_rate;
	/* original Fs */
	chst5 = (aud_fmt->sample_rate << 4) | aud_fmt->sample_size;
	/* sample size */
	r5 = (r5 & 0xf0) | aud_fmt->sample_size;
	/* disable audio sample rate conversion */
	r6 = r6 & 0xf0;
	
	N = 6144;
	/* ie, 148500 for 1080p, 74250 for 720p etc */
	n = priv->cfg.timings.dt.pixelclock.typ;
	switch (aud_fmt->sample_rate) {
	case 3: /* 32kHz */
		N = 4096;
		break;
	case 0: /* 44.1kHz */
		n = 165000 / (148500 / n);
		N = 6272;
		break;
	case 2: /* 48kHz */
	default:
		break;

	}

	f_hdmi_write_reg(base, 0x106, n);
	f_hdmi_write_reg(base, 0x107, n >> 8);
	f_hdmi_write_reg(base, 0x108, n >> 16);

	f_hdmi_write_reg(base, FHT14_ACR_N_SVAL_LOW_OFS, N);
	f_hdmi_write_reg(base, FHT14_ACR_N_SVAL_MID_OFS, N >> 8);
	f_hdmi_write_reg(base, FHT14_ACR_N_SVAL_UP_OFS, N >> 16);

	f_hdmi_write_reg(priv->base, 0x11b, 0xd0);

	f_hdmi_write_reg(base, FHT14_ACR_FREQ_SVAL_OFS, aud_cfg->mclk_mode);
	
	f_hdmi_write_reg(base, FHT14_ACRIN_AUD_MODE_OFS, r2);
	f_hdmi_write_reg(base, FHT14_ACRIN_I2S_CHST4_OFS, chst4);
	f_hdmi_write_reg(base, FHT14_ACRIN_I2S_CHST5_OFS, chst5);

	f_hdmi_write_reg(base, FHT14_ACRIN_I2S_IN_LEN_OFS, r5);
	f_hdmi_write_reg(base, FHT14_ACRIN_ASRC_OFS, r6);

	f_hdmi_write_reg(base, 0x123, 0);
	
	fdb_setb(base, FHT14_ACR_HDMI_CTRL,
		LAYOUT, aud_fmt->stereo_channels != HDMI_AUDIO_STEREO_ONECHANNEL);

	fdb_setb(base, FHT14_ACRIN_I2S_IN_CTRL, I2S_JUST,
		 	aud_fmt->justification != HDMI_AUDIO_JUSTIFY_LEFT);

	fdb_setb(base, FHT14_ACRIN_I2S_IN_CTRL, VBIT,
					aud_fmt->type != HDMI_AUDIO_TYPE_LPCM);

	fdb_setb(base, FHT14_ACRIN_I2S_IN_CTRL, I2S_SHIFT, 0);
	fdb_setb(base, FHT14_ACRIN_I2S_IN_CTRL, SCK_EDGE, 0);
}

static void f_hdmi_tx14_audio_infoframe_cfg(struct f_hdmi_tx14 *priv,
		struct snd_cea_861_aud_if *info_aud)
{
	u8 sum = 0, checksum = 0;
	void __iomem *av_base = priv->base;

	pr_err("%s\n", __func__);

	/*
	 * Set audio info frame type, version and length as
	 * described in HDMI 1.4a Section 8.2.2 specification.
	 * Checksum calculation is defined in Section 5.3.5.
	 */
	f_hdmi_write_reg(av_base, FHT14_PKT_AUD_TYPE_OFS, 0x84);
	f_hdmi_write_reg(av_base, FHT14_PKT_AUD_VERS_OFS, 1);
	f_hdmi_write_reg(av_base, FHT14_PKT_AUD_LEN_OFS, 0xa);
	sum += 0x84 + 1 + 0xa;

	info_aud->db1_ct_cc = 1 << 4 | 1;
	info_aud->db2_sf_ss = 3 << 2;
	info_aud->db3 = 0;
	info_aud->db4_ca = 0;
	info_aud->db5_dminh_lsv = 0;
	
	f_hdmi_write_reg(av_base, FHT14_PKT_AUD_DBYTE1_OFS,
		       info_aud->db1_ct_cc);
	sum += info_aud->db1_ct_cc;

	f_hdmi_write_reg(av_base, FHT14_PKT_AUD_DBYTE2_OFS,
		       info_aud->db2_sf_ss);
	sum += info_aud->db2_sf_ss;

	f_hdmi_write_reg(av_base, FHT14_PKT_AUD_DBYTE3_OFS, info_aud->db3);
	sum += info_aud->db3;

	f_hdmi_write_reg(av_base, FHT14_PKT_AUD_DBYTE4_OFS, info_aud->db4_ca);
	sum += info_aud->db4_ca;

	f_hdmi_write_reg(av_base, FHT14_PKT_AUD_DBYTE5_OFS,
		       info_aud->db5_dminh_lsv);
	sum += info_aud->db5_dminh_lsv;

	f_hdmi_write_reg(av_base, FHT14_PKT_AUD_DBYTE6_OFS, 0x00);
	f_hdmi_write_reg(av_base, FHT14_PKT_AUD_DBYTE7_OFS, 0x00);
	f_hdmi_write_reg(av_base, FHT14_PKT_AUD_DBYTE8_OFS, 0x00);
	f_hdmi_write_reg(av_base, FHT14_PKT_AUD_DBYTE9_OFS, 0x00);
	f_hdmi_write_reg(av_base, FHT14_PKT_AUD_DBYTE10_OFS, 0x00);

	checksum = 0x100 - sum;
	f_hdmi_write_reg(av_base, FHT14_PKT_AUD_CHSUM_OFS, checksum);

	/*
	 * TODO: Add MPEG and SPD enable and repeat cfg when EDID parsing
	 * is available.
	 */
}

int f_hdmi_tx14_audio_config(struct f_fdb_child *fdb_child,
		struct f_fdb_audio *audio)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);
	struct f_hdmi_audio_format audio_format;
	/*struct hdmi_audio_dma audio_dma;*/
	struct f_hdmi_audio_config audio_config;
	int /*err, n, cts,*/ channel_count;
	/*unsigned int fs_nr;*/
	bool word_length_16b = false;

	if (!audio || !audio->cea /*|| !audio->iec || !ip_data*/)
		return -EINVAL;

	dev_err(priv->dev, "%s\n", __func__);
	
	/*core.iec60958_cfg = audio->iec;*/
	/*
	 * In the IEC-60958 status word, check if the audio sample word length
	 * is 16-bit as several optimizations can be performed in such case.
	 */
	/*
	if (!(audio->iec->status[4] & IEC958_AES4_CON_MAX_WORDLEN_24))
		if (audio->iec->status[4] & IEC958_AES4_CON_WORDLEN_20_16)
			word_length_16b = true;
	*/
	/* TBD: need comfirm from designer and get data from ASoC dai */
	word_length_16b = true;
	channel_count = 2;

	audio_config.mclk_mode = HDMI_AUDIO_MCLK_512FS; // HDMI_AUDIO_MCLK_128FS;
	audio_format.sample_rate = HDMI_AUDIO_SAMPLE_RATE_48K;

	/* I2S configuration. See Phillips' specification */
	if (word_length_16b)
		audio_config.i2s_cfg.justification = HDMI_AUDIO_JUSTIFY_LEFT;
	else
		audio_config.i2s_cfg.justification = HDMI_AUDIO_JUSTIFY_RIGHT;

	/*
	 * the HDMI IP needs to enable four stereo channels when transmitting
	 * more than 2 audio channels
	 */
	if (channel_count == 2) {
		audio_format.stereo_channels = HDMI_AUDIO_STEREO_ONECHANNEL;
		audio_config.i2s_cfg.active_sds = HDMI_AUDIO_I2S_SD0_EN;
		audio_config.layout = HDMI_AUDIO_LAYOUT_2CH;
	} else {
		audio_format.stereo_channels = HDMI_AUDIO_STEREO_FOURCHANNELS;
		audio_config.i2s_cfg.active_sds = HDMI_AUDIO_I2S_SD0_EN |
				HDMI_AUDIO_I2S_SD1_EN | HDMI_AUDIO_I2S_SD2_EN |
				HDMI_AUDIO_I2S_SD3_EN;
		audio_config.layout = HDMI_AUDIO_LAYOUT_8CH;
	}

	/* use sample frequency from channel status word */
	audio_config.fs_override = false;
	/* enable ACR packets */
	audio_config.en_acr_pkt = true;
	/* disable direct streaming digital audio */
	audio_config.en_dsd_audio = false;
	/* use parallel audio interface */
	audio_config.en_parallel_aud_input = true;

	/* audio I2S format settings */
	if (word_length_16b) {
		audio_format.samples_per_word = HDMI_AUDIO_ONEWORD_TWOSAMPLES;
		audio_format.sample_size = HDMI_AUDIO_SAMPLE_16BITS;
		audio_format.justification = HDMI_AUDIO_JUSTIFY_LEFT;
	} else {
		audio_format.samples_per_word = HDMI_AUDIO_ONEWORD_ONESAMPLE;
		audio_format.sample_size = HDMI_AUDIO_SAMPLE_24BITS;
		audio_format.justification = HDMI_AUDIO_JUSTIFY_RIGHT;
	}
	audio_format.type = HDMI_AUDIO_TYPE_LPCM;
	audio_format.sample_order = HDMI_AUDIO_SAMPLE_LEFT_FIRST;
	/* disable start/stop signals of IEC 60958 blocks */
	/*audio_format.en_sig_blk_strt_end = HDMI_AUDIO_BLOCK_SIG_STARTEND_ON;*/

	/* TBD: configure the audio format */
	f_hdmi_tx14_audio_format_cfg(priv, &audio_format, &audio_config);

	/* configure CEA 861 audio infoframe */
	f_hdmi_tx14_audio_infoframe_cfg(priv, audio->cea);

	return 0;
}

void f_hdmi_tx14_initialized_by_drm(
	struct f_fdb_child *fdb_child, struct drm_device *drm_dev)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	priv->drm_dev = drm_dev;
}



static void f_hdmi_tx14_set_timings(struct f_fdb_child *fdb_child,
					struct fdb_video_timings *timings)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);

	dev_info(priv->dev, "f_hdmi_tx14_set_timings: --------------->>\n");

	mutex_lock(&priv->lock);

	priv->cfg.cm = f_hdmi_get_code(timings);
	priv->cfg.timings = *timings;

	f_hdmi_tx14_basic_configure(priv);
	f_hdmi_wp_video_start(priv);
	/* no HDCP authentication */
	f_hdmi_tx14_no_hdcp(priv);

	dev_info(priv->dev, "%s: %d x %d\n",
			__func__, priv->cfg.timings.dt.hactive.typ,
					      priv->cfg.timings.dt.vactive.typ);

	mutex_unlock(&priv->lock);

	dev_info(priv->dev, "<<-------------- f_hdmi_tx14_set_timings\n");
}

#if defined(CONFIG_DEBUG_FS)

static struct fdb_dumps dumps[] = {
	{ "Audio 1", 0x101, 0x10b - 0x101 },
	{ "Audio 2", 0x114, 0x115 - 0x114 },
	{ "Audio 3", 0x118, 0x119 - 0x118 },
	{ "Audio 4", 0x11b, 0x124 - 0x11b },
	{ "Audio 5", 0x12f, 0x130 - 0x12f },
	{ "Audio 6", 0x13d, 0 },
};

static int f_hdmi_tx14_fdb_debugfs(struct f_fdb_child *fdb_child,
						struct seq_file *m, int type)
{
	struct f_hdmi_tx14 *priv = dev_get_drvdata(fdb_child->dev);
	int i, n;

	switch (type) {
	case FDB_DEBUGFS_INFO:
		break;
	case FDB_DEBUGFS_DUMP:
		//pm_runtime_get_sync(priv->dev);
		seq_printf(m, "struct dump dump_hdmitx14[] = {\n");
		for (i = 0; i < ARRAY_SIZE(dumps); i++)
			for (n = 0; n <= dumps[i].len; n ++)
				seq_printf(m,
					"/* %s: +0x%04x */ { 0x%lx, 0x%02X },\n",
					dumps[i].name,
					dumps[i].start + n,
					(unsigned long)
							dumps[i].start + n,
					__raw_readb(priv->base +
							dumps[i].start + n));
		seq_printf(m, "};\n");
		//pm_runtime_put_sync_suspend(priv->dev);
		break;
	}
	return 0;
}
#endif

struct f_fdb_ops f_hdmi_tx14_fdb_ops = {
	.initialized_by_drm = f_hdmi_tx14_initialized_by_drm,
	.get_connector_type = f_hdmi_tx14_get_connector_type,
	.check_timings = f_hdmi_tx14_check_timings,
	.set_timings = f_hdmi_tx14_set_timings,
	.detect = f_hdmi_tx14_detect,
	.get_source_crtc_bitfield = f_hdmi_tx14_get_source_crtc_bitfield,
	.bind_to_source = f_hdmi_tx14_bind_to_source,
	.read_edid = f_hdmi_tx14_read_edid,
	.enable = f_hdmi_tx14_hdmi_enable,
	.disable = f_hdmi_tx14_hdmi_disable,
	.set_hdmi_dvi_mode = f_hdmi_tx14_set_hdmi_dvi_mode,
	.audio_enable	= f_hdmi_tx14_audio_enable,
	.audio_disable	= f_hdmi_tx14_audio_disable,
	.audio_start = f_hdmi_tx14_audio_start,
	.audio_stop = f_hdmi_tx14_audio_stop,
	.audio_config = f_hdmi_tx14_audio_config,
	/*.audio_supported = f_hdmi_tx14_audio_supported,*/
#if defined(CONFIG_DEBUG_FS)
	.fdb_debugfs = f_hdmi_tx14_fdb_debugfs,
#endif
};


static int
f_hdmi_tx14_probe(struct platform_device *pdev)
{
	struct f_hdmi_tx14 *priv;
	struct resource *res;
	int ret = 0;
	unsigned long rate;
	u8 devid[3];
	int n;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->lock);

	priv->dev = &pdev->dev;
	platform_set_drvdata(pdev, priv);

	/* enable power, clock, IRQ */
	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "get_sync failed with err %d\n", ret);
		goto fail1;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Missing resource\n");
		ret = -EINVAL;
		goto fail2;
	}

	priv->base = ioremap(res->start, res->end - res->start);
	if (!priv->base) {
		ret = -EINVAL;
		goto fail2;
	}

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0) {
		dev_err(&pdev->dev, "no irq resource\n");
		ret = -ENODEV;
		goto fail3;
	}

	fdb_setb(priv->base, FHT14_INTR_INT_CTRL, NPOLARITY, 0);
	fdb_setb(priv->base, FHT14_INTR_INT_UNMASK1, RSEN, 1);
	fdb_setb(priv->base, FHT14_INTR_INT_UNMASK1, HPD, 1);
	//fdb_setb(priv->base, FHT14_INTR_INT_UNMASK2, VSYNC_REC, 1);

	if (of_property_read_u32(pdev->dev.of_node,
				"sources", &priv->source_crtc_bitfield)) {
		dev_err(&pdev->dev, "Missing sources bitfield\n");
		ret = -EINVAL;
		goto fail3;
	}

	priv->pllclk = clk_get(&pdev->dev, "pllclk");
	if (IS_ERR(priv->pllclk)) {
		dev_err(&pdev->dev, "%s(): clock not found.\n", __func__);
		ret = PTR_ERR(priv->pllclk);
		goto fail3;
	}

	priv->pclk = clk_get(&pdev->dev, "pclk");
	if (IS_ERR(priv->pclk)) {
		dev_err(&pdev->dev, "%s(): clock not found.\n", __func__);
		ret = PTR_ERR(priv->pclk);
		goto fail4;
	}

	priv->dpiclk = clk_get(&pdev->dev, "dpiclk");
	if (IS_ERR(priv->dpiclk)) {
		dev_err(&pdev->dev, "%s(): clock not found.\n", __func__);
		ret = PTR_ERR(priv->dpiclk);
		goto fail5;
	}

	rate = clk_get_rate(priv->pllclk);
	dev_info(priv->dev, "[%s] pllclk = %ld\n", __FUNCTION__, rate);

	rate = clk_get_rate(priv->pclk);
	dev_info(priv->dev, "[%s] pclk = %ld\n", __FUNCTION__, rate);

	rate = clk_get_rate(priv->dpiclk);
	dev_info(priv->dev, "[%s] dpiclk = %ld\n", __FUNCTION__, rate);

	ret = clk_prepare_enable(priv->pllclk);
	if (ret < 0)
		goto fail6;

	ret = clk_prepare_enable(priv->pclk);
	if (ret < 0)
		goto fail6;

	ret = clk_prepare_enable(priv->dpiclk);
	if (ret < 0)
		goto fail6;

	for (n = 0; n < 3; n++)
		devid[n] = __raw_readb(priv->base + FHT14_BASE_DEV_IDL_OFS + n);

	dev_info(priv->dev, "HDMI IP: Id 0x%02X.0x%02X. Rev %02i\n",
						devid[0], devid[1], devid[2]);

	/* register our fdb_child with f_fdb bus */
	ret = fdb_register(&pdev->dev, &priv->fdb_child, &f_hdmi_tx14_fdb_ops);
	if (ret < 0) {
		dev_err(&pdev->dev, "fdb registration failed\n");
		goto fail6;
	}

	ret = devm_request_threaded_irq(&pdev->dev, priv->irq, NULL, f_hdmi_tx14_interrupt, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
						pdev->dev.driver->name, priv);
	if (ret) {
		dev_err(&pdev->dev, "failed to allocate irq.\n");
		goto fail7;
	}

	dev_info(&pdev->dev, "f_hdmi_tx14 initialized\n");

	return 0;

fail7:
	fdb_unregister(&pdev->dev, &priv->fdb_child);
fail6:
	clk_put(priv->dpiclk);
fail5:
	clk_put(priv->pclk);
fail4:
	clk_put(priv->pllclk);
fail3:
	iounmap(priv->base);
fail2:
	/*disable power, clock, irq, and suspend hdmi */
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
fail1:
	kfree(priv);


	return ret;
}

static int f_hdmi_tx14_remove(struct platform_device *pdev)
{
	struct f_hdmi_tx14 *priv = platform_get_drvdata(pdev);

	clk_disable_unprepare(priv->pllclk);
	clk_disable_unprepare(priv->pclk);
	clk_disable_unprepare(priv->dpiclk);
	clk_put(priv->pllclk);
	clk_put(priv->pclk);
	clk_put(priv->dpiclk);
	fdb_unregister(&pdev->dev, &priv->fdb_child);
	devm_free_irq(&pdev->dev, priv->irq, priv);
	iounmap(priv->base);
	kfree(priv);

	/*disable power, clock, irq, and suspend hdmi */
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM
static int f_hdmi_tx14_pm_suspend(struct device *dev)
{
	struct platform_device *pdev;
	struct f_hdmi_tx14 *priv;

	pdev = to_platform_device(dev);
	priv = platform_get_drvdata(pdev);

	disable_irq(priv->irq);

	f_hdmi_tx14_hdmi_disable(&priv->fdb_child);

	clk_disable_unprepare(priv->dpiclk);

	clk_disable_unprepare(priv->pclk);

	clk_disable_unprepare(priv->pllclk);

	return 0;

}

static int f_hdmi_tx14_pm_resume(struct device *dev)
{
	struct platform_device *pdev;
	struct f_hdmi_tx14 *priv;
	int ret = 0;

	pdev = to_platform_device(dev);
	priv = platform_get_drvdata(pdev);

	ret = clk_prepare_enable(priv->pllclk);
	if (ret < 0)
		goto fail;

	ret = clk_prepare_enable(priv->pclk);
	if (ret < 0)
		goto fail;

	ret = clk_prepare_enable(priv->dpiclk);
	if (ret < 0)
		goto fail;

	ret = f_hdmi_tx14_hdmi_enable(&priv->fdb_child);
	if (ret < 0)
		goto fail;

	enable_irq(priv->irq);

	return 0;

fail:
	dev_err(&pdev->dev, "resume failed\n");
	return ret;
}

static int f_hdmi_tx14_suspend(struct platform_device *pdev, pm_message_t msg)
{
	/* suspend here */
	return 0;
}

static int f_hdmi_tx14_resume(struct platform_device *pdev)
{
	/* resume here */
	return 0;
}

static const struct dev_pm_ops tx14_pm_ops = {
    .suspend = f_hdmi_tx14_pm_suspend,
    .resume  = f_hdmi_tx14_pm_resume,
};
#else
#define f_hdmi_tx14_suspend NULL
#define f_hdmi_tx14_resume NULL

#endif /* CONFIG_PM */

static const struct of_device_id f_hdmi_tx14_fb_dt_ids[] = {
	{ .compatible = "fujitsu,f_hdmi_tx14" },
	{ /* sentinel */ }
};

static struct platform_driver f_hdmi_tx14_driver = {
	.probe = f_hdmi_tx14_probe,
	.remove = f_hdmi_tx14_remove,
	.suspend = f_hdmi_tx14_suspend,
	.resume = f_hdmi_tx14_resume,
	.driver = {
		.name = "f_hdmi_tx14",
		.of_match_table = f_hdmi_tx14_fb_dt_ids,
#ifdef CONFIG_PM
        .pm = &tx14_pm_ops,
#endif/* ifdef CONFIG_PM */

	},
};

MODULE_DEVICE_TABLE(of, f_hdmi_tx14_fb_dt_ids);

static int __init f_hdmi_tx14_init(void)
{
	return platform_driver_register(&f_hdmi_tx14_driver);
}

static void __exit f_hdmi_tx14_exit(void)
{
	platform_driver_unregister(&f_hdmi_tx14_driver);
}

module_init(f_hdmi_tx14_init);
module_exit(f_hdmi_tx14_exit);

MODULE_LICENSE("GPL");
