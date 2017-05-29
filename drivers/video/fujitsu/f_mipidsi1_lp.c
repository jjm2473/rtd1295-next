/*
 * f_mipidsi1_lp MIPI DSI PHY driver
 * Copyright (C) 2013 Linaro, Ltd for Fujitsu Semi
 * Author: Andy Green <andy.green@linaro.org>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/mailbox.h>
#include <linux/scb_mhu_api.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/mutex.h>

#include <video/fdb.h>
#include <video/f_mipidsi1_lp.h>

#ifdef CONFIG_SN65DSI84
#include <linux/of_i2c.h>
#include <linux/i2c.h>
#include <video/sn65dsi84.h>
#endif


#define USE_DTIMING_VAL (~0)
#define IS_DTIMING_VAL(val) \
	(val == (typeof(val))USE_DTIMING_VAL)

struct param_clkmgr_cfg {
	u8 txesc_clock_division;
	u8 to_clock_division;
};

struct param_dpi_cfg {
	u8 dpi_vid;
	u8 dpi_color_coding;
	u8 shutd_act_low;
	u8 colorm_act_low;
	u8 en18_loosely;
};

struct param_vid_mode_cfg {
	u8 vid_mode_type;
	u8 en_lp_vsa;
	u8 en_lp_vbp;
	u8 en_lp_vfp;
	u8 en_lp_vact;
	u8 en_lp_hbp;
	u8 en_lp_hfp;
	u8 en_multi_pkt;
	u8 en_null_pkt;
	u8 lpcmden;
};

struct param_vid_pkt_cfg {
	u16 vid_pkt_size;
	u16 num_chunks;
	u16 null_pkt_size;
};

struct param_tmr_line_cfg {
	u16 hsa_time;
	u16 hbp_time;
	u16 hline_time;
};

struct param_vtiming_cfg {
	u16 vsa_lines;
	u16 vbp_lines;
	u16 vfp_lines;
	u16 v_active_lines;
};

struct param_phy_tmr_cfg {
	u8 phy_lp2hstime;
	u8 phy_hs2lptime;
};

struct param_to_cnt_cfg {
	u16 hstx_to_cnt;
};

struct param_phy_if_cfg {
	u8 num_lanes;
	u8 phy_stop_wait_time;
};

struct param_phy_if_ctrl {
	u8 phy_txrequestclkhs;
};

struct param_lp_cmd_tim {
	u8 invact_lpcmd_time;
	u8 outvact_lpcmd_time;
};

struct param_phy_setup_cl {
	u8 sap_tlpx_c;
	u8 sap_hs0_c;
	u8 sap_trail_c;
	u8 sap_pre_c;
};

struct param_phy_setup_dl {
	u8 sap_tlpx_d;
	u8 sap_hs0_d;
	u8 sap_trail_d;
	u8 sap_pre_d;
};


struct f_mipidsi1_lp {
	struct f_fdb_child fdb_child;
	struct f_fdb_child *bound; /* fdb fb we are bound to */

	struct device *dev;
	void __iomem *base;
	phys_addr_t base_pa;
	int irq;
	u32 source_crtc_bitfield;

	int state;
	int lanes;
	int virtual_channel;

	struct clk *clk[8];
	int count_clocks;

	struct fdb_video_timings last_timings;
	bool seen_timings;

	struct mutex lock;
	void *bridge;

	struct param_clkmgr_cfg clkmgr_cfg;
	struct param_dpi_cfg dpi_cfg;
	struct param_vid_mode_cfg vid_mode_cfg;
	struct param_vid_pkt_cfg vid_pkt_cfg;
	struct param_tmr_line_cfg tmr_line_cfg;
	struct param_vtiming_cfg vtiming_cfg;
	struct param_phy_tmr_cfg phy_tmr_cfg;
	struct param_to_cnt_cfg to_cnt_cfg;
	struct param_phy_if_cfg phy_if_cfg;
	struct param_phy_if_ctrl phy_if_ctrl;
	struct param_lp_cmd_tim lp_cmd_tim;
	struct param_phy_setup_cl phy_setup_cl;
	struct param_phy_setup_dl phy_setup_dl;
};

enum dsi_source {
	DSI_PIXCLK_SOURCE__EXTERNAL = 0x10,
	DSI_PIXCLK_SOURCE__HDMI_148MHZ = 0,
	DSI_PIXCLK_SOURCE__DPHY_166MHZ = 1,
	DSI_PIXCLK_SOURCE__HDMI_148MHZ2 = 3
};

static int bridge_set_timings(struct f_mipidsi1_lp *priv, struct display_timing *dt)
{
	int ret = 0;
#ifdef CONFIG_SN65DSI84
	void *bridge = priv->bridge;

	if (bridge)
		ret = sn65dsi84_i2c_set_timings((struct i2c_client *)bridge, dt);
#endif

	return ret;
}

static int bridge_start(struct f_mipidsi1_lp *priv)
{
	int ret = 0;
#ifdef CONFIG_SN65DSI84
	void *bridge = priv->bridge;

	if (bridge)
		ret = sn65dsi84_i2c_start((struct i2c_client *)bridge, false);
#endif

	return ret;
}

static int bridge_stop(struct f_mipidsi1_lp *priv)
{
	int ret = 0;

#ifdef CONFIG_SN65DSI84
	void *bridge = priv->bridge;

	if (bridge)
		ret = sn65dsi84_i2c_stop((struct i2c_client *)bridge);
#endif

	return ret;
}

static void *f_mipidsi1_lp_get_bridge_dev(struct platform_device *pdev)
{
#ifdef CONFIG_SN65DSI84
	struct device_node *bridge_node;
	struct i2c_client *bridge_dev;

	bridge_node = of_parse_phandle(pdev->dev.of_node, "bridge", 0);
	if (!bridge_node) {
		dev_dbg(&pdev->dev, "MIPI-LVDS Bridge is not found on this target\n");
		return NULL;
	}

	bridge_dev = of_find_i2c_device_by_node(bridge_node);
	if (!bridge_dev) {
		dev_info(&pdev->dev, "failed to find bridge i2c device(maybe not probed yet)\n");
		bridge_dev = ERR_PTR(-EPROBE_DEFER);
	}

	of_node_put(bridge_node);
	return (void *)bridge_dev;
#else
	return NULL;
#endif
}

static void f_mipidsi1_lp_put_bridge_dev(void *data)
{
#ifdef CONFIG_SN65DSI84
	struct i2c_client *bridge_dev = (struct i2c_client *)data;

	if (bridge_dev)
		put_device(&bridge_dev->dev);
#endif
}

static int f_mipidsi_select_clock_source(struct f_mipidsi1_lp *priv,
						enum dsi_source source)
{
	struct cmd_clk_dsi_pixel ccdp;
	struct completion got_rsp;
	int ret;

	ccdp.payload_size = sizeof(ccdp);
	ccdp.mode = source;

	dev_dbg(priv->dev, "%s: set dsi clock source to %d\n",
							__func__, source);

	init_completion(&got_rsp);
	ret = mhu_send_packet(CMD_CLOCK_DSI_PIXEL_REQ,
						&ccdp, sizeof(ccdp), &got_rsp);
	if (ret < 0) {
		pr_err("%s:%d failed!\n", __func__, __LINE__);
		 return ret;
	}
	if (ret)
		wait_for_completion(&got_rsp);

	dev_dbg(priv->dev, "%s: set dsi clock source %d OK\n",
							__func__, source);

	return 0;
}

static struct fdb_dumps dumps[] = {
	{ "Main", 0, 0xc },
	{ "Mode", 0x18, 0x28 },
	{ "Phy", 0x50, 0xc },
	{ "LP_CMD_TIM", 0x70, 0x0},
	{ "PHY_SETUP", 0x400, 0x4},
};

#if 0
#ifdef DEBUG
struct dumps {
	const char *name;
	unsigned int start;
	unsigned int len;
};

static void dsi_dump_regs(struct f_mipidsi1_lp *priv)
{
	int n, i;


	dev_dbg(priv->dev, "============================dsi_dump_regs %lx\n",
			(unsigned long)priv->base);

	for (i = 0; i < ARRAY_SIZE(dumps); i++)
		for (n = 0; n <= dumps[i].len; n += 4)
			dev_dbg(priv->dev, "%s: +0x%04x: 0x%08X\n",
					dumps[i].name, dumps[i].start + n,
				__raw_readl(priv->base + dumps[i].start + n));
}
#else
#define dsi_dump_regs(priv)
#endif
#endif

static void _f_mipidsi_calc_htimings(
	const struct display_timing *dt, u32 *hline_result, u32 *hsa_result, u32 *hbp_result)
{
	/* could not use floating point operation */

	/* pixelclock is KHz */
	u64 tmp = 1000000000ULL / dt->pixelclock.typ;
	u64 period_picosec = tmp / 8; /* per byte */
	u64 mod = tmp % 8;
	u32 htotal;
	u32 hline, hsa, hbp; /* nano sec */

	htotal = (dt->hactive.typ + dt->hfront_porch.typ + dt->hsync_len.typ + dt->hback_porch.typ);

	hline = (u32)(period_picosec * htotal);
	hsa = (u32)(period_picosec * dt->hsync_len.typ);
	hbp = (u32)(period_picosec * dt->hback_porch.typ);

	hline /= 1000;
	hsa /= 1000;
	hbp /= 1000;
	if (mod >= 4) { /* more than half of byte */
		hline++;
		hsa++;
		hbp++;
	}

	*hline_result = hline;
	*hsa_result = hsa;
	*hbp_result = hbp;

	return;
}

/*
 * Example characteristics
 *
 * DPI video resolution:
 * PCLK period = 50 ns
 * HSA = 8 PCLK		horizontal sync active
 * HBP = 8 PCLK		horizontal back porch
 * HACT = 480 PCLK
 * HFP = 24 PCLK	horizontal front porch
 * VSA = 2 Line		vertical sync active
 * VBP = 2 Line		vertical back porch
 * VAdr = 640 Line
 * VFP = 4 Line		vertical front porch
 */

void _f_mipidsi_set_timings(struct f_fdb_child *fdb_child,
			struct fdb_video_timings *timings)
{
	struct f_mipidsi1_lp *priv = dev_get_drvdata(fdb_child->dev);
	struct display_timing *dt = &timings->dt;
	u8 den_act_low, hsync_act_low, vsync_act_low;
	u16 vid_pkt_size;
	u16 hsa_time, hbp_time, hline_time;
	u8 vsa_lines, vbp_lines, vfp_lines;
	u16 v_active_lines;
	u32 hline, hsa, hbp;

	struct param_clkmgr_cfg *clkmgr_cfg = &priv->clkmgr_cfg;
	struct param_dpi_cfg *dpi_cfg = &priv->dpi_cfg;
	struct param_vid_mode_cfg *vid_mode_cfg = &priv->vid_mode_cfg;
	struct param_vid_pkt_cfg *vid_pkt_cfg = &priv->vid_pkt_cfg;
	struct param_tmr_line_cfg *tmr_line_cfg = &priv->tmr_line_cfg;
	struct param_vtiming_cfg *vtiming_cfg = &priv->vtiming_cfg;
	struct param_phy_tmr_cfg *phy_tmr_cfg = &priv->phy_tmr_cfg;
	struct param_to_cnt_cfg *to_cnt_cfg = &priv->to_cnt_cfg;
	struct param_phy_if_cfg *phy_if_cfg = &priv->phy_if_cfg;
	struct param_phy_if_ctrl *phy_if_ctrl = &priv->phy_if_ctrl;
	struct param_lp_cmd_tim *lp_cmd_tim = &priv->lp_cmd_tim;
	struct param_phy_setup_cl *phy_setup_cl = &priv->phy_setup_cl;
	struct param_phy_setup_dl *phy_setup_dl = &priv->phy_setup_dl;


	WARN_ON(bridge_set_timings(priv, dt));

	priv->last_timings = *timings;
	priv->seen_timings = true;

	_f_mipidsi_calc_htimings(dt, &hline, &hsa, &hbp);
	dev_dbg(priv->dev, "calculated hline:%u, hsa:%u, hbp:%u\n", hline, hsa, hbp);

	den_act_low = !!(dt->flags & DISPLAY_FLAGS_DE_LOW);
	hsync_act_low = !!(dt->flags & DISPLAY_FLAGS_HSYNC_LOW);
	vsync_act_low = !!(dt->flags & DISPLAY_FLAGS_VSYNC_LOW);

	vid_pkt_size = IS_DTIMING_VAL(vid_pkt_cfg->vid_pkt_size) ?
					(u16)dt->hactive.typ : vid_pkt_cfg->vid_pkt_size;

	hsa_time = IS_DTIMING_VAL(tmr_line_cfg->hsa_time) ? (u16)hsa : tmr_line_cfg->hsa_time;
	hbp_time = IS_DTIMING_VAL(tmr_line_cfg->hbp_time) ? (u16)hbp : tmr_line_cfg->hbp_time;
	hline_time = IS_DTIMING_VAL(tmr_line_cfg->hline_time) ? (u16)hline : tmr_line_cfg->hline_time;

	vsa_lines = IS_DTIMING_VAL(vtiming_cfg->vsa_lines) ?
				(u8)dt->vsync_len.typ : (u8)vtiming_cfg->vsa_lines;

	vbp_lines = IS_DTIMING_VAL(vtiming_cfg->vbp_lines) ?
				(u8)dt->vback_porch.typ : (u8)vtiming_cfg->vbp_lines;

	vfp_lines = IS_DTIMING_VAL(vtiming_cfg->vfp_lines) ?
				(u8)dt->vfront_porch.typ : (u8)vtiming_cfg->vfp_lines;

	v_active_lines = IS_DTIMING_VAL(vtiming_cfg->v_active_lines) ?
				(u16)dt->vactive.typ : vtiming_cfg->v_active_lines;

	dev_dbg(priv->dev, "dataen_act_low:%u, hsync_act_low:%u, vsync_act_low:%u\n",
		den_act_low, hsync_act_low, vsync_act_low);

	dev_dbg(priv->dev, "vid_pkt_size:%u, hsa_time:%u, hbp_time:%u, hline_time:%u\n",
				vid_pkt_size, hsa_time, hbp_time, hline_time);

	dev_dbg(priv->dev, "vsa_lines:%u, vbp_lines:%u, vfp_lines:%u, v_active_lines:%u\n",
					vsa_lines, vbp_lines, vfp_lines, v_active_lines);

	__raw_writel(0, priv->base + FMDSI_PHY_RSTZ_OFS);
	__raw_writel(0, priv->base + FMDSI_PWR_UP_OFS);

	__raw_writel(
		fdb_const(FMDSI_PHY_SETUP_CL, SAP_PRE_C, phy_setup_cl->sap_pre_c) |
		fdb_const(FMDSI_PHY_SETUP_CL, SAP_TRAIL_C, phy_setup_cl->sap_trail_c) |
		fdb_const(FMDSI_PHY_SETUP_CL, SAP_HS0_C, phy_setup_cl->sap_hs0_c) |
		fdb_const(FMDSI_PHY_SETUP_CL, SAP_TLPX_C, phy_setup_cl->sap_tlpx_c),
					priv->base + FMDSI_PHY_SETUP_CL_OFS);

	__raw_writel(
		fdb_const(FMDSI_PHY_SETUP_DL, SAP_PRE_D, phy_setup_dl->sap_pre_d) |
		fdb_const(FMDSI_PHY_SETUP_DL, SAP_TRAIL_D, phy_setup_dl->sap_trail_d) |
		fdb_const(FMDSI_PHY_SETUP_DL, SAP_HS0_D, phy_setup_dl->sap_hs0_d) |
		fdb_const(FMDSI_PHY_SETUP_DL, SAP_TLPX_D, phy_setup_dl->sap_tlpx_d),
					priv->base + FMDSI_PHY_SETUP_DL_OFS);

	__raw_writel(
		fdb_const(FMDSI_PHY_TMR_CFG, PHY_HS2LPTIME, phy_tmr_cfg->phy_hs2lptime) |
		fdb_const(FMDSI_PHY_TMR_CFG, PHY_LP2HSTIME, phy_tmr_cfg->phy_lp2hstime),
					priv->base + FMDSI_PHY_TMR_CFG_OFS);

	__raw_writel(
		fdb_const(FMDSI_PHY_IF_CFG, PHY_STOP_WAIT_TIME, phy_if_cfg->phy_stop_wait_time) |
		fdb_const(FMDSI_PHY_IF_CFG, NUM_LANES, phy_if_cfg->num_lanes),
					priv->base + FMDSI_PHY_IF_CFG_OFS);

	__raw_writel(
		fdb_const(FMDSI_PHY_IF_CTRL, PHY_TX_REQUEST_PPI_CLK_HS, phy_if_ctrl->phy_txrequestclkhs),
			priv->base + FMDSI_PHY_IF_CTRL_OFS);

	__raw_writel(
		fdb_const(FMDSI_CLKMGR_CFG, TO_CLOCK_DIV, clkmgr_cfg->to_clock_division) |
		fdb_const(FMDSI_CLKMGR_CFG, TXESC_CLOCK_DIV, clkmgr_cfg->txesc_clock_division),
					priv->base + FMDSI_CLKMGR_CFG_OFS);
	__raw_writel(
		fdb_const(FMDSI_TO_CNT_CFG, HSTX_TO_CNT, to_cnt_cfg->hstx_to_cnt),
					priv->base + FMDSI_TO_CNT_CFG_OFS);

	__raw_writel(
		fdb_const(FMDSI_DPI_CFG, DPI_VID, dpi_cfg->dpi_vid) |
		fdb_const(FMDSI_DPI_CFG, DPI_COLOR_CODING, dpi_cfg->dpi_color_coding) |
		fdb_const(FMDSI_DPI_CFG, DATAEN_ACT_LOW, den_act_low) |
		fdb_const(FMDSI_DPI_CFG, VSYNC_ACT_LOW, vsync_act_low) |
		fdb_const(FMDSI_DPI_CFG, HSYNC_ACT_LOW, hsync_act_low) |
		fdb_const(FMDSI_DPI_CFG, SHUTD_ACT_LOW, dpi_cfg->shutd_act_low) |
		fdb_const(FMDSI_DPI_CFG, COLORM_ACT_LOW, dpi_cfg->colorm_act_low) |
		fdb_const(FMDSI_DPI_CFG, EN18LOOSELY, dpi_cfg->en18_loosely),
					priv->base + FMDSI_DPI_CFG_OFS);

	__raw_writel(
		fdb_const(FMDSI_VIDMODE_CFG, EN_LP_HFP, vid_mode_cfg->en_lp_hfp) |
		fdb_const(FMDSI_VIDMODE_CFG, EN_LP_HBP, vid_mode_cfg->en_lp_hbp) |
		fdb_const(FMDSI_VIDMODE_CFG, EN_LP_VACT, vid_mode_cfg->en_lp_vact) |
		fdb_const(FMDSI_VIDMODE_CFG, EN_LP_VFP, vid_mode_cfg->en_lp_vfp) |
		fdb_const(FMDSI_VIDMODE_CFG, EN_LP_VBP, vid_mode_cfg->en_lp_vbp) |
		fdb_const(FMDSI_VIDMODE_CFG, EN_LP_VSA, vid_mode_cfg->en_lp_vsa) |
		fdb_const(FMDSI_VIDMODE_CFG, VID_MODE_TYPE, vid_mode_cfg->vid_mode_type) |
		fdb_const(FMDSI_VIDMODE_CFG, EN_MULTI_PKT, vid_mode_cfg->en_multi_pkt) |
		fdb_const(FMDSI_VIDMODE_CFG, EN_NULL_PKT, vid_mode_cfg->en_null_pkt) |
		fdb_const(FMDSI_VIDMODE_CFG, LPCMDEN, vid_mode_cfg->lpcmden) |
		fdb_const(FMDSI_VIDMODE_CFG, EN_VIDEO_MODE, 1), /* enable video mode */
					priv->base + FMDSI_VIDMODE_CFG_OFS);

	__raw_writel(
		fdb_const(FMDSI_VID_PKT_CFG, NULL_PKT_SIZE, vid_pkt_cfg->null_pkt_size) |
		fdb_const(FMDSI_VID_PKT_CFG, NUM_CHUNKS, vid_pkt_cfg->num_chunks) |
		fdb_const(FMDSI_VID_PKT_CFG, VID_PKT_SIZE, vid_pkt_size),
					priv->base + FMDSI_VID_PKT_CFG_OFS);

	__raw_writel(
		fdb_const(FMDSI_TMR_LINE_CFG, HLINE_TIME, hline_time) |
		fdb_const(FMDSI_TMR_LINE_CFG, HBP_TIME, hbp_time) |
		fdb_const(FMDSI_TMR_LINE_CFG, HSA_TIME, hsa_time),
					priv->base + FMDSI_TMR_LINE_CFG_OFS);

	__raw_writel(
		fdb_const(FMDSI_VTIMING_CFG, V_ACTIVE_LINES, v_active_lines) |
		fdb_const(FMDSI_VTIMING_CFG, VFP_LINES, vfp_lines) |
		fdb_const(FMDSI_VTIMING_CFG, VBP_LINES, vbp_lines) |
		fdb_const(FMDSI_VTIMING_CFG, VSA_LINES, vsa_lines),
					priv->base + FMDSI_VTIMING_CFG_OFS);

	__raw_writel(
		fdb_const(FMDSI_LP_CMD_TIM, OUTVACT_LPCMD_TIME, lp_cmd_tim->outvact_lpcmd_time) |
		fdb_const(FMDSI_LP_CMD_TIM, INVACT_LPCMD_TIME, lp_cmd_tim->invact_lpcmd_time),
					priv->base + FMDSI_LP_CMD_TIM_OFS);

	__raw_writel(
		fdb_const(FMDSI_CMD_MODE_CFG, EN_CMD_MODE, 0), /* disable command mode */
					priv->base + FMDSI_CMD_MODE_CFG_OFS);

	__raw_writel(1, priv->base + FMDSI_PWR_UP_OFS);
	__raw_writel(1, priv->base + FMDSI_PHY_RSTZ_OFS);

	dev_dbg(priv->dev, "f_mipidsi_set_timings: completed\n");
}

void f_mipidsi_set_timings(struct f_fdb_child *fdb_child,
			struct fdb_video_timings *timings)
{
	struct f_mipidsi1_lp *priv = dev_get_drvdata(fdb_child->dev);

	pm_runtime_get_sync(priv->dev);

	_f_mipidsi_set_timings(fdb_child, timings);

	pm_runtime_put_sync_suspend(priv->dev);
}

static irqreturn_t f_mipidsi_interrupt(int irq, void *dev)
{
	struct f_mipidsi1_lp *priv = dev;
	/* reading ISR auto-clears it */
	u32 status = __raw_readl(priv->base + FMDSI_ERROR_ST1_OFS);

	if (fdb_getval(status, FMDSI_ERROR_ST1, PAYLOAD_SEND))
		dev_err(priv->dev, "IRQ: PAYLOAD_SEND\n");

	if (fdb_getval(status, FMDSI_ERROR_ST1, PAYLOAD_WRITE))
		dev_err(priv->dev, "IRQ: PAYLOAD_WRITE\n");

	if (fdb_getval(status, FMDSI_ERROR_ST1, CMD_WRITE))
		dev_err(priv->dev, "IRQ: CMD_WRITE\n");

	if (fdb_getval(status, FMDSI_ERROR_ST1, DPI_OVERFLOW))
		dev_err(priv->dev, "IRQ: DPI_OVERFLOW\n");

	if (fdb_getval(status, FMDSI_ERROR_ST1, TIMEOUT_HS_TX))
		dev_err(priv->dev, "IRQ: TIMEOUT_HS_TX\n");

	return IRQ_HANDLED;
}

static int f_mipidsi_get_connector_type(struct f_fdb_child *fdb_child)
{
	return DRM_MODE_CONNECTOR_LVDS;
}

static int f_mipidsi_check_timings(struct f_fdb_child *fdb_child,
					struct fdb_video_timings *timings)
{
	struct f_mipidsi1_lp *priv = dev_get_drvdata(fdb_child->dev);

	dev_dbg(priv->dev, "f_mipidsi_check_timings\n");

	fdb_dump_video_timings(timings);

	switch (timings->bits_per_pixel) {
	case 15:
	case 16:
	case 32:
	case 24:
		break;
	default:
		/* for synthetic modes like 1024x768, no bpp info */
		timings->bits_per_pixel = 32;
	}

	if (!priv->bound)
		return 0;

	if (!priv->bound->ops->get_timings)
		return 0;

	/* accept them if crtc likes them */

	return priv->bound->ops->check_timings(priv->bound, timings);
}

static void f_mipidsi_get_timings(struct f_fdb_child *fdb_child,
					struct fdb_video_timings *timings)
{
	struct f_mipidsi1_lp *priv = dev_get_drvdata(fdb_child->dev);

	dev_dbg(priv->dev, "f_mipidsi_get_timings: source %p\n", priv->bound);

	if (!priv->bound)
		return;

	if (!priv->bound->ops->get_timings) {
		dev_err(priv->dev, "no get_timings in connector bind\n");
		return;
	}

	priv->bound->ops->get_timings(priv->bound, timings);
}


static bool f_mipidsi_detect(struct f_fdb_child *fdb_child)
{
	return true;
}

static u32 f_mipidsi_get_source_crtc_bitfield(struct f_fdb_child *fdb_child)
{
	struct f_mipidsi1_lp *priv = dev_get_drvdata(fdb_child->dev);

	return priv->source_crtc_bitfield;
}

static int f_mipidsi_bind_to_source(struct f_fdb_child *fdb_child,
						struct f_fdb_child *fdb_bound)
{
	struct f_mipidsi1_lp *priv = dev_get_drvdata(fdb_child->dev);

	priv->bound = fdb_bound;
	fdb_bound->reverse_binding = fdb_child;
	dev_info(fdb_child->dev, "binding to source %s\n",
					dev_name(fdb_bound->dev));
	return 0;
}

#if defined(CONFIG_DEBUG_FS)
static int f_mipidsi_fdb_debugfs(struct f_fdb_child *fdb_child,
						struct seq_file *m, int type)
{
	struct f_mipidsi1_lp *priv = dev_get_drvdata(fdb_child->dev);
	int i, n;

	switch (type) {
	case FDB_DEBUGFS_INFO:
		break;
	case FDB_DEBUGFS_DUMP:
		pm_runtime_get_sync(priv->dev);
		seq_printf(m, "struct dump dump_mipidsi_0x%lx[] = {\n",
						(unsigned long)priv->base_pa);
		for (i = 0; i < ARRAY_SIZE(dumps); i++)
			for (n = 0; n <= dumps[i].len; n += 4)
				seq_printf(m,
					"/* %s: +0x%04x */ { 0x%lx, 0x%08X },\n",
					dumps[i].name,
					dumps[i].start + n,
					(unsigned long)priv->base_pa +
							dumps[i].start + n,
					__raw_readl(priv->base +
							dumps[i].start + n));
		seq_printf(m, "};\n");
		pm_runtime_put_sync_suspend(priv->dev);
		break;
	}
	return 0;
}
#endif

static void
f_mipidsi_enable_core(struct f_mipidsi1_lp *priv)
{
	pm_runtime_get_sync(priv->dev);

	__raw_writel(
		fdb_const(FMDSI_GEN_HDR, GEN_WC_MSBYTE, 0) |
		fdb_const(FMDSI_GEN_HDR, GEN_WC_LSBYTE, 0) |
		fdb_const(FMDSI_GEN_HDR, GEN_VC, 0) |
		fdb_const(FMDSI_GEN_HDR, GEN_DT, 0x32),
					priv->base + FMDSI_GEN_HDR_OFS);

	__raw_writel(
		fdb_const(FMDSI_GEN_HDR, GEN_WC_MSBYTE, 0x10) |
		fdb_const(FMDSI_GEN_HDR, GEN_WC_LSBYTE, 0x10) |
		fdb_const(FMDSI_GEN_HDR, GEN_VC, 0) |
		fdb_const(FMDSI_GEN_HDR, GEN_DT, 0x5),
					priv->base + FMDSI_GEN_HDR_OFS);
}

static void
f_mipidsi_disable_core(struct f_mipidsi1_lp *priv)
{
	__raw_writel(
		fdb_const(FMDSI_GEN_HDR, GEN_WC_MSBYTE, 0) |
		fdb_const(FMDSI_GEN_HDR, GEN_WC_LSBYTE, 0) |
		fdb_const(FMDSI_GEN_HDR, GEN_VC, 0) |
		fdb_const(FMDSI_GEN_HDR, GEN_DT, 0x22),
					priv->base + FMDSI_GEN_HDR_OFS);

	__raw_writel(
		fdb_const(FMDSI_GEN_HDR, GEN_WC_MSBYTE, 0x11) |
		fdb_const(FMDSI_GEN_HDR, GEN_WC_LSBYTE, 0x11) |
		fdb_const(FMDSI_GEN_HDR, GEN_VC, 0) |
		fdb_const(FMDSI_GEN_HDR, GEN_DT, 0x5),
					priv->base + FMDSI_GEN_HDR_OFS);

	pm_runtime_put_sync_suspend(priv->dev);
}

static int
f_mipidsi_enable(struct f_fdb_child *fdb_child)
{
	struct f_mipidsi1_lp *priv = dev_get_drvdata(fdb_child->dev);
	int ret = 0;

	dev_info(priv->dev, "%s\n", __func__);

	mutex_lock(&priv->lock);

	if (priv->state)
		goto end;

	f_mipidsi_enable_core(priv);

	ret = bridge_start(priv);
	if (ret)
		goto bail;

	priv->state++;

end:
	mutex_unlock(&priv->lock);
	return ret;

bail:
	f_mipidsi_disable_core(priv);
	mutex_unlock(&priv->lock);
	return ret;
}

static void
f_mipidsi_disable(struct f_fdb_child *fdb_child)
{
	int ret;
	struct f_mipidsi1_lp *priv = dev_get_drvdata(fdb_child->dev);

	dev_info(priv->dev, "%s\n", __func__);

	mutex_lock(&priv->lock);

	if (!priv->state)
		goto end;

	ret = bridge_stop(priv);
	if (ret)
		dev_warn(priv->dev, "stop bridge device failed.\n");

	f_mipidsi_disable_core(priv);
	priv->state--;

end:
	mutex_unlock(&priv->lock);
	return;
}


struct f_fdb_ops f_mipidsi_fdb_ops = {
	.get_connector_type = f_mipidsi_get_connector_type,
	.check_timings = f_mipidsi_check_timings,
	.get_timings = f_mipidsi_get_timings,
	.set_timings = f_mipidsi_set_timings,
	.detect = f_mipidsi_detect,
	.get_source_crtc_bitfield = f_mipidsi_get_source_crtc_bitfield,
	.bind_to_source = f_mipidsi_bind_to_source,
	.enable = f_mipidsi_enable,
	.disable = f_mipidsi_disable,
#if defined(CONFIG_DEBUG_FS)
	.fdb_debugfs = f_mipidsi_fdb_debugfs,
#endif
};


static void f_mipidsi1_lp_get_register_settings(struct device *dev, struct f_mipidsi1_lp *priv)
{
	int ret;
	u8 val8;
	u16 val16;
	struct param_clkmgr_cfg *clkmgr_cfg = &priv->clkmgr_cfg;
	struct param_dpi_cfg *dpi_cfg = &priv->dpi_cfg;
	struct param_vid_mode_cfg *vid_mode_cfg = &priv->vid_mode_cfg;
	struct param_vid_pkt_cfg *vid_pkt_cfg = &priv->vid_pkt_cfg;
	struct param_tmr_line_cfg *tmr_line_cfg = &priv->tmr_line_cfg;
	struct param_vtiming_cfg *vtiming_cfg = &priv->vtiming_cfg;
	struct param_phy_tmr_cfg *phy_tmr_cfg = &priv->phy_tmr_cfg;
	struct param_to_cnt_cfg *to_cnt_cfg = &priv->to_cnt_cfg;
	struct param_phy_if_cfg *phy_if_cfg = &priv->phy_if_cfg;
	struct param_phy_if_ctrl *phy_if_ctrl = &priv->phy_if_ctrl;
	struct param_lp_cmd_tim *lp_cmd_tim = &priv->lp_cmd_tim;
	struct param_phy_setup_cl *phy_setup_cl = &priv->phy_setup_cl;
	struct param_phy_setup_dl *phy_setup_dl = &priv->phy_setup_dl;

	/* CLKMGR_CFG */
	ret = of_property_read_u8(dev->of_node, "txesc_clock_division", &val8);
	if (ret)
		val8 = 4; /* set default value */

	clkmgr_cfg->txesc_clock_division = val8;

	ret = of_property_read_u8(dev->of_node, "to_clock_division", &val8);
	if (ret)
		val8 = 1;

	clkmgr_cfg->to_clock_division = val8;

	/* DPI_CFG */
	ret = of_property_read_u8(dev->of_node, "virtual_channel", &val8);
	if (ret)
		val8 = 0;

	dpi_cfg->dpi_vid = val8 & 0x3;
	priv->virtual_channel = (int)dpi_cfg->dpi_vid;

	ret = of_property_read_u8(dev->of_node, "dpi_color_coding", &val8);
	if (ret)
		val8 = DPICC_24BPP;

	dpi_cfg->dpi_color_coding = val8 & 0x7;

	ret = of_property_read_u8(dev->of_node, "shutd_act_low", &val8);
	if (ret)
		val8 = 0;

	dpi_cfg->shutd_act_low = !!val8;

	ret = of_property_read_u8(dev->of_node, "colorm_act_low", &val8);
	if (ret)
		val8 = 0;

	dpi_cfg->colorm_act_low = !!val8;

	ret = of_property_read_u8(dev->of_node, "en18_loosely", &val8);
	if (ret)
		val8 = 0;

	dpi_cfg->en18_loosely = !!val8;

	/* VID_MODE_CFG */
	ret = of_property_read_u8(dev->of_node, "vid_mode_type", &val8);
	if (ret)
		val8 = 3; /* default: burst mode */

	vid_mode_cfg->vid_mode_type = val8 & 0x3;

	ret = of_property_read_u8(dev->of_node, "en_lp_vsa", &val8);
	if (ret)
		val8 = 1;

	vid_mode_cfg->en_lp_vsa = !!val8;

	ret = of_property_read_u8(dev->of_node, "en_lp_vbp", &val8);
	if (ret)
		val8 = 1;

	vid_mode_cfg->en_lp_vbp = !!val8;

	ret = of_property_read_u8(dev->of_node, "en_lp_vfp", &val8);
	if (ret)
		val8 = 1;

	vid_mode_cfg->en_lp_vfp = !!val8;

	ret = of_property_read_u8(dev->of_node, "en_lp_vact", &val8);
	if (ret)
		val8 = 1;

	vid_mode_cfg->en_lp_vact = !!val8;

	ret = of_property_read_u8(dev->of_node, "en_lp_hbp", &val8);
	if (ret)
		val8 = 1;

	vid_mode_cfg->en_lp_hbp = !!val8;

	ret = of_property_read_u8(dev->of_node, "en_lp_hfp", &val8);
	if (ret)
		val8 = 1;

	vid_mode_cfg->en_lp_hfp = !!val8;

	ret = of_property_read_u8(dev->of_node, "en_multi_pkt", &val8);
	if (ret)
		val8 = 0;

	vid_mode_cfg->en_multi_pkt = !!val8;

	ret = of_property_read_u8(dev->of_node, "en_null_pkt", &val8);
	if (ret)
		val8 = 0;

	vid_mode_cfg->en_null_pkt = !!val8;

	ret = of_property_read_u8(dev->of_node, "lpcmden", &val8);
	if (ret)
		val8 = 0;

	vid_mode_cfg->lpcmden = !!val8;

	/* VID_PKT_CFG */
	ret = of_property_read_u16(dev->of_node, "vid_pkt_size", &val16);
	if (ret) {
		val16 = USE_DTIMING_VAL; /* use display_timing.hactive.typ */
	}
	else {
		val16 &= 0x3ff;
	}

	vid_pkt_cfg->vid_pkt_size = val16;

	ret = of_property_read_u16(dev->of_node, "num_chunks", &val16);
	if (ret)
		val16 = 0;

	vid_pkt_cfg->num_chunks = val16 & 0x3ff;

	ret = of_property_read_u16(dev->of_node, "null_pkt_size", &val16);
	if (ret)
		val16 = 0;

	vid_pkt_cfg->null_pkt_size = val16 & 0x3ff;

	/* TMR_LINE_CFG */
	ret = of_property_read_u16(dev->of_node, "hsa_time", &val16);
	if (ret) {
		val16 = USE_DTIMING_VAL; /* calculated with display_timing */
	}
	else {
		val16 &= 0x1ff;
	}

	tmr_line_cfg->hsa_time = val16;

	ret = of_property_read_u16(dev->of_node, "hbp_time", &val16);
	if (ret) {
		val16 = USE_DTIMING_VAL;
	}
	else {
		val16 &= 0x1ff;
	}

	tmr_line_cfg->hbp_time = val16;

	ret = of_property_read_u16(dev->of_node, "hline_time", &val16);
	if (ret) {
		val16 = USE_DTIMING_VAL;
	}
	else {
		val16 &= 0x3fff;
	}

	tmr_line_cfg->hline_time = val16;

	/* VTIMING_CFG */
	ret = of_property_read_u8(dev->of_node, "vsa_lines", &val8);
	if (ret) {
		val16 = USE_DTIMING_VAL;
	}
	else {
		val16 = val8 & 0xf;
	}

	vtiming_cfg->vsa_lines = val16;

	ret = of_property_read_u8(dev->of_node, "vbp_lines", &val8);
	if (ret) {
		val16 = USE_DTIMING_VAL;
	}
	else {
		val16 = val8 & 0x3f;
	}

	vtiming_cfg->vbp_lines = val16;

	ret = of_property_read_u8(dev->of_node, "vfp_lines", &val8);
	if (ret) {
		val16 = USE_DTIMING_VAL;
	}
	else {
		val16 = val8 & 0x3f;
	}

	vtiming_cfg->vfp_lines = val16;

	ret = of_property_read_u16(dev->of_node, "v_active_lines", &val16);
	if (ret) {
		val16 = USE_DTIMING_VAL;
	}
	else {
		val16 &= 0x7ff;
	}

	vtiming_cfg->v_active_lines = val16;

	/* PHY_TMR_CFG */
	ret = of_property_read_u8(dev->of_node, "phy_lp2hstime", &val8);
	if (ret)
		val8 = 34;

	phy_tmr_cfg->phy_lp2hstime = val8;

	ret = of_property_read_u8(dev->of_node, "phy_hs2lptime", &val8);
	if (ret)
		val8 = 24;

	phy_tmr_cfg->phy_hs2lptime = val8;

	/* TO_CNT_CFG  */
	ret = of_property_read_u16(dev->of_node, "hstx_to_cnt", &val16);
	if (ret)
		val16 = 0;

	to_cnt_cfg->hstx_to_cnt = val16;

	/* PHY_IF_CFG */
	ret = of_property_read_u8(dev->of_node, "dsi_lanes", &val8);
	if (ret || val8 > 4)
		val8 = 2;

	/*
	 * user must set "real" number of lanes.
	 * on the other hand, num_lanes is value for register settings.
	 */
	phy_if_cfg->num_lanes = val8 - 1;
	priv->lanes = (int)val8;

	ret = of_property_read_u8(dev->of_node, "phy_stop_wait_time", &val8);
	if (ret)
		val8 = 54;

	phy_if_cfg->phy_stop_wait_time = val8;

	/* PHY_IF_CTRL */
	ret = of_property_read_u8(dev->of_node, "phy_txrequestclkhs", &val8);
	if (ret)
		val8 = 1;

	/* we support only phy_txrequestclkhs */
	phy_if_ctrl->phy_txrequestclkhs = !!val8;

	/* LP_CMD_TIM */
	ret = of_property_read_u8(dev->of_node, "invact_lpcmd_time", &val8);
	if (ret)
		val8 = 0;

	lp_cmd_tim->invact_lpcmd_time = val8;

	ret = of_property_read_u8(dev->of_node, "outvact_lpcmd_time", &val8);
	if (ret)
		val8 = 10;

	lp_cmd_tim->outvact_lpcmd_time = val8;

	/* PHY_SETUP_CL */
	ret = of_property_read_u8(dev->of_node, "sap_tlpx_c", &val8);
	if (ret)
		val8 = 7;

	phy_setup_cl->sap_tlpx_c = val8 & 0xf;

	ret = of_property_read_u8(dev->of_node, "sap_hs0_c", &val8);
	if (ret)
		val8 = 35;

	phy_setup_cl->sap_hs0_c = val8 & 0x3f;

	ret = of_property_read_u8(dev->of_node, "sap_trail_c", &val8);
	if (ret)
		val8 = 10;

	phy_setup_cl->sap_trail_c = val8 & 0xf;

	ret = of_property_read_u8(dev->of_node, "sap_pre_c", &val8);
	if (ret)
		val8 = 0;

	phy_setup_cl->sap_pre_c = val8 & 0xf;

	/* PHY_SETUP_DL */
	ret = of_property_read_u8(dev->of_node, "sap_tlpx_d", &val8);
	if (ret)
		val8 = 7;

	phy_setup_dl->sap_tlpx_d = val8 & 0xf;

	ret = of_property_read_u8(dev->of_node, "sap_hs0_d", &val8);
	if (ret)
		val8 = 17;

	phy_setup_dl->sap_hs0_d = val8 & 0x1f;

	ret = of_property_read_u8(dev->of_node, "sap_trail_d", &val8);
	if (ret)
		val8 = 9;

	phy_setup_dl->sap_trail_d = val8 & 0xf;

	ret = of_property_read_u8(dev->of_node, "sap_pre_d", &val8);
	if (ret)
		val8 = 0;

	phy_setup_dl->sap_pre_d = val8 & 0xf;

	return;
}

static int
f_mipidsi1_lp_probe(struct platform_device *pdev)
{
	struct f_mipidsi1_lp *priv;
	struct resource *res;
	int ret = 0;
	void *bridge_dev;

	bridge_dev = f_mipidsi1_lp_get_bridge_dev(pdev);
	if (IS_ERR(bridge_dev))
		return PTR_ERR(bridge_dev);

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		f_mipidsi1_lp_put_bridge_dev(bridge_dev);
		return -ENOMEM;
	}

	mutex_init(&priv->lock);
	priv->bridge = bridge_dev;

	f_mipidsi1_lp_get_register_settings(&pdev->dev, priv);

	platform_set_drvdata(pdev, priv);
	priv->dev = &pdev->dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Missing resource\n");
		ret = -EINVAL;
		goto bail1;
	}

	priv->base = ioremap(res->start, res->end - res->start);
	if (!priv->base) {
		ret = -EINVAL;
		goto bail1;
	}
	priv->base_pa = res->start;

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq > 0) {
		ret = request_irq(priv->irq, f_mipidsi_interrupt,
			IRQF_TRIGGER_RISING, pdev->dev.driver->name, priv);
		if (ret) {
			dev_err(&pdev->dev, "failed to allocate irq.\n");
			goto bail2;
		}
		disable_irq(priv->irq);
	}

	if (of_property_read_u32(pdev->dev.of_node,
				"sources", &priv->source_crtc_bitfield)) {
		dev_err(&pdev->dev, "Missing sources bitfield\n");
		ret = -EINVAL;
		goto bail4;
	}

	f_mipidsi_select_clock_source(priv, DSI_PIXCLK_SOURCE__EXTERNAL);

	priv->count_clocks = 0;
	while (priv->count_clocks < ARRAY_SIZE(priv->clk)) {
		priv->clk[priv->count_clocks] =
			of_clk_get(pdev->dev.of_node, priv->count_clocks);
		if (IS_ERR(priv->clk[priv->count_clocks]))
			break;
		priv->count_clocks++;
	}

	if (priv->count_clocks < 1) {
		dev_err(&pdev->dev, "At least PLL clock needed in DT\n");
		goto bail4a;
	}

	ret = clk_set_rate(priv->clk[0], 1000000000); /* 1 GHz */
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to set mipi-dsi dphy clock rate\n");
		goto bail4a;
	}

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
	
	/* register our fdb_child with f_fdb bus */
	ret = fdb_register(&pdev->dev, &priv->fdb_child, &f_mipidsi_fdb_ops);
	if (ret < 0) {
		dev_err(&pdev->dev, "framebuffer registration failed\n");
		goto bail5;
	}

	dev_info(&pdev->dev, "MIPI DSI PHY driver\n");

	return 0;

bail5:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	
bail4a:
	while (priv->count_clocks--)
		clk_put(priv->clk[priv->count_clocks]);

bail4:
	if (priv->irq > 0)
		free_irq(priv->irq, priv);
bail2:
	iounmap(priv->base);
bail1:
	kfree(priv);

	return ret;
}

static int f_mipidsi1_lp_remove(struct platform_device *pdev)
{
	struct f_mipidsi1_lp *priv = platform_get_drvdata(pdev);

	f_mipidsi1_lp_put_bridge_dev(priv->bridge);

	fdb_unregister(&pdev->dev, &priv->fdb_child);

	if (priv->irq > 0)
		free_irq(priv->irq, priv);

	while (priv->count_clocks--) {
		clk_disable_unprepare(priv->clk[priv->count_clocks]);
		clk_put(priv->clk[priv->count_clocks]);
	}

	iounmap(priv->base);
	kfree(priv);

	return 0;
}


#ifdef CONFIG_PM
#ifdef CONFIG_PM_RUNTIME
static int f_mipidsi1_lp_runtime_suspend(struct device *dev)
{
	struct f_mipidsi1_lp *priv = dev_get_drvdata(dev);
	int n;

	dev_dbg(dev, "%s\n", __func__);

	disable_irq(priv->irq);

	/* power the IP unit down */
	fdb_setl(priv->base, FMDSI_PWR_UP, SHUTDOWNZ, 0);

	for (n = priv->count_clocks - 1; n >= 0; n--)
		clk_disable_unprepare(priv->clk[n]);

	if (priv->state)
		WARN_ON(bridge_stop(priv));

	return 0;
}

static int f_mipidsi1_lp_runtime_resume(struct device *dev)
{
	struct f_mipidsi1_lp *priv = dev_get_drvdata(dev);
	int n;

	dev_dbg(dev, "%s\n", __func__);

	/* first let the clocks back on */

	for (n = 0; n < priv->count_clocks; n++)
		clk_prepare_enable(priv->clk[n]);

	/* power the IP unit up */

	fdb_setl(priv->base, FMDSI_PWR_UP, SHUTDOWNZ, 1);

	/* let the interrupts back on */

	enable_irq(priv->irq);

	/*
	 * if we have been given timings previously, reassert them
	 */
	if (priv->seen_timings)
		_f_mipidsi_set_timings(&priv->fdb_child, &priv->last_timings);

	if (priv->state)
		WARN_ON(bridge_start(priv));

	return 0;
}
#endif

static int f_mipidsi1_lp_pm_suspend(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	return f_mipidsi1_lp_runtime_suspend(dev);
}

static int f_mipidsi1_lp_pm_resume(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	return f_mipidsi1_lp_runtime_resume(dev);
}
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops f_mipidsi1_lp_pm_ops = {
       SET_SYSTEM_SLEEP_PM_OPS(f_mipidsi1_lp_pm_suspend, f_mipidsi1_lp_pm_resume)
       SET_RUNTIME_PM_OPS(f_mipidsi1_lp_runtime_suspend, f_mipidsi1_lp_runtime_resume, NULL)
};
#endif


static const struct of_device_id f_mipidsi1_lp_fb_dt_ids[] = {
	{ .compatible = "fujitsu,f_mipidsi1_lp" },
	{ /* sentinel */ }
};

static struct platform_driver f_mipidsi1_lp_driver = {
	.probe = f_mipidsi1_lp_probe,
	.remove = f_mipidsi1_lp_remove,
	.driver = {
		.name = "f_mipidsi1_lp",
		.of_match_table = f_mipidsi1_lp_fb_dt_ids,
#ifdef CONFIG_PM
		.pm = &f_mipidsi1_lp_pm_ops,
#endif
	},
};

MODULE_DEVICE_TABLE(of, f_mipidsi1_lp_fb_dt_ids);

static int __init f_mipidsi1_lp_init(void)
{
	return platform_driver_register(&f_mipidsi1_lp_driver);
}

static void __exit f_mipidsi1_lp_exit(void)
{
	platform_driver_unregister(&f_mipidsi1_lp_driver);
}

module_init(f_mipidsi1_lp_init);
module_exit(f_mipidsi1_lp_exit);

MODULE_LICENSE("GPL");
