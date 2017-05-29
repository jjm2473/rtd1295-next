/*
 * ALSA SoC DAI driver for HDMI audio on MB86S70 processors.
 * Copyright (C) 2013 FUJITSU SEMICONDUCTOR LIMITED
 * Copyright (C) 2015-2016 Socionext Inc.
 * Authors:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/asound.h>
#include <sound/asoundef.h>
#include <sound/dmaengine_pcm.h>
#include <video/fdb.h>
#include <linux/clk.h>
#include "mb8ac0300_pcm.h"
#include "f_saif.h"

#define DRV_NAME "hdmi-audio-dai"
#define F_HDMI_SDIN0		0
#define F_HDMI_SDIN1		1
#define F_HDMI_SDIN2		2
#define F_HDMI_SDIN3		3

struct hdmi_priv {
	struct snd_dmaengine_dai_dma_data dma_data;
	unsigned int dma_req;
	struct f_fdb_audio fdb_audio;
	struct snd_aes_iec958 iec;
	struct snd_cea_861_aud_if cea;
	struct f_fdb_child	*fdbdev;
};
#if 0
static void fujitsu_fdb_get_device(struct f_fdb_child *fdbdev)
{
	get_device(fdbdev->dev);
}

static void fujitsu_fdb_put_device(struct f_fdb_child *fdbdev)
{
	put_device(fdbdev->dev);
}
#endif
static int fujitsu_hdmi_dai_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct hdmi_priv *priv = snd_soc_dai_get_drvdata(dai);
	int err;
	/*
	 * Make sure that the period bytes are multiple of the DMA packet size.
	 * Largest packet size we use is 32 32-bit words = 128 bytes
	 */
	err = snd_pcm_hw_constraint_step(substream->runtime, 0,
				 SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 128);
	if (err < 0) {
		dev_err(dai->dev, "could not apply constraint\n");
		return err;
	}
	/*
	if (!priv->fdbdev->ops->audio_supported(priv->fdbdev)) {
		dev_err(dai->dev, "audio not supported\n");
		return -ENODEV;
	}
	*/
	snd_soc_dai_set_dma_data(dai, substream, &priv->dma_data);

	return 0;
}

static int fujitsu_hdmi_dai_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct hdmi_priv *priv = snd_soc_dai_get_drvdata(dai);

	return priv->fdbdev->ops->audio_enable(priv->fdbdev);
}

static int fujitsu_hdmi_dai_set_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	struct f_saif_info *i2s = f_get_i2s(F_HDMI_SDIN0);

	pr_debug("Entered %s\n", __func__);

	/* MASTER */
	i2s->i2s_cntreg |= F_SAIF_CNTREG_MSMD;
	i2s->master = F_SAIF_MASTER_MODE;

	/* I2S mode */
	i2s->i2s_cntreg &= ~F_SAIF_CNTREG_FSPH;
	i2s->i2s_cntreg |= F_SAIF_CNTREG_FSLN;
	i2s->format = SND_SOC_DAIFMT_I2S;

	/* normal bit clock + frame */
	i2s->i2s_cntreg |= F_SAIF_CNTREG_CPOL;
	i2s->i2s_cntreg &= ~F_SAIF_CNTREG_FSPL;

	return 0;
}

static int fujistu_hdmi_dai_set_sysclk(struct snd_soc_dai *cpu_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct f_saif_info *i2s = f_get_i2s(F_HDMI_SDIN0);

	/* ECLK */
	i2s->i2s_cntreg |= F_SAIF_CNTREG_ECKM;
	i2s->clk_rate = freq;

	return 0;
}

static int fujitsu_hdmi_dai_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai)
{
	struct hdmi_priv *priv = snd_soc_dai_get_drvdata(dai);
	struct snd_aes_iec958 *iec = &priv->iec;
	struct snd_cea_861_aud_if *cea = &priv->cea;
	unsigned int mclk_rate;
	int err = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mb8ac0300_pcm_runtime_data *prtd = runtime->private_data;
	struct f_saif_info *i2s = f_get_i2s(F_HDMI_SDIN0);

	if (f_hw_params(prtd, params, i2s))
		dev_err(dai->dev, "fail to set hw params\n");

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		priv->dma_data.maxburst = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
		priv->dma_data.maxburst = 32;
		break;
	default:
		dev_err(dai->dev, "format not supported!\n");
		return -EINVAL;
	}

	/*
	 * fill the IEC-60958 channel status word
	 */
	/* initialize the word bytes */
	memset(iec->status, 0, sizeof(iec->status));

	/* specify IEC-60958-3 (commercial use) */
	iec->status[0] &= ~IEC958_AES0_PROFESSIONAL;

	/* specify that the audio is LPCM*/
	iec->status[0] &= ~IEC958_AES0_NONAUDIO;

	iec->status[0] |= IEC958_AES0_CON_NOT_COPYRIGHT;

	iec->status[0] |= IEC958_AES0_CON_EMPHASIS_NONE;

	iec->status[0] |= IEC958_AES1_PRO_MODE_NOTID;

	iec->status[1] = IEC958_AES1_CON_GENERAL;

	iec->status[2] |= IEC958_AES2_CON_SOURCE_UNSPEC;

	iec->status[2] |= IEC958_AES2_CON_CHANNEL_UNSPEC;

	switch (params_rate(params)) {
	case 32000:
		iec->status[3] |= IEC958_AES3_CON_FS_32000;
		break;
	case 44100:
		iec->status[3] |= IEC958_AES3_CON_FS_44100;
		break;
	case 48000:
		iec->status[3] |= IEC958_AES3_CON_FS_48000;
		break;
	case 88200:
		iec->status[3] |= IEC958_AES3_CON_FS_88200;
		break;
	case 96000:
		iec->status[3] |= IEC958_AES3_CON_FS_96000;
		break;
	case 176400:
		iec->status[3] |= IEC958_AES3_CON_FS_176400;
		break;
	case 192000:
		iec->status[3] |= IEC958_AES3_CON_FS_192000;
		break;
	default:
		dev_err(dai->dev, "rate not supported!\n");
		return -EINVAL;
	}

	/* specify the clock accuracy */
	iec->status[3] |= IEC958_AES3_CON_CLOCK_1000PPM;

	/* MCLK is multiple of Fs  = 128 * Fs */
	mclk_rate = params_rate(params) * 128;

	/*
	 * specify the word length. The same word length value can mean
	 * two different lengths. Hence, we need to specify the maximum
	 * word length as well.
	 */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		iec->status[4] |= IEC958_AES4_CON_WORDLEN_20_16;
		iec->status[4] &= ~IEC958_AES4_CON_MAX_WORDLEN_24;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iec->status[4] |= IEC958_AES4_CON_WORDLEN_24_20;
		iec->status[4] |= IEC958_AES4_CON_MAX_WORDLEN_24;
		break;
	default:
		dev_err(dai->dev, "format not supported!\n");
		return -EINVAL;
	}

	/*
	 * Fill the CEA-861 audio infoframe (see spec for details)
	 */

	cea->db1_ct_cc = (params_channels(params) - 1)
		& CEA861_AUDIO_INFOFRAME_DB1CC;
	cea->db1_ct_cc |= CEA861_AUDIO_INFOFRAME_DB1CT_FROM_STREAM;

	cea->db2_sf_ss = CEA861_AUDIO_INFOFRAME_DB2SF_FROM_STREAM;
	cea->db2_sf_ss |= CEA861_AUDIO_INFOFRAME_DB2SS_FROM_STREAM;

	cea->db3 = 0; /* not used, all zeros */

	/*
	 * The HDMI IP requires to use the 8-channel channel code when
	 * transmitting more than two channels.
	 */
	if (params_channels(params) == 2)
		cea->db4_ca = 0x0;
	else
		cea->db4_ca = 0x13;

	cea->db5_dminh_lsv = CEA861_AUDIO_INFOFRAME_DB5_DM_INH_PROHIBITED;
	/* the expression is trivial but makes clear what we are doing */
	cea->db5_dminh_lsv |= (0 & CEA861_AUDIO_INFOFRAME_DB5_LSV);

	priv->fdb_audio.iec = iec;
	priv->fdb_audio.cea = cea;

	err = priv->fdbdev->ops->audio_config(priv->fdbdev,
						 &priv->fdb_audio);

	return err;
}

static int fujitsu_hdmi_dai_trigger(struct snd_pcm_substream *substream,
				int cmd, struct snd_soc_dai *dai)
{
	struct hdmi_priv *priv = snd_soc_dai_get_drvdata(dai);
	int err = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		f_saif_tx_enable(F_HDMI_SDIN0);
#if 0 // vivien start to add
		f_saif_tx_enable(F_HDMI_SDIN1);
		f_saif_tx_enable(F_HDMI_SDIN2);
		f_saif_tx_enable(F_HDMI_SDIN3);
#endif // vivien add
		err = priv->fdbdev->ops->audio_start(priv->fdbdev);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		f_saif_tx_disable(F_HDMI_SDIN0);
#if 0 // vivien start to add
		f_saif_tx_disable(F_HDMI_SDIN1);
		f_saif_tx_disable(F_HDMI_SDIN2);
		f_saif_tx_disable(F_HDMI_SDIN3);
#endif // vivien stop to add
		priv->fdbdev->ops->audio_stop(priv->fdbdev);
		break;
	default:
		err = -EINVAL;
	}
	return err;
}

static void fujitsu_hdmi_dai_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct hdmi_priv *priv = snd_soc_dai_get_drvdata(dai);

	priv->fdbdev->ops->audio_disable(priv->fdbdev);
}

static const struct snd_soc_dai_ops fujitsu_hdmi_dai_ops = {
	.startup	= fujitsu_hdmi_dai_startup,
	.hw_params	= fujitsu_hdmi_dai_hw_params,
	.prepare	= fujitsu_hdmi_dai_prepare,
	.trigger	= fujitsu_hdmi_dai_trigger,
	.shutdown	= fujitsu_hdmi_dai_shutdown,
	.set_fmt        = fujitsu_hdmi_dai_set_fmt,
	.set_sysclk     = fujistu_hdmi_dai_set_sysclk,
};

static struct snd_soc_dai_driver fujitsu_hdmi_dai = {
	.playback = {
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_32000 |
				SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &fujitsu_hdmi_dai_ops,
};

static const struct snd_soc_component_driver fujitsu_hdmi_component = {
	.name		= DRV_NAME,
};

void * f_hdmi_dai_get_drv_data(void)
{
	return &fujitsu_hdmi_dai;
}
EXPORT_SYMBOL_GPL(f_hdmi_dai_get_drv_data);

static int fujitsu_hdmi_probe(struct platform_device *pdev)
{
	int ret;
	/*struct resource *hdmi_rsrc;*/
	struct hdmi_priv *hdmi_data;
	bool hdmi_dev_found = false;
	struct f_fdb *bus = NULL;
	int n;

	hdmi_data = devm_kzalloc(&pdev->dev, sizeof(*hdmi_data), GFP_KERNEL);
	if (hdmi_data == NULL) {
		dev_err(&pdev->dev, "Cannot allocate memory for HDMI data\n");
		return -ENOMEM;
	}

	/*
	 * TODO: We assume that there is only one FDB HDMI device. Future
	 * implementations may support more than one HDMI devices and
	 * we should provided separate audio support for all of them.
	 */
	/* Find an HDMI device. */
	#if 0
	for_each_dss_dev(hdmi_data->fdbdev) {
		fujitsu_fdb_get_device(hdmi_data->fdbdev);

		if (!hdmi_data->fdbdev->ops) {
			fujitsu_fdb_put_device(hdmi_data->fdbdev);
			continue;
		}

		if (hdmi_data->fdbdev->ops->get_connector_type ==
			DRM_MODE_CONNECTOR_HDMIA) {
			hdmi_dev_found = true;
			break;
		}
	}
	#endif
	bus = f_fdb_from_dev(&pdev->dev);

	for (n = 0; n < bus->count_fdb_children; n++) {
		if (bus->child[n]->ops != NULL &&
				bus->child[n]->ops->get_connector_type) {
			if (bus->child[n]->ops->get_connector_type(bus->child[n]) ==
				DRM_MODE_CONNECTOR_HDMIA) {
				hdmi_dev_found = true;
				hdmi_data->fdbdev = bus->child[n];
				break;
			}
		}
	}

	if (!hdmi_dev_found) {
		dev_err(&pdev->dev, "no driver for HDMI display found\n");
		return -ENODEV;
	}

	dev_set_drvdata(&pdev->dev, hdmi_data);
	ret = snd_soc_register_component(&pdev->dev, &fujitsu_hdmi_component,
					 &fujitsu_hdmi_dai, 1);

	return ret;
}

static int fujitsu_hdmi_remove(struct platform_device *pdev)
{
	struct hdmi_priv *hdmi_data = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_component(&pdev->dev);

	if (hdmi_data == NULL) {
		dev_err(&pdev->dev, "cannot obtain HDMI data\n");
		return -ENODEV;
	}

	/*fujitsu_fdb_put_device(hdmi_data->fdbdev);*/
	return 0;
}

static const struct of_device_id mb86s70_hdmi_dai_dt_ids[] = {
	        { .compatible = "fujitsu,f_hdmi_audio_dai" },
		{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, mb86s70_hdmi_dai_dt_ids);

static struct platform_driver hdmi_dai_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mb86s70_hdmi_dai_dt_ids,
	},
	.probe = fujitsu_hdmi_probe,
	.remove = fujitsu_hdmi_remove,
};

module_platform_driver(hdmi_dai_driver);

MODULE_AUTHOR("Fujitsu Semiconductor Limited");
MODULE_DESCRIPTION("SoC Digital Audio Interface for HDMI audio");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
