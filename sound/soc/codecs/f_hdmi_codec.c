/*
 * ALSA SoC codec driver for HDMI audio on MB86S70.
 * Copyright (C) 2013 FUJITSU SEMICONDUCTOR LIMITED
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include <linux/module.h>
#include <sound/soc.h>

#define DRV_NAME "hdmi-audio-codec"

#if 0 // vivien start to add
static const struct snd_kcontrol_new hdmi_snd_controls[] = {
	/* Left & Right Channel Digital Volume */
	SOC_DOUBLE_R("PCM Volume", WM8973_LDAC, WM8973_RDAC, 0, 255, 0),

	/* LOUT2 & ROUT2 volume */
	SOC_DOUBLE_R("Speaker Playback Volume", WM8973_LOUT2V, WM8973_ROUT2V, 0, 127, 0),
};
//static const struct snd_soc_dapm_widget hdmi_dapm_widgets[] = {
//};
//static const struct snd_soc_dapm_route hdmi_dapm_routes[] = {
//};
static struct snd_soc_codec_driver fujitsu_hdmi_codec = {
	.controls = hdmi_snd_controls,
	.num_controls = ARRAY_SIZE(hdmi_snd_controls),
//	.dapm_widgets = hdmi_dapm_widgetsi,
//	.num_dapm_widgets = ARRAY_SIZE(hdmi_dapm_widgets),
//	.dapm_routes = hdmi_dapm_routes,
//	.num_dapm_routes = ARRAY_SIZE(hdmi_dapm_routes),
};
#else // vivien stop to add
static struct snd_soc_codec_driver fujitsu_hdmi_codec;
#endif // vivien add

static int fujitsu_hdmi_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	return 0;
}

static int fujitsu_hdmi_set_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	return 0;
}

static int fujitsu_hdmi_set_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static const struct snd_soc_dai_ops fujitsu_hdmi_dai_ops = {
	.hw_params	= fujitsu_hdmi_hw_params,
	.set_fmt	= fujitsu_hdmi_set_fmt,
	.set_sysclk	= fujitsu_hdmi_set_sysclk,
};

static struct snd_soc_dai_driver fujitsu_hdmi_codec_dai = {
	.name = "fujitsu-hdmi-hifi",
	.playback = {
		.stream_name = "HDMI-audio",
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

static int fujitsu_hdmi_codec_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &fujitsu_hdmi_codec,
			&fujitsu_hdmi_codec_dai, 1);
}

static int fujitsu_hdmi_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

void * f_hdmi_codec_get_drv_data(void)
{
	return &fujitsu_hdmi_codec_dai;
}
EXPORT_SYMBOL_GPL(f_hdmi_codec_get_drv_data);

static const struct of_device_id mb86s70_hdmi_codec_dt_ids[] = {
	        { .compatible = "fujitsu,f_hdmi_codec" },
		{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, mb86s70_hdmi_codec_dt_ids);

static struct platform_driver fujitsu_hdmi_codec_driver = {
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = mb86s70_hdmi_codec_dt_ids,
	},

	.probe		= fujitsu_hdmi_codec_probe,
	.remove		= fujitsu_hdmi_codec_remove,
};

module_platform_driver(fujitsu_hdmi_codec_driver);

MODULE_AUTHOR("Fujitsu Semiconductor Limited");
MODULE_DESCRIPTION("ASoC HDMI codec driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("*f_hdmi_codec*");
