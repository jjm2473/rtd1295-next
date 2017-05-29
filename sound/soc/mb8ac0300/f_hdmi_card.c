/*
 * linux/sound/soc/mb8ac0300/f_hdmi_card.c
 *
 * Copyright (C) 2013 FUJITSU SEMICONDUCTOR LIMITED
 * Copyright (C) 2015-2016 Socionext Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 */


#include <linux/module.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <asm/mach-types.h>
#include "f_saif.h"

#define DRV_NAME "fujitsu-hdmi-audio"

/**
 * Returns 0 if no error, -EINVAL or other negative errno on failure
 */
static int hdmi_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	//unsigned int rate = 0;
	int ret = 0;

	pr_debug("Entered %s\n", __func__);
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret)
		return ret;

	/* set i2s system clock */
#if 0 // vivien start to add
	ret = snd_soc_dai_set_sysclk(cpu_dai, F_SAIF_CLKSRC_ECLK,
					5645000, SND_SOC_CLOCK_IN);
#else // vivien stop to add
	ret = snd_soc_dai_set_sysclk(cpu_dai, F_SAIF_CLKSRC_ECLK,
					24576000, SND_SOC_CLOCK_IN);
#endif // vivien add
	if (ret)
		return ret;

#if 0
	switch (params_rate(params)) {
	case 8000:
	case 12000:
	case 16000:
	case 24000:
	case 32000:
	case 44100:
	case 48000:
	case 96000:
		rate = 12288000;
		break;
	default:
		pr_info(DRV_NAME": Sample rate not supported\n");
		return -EINVAL;
	}

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, rate, SND_SOC_CLOCK_OUT);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret)
		return ret;
#endif

	return 0;
}

/* operations of sound device */
static struct snd_soc_ops hdmi_ops = {
	.hw_params = hdmi_hw_params,
};

static struct snd_soc_dai_link fujitsu_hdmi_dai = {
	.name = "HDMI",
	.stream_name = "HDMI",
	.cpu_dai_name = "0.f_hdmi_audio_dai",
	.platform_name = "0.hdmi_pcm",
	.codec_name = "0.f_hdmi_codec",
	.be_id = 0,
	.ops = &hdmi_ops,
	.codec_dai_name = "fujitsu-hdmi-hifi",
};

static struct snd_soc_card snd_soc_fujitsu_hdmi = {
	.name = "FUJITSU-HDMI",
	.owner = THIS_MODULE,
	.dai_link = &fujitsu_hdmi_dai,
	.num_links = 1,
};

void * f_hdmi_dai_get_drv_data(void);
void * f_hdmi_pcm_get_platform(void);
void * f_hdmi_codec_get_drv_data(void);

static int fujitsu_hdmi_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_fujitsu_hdmi;
	int ret;

	if( !f_hdmi_dai_get_drv_data()
	 || !f_hdmi_pcm_get_platform()
	 || !f_hdmi_codec_get_drv_data() ) {
		return -EPROBE_DEFER;
	}

	card->dev = &pdev->dev;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_info(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		card->dev = NULL;
		return ret;
	}
	return 0;
}

static int fujitsu_hdmi_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	card->dev = NULL;
	return 0;
}

static const struct of_device_id mb86s70_hdmi_dt_ids[] = {
	        { .compatible = "fujitsu,f_hdmi_audio" },
		{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, mb86s70_hdmi_dt_ids);

static struct platform_driver fujitsu_hdmi_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mb86s70_hdmi_dt_ids,
	},
	.probe = fujitsu_hdmi_probe,
	.remove = fujitsu_hdmi_remove,
};

module_platform_driver(fujitsu_hdmi_driver);

MODULE_AUTHOR("Fujitsu Semiconductor Limited");
MODULE_DESCRIPTION("FUJITSU HDMI machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
