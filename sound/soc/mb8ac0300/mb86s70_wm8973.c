/*
 * linux/sound/soc/fujitsu/mb86s70_wm8973.c
 *
 * Copyright (C) 2013 FUJITSU GLOBAL MOBILE PLATFORM INC.
 * Copyright (C) 2015-2016 Socionext Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "f_saif.h"
#include "mb8ac0300_pcm.h"
#include "../codecs/wm8973.h"

/* driver name */
#define DRIVER_NAME "mb86s70_wm8973"

/**
 * mb86s70_wm8973_hw_params - set parameters of sound device
 * @substream:	pointer of substream
 * @params:	pointer of parameters
 *
 * Returns 0 if no error, -EINVAL or other negative errno on failure
 */
static int mb86s70_wm8973_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int rate = 0;
	int ret = 0;

	pr_debug("Entered %s\n", __func__);
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret)
		return ret;

	/* set i2s system clock */
	ret = snd_soc_dai_set_sysclk(cpu_dai, F_SAIF_CLKSRC_ECLK,
					24576000, SND_SOC_CLOCK_IN);
	if (ret)
		return ret;

	switch (params_rate(params)) {
	case 8000:
	case 12000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 96000:
		rate = 12288000;
		break;
	default:
		pr_info(DRIVER_NAME": Sample rate not supported\n");
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

	return 0;
}

/* operations of sound device */
static struct snd_soc_ops mb86s70_wm8973_ops = {
	.hw_params = mb86s70_wm8973_hw_params,
};

/* digital audio interface of sound device */
static struct snd_soc_dai_link mb86s70_wm8973_dai_link[] = {

	[0] = {
		.name = "wm8973-playback",
		.stream_name = "Playback",
		.codec_name = "wm8973.1-001a",      // I2C ch.1, addr=0x1a
		.codec_dai_name = "wm8973-hifi-playback",
		.be_id = 0,
		.cpu_dai_name = "36440000.fsaif",   // I2S ch.4
		.platform_name	= "0.mb8ac0300_pcm",
		.ops = &mb86s70_wm8973_ops,
	},
	[1] = {
		.name = "wm8973-capture",
		.stream_name = "Capture",
		.codec_name = "wm8973.1-001a",     // I2C ch.1, addr=0x1a
		.codec_dai_name = "wm8973-hifi-capture",
		.be_id = 0,
		.cpu_dai_name = "36450000.fsaif",  // I2S ch.5
		.platform_name  = "0.mb8ac0300_pcm",
		.ops = &mb86s70_wm8973_ops,
	},

};

/* card information of sound device */
static struct snd_soc_card snd_soc_mb86s70_wm8973 = {
	.name = "mb86s70_snd",
	.driver_name = DRIVER_NAME,
	.dai_link = mb86s70_wm8973_dai_link,
	.num_links = ARRAY_SIZE(mb86s70_wm8973_dai_link),
};

/**
 * mb86s70_wm8973_probe - mb86s70 sound card probe function
 * @pdev:	pointer of platform device
 *
 * Returns 0 if no error, -ENOMEM -EINVAL or other negative errno on failure
 */
static int mb86s70_wm8973_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_card *card = &snd_soc_mb86s70_wm8973;
	struct f_saif_info *i2s = f_get_i2s(4);

	pr_debug("Entered %s\n", __func__);

	if ( i2s->regs == NULL ) {
		dev_err(&pdev->dev, "fsaif not registered\n");
		return -EPROBE_DEFER;
	}

	card->dev = &pdev->dev;

	ret = snd_soc_register_card(card);
	if (ret)
		pr_info(DRIVER_NAME": Failed to register card!\n");

	return ret;
}

/**
 * mb86s70_wm8973_remove - mb86s70 sound card remove function
 * @pdev:	pointer of platform device
 *
 * Returns 0
 */
static int mb86s70_wm8973_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	pr_debug("Entered %s\n", __func__);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id mb86s70_wm8973_dt_ids[] = {
	{ .compatible = "fujitsu,mb86s70_wm8973" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, mb86s70_wm8973_dt_ids);

/* machine driver information of platform device */
static struct platform_driver mb86s70_wm8973_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mb86s70_wm8973_dt_ids,
	},
	.probe  = mb86s70_wm8973_probe,
	.remove = mb86s70_wm8973_remove,
};
module_platform_driver(mb86s70_wm8973_driver);

MODULE_AUTHOR("FMPI");
MODULE_DESCRIPTION("FMPI mb86s70_wm8973 Machine module");
MODULE_LICENSE("GPL");
MODULE_ALIAS("*mb86s70_wm8973*");
