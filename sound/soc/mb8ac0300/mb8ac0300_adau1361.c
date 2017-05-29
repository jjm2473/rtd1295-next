/*
 * linux/sound/soc/mb8ac0300/mb8ac0300_adau1361.c
 *
 * Copyright (C) 2011-2012 FUJITSU SEMICONDUCTOR LIMITED
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
#include "../codecs/adau17x1.h"

/* driver name */
#define DRIVER_NAME	"mb8ac0300_adau1361"

#define MB8AC0300_IIS_GPIO_NO	26
#define MB8AC0300_IIS_DATA_IN	0
#define MB8AC0300_IIS_DATA_OUT	1

/**
 * mb8ac0300_adau1361_startup - start up device
 * @substream:	pointer of substream
 *
 * Returns 0 if no error, -EBUSY on failure
 */
static int mb8ac0300_adau1361_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	/* hardware supports only one play or capture stream simultaneously */
	if (cpu_dai->active)
		return -EBUSY;

	return 0;
}

/**
 * mb8ac0300_adau1361_hw_params - set parameters of sound device
 * @substream:	pointer of substream
 * @params:	pointer of parameters
 *
 * Returns 0 if no error, -EINVAL or other negative errno on failure
 */
static int mb8ac0300_adau1361_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned pll_rate;
	int ret = 0;

	pr_debug("Entered %s\n", __func__);
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret)
		return ret;

	/* set i2s system clock */
	ret = snd_soc_dai_set_sysclk(cpu_dai,
		F_SAIF_CLKSRC_ECLK, 24576000, SND_SOC_CLOCK_IN);
	if (ret)
		return ret;

	if (params_rate(params) % 4000)
		pll_rate = 44100 * 1024;
	else
		pll_rate = 48000 * 1024;

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai,
			ADAU17X1_CLK_SRC_PLL, pll_rate, SND_SOC_CLOCK_IN);
	if (ret)
		return ret;

	/* set codec pll */
	ret = snd_soc_dai_set_pll(codec_dai,
		ADAU17X1_PLL, ADAU17X1_PLL_SRC_MCLK, 24576000,
		pll_rate);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret)
		return ret;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		gpio_set_value(MB8AC0300_IIS_GPIO_NO, MB8AC0300_IIS_DATA_IN);
	else
		gpio_set_value(MB8AC0300_IIS_GPIO_NO, MB8AC0300_IIS_DATA_OUT);

	return 0;
}

/* operations of sound device */
static struct snd_soc_ops mb8ac0300_adau1361_ops = {
	.hw_params = mb8ac0300_adau1361_hw_params,
	.startup = mb8ac0300_adau1361_startup,
};

/* digital audio interface of sound device */
static struct snd_soc_dai_link mb8ac0300_adau1361_dai_link = {
	.name = "adau1x61",
	.stream_name = "adau1x61",
	/* 0:i2c channel no, 0038: i2c address */
	.codec_name = "adau1x61.0-0038",
	.codec_dai_name = "adau-hifi",
	/* 0:i2s channel no */
	.cpu_dai_name = "fff44000.f_saif",
	.ops = &mb8ac0300_adau1361_ops,
	.platform_name	= "mb8ac0300_pcm.2",
};

/* card information of sound device */
static struct snd_soc_card snd_soc_mb8ac0300_adau1361 = {
	.name = "mb8ac0300_snd",
	.driver_name = DRIVER_NAME,
	.dai_link = &mb8ac0300_adau1361_dai_link,
	.num_links = 1,
};

/**
 * mb8ac0300_adau1361_probe - mb8ac0300 sound card probe function
 * @pdev:	pointer of platform device
 *
 * Returns 0 if no error, -ENOMEM -EINVAL or other negative errno on failure
 */
static int mb8ac0300_adau1361_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_card *card = &snd_soc_mb8ac0300_adau1361;

	pr_debug("Entered %s\n", __func__);

	ret = gpio_request(MB8AC0300_IIS_GPIO_NO, "I2S");
	if (ret) {
		pr_err(DRIVER_NAME": failed to get gpio port: %d\n", ret);
		return ret;
	}

	gpio_direction_output(MB8AC0300_IIS_GPIO_NO, MB8AC0300_IIS_DATA_OUT);

	card->dev = &pdev->dev;

	ret = snd_soc_register_card(card);
	if (ret) {
		pr_info(DRIVER_NAME": Failed to register card!\n");
		gpio_free(MB8AC0300_IIS_GPIO_NO);
	}

	return ret;
}

/**
 * mb8ac0300_adau1361_remove - mb8ac0300 sound card remove function
 * @pdev:	pointer of platform device
 *
 * Returns 0
 */
static int mb8ac0300_adau1361_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	pr_debug("Entered %s\n", __func__);
	gpio_free(MB8AC0300_IIS_GPIO_NO);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id mb8ac0300_adau1361_dt_ids[] = {
	{ .compatible = "fujitsu,mb8ac0300_adau1361" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, mb8ac0300_adau1361_dt_ids);

/* machine driver information of platform device */
static struct platform_driver mb8ac0300_adau1361_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mb8ac0300_adau1361_dt_ids,
	},
	.probe  = mb8ac0300_adau1361_probe,
	.remove = mb8ac0300_adau1361_remove,
};
module_platform_driver(mb8ac0300_adau1361_driver);

MODULE_AUTHOR("Fujitsu Semiconductor Limited");
MODULE_DESCRIPTION("Fujitsu Semiconductor mb8ac0300_adau1361 Machine module");
MODULE_LICENSE("GPL");
MODULE_ALIAS("*mb8ac0300_adau1361*");
