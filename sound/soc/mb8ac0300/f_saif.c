/*
 * linux/sound/soc/mb8ac0300/f_saif.c
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include "mb8ac0300_pcm.h"
#include "f_saif.h"
#include <linux/interrupt.h>
#include <linux/slab.h>

#define DRIVER_NAME "f_saif"


#ifdef DEBUG
struct dumps {
	const char *name;
	unsigned int start;
	unsigned int len;
};

static void f_saif_dump_regs(struct f_saif_info *par)
{
	void __iomem *base = par->regs;
	int n, i;
	struct dumps dumps[] = {
		{ "Regs", 8, 0x1c },
	};

	dev_info(par->dev, "============================f_saif_dump_regs %x\n",
							   (unsigned int)base);

	for (i = 0; i < ARRAY_SIZE(dumps); i++)
		for (n = 0; n <= dumps[i].len; n += 4)
			dev_info(par->dev, "%s: +0x%04x: 0x%08X\n",
				dumps[i].name, dumps[i].start + n,
					__raw_readl(base + dumps[i].start + n));
}
#else
#define f_saif_dump_regs(_x)
#endif



/**
 * f_saif_irq - i2s interrupt function
 * @id :	id of interrupt
 * @i2s:	pointer of i2s information
 * Returns IRQ_HANDLED
 */
static irqreturn_t f_saif_irq(int id, void *i2s)
{
	struct f_saif_info *p = (struct f_saif_info *) i2s;
	u32 status = F_SAIF_GET_STATUS(p);

	pr_debug(DRIVER_NAME": i2s err status: 0x%x\n", status);

	F_SAIF_SET_STATUS(p, status);

	return IRQ_HANDLED;
}

/**
 * f_saif_txctrl - setting i2s sending data enable
 * @cpu_dai:	pointer of i2s digital audio interface
 * @on:		setting enable/disable
 */
static void f_saif_txctrl(struct snd_soc_dai *cpu_dai, int on)
{
	struct f_saif_info *i2s = dev_get_drvdata(cpu_dai->dev);

	pr_debug("Entered %s (on = %d)\n", __func__, on);

	spin_lock(&i2s->lock);

	if (on) {
		/* initialize I2S */
		F_SAIF_SET_SRST(i2s, F_SAIF_SRST_SRST); /* software reset */
		while ((F_SAIF_GET_SRST(i2s) & F_SAIF_SRST_SRST))
			;

		F_SAIF_SET_OPR(i2s, 0);
		F_SAIF_SET_CNTREG(i2s, i2s->i2s_cntreg & ~F_SAIF_CNTREG_TXDIS);
		F_SAIF_SET_MCR0(i2s, i2s->i2s_mcr0);
		F_SAIF_SET_MCR1(i2s, i2s->i2s_mcr1);
		F_SAIF_SET_MCR2(i2s, i2s->i2s_mcr2);
		F_SAIF_SET_INTCNT(i2s, i2s->i2s_intcnt);
		F_SAIF_SET_OPR(i2s, F_SAIF_OPR_TX_ENABLE);
		F_SAIF_SET_DMAACT(i2s, F_SAIF_DMA_TX_REQ);

		f_saif_dump_regs(i2s);

	} else {
		F_SAIF_SET_OPR(i2s, 0);
		F_SAIF_SET_CNTREG(i2s, i2s->i2s_cntreg |
					F_SAIF_CNTREG_TXDIS);
		F_SAIF_SET_DMAACT(i2s, 0);
	}

	spin_unlock(&i2s->lock);

}

/**
 * f_saif_rxctrl - setting i2s receiving data enable
 * @cpu_dai:	pointer of i2s digital audio interface
 * @on:		setting enable/disable
 */
static void f_saif_rxctrl(struct snd_soc_dai *cpu_dai, int on)
{
	struct f_saif_info *i2s = dev_get_drvdata(cpu_dai->dev);

	pr_debug("Entered %s\n", __func__);

	spin_lock(&i2s->lock);

	if (on) {
		/* initialize I2S */
		F_SAIF_SET_SRST(i2s, F_SAIF_SRST_SRST); /* software reset */
		while ((F_SAIF_GET_SRST(i2s) & F_SAIF_SRST_SRST) != 0)
			;

		F_SAIF_SET_OPR(i2s, 0x00000000UL);
		F_SAIF_SET_CNTREG(i2s, i2s->i2s_cntreg &
					~F_SAIF_CNTREG_RXDIS);
		F_SAIF_SET_MCR0(i2s, i2s->i2s_mcr0);
		F_SAIF_SET_MCR1(i2s, i2s->i2s_mcr1);
		F_SAIF_SET_MCR2(i2s, i2s->i2s_mcr2);
		F_SAIF_SET_INTCNT(i2s, i2s->i2s_intcnt);
		F_SAIF_SET_OPR(i2s, F_SAIF_OPR_RX_ENABLE);
		F_SAIF_SET_DMAACT(i2s, F_SAIF_DMA_RX_REQ);
	} else {
		F_SAIF_SET_OPR(i2s, 0x00000000UL);
		F_SAIF_SET_CNTREG(i2s, i2s->i2s_cntreg |
					F_SAIF_CNTREG_RXDIS);
		F_SAIF_SET_DMAACT(i2s, 0x00000000UL);
	}

	spin_unlock(&i2s->lock);
}

/**
 * f_saif_remove - i2s module remove function
 * @pdev:	pointer of platform device
 *
 * Returns 0
 */
static int f_saif_remove(struct platform_device *pdev)
{
	struct f_saif_info *i2s = dev_get_drvdata(&pdev->dev);

	pr_debug("Entered %s\n", __func__);

	clk_disable_unprepare(i2s->clk);
	clk_put(i2s->clk);
	iounmap(i2s->regs);
	free_irq(i2s->irq, i2s);
	kfree(i2s);
	snd_soc_unregister_component(&pdev->dev);
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

/**
 * f_saif_set_sysclk - set i2s system clock
 * @cpu_dai:	pointer of i2s digital audio interface
 * @clk_id:	id of clock source
 * @freq:	frequence of clock source(unit:hz)
 * @dir:	direction of clock source(input/output)
 *
 * Returns 0 if no error, -EINVAL errno on failure
 */
static int f_saif_set_sysclk(struct snd_soc_dai *cpu_dai,
			 int clk_id, unsigned int freq, int dir)
{
	struct f_saif_info *i2s = dev_get_drvdata(cpu_dai->dev);

	pr_debug("Entered %s\n", __func__);

	switch (clk_id) {
	case F_SAIF_CLKSRC_AHB:
		i2s->i2s_cntreg &= ~F_SAIF_CNTREG_ECKM;
		i2s->clk_rate = clk_get_rate(i2s->clk);
		break;
	case F_SAIF_CLKSRC_ECLK:
		i2s->i2s_cntreg |= F_SAIF_CNTREG_ECKM;
		i2s->clk_rate = freq;
		break;
	default:
		dev_err(cpu_dai->dev, "Bad sysclk choice\n");
		return -EINVAL;
	}

	return 0;
}

/**
 * f_saif_set_fmt - set i2s format
 * @cpu_dai:	pointer of i2s digital audio interface
 * @fmt:	i2s format to be set
 *
 * Returns 0 if no error, -EINVAL errno on failure
 */
static int f_saif_set_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	struct f_saif_info *i2s = dev_get_drvdata(cpu_dai->dev);

	pr_debug("Entered %s\n", __func__);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		i2s->i2s_cntreg &= ~F_SAIF_CNTREG_MSMD; /* SLAVE */
		i2s->master = F_SAIF_SLAVE_MODE;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		i2s->i2s_cntreg |= F_SAIF_CNTREG_MSMD; /* MASTER */
		i2s->master = F_SAIF_MASTER_MODE;
		break;
	default:
		pr_err(DRIVER_NAME": invalid parameters! id:%d\n", cpu_dai->id);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		i2s->i2s_cntreg &= ~F_SAIF_CNTREG_FSPH;
		i2s->i2s_cntreg |= F_SAIF_CNTREG_FSLN;
		i2s->format = SND_SOC_DAIFMT_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		i2s->i2s_cntreg |= F_SAIF_CNTREG_FSPH;
		i2s->i2s_cntreg |= F_SAIF_CNTREG_FSLN;
		i2s->format = SND_SOC_DAIFMT_LEFT_J;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		i2s->i2s_cntreg &= ~F_SAIF_CNTREG_FSPH;
		i2s->i2s_cntreg &= ~F_SAIF_CNTREG_FSLN;
		i2s->format = SND_SOC_DAIFMT_DSP_A;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		i2s->i2s_cntreg |= F_SAIF_CNTREG_FSPH;
		i2s->i2s_cntreg &= ~F_SAIF_CNTREG_FSLN;
		i2s->format = SND_SOC_DAIFMT_DSP_B;
		break;
	default:
		pr_err(DRIVER_NAME": invalid parameters! id:%d\n", cpu_dai->id);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		i2s->i2s_cntreg |= F_SAIF_CNTREG_CPOL;
		i2s->i2s_cntreg &= ~F_SAIF_CNTREG_FSPL;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		i2s->i2s_cntreg |= F_SAIF_CNTREG_CPOL;
		i2s->i2s_cntreg |= F_SAIF_CNTREG_FSPL;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		i2s->i2s_cntreg &= ~F_SAIF_CNTREG_CPOL;
		i2s->i2s_cntreg &= ~F_SAIF_CNTREG_FSPL;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		i2s->i2s_cntreg &= ~F_SAIF_CNTREG_CPOL;
		i2s->i2s_cntreg |= F_SAIF_CNTREG_FSPL;
		break;
	default:
		pr_err(DRIVER_NAME": invalid parameters! id:%d\n", cpu_dai->id);
		return -EINVAL;
	}

    /* If dai format is SND_SOC_DAIFMT_I2S, set FSPL bit ON.*/
	if (i2s->format == SND_SOC_DAIFMT_I2S) {
		i2s->i2s_cntreg |= F_SAIF_CNTREG_FSPL;
	}

	return 0;
}

/**
 * f_saif_hw_params - set parameters of i2s
 * @substream:	pointer of substream
 * @params:	pointer of parameters
 * @cpu_dai:	pointer of i2s digital audio interface
 *
 * Returns 0 if no error, -EINVAL errno on failure
 */
static int f_saif_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *cpu_dai)
{
	unsigned int div;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mb8ac0300_pcm_runtime_data *prtd = runtime->private_data;
	struct f_saif_info *i2s = dev_get_drvdata(cpu_dai->dev);
	unsigned int div_min, div_max;
	unsigned int channel_length;

	pr_debug("Entered %s\n", __func__);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		/* I2s transmission FIFO register Address */
		prtd->params.dma_addr =
			(dma_addr_t)(i2s->regs_phys + F_SAIF_REG_TXFDAT);
	else
		/* I2s reception FIFO register Address */
		prtd->params.dma_addr =
			(dma_addr_t)(i2s->regs_phys + F_SAIF_REG_RXFDAT);

	i2s->channels = params_channels(params);

	if ((i2s->channels < F_SAIF_MIN_SOFT_CHANNELS) ||
				(i2s->channels > F_SAIF_MAX_SOFT_CHANNELS)) {
		pr_err(DRIVER_NAME": i2s channels out of range id:%d\n",
								cpu_dai->id);
		return -EINVAL;
	}

	if (i2s->channels == 1) {
		/* mono */
		i2s->i2s_mcr1 &= ~F_SAIF_MCR1_MASK;
		switch (i2s->format) {
		case SND_SOC_DAIFMT_I2S:
		case SND_SOC_DAIFMT_DSP_A:
		case SND_SOC_DAIFMT_DSP_B:
		case SND_SOC_DAIFMT_LEFT_J:
			/* enable  left channel */
			i2s->i2s_mcr1 |= (1 << 1);
			break;
		default:
			dev_err(cpu_dai->dev, "i2s format out of range\n");
			return -EINVAL;
		}

		i2s->channels = 2;

	} else {
		i2s->i2s_mcr1 &= ~F_SAIF_MCR1_MASK;
		i2s->i2s_mcr1 |= (1 << i2s->channels) - 1;
	}

	i2s->i2s_mcr0 &= ~F_SAIF_MCR0_S0CHN_MASK;
	i2s->i2s_mcr0 |= (i2s->channels - 1) << F_SAIF_MCR0_S0CHN_BIT_OFFSET;

	i2s->i2s_mcr0 &= ~F_SAIF_MCR0_S0CHL_MASK;
	i2s->i2s_mcr0 &= ~F_SAIF_MCR0_S0WDL_MASK;

	i2s->i2s_cntreg &= ~F_SAIF_CNTREG_BEXT;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		i2s->i2s_cntreg |= F_SAIF_CNTREG_BEXT;
	case SNDRV_PCM_FORMAT_U8:
		i2s->bits = F_SAIF_WORD_LENGTH_8BIT;
		channel_length = F_SAIF_WORD_LENGTH_8BIT;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		i2s->i2s_cntreg |= F_SAIF_CNTREG_BEXT;
	case SNDRV_PCM_FORMAT_U16_LE:
		i2s->bits = F_SAIF_WORD_LENGTH_16BIT;
		channel_length = F_SAIF_WORD_LENGTH_32BIT;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		i2s->i2s_cntreg |= F_SAIF_CNTREG_BEXT;
	case SNDRV_PCM_FORMAT_U24_LE:
		i2s->bits = F_SAIF_WORD_LENGTH_24BIT;
		channel_length = F_SAIF_WORD_LENGTH_32BIT;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		i2s->i2s_cntreg |= F_SAIF_CNTREG_BEXT;
	case SNDRV_PCM_FORMAT_U32_LE:
		i2s->bits = F_SAIF_WORD_LENGTH_32BIT;
		channel_length = F_SAIF_WORD_LENGTH_32BIT;
		break;
	default:
		dev_err(cpu_dai->dev, "unknown params_format\n");
		return -EINVAL;
	}

	/* word length */
	i2s->i2s_mcr0 |= (i2s->bits - 1) << F_SAIF_MCR0_S0WDL_BIT_OFFSET;
	/* channel length */
	i2s->i2s_mcr0 |= (channel_length - 1) << F_SAIF_MCR0_S0CHL_BIT_OFFSET;

	i2s->rate = params_rate(params);

	/* div */
	if ((i2s->channels == 0) || (i2s->rate == 0) || (i2s->bits == 0)) {
		dev_err(cpu_dai->dev, "channels/rate/bits on i2s bad\n");
		return -EINVAL;
	}

	if (i2s->master == F_SAIF_MASTER_MODE) {
		div = i2s->clk_rate / (i2s->channels * i2s->rate *
					channel_length * 2);

		if (i2s->i2s_cntreg & F_SAIF_CNTREG_ECKM) {
			div_min = F_SAIF_MIN_ECLK_DIV;
			div_max = F_SAIF_MAX_ECLK_DIV;
		} else {
			div_min = F_SAIF_MIN_AHB_DIV;
			div_max = F_SAIF_MAX_AHB_DIV;
		}

		if ((div < div_min) || (div > div_max)) {
			dev_err(cpu_dai->dev, "bad divider\n");
			return -EINVAL;
		}
		i2s->i2s_cntreg &= ~F_SAIF_CNTREG_CKRT_MASK;
		i2s->i2s_cntreg |= div << F_SAIF_CNTREG_CKRT_BIT_OFFSET;
	}


	f_saif_dump_regs(i2s);

	return 0;
}

/**
 * f_saif_trigger - start/stop i2s by judging command
 * @substream:	pointer of substream
 * @cmd:	command to process
 * @cpu_dai:	pointer of i2s digital audio interface
 *
 * Returns 0 if no error, -EINVAL errno on failure
 */
static int f_saif_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *cpu_dai)
{
	int ret = 0;

	pr_debug("Entered %s\n", __func__);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			f_saif_rxctrl(cpu_dai, F_SAIF_ENABLE);
		else
			f_saif_txctrl(cpu_dai, F_SAIF_ENABLE);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			f_saif_rxctrl(cpu_dai, F_SAIF_DISABLE);
		else
			f_saif_txctrl(cpu_dai, F_SAIF_DISABLE);
		break;
	default:
		dev_err(cpu_dai->dev, "uknown cmd\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* i2s operations */
static struct snd_soc_dai_ops f_saif_dai_ops = {
	.trigger	= f_saif_trigger,
	.hw_params	= f_saif_hw_params,
	.set_fmt	= f_saif_set_fmt,
	.set_sysclk	= f_saif_set_sysclk,
};

/* i2s digital audio interface */
struct snd_soc_dai_driver f_saif_dai_init = {
	.name = DRIVER_NAME,
	.playback = {
		.channels_min = F_SAIF_MIN_SOFT_CHANNELS,
		.channels_max = F_SAIF_MAX_SOFT_CHANNELS,
		.rates = F_SAIF_RATES,
		.formats = F_SAIF_FMTS,
	},
	.capture = {
		.channels_min = F_SAIF_MIN_SOFT_CHANNELS,
		.channels_max = F_SAIF_MAX_SOFT_CHANNELS,
		.rates = F_SAIF_RATES,
		.formats = F_SAIF_FMTS,
	},
	.ops = &f_saif_dai_ops,
};

static const struct snd_soc_component_driver f_saif_i2s_component = {
	.name = "f_saif-i2s",
};

/* i2s digital audio interface */
static struct snd_soc_dai_driver f_saif_dai[CONFIG_F_SAIF_MAX_CH_NUM];
static struct f_saif_info *f_saif_priv[CONFIG_F_SAIF_MAX_CH_NUM];
/**
 * f_saif_probe - i2s probe function
 * @pdev:	pointer of platform device
 *
 * Returns 0 if no error, -ENXIO -EINVAL -ENOMEM or other negative errno
 * on failure
 */
static int f_saif_probe(struct platform_device *pdev)
{
	struct f_saif_info *i2s;
	struct snd_soc_dai_driver *dai;
	struct device *dev = &pdev->dev;
	int ret, irq, err;
	struct resource *res;
	int n;

	pr_debug("Entered %s\n", __func__);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if ((!res) || (irq < 0)) {
		dev_err(&pdev->dev, "Missing memory resource\n");
		return -ENXIO;
	}

	if (pdev->id == -1)
		for (n = 0; n < CONFIG_F_SAIF_MAX_CH_NUM; n++)
			if (!f_saif_dai[n].name) {
				pdev->id = n;
				break;
			}

	if (pdev->id == -1 || pdev->id >= CONFIG_F_SAIF_MAX_CH_NUM) {
		dev_err(&pdev->dev, "pdev id out of range\n");
		return -EINVAL;
	}

	i2s = kzalloc(sizeof(struct f_saif_info), GFP_KERNEL);
	if (!i2s) {
		dev_err(&pdev->dev, "Out of memory\n");
		return -ENOMEM;
	}

	i2s->i2s_cntreg = F_SAIF_CNTREG_TXDIS | F_SAIF_CNTREG_RXDIS;
    i2s->i2s_cntreg |= F_SAIF_CNTREG_FRUN;
	i2s->i2s_mcr0 = 0x00000000UL;
	i2s->i2s_mcr1 = 0x00000000UL;
	i2s->i2s_mcr2 = 0x00000000UL;
	i2s->i2s_intcnt = F_SAIF_INTCNT_RXFIM | F_SAIF_INTCNT_TXFIM |
			F_SAIF_INTCNT_TFTH | F_SAIF_INTCNT_RFTH;
	i2s->i2s_oprreg = 0x00000000UL;
	i2s->i2s_dmaact = 0x00000000UL;
	i2s->dev = dev;
	i2s->regs_phys = res->start;
	i2s->irq = irq;
	i2s->master = F_SAIF_SLAVE_MODE;
	spin_lock_init(&i2s->lock);

	/* record our i2s structure for later use in the callbacks */
	f_saif_dai[pdev->id] = f_saif_dai_init;
	f_saif_dai[pdev->id].id = pdev->id;
	dai = &f_saif_dai[pdev->id];
	f_saif_priv[pdev->id] = i2s;

	dev_set_drvdata(&pdev->dev, i2s);

	err = request_irq(irq, f_saif_irq, 0, pdev->name, i2s);
	if (err) {
		dev_err(&pdev->dev, "Missing irq resource\n");
		ret = err;
		goto mem_free;
	}

	i2s->regs = ioremap(res->start, res->end - res->start + 1);
	if (i2s->regs == NULL) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto irq_free;
	}

	i2s->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(i2s->clk)) {
		ret = PTR_ERR(i2s->clk);
		dev_err(&pdev->dev, "Failed to get clock\n");
		goto dai_free;
	}

	ret = clk_prepare_enable(i2s->clk);
	if (ret) {
		pr_err(DRIVER_NAME": failed to enable audio-bus");
		goto dai_free;
	}

	ret = snd_soc_register_component(&pdev->dev, &f_saif_i2s_component,
								      dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register dai\n");
		goto iomap_free;
	}
	dev_info(&pdev->dev, "Registered id %d %s\n", pdev->id, dai->name);
	return 0;

dai_free:
	snd_soc_unregister_component(&pdev->dev);
iomap_free:
	iounmap(i2s->regs);
irq_free:
	free_irq(irq, i2s);
mem_free:
	kfree(i2s);

	return ret;
}

struct f_saif_info *f_get_i2s(unsigned int id)
{
	return f_saif_priv[id];
}
EXPORT_SYMBOL_GPL(f_get_i2s);

int f_hw_params(struct mb8ac0300_pcm_runtime_data *prtd,
		struct snd_pcm_hw_params *params, struct f_saif_info *i2s)
{
	unsigned int div;
	unsigned int div_min, div_max;
	unsigned int channel_length;

	/* I2s transmission FIFO register Address */
	prtd->params.dma_addr =
			(dma_addr_t)(i2s->regs_phys + F_SAIF_REG_TXFDAT);

	i2s->channels = params_channels(params);

	if ((i2s->channels < F_SAIF_MIN_SOFT_CHANNELS) ||
				(i2s->channels > F_SAIF_MAX_SOFT_CHANNELS)) {
		pr_err(DRIVER_NAME": i2s channels out of range\n");
		i2s->channels = 2;
		//return -EINVAL;
	}

	if (i2s->channels == 1) {
		/* mono */
		i2s->i2s_mcr1 &= ~F_SAIF_MCR1_MASK;
		switch (i2s->format) {
		case SND_SOC_DAIFMT_I2S:
		case SND_SOC_DAIFMT_DSP_A:
		case SND_SOC_DAIFMT_DSP_B:
		case SND_SOC_DAIFMT_LEFT_J:
			/* enable  left channel */
			i2s->i2s_mcr1 |= (1 << 1);
			break;
		default:
			pr_err(DRIVER_NAME"i2s format out of range\n");
			return -EINVAL;
		}

		i2s->channels = 2;

	} else {
		i2s->i2s_mcr1 &= ~F_SAIF_MCR1_MASK;
		i2s->i2s_mcr1 |= (1 << i2s->channels) - 1;
	}

	i2s->i2s_mcr0 &= ~F_SAIF_MCR0_S0CHN_MASK;
	i2s->i2s_mcr0 |= (i2s->channels - 1) << F_SAIF_MCR0_S0CHN_BIT_OFFSET;

	i2s->i2s_mcr0 &= ~F_SAIF_MCR0_S0CHL_MASK;
	i2s->i2s_mcr0 &= ~F_SAIF_MCR0_S0WDL_MASK;

	i2s->i2s_cntreg &= ~F_SAIF_CNTREG_BEXT;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		i2s->i2s_cntreg |= F_SAIF_CNTREG_BEXT;
	case SNDRV_PCM_FORMAT_U8:
		i2s->bits = F_SAIF_WORD_LENGTH_8BIT;
		channel_length = F_SAIF_WORD_LENGTH_8BIT;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		i2s->i2s_cntreg |= F_SAIF_CNTREG_BEXT;
	case SNDRV_PCM_FORMAT_U16_LE:
		i2s->bits = F_SAIF_WORD_LENGTH_16BIT;
		channel_length = F_SAIF_WORD_LENGTH_32BIT;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		i2s->i2s_cntreg |= F_SAIF_CNTREG_BEXT;
	case SNDRV_PCM_FORMAT_U24_LE:
		i2s->bits = F_SAIF_WORD_LENGTH_24BIT;
		channel_length = F_SAIF_WORD_LENGTH_32BIT;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		i2s->i2s_cntreg |= F_SAIF_CNTREG_BEXT;
	case SNDRV_PCM_FORMAT_U32_LE:
		i2s->bits = F_SAIF_WORD_LENGTH_32BIT;
		channel_length = F_SAIF_WORD_LENGTH_32BIT;
		break;
	default:
		pr_err(DRIVER_NAME"unknown params_format\n");
		return -EINVAL;
	}

	/* word length */
	i2s->i2s_mcr0 |= (i2s->bits - 1) << F_SAIF_MCR0_S0WDL_BIT_OFFSET;
	/* channel length */
	i2s->i2s_mcr0 |= (channel_length - 1) << F_SAIF_MCR0_S0CHL_BIT_OFFSET;

	i2s->rate = params_rate(params);

	/* div */
	if ((i2s->channels == 0) || (i2s->rate == 0) || (i2s->bits == 0)) {
		pr_err(DRIVER_NAME"channels/rate/bits on i2s bad\n");
		return -EINVAL;
	}

	if (i2s->master == F_SAIF_MASTER_MODE) {
		div = i2s->clk_rate / (i2s->channels * i2s->rate *
					channel_length * 2);

		if (i2s->i2s_cntreg & F_SAIF_CNTREG_ECKM) {
			div_min = F_SAIF_MIN_ECLK_DIV;
			div_max = F_SAIF_MAX_ECLK_DIV;
		} else {
			div_min = F_SAIF_MIN_AHB_DIV;
			div_max = F_SAIF_MAX_AHB_DIV;
		}

		if ((div < div_min) || (div > div_max)) {
			pr_err(DRIVER_NAME"bad divider\n");
			return -EINVAL;
		}
		i2s->i2s_cntreg &= ~F_SAIF_CNTREG_CKRT_MASK;
		i2s->i2s_cntreg |= div << F_SAIF_CNTREG_CKRT_BIT_OFFSET;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(f_hw_params);

void f_saif_tx_enable(unsigned int id)
{
	struct f_saif_info *i2s = f_saif_priv[id];

	pr_err("%s\n", __func__);
	
	spin_lock(&i2s->lock);

	F_SAIF_SET_SRST(i2s, F_SAIF_SRST_SRST); /* software reset */
	while ((F_SAIF_GET_SRST(i2s) & F_SAIF_SRST_SRST))
		;

#if 1
	F_SAIF_SET_OPR(i2s, 0);
	F_SAIF_SET_CNTREG(i2s, i2s->i2s_cntreg & ~((1 << 9) | F_SAIF_CNTREG_TXDIS));
	F_SAIF_SET_MCR0(i2s, i2s->i2s_mcr0);
	F_SAIF_SET_MCR1(i2s, i2s->i2s_mcr1);
	F_SAIF_SET_MCR2(i2s, i2s->i2s_mcr2);
	F_SAIF_SET_INTCNT(i2s, i2s->i2s_intcnt);
	F_SAIF_SET_OPR(i2s, F_SAIF_OPR_TX_ENABLE);
	F_SAIF_SET_DMAACT(i2s, F_SAIF_DMA_TX_REQ);
	
	pr_err("%s: OPR=%x, CNTREG=%x, MCR0=%x, MCR1=%x, MCR2=%x, INTCNT=%x\n",
	       __func__, F_SAIF_OPR_TX_ENABLE,
		i2s->i2s_cntreg & ~F_SAIF_CNTREG_TXDIS,
		i2s->i2s_mcr0, i2s->i2s_mcr1, i2s->i2s_mcr2,
		i2s->i2s_intcnt);
	

#else
// !!!
	F_SAIF_SET_OPR(i2s, 0x00000000UL);
	F_SAIF_SET_CNTREG(i2s, 0x04002D2B); // test code was 0x08...
	F_SAIF_SET_MCR0(i2s, 0x00001DEF);
	F_SAIF_SET_MCR1(i2s, 0x00000041);
	F_SAIF_SET_MCR2(i2s, 0);
	F_SAIF_SET_INTCNT(i2s, 0x41000F00);
	F_SAIF_SET_OPR(i2s, F_SAIF_OPR_TX_ENABLE);
	F_SAIF_SET_DMAACT(i2s, F_SAIF_DMA_TX_REQ);
#endif

	spin_unlock(&i2s->lock);

}
EXPORT_SYMBOL_GPL(f_saif_tx_enable);

void f_saif_tx_disable(unsigned int id)
{
	struct f_saif_info *i2s = f_saif_priv[id];

	spin_lock(&i2s->lock);

	F_SAIF_SET_OPR(i2s, 0x00000000UL);
	F_SAIF_SET_CNTREG(i2s, i2s->i2s_cntreg |
				F_SAIF_CNTREG_TXDIS);
	F_SAIF_SET_DMAACT(i2s, 0x00000000UL);

	spin_unlock(&i2s->lock);

}
EXPORT_SYMBOL_GPL(f_saif_tx_disable);

int f_saif_mclk_set(unsigned int id, unsigned int clk_rate)
{
	struct f_saif_info *i2s = f_saif_priv[id];
	/*unsigned int channel_length;*/
	unsigned int div, div_min, div_max;

	if (i2s->master == F_SAIF_MASTER_MODE) {

		div = (i2s->clk_rate / clk_rate) / 2;

		if (i2s->i2s_cntreg & F_SAIF_CNTREG_ECKM) {
			div_min = F_SAIF_MIN_ECLK_DIV;
			div_max = F_SAIF_MAX_ECLK_DIV;
		} else {
			div_min = F_SAIF_MIN_AHB_DIV;
			div_max = F_SAIF_MAX_AHB_DIV;
		}

		if ((div < div_min) || (div > div_max))
			return -EINVAL;

		i2s->i2s_cntreg &= ~F_SAIF_CNTREG_CKRT_MASK;
		i2s->i2s_cntreg |= div << F_SAIF_CNTREG_CKRT_BIT_OFFSET;
		/*F_SAIF_SET_CNTREG(i2s, i2s->i2s_cntreg);*/

	} else
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL_GPL(f_saif_mclk_set);

int f_saif_params_set(unsigned int id,
			struct snd_pcm_hw_params *params)
{
	struct f_saif_info *i2s = f_saif_priv[id];
	unsigned int channel_length;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		i2s->i2s_cntreg |= F_SAIF_CNTREG_BEXT;
	case SNDRV_PCM_FORMAT_U8:
		i2s->bits = F_SAIF_WORD_LENGTH_8BIT;
		channel_length = F_SAIF_WORD_LENGTH_8BIT;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		i2s->i2s_cntreg |= F_SAIF_CNTREG_BEXT;
	case SNDRV_PCM_FORMAT_U16_LE:
		i2s->bits = F_SAIF_WORD_LENGTH_16BIT;
		channel_length = F_SAIF_WORD_LENGTH_16BIT;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		i2s->i2s_cntreg |= F_SAIF_CNTREG_BEXT;
	case SNDRV_PCM_FORMAT_U24_LE:
		i2s->bits = F_SAIF_WORD_LENGTH_24BIT;
		channel_length = F_SAIF_WORD_LENGTH_32BIT;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		i2s->i2s_cntreg |= F_SAIF_CNTREG_BEXT;
	case SNDRV_PCM_FORMAT_U32_LE:
		i2s->bits = F_SAIF_WORD_LENGTH_32BIT;
		channel_length = F_SAIF_WORD_LENGTH_32BIT;
		break;
	default:
		return -EINVAL;
	}

	i2s->i2s_mcr0 &= ~F_SAIF_MCR0_S0CHL_MASK;
	i2s->i2s_mcr0 &= ~F_SAIF_MCR0_S0WDL_MASK;

	/* word length */
	i2s->i2s_mcr0 |= (i2s->bits - 1) << F_SAIF_MCR0_S0WDL_BIT_OFFSET;
	/* channel length */
	i2s->i2s_mcr0 |= (channel_length - 1) << F_SAIF_MCR0_S0CHL_BIT_OFFSET;
	/* sampling rate */
	i2s->rate = params_rate(params);
	/* channels */
	i2s->channels = params_channels(params);

	i2s->i2s_mcr0 &= ~F_SAIF_MCR0_S0CHN_MASK;
	i2s->i2s_mcr0 |= (i2s->channels - 1) << F_SAIF_MCR0_S0CHN_BIT_OFFSET;

	i2s->i2s_mcr1 &= ~F_SAIF_MCR1_MASK;
	i2s->i2s_mcr1 |= (1 << i2s->channels) - 1;

	return 0;
}
EXPORT_SYMBOL_GPL(f_saif_params_set);

static const struct of_device_id f_saif_dt_ids[] = {
	{ .compatible = "fujitsu,f_saif" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, f_saif_dt_ids);

/* i2s driver information */
static struct platform_driver f_saif_driver = {
	.probe = f_saif_probe,
	.remove = f_saif_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = f_saif_dt_ids,
	},
};

/**
 * f_saif_init - i2s module initialize function
 *
 * Returns 0 if no error, negative errno on failure
 */
static int __init f_saif_init(void)
{
	pr_debug("Entered %s\n", __func__);
	return platform_driver_register(&f_saif_driver);
}

/**
 * f_saif_exit - i2s module exit function
 */
static void __exit f_saif_exit(void)
{
	pr_debug("Entered %s\n", __func__);
	platform_driver_unregister(&f_saif_driver);
}

module_init(f_saif_init);
module_exit(f_saif_exit);

MODULE_AUTHOR("Fujitsu Semiconductor Limited");
MODULE_DESCRIPTION("Fujitsu Semiconductor I2S Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("*f_saif*");
