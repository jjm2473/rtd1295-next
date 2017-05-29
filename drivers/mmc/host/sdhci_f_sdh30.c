/*
 * linux/drivers/mmc/host/sdhci_f_sdh30.c
 *
 * Copyright (C) 2013 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 */

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/host.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/gpio-revision.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>

#include "sdhci.h"
#include "sdhci-pltfm.h"
#include "sdhci_f_sdh30.h"

#define DRIVER_NAME "f_sdh30"

#define SDCHI_F_SD30_POWER_CONTROL 0x124


struct f_sdhost_priv {
	struct clk *clk_sd4;
	struct clk *clk_b;
	u32 vendor_hs200;
	struct device *dev;
};

void sdhci_f_sdh30_soft_voltage_switch(struct sdhci_host *host)
{
	struct f_sdhost_priv *priv = sdhci_priv(host);
	u32 ctrl = 0;

	usleep_range(2500, 3000);
	ctrl = sdhci_readl(host, F_SDH30_IO_CONTROL2);
	ctrl |= F_SDH30_CRES_O_DN;
	sdhci_writel(host, ctrl, F_SDH30_IO_CONTROL2);
	ctrl |= F_SDH30_MSEL_O_1_8;
	sdhci_writel(host, ctrl, F_SDH30_IO_CONTROL2);

	ctrl &= ~F_SDH30_CRES_O_DN;
	sdhci_writel(host, ctrl, F_SDH30_IO_CONTROL2);
	usleep_range(2500, 3000);

	if (priv->vendor_hs200) {
		dev_info(priv->dev, "%s: setting hs200\n", __func__);
		ctrl = sdhci_readl(host, F_SDH30_ESD_CONTROL);
		ctrl |= priv->vendor_hs200;
		sdhci_writel(host, ctrl, F_SDH30_ESD_CONTROL);
	}

	ctrl= sdhci_readl(host, F_SDH30_TUNING_SETTING);
	ctrl |= F_SDH30_CMD_CHK_DIS;
	sdhci_writel(host, ctrl, F_SDH30_TUNING_SETTING);
}

unsigned int sdhci_f_sdh30_get_min_clock(struct sdhci_host *host)
{
	return F_SDH30_MIN_CLOCK;
}

void sdhci_f_sdh30_reset_enter(struct sdhci_host *host, u8 mask)
{
	if (sdhci_readw(host, SDHCI_CLOCK_CONTROL) == 0) {
		sdhci_writew(host, 0xBC01, SDHCI_CLOCK_CONTROL);
		mmiowb();
	}
}

void sdhci_f_sdh30_reset_exit(struct sdhci_host *host, u8 mask)
{
	int ctrl = 0;

	if (host->quirks2 & SDHCI_QUIRK2_SIG_VOL_FIXED_1_8V) {
		/* sdio(wlan): fixed 1.8V */
		ctrl = sdhci_readl(host, F_SDH30_IO_CONTROL2);
		ctrl |= F_SDH30_MSEL_O_1_8;
		sdhci_writel(host, ctrl, F_SDH30_IO_CONTROL2);
	}
}

static const struct sdhci_ops sdhci_f_sdh30_ops = {
	.voltage_switch = sdhci_f_sdh30_soft_voltage_switch,
	.get_min_clock = sdhci_f_sdh30_get_min_clock,
	.platform_reset_enter = sdhci_f_sdh30_reset_enter,
	.platform_reset_exit = sdhci_f_sdh30_reset_exit,
};

static int sdhci_f_sdh30_probe(struct platform_device *pdev)
{
	struct sdhci_host *host;
	struct resource *iomem;
	struct device *dev = &pdev->dev;
	int irq, ctrl = 0, ret = 0;
	struct f_sdhost_priv *priv;
	u32 reg, caps2, bus_width;
	u32 tuning_count_default;

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem) {
		dev_err(dev, "%s: resource get error\n", __func__);
		ret = -ENOENT;
		goto err;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "%s: no irq specified\n", __func__);
		ret = irq;
		goto err;
	}

	host = sdhci_alloc_host(dev, sizeof(struct sdhci_host) + sizeof(struct f_sdhost_priv));
	if (IS_ERR(host)) {
		dev_err(dev, "%s: host allocate error\n", __func__);
		ret = -ENOMEM;
		goto err;
	}
	priv = sdhci_priv(host); 
	priv->dev = dev;

	host->quirks =  SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC;
	host->quirks2 = SDHCI_QUIRK2_SUPPORT_SINGLE |
			SDHCI_QUIRK2_UNSUPPORT_3_0_V |
			SDHCI_QUIRK2_VOLTAGE_SWITCH |
			SDHCI_QUIRK2_TUNING_WORK_AROUND |
			SDHCI_QUIRK2_IGNORE_UNEXPECTED_IRQ;

	if (gpio_revision_of_bitmap(pdev->dev.of_node, "revs-no-1v8")) {
		dev_info(dev, "Applying no 1.8V quirk\n");
		host->quirks2 |= SDHCI_QUIRK2_NO_1_8_V;
	}

	if (!of_property_read_u32(pdev->dev.of_node, "mmc-caps2", &caps2)) {
		dev_info(dev, "Applying mmc capabilities 2\n");
		host->mmc->caps2 |= caps2;
	}

	if (!of_property_read_u32(pdev->dev.of_node, "vendor-hs200", &priv->vendor_hs200))
		dev_info(dev, "Applying vendor-hs200 setting\n");
	else
		priv->vendor_hs200 = 0;

	if (!of_property_read_u32(pdev->dev.of_node, "bus-width", &bus_width)) {
		if (bus_width == 8) {
			dev_info(dev, "Applying 8 bit bus width\n");
			host->mmc->caps |= MMC_CAP_8_BIT_DATA;
		}
	}

	if (of_find_property(pdev->dev.of_node, "resume-detect-retry", NULL)){
		dev_info(dev, "Applying resume detect retry quirk\n");
		host->quirks2 |= SDHCI_QUIRK2_RESUME_DETECT_RETRY;
	}

	if (of_find_property(pdev->dev.of_node, "no-dma", NULL)){
		dev_info(dev, "Applying no dma quirk\n");
		host->quirks |= SDHCI_QUIRK_BROKEN_DMA;
		host->quirks |= SDHCI_QUIRK_BROKEN_ADMA;
	}

	if (of_find_property(pdev->dev.of_node, "inverted-write-protect", NULL)){
		dev_info(dev, "Applying inverted write protect quirk\n");
		host->quirks |= SDHCI_QUIRK_INVERTED_WRITE_PROTECT;
	}

	if (of_find_property(pdev->dev.of_node, "enable-fixed-18v", NULL)) {
		dev_info(dev, "Applying enable-fixed-18v quirk\n");
		host->quirks2 |= SDHCI_QUIRK2_SIG_VOL_FIXED_1_8V;
	}

	if (of_find_property(pdev->dev.of_node, "keep-power-in-suspend", NULL)){
		dev_info(dev, "Applying keep-power-in-suspend quirk\n");
        host->mmc->pm_caps |= MMC_PM_KEEP_POWER;
	}

	if (of_find_property(pdev->dev.of_node, "enable-sdio-wakeup", NULL)) {
		dev_info(dev, "Applying enable-sdio-wakeup quirk\n");
        host->mmc->pm_caps |= MMC_PM_WAKE_SDIO_IRQ;
    }

    if ( ( ( host->mmc->pm_caps & MMC_PM_WAKE_SDIO_IRQ ) != 0) &&
         ( ( host->mmc->pm_caps & MMC_PM_KEEP_POWER ) != 0) ) {

        device_set_wakeup_capable(dev, 1);
    }

	host->hw_name = DRIVER_NAME;
	host->ops = &sdhci_f_sdh30_ops;
	host->irq = irq;

	if (!of_property_read_u32(pdev->dev.of_node, "tuning_count_default", &tuning_count_default)) {
		host->tuning_count = tuning_count_default;
		dev_info(dev, "%s: tuning_count = %08x\n", __func__, host->tuning_count);
	} else {
		host->tuning_count = SDHCI_RETUNING_TIMER_DEFAULT;
	}

	if (!request_mem_region(iomem->start, resource_size(iomem),
		mmc_hostname(host->mmc))) {
		dev_err(dev, "%s: cannot request region\n", __func__);
		ret = -ENXIO;
		goto err_request;
	}

	host->ioaddr = ioremap_nocache(iomem->start, resource_size(iomem));
	if (!host->ioaddr) {
		dev_err(dev, "%s: failed to remap registers\n", __func__);
		ret = -ENXIO;
		goto err_remap;
	}

	priv->clk_sd4 = clk_get(&pdev->dev, "sd_sd4clk");
	if (!IS_ERR(priv->clk_sd4)) {
		ret = clk_prepare_enable(priv->clk_sd4);
		if (ret < 0) {
			dev_err(dev, "Failed to enable sd4 clock: %d\n", ret);
			goto err_clk1;
		}
	}
	priv->clk_b = clk_get(&pdev->dev, "sd_bclk");
	if (!IS_ERR(priv->clk_b)) {
		ret = clk_prepare_enable(priv->clk_b);
		if (ret < 0) {
			dev_err(dev, "Failed to enable sd4 clock: %d\n", ret);
			goto err_clk2;
		}
	}

	platform_set_drvdata(pdev, host);

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		dev_err(dev, "Failed to pm_runtime_get_sync: %d\n", ret);
	}
#endif

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(dev, "%s: host add error\n", __func__);
		goto err_add_host;
	}

	/* init vendor specific regs */
	ctrl = sdhci_readw(host, F_SDH30_AHB_CONFIG);
	ctrl |= F_SDH30_SIN |
		F_SDH30_AHB_INCR_16 | F_SDH30_AHB_INCR_8 |
		F_SDH30_AHB_INCR_4;
	ctrl &= ~F_SDH30_AHB_BIGED;
	ctrl &= ~F_SDH30_BUSLOCK_EN;

	sdhci_writew(host, ctrl, F_SDH30_AHB_CONFIG);

	mmiowb();

	reg = readl(host->ioaddr + SDCHI_F_SD30_POWER_CONTROL);
	dev_dbg(priv->dev, "%s: 0x%llx read 0x%x\n", __func__,
						(u64)iomem->start, reg);
	writel(reg & ~2, host->ioaddr + SDCHI_F_SD30_POWER_CONTROL);

	msleep(10);

	writel(reg | 2, host->ioaddr + SDCHI_F_SD30_POWER_CONTROL);
	dev_dbg(priv->dev, "%s: write 0x%x\n", __func__,
			readl(host->ioaddr + SDCHI_F_SD30_POWER_CONTROL));

	mmiowb();

	return 0;

err_add_host:
	clk_put(priv->clk_sd4);
err_clk2:
	clk_put(priv->clk_b);	
err_clk1:
	iounmap(host->ioaddr);
err_remap:
	release_mem_region(iomem->start, resource_size(iomem));
err_request:
	sdhci_free_host(host);
err:
	return ret;
}

static int sdhci_f_sdh30_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct f_sdhost_priv *priv = sdhci_priv(host);
	struct resource *iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	int dead;
	u32 scratch;

	dead = 0;
	scratch = readl(host->ioaddr + SDHCI_INT_STATUS);
	if (scratch == (u32)-1)
		dead = 1;

	sdhci_remove_host(host, dead);
	iounmap(host->ioaddr);
	release_mem_region(iomem->start, resource_size(iomem));

	clk_disable_unprepare(priv->clk_sd4);
	clk_disable_unprepare(priv->clk_b);

	clk_put(priv->clk_b);
	clk_put(priv->clk_sd4);

	sdhci_free_host(host);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP

static u16 saved_irq_signal_enable = 0;

static int sdhci_f_sdh30_suspend(struct device *dev)
{
    u32 present;
	struct sdhci_host *host = dev_get_drvdata(dev);
	int ret = 0;
	dev_info(dev, "%s start \n", __func__);
    if (!device_may_wakeup(mmc_dev(host->mmc))) {
        dev_info(dev, "%s sdhci_suspend_host call\n", __func__);
        ret = sdhci_suspend_host(host);
    } else {
        dev_info(dev, "%s use sdio wakeup call\n", __func__);

        /* save sdhci nrm irq signal enable */
        saved_irq_signal_enable = sdhci_readw(host, SDHCI_SIGNAL_ENABLE);

        /* disable all nrm irq signal */
        sdhci_writew(host, 0, SDHCI_SIGNAL_ENABLE);

        /* debug sdhci nrm irq status enable */
        dev_dbg(dev, "nrm irq status enable 0x%04x",
                 sdhci_readw(host, SDHCI_INT_ENABLE) );

        /* debug sdhci nrm irq signal enable */
        dev_dbg( dev, "nrm irq signal enable 0x%04x",
                 saved_irq_signal_enable );

        /* get card status */
        present = sdhci_readl(host, SDHCI_PRESENT_STATE);

        /* debug sdhci wakeup control */
        dev_dbg( dev, "wake up control 0x%02x",
                 sdhci_readb(host,SDHCI_WAKE_UP_CONTROL) );

        if ( (present  & SDHCI_CARD_PRESENT) != 0) {


            dev_info(dev, "%s card present. wake_up_control: REMOVE,INT.\n", __func__);
            /* case card inserted */
            sdhci_writeb(host,
                         (SDHCI_WAKE_ON_REMOVE | SDHCI_WAKE_ON_INT),
                         SDHCI_WAKE_UP_CONTROL);

        } else {

            dev_info(dev, "%s card not present. wake_up_control: INSERT.\n", __func__);

            /* case card not inserted */
            sdhci_writeb(host,
                         SDHCI_WAKE_ON_INSERT,
                         SDHCI_WAKE_UP_CONTROL);
        }
    }
	dev_info(dev, "%s end \n", __func__);
	return ret;
}

#define DEBOUNCING_TIMEOUT 1000
static int sdhci_f_sdh30_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	int ret = 0;
	dev_info(dev, "%s start \n", __func__);
    if (!device_may_wakeup(mmc_dev(host->mmc))) { 
        int wait_msec = 0;
        while (!(sdhci_readl(host, SDHCI_PRESENT_STATE) & SDHCI_CARD_STABLE)){
            if ( wait_msec >= DEBOUNCING_TIMEOUT) {
                dev_err(dev, "%s: debouncing time out error (%d msec)\n", __func__, wait_msec);
                return 1;
            }
            mdelay(1);
            wait_msec++;
        }
        dev_info(dev, "%s: transition time to the stable state was %d msec.\n", __func__, wait_msec);
        ret = sdhci_resume_host(host);
    } else {
        dev_dbg(dev, "%s orig_resume call\n", __func__);

        sdhci_writeb(host,
                     0,
                     SDHCI_WAKE_UP_CONTROL);

        sdhci_writew(host, saved_irq_signal_enable, SDHCI_SIGNAL_ENABLE);

        saved_irq_signal_enable = 0;
    }
	dev_info(dev, "%s end \n", __func__);
	return ret;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int sdhci_f_sdh30_runtime_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	int ret;

	dev_info(dev, "%s start \n", __func__);
	ret = sdhci_runtime_suspend_host(host);
	dev_info(dev, "%s end \n", __func__);
	return ret;
}

static int sdhci_f_sdh30_runtime_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	int ret;

	dev_info(dev, "%s start \n", __func__);
	ret = sdhci_runtime_resume_host(host);
	dev_info(dev, "%s end \n", __func__);
	return ret;
}
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops sdhci_f_sdh30_pmops = {
	SET_SYSTEM_SLEEP_PM_OPS(sdhci_f_sdh30_suspend, sdhci_f_sdh30_resume)
	SET_RUNTIME_PM_OPS(sdhci_f_sdh30_runtime_suspend, sdhci_f_sdh30_runtime_resume,
			   NULL)
};

#define SDHCI_F_SDH30_PMOPS (&sdhci_f_sdh30_pmops)

#else
#define SDHCI_F_SDH30_PMOPS NULL
#endif

static const struct of_device_id f_sdh30_dt_ids[] = {
	{ .compatible = "fujitsu,f_sdh30" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, f_sdh30_dt_ids);

static struct platform_driver sdhci_f_sdh30_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = f_sdh30_dt_ids,
#ifdef CONFIG_PM_SLEEP
		.pm	= SDHCI_F_SDH30_PMOPS,
#endif
	},
	.probe	= sdhci_f_sdh30_probe,
	.remove	= sdhci_f_sdh30_remove,
};

static int __init sdhci_f_sdh30_init(void)
{
	return platform_driver_register(&sdhci_f_sdh30_driver);
}
module_init(sdhci_f_sdh30_init);

static void __exit sdhci_f_sdh30_exit(void)
{
	platform_driver_unregister(&sdhci_f_sdh30_driver);
}
module_exit(sdhci_f_sdh30_exit);

MODULE_DESCRIPTION("F_SDH30 SD Card Controller driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("FUJITSU SEMICONDUCTOR LTD.");
MODULE_ALIAS("platform: " DRIVER_NAME);
