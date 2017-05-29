/*
 * MB86S7x HSSPI controller driver
 *
 * Copyright (C) 2015 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>

#define MCTRL		0x0
#define MEN	0
#define CSEN	1
#define IPCLK	3
#define MES	4

#define PCC0		0x4
#define PCC(n)		(PCC0 + (n) * 4)
#define RTM	3
#define ACES	2
#define SAFESYNC	16
#define CPHA	0
#define CPOL	1
#define SSPOL	4
#define SDIR	7
#define SS2CD	5
#define SENDIAN	8
#define CDRS_SHIFT	9
#define CDRS_MASK	0x7f

#define TXF		0x14
#define TXE		0x18
#define TXC		0x1c
#define RXF		0x20
#define RXE		0x24
#define RXC		0x28
#define TFLETE	4
#define RFMTE	5

#define FAULTF		0x2c
#define FAULTC		0x30

#define DMCFG		0x34
#define SSDC		1
#define MSTARTEN	2

#define	DMSTART		0x38
#define TRIGGER		0
#define DMSTOP		8
#define CS_MASK		3
#define CS_SHIFT	16
#define DATA_TXRX	0
#define DATA_RX		1
#define DATA_TX		2
#define DATA_MASK	3
#define DATA_SHIFT	26
#define BUS_WIDTH	24

#define	DMBCC		0x3c
#define DMSTATUS	0x40
#define RX_DATA_MASK	0x1f
#define RX_DATA_SHIFT	8
#define TX_DATA_MASK	0x1f
#define TX_DATA_SHIFT	16

#define TXBITCNT	0x44

#define FIFOCFG		0x4c
#define BPW_MASK	0x3
#define BPW_SHIFT	8
#define RX_FLUSH	11
#define TX_FLUSH	12
#define RX_TRSHLD_MASK		0xf
#define RX_TRSHLD_SHIFT		0
#define TX_TRSHLD_MASK		0xf
#define TX_TRSHLD_SHIFT		4

#define TXFIFO		0x50
#define RXFIFO		0x90
#define MID		0xfc

#define FIFO_DEPTH	16
#define TX_TRSHLD	4
#define RX_TRSHLD	(FIFO_DEPTH - TX_TRSHLD)

#define TXBIT	1
#define RXBIT	2

struct s7x_hsspi {
	spinlock_t lock;
	struct device *dev;
	struct spi_master *master;

	unsigned cs;
	unsigned bpw;
	unsigned busy;
	unsigned mode;
	unsigned speed;
	unsigned aces;
	void *rx_buf;
	const void *tx_buf;
	struct clk *clk;
	void __iomem *regs;
	unsigned tx_words, rx_words;
	unsigned bus_width;
	unsigned old_transf_mode;
};

static void read_fifo(struct s7x_hsspi *hsspi)
{
	u32 len = readl_relaxed(hsspi->regs + DMSTATUS);
	int i;

	len = (len >> RX_DATA_SHIFT) & RX_DATA_MASK;
	len = min_t(unsigned, len, hsspi->rx_words);

	switch (hsspi->bpw) {
	case 8:
		{
		u8 *buf = hsspi->rx_buf;
		for (i = 0; i < len; i++)
			*buf++ = readb_relaxed(hsspi->regs + RXFIFO);
		hsspi->rx_buf = buf;
		break;
		}
	case 16:
		{
		u16 *buf = hsspi->rx_buf;
		for (i = 0; i < len; i++)
			*buf++ = readw_relaxed(hsspi->regs + RXFIFO);
		hsspi->rx_buf = buf;
		break;
		}
	default:
		{
		u32 *buf = hsspi->rx_buf;
		for (i = 0; i < len; i++)
			*buf++ = readl_relaxed(hsspi->regs + RXFIFO);
		hsspi->rx_buf = buf;
		break;
		}
	}

	hsspi->rx_words -= len;
}

static void write_fifo(struct s7x_hsspi *hsspi)
{
	u32 len = readl_relaxed(hsspi->regs + DMSTATUS);
	int i;

	len = (len >> TX_DATA_SHIFT) & TX_DATA_MASK;
	len = min_t(unsigned, FIFO_DEPTH - len, hsspi->tx_words);

	switch (hsspi->bpw) {
	case 8:
		{
		const u8 *buf = hsspi->tx_buf;
		for (i = 0; i < len; i++)
			writeb_relaxed(*buf++, hsspi->regs + TXFIFO);
		hsspi->tx_buf = buf;
		break;
		}
	case 16:
		{
		const u16 *buf = hsspi->tx_buf;
		for (i = 0; i < len; i++)
			writew_relaxed(*buf++, hsspi->regs + TXFIFO);
		hsspi->tx_buf = buf;
		break;
		}
	default:
		{
		const u32 *buf = hsspi->tx_buf;
		for (i = 0; i < len; i++)
			writel_relaxed(*buf++, hsspi->regs + TXFIFO);
		hsspi->tx_buf = buf;
		break;
		}
	}
	hsspi->tx_words -= len;
}

static int s7x_hsspi_config(struct spi_master *master,
				  struct spi_device *spi,
				  struct spi_transfer *xfer)
{
	struct s7x_hsspi *hsspi = spi_master_get_devdata(master);
	unsigned speed, mode, bpw, cs, bus_width;
	unsigned long rate, transf_mode;
	u32 val, div;

	/* Full Duplex only on 1bit wide bus */
	if (xfer->rx_buf && xfer->tx_buf &&
			(xfer->rx_nbits != 1 || xfer->tx_nbits != 1)) {
		dev_err(hsspi->dev, "RX and TX bus widths must match!\n");
		return -EINVAL;
	}

	if (xfer->tx_buf) {
		bus_width = xfer->tx_nbits;
		transf_mode = TXBIT;
	} else {
		bus_width = xfer->rx_nbits;
		transf_mode = RXBIT;
	}

	mode = spi->mode;
	cs = spi->chip_select;
	speed = xfer->speed_hz ? : spi->max_speed_hz;
	bpw = xfer->bits_per_word ? : spi->bits_per_word;

	/* return if nothing to change */
	if (speed == hsspi->speed &&
			bus_width == hsspi->bus_width && bpw == hsspi->bpw &&
			mode == hsspi->mode && cs == hsspi->cs &&
			transf_mode == hsspi->old_transf_mode) {
		return 0;
	}

	rate = clk_get_rate(hsspi->clk);

	div = DIV_ROUND_UP(rate, speed);
	if (div > 127) {
		dev_err(hsspi->dev, "Requested rate too low (%u)\n",
			hsspi->speed);
		return -EINVAL;
	}

	if (xfer->tx_buf)
		hsspi->old_transf_mode = TXBIT;
	else
		hsspi->old_transf_mode = RXBIT;

	val = readl_relaxed(hsspi->regs + PCC(cs));
	val &= ~BIT(RTM);
	val &= ~BIT(ACES);
	val &= ~BIT(SAFESYNC);
	if (bpw == 8 &&	(mode & (SPI_TX_DUAL | SPI_RX_DUAL)) && div < 3)
		val |= BIT(SAFESYNC);
	if (bpw == 8 &&	(mode & (SPI_TX_QUAD | SPI_RX_QUAD)) && div < 6)
		val |= BIT(SAFESYNC);
	if (bpw == 16 && (mode & (SPI_TX_QUAD | SPI_RX_QUAD)) && div < 3)
		val |= BIT(SAFESYNC);

	if (mode & SPI_CPHA)
		val |= BIT(CPHA);
	else
		val &= ~BIT(CPHA);
	if (mode & SPI_CPOL)
		val |= BIT(CPOL);
	else
		val &= ~BIT(CPOL);
	if (mode & SPI_CS_HIGH)
		val |= BIT(SSPOL);
	else
		val &= ~BIT(SSPOL);
	if (mode & SPI_LSB_FIRST)
		val |= BIT(SDIR);
	else
		val &= ~BIT(SDIR);

	if (hsspi->aces)
		val |= BIT(ACES);

	val |= (3 << SS2CD);
	val |= BIT(SENDIAN);

	val &= ~(CDRS_MASK << CDRS_SHIFT);
	val |= ((div >> 1) << CDRS_SHIFT);
	writel_relaxed(val, hsspi->regs + PCC(cs));

	val = readl_relaxed(hsspi->regs + FIFOCFG);
	val &= ~(BPW_MASK << BPW_SHIFT);
	val |= ((bpw / 8 - 1) << BPW_SHIFT);
	writel_relaxed(val, hsspi->regs + FIFOCFG);

	val = readl_relaxed(hsspi->regs + DMSTART);
	val &= ~(DATA_MASK << DATA_SHIFT);

	if (xfer->tx_buf && xfer->rx_buf)
		val |= (DATA_TXRX << DATA_SHIFT);
	else if (xfer->rx_buf)
		val |= (DATA_RX << DATA_SHIFT);
	else
		val |= (DATA_TX << DATA_SHIFT);

	val &= ~(3 << BUS_WIDTH);
	val |= ((bus_width >> 1) << BUS_WIDTH);
	writel_relaxed(val, hsspi->regs + DMSTART);

	hsspi->bpw = bpw;
	hsspi->mode = mode;
	hsspi->speed = speed;
	hsspi->cs = spi->chip_select;
	hsspi->bus_width = bus_width;

	return 0;
}

static int s7x_hsspi_transfer_one(struct spi_master *master,
				  struct spi_device *spi,
				  struct spi_transfer *xfer)
{
	struct s7x_hsspi *hsspi = spi_master_get_devdata(master);
	unsigned long bpw, flags;
	int ret, words;
	u32 val;

	val = readl_relaxed(hsspi->regs + FIFOCFG);
	val |= (1 << RX_FLUSH);
	val |= (1 << TX_FLUSH);
	writel_relaxed(val, hsspi->regs + FIFOCFG);

	/* See if we can tranfer 4-bytes as 1 word even if not asked */
	bpw = xfer->bits_per_word ? : spi->bits_per_word;
	if (bpw == 8 && !(xfer->len % 4) && !(spi->mode & SPI_LSB_FIRST)) {
		bpw = xfer->bits_per_word;
		xfer->bits_per_word = 32;
	} else {
		bpw = xfer->bits_per_word;
	}

	ret = s7x_hsspi_config(master, spi, xfer);
	if (ret) {
		xfer->bits_per_word = bpw;
		return ret;
	}

	hsspi->tx_buf = xfer->tx_buf;
	hsspi->rx_buf = xfer->rx_buf;

	if (hsspi->bpw == 8)
		words = xfer->len / 1;
	else if (hsspi->bpw == 16)
		words = xfer->len / 2;
	else
		words = xfer->len / 4;

	spin_lock_irqsave(&hsspi->lock, flags);
	if (xfer->tx_buf) {
		hsspi->busy |= BIT(TXBIT);
		hsspi->tx_words = words;
	} else {
		hsspi->busy &= ~BIT(TXBIT);
		hsspi->tx_words = 0;
	}

	if (xfer->rx_buf) {
		hsspi->busy |= BIT(RXBIT);
		hsspi->rx_words = words;
	} else {
		hsspi->busy &= ~BIT(RXBIT);
		hsspi->rx_words = 0;
	}
	spin_unlock_irqrestore(&hsspi->lock, flags);

	if (xfer->tx_buf)
		write_fifo(hsspi);

	if (xfer->rx_buf) {
		val = readl_relaxed(hsspi->regs + FIFOCFG);
		val &= ~(RX_TRSHLD_MASK << RX_TRSHLD_SHIFT);
		val |= ((hsspi->rx_words > FIFO_DEPTH ?
			RX_TRSHLD : hsspi->rx_words) << RX_TRSHLD_SHIFT);
		writel_relaxed(val, hsspi->regs + FIFOCFG);
	}

	writel_relaxed(~0, hsspi->regs + TXC);
	writel_relaxed(~0, hsspi->regs + RXC);

	/* Trigger */
	val = readl_relaxed(hsspi->regs + DMSTART);
	val |= BIT(TRIGGER);
	writel_relaxed(val, hsspi->regs + DMSTART);

	while (hsspi->busy & (BIT(RXBIT) | BIT(TXBIT))) {
		if (hsspi->rx_words)
			read_fifo(hsspi);
		else
			hsspi->busy &= ~BIT(RXBIT);

		if (hsspi->tx_words)
			write_fifo(hsspi);
		else {
			u32 len;
			do { /* wait for shifter to empty out */
				cpu_relax();
				len = readl_relaxed(hsspi->regs + DMSTATUS);
				len = (len >> TX_DATA_SHIFT) & TX_DATA_MASK;
			} while (xfer->tx_buf && len);
			hsspi->busy &= ~BIT(TXBIT);
		}
	}

	/* restore */
	xfer->bits_per_word = bpw;

	return 0;
}

static void s7x_hsspi_set_cs(struct spi_device *spi, bool enable)
{
	struct s7x_hsspi *hsspi = spi_master_get_devdata(spi->master);
	u32 val;

	/*
	 * drivers/spi/spi.c:
	 * static void spi_set_cs(struct spi_device *spi, bool enable)
	 * {
	 *              if (spi->mode & SPI_CS_HIGH)
	 *                      enable = !enable;
	 *
	 *              if (spi->cs_gpio >= 0)
	 *                      gpio_set_value(spi->cs_gpio, !enable);
	 *              else if (spi->master->set_cs)
	 *              spi->master->set_cs(spi, !enable);
	 * }
	 *
	 * Note: enable(s7x_hsspi_set_cs) = !enable(spi_set_cs)
	 */
	val = readl_relaxed(hsspi->regs + DMSTART);
	val &= ~(CS_MASK << CS_SHIFT);
	val |= spi->chip_select << CS_SHIFT;

	if (!enable) {
		writel_relaxed(val, hsspi->regs + DMSTART);

		val = readl_relaxed(hsspi->regs + DMSTART);
		val &= ~BIT(DMSTOP);
		writel_relaxed(val, hsspi->regs + DMSTART);
	} else {
		val |= BIT(DMSTOP);
		writel_relaxed(val, hsspi->regs + DMSTART);

		if (hsspi->rx_buf) {
			u32 buf[16];
			hsspi->rx_buf = buf;
			hsspi->rx_words = 16;
			read_fifo(hsspi);
		}
	}
}

static int s7x_hsspi_enable(struct spi_master *master)
{
	struct s7x_hsspi *hsspi = spi_master_get_devdata(master);
	u32 val;
	int clkid;
	struct clk *clk;

	clk = devm_clk_get(hsspi->dev, "iHCLK");
	clkid = 0;
	if (IS_ERR(hsspi->clk)) {
		clk = devm_clk_get(hsspi->dev, "iPCLK");
		clkid = 1;
		if (IS_ERR(hsspi->clk)) {
			dev_err(hsspi->dev, "No source clock\n");
			return PTR_ERR(clk);
		}
	}

	/* Disable module */
	writel_relaxed(0, hsspi->regs + MCTRL);
	while (readl_relaxed(hsspi->regs + MCTRL) & BIT(MES))
		cpu_relax();

	writel_relaxed(0, hsspi->regs + TXE);
	writel_relaxed(0, hsspi->regs + RXE);
	val = readl_relaxed(hsspi->regs + TXF);
	writel_relaxed(val, hsspi->regs + TXC);
	val = readl_relaxed(hsspi->regs + RXF);
	writel_relaxed(val, hsspi->regs + RXC);
	val = readl_relaxed(hsspi->regs + FAULTF);
	writel_relaxed(val, hsspi->regs + FAULTC);

	val = readl_relaxed(hsspi->regs + DMCFG);
	val &= ~BIT(SSDC);
	val &= ~BIT(MSTARTEN);
	writel_relaxed(val, hsspi->regs + DMCFG);

	val = readl_relaxed(hsspi->regs + MCTRL);
	if (clkid == 0)
		val &= ~BIT(IPCLK);
	else
		val |= BIT(IPCLK);
	val &= ~BIT(CSEN);
	val |= BIT(MEN);
	writel_relaxed(val, hsspi->regs + MCTRL);

	return 0;
}

static int s7x_hsspi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct s7x_hsspi *hsspi;
	struct resource *res;
	int ret, clkid;

	master = spi_alloc_master(&pdev->dev, sizeof(*hsspi));
	if (!master)
		return -ENOMEM;
	platform_set_drvdata(pdev, master);

	hsspi = spi_master_get_devdata(master);
	hsspi->dev = &pdev->dev;
	hsspi->master = master;
	spin_lock_init(&hsspi->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hsspi->regs = devm_ioremap_resource(hsspi->dev, res);
	if (IS_ERR(hsspi->regs)) {
		ret = PTR_ERR(hsspi->regs);
		goto put_spi;
	}

	hsspi->clk = devm_clk_get(hsspi->dev, "iHCLK");
	clkid = 0;
	if (IS_ERR(hsspi->clk)) {
		hsspi->clk = devm_clk_get(hsspi->dev, "iPCLK");
		clkid = 1;
		if (IS_ERR(hsspi->clk)) {
			dev_err(&pdev->dev, "No source clock\n");
			ret = PTR_ERR(hsspi->clk);
			goto put_spi;
		}
	}

	hsspi->aces = 0;
	if (of_find_property(pdev->dev.of_node,"active_clk_edges", NULL))
		hsspi->aces = 1;

	ret = clk_prepare_enable(hsspi->clk);
	if (ret)
		goto put_spi;

	master->auto_runtime_pm = true;
	master->bus_num = pdev->id;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_TX_DUAL | SPI_RX_DUAL |
				SPI_TX_QUAD | SPI_RX_QUAD;
	master->num_chipselect = 4;
	master->dev.of_node = pdev->dev.of_node;
	master->bits_per_word_mask = SPI_BPW_MASK(32) | SPI_BPW_MASK(24)
					 | SPI_BPW_MASK(16) | SPI_BPW_MASK(8);
	master->max_speed_hz = clk_get_rate(hsspi->clk);
	master->min_speed_hz = master->max_speed_hz / 254;

	master->set_cs = s7x_hsspi_set_cs;
	master->transfer_one = s7x_hsspi_transfer_one;

	ret = s7x_hsspi_enable(master);
        if (ret)
                goto fail_enable;

	pm_runtime_set_active(hsspi->dev);
	pm_runtime_enable(hsspi->dev);

	ret = devm_spi_register_master(hsspi->dev, master);
	if (ret)
		goto disable_pm;

	return 0;

disable_pm:
	pm_runtime_disable(hsspi->dev);
fail_enable:
	clk_disable_unprepare(hsspi->clk);
put_spi:
	spi_master_put(master);

	return ret;
}

static int s7x_hsspi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct s7x_hsspi *hsspi = spi_master_get_devdata(master);

	pm_runtime_disable(hsspi->dev);
	clk_disable_unprepare(hsspi->clk);
	spi_master_put(master);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int s7x_hsspi_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);

	return spi_master_suspend(master);
}

static int s7x_hsspi_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct s7x_hsspi *hsspi = spi_master_get_devdata(master);
	int ret;

	/* Ensure reconfigure during next xfer */
	hsspi->speed = 0;

	ret = s7x_hsspi_enable(master);
	if (ret)
		return ret;

	return spi_master_resume(master);
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops s7x_hsspi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(s7x_hsspi_suspend, s7x_hsspi_resume)
};

static const struct of_device_id s7x_hsspi_of_match[] = {
	{ .compatible = "fujitsu,mb86s7x-hsspi", },
	{ },
};
MODULE_DEVICE_TABLE(of, s7x_hsspi_of_match);

static struct platform_driver s7x_hsspi_driver = {
	.driver = {
		.name = "mb86s7x-hsspi",
		.pm = &s7x_hsspi_pm_ops,
		.of_match_table = of_match_ptr(s7x_hsspi_of_match),
	},
	.probe = s7x_hsspi_probe,
	.remove = s7x_hsspi_remove,
};
module_platform_driver(s7x_hsspi_driver);

MODULE_DESCRIPTION("Fujitsu MB86S70 HS-SPI controller driver");
MODULE_AUTHOR("Jassi Brar <jassisinghbrar@gmail.com>");
MODULE_LICENSE("GPL v2");
