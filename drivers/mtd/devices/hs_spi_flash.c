/*
 * MTD SPI driver for HS SPI Controller
 *
 * Copyright (C) 2010-2012 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/io.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#include <linux/platform_data/dma-mb8ac0300-xdmac.h>
#include <linux/platform_data/mb8ac0300-hs_spi.h>

/*
 * MTD driver for HS SPI Controller operation in command sequencer mode
 */
#define	DRV_NAME		"hsspi_cs"

struct command_sequencer {
	struct spi_device	*spi;
	struct mutex		lock;
	struct mtd_info		mtd;
	void __iomem		*base;

};

static inline struct command_sequencer *mtd_to_cs(struct mtd_info *mtd)
{
	return container_of(mtd, struct command_sequencer, mtd);
}

/*
 * MTD implementation
 */

/*
 * hs_spi_flash_read - Read data from Serial Flash Area
 * @mtd:	Mtd device data.
 * @from:	Start address to read.
 * @len:	Data length to read.
 * @retlen:	Actually read length.
 * @buf:	Buffer for read data.
 *
 * Returns 0 on success; negative errno on failure
 */
static int hs_spi_flash_read(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, unsigned char *buf)
{
	struct command_sequencer	*cs = mtd_to_cs(mtd);
	unsigned int		addr_virt = (unsigned int)cs->base
							 + (unsigned int)from;

	mutex_lock(&cs->lock);
	memcpy(buf, (const void *)addr_virt, len);
	*retlen = len;
	mutex_unlock(&cs->lock);

	return 0;
}

/*
 * hs_spi_flash_write - Write data to Serial Flash Area
 * @mtd:	Mtd device data.
 * @to:		Start address to write.
 * @len:	Data length to write.
 * @retlen:	Actually write length.
 * @buf:	Buffer for write data.
 *
 * Error code will be returned because write operation is not supported
 * by HS SPI Driver in command sequencer mode
 *
 * Returns negative errno on failure
 */
static int hs_spi_flash_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
	struct command_sequencer	*cs = mtd_to_cs(mtd);

	dev_err(&cs->spi->dev, "write operation is not supported by HS SPI Driver in command sequencer mode\n");
	return -EINVAL;
}

/*
 * hs_spi_flash_erase - Erase the data of Serial Flash Area
 * @mtd:	Mtd device data.
 * @instr:	Erase information.
 *
 * Error code will be returned because erase operation is not supported
 * by HS SPI Driver in command sequencer mode
 *
 * Returns negative errno on failure
 */
static int hs_spi_flash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct command_sequencer	*cs = mtd_to_cs(mtd);

	dev_err(&cs->spi->dev, "erase operation is not supported by HS SPI Driver in command sequencer mode\n");
	return -EINVAL;
}

/****************************************************************************/
/*
 * hs_spi_probe - probe the flash driver for HS SPI command sequencer mode
 * @spi:	Spi device data.
 *
 * Returns 0 on success; negative errno on failure
 */
static int hs_spi_flash_probe(struct spi_device *spi)
{
	struct hs_spi			*hs;
	struct command_sequencer	*cs;
	struct flash_platform_data	*data;
	struct mtd_partition		*parts = NULL;
	int				nr_parts = 0;
	hs = spi_master_get_devdata(spi->master);

	if (!hs) {
		pr_err("%s: No hs spi driver data\n",
			dev_name(&spi->dev));
		return -ENODEV;
	}

	data = spi->dev.platform_data;

	if (!data || !data->type) {
		pr_err("%s: No flash type designated\n",
			dev_name(&spi->dev));
		return -ENODEV;
	}

	cs = kzalloc(sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return -ENOMEM;

	cs->base	= hs->csa;
	cs->spi		= spi;
	mutex_init(&cs->lock);
	dev_set_drvdata(&spi->dev, cs);
	cs->mtd.name	= data->name;

	/* setup mtd information */
	cs->mtd.type		= MTD_NORFLASH;
	cs->mtd.writesize	= 1;
	cs->mtd.flags		= MTD_CAP_NORFLASH;
	cs->mtd.size		= hs->csa_size;
	/*
	 * only support read operation in command sequencer mode.
	 * error code will be returned when write or erase.
	 */
	cs->mtd._read		= hs_spi_flash_read;
	cs->mtd._write		= hs_spi_flash_write;
	cs->mtd._erase		= hs_spi_flash_erase;

	cs->mtd.erasesize	= SZ_64K; /* do not support erase */

	cs->mtd.dev.parent = &spi->dev;

	parts = data->parts;
	nr_parts = data->nr_parts;

	return mtd_device_register(&cs->mtd, parts, nr_parts) == 0 ?
		0 : -ENODEV;
}

/*
 * hs_spi_flash_remove - remove the flash driver
 * @spi:	Spi device data.
 *
 * Returns 0 on success
 */
static int hs_spi_flash_remove(struct spi_device *spi)
{
	struct command_sequencer	*cs = dev_get_drvdata(&spi->dev);
	int				status;

	/* Clean up MTD stuff. */
	status = mtd_device_unregister(&cs->mtd);
	if (status == 0)
		kfree(cs);
	return 0;
}

static struct spi_driver hs_spi_flash_driver = {
	.driver = {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= hs_spi_flash_probe,
	.remove	= hs_spi_flash_remove,
};

/*
 * hs_spi_init - initialize module
 *
 * Returns 0 on success; negative errno on failure
 */
static int __init hs_spi_flash_init(void)
{
	return spi_register_driver(&hs_spi_flash_driver);
}

/*
 * hs_spi_init - exit module
 */
static void __exit hs_spi_flash_exit(void)
{
	spi_unregister_driver(&hs_spi_flash_driver);
}


module_init(hs_spi_flash_init);
module_exit(hs_spi_flash_exit);

MODULE_DESCRIPTION("MTD SPI driver for HS SPI Controller");
MODULE_AUTHOR("Fujitsu Semiconductor Limitd");
MODULE_LICENSE("GPL");
