// SPDX-License-Identifier: GPL-2.0+
/*
 * Infineon TLV493D-A1B6 3D magnetic sensor
 *
 * Copyright (c) 2016-2017 Andreas Färber
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define MOD1_LOW	BIT(0)
#define MOD1_FAST	BIT(1)
#define MOD1_INT	BIT(2)
#define MOD1_P		BIT(7)

#define MOD2_PT		BIT(5)

struct tlv493d {
	struct i2c_client *i2c;
	struct iio_mount_matrix mount_matrix;
	u8 factset1, factset2, factset3;
};

static int tlv493d_read_regs(struct tlv493d *s, u8 *buf, int len)
{
	int ret;

	ret = i2c_master_recv(s->i2c, buf, len);
	if (ret < len) {
		dev_err(&s->i2c->dev, "failed to read registers (%d)", ret);
		return ret;
	}

	return 0;
}

static unsigned int tlv493d_parity(u32 v)
{
	v ^= v >> 16;
	v ^= v >> 8;
	v ^= v >> 4;
	v &= 0xf;
	return (0x6996 >> v) & 1;
}

#define MOD1_RES_MASK		(0x3 << 3)
#define MOD1_IICADDR_MASK	(0x3 << 5)

static int tlv493d_write_regs(struct tlv493d *s, const u8 *regs, int len)
{
	u8 buf[4];
	int ret;

	if (len != ARRAY_SIZE(buf))
		return -EINVAL;

	buf[0] = 0;
	buf[1] = s->factset1 & (MOD1_IICADDR_MASK | MOD1_RES_MASK);
	buf[1] |= regs[1] & ~(MOD1_P | MOD1_IICADDR_MASK | MOD1_RES_MASK);
	buf[2] = s->factset2;
	buf[3] = MOD2_PT | (s->factset3 & 0x1f);
	buf[3] |= regs[3] & ~(MOD2_PT | 0x1f);

	if ((buf[3] & MOD2_PT) &&
	    !tlv493d_parity((buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3]))
		buf[1] |= MOD1_P;

	ret = i2c_master_send(s->i2c, buf, 4);
	if (ret < 4) {
		dev_err(&s->i2c->dev, "failed to write registers (%d)", ret);
		return ret;
	}

	return 0;
}

static int tlv493d_power_down(struct tlv493d *s)
{
	u8 buf[4];

	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	return tlv493d_write_regs(s, buf, 4);
}

static const struct iio_mount_matrix *
tlv493d_get_mount_matrix(const struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct tlv493d *s = iio_priv(indio_dev);

	return &s->mount_matrix;
}

static const struct iio_chan_spec_ext_info tlv493d_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_DIR, tlv493d_get_mount_matrix),
	{}
};

#define TLV493D_AXIS_CHANNEL(axis, index)			\
	{							\
		.type = IIO_MAGN,				\
		.channel2 = IIO_MOD_##axis,			\
		.modified = 1,					\
		.address = index,				\
		.scan_index = index,				\
		.scan_type = {					\
			.sign = 's',				\
			.realbits = 12,				\
			.storagebits = 16,			\
			.endianness = IIO_CPU,			\
		},						\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
			BIT(IIO_CHAN_INFO_PROCESSED),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
		.ext_info = tlv493d_ext_info,			\
	}

#define TLV493D_TEMP_CHANNEL(index)				\
	{							\
		.type = IIO_TEMP,				\
		.channel2 = IIO_MOD_TEMP_OBJECT,		\
		.modified = 1,					\
		.address = index,				\
		.scan_index = index,				\
		.scan_type = {					\
			.sign = 's',				\
			.realbits = 12,				\
			.storagebits = 16,			\
			.endianness = IIO_CPU,			\
		},						\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
			BIT(IIO_CHAN_INFO_OFFSET) |		\
			BIT(IIO_CHAN_INFO_SCALE) |		\
			BIT(IIO_CHAN_INFO_PROCESSED),		\
	}

static const struct iio_chan_spec tlv493d_channels[] = {
	TLV493D_AXIS_CHANNEL(X, 0),
	TLV493D_AXIS_CHANNEL(Y, 1),
	TLV493D_AXIS_CHANNEL(Z, 2),
	TLV493D_TEMP_CHANNEL(3),
	IIO_CHAN_SOFT_TIMESTAMP(4),
};

static const unsigned long tlv493d_scan_masks[] = { 0xf, 0 };

static int tlv493d_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan,
	int *val, int *val2, long mask)
{
	struct tlv493d *s = iio_priv(indio_dev);
	u8 buf[7];
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
	case IIO_CHAN_INFO_RAW:
		do {
			ret = tlv493d_read_regs(s, buf, 7);
			if (ret)
				return ret;
		} while ((buf[3] & 0x3) || (buf[5] & BIT(4)));

		switch (chan->address) {
		case 0:
			*val = (((int)(s8)buf[0]) << 4) | (buf[4] >> 4);
			break;
		case 1:
			*val = (((int)(s8)buf[1]) << 4) | (buf[4] & 0xf);
			break;
		case 2:
			*val = (((int)(s8)buf[2]) << 4) | (buf[5] & 0xf);
			break;
		case 3:
			*val = (((int)(s8)(buf[3] & 0xf0)) << 4) | buf[6];
			if (mask != IIO_CHAN_INFO_RAW)
				*val -= 340;
			break;
		default:
			return -EINVAL;
		}
		*val2 = 0;
		if (mask == IIO_CHAN_INFO_RAW)
			return IIO_VAL_INT;

		switch (chan->type) {
		case IIO_MAGN:
			*val2 = (*val * 1000000) * 98 / 1000;
			*val = *val2 / 1000000;
			*val2 %= 1000000;
			break;
		case IIO_TEMP:
			*val2 = (*val * 1000000) * 11 / 10;
			*val = *val2 / 1000000;
			*val2 %= 1000000;
			/* According to datasheet, LSB offset is for 25°C */
			*val += 25;
			break;
		default:
			return -EINVAL;
		}
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_TEMP:
			*val = -340;
			*val2 = 0;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_MAGN:
			/* 0.098 mT/LSB */
			*val = 0;
			*val2 = 98000;
			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_TEMP:
			/* 1.1 °C/LSB */
			*val = 1;
			*val2 = 100000;
			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static const struct iio_info tlv493d_iio_info = {
	.read_raw = tlv493d_read_raw,
};

static int tlv493d_probe(struct i2c_client *i2c)
{
	struct iio_dev *indio_dev;
	struct tlv493d *tlv493d;
	u8 buf[10];
	int ret;

	indio_dev = devm_iio_device_alloc(&i2c->dev, sizeof(*tlv493d));
	if (!indio_dev)
		return -ENOMEM;

	tlv493d = iio_priv(indio_dev);
	i2c_set_clientdata(i2c, indio_dev);
	tlv493d->i2c = i2c;

	ret = of_iio_read_mount_matrix(&i2c->dev, "mount-matrix",
		&tlv493d->mount_matrix);
	if (ret)
		return ret;

	indio_dev->dev.parent = &i2c->dev;
	indio_dev->channels = tlv493d_channels;
	indio_dev->num_channels = ARRAY_SIZE(tlv493d_channels);
	indio_dev->info = &tlv493d_iio_info;
	indio_dev->available_scan_masks = tlv493d_scan_masks;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = "tlv493d";

	ret = tlv493d_read_regs(tlv493d, buf, 10);
	if (ret)
		return ret;

	tlv493d->factset1 = buf[7];
	tlv493d->factset2 = buf[8];
	tlv493d->factset3 = buf[9];

	buf[0] = 0;
	buf[1] = MOD1_FAST | MOD1_LOW;
	buf[2] = 0;
	buf[3] = 0;
	ret = tlv493d_write_regs(tlv493d, buf, 4);
	if (ret)
		return ret;

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&i2c->dev, "device registration failed");
		tlv493d_power_down(tlv493d);
		return ret;
	}

	return 0;
}

static int tlv493d_remove(struct i2c_client *i2c)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(i2c);
	struct tlv493d *tlv493d = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	tlv493d_power_down(tlv493d);

	return 0;
}

static const struct of_device_id tlv493d_dt_ids[] = {
	{ .compatible = "infineon,tlv493d-a1b6" },
	{}
};
MODULE_DEVICE_TABLE(of, tlv493d_dt_ids);

static struct i2c_driver tlv493d_i2c_driver = {
	.driver = {
		.name = "tlv493d",
		.of_match_table = tlv493d_dt_ids,
	},
	.probe_new = tlv493d_probe,
	.remove = tlv493d_remove,
};

module_i2c_driver(tlv493d_i2c_driver);

MODULE_DESCRIPTION("TLV493D I2C driver");
MODULE_AUTHOR("Andreas Faerber");
MODULE_LICENSE("GPL");
