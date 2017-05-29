/*
 * Copyright (C) 2015 Socionext Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* Drivers for MITSUBISHI TPCONTROLLERJ4_01 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/serial.h>
#include <linux/tty.h>

#ifdef CONFIG_SERIAL_SC16IS7XX_I2C
#include <linux/of_i2c.h>
#include <linux/i2c.h>
#include <linux/serial_sc16is7xx.h>
#endif

#define AXEMAX_X 4095
#define AXEMAX_Y 4095
#define BAUDRATE 38400

#define MAX_TOUCHES 2

#define REPORT_START 0x1
#define REPORT_STOP 0x2

#define SAMPLE_NORMAL 0x2
#define SAMPLE_SLOW 0x3
#define SAMPLE_MIN SAMPLE_NORMAL
#define SAMPLE_MAX 0x64

#define PARAM_ADJUST 0x1
#define PARAM_SENSITIVITY 0x2
#define ADJUST_NONE 0x0
#define ADJUST_IGNORE_SAMEAXE1 0x1
#define ADJUST_IGNORE_SAMEAXE2 0x2
#define ADJUST_IGNORE_SAMEAXE3 0x3
#define ADJUST_REPORT_SAMEAXE1 0x11
#define ADJUST_REPORT_SAMEAXE2 0x12
#define ADJUST_REPORT_SAMEAXE3 0x13
#define HIGH_SENSITIVITY 0x1
#define LOW_SENSITIVITY 0x2

#define DATALESS_MSG_LEN 3
#define REPORT_ONOFF_MSG_LEN 4
#define SAMPLERATE_REQ_MSG_LEN 4
#define MISC_PARAM_REQ_MSG_LEN 5

#define REPORT_AXE_RECV_DATA_SIZE 0x1
#define SAMPLERATE_RECV_DATA_SIZE 0x1
#define CHG_MODE_RECV_DATA_SIZE 0x1
#define MISC_PARAM_RECV_DATA_SIZE 0x1
#define VERSION_RECV_DATA_SIZE 0x3
#define STATUS_RECV_DATA_SIZE 0xb
#define CALIBRATE_RECV_DATA_SIZE 0x1
#define SW_RESET_RECV_DATA_SIZE 0x1
#define INIT_RECV_DATA_SIZE 0x1
#define AXEINFO_RECV_DATA_SIZE 0x7
#define ERR_RECV_DATA_SIZE 0x3

#define MAX_RECV_DATA_SIZE STATUS_RECV_DATA_SIZE
#define MAX_RECV_MSG_SIZE (MAX_RECV_DATA_SIZE + 2)

#define AXEINFO_MAGIC 0x14

#define DATTR_CALIBRATE 0
#define DATTR_SWRESET 1

enum tpcj4_msgid {
	/* Host -> Controller */
	REPORT_AXES_REQ = 0xa1,
	SET_SAMPLERATE_REQ = 0xa2,
	CHANGE_MODE_REQ = 0xa3,
	SET_MISC_PARAM_REQ = 0xa4,
	GET_VERSION_REQ = 0xa5,
	GET_STATUS_REQ = 0xa7,
	CALIBRATE_REQ = 0xa8,
	SW_RESET_REQ = 0xaf,
	/* Controller -> Host */
	REPORT_AXES_RES = 0xd1,
	SET_SAMPLERATE_RES = 0xd2,
	CHANGE_MODE_RES = 0xd3,
	SET_MISC_PARAM_RES = 0xd4,
	GET_VERSION_RES = 0xd5,
	GET_STATUS_RES = 0xd7,
	CALIBRATE_RES = 0xd8,
	SW_RESET_RES = 0xdf,
	INIT_DONE_REPORT = 0xd9,
	AXESINFO_REPORT = 0xdc,
	ERR_REPORT = 0xda,
};

typedef struct {
	u8 id;
	u8 len;
	u8 body[0];
} __packed tpcj4_msg_t;

struct tpcj4_priv {
	struct platform_device *pdev;
	struct input_dev *input;
	void *bridge;
	int bridge_port;
	u8 sample_rate;
	u8 adjust;
	u8 sensitivity;
	u8 save_buf[MAX_RECV_MSG_SIZE];
	size_t save_len;
	bool init;
};


static u8 tpcj4_calc_cksum(u8 *msg, int len)
{
	int cnt;
	u8 cksum;

	for (cnt = 0, cksum = 0; cnt < len; cnt++) {
		cksum += msg[cnt];
	}

	return (cksum & 0x7f);
}

static void tpcj4_mk_msg(u8 *msg, enum tpcj4_msgid id, const u8 *body, u8 body_len)
{
	tpcj4_msg_t *msg_ptr = (tpcj4_msg_t *)msg;

	msg_ptr->id = (u8)id;
	msg_ptr->len = body_len + 1;

	if (body_len > 0) {
		memcpy(msg_ptr->body, body, (size_t)body_len);
	}

	/* add checksum */
	msg_ptr->body[body_len] = tpcj4_calc_cksum((u8 *)msg_ptr, body_len + 2);
}

static void tpcj4_mk_dataless_msg(u8 *msg, enum tpcj4_msgid id)
{
	tpcj4_mk_msg(msg, id, NULL, 0);
}

static int tpcj4_send_msg(struct tpcj4_priv *priv, u8 *msg, size_t len)
{
	int ret = -EINVAL;

#ifdef CONFIG_SERIAL_SC16IS7XX_I2C
	ret = sc16is7xx_i2c_send_async(
		(struct i2c_client *)priv->bridge, priv->bridge_port, msg, len);
	if (ret == len) {
		ret = 0;
	}
	else {
		dev_err(&priv->pdev->dev, "sc16is7xx_i2c_send_async(%d)\n", ret);
		ret = -EIO;
	}
#endif
	return ret;
}

static int tpcj4_sw_reset(struct tpcj4_priv *priv)
{
	u8 msg[DATALESS_MSG_LEN];

	tpcj4_mk_dataless_msg(msg, SW_RESET_REQ);
	return tpcj4_send_msg(priv, msg, sizeof(msg));
}

static int tpcj4_req_calibrate(struct tpcj4_priv *priv)
{
	u8 msg[DATALESS_MSG_LEN];

	tpcj4_mk_dataless_msg(msg, CALIBRATE_REQ);
	return tpcj4_send_msg(priv, msg, sizeof(msg));
}

static int tpcj4_axe_repot_on_off(struct tpcj4_priv *priv, bool start)
{
	u8 msg[REPORT_ONOFF_MSG_LEN];
	u8 body[1];

	body[0] = start ? REPORT_START : REPORT_STOP;
	tpcj4_mk_msg(msg, REPORT_AXES_REQ, body, (u8)sizeof(body));

	return tpcj4_send_msg(priv, msg, sizeof(msg));
}

static int tpcj4_set_sample_rate(struct tpcj4_priv *priv, u8 sample_rate)
{
	u8 msg[SAMPLERATE_REQ_MSG_LEN];
	u8 body[1];

	body[0] = sample_rate;
	tpcj4_mk_msg(msg, SET_SAMPLERATE_REQ, body, (u8)sizeof(body));

	return tpcj4_send_msg(priv, msg, sizeof(msg));
}

static int tpcj4_set_misc_param(struct tpcj4_priv *priv, u8 kind, u8 val)
{
	u8 msg[MISC_PARAM_REQ_MSG_LEN];
	u8 body[2];

	body[0] = kind;
	body[1] = val;
	tpcj4_mk_msg(msg, SET_MISC_PARAM_REQ, body, (u8)sizeof(body));

	return tpcj4_send_msg(priv, msg, sizeof(msg));
}

static int tpcj4_stub_handler(const tpcj4_msg_t *msg_ptr, struct tpcj4_priv *priv)
{
	int ret = msg_ptr->len + 2; /* message body + header length */

	dev_dbg(&priv->pdev->dev, "%s: msg id:%#x\n", __func__, msg_ptr->id);

	return ret;
}

static int tpcj4_handle_errinfo(const tpcj4_msg_t *msg_ptr, struct tpcj4_priv *priv)
{
	int ret = msg_ptr->len + 2; /* message body + header length */

	dev_warn_ratelimited(&priv->pdev->dev,
			"error is reported - val1:%#x, val2:%#x\n", msg_ptr->body[0], msg_ptr->body[1]);

	return ret;
}

static int tpcj4_handle_init_done(const tpcj4_msg_t *msg_ptr, struct tpcj4_priv *priv)
{
	int ret = msg_ptr->len + 2; /* message body + header length */

	if (!priv->init) {
		int ret2;

		ret2 = tpcj4_set_sample_rate(priv, priv->sample_rate);
		if (ret2) {
			dev_warn(&priv->pdev->dev, "set sample rate failed\n");

			/* working with default settings  */
		}

		ret2 = tpcj4_set_misc_param(priv, PARAM_ADJUST, priv->adjust);
		if (ret2) {
			dev_warn(&priv->pdev->dev, "set parameter(%d) failed\n", PARAM_ADJUST);

			/* working with default settings  */
		}

		ret2 = tpcj4_set_misc_param(priv, PARAM_SENSITIVITY, priv->sensitivity);
		if (ret2) {
			dev_warn(&priv->pdev->dev, "set parameter(%d) failed\n", PARAM_SENSITIVITY);

			/* working with default settings  */
		}


		priv->init = true;
	}

	return ret;
}

static int tpcj4_handle_axeinfo(const tpcj4_msg_t *msg_ptr, struct tpcj4_priv *priv)
{
	int ret = msg_ptr->len + 2; /* message body + header length */
	struct input_dev *input = priv->input;
	const u8 *msg = msg_ptr->body;
	u8 id, touch;
	u16 axe_x, axe_y;

	if (msg[0] != AXEINFO_MAGIC) {
		dev_warn_ratelimited(&priv->pdev->dev, "%s: invalid magic number:%#x\n", __func__, msg[0]);
		ret = -EINVAL;
		goto end;
	}

	if (!(msg[1] & 0x80)) {
		/* valid flag is cleard(normal case) */
		goto end;
	}

	id = (msg[1] >> 2) & 0xf;
	touch = msg[1] & 0x1;

	input_mt_slot(input, (int)id);
	input_mt_report_slot_state(input, MT_TOOL_FINGER, (touch != 0));

	if (touch) {
		axe_x = (u16)((u16)msg[2] << 6) | msg[3];
		axe_y = (u16)((u16)msg[4] << 6) | msg[5];

		input_report_abs(input, ABS_MT_POSITION_X, (int)axe_x);
		input_report_abs(input, ABS_MT_POSITION_Y, (int)axe_y);
	}

	input_mt_report_pointer_emulation(input, false);
	input_sync(input);

end:
	return ret;
}

static int tpcj4_handle_msg_main(const u8 *msg, size_t len, struct tpcj4_priv *priv)
{
	tpcj4_msg_t *msg_ptr;
	u8 merge_buf[MAX_RECV_MSG_SIZE];
	u8 msg_len, saved_msg_len, old_save_len, chk_len;
	u8 cksum;
	int ret;
	int (*func)(const tpcj4_msg_t *msg_ptr, struct tpcj4_priv *priv);

	old_save_len = 0;
	msg_len = len > MAX_RECV_MSG_SIZE ? MAX_RECV_MSG_SIZE : (u8)len;
	saved_msg_len = msg_len;
	if (priv->save_len > 0) {
		u8 empty = MAX_RECV_MSG_SIZE - priv->save_len;
		u8 copy_len = empty > msg_len ? msg_len : empty;

		memcpy(merge_buf, priv->save_buf, priv->save_len);
		memcpy(&merge_buf[priv->save_len], msg, copy_len);
		msg_ptr = (tpcj4_msg_t *)merge_buf;
		msg_len = priv->save_len + copy_len;
		old_save_len = priv->save_len;
		priv->save_len = 0;
	}
	else {
		msg_ptr = (tpcj4_msg_t *)msg;
	}

	if (msg_len < 2) {
		goto save;
	}

	/* basic verification */
	if ((msg_ptr->id & 0xf0) != 0xd0) {
		dev_warn_ratelimited(&priv->pdev->dev, "invalid message ID:%#x\n", msg_ptr->id);
		ret = -EINVAL;
		goto end;
	}
	else if (msg_ptr->len > MAX_RECV_DATA_SIZE || msg_ptr->len <= 0) {
		dev_warn_ratelimited(&priv->pdev->dev, "invalid message length:%d\n", (int)msg_ptr->len);
		ret = -EINVAL;
		goto end;
	}

	switch (msg_ptr->id) {
	case REPORT_AXES_RES:
		func = tpcj4_stub_handler;
		chk_len = REPORT_AXE_RECV_DATA_SIZE;
		break;

	case SET_SAMPLERATE_RES:
		func = tpcj4_stub_handler;
		chk_len = SAMPLERATE_RECV_DATA_SIZE;
		break;

	case CHANGE_MODE_RES:
		func = tpcj4_stub_handler;
		chk_len = CHG_MODE_RECV_DATA_SIZE;
		break;

	case SET_MISC_PARAM_RES:
		func = tpcj4_stub_handler;
		chk_len = MISC_PARAM_RECV_DATA_SIZE;
		break;

	case GET_VERSION_RES:
		func = tpcj4_stub_handler;
		chk_len = VERSION_RECV_DATA_SIZE;
		break;

	case GET_STATUS_RES:
		func = tpcj4_stub_handler;
		chk_len = STATUS_RECV_DATA_SIZE;
		break;

	case CALIBRATE_RES:
		func = tpcj4_stub_handler;
		chk_len = CALIBRATE_RECV_DATA_SIZE;
		break;

	case SW_RESET_RES:
		func = tpcj4_stub_handler;
		chk_len = SW_RESET_RECV_DATA_SIZE;
		break;

	case INIT_DONE_REPORT:
		func = tpcj4_handle_init_done;
		chk_len = INIT_RECV_DATA_SIZE;
		break;

	case AXESINFO_REPORT:
		func = tpcj4_handle_axeinfo;
		chk_len = AXEINFO_RECV_DATA_SIZE;
		break;

	case ERR_REPORT:
		func = tpcj4_handle_errinfo;
		chk_len = ERR_RECV_DATA_SIZE;
		break;

	default:
		dev_warn_ratelimited(&priv->pdev->dev, "invalid message ID:%#x\n", msg_ptr->id);
		ret = -EINVAL;
		goto end;
	}

	if (msg_ptr->len != chk_len) {
		dev_warn_ratelimited(&priv->pdev->dev, "invalid message length:%d\n", (int)msg_ptr->len);
		ret = -EINVAL;
		goto end;
	}
	else if (msg_ptr->len + 2 > msg_len) {
		goto save;
	}

	cksum = tpcj4_calc_cksum((u8 *)msg_ptr, msg_ptr->len + 1);
	if (msg_ptr->body[msg_ptr->len - 1] != cksum) {
		dev_warn_ratelimited(&priv->pdev->dev, "checksum error\n");
		ret = -EINVAL;
		goto end;
	}

	ret = func(msg_ptr, priv);
	if (old_save_len > 0) {
		WARN_ON(ret <= old_save_len); /* bug */
		ret -= (int)old_save_len;
	}

end:
	return ret;

save:
	memcpy(&priv->save_buf[priv->save_len], msg_ptr, msg_len);
	priv->save_len += msg_len;

	return (int)saved_msg_len;
}

static void tpcj4_push_data_cb(const void *data, size_t len, void *cb_arg)
{
	struct tpcj4_priv *priv = cb_arg;
	const u8 *msg = data;
	int idx = 0, ret;

	do {
		ret = tpcj4_handle_msg_main(&msg[idx], len, priv);
		if (ret <= 0) {
			break;
		}

		len -= (size_t)ret;
		idx += ret;
	} while(len > 0);
}

static void *tpcj4_get_bridge_dev(struct platform_device *pdev)
{
#ifdef CONFIG_SERIAL_SC16IS7XX_I2C
	struct device_node *bridge_node;
	struct i2c_client *bridge_dev;

	bridge_node = of_parse_phandle(pdev->dev.of_node, "bridge", 0);
	if (!bridge_node) {
		dev_dbg(&pdev->dev, "UART Bridge is not found on this target\n");
		return NULL;
	}

	bridge_dev = of_find_i2c_device_by_node(bridge_node);
	if (!bridge_dev) {
		dev_info(&pdev->dev, "failed to find bridge i2c device(maybe not probed yet)\n");
		bridge_dev = ERR_PTR(-EPROBE_DEFER);
	}

	of_node_put(bridge_node);
	return (void *)bridge_dev;
#else
	return NULL;
#endif
}

static void tpcj4_put_bridge_dev(void *data)
{
#ifdef CONFIG_SERIAL_SC16IS7XX_I2C
	struct i2c_client *bridge_dev = (struct i2c_client *)data;

	if (bridge_dev)
		put_device(&bridge_dev->dev);
#endif
}

static ssize_t tpcj4_dattr_store_common(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count,
				int command)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tpcj4_priv *priv = platform_get_drvdata(pdev);
	unsigned long exec;
	int ret;

	if (kstrtoul(buf, 0, &exec))
		return -EINVAL;

	if (exec > 1)
		return -EINVAL;

	if (!exec) {
		goto end;
	}

	switch (command) {
	case DATTR_CALIBRATE:
		ret = tpcj4_req_calibrate(priv);
		if (ret) {
			return -EIO;
		}

		break;

	case DATTR_SWRESET:
		priv->init = false;
		ret = tpcj4_sw_reset(priv);
		if (ret) {
			return -EIO;
		}

		break;
	}

end:
	return count;
}

static ssize_t tpcj4_dattr_calibrate(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	return tpcj4_dattr_store_common(dev, attr, buf, count, DATTR_CALIBRATE);
}

static ssize_t tpcj4_dattr_reset(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	return tpcj4_dattr_store_common(dev, attr, buf, count, DATTR_SWRESET);
}


static DEVICE_ATTR(calibrate, 0644, NULL, tpcj4_dattr_calibrate);
static DEVICE_ATTR(reset, 0644, NULL, tpcj4_dattr_reset);

static struct attribute *tpcj4_attributes[] = {
	&dev_attr_calibrate.attr,
	&dev_attr_reset.attr,
	NULL,
};

static const struct attribute_group tpcj4_attr_group = {
	.attrs = tpcj4_attributes,
};


static int tpcj4_probe(struct platform_device *pdev)
{
	struct tpcj4_priv *priv;
	struct input_dev *input;
	void *bridge_dev;
	struct ktermios termios;
	u32 port, sample, adjust, sensitivity;
	int ret;
#ifdef CONFIG_SERIAL_SC16IS7XX_I2C
	struct i2c_client *client;
#endif

	dev_info(&pdev->dev, "%s start.\n", __func__);

	bridge_dev = tpcj4_get_bridge_dev(pdev);
	if (IS_ERR(bridge_dev)) {
		return PTR_ERR(bridge_dev);
	}
	else if (!bridge_dev) {
		/* we currently support only I/O via UART bridge */
		dev_warn(&pdev->dev, "UART Bridge is not found\n");
		return -EINVAL;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "could not allocate private data\n");
		ret = -ENOMEM;
		goto put_bridge;
	}

	input = devm_input_allocate_device(&pdev->dev);
	if (!input) {
		dev_err(&pdev->dev, "could not allocate input device\n");
		ret = -ENOMEM;
		goto put_bridge;
	}

	input->name = "MITSUBISHI TPCONTROLLERJ4_01";
	input->dev.parent = &pdev->dev;

	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(INPUT_PROP_DIRECT, input->propbit);

	input_set_abs_params(input, ABS_X, 0, AXEMAX_X, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, AXEMAX_Y, 0, 0);

	input_mt_init_slots(input, MAX_TOUCHES, INPUT_MT_DIRECT);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, AXEMAX_X, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, AXEMAX_Y, 0, 0);

	ret = input_register_device(input);
	if (ret) {
		dev_err(&pdev->dev, "could not register input device\n");
		goto free_input_dev;
	}

	if (of_property_read_u32(pdev->dev.of_node, "port", &port)) {
		port = 0;
	}

	if (of_property_read_u32(pdev->dev.of_node, "sample-rate", &sample)) {
		sample = SAMPLE_SLOW;
	}
	else if (sample < SAMPLE_MIN || sample > SAMPLE_MAX) {
		dev_info(&pdev->dev, "invalid property:%s(%u), use default value\n", "sample-rate", sample);
		sample = SAMPLE_SLOW;
	}

	if (of_property_read_u32(pdev->dev.of_node, "adjust", &adjust)) {
		adjust = ADJUST_REPORT_SAMEAXE1;
	}
	else if (adjust != ADJUST_NONE &&
			adjust != ADJUST_IGNORE_SAMEAXE1 &&
			adjust != ADJUST_IGNORE_SAMEAXE2 &&
			adjust != ADJUST_IGNORE_SAMEAXE3 &&
			adjust != ADJUST_REPORT_SAMEAXE1 &&
			adjust != ADJUST_REPORT_SAMEAXE2 &&
			adjust != ADJUST_REPORT_SAMEAXE3) {

		dev_info(&pdev->dev, "invalid property:%s(%u), use default value\n", "adjust", adjust);
		adjust = ADJUST_REPORT_SAMEAXE1;
	}

	if (of_property_read_u32(pdev->dev.of_node, "sensitivity", &sensitivity)) {
		sensitivity = HIGH_SENSITIVITY;
	}
	else if (sensitivity != HIGH_SENSITIVITY && sensitivity != LOW_SENSITIVITY) {
		dev_info(&pdev->dev, "invalid property:%s(%u), use default value\n", "sensitivity", sensitivity);
		sensitivity = HIGH_SENSITIVITY;
	}

	priv->pdev = pdev;
	priv->input = input;
	priv->bridge = bridge_dev;
	priv->bridge_port = (int)port;
	priv->sample_rate = (u8)sample;
	priv->adjust = (u8)adjust;
	priv->sensitivity = (u8)sensitivity;
	priv->save_len = 0;
	priv->init = false;

	ret = sysfs_create_group(&pdev->dev.kobj, &tpcj4_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "unable to create sysfs attributes(%d)\n", ret);
		goto unreg_input_dev;
	}

#ifdef CONFIG_SERIAL_SC16IS7XX_I2C
	memset(&termios, 0, sizeof(termios));
	termios.c_cflag = CS8;
	termios.c_ispeed = BAUDRATE;
	termios.c_ospeed = BAUDRATE;

	client = bridge_dev;
	ret = sc16is7xx_startup_not_tty(&client->dev, priv->bridge_port, &termios, tpcj4_push_data_cb, priv);
	if (ret) {
		dev_err(&pdev->dev, "could not start UART Bridge(%d)\n", ret);
		goto remove_sysfs;
	}

	ret = tpcj4_sw_reset(priv);
	if (ret) {
		dev_err(&pdev->dev, "could not reset UART Bridge\n");
		goto sdown_bridge;
	}
#else
#error "You must enable UART Bridge Driver(e.g. CONFIG_SERIAL_SC16IS7XX_I2C)"
#endif

	input_set_drvdata(input, priv);
	platform_set_drvdata(pdev, priv);

	dev_info(&pdev->dev, "%s done.\n", __func__);

	return 0;

#ifdef CONFIG_SERIAL_SC16IS7XX_I2C
sdown_bridge:
	sc16is7xx_shutdown_not_tty(&client->dev, priv->bridge_port);

remove_sysfs:
	sysfs_remove_group(&pdev->dev.kobj, &tpcj4_attr_group);

#endif
unreg_input_dev:
	input_unregister_device(input);
	input = NULL;

free_input_dev:
	if (input) {
		input_free_device(input);
	}

put_bridge:
	tpcj4_put_bridge_dev(bridge_dev);

	dev_info(&pdev->dev, "%s falied.\n", __func__);

	return ret;
}

static int tpcj4_remove(struct platform_device *pdev)
{
	struct tpcj4_priv *priv = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &tpcj4_attr_group);
#ifdef CONFIG_SERIAL_SC16IS7XX_I2C
	sc16is7xx_shutdown_not_tty(&pdev->dev, priv->bridge_port);
#endif
	tpcj4_put_bridge_dev(priv->bridge);
	input_unregister_device(priv->input);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tpcj4_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tpcj4_priv *priv = platform_get_drvdata(pdev);

	tpcj4_axe_repot_on_off(priv, false);

	return 0;
}

static int tpcj4_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tpcj4_priv *priv = platform_get_drvdata(pdev);

	tpcj4_axe_repot_on_off(priv, true);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(tpcj4_pm_ops,
			 tpcj4_suspend, tpcj4_resume);

static const struct of_device_id tpcj4_dt_ids[] = {
	{ .compatible = "mitsubishi,tpcj4" },
	{ /* sentinel */ }
};

static struct platform_driver tpcj4_driver = {
	.probe = tpcj4_probe,
	.remove = tpcj4_remove,
	.driver = {
		.name = "tpcj4",
		.of_match_table = tpcj4_dt_ids,
		.pm = &tpcj4_pm_ops,
	},
};

MODULE_DEVICE_TABLE(of, tpcj4_dt_ids);

static int __init tpcj4_init(void)
{
	return platform_driver_register(&tpcj4_driver);
}

static void __exit tpcj4_exit(void)
{
	platform_driver_unregister(&tpcj4_driver);
}

module_init(tpcj4_init);
module_exit(tpcj4_exit);

MODULE_LICENSE("GPL v2");
