/*
 * XMC4500 PORTS
 *
 * Copyright 2015-2016 Andreas Färber
 *
 * License: GPL-2.0+
 */

#include "pinctrl-xmc4000.h"

#define XMC4500_NUM_PORTS (7 + 2)
#define XMC4500_NUM_PINS (XMC4500_NUM_PORTS * 16)

static const char * const xmc4500_pin_names[XMC4500_NUM_PINS] = {
	XMC4000_PORT_PIN_NAMES(0),
	XMC4000_PORT_PIN_NAMES(1),
	XMC4000_PORT_PIN_NAMES(2),
	XMC4000_PORT_PIN_NAMES(3),
	XMC4000_PORT_PIN_NAMES(4),
	XMC4000_PORT_PIN_NAMES(5),
	XMC4000_PORT_PIN_NAMES(6),
	XMC4000_PORT_PIN_NAMES(14),
	XMC4000_PORT_PIN_NAMES(15),
};

#define XMC4500_PORT_GET_PIN(port, pin) \
	((((port) > 6) ? ((port) - 7) : (port)) * 16 + (pin))

#define XMC4500_PIN(port, pin) \
	{ \
		.pins = (unsigned int []) { XMC4500_PORT_GET_PIN(port, pin) }, \
	}

#define XMC4500_PORT_PINS(port) \
	XMC4500_PIN(port, 0), \
	XMC4500_PIN(port, 1), \
	XMC4500_PIN(port, 2), \
	XMC4500_PIN(port, 3), \
	XMC4500_PIN(port, 4), \
	XMC4500_PIN(port, 5), \
	XMC4500_PIN(port, 6), \
	XMC4500_PIN(port, 7), \
	XMC4500_PIN(port, 8), \
	XMC4500_PIN(port, 9), \
	XMC4500_PIN(port, 10), \
	XMC4500_PIN(port, 11), \
	XMC4500_PIN(port, 12), \
	XMC4500_PIN(port, 13), \
	XMC4500_PIN(port, 14), \
	XMC4500_PIN(port, 15)

static const struct xmc4000_pin xmc4500_pins[XMC4500_NUM_PINS] = {
	XMC4500_PORT_PINS(0),
	XMC4500_PORT_PINS(1),
	XMC4500_PORT_PINS(2),
	XMC4500_PORT_PINS(3),
	XMC4500_PORT_PINS(4),
	XMC4500_PORT_PINS(5),
	XMC4500_PORT_PINS(6),
	XMC4500_PORT_PINS(14),
	XMC4500_PORT_PINS(15),
};

#define XMC4500_PIN_DESC(port, pin) \
	PINCTRL_PIN(XMC4500_PORT_GET_PIN(port, pin), XMC4000_PIN_NAME(port, pin))

#define XMC4500_PORT_PIN_DESC(port) \
	XMC4500_PIN_DESC(port, 0), \
	XMC4500_PIN_DESC(port, 1), \
	XMC4500_PIN_DESC(port, 2), \
	XMC4500_PIN_DESC(port, 3), \
	XMC4500_PIN_DESC(port, 4), \
	XMC4500_PIN_DESC(port, 5), \
	XMC4500_PIN_DESC(port, 6), \
	XMC4500_PIN_DESC(port, 7), \
	XMC4500_PIN_DESC(port, 8), \
	XMC4500_PIN_DESC(port, 9), \
	XMC4500_PIN_DESC(port, 10), \
	XMC4500_PIN_DESC(port, 11), \
	XMC4500_PIN_DESC(port, 12), \
	XMC4500_PIN_DESC(port, 13), \
	XMC4500_PIN_DESC(port, 14), \
	XMC4500_PIN_DESC(port, 15)

static const struct pinctrl_pin_desc xmc4500_pin_desc[XMC4500_NUM_PINS] = {
	XMC4500_PORT_PIN_DESC(0),
	XMC4500_PORT_PIN_DESC(1),
	XMC4500_PORT_PIN_DESC(2),
	XMC4500_PORT_PIN_DESC(3),
	XMC4500_PORT_PIN_DESC(4),
	XMC4500_PORT_PIN_DESC(5),
	XMC4500_PORT_PIN_DESC(6),
	XMC4500_PORT_PIN_DESC(14),
	XMC4500_PORT_PIN_DESC(15),
};

#define XMC4500_PORT(port) \
	{ \
		.nr = port, \
	}

static const struct xmc4000_port xmc4500_ports[XMC4500_NUM_PORTS] = {
	XMC4500_PORT(0),
	XMC4500_PORT(1),
	XMC4500_PORT(2),
	XMC4500_PORT(3),
	XMC4500_PORT(4),
	XMC4500_PORT(5),
	XMC4500_PORT(6),
	XMC4500_PORT(14),
	XMC4500_PORT(15),
};

static const struct xmc4000_ports_data xmc4500_ports_data = {
	.name = "xm4500-pinctrl",
	.ports = xmc4500_ports,
	.nports = XMC4500_NUM_PORTS,
	.pin_names = xmc4500_pin_names,
	.pin_desc = xmc4500_pin_desc,
	.pins = xmc4500_pins,
	.npins = XMC4500_NUM_PINS,
};

static const struct of_device_id xmc4500_ports_of_match[] = {
	{
		.compatible = "infineon,xmc4500-ports",
		.data = &xmc4500_ports_data,
	},
	{ }
};

static struct platform_driver xmc4500_ports_driver = {
	.probe = xmc4000_ports_probe,
	.driver = {
		.name = "xmc4500-ports",
		.owner = THIS_MODULE,
		.of_match_table = xmc4500_ports_of_match,
	},
};

static int __init xmc4500_ports_init(void)
{
	return platform_driver_register(&xmc4500_ports_driver);
}
arch_initcall(xmc4500_ports_init);
