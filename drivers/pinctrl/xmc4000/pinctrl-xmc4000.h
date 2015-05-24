/*
 * XMC4x00 PORTS
 *
 * Copyright 2015-2016 Andreas Färber
 *
 * License: GPL-2.0+
 */
#ifndef __PINCTRL_XMC4000_H
#define __PINCTRL_XMC4000_H

#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_device.h>

#define XMC4000_PIN_NAME(port, pin) \
	"P" __stringify(port) "." __stringify(pin)

#define XMC4000_PORT_PIN_NAMES(port) \
	XMC4000_PIN_NAME(port, 0), \
	XMC4000_PIN_NAME(port, 1), \
	XMC4000_PIN_NAME(port, 2), \
	XMC4000_PIN_NAME(port, 3), \
	XMC4000_PIN_NAME(port, 4), \
	XMC4000_PIN_NAME(port, 5), \
	XMC4000_PIN_NAME(port, 6), \
	XMC4000_PIN_NAME(port, 7), \
	XMC4000_PIN_NAME(port, 8), \
	XMC4000_PIN_NAME(port, 9), \
	XMC4000_PIN_NAME(port, 10), \
	XMC4000_PIN_NAME(port, 11), \
	XMC4000_PIN_NAME(port, 12), \
	XMC4000_PIN_NAME(port, 13), \
	XMC4000_PIN_NAME(port, 14), \
	XMC4000_PIN_NAME(port, 15)

struct xmc4000_pin {
	const unsigned int *pins;
};

struct xmc4000_port {
	unsigned int nr;
};

struct xmc4000_ports_data {
	const char *name;
	const struct xmc4000_port *ports;
	unsigned int nports;
	const char * const *pin_names;
	const struct pinctrl_pin_desc *pin_desc;
	const struct xmc4000_pin *pins;
	unsigned int npins;
};

int xmc4000_ports_probe(struct platform_device *pdev);

#endif
