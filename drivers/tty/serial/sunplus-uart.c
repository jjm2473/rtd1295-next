// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2020 Andreas Färber
 */

#include <linux/bitops.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>

#define REG_DATA		0x00
#define REG_LSR			0x04

#define PLUS1_UART_LSR_TX	BIT(0)

static inline void plus1_uart_write(struct uart_port *port, unsigned int off, u32 val)
{
	writel_relaxed(val, port->membase + off);
}

static inline u32 plus1_uart_read(struct uart_port *port, unsigned int off)
{
	return readl_relaxed(port->membase + off);
}

#ifdef CONFIG_SERIAL_PLUS1_CONSOLE

static void plus1_console_putchar(struct uart_port *port, int ch)
{
	if (!port->membase)
		return;

	while (!(plus1_uart_read(port, REG_LSR) & PLUS1_UART_LSR_TX))
		cpu_relax();

	plus1_uart_write(port, REG_DATA, ch);
}

static void plus1_uart_port_write(struct uart_port *port, const char *s,
				  u_int count)
{
	unsigned long flags;
	int locked;

	local_irq_save(flags);

	if (port->sysrq)
		locked = 0;
	else if (oops_in_progress)
		locked = spin_trylock(&port->lock);
	else {
		spin_lock(&port->lock);
		locked = 1;
	}

	uart_console_write(port, s, count, plus1_console_putchar);

	if (locked)
		spin_unlock(&port->lock);

	local_irq_restore(flags);
}

static void plus1_uart_early_console_write(struct console *co,
					   const char *s,
					   u_int count)
{
	struct earlycon_device *dev = co->data;

	plus1_uart_port_write(&dev->port, s, count);
}

static int __init
plus1_uart_early_console_setup(struct earlycon_device *device, const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = plus1_uart_early_console_write;

	return 0;
}
OF_EARLYCON_DECLARE(sunplus, "sunplus,sp7021-uart",
		    plus1_uart_early_console_setup);

#endif
