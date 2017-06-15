/*
 * RDA8810PL serial console
 *
 * Copyright RDA Microelectronics Company Limited
 * Copyright (c) 2017 Andreas Färber
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/console.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>

#define RDA_UART_CTRL		0x00
#define RDA_UART_STATUS		0x04
#define RDA_UART_RXTX_BUFFER	0x08
#define RDA_UART_IRQ_MASK	0x0c

#define RDA_UART_STATUS_TX_FIFO_SPACE_MASK	(0x1f << 8)

static inline void rda_uart_write(struct uart_port *port, u32 val, unsigned int off)
{
	writel(val, port->membase + off);
}

static inline u32 rda_uart_read(struct uart_port *port, unsigned int off)
{
	return readl(port->membase + off);
}

#ifdef CONFIG_SERIAL_RDA_CONSOLE

static void rda_console_putchar(struct uart_port *port, int ch)
{
	if (!port->membase)
		return;

	while (!(rda_uart_read(port, RDA_UART_STATUS) & RDA_UART_STATUS_TX_FIFO_SPACE_MASK))
		cpu_relax();

	rda_uart_write(port, ch, RDA_UART_RXTX_BUFFER);
}

static void rda_uart_port_write(struct uart_port *port, const char *s,
				u_int count)
{
	u32 old_irq_mask;
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

	old_irq_mask = rda_uart_read(port, RDA_UART_IRQ_MASK);
	rda_uart_write(port, 0, RDA_UART_IRQ_MASK);

	uart_console_write(port, s, count, rda_console_putchar);

	/* wait until all contents have been sent out */
	while (!(rda_uart_read(port, RDA_UART_STATUS) & RDA_UART_STATUS_TX_FIFO_SPACE_MASK))
		cpu_relax();

	rda_uart_write(port, old_irq_mask, RDA_UART_IRQ_MASK);

	if (locked)
		spin_unlock(&port->lock);

	local_irq_restore(flags);
}

static void rda_uart_early_console_write(struct console *co,
					 const char *s,
					 u_int count)
{
	struct earlycon_device *dev = co->data;

	rda_uart_port_write(&dev->port, s, count);
}

static int __init
rda_uart_early_console_setup(struct earlycon_device *device, const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = rda_uart_early_console_write;

	return 0;
}
OF_EARLYCON_DECLARE(rda, "rda,8810pl-uart",
		    rda_uart_early_console_setup);

#endif /* CONFIG_SERIAL_RDA_CONSOLE */
