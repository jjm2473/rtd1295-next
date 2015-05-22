/*
 * XMC4000 USIC
 */

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/xmc4000-usic.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/slab.h>

#define XMC4500_USICx_CHy_TRBSR	0x114
#define XMC4500_USICx_CHy_TRBSR_TEMPTY	BIT(11)
#define XMC4500_USICx_CHy_TRBSR_TFULL	BIT(12)

#define XMC4500_USICx_CHy_IN0	0x180

#define DRIVER_NAME "xmc4000-usic-channel-asc"
#define XMC4000_UART_DEVICE "ttyXMC"
#define XMC4000_MAX_USIC_NR 1

#define to_xmc_port(_port) container_of(_port, struct xmc4000_uart_port, port)

struct xmc4000_uart_port {
	struct uart_port port;
	struct clk *clk;
};

static struct xmc4000_uart_port *xmc4000_uart_ports[XMC4000_MAX_USIC_NR];

static struct xmc4000_uart_port *xmc4000_uart_get(int index)
{
	if (index >= 0 && index < XMC4000_MAX_USIC_NR)
		return xmc4000_uart_ports[index];

	return NULL;
}

static void xmc4000_usic_putchar(struct xmc4000_uart_port *xmc_port, char ch)
{
	while ((readl(xmc_port->port.membase + XMC4500_USICx_CHy_TRBSR) & XMC4500_USICx_CHy_TRBSR_TFULL));

	writel_relaxed(ch, xmc_port->port.membase + XMC4500_USICx_CHy_IN0);
}

static unsigned int xmc4000_usic_tx_empty(struct uart_port *port)
{
	struct xmc4000_uart_port *xmc_port = to_xmc_port(port);
	u32 trbsr;

	trbsr = readl_relaxed(xmc_port->port.membase + XMC4500_USICx_CHy_TRBSR);

	return trbsr & XMC4500_USICx_CHy_TRBSR_TEMPTY ? TIOCSER_TEMT : 0;
}

static void xmc4000_usic_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int xmc4000_usic_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void xmc4000_usic_stop_tx(struct uart_port *port)
{
}

static void xmc4000_usic_start_tx(struct uart_port *port)
{
	struct xmc4000_uart_port *xmc_port = to_xmc_port(port);
	struct circ_buf *xmit;

	if (port->x_char) {
		xmc4000_usic_putchar(xmc_port, port->x_char);
		port->x_char = 0;
		port->icount.tx++;
		return;
	}

	if (uart_tx_stopped(port)) {
		xmc4000_usic_stop_tx(port);
		return;
	}

	xmit = &port->state->xmit;
	if (uart_circ_empty(xmit)) {
		xmc4000_usic_stop_tx(port);
		return;
	}
	xmc4000_usic_putchar(xmc_port, xmit->buf[xmit->tail]);
	xmit->tail = (xmit->tail + 1) % UART_XMIT_SIZE;
	port->icount.tx++;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		xmc4000_usic_stop_tx(port);
}

static void xmc4000_usic_stop_rx(struct uart_port *port)
{
}

static void xmc4000_usic_break_ctl(struct uart_port *port, int ctl)
{
}

static int xmc4000_usic_startup(struct uart_port *port)
{
	struct xmc4000_uart_port *xmc_port = to_xmc_port(port);
	int ret;

	ret = clk_enable(xmc_port->clk);
	if (ret) {
		dev_err(xmc_port->port.dev, "enabling clock failed\n");
		goto err_clk_enable;
	}

	return 0;

err_clk_enable:
	return ret;
}

static void xmc4000_usic_shutdown(struct uart_port *port)
{
	struct xmc4000_uart_port *xmc_port = to_xmc_port(port);

	clk_disable(xmc_port->clk);
}

static void xmc4000_usic_set_termios(struct uart_port *port,
	struct ktermios *new, struct ktermios *old)
{
}

static const char *xmc4000_usic_type(struct uart_port *port)
{
	return port->type == PORT_XMC4000_USIC ? DRIVER_NAME : NULL;
}

static int xmc4000_usic_request_port(struct uart_port *port)
{
	struct xmc4000_uart_port *xmc_port = to_xmc_port(port);
	int ret;

	port->membase = ioremap(port->mapbase, 0x200);
	if (!port->membase) {
		dev_err(port->dev, "failed to remap\n");
		ret = -ENOMEM;
		goto err_ioremap;
	}

	xmc_port->clk = clk_get(port->dev->parent, NULL);
	if (IS_ERR(xmc_port->clk)) {
		dev_err(port->dev, "failed to get clock\n");
		ret = PTR_ERR(xmc_port->clk);
		goto err_clk_get;
	}

	ret = clk_prepare(xmc_port->clk);
	if (ret) {
		dev_err(port->dev, "failed to prepare clock\n");
		goto err_clk_prepare;
	}

	return 0;

err_clk_prepare:
	clk_put(xmc_port->clk);
err_clk_get:
	iounmap(port->membase);
err_ioremap:
	return ret;
}

static void xmc4000_usic_release_port(struct uart_port *port)
{
	struct xmc4000_uart_port *xmc_port = to_xmc_port(port);

	clk_unprepare(xmc_port->clk);
	clk_put(xmc_port->clk);

	iounmap(port->membase);
}

static void xmc4000_usic_config_port(struct uart_port *port, int type)
{
	if (xmc4000_usic_request_port(port))
		return;

	if (type & UART_CONFIG_TYPE)
		port->type = PORT_XMC4000_USIC;
}

static int xmc4000_usic_verify_port(struct uart_port *port,
		struct serial_struct *serinfo)
{
	if (serinfo->type != PORT_UNKNOWN && serinfo->type != PORT_XMC4000_USIC)
		return -EINVAL;

	return 0;
}

static struct uart_ops xmc4000_uart_ops = {
	.tx_empty = xmc4000_usic_tx_empty,
	.set_mctrl = xmc4000_usic_set_mctrl,
	.get_mctrl = xmc4000_usic_get_mctrl,
	.start_tx = xmc4000_usic_start_tx,
	.stop_tx = xmc4000_usic_stop_tx,
	.stop_rx = xmc4000_usic_stop_rx,
	.break_ctl = xmc4000_usic_break_ctl,
	.startup = xmc4000_usic_startup,
	.shutdown = xmc4000_usic_shutdown,
	.set_termios = xmc4000_usic_set_termios,
	.type = xmc4000_usic_type,
	.request_port = xmc4000_usic_request_port,
	.release_port = xmc4000_usic_release_port,
	.config_port = xmc4000_usic_config_port,
	.verify_port = xmc4000_usic_verify_port,
};


#ifdef CONFIG_SERIAL_XMC4000_USIC_CONSOLE

static void xmc4000_console_putchar(struct uart_port *port, int ch)
{
	struct xmc4000_uart_port *xmc_port = to_xmc_port(port);

	xmc4000_usic_putchar(xmc_port, ch);
}

static void xmc4000_console_write(struct console *con, const char *s, unsigned int count)
{
	struct xmc4000_uart_port *xmc_port;

	xmc_port = xmc4000_uart_get(con->index);

	uart_console_write(&xmc_port->port, s, count, xmc4000_console_putchar);
}

static int xmc4000_console_setup(struct console *con, char *options)
{
	struct xmc4000_uart_port *xmc_port;
	int baud = 19200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int ret;

	xmc_port = xmc4000_uart_get(con->index);
	if (!xmc_port) {
		pr_warn("No XMC4000 console at %d\n", con->index);
		return -ENODEV;
	}

	ret = clk_prepare(xmc_port->clk);
	if (ret) {
		dev_warn(xmc_port->port.dev, "failed to prepare clock\n");
		return ret;
	}
	xmc_port->port.uartclk = clk_get_rate(xmc_port->clk);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&xmc_port->port, con, baud, parity, bits, flow);
}

static struct uart_driver xmc4000_uart_driver;

static struct console xmc4000_console = {
	.name	= XMC4000_UART_DEVICE,
	.device	= uart_console_device,
	.write	= xmc4000_console_write,
	.setup	= xmc4000_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
	.data	= &xmc4000_uart_driver,
};

#define XMC4000_SERIAL_CONSOLE (&xmc4000_console)

#else
#define XMC4000_SERIAL_CONSOLE NULL
#endif

static struct uart_driver xmc4000_uart_driver = {
	.driver_name	= DRIVER_NAME,
	.dev_name	= XMC4000_UART_DEVICE,
	.major		= 0,
	.minor		= 0,
	.nr		= ARRAY_SIZE(xmc4000_uart_ports),
	.cons		= XMC4000_SERIAL_CONSOLE,
};

static int xmc4000_usic_probe(struct platform_device *pdev)
{
	struct xmc4000_uart_port *xmc_port;
	struct resource *res;
	u32 mode;
	int ret, id;

	ret = of_property_read_u32(pdev->dev.parent->of_node,
				   "infineon,usic-mode", &mode);
	if (ret)
		return ret;

	if (WARN_ON(mode != USICx_CHy_CCR_MODE_ASC))
		return -EINVAL;

	xmc_port = kzalloc(sizeof(*xmc_port), GFP_KERNEL);
	if (!xmc_port)
		return -ENOMEM;

	xmc_port->port.dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_warn(&pdev->dev, "failed to determine base address\n");
		ret = -ENODEV;
		goto err_get_mem;
	}
	xmc_port->port.mapbase = res->start;

	xmc_port->port.type = PORT_XMC4000_USIC;
	xmc_port->port.iotype = UPIO_MEM32;
	xmc_port->port.ops = &xmc4000_uart_ops;
	xmc_port->port.flags = UPF_BOOT_AUTOCONF;

	id = of_alias_get_id(pdev->dev.parent->of_node, "serial");
	if (id >= 0) {
		xmc_port->port.line = id;
	} else
		dev_warn(&pdev->dev, "no alias found, using 0\n");
	xmc4000_uart_ports[xmc_port->port.line] = xmc_port;

	ret = uart_add_one_port(&xmc4000_uart_driver, &xmc_port->port);
	if (ret)
		goto err_add_port;

	platform_set_drvdata(pdev, xmc_port);

	return 0;

err_add_port:
	xmc4000_uart_ports[xmc_port->port.line] = NULL;
err_get_mem:
	kfree(xmc_port);
	return ret;
}

static int xmc4000_usic_remove(struct platform_device *pdev)
{
	struct xmc4000_uart_port *xmc_port = platform_get_drvdata(pdev);

	uart_remove_one_port(&xmc4000_uart_driver, &xmc_port->port);

	return 0;
}

static const struct of_device_id xmc4000_usic_of_match_table[] = {
	{ .compatible = "infineon,xmc4500-usic-channel-asc" },
	{ }
};
MODULE_DEVICE_TABLE(of, xmc4000_usic_of_match_table);

static struct platform_driver xmc4000_usic_driver = {
	.probe = xmc4000_usic_probe,
	.remove = xmc4000_usic_remove,

	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = xmc4000_usic_of_match_table,
	},
};

static __init int xmc4500_usic_init(void)
{
	int ret = 0;

	ret = uart_register_driver(&xmc4000_uart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&xmc4000_usic_driver);
	if (ret) {
		uart_unregister_driver(&xmc4000_uart_driver);
		return ret;
	}

	return ret;
}
module_init(xmc4500_usic_init);

static __exit void xmc4500_usic_exit(void)
{
	platform_driver_unregister(&xmc4000_usic_driver);
	uart_unregister_driver(&xmc4000_uart_driver);
}
module_exit(xmc4500_usic_exit);

MODULE_LICENSE("GPL");
