/*
 * FM4 MFS
 */

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/tty_flip.h>

#define FM4_MFSx_SCR		0x001

#define FM4_MFSx_SCR_TXE	BIT( 8 - 8)
#define FM4_MFSx_SCR_RXE	BIT( 9 - 8)
#define FM4_MFSx_SCR_TBIE	BIT(10 - 8)
#define FM4_MFSx_SCR_TIE	BIT(11 - 8)
#define FM4_MFSx_SCR_RIE	BIT(12 - 8)

#define FM4_MFSx_SSR		0x005

#define FM4_MFSx_SSR_TDRE	BIT(9 - 8)

#define FM4_MFSx_RDR		0x008
#define FM4_MFSx_TDR		0x008

#define DRIVER_NAME "fm4-mfs"
#define FM4_UART_DEVICE "ttyFM"
#define FM4_MAX_MFS_NR 16

#define to_fm4_port(_port) container_of(_port, struct fm4_uart_port, port)

struct fm4_uart_port {
	struct uart_port port;
	struct clk *clk;
	int receive_irq;
};

static struct fm4_uart_port *fm4_uart_ports[FM4_MAX_MFS_NR];

static struct fm4_uart_port *fm4_uart_get(int index)
{
	if (index >= 0 && index < FM4_MAX_MFS_NR)
		return fm4_uart_ports[index];

	return NULL;
}

static void fm4_mfs_putchar(struct fm4_uart_port *fm4_port, char ch)
{
	while (!(readb_relaxed(fm4_port->port.membase + FM4_MFSx_SSR) & FM4_MFSx_SSR_TDRE));

	writew_relaxed(ch, fm4_port->port.membase + FM4_MFSx_TDR);
}

static void fm4_mfs_do_rx(struct fm4_uart_port *fm4_port)
{
	struct tty_port *tty_port;
	u16 rdr;

	rdr = readw_relaxed(fm4_port->port.membase + FM4_MFSx_RDR);
	tty_port = &fm4_port->port.state->port;
	tty_insert_flip_char(tty_port, rdr & 0xff /*XXX*/, TTY_NORMAL);
	tty_flip_buffer_push(tty_port);
}

static void fm4_mfs_stop_tx(struct uart_port *port);

static void fm4_mfs_do_tx(struct fm4_uart_port *fm4_port)
{
	struct uart_port *port = &fm4_port->port;
	struct circ_buf *xmit;

	if (port->x_char) {
		fm4_mfs_putchar(fm4_port, port->x_char);
		port->x_char = 0;
		port->icount.tx++;
		return;
	}

	if (uart_tx_stopped(port)) {
		fm4_mfs_stop_tx(port);
		return;
	}

	xmit = &port->state->xmit;
	if (uart_circ_empty(xmit)) {
		fm4_mfs_stop_tx(port);
		return;
	}
	fm4_mfs_putchar(fm4_port, xmit->buf[xmit->tail]);
	xmit->tail = (xmit->tail + 1) % UART_XMIT_SIZE;
	port->icount.tx++;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		fm4_mfs_stop_tx(port);
}

static unsigned int fm4_mfs_tx_empty(struct uart_port *port)
{
	struct fm4_uart_port *fm4_port = to_fm4_port(port);
	u8 ssr;

	ssr = readb_relaxed(fm4_port->port.membase + FM4_MFSx_SSR);

	return ssr & FM4_MFSx_SSR_TDRE ? TIOCSER_TEMT : 0;
}

static void fm4_mfs_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int fm4_mfs_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void fm4_mfs_stop_tx(struct uart_port *port)
{
	u8 scr;

	scr = readb_relaxed(port->membase + FM4_MFSx_SCR);
	scr &= ~FM4_MFSx_SCR_TIE;
	writeb_relaxed(scr, port->membase + FM4_MFSx_SCR);
}

static void fm4_mfs_start_tx(struct uart_port *port)
{
	struct fm4_uart_port *fm4_port = to_fm4_port(port);
	u8 scr;

	scr = readb_relaxed(port->membase + FM4_MFSx_SCR);
	scr |= FM4_MFSx_SCR_TIE;
	writeb_relaxed(scr, port->membase + FM4_MFSx_SCR);

	fm4_mfs_do_tx(fm4_port);
}

static void fm4_mfs_stop_rx(struct uart_port *port)
{
	u8 scr;

	scr = readb_relaxed(port->membase + FM4_MFSx_SCR);
	scr &= ~FM4_MFSx_SCR_RXE;
	writeb_relaxed(scr, port->membase + FM4_MFSx_SCR);
}

static void fm4_mfs_break_ctl(struct uart_port *port, int ctl)
{
}

#define FM4_INTREQ_BASE	0x40031000

#define FM4_INTREQ_INTMON_BASE	(FM4_INTREQ_BASE + 0x204)

static irqreturn_t fm4_mfs_receive_irq(int irq, void *data)
{
	struct fm4_uart_port *fm4_port = data;
	void __iomem *intreq_base = (void __iomem *)FM4_INTREQ_INTMON_BASE;
	u32 intmon;

	intmon = readl_relaxed(intreq_base + 60 * 4);
	if (intmon & BIT(0)) {
		fm4_mfs_do_rx(fm4_port);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static irqreturn_t fm4_mfs_status_tx_irq(int irq, void *data)
{
	struct fm4_uart_port *fm4_port = data;
	void __iomem *intreq_base = (void __iomem *)FM4_INTREQ_INTMON_BASE;
	u32 intmon;

	intmon = readl_relaxed(intreq_base + 61 * 4);
	if (intmon & BIT(0)) {
		fm4_mfs_do_tx(fm4_port);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int fm4_mfs_startup(struct uart_port *port)
{
	struct fm4_uart_port *fm4_port = to_fm4_port(port);
	u8 scr;
	int ret;

	ret = clk_enable(fm4_port->clk);
	if (ret) {
		dev_err(fm4_port->port.dev, "enabling clock failed\n");
		goto err_clk_enable;
	}

	ret = request_irq(fm4_port->receive_irq, fm4_mfs_receive_irq,
		IRQF_IRQPOLL, DRIVER_NAME, fm4_port);
	if (ret) {
		dev_err(fm4_port->port.dev, "requesting first irq failed\n");
		goto err_request_irq0;
	}

	ret = request_irq(port->irq, fm4_mfs_status_tx_irq,
		IRQF_IRQPOLL, DRIVER_NAME, fm4_port);
	if (ret) {
		dev_err(fm4_port->port.dev, "requesting second irq failed\n");
		goto err_request_irq1;
	}

	scr = readb_relaxed(port->membase + FM4_MFSx_SCR);
	scr |= FM4_MFSx_SCR_RIE | FM4_MFSx_SCR_TIE | FM4_MFSx_SCR_RXE | FM4_MFSx_SCR_TXE;
	writeb_relaxed(scr, port->membase + FM4_MFSx_SCR);

	return 0;

err_request_irq1:
	free_irq(fm4_port->receive_irq, fm4_port);
err_request_irq0:
err_clk_enable:
	return ret;
}

static void fm4_mfs_shutdown(struct uart_port *port)
{
	struct fm4_uart_port *fm4_port = to_fm4_port(port);
	u8 scr;

	scr = readb_relaxed(port->membase + FM4_MFSx_SCR);
	scr &= ~(FM4_MFSx_SCR_RIE | FM4_MFSx_SCR_TIE);
	writeb_relaxed(scr, port->membase + FM4_MFSx_SCR);

	free_irq(port->irq, fm4_port);
	free_irq(fm4_port->receive_irq, fm4_port);

	clk_disable(fm4_port->clk);
}

static void fm4_mfs_set_termios(struct uart_port *port,
	struct ktermios *new, struct ktermios *old)
{
}

static const char *fm4_mfs_type(struct uart_port *port)
{
	return port->type == PORT_FM4_MFS ? DRIVER_NAME : NULL;
}

static int fm4_mfs_request_port(struct uart_port *port)
{
	struct fm4_uart_port *fm4_port = to_fm4_port(port);
	int ret;

	port->membase = ioremap(port->mapbase, 0x100);
	if (!port->membase) {
		dev_err(port->dev, "failed to remap\n");
		ret = -ENOMEM;
		goto err_ioremap;
	}

	fm4_port->clk = clk_get(port->dev, NULL);
	if (IS_ERR(fm4_port->clk)) {
		dev_err(port->dev, "failed to get clock\n");
		ret = PTR_ERR(fm4_port->clk);
		goto err_clk_get;
	}

	ret = clk_prepare(fm4_port->clk);
	if (ret) {
		dev_err(port->dev, "failed to prepare clock\n");
		goto err_clk_prepare;
	}

	return 0;

err_clk_prepare:
	clk_put(fm4_port->clk);
err_clk_get:
	iounmap(port->membase);
err_ioremap:
	return ret;
}

static void fm4_mfs_release_port(struct uart_port *port)
{
	struct fm4_uart_port *fm4_port = to_fm4_port(port);

	clk_unprepare(fm4_port->clk);
	clk_put(fm4_port->clk);

	iounmap(port->membase);
}

static void fm4_mfs_config_port(struct uart_port *port, int type)
{
	if (fm4_mfs_request_port(port))
		return;

	if (type & UART_CONFIG_TYPE)
		port->type = PORT_FM4_MFS;
}

static int fm4_mfs_verify_port(struct uart_port *port,
		struct serial_struct *serinfo)
{
	if (serinfo->type != PORT_UNKNOWN && serinfo->type != PORT_FM4_MFS)
		return -EINVAL;

	return 0;
}

static struct uart_ops fm4_uart_ops = {
	.tx_empty	= fm4_mfs_tx_empty,
	.set_mctrl	= fm4_mfs_set_mctrl,
	.get_mctrl	= fm4_mfs_get_mctrl,
	.start_tx	= fm4_mfs_start_tx,
	.stop_tx	= fm4_mfs_stop_tx,
	.stop_rx	= fm4_mfs_stop_rx,
	.break_ctl	= fm4_mfs_break_ctl,
	.startup	= fm4_mfs_startup,
	.shutdown	= fm4_mfs_shutdown,
	.set_termios	= fm4_mfs_set_termios,
	.type		= fm4_mfs_type,
	.request_port	= fm4_mfs_request_port,
	.release_port	= fm4_mfs_release_port,
	.config_port	= fm4_mfs_config_port,
	.verify_port	= fm4_mfs_verify_port,
};


#ifdef CONFIG_SERIAL_FM4_MFS_CONSOLE

static void fm4_console_putchar(struct uart_port *port, int ch)
{
	struct fm4_uart_port *fm4_port = to_fm4_port(port);

	fm4_mfs_putchar(fm4_port, ch);
}

static void fm4_serial_port_write(struct uart_port *port, const char *s, unsigned int count)
{
	u8 scr, interrupts;

	scr = readb_relaxed(port->membase + FM4_MFSx_SCR);
	interrupts = scr & (FM4_MFSx_SCR_RIE | FM4_MFSx_SCR_TIE | FM4_MFSx_SCR_TBIE);
	if (interrupts) {
		scr &= ~interrupts;
		writeb_relaxed(scr, port->membase + FM4_MFSx_SCR);
	}

	uart_console_write(port, s, count, fm4_console_putchar);

	if (interrupts) {
		scr = readb_relaxed(port->membase + FM4_MFSx_SCR);
		scr |= interrupts;
		writeb_relaxed(scr, port->membase + FM4_MFSx_SCR);
	}
}

static void fm4_console_write(struct console *con, const char *s, unsigned int count)
{
	struct fm4_uart_port *fm4_port;

	fm4_port = fm4_uart_get(con->index);

	fm4_serial_port_write(&fm4_port->port, s, count);
}

static int fm4_console_setup(struct console *con, char *options)
{
	struct fm4_uart_port *fm4_port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int ret;

	fm4_port = fm4_uart_get(con->index);
	if (!fm4_port) {
		pr_warn("No FM4 console at %d\n", con->index);
		return -ENODEV;
	}

	ret = clk_prepare(fm4_port->clk);
	if (ret) {
		dev_warn(fm4_port->port.dev, "failed to prepare clock\n");
		return ret;
	}
	fm4_port->port.uartclk = clk_get_rate(fm4_port->clk);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&fm4_port->port, con, baud, parity, bits, flow);
}

static struct uart_driver fm4_uart_driver;

static struct console fm4_console = {
	.name	= FM4_UART_DEVICE,
	.device	= uart_console_device,
	.write	= fm4_console_write,
	.setup	= fm4_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
	.data	= &fm4_uart_driver,
};

static int __init fm4_console_init(void)
{
	register_console(&fm4_console);
	return 0;
}
console_initcall(fm4_console_init);

static void fm4_serial_early_console_write(struct console *co,
					   const char *s,
					   u_int count)
{
	struct earlycon_device *dev = co->data;

	fm4_serial_port_write(&dev->port, s, count);
}

static int __init
fm4_serial_early_console_setup(struct earlycon_device *device, const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = fm4_serial_early_console_write;

	return 0;
}
OF_EARLYCON_DECLARE(fm, "cypress,fm4-mfs",
		    fm4_serial_early_console_setup);

#define FM4_SERIAL_CONSOLE (&fm4_console)

#else
#define FM4_SERIAL_CONSOLE NULL
#endif

static struct uart_driver fm4_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= DRIVER_NAME,
	.dev_name	= FM4_UART_DEVICE,
	.nr		= ARRAY_SIZE(fm4_uart_ports),
	.cons		= FM4_SERIAL_CONSOLE,
};

static int fm4_mfs_probe(struct platform_device *pdev)
{
	struct fm4_uart_port *fm4_port;
	struct resource *res;
	int ret, irq, id;

	fm4_port = kzalloc(sizeof(*fm4_port), GFP_KERNEL);
	if (!fm4_port)
		return -ENOMEM;

	fm4_port->port.dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_warn(&pdev->dev, "failed to determine base address\n");
		ret = -ENODEV;
		goto err_get_mem;
	}
	fm4_port->port.mapbase = res->start;

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(&pdev->dev, "failed to get first irq\n");
		ret = irq;
		goto err_get_irq0;
	}
	fm4_port->receive_irq = irq;

	irq = platform_get_irq(pdev, 1);
	if (irq <= 0) {
		dev_err(&pdev->dev, "failed to get second irq\n");
		ret = irq;
		goto err_get_irq1;
	}
	fm4_port->port.irq = irq;

	fm4_port->port.type = PORT_FM4_MFS;
	fm4_port->port.iotype = UPIO_MEM32;
	fm4_port->port.ops = &fm4_uart_ops;
	fm4_port->port.flags = UPF_BOOT_AUTOCONF;

	id = of_alias_get_id(pdev->dev.of_node, "serial");
	if (id >= 0) {
		fm4_port->port.line = id;
	} else
		dev_warn(&pdev->dev, "no alias found, using 0\n");
	fm4_uart_ports[fm4_port->port.line] = fm4_port;
	platform_set_drvdata(pdev, fm4_port);

	ret = uart_add_one_port(&fm4_uart_driver, &fm4_port->port);
	if (ret)
		goto err_add_port;

	return 0;

err_add_port:
	fm4_uart_ports[fm4_port->port.line] = NULL;
err_get_irq1:
err_get_irq0:
err_get_mem:
	kfree(fm4_port);
	return ret;
}

static int fm4_mfs_remove(struct platform_device *pdev)
{
	struct fm4_uart_port *fm4_port = platform_get_drvdata(pdev);

	uart_remove_one_port(&fm4_uart_driver, &fm4_port->port);

	return 0;
}

static const struct of_device_id fm4_mfs_of_match_table[] = {
	{ .compatible = "cypress,fm4-mfs" },
	{ }
};
MODULE_DEVICE_TABLE(of, fm4_mfs_of_match_table);

static struct platform_driver fm4_mfs_driver = {
	.probe = fm4_mfs_probe,
	.remove = fm4_mfs_remove,

	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = fm4_mfs_of_match_table,
	},
};

static __init int fm4_mfs_init(void)
{
	int ret = 0;

	ret = uart_register_driver(&fm4_uart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&fm4_mfs_driver);
	if (ret) {
		uart_unregister_driver(&fm4_uart_driver);
		return ret;
	}

	return ret;
}
module_init(fm4_mfs_init);

static __exit void fm4_mfs_exit(void)
{
	platform_driver_unregister(&fm4_mfs_driver);
	uart_unregister_driver(&fm4_uart_driver);
}
module_exit(fm4_mfs_exit);

MODULE_LICENSE("GPL");
