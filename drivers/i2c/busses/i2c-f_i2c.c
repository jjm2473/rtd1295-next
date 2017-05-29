/*
 * linux/drivers/i2c/busses/i2c-f_i2c.c
 *
 * Copyright (C) 2012 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/of_i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/platform_data/f_i2c.h>
#include "i2c-f_i2c.h"

#define DRV_NAME "f_i2c"

enum f_i2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_READ,
	STATE_WRITE
};

struct f_i2c {
	struct completion completion;

	struct i2c_msg *msg;
	unsigned int msg_num;
	unsigned int msg_idx;
	unsigned int msg_ptr;

	struct device *dev;
	void __iomem *base;
	unsigned int irq;
	struct clk *clk;
	unsigned long clkrate;
	unsigned int speed_khz;
	unsigned long timeout_ms;
	enum f_i2c_state state;
	struct i2c_adapter adapter;

	bool is_suspended;
};

static inline int is_lastmsg(struct f_i2c *i2c)
{
	return i2c->msg_idx >= (i2c->msg_num - 1);
}

static inline int is_msglast(struct f_i2c *i2c)
{
	return i2c->msg_ptr == (i2c->msg->len - 1);
}

static inline int is_msgend(struct f_i2c *i2c)
{
	return i2c->msg_ptr >= i2c->msg->len;
}

static inline unsigned long calc_timeout_ms(struct f_i2c *i2c,
					struct i2c_msg *msgs, int num)
{
	unsigned long bit_count = 0;
	int i;

	for (i = 0; i < num; i++, msgs++)
		bit_count += msgs->len;

	return DIV_ROUND_UP(((bit_count * 9) + (10 * num)) * 3, 200) + 10;
}

static void f_i2c_stop(struct f_i2c *i2c, int ret)
{
	dev_dbg(i2c->dev, "STOP\n");

	/*
	 * clear IRQ (INT=0, BER=0)
	 * set Stop Condition (MSS=0)
	 * Interrupt Disable
	 */
	writeb(0, i2c->base + F_I2C_REG_BCR);

	i2c->state = STATE_IDLE;

	i2c->msg_ptr = 0;
	i2c->msg = NULL;
	i2c->msg_idx++;
	i2c->msg_num = 0;
	if (ret)
		i2c->msg_idx = ret;

	complete(&i2c->completion);
}

static void f_i2c_hw_init(struct f_i2c *i2c)
{
	unsigned char ccr_cs, csr_cs;

	/* Set own Address */
	writeb(0, i2c->base + F_I2C_REG_ADR);

	/* Set PCLK frequency */
	writeb(F_I2C_BUS_CLK_FR(i2c->clkrate), i2c->base + F_I2C_REG_FSR);

	switch (i2c->speed_khz) {
	case F_I2C_SPEED_FM:
		if (i2c->clkrate <= F_I2C_CLK_RATE_18M) {
			ccr_cs = F_I2C_CCR_CS_FAST_MAX_18M(i2c->clkrate);
			csr_cs = F_I2C_CSR_CS_FAST_MAX_18M(i2c->clkrate);
		} else {
			ccr_cs = F_I2C_CCR_CS_FAST_MIN_18M(i2c->clkrate);
			csr_cs = F_I2C_CSR_CS_FAST_MIN_18M(i2c->clkrate);
		}

		/* Set Clock and enable, Set fast mode*/
		writeb(ccr_cs | F_I2C_CCR_FM | F_I2C_CCR_EN,
						i2c->base + F_I2C_REG_CCR);
		writeb(csr_cs, i2c->base + F_I2C_REG_CSR);
		break;
	case F_I2C_SPEED_SM:
		if (i2c->clkrate <= F_I2C_CLK_RATE_18M) {
			ccr_cs = F_I2C_CCR_CS_STANDARD_MAX_18M(i2c->clkrate);
			csr_cs = F_I2C_CSR_CS_STANDARD_MAX_18M(i2c->clkrate);
		} else {
			ccr_cs = F_I2C_CCR_CS_STANDARD_MIN_18M(i2c->clkrate);
			csr_cs = F_I2C_CSR_CS_STANDARD_MIN_18M(i2c->clkrate);
		}

		/* Set Clock and enable, Set standard mode */
		writeb(ccr_cs | F_I2C_CCR_EN, i2c->base + F_I2C_REG_CCR);
		writeb(csr_cs, i2c->base + F_I2C_REG_CSR);
		break;
	default:
		BUG();
	}

	/* clear IRQ (INT=0, BER=0), Interrupt Disable */
	writeb(0, i2c->base + F_I2C_REG_BCR);
	writeb(0, i2c->base + F_I2C_REG_BC2R);
}

static void f_i2c_hw_reset(struct f_i2c *i2c)
{
	/* Disable clock */
	writeb(0, i2c->base + F_I2C_REG_CCR);
	writeb(0, i2c->base + F_I2C_REG_CSR);

	WAIT_PCLK(100, i2c->clkrate);

	f_i2c_hw_init(i2c);
}

static int f_i2c_master_start(struct f_i2c *i2c, struct i2c_msg *pmsg)
{
	unsigned char bsr, bcr;

	if (pmsg->flags & I2C_M_RD)
		writeb((pmsg->addr << 1) | 1, i2c->base + F_I2C_REG_DAR);
	else
		writeb(pmsg->addr << 1, i2c->base + F_I2C_REG_DAR);

	dev_dbg(i2c->dev, "%s slave:0x%02x\n", __func__, pmsg->addr);

	/* Generate Start Condition */
	bsr = readb(i2c->base + F_I2C_REG_BSR);
	bcr = readb(i2c->base + F_I2C_REG_BCR);
	dev_dbg(i2c->dev, "%s bsr:0x%08x, bcr:0x%08x\n", __func__, bsr, bcr);

	if ((bsr & F_I2C_BSR_BB) && !(bcr & F_I2C_BCR_MSS)) {
		dev_dbg(i2c->dev, "%s bus is busy", __func__);
		return -EBUSY;
	}

	if (bsr & F_I2C_BSR_BB) { /* Bus is busy */
		dev_dbg(i2c->dev, "%s Continuous Start", __func__);
		writeb(bcr | F_I2C_BCR_SCC, i2c->base + F_I2C_REG_BCR);
	} else {
		if (bcr & F_I2C_BCR_MSS) {
			dev_dbg(i2c->dev, "%s is not in master mode", __func__);
			return -EAGAIN;
		}
		dev_dbg(i2c->dev, "%s Start Condition", __func__);
		/* Start Condition + Enable Interrupts */
		writeb(bcr | F_I2C_BCR_MSS | F_I2C_BCR_INTE | F_I2C_BCR_BEIE,
						     i2c->base + F_I2C_REG_BCR);
	}

	WAIT_PCLK(10, i2c->clkrate);

	/* get bsr&bcr register */
	bsr = readb(i2c->base + F_I2C_REG_BSR);
	bcr = readb(i2c->base + F_I2C_REG_BCR);
	dev_dbg(i2c->dev, "%s bsr:0x%08x, bcr:0x%08x\n", __func__, bsr, bcr);

	if ((bsr & F_I2C_BSR_AL) || !(bcr & F_I2C_BCR_MSS)) {
		dev_dbg(i2c->dev, "%s arbitration lost\n", __func__);
		return -EAGAIN;
	}

	return 0;
}

static int f_i2c_master_recover(struct f_i2c *i2c)
{
	unsigned int count = 0;
	unsigned char bc2r;

	/* Disable interrupts */
	writeb(0, i2c->base + F_I2C_REG_BCR);

	/* monitor SDA, SCL */
	bc2r = readb(i2c->base + F_I2C_REG_BC2R);
	dev_dbg(i2c->dev, "%s bc2r:0x%08x\n", __func__, (unsigned)bc2r);

	while (count <= 100) {
		WAIT_PCLK(20, i2c->clkrate);
		bc2r = readb(i2c->base + F_I2C_REG_BC2R);

		/* another master is running */
		if ((bc2r & F_I2C_BC2R_SDAS) || !(bc2r & F_I2C_BC2R_SCLS)) {
			dev_dbg(i2c->dev, "%s: another master is running?\n",
							__func__);
			return -EAGAIN;
		}
		count++;
	}

	/* Force to make one clock pulse */
	count = 0;
	for (;;) {
		/* SCL = L->H */
		writeb(F_I2C_BC2R_SCLL, i2c->base + F_I2C_REG_BC2R);
		WAIT_PCLK(20, i2c->clkrate);
		writeb(0, i2c->base + F_I2C_REG_BC2R);

		WAIT_PCLK(10, i2c->clkrate);

		bc2r = readb(i2c->base + F_I2C_REG_BC2R);

		WAIT_PCLK(5, i2c->clkrate);

		if (bc2r & F_I2C_BC2R_SDAS)
			break;
		WAIT_PCLK(10, i2c->clkrate);
		if (++count > 9) {
			dev_err(i2c->dev, "%s: count: %i, bc2r: 0x%x\n",
						__func__, count, bc2r);
			return -EIO;
		}
	}

	/* force to make bus-error phase */
	/* SDA = L */
	writeb(F_I2C_BC2R_SDAL, i2c->base + F_I2C_REG_BC2R);
	WAIT_PCLK(10, i2c->clkrate);
	/* SDA = H */
	writeb(0, i2c->base + F_I2C_REG_BC2R);
	WAIT_PCLK(10, i2c->clkrate);

	/* Both SDA & SDL should be H */
	bc2r = readb(i2c->base + F_I2C_REG_BC2R);
	if (!(bc2r & F_I2C_BC2R_SDAS) || !(bc2r & F_I2C_BC2R_SCLS)) {
		dev_err(i2c->dev, "%s: bc2r: 0x%x\n", __func__, bc2r);
		return -EIO;
	}

	return 0;
}

static int f_i2c_doxfer(struct f_i2c *i2c,
				struct i2c_msg *msgs, int num)
{
	unsigned char bsr;
	unsigned long timeout, bb_timout;
	int ret = 0;

	if (i2c->is_suspended)
		return -EBUSY;

	f_i2c_hw_init(i2c);
	bsr = readb(i2c->base + F_I2C_REG_BSR);
	if (bsr & F_I2C_BSR_BB) {
		dev_err(i2c->dev, "cannot get bus (bus busy)\n");
		return -EBUSY;
	}

	init_completion(&i2c->completion);

	i2c->msg = msgs;
	i2c->msg_num = num;
	i2c->msg_ptr = 0;
	i2c->msg_idx = 0;
	i2c->state = STATE_START;

	ret = f_i2c_master_start(i2c, i2c->msg);
	if (ret < 0) {
		dev_dbg(i2c->dev, "Address failed: (0x%08x)\n", ret);
		goto out;
	}

	timeout = wait_for_completion_timeout(&i2c->completion,
						F_I2C_TIMEOUT(i2c->timeout_ms));
	if (timeout <= 0) {
		dev_dbg(i2c->dev, "timeout\n");
		ret = -EAGAIN;
		goto out;
	}

	ret = i2c->msg_idx;
	if (ret != num) {
		dev_dbg(i2c->dev, "incomplete xfer (%d)\n", ret);
		ret = -EAGAIN;
		goto out;
	}

	/* ensure the stop has been through the bus */
	bb_timout = jiffies + HZ;
	do {
		bsr = readb(i2c->base + F_I2C_REG_BSR);
	} while ((bsr & F_I2C_BSR_BB) && time_before(jiffies, bb_timout));
out:
	return ret;
}

static irqreturn_t f_i2c_isr(int irq, void *dev_id)
{
	struct f_i2c *i2c = dev_id;

	unsigned char byte;
	unsigned char bsr, bcr;
	int ret = 0;

	bcr = readb(i2c->base + F_I2C_REG_BCR);
	bsr = readb(i2c->base + F_I2C_REG_BSR);
	dev_dbg(i2c->dev, "%s bsr:0x%08x, bcr:0x%08x\n", __func__,
					(unsigned)bsr, (unsigned)bcr);

	if (bcr & F_I2C_BCR_BER) {
		dev_err(i2c->dev, "%s: bus error\n", __func__);
		f_i2c_stop(i2c, -EAGAIN);
		goto out;
	}
	if ((bsr & F_I2C_BSR_AL) || !(bcr & F_I2C_BCR_MSS)) {
		dev_dbg(i2c->dev, "%s arbitration lost\n", __func__);
		f_i2c_stop(i2c, -EAGAIN);
		goto out;
	}

	switch (i2c->state) {

	case STATE_START:
		if (bsr & F_I2C_BSR_LRB) {
			dev_dbg(i2c->dev, "ack was not received\n");
			f_i2c_stop(i2c, -EAGAIN);
			goto out;
		}

		if (i2c->msg->flags & I2C_M_RD)
			i2c->state = STATE_READ;
		else
			i2c->state = STATE_WRITE;

		if (is_lastmsg(i2c) && i2c->msg->len == 0) {
			f_i2c_stop(i2c, 0);
			goto out;
		}

		if (i2c->state == STATE_READ)
			goto prepare_read;

		/* fallthru */

	case STATE_WRITE:
		if (bsr & F_I2C_BSR_LRB) {
			dev_dbg(i2c->dev, "WRITE: No Ack\n");
			f_i2c_stop(i2c, -EAGAIN);
			goto out;
		}

		if (!is_msgend(i2c)) {
			writeb(i2c->msg->buf[i2c->msg_ptr++],
						i2c->base + F_I2C_REG_DAR);

			/* clear IRQ, and continue */
			writeb(F_I2C_BCR_BEIE | F_I2C_BCR_MSS |
				F_I2C_BCR_INTE, i2c->base + F_I2C_REG_BCR);
			break;
		}
		if (is_lastmsg(i2c)) {
			f_i2c_stop(i2c, 0);
			break;
		}
		dev_dbg(i2c->dev, "WRITE: Next Message\n");

		i2c->msg_ptr = 0;
		i2c->msg_idx++;
		i2c->msg++;

		/* send the new start */
		ret = f_i2c_master_start(i2c, i2c->msg);
		if (ret < 0) {
			dev_dbg(i2c->dev, "restart err:0x%08x\n", ret);
			f_i2c_stop(i2c, -EAGAIN);
			break;
		}
		i2c->state = STATE_START;
		break;

	case STATE_READ:
		if (!(bsr & F_I2C_BSR_FBT)) { /* data */
			byte = readb(i2c->base + F_I2C_REG_DAR);
			i2c->msg->buf[i2c->msg_ptr++] = byte;
		} else /* address */
			dev_dbg(i2c->dev, ", address:0x%08x. ignore it.\n",
					readb(i2c->base + F_I2C_REG_DAR));

prepare_read:
		if (is_msglast(i2c)) {
			writeb(F_I2C_BCR_MSS | F_I2C_BCR_BEIE | F_I2C_BCR_INTE,
						     i2c->base + F_I2C_REG_BCR);
			break;
		}
		if (!is_msgend(i2c)) {
			writeb(F_I2C_BCR_MSS | F_I2C_BCR_BEIE |
					F_I2C_BCR_INTE | F_I2C_BCR_ACK,
						     i2c->base + F_I2C_REG_BCR);
			break;
		}
		if (is_lastmsg(i2c)) {
			/* last message, send stop and complete */
			dev_dbg(i2c->dev, "READ: Send Stop\n");
			f_i2c_stop(i2c, 0);
			break;
		}
		dev_dbg(i2c->dev, "READ: Next Transfer\n");

		i2c->msg_ptr = 0;
		i2c->msg_idx++;
		i2c->msg++;

		ret = f_i2c_master_start(i2c, i2c->msg);
		if (ret < 0) {
			dev_dbg(i2c->dev, "restart err: 0x%08x\n", ret);
			f_i2c_stop(i2c, -EAGAIN);
		} else
			i2c->state = STATE_START;
		break;
	default:
		dev_err(i2c->dev, "%s: called in err STATE (%d)\n",
			 __func__, i2c->state);
		break;
	}

out:
	WAIT_PCLK(10, i2c->clkrate);
	return IRQ_HANDLED;
}

static int f_i2c_xfer(struct i2c_adapter *adap,
				struct i2c_msg *msgs, int num)
{
	struct f_i2c *i2c;
	int retry;
	int ret = 0;

	if (!msgs)
		return -EINVAL;
	if (num <= 0)
		return -EINVAL;

	i2c = i2c_get_adapdata(adap);
	i2c->timeout_ms = calc_timeout_ms(i2c, msgs, num);

	dev_dbg(i2c->dev, "calculated timeout %ld ms\n", i2c->timeout_ms);

	for (retry = 0; retry < adap->retries; retry++) {

		ret = f_i2c_doxfer(i2c, msgs, num);
		if (ret != -EAGAIN)
			return ret;

		dev_dbg(i2c->dev, "Retrying transmission (%d)\n", retry);

		f_i2c_master_recover(i2c);
		f_i2c_hw_reset(i2c);
	}

	dev_err(i2c->dev, "transmission err: %d\n", retry);

	return -EIO;
}

static u32 f_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm f_i2c_algo = {
	.master_xfer   = f_i2c_xfer,
	.functionality = f_i2c_functionality,
};

static struct i2c_adapter f_i2c_ops = {
	.owner		= THIS_MODULE,
	.name		= "f_i2c-adapter",
	.algo		= &f_i2c_algo,
	.retries	= 5,
};

static int f_i2c_probe(struct platform_device *pdev)
{
	struct f_i2c *i2c;
	struct f_i2c_platform_data *pdata;
	struct resource *r;
	int ret = 0;
	int speed_khz = 100;
	const int *p;

	pdata = pdev->dev.platform_data;
	if (pdev->dev.of_node) {
		p = of_get_property(pdev->dev.of_node, "clock-frequency", NULL);
		if (!p) {
			dev_err(&pdev->dev,
					"Missing clock-frequency property\n");
			return -EINVAL;
		}
		speed_khz = be32_to_cpu(*p) / 1000;
	} else
		if (pdata)
			speed_khz = pdata->speed_khz;

	i2c = kzalloc(sizeof(struct f_i2c), GFP_KERNEL);
	if (!i2c) {
		dev_err(&pdev->dev, "no memory for state\n");
		return -ENOMEM;
	}

	i2c->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(i2c->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		ret = PTR_ERR(i2c->clk);
		goto err_noclk;
	}
	dev_dbg(&pdev->dev, "clock source %p\n", i2c->clk);

	i2c->clkrate = clk_get_rate(i2c->clk);
	if ((i2c->clkrate < F_I2C_MIN_CLK_RATE) ||
					(i2c->clkrate > F_I2C_MAX_CLK_RATE)) {
		dev_err(&pdev->dev, "get clock rate err\n");
		ret = -EINVAL;
		goto err_noclk;
	}
	dev_dbg(&pdev->dev, "clock rate %ld\n", i2c->clkrate);
	clk_prepare_enable(i2c->clk);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		ret = -ENXIO;
		goto err_clk;
	}

	i2c->base = ioremap(r->start, r->end - r->start + 1);
	if (!i2c->base) {
		ret = -ENOMEM;
		goto err_clk;
	}

	dev_dbg(&pdev->dev, "registers %p (%p)\n", i2c->base, r);

	i2c->irq = platform_get_irq(pdev, 0);
	if (i2c->irq <= 0) {
		ret = -ENXIO;
		dev_err(&pdev->dev, "cannot find IRQ\n");
		goto err_iomap;
	}

	ret = request_irq(i2c->irq, f_i2c_isr, 0, DRV_NAME, i2c);
	if (ret != 0) {
		dev_err(&pdev->dev, "cannot claim IRQ %d\n", i2c->irq);
		goto err_iomap;
	}

	i2c->state = STATE_IDLE;
	i2c->dev = &pdev->dev;
	i2c->msg = NULL;
	i2c->speed_khz = F_I2C_SPEED_SM;
	if (speed_khz == F_I2C_SPEED_FM)
		i2c->speed_khz = F_I2C_SPEED_FM;

	f_i2c_hw_init(i2c);

	i2c->adapter = f_i2c_ops;
	i2c_set_adapdata(&i2c->adapter, i2c);
	i2c->adapter.dev.parent = &pdev->dev;
	i2c->adapter.dev.of_node = of_node_get(pdev->dev.of_node);
	i2c->adapter.nr = pdev->id;

	ret = i2c_add_numbered_adapter(&i2c->adapter);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add bus to i2c core\n");
		goto err_irq;
	}

	platform_set_drvdata(pdev, i2c);
	of_i2c_register_devices(&i2c->adapter);

	dev_info(&pdev->dev, "%s: f_i2c adapter\n",
				dev_name(&i2c->adapter.dev));

	return 0;

err_irq:
	of_node_put(pdev->dev.of_node);
	free_irq(i2c->irq, i2c);

err_iomap:
	iounmap(i2c->base);

err_clk:
	clk_disable_unprepare(i2c->clk);
	clk_put(i2c->clk);

err_noclk:
	kfree(i2c);

	return ret;
}

static int f_i2c_remove(struct platform_device *pdev)
{
	struct f_i2c *i2c = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	i2c_del_adapter(&i2c->adapter);
	clk_disable_unprepare(i2c->clk);
	clk_put(i2c->clk);
	free_irq(i2c->irq, i2c);
	iounmap(i2c->base);
	kfree(i2c);
	of_node_put(pdev->dev.of_node);

	return 0;
};


#ifdef CONFIG_PM_SLEEP
static int f_i2c_suspend(struct device *dev)
{
	struct f_i2c *i2c = dev_get_drvdata(dev);

	i2c_lock_adapter(&i2c->adapter);
	i2c->is_suspended = true;
	i2c_unlock_adapter(&i2c->adapter);

	clk_disable_unprepare(i2c->clk);

	return 0;
}

static int f_i2c_resume(struct device *dev)
{
	struct f_i2c *i2c = dev_get_drvdata(dev);
	int ret;

	i2c_lock_adapter(&i2c->adapter);

	ret = clk_prepare_enable(i2c->clk);

	if (!ret)
		i2c->is_suspended = false;

	i2c_unlock_adapter(&i2c->adapter);

	return ret;
}

static SIMPLE_DEV_PM_OPS(f_i2c_pm, f_i2c_suspend, f_i2c_resume);
#define F_I2C_PM	(&f_i2c_pm)
#else
#define F_I2C_PM	NULL
#endif

static const struct of_device_id f_i2c_dt_ids[] = {
	{ .compatible = "fujitsu,f_i2c" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, f_i2c_dt_ids);

static struct platform_driver f_i2c_driver = {
	.probe   = f_i2c_probe,
	.remove  = f_i2c_remove,
	.driver  = {
		.owner = THIS_MODULE,
		.name = DRV_NAME,
		.of_match_table = f_i2c_dt_ids,
		.pm = F_I2C_PM,
	},
};

static int __init f_i2c_init(void)
{
	return platform_driver_register(&f_i2c_driver);
}

static void __exit f_i2c_exit(void)
{
	platform_driver_unregister(&f_i2c_driver);
}

module_init(f_i2c_init);
module_exit(f_i2c_exit);

MODULE_AUTHOR("Fujitsu Semiconductor Ltd");
MODULE_DESCRIPTION("Fujitsu Semiconductor I2C Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:"DRV_NAME);
