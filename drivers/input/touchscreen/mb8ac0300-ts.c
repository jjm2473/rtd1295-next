/*
 *  MB8AC0300 TPIF Driver
 *
 * linux/drivers/input/touchscreen/tpif.c
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
/* LIMITATION: multi touch is not supported yet. */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/timer.h>
#include <linux/clk.h>

/* Register definitions based on MB8AC0300 Specifications */
#define	TPIF_MES_START		0x000	/* Measure start control register */
#define	TPIF_TPIF_DIS		0x004	/* Measure disable control register */
#define	TPIF_AQ_COMP		0x008	/* Data process complete register */
#define	TPIF_MES_AUTO		0x00c	/* Measure mode control register */
#define	TPIF_TIMER_A		0x010	/* Timer cycle set register */
#define	TPIF_TPIF_STS		0x014	/* TPIF status register */
#define	TPIF_MM_MODE		0x020	/* Manual mode pattern register */
#define	TPIF_EXSW_MM		0x024	/* Manual mode ExSW control register */
#define	TPIF_MM_STC		0x028	/* Manual mode STC width set register */
#define	TPIF_PD_PS		0x040	/* ADC power control register */
#define	TPIF_WAPW		0x044	/* ADC power on debouncing register */
#define	TPIF_WAUX1		0x060	/* AUX1 channel STC width set */
#define	TPIF_WAUX2		0x064	/* AUX2 channel STC width set */
#define	TPIF_WLPFD		0x080	/* Area detect LPF debouncing */
#define	TPIF_WPIQB		0x084	/* Pen detect debouncing register */
#define	TPIF_NSW1		0x088	/* SW1 change number register */
#define	TPIF_NSW2		0x08c	/* SW2 change number register */
#define	TPIF_ACTAREA0		0x090	/* Area detect data register0 */
#define	TPIF_ACTAREA1		0x094	/* Area detect data register1 */
#define	TPIF_ACTAREA2		0x098	/* Area detect data register2 */
#define	TPIF_ACTAREA3		0x09c	/* Area detect data register3 */
#define	TPIF_WLPF		0x0a0	/* Coordinate detect LPF debouncing */
#define	TPIF_WLPF2		0x0a4	/* Coordinate STC width register */
#define	TPIF_NMES		0x0a8	/* Measure number set register */
#define	TPIF_ICD_TIMER		0x0c0	/* Interrupt detect cycle set */
#define	TPIF_IRQ_MASK		0x0c4	/* Interrupt mask resigter */
#define	TPIF_DBADDR		0x0e0	/* Indirect access address register */
#define	TPIF_ADC_DATA		0x0e4	/* ADC data register */

/* for polling timer */
#define	DIV_10MS		1000
#define	DIV_1MS			10000
#define	DIV_100US		100000
#define	DIV_10US		1000000

/* threshold of move value */
#define	VALID_MIN_ABSX		6
#define	VALID_MIN_ABSY		6
#define	VALID_MAX_ABSX		50
#define	VALID_MAX_ABSY		30

#define	MAX_ABS_X	799
#define	MAX_ABS_Y	429


struct tpif {
	struct platform_device *pdev;
	int irq;
	struct input_dev *input;
	char phys[32];
	unsigned int absx;
	unsigned int absy;
	unsigned char prev_touch;
	unsigned suspended;
	struct timer_list tpif_timer;
	void __iomem *register_base_address;
	struct clk	*clk;
	unsigned int prev_absx;
	unsigned int prev_absy;
};

/* voltage of the upper left corner of the each cell divided by 32. */
/* NOTICE: This table is device-specific. */
/* You may need to adjust the value to devices(Evaluation kit) each. */
long CellStartVolt[32][2] = {
	{ 454, 3348 }, { 775, 3243 }, { 1214, 3228 }, { 1632, 3201 },
		{ 2037, 3191 }, { 2463, 3204 }, { 2899, 3236 }, { 3336, 3295 },
	{ 480, 2742 }, { 770, 2708 }, { 1208, 2668 }, { 1604, 2653 },
		{ 2024, 2666 }, { 2449, 2682 }, { 2880, 2687 }, { 3305, 2751 },
	{ 453, 1978 }, { 777, 2025 }, { 1163, 2016 }, { 1591, 2044 },
		{ 2019, 2051 }, { 2456, 2072 }, { 2884, 2069 }, { 3288, 2058 },
	{ 411, 1222 }, { 702, 1317 }, { 1111, 1378 }, { 1540, 1423 },
		{ 1990, 1448 }, { 2440, 1444 }, { 2887, 1416 }, { 3308, 1390 }
};

/* scale of each coordinate(100times value) */
long CellDiv[32][2] = {
	{ 361, 594 }, { 422, 525 }, { 402, 549 }, { 389, 537 },
		{ 410, 515 }, { 419, 512 }, { 420, 538 }, { 355, 533 },
	{ 326, 742 }, { 421, 663 }, { 381, 633 }, { 404, 591 },
		{ 409, 597 }, { 414, 592 }, { 409, 600 }, { 338, 673 },
	{ 364, 734 }, { 371, 687 }, { 412, 619 }, { 412, 603 },
		{ 420, 585 }, { 412, 610 }, { 388, 634 }, { 343, 649 },
	{ 327, 563 }, { 393, 521 }, { 413, 523 }, { 433, 497 },
		{ 433, 488 }, { 430, 467 }, { 405, 495 }, { 340, 509 }
};

/* coordinates of the upper left point */
long CellStartCoordinate[32][2] = {
	{  0,  62},
	{ 89,  62},
	{193,  62},
	{297,  62},
	{401,  62},
	{505,  62},
	{609,  62},
	{713,  62},

	{  0, 164},
	{ 89, 164},
	{193, 164},
	{297, 164},
	{401, 164},
	{505, 164},
	{609, 164},
	{713, 164},

	{  0, 267},
	{ 89, 267},
	{193, 267},
	{297, 267},
	{401, 267},
	{505, 267},
	{609, 267},
	{713, 267},

	{  0, 370},
	{ 89, 370},
	{193, 370},
	{297, 370},
	{401, 370},
	{505, 370},
	{609, 370},
	{713, 370}
};


/* module variables */

/* rotate degree */
	/*   0 (  0 degree) -> Origin(0,0) is upper left corner */
	/* 180 (180 degree) -> Origin(0,0) is lower right corner */
static int rotate;

/* x,y adjust value */
	/* when rotation(rotate=180), */
	/* to correct for the coordinates after the rotation */
static int adjust_x;
static int adjust_y = -40;

#define MAX_ADJ_X	400
#define MIN_ADJ_X	-400
#define MAX_ADJ_Y	240
#define MIN_ADJ_Y	-240

/* module parmeter */
module_param(rotate, int, ((S_IRUGO) | (S_IWUSR)));
module_param(adjust_x, int, ((S_IRUGO) | (S_IWUSR)));
module_param(adjust_y, int, ((S_IRUGO) | (S_IWUSR)));

/* register access macro */
#define tpif_write(base_address, reg, val) \
			set_register_bits((base_address), (reg), (val))
#define tpif_read(base_address, reg) \
			get_register_bits((base_address), (reg))

/* inline function for register write */
static inline void set_register_bits(void __iomem *base_address,
			unsigned long register_offset, unsigned long value)
{
	__raw_writel(value, base_address + register_offset);
}

/* inline function for register read */
static inline unsigned long get_register_bits(void __iomem *base_address,
						unsigned long register_offset)
{
	return __raw_readl(base_address + register_offset);
}

/* adjustment from voltage to pixel */
/* 1. Calculates the position of from the upper left of each cell. */
/* 2. To adjust the scale of each cell. */
/* 3. Adding the upper left corner point of each cell      */
/*    to calculate the coordinates of overall touch panel. */
static void Volt2pixel(unsigned int areano, unsigned long *adcdata,
				unsigned int *pixxpt, unsigned int *pixypt)
{
	long adcx, adcy;

	/* voltage from register value */
	adcx = (*adcdata >> 16) & 0xfff;
	adcy = *adcdata & 0xfff;

	/* Calculating voltage difference from the upper left corner */
	adcx = adcx - CellStartVolt[areano][0];
	adcy = CellStartVolt[areano][1] - adcy;

	/* check min(negative) value */
	if (adcx < 0)
		adcx = 0;
	if (adcy < 0)
		adcy = 0;

	/* divided by scale */
	/* NOTICE: CellDiv values are 100times value, */
	/*         So adc also 100times("+CellDiv[][]/2" is due to rounding) */
	adcx = (adcx * 100 + (CellDiv[areano][0]/2)) / CellDiv[areano][0];
	adcy = (adcy * 100 + (CellDiv[areano][1]/2)) / CellDiv[areano][1];

	/* add the upper left corner point of each cell */
	adcx += CellStartCoordinate[areano][0];
	adcy += CellStartCoordinate[areano][1];

	/* check max value */
	if (adcx > MAX_ABS_X)
		adcx = MAX_ABS_X;
	if (adcy > MAX_ABS_Y)
		adcx = MAX_ABS_Y;

	*pixxpt = (unsigned int)adcx, *pixypt = (unsigned int)adcy;
}

/* Timer Handler */
/* function for detection area, and coordinate measuring */
static void tpif_timer_handler(unsigned long timer_num)
{
	struct tpif	*ts_dev = (struct tpif *)timer_num;
	struct input_dev	*input_dev = ts_dev->input;

	unsigned long active[4];
	unsigned int status;
	unsigned long offset, data;
	unsigned long i, no, bit;
	unsigned char cur_touch = 0;
	unsigned int areano;
	unsigned int valid_absx, valid_absy;
	unsigned long timeo;

	/* Check status */
	status = tpif_read(ts_dev->register_base_address, TPIF_TPIF_STS);

	/* wait for completion of timer measure mode */
	timeo = jiffies + usecs_to_jiffies(100);
	while ((status != 0x20) && time_before(jiffies, timeo)) {
		status = tpif_read(ts_dev->register_base_address,
							TPIF_TPIF_STS);
		cpu_relax();
	}
	if (status != 0x20) {
		pr_err("err : re-schedule tpif_timer_handler() next 100ms.\n");
		/* restart timer */
		ts_dev->tpif_timer.expires = jiffies + (10 * HZ) /
			(DIV_10MS/10);
		ts_dev->tpif_timer.function = tpif_timer_handler;
		ts_dev->tpif_timer.data = (unsigned long)(ts_dev);
		add_timer(&ts_dev->tpif_timer);
		return;
	}

	active[0] = tpif_read(ts_dev->register_base_address, TPIF_ACTAREA0);
	active[1] = tpif_read(ts_dev->register_base_address, TPIF_ACTAREA1);
	active[2] = tpif_read(ts_dev->register_base_address, TPIF_ACTAREA2);
	active[3] = tpif_read(ts_dev->register_base_address, TPIF_ACTAREA3);

	/* check active area(cell) */
	for (i = 0; i < 4; i++) {
		for (no = 0, bit = 1; no < 32; no++, bit <<= 1) {
			if ((active[i] & bit) == 0)
				continue;

			cur_touch = 1;
			offset = ((i*32)+no) << 2;

			tpif_write(ts_dev->register_base_address, TPIF_DBADDR,
									offset);
			data = tpif_read(ts_dev->register_base_address,
								TPIF_ADC_DATA);
			break;
		}
		if (cur_touch != 1)
			continue;
		if (i == 0) {
			if (no < 8)
				areano = no*4 + 1;
			else if ((15 < no) && (no < 24))
				areano = (no-16)*4 + 2;
			else
				areano = 1;
		} else if (i == 1) {
			if (no < 8)
				areano = no*4 + 3;
			else if ((15 < no) && (no < 24))
				areano = (no-16)*4 + 4;
			else
				areano = 1;
		} else
			areano = 1;

		/* adjustment from voltage to pixel */
		Volt2pixel((areano-1), &data, &ts_dev->absx, &ts_dev->absy);
		if (ts_dev->prev_touch != 1)
			break;

		if (ts_dev->absx < ts_dev->prev_absx)
			valid_absx = ts_dev->prev_absx - ts_dev->absx;
		else
			valid_absx = ts_dev->absx - ts_dev->prev_absx;

		if (ts_dev->absy < ts_dev->prev_absy)
			valid_absy = ts_dev->prev_absy - ts_dev->absy;
		else
			valid_absy = ts_dev->absy - ts_dev->prev_absy;

		/* check validation */
		if (((valid_absx >= VALID_MIN_ABSX) ||
			(valid_absy >= VALID_MIN_ABSY)) ||
				((VALID_MAX_ABSX >= valid_absx) &&
						(VALID_MAX_ABSY >= valid_absy)))
			break;

		ts_dev->absx = ts_dev->prev_absx;
		ts_dev->absy = ts_dev->prev_absy;

		/* complete */
		tpif_write(ts_dev->register_base_address, TPIF_AQ_COMP, 0);

		do {
			status = tpif_read(ts_dev->register_base_address,
								TPIF_TPIF_STS);
		} while (status);

		/* Timer Measure setting */
		/* Interrupt Disable */
		tpif_write(ts_dev->register_base_address, TPIF_IRQ_MASK, 1);
		/* cycle */
		tpif_write(ts_dev->register_base_address, TPIF_TIMER_A, 0);
		/* AUX1 timer */
		tpif_write(ts_dev->register_base_address, TPIF_WAUX1, 3);
		/* AUX2 timer */
		tpif_write(ts_dev->register_base_address, TPIF_WAUX2, 3);
		/* Deboucing time */
		tpif_write(ts_dev->register_base_address, TPIF_WLPFD, 10000);
		/* IRQ deboucing time */
		tpif_write(ts_dev->register_base_address, TPIF_WPIQB, 150);
		/* SW1 change time */
		tpif_write(ts_dev->register_base_address, TPIF_WLPF, 10000);
		/* SW2 change time */
		tpif_write(ts_dev->register_base_address, TPIF_WLPF2, 150);
		/* SW1 change num */
		tpif_write(ts_dev->register_base_address, TPIF_NSW1, 3);
		/* SW2 change num */
		tpif_write(ts_dev->register_base_address, TPIF_NSW2, 7);
		/* measure count */
		tpif_write(ts_dev->register_base_address, TPIF_NMES, 0);
		/* Time measure mode */
		tpif_write(ts_dev->register_base_address, TPIF_MES_AUTO, 2);
		/* start */
		tpif_write(ts_dev->register_base_address, TPIF_MES_START, 1);

		/* restart timer - 10ms? */
		ts_dev->tpif_timer.expires = jiffies + (10 * HZ) / DIV_10MS;
		ts_dev->tpif_timer.function = tpif_timer_handler;
		ts_dev->tpif_timer.data = (unsigned long)(ts_dev);
		add_timer(&ts_dev->tpif_timer);

		return;

	} /* for */

	if (rotate == 180) {
		/* 180 degree rotation */
		ts_dev->absx = MAX_ABS_X - ts_dev->absx;
		ts_dev->absy = MAX_ABS_Y - ts_dev->absy;

		/* adjust */
		if (!((adjust_x < MIN_ADJ_X) || (adjust_x > MAX_ADJ_X))) {
			if (!(ts_dev->absx < adjust_x)) {
				if ((ts_dev->absx - adjust_x) < MAX_ABS_X)
					ts_dev->absx = ts_dev->absx - adjust_x;
				else
					ts_dev->absx = MAX_ABS_X;
			} else
					ts_dev->absx = 0;
		}

		if (!((adjust_y < MIN_ADJ_Y) || (adjust_y > MAX_ADJ_Y))) {
			if (!(ts_dev->absy < adjust_y)) {
				if ((ts_dev->absy - adjust_y) < MAX_ABS_Y)
					ts_dev->absy = ts_dev->absy - adjust_y;
				else
					ts_dev->absy = MAX_ABS_Y;
			} else
					ts_dev->absy = 0;
		}

	} else {
		/* adjust */
		if (!((adjust_x < MIN_ADJ_X) || (adjust_x > MAX_ADJ_X))) {
			if (!((ts_dev->absx + adjust_x) < 0)) {
				if ((ts_dev->absx + adjust_x) < MAX_ABS_X)
					ts_dev->absx = ts_dev->absx + adjust_x;
				else
					ts_dev->absx = MAX_ABS_X;
			} else
					ts_dev->absx = 0;
		}

		if (!((adjust_y < MIN_ADJ_Y) || (adjust_y > MAX_ADJ_Y))) {
			if (!((ts_dev->absy + adjust_y) < 0)) {
				if ((ts_dev->absy + adjust_y) < MAX_ABS_Y)
					ts_dev->absy = ts_dev->absy + adjust_y;
				else
					ts_dev->absy = MAX_ABS_Y;
			} else
					ts_dev->absy = 0;
		}
	}

	if (ts_dev->prev_touch == 0) {
		if (cur_touch == 1) {	/* new touch */
			input_report_abs(input_dev, ABS_X, ts_dev->absx);
			input_report_abs(input_dev, ABS_Y, ts_dev->absy);
			input_report_key(input_dev, BTN_TOUCH, 1);
			input_sync(input_dev);
		}
	} else {
		if (cur_touch == 0) {	/* release */
			input_report_key(input_dev, BTN_TOUCH, 0);
			input_sync(input_dev);

		} else {		/* touch continue */
			input_report_abs(input_dev, ABS_X, ts_dev->absx);
			input_report_abs(input_dev, ABS_Y, ts_dev->absy);
			input_sync(input_dev);
		}
	}

	if (cur_touch) {		/* touch or continue */
		tpif_write(ts_dev->register_base_address, TPIF_AQ_COMP, 0);

		do {
			status = tpif_read(ts_dev->register_base_address,
								TPIF_TPIF_STS);
		} while (status);

		/* Timer Measure setting */
		tpif_write(ts_dev->register_base_address, TPIF_IRQ_MASK, 1);
		tpif_write(ts_dev->register_base_address, TPIF_TIMER_A, 0);
		tpif_write(ts_dev->register_base_address, TPIF_WAUX1, 3);
		tpif_write(ts_dev->register_base_address, TPIF_WAUX2, 3);
		tpif_write(ts_dev->register_base_address, TPIF_WLPFD, 10000);
		tpif_write(ts_dev->register_base_address, TPIF_WPIQB, 150);
		tpif_write(ts_dev->register_base_address, TPIF_WLPF, 10000);
		tpif_write(ts_dev->register_base_address, TPIF_WLPF2, 150);
		tpif_write(ts_dev->register_base_address, TPIF_NSW1, 3);
		tpif_write(ts_dev->register_base_address, TPIF_NSW2, 7);
		tpif_write(ts_dev->register_base_address, TPIF_NMES, 0);
		tpif_write(ts_dev->register_base_address, TPIF_MES_AUTO, 2);
		tpif_write(ts_dev->register_base_address, TPIF_MES_START, 1);

		/* restart timer */
		ts_dev->tpif_timer.expires = jiffies + (10 * HZ) / DIV_10MS;
		ts_dev->tpif_timer.function = tpif_timer_handler;
		ts_dev->tpif_timer.data = (unsigned long)(ts_dev);
		add_timer(&ts_dev->tpif_timer);
	} else {
		tpif_write(ts_dev->register_base_address, TPIF_AQ_COMP, 0);

		do {
			status = tpif_read(ts_dev->register_base_address,
								TPIF_TPIF_STS);
		} while (status);

		/* transition to pen detect */
		tpif_write(ts_dev->register_base_address, TPIF_IRQ_MASK, 0);
		tpif_write(ts_dev->register_base_address, TPIF_MES_AUTO, 1);
		tpif_write(ts_dev->register_base_address, TPIF_MES_START, 1);
	}

	ts_dev->prev_touch = cur_touch;		/* update touch status */
	ts_dev->prev_absx = ts_dev->absx;	/* update touch status */
	ts_dev->prev_absy = ts_dev->absy;	/* update touch status */

}

/* Interrupt Handler */
/* function for pen detection */
static irqreturn_t tpif_interrupt(int irq, void *dev)
{
	struct tpif	*ts_dev = (struct tpif *)dev;

	unsigned int status;

	status = tpif_read(ts_dev->register_base_address, TPIF_TPIF_STS);
	if (status == 0x200) {
		tpif_write(ts_dev->register_base_address, TPIF_AQ_COMP, 0);
		return IRQ_NONE;
	}

	tpif_write(ts_dev->register_base_address, TPIF_AQ_COMP, 0);

	do {
		status = tpif_read(ts_dev->register_base_address,
							TPIF_TPIF_STS);
	} while (status);

	/* Start Timer Measure */
	ts_dev->tpif_timer.expires = jiffies + (10 * HZ) / DIV_10MS;
	ts_dev->tpif_timer.function = tpif_timer_handler;
	ts_dev->tpif_timer.data = (unsigned long)(ts_dev);
	add_timer(&ts_dev->tpif_timer);

	tpif_write(ts_dev->register_base_address, TPIF_IRQ_MASK, 1);
	tpif_write(ts_dev->register_base_address, TPIF_TIMER_A, 0);
	tpif_write(ts_dev->register_base_address, TPIF_WAUX1, 3);
	tpif_write(ts_dev->register_base_address, TPIF_WAUX2, 3);
	tpif_write(ts_dev->register_base_address, TPIF_WLPFD, 10000);
	tpif_write(ts_dev->register_base_address, TPIF_WPIQB, 150);
	tpif_write(ts_dev->register_base_address, TPIF_WLPF, 10000);
	tpif_write(ts_dev->register_base_address, TPIF_WLPF2, 150);
	tpif_write(ts_dev->register_base_address, TPIF_NSW1, 3);
	tpif_write(ts_dev->register_base_address, TPIF_NSW2, 7);
	tpif_write(ts_dev->register_base_address, TPIF_NMES, 0);
	tpif_write(ts_dev->register_base_address, TPIF_MES_AUTO, 2);
	tpif_write(ts_dev->register_base_address, TPIF_MES_START, 1);

	return IRQ_HANDLED;
}

static int tpif_probe(struct platform_device *pdev)
{
	struct tpif		*ts_dev;
	struct input_dev	*input_dev;
	struct resource		*res;
	int		err = 0;
	void __iomem	*register_base_address;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no mmio resource defined.\n");
		return -ENXIO;
	}

	/* Allocate memory for device */
	ts_dev = kzalloc(sizeof(struct tpif), GFP_KERNEL);
	if (!ts_dev) {
		dev_err(&pdev->dev, "failed to allocate memory.\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, ts_dev);

	ts_dev->pdev = pdev;

	/* Create the input device and register it. */
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&pdev->dev, "failed to allocate input device.\n");
		err = -ENOMEM;
		goto err_free_mem;
	}

	register_base_address = ioremap(res->start, resource_size(res));
	if (!register_base_address) {
		dev_err(&pdev->dev, "failed to map registers.\n");
		err = -ENOMEM;
		goto err_release_mem;
	}

	ts_dev->register_base_address = register_base_address;

	ts_dev->irq = platform_get_irq(pdev, 0);
	if (ts_dev->irq < 0) {
		dev_err(&pdev->dev, "no irq ID is designated.\n");
		err = -ENODEV;
		goto err_unmap_regs;
	}

	/* IRQ detect rising edge */
	err = request_irq(ts_dev->irq, tpif_interrupt, IRQF_TRIGGER_RISING,
			pdev->dev.driver->name, ts_dev);
	if (err) {
		dev_err(&pdev->dev, "failed to allocate irq.\n");
		goto err_unmap_regs;
	}

	ts_dev->clk = clk_get(&pdev->dev, "tpif");
	if (IS_ERR(ts_dev->clk)) {
		dev_err(&pdev->dev, "%s(): clock not found.\n", __func__);
		err = PTR_ERR(ts_dev->clk);
		goto err_free_irq;
	}

	init_timer(&ts_dev->tpif_timer);

	ts_dev->input = input_dev;
	snprintf(ts_dev->phys, sizeof(ts_dev->phys),
		 "%s/input0", dev_name(&pdev->dev));

	input_dev->name = "tpif driver";
	input_dev->phys = ts_dev->phys;
	input_dev->dev.parent = &pdev->dev;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

#if 0
	input_set_abs_params(input_dev, ABS_X, 0, ts_dev->absx,
			     0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, ts_dev->absy,
			     0, 0);
#else
	/*
	 * input_set_abs_params(input_device, AXIS, MIN, MAX, FUZZ, FLAT)
	 * MIN: minimum value for the axis, MAX: maximum value for the axis
	 * FUZZ: used to filter noise from the event stream
	 * FLAT: within this value is discarded by joydev interface
	 * (for joystic flat position?)
	 */
	input_set_abs_params(input_dev, ABS_X, 0, MAX_ABS_X,
			     0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_ABS_Y,
			     0, 0);

	/* input_set_abs_res(input_device, AXIS, RESOLUTION) */
	/* RESOLUTION: resolution for the values reported for the axis.*/
	input_abs_set_res(input_dev, ABS_X, 0);
	input_abs_set_res(input_dev, ABS_Y, 0);
#endif

	err = clk_prepare_enable(ts_dev->clk);
	if (err)
		goto err_free_irq;

	/* TPIF register setting */
	tpif_write(ts_dev->register_base_address, TPIF_IRQ_MASK, 0);
	tpif_write(ts_dev->register_base_address, TPIF_MES_AUTO, 1);
	tpif_write(ts_dev->register_base_address, TPIF_MES_START, 1);

	/* All went ok, so register to the input system */
	err = input_register_device(input_dev);
	if (err)
		goto err_fail;

	return 0;

err_fail:
	clk_disable_unprepare(ts_dev->clk);
	clk_put(ts_dev->clk);
err_free_irq:
	free_irq(ts_dev->irq, ts_dev);
err_unmap_regs:
	iounmap(register_base_address);
err_release_mem:
	input_free_device(input_dev);
err_free_mem:
	kfree(ts_dev);
	return err;
}

static int __exit tpif_remove(struct platform_device *pdev)
{
	struct tpif *ts_dev = dev_get_drvdata(&pdev->dev);

	free_irq(ts_dev->irq, ts_dev);
	del_timer(&ts_dev->tpif_timer);
	clk_disable_unprepare(ts_dev->clk);
	clk_put(ts_dev->clk);
	input_unregister_device(ts_dev->input);

	iounmap(ts_dev->register_base_address);
	input_free_device(ts_dev->input);

	kfree(ts_dev);

	return 0;
}
#ifdef CONFIG_PM
static int tpif_suspend(struct device *dev)
{
	struct tpif *ts_dev = dev_get_drvdata(dev);

	dev_info(dev, "%s() is started.", __func__);

	/* mask the interrupt source before free_irq() */
	tpif_write(ts_dev->register_base_address, TPIF_IRQ_MASK,
		0xffffffff);

	free_irq(ts_dev->irq, ts_dev);
	del_timer(&ts_dev->tpif_timer);
	clk_disable_unprepare(ts_dev->clk);

	dev_info(dev, "%s() is ended.", __func__);
	return 0;
}

static int tpif_resume(struct device *dev)
{
	struct tpif *ts_dev = dev_get_drvdata(dev);
	int	err = 0;

	dev_info(dev, "%s() is started.", __func__);

	err = clk_prepare_enable(ts_dev->clk);
	if (err) {
		dev_err(dev, "%s() failed to enable clock", __func__);
		return -1;
	}

	err = request_irq(ts_dev->irq, tpif_interrupt, IRQF_TRIGGER_RISING,
		dev->driver->name, ts_dev);
	if (err) {
		dev_err(dev, "failed to allocate irq.\n");
		return -1;
	}

	/* TPIF register setting */
	tpif_write(ts_dev->register_base_address, TPIF_IRQ_MASK, 0);
	tpif_write(ts_dev->register_base_address, TPIF_MES_AUTO, 1);
	tpif_write(ts_dev->register_base_address, TPIF_MES_START, 1);

	dev_info(dev, "%s() is ended.", __func__);
	return 0;
}

static const struct dev_pm_ops tpif_pm_ops = {
	.suspend = tpif_suspend,
	.resume = tpif_resume,
};

#define pm_ops (&tpif_pm_ops)
#else
#define pm_ops NULL
#endif

static const struct of_device_id mb8ac0300_tpif_dt_ids[] = {
	{ .compatible = "fujitsu,mb8ac0300-tpif" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, mb8ac0300_tpif_dt_ids);

static struct platform_driver tpif_driver = {
	.driver		= {
		.name	= "mb8ac0300-tpif",
		.owner = THIS_MODULE,
		.pm = pm_ops,
		.of_match_table = mb8ac0300_tpif_dt_ids,
	},
	.probe		= tpif_probe,
	.remove		= __exit_p(tpif_remove),
};

static int __init tpif_init(void)
{
	return platform_driver_register(&tpif_driver);
}

static void __exit tpif_exit(void)
{
	platform_driver_unregister(&tpif_driver);
}

module_init(tpif_init);
module_exit(tpif_exit);


MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MB8AC0300 TPIF Driver");
MODULE_AUTHOR("Fujitsu Semiconductor Ltd.");
