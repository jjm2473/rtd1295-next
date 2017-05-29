/*
 * Copyright (C) 2013 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 */
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk-provider.h>
#include <linux/spinlock.h>
#include <linux/module.h>

#include <asm/system_misc.h>

/* Page-1298 of MB8AC0300_SP_R1.0J.pdf  */

#define CRG_REG_CRPLC	0x0
#define CRG_REG_CRRDY	0x4
#define CRG_REG_CRSTP	0x8
#define CRG_REG_CRIMA	0x10
#define CRG_REG_CRPIC	0x14
#define CRG_REG_CRRSC	0x20
#define CRG_REG_CRSWR	0x24
#define CRG_REG_CRRRS	0x28
#define CRG_REG_CRRSM	0x2c
#define CRG_REG_CRCDC	0x30

#define CRG_OFF_CRDM	0x100
#define CRG_OFF_CRLP	0x104

#define CRPLC_FBMODE_MASK	0x3f
#define CRPLC_FBMODE_SHFT	0
#define CRPLC_PSMODE_MASK	0xf
#define CRPLC_PSMODE_SHFT	8
#define CRPLC_LUWMODE_MASK	0xf
#define CRPLC_LUWMODE_SHFT	16
#define CRPLC_PLLBYPASS		BIT(24)

#define CRRDY_PLLRDY		BIT(0)
#define CRRDY_PSRMNT		BIT(4)

#define CRSTP_STOPEN		BIT(0)
#define CRSTP_STOPMNT		BIT(1)

#define CRIMA_RDYINTM		BIT(0)

#define CRPIC_PLLRDYINT		BIT(0)

#define CRRSC_ARSTMODE_MASK	0xf
#define CRRSC_ARSTMODE_SHFT	0
#define CRRSC_SWRSTM		BIT(8)
#define CRRSC_WDRSTM		BIT(9)
#define CRRSC_SRSTMODE_MASK	0xf
#define CRRSC_SRSTMODE_SHFT	16

#define CRSWR_SWRSTREQ		BIT(0)

#define CRRRS_RRSTREQ_MASK	0xff
#define CRRRS_RRSTREQ_SHFT	0

#define CRRSM_WDRST		BIT(0)
#define CRRSM_SWRST		BIT(1)
#define CRRSM_SRST		BIT(2)
#define CRRSM_PORESET		BIT(3)

#define CRCDC_DCHREQ		BIT(0)

#define CRDM_DIVMODE_MASK	0xff

#define CRLP_CSYSREQ_RMASK	0xff
#define CRLP_CSYSREQ_RSHFT	0
#define CRLP_LPOWERHS_MASK	0xff
#define CRLP_LPOWERHS_SHFT	8
#define CRLP_CACTIVE_CMASK	0xff
#define CRLP_CACTIVE_CSHFT	16
#define CRLP_CEN_MASK		0xff
#define CRLP_CEN_SHFT		24

#define CRG11_NOT_PORT		8
#define CRG11_NOT_DOMAIN	16

#define to_crg_clk(p)		container_of(p, struct crg_clk, hw)

struct crg_clk {
	struct clk_hw hw;
	struct crg11 *crg;
	unsigned domain; /* For Divider Only */
};

struct crg11 {
	unsigned id;
	/* Base address of the controller instance */
	void __iomem *reg_base;
	/* Input clock rate to the PLL */
	unsigned long xclk_rate;
	/* Protect reg access */
	spinlock_t lock;
	struct list_head list;
};

static LIST_HEAD(crg11_list);
static void __iomem *main_crg11;

/* For now when we don't have separate power-control driver, use this one */
void crg11_restart(char mode, const char *cmd)
{
	u32 val;

	writel(~CRRRS_RRSTREQ_MASK, main_crg11 + CRG_REG_CRRRS);

	val = readl(main_crg11 + CRG_REG_CRRSC);
	val |= CRRSC_SWRSTM;
	writel(val, main_crg11 + CRG_REG_CRRSC);

	/* invoke software reset */
	writel(CRSWR_SWRSTREQ, main_crg11 + CRG_REG_CRSWR);

	while (1)
	;
}

static unsigned long
_get_gcd(unsigned long a, unsigned long b)
{
	unsigned long t;

	if (a == b)
		return a;

	if (a < b) {
		t = b;
		b = a;
		a = t;
	}

	while (1) {
		t = b;
		b = a % b;
		a = t;
		if (a == 1)
			return 1;
		if (b == 0)
			return a;
	}
}

static unsigned long
crg11_pll_recalc_rate(struct clk_hw *hwclk,
	unsigned long parent_rate)
{
	struct crg_clk *crgclk = to_crg_clk(hwclk);
	void __iomem *reg = crgclk->crg->reg_base;
	u32 m, n, val = readl(reg + CRG_REG_CRPLC);

	/* PLLBYPASS mode */
	if (val & CRPLC_PLLBYPASS)
		return parent_rate;

	m = (val >> CRPLC_FBMODE_SHFT) & CRPLC_FBMODE_MASK;
	m <<= 1;
	/* PLL Stopped */
	if (m == 0)
		return 0;

	n = (val >> CRPLC_PSMODE_SHFT) & CRPLC_PSMODE_MASK;
	n <<= 1;
	if (n == 0)
		n = 1;

	return parent_rate / n * m;
}

static long
crg11_pll_round_rate(struct clk_hw *hwclk,
	unsigned long rate, unsigned long *prate)
{
	unsigned long r, m, n, parent_rate = *prate;

	r = _get_gcd(rate, parent_rate);
	m = rate / r;
	n = parent_rate / r;

	if (m > 126)
		m = 126;
	if (n > 30)
		n = 30;

	return parent_rate / n * m;
}

static int
crg11_pll_set_rate(struct clk_hw *hwclk,
	unsigned long rate, unsigned long parent_rate)
{
	struct crg_clk *crgclk = to_crg_clk(hwclk);
	void __iomem *reg = crgclk->crg->reg_base;
	u32 val = readl(reg + CRG_REG_CRPLC);
	unsigned long r, m, n;

	if (rate == 0) {
		/* Can't simply stop while PLL is in use */
		if (!(val & CRPLC_PLLBYPASS))
			return -EINVAL;

		/* FBMODE := 0 */
		val &= ~(CRPLC_FBMODE_MASK << CRPLC_FBMODE_SHFT);
		writel(val, reg + CRG_REG_CRPLC);
		return 0;
	}

	r = _get_gcd(rate, parent_rate);
	m = rate / r;
	n = parent_rate / r;

	val &= ~(CRPLC_FBMODE_MASK << CRPLC_FBMODE_SHFT);
	val |= ((m & CRPLC_FBMODE_MASK) << CRPLC_FBMODE_SHFT);
	val &= ~(CRPLC_PSMODE_MASK << CRPLC_PSMODE_SHFT);
	val |= ((n & CRPLC_PSMODE_MASK) << CRPLC_PSMODE_SHFT);
	writel(val, reg + CRG_REG_CRPLC);

	/* Wait until PLL Ready */
	do {
		cpu_relax();
	} while (!(readl(reg + CRG_REG_CRRDY) & CRRDY_PLLRDY));

	return 0;
}

const struct clk_ops crg11_pll_ops = {
	.recalc_rate = crg11_pll_recalc_rate,
	.round_rate = crg11_pll_round_rate,
	.set_rate = crg11_pll_set_rate,
};

static void __init crg11_mux_init(struct device_node *node)
{
	u32 phys_addr, clock_rate, id;
	char *xclk, *pllout, *cclk;
	const char *mux_parents[2];
	struct clk_init_data init;
	struct crg_clk *crgclk;
	struct crg11 *crg;
	struct clk *clk;
	int rc;

	rc = of_property_read_u32(node, "index", &id);
	if (WARN_ON(rc))
		return;

	rc = of_property_read_u32(node, "phys_addr", &phys_addr);
	if (WARN_ON(rc))
		return;

	rc = of_property_read_u32(node, "clock_rate", &clock_rate);
	if (WARN_ON(rc))
		return;

	xclk = kzalloc(60, GFP_KERNEL);
	pllout = xclk + 20;
	cclk = xclk + 40;
	mux_parents[0] = pllout;
	mux_parents[1] = xclk;

	crg = kzalloc(sizeof(*crg), GFP_KERNEL);
	crg->reg_base = ioremap(phys_addr, SZ_4K);
	crg->xclk_rate = clock_rate;
	spin_lock_init(&crg->lock);
	crg->id = id;
	if (id == 0) { /* MAIN_CRG11 */
		main_crg11 = crg->reg_base;
		arm_pm_restart = crg11_restart;
	}

	/* Register XCLK */
	snprintf(xclk, 20, "XCLK%d", crg->id);
	clk = clk_register_fixed_rate(NULL, xclk,
		NULL, CLK_IS_ROOT, crg->xclk_rate);
	if (IS_ERR(clk))
		pr_err("%s:%d Error!\n", __func__, __LINE__);
	else
		pr_debug("Registered %s\n", xclk);

	/* Register PLL */
	crgclk = kzalloc(sizeof(*crgclk), GFP_KERNEL);
	if (crgclk == NULL)
		return;

	snprintf(pllout, 20, "PLL_Out%d", crg->id);
	init.name = pllout;
	init.ops = &crg11_pll_ops;
	init.flags = CLK_GET_RATE_NOCACHE;
	init.parent_names = (const char **)&xclk;
	init.num_parents = 1;
	crgclk->hw.init = &init;
	crgclk->domain = CRG11_NOT_DOMAIN;
	crgclk->crg = crg;
	clk = clk_register(NULL, &crgclk->hw);
	if (IS_ERR(clk))
		pr_err("%s:%d Error!\n", __func__, __LINE__);
	else
		pr_debug("Registered %s\n", pllout);

	/* Register MUX CCLK_O */
	snprintf(cclk, 20, "CCLK%d", crg->id);
	clk = clk_register_mux(NULL, cclk, mux_parents, 2, CLK_GET_RATE_NOCACHE,
		crg->reg_base + CRG_REG_CRPLC, 24, 1, 0, &crg->lock);
	if (IS_ERR(clk))
		pr_err("%s:%d Error!\n", __func__, __LINE__);
	else
		pr_debug("Registered %s\n", cclk);

	list_add(&crg->list, &crg11_list);
	kfree(xclk);
}
CLK_OF_DECLARE(crg11_mux, "mb8ac0300,crg11_mux", crg11_mux_init);

static const unsigned crg_cdr[] = {
	1, 2, 3, 4, 6, 8, 9, 12, 18, 24, 27, 36, 54, 72, 108, 216,
};

static unsigned long
crg11_div_recalc_rate(struct clk_hw *hwclk,
	unsigned long parent_rate)
{
	struct crg_clk *crgclk = to_crg_clk(hwclk);
	void __iomem *reg = crgclk->crg->reg_base;
	unsigned dom = crgclk->domain;
	u32 val = readl(reg + CRG_OFF_CRDM + 0x10 * dom);

	return parent_rate / ((val & 0xff) + 1);
}

static long
crg11_div_round_rate(struct clk_hw *hwclk,
	unsigned long rate, unsigned long *prate)
{
	unsigned long div;
	int i;

	if (rate >= *prate)
		return *prate;

	div = *prate / rate;

	for (i = 0; i < ARRAY_SIZE(crg_cdr); i++)
		if (div < crg_cdr[i])
			break;

	return *prate / crg_cdr[i - 1];
}

static int
crg11_div_set_rate(struct clk_hw *hwclk,
	unsigned long rate, unsigned long parent_rate)
{
	struct crg_clk *crgclk = to_crg_clk(hwclk);
	void __iomem *reg = crgclk->crg->reg_base;
	unsigned long div = parent_rate / rate;
	unsigned dom = crgclk->domain;
	u32 val = readl(reg + CRG_OFF_CRDM + 0x10 * dom);

	if (div - 1 == (val & 0xff)) /* Nothing new to do */
		return 0;

	val = (div - 1) & CRDM_DIVMODE_MASK;
	writel(val, reg + CRG_OFF_CRDM + 0x10 * dom);

	writel(CRCDC_DCHREQ, reg + CRG_REG_CRCDC);

	/* Wait until Divider taken */
	do {
		cpu_relax();
	} while (readl(reg + CRG_REG_CRCDC) & CRCDC_DCHREQ);

	return 0;
}

const struct clk_ops crg11_div_ops = {
	.recalc_rate = crg11_div_recalc_rate,
	.round_rate = crg11_div_round_rate,
	.set_rate = crg11_div_set_rate,
};

static void __init crg11_gate_init(struct device_node *node)
{
	char *ungclk, *clkp, *cclk;
	struct clk_init_data init;
	struct crg11 *crg = NULL;
	u32 cntrlr, domain, port;
	struct crg_clk *crgclk;
	struct clk *clk;
	bool found;
	int rc;

	rc = of_property_read_u32(node, "cntrlr", &cntrlr);
	if (WARN_ON(rc))
		return;
	rc = of_property_read_u32(node, "domain", &domain);
	if (WARN_ON(rc))
		return;
	rc = of_property_read_u32(node, "port", &port);
	if (WARN_ON(rc))
		return;

	found = false;
	list_for_each_entry(crg, &crg11_list, list) {
		if (crg->id == cntrlr) {
			found = true;
			break;
		}
	}
	if (!found) {
		pr_err("CRG11_%d MUX not yet populated!\n", cntrlr);
		return;
	}

	ungclk = kzalloc(60, GFP_KERNEL);
	clkp = ungclk + 20;
	cclk = ungclk + 40;

	/* Look for UngatedCLK (port := 8) */
	snprintf(ungclk, 20, "CLK%d_%X_8", crg->id, domain);
	clk = __clk_lookup(ungclk);
	if (clk == NULL) { /* First user of the domain */
		snprintf(cclk, 20, "CCLK%d", crg->id);
		clk = __clk_lookup(cclk);
		if (WARN_ON(clk == NULL)) {
			pr_err("CCLK%d not found!\n", crg->id);
			return;
		}

		crgclk = kzalloc(sizeof(*crgclk), GFP_KERNEL);
		if (!crgclk)
			return;
		init.name = ungclk;
		init.ops = &crg11_div_ops;
		init.flags = CLK_GET_RATE_NOCACHE;
		init.parent_names = (const char **)&cclk;
		init.num_parents = 1;
		crgclk->hw.init = &init;
		crgclk->domain = domain;
		crgclk->crg = crg;
		clk = clk_register(NULL, &crgclk->hw);
		if (IS_ERR(clk))
			pr_err("%s:%d Error!\n", __func__, __LINE__);
		else
			pr_debug("Registered %s\n", ungclk);
		clk_register_clkdev(clk, ungclk, NULL);
	}
	if (port == 8)
		goto exit;

	snprintf(clkp, 20, "CLK%d_%X_%d", crg->id, domain, port);
	pr_debug("\t\t%s\n", clkp);
	clk = clk_register_gate(NULL, clkp, ungclk,
		CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
		crg->reg_base + CRG_OFF_CRLP + 0x10 * domain,
		CRLP_CSYSREQ_RSHFT + port, 0, &crg->lock);
	if (IS_ERR(clk))
		pr_err("%s:%d Error!\n", __func__, __LINE__);
	else
		pr_debug("Registered %s\n", clkp);

	clk_register_clkdev(clk, clkp, NULL);

exit:
	of_clk_add_provider(node, of_clk_src_simple_get, clk);
	kfree(ungclk);
}
CLK_OF_DECLARE(crg11_gate, "mb8ac0300,crg11_gate", crg11_gate_init);
