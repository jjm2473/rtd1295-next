/*
 * Copyright (C) 2013-2015 FUJITSU SEMICONDUCTOR LIMITED
 * Copyright (C) 2015-2016 Socionext Inc.
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
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk-provider.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/scb_mhu_api.h>
#include <linux/mailbox_client.h>

#define to_crg_clk(p) container_of(p, struct crg_clk, hw)

extern int skip_mhu;

struct hack_rate {
	unsigned clk_id;
	unsigned long rate;
	int gated;
};

static struct hack_rate brate[] = {
	{(0<<8)|(0<<4)|(0<<0), 250000000, 0},
	{(0<<8)|(0<<4)|(1<<0), 250000000, 0},
	{(0<<8)|(0<<4)|(2<<0), 250000000, 0},
	{(0<<8)|(0<<4)|(4<<0), 250000000, 0},
	{(0<<8)|(0<<4)|(5<<0), 250000000, 0},
	{(0<<8)|(1<<4)|(0<<0), 125000000, 0},
	{(0<<8)|(1<<4)|(1<<0), 125000000, 0},
	{(0<<8)|(1<<4)|(8<<0), 125000000, 0},
	{(0<<8)|(2<<4)|(0<<0), 62500000, 0},
	{(0<<8)|(2<<4)|(1<<0), 62500000, 0},
	{(0<<8)|(2<<4)|(2<<0), 62500000, 0},
	{(0<<8)|(2<<4)|(4<<0), 62500000, 0},
	{(0<<8)|(2<<4)|(5<<0), 62500000, 0},
	{(0<<8)|(2<<4)|(8<<0), 62500000, 0},
	{(0<<8)|(6<<4)|(8<<0), 31250000, 0},
	{(0<<8)|(7<<4)|(0<<0), 62500000, 0},
	{(0<<8)|(8<<4)|(0<<0), 2000000, 0},
	{(0<<8)|(10<<4)|(0<<0), 25000000, 0},
	{(0<<8)|(10<<4)|(1<<0), 1200000000, 0},
	{(1<<8)|(0<<4)|(0<<0), 1600000000, 0},
	{(2<<8)|(0<<4)|(0<<0), 400000000, 0},
	{(2<<8)|(0<<4)|(8<<0), 400000000, 0},
	{(2<<8)|(1<<4)|(3<<0), 400000000, 0},
	{(2<<8)|(1<<4)|(4<<0), 400000000, 0},
	{(2<<8)|(2<<4)|(0<<0), 200000000, 0},
	{(2<<8)|(2<<4)|(3<<0), 200000000, 0},
	{(2<<8)|(2<<4)|(7<<0), 200000000, 0},
	{(2<<8)|(3<<4)|(0<<0), 200000000, 0},
	{(2<<8)|(3<<4)|(3<<0), 200000000, 0},
	{(2<<8)|(3<<4)|(4<<0), 200000000, 0},
	{(2<<8)|(3<<4)|(5<<0), 200000000, 0},
	{(2<<8)|(3<<4)|(6<<0), 200000000, 0},
	{(2<<8)|(4<<4)|(0<<0), 100000000, 0},
	{(2<<8)|(4<<4)|(1<<0), 100000000, 0},
	{(2<<8)|(5<<4)|(0<<0), 100000000, 0},
	{(2<<8)|(5<<4)|(3<<0), 100000000, 0},
	{(2<<8)|(5<<4)|(4<<0), 100000000, 0},
	{(2<<8)|(5<<4)|(5<<0), 100000000, 0},
	{(2<<8)|(7<<4)|(0<<0), 800000000, 0},
	{(2<<8)|(8<<4)|(1<<0), 266666666, 0},
	{(2<<8)|(9<<4)|(0<<0), 50000000, 0},
	{(2<<8)|(11<<4)|(0<<0), 0, 0},
};

struct crg_clk {
	struct clk_hw hw;
	u8 cntrlr, domain, port;
};

static int crg_gate_control(struct clk_hw *hw, int en)
{
	struct crg_clk *crgclk = to_crg_clk(hw);
	struct cmd_periclk_control cmd;
	struct completion got_rsp;
	int ret;

	cmd.payload_size = sizeof(cmd);
	cmd.cntrlr = crgclk->cntrlr;
	cmd.domain = crgclk->domain;
	cmd.port = crgclk->port;
	cmd.en = en;
	/* Port-8 is UngatedCLK */
	if (cmd.port == 8)
		return en ? 0 : -EINVAL;

	mbox_dbg("%s:%d CMD Pyld-%u Cntrlr-%u Dom-%u Port-%u En-%u}\n",
		__func__, __LINE__, cmd.payload_size, cmd.cntrlr,
		cmd.domain, cmd.port, cmd.en);

	init_completion(&got_rsp);
	ret = mhu_send_packet(CMD_PERI_CLOCK_GATE_SET_REQ,
					&cmd, sizeof(cmd), &got_rsp);
	if (ret < 0) {
		pr_err("%s:%d failed!\n", __func__, __LINE__);
		return ret;
	}
	if (ret)
		wait_for_completion(&got_rsp);

	mbox_dbg("%s:%d REP Pyld-%u Cntrlr-%u Dom-%u Port-%u En-%u}\n",
		__func__, __LINE__, cmd.payload_size, cmd.cntrlr,
		cmd.domain, cmd.port, cmd.en);

	/* If the request was rejected */
	if (cmd.en != en)
		ret = -EINVAL;
	else
		ret = 0;

	return ret;
}

static int crg_port_prepare(struct clk_hw *hw)
{
	return skip_mhu ? 0 : crg_gate_control(hw, 1);
}

static void crg_port_unprepare(struct clk_hw *hw)
{
	if (!skip_mhu)
		crg_gate_control(hw, 0);
}

#define HDMI_CRG 594000		/* HDMI_CRG (KHz) */

static int crg_rate_control(struct clk_hw *hw, int set, unsigned long *rate)
{
	struct crg_clk *crgclk = to_crg_clk(hw);
	struct cmd_periclk_control cmd;
	struct completion got_rsp;
	int code, ret;
	int ratio, freq_khz, rounddown, roundup;

	cmd.payload_size = sizeof(cmd);
	cmd.cntrlr = crgclk->cntrlr;
	cmd.domain = crgclk->domain;
	cmd.port = crgclk->port;
	cmd.freqency = *rate;

	if (set) {
		code = CMD_PERI_CLOCK_RATE_SET_REQ;
		/* adjust freqency : roundup or rounddown */
		/* since MHU always roundups */
		freq_khz = cmd.freqency;
		freq_khz /= 1000;
		ratio = HDMI_CRG / freq_khz;	/* 1/n */
		roundup = HDMI_CRG / ratio;
		rounddown = HDMI_CRG / (ratio + 1);
		if((roundup - freq_khz) < (freq_khz - rounddown))
			cmd.freqency = roundup * 1000;
		else
			cmd.freqency = rounddown * 1000;
		mbox_dbg("%s:%d CMD Pyld-%u Cntrlr-%u Dom-%u Port-%u Rate-SET %lluHz}\n",
			__func__, __LINE__, cmd.payload_size, cmd.cntrlr,
			cmd.domain, cmd.port, cmd.freqency);
	} else {
		code = CMD_PERI_CLOCK_RATE_GET_REQ;
		mbox_dbg("%s:%d CMD Pyld-%u Cntrlr-%u Dom-%u Port-%u Rate-GET}\n",
			__func__, __LINE__, cmd.payload_size, cmd.cntrlr,
			cmd.domain, cmd.port);
	}

	if (skip_mhu) {
		int i;
		unsigned clk_id;
		clk_id = (cmd.cntrlr << 8)|(cmd.domain << 4)|(cmd.port << 0);
		for (i = 0; i < ARRAY_SIZE(brate); i++) {
			mbox_dbg("%s:%d brate[i].clk_id=%x clk_id=%x\n",
				__func__, __LINE__, brate[i].clk_id, clk_id);
			if (brate[i].clk_id == clk_id) {
				if (set)
					brate[i].rate = *rate;
				else
					*rate = brate[i].rate;
				return 0;
			}
		}
		if (set)
			return -EINVAL;
		mbox_dbg("%s:%d Clock Cntrlr-%u Dom-%u Port-%u not found\n",
			__func__, __LINE__, cmd.cntrlr,	cmd.domain, cmd.port);
		*rate = 150000000; /* Random value */
		return 0;
	}

	init_completion(&got_rsp);
	ret = mhu_send_packet(code, &cmd, sizeof(cmd), &got_rsp);
	if (ret < 0) {
		pr_err("%s:%d failed!\n", __func__, __LINE__);
		return ret;
	}
	if (ret)
		wait_for_completion(&got_rsp);

	if (set)
		mbox_dbg("%s:%d REP Pyld-%u Cntrlr-%u Dom-%u Port-%u Rate-SET %lluHz}\n",
			__func__, __LINE__, cmd.payload_size, cmd.cntrlr,
			cmd.domain, cmd.port, cmd.freqency);
	else
		mbox_dbg("%s:%d REP Pyld-%u Cntrlr-%u Dom-%u Port-%u Rate-GOT %lluHz}\n",
			__func__, __LINE__, cmd.payload_size, cmd.cntrlr,
			cmd.domain, cmd.port, cmd.freqency);

	*rate = cmd.freqency;
	return 0;
}

static unsigned long crg_port_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	unsigned long rate;

	crg_rate_control(hw, 0, &rate);

	return rate;
}

static long
crg_port_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *pr)
{
	return rate;
}

static int
crg_port_set_rate(struct clk_hw *hw,
	unsigned long rate, unsigned long parent_rate)
{
	return crg_rate_control(hw, 1, &rate);
}

const struct clk_ops crg_port_ops = {
	.prepare = crg_port_prepare,
	.unprepare = crg_port_unprepare,
	.recalc_rate = crg_port_recalc_rate,
	.round_rate = crg_port_round_rate,
	.set_rate = crg_port_set_rate,
};

static void __init crg_port_init(struct device_node *node)
{
	struct clk_init_data init;
	u32 cntrlr, domain, port;
	struct crg_clk *crgclk;
	struct clk *clk;
	char clkp[20];
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

	if (port > 7)
		snprintf(clkp, 20, "UngatedCLK%d_%X", cntrlr, domain);
	else
		snprintf(clkp, 20, "CLK%d_%X_%d", cntrlr, domain, port);

	clk = __clk_lookup(clkp);
	if (clk)
		return;

	crgclk = kzalloc(sizeof(*crgclk), GFP_KERNEL);
	if (!crgclk)
		return;
	init.name = clkp;
	init.num_parents = 0;
	init.ops = &crg_port_ops;
	init.flags = CLK_IS_ROOT; /* | CLK_GET_RATE_NOCACHE */
	crgclk->hw.init = &init;
	crgclk->cntrlr = cntrlr;
	crgclk->domain = domain;
	crgclk->port = port;
	clk = clk_register(NULL, &crgclk->hw);
	if (IS_ERR(clk))
		pr_err("%s:%d Error!\n", __func__, __LINE__);
	else
		pr_debug("Registered %s\n", clkp);

	of_clk_add_provider(node, of_clk_src_simple_get, clk);
	clk_register_clkdev(clk, clkp, NULL);
}
CLK_OF_DECLARE(crg11_gate, "mb86s70,crg11_gate", crg_port_init);

struct cl_clk {
	struct clk_hw hw;
	int cluster;
};

#define to_clc_clk(clc) container_of(clc, struct cl_clk, hw)

void mhu_cluster_rate(int cluster, unsigned long *rate, int get)
{
	struct cmd_cpu_control_rate cmd;
	struct completion got_rsp;
	int code, ret;

	cmd.payload_size = sizeof(cmd);
	cmd.cluster_class = 0;
	cmd.cluster_id = cluster;
	cmd.cpu_id = 0;
	cmd.freqency = *rate;

	if (skip_mhu) {
		if (cmd.cluster_id == 0)
			*rate = 1600000000;
		else
			*rate = 800000000;
		return;
	}

	if (get)
		code = CMD_CPU_CLOCK_RATE_GET_REQ;
	else
		code = CMD_CPU_CLOCK_RATE_SET_REQ;

	mbox_dbg("%s:%d CMD Pyld-%u Cl_Class-%u CL_ID-%u CPU_ID-%u Freq-%llu}\n",
		__func__, __LINE__, cmd.payload_size, cmd.cluster_class,
		cmd.cluster_id,	cmd.cpu_id, cmd.freqency);

	init_completion(&got_rsp);
	ret = mhu_send_packet(code, &cmd, sizeof(cmd), &got_rsp);
	if (ret < 0) {
		pr_err("%s:%d failed!\n", __func__, __LINE__);
		return;
	}
	if (ret)
		wait_for_completion(&got_rsp);

	mbox_dbg("%s:%d REP Pyld-%u Cl_Class-%u CL_ID-%u CPU_ID-%u Freq-%llu}\n",
		__func__, __LINE__, cmd.payload_size, cmd.cluster_class,
		cmd.cluster_id,	cmd.cpu_id, cmd.freqency);

	*rate = cmd.freqency;
}
EXPORT_SYMBOL_GPL(mhu_cluster_rate);

static unsigned long clc_recalc_rate(struct clk_hw *hw,
		unsigned long unused)
{
	unsigned long rate;

	mhu_cluster_rate(to_clc_clk(hw)->cluster, &rate, 1);
	return rate;
}

static long clc_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *unused)
{
	return rate;
}

static int clc_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long unused)
{
	unsigned long res = rate;

	mhu_cluster_rate(to_clc_clk(hw)->cluster, &rate, 0);

	return (res == rate) ? 0 : -EINVAL;
}

static struct clk_ops clk_clc_ops = {
	.recalc_rate = clc_recalc_rate,
	.round_rate = clc_round_rate,
	.set_rate = clc_set_rate,
};

struct clk *mb86s70_clclk_register(const char *name, int cluster_id)
{
	struct clk_init_data init;
	struct cl_clk *clc;
	struct clk *clk;

	clc = kzalloc(sizeof(*clc), GFP_KERNEL);
	if (!clc) {
		pr_err("could not allocate cl_clk\n");
		return ERR_PTR(-ENOMEM);
	}

	clc->hw.init = &init;
	clc->cluster = cluster_id;

	init.name = name;
	init.ops = &clk_clc_ops;
	init.flags = CLK_IS_ROOT | CLK_GET_RATE_NOCACHE;
	init.num_parents = 0;

	clk = clk_register(NULL, &clc->hw);
	if (!IS_ERR_OR_NULL(clk))
		return clk;

	pr_err("clk register failed\n");
	kfree(clc);
	return NULL;
}

void __init mb86s70_clclk_of_init(void)
{
	struct device_node *node = NULL;
	char name[14] = "cpu-cluster.";
	int cluster_id = 0, len;
	struct clk *clk;
	const u32 *val;

	if (!of_find_compatible_node(NULL, NULL, "fujitsu,mhu"))
		return;

	while ((node = of_find_node_by_name(node, "cluster"))) {
		val = of_get_property(node, "reg", &len);
		if (val && len == 4)
			cluster_id = be32_to_cpup(val);

		name[12] = cluster_id + '0';
		clk = mb86s70_clclk_register(name, cluster_id);
		if (IS_ERR(clk))
			return;

		pr_debug("Registered clock '%s'\n", name);
		clk_register_clkdev(clk, NULL, name);
	}
}
CLK_OF_DECLARE(cpu_clk, "fujitsu,mhu", mb86s70_clclk_of_init);
