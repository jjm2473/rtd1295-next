/*
 * mb86s7x generic powerdomain support
 * Copyright (C) 2014 Linaro, Ltd  Andy Green <andy.green@linaro.org>
 *
 * based on -->
 *
 * Exynos Generic power domain support.
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Implementation of Exynos specific power domain control which is used in
 * conjunction with runtime-pm. Support for both device-tree and non-device-tree
 * based power domain support is included.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
/* #define DEBUG */
#include <linux/io.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/pm_domain.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/sched.h>
#include <linux/scb_mhu_api.h>

/*
 * mb86s7x specific wrapper around the generic power domain
 */
struct mb86s7x_pm_domain {
	struct generic_pm_domain pd;
	char const *name;
	int powerdomain_index;
};

static int mb86s7x_pd_power(struct generic_pm_domain *domain, bool power_on)
{
	struct mb86s7x_pm_domain *pd;
	struct cmd_powerdomain cmd;
	struct completion got_rsp;
	int ret;

	pd = container_of(domain, struct mb86s7x_pm_domain, pd);

	pr_info("%s: domain %s <- %d\n", __func__, pd->name, power_on);

	if (skip_mhu)
		return 0;

	cmd.payload_size = sizeof(cmd);
	cmd.powerdomain_index = pd->powerdomain_index;
	cmd.state = power_on;

	if (pd->powerdomain_index >= 0) {
		pr_info("First powerdomain index1 set :%d .\n",
			pd->powerdomain_index);
		init_completion(&got_rsp);

		ret = mhu_send_packet(CMD_POWERDOMAIN_SET_REQ,
				&cmd, sizeof(cmd), &got_rsp);
		if (ret < 0) {
			pr_err("%s:%d failed to set powerdomain\n"
				, __func__, __LINE__);
			return ret;
		}
		if (ret)
			wait_for_completion(&got_rsp);
	}

	return 0;
}

static int mb86s7x_pd_power_on(struct generic_pm_domain *domain)
{
	return mb86s7x_pd_power(domain, true);
}

static int mb86s7x_pd_power_off(struct generic_pm_domain *domain)
{
	return mb86s7x_pd_power(domain, false);
}

static bool mb86s7x_pd_active_wakeup(struct device *dev)
{
	bool (*active_wakeup)(struct device *dev);
	bool result;

	active_wakeup = dev_gpd_data(dev)->ops.active_wakeup;
	result = active_wakeup ? active_wakeup(dev) : device_may_wakeup(dev);

	return result;
}

static struct genpd_onecell_data mb86s7x_pd;

static int mb86s7x_add_sub_domain(struct device_node *master,
			struct device_node *sub)
{
	struct mb86s7x_pm_domain *master_pd, *sub_pd;
	int ret;

	sub_pd = platform_get_drvdata(of_find_device_by_node(sub));
	master_pd = platform_get_drvdata(of_find_device_by_node(master));

	pr_debug("sub-domain:%d, master-domain:%d.\n"
		, sub_pd->powerdomain_index, master_pd->powerdomain_index);

	while (1) {
		ret = pm_genpd_add_subdomain(&master_pd->pd, &sub_pd->pd);
		if (ret != -EAGAIN)
			break;
		cond_resched();
	}

	return 0;
}

static int mb86s7x_pm_notifier_call(struct notifier_block *nb,
				    unsigned long event, void *data)
{
	struct device *dev = data;

	switch (event) {
	case BUS_NOTIFY_BIND_DRIVER:
		pm_genpd_dev_need_restore(dev, true);
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block platform_nb = {
	.notifier_call = mb86s7x_pm_notifier_call,
};

static __init int mb86s7x_pm_init_power_domain(void)
{
	struct device_node *np, *child = NULL, *master;
	struct cmd_powerdomain cmd;
	struct completion got_rsp;
	int ret, pd_num;
	struct platform_device *pdev;

	pr_debug("mb86s7x power domain initialization start.\n");

	np = of_find_compatible_node(NULL, NULL, "fujitsu,mb86s7x-pd");
	if (np == NULL) {
		pr_info("No mb86s7x power domains\n");
		return -ENOMEM;
	}

	/* populate power-domain cell device */
	pdev = of_find_device_by_node(np);
	ret = of_platform_populate(np, NULL, NULL, &pdev->dev);
	if (ret) {
		pr_err("%s: missing power-domain-cell device\n", __func__);
		return -ENOMEM;
	}

	/* count for power domain number */
	pd_num = 0;
	for  (child = of_get_next_child(np, NULL); child;
			child = of_get_next_child(np, child))
		pd_num++;

	pr_debug("mb86s7x power domain count is %d.\n", pd_num);
	mb86s7x_pd.domains = kmalloc(pd_num*sizeof(struct generic_pm_domain *),
			GFP_KERNEL);
	mb86s7x_pd.domain_num = pd_num;

	pd_num = 0;
	for  (child = of_get_next_child(np, NULL); child;
					child = of_get_next_child(np, child)) {
		struct mb86s7x_pm_domain *pd;
		struct platform_device *child_pdev =
				of_find_device_by_node(child);

		pd = kzalloc(sizeof(*pd), GFP_KERNEL);
		if (!pd) {
			pr_err("%s: failed to allocate memory for domain-cell.\n"
				, __func__);
			return -ENOMEM;
		}
		pd->pd.name = kstrdup(np->name, GFP_KERNEL);
		pd->pd.power_off = mb86s7x_pd_power_off;
		pd->pd.power_on = mb86s7x_pd_power_on;
		pd->pd.of_node = np; /*put parent's node*/
		pd->name = pd->pd.name;
		if (of_property_read_u32(child, "index"
					, &pd->powerdomain_index))
			pd->powerdomain_index = -1;
		mb86s7x_pd.domains[pd_num] = &pd->pd;

		/* sned power domain control cmd via mhu */
		cmd.payload_size = sizeof(cmd);
		cmd.powerdomain_index = pd->powerdomain_index;
		cmd.state = 0;
		if (!skip_mhu && pd->powerdomain_index >= 0) {
			init_completion(&got_rsp);

			ret = mhu_send_packet(CMD_POWERDOMAIN_GET_REQ,
					&cmd, sizeof(cmd), &got_rsp);
			if (ret < 0)
				pr_err("%s:%d failed to get SCB version\n"
							, __func__, __LINE__);
			if (ret)
				wait_for_completion(&got_rsp);
		}

		pm_genpd_init(&pd->pd, NULL, !cmd.state);

		pd->pd.dev_ops.active_wakeup = mb86s7x_pd_active_wakeup;

		platform_set_drvdata(child_pdev, pd);

		pr_debug("power domain %s starting.\n", child->name);
		pd_num++;
	}

	/* register sub-domain if necessary */
	for  (child = of_get_next_child(np, NULL); child;
				child = of_get_next_child(np, child)) {
		master = of_parse_phandle(child, "master-domain-cell", 0);
		if (master == NULL)
			continue;

		mb86s7x_add_sub_domain(master, child);
	}

	if (of_genpd_add_provider(np, of_genpd_xlate_onecell, &mb86s7x_pd)) {
		pr_err("mb86s7x power domain initialization failed.\n");
		return -1;
	}
	bus_register_notifier(&platform_bus_type, &platform_nb);

	pr_debug("mb86s7x power domain initialization end.\n");
	return 0;
}
arch_initcall(mb86s7x_pm_init_power_domain);

int __init mb86s7x_pm_late_initcall(void)
{
	pm_genpd_poweroff_unused();
	return 0;
}
