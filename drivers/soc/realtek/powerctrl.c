// SPDX-License-Identifier: GPL-2.0-only
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>

#define REG_SYS_GPU_SRAM_PWR4	0x3a4
#define REG_SYS_GPU_SRAM_PWR5	0x3ac
#define REG_SYS_POWER_CTRL	0x400

#define RTD1295_SYS_GPU_SRAM_PWR5_WRITE_DATA	BIT(0)
#define RTD1295_SYS_GPU_SRAM_PWR5_GPU_SRAM_INT	BIT(2)

#define RTD1295_SYS_POWER_CTRL_ISO_GPU	BIT(1)

struct rtd_powerctrl {
	struct device *dev;
	struct regmap *regmap;
	struct genpd_onecell_data genpd_data;
	struct generic_pm_domain *domains[];
};

struct rtd_powerctrl_domain {
	struct generic_pm_domain genpd;
	struct rtd_powerctrl *powerctrl;
};

#define to_rtd_pd(gpd) container_of(gpd, struct rtd_powerctrl_domain, genpd)

static int rtd1295_gpu_sram_set_power(struct rtd_powerctrl_domain *pd,
	bool enabled)
{
	struct regmap *regmap = pd->powerctrl->regmap;
	unsigned int timeout_us = 500;
	unsigned int val;
	int ret;

	val = (0x3 << 8);
	if (!enabled)
		val |= BIT(0);
	ret = regmap_write(regmap, REG_SYS_GPU_SRAM_PWR4, val);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(regmap, REG_SYS_GPU_SRAM_PWR5, val,
		val != RTD1295_SYS_GPU_SRAM_PWR5_GPU_SRAM_INT, 1, timeout_us);
	if (ret)
		return ret;

	val = RTD1295_SYS_GPU_SRAM_PWR5_GPU_SRAM_INT;
	return regmap_write(regmap, REG_SYS_GPU_SRAM_PWR5, val);
}

static int rtd1295_gpu_sram_power_on(struct generic_pm_domain *domain)
{
	struct rtd_powerctrl_domain *pd = to_rtd_pd(domain);

	dev_info(pd->powerctrl->dev, "%s power on\n", domain->name);

	return rtd1295_gpu_sram_set_power(pd, true);
}

static int rtd1295_gpu_sram_power_off(struct generic_pm_domain *domain)
{
	struct rtd_powerctrl_domain *pd = to_rtd_pd(domain);

	dev_info(pd->powerctrl->dev, "%s power off\n", domain->name);

	return rtd1295_gpu_sram_set_power(pd, false);
}

static int rtd1295_gpu_sram_is_off(struct rtd_powerctrl_domain *pd, bool *off)
{
	unsigned int val;
	int ret;

	ret = regmap_read(pd->powerctrl->regmap, 0x394 + 0x10, &val);
	if (ret)
		return ret;
	*off = !!(val & BIT(0));
	return 0;
}

static int rtd1295_gpu_power_on(struct generic_pm_domain *domain)
{
	struct rtd_powerctrl_domain *pd = to_rtd_pd(domain);
	unsigned int val;
	int ret;

	dev_info(pd->powerctrl->dev, "%s power on\n", domain->name);

	ret = regmap_read(pd->powerctrl->regmap, REG_SYS_POWER_CTRL, &val);
	if (ret)
		return ret;

	val &= ~RTD1295_SYS_POWER_CTRL_ISO_GPU;
	return regmap_write(pd->powerctrl->regmap, REG_SYS_POWER_CTRL, val);
}

static int rtd1295_gpu_power_off(struct generic_pm_domain *domain)
{
	struct rtd_powerctrl_domain *pd = to_rtd_pd(domain);
	unsigned int val;
	int ret;

	dev_info(pd->powerctrl->dev, "%s power off\n", domain->name);

	ret = regmap_read(pd->powerctrl->regmap, REG_SYS_POWER_CTRL, &val);
	if (ret)
		return ret;

	val |= RTD1295_SYS_POWER_CTRL_ISO_GPU;
	return regmap_write(pd->powerctrl->regmap, REG_SYS_POWER_CTRL, val);
}

static int rtd1295_gpu_is_off(struct rtd_powerctrl_domain *pd, bool *off)
{
	unsigned int val;
	int ret;

	ret = regmap_read(pd->powerctrl->regmap, REG_SYS_POWER_CTRL, &val);
	if (ret)
		return ret;
	*off = !!(val & RTD1295_SYS_POWER_CTRL_ISO_GPU);
	return 0;
}

static const struct of_device_id rtd_powerctrl_dt_ids[] = {
	 { .compatible = "realtek,rtd1295-powerctrl" },
	 { }
};

static int rtd_powerctrl_probe(struct platform_device *pdev)
{
	struct rtd_powerctrl_domain *pd;
	struct rtd_powerctrl *s;
	bool is_off;
	int i, ret;

	s = devm_kzalloc(&pdev->dev, struct_size(s, domains, 2), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->dev = &pdev->dev;
	s->genpd_data.domains = s->domains;
	s->genpd_data.num_domains = 2;

	s->regmap = syscon_node_to_regmap(pdev->dev.of_node->parent);
	if (IS_ERR(s->regmap))
		return PTR_ERR(s->regmap);

	pd = devm_kzalloc(&pdev->dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->powerctrl = s;
	pd->genpd.name = "iso_gpu";
	pd->genpd.power_on  = rtd1295_gpu_power_on;
	pd->genpd.power_off = rtd1295_gpu_power_off;
	ret = rtd1295_gpu_is_off(pd, &is_off);
	if (ret)
		return ret;
	ret = pm_genpd_init(&pd->genpd, NULL, is_off);
	if (ret)
		return ret;
	s->domains[0] = &pd->genpd;

	pd = devm_kzalloc(&pdev->dev, sizeof(*pd), GFP_KERNEL);
	if (!pd) {
		pm_genpd_remove(s->domains[0]);
		return -ENOMEM;
	}

	pd->powerctrl = s;
	pd->genpd.name = "gpu_sram";
	pd->genpd.power_on  = rtd1295_gpu_sram_power_on;
	pd->genpd.power_off = rtd1295_gpu_sram_power_off;
	ret = rtd1295_gpu_sram_is_off(pd, &is_off);
	if (ret) {
		pm_genpd_remove(s->domains[0]);
		return ret;
	}
	ret = pm_genpd_init(&pd->genpd, NULL, is_off);
	if (ret) {
		pm_genpd_remove(s->domains[0]);
		return ret;
	}
	s->domains[1] = &pd->genpd;

	ret = pm_genpd_add_subdomain(s->domains[1], s->domains[0]);
	if (ret) {
		dev_err(&pdev->dev, "adding %s subdomain %s failed (%d)\n",
			s->domains[1]->name, s->domains[0]->name, ret);
		pm_genpd_remove(s->domains[1]);
		pm_genpd_remove(s->domains[0]);
		return ret;
	}

	for (i = 0; i < s->genpd_data.num_domains; i++) {
		dev_info(&pdev->dev, "%s is %s\n", s->domains[i]->name, (s->domains[i]->status == GPD_STATE_ACTIVE) ? "active" : "off");
	}

	ret = of_genpd_add_provider_onecell(pdev->dev.of_node, &s->genpd_data);
	if (ret) {
		dev_err(&pdev->dev, "failed to add provider (%d)", ret);
		pm_genpd_remove_subdomain(s->domains[1], s->domains[0]);
		for (i = 0; i < s->genpd_data.num_domains; i++)
			pm_genpd_remove(s->domains[i]);
		return ret;
	}

	dev_info(&pdev->dev, "probed\n");

	return 0;
}

static struct platform_driver rtd_powerctrl_driver = {
	.probe = rtd_powerctrl_probe,
	.driver = {
		.name = "rtd1295-powerctrl",
		.of_match_table	= rtd_powerctrl_dt_ids,
	},
};
builtin_platform_driver(rtd_powerctrl_driver);
