/*
 * Copyright (C) 2014-2015 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/cpu.h>
#include <linux/clk-provider.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/topology.h>
#include <linux/cpufreq.h>
#include <linux/thermal.h>
#include <linux/cpu_cooling.h>
#include <linux/mailbox_client.h>
#include <linux/platform_device.h>
#include <linux/scb_mhu_api.h>

static DEFINE_PER_CPU(struct thermal_cooling_device *, cdev);
static DEFINE_PER_CPU(struct thermal_zone_device *, tzd);

static int scb_get_temp(void *devdata, long *temp)
{
	struct mb86s7x_thermal_info cmd;
	struct completion got_rsp;
	int ret, id = (int)devdata;

	cmd.payload_size = sizeof(cmd);
	init_completion(&got_rsp);
	/*
	 * We could read temperatures N times seperated by a few ms.
	 * and then average them out before reporting 'final' value.
	 * But for now we assume SCB provides averaged/calibrated readings.
	 */
	ret = mhu_send_packet(CMD_THERMAL_INFO_REQ,
				&cmd, sizeof(cmd), &got_rsp);
	if (ret < 0) {
		pr_err("%s:%d failed!\n", __func__, __LINE__);
		return ret;
	}
	if (ret)
		wait_for_completion(&got_rsp);

	*temp = cmd.temp[id];
	pr_debug("%s:%d CPU-%d Tmp=%ld\n", __func__, __LINE__, id, *temp);

	return 0;
}

static int scb_thermal_probe(struct platform_device *pdev)
{
	u32 ca15_0_id, ca7_0_id, ca15_1_id, ca7_1_id;
	struct cpumask clip_cpus;
	char cpu[12] = "/cpus/cpu@0";
	int i;

	of_property_read_u32(pdev->dev.of_node, "ca15-cpu0-id", &ca15_0_id);
	of_property_read_u32(pdev->dev.of_node, "ca15-cpu1-id", &ca15_1_id);
	of_property_read_u32(pdev->dev.of_node, "ca7-cpu0-id", &ca7_0_id);
	of_property_read_u32(pdev->dev.of_node, "ca7-cpu1-id", &ca7_1_id);

	per_cpu(tzd, 0) = thermal_zone_of_sensor_register(&pdev->dev, 0x0,
				(void *)ca15_0_id, scb_get_temp, NULL);
	per_cpu(tzd, 1) = thermal_zone_of_sensor_register(&pdev->dev, 0x1,
				(void *)ca15_1_id, scb_get_temp, NULL);
	per_cpu(tzd, 2) = thermal_zone_of_sensor_register(&pdev->dev, 0x100,
				(void *)ca7_0_id, scb_get_temp, NULL);
	per_cpu(tzd, 3) = thermal_zone_of_sensor_register(&pdev->dev, 0x101,
				(void *)ca7_1_id, scb_get_temp, NULL);

	for_each_possible_cpu(i) {
		struct device *cpu_dev = get_cpu_device(i);

		cpu[10] = '0' + i;
		cpu_dev->of_node = of_find_node_by_path(cpu);
		pr_err("%s:%d cpu=%u dev=%p np=%p %s\n",
			__func__, __LINE__, i, cpu_dev, cpu_dev->of_node,
			cpu_dev->of_node ? cpu_dev->of_node->full_name : "NULL");
		cpumask_clear(&clip_cpus);
		cpumask_set_cpu(i, &clip_cpus);
		per_cpu(cdev, i) = of_cpufreq_cooling_register(cpu_dev->of_node, &clip_cpus);
	}

	return 0;
}

static int scb_thermal_remove(struct platform_device *pdev)
{
	int i;

	for_each_possible_cpu(i) {
		if (!IS_ERR(per_cpu(cdev, i)))
			cpufreq_cooling_unregister(per_cpu(cdev, i));
		if (!IS_ERR(per_cpu(tzd, i)))
			thermal_zone_of_sensor_unregister(&pdev->dev, per_cpu(tzd, i));
	}

	return 0;
}

static const struct of_device_id of_scb_thermal_match[] = {
	{.compatible = "fujitsu,scb-thermal-s70" },
	{ }, /* Sentinel */
};
MODULE_DEVICE_TABLE(of, of_scb_thermal_match);

static struct platform_driver scb_thermal_driver = {
	.probe = scb_thermal_probe,
	.remove = scb_thermal_remove,
	.driver = {
			.name = "fujitsu-scb-thermal-s70",
			.of_match_table	= of_scb_thermal_match,
	},
};

static int __init scb_thermal_cooling_init(void)
{
	return platform_driver_register(&scb_thermal_driver);
}
late_initcall(scb_thermal_cooling_init);

static void __exit scb_thermal_cooling_exit(void)
{
	platform_driver_unregister(&scb_thermal_driver);
}
module_exit(scb_thermal_cooling_exit);

MODULE_DESCRIPTION("Fujitsu SCB-Thermal driver for S70");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:fujitsu-scb-thermal");
MODULE_AUTHOR("Jassi Brar <jaswinder.singh@linaro.org>");
