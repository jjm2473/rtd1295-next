#include <linux/ahci_platform.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/platform_device.h>

static const struct of_device_id rtd129x_ahci_dt_ids[] = {
	{ "realtek,rtd1295-ahci" },
	{ }
};

static int rtd129x_ahci_probe(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "probed\n");

	return 0;
}

static struct platform_driver rtd129x_ahci_platform_driver = {
	.driver = {
		.name = "ahci_rtd1295",
		.of_match_table = rtd129x_ahci_dt_ids,
	},
	.probe = rtd129x_ahci_probe,
};
builtin_platform_driver(rtd129x_ahci_platform_driver);
