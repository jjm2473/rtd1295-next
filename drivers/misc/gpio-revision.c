#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/seq_file.h>
#include <linux/module.h>

static int gpio_revision = -EPROBE_DEFER;

int gpio_revision_get(void)
{
	return gpio_revision;
}
EXPORT_SYMBOL_GPL(gpio_revision_get);

bool gpio_revision_of_bitmap(struct device_node *of_node, const char *of_name)
{
	u32 val;

	if (of_property_read_u32(of_node, of_name, &val))
		return false;

	return (1 << gpio_revision) & val;
}
EXPORT_SYMBOL_GPL(gpio_revision_of_bitmap);

static ssize_t show_gpio_revision(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%u\n", gpio_revision);
}

static DEVICE_ATTR(gpio_revision, 0444, show_gpio_revision, NULL);

static struct attribute *gpio_revision_attributes[] = {
	&dev_attr_gpio_revision.attr,
	NULL
};

static struct attribute_group gpio_revision_attribute_group = {
	.attrs = gpio_revision_attributes,
};

static int gpio_revision_probe(struct platform_device *pdev)
{
	int err, rev = 31, i, n;

	/* if no gpios present in DT, will set revision 31 */

	if (of_gpio_count(pdev->dev.of_node) > 0)
		rev = 0;

	for (i = 0; i < of_gpio_count(pdev->dev.of_node); i++) {

		n = of_get_gpio(pdev->dev.of_node, i);

		err = gpio_request(n, "gpio-revision");
		if (unlikely(err)) {
			dev_err(&pdev->dev, " gpio %d request failed ", n);
			return err;
		}

		err = gpio_direction_input(n);
		if (unlikely(err < 0)) {
			dev_err(&pdev->dev, "failed to set gpio as input\n");
			gpio_free(n);
			return err;
		}

		err = gpio_get_value(n);
		if (unlikely(err < 0)) {
			dev_err(&pdev->dev, "failed to read gpio\n");
			gpio_free(n);
			return err;
		}
		gpio_free(n);

		rev = (rev << 1) | (err & 1);
	}

	gpio_revision = rev;
	dev_info(&pdev->dev, "Using Revision: 0x%x\n", gpio_revision);

	return sysfs_create_group(&pdev->dev.kobj,
				    &gpio_revision_attribute_group);
}

static int gpio_revision_remove(struct platform_device *pdev)
{
	return sysfs_create_group(&pdev->dev.kobj,
				&gpio_revision_attribute_group);
}

static const struct of_device_id gpio_revision_dt_ids[] = {
	{
		.compatible = "fujitsu,gpio-revision",
	},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, gpio_revision_dt_ids);

static struct platform_driver gpio_revision_driver = {
	.driver = {
			.name = "gpio-revision",
			.of_match_table = gpio_revision_dt_ids,
		   },
	.probe = gpio_revision_probe,
	.remove = gpio_revision_remove,
};

module_platform_driver(gpio_revision_driver);
