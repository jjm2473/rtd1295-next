#ifndef __GPIO_REVISION_H__
#define __GPIO_REVISION_H__

#if defined(CONFIG_GPIO_REVISION)

/* returns either -EPROBE_DEFER or the board revision index */
int gpio_revision_get(void);

/*
 * if of_name property does not exist at of_node, return false
 * if it does exist, return the nth bit of the u32 value of the property
 * where n is the gpio_revision number that was detected
 */
bool gpio_revision_of_bitmap(struct device_node *of_node, const char *of_name);

#else

static inline int gpio_revision_get(void)
{
	return 0;
}

static inline bool gpio_revision_of_bitmap(
		struct device_node *of_node, const char *of_name)
{
	return false;
}

#endif
#endif

