#include <linux/io.h>
#include <linux/memblock.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#define REG_WRAP_CTRL	0x000

#define WRAP_CTRL_BUFABLE_SEL_SHIFT	12
#define WRAP_CTRL_BUFABLE_SEL_MASK	(0x3 << WRAP_CTRL_BUFABLE_SEL_SHIFT)

static struct map_desc rtd119x_io_desc[] __initdata = {
	/* rbus */
	{
		.virtual = 0xfe000000,
		.pfn = __phys_to_pfn(0x18000000),
		.length = 0x70000,
		.type = MT_DEVICE,
	},
	/* GIC */
	{
		.virtual = 0xff010000,
		.pfn = __phys_to_pfn(0xff010000),
		.length = 0x10000,
		.type = MT_DEVICE,
	},
	/* rpc ring buffer */
	{
		.virtual = 0xfc800000 - 0x4000,
		.pfn = __phys_to_pfn(0x01ffe000),
		.length = 0x4000,
		.type = MT_DEVICE,
	},
	/* rpc comm */
	{
		.virtual = 0xfe070000,
		.pfn = __phys_to_pfn(0x0000b000),
		.length = 0x1000,
		.type = MT_DEVICE,
	},
	/* spi */
	{
		.virtual = 0xfb000000,
		.pfn = __phys_to_pfn(0x18100000),
		.length = 0x1000000,
		.type = MT_DEVICE,
	},
};

static void __init rtd119x_map_io(void)
{
	debug_ll_io_init();
	iotable_init(rtd119x_io_desc, ARRAY_SIZE(rtd119x_io_desc));
}

static void __init rtd119x_memblock_remove(phys_addr_t base, phys_addr_t size)
{
	int ret;

	ret = memblock_remove(base, size);
	if (ret)
		pr_err("Failed to remove memblock %llx\n", base);
	else
		pr_debug("Removed memblock %llx\n", base);
}

static void __init rtd119x_reserve(void)
{
	rtd119x_memblock_remove(0x10000000, 0x00100000);
	rtd119x_memblock_remove(0x18000000, 0x00100000);
	rtd119x_memblock_remove(0x18100000, 0x01000000);
}

static void __init rtd119x_machine_init(void)
{
	struct device_node *node;
	void __iomem *base;
	u32 val;

	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);

	node = of_find_compatible_node(NULL, NULL, "realtek,rtd1195-scpu-wrapper");
	if (!node) {
		pr_err("%s: missing SCPU wrapper\n", __func__);
		return;
	}

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("%s: could not map SCPU wrapper registers\n", __func__);
		return;
	}

	val = readl(base + REG_WRAP_CTRL);
	val &= ~WRAP_CTRL_BUFABLE_SEL_MASK;
	val |= 0x1 << WRAP_CTRL_BUFABLE_SEL_SHIFT;
	writel(val, base + REG_WRAP_CTRL);

	node = of_find_compatible_node(NULL, NULL, "realtek,rtd1195-sb2");
	if (!node) {
		pr_err("%s: missing SB2\n", __func__);
		return;
	}

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("%s: could not map SB2 registers\n", __func__);
		return;
	}

	writel(0x1234, base + 0x20);
}

static const char *const rtd119x_dt_compat[] __initconst = {
	"realtek,rtd1195",
	NULL
};

DT_MACHINE_START(rtd119x, "RTD119x")
	.dt_compat = rtd119x_dt_compat,
	.init_machine = rtd119x_machine_init,
	.reserve = rtd119x_reserve,
	.map_io = rtd119x_map_io,
MACHINE_END
