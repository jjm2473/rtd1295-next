#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>

enum elink_id {
	ELINK_N = 0,
	ELINK_E,
	ELINK_S,
	ELINK_W,
	ELINK_MAX
};

struct chip_type {
	u16 elink_coreid[ELINK_MAX];
	u8 ctrl_mode[ELINK_MAX];
	u16 chip_coreid;
};

struct epiphany {
	struct platform_device *pdev;
	void __iomem *fpga_regs;
	struct clk *clock;
	const struct chip_type *chip_type;
	int row;
	int col;
	u8 elink_clk[ELINK_MAX];
};

#define REG_ESYSCONFIG	0x00
#define REG_ESYSRESET	0x04
#define REG_ESYSINFO	0x08
#define REG_ESYSFILTERL	0x0c
#define REG_ESYSFILTERH	0x10
#define REG_ESYSFILTERC	0x14

#define REG_ELINK_CLK	0x300

#define CTRLMODE_SOUTH_ELINK	0b1000
#define CTRLMODE_WEST_ELINK	0b1101

#define CORE_REGS_BASE	0xf0000
#define CORE_REGS_SIZE	0x800

static const struct chip_type e16g301 = {
	.elink_coreid[ELINK_N] = 0x002,
	.elink_coreid[ELINK_E] = 0x083,
	.elink_coreid[ELINK_S] = 0x0c2,
	.elink_coreid[ELINK_W] = 0x080,
	.ctrl_mode[ELINK_N] = 0x1,
	.ctrl_mode[ELINK_E] = 0x5,
	.ctrl_mode[ELINK_S] = 0x8,
	.ctrl_mode[ELINK_W] = 0xd,
	.chip_coreid = 0x083,
};

static const struct chip_type e64g401 = {
	.elink_coreid[ELINK_N] = 0x006,
	.elink_coreid[ELINK_E] = 0x087,
	.elink_coreid[ELINK_S] = 0x1c2,
	.elink_coreid[ELINK_W] = 0x084,
	.ctrl_mode[ELINK_N] = 0x1,
	.ctrl_mode[ELINK_E] = 0x5,
	.ctrl_mode[ELINK_S] = 0x8,
	.ctrl_mode[ELINK_W] = 0xd,
	.chip_coreid = 0x087, /* A.O. 2014-11-07: not 0x083 */
};

static inline u16 to_coreid(struct epiphany *epiphany, int row, int col)
{
	return (row + epiphany->row) * 64 + (col + epiphany->col);
}

static inline u16 get_elink_coreid(struct epiphany *epiphany, enum elink_id id)
{
	return epiphany->chip_type->elink_coreid[id]
		+ to_coreid(epiphany, 0, 0);
}

static inline u16 get_chip_coreid(struct epiphany *epiphany)
{
	return epiphany->chip_type->chip_coreid
		+ to_coreid(epiphany, 0, 0);
}

static inline void epiphany_reg_write(u32 value, void __iomem *base, u32 offset)
{
	writel(value, (u8 __iomem *)base + offset);
}

static inline u32 epiphany_reg_read(void __iomem *base, u32 offset)
{
	return readl((u8 __iomem *)base + offset);
}

static void epiphany_reset(struct epiphany *epiphany)
{
	struct device *dev = &epiphany->pdev->dev;
	void __iomem *chip_regs;
	bool ctrlmode_changed = false;
	u32 val;
	int i;

	epiphany_reg_write(0, epiphany->fpga_regs, REG_ESYSRESET);
	dev_info(dev, "Initiated reset\n");
	msleep(200);

	// determine chip type
	val = epiphany_reg_read(epiphany->fpga_regs, REG_ESYSINFO);
	switch ((val >> 16) & 0xff) {
	case 0x01:
	case 0x02:
	case 0x03:
	case 0x04:
		epiphany->chip_type = &e16g301;
		break;
	case 0x05:
		epiphany->chip_type = &e64g401;
		break;
	default:
		dev_warn(dev, "Unknown platform 0x%02x\n", (val >> 16) & 0xff);
		return;
	}

	val = epiphany_reg_read(epiphany->fpga_regs, REG_ESYSCONFIG);
	for (i = 0; i < ELINK_MAX; i++) {
		if (epiphany->elink_clk[i] == 0)
			continue;
		epiphany_reg_write((epiphany->chip_type->ctrl_mode[i] << 28) | (val & 0x0fffffff),
		          epiphany->fpga_regs, REG_ESYSCONFIG);
		ctrlmode_changed = true;
		chip_regs = devm_ioremap(dev, (get_elink_coreid(epiphany, i) << 28) | CORE_REGS_BASE, CORE_REGS_SIZE);
		if (chip_regs) {
			epiphany_reg_write(epiphany->elink_clk[i], chip_regs, REG_ELINK_CLK);
			devm_iounmap(dev, chip_regs);
		} else
			dev_warn(dev, "Mapping eLink registers failed.\n");
	}
	if (ctrlmode_changed)
		epiphany_reg_write(val, epiphany->fpga_regs, REG_ESYSCONFIG);
}

static int epiphany_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct epiphany *epiphany;
	struct resource *res;
	//int rc;
	u32 val;

	epiphany = devm_kzalloc(dev, sizeof(*epiphany), GFP_KERNEL);
	if (!epiphany)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOENT;

	if (!devm_request_mem_region(dev, res->start, resource_size(res), "epiphany"))
		return -ENOMEM;

	epiphany->fpga_regs = devm_ioremap_nocache(dev, res->start, resource_size(res));
	if (!epiphany->fpga_regs) {
		devm_release_mem_region(dev, res->start, resource_size(res));
		return -ENOMEM;
	}
/*
	epiphany->clock = devm_clk_get(dev, NULL);
	if (IS_ERR(epiphany->clock)) {
		dev_err(dev, "Epiphany: Clock not found.\n");
		devm_release_mem_region(dev, res->start, resource_size(res));
		return PTR_ERR(epiphany->clock);
	}

	rc = clk_prepare_enable(epiphany->clock);
	if (rc) {
		dev_err(dev, "Epiphany: Unable to enable clock.\n");
		devm_release_mem_region(dev, res->start, resource_size(res));
		return rc;
	}
*/
	epiphany->row = 32;
	epiphany->col = 8;
	epiphany->elink_clk[ELINK_E] = 1;
	epiphany->pdev = pdev;
	platform_set_drvdata(pdev, epiphany);

	dev_info(dev, "Epiphany at 0x%p\n", epiphany->fpga_regs);

	epiphany_reset(epiphany);

	val = epiphany_reg_read(epiphany->fpga_regs, REG_ESYSINFO);
	dev_info(dev, "generation %02x platform %02x type %02x version %02x\n",
	         val >> 24, (val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff);

	return 0;
}

static int epiphany_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct epiphany *epiphany = platform_get_drvdata(pdev);
	struct resource *res;

	//clk_disable_unprepare(epiphany->clock);
	devm_iounmap(dev, epiphany->fpga_regs);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		devm_release_mem_region(dev, res->start, resource_size(res));
	}

	return 0;
}

static const struct of_device_id epiphany_of_match[] = {
	{ .compatible = "adapteva,parallella-fpgaconf" },
	{ }
};
MODULE_DEVICE_TABLE(of, epiphany_of_match);

static struct platform_driver epiphany_driver = {
	.driver = {
		.name = "epiphany",
		.of_match_table = of_match_ptr(epiphany_of_match),
	},
	.probe	= epiphany_probe,
	.remove	= epiphany_remove,
};
module_platform_driver(epiphany_driver);

MODULE_DESCRIPTION("Adapteva Epiphany driver");
MODULE_LICENSE("GPL");
