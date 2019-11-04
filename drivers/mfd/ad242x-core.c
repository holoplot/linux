// SPDX-License-Identifier: GPL-2.0-only
/*
 * ad242x_core.c - Analog Devices AD242x A2B master devices
 *
 * Copyright (c) 2019, Holoplot GmbH
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/mfd/ad242x.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

static const struct mfd_cell ad242x_core_cells[] = {
	{ .name = "ad242x-i2c", },
	{ .name = "ad242x-i2s", },
	{ .name = "ad242x-gpio", },
};

static bool ad242x_core_is_volatile_reg(struct device *dev, unsigned int reg)
{
	return false;
}

static bool ad242x_core_is_precious_reg(struct device *dev, unsigned int reg)
{
	return false;
}

static bool ad242x_core_is_writeable_reg(struct device *dev, unsigned int reg)
{
	return !ad242x_core_is_volatile_reg(dev, reg);
}

static const struct regmap_config ad242x_core_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.volatile_reg	= ad242x_core_is_volatile_reg,
	.precious_reg	= ad242x_core_is_precious_reg,
	.writeable_reg	= ad242x_core_is_writeable_reg,
	.max_register	= 0x9b,
	.cache_type	= REGCACHE_RBTREE,
};

static irqreturn_t ad242x_core_i2c_isr(int irq, void *devid)
{
	//struct core *ad242x_core = devid
	return IRQ_HANDLED;
}

static int ad242x_core_i2c_probe(struct i2c_client *i2c,
				 const struct i2c_device_id *id)
{
	struct device_node *bus_np, *nodes_np;
	struct device *dev = &i2c->dev;
	struct ad242x_core *core;
	int ret;
	u32 val;

	core = devm_kzalloc(dev, sizeof(struct ad242x_core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	core->dev = dev;
	dev_set_drvdata(dev, core);
	i2c_set_clientdata(i2c, core);

	core->regmap = devm_regmap_init_i2c(i2c, &ad242x_core_regmap_config);
	if (IS_ERR(core->regmap)) {
		ret = PTR_ERR(core->regmap);
		dev_err(dev, "regmap init failed: %d\n", ret);
		return ret;
	}

	ret = devm_request_threaded_irq(dev, i2c->irq, NULL,
					ad242x_core_i2c_isr, IRQF_ONESHOT,
					dev_name(dev), core);
	if (ret < 0) {
		dev_err(dev, "failed to register IRQ handler %d\n", ret);
		return ret;
	}

	ret = regmap_read(core->regmap, AD242X_VENDOR, &val);
	if (ret < 0) {
		dev_err(dev, "failed to read VENDOR register %d\n", ret);
		return ret;
	}

	if (val != 0xad) {
		dev_err(dev, "wrong value 0x%02x in VENDOR register\n", val);
		return -ENODEV;
	}

	ret = regmap_read(core->regmap, AD242X_PRODUCT, &val);
	if (ret < 0) {
		dev_err(dev, "failed to read PRODUCT register %d\n", ret);
		return ret;
	}

	if (val != 0x28) {
		dev_err(dev, "wrong value 0x%02x in PRODUCT register\n", val);
		return -ENODEV;
	}

	ret = regmap_read(core->regmap, AD242X_VERSION, &val);
	if (ret < 0) {
		dev_err(dev, "failed to read VERSION register %d\n", ret);
		return ret;
	}

	dev_info(dev, "Detected AD242x device, version 0x%02x\n", val);

	bus_np = of_parse_phandle(dev->of_node, "adi,a2b-bus", 0);
	if (!bus_np) {
		dev_err(dev, "no 'adi,a2b-bus' handle specified\n");
		return -EINVAL;
	}

	ret = devm_mfd_add_devices(dev, -1, ad242x_core_cells,
				   ARRAY_SIZE(ad242x_core_cells),
				   NULL, 0, NULL);
	if (ret < 0) {
		dev_err(dev, "failed to add MFD devices %d\n", ret);
		return ret;
	}

	return 0;
}

static int ad242x_core_i2c_remove(struct i2c_client *i2c)
{
	//struct ad242x_core *core = i2c_get_clientdata(i2c);
	return 0;
}

static const struct of_device_id ad242x_core_of_match[] = {
	{ .compatible = "adi,ad2428w" },
	{},
};
MODULE_DEVICE_TABLE(of, ad242x_core_of_match);

static const struct i2c_device_id ad242x_core_i2c_id[] = {
	{"ad242x-core", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ad242x_core_i2c_id);

static struct i2c_driver ad242x_core_i2c_driver = {
	.driver = {
		.name = "ad242x-core",
		.of_match_table = ad242x_core_of_match,
	},
	.probe = ad242x_core_i2c_probe,
	.remove = ad242x_core_i2c_remove,
	.id_table = ad242x_core_i2c_id,
};

module_i2c_driver(ad242x_core_i2c_driver);

MODULE_DESCRIPTION("AD242x core driver");
MODULE_AUTHOR("Daniel Mack <daniel@zonque.org>");
MODULE_LICENSE("GPL v2");
