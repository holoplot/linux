// SPDX-License-Identifier: GPL-2.0-only
/*
 * i2c-ad242x.c - Analog Devices AD242x A2B I2C master
 *
 * Copyright (c) 2019, Holoplot GmbH
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/mfd/ad242x.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/regmap.h>

struct ad242x_i2c {
	struct device		*dev;
	struct ad242x_core	*core;
	struct i2c_adapter	adap;
	spinlock_t		lock;
	u32			node_index;
};

static int ad242x_i2c_xfer(struct i2c_adapter *adap,
			   struct i2c_msg msgs[], int num)
{
	struct ad242x_i2c *i2c = adap->algo_data;
	struct i2c_client *bus = i2c->core->bus_i2c_client;
	int ret, i;
	u8 val;

	val = AD242X_NODEADR_PERI;
	val |= i2c->node_index & AD242X_NODEADR_MASK;

	ret = regmap_write(i2c->core->regmap, AD242X_NODEADR, val);
	if (ret < 0) {
		dev_err(i2c->dev, "Cannot select A2B node: %d\n", ret);
		return ret;
	}

	for (i = 0; i < num; i++) {
		struct i2c_msg *msg = msgs + i;
		ret = i2c_transfer_buffer_flags(bus, msg->buf,
						msg->len, msg->flags);
		if (ret < 0) {
			dev_err(i2c->dev, "Unable to transfer: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static u32 ad242x_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL |
		I2C_FUNC_PROTOCOL_MANGLING | I2C_FUNC_NOSTART;
}

static const struct i2c_algorithm ad242x_i2c_algorithm = {
	.master_xfer	= ad242x_i2c_xfer,
	.functionality	= ad242x_i2c_functionality,
};

static const struct of_device_id ad242x_i2c_dt_ids[] = {
	{ .compatible = "adi,ad242x-i2c", },
	{}
};
MODULE_DEVICE_TABLE(of, ad242x_i2c_dt_ids);

static const struct platform_device_id ad242x_i2c_id_table[] = {
	{ "ad242x-i2c" },
	{},
};
MODULE_DEVICE_TABLE(platform, ad242x_i2c_id_table);

static int ad242x_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *parent_node;
	struct ad242x_i2c *i2c;
	int ret;

	i2c = devm_kzalloc(dev, sizeof(struct ad242x_i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->core = to_ad242x_core(dev->parent);
	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = dev;
	i2c->adap.dev.of_node = dev->of_node;

	parent_node = dev->of_node->parent;
	if (!parent_node) {
		dev_err(dev, "device must be embedded in an A2B node\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(parent, "reg", &i2c->node_number);
	if (ret < 0) {
		dev_err(dev, "error reading 'reg' property of parent: %d\n", ret);
		return ret;
	}

	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		dev_err(dev, "error registering adapter: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, i2c);

	return 0;
}

static int ad242x_i2c_remove(struct platform_device *dev)
{
	struct ad242x_i2c *i2c = platform_get_drvdata(dev);

	i2c_del_adapter(&i2c->adap);

	return 0;
}

static struct platform_driver ad242x_i2c_driver = {
	.probe		= ad242x_i2c_probe,
	.remove		= ad242x_i2c_remove,
	.driver		= {
		.name	= "ad242x-i2c",
		.of_match_table = ad242x_i2c_dt_ids,
	},
	.id_table	= ad242x_i2c_id_table,
};

static int __init ad242x_i2c_adap_init(void)
{
	return platform_driver_register(&ad242x_i2c_driver);
}

static void __exit ad242x_i2c_adap_exit(void)
{
	platform_driver_unregister(&ad242x_i2c_driver);
}

MODULE_LICENSE("GPL");

subsys_initcall(ad242x_i2c_adap_init);
module_exit(ad242x_i2c_adap_exit);
