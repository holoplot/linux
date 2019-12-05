// SPDX-License-Identifier: GPL-2.0-only

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/mfd/ad242x.h>
#include <linux/module.h>
#include <linux/of.h>

static int ad242x_bus_i2c_probe(struct i2c_client *i2c,
				const struct i2c_device_id *id)
{
	dev_set_drvdata(&i2c->dev, i2c);
	i2c_set_clientdata(i2c, &i2c->dev);
	return 0;
}

static const struct of_device_id ad242x_bus_of_match[] = {
	{ .compatible = "adi,ad2428w-bus" },
	{ },
};
MODULE_DEVICE_TABLE(of, ad242x_bus_of_match);

static const struct i2c_device_id ad242x_bus_i2c_id[] = {
	{ "ad242x_bus", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ad242x_bus_i2c_id);

static struct i2c_driver ad242x_bus_i2c_driver = {
	.driver = {
		.name = "ad242x-bus",
		.of_match_table = ad242x_bus_of_match,
	},
	.probe = ad242x_bus_i2c_probe,
	.id_table = ad242x_bus_i2c_id,
};

module_i2c_driver(ad242x_bus_i2c_driver);

MODULE_DESCRIPTION("AD242x bus driver");
MODULE_AUTHOR("Daniel Mack <daniel@zonque.org>");
MODULE_LICENSE("GPL v2");
