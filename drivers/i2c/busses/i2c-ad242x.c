// SPDX-License-Identifier: GPL-2.0-only

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mfd/ad242x.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

struct ad242x_i2c {
	struct device		*dev;
	struct ad242x_node	*node;
	struct i2c_adapter	adap;
	u32			node_index;
};

static int ad242x_set_addr(struct ad242x_node *mnode,
			   struct ad242x_i2c_bus *bus,
			   uint8_t node_id, uint8_t addr)
{
	int ret;
	uint8_t buf[2] = { AD242X_CHIP, addr };

	ret = regmap_update_bits(mnode->regmap, AD242X_NODEADR,
				 AD242X_NODEADR_PERI | AD242X_NODEADR_MASK,
				 node_id);
	if (ret < 0)
		return ret;

	/*
	 * We can't use the slave's regmap here as it holds the same
	 * lock we also need to guard this context.
	 */
	ret = i2c_transfer_buffer_flags(bus->client,
					buf, sizeof(buf), 0);
	if (ret < 0)
		return ret;

	return regmap_update_bits(mnode->regmap, AD242X_NODEADR,
				  AD242X_NODEADR_PERI, AD242X_NODEADR_PERI);
}

static int ad242x_i2c_xfer(struct i2c_adapter *adap,
			   struct i2c_msg msgs[], int num)
{
	struct ad242x_i2c *i2c = adap->algo_data;
	struct ad242x_i2c_bus *bus = ad242x_master_get_bus(i2c->node->master);
	struct ad242x_node *mnode = ad242x_master_get_node(i2c->node->master);
	int ret, i, current_addr = -1;

	mutex_lock(&bus->mutex);

	for (i = 0; i < num; i++) {
		struct i2c_msg *msg = msgs + i;

		if (msg->addr != current_addr) {
			ret = ad242x_set_addr(mnode, bus,
					      i2c->node->id, msg->addr);
			if (ret < 0) {
				dev_err(i2c->node->dev,
					"Cannot set address: %d\n", ret);
				break;
			}

			current_addr = msg->addr;
		}

		ret = i2c_transfer_buffer_flags(bus->client,
						msg->buf, msg->len, msg->flags);
		if (ret < 0)
			break;
	}

	mutex_unlock(&bus->mutex);

	return ret < 0 ? ret : num;
}

static u32 ad242x_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm ad242x_i2c_algorithm = {
	.master_xfer	= ad242x_i2c_xfer,
	.functionality	= ad242x_i2c_functionality,
};

static int ad242x_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ad242x_node *node;
	struct ad242x_i2c *i2c;
	u32 freq, val = 0;
	int ret;

	if (!dev->of_node)
		return -ENODEV;

	node = dev_get_drvdata(dev->parent);
	if ((node->caps & AD242X_CAPABILITY_I2C) == 0) {
		dev_err(dev, "Node %d has no I2C capability", node->id);
		return -ENOTSUPP;
	}

	if (ad242x_node_is_master(node))
		return -EINVAL;

	freq = ad242x_master_get_clk_rate(node->master);
	if (freq == 44100)
		val |= AD242X_I2CCFG_FRAMERATE;

	if (!of_property_read_u32(dev->of_node, "clock-frequency", &freq)) {
		if (freq == 400000)
			val |= AD242X_I2CCFG_DATARATE;
		else if (freq != 100000)
			dev_warn(dev, "Unsupported frequency %d\n", freq);
	}

	ret = regmap_write(node->regmap, AD242X_I2CCFG, val);
	if (ret < 0)
		return ret;

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->node = node;
	i2c->adap.algo = &ad242x_i2c_algorithm;
	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = dev;
	i2c->adap.dev.of_node = dev->of_node;
	i2c_set_adapdata(&i2c->adap, i2c);
	strlcpy(i2c->adap.name, "ad242x remote I2C bus",
		sizeof(i2c->adap.name));

	ret = i2c_add_adapter(&i2c->adap);
	if (ret < 0) {
		dev_err(dev, "error registering adapter: %d\n", ret);
		return ret;
	}

	dev_info(dev, "ad242x i2c driver, node ID %d\n", node->id);
	platform_set_drvdata(pdev, i2c);

	return 0;
}

static int ad242x_i2c_remove(struct platform_device *dev)
{
	struct ad242x_i2c *i2c = platform_get_drvdata(dev);

	i2c_del_adapter(&i2c->adap);

	return 0;
}

static const struct of_device_id ad242x_i2c_of_match[] = {
	{ .compatible = "adi,ad2428w-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad242x_i2c_of_match);

static struct platform_driver ad242x_i2c_driver = {
	.driver = {
		.name = "ad242x-i2c",
		.of_match_table = ad242x_i2c_of_match,
	},
	.probe = ad242x_i2c_probe,
	.remove = ad242x_i2c_remove,
};

module_platform_driver(ad242x_i2c_driver);
MODULE_LICENSE("GPL");
