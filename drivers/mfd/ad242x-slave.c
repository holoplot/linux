// SPDX-License-Identifier: GPL-2.0-only

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/mfd/ad242x.h>
#include <linux/mfd/core.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

struct ad242x_slave {
	struct ad242x_node		node;
	struct ad242x_node		*master;
	struct ad242x_slot_config	slot_config;
	unsigned int			sync_offset;
};

int ad242x_slave_read(struct ad242x_i2c_bus *bus,
		      struct regmap *master_regmap,
		      uint8_t node_id, uint8_t reg, unsigned int *val)
{
	int ret;

	mutex_lock(&bus->mutex);

	ret = regmap_write(master_regmap, AD242X_NODEADR, node_id);
	if (ret < 0)
		goto err_unlock;

	ret = i2c_smbus_read_byte_data(bus->client, reg);
	if (ret < 0)
		goto err_unlock;

	*val = ret;
	ret = 0;

err_unlock:
	mutex_unlock(&bus->mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(ad242x_slave_read);

int ad242x_slave_write(struct ad242x_i2c_bus *bus,
		       struct regmap *master_regmap,
		       uint8_t node_id, uint8_t reg, unsigned int val)
{
	int ret;

	mutex_lock(&bus->mutex);

	ret = regmap_write(master_regmap, AD242X_NODEADR, node_id);
	if (ret < 0)
		goto err_unlock;

	ret = i2c_smbus_write_byte_data(bus->client, reg, val);
	if (ret < 0)
		goto err_unlock;

	ret = 0;

err_unlock:
	mutex_unlock(&bus->mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(ad242x_slave_write);

static int ad242x_slave_regmap_read(void *context, unsigned int reg,
				    unsigned int *val)
{
	struct ad242x_slave *slave = context;
	struct ad242x_i2c_bus *bus = ad242x_master_get_bus(slave->node.master);
	struct ad242x_node *mnode = ad242x_master_get_node(slave->node.master);

	if (reg > 0xff)
		return -EINVAL;

	return ad242x_slave_read(bus, mnode->regmap, slave->node.id, reg, val);
}

static int ad242x_slave_regmap_write(void *context, unsigned int reg,
				     unsigned int val)
{
	struct ad242x_slave *slave = context;
	struct ad242x_i2c_bus *bus = ad242x_master_get_bus(slave->node.master);
	struct ad242x_node *mnode = ad242x_master_get_node(slave->node.master);

	if (val > 0xff || reg > 0xff)
		return -EINVAL;

	return ad242x_slave_write(bus, mnode->regmap, slave->node.id, reg, val);
}

static const struct regmap_config ad242x_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.volatile_reg	= ad242x_is_volatile_reg,
	.writeable_reg	= ad242x_is_writeable_reg,
	.reg_read	= ad242x_slave_regmap_read,
	.reg_write	= ad242x_slave_regmap_write,
	.max_register	= AD242X_MAX_REG,
	.cache_type	= REGCACHE_RBTREE,
};

static int ad242x_calc_sync_offset(unsigned int val)
{
	if (val == 0)
		return 0;

	if (val > 127)
		return -EINVAL;

	return 256 - val;
}

static int ad242x_slave_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ad242x_slave *slave;
	struct ad242x_node *mnode;
	struct regmap *regmap;
	unsigned int val;
	int i, ret;

	slave = devm_kzalloc(dev, sizeof(*slave), GFP_KERNEL);
	if (!slave)
		return -ENOMEM;

	regmap = devm_regmap_init(dev, NULL, slave, &ad242x_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "regmap init failed: %d\n", ret);
		return ret;
	}

	of_property_read_u32(dev->of_node, "reg", &val);
	slave->node.id = val;
	slave->node.dev = dev;
	slave->node.regmap = regmap;

	mnode = dev_get_drvdata(dev->parent);
	slave->node.master = mnode->master;

	dev_set_name(dev, "%s-a2b-%d", dev_name(dev->parent), slave->node.id);
	dev_set_drvdata(dev, &slave->node);

	ret = ad242x_node_probe(&slave->node);
	if (ret < 0)
		return ret;

	ret = ad242x_read_slot_config(dev, dev->of_node, &slave->slot_config);
	if (ret < 0) {
		dev_err(dev, "slot config is invalid: %d\n", ret);
		return ret;
	}

	ret = regmap_write(regmap, AD242X_UPSLOTS,
			   slave->slot_config.up_n_forward_slots);
	if (ret < 0)
		return ret;

	ret = regmap_write(regmap, AD242X_DNSLOTS,
			   slave->slot_config.dn_n_forward_slots);
	if (ret < 0)
		return ret;

	ret = regmap_write(regmap, AD242X_LUPSLOTS,
			   slave->slot_config.up_n_tx_slots);
	if (ret < 0)
		return ret;

	val = slave->slot_config.dn_n_tx_slots;

	if (slave->slot_config.dn_rx_slots)
		val |= AD242X_LDNSLOTS_DNMASKEN;

	ret = regmap_write(regmap, AD242X_LDNSLOTS, val);
	if (ret < 0)
		return ret;

	for (i = 0; i < 4; i++) {
		ret = regmap_write(regmap, AD242X_UPMASK(i),
			(slave->slot_config.up_rx_slots >> (i * 8)) & 0xff);
		if (ret < 0)
			return ret;

		ret = regmap_write(regmap, AD242X_DNMASK(i),
			(slave->slot_config.dn_rx_slots >> (i * 8)) & 0xff);
		if (ret < 0)
			return ret;
	}

	of_property_read_u32(dev->of_node, "adi,sync-offset",
			     &slave->sync_offset);

	ret = ad242x_calc_sync_offset(slave->sync_offset);
	if (ret < 0)
		return ret;

	ret = regmap_write(regmap, AD242X_SYNCOFFSET, ret);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct of_device_id ad242x_slave_of_match[] = {
	{ .compatible = "adi,ad2428w-slave" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad242x_slave_of_match);

static struct platform_driver ad242x_slave_driver = {
	.driver = {
		.name = "ad242x-slave",
		.of_match_table = ad242x_slave_of_match,
	},
	.probe = ad242x_slave_probe,
};

module_platform_driver(ad242x_slave_driver);

MODULE_DESCRIPTION("AD242x slave node driver");
MODULE_AUTHOR("Daniel Mack <daniel@zonque.org>");
MODULE_LICENSE("GPL v2");
