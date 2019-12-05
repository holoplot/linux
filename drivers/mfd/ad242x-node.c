// SPDX-License-Identifier: GPL-2.0-only

#include <linux/clk.h>
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

/* See Table 7-43 in the datasheet */
static int ad242x_tdmmode_index(unsigned int mode, bool slave)
{
	switch (mode) {
	case 2:
		return 0;
	case 4:
		return 1;
	case 8:
		return 2;
	case 12:
		return slave ? -EINVAL : 3;
	case 16:
		return 4;
	case 20:
		return slave ? -EINVAL : 5;
	case 24:
		return slave ? -EINVAL : 6;
	case 32:
		return 7;
	default:
		return -EINVAL;
	}
}

int ad242x_node_probe(struct ad242x_node *node)
{
	struct device_node *np = node->dev->of_node;
	unsigned int val;
	int ret;

	ret = regmap_read(node->regmap, AD242X_VENDOR, &val);
	if (ret < 0) {
		dev_err(node->dev, "failed to read VENDOR register %d\n", ret);
		return ret;
	}

	if (val != 0xad) {
		dev_err(node->dev, "bogus value 0x%02x in VENDOR register\n",
			val);
		return -ENODEV;
	}

	ret = regmap_read(node->regmap, AD242X_PRODUCT, &val);
	if (ret < 0) {
		dev_err(node->dev, "failed to read PRODUCT register %d\n",
			ret);
		return ret;
	}

	if (val != 0x28) {
		dev_err(node->dev, "bogus value 0x%02x in PRODUCT register\n",
			val);
		return -ENODEV;
	}

	ret = regmap_read(node->regmap, AD242X_VERSION, &val);
	if (ret < 0) {
		dev_err(node->dev, "failed to read VERSION register %d\n", ret);
		return ret;
	}

	if (ad242x_node_is_master(node))
		dev_info(node->dev,
			 "Detected AD242x master node, version %d.%d\n",
			 val >> 4, val & 0xf);
	else
		dev_info(node->dev,
			 "Detected AD242x slave node, version %d.%d, id %d\n",
			 val >> 4, val & 0xf, node->id);

	ret = regmap_read(node->regmap, AD242X_CAPABILITY, &val);
	if (ret < 0) {
		dev_err(node->dev, "failed to read CAPABILITY register %d\n",
			ret);
		return ret;
	}

	node->caps = val;

	val = 0;

	if (of_property_read_bool(np, "adi,spread-a2b-clock"))
		val |= AD242X_PLLCTL_SSMODE_AB;
	else if (of_property_read_bool(np, "adi,spread-a2b-i2s-clock"))
		val |= AD242X_PLLCTL_SSMODE_AB_I2S;

	if (of_property_read_bool(np, "adi,spread-spectrum-high"))
		val |= AD242X_PLLCTL_SSDEPTH;

	ret = regmap_write(node->regmap, AD242X_PLLCTL, val);
	if (ret < 0) {
		dev_err(node->dev, "failed to write PLLCTL register %d\n", ret);
		return ret;
	}

	/* I2S global setup */

	of_property_read_u32(np, "adi,tdm-mode", &node->tdm_mode);
	of_property_read_u32(np, "adi,tdm-slot-size", &node->tdm_slot_size);

	ret = ad242x_tdmmode_index(node->tdm_mode,
				   !ad242x_node_is_master(node));
	if (ret < 0) {
		dev_err(node->dev, "invalid TDM mode %d\n", node->tdm_mode);
		return -EINVAL;
	}

	val = AD242X_I2SGCTL_TDMMODE(ret);

	if (node->tdm_slot_size == 16) {
		val |= AD242X_I2SGCTL_TDMSS;
	} else if (node->tdm_slot_size != 32) {
		dev_err(node->dev, "invalid TDM slot size %d\n",
			node->tdm_slot_size);
		return -EINVAL;
	}

	if (of_property_read_bool(np, "adi,alternating-sync"))
		val |= AD242X_I2SGCTL_ALT;

	if (of_property_read_bool(np, "adi,early-sync"))
		val |= AD242X_I2SGCTL_EARLY;

	if (of_property_read_bool(np, "adi,invert-sync"))
		val |= AD242X_I2SGCTL_INV;

	ret = regmap_write(node->regmap, AD242X_I2SGCTL, val);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct mfd_cell ad242x_mfd_cells[] = {
	{
		.of_compatible	= "adi,ad2428w-i2c",
		.name		= "ad242x-i2c",
	},
	{
		.of_compatible	= "adi,ad2428w-gpio",
		.name		= "ad242x-gpio",
	},
	{
		.of_compatible	= "adi,ad2428w-clk",
		.name		= "ad242x-clk",
	},
	{
		.of_compatible	= "adi,ad2428w-codec",
		.name		= "ad242x-codec",
	},
};

int ad242x_node_add_mfd_cells(struct device *dev)
{
	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO,
				    ad242x_mfd_cells,
				    ARRAY_SIZE(ad242x_mfd_cells),
				    NULL, 0, NULL);
}

static int ad242x_get_slot_mask(const struct device_node *np,
				const char *propname, u32 *mask)
{
	unsigned int i, num;
	int ret, proplen;
	u32 slots[32];

	if (!of_get_property(np, propname, &proplen))
		return 0;

	num = proplen / sizeof(u32);

	if (num > ARRAY_SIZE(slots))
		return -EOVERFLOW;

	ret = of_property_read_u32_array(np, propname, slots, num);
	if (ret < 0)
		return ret;

	*mask = 0;

	for (i = 0; i < num; i++) {
		if (slots[i] >= 32)
			return -EINVAL;

		*mask |= BIT(slots[i]);
	}

	return 0;
}

int ad242x_read_slot_config(struct device *dev,
			    struct device_node *np,
			    struct ad242x_slot_config *config)
{
	struct device_node *dn_np, *up_np;
	int ret = 0;

	memset(config, 0, sizeof(*config));

	dn_np = of_get_child_by_name(np, "downstream");
	if (dn_np) {
		ret = ad242x_get_slot_mask(dn_np, "rx-slots", &config->dn_rx_slots);
		if (ret < 0 ) {
			dev_err(dev, "invalid downstream rx-slots property\n");
			goto err_put_dn_node;
		}

		of_property_read_u32(dn_np, "#tx-slots", &config->dn_n_tx_slots);
		of_property_read_u32(dn_np, "#forward-slots",
				&config->dn_n_forward_slots);
		if (config->dn_n_tx_slots + config->dn_n_forward_slots >= 32) {
			dev_err(dev, "invalid downstream tx-slots property\n");
			goto err_put_dn_node;
		}
	}

	up_np = of_get_child_by_name(np, "upstream");
	if (up_np) {
		ret = ad242x_get_slot_mask(up_np, "rx-slots", &config->up_rx_slots);
		if (ret < 0) {
			dev_err(dev, "invalid upstream rx-slots property\n");
			goto err_put_up_node;
		}

		of_property_read_u32(up_np, "#tx-slots", &config->up_n_tx_slots);
		of_property_read_u32(up_np, "#forward-slots",
				&config->up_n_forward_slots);
		if (config->up_n_tx_slots + config->up_n_forward_slots >= 32) {
			dev_err(dev, "invalid downstream tx-slots property\n");
			goto err_put_up_node;
		}
	}

err_put_up_node:
	of_node_put(up_np);
err_put_dn_node:
	of_node_put(dn_np);

	return ret;
}
