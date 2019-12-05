// SPDX-License-Identifier: GPL-2.0-only

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mfd/ad242x.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

struct ad242x_master {
	struct ad242x_node	node;
	struct clk		*sync_clk;
	struct completion	run_completion;
	struct completion	discover_completion;
	struct ad242x_i2c_bus	bus;
	struct irq_domain	*irq_domain;
	struct mutex		mutex;
	unsigned int		irq_mask;
	unsigned int		up_slot_size;
	unsigned int		dn_slot_size;
	bool			up_slot_alt_fmt;
	bool			dn_slot_alt_fmt;
	unsigned int		sync_clk_rate;
	int			irq;
	u8			response_cycles;
	u8			node_inttype[17];
	bool			up_enabled, dn_enabled;
};

struct ad242x_node *ad242x_master_get_node(struct ad242x_master *master)
{
	return &master->node;
}
EXPORT_SYMBOL_GPL(ad242x_master_get_node);

struct ad242x_i2c_bus *ad242x_master_get_bus(struct ad242x_master *master)
{
	return &master->bus;
}
EXPORT_SYMBOL_GPL(ad242x_master_get_bus);

const char *ad242x_master_get_clk_name(struct ad242x_master *master)
{
	return __clk_get_name(master->sync_clk);
}
EXPORT_SYMBOL_GPL(ad242x_master_get_clk_name);

unsigned int ad242x_master_get_clk_rate(struct ad242x_master *master)
{
	return master->sync_clk_rate;
}
EXPORT_SYMBOL_GPL(ad242x_master_get_clk_rate);

u8 ad242x_node_inttype(struct ad242x_node *node)
{
	struct ad242x_master *master = node->master;

	if (WARN_ON(node->id >= ARRAY_SIZE(master->node_inttype)))
		return 0;

	return master->node_inttype[node->id];
}
EXPORT_SYMBOL_GPL(ad242x_node_inttype);

static void ad242x_handle_node_irq(struct ad242x_master *master, u8 node_id)
{
	int virq;

	if (master->irq_mask & BIT(node_id))
		return;

	virq = irq_find_mapping(master->irq_domain, node_id);
	handle_nested_irq(virq);
}

static int ad242x_read_one_irq(struct ad242x_master *master)
{
	struct regmap *regmap = master->node.regmap;
	struct device *dev = master->node.dev;
	unsigned int val, intstat, intsrc, inttype;
	u8 node_id;
	int ret;

	ret = regmap_read(regmap, AD242X_INTSTAT, &intstat);
	if (ret < 0)
		return ret;

	if ((intstat & AD242X_INTSTAT_IRQ) == 0)
		return -ENOENT;

	ret = regmap_read(regmap, AD242X_INTSRC, &intsrc);
	if (ret < 0)
		return ret;

	if (intsrc == 0)
		return -ENOENT;

	if (intsrc & AD242X_INTSRC_SLVINT)
		node_id = intsrc & 0xf;
	else
		node_id = 16;

	ret = regmap_read(regmap, AD242X_INTTYPE, &inttype);
	if (ret < 0)
		return ret;

	switch (inttype) {
	case AD242X_INTTYPE_DSCDONE:
		complete(&master->discover_completion);
		break;
	case AD242X_INTTYPE_MSTR_RUNNING:
		complete(&master->run_completion);
		break;
	default:
		master->node_inttype[node_id] = inttype;
		ad242x_handle_node_irq(master, node_id);
	}

	return 0;
}

static int ad242x_read_irqs(struct ad242x_master *master)
{
	int ret;
	bool first = true;

	while (true) {
		ret = ad242x_read_one_irq(master);
		if (ret < 0)
			return ret;
		if (ret == -ENOENT)
			return first ? ret : 0;

		first = false;
	}
}

static irqreturn_t ad242x_handle_irq(int irq, void *dev_id)
{
	struct ad242x_master *master = dev_id;

	mutex_lock(&master->mutex);
	ad242x_read_irqs(master);
	mutex_unlock(&master->mutex);

	return IRQ_HANDLED;
}

static int ad242x_wait_for_irq(struct ad242x_master *master,
			       struct completion *completion,
			       unsigned int timeout)
{
	int ret;

	if (master->irq > 0) {
		ret = wait_for_completion_timeout(completion,
						  msecs_to_jiffies(timeout));
	} else {
		usleep_range(timeout * 1000, timeout * 1500);
		ad242x_read_irqs(master);
		ret = completion_done(completion);
	}

	return ret == 0 ? -ETIMEDOUT : 0;
}

static void ad242x_irq_ack(struct irq_data *data)
{
	printk(KERN_ERR "___ %s()\n", __func__);
}

static void ad242x_irq_mask(struct irq_data *data)
{
	struct ad242x_master *master = irq_data_get_irq_chip_data(data);

	master->irq_mask |= BIT(data->hwirq);
}

static void ad242x_irq_unmask(struct irq_data *data)
{
	struct ad242x_master *master = irq_data_get_irq_chip_data(data);

	master->irq_mask &= ~BIT(data->hwirq);
}

static struct irq_chip ad242x_irq_chip = {
	.name		= "ad242x",
	.irq_ack	= ad242x_irq_ack,
	.irq_mask	= ad242x_irq_mask,
	.irq_unmask	= ad242x_irq_unmask,
};

static int ad242x_irq_domain_map(struct irq_domain *d, unsigned int irq,
				  irq_hw_number_t hw)
{
	struct ad242x_master *master = d->host_data;

	irq_set_chip_data(irq, master);
	irq_set_chip_and_handler(irq, &ad242x_irq_chip, handle_edge_irq);
	irq_set_nested_thread(irq, 1);
	// irq_set_nothread(irq);
	irq_set_noprobe(irq);

	return 0;
}

static const struct irq_domain_ops ad242x_irq_domain_ops = {
	.map = ad242x_irq_domain_map,
};

/* See Table 3-2 in the datasheet */
static unsigned int ad242x_bus_bits(unsigned int slot_size, bool alt_fmt)
{
	int alt_bits[8] = { 0, 13, 17, 21, 30, 0, 39, 0 };
	int idx = AD242X_SLOTFMT_DNSIZE(slot_size);

	return alt_fmt ? alt_bits[idx] : slot_size + 1;
}

/* See Table 9-1 in the datasheet */
static unsigned int ad242x_master_respoffs(struct ad242x_node *node)
{
	if (node->tdm_mode == 2 && node->tdm_slot_size == 16)
		return 238;

	if ((node->tdm_mode == 2 && node->tdm_slot_size == 32) ||
	    (node->tdm_mode == 4 && node->tdm_slot_size == 16))
		return 245;

	return 248;
}

static int ad242x_newstrct(struct ad242x_master *master)
{
	return regmap_update_bits(master->node.regmap,
				  AD242X_CONTROL,
				  AD242X_CONTROL_NEWSTRCT,
				  AD242X_CONTROL_NEWSTRCT);
}

static int ad242x_discover(struct ad242x_master *master,
			   struct device_node *nodes_np)
{
	struct regmap *regmap = master->node.regmap;
	struct device *dev = master->node.dev;
	struct device_node *child_np;
	unsigned int val, n = 0, i, respoffs, respcycs;
	unsigned int respcycs_up_min = UINT_MAX;
	unsigned int respcycs_dn_max = 0;
	unsigned int master_up_slots = 0;
	unsigned int master_dn_slots = 0;
	uint8_t slave_control = 0;
	int ret;

	respoffs = ad242x_master_respoffs(&master->node);

	for_each_available_child_of_node(nodes_np, child_np) {
		unsigned int dnslot_activity, upslot_activity;
		unsigned int slave_dn_slots, slave_up_slots;
		unsigned int respcycs_dn, respcycs_up;
		struct ad242x_slot_config slot_config;

		ret = ad242x_read_slot_config(dev, child_np, &slot_config);
		if (ret < 0) {
			dev_err(dev, "slot config of slave %d is invalid\n", n);
			return ret;
		}

		/* See section 3-18 in the datasheet */
		slave_dn_slots = max_t(int, slot_config.dn_n_forward_slots,
				       fls(slot_config.dn_rx_slots));
		slave_up_slots = max_t(int, slot_config.up_n_forward_slots,
				       slot_config.up_n_tx_slots);

		if (n == 0) {
			master_up_slots = slave_up_slots;
			master_dn_slots = slave_dn_slots;
		}

		/* See Appendix B in the datasheet */
		dnslot_activity = slave_dn_slots *
			ad242x_bus_bits(master->dn_slot_size,
					master->dn_slot_alt_fmt);
		upslot_activity = slave_up_slots *
			ad242x_bus_bits(master->up_slot_size,
					master->up_slot_alt_fmt);

		respcycs_dn = DIV_ROUND_UP(64 + dnslot_activity, 4) + 4*n + 2;
		respcycs_up = respoffs -
			      (DIV_ROUND_UP(64 + upslot_activity, 4) + 1);

		if (respcycs_dn > respcycs_dn_max)
			respcycs_dn_max = respcycs_dn;

		if (respcycs_up < respcycs_up_min)
			respcycs_up_min = respcycs_up;

		if (slave_dn_slots > 0)
			master->dn_enabled = true;

		if (slave_up_slots > 0)
			master->up_enabled = true;

		n++;
	}

	if (n == 0) {
		dev_err(dev, "No child nodes specified\n");
		return -EINVAL;
	}

	if (of_property_read_bool(dev->of_node, "adi,invert-xcvr-b")) {
		ret = regmap_update_bits(regmap, AD242X_CONTROL,
					 AD242X_CONTROL_XCVRBINV,
					 AD242X_CONTROL_XCVRBINV);
		if (ret < 0)
			return ret;

		slave_control = AD242X_CONTROL_XCVRBINV;
	}

	if (respcycs_dn_max > respcycs_up_min) {
		dev_err(dev, "Unsupported bus topology\n");
		return -EINVAL;
	}

	respcycs = (respcycs_dn_max + respcycs_up_min) / 2;
	ret = regmap_write(regmap, AD242X_RESPCYCS, respcycs);
	if (ret < 0)
		return ret;

	ret = ad242x_newstrct(master);
	if (ret < 0)
		return ret;

	ret = regmap_write(regmap, AD242X_SWCTL, AD242X_SWCTL_ENSW);
	if (ret < 0)
		return ret;

	for (i = 0; i < n; i++) {
		ret = regmap_write(regmap, AD242X_DISCVRY, respcycs - (4*i));
		if (ret < 0)
			return ret;

		ret = ad242x_wait_for_irq(master,
					  &master->discover_completion, 50);
		if (ret < 0) {
			dev_err(dev, "Discovery of node %d timed out\n", i);
			return ret;
		}

		val = AD242X_SWCTL_MODE(2) | AD242X_SWCTL_ENSW;

		if (i == 0)
			ret = regmap_write(regmap, AD242X_SWCTL, val);
		else
			ret = ad242x_slave_write(&master->bus, regmap, i,
						 AD242X_SWCTL, val);

		if (ret < 0)
			return ret;

		dev_info(dev, "Node %d discovered\n", i);

		/* Last node? */
		if (i == n - 1)
			break;

		ret = ad242x_slave_write(&master->bus, regmap, i,
					 AD242X_INTMSK2,
					 AD242X_INTMSK2_DSCDIEN);
		if (ret < 0)
			return ret;

		ret = ad242x_slave_write(&master->bus, regmap, i,
					 AD242X_CONTROL, slave_control);
		if (ret < 0)
			return ret;

		ret = ad242x_slave_write(&master->bus, regmap, i,
					 AD242X_SWCTL, AD242X_SWCTL_ENSW);
		if (ret < 0)
			return ret;

		reinit_completion(&master->discover_completion);
	}

	ret = regmap_write(regmap, AD242X_SWCTL, AD242X_SWCTL_ENSW);
	if (ret < 0)
		return ret;

	ret = regmap_write(regmap, AD242X_DNSLOTS, master_dn_slots);
	if (ret < 0)
		return ret;

	ret = regmap_write(regmap, AD242X_UPSLOTS, master_up_slots);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad242x_init_irq(struct ad242x_master *master)
{
	struct regmap *regmap = master->node.regmap;
	struct device *dev = master->node.dev;
	int ret;

	ret = regmap_write(regmap, AD242X_INTMSK0,
			   AD242X_INTMSK0_SRFEIEN | AD242X_INTMSK0_BECIEN |
			   AD242X_INTMSK0_PWREIEN | AD242X_INTMSK0_CRCEIEN |
			   AD242X_INTMSK0_DDEIEN  | AD242X_INTMSK0_HCEIEN);
	if (ret < 0)
		return ret;

	ret = regmap_write(regmap, AD242X_INTMSK2,
			   AD242X_INTMSK2_DSCDIEN | AD242X_INTMSK2_SLVIRQEN);
	if (ret < 0)
		return ret;

	master->irq_domain =
		irq_domain_add_linear(dev->of_node, 16 /* XXX */,
				      &ad242x_irq_domain_ops, master);
	if (!master->irq_domain)
		return -ENODEV;

	/* Mask all slave IRQs */
	master->irq_mask = 0xffff;

	return 0;
}

static const struct regmap_config ad242x_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.volatile_reg	= ad242x_is_volatile_reg,
	.writeable_reg	= ad242x_is_writeable_reg,
	.max_register	= AD242X_MAX_REG,
	.cache_type	= REGCACHE_RBTREE,
};

static int ad242x_master_probe(struct i2c_client *i2c,
			       const struct i2c_device_id *id)
{
	struct device_node *bus_np, *nodes_np, *np;
	struct device *busdev, *dev = &i2c->dev;
	struct platform_device *slave_pdev[16];
	struct ad242x_master *master;
	struct regmap *regmap;
	unsigned int val;
	int ret, i, n;

	nodes_np = of_get_child_by_name(dev->of_node, "nodes");
	if (!nodes_np) {
		dev_err(dev, "no 'nodes' property given\n");
		return -EINVAL;
	}

	bus_np = of_parse_phandle(dev->of_node, "adi,a2b-bus", 0);
	if (!bus_np) {
		dev_err(dev, "no 'adi,a2b-bus' handle specified for master node\n");
		return -EINVAL;
	}

	busdev = bus_find_device_by_of_node(&i2c_bus_type, bus_np);
	if (!busdev) {
		dev_err(dev, "'adi,a2b-bus' handle invalid\n");
		return -EINVAL;
	}

	master = devm_kzalloc(dev, sizeof(struct ad242x_master), GFP_KERNEL);
	if (!master)
		return -ENOMEM;

	mutex_init(&master->mutex);
	mutex_init(&master->bus.mutex);
	init_completion(&master->run_completion);
	init_completion(&master->discover_completion);
	dev_set_drvdata(dev, &master->node);
	i2c_set_clientdata(i2c, master);

	regmap = devm_regmap_init_i2c(i2c, &ad242x_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "regmap init failed: %d\n", ret);
		return ret;
	}

	master->bus.client = to_i2c_client(busdev);
	master->node.regmap = regmap;
	master->node.dev = dev;
	master->node.master = master;
	master->node.id = AD242X_MASTER_ID;
	master->irq = i2c->irq;

	master->sync_clk = devm_clk_get(dev, "sync");
	if (IS_ERR(master->sync_clk)) {
		ret = PTR_ERR(master->sync_clk);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to get sync clk: %d\n", ret);

		return ret;
	}

	if (of_property_read_u32(dev->of_node, "clock-frequency",
				 &master->sync_clk_rate)) {
		ret = clk_set_rate(master->sync_clk, master->sync_clk_rate);
		if (ret < 0) {
			dev_err(dev, "Cannot set sync clock rate: %d\n", ret);
			return ret;
		}
	}

	master->sync_clk_rate = clk_get_rate(master->sync_clk);
	if (master->sync_clk_rate != 44100 && master->sync_clk_rate != 48000) {
		dev_err(dev, "SYNC clock rate %d is invalid\n",
			master->sync_clk_rate);
		return -EINVAL;
	}

	ret = clk_prepare_enable(master->sync_clk);
	if (ret < 0) {
		dev_err(dev, "failed to enable sync clk: %d\n", ret);
		return ret;
	}

	if (master->irq > 0) {
		ret = devm_request_threaded_irq(dev, master->irq, NULL,
						ad242x_handle_irq, IRQF_ONESHOT,
						dev_name(dev), master);
		if (ret < 0) {
			dev_err(dev, "IRQ request failed: %d\n", ret);
			return ret;
		}
	}

	/* Master node setup */

	ret = regmap_write(regmap, AD242X_CONTROL,
			   AD242X_CONTROL_MSTR | AD242X_CONTROL_SOFTRST);
	if (ret < 0)
		return ret;

	ret = ad242x_wait_for_irq(master, &master->run_completion, 100);
	if (ret < 0) {
		dev_err(dev, "timeout waiting for PLL sync: %d\n", ret);
		return ret;
	}

	ret = regmap_update_bits(regmap, AD242X_CONTROL,
				 AD242X_CONTROL_SOFTRST, 0);
	if (ret < 0)
		return ret;

	ret = ad242x_node_probe(&master->node);
	if (ret < 0)
		return ret;

	ret = ad242x_init_irq(master);
	if (ret < 0) {
		dev_err(dev, "Unable to set up IRQ: %d", ret);
		return ret;
	}

	/* Slot format setup */

	of_property_read_u32(dev->of_node, "adi,upstream-slot-size", &val);
	if (val < 8 || val > 32 || (val % 4 != 0)) {
		dev_err(dev, "invalid upstream-slot-size %d\n", val);
		return -EINVAL;
	}
	master->up_slot_size = val;

	of_property_read_u32(dev->of_node, "adi,downstream-slot-size", &val);
	if (val < 8 || val > 32 || (val % 4 != 0)) {
		dev_err(dev, "invalid downstream-slot-size %d\n", val);
		return -EINVAL;
	}
	master->dn_slot_size = val;

	master->dn_slot_alt_fmt =
		of_property_read_bool(dev->of_node,
				      "adi,alternate-downstream-slot-format");
	master->up_slot_alt_fmt =
		of_property_read_bool(dev->of_node,
				      "adi,alternate-upstream-slot-format");

	/* Node discovery and MFD setup */

	ret = ad242x_discover(master, nodes_np);
	if (ret < 0) {
		dev_err(dev, "error discovering nodes: %d\n", ret);
		return ret;
	}

	ret = ad242x_node_add_mfd_cells(dev);
	if (ret < 0) {
		dev_err(dev, "failed to add MFD devices %d\n", ret);
		return ret;
	}

	/* Register platform devices for nodes */
	n =  0;

	for_each_available_child_of_node(nodes_np, np) {
		slave_pdev[n++] = of_platform_device_create(np, NULL, dev);
	}

	val = AD242X_SLOTFMT_DNSIZE(master->dn_slot_size) |
	      AD242X_SLOTFMT_UPSIZE(master->up_slot_size);

	if (master->dn_slot_alt_fmt)
		val |= AD242X_SLOTFMT_DNFMT;

	if (master->up_slot_alt_fmt)
		val |= AD242X_SLOTFMT_UPFMT;

	ret = regmap_write(regmap, AD242X_SLOTFMT, val);
	if (ret < 0)
		return ret;

	val = 0;
	if (master->dn_enabled)
		val |= AD242X_DATCTL_DNS;

	if (master->up_enabled)
		val |= AD242X_DATCTL_UPS;

	ret = regmap_write(regmap, AD242X_DATCTL, val);
	if (ret < 0)
		return ret;

	ret = ad242x_newstrct(master);
	if (ret < 0)
		return ret;

	for (i = 0; i < n; i++) {
		ret = ad242x_node_add_mfd_cells(&slave_pdev[i]->dev);
		if (ret < 0) {
			dev_err(dev, "failed to add MFD devices %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int ad242x_master_remove(struct i2c_client *i2c)
{
	struct ad242x_master *master = i2c_get_clientdata(i2c);

	if (master->sync_clk)
		clk_disable_unprepare(master->sync_clk);

	return 0;
}

static const struct of_device_id ad242x_master_of_match[] = {
	{ .compatible = "adi,ad2428w-master" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad242x_master_of_match);

static const struct i2c_device_id ad242x_master_i2c_id[] = {
	{"ad242x-master", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ad242x_master_i2c_id);

static struct i2c_driver ad242x_master_i2c_driver = {
	.driver	= {
		.name = "ad242x-master",
		.of_match_table	= ad242x_master_of_match,
	},
	.probe = ad242x_master_probe,
	.remove = ad242x_master_remove,
	.id_table = ad242x_master_i2c_id,
};

module_i2c_driver(ad242x_master_i2c_driver);

MODULE_DESCRIPTION("AD242x master master driver");
MODULE_AUTHOR("Daniel Mack <daniel@zonque.org>");
MODULE_LICENSE("GPL v2");
