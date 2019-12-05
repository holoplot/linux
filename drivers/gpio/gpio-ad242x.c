// SPDX-License-Identifier: GPL-2.0-only

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/ad242x.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

struct ad242x_gpio {
	struct ad242x_node *node;
	struct gpio_chip chip;
	struct irq_chip irq_chip;
	struct mutex irq_buslock;
	u8 irq_mask, irq_inv;
	u32 gpio_od_mask;
};

static int ad242x_gpio_request(struct gpio_chip *chip, unsigned int gpio)
{
	struct ad242x_gpio *ad242x_gpio = gpiochip_get_data(chip);

	if (gpio == 0 && ad242x_node_is_master(ad242x_gpio->node))
		return -EBUSY;

	if (ad242x_gpio->gpio_od_mask & BIT(gpio))
		return -EBUSY;

	return 0;
}

static int ad242x_gpio_get_value(struct gpio_chip *chip, unsigned int gpio)
{
	struct ad242x_gpio *ad242x_gpio = gpiochip_get_data(chip);
	struct regmap *regmap = ad242x_gpio->node->regmap;
	unsigned int val;
	int ret;

	ret = regmap_read(regmap, AD242X_GPIODAT_IN, &val);
	if (ret < 0)
		return ret;

	return !!(val & BIT(gpio));
}

static void ad242x_gpio_set_value(struct gpio_chip *chip,
				  unsigned int gpio, int value)
{
	struct ad242x_gpio *ad242x_gpio = gpiochip_get_data(chip);
	struct regmap *regmap = ad242x_gpio->node->regmap;
	uint8_t bit = BIT(gpio);
	int ret;

	if (value)
		ret = regmap_write(regmap, AD242X_GPIODAT_SET, bit);
	else
		ret = regmap_write(regmap, AD242X_GPIODAT_CLR, bit);

	if (ret < 0)
		dev_err(ad242x_gpio->node->dev,
			"Unable to set GPIO #%d: %d\n", gpio, ret);
}

static int ad242x_gpio_direction_input(struct gpio_chip *chip,
				       unsigned int gpio)
{
	struct ad242x_gpio *ad242x_gpio = gpiochip_get_data(chip);
	struct regmap *regmap = ad242x_gpio->node->regmap;
	uint8_t bit = BIT(gpio);
	int ret;

	ret = regmap_update_bits(regmap, AD242X_GPIOOEN, bit, 0);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(regmap, AD242X_GPIOIEN, bit, bit);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad242x_gpio_direction_output(struct gpio_chip *chip,
					unsigned int gpio, int value)
{
	struct ad242x_gpio *ad242x_gpio = gpiochip_get_data(chip);
	struct regmap *regmap = ad242x_gpio->node->regmap;
	uint8_t bit = BIT(gpio);
	int ret;

	ret = regmap_update_bits(regmap, AD242X_GPIOIEN, bit, 0);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(regmap, AD242X_GPIOOEN, bit, bit);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(regmap, AD242X_INTMSK1, bit, 0);
	if (ret < 0)
		return ret;

	ad242x_gpio_set_value(chip, gpio, value);

	return 0;
}

static int ad242x_gpio_over_distance_init(struct device *dev,
					  struct ad242x_gpio *ad242x_gpio)
{
	struct regmap *regmap = ad242x_gpio->node->regmap;
	struct device_node *np, *child_np;
	int ret = 0;

	np = of_get_child_by_name(dev->of_node, "gpio-over-distance");
	if (!np)
		return 0;

	for_each_available_child_of_node(np, child_np) {
		u32 reg, port_mask, bit;
		bool output, inv;

		ret = of_property_read_u32(child_np, "reg", &reg);
		if (ret < 0)
			continue;

		ret = of_property_read_u32(child_np, "adi,virtual-port-mask",
					   &port_mask);
		if (ret < 0)
			continue;

		if (reg > 7) {
			ret = -EINVAL;
			break;
		}

		bit = BIT(reg);

		ret = regmap_update_bits(regmap, AD242X_GPIODEN, bit, bit);
		if (ret < 0)
			break;

		ret = regmap_write(regmap, AD242X_GPIOD_MSK(reg), port_mask);
		if (ret < 0)
			break;

		output = of_property_read_bool(child_np, "adi,gpio-output");
		ret = regmap_update_bits(regmap, AD242X_GPIOOEN,
					 bit, output ? bit : 0);
		if (ret < 0)
			break;

		inv = of_property_read_bool(child_np, "adi,gpio-inverted");
		ret = regmap_update_bits(regmap, AD242X_GPIODINV,
					 bit, inv ? bit : 0);
		if (ret < 0)
			break;

		ad242x_gpio->gpio_od_mask |= bit;
		dev_info(dev,
			 "pin %d set up as gpio-over-distance, port mask 0x%02x\n",
			 reg, port_mask);
	}

	of_node_put(np);

	return ret;
}

static void ad242x_gpio_irq_mask(struct irq_data *irq_data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(irq_data);
	struct ad242x_gpio *ad242x_gpio = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(irq_data);

	ad242x_gpio->irq_mask |= BIT(hwirq);
}

static void ad242x_gpio_irq_unmask(struct irq_data *irq_data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(irq_data);
	struct ad242x_gpio *ad242x_gpio = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(irq_data);

	ad242x_gpio->irq_mask &= ~BIT(hwirq);
}

static int ad242x_gpio_set_irq_type(struct irq_data *irq_data,
				    unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(irq_data);
	struct ad242x_gpio *ad242x_gpio = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(irq_data);

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		ad242x_gpio->irq_inv &= ~BIT(hwirq);
		break;

	case IRQ_TYPE_EDGE_FALLING:
		ad242x_gpio->irq_inv |= BIT(hwirq);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static void ad242x_gpio_bus_lock(struct irq_data *irq_data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(irq_data);
	struct ad242x_gpio *ad242x_gpio = gpiochip_get_data(gc);

	mutex_lock(&ad242x_gpio->irq_buslock);
}

static void ad242x_gpio_bus_sync_unlock(struct irq_data *irq_data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(irq_data);
	struct ad242x_gpio *ad242x_gpio = gpiochip_get_data(gc);
	struct regmap *regmap = ad242x_gpio->node->regmap;
	struct device *dev = ad242x_gpio->node->dev;
	u8 irq_enable = ~ad242x_gpio->irq_mask;
	int ret;

	ret = regmap_update_bits(regmap, AD242X_GPIOOEN, irq_enable, 0);
	if (ret < 0)
		dev_err(dev, "Error writing GPIOOEN: %d\n", ret);

	ret = regmap_update_bits(regmap, AD242X_GPIOIEN, irq_enable, irq_enable);
	if (ret < 0)
		dev_err(dev, "Error writing GPIOIEN: %d\n", ret);

	ret = regmap_write(regmap, AD242X_PINTEN, irq_enable);
	if (ret < 0)
		dev_err(dev, "Error writing IRQ mask: %d\n", ret);

	ret = regmap_write(regmap, AD242X_INTMSK1, irq_enable);
	if (ret < 0)
		dev_err(dev, "Error writing IRQ mask: %d\n", ret);

	ret = regmap_write(regmap, AD242X_PINTINV, ad242x_gpio->irq_inv);
	if (ret < 0)
		dev_err(dev, "Error writing IRQ inversion: %d\n", ret);

	mutex_unlock(&ad242x_gpio->irq_buslock);
}

static irqreturn_t ad242x_gpio_irq_handler(int irq, void *dev_id)
{
	struct ad242x_gpio *ad242x_gpio = dev_id;
	struct regmap *regmap = ad242x_gpio->node->regmap;
	struct gpio_chip *gc = &ad242x_gpio->chip;
	struct device *dev = gc->parent;
	unsigned int val, index;
	u8 inttype;
	int ret;

	inttype = ad242x_node_inttype(ad242x_gpio->node);

	if (inttype < AD242X_INTTYPE_IO0PND || inttype > AD242X_INTTYPE_IO7PND)
		return IRQ_NONE;

	index = inttype - AD242X_INTTYPE_IO0PND;

	ret = regmap_read(regmap, AD242X_INTPND1, &val);
	if (ret < 0) {
		dev_err(dev, "Failed to read pending IRQs: %d\n", ret);
		return IRQ_NONE;
	}

	ret = regmap_write(regmap, AD242X_INTPND1, val);
	if (ret < 0) {
		dev_err(dev, "Failed to clear IRQs: %d\n", ret);
		return IRQ_NONE;
	}

	if (!(ad242x_gpio->irq_mask & BIT(irq))) {
		unsigned int virq;
		virq = irq_find_mapping(gc->irq.domain, index);
		handle_nested_irq(virq);
	}

	return IRQ_HANDLED;
}

static int ad242x_gpio_irq_init(struct device *dev,
				struct ad242x_gpio *ad242x_gpio)
{
	struct irq_chip *ic = &ad242x_gpio->irq_chip;
	struct gpio_chip *gc = &ad242x_gpio->chip;
	int ret, irq;

	if (!of_property_read_bool(dev->of_node, "interrupt-controller"))
		return 0;

	irq = of_irq_get(dev->of_node, 0);
	if (irq < 0)
		return ret;

	ic->name = dev_name(dev);
	ic->irq_mask = ad242x_gpio_irq_mask;
	ic->irq_unmask = ad242x_gpio_irq_unmask;
	ic->irq_set_type = ad242x_gpio_set_irq_type;
	ic->irq_bus_lock = ad242x_gpio_bus_lock,
	ic->irq_bus_sync_unlock = ad242x_gpio_bus_sync_unlock,

	gc->irq.chip = ic;
	gc->irq.default_type = IRQ_TYPE_NONE;
	gc->irq.handler = handle_edge_irq;

	mutex_init(&ad242x_gpio->irq_buslock);
	ad242x_gpio->irq_mask = 0xff;

	ret = devm_request_threaded_irq(dev, irq,
					NULL, ad242x_gpio_irq_handler,
					IRQF_ONESHOT | IRQF_SHARED,
					ic->name, ad242x_gpio);
	if (ret) {
		dev_err(dev, "failed to request irq: %d\n", irq);
		return ret;
	}

	return 0;
}

static int ad242x_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ad242x_gpio *ad242x_gpio;
	struct gpio_chip *gc;
	int ret;

	if (!dev->of_node)
		return -ENODEV;

	ad242x_gpio = devm_kzalloc(dev, sizeof(*ad242x_gpio), GFP_KERNEL);
	if (!ad242x_gpio)
		return -ENOMEM;

	ad242x_gpio->node = dev_get_drvdata(dev->parent);

	gc = &ad242x_gpio->chip;
	gc->request = ad242x_gpio_request;
	gc->direction_input = ad242x_gpio_direction_input;
	gc->direction_output = ad242x_gpio_direction_output;
	gc->get = ad242x_gpio_get_value;
	gc->set = ad242x_gpio_set_value;
	gc->can_sleep = 1;
	gc->base = -1;
	gc->ngpio = 8;
	gc->label = "ad242x-gpio";
	gc->owner = THIS_MODULE;
	gc->parent = dev;

	dev_info(dev, "A2B node ID %d\n", ad242x_gpio->node->id);

	ret = ad242x_gpio_over_distance_init(dev, ad242x_gpio);
	if (ret < 0) {
		dev_err(dev, "GPIO over distance init failed: %d\n", ret);
		return ret;
	}

	ret = ad242x_gpio_irq_init(dev, ad242x_gpio);
	if (ret < 0)
		return ret;

	return devm_gpiochip_add_data(dev, gc, ad242x_gpio);
}

static const struct of_device_id ad242x_gpio_of_match[] = {
	{ .compatible = "adi,ad2428w-gpio", },
	{}
};
MODULE_DEVICE_TABLE(of, ad242x_gpio_of_match);

static struct platform_driver ad242x_gpio_driver = {
	.driver = {
		.name = "ad242x-gpio",
		.of_match_table = ad242x_gpio_of_match,
	},
	.probe = ad242x_gpio_probe,
};
module_platform_driver(ad242x_gpio_driver);

MODULE_DESCRIPTION("AD242x GPIO driver");
MODULE_AUTHOR("Daniel Mack <daniel@zonque.org>");
MODULE_LICENSE("GPL v2");
