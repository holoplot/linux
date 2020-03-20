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
	struct gpio_chip chip;
	struct ad242x_node *node;
	struct irq_chip irq_chip;
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

	ret = regmap_update_bits(regmap, AD242X_INTMSK1, bit, bit);
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

static void ad242x_gpio_irq_enable(struct irq_data *irq_data, bool enable)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(irq_data);
	struct ad242x_gpio *ad242x_gpio = gpiochip_get_data(gc);
	struct regmap *regmap = ad242x_gpio->node->regmap;
	irq_hw_number_t hwirq = irqd_to_hwirq(irq_data);
	uint8_t bit = BIT(hwirq);
	int ret;

	ret = regmap_update_bits(regmap, AD242X_PINTEN, bit, enable ? bit : 0);
	if (ret < 0)
		dev_err(ad242x_gpio->node->dev,
			"Error %sabling IRQ %d: %d\n",
			enable ? "en" : "dis", bit, ret);
}

static void ad242x_gpio_irq_mask(struct irq_data *irq_data)
{
	ad242x_gpio_irq_enable(irq_data, false);
}

static void ad242x_gpio_irq_unmask(struct irq_data *irq_data)
{
	ad242x_gpio_irq_enable(irq_data, true);
}

static int ad242x_gpio_set_irq_type(struct irq_data *irq_data,
				    unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(irq_data);
	struct ad242x_gpio *ad242x_gpio = gpiochip_get_data(gc);
	struct regmap *regmap = ad242x_gpio->node->regmap;
	irq_hw_number_t hwirq = irqd_to_hwirq(irq_data);
	uint8_t bit = BIT(hwirq);
	int ret;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		ret = regmap_update_bits(regmap, AD242X_PINTINV, bit, 0);
		break;

	case IRQ_TYPE_EDGE_FALLING:
		ret = regmap_update_bits(regmap, AD242X_PINTINV, bit, bit);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static irqreturn_t ad242x_gpio_irq_handler(int irq, void *devid)
{
	struct ad242x_gpio *ad242x_gpio = devid;
	struct regmap *regmap = ad242x_gpio->node->regmap;
	struct gpio_chip *gc = &ad242x_gpio->chip;
	struct device *dev = gc->parent;
	unsigned int pending, index;
	int ret;

	ret = regmap_read(regmap, AD242X_INTPND1, &pending);
	if (ret < 0) {
		dev_err(dev, "Failed to read pending IRQs: %d\n", ret);
		return IRQ_NONE;
	}

	for (index = 0; index < gc->ngpio; index++) {
		if (pending & BIT(index)) {
			unsigned int mapping =
				irq_find_mapping(gc->irq.domain, index);

			handle_nested_irq(mapping);
		}
	}

	ret = regmap_write(regmap, AD242X_INTPND1, pending);
	if (ret < 0) {
		dev_err(dev, "Failed to clear IRQs: %d\n", ret);
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static int ad242x_gpio_irq_init(struct device *dev,
				struct ad242x_gpio *ad242x_gpio)
{
	struct irq_chip *irq_chip = &ad242x_gpio->irq_chip;
	int ret, irq;

	if (!of_property_read_bool(dev->of_node, "interrupt-controller"))
		return 0;

	irq = of_irq_get(dev->of_node, 0);
	if (irq == -EPROBE_DEFER)
		return ret;

	if (irq < 0) {
		dev_err(dev, "Failed to lookup IRQ: %d\n", ret);
		return ret;
	}

	ret = devm_request_threaded_irq(dev, irq,
					NULL, ad242x_gpio_irq_handler,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT |
					IRQF_SHARED, dev_name(dev),
					ad242x_gpio);
	if (ret < 0) {
		dev_err(dev, "Failed to request IRQ: %d\n", ret);
		return ret;
	}

	irq_chip->irq_mask = ad242x_gpio_irq_mask;
	irq_chip->irq_unmask = ad242x_gpio_irq_unmask;
	irq_chip->irq_set_type = ad242x_gpio_set_irq_type;
	irq_chip->name = dev_name(dev);

	ret = gpiochip_irqchip_add_nested(&ad242x_gpio->chip, irq_chip,
					  0, handle_simple_irq,
					  IRQ_TYPE_NONE);
	if (ret < 0) {
		dev_err(dev, "Failed to connect IRQ to GPIO chip: %d\n", ret);
		return ret;
	}

	gpiochip_set_nested_irqchip(&ad242x_gpio->chip, irq_chip, irq);

	return 0;
}

static int ad242x_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ad242x_gpio *ad242x_gpio;
	int ret;

	if (!dev->of_node)
		return -ENODEV;

	ad242x_gpio = devm_kzalloc(dev, sizeof(*ad242x_gpio), GFP_KERNEL);
	if (!ad242x_gpio)
		return -ENOMEM;

	ad242x_gpio->node = dev_get_drvdata(dev->parent);

	ad242x_gpio->chip.request = ad242x_gpio_request;
	ad242x_gpio->chip.direction_input = ad242x_gpio_direction_input;
	ad242x_gpio->chip.direction_output = ad242x_gpio_direction_output;
	ad242x_gpio->chip.get = ad242x_gpio_get_value;
	ad242x_gpio->chip.set = ad242x_gpio_set_value;
	ad242x_gpio->chip.can_sleep = 1;
	ad242x_gpio->chip.base = -1;
	ad242x_gpio->chip.ngpio = 8;
	ad242x_gpio->chip.label = "ad242x-gpio";
	ad242x_gpio->chip.owner = THIS_MODULE;
	ad242x_gpio->chip.parent = dev;

	dev_info(dev, "A2B node ID %d\n", ad242x_gpio->node->id);

	ret = ad242x_gpio_over_distance_init(dev, ad242x_gpio);
	if (ret < 0) {
		dev_err(dev, "GPIO over distance init failed: %d\n", ret);
		return ret;
	}

	ret = devm_gpiochip_add_data(dev, &ad242x_gpio->chip, ad242x_gpio);
	if (ret < 0)
		return ret;

	return ad242x_gpio_irq_init(dev, ad242x_gpio);
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
