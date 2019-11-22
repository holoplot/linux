// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>

static int dummy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk *clk;
	int ret;

	clk = devm_clk_get(dev, "input");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to get clk: %d\n", ret);

		return ret;
	}

	ret = clk_set_rate(clk, 12288000);
	if (ret < 0)
		dev_err(dev, "Cannot set rate: %d\n", ret);

	return 0;
}

static const struct of_device_id dummy_dt_ids[] = {
	{ .compatible = "dummy-clock-consumer", },
	{}
};
MODULE_DEVICE_TABLE(of, dummy_dt_ids);

static struct platform_driver dummy_driver = {
	.probe = dummy_probe,
	.driver = {
		.name = "dummy-clock-consumer",
		.of_match_table	= dummy_dt_ids,
	},
};
module_platform_driver(dummy_driver);

MODULE_AUTHOR("Daniel Mack <daniel@zonque.org>");
MODULE_DESCRIPTION("dummy clock consumer driver");
MODULE_LICENSE("GPL v2");
