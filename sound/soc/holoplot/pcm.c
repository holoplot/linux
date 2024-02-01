// SPDX-License-Identifier: GPL-2.0

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/dmaengine_pcm.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

struct holoplot_pcm_priv {
	struct snd_soc_dai_driver		dai;
	struct snd_dmaengine_dai_dma_data	capture_dma_data;
	struct snd_dmaengine_pcm_config 	dmaengine_config;
	struct snd_pcm_hardware			hw;
	struct gpio_desc			*dma_ready_gpio;
};

static int holoplot_pcm_dai_probe(struct snd_soc_dai *dai)
{
	struct holoplot_pcm_priv *priv = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, NULL, &priv->capture_dma_data);

	return 0;
}

static int holoplot_pcm_dai_trigger(struct snd_pcm_substream *substream,
				    int cmd, struct snd_soc_dai *dai)
{
	struct holoplot_pcm_priv *priv = snd_soc_dai_get_drvdata(dai);
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		gpiod_set_value(priv->dma_ready_gpio, 1);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		gpiod_set_value(priv->dma_ready_gpio, 0);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct snd_soc_dai_ops holoplot_pcm_dai_ops = {
	.probe		= holoplot_pcm_dai_probe,
	.trigger	= holoplot_pcm_dai_trigger,
};

static const struct snd_soc_component_driver holoplot_pcm_component = {
	.name = "holoplot-pcm",
	.legacy_dai_naming = 1,
};

static int holoplot_pcm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct holoplot_pcm_priv *priv;
	u32 channels;
	int ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(dev, "DMA mask failed: %d\n", ret);
		return ret;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	ret = of_property_read_u32(dev->of_node,
				   "holoplot,channels", &channels);
	if (ret < 0) {
		dev_err(dev, "missing channels property");
		return -EINVAL;
	}

	priv->dma_ready_gpio = devm_gpiod_get(dev, "dma-ready", GPIOD_OUT_LOW);
	if (IS_ERR(priv->dma_ready_gpio)) {
		ret = PTR_ERR(priv->dma_ready_gpio);
		dev_err(dev, "Unable to claim dma-ready GPIO: %d\n", ret);
		return ret;
	}

	priv->dai.capture.stream_name = "Capture";
	priv->dai.capture.channels_min = channels;
	priv->dai.capture.channels_max = channels;
	priv->dai.capture.rates = SNDRV_PCM_RATE_48000;
	priv->dai.capture.formats = SNDRV_PCM_FMTBIT_S16_LE;
	priv->dai.ops = &holoplot_pcm_dai_ops;
	priv->dai.symmetric_rate = 1;
	priv->dai.name = "x2-tdm";

	priv->hw.info			= SNDRV_PCM_INFO_MMAP |
					  SNDRV_PCM_INFO_MMAP_VALID |
					  SNDRV_PCM_INFO_PAUSE |
					  SNDRV_PCM_INFO_RESUME |
					  SNDRV_PCM_INFO_INTERLEAVED |
					  SNDRV_PCM_INFO_HALF_DUPLEX;
	priv->hw.period_bytes_min	= SZ_2K*32;
	priv->hw.period_bytes_max	= SZ_2K*32;
	priv->hw.periods_min		= 64;
	priv->hw.periods_max		= 64;
	priv->hw.buffer_bytes_max	= priv->hw.period_bytes_max
					* priv->hw.periods_max;
	priv->hw.fifo_size		= 32;

	priv->capture_dma_data.chan_name = "rx";
	priv->dmaengine_config.pcm_hardware = &priv->hw;
	priv->dmaengine_config.prealloc_buffer_size = priv->hw.buffer_bytes_max;
	priv->dmaengine_config.chan_names[SNDRV_PCM_STREAM_CAPTURE] = "rx";
	priv->dmaengine_config.prepare_slave_config =
		snd_dmaengine_pcm_prepare_slave_config;

	ret = devm_snd_soc_register_component(dev, &holoplot_pcm_component,
					      &priv->dai, 1);
	if (ret < 0) {
		dev_err_probe(dev, ret, "failed to register component\n");
		return ret;
	}

	ret = devm_snd_dmaengine_pcm_register(dev, &priv->dmaengine_config, 0);
	if (ret < 0) {
		dev_err_probe(dev, ret, "failed to register dmaengine\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id holoplot_pcm_of_match[] = {
	{ .compatible = "holoplot,pcm", },
	{},
};
MODULE_DEVICE_TABLE(of, holoplot_pcm_of_match);

static struct platform_driver holoplot_card = {
	.driver = {
		.name = "holoplot-pcm",
		.pm = &snd_soc_pm_ops,
		.of_match_table = holoplot_pcm_of_match,
	},
	.probe = holoplot_pcm_probe,
};
module_platform_driver(holoplot_card);

MODULE_ALIAS("platform:holoplot-pcm");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ASoC PCM support for Holoplot devices");
MODULE_AUTHOR("Daniel Mack <daniel.mack@holoplot.com>");
