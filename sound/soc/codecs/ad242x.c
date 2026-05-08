// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mfd/ad242x.h>
#include <linux/of_device.h>
#include <linux/slab.h>

#include <sound/asoundef.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define AD242X_NUM_DAIS 2

struct ad242x_private {
	struct ad242x_node *node;
	struct clk *mclk;

	bool pdm[AD242X_NUM_DAIS];
	bool pdm_highpass;

	struct snd_soc_dai_driver *dai_drv;
};

static const struct snd_soc_dapm_widget ad242x_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("RX0", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("RX1", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TX0", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TX1", NULL, 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route ad242x_dapm_routes[] = {
	{ "DAI0 Playback", NULL, "RX0" },
	{ "TX0", NULL, "DAI0 Capture" },
	{ "DAI1 Playback", NULL, "RX1" },
	{ "TX1", NULL, "DAI1 Capture" },
};

static bool ad242x_valid_dai_id(struct snd_soc_dai *dai)
{
	return dai->id >= 0 && dai->id < AD242X_NUM_DAIS;
}

static int ad242x_set_i2s_rate(struct ad242x_private *priv, unsigned int rate)
{
	unsigned int sff_rate;
	unsigned int val;

	if (ad242x_node_is_master(priv->node))
		return 0;

	sff_rate = ad242x_master_get_clk_rate(priv->node->master);
	val = 0;

	if (rate == sff_rate / 2)
		val = AD242X_I2SRATE_I2SRATE(1);
	else if (rate == sff_rate / 4)
		val = AD242X_I2SRATE_I2SRATE(2);
	else if (rate == sff_rate * 2)
		val = AD242X_I2SRATE_I2SRATE(5);
	else if (rate == sff_rate * 4)
		val = AD242X_I2SRATE_I2SRATE(6);
	else if (rate != sff_rate)
		return -EINVAL;

	return regmap_write(priv->node->regmap, AD242X_I2SRATE, val);
}

static int ad242x_set_dai_fmt(struct snd_soc_dai *dai, unsigned int format)
{
	struct snd_soc_component *component = dai->component;
	struct ad242x_private *priv = snd_soc_component_get_drvdata(component);

	if (!ad242x_valid_dai_id(dai))
		return -EINVAL;

	switch (format & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		priv->pdm[dai->id] = false;
		break;

	case SND_SOC_DAIFMT_PDM:
		priv->pdm[dai->id] = true;
		break;

	default:
		dev_err(component->dev, "unsupported dai format\n");
		return -EINVAL;
	}

	switch (format & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
	case SND_SOC_DAIFMT_IB_NF:
		break;

	case SND_SOC_DAIFMT_NB_IF:
	case SND_SOC_DAIFMT_IB_IF:
	default:
		dev_err(component->dev, "unsupported inversion mask\n");
		return -EINVAL;
	}

	if (ad242x_node_is_master(priv->node) &&
	    ((format & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBC_CFC)) {
		dev_err(component->dev, "master node must be clock slave\n");
		return -EINVAL;
	}

	if (!ad242x_node_is_master(priv->node) &&
	    ((format & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBP_CFP)) {
		dev_err(component->dev, "slave node must be clock master\n");
		return -EINVAL;
	}

	return 0;
}

static int ad242x_pdm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct ad242x_private *priv = snd_soc_component_get_drvdata(component);
	unsigned int val;
	unsigned int mask;

	if (substream->stream != SNDRV_PCM_STREAM_CAPTURE)
		return -EINVAL;

	if (dai->id == 0) {
		val = AD242X_PDMCTL_PDM0EN;
		mask = AD242X_PDMCTL_PDM0EN | AD242X_PDMCTL_PDM0SLOTS;
	} else {
		val = AD242X_PDMCTL_PDM1EN;
		mask = AD242X_PDMCTL_PDM1EN | AD242X_PDMCTL_PDM1SLOTS;
	}

	switch (params_channels(params)) {
	case 1:
		break;

	case 2:
		val = mask;
		break;

	default:
		return -EINVAL;
	}

	mask |= AD242X_PDMCTL_HPFEN;

	if (priv->pdm_highpass)
		val |= AD242X_PDMCTL_HPFEN;

	return regmap_update_bits(priv->node->regmap, AD242X_PDMCTL, mask, val);
}

static int ad242x_pdm_hw_free(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct ad242x_private *priv = snd_soc_component_get_drvdata(component);
	unsigned int mask;

	if (substream->stream != SNDRV_PCM_STREAM_CAPTURE)
		return 0;

	if (dai->id == 0)
		mask = AD242X_PDMCTL_PDM0EN | AD242X_PDMCTL_PDM0SLOTS;
	else
		mask = AD242X_PDMCTL_PDM1EN | AD242X_PDMCTL_PDM1SLOTS;

	return regmap_update_bits(priv->node->regmap, AD242X_PDMCTL, mask, 0);
}

static int ad242x_i2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct ad242x_private *priv = snd_soc_component_get_drvdata(component);

	return ad242x_set_i2s_rate(priv, params_rate(params));
}

static int ad242x_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct ad242x_private *priv = snd_soc_component_get_drvdata(component);

	if (!ad242x_valid_dai_id(dai))
		return -EINVAL;

	if (priv->pdm[dai->id])
		return ad242x_pdm_hw_params(substream, params, dai);

	return ad242x_i2s_hw_params(substream, params, dai);
}

static int ad242x_hw_free(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct ad242x_private *priv = snd_soc_component_get_drvdata(component);

	if (!ad242x_valid_dai_id(dai))
		return -EINVAL;

	if (priv->pdm[dai->id])
		return ad242x_pdm_hw_free(substream, dai);

	return 0;
}

static const struct snd_soc_dai_ops ad242x_dai_ops = {
	.hw_params = ad242x_hw_params,
	.hw_free = ad242x_hw_free,
	.set_fmt = ad242x_set_dai_fmt,
};

/* FIXME: The current driver only advertises 48 kHz. */
#define AD242X_RATES (SNDRV_PCM_RATE_48000)

static const struct snd_soc_dai_driver ad242x_dai_template[] = {
	{
		.name = "ad242x-dai0",
		.id = 0,
		.playback = {
			.stream_name = "DAI0 Playback",
			.rates = AD242X_RATES,
		},
		.capture = {
			.stream_name = "DAI0 Capture",
			.rates = AD242X_RATES,
		},
		.ops = &ad242x_dai_ops,
	},
	{
		.name = "ad242x-dai1",
		.id = 1,
		.playback = {
			.stream_name = "DAI1 Playback",
			.rates = AD242X_RATES,
		},
		.capture = {
			.stream_name = "DAI1 Capture",
			.rates = AD242X_RATES,
		},
		.ops = &ad242x_dai_ops,
	},
};

static int ad242x_soc_probe(struct snd_soc_component *component)
{
	struct ad242x_private *priv = snd_soc_component_get_drvdata(component);

	component->regmap = priv->node->regmap;

	if (priv->mclk)
		return clk_prepare_enable(priv->mclk);

	return 0;
}

static void ad242x_soc_remove(struct snd_soc_component *component)
{
	struct ad242x_private *priv = snd_soc_component_get_drvdata(component);

	if (priv->mclk)
		clk_disable_unprepare(priv->mclk);
}

static const struct snd_soc_component_driver soc_component_device_ad242x = {
	.probe = ad242x_soc_probe,
	.remove = ad242x_soc_remove,
	.dapm_widgets = ad242x_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ad242x_dapm_widgets),
	.dapm_routes = ad242x_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(ad242x_dapm_routes),
	.idle_bias_on = 1,
	.use_pmdown_time = 1,
	.endianness = 1,
};

static int ad242x_codec_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct ad242x_private *priv;
	unsigned int channels;
	u32 mclk_freq;
	u64 formats;
	int i;
	int ret;

	if (!dev->of_node)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->mclk = devm_clk_get_optional(dev, "mclk");
	if (priv->mclk) {
		if (IS_ERR(priv->mclk)) {
			ret = PTR_ERR(priv->mclk);

			if (ret != -EPROBE_DEFER)
				dev_err(dev, "failed to get clk: %d\n", ret);

			return ret;
		}

		if (!of_property_read_u32(np, "clock-frequency", &mclk_freq)) {
			ret = clk_set_rate(priv->mclk, mclk_freq);
			if (ret < 0) {
				dev_err(dev,
					"Cannot set mclk frequency %d: %d\n",
					mclk_freq, ret);
				return ret;
			}
		}
	}

	priv->node = dev_get_drvdata(dev->parent);
	if (!priv->node)
		return -ENODEV;

	platform_set_drvdata(pdev, priv);

	priv->pdm_highpass =
		of_property_read_bool(np, "adi,pdm-highpass-filter");

	switch (priv->node->tdm_slot_size) {
	case 16:
		formats = SNDRV_PCM_FMTBIT_S16_LE;
		break;

	case 32:
		formats = SNDRV_PCM_FMTBIT_S32_LE;
		break;

	default:
		return -EINVAL;
	}

	channels = priv->node->tdm_mode;

	priv->dai_drv = devm_kmemdup(dev, ad242x_dai_template,
				     sizeof(ad242x_dai_template), GFP_KERNEL);
	if (!priv->dai_drv)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(ad242x_dai_template); i++) {
		priv->dai_drv[i].playback.formats = formats;
		priv->dai_drv[i].playback.channels_min = channels;
		priv->dai_drv[i].playback.channels_max = channels;

		priv->dai_drv[i].capture.formats = formats;
		priv->dai_drv[i].capture.channels_min = channels;
		priv->dai_drv[i].capture.channels_max = channels;
	}

	// HACK
	if (!ad242x_node_is_master(priv->node))
		regmap_write(priv->node->regmap, AD242X_CLK2CFG, 0x01);

	return devm_snd_soc_register_component(dev,
					       &soc_component_device_ad242x,
					       priv->dai_drv,
					       ARRAY_SIZE(ad242x_dai_template));
}

static const struct of_device_id ad242x_of_match[] = {
	{
		.compatible = "adi,ad2428w-codec",
	},
	{}
};
MODULE_DEVICE_TABLE(of, ad242x_of_match);

static struct platform_driver ad242x_platform_driver = {
	.driver = {
		.name = "ad242x-codec",
		.of_match_table = ad242x_of_match,
	},
	.probe = ad242x_codec_platform_probe,
};
module_platform_driver(ad242x_platform_driver);

MODULE_AUTHOR("Daniel Mack");
MODULE_DESCRIPTION("AD242X ALSA SoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ad242x-codec");
