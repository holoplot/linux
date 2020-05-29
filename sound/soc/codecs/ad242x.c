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

struct ad242x_private {
	struct ad242x_node	*node;
	struct clk		*mclk;
	unsigned int		inv_fmt;
	bool			pdm[2];
	bool			pdm_highpass;
};

static const struct snd_soc_dapm_widget ad242x_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("RX0",  NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("RX1",  NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TX0", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TX1", NULL, 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route ad242x_dapm_routes[] = {
	{ "DAI0 Playback", NULL, "RX0" },
	{ "TX0", NULL, "DAI0 Capture"  },
	{ "DAI1 Playback", NULL, "RX1" },
	{ "TX1", NULL, "DAI1 Capture"  },
};

static int ad242x_set_dai_fmt(struct snd_soc_dai *dai,
			      unsigned int format)
{
	struct snd_soc_component *component = dai->component;
	struct ad242x_private *priv = snd_soc_component_get_drvdata(component);

	/* set DAI format */
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

	/*
	 * Setting clock inversion is only supported globally for both DAIs,
	 * so we require the settings for DAI1 to match those for DAI0.
	 */
	if (dai->id == 0) {
		priv->inv_fmt = format & SND_SOC_DAIFMT_INV_MASK;
	} else {
		if ((format & SND_SOC_DAIFMT_INV_MASK) != priv->inv_fmt) {
			dev_err(component->dev,
				"inversion masks for both dai must be identical\n");
			return -EINVAL;			
		}
	}

	if (ad242x_node_is_master(priv->node) &&
	   ((format & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS)) {
		dev_err(component->dev, "master node must be clock slave\n");
		return -EINVAL;
	}

	if (!ad242x_node_is_master(priv->node) &&
	   ((format & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBM_CFM)) {
		dev_err(component->dev, "slave node must be clock master\n");
		return -EINVAL;
	}

	return 0;
}

static int ad242x_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct ad242x_private *priv = snd_soc_component_get_drvdata(component);
	unsigned int sff_rate = ad242x_master_get_clk_rate(priv->node->master);
	unsigned int rate = params_rate(params);
	unsigned int val, mask;
	int ret;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		if (priv->node->tdm_slot_size != 16)
			return -EINVAL;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		if (priv->node->tdm_slot_size != 32)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	if (priv->pdm[dai->id]) {
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

		ret = regmap_update_bits(priv->node->regmap, AD242X_PDMCTL,
					 mask, val);
		if (ret < 0)
			return ret;
	} else {
		if (params_channels(params) != priv->node->tdm_mode)
			return -EINVAL;

		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			if (dai->id == 0)
				mask = AD242X_I2SCTL_RX0EN;
			else
				mask = AD242X_I2SCTL_RX1EN;
		} else {
			if (dai->id == 0)
				mask = AD242X_I2SCTL_TX0EN;
			else
				mask = AD242X_I2SCTL_TX1EN;
		}

		ret = regmap_update_bits(priv->node->regmap, AD242X_I2SCTL, mask, mask);
		if (ret < 0)
			return ret;
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		mask = AD242X_I2SCTL_RXBCLKINV;
	else
		mask = AD242X_I2SCTL_TXBCLKINV;

	switch (priv->inv_fmt) {
	case SND_SOC_DAIFMT_NB_NF:
		val = 0;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		val = mask;
		break;
	case SND_SOC_DAIFMT_NB_IF:
	case SND_SOC_DAIFMT_IB_IF:
		dev_err(component->dev, "unsupported inversion mask\n");
		return -EINVAL;
	}

	ret = regmap_update_bits(priv->node->regmap, AD242X_I2SCTL, mask, val);
	if (ret < 0)
		return ret;

	if (!ad242x_node_is_master(priv->node)) {
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

		ret = regmap_write(priv->node->regmap, AD242X_I2SRATE, val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static const struct snd_soc_dai_ops ad242x_dai_ops = {
	.hw_params	= ad242x_hw_params,
	.set_fmt	= ad242x_set_dai_fmt,
};

#define AD242X_RATES (					\
	SNDRV_PCM_RATE_22050  | SNDRV_PCM_RATE_32000  |	\
	SNDRV_PCM_RATE_44100  | SNDRV_PCM_RATE_48000  |	\
	SNDRV_PCM_RATE_88200  | SNDRV_PCM_RATE_96000  |	\
	SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000)
#define AD242X_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver ad242x_dai[] = {
	{
		.name = "ad242x-dai0",
		.playback = {
			.stream_name	= "DAI0 Playback",
			.channels_min	= 1,
			.channels_max	= 32,
			.rates		= AD242X_RATES,
			.formats	= AD242X_FORMATS,
		},
		.capture = {
			.stream_name	= "DAI0 Capture",
			.channels_min	= 1,
			.channels_max	= 32,
			.rates		= AD242X_RATES,
			.formats	= AD242X_FORMATS,
		},
		.ops = &ad242x_dai_ops,
	},
	{
		.name = "ad242x-dai1",
		.playback = {
			.stream_name	= "DAI1 Playback",
			.channels_min	= 1,
			.channels_max	= 32,
			.rates		= AD242X_RATES,
			.formats	= AD242X_FORMATS,
		},
		.capture = {
			.stream_name	= "DAI1 Capture",
			.channels_min	= 1,
			.channels_max	= 32,
			.rates		= AD242X_RATES,
			.formats	= AD242X_FORMATS,
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
	.probe			= ad242x_soc_probe,
	.remove			= ad242x_soc_remove,
	.dapm_widgets		= ad242x_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(ad242x_dapm_widgets),
	.dapm_routes		= ad242x_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(ad242x_dapm_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static int ad242x_codec_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct ad242x_private *priv;
	u32 mclk_freq;
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
				dev_err(dev, "Cannot set mclk frequency %d: %d\n",
					mclk_freq, ret);
				return ret;
			}
		}
	}

	priv->node = dev_get_drvdata(dev->parent);
	platform_set_drvdata(pdev, priv);

	priv->pdm_highpass =
		of_property_read_bool(np, "adi,pdm-highpass-filter");

	// HACK
	if (!ad242x_node_is_master(priv->node))
		regmap_write(priv->node->regmap, AD242X_CLK2CFG, 0x01);

	return devm_snd_soc_register_component(dev,
					       &soc_component_device_ad242x,
					       ad242x_dai,
					       ARRAY_SIZE(ad242x_dai));
}

static const struct of_device_id ad242x_of_match[] = {
	{ .compatible = "adi,ad2428w-codec", },
	{ }
};
MODULE_DEVICE_TABLE(of, ad242x_of_match);

static struct platform_driver ad242x_platform_driver = {
	.driver  = {
		.name   = "ad242x-codec",
		.of_match_table = ad242x_of_match,
	},
	.probe  = ad242x_codec_platform_probe,
};

module_platform_driver(ad242x_platform_driver);

MODULE_AUTHOR("Daniel Mack <daniel@zonque.org>");
MODULE_DESCRIPTION("AD242X ALSA SoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ad242x-codec");
