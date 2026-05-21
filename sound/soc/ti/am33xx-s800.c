// SPDX-License-Identifier: GPL-2.0-only
/*
 * ASoC driver for StreamUnlimited S800/Raumfeld AM33xx based audio devices
 *
 *  (c) 2013 Daniel Mack <daniel@zonque.org>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define DATA_WORD_WIDTH 32

/* Mirror sound/soc/ti/davinci-mcasp.h since it is not exported. */
#define MCASP_CLKDIV_AUXCLK		0
#define MCASP_CLKDIV_BCLK		1
#define MCASP_CLKDIV_BCLK_FS_RATIO	2

struct snd_soc_am33xx_s800 {
	struct snd_soc_card	card;
	struct clk		*mclk;
	unsigned int		mclk_rate;
	unsigned int		mclk_rate_current;
	signed int		drift;
	struct gpio_desc	*passive_mode_gpiod;
	struct gpio_desc	*amp_overheat_gpiod;
	struct gpio_desc	*amp_overcurrent_gpiod;
	struct gpio_desc	*amp_reset_gpiod;
	int			amp_reset_delay_ms;
	struct snd_kcontrol	*amp_overheat_kctl;
	struct regulator	*regulator;
};

static int am33xx_s800_set_mclk(struct snd_soc_am33xx_s800 *priv)
{
	unsigned int drift;
	int sgn = priv->drift > 0 ? 1 : -1;
	signed long comp, clk;
	int ret;

	drift = priv->drift * sgn;
	comp = ((priv->mclk_rate_current / DATA_WORD_WIDTH) * drift) /
		(1000000ULL / DATA_WORD_WIDTH);
	comp *= sgn;
	clk = priv->mclk_rate_current - comp;

	if (clk_get_rate(priv->mclk) == clk)
		return 0;

	ret = clk_set_rate(priv->mclk, clk);
	if (ret < 0)
		return ret;

	/*
	 * The MCLK source (Si5351) reprograms its PLL on a rate change, which
	 * briefly glitches MCLK while the PLL relocks. The CS4271 control port
	 * NAKs I2C without a stable MCLK, so wait for the PLL to settle before
	 * the codec is accessed during the rest of hw_params/trigger.
	 */
	msleep(10);

	return 0;
}

static int snd_soc_am33xx_s800_set_control(struct snd_card *card,
					   const char *name,
					   const char *value)
{
	struct snd_kcontrol *ctl;
	struct snd_ctl_elem_value val;
	struct snd_ctl_elem_info *info;
	int i, ret = 0;

	memset(&val, 0, sizeof(val));

	ctl = snd_ctl_find_id_mixer(card, name);
	if (!ctl) {
		dev_warn(card->dev, "Unknown control name '%s'\n", name);
		return -ENOENT;
	}

	if (!ctl->put || !ctl->info) {
		dev_warn(card->dev, "Control '%s' not writable\n", name);
		return -ENOENT;
	}

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	ret = ctl->info(ctl, info);
	if (ret < 0) {
		dev_warn(card->dev, "Unable to get info for '%s'\n", name);
		goto exit_free_info;
	}

	if (info->type != SNDRV_CTL_ELEM_TYPE_ENUMERATED) {
		dev_warn(card->dev, "Control '%s' is not an enum\n", name);
		ret = -EINVAL;
		goto exit_free_info;
	}

	for (i = 0; i < info->value.enumerated.items; i++) {
		info->value.enumerated.item = i;
		ctl->info(ctl, info);

		if (strcmp(info->value.enumerated.name, value) != 0)
			continue;

		val.value.enumerated.item[0] = i;

		ret = ctl->put(ctl, &val);
		if (ret < 0) {
			dev_warn(card->dev, "Unable to write control '%s'\n",
				 name);
			goto exit_free_info;
		}

		dev_warn(card->dev, "Control default '%s' -> '%s'\n",
			 name, value);

		goto exit_free_info;
	}

	dev_warn(card->dev, "Enum '%s' has no entry '%s'\n", name, value);

exit_free_info:
	kfree(info);
	return ret;
}

static int am33xx_s800_common_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params,
					bool is_spdif)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_card *card = codec_dai->component->card;
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);
	unsigned int rate = params_rate(params);
	unsigned int bclk_div = is_spdif ? 4 : 2;
	int ret;

	const unsigned int base_44100_clks = 11025;
	const unsigned int base_48000_clks = 12000;

	if (priv->mclk_rate % base_48000_clks == 0)
		priv->mclk_rate_current =
			(rate % 16000 == 0) ?
			priv->mclk_rate :
			(priv->mclk_rate / 48000 * 44100);
	else if (priv->mclk_rate % base_44100_clks == 0)
		priv->mclk_rate_current =
			(rate % 16000 == 0) ?
			(priv->mclk_rate / 44100 * 48000) :
			priv->mclk_rate;
	else {
		dev_err(priv->card.dev, "mclk_rate in device tree is invalid\n");
		return -EINVAL;
	}

	ret = am33xx_s800_set_mclk(priv);
	if (ret < 0)
		return ret;

	/* propagate the clock rate */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, priv->mclk_rate_current,
				     SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* intentionally ignore errors - the codec driver may not care */
	snd_soc_dai_set_sysclk(codec_dai, 0, priv->mclk_rate_current,
			       SND_SOC_CLOCK_IN);

	/* MCLK divider */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, MCASP_CLKDIV_AUXCLK, 1);
	if (ret < 0)
		return ret;

	/* BCLK divider */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, MCASP_CLKDIV_BCLK,
		priv->mclk_rate_current / (rate * bclk_div * DATA_WORD_WIDTH));
	if (ret < 0)
		return ret;

	/* BCLK-to-LRCLK divider */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, MCASP_CLKDIV_BCLK_FS_RATIO,
				     2 * DATA_WORD_WIDTH);
	if (ret < 0)
		return ret;

	return 0;
}

static int am33xx_s800_i2s_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	return am33xx_s800_common_hw_params(substream, params, false);
}

static const struct snd_soc_ops am33xx_s800_i2s_dai_link_ops = {
	.hw_params	= am33xx_s800_i2s_hw_params,
};

static int am33xx_s800_spdif_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	return am33xx_s800_common_hw_params(substream, params, true);
}

static const struct snd_soc_ops am33xx_s800_spdif_dai_link_ops = {
	.hw_params	= am33xx_s800_spdif_hw_params,
};

static int am33xx_s800_drift_info(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->value.integer.min = -500;	/* +/- 500ppm */
	uinfo->value.integer.max = 500;
	uinfo->count = 1;

	return 0;
}

static int am33xx_s800_drift_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] = priv->drift;

	return 0;
}

static int am33xx_s800_drift_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);
	int ret;

	if (ucontrol->value.integer.value[0] == priv->drift)
		return 0;

	priv->drift = ucontrol->value.integer.value[0];

	if (priv->mclk_rate_current) {
		ret = am33xx_s800_set_mclk(priv);
		if (ret < 0)
			dev_warn(card->dev,
				 "Unable to set clock rate: %d\n", ret);
	}

	return 1;
}

static const struct snd_kcontrol_new am33xx_s800_controls[] = {
	{
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Drift compensator",
		.info	= am33xx_s800_drift_info,
		.get	= am33xx_s800_drift_get,
		.put	= am33xx_s800_drift_put,
	},
};

static int am33xx_s800_passive_mode_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] =
		!gpiod_get_value_cansleep(priv->passive_mode_gpiod);
	return 0;
}

static int am33xx_s800_passive_mode_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

	gpiod_set_value_cansleep(priv->passive_mode_gpiod,
				 !ucontrol->value.integer.value[0]);
	return 1;
}

static const struct snd_kcontrol_new am33xx_s800_passive_mode_control =
	SOC_SINGLE_BOOL_EXT("Passive mode", 0,
			    am33xx_s800_passive_mode_get,
			    am33xx_s800_passive_mode_put);

static int am33xx_s800_amp_overheat_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] =
		!gpiod_get_value_cansleep(priv->amp_overheat_gpiod);

	return 0;
}

static const struct snd_kcontrol_new am33xx_s800_amp_overheat_control =
	SOC_SINGLE_BOOL_EXT("Amplifier Overheat Sensor", 0,
			    am33xx_s800_amp_overheat_get,
			    NULL);

static irqreturn_t am33xx_s800_amp_overheat_irq(int irq, void *data)
{
	struct snd_soc_am33xx_s800 *priv = data;

	snd_ctl_notify(priv->card.snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
		       &priv->amp_overheat_kctl->id);

	return IRQ_HANDLED;
}

static irqreturn_t am33xx_s800_amp_overcurrent_irq(int irq, void *data)
{
	struct snd_soc_am33xx_s800 *priv = data;

	dev_warn(priv->card.dev,
		 "Amplifier signaled overcurrent/shutdown condition");

	return IRQ_HANDLED;
}

static const struct of_device_id snd_soc_am33xx_s800_match[] = {
	{ .compatible	= "sue,am33xx-generic-audio" },
	{ }
};
MODULE_DEVICE_TABLE(of, snd_soc_am33xx_s800_match);

static int snd_soc_am33xx_s800_parse_links(struct device *dev,
					   struct snd_soc_am33xx_s800 *priv,
					   struct device_node *links_node,
					   unsigned int dai_fmt)
{
	struct snd_soc_dai_link *link;
	struct device_node *child;

	priv->card.num_links = of_get_child_count(links_node);
	if (priv->card.num_links == 0)
		return -EINVAL;

	priv->card.dai_link = devm_kcalloc(dev, priv->card.num_links,
					   sizeof(*link), GFP_KERNEL);
	if (!priv->card.dai_link)
		return -ENOMEM;

	link = priv->card.dai_link;

	for_each_child_of_node(links_node, child) {
		struct snd_soc_dai_link_component *cpus, *codecs, *platforms;

		cpus = devm_kzalloc(dev, sizeof(*cpus), GFP_KERNEL);
		codecs = devm_kzalloc(dev, sizeof(*codecs), GFP_KERNEL);
		platforms = devm_kzalloc(dev, sizeof(*platforms), GFP_KERNEL);
		if (!cpus || !codecs || !platforms) {
			of_node_put(child);
			return -ENOMEM;
		}

		link->cpus = cpus;
		link->num_cpus = 1;
		link->codecs = codecs;
		link->num_codecs = 1;
		link->platforms = platforms;
		link->num_platforms = 1;

		platforms->of_node = of_parse_phandle(child, "sue,platform", 0);
		codecs->of_node = of_parse_phandle(child, "sue,codec", 0);

		of_property_read_string(child, "sue,name", &link->name);
		of_property_read_string(child, "sue,stream-name",
					&link->stream_name);
		of_property_read_string(child, "sue,cpu-dai-name",
					&cpus->dai_name);
		of_property_read_string(child, "sue,codec-dai-name",
					&codecs->dai_name);

		if (of_property_present(child, "sue,spdif"))
			link->ops = &am33xx_s800_spdif_dai_link_ops;
		else
			link->ops = &am33xx_s800_i2s_dai_link_ops;

		link->dai_fmt = dai_fmt;
		link++;
	}

	return 0;
}

static int snd_soc_am33xx_s800_probe(struct platform_device *pdev)
{
	int ret;
	unsigned int dai_fmt;
	struct device *dev = &pdev->dev;
	struct device_node *top_node, *node;
	struct snd_soc_am33xx_s800 *priv;
	struct pinctrl *pinctrl;

	top_node = dev->of_node;
	if (!top_node)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBC_CFC;

	if (of_property_present(top_node, "sue,invert-wclk"))
		dai_fmt |= SND_SOC_DAIFMT_NB_IF;
	else
		dai_fmt |= SND_SOC_DAIFMT_NB_NF;

	/* request pin mux */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl))
		dev_warn(dev, "pins are not configured from the driver\n");

	priv->mclk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->mclk)) {
		dev_err(dev, "failed to get MCLK\n");
		return -EPROBE_DEFER;
	}
	clk_prepare_enable(priv->mclk);

	priv->regulator = devm_regulator_get(dev, "vd");
	if (IS_ERR(priv->regulator)) {
		dev_err(dev, "failed to get regulator\n");
		return -EPROBE_DEFER;
	}

	/* this is a hack to temporarily disable the MCLK in test mode */
	if (of_property_present(top_node, "sue,disable-clk")) {
		clk_disable_unprepare(priv->mclk);
		return 0;
	}

	/* get desired master clock from dt, this is desired due to emc */
	if (of_property_read_u32(top_node, "mclk_rate", &priv->mclk_rate)) {
		dev_err(dev, "Invalid value for mclk_rate in device tree!\n");
		return 0;
	}

	dev_info(dev, "Setting mclk to %i Hz\n", priv->mclk_rate);

	/* machine controls */
	priv->card.controls = am33xx_s800_controls;
	priv->card.num_controls = ARRAY_SIZE(am33xx_s800_controls);

	priv->card.owner = THIS_MODULE;
	priv->card.dev = dev;
	snd_soc_of_parse_card_name(&priv->card, "sue,card-name");

	node = of_get_child_by_name(top_node, "links");
	if (node) {
		ret = snd_soc_am33xx_s800_parse_links(dev, priv, node, dai_fmt);
		of_node_put(node);
		if (ret < 0)
			return ret;
	}

	platform_set_drvdata(pdev, &priv->card);
	snd_soc_card_set_drvdata(&priv->card, priv);

	ret = regulator_enable(priv->regulator);
	if (ret < 0) {
		dev_err(dev, "error enabling regulator\n");
		return ret;
	}

	ret = snd_soc_register_card(&priv->card);
	if (ret < 0) {
		dev_err(dev, "error registering card (%d)\n", ret);
		regulator_disable(priv->regulator);
		return ret;
	}

	node = of_get_child_by_name(top_node, "control-defaults");
	if (node) {
		struct device_node *child;

		for_each_child_of_node(node, child) {
			const char *name = NULL, *value = NULL;

			of_property_read_string(child, "sue,control-name",
						&name);
			of_property_read_string(child, "sue,control-value",
						&value);

			if (name && value)
				snd_soc_am33xx_s800_set_control(priv->card.snd_card,
								name, value);
		}

		of_node_put(node);
	}

	priv->passive_mode_gpiod = devm_gpiod_get_optional(dev, "sue,passive-mode",
							   GPIOD_OUT_HIGH);
	if (!IS_ERR(priv->passive_mode_gpiod) && priv->passive_mode_gpiod) {
		struct snd_kcontrol *kc =
			snd_ctl_new1(&am33xx_s800_passive_mode_control, priv);
		ret = snd_ctl_add(priv->card.snd_card, kc);
		if (ret < 0)
			dev_warn(dev, "Failed to add passive mode control: %d\n",
				 ret);
	} else if (IS_ERR(priv->passive_mode_gpiod)) {
		priv->passive_mode_gpiod = NULL;
	}

	priv->amp_overheat_gpiod = devm_gpiod_get_optional(dev, "sue,amp-overheat",
							   GPIOD_IN);
	if (!IS_ERR(priv->amp_overheat_gpiod) && priv->amp_overheat_gpiod) {
		unsigned int irq_flags = IRQF_TRIGGER_RISING |
					 IRQF_TRIGGER_FALLING |
					 IRQF_ONESHOT;

		ret = devm_request_threaded_irq(
				dev, gpiod_to_irq(priv->amp_overheat_gpiod),
				NULL, am33xx_s800_amp_overheat_irq,
				irq_flags, "Amplifier Overheat", priv);
		if (ret < 0)
			dev_warn(dev,
				 "Unable to request amp overheat IRQ: %d\n",
				 ret);

		if (ret == 0) {
			priv->amp_overheat_kctl =
				snd_ctl_new1(&am33xx_s800_amp_overheat_control,
					     priv);

			ret = snd_ctl_add(priv->card.snd_card,
					  priv->amp_overheat_kctl);
			if (ret < 0)
				dev_warn(dev,
					 "Failed to add amp overheat control: %d\n",
					 ret);
		}

		if (ret < 0)
			priv->amp_overheat_gpiod = NULL;
	} else if (IS_ERR(priv->amp_overheat_gpiod)) {
		priv->amp_overheat_gpiod = NULL;
	}

	priv->amp_overcurrent_gpiod = devm_gpiod_get_optional(dev,
							      "sue,amp-overcurrent",
							      GPIOD_IN);
	if (!IS_ERR(priv->amp_overcurrent_gpiod) && priv->amp_overcurrent_gpiod) {
		unsigned int irq_flags = IRQF_TRIGGER_RISING |
					 IRQF_TRIGGER_FALLING |
					 IRQF_ONESHOT;

		ret = devm_request_threaded_irq(
				dev, gpiod_to_irq(priv->amp_overcurrent_gpiod),
				NULL, am33xx_s800_amp_overcurrent_irq,
				irq_flags, "Amplifier Overcurrent", priv);
		if (ret < 0)
			dev_warn(dev,
				 "Unable to request amp overcurrent IRQ: %d\n",
				 ret);
	} else if (IS_ERR(priv->amp_overcurrent_gpiod)) {
		priv->amp_overcurrent_gpiod = NULL;
	}

	priv->amp_reset_gpiod = devm_gpiod_get_optional(dev, "sue,amp-reset",
							GPIOD_OUT_HIGH);
	if (!IS_ERR(priv->amp_reset_gpiod) && priv->amp_reset_gpiod) {
		of_property_read_u32(top_node, "sue,amp-reset-delay-ms",
				     &priv->amp_reset_delay_ms);
	} else if (IS_ERR(priv->amp_reset_gpiod)) {
		priv->amp_reset_gpiod = NULL;
	}

	return 0;
}

static void snd_soc_am33xx_s800_shutdown_amp(struct device *dev,
					     struct snd_soc_am33xx_s800 *priv)
{
	pinctrl_pm_select_sleep_state(dev);

	if (priv->amp_reset_gpiod) {
		gpiod_set_value_cansleep(priv->amp_reset_gpiod, 0);
		msleep(priv->amp_reset_delay_ms);
	}
}

static void snd_soc_am33xx_s800_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

	if (priv->amp_overheat_gpiod)
		devm_free_irq(&pdev->dev,
			      gpiod_to_irq(priv->amp_overheat_gpiod), priv);

	if (priv->amp_overcurrent_gpiod)
		devm_free_irq(&pdev->dev,
			      gpiod_to_irq(priv->amp_overcurrent_gpiod), priv);

	snd_soc_am33xx_s800_shutdown_amp(&pdev->dev, priv);
	snd_soc_unregister_card(&priv->card);
	regulator_disable(priv->regulator);
	clk_disable_unprepare(priv->mclk);
}

static int snd_soc_am33xx_s800_suspend(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

	clk_disable_unprepare(priv->mclk);

	snd_soc_am33xx_s800_shutdown_amp(dev, priv);
	regulator_disable(priv->regulator);

	return snd_soc_suspend(dev);
}

static void snd_soc_am33xx_s800_shutdown(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

	snd_soc_am33xx_s800_shutdown_amp(&pdev->dev, priv);
}

static int snd_soc_am33xx_s800_resume(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);
	int ret;

	ret = clk_prepare_enable(priv->mclk);
	if (ret < 0) {
		dev_err(dev, "unable to enable clock: %d\n", ret);
		return ret;
	}

	ret = regulator_enable(priv->regulator);
	if (ret < 0) {
		dev_err(dev, "unable to enable regulator: %d\n", ret);
		return ret;
	}

	if (priv->amp_reset_gpiod)
		gpiod_set_value_cansleep(priv->amp_reset_gpiod, 1);

	pinctrl_pm_select_default_state(dev);

	return snd_soc_resume(dev);
}

static const struct dev_pm_ops snd_soc_am33xx_s800_pm_ops = {
	.suspend = snd_soc_am33xx_s800_suspend,
	.resume = snd_soc_am33xx_s800_resume,
	.freeze = snd_soc_suspend,
	.thaw = snd_soc_resume,
	.poweroff = snd_soc_poweroff,
	.restore = snd_soc_resume,
};

static struct platform_driver snd_soc_am33xx_s800_driver = {
	.driver = {
		.name		= "snd-soc-am33xx-s800",
		.of_match_table	= snd_soc_am33xx_s800_match,
		.pm		= &snd_soc_am33xx_s800_pm_ops,
	},
	.probe		= snd_soc_am33xx_s800_probe,
	.remove		= snd_soc_am33xx_s800_remove,
	.shutdown	= snd_soc_am33xx_s800_shutdown,
};

module_platform_driver(snd_soc_am33xx_s800_driver);

MODULE_AUTHOR("Daniel Mack <daniel@zonque.org>");
MODULE_DESCRIPTION("Stream Unlimited S800 / Raumfeld ASoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:snd-soc-am33xx-s800");
