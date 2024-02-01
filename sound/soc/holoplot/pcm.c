// SPDX-License-Identifier: GPL-2.0

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define REG_CONTROL		0x0
#define REG_CONTROL_RUN		BIT(0)
#define REG_CONTROL_RESET	BIT(1)

#define REG_SRC_DST_ADDR	0x08
#define REG_BUFFER_SIZE		0x10
#define REG_PERIOD_SIZE		0x18
#define REG_POSITION		0x20

#define REG_IRQ_CONTROL		0x28
#define REG_IRQ_CONTROL_ACK	BIT(0)

struct holoplot_pcm_priv {
	void __iomem			*reg;
	struct snd_pcm_substream	*substream;
	struct snd_soc_dai_driver	dai;
	struct snd_pcm_hardware		hw;
};

static u64 holoplot_pcm_read_reg(struct holoplot_pcm_priv *priv, u64 reg)
{
	return ioread64(priv->reg + reg);
}

static void holoplot_pcm_write_reg(struct holoplot_pcm_priv *priv,
				   u64 reg, u64 value)
{
	iowrite64(value, priv->reg + reg);
}

static int holoplot_pcm_open(struct snd_soc_component *component,
			     struct snd_pcm_substream *substream)
{
	struct holoplot_pcm_priv *priv = dev_get_drvdata(component->dev);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	runtime->hw = priv->hw;
	runtime->private_data = priv;

	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 32);
	if (ret)
		return ret;

	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 32);
	if (ret)
		return ret;

	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	return 0;
}

static int holoplot_pcm_construct(struct snd_soc_component *component,
				  struct snd_soc_pcm_runtime *rtd)
{
	struct holoplot_pcm_priv *priv = dev_get_drvdata(component->dev);
	struct snd_card *card = rtd->card->snd_card;
	struct device *dev = card->dev;
	struct snd_pcm *pcm = rtd->pcm;
	int ret;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret)
		return ret;

	return snd_pcm_set_fixed_buffer_all(pcm, SNDRV_DMA_TYPE_DEV_IRAM,
					    dev, priv->hw.buffer_bytes_max);
}

static int holoplot_pcm_hw_params(struct snd_soc_component *component,
				  struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	struct holoplot_pcm_priv *priv = dev_get_drvdata(component->dev);
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (WARN_ON(runtime->dma_addr == 0))
		return -EINVAL;

	priv->substream = substream;

	holoplot_pcm_write_reg(priv, REG_CONTROL, REG_CONTROL_RESET);
	holoplot_pcm_write_reg(priv, REG_SRC_DST_ADDR, runtime->dma_addr);
	holoplot_pcm_write_reg(priv, REG_BUFFER_SIZE, runtime->dma_bytes);
	holoplot_pcm_write_reg(priv, REG_PERIOD_SIZE,
			       params_period_bytes(params));

	return 0;
}

static int holoplot_pcm_trigger(struct snd_soc_component *component,
				struct snd_pcm_substream *substream,
				int cmd)
{
	struct holoplot_pcm_priv *priv = dev_get_drvdata(component->dev);
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		holoplot_pcm_write_reg(priv, REG_CONTROL, REG_CONTROL_RUN);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		holoplot_pcm_write_reg(priv, REG_CONTROL, 0);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static snd_pcm_uframes_t holoplot_pcm_pointer(
	struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct holoplot_pcm_priv *priv = dev_get_drvdata(component->dev);
	u64 pos = holoplot_pcm_read_reg(priv, REG_POSITION);

	return bytes_to_frames(substream->runtime, pos);
}

static irqreturn_t holoplot_pcm_handle_irq(int irq, void *p)
{
	struct holoplot_pcm_priv *priv = p;

	if (priv->substream)
		snd_pcm_period_elapsed(priv->substream);

	holoplot_pcm_write_reg(priv, REG_IRQ_CONTROL, REG_IRQ_CONTROL_ACK);

	return IRQ_HANDLED;
}

static const struct snd_soc_dai_ops holoplot_pcm_dai_ops = {};

static const struct snd_soc_component_driver holoplot_pcm_component = {
	.name			= "holoplot-pcm",
	.legacy_dai_naming	= 1,
	.open			= holoplot_pcm_open,
	.pcm_construct		= holoplot_pcm_construct,
	.hw_params		= holoplot_pcm_hw_params,
	.trigger		= holoplot_pcm_trigger,
	.pointer		= holoplot_pcm_pointer,
};

static int holoplot_pcm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct holoplot_pcm_priv *priv;
	u32 channels;
	int ret, irq;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(dev, "DMA mask failed: %d\n", ret);
		return ret;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->reg))
		return PTR_ERR(priv->reg);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_threaded_irq(dev, irq, NULL, holoplot_pcm_handle_irq,
					IRQF_SHARED | IRQF_ONESHOT,
					pdev->name, priv);
	if (ret < 0) {
		dev_err(dev, "Failed to request IRQ: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, priv);

	ret = of_property_read_u32(dev->of_node,
				   "holoplot,channels", &channels);
	if (ret < 0) {
		dev_err(dev, "missing channels property");
		return -EINVAL;
	}

	priv->hw.info			= SNDRV_PCM_INFO_MMAP |
					  SNDRV_PCM_INFO_MMAP_VALID |
					  SNDRV_PCM_INFO_PAUSE |
					  SNDRV_PCM_INFO_RESUME |
					  SNDRV_PCM_INFO_INTERLEAVED |
					  SNDRV_PCM_INFO_HALF_DUPLEX;
	priv->hw.period_bytes_min	= SZ_1K;
	priv->hw.period_bytes_max	= SZ_64K;
	priv->hw.periods_min		= 4;
	priv->hw.periods_max		= 64;
	priv->hw.buffer_bytes_max	= priv->hw.period_bytes_max
					* priv->hw.periods_max;
	priv->hw.fifo_size		= 32;

	priv->dai.ops = &holoplot_pcm_dai_ops;
	priv->dai.symmetric_rate = 1;
	priv->dai.name = "x2-tdm";

	if (of_property_read_bool(dev->of_node, "holoplot,playback")) {
		priv->dai.playback.stream_name = "Playback";
		priv->dai.playback.channels_min = channels;
		priv->dai.playback.channels_max = channels;
		priv->dai.playback.rates = SNDRV_PCM_RATE_48000;
		priv->dai.playback.formats = SNDRV_PCM_FMTBIT_S24_LE;
	}

	if (of_property_read_bool(dev->of_node, "holoplot,capture")) {
		priv->dai.capture.stream_name = "Capture";
		priv->dai.capture.channels_min = channels;
		priv->dai.capture.channels_max = channels;
		priv->dai.capture.rates = SNDRV_PCM_RATE_48000;
		priv->dai.capture.formats = SNDRV_PCM_FMTBIT_S16_LE;
	}

	if (!priv->dai.playback.stream_name &&
	    !priv->dai.capture.stream_name) {
		dev_err(dev, "Neither capture nor playback given in DTS\n");
		return -EINVAL;
	}

	ret = devm_snd_soc_register_component(dev, &holoplot_pcm_component,
					      &priv->dai, 1);
	if (ret < 0) {
		dev_err_probe(dev, ret, "failed to register component\n");
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
