/*
 * ALSA SoC MXS SPDIF codec driver
 *
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc.
 *
 * Based on stmp3xxx_spdif.h by:
 * Vladimir Barinov <vbarinov@embeddedalley.com>
 *
 * Copyright 2008 SigmaTel, Inc
 * Copyright 2008 Embedded Alley Solutions, Inc
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program  is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/dmaengine_pcm.h>
#include <sound/initval.h>

#include "mxs-spdif-dai.h"

// #define CONFIG_SND_SOC_MXS_SPDIF_CONTROLS

#define BF(value, field) (((value) << BP_##field) & BM_##field)

struct mxs_spdif_priv {
	struct platform_device *pdev;
	void __iomem *base;
	struct clk *clk;

	uint8_t ch_status[4];
	struct snd_soc_dai_driver cpu_dai_drv;
	struct snd_dmaengine_dai_dma_data dma_params;
};

/* This is currently not working as expected.
 * Do we really need to show all this guts to user? */
#ifdef CONFIG_SND_SOC_MXS_SPDIF_CONTROLS
static const int mxs_spdif_regmap[] = {
	HW_SPDIF_CTRL,
	HW_SPDIF_STAT,
	HW_SPDIF_FRAMECTRL,
	HW_SPDIF_SRR,
	HW_SPDIF_DEBUG,
	HW_SPDIF_DATA,
	HW_SPDIF_VERSION,
};

/*
 * ALSA core supports only 16 bit registers. It means we have to simulate it
 * by virtually splitting a 32bit SPDIF registers into two halves
 * high (bits 31:16) and low (bits 15:0). The routins abow detects which part
 * of 32bit register is accessed.
 */
static int mxs_spdif_write(struct snd_soc_component *component,
			   unsigned int reg, unsigned int value)
{
	struct mxs_spdif_priv *mxs_spdif = snd_soc_component_get_drvdata(component);
	unsigned int reg_val;
	unsigned int mask = 0xffff;

	if (reg >= SPDIF_REGNUM)
		return -EINVAL;

	if (reg & 0x1) {
		mask <<= 16;
		value <<= 16;
	}

	reg_val = __raw_readl(mxs_spdif->base + mxs_spdif_regmap[reg >> 1]);
	reg_val = (reg_val & ~mask) | value;
	__raw_writel(reg_val, mxs_spdif->base + mxs_spdif_regmap[reg >> 1]);

	return 0;
}

static int mxs_spdif_read(struct snd_soc_component *component,
				   unsigned int reg, unsigned int *value)
{
	struct mxs_spdif_priv *mxs_spdif = snd_soc_component_get_drvdata(component);
	unsigned int reg_val;

	if (reg >= SPDIF_REGNUM)
		return -EINVAL;

	reg_val = __raw_readl(mxs_spdif->base + mxs_spdif_regmap[reg >> 1]);
	if (reg & 1)
		reg_val >>= 16;

	*value = reg_val & 0xffff;

	return 0;
}

/* Codec controls */
static const struct snd_kcontrol_new mxs_spdif_controls[] = {
	SOC_SINGLE("PRO", SPDIF_FRAMECTRL_L, 0, 0x1, 0),
	SOC_SINGLE("AUDIO", SPDIF_FRAMECTRL_L, 1, 0x1, 0),
	SOC_SINGLE("COPY", SPDIF_FRAMECTRL_L, 2, 0x1, 0),
	SOC_SINGLE("PRE", SPDIF_FRAMECTRL_L, 3, 0x1, 0),
	SOC_SINGLE("CC", SPDIF_FRAMECTRL_L, 4, 0x7F, 0),
	SOC_SINGLE("L", SPDIF_FRAMECTRL_L, 12, 0x1, 0),
	SOC_SINGLE("V", SPDIF_FRAMECTRL_L, 13, 0x1, 0),
	SOC_SINGLE("USER DATA", SPDIF_FRAMECTRL_L, 14, 0x1, 0),
	SOC_SINGLE("AUTO MUTE", SPDIF_FRAMECTRL_H, 16, 0x1, 0),
	SOC_SINGLE("V CONFIG", SPDIF_FRAMECTRL_H, 17, 0x1, 0),
};
#endif	/* CONFIG_SND_SOC_MXS_SPDIF_CONTROLS */

struct spdif_srr {
	u32 rate;
	u32 basemult;
	u32 rate_factor;
};

static struct spdif_srr srr_values[] = {
	{96000, 0x2, 0x0BB80},
	{88200, 0x2, 0x0AC44},
	{64000, 0x2, 0x07D00},
	{48000, 0x1, 0x0BB80},
	{44100, 0x1, 0x0AC44},
	{32000, 0x1, 0x07D00},
};

static inline int get_srr_values(int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(srr_values); i++)
		if (srr_values[i].rate == rate)
			return i;

	return -1;
}

/**********************************************************************************
 * DAI functions
 **********************************************************************************/

static int mxs_spdif_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mxs_spdif_priv *mxs_spdif = snd_soc_dai_get_drvdata(rtd->cpu_dai);
	struct device *dev = &mxs_spdif->pdev->dev;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	int i;
	u32 srr_value = 0;
	u32 basemult;

	i = get_srr_values(params_rate(params));
	if (i < 0)
		dev_warn(dev, "unsupported rate %d\n", params_rate(params));
	else {
		basemult = srr_values[i].basemult;

		srr_value = BF(basemult, SPDIF_SRR_BASEMULT) |
		    BF(srr_values[i].rate_factor, SPDIF_SRR_RATE);

		if (playback)
			__raw_writel(srr_value, mxs_spdif->base + HW_SPDIF_SRR);
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		if (playback)
			__raw_writel(BM_SPDIF_CTRL_WORD_LENGTH,
				     mxs_spdif->base + HW_SPDIF_CTRL_SET);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
		if (playback)
			__raw_writel(BM_SPDIF_CTRL_WORD_LENGTH,
				     mxs_spdif->base + HW_SPDIF_CTRL_CLR);
		break;
	default:
		dev_warn(dev, "unsupported format %d\n", params_format(params));
		return -EINVAL;
	}

	return 0;
}

static int mxs_spdif_trigger(struct snd_pcm_substream *substream, int cmd,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mxs_spdif_priv *mxs_spdif = snd_soc_dai_get_drvdata(rtd->cpu_dai);
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (playback)
			__raw_writel(BM_SPDIF_CTRL_RUN,
				mxs_spdif->base + HW_SPDIF_CTRL_SET);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		if (playback)
			__raw_writel(BM_SPDIF_CTRL_RUN,
				mxs_spdif->base + HW_SPDIF_CTRL_CLR);
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int mxs_spdif_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mxs_spdif_priv *mxs_spdif = snd_soc_dai_get_drvdata(rtd->cpu_dai);
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;

	clk_enable(mxs_spdif->clk);

	/* Soft reset SPDIF block */
	__raw_writel(BM_SPDIF_CTRL_SFTRST, mxs_spdif->base + HW_SPDIF_CTRL_SET);

	while (!(__raw_readl(mxs_spdif->base + HW_SPDIF_CTRL)
		 & BM_SPDIF_CTRL_CLKGATE))
		;

	/* Move SPDIF codec out of reset */
	__raw_writel(BM_SPDIF_CTRL_SFTRST, mxs_spdif->base + HW_SPDIF_CTRL_CLR);

	/* Ungate SPDIF clocks */
	__raw_writel(BM_SPDIF_CTRL_CLKGATE,
		     mxs_spdif->base + HW_SPDIF_CTRL_CLR);

	/* 16 bit word length */
	__raw_writel(BM_SPDIF_CTRL_WORD_LENGTH,
		     mxs_spdif->base + HW_SPDIF_CTRL_SET);

	/* Enable error interrupt */
	if (playback) {
		__raw_writel(BM_SPDIF_CTRL_FIFO_OVERFLOW_IRQ,
				mxs_spdif->base + HW_SPDIF_CTRL_CLR);
		__raw_writel(BM_SPDIF_CTRL_FIFO_UNDERFLOW_IRQ,
				mxs_spdif->base + HW_SPDIF_CTRL_CLR);
		__raw_writel(BM_SPDIF_CTRL_FIFO_ERROR_IRQ_EN,
				mxs_spdif->base + HW_SPDIF_CTRL_SET);
	}

	return 0;
}

static void mxs_spdif_shutdown(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mxs_spdif_priv *mxs_spdif = snd_soc_dai_get_drvdata(rtd->cpu_dai);
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;

	/* Disable error interrupt */
	if (playback) {
		__raw_writel(BM_SPDIF_CTRL_FIFO_ERROR_IRQ_EN,
				mxs_spdif->base + HW_SPDIF_CTRL_CLR);
	}
	
	/* Gate SPDIF clocks */
	__raw_writel(BM_SPDIF_CTRL_CLKGATE,
		     mxs_spdif->base + HW_SPDIF_CTRL_SET);

	clk_disable(mxs_spdif->clk);
}


static int mxs_spdif_dai_probe(struct snd_soc_dai *dai)
{
	struct mxs_spdif_priv *mxs_spdif = snd_soc_dai_get_drvdata(dai);
	struct snd_soc_component *component = dai->component;

	snd_soc_dai_init_dma_data(dai, &mxs_spdif->dma_params, NULL);

#ifdef CONFIG_SND_SOC_MXS_SPDIF_CONTROLS
	component->read = mxs_spdif_read;
	component->write = mxs_spdif_write;
	snd_soc_add_component_controls(component, mxs_spdif_controls,
			ARRAY_SIZE(mxs_spdif_controls));
#endif

	return 0;
}

struct snd_soc_dai_ops mxs_spdif_dai_ops = {
	.hw_params = mxs_spdif_hw_params,
	.startup = mxs_spdif_startup,
	.shutdown = mxs_spdif_shutdown,
	.trigger = mxs_spdif_trigger,
};

struct snd_soc_dai_driver mxs_spdif_dai = {
	.probe = mxs_spdif_dai_probe,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = (SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_64000 | \
				SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000),
		.formats = (SNDRV_PCM_FMTBIT_S16_LE | \
				SNDRV_PCM_FMTBIT_S20_3LE | \
				SNDRV_PCM_FMTBIT_S24_LE | \
				SNDRV_PCM_FMTBIT_S32_LE),
	},
	.ops = &mxs_spdif_dai_ops,
};

static irqreturn_t mxs_spdif_isr(int irq, void *dev_id)
{
	struct mxs_spdif_priv *mxs_spdif = dev_id;
	struct device *dev = &mxs_spdif->pdev->dev;
	u32 ctrl_reg = 0;

	ctrl_reg = __raw_readl(mxs_spdif->base + HW_SPDIF_CTRL);

	if (ctrl_reg & BM_SPDIF_CTRL_FIFO_UNDERFLOW_IRQ) {
		dev_dbg(dev, "underflow detected\n");

		__raw_writel(BM_SPDIF_CTRL_FIFO_UNDERFLOW_IRQ,
				mxs_spdif->base + HW_SPDIF_CTRL_CLR);
	} else if (ctrl_reg & BM_SPDIF_CTRL_FIFO_OVERFLOW_IRQ) {
		dev_dbg(dev, "overflow detected\n");

		__raw_writel(BM_SPDIF_CTRL_FIFO_OVERFLOW_IRQ,
				mxs_spdif->base + HW_SPDIF_CTRL_CLR);
	} else
		dev_warn(dev, "Unknown SPDIF error interrupt\n");

	return IRQ_HANDLED;
}

static const struct snd_pcm_hardware mxs_spdif_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_RESUME,
	.formats = SNDRV_PCM_FMTBIT_S16_LE |
		SNDRV_PCM_FMTBIT_S20_3LE |
		SNDRV_PCM_FMTBIT_S24_LE,
	.channels_min = 2,
	.channels_max = 2,
	.period_bytes_min = 128,
	.period_bytes_max = 8192,
	.periods_min = 1,
	.periods_max = 52,
	.buffer_bytes_max = 64*1024,
	.fifo_size = 32,
};

static const struct snd_dmaengine_pcm_config mxs_spdif_dmaengine_pcm_config = {
	.pcm_hardware = &mxs_spdif_pcm_hardware,
	.prealloc_buffer_size = 64*1024,
};

static const struct snd_soc_component_driver mxs_spdif_component = {
	.name		= "mxs-spdif",
};

static int mxs_spdif_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mxs_spdif_priv *mxs_spdif;
	struct resource *res;
	int ret = 0, irq;

	dev_info(&pdev->dev, "MXS SPDIF Audio Transmitter\n");

	mxs_spdif = devm_kzalloc(&pdev->dev, sizeof(struct mxs_spdif_priv), GFP_KERNEL);
	if (mxs_spdif == NULL)
		return -ENOMEM;

	memcpy(&mxs_spdif->cpu_dai_drv, &mxs_spdif_dai, sizeof(mxs_spdif_dai));
	mxs_spdif->cpu_dai_drv.name = mxs_spdif_component.name;

	/* Get the addresses and IRQ */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mxs_spdif->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mxs_spdif->base))
		return PTR_ERR(mxs_spdif->base);

	/* TODO: use regmap framework
	mxs_spdif->regmap = devm_regmap_init_mmio_clk(&pdev->dev,
			"core", regs, &fsl_spdif_regmap_config);
	if (IS_ERR(mxs_spdif->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		return PTR_ERR(mxs_spdif->regmap);
	}
	*/

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq for node %s\n", np->full_name);
		return irq;
	}
	ret = devm_request_irq(&pdev->dev, irq, mxs_spdif_isr, 0,
			mxs_spdif_component.name, mxs_spdif);
	if (ret) {
		dev_err(&pdev->dev, "could not claim irq %u\n", irq);
		return ret;
	}

	mxs_spdif->clk = devm_clk_get(&pdev->dev, "spdif");
	if (IS_ERR(mxs_spdif->clk)) {
		dev_err(&pdev->dev, "clocks initialization failed\n");
		return PTR_ERR(mxs_spdif->clk);
	}
	clk_prepare(mxs_spdif->clk);

	mxs_spdif->dma_params.maxburst = 8;
	mxs_spdif->dma_params.addr = res->start + HW_SPDIF_DATA;

	platform_set_drvdata(pdev, mxs_spdif);

	ret = devm_snd_soc_register_component(&pdev->dev, &mxs_spdif_component,
			&mxs_spdif->cpu_dai_drv, 1);
	if (ret) {
		dev_err(&pdev->dev, "register DAI failed\n");
		return ret;
	}

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, &mxs_spdif_dmaengine_pcm_config, 0);
	if (ret) {
		dev_err(&pdev->dev, "register PCM failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id mxs_spdif_dt_ids[] = {
	{ .compatible = "fsl,mxs-spdif", },
	{ .compatible = "fsl,imx23-spdif", },
	{ .compatible = "fsl,imx28-spdif", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxs_spdif_dt_ids);

static struct platform_driver mxs_spdif_driver = {
	.probe = mxs_spdif_probe,

	.driver = {
		.name = "mxs-spdif-dai",
		.owner = THIS_MODULE,
		.of_match_table = mxs_spdif_dt_ids,
	},
};
module_platform_driver(mxs_spdif_driver);

MODULE_DESCRIPTION("MXS SPDIF transmitter");
MODULE_AUTHOR("Alexey Ignatov");
MODULE_LICENSE("GPL");
