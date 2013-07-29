/*
 * mxs-builtin-audio.c -- i.MX233 built-in codec ALSA Soc Audio driver
 *
 * Author: Michal Ulianko <michal.ulianko@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/soc-dapm.h>
#include <asm/mach-types.h>

static struct snd_soc_dai_link mxs_adc_dai_link[] = {
	{
		.name		= "MXS ADC/DAC",
		.stream_name	= "MXS ADC/DAC",
		.codec_dai_name	= "mxs-builtin-codec-dai",
//		.codec_name	= "mxs-builtin-codec",
//		.cpu_dai_name	= "mxs-builtin-cpu-dai",
//		.platform_name	= "mxs-builtin-cpu-dai",
//		.ops		= &mxs_sgtl5000_hifi_ops,
	},
};

static struct snd_soc_card mxs_adc_audio = {
	.name		= "mxs-builtin-audio",
	.owner		= THIS_MODULE,
	.dai_link	= mxs_adc_dai_link,
	.num_links	= ARRAY_SIZE(mxs_adc_dai_link),
};

static int mxsadc_audio_probe_dt(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *cpu_dai_np, *codec_np;
	int ret = 0;

	if (!np)
		return 1; /* no device tree */

	cpu_dai_np = of_parse_phandle(np, "cpu-dai", 0);
	codec_np = of_parse_phandle(np, "audio-codec", 0);
	if (!cpu_dai_np || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		return -EINVAL;
	}

	mxs_adc_dai_link[0].codec_name = NULL;
	mxs_adc_dai_link[0].codec_of_node = codec_np;
	mxs_adc_dai_link[0].cpu_dai_name = NULL;
	mxs_adc_dai_link[0].cpu_of_node = cpu_dai_np;
	mxs_adc_dai_link[0].platform_name = NULL;
	mxs_adc_dai_link[0].platform_of_node = cpu_dai_np;

//	of_node_put(codec_np);
//	of_node_put(cpu_dai_np);

	return ret;
}

static int mxsadc_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mxs_adc_audio;
	int ret;

	ret = mxsadc_audio_probe_dt(pdev);
	if (ret < 0)
		return ret;

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int mxsadc_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id mxs_adc_audio_dt_ids[] = {
	{ .compatible = "fsl,mxs-builtin-audio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxs_adc_audio_dt_ids);

static struct platform_driver mxs_adc_audio_driver = {
	.driver = {
		.name = "mxs-builtin-audio",
		.owner = THIS_MODULE,
		.of_match_table = mxs_adc_audio_dt_ids,
	},
	.probe = mxsadc_audio_probe,
	.remove = mxsadc_audio_remove,
};

module_platform_driver(mxs_adc_audio_driver);

MODULE_DESCRIPTION("Freescale MXS ADC/DAC SoC Machine Driver");
MODULE_AUTHOR("Michal Ulianko <michal.ulianko@gmail.com>");
MODULE_LICENSE("GPL");
