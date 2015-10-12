/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <sound/soc.h>

struct mxs_spdif_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
};

static int mxs_spdif_audio_probe(struct platform_device *pdev)
{
	struct device_node *spdif_np, *np = pdev->dev.of_node;
	struct mxs_spdif_data *data;
	int ret = 0;

	spdif_np = of_parse_phandle(np, "spdif-controller", 0);
	if (!spdif_np) {
		dev_err(&pdev->dev, "failed to find spdif-controller\n");
		ret = -EINVAL;
		goto end;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto end;
	}

	data->dai.name = "S/PDIF PCM";
	data->dai.stream_name = "S/PDIF PCM";
	data->dai.codec_dai_name = "snd-soc-dummy-dai";
	data->dai.codec_name = "snd-soc-dummy";
	data->dai.cpu_of_node = spdif_np;
	data->dai.platform_of_node = spdif_np;
	data->dai.playback_only = true;

	data->card.dev = &pdev->dev;
	data->card.dai_link = &data->dai;
	data->card.num_links = 1;

	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto end;

	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed: %d\n", ret);
		goto end;
	}

	platform_set_drvdata(pdev, data);

end:
	of_node_put(spdif_np);

	return ret;
}

static const struct of_device_id mxs_spdif_dt_ids[] = {
	{ .compatible = "fsl,mxs-spdif-audio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxs_spdif_dt_ids);

static struct platform_driver mxs_spdif_driver = {
	.driver = {
		.name = "mxs-spdif",
		.of_match_table = mxs_spdif_dt_ids,
	},
	.probe = mxs_spdif_audio_probe,
};

module_platform_driver(mxs_spdif_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale MXS S/PDIF machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mxs-spdif");
