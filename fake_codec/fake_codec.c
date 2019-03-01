/*
 * ALSA SoC fake codec driver
 *
 * Author:      Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>
 * Copyright:   (C) 2019 OscillatorIMP Digital
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <linux/of.h>

#define DRV_NAME "fake_codec"

#define STUB_RATES	SNDRV_PCM_RATE_CONTINUOUS
#define STUB_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
			SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE | \
			SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dapm_widget fake_codec_widgets[] = {
	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),
};

static const struct snd_soc_dapm_route fake_codec_routes[] = {
	{ "LOUT", NULL, "Left LR Playback Mixer" },
    { "ROUT", NULL, "Right LR Playback Mixer" },
};

static struct snd_soc_codec_driver soc_codec_fake_codec = {
	.component_driver = {
		.dapm_widgets		= fake_codec_widgets,
		.num_dapm_widgets	= ARRAY_SIZE(fake_codec_widgets),
		.dapm_routes		= fake_codec_routes,
		.num_dapm_routes	= ARRAY_SIZE(fake_codec_routes),
	},
};

static struct snd_soc_dai_driver fake_codec_stub_dai = {
	.name		= "fake_codec-hifi",
	.playback 	= {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 384,
		.rates		= STUB_RATES,
		.formats	= STUB_FORMATS,
	},
};

static int fake_codec_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_fake_codec,
			&fake_codec_stub_dai, 1);
}

static int fake_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static const struct of_device_id fake_codec_dt_ids[] = {
	{ .compatible = "ggm,fake_codec", },
	{ }
};
MODULE_DEVICE_TABLE(of, fake_codec_dt_ids);

static struct platform_driver fake_codec_driver = {
	.probe		= fake_codec_probe,
	.remove		= fake_codec_remove,
	.driver		= {
		.name	= DRV_NAME,
		.of_match_table = of_match_ptr(fake_codec_dt_ids),
	},
};

module_platform_driver(fake_codec_driver);

MODULE_AUTHOR("Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>");
MODULE_DESCRIPTION("fake codec driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
