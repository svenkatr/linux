/*
 * mxs-builtin-dai.c -- i.MX233 built-in codec ALSA Soc Audio driver
 *
 * Author: Michal Ulianko <michal.ulianko@gmail.com>
 *
 * Based on sound/soc/mxs/mxs-adc.c for kernel 2.6.35
 * by Vladislav Buzov <vbuzov@embeddedalley.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

#include "../codecs/mxs-builtin-codec.h"

#define ADC_VOLUME_MIN  0x37

/* TODO Use codec IO function soc snd write etc, instead of __writel __readl */

// TODO use container_of
struct mxs_irq_data {
	struct snd_pcm_substream *substream;
	struct mxs_adc_priv *mxs_adc;
};

struct mxs_adc_priv {
	struct mxs_irq_data irq_data;
	int dma_adc_err_irq;
	int dma_dac_err_irq;
	int hp_short_irq;
	void __iomem *audioin_base;
	void __iomem *audioout_base;
	void __iomem *rtc_base;
};

typedef struct {
	struct work_struct work;
	struct timer_list timer;

	/* target workqueue and CPU ->timer uses to queue ->work */
	struct workqueue_struct *wq;
	int cpu;

	struct mxs_adc_priv *mxs_adc;
} my_delayed_work_t;

// static struct delayed_work work;
// static struct delayed_work adc_ramp_work;
// static struct delayed_work dac_ramp_work;
// static struct delayed_work test;
static my_delayed_work_t work;
static my_delayed_work_t adc_ramp_work;
static my_delayed_work_t dac_ramp_work;
static my_delayed_work_t test;
static bool adc_ramp_done = 1;
static bool dac_ramp_done = 1;

static inline void mxs_adc_schedule_work(struct delayed_work *work)
{
	schedule_delayed_work(work, HZ / 10);
}

static void mxs_adc_work(struct work_struct *work)
{
	struct mxs_adc_priv *mxs_adc = ((my_delayed_work_t *)work)->mxs_adc;
	/* disable irq */
	disable_irq(mxs_adc->hp_short_irq);

	while (true) {
		__raw_writel(BM_AUDIOOUT_PWRDN_HEADPHONE,
		      mxs_adc->audioout_base + HW_AUDIOOUT_PWRDN_CLR);
		msleep(10);
		if ((__raw_readl(mxs_adc->audioout_base + HW_AUDIOOUT_ANACTRL)
			& BM_AUDIOOUT_ANACTRL_SHORT_LR_STS) != 0) {
			/* rearm the short protection */
			__raw_writel(BM_AUDIOOUT_ANACTRL_SHORTMODE_LR,
				mxs_adc->audioout_base + HW_AUDIOOUT_ANACTRL_CLR);
			__raw_writel(BM_AUDIOOUT_ANACTRL_SHORT_LR_STS,
				mxs_adc->audioout_base + HW_AUDIOOUT_ANACTRL_CLR);
			__raw_writel(BF_AUDIOOUT_ANACTRL_SHORTMODE_LR(0x1),
				mxs_adc->audioout_base + HW_AUDIOOUT_ANACTRL_SET);

			__raw_writel(BM_AUDIOOUT_PWRDN_HEADPHONE,
				mxs_adc->audioout_base + HW_AUDIOOUT_PWRDN_SET);
			printk(KERN_WARNING "WARNING : Headphone LR short!\r\n");
		} else {
			printk(KERN_WARNING "INFO : Headphone LR no longer short!\r\n");
			break;
		}
		msleep(1000);
	}

	/* power up the HEADPHONE and un-mute the HPVOL */
	__raw_writel(BM_AUDIOOUT_HPVOL_MUTE,
	      mxs_adc->audioout_base + HW_AUDIOOUT_HPVOL_CLR);
	__raw_writel(BM_AUDIOOUT_PWRDN_HEADPHONE,
		      mxs_adc->audioout_base + HW_AUDIOOUT_PWRDN_CLR);

	/* enable irq for next short detect*/
	enable_irq(mxs_adc->hp_short_irq);
}

static void mxs_adc_schedule_ramp_work(struct delayed_work *work)
{
	schedule_delayed_work(work, msecs_to_jiffies(2));
	adc_ramp_done = 0;
}

static void mxs_adc_ramp_work(struct work_struct *work)
{
	struct mxs_adc_priv *mxs_adc = ((my_delayed_work_t *)work)->mxs_adc;
	u32 reg = 0;
	u32 reg1 = 0;
	u32 reg2 = 0;
	u32 l, r;
	u32 ll, rr;
	int i;

	reg = __raw_readl(mxs_adc->audioin_base + \
		HW_AUDIOIN_ADCVOLUME);

	reg1 = reg & ~BM_AUDIOIN_ADCVOLUME_VOLUME_LEFT;
	reg1 = reg1 & ~BM_AUDIOIN_ADCVOLUME_VOLUME_RIGHT;
	/* minimize adc volume */
	reg2 = reg1 |
	    BF_AUDIOIN_ADCVOLUME_VOLUME_LEFT(ADC_VOLUME_MIN) |
	    BF_AUDIOIN_ADCVOLUME_VOLUME_RIGHT(ADC_VOLUME_MIN);
	__raw_writel(reg2,
		mxs_adc->audioin_base + HW_AUDIOIN_ADCVOLUME);
	msleep(1);

	l = (reg & BM_AUDIOIN_ADCVOLUME_VOLUME_LEFT) >>
		BP_AUDIOIN_ADCVOLUME_VOLUME_LEFT;
	r = (reg & BM_AUDIOIN_ADCVOLUME_VOLUME_RIGHT) >>
		BP_AUDIOIN_ADCVOLUME_VOLUME_RIGHT;

	/* fade in adc vol */
	for (i = ADC_VOLUME_MIN; (i < l) || (i < r);) {
		i += 0x8;
		ll = i < l ? i : l;
		rr = i < r ? i : r;
		reg2 = reg1 |
		    BF_AUDIOIN_ADCVOLUME_VOLUME_LEFT(ll) |
		    BF_AUDIOIN_ADCVOLUME_VOLUME_RIGHT(rr);
		__raw_writel(reg2,
		    mxs_adc->audioin_base + HW_AUDIOIN_ADCVOLUME);
		msleep(1);
	}
	adc_ramp_done = 1;
}

static void mxs_dac_schedule_ramp_work(struct delayed_work *work)
{
	schedule_delayed_work(work, msecs_to_jiffies(2));
	dac_ramp_done = 0;
}

static void mxs_dac_ramp_work(struct work_struct *work)
{
	struct mxs_adc_priv *mxs_adc = ((my_delayed_work_t *)work)->mxs_adc;
	u32 reg = 0;
	u32 reg1 = 0;
	u32 l, r;
	u32 ll, rr;
	int i;

	/* unmute hp and speaker */
	__raw_writel(BM_AUDIOOUT_HPVOL_MUTE,
		mxs_adc->audioout_base + HW_AUDIOOUT_HPVOL_CLR);
	__raw_writel(BM_AUDIOOUT_SPEAKERCTRL_MUTE,
		mxs_adc->audioout_base + HW_AUDIOOUT_SPEAKERCTRL_CLR);

	reg = __raw_readl(mxs_adc->audioout_base + \
			HW_AUDIOOUT_HPVOL);

	reg1 = reg & ~BM_AUDIOOUT_HPVOL_VOL_LEFT;
	reg1 = reg1 & ~BM_AUDIOOUT_HPVOL_VOL_RIGHT;

	l = (reg & BM_AUDIOOUT_HPVOL_VOL_LEFT) >>
		BP_AUDIOOUT_HPVOL_VOL_LEFT;
	r = (reg & BM_AUDIOOUT_HPVOL_VOL_RIGHT) >>
		BP_AUDIOOUT_HPVOL_VOL_RIGHT;
	/* fade in hp vol */
	for (i = 0x7f; i > 0 ;) {
		i -= 0x8;
		ll = i > (int)l ? i : l;
		rr = i > (int)r ? i : r;
		reg = reg1 | BF_AUDIOOUT_HPVOL_VOL_LEFT(ll)
			| BF_AUDIOOUT_HPVOL_VOL_RIGHT(rr);
		__raw_writel(reg,
			mxs_adc->audioout_base + HW_AUDIOOUT_HPVOL);
		msleep(1);
	}
	dac_ramp_done = 1;
}

/* IRQs */
static irqreturn_t mxs_short_irq(int irq, void *dev_id)
{
	struct mxs_adc_priv *mxs_adc = dev_id;
	//struct snd_pcm_substream *substream = mxs_adc->irq_data.substream;

	__raw_writel(BM_AUDIOOUT_ANACTRL_SHORTMODE_LR,
		mxs_adc->audioout_base + HW_AUDIOOUT_ANACTRL_CLR);
	__raw_writel(BM_AUDIOOUT_ANACTRL_SHORT_LR_STS,
		mxs_adc->audioout_base + HW_AUDIOOUT_ANACTRL_CLR);
	__raw_writel(BF_AUDIOOUT_ANACTRL_SHORTMODE_LR(0x1),
		mxs_adc->audioout_base + HW_AUDIOOUT_ANACTRL_SET);

	__raw_writel(BM_AUDIOOUT_HPVOL_MUTE,
	      mxs_adc->audioout_base + HW_AUDIOOUT_HPVOL_SET);
	__raw_writel(BM_AUDIOOUT_PWRDN_HEADPHONE,
		      mxs_adc->audioout_base + HW_AUDIOOUT_PWRDN_SET);
	__raw_writel(BM_AUDIOOUT_ANACTRL_HP_CLASSAB,
		mxs_adc->audioout_base + HW_AUDIOOUT_ANACTRL_SET);

	mxs_adc_schedule_work((struct delayed_work *) &work);
	return IRQ_HANDLED;
}

static irqreturn_t mxs_err_irq(int irq, void *dev_id)
{
	struct mxs_adc_priv *mxs_adc = dev_id;
	struct snd_pcm_substream *substream = mxs_adc->irq_data.substream;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	u32 ctrl_reg;
	u32 overflow_mask;
	u32 underflow_mask;

	if (playback) {
		ctrl_reg = __raw_readl(mxs_adc->audioout_base + HW_AUDIOOUT_CTRL);
		underflow_mask = BM_AUDIOOUT_CTRL_FIFO_UNDERFLOW_IRQ;
		overflow_mask = BM_AUDIOOUT_CTRL_FIFO_OVERFLOW_IRQ;
	} else {
		ctrl_reg = __raw_readl(mxs_adc->audioin_base + HW_AUDIOIN_CTRL);
		underflow_mask = BM_AUDIOIN_CTRL_FIFO_UNDERFLOW_IRQ;
		overflow_mask = BM_AUDIOIN_CTRL_FIFO_OVERFLOW_IRQ;
	}

	if (ctrl_reg & underflow_mask) {
		printk(KERN_DEBUG "%s underflow detected\n",
		       playback ? "DAC" : "ADC");

		if (playback)
			__raw_writel(
				BM_AUDIOOUT_CTRL_FIFO_UNDERFLOW_IRQ,
				mxs_adc->audioout_base + HW_AUDIOOUT_CTRL_CLR);
		else
			__raw_writel(
				BM_AUDIOIN_CTRL_FIFO_UNDERFLOW_IRQ,
				mxs_adc->audioin_base + HW_AUDIOIN_CTRL_CLR);

	} else if (ctrl_reg & overflow_mask) {
		printk(KERN_DEBUG "%s overflow detected\n",
		       playback ? "DAC" : "ADC");

		if (playback)
			__raw_writel(
				BM_AUDIOOUT_CTRL_FIFO_OVERFLOW_IRQ,
				mxs_adc->audioout_base + HW_AUDIOOUT_CTRL_CLR);
		else
			__raw_writel(BM_AUDIOIN_CTRL_FIFO_OVERFLOW_IRQ,
				mxs_adc->audioin_base + HW_AUDIOIN_CTRL_CLR);
	} else
		printk(KERN_WARNING "Unknown DAC error interrupt\n");

	return IRQ_HANDLED;
}
/* END IRQs */

static int mxs_trigger(struct snd_pcm_substream *substream,
				int cmd,
				struct snd_soc_dai *cpu_dai)
{
	struct mxs_adc_priv *mxs_adc = snd_soc_dai_get_drvdata(cpu_dai);
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:

		if (playback) {
			/* enable the fifo error interrupt */
			__raw_writel(BM_AUDIOOUT_CTRL_FIFO_ERROR_IRQ_EN,
			mxs_adc->audioout_base + HW_AUDIOOUT_CTRL_SET);
			/* write a data to data reg to trigger the transfer */
			__raw_writel(0x0,
				mxs_adc->audioout_base + HW_AUDIOOUT_DATA);
			mxs_dac_schedule_ramp_work((struct delayed_work *) &dac_ramp_work);
		} else {
// 		    mxs_dma_get_info(prtd->dma_ch, &dma_info);
// 		    cur_bar1 = dma_info.buf_addr;
// 		    xfer_count1 = dma_info.xfer_count;

		    __raw_writel(BM_AUDIOIN_CTRL_RUN,
			mxs_adc->audioin_base + HW_AUDIOIN_CTRL_SET);
		    udelay(100);

// 		    mxs_dma_get_info(prtd->dma_ch, &dma_info);
// 		    cur_bar2 = dma_info.buf_addr;
// 		    xfer_count2 = dma_info.xfer_count;
//
// 		    /* check if DMA getting stuck */
// 		    if ((xfer_count1 == xfer_count2) && (cur_bar1 == cur_bar2))
// 			/* read a data from data reg to trigger the receive */
// 			reg = __raw_readl(mxs_adc->audioin_base + HW_AUDIOIN_DATA);

		    mxs_adc_schedule_ramp_work((struct delayed_work *) &adc_ramp_work);
		}
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:

		if (playback) {
// 			printk(KERN_INFO "SNDRV_PCM_TRIGGER_START\n");
// 			printk(KERN_INFO "ctrl:%x\n", __raw_readl(mxs_adc->audioout_base + HW_AUDIOOUT_CTRL));
// 			printk(KERN_INFO "stat:%x\n", __raw_readl(mxs_adc->audioout_base + HW_AUDIOOUT_STAT));
// 			printk(KERN_INFO "srr:%x\n", __raw_readl(mxs_adc->audioout_base + HW_AUDIOOUT_DACSRR));
// 			printk(KERN_INFO "vol:%x\n", __raw_readl(mxs_adc->audioout_base + HW_AUDIOOUT_DACVOLUME));
// 			printk(KERN_INFO "debug:%x\n", __raw_readl(mxs_adc->audioout_base + HW_AUDIOOUT_DACDEBUG));
// 			printk(KERN_INFO "hpvol:%x\n", __raw_readl(mxs_adc->audioout_base + HW_AUDIOOUT_HPVOL));
// 			printk(KERN_INFO "pwrdn:%x\n", __raw_readl(mxs_adc->audioout_base + HW_AUDIOOUT_PWRDN));
// 			printk(KERN_INFO "refc:%x\n", __raw_readl(mxs_adc->audioout_base + HW_AUDIOOUT_REFCTRL));
// 			printk(KERN_INFO "anac:%x\n", __raw_readl(mxs_adc->audioout_base + HW_AUDIOOUT_ANACTRL));
// 			printk(KERN_INFO "test:%x\n", __raw_readl(mxs_adc->audioout_base + HW_AUDIOOUT_TEST));
// 			printk(KERN_INFO "bist:%x\n", __raw_readl(mxs_adc->audioout_base + HW_AUDIOOUT_BISTCTRL));
// 			printk(KERN_INFO "anaclk:%x\n", __raw_readl(mxs_adc->audioout_base + HW_AUDIOOUT_ANACLKCTRL));

			if (dac_ramp_done == 0) {
				cancel_delayed_work((struct delayed_work *) &dac_ramp_work);
				dac_ramp_done = 1;
			}
			__raw_writel(BM_AUDIOOUT_HPVOL_MUTE,
			  mxs_adc->audioout_base + HW_AUDIOOUT_HPVOL_SET);
			__raw_writel(BM_AUDIOOUT_SPEAKERCTRL_MUTE,
			  mxs_adc->audioout_base + HW_AUDIOOUT_SPEAKERCTRL_SET);
			/* disable the fifo error interrupt */
			__raw_writel(BM_AUDIOOUT_CTRL_FIFO_ERROR_IRQ_EN,
				mxs_adc->audioout_base + HW_AUDIOOUT_CTRL_CLR);
			mdelay(50);
		} else {
			if (adc_ramp_done == 0) {
				cancel_delayed_work((struct delayed_work *) &adc_ramp_work);
				adc_ramp_done = 1;
			}
			__raw_writel(BM_AUDIOIN_CTRL_RUN,
				mxs_adc->audioin_base + HW_AUDIOIN_CTRL_CLR);
		}
		break;

	default:
		printk(KERN_ERR "TRIGGER ERROR\n");
		ret = -EINVAL;
	}

	return ret;
}

static int mxs_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *cpu_dai)
{
	struct mxs_adc_priv *mxs_adc = snd_soc_dai_get_drvdata(cpu_dai);
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	mxs_adc->irq_data.mxs_adc = mxs_adc;
	mxs_adc->irq_data.substream = substream;

	work.mxs_adc = mxs_adc;
	adc_ramp_work.mxs_adc = mxs_adc;
	dac_ramp_work.mxs_adc = mxs_adc;
	test.mxs_adc = mxs_adc;
	INIT_DELAYED_WORK(&work, mxs_adc_work);
	INIT_DELAYED_WORK(&adc_ramp_work, mxs_adc_ramp_work);
	INIT_DELAYED_WORK(&dac_ramp_work, mxs_dac_ramp_work);

	/* Enable error interrupt */
	if (playback) {
		__raw_writel(BM_AUDIOOUT_CTRL_FIFO_OVERFLOW_IRQ,
			mxs_adc->audioout_base + HW_AUDIOOUT_CTRL_CLR);
		__raw_writel(BM_AUDIOOUT_CTRL_FIFO_UNDERFLOW_IRQ,
			mxs_adc->audioout_base + HW_AUDIOOUT_CTRL_CLR);
	} else {
		__raw_writel(BM_AUDIOIN_CTRL_FIFO_OVERFLOW_IRQ,
			mxs_adc->audioin_base + HW_AUDIOIN_CTRL_CLR);
		__raw_writel(BM_AUDIOIN_CTRL_FIFO_UNDERFLOW_IRQ,
			mxs_adc->audioin_base + HW_AUDIOIN_CTRL_CLR);
		__raw_writel(BM_AUDIOIN_CTRL_FIFO_ERROR_IRQ_EN,
			mxs_adc->audioin_base + HW_AUDIOIN_CTRL_SET);
	}

	return 0;
}

static void mxs_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *cpu_dai)
{
	struct mxs_adc_priv *mxs_adc = snd_soc_dai_get_drvdata(cpu_dai);
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;

	/* Disable error interrupt */
	if (playback) {
		__raw_writel(BM_AUDIOOUT_CTRL_FIFO_ERROR_IRQ_EN,
			mxs_adc->audioout_base + HW_AUDIOOUT_CTRL_CLR);
	} else {
		__raw_writel(BM_AUDIOIN_CTRL_FIFO_ERROR_IRQ_EN,
			mxs_adc->audioin_base + HW_AUDIOIN_CTRL_CLR);
	}
}

#define MXS_ADC_RATES	SNDRV_PCM_RATE_8000_192000
#define MXS_ADC_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops mxs_adc_dai_ops = {
	.startup = mxs_startup,
	.trigger = mxs_trigger,
	.shutdown = mxs_shutdown,
};

static int mxs_dai_probe(struct snd_soc_dai *dai)
{
	// TODO This does not make any sense.
	struct mxs_adc_priv *mxs_adc = dev_get_drvdata(dai->dev);

	snd_soc_dai_set_drvdata(dai, mxs_adc);

	return 0;
}

static struct snd_soc_dai_driver mxs_adc_dai = {
	.name = "mxs-builtin-cpu-dai",
	.probe = mxs_dai_probe,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = MXS_ADC_RATES,
		.formats = MXS_ADC_FORMATS,
	},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = MXS_ADC_RATES,
		.formats = MXS_ADC_FORMATS,
	},
	.ops = &mxs_adc_dai_ops,
};

static const struct snd_soc_component_driver mxs_adc_component = {
	.name		= "mxs-xxx",	//TODO change this name
};

static const struct snd_pcm_hardware mxs_adc_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME |
				  SNDRV_PCM_INFO_INTERLEAVED,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S20_3LE |
				  SNDRV_PCM_FMTBIT_S24_LE,
	.channels_min		= 2,
	.channels_max		= 2,
	.period_bytes_min	= 32,
	.period_bytes_max	= 8192,
	.periods_min		= 1,
	.periods_max		= 52,
	.buffer_bytes_max	= 64 * 1024,
	.fifo_size		= 32,
};

static const struct snd_dmaengine_pcm_config mxs_adc_dmaengine_pcm_config = {
	.pcm_hardware = &mxs_adc_hardware,
	.prealloc_buffer_size = 64 * 1024,
};

static int mxs_adc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mxs_adc_priv *mxs_adc;
	int ret = 0;

	if (!np)
		return -EINVAL;

	mxs_adc = devm_kzalloc(&pdev->dev, sizeof(*mxs_adc), GFP_KERNEL);
	if (!mxs_adc)
		return -ENOMEM;

	mxs_adc->audioout_base = devm_ioremap(&pdev->dev, 0x80048000, 0x2000);
	if (IS_ERR(mxs_adc->audioout_base))
		return PTR_ERR(mxs_adc->audioout_base);

	mxs_adc->audioin_base = devm_ioremap(&pdev->dev, 0x8004c000, 0x2000);
	if (IS_ERR(mxs_adc->audioin_base))
		return PTR_ERR(mxs_adc->audioin_base);

	mxs_adc->rtc_base = devm_ioremap(&pdev->dev, 0x8005c000, 0x2000);
	if (IS_ERR(mxs_adc->rtc_base))
		return PTR_ERR(mxs_adc->rtc_base);

	/* Get IRQ numbers */
	mxs_adc->dma_adc_err_irq = platform_get_irq(pdev, 0);
	if (mxs_adc->dma_adc_err_irq < 0) {
		ret = mxs_adc->dma_adc_err_irq;
		dev_err(&pdev->dev, "failed to get ADC DMA ERR irq resource: %d\n", ret);
		return ret;
	}

	mxs_adc->dma_dac_err_irq = platform_get_irq(pdev, 1);
	if (mxs_adc->dma_dac_err_irq < 0) {
		ret = mxs_adc->dma_dac_err_irq;
		dev_err(&pdev->dev, "failed to get DAC DMA ERR irq resource: %d\n", ret);
		return ret;
	}

	mxs_adc->hp_short_irq = platform_get_irq(pdev, 2);
	if (mxs_adc->hp_short_irq < 0) {
		ret = mxs_adc->hp_short_irq;
		dev_err(&pdev->dev, "failed to get HP_SHORT irq resource: %d\n", ret);
		return ret;
	}

	/* Request IRQs */
	ret = devm_request_irq(&pdev->dev, mxs_adc->dma_adc_err_irq, mxs_err_irq, 0, "MXS DAC and ADC Error",
			  mxs_adc);
	if (ret) {
		printk(KERN_ERR "%s: Unable to request ADC/DAC error irq %d\n",
		       __func__, mxs_adc->dma_adc_err_irq);
		return ret;
	}

	ret = devm_request_irq(&pdev->dev, mxs_adc->dma_dac_err_irq, mxs_err_irq, 0, "MXS DAC and ADC Error",
			  mxs_adc);
	if (ret) {
		printk(KERN_ERR "%s: Unable to request ADC/DAC error irq %d\n",
		       __func__, mxs_adc->dma_dac_err_irq);
		return ret;
	}

	ret = devm_request_irq(&pdev->dev, mxs_adc->hp_short_irq, mxs_short_irq,
		IRQF_SHARED, "MXS DAC and ADC HP SHORT", mxs_adc);
	if (ret) {
		printk(KERN_ERR "%s: Unable to request ADC/DAC HP SHORT irq %d\n",
		       __func__, mxs_adc->hp_short_irq);
		return ret;
	}

	platform_set_drvdata(pdev, mxs_adc);

	ret = snd_soc_register_component(&pdev->dev, &mxs_adc_component, &mxs_adc_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "register DAI failed\n");
		return ret;
	}

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, &mxs_adc_dmaengine_pcm_config, 0);
	if (ret) {
		dev_err(&pdev->dev, "register PCM failed: %d\n", ret);
		goto failed_pdev_alloc;
	}

	return 0;

failed_pdev_alloc:
	snd_soc_unregister_component(&pdev->dev);

	return ret;
}

static int mxs_adc_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);

	return 0;
}

static const struct of_device_id mxs_adc_dai_dt_ids[] = {
	{ .compatible = "fsl,mxs-builtin-cpu-dai", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxs_adc_dai_dt_ids);

static struct platform_driver mxs_adc_dai_driver = {
	.probe = mxs_adc_probe,
	.remove = mxs_adc_remove,

	.driver = {
		.name = "mxs-builtin-cpu-dai",
		.owner = THIS_MODULE,
		.of_match_table = mxs_adc_dai_dt_ids,
	},
};

module_platform_driver(mxs_adc_dai_driver);

MODULE_DESCRIPTION("Freescale MXS ADC/DAC SoC Codec DAI Driver");
MODULE_AUTHOR("Michal Ulianko <michal.ulianko@gmail.com>");
MODULE_LICENSE("GPL");
