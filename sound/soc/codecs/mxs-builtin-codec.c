/*
 * mxs-builtin-codec.c -- i.MX233 built-in codec ALSA Soc Audio driver
 *
 * Author: Michal Ulianko <michal.ulianko@gmail.com>
 *
 * Based on sound/soc/codecs/mxs-adc-codec.c for kernel 2.6.35
 * by Vladislav Buzov <vbuzov@embeddedalley.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "mxs-builtin-codec.h"

#ifndef BF
#define BF(value, field) (((value) << BP_##field) & BM_##field)
#endif

/* TODO Delete this and use BM_RTC_PERSISTENT0_RELEASE_GND from header file
 * if it works. */
#define BP_RTC_PERSISTENT0_SPARE_ANALOG	18
#define BM_RTC_PERSISTENT0_SPARE_ANALOG	0xFFFC0000
#define BM_RTC_PERSISTENT0_RELEASE_GND BF(0x2, RTC_PERSISTENT0_SPARE_ANALOG)

/* TODO Use codec IO function soc snd write etc, instead of __writel __readl */

struct mxs_adc_priv {
	void __iomem *ain_base;
	void __iomem *aout_base;
	void __iomem *rtc_base;
	struct clk *clk;
	bool disable_mic_bias;
};

static unsigned int mxs_regmap[] = {
	HW_AUDIOOUT_CTRL,
	HW_AUDIOOUT_STAT,
	HW_AUDIOOUT_DACSRR,
	HW_AUDIOOUT_DACVOLUME,
	HW_AUDIOOUT_DACDEBUG,
	HW_AUDIOOUT_HPVOL,
	HW_AUDIOOUT_PWRDN,
	HW_AUDIOOUT_REFCTRL,
	HW_AUDIOOUT_ANACTRL,
	HW_AUDIOOUT_TEST,
	HW_AUDIOOUT_BISTCTRL,
	HW_AUDIOOUT_BISTSTAT0,
	HW_AUDIOOUT_BISTSTAT1,
	HW_AUDIOOUT_ANACLKCTRL,
	HW_AUDIOOUT_DATA,
	HW_AUDIOOUT_SPEAKERCTRL,
	HW_AUDIOOUT_VERSION,
	HW_AUDIOIN_CTRL,
	HW_AUDIOIN_STAT,
	HW_AUDIOIN_ADCSRR,
	HW_AUDIOIN_ADCVOLUME,
	HW_AUDIOIN_ADCDEBUG,
	HW_AUDIOIN_ADCVOL,
	HW_AUDIOIN_MICLINE,
	HW_AUDIOIN_ANACLKCTRL,
	HW_AUDIOIN_DATA,
};

static void __iomem *mxs_getreg(struct mxs_adc_priv *mxs_adc, int i)
{
	if (i <= 16)
		return mxs_adc->aout_base + mxs_regmap[i];
	else if (i < ADC_REGNUM)
		return mxs_adc->ain_base + mxs_regmap[i];
	else
		return NULL;
}

static u8 dac_volumn_control_word[] = {
	0x37, 0x5e, 0x7e, 0x8e,
	0x9e, 0xae, 0xb6, 0xbe,
	0xc6, 0xce, 0xd6, 0xde,
	0xe6, 0xee, 0xf6, 0xfe,
};

struct dac_srr {
	u32 rate;
	u32 basemult;
	u32 src_hold;
	u32 src_int;
	u32 src_frac;
};

static struct dac_srr srr_values[] = {
	{192000, 0x4, 0x0, 0x0F, 0x13FF},
	{176400, 0x4, 0x0, 0x11, 0x0037},
	{128000, 0x4, 0x0, 0x17, 0x0E00},
	{96000, 0x2, 0x0, 0x0F, 0x13FF},
	{88200, 0x2, 0x0, 0x11, 0x0037},
	{64000, 0x2, 0x0, 0x17, 0x0E00},
	{48000, 0x1, 0x0, 0x0F, 0x13FF},
	{44100, 0x1, 0x0, 0x11, 0x0037},
	{32000, 0x1, 0x0, 0x17, 0x0E00},
	{24000, 0x1, 0x1, 0x0F, 0x13FF},
	{22050, 0x1, 0x1, 0x11, 0x0037},
	{16000, 0x1, 0x1, 0x17, 0x0E00},
	{12000, 0x1, 0x3, 0x0F, 0x13FF},
	{11025, 0x1, 0x3, 0x11, 0x0037},
	{8000, 0x1, 0x3, 0x17, 0x0E00}
};

static inline int get_srr_values(int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(srr_values); i++)
		if (srr_values[i].rate == rate)
			return i;

	return -1;
}

/* SoC IO functions */
static void mxs_codec_write_cache(struct snd_soc_codec *codec, unsigned int reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg < ADC_REGNUM)
		cache[reg] = value;
}

static int mxs_codec_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int value)
{
	struct mxs_adc_priv *mxs_adc = snd_soc_codec_get_drvdata(codec);
	unsigned int reg_val;
	unsigned int mask = 0xffff;

	if (reg >= ADC_REGNUM)
		return -EIO;

	mxs_codec_write_cache(codec, reg, value);

	if (reg & 0x1) {
		mask <<= 16;
		value <<= 16;
	}

	reg_val = __raw_readl(mxs_getreg(mxs_adc, reg >> 1));
	reg_val = (reg_val & ~mask) | value;
	__raw_writel(reg_val, mxs_getreg(mxs_adc, reg >> 1));

	return 0;
}

static unsigned int mxs_codec_read(struct snd_soc_codec *codec, unsigned int reg)
{
	struct mxs_adc_priv *mxs_adc = snd_soc_codec_get_drvdata(codec);
	unsigned int reg_val;

	if (reg >= ADC_REGNUM)
		return -1;

	reg_val = __raw_readl(mxs_getreg(mxs_adc, reg >> 1));
	if (reg & 1)
		reg_val >>= 16;

	return reg_val & 0xffff;
}

// static unsigned int mxs_codec_read_cache(struct snd_soc_codec *codec, unsigned int reg)
// {
// 	u16 *cache = codec->reg_cache;
// 	if (reg >= ADC_REGNUM)
// 		return -EINVAL;
// 	return cache[reg];
// }

static void mxs_codec_sync_reg_cache(struct snd_soc_codec *codec)
{
	int reg;
	for (reg = 0; reg < ADC_REGNUM; reg += 1)
		mxs_codec_write_cache(codec, reg,
					   mxs_codec_read(codec, reg));
}

// static int mxs_codec_restore_reg(struct snd_soc_codec *codec, unsigned int reg)
// {
// 	unsigned int cached_val, hw_val;
//
// 	cached_val = mxs_codec_read_cache(codec, reg);
// 	hw_val = mxs_codec_read(codec, reg);
//
// 	if (hw_val != cached_val)
// 		return mxs_codec_write(codec, reg, cached_val);
//
// 	return 0;
// }
/* END SoC IO functions */

/* Codec routines */
#define VAG_BASE_VALUE  ((1400/2 - 625)/25)

static void mxs_codec_dac_set_vag(struct mxs_adc_priv *mxs_adc)
{
	u32 refctrl_val = __raw_readl(mxs_adc->aout_base + HW_AUDIOOUT_REFCTRL);

	refctrl_val &= ~(BM_AUDIOOUT_REFCTRL_VAG_VAL);
	refctrl_val &= ~(BM_AUDIOOUT_REFCTRL_VBG_ADJ);
	refctrl_val |= BF(VAG_BASE_VALUE, AUDIOOUT_REFCTRL_VAG_VAL) |
		BM_AUDIOOUT_REFCTRL_ADJ_VAG |
		BF(0xF, AUDIOOUT_REFCTRL_ADC_REFVAL) |
		BM_AUDIOOUT_REFCTRL_ADJ_ADC |
		BF(0x3, AUDIOOUT_REFCTRL_VBG_ADJ) | BM_AUDIOOUT_REFCTRL_RAISE_REF;

	__raw_writel(refctrl_val, mxs_adc->aout_base + HW_AUDIOOUT_REFCTRL);
}

static bool mxs_codec_dac_is_capless(struct mxs_adc_priv *mxs_adc)
{
	if ((__raw_readl(mxs_adc->aout_base + HW_AUDIOOUT_PWRDN)
		& BM_AUDIOOUT_PWRDN_CAPLESS) == 0)
		return false;
	else
		return true;
}

static void mxs_codec_dac_arm_short_cm(struct mxs_adc_priv *mxs_adc, bool bShort)
{
	__raw_writel(BF(3, AUDIOOUT_ANACTRL_SHORTMODE_CM),
		      mxs_adc->aout_base + HW_AUDIOOUT_ANACTRL_CLR);
	__raw_writel(BM_AUDIOOUT_ANACTRL_SHORT_CM_STS,
		      mxs_adc->aout_base + HW_AUDIOOUT_ANACTRL_CLR);
	if (bShort)
		__raw_writel(BF(1, AUDIOOUT_ANACTRL_SHORTMODE_CM),
		      mxs_adc->aout_base + HW_AUDIOOUT_ANACTRL_SET);
}

static void mxs_codec_dac_arm_short_lr(struct mxs_adc_priv *mxs_adc, bool bShort)
{
	__raw_writel(BF(3, AUDIOOUT_ANACTRL_SHORTMODE_LR),
		      mxs_adc->aout_base + HW_AUDIOOUT_ANACTRL_CLR);
	__raw_writel(BM_AUDIOOUT_ANACTRL_SHORT_LR_STS,
		      mxs_adc->aout_base + HW_AUDIOOUT_ANACTRL_CLR);
	if (bShort)
		__raw_writel(BF(1, AUDIOOUT_ANACTRL_SHORTMODE_LR),
		      mxs_adc->aout_base + HW_AUDIOOUT_ANACTRL_SET);
}

static void mxs_codec_dac_set_short_trip_level(struct mxs_adc_priv *mxs_adc, u8 u8level)
{
	__raw_writel((__raw_readl(mxs_adc->aout_base +
		HW_AUDIOOUT_ANACTRL)
		& (~BM_AUDIOOUT_ANACTRL_SHORT_LVLADJL)
		& (~BM_AUDIOOUT_ANACTRL_SHORT_LVLADJR))
		| BF(u8level, AUDIOOUT_ANACTRL_SHORT_LVLADJL)
		| BF(u8level, AUDIOOUT_ANACTRL_SHORT_LVLADJR),
		mxs_adc->aout_base + HW_AUDIOOUT_ANACTRL);
}

static void mxs_codec_dac_arm_short(struct mxs_adc_priv *mxs_adc, bool bLatchCM, bool bLatchLR)
{
	if (bLatchCM) {
		if (mxs_codec_dac_is_capless(mxs_adc))
			mxs_codec_dac_arm_short_cm(mxs_adc, true);
	} else
		mxs_codec_dac_arm_short_cm(mxs_adc, false);

	if (bLatchLR)
		mxs_codec_dac_arm_short_lr(mxs_adc, true);
	else
		mxs_codec_dac_arm_short_lr(mxs_adc, false);
}

static void
mxs_codec_dac_power_on(struct mxs_adc_priv *mxs_adc)
{
	/* Ungate DAC clocks */
	__raw_writel(BM_AUDIOOUT_CTRL_CLKGATE,
			mxs_adc->aout_base + HW_AUDIOOUT_CTRL_CLR);
	__raw_writel(BM_AUDIOOUT_ANACLKCTRL_CLKGATE,
			mxs_adc->aout_base + HW_AUDIOOUT_ANACLKCTRL_CLR);

	/* 16 bit word length */
	__raw_writel(BM_AUDIOOUT_CTRL_WORD_LENGTH,
		      mxs_adc->aout_base + HW_AUDIOOUT_CTRL_SET);

	/* Arm headphone LR short protect */
	mxs_codec_dac_set_short_trip_level(mxs_adc, 0);
	mxs_codec_dac_arm_short(mxs_adc, false, true);

	/* Update DAC volume over zero crossings */
	__raw_writel(BM_AUDIOOUT_DACVOLUME_EN_ZCD,
		      mxs_adc->aout_base + HW_AUDIOOUT_DACVOLUME_SET);
	/* Mute DAC */
	__raw_writel(BM_AUDIOOUT_DACVOLUME_MUTE_LEFT |
		      BM_AUDIOOUT_DACVOLUME_MUTE_RIGHT,
		      mxs_adc->aout_base + HW_AUDIOOUT_DACVOLUME_SET);

	/* Update HP volume over zero crossings */
	__raw_writel(BM_AUDIOOUT_HPVOL_EN_MSTR_ZCD,
		      mxs_adc->aout_base + HW_AUDIOOUT_HPVOL_SET);

	__raw_writel(BM_AUDIOOUT_ANACTRL_HP_CLASSAB,
		      mxs_adc->aout_base + HW_AUDIOOUT_ANACTRL_SET);

	/* Mute HP output */
	__raw_writel(BM_AUDIOOUT_HPVOL_MUTE,
		      mxs_adc->aout_base + HW_AUDIOOUT_HPVOL_SET);
	/* Mute speaker amp */
	__raw_writel(BM_AUDIOOUT_SPEAKERCTRL_MUTE,
		      mxs_adc->aout_base + HW_AUDIOOUT_SPEAKERCTRL_SET);
	/* Enable the audioout */
	 __raw_writel(BM_AUDIOOUT_CTRL_RUN,
			mxs_adc->aout_base + HW_AUDIOOUT_CTRL_SET);
}

static void
mxs_codec_dac_power_down(struct mxs_adc_priv *mxs_adc)
{
	/* Disable the audioout */
	 __raw_writel(BM_AUDIOOUT_CTRL_RUN,
		mxs_adc->aout_base + HW_AUDIOOUT_CTRL_CLR);
	/* Disable class AB */
	__raw_writel(BM_AUDIOOUT_ANACTRL_HP_CLASSAB,
			mxs_adc->aout_base + HW_AUDIOOUT_ANACTRL_CLR);

	/* Set hold to ground */
	__raw_writel(BM_AUDIOOUT_ANACTRL_HP_HOLD_GND,
		      mxs_adc->aout_base + HW_AUDIOOUT_ANACTRL_SET);

	/* Mute HP output */
	__raw_writel(BM_AUDIOOUT_HPVOL_MUTE,
		      mxs_adc->aout_base + HW_AUDIOOUT_HPVOL_SET);
	/* Power down HP output */
	__raw_writel(BM_AUDIOOUT_PWRDN_HEADPHONE,
		      mxs_adc->aout_base + HW_AUDIOOUT_PWRDN_SET);

	/* Mute speaker amp */
	__raw_writel(BM_AUDIOOUT_SPEAKERCTRL_MUTE,
		      mxs_adc->aout_base + HW_AUDIOOUT_SPEAKERCTRL_SET);
	/* Power down speaker amp */
	__raw_writel(BM_AUDIOOUT_PWRDN_SPEAKER,
		      mxs_adc->aout_base + HW_AUDIOOUT_PWRDN_SET);

	/* Mute DAC */
	__raw_writel(BM_AUDIOOUT_DACVOLUME_MUTE_LEFT |
		      BM_AUDIOOUT_DACVOLUME_MUTE_RIGHT,
		      mxs_adc->aout_base + HW_AUDIOOUT_DACVOLUME_SET);
	/* Power down DAC */
	__raw_writel(BM_AUDIOOUT_PWRDN_DAC,
		      mxs_adc->aout_base + HW_AUDIOOUT_PWRDN_SET);

	/* Gate DAC clocks */
	__raw_writel(BM_AUDIOOUT_ANACLKCTRL_CLKGATE,
		      mxs_adc->aout_base + HW_AUDIOOUT_ANACLKCTRL_SET);
	__raw_writel(BM_AUDIOOUT_CTRL_CLKGATE,
		      mxs_adc->aout_base + HW_AUDIOOUT_CTRL_SET);
}

static void
mxs_codec_adc_power_on(struct mxs_adc_priv *mxs_adc)
{
	u32 reg;

	/* Ungate ADC clocks */
	__raw_writel(BM_AUDIOIN_CTRL_CLKGATE,
			mxs_adc->ain_base + HW_AUDIOIN_CTRL_CLR);
	__raw_writel(BM_AUDIOIN_ANACLKCTRL_CLKGATE,
			mxs_adc->ain_base + HW_AUDIOIN_ANACLKCTRL_CLR);

	/* 16 bit word length */
	__raw_writel(BM_AUDIOIN_CTRL_WORD_LENGTH,
		      mxs_adc->ain_base + HW_AUDIOIN_CTRL_SET);

	/* Unmute ADC channels */
	__raw_writel(BM_AUDIOIN_ADCVOL_MUTE,
			mxs_adc->ain_base + HW_AUDIOIN_ADCVOL_CLR);

	/*
	 * The MUTE_LEFT and MUTE_RIGHT fields need to be cleared.
	 * They aren't presented in the datasheet, so this is hardcode.
	 */
	__raw_writel(0x01000100, mxs_adc->ain_base + HW_AUDIOIN_ADCVOLUME_CLR);

	/* Set the Input channel gain 3dB */
	__raw_writel(BM_AUDIOIN_ADCVOL_GAIN_LEFT,
			mxs_adc->ain_base + HW_AUDIOIN_ADCVOL_CLR);
	__raw_writel(BM_AUDIOIN_ADCVOL_GAIN_RIGHT,
			mxs_adc->ain_base + HW_AUDIOIN_ADCVOL_CLR);
	__raw_writel(BF(2, AUDIOIN_ADCVOL_GAIN_LEFT),
		      mxs_adc->ain_base + HW_AUDIOIN_ADCVOL_SET);
	__raw_writel(BF(2, AUDIOIN_ADCVOL_GAIN_RIGHT),
		      mxs_adc->ain_base + HW_AUDIOIN_ADCVOL_SET);

	/* Select default input - Microphone */
	__raw_writel(BM_AUDIOIN_ADCVOL_SELECT_LEFT,
			mxs_adc->ain_base + HW_AUDIOIN_ADCVOL_CLR);
	__raw_writel(BM_AUDIOIN_ADCVOL_SELECT_RIGHT,
			mxs_adc->ain_base + HW_AUDIOIN_ADCVOL_CLR);
	__raw_writel(BF
		      (BV_AUDIOIN_ADCVOL_SELECT__MIC,
		       AUDIOIN_ADCVOL_SELECT_LEFT),
		      mxs_adc->ain_base + HW_AUDIOIN_ADCVOL_SET);
	__raw_writel(BF
		      (BV_AUDIOIN_ADCVOL_SELECT__MIC,
		       AUDIOIN_ADCVOL_SELECT_RIGHT),
		      mxs_adc->ain_base + HW_AUDIOIN_ADCVOL_SET);

	/* Supply bias voltage to microphone */
	if (!mxs_adc->disable_mic_bias) {
		__raw_writel(BF(1, AUDIOIN_MICLINE_MIC_RESISTOR),
			      mxs_adc->ain_base + HW_AUDIOIN_MICLINE_SET);
		__raw_writel(BM_AUDIOIN_MICLINE_MIC_SELECT,
			      mxs_adc->ain_base + HW_AUDIOIN_MICLINE_CLR);
		__raw_writel(BF(1, AUDIOIN_MICLINE_MIC_GAIN),
			      mxs_adc->ain_base + HW_AUDIOIN_MICLINE_SET);
		__raw_writel(BF(7, AUDIOIN_MICLINE_MIC_BIAS),
			      mxs_adc->ain_base + HW_AUDIOIN_MICLINE_SET);
    }

	/* Set max ADC volume */
	reg = __raw_readl(mxs_adc->ain_base + HW_AUDIOIN_ADCVOLUME);
	reg &= ~BM_AUDIOIN_ADCVOLUME_VOLUME_LEFT;
	reg &= ~BM_AUDIOIN_ADCVOLUME_VOLUME_RIGHT;
	reg |= BF(ADC_VOLUME_MAX, AUDIOIN_ADCVOLUME_VOLUME_LEFT);
	reg |= BF(ADC_VOLUME_MAX, AUDIOIN_ADCVOLUME_VOLUME_RIGHT);
	__raw_writel(reg, mxs_adc->ain_base + HW_AUDIOIN_ADCVOLUME);
}

static void
mxs_codec_adc_power_down(struct mxs_adc_priv *mxs_adc)
{
	/* Mute ADC channels */
	__raw_writel(BM_AUDIOIN_ADCVOL_MUTE,
		      mxs_adc->ain_base + HW_AUDIOIN_ADCVOL_SET);

	/* Power Down ADC */
	__raw_writel(BM_AUDIOOUT_PWRDN_ADC | BM_AUDIOOUT_PWRDN_RIGHT_ADC,
		      mxs_adc->aout_base + HW_AUDIOOUT_PWRDN_SET);

	/* Gate ADC clocks */
	__raw_writel(BM_AUDIOIN_CTRL_CLKGATE,
		      mxs_adc->ain_base + HW_AUDIOIN_CTRL_SET);
	__raw_writel(BM_AUDIOIN_ANACLKCTRL_CLKGATE,
		      mxs_adc->ain_base + HW_AUDIOIN_ANACLKCTRL_SET);

	/* Disable bias voltage to microphone */
	__raw_writel(BF(0, AUDIOIN_MICLINE_MIC_RESISTOR),
		      mxs_adc->ain_base + HW_AUDIOIN_MICLINE_SET);
}

static void mxs_codec_dac_enable(struct mxs_adc_priv *mxs_adc)
{
	/* Move DAC codec out of reset */
	__raw_writel(BM_AUDIOOUT_CTRL_SFTRST,
		mxs_adc->aout_base + HW_AUDIOOUT_CTRL_CLR);

	/* Reduce analog power */
	__raw_writel(BM_AUDIOOUT_TEST_HP_I1_ADJ,
			mxs_adc->aout_base + HW_AUDIOOUT_TEST_CLR);
	__raw_writel(BF(0x1, AUDIOOUT_TEST_HP_I1_ADJ),
			mxs_adc->aout_base + HW_AUDIOOUT_TEST_SET);
	__raw_writel(BM_AUDIOOUT_REFCTRL_LOW_PWR,
			mxs_adc->aout_base + HW_AUDIOOUT_REFCTRL_SET);
	__raw_writel(BM_AUDIOOUT_REFCTRL_XTAL_BGR_BIAS,
			mxs_adc->aout_base + HW_AUDIOOUT_REFCTRL_SET);
	__raw_writel(BM_AUDIOOUT_REFCTRL_BIAS_CTRL,
			mxs_adc->aout_base + HW_AUDIOOUT_REFCTRL_CLR);
	__raw_writel(BF(0x1, AUDIOOUT_REFCTRL_BIAS_CTRL),
			mxs_adc->aout_base + HW_AUDIOOUT_REFCTRL_CLR);

	/* Set Vag value */
	mxs_codec_dac_set_vag(mxs_adc);

	/* Power on DAC codec */
	mxs_codec_dac_power_on(mxs_adc);
}

static void mxs_codec_dac_disable(struct mxs_adc_priv *mxs_adc)
{
	mxs_codec_dac_power_down(mxs_adc);
}

static void mxs_codec_adc_enable(struct mxs_adc_priv *mxs_adc)
{
	/* Move ADC codec out of reset */
	__raw_writel(BM_AUDIOIN_CTRL_SFTRST,
			mxs_adc->ain_base + HW_AUDIOIN_CTRL_CLR);

	/* Power on ADC codec */
	mxs_codec_adc_power_on(mxs_adc);
}

static void mxs_codec_adc_disable(struct mxs_adc_priv *mxs_adc)
{
	mxs_codec_adc_power_down(mxs_adc);
}

static void mxs_codec_startup(struct snd_soc_codec *codec)
{
	struct mxs_adc_priv *mxs_adc = snd_soc_codec_get_drvdata(codec);

	/* Soft reset DAC block */
	__raw_writel(BM_AUDIOOUT_CTRL_SFTRST,
		      mxs_adc->aout_base + HW_AUDIOOUT_CTRL_SET);
	while (!(__raw_readl(mxs_adc->aout_base + HW_AUDIOOUT_CTRL)
		& BM_AUDIOOUT_CTRL_CLKGATE)){
	}

	/* Soft reset ADC block */
	__raw_writel(BM_AUDIOIN_CTRL_SFTRST,
		      mxs_adc->ain_base + HW_AUDIOIN_CTRL_SET);
	while (!(__raw_readl(mxs_adc->ain_base + HW_AUDIOIN_CTRL)
		& BM_AUDIOIN_CTRL_CLKGATE)){
	}

	mxs_codec_dac_enable(mxs_adc);
	mxs_codec_adc_enable(mxs_adc);

	/* Sync regs and cache */
	mxs_codec_sync_reg_cache(codec);
}

static void mxs_codec_stop(struct snd_soc_codec *codec)
{
	struct mxs_adc_priv *mxs_adc = snd_soc_codec_get_drvdata(codec);

	mxs_codec_dac_disable(mxs_adc);
	mxs_codec_adc_disable(mxs_adc);
}
/* END Codec routines */

/* kcontrol */
static int dac_info_volsw(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0xf;
	return 0;
}

static int dac_get_volsw(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct mxs_adc_priv *mxs_adc = snd_soc_codec_get_drvdata(codec);
	int reg, l, r;
	int i;

	reg = __raw_readl(mxs_adc->aout_base + HW_AUDIOOUT_DACVOLUME);

	l = (reg & BM_AUDIOOUT_DACVOLUME_VOLUME_LEFT) >>
	    BP_AUDIOOUT_DACVOLUME_VOLUME_LEFT;
	r = (reg & BM_AUDIOOUT_DACVOLUME_VOLUME_RIGHT) >>
	    BP_AUDIOOUT_DACVOLUME_VOLUME_RIGHT;
	/*Left channel */
	i = 0;
	while (i < 16) {
		if (l == dac_volumn_control_word[i]) {
			ucontrol->value.integer.value[0] = i;
			break;
		}
		i++;
	}
	if (i == 16)
		ucontrol->value.integer.value[0] = i;
	/*Right channel */
	i = 0;
	while (i < 16) {
		if (r == dac_volumn_control_word[i]) {
			ucontrol->value.integer.value[1] = i;
			break;
		}
		i++;
	}
	if (i == 16)
		ucontrol->value.integer.value[1] = i;

	return 0;
}

static int dac_put_volsw(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct mxs_adc_priv *mxs_adc = snd_soc_codec_get_drvdata(codec);
	int reg, l, r;
	int i;

	i = ucontrol->value.integer.value[0];
	l = dac_volumn_control_word[i];
	/*Get dac volume for left channel */
	reg = BF(l, AUDIOOUT_DACVOLUME_VOLUME_LEFT);

	i = ucontrol->value.integer.value[1];
	r = dac_volumn_control_word[i];
	/*Get dac volume for right channel */
	reg = reg | BF(r, AUDIOOUT_DACVOLUME_VOLUME_RIGHT);

	/*Clear left/right dac volume */
	__raw_writel(BM_AUDIOOUT_DACVOLUME_VOLUME_LEFT |
			BM_AUDIOOUT_DACVOLUME_VOLUME_RIGHT,
			mxs_adc->aout_base + HW_AUDIOOUT_DACVOLUME_CLR);
	__raw_writel(reg, mxs_adc->aout_base + HW_AUDIOOUT_DACVOLUME_SET);

	return 0;
}

static const char *mxs_codec_adc_input_sel[] = {
	 "Mic", "Line In 1", "Head Phone", "Line In 2" };

static const char *mxs_codec_hp_output_sel[] = { "DAC Out", "Line In 1" };

static const char *mxs_codec_adc_3d_sel[] = {
	"Off", "Low", "Medium", "High" };

static const struct soc_enum mxs_codec_enum[] = {
	SOC_ENUM_SINGLE(ADC_ADCVOL_L, 12, 4, mxs_codec_adc_input_sel),
	SOC_ENUM_SINGLE(ADC_ADCVOL_L, 4, 4, mxs_codec_adc_input_sel),
	SOC_ENUM_SINGLE(DAC_HPVOL_H, 0, 2, mxs_codec_hp_output_sel),
	SOC_ENUM_SINGLE(DAC_CTRL_L, 8, 4, mxs_codec_adc_3d_sel),
};

static const struct snd_kcontrol_new mxs_snd_controls[] = {
	/* Playback Volume */
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "DAC Playback Volume",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
		SNDRV_CTL_ELEM_ACCESS_VOLATILE,
		.info = dac_info_volsw,
		.get = dac_get_volsw,
		.put = dac_put_volsw,
	 },

	SOC_DOUBLE_R("DAC Playback Switch",
		     DAC_VOLUME_H, DAC_VOLUME_L, 8, 0x01, 1),
	SOC_DOUBLE("HP Playback Volume", DAC_HPVOL_L, 8, 0, 0x7F, 1),

	/* Capture Volume */
	SOC_DOUBLE_R("ADC Capture Volume",
		     ADC_VOLUME_H, ADC_VOLUME_L, 0, 0xFF, 0),
	SOC_DOUBLE("ADC PGA Capture Volume", ADC_ADCVOL_L, 8, 0, 0x0F, 0),
	SOC_SINGLE("ADC PGA Capture Switch", ADC_ADCVOL_H, 8, 0x1, 1),
	SOC_SINGLE("Mic PGA Capture Volume", ADC_MICLINE_L, 0, 0x03, 0),

	/* Virtual 3D effect */
	SOC_ENUM("3D effect", mxs_codec_enum[3]),
};
/* END kcontrol */

/* DAPM */
static int pga_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct mxs_adc_priv *mxs_adc = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Prepare powering up HP and SPEAKER output */
		__raw_writel(BM_AUDIOOUT_ANACTRL_HP_HOLD_GND,
			mxs_adc->aout_base + HW_AUDIOOUT_ANACTRL_SET);
		__raw_writel(BM_RTC_PERSISTENT0_RELEASE_GND,
			mxs_adc->rtc_base + HW_RTC_PERSISTENT0_SET);
		msleep(100);
		break;
	case SND_SOC_DAPM_POST_PMU:
		__raw_writel(BM_AUDIOOUT_ANACTRL_HP_HOLD_GND,
			mxs_adc->aout_base + HW_AUDIOOUT_ANACTRL_CLR);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* For some reason, clearing RTC_PERSISTENT0_RELEASE_GND also affects
		 * capture (it appears related to mic bias), so clear this bit only if
		 * capture is not running.
		 */
		if (!(__raw_readl(mxs_adc->ain_base + HW_AUDIOIN_CTRL) & BM_AUDIOIN_CTRL_RUN))
			__raw_writel(BM_RTC_PERSISTENT0_RELEASE_GND,
					mxs_adc->rtc_base + HW_RTC_PERSISTENT0_CLR);
		break;
	}
	return 0;
}

static int adc_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct mxs_adc_priv *mxs_adc = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		__raw_writel(BM_RTC_PERSISTENT0_RELEASE_GND,
			mxs_adc->rtc_base + HW_RTC_PERSISTENT0_SET);
		msleep(100);
		break;
	case SND_SOC_DAPM_POST_PMD:
		__raw_writel(BM_RTC_PERSISTENT0_RELEASE_GND,
			mxs_adc->rtc_base + HW_RTC_PERSISTENT0_CLR);
		break;
	}
	return 0;
}

/* Left ADC Mux */
static const struct snd_kcontrol_new mxs_left_adc_controls =
SOC_DAPM_ENUM("Route", mxs_codec_enum[0]);

/* Right ADC Mux */
static const struct snd_kcontrol_new mxs_right_adc_controls =
SOC_DAPM_ENUM("Route", mxs_codec_enum[1]);

/* Head Phone Mux */
static const struct snd_kcontrol_new mxs_hp_controls =
SOC_DAPM_ENUM("Route", mxs_codec_enum[2]);

static const struct snd_soc_dapm_widget mxs_dapm_widgets[] = {
	SND_SOC_DAPM_ADC_E("ADC", "Capture", DAC_PWRDN_L, 8, 1, adc_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC("DAC", "Playback", DAC_PWRDN_L, 12, 1),

	SND_SOC_DAPM_MUX("Left ADC Mux", SND_SOC_NOPM, 0, 0,
			 &mxs_left_adc_controls),
	SND_SOC_DAPM_MUX("Right ADC Mux", SND_SOC_NOPM, 0, 0,
			 &mxs_right_adc_controls),
	SND_SOC_DAPM_MUX("HP Mux", SND_SOC_NOPM, 0, 0,
			 &mxs_hp_controls),
	SND_SOC_DAPM_PGA_E("HP AMP", DAC_PWRDN_L, 0, 1, NULL, 0, pga_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA("SPEAKER AMP", DAC_PWRDN_H, 8, 1, NULL, 0),
	SND_SOC_DAPM_INPUT("LINE1L"),
	SND_SOC_DAPM_INPUT("LINE1R"),
	SND_SOC_DAPM_INPUT("LINE2L"),
	SND_SOC_DAPM_INPUT("LINE2R"),
	SND_SOC_DAPM_INPUT("MIC"),

	SND_SOC_DAPM_OUTPUT("SPEAKER"),
	SND_SOC_DAPM_OUTPUT("HPL"),
	SND_SOC_DAPM_OUTPUT("HPR"),
};

/* routes for sgtl5000 */
static const struct snd_soc_dapm_route mxs_dapm_routes[] = {
	/* Left ADC Mux */
	{"Left ADC Mux", "Mic", "MIC"},
	{"Left ADC Mux", "Line In 1", "LINE1L"},
	{"Left ADC Mux", "Line In 2", "LINE2L"},
	{"Left ADC Mux", "Head Phone", "HPL"},

	/* Right ADC Mux */
	{"Right ADC Mux", "Mic", "MIC"},
	{"Right ADC Mux", "Line In 1", "LINE1R"},
	{"Right ADC Mux", "Line In 2", "LINE2R"},
	{"Right ADC Mux", "Head Phone", "HPR"},

	/* ADC */
	{"ADC", NULL, "Left ADC Mux"},
	{"ADC", NULL, "Right ADC Mux"},

	/* HP Mux */
	{"HP Mux", "DAC Out", "DAC"},
	{"HP Mux", "Line In 1", "LINE1L"},
	{"HP Mux", "Line In 1", "LINE1R"},

	/* HP amp */
	{"HP AMP", NULL, "HP Mux"},
	/* HP output */
	{"HPR", NULL, "HP AMP"},
	{"HPL", NULL, "HP AMP"},

	/* Speaker amp */
	{"SPEAKER AMP", NULL, "DAC"},
	{"SPEAKER", NULL, "SPEAKER AMP"},
};
/* END DAPM */

static int mxs_set_bias_level(struct snd_soc_codec *codec,
				   enum snd_soc_bias_level level)
{
	struct mxs_adc_priv *mxs_adc = snd_soc_codec_get_drvdata(codec);

	pr_debug("dapm level %d\n", level);

	if (mxs_adc->disable_mic_bias)
		return 0;

	switch (level) {
	case SND_SOC_BIAS_ON:		/* full On */
		break;

	case SND_SOC_BIAS_PREPARE:	/* partial On */
		/* Set Capless mode */
		__raw_writel(BM_AUDIOOUT_PWRDN_CAPLESS,
		      mxs_adc->aout_base + HW_AUDIOOUT_PWRDN_CLR);
		break;

	case SND_SOC_BIAS_STANDBY:	/* Off, with power */
		/* Unset Capless mode */
		__raw_writel(BM_AUDIOOUT_PWRDN_CAPLESS,
			mxs_adc->aout_base + HW_AUDIOOUT_PWRDN_SET);
		break;

	case SND_SOC_BIAS_OFF:	/* Off, without power */
		/* Unset Capless mode */
		__raw_writel(BM_AUDIOOUT_PWRDN_CAPLESS,
			mxs_adc->aout_base + HW_AUDIOOUT_PWRDN_SET);
		break;
	}

	return 0;
}

/* MXS-ADC Codec DAI driver */
static int mxs_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct mxs_adc_priv *mxs_adc = snd_soc_codec_get_drvdata(codec);
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	int i;
	u32 srr_value = 0;
	u32 src_hold = 0;

	i = get_srr_values(params_rate(params));
	if (i < 0)
		dev_warn(codec->dev, "%s doesn't support rate %d\n",
		       codec->component.name, params_rate(params));
	else {
		src_hold = srr_values[i].src_hold;

		srr_value =
		    BF(srr_values[i].basemult, AUDIOOUT_DACSRR_BASEMULT) |
		    BF(srr_values[i].src_int, AUDIOOUT_DACSRR_SRC_INT) |
		    BF(srr_values[i].src_frac, AUDIOOUT_DACSRR_SRC_FRAC) |
		    BF(src_hold, AUDIOOUT_DACSRR_SRC_HOLD);

		if (playback)
			__raw_writel(srr_value,
				     mxs_adc->aout_base + HW_AUDIOOUT_DACSRR);
		else
			__raw_writel(srr_value,
				     mxs_adc->ain_base + HW_AUDIOIN_ADCSRR);
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		if (playback)
			__raw_writel(BM_AUDIOOUT_CTRL_WORD_LENGTH,
				mxs_adc->aout_base + HW_AUDIOOUT_CTRL_SET);
		else
			__raw_writel(BM_AUDIOIN_CTRL_WORD_LENGTH,
				mxs_adc->ain_base + HW_AUDIOIN_CTRL_SET);

		break;

	case SNDRV_PCM_FORMAT_S32_LE:
		if (playback)
			__raw_writel(BM_AUDIOOUT_CTRL_WORD_LENGTH,
				mxs_adc->aout_base + HW_AUDIOOUT_CTRL_CLR);
		else
			__raw_writel(BM_AUDIOIN_CTRL_WORD_LENGTH,
				mxs_adc->ain_base + HW_AUDIOIN_CTRL_CLR);

		break;

	default:
		dev_warn(codec->dev, "%s doesn't support format %d\n",
		       codec->component.name, params_format(params));

	}

	return 0;
}

/* mute the codec used by alsa core */
static int mxs_codec_dig_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct mxs_adc_priv *mxs_adc = snd_soc_codec_get_drvdata(codec_dai->codec);
	int l, r;
	int ll, rr;
	u32 reg, reg1, reg2;
	u32 dac_mask = BM_AUDIOOUT_DACVOLUME_MUTE_LEFT |
	    BM_AUDIOOUT_DACVOLUME_MUTE_RIGHT;

	if (mute) {
		reg = __raw_readl(mxs_adc->aout_base + \
				HW_AUDIOOUT_DACVOLUME);

		reg1 = reg & ~BM_AUDIOOUT_DACVOLUME_VOLUME_LEFT;
		reg1 = reg1 & ~BM_AUDIOOUT_DACVOLUME_VOLUME_RIGHT;

		l = (reg & BM_AUDIOOUT_DACVOLUME_VOLUME_LEFT) >>
			BP_AUDIOOUT_DACVOLUME_VOLUME_LEFT;
		r = (reg & BM_AUDIOOUT_DACVOLUME_VOLUME_RIGHT) >>
			BP_AUDIOOUT_DACVOLUME_VOLUME_RIGHT;

		/* fade out dac vol */
		while ((l > DAC_VOLUME_MIN) || (r > DAC_VOLUME_MIN)) {
			l -= 0x8;
			r -= 0x8;
			ll = l > DAC_VOLUME_MIN ? l : DAC_VOLUME_MIN;
			rr = r > DAC_VOLUME_MIN ? r : DAC_VOLUME_MIN;
			reg2 = reg1 | BF_AUDIOOUT_DACVOLUME_VOLUME_LEFT(ll)
				| BF_AUDIOOUT_DACVOLUME_VOLUME_RIGHT(rr);
			__raw_writel(reg2,
				mxs_adc->aout_base + HW_AUDIOOUT_DACVOLUME);
			msleep(1);
		}

		__raw_writel(dac_mask,
			mxs_adc->aout_base + HW_AUDIOOUT_DACVOLUME_SET);
		reg = reg | dac_mask;
		__raw_writel(reg,
			mxs_adc->aout_base + HW_AUDIOOUT_DACVOLUME);
	} else
		__raw_writel(dac_mask,
			mxs_adc->aout_base + HW_AUDIOOUT_DACVOLUME_CLR);

	return 0;
}

#define MXS_ADC_RATES	SNDRV_PCM_RATE_8000_192000
#define MXS_ADC_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops mxs_codec_dai_ops = {
	.hw_params = mxs_pcm_hw_params,
	.digital_mute = mxs_codec_dig_mute,
};

static struct snd_soc_dai_driver mxs_codec_dai_driver = {
	.name = "mxs-builtin-codec-dai",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = MXS_ADC_RATES,
		.formats = MXS_ADC_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = MXS_ADC_RATES,
		.formats = MXS_ADC_FORMATS,
	},
	.ops = &mxs_codec_dai_ops,
};
/* END MXS-ADC Codec DAI driver */

/* MXS-ADC Codec driver */
static int mxs_codec_driver_probe(struct snd_soc_codec *codec)
{
	struct mxs_adc_priv *mxs_adc = snd_soc_codec_get_drvdata(codec);
	struct device *dev = codec->dev;
	struct device_node *np = dev->of_node;
	int ret = 0;

	if (np) {
		mxs_adc->disable_mic_bias = of_property_read_bool(np,
						"disable-mic-bias");

	}


	/* We don't use snd_soc_codec_set_cache_io because we are using
	 * our own IO functions: write, read. */

	mxs_codec_startup(codec);

	/* leading to standby state */
	ret = mxs_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	if (ret)
		goto err;







	return 0;

err:
	mxs_codec_stop(codec);

	return ret;
}

static int mxs_codec_driver_remove(struct snd_soc_codec *codec)
{
	mxs_codec_stop(codec);

	return 0;
}

// static int mxs_codec_driver_suspend(struct snd_soc_codec *codec)
// {
// 	/* TODO Enable power management. */
// 	return 0;
// }

// static int mxs_codec_driver_resume(struct snd_soc_codec *codec)
// {
// 	/* TODO Enable power management. */
// 	return 0;
// }

static struct snd_soc_codec_driver mxs_codec_driver = {
	.probe = mxs_codec_driver_probe,
	.remove = mxs_codec_driver_remove,
// 	.suspend = mxs_codec_driver_suspend,
// 	.resume = mxs_codec_driver_resume,
	.set_bias_level = mxs_set_bias_level,
	.reg_cache_size = ADC_REGNUM,
	.reg_word_size = sizeof(u16),
	.reg_cache_step = 1,
// 	.reg_cache_default = mxsadc_regs,
// 	.volatile_register = sgtl5000_volatile_register,
	.component_driver = {
		.controls = mxs_snd_controls,
		.num_controls = ARRAY_SIZE(mxs_snd_controls),
		.dapm_widgets = mxs_dapm_widgets,
		.num_dapm_widgets = ARRAY_SIZE(mxs_dapm_widgets),
		.dapm_routes = mxs_dapm_routes,
		.num_dapm_routes = ARRAY_SIZE(mxs_dapm_routes),
	},
	.write = mxs_codec_write,
	.read = mxs_codec_read,
};
/* END MXS-ADC Codec driver */

/* Underlying platform device that registers codec */
static int mxs_adc_probe(struct platform_device *pdev)
{
	struct mxs_adc_priv *mxs_adc;
	struct resource *r;
	int ret;

	mxs_adc = devm_kzalloc(&pdev->dev, sizeof(struct mxs_adc_priv), GFP_KERNEL);
	if (!mxs_adc)
		return -ENOMEM;

	platform_set_drvdata(pdev, mxs_adc);

	mxs_adc->disable_mic_bias = false;

	/* audio-in IO memory */
	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "audioin");
	if (IS_ERR(r)) {
		dev_err(&pdev->dev, "failed to get resource\n");
		return PTR_ERR(r);
	}

	mxs_adc->ain_base = devm_ioremap(&pdev->dev, r->start, resource_size(r));
	if (IS_ERR(mxs_adc->ain_base)) {
		dev_err(&pdev->dev, "ioremap failed\n");
		return PTR_ERR(mxs_adc->ain_base);
	}

	/* audio-out IO memory */
	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "audioout");
	if (IS_ERR(r)) {
		dev_err(&pdev->dev, "failed to get resource\n");
		return PTR_ERR(r);
	}

	mxs_adc->aout_base = devm_ioremap(&pdev->dev, r->start, resource_size(r));
	if (IS_ERR(mxs_adc->aout_base)) {
		dev_err(&pdev->dev, "ioremap failed\n");
		return PTR_ERR(mxs_adc->aout_base);
	}

	/* rtc IO memory */
	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rtc");
	if (IS_ERR(r)) {
		dev_err(&pdev->dev, "failed to get resource\n");
		return PTR_ERR(r);
	}

	mxs_adc->rtc_base = devm_ioremap(&pdev->dev, r->start, resource_size(r));
	if (IS_ERR(mxs_adc->rtc_base)) {
		dev_err(&pdev->dev, "ioremap failed\n");
		return PTR_ERR(mxs_adc->rtc_base);
	}

	/* Get audio clock */
	mxs_adc->clk = devm_clk_get(&pdev->dev, "filt");
	if (IS_ERR(mxs_adc->clk)) {
		ret = PTR_ERR(mxs_adc->clk);
		dev_err(&pdev->dev, "%s: Clock initialization failed\n", __func__);
		return ret;
	}

	/* Turn on audio clock */
	ret = clk_prepare_enable(mxs_adc->clk);
	if (unlikely(ret != 0)) {
		dev_err(&pdev->dev, "%s: Clock prepare or enable failed\n", __func__);
		return ret;
	}

	ret = snd_soc_register_codec(&pdev->dev,
			&mxs_codec_driver,&mxs_codec_dai_driver, 1);
	if (unlikely(ret != 0)) {
		dev_err(&pdev->dev, "Codec registration failed\n");
		goto disable_clk;
	}

	return 0;

disable_clk:
	clk_disable_unprepare(mxs_adc->clk);
	return ret;
}

static int mxs_adc_remove(struct platform_device *pdev)
{
	struct mxs_adc_priv *mxs_adc = platform_get_drvdata(pdev);

	clk_disable_unprepare(mxs_adc->clk);
	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

static const struct of_device_id mxs_adc_dt_ids[] = {
	{ .compatible = "fsl,mxs-builtin-codec", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxs_adc_dt_ids);

static struct platform_driver mxs_adc_driver = {
	.driver = {
		   .name = "mxs-builtin-codec",
		   .owner = THIS_MODULE,
		   .of_match_table = mxs_adc_dt_ids,
		   },
	.probe = mxs_adc_probe,
	.remove = mxs_adc_remove,
};

module_platform_driver(mxs_adc_driver);
/* END Underlying platform device that registers codec */

MODULE_DESCRIPTION("Freescale MXS ADC/DAC SoC Codec Driver");
MODULE_AUTHOR("Michal Ulianko <michal.ulianko@gmail.com>");
MODULE_LICENSE("GPL");
