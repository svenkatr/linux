/*
 * Renesas R-Car SSIU/SSI support
 *
 * Copyright (C) 2013 Renesas Solutions Corp.
 * Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 *
 * Based on fsi.c
 * Kuninori Morimoto <morimoto.kuninori@renesas.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/delay.h>
#include "rsnd.h"
#define RSND_SSI_NAME_SIZE 16

/*
 * SSICR
 */
#define	FORCE		(1 << 31)	/* Fixed */
#define	DMEN		(1 << 28)	/* DMA Enable */
#define	UIEN		(1 << 27)	/* Underflow Interrupt Enable */
#define	OIEN		(1 << 26)	/* Overflow Interrupt Enable */
#define	IIEN		(1 << 25)	/* Idle Mode Interrupt Enable */
#define	DIEN		(1 << 24)	/* Data Interrupt Enable */

#define	DWL_8		(0 << 19)	/* Data Word Length */
#define	DWL_16		(1 << 19)	/* Data Word Length */
#define	DWL_18		(2 << 19)	/* Data Word Length */
#define	DWL_20		(3 << 19)	/* Data Word Length */
#define	DWL_22		(4 << 19)	/* Data Word Length */
#define	DWL_24		(5 << 19)	/* Data Word Length */
#define	DWL_32		(6 << 19)	/* Data Word Length */

#define	SWL_32		(3 << 16)	/* R/W System Word Length */
#define	SCKD		(1 << 15)	/* Serial Bit Clock Direction */
#define	SWSD		(1 << 14)	/* Serial WS Direction */
#define	SCKP		(1 << 13)	/* Serial Bit Clock Polarity */
#define	SWSP		(1 << 12)	/* Serial WS Polarity */
#define	SDTA		(1 << 10)	/* Serial Data Alignment */
#define	DEL		(1 <<  8)	/* Serial Data Delay */
#define	CKDV(v)		(v <<  4)	/* Serial Clock Division Ratio */
#define	TRMD		(1 <<  1)	/* Transmit/Receive Mode Select */
#define	EN		(1 <<  0)	/* SSI Module Enable */

/*
 * SSISR
 */
#define	UIRQ		(1 << 27)	/* Underflow Error Interrupt Status */
#define	OIRQ		(1 << 26)	/* Overflow Error Interrupt Status */
#define	IIRQ		(1 << 25)	/* Idle Mode Interrupt Status */
#define	DIRQ		(1 << 24)	/* Data Interrupt Status Flag */

/*
 * SSIWSR
 */
#define CONT		(1 << 8)	/* WS Continue Function */

#define SSI_NAME "ssi"

struct rsnd_ssi {
	struct clk *clk;
	struct rsnd_ssi_platform_info *info; /* rcar_snd.h */
	struct rsnd_ssi *parent;
	struct rsnd_mod mod;

	struct rsnd_dai *rdai;
	u32 cr_own;
	u32 cr_clk;
	u32 cr_etc;
	int err;
	unsigned int usrcnt;
	unsigned int rate;
};

#define for_each_rsnd_ssi(pos, priv, i)					\
	for (i = 0;							\
	     (i < rsnd_ssi_nr(priv)) &&					\
		((pos) = ((struct rsnd_ssi *)(priv)->ssi + i));		\
	     i++)

#define rsnd_ssi_nr(priv) ((priv)->ssi_nr)
#define rsnd_mod_to_ssi(_mod) container_of((_mod), struct rsnd_ssi, mod)
#define rsnd_dma_to_ssi(dma)  rsnd_mod_to_ssi(rsnd_dma_to_mod(dma))
#define rsnd_ssi_pio_available(ssi) ((ssi)->info->pio_irq > 0)
#define rsnd_ssi_dma_available(ssi) \
	rsnd_dma_available(rsnd_mod_to_dma(&(ssi)->mod))
#define rsnd_ssi_clk_from_parent(ssi) ((ssi)->parent)
#define rsnd_ssi_mode_flags(p) ((p)->info->flags)
#define rsnd_ssi_dai_id(ssi) ((ssi)->info->dai_id)

static void rsnd_ssi_status_check(struct rsnd_mod *mod,
				  u32 bit)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(mod);
	struct device *dev = rsnd_priv_to_dev(priv);
	u32 status;
	int i;

	for (i = 0; i < 1024; i++) {
		status = rsnd_mod_read(mod, SSISR);
		if (status & bit)
			return;

		udelay(50);
	}

	dev_warn(dev, "status check failed\n");
}

static int rsnd_ssi_master_clk_start(struct rsnd_ssi *ssi,
				     struct rsnd_dai_stream *io)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(&ssi->mod);
	struct snd_pcm_runtime *runtime = rsnd_io_to_runtime(io);
	struct device *dev = rsnd_priv_to_dev(priv);
	int i, j, ret;
	int adg_clk_div_table[] = {
		1, 6, /* see adg.c */
	};
	int ssi_clk_mul_table[] = {
		1, 2, 4, 8, 16, 6, 12,
	};
	unsigned int main_rate;
	unsigned int rate = rsnd_src_get_ssi_rate(priv, io, runtime);

	/*
	 * Find best clock, and try to start ADG
	 */
	for (i = 0; i < ARRAY_SIZE(adg_clk_div_table); i++) {
		for (j = 0; j < ARRAY_SIZE(ssi_clk_mul_table); j++) {

			/*
			 * this driver is assuming that
			 * system word is 64fs (= 2 x 32bit)
			 * see rsnd_ssi_init()
			 */
			main_rate = rate / adg_clk_div_table[i]
				* 32 * 2 * ssi_clk_mul_table[j];

			ret = rsnd_adg_ssi_clk_try_start(&ssi->mod, main_rate);
			if (0 == ret) {
				ssi->rate	= rate;
				ssi->cr_clk	= FORCE | SWL_32 |
						  SCKD | SWSD | CKDV(j);

				dev_dbg(dev, "ssi%d outputs %u Hz\n",
					rsnd_mod_id(&ssi->mod), rate);

				return 0;
			}
		}
	}

	dev_err(dev, "unsupported clock rate\n");
	return -EIO;
}

static void rsnd_ssi_master_clk_stop(struct rsnd_ssi *ssi)
{
	ssi->rate = 0;
	ssi->cr_clk = 0;
	rsnd_adg_ssi_clk_stop(&ssi->mod);
}

static void rsnd_ssi_hw_start(struct rsnd_ssi *ssi,
			      struct rsnd_dai *rdai,
			      struct rsnd_dai_stream *io)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(&ssi->mod);
	struct device *dev = rsnd_priv_to_dev(priv);
	u32 cr;

	if (0 == ssi->usrcnt) {
		clk_prepare_enable(ssi->clk);

		if (rsnd_dai_is_clk_master(rdai)) {
			if (rsnd_ssi_clk_from_parent(ssi))
				rsnd_ssi_hw_start(ssi->parent, rdai, io);
			else
				rsnd_ssi_master_clk_start(ssi, io);
		}
	}

	cr  =	ssi->cr_own	|
		ssi->cr_clk	|
		ssi->cr_etc	|
		EN;

	rsnd_mod_write(&ssi->mod, SSICR, cr);

	ssi->usrcnt++;

	dev_dbg(dev, "ssi%d hw started\n", rsnd_mod_id(&ssi->mod));
}

static void rsnd_ssi_hw_stop(struct rsnd_ssi *ssi,
			     struct rsnd_dai *rdai)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(&ssi->mod);
	struct device *dev = rsnd_priv_to_dev(priv);
	u32 cr;

	if (0 == ssi->usrcnt) /* stop might be called without start */
		return;

	ssi->usrcnt--;

	if (0 == ssi->usrcnt) {
		/*
		 * disable all IRQ,
		 * and, wait all data was sent
		 */
		cr  =	ssi->cr_own	|
			ssi->cr_clk;

		rsnd_mod_write(&ssi->mod, SSICR, cr | EN);
		rsnd_ssi_status_check(&ssi->mod, DIRQ);

		/*
		 * disable SSI,
		 * and, wait idle state
		 */
		rsnd_mod_write(&ssi->mod, SSICR, cr);	/* disabled all */
		rsnd_ssi_status_check(&ssi->mod, IIRQ);

		if (rsnd_dai_is_clk_master(rdai)) {
			if (rsnd_ssi_clk_from_parent(ssi))
				rsnd_ssi_hw_stop(ssi->parent, rdai);
			else
				rsnd_ssi_master_clk_stop(ssi);
		}

		clk_disable_unprepare(ssi->clk);
	}

	dev_dbg(dev, "ssi%d hw stopped\n", rsnd_mod_id(&ssi->mod));
}

/*
 *	SSI mod common functions
 */
static int rsnd_ssi_init(struct rsnd_mod *mod,
			 struct rsnd_dai *rdai)
{
	struct rsnd_ssi *ssi = rsnd_mod_to_ssi(mod);
	struct rsnd_dai_stream *io = rsnd_mod_to_io(mod);
	struct snd_pcm_runtime *runtime = rsnd_io_to_runtime(io);
	u32 cr;

	cr = FORCE;

	/*
	 * always use 32bit system word for easy clock calculation.
	 * see also rsnd_ssi_master_clk_enable()
	 */
	cr |= SWL_32;

	/*
	 * init clock settings for SSICR
	 */
	switch (runtime->sample_bits) {
	case 16:
		cr |= DWL_16;
		break;
	case 32:
		cr |= DWL_24;
		break;
	default:
		return -EIO;
	}

	if (rdai->bit_clk_inv)
		cr |= SCKP;
	if (rdai->frm_clk_inv)
		cr |= SWSP;
	if (rdai->data_alignment)
		cr |= SDTA;
	if (rdai->sys_delay)
		cr |= DEL;
	if (rsnd_dai_is_play(rdai, io))
		cr |= TRMD;

	/*
	 * set ssi parameter
	 */
	ssi->rdai	= rdai;
	ssi->cr_own	= cr;
	ssi->err	= -1; /* ignore 1st error */

	rsnd_src_ssi_mode_init(mod, rdai);

	return 0;
}

static int rsnd_ssi_quit(struct rsnd_mod *mod,
			 struct rsnd_dai *rdai)
{
	struct rsnd_ssi *ssi = rsnd_mod_to_ssi(mod);
	struct rsnd_priv *priv = rsnd_mod_to_priv(mod);
	struct device *dev = rsnd_priv_to_dev(priv);

	if (ssi->err > 0)
		dev_warn(dev, "ssi under/over flow err = %d\n", ssi->err);

	ssi->rdai	= NULL;
	ssi->cr_own	= 0;
	ssi->err	= 0;

	return 0;
}

static void rsnd_ssi_record_error(struct rsnd_ssi *ssi, u32 status)
{
	/* under/over flow error */
	if (status & (UIRQ | OIRQ)) {
		ssi->err++;

		/* clear error status */
		rsnd_mod_write(&ssi->mod, SSISR, 0);
	}
}

/*
 *		SSI PIO
 */
static irqreturn_t rsnd_ssi_pio_interrupt(int irq, void *data)
{
	struct rsnd_ssi *ssi = data;
	struct rsnd_mod *mod = &ssi->mod;
	struct rsnd_dai_stream *io = rsnd_mod_to_io(mod);
	u32 status = rsnd_mod_read(mod, SSISR);
	irqreturn_t ret = IRQ_NONE;

	if (io && (status & DIRQ)) {
		struct rsnd_dai *rdai = ssi->rdai;
		struct snd_pcm_runtime *runtime = rsnd_io_to_runtime(io);
		u32 *buf = (u32 *)(runtime->dma_area +
				   rsnd_dai_pointer_offset(io, 0));

		rsnd_ssi_record_error(ssi, status);

		/*
		 * 8/16/32 data can be assesse to TDR/RDR register
		 * directly as 32bit data
		 * see rsnd_ssi_init()
		 */
		if (rsnd_dai_is_play(rdai, io))
			rsnd_mod_write(mod, SSITDR, *buf);
		else
			*buf = rsnd_mod_read(mod, SSIRDR);

		rsnd_dai_pointer_update(io, sizeof(*buf));

		ret = IRQ_HANDLED;
	}

	return ret;
}

static int rsnd_ssi_pio_probe(struct rsnd_mod *mod,
			      struct rsnd_dai *rdai)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(mod);
	struct device *dev = rsnd_priv_to_dev(priv);
	struct rsnd_ssi *ssi = rsnd_mod_to_ssi(mod);
	int irq = ssi->info->pio_irq;
	int ret;

	ret = devm_request_irq(dev, irq,
			       rsnd_ssi_pio_interrupt,
			       IRQF_SHARED,
			       dev_name(dev), ssi);
	if (ret)
		dev_err(dev, "SSI request interrupt failed\n");

	dev_dbg(dev, "%s (PIO) is probed\n", rsnd_mod_name(mod));

	return ret;
}

static int rsnd_ssi_pio_start(struct rsnd_mod *mod,
			      struct rsnd_dai *rdai)
{
	struct rsnd_ssi *ssi = rsnd_mod_to_ssi(mod);
	struct rsnd_dai_stream *io = rsnd_mod_to_io(mod);

	/* enable PIO IRQ */
	ssi->cr_etc = UIEN | OIEN | DIEN;

	rsnd_src_enable_ssi_irq(mod, rdai);

	rsnd_ssi_hw_start(ssi, rdai, io);

	return 0;
}

static int rsnd_ssi_pio_stop(struct rsnd_mod *mod,
			     struct rsnd_dai *rdai)
{
	struct rsnd_ssi *ssi = rsnd_mod_to_ssi(mod);

	ssi->cr_etc = 0;

	rsnd_ssi_hw_stop(ssi, rdai);

	return 0;
}

static struct rsnd_mod_ops rsnd_ssi_pio_ops = {
	.name	= SSI_NAME,
	.probe	= rsnd_ssi_pio_probe,
	.init	= rsnd_ssi_init,
	.quit	= rsnd_ssi_quit,
	.start	= rsnd_ssi_pio_start,
	.stop	= rsnd_ssi_pio_stop,
};

static int rsnd_ssi_dma_probe(struct rsnd_mod *mod,
			  struct rsnd_dai *rdai)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(mod);
	struct rsnd_ssi *ssi = rsnd_mod_to_ssi(mod);
	struct device *dev = rsnd_priv_to_dev(priv);
	int dma_id = ssi->info->dma_id;
	int ret;

	ret = rsnd_dma_init(
		priv, rsnd_mod_to_dma(mod),
		rsnd_info_is_playback(priv, ssi),
		dma_id);

	if (ret < 0)
		dev_err(dev, "SSI DMA failed\n");

	dev_dbg(dev, "%s (DMA) is probed\n", rsnd_mod_name(mod));

	return ret;
}

static int rsnd_ssi_dma_remove(struct rsnd_mod *mod,
			       struct rsnd_dai *rdai)
{
	rsnd_dma_quit(rsnd_mod_to_priv(mod), rsnd_mod_to_dma(mod));

	return 0;
}

static int rsnd_ssi_dma_start(struct rsnd_mod *mod,
			      struct rsnd_dai *rdai)
{
	struct rsnd_ssi *ssi = rsnd_mod_to_ssi(mod);
	struct rsnd_dma *dma = rsnd_mod_to_dma(&ssi->mod);
	struct rsnd_dai_stream *io = rsnd_mod_to_io(mod);

	/* enable DMA transfer */
	ssi->cr_etc = DMEN;

	rsnd_dma_start(dma);

	rsnd_ssi_hw_start(ssi, ssi->rdai, io);

	/* enable WS continue */
	if (rsnd_dai_is_clk_master(rdai))
		rsnd_mod_write(&ssi->mod, SSIWSR, CONT);

	return 0;
}

static int rsnd_ssi_dma_stop(struct rsnd_mod *mod,
			     struct rsnd_dai *rdai)
{
	struct rsnd_ssi *ssi = rsnd_mod_to_ssi(mod);
	struct rsnd_dma *dma = rsnd_mod_to_dma(&ssi->mod);

	ssi->cr_etc = 0;

	rsnd_ssi_record_error(ssi, rsnd_mod_read(mod, SSISR));

	rsnd_ssi_hw_stop(ssi, rdai);

	rsnd_dma_stop(dma);

	return 0;
}

static struct rsnd_mod_ops rsnd_ssi_dma_ops = {
	.name	= SSI_NAME,
	.probe	= rsnd_ssi_dma_probe,
	.remove	= rsnd_ssi_dma_remove,
	.init	= rsnd_ssi_init,
	.quit	= rsnd_ssi_quit,
	.start	= rsnd_ssi_dma_start,
	.stop	= rsnd_ssi_dma_stop,
};

/*
 *		Non SSI
 */
static struct rsnd_mod_ops rsnd_ssi_non_ops = {
	.name	= SSI_NAME,
};

/*
 *		ssi mod function
 */
struct rsnd_mod *rsnd_ssi_mod_get(struct rsnd_priv *priv, int id)
{
	if (WARN_ON(id < 0 || id >= rsnd_ssi_nr(priv)))
		id = 0;

	return &((struct rsnd_ssi *)(priv->ssi) + id)->mod;
}

int rsnd_ssi_is_pin_sharing(struct rsnd_mod *mod)
{
	struct rsnd_ssi *ssi = rsnd_mod_to_ssi(mod);

	return !!(rsnd_ssi_mode_flags(ssi) & RSND_SSI_CLK_PIN_SHARE);
}

static void rsnd_ssi_parent_clk_setup(struct rsnd_priv *priv, struct rsnd_ssi *ssi)
{
	if (!rsnd_ssi_is_pin_sharing(&ssi->mod))
		return;

	switch (rsnd_mod_id(&ssi->mod)) {
	case 1:
	case 2:
		ssi->parent = rsnd_mod_to_ssi(rsnd_ssi_mod_get(priv, 0));
		break;
	case 4:
		ssi->parent = rsnd_mod_to_ssi(rsnd_ssi_mod_get(priv, 3));
		break;
	case 8:
		ssi->parent = rsnd_mod_to_ssi(rsnd_ssi_mod_get(priv, 7));
		break;
	}
}


static void rsnd_of_parse_ssi(struct platform_device *pdev,
			      const struct rsnd_of_data *of_data,
			      struct rsnd_priv *priv)
{
	struct device_node *node;
	struct device_node *np;
	struct rsnd_ssi_platform_info *ssi_info;
	struct rcar_snd_info *info = rsnd_priv_to_info(priv);
	struct device *dev = &pdev->dev;
	int nr, i;

	if (!of_data)
		return;

	node = of_get_child_by_name(dev->of_node, "rcar_sound,ssi");
	if (!node)
		return;

	nr = of_get_child_count(node);
	if (!nr)
		goto rsnd_of_parse_ssi_end;

	ssi_info = devm_kzalloc(dev,
				sizeof(struct rsnd_ssi_platform_info) * nr,
				GFP_KERNEL);
	if (!ssi_info) {
		dev_err(dev, "ssi info allocation error\n");
		goto rsnd_of_parse_ssi_end;
	}

	info->ssi_info		= ssi_info;
	info->ssi_info_nr	= nr;

	i = -1;
	for_each_child_of_node(node, np) {
		i++;

		ssi_info = info->ssi_info + i;

		/*
		 * pin settings
		 */
		if (of_get_property(np, "shared-pin", NULL))
			ssi_info->flags |= RSND_SSI_CLK_PIN_SHARE;

		/*
		 * irq
		 */
		ssi_info->pio_irq = irq_of_parse_and_map(np, 0);

		/*
		 * DMA
		 */
		ssi_info->dma_id = of_get_property(np, "pio-transfer", NULL) ?
			0 : 1;
	}

rsnd_of_parse_ssi_end:
	of_node_put(node);
}

int rsnd_ssi_probe(struct platform_device *pdev,
		   const struct rsnd_of_data *of_data,
		   struct rsnd_priv *priv)
{
	struct rcar_snd_info *info = rsnd_priv_to_info(priv);
	struct rsnd_ssi_platform_info *pinfo;
	struct device *dev = rsnd_priv_to_dev(priv);
	struct rsnd_mod_ops *ops;
	struct clk *clk;
	struct rsnd_ssi *ssi;
	char name[RSND_SSI_NAME_SIZE];
	int i, nr;

	rsnd_of_parse_ssi(pdev, of_data, priv);

	/*
	 *	init SSI
	 */
	nr	= info->ssi_info_nr;
	ssi	= devm_kzalloc(dev, sizeof(*ssi) * nr, GFP_KERNEL);
	if (!ssi) {
		dev_err(dev, "SSI allocate failed\n");
		return -ENOMEM;
	}

	priv->ssi	= ssi;
	priv->ssi_nr	= nr;

	for_each_rsnd_ssi(ssi, priv, i) {
		pinfo = &info->ssi_info[i];

		snprintf(name, RSND_SSI_NAME_SIZE, "%s.%d",
			 SSI_NAME, i);

		clk = devm_clk_get(dev, name);
		if (IS_ERR(clk))
			return PTR_ERR(clk);

		ssi->info	= pinfo;
		ssi->clk	= clk;

		ops = &rsnd_ssi_non_ops;
		if (pinfo->dma_id > 0)
			ops = &rsnd_ssi_dma_ops;
		else if (rsnd_ssi_pio_available(ssi))
			ops = &rsnd_ssi_pio_ops;

		rsnd_mod_init(priv, &ssi->mod, ops, RSND_MOD_SSI, i);

		rsnd_ssi_parent_clk_setup(priv, ssi);
	}

	return 0;
}
