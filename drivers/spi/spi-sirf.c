/*
 * SPI bus driver for CSR SiRFprimaII
 *
 * Copyright (c) 2011 Cambridge Silicon Radio Limited, a CSR plc group company.
 *
 * Licensed under GPLv2 or later.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/dmaengine.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>

#define DRIVER_NAME "sirfsoc_spi"

#define SIRFSOC_SPI_CTRL		0x0000
#define SIRFSOC_SPI_CMD			0x0004
#define SIRFSOC_SPI_TX_RX_EN		0x0008
#define SIRFSOC_SPI_INT_EN		0x000C
#define SIRFSOC_SPI_INT_STATUS		0x0010
#define SIRFSOC_SPI_TX_DMA_IO_CTRL	0x0100
#define SIRFSOC_SPI_TX_DMA_IO_LEN	0x0104
#define SIRFSOC_SPI_TXFIFO_CTRL		0x0108
#define SIRFSOC_SPI_TXFIFO_LEVEL_CHK	0x010C
#define SIRFSOC_SPI_TXFIFO_OP		0x0110
#define SIRFSOC_SPI_TXFIFO_STATUS	0x0114
#define SIRFSOC_SPI_TXFIFO_DATA		0x0118
#define SIRFSOC_SPI_RX_DMA_IO_CTRL	0x0120
#define SIRFSOC_SPI_RX_DMA_IO_LEN	0x0124
#define SIRFSOC_SPI_RXFIFO_CTRL		0x0128
#define SIRFSOC_SPI_RXFIFO_LEVEL_CHK	0x012C
#define SIRFSOC_SPI_RXFIFO_OP		0x0130
#define SIRFSOC_SPI_RXFIFO_STATUS	0x0134
#define SIRFSOC_SPI_RXFIFO_DATA		0x0138
#define SIRFSOC_SPI_DUMMY_DELAY_CTL	0x0144

/* SPI CTRL register defines */
#define SIRFSOC_SPI_SLV_MODE		BIT(16)
#define SIRFSOC_SPI_CMD_MODE		BIT(17)
#define SIRFSOC_SPI_CS_IO_OUT		BIT(18)
#define SIRFSOC_SPI_CS_IO_MODE		BIT(19)
#define SIRFSOC_SPI_CLK_IDLE_STAT	BIT(20)
#define SIRFSOC_SPI_CS_IDLE_STAT	BIT(21)
#define SIRFSOC_SPI_TRAN_MSB		BIT(22)
#define SIRFSOC_SPI_DRV_POS_EDGE	BIT(23)
#define SIRFSOC_SPI_CS_HOLD_TIME	BIT(24)
#define SIRFSOC_SPI_CLK_SAMPLE_MODE	BIT(25)
#define SIRFSOC_SPI_TRAN_DAT_FORMAT_8	(0 << 26)
#define SIRFSOC_SPI_TRAN_DAT_FORMAT_12	(1 << 26)
#define SIRFSOC_SPI_TRAN_DAT_FORMAT_16	(2 << 26)
#define SIRFSOC_SPI_TRAN_DAT_FORMAT_32	(3 << 26)
#define SIRFSOC_SPI_CMD_BYTE_NUM(x)		((x & 3) << 28)
#define SIRFSOC_SPI_ENA_AUTO_CLR		BIT(30)
#define SIRFSOC_SPI_MUL_DAT_MODE		BIT(31)

/* Interrupt Enable */
#define SIRFSOC_SPI_RX_DONE_INT_EN		BIT(0)
#define SIRFSOC_SPI_TX_DONE_INT_EN		BIT(1)
#define SIRFSOC_SPI_RX_OFLOW_INT_EN		BIT(2)
#define SIRFSOC_SPI_TX_UFLOW_INT_EN		BIT(3)
#define SIRFSOC_SPI_RX_IO_DMA_INT_EN	BIT(4)
#define SIRFSOC_SPI_TX_IO_DMA_INT_EN	BIT(5)
#define SIRFSOC_SPI_RXFIFO_FULL_INT_EN	BIT(6)
#define SIRFSOC_SPI_TXFIFO_EMPTY_INT_EN	BIT(7)
#define SIRFSOC_SPI_RXFIFO_THD_INT_EN	BIT(8)
#define SIRFSOC_SPI_TXFIFO_THD_INT_EN	BIT(9)
#define SIRFSOC_SPI_FRM_END_INT_EN	BIT(10)

#define SIRFSOC_SPI_INT_MASK_ALL		0x1FFF

/* Interrupt status */
#define SIRFSOC_SPI_RX_DONE		BIT(0)
#define SIRFSOC_SPI_TX_DONE		BIT(1)
#define SIRFSOC_SPI_RX_OFLOW		BIT(2)
#define SIRFSOC_SPI_TX_UFLOW		BIT(3)
#define SIRFSOC_SPI_RX_FIFO_FULL	BIT(6)
#define SIRFSOC_SPI_TXFIFO_EMPTY	BIT(7)
#define SIRFSOC_SPI_RXFIFO_THD_REACH	BIT(8)
#define SIRFSOC_SPI_TXFIFO_THD_REACH	BIT(9)
#define SIRFSOC_SPI_FRM_END		BIT(10)

/* TX RX enable */
#define SIRFSOC_SPI_RX_EN		BIT(0)
#define SIRFSOC_SPI_TX_EN		BIT(1)
#define SIRFSOC_SPI_CMD_TX_EN		BIT(2)

#define SIRFSOC_SPI_IO_MODE_SEL		BIT(0)
#define SIRFSOC_SPI_RX_DMA_FLUSH	BIT(2)

/* FIFO OPs */
#define SIRFSOC_SPI_FIFO_RESET		BIT(0)
#define SIRFSOC_SPI_FIFO_START		BIT(1)

/* FIFO CTRL */
#define SIRFSOC_SPI_FIFO_WIDTH_BYTE	(0 << 0)
#define SIRFSOC_SPI_FIFO_WIDTH_WORD	(1 << 0)
#define SIRFSOC_SPI_FIFO_WIDTH_DWORD	(2 << 0)

/* FIFO Status */
#define	SIRFSOC_SPI_FIFO_LEVEL_MASK	0xFF
#define SIRFSOC_SPI_FIFO_FULL		BIT(8)
#define SIRFSOC_SPI_FIFO_EMPTY		BIT(9)

/* 256 bytes rx/tx FIFO */
#define SIRFSOC_SPI_FIFO_SIZE		256
#define SIRFSOC_SPI_DAT_FRM_LEN_MAX	(64 * 1024)

#define SIRFSOC_SPI_FIFO_SC(x)		((x) & 0x3F)
#define SIRFSOC_SPI_FIFO_LC(x)		(((x) & 0x3F) << 10)
#define SIRFSOC_SPI_FIFO_HC(x)		(((x) & 0x3F) << 20)
#define SIRFSOC_SPI_FIFO_THD(x)		(((x) & 0xFF) << 2)

/*
 * only if the rx/tx buffer and transfer size are 4-bytes aligned, we use dma
 * due to the limitation of dma controller
 */

#define ALIGNED(x) (!((u32)x & 0x3))
#define IS_DMA_VALID(x) (x && ALIGNED(x->tx_buf) && ALIGNED(x->rx_buf) && \
	ALIGNED(x->len) && (x->len < 2 * PAGE_SIZE))

#define SIRFSOC_MAX_CMD_BYTES	4

struct sirfsoc_spi {
	struct spi_bitbang bitbang;
	struct completion rx_done;
	struct completion tx_done;

	void __iomem *base;
	u32 ctrl_freq;  /* SPI controller clock speed */
	struct clk *clk;

	/* rx & tx bufs from the spi_transfer */
	const void *tx;
	void *rx;

	/* place received word into rx buffer */
	void (*rx_word) (struct sirfsoc_spi *);
	/* get word from tx buffer for sending */
	void (*tx_word) (struct sirfsoc_spi *);

	/* number of words left to be tranmitted/received */
	unsigned int left_tx_word;
	unsigned int left_rx_word;

	/* rx & tx DMA channels */
	struct dma_chan *rx_chan;
	struct dma_chan *tx_chan;
	dma_addr_t src_start;
	dma_addr_t dst_start;
	void *dummypage;
	int word_width; /* in bytes */

	/*
	 * if tx size is not more than 4 and rx size is NULL, use
	 * command model
	 */
	bool	tx_by_cmd;

	int chipselect[0];
};

static void spi_sirfsoc_rx_word_u8(struct sirfsoc_spi *sspi)
{
	u32 data;
	u8 *rx = sspi->rx;

	data = readl(sspi->base + SIRFSOC_SPI_RXFIFO_DATA);

	if (rx) {
		*rx++ = (u8) data;
		sspi->rx = rx;
	}

	sspi->left_rx_word--;
}

static void spi_sirfsoc_tx_word_u8(struct sirfsoc_spi *sspi)
{
	u32 data = 0;
	const u8 *tx = sspi->tx;

	if (tx) {
		data = *tx++;
		sspi->tx = tx;
	}

	writel(data, sspi->base + SIRFSOC_SPI_TXFIFO_DATA);
	sspi->left_tx_word--;
}

static void spi_sirfsoc_rx_word_u16(struct sirfsoc_spi *sspi)
{
	u32 data;
	u16 *rx = sspi->rx;

	data = readl(sspi->base + SIRFSOC_SPI_RXFIFO_DATA);

	if (rx) {
		*rx++ = (u16) data;
		sspi->rx = rx;
	}

	sspi->left_rx_word--;
}

static void spi_sirfsoc_tx_word_u16(struct sirfsoc_spi *sspi)
{
	u32 data = 0;
	const u16 *tx = sspi->tx;

	if (tx) {
		data = *tx++;
		sspi->tx = tx;
	}

	writel(data, sspi->base + SIRFSOC_SPI_TXFIFO_DATA);
	sspi->left_tx_word--;
}

static void spi_sirfsoc_rx_word_u32(struct sirfsoc_spi *sspi)
{
	u32 data;
	u32 *rx = sspi->rx;

	data = readl(sspi->base + SIRFSOC_SPI_RXFIFO_DATA);

	if (rx) {
		*rx++ = (u32) data;
		sspi->rx = rx;
	}

	sspi->left_rx_word--;

}

static void spi_sirfsoc_tx_word_u32(struct sirfsoc_spi *sspi)
{
	u32 data = 0;
	const u32 *tx = sspi->tx;

	if (tx) {
		data = *tx++;
		sspi->tx = tx;
	}

	writel(data, sspi->base + SIRFSOC_SPI_TXFIFO_DATA);
	sspi->left_tx_word--;
}

static irqreturn_t spi_sirfsoc_irq(int irq, void *dev_id)
{
	struct sirfsoc_spi *sspi = dev_id;
	u32 spi_stat = readl(sspi->base + SIRFSOC_SPI_INT_STATUS);

	writel(spi_stat, sspi->base + SIRFSOC_SPI_INT_STATUS);

	if (sspi->tx_by_cmd && (spi_stat & SIRFSOC_SPI_FRM_END)) {
		complete(&sspi->tx_done);
		writel(0x0, sspi->base + SIRFSOC_SPI_INT_EN);
		return IRQ_HANDLED;
	}

	/* Error Conditions */
	if (spi_stat & SIRFSOC_SPI_RX_OFLOW ||
			spi_stat & SIRFSOC_SPI_TX_UFLOW) {
		complete(&sspi->rx_done);
		writel(0x0, sspi->base + SIRFSOC_SPI_INT_EN);
	}

	if (spi_stat & (SIRFSOC_SPI_FRM_END
			| SIRFSOC_SPI_RXFIFO_THD_REACH))
		while (!((readl(sspi->base + SIRFSOC_SPI_RXFIFO_STATUS)
				& SIRFSOC_SPI_FIFO_EMPTY)) &&
				sspi->left_rx_word)
			sspi->rx_word(sspi);

	if (spi_stat & (SIRFSOC_SPI_TXFIFO_EMPTY |
			SIRFSOC_SPI_TXFIFO_THD_REACH))
		while (!((readl(sspi->base + SIRFSOC_SPI_TXFIFO_STATUS)
				& SIRFSOC_SPI_FIFO_FULL)) &&
				sspi->left_tx_word)
			sspi->tx_word(sspi);

	/* Received all words */
	if ((sspi->left_rx_word == 0) && (sspi->left_tx_word == 0)) {
		complete(&sspi->rx_done);
		writel(0x0, sspi->base + SIRFSOC_SPI_INT_EN);
	}
	return IRQ_HANDLED;
}

static void spi_sirfsoc_dma_fini_callback(void *data)
{
	struct completion *dma_complete = data;

	complete(dma_complete);
}

static int spi_sirfsoc_transfer(struct spi_device *spi, struct spi_transfer *t)
{
	struct sirfsoc_spi *sspi;
	int timeout = t->len * 10;
	sspi = spi_master_get_devdata(spi->master);

	sspi->tx = t->tx_buf ? t->tx_buf : sspi->dummypage;
	sspi->rx = t->rx_buf ? t->rx_buf : sspi->dummypage;
	sspi->left_tx_word = sspi->left_rx_word = t->len / sspi->word_width;
	reinit_completion(&sspi->rx_done);
	reinit_completion(&sspi->tx_done);

	writel(SIRFSOC_SPI_INT_MASK_ALL, sspi->base + SIRFSOC_SPI_INT_STATUS);

	/*
	 * fill tx_buf into command register and wait for its completion
	 */
	if (sspi->tx_by_cmd) {
		u32 cmd;
		memcpy(&cmd, sspi->tx, t->len);

		if (sspi->word_width == 1 && !(spi->mode & SPI_LSB_FIRST))
			cmd = cpu_to_be32(cmd) >>
				((SIRFSOC_MAX_CMD_BYTES - t->len) * 8);
		if (sspi->word_width == 2 && t->len == 4 &&
				(!(spi->mode & SPI_LSB_FIRST)))
			cmd = ((cmd & 0xffff) << 16) | (cmd >> 16);

		writel(cmd, sspi->base + SIRFSOC_SPI_CMD);
		writel(SIRFSOC_SPI_FRM_END_INT_EN,
			sspi->base + SIRFSOC_SPI_INT_EN);
		writel(SIRFSOC_SPI_CMD_TX_EN,
			sspi->base + SIRFSOC_SPI_TX_RX_EN);

		if (wait_for_completion_timeout(&sspi->tx_done, timeout) == 0) {
			dev_err(&spi->dev, "transfer timeout\n");
			return 0;
		}

		return t->len;
	}

	if (sspi->left_tx_word == 1) {
		writel(readl(sspi->base + SIRFSOC_SPI_CTRL) |
			SIRFSOC_SPI_ENA_AUTO_CLR,
			sspi->base + SIRFSOC_SPI_CTRL);
		writel(0, sspi->base + SIRFSOC_SPI_TX_DMA_IO_LEN);
		writel(0, sspi->base + SIRFSOC_SPI_RX_DMA_IO_LEN);
	} else if ((sspi->left_tx_word > 1) && (sspi->left_tx_word <
				SIRFSOC_SPI_DAT_FRM_LEN_MAX)) {
		writel(readl(sspi->base + SIRFSOC_SPI_CTRL) |
				SIRFSOC_SPI_MUL_DAT_MODE |
				SIRFSOC_SPI_ENA_AUTO_CLR,
			sspi->base + SIRFSOC_SPI_CTRL);
		writel(sspi->left_tx_word - 1,
				sspi->base + SIRFSOC_SPI_TX_DMA_IO_LEN);
		writel(sspi->left_tx_word - 1,
				sspi->base + SIRFSOC_SPI_RX_DMA_IO_LEN);
	} else {
		writel(readl(sspi->base + SIRFSOC_SPI_CTRL),
			sspi->base + SIRFSOC_SPI_CTRL);
		writel(0, sspi->base + SIRFSOC_SPI_TX_DMA_IO_LEN);
		writel(0, sspi->base + SIRFSOC_SPI_RX_DMA_IO_LEN);
	}

	writel(SIRFSOC_SPI_FIFO_RESET, sspi->base + SIRFSOC_SPI_RXFIFO_OP);
	writel(SIRFSOC_SPI_FIFO_RESET, sspi->base + SIRFSOC_SPI_TXFIFO_OP);
	writel(SIRFSOC_SPI_FIFO_START, sspi->base + SIRFSOC_SPI_RXFIFO_OP);
	writel(SIRFSOC_SPI_FIFO_START, sspi->base + SIRFSOC_SPI_TXFIFO_OP);

	if (IS_DMA_VALID(t)) {
		struct dma_async_tx_descriptor *rx_desc, *tx_desc;

		sspi->dst_start = dma_map_single(&spi->dev, sspi->rx, t->len, DMA_FROM_DEVICE);
		rx_desc = dmaengine_prep_slave_single(sspi->rx_chan,
			sspi->dst_start, t->len, DMA_DEV_TO_MEM,
			DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		rx_desc->callback = spi_sirfsoc_dma_fini_callback;
		rx_desc->callback_param = &sspi->rx_done;

		sspi->src_start = dma_map_single(&spi->dev, (void *)sspi->tx, t->len, DMA_TO_DEVICE);
		tx_desc = dmaengine_prep_slave_single(sspi->tx_chan,
			sspi->src_start, t->len, DMA_MEM_TO_DEV,
			DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		tx_desc->callback = spi_sirfsoc_dma_fini_callback;
		tx_desc->callback_param = &sspi->tx_done;

		dmaengine_submit(tx_desc);
		dmaengine_submit(rx_desc);
		dma_async_issue_pending(sspi->tx_chan);
		dma_async_issue_pending(sspi->rx_chan);
	} else {
		/* Send the first word to trigger the whole tx/rx process */
		sspi->tx_word(sspi);

		writel(SIRFSOC_SPI_RX_OFLOW_INT_EN | SIRFSOC_SPI_TX_UFLOW_INT_EN |
			SIRFSOC_SPI_RXFIFO_THD_INT_EN | SIRFSOC_SPI_TXFIFO_THD_INT_EN |
			SIRFSOC_SPI_FRM_END_INT_EN | SIRFSOC_SPI_RXFIFO_FULL_INT_EN |
			SIRFSOC_SPI_TXFIFO_EMPTY_INT_EN, sspi->base + SIRFSOC_SPI_INT_EN);
	}

	writel(SIRFSOC_SPI_RX_EN | SIRFSOC_SPI_TX_EN, sspi->base + SIRFSOC_SPI_TX_RX_EN);

	if (!IS_DMA_VALID(t)) { /* for PIO */
		if (wait_for_completion_timeout(&sspi->rx_done, timeout) == 0)
			dev_err(&spi->dev, "transfer timeout\n");
	} else if (wait_for_completion_timeout(&sspi->rx_done, timeout) == 0) {
		dev_err(&spi->dev, "transfer timeout\n");
		dmaengine_terminate_all(sspi->rx_chan);
	} else
		sspi->left_rx_word = 0;

	/*
	 * we only wait tx-done event if transferring by DMA. for PIO,
	 * we get rx data by writing tx data, so if rx is done, tx has
	 * done earlier
	 */
	if (IS_DMA_VALID(t)) {
		if (wait_for_completion_timeout(&sspi->tx_done, timeout) == 0) {
			dev_err(&spi->dev, "transfer timeout\n");
			dmaengine_terminate_all(sspi->tx_chan);
		}
	}

	if (IS_DMA_VALID(t)) {
		dma_unmap_single(&spi->dev, sspi->src_start, t->len, DMA_TO_DEVICE);
		dma_unmap_single(&spi->dev, sspi->dst_start, t->len, DMA_FROM_DEVICE);
	}

	/* TX, RX FIFO stop */
	writel(0, sspi->base + SIRFSOC_SPI_RXFIFO_OP);
	writel(0, sspi->base + SIRFSOC_SPI_TXFIFO_OP);
	writel(0, sspi->base + SIRFSOC_SPI_TX_RX_EN);
	writel(0, sspi->base + SIRFSOC_SPI_INT_EN);

	return t->len - sspi->left_rx_word * sspi->word_width;
}

static void spi_sirfsoc_chipselect(struct spi_device *spi, int value)
{
	struct sirfsoc_spi *sspi = spi_master_get_devdata(spi->master);

	if (sspi->chipselect[spi->chip_select] == 0) {
		u32 regval = readl(sspi->base + SIRFSOC_SPI_CTRL);
		switch (value) {
		case BITBANG_CS_ACTIVE:
			if (spi->mode & SPI_CS_HIGH)
				regval |= SIRFSOC_SPI_CS_IO_OUT;
			else
				regval &= ~SIRFSOC_SPI_CS_IO_OUT;
			break;
		case BITBANG_CS_INACTIVE:
			if (spi->mode & SPI_CS_HIGH)
				regval &= ~SIRFSOC_SPI_CS_IO_OUT;
			else
				regval |= SIRFSOC_SPI_CS_IO_OUT;
			break;
		}
		writel(regval, sspi->base + SIRFSOC_SPI_CTRL);
	} else {
		int gpio = sspi->chipselect[spi->chip_select];
		switch (value) {
		case BITBANG_CS_ACTIVE:
			gpio_direction_output(gpio,
					spi->mode & SPI_CS_HIGH ? 1 : 0);
			break;
		case BITBANG_CS_INACTIVE:
			gpio_direction_output(gpio,
					spi->mode & SPI_CS_HIGH ? 0 : 1);
			break;
		}
	}
}

static int
spi_sirfsoc_setup_transfer(struct spi_device *spi, struct spi_transfer *t)
{
	struct sirfsoc_spi *sspi;
	u8 bits_per_word = 0;
	int hz = 0;
	u32 regval;
	u32 txfifo_ctrl, rxfifo_ctrl;
	u32 fifo_size = SIRFSOC_SPI_FIFO_SIZE / 4;

	sspi = spi_master_get_devdata(spi->master);

	bits_per_word = (t) ? t->bits_per_word : spi->bits_per_word;
	hz = t && t->speed_hz ? t->speed_hz : spi->max_speed_hz;

	regval = (sspi->ctrl_freq / (2 * hz)) - 1;
	if (regval > 0xFFFF || regval < 0) {
		dev_err(&spi->dev, "Speed %d not supported\n", hz);
		return -EINVAL;
	}

	switch (bits_per_word) {
	case 8:
		regval |= SIRFSOC_SPI_TRAN_DAT_FORMAT_8;
		sspi->rx_word = spi_sirfsoc_rx_word_u8;
		sspi->tx_word = spi_sirfsoc_tx_word_u8;
		break;
	case 12:
	case 16:
		regval |= (bits_per_word ==  12) ? SIRFSOC_SPI_TRAN_DAT_FORMAT_12 :
			SIRFSOC_SPI_TRAN_DAT_FORMAT_16;
		sspi->rx_word = spi_sirfsoc_rx_word_u16;
		sspi->tx_word = spi_sirfsoc_tx_word_u16;
		break;
	case 32:
		regval |= SIRFSOC_SPI_TRAN_DAT_FORMAT_32;
		sspi->rx_word = spi_sirfsoc_rx_word_u32;
		sspi->tx_word = spi_sirfsoc_tx_word_u32;
		break;
	default:
		BUG();
	}

	sspi->word_width = DIV_ROUND_UP(bits_per_word, 8);
	txfifo_ctrl = SIRFSOC_SPI_FIFO_THD(SIRFSOC_SPI_FIFO_SIZE / 2) |
					   sspi->word_width;
	rxfifo_ctrl = SIRFSOC_SPI_FIFO_THD(SIRFSOC_SPI_FIFO_SIZE / 2) |
					   sspi->word_width;

	if (!(spi->mode & SPI_CS_HIGH))
		regval |= SIRFSOC_SPI_CS_IDLE_STAT;
	if (!(spi->mode & SPI_LSB_FIRST))
		regval |= SIRFSOC_SPI_TRAN_MSB;
	if (spi->mode & SPI_CPOL)
		regval |= SIRFSOC_SPI_CLK_IDLE_STAT;

	/*
	 * Data should be driven at least 1/2 cycle before the fetch edge to make
	 * sure that data gets stable at the fetch edge.
	 */
	if (((spi->mode & SPI_CPOL) && (spi->mode & SPI_CPHA)) ||
	    (!(spi->mode & SPI_CPOL) && !(spi->mode & SPI_CPHA)))
		regval &= ~SIRFSOC_SPI_DRV_POS_EDGE;
	else
		regval |= SIRFSOC_SPI_DRV_POS_EDGE;

	writel(SIRFSOC_SPI_FIFO_SC(fifo_size - 2) |
			SIRFSOC_SPI_FIFO_LC(fifo_size / 2) |
			SIRFSOC_SPI_FIFO_HC(2),
		sspi->base + SIRFSOC_SPI_TXFIFO_LEVEL_CHK);
	writel(SIRFSOC_SPI_FIFO_SC(2) |
			SIRFSOC_SPI_FIFO_LC(fifo_size / 2) |
			SIRFSOC_SPI_FIFO_HC(fifo_size - 2),
		sspi->base + SIRFSOC_SPI_RXFIFO_LEVEL_CHK);
	writel(txfifo_ctrl, sspi->base + SIRFSOC_SPI_TXFIFO_CTRL);
	writel(rxfifo_ctrl, sspi->base + SIRFSOC_SPI_RXFIFO_CTRL);

	if (t && t->tx_buf && !t->rx_buf && (t->len <= SIRFSOC_MAX_CMD_BYTES)) {
		regval |= (SIRFSOC_SPI_CMD_BYTE_NUM((t->len - 1)) |
				SIRFSOC_SPI_CMD_MODE);
		sspi->tx_by_cmd = true;
	} else {
		regval &= ~SIRFSOC_SPI_CMD_MODE;
		sspi->tx_by_cmd = false;
	}
	/*
	 * set spi controller in RISC chipselect mode, we are controlling CS by
	 * software BITBANG_CS_ACTIVE and BITBANG_CS_INACTIVE.
	 */
	regval |= SIRFSOC_SPI_CS_IO_MODE;
	writel(regval, sspi->base + SIRFSOC_SPI_CTRL);

	if (IS_DMA_VALID(t)) {
		/* Enable DMA mode for RX, TX */
		writel(0, sspi->base + SIRFSOC_SPI_TX_DMA_IO_CTRL);
		writel(SIRFSOC_SPI_RX_DMA_FLUSH, sspi->base + SIRFSOC_SPI_RX_DMA_IO_CTRL);
	} else {
		/* Enable IO mode for RX, TX */
		writel(SIRFSOC_SPI_IO_MODE_SEL, sspi->base + SIRFSOC_SPI_TX_DMA_IO_CTRL);
		writel(SIRFSOC_SPI_IO_MODE_SEL, sspi->base + SIRFSOC_SPI_RX_DMA_IO_CTRL);
	}

	return 0;
}

static int spi_sirfsoc_setup(struct spi_device *spi)
{
	if (!spi->max_speed_hz)
		return -EINVAL;

	return spi_sirfsoc_setup_transfer(spi, NULL);
}

static int spi_sirfsoc_probe(struct platform_device *pdev)
{
	struct sirfsoc_spi *sspi;
	struct spi_master *master;
	struct resource *mem_res;
	int num_cs, cs_gpio, irq;
	int i;
	int ret;

	ret = of_property_read_u32(pdev->dev.of_node,
			"sirf,spi-num-chipselects", &num_cs);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to get chip select number\n");
		goto err_cs;
	}

	master = spi_alloc_master(&pdev->dev, sizeof(*sspi) + sizeof(int) * num_cs);
	if (!master) {
		dev_err(&pdev->dev, "Unable to allocate SPI master\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, master);
	sspi = spi_master_get_devdata(master);

	master->num_chipselect = num_cs;

	for (i = 0; i < master->num_chipselect; i++) {
		cs_gpio = of_get_named_gpio(pdev->dev.of_node, "cs-gpios", i);
		if (cs_gpio < 0) {
			dev_err(&pdev->dev, "can't get cs gpio from DT\n");
			ret = -ENODEV;
			goto free_master;
		}

		sspi->chipselect[i] = cs_gpio;
		if (cs_gpio == 0)
			continue; /* use cs from spi controller */

		ret = gpio_request(cs_gpio, DRIVER_NAME);
		if (ret) {
			while (i > 0) {
				i--;
				if (sspi->chipselect[i] > 0)
					gpio_free(sspi->chipselect[i]);
			}
			dev_err(&pdev->dev, "fail to request cs gpios\n");
			goto free_master;
		}
	}

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sspi->base = devm_ioremap_resource(&pdev->dev, mem_res);
	if (IS_ERR(sspi->base)) {
		ret = PTR_ERR(sspi->base);
		goto free_master;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = -ENXIO;
		goto free_master;
	}
	ret = devm_request_irq(&pdev->dev, irq, spi_sirfsoc_irq, 0,
				DRIVER_NAME, sspi);
	if (ret)
		goto free_master;

	sspi->bitbang.master = master;
	sspi->bitbang.chipselect = spi_sirfsoc_chipselect;
	sspi->bitbang.setup_transfer = spi_sirfsoc_setup_transfer;
	sspi->bitbang.txrx_bufs = spi_sirfsoc_transfer;
	sspi->bitbang.master->setup = spi_sirfsoc_setup;
	master->bus_num = pdev->id;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST | SPI_CS_HIGH;
	master->bits_per_word_mask = SPI_BPW_MASK(8) | SPI_BPW_MASK(12) |
					SPI_BPW_MASK(16) | SPI_BPW_MASK(32);
	sspi->bitbang.master->dev.of_node = pdev->dev.of_node;

	/* request DMA channels */
	sspi->rx_chan = dma_request_slave_channel(&pdev->dev, "rx");
	if (!sspi->rx_chan) {
		dev_err(&pdev->dev, "can not allocate rx dma channel\n");
		ret = -ENODEV;
		goto free_master;
	}
	sspi->tx_chan = dma_request_slave_channel(&pdev->dev, "tx");
	if (!sspi->tx_chan) {
		dev_err(&pdev->dev, "can not allocate tx dma channel\n");
		ret = -ENODEV;
		goto free_rx_dma;
	}

	sspi->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(sspi->clk)) {
		ret = PTR_ERR(sspi->clk);
		goto free_tx_dma;
	}
	clk_prepare_enable(sspi->clk);
	sspi->ctrl_freq = clk_get_rate(sspi->clk);

	init_completion(&sspi->rx_done);
	init_completion(&sspi->tx_done);

	writel(SIRFSOC_SPI_FIFO_RESET, sspi->base + SIRFSOC_SPI_RXFIFO_OP);
	writel(SIRFSOC_SPI_FIFO_RESET, sspi->base + SIRFSOC_SPI_TXFIFO_OP);
	writel(SIRFSOC_SPI_FIFO_START, sspi->base + SIRFSOC_SPI_RXFIFO_OP);
	writel(SIRFSOC_SPI_FIFO_START, sspi->base + SIRFSOC_SPI_TXFIFO_OP);
	/* We are not using dummy delay between command and data */
	writel(0, sspi->base + SIRFSOC_SPI_DUMMY_DELAY_CTL);

	sspi->dummypage = kmalloc(2 * PAGE_SIZE, GFP_KERNEL);
	if (!sspi->dummypage) {
		ret = -ENOMEM;
		goto free_clk;
	}

	ret = spi_bitbang_start(&sspi->bitbang);
	if (ret)
		goto free_dummypage;

	dev_info(&pdev->dev, "registerred, bus number = %d\n", master->bus_num);

	return 0;
free_dummypage:
	kfree(sspi->dummypage);
free_clk:
	clk_disable_unprepare(sspi->clk);
	clk_put(sspi->clk);
free_tx_dma:
	dma_release_channel(sspi->tx_chan);
free_rx_dma:
	dma_release_channel(sspi->rx_chan);
free_master:
	spi_master_put(master);
err_cs:
	return ret;
}

static int  spi_sirfsoc_remove(struct platform_device *pdev)
{
	struct spi_master *master;
	struct sirfsoc_spi *sspi;
	int i;

	master = platform_get_drvdata(pdev);
	sspi = spi_master_get_devdata(master);

	spi_bitbang_stop(&sspi->bitbang);
	for (i = 0; i < master->num_chipselect; i++) {
		if (sspi->chipselect[i] > 0)
			gpio_free(sspi->chipselect[i]);
	}
	kfree(sspi->dummypage);
	clk_disable_unprepare(sspi->clk);
	clk_put(sspi->clk);
	dma_release_channel(sspi->rx_chan);
	dma_release_channel(sspi->tx_chan);
	spi_master_put(master);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int spi_sirfsoc_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct sirfsoc_spi *sspi = spi_master_get_devdata(master);
	int ret;

	ret = spi_master_suspend(master);
	if (ret)
		return ret;

	clk_disable(sspi->clk);
	return 0;
}

static int spi_sirfsoc_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct sirfsoc_spi *sspi = spi_master_get_devdata(master);

	clk_enable(sspi->clk);
	writel(SIRFSOC_SPI_FIFO_RESET, sspi->base + SIRFSOC_SPI_RXFIFO_OP);
	writel(SIRFSOC_SPI_FIFO_RESET, sspi->base + SIRFSOC_SPI_TXFIFO_OP);
	writel(SIRFSOC_SPI_FIFO_START, sspi->base + SIRFSOC_SPI_RXFIFO_OP);
	writel(SIRFSOC_SPI_FIFO_START, sspi->base + SIRFSOC_SPI_TXFIFO_OP);

	return spi_master_resume(master);
}
#endif

static SIMPLE_DEV_PM_OPS(spi_sirfsoc_pm_ops, spi_sirfsoc_suspend,
			 spi_sirfsoc_resume);

static const struct of_device_id spi_sirfsoc_of_match[] = {
	{ .compatible = "sirf,prima2-spi", },
	{ .compatible = "sirf,marco-spi", },
	{}
};
MODULE_DEVICE_TABLE(of, spi_sirfsoc_of_match);

static struct platform_driver spi_sirfsoc_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm     = &spi_sirfsoc_pm_ops,
		.of_match_table = spi_sirfsoc_of_match,
	},
	.probe = spi_sirfsoc_probe,
	.remove = spi_sirfsoc_remove,
};
module_platform_driver(spi_sirfsoc_driver);

MODULE_DESCRIPTION("SiRF SoC SPI master driver");
MODULE_AUTHOR("Zhiwu Song <Zhiwu.Song@csr.com>, "
		"Barry Song <Baohua.Song@csr.com>");
MODULE_LICENSE("GPL v2");
