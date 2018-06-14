/**
 * drivers/serial/sc16is7x2.c
 *
 * Copyright (C) 2009 Manuel Stahl <manuel.stahl@iis.fraunhofer.de>
 * Copyright (C) 2013 Evgeny Boger <boger@contactless.ru>
 * Copyright (C) 2015 Plyaskin Stepan <strelok@e-kirov.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The SC16IS7x2 device is a SPI driven dual UART with GPIOs.
 *
 * The driver exports two uarts and a gpiochip interface.
 */

//#define DEBUG

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/serial_reg.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/tty_flip.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/freezer.h>
#include <linux/platform_data/sc16is7x2.h>
#include <linux/of_device.h>
#include <linux/console.h>


#define MAX_SC16IS7X2		8
#define FIFO_SIZE		64

#define DRIVER_NAME		"sc16is7x2"
#define TYPE_NAME		"SC16IS7x2"


#define REG_READ	0x80
#define REG_WRITE	0x00

/* Special registers */
#define REG_SPR	    0x07	/* Test character register */
#define REG_TXLVL	0x08	/* Transmitter FIFO Level register */
#define REG_RXLVL	0x09	/* Receiver FIFO Level register */
#define REG_IOD		0x0A	/* IO Direction register */
#define REG_IOS		0x0B	/* IO State register */
#define REG_IOI		0x0C	/* IO Interrupt Enable register */
#define REG_IOC		0x0E	/* IO Control register */
#define REG_EFCR	0x0F	/* Extra Features Control Register */


#define IOC_SRESET	0x08    /* Software reset */
#define IOC_GPIO30	0x04    /* GPIO 3:0 unset: as IO, set: as modem pins */
#define IOC_GPIO74	0x02    /* GPIO 7:4 unset: as IO, set: as modem pins */
#define IOC_IOLATCH	0x01    /* Unset: input unlatched, set: input latched */

#define EFCR_RTSINVER 0x20  /* Invert not-RTS signal in RS-485 mode */
#define EFCR_RTSCON   0x10  /* Auto RS-485 mode: Enable the transmitter to control the RTS pin. */


struct sc16is7x2_chip;

/*
 *
 *
 */
struct sc16is7x2_async_spi {
	void (*complete) (void*, struct sc16is7x2_async_spi*);
	void * context;
	bool	persistent;
	bool	exec;
	struct spi_message	m;
	struct spi_transfer	t[2];
	u8	*tx_buf;
	u8	*rx_buf;
};

/*
 * Some registers must be read back to modify.
 * To save time we cache them here in memory.
 */
struct sc16is7x2_channel {
	struct sc16is7x2_chip	*chip;	/* back link */
	struct uart_port	uart;

	/* Workqueue that does all the magic */
	struct workqueue_struct *workqueue;
	struct work_struct	work;

	struct sc16is7x2_async_spi as_rx;
	u8		as_rx_buf[FIFO_SIZE+1];

	struct sc16is7x2_async_spi as_tx;
	u8		as_tx_buf[(FIFO_SIZE>>1)+1];

	/* channel initiallited and works */
	bool	started;
	/* true if IIR reading async rught now */
	bool	iir_reading;
	/* true if rx buffer reading async rught now */
	bool	rx_reading;
	/* true if tx buffer sending async rught now */
	bool	tx_writing;
	/* uart have unhandled chars */
	bool	tx_buffer_wait;


	//bool	irq_handle;

	u16		quot;		/* baud rate divisor */
	u8		iir;		/* state of IIR register */
	u8		lsr;		/* state of LSR register */
	u8		msr;		/* state of MSR register */
	u8		ier;		/* cache for IER register */
	u8		fcr;		/* cache for FCR register */
	u8		lcr;		/* cache for LCR register */
	u8		mcr;		/* cache for MCR register */
	u8		efr;		/* cache for EFR register */
	u8      efcr;       /* cache for EFCR register */

	u8		rxlvl;		/* last readed value of rxlvl */
	u8		txlvl;		/* last readed value of txlvl */

#ifdef DEBUG
	bool		handle_irq;
#endif
	bool		handle_baud;	/* baud rate needs update */
	bool		handle_regs;	/* other regs need update */

	bool use_modem_pins_by_default;
	bool console_enabled;
};

struct sc16is7x2_chip {
	spinlock_t		spi_lock;

	struct spi_device *spi;
	struct sc16is7x2_channel channel[2];

	unsigned int	uartclk;
	/* uart line number of the first channel */
	unsigned	uart_base;
	/* number assigned to the first GPIO */
	unsigned	gpio_base;

	bool request_works;

	char		*gpio_label;
	/* list of GPIO names (array length = SC16IS7X2_NR_GPIOS) */
	const char	*const *gpio_names;


#ifdef CONFIG_GPIOLIB
	struct gpio_chip gpio;
	struct mutex	io_lock;	/* lock for GPIO functions */
	u8		io_dir;		/* cache for IODir register */
	u8		io_state;	/* cache for IOState register */
	u8		io_gpio;	/* PIN is GPIO */
	u8		io_control;	/* cache for IOControl register */
#endif
};

static struct sc16is7x2_channel *sc16is7x2_channels[MAX_SC16IS7X2];


/* ***************************** REG NAMES ****************************** */
static char* sc16is7x2_reg_name(u8 reg){
	char *reg_names[] = {"RHR|THR", "IER", "FCR|IIR", "LCR",
						 "MCR", "LSR", "MSR|TCR", "SPR|TLR",
						 "TXLVL", "RXLVL", "IODir", "IOState",
						 "IOIntEna", "reserved", "IOControl", "EFCR"};
	return reg_names[reg & 15];
}
/* ******************************** SPI ********************************* */

static inline u8 write_cmd(u8 reg, u8 ch)
{
	return REG_WRITE | (reg & 0xf) << 3 | (ch & 0x1) << 1;
}

static inline u8 read_cmd(u8 reg, u8 ch)
{
	return REG_READ  | (reg & 0xf) << 3 | (ch & 0x1) << 1;
}

/*
 * sc16is7x2_write - Write a new register content (sync)
 * @reg: Register offset
 * @ch:  Channel (0 or 1)
 */
static int sc16is7x2_write(struct sc16is7x2_chip *ts, u8 reg, u8 ch, u8 val)
{
	u8 out[2];
	dev_dbg(&ts->spi->dev, "%s ch%i: reg %s cmd hex %02x%02x \n", __func__, ch, sc16is7x2_reg_name(reg), write_cmd(reg, ch), val);

	out[0] = write_cmd(reg, ch);
	out[1] = val;


	return spi_write(ts->spi, out, sizeof(out));
}

/**
 * sc16is7x2_read - Read back register content
 * @reg: Register offset
 * @ch:  Channel (0 or 1)
 *
 * Returns positive 8 bit value from the device if successful or a
 * negative value on error
 */
static int sc16is7x2_read(struct sc16is7x2_chip *ts, unsigned reg, unsigned ch)
{
	dev_dbg(&ts->spi->dev, "%s ch%i: reg %s cmd %02x \n", __func__, ch, sc16is7x2_reg_name(reg), read_cmd(reg, ch));
	return spi_w8r8(ts->spi, read_cmd(reg, ch));
}

/* ******************************** IRQ ********************************* */

static void sc16is7x2_handle_baud(struct sc16is7x2_chip *ts, unsigned ch)
{
	struct sc16is7x2_channel *chan = &ts->channel[ch];

	if (!chan->handle_baud)
		return;

	dev_dbg(&ts->spi->dev, "%s\n", __func__);

	sc16is7x2_write(ts, UART_IER, ch, 0);
	sc16is7x2_write(ts, UART_LCR, ch, UART_LCR_DLAB); /* access DLL&DLM */
	sc16is7x2_write(ts, UART_DLL, ch, chan->quot & 0xff);
	sc16is7x2_write(ts, UART_DLM, ch, chan->quot >> 8);
	sc16is7x2_write(ts, UART_LCR, ch, chan->lcr);     /* reset DLAB */
	sc16is7x2_write(ts, UART_IER, ch, chan->ier);

	chan->handle_baud = false;
}

static void sc16is7x2_handle_regs(struct sc16is7x2_chip *ts, unsigned ch)
{
	struct sc16is7x2_channel *chan = &ts->channel[ch];

	if (!chan->handle_regs)
		return;

	dev_dbg(&ts->spi->dev, "%s\n", __func__);

	sc16is7x2_write(ts, UART_LCR, ch, 0xBF);  /* access EFR */
	sc16is7x2_write(ts, UART_EFR, ch, chan->efr);
	sc16is7x2_write(ts, UART_LCR, ch, chan->lcr);
	sc16is7x2_write(ts, UART_FCR, ch, chan->fcr);
	sc16is7x2_write(ts, UART_MCR, ch, chan->mcr);
	sc16is7x2_write(ts, UART_IER, ch, chan->ier);

	chan->handle_regs = false;
}

static void sc16is7x2_read_status(struct sc16is7x2_chip *ts, unsigned ch)
{
	struct sc16is7x2_channel *chan = &(ts->channel[ch]);
	u8 ier;

	ier = sc16is7x2_read(ts, UART_IER, ch);


	chan->iir = sc16is7x2_read(ts, UART_IIR, ch);
	chan->msr = sc16is7x2_read(ts, UART_MSR, ch);
	chan->lsr = sc16is7x2_read(ts, UART_LSR, ch);

	dev_err(&ts->spi->dev, " %s ier=0x%02x iir=0x%02x msr=0x%02x lsr=0x%02x\n",
			__func__, ier, chan->iir, chan->msr, chan->lsr);

}

static void sc16is7x2_handle_channel(struct work_struct *w)
{

	struct sc16is7x2_channel *chan =
			container_of(w, struct sc16is7x2_channel, work);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;

#ifdef DEBUG
	dev_dbg(&ts->spi->dev, "%s (%i) %s\n", __func__, ch,
			chan->handle_irq ? "irq" : "");
	chan->handle_irq = false;
#endif
	do {
		sc16is7x2_handle_baud(ts, ch);
		sc16is7x2_handle_regs(ts, ch);
		sc16is7x2_read_status(ts, ch); // read all registers at once
	} while (!(chan->iir & UART_IIR_NO_INT));
}


/* Trigger work thread*/
static void sc16is7x2_dowork(struct sc16is7x2_channel *chan)
{
	if (!freezing(current)) {
		queue_work(chan->workqueue, &chan->work);
	}
}

/* ***************************** SPI ASYNC ****************************** */

static inline void
sc16is7x2_async_spi_free(struct sc16is7x2_async_spi *as)
{
	kfree(as);
}

static inline bool
sc16is7x2_async_spi_tryfree(struct sc16is7x2_async_spi *as)
{
	if(as->persistent || as->exec){
		return false;
	}
	sc16is7x2_async_spi_free(as);
	return true;
}

static void sc16is7x2_async_spi_complete(void *data)
{
	struct sc16is7x2_async_spi *as = data;

	if(*as->complete){
		as->complete(as->context, as);
	}
	as->exec = false;

	sc16is7x2_async_spi_tryfree(as);
}

static inline void
sc16is7x2_async_spi_set_tx_len(struct sc16is7x2_async_spi *as, u8 len)
{
	as->t[0].len = len;
}

static inline void
sc16is7x2_async_spi_set_rx_len(struct sc16is7x2_async_spi *as, u8 len)
{
	as->t[1].len = len;
}

static struct sc16is7x2_async_spi*
sc16is7x2_async_spi_init(struct sc16is7x2_async_spi *as, u8 tx_len, u8 rx_len)
{
	spi_message_init(&as->m);

	if(tx_len > 0){
		as->t[0].tx_buf  = as->tx_buf;
		as->t[0].len     = tx_len;
		spi_message_add_tail(&as->t[0], &as->m);
	}
	if(rx_len > 0){
		as->t[1].rx_buf  = as->rx_buf;
		as->t[1].len     = rx_len;
		spi_message_add_tail(&as->t[1], &as->m);
	}

	as->m.complete = sc16is7x2_async_spi_complete;
	as->m.context  = as;

	return as;
}

static struct sc16is7x2_async_spi*
sc16is7x2_async_spi_alloc_and_init(u8 tx_len, u8 rx_len)
{
	struct sc16is7x2_async_spi *as = kzalloc(sizeof(struct sc16is7x2_async_spi)
										+ tx_len + rx_len, GFP_ATOMIC);

	if(!as){
		return NULL;
	}

	as->tx_buf = (u8*)as;
	as->tx_buf += sizeof(struct sc16is7x2_async_spi);
	as->rx_buf = (u8*)as;
	as->rx_buf += sizeof(struct sc16is7x2_async_spi) + tx_len;

	return sc16is7x2_async_spi_init(as, tx_len, rx_len);
}

static inline int
sc16is7x2_async_spi_run(struct spi_device *spi, struct sc16is7x2_async_spi *as)
{
	as->exec = true;
	return spi_async(spi, &as->m);
}

/**
 * sc16is7x2_read_reg_val - read reg value from sc16is7x2_async_spi
 *
 * @as - sc16is7x2_async_spi in complete function
 * Just helper
 */
static inline u8 sc16is7x2_read_reg_val(struct sc16is7x2_async_spi *as)
{
	return as->rx_buf[0];
}

static void
sc16is7x2_read_reg_async(struct sc16is7x2_channel *chan, u8 reg,
							struct sc16is7x2_async_spi *as_persist,
							void (*complete)(void*, struct sc16is7x2_async_spi*),
							void *context)
{
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;
	struct sc16is7x2_async_spi *as;

	if(as_persist == NULL){
		as = sc16is7x2_async_spi_alloc_and_init(1, 1);
	} else {
		as = as_persist;
	}
	if(!as){
		dev_err(&ts->spi->dev, "%s ch%d: ENOMEM", __func__, ch);
		return;
	}

	as->tx_buf[0] = read_cmd(reg, ch);
	as->complete = complete;
	as->context = context;

	dev_dbg(&ts->spi->dev, "%s ch%d: reading %s\n", __func__, ch, sc16is7x2_reg_name(reg));

	sc16is7x2_async_spi_run(ts->spi, as);
}

static void
sc16is7x2_write_reg_async(struct sc16is7x2_channel *chan,
							u8 reg, u8 tx,
							struct sc16is7x2_async_spi *as_persist,
							void (*complete)(void*, struct sc16is7x2_async_spi*),
							void *context)
{
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;
	struct sc16is7x2_async_spi *as;

	if(as_persist == NULL){
		as = sc16is7x2_async_spi_alloc_and_init(2, 0);
	} else {
		as = as_persist;
	}
	if(!as){
		dev_err(&ts->spi->dev, "%s ch%d: ENOMEM", __func__, ch);
		return;
	}

	as->tx_buf[0] = write_cmd(reg, ch);
	as->tx_buf[1] = tx;
	as->complete = complete;
	as->context = context;

	dev_dbg(&ts->spi->dev, "%s ch%d: writing %s", __func__, ch, sc16is7x2_reg_name(reg));

	sc16is7x2_async_spi_run(ts->spi, as);
}

static void
sc16is7x2_read_rx_async(struct sc16is7x2_channel *chan, u8 rxlvl);

static void
sc16is7x2_read_irq_complete(void *data, struct sc16is7x2_async_spi *as);

static void sc16is7x2_write_tx_async(struct sc16is7x2_channel *chan);

static void
sc16is7x2_read_status_lsr(void *data, struct sc16is7x2_async_spi *as)
{
	struct sc16is7x2_channel *chan = data;
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;

	chan->lsr = sc16is7x2_read_reg_val(as);
	dev_dbg(&ts->spi->dev, "%s ch%d: %02x", __func__, ch, sc16is7x2_read_reg_val(as));
}

static void
sc16is7x2_read_rxlvl_complete(void *data, struct sc16is7x2_async_spi *as)
{
	struct sc16is7x2_channel *chan = data;
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;

	chan->rxlvl = sc16is7x2_read_reg_val(as);
	dev_dbg(&ts->spi->dev, "%s ch%d: %d", __func__, ch, sc16is7x2_read_reg_val(as));
	if(chan->rxlvl > 0){
		chan->rx_reading = true;
		// turn off RDI interrupt
		if(chan->ier & UART_IER_RDI){
			chan->ier &= ~UART_IER_RDI;
			chan->ier &= ~UART_IER_RLSI;
			sc16is7x2_write_reg_async(chan, UART_IER, chan->ier, NULL, NULL, NULL);
		}
		sc16is7x2_read_rx_async(chan, chan->rxlvl);
	} else {
		// turn on RHI interrupt
		if(!(chan->ier & UART_IER_RDI)){
			chan->ier |= UART_IER_RDI;
			chan->ier |= UART_IER_RLSI;
			sc16is7x2_write_reg_async(chan, UART_IER, chan->ier, NULL, NULL, NULL);
		}
		chan->rx_reading = false;
	}
}

static void sc16is7x2_read_rxlvl_async(struct sc16is7x2_channel *chan)
{
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;

	if(!chan->rx_reading){
		chan->rx_reading = true;
		sc16is7x2_read_reg_async(chan, REG_RXLVL, NULL,
									sc16is7x2_read_rxlvl_complete, chan);
	} else {
		dev_dbg(&ts->spi->dev, "%s ch%d: already", __func__, ch);
	}
}

static void
sc16is7x2_read_txlvl_complete(void *data, struct sc16is7x2_async_spi *as)
{
	struct sc16is7x2_channel *chan = data;
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;

	chan->txlvl = sc16is7x2_read_reg_val(as);
	dev_dbg(&ts->spi->dev, "%s ch%d: %d", __func__, ch, sc16is7x2_read_reg_val(as));

	if(chan->tx_buffer_wait){
		if(chan->txlvl >= (FIFO_SIZE >> 1)){
			sc16is7x2_write_tx_async(chan);
		} else {
			if(!(chan->ier & UART_IER_THRI)){
				chan->ier |= UART_IER_THRI;
				sc16is7x2_write_reg_async(chan, UART_IER, chan->ier, NULL, NULL, NULL);
			}
		}
	} else {
		chan->tx_writing = false;
		if(chan->ier & UART_IER_THRI){
			chan->ier &= ~UART_IER_THRI;
			sc16is7x2_write_reg_async(chan, UART_IER, chan->ier, NULL, NULL, NULL);
		}
	}
}

static void sc16is7x2_read_txlvl_async(struct sc16is7x2_channel *chan)
{
	if(!chan->tx_writing){
		chan->tx_writing = true;
		sc16is7x2_read_reg_async(chan, REG_TXLVL, NULL,
									sc16is7x2_read_txlvl_complete, chan);
	}
}

/**
 * sc16is7x2_read_status_async - read IER, MSR, LSR, RXLVL, TXLVL, IIR. After - handle
 *
 */
static void sc16is7x2_read_status_async(struct sc16is7x2_channel *chan)
{
	sc16is7x2_read_rxlvl_async(chan);
	sc16is7x2_read_reg_async(chan, UART_IIR, NULL,
								sc16is7x2_read_irq_complete, chan);
}

/**
 * sc16is7x2_write_tx_complete - end of writing
 *
 */
static void sc16is7x2_write_tx_complete(void *data, struct sc16is7x2_async_spi *as){
	struct sc16is7x2_channel *chan = data;
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;

	dev_dbg(&ts->spi->dev, "%s ch%d", __func__, ch);

}

static void sc16is7x2_write_tx_async(struct sc16is7x2_channel *chan){
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;
	struct uart_port *uart = &chan->uart;
	struct circ_buf *xmit = &uart->state->xmit;
	//unsigned long flags;
	unsigned i, len;
	int chars_pending;

#ifdef DEBUG2
	u8 j, curByte, curHalfByte;
#endif

	dev_dbg(&ts->spi->dev, "%s ch%d", __func__, ch);


	if (chan->uart.x_char && chan->lsr & UART_LSR_THRE) {
		dev_dbg(&ts->spi->dev, " tx: x-char\n");
		sc16is7x2_write(ts, UART_TX, ch, uart->x_char);
		uart->icount.tx++;
		uart->x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(&chan->uart)){
		/* No data to send or TX is stopped */
		dev_dbg(&ts->spi->dev, "%s ch%d: No data to send or TX is stopped", __func__, ch);
		chan->tx_writing = false;
		chan->tx_buffer_wait = false;
		if(chan->ier & UART_IER_THRI){
				chan->ier &= ~UART_IER_THRI;
				sc16is7x2_write_reg_async(chan, UART_IER, chan->ier, NULL, NULL, NULL);
		}
		return;
	}

	chan->tx_writing = true;

	/* number of bytes to transfer to the fifo */
	chars_pending = (int)uart_circ_chars_pending(xmit);
	len = min(32, chars_pending);
	chan->tx_buffer_wait = len < chars_pending;

	dev_dbg(&ts->spi->dev, "%s ch%i: %d bytes will be sent\n", __func__, ch, len);

	/* fill buffer to send */
	//spin_lock_irqsave(&uart->lock, flags);
	for (i = 1; i <= len ; i++) {
		chan->as_tx.tx_buf[i] = xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
	}
	sc16is7x2_async_spi_set_tx_len(&chan->as_tx, len+1);
	uart->icount.tx += len;
	//spin_unlock_irqrestore(&uart->lock, flags);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(uart);

	/* send data */
	sc16is7x2_async_spi_run(ts->spi, &chan->as_tx);

	/* get TXLVL after sending */
	sc16is7x2_read_reg_async(chan, REG_TXLVL, NULL,
									sc16is7x2_read_txlvl_complete, chan);
}

/**
* sc16is7x2_read_rx_complete - handle readed data
*
* @data - current channel
*/
static void
sc16is7x2_read_rx_complete(void *data, struct sc16is7x2_async_spi *as){
	struct sc16is7x2_channel *chan = data;
	struct sc16is7x2_chip *ts = chan->chip;
	//unsigned ch = (chan == ts->channel) ? 0 : 1;	//unused

	struct uart_port *uart = &chan->uart;
	struct tty_port *tty = &uart->state->port;
	unsigned long flags;
	u8 rxlvl = chan->rxlvl;

	dev_dbg(&ts->spi->dev, "%s readed %d bytes", __func__, rxlvl);

	spin_lock_irqsave(&uart->lock, flags);

	/* Insert received data */
	tty_insert_flip_string(tty, (char *)as->rx_buf, rxlvl);
	/* Update RX counter */
	uart->icount.rx += rxlvl;

	spin_unlock_irqrestore(&uart->lock, flags);

	tty_flip_buffer_push(tty);
}

/**
* sc16is7x2_read_rx_async - reading data from FIFO
*
* @data - current channel
*/
static void sc16is7x2_read_rx_async(struct sc16is7x2_channel *chan, u8 rxlvl){
	struct sc16is7x2_chip *ts = chan->chip;

	sc16is7x2_async_spi_set_rx_len(&chan->as_rx, rxlvl);
	sc16is7x2_async_spi_run(ts->spi, &chan->as_rx);

	sc16is7x2_read_reg_async(chan, REG_RXLVL, NULL,
								sc16is7x2_read_rxlvl_complete, chan);
}

static void
sc16is7x2_read_irq_complete(void *data, struct sc16is7x2_async_spi *as){
	struct sc16is7x2_channel *chan = data;

	u8 curiir = as->rx_buf[0];

#ifdef DEBUG
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (chan == ts->channel) ? 0 : 1;
#endif

	chan->iir = curiir;
	chan->iir_reading = false;

#ifdef DEBUG
	#define print_iir_if(iir)	case iir: \
			dev_dbg(&ts->spi->dev, "%s: ch%d: " #iir, __func__, ch); \
			break;

	switch(curiir & 0x3F){
		print_iir_if(UART_IIR_NO_INT);
		print_iir_if(UART_IIR_MSI);
		print_iir_if(UART_IIR_THRI);
		print_iir_if(UART_IIR_RDI);
		print_iir_if(UART_IIR_RLSI);
		print_iir_if(UART_IIR_BUSY);
		print_iir_if(UART_IIR_RX_TIMEOUT);
		print_iir_if(UART_IIR_XOFF);
		print_iir_if(UART_IIR_CTS_RTS_DSR);
		default:
			dev_dbg(&ts->spi->dev, "%s: ch%d: UNKNOWN", __func__, ch);
			break;
	}
#endif

	switch(curiir & 0x3F){
		case UART_IIR_THRI:
			sc16is7x2_write_tx_async(chan);
			break;
		case UART_IIR_RLSI:
			sc16is7x2_read_reg_async(chan, UART_LSR, NULL,
								sc16is7x2_read_status_lsr, chan);
			break;
		default: ;
	}

}

/**
* sc16is7x2_irq - handle irq
*
*/
static irqreturn_t sc16is7x2_irq(int irq, void *data){

	struct sc16is7x2_chip *ts = data;
	u8 i;

	for(i = 0; i < 2; ++i){
		if(ts->channel[i].started){
			if(ts->channel[i].iir_reading){
				dev_dbg(&ts->spi->dev, "%s ch%d: iir already reading", __func__, i);
			} else {
				ts->channel[i].iir_reading = true;
				sc16is7x2_read_status_async(&ts->channel[i]);
			}
		}
	}

	dev_dbg(&ts->spi->dev, "%s: IRQ_HANDLED", __func__);
	return IRQ_HANDLED;
}

/* ******************************** UART ********************************* */

#define to_sc16is7x2_channel(port) \
		container_of(port, struct sc16is7x2_channel, uart)


static unsigned int sc16is7x2_tx_empty(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned lsr;

	dev_dbg(&ts->spi->dev, "%s = %s\n", __func__,
			chan->lsr & UART_LSR_TEMT ? "yes" : "no");

	lsr = chan->lsr;
	return lsr & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
}

static unsigned int sc16is7x2_get_mctrl(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned int status;
	unsigned int ret;

	dev_dbg(&ts->spi->dev, "%s (0x%02x)\n", __func__, chan->msr);

	status = chan->msr;

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void sc16is7x2_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (&ts->channel[1] == chan) ? 1 : 0;

	dev_dbg(&ts->spi->dev, "%s (0x%02x)\n", __func__, mctrl);

	/* TODO: set DCD and DSR
	 * CTS/RTS is handled automatically
	 */
}

static void sc16is7x2_stop_tx(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;

	dev_dbg(&ts->spi->dev, "%s\n", __func__);
}

static void sc16is7x2_start_tx(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (&ts->channel[1] == chan) ? 1 : 0;

	dev_dbg(&ts->spi->dev, "%s ch%d\n", __func__, ch);

	chan->tx_buffer_wait = true;
	sc16is7x2_read_txlvl_async(chan);
}

static void sc16is7x2_stop_rx(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;

	dev_dbg(&ts->spi->dev, "%s\n", __func__);

	chan->ier &= ~UART_IER_RLSI;
	chan->ier &= ~UART_IER_RDI;
	chan->uart.read_status_mask &= ~UART_LSR_DR;

	sc16is7x2_write_reg_async(chan, UART_IER, chan->ier, NULL, NULL, NULL);

}

static void sc16is7x2_enable_ms(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;

	dev_dbg(&ts->spi->dev, "%s\n", __func__);

	chan->ier |= UART_IER_MSI;
	chan->handle_regs = true;
	/* Trigger work thread for doing the actual configuration change */
	sc16is7x2_dowork(chan);
}

static void sc16is7x2_break_ctl(struct uart_port *port, int break_state)
{
	/* We don't support break control yet, do nothing */
}

static int sc16is7x2_startup(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned ch = (&ts->channel[1] == chan) ? 1 : 0;
	unsigned long flags;

	//unsigned char test_char_r;
	//unsigned char test_char_w = 'H';
	dev_info(&ts->spi->dev, "\n%s (%d)\n", __func__, port->line);

	if (chan->workqueue) {
		dev_err(&ts->spi->dev, "\n%s (%d) duplicate startup, do nothing\n", __func__, port->line);
		return 0;
	}


	dev_dbg(&ts->spi->dev, "%s ch%d", __func__, port->line);


	sc16is7x2_write(ts, UART_LCR, ch, 0xBF);  /* access EFR */
	sc16is7x2_write(ts, UART_EFR, ch, 0x10);

	/* Clear the interrupt registers. */
	sc16is7x2_write(ts, UART_IER, ch, 0);
	sc16is7x2_read_status(ts, ch);

	/* Initialize work queue */
	chan->workqueue = create_freezable_workqueue("sc16is7x2");
	if (!chan->workqueue) {
		dev_err(&ts->spi->dev, "Workqueue creation failed\n");
		return -EBUSY;
	}
	INIT_WORK(&chan->work, sc16is7x2_handle_channel);

	spin_lock_irqsave(&chan->uart.lock, flags);
	chan->lcr = UART_LCR_WLEN8;
	chan->mcr = 0;
	chan->fcr = 0;
	chan->ier = UART_IER_RLSI | UART_IER_RDI;
	chan->efcr = 0;

	if (port->rs485.flags & SER_RS485_ENABLED) {
		chan->efcr |= EFCR_RTSCON;
	}

	if (port->rs485.flags & SER_RS485_RTS_ON_SEND) {
		chan->efcr |= EFCR_RTSINVER;
	}




	spin_unlock_irqrestore(&chan->uart.lock, flags);

	sc16is7x2_write(ts, UART_FCR, ch, UART_FCR_ENABLE_FIFO |
			   UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	sc16is7x2_write(ts, UART_FCR, ch, chan->fcr);
	/* Now, initialize the UART */
	sc16is7x2_write(ts, UART_LCR, ch, chan->lcr);
	sc16is7x2_write(ts, UART_MCR, ch, chan->mcr);
	sc16is7x2_write(ts, UART_IER, ch, chan->ier);

	sc16is7x2_write(ts, REG_EFCR, ch, chan->efcr);  // RS-485. FIXME: user generic interface

	chan->started = true;

	return 0;
}

static void sc16is7x2_shutdown(struct uart_port *port)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned long flags;
	unsigned ch = port->line & 0x01;

	dev_info(&ts->spi->dev, "%s ch%d\n", __func__, ch);

	if (chan->console_enabled) {
		// no need to actually shutdown port if console is enabled on this port
		return;
	}


	BUG_ON(!chan);
	BUG_ON(!ts);

	if (chan->workqueue) {
		/* Flush and destroy work queue */
		flush_workqueue(chan->workqueue);
		destroy_workqueue(chan->workqueue);
		chan->workqueue = NULL;
	}

	/* Suspend HW */
	spin_lock_irqsave(&chan->uart.lock, flags);
	chan->ier = UART_IERX_SLEEP;
	spin_unlock_irqrestore(&chan->uart.lock, flags);
	//sc16is7x2_write(ts, UART_IER, ch, chan->ier);
	sc16is7x2_write_reg_async(chan, UART_IER, chan->ier, NULL, NULL, NULL);

	chan->started = false;
}

static void
sc16is7x2_set_termios(struct uart_port *port, struct ktermios *termios,
			   struct ktermios *old)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;
	unsigned long flags;
	unsigned int baud;
	u8 lcr, fcr = 0;

	/* Ask the core to calculate the divisor for us. */
	baud = uart_get_baud_rate(port, termios, old,
				  port->uartclk / 16 / 0xffff,
				  port->uartclk / 16);
	chan->quot = uart_get_divisor(port, baud);
	chan->handle_baud = true;

	dev_info(&ts->spi->dev, "%s (baud %u)\n", __func__, baud);

	/* set word length */
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr = UART_LCR_WLEN5;
		break;
	case CS6:
		lcr = UART_LCR_WLEN6;
		break;
	case CS7:
		lcr = UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		lcr = UART_LCR_WLEN8;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		lcr |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB)
		lcr |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		lcr |= UART_LCR_EPAR;
#ifdef CMSPAR
	if (termios->c_cflag & CMSPAR)
		lcr |= UART_LCR_SPAR;
#endif

	fcr = UART_FCR_ENABLE_FIFO;
	/* configure the fifo */
	if (baud < 2400)
		fcr |= UART_FCR_TRIGGER_1;
	else
		fcr |= UART_FCR_R_TRIG_01 | UART_FCR_T_TRIG_10;

	chan->efr = UART_EFR_ECB;
	//~ chan->mcr |= UART_MCR_RTS;   //override bug in WB (force RTS to high by default)
	if (termios->c_cflag & CRTSCTS)
		chan->efr |= UART_EFR_CTS | UART_EFR_RTS;

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&chan->uart.lock, flags);

	//FIXME:
	/* we are sending char from a workqueue so enable */
	chan->uart.state->port.low_latency = 1;

	/* Update the per-port timeout. */
	uart_update_timeout(port, termios->c_cflag, baud);

	chan->uart.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		chan->uart.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		chan->uart.read_status_mask |= UART_LSR_BI;

	/* Characters to ignore */
	chan->uart.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		chan->uart.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		chan->uart.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			chan->uart.ignore_status_mask |= UART_LSR_OE;
	}

	/* ignore all characters if CREAD is not set */
	if ((termios->c_cflag & CREAD) == 0)
		chan->uart.ignore_status_mask |= UART_LSR_DR;

	/* CTS flow control flag and modem status interrupts */
	chan->ier &= ~UART_IER_MSI;
	if (UART_ENABLE_MS(&chan->uart, termios->c_cflag))
		chan->ier |= UART_IER_MSI;

	chan->lcr = lcr;	/* Save LCR */
	chan->fcr = fcr;	/* Save FCR */
	chan->handle_regs = true;

	spin_unlock_irqrestore(&chan->uart.lock, flags);

	/* Trigger work thread for doing the actual configuration change */
	sc16is7x2_dowork(chan);


}

static const char * sc16is7x2_type(struct uart_port *port)
{
	pr_debug("%s\n", __func__);
	return TYPE_NAME;
}

static void sc16is7x2_release_port(struct uart_port *port)
{
	pr_debug("%s\n", __func__);
}

static int sc16is7x2_request_port(struct uart_port *port)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static void sc16is7x2_config_port(struct uart_port *port, int flags)
{
	struct sc16is7x2_channel *chan = to_sc16is7x2_channel(port);
	struct sc16is7x2_chip *ts = chan->chip;

	dev_dbg(&ts->spi->dev, "%s\n", __func__);
	if (flags & UART_CONFIG_TYPE)
		chan->uart.type = PORT_SC16IS7X2;
}

static int
sc16is7x2_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->type == PORT_UNKNOWN || ser->type == PORT_SC16IS7X2)
		return 0;

	return -EINVAL;
}

static struct uart_ops sc16is7x2_uart_ops = {
	.tx_empty	= sc16is7x2_tx_empty,
	.set_mctrl	= sc16is7x2_set_mctrl,
	.get_mctrl	= sc16is7x2_get_mctrl,
	.stop_tx        = sc16is7x2_stop_tx,
	.start_tx	= sc16is7x2_start_tx,
	.stop_rx	= sc16is7x2_stop_rx,
	.enable_ms      = sc16is7x2_enable_ms,
	.break_ctl      = sc16is7x2_break_ctl,
	.startup	= sc16is7x2_startup,
	.shutdown	= sc16is7x2_shutdown,
	.set_termios	= sc16is7x2_set_termios,
	.type		= sc16is7x2_type,
	.release_port   = sc16is7x2_release_port,
	.request_port   = sc16is7x2_request_port,
	.config_port	= sc16is7x2_config_port,
	.verify_port	= sc16is7x2_verify_port,
};


/* ******************************** GPIO ********************************* */

#ifdef CONFIG_GPIOLIB

static int sc16is7x2_gpio_request(struct gpio_chip *gpio, unsigned offset)
{
	struct sc16is7x2_chip *ts =
			container_of(gpio, struct sc16is7x2_chip, gpio);
	int control = (offset < 4) ? IOC_GPIO30 : IOC_GPIO74;
	int ret = 0;

	BUG_ON(offset > 8);
	dev_dbg(&ts->spi->dev, "%s: offset = %d\n", __func__, offset);

	mutex_lock(&ts->io_lock);

	/* GPIO 0:3 and 4:7 can only be controlled as block */
	ts->io_gpio |= BIT(offset);
	if (ts->io_control & control) {
		dev_dbg(&ts->spi->dev, "activate GPIOs %s\n",
				(offset < 4) ? "0-3" : "4-7");
		ts->io_control &= ~control;
		ret = sc16is7x2_write(ts, REG_IOC, 0, ts->io_control);
	}

	mutex_unlock(&ts->io_lock);

	return ret;
}

static void sc16is7x2_gpio_free(struct gpio_chip *gpio, unsigned offset)
{
	struct sc16is7x2_chip *ts =
			container_of(gpio, struct sc16is7x2_chip, gpio);
	int control = (offset < 4) ? IOC_GPIO30 : IOC_GPIO74;
	int mask = (offset < 4) ? 0x0f : 0xf0;

	BUG_ON(offset > 8);

	mutex_lock(&ts->io_lock);

	/* GPIO 0:3 and 4:7 can only be controlled as block */
	ts->io_gpio &= ~BIT(offset);
	dev_dbg(&ts->spi->dev, "%s: io_gpio = 0x%02X\n", __func__, ts->io_gpio);
	if (!(ts->io_control & control) && !(ts->io_gpio & mask)) {
		if (ts->channel[offset < 4 ? 0 : 1].use_modem_pins_by_default) {

			dev_dbg(&ts->spi->dev, "deactivate GPIOs %s\n",
					(offset < 4) ? "0-3" : "4-7");
			ts->io_control |= control;
			sc16is7x2_write(ts, REG_IOC, 0, ts->io_control);
		}
	}

	mutex_unlock(&ts->io_lock);
}

static int sc16is7x2_direction_input(struct gpio_chip *gpio, unsigned offset)
{
	struct sc16is7x2_chip *ts =
			container_of(gpio, struct sc16is7x2_chip, gpio);
	unsigned io_dir;

	BUG_ON(offset > 8);

	mutex_lock(&ts->io_lock);

	ts->io_dir &= ~BIT(offset);
	io_dir = ts->io_dir;

	mutex_unlock(&ts->io_lock);

	return sc16is7x2_write(ts, REG_IOD, 0, io_dir);
}

static int sc16is7x2_direction_output(struct gpio_chip *gpio, unsigned offset,
					int value)
{
	struct sc16is7x2_chip *ts =
			container_of(gpio, struct sc16is7x2_chip, gpio);

	BUG_ON(offset > 8);

	mutex_lock(&ts->io_lock);

	if (value)
		ts->io_state |= BIT(offset);
	else
		ts->io_state &= ~BIT(offset);

	ts->io_dir |= BIT(offset);

	mutex_unlock(&ts->io_lock);

	sc16is7x2_write(ts, REG_IOS, 0, ts->io_state);
	return sc16is7x2_write(ts, REG_IOD, 0, ts->io_dir);
}

static int sc16is7x2_get(struct gpio_chip *gpio, unsigned offset)
{
	struct sc16is7x2_chip *ts =
			container_of(gpio, struct sc16is7x2_chip, gpio);
	int level = -EINVAL;

	BUG_ON(offset > 8);

	mutex_lock(&ts->io_lock);

	if (ts->io_dir & BIT(offset)) {
		/* Output: return cached level */
		level = (ts->io_state >> offset) & 0x01;
	} else {
		/* Input: read out all pins */
		level = sc16is7x2_read(ts, REG_IOS, 0);
		if (level >= 0) {
			ts->io_state = level;
			level = (ts->io_state >> offset) & 0x01;
		}
	}

	mutex_unlock(&ts->io_lock);

	return level;
}

static void sc16is7x2_set(struct gpio_chip *gpio, unsigned offset, int value)
{
	struct sc16is7x2_chip *ts =
			container_of(gpio, struct sc16is7x2_chip, gpio);
	unsigned io_state;

	BUG_ON(offset > 8);

	mutex_lock(&ts->io_lock);

	if (value)
		ts->io_state |= BIT(offset);
	else
		ts->io_state &= ~BIT(offset);
	io_state = ts->io_state;

	mutex_unlock(&ts->io_lock);

	sc16is7x2_write(ts, REG_IOS, 0, io_state);
}

#endif /* CONFIG_GPIOLIB */

/* ******************************** INIT ********************************* */

static struct uart_driver sc16is7x2_uart_driver;

static int sc16is7x2_register_gpio(struct sc16is7x2_chip *ts)
{
#ifdef CONFIG_GPIOLIB
	ts->gpio.label = (ts->gpio_label) ? ts->gpio_label : DRIVER_NAME;
	ts->gpio.request	= sc16is7x2_gpio_request;
	ts->gpio.free		= sc16is7x2_gpio_free;
	ts->gpio.get		= sc16is7x2_get;
	ts->gpio.set		= sc16is7x2_set;
	ts->gpio.direction_input = sc16is7x2_direction_input;
	ts->gpio.direction_output = sc16is7x2_direction_output;

	ts->gpio.base = ts->gpio_base;
	ts->gpio.names = ts->gpio_names;
	ts->gpio.ngpio = SC16IS7X2_NR_GPIOS;
	ts->gpio.can_sleep = 1;
	ts->gpio.parent = &ts->spi->dev;
	ts->gpio.owner = THIS_MODULE;

	mutex_init(&ts->io_lock);

	ts->io_gpio = 0;
	ts->io_state = 0;
	ts->io_dir = 0;

	sc16is7x2_write(ts, REG_IOI, 0, 0); /* no support for irqs yet */
	sc16is7x2_write(ts, REG_IOS, 0, ts->io_state);
	sc16is7x2_write(ts, REG_IOD, 0, ts->io_dir);

	return gpiochip_add(&ts->gpio);
#else
	return 0;
#endif
}

static int sc16is7x2_register_uart_port(struct sc16is7x2_chip *ts, unsigned ch)
{
	struct sc16is7x2_channel *chan = &(ts->channel[ch]);
	struct uart_port *uart = &chan->uart;
	int ret;

	/* Disable irqs and go to sleep */
	sc16is7x2_write(ts, UART_IER, ch, UART_IERX_SLEEP);

	chan->chip = ts;

	printk(KERN_ERR "irq: %d\n" , ts->spi->irq);
	uart->irq = ts->spi->irq;
	uart->uartclk = ts->uartclk;
	uart->fifosize = FIFO_SIZE;
	uart->ops = &sc16is7x2_uart_ops;
	uart->flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF;
	uart->line = ts->uart_base + ch;
	uart->type = PORT_SC16IS7X2;
	uart->dev = &ts->spi->dev;


	sc16is7x2_channels[uart->line] = chan;

	ret = uart_add_one_port(&sc16is7x2_uart_driver, uart);
	if (!ret) {
		if (uart->cons) {
			register_console(uart->cons);
		}
	} else {
		sc16is7x2_channels[uart->line] = NULL;
	}


	return ret;
}


static int sc16is7x2_remove_one_port(struct sc16is7x2_chip *ts, unsigned ch)
{
	struct sc16is7x2_channel *chan = &(ts->channel[ch]);
	struct uart_port *uart = &chan->uart;
	int line =  uart->line;
	int ret;

	ret = uart_remove_one_port(&sc16is7x2_uart_driver, &ts->channel[ch].uart);
	if (!ret) {
		sc16is7x2_channels[line] = NULL;
	}

	return ret;
}




#ifdef CONFIG_OF





/* of_property_read_bool_with_override - read bool from a property
* @np:         device node from which the property value is to be read.
* @propname:   name of the property to be searched.
*
* Search for a property in a device node.
* Returns false if the property does not exist,
* returns true if property value (u32) is not zero
*/
static inline bool of_property_read_bool_with_override(const struct device_node *np,
                                         const char *propname)
{
	int val;
	if (of_find_property(np, propname, NULL)) {
		// found property, try to parse
		// return true by default, false if value explicitly set to 0
		if (0 == of_property_read_u32(np, propname, &val)) {
			if (val == 0) {
				return false;
			}
		}
		return true;
	} else {
		// property not found
		return false;
	}
}

/*
 * This function returns 1 iff pdev isn't a device instatiated by dt, 0 iff it
 * could successfully get all information from dt or a negative errno.
 */
static int sc16is7x2_probe_dt(struct sc16is7x2_chip *ts,
		struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	//~ const struct of_device_id *of_id =
			//~ of_match_device(imx_uart_dt_ids, &pdev->dev);
	//~ int ret;
	const __be32 *iprop;
	int prop_len;

	if (!np)
		/* no device tree device */
		return 1;



	iprop = of_get_property(np, "uartclk", &prop_len);
	if (!iprop || prop_len < sizeof(*iprop)) {
		dev_err(&spi->dev, "Missing uartclck property in devicetree\n");
		return -EINVAL;
	}

	ts->uartclk = be32_to_cpup(iprop);


	iprop = of_get_property(np, "gpio-base", NULL);
	if (!iprop || prop_len < sizeof(*iprop)) {
		dev_err(&spi->dev, "Missing gpio-base property in devicetree\n");
		return -EINVAL;
	}
	ts->gpio_base = be32_to_cpup(iprop);

	dev_err(&spi->dev, "gpio-base=%d\n",ts->gpio_base);
	dev_err(&spi->dev, "uartclk=%d\n",ts->uartclk);

	iprop = of_get_property(np, "uart-base", NULL);
	if (!iprop || prop_len < sizeof(*iprop)) {
		dev_err(&spi->dev, "Missing uart-base property in devicetree\n");
		return -EINVAL;
	}
	ts->uart_base = be32_to_cpup(iprop);

	dev_err(&spi->dev, "uart_base=%d\n",ts->uart_base);

	if (of_property_read_bool_with_override(np, "disable-modem-pins-on-startup-a"))
		ts->channel[0].use_modem_pins_by_default = false;

	if (of_property_read_bool_with_override(np, "disable-modem-pins-on-startup-b"))
		ts->channel[1].use_modem_pins_by_default = false;



	if (of_property_read_bool_with_override(np, "rs485-rts-active-high-a"))
		ts->channel[0].uart.rs485.flags |= SER_RS485_RTS_ON_SEND;
	else
		ts->channel[0].uart.rs485.flags |= SER_RS485_RTS_AFTER_SEND;

	if (of_property_read_bool_with_override(np, "rs485-rts-active-high-b"))
		ts->channel[1].uart.rs485.flags |= SER_RS485_RTS_ON_SEND;
	else
		ts->channel[1].uart.rs485.flags |= SER_RS485_RTS_AFTER_SEND;


	if (of_property_read_bool_with_override(np, "linux,rs485-enabled-at-boot-time")) {
		ts->channel[0].uart.rs485.flags |= SER_RS485_ENABLED;
		ts->channel[1].uart.rs485.flags |= SER_RS485_ENABLED;
	}

	if (of_property_read_bool_with_override(np, "rs485-enabled-at-boot-time-a")) {
		ts->channel[0].uart.rs485.flags |= SER_RS485_ENABLED;
	}

	if (of_property_read_bool_with_override(np, "rs485-enabled-at-boot-time-b")) {
		ts->channel[1].uart.rs485.flags |= SER_RS485_ENABLED;
	}




	return 0;
}
#else
static inline int sc16is7x2_probe_dt(struct sc16is7x2_chip *ts,
		struct spi_device *spi)
{
	return 1;
}
#endif

static void sc16is7x2_probe_pdata(struct sc16is7x2_chip *ts,
		struct spi_device *spi)
{
	struct sc16is7x2_platform_data *pdata = dev_get_platdata(&spi->dev);

	if (!pdata)
		return;

	ts->uart_base = pdata->uart_base;
	ts->gpio_base = pdata->gpio_base;
	ts->uartclk = pdata->uartclk;
	ts->gpio_label  = pdata->label;
	ts->gpio_names  = pdata->names;
}



static int sc16is7x2_probe(struct spi_device *spi)
{
	struct sc16is7x2_chip *ts;
	//~ struct sc16is7x2_platform_data fake_pdata;

	int ret;
	unsigned char ch;

	/* Only even uart base numbers are supported */
	dev_info(&spi->dev, "probe start\n");

	ts = kzalloc(sizeof(struct sc16is7x2_chip), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	spi_set_drvdata(spi, ts);
	ts->spi = spi;

	for (ch = 0; ch < 2; ++ch) {
		ts->channel[ch].use_modem_pins_by_default = true;
		ts->channel[ch].console_enabled = false;
	}

	ret = sc16is7x2_probe_dt(ts, spi);
	if (ret > 0)
		sc16is7x2_probe_pdata(ts, spi);
	else if (ret < 0)
		return ret;


	if (!ts->gpio_base) {
		dev_err(&spi->dev, "incorrect gpio_base\n");
		return -EINVAL;
	}

	if (ts->uart_base & 1) {
		dev_err(&spi->dev, "incorrect uart_base\n");
		return -EINVAL;
	}


	/* Reset the chip */
	sc16is7x2_write(ts, REG_IOC, 0, IOC_SRESET);

	/* disable all GPIOs, enable on request */
	ts->io_control = 0;
	if (ts->channel[0].use_modem_pins_by_default)
		ts->io_control |= IOC_GPIO30;

	if (ts->channel[1].use_modem_pins_by_default)
		ts->io_control |= IOC_GPIO74;

	sc16is7x2_write(ts, REG_IOC, 0, ts->io_control);


	ret = sc16is7x2_register_uart_port(ts, 0);
	if (ret)
		goto exit_destroy;

	ret = sc16is7x2_register_uart_port(ts, 1);
	if (ret)
		goto exit_uart0;

	ret = sc16is7x2_register_gpio(ts);
	if (ret)
		goto exit_uart1;

	ret = request_irq(spi->irq, sc16is7x2_irq,
			IRQF_TRIGGER_FALLING | IRQF_SHARED,
			"sc16is7x2", ts);
	if (ret)
		goto exit_gpio;

	// init sc16is7x2_async_spi
	for (ch = 0; ch < 2; ++ch) {
		ts->channel[ch].as_rx.tx_buf = &ts->channel[ch].as_rx_buf[0];
		ts->channel[ch].as_rx.rx_buf = &ts->channel[ch].as_rx_buf[1];
		sc16is7x2_async_spi_init(&ts->channel[ch].as_rx, 1, FIFO_SIZE);
		ts->channel[ch].as_rx.persistent = true;
		ts->channel[ch].as_rx.complete = &sc16is7x2_read_rx_complete;
		ts->channel[ch].as_rx.context = &ts->channel[ch];
		ts->channel[ch].as_rx_buf[0] = read_cmd(UART_RX, ch);

		ts->channel[ch].as_tx.tx_buf = &ts->channel[ch].as_tx_buf[0];
		sc16is7x2_async_spi_init(&ts->channel[ch].as_tx,
									(FIFO_SIZE >> 1) + 1, 0);
		ts->channel[ch].as_tx.persistent = true;
		ts->channel[ch].as_tx.complete = &sc16is7x2_write_tx_complete;
		ts->channel[ch].as_tx.context = &ts->channel[ch];
		ts->channel[ch].as_tx_buf[0] = write_cmd(UART_RX, ch);
	}


	dev_info(&spi->dev, DRIVER_NAME " at CS%d (irq %d), 2 UARTs, 8 GPIOs\n"
			"    ttyNSC%d, ttyNSC%d, gpiochip%d\n",
			spi->chip_select, spi->irq,
			ts->uart_base, ts->uart_base + 1,
			ts->gpio_base);

	return 0;

exit_gpio:
#ifdef CONFIG_GPIOLIB
	gpiochip_remove(&ts->gpio);
#endif

exit_uart1:
	sc16is7x2_remove_one_port(ts, 1);

exit_uart0:
	sc16is7x2_remove_one_port(ts, 0);

exit_destroy:
	dev_set_drvdata(&spi->dev, NULL);

	kfree(ts);
	return ret;
}

static int sc16is7x2_remove(struct spi_device *spi)
{
	struct sc16is7x2_chip *ts = spi_get_drvdata(spi);
	int ret;

	if (ts == NULL)
		return -ENODEV;

	/* Free the interrupt */
	free_irq(spi->irq, ts);

	ret = sc16is7x2_remove_one_port(ts, 0);
	if (ret)
		return ret;

	ret = sc16is7x2_remove_one_port(ts, 1);
	if (ret)
		return ret;

#ifdef CONFIG_GPIOLIB
	gpiochip_remove(&ts->gpio);
#endif

	kfree(ts);

	return 0;
}






static void sc16is7x2_console_putchar(struct uart_port *port, int character)
{
	struct sc16is7x2_channel *chan =  to_sc16is7x2_channel(port);
	unsigned ch = (chan == chan->chip->channel) ? 0 : 1;

	sc16is7x2_write(chan->chip, UART_TX, ch, character);



	//~ struct tty_port *tty_port = &ch->uart.state->port;
	//~ struct tty_struct *tty = tty_port_tty_get(tty_port);
	//~ struct uart_state *state = tty->driver_data;
	//~ printk("test state=%d\n", state);
	//~ return __uart_put_char(state->uart_port, &state->xmit, ch);




}

static void
sc16is7x2_console_write(struct console *co, const char *s, unsigned int count)
{

	struct sc16is7x2_channel *ch = sc16is7x2_channels[co->index];

	uart_console_write(&ch->uart, s, count, sc16is7x2_console_putchar);

}
static int __init sc16is7x2_console_setup(struct console *co, char *options)
{
	struct sc16is7x2_channel *ch;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	printk("sc16is7x2_console_setup\n");

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */

	if (co->index >= MAX_SC16IS7X2)
		co->index = 0;

	ch = sc16is7x2_channels[co->index];
	if (!ch)
		return -ENODEV;


	if (ch->console_enabled) {
		return 0;
	}
	ch->console_enabled = true;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	//~ printk("setup ch->uart=%d, co=%d\n", &ch->uart, co);

	sc16is7x2_startup(&ch->uart);




	return uart_set_options(&ch->uart, co, baud, parity, bits, flow);
	return 0;
}




static struct uart_driver sc16is7x2_uart_driver;
#ifndef MODULE
static struct console sc16is7x2_console = {
	.name		= "ttyNSC",
	.write		= sc16is7x2_console_write,
	.device		= uart_console_device,
	.setup		= sc16is7x2_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &sc16is7x2_uart_driver,
};

#define SC16IS7x2_CONSOLE	(&sc16is7x2_console)
#endif

static struct uart_driver sc16is7x2_uart_driver = {
	.owner          = THIS_MODULE,
	.driver_name    = DRIVER_NAME,
	.dev_name       = "ttyNSC",
	.nr             = MAX_SC16IS7X2,
#ifndef MODULE
	.cons			= SC16IS7x2_CONSOLE,
#endif
};





static struct of_device_id spi_sc16is7x2_dt_ids[] = {
	{ .compatible = "fsl,spi-sc16is7x2", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, spi_sc16is7x2_dt_ids);

/* Spi driver data */
static struct spi_driver sc16is7x2_spi_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
		.of_match_table = spi_sc16is7x2_dt_ids,
	},
	.probe		= sc16is7x2_probe,
	.remove		= sc16is7x2_remove,
};


/* Driver init function */
static int __init sc16is7x2_init(void)
{
	int ret = uart_register_driver(&sc16is7x2_uart_driver);
	if (ret)
		return ret;

	return spi_register_driver(&sc16is7x2_spi_driver);
}

/* Driver exit function */
static void __exit sc16is7x2_exit(void)
{
	spi_unregister_driver(&sc16is7x2_spi_driver);
	uart_unregister_driver(&sc16is7x2_uart_driver);
}

/* register after spi postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */

subsys_initcall(sc16is7x2_init);
module_exit(sc16is7x2_exit);

MODULE_AUTHOR("Manuel Stahl");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SC16IS7x2 SPI based UART chip");
MODULE_ALIAS("spi:" DRIVER_NAME);
