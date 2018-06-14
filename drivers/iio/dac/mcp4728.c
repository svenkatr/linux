/*
 * mcp4728.c - Support for Microchip MCP4728
 *
 * Copyright (C) 2016 Evgeny Boger <boger@contactless.ru>
 *
 * Based on mcp4728 by Peter Meerwald <pmeerw@pmeerw.net>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * driver for the Microchip MCP4728 I2C 12-bit quad digital-to-analog
 * converter (DAC)
 * (7-bit I2C slave address 0x60, the three LSBs can be configured in
 * hardware)
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <linux/iio/dac/mcp4728.h>

#define MCP4728_DRV_NAME "mcp4728"
#define MCP4728_NUM_CHANNELS 4
#define MCP4728_INT_VREF_MV 2048
#define MCP4728_RESOLUTION 12

struct mcp4728_scale {
	unsigned int		integer;
	unsigned int		micro;
};

struct mcp4728_channel_state {
	u16 dac_value;
	bool powerdown;
	u8 gain;
	u8 vref;
	unsigned powerdown_mode;
};

struct mcp4728_data {
	struct i2c_client *client;
	struct mcp4728_channel_state channel_state[MCP4728_NUM_CHANNELS];
	struct mcp4728_scale	scale_avail[MCP4728_NUM_CHANNELS][2];

};

static int mcp4728_write_reg(struct mcp4728_data *data, int ch, bool eeprom)
{
	struct mcp4728_channel_state *state = &data->channel_state[ch];
	u8 outbuf[3];
	u8 pd;
	int ret;

	if (state->dac_value >= (1 << MCP4728_RESOLUTION) ||
				state->dac_value < 0)
		return -EINVAL;
/*
  Write input register. EEPROM is not updated:
  c2=0, c1=1, c0=0, w1=0, w0=0 : 0x40

  Write input and EEPROM registers:
  c2=0, c1=1, c0=0, w1=1, w0=1 : 0x58

  b[0]: c2 | c1 | c0 | w1 | w0 | dac1 | dac0 | /udac
  b[1]: vref | pd1 | pd0 | gain | d11 | d10 | d9 | d8
  b[2]: d7-d0
*/
	pd = state->powerdown ? (state->powerdown_mode + 1) : 0;

	outbuf[0] = (eeprom ? 0x58 : 0x40) |
				((ch & 0x03) << 1);
	outbuf[1] = ((state->vref & 0x01) << 7) |
				((pd & 0x3) << 5)    |
				((state->gain & 0x01) << 4) |
				((state->dac_value >> 8) & 0x0f);
	outbuf[2] = state->dac_value & 0xff;

	ret = i2c_master_send(data->client, outbuf, 3);
	if (ret < 0)
		return ret;
	else if (ret != 3)
		return -EIO;
	else
		return 0;
}

static int mcp4728_write_input_reg(struct mcp4728_data *data, int ch)
{
	return mcp4728_write_reg(data, ch, false);
}

static int mcp4728_suspend(struct device *dev)
{
	struct mcp4728_data *data = iio_priv(i2c_get_clientdata(
		to_i2c_client(dev)));
	int ch, ret;

	for (ch = 0; ch < MCP4728_NUM_CHANNELS; ++ch) {
		data->channel_state[ch].powerdown = true;
		ret = mcp4728_write_input_reg(data, ch);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int mcp4728_resume(struct device *dev)
{
	struct mcp4728_data *data = iio_priv(i2c_get_clientdata(
		to_i2c_client(dev)));
	int ch, ret;

	for (ch = 0; ch < MCP4728_NUM_CHANNELS; ++ch) {
		data->channel_state[ch].powerdown = false;
		ret = mcp4728_write_input_reg(data, ch);
		if (ret < 0)
			return ret;
	}

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static SIMPLE_DEV_PM_OPS(mcp4728_pm_ops, mcp4728_suspend, mcp4728_resume);
#define MCP4728_PM_OPS (&mcp4728_pm_ops)
#else
#define MCP4728_PM_OPS NULL
#endif

static ssize_t mcp4728_store_eeprom(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct mcp4728_data *data = iio_priv(indio_dev);
	int tries;
	u8 inbuf[3];
	bool state;
	int ret, ch;

	ret = strtobool(buf, &state);
	if (ret < 0)
		return ret;

	if (!state)
		return 0;

	for (ch = 0; ch < MCP4728_NUM_CHANNELS; ++ch) {
		tries = 20;
		mcp4728_write_reg(data, ch, true);


		/* wait for write complete, takes up to 50ms */
		while (tries--) {
			msleep(20);
			ret = i2c_master_recv(data->client, inbuf, 1);
			if (ret < 0)
				return ret;
			else if (ret != 1)
				return -EIO;

			if (inbuf[0] & 0x80)
				break;
		}

		if (tries < 0) {
			dev_err(&data->client->dev,
				"mcp4728_store_eeprom() failed, incomplete\n");
			return -EIO;
		}
	}

	return len;
}

static IIO_DEVICE_ATTR(store_eeprom, S_IWUSR, NULL, mcp4728_store_eeprom, 0);


static ssize_t mcp4728_show_scale_available(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev_attr *iio_attr = to_iio_dev_attr(attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct mcp4728_data *data = iio_priv(indio_dev);
	u8 ch = iio_attr->address;
	int i, len = 0;

	for (i = 0; i < ARRAY_SIZE(data->scale_avail[ch]); i++)
		len += sprintf(buf + len, "%u.%06u ",
			       data->scale_avail[ch][i].integer,
			       data->scale_avail[ch][i].micro);

	len += sprintf(buf + len, "\n");

	return len;
}

#define SHOW_SCALE_AVAILABLE_ATTR(ch)					\
static IIO_DEVICE_ATTR(out_voltage##ch##_scale_available, S_IRUGO,	\
		       mcp4728_show_scale_available, NULL, ch)

SHOW_SCALE_AVAILABLE_ATTR(0);
SHOW_SCALE_AVAILABLE_ATTR(1);
SHOW_SCALE_AVAILABLE_ATTR(2);
SHOW_SCALE_AVAILABLE_ATTR(3);

static struct attribute *mcp4728_attributes[] = {
	&iio_dev_attr_store_eeprom.dev_attr.attr,
	&iio_dev_attr_out_voltage0_scale_available.dev_attr.attr,
	&iio_dev_attr_out_voltage1_scale_available.dev_attr.attr,
	&iio_dev_attr_out_voltage2_scale_available.dev_attr.attr,
	&iio_dev_attr_out_voltage3_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group mcp4728_attribute_group = {
	.attrs = mcp4728_attributes,
};

static const char * const mcp4728_powerdown_modes[] = {
	"1kohm_to_gnd",
	"100kohm_to_gnd",
	"500kohm_to_gnd"
};

static int mcp4728_get_powerdown_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct mcp4728_data *data = iio_priv(indio_dev);

	return data->channel_state[chan->channel].powerdown_mode;
}

static int mcp4728_set_powerdown_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned mode)
{
	struct mcp4728_data *data = iio_priv(indio_dev);

	data->channel_state[chan->channel].powerdown_mode = mode;

	return 0;
}

static ssize_t mcp4728_read_powerdown(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct mcp4728_data *data = iio_priv(indio_dev);

	return sprintf(buf, "%d\n",
		data->channel_state[chan->channel].powerdown);
}

static ssize_t mcp4728_write_powerdown(struct iio_dev *indio_dev,
	 uintptr_t private, const struct iio_chan_spec *chan,
	 const char *buf, size_t len)
{
	struct mcp4728_data *data = iio_priv(indio_dev);
	struct mcp4728_channel_state *state =
			&data->channel_state[chan->channel];
	bool powerdown;
	int ret;

	ret = strtobool(buf, &powerdown);
	if (ret)
		return ret;

	state->powerdown = powerdown;
	ret = mcp4728_write_input_reg(data, chan->channel);

	if (ret < 0)
		return ret;

	return len;
}

static const struct iio_enum mcp4728_powerdown_mode_enum = {
	.items = mcp4728_powerdown_modes,
	.num_items = ARRAY_SIZE(mcp4728_powerdown_modes),
	.get = mcp4728_get_powerdown_mode,
	.set = mcp4728_set_powerdown_mode,
};

static const struct iio_chan_spec_ext_info mcp4728_ext_info[] = {
	{
		.name = "powerdown",
		.read = mcp4728_read_powerdown,
		.write = mcp4728_write_powerdown,
		.shared = IIO_SEPARATE,
	},
	IIO_ENUM("powerdown_mode", IIO_SEPARATE, &mcp4728_powerdown_mode_enum),
	IIO_ENUM_AVAILABLE("powerdown_mode", &mcp4728_powerdown_mode_enum),
	{ },
};

#define MCP4728_CHAN(chan) {			\
	.type		= IIO_VOLTAGE, \
	.indexed	= 1, \
	.output		= 1, \
	.channel	= chan, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			      BIT(IIO_CHAN_INFO_SCALE), \
	.ext_info	= mcp4728_ext_info, \
}


static const struct iio_chan_spec mcp4728_channels[MCP4728_NUM_CHANNELS] = {
	MCP4728_CHAN(0),
	MCP4728_CHAN(1),
	MCP4728_CHAN(2),
	MCP4728_CHAN(3),
};

static int mcp4728_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct mcp4728_data *data = iio_priv(indio_dev);
	struct mcp4728_channel_state *state =
			&data->channel_state[chan->channel];

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*val = data->channel_state[chan->channel].dac_value;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = (state->gain + 1) * MCP4728_INT_VREF_MV;
		*val2 = MCP4728_RESOLUTION;
		return IIO_VAL_FRACTIONAL_LOG2;
	}
	return -EINVAL;
}

static int mcp4728_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask)
{
	struct mcp4728_data *data = iio_priv(indio_dev);
	struct mcp4728_channel_state *state =
			&data->channel_state[chan->channel];
	struct mcp4728_scale *scale_avail =
		data->scale_avail[chan->channel];

	int ret, gain;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		state->dac_value = val;
		ret = mcp4728_write_input_reg(data, chan->channel);
		break;
	case IIO_CHAN_INFO_SCALE:
		ret = -EINVAL;

		for (gain = 0; gain < 2; ++gain) {
			if (val == scale_avail[gain].integer &&
			val2 == scale_avail[gain].micro) {

				state->gain = gain;
				ret = mcp4728_write_input_reg(data,
					chan->channel);
			}
		}

		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct iio_info mcp4728_info = {
	.read_raw = mcp4728_read_raw,
	.write_raw = mcp4728_write_raw,
	.attrs = &mcp4728_attribute_group,
	.driver_module = THIS_MODULE,
};

static int mcp4728_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct mcp4728_data *data;
	struct iio_dev *indio_dev;
	u8 inbuf[24];
	u8 *reg;
	u8 pd;
	int err;
	int i, gain;
	u64 scale_uv;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (indio_dev == NULL)
		return -ENOMEM;
	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &mcp4728_info;
	indio_dev->channels = mcp4728_channels;
	indio_dev->num_channels = MCP4728_NUM_CHANNELS;
	indio_dev->modes = INDIO_DIRECT_MODE;

	/* read current DAC value */
	err = i2c_master_recv(client, inbuf, 24);
	if (err < 0) {
		dev_err(&client->dev, "failed to read DAC value");
		return err;
	}

	for (i = 0; i < MCP4728_NUM_CHANNELS; ++i) {
		reg = &inbuf[6*i];
		pd = (reg[1] >> 5) & 0x3;
		data->channel_state[i].powerdown = pd > 0 ? true : false;

		/* set to 500kohm_to_gnd if channel is now powered down */
		data->channel_state[i].powerdown_mode = pd ? pd-1 : 2;
		data->channel_state[i].dac_value = ((reg[1] & 0x0f) << 8) |
						   (reg[2]);
		data->channel_state[i].gain = (reg[1] >> 4) & 0x01;

		/* Use internal reference */
		data->channel_state[i].vref = 1;

		/* Populate available ADC input ranges */
		for (gain = 0; gain < 2; gain++) {
			/*
			  [gain=0] = Gain Setting 1, vref x 1
			  [gain=1] = Gain Setting 2, vref x 2

			  The scale is calculated by doing:
			    Vref >> (realbits - gain)
			  which multiplies by two on the second component
			  of the array.
			 */
			scale_uv = ((u64) MCP4728_INT_VREF_MV * 100000) >>
				   (MCP4728_RESOLUTION - gain);
			data->scale_avail[i][gain].micro =
					do_div(scale_uv, 100000) * 10;
			data->scale_avail[i][gain].integer = scale_uv;
		}
	}

	return iio_device_register(indio_dev);
}

static int mcp4728_remove(struct i2c_client *client)
{
	iio_device_unregister(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id mcp4728_id[] = {
	{ "mcp4728", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mcp4728_id);

static struct i2c_driver mcp4728_driver = {
	.driver = {
		.name	= MCP4728_DRV_NAME,
		.pm	= MCP4728_PM_OPS,
	},
	.probe		= mcp4728_probe,
	.remove		= mcp4728_remove,
	.id_table	= mcp4728_id,
};
module_i2c_driver(mcp4728_driver);

MODULE_AUTHOR("Evgeny Boger <boger@contactless.ru>");
MODULE_DESCRIPTION("MCP4728 quad 12-bit DAC");
MODULE_LICENSE("GPL");
