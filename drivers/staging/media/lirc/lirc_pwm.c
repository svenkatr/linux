/*
 * lirc_pwm.c
 *
 * lirc_pwm - driver that replays pulse- and space-lengths,
 *	      generating carrier with PWM.
 *
 * Author: Alexey Ignatov <lexszero@gmail.com>
 */
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/pwm.h>
#include <media/lirc.h>
#include <media/lirc_dev.h>

#define DRIVER_NAME	"lirc_pwm"

#define DEFAULT_DUTY_CYCLE	50
#define DEFAULT_FREQ		38000
#define IDLE_PERIOD			1000000

#define RBUF_LEN	4096

struct lirc_pwm_priv {
	struct lirc_driver *driver;

	/* Transmit */
	struct lirc_pwm_send {
		struct pwm_device *pwm;
		unsigned int pwm_max_duty;
		int gpio;
		bool gpio_active_low;
		/* Stored values of carrier frequency and duty percent as requested
		 * from LIRC
		 */
		unsigned int duty_cycle;
		unsigned int freq;
		/* Calculated values of PWM period and duty times in nanoseconds */
		unsigned int pwm_period;
		unsigned int pwm_duty;
		bool pwm_idle_active;

		int (*toggle)(struct lirc_pwm_priv *, bool);
		struct mutex lock;
	} send;

	/* Receive */
	struct lirc_pwm_recv {
		int gpio;
		bool gpio_active_low;
		int irq;
		bool enabled;
		bool filter;
		struct lirc_buffer rbuf;
		ktime_t last;
		int state;
		int pulse, space;
		unsigned int ptr;
	} recv;
};

#define priv_dev(x) ((x)->driver->dev)
#define recv_dev(x) priv_dev(container_of((x), struct lirc_pwm_priv, recv))
#define send_dev(x) priv_dev(container_of((x), struct lirc_pwm_priv, send))

#ifndef MAX_UDELAY_MS
#define MAX_UDELAY_US 5000
#else
#define MAX_UDELAY_US (MAX_UDELAY_MS*1000)
#endif

static void safe_udelay(unsigned long usecs)
{
	while (usecs > MAX_UDELAY_US) {
		udelay(MAX_UDELAY_US);
		usecs -= MAX_UDELAY_US;
	}
	udelay(usecs);
}

static inline bool send_available(struct lirc_pwm_priv *priv)
{
	return (priv->send.toggle != NULL);
}

static inline bool recv_available(struct lirc_pwm_priv *priv)
{
	return gpio_is_valid(priv->recv.gpio);
}

static void recv_enable(struct lirc_pwm_priv *priv)
{
	struct lirc_pwm_recv *recv = &priv->recv;
	if (!recv_available(priv) || recv->enabled)
		return;

	recv->last = ktime_get();
	gpio_direction_input(recv->gpio);
	enable_irq(recv->irq);
	recv->enabled = true;
}

static void recv_disable(struct lirc_pwm_priv *priv)
{
	struct lirc_pwm_recv *recv = &priv->recv;
	if (!recv_available(priv) || !recv->enabled)
		return;

	disable_irq(recv->irq);
	priv->recv.enabled = false;
}

static int send_toggle_pwm(struct lirc_pwm_priv *priv, bool pulse)
{
	struct lirc_pwm_send *send = &priv->send;
	int ret = 0;
	ret = pwm_config(send->pwm, pulse ? send->pwm_duty : 0,
			send->pwm_period);
	if (ret < 0)
		return ret;
	return pwm_enable(send->pwm);
}

static int send_toggle_gpio(struct lirc_pwm_priv *priv, bool pulse)
{
	if (priv->send.gpio_active_low)
		pulse = !pulse;
	gpio_set_value(priv->send.gpio, pulse);
	return 0;
}

static inline bool send_using_pwm(struct lirc_pwm_priv *priv)
{
	return (priv->send.toggle == send_toggle_pwm);
}

static int send_pwm_update_timings(struct lirc_pwm_priv *priv,
		unsigned int duty_cycle, unsigned int freq)
{
	struct lirc_pwm_send *send = &priv->send;

	if (!send->pwm)
		return -ENODEV;

	if (duty_cycle > 100)
		return -EINVAL;
	if (freq < 20000 || freq > 500000)
		return -EINVAL;

	mutex_lock(&send->lock);

	send->duty_cycle = duty_cycle;
	send->freq = freq;
	send->pwm_period = 1000000000 / freq;
	send->pwm_duty = send->pwm_period * duty_cycle * send->pwm_max_duty / 100 / 100;
	dev_dbg(priv_dev(priv), "period: %d ns, duty: %d ns\n",
			send->pwm_period, send->pwm_duty);

	mutex_unlock(&send->lock);
	return 0;
}

static void recv_rbuf_write(struct lirc_pwm_recv *recv, int l)
{
	struct lirc_buffer *rbuf = &recv->rbuf;
	if (lirc_buffer_full(rbuf)) {
		/* no new signals will be accepted */
		dev_warn(recv_dev(recv), "buffer overrun\n");
		return;
	}
	lirc_buffer_write(rbuf, (void *)&l);
}

static void recv_got_space_pulse(struct lirc_pwm_recv *recv)
{
	recv_rbuf_write(recv, recv->space);
	recv_rbuf_write(recv, recv->pulse | PULSE_BIT);
	recv->ptr = 0;
	recv->pulse = 0;
}

static void recv_rbuf_write_filtered(struct lirc_pwm_recv *recv, int l)
{
	/* simple noise filter */
	if (recv->ptr > 0 && (l & PULSE_BIT)) {
		recv->pulse += l & PULSE_MASK;
		if (recv->pulse > 250) {
			recv_got_space_pulse(recv);
		}
		return;
	}
	if (!(l & PULSE_BIT)) {
		if (!recv->ptr) {
			if (l > 20000) {
				recv->space = l;
				recv->ptr++;
				return;
			}
		} else {
			if (l > 20000) {
				recv->space += recv->pulse;
				if (recv->space > PULSE_MASK)
					recv->space = PULSE_MASK;
				recv->space += l;
				if (recv->space > PULSE_MASK)
					recv->space = PULSE_MASK;
				recv->pulse = 0;
				return;
			}
			recv_got_space_pulse(recv);
		}
	}
	recv_rbuf_write(recv, l);
}

static irqreturn_t recv_irq_handler(int irq, void *data)
{
	struct lirc_pwm_recv *recv = data;
	struct device *dev = recv_dev(recv);
	int duration, value = gpio_get_value(recv->gpio);
	ktime_t now;
	s64 delta;

	if (recv->gpio_active_low)
		value = !value;

	if (recv->state != -1) {
		now = ktime_get();
		delta = ktime_us_delta(now, recv->last);
		if (delta > 15000000) {
			duration = PULSE_MASK;
			if (value == recv->state) {
				dev_warn(dev, "Value on the pin #%d hasn't been changed! " \
						"(wtf?) value=%d, last=%lld, now=%lld, delta=%lld",
						recv->gpio, value, 
						ktime_to_us(recv->last), ktime_to_us(now), delta);
						
				recv->state = !recv->state;
			}
		}
		else {
			duration = (int)delta;
		}
		if (recv->filter) {
			recv_rbuf_write_filtered(recv,
					duration | ((value == recv->state) ? PULSE_BIT : 0));
		}
		else {
			recv_rbuf_write(recv, duration | (value ? 0 : PULSE_BIT));
		}
		recv->last = now;
		wake_up_interruptible(&recv->rbuf.wait_poll);
	}

	return IRQ_HANDLED;
}


static int lirc_pwm_set_use_inc(void *data)
{
	struct lirc_pwm_priv *priv = data;

	dev_dbg(priv_dev(priv), "set_use_inc");

	recv_enable(priv);
	return send_pwm_update_timings(priv, DEFAULT_DUTY_CYCLE, DEFAULT_FREQ);
}

static void lirc_pwm_set_use_dec(void *data)
{
	struct lirc_pwm_priv *priv = (struct lirc_pwm_priv *)data;

	dev_dbg(priv_dev(priv), "set_use_dec");

	recv_disable(priv);
}

static ssize_t lirc_pwm_write(struct file *filep, const char *buf,
	size_t n, loff_t *ppos)
{
	struct lirc_driver *driver = filep->private_data;
	struct lirc_pwm_priv *priv = driver->data;
	struct lirc_pwm_send *send = &priv->send;
	int i, count, ret = 0;
	int *wbuf;

	if (!send_available(priv))
		return -EINVAL;

	count = n / sizeof(int);
	if (n % sizeof(int) || count % 2 == 0)
		return -EINVAL;

	mutex_lock(&send->lock);
	recv_disable(priv);

	wbuf = memdup_user(buf, n);
	if (IS_ERR(wbuf)) {
		ret = PTR_ERR(wbuf);
		goto exit_unlock;
	}

	for (i = 0; i < count; i++) {
		send->toggle(priv, !(i & 1));
		safe_udelay(wbuf[i]);
	}
	send->toggle(priv, false);
	if (send_using_pwm(priv)) {
		if (send->pwm_idle_active)
			pwm_config(send->pwm, IDLE_PERIOD-1, IDLE_PERIOD);
		else
			pwm_disable(send->pwm);
	}

	kfree(wbuf);
	wbuf = NULL;

exit_unlock:
	recv_enable(priv);
	mutex_unlock(&send->lock);
	return ret;
}

static long lirc_pwm_ioctl(struct file *filep, unsigned int cmd,
		unsigned long arg)
{
	struct lirc_driver *driver = filep->private_data;
	struct lirc_pwm_priv *priv = driver->data;
	int ret;
	uint32_t value;

	switch (cmd) {
	case LIRC_GET_SEND_MODE:
		return -ENOIOCTLCMD;

	case LIRC_SET_SEND_MODE:
		ret = get_user(value, (uint32_t *) arg);
		if (ret)
			return ret;
		/* only LIRC_MODE_PULSE supported */
		if (value != LIRC_MODE_PULSE)
			return -ENOSYS;
		break;

	case LIRC_GET_LENGTH:
		return -ENOSYS;

	case LIRC_SET_SEND_DUTY_CYCLE:
		if (!(driver->features & LIRC_CAN_SET_SEND_DUTY_CYCLE))
			return -ENOSYS;
		ret = get_user(value, (uint32_t *) arg);
		if (ret)
			return ret;
		dev_dbg(driver->dev, "SET_SEND_DUTY_CYCLE %d\n", value);
		return send_pwm_update_timings(priv, value, priv->send.freq);

	case LIRC_SET_SEND_CARRIER:
		if (!(driver->features & LIRC_CAN_SET_SEND_CARRIER))
			return -ENOSYS;
		ret = get_user(value, (__u32 *) arg);
		if (ret)
			return ret;
		dev_dbg(driver->dev, "SET_SEND_CARRIER %d\n", value);
		return send_pwm_update_timings(priv, priv->send.duty_cycle, value);

	default:
		return lirc_dev_fop_ioctl(filep, cmd, arg);
	}
	return 0;
}

static const struct file_operations lirc_pwm_fops = {
	.owner		= THIS_MODULE,
	.write		= lirc_pwm_write,
	.unlocked_ioctl	= lirc_pwm_ioctl,
	.read		= lirc_dev_fop_read,
	.poll		= lirc_dev_fop_poll,
	.open		= lirc_dev_fop_open,
	.release	= lirc_dev_fop_close,
	.llseek		= no_llseek,
};

/*
static struct lirc_driver lirc_pwm_template = {
	.name		= DRIVER_NAME,
	.minor		= -1,
	.features	= LIRC_CAN_SET_SEND_DUTY_CYCLE |
		LIRC_CAN_SET_SEND_CARRIER |
		LIRC_CAN_SEND_PULSE,
	.code_length	= 1,
	.sample_rate	= 0,
	.data		= NULL,
	.add_to_buf	= NULL,
	.rbuf		= NULL,
	.set_use_inc	= lirc_pwm_set_use_inc,
	.set_use_dec	= lirc_pwm_set_use_dec,
	.fops		= &lirc_pwm_fops,
	.dev		= NULL,
	.owner		= THIS_MODULE,
};
*/

static inline int detect_recv_gpio_active_low(struct device *dev,
		int gpio)
{
	int ret, i, nlow = 0, nhigh = 0;

	ret = devm_gpio_request_one(dev, gpio, GPIOF_DIR_IN, "lirc-rx");
	if (ret < 0)
		return ret;

	/* wait for all transitions to settle, just for sure */
	mdelay(10);

	for (i = 0; i < 9; i++) {
		if (gpio_get_value(gpio))
			nlow++;
		else
			nhigh++;
		msleep(40);
	}
	devm_gpio_free(dev, gpio);
	return (nlow >= nhigh);
}

static inline int register_lirc_driver(struct device *dev,
		struct lirc_pwm_priv *priv)
{
	struct lirc_driver *driver;
	int ret;

	driver = devm_kzalloc(dev, sizeof(struct lirc_driver), GFP_KERNEL);
	if (!driver)
		return -ENOMEM;

	strncpy(driver->name, dev_name(dev), 40);
	driver->minor = -1;
	driver->code_length = 1;
	driver->sample_rate = 0;
	driver->features = 0;
	driver->data = priv;
	driver->rbuf = recv_available(priv) ? &priv->recv.rbuf : NULL;
	driver->set_use_inc = lirc_pwm_set_use_inc;
	driver->set_use_dec = lirc_pwm_set_use_dec;
	driver->fops = &lirc_pwm_fops;
	driver->dev = dev;
	driver->owner = THIS_MODULE;

	if (send_available(priv)) {
		driver->features |= LIRC_CAN_SEND_PULSE;
		if (send_using_pwm(priv))
			driver->features |=
				LIRC_CAN_SET_SEND_DUTY_CYCLE |
				LIRC_CAN_SET_SEND_CARRIER;
	}

	if (recv_available(priv))
		driver->features |= LIRC_CAN_REC_MODE2;

	ret = lirc_register_driver(driver);
	if (ret < 0) {
		dev_err(dev, "lirc_register_driver failed: %d\n", ret);
		return ret;
	}
	driver->minor = ret;
	priv->driver = driver;

	return ret;
}

static void recv_rbuf_free(struct lirc_pwm_recv *recv)
{
	if (kfifo_initialized(&recv->rbuf.fifo))
		lirc_buffer_free(&recv->rbuf);
}

static int lirc_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct lirc_pwm_priv *priv;
	struct lirc_pwm_send *send;
	struct lirc_pwm_recv *recv;
	enum of_gpio_flags gpio_flags;
	const char *gpio_label;
	int ret = -EINVAL;

	priv = devm_kzalloc(dev, sizeof(struct lirc_pwm_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	send = &priv->send;
	recv = &priv->recv;
	
	dev_set_drvdata(dev, priv);

	/* Fetch everything we need from DT */
	send->pwm = devm_of_pwm_get(dev, node, NULL);
	if (IS_ERR(send->pwm)) {
		ret = PTR_ERR(send->pwm);
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_dbg(dev, "can't get transmit PWM: %d\n", ret);
	}
	else {
		ret = of_property_read_u32(node, "pwm-max-duty", &send->pwm_max_duty);
		if (ret < 0 || send->pwm_max_duty > 100)
			send->pwm_max_duty = 100;

		if (of_find_property(node, "pwm-idle-active", NULL)) {
			send->pwm_idle_active = true;
			pwm_config(send->pwm, IDLE_PERIOD-1, IDLE_PERIOD);
			pwm_enable(send->pwm);
		}

		dev_info(dev, "PWM transmitter on %s, max duty %d%%\n",
				send->pwm->label ? : "<unnamed channel>",
				send->pwm_max_duty);
		send->toggle = send_toggle_pwm;
	}

	ret = of_get_named_gpio_flags(node, "gpio-send", 0, &gpio_flags);
	if (!gpio_is_valid(ret)) {
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_dbg(dev, "can't get transmit GPIO: %d\n", ret);
	}
	send->gpio = ret;
	send->gpio_active_low = (gpio_flags & OF_GPIO_ACTIVE_LOW);

	ret = of_get_named_gpio_flags(node, "gpio-recv", 0, &gpio_flags);
	if (!gpio_is_valid(ret)) {
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_dbg(dev, "can't get receive GPIO: %d\n", ret);
	}
	recv->gpio = ret;
	recv->gpio_active_low = (gpio_flags & OF_GPIO_ACTIVE_LOW);

	/* Request appropriate gpios, if needed */
	if (gpio_is_valid(send->gpio)) {
		if (!send->toggle) {
			if (recv->gpio == send->gpio) {
				dev_info(dev, "GPIO receiver and transmitter shares one pin\n");
				gpio_label = "lirc-rxtx";
			}
			else
				gpio_label = "lirc-tx";

			ret = devm_gpio_request(dev, priv->send.gpio, gpio_label);
			if (ret < 0) {
				dev_err(dev, "unable to request %s GPIO %d: %d\n",
						gpio_label, send->gpio, ret);
				return ret;
			}
			send->toggle = send_toggle_gpio;
			dev_info(dev, "GPIO transmitter on pin %d\n", send->gpio);
		}
		else
			dev_warn(dev, "both PWM and GPIO transmitters are specified, will use PWM, fix your DT\n");
	}

	mutex_init(&send->lock);

	if (gpio_is_valid(recv->gpio)) {
		/* Request gpio-recv pin only when it's not requested by gpio-recvtx */
		if (recv->gpio != send->gpio || send->toggle != send_toggle_gpio) {
			if (of_find_property(node, "gpio-recv-detect-active-level", NULL)) {
				ret = detect_recv_gpio_active_low(dev, recv->gpio);
				if (ret < 0)
					return ret;
				recv->gpio_active_low = ret;
			}

			ret = devm_gpio_request(dev, recv->gpio, "lirc-rx");
			if (ret < 0) {
				dev_err(dev, "unable to request lirc-rx GPIO %d: %d\n",
						recv->gpio, ret);
				return ret;
			}
		}
		else
			dev_dbg(dev, "shared GPIO receiver/transmitter pin already initialized\n");

		/* Initialize IRQ on recv->gpio pin */
		recv->irq = gpio_to_irq(recv->gpio);
		ret = devm_request_irq(dev, recv->irq, recv_irq_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				dev_name(dev), recv);
		if (ret < 0) {
			dev_err(dev, "unable to request lirc-rx IRQ %d: %d\n",
					recv->irq, ret);
			return ret;
		}
		disable_irq(recv->irq);

		recv->filter = !of_find_property(node, "gpio-recv-disable-filter", NULL);

		ret = lirc_buffer_init(&recv->rbuf, sizeof(int), RBUF_LEN);
		if (ret < 0)
			return ret;

		dev_info(dev, "GPIO receiver on pin %d\n", recv->gpio);
	}

	ret = register_lirc_driver(dev, priv);
	if (ret < 0) {
		dev_err(dev, "LIRC driver registration failed: %d\n", ret);
		goto exit_rbuf_free;
	}

	dev_info(dev, "probed\n");
	return 0;

exit_rbuf_free:
	recv_rbuf_free(recv);
	return ret;
}

static int lirc_pwm_remove(struct platform_device *pdev)
{
	struct lirc_pwm_priv *priv = platform_get_drvdata(pdev);
	int ret;

	dev_dbg(&pdev->dev, "removing\n");

	if (send_using_pwm(priv)) {
		pwm_disable(priv->send.pwm);
	}

	if (gpio_is_valid(priv->send.gpio)) {
		gpio_direction_input(priv->send.gpio);
	}

	if (kfifo_initialized(&priv->recv.rbuf.fifo))
		lirc_buffer_free(&priv->recv.rbuf);

	ret = lirc_unregister_driver(priv->driver->minor);
	if (ret)
		dev_err(&pdev->dev, "unable to unregister from lirc\n");
	else
		dev_info(&pdev->dev, "unregistered from lirc\n");

	return ret;
}

static struct of_device_id lirc_pwm_dt_ids[] = {
	{ .compatible = "lirc-pwm" },
	{}
};
MODULE_DEVICE_TABLE(of, lirc_pwm_dt_ids);

static struct platform_driver lirc_pwm_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(lirc_pwm_dt_ids),
	},
	.probe = lirc_pwm_probe,
	.remove	= lirc_pwm_remove,
};

module_platform_driver(lirc_pwm_driver);

MODULE_DESCRIPTION("LIRC generic GPIO/PWM Transceiver driver");
MODULE_AUTHOR("Alexey Ignatov <lexszero@gmail.com>");
MODULE_LICENSE("GPL");
