/*
 * w1-gpio - GPIO w1 bus master driver
 *
 * Copyright (C) 2007 Ville Syrjala <syrjala@sci.fi>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/w1-gpio.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/delay.h>

#include "../w1.h"
#include "../w1_int.h"

static u8 w1_gpio_set_pullup(void *data, int delay)
{
	struct w1_gpio_platform_data *pdata = data;

	if (delay) {
		pdata->pullup_duration = delay;
	} else {
		if (pdata->pullup_duration) {
			gpio_direction_output(pdata->pin, 1);

			msleep(pdata->pullup_duration);

			gpio_direction_input(pdata->pin);
		}
		pdata->pullup_duration = 0;
	}

	return 0;
}

static u8 w1_gpio_set_strong_pullup(void *data, int delay)
{
	struct w1_gpio_platform_data *pdata = data;

	if (delay) {
		pdata->pullup_duration = delay;
	} else {
		if (pdata->pullup_duration) {
			gpio_direction_output(pdata->strong_pullup_enable_pin, 1);

			msleep(pdata->pullup_duration);

			gpio_direction_output(pdata->strong_pullup_enable_pin, 0);
		}
		pdata->pullup_duration = 0;
	}

	return 0;
}

static void w1_gpio_write_bit_dir(void *data, u8 bit)
{
	struct w1_gpio_platform_data *pdata = data;

	if (bit)
		gpio_direction_input(pdata->pin);
	else
		gpio_direction_output(pdata->pin, 0);
}

static void w1_gpio_write_bit_val(void *data, u8 bit)
{
	struct w1_gpio_platform_data *pdata = data;

	gpio_set_value(pdata->pin, bit);
}

static u8 w1_gpio_read_bit(void *data)
{
	struct w1_gpio_platform_data *pdata = data;

	return gpio_get_value(pdata->pin) ? 1 : 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id w1_gpio_dt_ids[] = {
	{ .compatible = "w1-gpio" },
	{}
};
MODULE_DEVICE_TABLE(of, w1_gpio_dt_ids);
#endif

static struct w1_gpio_platform_data *w1_gpio_probe_dt(struct platform_device *pdev)
{
	struct w1_gpio_platform_data *pdata;
	struct device_node *np = pdev->dev.of_node;
	int gpio;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	if (of_get_property(np, "linux,open-drain", NULL))
		pdata->is_open_drain = 1;

	gpio = of_get_gpio(np, 0);
	if (gpio < 0) {
		if (gpio != -EPROBE_DEFER)
			dev_err(&pdev->dev,
					"Failed to parse gpio property for data pin (%d)\n",
					gpio);

		return ERR_PTR(gpio);
	}
	pdata->pin = gpio;

	gpio = of_get_gpio(np, 1);
	if (gpio == -EPROBE_DEFER)
		return ERR_PTR(gpio);
	/* ignore other errors as the pullup gpio is optional */
	pdata->ext_pullup_enable_pin = gpio;

	gpio = of_get_named_gpio(np, "pu-gpios", 0);
	if (gpio == -EPROBE_DEFER)
		return ERR_PTR(gpio);
	/* ignore other errors as the strong pullup gpio is optional */
	pdata->strong_pullup_enable_pin = gpio;

	return pdata;
}

static int w1_gpio_probe(struct platform_device *pdev)
{
	struct w1_bus_master *master;
	struct w1_gpio_platform_data *pdata;
	int err;

	if (of_have_populated_dt()) {
		pdata = w1_gpio_probe_dt(pdev);
		if (IS_ERR(pdata) < 0)
			return PTR_ERR(pdata);
	}
	else
		pdata = dev_get_platdata(&pdev->dev);

	if (!pdata) {
		dev_err(&pdev->dev, "No configuration data\n");
		return -ENXIO;
	}

	master = devm_kzalloc(&pdev->dev, sizeof(struct w1_bus_master),
			GFP_KERNEL);
	if (!master) {
		dev_err(&pdev->dev, "Out of memory\n");
		return -ENOMEM;
	}

	err = devm_gpio_request(&pdev->dev, pdata->pin, "w1");
	if (err) {
		dev_err(&pdev->dev, "gpio_request (pin) failed\n");
		return err;
	}

	if (gpio_is_valid(pdata->ext_pullup_enable_pin)) {
		err = devm_gpio_request_one(&pdev->dev,
				pdata->ext_pullup_enable_pin, GPIOF_INIT_LOW,
				"w1 pullup");
		if (err < 0) {
			dev_err(&pdev->dev, "gpio_request_one "
					"(ext_pullup_enable_pin) failed\n");
			return err;
		}
	}

	if (gpio_is_valid(pdata->strong_pullup_enable_pin)) {
		err = devm_gpio_request_one(&pdev->dev,
				pdata->strong_pullup_enable_pin, GPIOF_INIT_LOW,
				"w1 strong pullup");
		if (err < 0) {
			dev_err(&pdev->dev, "gpio_request_one "
					"(strong_pullup_enable_pin) failed\n");
			return err;
		}
	}

	master->data = pdata;
	master->read_bit = w1_gpio_read_bit;

	if (pdata->is_open_drain) {
		gpio_direction_output(pdata->pin, 1);
		master->write_bit = w1_gpio_write_bit_val;
	} else {
		gpio_direction_input(pdata->pin);
		master->write_bit = w1_gpio_write_bit_dir;
		master->set_pullup = w1_gpio_set_pullup;
	}

	if (gpio_is_valid(pdata->strong_pullup_enable_pin))
		master->set_pullup = w1_gpio_set_strong_pullup;


	err = w1_add_master_device(master);
	if (err) {
		dev_err(&pdev->dev, "w1_add_master device failed\n");
		return err;
	}

	if (pdata->enable_external_pullup)
		pdata->enable_external_pullup(1);

	if (gpio_is_valid(pdata->ext_pullup_enable_pin))
		gpio_set_value(pdata->ext_pullup_enable_pin, 1);

	if (gpio_is_valid(pdata->strong_pullup_enable_pin))
		gpio_set_value(pdata->strong_pullup_enable_pin, 0);

	platform_set_drvdata(pdev, master);

	return 0;
}

static int w1_gpio_remove(struct platform_device *pdev)
{
	struct w1_bus_master *master = platform_get_drvdata(pdev);
	struct w1_gpio_platform_data *pdata = master->data;

	pr_debug("%s: pdev=%p, master=%p\n", __func__, pdev, master);
	if (pdata->enable_external_pullup)
		pdata->enable_external_pullup(0);

	if (gpio_is_valid(pdata->ext_pullup_enable_pin))
		gpio_set_value(pdata->ext_pullup_enable_pin, 0);

	w1_remove_master_device(master);

	return 0;
}

static int __maybe_unused w1_gpio_suspend(struct device *dev)
{
	struct w1_bus_master *master = dev_get_drvdata(dev);
	struct w1_gpio_platform_data *pdata = master->data;

	if (pdata->enable_external_pullup)
		pdata->enable_external_pullup(0);

	return 0;
}

static int __maybe_unused w1_gpio_resume(struct device *dev)
{
	struct w1_bus_master *master = dev_get_drvdata(dev);
	struct w1_gpio_platform_data *pdata = master->data;

	if (pdata->enable_external_pullup)
		pdata->enable_external_pullup(1);

	return 0;
}

static SIMPLE_DEV_PM_OPS(w1_gpio_pm_ops, w1_gpio_suspend, w1_gpio_resume);

static struct platform_driver w1_gpio_driver = {
	.driver = {
		.name	= "w1-gpio",
		.pm	= &w1_gpio_pm_ops,
		.of_match_table = of_match_ptr(w1_gpio_dt_ids),
	},
	.probe = w1_gpio_probe,
	.remove = w1_gpio_remove,
};

module_platform_driver(w1_gpio_driver);

MODULE_DESCRIPTION("GPIO w1 bus master driver");
MODULE_AUTHOR("Ville Syrjala <syrjala@sci.fi>");
MODULE_LICENSE("GPL");
