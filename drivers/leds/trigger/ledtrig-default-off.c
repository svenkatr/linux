/*
 * LED Kernel Default OFF Trigger
 *
 * Copyright 2015 Evgent Boger <boger@contactles.ru>
 *
 * Based on ledtrig-default-on.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/leds.h>
#include "../leds.h"

static void defoff_trig_activate(struct led_classdev *led_cdev)
{
    led_set_brightness_nosleep(led_cdev, 0);
}

static struct led_trigger defoff_led_trigger = {
    .name     = "default-off",
    .activate = defoff_trig_activate,
};

static int __init defoff_trig_init(void)
{
    return led_trigger_register(&defoff_led_trigger);
}

static void __exit defoff_trig_exit(void)
{
    led_trigger_unregister(&defoff_led_trigger);
}

module_init(defoff_trig_init);
module_exit(defoff_trig_exit);

MODULE_AUTHOR("Evgeny Boger <boger@contactless.ru>");
MODULE_DESCRIPTION("Default-OFF LED trigger");
MODULE_LICENSE("GPL");
