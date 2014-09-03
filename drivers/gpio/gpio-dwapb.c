/*
 * Copyright (c) 2011 Jamie Iles
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */
#include <linux/basic_mmio_gpio.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define GPIO_SWPORTA_DR		0x00
#define GPIO_SWPORTA_DDR	0x04
#define GPIO_SWPORTB_DR		0x0c
#define GPIO_SWPORTB_DDR	0x10
#define GPIO_SWPORTC_DR		0x18
#define GPIO_SWPORTC_DDR	0x1c
#define GPIO_SWPORTD_DR		0x24
#define GPIO_SWPORTD_DDR	0x28
#define GPIO_INTEN		0x30
#define GPIO_INTMASK		0x34
#define GPIO_INTTYPE_LEVEL	0x38
#define GPIO_INT_POLARITY	0x3c
#define GPIO_INTSTATUS		0x40
#define GPIO_PORTA_EOI		0x4c
#define GPIO_EXT_PORTA		0x50
#define GPIO_EXT_PORTB		0x54
#define GPIO_EXT_PORTC		0x58
#define GPIO_EXT_PORTD		0x5c

#define DWAPB_MAX_PORTS		4
#define GPIO_EXT_PORT_SIZE	(GPIO_EXT_PORTB - GPIO_EXT_PORTA)
#define GPIO_SWPORT_DR_SIZE	(GPIO_SWPORTB_DR - GPIO_SWPORTA_DR)
#define GPIO_SWPORT_DDR_SIZE	(GPIO_SWPORTB_DDR - GPIO_SWPORTA_DDR)

struct dwapb_gpio;

struct dwapb_gpio_port {
	struct bgpio_chip	bgc;
	bool			is_registered;
	struct dwapb_gpio	*gpio;
};

struct dwapb_gpio {
	struct	device		*dev;
	void __iomem		*regs;
	struct dwapb_gpio_port	*ports;
	unsigned int		nr_ports;
	struct irq_domain	*domain;
};

static int dwapb_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct bgpio_chip *bgc = to_bgpio_chip(gc);
	struct dwapb_gpio_port *port = container_of(bgc, struct
						    dwapb_gpio_port, bgc);
	struct dwapb_gpio *gpio = port->gpio;

	return irq_find_mapping(gpio->domain, offset);
}

static void dwapb_toggle_trigger(struct dwapb_gpio *gpio, unsigned int offs)
{
	u32 v = readl(gpio->regs + GPIO_INT_POLARITY);

	if (gpio_get_value(gpio->ports[0].bgc.gc.base + offs))
		v &= ~BIT(offs);
	else
		v |= BIT(offs);

	writel(v, gpio->regs + GPIO_INT_POLARITY);
}

static void dwapb_irq_handler(u32 irq, struct irq_desc *desc)
{
	struct dwapb_gpio *gpio = irq_get_handler_data(irq);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 irq_status = readl_relaxed(gpio->regs + GPIO_INTSTATUS);

	while (irq_status) {
		int hwirq = fls(irq_status) - 1;
		int gpio_irq = irq_find_mapping(gpio->domain, hwirq);

		generic_handle_irq(gpio_irq);
		irq_status &= ~BIT(hwirq);

		if ((irq_get_trigger_type(gpio_irq) & IRQ_TYPE_SENSE_MASK)
			== IRQ_TYPE_EDGE_BOTH)
			dwapb_toggle_trigger(gpio, hwirq);
	}

	if (chip->irq_eoi)
		chip->irq_eoi(irq_desc_get_irq_data(desc));
}

static void dwapb_irq_enable(struct irq_data *d)
{
	struct irq_chip_generic *igc = irq_data_get_irq_chip_data(d);
	struct dwapb_gpio *gpio = igc->private;
	struct bgpio_chip *bgc = &gpio->ports[0].bgc;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&bgc->lock, flags);
	val = readl(gpio->regs + GPIO_INTEN);
	val |= BIT(d->hwirq);
	writel(val, gpio->regs + GPIO_INTEN);
	spin_unlock_irqrestore(&bgc->lock, flags);
}

static void dwapb_irq_disable(struct irq_data *d)
{
	struct irq_chip_generic *igc = irq_data_get_irq_chip_data(d);
	struct dwapb_gpio *gpio = igc->private;
	struct bgpio_chip *bgc = &gpio->ports[0].bgc;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&bgc->lock, flags);
	val = readl(gpio->regs + GPIO_INTEN);
	val &= ~BIT(d->hwirq);
	writel(val, gpio->regs + GPIO_INTEN);
	spin_unlock_irqrestore(&bgc->lock, flags);
}

static int dwapb_irq_reqres(struct irq_data *d)
{
	struct irq_chip_generic *igc = irq_data_get_irq_chip_data(d);
	struct dwapb_gpio *gpio = igc->private;
	struct bgpio_chip *bgc = &gpio->ports[0].bgc;

	if (gpio_lock_as_irq(&bgc->gc, irqd_to_hwirq(d))) {
		dev_err(gpio->dev, "unable to lock HW IRQ %lu for IRQ\n",
			irqd_to_hwirq(d));
		return -EINVAL;
	}
	return 0;
}

static void dwapb_irq_relres(struct irq_data *d)
{
	struct irq_chip_generic *igc = irq_data_get_irq_chip_data(d);
	struct dwapb_gpio *gpio = igc->private;
	struct bgpio_chip *bgc = &gpio->ports[0].bgc;

	gpio_unlock_as_irq(&bgc->gc, irqd_to_hwirq(d));
}

static int dwapb_irq_set_type(struct irq_data *d, u32 type)
{
	struct irq_chip_generic *igc = irq_data_get_irq_chip_data(d);
	struct dwapb_gpio *gpio = igc->private;
	struct bgpio_chip *bgc = &gpio->ports[0].bgc;
	int bit = d->hwirq;
	unsigned long level, polarity, flags;

	if (type & ~(IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING |
		     IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_LEVEL_LOW))
		return -EINVAL;

	spin_lock_irqsave(&bgc->lock, flags);
	level = readl(gpio->regs + GPIO_INTTYPE_LEVEL);
	polarity = readl(gpio->regs + GPIO_INT_POLARITY);

	switch (type) {
	case IRQ_TYPE_EDGE_BOTH:
		level |= BIT(bit);
		dwapb_toggle_trigger(gpio, bit);
		break;
	case IRQ_TYPE_EDGE_RISING:
		level |= BIT(bit);
		polarity |= BIT(bit);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		level |= BIT(bit);
		polarity &= ~BIT(bit);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		level &= ~BIT(bit);
		polarity |= BIT(bit);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		level &= ~BIT(bit);
		polarity &= ~BIT(bit);
		break;
	}

	writel(level, gpio->regs + GPIO_INTTYPE_LEVEL);
	writel(polarity, gpio->regs + GPIO_INT_POLARITY);
	spin_unlock_irqrestore(&bgc->lock, flags);

	return 0;
}

static void dwapb_configure_irqs(struct dwapb_gpio *gpio,
				 struct dwapb_gpio_port *port)
{
	struct gpio_chip *gc = &port->bgc.gc;
	struct device_node *node =  gc->of_node;
	struct irq_chip_generic	*irq_gc;
	unsigned int hwirq, ngpio = gc->ngpio;
	struct irq_chip_type *ct;
	int err, irq;

	irq = irq_of_parse_and_map(node, 0);
	if (!irq) {
		dev_warn(gpio->dev, "no irq for bank %s\n",
			port->bgc.gc.of_node->full_name);
		return;
	}

	gpio->domain = irq_domain_add_linear(node, ngpio,
					     &irq_generic_chip_ops, gpio);
	if (!gpio->domain)
		return;

	err = irq_alloc_domain_generic_chips(gpio->domain, ngpio, 1,
					     "gpio-dwapb", handle_level_irq,
					     IRQ_NOREQUEST, 0,
					     IRQ_GC_INIT_NESTED_LOCK);
	if (err) {
		dev_info(gpio->dev, "irq_alloc_domain_generic_chips failed\n");
		irq_domain_remove(gpio->domain);
		gpio->domain = NULL;
		return;
	}

	irq_gc = irq_get_domain_generic_chip(gpio->domain, 0);
	if (!irq_gc) {
		irq_domain_remove(gpio->domain);
		gpio->domain = NULL;
		return;
	}

	irq_gc->reg_base = gpio->regs;
	irq_gc->private = gpio;

	ct = irq_gc->chip_types;
	ct->chip.irq_ack = irq_gc_ack_set_bit;
	ct->chip.irq_mask = irq_gc_mask_set_bit;
	ct->chip.irq_unmask = irq_gc_mask_clr_bit;
	ct->chip.irq_set_type = dwapb_irq_set_type;
	ct->chip.irq_enable = dwapb_irq_enable;
	ct->chip.irq_disable = dwapb_irq_disable;
	ct->chip.irq_request_resources = dwapb_irq_reqres;
	ct->chip.irq_release_resources = dwapb_irq_relres;
	ct->regs.ack = GPIO_PORTA_EOI;
	ct->regs.mask = GPIO_INTMASK;

	irq_setup_generic_chip(irq_gc, IRQ_MSK(port->bgc.gc.ngpio),
			IRQ_GC_INIT_NESTED_LOCK, IRQ_NOREQUEST, 0);

	irq_set_chained_handler(irq, dwapb_irq_handler);
	irq_set_handler_data(irq, gpio);

	for (hwirq = 0 ; hwirq < ngpio ; hwirq++)
		irq_create_mapping(gpio->domain, hwirq);

	port->bgc.gc.to_irq = dwapb_gpio_to_irq;
}

static void dwapb_irq_teardown(struct dwapb_gpio *gpio)
{
	struct dwapb_gpio_port *port = &gpio->ports[0];
	struct gpio_chip *gc = &port->bgc.gc;
	unsigned int ngpio = gc->ngpio;
	irq_hw_number_t hwirq;

	if (!gpio->domain)
		return;

	for (hwirq = 0 ; hwirq < ngpio ; hwirq++)
		irq_dispose_mapping(irq_find_mapping(gpio->domain, hwirq));

	irq_domain_remove(gpio->domain);
	gpio->domain = NULL;
}

static int dwapb_gpio_add_port(struct dwapb_gpio *gpio,
			       struct device_node *port_np,
			       unsigned int offs)
{
	struct dwapb_gpio_port *port;
	u32 port_idx, ngpio;
	void __iomem *dat, *set, *dirout;
	int err;

	if (of_property_read_u32(port_np, "reg", &port_idx) ||
		port_idx >= DWAPB_MAX_PORTS) {
		dev_err(gpio->dev, "missing/invalid port index for %s\n",
			port_np->full_name);
		return -EINVAL;
	}

	port = &gpio->ports[offs];
	port->gpio = gpio;

	if (of_property_read_u32(port_np, "snps,nr-gpios", &ngpio)) {
		dev_info(gpio->dev, "failed to get number of gpios for %s\n",
			 port_np->full_name);
		ngpio = 32;
	}

	dat = gpio->regs + GPIO_EXT_PORTA + (port_idx * GPIO_EXT_PORT_SIZE);
	set = gpio->regs + GPIO_SWPORTA_DR + (port_idx * GPIO_SWPORT_DR_SIZE);
	dirout = gpio->regs + GPIO_SWPORTA_DDR +
		(port_idx * GPIO_SWPORT_DDR_SIZE);

	err = bgpio_init(&port->bgc, gpio->dev, 4, dat, set, NULL, dirout,
			 NULL, false);
	if (err) {
		dev_err(gpio->dev, "failed to init gpio chip for %s\n",
			port_np->full_name);
		return err;
	}

	port->bgc.gc.ngpio = ngpio;
	port->bgc.gc.of_node = port_np;

	/*
	 * Only port A can provide interrupts in all configurations of the IP.
	 */
	if (port_idx == 0 &&
	    of_property_read_bool(port_np, "interrupt-controller"))
		dwapb_configure_irqs(gpio, port);

	err = gpiochip_add(&port->bgc.gc);
	if (err)
		dev_err(gpio->dev, "failed to register gpiochip for %s\n",
			port_np->full_name);
	else
		port->is_registered = true;

	return err;
}

static void dwapb_gpio_unregister(struct dwapb_gpio *gpio)
{
	unsigned int m;

	for (m = 0; m < gpio->nr_ports; ++m)
		if (gpio->ports[m].is_registered)
			WARN_ON(gpiochip_remove(&gpio->ports[m].bgc.gc));
}

static int dwapb_gpio_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct dwapb_gpio *gpio;
	struct device_node *np;
	int err;
	unsigned int offs = 0;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;
	gpio->dev = &pdev->dev;

	gpio->nr_ports = of_get_child_count(pdev->dev.of_node);
	if (!gpio->nr_ports) {
		err = -EINVAL;
		goto out_err;
	}
	gpio->ports = devm_kzalloc(&pdev->dev, gpio->nr_ports *
				   sizeof(*gpio->ports), GFP_KERNEL);
	if (!gpio->ports) {
		err = -ENOMEM;
		goto out_err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gpio->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(gpio->regs)) {
		err = PTR_ERR(gpio->regs);
		goto out_err;
	}

	for_each_child_of_node(pdev->dev.of_node, np) {
		err = dwapb_gpio_add_port(gpio, np, offs++);
		if (err)
			goto out_unregister;
	}
	platform_set_drvdata(pdev, gpio);

	return 0;

out_unregister:
	dwapb_gpio_unregister(gpio);
	dwapb_irq_teardown(gpio);

out_err:
	return err;
}

static int dwapb_gpio_remove(struct platform_device *pdev)
{
	struct dwapb_gpio *gpio = platform_get_drvdata(pdev);

	dwapb_gpio_unregister(gpio);
	dwapb_irq_teardown(gpio);

	return 0;
}

static const struct of_device_id dwapb_of_match[] = {
	{ .compatible = "snps,dw-apb-gpio" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, dwapb_of_match);

static struct platform_driver dwapb_gpio_driver = {
	.driver		= {
		.name	= "gpio-dwapb",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(dwapb_of_match),
	},
	.probe		= dwapb_gpio_probe,
	.remove		= dwapb_gpio_remove,
};

module_platform_driver(dwapb_gpio_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jamie Iles");
MODULE_DESCRIPTION("Synopsys DesignWare APB GPIO driver");
