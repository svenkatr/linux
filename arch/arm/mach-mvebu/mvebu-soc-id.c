/*
 * ID and revision information for mvebu SoCs
 *
 * Copyright (C) 2014 Marvell
 *
 * Gregory CLEMENT <gregory.clement@free-electrons.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 * All the mvebu SoCs have information related to their variant and
 * revision that can be read from the PCI control register. This is
 * done before the PCI initialization to avoid any conflict. Once the
 * ID and revision are retrieved, the mapping is freed.
 */

#define pr_fmt(fmt) "mvebu-soc-id: " fmt

#include <linux/clk.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include "mvebu-soc-id.h"

#define PCIE_DEV_ID_OFF		0x0
#define PCIE_DEV_REV_OFF	0x8

#define SOC_ID_MASK	    0xFFFF0000
#define SOC_REV_MASK	    0xFF

static u32 soc_dev_id;
static u32 soc_rev;
static bool is_id_valid;

static const struct of_device_id mvebu_pcie_of_match_table[] = {
	{ .compatible = "marvell,armada-xp-pcie", },
	{ .compatible = "marvell,armada-370-pcie", },
	{ .compatible = "marvell,kirkwood-pcie" },
	{},
};

int mvebu_get_soc_id(u32 *dev, u32 *rev)
{
	if (is_id_valid) {
		*dev = soc_dev_id;
		*rev = soc_rev;
		return 0;
	} else
		return -1;
}

static int __init mvebu_soc_id_init(void)
{
	struct device_node *np;
	int ret = 0;
	void __iomem *pci_base;
	struct clk *clk;
	struct device_node *child;

	np = of_find_matching_node(NULL, mvebu_pcie_of_match_table);
	if (!np)
		return ret;

	/*
	 * ID and revision are available from any port, so we
	 * just pick the first one
	 */
	child = of_get_next_child(np, NULL);
	if (child == NULL) {
		pr_err("cannot get pci node\n");
		ret = -ENOMEM;
		goto clk_err;
	}

	clk = of_clk_get_by_name(child, NULL);
	if (IS_ERR(clk)) {
		pr_err("cannot get clock\n");
		ret = -ENOMEM;
		goto clk_err;
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		pr_err("cannot enable clock\n");
		goto clk_err;
	}

	pci_base = of_iomap(child, 0);
	if (pci_base == NULL) {
		pr_err("cannot map registers\n");
		ret = -ENOMEM;
		goto res_ioremap;
	}

	/* SoC ID */
	soc_dev_id = readl(pci_base + PCIE_DEV_ID_OFF) >> 16;

	/* SoC revision */
	soc_rev = readl(pci_base + PCIE_DEV_REV_OFF) & SOC_REV_MASK;

	is_id_valid = true;

	pr_info("MVEBU SoC ID=0x%X, Rev=0x%X\n", soc_dev_id, soc_rev);

	iounmap(pci_base);

res_ioremap:
	/*
	 * If the PCIe unit is actually enabled and we have PCI
	 * support in the kernel, we intentionally do not release the
	 * reference to the clock. We want to keep it running since
	 * the bootloader does some PCIe link configuration that the
	 * kernel is for now unable to do, and gating the clock would
	 * make us loose this precious configuration.
	 */
	if (!of_device_is_available(child) || !IS_ENABLED(CONFIG_PCI_MVEBU)) {
		clk_disable_unprepare(clk);
		clk_put(clk);
	}

clk_err:
	of_node_put(child);
	of_node_put(np);

	return ret;
}
core_initcall(mvebu_soc_id_init);

