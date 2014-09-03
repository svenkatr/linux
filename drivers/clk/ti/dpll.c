/*
 * OMAP DPLL clock support
 *
 * Copyright (C) 2013 Texas Instruments, Inc.
 *
 * Tero Kristo <t-kristo@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clk/ti.h>

#undef pr_fmt
#define pr_fmt(fmt) "%s: " fmt, __func__

#define DPLL_HAS_AUTOIDLE	0x1

#if defined(CONFIG_ARCH_OMAP4) || defined(CONFIG_SOC_OMAP5) || \
	defined(CONFIG_SOC_DRA7XX)
static const struct clk_ops dpll_m4xen_ck_ops = {
	.enable		= &omap3_noncore_dpll_enable,
	.disable	= &omap3_noncore_dpll_disable,
	.recalc_rate	= &omap4_dpll_regm4xen_recalc,
	.round_rate	= &omap4_dpll_regm4xen_round_rate,
	.set_rate	= &omap3_noncore_dpll_set_rate,
	.get_parent	= &omap2_init_dpll_parent,
};
#endif

static const struct clk_ops dpll_core_ck_ops = {
	.recalc_rate	= &omap3_dpll_recalc,
	.get_parent	= &omap2_init_dpll_parent,
};

#ifdef CONFIG_ARCH_OMAP3
static const struct clk_ops omap3_dpll_core_ck_ops = {
	.get_parent	= &omap2_init_dpll_parent,
	.recalc_rate	= &omap3_dpll_recalc,
	.round_rate	= &omap2_dpll_round_rate,
};
#endif

static const struct clk_ops dpll_ck_ops = {
	.enable		= &omap3_noncore_dpll_enable,
	.disable	= &omap3_noncore_dpll_disable,
	.recalc_rate	= &omap3_dpll_recalc,
	.round_rate	= &omap2_dpll_round_rate,
	.set_rate	= &omap3_noncore_dpll_set_rate,
	.get_parent	= &omap2_init_dpll_parent,
};

static const struct clk_ops dpll_no_gate_ck_ops = {
	.recalc_rate	= &omap3_dpll_recalc,
	.get_parent	= &omap2_init_dpll_parent,
	.round_rate	= &omap2_dpll_round_rate,
	.set_rate	= &omap3_noncore_dpll_set_rate,
};

#ifdef CONFIG_ARCH_OMAP3
static const struct clk_ops omap3_dpll_ck_ops = {
	.enable		= &omap3_noncore_dpll_enable,
	.disable	= &omap3_noncore_dpll_disable,
	.get_parent	= &omap2_init_dpll_parent,
	.recalc_rate	= &omap3_dpll_recalc,
	.set_rate	= &omap3_noncore_dpll_set_rate,
	.round_rate	= &omap2_dpll_round_rate,
};

static const struct clk_ops omap3_dpll_per_ck_ops = {
	.enable		= &omap3_noncore_dpll_enable,
	.disable	= &omap3_noncore_dpll_disable,
	.get_parent	= &omap2_init_dpll_parent,
	.recalc_rate	= &omap3_dpll_recalc,
	.set_rate	= &omap3_dpll4_set_rate,
	.round_rate	= &omap2_dpll_round_rate,
};
#endif

static const struct clk_ops dpll_x2_ck_ops = {
	.recalc_rate	= &omap3_clkoutx2_recalc,
};

/**
 * ti_clk_register_dpll - low level registration of a DPLL clock
 * @hw: hardware clock definition for the clock
 * @node: device node for the clock
 *
 * Finalizes DPLL registration process. In case a failure (clk-ref or
 * clk-bypass is missing), the clock is added to retry list and
 * the initialization is retried on later stage.
 */
static void __init ti_clk_register_dpll(struct clk_hw *hw,
					struct device_node *node)
{
	struct clk_hw_omap *clk_hw = to_clk_hw_omap(hw);
	struct dpll_data *dd = clk_hw->dpll_data;
	struct clk *clk;

	dd->clk_ref = of_clk_get(node, 0);
	dd->clk_bypass = of_clk_get(node, 1);

	if (IS_ERR(dd->clk_ref) || IS_ERR(dd->clk_bypass)) {
		pr_debug("clk-ref or clk-bypass missing for %s, retry later\n",
			 node->name);
		if (!ti_clk_retry_init(node, hw, ti_clk_register_dpll))
			return;

		goto cleanup;
	}

	/* register the clock */
	clk = clk_register(NULL, &clk_hw->hw);

	if (!IS_ERR(clk)) {
		omap2_init_clk_hw_omap_clocks(clk);
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
		kfree(clk_hw->hw.init->parent_names);
		kfree(clk_hw->hw.init);
		return;
	}

cleanup:
	kfree(clk_hw->dpll_data);
	kfree(clk_hw->hw.init->parent_names);
	kfree(clk_hw->hw.init);
	kfree(clk_hw);
}

#if defined(CONFIG_ARCH_OMAP4) || defined(CONFIG_SOC_OMAP5) || \
	defined(CONFIG_SOC_DRA7XX) || defined(CONFIG_SOC_AM33XX)
/**
 * ti_clk_register_dpll_x2 - Registers a DPLLx2 clock
 * @node: device node for this clock
 * @ops: clk_ops for this clock
 * @hw_ops: clk_hw_ops for this clock
 *
 * Initializes a DPLL x 2 clock from device tree data.
 */
static void ti_clk_register_dpll_x2(struct device_node *node,
				    const struct clk_ops *ops,
				    const struct clk_hw_omap_ops *hw_ops)
{
	struct clk *clk;
	struct clk_init_data init = { NULL };
	struct clk_hw_omap *clk_hw;
	const char *name = node->name;
	const char *parent_name;

	parent_name = of_clk_get_parent_name(node, 0);
	if (!parent_name) {
		pr_err("%s must have parent\n", node->name);
		return;
	}

	clk_hw = kzalloc(sizeof(*clk_hw), GFP_KERNEL);
	if (!clk_hw)
		return;

	clk_hw->ops = hw_ops;
	clk_hw->hw.init = &init;

	init.name = name;
	init.ops = ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	/* register the clock */
	clk = clk_register(NULL, &clk_hw->hw);

	if (IS_ERR(clk)) {
		kfree(clk_hw);
	} else {
		omap2_init_clk_hw_omap_clocks(clk);
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
	}
}
#endif

/**
 * of_ti_dpll_setup - Setup function for OMAP DPLL clocks
 * @node: device node containing the DPLL info
 * @ops: ops for the DPLL
 * @ddt: DPLL data template to use
 * @init_flags: flags for controlling init types
 *
 * Initializes a DPLL clock from device tree data.
 */
static void __init of_ti_dpll_setup(struct device_node *node,
				    const struct clk_ops *ops,
				    const struct dpll_data *ddt,
				    u8 init_flags)
{
	struct clk_hw_omap *clk_hw = NULL;
	struct clk_init_data *init = NULL;
	const char **parent_names = NULL;
	struct dpll_data *dd = NULL;
	int i;
	u8 dpll_mode = 0;

	dd = kzalloc(sizeof(*dd), GFP_KERNEL);
	clk_hw = kzalloc(sizeof(*clk_hw), GFP_KERNEL);
	init = kzalloc(sizeof(*init), GFP_KERNEL);
	if (!dd || !clk_hw || !init)
		goto cleanup;

	memcpy(dd, ddt, sizeof(*dd));

	clk_hw->dpll_data = dd;
	clk_hw->ops = &clkhwops_omap3_dpll;
	clk_hw->hw.init = init;
	clk_hw->flags = MEMMAP_ADDRESSING;

	init->name = node->name;
	init->ops = ops;

	init->num_parents = of_clk_get_parent_count(node);
	if (init->num_parents < 1) {
		pr_err("%s must have parent(s)\n", node->name);
		goto cleanup;
	}

	parent_names = kzalloc(sizeof(char *) * init->num_parents, GFP_KERNEL);
	if (!parent_names)
		goto cleanup;

	for (i = 0; i < init->num_parents; i++)
		parent_names[i] = of_clk_get_parent_name(node, i);

	init->parent_names = parent_names;

	dd->control_reg = ti_clk_get_reg_addr(node, 0);
	dd->idlest_reg = ti_clk_get_reg_addr(node, 1);
	dd->mult_div1_reg = ti_clk_get_reg_addr(node, 2);

	if (!dd->control_reg || !dd->idlest_reg || !dd->mult_div1_reg)
		goto cleanup;

	if (init_flags & DPLL_HAS_AUTOIDLE) {
		dd->autoidle_reg = ti_clk_get_reg_addr(node, 3);
		if (!dd->autoidle_reg)
			goto cleanup;
	}

	if (of_property_read_bool(node, "ti,low-power-stop"))
		dpll_mode |= 1 << DPLL_LOW_POWER_STOP;

	if (of_property_read_bool(node, "ti,low-power-bypass"))
		dpll_mode |= 1 << DPLL_LOW_POWER_BYPASS;

	if (of_property_read_bool(node, "ti,lock"))
		dpll_mode |= 1 << DPLL_LOCKED;

	if (dpll_mode)
		dd->modes = dpll_mode;

	ti_clk_register_dpll(&clk_hw->hw, node);
	return;

cleanup:
	kfree(dd);
	kfree(parent_names);
	kfree(init);
	kfree(clk_hw);
}

#if defined(CONFIG_ARCH_OMAP4) || defined(CONFIG_SOC_OMAP5) || \
	defined(CONFIG_SOC_DRA7XX)
static void __init of_ti_omap4_dpll_x2_setup(struct device_node *node)
{
	ti_clk_register_dpll_x2(node, &dpll_x2_ck_ops, &clkhwops_omap4_dpllmx);
}
CLK_OF_DECLARE(ti_omap4_dpll_x2_clock, "ti,omap4-dpll-x2-clock",
	       of_ti_omap4_dpll_x2_setup);
#endif

#ifdef CONFIG_SOC_AM33XX
static void __init of_ti_am3_dpll_x2_setup(struct device_node *node)
{
	ti_clk_register_dpll_x2(node, &dpll_x2_ck_ops, NULL);
}
CLK_OF_DECLARE(ti_am3_dpll_x2_clock, "ti,am3-dpll-x2-clock",
	       of_ti_am3_dpll_x2_setup);
#endif

#ifdef CONFIG_ARCH_OMAP3
static void __init of_ti_omap3_dpll_setup(struct device_node *node)
{
	const struct dpll_data dd = {
		.idlest_mask = 0x1,
		.enable_mask = 0x7,
		.autoidle_mask = 0x7,
		.mult_mask = 0x7ff << 8,
		.div1_mask = 0x7f,
		.max_multiplier = 2047,
		.max_divider = 128,
		.min_divider = 1,
		.freqsel_mask = 0xf0,
		.modes = (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	};

	of_ti_dpll_setup(node, &omap3_dpll_ck_ops, &dd, DPLL_HAS_AUTOIDLE);
}
CLK_OF_DECLARE(ti_omap3_dpll_clock, "ti,omap3-dpll-clock",
	       of_ti_omap3_dpll_setup);

static void __init of_ti_omap3_core_dpll_setup(struct device_node *node)
{
	const struct dpll_data dd = {
		.idlest_mask = 0x1,
		.enable_mask = 0x7,
		.autoidle_mask = 0x7,
		.mult_mask = 0x7ff << 16,
		.div1_mask = 0x7f << 8,
		.max_multiplier = 2047,
		.max_divider = 128,
		.min_divider = 1,
		.freqsel_mask = 0xf0,
	};

	of_ti_dpll_setup(node, &omap3_dpll_core_ck_ops, &dd, DPLL_HAS_AUTOIDLE);
}
CLK_OF_DECLARE(ti_omap3_core_dpll_clock, "ti,omap3-dpll-core-clock",
	       of_ti_omap3_core_dpll_setup);

static void __init of_ti_omap3_per_dpll_setup(struct device_node *node)
{
	const struct dpll_data dd = {
		.idlest_mask = 0x1 << 1,
		.enable_mask = 0x7 << 16,
		.autoidle_mask = 0x7 << 3,
		.mult_mask = 0x7ff << 8,
		.div1_mask = 0x7f,
		.max_multiplier = 2047,
		.max_divider = 128,
		.min_divider = 1,
		.freqsel_mask = 0xf00000,
		.modes = (1 << DPLL_LOW_POWER_STOP) | (1 << DPLL_LOCKED),
	};

	of_ti_dpll_setup(node, &omap3_dpll_per_ck_ops, &dd, DPLL_HAS_AUTOIDLE);
}
CLK_OF_DECLARE(ti_omap3_per_dpll_clock, "ti,omap3-dpll-per-clock",
	       of_ti_omap3_per_dpll_setup);

static void __init of_ti_omap3_per_jtype_dpll_setup(struct device_node *node)
{
	const struct dpll_data dd = {
		.idlest_mask = 0x1 << 1,
		.enable_mask = 0x7 << 16,
		.autoidle_mask = 0x7 << 3,
		.mult_mask = 0xfff << 8,
		.div1_mask = 0x7f,
		.max_multiplier = 4095,
		.max_divider = 128,
		.min_divider = 1,
		.sddiv_mask = 0xff << 24,
		.dco_mask = 0xe << 20,
		.flags = DPLL_J_TYPE,
		.modes = (1 << DPLL_LOW_POWER_STOP) | (1 << DPLL_LOCKED),
	};

	of_ti_dpll_setup(node, &omap3_dpll_per_ck_ops, &dd, DPLL_HAS_AUTOIDLE);
}
CLK_OF_DECLARE(ti_omap3_per_jtype_dpll_clock, "ti,omap3-dpll-per-j-type-clock",
	       of_ti_omap3_per_jtype_dpll_setup);
#endif

static void __init of_ti_omap4_dpll_setup(struct device_node *node)
{
	const struct dpll_data dd = {
		.idlest_mask = 0x1,
		.enable_mask = 0x7,
		.autoidle_mask = 0x7,
		.mult_mask = 0x7ff << 8,
		.div1_mask = 0x7f,
		.max_multiplier = 2047,
		.max_divider = 128,
		.min_divider = 1,
		.modes = (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	};

	of_ti_dpll_setup(node, &dpll_ck_ops, &dd, DPLL_HAS_AUTOIDLE);
}
CLK_OF_DECLARE(ti_omap4_dpll_clock, "ti,omap4-dpll-clock",
	       of_ti_omap4_dpll_setup);

static void __init of_ti_omap4_core_dpll_setup(struct device_node *node)
{
	const struct dpll_data dd = {
		.idlest_mask = 0x1,
		.enable_mask = 0x7,
		.autoidle_mask = 0x7,
		.mult_mask = 0x7ff << 8,
		.div1_mask = 0x7f,
		.max_multiplier = 2047,
		.max_divider = 128,
		.min_divider = 1,
		.modes = (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	};

	of_ti_dpll_setup(node, &dpll_core_ck_ops, &dd, DPLL_HAS_AUTOIDLE);
}
CLK_OF_DECLARE(ti_omap4_core_dpll_clock, "ti,omap4-dpll-core-clock",
	       of_ti_omap4_core_dpll_setup);

#if defined(CONFIG_ARCH_OMAP4) || defined(CONFIG_SOC_OMAP5) || \
	defined(CONFIG_SOC_DRA7XX)
static void __init of_ti_omap4_m4xen_dpll_setup(struct device_node *node)
{
	const struct dpll_data dd = {
		.idlest_mask = 0x1,
		.enable_mask = 0x7,
		.autoidle_mask = 0x7,
		.mult_mask = 0x7ff << 8,
		.div1_mask = 0x7f,
		.max_multiplier = 2047,
		.max_divider = 128,
		.min_divider = 1,
		.m4xen_mask = 0x800,
		.lpmode_mask = 1 << 10,
		.modes = (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	};

	of_ti_dpll_setup(node, &dpll_m4xen_ck_ops, &dd, DPLL_HAS_AUTOIDLE);
}
CLK_OF_DECLARE(ti_omap4_m4xen_dpll_clock, "ti,omap4-dpll-m4xen-clock",
	       of_ti_omap4_m4xen_dpll_setup);

static void __init of_ti_omap4_jtype_dpll_setup(struct device_node *node)
{
	const struct dpll_data dd = {
		.idlest_mask = 0x1,
		.enable_mask = 0x7,
		.autoidle_mask = 0x7,
		.mult_mask = 0xfff << 8,
		.div1_mask = 0xff,
		.max_multiplier = 4095,
		.max_divider = 256,
		.min_divider = 1,
		.sddiv_mask = 0xff << 24,
		.flags = DPLL_J_TYPE,
		.modes = (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	};

	of_ti_dpll_setup(node, &dpll_m4xen_ck_ops, &dd, DPLL_HAS_AUTOIDLE);
}
CLK_OF_DECLARE(ti_omap4_jtype_dpll_clock, "ti,omap4-dpll-j-type-clock",
	       of_ti_omap4_jtype_dpll_setup);
#endif

static void __init of_ti_am3_no_gate_dpll_setup(struct device_node *node)
{
	const struct dpll_data dd = {
		.idlest_mask = 0x1,
		.enable_mask = 0x7,
		.autoidle_mask = 0x7,
		.mult_mask = 0x7ff << 8,
		.div1_mask = 0x7f,
		.max_multiplier = 2047,
		.max_divider = 128,
		.min_divider = 1,
		.modes = (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	};

	of_ti_dpll_setup(node, &dpll_no_gate_ck_ops, &dd, 0);
}
CLK_OF_DECLARE(ti_am3_no_gate_dpll_clock, "ti,am3-dpll-no-gate-clock",
	       of_ti_am3_no_gate_dpll_setup);

static void __init of_ti_am3_jtype_dpll_setup(struct device_node *node)
{
	const struct dpll_data dd = {
		.idlest_mask = 0x1,
		.enable_mask = 0x7,
		.autoidle_mask = 0x7,
		.mult_mask = 0x7ff << 8,
		.div1_mask = 0x7f,
		.max_multiplier = 4095,
		.max_divider = 256,
		.min_divider = 2,
		.flags = DPLL_J_TYPE,
		.modes = (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	};

	of_ti_dpll_setup(node, &dpll_ck_ops, &dd, 0);
}
CLK_OF_DECLARE(ti_am3_jtype_dpll_clock, "ti,am3-dpll-j-type-clock",
	       of_ti_am3_jtype_dpll_setup);

static void __init of_ti_am3_no_gate_jtype_dpll_setup(struct device_node *node)
{
	const struct dpll_data dd = {
		.idlest_mask = 0x1,
		.enable_mask = 0x7,
		.autoidle_mask = 0x7,
		.mult_mask = 0x7ff << 8,
		.div1_mask = 0x7f,
		.max_multiplier = 2047,
		.max_divider = 128,
		.min_divider = 1,
		.flags = DPLL_J_TYPE,
		.modes = (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	};

	of_ti_dpll_setup(node, &dpll_no_gate_ck_ops, &dd, 0);
}
CLK_OF_DECLARE(ti_am3_no_gate_jtype_dpll_clock,
	       "ti,am3-dpll-no-gate-j-type-clock",
	       of_ti_am3_no_gate_jtype_dpll_setup);

static void __init of_ti_am3_dpll_setup(struct device_node *node)
{
	const struct dpll_data dd = {
		.idlest_mask = 0x1,
		.enable_mask = 0x7,
		.autoidle_mask = 0x7,
		.mult_mask = 0x7ff << 8,
		.div1_mask = 0x7f,
		.max_multiplier = 2047,
		.max_divider = 128,
		.min_divider = 1,
		.modes = (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	};

	of_ti_dpll_setup(node, &dpll_ck_ops, &dd, 0);
}
CLK_OF_DECLARE(ti_am3_dpll_clock, "ti,am3-dpll-clock", of_ti_am3_dpll_setup);

static void __init of_ti_am3_core_dpll_setup(struct device_node *node)
{
	const struct dpll_data dd = {
		.idlest_mask = 0x1,
		.enable_mask = 0x7,
		.autoidle_mask = 0x7,
		.mult_mask = 0x7ff << 8,
		.div1_mask = 0x7f,
		.max_multiplier = 2047,
		.max_divider = 128,
		.min_divider = 1,
		.modes = (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	};

	of_ti_dpll_setup(node, &dpll_core_ck_ops, &dd, 0);
}
CLK_OF_DECLARE(ti_am3_core_dpll_clock, "ti,am3-dpll-core-clock",
	       of_ti_am3_core_dpll_setup);
