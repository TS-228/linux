// SPDX-License-Identifier: GPL-2.0
/*
 * cgc.c - clock-gate controller
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 * Copyright (C) 2017 Cheng-Yu Lee <cylee12@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/bitops.h>
#include <linux/pm.h>
#include <linux/suspend.h>
#include "common.h"
#include "clk-mmio-gate.h"

#define MAX_CLOCK_GATES  32

struct cgc_desc {
	struct device *dev;
	unsigned int flags;

	/* name list */
	const char *names[MAX_CLOCK_GATES];
	int num_names;

	/* pm */
	unsigned int pm_mask;
	unsigned int pm_data;

	/* init data */
	struct clk_reg_init_data init_data;

	/* of clk provider */
	struct clk_hw_onecell_data *clk_data;
};

/* flags */
#define CGC_HW_USE_WRITE_EN  0x1

static inline int has_write_en(struct cgc_desc *cgcd)
{
	return !!(cgcd->flags & CGC_HW_USE_WRITE_EN);
}

static int init_clk_gates(struct cgc_desc *cgcd)
{
	int i;
	struct clk_hw_onecell_data *data;
	struct device *dev = cgcd->dev;
	struct device_node *np = dev->of_node;
	unsigned int flags = 0;
	unsigned int num_clks = cgcd->num_names;
	const char *name = NULL;
	int ret = 0;

	if (has_write_en(cgcd)) {
		flags |= CLK_MMIO_GATE_HAS_WRITE_EN;
		num_clks *= 2;
	}

	data = kzalloc(struct_size(data, hws, num_clks), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->num = num_clks;
	cgcd->clk_data = data;

	for (i = 0; i < cgcd->num_names; i++) {
		struct clk_mmio_gate *gate;
		int idx = has_write_en(cgcd) ? i * 2 : i;
		struct clk_init_data init = { 0 };

		name = cgcd->names[i];
		if (!name || !name[0])
			continue;

		/* setup clk init data */
		init.name = name;
		init.ops = &clk_mmio_gate_ops;
		init.flags = flags;
		if (of_clk_is_ignore_unused(np, name))
			init.flags |= CLK_IGNORE_UNUSED;

		gate = kzalloc(sizeof(*gate), GFP_KERNEL);
		if  (!gate) {
			ret = -ENOMEM;
			break;
		}

		/* setup init data for specific clk type */
		gate->bit_idx = idx;
		gate->base.hw.init = &init;
		gate->flags = flags;
		clk_reg_init(&gate->base, &cgcd->init_data);

		/* register clk */
		ret = clk_hw_register(NULL, &gate->base.hw);
		if (ret) {
			dev_err(dev, "%s: failed to register: %d\n",
				name, ret);
			kfree(gate);
			break;
		}
		data->hws[idx] = &gate->base.hw;

		dev_dbg(dev, "name-index=%d, clk-index=%d, clk-name=%s\n",
			i, idx, name);

		/* register clkdev */
		clk_hw_register_clkdev(&gate->base.hw, name, NULL);

		/* ignore in suspend */
		if (of_clk_is_ignore_pm(np, name))
			continue;
		cgcd->pm_mask |= BIT(idx);
	}

	if (ret)
		dev_err(dev, "failed to init %s(%d): %d\n", name, i, ret);

	return of_clk_add_hw_provider(np, of_clk_hw_onecell_get, data);
}

static int rtk_cgc_suspend(struct device *dev)
{
	struct cgc_desc *cgcd = dev_get_drvdata(dev);
	struct clk_reg clk_reg;

#ifdef CONFIG_SUSPEND
	if (pm_suspend_target_state == PM_SUSPEND_STANDBY)
		return 0;
#endif

	/* create a clk_reg for read/write reg */
	clk_reg_init(&clk_reg, &cgcd->init_data);
	cgcd->pm_data = clk_reg_read(&clk_reg, 0);

	return 0;
}

static int rtk_cgc_resume(struct device *dev)
{
	struct cgc_desc *cgcd = dev_get_drvdata(dev);
	struct clk_reg clk_reg;
	int mask;
	int val;

#ifdef CONFIG_SUSPEND
	if (pm_suspend_target_state == PM_SUSPEND_STANDBY)
		return 0;
#endif

	clk_reg_init(&clk_reg, &cgcd->init_data);

	mask = cgcd->pm_mask;
	val  = cgcd->pm_data;

	if (has_write_en(cgcd)) {
		mask |= cgcd->pm_mask << 1;
		val  |= cgcd->pm_mask << 1;
	}

	clk_reg_update(&clk_reg, 0, mask, val);

	return 0;
}

static const struct dev_pm_ops rtk_cgc_pm_ops = {
	.suspend_noirq = rtk_cgc_suspend,
	.resume_noirq = rtk_cgc_resume,
};

static int rtk_cgc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct cgc_desc *cgcd;
	void __iomem *reg = NULL;
	int ret;

	cgcd = devm_kzalloc(dev, sizeof(*cgcd), GFP_KERNEL);
	if (!cgcd)
		return -ENOMEM;

	reg = of_iomap(pdev->dev.of_node, 0);
	if (!reg) {
		ret = -EINVAL;
		goto error;
	}

	if (of_find_property(np, "has-write-en", NULL))
		cgcd->flags |= CGC_HW_USE_WRITE_EN;

	cgcd->num_names = of_property_count_strings(np, "clock-output-names");
	if (cgcd->num_names < 0)
		return cgcd->num_names;
	of_property_read_string_array(np, "clock-output-names",
				      cgcd->names, cgcd->num_names);

	cgcd->dev = dev;
	cgcd->init_data.reg = reg;

	platform_set_drvdata(pdev, cgcd);

	init_clk_gates(cgcd);
	dev_info(dev, "initialized");
	return 0;
error:
	if (reg)
		iounmap(reg);
	return ret;
}

static const struct of_device_id rtk_cgc_match[] = {
	{.compatible = "realtek,rtd1195-clock-gate-controller"},
	{}
};

static struct platform_driver rtk_cgc_driver = {
	.probe = rtk_cgc_probe,
	.driver = {
		.name = "rtk-cgc",
		.of_match_table = rtk_cgc_match,
		.pm = &rtk_cgc_pm_ops,
	},
};

static int __init rtk_cgc_init(void)
{
	return platform_driver_register(&rtk_cgc_driver);
}
core_initcall(rtk_cgc_init);
