/*
 * rtk-cpufreq.c - Realtek Generic CPUFreq Dirver
 *
 * This dirver is based on original rtk-cpufreq.c and cpufreq-dt.c.
 * The freq_table and volt_table is replaced by operating-points-v2.
 *
 * Copyright (C) 2017-2018 Realtek Semiconductor Corporation
 * Copyright (C) 2017-2018 Cheng-Yu Lee <cylee12@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define pr_fmt(fmt) "cpufreq: " fmt

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpu_cooling.h>
#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>
#include <soc/realtek/rtk_chip.h>

/**
 * struct cpufreq_driver_data - an optional driver data, if this is set to
 *                              cpufreq_driver->driver_data, the device of
 *                              platform device will be used for providing
 *                              resource.
 *
 * @dev: passing device of platform device to cpufreq dirver
 */
struct cpufreq_driver_data {
	struct device *dev;
};

/********************************************************************
 *                 Realtek Generic Cpufreq Driver                   *
 ********************************************************************/
/*
 * struct rtk_cpufreq_priv - cpufreq internal data
 *
 * @opp_table: only exists when cpu regulator is installed successfully.
 * @cpu_clk:   cpu clk
 * @dev:       the devices where resoueces is from.
 * @cdev:      handle of cpu cooling device
 * @lock:      lock for set_target
 */
struct rtk_cpufreq_priv {
	int opp_token;
	int prop_token;
	struct device *dev;
	struct clk *cpu_clk;
	struct mutex lock;
	int inited;
};

#define CPUFREQ_GET_CLK                    BIT(0)
#define CPUFREQ_OPP_SET_REGULATOR          BIT(1)
#define CPUFREQ_OPP_OF_CPUMASK_ADD_TABLE   BIT(2)
#define CPUFREQ_OPP_OF_ADD_TABLE           BIT(3)
#define CPUFREQ_OPP_SET_PROP_NAME          BIT(4)
#define CPUFREQ_OPP_SET_SUPPORTED_HW       BIT(5)
#define CPUFREQ_OPP_INIT_CPUFREQ_TABLE     BIT(6)
#define CPUFREQ_REGISTER_POLICY_NOTIFIER   BIT(7)

static const char *find_supply_name(struct device *dev)
{
	struct device_node *np;
	struct property *pp;
	int cpu = dev->id;
	const char *name = NULL;

	np = of_node_get(dev->of_node);
	if (WARN_ON(!np))
		return NULL;

	if (!cpu) {
		pp = of_find_property(np, "cpu0-supply", NULL);
		if (pp) {
			name = "cpu0";
			goto node_put;
		}
	}

	pp = of_find_property(np, "cpu-supply", NULL);
	if (pp)
		name = "cpu";

node_put:
	of_node_put(np);
	return name;
}

static inline void set_inited(struct rtk_cpufreq_priv *priv, int act)
{
	priv->inited |= act;
}

static inline bool should_undo(struct rtk_cpufreq_priv *priv, int act)
{
	return !!(priv->inited & act);
}

static inline int rtk_cpufreq_set_prop_name(struct device *dev,
					    struct rtk_cpufreq_priv *priv)
{
	char buf[20];
	unsigned int val;
	int ret;

	val = get_rtd_chip_revision();
	if ((val & 0xFFF) == 0)
		return -EINVAL;

	snprintf(buf, sizeof(buf), "%x", val);

	dev_info(dev, "dev_pm_opp_set_prop_name(): prop_name=%s\n", buf);
	ret = dev_pm_opp_set_prop_name(dev, buf);
	if (ret < 0)
		return ret;

	priv->prop_token = ret;
	return 0;
}

static int rtk_cpufreq_target(struct cpufreq_policy *policy,
			      unsigned int index)
{
	struct rtk_cpufreq_priv *priv = policy->driver_data;
	struct device *dev = priv->dev;
	unsigned int target_freq;

	target_freq = policy->freq_table[index].frequency;
	dev_dbg(dev, "%s: target_freq=%u\n", __func__, target_freq);
	return dev_pm_opp_set_rate(dev, target_freq * 1000);
}

static int rtk_cpufreq_exit(struct cpufreq_policy *policy)
{
	struct rtk_cpufreq_priv *priv = policy->driver_data;
	struct device *dev = priv->dev;

	if (should_undo(priv, CPUFREQ_OPP_OF_ADD_TABLE))
		dev_pm_opp_of_remove_table(dev);
	if (should_undo(priv, CPUFREQ_OPP_INIT_CPUFREQ_TABLE))
		dev_pm_opp_free_cpufreq_table(dev, &policy->freq_table);
	if (should_undo(priv, CPUFREQ_OPP_OF_CPUMASK_ADD_TABLE))
		dev_pm_opp_of_cpumask_remove_table(policy->related_cpus);
	if (should_undo(priv, CPUFREQ_OPP_SET_PROP_NAME))
		dev_pm_opp_put_prop_name(priv->prop_token);
	if (should_undo(priv, CPUFREQ_OPP_SET_REGULATOR))
		dev_pm_opp_put_regulators(priv->opp_token);
	if (should_undo(priv, CPUFREQ_GET_CLK))
		clk_put(policy->clk);
	kfree(priv);

	return 0;
}

static int rtk_cpufreq_init(struct cpufreq_policy *policy)
{
	struct cpufreq_driver_data *data = cpufreq_get_driver_data();
	struct device *dev;
	struct device *cpu_dev;
	struct rtk_cpufreq_priv *priv;
	struct cpufreq_frequency_table *freq_table;
	unsigned int transition_latency;
	unsigned long suspend_freq;
	const char *name;
	const char *reg_name[] = { NULL, NULL };
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	policy->driver_data = priv;

	/* get cpu device */
	cpu_dev = get_cpu_device(policy->cpu);
	if (!cpu_dev) {
		ret = -ENODEV;
		pr_err("failed to get cpu%d device\n", policy->cpu);
		goto error;
	}

	/*
	 * In xen, only cpu node in dom0 can get resource from original dt,
	 * so use the node of platfrom device for rest of doamins.
	 */
	if (data && data->dev)
		dev = data->dev;
	else
		dev = cpu_dev;
	dev_notice(dev, "used in cpufreq\n");

	/* get cpu clk */
	priv->cpu_clk = clk_get(dev, NULL);
	if (IS_ERR(priv->cpu_clk)) {
		ret = PTR_ERR(priv->cpu_clk);
		dev_err(dev, "clk_get() returns %d\n", ret);
		goto error;
	}
	policy->clk = priv->cpu_clk;
	set_inited(priv, CPUFREQ_GET_CLK);

	name = find_supply_name(cpu_dev);
	if (name) {
		reg_name[0] = name;
		ret = dev_pm_opp_set_regulators(cpu_dev, reg_name);
		if (ret < 0) {
			dev_warn(dev, "continue without regulator\n");
		} else {
			priv->opp_token = ret;
			set_inited(priv, CPUFREQ_OPP_SET_REGULATOR);
		}
	}

	priv->dev = dev;

	ret = rtk_cpufreq_set_prop_name(dev, priv);
	if (!ret)
		set_inited(priv, CPUFREQ_OPP_SET_PROP_NAME);

	if (cpu_dev == dev) {
		/* set cpu mask */
		ret = dev_pm_opp_of_get_sharing_cpus(dev, policy->cpus);
		if (ret) {
			dev_err(dev, "dev_pm_opp_of_get_sharing_cpus() returns %d\n",
				ret);
			goto error;
		}

		/* add opps from dt */
		ret = dev_pm_opp_of_cpumask_add_table(policy->cpus);
		if (ret) {
			dev_err(dev, "dev_pm_opp_of_cpumask_add_table() returns %d\n",
				ret);
			goto error;
		}
		set_inited(priv, CPUFREQ_OPP_OF_CPUMASK_ADD_TABLE);
	} else {
		/* set cpu mask */
		cpumask_setall(policy->cpus);

		/* add opps from dt */
		ret = dev_pm_opp_of_add_table(dev);
		if (ret) {
			dev_err(dev, "dev_pm_opp_of_add_table() returns %d\n",
				ret);
			goto error;
		}
		set_inited(priv, CPUFREQ_OPP_OF_ADD_TABLE);
	}

	/* if number of opps is less equal than zero, retry later */
	ret = dev_pm_opp_get_opp_count(dev);
	if (ret <= 0) {
		ret = -EPROBE_DEFER;
		dev_dbg(dev, "OPP table is not ready, deferring probe\n");
		goto error;
	}

	/* create cpufreq table */
	ret = dev_pm_opp_init_cpufreq_table(dev, &freq_table);
	if (ret) {
		dev_err(dev, "dev_pm_opp_init_cpufreq_table() returns %d\n",
			ret);
		goto error;
	}
	set_inited(priv, CPUFREQ_OPP_INIT_CPUFREQ_TABLE);

	/* get suspend opp */
	suspend_freq = dev_pm_opp_get_suspend_opp_freq(dev);
	if (suspend_freq)
		policy->suspend_freq = suspend_freq / 1000;

	policy->freq_table = freq_table;
	ret = cpufreq_table_validate_and_sort(policy);
	if (ret) {
		dev_err(dev, "cpufreq_table_validate_and_sort() returns %d\n",
			ret);
		goto error;
	}

	transition_latency = dev_pm_opp_get_max_transition_latency(dev);
	if (!transition_latency)
		transition_latency = 500000;
	policy->cpuinfo.transition_latency = transition_latency;
	dev_dbg(dev, "transition_latency = %d\n", transition_latency);

	return 0;
error:
	rtk_cpufreq_exit(policy);
	return ret;
}

enum {
	RTK_CPUFREQ_PM_UNDEFINED = 0,
	RTK_CPUFREQ_PM_SUSPEND,
	RTK_CPUFREQ_PM_STANDBY,
	RTK_CPUFREQ_PM_SHUTDOWN
};

static int dvfs_pm_state;

static int rtk_cpufreq_suspend(struct cpufreq_policy *policy)
{
	struct rtk_cpufreq_priv *priv = policy->driver_data;
	struct device *dev = priv->dev;
	unsigned int saved_freq = policy->suspend_freq;
	int ret;

	dev_info(dev, "Enter %s\n", __func__);

	if (dvfs_pm_state == RTK_CPUFREQ_PM_UNDEFINED) {
		dvfs_pm_state = RTK_CPUFREQ_PM_SUSPEND;
#ifdef CONFIG_SUSPEND
		if (RTK_PM_STATE == PM_SUSPEND_STANDBY)
			dvfs_pm_state = RTK_CPUFREQ_PM_STANDBY;
#endif
	}

	switch (dvfs_pm_state) {
	default:
		dev_err(dev, "unknown pm state\n");
		break;
	case RTK_CPUFREQ_PM_SUSPEND:
	case RTK_CPUFREQ_PM_SHUTDOWN:
		break;
	case RTK_CPUFREQ_PM_STANDBY:
		policy->suspend_freq = policy->min;
		break;
	}

	if (WARN_ON(!policy->suspend_freq))
		return -EINVAL;

	dev_err(dev, "set freq to %d MHz\n", policy->suspend_freq / 1000);
	ret = cpufreq_generic_suspend(policy);

	policy->suspend_freq = saved_freq;
	dvfs_pm_state = RTK_CPUFREQ_PM_UNDEFINED;

	dev_info(dev, "Exit %s\n", __func__);
	return ret;
}

static struct cpufreq_driver rtk_cpufreq_driver = {
	.name         = "rtk-cpufreq",
	.flags        = CPUFREQ_IS_COOLING_DEV,
	.attr         = cpufreq_generic_attr,
	.verify       = cpufreq_generic_frequency_table_verify,
	.get          = cpufreq_generic_get,
	.target_index = rtk_cpufreq_target,
	.init         = rtk_cpufreq_init,
	.exit         = rtk_cpufreq_exit,
	.suspend      = rtk_cpufreq_suspend,
};

static int resources_available(void)
{
	struct device *cpu_dev;
	struct regulator *cpu_reg;
	struct clk *cpu_clk;
	int ret = 0;

	cpu_dev = get_cpu_device(0);
	if (!cpu_dev) {
		pr_err("failed to get cpu0 device\n");
		return -ENODEV;
	}

	cpu_clk = clk_get(cpu_dev, NULL);
	ret = PTR_ERR_OR_ZERO(cpu_clk);
	if (ret) {
		if (ret == -EPROBE_DEFER)
			dev_dbg(cpu_dev, "clock not ready, retry\n");
		else
			dev_err(cpu_dev, "failed to get clock: %d\n", ret);

		return ret;
	}
	clk_put(cpu_clk);

	cpu_reg = regulator_get_optional(cpu_dev, "cpu");
	ret = PTR_ERR_OR_ZERO(cpu_reg);
	if (ret) {
		if (ret == -EPROBE_DEFER)
			dev_dbg(cpu_dev, "cpu0 regulator not ready, retry\n");
		else {
			dev_dbg(cpu_dev, "no regulator for cpu0: %d\n", ret);
			ret = 0;
		}
		return ret;
	}
	regulator_put(cpu_reg);

	return 0;
}

static int rtk_dvfs_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	ret = resources_available();
	if (ret)
		return ret;

	ret = cpufreq_register_driver(&rtk_cpufreq_driver);
	if (ret) {
		dev_err(dev, "cpufreq_register_driver() returns %d\n", ret);
		return ret;
	}
	dev_info(dev, "initialized\n");
	return 0;
}

static void rtk_dvfs_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	cpufreq_unregister_driver(&rtk_cpufreq_driver);
	dev_info(dev, "removed\n");
	return;
}

static void rtk_dvfs_shutdown(struct platform_device *pdev)
{
	/*
	 * set the inernal pm_state, the cpufreq suspend
	 * flow will do rest of it
	 */
	dvfs_pm_state = RTK_CPUFREQ_PM_SHUTDOWN;
}

static const struct of_device_id rtk_dvfs_ids[] = {
	{.compatible = "realtek,rtd129x-dvfs"},
	{.compatible = "realtek,rtd139x-dvfs"},
	{.compatible = "realtek,cpu-dvfs"},
	{ /* Sentinel */ }
};

static struct platform_driver rtk_dvfs_drv = {
	.driver = {
		.name           = "rtk-cpufreq",
		.of_match_table = rtk_dvfs_ids,
	},
	.probe    = rtk_dvfs_probe,
	.remove   = rtk_dvfs_remove,
	.shutdown = rtk_dvfs_shutdown,
};
module_platform_driver(rtk_dvfs_drv);
