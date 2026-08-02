/*
 * sensor.c - Realtek generic thermal sensor driver
 *
 * Copyright (C) 2017-2018 Realtek Semiconductor Corporation
 * Copyright (C) 2017-2018 Cheng-Yu Lee <cylee12@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/module.h>
#include "../thermal_core.h"
#include "sensor.h"

static LIST_HEAD(sensor_device_list);
static const struct of_device_id thermal_sensor_of_match[];

static int thermal_sensor_get_temp(struct thermal_zone_device *tz, int *temp);
static int thermal_sensor_get_trend(struct thermal_zone_device *tz, int trip,
				    enum thermal_trend *trend);

static const struct thermal_zone_device_ops thermal_sensor_ops = {
	.get_temp  = thermal_sensor_get_temp,
	.get_trend = thermal_sensor_get_trend,
};

static int thermal_sensor_device_add(struct device *dev,
	struct thermal_sensor_device *tdev,
	const struct thermal_sensor_desc *desc)
{
	tdev->dev = dev;
	tdev->desc = desc;

	tdev->sensors = devm_kcalloc(dev, desc->num_sensors,
		sizeof(*tdev->sensors), GFP_KERNEL);
	if (!tdev->sensors)
		return -ENOMEM;

	thermal_sensor_hw_init(tdev);
	thermal_sensor_hw_reset(tdev);

	tdev->tz = devm_thermal_of_zone_register(dev, 0, tdev,
						 &thermal_sensor_ops);
	if (IS_ERR(tdev->tz))
		return PTR_ERR(tdev->tz);

	list_add(&tdev->list, &sensor_device_list);
	tdev->passive_delay = jiffies_to_msecs(tdev->tz->passive_delay_jiffies);
	tdev->polling_delay = jiffies_to_msecs(tdev->tz->polling_delay_jiffies);
	return 0;
}

void thermal_sensor_device_remove(struct thermal_sensor_device *tdev)
{
	list_del(&tdev->list);
	devm_thermal_of_zone_unregister(tdev->dev, tdev->tz);
	thermal_sensor_hw_exit(tdev);
}

static int thermal_sensor_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct thermal_sensor_device *tdev = tz->devdata;
	int ret;

	ret = thermal_sensor_hw_get_temp(tdev, temp);
	dev_dbg(tdev->dev, "temp=%d\n", *temp);
	return ret;
}

static int thermal_sensor_get_trend(struct thermal_zone_device *tz, int trip,
				    enum thermal_trend *trend)
{
	struct thermal_sensor_device *tdev = tz->devdata;
	const struct thermal_trip *trips;
	int delta;

	if (!of_thermal_is_trip_valid(tz, trip))
		return -EINVAL;

	trips = of_thermal_get_trip_points(tz);
	delta = tz->temperature - trips[trip].temperature;

	if (-500 < delta && delta < 500)
		*trend = THERMAL_TREND_STABLE;
	else if (delta > 0)
		*trend = THERMAL_TREND_RAISING;
	else
		*trend = THERMAL_TREND_DROPPING;

	dev_dbg(tdev->dev, "trip%d, delta=%d, trend=%d\n", trip,
		delta, *trend);
	return 0;
}

static int thermal_sensor_resume(struct device *dev)
{
	struct thermal_sensor_device *tdev = dev_get_drvdata(dev);

	dev_info(dev, "Enter %s\n", __func__);
	thermal_sensor_hw_reset(tdev);
	dev_info(dev, "Exit %s\n", __func__);
	return 0;
}

static const struct dev_pm_ops thermal_sensor_pm_ops = {
	.resume = thermal_sensor_resume,
};

static int thermal_sensor_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct thermal_sensor_device *tdev;
	int ret = 0;
	const struct of_device_id *id;
	const struct thermal_sensor_desc *desc;

	id = of_match_node(thermal_sensor_of_match, np);
	if (id)
		desc = id->data;

	if (!id || !desc)
		return -EINVAL;

	tdev = devm_kzalloc(dev, sizeof(*tdev), GFP_KERNEL);
	if (!tdev)
		return -ENOMEM;

	ret = thermal_sensor_device_add(dev, tdev, desc);
	if (ret)
		dev_err(dev, "thermal_sensor_add() returns %d\n", ret);
	platform_set_drvdata(pdev, tdev);
	dev_info(dev, "initialized\n");
	return 0;
}

static int thermal_sensor_remove(struct platform_device *pdev)
{
	struct thermal_sensor_device *tdev = platform_get_drvdata(pdev);

	thermal_sensor_device_remove(tdev);
	platform_set_drvdata(pdev, NULL);
	dev_info(&pdev->dev, "removed\n");
	return 0;
}

static void thermal_sensor_shutdown(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct thermal_sensor_device *tdev = platform_get_drvdata(pdev);

	dev_info(dev, "%s: stop tz_dev\n", __func__);
	thermal_zone_device_disable(tdev->tz);
};

static const struct of_device_id thermal_sensor_of_match[] = {
#ifdef CONFIG_RTK_THERMAL_RTD119X
	{
		.compatible = "realtek,rtd119x-thermal-sensor",
		.data = &rtd119x_sensor_desc,
	},
#endif
#ifdef CONFIG_RTK_THERMAL_RTD129X
	{
		.compatible = "realtek,rtd129x-thermal-sensor",
		.data = &rtd129x_sensor_desc,
	},
#endif
#ifdef CONFIG_RTK_THERMAL_RTD139X
	{
		.compatible = "realtek,rtd139x-thermal-sensor",
		.data = &rtd139x_sensor_desc,
	},
#endif
#ifdef CONFIG_RTK_THERMAL_RTD16XX
	{
		.compatible = "realtek,rtd16xx-thermal-sensor",
		.data = &rtd16xx_sensor_desc,
	},
#endif
	{}
};

static struct platform_driver thermal_sensor_drv = {
	.driver = {
		.name           = "rtk-thermal-sensor",
		.of_match_table = of_match_ptr(thermal_sensor_of_match),
		.pm             = &thermal_sensor_pm_ops,
	},
	.probe    = thermal_sensor_probe,
	.remove   = thermal_sensor_remove,
	.shutdown = thermal_sensor_shutdown,
};
module_platform_driver(thermal_sensor_drv);
