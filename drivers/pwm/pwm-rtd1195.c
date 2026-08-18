// SPDX-License-Identifier: GPL-2.0
/*
 * RTD1195 pulse-width-modulation controller driver
 *
 * Copyright (C) 2014 Realtek Semiconductor Corporation
 */

#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/spinlock.h>

#define RTK_ADDR_PWM_OCD	0x0
#define RTK_ADDR_PWM_CD		0x4
#define RTK_ADDR_PWM_CSD	0x8

#define RTK_PWM_OCD_SHIFT	8
#define RTK_PWM_CD_SHIFT	8
#define RTK_PWM_CSD_SHIFT	4

#define RTK_PWM_OCD_MASK	GENMASK(7, 0)
#define RTK_PWM_CD_MASK		GENMASK(7, 0)
#define RTK_PWM_CSD_MASK	GENMASK(3, 0)

#define NUM_PWM			4

struct rtd1195_pwm_map {
	int duty_rate;
	int ocd;
	int cd;
};

static const struct rtd1195_pwm_map rtd1195_pwm_maps[] = {
	{ 100, 1, 1 },
	{  80, 4, 3 },
	{  75, 3, 2 },
	{  66, 2, 1 },
	{  60, 4, 2 },
	{  50, 3, 1 },
	{  40, 4, 1 },
	{  33, 5, 1 },
	{  25, 3, 0 },
	{  20, 3, 0 },
	{   0, 0, 0 },
};

struct rtd1195_pwm_chip {
	void __iomem *base;
	spinlock_t lock;
	u32 clksrc_div[NUM_PWM];
	struct pwm_state state[NUM_PWM];
};

static inline struct rtd1195_pwm_chip *to_rtd1195_pwm_chip(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static int rtd1195_pwm_map_lookup(int duty_rate)
{
	int i;

	if (duty_rate >= 100)
		return 0;
	if (duty_rate <= 0)
		return ARRAY_SIZE(rtd1195_pwm_maps) - 1;

	for (i = 0; i < ARRAY_SIZE(rtd1195_pwm_maps) - 1; i++) {
		if (duty_rate <= rtd1195_pwm_maps[i].duty_rate &&
		    duty_rate > rtd1195_pwm_maps[i + 1].duty_rate)
			return i;
	}

	return ARRAY_SIZE(rtd1195_pwm_maps) - 1;
}

static void rtd1195_pwm_update_hw(struct rtd1195_pwm_chip *pc, unsigned int hwpwm)
{
	const struct pwm_state *state = &pc->state[hwpwm];
	const struct rtd1195_pwm_map *map;
	u32 val;
	unsigned long flags;
	int map_idx, duty_rate;

	spin_lock_irqsave(&pc->lock, flags);

	if (state->enabled) {
		duty_rate = div64_u64(state->duty_cycle * 100, state->period);
		map_idx = rtd1195_pwm_map_lookup(duty_rate);
		map = &rtd1195_pwm_maps[map_idx];

		val = readl(pc->base + RTK_ADDR_PWM_OCD);
		val &= ~(RTK_PWM_OCD_MASK << (hwpwm * RTK_PWM_OCD_SHIFT));
		val |= map->ocd << (hwpwm * RTK_PWM_OCD_SHIFT);
		writel(val, pc->base + RTK_ADDR_PWM_OCD);

		val = readl(pc->base + RTK_ADDR_PWM_CD);
		val &= ~(RTK_PWM_CD_MASK << (hwpwm * RTK_PWM_CD_SHIFT));
		val |= map->cd << (hwpwm * RTK_PWM_CD_SHIFT);
		writel(val, pc->base + RTK_ADDR_PWM_CD);

		val = readl(pc->base + RTK_ADDR_PWM_CSD);
		val &= ~(RTK_PWM_CSD_MASK << (hwpwm * RTK_PWM_CSD_SHIFT));
		val |= pc->clksrc_div[hwpwm] << (hwpwm * RTK_PWM_CSD_SHIFT);
		writel(val, pc->base + RTK_ADDR_PWM_CSD);
	} else {
		val = readl(pc->base + RTK_ADDR_PWM_OCD);
		val &= ~(RTK_PWM_OCD_MASK << (hwpwm * RTK_PWM_OCD_SHIFT));
		writel(val, pc->base + RTK_ADDR_PWM_OCD);

		val = readl(pc->base + RTK_ADDR_PWM_CD);
		val &= ~(RTK_PWM_CD_MASK << (hwpwm * RTK_PWM_CD_SHIFT));
		writel(val, pc->base + RTK_ADDR_PWM_CD);

		val = readl(pc->base + RTK_ADDR_PWM_CSD);
		val &= ~(RTK_PWM_CSD_MASK << (hwpwm * RTK_PWM_CSD_SHIFT));
		writel(val, pc->base + RTK_ADDR_PWM_CSD);
	}

	spin_unlock_irqrestore(&pc->lock, flags);
}

static int rtd1195_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			     const struct pwm_state *state)
{
	struct rtd1195_pwm_chip *pc = to_rtd1195_pwm_chip(chip);

	if (state->enabled && !state->period)
		return -EINVAL;

	pc->state[pwm->hwpwm] = *state;
	rtd1195_pwm_update_hw(pc, pwm->hwpwm);

	return 0;
}

static int rtd1195_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				 struct pwm_state *state)
{
	struct rtd1195_pwm_chip *pc = to_rtd1195_pwm_chip(chip);

	*state = pc->state[pwm->hwpwm];
	return 0;
}

static const struct pwm_ops rtd1195_pwm_ops = {
	.apply = rtd1195_pwm_apply,
	.get_state = rtd1195_pwm_get_state,
};

static int rtd1195_pwm_probe(struct platform_device *pdev)
{
	struct pwm_chip *chip;
	struct rtd1195_pwm_chip *pc;
	struct device *dev = &pdev->dev;
	u32 clksrc_div = 8;
	int i, ret;

	chip = devm_pwmchip_alloc(dev, NUM_PWM, sizeof(*pc));
	if (IS_ERR(chip))
		return PTR_ERR(chip);

	pc = pwmchip_get_drvdata(chip);
	pc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pc->base))
		return PTR_ERR(pc->base);

	of_property_read_u32(dev->of_node, "clksrc_div", &clksrc_div);

	spin_lock_init(&pc->lock);

	for (i = 0; i < NUM_PWM; i++) {
		pc->clksrc_div[i] = clksrc_div;
		pc->state[i].period = 40000;
		pc->state[i].duty_cycle = 0;
		pc->state[i].enabled = false;
	}

	chip->ops = &rtd1195_pwm_ops;

	ret = devm_pwmchip_add(dev, chip);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add PWM chip\n");

	return 0;
}

static const struct of_device_id rtd1195_pwm_of_match[] = {
	{ .compatible = "realtek,rtd1195-pwm" },
	{ .compatible = "Realtek,rtd1195-pwm" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtd1195_pwm_of_match);

static struct platform_driver rtd1195_pwm_driver = {
	.driver = {
		.name = "pwm-rtd1195",
		.of_match_table = rtd1195_pwm_of_match,
	},
	.probe = rtd1195_pwm_probe,
};
module_platform_driver(rtd1195_pwm_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Realtek Semiconductor Corporation");
MODULE_DESCRIPTION("RTD1195 PWM controller driver");
