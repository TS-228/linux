// SPDX-License-Identifier: GPL-2.0-only
/*
 * rtc-rtk.c - Realtek RTD1xxx SoC RTC
 *
 * Copyright (c) 2017 Realtek Semiconductor Corp.
 *
 * The counter is a plain day/hour/minute/half-second chain counting from
 * January 1st of the "rtc-base-year" given in the device tree. The alarm
 * block only stores day/hour/minute, so alarms have minute resolution.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/rtc.h>
#include <linux/spinlock.h>
#include <linux/time.h>

#define REG_RTCSEC		0x00
#define REG_RTCMIN		0x04
#define REG_RTCHR		0x08
#define REG_RTCDATE_LOW		0x0c
#define REG_RTCDATE_HIGH	0x10
#define REG_ALARMMIN		0x14
#define REG_ALARMHR		0x18
#define REG_ALARMDATE_LOW	0x1c
#define REG_ALARMDATE_HIGH	0x20
#define REG_RTCSTOP		0x24
#define REG_RTCACR		0x28
#define REG_RTCEN		0x2c
#define REG_RTCCR		0x30

/* Isolated (always-on) block, second reg range. */
#define REG_ISO_ISR		0x00
#define REG_ISO_RTC		0x34

#define ISO_ISR_RTC_ALARM	BIT(13)
#define ISO_RTC_ALARM_EN	BIT(0)

#define RTCACR_RTCPWR		BIT(7)	/* powers the RTC counter domain */
#define RTCSTOP_STOP		BIT(0)
#define RTCCR_RTCRST		BIT(6)
#define RTCEN_MAGIC		0x5a

#define RTC_SEC_MASK		0x7f	/* counts half seconds */
#define RTC_MIN_MASK		0x3f
#define RTC_HR_MASK		0x1f
#define RTC_DAY_HIGH_MASK	0x3f
#define RTC_DAY_MAX		16383

struct rtk_rtc {
	struct device *dev;
	struct rtc_device *rtc;
	void __iomem *base;
	void __iomem *iso_base;
	struct clk *clk;
	struct reset_control *rstc;
	time64_t base_secs;
	spinlock_t lock;
};

static void rtk_rtc_hw_enable(struct rtk_rtc *rtc, bool enable)
{
	writel(enable ? RTCEN_MAGIC : 0, rtc->base + REG_RTCEN);
}

/*
 * The day/hour/minute/second registers are read one by one, so a carry
 * happening mid-sequence would yield a bogus time. Bracket the sequence with
 * two reads of the seconds register and retry while it moves.
 */
static void rtk_rtc_read_counter(struct rtk_rtc *rtc, unsigned int *dayp,
				 unsigned int *hourp, unsigned int *minp,
				 unsigned int *secp)
{
	unsigned int day, hour, min, sec, sec2;
	int tries = 3;

	do {
		sec = readl(rtc->base + REG_RTCSEC) & RTC_SEC_MASK;
		min = readl(rtc->base + REG_RTCMIN) & RTC_MIN_MASK;
		hour = readl(rtc->base + REG_RTCHR) & RTC_HR_MASK;
		day = readl(rtc->base + REG_RTCDATE_LOW) & 0xff;
		day |= (readl(rtc->base + REG_RTCDATE_HIGH) &
			RTC_DAY_HIGH_MASK) << 8;
		sec2 = readl(rtc->base + REG_RTCSEC) & RTC_SEC_MASK;
	} while (sec != sec2 && --tries);

	*dayp = day;
	*hourp = hour;
	*minp = min;
	*secp = sec >> 1;
}

static int rtk_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct rtk_rtc *rtc = dev_get_drvdata(dev);
	unsigned int day, hour, min, sec;
	unsigned long flags;

	spin_lock_irqsave(&rtc->lock, flags);
	rtk_rtc_read_counter(rtc, &day, &hour, &min, &sec);
	spin_unlock_irqrestore(&rtc->lock, flags);

	rtc_time64_to_tm(rtc->base_secs +
			 ((day * 24 + hour) * 60 + min) * 60 + sec, tm);
	return 0;
}

static int rtk_rtc_split(struct rtk_rtc *rtc, time64_t secs, unsigned int *dayp,
			 unsigned int *hourp, unsigned int *minp,
			 unsigned int *secp)
{
	time64_t offset;
	u32 hms;

	if (secs < rtc->base_secs)
		return -EINVAL;

	offset = secs - rtc->base_secs;
	*dayp = div_u64_rem(offset, 86400, &hms);
	if (*dayp > RTC_DAY_MAX)
		return -EINVAL;

	*hourp = hms / 3600;
	*minp = hms % 3600 / 60;
	*secp = hms % 60;
	return 0;
}

static int rtk_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct rtk_rtc *rtc = dev_get_drvdata(dev);
	unsigned int day, hour, min, sec;
	unsigned long flags;
	int ret;

	ret = rtk_rtc_split(rtc, rtc_tm_to_time64(tm), &day, &hour, &min, &sec);
	if (ret)
		return ret;

	spin_lock_irqsave(&rtc->lock, flags);
	rtk_rtc_hw_enable(rtc, false);
	writel(sec * 2, rtc->base + REG_RTCSEC);
	writel(min, rtc->base + REG_RTCMIN);
	writel(hour, rtc->base + REG_RTCHR);
	writel(day & 0xff, rtc->base + REG_RTCDATE_LOW);
	writel((day >> 8) & RTC_DAY_HIGH_MASK, rtc->base + REG_RTCDATE_HIGH);
	rtk_rtc_hw_enable(rtc, true);
	spin_unlock_irqrestore(&rtc->lock, flags);

	return 0;
}

static void rtk_rtc_alarm_enable(struct rtk_rtc *rtc, bool enable)
{
	unsigned long flags;

	spin_lock_irqsave(&rtc->lock, flags);
	if (enable) {
		writel(ISO_ISR_RTC_ALARM, rtc->iso_base + REG_ISO_ISR);
		writel(ISO_RTC_ALARM_EN, rtc->iso_base + REG_ISO_RTC);
	} else {
		writel(0, rtc->iso_base + REG_ISO_RTC);
	}
	spin_unlock_irqrestore(&rtc->lock, flags);
}

static bool rtk_rtc_alarm_enabled(struct rtk_rtc *rtc)
{
	unsigned long flags;
	bool enabled;

	spin_lock_irqsave(&rtc->lock, flags);
	enabled = readl(rtc->iso_base + REG_ISO_RTC) & ISO_RTC_ALARM_EN;
	spin_unlock_irqrestore(&rtc->lock, flags);

	return enabled;
}

static int rtk_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtk_rtc *rtc = dev_get_drvdata(dev);
	unsigned int day, hour, min;
	unsigned long flags;

	spin_lock_irqsave(&rtc->lock, flags);
	min = readl(rtc->base + REG_ALARMMIN) & RTC_MIN_MASK;
	hour = readl(rtc->base + REG_ALARMHR) & RTC_HR_MASK;
	day = readl(rtc->base + REG_ALARMDATE_LOW) & 0xff;
	day |= (readl(rtc->base + REG_ALARMDATE_HIGH) &
		RTC_DAY_HIGH_MASK) << 8;
	spin_unlock_irqrestore(&rtc->lock, flags);

	rtc_time64_to_tm(rtc->base_secs + ((day * 24 + hour) * 60 + min) * 60,
			 &alrm->time);
	alrm->enabled = rtk_rtc_alarm_enabled(rtc);

	return 0;
}

static int rtk_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtk_rtc *rtc = dev_get_drvdata(dev);
	unsigned int day, hour, min, sec;
	unsigned long flags;
	int ret;

	ret = rtk_rtc_split(rtc, rtc_tm_to_time64(&alrm->time), &day, &hour,
			    &min, &sec);
	if (ret)
		return ret;

	rtk_rtc_alarm_enable(rtc, false);

	spin_lock_irqsave(&rtc->lock, flags);
	writel(min, rtc->base + REG_ALARMMIN);
	writel(hour, rtc->base + REG_ALARMHR);
	writel(day & 0xff, rtc->base + REG_ALARMDATE_LOW);
	writel((day >> 8) & RTC_DAY_HIGH_MASK, rtc->base + REG_ALARMDATE_HIGH);
	spin_unlock_irqrestore(&rtc->lock, flags);

	if (alrm->enabled)
		rtk_rtc_alarm_enable(rtc, true);

	return 0;
}

static int rtk_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	rtk_rtc_alarm_enable(dev_get_drvdata(dev), enabled);
	return 0;
}

static const struct rtc_class_ops rtk_rtc_ops = {
	.read_time = rtk_rtc_read_time,
	.set_time = rtk_rtc_set_time,
	.read_alarm = rtk_rtc_read_alarm,
	.set_alarm = rtk_rtc_set_alarm,
	.alarm_irq_enable = rtk_rtc_alarm_irq_enable,
};

static void rtk_rtc_dump(struct rtk_rtc *rtc)
{
	dev_info(rtc->dev,
		"sec=%02x min=%02x hr=%02x day=%02x%02x stop=%02x acr=%02x en=%02x cr=%02x\n",
		readl(rtc->base + REG_RTCSEC), readl(rtc->base + REG_RTCMIN),
		readl(rtc->base + REG_RTCHR),
		readl(rtc->base + REG_RTCDATE_HIGH),
		readl(rtc->base + REG_RTCDATE_LOW),
		readl(rtc->base + REG_RTCSTOP), readl(rtc->base + REG_RTCACR),
		readl(rtc->base + REG_RTCEN), readl(rtc->base + REG_RTCCR));
}

/*
 * RTCPWR powers the counter domain and survives across reboots, so a clear bit
 * means the block has just come up cold and the counter holds garbage.
 * RTCACR is also the only plain read/write register here (RTCEN is a magic-key
 * register and the counter registers are clocked by the 32kHz domain), so its
 * readback doubles as the liveness check for the whole block.
 */
static int rtk_rtc_power_on(struct rtk_rtc *rtc)
{
	unsigned long flags;
	bool cold;
	u32 acr;

	spin_lock_irqsave(&rtc->lock, flags);

	cold = !(readl(rtc->base + REG_RTCACR) & RTCACR_RTCPWR);
	if (cold)
		writel(readl(rtc->base + REG_RTCACR) | RTCACR_RTCPWR,
		       rtc->base + REG_RTCACR);

	acr = readl(rtc->base + REG_RTCACR);
	if (!(acr & RTCACR_RTCPWR)) {
		spin_unlock_irqrestore(&rtc->lock, flags);
		dev_err(rtc->dev,
			"cannot power on RTC (RTCACR reads 0x%02x); no 32kHz crystal fitted?\n",
			acr);
		return -ENODEV;
	}

	if (cold) {
		dev_info(rtc->dev, "cold start, resetting counter to base year\n");

		writel(RTCCR_RTCRST, rtc->base + REG_RTCCR);
		writel(0, rtc->base + REG_RTCCR);
		writel(0, rtc->base + REG_RTCMIN);
		writel(0, rtc->base + REG_RTCHR);
		writel(0, rtc->base + REG_RTCDATE_LOW);
		writel(0, rtc->base + REG_RTCDATE_HIGH);
		writel(readl(rtc->base + REG_RTCSTOP) & ~RTCSTOP_STOP,
		       rtc->base + REG_RTCSTOP);
	}

	spin_unlock_irqrestore(&rtc->lock, flags);

	return 0;
}

static void __iomem *rtk_rtc_map(struct platform_device *pdev, int index)
{
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, index);
	if (!res)
		return IOMEM_ERR_PTR(-ENXIO);

	/*
	 * Plain devm_ioremap(): both windows sit inside larger RBUS ranges
	 * that neighbouring nodes (timer0, iso gpio) also describe, so
	 * requesting the regions exclusively would fail.
	 */
	return devm_ioremap(&pdev->dev, res->start, resource_size(res)) ?:
		IOMEM_ERR_PTR(-ENOMEM);
}

static int rtk_rtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtk_rtc *rtc;
	u32 base_year = 2016;
	int ret;

	rtc = devm_kzalloc(dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	rtc->dev = dev;
	spin_lock_init(&rtc->lock);
	platform_set_drvdata(pdev, rtc);

	rtc->base = rtk_rtc_map(pdev, 0);
	if (IS_ERR(rtc->base))
		return dev_err_probe(dev, PTR_ERR(rtc->base),
				     "cannot map RTC registers\n");

	rtc->iso_base = rtk_rtc_map(pdev, 1);
	if (IS_ERR(rtc->iso_base))
		return dev_err_probe(dev, PTR_ERR(rtc->iso_base),
				     "cannot map ISO registers\n");

	of_property_read_u32(dev->of_node, "rtc-base-year", &base_year);
	rtc->base_secs = mktime64(base_year, 1, 1, 0, 0, 0);

	/*
	 * RTD119x gates both the clock and the reset of the RTC block; without
	 * these the registers read back as zero. RTD129x only has the gate.
	 */
	rtc->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(rtc->clk))
		return dev_err_probe(dev, PTR_ERR(rtc->clk),
				     "cannot get clock\n");

	rtc->rstc = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(rtc->rstc))
		return dev_err_probe(dev, PTR_ERR(rtc->rstc),
				     "cannot get reset control\n");

	ret = clk_prepare_enable(rtc->clk);
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable clock\n");

	ret = reset_control_deassert(rtc->rstc);
	if (ret) {
		dev_err(dev, "cannot deassert reset: %d\n", ret);
		goto err_clk;
	}

	ret = rtk_rtc_power_on(rtc);
	if (ret)
		goto err_clk;

	rtk_rtc_hw_enable(rtc, true);
	rtk_rtc_dump(rtc);

	rtc->rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(rtc->rtc)) {
		ret = PTR_ERR(rtc->rtc);
		goto err_clk;
	}

	rtc->rtc->ops = &rtk_rtc_ops;
	rtc->rtc->range_min = rtc->base_secs;
	rtc->rtc->range_max = rtc->base_secs + (RTC_DAY_MAX + 1) * 86400LL - 1;
	/* The alarm block has no seconds field and no interrupt line. */
	set_bit(RTC_FEATURE_ALARM_RES_MINUTE, rtc->rtc->features);
	rtc->rtc->uie_unsupported = 1;

	ret = devm_rtc_register_device(rtc->rtc);
	if (ret)
		goto err_clk;

	device_init_wakeup(dev, true);

	return 0;

err_clk:
	clk_disable_unprepare(rtc->clk);
	return ret;
}

static void rtk_rtc_remove(struct platform_device *pdev)
{
	struct rtk_rtc *rtc = platform_get_drvdata(pdev);

	/*
	 * Leave the block out of reset so it keeps counting; only drop the
	 * clock reference we took in probe.
	 */
	clk_disable_unprepare(rtc->clk);

	return;
}

static const struct of_device_id rtk_rtc_ids[] = {
	{ .compatible = "Realtek,rtk-rtc" },
	{ .compatible = "realtek,rtk-rtc" },
	{ .compatible = "Realtek,rtk119x-rtc" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rtk_rtc_ids);

static struct platform_driver rtk_rtc_driver = {
	.probe = rtk_rtc_probe,
	.remove = rtk_rtc_remove,
	.driver = {
		.name = "rtk-rtc",
		.of_match_table = rtk_rtc_ids,
	},
};
module_platform_driver(rtk_rtc_driver);

MODULE_DESCRIPTION("Realtek RTD1xxx SoC RTC");
MODULE_LICENSE("GPL");
