// SPDX-License-Identifier: GPL-2.0
/*
 * Realtek RTD1195 timer driver
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 */

#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/sched_clock.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/clk.h>

#define TIMER0 0
#define TIMER1 1
#define TIMER2 2
#define TIMER_MAX (TIMER2 + 1)

#define SYSTEM_TIMER TIMER0

/*
 * Smallest match this driver will programme, in counter ticks. The match is
 * written relative to a freshly read counter value, and the read, the add and
 * the write are several accesses across a slow register bus, so anything much
 * tighter than this is missed as often as it is met. The vendor's own driver
 * for this silicon settled on the same figure, about 185us at 27MHz.
 */
#define RTK_TIMER_MIN_DELTA	5000

/*
 * Largest match, in counter ticks. The counter is 32 bits and "has the match
 * gone past" is decided by the sign of the difference, which only means
 * anything for less than half the range. Cap at exactly that.
 */
#define RTK_TIMER_MAX_DELTA	0x7fffffff

#define COUNTER 0
#define TIMER 1

#define TR_EN BIT(31)
#define TR_MODE BIT(30)
#define TR_PAUSE BIT(29)
#define TR_INT_EN BIT(31)

#define MISC_OFFSET 0x0001B000
#define UMSK_ISR_OFFSET 0x00000008
#define ISR_OFFSET 0x0000000C
#define UMSK_ISR_SWC 0x00000010
#define ISR_SWC 0x00000014
#define SETTING_SWC 0x00000018
#define FAST_INT_EN_0 0x0000001C
#define FAST_INT_EN_1 0x00000020
#define FAST_ISR 0x00000024
#define MISC_DBG 0x0000002C
#define MISC_DUMMY 0x00000030

#define TCTVR_OFFSET 0x00000500
#define TCCVR_OFFSET 0x0000050C
#define TCCR_OFFSET 0x00000518
#define TCICR_OFFSET 0x00000524

#define RTK_TIMER_HZ CONFIG_HZ

/* HW timer command enum description */
enum hwtimer_commands {
	HWT_START = 0x80, /* Start a timer/counter */
	HWT_STOP, /* Stop a timer/counter */
	HWT_PAUSE, /* Pause a timer/counter */
	HWT_RESUME, /* Restart a timer/counter */
	HWT_INT_ENABLE, /* Enable timer/counter interrupt */
	HWT_INT_DISABLE, /* Disable timer/counter interrupt */
	HWT_INT_CLEAR, /* Clear timer/counter interrupt pending bit */
};

int TC_SHIFT[2] = {
	(1 << 6),
	(1 << 7),
};

struct _suspend_data {
	unsigned int value;
	unsigned char mode;
};

struct rtk_clock_event_device {
	int index;
	struct clock_event_device *evt;
	struct irqaction *irq_action;
	bool registered;
};

static struct _suspend_data timer_suspend_data[TIMER_MAX];

static void __iomem *timer_base;
unsigned long clk_freq;
static bool rtk_cs_registered;

#define MISC_BASE(pa) (timer_base + (pa))

static inline void rtk_setbits(void __iomem *offset, u32 mask)
{
	__raw_writel(__raw_readl(offset) | mask, offset);
}

static inline void rtk_clearbits(void __iomem *offset, u32 mask)
{
	__raw_writel(__raw_readl(offset) & ~mask, offset);
}

/*
 * The per-timer registers are indexed by id, but the interrupt status and
 * unmask registers are shared by every timer in the block. TIMER0 and TIMER1
 * are the tick devices of different CPUs, so those two registers are reached
 * concurrently and must not be updated with a plain read-modify-write.
 */
static DEFINE_RAW_SPINLOCK(rtk_timer_lock);

static unsigned char rtk_timer_get_mode(unsigned char id)
{
	unsigned int reg = __raw_readl(MISC_BASE(TCCR_OFFSET + (id << 2)));

	return (reg & TR_MODE) ? TIMER : COUNTER;
}

static int rtk_timer_control(unsigned char id, unsigned int cmd)
{
	switch (cmd) {
	case HWT_INT_CLEAR:
		if (id < ARRAY_SIZE(TC_SHIFT)) {
			unsigned long flags;

			raw_spin_lock_irqsave(&rtk_timer_lock, flags);
			rtk_setbits(MISC_BASE(UMSK_ISR_OFFSET),
				    TC_SHIFT[id] | 0x1);
			raw_spin_unlock_irqrestore(&rtk_timer_lock, flags);
		}
		break;
	case HWT_START:
		rtk_setbits(MISC_BASE(TCCR_OFFSET + (id << 2)), TR_EN);
		if (id < ARRAY_SIZE(TC_SHIFT))
			__raw_writel(TC_SHIFT[id], MISC_BASE(ISR_OFFSET));
		break;
	case HWT_STOP:
		rtk_clearbits(MISC_BASE(TCCR_OFFSET + (id << 2)), TR_EN);
		break;
	case HWT_PAUSE:
		rtk_setbits(MISC_BASE(TCCR_OFFSET + (id << 2)), TR_PAUSE);
		break;
	case HWT_RESUME:
		rtk_clearbits(MISC_BASE(TCCR_OFFSET + (id << 2)), TR_PAUSE);
		break;
	case HWT_INT_ENABLE:
		rtk_setbits(MISC_BASE(TCICR_OFFSET + (id << 2)), TR_INT_EN);
		break;
	case HWT_INT_DISABLE:
		rtk_clearbits(MISC_BASE(TCICR_OFFSET + (id << 2)), TR_INT_EN);
		break;
	default:
		return 1;
	}
	return 0;
}

static int rtk_timer_get_value(unsigned char id)
{
	/* get the current timer's value */
	return __raw_readl(MISC_BASE(TCCVR_OFFSET + (id << 2)));
}

static int rtk_timer_set_value(unsigned char id, unsigned int value)
{
	/* set the timer's initial value */
	__raw_writel(value, MISC_BASE(TCCVR_OFFSET + (id << 2)));
	return 0;
}

static int rtk_timer_set_target(unsigned char id, unsigned int value)
{
	/* set the timer's initial value */
	__raw_writel(value, MISC_BASE(TCTVR_OFFSET + (id << 2)));
	return 0;
}

static int rtk_timer_set_mode(unsigned char id, unsigned char mode)
{
	switch (mode) {
	case COUNTER:
		rtk_clearbits((MISC_BASE(TCCR_OFFSET + (id << 2))), TR_MODE);
		break;
	case TIMER:
		rtk_setbits((MISC_BASE(TCCR_OFFSET + (id << 2))), TR_MODE);
		break;
	default:
		return 1;
	}

	return 0;
}

static irqreturn_t rtk_clock_event_isr(int irq, void *dev_id)
{
	struct rtk_clock_event_device *clkevt;
	struct clock_event_device *evt;
	int nr = 0;

	clkevt = (struct rtk_clock_event_device *)dev_id;
	evt = clkevt->evt;
	nr = clkevt->index;

	/* write-1-to-clear: only ever write our own bit */
	__raw_writel(TC_SHIFT[nr], MISC_BASE(ISR_OFFSET));

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static int rtk_clkevt_set_next(int nr, unsigned long cycles,
			       struct clock_event_device *evt)
{
	u32 target;
	int i;

	/*
	 * Free-running up-counter with a match register. A match that has
	 * already gone past is not merely late: nothing fires until the
	 * 32-bit counter wraps, 159s at 27MHz.
	 *
	 * Reading the counter, adding to it and writing the match is several
	 * accesses across a slow register bus, so a short delta can expire
	 * while it is still being programmed. Reporting -ETIME and leaving it
	 * to the caller is not enough: clockevents_program_min_delta() retries
	 * three times, then raises min_delta and returns -ETIME, and
	 * tick_program_event() discards that, so the event is simply dropped.
	 * On an idle CPU that means it never wakes again.
	 *
	 * Re-arm here instead, with more room each time round, until the match
	 * really is in the future. Firing a little late beats not firing.
	 */
	if (cycles < RTK_TIMER_MIN_DELTA)
		cycles = RTK_TIMER_MIN_DELTA;

	rtk_timer_control(nr, HWT_INT_ENABLE);

	for (i = 0; i < 16; i++) {
		target = rtk_timer_get_value(nr) + cycles;
		rtk_timer_set_target(nr, target);

		if ((s32)(rtk_timer_get_value(nr) - target) < 0)
			return 0;

		cycles *= 2;
	}

	return -ETIME;
}

static void rtk_clocksource_suspend(struct clocksource *cs)
{
	pr_info("[RTK-TIMER] Enter %s\n", __func__);
	timer_suspend_data[TIMER2].value = rtk_timer_get_value(TIMER2);
	pr_info("[RTK-TIMER] Exit %s\n", __func__);
}

static void rtk_clocksource_resume(struct clocksource *cs)
{
	pr_info("[RTK-TIMER] Enter %s\n", __func__);
	rtk_timer_control(TIMER2, HWT_STOP);
	rtk_timer_set_value(TIMER2, timer_suspend_data[TIMER2].value);
	rtk_timer_set_mode(TIMER2, COUNTER);
	rtk_timer_control(TIMER2, HWT_START);
	pr_info("[RTK-TIMER] Exit %s\n", __func__);
}

static int rtk_timer0_set_next(unsigned long cycles,
			       struct clock_event_device *evt)
{
	return rtk_clkevt_set_next(TIMER0, cycles, evt);
}

static int rtk_tm0_set_state_periodic(struct clock_event_device *cev)
{
	int nr = 0;

	pr_info("[RTK-TIMER%d] set mode: periodic\n", nr);
	rtk_timer_control(nr, HWT_INT_DISABLE);
	rtk_timer_control(nr, HWT_STOP);
	rtk_timer_set_value(nr, timer_suspend_data[nr].value);
	rtk_timer_set_target(nr, DIV_ROUND_UP(clk_freq, HZ));
	rtk_timer_set_mode(nr, TIMER);
	rtk_timer_control(nr, HWT_START);
	rtk_timer_control(nr, HWT_INT_ENABLE);

	return 0;
}

static int rtk_tm0_set_state_oneshot(struct clock_event_device *cev)
{
	int nr = 0;

	/* Match programmed in set_next; keep free-running counter alive. */
	pr_info("[RTK-TIMER%d] set mode: oneshot\n", nr);
	rtk_timer_control(nr, HWT_INT_DISABLE);
	rtk_timer_set_mode(nr, COUNTER);
	rtk_timer_control(nr, HWT_START);

	return 0;
}

static int rtk_tm0_set_state_oneshot_stopped(struct clock_event_device *cev)
{
	int nr = 0;

	/*
	 * Only silence the interrupt. The counter has to keep running,
	 * because set_next_event programmes the match register relative to
	 * the value it reads out of it.
	 */
	rtk_timer_control(nr, HWT_INT_DISABLE);

	return 0;
}

static int rtk_tm0_set_state_shutdown(struct clock_event_device *cev)
{
	int nr = 0;

	pr_info("[RTK-TIMER%d] set mode: shutdown\n", nr);
	timer_suspend_data[nr].value = rtk_timer_get_value(nr);
	timer_suspend_data[nr].mode = rtk_timer_get_mode(nr);
	rtk_timer_control(nr, HWT_INT_DISABLE);
	rtk_timer_control(nr, HWT_STOP);

	return 0;
}

static int rtk_tm0_tick_resume(struct clock_event_device *cev)
{
	int nr = 0;

	pr_info("[RTK-TIMER%d] set mode: CLOCK_EVT_MODE_RESUME\n", nr);
	rtk_timer_set_value(nr, timer_suspend_data[nr].value);
	rtk_timer_set_target(nr, DIV_ROUND_UP(clk_freq, HZ));
	rtk_timer_set_mode(nr, timer_suspend_data[nr].mode);
	rtk_timer_control(nr, HWT_RESUME);
	rtk_timer_control(nr, HWT_INT_ENABLE);

	return 0;
}

static u64 rtk_clocksource_read(struct clocksource *cs)
{
	return (u64)rtk_timer_get_value(TIMER2);
}

static u64 notrace rtk_read_sched_clock(void)
{
	return (u64)rtk_timer_get_value(TIMER2);
}

static struct clocksource rtk_cs = {
	.name = "rtk_timer2_counter",
	.rating = 400,
	.read = rtk_clocksource_read,
	.mask = CLOCKSOURCE_MASK(32),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
	.suspend = rtk_clocksource_suspend,
	.resume = rtk_clocksource_resume,
};

static struct clock_event_device timer0_clockevent = {
	.rating = 400,
	.shift = 32,
	/*
	 * Periodic only. One-shot is implemented and the tick core does
	 * switch over when it is advertised, but an idle CPU then loses an
	 * event now and then. See the comment above rtk_timer_starting_cpu().
	 */
	.features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_PERCPU,
	.set_next_event = rtk_timer0_set_next,
	.set_state_periodic = rtk_tm0_set_state_periodic,
	.set_state_oneshot = rtk_tm0_set_state_oneshot,
	.set_state_oneshot_stopped = rtk_tm0_set_state_oneshot_stopped,
	.set_state_shutdown = rtk_tm0_set_state_shutdown,
	.tick_resume = rtk_tm0_tick_resume,

};

static struct irqaction timer0_irq = {
	.flags = IRQF_TIMER | IRQF_IRQPOLL,
	.handler = rtk_clock_event_isr,
	.dev_id = &timer0_clockevent,
};

static int rtk_timer1_set_next(unsigned long cycles,
			       struct clock_event_device *evt)
{
	return rtk_clkevt_set_next(TIMER1, cycles, evt);
}

static int rtk_tm1_set_state_periodic(struct clock_event_device *cev)
{
	int nr = 1;

	pr_info("[RTK-TIMER%d] set mode: periodic\n", nr);
	rtk_timer_control(nr, HWT_INT_DISABLE);
	rtk_timer_control(nr, HWT_STOP);
	rtk_timer_set_value(nr, timer_suspend_data[nr].value);
	rtk_timer_set_target(nr, DIV_ROUND_UP(clk_freq, HZ));
	rtk_timer_set_mode(nr, TIMER);
	rtk_timer_control(nr, HWT_START);
	rtk_timer_control(nr, HWT_INT_ENABLE);

	return 0;
}

static int rtk_tm1_set_state_oneshot(struct clock_event_device *cev)
{
	int nr = 1;

	/* Match programmed in set_next; keep free-running counter alive. */
	pr_info("[RTK-TIMER%d] set mode: oneshot\n", nr);
	rtk_timer_control(nr, HWT_INT_DISABLE);
	rtk_timer_set_mode(nr, COUNTER);
	rtk_timer_control(nr, HWT_START);

	return 0;
}

static int rtk_tm1_set_state_oneshot_stopped(struct clock_event_device *cev)
{
	int nr = 1;

	/*
	 * Only silence the interrupt. The counter has to keep running,
	 * because set_next_event programmes the match register relative to
	 * the value it reads out of it.
	 */
	rtk_timer_control(nr, HWT_INT_DISABLE);

	return 0;
}

static int rtk_tm1_set_state_shutdown(struct clock_event_device *cev)
{
	int nr = 1;

	pr_info("[RTK-TIMER%d] set mode: shutdown\n", nr);
	timer_suspend_data[nr].value = rtk_timer_get_value(nr);
	timer_suspend_data[nr].mode = rtk_timer_get_mode(nr);
	rtk_timer_control(nr, HWT_INT_DISABLE);
	rtk_timer_control(nr, HWT_STOP);

	return 0;
}

static int rtk_tm1_tick_resume(struct clock_event_device *cev)
{
	int nr = 1;

	pr_info("[RTK-TIMER%d] set mode: CLOCK_EVT_MODE_RESUME\n", nr);
	rtk_timer_set_value(nr, timer_suspend_data[nr].value);
	rtk_timer_set_target(nr, DIV_ROUND_UP(clk_freq, HZ));
	rtk_timer_set_mode(nr, timer_suspend_data[nr].mode);
	rtk_timer_control(nr, HWT_RESUME);
	rtk_timer_control(nr, HWT_INT_ENABLE);

	return 0;
}

static struct clock_event_device timer1_clockevent = {
	.rating = 400,
	.shift = 32,
	/*
	 * Periodic only. One-shot is implemented and the tick core does
	 * switch over when it is advertised, but an idle CPU then loses an
	 * event now and then. See the comment above rtk_timer_starting_cpu().
	 */
	.features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_PERCPU,
	.set_next_event = rtk_timer1_set_next,
	.set_state_periodic = rtk_tm1_set_state_periodic,
	.set_state_oneshot = rtk_tm1_set_state_oneshot,
	.set_state_oneshot_stopped = rtk_tm1_set_state_oneshot_stopped,
	.set_state_shutdown = rtk_tm1_set_state_shutdown,
	.tick_resume = rtk_tm1_tick_resume,
};

static struct irqaction timer1_irq = {
	.flags = IRQF_TIMER,
	.handler = rtk_clock_event_isr,
	.dev_id = &timer1_clockevent,
};

static struct rtk_clock_event_device rtk_evt[] = {
	{ 0, &timer0_clockevent, &timer0_irq },
	{ 1, &timer1_clockevent, &timer1_irq },
};

/*
 * TIMER0 and TIMER1 each have their own SPI, so one can be pinned to each
 * CPU and used as that CPU's tick device. The interrupt is left disabled
 * until the CPU it belongs to comes up, and is only then given its
 * affinity, the same way exynos_mct drives per-CPU clockevents off SPIs.
 */
static void rtk_clockevent_prepare(int index, const char *name,
				   void __iomem *base, int irq, unsigned long freq)
{
	struct rtk_clock_event_device *clkevt = &rtk_evt[index];
	struct clock_event_device *evt = clkevt->evt;

	timer_base = base;
	clk_freq = freq;
	evt->name = name;
	evt->cpumask = cpumask_of(index);

	memset(&timer_suspend_data[index], 0, sizeof(timer_suspend_data[index]));

	irq_set_status_flags(irq, IRQ_NOAUTOEN);
	if (request_irq(irq, clkevt->irq_action->handler,
			clkevt->irq_action->flags | IRQF_NOBALANCING,
			name, clkevt)) {
		pr_err("[RTK-TIMER%d] cannot register IRQ %d\n", index, irq);
		return;
	}

	evt->irq = irq;
}

/*
 * Note on one-shot, which is deliberately not advertised above.
 *
 * The vendor's own 3.10 driver for this silicon runs one-shot and NOHZ
 * happily, but it does NOT give each CPU its own tick device: it has
 *
 *     //  evt->cpumask = cpumask_of(cpu);
 *         evt->cpumask = cpu_all_mask;
 *
 * so both timers are global devices and the secondary CPU is served by
 * tick broadcast. Binding a timer to a CPU and advertising one-shot, as
 * below, loses an event on an idle CPU every few boots and wedges it.
 * Three separate bugs were found and fixed along the way -- a shared
 * write-1-to-clear status register updated with a read-modify-write, a
 * stopped counter that set_next_event() measures from, and -ETIME handed
 * to a core path that discards it -- and none of them was the whole
 * story. Until the rest is understood, keep the per-CPU devices, which
 * have never failed, and stay periodic.
 */
static int rtk_timer_starting_cpu(unsigned int cpu)
{
	struct clock_event_device *evt;

	if (cpu >= ARRAY_SIZE(rtk_evt))
		return 0;

	evt = rtk_evt[cpu].evt;
	if (!evt->irq)
		return 0;

	irq_force_affinity(evt->irq, cpumask_of(cpu));
	enable_irq(evt->irq);

	/*
	 * Only register once. On a later re-online the device is still known
	 * to the core, which reprogrammes it itself; re-registering would put
	 * the same list node on the clockevent list twice.
	 */
	if (!rtk_evt[cpu].registered) {
		clockevents_config_and_register(evt, clk_freq,
						RTK_TIMER_MIN_DELTA,
						RTK_TIMER_MAX_DELTA);
		rtk_evt[cpu].registered = true;
	}

	return 0;
}

static int rtk_timer_dying_cpu(unsigned int cpu)
{
	struct clock_event_device *evt;

	if (cpu >= ARRAY_SIZE(rtk_evt))
		return 0;

	evt = rtk_evt[cpu].evt;
	if (!evt->irq)
		return 0;

	evt->set_state_shutdown(evt);
	disable_irq_nosync(evt->irq);

	return 0;
}

static void rtk_clocksource_init(void)
{
	if (rtk_cs_registered || !timer_base || !clk_freq)
		return;

	/*
	 * Dedicated free-running TIMER2 clocksource — same split as the
	 * working 3.10 RTD119x driver. TIMER0/1 stay clockevent-only so
	 * shutdown/oneshot never freezes timekeeping.
	 */
	rtk_timer_control(TIMER2, HWT_INT_DISABLE);
	rtk_timer_control(TIMER2, HWT_STOP);
	rtk_timer_set_value(TIMER2, 0);
	rtk_timer_set_target(TIMER2, ~0U);
	rtk_timer_set_mode(TIMER2, COUNTER);
	rtk_timer_control(TIMER2, HWT_START);

	/*
	 * TIMER2 is also the only sane sched_clock source here. Without this
	 * sched_clock() falls back to jiffies, which leaves every scheduler,
	 * printk, perf and ftrace timestamp quantised to a jiffy (10ms at
	 * HZ=100) instead of the counter's ~37ns.
	 */
	sched_clock_register(rtk_read_sched_clock, 32, clk_freq);

	pr_info("[RTK-TIMER2] clocksource register at %lu Hz\n", clk_freq);
	if (clocksource_register_hz(&rtk_cs, clk_freq))
		pr_err("[RTK-TIMER2] can't register clocksource\n");
	else
		rtk_cs_registered = true;
}

static int __init rtk_timer_init(struct device_node *np)
{
	void __iomem *iobase;
	struct clk *clk;
	unsigned long rate;
	int irq0, irq1;
	u32 freq;
	int ret;

	iobase = of_iomap(np, 0);
	if (!iobase) {
		pr_err("rtk-timer: failed to map registers\n");
		return -ENXIO;
	}

	irq0 = irq_of_parse_and_map(np, 0);
	irq1 = irq_of_parse_and_map(np, 1);
	if (irq0 <= 0 || irq1 <= 0) {
		pr_err("rtk-timer: need one interrupt per timer\n");
		return -EINVAL;
	}

	clk = of_clk_get(np, 0);
	if (!IS_ERR(clk)) {
		ret = clk_prepare_enable(clk);
		if (ret) {
			pr_err("rtk-timer: failed to enable clock\n");
			return ret;
		}
		rate = clk_get_rate(clk);
	} else {
		ret = of_property_read_u32(np, "clock-frequency", &freq);
		if (ret) {
			pr_err("rtk-timer: no clock and no clock-frequency\n");
			return ret;
		}
		rate = freq;
	}

	rtk_clockevent_prepare(TIMER0, "rtk_timer0", iobase, irq0, rate);
	rtk_clockevent_prepare(TIMER1, "rtk_timer1", iobase, irq1, rate);
	rtk_clocksource_init();

	/* Both tick devices are known now, so hand them to the CPUs. */
	return cpuhp_setup_state(CPUHP_AP_REALTEK_TIMER_STARTING,
				 "clockevents/realtek/timer:starting",
				 rtk_timer_starting_cpu, rtk_timer_dying_cpu);
}

TIMER_OF_DECLARE(realtek_timer, "realtek,rtd1195-timer", rtk_timer_init);
