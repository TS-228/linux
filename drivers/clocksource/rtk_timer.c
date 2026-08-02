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

#define TIMER0 0
#define TIMER1 1
#define TIMER2 2
#define TIMER_MAX (TIMER2 + 1)

#define SYSTEM_TIMER TIMER0

#define COUNTER 0
#define TIMER 1

#define TR_EN (1 << 31)
#define TR_MODE (1 << 30)
#define TR_PAUSE (1 << 29)
#define TR_INT_EN (1 << 31)

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
};

static struct _suspend_data sTimerSuspendData[TIMER_MAX];

static void __iomem *timer_base;
unsigned long clk_freq;
static bool rtk_cs_registered;

#define MISC_BASE(pa) (timer_base + pa)
#define rtk_setbits(offset, Mask) \
	__raw_writel(((__raw_readl(offset) | Mask)), offset)
#define rtk_clearbits(offset, Mask) \
	__raw_writel(((__raw_readl(offset) & ~Mask)), offset)

unsigned char rtk_timer_get_mode(unsigned char id)
{
	unsigned int reg = __raw_readl(MISC_BASE(TCCR_OFFSET + (id << 2)));

	return (reg & TR_MODE) ? TIMER : COUNTER;
}

int rtk_timer_control(unsigned char id, unsigned int cmd)
{
	switch (cmd) {
	case HWT_INT_CLEAR:
		if (id < ARRAY_SIZE(TC_SHIFT))
			rtk_setbits(MISC_BASE(UMSK_ISR_OFFSET), TC_SHIFT[id] | 0x1);
		break;
	case HWT_START:
		rtk_setbits(MISC_BASE(TCCR_OFFSET + (id << 2)), TR_EN);
		if (id < ARRAY_SIZE(TC_SHIFT))
			rtk_setbits(MISC_BASE(ISR_OFFSET), TC_SHIFT[id]);
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

int rtk_timer_get_value(unsigned char id)
{
	/* get the current timer's value */
	return __raw_readl(MISC_BASE(TCCVR_OFFSET + (id << 2)));
}

int rtk_timer_set_value(unsigned char id, unsigned int value)
{
	/* set the timer's initial value */
	__raw_writel(value, MISC_BASE(TCCVR_OFFSET + (id << 2)));
	return 0;
}

int rtk_timer_set_target(unsigned char id, unsigned int value)
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

	clkevt = (struct rtk_clock_event_device *) dev_id;
	evt = clkevt->evt;
	nr = clkevt->index;

	rtk_setbits(MISC_BASE(ISR_OFFSET), TC_SHIFT[nr]);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static int rtk_clkevt_set_next(int nr, unsigned long cycles,
	struct clock_event_device *evt)
{
	u32 cnt;

	/*
	 * Hardware is a free-running up-counter with a match register.
	 * Program match = now + cycles (same as the working 3.10 driver).
	 * Setting an absolute small target waits for a full 32-bit wrap
	 * (~159s at 27 MHz) and stalls softirqs / RCU.
	 */
	rtk_timer_control(nr, HWT_INT_ENABLE);
	cnt = rtk_timer_get_value(nr);
	cnt += cycles;
	rtk_timer_set_target(nr, cnt);

	/* Already past the match point — ask for a retry. */
	return ((s32)(rtk_timer_get_value(nr) - cnt) > 0) ? -ETIME : 0;
}

static void rtk_clocksource_suspend(struct clocksource *cs)
{
	pr_info("[RTK-TIMER] Enter %s\n", __func__);
	sTimerSuspendData[TIMER2].value = rtk_timer_get_value(TIMER2);
	pr_info("[RTK-TIMER] Exit %s\n", __func__);
}

static void rtk_clocksource_resume(struct clocksource *cs)
{
	pr_info("[RTK-TIMER] Enter %s\n", __func__);
	rtk_timer_control(TIMER2, HWT_STOP);
	rtk_timer_set_value(TIMER2, sTimerSuspendData[TIMER2].value);
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
	rtk_timer_set_value(nr, sTimerSuspendData[nr].value);
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

	pr_info("[RTK-TIMER%d] set mode: stopped\n", nr);
	rtk_timer_control(nr, HWT_STOP);

	return 0;
}

static int rtk_tm0_set_state_shutdown(struct clock_event_device *cev)
{
	int nr = 0;

	pr_info("[RTK-TIMER%d] set mode: shutdown\n", nr);
	sTimerSuspendData[nr].value = rtk_timer_get_value(nr);
	sTimerSuspendData[nr].mode = rtk_timer_get_mode(nr);
	rtk_timer_control(nr, HWT_INT_DISABLE);
	rtk_timer_control(nr, HWT_STOP);

	return 0;
}

static int rtk_tm0_tick_resume(struct clock_event_device *cev)
{
	int nr = 0;

	pr_info("[RTK-TIMER%d] set mode: CLOCK_EVT_MODE_RESUME\n", nr);
	rtk_timer_set_value(nr, sTimerSuspendData[nr].value);
	rtk_timer_set_target(nr, DIV_ROUND_UP(clk_freq, HZ));
	rtk_timer_set_mode(nr, sTimerSuspendData[nr].mode);
	rtk_timer_control(nr, HWT_RESUME);
	rtk_timer_control(nr, HWT_INT_ENABLE);

	return 0;
}

static u64 rtk_read_sched_clock(struct clocksource *cs)
{
	return (u64)rtk_timer_get_value(TIMER2);
}

static struct clocksource rtk_cs = {
	.name = "rtk_timer2_counter",
	.rating = 400,
	.read = rtk_read_sched_clock,
	.mask = CLOCKSOURCE_MASK(32),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
	.suspend = rtk_clocksource_suspend,
	.resume = rtk_clocksource_resume,
};

static struct clock_event_device timer0_clockevent = {
	.rating = 400,
	.shift = 32,
	/*
	 * Periodic-only for now: oneshot/highres left CPU1 without ticks
	 * (timer1 SPI affinity is unreliable on this GIC). CPU1 is serviced
	 * via tick broadcast from timer0.
	 */
	.features = CLOCK_EVT_FEAT_PERIODIC,
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
	rtk_timer_set_value(nr, sTimerSuspendData[nr].value);
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

	pr_info("[RTK-TIMER%d] set mode: stopped\n", nr);
	rtk_timer_control(nr, HWT_INT_DISABLE);
	rtk_timer_control(nr, HWT_STOP);

	return 0;
}

static int rtk_tm1_set_state_shutdown(struct clock_event_device *cev)
{
	int nr = 1;

	pr_info("[RTK-TIMER%d] set mode: shutdown\n", nr);
	sTimerSuspendData[nr].value = rtk_timer_get_value(nr);
	sTimerSuspendData[nr].mode = rtk_timer_get_mode(nr);
	rtk_timer_control(nr, HWT_INT_DISABLE);
	rtk_timer_control(nr, HWT_STOP);

	return 0;
}

static int rtk_tm1_tick_resume(struct clock_event_device *cev)
{
	int nr = 1;

	pr_info("[RTK-TIMER%d] set mode: CLOCK_EVT_MODE_RESUME\n", nr);
	rtk_timer_set_value(nr, sTimerSuspendData[nr].value);
	rtk_timer_set_target(nr, DIV_ROUND_UP(clk_freq, HZ));
	rtk_timer_set_mode(nr, sTimerSuspendData[nr].mode);
	rtk_timer_control(nr, HWT_RESUME);
	rtk_timer_control(nr, HWT_INT_ENABLE);

	return 0;
}

static struct clock_event_device timer1_clockevent = {
	.rating = 100,
	.shift = 32,
	.features = CLOCK_EVT_FEAT_PERIODIC,
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

void rtk_clockevent_init(int index, const char *name, void __iomem *base,
	int irq, unsigned long freq)
{
	struct rtk_clock_event_device *clkevt = &rtk_evt[index];
	struct clock_event_device *evt = clkevt->evt;

	timer_base = base;
	clk_freq = freq;
	evt->irq = irq;
	evt->name = name;
	/*
	 * Use timer0 as the global tick device; secondary CPUs get ticks via
	 * broadcast IPIs. Binding timer1 to CPU1 via SPI affinity left CPU1
	 * without reliable interrupts on this platform.
	 */
	if (index == TIMER0)
		evt->cpumask = cpu_possible_mask;
	else
		evt->cpumask = cpumask_of(index);

	memset(&sTimerSuspendData[index], 0, sizeof(sTimerSuspendData[index]));

	if (request_irq(irq, clkevt->irq_action->handler,
			clkevt->irq_action->flags, name, clkevt))
		pr_err("[RTK-TIMER%d] cannot register IRQ %d\n", index, irq);

	clockevents_config_and_register(evt, clk_freq, 0xF, UINT_MAX);
}

void rtk_clocksource_init(void)
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

	pr_info("[RTK-TIMER2] clocksource register at %lu Hz\n", clk_freq);
	if (clocksource_register_hz(&rtk_cs, clk_freq))
		pr_err("[RTK-TIMER2] can't register clocksource\n");
	else
		rtk_cs_registered = true;
}

static int __init rtk_timer0_init(struct device_node *np)
{
	int ret = 0;
	void __iomem *iobase;
	int irq = 0;
	int rate = 0;

	iobase = of_iomap(np, 0);
	if (!iobase) {
		pr_err("[RTK-TIMER0] failed to get base address\n");
		return 1;
	}

	irq = irq_of_parse_and_map(np, 0);
	if (irq <= 0) {
		pr_err("[RTK-TIMER0] can't parse IRQ\n");
		return 1;
	}

	ret = of_property_read_u32(np, "clock-frequency", &rate);
	if (ret) {
		pr_err("[RTK-TIMER0] can't get clock-frequency\n");
		return 1;
	}

	rtk_clockevent_init(TIMER0, np->name, iobase, irq, rate);
	rtk_clocksource_init();

	return ret;
}

static int __init rtk_timer1_init(struct device_node *np)
{
	/*
	 * TIMER1 is not used as a clockevent: SPI affinity to CPU1 is not
	 * reliable here, so CPU1 is ticked via broadcast from TIMER0.
	 * Keep this OF declare so the node is marked initialized.
	 */
	return 0;
}

TIMER_OF_DECLARE(realtek_timer0, "realtek,rtk-timer0", rtk_timer0_init);
TIMER_OF_DECLARE(realtek_timer1, "realtek,rtk-timer1", rtk_timer1_init);
/* Legacy RTD119x DT bindings (vendor trees). */
TIMER_OF_DECLARE(rtd119x_timer0, "Realtek,rtd119x-timer0", rtk_timer0_init);
TIMER_OF_DECLARE(rtd119x_timer1, "Realtek,rtd119x-timer1", rtk_timer1_init);
