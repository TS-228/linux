// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Realtek RTD1195 SMP support
 *
 * Secondary CPU release-from-reset via a direct register poke (no PSCI,
 * no firmware cooperation) - the SoC's "wrap_a7" reset-control register
 * gates CPU1's core-reset line, and a separate boot-vector scratch
 * register holds the physical address CPU1 jumps to once released.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/smp.h>
#include <asm/cacheflush.h>
#include <asm/smp_plat.h>

#define RTD1195_WRAP_A7_PHYS		0x1801d100
#define RTD1195_WRAP_A7_CORE1_RESET	BIT(5)
#define RTD1195_BOOT_VECTOR_PHYS	0x18007064

static void __iomem *wrap_a7_base;
static void __iomem *boot_vector_base;

/*
 * pen_release was removed from the ARM core in 5.x; keep a local copy for
 * the secondary-boot handshake (same pattern used by several other
 * platsmp.c files elsewhere in the tree, e.g. spear, exynos).
 */
static volatile int pen_release = -1;

static void write_pen_release(int val)
{
	pen_release = val;
	smp_wmb();
	__cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
}

static DEFINE_SPINLOCK(rtd1195_boot_lock);

static void rtd1195_secondary_init(unsigned int cpu)
{
	/* Let the primary processor know we're out of the pen */
	write_pen_release(-1);

	/* Synchronise with the boot thread */
	spin_lock(&rtd1195_boot_lock);
	spin_unlock(&rtd1195_boot_lock);
}

static int rtd1195_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;
	unsigned long phys_cpu = cpu_logical_map(cpu);

	if (!wrap_a7_base || !boot_vector_base)
		return -ENODEV;

	spin_lock(&rtd1195_boot_lock);

	write_pen_release(phys_cpu);

	writel(readl(wrap_a7_base) | RTD1195_WRAP_A7_CORE1_RESET, wrap_a7_base);
	writel(virt_to_phys(secondary_startup), boot_vector_base);

	arch_send_wakeup_ipi_mask(cpumask_of(cpu));

	timeout = jiffies + HZ;
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (pen_release == -1)
			break;
		udelay(10);
	}

	spin_unlock(&rtd1195_boot_lock);

	return pen_release != -1 ? -ENOSYS : 0;
}

static void __init rtd1195_smp_init_cpus(void)
{
	wrap_a7_base = ioremap(RTD1195_WRAP_A7_PHYS, 4);
	boot_vector_base = ioremap(RTD1195_BOOT_VECTOR_PHYS, 4);
	if (!wrap_a7_base || !boot_vector_base) {
		pr_err("rtd1195: failed to map SMP control registers\n");
		return;
	}

	/* Hold CPU1 in reset until we're ready to release it */
	writel(readl(wrap_a7_base) & ~RTD1195_WRAP_A7_CORE1_RESET, wrap_a7_base);
	writel(virt_to_phys(secondary_startup), boot_vector_base);
}

static void __init rtd1195_smp_prepare_cpus(unsigned int max_cpus)
{
	int i;

	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);
}

const struct smp_operations rtd1195_smp_ops __initconst = {
	.smp_init_cpus		= rtd1195_smp_init_cpus,
	.smp_prepare_cpus	= rtd1195_smp_prepare_cpus,
	.smp_secondary_init	= rtd1195_secondary_init,
	.smp_boot_secondary	= rtd1195_boot_secondary,
};
