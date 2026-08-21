// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Realtek RTD1195
 *
 * Copyright (c) 2017-2019 Andreas Färber
 */

#include <linux/io.h>
#include <linux/memblock.h>
#include <asm/mach/arch.h>

extern const struct smp_operations rtd1195_smp_ops;

/*
 * Chip ID / revision, read once at boot from a pair of adjacent RBUS
 * registers. Exported for drivers that need to distinguish RTD1195
 * silicon revisions (e.g. the eMMC and Ethernet glue drivers).
 */
unsigned int rtd1195_chip_id;
EXPORT_SYMBOL(rtd1195_chip_id);
unsigned int rtd1195_chip_rev;
EXPORT_SYMBOL(rtd1195_chip_rev);

#define RTD1195_CHIP_ID_PHYS	0x1801a200

static void __init rtd1195_init_machine(void)
{
	void __iomem *base;

	base = ioremap(RTD1195_CHIP_ID_PHYS, 8);
	if (!base) {
		pr_err("rtd1195: failed to map chip ID register\n");
		return;
	}

	rtd1195_chip_id = readl(base);
	rtd1195_chip_rev = readl(base + 4) >> 16;
	iounmap(base);
}

static void __init rtd1195_memblock_remove(phys_addr_t base, phys_addr_t size)
{
	int ret;

	ret = memblock_remove(base, size);
	if (ret)
		pr_err("Failed to remove memblock %pa (%d)\n", &base, ret);
}

static void __init rtd1195_reserve(void)
{
	/* Exclude boot ROM from RAM */
	rtd1195_memblock_remove(0x00000000, 0x0000a800);

	/* Exclude peripheral register spaces from RAM */
	rtd1195_memblock_remove(0x18000000, 0x00070000);
	rtd1195_memblock_remove(0x18100000, 0x01000000);
}

static const char *const rtd1195_dt_compat[] __initconst = {
	"realtek,rtd1195",
	NULL
};

DT_MACHINE_START(rtd1195, "Realtek RTD1195")
	.dt_compat = rtd1195_dt_compat,
	.reserve = rtd1195_reserve,
	.smp = smp_ops(rtd1195_smp_ops),
	.init_machine = rtd1195_init_machine,
	.l2c_aux_val = 0x0,
	.l2c_aux_mask = ~0x0,
MACHINE_END
