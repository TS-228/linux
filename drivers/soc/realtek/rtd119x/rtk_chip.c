// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2017 Realtek Semiconductor Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <soc/realtek/rtk_chip.h>
#include <linux/module.h>
#include <linux/io.h>

#define SB2_BASE_ADDR   0x1801A000
#define CHIP_ID         0x00000200
#define CHIP_REV        0x00000204

/* Phoenix's chip revision naming differs from Kylin's and Hercules's. */
#define RTD119X_CHIP_A (0x00000000)
#define RTD119X_CHIP_B (0x00010000)
#define RTD119X_CHIP_C (0x00020000)
#define RTD119X_CHIP_D (0x00030000)

/* Signature this SoC leaves in SB2's CHIP_ID register. */
#define RTD1195_CHIP_ID_VAL	0x00006329
#define RTD1195_CHIP_ID_MASK	0x0000ffff

int get_rtd_chip_id(void)
{
	void __iomem *chip_id_addr = ioremap(SB2_BASE_ADDR + CHIP_ID, 0x4);
	int val;

	if (!chip_id_addr)
		return 0;

	val = readl(chip_id_addr);
	iounmap(chip_id_addr);

	switch (val & RTD1195_CHIP_ID_MASK) {
	case RTD1195_CHIP_ID_VAL:
		return CHIP_ID_RTD1195;
	default:
		pr_warn("%s: unknown chip id 0x%08x\n", __func__, val);
	}
	return 0;
}
EXPORT_SYMBOL(get_rtd_chip_id);

int get_rtd_chip_revision(void)
{
	void __iomem *chip_revision_addr = ioremap(SB2_BASE_ADDR + CHIP_REV, 0x4);
	int val;

	if (!chip_revision_addr)
		return 0;

	val = readl(chip_revision_addr);
	iounmap(chip_revision_addr);

	switch (val) {
	case (RTD119X_CHIP_A):
		return RTD_CHIP_A00;
	case (RTD119X_CHIP_B):
		return RTD_CHIP_A01;
	case (RTD119X_CHIP_C):
		return RTD_CHIP_B00;
	case (RTD119X_CHIP_D):
		return RTD_CHIP_B01;
	default:
		pr_warn("%s: no chip revision for 0x%x\n", __func__, val);
	}
	return 0;
}
EXPORT_SYMBOL(get_rtd_chip_revision);
