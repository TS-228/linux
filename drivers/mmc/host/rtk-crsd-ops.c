// SPDX-License-Identifier: GPL-2.0-only
/*
 * Shared low-level helpers for Realtek's RTD1195 "Card Reader" IP core.
 * See rtk-crsd-ops.h for the scope/rationale of what lives here.
 *
 * Copyright (C) 2017 Realtek Ltd.
 */

#include <linux/completion.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include "rtk-crsd-ops.h"

/* Offset of the sync/flush register within the shared BS2 bank. */
#define RTK_CRSD_SYNC_REG 0x0020

/*
 * SD_TRANSFER and its status bits: offsets/values verified identical in
 * both rtk-sdmmc-reg.h (SD_TRANSFER/START_EN/END_STATE/IDLE_STATE/
 * ERR_STATUS) and reg_mmc_rtd119x.h (same names, same values) before
 * being duplicated here so this file doesn't need to pull in either
 * driver's full register header.
 */
#define RTK_CRSD_SD_TRANSFER 0x0193
#define RTK_CRSD_START_EN    (0x01 << 7)
#define RTK_CRSD_END_STATE   (0x01 << 6)
#define RTK_CRSD_IDLE_STATE  (0x01 << 5)
#define RTK_CRSD_ERR_STATUS  (0x01 << 4)

void rtk_crsd_sync(void __iomem *sysbrdg_base)
{
	writel(0x00000000, sysbrdg_base + RTK_CRSD_SYNC_REG);
}
EXPORT_SYMBOL_GPL(rtk_crsd_sync);

u8 rtk_crsd_get_rsp_len(u8 rsp_para)
{
	switch (rsp_para & 0x03) {
	case 0:
		return 0;
	case 1:
		return 6;
	case 2:
		return 16;
	default:
		return 0;
	}
}
EXPORT_SYMBOL_GPL(rtk_crsd_get_rsp_len);

void rtk_crsd_fire_and_wait(void __iomem *base, struct completion *int_waiting, u8 cmdcode)
{
	reinit_completion(int_waiting);
	writeb((u8)(cmdcode | RTK_CRSD_START_EN), base + RTK_CRSD_SD_TRANSFER);
	wait_for_completion_timeout(int_waiting, msecs_to_jiffies(300));
}
EXPORT_SYMBOL_GPL(rtk_crsd_fire_and_wait);

int rtk_crsd_poll_for_end(void __iomem *base, void __iomem *sysbrdg_base,
			  unsigned long start_jiffies, unsigned long timeout_jiffies)
{
	while (time_before(jiffies, start_jiffies + timeout_jiffies)) {
		unsigned int sd_trans;

		rtk_crsd_sync(sysbrdg_base);
		sd_trans = readb(base + RTK_CRSD_SD_TRANSFER);

		if ((sd_trans & (RTK_CRSD_END_STATE | RTK_CRSD_IDLE_STATE)) ==
		    (RTK_CRSD_END_STATE | RTK_CRSD_IDLE_STATE))
			return RTK_CRSD_TRANS_OK;
		if (sd_trans & RTK_CRSD_ERR_STATUS)
			return RTK_CRSD_TRANSFER_FAIL;
	}
	return RTK_CRSD_TRANSFER_TO;
}
EXPORT_SYMBOL_GPL(rtk_crsd_poll_for_end);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Shared low-level ops for Realtek RTD1195 SD/eMMC card-reader IP");
