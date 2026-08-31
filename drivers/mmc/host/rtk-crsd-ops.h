/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Shared low-level helpers for Realtek's RTD1195 "Card Reader" IP core.
 *
 * The core is instantiated twice on this SoC -- once feeding the SD card
 * slot (drivers/mmc/host/rtk-sdmmc.c) and once feeding the internal eMMC
 * (drivers/mmc/host/rtkemmc_rtd119x.c) -- at different physical base
 * addresses, but with an identical register layout (verified offset-for-
 * offset and bit-for-bit between rtk-sdmmc-reg.h and reg_mmc_rtd119x.h).
 * This file holds only the pieces of that shared contract which are
 * genuinely identical in both behavior and register target between the
 * two drivers -- not a merge of the drivers themselves, which remain
 * separate platform_drivers with their own probe/DT binding/host state.
 *
 * Copyright (C) 2017 Realtek Ltd.
 */

#ifndef DRIVERS_MMC_HOST_RTK_CRSD_OPS_H_
#define DRIVERS_MMC_HOST_RTK_CRSD_OPS_H_

#include <linux/completion.h>
#include <linux/types.h>
#include <linux/mmc/core.h>

/*
 * Flush/sync the shared bus bridge so a preceding register write is
 * guaranteed visible before the caller proceeds. sysbrdg_base must be the
 * "BS2" bank mapping (physical 0x1801a000 on this SoC) -- this is a single
 * SoC-wide register, not per-engine-instance, so both the SD-slot and
 * eMMC drivers pass their own mapping of the same physical bank.
 */
void rtk_crsd_sync(void __iomem *sysbrdg_base);

/* Decode the SD_CONFIGURE2-style 2-bit response-length selector. */
u8 rtk_crsd_get_rsp_len(u8 rsp_para);

/*
 * CR_TRANS_OK/CR_TRANSFER_TO/CR_TRANSFER_FAIL, numerically identical in
 * both rtk-sdmmc.h and rtkemmc_rtd119x.h -- defined once here so callers
 * of rtk_crsd_poll_for_end() below don't need their own header for just
 * these three values. Each driver's own header keeps defining its own
 * copies too (unchanged, still used throughout each file); these are
 * the same numbers, not a replacement for those.
 */
#define RTK_CRSD_TRANS_OK      0x00
#define RTK_CRSD_TRANSFER_TO   0x01
#define RTK_CRSD_TRANSFER_FAIL 0x04

/*
 * Fire `cmdcode` (already OR'd with any driver-specific flags, e.g.
 * START_EN, by the caller) into the engine's SD_TRANSFER register and
 * wait up to 300ms on `int_waiting` for its completion interrupt.
 *
 * Callers must configure the interrupt-enable register themselves
 * beforehand -- that selection genuinely differs between the SD-slot
 * and eMMC engines (verified: different bits, different read-modify-
 * write vs. absolute-write style) and is not part of this shared step.
 *
 * This always reinit_completion()s `int_waiting` first, so a stale
 * completion left over from a previous command can't be mistaken for
 * this one's. The 300ms bound and the reinit are both matched to
 * rtk-sdmmc.c's design, stress-tested on hardware (6400+ consecutive
 * commands, zero timeouts) before being applied here to the eMMC path
 * too, which previously waited unbounded on this same interrupt.
 *
 * Does not by itself determine success/failure -- each driver has its
 * own convention for reading that back (a flag set by its own ISR, or
 * a raw status-register snapshot) and should follow this call by
 * checking its own state, falling back to rtk_crsd_poll_for_end()
 * below if the interrupt-driven wait didn't see a clean completion.
 */
void rtk_crsd_fire_and_wait(void __iomem *base, struct completion *int_waiting, u8 cmdcode);

/*
 * Bounded busy-poll fallback for when the interrupt-driven wait above
 * didn't observe a clean completion. Polls the engine's SD_TRANSFER
 * register (syncing the bus before each read, matching the proven
 * rtk-sdmmc.c loop this was extracted from) until (END_STATE|
 * IDLE_STATE) is set (-> RTK_CRSD_TRANS_OK), ERR_STATUS is set
 * (-> RTK_CRSD_TRANSFER_FAIL), or timeout_jiffies has elapsed since
 * start_jiffies (-> RTK_CRSD_TRANSFER_TO). sysbrdg_base is the same
 * BS2-bank mapping passed to rtk_crsd_sync().
 */
int rtk_crsd_poll_for_end(void __iomem *base, void __iomem *sysbrdg_base,
			  unsigned long start_jiffies, unsigned long timeout_jiffies);

#endif /* DRIVERS_MMC_HOST_RTK_CRSD_OPS_H_ */
