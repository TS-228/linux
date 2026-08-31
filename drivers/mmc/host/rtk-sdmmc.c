// SPDX-License-Identifier: GPL-2.0-only
/* Realtek SD/MMC/mini SD card driver -- Copyright (C) 2017 Realtek Ltd. */

#include <linux/blkdev.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/utsname.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/suspend.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/bitops.h>
#include <linux/regulator/consumer.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/kthread.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include "../core/card.h"
#include "rtk-sdmmc-reg.h"
#include "rtk-sdmmc.h"
#include "rtk-crsd-ops.h"

#define DRIVER_NAME "rtk-sdmmc"
#define BANNER "Realtek SD/MMC Host Driver"

#define SD_ALLOC_LENGTH 2048
#define MAX_PHASE 31
#define TUNING_CNT 3

/* CMD25_WO_STOP_COMMAND: see commit log - deferring the stop command
 * leaves the card in receive-data state, which stalls the core's busy poll.
 */
#undef CMD25_WO_STOP_COMMAND

enum int_enable_mode {
	CMD_RSP_ONLY = 0,
	DATA_ONLY,
	CMD_RSP_DATA_WRITE,
	CMD_RSP_DATA_READ,
	CMD_RSP_DATA_WRITE_TUNNING,
	CMD_RSP_DATA_READ_TUNNING,
	DATA_WRITE_ONLY,
	DATA_READ_ONLY,
};

DECLARE_COMPLETION(rtk_sdmmc_wait);
static int rtk_sdmmc_get_cd(struct mmc_host *host);
static int rtk_sdmmc_send_cmd_get_rsp(struct sdmmc_cmd_pkt *cmd_info);
static int rtk_sdmmc_stream(struct sdmmc_cmd_pkt *cmd_info);
static void rtk_sdmmc_hw_initial(struct rtk_sdmmc_host *rtk_host);
static void rtk_sdmmc_timeout(struct timer_list *t);
static void rtk_sdmmc_plug(struct timer_list *t);
static void rtk_sdmmc_set_access_mode(struct rtk_sdmmc_host *rtk_host, u8 level);
static void remove_sdcard(struct rtk_sdmmc_host *rtk_host);

void rtk_sdmmc_sync(struct rtk_sdmmc_host *rtk_host);
static void rtk_sdmmc_speed(struct rtk_sdmmc_host *rtk_host, enum sdmmc_clock_speed sd_speed);

#ifdef CMD25_WO_STOP_COMMAND
struct mmc_command cmd12_stop_cmd;
#endif

/*
 * The one module-scope "current host" pointer. Kept as a bare global
 * (not a struct field -- a field can't point at "the" host) because a
 * couple of call sites need to reach a host with no rtk_host of their
 * own in scope. See the probe() assignment for the load-bearing caveat:
 * with two hosts this only ever points at whichever one probed last.
 */
struct rtk_sdmmc_host *rtk_global_host;

static void card_broken(struct rtk_sdmmc_host *rtk_host)
{
	void __iomem *sdmmc_base = rtk_host->sdmmc;

	pr_err("SD Card is broken!!!\n");

	writel(0x0, sdmmc_base + CR_SD_DMA_CTL3); //stop dma control
	rtk_host->data_xfer_mode = CMD_RSP_ONLY;
	writeb(0xff, sdmmc_base + CR_CARD_STOP); //SD Card module transfer stop and idle state
	writeb(0x40, sdmmc_base + SD_BUS_STATUS);
	mdelay(10);
	writeb(0x3b, sdmmc_base + CARD_CLOCK_EN_CTL);
	rtk_host->ops->card_power(rtk_host, 0);
	rtk_sdmmc_sync(rtk_host);
}

static void remove_sdcard(struct rtk_sdmmc_host *rtk_host)
{
	u32 det_time = 0;
	unsigned long timeout = 0;
	void __iomem *sdmmc_base = rtk_host->sdmmc;

	rtk_host->ins_event = EVENT_REMOV;
	rtk_host->rtflags &= ~RTKCR_FCARD_DETECTED;
	det_time = 1;

	//reset
	timeout = jiffies + msecs_to_jiffies(100);

	while (time_before(jiffies, timeout)) {
		if (!(readl(rtk_host->emmc + EMMC_DMA_CTL3) & 0x01))
			break;
	}

	writel(0x0, sdmmc_base + CR_SD_DMA_CTL3); //stop dma control
	rtk_host->data_xfer_mode = CMD_RSP_ONLY;
	writeb(0xff, sdmmc_base + CR_CARD_STOP); //SD Card module transfer stop and idle state

	rtk_host->ops->card_power(rtk_host, 0); //power off, Jim add

	rtk_sdmmc_sync(rtk_host);
	rtk_host->rtflags &= ~RTKCR_FCARD_SELECTED;
	rtk_host->sdmmc_rca = 0;
	mmc_detect_change(rtk_host->mmc, msecs_to_jiffies(det_time));
	rtk_host->int_status_old = 0x20;
}

#ifdef CONFIG_PM
static int rtk_sdmmc_pm_suspend(struct device *dev)
{
	int ret = 0;
	u32 det_time = 0;
	unsigned long timeout = 0;
	u32 reginfo = 0;
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct rtk_sdmmc_host *rtk_host = mmc_priv(mmc);
	void __iomem *sdmmc_base = rtk_host->sdmmc;

	rtk_host->suspend_flag = true;

	if (pm_suspend_target_state == PM_SUSPEND_STANDBY)
		pr_debug("[SD] Realtek SD card reader idle suspend!!\n");
	else
		pr_debug("[SD] Realtek SD card reader suspend to ram start!!\n");

	timer_delete_sync(&rtk_host->timer);
	timer_delete_sync(&rtk_host->plug_timer);
#ifdef CMD25_WO_STOP_COMMAND
	timer_delete_sync(&rtk_host->rtk_sdmmc_stop_cmd);
#endif

	ret = pm_runtime_force_suspend(dev);
	reginfo = readb(sdmmc_base + CARD_EXIST);
	if (reginfo & SD_EXISTENCE) {
		pr_err("[SD] Realtek SD turn off card power!!\n");
		rtk_host->suspend_SD_insert_flag = true;
		rtk_host->ins_event = EVENT_REMOV;
		rtk_host->rtflags &= ~RTKCR_FCARD_DETECTED;
		det_time = 1;
		//reset
		timeout = jiffies + msecs_to_jiffies(100);

		while (time_before(jiffies, timeout)) {
			if (!(readl(rtk_host->emmc + EMMC_DMA_CTL3) & 0x01))
				break;
		}
		writel(0x0, sdmmc_base + CR_SD_DMA_CTL3); //stop dma control
		rtk_host->data_xfer_mode = CMD_RSP_ONLY;
		writeb(0xff, sdmmc_base + CR_CARD_STOP); //SD Card module transfer stop and idle state

		rtk_host->ops->card_power(rtk_host, 0); //power off, Jim add

		rtk_sdmmc_sync(rtk_host);
		rtk_host->rtflags &= ~RTKCR_FCARD_SELECTED;
		rtk_host->sdmmc_rca = 0;
		mmc_detect_change(rtk_host->mmc, msecs_to_jiffies(det_time));
	} else {
		rtk_host->suspend_SD_insert_flag = false;
	}

	return ret;
}

static int rtk_sdmmc_pm_resume(struct device *dev)
{
	int ret = 0;
	u32 reginfo = 0;
	u32 det_time = 0;
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct rtk_sdmmc_host *rtk_host = mmc_priv(mmc);
	void __iomem *pll_base = rtk_host->pll;
	void __iomem *sdmmc_base = rtk_host->sdmmc;

	if (pm_suspend_target_state == PM_SUSPEND_STANDBY)
		pr_debug("[SD] Realtek SD card reader idle resume OK!!\n");
	else
		pr_debug("[SD] Realtek SD card reader resume OK!!\n");

	if (pm_suspend_target_state == PM_SUSPEND_STANDBY) {
		writel(0x003E0003, pll_base + CR_PLL_SD1);
		mdelay(10);
	} else {
		/*
		 * Suspend powers off the SD PLL, leaving it at its default
		 * rate, so re-initialize the PLL and clock before resuming.
		 */
		rtk_sdmmc_hw_initial(rtk_host);
	}

	reginfo = readb(sdmmc_base + CARD_EXIST);
	if ((reginfo & SD_EXISTENCE) && rtk_host->suspend_SD_insert_flag) {
		pr_err("[SD] Realtek SD turn on card power!!\n");
		rtk_host->rtflags &= ~RTKCR_FCARD_DETECTED;
		rtk_host->wp = 0;

		rtk_host->ops->card_power(rtk_host, 1); //power on, Jim add

		rtk_host->ins_event = EVENT_INSER;
		rtk_host->rtflags |= RTKCR_FCARD_DETECTED;
		det_time = 1;
		writel(0x00000000, sdmmc_base + CR_SD_DMA_CTL3); //stop dma control
		rtk_host->data_xfer_mode = CMD_RSP_ONLY;
		writeb(0x00, sdmmc_base + CR_CARD_STOP); //SD Card module transfer no stop
		/* Select the SD/MMC card module for the coming data transfer (bits 2:0 = 010). */
		writeb(0x02,
		       sdmmc_base + CARD_SELECT);
		writeb(0x04, sdmmc_base + CR_CARD_OE); //arget module is SD/MMC card module, bit 2 =1
		writeb(0x04, sdmmc_base + CARD_CLOCK_EN_CTL); //SD Card Module Clock Enable, bit 2 = 1
		writeb(0xD0, sdmmc_base + SD_CONFIGURE1);

		if (pm_suspend_target_state != PM_SUSPEND_STANDBY)
			rtk_sdmmc_speed(rtk_host, SDMMC_CLOCK_400KHZ);

		writeb(0x00, sdmmc_base + SD_STATUS2);
		rtk_sdmmc_sync(rtk_host);
		rtk_host->rtflags &= ~RTKCR_FCARD_SELECTED;
		rtk_host->sdmmc_rca = 0;
		mmc_detect_change(rtk_host->mmc, msecs_to_jiffies(det_time));
	}

	ret = pm_runtime_force_resume(dev);
	timer_setup(&rtk_host->timer, rtk_sdmmc_timeout, 0);
	timer_setup(&rtk_host->plug_timer, rtk_sdmmc_plug, 0);
#ifdef CMD25_WO_STOP_COMMAND
	timer_setup(&rtk_host->rtk_sdmmc_stop_cmd, rtk_sdmmc_cmd12_fun, 0);
#endif
	rtk_sdmmc_sync(rtk_host);
	mod_timer(&rtk_host->plug_timer, jiffies + 3 * HZ);

	rtk_host->suspend_flag = false;

	return ret;
}

static const struct dev_pm_ops rtk_sdmmc_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(rtk_sdmmc_pm_suspend, rtk_sdmmc_pm_resume)
};
#endif

static void rtk_sdmmc_reset(struct rtk_sdmmc_host *rtk_host)
{
	void __iomem *sdmmc_base = rtk_host->sdmmc;

	writel(0x00000000, sdmmc_base + CR_SD_DMA_CTL3);
	rtk_host->data_xfer_mode = CMD_RSP_ONLY;
	writel(0x00000016, sdmmc_base + CR_SD_ISREN);
	writel(0x00000007, sdmmc_base + CR_SD_ISREN);
	writeb(0x00, sdmmc_base + SD_TRANSFER);
	writeb(0xFF, sdmmc_base + CR_CARD_STOP); //SD Card module transfer stop and idle state.
	writeb(0x00, sdmmc_base + CR_CARD_STOP); //SD Card module transfer start.
}

void rtk_sdmmc_sync(struct rtk_sdmmc_host *rtk_host)
{
	rtk_crsd_sync(rtk_host->sysbrdg);
}

/*
 * Quiesce the engine after a transfer that did not finish.
 *
 * A timeout leaves the SD module mid-transfer with DMA still armed. The
 * caller is about to dma_unmap_sg() and hand those pages back, and the
 * controller would go on writing into them, so stop it first. Resetting
 * this DMA disturbs the eMMC engine, which shares the block, so wait for
 * that to go idle beforehand, as the driver already does elsewhere.
 */
static void rtk_sdmmc_abort_transfer(struct rtk_sdmmc_host *rtk_host)
{
	void __iomem *sdmmc_base = rtk_host->sdmmc;
	unsigned long timeout;

	timeout = jiffies + msecs_to_jiffies(100);
	while (time_before(jiffies, timeout)) {
		if (!(readl(rtk_host->emmc + EMMC_DMA_CTL3) & 0x01))
			break;
		cpu_relax();
	}

	writel(0x00000000, sdmmc_base + CR_SD_DMA_CTL3);
	rtk_host->data_xfer_mode = CMD_RSP_ONLY;

	writel(0x00000016, sdmmc_base + CR_SD_ISREN);
	writel(0x00000007, sdmmc_base + CR_SD_ISREN);

	writeb(0x00, sdmmc_base + SD_TRANSFER);
	writeb(0xFF, sdmmc_base + CR_CARD_STOP);
	writeb(0x00, sdmmc_base + CR_CARD_STOP);

	rtk_sdmmc_sync(rtk_host);
}

static u8 rtk_sdmmc_get_rsp_type(struct mmc_command *cmd)
{
	u32 rsp_type = 0;
	u32 rsp_type_chk = mmc_resp_type(cmd);

	if (rsp_type_chk == MMC_RSP_R1)
		rsp_type = SD_R1;
	else if (rsp_type_chk == MMC_RSP_R1B)
		rsp_type = SD_R1b;
	else if (rsp_type_chk == MMC_RSP_R2)
		rsp_type = SD_R2;
	else if (rsp_type_chk == MMC_RSP_R3)
		rsp_type = SD_R3;
	else if (rsp_type_chk == MMC_RSP_R6)
		rsp_type = SD_R6;
	else if (rsp_type_chk == MMC_RSP_R7)
		rsp_type = SD_R7;
	else
		rsp_type = SD_R0;

	return rsp_type;
}

static u8 rtk_sdmmc_get_rsp_len(u8 rsp_para)
{
	return rtk_crsd_get_rsp_len(rsp_para);
}

static u8 rtk_sdmmc_search_final_phase(struct rtk_sdmmc_host *rtk_host, u32 phase_map)
{
	struct timing_phase_path path[MAX_PHASE + 1];
	struct timing_phase_path swap;
	int i = 0;
	int j = 0;
	int k = 0;
	int cont_path_cnt = 0;
	int new_block = 1;
	int max_len = 0;
	u8 final_phase = 0xFF;

	/* Parse phase_map, take it as a bit-ring */
	for (i = 0 ; i < MAX_PHASE + 1 ; i++) {
		if (phase_map & (1 << i)) {
			if (new_block) {
				new_block = 0;
				j = cont_path_cnt++;
				path[j].start = i;
				path[j].end = i;
			} else {
				path[j].end = i;
			}
		} else {
			new_block = 1;
			if (cont_path_cnt) {
				/* Calculate path length and middle point */
				int idx = cont_path_cnt - 1;

				path[idx].len = path[idx].end - path[idx].start + 1;
				path[idx].mid = path[idx].start + path[idx].len / 2;
			}
		}
	}

	if (cont_path_cnt == 0) {
		pr_debug("rtk-sdmmc:  %s No continuous phase path\n", __func__);
		goto finish;
	} else {
		/* Calculate last continuous path length and middle point */
		int idx = cont_path_cnt - 1;

		path[idx].len = path[idx].end - path[idx].start + 1;
		path[idx].mid = path[idx].start + path[idx].len / 2;
	}

	/* Connect the first and last continuous paths if they are adjacent */
	if (!path[0].start && path[cont_path_cnt - 1].end == MAX_PHASE) {
		/* Using negative index */
		path[0].start = path[cont_path_cnt - 1].start - MAX_PHASE - 1;
		path[0].len += path[cont_path_cnt - 1].len;
		path[0].mid = path[0].start + path[0].len / 2;
		/* Convert negative middle point index to positive one */
		if (path[0].mid < 0)
			path[0].mid += MAX_PHASE + 1;
		cont_path_cnt--;
	}

	/* Sorting path array,jamestai20141223 */
	for (k = 0 ; k < cont_path_cnt ; ++k) {
		for (i = 0 ; i < cont_path_cnt - 1 - k ; ++i) {
			if (path[i].len < path[i + 1].len) {
				swap.end = path[i + 1].end;
				swap.len = path[i + 1].len;
				swap.mid = path[i + 1].mid;
				swap.start = path[i + 1].start;

				path[i + 1].end = path[i].end;
				path[i + 1].len = path[i].len;
				path[i + 1].mid = path[i].mid;
				path[i + 1].start = path[i].start;

				path[i].end = swap.end;
				path[i].len = swap.len;
				path[i].mid = swap.mid;
				path[i].start = swap.start;
			}
		}
	}

	/* Choose the longest continuous phase path */
	max_len = 0;
	final_phase = 0;
	for (i = 0 ; i < cont_path_cnt ; i++) {
		if (path[i].len > max_len) {
			max_len = path[i].len;
			if (max_len > 6)	//for compatibility issue, continue len should bigger than 6
				final_phase = (u8)path[i].mid;
			else
				final_phase = 0xFF;
		}

		pr_debug("rtk-sdmmc: %s path[%d].start = %d\n", __func__, i, path[i].start);
		pr_debug("rtk-sdmmc: %s path[%d].end = %d\n", __func__, i, path[i].end);
		pr_debug("rtk-sdmmc: %s path[%d].len = %d\n", __func__, i, path[i].len);
		pr_debug("rtk-sdmmc: %s path[%d].mid = %d\n", __func__, i, path[i].mid);
	}

finish:
	pr_debug("rtk-sdmmc: %s Final chosen phase: %d\n", __func__, final_phase);
	return final_phase;
}

static int rtk_sdmmc_change_tx_phase(struct rtk_sdmmc_host *rtk_host, u8 sample_point)
{
	void __iomem *pll_base = rtk_host->pll;
	unsigned int temp_reg = 0;

	writel(readl(pll_base + 0x1E0) & 0xfffffffd, pll_base + 0x1E0);
	temp_reg = readl(pll_base + CR_PLL_SD1);
	temp_reg = (temp_reg & ~0x000000F8) | (sample_point << 3);
	writel(temp_reg,  pll_base + CR_PLL_SD1);
	writel(readl(pll_base + 0x1E0) | 0x2, pll_base + 0x1E0);
	rtk_sdmmc_sync(rtk_host);
	udelay(100);

	return 0;
}

static int rtk_sdmmc_change_rx_phase(struct rtk_sdmmc_host *rtk_host, u8 sample_point)
{
	void __iomem *pll_base = rtk_host->pll;
	unsigned int temp_reg = 0;

	writel(readl(pll_base + 0x1E0) & 0xfffffffd, pll_base + 0x1E0);
	temp_reg = readl(pll_base + CR_PLL_SD1);
	temp_reg = (temp_reg & ~0x00001F00) | (sample_point << 8);
	writel(temp_reg, pll_base + CR_PLL_SD1);
	writel(readl(pll_base + 0x1E0) | 0x2, pll_base + 0x1E0);
	rtk_sdmmc_sync(rtk_host);
	udelay(100);

	return 0;
}

static int rtk_sdmmc_tuning_tx_cmd(struct rtk_sdmmc_host *rtk_host, u8 sample_point)
{
	struct mmc_command cmd;
	struct sdmmc_cmd_pkt cmd_info;
	void __iomem *sdmmc_base = rtk_host->sdmmc;

	rtk_sdmmc_change_tx_phase(rtk_host, sample_point);

	memset(&cmd, 0x00, sizeof(struct mmc_command));
	memset(&cmd_info, 0x00, sizeof(struct sdmmc_cmd_pkt));

	cmd.opcode = MMC_SEND_STATUS;
	cmd.arg = (rtk_host->sdmmc_rca << RCA_SHIFTER);
	cmd_info.cmd = &cmd;
	cmd_info.rtk_host = rtk_host;
	cmd_info.rsp_para2 = 0x41;
	cmd_info.rsp_len = rtk_sdmmc_get_rsp_len(0x41);

	rtk_sdmmc_send_cmd_get_rsp(&cmd_info);
	rtk_host->pre_cmd = cmd.opcode;

	if (readb(sdmmc_base + SD_TRANSFER) & 0x10)
		return -1;
	return 0;
}

static int rtk_sdmmc_tuning_rx_cmd(struct rtk_sdmmc_host *rtk_host, u8 sample_point,
				   struct scatterlist *p_sg)
{
	struct sdmmc_cmd_pkt cmd_info;
	struct mmc_request mrq = {NULL};
	struct mmc_command cmd = {0};
	struct mmc_data data = {0};
	void __iomem *sdmmc_base = rtk_host->sdmmc;

	rtk_sdmmc_change_rx_phase(rtk_host, sample_point);
	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.cmd->data = mrq.data;
	mrq.data->error = 0;
	mrq.data->mrq = &mrq;

	cmd.opcode = MMC_SEND_TUNING_BLOCK;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = 64;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = p_sg;
	data.sg_len = 1;

	mrq.cmd->error = 0;
	mrq.cmd->mrq = &mrq;
	mrq.cmd->data = mrq.data;
	mrq.data->error = 0;
	mrq.data->mrq = &mrq;

	memset(&cmd_info, 0x00, sizeof(struct sdmmc_cmd_pkt));
	cmd_info.cmd = mrq.cmd;
	cmd_info.rtk_host = rtk_host;
	cmd_info.rsp_para2 = rtk_sdmmc_get_rsp_type(cmd_info.cmd);
	cmd_info.rsp_len = rtk_sdmmc_get_rsp_len(cmd_info.rsp_para2);
	cmd_info.data = mrq.cmd->data;

	rtk_sdmmc_stream(&cmd_info);
	rtk_host->pre_cmd = cmd.opcode;

	if (readb(sdmmc_base + SD_STATUS1) & 0x01)
		return -1;

	return 0;
}

static int rtk_sdmmc_tuning_tx(struct rtk_sdmmc_host *rtk_host)
{
	int sample_point;
	int ret = 0;
	int i = 0;
	u32 raw_phase_map[TUNING_CNT] = {0};
	u32 phase_map = 0;
	u8 final_phase = 0;

	for (sample_point = 0 ; sample_point <= MAX_PHASE ; sample_point++) {
		for (i = 0 ; i < TUNING_CNT ; i++) {
			if (!(rtk_host->rtflags & RTKCR_FCARD_DETECTED)) {
				ret = -MMC_ERR_RMOVE;
				goto out;
			}

			ret = rtk_sdmmc_tuning_tx_cmd(rtk_host, (u8)sample_point);
			if (ret == 0)
				raw_phase_map[i] |= (1 << sample_point);
		}
	}

	phase_map = 0xFFFFFFFF;
	for (i = 0 ; i < TUNING_CNT ; i++) {
		pr_debug("rtk-sdmmc: %s TX raw_phase_map[%d] = 0x%08x\n", __func__, i, raw_phase_map[i]);
		phase_map &= raw_phase_map[i];
	}

	pr_debug("%s TX phase_map = 0x%08x\n", __func__, phase_map);

	if (phase_map) {
		final_phase = rtk_sdmmc_search_final_phase(rtk_host, phase_map);
		pr_debug("rtk-sdmmc: %s final phase = 0x%08x\n", __func__, final_phase);
		if (final_phase == 0xFF) {
			pr_debug("rtk-sdmmc: %s final phase = 0x%08x\n", __func__, final_phase);
			ret = -EINVAL;
			goto out;
		}
		rtk_sdmmc_change_tx_phase(rtk_host, final_phase);
		ret = 0;
		goto out;
	} else {
		pr_debug("rtk-sdmmc: %s  fail !phase_map\n", __func__);
		ret = -EINVAL;
		goto out;
	}

out:
	return ret;
}

static int rtk_sdmmc_tuning_rx(struct rtk_sdmmc_host *rtk_host)
{
	int sample_point = 0;
	int ret = 0;
	int i = 0;
	u32 raw_phase_map[TUNING_CNT] = {0};
	u32 phase_map = 0;
	u8 final_phase = 0;
	u8 *ssr;
	struct scatterlist sg;

	ssr = kmalloc(512, GFP_KERNEL | GFP_DMA);
	if (!ssr)
		return -ENOMEM;

	sg_init_one(&sg, ssr, 512);
	for (sample_point = 0 ; sample_point <= MAX_PHASE ; sample_point++) {
		for (i = 0 ; i < TUNING_CNT ; i++) {
			if (!(rtk_host->rtflags & RTKCR_FCARD_DETECTED)) {
				ret = -MMC_ERR_RMOVE;
				goto out;
			}

			ret = rtk_sdmmc_tuning_rx_cmd(rtk_host, (u8)sample_point, &sg);
			if (ret == 0)
				raw_phase_map[i] |= (1 << sample_point);
		}
	}

	phase_map = 0xFFFFFFFF;
	for (i = 0 ; i < TUNING_CNT ; i++) {
		pr_debug("rtk-sdmmc: %s RX raw_phase_map[%d] = 0x%08x\n", __func__, i, raw_phase_map[i]);
		phase_map &= raw_phase_map[i];
	}
	pr_debug("%s RX phase_map = 0x%08x\n", __func__, phase_map);

	if (phase_map) {
		final_phase = rtk_sdmmc_search_final_phase(rtk_host, phase_map);
		pr_debug("rtk-sdmmc: %s final phase = 0x%08x\n", __func__, final_phase);
		if (final_phase == 0xFF) {
			pr_debug("rtk-sdmmc: %s final phase = 0x%08x\n", __func__, final_phase);
			ret = -EINVAL;
			goto out;
		}
		rtk_sdmmc_change_rx_phase(rtk_host, final_phase);
		ret = 0;
		goto out;
	} else {
		pr_debug("rtk-sdmmc: %s  fail !phase_map\n", __func__);
		ret = -EINVAL;
		goto out;
	}

out:
	kfree(ssr);
	return ret;
}

static int rtk_sdmmc_wait_voltage_stable_low(struct rtk_sdmmc_host *rtk_host)
{
	u8 status = 0;
	u8 i = 0;
	void __iomem *sdmmc_base = rtk_host->sdmmc;

	while (1) {
		status = readb(sdmmc_base + SD_BUS_STATUS);
		if ((status & (SD_DAT3_0_LEVEL | SD_CMD_LEVEL)) == 0x0)
			break;
		usleep_range(3000, 5000);
		if (i++ > 100)
			break;
	}

	return 0;
}

static int rtk_sdmmc_wait_voltage_stable_high(struct rtk_sdmmc_host *rtk_host)
{
	u8 status = 0;
	u8 i = 0;
	void __iomem *sdmmc_base = rtk_host->sdmmc;

	while (1) {
		status = readb(sdmmc_base + SD_BUS_STATUS);
		if ((status & (SD_DAT3_0_LEVEL | SD_CMD_LEVEL)) == (SD_DAT3_0_LEVEL | SD_CMD_LEVEL))
			break;
		usleep_range(3000, 5000);
		if (i++ > 100)
			break;
	}

	return 0;
}

static void rtk_sdmmc_set_access_mode(struct rtk_sdmmc_host *rtk_host, u8 level)
{
	void __iomem *sdmmc_base = rtk_host->sdmmc;
	u32 tmp_bits = 0;

	tmp_bits = readb(sdmmc_base + SD_CONFIGURE1) & ~MODE_SEL_MASK;

	if (level == ACCESS_MODE_SD20)
		tmp_bits |= MODE_SEL_SD20;
	else if (level == ACCESS_MODE_DDR)
		tmp_bits |= MODE_SEL_DDR;
	else if (level == ACCESS_MODE_SD30)
		tmp_bits |= (MODE_SEL_SD30 | SD30_ASYNC_FIFO_RST);
	else
		tmp_bits |= MODE_SEL_SD20;

	writeb(tmp_bits, sdmmc_base + SD_CONFIGURE1);
}

static void rtk_sdmmc_set_bits(struct rtk_sdmmc_host *rtk_host, u8 set_bit)
{
	void  *sdmmc_base = rtk_host->sdmmc;
	u32 tmp_bits = 0;

	tmp_bits = readb(sdmmc_base + SD_CONFIGURE1);
	if ((tmp_bits & MASK_BUS_WIDTH) != set_bit) {
		tmp_bits &= ~MASK_BUS_WIDTH;
		writeb(tmp_bits | set_bit, sdmmc_base + SD_CONFIGURE1);
	}
}

static void rtk_sdmmc_set_div(struct rtk_sdmmc_host *rtk_host, u32 set_div)
{
	void __iomem *sdmmc_base = rtk_host->sdmmc;
	u8 tmp_div = 0;

	rtk_sdmmc_sync(rtk_host);
	tmp_div = readb(sdmmc_base + SD_CONFIGURE1) & ~MASK_CLOCK_DIV;
	if (set_div != CLOCK_DIV_NON)
		writeb(tmp_div | set_div | SDCLK_DIV, sdmmc_base + SD_CONFIGURE1);
	else
		writeb(tmp_div, sdmmc_base + SD_CONFIGURE1);
	rtk_sdmmc_sync(rtk_host);
}

static void rtk_sdmmc_set_speed(struct rtk_sdmmc_host *rtk_host, u8 level)
{
	void __iomem *sdmmc_base = rtk_host->sdmmc;

	switch (level) {
	case 0: //ddr50 , highest speed
		pr_debug("rtk-sdmmc: %s: speed 2100\n", __func__);
		writel(0x00002100, sdmmc_base + CR_SD_CKGEN_CTL);
		break;
	case 1:
		pr_debug("rtk-sdmmc: %s: speed 2101\n", __func__);
		writel(0x00002101, sdmmc_base + CR_SD_CKGEN_CTL);
		break;
	case 2:
		pr_debug("rtk-sdmmc: %s: speed 2102\n", __func__);
		writel(0x00002102, sdmmc_base + CR_SD_CKGEN_CTL);
		break;
	case 3:
		pr_debug("rtk-sdmmc: %s: speed 2103\n", __func__);
		writel(0x00002103, sdmmc_base + CR_SD_CKGEN_CTL);
		break;
	default:
		pr_debug("rtk-sdmmc: %s: default speed 2102\n", __func__);
		writel(0x00002102, sdmmc_base + CR_SD_CKGEN_CTL);
		break;
	}
	rtk_sdmmc_sync(rtk_host);
}

static void rtk_sdmmc_speed(struct rtk_sdmmc_host *rtk_host, enum sdmmc_clock_speed sd_speed)
{
	void __iomem *sdmmc_base = rtk_host->sdmmc;
	void __iomem *pll_base = rtk_host->pll;

	switch (sd_speed) {
	case SDMMC_CLOCK_200KHZ:
		pr_debug("rtk-sdmmc: %s: speed SDMMC_CLOCK_200KHZ\n", __func__);
		rtk_sdmmc_set_div(rtk_host, CLOCK_DIV_256); //0x580 = 0xd0
		rtk_sdmmc_set_speed(rtk_host, 1); //0x478 = 0x2101
		break;
	case SDMMC_CLOCK_400KHZ:
		pr_debug("rtk-sdmmc: %s: speed SDMMC_CLOCK_400KHZ\n", __func__);
		rtk_sdmmc_set_div(rtk_host, CLOCK_DIV_256); //0x580 = 0xd0
		rtk_sdmmc_set_speed(rtk_host, 0); //0x478 = 0x2100
		break;
	case SDMMC_CLOCK_6200KHZ:
		pr_debug("rtk-sdmmc: %s: speed SDMMC_CLOCK_6200KHZ\n", __func__);
		rtk_sdmmc_set_div(rtk_host, CLOCK_DIV_NON); //0x580 = 0x10
		rtk_sdmmc_set_speed(rtk_host, 3); //0x478 = 0x2103
		break;
	case SDMMC_CLOCK_25000KHZ:
		pr_debug("rtk-sdmmc: %s: speed SDMMC_CLOCK_25000KHZ\n", __func__);
		if (rtk_host->mmc->ios.timing == MMC_TIMING_UHS_SDR12) {
			rtk_sdmmc_set_div(rtk_host, CLOCK_DIV_NON); //0x580 = 0x10
			rtk_sdmmc_set_speed(rtk_host, 2); //0x478 = 0x2101
		} else {
			rtk_sdmmc_set_div(rtk_host, CLOCK_DIV_NON); //0x580 = 0x10
			rtk_sdmmc_set_speed(rtk_host, 1); //0x478 = 0x2101
		}
		break;
	case SDMMC_CLOCK_50000KHZ:
		pr_debug("rtk-sdmmc: %s: speed SDMMC_CLOCK_50000KHZ\n", __func__);
		if (rtk_host->mmc->ios.timing == MMC_TIMING_UHS_DDR50) {
			writeb(0X3F, sdmmc_base + CARD_SD_CLK_PAD_DRIVE);
			writeb(0X3F, sdmmc_base + CARD_SD_CMD_PAD_DRIVE);
			writeb(0X3F, sdmmc_base + CARD_SD_DAT_PAD_DRIVE);
			writel(readl(sdmmc_base + CR_SD_CKGEN_CTL) | 0x00070000,
			       sdmmc_base + CR_SD_CKGEN_CTL); //Switch SD source clock to 4MHz by Hsin-yin
			writel(0x00000006, pll_base + 0x01EC); //JIM modified, 1AC->1EC
			writel(0x00324388,  pll_base + CR_PLL_SD3);
			mdelay(2);
			writel(0x00000007, pll_base + 0x01EC); //JIM modified, 1AC->1EC
			udelay(200);
			writel(readl(sdmmc_base + CR_SD_CKGEN_CTL) & 0xFFF8FFFF,
			       sdmmc_base + CR_SD_CKGEN_CTL); //Switch SD source clock to normal clock source by Hsin-yin
			writeb(readb(sdmmc_base + SD_CONFIGURE1) & 0xEF,
			       sdmmc_base + SD_CONFIGURE1); //Reset FIFO pointer by Hsin-yin
			rtk_sdmmc_set_div(rtk_host, CLOCK_DIV_NON); //0x580 = 0x10
			rtk_sdmmc_set_speed(rtk_host, 0); //0x478 = 0x2100
		} else if (rtk_host->mmc->ios.timing == MMC_TIMING_UHS_SDR25) {
			rtk_sdmmc_set_div(rtk_host, CLOCK_DIV_NON); //0x580 = 0x10
			rtk_sdmmc_set_speed(rtk_host, 1); //0x478 = 0x2101
		} else {
			rtk_sdmmc_set_div(rtk_host, CLOCK_DIV_NON); //0x580 = 0x10
			rtk_sdmmc_set_speed(rtk_host, 0); //0x478 = 0x2100
		}
		break;
	case SDMMC_CLOCK_100000KHZ:
		pr_debug("rtk-sdmmc: %s: speed SDMMC_CLOCK_100000KHZ\n", __func__);

		writeb(0x3F, sdmmc_base + CARD_SD_CLK_PAD_DRIVE);
		writeb(0x3F, sdmmc_base + CARD_SD_CMD_PAD_DRIVE);
		writeb(0x3F, sdmmc_base + CARD_SD_DAT_PAD_DRIVE);
		rtk_sdmmc_set_div(rtk_host, CLOCK_DIV_NON); //0x580 = 0x10
		rtk_sdmmc_set_speed(rtk_host, 0); //0x478 = 0x2100
		break;
	case SDMMC_CLOCK_208000KHZ:
		pr_debug("rtk-sdmmc: %s: speed SDMMC_CLOCK_208000KHZ\n", __func__);
		if (rtk_host->magic_num == 0x0 || rtk_host->magic_num == 0x1) {
			writeb(0X3F, sdmmc_base + CARD_SD_CLK_PAD_DRIVE);
			writeb(0X3F, sdmmc_base + CARD_SD_CMD_PAD_DRIVE);
			writeb(0X3F, sdmmc_base + CARD_SD_DAT_PAD_DRIVE);

			rtk_sdmmc_set_div(rtk_host, CLOCK_DIV_NON); //0x580 = 0x10
			rtk_sdmmc_set_speed(rtk_host, 0);                  //0x478 = 0x2100
		} else {
			writeb(0Xbb, sdmmc_base + CARD_SD_CLK_PAD_DRIVE);
			writeb(0Xbb, sdmmc_base + CARD_SD_CMD_PAD_DRIVE);
			writeb(0Xbb, sdmmc_base + CARD_SD_DAT_PAD_DRIVE);
			if (rtk_host->card_manfid == 0x3) {
				writel(readl(sdmmc_base + CR_SD_CKGEN_CTL) | 0x00070000,
				       sdmmc_base + CR_SD_CKGEN_CTL); //Switch SD source clock to 4MHz by Hsin-yin
				writel(0x00a54388, sdmmc_base + CR_PLL_SD3);      //bus clock to 208MHz jamestai20150319
			} else {
				writel(readl(sdmmc_base + CR_SD_CKGEN_CTL) | 0x00070000,
				       sdmmc_base + CR_SD_CKGEN_CTL); //Switch SD source clock to 4MHz by Hsin-yin
				writel(0x00b64388, sdmmc_base + CR_PLL_SD3);      //bus clock to 208MHz jamestai20150319
			}
		}
		udelay(2);

		writel(0x00000006, pll_base + 0x01EC); //JIM modified, 1AC->1EC
		writel(readl(sdmmc_base + CR_SD_CKGEN_CTL) | 0x00070000,
		       sdmmc_base + CR_SD_CKGEN_CTL); //Switch SD source clock to 4MHz by Hsin-yin
		mdelay(2);
		writel(0x00000007, pll_base + 0x01EC); //JIM modified, 1AC->1EC
		udelay(200);

		writel(0x00000006, pll_base + 0x01EC); //JIM modified, 1AC->1EC
		writel(readl(sdmmc_base + CR_SD_CKGEN_CTL) & 0xFFF8FFFF,
		       sdmmc_base + CR_SD_CKGEN_CTL); //Switch SD source clock to normal clock source by Hsin-yin
		mdelay(2);
		writel(0x00000007, pll_base + 0x01EC); //JIM modified, 1AC->1EC
		udelay(200);
		writeb(readb(sdmmc_base + SD_CONFIGURE1) & 0xEF,
		       sdmmc_base + SD_CONFIGURE1); //Reset FIFO pointer by Hsin-yin

		rtk_sdmmc_set_div(rtk_host, CLOCK_DIV_NON); //0x580 = 0x10
		rtk_sdmmc_set_speed(rtk_host, 0); //0x478 = 0x2100
		break;
	default:
		pr_debug("rtk-sdmmc: %s: default speed SDMMC_CLOCK_400KHZ\n", __func__);
		rtk_sdmmc_set_div(rtk_host, CLOCK_DIV_256); //0x580 = 0xd0
		rtk_sdmmc_set_speed(rtk_host, 0); //0x478 = 0x2100
		break;
	}

	rtk_sdmmc_sync(rtk_host);
}

static int rtk_sdmmc_allocate_dma_buf(struct rtk_sdmmc_host *rtk_host, struct mmc_command *cmd)
{
	if (!rtk_host->dma_virt_addr) {
		rtk_host->dma_virt_addr = dma_alloc_coherent(rtk_host->dev, SD_ALLOC_LENGTH, &rtk_host->paddr,
							     GFP_KERNEL);
		rtk_host->dma_phy_addr = (u32 *)rtk_host->paddr;
	} else {
		return 0;
	}

	if (!rtk_host->dma_virt_addr) {
		WARN_ON(1);
		pr_err("%s: allocate rtk sd dma buf FAIL !\n", __func__);
		cmd->error = -ENOMEM;
		return 0;
	}

	return 1;
}

static int rtk_sdmmc_free_dma_buf(struct rtk_sdmmc_host *rtk_host)
{
	if (rtk_host->dma_virt_addr)
		dma_free_coherent(rtk_host->dev, SD_ALLOC_LENGTH, rtk_host->dma_virt_addr, rtk_host->paddr);
	else
		return 0;

	/*
	 * Now that dma_virt_addr/dma_phy_addr are per-host fields, this must
	 * re-arm the "not yet allocated" NULL-check that
	 * rtk_sdmmc_allocate_dma_buf() gates on -- previously a module-scope
	 * global that stayed non-NULL forever after the single host's one
	 * remove(), which never mattered with only one instance. Left
	 * dangling, a second host reusing this code path would see a stale
	 * non-NULL pointer and skip allocating its own DMA buffer entirely.
	 */
	rtk_host->dma_virt_addr = NULL;
	rtk_host->dma_phy_addr = NULL;

	return 1;
}

static u32 rtk_sdmmc_swap_endian(u32 input)
{
	u32 output = 0;

	output = (input & 0xFF000000) >> 24 |
			(input & 0x00FF0000) >> 8 |
			(input & 0x0000FF00) << 8 |
			(input & 0x000000FF) << 24;
	return output;
}

static void rtk_sdmmc_read_rsp(struct rtk_sdmmc_host *rtk_host, u32 *rsp, int reg_count)
{
	void __iomem *sdmmc_base = rtk_host->sdmmc;

	if (reg_count == 6) {
		rsp[0] = readb(sdmmc_base + SD_CMD1) << 24 |
				readb(sdmmc_base + SD_CMD2) << 16 |
				readb(sdmmc_base + SD_CMD3) << 8 |
				readb(sdmmc_base + SD_CMD4);
	} else if (reg_count == 16) {
		rsp[0] = rtk_sdmmc_swap_endian(rsp[0]);
		rsp[1] = rtk_sdmmc_swap_endian(rsp[1]);
		rsp[2] = rtk_sdmmc_swap_endian(rsp[2]);
		rsp[3] = rtk_sdmmc_swap_endian(rsp[3]);
	}
}

static u32 rtk_sdmmc_get_cmd_timeout(struct sdmmc_cmd_pkt *cmd_info)
{
	struct rtk_sdmmc_host *rtk_host = cmd_info->rtk_host;
	u32 timeout = 0;

	timeout += msecs_to_jiffies(1);
	cmd_info->timeout = rtk_host->timeout = timeout;

	return 0;
}

static int rtk_sdmmc_int_wait(char *drv_name, struct rtk_sdmmc_host *rtk_host, u8 cmdcode)
{
	int ret = CR_TRANSFER_TO;
	unsigned long timeout = 0;
	unsigned int sd_trans = 0;
	unsigned long old_jiffles = jiffies;
	void __iomem *sdmmc_base = rtk_host->sdmmc;
	int cmd_type = 0;

	rtk_host->poll = 0;

	if (rtk_host->broken_flag)
		return CR_TRANSFER_FAIL;

	/* timeout timer fire */
	if (&rtk_host->timer) {
		if (rtk_host && rtk_host->mrq && rtk_host->mrq->cmd &&
		    (rtk_host->mrq->cmd->opcode == 55 || rtk_host->mrq->cmd->opcode == 41 ||
		     rtk_host->mrq->cmd->opcode == 8 ||
		     (rtk_host->pre_cmd == 41 && rtk_host->mrq->cmd->opcode == 1))) {
			timeout = msecs_to_jiffies(100) + rtk_host->timeout;
			cmd_type = 1;
		} else if (rtk_host && rtk_host->mrq && rtk_host->mrq->cmd &&
			   (rtk_host->mrq->cmd->opcode == 2 || rtk_host->mrq->cmd->opcode == 3 ||
			    rtk_host->mrq->cmd->opcode == 9)) {
			timeout = msecs_to_jiffies(600) + rtk_host->timeout;
			cmd_type = 2;
		} else {
			timeout = msecs_to_jiffies(3000) + rtk_host->timeout;
			cmd_type = 3;
		}
		mod_timer(&rtk_host->timer, (jiffies + timeout));
	}

	writel(0x00000016, sdmmc_base + CR_SD_ISREN); //disable all
	switch (rtk_host->data_xfer_mode) {
	case CMD_RSP_ONLY:
	case DATA_WRITE_ONLY:
	case CMD_RSP_DATA_WRITE:
	case CMD_RSP_DATA_READ_TUNNING:
		writel(0x00000007, sdmmc_base + CR_SD_ISREN);
		break;
	case CMD_RSP_DATA_READ:
		writel(0x00000015, sdmmc_base + CR_SD_ISREN);
		break;
	default:
		writel(0x00000007, sdmmc_base + CR_SD_ISREN);
		break;
	}
	rtk_sdmmc_sync(rtk_host);
	rtk_host->irq_error_bit = 0;

	/*
	 * The vendor RTD1195 driver (rtk_crsd_ops.c, rtk_crsd_int_waitfor())
	 * never waits on this interrupt at all in production -- built with
	 * ENABLE_SD_INT_MODE undefined, so int_enable() is a no-op and
	 * completion is entirely via a bounded ~300ms register poll
	 * (SD_TRANSFER END_STATE|IDLE_STATE), with request_irq() and the
	 * ISR itself compiled out entirely. On this port the interrupt
	 * itself is reliable (confirmed on hardware: 6400+ consecutive
	 * commands via a stress test, zero timeouts) -- an earlier
	 * multi-minute stall that looked like a missed interrupt turned out
	 * to be an unrelated userspace process (smartd) queuing many of its
	 * own commands ahead of ours on the shared per-host request queue.
	 * Bounding the wait to the vendor's ~300ms and falling through to
	 * the existing polling loop below is still worth keeping as a
	 * defensive backstop matching vendor precedent, not because the
	 * interrupt is known to be unreliable. rtk_crsd_fire_and_wait()
	 * (shared with rtkemmc_rtd119x.c, which used to wait unbounded on
	 * this same class of interrupt) also reinit_completion()s
	 * int_waiting first, clearing any stale completion from a previous
	 * command's interrupt arriving late -- without that, a late
	 * complete() would satisfy *this* command's wait immediately
	 * instead of its own.
	 */
	rtk_crsd_fire_and_wait(sdmmc_base, rtk_host->int_waiting, cmdcode);

	rtk_sdmmc_sync(rtk_host);

	sd_trans = readb(sdmmc_base + SD_TRANSFER);

	if (rtk_host->irq_error_bit == 1) {      //error handling
		if (rtk_host->cmd_opcode == 55 && rtk_host->CMD8_SD1) {
			rtk_host->CMD55_MMC = true;   //MMC card
			pr_err("Reject SD 1.x command\n");
			ret = CR_TRANSFER_FAIL;
		} else {
			if (rtk_host)
				pr_err("%s: reset the card and adjust the driving capacity, cmd=%d, SD_TRANSFER=0x%x\n",
				       __func__, rtk_host->cmd_opcode, sd_trans);
			rtk_host->driving_capacity++;
			rtk_host->compatible_flag = true;
			remove_sdcard(rtk_host);
			ret = CR_TRANS_OK;
		}
	} else if ((sd_trans & (END_STATE | IDLE_STATE)) == (END_STATE | IDLE_STATE)) {
		ret = CR_TRANS_OK;
	} else {
		if (cmd_type == 1)
			timeout = 10;
		else if (cmd_type == 2)
			timeout = 60;
		else
			timeout = 300;
		while (time_before(jiffies, old_jiffles + timeout)) {
			rtk_sdmmc_sync(rtk_host);
			sd_trans = readb(sdmmc_base + SD_TRANSFER);

			if ((sd_trans & (END_STATE | IDLE_STATE)) == (END_STATE | IDLE_STATE)) {
				ret = CR_TRANS_OK;
				break;
			} else if ((sd_trans & (ERR_STATUS)) == (ERR_STATUS)) {
				ret = CR_TRANSFER_FAIL;
				pr_err("%s(%d) trans error(error status) :\n cfg1: 0x%02x, cfg2: 0x%02x, cfg3: 0x%02x, trans: 0x%08x, st1: 0x%08x, st2: 0x%08x, bus: 0x%08x\n",
				       __func__,
					__LINE__,
					readb(sdmmc_base + SD_CONFIGURE1),
					readb(sdmmc_base + SD_CONFIGURE2),
					readb(sdmmc_base + SD_CONFIGURE3),
					readb(sdmmc_base + SD_TRANSFER),
					readb(sdmmc_base + SD_STATUS1),
					readb(sdmmc_base + SD_STATUS2),
					readb(sdmmc_base + SD_BUS_STATUS));

				if (rtk_host && rtk_host->mrq && rtk_host->mrq->cmd)
					pr_err("%s: error opcode = %d\n", __func__, rtk_host->mrq->cmd->opcode);

			if (rtk_host && rtk_host->mrq && rtk_host->mrq->cmd &&
			    rtk_host->mrq->cmd->opcode != 13 && rtk_host->mrq->cmd->opcode != 19 &&
		    !rtk_host->CMD8_SD1 && !rtk_host->CMD55_MMC) {
					rtk_host->driving_capacity++;
					rtk_host->compatible_flag = true;
					remove_sdcard(rtk_host);
					ret = CR_TRANS_OK;
				}
				break;
			} else if (rtk_host && !(rtk_host->rtflags & RTKCR_FCARD_DETECTED)) {
				ret = CR_TRANSFER_FAIL;
				pr_err("card is being removed...\n");
				break;
			}
		}
	}
	if (ret == CR_TRANSFER_TO) {
		pr_err("%s(%d) trans error(timeout) :\n trans: 0x%08x, st1: 0x%08x, st2: 0x%08x, bus: 0x%08x\n",
		       __func__,
				__LINE__,
				readb(sdmmc_base + SD_TRANSFER),
				readb(sdmmc_base + SD_STATUS1),
				readb(sdmmc_base + SD_STATUS2),
				readb(sdmmc_base + SD_BUS_STATUS));

		if (rtk_host && rtk_host->mrq && rtk_host->mrq->cmd)
			pr_err("%s: error opcode = %d\n", __func__, rtk_host->mrq->cmd->opcode);
	}

	/*
	 * Whatever went wrong, the transfer is over as far as the driver is
	 * concerned. Make it over for the hardware too, before the caller
	 * releases the buffers it was writing into.
	 */
	if (ret != CR_TRANS_OK)
		rtk_sdmmc_abort_transfer(rtk_host);

	return ret;
}

static int rtk_sdmmc_set_rspparam(struct sdmmc_cmd_pkt *cmd_info)
{
	pr_debug("rtk-sdmmc: %s: opcode = %d\n", __func__, cmd_info->cmd->opcode);

	switch (cmd_info->cmd->opcode) {
	case MMC_GO_IDLE_STATE: //cmd 0
		cmd_info->rsp_para1 = 0xD0; //SD1_R0;
		cmd_info->rsp_para2 = 0x74; //0x50 | SD_R0;
		cmd_info->rsp_para3 = 0x00;
		break;
	case MMC_SEND_OP_COND:
		if (cmd_info->rtk_host->mmc_detected == 1) {
			cmd_info->rsp_para1 = SD1_R0;
			cmd_info->rsp_para2 = 0x45;
			cmd_info->rsp_para3 = 0x01;
			cmd_info->cmd->arg = MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
					     MMC_VDD_33_34 | MMC_SECTOR_ADDR;
		}
		break;
	case MMC_ALL_SEND_CID: //cmd 2
		cmd_info->rsp_para1 = -1; //SD1_R0;
		cmd_info->rsp_para2 = 0x42; //SD_R2;
		cmd_info->rsp_para3 = 0x05; //SD2_R0;
		break;
	case MMC_SET_RELATIVE_ADDR: //cmd3
		cmd_info->rsp_para1 = -1; //SD1_R0;
		cmd_info->rsp_para2 = 0x41; //SD_R1 | CRC16_CAL_DIS;
		cmd_info->rsp_para3 = 0x05;
		break;
	case MMC_SET_DSR: //cmd4
		cmd_info->rsp_para1 = -1; //don't update
		cmd_info->rsp_para3 = -1; //don't update
		break;
	case SD_APP_SET_BUS_WIDTH: //cmd6
		if ((cmd_info->cmd->flags & (0x3 << 5)) == MMC_CMD_ADTC) { //SD_SWITCH
			cmd_info->rsp_para1 = -1;
			cmd_info->rsp_para2 = 0x01; //SD_R1b | CRC16_CAL_DIS;
			cmd_info->rsp_para3 = 0x05;
		} else {
			cmd_info->rsp_para1 = -1;
			if (cmd_info->rtk_host->mmc_detected == 1) {
				cmd_info->rsp_para2 = 0x09; //SD_R1b | CRC16_CAL_DIS;
				cmd_info->rsp_para3 = 0x05;
			} else {
				cmd_info->rsp_para2 = 0x61; //SD_R1b | CRC16_CAL_DIS;
				cmd_info->rsp_para3 = 0x0D;
			}
		}
		break;
	case MMC_SELECT_CARD: //cmd7
		if (cmd_info->cmd->flags == (MMC_RSP_R1 | MMC_CMD_AC)) { //select
			cmd_info->rsp_para1 = -1;
			cmd_info->rsp_para2 = 0x41; //SD_R1b|CRC16_CAL_DIS;
			cmd_info->rsp_para3 = 0x05;
		} else {	//deselect
			cmd_info->rsp_para1 = -1;
			cmd_info->rsp_para2 = 0x40; //SD_R0|CRC16_CAL_DIS;
			cmd_info->rsp_para3 = 0x05;
		}
		break;
	case SD_SEND_IF_COND: //cmd8
		cmd_info->rsp_para1 = 0xD0;
		cmd_info->rsp_para2 = 0x41; //SD_R1;
		cmd_info->rsp_para3 = 0x0;
		if (cmd_info->rtk_host->rtflags & RTKCR_FCARD_SELECTED && cmd_info->rtk_host->mmc_detected == 1) {
			cmd_info->rsp_para1 = 0x50;
			cmd_info->rsp_para2 = SD_R1;
			cmd_info->rsp_para3 = 0x05;
		}
		break;
	case MMC_SEND_CSD: //cmd9
		cmd_info->rsp_para1 = -1;
		cmd_info->rsp_para2 = 0x42; //SD_R2;
		cmd_info->rsp_para3 = 0x05;
		break;
	case MMC_SEND_CID://cmd10
		cmd_info->rsp_para1 = -1;
		cmd_info->rsp_para2 = 0x42; //SD_R2;
		cmd_info->rsp_para3 = 0x5;
		break;
	case SD_SWITCH_VOLTAGE: //cmd11
		cmd_info->rsp_para1 = -1; //0;
		cmd_info->rsp_para2 = 0x41; //don't update
		cmd_info->rsp_para3 = 0x00; //don't update
		break;
	case MMC_STOP_TRANSMISSION: //cmd12
		cmd_info->rsp_para1 = -1;
	if (!cmd_info->rtk_host->suspend_flag)
			cmd_info->rsp_para2 = 0x7D; //SD_R1 | CRC16_CAL_DIS | 6 bytes response;
		else
			cmd_info->rsp_para2 = 0x7C;
		cmd_info->rsp_para3 = 0x00;
		break;
	case SD_APP_SD_STATUS: //cmd13
		if ((cmd_info->cmd->flags & (0x3 << 5)) == MMC_CMD_ADTC) { //SD_APP_SD_STATUS
			cmd_info->rsp_para1 = -1;
			cmd_info->rsp_para2 = 0x01; //SD_R1 | CRC16_CAL_DIS;
			cmd_info->rsp_para3 = 0x05;
		} else {
			cmd_info->rsp_para1 = -1;
			cmd_info->rsp_para2 = 0x41; //SD_R1 | CRC16_CAL_DIS;
			cmd_info->rsp_para3 = -1;
		}
		break;
	case MMC_GO_INACTIVE_STATE: //cmd15
		cmd_info->rsp_para1 = -1; //don't update
		cmd_info->rsp_para3 = -1; //don't update
		break;
	case MMC_SET_BLOCKLEN: //cmd16
		cmd_info->rsp_para1 = -1;
		cmd_info->rsp_para2 = 0x41;
		cmd_info->rsp_para3 = 0x05;
		break;
	case MMC_READ_SINGLE_BLOCK: //cmd17
		cmd_info->rsp_para1 = -1;
		cmd_info->rsp_para2 = 0x1;
		cmd_info->rsp_para3 = -1;
		break;
	case MMC_READ_MULTIPLE_BLOCK: //cmd18
		cmd_info->rsp_para1 = -1;
		cmd_info->rsp_para2 = 0x01;
		if (cmd_info->rtk_host->mmc_detected == 1)
			cmd_info->rsp_para3 = 0x04; //0x04;
		else
			cmd_info->rsp_para3 = 0x00;//0x02;  ----jim
		break;
	case MMC_SEND_TUNING_BLOCK: //cmd19
		cmd_info->rsp_para1 = -1;
		cmd_info->rsp_para2 = 0x01;
		cmd_info->rsp_para3 = -1; //0x04;
		break;
	case MMC_WRITE_DAT_UNTIL_STOP: //cmd20
		cmd_info->rsp_para1 = -1; //don't update
		cmd_info->rsp_para3 = -1; //don't update
		break;
	case SD_APP_SEND_NUM_WR_BLKS: //cmd22
		cmd_info->rsp_para1 = -1; //don't update
		cmd_info->rsp_para3 = -1; //don't update
		break;
	case MMC_SET_BLOCK_COUNT: //cmd 23
		cmd_info->rsp_para1 = -1; //don't update
		cmd_info->rsp_para3 = -1; //don't update
		break;
	case MMC_WRITE_BLOCK: //cmd24
		cmd_info->rsp_para1 = -1;
		cmd_info->rsp_para2 = 0x01;
		cmd_info->rsp_para3 = -1;
		break;
	case MMC_WRITE_MULTIPLE_BLOCK: //cmd25
		cmd_info->rsp_para1 = -1;
		cmd_info->rsp_para2 = 0x01;
		cmd_info->rsp_para3 = 0x00;//0x02;--------jim
		break;
	case MMC_PROGRAM_CSD: //cmd27
		cmd_info->rsp_para1 = -1; //don't update
		cmd_info->rsp_para3 = -1; //don't update
		break;
	case MMC_SET_WRITE_PROT: //cmd28
		cmd_info->rsp_para1 = -1; //don't update
		cmd_info->rsp_para3 = -1; //don't update
		break;
	case MMC_CLR_WRITE_PROT: //cmd29
		cmd_info->rsp_para1 = -1; //don't update
		cmd_info->rsp_para3 = -1; //don't update
		break;
	case MMC_SEND_WRITE_PROT: //cmd30
		cmd_info->rsp_para1 = -1; //don't update
		cmd_info->rsp_para3 = -1; //don't update
		break;
	case SD_ERASE_WR_BLK_START: //cmd32
		cmd_info->rsp_para1 = -1; //don't update
		cmd_info->rsp_para3 = -1; //don't update
		break;
	case SD_ERASE_WR_BLK_END: //cmd33
		cmd_info->rsp_para1 = -1; //don't update
		cmd_info->rsp_para3 = -1; //don't update
		break;
	case MMC_ERASE: //cmd38
		cmd_info->rsp_para1 = -1; //don't update
		cmd_info->rsp_para3 = -1; //don't update
		break;
	case SD_APP_OP_COND: //cmd41
		cmd_info->rsp_para1 = -1;
		cmd_info->rsp_para2 = 0x45;
		cmd_info->rsp_para3 = 0x00;
		break;
	case MMC_LOCK_UNLOCK: //cmd42
		if ((cmd_info->cmd->flags & (0x3 << 5)) == MMC_CMD_ADTC) { //SD_APP_SD_STATUS
			cmd_info->rsp_para1 = -1; //don't update
			cmd_info->rsp_para3 = -1; //don't update
		} else {
			cmd_info->rsp_para1 = -1; //don't update
			cmd_info->rsp_para3 = -1; //don't update
		}
		break;
	case SD_APP_SEND_SCR: //cmd51
		cmd_info->rsp_para1 = -1;
		cmd_info->rsp_para2 = 0x41;
		cmd_info->rsp_para3 = 0x05;
		break;
	case MMC_APP_CMD: //cmd55
		cmd_info->rsp_para1 = -1;
		cmd_info->rsp_para2 = 0x41;
		cmd_info->rsp_para3 = 0x05;
		break;
	case MMC_GEN_CMD: //cmd56
		cmd_info->rsp_para1 = -1; //don't update
		cmd_info->rsp_para3 = -1; //don't update
		break;
	case MMC_EXT_READ_SINGLE: //cmd48
		cmd_info->rsp_para1 = -1;
		cmd_info->rsp_para2 = 0x01;
		cmd_info->rsp_para3 = -1;
		break;
	case MMC_EXT_WRITE_SINGLE: //cmd49
		cmd_info->rsp_para1 = -1;
		cmd_info->rsp_para2 = 0x01;
		cmd_info->rsp_para3 = -1;
		break;
	case MMC_EXT_READ_MULTIPLE: //cmd58
		cmd_info->rsp_para1 = -1;
		cmd_info->rsp_para2 = 0x01;
		if (cmd_info->rtk_host->mmc_detected == 1)
			cmd_info->rsp_para3 = 0x04;
		else
			cmd_info->rsp_para3 = 0x02;
		break;
	case MMC_EXT_WRITE_MULTIPLE: //cmd59
		cmd_info->rsp_para1 = -1;
		cmd_info->rsp_para2 = 0x01;
		cmd_info->rsp_para3 = 0x02;
		break;
	default:
		cmd_info->rsp_para1 = -1; //don't update
		cmd_info->rsp_para3 = -1; //don't update
		break;
	}

	return 0;
}

static int rtk_sdmmc_stream_cmd(u16 cmdcode, struct sdmmc_cmd_pkt *cmd_info, u8 data_len)
{
	u8 cmd_idx = cmd_info->cmd->opcode;
	u32 sd_arg = 0;
	s8 rsp_para1 = 0;
	s8 rsp_para2 = 0;
	s8 rsp_para3 = 0;
	int rsp_len = cmd_info->rsp_len;
	int ret = 0;
	u32 data = cmd_info->dma_buffer;
	u32 sa = 0;
	u32 *rsp = (u32 *)&cmd_info->cmd->resp;
	u16 byte_count = cmd_info->byte_count;
	u16 block_count = cmd_info->block_count;
	struct rtk_sdmmc_host *rtk_host = cmd_info->rtk_host;
	void __iomem *sdmmc_base = rtk_host->sdmmc;

	rtk_host->INT4_flag = true;

	if (WARN_ON_ONCE(!data || !rsp))
		return -EINVAL;
	rtk_sdmmc_set_rspparam(cmd_info);
	sd_arg = cmd_info->cmd->arg;

	//if (rsp_para1 != -1)
	rsp_para1 = cmd_info->rsp_para1;

	/* Fix CRC error for SD_AUTOWRITE3, jamestai20150325 */
	if (cmdcode == SD_AUTOWRITE3)
		rsp_para2 = 0;
	else
		rsp_para2 = cmd_info->rsp_para2;

	//if (rsp_para3 != -1)
	rsp_para3 = cmd_info->rsp_para3;
	sa = (data / 8);

	if (cmdcode == SD_NORMALWRITE)
		byte_count = 512;
	else if (cmdcode == SD_NORMALREAD && rtk_host->mmc_detected == 1)
		byte_count = 512;

	rtk_host->g_cmd[0] = (0x40 | cmd_idx);
	rtk_host->g_cmd[1] = (sd_arg >> 24) & 0xff;
	rtk_host->g_cmd[2] = (sd_arg >> 16) & 0xff;
	rtk_host->g_cmd[3] = (sd_arg >> 8) & 0xff;
	rtk_host->g_cmd[4] = sd_arg & 0xff;
	rtk_host->g_cmd[5] = 0x00;

	writeb(rtk_host->g_cmd[0], sdmmc_base + SD_CMD0); //0x10
	writeb(rtk_host->g_cmd[1], sdmmc_base + SD_CMD1); //0x14
	writeb(rtk_host->g_cmd[2], sdmmc_base + SD_CMD2); //0x18
	writeb(rtk_host->g_cmd[3], sdmmc_base + SD_CMD3); //0x1C
	writeb(rtk_host->g_cmd[4], sdmmc_base + SD_CMD4); //0x20
	writeb(rtk_host->g_cmd[5], sdmmc_base + SD_CMD5); //0x20

	if (rsp_para1 != -1)
		writeb(readb(sdmmc_base + SD_CONFIGURE1) | rsp_para1, sdmmc_base + SD_CONFIGURE1); //0x0C

	writeb(rsp_para2, sdmmc_base + SD_CONFIGURE2); //0x0C

	if (rsp_para3 != -1)
		writeb(readb(sdmmc_base + SD_CONFIGURE3) | rsp_para3, sdmmc_base + SD_CONFIGURE3); //0x0C

	writeb(byte_count, sdmmc_base + SD_BYTE_CNT_L); //0x24
	writeb(byte_count >> 8, sdmmc_base + SD_BYTE_CNT_H); //0x28
	writeb(block_count, sdmmc_base + SD_BLOCK_CNT_L); //0x2C
	writeb(block_count >> 8, sdmmc_base + SD_BLOCK_CNT_H); //0x30

	if (cmd_info->cmd->data->flags & MMC_DATA_READ) {
		writel((u32)sa, sdmmc_base + CR_SD_DMA_CTL1);
		writel(block_count, sdmmc_base + CR_SD_DMA_CTL2);

		if (data_len == RESP_LEN64)
			writel(RSP64_SEL | DDR_WR | DMA_XFER, sdmmc_base + CR_SD_DMA_CTL3);
		else if (data_len == RESP_LEN17)
			writel(RSP17_SEL | DDR_WR | DMA_XFER, sdmmc_base + CR_SD_DMA_CTL3);
		else
			writel(DDR_WR | DMA_XFER, sdmmc_base + CR_SD_DMA_CTL3);
		rtk_host->data_xfer_mode = CMD_RSP_DATA_READ;
		if (cmd_info->cmd->opcode == MMC_SEND_TUNING_BLOCK) {
			rtk_host->INT4_flag = false;
			rtk_host->data_xfer_mode = CMD_RSP_DATA_READ_TUNNING;
		}

	} else if (cmd_info->cmd->data->flags & MMC_DATA_WRITE) {
		pr_debug("rtk-sdmmc: %s: DMA sa = 0x%x\nDMA len = 0x%x\nDMA set = 0x%x\n", __func__, (u32)sa,
			 block_count, DMA_XFER);
		writel((u32)sa, sdmmc_base + CR_SD_DMA_CTL1);
		writel(block_count, sdmmc_base + CR_SD_DMA_CTL2);

		if (data_len == RESP_LEN64)
			writel(RSP64_SEL | DMA_XFER, sdmmc_base + CR_SD_DMA_CTL3);
		else if (data_len == RESP_LEN17)
			writel(RSP17_SEL | DMA_XFER, sdmmc_base + CR_SD_DMA_CTL3);
		else
			writel(DMA_XFER, sdmmc_base + CR_SD_DMA_CTL3);
		rtk_host->data_xfer_mode = CMD_RSP_DATA_WRITE;
		if (cmdcode == SD_AUTOWRITE3)
			rtk_host->data_xfer_mode = DATA_WRITE_ONLY;
	}

	rtk_host->cmd_opcode = cmd_idx;
	rtk_sdmmc_get_cmd_timeout(cmd_info);
	ret = rtk_sdmmc_int_wait(DRIVER_NAME, rtk_host, cmdcode);
	/* Reset dat64_sel and rsp17_sel, #CMD19 DMA won't be auto-cleared */
	if (cmd_info->cmd->opcode == MMC_SEND_TUNING_BLOCK) {
		writel(0x00000000, sdmmc_base + CR_SD_DMA_CTL3);
		rtk_host->data_xfer_mode = CMD_RSP_ONLY;
	}

	if (ret == CR_TRANS_OK) {
		if (cmdcode == SD_AUTOREAD1 || cmdcode == SD_AUTOWRITE1) {
			pr_debug("rtk-sdmmc: %s: auto read/write 1 skip response\n", __func__);
#ifdef CMD25_WO_STOP_COMMAND
		} else if (cmdcode == SD_AUTOWRITE2) {
			pr_debug("rtk-sdmmc: %s: auto write 2 skip response\n", __func__); //CMD + DATA
		} else if (cmdcode == SD_AUTOWRITE3) {
			pr_debug("rtk-sdmmc: %s: auto write 3 skip response\n", __func__); //DATA only, clear rsp
#endif
		} else {
			rtk_sdmmc_read_rsp(rtk_host, rsp, rsp_len);
			pr_debug("rtk-sdmmc: %s: stream cmd done\n", __func__);
		}
	}

	return ret;
}

static int rtk_sdmmc_stream(struct sdmmc_cmd_pkt *cmd_info)
{
	u8 cmd_idx = cmd_info->cmd->opcode;
	int ret = 0;
	u32 i = 0;
	u32 dir = 0;
	u32 dma_nents = 0;
	u32 dma_leng = 0;
	u32 dma_addr = 0;
	u32 old_arg = 0;
	u16 cmdcode = 0;
	u8 data_len = 0;

	struct scatterlist *sg;
	struct mmc_host *host = cmd_info->rtk_host->mmc;
	struct rtk_sdmmc_host *rtk_host = cmd_info->rtk_host;

#ifdef CMD25_WO_STOP_COMMAND
	cmd12_stop_cmd.arg = cmd_info->cmd->arg;   //send stop command only use arg info and we need to save it as global variable for fear that timer trigger stop command data corruption
	rtk_host->rtk_sdmmc_cmd12 = &cmd12_stop_cmd;
	//rtk_host->rtk_sdmmc_cmd12 = cmd_info->cmd;   //bad using because of local variable ,
#endif

	if (cmd_info->data->flags & MMC_DATA_READ)
		dir = DMA_FROM_DEVICE;
	else
		dir = DMA_TO_DEVICE;

	cmd_info->data->bytes_xfered = 0;
	dma_nents = dma_map_sg(mmc_dev(host), cmd_info->data->sg, cmd_info->data->sg_len, dir);
	sg = cmd_info->data->sg;

	old_arg = cmd_info->cmd->arg;

	for (i = 0 ; i < dma_nents ; i++, sg++) {
		u32 blk_cnt = 0;

		dma_leng = sg_dma_len(sg);
		dma_addr = sg_dma_address(sg);

		pr_debug("rtk-sdmmc: %s: dma_addr: 0x%x, dma_leng: 0x%x\n", __func__, dma_addr, dma_leng);

		if (cmd_idx == SD_SWITCH && (cmd_info->cmd->flags | MMC_CMD_ADTC)) {
			cmd_info->byte_count = 0x40;
			blk_cnt = dma_leng / 0x40;
			data_len = RESP_LEN64;
		} else if (cmd_idx == SD_APP_SD_STATUS &&
			   (cmd_info->cmd->flags & (0x3 << 5)) == MMC_CMD_ADTC) {
			cmd_info->byte_count = 0x40;
			blk_cnt = dma_leng / 0x40;
			data_len = RESP_LEN64;
		} else if (cmd_idx == MMC_SEND_TUNING_BLOCK || cmd_idx == SD_APP_SEND_SCR ||
			   cmd_idx == SD_APP_SEND_NUM_WR_BLKS) {
			cmd_info->byte_count = 0x40; //rtk HW limite, one trigger 512 byte pass.
			blk_cnt = 1;
			data_len = RESP_LEN64;
		} else if (cmd_idx == MMC_ALL_SEND_CID || cmd_idx == MMC_SEND_CSD ||
			   cmd_idx == MMC_SEND_CID) {
			cmd_info->byte_count = 0x11;
			blk_cnt = dma_leng / 0x11;
			data_len = RESP_LEN17;
		} else {
			cmd_info->byte_count = BYTE_CNT; //rtk HW limite, one trigger 512 byte pass.
			blk_cnt = dma_leng / BYTE_CNT;
			data_len = 0;
		}

		/*
		 * The engine transfers whole blocks, so a segment that is not
		 * a multiple of the block size cannot be described. Rounding
		 * up, as this used to, makes it write past the end of the
		 * segment. Refuse instead; the limits above are meant to stop
		 * such a segment ever reaching us.
		 */
		if (blk_cnt == 0 || blk_cnt * cmd_info->byte_count != dma_leng) {
			dev_err(mmc_dev(host),
				"segment %u of %u is %u bytes, not a multiple of %u\n",
				i, dma_nents, dma_leng, cmd_info->byte_count);
			ret = -EINVAL;
			break;
		}

		cmd_info->block_count = blk_cnt;
		cmd_info->dma_buffer = dma_addr;

		cmdcode = rtk_host->ops->chk_cmdcode(rtk_host, cmd_info->cmd);

#ifdef CMD25_WO_STOP_COMMAND //new write method
		if (host->card && mmc_card_blockaddr(host->card) && (
			cmd_info->cmd->opcode == MMC_WRITE_BLOCK ||
			cmd_info->cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK ||
			cmd_info->cmd->opcode == MMC_EXT_WRITE_MULTIPLE)) {
			if (rtk_host->sd_in_receive_data_state) {
				if (rtk_host->sd_current_blk_address == cmd_info->cmd->arg) {
					cmdcode = SD_AUTOWRITE3; //DATA only
				} else {
					rtk_sdmmc_send_stop_cmd(cmd_info->cmd, rtk_host, 0); //send stop command first
					rtk_host->pre_cmd = 12;
					rtk_host->sd_in_receive_data_state = 0; //not set SD in receive-data state
					cmdcode = SD_AUTOWRITE2; //CMD + DATA
				}
			} else {
				cmdcode = SD_AUTOWRITE2; //CMD + DATA
			}
			//alwasy use multi-write command
			if (cmd_info->cmd->opcode == MMC_EXT_WRITE_MULTIPLE)
				cmd_info->cmd->opcode = MMC_EXT_WRITE_MULTIPLE;
			else
				cmd_info->cmd->opcode = MMC_WRITE_MULTIPLE_BLOCK;
		} else {
			if (cmd_info->cmd->opcode == MMC_SEND_STATUS) { //cmd13

			} else {
				if (rtk_host->sd_in_receive_data_state) {
					rtk_sdmmc_send_stop_cmd(cmd_info->cmd, rtk_host, 0); //send stop command first
					rtk_host->pre_cmd = 12;
					/* stop trigger, not set SD in receive-data state */
					rtk_host->sd_in_receive_data_state = 0;
				}
			}
		}
#endif
		ret = rtk_sdmmc_stream_cmd(cmdcode, cmd_info, data_len);
		if (ret == 0) {
#ifdef CMD25_WO_STOP_COMMAND //new write method
			if (host->card && mmc_card_blockaddr(host->card) && (
				cmd_info->cmd->opcode == MMC_WRITE_BLOCK ||
				cmd_info->cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK ||
				cmd_info->cmd->opcode == MMC_EXT_WRITE_MULTIPLE)) {
				rtk_host->sd_current_blk_address = cmd_info->cmd->arg + blk_cnt;
				rtk_host->sd_in_receive_data_state = 1;
				mod_timer(&rtk_host->rtk_sdmmc_stop_cmd, jiffies + 20/*1*HZ*/);
			}
#endif
			/*
			 * A high-capacity card addresses blocks, not bytes, so
			 * the next segment starts blk_cnt further on, not
			 * dma_leng. This only started to matter when the host
			 * began accepting more than one segment per request:
			 * with max_segs = 1 the loop ran exactly once and the
			 * advance was never used.
			 */
			if (host->card && mmc_card_blockaddr(host->card))
				cmd_info->cmd->arg += blk_cnt;
			else
				cmd_info->cmd->arg += dma_leng;
			cmd_info->data->bytes_xfered += dma_leng;
		}

		if (ret) {
			cmd_info->cmd->arg = old_arg;
			break;
		}
	}

	dma_unmap_sg(mmc_dev(host), cmd_info->data->sg, cmd_info->data->sg_len, dir);
	rtk_host->g_cmd[0] = 0x00;

	return ret;
}

static int rtk_sdmmc_send_cmd_get_rsp(struct sdmmc_cmd_pkt *cmd_info)
{
	u8 cmd_idx = cmd_info->cmd->opcode;
	u32 sd_arg = 0;
	s8 rsp_para1 = 0;
	s8 rsp_para2 = 0;
	s8 rsp_para3 = 0;
	u8 rsp_len = cmd_info->rsp_len;
	u32 *rsp = (u32 *)&cmd_info->cmd->resp;
	u32 dma_val = 0;
	u32 byte_count = 0x200;
	u32 block_count = 1;
	u32 sa = 0;
	u32 buf_ptr = 0;
	int ret = 0;

	struct rtk_sdmmc_host *rtk_host = cmd_info->rtk_host;
	void __iomem *sdmmc_base = rtk_host->sdmmc;

	rtk_host->INT4_flag = false;

	rtk_sdmmc_set_rspparam(cmd_info);

	sd_arg = cmd_info->cmd->arg;
	rsp_para1 = cmd_info->rsp_para1;
	rsp_para2 = cmd_info->rsp_para2;
	rsp_para3 = cmd_info->rsp_para3;

	if (WARN_ON_ONCE(!rsp))
		return -EINVAL;
	if ((cmd_idx == SD_IO_SEND_OP_COND) | (cmd_idx == SD_IO_RW_DIRECT) |
	    (cmd_idx == SD_IO_RW_EXTENDED)) {
		pr_info("%s : reject SDIO commands cmd:0x%02x\n", __func__, cmd_idx);
		return CR_TRANSFER_FAIL;
	}

	if (rtk_host->mini_SD == 1 && cmd_idx == 8) {
		pr_info("%s : reject SDIO commands cmd:0x%02x\n", __func__, cmd_idx);
		return CR_TRANSFER_FAIL;
	}

	if (rsp_para1 != -1)
		writeb(rsp_para1, sdmmc_base + SD_CONFIGURE1);

	writeb(rsp_para2, sdmmc_base + SD_CONFIGURE2);

	if (rsp_para3 != -1)
		writeb(rsp_para3, sdmmc_base + SD_CONFIGURE3);

	rtk_host->g_cmd[0] = (0x40 | cmd_idx);
	rtk_host->g_cmd[1] = (sd_arg >> 24) & 0xff;
	rtk_host->g_cmd[2] = (sd_arg >> 16) & 0xff;
	rtk_host->g_cmd[3] = (sd_arg >> 8) & 0xff;
	rtk_host->g_cmd[4] = sd_arg & 0x000000FF;
	rtk_host->g_cmd[5] = 0x00000000;

	writeb(rtk_host->g_cmd[0], sdmmc_base + SD_CMD0);
	writeb(rtk_host->g_cmd[1], sdmmc_base + SD_CMD1);
	writeb(rtk_host->g_cmd[2], sdmmc_base + SD_CMD2);
	writeb(rtk_host->g_cmd[3], sdmmc_base + SD_CMD3);
	writeb(rtk_host->g_cmd[4], sdmmc_base + SD_CMD4);
	writeb(rtk_host->g_cmd[5], sdmmc_base + SD_CMD5);

	rtk_host->cmd_opcode = cmd_idx;

	rtk_sdmmc_get_cmd_timeout(cmd_info);
	rtk_host->data_xfer_mode = CMD_RSP_ONLY;
	if (RESP_TYPE_17B & rsp_para2) {
		rtk_host->INT4_flag = true;
		/*remap the resp dst buffer to un-cache*/
		buf_ptr = ((u32)rtk_host->dma_virt_addr & ~0xff);
		sa = buf_ptr / 8;
		dma_val = RSP17_SEL | DDR_WR | DMA_XFER;
		writeb(byte_count, sdmmc_base + SD_BYTE_CNT_L); //0x24
		writeb(byte_count >> 8, sdmmc_base + SD_BYTE_CNT_H); //0x28
		writeb(block_count, sdmmc_base + SD_BLOCK_CNT_L); //0x2C
		writeb(block_count >> 8, sdmmc_base + SD_BLOCK_CNT_H); //0x30

		writel(sa, sdmmc_base + CR_SD_DMA_CTL1); //espeical for R2
		writel(0x00000001, sdmmc_base + CR_SD_DMA_CTL2); //espeical for R2
		writel(dma_val, sdmmc_base + CR_SD_DMA_CTL3); //espeical for R2
		rtk_host->data_xfer_mode = CMD_RSP_DATA_READ;

	} else if (RESP_TYPE_6B & rsp_para2) {
		//do nothing
	}
	ret = rtk_sdmmc_int_wait(DRIVER_NAME, rtk_host, SD_SENDCMDGETRSP);
	if (ret == CR_TRANS_OK) {
		if (buf_ptr != 0) {
			*(((unsigned int *)buf_ptr) + 4) = readb(sdmmc_base + SD_CMD5);
			buf_ptr++;
			rtk_sdmmc_read_rsp(rtk_host, (u32 *)buf_ptr, rsp_len);
			memcpy(rsp, (u32 *)buf_ptr, 16);
		} else {
			rtk_sdmmc_read_rsp(rtk_host, rsp, rsp_len);
		}

		rtk_sdmmc_sync(rtk_host);

		if (cmd_idx == MMC_SET_RELATIVE_ADDR) {
			rtk_host->g_crinit = 1;
			pr_info("SD/MMC card init done.\n");
		}
	}

	memset(rtk_host->dma_virt_addr, 0x00, SD_ALLOC_LENGTH);

	return ret;
}

static void rtk_sdmmc_send_command(struct rtk_sdmmc_host *rtk_host, struct mmc_command *cmd)
{
	int ret = 0;

	struct sdmmc_cmd_pkt cmd_info;
	void __iomem *sdmmc_base = rtk_host->sdmmc;

	memset(&cmd_info, 0, sizeof(struct sdmmc_cmd_pkt));

	if (!rtk_host || !cmd) {
		pr_err("%s: rtk_host or cmd is null\n", __func__);
		return;
	}
	if (cmd->opcode == MMC_SEND_OP_COND)
		rtk_host->mmc_detected = 1;

	/*work around of bug switch voltage fail : force clock disable after cmd11*/
	if (cmd->opcode == SD_SWITCH_VOLTAGE)
		writeb(0x40, sdmmc_base + SD_BUS_STATUS);

	cmd_info.cmd = cmd;
	cmd_info.rtk_host = rtk_host;
	cmd_info.rsp_para2 = rtk_sdmmc_get_rsp_type(cmd_info.cmd);
	cmd_info.rsp_len = rtk_sdmmc_get_rsp_len(cmd_info.rsp_para2);

	if (cmd->data) {
		cmd_info.data = cmd->data;

		if (cmd->data->flags == MMC_DATA_READ) {
			/* do nothing */
		} else if (cmd->data->flags == MMC_DATA_WRITE) {
			if (rtk_host->wp == 1) {
				pr_err("%s: card is locked!", __func__);
				ret = -1;
				cmd->retries = 0;
				goto err_out;
			}
		} else {
			pr_err("%s: error: cmd->data->flags= %d\n", __func__, cmd->data->flags);
			cmd->error = -MMC_ERR_INVALID;
			cmd->retries = 0;
			goto err_out;
		}

		if (cmd->opcode != SD_APP_SEND_SCR) {
			ret = rtk_sdmmc_stream(&cmd_info);
		} else {
			struct scatterlist sg;
			struct mmc_data data_SCR = {0};
			void *ssr;

			ssr = kmalloc(64, GFP_KERNEL | GFP_DMA);
			if (!ssr) {
				ret = CR_TRANSFER_FAIL;
				goto err_out;
			}

			sg_init_one(&sg, ssr, 64);

			pr_debug("rtk-sdmmc: %s: data->blksz= %d, data->blocks= %d\n", __func__, cmd_info.data->blksz,
				 cmd_info.data->blocks);

			data_SCR.blksz = cmd_info.data->blksz;
			data_SCR.blocks = cmd_info.data->blocks;
			data_SCR.sg = cmd_info.data->sg;
			data_SCR.sg_len = cmd_info.data->sg_len;

			cmd_info.data->blksz = 64;
			cmd_info.data->blocks = 1;
			cmd_info.data->sg = &sg;
			cmd_info.data->sg_len = 1;

			ret = rtk_sdmmc_stream(&cmd_info);

			if (!ret) {
				sg_copy_from_buffer(data_SCR.sg, data_SCR.sg_len, ssr, data_SCR.blksz);
				cmd_info.data->bytes_xfered = data_SCR.blksz;
				pr_debug("rtk-sdmmc: %s: SCR =\n", __func__);
			}
			kfree(ssr);
		}
	} else {
		ret = rtk_sdmmc_send_cmd_get_rsp(&cmd_info);
	}

	if (cmd->opcode == MMC_SELECT_CARD) {
		rtk_host->rtflags |= RTKCR_FCARD_SELECTED;
		rtk_sdmmc_speed(rtk_host, SDMMC_CLOCK_6200KHZ);
	} else if ((cmd->opcode == SD_SEND_RELATIVE_ADDR) && ((cmd->flags & (0x3 << 5)) == MMC_CMD_BCR)) {
		rtk_host->sdmmc_rca = ((cmd->resp[0]) >> RCA_SHIFTER);
	} else if (cmd->opcode == MMC_ALL_SEND_CID) {
		/*
		 * CID[127:120] is the manufacturer ID; for an R2 response that
		 * is the top byte of resp[0]. Latching it here keeps the PLL
		 * selection in rtk_sdmmc_speed() self-contained instead of
		 * reaching into a global exported by the MMC core.
		 */
		rtk_host->card_manfid = cmd->resp[0] >> 24;
	}

err_out:
	if (ret) {
		if (ret == -RTK_RMOV)
			cmd->retries = 0;

		/* If SD card no response do SD host reset, jamestai20151008 */
		if (cmd->opcode == 49 || cmd->opcode == 59 || cmd->opcode == 48 || cmd->opcode == 58) {
			rtk_sdmmc_reset(rtk_host);
			cmd->error  = -110;
		} else {
			cmd->error = -MMC_ERR_FAILED;
		}
	}
	tasklet_schedule(&rtk_host->req_end_tasklet);
}

static void rtk_sdmmc_request(struct mmc_host *host, struct mmc_request *mrq)
{
	struct rtk_sdmmc_host *rtk_host = mmc_priv(host);
	struct mmc_command *cmd;
	unsigned char card_cmd = 0;

	if (WARN_ON_ONCE(rtk_host->mrq)) {
		mrq->cmd->error = -EBUSY;
		mmc_request_done(host, mrq);
		return;
	}
	down(&rtk_host->cr_sd_sem);
	cmd = mrq->cmd;
	rtk_host->mrq = mrq;
	rtk_host->data_xfer_mode = CMD_RSP_ONLY; // init each request
	/* Check SD card extension command support,jamestai20151008*/
	if (cmd->opcode == 58 || cmd->opcode == 59) {
		card_cmd = host->card->raw_scr[0] & 0x0F;
		if ((card_cmd & 0x08) != 0x08) {
			cmd->error = -110;
			goto done;
		}
	}

	if (cmd->opcode == 48 || cmd->opcode == 49) {
		card_cmd = host->card->raw_scr[0] & 0xF;
		if ((card_cmd & 0x8) != 0x8 && (card_cmd & 0x4) != 0x4) {
			cmd->error = -110;
			goto done;
		}
	}

	if (!(rtk_host->rtflags & RTKCR_FCARD_DETECTED)) {
		cmd->error = -MMC_ERR_RMOVE;
		cmd->retries = 0;
		goto done;
	}

	if (rtk_host && cmd) {
		rtk_sdmmc_allocate_dma_buf(rtk_host, cmd);
		rtk_sdmmc_send_command(rtk_host, cmd);
	} else {
done:
		tasklet_schedule(&rtk_host->req_end_tasklet);
	}

	rtk_host->pre_cmd = cmd->opcode;
	/* Unplug releases the semaphore; don't release it again here. */
	if (rtk_host->cr_sd_sem.count == 0)
		up(&rtk_host->cr_sd_sem);
}

static int rtk_sdmmc_get_ro(struct mmc_host *host)
{
	struct rtk_sdmmc_host *rtk_host = mmc_priv(host);
	void __iomem *sdmmc_base = rtk_host->sdmmc;
	u32 reginfo = 0;

	rtk_host->g_ro = reginfo = readb(sdmmc_base + CARD_EXIST);

	if (reginfo & SD_WRITE_PROTECT) {
		pr_info("%s: SD card is write protect, regCARD_EXIST = %x\n", __func__,
			readb(sdmmc_base + CARD_EXIST));
		return 1;
	}

	pr_debug("%s: SD card is not write protect, regCARD_EXIST = %x\n", __func__,
		 readb(sdmmc_base + CARD_EXIST));

	return 0;
}

static int rtk_sdmmc_get_cd(struct mmc_host *host)
{
	struct rtk_sdmmc_host *rtk_host = mmc_priv(host);
	void __iomem *sdmmc_base = rtk_host->sdmmc;
	u32 reginfo = 0;

	reginfo = readb(sdmmc_base + CARD_EXIST);

	if (reginfo & SD_EXISTENCE) {
		pr_debug("%s: SD card exists, regCARD_EXIST = %x\n", __func__, readb(sdmmc_base + CARD_EXIST));
		return 1;
	}

	pr_debug("%s: SD card does not exist, regCARD_EXIST = %x\n", __func__,
		 readb(sdmmc_base + CARD_EXIST));

	return 0;
}

static void rtk_sdmmc_set_ios(struct mmc_host *host, struct mmc_ios *ios)
{
	struct rtk_sdmmc_host *rtk_host = mmc_priv(host);

	if (ios->bus_mode == MMC_BUSMODE_PUSHPULL) {
		pr_debug("rtk-sdmmc: %s: ios busmode = pushpull\n", __func__);
		if (ios->bus_width == MMC_BUS_WIDTH_8) {
			pr_debug("rtk-sdmmc: %s: set bus width 8\n", __func__);
			rtk_sdmmc_set_bits(rtk_host, BUS_WIDTH_8);
		} else if (ios->bus_width == MMC_BUS_WIDTH_4) {
			pr_debug("rtk-sdmmc: %s: set bus width 4\n", __func__);
			rtk_sdmmc_set_bits(rtk_host, BUS_WIDTH_4);
		} else {
			rtk_sdmmc_set_bits(rtk_host, BUS_WIDTH_1);
			pr_debug("rtk-sdmmc: %s: set bus width 1\n", __func__);
		}

		if (ios->clock >= UHS_SDR104_MAX_DTR && ios->timing == MMC_TIMING_UHS_SDR104) {
			rtk_sdmmc_set_access_mode(rtk_host, ACCESS_MODE_SD30);
			rtk_sdmmc_speed(rtk_host, SDMMC_CLOCK_208000KHZ);
			pr_debug("rtk-sdmmc: %s: UHS SDR104 SDMMC_CLOCK_208000KHZ\n", __func__);
		} else if (ios->clock >= UHS_SDR50_MAX_DTR && ios->timing == MMC_TIMING_UHS_SDR50) {
			rtk_sdmmc_set_access_mode(rtk_host, ACCESS_MODE_SD30);
			rtk_sdmmc_speed(rtk_host, SDMMC_CLOCK_100000KHZ);
			pr_debug("rtk-sdmmc: %s: UHS SDR50 SDMMC_CLOCK_100000KHZ\n", __func__);
		} else if (ios->clock >= UHS_SDR25_MAX_DTR && ios->timing == MMC_TIMING_UHS_SDR25) {
			rtk_sdmmc_set_access_mode(rtk_host, ACCESS_MODE_SD30);
			rtk_sdmmc_speed(rtk_host, SDMMC_CLOCK_50000KHZ);
			pr_debug("rtk-sdmmc: %s: UHS SDR25 SDMMC_CLOCK_50000KHZ\n", __func__);
		} else if (ios->clock >= UHS_SDR12_MAX_DTR && ios->timing == MMC_TIMING_UHS_SDR12) {
			rtk_sdmmc_set_access_mode(rtk_host, ACCESS_MODE_SD30);
			rtk_sdmmc_speed(rtk_host, SDMMC_CLOCK_25000KHZ);
			pr_debug("rtk-sdmmc: %s: UHS SDR12 SDMMC_CLOCK_25000KHZ\n", __func__);
		} else if (ios->clock >= UHS_DDR50_MAX_DTR && ios->timing == MMC_TIMING_UHS_DDR50) {
			rtk_sdmmc_set_access_mode(rtk_host, ACCESS_MODE_DDR);
			rtk_sdmmc_speed(rtk_host, SDMMC_CLOCK_50000KHZ);
			pr_debug("rtk-sdmmc: %s: UHS DDR50 SDMMC_CLOCK_50000KHZ\n", __func__);
		} else if (ios->clock >= HIGH_SPEED_MAX_DTR && ios->timing == MMC_TIMING_SD_HS) {
			rtk_sdmmc_set_access_mode(rtk_host, ACCESS_MODE_SD20);
			rtk_sdmmc_speed(rtk_host, SDMMC_CLOCK_50000KHZ);
			pr_debug("rtk-sdmmc: %s: High Speed SDMMC_CLOCK_50000KHZ\n", __func__);
		} else if (ios->clock >= 25000000) {
			rtk_sdmmc_set_access_mode(rtk_host, ACCESS_MODE_SD20);
			rtk_sdmmc_speed(rtk_host, SDMMC_CLOCK_25000KHZ);
		} else {
			if (rtk_host->rtflags & RTKCR_FCARD_SELECTED) {
				rtk_sdmmc_speed(rtk_host, SDMMC_CLOCK_6200KHZ);
				pr_debug("rtk-sdmmc: %s: Mid speed RTKCR_FCARD_SELECTED = 1 SDMMC_CLOCK_6200KHZ\n", __func__);
			} else {
				rtk_sdmmc_speed(rtk_host, SDMMC_CLOCK_400KHZ);
				pr_debug("rtk-sdmmc: %s: Low speed SDMMC_CLOCK_400KHZ\n", __func__);
			}
		}
	} else {  //MMC_BUSMODE_OPENDRAIN
		pr_debug("rtk-sdmmc: %s: ios busmode != pushpull low speed SDMMC_CLOCK_400KHZ\n", __func__);
		rtk_sdmmc_speed(rtk_host, SDMMC_CLOCK_400KHZ);
		rtk_sdmmc_set_bits(rtk_host, BUS_WIDTH_1);
	}

	if (ios->power_mode == MMC_POWER_UP) {
		rtk_host->ops->card_power(rtk_host, 1); //power on
		pr_debug("rtk-sdmmc: %s: Power on\n", __func__);
	} else if (ios->power_mode == MMC_POWER_OFF) {
		rtk_host->ops->card_power(rtk_host, 0); //power off
		pr_debug("rtk-sdmmc: %s: Power off\n", __func__);
	}
}

static void rtk_sdmmc_hw_initial(struct rtk_sdmmc_host *rtk_host)
{
	void __iomem *sdmmc_base = rtk_host->sdmmc;
	void __iomem *pll_base = rtk_host->pll;
	int use_alt_sample_point = 0;

	/*SD PLL Initialization, jamestai20150721*/
	writel(readl(pll_base + CR_PLL_SD4) | 0x00000004, pll_base + CR_PLL_SD4);
	writel(readl(pll_base + CR_PLL_SD4) | 0x00000007, pll_base + CR_PLL_SD4);

	writel(0x003E0003, pll_base + CR_PLL_SD1);
	mdelay(10);

	udelay(100);
	/* 0x1EC (not 0x1AC) changes the SD card PLL, not the SDIO PLL. */
	writel(0x00000006, pll_base + 0x01EC);
	writel(readl(sdmmc_base + CR_SD_CKGEN_CTL) | 0x00070000,
	       sdmmc_base + CR_SD_CKGEN_CTL); //Switch SD source clock to 4MHz by Hsin-yin
	mdelay(2);
	writel(0x00000007, pll_base + 0x01EC);	//JIM modified, 1AC->1EC
	udelay(200);

	udelay(100);
	writel(0x00000006, pll_base + 0x01EC);	 //JIM modified, 1AC->1EC
	writel(0x04515893, pll_base + CR_PLL_SD2); //change from 4517893 to 4515893 for passing the EMI
	writel(0x00564388, pll_base + CR_PLL_SD3); //Set PLL clock rate, default clock 100MHz
	mdelay(2);
	writel(0x00000007, pll_base + 0x01EC);	  //JIM modified, 1AC->1EC
	udelay(200);
	writeb(0x3, sdmmc_base + SD_BUS_TA_STATE);
#ifndef CONFIG_RTD119X
	writel(readl(sdmmc_base + 0x20) & (~0x01), sdmmc_base + 0x20);
	writel(readl(sdmmc_base + 0x20) | 0x00000001, sdmmc_base + 0x20);	//reset the DMA
#endif
	udelay(100);
	writel(0x00000006, pll_base + 0x01EC);	   //JIM modified, 1AC->1EC
	writel(readl(sdmmc_base + CR_SD_CKGEN_CTL) & 0xFFF8FFFF,
	       sdmmc_base + CR_SD_CKGEN_CTL); //Switch SD source clock to normal clock source by Hsin-yin
	udelay(100);
	writel(0x00000007, pll_base + 0x01EC);		//JIM modified, 1AC->1EC
	udelay(200);
	writeb(readb(sdmmc_base + SD_CONFIGURE1) & 0x000000EF,
	       sdmmc_base + SD_CONFIGURE1); //Reset FIFO pointer by Hsin-yin
	writel(0x00000007, pll_base + CR_PLL_SD4); //PLL_SD4
	udelay(100);
	writeb(0xD0, sdmmc_base + SD_CONFIGURE1);

	rtk_sdmmc_speed(rtk_host, SDMMC_CLOCK_400KHZ);

	writeb(0X0, sdmmc_base + CARD_SD_CLK_PAD_DRIVE);
	writeb(0X0, sdmmc_base + CARD_SD_CMD_PAD_DRIVE);
	writeb(0X0, sdmmc_base + CARD_SD_DAT_PAD_DRIVE);

	writel(0x00000000, sdmmc_base + CR_SD_PAD_CTL); //change to 3.3v

	writel(0x00000016, sdmmc_base + CR_SD_ISR); //enable interrupt
	writel(0x00000016, sdmmc_base + CR_SD_ISREN); // disable all
	writel(0x00000007, sdmmc_base + CR_SD_ISREN);
	rtk_sdmmc_sync(rtk_host);

	writeb(0x02, sdmmc_base + CARD_SELECT); //for emmc, select SD ip
	if (use_alt_sample_point) {
		writeb(0x08, sdmmc_base + SD_SAMPLE_POINT_CTL); //sample point = SDCLK / 4
		writeb(0x10, sdmmc_base + SD_PUSH_POINT_CTL); //output ahead SDCLK /4
	} else {
		writeb(0x00, sdmmc_base + SD_SAMPLE_POINT_CTL); //sample point = SDCLK / 4
		writeb(0x00, sdmmc_base + SD_PUSH_POINT_CTL); //output ahead SDCLK /4
	}
}

static void rtk_sdmmc_hw_reset(struct mmc_host *host)
{
	struct rtk_sdmmc_host *rtk_host = mmc_priv(host);
	void __iomem *sdmmc_base = rtk_host->sdmmc;

	rtk_host->sd_in_receive_data_state = 0; //CMD25_WO_STOP_COMMAND
	rtk_host->sd_current_blk_address = 0; //CMD25_WO_STOP_COMMAND
	rtk_host->rtk_sdmmc_cmd12 = NULL; //CMD25_WO_STOP_COMMAND
	rtk_host->rtflags &= ~RTKCR_FCARD_SELECTED;
	rtk_host->sdmmc_rca = 0;

	rtk_sdmmc_hw_initial(rtk_host);
	writeb(0xFF, sdmmc_base + CR_CARD_STOP); //SD Card module transfer stop and idle state
	writeb(0x00, sdmmc_base + CR_CARD_STOP); //SD Card module transfer no stop
	/* Select the SD/MMC card module for the coming data transfer (bits 2:0 = 010). */
	writeb(0x02,
	       sdmmc_base + CARD_SELECT);
	writeb(0x04, sdmmc_base + CR_CARD_OE); //arget module is SD/MMC card module, bit 2 =1
	writeb(0x04, sdmmc_base + CARD_CLOCK_EN_CTL); //SD Card Module Clock Enable, bit 2 = 1
	writeb(0xD0, sdmmc_base + SD_CONFIGURE1);
	writeb(0x00, sdmmc_base + SD_STATUS2); //SD CMD Response Timeout Error disable	//#x /b 0x18010584

	rtk_host->ops->card_power(rtk_host, 0); //power off
	rtk_host->ops->card_power(rtk_host, 1); //power on

	rtk_host->mmc_detected = 0;

	rtk_sdmmc_sync(rtk_host);
}

/*
 * The card signals busy by holding DAT[3:0] low; SD_BUS_STATUS reports the
 * current line level. Without this callback the core cannot confirm a 1.8V
 * signal-voltage switch and warns "cannot verify signal voltage switch".
 */
static int rtk_sdmmc_card_busy(struct mmc_host *mmc)
{
	struct rtk_sdmmc_host *rtk_host = mmc_priv(mmc);
	u8 status = readb(rtk_host->sdmmc + SD_BUS_STATUS);

	return !(status & SD_DAT3_0_LEVEL);
}

static int rtk_sdmmc_switch_voltage(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct rtk_sdmmc_host *rtk_host = mmc_priv(mmc);
	void __iomem *sdmmc_base = rtk_host->sdmmc;
	void __iomem *pll_base = rtk_host->pll;
	int ret = 0;

	if (ios->signal_voltage != MMC_SIGNAL_VOLTAGE_330) {
		ret = rtk_sdmmc_wait_voltage_stable_low(rtk_host); //check DAT[3:0] are at LOW level

		if (ret < 0)
			goto out;

		writeb(0x3B, sdmmc_base + CARD_CLOCK_EN_CTL); //host stop clk
		mdelay(10); //delay 5 ms to fit SD spec

		/* Workaround : keep IO pad voltage at 3.3v for SD card compatibility, jamestai20141225 */
		writel(0x00000000, sdmmc_base + CR_SD_PAD_CTL);

		rtk_sdmmc_sync(rtk_host);
		writel(0x00e0003, pll_base + CR_PLL_SD1);
		mdelay(10);
		writeb(0x3F, sdmmc_base + CARD_CLOCK_EN_CTL); //force output clk
		writeb(0x80, sdmmc_base + SD_BUS_STATUS); //force a short period 1.8v clk
		rtk_sdmmc_sync(rtk_host);

		ret = rtk_sdmmc_wait_voltage_stable_high(rtk_host); //check DAT[3:0] are at HIGH level
		if (ret < 0)
			goto out;
	}

	writeb(0x00, sdmmc_base + SD_BUS_STATUS); //stop 1.8v clk

out:
	return ret;
}

static int rtk_sdmmc_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct rtk_sdmmc_host *rtk_host = mmc_priv(mmc);
	void __iomem *sdmmc_base = rtk_host->sdmmc;
	void __iomem *pll_base = rtk_host->pll;
	int ret = 0;
	unsigned int reg_tmp = 0;
	unsigned int reg_tmp2 = 0;
	unsigned int reg_tuned3318 = 0;

	reg_tmp2 = readl(pll_base + CR_PLL_SD2); //disable spectrum
	writel((reg_tmp2 & 0xFFFF1FFF), pll_base + CR_PLL_SD2); //PLL_SD2 clear [15:13]

	/*if tune tx phase fail, down 8MHz and retry*/
	do {
		ret = rtk_sdmmc_tuning_tx(rtk_host);
		if (ret == -MMC_ERR_RMOVE) {
			pr_err("%s: Tuning TX fail.\n", __func__);
			return ret;
		} else if (ret) {
			reg_tmp = readl(pll_base + CR_PLL_SD3);
			reg_tuned3318 = (reg_tmp & 0x03FF0000) >> 16;
			if (reg_tuned3318 <= 100) {
				pr_err("%s: Tuning TX fail\n", __func__);
				return ret;
			}
			writel(readl(sdmmc_base + CR_SD_CKGEN_CTL) | 0x00070000,
			       sdmmc_base + CR_SD_CKGEN_CTL); //Switch SD source clock to 4MHz by Hsin-yin
			reg_tmp = ((reg_tmp & (~0x3FF0000)) | ((reg_tuned3318 - 8) << 16)); //down 8MHz
			writel(reg_tmp, pll_base + CR_PLL_SD3);
			mdelay(2);
			writel(readl(sdmmc_base + CR_SD_CKGEN_CTL) & 0xFFF8FFFF,
			       sdmmc_base + CR_SD_CKGEN_CTL); //Switch SD source clock to normal clock source by Hsin-yin
			writeb(readb(sdmmc_base + SD_CONFIGURE1) & 0xEF,
			       sdmmc_base + SD_CONFIGURE1); //Reset FIFO pointer by Hsin-yin
		}
	} while (ret);

	if (ret) {
		pr_err("%s: Tuning TX  fail\n", __func__);
		return ret;
	}

	ret = rtk_sdmmc_tuning_rx(rtk_host);
	writel(reg_tmp2, pll_base + CR_PLL_SD2); //enable spectrum

	if (ret) {
		pr_err("%s: Tuning RX fail\n", __func__);
		return ret;
	}

	pr_debug("%s CR_PLL_SD3: %d (0x%02x)\n", __func__, (readl(pll_base + CR_PLL_SD3) >> 16),
		 (readl(pll_base + CR_PLL_SD3) >> 16));
	pr_debug("%s CLK_GEN DVI: %d\n", __func__, 1 << (readl(sdmmc_base + CR_SD_CKGEN_CTL) & 0x03));

	return ret;
}

static const struct mmc_host_ops rtk_sdmmc_ops = {
	.request = rtk_sdmmc_request,
	.get_ro = rtk_sdmmc_get_ro,
	.get_cd = rtk_sdmmc_get_cd,
	.set_ios = rtk_sdmmc_set_ios,
	.card_hw_reset = rtk_sdmmc_hw_reset,
	.start_signal_voltage_switch = rtk_sdmmc_switch_voltage,
	.card_busy = rtk_sdmmc_card_busy,
	.execute_tuning = rtk_sdmmc_execute_tuning,
};

#ifdef CMD25_WO_STOP_COMMAND
static void rtk_sdmmc_cmd12_fun(struct timer_list *t)
{
	struct rtk_sdmmc_host *rtk_host = from_timer(rtk_host, t, rtk_sdmmc_stop_cmd);

	if (rtk_host->sd_in_receive_data_state && rtk_host->rtk_sdmmc_cmd12 && rtk_host) {
		if (down_trylock(&rtk_host->cr_sd_sem) != 0) {  // @rtk_sdmmc_cmd12_fun, lock fail
			mod_timer(&rtk_host->rtk_sdmmc_stop_cmd, jiffies + 20);
		} else {
			rtk_host->sd_in_receive_data_state = 0;

			rtk_host->do_stop_command_wo_complete = 1;
			rtk_host->wait_stop_command_irq_done = 1;
			rtk_sdmmc_send_stop_cmd(rtk_host->rtk_sdmmc_cmd12, rtk_host, 1);
			rtk_sdmmc_wait_stop_cmd_irq_done(rtk_host);

			rtk_host->pre_cmd = 12;
			/* Unplug releases the semaphore; don't release it again here. */
			if (rtk_host->cr_sd_sem.count == 0)
				up(&rtk_host->cr_sd_sem);
		}
	}
}
#endif

static void rtk_sdmmc_plug(struct timer_list *t)
{
	u32 reginfo = 0;
	u32 det_time = 0;
	unsigned long timeout = 0;
	struct rtk_sdmmc_host *rtk_host = from_timer(rtk_host, t, plug_timer);
	void __iomem *sdmmc_base = rtk_host->sdmmc;

	reginfo = readb(sdmmc_base + CARD_EXIST);
	if ((reginfo & SD_EXISTENCE) ^ (rtk_host->int_status_old & SD_EXISTENCE)) {
		rtk_host->rtflags &= ~RTKCR_FCARD_DETECTED;
		rtk_host->wp = 0;

		if (reginfo & SD_EXISTENCE) {
			rtk_host->ops->card_power(rtk_host, 1); //power on, Jim add

			rtk_host->ins_event = EVENT_INSER;
			rtk_host->rtflags |= RTKCR_FCARD_DETECTED;
			det_time = 1;
			rtk_host->CMD8_SD1 = false;
			rtk_host->CMD55_MMC = false;

			writel(0x00000000, sdmmc_base + CR_SD_DMA_CTL3); //stop dma control
			rtk_host->data_xfer_mode = CMD_RSP_ONLY;
			writeb(0x00, sdmmc_base + CR_CARD_STOP); //SD Card module transfer no stop
			/* Select the SD/MMC card module for the coming data transfer (bits 2:0 = 010). */
			writeb(0x02,
			       sdmmc_base + CARD_SELECT);
			writeb(0x04, sdmmc_base + CR_CARD_OE); //arget module is SD/MMC card module, bit 2 =1
			writeb(0x04, sdmmc_base + CARD_CLOCK_EN_CTL); //SD Card Module Clock Enable, bit 2 = 1
			writeb(0xD0, sdmmc_base + SD_CONFIGURE1);
			rtk_sdmmc_speed(rtk_host, SDMMC_CLOCK_400KHZ);
			writeb(0x00, sdmmc_base + SD_STATUS2);
			pr_debug("SD card is being inserted now...!!!\n");
		} else {
			rtk_host->ins_event = EVENT_REMOV;
			rtk_host->rtflags &= ~RTKCR_FCARD_DETECTED;
			det_time = 1;
			rtk_host->driving_capacity = 0;

			/*
			 * Only release if actually held, else this would
			 * over-release a held lock (see rtk_sdmmc_request()).
			 */
			if (rtk_host->cr_sd_sem.count == 0)
				up(&rtk_host->cr_sd_sem);
			writeb(0X0, sdmmc_base + CARD_SD_CLK_PAD_DRIVE);
			writeb(0X0, sdmmc_base + CARD_SD_CMD_PAD_DRIVE);
			writeb(0X0, sdmmc_base + CARD_SD_DAT_PAD_DRIVE);
			rtk_host->mini_SD = 0;
			rtk_host->broken_flag = false;

			/* This DMA reset disturbs the eMMC engine; let it go idle first. */
			timeout = jiffies + msecs_to_jiffies(100);

			while (time_before(jiffies, timeout)) {
				if (!(readl(rtk_host->emmc + EMMC_DMA_CTL3) & 0x01))
					break;
			}
			writel(0x0, sdmmc_base + CR_SD_DMA_CTL3); //stop dma control
			rtk_host->data_xfer_mode = CMD_RSP_ONLY;
			writeb(0xff, sdmmc_base + CR_CARD_STOP); //SD Card module transfer stop and idle state

			writeb(0x3b, sdmmc_base + CARD_CLOCK_EN_CTL);

			rtk_host->ops->card_power(rtk_host, 0); //power off, Jim add
			pr_err("SD card is being removed now...!!!\n");
		}

		rtk_sdmmc_sync(rtk_host);
		pr_info("[SD] SD card power register=%x\n", readl(rtk_host->pll + CR_PFUNC_CR));
		rtk_host->rtflags &= ~RTKCR_FCARD_SELECTED;
		rtk_host->sdmmc_rca = 0;
		mmc_detect_change(rtk_host->mmc, msecs_to_jiffies(det_time));
	}

	rtk_host->int_status_old = reginfo;
	mod_timer(&rtk_host->plug_timer, jiffies + HZ / 10);
}

static void rtk_sdmmc_req_end_tasklet(unsigned long param)
{
	struct rtk_sdmmc_host *rtk_host = (struct rtk_sdmmc_host *)param;
	struct mmc_request *mrq;

	mrq = rtk_host->mrq;
	rtk_host->mrq = NULL;

	mmc_request_done(rtk_host->mmc, mrq);
}

static void rtk_sdmmc_timeout(struct timer_list *t)
{
	struct rtk_sdmmc_host *rtk_host = from_timer(rtk_host, t, timer);
	u32 int_status = readl(rtk_host->sdmmc + CR_SD_ISR);
	u32 int_status_en = readl(rtk_host->sdmmc + CR_SD_ISREN);

	if (rtk_host->poll == 1)
		return;	//this timeout function is for interrupt mode

	if (rtk_host->ins_event == EVENT_REMOV) {
		pr_err("command timeout because the SD card is removed\n");
		complete(rtk_host->int_waiting);
		writel(0x00000016, rtk_host->sdmmc + CR_SD_ISR);
		return;
	}

	if (rtk_host->broken_flag) {
		complete(rtk_host->int_waiting);
		writel(0x00000016, rtk_host->sdmmc + CR_SD_ISR);
		return;
	}

	pr_err("%s: command/data timeout, rtk_host->pre_cmd=%d, cmd=%d, poll=%d, ISR=%x, ISR_EN=%x\n",
	       __func__, rtk_host->pre_cmd, rtk_host->cmd_opcode, rtk_host->poll, int_status, int_status_en);

	if (rtk_host->cmd_opcode == 8 && int_status == 0) {
		rtk_host->CMD8_SD1 = true;	//SD 1.x
		pr_err("Reject SD 2.x command\n");
	} else if (rtk_host->cmd_opcode == 55 && rtk_host->CMD8_SD1 &&
		   (int_status == 0x0 || int_status == 0x6)) {
		rtk_host->CMD55_MMC = true;   //MMC card
		pr_err("Reject SD 1.x command\n");
	}

	complete(rtk_host->int_waiting);
	writel(0x00000016, rtk_host->sdmmc + CR_SD_ISR);
	rtk_sdmmc_sync(rtk_host);

	if (rtk_host->pre_cmd == 0 && rtk_host->cmd_opcode == 1 &&
	    (int_status == 0x0 || int_status == 0x16)) {
		if (rtk_host->mini_SD == 0) {
			pr_err("This might be mini SD card\n");
			rtk_host->mini_SD = 1;
			rtk_host->compatible_flag = true;
			remove_sdcard(rtk_host);
		} else {
			pr_err("SD/MMC card is broken!\n");
			rtk_host->broken_flag = true;
			card_broken(rtk_host);
			return;
		}
	} else if (!rtk_host->CMD55_MMC && rtk_host->cmd_opcode != 8 &&
		   (int_status == 0x0 || int_status == 0x16)) {
		/*
		 * This is a workaround: some SD cards fail to be recognized
		 * and read commands return errors. Tuning the driving or PLL
		 * can fix this, but this reset is here as a fallback.
		 */
		rtk_host->compatible_flag = true;
		pr_err("SD compatible issue: reset the card and adjust the driving capacity!\n");
		rtk_host->driving_capacity++;
		remove_sdcard(rtk_host);
	}
}

static void rtk_sdmmc_card_power(struct rtk_sdmmc_host *rtk_host, u8 status)
{
	int res = 0;
	u32 power_status = rtk_host->power_status;
	u32 tmp;

	if (status == power_status)
		return;

	if (status) {
		res = gpio_direction_output(rtk_host->sdmmc_gpio, 0);
		tmp = readl(rtk_host->pll + CR_PFUNC_CR);
		writel(0x33333223 | (tmp & 0x00000F00), rtk_host->pll + CR_PFUNC_CR);   //Jim modified
		if (res < 0) {
			pr_err("%s: can't gpio output = %d (power on)\n", __func__, rtk_host->sdmmc_gpio);
		} else {
			power_status = 1; //card is power on
			mdelay(10); //delay 10 ms after power on
		}
	} else {
		res = gpio_direction_input(rtk_host->sdmmc_gpio);
		tmp = readl(rtk_host->pll + CR_PFUNC_CR);
		writel(0x22223222 | (tmp & 0x00000F00), rtk_host->pll + CR_PFUNC_CR);   //Jim modified
		if (res < 0) {
			pr_err("%s: can't gpio input = %d (power off)\n", __func__, rtk_host->sdmmc_gpio);
		} else {
			power_status = 0; //card is power off
			mdelay(10); //delay 10 ms after power off
		}
	}
	rtk_host->power_status = power_status;
}

static u8 rtk_sdmmc_chk_cmdcode(struct rtk_sdmmc_host *rtk_host, struct mmc_command *cmd)
{
	u8 cmdcode = 0;

	if (cmd->opcode < 59) {
		if (cmd->opcode == 8 && rtk_host->mmc_detected == 1)
			cmdcode = SD_NORMALREAD;
		else
			cmdcode = rtk_sdmmc_cmdcode[cmd->opcode][0];

		WARN_ON(!cmd->data);

		if (cmd->data->flags & MMC_DATA_WRITE) {
			if (cmd->opcode == 42)
				cmdcode = SD_NORMALWRITE;
			else if (cmd->opcode == 56)
				cmdcode = SD_AUTOWRITE2;
		}
	} else {
		cmdcode = SD_CMD_UNKNOW;
	}

	return cmdcode;
}

static irqreturn_t rtk_sdmmc_irq(int irq, void *data)
{
	struct rtk_sdmmc_host *rtk_host = (struct rtk_sdmmc_host *)data;
	void __iomem *sdmmc_base = rtk_host->sdmmc;
	u32 int_status = 0;

	rtk_sdmmc_sync(rtk_host);
	int_status = readl(sdmmc_base + CR_SD_ISR);
	if ((int_status & ISRSTA_INT2) && rtk_host->cmd_opcode != 13 && rtk_host->cmd_opcode != 19) {
		rtk_host->irq_error_bit = 1;
		goto clear_irq;
	}
	if (rtk_host->INT4_flag) {
		if (rtk_host->data_xfer_mode == CMD_RSP_DATA_READ || rtk_host->data_xfer_mode == DATA_ONLY) {
			if (!(int_status & ISRSTA_INT4))
				return IRQ_HANDLED;
		} else {
			if (!(int_status & ISRSTA_INT1))
				return IRQ_HANDLED;
		}
	} else {
		if (!(int_status & ISRSTA_INT1))
			return IRQ_HANDLED;
	}
clear_irq:
	if (rtk_host->poll == 0) {
		timer_delete(&rtk_host->timer);
		complete(rtk_host->int_waiting);
	}

	writel(0x00000016, sdmmc_base + CR_SD_ISR);
	rtk_sdmmc_sync(rtk_host);

	if (rtk_host->wait_stop_command_irq_done)
		rtk_host->wait_stop_command_irq_done = 0; // irq finish
	return IRQ_HANDLED;
}

static struct rtk_sdmmc_host_ops sdmmc_ops = {
	.card_power = rtk_sdmmc_card_power,
	.chk_cmdcode = rtk_sdmmc_chk_cmdcode,
};

static const struct of_device_id rtk_sdmmc_match[] = {
	{ .compatible = "realtek,rtd1195-sdmmc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rtk_sdmmc_match);

static void rtk_sdmmc_show_version(void)
{
	pr_info("%s: build at : %s\n", DRIVER_NAME, utsname()->version);

	pr_info("%s: CONFIG_MMC_BLOCK_BOUNCE disable\n", DRIVER_NAME);
}

static void __iomem *rtk_sdmmc_iomap(struct device_node *np, const char *name)
{
	int index = of_property_match_string(np, "reg-names", name);

	if (index < 0)
		return NULL;

	return of_iomap(np, index);
}

static int rtk_sdmmc_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc = NULL;
	struct rtk_sdmmc_host *rtk_host = NULL;
	int ret = 0;
	int irq = 0;
	struct device_node *sdmmc_node = pdev->dev.of_node;

	/*
	 * irq_of_parse_and_map() and mmc_alloc_host()/rtk_host setup are
	 * pulled ahead of the clk/reset acquisition and the do_stop_command_
	 * wo_complete/wait_stop_command_irq_done/cr_sd_sem init below (all of
	 * which used to run before rtk_host existed, when they were plain
	 * module-scope globals). Now that they're rtk_host fields, rtk_host
	 * has to be allocated first -- it used to be a NULL-pointer write
	 * here once these became rtk_host-> stores. No hardware enable calls
	 * moved: reset_control_deassert()/clk_prepare_enable() are still
	 * issued at their original point below, unchanged.
	 */
	irq = irq_of_parse_and_map(sdmmc_node, 0);
	if (!irq) {
		pr_err("%s: fail to parse of irq.\n", __func__);
		return -ENXIO;
	}

	mmc = mmc_alloc_host(sizeof(struct rtk_sdmmc_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto out;
	}

	rtk_global_host = rtk_host = mmc_priv(mmc);
	memset(rtk_host, 0, sizeof(struct rtk_sdmmc_host));

	/*
	 * Matches the old file-scope global's initializer ("int magic_num =
	 * 0xff;"); overwritten below from hardware on CONFIG_ARCH_REALTEK
	 * before any code reads it, so this default is not load-bearing on
	 * this board -- kept for parity with the original semantics in case
	 * that ever changes, and for platforms that don't overwrite it.
	 */
	rtk_host->magic_num = 0xff;

	/* Matches the old file-scope global's initializer ("int irq_error_bit=1;"). */
	rtk_host->irq_error_bit = 1;

	rtk_host->clk_cr = devm_clk_get(&pdev->dev, "clk_en_cr");
	if (IS_ERR(rtk_host->clk_cr)) {
		pr_warn("%s: clk_get() returns %ld\n", __func__,
			PTR_ERR(rtk_host->clk_cr));
		rtk_host->clk_cr = NULL;
	}
	rtk_host->clk_sd_ip = devm_clk_get(&pdev->dev, "clk_en_sd_ip");
	if (IS_ERR(rtk_host->clk_sd_ip)) {
		pr_warn("%s: clk_get() returns %ld\n", __func__,
			PTR_ERR(rtk_host->clk_sd_ip));
		rtk_host->clk_sd_ip = NULL;
	}
	rtk_host->rstc_cr = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(rtk_host->rstc_cr)) {
		pr_warn("%s: reset_control_get() returns %ld\n", __func__,
			PTR_ERR(rtk_host->rstc_cr));
		rtk_host->rstc_cr = NULL;
	}

	rtk_sdmmc_show_version();

	rtk_host->do_stop_command_wo_complete = 0; // probe stage
	rtk_host->wait_stop_command_irq_done = 0; // probe stage

	sema_init(&rtk_host->cr_sd_sem, 1);//probe stage

	/*
	 * Take the windows by name. Taking them by index tied the driver to
	 * the order the entries happen to be written in, and forced reg[0] to
	 * be the CRT block, so the controller came up as "18000000.sdmmc",
	 * named after a block it does not own.
	 */
	rtk_host->sdmmc = rtk_sdmmc_iomap(sdmmc_node, "sdmmc");
	rtk_host->pll = rtk_sdmmc_iomap(sdmmc_node, "crt");
	rtk_host->sysbrdg = rtk_sdmmc_iomap(sdmmc_node, "bs2");
	rtk_host->emmc = rtk_sdmmc_iomap(sdmmc_node, "emmc");
	if (!rtk_host->sdmmc || !rtk_host->pll || !rtk_host->sysbrdg ||
	    !rtk_host->emmc) {
		ret = -ENOMEM;
		goto out;
	}
	rtk_host->int_waiting = &rtk_sdmmc_wait;

	rtk_host->sdmmc_gpio = of_get_named_gpio(sdmmc_node, "gpios", 0);
	if (gpio_is_valid(rtk_host->sdmmc_gpio)) {
		ret = gpio_request(rtk_host->sdmmc_gpio, "sdmmc_gpio");
		if (ret < 0)
			pr_err("%s: can't request gpio %d\n", __func__, rtk_host->sdmmc_gpio);
	} else {
		pr_err("%s: gpio %d is not valid\n", __func__, rtk_host->sdmmc_gpio);
	}
	rtk_host->magic_num = readl(rtk_host->sysbrdg + 0x204) >> 16;

	reset_control_deassert(rtk_host->rstc_cr);
	clk_prepare_enable(rtk_host->clk_cr);
	clk_prepare_enable(rtk_host->clk_sd_ip);

	rtk_host->mmc = mmc;
	rtk_host->dev = &pdev->dev;
	rtk_host->ops = &sdmmc_ops;

	rtk_sdmmc_hw_initial(rtk_host);

	spin_lock_init(&rtk_host->lock);

	mmc->ocr_avail = MMC_VDD_30_31 |
				MMC_VDD_31_32 |
				MMC_VDD_32_33 |
				MMC_VDD_33_34;

	/*
	 * No UHS modes. Every one of them signals at 1.8V, and this board
	 * does not: rtk_sdmmc_switch_voltage() goes through the CMD11
	 * handshake and then deliberately leaves the pads at 3.3V, so
	 * advertising UHS made the card switch to 1.8V while the host kept
	 * driving 3.3V, and told the core "signal voltage: 1.80V" when
	 * debugfs was asked.
	 */
	mmc->caps = MMC_CAP_4_BIT_DATA |
				MMC_CAP_SD_HIGHSPEED |
				MMC_CAP_MMC_HIGHSPEED |
				MMC_CAP_HW_RESET;

	mmc->caps2 = MMC_CAP2_NO_SDIO | MMC_CAP2_NO_MMC;
	mmc->f_min = 10000000 >> 8; //RTK min bus clk is 10Mhz/256
	mmc->f_max = 208000000; //RTK max bus clk is 208Mhz
	/*
	 * rtk_sdmmc_stream_cmd() already walks the scatterlist and programs
	 * one DMA transfer per segment, so the host is not limited to a single
	 * segment. Advertising 1 forced the block layer to split every request,
	 * and the core then busy-polls the card once per fragment.
	 */
	mmc->max_segs = 128;
	mmc->max_blk_size = 512;
	mmc->max_blk_count = 0x1000;
	mmc->max_seg_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;

	tasklet_init(&rtk_host->req_end_tasklet, rtk_sdmmc_req_end_tasklet, (unsigned long)rtk_host);

	ret = request_irq(irq, rtk_sdmmc_irq, IRQF_SHARED, DRIVER_NAME, rtk_host);
	if (ret) {
		pr_err("%s: cannot assign irq %d\n", __func__, irq);
		goto out;
	} else {
		rtk_host->irq = irq;
	}
	timer_setup(&rtk_host->plug_timer, rtk_sdmmc_plug, 0);
	timer_setup(&rtk_host->timer, rtk_sdmmc_timeout, 0);

#ifdef CMD25_WO_STOP_COMMAND
	timer_setup(&rtk_host->rtk_sdmmc_stop_cmd, rtk_sdmmc_cmd12_fun, 0);
#endif
	mmc->ops = &rtk_sdmmc_ops;
	rtk_host->rtflags &= ~(RTKCR_FCARD_DETECTED | RTKCR_FCARD_SELECTED);
	rtk_host->int_status_old &= ~SD_EXISTENCE;
	rtk_host->sdmmc_rca = 0;
	rtk_host->g_crinit = 0;

	platform_set_drvdata(pdev, mmc);

	rtk_sdmmc_sync(rtk_host);

	ret = mmc_add_host(mmc);
	if (ret)
		goto out;
	mod_timer(&rtk_host->plug_timer, jiffies + 3 * HZ);
	return 0;

out:
	if (mmc)
		mmc_free_host(mmc);
	return ret;
}

static void rtk_sdmmc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);

	if (mmc) {
		struct rtk_sdmmc_host *rtk_host = mmc_priv(mmc);

		rtk_sdmmc_free_dma_buf(rtk_host);

		mmc_remove_host(mmc);

		free_irq(rtk_host->irq, rtk_host);

		timer_delete_sync(&rtk_host->timer);
		timer_delete_sync(&rtk_host->plug_timer);
#ifdef CMD25_WO_STOP_COMMAND
		timer_delete_sync(&rtk_host->rtk_sdmmc_stop_cmd);
#endif
		if (rtk_host->sdmmc)
			iounmap(rtk_host->sdmmc);
		if (rtk_host->pll)
			iounmap(rtk_host->pll);
		if (rtk_host->sysbrdg)
			iounmap(rtk_host->sysbrdg);
		if (rtk_host->emmc)
			iounmap(rtk_host->emmc);

		/*
		 * rtk_host is mmc_priv(mmc), so it lives inside the mmc host
		 * allocation. Everything that reads it has to happen before
		 * mmc_free_host().
		 */
		if (gpio_is_valid(rtk_host->sdmmc_gpio))
			gpio_free(rtk_host->sdmmc_gpio);

		mmc_free_host(mmc);
	}

	platform_set_drvdata(pdev, NULL);
}

static struct platform_driver rtk_sdmmc_driver = {
	.probe = rtk_sdmmc_probe,
	.remove = rtk_sdmmc_remove,
	.driver = {
		.name = DRIVER_NAME,
#ifdef CONFIG_PM
		.pm = &rtk_sdmmc_pm_ops,
#endif
		.of_match_table = rtk_sdmmc_match,
	},
};

module_platform_driver(rtk_sdmmc_driver);

MODULE_DESCRIPTION("Realtek SD/MMC host driver");
MODULE_AUTHOR("James Tai");
MODULE_LICENSE("GPL");
