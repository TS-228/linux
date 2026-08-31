// SPDX-License-Identifier: GPL-2.0-only
/* Realtek RTD119x eMMC/SD/SDIO driver -- Copyright (C) 2008-2009 Realtek Ltd. */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/mbus.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/mmc/host.h>
#include <linux/unaligned.h>

#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/semaphore.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include "reg_mmc_rtd119x.h"
#include "reg_sys_rtd119x.h"

#define soc_is_rtk1195()	(get_rtd_chip_id() == CHIP_ID_RTD1195)
#define realtek_rev()		get_rtd_chip_revision()

/* CR (card-reader/eMMC) controller IRQ, formerly mach/irqs.h's IRQ_CR. */
#define RTKEMMC_IRQ_CR		74

#include "../core/card.h"
#include "../core/core.h"
#include "rtkemmc_rtd119x.h"
#include "rtk-crsd-ops.h"
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/reset.h>

#include <asm/system_info.h>

#define DRIVER_NAME "rtkemmc"
#define BANNER      "Realtek eMMC Driver"
#define VERSION     "rtkemmc.c Phoenix 2014-03-26 10:00"

#define EXT_CSD_ENH_START_ADDR          136     /* R/W, 4 bytes */
#define EXT_CSD_MAX_ENH_SIZE_MULT       157     /* R/W, 3 bytes */
#define EXT_CSD_WR_REL_SET              167     /* R/W ifHS_CTRL_REL=1 */
#define EXT_CSD_ENH_SIZE_MULT           140     /* R/W, 3 bytes */
#define MMC_DATA_TUNING BIT(11)
#define MMC_MICRON_60            60
#define mmc_card_cmd24_err(c)  ((c)->state & MMC_STATE_CMD24_ERR)
#define MMC_STATE_CMD24_ERR    BIT(13)         /* for cmd24 error handling */
#define mmc_card_set_cmd24_err(c)       ((c)->state |= MMC_STATE_CMD24_ERR)
/*
 * Known limitation carried over from the vendor driver: if the system and
 * card both enter suspend, resume does not wake the card correctly. The
 * vendor's unfinished "REAL_SUSPEND" rework was never enabled and has been
 * dropped; this has not been re-tested since the resume-path NULL derefs
 * were fixed.
 */
void __iomem *rtkemmc_rbus_base;
static int maxfreq = RTKSD_CLOCKRATE_MAX;
static int nodma;
static u32 rtk_emmc_bus_wid;

static void rtksd_request(struct mmc_host *host, struct mmc_request *mrq);
static int rtksd_get_ro(struct mmc_host *mmc);
static void rtksd_set_ios(struct mmc_host *host, struct mmc_ios *ios);

static void set_cmd_info(struct mmc_card *card, struct mmc_command *cmd,
			 struct sd_cmd_pkt *cmd_info, u32 opcode, u32 arg, u8 rsp_para);

static int rtksd_stop_transmission(struct mmc_card *card, int ignore);
static int rtksd_send_status(struct mmc_card *card, u16 *state, u8 divider, int ignore);
static int rtksd_wait_status(struct mmc_card *card, u8 state, u8 divider, int ignore);

static void rtkcr_set_speed(struct rtkemmc_host *sdport, u8 level);
static int mmc_tuning_ddr50(struct rtkemmc_host *sdport);
static int mmc_tuning_hs200(struct rtkemmc_host *sdport);
static int rtksd_execute_tuning(struct mmc_host *host, u32 opcode);
static int mmc_select_sdr50_push_sample(struct rtkemmc_host *sdport);
static u32 rtkemmc_backup_registers(struct rtkemmc_host *sdport);

typedef void (*set_gpio_func_t)(u32 gpio_num, u8 dir, u8 level);

static struct resource rtkemmc_resources[] = {
	[0] = {
	.start  = EM_BASE_ADDR,
	.end    = EM_BASE_ADDR + 0x200,
	.flags  = IORESOURCE_MEM,
	},
	[1] = {
	.start  = RTKEMMC_IRQ_CR,
	.end    = RTKEMMC_IRQ_CR,
	.flags  = IORESOURCE_IRQ,
	},
};

static const struct mmc_host_ops rtkemmc_ops = {
	.request        = rtksd_request,
	.get_ro         = rtksd_get_ro,
	.set_ios        = rtksd_set_ios,
	.execute_tuning = rtksd_execute_tuning
};

#define UNSTUFF_BITS(resp, start, size)					\
	({								\
		const int __size = size;				\
		const u32 __mask = (__size < 32 ? 1 << __size : 0) - 1;	\
		const int __off = 3 - ((start) / 32);			\
		const int __shft = (start) & 31;			\
		u32 __res;						\
									\
		__res = resp[__off] >> __shft;				\
		if (__size + __shft > 32)				\
			__res |= resp[__off - 1] << ((32 - __shft) % 32);	\
		__res & __mask;						\
	})
DECLARE_COMPLETION(rtk_emmc_wait);

static void sync(void)
{
	/*
	 * rtk_crsd_sync() adds its own +0x20 offset internally, so this
	 * passes the BS2-bank *base* (0x1801a000, also reached as
	 * rtk_host->sysbrdg from rtk-sdmmc.c) -- together they land on the
	 * same shared sync/flush register (physical 0x1801a020) this used
	 * to reach directly. Delegating also drops the raw cr_writel()
	 * pointer store's explicit DMB pair in favor of the standard
	 * writel() accessor's own implicit barrier, which supersedes it.
	 */
	rtk_crsd_sync(RTKEMMC_REG(0x1801a000));
}

#define TIMEOUT_MS 3000
int rtkcr_wait_opt_end(char *drv_name, struct rtkemmc_host *sdport, u8 cmdcode, u8 cmd_idx,
		       u8 cpu_mode)
{
	u8 sd_transfer_reg;
	int dma_val = 0;
	int err = CR_TRANS_OK;
	unsigned long timeend = 0;
	unsigned int dma_to = 0;

	switch (cmdcode) {
	case 0x2:
	case 0x7:
	case 0x8:
		dma_to = TIMEOUT_MS;
		break;
	default:
		dma_to = 0;
		break;
	}
	switch (cmd_idx) {
	case MMC_READ_SINGLE_BLOCK:
	case MMC_READ_MULTIPLE_BLOCK:
	case MMC_WRITE_BLOCK:
	case MMC_WRITE_MULTIPLE_BLOCK:
		dma_to = TIMEOUT_MS;
		break;
	}
	    err = rtk_int_enable_and_waitfor(sdport, cmdcode, cmd_idx, TIMEOUT_MS, dma_to);
	if (err != 0)
		return err;
	    sync();
	    err = CR_DMA_FAIL;
	    timeend = jiffies + msecs_to_jiffies(TIMEOUT_MS);
	while (time_before(jiffies, timeend)) {
		sync();
		if (!(cr_readl(sdport->base_io + EMMC_DMA_CTL3) & DMA_XFER)) {
			err = 0;
			break;
		}
	}
	if (err) {
		pr_info("\n%s - trans dma fail (cmd/2193/status1/status2/bus_status/cfg1/cfg2/cfg3/dma) :\n"
			"\t0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%02x 0x%02x 0x%02x 0x%08x\n",
			drv_name, sdport->last_cmd[0], cr_readb(sdport->base_io + SD_TRANSFER),
			cr_readb(sdport->base_io + SD_STATUS1), cr_readb(sdport->base_io + SD_STATUS2),
			cr_readb(sdport->base_io + SD_BUS_STATUS), cr_readb(sdport->base_io + SD_CONFIGURE1),
			cr_readb(sdport->base_io + SD_CONFIGURE2), cr_readb(sdport->base_io + SD_CONFIGURE3),
			cr_readl(sdport->base_io + EMMC_DMA_CTL3));
		return err;
	}
	if (sdport->last_cmd[0] == 21) {
		cr_writel(DAT64_SEL | DDR_WR, sdport->base_io + EMMC_DMA_CTL3);
		sync();
		err = CR_TRANSFER_TO;
		timeend = jiffies + msecs_to_jiffies(TIMEOUT_MS);
		while (time_before(jiffies, timeend)) {
			sd_transfer_reg = cr_readb(sdport->base_io + SD_TRANSFER);
			sync();
			if ((sd_transfer_reg & (END_STATE | IDLE_STATE)) == (END_STATE | IDLE_STATE)) {
				err = 0;
				break;
			}
		}
		if (err)
			return err;
		sync();
		err = CR_DMA_FAIL;
		timeend = jiffies + msecs_to_jiffies(TIMEOUT_MS);
		while (time_before(jiffies, timeend)) {
			dma_val = cr_readl(sdport->base_io + EMMC_DMA_CTL3);
			sync();
			if ((dma_val & DMA_XFER) != DMA_XFER)
				return 0;
		}
		if (err)
			return err;
	}
	return err;
}

int rtk_int_enable_and_waitfor(struct rtkemmc_host *sdport, u8 cmdcode, u8 cmd_idx,
			       unsigned long msec, unsigned long dma_msec)
{
	unsigned long timeend = 0;

	sdport->int_status  = 0;
	sdport->sd_trans    = -1;
	sdport->sd_status1   = 0;
	sdport->sd_status2   = 0;
	sdport->bus_status   = 0;
	sdport->dma_trans   = 0;

	sdport->int_waiting = &rtk_emmc_wait;
	/* timeout timer fire */
	if (&sdport->timer) {
		timeend = msecs_to_jiffies(msec) + sdport->tmout;

		mod_timer(&sdport->timer, (jiffies + timeend));
	}

	//wait for ^M

	if (cmdcode == 25 && (cr_readl(sdport->base_io + EMMC_DMA_CTL3) & 0x2))
		pr_debug("cmd=25 but read direct ..........->\n");

	rtk_int_waitfor(sdport, cmdcode, cmd_idx, msec, dma_msec);
	/* Ensure sdport's status fields are visible before we read them below. */
	smp_wmb();
	sync();

	if (sdport->sd_trans & ERR_STATUS) { //transfer error
		if (sdport->tuning && cmd_idx == MMC_WRITE_MULTIPLE_BLOCK &&
		    (cr_readb(sdport->base_io + SD_STATUS1) == CRC7_STATUS ||
		     cr_readb(sdport->base_io + SD_STATUS1) == (CRC7_STATUS | 0x8))) {
			pr_err("\nignore cmd25 crc7 error\n");
			return CR_TRANSFER_IGN;
		}
		if (!sdport->tuning) {
			pr_debug(" trans err : register settings(base=0x%08x,0x%08x,cmd=%02x)\n", sdport->base_io,
				 sdport->base_io + SD_CMD0, cmd_idx);
			pr_debug(" cmd0:0x%02x cmd1:0x%02x cmd2:0x%02x cmd3:0x%02x cmd4:0x%02x cmd5:0x%02x\n",
				 cr_readb(sdport->base_io + SD_CMD0), cr_readb(sdport->base_io + SD_CMD1),
				cr_readb(sdport->base_io + SD_CMD2), cr_readb(sdport->base_io + SD_CMD3),
				cr_readb(sdport->base_io + SD_CMD4), cr_readb(sdport->base_io + SD_CMD5));
			pr_debug(" trans:0x%02x status1:0x%02x status2:0x%02x bus_status:0x%02x\n",
				 cr_readb(sdport->base_io + SD_TRANSFER), cr_readb(sdport->base_io + SD_STATUS1),
				cr_readb(sdport->base_io + SD_STATUS2), cr_readb(sdport->base_io + SD_BUS_STATUS));
			pr_debug(" configure1:0x%02x configure2:0x%02x configure3:0x%02x\n",
				 cr_readb(sdport->base_io + SD_CONFIGURE1), cr_readb(sdport->base_io + SD_CONFIGURE2),
				cr_readb(sdport->base_io + SD_CONFIGURE3));
			pr_debug(" byteH:0x%02x byteL:0x%02x blkH:0x%02x blkL:0x%02x\n",
				 cr_readb(sdport->base_io + SD_BYTE_CNT_H), cr_readb(sdport->base_io + SD_BYTE_CNT_L),
				cr_readb(sdport->base_io + SD_BLOCK_CNT_H), cr_readb(sdport->base_io + SD_BLOCK_CNT_L));
			pr_debug(" CPU_ACC:0x%08x dma_ctl1:0x%08x dma_ctl2:0x%08x dma_ctl3:0x%08x\n",
				 cr_readl(sdport->base_io + EMMC_CPU_ACC), cr_readl(sdport->base_io + CR_DMA_CTL1),
				cr_readl(sdport->base_io + CR_DMA_CTL2), cr_readl(sdport->base_io + EMMC_DMA_CTL3));
		}
		return CR_TRANSFER_FAIL;
	}
	/* transfer error */
	if ((sdport->sd_trans & (END_STATE | IDLE_STATE)) != (END_STATE | IDLE_STATE) &&
	    sdport->tuning) {
		if (!sdport->tuning) {
			pr_debug(" trans to : register settings(base=0x%08x,0x%08x,cmd=0x%02x)\n", sdport->base_io,
				 sdport->base_io + SD_CMD0, cmd_idx);
			pr_debug(" cmd0:0x%02x cmd1:0x%02x cmd2:0x%02x cmd3:0x%02x cmd4:0x%02x cmd5:0x%02x\n",
				 cr_readb(sdport->base_io + SD_CMD0), cr_readb(sdport->base_io + SD_CMD1),
				cr_readb(sdport->base_io + SD_CMD2), cr_readb(sdport->base_io + SD_CMD3),
				cr_readb(sdport->base_io + SD_CMD4), cr_readb(sdport->base_io + SD_CMD5));
			pr_debug(" trans:0x%02x status1:0x%02x status2:0x%02x bus_status:0x%02x\n",
				 cr_readb(sdport->base_io + SD_TRANSFER), cr_readb(sdport->base_io + SD_STATUS1),
				cr_readb(sdport->base_io + SD_STATUS2), cr_readb(sdport->base_io + SD_BUS_STATUS));
			pr_debug(" configure1:0x%02x configure2:0x%02x configure3:0x%02x\n",
				 cr_readb(sdport->base_io + SD_CONFIGURE1), cr_readb(sdport->base_io + SD_CONFIGURE2),
				cr_readb(sdport->base_io + SD_CONFIGURE3));
			pr_debug(" byteH:0x%02x byteL:0x%02x blkH:0x%02x blkL:0x%02x\n",
				 cr_readb(sdport->base_io + SD_BYTE_CNT_H), cr_readb(sdport->base_io + SD_BYTE_CNT_L),
				cr_readb(sdport->base_io + SD_BLOCK_CNT_H), cr_readb(sdport->base_io + SD_BLOCK_CNT_L));
			pr_debug(" CPU_ACC:0x%08x dma_ctl1:0x%08x dma_ctl2:0x%08x dma_ctl3:0x%08x\n",
				 cr_readl(sdport->base_io + EMMC_CPU_ACC), cr_readl(sdport->base_io + CR_DMA_CTL1),
				cr_readl(sdport->base_io + CR_DMA_CTL2), cr_readl(sdport->base_io + EMMC_DMA_CTL3));
		}
		return CR_TRANSFER_FAIL;
	}
	return 0;
}

void rtk_int_waitfor(struct rtkemmc_host *sdport, u8 cmdcode, u8 cmd_idx, unsigned long msec,
		     unsigned long dma_msec)
{
	sync();
	if (sdport->rtflags & RTKCR_FOPEN_LOG) {
		pr_debug(" rtkemmc : register settings(base=0x%08x,0x%02x)\n", sdport->base_io,
			 cr_readb(sdport->base_io + SD_CMD0));
		pr_debug(" cmd0:0x%02x cmd1:0x%02x cmd2:0x%02x cmd3:0x%02x cmd4:0x%02x cmd5:0x%02x\n",
			 cr_readb(sdport->base_io + SD_CMD0), cr_readb(sdport->base_io + SD_CMD1),
			cr_readb(sdport->base_io + SD_CMD2), cr_readb(sdport->base_io + SD_CMD3),
			cr_readb(sdport->base_io + SD_CMD4), cr_readb(sdport->base_io + SD_CMD5));
		pr_debug(" trans:0x%02x status1:0x%02x status2:0x%02x bus_status:0x%02x\n",
			 cr_readb(sdport->base_io + SD_TRANSFER), cr_readb(sdport->base_io + SD_STATUS1),
			cr_readb(sdport->base_io + SD_STATUS2), cr_readb(sdport->base_io + SD_BUS_STATUS));
		pr_debug(" configure1:0x%02x configure2:0x%02x configure3:0x%02x\n",
			 cr_readb(sdport->base_io + SD_CONFIGURE1), cr_readb(sdport->base_io + SD_CONFIGURE2),
			cr_readb(sdport->base_io + SD_CONFIGURE3));
		pr_debug(" byteH:0x%02x byteL:0x%02x blkH:0x%02x blkL:0x%02x\n",
			 cr_readb(sdport->base_io + SD_BYTE_CNT_H), cr_readb(sdport->base_io + SD_BYTE_CNT_L),
			cr_readb(sdport->base_io + SD_BLOCK_CNT_H), cr_readb(sdport->base_io + SD_BLOCK_CNT_L));
		pr_debug(" CPU_ACC:0x%08x dma_ctl1:0x%08x dma_ctl2:0x%08x dma_ctl3:0x%08x sample_ctl:0x%08x pull_ctl:0x%08x\n",
			 cr_readl(sdport->base_io + EMMC_CPU_ACC), cr_readl(sdport->base_io + CR_DMA_CTL1),
			cr_readl(sdport->base_io + CR_DMA_CTL2), cr_readl(sdport->base_io + EMMC_DMA_CTL3),
			cr_readb(sdport->base_io + SD_SAMPLE_POINT_CTL), cr_readb(sdport->base_io + SD_PUSH_POINT_CTL));
		pr_debug(" card_pad_drv:0x%08x cmd_pad_drv:0x%08x data_pad_drv:0x%08x EMMC_CKGEN_CTL:0x%08x, SYS_PLL_EMMC3=0x%08x\n",
			 cr_readb(sdport->base_io + EMMC_CARD_PAD_DRV), cr_readb(sdport->base_io + EMMC_CMD_PAD_DRV),
			cr_readb(sdport->base_io + EMMC_DATA_PAD_DRV), cr_readl(sdport->base_io + EMMC_CKGEN_CTL),
			cr_readl(SYS_PLL_EMMC3));
	}

	if (sdport->int_waiting) {
		rtkcr_hold_int_dec(sdport->base_io);
		rtkcr_clr_int_sta(sdport->base_io);
		rtkcr_en_int(sdport->base_io);
		rtkcr_clr_int_sta(sdport->base_io);

		sync();
		/*
		 * Bounded (300ms) interrupt-driven wait, matching rtk-sdmmc.c's
		 * design (stress-tested on hardware: 6400+ consecutive commands,
		 * zero timeouts) -- this used to be an unbounded
		 * wait_for_completion(), relying solely on the mod_timer()
		 * backstop armed by rtk_int_enable_and_waitfor() to ever
		 * recover a command whose interrupt never arrives.
		 */
		rtk_crsd_fire_and_wait(RTKEMMC_REG(sdport->base_io), sdport->int_waiting, cmdcode);
		sync();
		sync();

		if (sdport->sd_trans == -1) {
			/*
			 * The 300ms bound above elapsed without rtksd_irq() firing
			 * (it would have moved sdport->sd_trans off the -1 sentinel
			 * rtk_int_enable_and_waitfor() set before arming this wait,
			 * via rtk_op_complete()). Disarm the backstop timer now --
			 * before it can later fire against a *different*, unrelated
			 * command's int_waiting/sd_trans -- then fall back to the
			 * same bounded register-poll rtk-sdmmc.c uses, and snapshot
			 * status into sdport exactly like rtksd_irq()/
			 * rtksd_timeout_timer() would have, so
			 * rtk_int_enable_and_waitfor()'s existing checks below keep
			 * working unchanged.
			 */
			unsigned long flags;

			timer_delete_sync(&sdport->timer);
			rtk_crsd_poll_for_end(RTKEMMC_REG(sdport->base_io), RTKEMMC_REG(0x1801a000),
					      jiffies, msecs_to_jiffies(msec));

			spin_lock_irqsave(&sdport->lock, flags);
			rtkcr_get_int_sta(sdport->base_io, &sdport->int_status);
			rtkcr_get_sd_trans(sdport->base_io, &sdport->sd_trans);
			rtkcr_get_dma_trans(sdport->base_io, &sdport->dma_trans);
			rtkcr_get_sd_sta(sdport->base_io, &sdport->sd_status1, &sdport->sd_status2, &sdport->bus_status);
			spin_unlock_irqrestore(&sdport->lock, flags);
		}
	}
}

void rtk_op_complete(struct rtkemmc_host *sdport)
{
	if (sdport->int_waiting) {
		struct completion *waiting = sdport->int_waiting;

		complete(waiting);
		sync();
	}
}

char *rtkcr_parse_token(const char *parsed_string, const char *token)
{
	const char *ptr = parsed_string;
	const char *start, *end, *value_start, *value_end;
	char *ret_str;

	while (1) {
		value_start = value_end = 0;
		for (;  *ptr == ' ' || *ptr == '\t'; ptr++)
			;
		if (*ptr == '\0')
			break;
		start = ptr;
		for (;  *ptr != ' ' && *ptr != '\t' && *ptr != '=' && *ptr != '\0'; ptr++)
			;
		end = ptr;
		if (*ptr == '=') {
			ptr++;
			if (*ptr == '"') {
				ptr++;
				value_start = ptr;
				for (; *ptr != '"' && *ptr != '\0'; ptr++)
					;
				if (*ptr != '"' || (*(ptr + 1) != '\0' && *(ptr + 1) != ' ' && *(ptr + 1) != '\t')) {
					pr_err("system_parameters error! Check your parameters     .");
					break;
				}
			} else {
				value_start = ptr;
				for (;  *ptr != ' ' && *ptr != '\t' && *ptr != '\0' && *ptr != '"'; ptr++)
					;
				if (*ptr == '"') {
					pr_err("system_parameters error! Check your parameters.");
					break;
				}
			}
			value_end = ptr;
		}

		if (!strncmp(token, start, end - start)) {
			if (value_start) {
				ret_str = kmalloc(value_end - value_start + 1, GFP_KERNEL);
				if (ret_str)
					strscpy(ret_str, value_start, value_end - value_start + 1);
				return ret_str;
			}
			ret_str = kmalloc(1, GFP_KERNEL);
			if (ret_str)
				ret_str[0] = '\0';
			return ret_str;
		}
	}

	return (char *)NULL;
}

void rtkcr_chk_param(u32 *pparam, u32 len, u8 *ptr)
{
	u32 value, i;

	*pparam = 0;
	for (i = 0; i < len; i++) {
		value = ptr[i] - '0';
		if (value >= 0 && value <= 9) {
			*pparam += value << (4 * (len - 1 - i));
			continue;
		}

		value = ptr[i] - 'a';
		if (value >= 0 && value <= 5) {
			value += 10;
			    *pparam += value << (4 * (len - 1 - i));
			continue;
		}

		value = ptr[i] - 'A';
		if (value >= 0 && value <= 5) {
			value += 10;
			    *pparam += value << (4 * (len - 1 - i));
			continue;
		}
	}
}

u32 verA_magic_num;
int rtkcr_chk_ver_a(void)
{
	return verA_magic_num; //0: polling, 1: interrupt
}

void emmc_show_config123(struct rtkemmc_host *sdport)
{
	u32 reginfo = 0;	/* stays 0 for an unrecognised base -- must not print stack garbage */
	u32 iobase = sdport->base_io;

	if (iobase == EM_BASE_ADDR)
		reginfo = cr_readl(iobase + EMMC_CKGEN_CTL);
	else if (iobase == CR_BASE_ADDR)
		reginfo = cr_readl(iobase + CR_SD_CKGEN_CTL);

	pr_debug("CFG1=0x%x CFG2=0x%x CFG3=0x%x bus clock CKGEN=%08x\n",
		 cr_readb(iobase + SD_CONFIGURE1),
	cr_readb(iobase + SD_CONFIGURE2),
	cr_readb(iobase + SD_CONFIGURE3),
	reginfo);
}

static void rtkemmc_set_crt_muxpad(struct rtkemmc_host *sdport)
{
	u32 reg_val = 0;

	//set default i/f to cr
	reg_val = cr_readl(SYS_muxpad0);
	reg_val &= ~0xFFFF0FFF;
	reg_val |= 0xaaaa0aa8;
	cr_writel(reg_val, SYS_muxpad0);

	if (soc_is_rtk1195() && (realtek_rev() >= RTD_CHIP_A01)) {
		cr_writel(0xe0003, PLL_EMMC1); //LDO1.8v
		cr_writel(0x0, CR_PAD_CTL); //PAD to 1.8v
	}
}

static u32 swap_endian(u32 input)
{
	u32 output;

	output = (input & 0xff000000) >> 24 |
			 (input & 0x00ff0000) >> 8 |
			 (input & 0x0000ff00) << 8 |
			 (input & 0x000000ff) << 24;
	return output;
}

static void rtksd_read_rsp(struct rtkemmc_host *sdport, u32 *rsp, int reg_count)
{
	u32 iobase = sdport->base_io;

	if (reg_count == 6) {
		rsp[0] = rsp[1] = 0;
		rsp[0] = cr_readb(iobase + SD_CMD1) << 24 |
			 cr_readb(iobase + SD_CMD2) << 16 |
			 cr_readb(iobase + SD_CMD3) << 8 |
			 cr_readb(iobase + SD_CMD4);

	} else if (reg_count == 16) {
		rsp[0] = swap_endian(rsp[0]);
		rsp[1] = swap_endian(rsp[1]);
		rsp[2] = swap_endian(rsp[2]);
		rsp[3] = swap_endian(rsp[3]);
	}
}

/*******************************************************
 *  *
 *   *******************************************************/
static void rtkcr_set_mode_selection(struct rtkemmc_host *sdport, unsigned int set_bit)
{
	u32 iobase = sdport->base_io;
	unsigned int tmp_bits;
	unsigned long flags;

	spin_lock_irqsave(&sdport->lock, flags);
	tmp_bits = cr_readb(iobase + SD_CONFIGURE1) & ~MASK_MODE_SELECT;
	cr_writeb((unsigned char)(tmp_bits | set_bit), iobase + SD_CONFIGURE1);
	sync();
	spin_unlock_irqrestore(&sdport->lock, flags);
}

static int rtkcr_get_mode_selection(struct rtkemmc_host *sdport)
{
	u32 iobase = sdport->base_io;
	int tmp_bits;
	unsigned long flags;

	spin_lock_irqsave(&sdport->lock, flags);
	tmp_bits = cr_readb(iobase + SD_CONFIGURE1) & MASK_MODE_SELECT;
	sync();
	spin_unlock_irqrestore(&sdport->lock, flags);
	return tmp_bits;
}

static void rtkcr_set_ldo(struct rtkemmc_host *sdport, u32 set_ldo)
{
	u32 tmp_val = 0;
	unsigned long flags;

	spin_lock_irqsave(&sdport->lock, flags);
	tmp_val = (cr_readl(SYS_PLL_EMMC3) & 0xffff) | (set_ldo << 16);
	cr_writel(tmp_val, SYS_PLL_EMMC3);
	sync();
	spin_unlock_irqrestore(&sdport->lock, flags);
}

static void rtkcr_set_pad_driving(struct rtkemmc_host *sdport, unsigned int mode,
				  unsigned char CLOCK_DRIVING, unsigned char CMD_DRIVING, unsigned char DATA_DRIVING)
{
	static unsigned char card_pad_val = 0, cmd_pad_val = 0, data_pad_val;

	switch (mode) {
	case MMC_IOS_GET_PAD_DRV:
		card_pad_val = cr_readb(sdport->base_io + EMMC_CARD_PAD_DRV);
		cmd_pad_val = cr_readb(sdport->base_io + EMMC_CMD_PAD_DRV);
		data_pad_val = cr_readb(sdport->base_io + EMMC_DATA_PAD_DRV);
		break;
	case MMC_IOS_SET_PAD_DRV:
		cr_writeb(CLOCK_DRIVING, sdport->base_io + EMMC_CARD_PAD_DRV); //clock pad driving
		cr_writeb(CMD_DRIVING, sdport->base_io + EMMC_CMD_PAD_DRV);   //cmd pad driving
		cr_writeb(DATA_DRIVING, sdport->base_io + EMMC_DATA_PAD_DRV);  //data pads driving
		sync();
		sync();
		sync();
		break;
	case MMC_IOS_RESTORE_PAD_DRV:
		cr_writeb(card_pad_val, sdport->base_io + EMMC_CARD_PAD_DRV); //clock pad driving
		cr_writeb(cmd_pad_val, sdport->base_io + EMMC_CMD_PAD_DRV);   //cmd pad driving
		cr_writeb(data_pad_val, sdport->base_io + EMMC_DATA_PAD_DRV);  //data pads driving
		sync();
		sync();
		break;
	}
	sync();
	sync();
}

static void rtkcr_set_div(struct rtkemmc_host *sdport, u32 set_div)
{
	u32 iobase = sdport->base_io;
	u32 tmp_div;
	unsigned long flags;

	spin_lock_irqsave(&sdport->lock, flags);
	tmp_div = cr_readb(iobase + SD_CONFIGURE1) & ~MASK_CLOCK_DIV;
	cr_writeb(tmp_div | set_div, iobase + SD_CONFIGURE1);
	sync();
	spin_unlock_irqrestore(&sdport->lock, flags);
}

static void rtksd_set_bits(struct rtkemmc_host *sdport, u8 set_bit)
{
	u32 iobase = sdport->base_io;
	u32 tmp_bits;
	unsigned long flags;

	spin_lock_irqsave(&sdport->lock, flags);
	tmp_bits = cr_readb(iobase + SD_CONFIGURE1);
	if ((tmp_bits & MASK_BUS_WIDTH) != set_bit) {
		tmp_bits &= ~MASK_BUS_WIDTH;
		cr_writeb(tmp_bits | set_bit, iobase + SD_CONFIGURE1);
	}
	sync();
	spin_unlock_irqrestore(&sdport->lock, flags);
}

static void rtkcr_set_speed(struct rtkemmc_host *sdport, u8 level)
{
	u32 iobase = sdport->base_io;
	unsigned long flags;

	spin_lock_irqsave(&sdport->lock, flags);

	switch (level) {
	case 0:  //ddr52 , highest speed
		cr_writel(0x2100, iobase + EMMC_CKGEN_CTL);
		break;
	case 1:
		cr_writel(0x2101, iobase + EMMC_CKGEN_CTL);
		break;
	case 2:
	default:
		cr_writel(0x2102, iobase + EMMC_CKGEN_CTL);
		break;
	}
	sync();
	spin_unlock_irqrestore(&sdport->lock, flags);
}

static void rtkemmc_bus_speed_down(struct rtkemmc_host *sdport)
{
	cr_writeb(0x8,  sdport->base_io + SD_SAMPLE_POINT_CTL);    //sample point = SDCLK / 4
	cr_writeb(0x10, sdport->base_io + SD_PUSH_POINT_CTL);     //output ahead SDCLK /4
}

static u8 rtksd_get_rsp_len(u8 rsp_para)
{
	return rtk_crsd_get_rsp_len(rsp_para);
}

static u32 rtksd_get_cmd_timeout(struct sd_cmd_pkt *cmd_info)
{
	struct rtkemmc_host *sdport   = cmd_info->sdport;
	u16 block_count             = cmd_info->block_count;
	u32 tmout = 0;

	if (cmd_info->cmd->data) {
		tmout = msecs_to_jiffies(200);
		if (block_count > 0x100)
			tmout = tmout + msecs_to_jiffies(block_count >> 1);
	} else {
		tmout = msecs_to_jiffies(80);
	}

	cmd_info->timeout = sdport->tmout = tmout;
	return 0;
}

#define SD_ALLOC_LENGTH     2048
static int rtksd_allocate_dma_buf(struct rtkemmc_host *sdport, struct mmc_command *cmd)
{
	if (!sdport->rsp_buf) {
		sdport->rsp_buf = dma_alloc_coherent(sdport->dev, SD_ALLOC_LENGTH,
						     &sdport->paddr, GFP_KERNEL);
		sdport->rsp_buf_org = sdport->rsp_buf;
	} else {
		return 0;
	}

	if (!sdport->rsp_buf) {
		WARN_ON(1);
		cmd->error = -ENOMEM;
		return 0;
	}

	return 1;
}

static int rtksd_free_dma_buf(struct rtkemmc_host *sdport)
{
	if (sdport->rsp_buf_org)
		dma_free_coherent(sdport->dev, SD_ALLOC_LENGTH, sdport->rsp_buf_org, sdport->paddr);
	else
		return 0;

	return 1;
}

static unsigned char *rtksd_get_buffer_start_addr(struct rtkemmc_host *sdport)
{
	if (sdport->rsp_buf_org)
		return sdport->rsp_buf_org;

	return NULL;
}

static void rtksd_set_rspparam(struct rtkemmc_host *sdport, struct sd_cmd_pkt *cmd_info)
{
	unsigned char cfg3 = 0, cfg1 = 0;

	cfg3 = cr_readb(sdport->base_io + SD_CONFIGURE3) | SD_CMD_RSP_TO;
	cfg1 = cr_readb(sdport->base_io + SD_CONFIGURE1) &
	       (MASK_CLOCK_DIV | MASK_BUS_WIDTH | MASK_MODE_SELECT);

	//correct emmc setting for rt1195
	switch (cmd_info->cmd->opcode) {
	case MMC_GO_IDLE_STATE:
	    cmd_info->rsp_para1 = cfg1 | SD1_R0;
	    cmd_info->rsp_para2 = (IGN_WR_CRC_ERR_EN | CRC16_CAL_DIS) | SD_R0;
	    cmd_info->rsp_para3 = 0;
	break;
	case MMC_SEND_OP_COND:
	    cmd_info->rsp_para1 = cfg1 | SD1_R0;
	    cmd_info->rsp_para2 = SD_R3 | CRC7_CHK_DIS;
	    cmd_info->rsp_para3 = 0;
	if (soc_is_rtk1195() && (realtek_rev() >= RTD_CHIP_A01))
		cmd_info->cmd->arg = MMC_SECTOR_ADDR | MMC_VDD_165_195;
	else
		/* sector mode */
		cmd_info->cmd->arg = MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
				      MMC_VDD_33_34 | MMC_SECTOR_ADDR;
	break;
	case MMC_ALL_SEND_CID:
	    cmd_info->rsp_para1 = cfg1 | SD1_R0;
	    cmd_info->rsp_para2 = SD_R2;
	    cmd_info->rsp_para3 = SD2_R0;
	break;
	case MMC_SET_RELATIVE_ADDR:
	    cmd_info->rsp_para1 = cfg1 | SD1_R0;
	    cmd_info->rsp_para2 = SD_R1 | CRC16_CAL_DIS;
	    cmd_info->rsp_para3 = cfg3;
	    cmd_info->cmd->arg = 0x10000;
	break;
	case MMC_SEND_CSD:
	case MMC_SEND_CID:
	    cmd_info->rsp_para1 = cfg1 | SD1_R0;
	    cmd_info->rsp_para2 = SD_R2;
	    cmd_info->rsp_para3 = cfg3;
	    cmd_info->cmd->arg = 0x10000;
	break;
	case MMC_SEND_EXT_CSD:
	    cmd_info->rsp_para1 = cfg1 | SD1_R0;
	    cmd_info->rsp_para2 = SD_R1;
	    cmd_info->rsp_para3 = cfg3;
	break;
	case MMC_SLEEP_AWAKE:
	    cmd_info->rsp_para1 = cfg1 | SD1_R0;
	    cmd_info->rsp_para2 = SD_R1b;
	    cmd_info->rsp_para3 = SD_CMD_RSP_TO;
	    cmd_info->cmd->arg = cmd_info->cmd->arg;
	    pr_info("%s : cmd5 arg=0x%08x\n", __func__, cmd_info->cmd->arg);
	break;
	case MMC_SELECT_CARD:
	    cmd_info->rsp_para1 = cfg1 | SD1_R0;
	    cmd_info->cmd->arg = cmd_info->cmd->arg;
	if (cmd_info->cmd->flags == (MMC_RSP_NONE | MMC_CMD_AC)) {
		pr_debug("%s : cmd7 with rsp none\n", __func__);
		cmd_info->rsp_para2 = RESP_TYPE_NON;
	} else {
		pr_debug("%s : cmd7 with rsp\n", __func__);
		cmd_info->rsp_para2 = SD_R1b | CRC16_CAL_DIS;
	}
	    cmd_info->rsp_para3 = -1;
	break;
	case MMC_SWITCH:
	    cmd_info->rsp_para1 = cfg1;
	    cmd_info->rsp_para2 = SD_R1 | CRC16_CAL_DIS;
	    cmd_info->rsp_para3 = SD_CMD_RSP_TO;
	break;
	case MMC_SEND_STATUS:
	    cmd_info->rsp_para1 = -1;
	    cmd_info->rsp_para2 = SD_R1 | CRC16_CAL_DIS;
	    cmd_info->rsp_para3 = SD_CMD_RSP_TO;
	    cmd_info->cmd->arg = 0x10000;
	break;
	case MMC_STOP_TRANSMISSION:
	    cmd_info->rsp_para1 = -1;
	    cmd_info->rsp_para2 = SD_R1 | CRC16_CAL_DIS;
	    cmd_info->rsp_para3 = -1;
	break;
	case MMC_READ_MULTIPLE_BLOCK:
	    cmd_info->rsp_para1 = -1;
	    cmd_info->rsp_para3 = (SD_CMD_RSP_TO | ADDR_BYTE_MODE);
	break;
	case MMC_WRITE_BLOCK:
	    cmd_info->rsp_para1 = -1;
	    cmd_info->rsp_para3 = (SD_CMD_RSP_TO | ADDR_BYTE_MODE);
	break;
	case MMC_SET_BLOCK_COUNT:
	    cmd_info->rsp_para1 = -1;
	    cmd_info->rsp_para2 = SD_R1 | CRC16_CAL_DIS;
	    cmd_info->rsp_para3 = SD_CMD_RSP_TO;
	break;
	case MMC_WRITE_MULTIPLE_BLOCK:
	    cmd_info->rsp_para1 = -1;
	    cmd_info->rsp_para3 = (SD_CMD_RSP_TO | ADDR_BYTE_MODE);
	break;
	default:
	    cmd_info->rsp_para1 = -1;     //don't update
	    cmd_info->rsp_para3 = -1;     //don't update
	break;
	}
}

static int SD_SendCMDGetRSP_Cmd(struct sd_cmd_pkt *cmd_info, int ignore)
{
	u8 cmd_idx              = cmd_info->cmd->opcode;
	u32 sd_arg;
	s8 rsp_para1 = 0;
	s8 rsp_para2 = 0;
	s8 rsp_para3 = 0;
	u8 rsp_len              = cmd_info->rsp_len;
	u32 *rsp                = (u32 *)&cmd_info->cmd->resp;
	struct rtkemmc_host *sdport = cmd_info->sdport;
	struct mmc_host *host = sdport->mmc;
	u32 iobase = sdport->base_io;
	int err, retry_count = 0;
	u32 dma_val = 0;
	u32 byte_count = 0x200, block_count = 1, cpu_mode = 0, sa = 0;
	u32 buf_ptr = 0;
	u16 state = 0;
	unsigned char *rsp_buf;

	rtksd_set_rspparam(sdport, cmd_info);   //for 119x
	sd_arg              = cmd_info->cmd->arg;
	rsp_para1           = cmd_info->rsp_para1;
	rsp_para2           = cmd_info->rsp_para2;
	rsp_para3           = cmd_info->rsp_para3;

RET_CMD:

	if (WARN_ON_ONCE(!rsp))
		return -EINVAL;
	if (sdport->g_crinit == 0 && cmd_idx > MMC_SET_RELATIVE_ADDR) {
		pr_debug("%s : ignore cmd:0x%02x since we're still in emmc init stage\n", DRIVER_NAME, cmd_idx);
		return CR_TRANSFER_FAIL;
	}

	if (rsp_para1 != -1)
		cr_writeb(rsp_para1, iobase + SD_CONFIGURE1);
	cr_writeb(rsp_para2,     iobase + SD_CONFIGURE2);
	if (rsp_para3 != -1)
		cr_writeb(rsp_para3, iobase + SD_CONFIGURE3);
	sdport->last_cmd[0] = (0x40 | cmd_idx);
	sdport->last_cmd[1] = (sd_arg >> 24) & 0xff;
	sdport->last_cmd[2] = (sd_arg >> 16) & 0xff;
	sdport->last_cmd[3] = (sd_arg >> 8) & 0xff;
	sdport->last_cmd[4] = sd_arg & 0xff;
	sdport->last_cmd[5] = 0x00;

	cr_writeb((0x40 | cmd_idx), iobase + SD_CMD0);
	cr_writeb((sd_arg >> 24) & 0xff, iobase + SD_CMD1);
	cr_writeb((sd_arg >> 16) & 0xff, iobase + SD_CMD2);
	cr_writeb((sd_arg >> 8) & 0xff, iobase + SD_CMD3);
	cr_writeb(sd_arg & 0xff,    iobase + SD_CMD4);
	cr_writeb(sdport->last_cmd[5],    iobase + SD_CMD5);
	sdport->cmd_opcode = cmd_idx;
	sync();
	if (sdport->send_cmd0)
		pr_debug("0 cmd0:0x%02x,cmd1:0x%02x,cmd2:0x%02x,cmd3:0x%02x,cmd4:0x%02x,cmd5:0x%02x\n",
			 cr_readb(iobase + SD_CMD0), cr_readb(iobase + SD_CMD1), cr_readb(iobase + SD_CMD2),
			cr_readb(iobase + SD_CMD3), cr_readb(iobase + SD_CMD4), cr_readb(iobase + SD_CMD5));
	rtksd_get_cmd_timeout(cmd_info);
	if (sdport->rtflags & RTKCR_FOPEN_LOG) {
		pr_info("cmd %s: cmd_idx=%u, rsp_addr=%p\nsd_arg=0x%08x; rsp_para1=0x%08x rsp_para2=0x%08x rsp_para3=0x%x rsp_len=0x%x cfg1=0x%02x,cfg2=0x%02x,cfg3=0x%02x\n",
			DRIVER_NAME, cmd_idx, rsp, sd_arg, rsp_para1, rsp_para2, rsp_para3, rsp_len,
			cr_readb(sdport->base_io + SD_CONFIGURE1), cr_readb(sdport->base_io + SD_CONFIGURE2),
			cr_readb(sdport->base_io + SD_CONFIGURE3));
	}
	cr_writel(0x00, iobase + CR_DMA_CTL1);   //espeical for R2
	cr_writel(0x00, iobase + CR_DMA_CTL2);   //espeical for R2
	cr_writel(0x00, iobase + CR_DMA_CTL3);   //espeical for R2
	sync();
	if (RESP_TYPE_17B & rsp_para2) {
		//remap the resp dst buffer to un-cache
		rsp_buf = rtksd_get_buffer_start_addr(sdport);
		buf_ptr = ((u32)sdport->paddr & ~0xff);
		sa = buf_ptr / 8;

		dma_val = RSP17_SEL | DDR_WR | DMA_XFER;
		cr_writeb(byte_count,       iobase + SD_BYTE_CNT_L);     //0x24
		cr_writeb(byte_count >> 8,    iobase + SD_BYTE_CNT_H);     //0x28
		cr_writeb(block_count,      iobase + SD_BLOCK_CNT_L);    //0x2C
		cr_writeb(block_count >> 8,   iobase + SD_BLOCK_CNT_H);    //0x30
		if (cpu_mode && iobase == EM_BASE_ADDR)
			cr_writel(CPU_MODE_EN, iobase + EMMC_CPU_ACC); //enable cpu mode
		else
			cr_writel(0, iobase + EMMC_CPU_ACC);
		cr_writel(sa, iobase + CR_DMA_CTL1);   //espeical for R2
		cr_writel(1, iobase + CR_DMA_CTL2);   //espeical for R2
		cr_writel(dma_val, iobase + CR_DMA_CTL3);   //espeical for R2
	} else if (RESP_TYPE_6B & rsp_para2) {
		cr_writel(0x00, iobase + EMMC_CPU_ACC);
	}

	sync();
	err = rtkcr_wait_opt_end(DRIVER_NAME, sdport, EMMC_SENDCMDGETRSP, cmd_idx, ignore);

	if (err == CR_TRANS_OK) {
		sync();
		if (buf_ptr) {
			//ignore start pattern
			rsp_buf = (unsigned char *)((uintptr_t)rsp_buf & ~0xff);
			*(((unsigned int *)rsp_buf) + 4) = cr_readb(iobase + SD_CMD5);
			rsp_buf++;
			rtksd_read_rsp(sdport, (u32 *)rsp_buf, rsp_len);
			if (cmd_idx == MMC_SEND_EXT_CSD) {
				pr_err("!!!!!!!!!!!!!!!! %s: MMC_SEND_EXT_CSD case should not be invoked !!!!!!!!!!!!!!!\n",
				       __func__);
				memcpy(rsp, (u32 *)rsp_buf, 512);
			} else {
				memcpy(rsp, (u32 *)rsp_buf, 16);
			}
		} else {
			rtksd_read_rsp(sdport, rsp, rsp_len);
		}
		sync();
		if (cmd_idx == MMC_SET_RELATIVE_ADDR)
			sdport->g_crinit = 1;
		//get cmd7 status
		if (cmd_info->cmd->flags == (MMC_RSP_NONE | MMC_CMD_AC) && cmd_idx == MMC_SELECT_CARD) {
			pr_info("get status =>\n");
			rtksd_send_status(host->card, &state, 0, 0);
		}
	} else {
		if (!ignore)
			pr_warn("%s: %s cmd trans fail, err=%d, ignore=%d, prevent_retry=%d, boot_mode=%d, cmd_idx=%d\n",
				DRIVER_NAME, __func__, err, ignore,
				sdport->prevent_retry, sdport->boot_mode, cmd_idx);
		if (sdport->prevent_retry) {
			pr_warn("ignore error when card in tuning state, sdport->boot_mode=%d, err=%d\n",
				sdport->boot_mode, err);
			return err;
		}
		if (sdport->boot_mode >= MODE_DDR)
			return err;
		if (cmd_idx == MMC_SEND_STATUS) //prevent dead lock looping
			return err;
		if (retry_count++ < MAX_CMD_RETRY_COUNT) {
			pr_err("d ignore=0x%02x,tuning=0x%02x,boot_mode=0x%02x\n",
			       ignore, sdport->tuning, sdport->boot_mode);
			if (ignore == 0x1 && cmd_idx == MMC_STOP_TRANSMISSION && sdport->tuning == 0x1)
				return 0;

			pr_warn("retry %d ---->\n", retry_count);
			err = error_handling(sdport, cmd_idx, ignore);
			goto RET_CMD;
		}
	}
	return err;
}

static int SD_SendCMDGetRSP(struct sd_cmd_pkt *cmd_info, int ignore)
{
	int rc;

	rc = SD_SendCMDGetRSP_Cmd(cmd_info, ignore);

	return rc;
}

static void duplicate_pkt(struct sd_cmd_pkt *sour, struct sd_cmd_pkt *dist)
{
	dist->sdport      = sour->sdport;
	dist->cmd         = sour->cmd;
	dist->data        = sour->data;

	dist->dma_buffer  = sour->dma_buffer;
	dist->byte_count  = sour->byte_count;
	dist->block_count = sour->block_count;

	dist->flags       = sour->flags;
	dist->rsp_para1    = sour->rsp_para1;
	dist->rsp_para2    = sour->rsp_para2;
	dist->rsp_para3    = sour->rsp_para3;
	dist->rsp_len     = sour->rsp_len;
	dist->timeout     = sour->timeout;
}

static int rtksd_err_handle(u16 cmdcode, struct sd_cmd_pkt *cmd_info)
{
	struct mmc_host *host       = cmd_info->sdport->mmc;
	u16 state = 0;
	int err = 0;

	if (host->card) {
		if (cmdcode == EMMC_AUTOWRITE2) {
			if (cmd_info->cmd->opcode == 18 ||
			    cmd_info->cmd->opcode == 25) {
				int stop_loop = 5;

				while (stop_loop--) {
					err = rtksd_stop_transmission(host->card, 0);
					if (err) {
						rtksd_send_status(host->card, &state, 0, 0);
						if (state == STATE_TRAN)
							break;
					} else {
						break;
					}
				}
			}
		}
		err = rtksd_wait_status(host->card, STATE_TRAN, 0, 0);
	}
	return err;
}

static int SD_Stream_Cmd34(u16 cmdcode, struct sd_cmd_pkt *cmd_info)
{
	u8 cmd_idx		= cmd_info->cmd->opcode;
	u16 byte_count              = cmd_info->byte_count;
	u16 block_count             = cmd_info->block_count;
	void *data                  = cmd_info->dma_buffer;
	struct rtkemmc_host *sdport   = cmd_info->sdport;
	u32 iobase                  = sdport->base_io;
	int err;
	u32 cpu_mode = 0;
	u32 sa = 0;

	if (WARN_ON_ONCE(!data))
		return -EINVAL;

	if (WARN_ON_ONCE(cmdcode != EMMC_AUTOREAD3  && cmdcode != EMMC_AUTOREAD4 &&
			 cmdcode != EMMC_AUTOWRITE3 && cmdcode != EMMC_AUTOWRITE4))
		return -EINVAL;

	sa = (u32)data / 8;

	cr_writel(0x00, iobase + CR_DMA_CTL1);   //espeical for R2
	cr_writel(0x00, iobase + CR_DMA_CTL2);   //espeical for R2
	cr_writel(0x00, iobase + CR_DMA_CTL3);   //espeical for R2
	cr_writeb(byte_count,       iobase + SD_BYTE_CNT_L);     //0x24
	cr_writeb(byte_count >> 8,    iobase + SD_BYTE_CNT_H);     //0x28
	cr_writeb(block_count,      iobase + SD_BLOCK_CNT_L);    //0x2C
	cr_writeb(block_count >> 8,   iobase + SD_BLOCK_CNT_H);    //0x30
	sync();

	if (cmd_info->cmd->data->flags & MMC_DATA_READ) {
		cr_writel(0x00, iobase + EMMC_CPU_ACC);
		cr_writel((u32)sa, iobase + CR_DMA_CTL1);
		cr_writel(block_count, iobase + CR_DMA_CTL2);
		cr_writel(DDR_WR | DMA_XFER, iobase + CR_DMA_CTL3);
	} else if (cmd_info->cmd->data->flags & MMC_DATA_WRITE) {
		cr_writel(0, iobase + EMMC_CPU_ACC);
		cr_writel((u32)sa, iobase + CR_DMA_CTL1);
		cr_writel(block_count, iobase + CR_DMA_CTL2);
		sync();
		cr_writel(DMA_XFER, iobase + CR_DMA_CTL3);
	} else { //sram read

		cpu_mode = 1;
		cr_writel(0, iobase + EMMC_CPU_ACC);
		cr_writel(CPU_MODE_EN, iobase + EMMC_CPU_ACC);
		cr_writel((u32)sa, iobase + CR_DMA_CTL1);
		cr_writel(block_count, iobase + CR_DMA_CTL2);
		sync();
		cr_writel(DDR_WR | DMA_XFER, iobase + CR_DMA_CTL3);
	}

	sdport->cmd_opcode = 0xf1;
	rtksd_get_cmd_timeout(cmd_info);
	err = rtkcr_wait_opt_end(DRIVER_NAME, sdport, cmdcode, cmd_idx, cpu_mode);
	return err;
}

static int SD_Stream_Cmd(u16 cmdcode, struct sd_cmd_pkt *cmd_info, unsigned int ignore)
{
	u8 cmd_idx              = cmd_info->cmd->opcode;
	u32 sd_arg              = cmd_info->cmd->arg;
	s8 rsp_para1             = cmd_info->rsp_para1;
	s8 rsp_para2             = cmd_info->rsp_para2;
	s8 rsp_para3             = cmd_info->rsp_para3;
	int rsp_len             = cmd_info->rsp_len;
	u32 *rsp                = (u32 *)&cmd_info->cmd->resp;
	u16 byte_count          = cmd_info->byte_count;
	u16 block_count         = cmd_info->block_count;
	struct rtkemmc_host *sdport = cmd_info->sdport;
	u32 iobase = sdport->base_io;
	int err;
	u8 *data              = cmd_info->dma_buffer;
	u32 cpu_mode = 0;
	u32 sa = 0, retry_count = 0;

	if (sdport->rtflags & RTKCR_FOPEN_LOG) {
		pr_info("%s strm: cmd_idx=%u, rsp_addr=%p\nsd_arg=0x%08x; rsp_para1=0x%x rsp_para2=0x%x rsp_para3=0x%x rsp_len=0x,%x cfg1=0x%02x,cfg2=0x%02x,cfg3=0x%02x\n",
			DRIVER_NAME, cmd_idx, rsp, sd_arg, rsp_para1, rsp_para2, rsp_para3, rsp_len,
			cr_readb(sdport->base_io + SD_CONFIGURE1), cr_readb(sdport->base_io + SD_CONFIGURE2),
			cr_readb(sdport->base_io + SD_CONFIGURE3));
	}

	if (WARN_ON_ONCE(!data || !rsp))
		return -EINVAL;

	if (sdport->g_crinit == 0 && cmd_idx > MMC_SET_RELATIVE_ADDR) {
		pr_debug("%s : ignore cmd:0x%02x since we're still in emmc init stage\n", DRIVER_NAME, cmd_idx);
		return CR_TRANSFER_FAIL;
	}

	rtksd_set_rspparam(sdport, cmd_info);   //for 119x
	sd_arg              = cmd_info->cmd->arg;
	if (rsp_para1 != -1)
		rsp_para1           = cmd_info->rsp_para1;
	rsp_para2           = cmd_info->rsp_para2;
	if (rsp_para3 != -1)
		rsp_para3           = cmd_info->rsp_para3;
	sa = (u32)data / 8;

	if (cmdcode == EMMC_NORMALWRITE || cmdcode == EMMC_NORMALREAD)
		byte_count = 512;
STR_CMD_RET:
	sdport->last_cmd[0] = (0x40 | cmd_idx);
	sdport->last_cmd[1] = (sd_arg >> 24) & 0xff;
	sdport->last_cmd[2] = (sd_arg >> 16) & 0xff;
	sdport->last_cmd[3] = (sd_arg >> 8) & 0xff;
	sdport->last_cmd[4] = sd_arg & 0xff;
	sdport->last_cmd[5] = 0x00;

	cr_writeb(sdport->last_cmd[0],    iobase + SD_CMD0);           //0x10
	cr_writeb(sdport->last_cmd[1],    iobase + SD_CMD1);           //0x14
	cr_writeb(sdport->last_cmd[2],    iobase + SD_CMD2);           //0x18
	cr_writeb(sdport->last_cmd[3],    iobase + SD_CMD3);           //0x1C
	cr_writeb(sdport->last_cmd[4],    iobase + SD_CMD4);           //0x20
	cr_writeb(sdport->last_cmd[5],    iobase + SD_CMD5);           //0x20

	if (rsp_para1 != -1)
		cr_writeb(rsp_para1,         iobase + SD_CONFIGURE1);     //0x0C
	cr_writeb(rsp_para2,         iobase + SD_CONFIGURE2);     //0x0C
	if (rsp_para3 != -1)
		cr_writeb(rsp_para3,         iobase + SD_CONFIGURE3);     //0x0C

	cr_writel(0x00, iobase + CR_DMA_CTL1);   //espeical for R2
	cr_writel(0x00, iobase + CR_DMA_CTL2);   //espeical for R2
	cr_writel(0x00, iobase + CR_DMA_CTL3);   //espeical for R2
	cr_writeb(byte_count,       iobase + SD_BYTE_CNT_L);     //0x24
	cr_writeb(byte_count >> 8,    iobase + SD_BYTE_CNT_H);     //0x28
	cr_writeb(block_count,      iobase + SD_BLOCK_CNT_L);    //0x2C
	cr_writeb(block_count >> 8,   iobase + SD_BLOCK_CNT_H);    //0x30

	sync();

	if (cmd_info->cmd->data->flags & MMC_DATA_READ) {
		cr_writel(0x00, iobase + EMMC_CPU_ACC);
		cr_writel((u32)sa, iobase + CR_DMA_CTL1);
		cr_writel(block_count, iobase + CR_DMA_CTL2);

		sync();
		cr_writel(DDR_WR | DMA_XFER, iobase + CR_DMA_CTL3);
	} else if (cmd_info->cmd->data->flags & MMC_DATA_TUNING) {
		cr_writel(0x00, iobase + EMMC_CPU_ACC);
		    cr_writel((u32)sa, iobase + CR_DMA_CTL1);
		    cr_writel(block_count, iobase + CR_DMA_CTL2);

		    sync();
		    cr_writel(DAT64_SEL | DDR_WR | DMA_XFER, iobase + CR_DMA_CTL3);
	} else if ((cmd_info->cmd->data->flags & MMC_DATA_WRITE) ||
		   (cmd_info->cmd->data->flags & MMC_MICRON_60)) {
		cr_writel(0, iobase + EMMC_CPU_ACC);
		cr_writel((u32)sa, iobase + CR_DMA_CTL1);
		cr_writel(block_count, iobase + CR_DMA_CTL2);
		sync();
		cr_writel(DMA_XFER, iobase + CR_DMA_CTL3);
	} else { //sram read

		cpu_mode = 1;
		cr_writel(0, iobase + EMMC_CPU_ACC);
		cr_writel(CPU_MODE_EN, iobase + EMMC_CPU_ACC);
		cr_writel((u32)sa, iobase + CR_DMA_CTL1);
		cr_writel(block_count, iobase + CR_DMA_CTL2);
		sync();
		cr_writel(DDR_WR | DMA_XFER, iobase + CR_DMA_CTL3);
	}

	sdport->cmd_opcode = cmd_idx;

	rtksd_get_cmd_timeout(cmd_info);

	/*
	 * Pass cpu_mode, not ignore: this argument feeds rtkcr_wait_opt_end()'s
	 * cpu_mode parameter, and the sibling caller above already passes the
	 * computed flag. No runtime change today -- the callee currently ignores
	 * the parameter -- but the two call sites no longer disagree.
	 */
	err = rtkcr_wait_opt_end(DRIVER_NAME, sdport, cmdcode, cmd_idx, cpu_mode);
	if (err == CR_TRANSFER_IGN)
		err = RTK_SUCC;

	if (err == RTK_SUCC) {
		if (!(cmdcode == EMMC_AUTOREAD1 || cmdcode == EMMC_AUTOWRITE1))
			rtksd_read_rsp(sdport, rsp, rsp_len);
	} else {
		if (ignore)
			return err;
		if (retry_count++ < MAX_CMD_RETRY_COUNT) {
			err = error_handling(sdport, cmd_idx, ignore);
			goto STR_CMD_RET;
		}
	}
	return err;
}

static u32 rtkcr_chk_cmdcode(struct mmc_command *cmd)
{
	u32 cmdcode;

	if (cmd->opcode < 56) {
		cmdcode = (u32)rtk_sd_cmdcode[cmd->opcode][0];
		WARN_ON(!cmd->data);
		if (cmd->data->flags & MMC_DATA_WRITE) {
			if (cmd->opcode == 42)
				cmdcode = EMMC_NORMALWRITE;
			else if (cmd->opcode == 56)
				cmdcode = EMMC_AUTOWRITE2;
		}
	} else {
		cmdcode = EMMC_CMD_UNKNOW;
	}

	return cmdcode;
}

static u32 rtkcr_chk_r1_type(struct mmc_command *cmd)
{
	return 0;
}

static u8 rtk_get_rsp_type(struct mmc_command *cmd)
{
	u32 rsp_type;

	if (mmc_resp_type(cmd) == MMC_RSP_R1)
		rsp_type = SD_R1;
	else if (mmc_resp_type(cmd) == MMC_RSP_R1B)
		rsp_type = SD_R1b;
	else if (mmc_resp_type(cmd) == MMC_RSP_R2)
		rsp_type = SD_R2;
	else if (mmc_resp_type(cmd) == MMC_RSP_R3)
		rsp_type = SD_R3;
	else if (mmc_resp_type(cmd) == MMC_RSP_R6)
		rsp_type = SD_R6;
	else if (mmc_resp_type(cmd) == MMC_RSP_R7)
		rsp_type = SD_R7;
	else
		rsp_type = SD_R0;
	return rsp_type;
}

static int SD_Stream(struct sd_cmd_pkt *cmd_info)
{
	int err = 0;
	u32 i;
	struct scatterlist *sg;
	u32 dir = 0;
	u32 dma_nents = 0;
	u32 dma_leng;
	u32 dma_addr;
	u32 dma_addr_sys = 0;
	u32 old_arg;
	u8 one_blk = 0;
	u8 f_in_dma = 0;
	u16 cmdcode = 0;

	struct mmc_host *host = cmd_info->sdport->mmc;
	struct rtkemmc_host *sdport = cmd_info->sdport;

	rtksd_set_rspparam(sdport, cmd_info);   //for 119x

	if (cmd_info->data->flags & MMC_DATA_READ) {
		dir = DMA_FROM_DEVICE;
	} else {
		if (host->card && mmc_card_cmd24_err(host->card))
			one_blk = 1;

		dir = DMA_TO_DEVICE;
	}

	cmd_info->data->bytes_xfered = 0;
	dma_nents = dma_map_sg(mmc_dev(host), cmd_info->data->sg,
			       cmd_info->data->sg_len,  dir);
	if (dir == DMA_TO_DEVICE)
		dma_sync_sg_for_device(mmc_dev(host), cmd_info->data->sg,
				       cmd_info->data->sg_len, dir);
	sg = cmd_info->data->sg;

	old_arg = cmd_info->cmd->arg;

	for (i = 0; i < dma_nents; i++, sg++) {
		dma_leng = sg_dma_len(sg);

		if (dma_leng & 0x1ff) {
			dma_addr_sys = sg_dma_address(sg);

			if (sdport->tmp_buf) {    //use tmp buffer
				f_in_dma = 1;
				dma_addr = sdport->tmp_buf_phy_addr;
				WARN_ON(1);
			} else {                  //use default buffer
				dma_addr = dma_addr_sys;
			}
		} else {
			dma_addr = sg_dma_address(sg);
		}

		if (one_blk) {    /* occur at write case only */
			u8 i;
			u32 blk_cnt;
			struct sd_cmd_pkt tmp_pkt;

			    blk_cnt = dma_leng / BYTE_CNT;
			if (blk_cnt == 0) {
				WARN_ON(1);
				blk_cnt = 1;
			}
			    duplicate_pkt(cmd_info, &tmp_pkt);

			    tmp_pkt.byte_count  = BYTE_CNT;
			    tmp_pkt.block_count = 1;
			    tmp_pkt.dma_buffer  = (unsigned char *)dma_addr;

			if (tmp_pkt.cmd->opcode == 25)
				tmp_pkt.cmd->opcode = 24;

			if (tmp_pkt.cmd->opcode == 18)
				tmp_pkt.cmd->opcode = 17;

			    cmdcode = sdport->ops->chk_cmdcode(tmp_pkt.cmd);
			for (i = 0; i < blk_cnt; i++) {
				err = SD_Stream_Cmd(cmdcode, &tmp_pkt, 0);
				if (err == 0) {
					if (host->card) {
						if (tmp_pkt.cmd->opcode == 24)
							host->card->state &= ~MMC_STATE_CMD24_ERR;
					}

					if (host->card && mmc_card_blockaddr(host->card))
						tmp_pkt.cmd->arg += 1;
					else
						tmp_pkt.cmd->arg += BYTE_CNT;

					    tmp_pkt.dma_buffer += BYTE_CNT;
					    tmp_pkt.data->bytes_xfered += BYTE_CNT;

				} else {
					if (rtksd_err_handle(cmdcode, &tmp_pkt)) {
						if (tmp_pkt.cmd->opcode == 24) {
							int err34 = 0;

							    err34 = SD_Stream_Cmd34(EMMC_AUTOWRITE3, &tmp_pkt);
							if (err34)
								rtksd_err_handle(EMMC_AUTOWRITE3, &tmp_pkt);
							else
								rtksd_wait_status(host->card, STATE_TRAN, 0, 0);
						}
					}
					break;
				}
		}
		} else {
			u32 blk_cnt;

			    cmd_info->byte_count = BYTE_CNT;     //rtk HW limite, one trigger 512 byte pass.
			    blk_cnt = dma_leng / BYTE_CNT;

			if (blk_cnt == 0 && dma_leng)
				blk_cnt = 1;

			    cmd_info->block_count = blk_cnt;
			    cmd_info->dma_buffer = (unsigned char *)dma_addr;

			    cmdcode = sdport->ops->chk_cmdcode(cmd_info->cmd);
			    err = SD_Stream_Cmd(cmdcode, cmd_info, 0);

			if (err == 0) {
				if (host->card) {
					if (cmd_info->cmd->opcode == 25 &&
					    cmdcode == EMMC_AUTOWRITE2) {
						int stop_err;

						stop_err = rtksd_stop_transmission(host->card, 0);
						if (stop_err)
							goto ERR_HANDLE;
					}
					if (dir == DMA_TO_DEVICE &&
					    rtksd_wait_status(host->card, STATE_TRAN, 0, 0))
						err = -1;
				}

				if (!err) {
					if (host->card && mmc_card_blockaddr(host->card))
						cmd_info->cmd->arg += cmd_info->block_count;
					else
						cmd_info->cmd->arg += dma_leng;

					    cmd_info->data->bytes_xfered += dma_leng;
				}

			} else {
ERR_HANDLE:
				if (rtksd_err_handle(cmdcode, cmd_info)) {
					if (cmd_info->cmd->opcode == 18 ||
					    cmd_info->cmd->opcode == 25) {
						if (host->card) {
							/*
							 * Already recovering; a
							 * failed stop is not
							 * actionable here, the
							 * wait below is what
							 * matters.
							 */
							rtksd_stop_transmission(host->card, 0);
							rtksd_wait_status(host->card, STATE_TRAN, 0, 0);
						}

					} else if (cmd_info->cmd->opcode == 24) {
						int err34 = 0;

						err34 = SD_Stream_Cmd34(EMMC_AUTOWRITE3, cmd_info);
						if (err34) {
							rtksd_err_handle(EMMC_AUTOWRITE3, cmd_info);
						} else {
							if (host->card)
								rtksd_wait_status(host->card, STATE_TRAN, 0, 0);
						}
					}
				}
			}
		}

		if (err) {
			cmd_info->cmd->arg = old_arg;
			break;
		}

		if (f_in_dma) {
			u32 i;
			u8 *ptr1 = phys_to_virt(dma_addr_sys);
			u8 *ptr2 = phys_to_virt(dma_addr);

			    f_in_dma = 0;
			for (i = 0; i < dma_leng; i++)
				*ptr1 = *ptr2;
		}
	}
	dma_unmap_sg(mmc_dev(host), cmd_info->data->sg,
		     cmd_info->data->sg_len,  dir);

	//reset cmd0
	sdport->last_cmd[0] = 0x00;

	return err;
}

static void rtkcr_req_end_tasklet(unsigned long param)
{
	struct rtkemmc_host *sdport;
	struct mmc_request *mrq;
	unsigned long flags;

	sdport = (struct rtkemmc_host *)param;
	spin_lock_irqsave(&sdport->lock, flags);
	mrq = sdport->mrq;
	sdport->mrq = NULL;

	spin_unlock_irqrestore(&sdport->lock, flags);
	mmc_request_done(sdport->mmc, mrq);
}

static void rtksd_send_command(struct rtkemmc_host *sdport, struct mmc_command *cmd)
{
	int rc = 0;
	struct sd_cmd_pkt cmd_info;

	memset(&cmd_info, 0, sizeof(struct sd_cmd_pkt));

	if (!sdport || !cmd) {
		pr_err("%s: sdport or cmd is null\n", DRIVER_NAME);
		return;
	}
	cmd_info.cmd    = cmd;
	cmd_info.sdport = sdport;
	cmd_info.rsp_para2 = rtk_get_rsp_type(cmd_info.cmd);
	cmd_info.rsp_len  = rtksd_get_rsp_len(cmd_info.rsp_para2);
	if (cmd->data) {
		cmd_info.data = cmd->data;
		if (cmd->data->flags == MMC_DATA_READ) {
		    /* do nothing */
		} else if (cmd->data->flags == MMC_DATA_WRITE) {
			if (sdport->wp == 1) {
				pr_warn("%s: card is locked!",
					DRIVER_NAME);
				rc = -1;
				cmd->retries = 0;
				goto err_out;
			}
		} else {
			pr_err("error: cmd->data->flags=%d\n",
			       cmd->data->flags);
			    cmd->error = -MMC_ERR_INVALID;
			    cmd->retries = 0;
			goto err_out;
		}
		if (sdport->tuning)
			pr_info("................[%s:HS200] still tuning..............\n", DRIVER_NAME);
		rc = SD_Stream(&cmd_info);
	} else {
		rc = SD_SendCMDGetRSP(&cmd_info, 0);
	}
	if (cmd->opcode == MMC_SWITCH) {
		if ((cmd->arg & 0xffff00ff) == 0x03b30001) {
			if ((cmd->arg & 0x0000ff00) == 0)
				sdport->rtflags |= RTKCR_USER_PARTITION;
			else
				sdport->rtflags &= ~RTKCR_USER_PARTITION;
		}
	}
	if (cmd->opcode == MMC_SELECT_CARD) {
		rtkcr_set_div(sdport, EMMC_CLOCK_DIV_NON);
		mmc_select_sdr50_push_sample(sdport);
	}

err_out:
	if (rc) {
		if (rc == -RTK_RMOV)
			cmd->retries = 0;

		cmd->error = -MMC_ERR_FAILED;
	}
	tasklet_schedule(&sdport->req_end_tasklet);
}

static void rtksd_request(struct mmc_host *host, struct mmc_request *mrq)
{
	struct rtkemmc_host *sdport;
	struct mmc_command *cmd;

	sdport = mmc_priv(host);
	if (WARN_ON_ONCE(sdport->mrq)) {
		mrq->cmd->error = -EBUSY;
		mmc_request_done(host, mrq);
		return;
	}

	down_write(&sdport->rw_sem);
	cmd = mrq->cmd;
	sdport->mrq = mrq;

	if (!(sdport->rtflags & RTKCR_FCARD_DETECTED)) {
		cmd->error = -MMC_ERR_RMOVE;
		cmd->retries = 0;
		goto done;
	}

	if (sdport && cmd) {
		rtksd_allocate_dma_buf(sdport, cmd);
		rtksd_send_command(sdport, cmd);
	} else {
done:
		tasklet_schedule(&sdport->req_end_tasklet);
	}
	up_write(&sdport->rw_sem);
}

static int rtksd_execute_tuning(struct mmc_host *host, u32 opcode)
{
	struct rtkemmc_host *sdport;
	struct sd_cmd_pkt cmd_info;

	sdport = mmc_priv(host);
	memset(&cmd_info, 0, sizeof(struct sd_cmd_pkt));

	cmd_info.sdport = sdport;

	switch (host->ios.timing) {
	case MMC_TIMING_UHS_DDR50:
		mmc_tuning_ddr50(sdport);
		break;
	case MMC_TIMING_MMC_HS200:
		mmc_tuning_hs200(sdport);
		break;
	default:
		down_write(&sdport->rw_sem);
		mmc_select_sdr50_push_sample(sdport);
		up_write(&sdport->rw_sem);
		break;
	}

	return 0;
}

static void rtksd_set_ios(struct mmc_host *host, struct mmc_ios *ios)
{
	struct rtkemmc_host *sdport;

	sdport = mmc_priv(host);

	if (!sdport->resuming) {
		switch (ios->timing) {
		case MMC_TIMING_MMC_HS200:
			rtkcr_set_mode_selection(sdport, MODE_SD30);
			break;
		case MMC_TIMING_UHS_DDR50:
			rtkcr_set_mode_selection(sdport, MODE_DDR);
			break;
		case MMC_TIMING_MMC_HS:
		default:
			rtkcr_set_mode_selection(sdport, MODE_SD20);
			break;
		}

		switch (ios->timing) {
		case MMC_TIMING_MMC_HS:
			rtkcr_set_ldo(sdport, 0x57);  //50Mhz
			rtkcr_set_div(sdport, EMMC_CLOCK_DIV_NON);
			break;
		case MMC_TIMING_UHS_DDR50:
			rtkcr_set_ldo(sdport, 0x57);  //50Mhz
			rtkcr_set_div(sdport, EMMC_CLOCK_DIV_NON);
			break;
		case MMC_TIMING_MMC_HS200:
			rtkcr_set_ldo(sdport, 0xaf);  //200Mhz
			rtkcr_set_div(sdport, EMMC_CLOCK_DIV_NON);
			break;
		}
	}

	if (ios->bus_width == MMC_BUS_WIDTH_8)
		rtksd_set_bits(sdport, BUS_WIDTH_8);
	else if (ios->bus_width == MMC_BUS_WIDTH_4)
		rtksd_set_bits(sdport, BUS_WIDTH_4);
	else
		rtksd_set_bits(sdport, BUS_WIDTH_1);

	if (ios->bus_mode != MMC_BUSMODE_PUSHPULL) {  //MMC_BUSMODE_OPENDRAIN
		rtkcr_set_div(sdport, EMMC_CLOCK_DIV_128);
		rtksd_set_bits(sdport, BUS_WIDTH_1);
	}
}

static void rtkemmc_dump_registers(struct rtkemmc_host *sdport)
{
	if (sdport->tuning)
		return;
	pr_debug("card_select=0x%02x\n", sdport->reg_backup.card_select);
	pr_debug("sample_point_ctl=0x%02x\n", sdport->reg_backup.sample_point_ctl);
	pr_debug("push_point_ctl=0x%02x\n", sdport->reg_backup.push_point_ctl);
	pr_debug("sys_pll_emmc3=0x%08x\n", sdport->reg_backup.sys_pll_emmc3);
	pr_debug("pll_emmc1=0x%08x\n", sdport->reg_backup.pll_emmc1);
	pr_debug("sd_configure1=0x%02x\n", sdport->reg_backup.sd_configure1);
	pr_debug("sd_configure2=0x%02x\n", sdport->reg_backup.sd_configure2);
	pr_debug("sd_configure3=0x%02x\n", sdport->reg_backup.sd_configure3);
	pr_debug("EMMC_CARD_PAD_DRV=0x%02x\n", cr_readb(sdport->base_io + EMMC_CARD_PAD_DRV));
	pr_debug("EMMC_CMD_PAD_DRV=0x%02x\n", cr_readb(sdport->base_io + EMMC_CMD_PAD_DRV));
	pr_debug("EMMC_DATA_PAD_DRV=0x%02x\n", cr_readb(sdport->base_io + EMMC_DATA_PAD_DRV));
	sync();
}

static u32 rtkemmc_restore_registers(struct rtkemmc_host *sdport)
{
	if (!sdport->tuning)
		rtkcr_set_pad_driving(sdport, MMC_IOS_RESTORE_PAD_DRV, 0x66, 0x64, 0x66);
	cr_writeb(sdport->reg_backup.card_select, sdport->base_io + CARD_SELECT);
	cr_writeb(sdport->reg_backup.sample_point_ctl, sdport->base_io + SD_SAMPLE_POINT_CTL);
	cr_writeb(sdport->reg_backup.push_point_ctl, sdport->base_io + SD_PUSH_POINT_CTL);
	cr_writel(sdport->reg_backup.sys_pll_emmc3, SYS_PLL_EMMC3);
	cr_writel(sdport->reg_backup.pll_emmc1, PLL_EMMC1);
	cr_writeb(sdport->reg_backup.sd_configure1, sdport->base_io + SD_CONFIGURE1);
	cr_writeb(sdport->reg_backup.sd_configure2, sdport->base_io + SD_CONFIGURE2);
	cr_writeb(sdport->reg_backup.sd_configure3, sdport->base_io + SD_CONFIGURE3);
	cr_writel(sdport->reg_backup.emmc_pad_ctl, sdport->base_io + EMMC_PAD_CTL);
	cr_writel(sdport->reg_backup.emmc_ckgen_ctl, sdport->base_io + EMMC_CKGEN_CTL);
	sync();
	rtkemmc_dump_registers(sdport);
	return 0;
}

static u32 rtkemmc_backup_registers(struct rtkemmc_host *sdport)
{
	if (!sdport->tuning)
		rtkcr_set_pad_driving(sdport, MMC_IOS_GET_PAD_DRV, 0x66, 0x64, 0x66);
	sdport->reg_backup.card_select      = cr_readb(sdport->base_io + CARD_SELECT);
	sdport->reg_backup.sample_point_ctl = cr_readb(sdport->base_io + SD_SAMPLE_POINT_CTL);
	sdport->reg_backup.push_point_ctl   = cr_readb(sdport->base_io + SD_PUSH_POINT_CTL);
	sdport->reg_backup.sys_pll_emmc3    = cr_readl(SYS_PLL_EMMC3);
	sdport->reg_backup.pll_emmc1        = cr_readl(PLL_EMMC1);
	if (!sdport->tuning)
		pr_info("%s : pll_emmc1=0x%08x, reg_pll_emmc1=0x%08x\n", __func__,
			sdport->reg_backup.pll_emmc1, cr_readl(PLL_EMMC1));

	sdport->reg_backup.sd_configure1    = cr_readb(sdport->base_io + SD_CONFIGURE1);
	sdport->reg_backup.sd_configure2    = cr_readb(sdport->base_io + SD_CONFIGURE2);
	sdport->reg_backup.sd_configure3    = cr_readb(sdport->base_io + SD_CONFIGURE3);
	sdport->reg_backup.emmc_pad_ctl     = cr_readl(sdport->base_io + EMMC_PAD_CTL);
	sdport->reg_backup.emmc_ckgen_ctl   = cr_readl(sdport->base_io + EMMC_CKGEN_CTL);
	sync();
	rtkemmc_dump_registers(sdport);
	return 0;
}

static void rtkemmc_chk_card_insert(struct rtkemmc_host *sdport)
{
	struct mmc_host *host = sdport->mmc;

	rtksd_set_bits(sdport, BUS_WIDTH_1);
	rtkcr_set_mode_selection(sdport, MODE_SD20);
	rtkcr_set_speed(sdport, 2);
	rtkcr_set_div(sdport, EMMC_CLOCK_DIV_128);
	rtkcr_set_ldo(sdport, 0x57);  //50Mhz
	cr_writeb(0x0,  sdport->base_io + SD_SAMPLE_POINT_CTL);   //sample point = SDCLK / 4
	cr_writeb(0x0, sdport->base_io + SD_PUSH_POINT_CTL);     //output ahead SDCLK /4
	host->ops = &rtkemmc_ops;
	sdport->rtflags |= RTKCR_FCARD_DETECTED;
}

static void rtksd_timeout_timer(struct timer_list *t)
{
	struct rtkemmc_host *sdport = from_timer(sdport, t, timer);
	u32 int_status = 0;
	u32 sd_status1 = 0, sd_status2 = 0, bus_status = 0;
	u32 sd_trans = 0, dma_trans = 0;
	unsigned long flags;

	spin_lock_irqsave(&sdport->lock, flags);
	if (sdport->int_waiting) {
		rtkcr_hold_int_dec(sdport->base_io);
		rtkcr_clr_int_sta(sdport->base_io);
		sync();
		rtkcr_get_int_sta(sdport->base_io, &int_status);
		sdport->int_status  = int_status;
		rtkcr_get_sd_trans(sdport->base_io, &sd_trans);
		rtkcr_get_dma_trans(sdport->base_io, &dma_trans);

		rtkcr_get_sd_sta(sdport->base_io, &sd_status1, &sd_status2, &bus_status);

		sdport->sd_trans    = sd_trans;
		sdport->sd_status1   = sd_status1;
		sdport->sd_status2   = sd_status2;
		sdport->bus_status   = bus_status;
		sdport->dma_trans    = dma_trans;
	} else {
		WARN_ON(1);
	}

	if (sdport->int_waiting)
		rtk_op_complete(sdport);

	spin_unlock_irqrestore(&sdport->lock, flags);
}

static irqreturn_t rtksd_irq(int irq, void *dev)
{
	struct rtkemmc_host *sdport = dev;

	int irq_handled = 0;
	u32 int_status = 0;
	u32 sd_trans   = 0, dma_trans = 0;
	u32 sd_status1 = 0, sd_status2 = 0, bus_status = 0;

	rtkcr_hold_int_dec(sdport->base_io);
	rtkcr_get_int_sta(sdport->base_io, &int_status);
	rtkcr_clr_int_sta(sdport->base_io);
	sync();

	if (int_status & (ISRSTA_INT1 | ISRSTA_INT2 | ISRSTA_INT4)) { //card_end ?
		rtkcr_get_sd_trans(sdport->base_io, &sd_trans);
		rtkcr_get_dma_trans(sdport->base_io, &dma_trans);
		rtkcr_get_sd_sta(sdport->base_io, &sd_status1, &sd_status2, &bus_status);
		sync();

		sdport->int_status  = int_status;
		sdport->sd_trans    = sd_trans;
		sdport->sd_status1   = sd_status1;
		sdport->sd_status2   = sd_status2;
		sdport->bus_status   = bus_status;
		sdport->dma_trans    = dma_trans;
		if (sdport->int_waiting) {
			timer_delete(&sdport->timer);
			    rtk_op_complete(sdport);
		}
		irq_handled = 1;
	} else {
		pr_err("INT no END_STATE!!!\n");
	}

	sync();

	if (irq_handled)
		return IRQ_HANDLED;
	else
		return IRQ_NONE;
}

/* liao ********************
 * check read only func
 *
 *
 ***************************/
static int rtksd_get_ro(struct mmc_host *mmc)
{
	return 0;
}

static int rtksd_wait_status(struct mmc_card *card, u8 state, u8 divider, int ignore)
{
	struct mmc_command cmd;
	struct sd_cmd_pkt cmd_info;
	unsigned long timeend;
	int err;

	/*
	 * The core only publishes mmc_host.card once the card is fully
	 * initialised, so these helpers can be reached with no card while
	 * tuning. There is nothing to ask in that case.
	 */
	if (!card)
		return -ENODEV;

	timeend = jiffies + msecs_to_jiffies(100);    /* wait 100ms */

	do {
		memset(&cmd, 0, sizeof(struct mmc_command));
		memset(&cmd_info, 0, sizeof(struct sd_cmd_pkt));

		set_cmd_info(card, &cmd, &cmd_info,
			     MMC_SEND_STATUS,
			     (card->rca) << RCA_SHIFTER,
			     SD_R1 | divider);
		err = SD_SendCMDGetRSP_Cmd(&cmd_info, ignore);

		if (err)
			break;
		u8 cur_state = R1_CURRENT_STATE(cmd.resp[0]);

		err = -1;
		if (cur_state == state) {
			if (cmd.resp[0] & R1_READY_FOR_DATA) {
				err = 0;
				break;
			}
		}

	} while (time_before(jiffies, timeend));

	return err;
}

static int rtksd_send_status(struct mmc_card *card, u16 *state, u8 divider, int ignore)
{
	struct mmc_command cmd;
	struct sd_cmd_pkt cmd_info;
	int err = 0;

	/*
	 * The core only publishes mmc_host.card once the card is fully
	 * initialised, so these helpers can be reached with no card while
	 * tuning. There is nothing to ask in that case.
	 */
	if (!card)
		return -ENODEV;

	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&cmd_info, 0, sizeof(struct sd_cmd_pkt));

	set_cmd_info(card, &cmd, &cmd_info,
		     MMC_SEND_STATUS,
		 (card->rca) << RCA_SHIFTER,
		 SD_R1 | divider);
	err = SD_SendCMDGetRSP(&cmd_info, ignore);

	if (err) {
	} else {
		u8 cur_state = R1_CURRENT_STATE(cmd.resp[0]);
		*state = cur_state;
		if (!ignore)
			pr_info("cur_state=%s\n", state_tlb[cur_state]);
	}

	return err;
}

/*
 * tuning area
 */
static void mmc_CRT_reset(struct rtkemmc_host *sdport)
{
	if (WARN_ON_ONCE(!sdport))
		return;

	sync();
	rtkemmc_backup_registers(sdport);
	sync();
	cr_writel(cr_readl(SYS_SOFT_RESET2) & ((u32)~(1 << 11)), SYS_SOFT_RESET2);          //reset emmc
	sync();
	cr_writel(cr_readl(SYS_CLOCK_ENABLE1) & ((u32)~(1 << 24)),
		  SYS_CLOCK_ENABLE1);      //disable emmc clk
	sync();
	cr_writel(cr_readl(SYS_CLOCK_ENABLE1) | (1 << 24), SYS_CLOCK_ENABLE1);      //disable emmc clk
	sync();
	cr_writel(cr_readl(SYS_SOFT_RESET2) | (1 << 11), SYS_SOFT_RESET2);          //reset emmc
	sync();
	rtkcr_set_pad_driving(sdport, MMC_IOS_SET_PAD_DRV, 0xff, 0xff, 0xff);
	rtkemmc_restore_registers(sdport);
	sync();
}

static void mmc_phase_adjust(struct rtkemmc_host *sdport, u32 VP0, u32 VP1)
{
	u32 iobase = sdport->base_io;
//phase selection
	if ((VP0 == 0xff) & (VP1 == 0xff)) {
	} else if ((VP0 != 0xff) & (VP1 == 0xff)) {
		cr_writel(cr_readl(iobase + EMMC_CKGEN_CTL) | 0x70000,
			  iobase + EMMC_CKGEN_CTL);		//change clk to 4Mhz
		cr_writel(cr_readl(PLL_EMMC1) & 0xfffffffd, PLL_EMMC1);					//reset pll
		cr_writel(((cr_readl(PLL_EMMC1) & 0xffffff07) | (VP0 << 3)), PLL_EMMC1);			//vp0 phase:0x0~0x1f
		cr_writel(cr_readl(PLL_EMMC1) | 0x2, PLL_EMMC1);						//release reset pll
		cr_writel(cr_readl(iobase + EMMC_CKGEN_CTL) & 0xfff8ffff,
			  iobase + EMMC_CKGEN_CTL);		//change clock to PLL
		cr_writel(cr_readl(iobase + SD_CONFIGURE1) & 0xef, iobase + SD_CONFIGURE1);
	} else if ((VP0 == 0xff) & (VP1 != 0xff)) {
		cr_writel(cr_readl(iobase + EMMC_CKGEN_CTL) | 0x70000,
			  iobase + EMMC_CKGEN_CTL);		//change clk to 4Mhz
		cr_writel(cr_readl(PLL_EMMC1) & 0xfffffffd, PLL_EMMC1);					//reset pll
		cr_writel(((cr_readl(PLL_EMMC1) & 0xffffe0ff) | (VP1 << 8)), PLL_EMMC1);			//vp0 phase:0x0~0x1f
		cr_writel(cr_readl(PLL_EMMC1) | 0x2, PLL_EMMC1);						//release reset pll
		cr_writel(cr_readl(iobase + EMMC_CKGEN_CTL) & 0xfff8ffff,
			  iobase + EMMC_CKGEN_CTL);		//change clock to PLL
		cr_writel(cr_readl(iobase + SD_CONFIGURE1) & 0xef, iobase + SD_CONFIGURE1);
	} else {
		cr_writel(cr_readl(iobase + EMMC_CKGEN_CTL) | 0x70000,
			  iobase + EMMC_CKGEN_CTL);		//change clk to 4Mhz
		cr_writel(cr_readl(PLL_EMMC1) & 0xfffffffd, PLL_EMMC1);					//reset pll
		cr_writel(((cr_readl(PLL_EMMC1) & 0xffffff07) | (VP0 << 3)), PLL_EMMC1);			//vp0 phase:0x0~0x1f
		cr_writel(((cr_readl(PLL_EMMC1) & 0xffffe0ff) | (VP1 << 8)), PLL_EMMC1);			//vp1 phase:0x0~0x1f
		cr_writel(cr_readl(PLL_EMMC1) | 0x2, PLL_EMMC1);						//release reset pll
		cr_writel(cr_readl(iobase + EMMC_CKGEN_CTL) & 0xfff8ffff,
			  iobase + EMMC_CKGEN_CTL);		//change clock to PLL
		cr_writel(cr_readl(iobase + SD_CONFIGURE1) & 0xef, iobase + SD_CONFIGURE1);
	}
	udelay(300);
	sync();
	sync();
}

static int SEARCH_BEST(u32 window)
{
	int i, j, k, max;
	int window_temp[32];
	int window_start[32];
	int window_end[32];
	int window_max = 0;
	int window_best = 0;
	int parse_end = 1;

	for (i = 0; i < 0x20; i++) {
		window_temp[i] = 0;
		window_start[i] = 0;
		window_end[i] =  -1;
		}
	j = 1;
	i = 0;
	k = 0;
	max = 0;
	while ((i < 0x1f) && (k < 0x1f)) {
		parse_end = 0;
		for (i = window_end[j - 1] + 1; i < 0x20; i++) {
			if (((window >> i) & 1) == 1) {
				window_start[j] = i;
				break;
				}
			}
		if (i == 0x20)
			break;
		for (k = window_start[j] + 1; k < 0x20; k++) {
			if (((window >> k) & 1) == 0) {
				window_end[j] = k - 1;
				parse_end = 1;
				break;
				}
			}
		if (parse_end == 0)
			window_end[j] = 0x1f;
		j++;
		}
	for (i = 1; i < j; i++)
		window_temp[i] = window_end[i] - window_start[i] + 1;
	if (((window & 1) == 1) && (((window >> 0x1f) & 1) == 1)) {
		window_temp[1] = window_temp[1] + window_temp[j - 1];
		window_start[1] = window_start[j - 1];
		}
	for (i = 1; i < j; i++) {
		if (window_temp[i] > window_max) {
			window_max = window_temp[i];
			max = i;
			}
		}
	if (((window & 1) == 1 && ((window >> 0x1f) & 1) == 1) && max == 1)
		window_best = ((window_start[max] + window_end[max] + 0x20) / 2) & 0x1f;
	else
		window_best = ((window_start[max] + window_end[max]) / 2) & 0x1f;
	return window_best;
}

static int mmc_tuning_hs200(struct rtkemmc_host *sdport)
{
	u32 TX_window = 0xffffffff;
	int TX_best = 0xff;
	u32 RX_window = 0xffffffff;
	int RX_best = 0xff;
	int i = 0;

	down_write(&sdport->rw_sem);
	sdport->tuning = 1;
	if (!sdport->resuming)
		sdport->boot_mode = MODE_SD30;

	rtkcr_set_ldo(sdport, 0xaf);
	rtkcr_set_speed(sdport, 0);        //no wrapper divider
	rtkcr_set_pad_driving(sdport, MMC_IOS_SET_PAD_DRV, 0xff, 0xff, 0xff);
	mmc_phase_adjust(sdport, 0, 0);	//VP0, VP1 phas
	udelay(100);
	sync();
	for (i = 0x0; i < 0x20; i++) {
		mmc_phase_adjust(sdport, i, 0xff);
		if (rtkcr_send_cmd25(sdport) != 0)
			TX_window = TX_window & (~(1 << i));
	}
	TX_best = SEARCH_BEST(TX_window);
	pr_info("[%s:HS200] TX_WINDOW=0x%x, TX_best=0x%x\n", DRIVER_NAME, TX_window, TX_best);
	if (TX_window == 0x0) {
		pr_warn("[%s:HS200] TX tuning fail\n", DRIVER_NAME);
		sdport->tuning = 0;
		up_write(&sdport->rw_sem);
		return -1;
	}
	mmc_phase_adjust(sdport, TX_best, 0xff);
	i = 0;
	for (i = 0x0; i < 0x20; i++) {
		mmc_phase_adjust(sdport, 0xff, i);
		if (rtkcr_send_cmd18(sdport) != 0)
			RX_window = RX_window & (~(1 << i));
	}
	RX_best = SEARCH_BEST(RX_window);
	pr_info("[%s:HS200] RX_WINDOW=0x%x, RX_best=0x%x\n", DRIVER_NAME, RX_window, RX_best);
	if (RX_window == 0x0) {
		pr_warn("[%s:HS200] RX tuning fail\n", DRIVER_NAME);
		sdport->tuning = 0;
		up_write(&sdport->rw_sem);
		return -2;
	}
	mmc_phase_adjust(sdport, 0xff, RX_best);
	mmc_CRT_reset(sdport);
	sdport->tuning = 0;
	up_write(&sdport->rw_sem);
	return 0;
}

static int mmc_tuning_ddr50(struct rtkemmc_host *sdport)
{
	u32 TX_window = 0xffffffff;
	int TX_best = 0xff;
	u32 RX_window = 0xffffffff;
	int RX_best = 0xff;
	int i = 0;

	//set current boot mode
	down_write(&sdport->rw_sem);
	sdport->tuning = 1;
	if (!sdport->resuming)
		sdport->boot_mode = MODE_DDR;
	pr_debug("ddr50 sdport->boot_mode =%d\n", sdport->boot_mode);

	rtkcr_set_ldo(sdport, 0x57); //50Mhz
	rtkcr_set_speed(sdport, 0);    //no wrapper divider
	rtkcr_set_pad_driving(sdport, MMC_IOS_SET_PAD_DRV, 0x66, 0x64, 0x66);
	/* Using phase-shift clock for DDR mode command/data sample point selection */
	cr_writeb(0xa8, sdport->base_io + SD_SAMPLE_POINT_CTL);
	/* Using phase-shift clock for DDR mode command/data push point selection */
	cr_writeb(0x90, sdport->base_io + SD_PUSH_POINT_CTL);
	udelay(100);
	sync();
	for (i = 0x0; i < 0x20; i++) {
		mmc_phase_adjust(sdport, i, 0xff);
		if (rtkcr_send_cmd25(sdport) != 0)
			TX_window = TX_window & (~(1 << i));
	}
	TX_best = SEARCH_BEST(TX_window);
	pr_info("[%s:DDR50] TX_WINDOW=0x%x, TX_best=0x%x\n", DRIVER_NAME, TX_window, TX_best);
	if (TX_window == 0x0) {
		pr_warn("[%s:DDR50] TX tuning fail\n", DRIVER_NAME);
		sdport->tuning = 0;
		up_write(&sdport->rw_sem);
		return -1;
	}
	mmc_phase_adjust(sdport, TX_best, 0xff);

	sync();
	i = 0;
	for (i = 0x0; i < 0x20; i++) {
		mmc_phase_adjust(sdport, 0xff, i);
		if (rtkcr_send_cmd18(sdport) != 0)
			RX_window = RX_window & (~(1 << i));
	}
	RX_best = SEARCH_BEST(RX_window);
	pr_info("[%s:DDR50] RX_WINDOW=0x%x, RX_best=0x%x\n", DRIVER_NAME, RX_window, RX_best);
	sync();
	if (RX_window == 0x0) {
		pr_warn("[%s:DDR50] RX tuning fail\n", DRIVER_NAME);
		sdport->tuning = 0;
		up_write(&sdport->rw_sem);
		return -2;
	}
	mmc_phase_adjust(sdport, 0xff, RX_best);
	sdport->tuning = 0;
	up_write(&sdport->rw_sem);
	return 0;
}

static int mmc_select_sdr50_push_sample(struct rtkemmc_host *sdport)
{
	int err = 0;
	u32 iobase = sdport->base_io;

	sdport->tuning = 1;
	if (!sdport->resuming)
		sdport->boot_mode = MODE_SD20;
	cr_writeb(0x0,  sdport->base_io + SD_SAMPLE_POINT_CTL);   //sample point = SDCLK / 4
	cr_writeb(0x0, sdport->base_io + SD_PUSH_POINT_CTL);     //output ahead SDCLK /4
	rtkcr_set_ldo(sdport, 0x57); //50Mhz
	rtkcr_set_speed(sdport, 0);    //no wrapper divider
	/* SDR50 deliberately leaves the pad driving strength untouched. */
	rtkcr_set_div(sdport, EMMC_CLOCK_DIV_NON);
	sync();
	udelay(100);
	err = rtkcr_send_cmd25(sdport);
	if (err == 0) {
	} else {
		cr_writeb(0x10, iobase + SD_PUSH_POINT_CTL);	//Push point =output at falling edge
		sync();
		err = rtkcr_send_cmd25(sdport);
		if (err == 0) {
		} else {
			pr_err("sdr tuning : No good push point\n");
			sdport->tuning = 0;
			return -1;
		}
	}

	err = rtkcr_send_cmd18(sdport);
	if (err == 0) {
	} else {
		cr_writeb(8, iobase + SD_SAMPLE_POINT_CTL);	//sample point is delayed by 1/4 SDCLK period
		sync();
		err = rtkcr_send_cmd18(sdport);
		if (err == 0) {
		} else {
			pr_err("sdr tuning : No good Sample point\n");
			sdport->tuning = 0;
			return -2;
		}
	}
	sync();
	pr_debug("SDR select (sample/push) : 0x%02x/0x%02x\n",
		 cr_readb(sdport->base_io + SD_SAMPLE_POINT_CTL), cr_readb(sdport->base_io + SD_PUSH_POINT_CTL));
	rtkemmc_backup_registers(sdport);
	sdport->tuning = 0;

	return 0;
}

static int sample_ctl_switch(struct rtkemmc_host *sdport, int cmd_idx, int ignore)
{
	int err = 0;
	int mode_val = 0;
	u32 iobase = sdport->base_io;
	static int error_retry_count;
	int sts1_val = 0;

	mode_val = rtkcr_get_mode_selection(sdport);
	pr_info("cur point: s:0x%02x, p:0x%02x, mode=0x%02x\n",
		cr_readb(sdport->base_io + SD_SAMPLE_POINT_CTL), cr_readb(sdport->base_io + SD_PUSH_POINT_CTL),
		mode_val);
		switch (error_retry_count) {
		case 0:
				cr_writeb(0x0, iobase + SD_SAMPLE_POINT_CTL);    //sample point = SDCLK / 4
				cr_writeb(0x10, iobase + SD_PUSH_POINT_CTL);     //output ahead SDCLK /4
				pr_info("mode switch 0x0/0x10\n");
				error_retry_count++;
				break;
		case 1:
				cr_writeb(0x8, iobase + SD_SAMPLE_POINT_CTL);    //sample point = SDCLK / 4
				cr_writeb(0x10, iobase + SD_PUSH_POINT_CTL);     //output ahead SDCLK /4
				pr_info("mode switch 0x8/0x10\n");
				error_retry_count++;
				break;
		case 2:
				cr_writeb(0x8, iobase + SD_SAMPLE_POINT_CTL);    //sample point = SDCLK / 4
				cr_writeb(0x0, iobase + SD_PUSH_POINT_CTL);     //output ahead SDCLK /4
				pr_info("mode switch 0x8/0x0\n");
				error_retry_count++;
				break;
		case 3:
				cr_writeb(0x0, iobase + SD_SAMPLE_POINT_CTL);    //sample point = SDCLK / 4
				cr_writeb(0x0, iobase + SD_PUSH_POINT_CTL);     //output ahead SDCLK /4
				pr_info("mode switch 0x0/0x0\n");
				error_retry_count = 0;
				break;
		}
		sync();
		udelay(100);
		cr_writeb(0x14, iobase + CR_CARD_STOP);
		sync();
		udelay(100);

	if (cmd_idx != MMC_SET_BLOCKLEN && cmd_idx > MMC_SET_RELATIVE_ADDR) {
		err = polling_to_tran_state(sdport, ignore);
		sts1_val = cr_readb(iobase + SD_STATUS1);
		sync();
		if ((sts1_val & WRT_ERR_BIT) || (sts1_val & CRC16_STATUS) || (sts1_val & CRC7_STATUS))
			host_card_stop(sdport);
	}

	return err;
}

/*
 * error handling area
 */
int error_handling(struct rtkemmc_host *sdport, unsigned int cmd_idx, unsigned int ignore)
{
	u32 iobase = sdport->base_io;
	unsigned char sts1_val = 0;
	int err = 0;
	struct mmc_host *host = sdport->mmc;

		sts1_val = cr_readb(iobase + SD_STATUS1);
		pr_info("%s : status1 val=%02x, cmd_idx=0x%02x, sdport->boot_mode=0x%02x\n", __func__, sts1_val,
			cmd_idx, sdport->boot_mode);
		host_card_stop(sdport);
		if (cmd_idx > MMC_SET_RELATIVE_ADDR) {
			polling_to_tran_state(sdport, ignore);
			  sts1_val = cr_readb(iobase + SD_STATUS1);
			if ((sts1_val & WRT_ERR_BIT) || (sts1_val & CRC16_STATUS) || (sts1_val & CRC7_STATUS))
				rtksd_stop_transmission(host->card, ignore);
		}

		if (ignore)
			return 0;
		//till here, we are good to go
		if (sdport->boot_mode >= MODE_DDR) {
			pr_info("change mode from %d to %d --->\n", sdport->boot_mode, MODE_SD20);
			if (err)
				pr_err(" change mode result ==> fail\n");
			else
				pr_info(" change mode result ==> successful\n");
		} else {
			err = sample_ctl_switch(sdport, cmd_idx, ignore);
		}
		if (err)
			pr_err("error handling fail\n");
	return err;
}

int polling_to_tran_state(struct rtkemmc_host *sdport, int ignore)
{
	int err = 1, retry_cnt = 5;
	u16 ret_state = 0;
	struct mmc_host *host = sdport->mmc;

		err = 1;
		retry_cnt = 5;
		while (retry_cnt-- && err)
			err = rtksd_send_status(host->card, &ret_state, 0, ignore);
		sync();
		if (ret_state == STATE_DATA) {
			rtksd_stop_transmission(host->card, 1);
			err = rtksd_wait_status(host->card, STATE_TRAN, 0, ignore);
		} else if (ret_state == STATE_RCV) {
			rtksd_stop_transmission(host->card, 1);
			err = rtksd_wait_status(host->card, STATE_TRAN, 0, ignore);
		}
		sync();
	return err;
}

int rtkcr_send_cmd8(struct rtkemmc_host *sdport, unsigned int ignore)
{
	int ret_err = 0;
	struct sd_cmd_pkt cmd_info;
	struct mmc_host *host = sdport->mmc;
	int rsp_para1, rsp_para2, rsp_para3;
	unsigned char cfg3 = 0, cfg1 = 0;
	int sts1_val = 0;
	unsigned char *crd_ext_csd = NULL;
	u32 iobase = sdport->base_io;
	struct mmc_data *data = NULL;
	struct mmc_command *cmd = NULL;

	memset(&cmd_info, 0x00, sizeof(struct sd_cmd_pkt));

	crd_ext_csd = rtksd_get_buffer_start_addr(sdport);
	if (!crd_ext_csd) {
		pr_err("%s,%s : crd_ext_csd == NULL\n", DRIVER_NAME, __func__);
		return -5;
	}

	rsp_para1 = 0;
	rsp_para2 = 0;
	rsp_para3 = 0;
	cfg3 = cr_readb(iobase + SD_CONFIGURE3) | SD_CMD_RSP_TO;
	cfg1 = cr_readb(iobase + SD_CONFIGURE1) & (MASK_CLOCK_DIV | MASK_BUS_WIDTH | MASK_MODE_SELECT);
	rsp_para1 = cfg1 | SD1_R0;
	rsp_para2 = SD_R1;
	rsp_para3 = cfg3;

	if (!cmd_info.cmd) {
		cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
		if (!cmd) {
			ret_err = -ENOMEM;
			goto err8;
		}
		cmd_info.cmd = cmd;
	}
	cmd_info.sdport = sdport;
	cmd_info.cmd->arg = 0;
	cmd_info.cmd->opcode = MMC_SEND_EXT_CSD;
	cmd_info.rsp_para1	  = rsp_para1;
	cmd_info.rsp_para2	  = rsp_para2;
	cmd_info.rsp_para3	  = rsp_para3;
	cmd_info.rsp_len	 = rtksd_get_rsp_len(rsp_para2);
	cmd_info.byte_count  = 0x200;
	cmd_info.block_count = 1;
	cmd_info.dma_buffer = crd_ext_csd;
	if (!cmd_info.cmd->data) {
		data = kzalloc(sizeof(*data), GFP_KERNEL);
		if (!data) {
			ret_err = -ENOMEM;
			goto err8;
		}
		cmd_info.cmd->data = data;
		data->flags = MMC_DATA_READ;
	} else {
		cmd_info.cmd->data->flags = MMC_DATA_READ;
	}
	sync();
	ret_err = SD_Stream_Cmd(EMMC_NORMALREAD, &cmd_info, ignore);
	if (ret_err) {
		if (ignore)
			goto err8;
		host_card_stop(sdport);
		polling_to_tran_state(sdport, 1);
		sts1_val = cr_readb(iobase + SD_STATUS1);
		if ((sts1_val & CRC16_STATUS) || (sts1_val & CRC7_STATUS))
			rtksd_stop_transmission(host->card, 1);
	}
err8:
	if (cmd) {
		cmd_info.cmd = NULL;
		kfree(cmd);
		cmd = NULL;
	}
	kfree(data);
	data = NULL;
	sync();
	return ret_err;
}

int rtkcr_send_cmd18(struct rtkemmc_host *sdport)
{
	int ret_err = 0;
	struct sd_cmd_pkt cmd_info;
	struct mmc_host *host = sdport->mmc;
	int sts1_val = 0;
	unsigned char *crd_tmp_buffer = NULL;
	u32 iobase = sdport->base_io;
	struct mmc_data *data = NULL;
	struct mmc_command *cmd = NULL;

	memset(&cmd_info, 0x00, sizeof(struct sd_cmd_pkt));

	crd_tmp_buffer = rtksd_get_buffer_start_addr(sdport);
	if (!crd_tmp_buffer) {
		pr_err("%s,%s : crd_ext_csd == NULL\n", DRIVER_NAME, __func__);
		return -5;
	}

	if (!cmd_info.cmd) {
		cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
		if (!cmd) {
			ret_err = -ENOMEM;
			goto err18;
		}
		cmd_info.cmd = cmd;
	}
	cmd_info.sdport = sdport;
	cmd_info.cmd->arg = 0x100;
	cmd_info.cmd->opcode = MMC_READ_MULTIPLE_BLOCK;
	cmd_info.rsp_para1	  = -1;
	cmd_info.rsp_para2	  = SD_R1;
	cmd_info.rsp_para3	  = -1;
	cmd_info.rsp_len	 = rtksd_get_rsp_len(SD_R1);
	cmd_info.byte_count  = 0x200;
	cmd_info.block_count = 2;
	cmd_info.dma_buffer = crd_tmp_buffer;
	if (!cmd_info.cmd->data) {
		data = kzalloc(sizeof(*data), GFP_KERNEL);
		if (!data) {
			ret_err = -ENOMEM;
			goto err18;
		}
		cmd_info.cmd->data = data;
		data->flags = MMC_DATA_READ;
	} else {
		cmd_info.cmd->data->flags = MMC_DATA_READ;
	}
	ret_err = SD_Stream_Cmd(EMMC_AUTOREAD1, &cmd_info, 1);
	if (ret_err) {
		host_card_stop(sdport);
		polling_to_tran_state(sdport, 1);
		sts1_val = cr_readb(iobase + SD_STATUS1);
		if ((sts1_val & WRT_ERR_BIT) || (sts1_val & CRC16_STATUS) ||
		    (sts1_val & CRC7_STATUS) || (sts1_val & CRC_TIMEOUT_ERR))
			rtksd_stop_transmission(host->card, 1);
	}
err18:
	kfree(cmd);
	cmd_info.cmd = NULL;
	cmd = NULL;
	kfree(data);
	data = NULL;
	return ret_err;
}

int rtkcr_send_cmd25(struct rtkemmc_host *sdport)
{
	int ret_err = 0, i = 0;
	struct sd_cmd_pkt cmd_info;
	int sts1_val = 0;
	char *crd_tmp_buffer = NULL;
	u32 iobase = sdport->base_io;
	struct mmc_data *data = NULL;
	struct mmc_command *cmd = NULL;

	memset(&cmd_info, 0x00, sizeof(struct sd_cmd_pkt));

	crd_tmp_buffer = rtksd_get_buffer_start_addr(sdport);
	if (!crd_tmp_buffer) {
		pr_err("%s,%s : crd_ext_csd == NULL\n", DRIVER_NAME, __func__);
		return -5;
	}

	for (i = 0; i < 0x400; i++)
		crd_tmp_buffer[i] = i;

	if (!cmd_info.cmd) {
		cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
		if (!cmd) {
			ret_err = -ENOMEM;
			goto err25;
		}
		cmd_info.cmd = cmd;
	}
	cmd_info.sdport = sdport;
	cmd_info.cmd->arg = 0xfe;
	cmd_info.cmd->opcode = MMC_WRITE_MULTIPLE_BLOCK;
	cmd_info.rsp_para1	  = -1;
	cmd_info.rsp_para2	  = SD_R1;
	cmd_info.rsp_para3	  = -1;
	cmd_info.rsp_len	 = rtksd_get_rsp_len(SD_R1);
	cmd_info.byte_count  = 0x200;
	cmd_info.block_count = 2;
	cmd_info.dma_buffer = crd_tmp_buffer;
	if (!cmd_info.cmd->data) {
		data = kzalloc(sizeof(*data), GFP_KERNEL);
		if (!data) {
			ret_err = -ENOMEM;
			goto err25;
		}
		cmd_info.cmd->data = data;
		data->flags = MMC_DATA_WRITE;
	} else {
		cmd_info.cmd->data->flags = MMC_DATA_WRITE;
	}
	ret_err = SD_Stream_Cmd(EMMC_AUTOWRITE1, &cmd_info, 1);
	if (ret_err && !(sdport->tuning && sdport->boot_mode == MODE_SD20)) {
		host_card_stop(sdport);
		polling_to_tran_state(sdport, 1);
		sts1_val = cr_readb(iobase + SD_STATUS1);
		sync();
		if ((sts1_val & WRT_ERR_BIT) || (sts1_val & CRC16_STATUS) || (sts1_val & CRC7_STATUS))
			host_card_stop(sdport);
	}
err25:
	if (cmd) {
		cmd_info.cmd = NULL;
		kfree(cmd);
		cmd = NULL;
	}
	kfree(data);
	data = NULL;
	sync();
	return ret_err;
}

void host_card_stop(struct rtkemmc_host *sdport)
{
	mmc_CRT_reset(sdport);
	sync();
}

static int rtksd_stop_transmission(struct mmc_card *card, int ignore)
{
	struct mmc_command cmd;
	struct sd_cmd_pkt cmd_info;
	int err;

	/*
	 * The core only publishes mmc_host.card once the card is fully
	 * initialised, so these helpers can be reached with no card while
	 * tuning. There is nothing to ask in that case.
	 */
	if (!card)
		return -ENODEV;

	set_cmd_info(card, &cmd, &cmd_info,
		     MMC_STOP_TRANSMISSION,
		 0x00,
		 SD_R1 | CRC16_CAL_DIS);
	err = SD_SendCMDGetRSP_Cmd(&cmd_info, ignore);

	return err;
}

static void set_cmd_info(struct mmc_card *card, struct mmc_command *cmd,
			 struct sd_cmd_pkt *cmd_info, u32 opcode, u32 arg, u8 rsp_para)
{
	memset(cmd, 0, sizeof(struct mmc_command));
	memset(cmd_info, 0, sizeof(struct sd_cmd_pkt));

	cmd->opcode         = opcode;
	cmd->arg            = arg;
	cmd_info->cmd       = cmd;
	cmd_info->sdport    = mmc_priv(card->host);
	cmd_info->rsp_para2  = rsp_para;
	cmd_info->rsp_len   = rtksd_get_rsp_len(rsp_para);
}

static struct rtk_host_ops emmc_ops = {
	.chk_card_insert = rtkemmc_chk_card_insert,
	.set_crt_muxpad = rtkemmc_set_crt_muxpad,
	.bus_speed_down = rtkemmc_bus_speed_down,
	.chk_cmdcode    = rtkcr_chk_cmdcode,
	.chk_r1_type    = rtkcr_chk_r1_type,
	.backup_regs    = rtkemmc_backup_registers,
	.restore_regs   = rtkemmc_restore_registers,
};

static ssize_t
em_open_log_show(struct device *dev, struct device_attribute *attr,
		 char *buf)
{
	struct mmc_host *host = dev_get_drvdata(dev);
	struct rtkemmc_host *sdport = mmc_priv(host);

	if (sdport->rtflags & RTKCR_FOPEN_LOG)
		sdport->rtflags &= ~RTKCR_FOPEN_LOG;
	else
		sdport->rtflags |=  RTKCR_FOPEN_LOG;

	return sprintf(buf, "%s log %s\n",
	    DRIVER_NAME,
	    (sdport->rtflags & RTKCR_FOPEN_LOG) ? "open" : "close");
}

static ssize_t
em_open_log_store(struct device *dev,
		  struct device_attribute *attr,
		     const char *buf,
		     size_t count)
{
	pr_debug("%s(%u)Not thing to do.\n", __func__, __LINE__);

	return count;
}
DEVICE_ATTR_RW(em_open_log);

static ssize_t
emmc_id_show(struct device *dev, struct device_attribute *attr,
	     char *buf)
{
	struct mmc_host *host = dev_get_drvdata(dev);
	struct mmc_card *card = host->card;
	struct rtkemmc_host *sdport = mmc_priv(host);

	pr_info("%s(%u)\n", __func__, __LINE__);
	return sprintf(buf, "emmcid=0x%02x%02x\n"
		    "cfg1=0x%02x,cfg2=0x%02x,cfg3=0x%02x,sts1=0x%02x,sts2=0x%02x,bus_sts=0x%02x\n"
		    "sample_pnt=0x%02x,push_pnt=0x%02x,trans=0x%02x,pad_ctl=0x%02x,ckgen_ctl=0x%02x\n"
		    "CARD_SELECT=0x%02x,SYS_PLL_EMMC3=0x%08x,PLL_EMMC1=0x%08x\n",
		    (unsigned char)(card->cid.manfid),
		    (unsigned char)(card->cid.oemid), cr_readb(sdport->base_io + SD_CONFIGURE1),
		    cr_readb(sdport->base_io + SD_CONFIGURE2), cr_readb(sdport->base_io + SD_CONFIGURE3),
		    cr_readb(sdport->base_io + SD_STATUS1), cr_readb(sdport->base_io + SD_STATUS2),
		    cr_readb(sdport->base_io + SD_BUS_STATUS), cr_readb(sdport->base_io + SD_TRANSFER),
		    cr_readb(sdport->base_io + SD_SAMPLE_POINT_CTL),
		    cr_readb(sdport->base_io + SD_PUSH_POINT_CTL),
		    cr_readl(sdport->base_io + EMMC_PAD_CTL), cr_readl(sdport->base_io + EMMC_CKGEN_CTL),
		    cr_readb(sdport->base_io + CARD_SELECT), cr_readl(SYS_PLL_EMMC3), cr_readl(PLL_EMMC1));
}

static ssize_t
emmc_id_store(struct device *dev,
	      struct device_attribute *attr,
		     const char *buf,
		     size_t count)
{
	pr_err("%s(%u)Nothing to do\n", __func__, __LINE__);

	/*
	 * return value must be equare or big then "count"
	 * to finish this attribute
	 */
	return count;
}
DEVICE_ATTR_RW(emmc_id);

static const struct of_device_id rtk_rtkemmc_ids[] = {
	{ .compatible = "realtek,rtd1195-emmc" },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, rtk_rtkemmc_ids);

static int rtkemmc_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc = NULL;
	struct rtkemmc_host *sdport = NULL;
	struct resource *r;
	struct reset_control *rstc_emmc;
	struct clk *clk_en_emmc;
	struct clk *clk_en_emmc_ip;
	struct clk *clk_cr;
	int ret, irq;
	int att_err;
	const u32 *prop;
	int size, speed_step = 0;
	struct device_node *rtk119x_emmc_node = NULL;

	rtk119x_emmc_node = pdev->dev.of_node;

	if (!rtk119x_emmc_node) {
		pr_err("%s : No emmc of_node found\n", DRIVER_NAME);
	} else {
		/*
		 * Diagnostic-only sysfs attribute: warn but keep probing.
		 */
		att_err = device_create_file(&pdev->dev, &dev_attr_em_open_log);
	}
	if (att_err)
		pr_warn("%s : cannot create em_open_log sysfs attr (%d)\n", DRIVER_NAME, att_err);
	att_err = device_create_file(&pdev->dev, &dev_attr_emmc_id);
	if (att_err)
		pr_warn("%s : cannot create emmc_id sysfs attr (%d)\n", DRIVER_NAME, att_err);
	/* Request IRQ */
	irq = irq_of_parse_and_map(rtk119x_emmc_node, 0);
	if (irq <= 0) {
		pr_err("%s : fail to parse of irq.\n", DRIVER_NAME);
		return -ENXIO;
	}
	ret = platform_device_add_resources(pdev, rtkemmc_resources, 2);
	if (ret) {
		pr_err("%s : fail to add resources.\n", DRIVER_NAME);
		return -ENXIO;
	}
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!r || irq < 0) {
		pr_err("%s : fail to get resources or irq\n", DRIVER_NAME);
		return -ENXIO;
	}

	r = request_mem_region(r->start, 0x3FF, DRIVER_NAME);
	if (!r) {
		pr_err("%s : fail to request mem region\n", DRIVER_NAME);
		return -EBUSY;
	}

	mmc = mmc_alloc_host(sizeof(struct rtkemmc_host), &pdev->dev);

	if (!mmc) {
		ret = -ENOMEM;
		goto out;
	}

	sdport = mmc_priv(mmc);
	memset(sdport, 0, sizeof(struct rtkemmc_host));

	sdport->mmc = mmc;
	sdport->dev = &pdev->dev;
	sdport->res = r;
	sdport->base_io = EM_BASE_ADDR;
	sdport->ops = &emmc_ops;

	sema_init(&sdport->sem, 1);
	sema_init(&sdport->sem_op_end, 1);

	prop = of_get_property(pdev->dev.of_node, "speed-step", &size);
	if (prop) {
		speed_step = of_read_number(prop, 1);
		} else {
		pr_err("[%s] no speed-step property, defaulting to %d\n", __func__, speed_step);
	}

	rstc_emmc = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(rstc_emmc)) {
		pr_err("%s: reset_control_get() returns %ld\n", __func__, PTR_ERR(rstc_emmc));
		rstc_emmc = NULL;
	}
	clk_en_emmc = devm_clk_get(&pdev->dev, "emmc");
	if (IS_ERR(clk_en_emmc)) {
		pr_err("%s: clk_get() returns %ld\n", __func__, PTR_ERR(clk_en_emmc));
		clk_en_emmc = NULL;
	}
	clk_en_emmc_ip = devm_clk_get(&pdev->dev, "emmc_ip");
	if (IS_ERR(clk_en_emmc_ip)) {
		pr_err("%s: clk_get() returns %ld\n", __func__, PTR_ERR(clk_en_emmc_ip));
		clk_en_emmc_ip = NULL;
	}
	/*
	 * 1195 uses the same DMA bus between SD, SDIO, and eMMC, so this
	 * clock still needs to be opened even without an SD/SDIO driver;
	 * 1395 separates the DMA bus.
	 */
	clk_cr = devm_clk_get(&pdev->dev, "cr");
	if (IS_ERR(clk_cr)) {
		pr_err("%s: clk_get() returns %ld\n", __func__, PTR_ERR(clk_cr));
		clk_cr = NULL;
	}

	clk_prepare_enable(clk_en_emmc);
	clk_prepare_enable(clk_en_emmc_ip);
	clk_prepare_enable(clk_cr);
	reset_control_deassert(rstc_emmc);

	mmc->ocr_avail = MMC_VDD_30_31
			| MMC_VDD_31_32
			| MMC_VDD_32_33
			| MMC_VDD_33_34
			| MMC_VDD_165_195;

	mmc->caps = MMC_CAP_4_BIT_DATA
		| MMC_CAP_8_BIT_DATA
		| MMC_CAP_SD_HIGHSPEED
		| MMC_CAP_MMC_HIGHSPEED
		| MMC_CAP_NONREMOVABLE;

	mmc->caps2 = MMC_CAP2_NO_SDIO | MMC_CAP2_NO_SD | MMC_CAP2_HS200_1_8V_SDR;

	switch (speed_step) {
	case 0: //sdr50
		mmc->caps &= ~(MMC_CAP_UHS_DDR50 | MMC_CAP_1_8V_DDR);
		mmc->caps2 &= ~(MMC_CAP2_HS200_1_8V_SDR);
		break;
	case 1: //ddr50
		mmc->caps2 &= ~(MMC_CAP2_HS200_1_8V_SDR);
		break;
	}

	if (rtk_emmc_bus_wid == 4 || rtk_emmc_bus_wid == 5)
		mmc->caps &= ~MMC_CAP_8_BIT_DATA;

	mmc->f_min = 10000000 >> 8;   /* RTK min bus clk is 10Mhz/256 */
	mmc->f_max = 48000000;      /* RTK max bus clk is 48Mhz */

	mmc->max_segs = 1;
	mmc->max_blk_size   = 512;

	mmc->max_blk_count  = 0x400;

	mmc->max_seg_size   = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_req_size   = mmc->max_blk_size * mmc->max_blk_count;

	spin_lock_init(&sdport->lock);
	init_rwsem(&sdport->rw_sem);
	tasklet_init(&sdport->req_end_tasklet, rtkcr_req_end_tasklet,
		     (unsigned long)sdport);

	sdport->base = ioremap(r->start, 0x00000200);

	if (rtk_emmc_bus_wid == 9 || rtk_emmc_bus_wid == 5)
		sdport->rtflags |= RTKCR_FOPEN_LOG;

	if (!sdport->base) {
		pr_info("---- Realtek EMMC Controller Driver probe fail - nomem ----\n\n");
		ret = -ENOMEM;
		goto out;
	}
	rtkcr_hold_int_dec(sdport->base_io);       /* hold status interrupt */
	rtkcr_clr_int_sta(sdport->base_io);

	ret = request_irq(irq, rtksd_irq, IRQF_SHARED, DRIVER_NAME, sdport);   //rtkcr_interrupt
	if (ret) {
		pr_err("%s: cannot assign irq %d\n", DRIVER_NAME, irq);
		goto out;
	} else {
		sdport->irq = irq;
	}
	timer_setup(&sdport->timer, rtksd_timeout_timer, 0);

	sdport->ops->set_crt_muxpad(sdport);

	sdport->ops->chk_card_insert(sdport);

	platform_set_drvdata(pdev, mmc);
	ret = mmc_add_host(mmc);
	if (ret)
		goto out;

	cr_writeb(0x2, sdport->base_io + CARD_SELECT);            //for emmc, select SD ip
	rtkcr_set_pad_driving(sdport, MMC_IOS_GET_PAD_DRV, 0x66, 0x64, 0x66);
	sync();
	memset((void *)sdport->last_cmd, 0, 6);
	memset((void *)&sdport->reg_backup, 0, sizeof(sdport->reg_backup));
	sdport->boot_mode = MODE_SD20;
	sdport->g_crinit = 0;
	sdport->prevent_retry = 0;
	sdport->resuming = 0;
	sdport->tuning = 0;

	pr_info("%s: %s driver initialized\n",
		mmc_hostname(mmc), DRIVER_NAME);

	return 0;

out:
	if (sdport) {
		if (sdport->irq)
			free_irq(sdport->irq, sdport);

		if (sdport->base)
			iounmap(sdport->base);
	}
	if (r)
		release_resource(r);
	if (mmc)
		mmc_free_host(mmc);
	return ret;
}

static void rtksd_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_em_open_log);
	device_remove_file(&pdev->dev, &dev_attr_emmc_id);

	if (mmc) {
		struct rtkemmc_host *sdport = mmc_priv(mmc);

		rtksd_free_dma_buf(sdport);

		mmc_remove_host(mmc);
		pr_info("eMMC host have removed.\n");
		free_irq(sdport->irq, sdport);

		timer_delete_sync(&sdport->timer);
		iounmap(sdport->base);

		release_resource(sdport->res);
		mmc_free_host(mmc);
	}
	platform_set_drvdata(pdev, NULL);
}

#ifdef CONFIG_PM
static int rtksd_suspend(struct device *dev)
{
	int ret = 0;

	//For suspend mode
	pr_err("[%s] Enter %s Suspend mode\n", DRIVER_NAME, __func__);

	ret = pm_runtime_force_suspend(dev);
	pr_err("[%s] Exit %s\n", DRIVER_NAME, __func__);

	return ret;
}

static int rtksd_resume(struct device *dev)
{
	int ret = 0;
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct rtkemmc_host *sdport;

	/*
	 * Guard the host pointer itself, not mmc_priv()'s result: private[] is a
	 * flexible array member, so mmc_priv() is pure pointer arithmetic that
	 * never returns NULL -- the old `if (!sdport) BUG()` could not fire even
	 * when the drvdata was NULL, and the sdport->mmc deref below then
	 * faulted on a near-NULL address.
	 */
	if (!mmc) {
		pr_err("[%s] %s: no eMMC host registered\n", DRIVER_NAME, __func__);
		return -ENODEV;
	}
	sdport = mmc_priv(mmc);

	/* sdport->mmc is this same host; re-link only if a card is attached. */
	if (mmc->card)
		mmc->card->host = mmc;
	sdport->resuming = 1;

	pr_err("[%s] Enter %s Resume mode\n", DRIVER_NAME, __func__);

	if (!ret)
		ret = pm_runtime_force_resume(dev);

	sdport->ops->set_crt_muxpad(sdport);
	rtkemmc_chk_card_insert(sdport);
	sync();

	sdport->resuming = 0;
	/* int_waiting is only non-NULL while a command is in flight. */
	if (sdport->int_waiting)
		init_completion(sdport->int_waiting);
	pr_err("[%s] Exit %s\n", DRIVER_NAME, __func__);

	return ret;
}

static const struct dev_pm_ops rtk_dev_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(rtksd_suspend, rtksd_resume)
};
#endif

/*****************************************************************************************/
/* driver / device attach area */
/*****************************************************************************************/

static struct platform_driver rtkemmc_driver = {
	.probe      = rtkemmc_probe,
	.remove     = rtksd_remove,
	.driver = {
	    .name   = "rtkemmc",
	    .owner  = THIS_MODULE,
	    .of_match_table = rtk_rtkemmc_ids,
#ifdef CONFIG_PM
		.pm     = &rtk_dev_pm_ops
#endif
	},
};

static void rtkcr_display_version(void)
{
	pr_info(BANNER " %s\n", VERSION);

#ifdef CONFIG_SMP
#else
	pr_info("%s: ##### CONFIG_SMP disable!! #####\n", DRIVER_NAME);
#endif
}

static int rtkemmc_set_bus_width(char *buf)
{
	/*
	 * get eMMC bus width setting by bootcode parameter, like below
	 * bootargs=console=ttyS0,115200 earlyprintk emmc_bus=8
	 * the keyword is "emmc_bus"
	 * the getted parameter is hex.
	 * example:
	 * emmc_bus=8
	 */

	rtkcr_chk_param(&rtk_emmc_bus_wid, 1, buf + 1);
	pr_info("%s: setting bus width is %u-bit\n",
		DRIVER_NAME, rtk_emmc_bus_wid);
	return 0;
}

static int __init rtkemmc_init(void)
{
	int rc = 0;

	rtkemmc_rbus_base = ioremap(0x18000000, 0x00070000);
	if (!rtkemmc_rbus_base) {
		pr_err("%s : failed to map RBUS\n", DRIVER_NAME);
		return -ENOMEM;
	}

	rtkcr_display_version();

	rc = platform_driver_register(&rtkemmc_driver);

	if (rc < 0) {
		pr_info("Realtek EMMC Controller Driver installation fails.\n\n");
		return -ENODEV;
	}
	return 0;

	return rc;
}

static void __exit rtkemmc_exit(void)
{
	platform_driver_unregister(&rtkemmc_driver);
}

// allow emmc driver initialization earlier
module_init(rtkemmc_init);
module_exit(rtkemmc_exit);

/* maximum card clock frequency (default 50MHz) */
module_param(maxfreq, int, 0);

/* force PIO transfers all the time */
module_param(nodma, int, 0);

MODULE_AUTHOR("Elbereth");
MODULE_DESCRIPTION("Realtek EMMC Host Controller driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtkemmc");

__setup("emmc_bus", rtkemmc_set_bus_width);
