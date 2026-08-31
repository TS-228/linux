/* SPDX-License-Identifier: GPL-2.0-only */
/* Realtek RTD119x eMMC driver -- Copyright (C) 2010 Realtek Semiconductors, All Rights Reserved. */

#ifndef __RTKEMMC_H
#define __RTKEMMC_H

#include <soc/realtek/rtk_chip.h>

/*
 * Driver-private command error codes. These used to live in
 * include/linux/mmc/core.h; they are not generic MMC constants.
 * rtk-sdmmc.h carries its own copy of the same list.
 */
#define MMC_ERR_NONE        0
#define MMC_ERR_TIMEOUT     1
#define MMC_ERR_BADCRC      2
#define MMC_ERR_RMOVE       3
#define MMC_ERR_FAILED      4
#define MMC_ERR_INVALID     5

#include "reg_mmc_rtd119x.h"

/* cmd1 sector mode */
#define MMC_SECTOR_ADDR         0x40000000
/*
 * Clock rates
 */
#define RTKSD_CLOCKRATE_MAX			48000000
#define RTKSD_BASE_DIV_MAX			0x100

/*
 * pad driving
 */
#define MMC_IOS_GET_PAD_DRV     0x1
#define MMC_IOS_SET_PAD_DRV     0x2
#define MMC_IOS_RESTORE_PAD_DRV 0x4

/*
 * Register access base, dynamically ioremap'd (set once in
 * rtkemmc_init(), well before any probe/register access happens) - no
 * static mapping table exists under mainline's machine descriptor.
 */
extern void __iomem *rtkemmc_rbus_base;
#define RTKEMMC_REG(offset)      (rtkemmc_rbus_base + ((offset) - 0x18000000))

/*
 * Chip ID/revision, exported from arch/arm/mach-realtek/rtd1195.c
 * (mach-rtd119x/include/mach/cpu.h's realtek_cpu_id/realtek_rev()
 * equivalent - that header no longer exists once this board switches
 * off the vendor machine descriptor).
 */

#define cr_readb(offset)        (*(volatile unsigned char *)RTKEMMC_REG(offset))
#define cr_writeb(val, offset)  do {							\
				(*(volatile unsigned char *)RTKEMMC_REG(offset)) = val;           \
				asm ("DMB");						\
				} while (0)
#define cr_readl(offset)        (*(volatile unsigned int *)RTKEMMC_REG(offset))
#define cr_writel(val, offset)  do {							\
				(*(volatile unsigned int *)RTKEMMC_REG(offset)) = val;            \
				asm ("DMB");						\
				} while (0)

#define CLEAR_CR_CARD_STATUS(reg_addr)      \
	cr_writel(0xffffffff, reg_addr)

#define CLEAR_ALL_CR_CARD_STATUS(io_base)        \
do {                                \
	int i = 0;  \
	for (i = 0; i < 5; i++)  \
		CLEAR_CR_CARD_STATUS((io_base) + 20 + (i * 4));  \
} while (0)

/*  ===== macros and funcitons for Realtek CR ===== */
/* for SD/MMC usage */
#define ON                      0
#define OFF                     1
#define GPIO_OUT    1
#define GPIO_IN     0
#define GPIO_HIGH   1
#define GPIO_LOW    0

/* MMC configure1, for SD_CONFIGURE1 */
#define SD30_FIFO_RST               BIT(4)
#define SD1_R0                      (SD30_FIFO_RST)

/* MMC configure3 , for SD_CONFIGURE3 */
#define SD30_CLK_STOP               BIT(4)
#define SD2_R0                      (RESP_CHK_EN | ADDR_BYTE_MODE)

/* MMC configure2, for SD_CONFIGURE2, response type */
#define SD_R0                   (RESP_TYPE_NON | CRC7_CHK_DIS)
#define SD_R1                   (RESP_TYPE_6B)
#define SD_R1b                  (RESP_TYPE_6B | WAIT_BUSY_EN)
#define SD_R2                   (RESP_TYPE_17B)
#define SD_R3                   (RESP_TYPE_6B)
#define SD_R4                   (RESP_TYPE_6B)
#define SD_R5                   (RESP_TYPE_6B)
#define SD_R6                   (RESP_TYPE_6B)
#define SD_R7                   (RESP_TYPE_6B)
#define SD_R_NO                 (0xFF)

/* rtflags */
#define RTKCR_FCARD_DETECTED    BIT(0)      /* Card is detected */
#define RTKCR_FOPEN_LOG         BIT(1)      /* open command log */
#define RTKCR_USER_PARTITION    BIT(2)      /* card is working on normal partition */

#define RTKCR_FCARD_POWER       BIT(4)      /* Card is power on */
#define RTKCR_FHOST_POWER       BIT(5)      /* Host is power on */

struct rtk_emmc_reg_backup {
	u32			emmc_pad_ctl;
	u32			emmc_ckgen_ctl;
	u32			sys_pll_emmc3;
	u32			pll_emmc1;
	u8			card_select;
	u8			sample_point_ctl;
	u8			push_point_ctl;
	u8			sd_configure1;
	u8			sd_configure2;
	u8			sd_configure3;
};

struct rtkemmc_host {
	struct mmc_host     *mmc;           /* MMC structure */
	u32                 rtflags;        /* Driver states */
	u8                  ins_event;
	u8                  cmd_opcode;

#define EVENT_NON		    0x00
#define EVENT_INSER		    0x01
#define EVENT_REMOV		    0x02
#define EVENT_USER		    0x10

	u8                  reset_event;
	struct mmc_request  *mrq;            /* Current request */
	u8                  wp;
	struct rtk_host_ops *ops;
	struct semaphore	sem;
	struct semaphore	sem_op_end;

	void __iomem        *base;
	u32 base_io;
	spinlock_t          lock;
	unsigned int        ns_per_clk;
	struct delayed_work cmd_work;
	struct tasklet_struct req_end_tasklet;

	struct timer_list   timer;
	struct timer_list   plug_timer;
	struct completion   *int_waiting;
	struct device       *dev;
	struct resource     *res;
	int                 irq;
	u8                  *tmp_buf;
	dma_addr_t          tmp_buf_phy_addr;
	dma_addr_t          paddr;
	u32                 test_count;
	u32                 int_status;
	u32                 int_status_old;
	u32                 sd_status1;
	u32                 sd_status2;
	u32                 bus_status;
	u32                 dma_trans;
	u32                 sd_trans;
	u32                 gpio_isr_info;
	u32                 tmout;

	unsigned int        boot_mode;
	unsigned int        g_crinit;
	int                 tuning;
	int                 resuming;
	int                 prevent_retry;
	int                 send_cmd0;
	u8                  last_cmd[6];
	struct rtk_emmc_reg_backup reg_backup;
	struct rw_semaphore rw_sem;
	unsigned char       *rsp_buf;
	unsigned char       *rsp_buf_org;
};

struct rtk_host_ops {
	void (*chk_card_insert)(struct rtkemmc_host *rtkhost);
	void (*set_crt_muxpad)(struct rtkemmc_host *rtkhost);
	void (*bus_speed_down)(struct rtkemmc_host *sdport);
	u32 (*chk_cmdcode)(struct mmc_command *cmd);
	u32 (*chk_r1_type)(struct mmc_command *cmd);
	u32 (*backup_regs)(struct rtkemmc_host *sdport);
	u32 (*restore_regs)(struct rtkemmc_host *sdport);
};

struct sd_cmd_pkt {
	struct mmc_host     *mmc;       /* MMC structure */
	struct rtkemmc_host   *sdport;
	struct mmc_command  *cmd;    /* cmd->opcode; cmd->arg; cmd->resp; cmd->data */
	struct mmc_data     *data;
	unsigned char       *dma_buffer;
	u16                 byte_count;
	u16                 block_count;

	u32                 flags;
	s8                  rsp_para1;
	s8                  rsp_para2;
	s8                  rsp_para3;
	u8                  rsp_len;
	u32                 timeout;
};

/* Magellan_ISO_arch_spec.doc; PMM_Magellan_PinMux.doc */
// iso gpio pinmux map
// iso gpio pinmux bit operation map
#define MAX_CMD_RETRY_COUNT 4
#define MAX_ISO_GPIO_CNT 35
#define MAX_MIS_GPIO_CNT 169
#define CARD_SWITCHCLOCK_25MHZ_B    (0x00UL)    //(0x05)
#define CARD_SWITCHCLOCK_33MHZ_B    (0x01UL)    //(0x06)
#define CARD_SWITCHCLOCK_40MHZ_B    (0x02UL)    //(0x00)
#define CARD_SWITCHCLOCK_50MHZ_B    (0x03UL)    //(0x01)
#define CARD_SWITCHCLOCK_66MHZ_B    (0x04UL)    //(0x02)
#define CARD_SWITCHCLOCK_98MH_B     (0x05UL)    //(0x03)
#define CLOCK_SPEED_GAP          CARD_SWITCHCLOCK_50MHZ_B
#define LOW_SPEED_LMT               CARD_SWITCHCLOCK_33MHZ_B

#define CARD_SWITCHCLOCK_60MHZ  (0x00UL)      //(0x00<<12)
#define CARD_SWITCHCLOCK_80MHZ  (0x01UL)      //(0x01<<12)
#define CARD_SWITCHCLOCK_98MHZ  (0x02UL)      //(0x02<<12)
#define CARD_SWITCHCLOCK_98MHZS (0x03UL)      //(0x03<<12)
#define CARD_SWITCHCLOCK_30MHZ  (0x04UL)      //(0x04<<12)
#define CARD_SWITCHCLOCK_40MHZ  (0x05UL)      //(0x05<<12)
#define CARD_SWITCHCLOCK_49MHZ  (0x06UL)      //(0x06<<12)
#define CARD_SWITCHCLOCK_49MHZS (0x07UL)      //(0x07<<12)

/* CRT_SYS_CLKSEL setting bit &&& */

/* CRT_SYS_SRST2 setting bit *** */
#define RSTN_CR (0x01 << 23)      //for eMMC
/* CRT_SYS_SRST2 setting bit &&& */

/* CRT_SYS_SRST3 setting bit *** */
#define RSTN_SD (0x01 << 16)      //for card reader
/* CRT_SYS_SRST3 setting bit &&& */

/* CRT_SYS_CLKEN2 setting bit *** */
#define CLKEN_CR (0x01 << 23)      //for eMMC
/* CRT_SYS_CLKEN2 setting bit &&& */

/* CRT_SYS_CLKEN3 setting bit *** */
#define CLKEN_SD (0x01 << 16)      //for card reader
/* CRT_SYS_CLKEN3 setting bit &&& */

#define CR_PINMUX_CR_MASK       0xFFFFC000

#define MUX_SD_DAT2             (0x01 << 26)
#define MUX_MS_CLK              (0x02 << 26)
#define MUX_SD_DAT1             (0x01 << 24)
#define MUX_MS_BS               (0x02 << 24)
#define MUX_SD_DT               (0x01 << 20)
#define MUX_OTP_ATE_FAIL        (0x02 << 20)
#define MUX_SD_WP               (0x01 << 18)
#define MUX_MS_DT               (0x02 << 18)
#define MUX_SD_CLK              (0x01 << 16)
#define MUX_MS_DAT0             (0x02 << 16)
#define MUX_SD_CMD              (0x01 << 14)
#define MUX_MS_DAT2             (0x02 << 14)

#define MUX_PAD5_DEFAULT         0x15554000

#define CR_PMUX_CMD_MASK        ~(0x03 << 14)  //mask 15,14
#define CR_PMUX_CLK_MASK        ~(0x03 << 16)  //mask 16,17
#define CR_PMUX_MOD_MASK        ~(0x0f << 18)  //mask 18,19,20,21
#define DETECT_MODE             MUX_SD_DT

#define MUX_PAD5_SD_MODE        0x15554000

#define RCA_SHIFTER             16
#define NORMAL_READ_BUF_SIZE    512      //no matter FPGA & QA

#define SD_CARD_WP              BIT(5)
#define SD_CARD_EXIST           BIT(2)

/* ======================================================= */
// see "Main_Magellan_PinMux.doc"
#define PINMUX_REG_BASE		0xb8000800
#define GPIO_MUXCFG_0		(PINMUX_REG_BASE + 0x00)    //0xb8000800
#define GPIO_MUXCFG_1		(PINMUX_REG_BASE + 0x04)    //0xb8000804
#define GPIO_MUXCFG_2		(PINMUX_REG_BASE + 0x08)    //0xb8000808
#define GPIO_MUXCFG_3		(PINMUX_REG_BASE + 0x0c)    //0xb800080C
#define GPIO_MUXCFG_4		(PINMUX_REG_BASE + 0x10)    //0xb8000810
#define GPIO_MUXCFG_5		(PINMUX_REG_BASE + 0x14)    //0xb8000814
#define GPIO_MUXCFG_6		(PINMUX_REG_BASE + 0x18)    //0xb8000818

#define GPIO_MUXCFG_13		(PINMUX_REG_BASE + 0x34)    //0xb8000834
#define GPIO_MUXCFG_14		(PINMUX_REG_BASE + 0x38)    //0xb8000838
#define GPIO_MUXCFG_15		(PINMUX_REG_BASE + 0x3C)    //0xb800083C
#define GPIO_MUXCFG_16		(PINMUX_REG_BASE + 0x40)    //0xb8000840
#define GPIO_MUXCFG_17		(PINMUX_REG_BASE + 0x44)    //0xb8000844
#define GPIO_MUXCFG_18		(PINMUX_REG_BASE + 0x48)    //0xb8000848

/*======================================================== */

/* move from c file *** */
#define BYTE_CNT            0x200
#define RTK_NORMAL_SPEED    0x00
#define RTK_HIGH_SPEED      0x01
#define RTK_1_BITS          0x00
#define RTK_4_BITS          0x10
#define RTK_BITS_MASK       0x30
#define RTK_SPEED_MASK      0x01
#define RTK_PHASE_MASK      0x06

#define R_W_CMD             2   //read/write command
#define INN_CMD             1   //command work chip inside
#define UIN_CMD             0   //no interrupt rtk command

#define RTK_FAIL            3  /* DMA error & cmd parser error */
#define RTK_RMOV            2  /* card removed */
#define RTK_TOUT            1  /* time out include DMA finish & cmd parser finish */
#define RTK_SUCC            0

#define CR_TRANS_OK         0x0
#define CR_TRANSFER_TO      0x1
#define CR_BUF_FULL_TO      0x2
#define CR_DMA_FAIL         0x3
#define CR_TRANSFER_FAIL    0x4
#define CR_TRANSFER_IGN     0xf

/* send status event */
#define STATE_IDLE          0
#define STATE_READY         1
#define STATE_IDENT         2
#define STATE_STBY          3
#define STATE_TRAN          4
#define STATE_DATA          5
#define STATE_RCV           6
#define STATE_PRG           7
#define STATE_DIS           8

#define GPIO_HI             0x78
#define GPIO_LO             0x87

#define LVL_HI             0x01
#define LVL_LO             0x00

#define POW_CHECK 0
#define POW_FORCE 1

#define rtkcr_get_int_sta(io_base, status_addr)                           \
	do {                                                         \
		if ((io_base) == CR_BASE_ADDR)                             \
		    *(u32 *)status_addr = cr_readl((io_base) + CR_SD_ISR);   \
		else                                                     \
		    *(u32 *)status_addr = cr_readl((io_base) + EMMC_SD_ISR); \
	} while (0)

#define rtkcr_get_sd_sta(io_base, status_addr1, status_addr2, bus_status)       \
	do {                                                             \
		    *(u32 *)status_addr1 = cr_readb((io_base) + SD_STATUS1);     \
		    *(u32 *)status_addr2 = cr_readb((io_base) + SD_STATUS2);     \
		    *(u32 *)bus_status = cr_readb((io_base) + SD_BUS_STATUS);    \
	} while (0)

#define rtkcr_get_sd_trans(io_base, status_addr) \
	(*(u32 *)status_addr = cr_readb((io_base) + SD_TRANSFER))
#define rtkcr_get_dma_trans(io_base, status_addr) \
	(*(u32 *)status_addr = cr_readb((io_base) + CR_DMA_CTL3))

static inline void rtkcr_clr_int_sta(u32 io_base)
{
	if (io_base == CR_BASE_ADDR)
		cr_writel((cr_readl(io_base + CR_SD_ISR) | (ISRSTA_INT1 | ISRSTA_INT2)) |
			  CLR_WRITE_DATA, io_base + CR_SD_ISR);
	else if (io_base == EM_BASE_ADDR)
		cr_writel((cr_readl(io_base + EMMC_SD_ISR) | (ISRSTA_INT1 | ISRSTA_INT2)) |
			  CLR_WRITE_DATA, io_base + EMMC_SD_ISR);
	else
		cr_writel((cr_readl(io_base + SDIO_SD_ISR) |
			  (SDIO_ISRSTA_INT1 | SDIO_ISRSTA_INT2 | SDIO_ISRSTA_INT3 | SDIO_ISRSTA_INT4)) |
			  CLR_WRITE_DATA, io_base + SDIO_SD_ISR);
}

static inline void rtkcr_hold_int_dec(u32 io_base)
{
	if (io_base == CR_BASE_ADDR)
		cr_writel((cr_readl(io_base + CR_SD_ISREN) | (ISRSTA_INT1EN | ISRSTA_INT2EN)) |
			  CLR_WRITE_DATA, io_base + CR_SD_ISREN);
	else if (io_base == EM_BASE_ADDR)
		cr_writel((cr_readl(io_base + EMMC_SD_ISREN) | (ISRSTA_INT1EN | ISRSTA_INT2EN)) |
			  CLR_WRITE_DATA, io_base + EMMC_SD_ISREN);
	else
		cr_writel((cr_readl(io_base + SDIO_SD_ISREN) | SDIO_ISRSTA_INT1EN) |
			  CLR_WRITE_DATA, io_base + SDIO_SD_ISREN);
}

static inline void rtkcr_en_int(u32 io_base)
{
	if (io_base == CR_BASE_ADDR)
		cr_writel((cr_readl(io_base + CR_SD_ISREN) | (ISRSTA_INT1EN | ISRSTA_INT2EN)) |
			  WRITE_DATA, io_base + CR_SD_ISREN);
	else if (io_base == EM_BASE_ADDR)
		cr_writel((cr_readl(io_base + EMMC_SD_ISREN) | (ISRSTA_INT1EN | ISRSTA_INT2EN)) |
			  WRITE_DATA, io_base + EMMC_SD_ISREN);
	else
		cr_writel((cr_readl(io_base + SDIO_SD_ISREN) | SDIO_ISRSTA_INT1EN) |
			  WRITE_DATA, io_base + SDIO_SD_ISREN);
}

#define  rtkcr_mdelay(x)  \
	do {                                             \
		set_current_state(TASK_INTERRUPTIBLE);  \
		schedule_timeout(msecs_to_jiffies(x));  \
	} while (0)

#define INT_BLOCK_R_GAP 0x200
#define INT_BLOCK_W_GAP 5

static const char *const state_tlb[9] = {
	"STATE_IDLE",
	"STATE_READY",
	"STATE_IDENT",
	"STATE_STBY",
	"STATE_TRAN",
	"STATE_DATA",
	"STATE_RCV",
	"STATE_PRG",
	"STATE_DIS"
};

/* data read cmd */
/* data write cmd */
/* data xfer cmd */
#define card_sta_err_mask ( \
	(1 << 31) | (1 << 30) | (1 << 29) | (1 << 28) | (1 << 27) | (1 << 26) | \
	(1 << 24) | (1 << 23) | (1 << 22) | (1 << 21) | (1 << 20) | (1 << 19) | \
	(1 << 18) | (1 << 17) | (1 << 16) | (1 << 15) | (1 << 13) | (1 << 7))

/* Indexed by SD/MMC command number, 0-63; only ADTC-type commands use this. */
static const unsigned char rtk_sd_cmdcode[64][2] = {
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_NORMALREAD, SD_R1 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_NORMALREAD, SD_R1 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_AUTOREAD1, SD_R1 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_NORMALREAD, SD_R1 },
	{EMMC_NORMALREAD, SD_R1 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_AUTOREAD2, SD_R1 },
	{EMMC_AUTOREAD1, SD_R1 }, {EMMC_NORMALWRITE, SD_R1 },
	{EMMC_AUTOWRITE1, SD_R1 }, {EMMC_TUNING, SD_R0 },
	{EMMC_NORMALREAD, SD_R1 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_AUTOWRITE2, SD_R1 }, {EMMC_AUTOWRITE1, SD_R1 },
	{EMMC_NORMALWRITE, SD_R1 }, {EMMC_NORMALWRITE, SD_R1 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_NORMALREAD, SD_R1 }, {EMMC_NORMALREAD, SD_R1 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_NORMALREAD, SD_R1 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_NORMALREAD, SD_R1 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_AUTOREAD2, SD_R1 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_CMD_UNKNOW, SD_R0 },
	{EMMC_AUTOWRITE2, SD_R1 }, {EMMC_NORMALREAD, SD_R1 },
	{EMMC_CMD_UNKNOW, SD_R0 }, {EMMC_CMD_UNKNOW, SD_R0 }
};

int rtkcr_wait_opt_end(char *drv_name, struct rtkemmc_host *sdport, unsigned char cmdcode,
		       unsigned char cmd_idx, unsigned char cpu_mode);
void rtk_op_complete(struct rtkemmc_host *sdport);
char *rtkcr_parse_token(const char *parsed_string, const char *token);
void rtkcr_chk_param(u32 *pparam, u32 len, u8 *ptr);
int rtkcr_chk_ver_a(void);
void emmc_show_config123(struct rtkemmc_host *sdport);
void rtkcr_set_mis_gpio(u32 gpio_num, u8 dir, u8 level);
void rtkcr_set_iso_gpio(u32 gpio_num, u8 dir, u8 level);

struct completion *rtk_int_enable(struct rtkemmc_host *sdport, unsigned long msec);
int rtk_int_enable_and_waitfor(struct rtkemmc_host *sdport, u8 cmdcode, u8 cmd_dix,
			       unsigned long msec, unsigned long dma_msec);
void rtk_int_waitfor(struct rtkemmc_host *sdport, u8 cmdcode, u8 cmd_idx, unsigned long msec,
		     unsigned long dma_msec);
/* remove from c file &&& */

/* rtk function definition */
int error_handling(struct rtkemmc_host *sdport, unsigned int cmd_idx, unsigned int ignore);
int rtkcr_send_cmd25(struct rtkemmc_host *sdport);
int rtkcr_send_cmd18(struct rtkemmc_host *sdport);
int rtkcr_send_cmd8(struct rtkemmc_host *sdport, unsigned int ignore);
int polling_to_tran_state(struct rtkemmc_host *sdport, int ignore);
void host_card_stop(struct rtkemmc_host *sdport);
static int mmc_select_sdr50_push_sample(struct rtkemmc_host *sdport);

#endif
