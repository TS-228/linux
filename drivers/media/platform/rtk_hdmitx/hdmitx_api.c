/*
 * hdmitx_api.c - RTK hdmitx driver
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/ioctl.h> /* needed for the _IOW etc stuff used later */
#include <linux/syscalls.h> /* needed for the _IOW etc stuff used later */
#include <linux/mpage.h>
#include <linux/dcache.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <sound/asound.h>
#include <asm/cacheflush.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <soc/realtek/power-control.h>
#include <linux/reset.h>
#include <linux/clk.h> /* clk_get */
#include <linux/clk-provider.h>

#include <soc/realtek/kernel-rpc.h>
#include <linux/dma-mapping.h>
#include "hdmitx_rpc.h"
#include <dc2vo/dc_rpc.h>
#include "hdmitx.h"
#include "hdmitx_dev.h"
#include "hdmitx_reg.h"
#include "rtk_edid.h"
#include "crt_reg.h"
#include "hdmitx_scdc.h"


#ifdef USE_ION_AUDIO_HEAP
#include "uapi/ion.h"
#include "ion/ion.h"
#include "uapi/ion_rtk.h"

struct ion_client *rpc_ion_client;
extern struct ion_device *rtk_phoenix_ion_device;
#endif

extern hdmitx_device_t tx_dev;

enum HDMI_AVMUTE {
	HDMI_CLRAVM = 0,
	HDMI_SETAVM
};

enum HDMI_CLK_STATE {
	HDMI_CLK_OFF = 0,
	HDMI_CLK_ON,
	HDMI_CLK_PLL_ON
};

#if defined(CONFIG_ARCH_RTD129x) || defined(CONFIG_ARCH_RTD119X)
#define CONVERT_FOR_AVCPU(x)		((unsigned int)(x) | 0xA0000000)
#else
#define CONVERT_FOR_AVCPU(x)		(x)
#endif

static int hdmi_error;
unsigned int tmds_en = 0;
unsigned int hdmi_clk_always_on = 0;
unsigned int displayport_exist = 0;

void hdmitx_print_sink_info(asoc_hdmi_t *p_this);

int is_hdmi_clock_on(void)
{
	int ret_value;
	void __iomem *vaddr;
	unsigned char pll_hdmi;
	struct clk *clk_hdmitx;
	struct reset_control *reset_hdmitx;

	vaddr = ioremap(0x98000190, 0x4);/* PLL_HDMI */
	pll_hdmi = rd_reg(vaddr)&0xBF;
	iounmap(vaddr);

	clk_hdmitx = tx_dev.clk_hdmi;
	reset_hdmitx = tx_dev.reset_hdmi;

	if (__clk_is_enabled(clk_hdmitx) &&
		(!reset_control_status(reset_hdmitx))) {

		if (pll_hdmi == 0xBF)
			ret_value = HDMI_CLK_PLL_ON;
		else
			ret_value = HDMI_CLK_ON;
	} else {
		ret_value = HDMI_CLK_OFF;
	}

	return ret_value;
}

u32 check_hdmi_mhl_mode(void)
{
	void __iomem *vaddr;
	u32 ret;

	vaddr = ioremap(SYS_DISP_PLL_DIV2_reg, 0x4);

	/* 0: HDMI mode , 1:  MHL mode */
	ret = SYS_DISP_PLL_DIV2_get_sel_pllhdmi_mhl(rd_reg(vaddr));
	pr_info("rtk-hdmitx: " "Mode = %s", ret?"MHL":"HDMI");

	iounmap(vaddr);
	return ret;
}

int hdmitx_check_rx_sense(void __iomem *reg_base)
{
	u32 val;

	val = HDMI_PHY_STATUS_get_Rxstatus(rd_reg(reg_base + HDMI_PHY_STATUS));

	pr_debug("rtk-hdmitx: " "RX status=%u", val);

	if (val)
		return HDMI_RX_SENSE_ON;
	else
		return HDMI_RX_SENSE_OFF;
}

int hdmitx_check_tmds_src(void __iomem *reg_base)
{
	u32 val;

	if (tmds_en == 1) {
		val = rd_reg(reg_base + HDMI_CR);
		pr_info("rtk-hdmitx: " "CR=%x", val);

		if (val == 0x15)
			return TMDS_HDMI_ENABLED;
		else if (val == 0x0)
			return TMDS_HDMI_DISABLED;
		else if (val == 0x11)
			return TMDS_MHL_ENABLED;
		else
			return TMDS_MODE_UNKNOW;

	} else {
		return TMDS_HDMI_ENABLED;
	}

}

void hdmitx_turn_off_tmds(int vo_mode)
{
	void __iomem *vaddr;

	if (!tmds_en) {
		pr_info("rtk-hdmitx: " "skip %s", __func__);
		return;
	}

	vaddr = ioremap(HDMI_CR_reg, 0x4);

	wr_reg(vaddr, 0x2A);
	pr_info("rtk-hdmitx: " "CR OFF = %x", rd_reg(vaddr));
	iounmap(vaddr);
}

int hdmitx_send_AVmute(void __iomem *reg_base, int flag)
{
	if ((flag == HDMI_SETAVM) || (flag == HDMI_CLRAVM))
		set_mute_gpio_pulse();

	/*  Skip AV mute if HDMI clock not enable */
	if (is_hdmi_clock_on() != HDMI_CLK_PLL_ON) {
		pr_debug("rtk-hdmitx: " "Skip av mute, HDMI clock not enable");
		return 1;
	}

	if (flag == HDMI_SETAVM) {

		pr_debug("rtk-hdmitx: " "set av mute");

		wr_reg((reg_base + HDMI_GCPCR), HDMI_GCPCR_enablegcp(1) |
									HDMI_GCPCR_gcp_clearavmute(1) |
									HDMI_GCPCR_gcp_setavmute(1) |
									HDMI_GCPCR_write_data(0));
		wr_reg((reg_base + HDMI_GCPCR), HDMI_GCPCR_enablegcp(1) |
									HDMI_GCPCR_gcp_clearavmute(0) |
									HDMI_GCPCR_gcp_setavmute(1) |
									HDMI_GCPCR_write_data(1));

		msleep(50);/* Makes sure sink device can receive mute */
		return 0;
	} else if (flag == HDMI_CLRAVM) {

		pr_debug("rtk-hdmitx: " "clear av mute");

		wr_reg((reg_base + HDMI_GCPCR), HDMI_GCPCR_enablegcp(1) |
									HDMI_GCPCR_gcp_clearavmute(1) |
									HDMI_GCPCR_gcp_setavmute(1) |
									HDMI_GCPCR_write_data(0));
		wr_reg((reg_base + HDMI_GCPCR), HDMI_GCPCR_enablegcp(1) |
									HDMI_GCPCR_gcp_clearavmute(1) |
									HDMI_GCPCR_gcp_setavmute(0) |
									HDMI_GCPCR_write_data(1));
		return 0;
	} else {

		pr_debug("rtk-hdmitx: " "unknown av mute parameter");
		return 1;
	}
}


/* Need ION_AUDIO_HEAP */
void register_ion_client(const char *name)
{
	rpc_ion_client = ion_client_create(rtk_phoenix_ion_device, name);
}

void deregister_ion_client(const char *name)
{
	if (rpc_ion_client != NULL) {
		pr_info("rtk-hdmitx: " "%s :%s", __func__, name);
		ion_client_destroy(rpc_ion_client);
	}
}

#if __RTK_HDMI_GENERIC_DEBUG__
void hdmitx_dump_VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM(struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *arg, char *str)
{
	pr_debug("rtk-hdmitx: " "%s", __func__);
	pr_info("rtk-hdmitx: " "interfaceType       =0x%x", arg->interfaceType);

	pr_info("rtk-hdmitx: " "videoInfo.standard  =0x%x", arg->videoInfo.standard);
	pr_info("rtk-hdmitx: " "videoInfo.enProg    =0x%x", arg->videoInfo.enProg);
	pr_info("rtk-hdmitx: " "videoInfo.enDIF     =0x%x", arg->videoInfo.enDIF);
	pr_info("rtk-hdmitx: " "videoInfo.enCompRGB =0x%x", arg->videoInfo.enCompRGB);
	pr_info("rtk-hdmitx: " "videoInfo.pedType   =0x%x", arg->videoInfo.pedType);
	pr_info("rtk-hdmitx: " "videoInfo.dataInt0  =0x%x", arg->videoInfo.dataInt0);
	pr_info("rtk-hdmitx: " "videoInfo.dataInt1  =0x%x", arg->videoInfo.dataInt1);

	pr_info("rtk-hdmitx: " "hdmiInfo.hdmiMode   =0x%x", arg->hdmiInfo.hdmiMode);
    pr_info("rtk-hdmitx: " "hdmiInfo.audioSampleFreq    =0x%x", arg->hdmiInfo.audioSampleFreq);
	pr_info("rtk-hdmitx: " "hdmiInfo.audioChannelCount  =0x%x", arg->hdmiInfo.audioChannelCount);
	pr_info("rtk-hdmitx: " "hdmiInfo.dataByte1  =0x%x", arg->hdmiInfo.dataByte1);
	pr_info("rtk-hdmitx: " "hdmiInfo.dataByte2  =0x%x", arg->hdmiInfo.dataByte2);
	pr_info("rtk-hdmitx: " "hdmiInfo.dataByte3  =0x%x", arg->hdmiInfo.dataByte3);
	pr_info("rtk-hdmitx: " "hdmiInfo.dataByte4  =0x%x", arg->hdmiInfo.dataByte4);
	pr_info("rtk-hdmitx: " "hdmiInfo.dataByte5  =0x%x", arg->hdmiInfo.dataByte5);
	pr_info("rtk-hdmitx: " "hdmiInfo.dataInt0   =0x%x", arg->hdmiInfo.dataInt0);
	pr_info("rtk-hdmitx: " "hdmiInfo.hdmi2p0_feature   =0x%x", arg->hdmiInfo.hdmi2p0_feature);
	pr_info("rtk-hdmitx: " "hdmiInfo.hdmi_off_mode     =0x%x", arg->hdmiInfo.hdmi_off_mode);
	pr_info("rtk-hdmitx: " "hdmiInfo.hdr_ctrl_mode     =0x%x", arg->hdmiInfo.hdr_ctrl_mode);
	pr_info("rtk-hdmitx: " "hdmiInfo.reserved4         =0x%x", arg->hdmiInfo.reserved4);

}

void hdmitx_dump_AUDIO_HDMI_OUT_EDID_DATA2(struct AUDIO_HDMI_OUT_EDID_DATA2 *arg, char *str)
{
	pr_debug("rtk-hdmitx: " "%s", __func__);
	pr_info("rtk-hdmitx: " "Version         =0x%x", arg->Version);
	pr_info("rtk-hdmitx: " "HDMI_output_enable  =0x%x", arg->HDMI_output_enable);
	pr_info("rtk-hdmitx: " "EDID_DATA_addr  =0x%x", arg->EDID_DATA_addr);
}
#endif

int RPC_TOAGENT_HDMI_Config_TV_System(struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *arg)
{
	struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *rpc = NULL;
	int ret = -1;
	u32 RPC_ret;
	struct ion_handle *handle = NULL;
	phys_addr_t dat;
	size_t len;

	handle = ion_alloc(rpc_ion_client, 4096, 1024,
		RTK_PHOENIX_ION_HEAP_AUDIO_MASK,
		ION_FLAG_NONCACHED | ION_FLAG_SCPUACC | ION_FLAG_ACPUACC);

	if (IS_ERR(handle)) {
		pr_err("rtk-hdmitx: " "[%s %d ion_alloc fail]", __func__, __LINE__);
		goto exit;
	}

	if (ion_phys(rpc_ion_client, handle, &dat, &len) != 0) {
		pr_err("rtk-hdmitx: " "[%s %d fail]", __func__, __LINE__);
		goto exit;
	}

	rpc = ion_map_kernel(rpc_ion_client, handle);

	memcpy(rpc, arg, sizeof(struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM));

	rpc->interfaceType = htonl(arg->interfaceType);

	rpc->videoInfo.standard = htonl(arg->videoInfo.standard);
	rpc->videoInfo.pedType  = htonl(arg->videoInfo.pedType);
	rpc->videoInfo.dataInt0 = htonl(arg->videoInfo.dataInt0);
	rpc->videoInfo.dataInt1 = htonl(arg->videoInfo.dataInt1);

	rpc->hdmiInfo.hdmiMode  = htonl(arg->hdmiInfo.hdmiMode);
	rpc->hdmiInfo.audioSampleFreq = htonl(arg->hdmiInfo.audioSampleFreq);
	rpc->hdmiInfo.dataInt0  = htonl(arg->hdmiInfo.dataInt0);
	rpc->hdmiInfo.hdmi2p0_feature = htonl(arg->hdmiInfo.hdmi2p0_feature);
	rpc->hdmiInfo.hdmi_off_mode = htonl(arg->hdmiInfo.hdmi_off_mode);
	rpc->hdmiInfo.hdr_ctrl_mode = htonl(arg->hdmiInfo.hdr_ctrl_mode);
	rpc->hdmiInfo.reserved4 = htonl(arg->hdmiInfo.reserved4);

	if (send_rpc_command(RPC_AUDIO, ENUM_VIDEO_KERNEL_RPC_CONFIG_TV_SYSTEM,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM))),
		&RPC_ret)) {

		pr_err("rtk-hdmitx: " "[%s %d RPC fail]", __func__, __LINE__);
		goto exit;
	}

	ret = 0;
exit:
	if (handle != NULL) {
		ion_unmap_kernel(rpc_ion_client, handle);
		ion_free(rpc_ion_client, handle);
	}
	return ret;
}

int RPC_TOAGENT_HDMI_Config_AVI_Info(struct VIDEO_RPC_VOUT_CONFIG_HDMI_INFO_FRAME *arg)
{
	struct VIDEO_RPC_VOUT_CONFIG_HDMI_INFO_FRAME *rpc = NULL;
	int ret = -1;
	u32 RPC_ret;
	struct ion_handle *handle = NULL;
	phys_addr_t dat;
	size_t len;

	handle = ion_alloc(rpc_ion_client, 4096, 1024,
		RTK_PHOENIX_ION_HEAP_AUDIO_MASK,
		ION_FLAG_NONCACHED | ION_FLAG_SCPUACC | ION_FLAG_ACPUACC);

	if (IS_ERR(handle)) {
		pr_err("rtk-hdmitx: " "[%s %d ion_alloc fail]", __func__, __LINE__);
		goto exit;
	}

	if (ion_phys(rpc_ion_client, handle, &dat, &len) != 0) {
		pr_err("rtk-hdmitx: " "[%s %d fail]", __func__, __LINE__);
		goto exit;
	}

	rpc = ion_map_kernel(rpc_ion_client, handle);

	memcpy(rpc, arg, sizeof(struct VIDEO_RPC_VOUT_CONFIG_HDMI_INFO_FRAME));

	rpc->hdmiMode  = htonl(arg->hdmiMode);
	rpc->audioSampleFreq = htonl(arg->audioSampleFreq);
	rpc->dataInt0 =  htonl(arg->dataInt0);
	rpc->hdmi2p0_feature = htonl(arg->hdmi2p0_feature);
	rpc->hdmi_off_mode = htonl(arg->hdmi_off_mode);
	rpc->hdr_ctrl_mode = htonl(arg->hdr_ctrl_mode);
	rpc->reserved4 = htonl(arg->reserved4);

	if (send_rpc_command(RPC_AUDIO, ENUM_VIDEO_KERNEL_RPC_CONFIG_HDMI_INFO_FRAME,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(struct VIDEO_RPC_VOUT_CONFIG_HDMI_INFO_FRAME))),
		&RPC_ret)) {

		pr_err("rtk-hdmitx: " "[%s %d RPC fail]", __func__, __LINE__);
		goto exit;
	}

	ret = 0;
exit:
	if (handle != NULL) {
		ion_unmap_kernel(rpc_ion_client, handle);
		ion_free(rpc_ion_client, handle);
	}
	return ret;
}

int RPC_TOAGENT_HDMI_Set(struct AUDIO_HDMI_SET *arg)
{
	struct AUDIO_HDMI_SET *rpc = NULL;
	int ret = -1;
	u32 RPC_ret;
	struct ion_handle *handle = NULL;
	phys_addr_t dat;
	size_t len;

	handle = ion_alloc(rpc_ion_client, 4096, 1024,
		RTK_PHOENIX_ION_HEAP_AUDIO_MASK,
		ION_FLAG_NONCACHED | ION_FLAG_SCPUACC | ION_FLAG_ACPUACC);

	if (IS_ERR(handle)) {
		pr_err("rtk-hdmitx: " "[%s %d ion_alloc fail]", __func__, __LINE__);
		goto exit;
	}

	if (ion_phys(rpc_ion_client, handle, &dat, &len) != 0) {
		pr_err("rtk-hdmitx: " "[%s %d fail]", __func__, __LINE__);
		goto exit;
	}

	rpc = ion_map_kernel(rpc_ion_client, handle);

	pli_ipcCopyMemory((unsigned char *)rpc, (unsigned char *)arg,
		sizeof(struct AUDIO_HDMI_SET));

	if (send_rpc_command(RPC_AUDIO, ENUM_KERNEL_RPC_HDMI_SET,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(struct AUDIO_HDMI_SET))),
		&RPC_ret)) {

		pr_err("rtk-hdmitx: " "[%s %d RPC fail]", __func__, __LINE__);
		goto exit;
	}

	ret = 0;
exit:
	if (handle != NULL) {
		ion_unmap_kernel(rpc_ion_client, handle);
		ion_free(rpc_ion_client, handle);
	}
	return ret;
}


int RPC_TOAGENT_HDMI_Mute(struct AUDIO_HDMI_MUTE_INFO *arg)
{
	struct AUDIO_HDMI_MUTE_INFO *rpc = NULL;
	int ret = -1;
	u32 RPC_ret;
	struct ion_handle *handle = NULL;
	phys_addr_t dat;
	size_t len;

	handle = ion_alloc(rpc_ion_client, 4096, 1024,
		RTK_PHOENIX_ION_HEAP_AUDIO_MASK,
		ION_FLAG_NONCACHED | ION_FLAG_SCPUACC | ION_FLAG_ACPUACC);

	if (IS_ERR(handle)) {
	    pr_err("rtk-hdmitx: " "[%s %d ion_alloc fail]", __func__, __LINE__);
		goto exit;
	}

	if (ion_phys(rpc_ion_client, handle, &dat, &len) != 0) {
	    pr_err("rtk-hdmitx: " "[%s %d fail]", __func__, __LINE__);
		goto exit;
	}

	rpc = ion_map_kernel(rpc_ion_client, handle);

	rpc->instanceID = htonl(arg->instanceID);
	rpc->hdmi_mute = arg->hdmi_mute;

	if (send_rpc_command(RPC_AUDIO, ENUM_KERNEL_RPC_HDMI_MUTE,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(struct AUDIO_HDMI_MUTE_INFO))),
		&RPC_ret)) {

		pr_err("rtk-hdmitx: " "[%s %d RPC fail]", __func__, __LINE__);
		goto exit;
	}

	ret = 0;
exit:
	if (handle != NULL) {
		ion_unmap_kernel(rpc_ion_client, handle);
		ion_free(rpc_ion_client, handle);
	}
	return ret;
}

int RPC_TOAGENT_HDMI_OUT_VSDB(struct AUDIO_HDMI_OUT_VSDB_DATA *arg)
{
	struct AUDIO_HDMI_OUT_VSDB_DATA *rpc = NULL;
	int ret = -1;
	u32 RPC_ret;
	struct ion_handle *handle = NULL;
	phys_addr_t dat;
	size_t len;

	handle = ion_alloc(rpc_ion_client, 4096, 1024,
		RTK_PHOENIX_ION_HEAP_AUDIO_MASK,
		ION_FLAG_NONCACHED | ION_FLAG_SCPUACC | ION_FLAG_ACPUACC);

	if (IS_ERR(handle)) {
	    pr_err("rtk-hdmitx: " "[%s %d ion_alloc fail]", __func__, __LINE__);
		goto exit;
	}

	if (ion_phys(rpc_ion_client, handle, &dat, &len) != 0) {
	    pr_err("rtk-hdmitx: " "[%s %d fail]", __func__, __LINE__);
		goto exit;
	}

	rpc = ion_map_kernel(rpc_ion_client, handle);

	pli_ipcCopyMemory((unsigned char *)rpc, (unsigned char *)arg,
		sizeof(struct AUDIO_HDMI_OUT_VSDB_DATA));

	if (send_rpc_command(RPC_AUDIO, ENUM_KERNEL_RPC_HDMI_OUT_VSDB,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(struct AUDIO_HDMI_OUT_VSDB_DATA))),
		&RPC_ret)) {

		pr_err("rtk-hdmitx: " "[%s %d RPC fail]", __func__, __LINE__);
		goto exit;
	}

	ret = 0;
exit:
	if (handle != NULL) {
		ion_unmap_kernel(rpc_ion_client, handle);
		ion_free(rpc_ion_client, handle);
	}
	return ret;
}

int RPC_ToAgent_HDMI_OUT_EDID_0(struct AUDIO_HDMI_OUT_EDID_DATA2 *arg)
{
	struct AUDIO_HDMI_OUT_EDID_DATA2 *rpc = NULL;
	int ret = -1;
	u32 RPC_ret;
	struct ion_handle *handle = NULL;
	phys_addr_t dat;
	size_t len;

	handle = ion_alloc(rpc_ion_client, 4096, 1024,
				RTK_PHOENIX_ION_HEAP_AUDIO_MASK,
				ION_FLAG_NONCACHED | ION_FLAG_SCPUACC | ION_FLAG_ACPUACC);

	if (IS_ERR(handle)) {
	    pr_err("rtk-hdmitx: " "[%s %d ion_alloc fail]", __func__, __LINE__);
		goto exit;
	}

	if (ion_phys(rpc_ion_client, handle, &dat, &len) != 0) {
	    pr_err("rtk-hdmitx: " "[%s %d fail]", __func__, __LINE__);
		goto exit;
	}

	rpc = ion_map_kernel(rpc_ion_client, handle);

	pli_ipcCopyMemory((unsigned char *)rpc, (unsigned char *)arg,
		sizeof(struct AUDIO_HDMI_OUT_EDID_DATA2));

	if (send_rpc_command(RPC_AUDIO, ENUM_KERNEL_RPC_HDMI_OUT_EDID2,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(struct AUDIO_HDMI_OUT_EDID_DATA2))),
		&RPC_ret)) {

		pr_err("rtk-hdmitx: " "[%s %d RPC fail]", __func__, __LINE__);
		goto exit;
	}

	ret = 0;
exit:
	if (handle != NULL) {
		ion_unmap_kernel(rpc_ion_client, handle);
		ion_free(rpc_ion_client, handle);
	}
	return ret;
}

int RPC_ToAgent_QueryDisplayWin_0(struct VIDEO_RPC_VOUT_QUERY_DISP_WIN_OUT *arg)
{
	struct VIDEO_RPC_VOUT_QUERY_DISP_WIN_IN *i_rpc = NULL;
	struct VIDEO_RPC_VOUT_QUERY_DISP_WIN_OUT *o_rpc = NULL;
	int ret = -1;
	u32 RPC_ret;
	struct ion_handle *handle = NULL;
	phys_addr_t dat;
	size_t len;
	unsigned int offset;

	handle = ion_alloc(rpc_ion_client, 4096, 1024,
		RTK_PHOENIX_ION_HEAP_AUDIO_MASK,
		ION_FLAG_NONCACHED | ION_FLAG_SCPUACC | ION_FLAG_ACPUACC);

	if (IS_ERR(handle)) {
	    pr_err("rtk-hdmitx: " "[%s %d ion_alloc fail]", __func__, __LINE__);
	    goto exit;
	}

	if (ion_phys(rpc_ion_client, handle, &dat, &len) != 0) {
	    pr_err("rtk-hdmitx: " "[%s %d fail]", __func__, __LINE__);
	    goto exit;
	}

	i_rpc = ion_map_kernel(rpc_ion_client, handle);
	offset = get_rpc_alignment_offset(sizeof(struct VIDEO_RPC_VOUT_QUERY_DISP_WIN_IN));
	o_rpc = (struct VIDEO_RPC_VOUT_QUERY_DISP_WIN_OUT *)((unsigned long)i_rpc + offset);

	pr_debug("rtk-hdmitx: " "i_rpc=%p ,o_rpc=%p\n", i_rpc, o_rpc);

    if (send_rpc_command(RPC_AUDIO, ENUM_VIDEO_KERNEL_RPC_QUERY_DISPLAY_WIN,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&RPC_ret)) {

		pr_err("rtk-hdmitx: " "[%s %d RPC fail]", __func__, __LINE__);
		goto exit;
	}

	if (RPC_ret != S_OK) {
		pr_err("rtk-hdmitx: " "[%s %d RPC fail]", __func__, __LINE__);
		goto exit;
	}

	arg->standard = htonl(o_rpc->standard);
	pr_debug("rtk-hdmitx: " "%s: standard=%d\n", __func__, arg->standard);

	ret = 0;
exit:
	if (handle != NULL) {
		ion_unmap_kernel(rpc_ion_client, handle);
		ion_free(rpc_ion_client, handle);
	}
	return ret;
}

int RPC_ToAgent_Vout_EDIDdata(struct VIDEO_RPC_VOUT_EDID_DATA *arg)
{
	struct VIDEO_RPC_VOUT_EDID_DATA *rpc = NULL;
	int ret = -1;
	u32 RPC_ret;
	struct ion_handle *handle = NULL;
	phys_addr_t dat;
	size_t len;

	handle = ion_alloc(rpc_ion_client, 4096, 1024,
		RTK_PHOENIX_ION_HEAP_AUDIO_MASK,
		ION_FLAG_NONCACHED | ION_FLAG_SCPUACC | ION_FLAG_ACPUACC);

	if (IS_ERR(handle)) {
		pr_err("rtk-hdmitx: " "[%s %d ion_alloc fail]", __func__, __LINE__);
		goto exit;
	}

	if (ion_phys(rpc_ion_client, handle, &dat, &len) != 0) {
		pr_err("rtk-hdmitx: " "[%s %d fail]", __func__, __LINE__);
		goto exit;
	}

	rpc = ion_map_kernel(rpc_ion_client, handle);

	memcpy(rpc, arg, sizeof(struct VIDEO_RPC_VOUT_EDID_DATA));

	if (send_rpc_command(RPC_AUDIO, ENUM_VIDEO_KERNEL_RPC_VOUT_EDID_DATA,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(struct VIDEO_RPC_VOUT_EDID_DATA))),
		&RPC_ret)) {

		pr_err("rtk-hdmitx: " "[%s %d RPC fail]", __func__, __LINE__);
		goto exit;
	}

	ret = 0;
exit:
	if (handle != NULL) {
		ion_unmap_kernel(rpc_ion_client, handle);
		ion_free(rpc_ion_client, handle);
	}
	return ret;
}

int RPC_ToAgent_QueryConfigTvSystem(struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *arg)
{
	struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *i_rpc = NULL;
	struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *o_rpc = NULL;
	int ret = -1;
	u32 RPC_ret;
	struct ion_handle *handle = NULL;
	phys_addr_t dat;
	size_t len;
	unsigned int offset;

	handle = ion_alloc(rpc_ion_client, 4096, 1024,
		RTK_PHOENIX_ION_HEAP_AUDIO_MASK,
		ION_FLAG_NONCACHED | ION_FLAG_SCPUACC | ION_FLAG_ACPUACC);

	if (IS_ERR(handle)) {
		pr_err("rtk-hdmitx: " "[%s %d ion_alloc fail]", __func__, __LINE__);
		goto exit;
	}

	if (ion_phys(rpc_ion_client, handle, &dat, &len) != 0) {
	    pr_err("rtk-hdmitx: " "[%s %d fail]", __func__, __LINE__);
		goto exit;
	}

	i_rpc = ion_map_kernel(rpc_ion_client, handle);
	offset = get_rpc_alignment_offset(sizeof(struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM));
	o_rpc = (struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *)((unsigned long)i_rpc + offset);

	if (send_rpc_command(RPC_AUDIO, ENUM_VIDEO_KERNEL_RPC_QUERY_CONFIG_TV_SYSTEM,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + offset),
		&RPC_ret)) {

		pr_err("rtk-hdmitx: " "[%s %d RPC fail]", __func__, __LINE__);
		goto exit;
	}

	memcpy(arg, o_rpc, sizeof(struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM));

	arg->interfaceType				= htonl(arg->interfaceType);
	arg->videoInfo.standard			= htonl(arg->videoInfo.standard);
	arg->videoInfo.pedType			= htonl(arg->videoInfo.pedType);
	arg->videoInfo.dataInt0			= htonl(arg->videoInfo.dataInt0);
	arg->videoInfo.dataInt1			= htonl(arg->videoInfo.dataInt1);
	arg->hdmiInfo.hdmiMode          = htonl(arg->hdmiInfo.hdmiMode);
	arg->hdmiInfo.audioSampleFreq	= htonl(arg->hdmiInfo.audioSampleFreq);
	arg->hdmiInfo.dataInt0			= htonl(arg->hdmiInfo.dataInt0);
	arg->hdmiInfo.hdmi2p0_feature	= htonl(arg->hdmiInfo.hdmi2p0_feature);
	arg->hdmiInfo.hdmi_off_mode		= htonl(arg->hdmiInfo.hdmi_off_mode);
	arg->hdmiInfo.hdr_ctrl_mode		= htonl(arg->hdmiInfo.hdr_ctrl_mode);

	ret = 0;
exit:
	if (handle != NULL) {
		ion_unmap_kernel(rpc_ion_client, handle);
		ion_free(rpc_ion_client, handle);
	}
	return ret;
}

int hdmitx_reset_sink_capability(asoc_hdmi_t *p_this)
{
	p_this->sink_cap_available = false;
	memset(&p_this->sink_cap, 0x0, sizeof(p_this->sink_cap));
	memset(&p_this->sink_cap.cec_phy_addr, 0xff, sizeof(p_this->sink_cap.cec_phy_addr));
	memset(&hdmitx_edid_info, 0x0, sizeof(hdmitx_edid_info));

	if (p_this->edid_ptr != NULL) {
		kfree(p_this->edid_ptr);
		p_this->edid_ptr = NULL;
	}

	return 0;
}

void hdmitx_set_error_code(int error_code)
{
	hdmi_error = error_code;
}

void hdmitx_dump_error_code(void)
{
	switch (hdmi_error) {
	case HDMI_ERROR_NO_MEMORY:
		pr_err("rtk-hdmitx: " "get EDID failed, no memory");
		break;
	case HDMI_ERROR_I2C_ERROR:
		pr_err("rtk-hdmitx: " "get EDID failed, I2C error");
		break;
	case HDMI_ERROR_HPD:
		pr_err("rtk-hdmitx: " "get EDID failed, cable pulled out");
		break;
	case HDMI_ERROR_INVALID_EDID:
		pr_err("rtk-hdmitx: " "get EDID failed, invalid EDID");
		break;
	default:
		break;
	}
}

int hdmitx_check_same_edid(asoc_hdmi_t *p_this)
{
	int ret_val;
	struct edid *base_edid;
	struct edid *new_base_edid;

	ret_val = 0;

	if (p_this->sink_cap_available != true)
		goto out;

	new_base_edid = rtk_get_base_edid();
	if (new_base_edid == NULL)
		goto out;

	base_edid = (struct edid *) p_this->edid_ptr;

	if ((new_base_edid->mfg_id[0] == base_edid->mfg_id[0]) &&
		(new_base_edid->mfg_id[1] == base_edid->mfg_id[1]) &&
		(new_base_edid->checksum == base_edid->checksum)) {

		pr_info("rtk-hdmitx: " "Same EDID mfg_id=%02x %02x checksum=%02x",
			new_base_edid->mfg_id[0], new_base_edid->mfg_id[1],
			new_base_edid->checksum);

		ret_val = 1;
	}

	kfree(new_base_edid);
out:
	return ret_val;
}

int hdmitx_get_sink_capability(asoc_hdmi_t *p_this)
{
	struct edid *edid = NULL;

	if (p_this->sink_cap_available == true)
		return 0;

	hdmitx_set_error_code(HDMI_ERROR_RESERVED);

	edid = rtk_get_edid(p_this);
	if (edid == NULL)
		return -1;

	p_this->sink_cap_available = true;

	if (rtk_detect_hdmi_monitor(edid))
		p_this->sink_cap.hdmi_mode = HDMI_MODE_HDMI;
	else
		p_this->sink_cap.hdmi_mode = HDMI_MODE_DVI;

	rtk_add_edid_modes(edid, &p_this->sink_cap);
	rtk_edid_to_eld(edid, &p_this->sink_cap);

	hdmitx_print_sink_info(p_this);

	/* VIC filter for specific sink device */
	rtk_filter_specific_profuct_vic(edid, &p_this->sink_cap);

	tx_support_size = gen_hdmi_format_support(&tx_support[0],
		&p_this->sink_cap, &hdmitx_edid_info);

	return 0;
}

#if __RTK_HDMI_GENERIC_DEBUG__
void hdmitx_print_sink_capability(asoc_hdmi_t *p_this)
{
	pr_info("rtk-hdmitx: " "\n=================== sink_capabilities ========================");
	pr_info("rtk-hdmitx: " "hdmi_mode=%u ", p_this->sink_cap.hdmi_mode);
	pr_info("rtk-hdmitx: " "est_modes=%u ", p_this->sink_cap.est_modes);
	pr_info("rtk-hdmitx: " "\n");
	//VSBD
	pr_info("rtk-hdmitx: " "cec_phy_addr=0x%02x 0x%02x ", p_this->sink_cap.cec_phy_addr[0], p_this->sink_cap.cec_phy_addr[1]);
	pr_info("rtk-hdmitx: " "support_AI=%d ", p_this->sink_cap.support_AI);
	pr_info("rtk-hdmitx: " "DC_Y444=%d ", p_this->sink_cap.DC_Y444);
	pr_info("rtk-hdmitx: " "color_space=%u ", p_this->sink_cap.color_space);
	pr_info("rtk-hdmitx: " "dvi_dual=%u ", p_this->sink_cap.dvi_dual);
	pr_info("rtk-hdmitx: " "max_tmds_clock=%d ", p_this->sink_cap.max_tmds_clock);
	pr_info("rtk-hdmitx: " "latency_present=%d %d ", p_this->sink_cap.latency_present[0], p_this->sink_cap.latency_present[1]);
	pr_info("rtk-hdmitx: " "video_latency=%u %u ", p_this->sink_cap.video_latency[0], p_this->sink_cap.video_latency[1]);
	pr_info("rtk-hdmitx: " "audio_latency=%u %u ", p_this->sink_cap.audio_latency[0], p_this->sink_cap.audio_latency[1]);
	pr_info("rtk-hdmitx: " "\n");

	pr_info("rtk-hdmitx: " "structure_all=0x%x\n", p_this->sink_cap.structure_all);

	//Video
	pr_info("rtk-hdmitx: " "display_info.width_mm=%u ", p_this->sink_cap.display_info.width_mm);
	pr_info("rtk-hdmitx: " "display_info.height_mm=%u ", p_this->sink_cap.display_info.height_mm);
	pr_info("rtk-hdmitx: " "display_info.bpc=%u ", p_this->sink_cap.display_info.bpc);
	pr_info("rtk-hdmitx: " "display_info.color_formats=%u ", p_this->sink_cap.display_info.color_formats);
	pr_info("rtk-hdmitx: " "display_info.cea_rev=%u ", p_this->sink_cap.display_info.cea_rev);
	pr_info("rtk-hdmitx: " "vic=0x%llx ", p_this->sink_cap.vic);
	pr_info("rtk-hdmitx: " "extended_vic=%u ", p_this->sink_cap.extended_vic);
	pr_info("rtk-hdmitx: " "vic2=0x%llx ", p_this->sink_cap.vic2);
	pr_info("rtk-hdmitx: " "vic2_420=0x%llx ", p_this->sink_cap.vic2_420);
	pr_info("rtk-hdmitx: " "HDR metadata: et(0x%02x) sm(0x%02x) max_luminace(0x%02x) max_frame_avg(0x%02x) min_luminace(0x%02x) ",
		p_this->sink_cap.vout_edid_data.et, p_this->sink_cap.vout_edid_data.sm,
		p_this->sink_cap.vout_edid_data.max_luminace, p_this->sink_cap.vout_edid_data.max_frame_avg,
		p_this->sink_cap.vout_edid_data.min_luminace);
	pr_info("rtk-hdmitx: " "Color characteristics: red_green_lo(0x%02x) black_white_lo(0x%02x) ",
		p_this->sink_cap.vout_edid_data.red_green_lo, p_this->sink_cap.vout_edid_data.black_white_lo);
	pr_info("rtk-hdmitx: " "Color characteristics: red_x(0x%02x) red_y(0x%02x) ",
		p_this->sink_cap.vout_edid_data.red_x, p_this->sink_cap.vout_edid_data.red_y);
	pr_info("rtk-hdmitx: " "Color characteristics: green_x(0x%02x) green_y(0x%02x) ",
		p_this->sink_cap.vout_edid_data.green_x, p_this->sink_cap.vout_edid_data.green_y);
	pr_info("rtk-hdmitx: " "Color characteristics: blue_x(0x%02x) blue_y(0x%02x) ",
		p_this->sink_cap.vout_edid_data.blue_x, p_this->sink_cap.vout_edid_data.blue_y);
	pr_info("rtk-hdmitx: " "Color characteristics: white_x(0x%02x) white_y(0x%02x) ",
		p_this->sink_cap.vout_edid_data.white_x, p_this->sink_cap.vout_edid_data.white_y);

	pr_info("rtk-hdmitx: " "=================================================================\n");
}
#endif

void hdmitx_print_sink_info(asoc_hdmi_t *p_this)
{
	pr_info("rtk-hdmitx: " "Dump EDID\n");

	print_color_formats(p_this->sink_cap.display_info.color_formats);

	if (p_this->sink_cap.hdmi_mode == HDMI_MODE_HDMI)
		pr_info("rtk-hdmitx: " "DEVICE MODE: HDMI Mode ");
	else if (p_this->sink_cap.hdmi_mode == HDMI_MODE_DVI)
		pr_info("rtk-hdmitx: " "DEVICE MODE: DVI Mode ");
	else if (p_this->sink_cap.hdmi_mode == HDMI_MODE_MHL)
		pr_info("rtk-hdmitx: " "DEVICE MODE: MHL Mode ");

	print_cea_modes(p_this->sink_cap.vic, p_this->sink_cap.vic2, p_this->sink_cap.vic2_420);

	pr_info("rtk-hdmitx: " "CEC Address: 0x%x%02x ", p_this->sink_cap.cec_phy_addr[0],
										p_this->sink_cap.cec_phy_addr[1]);

	print_deep_color(p_this->sink_cap.display_info.bpc);

	if (p_this->sink_cap.DC_Y444)
		pr_info("rtk-hdmitx: " "Support Deep Color in YCbCr444 ");

	if (p_this->sink_cap.max_tmds_clock)
		pr_info("rtk-hdmitx: " "Max TMDS clock: %d", p_this->sink_cap.max_tmds_clock);

	pr_info("rtk-hdmitx: " "Video Latency: %d %d ", p_this->sink_cap.video_latency[0],
										p_this->sink_cap.video_latency[1]);

	pr_info("rtk-hdmitx: " "Audio Latency: %d %d ", p_this->sink_cap.audio_latency[0],
										p_this->sink_cap.audio_latency[1]);

	print_color_space(p_this->sink_cap.color_space);

	hdmi_print_raw_edid(p_this->edid_ptr);
}

int ops_config_tv_system(void __user *arg)
{
	struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM tv_system;
	unsigned char scramble;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	if (copy_from_user(&tv_system, arg, sizeof(tv_system))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy from user !", __func__);
		return -EFAULT;
	}

	pr_info("rtk-hdmitx: " "TV system: hdmiMode(%u) standard(%u) dataByte1(0x%02x) dataInt0(0x%02x)",
				tv_system.hdmiInfo.hdmiMode,
				tv_system.videoInfo.standard,
				tv_system.hdmiInfo.dataByte1,
				tv_system.hdmiInfo.dataInt0);

	if (tv_system.hdmiInfo.hdmiMode == VO_HDMI_ON) {
		if (hdmitx_edid_info.hdmi_id == HDMI_2P0_IDENTIFIER)
			tv_system.hdmiInfo.hdmi2p0_feature |= 0x1;/* [Bit0] HDMI2.0 */

		scramble = hdmitx_send_scdc_TmdsConfig(tv_system.videoInfo.standard,
			tv_system.hdmiInfo.dataInt0, tv_system.hdmiInfo.dataByte1);

		if (scramble != 0)
			tv_system.hdmiInfo.hdmi2p0_feature |= 0x2;/* [Bit1]Scrabmle */
	} else if (tv_system.hdmiInfo.hdmiMode == VO_HDMI_OFF) {
		if (hdmi_clk_always_on) {
			pr_info("rtk-hdmitx: " "Keep clock always on");
			tv_system.hdmiInfo.hdmi_off_mode = VO_HDMI_OFF_CLOCK_ON;
		} else {
#if HDMI_RX_SENSE_SUPPORT
			if (hdmitx_switch_get_hpd()) {
				pr_info("rtk-hdmitx: " "Keep clock on when cable still connected");
				tv_system.hdmiInfo.hdmi_off_mode = VO_HDMI_OFF_CLOCK_ON;
			}
#endif
		}
	}

#if __RTK_HDMI_GENERIC_DEBUG__
	hdmitx_dump_VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM(&tv_system, NULL);
#endif
	return RPC_TOAGENT_HDMI_Config_TV_System(&tv_system);
}

int ops_config_avi_info(void __user *arg)
{
	struct VIDEO_RPC_VOUT_CONFIG_HDMI_INFO_FRAME info_frame;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	if (copy_from_user(&info_frame, arg, sizeof(info_frame))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy from user !", __func__);
		return -EFAULT;
	}

	return RPC_TOAGENT_HDMI_Config_AVI_Info(&info_frame);
}

int ops_set_frequency(void __user *arg)
{
	struct AUDIO_HDMI_SET set;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	if (copy_from_user(&set, arg, sizeof(set))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy from user !", __func__);
		return -EFAULT;
	}

	return RPC_TOAGENT_HDMI_Set(&set);
}

int ops_set_audio_mute(void __user *arg)
{
	struct AUDIO_HDMI_MUTE_INFO mute_info;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	if (copy_from_user(&mute_info, arg, sizeof(mute_info))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy from user !", __func__);
		return -EFAULT;
	}

	return RPC_TOAGENT_HDMI_Mute(&mute_info);
}

int ops_send_audio_vsdb_data(void __user *arg)
{
	struct AUDIO_HDMI_OUT_VSDB_DATA vsdb_data;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	if (copy_from_user(&vsdb_data, arg, sizeof(vsdb_data))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy from user ! ", __func__);
		return -EFAULT;
	}

	return RPC_TOAGENT_HDMI_OUT_VSDB(&vsdb_data);
}

int ops_send_audio_edid2(void __user *arg)
{
	struct AUDIO_HDMI_OUT_EDID_DATA2 edid2;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	if (copy_from_user(&edid2, arg, sizeof(edid2))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy from user !", __func__);
		return -EFAULT;
	}

	return RPC_ToAgent_HDMI_OUT_EDID_0(&edid2);
}

int ops_query_display_standard(void __user *arg)
{
	int ret = 0;
	int standard = 0;
	struct VIDEO_RPC_VOUT_QUERY_DISP_WIN_OUT display_win;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	ret = RPC_ToAgent_QueryDisplayWin_0(&display_win);
	standard = (int)display_win.standard;

	if (copy_to_user(arg, &standard, sizeof(standard))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy to user !", __func__);
		return -EFAULT;
	}

	return ret;
}

int ops_get_sink_cap(void __user *arg, asoc_hdmi_t *data)
{
	int ret = 0;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	if (!(data->sink_cap_available)) {
		pr_err("rtk-hdmitx: " "[%s]sink cap is not available", __func__);
		return -ENOMSG;
	}

	if (copy_to_user(arg, &data->sink_cap, sizeof(data->sink_cap))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy to user !", __func__);
		return -EFAULT;
	}

#if __RTK_HDMI_GENERIC_DEBUG__
	hdmitx_print_sink_capability(data);
#endif
	return ret;
}

int ops_get_raw_edid(void __user *arg, asoc_hdmi_t *data)
{
	int ret = 0;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	if (!(data->sink_cap_available)) {
		pr_err("rtk-hdmitx: " "[%s]sink cap is not available", __func__);
		return -ENOMSG;
	}

	if (copy_to_user(arg, data->edid_ptr, sizeof(struct raw_edid))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy to user !", __func__);
		return -EFAULT;
	}

	return ret;
}

int ops_get_link_status(void __user *arg)
{
	int ret = 0;
	int status = 0;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	status = show_hpd_status(false);

	if (copy_to_user(arg, &status, sizeof(status))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy to user !", __func__);
		return -EFAULT;
	}

	return ret;
}

int ops_get_video_config(void __user *arg, asoc_hdmi_t *data)
{
	int ret = 0;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	if (copy_to_user(arg, &data->sink_cap.vic,
		sizeof(data->sink_cap.vic))) {

		pr_err("rtk-hdmitx: " "%s:failed to copy to user !", __func__);
		return -EFAULT;
	}

	return ret;
}

int ops_send_AVmute(void __user *arg, void __iomem *base)
{
	int flag;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	if (copy_from_user(&flag, arg, sizeof(flag))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy from user !", __func__);
		return -EFAULT;
	}

	return hdmitx_send_AVmute(base, flag);
}

int ops_check_tmds_src(void __user *arg, void __iomem *base)
{
	int tmds_mode = hdmitx_check_tmds_src(base);
	int ret = 0;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	if (copy_to_user(arg, &tmds_mode, sizeof(tmds_mode))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy to user !", __func__);
		return -EFAULT;
	}

	return ret;
}

int ops_check_rx_sense(void __user *arg, void __iomem *base)
{
	int rx_sense;
	int ret = 0;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	if (is_hdmi_clock_on() != HDMI_CLK_OFF) {
		rx_sense = hdmitx_check_rx_sense(base);
	} else {
		pr_debug("rtk-hdmitx: " "Skip check_rx_sense, HDMI clock not enable");
		rx_sense = HDMI_RX_SENSE_OFF;
	}

	if (copy_to_user(arg, &rx_sense, sizeof(rx_sense))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy to user !", __func__);
		return -EFAULT;
	}

	return ret;
}

int ops_get_extension_blk_count(void __user *arg, asoc_hdmi_t *data)
{
	int ret = 0;
	int count = 0;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	if (!(data->sink_cap_available)) {
		pr_err("rtk-hdmitx: " "[%s]sink cap is not available", __func__);
		return -ENOMSG;
	}

	count = (int) data->edid_ptr[0x7e];

	if (copy_to_user(arg, &count, sizeof(int))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy to user !", __func__);
		return -EFAULT;
	}

	return ret;
}


int ops_get_extended_edid(void __user *arg, asoc_hdmi_t *data)
{
	int ret = 0;
	unsigned char number;
	struct ext_edid ext = {0};

	pr_debug("rtk-hdmitx: " "%s", __func__);

	if (!(data->sink_cap_available)) {
		pr_err("rtk-hdmitx: " "[%s]sink cap is not available", __func__);
		return -ENOMSG;
	}

	if (copy_from_user(&ext, arg, sizeof(struct ext_edid))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy from user !", __func__);
		return -EFAULT;
	}

	if (ext.current_blk > ext.extension) {
		pr_err("rtk-hdmitx: " "[%s] out of range !", __func__);
		return -EFAULT;
	}

	pr_debug("rtk-hdmitx: " "ext.extension=%d ext.current_blk=%d",
		ext.extension, ext.current_blk);

	number = ext.extension - ext.current_blk + 1;
	if (number > 2)
		number = 2;

	memcpy(ext.data, data->edid_ptr + EDID_LENGTH*(ext.current_blk),
		EDID_LENGTH*number);

	if (copy_to_user(arg, &ext, sizeof(struct ext_edid))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy to user ! ", __func__);
		return -EFAULT;
	}

	return ret;
}


int ops_send_vout_edid_data(void __user *arg)
{
	struct VIDEO_RPC_VOUT_EDID_DATA vout_edid_data;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	if (copy_from_user(&vout_edid_data, arg, sizeof(vout_edid_data))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy from user !", __func__);
		return -EFAULT;
	}
	return RPC_ToAgent_Vout_EDIDdata(&vout_edid_data);
}

int ops_get_edid_support_list(void __user *arg, asoc_hdmi_t *data)
{
	int ret = 0;
	struct hdmi_support_list support_list;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	if (!(data->sink_cap_available)) {
		pr_err("rtk-hdmitx: " "[%s]sink cap is not available", __func__);
		ret = -ENOMSG;
		goto exit;
	}

	memcpy(&support_list.tx_support[0], &tx_support[0], sizeof(tx_support));
	support_list.tx_support_size = tx_support_size;

	if (copy_to_user(arg, &support_list, sizeof(struct hdmi_support_list))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy to user ! ", __func__);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}

int ops_set_output_format(void __user *arg)
{
	int ret = 0;
	struct hdmi_format_setting format_setting;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	if (copy_from_user(&format_setting, arg, sizeof(format_setting))) {
			pr_err("rtk-hdmitx: " "%s:failed to copy from user !", __func__);
			return -EFAULT;
	}

	ret = set_hdmitx_format(&format_setting);

#ifdef CONFIG_ARCH_RTD139x
#ifdef CONFIG_RTK_HDCP_1x_TEE
	if ((format_setting.mode != FORMAT_MODE_OFF) &&
		(format_setting.vic == VIC_720X480P60)) {
		ta_hdcp14_init();
		ta_hdcp_fix480p();
	}
#endif
#endif

	return ret;
}

int ops_get_output_format(void __user *arg)
{
	int ret = 0;
	struct hdmi_format_setting format_setting;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	ret = get_hdmitx_format(&format_setting);

	if (ret != 0)
		goto exit;

	if (copy_to_user(arg, &format_setting, sizeof(struct hdmi_format_setting))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy to user ! ", __func__);
		ret =  -EFAULT;
	}

exit:
	return ret;
}

int ops_set_interface_type(void __user *arg)
{
	int ret = 0;
	int type;

	pr_debug("rtk-hdmitx: " "%s", __func__);

	if (copy_from_user(&type, arg, sizeof(type))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy from user !", __func__);
		ret = -EFAULT;
	}

	set_vo_interface_type(type);

	return ret;
}

int ops_get_config_tv_system(void __user *arg)
{
	int ret;
	struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM tv_system;

	ret = RPC_ToAgent_QueryConfigTvSystem(&tv_system);

	if (ret != 0) {
		ret =  -EFAULT;
		goto exit;
	}

	if (copy_to_user(arg, &tv_system, sizeof(tv_system))) {
		pr_err("rtk-hdmitx: " "%s:failed to copy to user ! ", __func__);
		ret =  -EFAULT;
	}

	ret = 0;

exit:
	return ret;
}
