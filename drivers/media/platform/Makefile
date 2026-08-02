# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the video capture/playback device drivers.
#

# Place here, alphabetically sorted by directory
# (e. g. LC_ALL=C sort Makefile)
obj-y += allegro-dvt/
obj-y += amlogic/
obj-y += amphion/
obj-y += aspeed/
obj-y += atmel/
obj-y += cadence/
obj-y += chips-media/
obj-y += intel/
obj-y += marvell/
obj-y += mediatek/
obj-y += nvidia/
obj-y += nxp/
obj-y += qcom/
obj-y += renesas/
obj-y += rockchip/
obj-y += samsung/
obj-y += st/
obj-y += sunxi/
obj-y += ti/
obj-y += verisilicon/
obj-y += via/
obj-y += xilinx/

obj-$(CONFIG_RTK_HDMITX)	+= rtk_hdmitx/
obj-$(CONFIG_RTK_HDCP_1x)	+= rtk_hdcptx/
ifeq ($(CONFIG_ARCH_RTD129x),y)
obj-$(CONFIG_RTK_HDMIRX)	+= rtk_hdmirx/rtd129x/
endif
ifeq ($(CONFIG_ARCH_RTD16xx),y)
obj-$(CONFIG_RTK_HDMIRX)	+= rtk_hdmirx/rtd161x/
endif
obj-$(CONFIG_CEC)		+= rtk_cec/
obj-$(CONFIG_XEN_CECDEV_FRONTEND) += rtk_cec/
obj-$(CONFIG_RTK_CVBS_SWITCH)	+= rtk_cvbsdetect/
obj-$(CONFIG_RTK_DPTX)		+= rtk_dptx/

ccflags-y += -I$(srctree)/drivers/media/i2c

# Please place here only ancillary drivers that aren't SoC-specific
# Please keep it alphabetically sorted by Kconfig name
# (e. g. LC_ALL=C sort Makefile)
obj-$(CONFIG_VIDEO_MEM2MEM_DEINTERLACE)	+= m2m-deinterlace.o
obj-$(CONFIG_VIDEO_MUX)			+= video-mux.o
