include $(srctree)/drivers/soc/realtek/rtk-vendor-ccflags.mk

ccflags-$(CONFIG_ARCH_RTD119X)	:= -I$(srctree) $(rtk_vendor-ccflags)

obj-$(CONFIG_ARCH_RTD119X)	+= rtd119x.o
obj-$(CONFIG_SMP)		+= platsmp.o headsmp.o
