include $(srctree)/drivers/soc/realtek/rtk-vendor-ccflags.mk
CFLAGS_sensor.o += $(rtk_vendor-ccflags)

obj-$(CONFIG_RTK_THERMAL) += rtk-thermal.o

rtk-thermal-y += sensor.o
rtk-thermal-$(CONFIG_RTK_THERMAL_RTD119X) += sensor-rtd119x.o
rtk-thermal-$(CONFIG_RTK_THERMAL_RTD129X) += sensor-rtd129x.o
rtk-thermal-$(CONFIG_RTK_THERMAL_RTD139X) += sensor-rtd139x.o
rtk-thermal-$(CONFIG_RTK_THERMAL_RTD16XX) += sensor-rtd16xx.o
