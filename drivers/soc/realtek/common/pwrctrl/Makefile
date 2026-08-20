obj-$(CONFIG_POWER_CONTROL)         += pwrctrl_core.o
obj-$(CONFIG_POWER_CONTROL_REALTEK) += pwrctrl-rtk.o

pwrctrl-rtk-y += pwrctrl-pd.o
pwrctrl-rtk-$(CONFIG_POWER_CONTROL_PD_SRAM) += pwrctrl-pd-sram.o
pwrctrl-rtk-$(CONFIG_POWER_CONTROL_PD_ANALOG) += pwrctrl-pd-analog.o
pwrctrl-rtk-$(CONFIG_POWER_CONTROL_PD_SRAM_CH) += pwrctrl-pd-sram_ch.o
pwrctrl-rtk-$(CONFIG_POWER_CONTROL_PD_SIMPLE) += pwrctrl-pd-simple.o

