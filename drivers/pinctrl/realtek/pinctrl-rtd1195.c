// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Realtek RTD1195 pin controller driver
 *
 * Pin, mux and config tables derived from the Realtek vendor driver
 * (pinctrl-rtd119x.c/.h) and re-expressed on the mainline pinctrl-rtd core.
 *
 * The SoC has two independent pin controllers: "crt" in the CRT register
 * block and "iso" in the always-on ISO block, each with its own register
 * space, so they are registered as two separate devices.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/property.h>

#include "pinctrl-rtd.h"

enum rtd1195_crt_pins {
	RTD1195_CRT_GPIO_0 = 0,
	RTD1195_CRT_GPIO_1 = 1,
	RTD1195_CRT_GPIO_2 = 2,
	RTD1195_CRT_GPIO_3 = 3,
	RTD1195_CRT_GPIO_4 = 4,
	RTD1195_CRT_GPIO_5 = 5,
	RTD1195_CRT_GPIO_6 = 6,
	RTD1195_CRT_GPIO_7 = 7,
	RTD1195_CRT_GPIO_8 = 8,
	RTD1195_CRT_NF_DD_0 = 9,
	RTD1195_CRT_NF_DD_1 = 10,
	RTD1195_CRT_NF_DD_2 = 11,
	RTD1195_CRT_NF_DD_3 = 12,
	RTD1195_CRT_NF_DD_4 = 13,
	RTD1195_CRT_NF_DD_5 = 14,
	RTD1195_CRT_NF_DD_6 = 15,
	RTD1195_CRT_NF_DD_7 = 16,
	RTD1195_CRT_NF_RDY = 17,
	RTD1195_CRT_NF_RD_N = 18,
	RTD1195_CRT_NF_WR_N = 19,
	RTD1195_CRT_NF_ALE = 20,
	RTD1195_CRT_NF_CLE = 21,
	RTD1195_CRT_NF_CE_N_0 = 22,
	RTD1195_CRT_NF_CE_N_1 = 23,
	RTD1195_CRT_MMC_DATA_0 = 24,
	RTD1195_CRT_MMC_DATA_1 = 25,
	RTD1195_CRT_MMC_DATA_2 = 26,
	RTD1195_CRT_MMC_DATA_3 = 27,
	RTD1195_CRT_MMC_CLK = 28,
	RTD1195_CRT_MMC_CMD = 29,
	RTD1195_CRT_MMC_WP = 30,
	RTD1195_CRT_MMC_CD = 31,
	RTD1195_CRT_SDIO_CLK = 32,
	RTD1195_CRT_SDIO_DATA_0 = 33,
	RTD1195_CRT_SDIO_DATA_1 = 34,
	RTD1195_CRT_SDIO_DATA_2 = 35,
	RTD1195_CRT_SDIO_DATA_3 = 36,
	RTD1195_CRT_SDIO_CMD = 37,
	RTD1195_CRT_I2C_SCL_5 = 38,
	RTD1195_CRT_I2C_SDA_5 = 39,
	RTD1195_CRT_TP1_DATA = 40,
	RTD1195_CRT_TP1_CLK = 41,
	RTD1195_CRT_TP1_VALID = 42,
	RTD1195_CRT_TP1_SYNC = 43,
	RTD1195_CRT_TP0_DATA = 44,
	RTD1195_CRT_TP0_CLK = 45,
	RTD1195_CRT_TP0_VALID = 46,
	RTD1195_CRT_TP0_SYNC = 47,
	RTD1195_CRT_USB_ID = 48,
	RTD1195_CRT_HDMI_HPD = 49,
	RTD1195_CRT_SPDIF = 50,
	RTD1195_CRT_I2C_SCL_1 = 51,
	RTD1195_CRT_I2C_SDA_1 = 52,
	RTD1195_CRT_I2C_SCL_4 = 53,
	RTD1195_CRT_I2C_SDA_4 = 54,
	RTD1195_CRT_SENSOR_CKO_0 = 55,
	RTD1195_CRT_SENSOR_CKO_1 = 56,
	RTD1195_CRT_SENSOR_RST = 57,
	RTD1195_CRT_SENSOR_STB_0 = 58,
	RTD1195_CRT_SENSOR_STB_1 = 59,
	RTD1195_CRT_HI_LOC = 60,
	RTD1195_CRT_EJTAG_SCPU_LOC = 61,
	RTD1195_CRT_SF_EN = 62,
	RTD1195_CRT_AO_LOC = 63,
};

static const struct pinctrl_pin_desc rtd1195_crt_pin_desc[] = {
	PINCTRL_PIN(RTD1195_CRT_GPIO_0, "gpio_0"),
	PINCTRL_PIN(RTD1195_CRT_GPIO_1, "gpio_1"),
	PINCTRL_PIN(RTD1195_CRT_GPIO_2, "gpio_2"),
	PINCTRL_PIN(RTD1195_CRT_GPIO_3, "gpio_3"),
	PINCTRL_PIN(RTD1195_CRT_GPIO_4, "gpio_4"),
	PINCTRL_PIN(RTD1195_CRT_GPIO_5, "gpio_5"),
	PINCTRL_PIN(RTD1195_CRT_GPIO_6, "gpio_6"),
	PINCTRL_PIN(RTD1195_CRT_GPIO_7, "gpio_7"),
	PINCTRL_PIN(RTD1195_CRT_GPIO_8, "gpio_8"),
	PINCTRL_PIN(RTD1195_CRT_NF_DD_0, "nf_dd_0"),
	PINCTRL_PIN(RTD1195_CRT_NF_DD_1, "nf_dd_1"),
	PINCTRL_PIN(RTD1195_CRT_NF_DD_2, "nf_dd_2"),
	PINCTRL_PIN(RTD1195_CRT_NF_DD_3, "nf_dd_3"),
	PINCTRL_PIN(RTD1195_CRT_NF_DD_4, "nf_dd_4"),
	PINCTRL_PIN(RTD1195_CRT_NF_DD_5, "nf_dd_5"),
	PINCTRL_PIN(RTD1195_CRT_NF_DD_6, "nf_dd_6"),
	PINCTRL_PIN(RTD1195_CRT_NF_DD_7, "nf_dd_7"),
	PINCTRL_PIN(RTD1195_CRT_NF_RDY, "nf_rdy"),
	PINCTRL_PIN(RTD1195_CRT_NF_RD_N, "nf_rd_n"),
	PINCTRL_PIN(RTD1195_CRT_NF_WR_N, "nf_wr_n"),
	PINCTRL_PIN(RTD1195_CRT_NF_ALE, "nf_ale"),
	PINCTRL_PIN(RTD1195_CRT_NF_CLE, "nf_cle"),
	PINCTRL_PIN(RTD1195_CRT_NF_CE_N_0, "nf_ce_n_0"),
	PINCTRL_PIN(RTD1195_CRT_NF_CE_N_1, "nf_ce_n_1"),
	PINCTRL_PIN(RTD1195_CRT_MMC_DATA_0, "mmc_data_0"),
	PINCTRL_PIN(RTD1195_CRT_MMC_DATA_1, "mmc_data_1"),
	PINCTRL_PIN(RTD1195_CRT_MMC_DATA_2, "mmc_data_2"),
	PINCTRL_PIN(RTD1195_CRT_MMC_DATA_3, "mmc_data_3"),
	PINCTRL_PIN(RTD1195_CRT_MMC_CLK, "mmc_clk"),
	PINCTRL_PIN(RTD1195_CRT_MMC_CMD, "mmc_cmd"),
	PINCTRL_PIN(RTD1195_CRT_MMC_WP, "mmc_wp"),
	PINCTRL_PIN(RTD1195_CRT_MMC_CD, "mmc_cd"),
	PINCTRL_PIN(RTD1195_CRT_SDIO_CLK, "sdio_clk"),
	PINCTRL_PIN(RTD1195_CRT_SDIO_DATA_0, "sdio_data_0"),
	PINCTRL_PIN(RTD1195_CRT_SDIO_DATA_1, "sdio_data_1"),
	PINCTRL_PIN(RTD1195_CRT_SDIO_DATA_2, "sdio_data_2"),
	PINCTRL_PIN(RTD1195_CRT_SDIO_DATA_3, "sdio_data_3"),
	PINCTRL_PIN(RTD1195_CRT_SDIO_CMD, "sdio_cmd"),
	PINCTRL_PIN(RTD1195_CRT_I2C_SCL_5, "i2c_scl_5"),
	PINCTRL_PIN(RTD1195_CRT_I2C_SDA_5, "i2c_sda_5"),
	PINCTRL_PIN(RTD1195_CRT_TP1_DATA, "tp1_data"),
	PINCTRL_PIN(RTD1195_CRT_TP1_CLK, "tp1_clk"),
	PINCTRL_PIN(RTD1195_CRT_TP1_VALID, "tp1_valid"),
	PINCTRL_PIN(RTD1195_CRT_TP1_SYNC, "tp1_sync"),
	PINCTRL_PIN(RTD1195_CRT_TP0_DATA, "tp0_data"),
	PINCTRL_PIN(RTD1195_CRT_TP0_CLK, "tp0_clk"),
	PINCTRL_PIN(RTD1195_CRT_TP0_VALID, "tp0_valid"),
	PINCTRL_PIN(RTD1195_CRT_TP0_SYNC, "tp0_sync"),
	PINCTRL_PIN(RTD1195_CRT_USB_ID, "usb_id"),
	PINCTRL_PIN(RTD1195_CRT_HDMI_HPD, "hdmi_hpd"),
	PINCTRL_PIN(RTD1195_CRT_SPDIF, "spdif"),
	PINCTRL_PIN(RTD1195_CRT_I2C_SCL_1, "i2c_scl_1"),
	PINCTRL_PIN(RTD1195_CRT_I2C_SDA_1, "i2c_sda_1"),
	PINCTRL_PIN(RTD1195_CRT_I2C_SCL_4, "i2c_scl_4"),
	PINCTRL_PIN(RTD1195_CRT_I2C_SDA_4, "i2c_sda_4"),
	PINCTRL_PIN(RTD1195_CRT_SENSOR_CKO_0, "sensor_cko_0"),
	PINCTRL_PIN(RTD1195_CRT_SENSOR_CKO_1, "sensor_cko_1"),
	PINCTRL_PIN(RTD1195_CRT_SENSOR_RST, "sensor_rst"),
	PINCTRL_PIN(RTD1195_CRT_SENSOR_STB_0, "sensor_stb_0"),
	PINCTRL_PIN(RTD1195_CRT_SENSOR_STB_1, "sensor_stb_1"),
	PINCTRL_PIN(RTD1195_CRT_HI_LOC, "hi_loc"),
	PINCTRL_PIN(RTD1195_CRT_EJTAG_SCPU_LOC, "ejtag_scpu_loc"),
	PINCTRL_PIN(RTD1195_CRT_SF_EN, "sf_en"),
	PINCTRL_PIN(RTD1195_CRT_AO_LOC, "ao_loc"),
};

static const unsigned int rtd1195_crt_gpio_0_pins[] = { RTD1195_CRT_GPIO_0 };
static const unsigned int rtd1195_crt_gpio_1_pins[] = { RTD1195_CRT_GPIO_1 };
static const unsigned int rtd1195_crt_gpio_2_pins[] = { RTD1195_CRT_GPIO_2 };
static const unsigned int rtd1195_crt_gpio_3_pins[] = { RTD1195_CRT_GPIO_3 };
static const unsigned int rtd1195_crt_gpio_4_pins[] = { RTD1195_CRT_GPIO_4 };
static const unsigned int rtd1195_crt_gpio_5_pins[] = { RTD1195_CRT_GPIO_5 };
static const unsigned int rtd1195_crt_gpio_6_pins[] = { RTD1195_CRT_GPIO_6 };
static const unsigned int rtd1195_crt_gpio_7_pins[] = { RTD1195_CRT_GPIO_7 };
static const unsigned int rtd1195_crt_gpio_8_pins[] = { RTD1195_CRT_GPIO_8 };
static const unsigned int rtd1195_crt_nf_dd_0_pins[] = { RTD1195_CRT_NF_DD_0 };
static const unsigned int rtd1195_crt_nf_dd_1_pins[] = { RTD1195_CRT_NF_DD_1 };
static const unsigned int rtd1195_crt_nf_dd_2_pins[] = { RTD1195_CRT_NF_DD_2 };
static const unsigned int rtd1195_crt_nf_dd_3_pins[] = { RTD1195_CRT_NF_DD_3 };
static const unsigned int rtd1195_crt_nf_dd_4_pins[] = { RTD1195_CRT_NF_DD_4 };
static const unsigned int rtd1195_crt_nf_dd_5_pins[] = { RTD1195_CRT_NF_DD_5 };
static const unsigned int rtd1195_crt_nf_dd_6_pins[] = { RTD1195_CRT_NF_DD_6 };
static const unsigned int rtd1195_crt_nf_dd_7_pins[] = { RTD1195_CRT_NF_DD_7 };
static const unsigned int rtd1195_crt_nf_rdy_pins[] = { RTD1195_CRT_NF_RDY };
static const unsigned int rtd1195_crt_nf_rd_n_pins[] = { RTD1195_CRT_NF_RD_N };
static const unsigned int rtd1195_crt_nf_wr_n_pins[] = { RTD1195_CRT_NF_WR_N };
static const unsigned int rtd1195_crt_nf_ale_pins[] = { RTD1195_CRT_NF_ALE };
static const unsigned int rtd1195_crt_nf_cle_pins[] = { RTD1195_CRT_NF_CLE };
static const unsigned int rtd1195_crt_nf_ce_n_0_pins[] = { RTD1195_CRT_NF_CE_N_0 };
static const unsigned int rtd1195_crt_nf_ce_n_1_pins[] = { RTD1195_CRT_NF_CE_N_1 };
static const unsigned int rtd1195_crt_mmc_data_0_pins[] = { RTD1195_CRT_MMC_DATA_0 };
static const unsigned int rtd1195_crt_mmc_data_1_pins[] = { RTD1195_CRT_MMC_DATA_1 };
static const unsigned int rtd1195_crt_mmc_data_2_pins[] = { RTD1195_CRT_MMC_DATA_2 };
static const unsigned int rtd1195_crt_mmc_data_3_pins[] = { RTD1195_CRT_MMC_DATA_3 };
static const unsigned int rtd1195_crt_mmc_clk_pins[] = { RTD1195_CRT_MMC_CLK };
static const unsigned int rtd1195_crt_mmc_cmd_pins[] = { RTD1195_CRT_MMC_CMD };
static const unsigned int rtd1195_crt_mmc_wp_pins[] = { RTD1195_CRT_MMC_WP };
static const unsigned int rtd1195_crt_mmc_cd_pins[] = { RTD1195_CRT_MMC_CD };
static const unsigned int rtd1195_crt_sdio_clk_pins[] = { RTD1195_CRT_SDIO_CLK };
static const unsigned int rtd1195_crt_sdio_data_0_pins[] = { RTD1195_CRT_SDIO_DATA_0 };
static const unsigned int rtd1195_crt_sdio_data_1_pins[] = { RTD1195_CRT_SDIO_DATA_1 };
static const unsigned int rtd1195_crt_sdio_data_2_pins[] = { RTD1195_CRT_SDIO_DATA_2 };
static const unsigned int rtd1195_crt_sdio_data_3_pins[] = { RTD1195_CRT_SDIO_DATA_3 };
static const unsigned int rtd1195_crt_sdio_cmd_pins[] = { RTD1195_CRT_SDIO_CMD };
static const unsigned int rtd1195_crt_i2c_scl_5_pins[] = { RTD1195_CRT_I2C_SCL_5 };
static const unsigned int rtd1195_crt_i2c_sda_5_pins[] = { RTD1195_CRT_I2C_SDA_5 };
static const unsigned int rtd1195_crt_tp1_data_pins[] = { RTD1195_CRT_TP1_DATA };
static const unsigned int rtd1195_crt_tp1_clk_pins[] = { RTD1195_CRT_TP1_CLK };
static const unsigned int rtd1195_crt_tp1_valid_pins[] = { RTD1195_CRT_TP1_VALID };
static const unsigned int rtd1195_crt_tp1_sync_pins[] = { RTD1195_CRT_TP1_SYNC };
static const unsigned int rtd1195_crt_tp0_data_pins[] = { RTD1195_CRT_TP0_DATA };
static const unsigned int rtd1195_crt_tp0_clk_pins[] = { RTD1195_CRT_TP0_CLK };
static const unsigned int rtd1195_crt_tp0_valid_pins[] = { RTD1195_CRT_TP0_VALID };
static const unsigned int rtd1195_crt_tp0_sync_pins[] = { RTD1195_CRT_TP0_SYNC };
static const unsigned int rtd1195_crt_usb_id_pins[] = { RTD1195_CRT_USB_ID };
static const unsigned int rtd1195_crt_hdmi_hpd_pins[] = { RTD1195_CRT_HDMI_HPD };
static const unsigned int rtd1195_crt_spdif_pins[] = { RTD1195_CRT_SPDIF };
static const unsigned int rtd1195_crt_i2c_scl_1_pins[] = { RTD1195_CRT_I2C_SCL_1 };
static const unsigned int rtd1195_crt_i2c_sda_1_pins[] = { RTD1195_CRT_I2C_SDA_1 };
static const unsigned int rtd1195_crt_i2c_scl_4_pins[] = { RTD1195_CRT_I2C_SCL_4 };
static const unsigned int rtd1195_crt_i2c_sda_4_pins[] = { RTD1195_CRT_I2C_SDA_4 };
static const unsigned int rtd1195_crt_sensor_cko_0_pins[] = { RTD1195_CRT_SENSOR_CKO_0 };
static const unsigned int rtd1195_crt_sensor_cko_1_pins[] = { RTD1195_CRT_SENSOR_CKO_1 };
static const unsigned int rtd1195_crt_sensor_rst_pins[] = { RTD1195_CRT_SENSOR_RST };
static const unsigned int rtd1195_crt_sensor_stb_0_pins[] = { RTD1195_CRT_SENSOR_STB_0 };
static const unsigned int rtd1195_crt_sensor_stb_1_pins[] = { RTD1195_CRT_SENSOR_STB_1 };
static const unsigned int rtd1195_crt_hi_loc_pins[] = { RTD1195_CRT_HI_LOC };
static const unsigned int rtd1195_crt_ejtag_scpu_loc_pins[] = { RTD1195_CRT_EJTAG_SCPU_LOC };
static const unsigned int rtd1195_crt_sf_en_pins[] = { RTD1195_CRT_SF_EN };
static const unsigned int rtd1195_crt_ao_loc_pins[] = { RTD1195_CRT_AO_LOC };

#define RTD1195_CRT_GROUP(_name) \
	{ .name = # _name, .pins = rtd1195_crt_ ## _name ## _pins, \
	  .num_pins = ARRAY_SIZE(rtd1195_crt_ ## _name ## _pins) }

static const struct rtd_pin_group_desc rtd1195_crt_pin_groups[] = {
	RTD1195_CRT_GROUP(gpio_0),
	RTD1195_CRT_GROUP(gpio_1),
	RTD1195_CRT_GROUP(gpio_2),
	RTD1195_CRT_GROUP(gpio_3),
	RTD1195_CRT_GROUP(gpio_4),
	RTD1195_CRT_GROUP(gpio_5),
	RTD1195_CRT_GROUP(gpio_6),
	RTD1195_CRT_GROUP(gpio_7),
	RTD1195_CRT_GROUP(gpio_8),
	RTD1195_CRT_GROUP(nf_dd_0),
	RTD1195_CRT_GROUP(nf_dd_1),
	RTD1195_CRT_GROUP(nf_dd_2),
	RTD1195_CRT_GROUP(nf_dd_3),
	RTD1195_CRT_GROUP(nf_dd_4),
	RTD1195_CRT_GROUP(nf_dd_5),
	RTD1195_CRT_GROUP(nf_dd_6),
	RTD1195_CRT_GROUP(nf_dd_7),
	RTD1195_CRT_GROUP(nf_rdy),
	RTD1195_CRT_GROUP(nf_rd_n),
	RTD1195_CRT_GROUP(nf_wr_n),
	RTD1195_CRT_GROUP(nf_ale),
	RTD1195_CRT_GROUP(nf_cle),
	RTD1195_CRT_GROUP(nf_ce_n_0),
	RTD1195_CRT_GROUP(nf_ce_n_1),
	RTD1195_CRT_GROUP(mmc_data_0),
	RTD1195_CRT_GROUP(mmc_data_1),
	RTD1195_CRT_GROUP(mmc_data_2),
	RTD1195_CRT_GROUP(mmc_data_3),
	RTD1195_CRT_GROUP(mmc_clk),
	RTD1195_CRT_GROUP(mmc_cmd),
	RTD1195_CRT_GROUP(mmc_wp),
	RTD1195_CRT_GROUP(mmc_cd),
	RTD1195_CRT_GROUP(sdio_clk),
	RTD1195_CRT_GROUP(sdio_data_0),
	RTD1195_CRT_GROUP(sdio_data_1),
	RTD1195_CRT_GROUP(sdio_data_2),
	RTD1195_CRT_GROUP(sdio_data_3),
	RTD1195_CRT_GROUP(sdio_cmd),
	RTD1195_CRT_GROUP(i2c_scl_5),
	RTD1195_CRT_GROUP(i2c_sda_5),
	RTD1195_CRT_GROUP(tp1_data),
	RTD1195_CRT_GROUP(tp1_clk),
	RTD1195_CRT_GROUP(tp1_valid),
	RTD1195_CRT_GROUP(tp1_sync),
	RTD1195_CRT_GROUP(tp0_data),
	RTD1195_CRT_GROUP(tp0_clk),
	RTD1195_CRT_GROUP(tp0_valid),
	RTD1195_CRT_GROUP(tp0_sync),
	RTD1195_CRT_GROUP(usb_id),
	RTD1195_CRT_GROUP(hdmi_hpd),
	RTD1195_CRT_GROUP(spdif),
	RTD1195_CRT_GROUP(i2c_scl_1),
	RTD1195_CRT_GROUP(i2c_sda_1),
	RTD1195_CRT_GROUP(i2c_scl_4),
	RTD1195_CRT_GROUP(i2c_sda_4),
	RTD1195_CRT_GROUP(sensor_cko_0),
	RTD1195_CRT_GROUP(sensor_cko_1),
	RTD1195_CRT_GROUP(sensor_rst),
	RTD1195_CRT_GROUP(sensor_stb_0),
	RTD1195_CRT_GROUP(sensor_stb_1),
	RTD1195_CRT_GROUP(hi_loc),
	RTD1195_CRT_GROUP(ejtag_scpu_loc),
	RTD1195_CRT_GROUP(sf_en),
	RTD1195_CRT_GROUP(ao_loc),
};

static const char * const rtd1195_crt_ao_loc_misc_groups[] = {
	"gpio_4", "gpio_5", "gpio_6", "gpio_7" };
static const char * const rtd1195_crt_ao_loc_tp_groups[] = {
	"tp0_data", "tp0_clk", "tp0_valid", "tp0_sync" };
static const char * const rtd1195_crt_avcpu_ejtag_loc_nf_groups[] = {
	"nf_dd_5", "nf_dd_6", "nf_dd_7", "nf_rdy", "nf_rd_n" };
static const char * const rtd1195_crt_cpu_loop_groups[] = {
	"usb_id" };
static const char * const rtd1195_crt_emmc_groups[] = {
	"nf_dd_0", "nf_dd_1", "nf_dd_2", "nf_dd_3", "nf_dd_4", "nf_dd_5", "nf_dd_6", "nf_dd_7",
	"nf_rdy", "nf_rd_n", "nf_wr_n", "nf_ale", "nf_cle" };
static const char * const rtd1195_crt_gpio_groups[] = {
	"gpio_0", "gpio_1", "gpio_2", "gpio_3", "gpio_4", "gpio_5", "gpio_6", "gpio_7", "gpio_8",
	"nf_dd_0", "nf_dd_1", "nf_dd_2", "nf_dd_3", "nf_dd_4", "nf_dd_5", "nf_dd_6", "nf_dd_7",
	"nf_rdy", "nf_rd_n", "nf_wr_n", "nf_ale", "nf_cle", "nf_ce_n_0", "nf_ce_n_1", "mmc_data_0",
	"mmc_data_1", "mmc_data_2", "mmc_data_3", "mmc_clk", "mmc_cmd", "mmc_wp", "mmc_cd", "sdio_clk",
	"sdio_data_0", "sdio_data_1", "sdio_data_2", "sdio_data_3", "sdio_cmd", "i2c_scl_5",
	"i2c_sda_5", "tp1_data", "tp1_clk", "tp1_valid", "tp1_sync", "tp0_data", "tp0_clk",
	"tp0_valid", "tp0_sync", "usb_id", "hdmi_hpd", "spdif", "i2c_scl_1", "i2c_sda_1", "i2c_scl_4",
	"i2c_sda_4", "sensor_cko_0", "sensor_cko_1", "sensor_rst", "sensor_stb_0", "sensor_stb_1",
	"sf_en" };
static const char * const rtd1195_crt_gspi_groups[] = {
	"gpio_0", "gpio_1", "gpio_2", "gpio_3", "sf_en" };
static const char * const rtd1195_crt_hif_loc_misc_groups[] = {
	"gpio_0", "gpio_1", "gpio_2", "gpio_3", "hi_loc", "sf_en" };
static const char * const rtd1195_crt_hif_loc_nf_groups[] = {
	"nf_dd_4", "nf_wr_n", "nf_ale", "nf_cle", "hi_loc" };
static const char * const rtd1195_crt_i2c1_groups[] = {
	"i2c_scl_1", "i2c_sda_1" };
static const char * const rtd1195_crt_i2c2_groups[] = {
	"tp1_clk", "tp1_sync" };
static const char * const rtd1195_crt_i2c3_groups[] = {
	"tp1_data", "tp1_valid" };
static const char * const rtd1195_crt_i2c4_groups[] = {
	"i2c_scl_4", "i2c_sda_4" };
static const char * const rtd1195_crt_i2c5_groups[] = {
	"i2c_scl_5", "i2c_sda_5" };
static const char * const rtd1195_crt_mmc_groups[] = {
	"mmc_data_0", "mmc_data_1", "mmc_data_2", "mmc_data_3", "mmc_clk", "mmc_cmd", "mmc_wp", "mmc_cd" };
static const char * const rtd1195_crt_nand_groups[] = {
	"nf_dd_0", "nf_dd_1", "nf_dd_2", "nf_dd_3", "nf_dd_4", "nf_dd_5", "nf_dd_6", "nf_dd_7",
	"nf_rdy", "nf_rd_n", "nf_wr_n", "nf_ale", "nf_cle", "nf_ce_n_0", "nf_ce_n_1", "i2c_scl_5",
	"i2c_sda_5" };
static const char * const rtd1195_crt_scpu_ejtag_loc_cr_groups[] = {
	"mmc_data_0", "mmc_data_3", "mmc_clk", "mmc_cmd", "mmc_wp", "ejtag_scpu_loc" };
static const char * const rtd1195_crt_scpu_ejtag_loc_misc_groups[] = {
	"gpio_4", "gpio_5", "gpio_6", "gpio_7", "gpio_8", "ejtag_scpu_loc" };
static const char * const rtd1195_crt_sdio_groups[] = {
	"sdio_clk", "sdio_data_0", "sdio_data_1", "sdio_data_2", "sdio_data_3", "sdio_cmd" };
static const char * const rtd1195_crt_sensor_groups[] = {
	"sensor_cko_0", "sensor_cko_1" };
static const char * const rtd1195_crt_spdif_groups[] = {
	"spdif" };
static const char * const rtd1195_crt_spi_groups[] = {
	"sf_en" };
static const char * const rtd1195_crt_tp0_groups[] = {
	"tp1_data", "tp1_clk", "tp1_valid", "tp1_sync", "tp0_data", "tp0_clk", "tp0_valid", "tp0_sync" };
static const char * const rtd1195_crt_tp1_groups[] = {
	"tp1_data", "tp1_clk", "tp1_valid", "tp1_sync", "tp0_data", "tp0_clk", "tp0_valid", "tp0_sync" };
static const char * const rtd1195_crt_uart1_loc_misc_groups[] = {
	"gpio_0", "gpio_1", "gpio_2", "gpio_3", "sf_en" };
static const char * const rtd1195_crt_usb_groups[] = {
	"sensor_cko_1" };

#define RTD1195_CRT_FUNC(_name) \
	{ .name = # _name, .groups = rtd1195_crt_ ## _name ## _groups, \
	  .num_groups = ARRAY_SIZE(rtd1195_crt_ ## _name ## _groups) }

static const struct rtd_pin_func_desc rtd1195_crt_pin_functions[] = {
	RTD1195_CRT_FUNC(ao_loc_misc),
	RTD1195_CRT_FUNC(ao_loc_tp),
	RTD1195_CRT_FUNC(avcpu_ejtag_loc_nf),
	RTD1195_CRT_FUNC(cpu_loop),
	RTD1195_CRT_FUNC(emmc),
	RTD1195_CRT_FUNC(gpio),
	RTD1195_CRT_FUNC(gspi),
	RTD1195_CRT_FUNC(hif_loc_misc),
	RTD1195_CRT_FUNC(hif_loc_nf),
	RTD1195_CRT_FUNC(i2c1),
	RTD1195_CRT_FUNC(i2c2),
	RTD1195_CRT_FUNC(i2c3),
	RTD1195_CRT_FUNC(i2c4),
	RTD1195_CRT_FUNC(i2c5),
	RTD1195_CRT_FUNC(mmc),
	RTD1195_CRT_FUNC(nand),
	RTD1195_CRT_FUNC(scpu_ejtag_loc_cr),
	RTD1195_CRT_FUNC(scpu_ejtag_loc_misc),
	RTD1195_CRT_FUNC(sdio),
	RTD1195_CRT_FUNC(sensor),
	RTD1195_CRT_FUNC(spdif),
	RTD1195_CRT_FUNC(spi),
	RTD1195_CRT_FUNC(tp0),
	RTD1195_CRT_FUNC(tp1),
	RTD1195_CRT_FUNC(uart1_loc_misc),
	RTD1195_CRT_FUNC(usb),
};

static const struct rtd_pin_desc rtd1195_crt_muxes[ARRAY_SIZE(rtd1195_crt_pin_desc)] = {
	[RTD1195_CRT_GPIO_0] = RTK_PIN_MUX(gpio_0, 0x370, GENMASK(2, 0),
		RTK_PIN_FUNC(0x1, "gpio"),
		RTK_PIN_FUNC(0x2, "uart1_loc_misc"),
		RTK_PIN_FUNC(0x3, "hif_loc_misc"),
		RTK_PIN_FUNC(0x4, "gspi")),
	[RTD1195_CRT_GPIO_1] = RTK_PIN_MUX(gpio_1, 0x370, GENMASK(5, 3),
		RTK_PIN_FUNC(0x8, "gpio"),
		RTK_PIN_FUNC(0x10, "uart1_loc_misc"),
		RTK_PIN_FUNC(0x18, "hif_loc_misc"),
		RTK_PIN_FUNC(0x20, "gspi")),
	[RTD1195_CRT_GPIO_2] = RTK_PIN_MUX(gpio_2, 0x370, GENMASK(8, 6),
		RTK_PIN_FUNC(0x40, "gpio"),
		RTK_PIN_FUNC(0x80, "uart1_loc_misc"),
		RTK_PIN_FUNC(0xc0, "hif_loc_misc"),
		RTK_PIN_FUNC(0x100, "gspi")),
	[RTD1195_CRT_GPIO_3] = RTK_PIN_MUX(gpio_3, 0x370, GENMASK(11, 9),
		RTK_PIN_FUNC(0x200, "gpio"),
		RTK_PIN_FUNC(0x400, "uart1_loc_misc"),
		RTK_PIN_FUNC(0x600, "hif_loc_misc"),
		RTK_PIN_FUNC(0x800, "gspi")),
	[RTD1195_CRT_GPIO_4] = RTK_PIN_MUX(gpio_4, 0x370, GENMASK(13, 12),
		RTK_PIN_FUNC(0x1000, "gpio"),
		RTK_PIN_FUNC(0x2000, "scpu_ejtag_loc_misc"),
		RTK_PIN_FUNC(0x3000, "ao_loc_misc")),
	[RTD1195_CRT_GPIO_5] = RTK_PIN_MUX(gpio_5, 0x370, GENMASK(15, 14),
		RTK_PIN_FUNC(0x4000, "gpio"),
		RTK_PIN_FUNC(0x8000, "scpu_ejtag_loc_misc"),
		RTK_PIN_FUNC(0xc000, "ao_loc_misc")),
	[RTD1195_CRT_GPIO_6] = RTK_PIN_MUX(gpio_6, 0x370, GENMASK(17, 16),
		RTK_PIN_FUNC(0x10000, "gpio"),
		RTK_PIN_FUNC(0x20000, "scpu_ejtag_loc_misc"),
		RTK_PIN_FUNC(0x30000, "ao_loc_misc")),
	[RTD1195_CRT_GPIO_7] = RTK_PIN_MUX(gpio_7, 0x370, GENMASK(19, 18),
		RTK_PIN_FUNC(0x40000, "gpio"),
		RTK_PIN_FUNC(0x80000, "scpu_ejtag_loc_misc"),
		RTK_PIN_FUNC(0xc0000, "ao_loc_misc")),
	[RTD1195_CRT_GPIO_8] = RTK_PIN_MUX(gpio_8, 0x370, GENMASK(21, 20),
		RTK_PIN_FUNC(0x100000, "gpio"),
		RTK_PIN_FUNC(0x200000, "scpu_ejtag_loc_misc")),
	[RTD1195_CRT_NF_DD_0] = RTK_PIN_MUX(nf_dd_0, 0x360, GENMASK(17, 16),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x10000, "nand"),
		RTK_PIN_FUNC(0x20000, "emmc")),
	[RTD1195_CRT_NF_DD_1] = RTK_PIN_MUX(nf_dd_1, 0x360, GENMASK(19, 18),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x40000, "nand"),
		RTK_PIN_FUNC(0x80000, "emmc")),
	[RTD1195_CRT_NF_DD_2] = RTK_PIN_MUX(nf_dd_2, 0x360, GENMASK(21, 20),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x100000, "nand"),
		RTK_PIN_FUNC(0x200000, "emmc")),
	[RTD1195_CRT_NF_DD_3] = RTK_PIN_MUX(nf_dd_3, 0x360, GENMASK(23, 22),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x400000, "nand"),
		RTK_PIN_FUNC(0x800000, "emmc")),
	[RTD1195_CRT_NF_DD_4] = RTK_PIN_MUX(nf_dd_4, 0x360, GENMASK(25, 24),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x1000000, "nand"),
		RTK_PIN_FUNC(0x2000000, "emmc"),
		RTK_PIN_FUNC(0x3000000, "hif_loc_nf")),
	[RTD1195_CRT_NF_DD_5] = RTK_PIN_MUX(nf_dd_5, 0x360, GENMASK(27, 26),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x4000000, "nand"),
		RTK_PIN_FUNC(0x8000000, "emmc"),
		RTK_PIN_FUNC(0xc000000, "avcpu_ejtag_loc_nf")),
	[RTD1195_CRT_NF_DD_6] = RTK_PIN_MUX(nf_dd_6, 0x360, GENMASK(29, 28),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x10000000, "nand"),
		RTK_PIN_FUNC(0x20000000, "emmc"),
		RTK_PIN_FUNC(0x30000000, "avcpu_ejtag_loc_nf")),
	[RTD1195_CRT_NF_DD_7] = RTK_PIN_MUX(nf_dd_7, 0x360, GENMASK(31, 30),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x40000000, "nand"),
		RTK_PIN_FUNC(0x80000000, "emmc"),
		RTK_PIN_FUNC(0xc0000000, "avcpu_ejtag_loc_nf")),
	[RTD1195_CRT_NF_RDY] = RTK_PIN_MUX(nf_rdy, 0x360, GENMASK(3, 2),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x4, "nand"),
		RTK_PIN_FUNC(0x8, "emmc"),
		RTK_PIN_FUNC(0xc, "avcpu_ejtag_loc_nf")),
	[RTD1195_CRT_NF_RD_N] = RTK_PIN_MUX(nf_rd_n, 0x360, GENMASK(5, 4),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x10, "nand"),
		RTK_PIN_FUNC(0x20, "emmc"),
		RTK_PIN_FUNC(0x30, "avcpu_ejtag_loc_nf")),
	[RTD1195_CRT_NF_WR_N] = RTK_PIN_MUX(nf_wr_n, 0x360, GENMASK(7, 6),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x40, "nand"),
		RTK_PIN_FUNC(0x80, "emmc"),
		RTK_PIN_FUNC(0xc0, "hif_loc_nf")),
	[RTD1195_CRT_NF_ALE] = RTK_PIN_MUX(nf_ale, 0x360, GENMASK(9, 8),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x100, "nand"),
		RTK_PIN_FUNC(0x200, "emmc"),
		RTK_PIN_FUNC(0x300, "hif_loc_nf")),
	[RTD1195_CRT_NF_CLE] = RTK_PIN_MUX(nf_cle, 0x360, GENMASK(11, 10),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x400, "nand"),
		RTK_PIN_FUNC(0x800, "emmc"),
		RTK_PIN_FUNC(0xc00, "hif_loc_nf")),
	[RTD1195_CRT_NF_CE_N_0] = RTK_PIN_MUX(nf_ce_n_0, 0x360, GENMASK(13, 12),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x1000, "nand")),
	[RTD1195_CRT_NF_CE_N_1] = RTK_PIN_MUX(nf_ce_n_1, 0x360, GENMASK(15, 14),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x4000, "nand")),
	[RTD1195_CRT_MMC_DATA_0] = RTK_PIN_MUX(mmc_data_0, 0x364, GENMASK(25, 24),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x1000000, "mmc"),
		RTK_PIN_FUNC(0x3000000, "scpu_ejtag_loc_cr")),
	[RTD1195_CRT_MMC_DATA_1] = RTK_PIN_MUX(mmc_data_1, 0x364, GENMASK(27, 26),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x4000000, "mmc")),
	[RTD1195_CRT_MMC_DATA_2] = RTK_PIN_MUX(mmc_data_2, 0x364, GENMASK(29, 28),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x10000000, "mmc")),
	[RTD1195_CRT_MMC_DATA_3] = RTK_PIN_MUX(mmc_data_3, 0x364, GENMASK(31, 30),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x40000000, "mmc"),
		RTK_PIN_FUNC(0xc0000000, "scpu_ejtag_loc_cr")),
	[RTD1195_CRT_MMC_CLK] = RTK_PIN_MUX(mmc_clk, 0x364, GENMASK(19, 18),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x40000, "mmc"),
		RTK_PIN_FUNC(0xc0000, "scpu_ejtag_loc_cr")),
	[RTD1195_CRT_MMC_CMD] = RTK_PIN_MUX(mmc_cmd, 0x364, GENMASK(17, 16),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x10000, "mmc"),
		RTK_PIN_FUNC(0x30000, "scpu_ejtag_loc_cr")),
	[RTD1195_CRT_MMC_WP] = RTK_PIN_MUX(mmc_wp, 0x364, GENMASK(21, 20),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x100000, "mmc"),
		RTK_PIN_FUNC(0x300000, "scpu_ejtag_loc_cr")),
	[RTD1195_CRT_MMC_CD] = RTK_PIN_MUX(mmc_cd, 0x364, GENMASK(23, 22),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x400000, "mmc")),
	[RTD1195_CRT_SDIO_CLK] = RTK_PIN_MUX(sdio_clk, 0x364, GENMASK(3, 2),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x4, "sdio")),
	[RTD1195_CRT_SDIO_DATA_0] = RTK_PIN_MUX(sdio_data_0, 0x364, GENMASK(5, 4),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x10, "sdio")),
	[RTD1195_CRT_SDIO_DATA_1] = RTK_PIN_MUX(sdio_data_1, 0x364, GENMASK(7, 6),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x40, "sdio")),
	[RTD1195_CRT_SDIO_DATA_2] = RTK_PIN_MUX(sdio_data_2, 0x364, GENMASK(9, 8),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x100, "sdio")),
	[RTD1195_CRT_SDIO_DATA_3] = RTK_PIN_MUX(sdio_data_3, 0x364, GENMASK(11, 10),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x400, "sdio")),
	[RTD1195_CRT_SDIO_CMD] = RTK_PIN_MUX(sdio_cmd, 0x364, GENMASK(1, 0),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x1, "sdio")),
	[RTD1195_CRT_I2C_SCL_5] = RTK_PIN_MUX(i2c_scl_5, 0x36c, GENMASK(11, 10),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x400, "i2c5"),
		RTK_PIN_FUNC(0xc00, "nand")),
	[RTD1195_CRT_I2C_SDA_5] = RTK_PIN_MUX(i2c_sda_5, 0x36c, GENMASK(9, 8),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x100, "i2c5"),
		RTK_PIN_FUNC(0x300, "nand")),
	[RTD1195_CRT_TP1_DATA] = RTK_PIN_MUX(tp1_data, 0x368, GENMASK(17, 16),
		RTK_PIN_FUNC(0x0, "tp1"),
		RTK_PIN_FUNC(0x10000, "tp0"),
		RTK_PIN_FUNC(0x20000, "gpio"),
		RTK_PIN_FUNC(0x30000, "i2c3")),
	[RTD1195_CRT_TP1_CLK] = RTK_PIN_MUX(tp1_clk, 0x368, GENMASK(23, 22),
		RTK_PIN_FUNC(0x0, "tp1"),
		RTK_PIN_FUNC(0x400000, "tp0"),
		RTK_PIN_FUNC(0x800000, "gpio"),
		RTK_PIN_FUNC(0xc00000, "i2c2")),
	[RTD1195_CRT_TP1_VALID] = RTK_PIN_MUX(tp1_valid, 0x368, GENMASK(21, 20),
		RTK_PIN_FUNC(0x0, "tp1"),
		RTK_PIN_FUNC(0x100000, "tp0"),
		RTK_PIN_FUNC(0x200000, "gpio"),
		RTK_PIN_FUNC(0x300000, "i2c3")),
	[RTD1195_CRT_TP1_SYNC] = RTK_PIN_MUX(tp1_sync, 0x368, GENMASK(19, 18),
		RTK_PIN_FUNC(0x0, "tp1"),
		RTK_PIN_FUNC(0x40000, "tp0"),
		RTK_PIN_FUNC(0x80000, "gpio"),
		RTK_PIN_FUNC(0xc0000, "i2c2")),
	[RTD1195_CRT_TP0_DATA] = RTK_PIN_MUX(tp0_data, 0x368, GENMASK(1, 0),
		RTK_PIN_FUNC(0x0, "tp0"),
		RTK_PIN_FUNC(0x1, "tp1"),
		RTK_PIN_FUNC(0x2, "gpio"),
		RTK_PIN_FUNC(0x3, "ao_loc_tp")),
	[RTD1195_CRT_TP0_CLK] = RTK_PIN_MUX(tp0_clk, 0x368, GENMASK(7, 6),
		RTK_PIN_FUNC(0x0, "tp0"),
		RTK_PIN_FUNC(0x40, "tp1"),
		RTK_PIN_FUNC(0x80, "gpio"),
		RTK_PIN_FUNC(0xc0, "ao_loc_tp")),
	[RTD1195_CRT_TP0_VALID] = RTK_PIN_MUX(tp0_valid, 0x368, GENMASK(5, 4),
		RTK_PIN_FUNC(0x0, "tp0"),
		RTK_PIN_FUNC(0x10, "tp1"),
		RTK_PIN_FUNC(0x20, "gpio"),
		RTK_PIN_FUNC(0x30, "ao_loc_tp")),
	[RTD1195_CRT_TP0_SYNC] = RTK_PIN_MUX(tp0_sync, 0x368, GENMASK(3, 2),
		RTK_PIN_FUNC(0x0, "tp0"),
		RTK_PIN_FUNC(0x4, "tp1"),
		RTK_PIN_FUNC(0x8, "gpio"),
		RTK_PIN_FUNC(0xc, "ao_loc_tp")),
	[RTD1195_CRT_USB_ID] = RTK_PIN_MUX(usb_id, 0x36c, GENMASK(17, 16),
		RTK_PIN_FUNC(0x10000, "gpio"),
		RTK_PIN_FUNC(0x20000, "cpu_loop")),
	[RTD1195_CRT_HDMI_HPD] = RTK_PIN_MUX(hdmi_hpd, 0x36c, GENMASK(15, 14),
		RTK_PIN_FUNC(0x4000, "gpio")),
	[RTD1195_CRT_SPDIF] = RTK_PIN_MUX(spdif, 0x36c, GENMASK(13, 12),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x1000, "spdif")),
	[RTD1195_CRT_I2C_SCL_1] = RTK_PIN_MUX(i2c_scl_1, 0x36c, GENMASK(3, 2),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x4, "i2c1")),
	[RTD1195_CRT_I2C_SDA_1] = RTK_PIN_MUX(i2c_sda_1, 0x36c, GENMASK(1, 0),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x1, "i2c1")),
	[RTD1195_CRT_I2C_SCL_4] = RTK_PIN_MUX(i2c_scl_4, 0x36c, GENMASK(7, 6),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x40, "i2c4")),
	[RTD1195_CRT_I2C_SDA_4] = RTK_PIN_MUX(i2c_sda_4, 0x36c, GENMASK(5, 4),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x10, "i2c4")),
	[RTD1195_CRT_SENSOR_CKO_0] = RTK_PIN_MUX(sensor_cko_0, 0x36c, GENMASK(31, 30),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x40000000, "sensor")),
	[RTD1195_CRT_SENSOR_CKO_1] = RTK_PIN_MUX(sensor_cko_1, 0x36c, GENMASK(29, 28),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x10000000, "sensor"),
		RTK_PIN_FUNC(0x30000000, "usb")),
	[RTD1195_CRT_SENSOR_RST] = RTK_PIN_MUX(sensor_rst, 0x36c, GENMASK(27, 26),
		RTK_PIN_FUNC(0x0, "gpio")),
	[RTD1195_CRT_SENSOR_STB_0] = RTK_PIN_MUX(sensor_stb_0, 0x36c, GENMASK(25, 24),
		RTK_PIN_FUNC(0x0, "gpio")),
	[RTD1195_CRT_SENSOR_STB_1] = RTK_PIN_MUX(sensor_stb_1, 0x36c, GENMASK(23, 22),
		RTK_PIN_FUNC(0x0, "gpio")),
	[RTD1195_CRT_HI_LOC] = RTK_PIN_MUX(hi_loc, 0x36c, GENMASK(19, 18),
		RTK_PIN_FUNC(0x40000, "hif_loc_misc"),
		RTK_PIN_FUNC(0x80000, "hif_loc_nf")),
	[RTD1195_CRT_EJTAG_SCPU_LOC] = RTK_PIN_MUX(ejtag_scpu_loc, 0x36c, GENMASK(21, 20),
		RTK_PIN_FUNC(0x100000, "scpu_ejtag_loc_misc"),
		RTK_PIN_FUNC(0x200000, "scpu_ejtag_loc_cr")),
	[RTD1195_CRT_SF_EN] = RTK_PIN_MUX(sf_en, 0x374, GENMASK(0, 0),
		RTK_PIN_FUNC(0x0, "uart1_loc_misc"),
		RTK_PIN_FUNC(0x0, "hif_loc_misc"),
		RTK_PIN_FUNC(0x0, "gspi"),
		RTK_PIN_FUNC(0x1, "spi")),
	{ },
};

static const struct rtd_pin_config_desc rtd1195_crt_configs[ARRAY_SIZE(rtd1195_crt_pin_desc)] = {
	[RTD1195_CRT_GPIO_0] = RTK_PIN_CONFIG(gpio_0, 0x398, 0, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_GPIO_1] = RTK_PIN_CONFIG(gpio_1, 0x398, 4, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_GPIO_2] = RTK_PIN_CONFIG(gpio_2, 0x398, 8, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_GPIO_3] = RTK_PIN_CONFIG(gpio_3, 0x398, 12, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_GPIO_4] = RTK_PIN_CONFIG(gpio_4, 0x398, 16, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_GPIO_5] = RTK_PIN_CONFIG(gpio_5, 0x398, 20, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_GPIO_6] = RTK_PIN_CONFIG(gpio_6, 0x398, 24, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_GPIO_7] = RTK_PIN_CONFIG(gpio_7, 0x398, 28, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_GPIO_8] = RTK_PIN_CONFIG(gpio_8, 0x39c, 0, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_NF_DD_0] = RTK_PIN_CONFIG(nf_dd_0, 0x37c, 0, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_NF_DD_1] = RTK_PIN_CONFIG(nf_dd_1, 0x37c, 4, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_NF_DD_2] = RTK_PIN_CONFIG(nf_dd_2, 0x37c, 8, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_NF_DD_3] = RTK_PIN_CONFIG(nf_dd_3, 0x37c, 12, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_NF_DD_4] = RTK_PIN_CONFIG(nf_dd_4, 0x37c, 16, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_NF_DD_5] = RTK_PIN_CONFIG(nf_dd_5, 0x37c, 20, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_NF_DD_6] = RTK_PIN_CONFIG(nf_dd_6, 0x37c, 24, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_NF_DD_7] = RTK_PIN_CONFIG(nf_dd_7, 0x37c, 28, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_NF_RDY] = RTK_PIN_CONFIG(nf_rdy, 0x378, 16, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_NF_RD_N] = RTK_PIN_CONFIG(nf_rd_n, 0x378, 20, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_NF_WR_N] = RTK_PIN_CONFIG(nf_wr_n, 0x378, 24, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_NF_ALE] = RTK_PIN_CONFIG(nf_ale, 0x378, 4, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_NF_CLE] = RTK_PIN_CONFIG(nf_cle, 0x378, 28, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_NF_CE_N_0] = RTK_PIN_CONFIG(nf_ce_n_0, 0x378, 8, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_NF_CE_N_1] = RTK_PIN_CONFIG(nf_ce_n_1, 0x378, 12, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_MMC_DATA_0] = RTK_PIN_CONFIG(mmc_data_0, 0x380, 16, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_MMC_DATA_1] = RTK_PIN_CONFIG(mmc_data_1, 0x380, 20, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_MMC_DATA_2] = RTK_PIN_CONFIG(mmc_data_2, 0x380, 24, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_MMC_DATA_3] = RTK_PIN_CONFIG(mmc_data_3, 0x380, 28, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_MMC_CLK] = RTK_PIN_CONFIG(mmc_clk, 0x380, 4, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_MMC_CMD] = RTK_PIN_CONFIG(mmc_cmd, 0x380, 0, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_MMC_WP] = RTK_PIN_CONFIG(mmc_wp, 0x380, 8, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_MMC_CD] = RTK_PIN_CONFIG(mmc_cd, 0x380, 12, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_SDIO_CLK] = RTK_PIN_CONFIG(sdio_clk, 0x384, 4, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_SDIO_DATA_0] = RTK_PIN_CONFIG(sdio_data_0, 0x384, 16, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_SDIO_DATA_1] = RTK_PIN_CONFIG(sdio_data_1, 0x384, 20, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_SDIO_DATA_2] = RTK_PIN_CONFIG(sdio_data_2, 0x384, 24, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_SDIO_DATA_3] = RTK_PIN_CONFIG(sdio_data_3, 0x384, 28, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_SDIO_CMD] = RTK_PIN_CONFIG(sdio_cmd, 0x384, 0, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_I2C_SCL_5] = RTK_PIN_CONFIG(i2c_scl_5, 0x394, 20, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_I2C_SDA_5] = RTK_PIN_CONFIG(i2c_sda_5, 0x394, 16, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_TP1_DATA] = RTK_PIN_CONFIG(tp1_data, 0x390, 12, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_TP1_CLK] = RTK_PIN_CONFIG(tp1_clk, 0x390, 0, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_TP1_VALID] = RTK_PIN_CONFIG(tp1_valid, 0x390, 8, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_TP1_SYNC] = RTK_PIN_CONFIG(tp1_sync, 0x390, 4, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_TP0_DATA] = RTK_PIN_CONFIG(tp0_data, 0x38c, 12, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_TP0_CLK] = RTK_PIN_CONFIG(tp0_clk, 0x38c, 0, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_TP0_VALID] = RTK_PIN_CONFIG(tp0_valid, 0x38c, 8, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_TP0_SYNC] = RTK_PIN_CONFIG(tp0_sync, 0x38c, 4, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_USB_ID] = RTK_PIN_CONFIG(usb_id, 0x39c, 8, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_HDMI_HPD] = RTK_PIN_CONFIG(hdmi_hpd, 0x39c, 4, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_SPDIF] = RTK_PIN_CONFIG(spdif, 0x388, 0, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_I2C_SCL_1] = RTK_PIN_CONFIG(i2c_scl_1, 0x394, 4, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_I2C_SDA_1] = RTK_PIN_CONFIG(i2c_sda_1, 0x394, 0, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_I2C_SCL_4] = RTK_PIN_CONFIG(i2c_scl_4, 0x394, 12, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_I2C_SDA_4] = RTK_PIN_CONFIG(i2c_sda_4, 0x394, 8, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_CRT_SENSOR_CKO_0] = RTK_PIN_CONFIG(sensor_cko_0, 0x39c, 24, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_SENSOR_CKO_1] = RTK_PIN_CONFIG(sensor_cko_1, 0x39c, 28, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_SENSOR_RST] = RTK_PIN_CONFIG(sensor_rst, 0x39c, 20, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_SENSOR_STB_0] = RTK_PIN_CONFIG(sensor_stb_0, 0x39c, 12, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_CRT_SENSOR_STB_1] = RTK_PIN_CONFIG(sensor_stb_1, 0x39c, 16, 1, 0, 2, 3, NA, PADDRI_2_4),
	{ },
	{ },
	{ },
	{ },
};

static const struct rtd_pinctrl_desc rtd1195_crt_pinctrl_desc = {
	.pins = rtd1195_crt_pin_desc,
	.num_pins = ARRAY_SIZE(rtd1195_crt_pin_desc),
	.groups = rtd1195_crt_pin_groups,
	.num_groups = ARRAY_SIZE(rtd1195_crt_pin_groups),
	.functions = rtd1195_crt_pin_functions,
	.num_functions = ARRAY_SIZE(rtd1195_crt_pin_functions),
	.muxes = rtd1195_crt_muxes,
	.num_muxes = ARRAY_SIZE(rtd1195_crt_muxes),
	.configs = rtd1195_crt_configs,
	.num_configs = ARRAY_SIZE(rtd1195_crt_configs),
};

enum rtd1195_iso_pins {
	RTD1195_ISO_GPIO_0 = 0,
	RTD1195_ISO_GPIO_1 = 1,
	RTD1195_ISO_USB0 = 2,
	RTD1195_ISO_USB1 = 3,
	RTD1195_ISO_VFD_CS_N = 4,
	RTD1195_ISO_VFD_CLK = 5,
	RTD1195_ISO_VFD_D = 6,
	RTD1195_ISO_IR_RX = 7,
	RTD1195_ISO_IR_TX = 8,
	RTD1195_ISO_UR0_RX = 9,
	RTD1195_ISO_UR0_TX = 10,
	RTD1195_ISO_UR1_RX = 11,
	RTD1195_ISO_UR1_TX = 12,
	RTD1195_ISO_UR1_CTS_N = 13,
	RTD1195_ISO_UR1_RTS_N = 14,
	RTD1195_ISO_I2C_SCL_0 = 15,
	RTD1195_ISO_I2C_SDA_0 = 16,
	RTD1195_ISO_ETN_LED_LINK = 17,
	RTD1195_ISO_ETN_LED_RXTX = 18,
	RTD1195_ISO_I2C_SCL_6 = 19,
	RTD1195_ISO_I2C_SDA_6 = 20,
	RTD1195_ISO_PWM_23_OPEN_DRAIN_SWITCH = 21,
	RTD1195_ISO_PWM_01_OPEN_DRAIN_SWITCH = 22,
	RTD1195_ISO_UR1_LOC = 23,
	RTD1195_ISO_EJTAG_AVCPU_LOC = 24,
	RTD1195_ISO_AI_LOC = 25,
};

static const struct pinctrl_pin_desc rtd1195_iso_pin_desc[] = {
	PINCTRL_PIN(RTD1195_ISO_GPIO_0, "gpio_0"),
	PINCTRL_PIN(RTD1195_ISO_GPIO_1, "gpio_1"),
	PINCTRL_PIN(RTD1195_ISO_USB0, "usb0"),
	PINCTRL_PIN(RTD1195_ISO_USB1, "usb1"),
	PINCTRL_PIN(RTD1195_ISO_VFD_CS_N, "vfd_cs_n"),
	PINCTRL_PIN(RTD1195_ISO_VFD_CLK, "vfd_clk"),
	PINCTRL_PIN(RTD1195_ISO_VFD_D, "vfd_d"),
	PINCTRL_PIN(RTD1195_ISO_IR_RX, "ir_rx"),
	PINCTRL_PIN(RTD1195_ISO_IR_TX, "ir_tx"),
	PINCTRL_PIN(RTD1195_ISO_UR0_RX, "ur0_rx"),
	PINCTRL_PIN(RTD1195_ISO_UR0_TX, "ur0_tx"),
	PINCTRL_PIN(RTD1195_ISO_UR1_RX, "ur1_rx"),
	PINCTRL_PIN(RTD1195_ISO_UR1_TX, "ur1_tx"),
	PINCTRL_PIN(RTD1195_ISO_UR1_CTS_N, "ur1_cts_n"),
	PINCTRL_PIN(RTD1195_ISO_UR1_RTS_N, "ur1_rts_n"),
	PINCTRL_PIN(RTD1195_ISO_I2C_SCL_0, "i2c_scl_0"),
	PINCTRL_PIN(RTD1195_ISO_I2C_SDA_0, "i2c_sda_0"),
	PINCTRL_PIN(RTD1195_ISO_ETN_LED_LINK, "etn_led_link"),
	PINCTRL_PIN(RTD1195_ISO_ETN_LED_RXTX, "etn_led_rxtx"),
	PINCTRL_PIN(RTD1195_ISO_I2C_SCL_6, "i2c_scl_6"),
	PINCTRL_PIN(RTD1195_ISO_I2C_SDA_6, "i2c_sda_6"),
	PINCTRL_PIN(RTD1195_ISO_PWM_23_OPEN_DRAIN_SWITCH, "pwm_23_open_drain_switch"),
	PINCTRL_PIN(RTD1195_ISO_PWM_01_OPEN_DRAIN_SWITCH, "pwm_01_open_drain_switch"),
	PINCTRL_PIN(RTD1195_ISO_UR1_LOC, "ur1_loc"),
	PINCTRL_PIN(RTD1195_ISO_EJTAG_AVCPU_LOC, "ejtag_avcpu_loc"),
	PINCTRL_PIN(RTD1195_ISO_AI_LOC, "ai_loc"),
};

static const unsigned int rtd1195_iso_gpio_0_pins[] = { RTD1195_ISO_GPIO_0 };
static const unsigned int rtd1195_iso_gpio_1_pins[] = { RTD1195_ISO_GPIO_1 };
static const unsigned int rtd1195_iso_usb0_pins[] = { RTD1195_ISO_USB0 };
static const unsigned int rtd1195_iso_usb1_pins[] = { RTD1195_ISO_USB1 };
static const unsigned int rtd1195_iso_vfd_cs_n_pins[] = { RTD1195_ISO_VFD_CS_N };
static const unsigned int rtd1195_iso_vfd_clk_pins[] = { RTD1195_ISO_VFD_CLK };
static const unsigned int rtd1195_iso_vfd_d_pins[] = { RTD1195_ISO_VFD_D };
static const unsigned int rtd1195_iso_ir_rx_pins[] = { RTD1195_ISO_IR_RX };
static const unsigned int rtd1195_iso_ir_tx_pins[] = { RTD1195_ISO_IR_TX };
static const unsigned int rtd1195_iso_ur0_rx_pins[] = { RTD1195_ISO_UR0_RX };
static const unsigned int rtd1195_iso_ur0_tx_pins[] = { RTD1195_ISO_UR0_TX };
static const unsigned int rtd1195_iso_ur1_rx_pins[] = { RTD1195_ISO_UR1_RX };
static const unsigned int rtd1195_iso_ur1_tx_pins[] = { RTD1195_ISO_UR1_TX };
static const unsigned int rtd1195_iso_ur1_cts_n_pins[] = { RTD1195_ISO_UR1_CTS_N };
static const unsigned int rtd1195_iso_ur1_rts_n_pins[] = { RTD1195_ISO_UR1_RTS_N };
static const unsigned int rtd1195_iso_i2c_scl_0_pins[] = { RTD1195_ISO_I2C_SCL_0 };
static const unsigned int rtd1195_iso_i2c_sda_0_pins[] = { RTD1195_ISO_I2C_SDA_0 };
static const unsigned int rtd1195_iso_etn_led_link_pins[] = { RTD1195_ISO_ETN_LED_LINK };
static const unsigned int rtd1195_iso_etn_led_rxtx_pins[] = { RTD1195_ISO_ETN_LED_RXTX };
static const unsigned int rtd1195_iso_i2c_scl_6_pins[] = { RTD1195_ISO_I2C_SCL_6 };
static const unsigned int rtd1195_iso_i2c_sda_6_pins[] = { RTD1195_ISO_I2C_SDA_6 };
static const unsigned int rtd1195_iso_pwm_23_open_drain_switch_pins[] = {
	RTD1195_ISO_PWM_23_OPEN_DRAIN_SWITCH };
static const unsigned int rtd1195_iso_pwm_01_open_drain_switch_pins[] = {
	RTD1195_ISO_PWM_01_OPEN_DRAIN_SWITCH };
static const unsigned int rtd1195_iso_ur1_loc_pins[] = { RTD1195_ISO_UR1_LOC };
static const unsigned int rtd1195_iso_ejtag_avcpu_loc_pins[] = { RTD1195_ISO_EJTAG_AVCPU_LOC };
static const unsigned int rtd1195_iso_ai_loc_pins[] = { RTD1195_ISO_AI_LOC };

#define RTD1195_ISO_GROUP(_name) \
	{ .name = # _name, .pins = rtd1195_iso_ ## _name ## _pins, \
	  .num_pins = ARRAY_SIZE(rtd1195_iso_ ## _name ## _pins) }

static const struct rtd_pin_group_desc rtd1195_iso_pin_groups[] = {
	RTD1195_ISO_GROUP(gpio_0),
	RTD1195_ISO_GROUP(gpio_1),
	RTD1195_ISO_GROUP(usb0),
	RTD1195_ISO_GROUP(usb1),
	RTD1195_ISO_GROUP(vfd_cs_n),
	RTD1195_ISO_GROUP(vfd_clk),
	RTD1195_ISO_GROUP(vfd_d),
	RTD1195_ISO_GROUP(ir_rx),
	RTD1195_ISO_GROUP(ir_tx),
	RTD1195_ISO_GROUP(ur0_rx),
	RTD1195_ISO_GROUP(ur0_tx),
	RTD1195_ISO_GROUP(ur1_rx),
	RTD1195_ISO_GROUP(ur1_tx),
	RTD1195_ISO_GROUP(ur1_cts_n),
	RTD1195_ISO_GROUP(ur1_rts_n),
	RTD1195_ISO_GROUP(i2c_scl_0),
	RTD1195_ISO_GROUP(i2c_sda_0),
	RTD1195_ISO_GROUP(etn_led_link),
	RTD1195_ISO_GROUP(etn_led_rxtx),
	RTD1195_ISO_GROUP(i2c_scl_6),
	RTD1195_ISO_GROUP(i2c_sda_6),
	RTD1195_ISO_GROUP(pwm_23_open_drain_switch),
	RTD1195_ISO_GROUP(pwm_01_open_drain_switch),
	RTD1195_ISO_GROUP(ur1_loc),
	RTD1195_ISO_GROUP(ejtag_avcpu_loc),
	RTD1195_ISO_GROUP(ai_loc),
};

static const char * const rtd1195_iso_ai_loc_iso_ur_groups[] = {
	"ur1_rx", "ur1_tx", "ur1_cts_n", "ur1_rts_n", "ai_loc" };
static const char * const rtd1195_iso_ai_loc_iso_usb_groups[] = {
	"usb0", "usb1", "vfd_cs_n", "vfd_clk", "ai_loc" };
static const char * const rtd1195_iso_avcpu_ejtag_loc_iso_groups[] = {
	"usb0", "usb1", "vfd_cs_n", "vfd_clk", "vfd_d", "ejtag_avcpu_loc" };
static const char * const rtd1195_iso_avcpu_ejtag_loc_nf_groups[] = {
	"ejtag_avcpu_loc" };
static const char * const rtd1195_iso_etn_led_groups[] = {
	"etn_led_link", "etn_led_rxtx" };
static const char * const rtd1195_iso_gpio_groups[] = {
	"gpio_1", "usb0", "usb1", "vfd_cs_n", "vfd_clk", "vfd_d", "ir_rx", "ir_tx", "ur0_rx", "ur0_tx",
	"ur1_rx", "ur1_tx", "ur1_cts_n", "ur1_rts_n", "i2c_scl_0", "i2c_sda_0", "etn_led_link",
	"etn_led_rxtx", "i2c_scl_6", "i2c_sda_6" };
static const char * const rtd1195_iso_i2c0_groups[] = {
	"i2c_scl_0", "i2c_sda_0" };
static const char * const rtd1195_iso_i2c2_groups[] = {
	"vfd_d" };
static const char * const rtd1195_iso_i2c3_groups[] = {
	"ir_tx" };
static const char * const rtd1195_iso_i2c6_groups[] = {
	"i2c_scl_6", "i2c_sda_6" };
static const char * const rtd1195_iso_irrx_groups[] = {
	"ir_rx" };
static const char * const rtd1195_iso_irtx_groups[] = {
	"ir_tx" };
static const char * const rtd1195_iso_pwm_groups[] = {
	"ur0_rx", "ur0_tx", "ur1_rx", "ur1_tx", "etn_led_link", "etn_led_rxtx" };
static const char * const rtd1195_iso_standby_dbg_groups[] = {
	"usb0", "usb1", "ir_rx" };
static const char * const rtd1195_iso_uart0_groups[] = {
	"ur0_rx", "ur0_tx" };
static const char * const rtd1195_iso_uart1_loc_iso_groups[] = {
	"ur1_rx", "ur1_tx", "ur1_cts_n", "ur1_rts_n", "ur1_loc" };
static const char * const rtd1195_iso_uart1_loc_misc_groups[] = {
	"ur1_loc" };
static const char * const rtd1195_iso_vfd_groups[] = {
	"vfd_cs_n", "vfd_clk", "vfd_d" };

#define RTD1195_ISO_FUNC(_name) \
	{ .name = # _name, .groups = rtd1195_iso_ ## _name ## _groups, \
	  .num_groups = ARRAY_SIZE(rtd1195_iso_ ## _name ## _groups) }

static const struct rtd_pin_func_desc rtd1195_iso_pin_functions[] = {
	RTD1195_ISO_FUNC(ai_loc_iso_ur),
	RTD1195_ISO_FUNC(ai_loc_iso_usb),
	RTD1195_ISO_FUNC(avcpu_ejtag_loc_iso),
	RTD1195_ISO_FUNC(avcpu_ejtag_loc_nf),
	RTD1195_ISO_FUNC(etn_led),
	RTD1195_ISO_FUNC(gpio),
	RTD1195_ISO_FUNC(i2c0),
	RTD1195_ISO_FUNC(i2c2),
	RTD1195_ISO_FUNC(i2c3),
	RTD1195_ISO_FUNC(i2c6),
	RTD1195_ISO_FUNC(irrx),
	RTD1195_ISO_FUNC(irtx),
	RTD1195_ISO_FUNC(pwm),
	RTD1195_ISO_FUNC(standby_dbg),
	RTD1195_ISO_FUNC(uart0),
	RTD1195_ISO_FUNC(uart1_loc_iso),
	RTD1195_ISO_FUNC(uart1_loc_misc),
	RTD1195_ISO_FUNC(vfd),
};

static const struct rtd_pin_desc rtd1195_iso_muxes[ARRAY_SIZE(rtd1195_iso_pin_desc)] = {
	{ },
	{ },
	[RTD1195_ISO_USB0] = RTK_PIN_MUX(usb0, 0x310, GENMASK(9, 8),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x100, "standby_dbg"),
		RTK_PIN_FUNC(0x200, "ai_loc_iso_usb"),
		RTK_PIN_FUNC(0x300, "avcpu_ejtag_loc_iso")),
	[RTD1195_ISO_USB1] = RTK_PIN_MUX(usb1, 0x310, GENMASK(11, 10),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x400, "standby_dbg"),
		RTK_PIN_FUNC(0x800, "ai_loc_iso_usb"),
		RTK_PIN_FUNC(0xc00, "avcpu_ejtag_loc_iso")),
	[RTD1195_ISO_VFD_CS_N] = RTK_PIN_MUX(vfd_cs_n, 0x310, GENMASK(5, 4),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x10, "vfd"),
		RTK_PIN_FUNC(0x20, "ai_loc_iso_usb"),
		RTK_PIN_FUNC(0x30, "avcpu_ejtag_loc_iso")),
	[RTD1195_ISO_VFD_CLK] = RTK_PIN_MUX(vfd_clk, 0x310, GENMASK(3, 2),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x4, "vfd"),
		RTK_PIN_FUNC(0x8, "ai_loc_iso_usb"),
		RTK_PIN_FUNC(0xc, "avcpu_ejtag_loc_iso")),
	[RTD1195_ISO_VFD_D] = RTK_PIN_MUX(vfd_d, 0x310, GENMASK(1, 0),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x1, "vfd"),
		RTK_PIN_FUNC(0x2, "i2c2"),
		RTK_PIN_FUNC(0x3, "avcpu_ejtag_loc_iso")),
	[RTD1195_ISO_IR_RX] = RTK_PIN_MUX(ir_rx, 0x310, GENMASK(7, 6),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x40, "irrx"),
		RTK_PIN_FUNC(0x80, "standby_dbg")),
	[RTD1195_ISO_IR_TX] = RTK_PIN_MUX(ir_tx, 0x314, GENMASK(5, 4),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x10, "irtx"),
		RTK_PIN_FUNC(0x20, "i2c3")),
	[RTD1195_ISO_UR0_RX] = RTK_PIN_MUX(ur0_rx, 0x310, GENMASK(21, 20),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x100000, "uart0"),
		RTK_PIN_FUNC(0x200000, "pwm")),
	[RTD1195_ISO_UR0_TX] = RTK_PIN_MUX(ur0_tx, 0x310, GENMASK(23, 22),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x400000, "uart0"),
		RTK_PIN_FUNC(0x800000, "pwm")),
	[RTD1195_ISO_UR1_RX] = RTK_PIN_MUX(ur1_rx, 0x310, GENMASK(13, 12),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x1000, "uart1_loc_iso"),
		RTK_PIN_FUNC(0x2000, "ai_loc_iso_ur"),
		RTK_PIN_FUNC(0x3000, "pwm")),
	[RTD1195_ISO_UR1_TX] = RTK_PIN_MUX(ur1_tx, 0x310, GENMASK(15, 14),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x4000, "uart1_loc_iso"),
		RTK_PIN_FUNC(0x8000, "ai_loc_iso_ur"),
		RTK_PIN_FUNC(0xc000, "pwm")),
	[RTD1195_ISO_UR1_LOC] = RTK_PIN_MUX(ur1_loc, 0x314, GENMASK(27, 26),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x4000000, "uart1_loc_iso"),
		RTK_PIN_FUNC(0x8000000, "uart1_loc_misc")),
	[RTD1195_ISO_UR1_CTS_N] = RTK_PIN_MUX(ur1_cts_n, 0x310, GENMASK(19, 18),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x40000, "uart1_loc_iso"),
		RTK_PIN_FUNC(0x80000, "ai_loc_iso_ur")),
	[RTD1195_ISO_UR1_RTS_N] = RTK_PIN_MUX(ur1_rts_n, 0x310, GENMASK(17, 16),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x10000, "uart1_loc_iso"),
		RTK_PIN_FUNC(0x20000, "ai_loc_iso_ur")),
	[RTD1195_ISO_I2C_SCL_0] = RTK_PIN_MUX(i2c_scl_0, 0x310, GENMASK(25, 24),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x1000000, "i2c0")),
	[RTD1195_ISO_I2C_SDA_0] = RTK_PIN_MUX(i2c_sda_0, 0x310, GENMASK(27, 26),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x4000000, "i2c0")),
	[RTD1195_ISO_ETN_LED_LINK] = RTK_PIN_MUX(etn_led_link, 0x310, GENMASK(29, 28),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x10000000, "etn_led"),
		RTK_PIN_FUNC(0x20000000, "pwm")),
	[RTD1195_ISO_ETN_LED_RXTX] = RTK_PIN_MUX(etn_led_rxtx, 0x310, GENMASK(31, 30),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x40000000, "etn_led"),
		RTK_PIN_FUNC(0x80000000, "pwm")),
	[RTD1195_ISO_I2C_SCL_6] = RTK_PIN_MUX(i2c_scl_6, 0x314, GENMASK(1, 0),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x1, "i2c6")),
	[RTD1195_ISO_I2C_SDA_6] = RTK_PIN_MUX(i2c_sda_6, 0x314, GENMASK(3, 2),
		RTK_PIN_FUNC(0x0, "gpio"),
		RTK_PIN_FUNC(0x4, "i2c6")),
	{ },
	{ },
	[RTD1195_ISO_EJTAG_AVCPU_LOC] = RTK_PIN_MUX(ejtag_avcpu_loc, 0x314, GENMASK(29, 28),
		RTK_PIN_FUNC(0x10000000, "avcpu_ejtag_loc_iso"),
		RTK_PIN_FUNC(0x20000000, "avcpu_ejtag_loc_nf")),
	[RTD1195_ISO_AI_LOC] = RTK_PIN_MUX(ai_loc, 0x314, GENMASK(31, 30),
		RTK_PIN_FUNC(0x40000000, "ai_loc_iso_ur"),
		RTK_PIN_FUNC(0x80000000, "ai_loc_iso_usb")),
};

static const struct rtd_pin_config_desc rtd1195_iso_configs[ARRAY_SIZE(rtd1195_iso_pin_desc)] = {
	{ },
	{ },
	[RTD1195_ISO_USB0] = RTK_PIN_CONFIG(usb0, 0x300, 0, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_ISO_USB1] = RTK_PIN_CONFIG(usb1, 0x300, 4, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_ISO_VFD_CS_N] = RTK_PIN_CONFIG(vfd_cs_n, 0x300, 12, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_ISO_VFD_CLK] = RTK_PIN_CONFIG(vfd_clk, 0x300, 16, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_ISO_VFD_D] = RTK_PIN_CONFIG(vfd_d, 0x300, 20, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_ISO_IR_RX] = RTK_PIN_CONFIG(ir_rx, 0x300, 8, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_ISO_IR_TX] = RTK_PIN_CONFIG(ir_tx, 0x308, 20, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_ISO_UR0_RX] = RTK_PIN_CONFIG(ur0_rx, 0x304, 16, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_ISO_UR0_TX] = RTK_PIN_CONFIG(ur0_tx, 0x304, 20, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_ISO_UR1_RX] = RTK_PIN_CONFIG(ur1_rx, 0x304, 0, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_ISO_UR1_TX] = RTK_PIN_CONFIG(ur1_tx, 0x304, 4, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_ISO_UR1_CTS_N] = RTK_PIN_CONFIG(ur1_cts_n, 0x304, 12, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_ISO_UR1_RTS_N] = RTK_PIN_CONFIG(ur1_rts_n, 0x304, 8, 1, 0, 2, 3, NA, PADDRI_2_4),
	[RTD1195_ISO_I2C_SCL_0] = RTK_PIN_CONFIG(i2c_scl_0, 0x300, 28, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_ISO_I2C_SDA_0] = RTK_PIN_CONFIG(i2c_sda_0, 0x300, 24, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_ISO_ETN_LED_LINK] = RTK_PIN_CONFIG(etn_led_link, 0x304, 24, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_ISO_ETN_LED_RXTX] = RTK_PIN_CONFIG(etn_led_rxtx, 0x304, 28, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_ISO_I2C_SCL_6] = RTK_PIN_CONFIG(i2c_scl_6, 0x308, 16, 1, 0, 2, 3, NA, PADDRI_4_8),
	[RTD1195_ISO_I2C_SDA_6] = RTK_PIN_CONFIG(i2c_sda_6, 0x308, 12, 1, 0, 2, 3, NA, PADDRI_4_8),
	{ },
	{ },
	{ },
	{ },
	{ },
};

static const struct rtd_pinctrl_desc rtd1195_iso_pinctrl_desc = {
	.pins = rtd1195_iso_pin_desc,
	.num_pins = ARRAY_SIZE(rtd1195_iso_pin_desc),
	.groups = rtd1195_iso_pin_groups,
	.num_groups = ARRAY_SIZE(rtd1195_iso_pin_groups),
	.functions = rtd1195_iso_pin_functions,
	.num_functions = ARRAY_SIZE(rtd1195_iso_pin_functions),
	.muxes = rtd1195_iso_muxes,
	.num_muxes = ARRAY_SIZE(rtd1195_iso_muxes),
	.configs = rtd1195_iso_configs,
	.num_configs = ARRAY_SIZE(rtd1195_iso_configs),
};

static int rtd1195_pinctrl_probe(struct platform_device *pdev)
{
	const struct rtd_pinctrl_desc *desc;

	desc = device_get_match_data(&pdev->dev);
	if (!desc)
		return -EINVAL;

	return rtd_pinctrl_probe(pdev, desc);
}

static const struct of_device_id rtd1195_pinctrl_of_match[] = {
	{ .compatible = "realtek,rtd1195-crt-pinctrl", .data = &rtd1195_crt_pinctrl_desc },
	{ .compatible = "realtek,rtd1195-iso-pinctrl", .data = &rtd1195_iso_pinctrl_desc },
	{ }
};

static struct platform_driver rtd1195_pinctrl_driver = {
	.driver = {
		.name = "rtd1195-pinctrl",
		.of_match_table = rtd1195_pinctrl_of_match,
	},
	.probe = rtd1195_pinctrl_probe,
};

static int __init rtd1195_pinctrl_init(void)
{
	return platform_driver_register(&rtd1195_pinctrl_driver);
}
arch_initcall(rtd1195_pinctrl_init);

static void __exit rtd1195_pinctrl_exit(void)
{
	platform_driver_unregister(&rtd1195_pinctrl_driver);
}
module_exit(rtd1195_pinctrl_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Realtek RTD1195 pinctrl driver");
