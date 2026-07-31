# SPDX-License-Identifier: GPL-2.0

nandcore-objs := core.o bbt.o
obj-$(CONFIG_MTD_NAND_CORE) += nandcore.o

obj-y	+= onenand/
obj-y	+= raw/

nand-objs := rtk_nand.o nand_base_rtk.o
obj-y	+= spi/

nandcore-$(CONFIG_MTD_NAND_ECC) += ecc.o
nandcore-$(CONFIG_MTD_NAND_ECC_SW_HAMMING) += ecc-sw-hamming.o
nandcore-$(CONFIG_MTD_NAND_ECC_SW_BCH) += ecc-sw-bch.o
