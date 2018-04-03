# SPDX-License-Identifier: GPL-2.0

nandcore-objs := core.o bbt.o
obj-$(CONFIG_MTD_NAND_CORE) += nandcore.o
obj-$(CONFIG_MTD_NAND_EP501X)		+= axxia_acp_nand.o
obj-$(CONFIG_MTD_NAND_EP501X_UBOOTENV)	+= axxia_ubootenv.o

obj-y	+= onenand/
obj-y	+= raw/
