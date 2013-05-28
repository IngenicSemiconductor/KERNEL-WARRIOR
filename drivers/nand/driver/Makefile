# drivers/nand/driver/Makefile 
#

KBUILD_CFLAGS += -I$(srctree)/drivers/nand/inc/ \
		-I$(srctree)/drivers/nand/manager/inc/

obj-$(CONFIG_REAL_NAND_DRIVER) += nand_api.o
obj-$(CONFIG_REAL_NAND_DRIVER) += core/ jz_ctrl/
obj-$(CONFIG_NAND_DMA) += dma/
obj-$(CONFIG_FAKE_NAND_DRIVER) += fake_driver/
