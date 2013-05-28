
# drivers/nand/driver/core/Makefile 
EXTRA_CFLAGS += -Idrivers/nand/inc/
ifdef CONFIG_TOGGLE_NAND
obj-y += toggle_nand/
else
obj-y += raw_nand/
endif
obj-y += ecc/nand_ecc.o
obj-y += io/nand_io.o
obj-y += jz4780_nand_def.o


