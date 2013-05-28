obj-y += jzdma.o

ifeq ($(CONFIG_NAND), y)

$(obj)/jzdma.o:$(obj)/firmware_hex

ifeq ($(CONFIG_XBURST_DMAC_COMMON),y)
$(obj)/firmware_hex:
	$(Q)$(MAKE) $(build)=drivers/dma/jzdma/common_firmware firmware_hex
endif

ifeq ($(CONFIG_XBURST_DMAC_TOGGLE),y)
$(obj)/firmware_hex:
	$(Q)$(MAKE) $(build)=drivers/dma/jzdma/toggle_firmware firmware_hex
endif

endif

