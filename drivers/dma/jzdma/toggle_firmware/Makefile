
targets		:= firmware_hex firmware.o

drop-sections := .reginfo .mdebug .oomment .note .pdr .options .MIPS.options
strip-flags   := $(addprefix --remove-section=,$(drop-sections))
OBJCOPY_ARGS 	:= -O elf32-tradlittlemips

KBUILD_CFLAGS += -I$(obj)/include

LDFLAGS	:= -nostdlib -EL -T target.ld

OBJS := $(obj)/src/start.o		\
	$(obj)/src/main.o		\
	$(obj)/src/irq_pdma.o		\
	$(obj)/src/pdma.o		\
	$(obj)/src/nand/bch.o		\
	$(obj)/src/nand/nand_mcu.o	\
	$(obj)/src/nand/nand_pdma.o

FIRMWARE_BIN := $(obj)/firmware.bin

firmware_hex: $(FIRMWARE_BIN) FORCE
	@hexdump -v -e '"0x" 4/1 "%02x" "," "\n"' $< > $(obj)/../firmware.hex

$(FIRMWARE_BIN):$(obj)/firmware.o
	@$(LD) -nostdlib -EL -T $(obj)/target.ld $(OBJS) -o $(obj)/tmp.elf
	@$(OBJCOPY) $(strip-flags) $(OBJCOPY_ARGS) -O binary $(obj)/tmp.elf $@

$(obj)/firmware.o:$(OBJS)
