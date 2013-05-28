OUTPUT_ARCH(mips)
ENTRY(_reset)
MEMORY{
	tcsm_bank0(rw)  : ORIGIN = 0xF4000000 , LENGTH = 0x2000
	tcsm_bank4(rw)  : ORIGIN = 0xF4002000 , LENGTH = 0x800
	tcsm_bank5(rw)  : ORIGIN = 0xF4002800 , LENGTH = 0x800
}

SECTIONS
{
	.tcsm_bank0 : {
		. = ALIGN(4);
                *(.text)

                *(.data)
                CONSTRUCTORS
                _gp = ALIGN(4);
                *(.sdata)
                *(.sdata.*)
                *(.sbss)
                *(.bss)
                *(.scommon)
                *(.rodata)
	} > tcsm_bank0

	.tcsm_bank4 : {
		*(.tcsm_data4)
	} > tcsm_bank4

	.tcsm_bank5 : {
		*(.tcsm_data5)
	} > tcsm_bank5

	/DISCARD/ : {
		*(COMMON)
		*(.pdri)
		*(.comment)
		*(.gnu.attributes)
		*(.reginfo)
	}
}

