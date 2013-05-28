/*
 * common.h
 */
#ifndef __COMMON_H__
#define __COMMON_H__

#define TCSM_BANK0	0xF4000000
#define TCSM_BANK1	0xF4000800
#define TCSM_BANK2	0xF4001000
#define TCSM_BANK3	0xF4001800
#define TCSM_BANK4	0xF4002000
#define TCSM_BANK5	0xF4002800
#define TCSM_BANK6	0xF4003000
#define TCSM_BANK7	0xF4003800

//#define MCU_TEST_INTER
#ifdef MCU_TEST_INTER
#define MCU_TEST_DATA 0xF4002FC0  //TCSM_BANK6 - 0x40
#endif

#define NULL		0
#define UNCOR_ECC	(0x01<<0)
#define ALL_FF          (0x01<<1)
#define MOVE_BLOCK      (0x01<<2) 
#define ALIGN_ADDR_WORD(addr)	(void *)((((unsigned int)(addr) >> 2) + 1) << 2)

typedef		char s8;
typedef	unsigned char	u8;
typedef		short s16;
typedef unsigned short	u16;
typedef		int s32;
typedef unsigned int	u32;

#endif /* __COMMON_H__ */
