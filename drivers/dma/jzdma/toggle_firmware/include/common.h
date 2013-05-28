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

#define NULL		0
#define ALIGN_ADDR_WORD(addr)	(void *)((((unsigned int)(addr) >> 2) + 1) << 2)

typedef		char s8;
typedef	unsigned char	u8;
typedef		short s16;
typedef unsigned short	u16;
typedef		int s32;
typedef unsigned int	u32;

#endif /* __COMMON_H__ */
