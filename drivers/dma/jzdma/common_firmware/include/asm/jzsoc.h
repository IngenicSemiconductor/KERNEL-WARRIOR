/*
 *  include/asm/jzsoc.h
 *
 *  Ingenic's JZXXXX SoC common include.
 *
 *  Copyright (C) 2006 - 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZSOC_H__
#define __ASM_JZSOC_H__

/*
 * SoC include
 */
#include <asm/mach-jz4780/jz4780.h>

/*
 * Generic I/O routines
 */
#define readb(addr)	(*(volatile unsigned char *)(addr))
#define readw(addr)	(*(volatile unsigned short *)(addr))
#define readl(addr)	(*(volatile unsigned int *)(addr))

#define writeb(b,addr)	((*(volatile unsigned char *)(addr)) = (b))
#define writew(b,addr)	((*(volatile unsigned short *)(addr)) = (b))
#define writel(b,addr)	((*(volatile unsigned int *)(addr)) = (b))

#endif /* __ASM_JZSOC_H__ */
