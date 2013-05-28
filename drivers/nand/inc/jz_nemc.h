/*
 * lib_nand/nand_controller/jz4770_nemc.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#ifndef __NEMC_H__
#define __NEMC_H__

#include "nand_ecc.h"
#include "nand_io.h"

#define USE_BCH_AC	1	/* enable BCH Auto-Correction */

#define USE_DIRECT	0
#define	USE_PN		0
#define	USE_COUNTER	0
#define	COUNT_0		0	/* 1:count the number of 1; 0:count the number of 0 */
#define PN_ENABLE			(1 | PN_RESET)
#define PN_DISABLE			0
#define PN_RESET			(1 << 1)
#define COUNTER_ENABLE  	((1 << 3) | COUNTER_RESET)
#define COUNTER_DISABLE 	(0 << 3)
#define COUNTER_RESET   	(1 << 5)
#define COUNT_FOR_1			(0 << 4)
#define COUNT_FOR_0			(1 << 4)

#define SMCR_DEFAULT_VAL   0x11444400  //slowest

struct nand_ctrl
{
        int (*chip_select) (NAND_BASE *host,void *pnand_io,int cs);
        void (*setup_default) (NAND_BASE *host,void *pnand_io);
        void (*setup_later) (NAND_BASE *host,void *pnand_io,void *flash_chip);
};

typedef struct nand_ctrl NAND_CTRL;
#ifndef GPIO_CS1_N

#define GPIO_CS1_N (32*0+21)
#define GPIO_CS2_N (32*0+22)
#define GPIO_CS3_N (32*0+23)
#define GPIO_CS4_N (32*0+24)
#define GPIO_CS5_N (32*0+25)

#endif // GPIO_CS*_N

//JZ_ECC jznand_ecc;
//JZ_ECC jznand_dma_ecc;

//JZ_IO jznand_io;
//JZ_IO jznand_dma_io;

#endif /* __NEMC_H__ */
