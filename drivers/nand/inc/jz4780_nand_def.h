/* driver/nand/inc/jz4780_nand.h
*
* JZ4780 nand definition.
*
* Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
*/
#ifndef __JZ4780_NAND_H__
#define __JZ4780_NAND_H__

#include <linux/clk.h>
#include <linux/compiler.h>
#include "jz4780_nemc.h"
#include "jz4780_bch.h"
#include "jz4780_pdma.h"

struct nand_base{
	struct clk *nemc_gate;
	struct clk *bch_clk;
	struct clk *bch_gate;
	/* interrupt vector  */
	int            irq;
	int            nemc_irq;
        int            bch_irq;
	int            rb_irq;
	/*  physical address to virtual address  */
	void __iomem        *nemc_iomem;
	void __iomem        *bch_iomem;
	void __iomem        *pdma_iomem;
	void __iomem        *nemc_cs6_iomem;
};
typedef struct nand_base NAND_BASE;

/* Register access macros */
#define nemc_readl(base,reg)             \
     __raw_readl((base) + reg)
#define nemc_writel(base,reg,value)          \
     __raw_writel((value), (base) + reg)

#define bch_readl(base,reg)             \
     __raw_readl((base) + reg)
#define bch_writel(base,reg,value)          \
     __raw_writel((value), (base) + reg)
#define bch_writeb(base,reg,value)          \
     __raw_writeb((value), (base) + reg)

#define pdma_readl(base,reg)             \
     __raw_readl((base) + reg)
#define pdma_writel(base,reg,value)          \
     __raw_writel((value), (base) + reg)

/*****   the operation of nemc register   ******/
void init_nandchip_smcr_n(NAND_BASE *host,unsigned int n,unsigned int value);
void pn_enable(NAND_BASE *host);
void pn_disable(NAND_BASE *host);
void nand_enable(NAND_BASE *host,unsigned int n);
void nand_disable(NAND_BASE *host);
void tnand_dphtd_sync(NAND_BASE *host,unsigned int n);
void tnand_enable(NAND_BASE *host,unsigned int n);
void tnand_disable(NAND_BASE *host);
void tnand_dae_sync(NAND_BASE *host);
void tnand_datard_perform(NAND_BASE *host);
void tnand_dpsdelay_init(NAND_BASE *host,unsigned int n);
void tnand_dae_clr(NAND_BASE *host);
void tnand_wcd_sync(NAND_BASE *host);
void tnand_dr_sync(NAND_BASE *host,unsigned int timeout);
void tnand_delay_sync(NAND_BASE *host);
unsigned int tnand_dpsdelay_done(NAND_BASE *host);
void tnand_dpsdelay_probe(NAND_BASE *host);
void tnand_fce_set(NAND_BASE *host,unsigned int n);
void tnand_fce_clear(NAND_BASE *host,unsigned int n);

/****  the operation of bch registers   ****/
void bch_encoding_nbit(NAND_BASE *host,unsigned int n);
void bch_decoding_nbit(NAND_BASE *host,unsigned int n);
void bch_disable(NAND_BASE *host);
void bch_encode_sync(NAND_BASE *host);
void bch_decode_sync(NAND_BASE *host);
void bch_decode_sdmf(NAND_BASE *host);
void bch_encints_clear(NAND_BASE *host);
void bch_decints_clear(NAND_BASE *host);
void bch_cnt_set(NAND_BASE *host,unsigned int blk,unsigned int par);
void bch_enable(NAND_BASE *host, unsigned int mode,unsigned int eccsize,unsigned int eccbit);  //mode :BCH_ENCODE or BCH_DECODE 

/****  the operation of pdma registers   ****/
void dmac_enable(NAND_BASE *host);
void dmac_disable(NAND_BASE *host);

#endif
