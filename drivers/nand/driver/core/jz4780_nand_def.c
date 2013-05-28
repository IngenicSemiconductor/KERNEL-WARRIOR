#include <asm/io.h>

#include "jz4780_nand_def.h"

/******  the operation of nemc registers  ******/
void init_nandchip_smcr_n(NAND_BASE *host,unsigned int chip,unsigned int value)
{
	nemc_writel(host->nemc_iomem,NEMC_SMCR(chip),value);
}
void pn_enable(NAND_BASE *host)
{
	nemc_writel(host->nemc_iomem,NEMC_PNCR,(NEMC_PNCR_PNRST | NEMC_PNCR_PNEN));
}

void pn_disable(NAND_BASE *host)
{
	nemc_writel(host->nemc_iomem,NEMC_PNCR,0x0);
}

void nand_enable(NAND_BASE *host,unsigned int n)
{
	nemc_writel(host->nemc_iomem,NEMC_NFCSR,(NEMC_NFCSR_NFE(n) | NEMC_NFCSR_NFCE(n)));
}

void nand_disable(NAND_BASE *host)
{
	nemc_writel(host->nemc_iomem,NEMC_NFCSR,0x0);
}

void tnand_dphtd_sync(NAND_BASE *host,unsigned int n)
{
	unsigned int tmp=nemc_readl(host->nemc_iomem,NEMC_TGPD);
	while(!(tmp & NEMC_TGPD_DPHTD(n)))
		tmp =nemc_readl(host->nemc_iomem,NEMC_TGPD);
}

void tnand_enable(NAND_BASE *host,unsigned int n)
{
	unsigned int tmp;
	nemc_writel(host->nemc_iomem,NEMC_NFCSR,(NEMC_NFCSR_TNFE(n) | NEMC_NFCSR_NFE(n)));
	tnand_dphtd_sync(host,n);
	tmp =nemc_readl(host->nemc_iomem,NEMC_NFCSR);
	tmp |=NEMC_NFCSR_NFCE(n) | NEMC_NFCSR_DAEC;
	nemc_writel(host->nemc_iomem,NEMC_NFCSR,tmp);
}

void tnand_disable(NAND_BASE *host)
{
	unsigned int tmp =nemc_readl(host->nemc_iomem,NEMC_NFCSR);
	tmp &=~NEMC_NFCSR_NFCE1;
	nemc_writel(host->nemc_iomem,NEMC_NFCSR,tmp);
	tnand_dphtd_sync(host,1);
	nemc_writel(host->nemc_iomem,NEMC_NFCSR,0x0);
}

void tnand_dae_sync(NAND_BASE *host)
{
	unsigned int tmp=nemc_readl(host->nemc_iomem,NEMC_TGWE);
	while(!(tmp & NEMC_TGWE_DAE))
		tmp =nemc_readl(host->nemc_iomem,NEMC_TGWE);
}

void tnand_datard_perform(NAND_BASE *host)
{
	unsigned int tmp =nemc_readl(host->nemc_iomem,NEMC_TGWE);
	tmp |= NEMC_TGWE_DAE;
	tnand_dae_sync(host);
}

void tnand_dpsdelay_init(NAND_BASE *host,unsigned int n)
{
	unsigned int tmp =nemc_readl(host->nemc_iomem,NEMC_TGDR);
	tmp &=(~NEMC_TGDR_RDQS_MASK);
	tmp |=((n) & NEMC_TGDR_RDQS_MASK);
	nemc_writel(host->nemc_iomem,NEMC_TGDR,tmp);
}

void tnand_dae_clr(NAND_BASE *host)
{
	unsigned int tmp =nemc_readl(host->nemc_iomem,NEMC_TGWE);
	while(tmp & NEMC_TGWE_DAE)
		tmp =nemc_readl(host->nemc_iomem,NEMC_TGWE);
}

void tnand_wcd_sync(NAND_BASE *host)
{
	unsigned int tmp =nemc_readl(host->nemc_iomem,NEMC_TGWE);
	while(!(tmp & NEMC_TGWE_WCD))
		tmp =nemc_readl(host->nemc_iomem,NEMC_TGWE);
}

void tnand_dr_sync(NAND_BASE *host,unsigned int timeout)
{
	unsigned int tmp =nemc_readl(host->nemc_iomem,NEMC_TGDR);
	while(!(tmp & NEMC_TGDR_DONE_MASK) && timeout--)
		tmp =nemc_readl(host->nemc_iomem,NEMC_TGDR);
}

void tnand_delay_sync(NAND_BASE *host)
{
	unsigned int tmp =nemc_readl(host->nemc_iomem,NEMC_TGDR);
	while(!(tmp & NEMC_TGDR_DONE_MASK))
		tmp =nemc_readl(host->nemc_iomem,NEMC_TGDR);
}

unsigned int tnand_dpsdelay_done(NAND_BASE *host)
{
	unsigned int tmp;
	tmp =nemc_readl(host->nemc_iomem,NEMC_TGDR);
	tmp &= NEMC_TGDR_DONE_MASK;
	return tmp;
}

void tnand_dpsdelay_probe(NAND_BASE *host)
{
	unsigned int tmp =nemc_readl(host->nemc_iomem,NEMC_TGDR);
	tmp |= NEMC_TGDR_DET | NEMC_TGDR_AUTO;
	nemc_writel(host->nemc_iomem,NEMC_TGDR,tmp);
	tnand_delay_sync(host);
}

void tnand_fce_set(NAND_BASE *host,unsigned int n)
{
	unsigned int tmp;
	tnand_dphtd_sync(host,n);
	tmp =nemc_readl(host->nemc_iomem,NEMC_NFCSR);
	tmp |= NEMC_NFCSR_NFCE(n);
	nemc_writel(host->nemc_iomem,NEMC_NFCSR,tmp);
}

void tnand_fce_clear(NAND_BASE *host,unsigned int n)
{
	unsigned int tmp;
	tmp = nemc_readl(host->nemc_iomem,NEMC_NFCSR);
	tmp |= NEMC_NFCSR_DAEC;
	nemc_writel(host->nemc_iomem,NEMC_NFCSR,tmp);

	tmp = nemc_readl(host->nemc_iomem,NEMC_NFCSR);
	tmp &= ~NEMC_NFCSR_NFCE(n);
	nemc_writel(host->nemc_iomem,NEMC_NFCSR,tmp);
	tnand_dphtd_sync(host,n);
}

/*****  the operation of bch registers  *****/

void bch_encoding_nbit(NAND_BASE *host,unsigned int n)
{
	unsigned int tmp = (BCH_CR_BSEL(n) | BCH_CR_ENCE | BCH_CR_BCHE |BCH_CR_INIT);
	bch_writel(host->bch_iomem,BCH_CRS,tmp);
}
void bch_decoding_nbit(NAND_BASE *host,unsigned int n)
{
	unsigned int tmp = (BCH_CR_BSEL(n) | BCH_CR_DECE | BCH_CR_BCHE |BCH_CR_INIT);
	bch_writel(host->bch_iomem,BCH_CRS,tmp);
}

void bch_disable(NAND_BASE *host)
{
	bch_writel(host->bch_iomem,BCH_CRC,BCH_CR_BCHE);
}

void bch_encode_sync(NAND_BASE *host)
{
	unsigned int tmp;
		tmp = bch_readl(host->bch_iomem,BCH_INTS);
	while(!(tmp & BCH_INTS_ENCF))
		tmp = bch_readl(host->bch_iomem,BCH_INTS);
}

void bch_decode_sync(NAND_BASE *host)
{
	unsigned int tmp;
	tmp = bch_readl(host->bch_iomem,BCH_INTS);
	while(!(tmp & BCH_INTS_DECF))
		tmp = bch_readl(host->bch_iomem,BCH_INTS);
}

void bch_decode_sdmf(NAND_BASE *host)
{
	unsigned int tmp;
	tmp = bch_readl(host->bch_iomem,BCH_INTS);
	while(!(tmp & BCH_INTS_SDMF))
		tmp = bch_readl(host->bch_iomem,BCH_INTS);
}

void bch_encints_clear(NAND_BASE *host)
{
	unsigned int tmp;
	tmp = bch_readl(host->bch_iomem,BCH_INTS);
	tmp |= BCH_INTS_ENCF;
	bch_writel(host->bch_iomem,BCH_INTS,tmp);
}

void bch_decints_clear(NAND_BASE *host)
{
	unsigned int tmp;
	tmp = bch_readl(host->bch_iomem,BCH_INTS);
	tmp |= BCH_INTS_DECF;
	bch_writel(host->bch_iomem,BCH_INTS,tmp);
}

/* blk = ECC_BLOCK_SIZE, par = ECC_PARITY_SIZE */
void bch_cnt_set(NAND_BASE *host,unsigned int blk,unsigned int par)
{
	unsigned int tmp;
	tmp = bch_readl(host->bch_iomem,BCH_CNT);
	tmp &= ~(BCH_CNT_PARITY_MASK | BCH_CNT_BLOCK_MASK);
	tmp |= ((par) << BCH_CNT_PARITY_BIT | (blk) << BCH_CNT_BLOCK_BIT);
	bch_writel(host->bch_iomem,BCH_CNT,tmp);
}

void bch_enable(NAND_BASE *host,unsigned int mode,unsigned int eccsize,unsigned int eccbit)
{
	unsigned int tmp = BCH_CR_BCHE;
		if (mode)
			tmp |= BCH_CR_ENCE;
		tmp |= BCH_CR_BSEL(eccbit);
		bch_cnt_set(host,eccsize,__bch_cale_eccbytes(eccbit));
#ifdef CONFIG_MTD_NAND_DMA
		tmp |=BCH_CR_DMA;
#endif
		bch_writel(host->bch_iomem,BCH_CRC,(~tmp));
		bch_writel(host->bch_iomem,BCH_CRS,tmp);
		bch_writel(host->bch_iomem,BCH_CRS,BCH_CR_INIT);
                /* set BCH_CR MZEBS = 0x7 */
		bch_writel(host->bch_iomem,BCH_CRS,BCH_CR_MZEB(7));
}


/*****  the operation of pdma registers  *****/

void dmac_enable(NAND_BASE *host)
{
	unsigned int tmp =pdma_readl(host->pdma_iomem,DMAC_DMACR);
	tmp |= DMAC_DMACR_DMAE;
	pdma_writel(host->pdma_iomem,DMAC_DMACR,tmp);
}

void dmac_disable(NAND_BASE *host)
{
	unsigned int tmp =pdma_readl(host->pdma_iomem,DMAC_DMACR);
	tmp &= ~DMAC_DMACR_DMAE;
	pdma_writel(host->pdma_iomem,DMAC_DMACR,tmp);
}


