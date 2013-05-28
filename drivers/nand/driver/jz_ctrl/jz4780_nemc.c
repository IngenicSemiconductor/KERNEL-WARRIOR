/*
 *  lib_nand/nand_controller/nand_init.c
 *
 * JZ4770 Nand Driver
 *
 *  Init utils.
 */

#include "nand_api.h"
#define NAND_CS_OFFSET         0x01000000      /* nand cs physical address offset */
#define NAND_CMD_OFFSET        0x00400000      /* command port offset for unshare mode */
#define NAND_ADDR_OFFSET       0x00800000      /* address port offset for unshare mode */

int g_maxchips = 0; /*the real support chip number on this device*/
unsigned char g_chips_mark[NEMC_CS_COUNT]={0};
/**
 * nand_calc_smcr - calculate NEMC.SMCR value.
 * @index:	NAND Flash index in nand_flash_ids table
 */
static int nand_calc_smcr(NAND_BASE *host,void *flash_chip)
{
	NAND_FLASH_DEV *flash = (NAND_FLASH_DEV *)flash_chip;
	/*int div[10] = {
		1, 2, 3, 4, 6, 8, 12, 16, 24, 32
	};
	int h2div = (REG_CPM_CPCCR & CPCCR_H2DIV_MASK) >> CPCCR_H2DIV_LSB;          //jz4780cpm.h
	int cclk = (jz_clocks.cclk + 500000) / 1000000;    
	int h2clk = cclk / div[h2div];
	
	int cycle = 1000 / h2clk + 1;	// unit: ns */
	int clk =clk_get_rate(host->nemc_gate);
	int cycle = 1000000000 / (clk / 1000);  //unit: ps
	int smcr_val = 0;
	int data;
/*
	dprintf("INFO: bchclk=%dMHz clk=%dMHz cycle=%dns\n",bchclk/1000000,clk/1000000,cycle);
	dprintf("==>%s L%d, h2div=%d, cclk=%d, h2clk=%d, cycle=%d\n",
			__func__, __LINE__,
			div[h2div], cclk,
			h2clk, cycle);
*/
	/* NEMC.TAS */
	data = (flash->tals * 1000 + cycle - 1) / cycle;
	smcr_val |= (data & NEMC_SMCR_TAS_MASK) << NEMC_SMCR_TAS_BIT;
	/* NEMC.TAH */
	data = (flash->talh * 1000 + cycle -1) / cycle;
	smcr_val |= (data & NEMC_SMCR_TAH_MASK) << NEMC_SMCR_TAH_BIT;
	/* NEMC.TBP */
	data = (flash->twp * 1000 + cycle - 1) / cycle;
	smcr_val |= (data & NEMC_SMCR_TBP_MASK) << NEMC_SMCR_TBP_BIT;
	/* NEMC.TAW */
	data = (flash->trp * 1000 + cycle -1) / cycle;
	smcr_val |= (data & NEMC_SMCR_TAW_MASK) << NEMC_SMCR_TAW_BIT;
	/* NEMC.STRV */
	data = (flash->trhw * 1000 + cycle - 1) / cycle;
	smcr_val |= (data & NEMC_SMCR_STRV_MASK) << NEMC_SMCR_STRV_BIT;

	dprintf("INFO: tals=%d talh=%d twp=%d trp=%d smcr=0x%08x\n"
                        , flash->tals, flash->talh, flash->twp, flash->trp, smcr_val);
	return smcr_val;
}

/**
 * jz_nemc_setup_default - NEMC setup.
 *
 */
static inline void jz_nemc_setup_default(NAND_BASE *host,void *pnand_io)
{
	JZ_IO *p_io = (JZ_IO *)pnand_io;
	unsigned int ret =0, i=NEMC_CS_COUNT;
#ifdef JZNAND_DRIVE_DEBUG
	int clk =clk_get_rate(host->nemc_gate);
	int bchclk =clk_get_rate(host->bch_clk);
#endif
	if(p_io == 0)
                dprintf("error addr!p_io is 0x%x\n",(unsigned int)p_io);
/*      gpio init          */
#if 0
	*(volatile unsigned int *)(0xb0010018) =0x00430000;
	*(volatile unsigned int *)(0xb0010028) =0x00430000;
	*(volatile unsigned int *)(0xb0010038) =0x00430000;
	*(volatile unsigned int *)(0xb0010048) =0x00430000;
	*(volatile unsigned int *)(0xb0010014) =0x00100000;
	*(volatile unsigned int *)(0xb0010028) =0x00100000;
	*(volatile unsigned int *)(0xb0010034) =0x00100000;
	*(volatile unsigned int *)(0xb0010044) =0x00100000;

	*(volatile unsigned int *)(0xb0010018) =0x002c00ff;
	*(volatile unsigned int *)(0xb0010028) =0x002c00ff;
	*(volatile unsigned int *)(0xb0010038) =0x002c00ff;
	*(volatile unsigned int *)(0xb0010048) =0x002c00ff;
	*(volatile unsigned int *)(0xb0010074) =0x002c00ff;
	*(volatile unsigned int *)(0xb0010174) =0x00000003;
	dprintf("gpioa int =0x%x \n ",*(unsigned int *)0xb0010010);
	dprintf("gpioa mask =0x%x \n ",*(unsigned int *)0xb0010020);
	dprintf("gpioa pat1 =0x%x \n ",*(unsigned int *)0xb0010030);
	dprintf("gpioa pat0 =0x%x \n ",*(unsigned int *)0xb0010040);
	dprintf("gpiob int =0x%x \n ",*(unsigned int *)0xb0010110);
	dprintf("gpiob mask =0x%x \n ",*(unsigned int *)0xb0010120);
	dprintf("gpiob pat1 =0x%x \n ",*(unsigned int *)0xb0010130);
	dprintf("gpiob pat0 =0x%x \n ",*(unsigned int *)0xb0010140);
	dprintf("nemc nfcsr =0x%x \n ",*(unsigned int *)0xb3410050);
	dprintf("nemc smcr =0x%x \n ",*(unsigned int *)0xb3410014);
#endif
	p_io->buswidth = 8;
	p_io->pagesize = 2048;
	/* Read/Write timings */
//	init_nandchip_smcr_n(host,1,SMCR_DEFAULT_VAL);
        dprintf("INFO: Nand Driver Timing SMCR=0x%08x BCHCLK=%dMHz NEMCLK=%dMHz\n"
                        ,*(volatile unsigned int *)0xb3410014
                        ,bchclk/1000000,clk/1000000);
	//*(volatile unsigned int *)0xb3410014 = 0x11444400;
	g_chips_mark[0]=1;
	
#if defined(CONFIG_NAND_CS2)
	if (NEMC_CS_COUNT < 2)
		return;
	/* Read/Write timings */
	init_nandchip_smcr_n(host,2,SMCR_DEFAULT_VAL);
	g_chips_mark[1]=1;
#endif

#if defined(CONFIG_NAND_CS3)
	if (NEMC_CS_COUNT < 3)
		return;
	/* Read/Write timings */
	init_nandchip_smcr_n(host,3,SMCR_DEFAULT_VAL);
	g_chips_mark[2]=1;
#endif

#if defined(CONFIG_NAND_CS4)
	if (NEMC_CS_COUNT < 4)
		return;
	/* Read/Write timings */
	init_nandchip_smcr_n(host,4,SMCR_DEFAULT_VAL);
	g_chips_mark[3]=1;
#endif

#if defined(CONFIG_NAND_CS5)
	if (NEMC_CS_COUNT < 5)
		return;
	/* Read/Write timings */
	init_nandchip_smcr_n(host,5,SMCR_DEFAULT_VAL);
	g_chips_mark[4]=1;
#endif

#if defined(CONFIG_NAND_CS6)
	if (NEMC_CS_COUNT < 6)
		return;
	/* Read/Write timings */
	init_nandchip_smcr_n(host,6,SMCR_DEFAULT_VAL);
	g_chips_mark[5]=1;
#endif
	while(i--){
		if(g_chips_mark[i])
			ret++;
	}
	g_maxchips = ret;
}

/**
 * jz_nemc_ctrl_select -
 * @nand_nce:	
 */
int jz_nemc_ctrl_select(NAND_BASE *host,void *pnand_io,int nand_nce)
{
	JZ_IO *p_io = (JZ_IO *)pnand_io;
	unsigned int i=0,ret=0;	
	if ((nand_nce >= g_maxchips) || (nand_nce == -1))
	{
		nemc_writel(host->nemc_iomem,NEMC_NFCSR,0);
		return -1; /*bigger than support, or unselect all chips*/
	}
	while(ret <= nand_nce && ret < NEMC_CS_COUNT)
	{
		if(g_chips_mark[i])
			ret++;
		i++;
	}
	switch(i){
	case 1:
		p_io->dataport = (void *)(host->nemc_cs6_iomem + 5 * 0x01000000);
		nand_enable(host,1);
		ret =1;
		break;
	case 2:
		p_io->dataport = (void *)(host->nemc_cs6_iomem + 4 * 0x01000000);
		nand_enable(host,2);
		ret =2;
		break;
	case 3:
		p_io->dataport = (void *)(host->nemc_cs6_iomem + 3 * 0x01000000);
		nand_enable(host,3);
		ret =3;
		break;
	case 4:
		p_io->dataport = (void *)(host->nemc_cs6_iomem + 2 * 0x01000000);
		nand_enable(host,4);
		ret =4;
		break;
	case 5:
		p_io->dataport = (void *)(host->nemc_cs6_iomem + 1 * 0x01000000);
		nand_enable(host,5);
		ret =5;
		break;
	case 6:
		p_io->dataport = (void *)(host->nemc_cs6_iomem + 0 * 0x01000000);
		nand_enable(host,6);
		ret =6;
		break;
	default:
	  eprintf("error: no nand_nce 0x%x\n",nand_nce);
	  ret = -1;
		break;
	}
#ifdef CONFIG_TOGGLE_NAND
	switch(i){
	case 1:
		tnand_enable(host,1);
		break;
	case 2:
		tnand_enable(host,2);
		break;
	case 3:
		tnand_enable(host,3);
		break;
	case 4:
		tnand_enable(host,4);
		break;
	case 5:
		tnand_enable(host,5);
		break;
	case 6:
		tnand_enable(host,6);
		break;
	default:
	  eprintf(1,"error: no nand_nce 0x%x\n",nand_nce);
		break;
	}
#endif
	p_io->cmdport = p_io->dataport + NAND_CMD_OFFSET;
	p_io->addrport = p_io->dataport + NAND_ADDR_OFFSET;
//	dprintf("DEBUG nand: jz_nemc_ctrl_select i = %d , p_io->dataport = 0x%x\n",i,(unsigned int)p_io->dataport);
	return ret;
}

void jz_nemc_setup_later(NAND_BASE *host,void *pnand_io,void *flash_chip)
{
	unsigned int i=0 ,ret =0;
	JZ_IO *p_io = (JZ_IO *)pnand_io;
	/* Calculate appropriate Read/Write timings */
	int smcr = nand_calc_smcr(host,flash_chip);	
	NAND_FLASH_DEV *pnand_type = flash_chip;
	if(pnand_type->buswidth == 16)
	{
		smcr |= 1 << NEMC_SMCR_BW_BIT;
		p_io->buswidth = 16;
#ifdef CONFIG_NAND_BUS_WIDTH_8
		p_io->buswidth =8;
#endif
#ifdef CONFIG_NAND_JZ4780
		p_io->buswidth =8;
#endif
	}
	p_io->pagesize = pnand_type->pagesize;
	nemc_writel(host->nemc_iomem,NEMC_SMCR1,smcr);
	while(ret < g_maxchips)
	{
		if(g_chips_mark[i]){
			ret++;
			init_nandchip_smcr_n(host,i,smcr);
		}
		i++;
	}
}

NAND_CTRL jznand_nemc =
{
	.chip_select = jz_nemc_ctrl_select,
	.setup_default = jz_nemc_setup_default,
	.setup_later = jz_nemc_setup_later,
};
