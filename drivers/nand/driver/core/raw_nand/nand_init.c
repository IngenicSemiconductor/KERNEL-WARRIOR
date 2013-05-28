/*
 *  lib_nand/nand_chip/nand_init.c
 *
 *  Nand Driver Init
 *
 *  Init utils.
 */

#include "nand_api.h"

JZ_NAND_CHIP jz_raw_nand;

/* declared at nand_ids.c */
extern int g_maxchips; /*the real support chip number on this device*/
extern NAND_FLASH_DEV *nand_scan_table(unsigned char *nand_id);
extern void do_nand_register(NAND_API *pnand_api);
extern int nand_reset(void);
extern void nand_get_id(char *nand_id);
extern void nand_set_features(unsigned char addr, unsigned char *data);
extern void nand_get_features(unsigned char addr, unsigned char *data);
extern void get_read_retrial_mode(unsigned char *buf);
extern void get_enhanced_slc(unsigned char *data);
unsigned char *retrialbuf;
unsigned char slcdata[4] = {0xff};
#define RETRY_SIZE 1026
//#define DEBUG_L   dbg_line()

static inline void dump_id(unsigned char *nand_id)
{
	unsigned char maf_id = 0;
	unsigned char dev_id = 0;
	unsigned int  ext_id;

	/* Read manufacturer and device IDs */	
	maf_id = nand_id[0];
	dev_id = nand_id[1];
	ext_id = ((nand_id[4] << 16) | (nand_id[3] << 8) | nand_id[2]);
}

/**
 * nand_get_flash_type
 * @return val: this nand chip index in nand_flash_ids table
 */
static inline NAND_FLASH_DEV *nand_get_flash_type(NAND_BASE *host,NAND_API *pnand_api)
{
	int ret=0;
	unsigned char nand_id[6] = {0};
	NAND_FLASH_DEV *pnand_type;
	NAND_CTRL *pnand_ctrl = pnand_api->nand_ctrl;
	JZ_IO *pnand_io = pnand_api->nand_io;
	/* select nand chip */
	pnand_ctrl->chip_select(host,pnand_io,0);

	/* reset nand */
	ret = nand_reset();
	if(ret < 0) {
		printk("NAND reset error!! - ret=%d -\n",ret);
		return 0;
	}

	/* read nand id */
	nand_get_id(nand_id);
//	dump_id(&nand_id[0]);

	/*get nand info from nand type info table in nand_ids.c*/
	pnand_type = nand_scan_table(nand_id);
#ifdef CONFIG_HW_BCH
        dprintf("nand hardware bch\n");
#endif	
	pnand_ctrl->chip_select(host,pnand_io,-1);
	return pnand_type;
}

static inline int issamechip(unsigned char *nand_id, NAND_FLASH_DEV *pnand_type)
{
	unsigned short dev_id;
	unsigned int ext_id;

	dev_id = ((nand_id[1] << 8) | nand_id[0]);
	ext_id = ((nand_id[4] << 16) | (nand_id[3] << 8) | nand_id[2]);

	if ((dev_id == pnand_type->id) && (ext_id == (pnand_type->extid & 0x00FFFFFF)))
	{
		return 1; /*yes,they are same*/
	}
	
	return 0; /*no, they aren't same*/
}

/**
 * calc_free_size -
 * @this:	jz_nand_chip structure
 */
static inline unsigned int calc_free_size(NAND_API *pnand_api)
{
	JZ_NAND_CHIP *pnand_chip = pnand_api->nand_chip;
	JZ_ECC 		*pnand_ecc = pnand_api->nand_ecc;
	
	unsigned int pagesize = pnand_chip->pagesize;
	unsigned int oobsize = pnand_chip->oobsize;
	unsigned int eccsize = pnand_ecc->eccsize;
	unsigned int eccbytes = pnand_ecc->eccbytes;
	unsigned int eccpos = pnand_ecc->eccpos;
	unsigned int freesize;

	if ((pagesize / eccsize + 1) * eccbytes + eccpos >= oobsize)
		freesize = 512;
	else
		freesize = 0;

	dprintf("INFO: pagesize=%d eccsize=%d eccbytes=%d eccpos=%d oobsize=%d freesize=%d\n"
                        ,pagesize, eccsize, eccbytes, eccpos, oobsize, freesize);

	return freesize;
}
 
static inline void get_nand_chip(NAND_API *pnand_api, NAND_FLASH_DEV *pnand_type)
{
	NAND_FLASH_DEV *type = pnand_type;
	JZ_NAND_CHIP *this = pnand_api->nand_chip;
		
	this->pagesize = type->pagesize;
	this->oobsize = type->oobsize;
	this->blocksize = type->erasesize;
	this->ppblock = this->blocksize / this->pagesize;
	this->bpchip = type->maxvalidblocks;
	this->ppchip = this->bpchip * this->ppblock;
	this->realplanenum = type->realplanenum;
	this->row_cycles = type->rowcycle;
	this->buswidth = type->buswidth;
	
	/* Set the max count of bad block */
	this->maxbadblockcount =type->maxbadblocks;
	
	/* Set the bad block position */
//	this->badblockpos = this->pagesize > 512 ?
//		NAND_LARGE_BADBLOCK_POS : NAND_SMALL_BADBLOCK_POS;
	this->badblockpos = NAND_BADBLOCK_POS;

	/* Store the number of chips and calc total size for mtd */

	this->numchips = g_maxchips;
	
	/* Calulate freesize */
	this->freesize = calc_free_size(pnand_api);
	this->writesize = this->pagesize - this->freesize;
	this->erasesize = this->blocksize - (this->freesize * this->ppblock);
	this->chipsize = (unsigned long long)this->erasesize * this->bpchip;
	if (this->realplanenum >= 2)
		this->planenum = 2;
	else
		this->planenum = this->realplanenum;
	this->priv = (unsigned int)pnand_type;
}

/**
 * nand_chip_init
 * @nand_api: nand_api struct for nand_api.c which preper for nand driver
 */
int nand_chip_init(NAND_BASE *host,NAND_API *pnand_api)
{
	unsigned char nand_id[6];
	int i,ret=0;
	NAND_FLASH_DEV	*pnand_type = 0;
	NAND_CTRL 		*pnand_ctrl = pnand_api->nand_ctrl;
	JZ_IO 			*pnand_io = pnand_api->nand_io;
	JZ_ECC 			*pnand_ecc = pnand_api->nand_ecc;

	do_nand_register(pnand_api);

	/*set default confige now*/
	pnand_ctrl->setup_default(host,pnand_io);

	pnand_io->io_init(pnand_io);

	pnand_type = nand_get_flash_type(host,pnand_api);
//	dprintf("DEBUG nand:nand_get_flash_type success\n");

	if (pnand_type == 0) {
		dprintf(" No NAND device found!!!\n");
		return -1;
	}
	pnand_ctrl->chip_select(host,pnand_io,0);
	ret = nand_reset();
	if(ret < 0) {
		printk("Error: NAND second reset error!! - ret=%d -\n",ret);
		return ret;
	}
	if (pnand_type->timemode) {
		unsigned char wdata[4] = {0x00};
		unsigned char rdata[4] = {0x00};
		wdata[0] = pnand_type->timemode;
		nand_set_features(0x01, wdata);
		nand_get_features(0x01, rdata);
		if (wdata[0] != rdata[0])
			printk("Warning: Nand flash timing mode set faild!!\n");
	}

	if (pnand_type->lowdriver != 0 || pnand_type->normaldriver != 0 || pnand_type->highdriver != 0) {
		unsigned char wdata[4] = {0x00};
		unsigned char rdata[4] = {0x00};
#if defined(CONFIG_NAND_LOW_DRIVER)
		wdata[0] = pnand_type->lowdriver;
#elif defined(CONFIG_NAND_NORMAL_DRIVER)
		wdata[0] = pnand_type->normaldriver;
#elif defined(CONFIG_NAND_HIGH_DRIVER)
		wdata[0] = pnand_type->highdriver;
#endif
		nand_set_features(0x10, wdata);
		nand_get_features(0x10, rdata);
		if (wdata[0] != rdata[0])
			printk("Warning: Nand flash output driver set faild!!\n");
	}

	/* Check for a chip array */
	for (i = 1; i < g_maxchips; i++) {
		/* select this device */
		pnand_ctrl->chip_select(host,pnand_io,i);
		/* See comment in nand_get_flash_type for reset */
		ret =nand_reset();
		if(ret < 0)
			return -1;
		/* Send the command for reading device ID */
		nand_get_id(&nand_id[0]);
		
		/* De-select the device */
		pnand_ctrl->chip_select(host,pnand_io,-1);
		
		/*compare this id with the first chip id,
                 * if not same, break,we cann't support,
                 * at the same time,
		 *ingore more chips which have been link to  CE.
		 */
		if (!issamechip(&nand_id[0],pnand_type)){
		        i--;
		        break;
		}
	}
	
	if (i > 1)
		dprintf(" %d NAND chips detected\n", i);
	
	/*set really confige now*/
	g_maxchips = i;
	pnand_ctrl->setup_later(host,pnand_io,pnand_type);
	pnand_ecc->ecc_init(pnand_ecc,pnand_type);
	get_nand_chip(pnand_api,pnand_type);
	pnand_io->io_init(pnand_io);	

        retrialbuf = (unsigned char *)kmalloc(RETRY_SIZE ,GFP_KERNEL);
        memset(retrialbuf, 0xff, RETRY_SIZE);
	get_read_retrial_mode(retrialbuf);
        get_enhanced_slc(slcdata);

	return 0;
}
