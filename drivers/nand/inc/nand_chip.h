/*
 * lib_nand/nand_IC/nand_chip.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#ifndef __NAND_CHIP_H__
#define __NAND_CHIP_H__
#define MAX_NANDADDR_CYCLE	5
#include "nand_api.h"
/* NAND */
#define NAND_MAX_OOBSIZE	(4 * (640 + 512))
#define NAND_MAX_PAGESIZE	(4 * 8192)


/**
 * struct jz_nand_chip -
 *
 * @pagesize:		physical page size
 * @oobsize:		physical oob size
 * @blocksize:		physical block size
 * @ppblock:		pages per block
 * @chipsize:		physical chip size
 * @realplanenum:	physical plane count in one chip
 * @buswidth:		
 * @row_cycles:
 * @numchips:		how many valid chips
 * @badblockpos:	badblock mark position in the oob
 * @maxbadblockcount : the max number of bad block
 * @planenum:		support two-plane operation or one-plane operation
 * @freesize:		the area size in the page which cache ecc code
 * @writesize:		pagesize - freesize
 * @erasesize:		blocksize - freesize*ppblock
 */
struct jz_nand_chip {
        unsigned int pagesize;
        unsigned int oobsize;
        unsigned int blocksize;	
        unsigned int ppblock;
        unsigned int bpchip;
        unsigned int ppchip;
        unsigned int realplanenum;
        unsigned int buswidth;
        unsigned int row_cycles;
        unsigned int numchips;
        unsigned int badblockpos;
        unsigned int maxbadblockcount;      //add by hui
        unsigned int planenum;
        unsigned int freesize;
        unsigned int writesize;
        unsigned int erasesize;
        unsigned long long chipsize;
	unsigned int priv;
};

typedef struct jz_nand_chip JZ_NAND_CHIP;

/**
 * struct nand_flash_dev - NAND Flash Device ID Structure
 * @name:	Identify the device type
 * @id:		device ID code
 * @pagesize:	Pagesize in bytes. Either 256 or 512 or 0
 * 		If the pagesize is 0, then the real pagesize
 * 		and the eraseize are determined from the
 * 		extended id bytes in the chip
 * @erasesize:	Size of an erase block in the flash device.
 * @chipsize:	Total chipsize in Mega Bytes
 * @options:	Bitfield to store chip relevant options
 */
struct nand_flash_dev {
        char *name;
        int id; 
        unsigned int extid;
        int realplanenum;
        int dienum;
        int tals;
        int talh;
        int trp;
        int twp;
        int trhw;
        int twhr;
	int twhr2;
	int trr;
	int twb;
	int tadl;
	int tcwaw;
        unsigned long pagesize;
        unsigned long erasesize;
        unsigned int oobsize;
        int rowcycle;
        int maxbadblocks;
        int maxvalidblocks;
        int eccblock;
        int eccbit;
        int buswidth;
        int badblockpos;    
        unsigned long options;
	unsigned char lowdriver;
	unsigned char normaldriver;
	unsigned char highdriver;
	unsigned char high2driver;
	unsigned char timemode;
};
typedef struct nand_flash_dev NAND_FLASH_DEV;

/**
 * struct nand_manufacturers - NAND Flash Manufacturer ID Structure
 * @name:	Manufacturer name
 * @id:		manufacturer ID code of device.
 */
struct nand_manufacturers {
        int id;
        char * name;
};

typedef struct nand_manufacturers NAND_MANUFACTURERS;

/*
 * struct nand_api -
 *
 * @writesize:  per page size
 * @erasesize:  per block size
 * @ppblock:    pages per block
 * @bpchip: blocks per chip
 * @totalblock: total block count
 * @pdev :      platform device
 * 
 */
struct nand_api {
        unsigned int writesize;
        unsigned int erasesize;
        unsigned int ppblock;
        unsigned int totalblock;

        JZ_ECC *nand_ecc;
        JZ_IO *nand_io;
        NAND_CTRL *nand_ctrl;
        JZ_NAND_CHIP *nand_chip;

        void * pdev;   // nand platform_device 
        struct jznand_dma  *nand_dma;
        NAND_BASE * vnand_base; //virtual nand base
        NAND_BASE * pnand_base; // physical nand base
        unsigned gpio_wp;
};

typedef struct nand_api NAND_API;

/* NAND Init */
int nand_chip_init(NAND_BASE *host,NAND_API *nand_api);

/* NAND Chip operations */
/*
   int nand_read_page(unsigned int pageid, unsigned int offset,unsigned int bytes,void * databuf);
   int nand_read_pages(Aligned_List *aligned_list);

   int nand_write_page(unsigned int pageid, unsigned int offset,unsigned int bytes,void * databuf);
   int nand_write_pages(Aligned_List *aligned_list);

   int nand_erase_blocks(BlockList *headlist);

   int isbadblock(unsigned int blockid);
   int markbadblock(unsigned int blockid);

*/
/* copyback ops */
//void nand_cb_page(unsigned char *buf, int len, int srcpage, int destpage);
//void nand_cb_planes(unsigned char *buf, int len, int srcpage, int destpage);
/*
   void nand_get_id(char *nand_id);
   void nand_reset(void);
   void send_read_status(unsigned char *status);
   */
#endif /* __NAND_CHIP_H__ */
