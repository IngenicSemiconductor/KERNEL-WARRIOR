#ifndef __NAND_DMA_OPS_H__
#define __NAND_DMA_OPS_H__
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <mach/jzdma.h>
#include "ppartition.h"
#include "nand_api.h"
#include "pagelist.h"
#include "blocklist.h"

#define NAND_DMA_WRITE          0
#define NAND_DMA_READ           1
#define SGL_LEN                 1
//#define TCSM_DMA_ADDR           (unsigned long)(0x13422000)

#define PDMA_FW_TCSM        0xB3422000
#define PDMA_MSG_CHANNEL    3
#define PDMA_MSG_TCSMPA     0x134247C0
#define PDMA_BANK3      0xB3423800
#define PDMA_BANK4      0xB3424000
#define PDMA_BANK5      0xB3424800
#define PDMA_BANK6      0xB3425000
#define PDMA_BANK7      0xB3425800
#define PDMA_MSG_TCSMVA    (PDMA_BANK5-0x50)

/* CSn for NEMC*/
#define NEMC_CS1                1
#define NEMC_CS2                2
#define NEMC_CS3                3
#define NEMC_CS4                4
#define NEMC_CS5                5
#define NEMC_CS6                6

/* Message for NAND cmd */
#define MSG_NAND_INIT           0x01
#define MSG_NAND_READ           0x02
#define MSG_NAND_WRITE          0x03
#define MSG_NAND_ERASE          0x04
#define MSG_NAND_FORCEREAD      0x05
#define MSG_NAND_FORCEWRITE     0x06

/* Message info bit for NAND ops */
#define MSG_NAND_BANK           0
#define MSG_DDR_ADDR            1
#define MSG_PAGEOFF             2
#define MSG_MCU_TEST		3

/* PDMA MailBox msg */
#define MB_NAND_INIT_DONE       0x01
#define MB_NAND_READ_DONE       0x02
#define MB_NAND_UNCOR_ECC       0x03
#define MB_NAND_WRITE_DONE      0x04
#define MB_NAND_WRITE_FAIL      0x05
#define MB_NAND_WRITE_PROTECT   0x06
#define MB_NAND_ERASE_DONE      0x07
#define MB_NAND_ERASE_FAIL      0x08
#define MB_NAND_ALL_FF          0x09
#define MB_MOVE_BLOCK           0x0a

/* Message info bit for NAND init */
#define MSG_NANDTYPE            0
#define MSG_PAGESIZE            1
#define MSG_OOBSIZE             2
#define MSG_ROWCYCLE            3
#define MSG_ECCLEVEL            4
#define MSG_ECCSIZE             5
#define MSG_ECCBYTES            6
#define MSG_ECCSTEPS            7
#define MSG_ECCTOTAL            8
#define MSG_ECCPOS              9

/* Message info bit for NAND delay */
#define MSG_TWHR		10
#define MSG_TWHR2		11
#define MSG_TRR 		12
#define MSG_TWB 		13
#define MSG_TADL		14
#define MSG_TCWAW		15

/* The time info bit for NEMC delay */
#define DEALY_TWHR              1
#define DEALY_TWHR2             2
#define DEALY_TRR               3
#define DEALY_TADL              4
#define DEALY_TWB               5

struct pdma_msg {
	unsigned int cmd;
	unsigned int info[16];
};
//#define NEW_NAND_DMA

#ifdef  NEW_NAND_DMA

#define MAXPAGENODE 256

struct _pdma_pagelist{
        unsigned int pageid;
        unsigned short offset;
        unsigned short bytes;
        unsigned int pdata_addr;
        unsigned int opsmodel;

        PageList *pagenode;
};

struct _pdma_blocklist{
        int blockid;
        unsigned int count;
        int opsmodel;
        BlockList *blocknode;
};

union _pdma_list{
        struct _pdma_pagelist pagelist;
        struct _pdma_blocklist blocklist;
};

typedef union _pdma_list PdmaList;

#endif


struct jznand_dma{
	struct device           *dev;
	struct device           *dma_dev;
	struct dma_chan         *mcu_chan;
	struct dma_chan         *data_chan;
	enum jzdma_type         chan_type;
	struct dma_slave_config dma_config;
//	struct scatterlist      sg;
#ifdef NEW_NAND_DMA
        PdmaList                *dma_opslist;
#else
	unsigned char           *data_buf;
	dma_addr_t              data_buf_phyaddr;
	unsigned int            data_buf_len;
#endif
	struct dma_async_tx_descriptor *desc;	
	int                     mailbox;
	struct pdma_msg         *msg;
        dma_addr_t              msg_phyaddr;
	PPartition              *ppt;
	int                     cache_phypageid;
};
/*
int nand_dma_init(NAND_API *pnand_api);
void nand_dma_deinit(struct jznand_dma *nand_dma);
int page_dma_read(const NAND_API *pnand_api,int pageid, int offset, int bytes, void *databuf);
int page_dma_write(const NAND_API *pnand_api,int pageid, int offset, int bytes, void *databuf);
int multipage_dma_read(const NAND_API *pnand_api, Aligned_List *list);
int multipage_dma_write(const NAND_API *pnand_api, Aligned_List *list);
**/
#endif
