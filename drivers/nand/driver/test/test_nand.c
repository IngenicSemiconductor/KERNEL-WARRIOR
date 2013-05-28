/*
 * test_nand.c
 *
 * Copyright (c) 2012 - 2015 Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


#include "test_nand.h"
#include <linux/delay.h>

//extern int nand_test_init(NAND_API *pnand_api);
extern int nand_test_read_page(PPartition *ppt,unsigned int pageaddr, unsigned int offset, unsigned int bytes, void *databuf);
extern int nand_test_read_pages(PPartition *ppt,PageList* pagelist);
extern int nand_test_write_page(PPartition *ppt,unsigned int pageaddr, unsigned int offset, unsigned int bytes, void *databuf);
extern int nand_test_write_pages(PPartition *ppt,PageList* pagelist);
extern int nand_test_erase(PPartition *ppt,BlockList *blocklist);
//extern void nand_test_2p_erase(unsigned int block);
//extern int nand_test_write_pages(unsigned int pageaddr, unsigned int count, unsigned int *buf);
//extern int nand_test_write_2p_pages(unsigned int pageaddr, unsigned int count,  unsigned int *buf);


NandInterface *vnand_interface;
VNandInfo g_vnand;
static PageList head_read[256] ,head_write[256];

void dump_pagelist(PageList *pagelist)
{
        int i = 0;
        PageList *list = pagelist;
        struct singlelist *listhead = NULL;
        printk("\n\n");
        for(i = 0; i < 256; i++) {
                printk("pagelist %d: \nstartPageID is %d, OffsetBytes is %d, Bytes is %d\n",i,list->startPageID,list->OffsetBytes,list->Bytes);
                listhead = (list->head).next;
                list = singlelist_entry(listhead,PageList,head);
        }
        printk("\n\n");
}

static inline int creat_pagelist(unsigned int pageid, unsigned char *writebuf ,unsigned char *readbuf,unsigned int n)
{
        unsigned int i,j=0;
        PageList *temp_read , *temp_write;
//	temp_read =head_read ;
//	temp_write =head_write;
        /*
           for(i=0;i<n;i++)
           {
           if(j%10 == 0)
           j++;
           temp_read =head_read+i;
           temp_write =head_write+i;
           temp_read->startPageID =temp_write->startPageID =pageid+j;
           temp_read->OffsetBytes =temp_write->OffsetBytes =((i+1)%2)*2048;
           temp_read->Bytes =temp_write->Bytes =2048;
           temp_read->pData =readbuf;
           temp_write->pData =writebuf;

           readbuf+=temp_read->Bytes;
           writebuf+=temp_write->Bytes;
           if(i>0 && i<n){
           singlelist_add(&(head_read[i-1].head),&(temp_read->head));
           singlelist_add(&(head_write[i-1].head),&(temp_write->head));
           }
           if(i%8)			
           j++;
           }
           */
        for(i = 0; i < n; i++) {
                temp_read = head_read + i;
                temp_write = head_write + i;
                temp_read->startPageID = temp_write->startPageID = pageid + j;
                switch(i%6) {
                        case 0:
                        case 1:
                        case 3:
                                temp_read->OffsetBytes = temp_write->OffsetBytes = 0;
                                temp_read->Bytes = temp_write->Bytes = 512;
                                break;
                        case 2:
                        case 4:
                                temp_read->OffsetBytes = temp_write->OffsetBytes = 512;
                                temp_read->Bytes = temp_write->Bytes = 1536;
                                break;
                        case 5:
                                temp_read->OffsetBytes = temp_write->OffsetBytes = 2048;
                                temp_read->Bytes = temp_write->Bytes = 2048;
                                break;
                }
                temp_read->pData =readbuf;
                temp_write->pData =writebuf;

                readbuf+=temp_read->Bytes;
                writebuf+=temp_write->Bytes;
                if(i > 0 && i < n) {
                        singlelist_add(&(head_read[i-1].head),&(temp_read->head));
                        singlelist_add(&(head_write[i-1].head),&(temp_write->head));
                }
                switch(i%6) {
                        case 0:
                                j++;
                                break;
                        case 1:
                                break;
                        case 2:
                                j++;
                                break;
                        case 3:
                                break;
                        case 4:
                                break;
                        case 5:
                                j += 2;
                                break;
                }
        }

//      dump_pagelist(head_write);

        return 0;	
}  

static inline int creat_spl_pagelist(unsigned int pageid, unsigned char *writebuf ,unsigned char *readbuf,unsigned int n)
{
        unsigned int i,j=0;
        PageList *temp_read , *temp_write;
        for(i=0;i<n;i++)
        {
                temp_read =head_read+i;
                temp_write =head_write+i;
                temp_read->startPageID =temp_write->startPageID =pageid+j;
                if(i >= 28){
                        temp_read->OffsetBytes =temp_write->OffsetBytes = ((i+3)%8)*512;
                        if((i+4)%8 == 0)
                                j++;
                }else{
                        temp_read->OffsetBytes =temp_write->OffsetBytes = (i%8)*512;
                        if((i+1)%8 == 0)
                                j++;
                }
                temp_read->Bytes =temp_write->Bytes =512;
                temp_read->pData =readbuf;
                temp_write->pData =writebuf;

                readbuf+=temp_read->Bytes;
                writebuf+=temp_write->Bytes;
                if(i>0 && i<n){
                        singlelist_add(&(head_read[i-1].head),&(temp_read->head));
                        singlelist_add(&(head_write[i-1].head),&(temp_write->head));
                }
                if(i==28){
                        pageid +=384;
                        j=0;
                }

        }
//      dump_pagelist(head_write);
        return 0;	
}

static inline void check_pagelist(PageList *pagelist)
{
        struct singlelist *listhead;
        while(pagelist)
        {
                if(pagelist->retVal <0){
                        dprintf("pagelist->startpageid =%d ,offset =%d ,retval =%d",pagelist->startPageID,pagelist->OffsetBytes,pagelist->retVal);
                        break;
                }
                dprintf("pagelist->retval =0x%x\n",pagelist->retVal);
                listhead = (pagelist->head).next;
                if(!listhead)
                        return;
                pagelist = singlelist_entry(listhead,PageList,head);
//		pagelist =pagelist->Next;
        }
}

int buf_check(unsigned char *org, unsigned char *obj, int size)
{
        int i;
        for (i=0; i<size; i++)
        {
                if (org[i] != obj[i])
                {
                        eprintf("\n %s: Error! data%d=0x%x:0x%x \n", __FUNCTION__, i, org[i], obj[i]);
                        return -1;
                }
        }
        dprintf("\n %s: No Error! \n", __FUNCTION__);
        return 0;
}

int checkFF(unsigned char *org, int size)
{
        int i;
        for (i=0; i<size; i++)
        {
                if (org[i] != 0xFF)
                {
                        eprintf("\n %s: Error! data%d=0x%x\n", __FUNCTION__, i, org[i]);
                        return -1;
                }
        }
        dprintf("\n %s: No Error! \n", __FUNCTION__);
        return 0;
}

void dump_buf(unsigned char *buf, int len)
{
        int i;

        for (i=0; i<len; i++) {
                dprintf("0x%02x, ", buf[i]);
                if ((i + 1) % 16 == 0)
                        dprintf("\n");
        }
        dprintf("\n");
}

void dump_data(unsigned char *buf)
{
        dprintf("0x%02x\n", *buf);
}


void buf_init(unsigned char *buf, int len)
{
        int i;
        for (i = 0; i < len; i++)
        {
                buf[i] = i % 256;
        }
}

unsigned int *malloc_buf(void)
{
        unsigned int *buf;

        buf = (unsigned int *)kmalloc(1024*1024, GFP_KERNEL);/*get 1024KB buf*/

        return buf;
}

int spl_test(PPartition *ppt,unsigned int block, unsigned int *buf, unsigned int ppb)
{
        int ret = 0,i=0;
        unsigned char *writebuf = (unsigned char *)buf;
        unsigned int writesize =ppt->byteperpage;
        unsigned char *readbuf = (unsigned char *)(buf + writesize * 3);
        struct platform_nand_partition *pnand_pt =(struct platform_nand_partition *)ppt->prData;
        unsigned int v_ppb=0;
        BlockList blocklist;
        while(i<8){		
                ret =vnand_interface->iIsBadBlock(ppt,block+i);
                if(ret)
                        dprintf("block %d is badblock\n",block+i);
                else
                        dprintf("block %d is goodblock\n",block+i);
                i++;
        }
        blocklist.startBlock =block;
        blocklist.BlockCount =5;
        blocklist.head.next =0;
        nand_test_erase(ppt,&blocklist);
        if(pnand_pt->use_planes)
                v_ppb =ppb*2;
        /*init databuf with 1 2 3 ... */
        buf_init(writebuf, writesize * ppb);
        creat_spl_pagelist(v_ppb*block,writebuf,readbuf,ppb);

        ret = nand_test_write_pages(ppt,head_write);
        if(ret <0){
                check_pagelist(head_write);
                return 0;
        }
        dprintf("\n******nand_test_write_pages OK*****\n");
        /*clear buf for read*/
        memset(readbuf,0xFF, writesize*ppb);

        ret = nand_test_read_pages(ppt,head_read);
        if(ret){
                check_pagelist(head_read);
                return 0;
        }
        buf_check(writebuf, readbuf, writesize*ppb);

        return ret;
}

int page_test(PPartition *ppt,unsigned int block, unsigned int *buf, unsigned int ppb, unsigned char data)
{
        int ret = 0;
        unsigned char *writebuf = (unsigned char *)buf;
        unsigned int writesize = ppt->byteperpage;
        unsigned char *readbuf = (unsigned char *)(buf + writesize * 3);
        BlockList blocklist;
        int i;
        int j = 0;

        ret =vnand_interface->iIsBadBlock(ppt,block);
        if(ret)
                dprintf("block %d is badblock\n",block);
        else
                dprintf("block %d is goodblock\n",block);

        blocklist.startBlock = block;
        blocklist.BlockCount =1;
        blocklist.head.next =0;
/*
//      g_vnand.operator->MultiBlockErase(&blocklist);
        ret = nand_test_erase(ppt,&blocklist);
        if(ret != 0){
                printk("erase %d block err!\n",block);
                return ret;
        }
*/
        memset(writebuf, data, writesize);
        buf_init(writebuf, 1024);
//while (j--) {
//for (j=0;j<ppt->pageperblock;j++) {
//        printk("test pageid = %d\n",j);
//        ret = nand_test_write_page(ppt, j + block * ppt->pageperblock,0,writesize,(unsigned int *)writebuf);
//        if(ret < 0)
//                dprintf("\n******  page_test  nand_test_write_page wrong ; ret =%d  *****\n",ret);
        //clear buf for read
        memset(readbuf, 0xff, writesize*2);
        ret = nand_test_read_page(ppt, 11 + block * ppt->pageperblock,0,writesize,(unsigned int *)readbuf);
        if(ret) {
                dprintf("\n******   page_test  nand_test_read_page wrong ; ret =%d  *****\n",ret);
                        for(i = 0; i < ppt->byteperpage; i++) {
                                if(i % 8 == 0)
                                        printk("\n");
                                printk("%02x ",readbuf[i]);
                        }
                        printk("\n");
                        while(1);
        }
//        buf_check(writebuf, readbuf, writesize);
        dprintf("\n******   readbuf_phy_addr = 0x%08x writebuf_phy_addr = 0x%08x*****\n",
                        CPHYSADDR(readbuf),CPHYSADDR(writebuf));
//        dump_buf(writebuf,2048);	
//        dump_buf(readbuf,2048);	
        dprintf("\n******   page_test  ok   ok   ok  *****\n");
        /* If you want know the data,please open these.*/
//}
        return ret;
}

int block_test_pages(PPartition *ppt,unsigned int block, unsigned int *buf, unsigned int ppb)
{
        int ret = 0;
        unsigned char *writebuf = (unsigned char *)buf;
        unsigned int writesize =ppt->byteperpage;
        unsigned char *readbuf = (unsigned char *)buf+writesize * ppb;
        struct platform_nand_partition *pnand_pt =(struct platform_nand_partition *)ppt->prData;
        unsigned int v_ppb=ppb;
        BlockList blocklist;
        dprintf("\n****** go into block_test_pages  *****\n");
        /*	ret =g_vnand.operator->IsBadBlock(block);
                if(ret)
                dprintf("block %d is badblock\n",block);
                else
                dprintf("block %d is goodblock\n",block);
                ret =g_vnand.operator->IsBadBlock(block+1);
                if(ret)
                dprintf("block %d is badblock\n",block+1);
                else
                dprintf("block %d is goodblock\n",block+1);
                */	
        blocklist.startBlock =block;
        blocklist.BlockCount =2;
        blocklist.head.next =0;
        nand_test_erase(ppt,&blocklist);

        printk("test blockid = %d\n",block);
        if(pnand_pt->use_planes)
                v_ppb =ppb*2;
        /*init databuf with 1 2 3 ... */
        buf_init(writebuf, writesize * ppb);
        creat_pagelist(v_ppb*block,writebuf,readbuf,ppb*2);

        dprintf("\n******nand_test_write_pages start*****\n");
        ret = nand_test_write_pages(ppt,head_write);
        dprintf("\n******nand_test_write_pages finish*****\n");
        if(ret <0){
                check_pagelist(head_write);
                return 0;
        }
        dprintf("\n******nand_test_write_pages OK*****\n");
//	while(1);
        /*clear buf for read*/
        memset(readbuf,0xFF, writesize*ppb);

        ret = nand_test_read_pages(ppt,head_read);
        if(ret){
                check_pagelist(head_read);
                return 0;
        }

        buf_check(writebuf, readbuf, writesize*ppb);

        return ret;
}
static inline void dump_ppartarray(PPartArray *tmp)
{
        unsigned int i;
        PPartition *ppart =tmp->ppt;
        for(i=0;i < tmp->ptcount;i++)
        {
                dprintf("PPartition : name = %s\n",ppart[i].name);	
                dprintf("PPartition : startblockID = %d\n",ppart[i].startblockID);	
                dprintf("PPartition : pageperblock = %d\n",ppart[i].pageperblock);	
                dprintf("PPartition : byteperpage = %d\n",ppart[i].byteperpage);	
                dprintf("PPartition : totalblocks = %d\n",ppart[i].totalblocks);	
                dprintf("PPartition : badblockcount = %d\n",ppart[i].badblockcount);	
                dprintf("PPartition : startPage = %d\n",ppart[i].startPage);	
                dprintf("PPartition : PageCount = %d\n",ppart[i].PageCount);	
                dprintf("PPartition : mode = %d\n",ppart[i].mode);	
        }
}
static VNandManager vnand_manager;
static PPartArray partarray;
void Register_NandDriver(NandInterface *ni)
{
        int ret;	
        unsigned int *databuf;
        unsigned int ppb;
//	unsigned int i=0;
        PPartition *tmp_ppart=0;

        dprintf("\nDEBUG nand:nand_test_init *******\n");
        vnand_manager.pt =&partarray;
        vnand_interface = ni;
        vnand_interface->iInitNand((void *)&vnand_manager);

        dprintf("VNandManager : StartBlockID = %d\n",vnand_manager.info.startBlockID);	
        dprintf("VNandManager : PagePerBlock = %d\n",vnand_manager.info.PagePerBlock);	
        dprintf("VNandManager : BytePerPage = %d\n",vnand_manager.info.BytePerPage);	
        dprintf("VNandManager : TotalBlocks = %d\n",vnand_manager.info.TotalBlocks);	
        dprintf("VNandManager : MaxBadBlockCount = %d\n",vnand_manager.info.MaxBadBlockCount);	
        dprintf("VNandManager : hwsector = %d\n",vnand_manager.info.hwSector);	

        ppb = vnand_manager.info.PagePerBlock;

        dump_ppartarray(vnand_manager.pt);
        tmp_ppart =vnand_manager.pt->ppt;

        dprintf("\nDEBUG nand:nand_test_init ok  *******\n");
        databuf = malloc_buf();
        if(!databuf)
        {
                eprintf("nand_test: error!get databuf failed.\n");
        }
//#define test_spl_page
#ifdef test_spl_page
        ret = spl_test(&tmp_ppart[0],0, databuf, ppb);

        if(ret != 0)
        {
                eprintf("test_spl_page error.\n");
        }
#endif
#define test_1_page
#ifdef test_1_page
{
        unsigned char data;
        int i,j;
        /*
        while(1) {
                for(j=0;j<tmp_ppart[4].totalblocks;j++) {
                        for (i = 0; i < 4; i++) {
                                if (i % 2 == 0)
                                        data = 0x00;
                                else
                                        data = 0xff;
                                ret = page_test(&tmp_ppart[4],j, databuf, tmp_ppart[4].pageperblock, data);
                                if(ret != 0)
                                {
                                        eprintf("test_1_page ok ret=%d.\n",ret);
                                }
                        }
                }
        }
        */
        printk("pt name = %s\n",tmp_ppart[4].name);
        ret = page_test(&tmp_ppart[4],101, databuf, tmp_ppart[4].pageperblock, data);
        if(ret != 0) {
                eprintf("test_1_page ok ret=%d.\n",ret);
        }
}
#endif

//#define test_1p_block
#ifdef test_1p_block
                for(j=256;j<512;j++){
                        ret = block_test_pages(&tmp_ppart[4],j, databuf, tmp_ppart[4].pageperblock);//tmp_ppart[3].pageperblock);
                        if(ret != 0)
                        {
                                eprintf("test_1p_block error.\n");
                        }
                }

#endif
//	dprintf("\n****** g_vnand.operator->DeInitNand  = 0x%x  *****\n",(unsigned int)g_vnand.operator->DeInitNand);
        vnand_interface->iDeInitNand((void *)&vnand_manager);
}
