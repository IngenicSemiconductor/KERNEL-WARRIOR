/*
 *  lib_nand/nand_api.c
 *
 * JZ Nand Driver API
 *
 */
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/interrupt.h>
//#include <include/linuxver.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/math64.h>
#include <linux/completion.h>
#include <mach/jznand.h>
#include "nand_api.h"
#include "vnandinfo.h"   //change later
#include "nand_char.h"   //change later
#include "nand_debug.h"   //change later

//#define DEBUG_ERASE
//#define NAPI_DEBUG_TIME_WRITE
//#define NAPI_DEBUG_TIME_READ
#ifdef JZNAND_DRIVE_DEBUG
#define NAND_API_DUMP
#endif

#if defined(NAPI_DEBUG_TIME_WRITE) || defined(NAPI_DEBUG_TIME_READ)
#define NAPI_DBG_READ	0
#define NAPI_DBG_WRITE	1
#define BYTE_PER_PAGE	8192
#define DEBUG_TIME_BYTES (10 * 1024 *1024)
struct __data_distrib {
	unsigned int _0KBytes;		// x == 0
	unsigned int _0_2KBytes;	// 0 < x < 2
	unsigned int _2KBytes;		// x == 2
	unsigned int _2_4KBytes;	// 2 < x < 4
	unsigned int _4KBytes;		// x == 4
	unsigned int _4_6KBytes;	// 4 < x < 6
	unsigned int _6KBytes;		// x == 6
	unsigned int _6_8KBytes;	// 6 < x < 8
	unsigned int _8KBytes;		// x == 8
};

static unsigned long long rd_btime = 0;
static unsigned long long wr_btime = 0;
static unsigned long long rd_sum_time = 0;
static unsigned long long wr_sum_time = 0;
static unsigned int rd_sum_bytes = 0;
static unsigned int wr_sum_bytes = 0;
static unsigned int rd_sum_pages = 0;
static unsigned int wr_sum_pages = 0;
static struct __data_distrib rd_dbg_distrib = {0};
static struct __data_distrib wr_dbg_distrib = {0};

/*for example: div_s64_32(3,2) = 2*/
static inline int div_s64_32(long long dividend , int divisor);
static inline void calc_bytes(int mode, int bytes)
{
	if (mode == NAPI_DBG_READ)
		rd_sum_bytes += bytes;
	else
		wr_sum_bytes += bytes;
}

static inline void calc_pages(int mode, int pages)
{
	if (mode == NAPI_DBG_READ)
		rd_sum_pages += pages;
	else
		wr_sum_pages += pages;
}

static inline void calc_distrib(int mode, int bytes)
{
	struct __data_distrib *distrib;

	if (mode == NAPI_DBG_READ)
		distrib = &rd_dbg_distrib;
	else
		distrib = &wr_dbg_distrib;

	if (bytes == 0) {
		distrib->_0KBytes ++;
	} else if ((bytes > 0 * 1024) && (bytes < 2 * 1024)) {
		distrib->_0_2KBytes ++;
	} else if (bytes == 2 * 1024) {
		distrib->_2KBytes ++;
	} else if ((bytes > 2 * 1024) && (bytes < 4 * 1024)) {
		distrib->_2_4KBytes ++;
	} else if (bytes == 4 * 1024) {
		distrib->_4KBytes ++;
	} else if ((bytes > 4 * 1024) && (bytes < 6 * 1024)) {
		distrib->_4_6KBytes ++;
	} else if (bytes == 6 * 1024) {
		distrib->_6KBytes ++;
	} else if ((bytes > 6 * 1024) && (bytes < 8 * 1024)) {
		distrib->_6_8KBytes ++;
	} else if (bytes == 8 * 1024) {
		distrib->_8KBytes ++;
	} else {
		printk("%s, line:%d, Bytes error!, bytes = %d\n", __func__, __LINE__, bytes);
	}
}

static inline void begin_time(int mode)
{
	if (mode == NAPI_DBG_READ)
		rd_btime = sched_clock();
	else
		wr_btime = sched_clock();
}

static inline void end_time(int mode)
{
	int times, bytes, theory_bytes;
	unsigned long long etime = sched_clock();

	if (mode == NAPI_DBG_READ) {
		rd_sum_time += (etime - rd_btime);
		if (rd_sum_bytes >= DEBUG_TIME_BYTES) {
#ifdef NAPI_DEBUG_TIME_READ
			times = div_s64_32(rd_sum_time, 1000 * 1000);
			bytes = rd_sum_bytes / 1024;
			theory_bytes = rd_sum_pages * BYTE_PER_PAGE / 1024;
			printk("READ: nand_api speed debug, %dms, %dKb, %d, %dKb/s, %dKb/s\n",
				   times, bytes, rd_sum_pages, (bytes * 1000) / times, (theory_bytes * 1000) / times);
			printk("[    0KBytes]:(%d)\n[0 - 2KBytes]:(%d)\n[    2KBytes]:(%d)\n[2 - 4KBytes]:(%d)\n",
				   rd_dbg_distrib._0KBytes, rd_dbg_distrib._0_2KBytes,
				   rd_dbg_distrib._2KBytes, rd_dbg_distrib._2_4KBytes);
			printk("[    4KBytes]:(%d)\n[4 - 6KBytes]:(%d)\n[    6KBytes]:(%d)\n[6 - 8KBytes]:(%d)\n",
				   rd_dbg_distrib._4KBytes, rd_dbg_distrib._4_6KBytes,
				   rd_dbg_distrib._6KBytes, rd_dbg_distrib._6_8KBytes);
			printk("[    8KBytes]:(%d)\n\n", rd_dbg_distrib._8KBytes);
#endif
			rd_sum_bytes = rd_sum_time = rd_sum_pages = 0;
			memset(&rd_dbg_distrib, 0, sizeof(struct __data_distrib));
		}
	} else {
		wr_sum_time += (etime - wr_btime);
		if (wr_sum_bytes >= DEBUG_TIME_BYTES) {
#ifdef NAPI_DEBUG_TIME_WRITE
			times = div_s64_32(wr_sum_time, 1000 * 1000);
			bytes = wr_sum_bytes / 1024;
			theory_bytes = wr_sum_pages * BYTE_PER_PAGE / 1024;
			printk("WRITE: nand_api speed debug, %dms, %dKb, %d, %dKb/s, %dKb/s\n",
				   times, bytes, wr_sum_pages, (bytes * 1000) / times, (theory_bytes * 1000) / times);
			printk("[    0KBytes]:(%d)\n[0 - 2KBytes]:(%d)\n[    2KBytes]:(%d)\n[2 - 4KBytes]:(%d)\n",
				   wr_dbg_distrib._0KBytes, wr_dbg_distrib._0_2KBytes,
				   wr_dbg_distrib._2KBytes, wr_dbg_distrib._2_4KBytes);
			printk("[    4KBytes]:(%d)\n[4 - 6KBytes]:(%d)\n[    6KBytes]:(%d)\n[6 - 8KBytes]:(%d)\n",
				   wr_dbg_distrib._4KBytes, wr_dbg_distrib._4_6KBytes,
				   wr_dbg_distrib._6KBytes, wr_dbg_distrib._6_8KBytes);
			printk("[    8KBytes]:(%d)\n\n", wr_dbg_distrib._8KBytes);
#endif
			wr_sum_bytes = wr_sum_time = wr_sum_pages = 0;
			memset(&wr_dbg_distrib, 0, sizeof(struct __data_distrib));
		}
	}
}
#endif

static inline int div_s64_32(long long dividend , int divisor)  // for example: div_s64_32(3,2) = 2
{
	long long result=0;
	int remainder=0;
	result =div_s64_rem(dividend,divisor,&remainder);
	result = remainder ? (result +1):result;
	return result;
}

void setup_mtd_struct(NAND_API *pnand_api, JZ_NAND_CHIP *pnand_chip)
{
	NAND_API *this = pnand_api;

	this->writesize = pnand_chip->writesize;
	this->erasesize = pnand_chip->erasesize;
	this->ppblock = pnand_chip->ppblock;
	this->totalblock = pnand_chip->bpchip * pnand_chip->numchips;

	/*
	 * Set the number of read / write steps for one page depending on ECC
	 * mode
	 */

	this->nand_ecc->eccsteps = this->writesize / this->nand_ecc->eccsize;
	if (this->nand_ecc->eccsteps * this->nand_ecc->eccsize != this->writesize) {
		printk(KERN_WARNING "Invalid ecc parameters\n");
		BUG();
	}
}

#ifdef NAND_API_DUMP
/**
 * ffs_ll - find first bit set in a 64bit word.
 * @word: The word to search
 */
static inline int ffs_ll(unsigned long long word)
{
	unsigned int low = word & 0xffffffff;
	unsigned int high = word >> 32;
	int i;

	for(i = 0; i < 32; i++) {
		if (low & 0x1)
			break;
		low >>= 1;
	}
	if (i == 32) {
		for(i = 0; i < 32; i++) {
			if (high & 0x1)
				break;
			high >>= 1;
		}
		i += 32;
	}
	if (i == 64)
		return 0;
	else
		return (i + 1);
}


void dump_nand_chip(JZ_NAND_CHIP *pnand_chip)
{
	dprintf("NAND chip:0x%x\n",(int)pnand_chip);

	dprintf("  pagesize    %d\n", pnand_chip->pagesize);
	dprintf("  oobsize     %d\n", pnand_chip->oobsize);
	dprintf("  blocksize    %d\n", pnand_chip->blocksize);
	dprintf("  ppblock    %d\n", pnand_chip->ppblock);
	dprintf("  realplanenum   %d\n", pnand_chip->realplanenum);
	dprintf("  buswidth   %d\n", pnand_chip->buswidth);
	dprintf("  row_cycles     %d\n", pnand_chip->row_cycles);
	dprintf("  numchips     %d\n", pnand_chip->numchips);
	dprintf("  badblockpos     %d\n", pnand_chip->badblockpos);
	dprintf("  planenum     %d\n", pnand_chip->planenum);
	dprintf("  freesize     %d\n", pnand_chip->freesize);
	dprintf("  writesize     %d\n", pnand_chip->writesize);
	dprintf("  erasesize     %d\n", pnand_chip->erasesize);
	dprintf("  chipsize     %lld\n", pnand_chip->chipsize);
	dprintf("^_^\n");

}

static void dump_nand_io(JZ_IO *pnand_io)
{
	dprintf("NAND io:0x%x\n",(int)pnand_io);

	dprintf("  dataport    0x%x\n", (int)pnand_io->dataport);
	dprintf("  cmdport     0x%x\n", (int)pnand_io->cmdport);
	dprintf("  addrport    0x%x\n", (int)pnand_io->addrport);
	dprintf("  buswidth    %d\n", pnand_io->buswidth);
	dprintf("  pagesize    %d\n", pnand_io->pagesize);
	dprintf("-_-\n");
}

static void dump_nand_ecc(JZ_ECC *pnand_ecc)
{
	dprintf("NAND ecc:0x%x\n",(int)pnand_ecc);
	dprintf("  eccsize    %d\n", pnand_ecc->eccsize);
	dprintf("  eccbytes     %d\n", pnand_ecc->eccbytes);
	dprintf("  parsize    %d\n", pnand_ecc->parsize);
	dprintf("  eccbit    %d\n", pnand_ecc->eccbit);
	dprintf("  eccpos    %d\n", pnand_ecc->eccpos);
	dprintf("  eccsteps   %d\n", pnand_ecc->eccsteps);
	dprintf("-_-\n");
}

void dump_nand_api(NAND_API *pnand_api)
{
	dprintf("NAND API:\n");
	dprintf("  writesize=%d\n", pnand_api->writesize);
	dprintf("  erasesize=%d\n", pnand_api->erasesize);
	dprintf("  ppblock=%d\n", pnand_api->ppblock);
	dprintf("  totalblock=%d\n", pnand_api->totalblock);
	dprintf("  nemc:0x%x\n",(unsigned int)(pnand_api->nand_ctrl));
	dprintf("  ecc:0x%x\n",(unsigned int)(pnand_api->nand_ecc));
	dprintf("  io:0x%x\n",(unsigned int)(pnand_api->nand_io));
	dprintf("  chip:0x%x\n",(unsigned int)(pnand_api->nand_chip));
	dprintf("o_o\n");
}
#endif

/*************       nand  configure        ************/
#define GPIOA_20_IRQ (0*32 + 20)
static struct completion nand_rb;
DECLARE_WAIT_QUEUE_HEAD(nand_rb_queue);

extern JZ_NAND_CHIP jz_raw_nand;
extern JZ_IO        jznand_io;
extern JZ_ECC       jznand_ecc;
extern NAND_CTRL    jznand_nemc;

/*********            nand api             ********/

static NAND_API  g_pnand_api;
static Aligned_List * g_aligned_list;
static struct platform_device *g_pdev;
static struct platform_nand_data *g_pnand_data;
static PPartArray g_partarray;
static PPartition *g_partition = NULL;

/*
 * nand_board_init
 */
static int nand_board_init(NAND_API * pnand_api)
{
	int ret =0;
	pnand_api->nand_ecc =&jznand_ecc;
	pnand_api->nand_io = &jznand_io;

#ifdef CONFIG_TOGGLE_NAND
	pnand_api->nand_chip =&jz_toggle_nand;
#else
	pnand_api->nand_chip =&jz_raw_nand;
#endif
	pnand_api->nand_ctrl =&jznand_nemc;

	ret = nand_chip_init(g_pnand_api.vnand_base,pnand_api);
	if (ret == -1) {
		printk("ERROR: NAND Chip Init Failed\n");
		return -1;
	}
#ifdef CONFIG_NAND_DMA
	ret =nand_dma_init(pnand_api);    //add later
#endif
	return ret;
}
/*
 * nand_board_deinit
 */
static void nand_board_deinit(NAND_API * pnand_api)
{
#ifdef CONFIG_NAND_DMA
	nand_dma_deinit(pnand_api->nand_dma);    //add later
#endif
	return ;
}

/*
 * Main initialization routine
 */

/*    nand_wait_rb interrupt handler  */
static irqreturn_t jznand_waitrb_interrupt(int irq, void *dev_id)
{
	complete(&nand_rb);
	return IRQ_HANDLED;
}
/*    nand_wait_rb  */
int nand_wait_rb(void)
{
	unsigned int ret;
	do{
#if 1
	ret = wait_for_completion_timeout(&nand_rb,(msecs_to_jiffies(500)));
#else
	ret =wait_for_completion(&nand_rb);
#endif
        udelay(10);
	}while(ret == -ERESTARTSYS);
	if(!ret)
		return TIMEOUT;
	return 0;
}

/*  nand driver init    */
static inline int multiblock_erase(void *ppartition,BlockList * erase_blocklist);
static inline int init_nand_vm(void * vNand)
{
	VNandManager *tmp_manager =(VNandManager *)vNand;
	VNandInfo *tmp_info = &(tmp_manager->info);
	struct platform_nand_partition *plat_data,*tmpplat;
	PPartition *pt;

	/* platform_nand_partition */
	plat_data = (struct platform_nand_partition *)nand_malloc_buf(sizeof(struct platform_nand_partition));
	if(!g_pnand_data)
		return -1;
	tmpplat = g_pnand_data->partitions;
	plat_data->eccbit = tmpplat->eccbit;  // these eccbit should be equal in every partitions
	plat_data->use_planes = ONE_PLANE;
	/* PPartition */
	pt = (PPartition *)nand_malloc_buf(sizeof(PPartition));
	pt->startblockID = 0;
	pt->pageperblock = g_pnand_api.nand_chip->ppblock;
	pt->byteperpage = g_pnand_api.nand_chip->pagesize;
	pt->totalblocks = g_pnand_api.totalblock;
	pt->badblockcount = 0;
	pt->actualbadblockcount = 0;
	pt->hwsector = 512;
	pt->startPage = pt->startblockID * pt->pageperblock;
	pt->PageCount = pt->totalblocks * pt->pageperblock;
	pt->prData = plat_data;

	tmp_info->startBlockID = 0;
	tmp_info->PagePerBlock = g_pnand_api.nand_chip->ppblock;
	tmp_info->BytePerPage =  g_pnand_api.nand_chip->pagesize;
	tmp_info->TotalBlocks =  g_pnand_api.totalblock;
	tmp_info->MaxBadBlockCount = g_pnand_api.nand_chip->maxbadblockcount;
	tmp_info->hwSector = g_pnand_api.nand_ecc->eccsize;
	tmp_info->prData = (void *)pt;
	if(!g_partition)
		return -1;
	tmp_manager->pt = &g_partarray;
	dprintf("*** init_nand_vm is ok ***\n");
	return 0;
}
static int init_nand_driver(void)
{
	static PPartition *t_partition;  // temporary partition

	int ret;
	int j;//record count for ex_partition
	int ipartition_num=0;
	struct  platform_nand_partition *ptemp=0;
	unsigned int *tmp_badblock_info=0;
	int tmp_eccbit=0, tmp_freesize=0;
	int tmp_oobsize=0,tmp_eccpos=0,tmp_eccbytes=0;
	int blockid =0;
	int lastblockid =0;
	unsigned int use_planenum=0;
	unsigned int pageperblock = 0;
	unsigned int byteperpage =  0;
	unsigned int totalblocks =  0;
	unsigned int hwsector = 0;

	ret =nand_board_init(&g_pnand_api);
	if(ret){
		eprintf("ERROR: nand_board_init Failed\n");
		goto init_nand_driver_error1;
	}
	setup_mtd_struct(&g_pnand_api,g_pnand_api.nand_chip);
	if(g_pnand_api.nand_chip->pagesize < g_pnand_api.nand_ecc->eccsize){
		eprintf("ERROR: pagesize < eccsize\n");
		goto init_nand_driver_error1;
	}
	pageperblock = g_pnand_api.nand_chip->ppblock;
	byteperpage =  g_pnand_api.nand_chip->pagesize;
	totalblocks =  g_pnand_api.totalblock;
	hwsector = g_pnand_api.nand_ecc->eccsize;
#ifdef NAND_API_DUMP
	dump_nand_api(&g_pnand_api);
	dump_nand_chip(g_pnand_api.nand_chip);
	dump_nand_io(g_pnand_api.nand_io);
	dump_nand_ecc(g_pnand_api.nand_ecc);
#endif
	tmp_oobsize =g_pnand_api.nand_chip->oobsize;
	tmp_eccpos  =g_pnand_api.nand_ecc->eccpos;
	tmp_badblock_info =(unsigned int *)g_pnand_data->priv;

	ipartition_num =g_pnand_data->nr_partitions;
	ptemp =g_pnand_data->partitions;
	/*  add a partition that stored badblock tabel */
	g_partition =(PPartition *)nand_malloc_buf((ipartition_num+1)*(sizeof(PPartition)));
	if (!g_partition) {
		eprintf("ERROR: g_partition malloc Failed\n");
		goto init_nand_driver_error1;
	}
	memset(g_partition, 0x0, (ipartition_num+1)*(sizeof(PPartition)));

	for(ret=0; ret<ipartition_num; ret++)
	{
		tmp_eccbit =(ptemp+ret)->eccbit;
		if (tmp_eccbit < g_pnand_api.nand_ecc->eccbit)
			tmp_eccbit = g_pnand_api.nand_ecc->eccbit;
		(ptemp+ret)->eccbit = tmp_eccbit;
		/**  cale freesize  **/
		tmp_eccbytes =__bch_cale_eccbytes(tmp_eccbit);  //eccbytes per eccstep
		if(((byteperpage/hwsector)*tmp_eccbytes)+tmp_eccpos > tmp_oobsize)
			tmp_freesize =hwsector;
		else
			tmp_freesize =0;
		/*   cale ppartition vnand info     */
		use_planenum = (ptemp+ret)->use_planes ? g_pnand_api.nand_chip->planenum : 1;
		(g_partition+ret)->name = (ptemp+ret)->name;
		(g_partition+ret)->hwsector =512;
		(g_partition+ret)->byteperpage = byteperpage-tmp_freesize;
		(g_partition+ret)->actualbadblockcount = 0;
		(g_partition+ret)->startblockID = div_s64_32((ptemp+ret)->offset,((g_partition+ret)->byteperpage * pageperblock));
		/*  two-plane operation : startblockID must be even  */
		(g_partition+ret)->startblockID +=(g_partition+ret)->startblockID % use_planenum;
		(g_partition+ret)->startPage = (g_partition+ret)->startblockID  * pageperblock;
		(g_partition+ret)->pageperblock = pageperblock * use_planenum;
		(g_partition+ret)->PageCount = div_s64_32(((ptemp+ret)->size),((g_partition+ret)->byteperpage));
		(g_partition+ret)->totalblocks = (g_partition+ret)->PageCount / (g_partition+ret)->pageperblock;
		(g_partition+ret)->PageCount = (g_partition+ret)->totalblocks * (g_partition+ret)->pageperblock;
		(g_partition+ret)->badblockcount = ((g_partition+ret)->totalblocks
                                * g_pnand_api.nand_chip->maxbadblockcount
                                + g_pnand_api.nand_chip->bpchip - 1) / g_pnand_api.nand_chip->bpchip;

		if ((g_partition+ret)->badblockcount < tmp_badblock_info[ret])
			(g_partition+ret)->badblockcount = tmp_badblock_info[ret];

		(g_partition+ret)->mode = (ptemp+ret)->mode;
		(g_partition+ret)->prData = (void *)(ptemp+ret);

		j = 0;
		while(((ptemp+ret)->ex_partition+j)->size != 0){
		    ((g_partition+ret)->parts+j)->startblockID =
			    div_s64_32(((ptemp+ret)->ex_partition+j)->offset, ((g_partition+ret)->byteperpage * pageperblock));
		    ((g_partition+ret)->parts+j)->totalblocks =
			    div_s64_32(((ptemp+ret)->ex_partition+j)->size, ((g_partition+ret)->byteperpage * pageperblock));
		    ((g_partition+ret)->parts+j)->name = ((ptemp+ret)->ex_partition+j)->name;
		    j++;
		}
		(g_partition+ret)->parts_num= j;

		if(byteperpage-tmp_freesize == 0){
			eprintf("ERROR: pagesize equal 0\n");
			goto init_nand_driver_error2;
		}
		/*  blockid is physcial block id  */
		blockid = (g_partition+ret)->startblockID + (g_partition+ret)->PageCount / pageperblock;
		if(blockid > (totalblocks) || lastblockid > (g_partition +ret)->startblockID){
			eprintf("ERROR: nand capacity insufficient\n");
			goto init_nand_driver_error2;
		}
		lastblockid = blockid;
	}
	/*  add partition of badblock tabel;the block,which is subtracted from last partition,
	 *   is as badblock partition.
	 */
	t_partition = g_partition+ipartition_num-1;  //last partiton from board
	(g_partition+ret)->name = "nderror";
	(g_partition+ret)->byteperpage = t_partition->byteperpage;
	(g_partition+ret)->badblockcount = (g_partition + ret - 1)->badblockcount;//tmp_badblock_info[ret];
	(g_partition+ret)->actualbadblockcount = 0;
	(g_partition+ret)->startblockID = blockid;
	(g_partition+ret)->startPage = (g_partition+ret)->startblockID  * pageperblock;
	(g_partition+ret)->pageperblock = t_partition->pageperblock;
	(g_partition+ret)->totalblocks = 4;
	(g_partition+ret)->PageCount =(g_partition+ret)->pageperblock * (g_partition+ret)->totalblocks;
	(g_partition+ret)->mode = ONCE_MANAGER;   // this is special mark of badblock tabel partition
	(g_partition+ret)->prData = t_partition->prData;
	(g_partition+ret)->hwsector =512;
	g_partarray.ppt = g_partition;
	g_partarray.ptcount = ipartition_num+1;

	g_aligned_list =nand_malloc_buf((VNANDCACHESIZE + 2048) / 512 *sizeof(Aligned_List));
	if (!g_aligned_list){
		eprintf("ERROR: g_aligned_list malloc Failed\n");
		goto init_nand_driver_error2;
	}
	memset(g_aligned_list,0,(VNANDCACHESIZE + 2048) / 512 * sizeof(Aligned_List));

	nand_ops_parameter_init();

#ifdef DEBUG_ERASE
	{
		int ret;
		BlockList blocklist;
        blocklist.startBlock = 0;
        blocklist.BlockCount = 128;
        blocklist.head.next = 0;
		ret = multiblock_erase(&g_partition[0], &blocklist);
		if (ret != 0) {
			printk("ndisk erase err \n");
			dump_stack();
			while(1);
		}
		blocklist.startBlock = 0;
		blocklist.BlockCount = 128;
		blocklist.head.next = 0;
		ret = multiblock_erase(&g_partition[1], &blocklist);
		if (ret != 0) {
			printk("mdisk erase err \n");
			dump_stack();
			while(1);
		}
	}
#endif

	dprintf("INFO: nand init finish\n");
	return 0;
init_nand_driver_error2:
	nand_free_buf(g_partition);
init_nand_driver_error1:
	return -1;
}

/*
 * page_aligned -- pagelist will be transformed into aligned_list,
 *
 */
#if defined(NAPI_DEBUG_TIME_WRITE) || defined(NAPI_DEBUG_TIME_READ)
static inline void page_aligned(PageList * pagelist, Aligned_List * aligned_list, int rwflag)
#else
static inline void page_aligned(PageList * pagelist, Aligned_List * aligned_list)
#endif
{
	struct singlelist *listhead;
	PageList *pnextpagelist=0;
	int i=0;
	//memset(aligned_list,0,sizeof(Aligned_List));
	aligned_list->pagelist =pagelist;
	aligned_list->opsmodel =1;
	aligned_list->next =0;
	i++;
	listhead = (pagelist->head).next;
#if defined(NAPI_DEBUG_TIME_WRITE) || defined(NAPI_DEBUG_TIME_READ)
	calc_bytes(rwflag, pagelist->Bytes);
	calc_distrib(rwflag, pagelist->Bytes);
	calc_pages(rwflag, 1);
#endif
	while(listhead)
	{
		pnextpagelist = singlelist_entry(listhead,PageList,head);
		switch(pnextpagelist->startPageID - pagelist->startPageID){
			case 0:
				pagelist = pnextpagelist;
				aligned_list->opsmodel++;
				break;
			case 1:
				if(!(pagelist->startPageID %2))
					aligned_list->opsmodel |= 1<<24;
			default :
				aligned_list->next = aligned_list+1;
				aligned_list =aligned_list->next;
				//memset(aligned_list,0,sizeof(Aligned_List));
				pagelist = pnextpagelist;
				aligned_list->pagelist =pagelist;
				aligned_list->opsmodel =1;
				aligned_list->next =0;
#if defined(NAPI_DEBUG_TIME_WRITE) || defined(NAPI_DEBUG_TIME_READ)
				calc_pages(rwflag, 1);
#endif
				break;
		}
		i++;
		listhead = (pagelist->head).next;
#if defined(NAPI_DEBUG_TIME_WRITE) || defined(NAPI_DEBUG_TIME_READ)
		calc_bytes(rwflag, pagelist->Bytes);
		calc_distrib(rwflag, pagelist->Bytes);
#endif
	}
	return;
}

/*
 * page_read - read one page data
 * @page: the page which will be read
 * @databuf: the data buffer will return the page data
 */
static inline int page_read(void *ppartition,int pageid, int offset,int bytes,void * databuf)
{
	int ret;
	PPartition * tmp_ppt = (PPartition *)ppartition;
	//	struct platform_nand_partition * tmp_pf = (struct platform_nand_partition *)tmp_ppt->prData;
	if((pageid < 0) || (pageid > tmp_ppt->PageCount)){
		eprintf("ERROR: pageid(%d) is more than totalpage(%d)\n",pageid, tmp_ppt->PageCount);
		return ENAND;   //return pageid error
	}
#ifdef CONFIG_NAND_DMA
	g_pnand_api.nand_dma->ppt = tmp_ppt;
	ret =nand_dma_read_page(&g_pnand_api,pageid,offset,bytes,databuf);
#else
	nand_ops_parameter_reset(tmp_ppt);
	ret =nand_read_page(g_pnand_api.vnand_base,pageid,offset,bytes,databuf);
#endif
	return ret;
}

/*
 * multipage_read - read many pages data
 *
 */
static inline int multipage_read(void *ppartition,PageList * read_pagelist)
{
	int ret;
	PPartition * tmp_ppt = (PPartition *)ppartition;
	struct platform_nand_partition * tmp_pf = (struct platform_nand_partition *)tmp_ppt->prData;
	unsigned char tmp_part_attrib = tmp_pf->part_attrib;
#if defined(NAPI_DEBUG_TIME_WRITE) || defined(NAPI_DEBUG_TIME_READ)
	begin_time(NAPI_DBG_READ);
#endif
	if(!read_pagelist){
		dev_err(&g_pdev->dev," nand read_pagelist is null\n");
		return -1;
	}

#if defined(NAPI_DEBUG_TIME_WRITE) || defined(NAPI_DEBUG_TIME_READ)
	page_aligned(read_pagelist,g_aligned_list, NAPI_DBG_READ);
#else
	page_aligned(read_pagelist,g_aligned_list);
#endif

	if(tmp_part_attrib == PART_XBOOT){
		nand_ops_parameter_reset(tmp_ppt);
		ret =read_spl(g_pnand_api.vnand_base,g_aligned_list);
	}else{
#ifdef CONFIG_NAND_DMA
                g_pnand_api.nand_dma->ppt = tmp_ppt;
                ret =nand_dma_read_pages(&g_pnand_api,g_aligned_list);
#else
		nand_ops_parameter_reset(tmp_ppt);
		ret =nand_read_pages(g_pnand_api.vnand_base,g_aligned_list);
#endif
	}
#if defined(NAPI_DEBUG_TIME_WRITE) || defined(NAPI_DEBUG_TIME_READ)
	end_time(NAPI_DBG_READ);
#endif
	return ret;
}

/*
 * panic_page_write - write one page data no schedule
 * @page: the page which will be read
 * @databuf: the data buffer will return the page data
 * add by yqwang
 */
static inline int panic_page_write(void *ppartition,int pageid,int offset,int bytes,void * databuf)
{
	int ret; 
	PPartition * tmp_ppt = (PPartition *)ppartition;
	//      struct platform_nand_partition * tmp_pf = (struct platform_nand_partition *)tmp_ppt->prData;
	if((pageid < 0) || (pageid > tmp_ppt->PageCount)) {
		eprintf("ERROR: pageid(%d) is more than totalpage(%d)\n",pageid, tmp_ppt->PageCount);
		return ENAND;   //return pageid error
	}

	nand_ops_parameter_reset(tmp_ppt);
	ret =panic_nand_write_page(g_pnand_api.vnand_base,pageid,offset,bytes,databuf);
	return ret; 
}

/*
 * page_write - write one page data
 * @page: the page which will be read
 * @databuf: the data buffer will return the page data
 */
static inline int page_write(void *ppartition,int pageid,int offset,int bytes,void * databuf)
{
	int ret;
	PPartition * tmp_ppt = (PPartition *)ppartition;
	//	struct platform_nand_partition * tmp_pf = (struct platform_nand_partition *)tmp_ppt->prData;
	if((pageid < 0) || (pageid > tmp_ppt->PageCount)){
		eprintf("ERROR: pageid(%d) is more than totalpage(%d)\n",pageid, tmp_ppt->PageCount);
		return ENAND;   //return pageid error
	}
#ifdef CONFIG_NAND_DMA
        g_pnand_api.nand_dma->ppt = tmp_ppt;
        ret =nand_dma_write_page(&g_pnand_api,pageid,offset,bytes,databuf);
#else
	nand_ops_parameter_reset(tmp_ppt);
	ret =nand_write_page(g_pnand_api.vnand_base,pageid,offset,bytes,databuf);
#endif
	return ret;
}
/*
 * multipage_write - read many pages data
 *
 */
static inline int multipage_write(void *ppartition,PageList * write_pagelist)
{
	int ret;
	PPartition * tmp_ppt = (PPartition *)ppartition;
	struct platform_nand_partition * tmp_pf = (struct platform_nand_partition *)tmp_ppt->prData;
	unsigned char tmp_part_attrib = tmp_pf->part_attrib;
#if defined(NAPI_DEBUG_TIME_WRITE) || defined(NAPI_DEBUG_TIME_READ)
	begin_time(NAPI_DBG_WRITE);
#endif
	if(!write_pagelist){
		dev_err(&g_pdev->dev," nand write_pagelist is null\n");
		return -1;
	}

#if defined(NAPI_DEBUG_TIME_WRITE) || defined(NAPI_DEBUG_TIME_READ)
	page_aligned(write_pagelist,g_aligned_list, NAPI_DBG_WRITE);
#else
	page_aligned(write_pagelist,g_aligned_list);
#endif

	if(tmp_part_attrib == PART_XBOOT){
		nand_ops_parameter_reset(tmp_ppt);
		ret =write_spl(g_pnand_api.vnand_base,g_aligned_list);
	}else{
		//printk("multipage write start\n");
#ifdef CONFIG_NAND_DMA
		//printk("multipage write set ppt\n");
		g_pnand_api.nand_dma->ppt = tmp_ppt;
		//printk("multipage dma write\n");
		ret =nand_dma_write_pages(&g_pnand_api,g_aligned_list);
#else
		nand_ops_parameter_reset(tmp_ppt);
		ret =nand_write_pages(g_pnand_api.vnand_base,g_aligned_list);
#endif
	}
#if defined(NAPI_DEBUG_TIME_WRITE) || defined(NAPI_DEBUG_TIME_READ)
	end_time(NAPI_DBG_WRITE);
#endif
	return ret;
}

/*
 * multiblock_erase - erase blocks
 *
 */
static inline int multiblock_erase(void *ppartition,BlockList * erase_blocklist)
{
	int ret;
	PPartition * tmp_ppt = (PPartition *)ppartition;
	//	struct platform_nand_partition * tmp_pf = (struct platform_nand_partition *)tmp_ppt->prData;
	//	unsigned char tmp_part_attrib = tmp_pf->part_attrib;
	if(!erase_blocklist){
		eprintf("ERROR: %s[%d] blocklist is NULL\n",__func__,__LINE__);
		return -1;
	}
	nand_ops_parameter_reset(tmp_ppt);
	ret =nand_erase_blocks(g_pnand_api.vnand_base,erase_blocklist);
	g_pnand_api.nand_dma->cache_phypageid = -1;
	return ret;
}

/*
 * is_badblock - judge badblock
 *
 */
static inline int is_badblock(void *ppartition,int blockid)
{
	PPartition * tmp_ppt = (PPartition *)ppartition;
	if(blockid >tmp_ppt->totalblocks){
		eprintf("ERROR: blockid(%d) is more than totalblocks(%d)\n",blockid, tmp_ppt->totalblocks);
		return -1;
	}
	nand_ops_parameter_reset(tmp_ppt);
	return isbadblock(g_pnand_api.vnand_base,blockid);
}

/*
 * is_inherent_badblock - judge inherent badblock
 *
 */
static int is_inherent_badblock(void *ppartition,int blockid)
{
	PPartition * tmp_ppt = (PPartition *)ppartition;
	if(blockid >tmp_ppt->totalblocks){
		eprintf("ERROR: blockid(%d) is more than totalblocks(%d)\n",blockid, tmp_ppt->totalblocks);
		return -1;
	}
	nand_ops_parameter_reset(tmp_ppt);
	return isinherentbadblock(g_pnand_api.vnand_base,blockid);
}

/*
 * mark_badblock - mark badblock
 *
 */
static inline int mark_badblock(void *ppartition,int blockid)
{
	PPartition * tmp_ppt = (PPartition *)ppartition;
	if(blockid <0 || blockid >g_pnand_api.totalblock){
		eprintf("ERROR: blockid(%d) is more than totalblocks(%d)\n",blockid, tmp_ppt->totalblocks);
		return -1;
	}
	nand_ops_parameter_reset(tmp_ppt);
	return markbadblock(g_pnand_api.vnand_base,blockid);
}

/*
 * deinit_nand
 *
 */
static inline int deinit_nand(void *vNand)
{
	nand_board_deinit(&g_pnand_api);

	nand_free_buf(g_pnand_api.vnand_base);
	g_pnand_api.vnand_base =0;
	nand_free_buf(g_pnand_api.pnand_base);
	g_pnand_api.vnand_base =0;
	nand_free_buf(g_partition);
	g_partition =0;
	nand_free_buf(g_aligned_list);
	g_aligned_list =0;

	free_irq(gpio_to_irq(g_pnand_api.pnand_base->rb_irq),NULL);

	dprintf("\ndeinit_nand  nand_free_buf ok  *********\n");
	return 0;
}

/*********************************************/
/******         nand driver register    ******/
/*********************************************/

NandInterface jz_nand_interface = {
	.iInitNand = init_nand_vm,
	.iPageRead = page_read,
	.iPageWrite = page_write,
	.iMultiPageRead = multipage_read,
	.iMultiPageWrite = multipage_write,
	.iMultiBlockErase = multiblock_erase,
	.iIsBadBlock = is_badblock,
	.iIsInherentBadBlock = is_inherent_badblock,
	.iMarkBadBlock = mark_badblock,
	.iDeInitNand = deinit_nand,
	.iPanicPageWrite = panic_page_write,
};
/*
 * Probe for the NAND device.
 */
static int __devinit plat_nand_probe(struct platform_device *pdev)
{
	struct resource         *regs;
	int             irq;
	int ret=0;

	dprintf("INFO: Nand driver start...\n");
	g_pdev = pdev;
	g_pnand_api.pdev = (void *)pdev;
	g_pnand_data = (struct platform_nand_data *)(pdev->dev.platform_data);
	if (!g_pnand_data) {
		dev_err(&pdev->dev, "No platform_data\n");
                return -1;
	}
        g_pnand_api.gpio_wp = g_pnand_data->gpio_wp;
        if (!g_pnand_api.gpio_wp) {
		dev_err(&pdev->dev, "No set gpio wp pin\n");
                return -1;
        }
	if (gpio_request(g_pnand_api.gpio_wp, "nand_wp")) {
		dev_err(&pdev->dev, "the gpio wp pin is err\n");
                return -1;
        }
        /* disable nand 'WP' */
        gpio_direction_output(g_pnand_api.gpio_wp, 1);

	g_pnand_api.vnand_base =(NAND_BASE *)nand_malloc_buf(sizeof(NAND_BASE));
	if(!g_pnand_api.vnand_base){
		dev_err(&pdev->dev,"Malloc virtual nand base info \n");
		goto nand_probe_error1;
	}
	g_pnand_api.pnand_base =(NAND_BASE *)nand_malloc_buf(sizeof(NAND_BASE));
	if(!g_pnand_api.pnand_base){
		dev_err(&pdev->dev,"Malloc physical nand base info \n");
		goto nand_probe_error2;
	}
	/* get nemc clock and bch clock */
	g_pnand_api.vnand_base->nemc_gate =clk_get(&pdev->dev,"nemc");
	clk_enable(g_pnand_api.vnand_base->nemc_gate);
	g_pnand_api.vnand_base->bch_gate =clk_get(&pdev->dev,"bch");
	clk_enable(g_pnand_api.vnand_base->bch_gate);
	g_pnand_api.vnand_base->bch_clk =clk_get(&pdev->dev,"cgu_bch");
        clk_disable(g_pnand_api.vnand_base->bch_clk);
        clk_set_rate(g_pnand_api.vnand_base->bch_clk, clk_get_rate(g_pnand_api.vnand_base->nemc_gate));
	clk_enable(g_pnand_api.vnand_base->bch_clk);

	/*   nemc resource  */
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "No nemc iomem resource\n");
		goto nand_probe_error3;
	}
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "No nemc irq resource\n");
		goto nand_probe_error3;
	}
	g_pnand_api.vnand_base->nemc_iomem =ioremap(regs->start, resource_size(regs));
	g_pnand_api.pnand_base->nemc_iomem =(void __iomem *)regs->start;
	g_pnand_api.vnand_base->nemc_irq = g_pnand_api.pnand_base->nemc_irq=irq;
	/*  bch resource  */
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!regs) {
		dev_err(&pdev->dev, "No bch iomem resource\n");
		goto nand_probe_error3;
	}
	irq = platform_get_irq(pdev, 1);
	if (irq < 0) {
		dev_err(&pdev->dev, "No bch irq resource\n");
		goto nand_probe_error3;
	}
	g_pnand_api.vnand_base->bch_iomem =ioremap(regs->start, resource_size(regs));
	g_pnand_api.pnand_base->bch_iomem =(void __iomem *)regs->start;
	g_pnand_api.vnand_base->bch_irq = g_pnand_api.pnand_base->bch_irq=irq;
	/*  pdma resource  */
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!regs) {
		dev_err(&pdev->dev, "No pdma iomem resource\n");
		goto nand_probe_error3;
	}
	irq = platform_get_irq(pdev, 2);
	if (irq < 0) {
		dev_err(&pdev->dev, "No pdma irq resource\n");
		goto nand_probe_error3;
	}
	g_pnand_api.vnand_base->pdma_iomem =ioremap(regs->start, resource_size(regs));
	g_pnand_api.pnand_base->pdma_iomem =(void __iomem *)regs->start;
	g_pnand_api.vnand_base->irq = g_pnand_api.pnand_base->irq=irq;

	/*  nand chip port resource  */
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	if (!regs) {
		dev_err(&pdev->dev, "No nand_chip iomem resource\n");
		goto nand_probe_error3;
	}
	g_pnand_api.vnand_base->nemc_cs6_iomem =ioremap(regs->start, resource_size(regs));
	g_pnand_api.pnand_base->nemc_cs6_iomem =(void __iomem *)regs->start;

	/*   nand rb irq request */
	if (gpio_request_one(g_pnand_api.pnand_base->irq,
				GPIOF_DIR_IN, "nand_rb")) {
		dev_err(&pdev->dev, "No nand_chip iomem resource\n");
		goto nand_probe_error3;
	}
	irq = gpio_to_irq(g_pnand_api.pnand_base->irq);
	ret = request_irq(irq,jznand_waitrb_interrupt,IRQF_DISABLED | IRQF_TRIGGER_RISING,
			"jznand-wait-rb",NULL);
	if (ret) {
		dev_err(&g_pdev->dev,"request detect irq-%d fail\n",gpio_to_irq(GPIOA_20_IRQ));
		goto nand_probe_error3;
	}
	g_pnand_api.vnand_base->rb_irq = g_pnand_api.pnand_base->rb_irq = irq;
	init_completion(&nand_rb);

	ret = init_nand_driver();
	if(ret){
		dev_err(&g_pdev->dev,"init_nand_driver failed\n");
		goto nand_probe_error3;
		}

	Register_NandDriver(&jz_nand_interface);

	dprintf("INFO: Nand probe success!\n");
	return 0;
nand_probe_error3:
	nand_free_buf(g_pnand_api.pnand_base);
nand_probe_error2:
	nand_free_buf(g_pnand_api.vnand_base);
nand_probe_error1:
	return -ENXIO;
}

/*
 * Remove a NAND device.
 */
static int __devexit plat_nand_remove(struct platform_device *pdev)
{
	// do nothing
	iounmap(g_pnand_api.vnand_base->nemc_iomem);
	iounmap(g_pnand_api.vnand_base->bch_iomem);
	iounmap(g_pnand_api.vnand_base->pdma_iomem);
	iounmap(g_pnand_api.vnand_base->nemc_cs6_iomem);

	clk_disable(g_pnand_api.vnand_base->bch_gate);
	clk_disable(g_pnand_api.vnand_base->bch_clk);
	clk_disable(g_pnand_api.vnand_base->nemc_gate);
	clk_put(g_pnand_api.vnand_base->bch_gate);
	clk_put(g_pnand_api.vnand_base->bch_clk);
	clk_put(g_pnand_api.vnand_base->nemc_gate);

	return 0;
}

static int plat_nand_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}
static int plat_nand_resume(struct platform_device *dev)
{
	int ret = 0;
#ifdef CONFIG_NAND_DMA
	ret = nand_dma_resume(&g_pnand_api);
#endif
	return ret;
}
static struct platform_driver plat_nand_driver = {
	.probe		= plat_nand_probe,
	.remove		= __exit_p(plat_nand_remove),
	.suspend        = plat_nand_suspend,
	.resume         = plat_nand_resume,
	.driver		= {
		.name	= "jz_nand",
		.owner	= THIS_MODULE,
	},
};

static int __init plat_nand_init(void)
{
	//	return platform_driver_probe(&plat_nand_driver, plat_nand_probe);
	return platform_driver_register(&plat_nand_driver);
}

static void __exit plat_nand_exit(void)
{
	platform_driver_unregister(&plat_nand_driver);
}

#ifdef CONFIG_EARLY_INIT_RUN
rootfs_initcall(plat_nand_init);
#else
module_init(plat_nand_init);
#endif
module_exit(plat_nand_exit);
MODULE_DESCRIPTION("JZ4780 Nand driver");
MODULE_AUTHOR(" ingenic ");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("20120623");
