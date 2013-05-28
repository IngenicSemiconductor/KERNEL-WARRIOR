/*
 *  lib_nand/nand_ecc.c
 *
 *  ECC utils.
 */

#include "nand_api.h"
static JZ_ECC *pnand_ecc;
int g_eccbuf_len = 0;
int g_ret = 0;
//extern void dump_buf(unsigned char *databuf,int len);

extern void bch_enable(NAND_BASE *host, unsigned int mode,unsigned int eccsize,unsigned int eccbit);
extern void bch_encoding_nbit(NAND_BASE *host,unsigned int n);
extern void bch_decoding_nbit(NAND_BASE *host,unsigned int n);
extern void bch_disable(NAND_BASE *host);
extern void bch_encode_sync(NAND_BASE *host);
extern void bch_decode_sync(NAND_BASE *host);
extern void bch_decode_sdmf(NAND_BASE *host);
extern void bch_encints_claer(NAND_BASE *host);
extern void bch_decints_clear(NAND_BASE *host);
extern void bch_cnt_set(NAND_BASE *host,unsigned int blk,unsigned int par);

/*        nand ecc operation  */
/*
   static inline unsigned int *get_ecc_code_buffer(unsigned int len)
   {
   int start = g_eccbuf_index;
   g_eccbuf_index += (len + 3) / 4;
   if (g_eccbuf_index > ECC_CODE_BUFF_LEN)
   {
   g_eccbuf_index = 0;
   start = 0;	
   }
   return (&pnand_ecc->ecc_code[start]);
   }*/
static inline unsigned char *get_ecc_code_buffer(unsigned int len)
{
	if(!pnand_ecc->peccbuf)
		return 0;
	if(len > g_eccbuf_len)
		return 0;
	return pnand_ecc->peccbuf;
}

static inline void free_bch_buffer(unsigned int len)
{
	len = (len > g_eccbuf_len) ? g_eccbuf_len : len;
	memset(pnand_ecc->peccbuf,0xff,len);
}

/**
 * bch_decode_enable -
 * @eccsize:	Data bytes per ECC step
 * @parsize:	ECC bytes per ECC step
 */
static inline void bch_decode_enable(NAND_BASE *host,unsigned int eccbit)    
{
	bch_enable(host,BCH_DECODE, pnand_ecc->eccsize,eccbit);
}

/**
  bch_encode_enable -
 * @eccsize:	Data bytes per ECC step
 */
static inline void bch_encode_enable(NAND_BASE *host,unsigned int eccbit)
{
	bch_enable(host,BCH_ENCODE, pnand_ecc->eccsize,eccbit);
}

/**
 * bch_correct
 * @dat:        data to be corrected
 * @idx:        the index of error bit in an eccsize
 */
static inline void bch_correct(unsigned char *dat, unsigned int idx)
{
	int eccsize = pnand_ecc->eccsize;
	int i, bits;		/* the 'bits' of i unsigned short is error */
	unsigned short *ptmp =(unsigned short *)dat;

	i = idx & BCH_ERR_INDEX_MASK;                           
	bits = (idx & BCH_ERR_MASK_MASK)>>BCH_ERR_MASK_BIT;

//	dprintf("error:i=%d unsigned short, bits=%d\n",i,bits);

	if (i < (eccsize>>1)){
		ptmp[i] ^= bits;
	}
}

static inline void new_bch_correct(void *dat, unsigned int idx)
{
	int eccsize = pnand_ecc->eccsize;
	int i, bits;		/* the 'bits' of i unsigned short is error */
	int index_s,index_b;
	unsigned short *ptmp;
	struct EccSector *sector =(struct EccSector *)dat;

	i = idx & BCH_ERR_INDEX_MASK;                           
	bits = (idx & BCH_ERR_MASK_MASK)>>BCH_ERR_MASK_BIT;
	index_s = i / 256;  // 512'bytes sector num
	index_b = i % 256;  // unsigned short num  in a 512'bytes  sector

//	dprintf("error:i=%d unsigned short, bits=%d\n",i,bits);

	if (i < (eccsize>>1)){
		ptmp = (unsigned short *)(sector[index_s].buf);
		ptmp[index_b] ^= bits;
	}
}
/**
 * bch_decode_setup_data
 * @databuf:	data to be corrected
 * @eccbuf:	pointer to ecc buffer calculated when nand writing
 */
static inline void bch_decode_setup_data(NAND_BASE *host,unsigned char *databuf, unsigned char *eccbuf,unsigned int eccbit)
{
	int k;
	int eccsize = pnand_ecc->eccsize;
	int eccbytes = __bch_cale_eccbytes(eccbit);

	/* Write data to BCH_DR */
	for (k = 0; k < eccsize; k++) {
		bch_writeb(host->bch_iomem,BCH_DR,databuf[k]);
	}
	/* Write parities to BCH_DR */
	for (k = 0; k < eccbytes; k++) {
		bch_writeb(host->bch_iomem,BCH_DR,eccbuf[k]);
	}	
}

static inline void bch_decode_complete(NAND_BASE *host)
{
	/* Wait for completion */
	bch_decode_sync(host);
}

static inline int bch_decode_correct(NAND_BASE *host,unsigned char *databuf)
{
	int i;
	unsigned int stat;
	volatile unsigned int *errs;
	unsigned int errbits =0;
	unsigned int errcnt =0;
	/* Check decoding */
	stat = bch_readl(host->bch_iomem,BCH_INTS);
//	dprintf("NAND: BHINTS = 0x%x\n",stat);
	if(stat & BCH_INTS_ALLf){
		g_ret =ALL_FF;
		return g_ret;
	}		
	if (stat & BCH_INTS_UNCOR) {
//		dprintf("NAND: Uncorrectable ECC error--   stat = 0x%x\n",stat);
		g_ret =ECC_ERROR;
		return g_ret;
	} else {
		if (stat & BCH_INTS_ERR) {
			/* Error occurred */

			errcnt = (stat & BCH_INTS_ERRC_MASK) >> BCH_INTS_ERRC_BIT;
			errbits = (stat & BCH_INTS_TERRC_MASK) >> BCH_INTS_TERRC_BIT;

			/*begin at the second DWORD*/
			errs = (volatile unsigned int *)(host->bch_iomem+BCH_ERR0);
			for (i = 0; i < errcnt; i++)
			{
				bch_correct(databuf, errs[i]);
			}
		}
	}
//	printk(" ecc error bits is %d !!!!!!!!!!!!!!!!\n",errcnt);
	return errbits;
}
static int new_bch_decode_correct(NAND_BASE *host,void *buf)
{
	int i;
	unsigned int stat;
	volatile unsigned int *errs;
	unsigned int errbits =0;
	unsigned int errcnt =0;
	/* Check decoding */
	stat = bch_readl(host->bch_iomem,BCH_INTS);
//	dprintf("NAND: BHINTS = 0x%x\n",stat);
	if(stat & BCH_INTS_ALLf){
		g_ret =ALL_FF;
		return g_ret;
	}		
	if (stat & BCH_INTS_UNCOR) {
		dprintf("NAND: Uncorrectable ECC error--   stat = 0x%x\n",stat);
		g_ret =ECC_ERROR;
		return g_ret;
	} else {
		if (stat & BCH_INTS_ERR) {
			/* Error occurred */

			errcnt = (stat & BCH_INTS_ERRC_MASK) >> BCH_INTS_ERRC_BIT;
			errbits = (stat & BCH_INTS_TERRC_MASK) >> BCH_INTS_TERRC_BIT;

			/*begin at the second DWORD*/
			errs = (volatile unsigned int *)(host->bch_iomem+BCH_ERR0);
			for (i = 0; i < errcnt; i++)
			{
				new_bch_correct(buf, errs[i]);
			}
		}
	}
	return errbits;
}

/**
 * bch_correct_cpu
 * @databuf:	data to be corrected
 * @eccbuf:	pointer to ecc buffer calculated when nand writing
 */
static inline int bch_correct_cpu(NAND_BASE *host,unsigned char *databuf, unsigned char *eccbuf,unsigned int eccbit)
{	
	int ret=0;
	bch_decode_setup_data(host,databuf,eccbuf,eccbit);
	bch_decode_complete(host);
	ret =bch_decode_correct(host,databuf);
	bch_decints_clear(host);
	bch_disable(host);

	return ret;
}

static inline int ecc_decode_onestep(NAND_BASE *host,unsigned char *databuf,unsigned  char *eccbuf,unsigned int eccbit)
{		
	bch_decode_enable(host,eccbit);
	return (bch_correct_cpu(host,databuf, eccbuf,eccbit));
}

static inline int ecc_do_decode(NAND_BASE *host,unsigned char *databuf, unsigned char *eccbuf, unsigned int eccbit, unsigned int steps)
{
	unsigned int i,errcnt=0;
	unsigned int eccsize = pnand_ecc->eccsize;
	unsigned int eccbytes = __bch_cale_eccbytes(eccbit);
	int ret = 0;

	for (i = 0; i < steps; i++) {
//		dprintf(" ecc decode  i =%d \n  **************\n",i);
//		dump_buf(databuf,eccsize);
//		dump_buf(eccbuf,eccbytes);
		ret = ecc_decode_onestep(host,databuf, eccbuf,eccbit);
		if (ret < 0){
			return ret; //Uncorrectable error
		}
		else
			errcnt = (errcnt < ret) ? ret : errcnt;
		databuf += eccsize;
		eccbuf += eccbytes;
	}
	return errcnt;    //return the largest number of corrective bits
}

static inline void ecc_encode_onestep(NAND_BASE *host,unsigned char *databuf,unsigned  char *eccbuf, unsigned int eccbytes,unsigned int eccbit)
{
	volatile unsigned char *paraddr = (volatile unsigned char *)(host->bch_iomem+BCH_PAR0);
	unsigned int i;
	unsigned int eccsize = pnand_ecc->eccsize;

	bch_encode_enable(host,eccbit);

	/* Write data to BCH_DR */
	for (i = 0; i < eccsize; i++) {
		bch_writeb(host->bch_iomem,BCH_DR,databuf[i]);
	}

	bch_encode_sync(host);
	//	dprintf("ecc encode  bch_inits =0x%x ~~~~~~~~~~~\n",bch_readl(host->bch_iomem,BCH_INTS));

	bch_encints_clear(host);
	bch_disable(host);
	for (i = 0; i < eccbytes; i++) {
		eccbuf[i] = *paraddr++;
	}
}
/*********************************************
 * ecc_do_encode   cpu model, the operation per node of enable pagelist
 * @eccbit  : the number of corrected error bits 
 **********************************************/                                          
static inline void ecc_do_encode(NAND_BASE *host,unsigned char *databuf,unsigned char *eccbuf, unsigned int eccbit, unsigned int steps)
{
	unsigned int i;
	unsigned int eccsize;
	unsigned int eccbytes;

	eccsize = pnand_ecc->eccsize;
	eccbytes = __bch_cale_eccbytes(eccbit);

	//clear five lsb of bch_interrupt_status reg

	for (i = 0; i < steps; i++) {
		ecc_encode_onestep(host,databuf,eccbuf,eccbytes,eccbit);
		databuf += eccsize;
		eccbuf += eccbytes;
	}

}
/*
static inline int ecc_finish(void)
{
	int stat = g_ret;
	g_ret = 0;
	return stat;
}
*/
/**
 * ecc_init -
 */
static inline void ecc_init(void *nand_ecc, void *flash_type)
{

	NAND_FLASH_DEV *type = (NAND_FLASH_DEV *)flash_type;

	pnand_ecc = (JZ_ECC *)nand_ecc;

	if((type == 0) || (pnand_ecc == 0))
		eprintf("ecc_init: error!!type = 0x%x,pnand_ecc = 0x%x\n",(unsigned int)type,(unsigned int)pnand_ecc);

	pnand_ecc->get_ecc_code_buffer = get_ecc_code_buffer;
	pnand_ecc->free_bch_buffer =free_bch_buffer;
	pnand_ecc->ecc_enable_encode = ecc_do_encode;
	pnand_ecc->ecc_enable_decode = ecc_do_decode;
	pnand_ecc->bch_decode_correct = new_bch_decode_correct;
//	pnand_ecc->ecc_finish = ecc_finish;

	pnand_ecc->eccsize = type->eccblock;
#ifdef CONFIG_NAND_BCH_ECCSIZE_512
	pnand_ecc->eccsize = 512;
#endif
#ifdef CONFIG_NAND_BCH_ECCSIZE_1024
	pnand_ecc->eccsize = 1024;
#endif

#if defined(CONFIG_HW_BCH_64BIT)
	pnand_ecc->eccbit = 64;
#elif defined(CONFIG_HW_BCH_60BIT)
	pnand_ecc->eccbit = 60;
#elif defined(CONFIG_HW_BCH_56BIT)
	pnand_ecc->eccbit = 56;
#elif defined(CONFIG_HW_BCH_52BIT)
	pnand_ecc->eccbit = 52;
#elif defined(CONFIG_HW_BCH_48BIT)
	pnand_ecc->eccbit = 48;
#elif defined(CONFIG_HW_BCH_44BIT)
	pnand_ecc->eccbit = 44;
#elif defined(CONFIG_HW_BCH_40BIT)
	pnand_ecc->eccbit = 40;
#elif defined(CONFIG_HW_BCH_36BIT)
	pnand_ecc->eccbit = 36;
#elif defined(CONFIG_HW_BCH_32BIT)
	pnand_ecc->eccbit = 32;
#elif defined(CONFIG_HW_BCH_28BIT)
	pnand_ecc->eccbit = 28;
#elif defined(CONFIG_HW_BCH_24BIT)
	pnand_ecc->eccbit = 24;
#elif defined(CONFIG_HW_BCH_20BIT)
	pnand_ecc->eccbit= 20;
#elif defined(CONFIG_HW_BCH_16BIT)
	pnand_ecc->eccbit = 16;
#elif defined(CONFIG_HW_BCH_12BIT)
	pnand_ecc->eccbit = 12;
#elif defined(CONFIG_HW_BCH_8BIT)
	pnand_ecc->eccbit = 8;
#elif defined(CONFIG_HW_BCH_4BIT)
	pnand_ecc->eccbit = 4;
#else
	pnand_ecc->eccbit = type->eccbit;
#endif

	pnand_ecc->eccpos = NAND_ECC_POS;

	g_eccbuf_len =(type->pagesize / pnand_ecc->eccsize +1) * __bch_cale_eccbytes(BCH_ENABLE_MAXBITS) +1024;  
	pnand_ecc->peccbuf =(unsigned char *)nand_malloc_buf(g_eccbuf_len);
	if(!pnand_ecc->peccbuf)
		eprintf("ERROR: pnand_ecc->peccbuf malloc Failed\n");
	memset(pnand_ecc->peccbuf,0xff,g_eccbuf_len);

}

JZ_ECC jz_default_ecc = 
{
	.ecc_init = ecc_init,
};
