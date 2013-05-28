/*
 * src/bch.c
 */
#include <common.h>
#include <nand.h>
#include <pdma.h>

#include <asm/jzsoc.h>

//#define BCH_DEBUG

#define UNCOR_ECC	0xFFFF

static inline void bch_encode_enable(int ecclevel, int eccsize, int parsize)
{
	__bch_cnt_set(eccsize, parsize);
	__bch_encoding(ecclevel);
}

static inline void bch_decode_enable(int ecclevel, int eccsize, int parsize)
{
	__bch_cnt_set(eccsize, parsize);
	__bch_decoding(ecclevel);
}

/* BCH ENCOGING CFG */
void pdma_bch_calculate(struct nand_chip *nand, struct nand_pipe_buf *pipe_buf)
{
	bch_encode_enable(nand->ecclevel, nand->eccsize, nand->eccbytes);

	/* write DATA to BCH_DR */
	bch_channel_cfg(pipe_buf->pipe_data, (unsigned char *)BCH_DR,
			nand->eccsize, TCSM_TO_BCH);
	__pdmac_special_channel_launch(PDMA_BCH_CHANNEL);
}

/* BCH DECOGING CFG */
void pdma_bch_correct(struct nand_chip *nand, struct nand_pipe_buf *pipe_buf, unsigned char *par_buf)
{
	int i;

	bch_decode_enable(nand->ecclevel, nand->eccsize, nand->eccbytes);

	/* Move Parity Part to follow Current Data */
	for (i = 0; i < nand->eccbytes; i++)
		pipe_buf->pipe_par[i] = par_buf[i];

	/* write DATA and PARITY to BCH_DR */
	bch_channel_cfg(pipe_buf->pipe_data, (unsigned char *)BCH_DR,
			nand->eccsize + nand->eccbytes, TCSM_TO_BCH);
	__pdmac_special_channel_launch(PDMA_BCH_CHANNEL);
}
//----------------------------------------------------------------------------/

/* BCH ENCOGING HANDLE */
static void bch_calculate_handle(struct nand_chip *nand, struct nand_pipe_buf *pipe_buf,
				unsigned char *par_buf)
{
	int i;

	/* get Parity to TCSM */
	__pdma_irq_disable();
	bch_channel_cfg((unsigned char *)BCH_PAR0, pipe_buf->pipe_par, nand->eccbytes, BCH_TO_TCSM);
	__pdmac_special_channel_launch(PDMA_BCH_CHANNEL);

#ifdef BCH_DEBUG
	volatile int timeout = 4000; //for debug
	while (!__pdmac_channel_end_detected(PDMA_BCH_CHANNEL) && timeout--);
#else
	while (!__pdmac_channel_end_detected(PDMA_BCH_CHANNEL));
#endif
	__pdmac_channel_mirq_clear(PDMA_BCH_CHANNEL);
	__pdma_irq_enable();

	/* Move Current Data to Parity Part */
	for (i = 0; i < nand->eccbytes; i++)
		par_buf[i] = pipe_buf->pipe_par[i];
}

void irq_bch_calculate_handle(struct nand_chip *nand, struct nand_pipe_buf *pipe_buf,
				unsigned char *par_buf)
{
	__mbch_encode_sync();
	__bch_disable();

	bch_calculate_handle(nand, pipe_buf, par_buf);
	__bch_encints_clear();
}

/* BCH DECOGING HANDLE */
static void bch_error_correct(struct nand_chip *nand, unsigned short *data_buf,
				unsigned int *err_buf, int err_bit)
{
	unsigned short err_mask;
	u32 idx; /* the 'bit' of idx half-word is error */

	idx = (err_buf[err_bit] & BCH_ERR_INDEX_MASK) >> BCH_ERR_INDEX_BIT;
	err_mask = (err_buf[err_bit] & BCH_ERR_MASK_MASK) >> BCH_ERR_MASK_BIT;

	data_buf[idx] ^= err_mask;
}

static void bch_correct_handle(struct nand_chip *nand, unsigned char *data_buf,
				unsigned int *err_buf, int *report)
{
	unsigned int stat;
	int i, err_cnt;

	/* get BCH Status */
	stat = REG_BCH_INTS;

	if (stat & BCH_INTS_UNCOR) {
		*report = UNCOR_ECC; /* Uncorrectable ECC Error*/
	} else if (stat & BCH_INTS_ERR) {
		err_cnt = (stat & BCH_INTS_ERRC_MASK) >> BCH_INTS_ERRC_BIT;
		*report = err_cnt;

		if (err_cnt) {
			/* read BCH Error Report use Special CH0 */
			__pdma_irq_disable();
			bch_channel_cfg((unsigned char *)BCH_ERR0, (void *)err_buf, err_cnt << 2, BCH_TO_TCSM);
			__pdmac_special_channel_launch(PDMA_BCH_CHANNEL);

#ifdef BCH_DEBUG
			volatile int timeout = 4000; //for debug
			while (!__pdmac_channel_end_detected(PDMA_BCH_CHANNEL) && timeout--);
#else
			while (!__pdmac_channel_end_detected(PDMA_BCH_CHANNEL));
#endif
			__pdmac_channel_mirq_clear(PDMA_BCH_CHANNEL);
			__pdma_irq_enable();

			for (i = 0; i < err_cnt; i++)
				bch_error_correct(nand, (unsigned short *)data_buf, err_buf, i);
		}
	} else {
		*report = 0;
	}
}

void irq_bch_correct_handle(struct nand_chip *nand, struct nand_pipe_buf *pipe_buf)
{
	unsigned int *err_buf = (unsigned int *)(pipe_buf->pipe_par);
	int report;

	__mbch_decode_sync();
	__bch_disable();

	bch_correct_handle(nand, pipe_buf->pipe_data, err_buf, &report);
	__bch_decints_clear();

	nand->report = report != UNCOR_ECC ? 0 : -1;
}
