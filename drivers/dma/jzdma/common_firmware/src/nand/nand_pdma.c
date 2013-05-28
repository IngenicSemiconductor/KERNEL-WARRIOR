/*
 * src/nand_pdma.c
 */
#include <common.h>
#include <pdma.h>
#include <nand.h>

#include <asm/jzsoc.h>

void pdma_nand_read_data(struct nand_chip *nand, struct nand_pipe_buf *pipe_buf,
			unsigned char *par_buf)
{
	unsigned pipe_sel = nand->pipe_cnt & 0x1;

	if (nand->mode & PNAND_NEMC) {
		nemc_channel_cfg(nand->nand_io, (pipe_buf + pipe_sel)->pipe_data,
				nand->eccsize, NEMC_TO_TCSM);
		__pdmac_special_channel_launch(PDMA_NEMC_CHANNEL);
	}

	pipe_sel ^= 1;
/*
	pipe_buf->pipe_data[0] = 0xff;
	pipe_buf->pipe_data[1] = 0xff;
*/	if (nand->mode & PNAND_BCH_DEC)
		pdma_bch_correct(nand, pipe_buf + pipe_sel, par_buf);
}

void pdma_nand_read_oob(struct nand_chip *nand, unsigned char *oob_buf)
{
	nemc_channel_cfg(nand->nand_io, oob_buf, nand->eccpos + nand->ecctotal, NEMC_TO_TCSM);
	__pdmac_special_channel_launch(PDMA_NEMC_CHANNEL);
}

void pdma_nand_write_data(struct nand_chip *nand, struct nand_pipe_buf *pipe_buf,
			unsigned char *ddr_addr)
{
	unsigned pipe_sel = nand->pipe_cnt & 0x1;

	if (nand->mode & PNAND_DDR) {
		ddr_channel_cfg(ddr_addr, (pipe_buf + pipe_sel)->pipe_data,
				nand->eccsize, DDR_TO_TCSM);
		__pdmac_channel_launch(PDMA_DDR_CHANNEL);
//			*(unsigned int *)(nand->mcu_test) = nand->pipe_cnt | (0x14<<16);
	}

	pipe_sel ^= 1;

	if (nand->mode & PNAND_NEMC) {
		nemc_channel_cfg((pipe_buf + pipe_sel)->pipe_data, nand->nand_io,
				nand->eccsize, TCSM_TO_NEMC);
		__pdmac_special_channel_launch(PDMA_NEMC_CHANNEL);
//		*(unsigned int *)(nand->mcu_test) = nand->pipe_cnt | (8 << 16);
	}
}

void pdma_nand_write_oob(struct nand_chip *nand, unsigned char *oob_buf)
{
	int i;

	for (i = 0; i < nand->eccpos; i++)
		oob_buf[i] = 0xff;

	nemc_channel_cfg(oob_buf, nand->nand_io, nand->eccpos + nand->ecctotal, TCSM_TO_NEMC);
	__pdmac_special_channel_launch(PDMA_NEMC_CHANNEL);
}
