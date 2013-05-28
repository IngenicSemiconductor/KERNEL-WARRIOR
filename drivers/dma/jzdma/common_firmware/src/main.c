/*
 * main.c
 */
#include <common.h>
#include <pdma.h>
#include <nand.h>
#include <message.h>
#include <irq_mcu.h>

#include <asm/jzsoc.h>

volatile unsigned int channel_irq;

static inline void mcu_init(void)
{
	__pdmac_mnmb_clear();
	__pdmac_msmb_clear();

	/* clear mailbox irq pending */
	__pdmac_mmb_irq_clear();
        __pdmac_mnmb_unmask();
	/* clear irq mask for channel irq */
	__pdmac_channel_irq_unmask();
}

int main(void)
{
	struct nand_chip nand_info;
	struct nand_chip *nand = &nand_info;
	unsigned char *oob_buf = (unsigned char *)TCSM_BANK4;//(unsigned char *)TCSM_BANK4;
	struct nand_pipe_buf pipe_buf[2];
	struct pdma_msg *msg;

	pipe_buf[0].pipe_data = (unsigned char *)TCSM_BANK5;
	pipe_buf[1].pipe_data = (unsigned char *)TCSM_BANK6;
	msg = (struct pdma_msg *)(TCSM_BANK5 - 0x50); // TCSM last bank start

	mcu_init();

	channel_irq = 0;
	nand->ctrl = 0;
	__pdma_irq_enable();

	while (1) {
		__pdma_irq_disable();
		if (!channel_irq)
			__pdma_mwait();
		__pdma_irq_enable();

		pdma_nand_irq_handle(nand, pipe_buf, oob_buf);
		pdma_msg_irq_handle(nand, pipe_buf, msg, oob_buf);
	}
}
