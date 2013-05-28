/*
 * irq_mcu.h
 */
#ifndef __IRQ_MCU_H__
#define __IRQ_MCU_H__

#define MCU_SOFT_IRQ		(1 << 2)
#define MCU_CHANNEL_IRQ		(1 << 1)
#define MCU_INTC_IRQ		(1 << 0)

extern void irq_bch_calculate_handle(struct nand_chip *nand, struct nand_pipe_buf *pipe_buf,
			unsigned char *par_buf);
extern int irq_bch_correct_handle(struct nand_chip *nand, struct nand_pipe_buf *pipe_buf);

extern void pdma_nand_irq_handle(struct nand_chip *nand, struct nand_pipe_buf *pipe_buf,
			unsigned char *oob_buf);
extern void pdma_msg_irq_handle(struct nand_chip *nand, struct nand_pipe_buf *pipe_buf,
			struct pdma_msg *msg, unsigned char *oob_buf);

#endif /* __IRQ_MCU_H__ */
