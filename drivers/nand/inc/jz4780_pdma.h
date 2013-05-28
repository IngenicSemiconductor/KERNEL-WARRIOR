 /*
  * linux/drivers/nand/jz4780_pdma.h - Ingenic PDMA Controller driver head file
  * 
  * Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
  * Written by Feng Gao <fenggao@ingenic.com>.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation.
  */

#ifndef __JZ4780_PDMA_H__
#define __JZ4780_PDMA_H__

#define MAX_DMA_NUM 	32  /* max 32 channels */

/* DMA Channel Register (0 ~ 31) */
#define DMAC_DSAR(n)  (0x00 + (n) * 0x20) /* DMA source address */
#define DMAC_DTAR(n)  (0x04 + (n) * 0x20) /* DMA target address */
#define DMAC_DTCR(n)  (0x08 + (n) * 0x20) /* DMA transfer count */
#define DMAC_DRSR(n)  (0x0C + (n) * 0x20) /* DMA request source */
#define DMAC_DCCSR(n) (0x10 + (n) * 0x20) /* DMA control/status */
#define DMAC_DCMD(n)  (0x14 + (n) * 0x20) /* DMA command */
#define DMAC_DDA(n)   (0x18 + (n) * 0x20) /* DMA descriptor addr */
#define DMAC_DSD(n)   (0x1C + (n) * 0x04) /* DMA Stride Address */

#define DMAC_DMACR    0x1000            /* DMA control register */
#define DMAC_DMAIPR   0x1004            /* DMA interrupt pending */
#define DMAC_DMADBR   0x1008            /* DMA doorbell */
#define DMAC_DMADBSR  0x100C         	/* DMA doorbell set */
#define DMAC_DMACK    0x1010
#define DMAC_DMACKS   0x1014
#define DMAC_DMACKC   0x1018
#define DMAC_DMACP    0x101C		/* DMA Channel Programmable */
#define DMAC_DSIRQP   0x1020		/* DMA Soft IRQ Pending */
#define DMAC_DSIRQM   0x1024		/* DMA Soft IRQ Mask */
#define DMAC_DCIRQP   0x1028		/* DMA Channel IRQ Pending to MCU */
#define DMAC_DCIRQM   0x102C		/* DMA Channel IRQ Mask to MCU */

/* PDMA MCU Control Register */
#define DMAC_DMCS     0x1030		/* DMA MCU Control/Status */
#define DMAC_DMNMB    0x1034		/* DMA MCU Normal Mailbox */
#define DMAC_DMSMB    0x1038		/* DMA MCU Security Mailbox */
#define DMAC_DMINT    0x103C		/* DMA MCU Interrupt */

// DMA request source register
#define DMAC_DRSR_RS_BIT	0
#define DMAC_DRSR_RS_MASK	(0x3f << DMAC_DRSR_RS_BIT)
/* 0~7 is reserved */
#define DMAC_DRSR_RS_AUTO	(8 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_TSSIIN	(9 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_PM1OUT	(0xa << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_PM1IN	(0xb << DMAC_DRSR_RS_BIT)
/* 10 ~ 11 is reserved */
#define DMAC_DRSR_RS_EXTERN	(12 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_UART4OUT	(12 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_UART4IN	(13 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_UART3OUT	(14 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_UART3IN	(15 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_UART2OUT	(16 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_UART2IN	(17 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_UART1OUT	(18 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_UART1IN	(19 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_UART0OUT	(20 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_UART0IN	(21 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_SSI0OUT	(22 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_SSI0IN	(23 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_AICOUT	(24 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_AICIN	(25 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_MSC0OUT	(26 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_MSC0IN	(27 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_MSC1OUT	(28 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_MSC1IN	(29 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_MSC2OUT	(30 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_MSC2IN	(31 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_SSI1OUT	(32 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_SSI1IN	(33 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_PM0OUT	(34 << DMAC_DRSR_RS_BIT)
#define DMAC_DRSR_RS_PM0IN	(35 << DMAC_DRSR_RS_BIT)
/* others are reserved */

// DMA channel control/status register
#define DMAC_DCCSR_NDES		(1 << 31) /* descriptor (0) or not (1) ? */
#define DMAC_DCCSR_DES8    	(1 << 30) /* Descriptor 8 Word */
#define DMAC_DCCSR_DES4    	(0 << 30) /* Descriptor 4 Word */
#define DMAC_DCCS_TOC_BIT	16        /* Time out counter */
#define DMAC_DCCS_TOC_MASK	(0x3fff << DMAC_DCCS_TOC_BIT)
#define DMAC_DCCSR_CDOA_BIT	8         /* copy of DMA offset address */
#define DMAC_DCCSR_CDOA_MASK	(0xff << DMAC_DCCSR_CDOA_BIT)
/* [7:5] reserved */
#define DMAC_DCCSR_AR		(1 << 4)  /* Address Error */
#define DMAC_DCCSR_TT		(1 << 3)  /* Transfer Terminated */
#define DMAC_DCCSR_HLT		(1 << 2)  /* DMA Halted */
#define DMAC_DCCSR_CT		(1 << 1)  /* count terminated */
#define DMAC_DCCSR_EN		(1 << 0)  /* Channel Transfer Enable */

// DMA channel command register
#define DMAC_DCMD_EACKS_LOW  	(1 << 31) /* External DACK Output Level Select, active low */
#define DMAC_DCMD_EACKS_HIGH  	(0 << 31) /* External DACK Output Level Select, active high */
#define DMAC_DCMD_EACKM_WRITE 	(1 << 30) /* External DACK Output Mode Select, output in write cycle */
#define DMAC_DCMD_EACKM_READ 	(0 << 30) /* External DACK Output Mode Select, output in read cycle */
#define DMAC_DCMD_ERDM_BIT      28        /* External DREQ Detection Mode Select */
#define DMAC_DCMD_ERDM_MASK     (0x03 << DMAC_DCMD_ERDM_BIT)
#define DMAC_DCMD_ERDM_LOW	(0 << DMAC_DCMD_ERDM_BIT)
#define DMAC_DCMD_ERDM_FALL	(1 << DMAC_DCMD_ERDM_BIT)
#define DMAC_DCMD_ERDM_HIGH	(2 << DMAC_DCMD_ERDM_BIT)
#define DMAC_DCMD_ERDM_RISE  	(3 << DMAC_DCMD_ERDM_BIT)
/* [31:28] reserved */
#define DMAC_DCMD_SID_BIT	26        /* Source Identification */
#define DMAC_DCMD_SID_MASK	(0x3 << DMAC_DCMD_SID_BIT)
#define DMAC_DCMD_SID(n)	((n) << DMAC_DCMD_SID_BIT)
#define DMAC_DCMD_SID_TCSM	(0 << DMAC_DCMD_SID_BIT)
#define DMAC_DCMD_SID_SPECIAL	(1 << DMAC_DCMD_SID_BIT)
#define DMAC_DCMD_SID_DDR	(2 << DMAC_DCMD_SID_BIT)
#define DMAC_DCMD_DID_BIT	24       /* Destination Identification */
#define DMAC_DCMD_DID_MASK	(0x3 << DMAC_DCMD_DID_BIT)
#define DMAC_DCMD_DID(n)	((n) << DMAC_DCMD_DID_BIT)
#define DMAC_DCMD_DID_TCSM	(0 << DMAC_DCMD_DID_BIT)
#define DMAC_DCMD_DID_SPECIAL	(1 << DMAC_DCMD_DID_BIT)
#define DMAC_DCMD_DID_DDR	(2 << DMAC_DCMD_DID_BIT)
#define DMAC_DCMD_SAI		(1 << 23) /* source address increment */
#define DMAC_DCMD_DAI		(1 << 22) /* dest address increment */
#define DMAC_DCMD_RDIL_BIT	16        /* request detection interval length */
#define DMAC_DCMD_RDIL_MASK	(0x0f << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_RDIL_IGN	(0 << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_RDIL_2	(1 << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_RDIL_4	(2 << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_RDIL_8	(3 << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_RDIL_12	(4 << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_RDIL_16	(5 << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_RDIL_20	(6 << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_RDIL_24	(7 << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_RDIL_28	(8 << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_RDIL_32	(9 << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_RDIL_48	(10 << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_RDIL_60	(11 << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_RDIL_64	(12 << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_RDIL_124	(13 << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_RDIL_128	(14 << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_RDIL_200	(15 << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_SWDH_BIT	14  /* source port width */
#define DMAC_DCMD_SWDH_MASK	(0x03 << DMAC_DCMD_SWDH_BIT)
#define DMAC_DCMD_SWDH_32	(0 << DMAC_DCMD_SWDH_BIT)
#define DMAC_DCMD_SWDH_8	(1 << DMAC_DCMD_SWDH_BIT)
#define DMAC_DCMD_SWDH_16	(2 << DMAC_DCMD_SWDH_BIT)
#define DMAC_DCMD_DWDH_BIT	12  /* dest port width */
#define DMAC_DCMD_DWDH_MASK	(0x03 << DMAC_DCMD_DWDH_BIT)
#define DMAC_DCMD_DWDH_32	(0 << DMAC_DCMD_DWDH_BIT)
#define DMAC_DCMD_DWDH_8	(1 << DMAC_DCMD_DWDH_BIT)
#define DMAC_DCMD_DWDH_16	(2 << DMAC_DCMD_DWDH_BIT)
#define DMAC_DCMD_SWDH(w)   (DMAC_DCMD_SWDH_##w)            //add later
#define DMAC_DCMD_DWDH(w)   (DMAC_DCMD_DWDH_##w)            //add later
/* [11] reserved */
#define DMAC_DCMD_DS_BIT	8   /* Transfer Data Size of a data unit */
#define DMAC_DCMD_DS_MASK	(0x07 << DMAC_DCMD_DS_BIT)
#define DMAC_DCMD_DS_32BIT	(0 << DMAC_DCMD_DS_BIT)
#define DMAC_DCMD_DS_8BIT	(1 << DMAC_DCMD_DS_BIT)
#define DMAC_DCMD_DS_16BIT	(2 << DMAC_DCMD_DS_BIT)
#define DMAC_DCMD_DS_16BYTE	(3 << DMAC_DCMD_DS_BIT)
#define DMAC_DCMD_DS_32BYTE	(4 << DMAC_DCMD_DS_BIT)
#define DMAC_DCMD_DS_64BYTE	(5 << DMAC_DCMD_DS_BIT)
#define DMAC_DCMD_DS_128BYTE	(6 << DMAC_DCMD_DS_BIT)
#define DMAC_DCMD_DS_AUTO	(7 << DMAC_DCMD_DS_BIT)
/* [7:3] reserved */
#define DMAC_DCMD_STDE   	(1 << 2)  /* Stride Disable/Enable */
#define DMAC_DCMD_TIE		(1 << 1)  /* DMA transfer interrupt enable */
#define DMAC_DCMD_LINK		(1 << 0)  /* descriptor link enable */

// DMA descriptor address register
#define DMAC_DDA_BASE_BIT	12  /* descriptor base address */
#define DMAC_DDA_BASE_MASK	(0x0fffff << DMAC_DDA_BASE_BIT)
#define DMAC_DDA_OFFSET_BIT	4   /* descriptor offset address */
#define DMAC_DDA_OFFSET_MASK	(0x0ff << DMAC_DDA_OFFSET_BIT)
/* [3:0] reserved */

// DMA stride address register
#define DMAC_DSD_TSD_BIT        16  /* target stride address */
#define DMAC_DSD_TSD_MASK      	(0xffff << DMAC_DSD_TSD_BIT)
#define DMAC_DSD_SSD_BIT        0  /* source stride address */
#define DMAC_DSD_SSD_MASK      	(0xffff << DMAC_DSD_SSD_BIT)

// DMA control register
#define DMAC_DMACR_FMSC		(1 << 31)  /* MSC Fast DMA mode */
#define DMAC_DMACR_FSSI		(1 << 30)  /* SSI Fast DMA mode */
#define DMAC_DMACR_FTSSI	(1 << 29)  /* TSSI Fast DMA mode */
#define DMAC_DMACR_FUART	(1 << 28)  /* UART Fast DMA mode */
#define DMAC_DMACR_FAIC		(1 << 27)  /* AIC Fast DMA mode */
/* [26:22] reserved */
#define DMAC_DMACR_INTCC_BIT	17         /* Channel Interrupt Counter */
#define DMAC_DMACR_INTCC_MASK	(0x1f << DMAC_DMACR_INTCC_MASK)
#define DMAC_DMACR_INTCE	(1 << 16)  /*Channel Intertupt Enable */
/* [15:4] reserved */
#define DMAC_DMACR_PR_BIT	8  /* channel priority mode */
#define DMAC_DMACR_PR_MASK	(0x03 << DMAC_DMACR_PR_BIT)
#define DMAC_DMACR_PR_012345	(0 << DMAC_DMACR_PR_BIT)
#define DMAC_DMACR_PR_120345	(1 << DMAC_DMACR_PR_BIT)
#define DMAC_DMACR_PR_230145	(2 << DMAC_DMACR_PR_BIT)
#define DMAC_DMACR_PR_340125	(3 << DMAC_DMACR_PR_BIT)
/* [15:4] resered */
#define DMAC_DMACR_HLT		(1 << 3)  /* DMA halt flag */
#define DMAC_DMACR_AR		(1 << 2)  /* address error flag */
#define DMAC_DMACR_CH01		(1 << 1)  /* Special Channel 0 and Channel 1 Enable */
#define DMAC_DMACR_DMAE		(1 << 0)  /* DMA enable bit */

// DMA doorbell register
#define DMAC_DDB_DB(n)		(1 << (n)) /* Doorbell for Channel n */
#define DMAC_DMADBR_DB5		(1 << 5)  /* doorbell for channel 5 */
#define DMAC_DMADBR_DB4		(1 << 4)  /* doorbell for channel 4 */
#define DMAC_DMADBR_DB3		(1 << 3)  /* doorbell for channel 3 */
#define DMAC_DMADBR_DB2		(1 << 2)  /* doorbell for channel 2 */
#define DMAC_DMADBR_DB1		(1 << 1)  /* doorbell for channel 1 */
#define DMAC_DMADBR_DB0		(1 << 0)  /* doorbell for channel 0 */

// DMA doorbell set register
#define DMAC_DDBS_DBS(n)	(1 << (n)) /* Enable Doorbell for Channel n */
#define DMAC_DMADBSR_DBS5	(1 << 5)  /* enable doorbell for channel 5 */
#define DMAC_DMADBSR_DBS4	(1 << 4)  /* enable doorbell for channel 4 */
#define DMAC_DMADBSR_DBS3	(1 << 3)  /* enable doorbell for channel 3 */
#define DMAC_DMADBSR_DBS2	(1 << 2)  /* enable doorbell for channel 2 */
#define DMAC_DMADBSR_DBS1	(1 << 1)  /* enable doorbell for channel 1 */
#define DMAC_DMADBSR_DBS0	(1 << 0)  /* enable doorbell for channel 0 */

// DMA interrupt pending register
#define DMAC_DMAIPR_CIRQ(n)	(1 << (n)) /* irq pending status for channel 5 */
#define DMAC_DMAIPR_CIRQ5	(1 << 5)  /* irq pending status for channel 5 */
#define DMAC_DMAIPR_CIRQ4	(1 << 4)  /* irq pending status for channel 4 */
#define DMAC_DMAIPR_CIRQ3	(1 << 3)  /* irq pending status for channel 3 */
#define DMAC_DMAIPR_CIRQ2	(1 << 2)  /* irq pending status for channel 2 */
#define DMAC_DMAIPR_CIRQ1	(1 << 1)  /* irq pending status for channel 1 */
#define DMAC_DMAIPR_CIRQ0	(1 << 0)  /* irq pending status for channel 0 */

// DMA Channel Programmable
#define DMAC_DMACP_DCP(n)	(1 << (n)) /* Channel programmable enable */

// MCU Control/Status
#define DMAC_DMCS_SLEEP		(1 << 31) /* Sleep Status of MCU */
#define DMAC_DMCS_SCMD		(1 << 30) /* Security Mode */
/* [29:24] reserved */
#define DMAC_DMCS_SCOFF_BIT	8
#define DMAC_DMCS_SCOFF_MASK	(0xffff << DMAC_DMCS_SCOFF_BIT)
#define DMAC_DMCS_BCH_DB	(1 << 7)  /* Block Syndrome of BCH Decoding */
#define DMAC_DMCS_BCH_DF	(1 << 6)  /* BCH Decoding finished */ 
#define DMAC_DMCS_BCH_EF	(1 << 5)  /* BCH Encoding finished */
#define DMAC_DMCS_BTB_INV	(1 << 4)  /* Invalidates BTB in MCU */
#define DMAC_DMCS_SC_CALL	(1 << 3)  /* SecurityCall */
#define DMAC_DMCS_SW_RST	(1 << 0)  /* Software reset */

// MCU Interrupt
/* [31:18] reserved */
#define DMAC_DMINT_S_IP		(1 << 17) /* Security Mailbox IRQ pending */
#define DMAC_DMINT_N_IP		(1 << 16) /* Normal Mailbox IRQ pending */
/* [15:2] reserved */
#define DMAC_DMINT_S_IMSK	(1 << 1)  /* Security Mailbox IRQ mask */
#define DMAC_DMINT_N_IMSK	(1 << 0)  /* Normal Mailbox IRQ mask */


#endif /* __JZ4780_PDMA_H__ */
