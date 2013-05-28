/*
 * linux/include/asm-mips/mach-jz4780/jz4780pdma.h
 *
 * JZ4780 PDMAC register definition.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */
#ifndef __JZ4780PDMAC_H__
#define __JZ4780PDMAC_H__

#define PDMAC_BASE	0x13420000

#define MAX_DMA_NUM	32  /* max 32 channels */

/*************************************************************************
 * PDMAC (DMA Controller)
 *************************************************************************/
/* DMA Channel Register (0 ~ 31) */
#define PDMAC_DSA(n)	(PDMAC_BASE + (n) * 0x20 + 0x00) /* DMA Source Address */
#define PDMAC_DTA(n)	(PDMAC_BASE + (n) * 0x20 + 0x04) /* DMA Target Address */
#define PDMAC_DTC(n)	(PDMAC_BASE + (n) * 0x20 + 0x08) /* DMA Transfer Count */
#define PDMAC_DRT(n)	(PDMAC_BASE + (n) * 0x20 + 0x0C) /* DMA Request Type */
#define PDMAC_DCCS(n)	(PDMAC_BASE + (n) * 0x20 + 0x10) /* DMA Channel Control/Status */
#define PDMAC_DCMD(n)	(PDMAC_BASE + (n) * 0x20 + 0x14) /* DMA Command */
#define PDMAC_DDA(n)	(PDMAC_BASE + (n) * 0x20 + 0x18) /* DMA Descriptor Addr */
#define PDMAC_DSD(n)	(PDMAC_BASE + (n) * 0x20 + 0x1C) /* DMA Stride Address */

#define PDMAC_DMAC	(PDMAC_BASE + 0x1000) /* DMA Control Register */
#define PDMAC_DIRQP	(PDMAC_BASE + 0x1004) /* DMA Interrupt Pending */
#define PDMAC_DDB	(PDMAC_BASE + 0x1008) /* DMA Doorbell */
#define PDMAC_DDBS	(PDMAC_BASE + 0x100C) /* DMA Doorbell Set */
#define PDMAC_DCKE	(PDMAC_BASE + 0x1010) /* DMA Clock Enable */
#define PDMAC_DCKES	(PDMAC_BASE + 0x1014) /* DMA Clock Enable Set */
#define PDMAC_DCKEC	(PDMAC_BASE + 0x1018) /* DAM Clock Enable Clear */
#define PDMAC_DMACP	(PDMAC_BASE + 0x101C) /* DMA Channel Programmable */
#define PDMAC_DSIRQP	(PDMAC_BASE + 0x1020) /* DMA Soft IRQ Pending */
#define PDMAC_DSIRQM	(PDMAC_BASE + 0x1024) /* DMA Soft IRQ Mask */
#define PDMAC_DCIRQP	(PDMAC_BASE + 0x1028) /* DMA Channel IRQ Pending to MCU */
#define PDMAC_DCIRQM	(PDMAC_BASE + 0x102C) /* DMA Channel IRQ Mask to MCU */

/* PDMA MCU Control Register */
#define PDMAC_DMCS	(PDMAC_BASE + 0x1030) /* DMA MCU Control/Status */
#define PDMAC_DMNMB	(PDMAC_BASE + 0x1034) /* DMA MCU Normal Mailbox */
#define PDMAC_DMSMB	(PDMAC_BASE + 0x1038) /* DMA MCU Security Mailbox */
#define PDMAC_DMINT	(PDMAC_BASE + 0x103C) /* DMA MCU Interrupt */

#define REG_PDMAC_DSA(n)	REG32(PDMAC_DSA(n))
#define REG_PDMAC_DTA(n)	REG32(PDMAC_DTA(n))
#define REG_PDMAC_DTC(n)	REG32(PDMAC_DTC(n))
#define REG_PDMAC_DRT(n)	REG32(PDMAC_DRT(n))
#define REG_PDMAC_DCCS(n)	REG32(PDMAC_DCCS(n))
#define REG_PDMAC_DCMD(n)	REG32(PDMAC_DCMD(n))
#define REG_PDMAC_DDA(n)	REG32(PDMAC_DDA(n))
#define REG_PDMAC_DSD(n)	REG32(PDMAC_DSD(n))
#define REG_PDMAC_DMAC		REG32(PDMAC_DMAC)
#define REG_PDMAC_DIRQP		REG32(PDMAC_DIRQP)
#define REG_PDMAC_DDB		REG32(PDMAC_DDB)
#define REG_PDMAC_DDBS		REG32(PDMAC_DDBS)
#define REG_PDMAC_DCKE		REG32(PDMAC_DCKE)
#define REG_PDMAC_DCKES		REG32(PDMAC_DCKES)
#define REG_PDMAC_DCKEC		REG32(PDMAC_DCKEC)
#define REG_PDMAC_DMACP		REG32(PDMAC_DMACP)
#define REG_PDMAC_DSIRQP	REG32(PDMAC_DSIRQP)
#define REG_PDMAC_DSIRQM	REG32(PDMAC_DSIRQM)
#define REG_PDMAC_DCIRQP	REG32(PDMAC_DCIRQP)
#define REG_PDMAC_DCIRQM	REG32(PDMAC_DCIRQM)
#define REG_PDMAC_DMCS		REG32(PDMAC_DMCS)
#define REG_PDMAC_DMNMB		REG32(PDMAC_DMNMB)
#define REG_PDMAC_DMSMB		REG32(PDMAC_DMSMB)
#define REG_PDMAC_DMINT		REG32(PDMAC_DMINT)

/* DMA Transfer Count */
#define PDMAC_DTC_TC_BIT	0
#define PDMAC_DTC_TC_MAKS	(0x7fffff << PDMAC_DTC_TC_BIT)

/* DMA Request Source Register */
#define PDMAC_DRT_RT_BIT	0
#define PDMAC_DRT_RT_MASK	(0x3f << PDMAC_DRT_RT_BIT)
/* 0~1 is reserved */
#define PDMAC_DRT_RT_I2S1OUT	(0x04 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRT_RT_I2S1IN	(0x05 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRT_RT_I2S0OUT	(0x06 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRT_RT_I2S0IN	(0x07 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRT_RT_AUTO	(0x08 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRT_RT_SADCIN	(0x09 << PDMAC_DRT_RT_BIT)
#if 0
/* 10 ~ 11 is reserved */
#define PDMAC_DRSR_RS_EXTERN	(12 << PDMAC_DRT_RT_BIT)
/* 13 is reserved */
#define PDMAC_DRSR_RS_UART3OUT	(14 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_UART3IN	(15 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_UART2OUT	(16 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_UART2IN	(17 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_UART1OUT	(18 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_UART1IN	(19 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_UART0OUT	(20 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_UART0IN	(21 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_SSI0OUT	(22 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_SSI0IN	(23 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_AICOUT	(24 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_AICIN	(25 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_MSC0OUT	(26 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_MSC0IN	(27 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_TCU	(28 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_SADC	(29 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_MSC1OUT	(30 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_MSC1IN	(31 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_SSI1OUT	(32 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_SSI1IN	(33 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_PM0OUT	(34 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_PM0IN	(35 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_MSC2OUT	(36 << PDMAC_DRT_RT_BIT)
#define PDMAC_DRSR_RS_MSC2IN	(37 << PDMAC_DRT_RT_BIT)
/* others are reserved */
#endif

/* DMA Channel Control/Status Register */
#define PDMAC_DCCS_NDES		(1 << 31) /* Descriptor or No-Descriptor Transfer Select (0 : 1) */
#define PDMAC_DCCS_DES8		(1 << 30) /* Descriptor 8 Word */
#define PDMAC_DCCS_DES4		(0 << 30) /* Descriptor 4 Word */
#define PDMAC_DCCS_TOC_BIT	16
#define PDMAC_DCCS_TOC_MASK	(0x3fff << PDMAC_DCCS_TOC_MASK)
#define PDMAC_DCCS_CDOA_BIT	8 /* Copy of DMA Offset Address */
#define PDMAC_DCCS_CDOA_MASK	(0xff << PDMAC_DCCS_CDOA_BIT)
/* [7:5] reserved */
#define PDMAC_DCCS_AR		(1 << 4)  /* Address Error */
#define PDMAC_DCCS_TT		(1 << 3)  /* Transfer Terminated */
#define PDMAC_DCCS_HLT		(1 << 2)  /* DMA Halted */
#define PDMAC_DCCS_TOE		(1 << 1)  /* Timeout Enable */
#define PDMAC_DCCS_CTE		(1 << 0)  /* Channel Transfer Enable */

/* DMA Channel Command Register */
/* [31:28] reserved */
#define PDMAC_DCMD_SID_BIT	26 /* Source Identification */
#define PDMAC_DCMD_SID_MASK	(0x3 << PDMAC_DCMD_SID_BIT)
#define PDMAC_DCMD_SID(n)	((n) << PDMAC_DCMD_SID_BIT)
#define PDMAC_DCMD_SID_TCSM	(0 << PDMAC_DCMD_SID_BIT)
#define PDMAC_DCMD_SID_SPECIAL	(1 << PDMAC_DCMD_SID_BIT)
#define PDMAC_DCMD_SID_DDR	(2 << PDMAC_DCMD_SID_BIT)
#define PDMAC_DCMD_DID_BIT	24 /* Destination Identification */
#define PDMAC_DCMD_DID_MASK	(0x3 << PDMAC_DCMD_DID_BIT)
#define PDMAC_DCMD_DID(n)	((n) << PDMAC_DCMD_DID_BIT)
#define PDMAC_DCMD_DID_TCSM	(0 << PDMAC_DCMD_DID_BIT)
#define PDMAC_DCMD_DID_SPECIAL	(1 << PDMAC_DCMD_DID_BIT)
#define PDMAC_DCMD_DID_DDR	(2 << PDMAC_DCMD_DID_BIT)
#define PDMAC_DCMD_SAI		(1 << 23) /* Source Address Increment */
#define PDMAC_DCMD_DAI		(1 << 22) /* Dest Address Increment */
/* [21:20] reserved */
#define PDMAC_DCMD_RDIL_BIT	16 /* Request Detection Interval Length */
#define PDMAC_DCMD_RDIL_MASK	(0xf << PDMAC_DCMD_RDIL_BIT)
#define PDMAC_DCMD_RDIL(n)	((n) << PDMAC_DCMD_RDIL_BIT)
#define PDMAC_DCMD_RDIL_1BYTE	(1 << PDMAC_DCMD_RDIL_BIT)
#define PDMAC_DCMD_RDIL_2BYTE	(2 << PDMAC_DCMD_RDIL_BIT)
#define PDMAC_DCMD_RDIL_3BYTE	(3 << PDMAC_DCMD_RDIL_BIT)
#define PDMAC_DCMD_RDIL_4BYTE	(4 << PDMAC_DCMD_RDIL_BIT)
#define PDMAC_DCMD_RDIL_8BYTE	(5 << PDMAC_DCMD_RDIL_BIT)
#define PDMAC_DCMD_RDIL_16BYTE	(6 << PDMAC_DCMD_RDIL_BIT)
#define PDMAC_DCMD_RDIL_32BYTE	(7 << PDMAC_DCMD_RDIL_BIT)
#define PDMAC_DCMD_RDIL_64BYTE	(8 << PDMAC_DCMD_RDIL_BIT)
#define PDMAC_DCMD_RDIL_128BYTE	(9 << PDMAC_DCMD_RDIL_BIT)
#define PDMAC_DCMD_SWDH_BIT	14 /* Source Port Width */
#define PDMAC_DCMD_SWDH_MASK	(0x3 << PDMAC_DCMD_SWDH_BIT)
#define PDMAC_DCMD_SWDH_32	(0 << PDMAC_DCMD_SWDH_BIT)
#define PDMAC_DCMD_SWDH_8	(1 << PDMAC_DCMD_SWDH_BIT)
#define PDMAC_DCMD_SWDH_16	(2 << PDMAC_DCMD_SWDH_BIT)
#define PDMAC_DCMD_DWDH_BIT	12 /* Dest Port Width */
#define PDMAC_DCMD_DWDH_MASK	(0x03 << PDMAC_DCMD_DWDH_BIT)
#define PDMAC_DCMD_DWDH_32	(0 << PDMAC_DCMD_DWDH_BIT)
#define PDMAC_DCMD_DWDH_8	(1 << PDMAC_DCMD_DWDH_BIT)
#define PDMAC_DCMD_DWDH_16	(2 << PDMAC_DCMD_DWDH_BIT)
/* [11] reserved */
#define PDMAC_DCMD_TSZ_BIT	8 /* Transfer Data Size of a data unit */
#define PDMAC_DCMD_TSZ_MASK	(0x7 << PDMAC_DCMD_TSZ_BIT)
#define PDMAC_DCMD_TSZ_32BIT	(0 << PDMAC_DCMD_TSZ_BIT)
#define PDMAC_DCMD_TSZ_8BIT	(1 << PDMAC_DCMD_TSZ_BIT)
#define PDMAC_DCMD_TSZ_16BIT	(2 << PDMAC_DCMD_TSZ_BIT)
#define PDMAC_DCMD_TSZ_16BYTE	(3 << PDMAC_DCMD_TSZ_BIT)
#define PDMAC_DCMD_TSZ_32BYTE	(4 << PDMAC_DCMD_TSZ_BIT)
#define PDMAC_DCMD_TSZ_64BYTE	(5 << PDMAC_DCMD_TSZ_BIT)
#define PDMAC_DCMD_TSZ_128BYTE	(6 << PDMAC_DCMD_TSZ_BIT)
#define PDMAC_DCMD_TSZ_AUTO	(7 << PDMAC_DCMD_TSZ_BIT)
/* [7:3] reserved */
#define PDMAC_DCMD_STDE   	(1 << 2) /* Stride Disable/Enable */
#define PDMAC_DCMD_TIE		(1 << 1) /* DMA Transfer Interrupt Enable */
#define PDMAC_DCMD_LINK		(1 << 0) /* Descriptor Link Enable */

/* DMA Descriptor Address Register */
#define PDMAC_DDA_BASE_BIT	12 /* Descriptor Base Address */
#define PDMAC_DDA_BASE_MASK	(0x0fffff << PDMAC_DDA_BASE_BIT)
#define PDMAC_DDA_OFFSET_BIT	4  /* Descriptor Offset Address */
#define PDMAC_DDA_OFFSET_MASK	(0x0ff << PDMAC_DDA_OFFSET_BIT)
/* [3:0] reserved */

/* DMA Stride Address Register */
#define PDMAC_DSD_TSD_BIT	16 /* Target Stride Address */
#define PDMAC_DSD_TSD_MASK	(0xffff << PDMAC_DSD_TSD_BIT)
#define PDMAC_DSD_SSD_BIT        0 /* Source Stride Address */
#define PDMAC_DSD_SSD_MASK	(0xffff << PDMAC_DSD_SSD_BIT)

/* DMA Control Register */
#define PDMAC_DMAC_FMSC		(1 << 31) /* MSC Fast DMA mode */
#define PDMAC_DMAC_FSSI		(1 << 30) /* SSI Fast DMA mode */
#define PDMAC_DMAC_FTSSI	(1 << 29) /* TSSI Fast DMA mode */
#define PDMAC_DMAC_FUART	(1 << 28) /* UART Fast DMA mode */
#define PDMAC_DMAC_FAIC		(1 << 27) /* AIC Fast DMA mode */
/* [26:22] reserved */
#define PDMAC_DMAC_INTCC_BIT	17 /* Channel Interrupt Counter */
#define PDMAC_DMAC_INTCC_MASK	(0x1f << PDMAC_DMAC_INTCC_MASK)
#define PDMAC_DMAC_INTCE	(1 << 16) /*Channel Intertupt Enable */
/* [15:4] reserved */
#define PDMAC_DMAC_HLT		(1 << 3) /* DMA Halt Flag */
#define PDMAC_DMAC_AR		(1 << 2) /* Address Error Flag */
#define PDMAC_DMAC_CH01		(1 << 1) /* Special Channel 0 and Channel 1 Enable */
#define PDMAC_DMAC_DMAE		(1 << 0) /* Globale DMA Enable */

/* DMA Doorbell Register */
#define PDMAC_DDB_DB(n)		(1 << (n)) /* Doorbell for Channel n */

/* DMA Doorbell Set Register */
#define PDMAC_DDBS_DBS(n)	(1 << (n)) /* Enable Doorbell for Channel n */

/* DMA Interrupt Pending Register */
#define PDMAC_DIRQP_CIRQ(n)	(1 << (n)) /* IRQ Pending Status for Channel n */

/* DMA Channel Programmable */
#define PDMAC_DMACP_DCP(n)	(1 << (n)) /* Channel programmable enable */

/* MCU Control/Status */
#define PDMAC_DMCS_SLEEP		(1 << 31) /* Sleep Status of MCU */
#define PDMAC_DMCS_SCMD		(1 << 30) /* Security Mode */
/* [29:24] reserved */
#define PDMAC_DMCS_SCOFF_BIT	8
#define PDMAC_DMCS_SCOFF_MASK	(0xffff << PDMAC_DMCS_SCOFF_BIT)
#define PDMAC_DMCS_BCH_DB	(1 << 7) /* Block Syndrome of BCH Decoding */
#define PDMAC_DMCS_BCH_DF	(1 << 6) /* BCH Decoding finished */ 
#define PDMAC_DMCS_BCH_EF	(1 << 5) /* BCH Encoding finished */
#define PDMAC_DMCS_BTB_INV	(1 << 4) /* Invalidates BTB in MCU */
#define PDMAC_DMCS_SC_CALL	(1 << 3) /* SecurityCall */
#define PDMAC_DMCS_SW_RST	(1 << 0) /* Software reset */

/* MCU Interrupt */
/* [31:18] reserved */
#define PDMAC_DMINT_S_IP		(1 << 17) /* Security Mailbox IRQ pending */
#define PDMAC_DMINT_N_IP		(1 << 16) /* Normal Mailbox IRQ pending */
/* [15:2] reserved */
#define PDMAC_DMINT_S_IMSK	(1 << 1) /* Security Mailbox IRQ mask */
#define PDMAC_DMINT_N_IMSK	(1 << 0) /* Normal Mailbox IRQ mask */

#ifndef __MIPS_ASSEMBLER
/***************************************************************************
 * PDMAC
 ***************************************************************************/

#define __pdmac_enable()	(REG_PDMAC_DMAC |= PDMAC_DMAC_DMAE)
#define __pdmac_disable()	(REG_PDMAC_DMAC &= ~PDMAC_DMAC_DMAE)

/* m is the DMA controller index (0, 1), n is the DMA channel index (0 - 11) */
#if 0
#define __dmac_enable_module()	(REG_PDMAC_PDMACR() |= PDMAC_PDMACR_DMAE | PDMAC_PDMACR_PR_012345)
#define __dmac_disable_module(m)			\
	( REG_PDMAC_PDMACR(m) &= ~PDMAC_PDMACR_DMAE )

/* p=0,1,2,3 */
#define __dmac_set_priority(m,p)					\
	do {								\
		REG_PDMAC_PDMACR(m) &= ~PDMAC_PDMACR_PR_MASK;		\
		REG_PDMAC_PDMACR(m) |= ((p) << PDMAC_PDMACR_PR_BIT);	\
	} while (0)
#endif

#define __pdmac_check_halt_error() (REG_PDMAC_DMAC & PDMAC_DMAC_HLT)
#define __pdmac_check_addr_error() (REG_PDMAC_DMAC & PDMAC_DMAC_AR)

#if 0
#define __dmac_channel_enable_clk(n)					\
	REG_PDMAC_PDMACKE((n)/HALF_DMA_NUM) |= 1 << ((n)-(n)/HALF_DMA_NUM*HALF_DMA_NUM);

#define __dmac_enable_descriptor(n)			\
	( REG_PDMAC_DCCSR((n)) &= ~PDMAC_DCCSR_NDES )
#define __dmac_disable_descriptor(n)			\
	( REG_PDMAC_DCCSR((n)) |= PDMAC_DCCSR_NDES )
#endif

#define __pdmac_channel_enable(n)		(REG_PDMAC_DCCS(n) |= PDMAC_DCCS_CTE)
#define __pdmac_channel_disable(n)		(REG_PDMAC_DCCS(n) &= ~PDMAC_DCCS_CTE)

#define __pdmac_channel_enabled(n)		(REG_PDMAC_DCCS(n) & PDMAC_DCCS_CTE)

#define __pdmac_channel_irq_unmask()		(REG_PDMAC_DCIRQM = 0)
#define __pdmac_channel_irq_enable(n)		(REG_PDMAC_DCMD(n) |= PDMAC_DCMD_TIE)
#define __pdmac_channel_irq_disable(n)		(REG_PDMAC_DCMD(n) &= ~PDMAC_DCMD_TIE)

#if 0
#define __dmac_channel_enable_irq(n)		\
	( REG_PDMAC_DCMD((n)) |= PDMAC_DCMD_TIE )
#define __dmac_channel_disable_irq(n)			\
	( REG_PDMAC_DCMD((n)) &= ~PDMAC_DCMD_TIE )
#endif

#define __pdmac_channel_halt_detected(n)	(REG_PDMAC_DCCS(n) & PDMAC_DCCS_HLT)
#define __pdmac_channel_end_detected(n)		(REG_PDMAC_DCCS(n) & PDMAC_DCCS_TT)
#define __pdmac_channel_address_error_detected(n)	\
		(REG_PDMAC_DCCS(n) & PDMAC_DCCS_AR)

#if 0
#define __dmac_channel_clear_transmit_halt(n)				\
	do {								\
		/* clear both channel halt error and globle halt error */ \
		REG_PDMAC_DCCSR(n) &= ~PDMAC_DCCSR_HLT;			\
		REG_PDMAC_PDMACR(n/HALF_DMA_NUM) &= ~PDMAC_PDMACR_HLT;	\
	} while (0)
#define __dmac_channel_clear_transmit_end(n)		\
	(  REG_PDMAC_DCCSR(n) &= ~PDMAC_DCCSR_TT )
#define __dmac_channel_clear_address_error(n)				\
	do {								\
		REG_PDMAC_DDA(n) = 0; /* clear descriptor address register */ \
		REG_PDMAC_DSAR(n) = 0; /* clear source address register */ \
		REG_PDMAC_DTAR(n) = 0; /* clear target address register */ \
		/* clear both channel addr error and globle address error */ \
		REG_PDMAC_DCCSR(n) &= ~PDMAC_DCCSR_AR;			\
		REG_PDMAC_PDMACR(n/HALF_DMA_NUM) &= ~PDMAC_PDMACR_AR;	\
	} while (0)
#define __dmac_channel_clear_count_terminated(n)	\
	(  REG_PDMAC_DCCSR((n)) &= ~PDMAC_DCCSR_CT )
#define __dmac_channel_clear_descriptor_invalid(n)	\
	(  REG_PDMAC_DCCSR((n)) &= ~PDMAC_DCCSR_INV )

#define __dmac_channel_set_transfer_unit_32bit(n)		\
	do {							\
		REG_PDMAC_DCMD((n)) &= ~PDMAC_DCMD_DS_MASK;	\
		REG_PDMAC_DCMD((n)) |= PDMAC_DCMD_DS_32BIT;	\
	} while (0)

#define __dmac_channel_set_transfer_unit_16bit(n)		\
	do {							\
		REG_PDMAC_DCMD((n)) &= ~PDMAC_DCMD_DS_MASK;	\
		REG_PDMAC_DCMD((n)) |= PDMAC_DCMD_DS_16BIT;	\
	} while (0)

#define __dmac_channel_set_transfer_unit_8bit(n)		\
	do {							\
		REG_PDMAC_DCMD((n)) &= ~PDMAC_DCMD_DS_MASK;	\
		REG_PDMAC_DCMD((n)) |= PDMAC_DCMD_DS_8BIT;	\
	} while (0)

#define __dmac_channel_set_transfer_unit_16byte(n)		\
	do {							\
		REG_PDMAC_DCMD((n)) &= ~PDMAC_DCMD_DS_MASK;	\
		REG_PDMAC_DCMD((n)) |= PDMAC_DCMD_DS_16BYTE;	\
	} while (0)

#define __dmac_channel_set_transfer_unit_32byte(n)		\
	do {							\
		REG_PDMAC_DCMD((n)) &= ~PDMAC_DCMD_DS_MASK;	\
		REG_PDMAC_DCMD((n)) |= PDMAC_DCMD_DS_32BYTE;	\
	} while (0)

/* w=8,16,32 */
#define __dmac_channel_set_dest_port_width(n,w)			\
	do {							\
		REG_PDMAC_DCMD((n)) &= ~PDMAC_DCMD_DWDH_MASK;	\
		REG_PDMAC_DCMD((n)) |= PDMAC_DCMD_DWDH_##w;	\
	} while (0)

/* w=8,16,32 */
#define __dmac_channel_set_src_port_width(n,w)			\
	do {							\
		REG_PDMAC_DCMD((n)) &= ~PDMAC_DCMD_SWDH_MASK;	\
		REG_PDMAC_DCMD((n)) |= PDMAC_DCMD_SWDH_##w;	\
	} while (0)

/* v=0-15 */
#define __dmac_channel_set_rdil(n,v)				\
	do {							\
	REG_PDMAC_DCMD((n)) &= ~PDMAC_DCMD_RDIL_MASK;		\
	REG_PDMAC_DCMD((n) |= ((v) << PDMAC_DCMD_RDIL_BIT));	\
} while (0)

#define __dmac_channel_dest_addr_fixed(n)		\
	(  REG_PDMAC_DCMD((n)) &= ~PDMAC_DCMD_DAI )
#define __dmac_channel_dest_addr_increment(n)		\
	(  REG_PDMAC_DCMD((n)) |= PDMAC_DCMD_DAI )

#define __dmac_channel_src_addr_fixed(n)		\
	(  REG_PDMAC_DCMD((n)) &= ~PDMAC_DCMD_SAI )
#define __dmac_channel_src_addr_increment(n)		\
	(  REG_PDMAC_DCMD((n)) |= PDMAC_DCMD_SAI )

#define __dmac_channel_set_doorbell(n)					\
	(  REG_PDMAC_DMADBSR((n)/HALF_DMA_NUM) = (1 << ((n)-(n)/HALF_DMA_NUM*HALF_DMA_NUM)) )

#define __dmac_channel_irq_detected(n)  ( REG_PDMAC_DMAIPR((n)/HALF_DMA_NUM) & (1 << ((n)-(n)/HALF_DMA_NUM*HALF_DMA_NUM)) )
#define __dmac_channel_ack_irq(n)       ( REG_PDMAC_DMAIPR((n)/HALF_DMA_NUM) &= ~(1 <<((n)-(n)/HALF_DMA_NUM*HALF_DMA_NUM)) )
#endif

#define __pdmac_channel_priv_irq_set(n)		(REG_PDMAC_DCIRQM = ~(1 << (n)))
#define __pdmac_channel_priv_irq_add(n)		(REG_PDMAC_DCIRQM &= ~(1 << (n)))
#define __pdmac_channel_priv_irq_clear()	(REG_PDMAC_DCIRQM = 0)

#define __pdmac_channel_mirq_check(n)		(REG_PDMAC_DCIRQP & (1 << (n)))
#define __pdmac_channel_mirq_clear(n) \
	(REG_PDMAC_DCCS(n) &= ~(PDMAC_DCCS_AR | PDMAC_DCCS_TT | PDMAC_DCCS_HLT | PDMAC_DCCS_CTE))

#define __pdmac_special_channel_enable()	(REG_PDMAC_DMAC |= PDMAC_DMAC_CH01)
#define __pdmac_special_channel_disable()	(REG_PDMAC_DMAC &= ~PDMAC_DMAC_CH01)

#define __pdmac_channel_programmable_set(n)	(REG_PDMAC_DMACP |= 1 << (n))
#define __pdmac_channel_programmable_clear()	(REG_PDMAC_DMACP = 0)

#define __pdmac_channel_launch(n)		(REG_PDMAC_DCCS(n) |= PDMAC_DCCS_CTE)
#define __pdmac_special_channel_launch(n)	(REG_PDMAC_DCCS(n) |= PDMAC_DCCS_NDES | PDMAC_DCCS_CTE)
#define __pdmac_channel_halt(n)			(REG_PDMAC_DCCS(n) &= ~PDMAC_DCCS_CTE)

/* MCU BCH Control */
#define __mbch_encode_sync()		while (REG_PDMAC_DMCS & PDMAC_DMCS_BCH_EF)
#define __mbch_decode_sync()		while (REG_PDMAC_DMCS & PDMAC_DMCS_BCH_DF)
#define __mbch_decode_sdmf()		while (REG_PDMAC_DMCS & PDMAC_DMCS_BCH_DB)

/* MCU MailBox */
#define __pdmac_mmb_mask()		(REG_PDMAC_DMINT |= PDMAC_DMINT_N_IMSK | PDMAC_DMINT_S_IMSK)
#define __pdmac_mmb_unmask()		(REG_PDMAC_DMINT &= ~(PDMAC_DMINT_N_IMSK | PDMAC_DMINT_S_IMSK))
#define __pdmac_mmb_irq_clear()		(REG_PDMAC_DMINT = ~(PDMAC_DMINT_N_IP | PDMAC_DMINT_S_IP))

#define __pdmac_mnmb_mask()		(REG_PDMAC_DMINT |= PDMAC_DMINT_N_IMSK)
#define __pdmac_mnmb_unmask()		(REG_PDMAC_DMINT &= PDMAC_DMINT_N_IMSK)
#define __pdmac_mnmb_send(n) 		(REG_PDMAC_DMNMB = (n))
#define __pdmac_mnmb_get(n)		((n) = REG_PDMAC_DMNMB)
#define __pdmac_mnmb_clear()		(REG_PDMAC_DMNMB = 0)

#define __pdmac_msmb_send(n)		(REG_PDMAC_DMSMB = (n))
#define __pdmac_msmb_clear()		(REG_PDMAC_DMSMB = 0)

#if 0
static __inline__ int __dmac_get_irq(void)
{
	int i;
	for (i = 0; i < MAX_DMA_NUM; i++)
		if (__dmac_channel_irq_detected(i))
			return i;
	return -1;
}
#endif

#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4780PDMAC_H__ */
