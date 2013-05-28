/*
 * linux/include/asm-mips/mach-jz4780/jz4780intc.h
 *
 * JZ4780 INTC register definition.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ4780INTC_H__
#define __JZ4780INTC_H__


#define	INTC_BASE	0xB0001000


/*************************************************************************
 * INTC (Interrupt Controller)
 *************************************************************************/
/* n = 0 ~ 1 */
#define INTC_ISR(n)	(INTC_BASE + 0x00 + (n) * 0x20)
#define INTC_ICMR(n)	(INTC_BASE + 0x04 + (n) * 0x20)
#define INTC_ICMSR(n)	(INTC_BASE + 0x08 + (n) * 0x20)
#define INTC_ICMCR(n)	(INTC_BASE + 0x0c + (n) * 0x20)
#define INTC_IPR(n)	(INTC_BASE + 0x10 + (n) * 0x20)

#define INTC_DSR0       (INTC_BASE + 0x34)
#define INTC_DMR0       (INTC_BASE + 0x38)
#define INTC_DPR0       (INTC_BASE + 0x3c)
#define INTC_DSR1       (INTC_BASE + 0x40)
#define INTC_DMR1       (INTC_BASE + 0x44)
#define INTC_DPR1       (INTC_BASE + 0x48)

#define REG_INTC_ISR(n)		REG32(INTC_ISR(n))
#define REG_INTC_ICMR(n)	REG32(INTC_ICMR(n))
#define REG_INTC_ICMSR(n)	REG32(INTC_ICMSR(n))
#define REG_INTC_ICMCR(n)	REG32(INTC_ICMCR(n))
#define REG_INTC_IPR(n)		REG32(INTC_IPR(n))

#define REG_INTC_DSR0           REG32(INTC_DSR0)
#define REG_INTC_DMR0           REG32(INTC_DMR0)
#define REG_INTC_DPR0           REG32(INTC_DPR0)
#define REG_INTC_DSR1           REG32(INTC_DSR1)
#define REG_INTC_DMR1           REG32(INTC_DMR1)
#define REG_INTC_DPR1           REG32(INTC_DPR1)

// 1st-level interrupts
#define IRQ_AIC1	0
#define IRQ_AIC0	1
#define IRQ_BCH         2
#define IRQ_HDMI	3
#define IRQ_HDMI_WAKEUP	4
#define IRQ_OHCI	5
/*
  Reserved
*/
#define IRQ_SSI1   	7
#define IRQ_SSI0   	8
#define IRQ_TSSI0	9
#define IRQ_PDMA	10
#define IRQ_TSSI1	11
#define IRQ_GPIO5	12
#define IRQ_GPIO4	13
#define IRQ_GPIO3	14
#define IRQ_GPIO2	15
#define IRQ_GPIO1	16
#define IRQ_GPIO0	17
#define IRQ_SADC	18
#define IRQ_X2D		19
#define IRQ_EHCI	20
#define IRQ_OTG		21
/*
  Reserved
*/
#define IRQ_TCU2	25
#define IRQ_TCU1	26
#define IRQ_TCU0	27
#define IRQ_GPS		28
#define IRQ_IPU		29
#define IRQ_CIM		30
#define IRQ_LCD		31

#define IRQ_RTC		32
#define IRQ_OWI		33
#define IRQ_UART4 	34
#define IRQ_MSC2	35
#define IRQ_MSC1	36
#define IRQ_MSC0	37
#define IRQ_SCC		38
/*
  Reserved
*/
#define IRQ_PCM0	40
#define IRQ_KBC 	41
#define IRQ_GPVLC	42
#define IRQ_AOSD        43
#define IRQ_HARB2	44
#define IRQ_HARB1	45
#define IRQ_HARB0	46
#define IRQ_CPM         47
#define IRQ_UART3       48
#define IRQ_UART2       49
#define IRQ_UART1       50
#define IRQ_UART0       51
#define IRQ_DDR         52
#define IRQ_DES         53
#define IRQ_NEMC        54
#define IRQ_ETH         55
#define IRQ_I2C4        56
#define IRQ_I2C3        57
#define IRQ_I2C2        58
#define IRQ_I2C1        59
#define IRQ_I2C0        60
#define IRQ_PDMAM       61
#define IRQ_VPU         62
#define IRQ_GPU         63


// 2nd-level interrupts

#define IRQ_DMA_0	65    /* 64,65,66,67,68,69 */
#define IRQ_GPIO_0	(IRQ_DMA_0 + MAX_DMA_NUM)
#define IRQ_MCU_0	(IRQ_GPIO_0 + MAX_GPIO_NUM)

#define NUM_INTC	64
#define NUM_DMA         MAX_DMA_NUM	/* 12 */
#define NUM_GPIO        MAX_GPIO_NUM	/* GPIO NUM: 192, Jz4780 real num GPIO 178 */


#ifndef __MIPS_ASSEMBLER


/***************************************************************************
 * INTC
 ***************************************************************************/
#define __intc_unmask_irq(n)	(REG_INTC_ICMCR((n)/32) = (1 << ((n)%32)))
#define __intc_mask_irq(n)	(REG_INTC_ICMSR((n)/32) = (1 << ((n)%32)))
#define __intc_ack_irq(n)	(REG_INTC_IPR((n)/32) = (1 << ((n)%32))) /* A dummy ack, as the Pending Register is Read Only. Should we remove __intc_ack_irq() */


#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4780INTC_H__ */

