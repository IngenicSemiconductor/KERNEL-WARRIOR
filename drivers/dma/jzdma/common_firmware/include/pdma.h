/*
 * pdma.h
 */
#ifndef __PDMA_H__
#define __PDMA_H__

#define PDMA_BCH_CHANNEL        0
#define PDMA_NEMC_CHANNEL       1
#define PDMA_DDR_CHANNEL        2
#define PDMA_MSG_CHANNEL	3

#define NEMC_TO_TCSM	(1 << 0)
#define TCSM_TO_NEMC	(1 << 1)
#define BCH_TO_TCSM	(1 << 2)
#define TCSM_TO_BCH	(1 << 3)
#define DDR_TO_TCSM	(1 << 4)
#define TCSM_TO_DDR	(1 << 5)

/* trans Sys32 VA to Physics Address */
#define CPHYSADDR(a)	((a) & 0x1fffffff)
/* trans Sys32 TCSM VA to Physics Address */
#define TCSMVAOFF	0xE0BDE000
#define TPHYSADDR(a)	((a) - TCSMVAOFF)

#define __pdma_write_cp0(value,cn,slt)	\
	__asm__ __volatile__(		\
		"mtc0\t%0, $%1, %2\n"	\
		: /* no output */	\
		:"r"(value),"i"(cn),"i"(slt))

#define __pdma_read_cp0(cn,slt)		\
({					\
	unsigned int _res_;		\
					\
	__asm__ __volatile__(		\
		"mfc0\t%0, $%1, %2\n"	\
		:"=r"(_res_)		\
		:"i"(cn), "i"(slt));	\
					\
	_res_;				\
})
#define __pdma_mwait() 	__asm__ __volatile__("wait")

#define __pdma_irq_enable()	__pdma_write_cp0(0x1, 12, 0)
#define __pdma_irq_disable()	__pdma_write_cp0(0x0, 12, 0)

extern void nemc_channel_cfg(unsigned char *source_addr, unsigned char *dest_addr,
			unsigned int trans_count, int mode);
extern void bch_channel_cfg(unsigned char *source_addr, unsigned char *dest_addr,
			unsigned int trans_count, int mode);
extern void ddr_channel_cfg(unsigned char *source_addr, unsigned char *dest_addr,
			unsigned int trans_count, int mode);
extern void pdma_channel_init(void);

#endif /* __PDMA_H__ */
