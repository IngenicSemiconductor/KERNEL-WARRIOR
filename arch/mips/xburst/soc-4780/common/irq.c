/*
 * Interrupt controller.
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Author: <lhhuang@ingenic.cn>
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>

#include <linux/proc_fs.h>

#include <asm/irq_cpu.h>

#include <soc/base.h>
#include <soc/irq.h>
#include <soc/ost.h>

#include <linux/io.h>
#include <smp_cp0.h>
#include <linux/uaccess.h>

#define TRACE_IRQ        1
#define PART_OFF	0x20

#define ISR_OFF		(0x00)
#define IMR_OFF		(0x04)
#define IMSR_OFF	(0x08)
#define IMCR_OFF	(0x0c)
#define IPR_OFF		(0x10)

#define regr(off) 	inl(OST_IOBASE + (off))
#define regw(val,off)	outl(val, OST_IOBASE + (off))

static void __iomem *intc_base;
static unsigned long intc_saved[2];
static unsigned long intc_wakeup[2];
//#define IRQ_TIME_MONITOR_DEBUG
#ifdef IRQ_TIME_MONITOR_DEBUG
unsigned long long time_monitor[500] = {0};
int time_over[500] = {0};
int echo_success = 0;
#endif

#ifdef IRQ_TIME_MONITOR_DEBUG
enum {
	IRQ_ID_AIC = 9,
#define IRQ_NAME_AIC		"aic_irq"
	IRQ_ID_HDMI = 11,
#define IRQ_NAME_HDMI		"HDMI TX_INT"
	IRQ_ID_PDMA = 18,
#define IRQ_NAME_PDMA		"pdma"
	IRQ_ID_GPIO_F = 20,
#define IRQ_NAME_GPIO_F		"GPIO F"
	IRQ_ID_GPIO_E = 21,
#define IRQ_NAME_GPIO_E		"GPIO E"
	IRQ_ID_GPIO_D = 22,
#define IRQ_NAME_GPIO_D		"GPIO D"
	IRQ_ID_GPIO_C = 23,
#define IRQ_NAME_GPIO_C		"GPIO C"
	IRQ_ID_GPIO_B = 24,
#define IRQ_NAME_GPIO_B		"GPIO B"
	IRQ_ID_GPIO_A = 25,
#define IRQ_NAME_GPIO_A		"GPIO A"

	/**********************************************************************************/
	IRQ_ID_X2D = 27,
#define IRQ_NAME_X2D		"x2d"
	IRQ_ID_OTG = 29,
#define IRQ_NAME_OTG		"dwc_otg, dwc_otg_pcd, dwc_otg_hcd:usb1"
	IRQ_ID_IPU0 = 30,
#define IRQ_NAME_IPU0		"ipu0"
	IRQ_ID_LCDC1 = 31,
#define IRQ_NAME_LCDC1		"lcdc1"
	IRQ_ID_CPU0TIMER = 34,
#define IRQ_NAME_CPU0TIMER	"CPU0 jz-timerirq"
	IRQ_ID_CPU1TIMER = 35,
#define IRQ_NAME_CPU1TIMER	"CPU1 jz-timerirq"
	IRQ_ID_IPU1 = 37,
#define IRQ_NAME_IPU1		"ipu1"
	IRQ_ID_CIM = 38,
#define IRQ_NAME_CIM		"jz-cim"
	IRQ_ID_LCDC0 = 39,
#define IRQ_NAME_LCDC0		"lcdc0"
	IRQ_ID_RTC = 40,
#define IRQ_NAME_RTC		"rtc 1Hz and alarm"
	IRQ_ID_MMC1 = 44,
#define IRQ_NAME_MMC1		"jzmmc.1"
	IRQ_ID_MMC0 = 45,
#define IRQ_NAME_MMC0		"jzmmc.0"
	IRQ_ID_AOSD = 51,
#define IRQ_NAME_AOSD		"jz-aosd"
	IRQ_ID_UART3 = 56,
#define IRQ_NAME_UART3		"uart3"
	IRQ_ID_I2C3 = 65,
#define IRQ_NAME_I2C3		"jz-i2c.3"
	IRQ_ID_I2C1 = 67,
#define IRQ_NAME_I2C1		"jz-i2c.1"
	IRQ_ID_PDMAM = 69,
#define IRQ_NAME_PDMAM		"pdmam"
	IRQ_ID_VPU = 70,
#define IRQ_NAME_VPU		"vpu"
	IRQ_ID_SGXISR = 71,
#define IRQ_NAME_SGXISR		"SGX ISR"
};

struct irq_timer_monitor
{
	const char *name;
};

static struct irq_timer_monitor irq_srcs[] = {
#define DEF_IRQ(N)						\
		[IRQ_ID_##N] = { .name = IRQ_NAME_##N }

		DEF_IRQ(AIC),
		DEF_IRQ(HDMI),
		DEF_IRQ(PDMA),
		DEF_IRQ(GPIO_F),
		DEF_IRQ(GPIO_E),
		DEF_IRQ(GPIO_D),
		DEF_IRQ(GPIO_C),
		DEF_IRQ(GPIO_B),
		DEF_IRQ(GPIO_A),
		DEF_IRQ(OTG),
		DEF_IRQ(IPU0),
		DEF_IRQ(LCDC1),
		DEF_IRQ(CPU0TIMER),
		DEF_IRQ(CPU1TIMER),
		DEF_IRQ(IPU1),
		DEF_IRQ(CIM),
		DEF_IRQ(LCDC0),
		DEF_IRQ(RTC),
		DEF_IRQ(MMC1),
		DEF_IRQ(MMC0),
		DEF_IRQ(AOSD),
		DEF_IRQ(UART3),
		DEF_IRQ(I2C3),
		DEF_IRQ(I2C1),
		DEF_IRQ(PDMAM),
		DEF_IRQ(VPU),
		DEF_IRQ(SGXISR),
#undef DEF_IRQ
};
#endif

extern void __enable_irq(struct irq_desc *desc, unsigned int irq, bool resume);

static void intc_irq_ctrl(struct irq_data *data, int msk, int wkup)
{
	int intc = (int)irq_data_get_irq_chip_data(data);
	void *base = intc_base + PART_OFF * (intc/32);

	if (msk == 1)
		writel(BIT(intc%32), base + IMSR_OFF);
	else if (msk == 0)
		writel(BIT(intc%32), base + IMCR_OFF);

	if (wkup == 1)
		intc_wakeup[intc / 32] |= 1 << (intc % 32);
	else if (wkup == 0)
		intc_wakeup[intc / 32] &= ~(1 << (intc % 32));
}

static void intc_irq_unmask(struct irq_data *data)
{
	intc_irq_ctrl(data, 0, -1);
}

static void intc_irq_mask(struct irq_data *data)
{
	intc_irq_ctrl(data, 1, -1);
}

static int intc_irq_set_wake(struct irq_data *data, unsigned int on)
{
	intc_irq_ctrl(data, -1, !!on);
	return 0;
}
#ifdef CONFIG_SMP

static unsigned int cpu_irq_affinity[NR_CPUS];
static unsigned int cpu_irq_unmask[NR_CPUS];
static unsigned int cpu_mask_affinity[NR_CPUS];

static inline void set_intc_cpu(unsigned long irq_num,long cpu) {
	int mask,i;
	int num = irq_num / 32;
	mask = 1 << (irq_num % 32);
	BUG_ON(num);

	cpu_irq_affinity[cpu] |= mask;
	for(i = 0;i < NR_CPUS;i++) {
		if(i != cpu)
			cpu_irq_unmask[i] &= ~mask;
	}
}
static inline void init_intc_affinity(void) {
	int i;
	for(i = 0;i < NR_CPUS;i++) {
		cpu_irq_unmask[i] = 0xffffffff;
		cpu_irq_affinity[i] = 0;
	}
}
static int intc_set_affinity(struct irq_data *data, const struct cpumask *dest, bool force) {
	return 0;
}
#endif
static struct irq_chip jzintc_chip = {
	.name 		= "jz-intc",
	.irq_mask	= intc_irq_mask,
	.irq_mask_ack 	= intc_irq_mask,
	.irq_unmask 	= intc_irq_unmask,
	.irq_set_wake 	= intc_irq_set_wake,
#ifdef CONFIG_SMP
	.irq_set_affinity = intc_set_affinity,
#endif
};

#ifdef CONFIG_SMP
extern void jzsoc_mbox_interrupt(void);
static irqreturn_t ipi_reschedule(int irq, void *d)
{
	scheduler_ipi();
	return IRQ_HANDLED;
}

static irqreturn_t ipi_call_function(int irq, void *d)
{
	smp_call_function_interrupt();
	return IRQ_HANDLED;
}

static int setup_ipi(void)
{
	if (request_irq(IRQ_SMP_RESCHEDULE_YOURSELF, ipi_reschedule, IRQF_DISABLED,
				"ipi_reschedule", NULL))
		BUG();
	if (request_irq(IRQ_SMP_CALL_FUNCTION, ipi_call_function, IRQF_DISABLED,
				"ipi_call_function", NULL))
		BUG();

	set_c0_status(STATUSF_IP3);
	return 0;
}


#endif

static void ost_irq_unmask(struct irq_data *data)
{
	regw(0xffffffff,  OST_TMCR);
}

static void ost_irq_mask(struct irq_data *data)
{
	regw(0xffffffff,  OST_TMSR);
}

static void ost_irq_mask_ack(struct irq_data *data)
{
	regw(0xffffffff,  OST_TMSR);
	regw(0xffffffff,  OST_TFCR);  /* clear ost flag */
}

static struct irq_chip ost_irq_type = {
	.name 		= "ost",
	.irq_mask	= ost_irq_mask,
	.irq_mask_ack 	= ost_irq_mask_ack,
	.irq_unmask 	= ost_irq_unmask,
};

void __init arch_init_irq(void)
{
	int i;

	clear_c0_status(0xff04); /* clear ERL */
	set_c0_status(0x0400);   /* set IP2 */

	/* Set up MIPS CPU irq */
	mips_cpu_irq_init();

	/* Set up INTC irq */
	intc_base = ioremap(INTC_IOBASE, 0xfff);

	writel(0xffffffff, intc_base + IMSR_OFF);
	writel(0xffffffff, intc_base + PART_OFF + IMSR_OFF);
	for (i = IRQ_INTC_BASE; i < IRQ_INTC_BASE + INTC_NR_IRQS; i++) {
		irq_set_chip_data(i, (void *)(i - IRQ_INTC_BASE));
		irq_set_chip_and_handler(i, &jzintc_chip, handle_level_irq);
	}

	for (i = IRQ_OST_BASE; i < IRQ_OST_BASE + OST_NR_IRQS; i++) {
		irq_set_chip_data(i, (void *)(i - IRQ_OST_BASE));
		irq_set_chip_and_handler(i, &ost_irq_type, handle_level_irq);
	}
#ifdef CONFIG_SMP
	init_intc_affinity();
	set_intc_cpu(26,0);
	set_intc_cpu(27,1);
#endif
	/* enable cpu interrupt mask */
	set_c0_status(IE_IRQ0 | IE_IRQ1);

#ifdef CONFIG_SMP
	setup_ipi();
#endif
	return;
}

static void intc_irq_dispatch(void)
{
#ifdef IRQ_TIME_MONITOR_DEBUG
	unsigned long long time_num[2];
	unsigned long long time_sub;
	unsigned int	irq_con;
#endif
	unsigned long ipr[2],gpr[2];
	unsigned long ipr_intc;
#ifdef CONFIG_SMP
	unsigned long cpuid = smp_processor_id();
	unsigned long nextcpu;
#endif
	ipr_intc = readl(intc_base + IPR_OFF);
#ifdef CONFIG_SMP

	ipr[0] = ipr_intc & cpu_irq_unmask[cpuid];
#else
	ipr[0] = ipr_intc;
#endif
	gpr[0] = ipr[0] & 0x3f000;
	ipr[0] &= ~0x3f000;

	ipr[1] = readl(intc_base + PART_OFF + IPR_OFF);
	gpr[1] = ipr[1] & 0x9f0f0004;
	ipr[1] &= ~0x9f0f0004;

#ifdef IRQ_TIME_MONITOR_DEBUG
	time_num[0] = sched_clock();
	if(gpr[1] & 0x1f000000) {
		generic_handle_irq(ffs(gpr[1]) +31 +IRQ_INTC_BASE);
		irq_con = ffs(gpr[1]) +31 +IRQ_INTC_BASE;
	}else {

		if (ipr[0]) {
			generic_handle_irq(ffs(ipr[0]) -1 +IRQ_INTC_BASE);
			irq_con = ffs(ipr[0]) -1 +IRQ_INTC_BASE;
		}
		if (gpr[0]) {
			generic_handle_irq(ffs(gpr[0]) -1 +IRQ_INTC_BASE);
			irq_con = ffs(gpr[0]) -1 +IRQ_INTC_BASE;
		}

		if (ipr[1]) {
			generic_handle_irq(ffs(ipr[1]) +31 +IRQ_INTC_BASE);
			irq_con = ffs(ipr[1]) +31 +IRQ_INTC_BASE;
		}
		if (gpr[1]) {
			generic_handle_irq(ffs(gpr[1]) +31 +IRQ_INTC_BASE);
			irq_con = ffs(gpr[1]) +31 +IRQ_INTC_BASE;
		}
	}
	time_num[1] = sched_clock();
	time_sub = time_num[1] - time_num[0];
	if (echo_success && (time_sub > time_monitor[499]))
		time_over[irq_con] += 1;
	if (time_sub > time_monitor[irq_con])
		time_monitor[irq_con] = time_sub;
#else

	if (ipr[0]) {
		do_IRQ(ffs(ipr[0]) -1 +IRQ_INTC_BASE);
	}
	if (gpr[0]) {
		generic_handle_irq(ffs(gpr[0]) -1 +IRQ_INTC_BASE);
	}

	if (ipr[1]) {
		do_IRQ(ffs(ipr[1]) +31 +IRQ_INTC_BASE);
	}
	if (gpr[1]) {
		generic_handle_irq(ffs(gpr[1]) +31 +IRQ_INTC_BASE);
	}
#endif


#ifdef CONFIG_SMP
	nextcpu = switch_cpu_irq(cpuid);
	if(nextcpu & 0x80000000) {
		nextcpu &= ~0x80000000;
		ipr_intc = ipr_intc & cpu_irq_affinity[nextcpu];
		if(ipr_intc) {
			cpu_mask_affinity[nextcpu] |= ipr_intc;
			writel(ipr_intc, intc_base + IMSR_OFF);
		}
	}else if(nextcpu) {
		if(cpu_mask_affinity[nextcpu]) {
			writel(cpu_mask_affinity[nextcpu], intc_base + IMCR_OFF);
			cpu_mask_affinity[nextcpu] = 0;
		}
	}
#endif
}

asmlinkage void plat_irq_dispatch(void)
{
	unsigned int cause = read_c0_cause();
	unsigned int pending = cause & read_c0_status() & ST0_IM;
	if (cause & CAUSEF_IP4) {
		do_IRQ(IRQ_OST);
	}
#ifdef CONFIG_SMP
	if(pending & CAUSEF_IP3) {
		jzsoc_mbox_interrupt();
	}
#endif
	if(pending & CAUSEF_IP2)
		intc_irq_dispatch();
	cause = read_c0_cause();
	pending = cause & read_c0_status() & ST0_IM;
}

void arch_suspend_disable_irqs(void)
{
	int i,j,irq;
	struct irq_desc *desc;

	local_irq_disable();

	intc_saved[0] = readl(intc_base + IMR_OFF);
	intc_saved[1] = readl(intc_base + PART_OFF + IMR_OFF);

	writel(0xffffffff & ~intc_wakeup[0], intc_base + IMSR_OFF);
	writel(0xffffffff & ~intc_wakeup[1], intc_base + PART_OFF + IMSR_OFF);

	for(j=0;j<2;j++) {
		for(i=0;i<32;i++) {
			if(intc_wakeup[j] & (0x1<<i)) {
				irq = i + IRQ_INTC_BASE + 32*j;
				desc = irq_to_desc(irq);
				__enable_irq(desc, irq, true);
			}
		}
	}
}

void arch_suspend_enable_irqs(void)
{
	writel(0xffffffff & ~intc_saved[0], intc_base + IMCR_OFF);
	writel(0xffffffff & ~intc_saved[1], intc_base + PART_OFF + IMCR_OFF);
	local_irq_enable();
}

#ifdef IRQ_TIME_MONITOR_DEBUG
/* cat gap_time_irq to show it */
static int irq_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len = 0;
	int i;
	struct irq_desc *desc;

#define PRINT(ARGS...) len += sprintf (page+len, ##ARGS)
	PRINT("ID NAME       gap_time   over_times       dev_name\n");
	for(i = 0; i < 499; i++) {
		if (time_monitor[i] == 0)
			continue;
		else {
			desc = irq_to_desc(i);
			if (desc->action == NULL)
				PRINT("%3d %18lld   %10d       %s \n", i, time_monitor[i], time_over[i], irq_srcs[i].name);
			else
				PRINT("%3d %18lld   %10d       %s \n", i, time_monitor[i], time_over[i],desc->action->name);
		}
	}
	PRINT("EER: \n");

	return len;
}

/* echo value to clear time_monitor value*/
static int irq_write_proc(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
	int ret, i;
	char buf[32];
	unsigned long long cgr;

	if (count > 32)
		count = 32;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	ret = sscanf(buf,"%lld", &cgr);
	if (cgr == 0)
		for (i = 0; i < 499; i++) {
			time_monitor[i] = 0;
			time_over[i] = 0;
			if (i == 498)
				printk(" clear time_monitor successed!! \n");
		}
	else {
		time_monitor[499] = cgr;
		printk(" echo value successed: %lld \n", time_monitor[499]);
		echo_success = 1;
	}

	return count;
}

static int __init init_irq_proc_time(void)
{
	struct proc_dir_entry *res;

	res = create_proc_entry("gap_time_irq", 0666, NULL);
	if (res) {
		res->read_proc = irq_read_proc;
		res->write_proc = irq_write_proc;
		res->data = NULL;
	}
	return 0;
}

module_init(init_irq_proc_time);
#endif
