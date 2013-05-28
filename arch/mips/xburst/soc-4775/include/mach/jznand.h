/*
 *  linux/include/linux/mtd/nand.h
 *
 *  Copyright (c) 2000 David Woodhouse <dwmw2@infradead.org>
 *                     Steven J. Hill <sjhill@realitydiluted.com>
 *		       Thomas Gleixner <tglx@linutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Info:
 *	Contains standard defines and IDs for NAND flash devices
 *
 * Changelog:
 *	See git changelog.
 */
#ifndef __LINUX_MTD_NAND_H
#define __LINUX_MTD_NAND_H

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define dprintk(x...)	printk(x)
#else
#define dprintk(x...)
#endif

#define SPL_MANAGER  	0
#define DIRECT_MANAGER 	1
#define ZONE_MANAGER   	2
#define ONCE_MANAGER  	3

#define PART_OOB       0
#define PART_NO_OOB    1
#define PART_PROTECT   2
#define PART_NANDSPL   3
#define PART_DEVICE_ID 4
#define MUL_PARTS 4
enum part_attribute{
	PART_XBOOT,
	PART_KERNEL,
	PART_RECOVERY,
	PART_SYSTEM,
	PART_DATA,
	PART_MISC
};
enum operation_mode{
	ONE_PLANE,
	TWO_PLANES
};

/**
 * struct platform_nand_ex_partition
 * an element in platform_nand_partition
 * the member is as same as its
 */
struct platform_nand_ex_partition{
	long long offset;
	long long size;
	char *name;
};
/**
 * struct platform_nand_chip - chip level device structure
 * @name:		the name of this partition
 * @offset:	    offset within the master MTD space 
 * @size:	    partition size
 * @mode:		partition mode     0  DIRECT_MANAGER ; 1  ZONE_MANAGER 
 * @eccbit:		the number of eccbit  per partition 
 * @use_planes: operation mode     one_plane operation or two_plane operation
 * @part_attrib: partion attribute
 */
 
struct  platform_nand_partition{
	long long offset;
	long long size;
	const char *name;
	int mode;
	short eccbit;
	unsigned char use_planes;
	enum part_attribute part_attrib;
	struct platform_nand_ex_partition ex_partition[MUL_PARTS];
};

/**
 * struct platform_nand_data - container structure for platform-specific data
 * @nr_partitions:	number of partitions pointed to by partitions (or zero)
 * @partitions:		platform_nand_partition list
 * @priv:		hardware controller specific settings
 */
struct platform_nand_data {
	int	nr_partitions;
	struct platform_nand_partition	*partitions;
	void	*priv;
        unsigned        gpio_wp;
};

#endif /* __LINUX_MTD_NAND_H */
