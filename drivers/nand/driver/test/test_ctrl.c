/*
 * test_ctrl.c
 *
 * Copyright (c) 2012 - 2015 Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "test_nand.h"

/*
 * Main initialization routine
 */

int nand_test_init(NAND_API *pnand_api)
{
	int ret;

	printk("***********JZ4770 NAND test init**********************\n");

	ret = nand_init(pnand_api);
	if (ret) {
		printk("ERROR: NAND Chip Init Failed\n");
		return -1;
	}
	return 0;
}

