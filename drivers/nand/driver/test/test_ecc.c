/*
 * test_ecc.c
 *
 * Copyright (c) 2012 - 2015 Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "test_nand.h"

#define TEST_DATASIZE	(4096)
#define TEST_ECCSIZE	(218 + 512)

//#define DEBG	printk("%s: L%d\n", __FUNCTION__, __LINE__)
#define DEBG 
/* *********Global Declaration for Nand Chip********* */

/* Maybe modified as need */
unsigned char databuf[TEST_DATASIZE + TEST_OOBSIZE];
unsigned char tempbuf[TEST_DATASIZE];
unsigned char eccbuf[TEST_ECCSIZE];


int ecc_4bit(unsigned char *buf, int num)
{
	int ret;

	pnand_ecc->ecc_get_eccbytes();

	return ret;
}

int ecc_8bit(unsigned char *buf, int num)
{
	int ret;

	return ret;
}

int ecc_12bit(unsigned char *buf, int num)
{
	int ret;

	return ret;
}

int ecc_16bit(unsigned char *buf, int num)
{
	int ret;

	return ret;
}

int ecc_20bit(unsigned char *buf, int num)
{
	int ret;

	return ret;
}

int ecc_24bit(unsigned char *buf, int num)
{
	int ret;

	return ret;
}

int ecc_28bit(unsigned char *buf, int num)
{
	int ret;

	return ret;
}

int ecc_32bit(unsigned char *buf, int num)
{
	int ret;

	return ret;
}

int ecc_36bit(unsigned char *buf, int num)
{
	int ret;

	return ret;
}

