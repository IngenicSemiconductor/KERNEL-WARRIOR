/*
 * test_write.c
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
#define TEST_OOBSIZE	(218 + 512)

/* Maybe modified as need */
unsigned int local_writebuf[TEST_DATASIZE + TEST_OOBSIZE];

extern NandInterface *vnand_interface;

int nand_test_write_page(PPartition *ppt,unsigned int pageaddr, unsigned int offset, unsigned int bytes, void *databuf)
{
	int ret;
	
	if(!databuf)
	 databuf = local_writebuf;

	ret = vnand_interface->iPageWrite(ppt,pageaddr, offset ,bytes ,databuf);

	return ret;
}

int nand_test_write_pages(PPartition *ppt,PageList* pagelist)
{
	int ret;
	

	ret = vnand_interface->iMultiPageWrite(ppt,pagelist);

	return ret;
}

