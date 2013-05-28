/*
 * test_nand.h
 * define the nand driver test functions, nand test api.
 *
 * Copyright (c) 2012 - 2015 Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __TEST_NAND_H__
#define __TEST_NAND_H__

#include "nand_api.h"
#include "vnandinfo.h"   //change later

int nand_test_read_page(PPartition *ppt,unsigned int pageaddr, unsigned int offset, unsigned int bytes, void *databuf);
int nand_test_read_pages(PPartition *ppt,PageList* pagelist);
int nand_test_write_page(PPartition *ppt,unsigned int pageaddr, unsigned int offset, unsigned int bytes, void *databuf);
int nand_test_write_pages(PPartition *ppt,PageList* pagelist);
int nand_test_erase(PPartition *ppt,BlockList *blocklist);

#endif //__TEST_NAND_H__

