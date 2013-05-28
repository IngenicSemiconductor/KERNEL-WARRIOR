/*
 * test_erase.c
 *
 * Copyright (c) 2012 - 2015 Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "test_nand.h"

extern NandInterface *vnand_interface;
int nand_test_erase(PPartition *ppt,BlockList *blocklist)
{
	int state;
	state =vnand_interface->iMultiBlockErase(ppt,blocklist);
        if (state < 0) {
		eprintf(">>>Erase block %d Failed\n",blocklist->startBlock);
        }
        return state;
}

