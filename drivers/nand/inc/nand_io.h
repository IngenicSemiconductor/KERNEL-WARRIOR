/*
 * lib_nand/jz_nand.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ_IO_H__
#define __JZ_IO_H__

/**
 * struct jz_io -
 *
 * @dataport:
 * @cmdport:
 * @addrport:
 * @buswidth:
 */
struct jz_io{
	unsigned int buswidth;
	unsigned int pagesize;
	void *dataport;
	void *cmdport;
	void *addrport;
	void (*io_init) (void *nand_io);
	void (*send_cmd_norb) (unsigned char cmd);
	int (*send_cmd_withrb) (unsigned char cmd);
	void (*send_addr) (int col, int row, int cycles);
	void (*read_data_norb) (void *buf, int len);
	void (*write_data_norb) (void *buf, int len);
	int (*read_data_withrb) (void *buf, int len);
	int (*verify_data) (void *buf, int len);
};

typedef struct jz_io JZ_IO;

#endif /* __JZ_NAND_H__ */
