/*
 * lib_nand/nand_io.c
 *
 * NAND I/O utils.
 */

#include "nand_api.h"

static JZ_IO *pnand_io;

extern int nand_wait_rb(void);

/**
 * read_data - select bus width for read
 * @buf:	data buffer
 * @count:	read length
 */
void (*read_data)(void *buf, int count);

/**
 * write_data - select bus width for write
 * @buf:	data buffer
 * @count:	write length
 */
void (*write_data)(void *buf, int count);

/**
 * verify_buf - read data from nand and compare with the given buf
 * @buf:	data buffer
 * @len:	data length
 */
int (*verify_buf)(void *buf, int len);

static inline void write_data8(unsigned char data)
{
	*((volatile unsigned char *)(pnand_io->dataport)) = data;
}

static inline void write_data16(unsigned short data)
{
	*((volatile unsigned short *)(pnand_io->dataport)) = data;
}

static inline char read_data8(void)
{
	return *((volatile unsigned char *)(pnand_io->dataport));
}

static inline short read_data16(void)
{
	return *((volatile unsigned short *)(pnand_io->dataport));
}

/**
 * nand_wait_ready - wait for R/B# state is Ready
 */
 /*
static inline void nand_wait_ready(void)
{
	int timeout = 100;
	while((REG_GPIO_PXPIN(0) & 0x00100000) && timeout--);
	while( !(REG_GPIO_PXPIN(0) & 0x00100000) );
}
*/
/**
 * __nand_cmd - write command to NAND
 * @cmd:	NAND command
 */
static inline void __nand_cmd(unsigned char cmd)
{
	//dprintf("DEBUG nand:go into nand_io.c _nand_cmd  cmdport = 0x%x  cmd =0x%0x\n",(unsigned int)pnand_io->cmdport,cmd);
	*((volatile unsigned char *)(pnand_io->cmdport)) = cmd;
}

/**
 * __nand_addr - write 1 byte NAND address to NAND
 * @addr:	NAND Address
 */
static inline void __nand_addr(unsigned char addr)
{
	*((volatile unsigned char *)(pnand_io->addrport)) = addr;
}

/**
 * send_cmd_withrb - Send command to NAND after R/B# waiting
 * @cmd:	NAND command
 */
static inline int send_cmd_withrb(unsigned char cmd)
{
	int ret;
	ret =nand_wait_rb();
	if(ret <0)
		return ret;
	__nand_cmd(cmd);
	return 0;
}

/**
 * send_cmd_norb - Send command to NAND without R/B#
 * @cmd:	NAND command
 */
static inline void send_cmd_norb(unsigned char cmd)
{
	__nand_cmd(cmd);
}

/**
 * send_addr512 - Send address to NAND, the page size of which is 512 bytes
 * @col:	colunm address
 * @row:	row address
 * @cycles:	row cycles of NAND
 */
static inline void send_addr512(int col, int row, int cycles)
{
	/* send column address */
	if (col >= 0) {
		__nand_addr(col & 0xff);
	}
	
	/* send row address */
	if (row >= 0) {
		for (; cycles > 0; cycles--) {
			__nand_addr(row & 0xff);
			row >>= 8;
		}
	}
}

/**
 * send_addr2k - Send address to NAND, the page size of which is bigger than 512 bytes
 * @col:	colunm address
 * @row:	row address
 * @cycles:	row cycles of NAND
 */
static inline void send_addr2k(int col, int row, int cycles)
{
	//dprintf("send_addr2k:col 0x%x,row 0x%08x,cycles %d\n",col,row,cycles);
	/* send column address */
	if (col >= 0) {
		__nand_addr(col & 0xff);
		__nand_addr((col >> 8) & 0xff);
	}
	
	/* send row address */
	if (row >= 0) {
		for (; cycles > 0; cycles--) {
			__nand_addr(row & 0xff);
			row >>= 8;
		}
	}
}

/**
 * write_buf8 - send data to NAND using 8 bits I/O
 * @buf:	data buffer
 * @count:	send size in byte
 */
static inline void write_buf8(void *buf, int count)
{
	int i;
	unsigned char *p = (unsigned char *)buf;
	for (i = 0; i < count; i++)
		write_data8(p[i]);
}

/**
 * write_buf16 - send data to NAND using 16 bits I/O
 * @buf:	data buffers
 * @count:	send size in byte
 */
static inline void write_buf16(void *buf, int count)
{
	int i;
	unsigned short *p = (unsigned short *)buf;
	for (i = 0; i < count / 2; i++)
		write_data16(p[i]);
}

/**
 * read_buf8 - read data from NAND using 8bits I/O
 * @buf:	data buffer
 * @count:	send size in byte
 */
static inline void read_buf8(void *buf, int count)
{
	int i;
	unsigned char *p = (unsigned char *)buf;
	for (i = 0; i < count; i++)
		p[i] = read_data8();
}

/**
 * read_buf16 - read data from NAND using 16 bits I/O
 * @buf:	data buffer
 * @count:	send size in byte
 */
static inline void read_buf16(void *buf, int count)
{
	int i;
	unsigned short *p = (unsigned short *)buf;
	for (i = 0; i < count / 2; i++)
		p[i] = read_data16();
}

/**
 * verify_buf8 - verify data to the buffer with 8 bits I/O
 * @buf:	data buffer
 * @len:	verify length
 */
static inline int verify_buf8(void *buf, int len)
{
	int i;
	unsigned char *p = (unsigned char *)buf;
	for (i = 0; i < len; i++)
		if (p[i] != read_data8())
			return -ENAND;
	return 0;
}

/**
 * verify_buf16 - verify data to the buffer with 16 bits I/O
 * @buf:	data buffer
 * @len:	verify length
 */
static inline int verify_buf16(void *buf, int len)
{
	int i;
	unsigned short *p = (unsigned short *)buf;
	for (i = 0; i < len; i++)
		if (p[i] != read_data16())
			return -ENAND;
	return 0;
}

/**
 * write_data_norb - write data without R/B#
 * @buf:	data buffer
 * @len:	wirte length
 */
static inline void write_data_norb(void *buf, int len)
{
//  dbg_line();
//  dprintf("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x \n",*(unsigned char *)buf,*((unsigned char *)buf+1),*((unsigned char *)buf+2),*((unsigned char *)buf+3),*((unsigned char *)buf+4),*((unsigned char *)buf+5),*((unsigned char *)buf+6),*((unsigned char *)buf+7));
	write_data(buf, len);
}

/**
 * read_data_wiithrb - read data with R/B#
 * @buf:	data buffer
 * @len:	read length
 */
static inline int read_data_withrb(void *buf, int len)
{
	int ret;
	ret =nand_wait_rb();
	if(ret < 0)
		return ret;

	read_data(buf, len);
	return 0;
}

/**
 * read_data_norb - read data without R/B#
 * @buf:	data buffer
 * @len:	read length
 */
static inline void read_data_norb(void *buf, int len)
{
	read_data(buf, len);
}

/**
 * verify_data - verify data buffer with R/B#
 * @buf:	data buffer
 * @len:	verify lenth
 */
static inline int verify_data(void *buf, int len)
{
	int ret;
	ret =nand_wait_rb();
	if(ret < 0)
		return ret;

	return verify_buf(buf, len);
}

static inline void io_init(void *nand_io)
{
	pnand_io = nand_io;
	
	if (pnand_io->buswidth == 8)
	{
		read_data = read_buf8;
		write_data = write_buf8;
		verify_buf = verify_buf8;
	}
	else
	{
		read_data = read_buf16;
		write_data = write_buf16;
		verify_buf = verify_buf16;
	}
	
	if(pnand_io->pagesize == 512)
		pnand_io->send_addr = send_addr512;
	else
		pnand_io->send_addr = send_addr2k;

}

JZ_IO jz_nand_io =
{
	.io_init = io_init,
	.send_cmd_norb = send_cmd_norb,
	.send_cmd_withrb = send_cmd_withrb,
	.send_addr = send_addr2k,
	.read_data_norb = read_data_norb,
	.write_data_norb = write_data_norb,
	.read_data_withrb = read_data_withrb,
	.verify_data = verify_data,
//	.wait_ready = nand_wait_ready,
};
