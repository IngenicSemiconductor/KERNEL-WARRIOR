/*
 * include/nand.h
 */
#ifndef __NAND_H__
#define __NAND_H__

//#define PIPE_LEVEL		2

#define PNAND_NEMC		(1 << 0)
#define PNAND_DDR		(1 << 1)
#define PNAND_BCH_ENC		(1 << 2)
#define PNAND_BCH_DEC		(1 << 3)
#define PNAND_HALT		(1 << 4)

#define CTRL_READ_DATA		(1 << 0)
#define CTRL_READ_OOB		(1 << 1)
#define CTRL_WRITE_DATA		(1 << 2)
#define CTRL_WRITE_OOB		(1 << 3)
#define CTRL_WRITE_CONFIRM	(1 << 4)

#define NANDTYPE_COMMON		0
#define NANDTYPE_TOGGLE		1

/*
 * Standard NAND flash commands
 */
#define NAND_CMD_READ0		0x00
#define NAND_CMD_RNDOUT		0x05
#define NAND_CMD_PAGEPROG	0x10
#define NAND_CMD_ERASE1		0x60
#define NAND_CMD_STATUS		0x70
#define NAND_CMD_SEQIN		0x80
#define NAND_CMD_RNDIN		0x85
#define NAND_CMD_READID		0x90
#define NAND_CMD_ERASE2		0xD0
#define NAND_CMD_RESET		0xFF

/* Extended commands for large page devices */
#define NAND_CMD_READSTART      0x30
#define NAND_CMD_RNDOUTSTART    0xE0
#define NAND_CMD_CACHEDPROG     0x15

/* Status Bits */
#define NAND_STATUS_FAIL	0x01
#define NAND_STATUS_FAIL_N1	0x02
#define NAND_STATUS_TRUE_READY	0x20
#define NAND_STATUS_READY	0x40
#define NAND_STATUS_WP		0x80

#define NAND_CMD_OFFSET		0x00400000
#define NAND_ADDR_OFFSET	0x00800000

#define __nand_cmd(cmd, port)		(REG8((port) | NAND_CMD_OFFSET) = (cmd))
#define __nand_addr(addr, port)		(REG8((port) | NAND_ADDR_OFFSET) = (addr))
#define __nand_status(state, port)	((state) = REG8(port))

struct nand_chip {
	void *nand_io;
	void *ddr_addr;

	int nandtype;
	int pagesize;
	int oobsize;
	int rowcycle;
	int chipsel;

	int ecclevel;
	int eccsize;
	int eccbytes;
	int eccsteps;
	int ecctotal;
	int eccpos;

        int twhr;
        int twhr2;
        int trr;
        int twb;
        int tadl;
        int tcwaw;

	int pipe_cnt;

	int ctrl;
	int mode;
	int report;
	
	unsigned int mcu_test;
	void *priv;
};

struct nand_pipe_buf {
	unsigned char *pipe_data;
	unsigned char *pipe_par;
};

#define NAND_DATA_PORT1         0x1B000000      /* read-write area in static bank 1 */
#define NAND_DATA_PORT2         0x1A000000      /* read-write area in static bank 2 */
#define NAND_DATA_PORT3         0x19000000      /* read-write area in static bank 3 */
#define NAND_DATA_PORT4         0x18000000      /* read-write area in static bank 4 */
#define NAND_DATA_PORT5         0x17000000
#define NAND_DATA_PORT6         0x16000000

extern void pdma_bch_calculate(struct nand_chip *nand, struct nand_pipe_buf *pipe_buf);
extern void pdma_bch_correct(struct nand_chip *nand, struct nand_pipe_buf *pipe_buf, unsigned char *par_buf);

extern void pdma_nand_init(struct nand_chip *nand, struct nand_pipe_buf *pipe_buf,
			unsigned int *info, unsigned char *oob_buf);
extern void inline pdma_nand_status(struct nand_chip *nand, u8 *status);
extern void pdma_nand_read_ctrl(struct nand_chip *nand, int page, const int ctrl);
extern void pdma_nand_read_data(struct nand_chip *nand, struct nand_pipe_buf *pipe_buf,
			unsigned char *par_buf);
extern void pdma_nand_read_oob(struct nand_chip *nand, unsigned char *oob_buf);
extern int pdma_nand_write_ctrl(struct nand_chip *nand, int page, const int ctrl);
extern void pdma_nand_write_data(struct nand_chip *nand, struct nand_pipe_buf *pipe_buf,
			unsigned char *ddr_addr);
extern void pdma_nand_write_oob(struct nand_chip *nand, unsigned char *oob_buf);

extern int pdma_nand_erase(struct nand_chip *nand, int page);

#endif /*__NAND_H__*/
