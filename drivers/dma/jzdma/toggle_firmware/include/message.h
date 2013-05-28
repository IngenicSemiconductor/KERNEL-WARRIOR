/*
 * message.h
 */
#ifndef __MESSAGE_H__
#define __MESSAGE_H__

/* Message for NAND cmd */
#define MSG_NAND_INIT		0x01
#define MSG_NAND_READ		0x02
#define MSG_NAND_WRITE		0x03
#define MSG_NAND_ERASE		0x04

/* Message info bit for NAND init */
#define MSG_NANDTYPE		0
#define MSG_PAGESIZE		1
#define MSG_OOBSIZE		2
#define MSG_ROWCYCLE		3
#define MSG_ECCLEVEL		4
#define MSG_ECCSIZE		5
#define MSG_ECCBYTES		6
#define MSG_ECCSTEPS		7
#define MSG_ECCTOTAL		8
#define MSG_ECCPOS		9

/* Message info bit for NAND ops */
#define MSG_NAND_BANK		0
#define MSG_DDR_ADDR		1
#define MSG_PAGEOFF		2

struct pdma_msg {
	unsigned int cmd;
	unsigned int info[10];
};

/* PDMA MailBox msg */
#define MB_NAND_INIT_DONE	0x01
#define MB_NAND_READ_DONE	0x02
#define MB_NAND_UNCOR_ECC	0x03
#define MB_NAND_WRITE_DONE	0x04
#define MB_NAND_WRITE_FAIL	0x05
#define MB_NAND_WRITE_PROTECT	0x06
#define MB_NAND_ERASE_DONE	0x07
#define MB_NAND_ERASE_FAIL	0x08

#endif /*__MESSAGE_H__*/
