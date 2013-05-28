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

/* Message info bit for NAND delay */
#define MSG_TWHR		10
#define MSG_TWHR2		11
#define MSG_TRR  		12
#define MSG_TWB 		13
#define MSG_TADL		14
#define MSG_TCWAW		15

/* Message info bit for NAND ops */
#define MSG_NAND_BANK		0
#define MSG_DDR_ADDR		1
#define MSG_PAGEOFF		2

struct pdma_msg {
	unsigned int cmd;
	unsigned int info[16];
};

/* INTC MailBox msg */
#define MCU_MSG_NORMAL          0x1<<24
#define MCU_MSG_INTC            0x2<<24
#define MCU_MSG_INTC_MASKA      0x3<<24

/* PDMA MailBox msg */
#define MB_NAND_INIT_DONE	0x01 | MCU_MSG_NORMAL
#define MB_NAND_READ_DONE	0x02 | MCU_MSG_NORMAL
#define MB_NAND_UNCOR_ECC	0x03 | MCU_MSG_NORMAL
#define MB_NAND_WRITE_DONE	0x04 | MCU_MSG_NORMAL
#define MB_NAND_WRITE_FAIL	0x05 | MCU_MSG_NORMAL
#define MB_NAND_WRITE_PROTECT	0x06 | MCU_MSG_NORMAL
#define MB_NAND_ERASE_DONE	0x07 | MCU_MSG_NORMAL
#define MB_NAND_ERASE_FAIL	0x08 | MCU_MSG_NORMAL
#define MB_NAND_ALL_FF  	0x09 | MCU_MSG_NORMAL
#define MB_MOVE_BLOCK           0x0a | MCU_MSG_NORMAL

#endif /*__MESSAGE_H__*/
