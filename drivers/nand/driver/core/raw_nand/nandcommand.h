/********************** BEGIN LICENSE BLOCK ************************************
 *
 * INGENIC CONFIDENTIAL--NOT FOR DISTRIBUTION IN SOURCE CODE FORM
 * Copyright (c) Ingenic Semiconductor Co. Ltd 2005. All rights reserved.
 *
 * This file, and the files included with this file, is distributed and made
 * available on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND REALNETWORKS HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 *
 * http://www.ingenic.cn
 *
 ********************** END LICENSE BLOCK **************************************/



/*
 * Include file for Ingenic Semiconductor's JZ4760 CPU.
 */

#ifndef __NAND_COMMAND_H__
#define __NAND_COMMAND_H__

#define CMD_READID					0x90        //  ReadID
#define CMD_READ0					0x00        //  Read0
#define CMD_READ1					0x01        //  Read1
#define CMD_READ2					0x50        //  Read2
#define CMD_READ_CONFIRM			0x30		//	Read confirm
#define CMD_RESET					0xFF        //  Reset
#define CMD_ERASE					0x60        //  Erase phase 1
#define CMD_ERASE_CONFIRM			0xD0        //  Erase phase 2,Erase confirm
#define CMD_READSTATUS				0x70        //  Read Status
#define CMD_WRITE					0x80        //  Write phase 1
#define	CMD_WRITE_CONFIRM			0x10		//  Write phase 2
#define CMD_RANDOM_INPUT			0x85
#define CMD_RANDOM_OUTPUT			0x05
#define CMD_RANDOM_CONFIRM			0xE0
#define	CMD_RANDOM_READ				0x05
#define	CMD_RANDOM_READ_CONFIRM		0xE0
#define	CMD_RANDOM_WRITE			0x85
#define	CMD_RANDOM_WRITE_CONFIRM	0x10

#define	CMD_SET_FEATURES		0xEF
#define	CMD_GET_FEATURES		0xEE

#define CMD_2P_READ_START1          0x60   //Two-plane Read phase before addr 1 and addr 2
#define CMD_2P_READ_CONFIRM1        0x30
#define CMD_2P_READ_START2          0x00
#define CMD_2P_READ_RANDOM_OUTPUT   0x05
#define CMD_2P_READ_RANDOM_CONFIRM  0xE0

#define	CMD_COPYBACK_READ			0x00
#define	CMD_COPYBACK_READ_CONFIRM	0x35
#define	CMD_COPYBACK_WRITE			0x85
#define	CMD_COPYBACK_WRITE_CONFIRM	0x10

#define CMD_2P_PROGRAM_START1       0x80	//  Two-plane Write phase before addr 1
#define CMD_2P_PROGRAM_CONFIRM1     0x11
#define CMD_2P_PROGRAM_START2       0x81	//  Two-plane Write phase before addr 2
#define CMD_2P_PROGRAM_CONFIRM2     0x10

#define CMD_2P_ERASE_START1			0x60	//  Two-plane erase phase before addr 1 
#define CMD_2P_ERASE_START2			0x60	//  Two-plane erase phase before addr 2
#define CMD_2P_ERASE_CONFIRM		0xD0	//	Two-plane erase confirm

#define	CMD_2P_COPYBACK_READ			0x00
#define	CMD_2P_COPYBACK_READ_CONFIRM	0x35
#define	CMD_2P_COPYBACK_WRITE			0x85
#define	CMD_2P_COPYBACK_WRITE_CONFIRM	0x10

/* Status bits */
#define NAND_STATUS_FAIL        0x01
#define NAND_STATUS_FAIL_N1     0x02
#define NAND_STATUS_TRUE_READY  0x20
#define NAND_STATUS_READY       0x40
#define NAND_STATUS_WP          0x80

/* Option constants for bizarre disfunctionality and real
 * features
 */
/* Chip can not auto increment pages */
#define NAND_NO_AUTOINCR        0x00000001
/* Buswitdh is 16 bit */
#define NAND_BUSWIDTH_16        0x00000002
/* Device supports partial programming without padding */
#define NAND_NO_PADDING         0x00000004
/* Chip has cache program function */
#define NAND_CACHEPRG           0x00000008
/* Chip has copy back function */
#define NAND_COPYBACK           0x00000010
/* AND Chip which has 4 banks and a confusing page / block
 * assignment. See Renesas datasheet for further information */
#define NAND_IS_AND             0x00000020
/* Chip has a array of 4 pages which can be read without
 * additional ready /busy waits */
#define NAND_4PAGE_ARRAY        0x00000040
/* Chip requires that BBT is periodically rewritten to prevent
 * bits from adjacent blocks from 'leaking' in altering data.
 * This happens with the Renesas AG-AND chips, possibly others.  */
#define BBT_AUTO_REFRESH        0x00000080
/* Chip does not require ready check on read. True
 * for all large page devices, as they do not support
 * autoincrement.*/
#define NAND_NO_READRDY         0x00000100
/* Chip does not allow subpage writes */
#define NAND_NO_SUBPAGE_WRITE   0x00000200
/* Options valid for Samsung large page devices */
#define NAND_SAMSUNG_LP_OPTIONS \
        (NAND_NO_PADDING | NAND_CACHEPRG | NAND_COPYBACK)

#endif // __NAND_COMMAND_H__
