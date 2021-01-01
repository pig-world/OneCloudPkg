/** @file

  Copyright (c) 2008 - 2009, Apple Inc. All rights reserved.<BR>

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef FLASH_H
#define FLASH_H

#include <Uefi.h>
#include <Library/UefiLib.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>

#include <Protocol/BlockIo.h>
#include "reg_addr.h"

#define IO_CBUS_BASE                                                      0xc1100000

#define setbits_le32(reg,mask)                                            MmioWrite32(reg,MmioRead32(reg)|(mask))
#define clrbits_le32(reg,mask)                                            MmioWrite32(reg,MmioRead32(reg)&(~(mask)))
#define clrsetbits_le32(reg,clr,mask)                                     MmioWrite32(reg,(MmioRead32(reg)&(~(clr)))|(mask))

#define CBUS_REG_OFFSET(reg)                                              ((reg) << 2)
#define CBUS_REG_ADDR(reg)                                                (IO_CBUS_BASE + CBUS_REG_OFFSET(reg))

#define WRITE_CBUS_REG(reg, val)                                          MmioWrite32(CBUS_REG_ADDR(reg),val)
#define READ_CBUS_REG(reg)                                                MmioRead32(CBUS_REG_ADDR(reg))
#define WRITE_CBUS_REG_BITS(reg, val, start, len)                         WRITE_CBUS_REG(reg,	(READ_CBUS_REG(reg) & ~(((1L<<(len))-1)<<(start)) )| ((unsigned)((val)&((1L<<(len))-1)) << (start)))
#define READ_CBUS_REG_BITS(reg, start, len)                               ((READ_CBUS_REG(reg) >> (start)) & ((1L<<(len))-1))
#define CLEAR_CBUS_REG_MASK(reg, mask)                                    WRITE_CBUS_REG(reg, (READ_CBUS_REG(reg)&(~(mask))))
#define SET_CBUS_REG_MASK(reg, mask)                                      WRITE_CBUS_REG(reg, (READ_CBUS_REG(reg)|(mask)))

#define PREG_SDIO_CFG     	CBUS_REG_ADDR(SDIO_CONFIG)
#define PREG_SDIO_CMD_ARG 	CBUS_REG_ADDR(CMD_ARGUMENT)
#define PREG_SDIO_CMD_SEND  CBUS_REG_ADDR(CMD_SEND)
#define PREG_SDIO_MULT_CFG  CBUS_REG_ADDR(SDIO_MULT_CONFIG)
#define PREG_SDIO_EXT		CBUS_REG_ADDR(SDIO_EXTENSION)
#define PREG_SDIO_MEM_ADDR  CBUS_REG_ADDR(SDIO_M_ADDR)
#define PREG_SDIO_STAT_IRQ  CBUS_REG_ADDR(SDIO_STATUS_IRQ)
#define PREG_SDIO_IRQ_CFG   CBUS_REG_ADDR(SDIO_IRQ_CONFIG)

#define SD_VERSION_SD	0x20000
#define SD_VERSION_2	(SD_VERSION_SD | 0x20)
#define SD_VERSION_1_0	(SD_VERSION_SD | 0x10)
#define SD_VERSION_1_10	(SD_VERSION_SD | 0x1a)
#define MMC_VERSION_MMC		0x10000
#define MMC_VERSION_UNKNOWN	(MMC_VERSION_MMC)
#define MMC_VERSION_1_2		(MMC_VERSION_MMC | 0x12)
#define MMC_VERSION_1_4		(MMC_VERSION_MMC | 0x14)
#define MMC_VERSION_2_2		(MMC_VERSION_MMC | 0x22)
#define MMC_VERSION_3		(MMC_VERSION_MMC | 0x30)
#define MMC_VERSION_4		(MMC_VERSION_MMC | 0x40)

#define MMC_MODE_HS		0x001
#define MMC_MODE_HS_52MHz	0x010
#define MMC_MODE_4BIT		0x100
#define MMC_MODE_8BIT		0x200

#define SD_DATA_4BIT	0x00040000

#define IS_SD(x) (x->version & SD_VERSION_SD)

#define MMC_DATA_READ		1
#define MMC_DATA_WRITE		2

#define NO_CARD_ERR		-16 /* No SD/MMC card inserted */
#define UNUSABLE_ERR		-17 /* Unusable Card */
#define COMM_ERR		-18 /* Communications Error */
#define TIMEOUT			-19

#define MMC_CMD_GO_IDLE_STATE		0
#define MMC_CMD_SEND_OP_COND		1
#define MMC_CMD_ALL_SEND_CID		2
#define MMC_CMD_SET_RELATIVE_ADDR	3
#define MMC_CMD_SET_DSR			4
#define MMC_CMD_SWITCH			6
#define MMC_CMD_SELECT_CARD		7
#define MMC_CMD_SEND_EXT_CSD		8
#define MMC_CMD_SEND_CSD		9
#define MMC_CMD_SEND_CID		10
#define MMC_CMD_STOP_TRANSMISSION	12
#define MMC_CMD_SEND_STATUS		13
#define MMC_CMD_SET_BLOCKLEN		16
#define MMC_CMD_READ_SINGLE_BLOCK	17
#define MMC_CMD_READ_MULTIPLE_BLOCK	18
#define MMC_CMD_WRITE_SINGLE_BLOCK	24
#define MMC_CMD_WRITE_MULTIPLE_BLOCK	25
#define MMC_CMD_APP_CMD			55

#define MMC_TAG_SECTOR_START            32	// R1
#define MMC_TAG_SECTOR_END              33	// R1
#define MMC_UNTAG_SECTOR                34	// R1
#define MMC_TAG_ERASE_GROUP_START       35	// R1
#define MMC_TAG_ERASE_GROUP_END         36	// R1
#define MMC_UNTAG_ERASE_GROUP           37	// R1
#define SD_MMC_ERASE                    38	// R1b

#define SD_ERASE_WR_BLK_START           32	//  R1
#define SD_ERASE_WR_BLK_END             33	//   R1

#define SD_CMD_SEND_RELATIVE_ADDR	3
#define SD_CMD_SWITCH_FUNC		6
#define SD_CMD_SEND_IF_COND		8

#define SD_CMD_APP_SET_BUS_WIDTH	6
#define SD_CMD_APP_SEND_OP_COND		41
#define SD_CMD_APP_SEND_SCR		51

/* SCR definitions in different words */
#define SD_HIGHSPEED_BUSY	0x00020000
#define SD_HIGHSPEED_SUPPORTED	0x00020000

#define MMC_HS_TIMING		0x00000100
#define MMC_HS_52MHZ		0x2

#define OCR_BUSY	0x80000000
#define OCR_HCS		0x40000000

#define MMC_VDD_165_195		0x00000080	/* VDD voltage 1.65 - 1.95 */
#define MMC_VDD_20_21		0x00000100	/* VDD voltage 2.0 ~ 2.1 */
#define MMC_VDD_21_22		0x00000200	/* VDD voltage 2.1 ~ 2.2 */
#define MMC_VDD_22_23		0x00000400	/* VDD voltage 2.2 ~ 2.3 */
#define MMC_VDD_23_24		0x00000800	/* VDD voltage 2.3 ~ 2.4 */
#define MMC_VDD_24_25		0x00001000	/* VDD voltage 2.4 ~ 2.5 */
#define MMC_VDD_25_26		0x00002000	/* VDD voltage 2.5 ~ 2.6 */
#define MMC_VDD_26_27		0x00004000	/* VDD voltage 2.6 ~ 2.7 */
#define MMC_VDD_27_28		0x00008000	/* VDD voltage 2.7 ~ 2.8 */
#define MMC_VDD_28_29		0x00010000	/* VDD voltage 2.8 ~ 2.9 */
#define MMC_VDD_29_30		0x00020000	/* VDD voltage 2.9 ~ 3.0 */
#define MMC_VDD_30_31		0x00040000	/* VDD voltage 3.0 ~ 3.1 */
#define MMC_VDD_31_32		0x00080000	/* VDD voltage 3.1 ~ 3.2 */
#define MMC_VDD_32_33		0x00100000	/* VDD voltage 3.2 ~ 3.3 */
#define MMC_VDD_33_34		0x00200000	/* VDD voltage 3.3 ~ 3.4 */
#define MMC_VDD_34_35		0x00400000	/* VDD voltage 3.4 ~ 3.5 */
#define MMC_VDD_35_36		0x00800000	/* VDD voltage 3.5 ~ 3.6 */

#define MMC_SWITCH_MODE_CMD_SET		0x00 /* Change the command set */
#define MMC_SWITCH_MODE_SET_BITS	0x01 /* Set bits in EXT_CSD byte
						addressed by index which are
						1 in value field */
#define MMC_SWITCH_MODE_CLEAR_BITS	0x02 /* Clear bits in EXT_CSD byte
						addressed by index, which are
						1 in value field */
#define MMC_SWITCH_MODE_WRITE_BYTE	0x03 /* Set target byte to value */

#define SD_SWITCH_CHECK		0
#define SD_SWITCH_SWITCH	1

/*
 * EXT_CSD fields
 */

#define EXT_CSD_PART_CONFIG	179	/* R/W */
#define EXT_CSD_BUS_WIDTH	183	/* R/W */
#define EXT_CSD_HS_TIMING	185	/* R/W */
#define EXT_CSD_CARD_TYPE	196	/* RO */
#define EXT_CSD_REV		192	/* RO */
#define EXT_CSD_SEC_CNT		212	/* RO, 4 bytes */
#define EXT_CSD_BOOT_MULT	226	/* RO */

/*
 * EXT_CSD field definitions
 */

#define EXT_CSD_CMD_SET_NORMAL		(1<<0)
#define EXT_CSD_CMD_SET_SECURE		(1<<1)
#define EXT_CSD_CMD_SET_CPSECURE	(1<<2)

#define EXT_CSD_CARD_TYPE_26	(1<<0)	/* Card can run at 26MHz */
#define EXT_CSD_CARD_TYPE_52	(1<<1)	/* Card can run at 52MHz */

#define EXT_CSD_BUS_WIDTH_1	0	/* Card is in 1 bit mode */
#define EXT_CSD_BUS_WIDTH_4	1	/* Card is in 4 bit mode */
#define EXT_CSD_BUS_WIDTH_8	2	/* Card is in 8 bit mode */

#define R1_ILLEGAL_COMMAND		(1 << 22)
#define R1_APP_CMD			(1 << 5)

#define MMC_RSP_PRESENT (1 << 0)
#define MMC_RSP_136     (1 << 1)                /* 136 bit response */
#define MMC_RSP_CRC     (1 << 2)                /* expect valid crc */
#define MMC_RSP_BUSY    (1 << 3)                /* card may send busy */
#define MMC_RSP_OPCODE  (1 << 4)                /* response contains opcode */
#define MMC_RSP_R5_FLAG  (1 << 5)
#define MMC_RSP_R6_FLAG  (1 << 6)
#define MMC_RSP_R7_FLAG  (1 << 7)

#define MMC_RSP_NONE    (0)
#define MMC_RSP_R1      (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R1b	(MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE| \
			MMC_RSP_BUSY)
#define MMC_RSP_R2      (MMC_RSP_PRESENT|MMC_RSP_136|MMC_RSP_CRC)
#define MMC_RSP_R3      (MMC_RSP_PRESENT)
#define MMC_RSP_R4      (MMC_RSP_PRESENT)
#define MMC_RSP_R5      (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE|MMC_RSP_R5_FLAG)
#define MMC_RSP_R6      (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE|MMC_RSP_R6_FLAG)
#define MMC_RSP_R7      (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE|MMC_RSP_R7_FLAG)


#define R1_OUT_OF_RANGE		(1 << 31)	/* er, c */
#define R1_ADDRESS_ERROR	(1 << 30)	/* erx, c */
#define R1_BLOCK_LEN_ERROR	(1 << 29)	/* er, c */
#define R1_ERASE_SEQ_ERROR      (1 << 28)	/* er, c */
#define R1_ERASE_PARAM		(1 << 27)	/* ex, c */
#define R1_WP_VIOLATION		(1 << 26)	/* erx, c */
#define R1_CARD_IS_LOCKED	(1 << 25)	/* sx, a */
#define R1_LOCK_UNLOCK_FAILED	(1 << 24)	/* erx, c */
#define R1_COM_CRC_ERROR	(1 << 23)	/* er, b */
#define R1_ILLEGAL_COMMAND	(1 << 22)	/* er, b */
#define R1_CARD_ECC_FAILED	(1 << 21)	/* ex, c */
#define R1_CC_ERROR		(1 << 20)	/* erx, c */
#define R1_ERROR		(1 << 19)	/* erx, c */
#define R1_UNDERRUN		(1 << 18)	/* ex, c */
#define R1_OVERRUN		(1 << 17)	/* ex, c */
#define R1_CID_CSD_OVERWRITE	(1 << 16)	/* erx, c, CID/CSD overwrite */
#define R1_WP_ERASE_SKIP	(1 << 15)	/* sx, c */
#define R1_CARD_ECC_DISABLED	(1 << 14)	/* sx, a */
#define R1_ERASE_RESET		(1 << 13)	/* sr, c */
#define R1_STATUS(x)            (x & 0xFFFFE000)
#define R1_CURRENT_STATE(x)	((x & 0x00001E00) >> 9)	/* sx, b (4 bits) */
#define R1_READY_FOR_DATA	(1 << 8)	/* sx, a */
#define R1_SWITCH_ERROR		(1 << 7)	/* sx, c */
#define R1_EXCEPTION_EVENT	(1 << 6)	/* sx, a */
#define R1_APP_CMD		(1 << 5)	/* sr, c */

#define R1_STATE_IDLE	0
#define R1_STATE_READY	1
#define R1_STATE_IDENT	2
#define R1_STATE_STBY	3
#define R1_STATE_TRAN	4
#define R1_STATE_DATA	5
#define R1_STATE_RCV	6
#define R1_STATE_PRG	7
#define R1_STATE_DIS	8

#define	SD_READ_BUSY_COUNT                                                5000000//20
#define	SD_WRITE_BUSY_COUNT                                               1000000//500000
#define	SD_RETRY_COUNT                                                    8




//Register defines
typedef struct SDHW_CMD_Argument_Reg
{
	unsigned int arg;
} SDHW_CMD_Argument_Reg_t;

typedef struct SDHW_CMD_Send_Reg
{
	unsigned cmd_data: 8;							// Bit 7:0
	unsigned cmd_res_bits: 8;						// Bit 15:8
	unsigned res_without_crc7: 1;					// Bit 16
	unsigned res_with_data: 1;						// Bit 17
	unsigned res_crc7_from_8: 1;					// Bit 18
	unsigned check_dat0_busy: 1;					// Bit 19
	unsigned cmd_send_data: 1;						// Bit 20
	unsigned use_int_window: 1;						// Bit 21
	unsigned reserved: 2;						    // Bit 22:23
	unsigned repeat_package_times: 8;				// Bit 31:24
} __attribute__((__may_alias__)) SDHW_CMD_Send_Reg_t;

typedef struct SDIO_Config_Reg
{
	unsigned cmd_clk_divide: 10;					// Bit 9:0
	unsigned cmd_disable_crc: 1;					// Bit 10
	unsigned cmd_out_at_posedge: 1;					// Bit 11
	unsigned cmd_argument_bits: 6;					// Bit 17:12
	unsigned res_latch_at_negedge: 1;				// Bit 18
	unsigned data_latch_at_negedge: 1;				// Bit 19
	unsigned bus_width: 1;							// Bit 20
	unsigned m_endian: 2;							// Bit 22:21
	unsigned write_Nwr: 6;							// Bit 28:23
	unsigned write_crc_ok_status: 3;				// Bit 31:29
}  __attribute__((__may_alias__)) SDIO_Config_Reg_t;

typedef struct SDIO_Status_IRQ_Reg
{
	unsigned status: 4;								// Bit 3:0
	unsigned cmd_busy: 1;							// Bit 4
	unsigned res_crc7_ok: 1;						// Bit 5
	unsigned data_read_crc16_ok: 1;					// Bit 6
	unsigned data_write_crc16_ok: 1;				// Bit 7
	unsigned if_int: 1;								// Bit 8
	unsigned cmd_int: 1;							// Bit 9
	unsigned soft_int: 1;							// Bit 10
	unsigned set_soft_int: 1;						// Bit 11
	unsigned status_info: 4;						// Bit 15:12
	unsigned timing_out_int: 1;						// Bit 16
	unsigned amrisc_timing_out_int_en:1;				// Bit 17
	unsigned arc_timing_out_int_en: 1;				// Bit 18
	unsigned timing_out_count: 13;					// Bit 31:19
}  __attribute__((__may_alias__)) SDIO_Status_IRQ_Reg_t;

typedef struct SDHW_IRQ_Config_Reg
{
	unsigned amrisc_if_int_en: 1;					// Bit 0
	unsigned amrisc_cmd_int_en: 1;					// Bit 1
	unsigned amrisc_soft_int_en: 1;					// Bit 2
	unsigned arc_if_int_en: 1;						// Bit 3
	unsigned arc_cmd_int_en: 1;						// Bit 4
	unsigned arc_soft_int_en: 1;					// Bit 5
	unsigned if_int_config: 2;						// Bit 7:6
	unsigned data: 6;								// Bit 13:8
	unsigned force_enable: 1;						// Bit 14
	unsigned soft_reset: 1;							// Bit 15
	unsigned force_output_en: 6;					// Bit 21:16
	unsigned diable_mem_halt: 2;					// Bit 23:22
	unsigned force_data_read: 6;					// Bit 29:24
	unsigned force_halt: 1;							// Bit 30
	unsigned halt_hole:1;								// Bit 31
}  __attribute__((__may_alias__)) SDHW_IRQ_Config_Reg_t;

typedef struct MSHW_IRQ_Config_Reg
{
	unsigned amrisc_if_int_en: 1;					// Bit 0
	unsigned amrisc_cmd_int_en: 1;					// Bit 1
	unsigned amrisc_soft_int_en: 1;					// Bit 2
	unsigned arc_if_int_en: 1;						// Bit 3
	unsigned arc_cmd_int_en: 1;						// Bit 4
	unsigned arc_soft_int_en: 1;					// Bit 5
	unsigned if_int_config: 2;						// Bit 7:6
	unsigned data0: 1;								// Bit 8
	unsigned data1: 1;								// Bit 9
	unsigned data2: 1;								// Bit 10
	unsigned data3: 1;								// Bit 11
	unsigned bs: 1;									// Bit 12
	unsigned sclk: 1;								// Bit 13
	unsigned force_enable: 1;						// Bit 14
	unsigned soft_reset: 1;							// Bit 15
	unsigned force_output_en: 6;					// Bit 21:16
	unsigned diable_mem_halt: 2;					// Bit 23:22
	unsigned force_data_read: 6;					// Bit 29:24
	unsigned force_halt: 1;							// Bit 30
	unsigned halt_hole:1;								// Bit 31
}  __attribute__((__may_alias__)) MSHW_IRQ_Config_Reg_t;

typedef struct SDIO_Multi_Config_Reg
{
	unsigned port_sel: 2;							// Bit 1:0
	unsigned ms_enable: 1;							// Bit 2
	unsigned ms_sclk_always: 1;						// Bit 3
	unsigned stream_enable: 1;						// Bit 4
	unsigned stream_8_bits_mode: 1;					// Bit 5
	unsigned data_catch_level: 2;					// Bit 7:6
	unsigned write_read_out_index: 1;				// Bit 8
	unsigned data_catch_out_en: 1;					// Bit 9
	unsigned reserved: 2;							// Bit 11:10
	unsigned res_read_index: 4;						// Bit 15:12
	unsigned data_catch_finish_point: 12;			// Bit 27:16
	unsigned reserved1: 4;							// Bit 31:28
}  __attribute__((__may_alias__)) SDIO_Multi_Config_Reg_t;

typedef struct SDIO_M_Addr_Reg
{
	unsigned int m_addr;
}  __attribute__((__may_alias__)) SDIO_M_Addr_Reg_t;

typedef struct SDHW_Extension_Reg
{
	unsigned cmd_arg_ext: 16;						// Bit 15:0
	unsigned data_rw_number: 14;					// Bit 29:16
	unsigned data_rw_without_crc16: 1;				// Bit 30
	unsigned crc_status_4line: 1;					// Bit 31
} __attribute__((__may_alias__)) SDHW_Extension_Reg_t;                             	

//Never change any sequence of following data variables
#pragma pack(1)

//LSB -> MSB, structrue for SD Card Status
typedef struct _SD_Card_Status
{
	unsigned Reserved3: 2;
	unsigned Reserved4: 1;
	unsigned AKE_SEQ_ERROR: 1;                  //Error in the sequence of authentication process.
	unsigned Reserved5: 1;
	unsigned APP_CMD: 1;                        //The card will expect ACMD, or indication that the command has been interpreted as ACMD.
	unsigned NotUsed: 2;
	
	unsigned READY_FOR_DATA: 1;                 //Corresponds to buffer empty signalling on the bus.
	unsigned CURRENT_STATE: 4;                  //The state of the card when receiving the command. 
	unsigned ERASE_RESET: 1;                    //An erase sequence was cleared beforem executing because an out of erase sequence command was received.
	unsigned CARD_ECC_DISABLED: 1;              //The command has been executed without using the internal ECC.
	unsigned WP_ERASE_SKIP: 1;                  //Only partial address space was erased due to existing write protected blocks.
	
	unsigned CID_CSD_OVERWRITE: 1;              //Can be either one of the following errors:
	unsigned Reserved1: 1;
	unsigned Reserved2: 1;
	unsigned ERROR: 1;                          //A general or an unknown error occurred during the operation.
	unsigned CC_ERROR: 1;                       //Internal card controller error
	unsigned CARD_ECC_FAILED: 1;                //Card internal ECC was applied but failed to correct the data.
	unsigned ILLEGAL_COMMAND: 1;                //Command not legal for the card state
	unsigned COM_CRC_ERROR: 1;                  //The CRC check of the previous command failed.
	
    unsigned LOCK_UNLOCK_FAILED: 1;             //Set when a sequence or password error has been detected in lock/ unlock card command or if there was an attempt to access a locked card
	unsigned CARD_IS_LOCKED: 1;                 //When set, signals that the card is locked by the host
	unsigned WP_VIOLATION: 1;                   //Attempt to program a write-protected block.
	unsigned ERASE_PARAM: 1;                    //An invalid selection of write-blocks for erase occurred.
	unsigned ERASE_SEQ_ERROR: 1;                //An error in the sequence of erase commands occurred.
	unsigned BLOCK_LEN_ERROR: 1;                //The transferred block length is not allowed for this card, or the number of transferred bytes does not match the block length.
	unsigned ADDRESS_ERROR: 1;                  //A misaligned address that did not match the block length was used in the command.
	unsigned OUT_OF_RANGE: 1;                   //The command??s argument was out of the allowed range for this card.
	
} SD_Card_Status_t;

//structure for response
typedef struct _SD_Response_R1
{
	SD_Card_Status_t card_status;               //card status
} SD_Response_R1_t;
#pragma pack()
/* Error codes */
typedef enum _SD_Error_Status_t { 
	SD_NO_ERROR                 = 0,
	SD_ERROR_OUT_OF_RANGE,                  //Bit 31
	SD_ERROR_ADDRESS,                       //Bit 30 
	SD_ERROR_BLOCK_LEN,                     //Bit 29
	SD_ERROR_ERASE_SEQ,                     //Bit 28
	SD_ERROR_ERASE_PARAM,                   //Bit 27
	SD_ERROR_WP_VIOLATION,                  //Bit 26
	SD_ERROR_CARD_IS_LOCKED,                    //Bit 25
	SD_ERROR_LOCK_UNLOCK_FAILED,                //Bit 24
	SD_ERROR_COM_CRC,                       //Bit 23
	SD_ERROR_ILLEGAL_COMMAND,               //Bit 22
	SD_ERROR_CARD_ECC_FAILED,                   //Bit 21
	SD_ERROR_CC,                                //Bit 20
	SD_ERROR_GENERAL,                       //Bit 19
	SD_ERROR_Reserved1,                         //Bit 18
	SD_ERROR_Reserved2,                         //Bit 17
	SD_ERROR_CID_CSD_OVERWRITE,             //Bit 16
	SD_ERROR_AKE_SEQ,                           //Bit 03
	SD_ERROR_STATE_MISMATCH,
	SD_ERROR_HEADER_MISMATCH,
	SD_ERROR_DATA_CRC,
	SD_ERROR_TIMEOUT,  
	SD_ERROR_DRIVER_FAILURE,
	SD_ERROR_WRITE_PROTECTED,
	SD_ERROR_NO_MEMORY,
	SD_ERROR_SWITCH_FUNCTION_COMUNICATION,
	SD_ERROR_NO_FUNCTION_SWITCH,
	SD_ERROR_NO_CARD_INS
} SD_Error_Status_t;

typedef enum _SD_Bus_Width
{
	SD_BUS_SINGLE                   = 1,        //only DAT0
	SD_BUS_WIDE                     = 4         //use DAT0-4
} SD_Bus_Width_t;


//Misc definitions
#define MAX_RESPONSE_BYTES              18

#define RESPONSE_R1_R3_R6_R7_LENGTH     6
#define RESPONSE_R2_CID_CSD_LENGTH      17
#define RESPONSE_R4_R5_NONE_LENGTH      0

// for CMD_SEND
#define     repeat_package_times_bit  24
#define     use_int_window_bit        21
#define     use_int_window_bit        21
#define     cmd_send_data_bit         20
#define     check_busy_on_dat0_bit    19
#define     response_crc7_from_8_bit  18
#define     response_have_data_bit 17
#define     response_do_not_have_crc7_bit 16
#define     cmd_response_bits_bit 8
#define     cmd_command_bit       0
// for SDIO_CONFIG
#define     sdio_write_CRC_ok_status_bit 29
#define     sdio_write_Nwr_bit        23
#define     m_endian_bit              21
#define     bus_width_bit             20   // 0 1-bit, 1-4bits
#define     data_latch_at_negedge_bit 19
#define     response_latch_at_negedge_bit 18
#define     cmd_argument_bits_bit 12
#define     cmd_out_at_posedge_bit 11
#define     cmd_disable_CRC_bit   10
#define     cmd_clk_divide_bit    0

// SDIO_STATUS_IRQ
#define     sdio_timing_out_count_bit   19
#define     arc_timing_out_int_en_bit   18
#define     amrisc_timing_out_int_en_bit 17
#define     sdio_timing_out_int_bit      16
#define     sdio_status_info_bit        12
#define     sdio_set_soft_int_bit       11
#define     sdio_soft_int_bit           10
#define     sdio_cmd_int_bit             9
#define     sdio_if_int_bit              8
#define     sdio_data_write_crc16_ok_bit 7
#define     sdio_data_read_crc16_ok_bit  6
#define     sdio_cmd_crc7_ok_bit  5
#define     sdio_cmd_busy_bit     4
#define     sdio_status_bit       0

// SDIO_IRQ_CONFIG
#define     force_halt_bit           30
#define     sdio_force_read_back_bit 24
#define     disable_mem_halt_bit     22
#define     sdio_force_output_en_bit 16
#define     soft_reset_bit           15
#define     sdio_force_enable_bit    14
#define     sdio_force_read_data_bit  8
#define     sdio_if_int_config_bit 6
#define     arc_soft_int_en_bit    5
#define     arc_cmd_int_en_bit     4
#define     arc_if_int_en_bit      3
#define     amrisc_soft_int_en_bit 2
#define     amrisc_cmd_int_en_bit  1
#define     amrisc_if_int_en_bit   0


// for SDIO_MULT_CONFIG
#define     data_catch_finish_point_bit 16
#define     response_read_index_bit     12
#define     data_catch_readout_en_bit    9
#define     write_read_out_index_bit     8
#define     data_catch_level_bit   6
#define     stream_8_bits_mode_bit 5
#define     stream_enable_bit      4
#define     ms_sclk_always_bit     3
#define     ms_enable_bit_bit      2
#define     SDIO_port_sel_bit      0

#define ERROR_NONE                  0
#define ERROR_GO_IDLE1              1
#define ERROR_GO_IDLE2              2
#define ERROR_APP55_1               3
#define ERROR_ACMD41                4
#define ERROR_APP55_2               5
#define ERROR_VOLTAGE_VALIDATION    6
#define ERROR_SEND_CID1             7
#define ERROR_SEND_RELATIVE_ADDR    8
#define ERROR_SEND_CID2             9
#define ERROR_SELECT_CARD           10
#define ERROR_APP55_RETRY3          11
#define ERROR_SEND_SCR              12
#define ERROR_READ_BLOCK            13
#define ERROR_STOP_TRANSMISSION     14
#define ERROR_MAGIC_WORDS           15
#define ERROR_CMD1                  16
#define ERROR_MMC_SWITCH_BUS        17

//#define VOLTAGE_VALIDATION_RETRY    0x8000
//#define APP55_RETRY                 3


/* 250ms of timeout */
#define CLK_DIV                     250  /* (200/((31+1)*2)=0.390625MHz),this define is for SD_CLK in Card Identification Stage */

#define VOLTAGE_VALIDATION_RETRY    0x8000
#define APP55_RETRY                 3

#define TIMEOUT_SHORT       (300*1000)  /* 250ms */
#define TIMEOUT_DATA        (400*1000)  /* 300ms (TIMEOUT_SHORT+ (READ_SIZE*8)/10000000) */
#ifndef CONFIG_SDIO_BUFFER_SIZE
#define CONFIG_SDIO_BUFFER_SIZE 64*1024
#endif
#define NO_DELAY_DATA 0

// CPU relative configuration
#define CLK_CLK81                          (7)
#define SDIO_CLKSRC CLK_CLK81
#define SDIO_PORT_MAX   (4+2)
#define SDIO_PORT_A    0
#define SDIO_PORT_B    1
#define SDIO_PORT_C    2
#define SDIO_PORT_XC_A   3
#define SDIO_PORT_XC_B   4
#define SDIO_PORT_XC_C   5

#define CARD_SD_SDIO_INIT          (1<<0)
#define CARD_SD_SDIO_DETECT        (1<<1)
#define CARD_SD_SDIO_PWR_PREPARE   (1<<2)
#define CARD_SD_SDIO_PWR_ON        (1<<3)
#define CARD_SD_SDIO_PWR_OFF       (1<<4)

enum dma_data_direction {
	DMA_BIDIRECTIONAL	= 0,
	DMA_TO_DEVICE		= 1,
	DMA_FROM_DEVICE		= 2,
};

struct mmc_cmd {
	UINT16 cmdidx;
	UINT32 resp_type;
	UINT32 cmdarg;
	char response[18];
	UINT32 flags;
};

struct mmc_data {
	union {
		char *dest;
		const char *src; /* src buffers don't get written to */
	};
	UINT32 flags;
	UINT32 blocks;
	UINT32 blocksize;
};

struct mmc_storage_info_t{
	struct storage_node_t * valid_node;
	struct storage_node_t * free_node;
	UINT64 start_addr;
	UINT64 end_addr;
	unsigned char secure_valid;
	unsigned char secure_init;
};

struct emmckey_valid_node_t {
	UINT64 phy_addr;
	UINT64 phy_size;
	struct emmckey_valid_node_t *next;
};

struct aml_card_sd_info
{
	unsigned sdio_port;				 //0: sdioa, 1:sdiob, 2:sdioc
	char * name;
	int inited_flag;
	int removed_flag;
	int init_retry;
	int single_blk_failed;
	unsigned int sdio_pwr_flag;
};

struct aml_emmckey_info_t {
	//struct memory_card *card;
	struct emmckey_valid_node_t *key_valid_node;
	UINT64    keyarea_phy_addr;
	UINT64    keyarea_phy_size;
	UINT64    lba_start;
	UINT64    lba_end;
	UINT32    blk_size;
	UINT32    blk_shift;
	UINT8     key_init;
	UINT8     key_valid;
	UINT8    key_part_count;
};

#define EMMC_KEY_AREA_SIGNAL		"emmckeys"
#define EMMC_KEY_AREA_SIGNAL_LEN	16

#define EMMC_KEYAREA_SIZE		(128*1024)
#define EMMC_KEYAREA_COUNT		2

#if defined(EMMC_KEY_KERNEL)
//#define KEY_PREVIOUS_PARTITION 	"boot_env"
//#define KEY_LATER_PARTITION		"logo"
#endif
#ifdef EMMC_KEY_UBOOT
//#define EMMCKEY_AREA_PHY_START_ADDR    (0x60000+0x2000)
//#define EMMCKEY_AREA_PHY_SIZE          (128*1024)
//#define EMMCKEY_AREA_PHY_START_ADDR    (CONFIG_ENV_OFFSET+CONFIG_ENV_SIZE)
#endif

#define EMMCKEY_RESERVE_OFFSET          0x4000  // we store partition table in the previous 16KB space
#define EMMCKEY_AREA_PHY_SIZE           (EMMC_KEYAREA_COUNT * EMMC_KEYAREA_SIZE)


#define EMMCKEY_DATA_VALID_LEN		(EMMC_KEYAREA_SIZE - EMMC_KEY_AREA_SIGNAL_LEN - 4 - 4 -4)
struct emmckey_data_t {
	UINT8     keyarea_mark[EMMC_KEY_AREA_SIGNAL_LEN];
	UINT32	   keyarea_mark_checksum;
	UINT32    checksum;
	UINT32    reserve;
	UINT8     data[EMMCKEY_DATA_VALID_LEN];
};

typedef struct block_dev_desc {
	int		if_type;	/* type of the interface */
	int		dev;		/* device number */
	unsigned char	part_type;	/* partition type */
	unsigned char	target;		/* target SCSI ID */
	unsigned char	lun;		/* target LUN */
	unsigned char	type;		/* device type */
	unsigned char	removable;	/* removable device */

	UINT64		lba;		/* number of blocks */
	unsigned long	blksz;		/* block size */
	char		vendor [40+1];	/* IDE model, SCSI Vendor */
	char		product[20+1];	/* IDE Serial no, SCSI product */
	char		revision[8+1];	/* firmware revision */

	void		*priv;		/* driver private struct pointer */
}block_dev_desc_t;

struct mmc {
	char name[32];
	void *priv;
	UINT32 voltages;
	UINT32 version;
	//UINT32 has_init;
	UINT32 f_min;
	UINT32 f_max;
	INT32 high_capacity;
	UINT32 bus_width;
	UINT32 clock;
	UINT32 card_caps;
	UINT32 host_caps;
	UINT32 ocr;
	UINT32 scr[2];
	UINT32 csd[4];
	char cid[16];
	UINT16 rca;
	UINT32 tran_speed;
	UINT32 read_bl_len;
	UINT32 write_bl_len;
	UINT64 capacity;
	UINT64 boot_size;
	block_dev_desc_t block_dev;

	UINT32 storage_protect;
	struct mmc_storage_info_t *mmc_storage_info;


	UINT32 key_protect;
	struct aml_emmckey_info_t *aml_emmckey_info;

    UINT32 is_inited; // is initialize?
};

#define CARD_TYPE_SHIFT             4
#define CARD_TYPE_MASK              0xf
#define CARD_TYPE_UNKNOWN           0        /* unknown */
#define CARD_TYPE_MMC               1        /* MMC card */
#define CARD_TYPE_SD                2        /* SD card */
#define CARD_TYPE_SDIO              3        /* SDIO card */
#define CARD_TYPE_SD_COMBO          4        /* SD combo (IO+mem) card */
#define CARD_TYPE_NON_SDIO          5        /* NON sdio device (means SD/MMC card) */

#define CARD_TYPE_SDHC		1
#define CARD_TYPE_EMMC		3

#define AML_GET_CARD_TYPE(val, port)    ((val >> (port * CARD_TYPE_SHIFT)) & CARD_TYPE_MASK)
#define AML_SET_CARD_TYPE(val, port, type)   \
    (val |= ((type & CARD_TYPE_MASK) << (port * CARD_TYPE_SHIFT)))

/* Interface types: */
#define IF_TYPE_UNKNOWN		0
#define IF_TYPE_IDE		1
#define IF_TYPE_SCSI		2
#define IF_TYPE_ATAPI		3
#define IF_TYPE_USB		4
#define IF_TYPE_DOC		5
#define IF_TYPE_MMC		6
#define IF_TYPE_SD		7
#define IF_TYPE_SATA		8

/* Part types */
#define PART_TYPE_UNKNOWN	0x00
#define PART_TYPE_MAC		0x01
#define PART_TYPE_DOS		0x02
#define PART_TYPE_ISO		0x03
#define PART_TYPE_AMIGA		0x04
#define PART_TYPE_EFI		0x05

#endif //FLASH_H
