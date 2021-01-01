/*
 * SDHC definitions
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __AML_SDHC_H__
#define __AML_SDHC_H__

#include <Library/BaseMemoryLib.h>
#include <Library/CacheMaintenanceLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/TimerLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/DmaLib.h>

#include <Protocol/MmcHost.h>

#define AML_ERROR_RETRY_COUNTER 10
#define AML_TIMEOUT_RETRY_COUNTER 2

#define AML_SDHC_MAGIC "amlsdhc"
#define AML_SDIO_MAGIC "amlsdio"

#define MMC_IOBLOCKS_READ       0
#define MMC_IOBLOCKS_WRITE      1

#define IO_CBUS_BASE 0xc1100000
#define CBUS_REG_OFFSET(reg) ((reg) << 2)
#define CBUS_REG_ADDR(reg) (IO_CBUS_BASE + CBUS_REG_OFFSET(reg))

#define SDIO_ARGU       (0xc1108c20)
#define SDIO_SEND       (0xc1108c24)
#define SDIO_CONF       (0xc1108c28)
#define SDIO_IRQS       (0xc1108c2c)
#define SDIO_IRQC       (0xc1108c30)
#define SDIO_MULT       (0xc1108c34)
#define SDIO_ADDR       (0xc1108c38)
#define SDIO_EXT        (0xc1108c3c)
#define SDIO_CCTL       (0xc1108c40)
#define SDIO_CDAT       (0xc1108c44)

#define BIT_CMD_RESPONSE_EXPECT (1 << 6)
#define BIT_CMD_LONG_RESPONSE (1 << 7)
#define BIT_CMD_CHECK_RESPONSE_CRC (1 << 8)
#define BIT_CMD_DATA_EXPECTED (1 << 9)
#define BIT_CMD_READ (0 << 10)
#define BIT_CMD_WRITE (1 << 10)
#define BIT_CMD_BLOCK_TRANSFER (0 << 11)
#define BIT_CMD_STREAM_TRANSFER (1 << 11)
#define BIT_CMD_SEND_AUTO_STOP (1 << 12)
#define BIT_CMD_WAIT_PRVDATA_COMPLETE (1 << 13)
#define BIT_CMD_STOP_ABORT_CMD (1 << 14)
#define BIT_CMD_SEND_INIT (1 << 15)
#define BIT_CMD_UPDATE_CLOCK_ONLY (1 << 21)
#define BIT_CMD_READ_CEATA_DEVICE (1 << 22)
#define BIT_CMD_CCS_EXPECTED (1 << 23)
#define BIT_CMD_ENABLE_BOOT (1 << 24)
#define BIT_CMD_EXPECT_BOOT_ACK (1 << 25)
#define BIT_CMD_DISABLE_BOOT (1 << 26)
#define BIT_CMD_MANDATORY_BOOT (0 << 27)
#define BIT_CMD_ALTERNATE_BOOT (1 << 27)
#define BIT_CMD_VOLT_SWITCH (1 << 28)
#define BIT_CMD_USE_HOLD_REG (1 << 29)
#define BIT_CMD_START (1 << 31)

enum aml_mmc_waitfor
{
	XFER_INIT,			   /* 0 */
	XFER_START,			   /* 1 */
	XFER_AFTER_START,	  /* 2 */
	XFER_IRQ_OCCUR,		   /* 3 */
	XFER_IRQ_TASKLET_CMD,  /* 4 */
	XFER_IRQ_TASKLET_DATA, /* 5 */
	XFER_IRQ_TASKLET_BUSY, /* 6 */
	XFER_IRQ_UNKNOWN_IRQ,  /* 7 */
	XFER_TIMER_TIMEOUT,	/* 8 */
	XFER_TASKLET_CMD,	  /* 9 */
	XFER_TASKLET_DATA,	 /* 10 */
	XFER_TASKLET_BUSY,	 /* 11 */
	XFER_TIMEDOUT,		   /* 12 */
	XFER_FINISHED,		   /* 13 */
};

enum aml_host_status
{						 /* Host controller status */
  HOST_INVALID = 0,		 /* 0, invalid value used for initialization */
  HOST_RX_FIFO_FULL = 1, /* 1, start with 1 */
  HOST_TX_FIFO_EMPTY,	/* 2 */
  HOST_RSP_CRC_ERR,		 /* 3 */
  HOST_DAT_CRC_ERR,		 /* 4 */
  HOST_RSP_TIMEOUT_ERR,  /* 5 */
  HOST_DAT_TIMEOUT_ERR,  /* 6 */
  HOST_ERR_END,			 /* 7, end of errors */
  HOST_TASKLET_CMD,		 /* 8 */
  HOST_TASKLET_DATA,	 /* 9 */
};

#define PORT_SDIO_A 0
#define PORT_SDIO_B 1
#define PORT_SDIO_C 2
#define PORT_SDHC_A 3
#define PORT_SDHC_B 4
#define PORT_SDHC_C 5

/* 250ms of timeout */
#define CLK_DIV 250 /* (200/((31+1)*2)=0.390625MHz),this define is for SD_CLK in Card Identification Stage */

#define VOLTAGE_VALIDATION_RETRY 0x8000
#define APP55_RETRY 3

#define TIMEOUT_SHORT (250 * 1000) /* 250ms */
#define TIMEOUT_DATA (300 * 1000)  /* 300ms (TIMEOUT_SHORT+ (READ_SIZE*8)/10000000) */
#ifndef CONFIG_SDIO_BUFFER_SIZE
#define CONFIG_SDIO_BUFFER_SIZE 64 * 1024
#endif
#define NO_DELAY_DATA 0

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

#define MMC_CAP_4_BIT_DATA	(1 << 0)	/* Can the host do 4 bit transfers */
#define MMC_CAP_MMC_HIGHSPEED	(1 << 1)	/* Can do MMC high-speed timing */
#define MMC_CAP_SD_HIGHSPEED	(1 << 2)	/* Can do SD high-speed timing */
#define MMC_CAP_SDIO_IRQ	(1 << 3)	/* Can signal pending SDIO IRQs */
#define MMC_CAP_SPI		(1 << 4)	/* Talks only SPI protocols */
#define MMC_CAP_NEEDS_POLL	(1 << 5)	/* Needs polling for card-detection */
#define MMC_CAP_8_BIT_DATA	(1 << 6)	/* Can the host do 8 bit transfers */

#define MMC_CAP_NONREMOVABLE	(1 << 8)	/* Nonremovable e.g. eMMC */
#define MMC_CAP_WAIT_WHILE_BUSY	(1 << 9)	/* Waits while card is busy */
#define MMC_CAP_ERASE		(1 << 10)	/* Allow erase/trim commands */
#define MMC_CAP_1_8V_DDR	(1 << 11)	/* can support */
						/* DDR mode at 1.8V */
#define MMC_CAP_1_2V_DDR	(1 << 12)	/* can support */
						/* DDR mode at 1.2V */
#define MMC_CAP_POWER_OFF_CARD	(1 << 13)	/* Can power off after boot */
#define MMC_CAP_BUS_WIDTH_TEST	(1 << 14)	/* CMD14/CMD19 bus width ok */
#define MMC_CAP_UHS_SDR12	(1 << 15)	/* Host supports UHS SDR12 mode */
#define MMC_CAP_UHS_SDR25	(1 << 16)	/* Host supports UHS SDR25 mode */
#define MMC_CAP_UHS_SDR50	(1 << 17)	/* Host supports UHS SDR50 mode */
#define MMC_CAP_UHS_SDR104	(1 << 18)	/* Host supports UHS SDR104 mode */
#define MMC_CAP_UHS_DDR50	(1 << 19)	/* Host supports UHS DDR50 mode */
#define MMC_CAP_DRIVER_TYPE_A	(1 << 23)	/* Host supports Driver Type A */
#define MMC_CAP_DRIVER_TYPE_C	(1 << 24)	/* Host supports Driver Type C */
#define MMC_CAP_DRIVER_TYPE_D	(1 << 25)	/* Host supports Driver Type D */
#define MMC_CAP_CMD23		(1 << 30)	/* CMD23 supported. */
#define MMC_CAP_HW_RESET	(1 << 31)	/* Hardware reset */

#define MMC_CAP2_BOOTPART_NOACC	(1 << 0)	/* Boot partition no access */
#define MMC_CAP2_CACHE_CTRL	(1 << 1)	/* Allow cache control */
#define MMC_CAP2_POWEROFF_NOTIFY (1 << 2)	/* Notify poweroff supported */
#define MMC_CAP2_NO_MULTI_READ	(1 << 3)	/* Multiblock reads don't work */
#define MMC_CAP2_NO_SLEEP_CMD	(1 << 4)	/* Don't allow sleep command */
#define MMC_CAP2_HS200_1_8V_SDR	(1 << 5)        /* can support */
#define MMC_CAP2_HS200_1_2V_SDR	(1 << 6)        /* can support */
#define MMC_CAP2_HS200		(MMC_CAP2_HS200_1_8V_SDR | \
				 MMC_CAP2_HS200_1_2V_SDR)
#define MMC_CAP2_BROKEN_VOLTAGE	(1 << 7)	/* Use the broken voltage */
#define MMC_CAP2_DETECT_ON_ERR	(1 << 8)	/* On I/O err check card removal */
#define MMC_CAP2_HC_ERASE_SZ	(1 << 9)	/* High-capacity erase size */
#define MMC_CAP2_CD_ACTIVE_HIGH	(1 << 10)	/* Card-detect signal active high */
#define MMC_CAP2_RO_ACTIVE_HIGH	(1 << 11)	/* Write-protect signal active high */
#define MMC_CAP2_PACKED_RD	(1 << 12)	/* Allow packed read */
#define MMC_CAP2_PACKED_WR	(1 << 13)	/* Allow packed write */
#define MMC_CAP2_PACKED_CMD	(MMC_CAP2_PACKED_RD | \
				 MMC_CAP2_PACKED_WR)
#define MMC_CAP2_NO_PRESCAN_POWERUP (1 << 14)	/* Don't power up before scan */

/* 0:unknown, 1:mmc card(include eMMC), 2:sd card(include tSD), 3:sdio device(ie:sdio-wifi), 4:SD combo (IO+mem) card, 5:NON sdio device(means sd/mmc card), other:reserved */
#define CARD_TYPE_UNKNOWN 0  /* unknown */
#define CARD_TYPE_MMC 1		 /* MMC card */
#define CARD_TYPE_SD 2		 /* SD card */
#define CARD_TYPE_SDIO 3	 /* SDIO card */
#define CARD_TYPE_SD_COMBO 4 /* SD combo (IO+mem) card */
#define CARD_TYPE_NON_SDIO 5 /* NON sdio device (means SD/MMC card) */
#define aml_card_type_unknown(c) ((c) == CARD_TYPE_UNKNOWN)
#define aml_card_type_mmc(c) ((c) == CARD_TYPE_MMC)
#define aml_card_type_sd(c) ((c) == CARD_TYPE_SD)
#define aml_card_type_sdio(c) ((c) == CARD_TYPE_SDIO)
#define aml_card_type_non_sdio(c) ((c) == CARD_TYPE_NON_SDIO)

typedef union cmd_send {
	UINT32 Raw;
	struct
	{
		UINT32 cmd_command : 8;				  /*[7:0] Command Index*/
		UINT32 cmd_response_bits : 8;		  /*[15:8]
        * 00 means no response
        * others: Response bit number(cmd bits+response bits+crc bits-1)*/
		UINT32 response_do_not_have_crc7 : 1; /*[16]
        * 0:Response need check CRC7, 1: dont need check*/
		UINT32 response_have_data : 1;		  /*[17]
        * 0:Receiving Response without data, 1:Receiving response with data*/
		UINT32 response_crc7_from_8 : 1;	  /*[18]
        * 0:Normal CRC7, Calculating CRC7 will be from bit0 of all response bits,
        * 1:Calculating CRC7 will be from bit8 of all response bits*/
		UINT32 check_busy_on_dat0 : 1;		  /*[19]
        * used for R1b response 0: dont check busy on dat0, 1:need check*/
		UINT32 cmd_send_data : 1;			  /*[20]
        * 0:This command is not for transmitting data,
        * 1:This command is for transmitting data*/
		UINT32 use_int_window : 1;			  /*[21]
        * 0:SDIO DAT1 interrupt window disabled, 1:Enabled*/
		UINT32 reserved : 2;				  /*[23:22]*/
		UINT32 repeat_package_times : 8;	  /*[31:24] Total packages to be sent*/
	} Bit;
} CMD_SEND;

typedef union sdio_config {
	UINT32 Raw;
	struct
	{
		UINT32 cmd_clk_divide : 10;			 /*[9:0] Clock rate setting,
        * Frequency of SD equals to Fsystem/((cmd_clk_divide+1)*2)*/
		UINT32 cmd_disable_crc : 1;			 /*[10]
        * 0:CRC employed, 1:dont send CRC during command being sent*/
		UINT32 cmd_out_at_posedge : 1;		 /*[11]
        * Command out at negedge normally, 1:at posedge*/
		UINT32 cmd_argument_bits : 6;		 /*[17:12] before CRC added, normally 39*/
		UINT32 do_not_delay_data : 1;		 /*[18]
        *0:Delay one clock normally, 1:dont delay*/
		UINT32 data_latch_at_negedge : 1;	/*[19]
        * 0:Data caught at posedge normally, 1:negedge*/
		UINT32 bus_width : 1;				 /*[20] 0:1bit, 1:4bit*/
		UINT32 m_endian : 2;				 /*[22:21]
        * Change ENDIAN(bytes order) from DMA data (e.g. dma_din[31:0]).
        * (00: ENDIAN no change, data output equals to original dma_din[31:0];
        * 01: data output equals to {dma_din[23:16],dma_din[31:24],
        * dma_din[7:0],dma_din[15:8]};10: data output equals to
        * {dma_din[15:0],dma_din[31:16]};11: data output equals to
        * {dma_din[7:0],dma_din[15:8],dma_din[23:16],dma_din[31:24]})*/
		UINT32 sdio_write_nwr : 6;			 /*[28:23]
        * Number of clock cycles waiting before writing data*/
		UINT32 sdio_write_crc_ok_status : 3; /*[31:29] if CRC status
        * equals this register, sdio write can be consider as correct*/
	} Bit;
} SDIO_CONFIG;

typedef union sdio_status_irq {
	UINT32 Raw;
	struct
	{
		UINT32 sdio_status : 4;				 /*[3:0] Read Only
        * SDIO State Machine Current State, just for debug*/
		UINT32 sdio_cmd_busy : 1;			 /*[4] Read Only
        * SDIO Command Busy, 1:Busy State*/
		UINT32 sdio_response_crc7_ok : 1;	/*[5] Read Only
        * SDIO Response CRC7 status, 1:OK*/
		UINT32 sdio_data_read_crc16_ok : 1;  /*[6] Read Only
        * SDIO Data Read CRC16 status, 1:OK*/
		UINT32 sdio_data_write_crc16_ok : 1; /*[7] Read Only
        * SDIO Data Write CRC16 status, 1:OK*/
		UINT32 sdio_if_int : 1;				 /*[8] write 1 clear this int bit
        * SDIO DAT1 Interrupt Status*/
		UINT32 sdio_cmd_int : 1;			 /*[9] write 1 clear this int bit
        * Command Done Interrupt Status*/
		UINT32 sdio_soft_int : 1;			 /*[10] write 1 clear this int bit
        * Soft Interrupt Status*/
		UINT32 sdio_set_soft_int : 1;		 /*[11] write 1 to this bit
        * will set Soft Interrupt, read out is m_req_sdio, just for debug*/
		UINT32 sdio_status_info : 4;		 /*[15:12]
        * used for change information between ARC and Amrisc */
		UINT32 sdio_timing_out_int : 1;		 /*[16] write 1 clear this int bit
        * Timeout Counter Interrupt Status*/
		UINT32 amrisc_timing_out_int_en : 1; /*[17]
        * Timeout Counter Interrupt Enable for AMRISC*/
		UINT32 arc_timing_out_int_en : 1;	/*[18]
        * Timeout Counter Interrupt Enable for ARC/ARM*/
		UINT32 sdio_timing_out_count : 13;   /*[31:19]
        * Timeout Counter Preload Setting and Present Status*/
	} Bit;
} SDIO_STATUS_IRQ;

typedef union sdio_irq_config {
	UINT32 Raw;
	struct
	{
		UINT32 amrisc_if_int_en : 1;	 /*[0]
        * 1:SDIO DAT1 Interrupt Enable for AMRISC*/
		UINT32 amrisc_cmd_int_en : 1;	/*[1]
        * 1:Command Done Interrupt Enable for AMRISC*/
		UINT32 amrisc_soft_int_en : 1;   /*[2]
        * 1:Soft Interrupt Enable for AMRISC*/
		UINT32 arc_if_int_en : 1;		 /*[3]
        * 1:SDIO DAT1 Interrupt Enable for ARM/ARC*/
		UINT32 arc_cmd_int_en : 1;		 /*[4]
        * 1:Command Done Interrupt Enable for ARM/ARC*/
		UINT32 arc_soft_int_en : 1;		 /*[5]
        * 1:Soft Interrupt Enable for ARM/ARC*/
		UINT32 sdio_if_int_config : 2;   /*[7:6]
        * 00:sdio_if_interrupt window will reset after data Tx/Rx or command
        * done, others: only after command done*/
		UINT32 sdio_force_data : 6;		 /*[13:8]
        * Write operation: Data forced by software
        * Read operation: {CLK,CMD,DAT[3:0]}*/
		UINT32 sdio_force_enable : 1;	/*[14] Software Force Enable
        * This is the software force mode, Software can directly
        * write to sdio 6 ports (cmd, clk, dat0..3) if force_output_en
        * is enabled. and hardware outputs will be bypassed.*/
		UINT32 soft_reset : 1;			 /*[15]
        * Write 1 Soft Reset, Don't need to clear it*/
		UINT32 sdio_force_output_en : 6; /*[21:16]
        * Force Data Output Enable,{CLK,CMD,DAT[3:0]}*/
		UINT32 disable_mem_halt : 2;	 /*[23:22] write and read
        * 23:Disable write memory halt, 22:Disable read memory halt*/
		UINT32 sdio_force_data_read : 6; /*[29:24] Read Only
        * Data read out which have been forced by software*/
		UINT32 force_halt : 1;			 /*[30] 1:Force halt SDIO by software
        * Halt in this sdio host controller means stop to transmit or
        * receive data from sd card. and then sd card clock will be shutdown.
        * Software can force to halt anytime, and hardware will automatically
        * halt the sdio when reading fifo is full or writing fifo is empty*/
		UINT32 halt_hole : 1;			 /*[31]
        * 0: SDIO halt for 8bit mode, 1:SDIO halt for 16bit mode*/
	} Bit;
} SDIO_IRQ_CONFIG;

typedef union sdio_mult_config {
	UINT32 Raw;
	struct
	{
		UINT32 sdio_port_sel : 2;			 /*[1:0] 0:sdio_a, 1:sdio_b, 2:sdio_c*/
		UINT32 ms_enable : 1;				 /*[2] 1:Memory Stick Enable*/
		UINT32 ms_sclk_always : 1;			 /*[3] 1: Always send ms_sclk*/
		UINT32 stream_enable : 1;			 /*[4] 1:Stream Enable*/
		UINT32 stream_8_bits_mode : 1;		 /*[5] Stream 8bits mode*/
		UINT32 data_catch_level : 2;		 /*[7:6] Level of data catch*/
		UINT32 write_read_out_index : 1;	 /*[8] Write response index Enable
        * [31:16], [11:10], [7:0] is set only when  bit8 of this register is not set.
        * And other bits are set only when bit8 of this register is also set.*/
		UINT32 data_catch_readout_en : 1;	/*[9] Data catch readout Enable*/
		UINT32 sdio_0_data_on_1 : 1;		 /*[10] 1:dat0 is on dat1*/
		UINT32 sdio_1_data_swap01 : 1;		 /*[11] 1:dat1 and dat0 swapped*/
		UINT32 response_read_index : 4;		 /*[15:12] Index of internal read response*/
		UINT32 data_catch_finish_point : 12; /*[27:16] If internal data
        * catch counter equals this register, it indicates data catching is finished*/
		UINT32 reserved : 4;				 /*[31:28]*/
	} Bit;
} SDIO_MULT_CONFIG;

typedef union sdio_extension {
	UINT32 Raw;
	struct
	{
		UINT32 cmd_argument_ext : 16;		  /*[15:0] for future use*/
		UINT32 data_rw_number : 14;			  /*[29:16]
        * Data Read/Write Number in one packet, include CRC16 if has CRC16*/
		UINT32 data_rw_do_not_have_crc16 : 1; /*[30]
        * 0:data Read/Write has crc16, 1:without crc16*/
		UINT32 crc_status_4line : 1;		  /*[31] 1:4Lines check CRC Status*/
	} Bit;
} SDIO_EXTENSION;

typedef union sdhc_send {
	UINT32 Raw;
	struct
	{
		UINT32 cmd_index : 6;	/*[5:0] command index*/
		UINT32 cmd_has_resp : 1; /*[6] 0:no resp 1:has resp*/
		UINT32 cmd_has_data : 1; /*[7] 0:no data 1:has data*/
		UINT32 resp_len : 1;	 /*[8] 0:48bit 1:136bit*/
		UINT32 resp_no_crc : 1;  /*[9] 0:check crc7 1:don't check crc7*/
		UINT32 data_dir : 1;	 /*[10] 0:data rx, 1:data tx*/
		UINT32 data_stop : 1;	/*[11] 0:rx or tx, 1:data stop,ATTN:will give rx a softreset*/
		UINT32 r1b : 1;			 /*[12] 0: resp with no busy, 1:R1B*/
		UINT32 reserved : 3;	 /*[15:13] reserved*/
		UINT32 total_pack : 16;  /*[31:16] total package number for writing or reading*/
	} Bit;
} SDHC_SEND;

typedef union sdhc_ctrl {
	UINT32 Raw;
	struct
	{
		UINT32 dat_type : 2;	   /*[1:0] 0:1bit, 1:4bits, 2:8bits, 3:reserved*/
		UINT32 ddr_mode : 1;	   /*[2] 0:SDR mode, 1:Don't set it*/
		UINT32 tx_crc_nocheck : 1; /*[3] 0:check sd write crc result, 1:disable tx crc check*/
		UINT32 pack_len : 9;	   /*[12:4] 0:512Bytes, 1:1, 2:2, ..., 511:511Bytes*/
		UINT32 rx_timeout : 7;	 /*[19:13] cmd or wcrc Receiving Timeout, default 64*/
		UINT32 rx_period : 4;	  /*[23:20]Period between response/cmd and next cmd, default 8*/
		UINT32 rx_endian : 3;	  /*[26:24] Rx Endian Control*/
		UINT32 sdio_irq_mode : 1;  /*[27]0:Normal mode, 1: support data block gap
			(need turn off clock gating)*/
		UINT32 dat0_irq_sel : 1;   /*[28] Dat0 Interrupt selection,
			0:busy check after response, 1:any rising edge of dat0*/
		UINT32 tx_endian : 3;	  /*[31:29] Tx Endian Control*/
	} Bit;
} SDHC_CTRL;

typedef union sdhc_stat {
	UINT32 Raw;
	struct
	{
		UINT32 cmd_busy : 1;   /*[0] 0:Ready for command, 1:busy*/
		UINT32 dat3_0 : 4;	 /*[4:1] DAT[3:0]*/
		UINT32 cmd : 1;		   /*[5] CMD*/
		UINT32 rxfifo_cnt : 7; /*[12:6] RxFIFO count*/
		UINT32 txfifo_cnt : 7; /*[19:13] TxFIFO count*/
		UINT32 dat7_4 : 4;	 /*[23:20] DAT[7:4]*/
		UINT32 reserved : 8;   /*[31:24] Reserved*/
	} Bit;
} SDHC_STAT;

/*
* to avoid glitch issue,
* 1. clk_switch_on better be set after cfg_en be set to 1'b1
* 2. clk_switch_off shall be set before cfg_en be set to 1'b0
* 3. rx_clk/sd_clk phase diff please see SD_REGE_CLK2.
*/
typedef union sdhc_clkc {
	UINT32 Raw;
	struct
	{
		UINT32 clk_div : 12;	/*[11:0] clk_div for TX_CLK 0: don't set it,
			1:div2, 2:div3, 3:div4 ...*/
		UINT32 tx_clk_on : 1;   /*[12] TX_CLK 0:switch off, 1:switch on*/
		UINT32 rx_clk_on : 1;   /*[13] RX_CLK 0:switch off, 1:switch on*/
		UINT32 sd_clk_on : 1;   /*[14] SD_CLK 0:switch off, 1:switch on*/
		UINT32 mod_clk_on : 1;  /*[15] Clock Module Enable, Should
			set before bit[14:12] switch on, and after bit[14:12] switch off*/
		UINT32 clk_src_sel : 2; /*[17:16] 0:osc, 1:fclk_div4, 2:fclk_div3, 3:fclk_div5*/
		UINT32 reserved : 6;	/*[23:18] Reserved*/
		UINT32 clk_jic : 1;		/*[24] Clock JIC for clock gating control
			1: will turn off clock gating*/
		UINT32 mem_pwr_off : 2; /*[26:25] 00:Memory Power Up, 11:Memory Power Off*/
		UINT32 reserved2 : 5;   /*[31:27] Reserved*/
	} Bit;
} SDHC_CLKC;

/*
* Note1: dma_urgent is just set when bandwidth is very tight
* Note2: pio_rdresp need to be combined with REG0_ARGU;
* For R0, when 0, reading REG0 will get the normal 32bit response;
* For R2, when 1, reading REG0 will get CID[31:0], when 2, get CID[63:32],
* and so on; 6 or 7, will get original command argument.
*/
typedef union sdhc_pdma {
	UINT32 Raw;
	struct
	{
		UINT32 dma_mode : 1;			/*[0] 0:PIO mode, 1:DMA mode*/
		UINT32 pio_rdresp : 3;			/*[3:1] 0:[39:8] 1:1st 32bits, 2:2nd ...,
			6 or 7:command argument*/
		UINT32 dma_urgent : 1;			/*[4] 0:not urgent, 1:urgent*/
		UINT32 wr_burst : 5;			/*[9:5] Number in one Write request burst(0:1,1:2...)*/
		UINT32 rd_burst : 5;			/*[14:10] Number in one Read request burst(0:1, 1:2...)*/
		UINT32 rxfifo_th : 7;			/*[21:15] RxFIFO threshold, >=rxth, will request write*/
		UINT32 txfifo_th : 7;			/*[28:22] TxFIFO threshold, <=txth, will request read*/
		UINT32 rxfifo_manual_flush : 2; /*[30:29] [30]self-clear-flush,
			[29] mode: 0:hw, 1:sw*/
		UINT32 txfifo_fill : 1;			/*[31] self-clear-fill, recommand to write before sd send*/
	} Bit;
} SDHC_PDMA;

typedef union sdhc_misc {
	UINT32 Raw;
	struct
	{
		UINT32 reserved : 4;	  /*[3:0] reserved*/
		UINT32 wcrc_err_patt : 3; /*[6:4] WCRC Error Pattern*/
		UINT32 wcrc_ok_patt : 3;  /*[9:7] WCRC OK Pattern*/
		UINT32 reserved1 : 6;	 /*[15:10] reserved*/
		UINT32 burst_num : 6;	 /*[21:16] Burst Number*/
		UINT32 thread_id : 6;	 /*[27:22] Thread ID*/
		UINT32 manual_stop : 1;   /*[28] 0:auto stop mode, 1:manual stop mode*/
		UINT32 txstart_thres : 3; /*[31:29] txstart_thres(if (txfifo_cnt/4)>(threshold*2), Tx will start)*/
	} Bit;
} SDHC_MISC;

typedef union sdhc_ictl {
	UINT32 Raw;
	struct
	{
		UINT32 resp_ok : 1;			/*[0] Response is received OK*/
		UINT32 resp_timeout : 1;	/*[1] Response Timeout Error*/
		UINT32 resp_err_crc : 1;	/*[2] Response CRC Error*/
		UINT32 resp_ok_noclear : 1; /*[3] Response is received OK(always no self reset)*/
		UINT32 data_1pack_ok : 1;   /*[4] One Package Data Completed ok*/
		UINT32 data_timeout : 1;	/*[5] One Package Data Failed (Timeout Error)*/
		UINT32 data_err_crc : 1;	/*[6] One Package Data Failed (CRC Error)*/
		UINT32 data_xfer_ok : 1;	/*[7] Data Transfer Completed ok*/
		UINT32 rx_higher : 1;		/*[8] RxFIFO count > threshold*/
		UINT32 tx_lower : 1;		/*[9] TxFIFO count < threshold*/
		UINT32 dat1_irq : 1;		/*[10] SDIO DAT1 Interrupt*/
		UINT32 dma_done : 1;		/*[11] DMA Done*/
		UINT32 rxfifo_full : 1;		/*[12] RxFIFO Full*/
		UINT32 txfifo_empty : 1;	/*[13] TxFIFO Empty*/
		UINT32 addi_dat1_irq : 1;   /*[14] Additional SDIO DAT1 Interrupt*/
		UINT32 reserved : 1;		/*[15] reserved*/
		UINT32 dat1_irq_delay : 2;  /*[17:16] sdio dat1 interrupt mask windows clear
			delay control,0:2cycle 1:1cycles*/
		UINT32 reserved1 : 14;		/*[31:18] reserved*/
	} Bit;
} SDHC_ICTL;

/*Note1: W1C is write one clear.*/
typedef union sdhc_ista {
	UINT32 Raw;
	struct
	{
		UINT32 resp_ok : 1;			/*[0] Response is received OK (W1C)*/
		UINT32 resp_timeout : 1;	/*[1] Response is received Failed (Timeout Error) (W1C)*/
		UINT32 resp_err_crc : 1;	/*[2] Response is received Failed (CRC Error) (W1C)*/
		UINT32 resp_ok_noclear : 1; /*[3] Response is Received OK (always no self reset)*/
		UINT32 data_1pack_ok : 1;   /*[4] One Package Data Completed ok (W1C)*/
		UINT32 data_timeout : 1;	/*[5] One Package Data Failed (Timeout Error) (W1C)*/
		UINT32 data_err_crc : 1;	/*[6] One Package Data Failed (CRC Error) (W1C)*/
		UINT32 data_xfer_ok : 1;	/*[7] Data Transfer Completed ok (W1C)*/
		UINT32 rx_higher : 1;		/*[8] RxFIFO count > threshold (W1C)*/
		UINT32 tx_lower : 1;		/*[9] TxFIFO count < threshold (W1C)*/
		UINT32 dat1_irq : 1;		/*[10] SDIO DAT1 Interrupt (W1C)*/
		UINT32 dma_done : 1;		/*[11] DMA Done (W1C)*/
		UINT32 rxfifo_full : 1;		/*[12] RxFIFO Full(W1C)*/
		UINT32 txfifo_empty : 1;	/*[13] TxFIFO Empty(W1C)*/
		UINT32 addi_dat1_irq : 1;   /*[14] Additional SDIO DAT1 Interrupt*/
		UINT32 reserved : 17;		/*[31:13] reserved*/
	} Bit;
} SDHC_ISTA;

/*
* Note1: Soft reset for DPHY TX/RX needs programmer to set it
* and then clear it manually.*/
typedef union sdhc_srst {
	UINT32 Raw;
	struct
	{
		UINT32 main_ctrl : 1; /*[0] Soft reset for MAIN CTRL(self clear)*/
		UINT32 rxfifo : 1;	/*[1] Soft reset for RX FIFO(self clear)*/
		UINT32 txfifo : 1;	/*[2] Soft reset for TX FIFO(self clear)*/
		UINT32 dphy_rx : 1;   /*[3] Soft reset for DPHY RX*/
		UINT32 dphy_tx : 1;   /*[4] Soft reset for DPHY TX*/
		UINT32 dma_if : 1;	/*[5] Soft reset for DMA IF(self clear)*/
		UINT32 reserved : 26; /*[31:6] reserved*/
	} Bit;
} SDHC_SRST;

typedef union sdhc_enhc {
	UINT32 Raw;
	struct
	{
		UINT32 rx_timeout : 8;		/*[7:0] Data Rx Timeout Setting*/
		UINT32 sdio_irq_period : 8; /*[15:8] SDIO IRQ Period Setting					
			(IRQ checking window length)*/
		UINT32 dma_rd_resp : 1;		/*[16] No Read DMA Response Check*/
		UINT32 dma_wr_resp : 1;		/*[16] No Write DMA Response Check*/
		UINT32 rxfifo_th : 7;		/*[24:18] RXFIFO Full Threshold,default 60*/
		UINT32 txfifo_th : 7;		/*[31:25] TXFIFO Empty Threshold,default 0*/
	} MESON;
} SDHC_ENHC;

typedef union sdhc_clk2 {
	UINT32 Raw;
	struct
	{
		UINT32 rx_clk_phase : 12; /*[11:0] rx_clk phase diff(default 0:no diff,
			1:one input clock cycle ...)*/
		UINT32 sd_clk_phase : 12; /*[23:12] sd_clk phase diff(default 0:half(180 degree),
			1:half+one input clock cycle, 2:half+2 input clock cycles, ...)*/
		UINT32 reserved : 8;	  /*[31:24] reserved*/
	} Bit;
} SDHC_CLK2;

#define SDHC_CLOCK_SRC_OSC 0 // 24MHz
#define SDHC_CLOCK_SRC_FCLK_DIV4 1
#define SDHC_CLOCK_SRC_FCLK_DIV3 2
#define SDHC_CLOCK_SRC_FCLK_DIV5 3
#define SDHC_ISTA_W1C_ALL 0x7fff
#define SDHC_SRST_ALL 0x3f
#define SDHC_ICTL_ALL 0x7fff

#define STAT_POLL_TIMEOUT 0xfffff

#define MMC_RSP_136_NUM 4
#define MMC_MAX_DEVICE 3
#define MMC_TIMEOUT 5000

#define AML_MMC_DISABLED_TIMEOUT 100
#define AML_MMC_SLEEP_TIMEOUT 1000
#define AML_MMC_OFF_TIMEOUT 8000

#define SDHC_BOUNCE_REQ_SIZE (512 * 1024)
#define SDIO_BOUNCE_REQ_SIZE (128 * 1024)
#define MMC_TIMEOUT_MS 20

#define MESON_SDIO_PORT_A 0
#define MESON_SDIO_PORT_B 1
#define MESON_SDIO_PORT_C 2
#define MESON_SDIO_PORT_XC_A 3
#define MESON_SDIO_PORT_XC_B 4
#define MESON_SDIO_PORT_XC_C 5

#endif
