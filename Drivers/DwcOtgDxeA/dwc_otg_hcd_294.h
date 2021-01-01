/* **************************************************************************
 *
 *  Copyright (C) 2011 AMLOGIC, INC.
 *
 *
 * remark: copy from @ trunk/drivers/usb/host  by Haixiang.Bao 2011.10.17
 *              haixiang.bao@amlogic.com
 *
 *
 ****************************************************************************/
#ifndef __DWC_OTG_HCD_H__
#define __DWC_OTG_HCD_H__

#include <Library/IoLib.h>

#define DWC_DRIVER_VERSION	"2.94 6-June-2012"

/* ------------------------------------------------------------------------- */
/*
 * Reasons for halting a host channel.
 */
typedef enum dwc_otg_halt_status 
{
	DWC_OTG_HC_XFER_NO_HALT_STATUS,
	DWC_OTG_HC_XFER_COMPLETE,
	DWC_OTG_HC_XFER_URB_COMPLETE,
	DWC_OTG_HC_XFER_ACK,
	DWC_OTG_HC_XFER_NAK,
	DWC_OTG_HC_XFER_NYET,
	DWC_OTG_HC_XFER_STALL,
	DWC_OTG_HC_XFER_XACT_ERR,
	DWC_OTG_HC_XFER_FRAME_OVERRUN,
	DWC_OTG_HC_XFER_BABBLE_ERR,
	DWC_OTG_HC_XFER_DATA_TOGGLE_ERR,
	DWC_OTG_HC_XFER_AHB_ERR,
	DWC_OTG_HC_XFER_PERIODIC_INCOMPLETE,
	DWC_OTG_HC_XFER_URB_DEQUEUE
} dwc_otg_halt_status_e;

/**
 * Host channel descriptor. This structure represents the state of a single
 * host channel when acting in host mode. It contains the data items needed to
 * transfer packets to an endpoint via a host channel.
 */
typedef struct dwc_hc 
{
	/** Host channel number used for register address lookup */
	UINT8	 hc_num;

	/** Device to access */
	unsigned dev_addr : 7;

	/** EP to access */
	unsigned ep_num : 4;

	/** EP direction. 0: OUT, 1: IN */
	unsigned ep_is_in : 1;

	/**
	 * EP speed.
	 * One of the following values:
	 *	- DWC_OTG_EP_SPEED_LOW
	 *	- DWC_OTG_EP_SPEED_FULL
	 *	- DWC_OTG_EP_SPEED_HIGH
	 */
	unsigned speed : 2;
#define DWC_OTG_EP_SPEED_LOW	0
#define DWC_OTG_EP_SPEED_FULL	1
#define DWC_OTG_EP_SPEED_HIGH	2	

	/**
	 * Endpoint type.
	 * One of the following values:
	 *	- DWC_OTG_EP_TYPE_CONTROL: 0
	 *	- DWC_OTG_EP_TYPE_ISOC: 1
	 *	- DWC_OTG_EP_TYPE_BULK: 2
	 *	- DWC_OTG_EP_TYPE_INTR: 3
	 */
	unsigned ep_type : 2;

	/** Max packet size in bytes */
	unsigned max_packet : 11;

	/**
	 * PID for initial transaction.
	 * 0: DATA0,<br>
	 * 1: DATA2,<br>
	 * 2: DATA1,<br>
	 * 3: MDATA (non-Control EP),
	 *	  SETUP (Control EP)
	 */
	unsigned data_pid_start : 2;
#define DWC_OTG_HC_PID_DATA0 0
#define DWC_OTG_HC_PID_DATA2 1
#define DWC_OTG_HC_PID_DATA1 2
#define DWC_OTG_HC_PID_MDATA 3
#define DWC_OTG_HC_PID_SETUP 3

	/** Number of periodic transactions per (micro)frame */
	unsigned multi_count: 2;

	/** @name Transfer State */
	/** @{ */

	/** Pointer to the current transfer buffer position. */
	UINT8 *xfer_buff;
	/** Total number of bytes to transfer. */
	UINT32 xfer_len;
	/** Number of bytes transferred so far. */
	UINT32 xfer_count;
	/** Packet count at start of transfer.*/
	UINT16 start_pkt_count;

	/**
	 * Flag to indicate whether the transfer has been started. Set to 1 if
	 * it has been started, 0 otherwise.
	 */
	UINT8 xfer_started;

	/**
	 * Set to 1 to indicate that a PING request should be issued on this
	 * channel. If 0, process normally.
	 */
	UINT8 do_ping;

	/**
	 * Set to 1 to indicate that the error count for this transaction is
	 * non-zero. Set to 0 if the error count is 0.
	 */
	UINT8 error_state;

	/**
	 * Set to 1 to indicate that this channel should be halted the next
	 * time a request is queued for the channel. This is necessary in
	 * slave mode if no request queue space is available when an attempt
	 * is made to halt the channel.
	 */
	UINT8 halt_on_queue;

	/**
	 * Set to 1 if the host channel has been halted, but the core is not
	 * finished flushing queued requests. Otherwise 0.
	 */
	UINT8 halt_pending;

	/**
	 * Reason for halting the host channel.
	 */
	dwc_otg_halt_status_e	halt_status;

	/*
	 * Split settings for the host channel
	 */
	UINT8 do_split;		   /**< Enable split for the channel */
	UINT8 complete_split;	   /**< Enable complete split */
	UINT8 hub_addr;		   /**< Address of high speed hub */

	UINT8 port_addr;		   /**< Port of the low/full speed device */
	/** Split transaction position 
	 * One of the following values:
	 *	  - DWC_HCSPLIT_XACTPOS_MID 
	 *	  - DWC_HCSPLIT_XACTPOS_BEGIN
	 *	  - DWC_HCSPLIT_XACTPOS_END
	 *	  - DWC_HCSPLIT_XACTPOS_ALL */
	UINT8 xact_pos;

	/** Set when the host channel does a short read. */
	UINT8 short_read;

	/**
	 * Number of requests issued for this channel since it was assigned to
	 * the current transfer (not counting PINGs).
	 */
	UINT8 requests;

	/**
	 * Queue Head for the transfer being processed by this channel.
	 */
	struct dwc_otg_qh *qh;

	/** @} */

	/** Entry in list of host channels. */
	//struct list_head	hc_list_entry;
} dwc_hc_t;
#define DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE 0
#define DWC_OTG_CAP_PARAM_SRP_ONLY_CAPABLE 1
#define DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE 2

#define DWC_PHY_TYPE_PARAM_FS 0
#define DWC_PHY_TYPE_PARAM_UTMI 1
#define DWC_PHY_TYPE_PARAM_ULPI 2

#define DWC_SPEED_PARAM_HIGH 0
#define DWC_SPEED_PARAM_FULL 1
/**
 * The following parameters may be specified when starting the module. These
 * parameters define how the DWC_otg controller should be configured.
 */
typedef struct dwc_otg_core_params {
	INT32 opt;

	/**
	 * Specifies the OTG capabilities. The driver will automatically
	 * detect the value for this parameter if none is specified.
	 * 0 - HNP and SRP capable (default)
	 * 1 - SRP Only capable
	 * 2 - No HNP/SRP capable
	 */
	INT32 otg_cap;

	/**
	 * Specifies whether to use slave or DMA mode for accessing the data
	 * FIFOs. The driver will automatically detect the value for this
	 * parameter if none is specified.
	 * 0 - Slave
	 * 1 - DMA (default, if available)
	 */
	INT32 dma_enable;

	/**
	 * When DMA mode is enabled specifies whether to use address DMA or DMA 
	 * Descriptor mode for accessing the data FIFOs in device mode. The driver 
	 * will automatically detect the value for this if none is specified.
	 * 0 - address DMA
	 * 1 - DMA Descriptor(default, if available)
	 */
	INT32 dma_desc_enable;
	/** The DMA Burst size (applicable only for External DMA
	 * Mode). 1, 4, 8 16, 32, 64, 128, 256 (default 32)
	 */
	INT32 dma_burst_size;	/* Translate this to GAHBCFG values */

	/**
	 * Specifies the maximum speed of operation in host and device mode.
	 * The actual speed depends on the speed of the attached device and
	 * the value of phy_type. The actual speed depends on the speed of the
	 * attached device.
	 * 0 - High Speed (default)
	 * 1 - Full Speed
	 */
	INT32 speed;
	/** Specifies whether low power mode is supported when attached
	 *	to a Full Speed or Low Speed device in host mode.
	 * 0 - Don't support low power mode (default)
	 * 1 - Support low power mode
	 */
	INT32 host_support_fs_ls_low_power;

	/** Specifies the PHY clock rate in low power mode when connected to a
	 * Low Speed device in host mode. This parameter is applicable only if
	 * HOST_SUPPORT_FS_LS_LOW_POWER is enabled. If PHY_TYPE is set to FS
	 * then defaults to 6 MHZ otherwise 48 MHZ.
	 *
	 * 0 - 48 MHz
	 * 1 - 6 MHz
	 */
	INT32 host_ls_low_power_phy_clk;

	/**
	 * 0 - Use cC FIFO size parameters
	 * 1 - Allow dynamic FIFO sizing (default)
	 */
	INT32 enable_dynamic_fifo;

	/** Total number of 4-byte words in the data FIFO memory. This
	 * memory includes the Rx FIFO, non-periodic Tx FIFO, and periodic
	 * Tx FIFOs.
	 * 32 to 32768 (default 8192)
	 * Note: The total FIFO memory depth in the FPGA configuration is 8192.
	 */
	INT32 data_fifo_size;

	/** Number of 4-byte words in the Rx FIFO in device mode when dynamic
	 * FIFO sizing is enabled.
	 * 16 to 32768 (default 1064)
	 */
	INT32 dev_rx_fifo_size;

	/** Number of 4-byte words in the non-periodic Tx FIFO in device mode
	 * when dynamic FIFO sizing is enabled.
	 * 16 to 32768 (default 1024)
	 */
	INT32 dev_nperio_tx_fifo_size;

	/** Number of 4-byte words in each of the periodic Tx FIFOs in device
	 * mode when dynamic FIFO sizing is enabled.
	 * 4 to 768 (default 256)
	 */
	UINT32 dev_perio_tx_fifo_size[MAX_PERIO_FIFOS];

	/** Number of 4-byte words in the Rx FIFO in host mode when dynamic
	 * FIFO sizing is enabled.
	 * 16 to 32768 (default 1024)
	 */
	INT32 host_rx_fifo_size;

	/** Number of 4-byte words in the non-periodic Tx FIFO in host mode
	 * when Dynamic FIFO sizing is enabled in the core.
	 * 16 to 32768 (default 1024)
	 */
	INT32 host_nperio_tx_fifo_size;

	/** Number of 4-byte words in the host periodic Tx FIFO when dynamic
	 * FIFO sizing is enabled.
	 * 16 to 32768 (default 1024)
	 */
	INT32 host_perio_tx_fifo_size;

	/** The maximum transfer size supported in bytes.
	 * 2047 to 65,535  (default 65,535)
	 */
	INT32 max_transfer_size;

	/** The maximum number of packets in a transfer.
	 * 15 to 511  (default 511)
	 */
	INT32 max_packet_count;

	/** The number of host channel registers to use.
	 * 1 to 16 (default 12)
	 * Note: The FPGA configuration supports a maximum of 12 host channels.
	 */
	INT32 host_channels;

	/** The number of endpoints in addition to EP0 available for device
	 * mode operations.
	 * 1 to 15 (default 6 IN and OUT)
	 * Note: The FPGA configuration supports a maximum of 6 IN and OUT
	 * endpoints in addition to EP0.
	 */
	INT32 dev_endpoints;

		/**
		 * Specifies the type of PHY interface to use. By default, the driver
		 * will automatically detect the phy_type.
		 *
		 * 0 - Full Speed PHY
		 * 1 - UTMI+ (default)
		 * 2 - ULPI
		 */
	INT32 phy_type;

	/**
	 * Specifies the UTMI+ Data Width. This parameter is
	 * applicable for a PHY_TYPE of UTMI+ or ULPI. (For a ULPI
	 * PHY_TYPE, this parameter indicates the data width between
	 * the MAC and the ULPI Wrapper.) Also, this parameter is
	 * applicable only if the OTG_HSPHY_WIDTH cC parameter was set
	 * to "8 and 16 bits", meaning that the core has been
	 * configured to work at either data path width.
	 *
	 * 8 or 16 bits (default 16)
	 */
	INT32 phy_utmi_width;

	/**
	 * Specifies whether the ULPI operates at double or single
	 * data rate. This parameter is only applicable if PHY_TYPE is
	 * ULPI.
	 *
	 * 0 - single data rate ULPI interface with 8 bit wide data
	 * bus (default)
	 * 1 - double data rate ULPI interface with 4 bit wide data
	 * bus
	 */
	INT32 phy_ulpi_ddr;

	/**
	 * Specifies whether to use the internal or external supply to
	 * drive the vbus with a ULPI phy.
	 */
	INT32 phy_ulpi_ext_vbus;

	/**
	 * Specifies whether to use the I2Cinterface for full speed PHY. This
	 * parameter is only applicable if PHY_TYPE is FS.
	 * 0 - No (default)
	 * 1 - Yes
	 */
	INT32 i2c_enable;

	INT32 ulpi_fs_ls;

	INT32 ts_dline;

	/**
	 * Specifies whether dedicated transmit FIFOs are
	 * enabled for non periodic IN endpoints in device mode
	 * 0 - No
	 * 1 - Yes
	 */
	INT32 en_multiple_tx_fifo;

	/** Number of 4-byte words in each of the Tx FIFOs in device
	 * mode when dynamic FIFO sizing is enabled.
	 * 4 to 768 (default 256)
	 */
	UINT32 dev_tx_fifo_size[MAX_TX_FIFOS];

	/** Thresholding enable flag-
	 * bit 0 - enable non-ISO Tx thresholding
	 * bit 1 - enable ISO Tx thresholding
	 * bit 2 - enable Rx thresholding
	 */
	UINT32 thr_ctl;

	/** Thresholding length for Tx
	 *	FIFOs in 32 bit DWORDs
	 */
	UINT32 tx_thr_length;

	/** Thresholding length for Rx
	 *	FIFOs in 32 bit DWORDs
	 */
	UINT32 rx_thr_length;

	/**
	 * Specifies whether LPM (Link Power Management) support is enabled
	 */
	INT32 lpm_enable;

	/** Per Transfer Interrupt
	 *	mode enable flag
	 * 1 - Enabled
	 * 0 - Disabled
	 */
	INT32 pti_enable;

	/** Multi Processor Interrupt
	 *	mode enable flag
	 * 1 - Enabled
	 * 0 - Disabled
	 */
	INT32 mpi_enable;

	/** IS_USB Capability
	 * 1 - Enabled
	 * 0 - Disabled
	 */
	INT32 ic_usb_cap;

	/** AHB Threshold Ratio
	 * 2'b00 AHB Threshold = 	MAC Threshold
	 * 2'b01 AHB Threshold = 1/2 	MAC Threshold
	 * 2'b10 AHB Threshold = 1/4	MAC Threshold
	 * 2'b11 AHB Threshold = 1/8	MAC Threshold
	 */
	INT32 ahb_thr_ratio;

	/** ADP Support
	 * 1 - Enabled
	 * 0 - Disabled
	 */
	INT32 adp_supp_enable;

	/** HFIR Reload Control
	 * 0 - The HFIR cannot be reloaded dynamically.
	 * 1 - Allow dynamic reloading of the HFIR register during runtime.
	 */
	INT32 reload_ctl;

	/** DCFG: Enable device Out NAK 
	 * 0 - The core does not set NAK after Bulk Out transfer complete.
	 * 1 - The core sets NAK after Bulk OUT transfer complete.
	 */
	INT32 dev_out_nak;

	/** DCFG: Enable Continue on BNA 
	 * After receiving BNA interrupt the core disables the endpoint,when the
	 * endpoint is re-enabled by the application the core starts processing 
	 * 0 - from the DOEPDMA descriptor
	 * 1 - from the descriptor which received the BNA.
	 */
	INT32 cont_on_bna;

	/** GAHBCFG: AHB Single Support 
	 * This bit when programmed supports SINGLE transfers for remainder 
	 * data in a transfer for DMA mode of operation.
	 * 0 - in this case the remainder data will be sent using INCR burst size.
	 * 1 - in this case the remainder data will be sent using SINGLE burst size.
	 */
	INT32 ahb_single;

	/** Core Power down mode
	 * 0 - No Power Down is enabled
	 * 1 - Reserved
	 * 2 - Complete Power Down (Hibernation)
	 */
	INT32 power_down;

	/** OTG revision supported
	 * 0 - OTG 1.3 revision
	 * 1 - OTG 2.0 revision
	 */
	INT32 otg_ver;

} dwc_otg_core_params_t;
/**
 * The <code>dwc_otg_core_if</code> structure contains information needed to manage
 * the DWC_otg controller acting in either host or device mode. It
 * represents the programming view of the controller as a whole.
 */
typedef struct dwc_otg_core_if 
{
	/** Parameters that define how the core should be configured.*/
	dwc_otg_core_params_t	   *core_params;

	/** Core Global registers starting at offset 000h. */
	dwc_otg_core_global_regs_t *core_global_regs;
  
	/** Device-specific information */
	dwc_otg_dev_if_t		   *dev_if;
	/** Host-specific information */
	dwc_otg_host_if_t		   *host_if;

	/*
	 * Set to 1 if the core PHY interface bits in USBCFG have been
	 * initialized.
	 */
	UINT8 phy_init_done;

	/*
	 * SRP Success flag, set by srp success interrupt in FS I2C mode
	 */
	UINT8 srp_success;
	UINT8 srp_timer_started;

	/* Common configuration information */
	/** Power and Clock Gating Control Register */
	volatile UINT32 *pcgcctl;
#define DWC_OTG_PCGCCTL_OFFSET 0xE00

	/** Push/pop addresses for endpoints or host channels.*/
	UINT32 *data_fifo[MAX_EPS_CHANNELS];
#define DWC_OTG_DATA_FIFO_OFFSET 0x1000
#define DWC_OTG_DATA_FIFO_SIZE 0x1000

	/** Total RAM for FIFOs (Bytes) */
	UINT16 total_fifo_size;
	/** Size of Rx FIFO (Bytes) */
	UINT16 rx_fifo_size;
	/** Size of Non-periodic Tx FIFO (Bytes) */
	UINT16 nperio_tx_fifo_size;
		
		
	/** 1 if DMA is enabled, 0 otherwise. */
	UINT8 dma_enable;

	/** 1 if dedicated Tx FIFOs are enabled, 0 otherwise. */
	UINT8 en_multiple_tx_fifo;

	/** Set to 1 if multiple packets of a high-bandwidth transfer is in
	 * process of being queued */
	UINT8 queuing_high_bandwidth;

	/** Hardware Configuration -- stored here for convenience.*/
	hwcfg1_data_t hwcfg1;
	hwcfg2_data_t hwcfg2;
	hwcfg3_data_t hwcfg3;
	hwcfg4_data_t hwcfg4;

	/** The operational State, during transations
	 * (a_host>>a_peripherial and b_device=>b_host) this may not
	 * match the core but allows the software to determine
	 * transitions.
	 */
	UINT8 op_state;
		
	/**
	 * Set to 1 if the HCD needs to be restarted on a session request
	 * interrupt. This is required if no connector ID status change has
	 * occurred since the HCD was last disconnected.
	 */
	UINT8 restart_hcd_on_session_req;

	/** HCD callbacks */
	/** A-Device is a_host */
#define A_HOST		(1)
	/** A-Device is a_suspend */
#define A_SUSPEND	(2)
	/** A-Device is a_peripherial */
#define A_PERIPHERAL	(3)
	/** B-Device is operating as a Peripheral. */
#define B_PERIPHERAL	(4)
	/** B-Device is operating as a Host. */
#define B_HOST		(5)		   
#define DWC_OTG_MAX_TRANSFER_SIZE  0x10000
    char *temp_buffer;
    int transfer_size;
 
 	 /* Set VBus Power though GPIO */
	 void (* set_vbus_power)(char is_power_on);
    
#if 0
	/** HCD callbacks */
	struct dwc_otg_cil_callbacks *hcd_cb;
	/** PCD callbacks */
	struct dwc_otg_cil_callbacks *pcd_cb;		 
#endif
	/** Device mode Periodic Tx FIFO Mask */
	UINT32 p_tx_msk;
	/** Device mode Periodic Tx FIFO Mask */
	UINT32 tx_msk;

	UINT32 otg_ver;
#if 0	
	UINT32		start_hcchar_val[MAX_EPS_CHANNELS];

	hc_xfer_info_t		hc_xfer_info[MAX_EPS_CHANNELS];
	struct timer_list	hc_xfer_timer[MAX_EPS_CHANNELS];

	UINT32		hfnum_7_samples;
	uint64_t		hfnum_7_frrem_accum;
	UINT32		hfnum_0_samples;
	uint64_t		hfnum_0_frrem_accum;
	UINT32		hfnum_other_samples;
	uint64_t		hfnum_other_frrem_accum;
	
#endif

} dwc_otg_core_if_t;
/**
 * This structure is a wrapper that encapsulates the driver components used to
 * manage a single DWC_otg controller.
 */
typedef struct dwc_otg_device
{
	/** Base address returned from ioremap() */
	void *base;


	/** Pointer to the core interface structure. */
	dwc_otg_core_if_t *core_if;

	/** Register offset for Diagnostic API.*/
	UINT32 reg_offset;
		
	/** Pointer to the HCD structure. */
	struct dwc_otg_hcd *hcd;

	/** Flag to indicate whether the common IRQ handler is installed. */
	UINT8 common_irq_installed;
  
  int disabled;

	int index;
} dwc_otg_device_t;
/**
 * Reads the content of a register.
 *
 * @param _reg address of register to read.
 * @return contents of the register.
 *

 * Usage:<br>
 * <code>UINT32 dev_ctl = dwc_read_reg32(&dev_regs->dctl);</code> 
 */

/**
 * This function returns the mode of the operation, host or device.
 *
 * @return 0 - Device Mode, 1 - Host Mode 
 */
static inline UINT32 dwc_otg_mode(dwc_otg_core_if_t *_core_if) 
{
	//doubt
	return (MmioRead32( _core_if->core_global_regs->gintsts ) & 0x1);
}

static inline UINT8 dwc_otg_is_device_mode(dwc_otg_core_if_t *_core_if) 
{
	return (dwc_otg_mode(_core_if) != DWC_HOST_MODE);
}
static inline UINT8 dwc_otg_is_host_mode(dwc_otg_core_if_t *_core_if) 
{
	return (dwc_otg_mode(_core_if) == DWC_HOST_MODE);
}


/* --- USB HUB constants (not OHCI-specific; see hub.h) -------------------- */

/* destination of request */
#define RH_INTERFACE               0x01
#define RH_ENDPOINT                0x02
#define RH_OTHER                   0x03

#define RH_CLASS                   0x20
#define RH_VENDOR                  0x40

/* Requests: bRequest << 8 | bmRequestType */
#define RH_GET_STATUS           0x0080
#define RH_CLEAR_FEATURE        0x0100
#define RH_SET_FEATURE          0x0300
#define RH_SET_ADDRESS          0x0500
#define RH_GET_DESCRIPTOR       0x0680
#define RH_SET_DESCRIPTOR       0x0700
#define RH_GET_CONFIGURATION    0x0880
#define RH_SET_CONFIGURATION    0x0900
#define RH_GET_STATE            0x0280
#define RH_GET_INTERFACE        0x0A80
#define RH_SET_INTERFACE        0x0B00
#define RH_SYNC_FRAME           0x0C80
/* Our Vendor Specific Request */
#define RH_SET_EP               0x2000

/* Hub port features */
#define RH_PORT_CONNECTION         0x00
#define RH_PORT_ENABLE             0x01
#define RH_PORT_SUSPEND            0x02
#define RH_PORT_OVER_CURRENT       0x03
#define RH_PORT_RESET              0x04
#define RH_PORT_POWER              0x08
#define RH_PORT_LOW_SPEED          0x09

#define RH_C_PORT_CONNECTION       0x10
#define RH_C_PORT_ENABLE           0x11
#define RH_C_PORT_SUSPEND          0x12
#define RH_C_PORT_OVER_CURRENT     0x13
#define RH_C_PORT_RESET            0x14

/* Hub features */
#define RH_C_HUB_LOCAL_POWER       0x00
#define RH_C_HUB_OVER_CURRENT      0x01

#define RH_DEVICE_REMOTE_WAKEUP    0x00
#define RH_ENDPOINT_STALL          0x01

#define RH_ACK                     0x01
#define RH_REQ_ERR                 -1
#define RH_NACK                    0x00

#endif

