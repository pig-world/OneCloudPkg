/** @file

  Copyright (c) 2008 - 2009, Apple Inc. All rights reserved.<BR>
  Copyright (c) 2016, Linaro, Ltd. All rights reserved.<BR>

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <PiDxe.h>

#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/NonDiscoverableDeviceRegistrationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/TimerLib.h>

#include <Protocol/EmbeddedExternalDevice.h>

#include "dwc_otg_hcd_regs_294.h"
#include "dwc_otg_hcd_294.h"

#define CONFIG_M8_USBPORT_BASE_A	0xC9040000
#define CONFIG_M8_USBPORT_BASE_B	0xC90C0000
#define USB_PHY_CLK_SEL_XTAL	0

/* usb id mode, only after M2
	 mode = 0 : HARDWARE
	 mode = 1 : SW_HOST
	 mode = 2 : SW_DEVICE
 */
#define USB_ID_MODE_HARDWARE    (1)
#define USB_ID_MODE_SW_HOST     (2)
#define USB_ID_MODE_SW_DEVICE   (3) 

typedef struct amlogic_usb_config{
	/* clock info */
	int clk_selecter; // usb USB_PHY_CLOCK_SEL_xxx
	int pll_divider;  // when other/ddr/demod pll used, fill this
	
	/* controller */
	unsigned int base_addr;
	
	/* role */
	int id_mode; // only used after M2
	
	/* vbus call back */
	void (* set_vbus_power)(char is_power_on);

	/* battery charging detect call back */
	int(* battery_charging_det_cb)(char bc_mode);
#define BC_MODE_UNKNOWN	0
#define BC_MODE_SDP		1	/* Standard Downstream Port */
#define BC_MODE_DCP		2	/* Dedicated Charging Port */
#define BC_MODE_CDP		3	/* Charging Downstream Port */
	
}amlogic_usb_config_t;

#define BOARD_USB_MODE_HOST	0
#define BOARD_USB_MODE_SLAVE	1
#define BOARD_USB_MODE_CHARGER	2
#define BOARD_USB_MODE_MAX	3

#define USB_PHY_PORT_A	    (0x40000)
#define USB_PHY_PORT_B	    (0xc0000)
#define USB_PHY_PORT_C	    (0x100000)
#define USB_PHY_PORT_D	    (0x140000)
#define USB_PHY_PORT_MSK	(0x1f0000)
#define USB_PHY_PORT_MAX	2
#define PREI_USB_PHY_REG_A     0x2200
#define PREI_USB_PHY_REG_B     0x2208

#define IO_CBUS_BASE                                                      0xc1100000
#define CBUS_REG_OFFSET(reg)                                              ((reg) << 2)
#define CBUS_REG_ADDR(reg)                                                (IO_CBUS_BASE + CBUS_REG_OFFSET(reg))
#define P_RESET1_REGISTER                           						0xc1104408


static amlogic_usb_config_t * g_usb_cfg[BOARD_USB_MODE_MAX][USB_PHY_PORT_MAX];

struct amlogic_usb_config g_usb_config_m6_skt_a={
	USB_PHY_CLK_SEL_XTAL,
	1, //PLL divider: (clock/12 -1)
	CONFIG_M8_USBPORT_BASE_A,
	USB_ID_MODE_SW_HOST,
	NULL,//gpio_set_vbus_power, //set_vbus_power
	NULL,
};
struct amlogic_usb_config g_usb_config_m6_skt_b={
	USB_PHY_CLK_SEL_XTAL,
	1, //PLL divider: (clock/12 -1)
	CONFIG_M8_USBPORT_BASE_B,
	USB_ID_MODE_SW_HOST,
	NULL,//gpio_set_vbus_power, //set_vbus_power
	NULL,
};

typedef struct usb_peri_reg {
	volatile UINT32 config; 
	volatile UINT32 ctrl;      
	volatile UINT32 endp_intr; 
	volatile UINT32 adp_bc;    
	volatile UINT32 dbg_uart;  
	volatile UINT32 test;
	volatile UINT32 tune;
	volatile UINT32 reserved;
} usb_peri_reg_t;

typedef union usb_config_data {
    /** raw register data */
    UINT32 d32;
    /** register bits */
    struct {
        unsigned clk_en:1;
        unsigned clk_sel:3;
        unsigned clk_div:7;
        unsigned reserved0:4;
        unsigned clk_32k_alt_sel:1;
        unsigned reserved1:15;
        unsigned test_trig:1;
    } b;
} usb_config_data_t;

typedef union usb_ctrl_data {
    /** raw register data */
    UINT32 d32;
    /** register bits */
    struct {
        unsigned soft_prst:1;
        unsigned soft_hreset:1;
        unsigned ss_scaledown_mode:2;
        unsigned clk_det_rst:1;
        unsigned intr_sel:1;
        unsigned reserved:2;
        unsigned clk_detected:1;
        unsigned sof_sent_rcvd_tgl:1; 
        unsigned sof_toggle_out:1; 
        unsigned not_used:4;
        unsigned por:1;
        unsigned sleepm:1;
        unsigned txbitstuffennh:1;
        unsigned txbitstuffenn:1;
        unsigned commononn:1; 
        unsigned refclksel:2; 
        unsigned fsel:3;
        unsigned portreset:1;
        unsigned thread_id:6;
    } b;
} usb_ctrl_data_t;

static int set_usb_phy_clock(amlogic_usb_config_t * usb_cfg)
{
	
	int port_idx;
	usb_peri_reg_t * peri;
	usb_config_data_t config;
	usb_ctrl_data_t control;
	int clk_sel,clk_div;
	unsigned int port = usb_cfg->base_addr & USB_PHY_PORT_MSK;
	
	if(!usb_cfg)
		return -1;


	if(port == USB_PHY_PORT_A){
		port_idx = 0;
		peri = (usb_peri_reg_t*)CBUS_REG_ADDR(PREI_USB_PHY_REG_A);
	}else if(port == USB_PHY_PORT_B){
		port_idx = 1;
		peri = (usb_peri_reg_t*)CBUS_REG_ADDR(PREI_USB_PHY_REG_B);
	}else{
		DEBUG((EFI_D_INFO,"usb base address error: %x\n",usb_cfg->base_addr));
		return -1;
	}
	MmioWrite32(P_RESET1_REGISTER,(1 << 2));	
	DEBUG((EFI_D_INFO,"USB (%d) peri reg base: %x\n",port_idx,(UINT32)peri));

	clk_sel = usb_cfg->clk_selecter;
	clk_div = usb_cfg->pll_divider;

	config.d32 = peri->config;
	config.b.clk_32k_alt_sel= 1;
	peri->config = config.d32;

	DEBUG((EFI_D_INFO,"USB (%d) use clock source: %s\n",port_idx,"XTAL input"));

	control.d32 = peri->ctrl;
	control.b.fsel = 5;	/* PHY default is 24M (5), change to 12M (2) */
	control.b.por = 1;  /* power off default*/
	peri->ctrl = control.d32;
	MicroSecondDelay(1);
	
	return 0;
}
//call after set clock
void set_usb_phy_power(amlogic_usb_config_t * usb_cfg,int is_on)
{
	int port_idx = 100;
	unsigned int port = usb_cfg->base_addr & USB_PHY_PORT_MSK;
	usb_peri_reg_t *peri_a,*peri_b,*peri_c,*peri_d,*peri;
	peri = NULL;
	usb_ctrl_data_t control;

	peri_a = (usb_peri_reg_t*)CBUS_REG_ADDR(PREI_USB_PHY_REG_A);
	peri_b = (usb_peri_reg_t*)CBUS_REG_ADDR(PREI_USB_PHY_REG_B);
//	peri_c = (usb_peri_reg_t*)CBUS_REG_ADDR(PREI_USB_PHY_REG_C);
//	peri_d = (usb_peri_reg_t*)CBUS_REG_ADDR(PREI_USB_PHY_REG_D);

	if(port == USB_PHY_PORT_A){
		peri = peri_a;
		port_idx = 0;
	}else if(port == USB_PHY_PORT_B){
		peri = peri_b;
		port_idx = 1;
	}else if(port == USB_PHY_PORT_C){
		peri = peri_c;
		port_idx = 2;
	}else if(port == USB_PHY_PORT_D){
		peri = peri_d;
		port_idx = 3;
	}
	
	if(is_on){
		control.d32 = peri->ctrl;
		control.b.por = 0;
		peri->ctrl = control.d32;

		MicroSecondDelay(1);
		/* read back clock detected flag*/
		control.d32 = peri->ctrl;
		if(!control.b.clk_detected){
			DEBUG((EFI_D_INFO,"USB (%d) PHY Clock not detected!\n",port_idx));
		}

	}else{
		control.d32 = peri->ctrl;
		control.b.por = 1;
		peri->ctrl = control.d32;
	}
	MicroSecondDelay(1);
}

amlogic_usb_config_t * board_usb_start(int mode,int index)
{
	if(mode < 0 || mode >= BOARD_USB_MODE_MAX||!g_usb_cfg[mode][index])
		return 0;

	set_usb_phy_clock(g_usb_cfg[mode][index]);
	set_usb_phy_power(g_usb_cfg[mode][index],1);//on
	// if(mode == BOARD_USB_MODE_CHARGER && 
	    // g_usb_cfg[mode][index]->battery_charging_det_cb)
		// usb_bc_detect(g_usb_cfg[mode][index]);
	return g_usb_cfg[mode][index];
}
void board_usb_init(amlogic_usb_config_t * usb_cfg,int mode)
{
	static int usb_index = 0;
	if(mode < 0 || mode >= BOARD_USB_MODE_MAX || !usb_cfg)
		return ;
	
	if(mode == BOARD_USB_MODE_HOST){		
		if(usb_index >= USB_PHY_PORT_MAX)
			return;
		g_usb_cfg[mode][usb_index] = usb_cfg;
		usb_index++;
	}else
		g_usb_cfg[mode][0] = usb_cfg;
	DEBUG((EFI_D_INFO,"register usb cfg[%d][%d] = %p\n",mode,(mode==BOARD_USB_MODE_HOST)?usb_index:0,usb_cfg));
}

static dwc_otg_core_if_t *
dwc_otg_cil_init(const UINT32 * _reg_base_addr, dwc_otg_core_params_t * _core_params)
{

    dwc_otg_core_if_t *core_if = 0;
    // dwc_otg_dev_if_t *dev_if = 0;
    dwc_otg_host_if_t *host_if = 0;
    UINT8        *reg_base = (UINT8 *) _reg_base_addr;
    int             i = 0;

    DEBUG((EFI_D_INFO, "%s(%p,%p)\n", __func__, _reg_base_addr, _core_params));

    core_if = (dwc_otg_core_if_t *) AllocateZeroPool(sizeof(dwc_otg_core_if_t));

    if (core_if == 0) {
        DEBUG((EFI_D_INFO, "Allocation of dwc_otg_core_if_t failed\n"));
        return 0;
    }


    core_if->core_params = _core_params;
    core_if->core_global_regs = (dwc_otg_core_global_regs_t *) reg_base;

    /*
     * Allocate the Host Mode structures.
     */
    host_if = (dwc_otg_host_if_t *) AllocateZeroPool(sizeof(dwc_otg_host_if_t));

    if (host_if == 0) {
        DEBUG((EFI_D_INFO, "Allocation of dwc_otg_host_if_t failed\n"));
        // kfree(dev_if);
        FreePool(core_if); 
        return 0;
    }

    host_if->host_global_regs = (dwc_otg_host_global_regs_t *)
        (reg_base + DWC_OTG_HOST_GLOBAL_REG_OFFSET);

    host_if->hprt0 = (UINT32 *) (reg_base + DWC_OTG_HOST_PORT_REGS_OFFSET);

    for (i = 0; i < MAX_EPS_CHANNELS; i++) {
        host_if->hc_regs[i] = (dwc_otg_hc_regs_t *)(reg_base + DWC_OTG_HOST_CHAN_REGS_OFFSET + (i * DWC_OTG_CHAN_REGS_OFFSET));
        DEBUG((EFI_D_INFO, "hc_reg[%d]->hcchar=%p\n", i, &host_if->hc_regs[i]->hcchar));
    }

    host_if->num_host_channels = MAX_EPS_CHANNELS;
    core_if->host_if = host_if;

    for (i = 0; i < MAX_EPS_CHANNELS; i++) {
        core_if->data_fifo[i] =
            (UINT32 *) (reg_base + DWC_OTG_DATA_FIFO_OFFSET + (i * DWC_OTG_DATA_FIFO_SIZE));
        DEBUG((EFI_D_INFO, "data_fifo[%d]=0x%08x\n", i, (unsigned) core_if->data_fifo[i]));
    }

    core_if->pcgcctl = (UINT32 *) (reg_base + DWC_OTG_PCGCCTL_OFFSET);

    /*
     * Store the contents of the hardware configuration registers here for
     * easy access later.
     */
    // core_if->hwcfg1.d32 = MmioRead32(core_if->core_global_regs->ghwcfg1);
    // core_if->hwcfg2.d32 = MmioRead32(core_if->core_global_regs->ghwcfg2);
    // core_if->hwcfg3.d32 = MmioRead32(core_if->core_global_regs->ghwcfg3);
    // core_if->hwcfg4.d32 = MmioRead32(core_if->core_global_regs->ghwcfg4);

    // DEBUG((EFI_D_INFO, "hwcfg1=%08x\n", core_if->hwcfg1.d32));
    // DEBUG((EFI_D_INFO, "hwcfg2=%08x\n", core_if->hwcfg2.d32));
    // DEBUG((EFI_D_INFO, "hwcfg3=%08x\n", core_if->hwcfg3.d32));
    // DEBUG((EFI_D_INFO, "hwcfg4=%08x\n", core_if->hwcfg4.d32));

    DEBUG((EFI_D_INFO, "op_mode=%0x\n", core_if->hwcfg2.b.op_mode));
    DEBUG((EFI_D_INFO, "arch=%0x\n", core_if->hwcfg2.b.architecture));
    DEBUG((EFI_D_INFO, "num_dev_ep=%d\n", core_if->hwcfg2.b.num_dev_ep));
    DEBUG((EFI_D_INFO, "num_host_chan=%d\n", core_if->hwcfg2.b.num_host_chan));
    DEBUG((EFI_D_INFO, "nonperio_tx_q_depth=0x%0x\n", core_if->hwcfg2.b.nonperio_tx_q_depth));
    DEBUG((EFI_D_INFO, "host_perio_tx_q_depth=0x%0x\n", core_if->hwcfg2.b.host_perio_tx_q_depth));
    DEBUG((EFI_D_INFO, "dev_token_q_depth=0x%0x\n", core_if->hwcfg2.b.dev_token_q_depth));

    DEBUG((EFI_D_INFO, "Total FIFO SZ=%d\n", core_if->hwcfg3.b.dfifo_depth));
    DEBUG((EFI_D_INFO, "xfer_size_cntr_width=%0x\n", core_if->hwcfg3.b.xfer_size_cntr_width));
    
    /*
     * Set the SRP sucess bit for FS-I2c
     */
    core_if->srp_success = 0;
    core_if->srp_timer_started = 0;
    core_if->temp_buffer = AllocateZeroPool(DWC_OTG_MAX_TRANSFER_SIZE);
    if(!core_if->temp_buffer) {
        FreePool(core_if->host_if);
        FreePool(core_if);
        return 0;
    }        

    return core_if;

}
#define FORCE_ID_CLEAR	-1
#define FORCE_ID_HOST	0
#define FORCE_ID_SLAVE	1
#define FORCE_ID_ERROR	2
static void dwc_otg_set_force_id(dwc_otg_core_if_t *core_if,int mode)
{
	gusbcfg_data_t gusbcfg_data;

	gusbcfg_data.d32 = MmioRead32(core_if->core_global_regs->gusbcfg);
	switch(mode){
		case FORCE_ID_CLEAR:
			DEBUG((EFI_D_INFO,"Force id mode: Hardware\n"));
			gusbcfg_data.b.force_host_mode = 0;
			gusbcfg_data.b.force_dev_mode = 0;
			break;
		case FORCE_ID_HOST:
			DEBUG((EFI_D_INFO,"Force id mode: Host\n"));
			gusbcfg_data.b.force_host_mode = 1;
			gusbcfg_data.b.force_dev_mode = 0;
			break;
		case FORCE_ID_SLAVE:
			DEBUG((EFI_D_INFO,"Force id mode: Slave\n"));
			gusbcfg_data.b.force_host_mode = 0;
			gusbcfg_data.b.force_dev_mode = 1;
			break;
		default:
			DEBUG((EFI_D_INFO,"error id mode\n"));
			return;
			break;
	}
	MmioWrite32(core_if->core_global_regs->gusbcfg,gusbcfg_data.d32);

	gusbcfg_data.d32 = MmioRead32(core_if->core_global_regs->gusbcfg);
	DEBUG((EFI_D_INFO, "force_host: %d, force_device: %d\n",
		gusbcfg_data.b.force_host_mode,gusbcfg_data.b.force_dev_mode));

	return;
}

static struct dwc_otg_core_params dwc_otg_module_params_host = {
	.otg_cap = DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE,//NO HNP SRP
	.dma_enable = 0,
	.dma_desc_enable = 0,
	.dma_burst_size = 3,//DWC_GAHBCFG_INT_DMA_BURST_INCR4; 
	.speed = DWC_SPEED_PARAM_HIGH,//0 Highspeed, 1 Fullspeed
	.host_support_fs_ls_low_power = 0,
	.host_ls_low_power_phy_clk = 0,
	.enable_dynamic_fifo = 1,
	.data_fifo_size = 1024,
	.dev_endpoints = 6,
	.en_multiple_tx_fifo = 0,
	.dev_rx_fifo_size = 256,
	.dev_nperio_tx_fifo_size = 256,
	.dev_tx_fifo_size = {
			     /* dev_tx_fifo_size */
			     256,
			     256,
			     128,
			     -1,
			     -1,
			     -1,
			     -1,
			     -1,
			     -1,
			     -1,
			     -1,
			     -1,
			     -1,
			     -1,
			     -1
			     /* 15 */
			     },
	.dev_perio_tx_fifo_size = {
				   /* dev_perio_tx_fifo_size_1 */
				   -1,
				   -1,
				   -1,
				   -1,
				   -1,
				   -1,
				   -1,
				   -1,
				   -1,
				   -1,
				   -1,
				   -1,
				   -1,
				   -1,
				   -1
				   /* 15 */
				   },
	.host_rx_fifo_size = 512,
	.host_nperio_tx_fifo_size = 500,
	.host_perio_tx_fifo_size = 100,
	.max_transfer_size = ((1 << 19) - 1),
	.max_packet_count = ((1 << 10) - 1),
	.host_channels = 15,
	.phy_type = DWC_PHY_TYPE_PARAM_UTMI, //UTMI+
	.phy_utmi_width = 16,
	.phy_ulpi_ddr = 0,
	.phy_ulpi_ext_vbus = 0,
	.i2c_enable = 0,
	.ulpi_fs_ls = 0,
	.ts_dline = 0,
	.thr_ctl = 0,
	.tx_thr_length = 0,
	.rx_thr_length = 0,
	.pti_enable = 0,
	.mpi_enable = 0,
	.lpm_enable = 0,
	.ic_usb_cap = 0,
	.ahb_thr_ratio = 0,
	.power_down = 0,
	.reload_ctl = 0,
	.dev_out_nak = 0,
	.cont_on_bna = 0,
	.ahb_single = 1,
	.otg_ver = 1,
};
static dwc_otg_device_t dwc_otg_dev;
int
usb_lowlevel_init(int index)
{
	
    dwc_otg_device_t *dwc_otg_device = &dwc_otg_dev;
    
    INT32         snpsid;
    int           port_idx = 100;	//remove warning.
	int 		  id_mode=-1;
    amlogic_usb_config_t * usb_config;
    
    DEBUG((EFI_D_INFO,"dwc_usb driver version: 2020-03-16\n"));
    
    usb_config = board_usb_start(BOARD_USB_MODE_HOST,index);

    if((usb_config->base_addr & USB_PHY_PORT_MSK) == USB_PHY_PORT_A)
	    port_idx = 0;
    else if((usb_config->base_addr & USB_PHY_PORT_MSK) == USB_PHY_PORT_B)
	    port_idx = 1;

	dwc_otg_device=(dwc_otg_device_t *)AllocateZeroPool(sizeof(dwc_otg_device_t));
    //memset(dwc_otg_device, 0, sizeof(dwc_otg_device_t));

    DEBUG((EFI_D_INFO,"USB (%d) base addr: 0x%x\n",port_idx,usb_config->base_addr));
	
	dwc_otg_device->base = (void *)usb_config->base_addr;
    dwc_otg_device->index = index;
    snpsid = MmioRead32(usb_config->base_addr + 0x40);

    if ((snpsid & 0xFFFF0000) != 0x4F540000) {
        DEBUG((EFI_D_INFO,"Bad value for SNPSID: 0x%08x\n", snpsid));
        return -1;
    }
    //dwc_otg_module_params_host.dma_enable = 1;

    dwc_otg_device->core_if = dwc_otg_cil_init(dwc_otg_device->base, &dwc_otg_module_params_host);
    if (dwc_otg_device->core_if == 0) {
        DEBUG((EFI_D_INFO,"CIL initialization failed!\n"));
		return -1;
    }
    dwc_otg_device->core_if->set_vbus_power = usb_config->set_vbus_power;

	switch(usb_config->id_mode){
	case USB_ID_MODE_HARDWARE:
		id_mode = FORCE_ID_CLEAR;
		break;
	case USB_ID_MODE_SW_HOST:
		id_mode = FORCE_ID_HOST;		
		break;
	case USB_ID_MODE_SW_DEVICE:
		id_mode = FORCE_ID_SLAVE;
		break;
	default:
		id_mode = FORCE_ID_ERROR;
		break;
	}
     dwc_otg_set_force_id(dwc_otg_device->core_if,id_mode);
	 
	return 0;
    // /*
     // * Disable the global interrupt until all the interrupt
     // * handlers are installed.
     // */
    // //dwc_otg_disable_global_interrupts(dwc_otg_device->core_if);
    // /*
     // * Initialize the DWC_otg core.
     // */
    // if(dwc_otg_core_init(dwc_otg_device->core_if))
	// goto fail;
    // for (i = 0; i < 2; i++) {
        // dwc_otg_hc_cleanup(dwc_otg_device->core_if, i);
    // }

    // dwc_otg_core_host_init(dwc_otg_device->core_if);

    // /*
     // * Enable the global interrupt after all the interrupt
     // * handlers are installed.
     // */
    // //dwc_otg_enable_global_interrupts(dwc_otg_device->core_if);

    // if(!dwc_otg_port_init(dwc_otg_device->core_if))/* host initialization 6.1.1.3----6.1.1.9*/
				// goto fail;
				
    // dwc_otg_hcd_enable = 1;
    // dwc_otg_device->disabled = 0;
    // return 0;
  // fail:
    // return -1;
}


EFI_STATUS
EFIAPI
PciEmulationEntryPoint (
  IN EFI_HANDLE       ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  board_usb_init(&g_usb_config_m6_skt_a,BOARD_USB_MODE_HOST);
  board_usb_init(&g_usb_config_m6_skt_b,BOARD_USB_MODE_HOST);
  
  usb_lowlevel_init(0);
  usb_lowlevel_init(1);
  return RegisterNonDiscoverableMmioDevice (
           NonDiscoverableDeviceTypeEhci,
           NonDiscoverableDeviceDmaTypeNonCoherent,
           NULL,
           NULL,
           1,
           0xC9040000, 0x4000
           );
}
