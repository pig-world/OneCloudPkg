/** @file
 *
 *  Copyright (c) 2017 - 2019, Andrey Warkentin <andrey.warkentin@gmail.com>
 *  Copyright (c) 2015 - 2016, Linaro Limited. All rights reserved.
 *
 *  This program and the accompanying materials
 *  are licensed and made available under the terms and conditions of the BSD License
 *  which accompanies this distribution.  The full text of the license may be found at
 *  http://opensource.org/licenses/bsd-license.php
 *
 *  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
 *
 **/

#include "DwcOtgDxe.h"
#include "DwcReg.h"
#include "reg_addr.h"

  /*
   * TimerPeriodic to account for timeout processing
   * within DwHcTransfer.
   */
#define TimerForTransfer TimerRelative

typedef enum {
  XFER_NOT_HALTED,
  XFER_NOT_INIT,
  XFER_ERROR,
  XFER_CSPLIT,
  XFER_NAK,
  XFER_STALL,
  XFER_FRMOVRUN,
  XFER_DONE
} CHANNEL_HALT_REASON;

typedef struct {
  BOOLEAN Splitting;
  BOOLEAN SplitStart;
  UINT32 Tries;
} SPLIT_CONTROL;



#define GPIO_REG_BIT(reg,bit) ((reg<<5)|bit)
#define GPIO_REG(value) ((value>>5))
#define GPIO_BIT(value) ((value&0x1F))

#define PIN_MAP(pin,reg,bit) \
{ \
	.num=pin, \
	.name=#pin, \
	.out_en_reg_bit=GPIO_REG_BIT(reg,bit), \
	.out_value_reg_bit=GPIO_REG_BIT(reg,bit), \
	.input_value_reg_bit=GPIO_REG_BIT(reg,bit), \
}
#define PIN_AOMAP(pin,en_reg,en_bit,out_reg,out_bit,in_reg,in_bit) \
{ \
	.num=pin, \
	.name=#pin, \
	.out_en_reg_bit=GPIO_REG_BIT(en_reg,en_bit), \
	.out_value_reg_bit=GPIO_REG_BIT(out_reg,out_bit), \
	.input_value_reg_bit=GPIO_REG_BIT(in_reg,in_bit), \
	.gpio_owner=NULL, \
}
typedef enum {
	GPIOAO_0=0,
	GPIOAO_1=1,
	GPIOAO_2=2,
	GPIOAO_3=3,
	GPIOAO_4=4,
	GPIOAO_5=5,
	GPIOAO_6=6,
	GPIOAO_7=7,
	GPIOAO_8=8,
	GPIOAO_9=9,
	GPIOAO_10=10,
	GPIOAO_11=11,
	GPIOAO_12=12,
	GPIOAO_13=13,
	GPIOH_0=14,
	GPIOH_1=15,
	GPIOH_2=16,
	GPIOH_3=17,
	GPIOH_4=18,
	GPIOH_5=19,
	GPIOH_6=20,
	GPIOH_7=21,
	GPIOH_8=22,
	GPIOH_9=23,
	BOOT_0=24,
	BOOT_1=25,
	BOOT_2=26,
	BOOT_3=27,
	BOOT_4=28,
	BOOT_5=29,
	BOOT_6=30,
	BOOT_7=31,
	BOOT_8=32,
	BOOT_9=33,
	BOOT_10=34,
	BOOT_11=35,
	BOOT_12=36,
	BOOT_13=37,
	BOOT_14=38,
	BOOT_15=39,
	BOOT_16=40,
	BOOT_17=41,
	BOOT_18=42,
	CARD_0=43,
	CARD_1=44,
	CARD_2=45,
	CARD_3=46,
	CARD_4=47,
	CARD_5=48,
	CARD_6=49,
	GPIODV_0=50,
	GPIODV_1=51,
	GPIODV_2=52,
	GPIODV_3=53,
	GPIODV_4=54,
	GPIODV_5=55,
	GPIODV_6=56,
	GPIODV_7=57,
	GPIODV_8=58,
	GPIODV_9=59,
	GPIODV_10=60,
	GPIODV_11=61,
	GPIODV_12=62,
	GPIODV_13=63,
	GPIODV_14=64,
	GPIODV_15=65,
	GPIODV_16=66,
	GPIODV_17=67,
	GPIODV_18=68,
	GPIODV_19=69,
	GPIODV_20=70,
	GPIODV_21=71,
	GPIODV_22=72,
	GPIODV_23=73,
	GPIODV_24=74,
	GPIODV_25=75,
	GPIODV_26=76,
	GPIODV_27=77,
	GPIODV_28=78,
	GPIODV_29=79,
	GPIOY_0=80,
	GPIOY_1=81,
	GPIOY_2=82,
	GPIOY_3=83,
	GPIOY_4=84,
	GPIOY_5=85,
	GPIOY_6=86,
	GPIOY_7=87,
	GPIOY_8=88,
	GPIOY_9=89,
	GPIOY_10=90,
	GPIOY_11=91,
	GPIOY_12=92,
	GPIOY_13=93,
	GPIOY_14=94,
	GPIOY_15=95,
	GPIOY_16=96,
	GPIOX_0=97,
	GPIOX_1=98,
	GPIOX_2=99,
	GPIOX_3=100,
	GPIOX_4=101,
	GPIOX_5=102,
	GPIOX_6=103,
	GPIOX_7=104,
	GPIOX_8=105,
	GPIOX_9=106,
	GPIOX_10=107,
	GPIOX_11=108,
	GPIOX_12=109,
	GPIOX_13=110,
	GPIOX_14=111,
	GPIOX_15=112,
	GPIOX_16=113,
	GPIOX_17=114,
	GPIOX_18=115,
	GPIOX_19=116,
	GPIOX_20=117,
	GPIOX_21=118,
	DIF_TTL_0_P=119,
	DIF_TTL_0_N=120,
	DIF_TTL_1_P=121,
	DIF_TTL_1_N=122,
	DIF_TTL_2_P=123,
	DIF_TTL_2_N=124,
	DIF_TTL_3_P=125,
	DIF_TTL_3_N=126,
	DIF_TTL_4_P=127,
	DIF_TTL_4_N=128,
	HDMI_TTL_0_P=129,
	HDMI_TTL_0_N=130,
	HDMI_TTL_1_P=131,
	HDMI_TTL_1_N=132,
	HDMI_TTL_2_P=133,
	HDMI_TTL_2_N=134,
	HDMI_TTL_CK_P=135,
	HDMI_TTL_CK_N=136,
	GPIO_BSD_EN=137,
	GPIO_TEST_N=138,
	GPIO_MAX=139,
}gpio_t;
struct amlogic_gpio_desc{
	unsigned num;
	char *name;
	unsigned int out_en_reg_bit;
	unsigned int out_value_reg_bit;
	unsigned int input_value_reg_bit;
	unsigned int map_to_irq;
	unsigned int pull_up_reg_bit;
	const char *gpio_owner;
};
struct amlogic_gpio_desc amlogic_pins[]=
{
	PIN_AOMAP(GPIOAO_0,6,0,6,16,6,0),
	PIN_AOMAP(GPIOAO_1,6,1,6,17,6,1),
	PIN_AOMAP(GPIOAO_2,6,2,6,18,6,2),
	PIN_AOMAP(GPIOAO_3,6,3,6,19,6,3),
	PIN_AOMAP(GPIOAO_4,6,4,6,20,6,4),
	PIN_AOMAP(GPIOAO_5,6,5,6,21,6,5),
	PIN_AOMAP(GPIOAO_6,6,6,6,22,6,6),
	PIN_AOMAP(GPIOAO_7,6,7,6,23,6,7),
	PIN_AOMAP(GPIOAO_8,6,8,6,24,6,8),
	PIN_AOMAP(GPIOAO_9,6,9,6,25,6,9),
	PIN_AOMAP(GPIOAO_10,6,10,6,26,6,10),
	PIN_AOMAP(GPIOAO_11,6,11,6,27,6,11),
	PIN_AOMAP(GPIOAO_12,6,12,6,28,6,12),
	PIN_AOMAP(GPIOAO_13,6,13,6,29,6,13),
	PIN_MAP(GPIOH_0,3,19),
	PIN_MAP(GPIOH_1,3,20),
	PIN_MAP(GPIOH_2,3,21),
	PIN_MAP(GPIOH_3,3,22),
	PIN_MAP(GPIOH_4,3,23),
	PIN_MAP(GPIOH_5,3,24),
	PIN_MAP(GPIOH_6,3,25),
	PIN_MAP(GPIOH_7,3,26),
	PIN_MAP(GPIOH_8,3,27),
	PIN_MAP(GPIOH_9,3,28),
	PIN_MAP(BOOT_0,3,0),
	PIN_MAP(BOOT_1,3,1),
	PIN_MAP(BOOT_2,3,2),
	PIN_MAP(BOOT_3,3,3),
	PIN_MAP(BOOT_4,3,4),
	PIN_MAP(BOOT_5,3,5),
	PIN_MAP(BOOT_6,3,6),
	PIN_MAP(BOOT_7,3,7),
	PIN_MAP(BOOT_8,3,8),
	PIN_MAP(BOOT_9,3,9),
	PIN_MAP(BOOT_10,3,10),
	PIN_MAP(BOOT_11,3,11),
	PIN_MAP(BOOT_12,3,12),
	PIN_MAP(BOOT_13,3,13),
	PIN_MAP(BOOT_14,3,14),
	PIN_MAP(BOOT_15,3,15),
	PIN_MAP(BOOT_16,3,16),
	PIN_MAP(BOOT_17,3,17),
	PIN_MAP(BOOT_18,3,18),
	PIN_MAP(CARD_0,0,22),
	PIN_MAP(CARD_1,0,23),
	PIN_MAP(CARD_2,0,24),
	PIN_MAP(CARD_3,0,25),
	PIN_MAP(CARD_4,0,26),
	PIN_MAP(CARD_5,0,27),
	PIN_MAP(CARD_6,0,28),
	PIN_MAP(GPIODV_0,2,0),
	PIN_MAP(GPIODV_1,2,1),
	PIN_MAP(GPIODV_2,2,2),
	PIN_MAP(GPIODV_3,2,3),
	PIN_MAP(GPIODV_4,2,4),
	PIN_MAP(GPIODV_5,2,5),
	PIN_MAP(GPIODV_6,2,6),
	PIN_MAP(GPIODV_7,2,7),
	PIN_MAP(GPIODV_8,2,8),
	PIN_MAP(GPIODV_9,2,9),
	PIN_MAP(GPIODV_10,2,10),
	PIN_MAP(GPIODV_11,2,11),
	PIN_MAP(GPIODV_12,2,12),
	PIN_MAP(GPIODV_13,2,13),
	PIN_MAP(GPIODV_14,2,14),
	PIN_MAP(GPIODV_15,2,15),
	PIN_MAP(GPIODV_16,2,16),
	PIN_MAP(GPIODV_17,2,17),
	PIN_MAP(GPIODV_18,2,18),
	PIN_MAP(GPIODV_19,2,19),
	PIN_MAP(GPIODV_20,2,20),
	PIN_MAP(GPIODV_21,2,21),
	PIN_MAP(GPIODV_22,2,22),
	PIN_MAP(GPIODV_23,2,23),
	PIN_MAP(GPIODV_24,2,24),
	PIN_MAP(GPIODV_25,2,25),
	PIN_MAP(GPIODV_26,2,26),
	PIN_MAP(GPIODV_27,2,27),
	PIN_MAP(GPIODV_28,2,28),
	PIN_MAP(GPIODV_29,2,29),
	PIN_MAP(GPIOY_0,1,0),
	PIN_MAP(GPIOY_1,1,1),
	PIN_MAP(GPIOY_2,1,2),
	PIN_MAP(GPIOY_3,1,3),
	PIN_MAP(GPIOY_4,1,4),
	PIN_MAP(GPIOY_5,1,5),
	PIN_MAP(GPIOY_6,1,6),
	PIN_MAP(GPIOY_7,1,7),
	PIN_MAP(GPIOY_8,1,8),
	PIN_MAP(GPIOY_9,1,9),
	PIN_MAP(GPIOY_10,1,10),
	PIN_MAP(GPIOY_11,1,11),
	PIN_MAP(GPIOY_12,1,12),
	PIN_MAP(GPIOY_13,1,13),
	PIN_MAP(GPIOY_14,1,14),
	PIN_MAP(GPIOY_15,1,15),
	PIN_MAP(GPIOY_16,1,16),
	PIN_MAP(GPIOX_0,0,0),
	PIN_MAP(GPIOX_1,0,1),
	PIN_MAP(GPIOX_2,0,2),
	PIN_MAP(GPIOX_3,0,3),
	PIN_MAP(GPIOX_4,0,4),
	PIN_MAP(GPIOX_5,0,5),
	PIN_MAP(GPIOX_6,0,6),
	PIN_MAP(GPIOX_7,0,7),
	PIN_MAP(GPIOX_8,0,8),
	PIN_MAP(GPIOX_9,0,9),
	PIN_MAP(GPIOX_10,0,10),
	PIN_MAP(GPIOX_11,0,11),
	PIN_MAP(GPIOX_12,0,12),
	PIN_MAP(GPIOX_13,0,13),
	PIN_MAP(GPIOX_14,0,14),
	PIN_MAP(GPIOX_15,0,15),
	PIN_MAP(GPIOX_16,0,16),
	PIN_MAP(GPIOX_17,0,17),
	PIN_MAP(GPIOX_18,0,18),
	PIN_MAP(GPIOX_19,0,19),
	PIN_MAP(GPIOX_20,0,20),
	PIN_MAP(GPIOX_21,0,21),
	PIN_MAP(DIF_TTL_0_P,4,12),
	PIN_MAP(DIF_TTL_0_N,4,13),
	PIN_MAP(DIF_TTL_1_P,4,14),
	PIN_MAP(DIF_TTL_1_N,4,15),
	PIN_MAP(DIF_TTL_2_P,4,16),
	PIN_MAP(DIF_TTL_2_N,4,17),
	PIN_MAP(DIF_TTL_3_P,4,18),
	PIN_MAP(DIF_TTL_3_N,4,19),
	PIN_MAP(DIF_TTL_4_P,4,20),
	PIN_MAP(DIF_TTL_4_N,4,21),
	PIN_MAP(HDMI_TTL_0_P,4,22),
	PIN_MAP(HDMI_TTL_0_N,4,23),
	PIN_MAP(HDMI_TTL_1_P,4,24),
	PIN_MAP(HDMI_TTL_1_N,4,25),
	PIN_MAP(HDMI_TTL_2_P,4,26),
	PIN_MAP(HDMI_TTL_2_N,4,27),
	PIN_MAP(HDMI_TTL_CK_P,4,28),
	PIN_MAP(HDMI_TTL_CK_N,4,29),
	PIN_AOMAP(GPIO_BSD_EN,0,30,0,31,0,0x1f),
	PIN_AOMAP(GPIO_TEST_N,0,0,4,31,0,0),
};

static unsigned p_gpio_output_addr[]={
	P_PREG_PAD_GPIO0_O,
	P_PREG_PAD_GPIO1_O,
	P_PREG_PAD_GPIO2_O,
	P_PREG_PAD_GPIO3_O,
	P_PREG_PAD_GPIO4_O,
	P_PREG_PAD_GPIO5_O,
	P_AO_GPIO_O_EN_N,
};


VOID
DwcOtgSetVbusPower(
  IN  DWUSB_OTGHC_DEV *DwHc, 
  IN  BOOLEAN IsPowerOn
  )
{
  UINT8 Pin=15;
  if(IsPowerOn)
  {
    //DEBUG((DEBUG_INFO,"DwcOtgSetVbusPower: Set usb port power on (board gpio %d)!\n",Pin));
    UINT32 reg=GPIO_REG(amlogic_pins[Pin].out_value_reg_bit);
	UINT8 bit=GPIO_BIT(amlogic_pins[Pin].out_value_reg_bit);
	MmioOr32(p_gpio_output_addr[reg],1<<bit);
  }
  else
  {
    //DEBUG((DEBUG_INFO,"DwcOtgSetVbusPower: Set usb port power off (board gpio %d)!\n",Pin));
    UINT32 reg=GPIO_REG(amlogic_pins[Pin].out_value_reg_bit);
	UINT8 bit=GPIO_BIT(amlogic_pins[Pin].out_value_reg_bit);
	MmioOr32(p_gpio_output_addr[reg],1<<bit);
  }
}


EFI_STATUS DwcOtgCilInit (IN DWUSB_OTGHC_DEV *DwHc);
EFI_STATUS DwcOtgCoreInit (IN DWUSB_OTGHC_DEV *DwHc);

EFI_STATUS
Wait4Bit (
  IN  EFI_EVENT Timeout,
  IN  UINT32    Reg,
  IN  UINT32    Mask,
  IN  BOOLEAN   Set
  )
{
  UINT32 Value;

  do {
    Value = MmioRead32 (Reg);
    if (!Set) {
      Value = ~Value;
    }

    if ((Value & Mask) == Mask) {
      return EFI_SUCCESS;
    }
  } while (EFI_ERROR (gBS->CheckEvent (Timeout)));

  return EFI_TIMEOUT;
}

CHANNEL_HALT_REASON
DwcOtgInterrupt (
  IN  DWUSB_OTGHC_DEV *DwHc,
  IN  EFI_EVENT       Timeout,
  IN  UINT32          Channel,
  IN  UINT32          *Sub,
  IN  UINT32          *Toggle,
  IN  BOOLEAN         IgnoreAck,
  IN  SPLIT_CONTROL   *Split
  )
{
  EFI_STATUS Status;
  UINT32  Hcint, Hctsiz, Gintsts;
  UINT32  HcintCompHltAck = DWC2_HCINT_XFERCOMP;
  
  MicroSecondDelay (100);
  Status  = Wait4Bit (Timeout, DwHc->DwUsbBase + HCINT(Channel), DWC2_HCINT_CHHLTD, 1);
  if (EFI_ERROR (Status)) {
	DEBUG ((DEBUG_ERROR, "DwcOtgInterrupt: XFER_NOT_HALTED\n"));
    return XFER_NOT_HALTED;
  }

  MicroSecondDelay (100);
  /* Read Global Interrupts Register */
  Gintsts = MmioRead32(DwHc->DwUsbBase + GINTSTS);
  /* Read Channel Interrupts Register */
  Hcint = MmioRead32 (DwHc->DwUsbBase + HCINT(Channel));
  
  if ((Gintsts & DWC2_GINTSTS_PORTINTR) >> DWC2_GINTSTS_PORTINTR_OFFSET)
  {
    DEBUG((DEBUG_INFO,"DwcOtgInterrupt: Port Status Changed\n"));
    return XFER_NOT_INIT;
  }

  ASSERT ((Hcint & DWC2_HCINT_CHHLTD) != 0);
  Hcint &= ~DWC2_HCINT_CHHLTD;

  if (!IgnoreAck ||
      (Split->Splitting && Split->SplitStart)) {
    HcintCompHltAck |= DWC2_HCINT_ACK;
  } else {
    Hcint &= ~DWC2_HCINT_ACK;
  }

  if ((Hcint & DWC2_HCINT_XACTERR) != 0) {
	DEBUG ((DEBUG_ERROR, "DwcOtgInterrupt: XFER_ERROR\n"));
    return XFER_ERROR;
  }

  if ((Hcint & DWC2_HCINT_NYET) != 0) {
	DEBUG ((DEBUG_ERROR, "DwcOtgInterrupt: XFER_CSPLIT\n"));
    return XFER_CSPLIT;
  }

  if ((Hcint & DWC2_HCINT_NAK) != 0) {
	DEBUG ((DEBUG_ERROR, "DwcOtgInterrupt: DWC2_HCINT_NAK\n"));
    return XFER_NAK;
  }

  if ((Hcint & DWC2_HCINT_STALL) != 0) {
	DEBUG ((DEBUG_ERROR, "DwcOtgInterrupt: DWC2_HCINT_STALL\n"));
    return XFER_STALL;
  }

  if ((Hcint & DWC2_HCINT_FRMOVRUN) != 0) {
	DEBUG ((DEBUG_ERROR, "DwcOtgInterrupt: DWC2_HCINT_FRMOVRUN\n"));
    return XFER_FRMOVRUN;
  }

  if (Split->Splitting &&
      Split->SplitStart &&
      ((Hcint & DWC2_HCINT_ACK) != 0)) {
    Split->SplitStart = FALSE;
    Split->Tries = 0;
	DEBUG ((DEBUG_ERROR, "DwcOtgInterrupt: XFER_CSPLIT\n"));
    return XFER_CSPLIT;
  }

  if (Hcint != HcintCompHltAck) {
    // DEBUG ((DEBUG_ERROR, "DwcOtgInterrupt: Channel %u HCINT 0x%x %a%a\n",
            // Channel, Hcint,
            // IgnoreAck ? "IgnoreAck " : "",
            // Split->SplitStart ? "split start" :
            // (Split->Splitting ? "split complete" : "")));
    return XFER_ERROR;
  }

  Hctsiz = MmioRead32 (DwHc->DwUsbBase + HCTSIZ(Channel));
  *Sub = (Hctsiz & DWC2_HCTSIZ_XFERSIZE_MASK) >> DWC2_HCTSIZ_XFERSIZE_OFFSET;
  *Toggle = (Hctsiz & DWC2_HCTSIZ_PID_MASK) >> DWC2_HCTSIZ_PID_OFFSET;

  return XFER_DONE;
}

/**
 * This function initializes the commmon interrupts, used in both
 * device and host modes.
 *
 * @param[in] _core_if Programming view of the DWC_otg controller
 *
 */
VOID
DwcOtgEnableCommonInterrupts(
    IN DWUSB_OTGHC_DEV *DwHc
	)
{
  //DEBUG((DEBUG_INFO, "DwcOtgEnableCommonInterrupts\n"));
  UINT32 intr_mask = MmioRead32(DwHc->DwUsbBase + GINTMSK);

  /* Clear any pending OTG Interrupts */
  MmioWrite32(DwHc->DwUsbBase + GOTGINT, 0xFFFFFFFF);

  /* Clear any pending interrupts */
  MmioWrite32(DwHc->DwUsbBase + GINTSTS, 0xFFFFFFFF);

  /* Enable the interrupts in the GINTMSK */
  intr_mask |= DWC2_GINTMSK_MODEMISMATCH;
  intr_mask |= DWC2_GINTMSK_OTGINTR;
  //intr_mask |= DWC2_GINTMSK_RXSTSQLVL; if DMA enabled turn this off
  intr_mask |= DWC2_GINTMSK_CONIDSTSCHNG;
  intr_mask |= DWC2_GINTMSK_WKUPINTR;
  intr_mask |= DWC2_GINTMSK_DISCONNECT;
  intr_mask |= DWC2_GINTMSK_USBSUSPEND;
  intr_mask |= DWC2_GINTMSK_SESSREQINTR;
  MmioWrite32(DwHc->DwUsbBase + GINTMSK, intr_mask);
}

/** 
 * This function enables the Host mode interrupts.
 *
 * @param _core_if Programming view of DWC_otg controller
 */
VOID
DwcOtgEnableHostInterrupts(
	IN  DWUSB_OTGHC_DEV    *DwHc
  )
{
  //DEBUG((DEBUG_INFO, "DwcOtgEnableHostInterrupts\n"));

  /* Disable all interrupts */
  MmioWrite32(DwHc->DwUsbBase + GINTMSK, 0);

  /* Clear any pending interrupts */
  MmioWrite32(DwHc->DwUsbBase + GINTSTS, 0xFFFFFFFF);

  /* Enable the common interrupts */
  DwcOtgEnableCommonInterrupts(DwHc);

  /* Enable host mode interrupts without disturbing common interrupts */
  MmioAndThenOr32(DwHc->DwUsbBase + GINTMSK,
                  (~DWC2_GINTMSK_PORTINTR) & (~DWC2_GINTMSK_HCINTR),
                  DWC2_GINTMSK_PORTINTR | DWC2_GINTMSK_HCINTR);
}

/**
 * This function disables the controller's Global Interrupt in the AHB Config
 * register.
 *
 * @param[in] _core_if Programming view of DWC_otg controller.
 */
VOID
DwcOtgDisableGlobalInterrupts(
  IN  DWUSB_OTGHC_DEV    *DwHc
  )
{
    /* Set 0 to disable interrupts */
    MmioAnd32(DwHc->DwUsbBase+GAHBCFG, ~DWC2_GAHBCFG_GLBLINTRMSK);
}

/**
 * This function enables the controller's Global Interrupt in the AHB Config
 * register.
 *
 * @param[in] _core_if Programming view of DWC_otg controller.
 */
VOID
DwcOtgEnableGlobalInterrupts(  
  IN  DWUSB_OTGHC_DEV    *DwHc
  )
{
    /* Set 1 to Enable interrupts */
    MmioOr32(DwHc->DwUsbBase+GAHBCFG, DWC2_GAHBCFG_GLBLINTRMSK);
}

/**
 * This function Reads HPRT0 in preparation to modify.  It keeps the
 * WC bits 0 so that if they are read as 1, they won't clear when you
 * write it back 
 */
UINT32
DwcOtgReadHprt0(
  IN  DWUSB_OTGHC_DEV    *DwHc
  )
{
    UINT32  hprt0 = MmioRead32(DwHc->DwUsbBase+HPRT0);
    hprt0&= ~(DWC2_HPRT0_PRTENA | DWC2_HPRT0_PRTCONNDET | DWC2_HPRT0_PRTENCHNG | DWC2_HPRT0_PRTOVRCURRCHNG);
    return hprt0;
}

/**
 * Initializes the FSLSPClkSel field of the HCFG register depending on the PHY
 * type.
 */
VOID
InitFSLSPClkSel(
  IN  DWUSB_OTGHC_DEV *DwHc,
  IN  INT32 Type
  )
{
  /* Clear FSLSPCLKSEL bit */
  MmioAnd32(DwHc->DwUsbBase + HCFG, DWC2_HCFG_FSLSPCLKSEL_MASK);
  if (Type == DWC2_PHY_TYPE_FS)
  {
    /* Full speed PHY */
    MmioOr32(DwHc->DwUsbBase + HCFG,
             DWC2_HCFG_FSLSPCLKSEL_48_MHZ << DWC2_HCFG_FSLSPCLKSEL_OFFSET);
  }
  else
  {
    /* High speed PHY running at full speed or high speed */
    MmioOr32(DwHc->DwUsbBase + HCFG,
             DWC2_HCFG_FSLSPCLKSEL_30_60_MHZ << DWC2_HCFG_FSLSPCLKSEL_OFFSET);
  }
}

VOID
DwcOtgFlushTxFifo (
  IN  DWUSB_OTGHC_DEV *DwHc,
  IN INT32 Num
  )
{
  INT32		Greset = 0;
  INT32		Count = 0;
  
  //DEBUG((DEBUG_INFO, "DwcOtgFlushTxFifo, Num=%d\n", Num));
  
  MmioAndThenOr32(DwHc->DwUsbBase + GRSTCTL, ~DWC2_GRSTCTL_TXFNUM_MASK, Num << DWC2_GRSTCTL_TXFNUM_OFFSET | DWC2_GRSTCTL_TXFFLSH);
  
  do {
      Greset = MmioRead32(DwHc->DwUsbBase + GRSTCTL);
      if (++Count > 10000) {
          //DEBUG((DEBUG_INFO,"HANG! DwcOtgFlushTxFifo"));
          break;
      }
  
  } while ((Greset & DWC2_GRSTCTL_TXFFLSH) >> DWC2_GRSTCTL_TXFFLSH_OFFSET);
  /* Wait for 3 PHY Clocks */
  MicroSecondDelay (1);
}

VOID
DwcOtgFlushRxFifo (
  IN  DWUSB_OTGHC_DEV *DwHc
  )
{
  INT32	Greset = 0;
  INT32	Count = 0;

  //DEBUG((DEBUG_INFO, "DwcOtgFlushRxFifo\n"));

  MmioOr32(DwHc->DwUsbBase + GRSTCTL, DWC2_GRSTCTL_TXFFLSH);

  do {
      Greset = MmioRead32(DwHc->DwUsbBase + GRSTCTL);
      if (++Count > 10000) {
           //DEBUG((DEBUG_INFO,"HANG! DwcOtgFlushRxFifo", Greset));
          break;
      }
  } while ((Greset & DWC2_GRSTCTL_TXFFLSH) >> DWC2_GRSTCTL_TXFFLSH_OFFSET);
  /*
   * Wait for 3 PHY Clocks
   */
  MicroSecondDelay (1);
}

VOID
DwcOtgSetForceId(
  	IN  DWUSB_OTGHC_DEV    *DwHc,
    IN  INT8 Mode
    )
{
	UINT32 gUsbCfg_data;

	gUsbCfg_data = MmioRead32(DwHc->DwUsbBase + GUSBCFG);
	switch(Mode){
		case FORCE_ID_CLEAR:
			//DEBUG((DEBUG_INFO,"Force id mode: Hardware\n"));
			gUsbCfg_data &= (~DWC2_GUSBCFG_FORCEHOSTMODE);
			gUsbCfg_data &= (~DWC2_GUSBCFG_FORCEDEVMODE);
			break;
		case FORCE_ID_HOST:
			//DEBUG((DEBUG_INFO,"Force id mode: Host\n"));
			gUsbCfg_data |= DWC2_GUSBCFG_FORCEHOSTMODE;
			gUsbCfg_data &= (~DWC2_GUSBCFG_FORCEDEVMODE);
			break;
		case FORCE_ID_SLAVE:
			//DEBUG((DEBUG_INFO,"Force id mode: Slave\n"));
			gUsbCfg_data &= (~DWC2_GUSBCFG_FORCEHOSTMODE);
			gUsbCfg_data |= DWC2_GUSBCFG_FORCEDEVMODE;
			break;
		default:
			//DEBUG((DEBUG_ERROR,"DwcOtgSetForceId: Error Id Mode\n"));
			return;
	}
	MmioWrite32(DwHc->DwUsbBase+GUSBCFG,gUsbCfg_data);
}

VOID
DwcOtgHcCleanup (
  IN  DWUSB_OTGHC_DEV    *DwHc,
  IN  UINT8              HcNum      
  )
{
  //DEBUG((DEBUG_INFO, "DwcOtgHcCleanup\n"));

  /* Clear channel interrupt enables and any unhandled channel interrupt conditions */
  MmioWrite32(DwHc->DwUsbBase + HCINTMSK(HcNum), 0);
  MmioWrite32(DwHc->DwUsbBase + HCINT(HcNum), 0xFFFFFFFF);
  MmioWrite32(DwHc->DwUsbBase + HCTSIZ(HcNum), 0);
  MmioWrite32(DwHc->DwUsbBase + HCDMA(HcNum), 0);
  MmioWrite32(DwHc->DwUsbBase + HCCHAR(HcNum), 0);
  
  ArmDataSynchronizationBarrier();
}

/*
	dwc_otg_core_reset (dwc_otg_core_if_t *core_if)
 	Do core a soft reset of the core.
*/
EFI_STATUS
DwcOtgCoreReset (
  IN  DWUSB_OTGHC_DEV *DwHc
  )
{
  INT32 Greset = 0;
  INT32 Count = 0;
  //DEBUG((DEBUG_INFO, "DwcOtgCoreReset\n"));
  /* Wait for AHB master IDLE state */
  do
  {
    MicroSecondDelay(10);
    Greset = MmioRead32(DwHc->DwUsbBase + GRSTCTL);
	//DEBUG((DEBUG_INFO, "AHB Idle GRSTCTL=%0x\n", Greset));
    if (++Count > 1000)
    {
      //DEBUG((DEBUG_ERROR, "DwcOtgCoreReset HANG! AHB Idle GRSTCTL=%0x\n", Greset));
      return EFI_DEVICE_ERROR;
    }
  } while ((Greset & DWC2_GRSTCTL_AHBIDLE) >> DWC2_GRSTCTL_AHBIDLE_OFFSET == 0);

  /* Set csftrst 1 to Soft Reset Core */
  Count = 0;
  MmioOr32(DwHc->DwUsbBase + GRSTCTL, DWC2_GRSTCTL_CSFTRST);
  do
  {
    MicroSecondDelay(10);
    Greset = MmioRead32(DwHc->DwUsbBase + GRSTCTL);
	//DEBUG((DEBUG_INFO, "AHB Idle GRSTCTL=%0x\n", Greset));
    if (++Count > 1000)
    {
      //DEBUG((DEBUG_ERROR, "DwcOtgCoreReset HANG! Soft Reset GRSTCTL=%0x\n", Greset));
      break;
    }
  } while ((Greset & DWC2_GRSTCTL_CSFTRST) >> DWC2_GRSTCTL_CSFTRST_OFFSET);

  /* Wait for 3 PHY Clocks */
  MicroSecondDelay(100);
  return EFI_SUCCESS;
}

/**
 * This function initializes the DWC_otg controller registers for
 * host mode.
 *
 * This function flushes the Tx and Rx FIFOs and it flushes any entries in the
 * request queues. Host channels are reset to ensure that they are ready for
 * performing transfers.
 *
 * @param _core_if Programming view of DWC_otg controller
 *
 */
STATIC
VOID
DwcOtgCoreHostInit(
    IN  DWUSB_OTGHC_DEV *DwHc,
	IN  UINT8 Speed
	)
{
  UINT32 Hcchar;
  UINT8  OpState = A_HOST;

  //DEBUG((DEBUG_INFO, "DwcOtgCoreHostInit\n"));
  /* Restart the Phy Clock */
  MmioWrite32(DwHc->DwUsbBase + PCGCCTL, 0);

  /* Initialize Host Configuration Register */
  InitFSLSPClkSel(DwHc, DWC_PHY_TYPE_PARAM_UTMI);

  if (Speed == DWC_SPEED_PARAM_FULL)
  {
    MmioOr32(DwHc->DwUsbBase + HCFG, DWC2_HCFG_FSLSSUPP);
  }

  /* Configure data FIFO sizes 6.1.1.11----6.1.1.13 */
  if (MmioRead32(DwHc->DwUsbBase + GHWCFG2) & DWC2_HWCFG2_DYNAMIC_FIFO)
  {
    //DEBUG((DEBUG_INFO, "Total FIFO Size=%d\n", (MmioRead32(DwHc->DwUsbBase + GHWCFG3) & DWC2_FIFOSIZE_DEPTH_MASK) >> DWC2_FIFOSIZE_DEPTH_OFFSET));
    //DEBUG((DEBUG_INFO, "Rx FIFO Size=%d\n", 512));    //params->host_rx_fifo_size
    //DEBUG((DEBUG_INFO, "NP Tx FIFO Size=%d\n", 500)); //params->host_nperio_tx_fifo_size
    //DEBUG((DEBUG_INFO, "P Tx FIFO Size=%d\n", 100));  //params->host_perio_tx_fifo_size

    /* Rx FIFO */
    //DEBUG((DEBUG_INFO, "initial grxfsiz=%08x\n", MmioRead32(DwHc->DwUsbBase + GRXFSIZ)));
    MmioWrite32(DwHc->DwUsbBase + GRXFSIZ, 512); //params->host_rx_fifo_size
    //DEBUG((DEBUG_INFO, "new grxfsiz=%08x\n", MmioRead32(DwHc->DwUsbBase + GRXFSIZ)));

    /* Non-periodic Tx FIFO */
    //DEBUG((DEBUG_INFO, "initial gnptxfsiz=%08x\n", MmioRead32(DwHc->DwUsbBase + GNPTXFSIZ)));
    MmioAndThenOr32(DwHc->DwUsbBase + GNPTXFSIZ, (~DWC2_FIFOSIZE_DEPTH_MASK) | (~DWC2_FIFOSIZE_STARTADDR_OFFSET), (500 << DWC2_FIFOSIZE_DEPTH_OFFSET) | (512 << DWC2_FIFOSIZE_STARTADDR_OFFSET));
    //DEBUG((DEBUG_INFO, "new gnptxfsiz=%08x\n", MmioRead32(DwHc->DwUsbBase + GNPTXFSIZ)));

    /* Periodic Tx FIFO */
    //DEBUG((DEBUG_INFO, "initial hptxfsiz=%08x\n", MmioRead32(DwHc->DwUsbBase + HPTXFSIZ)));
    MmioAndThenOr32(DwHc->DwUsbBase + HPTXFSIZ, (~DWC2_FIFOSIZE_DEPTH_MASK) | (~DWC2_FIFOSIZE_STARTADDR_MASK), 100 << DWC2_FIFOSIZE_DEPTH_OFFSET | ((500 + 512) << (DWC2_FIFOSIZE_STARTADDR_OFFSET)));
    //DEBUG((DEBUG_INFO, "new hptxfsiz=%08x\n", MmioRead32(DwHc->DwUsbBase + HPTXFSIZ)));
  }

  /* Clear Host Set HNP Enable in the OTG Control Register */
  MmioAnd32(DwHc->DwUsbBase + GOTGCTL, ~DWC2_GOTGCTL_HSTSETHNPEN);

  /* Make sure the FIFOs are flushed */
  DwcOtgFlushTxFifo(DwHc, 0x10);
  DwcOtgFlushRxFifo(DwHc);

  /* Flush out any leftover queued requests */
  for (UINT8 i = 0; i < MAX_EPS_CHANNELS; i++)
  {
    MmioAndThenOr32(DwHc->DwUsbBase + HCCHAR(i),
                    (~DWC2_HCCHAR_CHEN) & (~DWC2_HCCHAR_EPDIR),
                    DWC2_HCCHAR_CHDIS);
  }

  /* Halt all channels to put them into a known state */
  for (UINT8 i = 0; i < MAX_EPS_CHANNELS; i++)
  {
    UINT8 count = 0;
    MmioAndThenOr32(DwHc->DwUsbBase + HCCHAR(i), ~DWC2_HCCHAR_EPDIR, DWC2_HCCHAR_CHEN | DWC2_HCCHAR_CHDIS);
    //DEBUG((DEBUG_INFO, "DwcOtgCoreHostInit: Halt channel %d\n", i));
    do
    {
      Hcchar = MmioRead32(DwHc->DwUsbBase + HCCHAR(i));
      if (++count > 100)
      {
        //DEBUG((DEBUG_ERROR, "DwcOtgCoreHostInit: Unable to clear halt on channel %d\n", i));
        break;
      }
    } while ((Hcchar && DWC2_HCCHAR_CHEN) >> DWC2_HCCHAR_CHEN_OFFSET);
  }

  /* Turn on the vbus power */
  //DEBUG((DEBUG_INFO, "DwcOtgCoreHostInit: Port Power? op_state=%d\n", OpState));
  if (OpState == A_HOST)
  {
    UINT32 hprt0 = DwcOtgReadHprt0(DwHc);
    //DEBUG((DEBUG_INFO, "DwcOtgCoreHostInit: Power Port (%d)\n", (hprt0 & DWC2_HPRT0_PRTPWR) >> DWC2_HPRT0_PRTPWR_OFFSET));
    if (((hprt0 & DWC2_HPRT0_PRTPWR) >> DWC2_HPRT0_PRTPWR_OFFSET) == 0)
    {
      MmioOr32(DwHc->DwUsbBase + HPRT0, DWC2_HPRT0_PRTPWR);
      DwcOtgSetVbusPower(DwHc, TRUE); // Power on VBus
    }
  }
  else
  {
    DwcOtgSetVbusPower(DwHc, FALSE); // Power off VBus
  }
  DwcOtgEnableHostInterrupts(DwHc);
}

VOID
DwcOtgResetPort(
  IN  DWUSB_OTGHC_DEV *DwHc
  )
{
    DEBUG ((DEBUG_INFO, "DwcOtgResetPort\n"));
    
    MmioAndThenOr32 (DwHc->DwUsbBase + HPRT0,
                     ~(DWC2_HPRT0_PRTENA | DWC2_HPRT0_PRTCONNDET |
                       DWC2_HPRT0_PRTENCHNG | DWC2_HPRT0_PRTOVRCURRCHNG),
                     DWC2_HPRT0_PRTRST);
    MicroSecondDelay (50000);
    MmioAnd32 (DwHc->DwUsbBase + HPRT0, ~DWC2_HPRT0_PRTRST);
	MicroSecondDelay (50000);
}

EFI_STATUS
DwcOtgPortInit(
    IN DWUSB_OTGHC_DEV *DwHc
    )
{

  UINT32 hprt0;
  UINT8 PortSpeed;
  for (UINT8 i = 0; i < 10; i++)
  {
    hprt0 = MmioRead32(DwHc->DwUsbBase + HPRT0);
    //DEBUG((DEBUG_INFO, "DwcOtgPortInit: Port data is 0x%X\n", hprt0));
    PortSpeed = (hprt0 & DWC2_HPRT0_PRTSPD_MASK) >> DWC2_HPRT0_PRTSPD_OFFSET;

    /* wait prtconndet interrupt */
    if ((hprt0 & DWC2_HPRT0_PRTCONNDET) >> DWC2_HPRT0_PRTCONNDET_OFFSET)
    {
      /* clear detect intr */
      MicroSecondDelay(30);
      MmioOr32(DwHc->DwUsbBase + HPRT0, DWC2_HPRT0_PRTCONNDET);
      MicroSecondDelay(30);

      /* reset port  6.1.1.6 */
      DwcOtgResetPort(DwHc);
    }
	else
	{
		MicroSecondDelay(50000);
		continue;
	}
  
	if ((hprt0 & DWC2_HPRT0_PRTENCHNG) >> DWC2_HPRT0_PRTENCHNG_OFFSET)
	{
		//DEBUG((DEBUG_INFO, "DwcOtgPortInit: Port Enable Changed\n"));
		/* Port Enable Changed
		 * Clear if detected - Set internal flag if disabled */

		/* Port has been enabled set the reset change flag */
		MmioAndThenOr32(DwHc->DwUsbBase + HPRT0, ~(DWC2_HPRT0_PRTENA | DWC2_HPRT0_PRTCONNDET | DWC2_HPRT0_PRTOVRCURRCHNG), DWC2_HPRT0_PRTENCHNG);
	}
	
	/* Determine if the connected device is a high/full/low speed device */
    switch (PortSpeed)
    {
      case DWC_HPRT0_PRTSPD_LOW_SPEED:
        //DEBUG((DEBUG_INFO, "Lowspeed device found !\n"));
        break;
      case DWC_HPRT0_PRTSPD_FULL_SPEED:
        //DEBUG((DEBUG_INFO, "Fullspeed device found !\n"));
        break;
      case DWC_HPRT0_PRTSPD_HIGH_SPEED:
        //DEBUG((DEBUG_INFO, "Highspeed device found !\n"));
        break;
    }
	
	return EFI_SUCCESS;
  }

  //DEBUG((DEBUG_INFO, "No USB device found\n"));
  return EFI_NOT_FOUND;
}
VOID
DwOtgHcInit(
      IN DWUSB_OTGHC_DEV * DwHc,
      IN UINT8 HcNum,
      IN EFI_USB2_HC_TRANSACTION_TRANSLATOR * Translator,
      IN UINT8 DeviceSpeed,
      IN UINT8 DevAddr,
      IN UINT8 Endpoint,
      IN UINT8 EpDir,
      IN UINT8 EpType,
      IN UINT16 MaxPacket,
      IN SPLIT_CONTROL * SplitControl)
{
    UINT32 Split = 0;
    UINT32 hc_intr_mask = 0;
    UINT32 Hcchar = (DevAddr << DWC2_HCCHAR_DEVADDR_OFFSET) |
                    (Endpoint << DWC2_HCCHAR_EPNUM_OFFSET) |
                    (EpDir << DWC2_HCCHAR_EPDIR_OFFSET) |
                    (EpType << DWC2_HCCHAR_EPTYPE_OFFSET) |
                    (MaxPacket << DWC2_HCCHAR_MPS_OFFSET) |
                    ((DeviceSpeed == EFI_USB_SPEED_LOW) ? DWC2_HCCHAR_LSPDDEV : 0);

    /* set hc interrupt mask */

    /* Clear old interrupt conditions for this host channel. */
    MmioWrite32(DwHc->DwUsbBase + HCINT(HcNum), 0x3FFF);

    /* Enable channel interrupts required for this transfer. */
    hc_intr_mask |= DWC2_HCINT_CHHLTD;
    hc_intr_mask |= DWC2_HCINT_XFERCOMP;
    hc_intr_mask |= DWC2_HCINT_AHBERR;
    MmioWrite32(DwHc->DwUsbBase + HCINTMSK(HcNum), hc_intr_mask);

    /* Enable the top level host channel interrupt. */
    MmioOr32(DwHc->DwUsbBase + HAINTMSK, (1 << HcNum));

    /* Make sure host channel interrupts are enabled. */
    MmioOr32(DwHc->DwUsbBase + GINTMSK, DWC2_GINTMSK_HCINTR);

    MmioWrite32(DwHc->DwUsbBase + HCCHAR(HcNum), Hcchar);

#define DWC_HCSPLIT_XACTPOS_MID 0
#define DWC_HCSPLIT_XACTPOS_END 1
#define DWC_HCSPLIT_XACTPOS_BEGIN 2
#define DWC_HCSPLIT_XACTPOS_ALL 3

  if (SplitControl->Splitting) {
    Split = DWC2_HCSPLT_SPLTENA |
      ((Translator->TranslatorPortNumber) << DWC2_HCSPLT_PRTADDR_OFFSET) |
      ((Translator->TranslatorHubAddress) << DWC2_HCSPLT_HUBADDR_OFFSET);

    if (!SplitControl->SplitStart) {
      Split |= DWC2_HCSPLT_COMPSPLT;
    }
	Split |= DWC_HCSPLIT_XACTPOS_ALL << DWC2_HCSPLT_XACTPOS_OFFSET;
  }

  MmioWrite32 (DwHc->DwUsbBase + HCSPLT(HcNum), Split);
}

STATIC
EFI_STATUS
DwHcTransfer (
  IN      DWUSB_OTGHC_DEV        *DwHc,
  IN      EFI_EVENT              Timeout,
  IN      UINT8                  Channel,
  IN      EFI_USB2_HC_TRANSACTION_TRANSLATOR *Translator,
  IN      UINT8                  DeviceSpeed,
  IN      UINT8                  DeviceAddress,
  IN      UINTN                  MaximumPacketLength,
  IN  OUT UINT32                 *Pid,
  IN      UINT32                 TransferDirection,
  IN  OUT VOID                   *Data,
  IN  OUT UINTN                  *DataLength,
  IN      UINT32                 EpAddress,
  IN      UINT32                 EpType,
  OUT     UINT32                 *TransferResult,
  IN      BOOLEAN                IgnoreAck
  )
{
  UINT32                          TxferLen;
  UINT32                          Done = 0;
  UINT32                          NumPackets;
  UINT32                          Sub;
  UINT32                          Ret = 0;
  BOOLEAN                         StopTransfer = FALSE;
  EFI_STATUS                      Status = EFI_SUCCESS;
  SPLIT_CONTROL                   Split = { 0 };

  EFI_TPL Tpl = gBS->RaiseTPL (TPL_NOTIFY);

  *TransferResult = EFI_USB_NOERROR;

  DEBUG((DEBUG_INFO,"DwHcTransfer: DeviceAddress=%d, EpAddress=%d, TransferDirection=%d, DataLength=%d, MaximumPacketLength=%d, EpType=%d\n", 
													DeviceAddress,EpAddress,TransferDirection,*DataLength,MaximumPacketLength,EpType));
  
  do {
  restart_xfer:
    if (DeviceSpeed == EFI_USB_SPEED_LOW ||
        DeviceSpeed == EFI_USB_SPEED_FULL) {
      Split.Splitting = TRUE;
      Split.SplitStart = TRUE;
      Split.Tries = 0;
    }

    TxferLen = *DataLength - Done;

    if (TxferLen > DWC2_MAX_TRANSFER_SIZE) {
      TxferLen = DWC2_MAX_TRANSFER_SIZE - MaximumPacketLength + 1;
    }

    if (TxferLen > DWC2_DATA_BUF_SIZE) {
      TxferLen = DWC2_DATA_BUF_SIZE - MaximumPacketLength + 1;
    }

    if (Split.Splitting || TxferLen == 0) {
      NumPackets = 1;
    } else {
      NumPackets = (TxferLen + MaximumPacketLength - 1) / MaximumPacketLength;
      if (NumPackets > DWC2_MAX_PACKET_COUNT) {
        NumPackets = DWC2_MAX_PACKET_COUNT;
      }
    }

    if (!TransferDirection) { // Out
      CopyMem (DwHc->AlignedBuffer, Data+Done, TxferLen);
      ArmDataSynchronizationBarrier();
    }
	
	DEBUG((DEBUG_INFO,"DwHcTransfer: Split.Splitting=%d, NumPackets=%d, TxferLen=%d,DwHc->AlignedBuffer=0x%x\n", 
													Split.Splitting,NumPackets,TxferLen,DwHc->AlignedBuffer));
restart_channel:

    DwOtgHcInit (DwHc, Channel, Translator, DeviceSpeed,
                 DeviceAddress, EpAddress,
                 TransferDirection, EpType,
                 MaximumPacketLength, &Split);
				 
    MmioWrite32 (DwHc->DwUsbBase + HCDMA(Channel),
                 (UINTN)DwHc->AlignedBufferBusAddress);
				 
	ArmDataSynchronizationBarrier();

    MmioAndThenOr32 (DwHc->DwUsbBase + HCTSIZ(Channel),
					~(DWC2_HCTSIZ_DOPNG |
					DWC2_HCTSIZ_XFERSIZE_MASK |
					DWC2_HCTSIZ_PKTCNT_MASK |
					DWC2_HCTSIZ_PID_MASK),
					(TxferLen << DWC2_HCTSIZ_XFERSIZE_OFFSET) |
					(NumPackets << DWC2_HCTSIZ_PKTCNT_OFFSET) |
					(*Pid << DWC2_HCTSIZ_PID_OFFSET));

    MmioAndThenOr32 (DwHc->DwUsbBase + HCCHAR(Channel),
                     ~(DWC2_HCCHAR_MULTICNT_MASK |
                       DWC2_HCCHAR_CHEN |
                       DWC2_HCCHAR_CHDIS),
                     ((1 << DWC2_HCCHAR_MULTICNT_OFFSET) |
                      DWC2_HCCHAR_CHEN));

    Ret = DwcOtgInterrupt (DwHc, Timeout, Channel, &Sub, Pid, IgnoreAck, &Split);

    if (Ret == XFER_NOT_HALTED) {
      *TransferResult = EFI_USB_ERR_TIMEOUT;
      MmioOr32 (DwHc->DwUsbBase + HCCHAR (Channel), DWC2_HCCHAR_CHDIS);
      Status = gBS->SetTimer (Timeout, TimerRelative,
                              EFI_TIMER_PERIOD_MILLISECONDS (1));
      ASSERT_EFI_ERROR (Status);
      if (EFI_ERROR (Status)) {
        break;
      }
      Status = Wait4Bit (Timeout, DwHc->DwUsbBase + HCINT (Channel),
                         DWC2_HCINT_CHHLTD, 1);
      if (Status == EFI_SUCCESS) {
        Status = EFI_TIMEOUT;
      } else {
        DEBUG ((DEBUG_ERROR, "Channel %u did not halt\n", Channel));
        Status = EFI_DEVICE_ERROR;
      }
      break;
    } else if (Ret == XFER_NOT_INIT) {
		Status = DwcOtgPortInit(DwHc);
		if(EFI_ERROR(Status)) {
			*TransferResult = EFI_USB_ERR_TIMEOUT;
			break;
		}
		goto restart_xfer;
    } else if (Ret == XFER_STALL) {
      *TransferResult = EFI_USB_ERR_STALL;
      Status = EFI_DEVICE_ERROR;
      break;
    } else if (Ret == XFER_CSPLIT) {
      ASSERT (Split.Splitting);

      if (Split.Tries++ < 3) {
        goto restart_channel;
      }

      goto restart_xfer;
    } else if (Ret == XFER_ERROR) {
      *TransferResult =
        EFI_USB_ERR_CRC |
        EFI_USB_ERR_TIMEOUT |
        EFI_USB_ERR_BITSTUFF |
        EFI_USB_ERR_SYSTEM;
      Status = EFI_DEVICE_ERROR;
      break;
    } else if (Ret == XFER_FRMOVRUN) {
      goto restart_channel;
    } else if (Ret == XFER_NAK) {
      if (Split.Splitting &&
          (EpType == DWC2_HCCHAR_EPTYPE_CONTROL)) {
	    DEBUG ((DEBUG_ERROR, "DwHcTransfer: restart_xfer\n"));
        goto restart_xfer;
      }

      *TransferResult = EFI_USB_ERR_NAK;
      Status = EFI_DEVICE_ERROR;
      break;
    }

    if (TransferDirection) { // in
      ArmDataSynchronizationBarrier();
	  if(TxferLen > 0) {
		CopyMem (Data+Done, DwHc->AlignedBuffer, TxferLen-Sub);
	  }
      if (Sub) {
        StopTransfer = TRUE;
      }
    }

    Done += TxferLen;
	
	DEBUG((DEBUG_INFO,"DwHcTransfer: Done=%d, NumPackets=%d, TxferLen=%d, Sub=%d\n", 
													Done,NumPackets,TxferLen,Sub));
  } while (Done < *DataLength && !StopTransfer);

  MmioWrite32 (DwHc->DwUsbBase + HCINTMSK(Channel), 0);
  MmioWrite32 (DwHc->DwUsbBase + HCINT(Channel), 0xFFFFFFFF);

  if(Done > 0) {
	*DataLength = Done;
  }

  gBS->RestoreTPL (Tpl);

  ASSERT (!EFI_ERROR (Status) ||
          *TransferResult != EFI_USB_NOERROR);

  return Status;
}

STATIC
DWUSB_DEFERRED_REQ *
DwHcFindDeferredTransfer (
  IN  DWUSB_OTGHC_DEV *DwHc,
  IN  UINT8 DeviceAddress,
  IN  UINT8 EndPointAddress
  )
{
  LIST_ENTRY *Entry;

  //DEBUG((DEBUG_INFO,"DwHcFindDeferredTransfer\n"));

  EFI_LIST_FOR_EACH (Entry, &DwHc->DeferredList) {
    DWUSB_DEFERRED_REQ *Req = EFI_LIST_CONTAINER (Entry, DWUSB_DEFERRED_REQ,
                                                  List);
												  
    if (Req->DeviceAddress == DeviceAddress &&
        Req->EpAddress == (EndPointAddress & 0xF) &&
        Req->TransferDirection == ((EndPointAddress >> 7) & 0x01)) {
      return Req;
    }
  }

  return NULL;
}

STATIC
VOID
DwHcDeferredTransfer (
  IN  DWUSB_DEFERRED_REQ *Req
  )
{
  EFI_STATUS Status;
  EFI_EVENT TimeoutEvt = NULL;

  //DEBUG((DEBUG_INFO,"DwHcDeferredTransfer\n"));

  Status = gBS->CreateEvent (EVT_TIMER, 0, NULL, NULL,
                             &TimeoutEvt
                             );
  ASSERT_EFI_ERROR (Status);
  if (EFI_ERROR (Status)) {
    goto out;
  }

  Status = gBS->SetTimer (TimeoutEvt, TimerForTransfer,
                          EFI_TIMER_PERIOD_MILLISECONDS(Req->TimeOut));
  ASSERT_EFI_ERROR (Status);
  if (EFI_ERROR (Status)) {
    goto out;
  }

  Req->TransferResult = EFI_USB_NOERROR;
  Status = DwHcTransfer (Req->DwHc, TimeoutEvt,
                         Req->Channel, Req->Translator,
                         Req->DeviceSpeed, Req->DeviceAddress,
                         Req->MaximumPacketLength, &Req->Pid,
                         Req->TransferDirection, Req->Data, &Req->DataLength,
                         Req->EpAddress, Req->EpType, &Req->TransferResult,
                         Req->IgnoreAck);

  if (Req->EpType == DWC2_HCCHAR_EPTYPE_INTR &&
      Status == EFI_DEVICE_ERROR &&
      Req->TransferResult == EFI_USB_ERR_NAK) {

    /* Swallow the NAK, the upper layer expects us to resubmit automatically */
	//DEBUG((DEBUG_INFO,"DwHcDeferredTransfer: IgnoreNak\n"));
    goto out;
  }
  DEBUG((DEBUG_INFO,"DwHcDeferredTransfer: CallBackFunction, Data=%x\n",Req->Data));
  Req->CallbackFunction (Req->Data, Req->DataLength,
                         Req->CallbackContext,
                         Req->TransferResult);
 out:
  if (TimeoutEvt != NULL) {
    gBS->CloseEvent (TimeoutEvt);
  }
}

/**
  Retrieves the Host Controller capabilities.

  @param  This           A pointer to the EFI_USB2_HC_PROTOCOL instance.
  @param  MaxSpeed       Host controller data transfer speed.
  @param  PortNumber     Number of the root hub ports.
  @param  Is64BitCapable TRUE if controller supports 64-bit memory addressing,
                         FALSE otherwise.

  @retval EFI_SUCCESS           The host controller capabilities were retrieved successfully.
  @retval EFI_INVALID_PARAMETER One of the input args was NULL.
  @retval EFI_DEVICE_ERROR      An error was encountered while attempting to
                                retrieve the capabilities.

**/
EFI_STATUS
EFIAPI
DwHcGetCapability (
  IN  EFI_USB2_HC_PROTOCOL *This,
  OUT UINT8                *MaxSpeed,
  OUT UINT8                *PortNumber,
  OUT UINT8                *Is64BitCapable
  )
{
  if ((MaxSpeed == NULL) || (PortNumber == NULL) || (Is64BitCapable == NULL)) {
    return EFI_INVALID_PARAMETER;
  }

  *MaxSpeed = EFI_USB_SPEED_HIGH;
  *PortNumber = 1;
  *Is64BitCapable = FALSE;
  DEBUG ((DEBUG_INFO, "DwHcGetCapability: %d ports, 64 bit %d\n", *PortNumber, *Is64BitCapable));

  return EFI_SUCCESS;
}


/**
  Retrieves current state of the USB host controller.

  @param  This  A pointer to the EFI_USB2_HC_PROTOCOL instance.
  @param  State A pointer to the EFI_USB_HC_STATE data structure that
                indicates current state of the USB host controller.

  @retval EFI_SUCCESS           The state information of the host controller was returned in State.
  @retval EFI_INVALID_PARAMETER State is NULL.
  @retval EFI_DEVICE_ERROR      An error was encountered while attempting to retrieve the
                                host controller's current state.

**/
EFI_STATUS
EFIAPI
DwHcGetState (
  IN  EFI_USB2_HC_PROTOCOL *This,
  OUT EFI_USB_HC_STATE     *State
  )
{
  //DEBUG((DEBUG_INFO,"DwHcGetState\n"));
  DWUSB_OTGHC_DEV *DwHc;

  DwHc = DWHC_FROM_THIS (This);

  *State = DwHc->DwHcState;

  return EFI_SUCCESS;
}

/**
  Sets the USB host controller to a specific state.

  @param  This  A pointer to the EFI_USB2_HC_PROTOCOL instance.
  @param  State Indicates the state of the host controller that will be set.

  @retval EFI_SUCCESS           The USB host controller was successfully placed in the state
                                specified by State.
  @retval EFI_INVALID_PARAMETER State is not valid.
  @retval EFI_DEVICE_ERROR      Failed to set the state specified by State due to device error.

**/
EFI_STATUS
EFIAPI
DwHcSetState (
  IN  EFI_USB2_HC_PROTOCOL *This,
  IN  EFI_USB_HC_STATE     State
  )
{
  //DEBUG((DEBUG_INFO,"DwHcSetState\n"));

  DWUSB_OTGHC_DEV *DwHc;

  DwHc = DWHC_FROM_THIS (This);

  DwHc->DwHcState = State;

  return EFI_SUCCESS;
}

/**
  Submits control transfer to a target USB device.

  @param  This                A pointer to the EFI_USB2_HC_PROTOCOL instance.
  @param  DeviceAddress       Represents the address of the target device on the USB.
  @param  DeviceSpeed         Indicates device speed.
  @param  MaximumPacketLength Indicates the maximum packet size that the default control transfer
                              endpoint is capable of sending or receiving.
  @param  Request             A pointer to the USB device request that will be sent to the USB device.
  @param  TransferDirection   Specifies the data direction for the transfer. There are three values
                              available, EfiUsbDataIn, EfiUsbDataOut and EfiUsbNoData.
  @param  Data                A pointer to the buffer of data that will be transmitted to USB device or
                              received from USB device.
  @param  DataLength          On input, indicates the size, in bytes, of the data buffer specified by Data.
                              On output, indicates the amount of data actually transferred.
  @param  TimeOut             Indicates the maximum time, in milliseconds, which the transfer is
                              allowed to complete.
  @param  Translator          A pointer to the transaction translator data.
  @param  TransferResult      A pointer to the detailed result information generated by this control
                              transfer.

  @retval EFI_SUCCESS           The control transfer was completed successfully.
  @retval EFI_INVALID_PARAMETER Some parameters are invalid.
  @retval EFI_OUT_OF_RESOURCES  The control transfer could not be completed due to a lack of resources.
  @retval EFI_TIMEOUT           The control transfer failed due to timeout.
  @retval EFI_DEVICE_ERROR      The control transfer failed due to host controller or device error.
                                Caller should check TransferResult for detailed error information.

**/
EFI_STATUS
EFIAPI
DwHcControlTransfer (
  IN  EFI_USB2_HC_PROTOCOL               *This,
  IN  UINT8                              DeviceAddress,
  IN  UINT8                              DeviceSpeed,
  IN  UINTN                              MaximumPacketLength,
  IN  EFI_USB_DEVICE_REQUEST             *Request,
  IN  EFI_USB_DATA_DIRECTION             TransferDirection,
  IN  OUT VOID                           *Data,
  IN  OUT UINTN                          *DataLength,
  IN  UINTN                              TimeOut,
  IN  EFI_USB2_HC_TRANSACTION_TRANSLATOR *Translator,
  OUT UINT32                             *TransferResult
  )
{
  DWUSB_OTGHC_DEV *DwHc;
  EFI_STATUS Status;
  UINT32 Pid;
  UINTN Length;
  EFI_USB_DATA_DIRECTION StatusDirection;
  UINT32 Direction;
  EFI_EVENT TimeoutEvt = NULL;

  //
  // Validate parameters
  //
  
  //DEBUG((DEBUG_INFO, "DwHcControlTransfer\n"));
  if ((Request == NULL) || (TransferResult == NULL))
  {
    return EFI_INVALID_PARAMETER;
  }

  if ((TransferDirection != EfiUsbDataIn) &&
      (TransferDirection != EfiUsbDataOut) &&
      (TransferDirection != EfiUsbNoData))
  {
    return EFI_INVALID_PARAMETER;
  }

  if ((TransferDirection == EfiUsbNoData) &&
      ((Data != NULL) || (*DataLength != 0)))
  {
    return EFI_INVALID_PARAMETER;
  }

  if ((TransferDirection != EfiUsbNoData) &&
      ((Data == NULL) || (*DataLength == 0)))
  {
    return EFI_INVALID_PARAMETER;
  }

  if ((MaximumPacketLength != 8) && (MaximumPacketLength != 16) &&
      (MaximumPacketLength != 32) && (MaximumPacketLength != 64))
  {
    return EFI_INVALID_PARAMETER;
  }

  if ((DeviceSpeed == EFI_USB_SPEED_LOW) && (MaximumPacketLength != 8))
  {
    return EFI_INVALID_PARAMETER;
  }

  DwHc = DWHC_FROM_THIS(This);

  Status = gBS->CreateEvent(
      EVT_TIMER, 0, NULL, NULL,
      &TimeoutEvt);
  ASSERT_EFI_ERROR(Status);
  if (EFI_ERROR(Status))
  {
    goto out;
  }

  Status = gBS->SetTimer(TimeoutEvt, TimerForTransfer,
                         EFI_TIMER_PERIOD_MILLISECONDS(TimeOut));
  ASSERT_EFI_ERROR(Status);
  if (EFI_ERROR(Status))
  {
    goto out;
  }

  //---  SETUP PHASE  ---
  //DEBUG((DEBUG_INFO, "DwHcControlTransfer: Enter Setup Stage\n"));
  Pid = DWC2_HC_PID_SETUP;
  Length = 8;
  Status = DwHcTransfer(DwHc, TimeoutEvt,
                        DWC2_HC_CHANNEL, Translator, DeviceSpeed,
                        DeviceAddress, MaximumPacketLength, &Pid, 0,
                        Request, &Length, 0, DWC2_HCCHAR_EPTYPE_CONTROL,
                        TransferResult, 1);

  if (EFI_ERROR(Status))
  {
    //DEBUG((EFI_D_ERROR, "DwHcControlTransfer: Setup Stage Error for device 0x%x: 0x%x\n",
    //       DeviceAddress, *TransferResult));
	goto out;
  }

  //---  DATA PHASE  ---
  if(Data)
  {
	  //DEBUG((DEBUG_INFO, "DwHcControlTransfer: Enter Data Stage\n"));
	  MicroSecondDelay(1000);
	  Pid = DWC2_HC_PID_DATA1;

	  if (TransferDirection == EfiUsbDataIn)
	  {
		Direction = 1;
	  }
	  else
	  {
		Direction = 0;
	  }

	  Status = DwHcTransfer(DwHc, TimeoutEvt,
							DWC2_HC_CHANNEL, Translator, DeviceSpeed,
							DeviceAddress, MaximumPacketLength, &Pid,
							Direction, Data, DataLength, 0,
							DWC2_HCCHAR_EPTYPE_CONTROL, TransferResult, 0);

	  if (EFI_ERROR(Status))
	  {
		//DEBUG((EFI_D_ERROR, "DwHcControlTransfer: Data Stage Error for device 0x%x: 0x%x\n",
		//	   DeviceAddress, *TransferResult));
		goto out;
	  }
  }
  
  //---  STATUS PHASE  ---
  //DEBUG((DEBUG_INFO, "DwHcControlTransfer: Enter Status Stage\n"));
  MicroSecondDelay(1000);
  if ((TransferDirection == EfiUsbDataOut) ||
      (TransferDirection == EfiUsbNoData))
  {
    StatusDirection = 1;
  }
  else
  {
    StatusDirection = 0;
  }

  Pid = DWC2_HC_PID_DATA1;
  Length = 0;
  Status = DwHcTransfer(DwHc, TimeoutEvt,
                        DWC2_HC_CHANNEL, Translator, DeviceSpeed,
                        DeviceAddress, MaximumPacketLength, &Pid,
                        StatusDirection, Data, &Length, 0,
                        DWC2_HCCHAR_EPTYPE_CONTROL, TransferResult, 1);

  if (EFI_ERROR(Status))
  {
    //DEBUG((EFI_D_ERROR, "DwHcControlTransfer: Status Stage Error for 0x%x: 0x%x\n",
    //       DeviceAddress, *TransferResult));
    goto out;
  }

out:
  if (TimeoutEvt != NULL)
  {
    gBS->CloseEvent(TimeoutEvt);
  }

  if (EFI_ERROR(Status))
  {
    //DEBUG((DEBUG_ERROR, "DwHcControlTransfer: RequestType 0x%x\n", Request->RequestType));
    //DEBUG((DEBUG_ERROR, "DwHcControlTransfer: Request 0x%x\n", Request->Request));
    //DEBUG((DEBUG_ERROR, "DwHcControlTransfer: Value 0x%x\n", Request->Value));
    //DEBUG((DEBUG_ERROR, "DwHcControlTransfer: Index 0x%x\n", Request->Index));
    //DEBUG((DEBUG_ERROR, "DwHcControlTransfer: Length 0x%x\n", Request->Length));
  }
  DEBUG ((DEBUG_INFO, "DwHcControlTransfer Status=%d\n",Status));
  return Status;
}

/**
  Submits bulk transfer to a bulk endpoint of a USB device.

  @param  This                A pointer to the EFI_USB2_HC_PROTOCOL instance.
  @param  DeviceAddress       Represents the address of the target device on the USB.
  @param  EndPointAddress     The combination of an endpoint number and an endpoint direction of the
                              target USB device.
  @param  DeviceSpeed         Indicates device speed.
  @param  MaximumPacketLength Indicates the maximum packet size the target endpoint is capable of
                              sending or receiving.
  @param  DataBuffersNumber   Number of data buffers prepared for the transfer.
  @param  Data                Array of pointers to the buffers of data that will be transmitted to USB
                              device or received from USB device.
  @param  DataLength          When input, indicates the size, in bytes, of the data buffers specified by
                              Data. When output, indicates the actually transferred data size.
  @param  DataToggle          A pointer to the data toggle value.
  @param  TimeOut             Indicates the maximum time, in milliseconds, which the transfer is
                              allowed to complete.
  @param  Translator          A pointer to the transaction translator data.
  @param  TransferResult      A pointer to the detailed result information of the bulk transfer.

  @retval EFI_SUCCESS           The bulk transfer was completed successfully.
  @retval EFI_INVALID_PARAMETER Some parameters are invalid.
  @retval EFI_OUT_OF_RESOURCES  The bulk transfer could not be submitted due to a lack of resources.
  @retval EFI_TIMEOUT           The bulk transfer failed due to timeout.
  @retval EFI_DEVICE_ERROR      The bulk transfer failed due to host controller or device error.
                                Caller should check TransferResult for detailed error information.

**/
EFI_STATUS
EFIAPI
DwHcBulkTransfer (
  IN  EFI_USB2_HC_PROTOCOL               *This,
  IN  UINT8                              DeviceAddress,
  IN  UINT8                              EndPointAddress,
  IN  UINT8                              DeviceSpeed,
  IN  UINTN                              MaximumPacketLength,
  IN  UINT8                              DataBuffersNumber,
  IN  OUT VOID                           *Data[EFI_USB_MAX_BULK_BUFFER_NUM],
  IN  OUT UINTN                          *DataLength,
  IN  OUT UINT8                          *DataToggle,
  IN  UINTN                              TimeOut,
  IN  EFI_USB2_HC_TRANSACTION_TRANSLATOR *Translator,
  OUT UINT32                             *TransferResult
  )
{
  DWUSB_OTGHC_DEV         *DwHc;
  EFI_STATUS              Status;
  UINT8                   TransferDirection;
  UINT8                   EpAddress;
  UINT32                  Pid;
  EFI_EVENT TimeoutEvt = NULL;

  //DEBUG((DEBUG_INFO,"DwHcBulkTransfer\n"));
  
  /* Validate the parameters */
  if ((DataLength == NULL) || (*DataLength == 0) ||
      (Data == NULL) || (Data[0] == NULL) || (TransferResult == NULL)) {
    return EFI_INVALID_PARAMETER;
  }

  if ((*DataToggle != 0) && (*DataToggle != 1)) {
    return EFI_INVALID_PARAMETER;
  }

  if ((DeviceSpeed == EFI_USB_SPEED_LOW) ||
      ((DeviceSpeed == EFI_USB_SPEED_FULL) && (MaximumPacketLength > 64)) ||
      ((EFI_USB_SPEED_HIGH == DeviceSpeed) && (MaximumPacketLength > 512))) {
    return EFI_INVALID_PARAMETER;
  }

  DwHc = DWHC_FROM_THIS (This);

  Status = gBS->CreateEvent (EVT_TIMER, 0, NULL, NULL,
                             &TimeoutEvt
                             );
  ASSERT_EFI_ERROR (Status);
  if (EFI_ERROR (Status)) {
    goto out;
  }

  Status = gBS->SetTimer (TimeoutEvt, TimerForTransfer,
                          EFI_TIMER_PERIOD_MILLISECONDS(TimeOut));
  ASSERT_EFI_ERROR (Status);
  if (EFI_ERROR (Status)) {
    goto out;
  }

  Status                  = EFI_DEVICE_ERROR;
  TransferDirection       = (EndPointAddress >> 7) & 0x01;
  EpAddress               = EndPointAddress & 0x0F;
  Pid                     = (*DataToggle << 1);

  Status = DwHcTransfer (DwHc, TimeoutEvt,
                         DWC2_HC_CHANNEL_BULK, Translator, DeviceSpeed,
                         DeviceAddress, MaximumPacketLength, &Pid,
                         TransferDirection, Data[0], DataLength, EpAddress,
                         DWC2_HCCHAR_EPTYPE_BULK, TransferResult, 1);

  *DataToggle = (Pid >> 1);

 out:
  if (TimeoutEvt != NULL) {
    gBS->CloseEvent (TimeoutEvt);
  }

  return Status;
}

/**
  Submits an asynchronous interrupt transfer to an interrupt endpoint of a USB device.
  Translator parameter doesn't exist in UEFI2.0 spec, but it will be updated in the following specification version.

  @param  This                A pointer to the EFI_USB2_HC_PROTOCOL instance.
  @param  DeviceAddress       Represents the address of the target device on the USB.
  @param  EndPointAddress     The combination of an endpoint number and an endpoint direction of the
                              target USB device.
  @param  DeviceSpeed         Indicates device speed.
  @param  MaximumPacketLength Indicates the maximum packet size the target endpoint is capable of
                              sending or receiving.
  @param  IsNewTransfer       If TRUE, an asynchronous interrupt pipe is built between the host and the
                              target interrupt endpoint. If FALSE, the specified asynchronous interrupt
                              pipe is canceled. If TRUE, and an interrupt transfer exists for the target
                              end point, then EFI_INVALID_PARAMETER is returned.
  @param  DataToggle          A pointer to the data toggle value.
  @param  PollingInterval     Indicates the interval, in milliseconds, that the asynchronous interrupt
                              transfer is polled.
  @param  DataLength          Indicates the length of data to be received at the rate specified by
                              PollingInterval from the target asynchronous interrupt endpoint.
  @param  Translator          A pointr to the transaction translator data.
  @param  CallBackFunction    The Callback function. This function is called at the rate specified by
                              PollingInterval.
  @param  Context             The context that is passed to the CallBackFunction. This is an
                              optional parameter and may be NULL.

  @retval EFI_SUCCESS           The asynchronous interrupt transfer request has been successfully
                                submitted or canceled.
  @retval EFI_INVALID_PARAMETER Some parameters are invalid.
  @retval EFI_OUT_OF_RESOURCES  The request could not be completed due to a lack of resources.

**/
EFI_STATUS
EFIAPI
DwHcAsyncInterruptTransfer (
  IN  EFI_USB2_HC_PROTOCOL               *This,
  IN  UINT8                              DeviceAddress,
  IN  UINT8                              EndPointAddress,
  IN  UINT8                              DeviceSpeed,
  IN  UINTN                              MaximumPacketLength,
  IN  BOOLEAN                            IsNewTransfer,
  IN  OUT UINT8                          *DataToggle,
  IN  UINTN                              PollingInterval,
  IN  UINTN                              DataLength,
  IN  EFI_USB2_HC_TRANSACTION_TRANSLATOR *Translator,
  IN  EFI_ASYNC_USB_TRANSFER_CALLBACK    CallbackFunction,
  IN  VOID                               *Context OPTIONAL
  )
{
  DWUSB_OTGHC_DEV                 *DwHc;
  EFI_STATUS                      Status;
  EFI_TPL                         PreviousTpl;
  VOID                            *Data = NULL;
  DWUSB_DEFERRED_REQ              *FoundReq = NULL;
  DWUSB_DEFERRED_REQ              *NewReq = NULL;

  //DEBUG((DEBUG_INFO,"DwHcAsyncInterruptTransfer\n"));

  if (!(EndPointAddress & USB_ENDPOINT_DIR_IN)) {
    return EFI_INVALID_PARAMETER;
  }

  DwHc = DWHC_FROM_THIS (This);

  PreviousTpl = gBS->RaiseTPL(TPL_NOTIFY);
  FoundReq = DwHcFindDeferredTransfer(DwHc, DeviceAddress, EndPointAddress);

  if (IsNewTransfer) {
    if (FoundReq != NULL) {
      Status = EFI_INVALID_PARAMETER;
      goto done;
    }

    if (DataLength == 0) {
      Status = EFI_INVALID_PARAMETER;
      goto done;
    }

    if ((*DataToggle != 1) && (*DataToggle != 0)) {
      Status = EFI_INVALID_PARAMETER;
      goto done;
    }

    if ((PollingInterval > 255) || (PollingInterval < 1)) {
      Status = EFI_INVALID_PARAMETER;
      goto done;
    }

    if (CallbackFunction == NULL) {
      Status = EFI_INVALID_PARAMETER;
      goto done;
    }
  }

  if (!IsNewTransfer) {
    if (FoundReq == NULL) {
      DEBUG ((DEBUG_ERROR,
              "DwHcAsyncInterruptTransfer: %u:%u> transfer not found\n",
              DeviceAddress,
              EndPointAddress & 0xF));
      Status = EFI_INVALID_PARAMETER;
      goto done;
    }

    *DataToggle = FoundReq->Pid >> 1;
    FreePool (FoundReq->Data);

    RemoveEntryList (&FoundReq->List);
    FreePool (FoundReq);

    Status = EFI_SUCCESS;
    goto done;
  }

  NewReq = AllocateZeroPool (sizeof *NewReq);
  if (NewReq == NULL) {
    DEBUG ((DEBUG_ERROR, "DwHcAsyncInterruptTransfer: failed to allocate req"));
    Status = EFI_OUT_OF_RESOURCES;
    goto done;
  }

  Data = AllocateZeroPool (DataLength);
  if (Data == NULL) {
    DEBUG ((DEBUG_ERROR, "DwHcAsyncInterruptTransfer: failed to allocate buffer\n"));
    Status = EFI_OUT_OF_RESOURCES;
    goto done;
  }

  InitializeListHead (&NewReq->List);

  NewReq->FrameInterval = PollingInterval;
  NewReq->TargetFrame = DwHc->CurrentFrame +
    NewReq->FrameInterval;

  NewReq->DwHc = DwHc;
  NewReq->Channel = DWC2_HC_CHANNEL_ASYNC;
  NewReq->Translator = Translator;
  NewReq->DeviceSpeed = DeviceSpeed;
  NewReq->DeviceAddress = DeviceAddress;
  NewReq->MaximumPacketLength = MaximumPacketLength;
  NewReq->TransferDirection = (EndPointAddress >> 7) & 0x01;
  NewReq->Data = Data;
  NewReq->DataLength = DataLength;
  NewReq->Pid = *DataToggle << 1;
  NewReq->EpAddress = EndPointAddress & 0x0F;
  NewReq->EpType = DWC2_HCCHAR_EPTYPE_INTR;
  NewReq->IgnoreAck = FALSE;
  NewReq->CallbackFunction = CallbackFunction;
  NewReq->CallbackContext = Context;
  NewReq->TimeOut = 1000; /* 1000 ms */

  InsertTailList (&DwHc->DeferredList, &NewReq->List);
  Status = EFI_SUCCESS;

 done:
  gBS->RestoreTPL(PreviousTpl);

  if (Status != EFI_SUCCESS) {
    if (Data != NULL) {
      FreePool (Data);
    }

    if (NewReq != NULL) {
      FreePool (NewReq);
    }
  }

  return Status;
}

/**
  Submits synchronous interrupt transfer to an interrupt endpoint of a USB device.
  Translator parameter doesn't exist in UEFI2.0 spec, but it will be updated in the following specification version.

  @param  This                  A pointer to the EFI_USB2_HC_PROTOCOL instance.
  @param  DeviceAddress         Represents the address of the target device on the USB.
  @param  EndPointAddress       The combination of an endpoint number and an endpoint direction of the
                                target USB device.
  @param  DeviceSpeed           Indicates device speed.
  @param  MaximumPacketLength   Indicates the maximum packet size the target endpoint is capable of
                                sending or receiving.
  @param  Data                  A pointer to the buffer of data that will be transmitted to USB device or
                                received from USB device.
  @param  DataLength            On input, the size, in bytes, of the data buffer specified by Data. On
                                output, the number of bytes transferred.
  @param  DataToggle            A pointer to the data toggle value.
  @param  TimeOut               Indicates the maximum time, in milliseconds, which the transfer is
                                allowed to complete.
  @param  Translator            A pointr to the transaction translator data.
  @param  TransferResult        A pointer to the detailed result information from the synchronous
                                interrupt transfer.

  @retval EFI_SUCCESS           The synchronous interrupt transfer was completed successfully.
  @retval EFI_INVALID_PARAMETER Some parameters are invalid.
  @retval EFI_OUT_OF_RESOURCES  The synchronous interrupt transfer could not be submitted due to a lack of resources.
  @retval EFI_TIMEOUT           The synchronous interrupt transfer failed due to timeout.
  @retval EFI_DEVICE_ERROR      The synchronous interrupt transfer failed due to host controller or device error.
                                Caller should check TransferResult for detailed error information.

**/
EFI_STATUS
EFIAPI
DwHcSyncInterruptTransfer (
  IN  EFI_USB2_HC_PROTOCOL               *This,
  IN  UINT8                              DeviceAddress,
  IN  UINT8                              EndPointAddress,
  IN  UINT8                              DeviceSpeed,
  IN  UINTN                              MaximumPacketLength,
  IN  OUT VOID                           *Data,
  IN  OUT UINTN                          *DataLength,
  IN  OUT UINT8                          *DataToggle,
  IN  UINTN                              TimeOut,
  IN  EFI_USB2_HC_TRANSACTION_TRANSLATOR *Translator,
  OUT UINT32                             *TransferResult
  )
{
  DWUSB_OTGHC_DEV *DwHc;
  EFI_STATUS Status;
  EFI_EVENT TimeoutEvt;
  UINT8 TransferDirection;
  UINT8 EpAddress;
  UINT32 Pid;

  //DEBUG((DEBUG_INFO,"DwHcSyncInterruptTransfer\n"));

  DwHc  = DWHC_FROM_THIS(This);

  if (Data == NULL ||
      DataLength == NULL ||
      DataToggle == NULL ||
      TransferResult == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  if (*DataLength == 0) {
    return EFI_INVALID_PARAMETER;
  }

  if ((*DataToggle != 0) && (*DataToggle != 1)) {
    return EFI_INVALID_PARAMETER;
  }

  Status = gBS->CreateEvent (
                             EVT_TIMER, 0, NULL,
                             NULL, &TimeoutEvt
                             );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = gBS->SetTimer (TimeoutEvt, TimerForTransfer, EFI_TIMER_PERIOD_MILLISECONDS(TimeOut));
  if (EFI_ERROR (Status)) {
    goto out;
  }

  TransferDirection = (EndPointAddress >> 7) & 0x01;
  EpAddress = EndPointAddress & 0x0F;
  Pid = (*DataToggle << 1);
  Status = DwHcTransfer(DwHc, TimeoutEvt,
                        DWC2_HC_CHANNEL_SYNC, Translator,
                        DeviceSpeed, DeviceAddress,
                        MaximumPacketLength,
                        &Pid, TransferDirection, Data,
                        DataLength, EpAddress,
                        DWC2_HCCHAR_EPTYPE_INTR,
                        TransferResult, 0);
  *DataToggle = (Pid >> 1);

 out:
  if (TimeoutEvt != NULL) {
    gBS->CloseEvent (TimeoutEvt);
  }
  return Status;
}

/**
  Submits isochronous transfer to an isochronous endpoint of a USB device.

  This function is used to submit isochronous transfer to a target endpoint of a USB device. 
  The target endpoint is specified by DeviceAddressand EndpointAddress. Isochronous transfers are 
  used when working with isochronous date. It provides periodic, continuous communication between 
  the host and a device. Isochronous transfers can beused only by full-speed, high-speed, and 
  super-speed devices.

  High-speed isochronous transfers can be performed using multiple data buffers. The number of 
  buffers that are actually prepared for the transfer is specified by DataBuffersNumber. For
  full-speed isochronous transfers this value is ignored.

  Data represents a list of pointers to the data buffers. For full-speed isochronous transfers
  only the data pointed by Data[0]shall be used. For high-speed isochronous transfers and for
  the split transactions depending on DataLengththere several data buffers canbe used. For the
  high-speed isochronous transfers the total number of buffers must not exceed EFI_USB_MAX_ISO_BUFFER_NUM. 

  For split transactions performed on full-speed device by high-speed host controller the total
  number of buffers is limited to EFI_USB_MAX_ISO_BUFFER_NUM1.
  If the isochronous transfer is successful, then EFI_SUCCESSis returned. The isochronous transfer 
  is designed to be completed within one USB frame time, if it cannot be completed, EFI_TIMEOUT
  is returned. If an error other than timeout occurs during the USB transfer, then EFI_DEVICE_ERROR
  is returned and the detailed status code will be returned in TransferResult.

  EFI_INVALID_PARAMETERis returned if one of the following conditionsis satisfied:
    - Data is NULL. 
    - DataLength is 0.
    - DeviceSpeed is not one of the supported values listed above.
    - MaximumPacketLength is invalid. MaximumPacketLength must be 1023 or less for full-speed devices,
      and 1024 or less for high-speed and super-speed devices.
    - TransferResult is NULL.

  @param  This                  A pointer to the EFI_USB2_HC_PROTOCOL instance.
  @param  DeviceAddress         Represents the address of the target device on the USB.
  @param  EndPointAddress       The combination of an endpoint number and an endpoint direction of the
                                target USB device.
  @param  DeviceSpeed           Indicates device speed. The supported values are EFI_USB_SPEED_FULL, 
                                EFI_USB_SPEED_HIGH, or EFI_USB_SPEED_SUPER.
  @param  MaximumPacketLength   Indicates the maximum packet size the target endpoint is capable of
                                sending or receiving.
  @param  DataBuffersNumber     Number of data buffers prepared for the transfer.
  @param  Data                  Array of pointers to the buffers of data that will be transmitted to USB
                                device or received from USB device.
  @param  DataLength            Specifies the length, in bytes, of the data to be sent to or received from
                                the USB device.
  @param  Translator            A pointer to the transaction translator data.
  @param  TransferResult        A pointer to the detailed result information of the isochronous transfer.

  @retval EFI_SUCCESS           The isochronous transfer was completed successfully.
  @retval EFI_INVALID_PARAMETER Some parameters are invalid.
  @retval EFI_OUT_OF_RESOURCES  The isochronous transfer could not be submitted due to a lack of resources.
  @retval EFI_TIMEOUT           The isochronous transfer cannot be completed within the one USB frame time.
  @retval EFI_DEVICE_ERROR      The isochronous transfer failed due to host controller or device error.
                                Caller should check TransferResult for detailed error information.

**/
EFI_STATUS
EFIAPI
DwHcIsochronousTransfer (
  IN  EFI_USB2_HC_PROTOCOL               *This,
  IN  UINT8                              DeviceAddress,
  IN  UINT8                              EndPointAddress,
  IN  UINT8                              DeviceSpeed,
  IN  UINTN                              MaximumPacketLength,
  IN  UINT8                              DataBuffersNumber,
  IN  OUT VOID                           *Data[EFI_USB_MAX_ISO_BUFFER_NUM],
  IN  UINTN                              DataLength,
  IN  EFI_USB2_HC_TRANSACTION_TRANSLATOR *Translator,
  OUT UINT32                             *TransferResult
  )
{
  //DEBUG((DEBUG_ERROR, "IsochronousTransfer UNSUPPORTED\n"));
  return EFI_UNSUPPORTED;
}

/**
  Submits nonblocking isochronous transfer to an isochronous endpoint of a USB device.

  This is an asynchronous type of USB isochronous transfer. If the caller submits a USB
  isochronous transfer request through this function, this function will return immediately.

  When the isochronous transfer completes, the IsochronousCallbackfunction will be triggered,
  the caller can know the transfer results. If the transfer is successful, the caller can get
  the data received or sent in this callback function.

  The target endpoint is specified by DeviceAddressand EndpointAddress. Isochronous transfers
  are used when working with isochronous date. It provides periodic, continuous communication
  between the host and a device. Isochronous transfers can be used only by full-speed, high-speed,
  and super-speed devices.

  High-speed isochronous transfers can be performed using multiple data buffers. The number of 
  buffers that are actually prepared for the transfer is specified by DataBuffersNumber. For
  full-speed isochronous transfers this value is ignored.

  Data represents a list of pointers to the data buffers. For full-speed isochronous transfers
  only the data pointed by Data[0] shall be used. For high-speed isochronous transfers and for
  the split transactions depending on DataLength there several data buffers can be used. For
  the high-speed isochronous transfers the total number of buffers must not exceed EFI_USB_MAX_ISO_BUFFER_NUM.

  For split transactions performed on full-speed device by high-speed host controller the total
  number of buffers is limited to EFI_USB_MAX_ISO_BUFFER_NUM1.

  EFI_INVALID_PARAMETER is returned if one of the following conditionsis satisfied:
    - Data is NULL. 
    - DataLength is 0.
    - DeviceSpeed is not one of the supported values listed above.
    - MaximumPacketLength is invalid. MaximumPacketLength must be 1023 or less for full-speed 
      devices and 1024 or less for high-speed and super-speed devices.

  @param  This                  A pointer to the EFI_USB2_HC_PROTOCOL instance.
  @param  DeviceAddress         Represents the address of the target device on the USB.
  @param  EndPointAddress       The combination of an endpoint number and an endpoint direction of the
                                target USB device.
  @param  DeviceSpeed           Indicates device speed. The supported values are EFI_USB_SPEED_FULL, 
                                EFI_USB_SPEED_HIGH, or EFI_USB_SPEED_SUPER.
  @param  MaximumPacketLength   Indicates the maximum packet size the target endpoint is capable of
                                sending or receiving.
  @param  DataBuffersNumber     Number of data buffers prepared for the transfer.
  @param  Data                  Array of pointers to the buffers of data that will be transmitted to USB
                                device or received from USB device.
  @param  DataLength            Specifies the length, in bytes, of the data to be sent to or received from
                                the USB device.
  @param  Translator            A pointer to the transaction translator data.
  @param  IsochronousCallback   The Callback function. This function is called if the requested
                                isochronous transfer is completed.
  @param  Context               Data passed to the IsochronousCallback function. This is an
                                optional parameter and may be NULL.

  @retval EFI_SUCCESS           The asynchronous isochronous transfer request has been successfully
                                submitted or canceled.
  @retval EFI_INVALID_PARAMETER Some parameters are invalid.
  @retval EFI_OUT_OF_RESOURCES  The asynchronous isochronous transfer could not be submitted due to
                                a lack of resources.

**/
EFI_STATUS
EFIAPI
DwHcAsyncIsochronousTransfer (
  IN  EFI_USB2_HC_PROTOCOL               *This,
  IN  UINT8                              DeviceAddress,
  IN  UINT8                              EndPointAddress,
  IN  UINT8                              DeviceSpeed,
  IN  UINTN                              MaximumPacketLength,
  IN  UINT8                              DataBuffersNumber,
  IN  OUT VOID                           *Data[EFI_USB_MAX_ISO_BUFFER_NUM],
  IN  UINTN                              DataLength,
  IN  EFI_USB2_HC_TRANSACTION_TRANSLATOR *Translator,
  IN  EFI_ASYNC_USB_TRANSFER_CALLBACK    IsochronousCallBack,
  IN  VOID                               *Context
  )
{
  //DEBUG((DEBUG_ERROR, "AsyncIsochronousTransfer UNSUPPORTED\n"));
  return EFI_UNSUPPORTED;
}

EFI_STATUS
DwcOtgCoreInit(
    IN  DWUSB_OTGHC_DEV *DwHc
    )
{
  UINT32 Speed = DWC_SPEED_PARAM_HIGH;
  UINT32 PhyType = DWC_PHY_TYPE_PARAM_UTMI;
  UINT32 PhyUtmiWidth = 16;
  UINT32 UsbCfg = MmioRead32(DwHc->DwUsbBase + GUSBCFG);
  UINT32 AhbCfg = MmioRead32(DwHc->DwUsbBase + GAHBCFG);
  UINT8 DmaBurstSize = 3;
  UINT8 OtgVer = 1;
  UINT8 OtgCap = DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE;

  //DEBUG((DEBUG_INFO, "DwcOtgCoreInit\n"));

  /* Reset the Controller */
  DwcOtgCoreReset(DwHc);

  //UINT32 total_fifo_size = (MmioRead32(DwHc->DwUsbBase + GHWCFG3) & DWC2_HWCFG3_DFIFO_DEPTH_MASK) >> DWC2_HWCFG3_DFIFO_DEPTH_OFFSET;
  //UINT32 rx_fifo_size = MmioRead32(DwHc->DwUsbBase + GRXFSIZ);
  //UINT32 nperio_tx_fifo_size = MmioRead32(DwHc->DwUsbBase + GNPTXFSIZ) >> 16;

  //DEBUG((DEBUG_INFO, "Total FIFO SZ=%d\n", total_fifo_size));
  //DEBUG((DEBUG_INFO, "HW Rx FIFO SZ=%d\n", rx_fifo_size));
  //DEBUG((DEBUG_INFO, "HW NP Tx FIFO SZ=%d\n", nperio_tx_fifo_size));

  /*
   * This programming sequence needs to happen in FS mode before any other
   * programming occurs 
   */
  if ((Speed == DWC_SPEED_PARAM_FULL) && (PhyType == DWC2_PHY_TYPE_FS))
  {
    /* If FS mode with FS PHY */
    //DEBUG((DEBUG_INFO, "DwcOtgCoreInit: Speed == FULL\n"));
    /*
     * core_init() is now called on every switch so only call the
     * following for the first time through. 
     */
    //DEBUG((DEBUG_INFO, "FS_PHY detected\n"));
    UsbCfg = MmioOr32(DwHc->DwUsbBase + GUSBCFG, DWC2_GUSBCFG_PHYSEL);
	
    /* Reset after a PHY select */
    DwcOtgCoreReset(DwHc);

    /*
     * Program DCFG.DevSpd or HCFG.FSLSPclkSel to 48Mhz in FS. Also do
     * this on HNP Dev/Host mode switches (done in dev_init and
     * host_init). 
     */
    InitFSLSPClkSel(DwHc,DWC_PHY_TYPE_PARAM_UTMI);
  }
  /*
   * endif speed == DWC_SPEED_PARAM_FULL 
   */
  else
  {
    /* High speed PHY */
    //DEBUG((DEBUG_INFO, "DwcOtgCoreInit: Speed == HIGH\n"));
    UsbCfg = MmioRead32(DwHc->DwUsbBase + GUSBCFG);
    /*
     * HS PHY parameters.  These parameters are preserved during soft 
     * reset so only program the first time.  Do a soft reset
     * immediately after setting phyif.  
     */
    if (PhyType == DWC_PHY_TYPE_PARAM_ULPI)
    {
      /* ULPI interface */
      UsbCfg |= DWC2_GUSBCFG_ULPI_UTMI_SEL;
      UsbCfg &= (~DWC2_GUSBCFG_PHYIF);
      UsbCfg &= (~DWC2_GUSBCFG_DDRSEL);
      //DEBUG((DEBUG_INFO, "ULPI phy\n"));
    }
    else if (PhyType == DWC_PHY_TYPE_PARAM_UTMI)
    {
      /* UTMI+ interface */
      UsbCfg &= (~DWC2_GUSBCFG_ULPI_UTMI_SEL);
      if (PhyUtmiWidth == 16)
      {
        UsbCfg |= DWC2_GUSBCFG_PHYIF;
        //DEBUG((DEBUG_INFO, "UTMI+ 16 bit PHY\n"));
      }
      else
      {
        UsbCfg &= (~DWC2_GUSBCFG_PHYIF);
        //DEBUG((DEBUG_INFO, "UTMI+ 8 bit PHY\n"));
      }
    }

    MmioWrite32(DwHc->DwUsbBase + GUSBCFG, UsbCfg);

    /* Reset after setting the PHY parameters */
    DwcOtgCoreReset(DwHc);
  }

  /* Program the GAHBCFG Register */
  switch ((MmioRead32(DwHc->DwUsbBase + GHWCFG2) & DWC2_HWCFG2_ARCHITECTURE_MASK) >> DWC2_HWCFG2_ARCHITECTURE_OFFSET)
  {
  case DWC_SLAVE_ONLY_ARCH:
    //DEBUG((DEBUG_INFO, "Slave Only Mode\n"));
    AhbCfg &= (~DWC2_GAHBCFG_NPTXFEMPLVL_TXFEMPLVL);
    AhbCfg &= (~DWC2_GAHBCFG_PTXFEMPLVL);
    break;

  case DWC_EXT_DMA_ARCH:
    //DEBUG((DEBUG_INFO, "External DMA Mode\n"));
    AhbCfg &= (~DWC2_GAHBCFG_HBURSTLEN);
    AhbCfg |= DWC_GAHBCFG_INT_DMA_BURST_INCR8 << DWC2_GAHBCFG_HBURSTLEN_OFFSET;
    break;

  case DWC_INT_DMA_ARCH:
    //DEBUG((DEBUG_INFO, "Internal DMA Mode\n"));
    AhbCfg &= (~DWC2_GAHBCFG_HBURSTLEN);
    AhbCfg |= DmaBurstSize << DWC2_GAHBCFG_HBURSTLEN_OFFSET;
    break;
  }

  AhbCfg |= DWC2_GAHBCFG_DMAENABLE;
  MmioWrite32(DwHc->DwUsbBase + GAHBCFG, AhbCfg);

  /* Set OTG version supported */
  MmioAndThenOr32(DwHc->DwUsbBase + GOTGCTL, ~DWC2_GOTGCTL_OTGVER, OtgVer << DWC2_GOTGCTL_OTGVER_OFFSET);

  /* Program the GUSBCFG register */
  UsbCfg = MmioRead32(DwHc->DwUsbBase + GUSBCFG);
  UsbCfg &= (~DWC2_GUSBCFG_HNPCAP);
  UsbCfg &= (~DWC2_GUSBCFG_SRPCAP);

  switch ((MmioRead32(DwHc->DwUsbBase + GHWCFG2) & DWC2_HWCFG2_OP_MODE_MASK) >> DWC2_HWCFG2_OP_MODE_OFFSET)
  {
  case DWC_MODE_HNP_SRP_CAPABLE:
    UsbCfg |= (OtgCap == DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE) << DWC2_GUSBCFG_HNPCAP_OFFSET;
    UsbCfg |= (OtgCap != DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE) << DWC2_GUSBCFG_SRPCAP_OFFSET;
    break;

  case DWC_MODE_SRP_ONLY_CAPABLE:
    UsbCfg |= (OtgCap != DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE) << DWC2_GUSBCFG_SRPCAP_OFFSET;
    break;

  case DWC_MODE_NO_HNP_SRP_CAPABLE:
    break;

  case DWC_MODE_SRP_CAPABLE_DEVICE:
    UsbCfg |= (OtgCap != DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE) << DWC2_GUSBCFG_SRPCAP_OFFSET;
    break;

  case DWC_MODE_NO_SRP_CAPABLE_DEVICE:
    break;

  case DWC_MODE_SRP_CAPABLE_HOST:
    UsbCfg |= (OtgCap != DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE) << DWC2_GUSBCFG_SRPCAP_OFFSET;
    break;

  case DWC_MODE_NO_SRP_CAPABLE_HOST:
    break;
  }

  MmioWrite32(DwHc->DwUsbBase + GUSBCFG, UsbCfg);

  /* Enable common interrupts */
  DwcOtgEnableCommonInterrupts(DwHc);

  /*
   * Do device or host intialization based on mode during PCD and HCD
   * initialization 
   */
  //DEBUG((DEBUG_INFO, "Host Mode\n"));
  return EFI_SUCCESS;
}

/**
  Provides software reset for the USB host controller.

  @param  This       A pointer to the EFI_USB2_HC_PROTOCOL instance.
  @param  Attributes A bit mask of the reset operation to perform.

  @retval EFI_SUCCESS           The reset operation succeeded.
  @retval EFI_INVALID_PARAMETER Attributes is not valid.
  @retval EFI_UNSUPPORTED       The type of reset specified by Attributes is not currently
                                supported by the host controller hardware.
  @retval EFI_ACCESS_DENIED     Reset operation is rejected due to the debug port being configured
                                and active; only EFI_USB_HC_RESET_GLOBAL_WITH_DEBUG or
                                EFI_USB_HC_RESET_HOST_WITH_DEBUG reset Attributes can be used to
                                perform reset operation for this host controller.
  @retval EFI_DEVICE_ERROR      An error was encountered while attempting to
                                retrieve the capabilities.

**/
EFI_STATUS
EFIAPI
DwHcReset (
           IN EFI_USB2_HC_PROTOCOL *This,
           IN UINT16               Attributes
           )
{
  EFI_STATUS Status;
  EFI_EVENT TimeoutEvt = NULL;

  //DEBUG((DEBUG_INFO, "DwHcReset\n"));
  DWUSB_OTGHC_DEV *DwHc;
  DwHc = DWHC_FROM_THIS(This);

  Status = gBS->CreateEvent(EVT_TIMER, 0, NULL, NULL,
                            &TimeoutEvt);
  ASSERT_EFI_ERROR(Status);
  if (EFI_ERROR(Status))
  {
    goto out;
  }

  Status = gBS->SetTimer(TimeoutEvt, TimerRelative,
                         EFI_TIMER_PERIOD_MILLISECONDS(10000));
  ASSERT_EFI_ERROR(Status);
  if (EFI_ERROR(Status))
  {
    goto out;
  }

  UINT32 snpsid = MmioRead32(DwHc->DwUsbBase + 0x40);

  if ((snpsid & 0xFFFF0000) != 0x4F540000)
  {
    //DEBUG((DEBUG_ERROR, "Bad value for SNPSID: 0x%08x\n", snpsid));
    goto out;
  }

#define FORCE_ID_CLEAR -1
#define FORCE_ID_HOST 0
#define FORCE_ID_SLAVE 1
#define FORCE_ID_ERROR 2

  DwcOtgSetForceId(DwHc, FORCE_ID_HOST);
  /*
   * Disable the global interrupt until all the interrupt
   * handlers are installed.
   */
  DwcOtgDisableGlobalInterrupts(DwHc);
  /* Initialize the DWC_otg core */
  if (DwcOtgCoreInit(DwHc))
  {
    //DEBUG((DEBUG_INFO, "DwcOtgCoreInit failed!\n"));
    goto out;
  }

  for (int i = 0; i < 2; i++)
  {
    DwcOtgHcCleanup(DwHc, i);
  }

  DwcOtgCoreHostInit(DwHc, DWC_SPEED_PARAM_HIGH);

  /*
   * Enable the global interrupt after all the interrupt
   * handlers are installed.
   */
  DwcOtgEnableGlobalInterrupts(DwHc);

out:
  if (TimeoutEvt != NULL)
  {
    gBS->CloseEvent(TimeoutEvt);
  }

  return Status;
}

/**
  Retrieves the current status of a USB root hub port.

  @param  This       A pointer to the EFI_USB2_HC_PROTOCOL instance.
  @param  PortNumber Specifies the root hub port from which the status is to be retrieved.
                     This value is zero based.
  @param  PortStatus A pointer to the current port status bits and port status change bits.

  @retval EFI_SUCCESS           The status of the USB root hub port specified by PortNumber
                                was returned in PortStatus.
  @retval EFI_INVALID_PARAMETER PortNumber is invalid.

**/
EFI_STATUS
EFIAPI
DwHcGetRootHubPortStatus (
  IN  EFI_USB2_HC_PROTOCOL *This,
  IN  UINT8                PortNumber,
  OUT EFI_USB_PORT_STATUS  *PortStatus
  )
{
  DWUSB_OTGHC_DEV	*DwHc;
  UINT32			Hprt0;

  //DEBUG((DEBUG_INFO, "DwHcGetRootHubPortStatus: PortNumber=%d\n", PortNumber));

  if (PortNumber > DWC2_HC_PORT)
  {
    return EFI_INVALID_PARAMETER;
  }

  if (PortStatus == NULL)
  {
    return EFI_INVALID_PARAMETER;
  }

  DwHc = DWHC_FROM_THIS(This);

  PortStatus->PortStatus = 0;
  PortStatus->PortChangeStatus = 0;
  
  Hprt0 = MmioRead32(DwHc->DwUsbBase + HPRT0);

  if (Hprt0 & DWC2_HPRT0_PRTCONNSTS)
  {
    PortStatus->PortStatus |= USB_PORT_STAT_CONNECTION;
  }

  if (Hprt0 & DWC2_HPRT0_PRTENA)
  {
    PortStatus->PortStatus |= USB_PORT_STAT_ENABLE;
  }

  if (Hprt0 & DWC2_HPRT0_PRTSUSP)
  {
    PortStatus->PortStatus |= USB_PORT_STAT_SUSPEND;
  }

  if (Hprt0 & DWC2_HPRT0_PRTOVRCURRACT)
  {
    PortStatus->PortStatus |= USB_PORT_STAT_OVERCURRENT;
  }

  if (Hprt0 & DWC2_HPRT0_PRTRST)
  {
    PortStatus->PortStatus |= USB_PORT_STAT_RESET;
  }

  if (Hprt0 & DWC2_HPRT0_PRTPWR)
  {
    PortStatus->PortStatus |= USB_PORT_STAT_POWER;
  }

  if (((Hprt0 & DWC2_HPRT0_PRTSPD_MASK) >> DWC2_HPRT0_PRTSPD_OFFSET) == DWC_HPRT0_PRTSPD_HIGH_SPEED)
  {
    PortStatus->PortStatus |= USB_PORT_STAT_HIGH_SPEED;
  }
  else if (((Hprt0 & DWC2_HPRT0_PRTSPD_MASK) >> DWC2_HPRT0_PRTSPD_OFFSET) == DWC_HPRT0_PRTSPD_LOW_SPEED)
  {
    PortStatus->PortStatus |= USB_PORT_STAT_LOW_SPEED;
  }

  PortStatus->PortStatus |= USB_PORT_STAT_HIGH_SPEED;

  if (Hprt0 & DWC2_HPRT0_PRTCONNDET)
  {
    PortStatus->PortChangeStatus |= USB_PORT_STAT_C_CONNECTION;
  }

  if (Hprt0 & DWC2_HPRT0_PRTOVRCURRCHNG)
  {
    PortStatus->PortChangeStatus |= USB_PORT_STAT_C_OVERCURRENT;
  }
  //DEBUG((DEBUG_INFO, "DwHcGetRootHubPortStatus: PortStatus->PortStatus=%x, PortStatus->PortChangeStatus=%x\n", PortStatus->PortStatus, PortStatus->PortChangeStatus));
  return EFI_SUCCESS;
}

/**
  Clears a feature for the specified root hub port.

  @param  This        A pointer to the EFI_USB2_HC_PROTOCOL instance.
  @param  PortNumber  Specifies the root hub port whose feature is requested to be cleared. This
                      value is zero based.
  @param  PortFeature Indicates the feature selector associated with the feature clear request.

  @retval EFI_SUCCESS           The feature specified by PortFeature was cleared for the USB
                                root hub port specified by PortNumber.
  @retval EFI_INVALID_PARAMETER PortNumber is invalid or PortFeature is invalid for this function.

**/
EFI_STATUS
EFIAPI
DwHcClearRootHubPortFeature (
  IN  EFI_USB2_HC_PROTOCOL *This,
  IN  UINT8                PortNumber,
  IN  EFI_USB_PORT_FEATURE PortFeature
  )
{
  DWUSB_OTGHC_DEV         *DwHc;
  UINT32                  Hprt0;
  EFI_STATUS              Status = EFI_SUCCESS;

  //DEBUG((DEBUG_INFO,"DwHcClearRootHubPortFeature: PortNumber=%d, PortFeature=%d\n",PortNumber,PortFeature));

  if (PortNumber > DWC2_HC_PORT) {
    Status = EFI_INVALID_PARAMETER;
    goto End;
  }

  DwHc = DWHC_FROM_THIS (This);
  
  Hprt0 = MmioRead32 (DwHc->DwUsbBase + HPRT0);
  
  switch (PortFeature)
  {
  case EfiUsbPortEnable:
    //DEBUG((DEBUG_INFO, "DwHcClearRootHubPortFeature: EfiUsbPortEnable\n"));
    MmioAndThenOr32(DwHc->DwUsbBase + HPRT0, ~(DWC2_HPRT0_PRTENA | DWC2_HPRT0_PRTCONNDET | DWC2_HPRT0_PRTENCHNG | DWC2_HPRT0_PRTOVRCURRCHNG), DWC2_HPRT0_PRTENA);
    break;
  case EfiUsbPortSuspend:
    //DEBUG((DEBUG_INFO, "DwHcClearRootHubPortFeature: EfiUsbPortSuspend\n"));
    MmioWrite32(DwHc->DwUsbBase + PCGCCTL, 0);
    MicroSecondDelay(50000);
    MmioAndThenOr32(DwHc->DwUsbBase + HPRT0, ~(DWC2_HPRT0_PRTENA | DWC2_HPRT0_PRTCONNDET | DWC2_HPRT0_PRTENCHNG | DWC2_HPRT0_PRTOVRCURRCHNG), DWC2_HPRT0_PRTRES);
    MicroSecondDelay(50000);
    MmioAnd32(DwHc->DwUsbBase + HPRT0, ~(DWC2_HPRT0_PRTSUSP | DWC2_HPRT0_PRTRES));
    break;
  case EfiUsbPortReset:
    //DEBUG((DEBUG_INFO, "DwHcClearRootHubPortFeature: EfiUsbPortReset\n"));
	MicroSecondDelay(10000);
    MmioAnd32(DwHc->DwUsbBase + HPRT0, ~(DWC2_HPRT0_PRTENA | DWC2_HPRT0_PRTCONNDET | DWC2_HPRT0_PRTENCHNG | DWC2_HPRT0_PRTOVRCURRCHNG | DWC2_HPRT0_PRTRST));
    break;
  case EfiUsbPortOwner:
    //DEBUG((DEBUG_INFO, "DwHcClearRootHubPortFeature: EfiUsbPortOwner\n"));
    break;
  case EfiUsbPortConnectChange:
    //DEBUG((DEBUG_INFO, "DwHcClearRootHubPortFeature: EfiUsbPortConnectChange\n"));
    MmioAndThenOr32(DwHc->DwUsbBase + HPRT0, ~(DWC2_HPRT0_PRTENA | DWC2_HPRT0_PRTCONNDET | DWC2_HPRT0_PRTOVRCURRCHNG), DWC2_HPRT0_PRTCONNDET);
    break;
  case EfiUsbPortEnableChange:
    //DEBUG((DEBUG_INFO, "DwHcClearRootHubPortFeature: EfiUsbPortEnableChange\n"));
    MmioAndThenOr32(DwHc->DwUsbBase + HPRT0, ~(DWC2_HPRT0_PRTENA | DWC2_HPRT0_PRTCONNDET | DWC2_HPRT0_PRTOVRCURRCHNG), DWC2_HPRT0_PRTENCHNG);
    break;
  case EfiUsbPortOverCurrentChange:
    //DEBUG((DEBUG_INFO, "DwHcClearRootHubPortFeature: EfiUsbPortOverCurrentChange\n"));
    MmioOr32(DwHc->DwUsbBase + HPRT0, DWC2_HPRT0_PRTOVRCURRCHNG);
    break;
  case EfiUsbPortPower:
    //DEBUG((DEBUG_INFO, "DwHcClearRootHubPortFeature: EfiUsbPortPower\n"));
    MmioAnd32(DwHc->DwUsbBase + HPRT0, ~(DWC2_HPRT0_PRTENA | DWC2_HPRT0_PRTCONNDET | DWC2_HPRT0_PRTENCHNG | DWC2_HPRT0_PRTOVRCURRCHNG | DWC2_HPRT0_PRTPWR));
    break;
  case EfiUsbPortSuspendChange:
    //DEBUG((DEBUG_INFO, "DwHcClearRootHubPortFeature: EfiUsbPortSuspendChange\n"));
    break;
  case EfiUsbPortResetChange:
    //DEBUG((DEBUG_INFO, "DwHcClearRootHubPortFeature: EfiUsbPortResetChange\n"));
    MicroSecondDelay(10000);
    break;
  default:
    Status = EFI_INVALID_PARAMETER;
    break;
  }

 End:
  return Status;
}

/**
  Sets a feature for the specified root hub port.

  @param  This        A pointer to the EFI_USB2_HC_PROTOCOL instance.
  @param  PortNumber  Specifies the root hub port whose feature is requested to be set. This
                      value is zero based.
  @param  PortFeature Indicates the feature selector associated with the feature set request.

  @retval EFI_SUCCESS           The feature specified by PortFeature was set for the USB
                                root hub port specified by PortNumber.
  @retval EFI_INVALID_PARAMETER PortNumber is invalid or PortFeature is invalid for this function.

**/
EFI_STATUS
EFIAPI
DwHcSetRootHubPortFeature (
  IN  EFI_USB2_HC_PROTOCOL *This,
  IN  UINT8                PortNumber,
  IN  EFI_USB_PORT_FEATURE PortFeature
  )
{
  DWUSB_OTGHC_DEV         *DwHc;
  UINT32                  Hprt0;
  EFI_STATUS              Status = EFI_SUCCESS;

  //DEBUG((DEBUG_INFO,"DwHcSetRootHubPortFeature: PortNumber=%d, PortFeature=%d\n",PortNumber,PortFeature));

  if (PortNumber > DWC2_HC_PORT) {
    Status = EFI_INVALID_PARAMETER;
    goto End;
  }

  DwHc = DWHC_FROM_THIS (This);

  switch (PortFeature) {
  case EfiUsbPortEnable:
	//DEBUG((DEBUG_INFO,"DwHcSetRootHubPortFeature: EfiUsbPortEnable\n"));
    break;
  case EfiUsbPortSuspend:
	//DEBUG((DEBUG_INFO,"DwHcSetRootHubPortFeature: EfiUsbPortSuspend\n"));
    MmioAndThenOr32 (DwHc->DwUsbBase + HPRT0, ~(DWC2_HPRT0_PRTENA | DWC2_HPRT0_PRTCONNDET | DWC2_HPRT0_PRTENCHNG | DWC2_HPRT0_PRTOVRCURRCHNG),DWC2_HPRT0_PRTSUSP);
    break;
  case EfiUsbPortReset:
	//DEBUG((DEBUG_INFO,"DwHcSetRootHubPortFeature: EfiUsbPortReset\n"));
	MmioAndThenOr32 (DwHc->DwUsbBase + HPRT0, ~(DWC2_HPRT0_PRTENA | DWC2_HPRT0_PRTCONNDET | DWC2_HPRT0_PRTENCHNG | DWC2_HPRT0_PRTOVRCURRCHNG),DWC2_HPRT0_PRTRST);
    break;
  case EfiUsbPortPower:
	//DEBUG((DEBUG_INFO,"DwHcSetRootHubPortFeature: EfiUsbPortPower\n"));
	MmioAndThenOr32 (DwHc->DwUsbBase + HPRT0, ~(DWC2_HPRT0_PRTENA | DWC2_HPRT0_PRTCONNDET | DWC2_HPRT0_PRTENCHNG | DWC2_HPRT0_PRTOVRCURRCHNG),DWC2_HPRT0_PRTPWR);
    MmioWrite32 (DwHc->DwUsbBase + HPRT0, Hprt0);
    break;
  case EfiUsbPortOwner:
	//DEBUG((DEBUG_INFO,"DwHcSetRootHubPortFeature: EfiUsbPortOwner\n"));
    break;
  default:
	//DEBUG((DEBUG_INFO,"DwHcSetRootHubPortFeature: EFI_INVALID_PARAMETER\n"));
    Status = EFI_INVALID_PARAMETER;
    break;
  }

 End:
  return Status;
}

VOID
DestroyDwUsbHc(
  IN  DWUSB_OTGHC_DEV *DwHc
  )
{
  UINT32 Pages;
  EFI_TPL PreviousTpl;
  DEBUG ((DEBUG_INFO, "DestroyDwUsbHc\n"));
  if (DwHc == NULL) {
    return;
  }

  if (DwHc->PeriodicEvent != NULL) {
    PreviousTpl = gBS->RaiseTPL(TPL_NOTIFY);
    gBS->CloseEvent (DwHc->PeriodicEvent);
    gBS->RestoreTPL(PreviousTpl);
  }

  if (DwHc->ExitBootServiceEvent != NULL) {
    gBS->CloseEvent (DwHc->ExitBootServiceEvent);
  }

  Pages = EFI_SIZE_TO_PAGES (DWC2_DATA_BUF_SIZE);
  DmaUnmap (DwHc->AlignedBufferMapping);
  DmaFreeBuffer (Pages, DwHc->AlignedBuffer);

  Pages = EFI_SIZE_TO_PAGES (DWC2_STATUS_BUF_SIZE);
  FreePages (DwHc->StatusBuffer, Pages);

  gBS->FreePool (DwHc);
}

STATIC
VOID
EFIAPI
DwUsbHcExitBootService (
  IN  EFI_EVENT Event,
  IN  VOID *Context
  )
{
  DEBUG ((DEBUG_INFO, "DwUsbHcExitBootService\n"));
  DWUSB_OTGHC_DEV *DwHc;

  DwHc = (DWUSB_OTGHC_DEV *) Context;
  DwHcQuiesce (DwHc);
}

STATIC
UINT32
FramesPassed (
  IN  DWUSB_OTGHC_DEV *DwHc
  )
{
  //DEBUG ((DEBUG_INFO, "FramesPassed\n"));
  UINT32 MicroFrameStart = DwHc->LastMicroFrame;
  UINT32 MicroFrameEnd = MmioRead32 (DwHc->DwUsbBase + HFNUM) & DWC2_HFNUM_FRNUM_MASK;
  UINT32 MicroFramesPassed;

  DwHc->LastMicroFrame = (UINT16) MicroFrameEnd;

  if (MicroFrameEnd < MicroFrameStart) {
    /*
     * Being delayed by 0x8000 microframes is 262 seconds.
     * Unlikely. Also, we can't really do better unless we
     * start polling time (which is tedious in EFI...).
     */
    MicroFrameEnd += DWC2_HFNUM_FRNUM_MASK + 1;
  }

  MicroFramesPassed = MicroFrameEnd - MicroFrameStart;

  /*
   * Round up. We're supposedly getting called every
   * 8 microframes anyway. This means we'll end up
   * going a bit faster, which is okay.
   */
  return ALIGN_VALUE(MicroFramesPassed, 8) / 8;
}

STATIC
VOID
DwHcPeriodicHandler (
                      IN EFI_EVENT Event,
                      IN VOID      *Context
                      )
{
  //DEBUG ((DEBUG_INFO, "DwHcPeriodicHandler\n"));
  UINT32 Frame;
  LIST_ENTRY *Entry;
  LIST_ENTRY *NextEntry;
  DWUSB_OTGHC_DEV *DwHc = Context;

  DwHc->CurrentFrame += FramesPassed(DwHc);
  Frame = DwHc->CurrentFrame;

  EFI_LIST_FOR_EACH_SAFE (Entry, NextEntry,
                          &DwHc->DeferredList) {
    DWUSB_DEFERRED_REQ *Req = EFI_LIST_CONTAINER (Entry, DWUSB_DEFERRED_REQ,
                                                  List);

    if (Frame >= Req->TargetFrame) {
      Req->TargetFrame = Frame + Req->FrameInterval;
      DwHcDeferredTransfer(Req);
    }
  }
}

EFI_STATUS
CreateDwcOtgDevice (
  IN  UINT32          DwUsbBase,
  OUT DWUSB_OTGHC_DEV **OutDwHc
  )
{
  DEBUG ((DEBUG_INFO, "CreateDwcOtgDevice\n"));
  DWUSB_OTGHC_DEV *DwHc;
  EFI_STATUS      Status;

  DwHc = AllocateZeroPool (sizeof(DWUSB_OTGHC_DEV));
  if (DwHc == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  DwHc->Signature                                 = DWUSB_OTGHC_DEV_SIGNATURE;
  DwHc->DwUsbOtgHc.GetCapability                  = DwHcGetCapability;
  DwHc->DwUsbOtgHc.Reset                          = DwHcReset;
  DwHc->DwUsbOtgHc.GetState                       = DwHcGetState;
  DwHc->DwUsbOtgHc.SetState                       = DwHcSetState;
  DwHc->DwUsbOtgHc.ControlTransfer                = DwHcControlTransfer;
  DwHc->DwUsbOtgHc.BulkTransfer                   = DwHcBulkTransfer;
  DwHc->DwUsbOtgHc.AsyncInterruptTransfer         = DwHcAsyncInterruptTransfer;
  DwHc->DwUsbOtgHc.SyncInterruptTransfer          = DwHcSyncInterruptTransfer;
  DwHc->DwUsbOtgHc.IsochronousTransfer            = DwHcIsochronousTransfer;
  DwHc->DwUsbOtgHc.AsyncIsochronousTransfer       = DwHcAsyncIsochronousTransfer;
  DwHc->DwUsbOtgHc.GetRootHubPortStatus           = DwHcGetRootHubPortStatus;
  DwHc->DwUsbOtgHc.SetRootHubPortFeature          = DwHcSetRootHubPortFeature;
  DwHc->DwUsbOtgHc.ClearRootHubPortFeature        = DwHcClearRootHubPortFeature;
  DwHc->DwUsbOtgHc.MajorRevision                  = 0x02;
  DwHc->DwUsbOtgHc.MinorRevision                  = 0x00;
  DwHc->DwUsbBase                                 = DwUsbBase;

  InitializeListHead (&DwHc->DeferredList);

  Status = gBS->CreateEventEx (
                               EVT_NOTIFY_SIGNAL,
                               TPL_NOTIFY,
                               DwUsbHcExitBootService,
                               DwHc,
                               &gEfiEventExitBootServicesGuid,
                               &DwHc->ExitBootServiceEvent
                               );

  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "CreateDwcOtgDevice: DwUsbHcExitBootService: %r\n", Status));
    return Status;
  }

  Status = gBS->CreateEvent (
                             EVT_TIMER | EVT_NOTIFY_SIGNAL,
                             TPL_NOTIFY,
                             DwHcPeriodicHandler,
                             DwHc, &DwHc->PeriodicEvent
                             );
  if (Status != EFI_SUCCESS) {
    DEBUG ((DEBUG_ERROR, "CreateDwcOtgDevice: DwHcPeriodicHandler: %r\n", Status));
    return Status;
  }

  Status = gBS->SetTimer (DwHc->PeriodicEvent, TimerPeriodic,
                          EFI_TIMER_PERIOD_MILLISECONDS(3));
  if (Status != EFI_SUCCESS) {
    DEBUG ((DEBUG_ERROR, "CreateDwcOtgDevice: PeriodicEvent: %r\n", Status));
    return Status;
  }

  *OutDwHc = DwHc;
  return EFI_SUCCESS;
}

VOID
DwHcQuiesce (
  IN  DWUSB_OTGHC_DEV *DwHc
  )
{
  DEBUG ((DEBUG_INFO, "DwHcQuiesce\n"));
  if (DwHc->PeriodicEvent != NULL) {
    EFI_TPL PreviousTpl;
    PreviousTpl = gBS->RaiseTPL(TPL_NOTIFY);
    gBS->CloseEvent (DwHc->PeriodicEvent);
    DwHc->PeriodicEvent = NULL;
    gBS->RestoreTPL(PreviousTpl);
  }

  MmioAndThenOr32 (DwHc->DwUsbBase + HPRT0,
                   ~(DWC2_HPRT0_PRTENA | DWC2_HPRT0_PRTCONNDET |
                     DWC2_HPRT0_PRTENCHNG | DWC2_HPRT0_PRTOVRCURRCHNG),
                   DWC2_HPRT0_PRTRST);

  MicroSecondDelay (10000);
  MmioWrite32 (DwHc->DwUsbBase + GRSTCTL, DWC2_GRSTCTL_CSFTRST);
  MicroSecondDelay (10000);
}
