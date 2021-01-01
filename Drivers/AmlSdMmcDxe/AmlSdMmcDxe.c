/** @file
  This file implement the MMC Host Protocol for the DesignWare eMMC.

  Copyright (c) 2014-2017, Linaro Limited. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include "AmlSdMmc.h"

#define MMC_CAP MMC_CAP_NONREMOVABLE
#define CARD_TYPE CARD_TYPE_SD

#define SDIO_PORT MESON_SDIO_PORT_B
// ----------------------------
// clock measure (4)
// ----------------------------
#define MSR_CLK_DUTY 0x21d6
#define MSR_CLK_REG0 0x21d7
#define MSR_CLK_REG1 0x21d8
#define MSR_CLK_REG2 0x21d9

#define PREG_PAD_GPIO3_EN_N 0x2015
#define PREG_PAD_GPIO3_O 0x2016
#define PREG_PAD_GPIO3_I 0x2017

#define PREG_PAD_GPIO5_EN_N 0x201b
#define PREG_PAD_GPIO5_O 0x201c
#define PREG_PAD_GPIO5_I 0x201d

#define PERIPHS_PIN_MUX_2 0x202e
#define PERIPHS_PIN_MUX_6 0x2032

#define FREQ_MIN 300000
#define FREQ_MAX 100000000

#define SDIO_BLOCKSIZE 512

EFI_MMC_HOST_PROTOCOL *gpMmcHost;
EFI_GUID mAmlSdMmcDevicePathGuid = EFI_CALLER_ID_GUID;
STATIC UINT32 mAmlSdMmcCommand;
STATIC UINT32 mAmlSdMmcArgument;
STATIC BOOLEAN init_flag = FALSE;
STATIC BOOLEAN is_fir_init = FALSE;
UINT8 mClkcDiv = 0;
UINT8 mBusWidth = 0;
UINT32 timeout_cnt = 0;
UINT32 mActureClock = 0;

BOOLEAN
AmlSdMmcIsCardPresent(
  IN  EFI_MMC_HOST_PROTOCOL   *This
  )
{
  DEBUG((DEBUG_INFO, "AmlSdMmcIsCardPresent\n"));

  MmioOr32(CBUS_REG_ADDR(PREG_PAD_GPIO5_EN_N), 1 << 29); //CARD_6
  return MmioRead32(CBUS_REG_ADDR(PREG_PAD_GPIO5_I)) & (1 << 29) ? FALSE : TRUE;
}

BOOLEAN
AmlSdMmcIsReadOnly(
  IN  EFI_MMC_HOST_PROTOCOL   *This
  )
{
  return FALSE;
}

/*soft reset after errors*/
VOID
AmlSdMmcSoftReset(
  VOID
  )
{
  DEBUG((DEBUG_INFO, "AmlSdMmcSoftReset\n"));

  SDIO_IRQ_CONFIG irqc;
  /*soft reset*/
  irqc.Bit.soft_reset = 1;
  MmioWrite32(SDIO_IRQC, irqc.Raw);

  MicroSecondDelay(2);
}

/*wait sdhc controller cmd send*/
EFI_STATUS
AmlSdMmcWaitReady(
  IN  UINT32                  Timeout
  )
{
  SDIO_STATUS_IRQ irqs;
  DEBUG((DEBUG_INFO, "AmlSdMmcWaitReady\n"));
  // write sdio_timing_out_count
  MmioWrite32(SDIO_IRQS, 0x1fff << 19);
  for (UINT32 i = 0; i < Timeout; i+=100)
  {
    irqs.Raw = MmioRead32(SDIO_IRQS);
    if (!irqs.Bit.sdio_cmd_busy)
    {
      return EFI_SUCCESS;
    }
	if(irqs.Bit.sdio_timing_out_count==0)
	{
	  // write sdio_timing_out_count
	  MmioWrite32(SDIO_IRQS, 0x1fff << 19);
	}
    MicroSecondDelay(100);
  }
  AmlSdMmcSoftReset();
  return EFI_TIMEOUT;
}

/*setup reg initial value*/
VOID
AmlSdMmcRegInit(
  VOID
  )
{
  DEBUG((DEBUG_INFO, "AmlSdMmcRegInit\n"));
  SDIO_STATUS_IRQ irqs;
  SDIO_CONFIG conf;

  /* write 1 clear bit8,9 */
  irqs.Bit.sdio_if_int = 1;
  irqs.Bit.sdio_cmd_int = 1;
  MmioWrite32(SDIO_IRQS, irqs.Raw);

  /* setup config */
  conf.Bit.sdio_write_crc_ok_status = 2;
  conf.Bit.sdio_write_nwr = 2;
  conf.Bit.m_endian = 3;
  conf.Bit.cmd_argument_bits = 39;
  conf.Bit.cmd_out_at_posedge = 0;
  conf.Bit.cmd_disable_crc = 0;
  conf.Bit.data_latch_at_negedge = 0;
  conf.Bit.cmd_clk_divide = CLK_DIV;
  conf.Bit.bus_width = mBusWidth;
  MmioWrite32(SDIO_CONF, conf.Raw);
}

EFI_STATUS
AmlSdMmcBuildDevicePath(
  IN  EFI_MMC_HOST_PROTOCOL     *This,
  OUT EFI_DEVICE_PATH_PROTOCOL  **DevicePath
  )
{
  EFI_DEVICE_PATH_PROTOCOL *NewDevicePathNode;

  NewDevicePathNode = CreateDeviceNode(HARDWARE_DEVICE_PATH, HW_VENDOR_DP, sizeof(VENDOR_DEVICE_PATH));
  CopyGuid(&((VENDOR_DEVICE_PATH *)NewDevicePathNode)->Guid, &mAmlSdMmcDevicePathGuid);

  *DevicePath = NewDevicePathNode;
  return EFI_SUCCESS;
}

UINT32
ClkUtilClkMsr(
  IN UINT32                     ClkMux
  )
{
  DEBUG((DEBUG_INFO, "ClkUtilClkMsr\n"));
#define MSR_CLK_REG0 0x21d7
  MmioWrite32(CBUS_REG_ADDR(MSR_CLK_REG0), 0);
  // Set the measurement gate to 64uS
  MmioAndThenOr32(CBUS_REG_ADDR(MSR_CLK_REG0), ~0xffff, 64 - 1);
  // Disable continuous measurement
  // Disable interrupts
  MmioAnd32(CBUS_REG_ADDR(MSR_CLK_REG0), ~((1 << 18) | (1 << 17)));
  MmioAndThenOr32(CBUS_REG_ADDR(MSR_CLK_REG0), ~(0x1f << 20), (ClkMux << 20) | (1 << 19) | (1 << 16));

  MmioRead32(CBUS_REG_ADDR(MSR_CLK_REG0));
  // Wait for the measurement to be done
  while (MmioRead32(CBUS_REG_ADDR(MSR_CLK_REG0)) & (1 << 31))
    ;
  // Disable measuring
  MmioAnd32(CBUS_REG_ADDR(MSR_CLK_REG0), ~(1 << 16));
  UINT32 msr = (MmioRead32(CBUS_REG_ADDR(MSR_CLK_REG2)) + 31) & 0x000FFFFF;
  // Return value in MHz*measured_val
  return (msr >> 6);
}

/*
1. clock valid range
2. clk config enable
3. select clock source
4. set clock divide
*/
EFI_STATUS
AmlSdMmcSetClkRate(
  IN  UINT32                     ClkIos
  )
{
  DEBUG((DEBUG_INFO, "AmlSdMmcSetClkRate, ClockIos=%d\n", ClkIos));

  SDIO_CONFIG conf;
  UINT32 ClkDiv;
  UINT32 ClkRate;
  conf.Raw = MmioRead32(SDIO_CONF);

#define CLK81 (7)
  ClkRate = ClkUtilClkMsr(CLK81) * 1000000;

  if (ClkIos < FREQ_MIN)
    ClkIos = FREQ_MIN;
  if (ClkIos > FREQ_MAX)
    ClkIos = FREQ_MAX;

  /*0: dont set it, 1:div2, 2:div3, 3:div4...*/
  ClkDiv = ClkRate / (2 * ClkIos);

  if (aml_card_type_sdio(CARD_TYPE) && ClkIos > 50000000) // if > 50MHz
    ClkDiv = 0;

  conf.Bit.cmd_clk_divide = ClkDiv;

  mClkcDiv = ClkDiv;
  MmioWrite32(SDIO_CONF, conf.Raw);

  MmioOr32(SDIO_IRQS, 1 << 15);

  mActureClock = ClkRate / (ClkDiv + 1) / 2;
  DEBUG((DEBUG_INFO, "ClkIos %d, ClkSrc %d, ClkDiv=%d, ActualClk=%d\n",
         ClkIos, ClkRate, ClkDiv, mActureClock));

  return EFI_SUCCESS;
}

EFI_STATUS
AmlSdMmcSetBusWidth (
  IN UINT32                     BusWidth
  )
{
  DEBUG((DEBUG_INFO, "AmlSdMmcSetBusWidth\n"));

  SDIO_CONFIG conf;
  conf.Raw = MmioRead32(SDIO_CONF);

  switch (BusWidth)
  {
  case 1:
    conf.Bit.bus_width = 0;
    break;
  case 4:
    conf.Bit.bus_width = 1;
    break;
  default:
    DEBUG((DEBUG_ERROR, "AmlSdMmcSetBusWidth: Error Data Bus\n"));
    return EFI_DEVICE_ERROR;
  }
  mBusWidth = conf.Bit.bus_width;
  MmioWrite32(SDIO_CONF, conf.Raw);
  DEBUG((DEBUG_INFO, "Bus Width Ios %d\n", BusWidth));
  return EFI_SUCCESS;
}

EFI_STATUS
AmlSdMmcSetPortIos (
  VOID
  )
{
  DEBUG((DEBUG_INFO, "AmlSdMmcSetPortIos\n"));
  SDIO_CONFIG conf;
  conf.Raw = MmioRead32(SDIO_CONF);

  if (aml_card_type_sdio(CARD_TYPE) && (mActureClock > 50000000))
  {                                     // if > 50MHz
    conf.Bit.data_latch_at_negedge = 1; //[19] //0
    conf.Bit.do_not_delay_data = 1;     //[18]
    conf.Bit.cmd_out_at_posedge = 0;    //[11]
  }
  else
  {
    conf.Bit.data_latch_at_negedge = 0; //[19] //0
    conf.Bit.do_not_delay_data = 0;     //[18]
    conf.Bit.cmd_out_at_posedge = 0;    //[11]
  }
  MmioWrite32(SDIO_CONF, conf.Raw);

  /* Setup Clock */
  conf.Bit.cmd_clk_divide = mClkcDiv;
  /* Setup Bus Width */
  conf.Bit.bus_width = mBusWidth;
  MmioWrite32(SDIO_CONF, conf.Raw);
  return EFI_SUCCESS;
}

EFI_STATUS
AmlSdMmcSetIos (
  IN  EFI_MMC_HOST_PROTOCOL     *This,
  IN  UINT32                    BusClockFreq,
  IN  UINT32                    BusWidth,
  IN  UINT32                    TimingMode
  )
{
  EFI_STATUS Status;
  /* Set Clock */
  Status = AmlSdMmcSetClkRate(BusClockFreq);
  if (EFI_ERROR(Status))
  {
    DEBUG((DEBUG_ERROR, "AmlSdMmcSetIos: Error Data Bus\n"));
    goto out;
  }
  /* Set Bus Width */
  Status = AmlSdMmcSetClkRate(BusWidth);
  if (EFI_ERROR(Status))
  {
    DEBUG((DEBUG_ERROR, "AmlSdMmcSetIos: Error Data Bus\n"));
    goto out;
  }
out:
  return Status;
}

VOID
AmlSdMmcPwr (
  IN BOOLEAN IsOn
  )
{
  if (IsOn)
  {
    MmioAnd32(CBUS_REG_ADDR(PREG_PAD_GPIO5_O), ~(1 << 31)); //CARD_8
    MmioAnd32(CBUS_REG_ADDR(PREG_PAD_GPIO5_EN_N), ~(1 << 31));
    MmioOr32(CBUS_REG_ADDR(PERIPHS_PIN_MUX_2), (0x3f << 10));
  }
  else
  {
    MmioOr32(CBUS_REG_ADDR(PREG_PAD_GPIO5_O), (1 << 31)); //CARD_8
    MmioAnd32(CBUS_REG_ADDR(PREG_PAD_GPIO5_EN_N), ~(1 << 31));
    MmioAnd32(CBUS_REG_ADDR(PERIPHS_PIN_MUX_2), ~(0x3f << 10));
  }
}

VOID
AmlSdMmcCsHigh (
  VOID
  )
{
  /*
   * Non-SPI hosts need to prevent chipselect going active during
   * GO_IDLE; that would put chips into SPI mode.  Remind them of
   * that in case of hardware that won't pull up DAT3/nCS otherwise.
   *
   * Now the way to accomplish this is: 
   * 1) set DAT3-pin as a GPIO pin(by pinmux), and pulls up;
   * 2) send CMD0;
   * 3) set DAT3-pin as a card-dat3-pin(by pinmux);
  */
  // clear bit[26] to make BOOT_3 used as a GPIO other than SD_D3_C
  MmioAnd32(CBUS_REG_ADDR(PERIPHS_PIN_MUX_6), ~(1 << 26));  // make BOOT_3 used as a GPIO other than SD_D3_C
  MmioOr32(CBUS_REG_ADDR(PREG_PAD_GPIO3_O), ~(1 << 3));     // pull up GPIO
  MmioAnd32(CBUS_REG_ADDR(PREG_PAD_GPIO3_EN_N), ~(1 << 3)); // enable gpio output
}

VOID
AmlSdMmcCsDontCare (
  VOID
  )
{
  MmioOr32(CBUS_REG_ADDR(PERIPHS_PIN_MUX_6), (1 << 26)); // make BOOT_3 used as SD_D3_C other than a GPIO
}

EFI_STATUS
AmlSdMmcNotifyState (
  IN  EFI_MMC_HOST_PROTOCOL     *This,
  IN  MMC_STATE                 State
  )
{
  EFI_STATUS Status;
  DEBUG((DEBUG_INFO, "AmlSdMmcNotifyState, State=%d\n", State));
  switch (State)
  {
  case MmcInvalidState:
    return EFI_INVALID_PARAMETER;
  case MmcHwInitializationState:

    /* Enable Power */
    AmlSdMmcPwr(FALSE);
    MicroSecondDelay(200000);
    AmlSdMmcPwr(TRUE);
    MicroSecondDelay(200000);
    /* Initialize Register Default Value */
    AmlSdMmcRegInit();
    ASSERT(!EFI_ERROR(Status));

    /* Setup clock that could not be higher than 400KHz */
    Status = AmlSdMmcSetClkRate(300000);

    MmioAnd32(CBUS_REG_ADDR(PREG_PAD_GPIO3_EN_N), ~(0x1 << 3));
    MmioOr32(CBUS_REG_ADDR(PREG_PAD_GPIO3_O), (0x1 << 3));
    MmioAnd32(CBUS_REG_ADDR(PERIPHS_PIN_MUX_6), ~(0x1 << 26)); //D3
    MicroSecondDelay(1000);

    AmlSdMmcCsHigh();
    is_fir_init = TRUE;
    break;
  case MmcIdleState:
    AmlSdMmcCsDontCare();
    MicroSecondDelay(2000);
    MmioOr32(CBUS_REG_ADDR(PERIPHS_PIN_MUX_6), (0x1 << 26)); //D3
    MicroSecondDelay(2000);
    break;
  case MmcReadyState:
    break;
  case MmcIdentificationState:
    break;
  case MmcStandByState:
    break;
  case MmcTransferState:
    break;
  case MmcSendingDataState:
    break;
  case MmcReceiveDataState:
    break;
  case MmcProgrammingState:
    break;
  case MmcDisconnectState:
    break;
  default:
    return EFI_INVALID_PARAMETER;
  }
  return EFI_SUCCESS;
}

EFI_STATUS
AmlCheckUnsupportCmd (
  IN  MMC_CMD                   Cmd
  )
{
  DEBUG((DEBUG_INFO, "AmlCheckUnsupportCmd\n"));
  if (Cmd == MMC_CMD3)
  { // CMD3 means the first time initialized flow is running
    is_fir_init = FALSE;
  }

  if (MMC_CAP == MMC_CAP_NONREMOVABLE)
  { // nonremovable device
    if (is_fir_init)
    { // init for the first time
      if (aml_card_type_sdio(CARD_TYPE))
      {
        return EFI_SUCCESS; //for 8189ETV needs ssdio reset when starts
      }
      else if (aml_card_type_mmc(CARD_TYPE))
      {
        if (Cmd == MMC_CMD5
            //|| Cmd == MMC_CMD52
            //|| Cmd == MMC_CMD53
            || Cmd == MMC_CMD8 || Cmd == MMC_CMD55)
        { // filter cmd 5/52/53/8/55 for an mmc device before init
          return EFI_INVALID_PARAMETER;
        }
      }
      else if (aml_card_type_sd(CARD_TYPE) || aml_card_type_non_sdio(CARD_TYPE))
      {
        if (Cmd == MMC_CMD5
            //|| Cmd == MMC_CMD52
            //|| Cmd == MMC_CMD53
        )
        { // filter cmd 5/52/53 for a sd card before init
          return EFI_INVALID_PARAMETER;
        }
      }
    }
  }
  else
  { // removable device
    // filter cmd 5/52/53 for a non-sdio device
    if (!aml_card_type_sdio(CARD_TYPE) && !aml_card_type_unknown(CARD_TYPE))
    {
      if (Cmd == MMC_CMD5
          //|| Cmd == MMC_CMD52
          //|| Cmd == MMC_CMD53
      )
      {
        return EFI_INVALID_PARAMETER;
      }
    }
  }
  // sdio_err("%s: cmd%d, card_type=%d\n", mmc_hostname(mmc), mrq->cmd->opcode, pdata->card_type);
  return EFI_SUCCESS;
}

EFI_STATUS
AmlSdMmcStartCmd (
  IN  MMC_CMD                   Cmd,
  IN  UINT32                    Argument,
  IN  UINT8                     Direction,
  IN  BOOLEAN                   HasData,
  IN  UINT32                    BlockCount,
  IN  UINT32                    BlockSize
  )
{
  DEBUG((DEBUG_INFO, "AmlSdMmcStartCmd\n"));

  EFI_STATUS Status;
  CMD_SEND send;
  SDIO_EXTENSION ext;
  SDIO_IRQ_CONFIG irqc;
  SDIO_STATUS_IRQ irqs;
  SDIO_MULT_CONFIG mult;
  UINT32 pack_size;

  ext.Raw = 0;
  irqc.Raw = MmioRead32(SDIO_IRQC);
  irqs.Raw = MmioRead32(SDIO_IRQS);
  mult.Raw = MmioRead32(SDIO_MULT);
  
  if (Cmd & MMC_CMD_LONG_RESPONSE)
  {
    /* 7(cmd)+120(respnse)+7(crc)-1 data */
    send.Bit.cmd_response_bits = 133;
    send.Bit.response_crc7_from_8 = 1;
  }
  else if (Cmd & MMC_CMD_WAIT_RESPONSE)
  {
    send.Bit.cmd_response_bits = 45;
  }
  else
  {
    send.Bit.cmd_response_bits = 0;
  }

  if (Cmd & MMC_CMD_NO_CRC_RESPONSE)
    send.Bit.response_do_not_have_crc7 = 1;

  if (Cmd == MMC_CMD12)
    send.Bit.check_busy_on_dat0 = 1;

  if (HasData)
  {
    DEBUG((DEBUG_INFO, "AmlSdMmcStartCmd: BlockSize=%d, BlockCount=%d\n", BlockSize, BlockCount));
    /* total package num */
    send.Bit.repeat_package_times = BlockCount - 1;
    if (BlockCount > 256)
    {
      DEBUG((DEBUG_ERROR, "AmlSdMmcStartCmd: BlockCount Too Large, BlockCount=%d", BlockCount));
      return EFI_INVALID_PARAMETER;
    }
    /* package size */
    if (mBusWidth) /* 0: 1bit, 1: 4bit */
      pack_size = BlockSize * 8 + (16 - 1) * 4;
    else
      pack_size = BlockSize * 8 + (16 - 1);
    ext.Bit.data_rw_number = pack_size;
    if (Direction)
      send.Bit.cmd_send_data = 1;
    else
      send.Bit.response_have_data = 1;
  }
  
  /* cmd index */
  send.Bit.cmd_command = 0x40 | MMC_GET_INDX(Cmd);
  send.Bit.use_int_window = 1;

  AmlSdMmcSoftReset();

  /* enable cmd irq */
  irqc.Bit.arc_cmd_int_en = 1;

  /* clear pending */
  irqs.Bit.sdio_cmd_int = 1;
  
  AmlSdMmcSetPortIos();

  mult.Bit.sdio_port_sel = SDIO_PORT;
  mult.Raw |= (1 << 31);
  MmioWrite32(SDIO_MULT, mult.Raw);
  MmioWrite32(SDIO_IRQS, irqs.Raw);
  MmioWrite32(SDIO_IRQC, irqc.Raw);
  //setup all reg to send cmd
  
  MmioWrite32(SDIO_EXT, ext.Raw);
  MmioWrite32(SDIO_ARGU, Argument);
  MmioWrite32(SDIO_SEND, send.Raw);
  
  DEBUG((DEBUG_INFO, "AmlSdMmcStartCmd,CmdId=%d, Send=0x%8x, Ext=0x%8x, IrqS=0x%8x, IrqC=0x%8x, Argument=0x%8x\n", MMC_GET_INDX(Cmd),send.Raw,ext.Raw,irqs.Raw,irqc.Raw,Argument));
  
  if (HasData)
  {
    Status=AmlSdMmcWaitReady(30000);
  }
  else
  {
    Status=AmlSdMmcWaitReady(10000);
  }
  if (MMC_GET_INDX(Cmd) == 0)
  {
    init_flag = TRUE;
  }
  
  return Status;
}

EFI_STATUS
AmlSdMmcSendCommand(
  IN  EFI_MMC_HOST_PROTOCOL     *This,
  IN  MMC_CMD                   Cmd,
  IN  UINT32                    Argument
  )
{
  DEBUG((DEBUG_INFO, "AmlSdMmcSendCommand, Ready Send CmdId=%d, Argument=0x%x\n",MMC_GET_INDX(Cmd),Argument));
  EFI_STATUS Status = EFI_SUCCESS;

  SDIO_IRQ_CONFIG irqc;

  irqc.Raw = MmioRead32(SDIO_IRQC);

  if (aml_card_type_non_sdio(CARD_TYPE))
  {
    irqc.Bit.arc_if_int_en = 0;
    MmioWrite32(SDIO_IRQC, irqc.Raw);
  }

  Status = AmlCheckUnsupportCmd(Cmd);
  if (EFI_ERROR(Status))
  {
    DEBUG((DEBUG_INFO, "AmlCheckUnsupportCmd fail\n"));
    return Status;
  }

  //only for SDCARD hotplag
  if ((!AmlSdMmcIsCardPresent(gpMmcHost) || (!init_flag && aml_card_type_non_sdio(CARD_TYPE))) && (MMC_GET_INDX(Cmd) == 0))
  {
    DEBUG((DEBUG_INFO, "AmlSdMmcSendCommand: SDCARD hotplag\n"));
    return EFI_DEVICE_ERROR;
  }

  if (Cmd == MMC_ACMD51 || Cmd == MMC_CMD6
  || Cmd == MMC_CMD17 || Cmd == MMC_CMD18 
  || Cmd == MMC_CMD24 || Cmd == MMC_CMD25  ) 
  {
	mAmlSdMmcCommand = Cmd;
    mAmlSdMmcArgument = Argument;
  }
  else
  {
    /*setup reg for all cmd*/
    Status = AmlSdMmcStartCmd(Cmd, Argument, 0, FALSE, 0, 0);
  }
  return Status;
}

EFI_STATUS
AmlSdMmcReceiveResponse(
  IN  EFI_MMC_HOST_PROTOCOL     *This,
  IN  MMC_RESPONSE_TYPE         Type,
  IN  UINT32                    *Buffer
  )
{
  DEBUG((DEBUG_INFO, "AmlSdMmcReceiveResponse\n"));
  if (Buffer == NULL)
  {
    return EFI_INVALID_PARAMETER;
  }

  UINT32 Response[4];

  SDIO_MULT_CONFIG mult;
  mult.Raw = MmioRead32(SDIO_MULT);
  mult.Bit.write_read_out_index = 1;
  mult.Bit.response_read_index = 0;

  MmioWrite32(SDIO_MULT, mult.Raw);

  if (Type == MMC_RESPONSE_TYPE_R2) /*136 bit*/
  {
    for (UINT8 i = 0; i <= 3; i++)
    {
      Response[3 - i] = MmioRead32(SDIO_ARGU);
      Buffer[0] = (Response[0] << 8) | ((Response[1] >> 24) & 0xff);
      Buffer[1] = (Response[1] << 8) | ((Response[2] >> 24) & 0xff);
      Buffer[2] = (Response[2] << 8) | ((Response[3] >> 24) & 0xff);
      Buffer[3] = (Response[3] << 8);
      DEBUG((DEBUG_INFO, "AmlSdMmcReceiveResponse: Response(R2)[%d] 0x%x\n", i, Buffer[i]));
    }
  }
  else /*48 bit*/
  {
    Buffer[0] = MmioRead32(SDIO_ARGU);
    DEBUG((DEBUG_INFO, "AmlSdMmcReceiveResponse: Response 0x%x\n", Buffer[0]));
  }
  return EFI_SUCCESS;
}

EFI_STATUS
AmlSdMmcReadBlockData(
  IN  EFI_MMC_HOST_PROTOCOL     *This,
  IN  EFI_LBA                   Lba,
  IN  UINTN                     Length,
  OUT UINT32                    *Buffer
  )
{
  EFI_STATUS                      Status;
  EFI_TPL                         Tpl;
  EFI_PHYSICAL_ADDRESS            BufferAddress;
  VOID                            *BufferMap;
  UINT32                          BlockSize;
  UINT32                          BlockCount;

  DEBUG((DEBUG_INFO, "AmlSdMmcReadBlockData\n"));

  Tpl = gBS->RaiseTPL(TPL_NOTIFY);
  
  for (UINT8 i = 0; i < 10; i++)
  {
    if (Length > SDIO_BLOCKSIZE)
    {
      BlockCount = Length / SDIO_BLOCKSIZE + 1;
    }
    else
    {
      BlockCount = 1;
    }
    
    if (mAmlSdMmcCommand== MMC_ACMD51) 
	{
  	  BlockSize=8;
	  Length=79;
	  Status = DmaAllocateBuffer (EfiBootServicesData, EFI_SIZE_TO_PAGES(Length), (VOID **) &Buffer);
      if (EFI_ERROR (Status)) {
        DEBUG ((DEBUG_ERROR, "CreateDwcOtgDevice: DmaAllocateBuffer: %r\n", Status));
        return Status;
      }
    }
    else
	{
  	  BlockSize=512;
    }
	
    Status = DmaMap(MapOperationBusMasterCommonBuffer, Buffer, &Length, &BufferAddress, &BufferMap);
    if (EFI_ERROR(Status))
    {
      DEBUG((DEBUG_ERROR, "AmlSdMmcReadBlockData: No pages available for DmaBuffer\n"));
      goto out;
    }
    MmioWrite32(SDIO_ADDR, (UINT32)BufferAddress);
  
    Status = AmlSdMmcStartCmd(mAmlSdMmcCommand, mAmlSdMmcArgument, MMC_IOBLOCKS_READ, TRUE, BlockCount, BlockSize);
    if (EFI_ERROR(Status))
    {
      DEBUG((DEBUG_ERROR, "Failed to read data, Status:%r\n",Status));
    }
	else
	{
	  break;
	}
  }

out:
  // FreePages and Restore Tpl
  DmaUnmap(BufferMap);
  gBS->RestoreTPL(Tpl);
  return Status;
}

EFI_STATUS
AmlSdMmcWriteBlockData(
  IN  EFI_MMC_HOST_PROTOCOL     *This,
  IN  EFI_LBA                   Lba,
  IN  UINTN                     Length,
  IN  UINT32                    *Buffer
  )
{
  EFI_STATUS                      Status;
  EFI_TPL                         Tpl;
  EFI_PHYSICAL_ADDRESS            BufferAddress;
  VOID *                          BufferMap;
  UINT32                          BlockCount;

  DEBUG((DEBUG_INFO, "AmlSdMmcWriteBlockData\n"));

  Tpl = gBS->RaiseTPL(TPL_NOTIFY);

  for (UINT8 i = 0; i < 10; i++)
  {
    if (Length > SDIO_BLOCKSIZE)
    {
      BlockCount = Length / SDIO_BLOCKSIZE + 1;
    }
    else
    {
      BlockCount = 1;
    }
    
    Status = DmaMap(MapOperationBusMasterRead, Buffer, &Length, &BufferAddress, &BufferMap);
    if (EFI_ERROR(Status))
    {
      DEBUG((DEBUG_ERROR, "AmlSdMmcReadBlockData: No pages available for DmaBuffer\n"));
      goto out;
    }
    MmioWrite32(SDIO_ADDR, (UINT32)BufferAddress);
    
    Status = AmlSdMmcStartCmd(mAmlSdMmcCommand, mAmlSdMmcArgument, MMC_IOBLOCKS_WRITE, TRUE, BlockCount, SDIO_BLOCKSIZE);
    if (EFI_ERROR(Status))
    {
      DEBUG((DEBUG_ERROR, "Failed to write data, mAmlSdMmcCommand:%x, mAmlSdMmcArgument:%x, Status:%r\n", mAmlSdMmcCommand, mAmlSdMmcArgument, Status));
    }
	else
	{
		break;
	}
  }

out:
  // FreePages and Restore Tpl
  DmaUnmap(BufferMap);
  gBS->RestoreTPL(Tpl);
  return Status;
}

BOOLEAN
AmlSdMmcIsMultiBlock(
  IN  EFI_MMC_HOST_PROTOCOL     *This
  )
{
  return TRUE;
}

EFI_MMC_HOST_PROTOCOL gMciHost = {
  MMC_HOST_PROTOCOL_REVISION,
  AmlSdMmcIsCardPresent,
  AmlSdMmcIsReadOnly,
  AmlSdMmcBuildDevicePath,
  AmlSdMmcNotifyState,
  AmlSdMmcSendCommand,
  AmlSdMmcReceiveResponse,
  AmlSdMmcReadBlockData,
  AmlSdMmcWriteBlockData,
  AmlSdMmcSetIos,
  AmlSdMmcIsMultiBlock
};

EFI_STATUS
AmlSdMmcDxeInitialize(
  IN  EFI_HANDLE                ImageHandle,
  IN  EFI_SYSTEM_TABLE          *SystemTable
  )
{
  EFI_STATUS Status;
  EFI_HANDLE Handle;

  Handle = NULL;

  DEBUG((DEBUG_BLKIO, "AmlSdMmcDxeInitialize()\n"));

  //Publish Component Name, BlockIO protocol interfaces
  Status = gBS->InstallMultipleProtocolInterfaces(
      &Handle,
      &gEfiMmcHostProtocolGuid, &gMciHost,
      NULL);
  ASSERT_EFI_ERROR(Status);

  return EFI_SUCCESS;
}
