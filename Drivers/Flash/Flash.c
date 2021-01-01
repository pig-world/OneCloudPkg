/** @file

  Copyright (c) 2008 - 2009, Apple Inc. All rights reserved.<BR>

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include "Flash.h"
static int cur_dev_num = -1;
struct mmc *mmc;
unsigned sdio_debug_1bit_flag;
/* frequency bases */
/* divided by 10 to be nice to platforms without floating point */
int fbase[] = {
    10000,
    100000,
    1000000,
    10000000,
};

/* Multiplier values for TRAN_SPEED.  Multiplied by 10 to be nice
 * to platforms without floating point.
 */
int multipliers[] = {
    0, /* reserved */
    10,
    12,
    13,
    15,
    20,
    25,
    30,
    35,
    40,
    45,
    50,
    55,
    60,
    70,
    80,
};
typedef struct {

  VENDOR_DEVICE_PATH        Guid;

  EFI_DEVICE_PATH_PROTOCOL  End;

} FLASH_DEVICE_PATH;



FLASH_DEVICE_PATH gDevicePath = {
  {
    { HARDWARE_DEVICE_PATH, HW_VENDOR_DP, { sizeof (VENDOR_DEVICE_PATH), 0 } },
    EFI_CALLER_ID_GUID
  },
  { END_DEVICE_PATH_TYPE, END_ENTIRE_DEVICE_PATH_SUBTYPE, { sizeof (EFI_DEVICE_PATH_PROTOCOL), 0} }
};

void aml_sd_cs_high(void) // chip select high
{
  DEBUG((EFI_D_INFO,"ENTER Function: %s\n",__FUNCTION__));
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
  clrbits_le32(P_PERIPHS_PIN_MUX_6, (1 << 26));  // make BOOT_3 used as a GPIO other than SD_D3_C
  setbits_le32(P_PREG_PAD_GPIO3_O, (1 << 3));    // pull up GPIO
  clrbits_le32(P_PREG_PAD_GPIO3_EN_N, (1 << 3)); // enable gpio output
}
//Clear response data buffer
static void sd_inand_clear_response(char *res_buf)
{
  DEBUG((EFI_D_INFO,"ENTER Function: %s\n",__FUNCTION__));
  int i;
  if (res_buf == NULL)
    return;

  for (i = 0; i < MAX_RESPONSE_BYTES; i++)
    res_buf[i] = 0;
}
void aml_sd_cs_dont_care(void) // chip select don't care
{
  DEBUG((EFI_D_INFO,"ENTER Function: %s\n",__FUNCTION__));
  setbits_le32(P_PERIPHS_PIN_MUX_6, (1 << 26)); // make BOOT_3 used as SD_D3_C other than a GPIO
}

static int sd_inand_check_response(struct mmc_cmd *cmd)
{
  DEBUG((EFI_D_INFO,"ENTER Function: %s\n",__FUNCTION__));
  int ret = SD_NO_ERROR;
  SD_Response_R1_t *r1 = (SD_Response_R1_t *)cmd->response;
  switch (cmd->resp_type)
  {
  case MMC_RSP_R1:
  case MMC_RSP_R1b:
    if (r1->card_status.OUT_OF_RANGE)
      return SD_ERROR_OUT_OF_RANGE;
    else if (r1->card_status.ADDRESS_ERROR)
      return SD_ERROR_ADDRESS;
    else if (r1->card_status.BLOCK_LEN_ERROR)
      return SD_ERROR_BLOCK_LEN;
    else if (r1->card_status.ERASE_SEQ_ERROR)
      return SD_ERROR_ERASE_SEQ;
    else if (r1->card_status.ERASE_PARAM)
      return SD_ERROR_ERASE_PARAM;
    else if (r1->card_status.WP_VIOLATION)
      return SD_ERROR_WP_VIOLATION;
    else if (r1->card_status.CARD_IS_LOCKED)
      return SD_ERROR_CARD_IS_LOCKED;
    else if (r1->card_status.LOCK_UNLOCK_FAILED)
      return SD_ERROR_LOCK_UNLOCK_FAILED;
    else if (r1->card_status.COM_CRC_ERROR)
      return SD_ERROR_COM_CRC;
    else if (r1->card_status.ILLEGAL_COMMAND)
      return SD_ERROR_ILLEGAL_COMMAND;
    else if (r1->card_status.CARD_ECC_FAILED)
      return SD_ERROR_CARD_ECC_FAILED;
    else if (r1->card_status.CC_ERROR)
      return SD_ERROR_CC;
    else if (r1->card_status.ERROR)
      return SD_ERROR_GENERAL;
    else if (r1->card_status.CID_CSD_OVERWRITE)
      return SD_ERROR_CID_CSD_OVERWRITE;
    else if (r1->card_status.AKE_SEQ_ERROR)
      return SD_ERROR_AKE_SEQ;
    break;
  default:
    break;
  }
  return ret;
}

#define be32dec(x) \
	((UINT32)( \
		(((UINT32)(x) & (UINT32)0x000000ffUL) << 24) | \
		(((UINT32)(x) & (UINT32)0x0000ff00UL) <<  8) | \
		(((UINT32)(x) & (UINT32)0x00ff0000UL) >>  8) | \
		(((UINT32)(x) & (UINT32)0xff000000UL) >> 24) ))

int aml_sd_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd, struct mmc_data *data)
{
  DEBUG((EFI_D_INFO,"ENTER Function: aml_sd_send_cmd\n"));
  int ret = SD_NO_ERROR, num_res;
  unsigned buffer = 0;
  unsigned int cmd_send = 0;
  SDHW_CMD_Send_Reg_t *cmd_send_reg = (void *)&cmd_send;

  DEBUG((EFI_D_INFO, "cmd=%d", cmd->cmdidx));
  cmd_send_reg->cmd_data = 0x40 | cmd->cmdidx;
  cmd_send_reg->use_int_window = 1;

  unsigned int cmd_ext = 0;
  SDHW_Extension_Reg_t *cmd_ext_reg = (void *)&cmd_ext;

  /* check read/write address overflow */
  switch (cmd->cmdidx)
  {
  case MMC_CMD_READ_SINGLE_BLOCK:
  case MMC_CMD_READ_MULTIPLE_BLOCK:
  case MMC_CMD_WRITE_SINGLE_BLOCK:
  case MMC_CMD_WRITE_MULTIPLE_BLOCK:
    if (mmc->high_capacity)
      ret = mmc->capacity / mmc->read_bl_len <= cmd->cmdarg;
    else
      ret = mmc->capacity <= cmd->cmdarg;
    if (ret)
      return -1;
    else
      break;
  }

  sd_inand_clear_response(cmd->response);
  switch (cmd->resp_type)
  {
  case MMC_RSP_R1:
  case MMC_RSP_R1b:
  case MMC_RSP_R3:
  case MMC_RSP_R6:
  case MMC_RSP_R7:
    cmd_send_reg->cmd_res_bits = 45; // RESPONSE	have 7(cmd)+32(respnse)+7(crc)-1 data
    break;
  case MMC_RSP_R2:
    cmd_send_reg->cmd_res_bits = 133; // RESPONSE	have 7(cmd)+120(respnse)+7(crc)-1 data
    cmd_send_reg->res_crc7_from_8 = 1;
    break;
  default:
    cmd_send_reg->cmd_res_bits = 0; // NO_RESPONSE
    break;
  }

  switch (cmd->cmdidx)
  {
  case MMC_CMD_READ_SINGLE_BLOCK:
  case MMC_CMD_READ_MULTIPLE_BLOCK:
  case MMC_CMD_SEND_EXT_CSD: //same as: SD_CMD_SEND_IF_COND
  case SD_CMD_SWITCH_FUNC:   //same as: MMC_CMD_SWITCH
    if (!data)
      break;
    //        dcache_flush();

    cmd_send_reg->res_with_data = 1;
    cmd_send_reg->repeat_package_times = data->blocks - 1;
    if (mmc->bus_width == SD_BUS_WIDE)
      cmd_ext_reg->data_rw_number = data->blocksize * 8 + (16 - 1) * 4;
    else
      cmd_ext_reg->data_rw_number = data->blocksize * 8 + 16 - 1;
    //buffer = dma_map_single((void*)data->dest,data->blocks*data->blocksize,DMA_FROM_DEVICE);
    //buffer = data->dest;
    //dcache_invalid_range(buffer,data->blocks<<9);
    break;
  case MMC_CMD_WRITE_SINGLE_BLOCK:
  case MMC_CMD_WRITE_MULTIPLE_BLOCK:

    cmd_send_reg->cmd_send_data = 1;
    cmd_send_reg->repeat_package_times = data->blocks - 1;
    if (mmc->bus_width == SD_BUS_WIDE)
      cmd_ext_reg->data_rw_number = data->blocksize * 8 + (16 - 1) * 4;
    else
      cmd_ext_reg->data_rw_number = data->blocksize * 8 + 16 - 1;
    //buffer = dma_map_single((void*)data->src,data->blocks*data->blocksize,DMA_TO_DEVICE);//(char *)data->src;
    //        dcache_clean_range(buffer,data->blocks<<9);
    //        dcache_flush();
    break;
  case SD_CMD_APP_SEND_SCR:
    cmd_send_reg->res_with_data = 1;
    if (mmc->bus_width == SD_BUS_WIDE)
      cmd_ext_reg->data_rw_number = data->blocksize * 8 + (16 - 1) * 4;
    else
      cmd_ext_reg->data_rw_number = data->blocksize * 8 + 16 - 1;
    //buffer = (char *)data->src;
    //dcache_flush();
    //buffer = dma_map_single(data->dest,cmd_ext_reg->data_rw_number,DMA_BIDIRECTIONAL);//(char *)data->src;
    break;
  default:
    break;
  }

  //cmd with R1b
  switch (cmd->cmdidx)
  {
  case MMC_CMD_STOP_TRANSMISSION:
    cmd_send_reg->check_dat0_busy = 1;
    break;
  default:
    break;
  }

  //cmd with R3
  switch (cmd->cmdidx)
  {
  case MMC_CMD_SEND_OP_COND:
  case SD_CMD_APP_SEND_OP_COND:
    cmd_send_reg->res_without_crc7 = 1;
    break;
  default:
    break;
  }

  unsigned int timeout, timeout_count, repeat_time = 0;

  if (cmd_send_reg->cmd_send_data)
  {
    if (cmd->cmdidx == MMC_CMD_WRITE_MULTIPLE_BLOCK)
      timeout = SD_WRITE_BUSY_COUNT * (data->blocks);
    else
      timeout = SD_WRITE_BUSY_COUNT;
  }
  else
  {
    if (cmd->cmdidx == MMC_CMD_READ_MULTIPLE_BLOCK)
      timeout = SD_READ_BUSY_COUNT * (data->blocks);
    else
      timeout = SD_READ_BUSY_COUNT;
  }

  if (cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION)
    timeout = 2000;

  unsigned int status_irq = 0;
  SDIO_Status_IRQ_Reg_t *status_irq_reg = (void *)&status_irq;
  unsigned int irq_config = 0;
  MSHW_IRQ_Config_Reg_t *irq_config_reg = (void *)&irq_config;

CMD_RETRY:
  status_irq_reg->if_int = 1;
  status_irq_reg->cmd_int = 1;
  DEBUG((EFI_D_INFO, "PREG_SDIO_STAT_IRQ=%x PREG_SDIO_MEM_ADDR=%x PREG_SDIO_CMD_SEND=%x", status_irq, buffer, cmd_send));
  DEBUG((EFI_D_INFO, "PREG_SDIO_CMD_ARG =%x      PREG_SDIO_EXT=%x", cmd->cmdarg, cmd_ext));

  MmioWrite32(PREG_SDIO_STAT_IRQ, status_irq | (0x1fff << 19));

  MmioWrite32(PREG_SDIO_CMD_ARG, cmd->cmdarg);
  MmioWrite32(PREG_SDIO_EXT, cmd_ext);
  MmioWrite32(PREG_SDIO_MEM_ADDR, (unsigned int)buffer);
  MmioWrite32(PREG_SDIO_CMD_SEND, cmd_send);

  timeout_count = 0;
  while (1)
  {
    status_irq = MmioRead32(PREG_SDIO_STAT_IRQ);
    if (!status_irq_reg->cmd_busy && status_irq_reg->cmd_int)
      break;

    if ((++timeout_count) > timeout)
    {
      if (!cmd->flags)
        return TIMEOUT;

      irq_config_reg->soft_reset = 1;
      MmioWrite32(PREG_SDIO_IRQ_CFG, irq_config);

      if ((++repeat_time) > SD_RETRY_COUNT)
        return TIMEOUT;
      goto CMD_RETRY;
    }

    if (cmd_send_reg->cmd_send_data)
      MicroSecondDelay(1);
    if (cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION)
      MicroSecondDelay(1);
  }

  if (cmd_send_reg->cmd_res_bits && !cmd_send_reg->res_without_crc7 && !status_irq_reg->res_crc7_ok)
    return SD_ERROR_COM_CRC;

  switch (cmd->resp_type)
  {
  case MMC_RSP_R1:
  case MMC_RSP_R1b:
  case MMC_RSP_R3:
  case MMC_RSP_R6:
  case MMC_RSP_R7:
    num_res = RESPONSE_R1_R3_R6_R7_LENGTH;
    break;
  case MMC_RSP_R2:
    num_res = RESPONSE_R2_CID_CSD_LENGTH;
    break;
  default:
    num_res = RESPONSE_R4_R5_NONE_LENGTH;
    break;
  }

  /*	switch (cmd->cmdidx) {
	case MMC_CMD_READ_SINGLE_BLOCK:
	case MMC_CMD_READ_MULTIPLE_BLOCK:
	case MMC_CMD_SEND_EXT_CSD:					//same as: SD_CMD_SEND_IF_COND
	case SD_CMD_SWITCH_FUNC:					//same as: MMC_CMD_SWITCH
	case SD_CMD_APP_SEND_SCR:
		if(!data)
			break;
        dma_unmap_single((void*)data->dest,data->blocks*data->blocksize,buffer);
		break;
	case MMC_CMD_WRITE_SINGLE_BLOCK:
	case MMC_CMD_WRITE_MULTIPLE_BLOCK:
	    dma_unmap_single((void*)data->src,data->blocks*data->blocksize,buffer);
		break;
	default:
		break;
	}*/
  //  dcache_flush();
  if (num_res > 0)
  {
    unsigned int multi_config = 0;
    SDIO_Multi_Config_Reg_t *multi_config_reg = (void *)&multi_config;
    multi_config_reg->write_read_out_index = 1;
    MmioWrite32(PREG_SDIO_MULT_CFG, multi_config);
    num_res--; // Minus CRC byte
  }
  unsigned int data_temp;
  unsigned int loop_num = (num_res + 3 - 1) / 4;
  while (num_res > 0)
  {
    data_temp = MmioRead32(PREG_SDIO_CMD_ARG);
    if (num_res <= 1)
      break;
    cmd->response[--num_res - 1] = data_temp & 0xFF;
    if (num_res <= 1)
      break;
    cmd->response[--num_res - 1] = (data_temp >> 8) & 0xFF;
    if (num_res <= 1)
      break;
    cmd->response[--num_res - 1] = (data_temp >> 16) & 0xFF;
    if (num_res <= 1)
      break;
    cmd->response[--num_res - 1] = (data_temp >> 24) & 0xFF;
  }
  while (loop_num--)
  {
    ((UINT32 *)cmd->response)[loop_num] = be32dec(((UINT32 *)cmd->response)[loop_num]);
  }

  //check_response
  ret = sd_inand_check_response(cmd);
  if (ret)
    return ret;

  //cmd with adtc
  switch (cmd->cmdidx)
  {
  case MMC_CMD_READ_SINGLE_BLOCK:
  case MMC_CMD_READ_MULTIPLE_BLOCK:
  case MMC_CMD_SEND_EXT_CSD: //same as SD_CMD_SEND_IF_COND
  case SD_CMD_SWITCH_FUNC:   //same as: MMC_CMD_SWITCH
    if (!data)
      break;
    if (!status_irq_reg->data_read_crc16_ok)
      return SD_ERROR_DATA_CRC;
    break;
  case MMC_CMD_WRITE_SINGLE_BLOCK:
  case MMC_CMD_WRITE_MULTIPLE_BLOCK:
    if (!status_irq_reg->data_write_crc16_ok)
      return SD_ERROR_DATA_CRC;
    break;
  case SD_CMD_APP_SEND_SCR:
    if (!status_irq_reg->data_read_crc16_ok)
      return SD_ERROR_DATA_CRC;
    break;
  default:
    break;
  }

  return SD_NO_ERROR;
}
EFI_STATUS
EFIAPI
NandFlashReset(
    IN EFI_BLOCK_IO_PROTOCOL *This,
    IN BOOLEAN ExtendedVerification)
{
  DEBUG((EFI_D_INFO,"ENTER Function: %s\n",__FUNCTION__));
  //Send RESET command to device.
  //reset equal mmc_go_idle
  struct mmc_cmd cmd;
  int err;

  CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO3_EN_N, (0x1 << 3));
  SET_CBUS_REG_MASK(PREG_PAD_GPIO3_O, (0x1 << 3));
  CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_6, (0x1 << 26)); //D3

  MicroSecondDelay(1);

  aml_sd_cs_high();

  cmd.cmdidx = MMC_CMD_GO_IDLE_STATE;
  cmd.cmdarg = 0;
  cmd.resp_type = MMC_RSP_NONE;
  cmd.flags = 0;

  err = aml_sd_send_cmd(mmc, &cmd, NULL);

  aml_sd_cs_dont_care();

  if (err)
    return EFI_DEVICE_ERROR;

  MicroSecondDelay(2);
  SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_6, (0x1 << 26)); //D3
  MicroSecondDelay(2);

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
NandFlashReadBlocks(
    IN EFI_BLOCK_IO_PROTOCOL *This,
    IN UINT32 MediaId,
    IN EFI_LBA Lba,
    IN UINTN BufferSize,
    OUT VOID *Buffer)
{
  DEBUG((EFI_D_INFO,"ENTER Function: %s\n",__FUNCTION__));
  struct mmc_cmd cmd;
  struct mmc_data data;
  //need count blkcnt
  int blkcnt = 0;
  if (BufferSize % (mmc->read_bl_len))
    blkcnt = BufferSize / (mmc->read_bl_len) + 1;
  else
    blkcnt = BufferSize / (mmc->read_bl_len);
  blkcnt = (blkcnt > 256) ? 256 : blkcnt;

  if (blkcnt > 1)
    cmd.cmdidx = MMC_CMD_READ_MULTIPLE_BLOCK;
  else
    cmd.cmdidx = MMC_CMD_READ_SINGLE_BLOCK;

  if (mmc->high_capacity)
    cmd.cmdarg = Lba;
  else
    cmd.cmdarg = Lba * (mmc->read_bl_len);

  cmd.resp_type = MMC_RSP_R1;
  cmd.flags = 0;

  data.dest = Buffer;
  data.blocks = blkcnt;
  data.blocksize = mmc->read_bl_len;
  data.flags = MMC_DATA_READ;

  if (aml_sd_send_cmd(mmc, &cmd, &data))
    return 0;

  if (blkcnt > 1)
  {
    cmd.cmdidx = MMC_CMD_STOP_TRANSMISSION;
    cmd.cmdarg = 0;
    cmd.resp_type = MMC_RSP_R1b;
    cmd.flags = 0;
    if (aml_sd_send_cmd(mmc, &cmd, NULL))
    {
      DEBUG((EFI_D_INFO, "mmc fail to send stop cmd\n"));
      return 0;
    }
  }

  return blkcnt;
}

EFI_STATUS
EFIAPI
NandFlashWriteBlocks(
    IN EFI_BLOCK_IO_PROTOCOL *This,
    IN UINT32 MediaId,
    IN EFI_LBA Lba,
    IN UINTN BufferSize,
    IN VOID *Buffer)
{
  DEBUG((EFI_D_INFO,"ENTER Function: %s\n",__FUNCTION__));
  struct mmc_cmd cmd;
  struct mmc_data data;
  int blkcnt = 0;
  if (BufferSize % mmc->read_bl_len)
    blkcnt = BufferSize / mmc->read_bl_len;
  else
    blkcnt = BufferSize / mmc->read_bl_len + 1;
  if ((Lba + blkcnt) > mmc->block_dev.lba)
  {
    DEBUG((EFI_D_INFO, "MMC: block number 0x%lx exceeds max(0x%lx)\n",
           Lba + blkcnt, mmc->block_dev.lba));
    return EFI_DEVICE_ERROR;
  }

  if (blkcnt > 1)
    cmd.cmdidx = MMC_CMD_WRITE_MULTIPLE_BLOCK;
  else
    cmd.cmdidx = MMC_CMD_WRITE_SINGLE_BLOCK;

  if (mmc->high_capacity)
    cmd.cmdarg = Lba;
  else
    cmd.cmdarg = Lba * mmc->write_bl_len;

  cmd.resp_type = MMC_RSP_R1;
  cmd.flags = 0;

  data.src = Buffer;
  data.blocks = blkcnt;
  data.blocksize = mmc->write_bl_len;
  data.flags = MMC_DATA_WRITE;

  if (aml_sd_send_cmd(mmc, &cmd, &data))
  {
    DEBUG((EFI_D_INFO, "mmc write failed\n"));
    return EFI_DEVICE_ERROR;
  }

  if (blkcnt > 1)
  {
    cmd.cmdidx = MMC_CMD_STOP_TRANSMISSION;
    cmd.cmdarg = 0;
    cmd.resp_type = MMC_RSP_R1b;
    cmd.flags = 0;
    if (aml_sd_send_cmd(mmc, &cmd, NULL))
    {
      DEBUG((EFI_D_INFO, "mmc fail to send stop cmd\n"));
      return EFI_DEVICE_ERROR;
    }
  }

  return EFI_SUCCESS;
}
int mmc_send_ext_csd(struct mmc *mmc, char *ext_csd)
{
  DEBUG((EFI_D_INFO,"ENTER Function: %s\n",__FUNCTION__));
  struct mmc_cmd cmd;
  struct mmc_data data;
  int err;

  /*delay for some emmc init fail*/
  MicroSecondDelay(1);

  /* Get the Card Status Register */
  cmd.cmdidx = MMC_CMD_SEND_EXT_CSD;
  cmd.resp_type = MMC_RSP_R1;
  cmd.cmdarg = 0;
  cmd.flags = 0;

  data.dest = ext_csd;
  data.blocks = 1;
  data.blocksize = 512;
  data.flags = MMC_DATA_READ;

  err = aml_sd_send_cmd(mmc, &cmd, &data);

  return err;
}
#define TURN_TRANSFER_UNIT(val, is_high_cap, blk_len) ((is_high_cap) ? (val) : ((val)*blk_len))
UINT64 __mmc_berase(struct mmc *mmc, UINT64 start, UINT64 blkcnt)
{
  DEBUG((EFI_D_INFO,"ENTER Function: %s\n",__FUNCTION__));
  struct mmc_cmd cmd;
  int err;
  UINT64 end, max_end;

  max_end = mmc->capacity / 512 - 1; //max end address in block
  end = start + blkcnt - 1;

  DEBUG((EFI_D_INFO, "card_type:mmc or emmc, "));

  int erase_unit;
  char ext_csd[512];

  err = mmc_send_ext_csd(mmc, ext_csd);
  if (err)
    return err;

  if (ext_csd[175])
  {                                                              //erase_group_def
    int erase_unit_byte = ((unsigned)ext_csd[224]) * 512 * 1024; //byte cnt
    erase_unit = erase_unit_byte / mmc->write_bl_len;            //blk cnt
    DEBUG((EFI_D_INFO, "ext_csd:erase_unit = %d, ", erase_unit));
  }
  else
  {
    int erase_gsize = (mmc->csd[2] >> 10) & 0x1f;
    int erase_gmult = (mmc->csd[2] >> 5) & 0x1f;
    erase_unit = (erase_gsize + 1) * (erase_gmult + 1); //blk cnt
    DEBUG((EFI_D_INFO, "csd:erase_unit = %d, ", erase_unit));
  }

  start = start - start % erase_unit;
  end = blkcnt ? ((end / erase_unit + 1) * erase_unit - 1) : max_end;
  if (end > max_end)
  {
    DEBUG((EFI_D_INFO, "MMC: group number 0x%lx exceeds max(0x%lx)\n",
           end, max_end));
    return 1;
  }

  cmd.cmdidx = MMC_TAG_ERASE_GROUP_START;
  cmd.resp_type = MMC_RSP_R1;
  cmd.cmdarg = TURN_TRANSFER_UNIT(start, mmc->high_capacity, mmc->write_bl_len);
  cmd.flags = 0;
  err = aml_sd_send_cmd(mmc, &cmd, NULL);
  if (err)
    return err;

  cmd.cmdidx = MMC_TAG_ERASE_GROUP_END;
  cmd.resp_type = MMC_RSP_R1;
  cmd.cmdarg = TURN_TRANSFER_UNIT(end, mmc->high_capacity, mmc->write_bl_len);
  cmd.flags = 0;
  err = aml_sd_send_cmd(mmc, &cmd, NULL);
  if (err)
    return err;

  DEBUG((EFI_D_INFO, "is being erased ...\n"));
  cmd.cmdidx = SD_MMC_ERASE;
  cmd.resp_type = MMC_RSP_R1b;
  cmd.cmdarg = 0;
  cmd.flags = 0;
  err = aml_sd_send_cmd(mmc, &cmd, NULL);
  if (err)
    return err;

  unsigned int timeout = 0;
  UINT32 *res = (UINT32 *)(&(cmd.response[0]));
  do
  {
    cmd.cmdidx = MMC_CMD_SEND_STATUS;
    cmd.cmdarg = mmc->rca << 16;
    cmd.resp_type = MMC_RSP_R1;
    cmd.flags = 0;

    err = aml_sd_send_cmd(mmc, &cmd, NULL);
    if (err || (*res & 0xFDF92000))
    {
      DEBUG((EFI_D_INFO, "error %d requesting status %#x\n", err, *res));
      return -1;
    }

    timeout++;
    if (timeout > 10 * 60 * 1000)
      return -1;
    MicroSecondDelay(1);
  } while (!(*res & R1_READY_FOR_DATA) || (R1_CURRENT_STATE(*res) == R1_STATE_PRG));

  return 0;
}

UINT64 mmc_berase(int dev_num, UINT64 start, UINT64 blkcnt)
{
  DEBUG((EFI_D_INFO,"ENTER Function: %s\n",__FUNCTION__));
  int ret = 0;

  if (!mmc)
    return 0;

  // if (emmckey_is_access_range_legal(mmc, start, blkcnt) == false) {
  //     return blkcnt; // illegal error
  // }

  ret = __mmc_berase(mmc, start, blkcnt);

  DEBUG((EFI_D_INFO, "erase %#lx --> %#lx %s\n", start, start + blkcnt - 1, (ret == 0) ? "OK" : "ERROR"));

  return ret;
}

EFI_STATUS
EFIAPI
NandFlashFlushBlocks(
    IN EFI_BLOCK_IO_PROTOCOL *This)
{
  return EFI_SUCCESS;
}

EFI_BLOCK_IO_MEDIA gNandFlashMedia = {
    SIGNATURE_32('n', 'a', 'n', 'd'), // MediaId
    FALSE,                            // RemovableMedia
    TRUE,                             // MediaPresent
    FALSE,                            // LogicalPartition
    FALSE,                            // ReadOnly
    FALSE,                            // WriteCaching
    0,                                // BlockSize
    2,                                // IoAlign
    0,                                // Pad
    0                                 // LastBlock
};
int cpu_sdio_init(unsigned port)
{
  DEBUG((EFI_D_INFO,"ENTER Function: cpu_sdio_init\n"));
  //printf("inand sdio  port:%d\n",port);
  switch (port)
  {
  case SDIO_PORT_A:
    setbits_le32(P_PERIPHS_PIN_MUX_8, 0x3f);
    break;

  case SDIO_PORT_B:
    setbits_le32(P_PERIPHS_PIN_MUX_2, 0x3f << 10);
    break;
  case SDIO_PORT_C: //SDIOC GPIOB_2~GPIOB_7
    clrbits_le32(P_PERIPHS_PIN_MUX_2, (0x1f << 22));
    setbits_le32(P_PERIPHS_PIN_MUX_6, (0x3f << 26));
    //printf("inand sdio  port:%d\n",port);
    break;
  case SDIO_PORT_XC_A:
    break;
  case SDIO_PORT_XC_B:
    //sdxc controller can't work
    //setbits_le32(P_PERIPHS_PIN_MUX_2,(0xf<<4));
    break;
  case SDIO_PORT_XC_C:
    break;
  default:
    return -1;
  }
  DEBUG((EFI_D_INFO,"LEAVE Function: cpu_sdio_init\n"));
  return 0;
}
int sdio_detect(unsigned port)
{
  DEBUG((EFI_D_INFO,"ENTER Function: sdio_detect\n"));
  int ret;
  switch (port)
  {
  case SDIO_PORT_A:
    break;
  case SDIO_PORT_B:
    setbits_le32(P_PREG_PAD_GPIO5_EN_N, 1 << 29); //CARD_6
    ret = MmioRead32(P_PREG_PAD_GPIO5_I) & (1 << 29) ? 0 : 1;

    if ((MmioRead32(P_PERIPHS_PIN_MUX_8) & (3 << 9)))
    { //if uart pinmux set, debug board in
      if (!(MmioRead32(P_PREG_PAD_GPIO0_I) & (1 << 22)))
      {
        DEBUG((EFI_D_INFO, "sdio debug board detected, sd card with 1bit mode\n"));
        sdio_debug_1bit_flag = 1;
      }
      else
      {
        DEBUG((EFI_D_INFO, "sdio debug board detected, no sd card in\n"));
        sdio_debug_1bit_flag = 0;
        return 1;
      }
    }

    break;
  case SDIO_PORT_C:
    break;
  case SDIO_PORT_XC_A:
    break;
  case SDIO_PORT_XC_B:
    break;
  case SDIO_PORT_XC_C:
    break;
  default:
    break;
  }

  return 0;
}
int sdio_init(unsigned port)
{
  DEBUG((EFI_D_INFO,"ENTER Function: sdio_init\n"));
  switch (port)
  {
  case SDIO_PORT_A:
    break;
  case SDIO_PORT_B:
    //todo add card detect
    setbits_le32(P_PREG_PAD_GPIO5_EN_N, 1 << 29); //CARD_6
    break;
  case SDIO_PORT_C:
    //enable pull up
    clrbits_le32(P_PAD_PULL_UP_REG3, 0xff << 0);
    break;
  case SDIO_PORT_XC_A:
    break;
  case SDIO_PORT_XC_B:
    break;
  case SDIO_PORT_XC_C:
    break;
  default:
    break;
  }

  return cpu_sdio_init(port);
}
static int sd_inand_check_insert(struct mmc *mmc)
{
  DEBUG((EFI_D_INFO,"ENTER Function: sd_inand_check_insert\n"));
  int level;
  struct aml_card_sd_info *sd_inand_info = mmc->priv;

  level = sdio_detect(sd_inand_info->sdio_port);

  if (level)
  {

    if (sd_inand_info->init_retry)
    {
      //no need to power off
      //sdio_pwr_off(sd_inand_info->sdio_port);

      sd_inand_info->init_retry = 0;
    }
    if (sd_inand_info->inited_flag)
    {

      //sdio_pwr_off(sd_inand_info->sdio_port);

      sd_inand_info->removed_flag = 1;
      sd_inand_info->inited_flag = 0;
    }
    return 0; //No card is inserted
  }
  else
  {
    return 1; //A	card is	inserted
  }
}

unsigned long clk_util_clk_msr(unsigned long clk_mux)
{
  DEBUG((EFI_D_INFO,"ENTER Function: clk_util_clk_msr\n"));
  MmioWrite32(P_MSR_CLK_REG0, 0);
  // Set the measurement gate to 64uS
  clrsetbits_le32(P_MSR_CLK_REG0, 0xffff, 64 - 1);
  // Disable continuous measurement
  // disable interrupts
  clrbits_le32(P_MSR_CLK_REG0, ((1 << 18) | (1 << 17)));
  clrsetbits_le32(P_MSR_CLK_REG0, (0xf << 20), (clk_mux << 20) | (1 << 19) | (1 << 16));

  MmioRead32(P_MSR_CLK_REG0);
  // Wait for the measurement to be done
  while (MmioRead32(P_MSR_CLK_REG0) & (1 << 31))
  {
  };
  // disable measuring
  clrbits_le32(P_MSR_CLK_REG0, (1 << 16));
  UINT32 msr = (MmioRead32(P_MSR_CLK_REG2) + 31) & 0x000FFFFF;
  // Return value in MHz*measured_val
  return (msr >> 6);
}

int clk_get_rate(unsigned clksrc)
{
  DEBUG((EFI_D_INFO,"ENTER Function: clk_get_rate\n"));
  static UINT32 clk81_freq = 0xffffffff;
  if (clk81_freq == 0xffffffff)
  {
    clk81_freq = (clk_util_clk_msr(7) * 1000000);
  }
  return clk81_freq;
}


void aml_sd_cfg_swth(struct mmc *mmc)
{
  //DECLARE_GLOBAL_DATA_PTR;
  DEBUG((EFI_D_INFO,"ENTER Function: aml_sd_cfg_swth\n"));
  struct aml_card_sd_info *aml_priv = mmc->priv;

  unsigned long sdio_config = 0;

  unsigned bus_width = (mmc->bus_width == 4) ? 1 : 0;
  if (mmc->clock < mmc->f_min)
    mmc->clock = mmc->f_min;
  if (mmc->clock > mmc->f_max)
    mmc->clock = mmc->f_max;

  int clk = clk_get_rate(SDIO_CLKSRC);
  unsigned clk_div = (clk / (2 * mmc->clock));

  MmioWrite32(PREG_SDIO_IRQ_CFG, 1 << soft_reset_bit);
  MicroSecondDelay(3);
  //printf("test get_clk81:%d,sdio clk_div:0x%x,clock:%d\n",clk,clk_div,mmc->clock);

  sdio_config = ((2 << sdio_write_CRC_ok_status_bit) |
                 (2 << sdio_write_Nwr_bit) |
                 (3 << m_endian_bit) |
                 (bus_width << bus_width_bit) |
                 (39 << cmd_argument_bits_bit) |
                 (0 << cmd_out_at_posedge_bit) |
                 (0 << cmd_disable_CRC_bit) |
                 (0 << response_latch_at_negedge_bit) |
                 (clk_div << cmd_clk_divide_bit));

  DEBUG((EFI_D_INFO, "sdio_config=%x", sdio_config));
  MmioWrite32(PREG_SDIO_CFG, sdio_config);
  MmioWrite32(PREG_SDIO_MULT_CFG, aml_priv->sdio_port & 0x3); //Switch to	SDIO_A/B/C

  DEBUG((EFI_D_INFO, "bus_width=%d\tclk_div=%d\n\tclk=%d\tsd_clk=%d",
         bus_width, clk_div, clk, mmc->clock));

  DEBUG((EFI_D_INFO, "port=%d act_clk=%d", aml_priv->sdio_port, clk / (2 * (clk_div + 1))));
  return;
}

static void sdio_pwr_prepare(unsigned port)
{
    /// @todo NOT FINISH
	///do nothing here
	switch(port)
    {
        case SDIO_PORT_A:
            clrbits_le32(P_PREG_PAD_GPIO4_EN_N,0x30f);
            clrbits_le32(P_PREG_PAD_GPIO4_O   ,0x30f);
            clrbits_le32(P_PERIPHS_PIN_MUX_8,0x3f);break;
        case SDIO_PORT_B:
            clrbits_le32(P_PREG_PAD_GPIO5_EN_N,0x3f<<23);
            clrbits_le32(P_PREG_PAD_GPIO5_O   ,0x3f<<23);
            clrbits_le32(P_PERIPHS_PIN_MUX_2,0x3f<<10);break;
        case SDIO_PORT_C:
            //clrbits_le32(P_PREG_PAD_GPIO3_EN_N,0xc0f);
            //clrbits_le32(P_PREG_PAD_GPIO3_O   ,0xc0f);
            //clrbits_le32(P_PERIPHS_PIN_MUX_6,(0x3f<<24));break;
        case SDIO_PORT_XC_A:
            break;
        case SDIO_PORT_XC_B:
            break;
        case SDIO_PORT_XC_C:
            //clrbits_le32(P_PREG_PAD_GPIO3_EN_N,0xcff);
            //clrbits_le32(P_PREG_PAD_GPIO3_O   ,0xcff);
            //clrbits_le32(P_PERIPHS_PIN_MUX_4,(0xf<<26));
            //printf("inand sdio xc-c prepare\n");
            break;
    }
}
static void sdio_pwr_off(unsigned port)
{
    /// @todo NOT FINISH
    switch(port)
    {
        case SDIO_PORT_A:
            break;
        case SDIO_PORT_B:
            setbits_le32(P_PREG_PAD_GPIO5_O,(1<<31)); //CARD_8
            clrbits_le32(P_PREG_PAD_GPIO5_EN_N,(1<<31));
            break;
        case SDIO_PORT_C:
            break;
        case SDIO_PORT_XC_A:
            break;
        case SDIO_PORT_XC_B:
            break;
        case SDIO_PORT_XC_C:
            break;
        default:
            break;
    }
    return;
}
static void sdio_pwr_on(unsigned port)
{
    switch(port)
    {
        case SDIO_PORT_A:
            break;
        case SDIO_PORT_B:
            clrbits_le32(P_PREG_PAD_GPIO5_O,(1<<31)); //CARD_8
            clrbits_le32(P_PREG_PAD_GPIO5_EN_N,(1<<31));
			/// @todo NOT FINISH
            break;
        case SDIO_PORT_C:    	
            break;
        case SDIO_PORT_XC_A:
            break;
        case SDIO_PORT_XC_B:
            break;
        case SDIO_PORT_XC_C:
            break;
        default:
            break;
    }
    return;
}

int sd_inand_staff_init(struct mmc *mmc)
{
  DEBUG((EFI_D_INFO,"ENTER Function: sd_inand_staff_init\n"));
  struct aml_card_sd_info *sdio = mmc->priv;

  sdio_pwr_prepare(sdio->sdio_port);
  sdio_pwr_off(sdio->sdio_port);

	if(sdio->sdio_pwr_flag&CARD_SD_SDIO_PWR_OFF)
	{	
	    sdio->sdio_pwr_flag &=~CARD_SD_SDIO_PWR_OFF;

		MicroSecondDelay(200);
    }
    DEBUG((EFI_D_INFO,"pre power on"));
    sdio_pwr_on(sdio->sdio_port);
    sdio_init(sdio->sdio_port);
    DEBUG((EFI_D_INFO,"post power on"));
        
    if(sdio->sdio_pwr_flag&CARD_SD_SDIO_PWR_ON)
    {    	
        sdio->sdio_pwr_flag &=~CARD_SD_SDIO_PWR_ON;

		MicroSecondDelay(200);
    }

  aml_sd_cfg_swth(mmc);
  if (!sdio->inited_flag)
    sdio->inited_flag = 1;
  //    int ret=rom_c();
  //    sd_debug("rom_c==%d",rom_c());
  //    return ret;
  return SD_NO_ERROR;
}

int aml_sd_init(struct mmc *mmc)
{
  DEBUG((EFI_D_INFO,"ENTER Function: aml_sd_init\n"));
  //int ret;
  struct aml_card_sd_info *sdio = mmc->priv;
  //setting io pin mux

  if (sdio->inited_flag)
  {

    sdio_init(sdio->sdio_port);

    if ((sdio->sdio_port == SDIO_PORT_B) && (sdio_debug_1bit_flag))
    {
      clrbits_le32(P_PERIPHS_PIN_MUX_2, 7 << 12);
    }

    aml_sd_cfg_swth(mmc);
    return 0;
  }

  if (sd_inand_check_insert(mmc))
  {
    sd_inand_staff_init(mmc);
    return 0;
  }
  else
    return 1;
}

EFI_BLOCK_IO_PROTOCOL BlockIo =
    {
        EFI_BLOCK_IO_INTERFACE_REVISION, // Revision
        &gNandFlashMedia,                // *Media
        NandFlashReset,                  // Reset
        NandFlashReadBlocks,             // ReadBlocks
        NandFlashWriteBlocks,            // WriteBlocks
        NandFlashFlushBlocks             // FlushBlocks
};
int mmc_send_op_cond(struct mmc *mmc)
{
  DEBUG((EFI_D_INFO,"ENTER Function: mmc_send_op_cond\n"));
  int timeout = 1000;
  struct mmc_cmd cmd;
  int err;
  UINT32 *response = (UINT32 *)(&((cmd.response)[0]));

  /* Some cards seem to need this */
  NandFlashReset(&BlockIo, FALSE);

  do
  {
    cmd.cmdidx = MMC_CMD_SEND_OP_COND;
    cmd.resp_type = MMC_RSP_R3;
    cmd.cmdarg = OCR_HCS | mmc->voltages;
    cmd.flags = 0;

    err = aml_sd_send_cmd(mmc, &cmd, NULL);

    if (err)
      return err;

    MicroSecondDelay(1);
  } while (!((*response) & OCR_BUSY) && timeout--); //while (!(((uint *)(cmd.response))[0] & OCR_BUSY) && timeout--);

  if (timeout <= 0)
    return UNUSABLE_ERR;

  mmc->version = MMC_VERSION_UNKNOWN;
  //mmc->ocr = ((uint *)(cmd.response))[0];
  mmc->ocr = *response;

  mmc->high_capacity = ((mmc->ocr & OCR_HCS) == OCR_HCS);
  mmc->rca = 1;

  return 0;
}

void mmc_set_bus_width(struct mmc *mmc, UINT32 width)
{
  
  DEBUG((EFI_D_INFO,"ENTER Function: mmc_set_bus_width\n"));
  mmc->bus_width = width;

  aml_sd_cfg_swth(mmc);
}

int mmc_switch(struct mmc *mmc, UINT8 set, UINT8 index, UINT8 value)
{
  DEBUG((EFI_D_INFO,"ENTER Function: mmc_switch\n"));
  struct mmc_cmd cmd;

  cmd.cmdidx = MMC_CMD_SWITCH;
  cmd.resp_type = MMC_RSP_R1b;
  cmd.cmdarg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) |
               (index << 16) |
               (value << 8);
  cmd.flags = 0;

  return aml_sd_send_cmd(mmc, &cmd, NULL);
}
void mmc_set_clock(struct mmc *mmc, UINT32 clock)
{
  DEBUG((EFI_D_INFO,"ENTER Function: mmc_set_clock\n"));
  if (clock > mmc->f_max)
    clock = mmc->f_max;

  if (clock < mmc->f_min)
    clock = mmc->f_min;

  mmc->clock = clock;

  aml_sd_cfg_swth(mmc);
}
int mmc_change_freq(struct mmc *mmc)
{
  DEBUG((EFI_D_INFO,"ENTER Function: mmc_change_freq\n"));
  char ext_csd[512];
  char cardtype;
  UINT64 sector_count;
  int err;
  mmc->card_caps = 0;

  /* Only version 4 supports high-speed */
  if (mmc->version < MMC_VERSION_4)
    return 0;

  mmc->card_caps |= MMC_MODE_4BIT;

  err = mmc_send_ext_csd(mmc, ext_csd);
  MicroSecondDelay(1);
  if (err)
    return err;
  //printf("ERASED_MEM_CONT is %d\n",ext_csd[181]);

  if (ext_csd[212] || ext_csd[213] || ext_csd[214] || ext_csd[215])
  {
    //mmc->capacity = (*(unsigned *)(&ext_csd[212]))*mmc->read_bl_len;
    sector_count = (((unsigned)ext_csd[215]) << 24) | (((unsigned)ext_csd[214]) << 16) | (((unsigned)ext_csd[213]) << 8) | (((unsigned)ext_csd[212]) << 0);
    mmc->capacity = sector_count * mmc->read_bl_len;
    /*
		 * There are two boot regions of equal size, defined in
		 * multiples of 128K.
		 */
    mmc->boot_size = ext_csd[EXT_CSD_BOOT_MULT] << 17;
    //mmc->high_capacity = 1;
  }

  cardtype = ext_csd[196] & 0xf;

  err = mmc_switch(mmc, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_HS_TIMING, 1);
  MicroSecondDelay(1);
  if (err)
    return err;

  /* Now check to see that it worked */
  err = mmc_send_ext_csd(mmc, ext_csd);

  if (err)
    return err;

  /* No high-speed support */
  if (!ext_csd[185])
    return 0;

  /* High Speed is set, there are two types: 52MHz and 26MHz */
  if (cardtype & MMC_HS_52MHZ)
    mmc->card_caps |= MMC_MODE_HS_52MHz | MMC_MODE_HS;
  else
    mmc->card_caps |= MMC_MODE_HS;

  return 0;
}
int mmc_startup(struct mmc *mmc)
{
  DEBUG((EFI_D_INFO,"ENTER Function: mmc_startup\n"));
  int err;
  UINT32 mult, freq;
  UINT64 cmult, csize;
  struct mmc_cmd cmd;
  //char ext_csd[512];
  UINT32 *cid = (UINT32 *)(&(mmc->cid[0]));
  UINT32 *response = (UINT32 *)(&(cmd.response[0]));

  /* Put the Card in Identify Mode */
  cmd.cmdidx = MMC_CMD_ALL_SEND_CID;
  cmd.resp_type = MMC_RSP_R2;
  cmd.cmdarg = 0;
  cmd.flags = 0;

  err = aml_sd_send_cmd(mmc, &cmd, NULL);

  if (err)
    return err;

  CopyMem(mmc->cid, cmd.response, 16);

  cid[0] = be32dec(cid[0]);
  cid[1] = be32dec(cid[1]);
  cid[2] = be32dec(cid[2]);
  cid[3] = be32dec(cid[3]);

  /*
	 * For MMC cards, set the Relative Address.
	 * For SD cards, get the Relatvie Address.
	 * This also puts the cards into Standby State
	 */
  cmd.cmdidx = SD_CMD_SEND_RELATIVE_ADDR;
  cmd.cmdarg = mmc->rca << 16;
  cmd.resp_type = MMC_RSP_R6;
  cmd.flags = 0;

  err = aml_sd_send_cmd(mmc, &cmd, NULL);

  if (err)
    return err;

  //mmc->rca = (((UINT *)(cmd.response))[0] >> 16) & 0xffff;

  /* Get the Card-Specific Data */
  cmd.cmdidx = MMC_CMD_SEND_CSD;
  cmd.resp_type = MMC_RSP_R2;
  cmd.cmdarg = mmc->rca << 16;
  cmd.flags = 0;

  err = aml_sd_send_cmd(mmc, &cmd, NULL);

  if (err)
    return err;

  mmc->csd[0] = response[0]; //((UINT *)(cmd.response))[0];
  mmc->csd[1] = response[1]; //((UINT *)(cmd.response))[1];
  mmc->csd[2] = response[2]; //((UINT *)(cmd.response))[2];
  mmc->csd[3] = response[3]; //((UINT *)(cmd.response))[3];

  if (mmc->version == MMC_VERSION_UNKNOWN)
  {
    //int version = (((UINT *)(cmd.response))[0]  >> 26) & 0xf;
    int version = (response[0] >> 26) & 0xf;

    switch (version)
    {
    case 0:
      mmc->version = MMC_VERSION_1_2;
      break;
    case 1:
      mmc->version = MMC_VERSION_1_4;
      break;
    case 2:
      mmc->version = MMC_VERSION_2_2;
      break;
    case 3:
      mmc->version = MMC_VERSION_3;
      break;
    case 4:
      mmc->version = MMC_VERSION_4;
      break;
    default:
      mmc->version = MMC_VERSION_1_2;
      break;
    }
  }

  /* divide frequency by 10, since the mults are 10x bigger */
  //freq = fbase[(((UINT *)(cmd.response))[0]  & 0x7)];
  freq = fbase[(response[0] & 0x7)];
  //mult = multipliers[((((UINT *)(cmd.response))[0] >> 3) & 0xf)];
  mult = multipliers[((response[0] >> 3) & 0xf)];

  mmc->tran_speed = freq * mult;

  //mmc->read_bl_len = 1 << ((((UINT *)(cmd.response))[1] >> 16) & 0xf);
  mmc->read_bl_len = 1 << ((response[1] >> 16) & 0xf);

  mmc->write_bl_len = 1 << ((response[3] >> 22) & 0xf);

  if (mmc->high_capacity)
  {
    csize = (mmc->csd[1] & 0x3f) << 16 | (mmc->csd[2] & 0xffff0000) >> 16;
    cmult = 8;
  }
  else
  {
    csize = (mmc->csd[1] & 0x3ff) << 2 | (mmc->csd[2] & 0xc0000000) >> 30;
    cmult = (mmc->csd[2] & 0x00038000) >> 15;
  }

  mmc->capacity = (csize + 1) << (cmult + 2);
  mmc->capacity *= mmc->read_bl_len;
  mmc->boot_size = 0;
  if (mmc->read_bl_len > 512)
    mmc->read_bl_len = 512;

  if (mmc->write_bl_len > 512)
    mmc->write_bl_len = 512;

  /* Select the card, and put it into Transfer Mode */
  cmd.cmdidx = MMC_CMD_SELECT_CARD;
  cmd.resp_type = MMC_RSP_R1b;
  cmd.cmdarg = mmc->rca << 16;
  cmd.flags = 0;
  err = aml_sd_send_cmd(mmc, &cmd, NULL);

  if (err)
    return err;

  //	if (!IS_SD(mmc) && (mmc->version >= MMC_VERSION_4)) {
  /* check  ext_csd version and capacity */
  /*		err = mmc_send_ext_csd(mmc, ext_csd);
		if (!err & (ext_csd[192] >= 2)) {
			mmc->capacity = ext_csd[212] << 0 | ext_csd[213] << 8 |
					ext_csd[214] << 16 | ext_csd[215] << 24;
			mmc->capacity *= 512;
		}
	}*/

  // if (IS_SD(mmc))
  // 	err = sd_change_freq(mmc);
  // else
  err = mmc_change_freq(mmc);

  if (err)
    return err;

  /* Restrict card's capabilities by what the host can do */
  mmc->card_caps &= mmc->host_caps;

  if (mmc->card_caps & MMC_MODE_4BIT)
  {
    /* Set the card to use 4 bit*/
    err = mmc_switch(mmc, EXT_CSD_CMD_SET_NORMAL,
                     EXT_CSD_BUS_WIDTH,
                     EXT_CSD_BUS_WIDTH_4);

    if (err)
      return err;

    mmc_set_bus_width(mmc, 4);
  }
  else if (mmc->card_caps & MMC_MODE_8BIT)
  {
    /* Set the card to use 8 bit*/
    err = mmc_switch(mmc, EXT_CSD_CMD_SET_NORMAL,
                     EXT_CSD_BUS_WIDTH,
                     EXT_CSD_BUS_WIDTH_8);

    if (err)
      return err;

    mmc_set_bus_width(mmc, 8);
  }

  if (mmc->card_caps & MMC_MODE_HS)
  {
    if (mmc->card_caps & MMC_MODE_HS_52MHz)
      mmc_set_clock(mmc, 52000000);
    else
    {
#ifdef CONFIG_STORE_COMPATIBLE
      if (aml_is_emmc_tsd(mmc))
      {
        mmc_set_clock(mmc, 40000000);
      }
      else
      {
        mmc_set_clock(mmc, 26000000);
      }
#else
      mmc_set_clock(mmc, 26000000);
#endif
    }
  }
  else
    mmc_set_clock(mmc, 20000000);

  /* fill in device description */
  mmc->block_dev.lun = 0;
  mmc->block_dev.type = 0;
  mmc->block_dev.blksz = mmc->read_bl_len;
  mmc->block_dev.lba = mmc->capacity / mmc->read_bl_len;
  // sprintf(mmc->block_dev.vendor,"Man %02x%02x%02x Snr %02x%02x%02x%02x",
  // 		mmc->cid[0], mmc->cid[1], mmc->cid[2],
  // 		mmc->cid[9], mmc->cid[10], mmc->cid[11], mmc->cid[12]);
  // sprintf(mmc->block_dev.product,"%c%c%c%c%c", mmc->cid[3],
  // 		mmc->cid[4], mmc->cid[5], mmc->cid[6], mmc->cid[7]);
  // sprintf(mmc->block_dev.revision,"%d.%d", mmc->cid[8] >> 4,
  // 		mmc->cid[8] & 0xf);

  DEBUG((EFI_D_INFO, "MMC init part\n"));
  //init_part(&mmc->block_dev);
  //print_part(&mmc->block_dev);
  return 0;
}
int sd_send_op_cond(struct mmc *mmc)
{
  DEBUG((EFI_D_INFO, "ENTER Function: sd_send_op_cond\n"));
  int timeout = 1000;
  int err;
  struct mmc_cmd cmd;
  UINT32 *response = (UINT32 *)(&((cmd.response)[0]));

  do
  {
    cmd.cmdidx = MMC_CMD_APP_CMD;
    cmd.resp_type = MMC_RSP_R1;
    cmd.cmdarg = 0;
    cmd.flags = 0;

    err = aml_sd_send_cmd(mmc, &cmd, NULL);

    if (err == TIMEOUT)
      return err;
    else if (err)
    {
      MicroSecondDelay(10);
      continue;
    }

    cmd.cmdidx = SD_CMD_APP_SEND_OP_COND;
    cmd.resp_type = MMC_RSP_R3;

    /*
		 * Most cards do not answer if some reserved bits
		 * in the ocr are set. However, Some controller
		 * can set bit 7 (reserved for low voltages), but
		 * how to manage low voltages SD card is not yet
		 * specified.
		 */
    cmd.cmdarg = mmc->voltages & 0xff8000;

    if (mmc->version == SD_VERSION_2)
      cmd.cmdarg |= OCR_HCS;

    err = aml_sd_send_cmd(mmc, &cmd, NULL);

    if (err)
      return err;

    MicroSecondDelay(1);
  } while ((!((*response) & OCR_BUSY) || err) && timeout--); //while ((!(((uint *)cmd.response)[0] & OCR_BUSY) || err) && timeout--);

  if (timeout <= 0)
    return UNUSABLE_ERR;

  if (mmc->version != SD_VERSION_2)
    mmc->version = SD_VERSION_1_0;

  //mmc->ocr = ((uint *)(cmd.response))[0];
  mmc->ocr = *response;

  mmc->high_capacity = ((mmc->ocr & OCR_HCS) == OCR_HCS);
  mmc->rca = 0;

  return 0;
}


int mmc_send_if_cond(struct mmc *mmc)
{
  DEBUG((EFI_D_INFO,"ENTER Function: mmc_send_if_cond\n"));
  struct mmc_cmd cmd;
  int err;
  UINT32 *response = (UINT32 *)(&(cmd.response[0]));

  cmd.cmdidx = SD_CMD_SEND_IF_COND;
  /* We set the bit if the host supports voltages between 2.7 and 3.6 V */
  cmd.cmdarg = ((mmc->voltages & 0xff8000) != 0) << 8 | 0xaa;
  cmd.resp_type = MMC_RSP_R7;
  cmd.flags = 0;

  err = aml_sd_send_cmd(mmc, &cmd, NULL);

  if (err)
    return err;

  //	if ((((UINT *)(cmd.response))[0] & 0xff) != 0xaa)
  if ((response[0] & 0xff) != 0xaa)
    return UNUSABLE_ERR;
  else
    mmc->version = SD_VERSION_2;

  return 0;
}
static void print_mmcinfo(struct mmc *mmc)
{
	DEBUG((DEBUG_INFO,"Device: %s\n", mmc->name));
	DEBUG((DEBUG_INFO,"Manufacturer ID: %x\n", mmc->cid[0] >> 24));
	DEBUG((DEBUG_INFO,"OEM: %x\n", (mmc->cid[0] >> 8) & 0xffff));
	DEBUG((DEBUG_INFO,"Name: %c%c%c%c%c \n", mmc->cid[0] & 0xff,
			(mmc->cid[1] >> 24), (mmc->cid[1] >> 16) & 0xff,
			(mmc->cid[1] >> 8) & 0xff, mmc->cid[1] & 0xff));

	DEBUG((DEBUG_INFO,"Tran Speed: %d\n", mmc->tran_speed));
	DEBUG((DEBUG_INFO,"Rd Block Len: %d\n", mmc->read_bl_len));

	DEBUG((DEBUG_INFO,"%s version %d.%d\n", IS_SD(mmc) ? "SD" : "MMC",
			(mmc->version >> 4) & 0xf, mmc->version & 0xf));

	DEBUG((DEBUG_INFO,"High Capacity: %s\n", mmc->high_capacity ? "Yes" : "No"));
	DEBUG((DEBUG_INFO,"Capacity: %lld\n", mmc->capacity));
	DEBUG((DEBUG_INFO,"Boot Part Size: %lld\n", mmc->boot_size));

	DEBUG((DEBUG_INFO,"Bus Width: %d-bit\n", mmc->bus_width));
}

int mmc_init(struct mmc *mmc)
{
  DEBUG((EFI_D_INFO,"ENTER Function: mmc_init\n"));
  int err;

  err = aml_sd_init(mmc);
  if (err)
    return EFI_DEVICE_ERROR;

  if (mmc->is_inited) // has been initialized
    return 0;

  /* Reset the Card */
  err = NandFlashReset(&BlockIo, TRUE);
  if (err)
    return EFI_DEVICE_ERROR;

  // check SDCARD/EMMC to reduce EMMC load env data cycles
  //tsd could not init as emmc (cmd1)
  //delete SD/mmc card line
  /* Test for SD version 2 */
  err = mmc_send_if_cond(mmc);
	/* If we got an error other than timeout, we bail */
  if (err && err != TIMEOUT)
  	return err;
  else if(err)
  	NandFlashReset(&BlockIo, TRUE);
  /* Now try to get the SD card's operating condition */
  err = sd_send_op_cond(mmc);
  
  /* If the command timed out, we check for an MMC card */
  if (err == TIMEOUT) {
  	err = mmc_send_op_cond(mmc);
  
  	if (err)
  	  {
  		DEBUG((EFI_D_INFO, "[%s] %s:%d, eMMC: Card did not respond to voltage select! "
  						   "mmc->block_dev.if_type=%d\n",
  			   __FUNCTION__, mmc->name, mmc->block_dev.dev, mmc->block_dev.if_type));
  		return UNUSABLE_ERR;
  	  }
  }
  

  err = mmc_startup(mmc);
  DEBUG((EFI_D_INFO, "[%s] %s:%d, if_type=%d, initialized %s!\n", __FUNCTION__,
         mmc->name, mmc->block_dev.dev, mmc->block_dev.if_type, (err == 0) ? "OK" : "ERROR"));
  if (err)
  {
    return err;
  }
  else
  {
    mmc->is_inited = 1; // init OK
  }
  return err;
}

static struct aml_card_sd_info m3_sdio_ports[] = {
    {.sdio_port = SDIO_PORT_A, .name = "SDIO Port A"},
    {.sdio_port = SDIO_PORT_B, .name = "SDIO Port B"},
    {.sdio_port = SDIO_PORT_C, .name = "SDIO Port C"},
    {.sdio_port = SDIO_PORT_XC_A, .name = "SDIO Port XC-A"},
    {.sdio_port = SDIO_PORT_XC_B, .name = "SDIO Port XC-B"},
    {.sdio_port = SDIO_PORT_XC_C, .name = "SDIO Port XC-C"},
};
struct aml_card_sd_info *cpu_sdio_get(unsigned port)
{
  DEBUG((EFI_D_INFO,"ENTER Function: cpu_sdio_get\n"));
  if (port < SDIO_PORT_C + 1)
    return &m3_sdio_ports[port];
  return NULL;
}
int mmc_register(struct mmc *mmc)
{
  DEBUG((EFI_D_INFO,"ENTER Function: mmc_register\n"));
  struct aml_card_sd_info *sdio = mmc->priv;

  /* Setup the universal parts of the block interface just once */
  if (mmc->block_dev.if_type != IF_TYPE_SD)
    mmc->block_dev.if_type = IF_TYPE_MMC;
  mmc->block_dev.dev = cur_dev_num++;
  mmc->block_dev.removable = 1;

  DEBUG((EFI_D_INFO, "[%s] add mmc dev_num=%d, port=%d, if_type=%d\n",
         __FUNCTION__, mmc->block_dev.dev, sdio->sdio_port, mmc->block_dev.if_type));

  return 0;
}
int aml_card_type = 0;

void sdio_register(struct mmc *mmc, struct aml_card_sd_info *aml_priv)
{
  DEBUG((EFI_D_INFO,"ENTER Function: sdio_register\n"));
  int card_type;

  AsciiStrCpyS(mmc->name, 31,aml_priv->name);
  mmc->priv = aml_priv;
  aml_priv->removed_flag = 1;
  aml_priv->inited_flag = 0;
  /*	
	aml_priv->sdio_init=sdio_init;
	aml_priv->sdio_detect=sdio_detect;
	aml_priv->sdio_pwr_off=sdio_pwr_off;
	aml_priv->sdio_pwr_on=sdio_pwr_on;
	aml_priv->sdio_pwr_prepare=sdio_pwr_prepare;
*/
  mmc->rca = 1;
  mmc->voltages = MMC_VDD_33_34;
  mmc->host_caps = MMC_MODE_4BIT | MMC_MODE_HS;
  //mmc->host_caps = MMC_MODE_4BIT;
  mmc->bus_width = 1;
  mmc->clock = 300000;
  mmc->f_min = 200000;
  mmc->f_max = 50000000;
  mmc->is_inited = 0;
  mmc_register(mmc);

  //WRITE_CBUS_REG(RESET6_REGISTER, (1<<8));
  WRITE_CBUS_REG(SDIO_AHB_CBUS_CTRL, 0);

  card_type = AML_GET_CARD_TYPE(aml_card_type, aml_priv->sdio_port);
  if (card_type == CARD_TYPE_MMC)
    mmc->block_dev.if_type = IF_TYPE_MMC;
  else
    mmc->block_dev.if_type = IF_TYPE_SD;
  // printf("\033[0;40;32m [%s] port=%d, aml_card_type=%#x, card_type=%d, mmc->block_dev.if_type=%d \033[0m\n",
  // __FUNCTION__, aml_priv->sdio_port, aml_card_type, card_type, mmc->block_dev.if_type);
}
void board_mmc_register(unsigned port)
{
  DEBUG((EFI_D_INFO,"ENTER Function: board_mmc_register\n"));
  struct aml_card_sd_info *aml_priv = cpu_sdio_get(port);

  mmc = (struct mmc *)AllocateZeroPool(sizeof(struct mmc));
  if (aml_priv == NULL || mmc == NULL)
    return;

  sdio_register(mmc, aml_priv);
  mmc_init(mmc);
  print_mmcinfo(mmc);
  DEBUG((EFI_D_INFO,"LEAVE Function: board_mmc_register\n"));
}

EFI_STATUS
NandFlashInitialize(
    IN EFI_HANDLE ImageHandle,
    IN EFI_SYSTEM_TABLE *SystemTable)
{
  DEBUG((EFI_D_INFO,"ENTER Function: NandFlashInitialize\n"));
  EFI_STATUS  Status;


  board_mmc_register(SDIO_PORT_B);

  //Publish BlockIO.
  Status = gBS->InstallMultipleProtocolInterfaces (
                  &ImageHandle,
                  &gEfiBlockIoProtocolGuid, &BlockIo,
                  &gEfiDevicePathProtocolGuid, &gDevicePath,
                  NULL
                  );
  return Status;
}

