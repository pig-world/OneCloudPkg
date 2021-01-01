/** @file
 *
 *  Copyright (c) 2017 - 2018, Andrei Warkentin <andrey.warkentin@gmail.com>
 *  Copyright (c), Microsoft Corporation. All rights reserved.
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

#include "DisplayDxe.h"

#define DEVICE_NAME "amhdmitx"
#define HDMI_TX_COUNT 32
#define HDMI_TX_POOL_NUM  6
#define HDMI_TX_RESOURCE_NUM 4
#define HDMI_TX_PWR_CTRL_NUM    6

#define MODE_800_ENABLED    BIT0
#define MODE_640_ENABLED    BIT1
#define MODE_1024_ENABLED   BIT2
#define MODE_720P_ENABLED   BIT3
#define MODE_1080P_ENABLED  BIT4
#define MODE_NATIVE_ENABLED BIT5
#define JUST_NATIVE_ENABLED MODE_NATIVE_ENABLED
#define FB_VISUAL_PSEUDOCOLOR		3	/* Pseudo color (like atari) */
#define FB_VISUAL_TRUECOLOR		2	/* True color	*/

static unsigned int tvmode_vmode_vic_map[][3] = {
    {TVOUT_480I, VMODE_480I, HDMI_480i60_16x9},
    {TVOUT_480I, VMODE_480I, HDMI_480i60},
    {TVOUT_480CVBS, VMODE_480I, HDMI_480i60},
    {TVOUT_480P, VMODE_480P, HDMI_480p60_16x9},
    {TVOUT_480P, VMODE_480P, HDMI_480p60},
    {TVOUT_576I, VMODE_576I, HDMI_576i50_16x9},
    {TVOUT_576I, VMODE_576I, HDMI_576i50},
    {TVOUT_576CVBS, VMODE_576I, HDMI_576i50},
    {TVOUT_576P, VMODE_576P, HDMI_576p50_16x9},
    {TVOUT_576P, VMODE_576P, HDMI_576p50},
    {TVOUT_720P, VMODE_720P, HDMI_720p60},
    {TVOUT_1080I, VMODE_1080I, HDMI_1080i60},
    {TVOUT_1080P, VMODE_1080P, HDMI_1080p60},
    {TVOUT_720P_50HZ, VMODE_720P_50HZ, HDMI_720p50},
    {TVOUT_1080I_50HZ, VMODE_1080I_50HZ, HDMI_1080i50},
    {TVOUT_1080P_50HZ, VMODE_1080P_50HZ, HDMI_1080p50},
    {TVOUT_1080P_24HZ, VMODE_1080P_24HZ, HDMI_1080p24},
    {TVOUT_4K2K_30HZ, VMODE_4K2K_30HZ, HDMI_4k2k_30},
    {TVOUT_4K2K_25HZ, VMODE_4K2K_25HZ, HDMI_4k2k_25},
    {TVOUT_4K2K_24HZ, VMODE_4K2K_24HZ, HDMI_4k2k_24},
    {TVOUT_4K2K_SMPTE, VMODE_4K2K_SMPTE, HDMI_4k2k_smpte},
    {TVOUT_MAX, VMODE_MAX, HDMI_Unkown},
};

#define VNBYTES(bpix) (1 << (bpix)) / 8

enum video_log2_bpp {
  VIDEO_BPP1 = 0,
  VIDEO_BPP2,
  VIDEO_BPP4,
  VIDEO_BPP8,
  VIDEO_BPP16,
  VIDEO_BPP32,
};

STATIC
EFI_STATUS
EFIAPI
DriverSupported (
  IN EFI_DRIVER_BINDING_PROTOCOL *This,
  IN EFI_HANDLE                  Controller,
  IN EFI_DEVICE_PATH_PROTOCOL    *RemainingDevicePath
  );

STATIC
EFI_STATUS
EFIAPI
DriverStart (
  IN EFI_DRIVER_BINDING_PROTOCOL *This,
  IN EFI_HANDLE                  Controller,
  IN EFI_DEVICE_PATH_PROTOCOL    *RemainingDevicePath
  );

STATIC
EFI_STATUS
EFIAPI
DriverStop (
  IN EFI_DRIVER_BINDING_PROTOCOL *This,
  IN EFI_HANDLE                  Controller,
  IN UINTN                       NumberOfChildren,
  IN EFI_HANDLE                  *ChildHandleBuffer
  );

STATIC
EFI_STATUS
EFIAPI
DisplayQueryMode(
                 IN  EFI_GRAPHICS_OUTPUT_PROTOCOL          *This,
                 IN  UINT32                                ModeNumber,
                 OUT UINTN                                 *SizeOfInfo,
                 OUT EFI_GRAPHICS_OUTPUT_MODE_INFORMATION  **Info
                 );

STATIC
EFI_STATUS
EFIAPI
DisplaySetMode(
               IN  EFI_GRAPHICS_OUTPUT_PROTOCOL *This,
               IN  UINT32                       ModeNumber
               );

STATIC
EFI_STATUS
EFIAPI
DisplayBlt(
           IN  EFI_GRAPHICS_OUTPUT_PROTOCOL            *This,
           IN  EFI_GRAPHICS_OUTPUT_BLT_PIXEL           *BltBuffer,   OPTIONAL
           IN  EFI_GRAPHICS_OUTPUT_BLT_OPERATION       BltOperation,
           IN  UINTN                                   SourceX,
           IN  UINTN                                   SourceY,
           IN  UINTN                                   DestinationX,
           IN  UINTN                                   DestinationY,
           IN  UINTN                                   Width,
           IN  UINTN                                   Height,
           IN  UINTN                                   Delta         OPTIONAL
           );

STATIC EFI_GRAPHICS_OUTPUT_PROTOCOL mDisplay = {
    DisplayQueryMode, DisplaySetMode, DisplayBlt, NULL};
	
STATIC FRAME_BUFFER_CONFIGURE *mFrameBufferBltLibConfigure;
STATIC UINTN mFrameBufferBltLibConfigureSize;
	
STATIC EFI_DRIVER_BINDING_PROTOCOL mDriverBinding = {
  DriverSupported,
  DriverStart,
  DriverStop,
  0xa,
  NULL,
  NULL
};


//Hdmi_tx start
#define HDMI_SOURCE_DESCRIPTION 0
#define HDMI_PACKET_VEND        1
#define HDMI_MPEG_SOURCE_INFO   2
#define HDMI_PACKET_AVI         3
#define HDMI_AUDIO_INFO         4
#define HDMI_AUDIO_CONTENT_PROTECTION   5
#define HDMI_PACKET_HBR         6

#define HSYNC_POLARITY      1                       // HSYNC polarity: active high 
#define VSYNC_POLARITY      1                       // VSYNC polarity: active high


typedef struct {
	unsigned int h_res;
	unsigned int v_res;
	unsigned int refresh_rate;
	unsigned int clk_level;
}VPU_Conf_t;


//************************************************
// VPU is not supposed to run at 364MHz.  It was designed to max out around 225MHz in BABY.
// Please lower it to 364/2 = 182MHz.  This is super urgent.
//************************************************
#define CLK_LEVEL_DFT		3
#define CLK_LEVEL_MAX		5	//limit max clk to 212M
static unsigned int vpu_clk_setting[][3] = {
	//frequency		clk_mux		div
	{106250000,		1,			7},	//0
	{127500000,		2,			3},	//1
	{159375000,		0,			3},	//2
	{182150000,		3,			1},	//3
	{212500000,		1,			3},	//4
	{255000000,		2,			1},	//5
	{318750000,		0,			1},	//6
	{364300000,		3,			0},	//7
	{425000000,		1,			1},	//8
	{510000000,		2,			0},	//9
	{637500000,		0,			0},	//10
	//{850000000,		1,			0},	//11
};

static VPU_Conf_t vpu_config = {
	.h_res = 2048,
	.v_res = 1536,
	.refresh_rate = 60,	//Hz
	.clk_level = CLK_LEVEL_DFT,
};

static unsigned int get_vpu_clk_level(unsigned int video_clk)
{
	unsigned int video_bw;
	unsigned clk_level;
	int i;
	
	video_bw = video_clk + 2000000;

	for (i=0; i<CLK_LEVEL_MAX; i++) {
		if (video_bw <= vpu_clk_setting[i][0])			
			break;
	}
	clk_level = i;

	return clk_level;
}

static int set_vpu_clk(unsigned int vclk)
{
	int ret = 0;
	unsigned clk_level;
	
	if (vclk >= 100) {	//regard as vpu_clk
		clk_level = get_vpu_clk_level(vclk);
	}
	else {	//regard as clk_level
		clk_level = vclk;
	}

	if (clk_level >= CLK_LEVEL_MAX) {
		ret = 1;
		clk_level = CLK_LEVEL_DFT;
		//printf("vpu clk out of supported range, set to default\n");
	}
	
	MmioWrite32(P_HHI_VPU_CLK_CNTL,((1 << 8) | (vpu_clk_setting[clk_level][1] << 9) | (vpu_clk_setting[clk_level][2] << 0)));
	vpu_config.clk_level = clk_level;
	//printf("set vpu clk: %uHz, readback: %uHz(0x%x)\n", vpu_clk_setting[clk_level][0], get_vpu_clk(), (readl(P_HHI_VPU_CLK_CNTL)));

	return ret;
}

static void vpu_driver_init(void)
{
    set_vpu_clk(vpu_config.clk_level);

    clrbits_le32(P_AO_RTI_GEN_PWR_SLEEP0, (0x1<<8)); // [8] power on
    MmioWrite32(P_HHI_VPU_MEM_PD_REG0,0x00000000);
    MmioWrite32(P_HHI_VPU_MEM_PD_REG1,0x00000000);
    clrbits_le32(P_HHI_MEM_PD_REG0, (0xff << 8)); // MEM-PD

    MmioWrite32( P_VPU_MEM_PD_REG0,0x00000000);
    MmioWrite32(P_VPU_MEM_PD_REG1,0x00000000);
    
    // Powerup VPU_HDMI
    clrbits_le32(P_RESET0_MASK, ((0x1 << 5) | (0x1<<10)));
    clrbits_le32(P_RESET4_MASK, ((0x1 << 6) | (0x1<<7) | (0x1<<9) | (0x1<<13)));
    clrbits_le32(P_RESET2_MASK, ((0x1 << 2) | (0x1<<3) | (0x1<<11) | (0x1<<15)));
    MmioWrite32(P_RESET2_REGISTER,((0x1 << 2) | (0x1<<3) | (0x1<<11) | (0x1<<15)));
    MmioWrite32(P_RESET4_REGISTER,((0x1 << 6) | (0x1<<7) | (0x1<<9) | (0x1<<13)));    // reset this will cause VBUS reg to 0
    MmioWrite32(P_RESET0_REGISTER,((0x1 << 5) | (0x1<<10)));
    MmioWrite32(P_RESET4_REGISTER,((0x1 << 6) | (0x1<<7) | (0x1<<9) | (0x1<<13)));
    MmioWrite32(P_RESET2_REGISTER,((0x1 << 2) | (0x1<<3) | (0x1<<11) | (0x1<<15)));
    setbits_le32(P_RESET0_MASK, ((0x1 << 5) | (0x1<<10)));
    setbits_le32(P_RESET4_MASK, ((0x1 << 6) | (0x1<<7) | (0x1<<9) | (0x1<<13)));
    setbits_le32(P_RESET2_MASK, ((0x1 << 2) | (0x1<<3) | (0x1<<11) | (0x1<<15)));

    clrbits_le32(P_AO_RTI_GEN_PWR_SLEEP0, (0x1<<9)); // [9] VPU_HDMI
}

static void hdmi_wr_reg(unsigned int addr, unsigned int data);
static unsigned int hdmi_rd_reg(unsigned int addr);
static unsigned int modulo(unsigned int a, unsigned int b);
static signed int to_signed(unsigned int a);


/*
 * HDMI reg read/write operation
 */
/**************************************  HDMI reg read/write operation start **************************************/


void set_reg32_bits_op(UINT32 _reg, const UINT32 _val, const UINT32 _start, const UINT32 _len)
{
    unsigned int tmp;
    tmp = (MmioRead32(_reg) & ~(((1L<<(_len))-1)<<(_start))) | ((unsigned int)(_val) << (_start));
    MmioWrite32(_reg, tmp);
}

static void hdmi_wr_reg(unsigned int addr, unsigned int data)
{
    MmioWrite32(P_HDMI_ADDR_PORT, addr);
    MmioWrite32(P_HDMI_ADDR_PORT, addr);
    MmioWrite32(P_HDMI_DATA_PORT, data);
}

static unsigned int hdmi_rd_reg(unsigned int addr)
{
    MmioWrite32(P_HDMI_ADDR_PORT, addr);
    MmioWrite32(P_HDMI_ADDR_PORT, addr);
    
    return MmioRead32(P_HDMI_DATA_PORT);
}
/************************************** end **************************************/


// Use this self-made function rather than %, because % appears to produce wrong
// value for divisor which are not 2's exponential.
static unsigned int modulo(unsigned int a, unsigned int b)
{
    if (a >= b) {
        return(a-b);
    } else {
        return(a);
    }
}

static signed int to_signed(unsigned int a)
{
    if (a <= 7) {
        return(a);
    } else {
        return(a-16);
    }
}

static void hdmi_tvenc480i_set(HDMI_Video_Codes_t vic)
{
    unsigned long VFIFO2VD_TO_HDMI_LATENCY = 1; // Annie 01Sep2011: Change value from 2 to 1, due to video encoder path delay change.
    unsigned long TOTAL_PIXELS, PIXEL_REPEAT_HDMI, PIXEL_REPEAT_VENC, ACTIVE_PIXELS;
    unsigned FRONT_PORCH = 38, HSYNC_PIXELS = 124, ACTIVE_LINES = 0, INTERLACE_MODE, TOTAL_LINES, SOF_LINES, VSYNC_LINES;
    unsigned LINES_F0 = 262, LINES_F1 = 263, BACK_PORCH = 114, EOF_LINES = 2, TOTAL_FRAMES;

    unsigned long total_pixels_venc ;
    unsigned long active_pixels_venc;
    unsigned long front_porch_venc  ;
    unsigned long hsync_pixels_venc ;

    unsigned long de_h_begin, de_h_end;
    unsigned long de_v_begin_even, de_v_end_even, de_v_begin_odd, de_v_end_odd;
    unsigned long hs_begin, hs_end;
    unsigned long vs_adjust;
    unsigned long vs_bline_evn, vs_eline_evn, vs_bline_odd, vs_eline_odd;
    unsigned long vso_begin_evn, vso_begin_odd;

    if((vic == HDMI_480i60)||(vic == HDMI_480i60_16x9)){
         INTERLACE_MODE     = 1;                   
         PIXEL_REPEAT_VENC  = 1;                   
         PIXEL_REPEAT_HDMI  = 1;                   
         ACTIVE_PIXELS  =     (720*(1+PIXEL_REPEAT_HDMI)); // Number of active pixels per line.
         ACTIVE_LINES   =     (480/(1+INTERLACE_MODE));    // Number of active lines per field.
         LINES_F0           = 262;                 
         LINES_F1           = 263;                 
         FRONT_PORCH        = 38;                  
         HSYNC_PIXELS       = 124;                  
         BACK_PORCH         = 114;                  
         EOF_LINES          = 4;                   
         VSYNC_LINES        = 3;                   
         SOF_LINES          = 15;                  
         TOTAL_FRAMES       = 4;                   
    }
    else if((vic == HDMI_576i50)||(vic == HDMI_576i50_16x9)){
         INTERLACE_MODE     = 1;                   
         PIXEL_REPEAT_VENC  = 1;                   
         PIXEL_REPEAT_HDMI  = 1;                   
         ACTIVE_PIXELS  =     (720*(1+PIXEL_REPEAT_HDMI)); // Number of active pixels per line.
         ACTIVE_LINES   =     (576/(1+INTERLACE_MODE));    // Number of active lines per field.
         LINES_F0           = 312;                 
         LINES_F1           = 313;                 
         FRONT_PORCH        = 24;                  
         HSYNC_PIXELS       = 126;                  
         BACK_PORCH         = 138;                  
         EOF_LINES          = 2;                   
         VSYNC_LINES        = 3;                   
         SOF_LINES          = 19;                  
         TOTAL_FRAMES       = 4;                   
    }
    TOTAL_PIXELS =(FRONT_PORCH+HSYNC_PIXELS+BACK_PORCH+ACTIVE_PIXELS); // Number of total pixels per line.
    TOTAL_LINES  =(LINES_F0+(LINES_F1*INTERLACE_MODE));                // Number of total lines per frame.

    total_pixels_venc = (TOTAL_PIXELS  / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC); // 1716 / 2 * 2 = 1716
    active_pixels_venc= (ACTIVE_PIXELS / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC); // 1440 / 2 * 2 = 1440
    front_porch_venc  = (FRONT_PORCH   / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC); // 38   / 2 * 2 = 38
    hsync_pixels_venc = (HSYNC_PIXELS  / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC); // 124  / 2 * 2 = 124

    // Annie 01Sep2011: Comment out the following 2 lines. Because ENCP is not used for 480i and 576i.
    //hdmi_print(0, "[ENCP_VIDEO_MODE:%x]=%x\n",ENCP_VIDEO_MODE, Rd(ENCP_VIDEO_MODE));
    //Wr(ENCP_VIDEO_MODE,Rd(ENCP_VIDEO_MODE)|(1<<14)); // cfg_de_v = 1

    // Program DE timing
    // Annie 01Sep2011: for 480/576i, replace VFIFO2VD_PIXEL_START with ENCI_VFIFO2VD_PIXEL_START.
    DEBUG((DEBUG_INFO,"[ENCI_VFIFO2VD_PIXEL_START:%x]=%x\n",ENCI_VFIFO2VD_PIXEL_START, MmioRead32(P_ENCI_VFIFO2VD_PIXEL_START))); 
    de_h_begin = modulo(MmioRead32(P_ENCI_VFIFO2VD_PIXEL_START) + VFIFO2VD_TO_HDMI_LATENCY,   total_pixels_venc); // (233 + 2) % 1716 = 235
    de_h_end   = modulo(de_h_begin + active_pixels_venc,                            total_pixels_venc); // (235 + 1440) % 1716 = 1675
    MmioWrite32(P_ENCI_DE_H_BEGIN, de_h_begin);    // 235
    MmioWrite32(P_ENCI_DE_H_END,   de_h_end);      // 1675

    // Annie 01Sep2011: for 480/576i, replace VFIFO2VD_LINE_TOP/BOT_START with ENCI_VFIFO2VD_LINE_TOP/BOT_START.
    DEBUG((DEBUG_INFO, "[ENCI_VFIFO2VD_LINE_TOP_START:%x]=%x\n",ENCI_VFIFO2VD_LINE_TOP_START, MmioRead32(P_ENCI_VFIFO2VD_LINE_TOP_START))); 
    DEBUG((DEBUG_INFO, "[ENCI_VFIFO2VD_LINE_BOT_START:%x]=%x\n",ENCI_VFIFO2VD_LINE_BOT_START, MmioRead32(P_ENCI_VFIFO2VD_LINE_BOT_START))); 
    de_v_begin_even = MmioRead32(P_ENCI_VFIFO2VD_LINE_TOP_START);      // 17
    de_v_end_even   = de_v_begin_even + ACTIVE_LINES;   // 17 + 240 = 257
    de_v_begin_odd  = MmioRead32(P_ENCI_VFIFO2VD_LINE_BOT_START);      // 18
    de_v_end_odd    = de_v_begin_odd + ACTIVE_LINES;    // 18 + 480/2 = 258
    MmioWrite32(P_ENCI_DE_V_BEGIN_EVEN,de_v_begin_even);   // 17
    MmioWrite32(P_ENCI_DE_V_END_EVEN,  de_v_end_even);     // 257
    MmioWrite32(P_ENCI_DE_V_BEGIN_ODD, de_v_begin_odd);    // 18
    MmioWrite32(P_ENCI_DE_V_END_ODD,   de_v_end_odd);      // 258

    // Program Hsync timing
    if (de_h_end + front_porch_venc >= total_pixels_venc) {
        hs_begin    = de_h_end + front_porch_venc - total_pixels_venc;
        vs_adjust   = 1;
    } else {
        hs_begin    = de_h_end + front_porch_venc; // 1675 + 38 = 1713
        vs_adjust   = 1;
    }
    hs_end  = modulo(hs_begin + hsync_pixels_venc,   total_pixels_venc); // (1713 + 124) % 1716 = 121
    MmioWrite32(P_ENCI_DVI_HSO_BEGIN,  hs_begin);  // 1713
    MmioWrite32(P_ENCI_DVI_HSO_END,    hs_end);    // 121
    
    // Program Vsync timing for even field
    if (de_v_end_odd-1 + EOF_LINES + vs_adjust >= LINES_F1) {
        vs_bline_evn = de_v_end_odd-1 + EOF_LINES + vs_adjust - LINES_F1;
        vs_eline_evn = vs_bline_evn + VSYNC_LINES;
        MmioWrite32(P_ENCI_DVI_VSO_BLINE_EVN, vs_bline_evn);
        //vso_bline_evn_reg_wr_cnt ++;
        MmioWrite32(P_ENCI_DVI_VSO_ELINE_EVN, vs_eline_evn);
        //vso_eline_evn_reg_wr_cnt ++;
        MmioWrite32(P_ENCI_DVI_VSO_BEGIN_EVN, hs_begin);
        MmioWrite32(P_ENCI_DVI_VSO_END_EVN,   hs_begin);
    } else {
        vs_bline_odd = de_v_end_odd-1 + EOF_LINES + vs_adjust; // 258-1 + 4 + 0 = 261
        MmioWrite32(P_ENCI_DVI_VSO_BLINE_ODD, vs_bline_odd); // 261
        //vso_bline_odd_reg_wr_cnt ++;
        MmioWrite32(P_ENCI_DVI_VSO_BEGIN_ODD, hs_begin);  // 1713
        if (vs_bline_odd + VSYNC_LINES >= LINES_F1) {
            vs_eline_evn = vs_bline_odd + VSYNC_LINES - LINES_F1; // 261 + 3 - 263 = 1
            MmioWrite32(P_ENCI_DVI_VSO_ELINE_EVN, vs_eline_evn);   // 1
            //vso_eline_evn_reg_wr_cnt ++;
            MmioWrite32(P_ENCI_DVI_VSO_END_EVN,   hs_begin);       // 1713
        } else {
            vs_eline_odd = vs_bline_odd + VSYNC_LINES;
            MmioWrite32(P_ENCI_DVI_VSO_ELINE_ODD, vs_eline_odd);
            //vso_eline_odd_reg_wr_cnt ++;
            MmioWrite32(P_ENCI_DVI_VSO_END_ODD,   hs_begin);
        }
    }
    // Program Vsync timing for odd field
    if (de_v_end_even-1 + EOF_LINES + 1 >= LINES_F0) {
        vs_bline_odd = de_v_end_even-1 + EOF_LINES + 1 - LINES_F0;
        vs_eline_odd = vs_bline_odd + VSYNC_LINES;
        MmioWrite32(P_ENCI_DVI_VSO_BLINE_ODD, vs_bline_odd);
        //vso_bline_odd_reg_wr_cnt ++;
        MmioWrite32(P_ENCI_DVI_VSO_ELINE_ODD, vs_eline_odd);
        //vso_eline_odd_reg_wr_cnt ++;
        vso_begin_odd   = modulo(hs_begin + (total_pixels_venc>>1), total_pixels_venc);
        MmioWrite32(P_ENCI_DVI_VSO_BEGIN_ODD, vso_begin_odd);
        MmioWrite32(P_ENCI_DVI_VSO_END_ODD,   vso_begin_odd);
    } else {
        vs_bline_evn = de_v_end_even-1 + EOF_LINES + 1; // 257-1 + 4 + 1 = 261
        MmioWrite32(P_ENCI_DVI_VSO_BLINE_EVN, vs_bline_evn); // 261
        //vso_bline_evn_reg_wr_cnt ++;
        vso_begin_evn   = modulo(hs_begin + (total_pixels_venc>>1), total_pixels_venc);   // (1713 + 1716/2) % 1716 = 855
        MmioWrite32(P_ENCI_DVI_VSO_BEGIN_EVN, vso_begin_evn);  // 855
        if (vs_bline_evn + VSYNC_LINES >= LINES_F0) {
            vs_eline_odd = vs_bline_evn + VSYNC_LINES - LINES_F0; // 261 + 3 - 262 = 2
            MmioWrite32(P_ENCI_DVI_VSO_ELINE_ODD, vs_eline_odd);   // 2
            //vso_eline_odd_reg_wr_cnt ++;
            MmioWrite32(P_ENCI_DVI_VSO_END_ODD,   vso_begin_evn);  // 855
        } else {
            vs_eline_evn = vs_bline_evn + VSYNC_LINES;
            MmioWrite32(P_ENCI_DVI_VSO_ELINE_EVN, vs_eline_evn);
            //vso_eline_evn_reg_wr_cnt ++;
            MmioWrite32(P_ENCI_DVI_VSO_END_EVN,   vso_begin_evn);
        }
    }

    // Check if there are duplicate or missing timing settings
    //if ((vso_bline_evn_reg_wr_cnt != 1) || (vso_bline_odd_reg_wr_cnt != 1) ||
    //    (vso_eline_evn_reg_wr_cnt != 1) || (vso_eline_odd_reg_wr_cnt != 1)) {
        //stimulus_print("[TEST.C] Error: Multiple or missing timing settings on reg ENCI_DVI_VSO_B(E)LINE_EVN(ODD)!\n");
        //stimulus_finish_fail(1);
    //}

    // Annie 01Sep2011: Register VENC_DVI_SETTING and VENC_DVI_SETTING_MORE are no long valid, use VPU_HDMI_SETTING instead.
    MmioWrite32(P_VPU_HDMI_SETTING, (0                                 << 0) | // [    0] src_sel_enci
                         (0                                 << 1) | // [    1] src_sel_encp
                         (0                                 << 2) | // [    2] inv_hsync. 1=Invert Hsync polarity.
                         (0                                 << 3) | // [    3] inv_vsync. 1=Invert Vsync polarity.
                         (0                                 << 4) | // [    4] inv_dvi_clk. 1=Invert clock to external DVI, (clock invertion exists at internal HDMI).
                         (((1==0)?1:0)  << 5) | // [ 7: 5] data_comp_map. Input data is CrYCb(BRG), map the output data to desired format:
                                                                    //                          0=output CrYCb(BRG);
                                                                    //                          1=output YCbCr(RGB);
                                                                    //                          2=output YCrCb(RBG);
                                                                    //                          3=output CbCrY(GBR);
                                                                    //                          4=output CbYCr(GRB);
                                                                    //                          5=output CrCbY(BGR);
                                                                    //                          6,7=Rsrv.
                         (1                                 << 8) | // [11: 8] wr_rate. 0=A write every clk1; 1=A write every 2 clk1; ...; 15=A write every 16 clk1.
                         (1                                 <<12)   // [15:12] rd_rate. 0=A read every clk2; 1=A read every 2 clk2; ...; 15=A read every 16 clk2.
    );
    set_reg32_bits_op(P_VPU_HDMI_SETTING, 1, 0, 1);  // [    0] src_sel_enci: Enable ENCI output to HDMI

}    

static void hdmi_tvenc1080i_set(HDMI_Video_Codes_t vic)
{
    unsigned long VFIFO2VD_TO_HDMI_LATENCY = 2; // Annie 01Sep2011: Change value from 3 to 2, due to video encoder path delay change.
    unsigned long TOTAL_PIXELS, PIXEL_REPEAT_HDMI, PIXEL_REPEAT_VENC, ACTIVE_PIXELS;
    unsigned FRONT_PORCH = 88, HSYNC_PIXELS, ACTIVE_LINES = 0, INTERLACE_MODE, TOTAL_LINES, SOF_LINES, VSYNC_LINES;
    unsigned LINES_F0, LINES_F1 = 563,BACK_PORCH, EOF_LINES = 2, TOTAL_FRAMES;

    unsigned long total_pixels_venc ;
    unsigned long active_pixels_venc;
    unsigned long front_porch_venc  ;
    unsigned long hsync_pixels_venc ;

    unsigned long de_h_begin, de_h_end;
    unsigned long de_v_begin_even, de_v_end_even, de_v_begin_odd, de_v_end_odd;
    unsigned long hs_begin, hs_end;
    unsigned long vs_adjust;
    unsigned long vs_bline_evn, vs_eline_evn, vs_bline_odd, vs_eline_odd;
    unsigned long vso_begin_evn, vso_begin_odd;
    
    if(vic == HDMI_1080i60){
         INTERLACE_MODE     = 1;                   
         PIXEL_REPEAT_VENC  = 1;                   
         PIXEL_REPEAT_HDMI  = 0;                   
         ACTIVE_PIXELS  =     (1920*(1+PIXEL_REPEAT_HDMI)); // Number of active pixels per line.
         ACTIVE_LINES   =     (1080/(1+INTERLACE_MODE));    // Number of active lines per field.
         LINES_F0           = 562;                 
         LINES_F1           = 563;                 
         FRONT_PORCH        = 88;                  
         HSYNC_PIXELS       = 44;                  
         BACK_PORCH         = 148;                  
         EOF_LINES          = 2;                   
         VSYNC_LINES        = 5;                   
         SOF_LINES          = 15;                  
         TOTAL_FRAMES       = 4;                   
    }
    else if(vic == HDMI_1080i50){
         INTERLACE_MODE     = 1;                   
         PIXEL_REPEAT_VENC  = 1;                   
         PIXEL_REPEAT_HDMI  = 0;                   
         ACTIVE_PIXELS  =     (1920*(1+PIXEL_REPEAT_HDMI)); // Number of active pixels per line.
         ACTIVE_LINES   =     (1080/(1+INTERLACE_MODE));    // Number of active lines per field.
         LINES_F0           = 562;                 
         LINES_F1           = 563;                 
         FRONT_PORCH        = 528;                  
         HSYNC_PIXELS       = 44;                  
         BACK_PORCH         = 148;                  
         EOF_LINES          = 2;                   
         VSYNC_LINES        = 5;                   
         SOF_LINES          = 15;                  
         TOTAL_FRAMES       = 4;                   
    }
    TOTAL_PIXELS =(FRONT_PORCH+HSYNC_PIXELS+BACK_PORCH+ACTIVE_PIXELS); // Number of total pixels per line.
    TOTAL_LINES  =(LINES_F0+(LINES_F1*INTERLACE_MODE));                // Number of total lines per frame.

    total_pixels_venc = (TOTAL_PIXELS  / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC); // 2200 / 1 * 2 = 4400
    active_pixels_venc= (ACTIVE_PIXELS / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC); // 1920 / 1 * 2 = 3840
    front_porch_venc  = (FRONT_PORCH   / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC); // 88   / 1 * 2 = 176
    hsync_pixels_venc = (HSYNC_PIXELS  / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC); // 44   / 1 * 2 = 88

//    hdmi_print(0, "[ENCP_VIDEO_MODE:%x]=%x\n",ENCP_VIDEO_MODE, Rd(ENCP_VIDEO_MODE)); 
//    hdmi_print(0, "[ENCP_VIDEO_MODE:%x]=%x\n",ENCP_VIDEO_MODE, MmioRead32(P_ENCP_VIDEO_MODE)); 
    MmioWrite32(P_ENCP_VIDEO_MODE, MmioRead32(P_ENCP_VIDEO_MODE)|(1<<14)); // cfg_de_v = 1

    // Program DE timing
    DEBUG((DEBUG_INFO,"[ENCP_VIDEO_HAVON_BEGIN:%x]=%x\n",ENCP_VIDEO_HAVON_BEGIN, MmioRead32(P_ENCP_VIDEO_HAVON_BEGIN))); 
    de_h_begin = modulo(MmioRead32(P_ENCP_VIDEO_HAVON_BEGIN) + VFIFO2VD_TO_HDMI_LATENCY,  total_pixels_venc); // (383 + 3) % 4400 = 386
    de_h_end   = modulo(de_h_begin + active_pixels_venc,                        total_pixels_venc); // (386 + 3840) % 4400 = 4226
    MmioWrite32(P_ENCP_DE_H_BEGIN, de_h_begin);    // 386
    MmioWrite32(P_ENCP_DE_H_END,   de_h_end);      // 4226
    // Program DE timing for even field
    DEBUG((DEBUG_INFO, "[ENCP_VIDEO_VAVON_BLINE:%x]=%x\n",ENCP_VIDEO_VAVON_BLINE, MmioRead32(P_ENCP_VIDEO_VAVON_BLINE))); 
    de_v_begin_even = MmioRead32(P_ENCP_VIDEO_VAVON_BLINE);       // 20
    de_v_end_even   = de_v_begin_even + ACTIVE_LINES;   // 20 + 540 = 560
    MmioWrite32(P_ENCP_DE_V_BEGIN_EVEN,de_v_begin_even);   // 20
    MmioWrite32(P_ENCP_DE_V_END_EVEN,  de_v_end_even);     // 560
    // Program DE timing for odd field if needed
    if (INTERLACE_MODE) {
        // Calculate de_v_begin_odd according to enc480p_timing.v:
        //wire[10:0]	cfg_ofld_vavon_bline	= {{7{ofld_vavon_ofst1 [3]}},ofld_vavon_ofst1 [3:0]} + cfg_video_vavon_bline	+ ofld_line;
        DEBUG((DEBUG_INFO, "[ENCP_VIDEO_OFLD_VOAV_OFST:%x]=%x\n",ENCP_VIDEO_OFLD_VOAV_OFST, MmioRead32(P_ENCP_VIDEO_OFLD_VOAV_OFST))); 
        de_v_begin_odd  = to_signed((MmioRead32(P_ENCP_VIDEO_OFLD_VOAV_OFST) & 0xf0)>>4) + de_v_begin_even + (TOTAL_LINES-1)/2; // 1 + 20 + (1125-1)/2 = 583
        de_v_end_odd    = de_v_begin_odd + ACTIVE_LINES;    // 583 + 540 = 1123
        MmioWrite32(P_ENCP_DE_V_BEGIN_ODD, de_v_begin_odd);// 583
        MmioWrite32(P_ENCP_DE_V_END_ODD,   de_v_end_odd);  // 1123
    }

    // Program Hsync timing
    if (de_h_end + front_porch_venc >= total_pixels_venc) {
        hs_begin    = de_h_end + front_porch_venc - total_pixels_venc; // 4226 + 176 - 4400 = 2





        vs_adjust   = 1;
    } else {
        hs_begin    = de_h_end + front_porch_venc;
        vs_adjust   = 0;
    }
    hs_end  = modulo(hs_begin + hsync_pixels_venc,   total_pixels_venc); // (2 + 88) % 4400 = 90
    MmioWrite32(P_ENCP_DVI_HSO_BEGIN,  hs_begin);  // 2
    MmioWrite32(P_ENCP_DVI_HSO_END,    hs_end);    // 90
    
    // Program Vsync timing for even field
    if (de_v_begin_even >= SOF_LINES + VSYNC_LINES + (1-vs_adjust)) {
        vs_bline_evn = de_v_begin_even - SOF_LINES - VSYNC_LINES - (1-vs_adjust); // 20 - 15 - 5 - 0 = 0
    } else {
        vs_bline_evn = TOTAL_LINES + de_v_begin_even - SOF_LINES - VSYNC_LINES - (1-vs_adjust);
    }
    vs_eline_evn = modulo(vs_bline_evn + VSYNC_LINES, TOTAL_LINES); // (0 + 5) % 1125 = 5
    MmioWrite32(P_ENCP_DVI_VSO_BLINE_EVN, vs_bline_evn);   // 0
    MmioWrite32(P_ENCP_DVI_VSO_ELINE_EVN, vs_eline_evn);   // 5
    vso_begin_evn = hs_begin; // 2
    MmioWrite32(P_ENCP_DVI_VSO_BEGIN_EVN, vso_begin_evn);  // 2
    MmioWrite32(P_ENCP_DVI_VSO_END_EVN,   vso_begin_evn);  // 2
    // Program Vsync timing for odd field if needed
    if (INTERLACE_MODE) {
        vs_bline_odd = de_v_begin_odd-1 - SOF_LINES - VSYNC_LINES;  // 583-1 - 15 - 5   = 562
        vs_eline_odd = de_v_begin_odd-1 - SOF_LINES;                // 583-1 - 15       = 567
        vso_begin_odd   = modulo(hs_begin + (total_pixels_venc>>1), total_pixels_venc); // (2 + 4400/2) % 4400 = 2202
        MmioWrite32(P_ENCP_DVI_VSO_BLINE_ODD, vs_bline_odd);   // 562
        MmioWrite32(P_ENCP_DVI_VSO_ELINE_ODD, vs_eline_odd);   // 567
        MmioWrite32(P_ENCP_DVI_VSO_BEGIN_ODD, vso_begin_odd);  // 2202
        MmioWrite32(P_ENCP_DVI_VSO_END_ODD,   vso_begin_odd);  // 2202
    }

    // Annie 01Sep2011: Register VENC_DVI_SETTING and VENC_DVI_SETTING_MORE are no long valid, use VPU_HDMI_SETTING instead.
    MmioWrite32(P_VPU_HDMI_SETTING, (0                                 << 0) | // [    0] src_sel_enci
                         (0                                 << 1) | // [    1] src_sel_encp
                         (1                    << 2) | // [    2] inv_hsync. 1=Invert Hsync polarity.
                         (1                    << 3) | // [    3] inv_vsync. 1=Invert Vsync polarity.
                         (0                                 << 4) | // [    4] inv_dvi_clk. 1=Invert clock to external DVI, (clock invertion exists at internal HDMI).
                         (((1==0)?1:0)  << 5) | // [ 7: 5] data_comp_map. Input data is CrYCb(BRG), map the output data to desired format:
                                                                    //                          0=output CrYCb(BRG);
                                                                    //                          1=output YCbCr(RGB);
                                                                    //                          2=output YCrCb(RBG);
                                                                    //                          3=output CbCrY(GBR);
                                                                    //                          4=output CbYCr(GRB);
                                                                    //                          5=output CrCbY(BGR);
                                                                    //                          6,7=Rsrv.
#ifdef DOUBLE_CLK_720P_1080I
                         (0                                 << 8) | // [11: 8] wr_rate. 0=A write every clk1; 1=A write every 2 clk1; ...; 15=A write every 16 clk1.
#else                         
                         (1                                 << 8) | // [11: 8] wr_rate. 0=A write every clk1; 1=A write every 2 clk1; ...; 15=A write every 16 clk1.
#endif                         
                         (0                                 <<12)   // [15:12] rd_rate. 0=A read every clk2; 1=A read every 2 clk2; ...; 15=A read every 16 clk2.
    );
    set_reg32_bits_op(P_VPU_HDMI_SETTING, 1, 1, 1);  // [    1] src_sel_encp: Enable ENCP output to HDMI

}    

static void hdmi_tvenc4k2k_set(HDMI_Video_Codes_t vic)
{
    unsigned long VFIFO2VD_TO_HDMI_LATENCY = 2; // Annie 01Sep2011: Change value from 3 to 2, due to video encoder path delay change.
    unsigned long TOTAL_PIXELS = 4400, PIXEL_REPEAT_HDMI = 0, PIXEL_REPEAT_VENC = 0, ACTIVE_PIXELS = 3840;
    unsigned FRONT_PORCH = 1020, HSYNC_PIXELS = 0, ACTIVE_LINES = 2160, INTERLACE_MODE = 0, TOTAL_LINES = 0, SOF_LINES = 0, VSYNC_LINES = 0;
    unsigned LINES_F0 = 2250, LINES_F1 = 2250, BACK_PORCH = 0, EOF_LINES = 8, TOTAL_FRAMES = 0;

    unsigned long total_pixels_venc ;
    unsigned long active_pixels_venc;
    unsigned long front_porch_venc  ;
    unsigned long hsync_pixels_venc ;

    unsigned long de_h_begin, de_h_end;
    unsigned long de_v_begin_even, de_v_end_even, de_v_begin_odd, de_v_end_odd;
    unsigned long hs_begin, hs_end;
    unsigned long vs_adjust;
    unsigned long vs_bline_evn, vs_eline_evn, vs_bline_odd, vs_eline_odd;
    unsigned long vso_begin_evn, vso_begin_odd;

    if(vic == HDMI_4k2k_30){
         INTERLACE_MODE     = 0;
         PIXEL_REPEAT_VENC  = 0;
         PIXEL_REPEAT_HDMI  = 0;
         ACTIVE_PIXELS  =     (3840*(1+PIXEL_REPEAT_HDMI)); // Number of active pixels per line.
         ACTIVE_LINES   =     (2160/(1+INTERLACE_MODE));    // Number of active lines per field.
         LINES_F0           = 2250;
         LINES_F1           = 2250;
         FRONT_PORCH        = 176;
         HSYNC_PIXELS       = 88;
         BACK_PORCH         = 296;
         EOF_LINES          = 8 + 1;
         VSYNC_LINES        = 10;
         SOF_LINES          = 72 + 1;
         TOTAL_FRAMES       = 3;
    }
    else if(vic == HDMI_4k2k_25){
         INTERLACE_MODE     = 0;
         PIXEL_REPEAT_VENC  = 0;
         PIXEL_REPEAT_HDMI  = 0;
         ACTIVE_PIXELS  =     (3840*(1+PIXEL_REPEAT_HDMI)); // Number of active pixels per line.
         ACTIVE_LINES   =     (2160/(1+INTERLACE_MODE));    // Number of active lines per field.
         LINES_F0           = 2250;
         LINES_F1           = 2250;
         FRONT_PORCH        = 1056;
         HSYNC_PIXELS       = 88;
         BACK_PORCH         = 296;
         EOF_LINES          = 8 + 1;
         VSYNC_LINES        = 10;
         SOF_LINES          = 72 + 1;
         TOTAL_FRAMES       = 3;
    }
    else if(vic == HDMI_4k2k_24){
         INTERLACE_MODE     = 0;
         PIXEL_REPEAT_VENC  = 0;
         PIXEL_REPEAT_HDMI  = 0;
         ACTIVE_PIXELS  =     (3840*(1+PIXEL_REPEAT_HDMI)); // Number of active pixels per line.
         ACTIVE_LINES   =     (2160/(1+INTERLACE_MODE));    // Number of active lines per field.
         LINES_F0           = 2250;
         LINES_F1           = 2250;
         FRONT_PORCH        = 1276;
         HSYNC_PIXELS       = 88;
         BACK_PORCH         = 296;
         EOF_LINES          = 8 + 1;
         VSYNC_LINES        = 10;
         SOF_LINES          = 72 + 1;
         TOTAL_FRAMES       = 3;
    }
    else if(vic == HDMI_4k2k_smpte){
         INTERLACE_MODE     = 0;
         PIXEL_REPEAT_VENC  = 0;
         PIXEL_REPEAT_HDMI  = 0;
         ACTIVE_PIXELS  =     (4096*(1+PIXEL_REPEAT_HDMI)); // Number of active pixels per line.
         ACTIVE_LINES   =     (2160/(1+INTERLACE_MODE));    // Number of active lines per field.
         LINES_F0           = 2250;
         LINES_F1           = 2250;
         FRONT_PORCH        = 1020;
         HSYNC_PIXELS       = 88;
         BACK_PORCH         = 296;
         EOF_LINES          = 8 + 1;
         VSYNC_LINES        = 10;
         SOF_LINES          = 72 + 1;
         TOTAL_FRAMES       = 3;
    }
    else {
        // nothing
    }
    total_pixels_venc = (TOTAL_PIXELS  / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC);
    active_pixels_venc= (ACTIVE_PIXELS / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC);
    front_porch_venc  = (FRONT_PORCH   / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC);
    hsync_pixels_venc = (HSYNC_PIXELS  / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC);

    de_h_begin = modulo(MmioRead32(P_ENCP_VIDEO_HAVON_BEGIN) + VFIFO2VD_TO_HDMI_LATENCY,  total_pixels_venc);
    de_h_end   = modulo(de_h_begin + active_pixels_venc,                        total_pixels_venc);
    MmioWrite32(P_ENCP_DE_H_BEGIN, de_h_begin);
    MmioWrite32(P_ENCP_DE_H_END,   de_h_end);
    // Program DE timing for even field
    de_v_begin_even = MmioRead32(P_ENCP_VIDEO_VAVON_BLINE);
    de_v_end_even   = modulo(de_v_begin_even + ACTIVE_LINES, TOTAL_LINES);
    MmioWrite32(P_ENCP_DE_V_BEGIN_EVEN,de_v_begin_even);
    MmioWrite32(P_ENCP_DE_V_END_EVEN,  de_v_end_even);
    // Program DE timing for odd field if needed
    if (INTERLACE_MODE) {
        // Calculate de_v_begin_odd according to enc480p_timing.v:
        //wire[10:0]	cfg_ofld_vavon_bline	= {{7{ofld_vavon_ofst1 [3]}},ofld_vavon_ofst1 [3:0]} + cfg_video_vavon_bline	+ ofld_line;
        de_v_begin_odd  = to_signed((MmioRead32(P_ENCP_VIDEO_OFLD_VOAV_OFST) & 0xf0)>>4) + de_v_begin_even + (TOTAL_LINES-1)/2;
        de_v_end_odd    = modulo(de_v_begin_odd + ACTIVE_LINES, TOTAL_LINES);
        MmioWrite32(P_ENCP_DE_V_BEGIN_ODD, de_v_begin_odd);
        MmioWrite32(P_ENCP_DE_V_END_ODD,   de_v_end_odd);
    }

    // Program Hsync timing
    if (de_h_end + front_porch_venc >= total_pixels_venc) {
        hs_begin    = de_h_end + front_porch_venc - total_pixels_venc;
        vs_adjust   = 1;
    } else {
        hs_begin    = de_h_end + front_porch_venc;
        vs_adjust   = 0;
    }
    hs_end  = modulo(hs_begin + hsync_pixels_venc,   total_pixels_venc);
    MmioWrite32(P_ENCP_DVI_HSO_BEGIN,  hs_begin);
    MmioWrite32(P_ENCP_DVI_HSO_END,    hs_end);
    
    // Program Vsync timing for even field
    if (de_v_begin_even >= SOF_LINES + VSYNC_LINES + (1-vs_adjust)) {
        vs_bline_evn = de_v_begin_even - SOF_LINES - VSYNC_LINES - (1-vs_adjust);
    } else {
        vs_bline_evn = TOTAL_LINES + de_v_begin_even - SOF_LINES - VSYNC_LINES - (1-vs_adjust);
    }
    vs_eline_evn = modulo(vs_bline_evn + VSYNC_LINES, TOTAL_LINES);
    MmioWrite32(P_ENCP_DVI_VSO_BLINE_EVN, vs_bline_evn);
    MmioWrite32(P_ENCP_DVI_VSO_ELINE_EVN, vs_eline_evn);
    vso_begin_evn = hs_begin;
    MmioWrite32(P_ENCP_DVI_VSO_BEGIN_EVN, vso_begin_evn);
    MmioWrite32(P_ENCP_DVI_VSO_END_EVN,   vso_begin_evn);
    // Program Vsync timing for odd field if needed
    if (INTERLACE_MODE) {
        vs_bline_odd = de_v_begin_odd-1 - SOF_LINES - VSYNC_LINES;
        vs_eline_odd = de_v_begin_odd-1 - SOF_LINES;
        vso_begin_odd   = modulo(hs_begin + (total_pixels_venc>>1), total_pixels_venc);
        MmioWrite32(P_ENCP_DVI_VSO_BLINE_ODD, vs_bline_odd);
        MmioWrite32(P_ENCP_DVI_VSO_ELINE_ODD, vs_eline_odd);
        MmioWrite32(P_ENCP_DVI_VSO_BEGIN_ODD, vso_begin_odd);
        MmioWrite32(P_ENCP_DVI_VSO_END_ODD,   vso_begin_odd);
    }
    MmioWrite32(P_VPU_HDMI_SETTING, (0                                 << 0) | // [    0] src_sel_enci
                         (0                                 << 1) | // [    1] src_sel_encp
                         (HSYNC_POLARITY                    << 2) | // [    2] inv_hsync. 1=Invert Hsync polarity.
                         (VSYNC_POLARITY                    << 3) | // [    3] inv_vsync. 1=Invert Vsync polarity.
                         (0                                 << 4) | // [    4] inv_dvi_clk. 1=Invert clock to external DVI, (clock invertion exists at internal HDMI).
                         (((1==0)?1:0)  << 5) | // [ 7: 5] data_comp_map. Input data is CrYCb(BRG), map the output data to desired format:
                                                                    //                          0=output CrYCb(BRG);
                                                                    //                          1=output YCbCr(RGB);
                                                                    //                          2=output YCrCb(RBG);
                                                                    //                          3=output CbCrY(GBR);
                                                                    //                          4=output CbYCr(GRB);
                                                                    //                          5=output CrCbY(BGR);
                                                                    //                          6,7=Rsrv.
                         (0                                 << 8) | // [11: 8] wr_rate. 0=A write every clk1; 1=A write every 2 clk1; ...; 15=A write every 16 clk1.
                         (0                                 <<12)   // [15:12] rd_rate. 0=A read every clk2; 1=A read every 2 clk2; ...; 15=A read every 16 clk2.
    );
    set_reg32_bits_op(P_VPU_HDMI_SETTING, 1, 1, 1);  // [    1] src_sel_encp: Enable ENCP output to HDMI
    MmioWrite32(P_ENCP_VIDEO_EN, 1); // Enable VENC
}

static void hdmi_tvenc_set(HDMI_Video_Codes_t vic)
{
	
    unsigned long VFIFO2VD_TO_HDMI_LATENCY = 2; // Annie 01Sep2011: Change value from 3 to 2, due to video encoder path delay change.
    unsigned long TOTAL_PIXELS, PIXEL_REPEAT_HDMI, PIXEL_REPEAT_VENC, ACTIVE_PIXELS;
    unsigned FRONT_PORCH, HSYNC_PIXELS, ACTIVE_LINES, INTERLACE_MODE, TOTAL_LINES, SOF_LINES, VSYNC_LINES;
    unsigned LINES_F0, LINES_F1,BACK_PORCH, EOF_LINES, TOTAL_FRAMES;

    unsigned long total_pixels_venc ;
    unsigned long active_pixels_venc;
    unsigned long front_porch_venc  ;
    unsigned long hsync_pixels_venc ;

    unsigned long de_h_begin, de_h_end;
    unsigned long de_v_begin_even, de_v_end_even, de_v_begin_odd, de_v_end_odd;
    unsigned long hs_begin, hs_end;
    unsigned long vs_adjust;
    unsigned long vs_bline_evn, vs_eline_evn, vs_bline_odd, vs_eline_odd;
    unsigned long vso_begin_evn, vso_begin_odd;

    if((vic == HDMI_480p60)||(vic == HDMI_480p60_16x9)){
         INTERLACE_MODE     = 0;                   
         PIXEL_REPEAT_VENC  = 1;                   
         PIXEL_REPEAT_HDMI  = 0;                   
         ACTIVE_PIXELS      = (720*(1+PIXEL_REPEAT_HDMI)); // Number of active pixels per line.
         ACTIVE_LINES       = (480/(1+INTERLACE_MODE));    // Number of active lines per field.
         LINES_F0           = 525;                 
         LINES_F1           = 525;                 
         FRONT_PORCH        = 16;                  
         HSYNC_PIXELS       = 62;                  
         BACK_PORCH         = 60;                  
         EOF_LINES          = 9;                   
         VSYNC_LINES        = 6;                   
         SOF_LINES          = 30;                  
         TOTAL_FRAMES       = 4;                   
    }
    else if((vic == HDMI_576p50)||(vic == HDMI_576p50_16x9)){
         INTERLACE_MODE     = 0;                   
         PIXEL_REPEAT_VENC  = 1;                   
         PIXEL_REPEAT_HDMI  = 0;                   
         ACTIVE_PIXELS      = (720*(1+PIXEL_REPEAT_HDMI)); // Number of active pixels per line.
         ACTIVE_LINES       = (576/(1+INTERLACE_MODE));    // Number of active lines per field.
         LINES_F0           = 625;                 
         LINES_F1           = 625;                 
         FRONT_PORCH        = 12;                  
         HSYNC_PIXELS       = 64;                  
         BACK_PORCH         = 68;                  
         EOF_LINES          = 5;                   
         VSYNC_LINES        = 5;                   
         SOF_LINES          = 39;                  
         TOTAL_FRAMES       = 4;                   
    }
    else if(vic == HDMI_720p60){
         INTERLACE_MODE     = 0;                   
         PIXEL_REPEAT_VENC  = 1;                   
         PIXEL_REPEAT_HDMI  = 0;                   
         ACTIVE_PIXELS      = (1280*(1+PIXEL_REPEAT_HDMI)); // Number of active pixels per line.
         ACTIVE_LINES       = (720/(1+INTERLACE_MODE));    // Number of active lines per field.
         LINES_F0           = 750;                 
         LINES_F1           = 750;                 
         FRONT_PORCH        = 110;                  
         HSYNC_PIXELS       = 40;                  
         BACK_PORCH         = 220;                  
         EOF_LINES          = 5;                   
         VSYNC_LINES        = 5;                   
         SOF_LINES          = 20;                  
         TOTAL_FRAMES       = 4;                   
    }
    else if(vic == HDMI_720p50){
         INTERLACE_MODE     = 0;                   
         PIXEL_REPEAT_VENC  = 1;                   
         PIXEL_REPEAT_HDMI  = 0;                   
         ACTIVE_PIXELS      = (1280*(1+PIXEL_REPEAT_HDMI)); // Number of active pixels per line.
         ACTIVE_LINES       = (720/(1+INTERLACE_MODE));    // Number of active lines per field.
         LINES_F0           = 750;                 
         LINES_F1           = 750;                 
         FRONT_PORCH        = 440;                  
         HSYNC_PIXELS       = 40;                  
         BACK_PORCH         = 220;                  
         EOF_LINES          = 5;                   
         VSYNC_LINES        = 5;                   
         SOF_LINES          = 20;                  
         TOTAL_FRAMES       = 4;                   
    }
    else if(vic == HDMI_1080p50){
         INTERLACE_MODE      =0;              
         PIXEL_REPEAT_VENC   =0;              
         PIXEL_REPEAT_HDMI   =0;              
         ACTIVE_PIXELS       =(1920*(1+PIXEL_REPEAT_HDMI)); // Number of active pixels per line.
         ACTIVE_LINES        =(1080/(1+INTERLACE_MODE));    // Number of active lines per field.
         LINES_F0            =1125;           
         LINES_F1            =1125;           
         FRONT_PORCH         =528;             
         HSYNC_PIXELS        =44;             
         BACK_PORCH          =148;            
         EOF_LINES           =4;              
         VSYNC_LINES         =5;              
         SOF_LINES           =36;             
         TOTAL_FRAMES        =4;              
    }
    else if(vic == HDMI_1080p24){//1080p24 support
         INTERLACE_MODE      =0;              
         PIXEL_REPEAT_VENC   =0;              
         PIXEL_REPEAT_HDMI   =0;              
         ACTIVE_PIXELS       =(1920*(1+PIXEL_REPEAT_HDMI)); // Number of active pixels per line.
         ACTIVE_LINES        =(1080/(1+INTERLACE_MODE));    // Number of active lines per field.
         LINES_F0            =1125;           
         LINES_F1            =1125;           
         FRONT_PORCH         =638;             
         HSYNC_PIXELS        =44;             
         BACK_PORCH          =148;            
         EOF_LINES           =4;              
         VSYNC_LINES         =5;              
         SOF_LINES           =36;             
         TOTAL_FRAMES        =4;    
    }
    else{ //HDMI_1080p60, HDMI_1080p30
         INTERLACE_MODE      =0;              
         PIXEL_REPEAT_VENC   =0;              
         PIXEL_REPEAT_HDMI   =0;              
         ACTIVE_PIXELS       =(1920*(1+PIXEL_REPEAT_HDMI)); // Number of active pixels per line.
         ACTIVE_LINES        =(1080/(1+INTERLACE_MODE));    // Number of active lines per field.
         LINES_F0            =1125;           
         LINES_F1            =1125;           
         FRONT_PORCH         =88;             
         HSYNC_PIXELS        =44;             
         BACK_PORCH          =148;            
         EOF_LINES           =4;              
         VSYNC_LINES         =5;              
         SOF_LINES           =36;             
         TOTAL_FRAMES        =4;              
    }
	
    TOTAL_PIXELS       = (FRONT_PORCH+HSYNC_PIXELS+BACK_PORCH+ACTIVE_PIXELS); // Number of total pixels per line.
    TOTAL_LINES        = (LINES_F0+(LINES_F1*INTERLACE_MODE));                // Number of total lines per frame.

    total_pixels_venc = (TOTAL_PIXELS  / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC); // 858 / 1 * 2 = 1716
    active_pixels_venc= (ACTIVE_PIXELS / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC); // 720 / 1 * 2 = 1440
    front_porch_venc  = (FRONT_PORCH   / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC); // 16   / 1 * 2 = 32
    hsync_pixels_venc = (HSYNC_PIXELS  / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC); // 62   / 1 * 2 = 124
	
    DEBUG((DEBUG_INFO, "[ENCP_VIDEO_MODE:%x]=%x\n",ENCP_VIDEO_MODE, MmioRead32(P_ENCP_VIDEO_MODE))); 
    MmioWrite32(P_ENCP_VIDEO_MODE,MmioRead32(P_ENCP_VIDEO_MODE)|(1<<14)); // cfg_de_v = 1
    // Program DE timing
    //hdmi_print(0, "[ENCP_VIDEO_HAVON_BEGIN:%x]=%x\n",ENCP_VIDEO_HAVON_BEGIN, MmioRead32(P_ENCP_VIDEO_HAVON_BEGIN)); 
    de_h_begin = modulo(MmioRead32(P_ENCP_VIDEO_HAVON_BEGIN) + VFIFO2VD_TO_HDMI_LATENCY,  total_pixels_venc); // (217 + 3) % 1716 = 220
    de_h_end   = modulo(de_h_begin + active_pixels_venc,                        total_pixels_venc); // (220 + 1440) % 1716 = 1660
    MmioWrite32(P_ENCP_DE_H_BEGIN, de_h_begin);    // 220
    MmioWrite32(P_ENCP_DE_H_END,   de_h_end);      // 1660
    // Program DE timing for even field
    //hdmi_print(0, "[ENCP_VIDEO_VAVON_BLINE:%x]=%x\n",ENCP_VIDEO_VAVON_BLINE, MmioRead32(P_ENCP_VIDEO_VAVON_BLINE)); 
    de_v_begin_even = MmioRead32(P_ENCP_VIDEO_VAVON_BLINE);       // 42
    de_v_end_even   = de_v_begin_even + ACTIVE_LINES;   // 42 + 480 = 522
    MmioWrite32(P_ENCP_DE_V_BEGIN_EVEN,de_v_begin_even);   // 42
    MmioWrite32(P_ENCP_DE_V_END_EVEN,  de_v_end_even);     // 522
    // Program DE timing for odd field if needed
    if (INTERLACE_MODE) {
        // Calculate de_v_begin_odd according to enc480p_timing.v:
        //wire[10:0]    cfg_ofld_vavon_bline    = {{7{ofld_vavon_ofst1 [3]}},ofld_vavon_ofst1 [3:0]} + cfg_video_vavon_bline    + ofld_line;
        //hdmi_print(0, "[ENCP_VIDEO_OFLD_VOAV_OFST:%x]=%x\n",ENCP_VIDEO_OFLD_VOAV_OFST, MmioRead32(P_ENCP_VIDEO_OFLD_VOAV_OFST)); 
        de_v_begin_odd  = to_signed((MmioRead32(P_ENCP_VIDEO_OFLD_VOAV_OFST) & 0xf0)>>4) + de_v_begin_even + (TOTAL_LINES-1)/2;
        de_v_end_odd    = de_v_begin_odd + ACTIVE_LINES;
        MmioWrite32(P_ENCP_DE_V_BEGIN_ODD, de_v_begin_odd);
        MmioWrite32(P_ENCP_DE_V_END_ODD,   de_v_end_odd);
    }
    // Program Hsync timing
    if (de_h_end + front_porch_venc >= total_pixels_venc) {
        hs_begin    = de_h_end + front_porch_venc - total_pixels_venc;
        vs_adjust   = 1;
    } else {
        hs_begin    = de_h_end + front_porch_venc; // 1660 + 32 = 1692
        vs_adjust   = 0;
    }
    hs_end  = modulo(hs_begin + hsync_pixels_venc,   total_pixels_venc); // (1692 + 124) % 1716 = 100
    MmioWrite32(P_ENCP_DVI_HSO_BEGIN,  hs_begin);  // 1692
    MmioWrite32(P_ENCP_DVI_HSO_END,    hs_end);    // 100  
    // Program Vsync timing for even field
    if (de_v_begin_even >= SOF_LINES + VSYNC_LINES + (1-vs_adjust)) {
        vs_bline_evn = de_v_begin_even - SOF_LINES - VSYNC_LINES - (1-vs_adjust); // 42 - 30 - 6 - 1 = 5
    } else {
        vs_bline_evn = TOTAL_LINES + de_v_begin_even - SOF_LINES - VSYNC_LINES - (1-vs_adjust);
    }
    vs_eline_evn = modulo(vs_bline_evn + VSYNC_LINES, TOTAL_LINES); // (5 + 6) % 525 = 11
    MmioWrite32(P_ENCP_DVI_VSO_BLINE_EVN, vs_bline_evn);   // 5
    MmioWrite32(P_ENCP_DVI_VSO_ELINE_EVN, vs_eline_evn);   // 11
    vso_begin_evn = hs_begin; // 1692
    MmioWrite32(P_ENCP_DVI_VSO_BEGIN_EVN, vso_begin_evn);  // 1692
    MmioWrite32(P_ENCP_DVI_VSO_END_EVN,   vso_begin_evn);  // 1692
    // Program Vsync timing for odd field if needed
    if (INTERLACE_MODE) {
        vs_bline_odd = de_v_begin_odd-1 - SOF_LINES - VSYNC_LINES;
        vs_eline_odd = de_v_begin_odd-1 - SOF_LINES;
        vso_begin_odd   = modulo(hs_begin + (total_pixels_venc>>1), total_pixels_venc);
        MmioWrite32(P_ENCP_DVI_VSO_BLINE_ODD, vs_bline_odd);
        MmioWrite32(P_ENCP_DVI_VSO_ELINE_ODD, vs_eline_odd);
        MmioWrite32(P_ENCP_DVI_VSO_BEGIN_ODD, vso_begin_odd);
        MmioWrite32(P_ENCP_DVI_VSO_END_ODD,   vso_begin_odd);
    }
    // Annie 01Sep2011: Remove the following line as register VENC_DVI_SETTING_MORE is no long valid, use VPU_HDMI_SETTING instead.
    //Wr(VENC_DVI_SETTING_MORE, (TX_INPUT_COLOR_FORMAT==0)? 1 : 0); // [0] 0=Map data pins from Venc to Hdmi Tx as CrYCb mode;
    switch(vic)
    {
        case HDMI_480p60:
        case HDMI_480p60_16x9:
        case HDMI_576p50:
        case HDMI_576p50_16x9:
//Note: Hsync & Vsync polarity should be negative.
//Refer to HDMI CTS 1.4A Page 169
            // Annie 01Sep2011: Register VENC_DVI_SETTING and VENC_DVI_SETTING_MORE are no long valid, use VPU_HDMI_SETTING instead.
            MmioWrite32(P_VPU_HDMI_SETTING, (0                                 << 0) | // [    0] src_sel_enci
                                 (0                                 << 1) | // [    1] src_sel_encp
                                 (0                                 << 2) | // [    2] inv_hsync. 1=Invert Hsync polarity.
                                 (0                                 << 3) | // [    3] inv_vsync. 1=Invert Vsync polarity.
                                 (0                                 << 4) | // [    4] inv_dvi_clk. 1=Invert clock to external DVI, (clock invertion exists at internal HDMI).
                                 (((1==0)?1:0)  << 5) | // [ 7: 5] data_comp_map. Input data is CrYCb(BRG), map the output data to desired format:
                                                                            //                          0=output CrYCb(BRG);
                                                                            //                          1=output YCbCr(RGB);
                                                                            //                          2=output YCrCb(RBG);
                                                                            //                          3=output CbCrY(GBR);
                                                                            //                          4=output CbYCr(GRB);
                                                                            //                          5=output CrCbY(BGR);
                                                                            //                          6,7=Rsrv.
                                 (1                                 << 8) | // [11: 8] wr_rate. 0=A write every clk1; 1=A write every 2 clk1; ...; 15=A write every 16 clk1.
                                 (0                                 <<12)   // [15:12] rd_rate. 0=A read every clk2; 1=A read every 2 clk2; ...; 15=A read every 16 clk2.
            );
            break;
        case HDMI_720p60:
        case HDMI_720p50:
            // Annie 01Sep2011: Register VENC_DVI_SETTING and VENC_DVI_SETTING_MORE are no long valid, use VPU_HDMI_SETTING instead.
            MmioWrite32(P_VPU_HDMI_SETTING, (0                                 << 0) | // [    0] src_sel_enci
                                 (0                                 << 1) | // [    1] src_sel_encp
                                 (HSYNC_POLARITY                    << 2) | // [    2] inv_hsync. 1=Invert Hsync polarity.
                                 (VSYNC_POLARITY                    << 3) | // [    3] inv_vsync. 1=Invert Vsync polarity.
                                 (0                                 << 4) | // [    4] inv_dvi_clk. 1=Invert clock to external DVI, (clock invertion exists at internal HDMI).
                                 (((1==0)?1:0)  << 5) | // [ 7: 5] data_comp_map. Input data is CrYCb(BRG), map the output data to desired format:
                                                                            //                          0=output CrYCb(BRG);
                                                                            //                          1=output YCbCr(RGB);
                                                                            //                          2=output YCrCb(RBG);
                                                                            //                          3=output CbCrY(GBR);
                                                                            //                          4=output CbYCr(GRB);
                                                                            //                          5=output CrCbY(BGR);
                                                                            //                          6,7=Rsrv.
#ifdef DOUBLE_CLK_720P_1080I
                                 (0                                 << 8) | // [11: 8] wr_rate. 0=A write every clk1; 1=A write every 2 clk1; ...; 15=A write every 16 clk1.
#else
                                 (1                                 << 8) | // [11: 8] wr_rate. 0=A write every clk1; 1=A write every 2 clk1; ...; 15=A write every 16 clk1.
#endif                             
                                 (0                                 <<12)   // [15:12] rd_rate. 0=A read every clk2; 1=A read every 2 clk2; ...; 15=A read every 16 clk2.
            );
            break;
        default:
            // Annie 01Sep2011: Register VENC_DVI_SETTING and VENC_DVI_SETTING_MORE are no long valid, use VPU_HDMI_SETTING instead.
            MmioWrite32(P_VPU_HDMI_SETTING, (0                                 << 0) | // [    0] src_sel_enci
                                 (0                                 << 1) | // [    1] src_sel_encp
                                 (HSYNC_POLARITY                    << 2) | // [    2] inv_hsync. 1=Invert Hsync polarity.
                                 (VSYNC_POLARITY                    << 3) | // [    3] inv_vsync. 1=Invert Vsync polarity.
                                 (0                                 << 4) | // [    4] inv_dvi_clk. 1=Invert clock to external DVI, (clock invertion exists at internal HDMI).
                                 (((1==0)?1:0)  << 5) | // [ 7: 5] data_comp_map. Input data is CrYCb(BRG), map the output data to desired format:
                                                                            //                          0=output CrYCb(BRG);
                                                                            //                          1=output YCbCr(RGB);
                                                                            //                          2=output YCrCb(RBG);
                                                                            //                          3=output CbCrY(GBR);
                                                                            //                          4=output CbYCr(GRB);
                                                                            //                          5=output CrCbY(BGR);
                                                                            //                          6,7=Rsrv.
                                 (0                                 << 8) | // [11: 8] wr_rate. 0=A write every clk1; 1=A write every 2 clk1; ...; 15=A write every 16 clk1.
                                 (0                                 <<12)   // [15:12] rd_rate. 0=A read every clk2; 1=A read every 2 clk2; ...; 15=A read every 16 clk2.
            );
    }
    // Annie 01Sep2011: Register VENC_DVI_SETTING and VENC_DVI_SETTING_MORE are no long valid, use VPU_HDMI_SETTING instead.
    set_reg32_bits_op(P_VPU_HDMI_SETTING, 1, 1, 1);  // [    1] src_sel_encp: Enable ENCP output to HDMI
}    

static void hdmi_tx_misc(HDMI_Video_Codes_t vic)
{
    unsigned int tmp_add_data;
    unsigned int checksum, i;
    
    // Enable APB3 fail on error
    set_reg32_bits_op(P_HDMI_CTRL_PORT, 1, 15, 1);

    // Disable these interrupts: [2] tx_edid_int_rise [1] tx_hpd_int_fall [0] tx_hpd_int_rise
    hdmi_wr_reg(OTHER_BASE_ADDR + HDMI_OTHER_INTR_MASKN, 0x0);

    // HPD glitch filter
    hdmi_wr_reg(TX_HDCP_HPD_FILTER_L, 0xa0);
    hdmi_wr_reg(TX_HDCP_HPD_FILTER_H, 0xa0);

    // Disable MEM power-down
    hdmi_wr_reg(TX_MEM_PD_REG0, 0);

    // Keep TX (except register I/F) in reset, while programming the registers:
    tmp_add_data  = 0;
    tmp_add_data |= 1   << 7; // [7] tx_pixel_rstn
    tmp_add_data |= 1   << 6; // [6] tx_tmds_rstn
    tmp_add_data |= 1   << 5; // [5] tx_audio_master_rstn
    tmp_add_data |= 1   << 4; // [4] tx_audio_sample_rstn
    tmp_add_data |= 1   << 3; // [3] tx_i2s_reset_rstn
    tmp_add_data |= 1   << 2; // [2] tx_dig_reset_n_ch2
    tmp_add_data |= 1   << 1; // [1] tx_dig_reset_n_ch1
    tmp_add_data |= 1   << 0; // [0] tx_dig_reset_n_ch0
    hdmi_wr_reg(TX_SYS5_TX_SOFT_RESET_1, tmp_add_data);

    tmp_add_data  = 0;
    tmp_add_data |= 0   << 4; // [4] HDMI_CH0_RST_IN
    tmp_add_data |= 1   << 2; // [0] tx_ddc_hdcp_reset_n
    tmp_add_data |= 1   << 1; // [0] tx_ddc_edid_reset_n
    tmp_add_data |= 1   << 0; // [0] tx_dig_reset_n_ch3
    hdmi_wr_reg(TX_SYS5_TX_SOFT_RESET_2, tmp_add_data);

    tmp_add_data  = 0;
    tmp_add_data |= 0   << 7; // [7] forced_sys_trigger
    tmp_add_data |= 0   << 6; // [6] sys_trigger_config
    tmp_add_data |= 0   << 5; // [5] mem_acc_seq_mode
    tmp_add_data |= 0   << 4; // [4] mem_acc_seq_start
    tmp_add_data |= 1   << 3; // [3] forced_mem_copy_done
    tmp_add_data |= 1   << 2; // [2] mem_copy_done_config
    tmp_add_data |= 0   << 1; // [1] edid_int_forced_clear
    tmp_add_data |= 0   << 0; // [0] edid_int_auto_clear
    hdmi_wr_reg(TX_HDCP_EDID_CONFIG, tmp_add_data);

    tmp_add_data  = 0;
    tmp_add_data |= 0               << 7; // [7]   Force DTV timing (Auto)
    tmp_add_data |= 0               << 6; // [6]   Force Video Scan, only if [7]is set
    tmp_add_data |= 0               << 5; // [5]   Force Video field, only if [7]is set
    tmp_add_data |= ((vic==39)?0:1) << 4; // [4]   disable_vic39_correction
    tmp_add_data |= 0               << 0; // [3:0] Rsrv
    hdmi_wr_reg(TX_VIDEO_DTV_TIMING, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 0                       << 7; // [7]   forced_default_phase
    tmp_add_data |= 0                       << 2; // [6:2] Rsrv
    tmp_add_data |= 0   << 0; // [1:0] Color_depth:0=24-bit pixel; 1=30-bit pixel; 2=36-bit pixel; 3=48-bit pixel
    hdmi_wr_reg(TX_VIDEO_DTV_MODE, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 0                       << 7; // [7]   gc_pack_mode: 0=clear color_depth and pixel_phase when GC packet is transmitting AV_mute/clear info;
                                                  //                     1=do not clear.
    tmp_add_data |= 0                       << 0; // [6:0] forced_islands_per_period_active
    hdmi_wr_reg(TX_PACKET_ALLOC_ACTIVE_1, tmp_add_data);

    tmp_add_data  = 0;
    tmp_add_data |= 0   << 7; // [7]   Force packet timing
    tmp_add_data |= 0   << 6; // [6]   PACKET ALLOC MODE
    tmp_add_data |= 58  << 0; // [5:0] PACKET_START_LATENCY
    hdmi_wr_reg(TX_PACKET_CONTROL_1, tmp_add_data);
    
    hdmi_wr_reg(TX_PACKET_CONTROL_2, 0x2);      //deep_color_request_enable disable
    tmp_add_data  = 0;
    tmp_add_data |= 0   << 6; // [7:6] audio_source_select[1:0]
    tmp_add_data |= 0   << 5; // [5]   external_packet_enable
    tmp_add_data |= 1   << 4; // [4]   internal_packet_enable
    tmp_add_data |= 0   << 2; // [3:2] afe_fifo_source_select_lane_1[1:0]
    tmp_add_data |= 0   << 0; // [1:0] afe_fifo_source_select_lane_0[1:0]
    hdmi_wr_reg(TX_CORE_DATA_CAPTURE_2, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 0   << 7; // [7]   monitor_lane_1
    tmp_add_data |= 0   << 4; // [6:4] monitor_select_lane_1[2:0]
    tmp_add_data |= 1   << 3; // [3]   monitor_lane_0
    tmp_add_data |= 7   << 0; // [2:0] monitor_select_lane_0[2:0]
    hdmi_wr_reg(TX_CORE_DATA_MONITOR_1, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 0   << 3; // [7:3] Rsrv
    tmp_add_data |= 2   << 0; // [2:0] monitor_select[2:0]
    hdmi_wr_reg(TX_CORE_DATA_MONITOR_2, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 1   << 7; // [7]   forced_hdmi
    tmp_add_data |= 1   << 6; // [6]   hdmi_config
    tmp_add_data |= 0   << 4; // [5:4] Rsrv
    tmp_add_data |= 0   << 3; // [3]   bit_swap.
    tmp_add_data |= 0   << 0; // [2:0] channel_swap[2:0]
    hdmi_wr_reg(TX_TMDS_MODE, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 0   << 7; // [7]   Rsrv
    tmp_add_data |= 0   << 6; // [6]   TX_CONNECT_SEL: 0=use lower channel data[29:0]; 1=use upper channel data[59:30]
    tmp_add_data |= 0   << 0; // [5:0] Rsrv
    hdmi_wr_reg(TX_SYS4_CONNECT_SEL_1, tmp_add_data);
    
    // Normally it makes sense to synch 3 channel output with clock channel's rising edge,
    // as HDMI's serializer is LSB out first, invert tmds_clk pattern from "1111100000" to
    // "0000011111" actually enable data synch with clock rising edge.
    tmp_add_data = 1 << 4; // Set tmds_clk pattern to be "0000011111" before being sent to AFE clock channel
    hdmi_wr_reg(TX_SYS4_CK_INV_VIDEO, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 0   << 7; // [7] Rsrv
    tmp_add_data |= 0   << 6; // [6] TX_AFE_FIFO channel 2 bypass=0
    tmp_add_data |= 0   << 5; // [5] TX_AFE_FIFO channel 1 bypass=0
    tmp_add_data |= 0   << 4; // [4] TX_AFE_FIFO channel 0 bypass=0
    tmp_add_data |= 1   << 3; // [3] output enable of clk channel (channel 3)
    tmp_add_data |= 1   << 2; // [2] TX_AFE_FIFO channel 2 enable
    tmp_add_data |= 1   << 1; // [1] TX_AFE_FIFO channel 1 enable
    tmp_add_data |= 1   << 0; // [0] TX_AFE_FIFO channel 0 enable
    hdmi_wr_reg(TX_SYS5_FIFO_CONFIG, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 1  << 6; // [7:6] output_color_format: 0=RGB444; 1=YCbCr444; 2=Rsrv; 3=YCbCr422.
    tmp_add_data |= 1   << 4; // [5:4] input_color_format:  0=RGB444; 1=YCbCr444; 2=Rsrv; 3=YCbCr422.
    tmp_add_data |= 0   << 2; // [3:2] output_color_depth:  0=24-b; 1=30-b; 2=36-b; 3=48-b.
    tmp_add_data |= 0    << 0; // [1:0] input_color_depth:   0=24-b; 1=30-b; 2=36-b; 3=48-b.
    hdmi_wr_reg(TX_VIDEO_DTV_OPTION_L, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 0                       << 4; // [7:4] Rsrv
    tmp_add_data |= 0   << 2; // [3:2] output_color_range:  0=16-235/240; 1=16-240; 2=1-254; 3=0-255.
    tmp_add_data |= 0    << 0; // [1:0] input_color_range:   0=16-235/240; 1=16-240; 2=1-254; 3=0-255.
    hdmi_wr_reg(TX_VIDEO_DTV_OPTION_H, tmp_add_data);

    tmp_add_data  = 0;
    tmp_add_data |= 0   << 7; // [7] cp_desired
    tmp_add_data |= 0   << 6; // [6] ess_config
    tmp_add_data |= 0   << 5; // [5] set_avmute
    tmp_add_data |= 1   << 4; // [4] clear_avmute
    tmp_add_data |= 0   << 3; // [3] hdcp_1_1
    tmp_add_data |= 0   << 2; // [2] Vsync/Hsync forced_polarity_select
    tmp_add_data |= 0   << 1; // [1] forced_vsync_polarity
    tmp_add_data |= 0   << 0; // [0] forced_hsync_polarity
    hdmi_wr_reg(TX_HDCP_MODE, tmp_add_data);

    // Disable ALL packet generation
    for(i = 0; i < 16; i++) {
        hdmi_wr_reg(TX_PKT_REG_SPD_INFO_BASE_ADDR + i * 32 + 0x1F, 0);
    }

    // AVI frame
    //hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x00, 0x46);              // PB0: Checksum
    hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x01, 0x5e);              // PB1 (Note: the value should be meaningful but is not!)
    switch(vic) {
    case HDMI_640x480p60:
    case HDMI_480p60:
    case HDMI_480i60:
    case HDMI_1440x480p60:
    case HDMI_576p50:
    case HDMI_576i50:
        hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x02, 0x58);              // PB2 (Note: the value should be meaningful but is not!)
        break;
    case HDMI_480p60_16x9:
    case HDMI_480i60_16x9:
    case HDMI_576p50_16x9:
    case HDMI_576i50_16x9:
        hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x02, 0x68);
        break;
    default:
        hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x02, 0xa8);              // PB2 (Note: the value should be meaningful but is not!)
        break;
    }
    switch(vic) {
    case HDMI_480p60:
    case HDMI_480i60:
    case HDMI_576p50:
    case HDMI_576i50:
    case HDMI_480p60_16x9:
    case HDMI_480i60_16x9:
    case HDMI_576p50_16x9:
    case HDMI_576i50_16x9:
        hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x03, 0x03);              // PB3 (Note: the value should be meaningful but is not!)
        break;
    default:
        hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x03, 0x13);              // PB3 (Note: the value should be meaningful but is not!)
        break;
    }
    hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x04, vic);               // PB4: [7]    Rsrv
    if((vic >= HDMI_4k2k_30) && (vic <= HDMI_4k2k_smpte)) {
        hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x04, 0);             // if mode is 4k, then set vic = 0 and set to VSDB
    }
    switch(vic) {
    case HDMI_480i60:
    case HDMI_576i50:
    case HDMI_480i60_16x9:
    case HDMI_576i50_16x9:
        hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x05, 1); // PB5: [7:4]  Rsrv     [3:0]  PixelRepeat
        break;
    default:
        hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x05, 0); // PB5: [7:4]  Rsrv     [3:0]  PixelRepeat
        break;
    }
    hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x1C, 0x82);              // HB0: packet type=0x82
    hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x1D, 0x02);              // HB1: packet version =0x02
    hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x1E, 0x0D);              // HB2: payload bytes=13
    
    // calculate Checksum
    checksum = 0;
    for(i = 1; i < 0x1f; i++) {
        checksum += hdmi_rd_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR + i);
    }

    checksum = ((~checksum) & 0xff) + 1;
    hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x00, checksum);              // PB0: Checksum

    hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x1F, 0xFF);              // Enable AVI packet generation

    tmp_add_data = 0x18 - 1; // time_divider[7:0] for DDC I2C bus clock
    hdmi_wr_reg(TX_HDCP_CONFIG3, tmp_add_data);
    hdmi_wr_reg(TX_HDCP_CONFIG0, 0x3<<3);
    
    hdmi_wr_reg(TX_HDCP_MODE, 0x40);
    
    // --------------------------------------------------------
    // Release TX out of reset
    // --------------------------------------------------------
    MmioWrite32(P_HHI_HDMI_PLL_CNTL1, 0x00040000);         // turn off phy_clk
    MmioWrite32(P_HHI_HDMI_PLL_CNTL1, 0x00040003);         // turn on phy_clk
    hdmi_wr_reg(TX_SYS5_TX_SOFT_RESET_2, 0x00); // Release reset on TX digital clock channel
    hdmi_wr_reg(TX_SYS5_TX_SOFT_RESET_1, 1<<6); // Release resets all other TX digital clock domain, except tmds_clk
    hdmi_wr_reg(TX_SYS5_TX_SOFT_RESET_1, 0x00); // Final release reset on tmds_clk domain
}


typedef struct {
  VENDOR_DEVICE_PATH DisplayDevicePath;
  EFI_DEVICE_PATH EndDevicePath;
} DISPLAY_DEVICE_PATH;

typedef struct {
  UINT32 Width;
  UINT32 Height;
} GOP_MODE_DATA;

STATIC EFI_HANDLE mDevice;
STATIC EFI_CPU_ARCH_PROTOCOL *mCpu;

// Enable nessary hdmi related clock gate & power control
static void hdmi_tx_gate(HDMI_Video_Codes_t vic)
{
    set_reg32_bits_op(P_HHI_VPU_MEM_PD_REG1, 0x0, 20, 2);
// Powerup VPU_HDMI
    MmioWrite32(P_AO_RTI_GEN_PWR_SLEEP0, MmioRead32(P_AO_RTI_GEN_PWR_SLEEP0) & (~(0x1<<8))); // [8] power on
    MmioWrite32(P_HHI_MEM_PD_REG0, MmioRead32(P_HHI_MEM_PD_REG0) & (~(0xff << 8))); // HDMI MEM-PD

    // Remove VPU_HDMI ISO
    MmioWrite32(P_AO_RTI_GEN_PWR_SLEEP0, MmioRead32(P_AO_RTI_GEN_PWR_SLEEP0) & (~(0x1<<9))); // [9] VPU_HDMI

    set_reg32_bits_op(P_HHI_GCLK_MPEG2, 1, 4, 1); //enable HDMI PCLK
    set_reg32_bits_op(P_HHI_GCLK_MPEG2, 1, 3, 1); //enable HDMI Int Sync
    MmioWrite32(P_HHI_HDMI_CLK_CNTL,  ((0 << 9)  |   // select XTAL
                             (1 << 8)  |   // Enable gated clock
                             (0 << 0)) );  // Divide by 1
    
    if((vic == HDMI_480i60) || (vic == HDMI_576i50) || (vic == HDMI_480i60_16x9) || (vic == HDMI_576i50_16x9)) {
        // For ENCI
        set_reg32_bits_op(P_HHI_GCLK_OTHER, 1, 8, 1); //enable VCLK2_ENCI
        set_reg32_bits_op(P_HHI_GCLK_OTHER, 1, 2, 1); //enable VCLK2_VENCI
    }
    else {
        // For ENCP
        set_reg32_bits_op(P_HHI_GCLK_OTHER, 1, 4, 1); //enable VCLK2_VENCP
        set_reg32_bits_op(P_HHI_GCLK_OTHER, 1, 9, 1); //enable VCLK2_ENC
    }
    
    set_reg32_bits_op(P_PERIPHS_PIN_MUX_1, 0x7, 24, 3);  //HPD SCL pinmux
}

vmode_t vic_to_vmode(HDMI_Video_Codes_t vic)
{
    vmode_t vmode = VMODE_INIT_NULL;
    int i = 0;
    while(tvmode_vmode_vic_map[i][2] != HDMI_Unkown) {
        if(tvmode_vmode_vic_map[i][2] == vic) {
            vmode = tvmode_vmode_vic_map[i][1];
            break;
        }
        i ++;
    }
    return vmode;
}

static void hdmi_tx_clk(HDMI_Video_Codes_t vic)
{
	extern void set_vmode_clk(vmode_t mode);
    set_vmode_clk(vic_to_vmode(vic));
}

static void hdmi_tx_enc(HDMI_Video_Codes_t vic)
{
    switch(vic){
        case HDMI_480i60:
        case HDMI_480i60_16x9:
        case HDMI_576i50:
        case HDMI_576i50_16x9:
            hdmi_tvenc480i_set(vic);
            break;
        case HDMI_1080i60:
        case HDMI_1080i50:
            hdmi_tvenc1080i_set(vic);
            break;
        case HDMI_4k2k_30:
        case HDMI_4k2k_25:
        case HDMI_4k2k_24:
        case HDMI_4k2k_smpte:
            hdmi_tvenc4k2k_set(vic);
            break;
        default:
            hdmi_tvenc_set(vic);
        }
}

static void hdmi_tx_phy(HDMI_Video_Codes_t vic)
{
    switch(vic) {
    case HDMI_4k2k_30:
    case HDMI_4k2k_25:
    case HDMI_4k2k_24:
    case HDMI_4k2k_smpte:
        MmioWrite32(P_HHI_HDMI_PHY_CNTL0, 0x08c34d0b);
        break;
    default:
        MmioWrite32(P_HHI_HDMI_PHY_CNTL0, 0x08c31e8b);
    }
    MmioWrite32(P_HHI_HDMI_PHY_CNTL1, 0);
    MmioWrite32(P_HHI_HDMI_PHY_CNTL1, 1);       // Soft Reset HDMI PHY
    MicroSecondDelay(1000);
    MmioWrite32(P_HHI_HDMI_PHY_CNTL1, 0);
    MicroSecondDelay(1000);
    MmioWrite32(P_HHI_HDMI_PHY_CNTL1, 2);       // Enable HDMI PHY
}

static void hdmitx_set_packet(int type, unsigned char* DB, unsigned char* HB)
{
    // AVI frame
    int i ;
    unsigned char ucData ;
    unsigned int pkt_reg_base=TX_PKT_REG_AVI_INFO_BASE_ADDR;
    int pkt_data_len=0;
    
    switch(type)
    {
        case HDMI_PACKET_AVI:
            pkt_reg_base=TX_PKT_REG_AVI_INFO_BASE_ADDR; 
            pkt_data_len=13;
            break;
        case HDMI_PACKET_VEND:
            pkt_reg_base=TX_PKT_REG_VEND_INFO_BASE_ADDR;
            pkt_data_len=6;
            break;
        case HDMI_AUDIO_INFO:
            pkt_reg_base=TX_PKT_REG_AUDIO_INFO_BASE_ADDR;
            pkt_data_len=9;
            break;
        case HDMI_SOURCE_DESCRIPTION:
            pkt_reg_base=TX_PKT_REG_SPD_INFO_BASE_ADDR;
            pkt_data_len=25;
        default:
            break;
    }
    
    if(DB){
        for(i=0;i<pkt_data_len;i++){
            hdmi_wr_reg(pkt_reg_base+i+1, DB[i]);  
        }
    
        for(i = 0,ucData = 0; i < pkt_data_len ; i++)
        {
            ucData -= DB[i] ;
        }
        for(i=0; i<3; i++){
            ucData -= HB[i];
        }
        hdmi_wr_reg(pkt_reg_base+0x00, ucData);  
    
        hdmi_wr_reg(pkt_reg_base+0x1C, HB[0]);        
        hdmi_wr_reg(pkt_reg_base+0x1D, HB[1]);        
        hdmi_wr_reg(pkt_reg_base+0x1E, HB[2]);        
        hdmi_wr_reg(pkt_reg_base+0x1F, 0x00ff);        // Enable packet generation
    }
    else{
        hdmi_wr_reg(pkt_reg_base+0x1F, 0x0);        // disable packet generation
    }
}

static void hdmi_tx_set_vend_spec_infofram(HDMI_Video_Codes_t vic)
{
    int i;
    unsigned char VEN_DB[6];
    unsigned char VEN_HB[3];
    VEN_HB[0] = 0x81; 
    VEN_HB[1] = 0x01; 
    VEN_HB[2] = 0x6; 

    if(!((vic >= HDMI_4k2k_30) && (vic <= HDMI_4k2k_smpte)))
        return;

    for(i = 0; i < 0x6; i++){
        VEN_DB[i] = 0;
    }
    VEN_DB[0] = 0x03;
    VEN_DB[1] = 0x0c;
    VEN_DB[2] = 0x00;

    VEN_DB[3] = 0x20;         // 4k x 2k  Spec P156
    if(vic == HDMI_4k2k_30)
        VEN_DB[4] = 0x1;
    else if(vic == HDMI_4k2k_25)
        VEN_DB[4] = 0x2;
    else if(vic == HDMI_4k2k_24)
        VEN_DB[4] = 0x3;
    else if(vic == HDMI_4k2k_smpte)
        VEN_DB[4] = 0x4;
    else {
        // nothing
    }
    hdmitx_set_packet(HDMI_PACKET_VEND, VEN_DB, VEN_HB);
}

// When have below format output, we shall manually configure
// bolow register to get stable Video Timing.
static void hdmi_reconfig_packet_setting(HDMI_Video_Codes_t vic)
{
    switch(vic) {
    case HDMI_1080p50:
        hdmi_wr_reg(TX_PACKET_CONTROL_1, 0x3a);         //0x7e
        hdmi_wr_reg(TX_PACKET_ALLOC_ACTIVE_1, 0x01);    //0x78
        hdmi_wr_reg(TX_PACKET_ALLOC_ACTIVE_2, 0x12);    //0x79
        hdmi_wr_reg(TX_PACKET_ALLOC_EOF_1, 0x10);       //0x7a
        hdmi_wr_reg(TX_PACKET_ALLOC_EOF_2, 0x12);       //0x7b
        hdmi_wr_reg(TX_CORE_ALLOC_VSYNC_0, 0x01);       //0x81
        hdmi_wr_reg(TX_CORE_ALLOC_VSYNC_1, 0x00);       //0x82
        hdmi_wr_reg(TX_CORE_ALLOC_VSYNC_2, 0x0a);       //0x83
        hdmi_wr_reg(TX_PACKET_ALLOC_SOF_1, 0xb6);       //0x7c
        hdmi_wr_reg(TX_PACKET_ALLOC_SOF_2, 0x11);       //0x7d
        hdmi_wr_reg(TX_PACKET_CONTROL_1, 0xba);         //0x7e
        break;
    default:
        break;
    }
    DEBUG((EFI_D_INFO,"reconfig packet setting done\n"));
}

void hdmi_tx_set(HDMI_Video_Codes_t vic) 
{
	// reset SD 4:3 to 16:9 formats
	if((vic == HDMI_480p60) || (vic == HDMI_480i60) || (vic == HDMI_576p50) || (vic == HDMI_576i50))
		vic ++;
	DEBUG((EFI_D_INFO,"set HDMI vic: %d\n", vic));

    if((vic >= HDMI_4k2k_30) && (vic <= HDMI_4k2k_smpte)) {
        return;
    }
    hdmi_tx_gate(vic);
    hdmi_tx_clk(vic);
    hdmi_tx_misc(vic);
    hdmi_tx_enc(vic);
    hdmi_tx_set_vend_spec_infofram(vic);
    hdmi_reconfig_packet_setting(vic);
    hdmi_tx_phy(vic);
}

HDMI_Video_Codes_t tvmode_to_vic(int mode)
{
    HDMI_Video_Codes_t vic = HDMI_Unkown;
    int i = 0;
    while(tvmode_vmode_vic_map[i][0] != TVOUT_MAX) {
        if(tvmode_vmode_vic_map[i][0] == mode) {
            vic = tvmode_vmode_vic_map[i][2];
            break;
        }
        i ++;
    }
    return vic;
}


STATIC DISPLAY_DEVICE_PATH mDisplayProtoDevicePath =
  {
    {
      {
        HARDWARE_DEVICE_PATH,
        HW_VENDOR_DP,
        {
          (UINT8)(sizeof(VENDOR_DEVICE_PATH)),
          (UINT8)((sizeof(VENDOR_DEVICE_PATH)) >> 8),
        }
      },
      EFI_CALLER_ID_GUID,
    },
    {
      END_DEVICE_PATH_TYPE,
      END_ENTIRE_DEVICE_PATH_SUBTYPE,
      {
        sizeof(EFI_DEVICE_PATH_PROTOCOL),
        0
      }
    }
  };

#define PI2_BITS_PER_PIXEL              (32)
#define PI2_BYTES_PER_PIXEL             (PI2_BITS_PER_PIXEL / 8)

EFI_GRAPHICS_OUTPUT_PROTOCOL gDisplayProto = {
  DisplayQueryMode,
  DisplaySetMode,
  DisplayBlt,
  NULL
};

/**
  Returns information for an available graphics mode that the graphics device
  and the set of active video output devices supports.

  @param  This                  The EFI_GRAPHICS_OUTPUT_PROTOCOL instance.
  @param  ModeNumber            The mode number to return information on.
  @param  SizeOfInfo            A pointer to the size, in bytes, of the Info buffer.
  @param  Info                  A pointer to callee allocated buffer that returns information about ModeNumber.

  @retval EFI_SUCCESS           Valid mode information was returned.
  @retval EFI_DEVICE_ERROR      A hardware error occurred trying to retrieve the video mode.
  @retval EFI_INVALID_PARAMETER ModeNumber is not valid.

**/
STATIC
EFI_STATUS
EFIAPI
DisplayQueryMode(
                 IN  EFI_GRAPHICS_OUTPUT_PROTOCOL          *This,
                 IN  UINT32                                ModeNumber,
                 OUT UINTN                                 *SizeOfInfo,
                 OUT EFI_GRAPHICS_OUTPUT_MODE_INFORMATION  **Info
                 )
{
  EFI_STATUS Status;

  if (ModeNumber > TVOUT_MAX) {
    return EFI_INVALID_PARAMETER;
  }

  Status = gBS->AllocatePool(
                             EfiBootServicesData,
                             sizeof(EFI_GRAPHICS_OUTPUT_MODE_INFORMATION),
                             (VOID **)Info
                             );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  
  //TODO
  //Mode = &mGopModeData[ModeNumber];

  *SizeOfInfo                   = sizeof(EFI_GRAPHICS_OUTPUT_MODE_INFORMATION);
  (*Info)->Version              = This->Mode->Info->Version;
  (*Info)->HorizontalResolution = This->Mode->Info->HorizontalResolution;
  (*Info)->VerticalResolution   = This->Mode->Info->VerticalResolution;
  (*Info)->PixelFormat          = This->Mode->Info->PixelFormat;
  (*Info)->PixelInformation     = This->Mode->Info->PixelInformation;
  (*Info)->PixelsPerScanLine    = This->Mode->Info->PixelsPerScanLine;

  return EFI_SUCCESS;
}

STATIC
VOID
ClearScreen(
  IN  EFI_GRAPHICS_OUTPUT_PROTOCOL *This
  )
{
  EFI_GRAPHICS_OUTPUT_BLT_PIXEL Fill;

  Fill.Red                      = 0x00;
  Fill.Green                    = 0x00;
  Fill.Blue                     = 0x00;
  This->Blt (This, &Fill, EfiBltVideoFill,
             0, 0, 0, 0, This->Mode->Info->HorizontalResolution,
             This->Mode->Info->VerticalResolution,
             This->Mode->Info->HorizontalResolution *
             sizeof (EFI_GRAPHICS_OUTPUT_BLT_PIXEL));
}

/**
  Set the video device into the specified mode and clears the visible portions of 
  the output display to black.

  @param  This              The EFI_GRAPHICS_OUTPUT_PROTOCOL instance.
  @param  ModeNumber        Abstraction that defines the current video mode.

  @retval EFI_SUCCESS       The graphics mode specified by ModeNumber was selected.
  @retval EFI_DEVICE_ERROR  The device had an error and could not complete the request.
  @retval EFI_UNSUPPORTED   ModeNumber is not supported by this device.

**/
STATIC
EFI_STATUS
EFIAPI
DisplaySetMode(
  IN  EFI_GRAPHICS_OUTPUT_PROTOCOL *This,
  IN  UINT32                       ModeNumber
  )
{
    HDMI_Video_Codes_t vic;
	
    if(ModeNumber >= TVOUT_MAX) {
        DEBUG((EFI_D_INFO,"Invalid hdmi mode %d\n", ModeNumber));
        return EFI_DEVICE_ERROR;
    }
	DEBUG((EFI_D_INFO,"hdmi mode %d\n", ModeNumber));
	MmioOr32(CBUS_REG_ADDR(VENC_VDAC_SETTING), 0x1f);
	WRITE_CBUS_REG(HHI_VDAC_CNTL0, 0); 
	WRITE_CBUS_REG(HHI_VDAC_CNTL1, 8);
	tv_out_reg(ModeNumber);
    vic = tvmode_to_vic(ModeNumber);
    DEBUG((EFI_D_INFO,"mode = %d  vic = %d\n", ModeNumber, vic));
    hdmi_tx_set(vic);
  return EFI_SUCCESS;
}

/**
  Blt a rectangle of pixels on the graphics screen. Blt stands for BLock Transfer.
  
  @param  This         Protocol instance pointer.
  @param  BltBuffer    The data to transfer to the graphics screen.
                       Size is at least Width*Height*sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL).
  @param  BltOperation The operation to perform when copying BltBuffer on to the graphics screen.
  @param  SourceX      The X coordinate of source for the BltOperation.
  @param  SourceY      The Y coordinate of source for the BltOperation.
  @param  DestinationX The X coordinate of destination for the BltOperation.
  @param  DestinationY The Y coordinate of destination for the BltOperation.
  @param  Width        The width of a rectangle in the blt rectangle in pixels.
  @param  Height       The height of a rectangle in the blt rectangle in pixels.
  @param  Delta        Not used for EfiBltVideoFill or the EfiBltVideoToVideo operation.
                       If a Delta of zero is used, the entire BltBuffer is being operated on.
                       If a subrectangle of the BltBuffer is being used then Delta
                       represents the number of bytes in a row of the BltBuffer.

  @retval EFI_SUCCESS           BltBuffer was drawn to the graphics screen.
  @retval EFI_INVALID_PARAMETER BltOperation is not valid.
  @retval EFI_DEVICE_ERROR      The device had an error and could not complete the request.

**/
STATIC
EFI_STATUS
EFIAPI
DisplayBlt(
           IN  EFI_GRAPHICS_OUTPUT_PROTOCOL      *This,
           IN  EFI_GRAPHICS_OUTPUT_BLT_PIXEL     *BltBuffer,   OPTIONAL
           IN  EFI_GRAPHICS_OUTPUT_BLT_OPERATION BltOperation,
           IN  UINTN                             SourceX,
           IN  UINTN                             SourceY,
           IN  UINTN                             DestinationX,
           IN  UINTN                             DestinationY,
           IN  UINTN                             Width,
           IN  UINTN                             Height,
           IN  UINTN                             Delta         OPTIONAL
           )
{
  RETURN_STATUS Status;
  EFI_TPL       Tpl;
  //
  // We have to raise to TPL_NOTIFY, so we make an atomic write to the frame
  // buffer. We would not want a timer based event (Cursor, ...) to come in
  // while we are doing this operation.
  //
  Tpl    = gBS->RaiseTPL(TPL_NOTIFY); 
  Status = FrameBufferBlt(
      mFrameBufferBltLibConfigure, BltBuffer, BltOperation, SourceX, SourceY,
      DestinationX, DestinationY, Width, Height, Delta);
  gBS->RestoreTPL(Tpl);

  return RETURN_ERROR(Status) ? EFI_INVALID_PARAMETER : EFI_SUCCESS;
}


#define OSD2_CANVAS_INDEX 0x43
STATIC
VOID
osd2_update_color_mode()
{
	UINT32  data32=0;
	data32 |=MmioRead32(P_VIU_OSD2_BLK0_CFG_W0)&0x7040;
	data32 |= OSD2_CANVAS_INDEX << 16 ;
	data32 |= OSD_DATA_LITTLE_ENDIAN	 <<15 ;
    	//data32 |= osd_hw.color_info[OSD2]->hw_colormat<< 2;	default_color_format_array[16]
	data32 |= 3 << 2;
	
	data32 |= 1<< 7; /* rgb enable */
	data32 |=  5<< 8; /* osd_blk_mode */
    DEBUG((EFI_D_INFO,"---VIU_OSD2_BLK0_CFG_W0 = %d(0x%x)\n", data32, data32));
	MmioWrite32(P_VIU_OSD2_BLK0_CFG_W0,data32);
	//remove_from_update_list(OSD2, OSD_COLOR_MODE);
}

void canvas_config(UINT32 index, UINTN addr, UINT32 width,
				  UINT32 height, UINT32 wrap, UINT32 blkmode)
{

   DEBUG((EFI_D_INFO,"index=%d, addr=0x%x width=%d, height=%d wrap=%d, blkmode=%d \n",
        index, addr, width, height, wrap, blkmode));

	// MmioWrite32(M8M2_P_DC_CAV_LUT_DATAL,(((addr + 7) >> 3) & CANVAS_ADDR_LMASK) |
    					// ((((width + 7) >> 3) & CANVAS_WIDTH_LMASK) << CANVAS_WIDTH_LBIT)
    					// );
        // MmioWrite32(M8M2_P_DC_CAV_LUT_DATAH,((((width + 7) >> 3) >> CANVAS_WIDTH_LWID) << CANVAS_WIDTH_HBIT) |
    					// ((height & CANVAS_HEIGHT_MASK) << CANVAS_HEIGHT_BIT)	|
    					// ((wrap & CANVAS_XWRAP) ? CANVAS_XWRAP : 0)              |
    					// ((wrap & CANVAS_YWRAP) ? CANVAS_YWRAP : 0)              |
    					// ((blkmode & CANVAS_BLKMODE_MASK) << CANVAS_BLKMODE_BIT)
    					// );
        // MmioWrite32( M8M2_P_DC_CAV_LUT_ADDR,CANVAS_LUT_WR_EN | index);
    	// // read a cbus to make sure last write finish.
        // MmioRead32(M8M2_P_DC_CAV_LUT_DATAH);

        MmioWrite32( P_DC_CAV_LUT_DATAL,(((addr + 7) >> 3) & CANVAS_ADDR_LMASK) |
                        ((((width + 7) >> 3) & CANVAS_WIDTH_LMASK) << CANVAS_WIDTH_LBIT));
        MmioWrite32(P_DC_CAV_LUT_DATAH,((((width + 7) >> 3) >> CANVAS_WIDTH_LWID) << CANVAS_WIDTH_HBIT) |
                        ((height & CANVAS_HEIGHT_MASK) << CANVAS_HEIGHT_BIT)    |
                        ((wrap & CANVAS_XWRAP) ? CANVAS_XWRAP : 0)              |
                        ((wrap & CANVAS_YWRAP) ? CANVAS_YWRAP : 0)              |
                        ((blkmode & CANVAS_BLKMODE_MASK) << CANVAS_BLKMODE_BIT));
        MmioWrite32(P_DC_CAV_LUT_ADDR,CANVAS_LUT_WR_EN | index);
        // read a cbus to make sure last write finish.
        MmioRead32(P_DC_CAV_LUT_DATAH);


}
STATIC
VOID
osd2_update_enable(void)
{
	
	UINT32 video_enable=0;
    //DEBUG((EFI_D_INFO,"---osd_hw.enable[OSD2] = %d(0x%x)\n", osd_hw.enable[OSD2], osd_hw.enable[OSD2]));
	video_enable |= MmioRead32(P_VPP_MISC)&VPP_VD1_PREBLEND;

	clrbits_le32(P_VPP_MISC,VPP_OSD2_PREBLEND);
	if(!video_enable)
	{
		clrbits_le32(P_VPP_MISC,VPP_VD1_POSTBLEND);
	}
	setbits_le32(P_VPP_MISC,VPP_OSD2_POSTBLEND);

	
	//remove_from_update_list(OSD2,OSD_ENABLE);
}


void osd_setup(UINT32 xoffset,
                UINT32 yoffset,
                UINT32 xres,
                UINT32 yres,
                UINT32 xres_virtual,
                UINT32 yres_virtual,
                UINT32 disp_start_x,
                UINT32 disp_start_y,
                UINT32 disp_end_x,
                UINT32 disp_end_y,
                UINT32 fbmem,
                const color_bit_define_t *color,
                int index 
                )
{
    DEBUG((EFI_D_INFO,"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"));
	DEBUG((EFI_D_INFO,"xoffset = %d(0x%x)\n", xoffset, xoffset));
	DEBUG((EFI_D_INFO,"yoffset = %d(0x%x)\n", yoffset, yoffset));
	DEBUG((EFI_D_INFO,"xres = %d(0x%x)\n", xres, xres));
	DEBUG((EFI_D_INFO,"yres = %d(0x%x)\n", yres, yres));
	DEBUG((EFI_D_INFO,"xres_virtual = %d(0x%x)\n", xres_virtual, xres_virtual));
	DEBUG((EFI_D_INFO,"yres_virtual = %d(0x%x)\n", yres_virtual, yres_virtual));
	DEBUG((EFI_D_INFO,"disp_start_x = %d(0x%x)\n", disp_start_x, disp_start_x));
	DEBUG((EFI_D_INFO,"disp_start_y = %d(0x%x)\n", disp_start_y, disp_start_y));
	DEBUG((EFI_D_INFO,"disp_end_x = %d(0x%x)\n", disp_end_x, disp_end_x));
	DEBUG((EFI_D_INFO,"disp_end_y = %d(0x%x)\n", disp_end_y, disp_end_y));
	DEBUG((EFI_D_INFO,"fbmem = %d(0x%x)\n", fbmem, fbmem));
	DEBUG((EFI_D_INFO,"index = %d(0x%x)\n", index, index));
	DEBUG((EFI_D_INFO,"color->color_index = %d(0x%x)\n", color->color_index, color->color_index));
	DEBUG((EFI_D_INFO,"color->hw_colormat = %d(0x%x)\n", color->hw_colormat, color->hw_colormat));
	DEBUG((EFI_D_INFO,"color->hw_blkmode = %d(0x%x)\n", color->hw_blkmode, color->hw_blkmode));
	DEBUG((EFI_D_INFO,"color->red_offset = %d(0x%x)\n", color->red_offset, color->red_offset));
	DEBUG((EFI_D_INFO,"color->red_length = %d(0x%x)\n", color->red_length, color->red_length));
	DEBUG((EFI_D_INFO,"color->red_msb_right = %d(0x%x)\n", color->red_msb_right, color->red_msb_right));
	DEBUG((EFI_D_INFO,"color->green_offset = %d(0x%x)\n", color->green_offset, color->green_offset));
	DEBUG((EFI_D_INFO,"color->green_length = %d(0x%x)\n", color->green_length, color->green_length));
	DEBUG((EFI_D_INFO,"color->green_msb_right = %d(0x%x)\n", color->green_msb_right, color->green_msb_right));
	DEBUG((EFI_D_INFO,"color->blue_offset = %d(0x%x)\n", color->blue_offset, color->blue_offset));
	DEBUG((EFI_D_INFO,"color->blue_length = %d(0x%x)\n", color->blue_length, color->blue_length));
	DEBUG((EFI_D_INFO,"color->blue_msb_right = %d(0x%x)\n", color->blue_msb_right, color->blue_msb_right));
	DEBUG((EFI_D_INFO,"color->transp_offset = %d(0x%x)\n", color->transp_offset, color->transp_offset));
	DEBUG((EFI_D_INFO,"color->transp_length = %d(0x%x)\n", color->transp_length, color->transp_length));
	DEBUG((EFI_D_INFO,"color->transp_msb_right = %d(0x%x)\n", color->transp_msb_right, color->transp_msb_right));
	DEBUG((EFI_D_INFO,"color->color_type = %d(0x%x)\n", color->color_type, color->color_type));
	DEBUG((EFI_D_INFO,"color->bpp = %d(0x%x)\n", color->bpp, color->bpp));


    UINT32  w=(color->bpp * xres_virtual + 7) >> 3;
	dispdata_t   disp_data;
	pandata_t    pan_data;

	pan_data.x_start=xoffset;
	pan_data.x_end=xoffset + (disp_end_x-disp_start_x);
	pan_data.y_start=yoffset;
	pan_data.y_end=yoffset + (disp_end_y-disp_start_y);

	disp_data.x_start=disp_start_x;
	disp_data.y_start=disp_start_y;
	disp_data.x_end=disp_end_x;
	disp_data.y_end=disp_end_y;
	

	canvas_config(OSD2_CANVAS_INDEX, fbmem,
	              			w, yres_virtual,
	              			CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_LINEAR);

	

    osd2_update_color_mode();

	osd2_update_enable();
	

		UINT32 data32;
		data32 = (disp_data.x_start& 0xfff) | (disp_data.x_end & 0xfff) <<16 ;
		DEBUG((EFI_D_INFO,"---VIU_OSD2_BLK0_CFG_W3 = %d(0x%x)\n", data32, data32));
		MmioWrite32(P_VIU_OSD2_BLK0_CFG_W3,data32);

		data32 = (disp_data.y_start & 0xfff) | (disp_data.y_end & 0xfff) <<16 ;
		
		DEBUG((EFI_D_INFO,"osd2_update_disp_geometry---VIU_OSD2_BLK0_CFG_W4 = %d(0x%x)\n", data32, data32));
		MmioWrite32(P_VIU_OSD2_BLK0_CFG_W4,data32);


			data32=(pan_data.x_start & 0x1fff) | (pan_data.x_end & 0x1fff) << 16;
			MmioWrite32(P_VIU_OSD2_BLK0_CFG_W1,data32);
			data32=(pan_data.y_start & 0x1fff) | (pan_data.y_end & 0x1fff) << 16 ;
			MmioWrite32(P_VIU_OSD2_BLK0_CFG_W2,data32);

	MicroSecondDelay(32);
}

void osd_init_hw(void)
{
	UINT32 data32 = 0;
	
	// for(group=0;group<HW_OSD_COUNT;group++)
	// {
		// for(idx=0;idx<HW_REG_INDEX_MAX;idx++)
		// {
			// osd_hw.reg[group][idx].update_func = hw_func_array[group][idx];
		// }
	// }

	//osd_hw.updated[OSD1]=0;
	//osd_hw.updated[OSD2]=0;
	//here we will init default value ,these value only set once .

	data32 |= 4   << 5;  // hold_fifo_lines

	data32 |= 3   << 10; // burst_len_sel: 3=64
	data32 |= 32  << 12; // fifo_depth_val: 32*8=256
	data32 |= 1 << 0;
	
	MmioWrite32(P_VIU_OSD1_FIFO_CTRL_STAT,data32);
	MmioWrite32(P_VIU_OSD2_FIFO_CTRL_STAT,data32);

	setbits_le32(P_VPP_MISC,VPP_POSTBLEND_EN);
	clrbits_le32(P_VPP_MISC, VPP_PREBLEND_EN);
	clrbits_le32(P_VPP_MISC,VPP_OSD1_POSTBLEND|VPP_OSD2_POSTBLEND );
	//data32  = 0x1          << 0; // osd_blk_enable

	data32 = 0x1 << 0;

	data32 |= OSD_GLOBAL_ALPHA_DEF<< 12;
	data32 |= (1<<21)	;
	MmioWrite32(P_VIU_OSD1_CTRL_STAT,data32);
	MmioWrite32(P_VIU_OSD2_CTRL_STAT,data32);

	setbits_le32(P_VPP_MISC,VPP_OUT_SATURATE);

	data32 = MmioRead32(P_VPP_OFIFO_SIZE);

    data32 &= 0xffffe000; //0~13bit

	data32 |= 0x77f;
	MmioWrite32(P_VPP_OFIFO_SIZE,data32);

 
	clrbits_le32(P_VPP_MISC,VPP_POST_FG_OSD2|VPP_PRE_FG_OSD2);
	//osd_hw.osd_order=OSD_ORDER_01;

	//changed by Elvis Yu
	//CLEAR_MPEG_REG_MASK(VPP_MISC,VPP_OSD1_POSTBLEND|VPP_OSD2_POSTBLEND );
	clrbits_le32(P_VPP_MISC,
		VPP_OSD1_POSTBLEND|VPP_OSD2_POSTBLEND|VPP_VD1_POSTBLEND|VPP_VD2_POSTBLEND);
	DEBUG((EFI_D_INFO,"READ_CBUS_REG(VPP_MISC) = 0x%x\n", MmioRead32(CBUS_REG_ADDR(VPP_MISC))));
	
	//osd_hw.enable[OSD2]=osd_hw.enable[OSD1]=DISABLE;
	//osd_hw.fb_gem[OSD1].canvas_idx=OSD1_CANVAS_INDEX;
	//osd_hw.fb_gem[OSD2].canvas_idx=OSD2_CANVAS_INDEX;
	//osd_hw.gbl_alpha[OSD1]=OSD_GLOBAL_ALPHA_DEF;
	//osd_hw.gbl_alpha[OSD2]=OSD_GLOBAL_ALPHA_DEF;
	//osd_hw.color_info[OSD1]=NULL;
	//osd_hw.color_info[OSD2]=NULL;
	//vf.width =vf.height=0;
	//osd_hw.color_key[OSD1]=osd_hw.color_key[OSD2]=0xffffffff;
	//osd_hw.free_scale_enable[OSD1]=osd_hw.free_scale_enable[OSD2]=0;
	//osd_hw.scale[OSD1].h_enable=osd_hw.scale[OSD1].v_enable=0;
	//osd_hw.scale[OSD2].h_enable=osd_hw.scale[OSD2].v_enable=0;
	//osd_hw.mode_3d[OSD2].enable=osd_hw.mode_3d[OSD1].enable=0;
	//osd_hw.block_mode[OSD1] = osd_hw.block_mode[OSD2] = 0;
	//osd_hw.free_scale[OSD1].hfs_enable=0;
	//osd_hw.free_scale[OSD1].hfs_enable=0;
	//osd_hw.free_scale[OSD2].vfs_enable=0;
	//osd_hw.free_scale[OSD2].vfs_enable=0;
	//osd_hw.free_scale_mode[OSD1] = osd_hw.free_scale_mode[OSD2] = 0;

	MmioWrite32(P_HHI_VPU_MEM_PD_REG0,0x00000000);	
	MmioWrite32(P_HHI_VPU_MEM_PD_REG1,0x00000000);
	MmioWrite32(P_VPU_MEM_PD_REG0,    0x00000000);
	MmioWrite32(P_VPU_MEM_PD_REG1,    0x00000000);

	return ;
}

#define  INVALID_BPP_ITEM    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
static const  color_bit_define_t   default_color_format_array[]={
	INVALID_BPP_ITEM,
	INVALID_BPP_ITEM,
	{COLOR_INDEX_02_PAL4,0,0,/*red*/ 0,2,0,/*green*/0,2,0,/*blue*/0,2,0,/*trans*/0,0,0,FB_VISUAL_PSEUDOCOLOR,2},
	INVALID_BPP_ITEM,	
	{COLOR_INDEX_04_PAL16,0,1,/*red*/ 0,4,0,/*green*/0,4,0,/*blue*/0,4,0,/*trans*/0,0,0,FB_VISUAL_PSEUDOCOLOR,4},
	INVALID_BPP_ITEM,	
	INVALID_BPP_ITEM,	
	INVALID_BPP_ITEM,	
	{COLOR_INDEX_08_PAL256,0,2,/*red*/ 0,8,0,/*green*/0,8,0,/*blue*/0,8,0,/*trans*/0,0,0,FB_VISUAL_PSEUDOCOLOR,8},
/*16 bit color*/
	{COLOR_INDEX_16_655,0,4,/*red*/ 10,6,0,/*green*/5,5,0,/*blue*/0,5,0,/*trans*/0,0,0,FB_VISUAL_TRUECOLOR,16},
	{COLOR_INDEX_16_844,1,4,/*red*/ 8,8,0,/*green*/4,4,0,/*blue*/0,4,0,/*trans*/0,0,0,FB_VISUAL_TRUECOLOR,16},
	{COLOR_INDEX_16_6442,2,4,/*red*/ 10,6,0,/*green*/6,4,0,/*blue*/2,4,0,/*trans*/0,2,0,FB_VISUAL_TRUECOLOR,16},
	{COLOR_INDEX_16_4444_R,3,4,/*red*/ 12,4,0,/*green*/8,4,0,/*blue*/4,4,0,/*trans*/0,4,0,FB_VISUAL_TRUECOLOR,16,},
	{COLOR_INDEX_16_4642_R,7,4,/*red*/ 12,4,0,/*green*/6,6,0,/*blue*/2,4,0,/*trans*/0,2,0,FB_VISUAL_TRUECOLOR,16},
	{COLOR_INDEX_16_1555_A,6,4,/*red*/ 10,5,0,/*green*/5,5,0,/*blue*/0,5,0,/*trans*/15,1,0,FB_VISUAL_TRUECOLOR,16},
	{COLOR_INDEX_16_4444_A,5,4,/*red*/ 8,4,0,/*green*/4,4,0,/*blue*/0,4,0,/*trans*/12,4,0,FB_VISUAL_TRUECOLOR,16},
	{COLOR_INDEX_16_565,4,4,/*red*/ 11,5,0,/*green*/5,6,0,/*blue*/0,5,0,/*trans*/0,0,0,FB_VISUAL_TRUECOLOR,16},
/*24 bit color*/
	INVALID_BPP_ITEM,
	INVALID_BPP_ITEM,
	{COLOR_INDEX_24_6666_A,4,7,/*red*/ 12,6,0,/*green*/6,6,0,/*blue*/0,6,0,/*trans*/18,6,0,FB_VISUAL_TRUECOLOR,24},
	{COLOR_INDEX_24_6666_R,3,7,/*red*/ 18,6,0,/*green*/12,6,0,/*blue*/6,6,0,/*trans*/0,6,0,FB_VISUAL_TRUECOLOR,24},
	{COLOR_INDEX_24_8565,2,7,/*red*/ 11,5,0,/*green*/5,6,0,/*blue*/0,5,0,/*trans*/16,8,0,FB_VISUAL_TRUECOLOR,24},
	{COLOR_INDEX_24_5658,1,7,/*red*/ 19,5,0,/*green*/13,6,0,/*blue*/8,5,0,/*trans*/0,8,0,FB_VISUAL_TRUECOLOR,24},
	{COLOR_INDEX_24_888_B,5,7,/*red*/ 0,8,0,/*green*/8,8,0,/*blue*/16,8,0,/*trans*/0,0,0,FB_VISUAL_TRUECOLOR,24},
	{COLOR_INDEX_24_RGB,0,7,/*red*/ 16,8,0,/*green*/8,8,0,/*blue*/0,8,0,/*trans*/0,0,0,FB_VISUAL_TRUECOLOR,24},
/*32 bit color*/
	INVALID_BPP_ITEM,
	INVALID_BPP_ITEM,
	INVALID_BPP_ITEM,
	INVALID_BPP_ITEM,
	{COLOR_INDEX_32_BGRA,3,5,/*red*/ 8,8,0,/*green*/16,8,0,/*blue*/24,8,0,/*trans*/0,8,0,FB_VISUAL_TRUECOLOR,32},
	{COLOR_INDEX_32_ABGR,2,5,/*red*/ 0,8,0,/*green*/8,8,0,/*blue*/16,8,0,/*trans*/24,8,0,FB_VISUAL_TRUECOLOR,32},
	{COLOR_INDEX_32_RGBA,0,5,/*red*/ 24,8,0,/*green*/16,8,0,/*blue*/8,8,0,/*trans*/0,8,0,FB_VISUAL_TRUECOLOR,32},
	{COLOR_INDEX_32_ARGB,1,5,/*red*/ 16,8,0,/*green*/8,8,0,/*blue*/0,8,0,/*trans*/24,8,0,FB_VISUAL_TRUECOLOR,32},
/*YUV color*/
	{COLOR_INDEX_YUV_422,0,3,0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,16},
};


/*-----------------------------------------------------------------------------
 * osd_init --
 *-----------------------------------------------------------------------------
 */
static void osd_layer_init()
{
//	printf("%s\n", __FUNCTION__);
	osd_init_hw();
	osd_setup(0,
                0,
                1920,
                1080,
                1920,
                1080 * 2,
                0,
                0,
                1920- 1,
                1080- 1,
                0x7900000,
                &default_color_format_array[29],
                1);
}

/**
   Initialize the state information for the Display Dxe
   @param  ImageHandle   of the loaded driver
   @param  SystemTable   Pointer to the System Table
   @retval EFI_SUCCESS           Protocol registered
   @retval EFI_OUT_OF_RESOURCES  Cannot allocate protocol data structure
   @retval EFI_DEVICE_ERROR      Hardware problems
**/
EFI_STATUS
EFIAPI
DisplayDxeInitialize (
                      IN EFI_HANDLE         ImageHandle,
                      IN EFI_SYSTEM_TABLE   *SystemTable
                      )
{
  EFI_STATUS Status;

  Status = gBS->LocateProtocol (&gEfiCpuArchProtocolGuid, NULL,
                                (VOID **) &mCpu);
  ASSERT_EFI_ERROR (Status);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  // Query the current display resolution from mailbox
  //Status = mFwProtocol->GetFBSize(&mBootWidth, &mBootHeight);
  //if(EFI_ERROR(Status)) {
  //  return Status;
  //}
    /* Prepare struct */
  if (mDisplay.Mode == NULL) {
    Status = gBS->AllocatePool(
        EfiBootServicesData, sizeof(EFI_GRAPHICS_OUTPUT_PROTOCOL_MODE),
        (VOID **)&mDisplay.Mode);

    ASSERT_EFI_ERROR(Status);
    if (EFI_ERROR(Status))
      return Status;

    ZeroMem(mDisplay.Mode, sizeof(EFI_GRAPHICS_OUTPUT_PROTOCOL_MODE));
  }

  if (mDisplay.Mode->Info == NULL) {
    Status = gBS->AllocatePool(
        EfiBootServicesData, sizeof(EFI_GRAPHICS_OUTPUT_MODE_INFORMATION),
        (VOID **)&mDisplay.Mode->Info);

    ASSERT_EFI_ERROR(Status);
    if (EFI_ERROR(Status))
      return Status;

    ZeroMem(mDisplay.Mode->Info, sizeof(EFI_GRAPHICS_OUTPUT_MODE_INFORMATION));
  }

  /* Set information */
  mDisplay.Mode->MaxMode       = 1;
  mDisplay.Mode->Mode          = 0;
  mDisplay.Mode->Info->Version = 0;

  mDisplay.Mode->Info->HorizontalResolution = 1920;
  mDisplay.Mode->Info->VerticalResolution   = 1080;

  /* SimpleFB runs on a8r8g8b8 (VIDEO_BPP32)*/
  UINT32               LineLength = 1920 * VNBYTES(VIDEO_BPP32);
  UINT32               FrameBufferSize    = LineLength * 1080;
  EFI_PHYSICAL_ADDRESS FrameBufferAddress = 0x7900000;

  mDisplay.Mode->Info->PixelsPerScanLine = 1920;
  mDisplay.Mode->Info->PixelFormat = PixelBlueGreenRedReserved8BitPerColor;
  mDisplay.Mode->SizeOfInfo      = sizeof(EFI_GRAPHICS_OUTPUT_MODE_INFORMATION);
  mDisplay.Mode->FrameBufferBase = FrameBufferAddress;
  mDisplay.Mode->FrameBufferSize = FrameBufferSize;

  /* Create the FrameBufferBltLib configuration. */
  Status = FrameBufferBltConfigure(
      (VOID *)(UINTN)mDisplay.Mode->FrameBufferBase, mDisplay.Mode->Info,
      mFrameBufferBltLibConfigure, &mFrameBufferBltLibConfigureSize);

  if (Status == RETURN_BUFFER_TOO_SMALL) {
    mFrameBufferBltLibConfigure = AllocatePool(mFrameBufferBltLibConfigureSize);
    if (mFrameBufferBltLibConfigure != NULL) {
      Status = FrameBufferBltConfigure(
          (VOID *)(UINTN)mDisplay.Mode->FrameBufferBase, mDisplay.Mode->Info,
          mFrameBufferBltLibConfigure, &mFrameBufferBltLibConfigureSize);
    }
  }

  ASSERT_EFI_ERROR(Status);
  ZeroMem((VOID *) (UINTN) FrameBufferAddress, FrameBufferSize);


  /* Register handle */
  Status = gBS->InstallMultipleProtocolInterfaces(
      &mDevice, &gEfiDevicePathProtocolGuid, &mDisplayProtoDevicePath,
      &gEfiCallerIdGuid, &mDisplay, NULL);

  ASSERT_EFI_ERROR (Status);
  if (EFI_ERROR (Status)) {
    return Status;
  }
  
	  
  Status = EfiLibInstallDriverBindingComponentName2 (
             ImageHandle,
             SystemTable,
             &mDriverBinding,
             ImageHandle,
             &gComponentName,
             &gComponentName2
             );
  ASSERT_EFI_ERROR (Status);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  return Status;
}

STATIC
EFI_STATUS
EFIAPI
DriverSupported (
  IN EFI_DRIVER_BINDING_PROTOCOL *This,
  IN EFI_HANDLE                  Controller,
  IN EFI_DEVICE_PATH_PROTOCOL    *RemainingDevicePath
  )
{
  VOID *Temp;

  if (Controller != mDevice) {
    return EFI_UNSUPPORTED;
  }

  if (gBS->HandleProtocol(Controller, &gEfiGraphicsOutputProtocolGuid,
                          (VOID **) &Temp) == EFI_SUCCESS) {
    return EFI_ALREADY_STARTED;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
DriverStart (
  IN EFI_DRIVER_BINDING_PROTOCOL *This,
  IN EFI_HANDLE                  Controller,
  IN EFI_DEVICE_PATH_PROTOCOL    *RemainingDevicePath
  )
{
  //UINTN Index;
  //UINTN TempIndex;
  EFI_STATUS Status;
  VOID *Dummy;

  Status = gBS->OpenProtocol (
    Controller,
    &gEfiCallerIdGuid,
    (VOID **) &Dummy,
    This->DriverBindingHandle,
    Controller,
    EFI_OPEN_PROTOCOL_BY_DRIVER
    );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  gDisplayProto.Mode = AllocateZeroPool(sizeof(EFI_GRAPHICS_OUTPUT_PROTOCOL_MODE));
  if (gDisplayProto.Mode == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto done;
  }

  gDisplayProto.Mode->Info = AllocateZeroPool(sizeof(EFI_GRAPHICS_OUTPUT_MODE_INFORMATION));
  if (gDisplayProto.Mode->Info == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto done;
  }

  // PcdSet8(PcdDisplayEnableScaledVModes,
  //         PcdGet8(PcdDisplayEnableScaledVModes) & ALL_MODES);

  // if (PcdGet8(PcdDisplayEnableScaledVModes) == 0) {
  //   PcdSet8(PcdDisplayEnableScaledVModes, JUST_NATIVE_ENABLED);
  // }

  // mLastMode = 0;
  // for  (TempIndex = 0, Index = 0;
  //       TempIndex < ELES(mGopModeTemplate); TempIndex++) {
  //   if ((PcdGet8(PcdDisplayEnableScaledVModes) & (1 << TempIndex)) != 0) {
  //     DEBUG((EFI_D_ERROR, "Mode %u: %u x %u present\n",
  //            TempIndex, mGopModeTemplate[TempIndex].Width,
  //            mGopModeTemplate[TempIndex].Height));

  //     CopyMem(&mGopModeData[Index], &mGopModeTemplate[TempIndex],
  //             sizeof (GOP_MODE_DATA));
  //     mLastMode = Index;
  //     Index++;
  //   }
  // }

  // if (PcdGet8(PcdDisplayEnableScaledVModes) == JUST_NATIVE_ENABLED) {
  //   /*
  //    * mBootWidth x mBootHeight may not be sensible,
  //    * so clean it up, since we won't be adding
  //    * any other extra vmodes.
  //    */
  //   if (mBootWidth < 640 ||
  //       mBootHeight < 480) {
  //     /*
  //      * At least 640x480.
  //      */
  //     mBootWidth = 640;
  //     mBootHeight = 480;
  //   } else if (mBootWidth == 800 &&
  //              mBootHeight == 480) {
  //     /*
  //      * The Pi 7" screen is close to 800x600, just
  //      * pretend it is.
  //      */
  //     mBootHeight = 600;
  //   }
  // }

  // if ((PcdGet8(PcdDisplayEnableScaledVModes) & MODE_NATIVE_ENABLED) != 0) {
  //    /*
  //     * Adjust actual native res only if enabled native res (so
  //     * last mode is native res).
  //     */
  //    mGopModeData[mLastMode].Width = mBootWidth;
  //    mGopModeData[mLastMode].Height = mBootHeight;
  // }

  // for (Index = 0; Index <= mLastMode; Index++) {
  //   UINTN FbSize;
  //   UINTN FbPitch;
  //   EFI_PHYSICAL_ADDRESS FbBase;

  //   GOP_MODE_DATA *Mode = &mGopModeData[Index];

  //   Status = mFwProtocol->GetFB(Mode->Width, Mode->Height,
  //                               PI2_BITS_PER_PIXEL, &FbBase,
  //                               &FbSize, &FbPitch);
  //   if (EFI_ERROR(Status)) {
  //     goto done;
  //   }

  //   //
  //   // There is no way to communicate pitch back to OS. OS and even UEFI
  //   // expect a fully linear frame buffer. So the width should
  //   // be based on the frame buffer's pitch value. In some cases VC
  //   // firmware would allocate ao frame buffer with some padding
  //   // presumably to be 8 byte align.
  //   //
  //   Mode->Width = FbPitch / PI2_BYTES_PER_PIXEL;

  //   DEBUG((EFI_D_INFO, "Mode %u: %u x %u framebuffer is %u bytes at %p\n",
  //          Index, Mode->Width, Mode->Height, FbSize, FbBase));

  //   ASSERT (FbPitch != 0);
  //   ASSERT (FbBase != 0);
  //   ASSERT (FbSize != 0);
  // }

  // // Both set the mode and initialize current mode information.
  // gDisplayProto.Mode->MaxMode = mLastMode + 1;
  vpu_driver_init();
  osd_layer_init();
  DisplaySetMode(&gDisplayProto, 8);

  Status = gBS->InstallMultipleProtocolInterfaces (
    &Controller, &gEfiGraphicsOutputProtocolGuid,
    &gDisplayProto, NULL);
  if (EFI_ERROR (Status)) {
    goto done;
  }

  RegisterScreenshotHandlers();

done:
  if (EFI_ERROR (Status)) {
    DEBUG((EFI_D_ERROR, "Could not start DisplayDxe: %r\n", Status));
    if (gDisplayProto.Mode->Info != NULL) {
      FreePool(gDisplayProto.Mode->Info);
      gDisplayProto.Mode->Info = NULL;
    }

    if (gDisplayProto.Mode != NULL) {
      FreePool(gDisplayProto.Mode);
      gDisplayProto.Mode = NULL;
    }

    gBS->CloseProtocol (
      Controller,
      &gEfiCallerIdGuid,
      This->DriverBindingHandle,
      Controller
      );
  }
  return Status;
}

STATIC
EFI_STATUS
EFIAPI
DriverStop (
  IN EFI_DRIVER_BINDING_PROTOCOL *This,
  IN EFI_HANDLE                  Controller,
  IN UINTN                       NumberOfChildren,
  IN EFI_HANDLE                  *ChildHandleBuffer
  )
{
  EFI_STATUS Status;

  ClearScreen(&gDisplayProto);

  Status = gBS->UninstallMultipleProtocolInterfaces (
    Controller, &gEfiGraphicsOutputProtocolGuid,
    &gDisplayProto, NULL);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  FreePool(gDisplayProto.Mode->Info);
  gDisplayProto.Mode->Info = NULL;
  FreePool(gDisplayProto.Mode);
  gDisplayProto.Mode = NULL;

  gBS->CloseProtocol (
    Controller,
    &gEfiCallerIdGuid,
    This->DriverBindingHandle,
    Controller
    );

  return Status;
}