/** @file
*
*  Copyright (c) 2011-2012, ARM Limited. All rights reserved.
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

#include <Library/ArmLib.h>
#include <Library/ArmPlatformLib.h>
#include <Library/TimerLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Ppi/ArmMpCoreInfo.h>
#include "Clock.h"
#include "DDR.h"
#include "MMC.h"
#include "SecMmc.h"
#define AML_WATCH_DOG_DISABLE() \
	MmioWrite16(0x2640,0); \
	MmioWrite16(0x2641,0);
	
RETURN_STATUS
EFIAPI
TimerConstructor (
  VOID
  );

ARM_CORE_INFO mArmPlatformNullMpCoreInfoTable[] = {
  {
    // Cluster 0, Core 0
    0x0, 0x0,

    // MP Core MailBox Set/Get/Clear Addresses and Clear Value
    (EFI_PHYSICAL_ADDRESS)0,
    (EFI_PHYSICAL_ADDRESS)0,
    (EFI_PHYSICAL_ADDRESS)0,
    (UINT64)0xFFFFFFFF
  },
  {
    // Cluster 0, Core 1
    0x0, 0x1,

    // MP Core MailBox Set/Get/Clear Addresses and Clear Value
    (EFI_PHYSICAL_ADDRESS)0,
    (EFI_PHYSICAL_ADDRESS)0,
    (EFI_PHYSICAL_ADDRESS)0,
    (UINT64)0xFFFFFFFF
  },
  {
    // Cluster 0, Core 2
    0x0, 0x2,

    // MP Core MailBox Set/Get/Clear Addresses and Clear Value
    (EFI_PHYSICAL_ADDRESS)0,
    (EFI_PHYSICAL_ADDRESS)0,
    (EFI_PHYSICAL_ADDRESS)0,
    (UINT64)0xFFFFFFFF
  },
  {
    // Cluster 0, Core 3
    0x0, 0x3,

    // MP Core MailBox Set/Get/Clear Addresses and Clear Value
    (EFI_PHYSICAL_ADDRESS)0,
    (EFI_PHYSICAL_ADDRESS)0,
    (EFI_PHYSICAL_ADDRESS)0,
    (UINT64)0xFFFFFFFF
  }
};

struct pll_clk_settings{
	unsigned sys_pll_cntl;	//HHI_SYS_PLL_CNTL	       0x10c0
	unsigned sys_clk_cntl;	//HHI_SYS_CPU_CLK_CNTL0	0x1067
	unsigned sys_clk_cntl1;	//HHI_SYS_CPU_CLK_CNTL1	0x1057
	unsigned sys_clk;
	unsigned a9_clk;
	unsigned mpll_cntl;
	unsigned mpeg_clk_cntl;
	unsigned vid_pll_cntl;
	unsigned vid2_pll_cntl;

	unsigned spi_setting;
	unsigned nfc_cfg;
	unsigned sdio_cmd_clk_divide;
	unsigned sdio_time_short;
	unsigned uart;

	unsigned clk81;
}__attribute__ ((packed));

//----------------------------------------------------------------------
//Please set the M8 CPU clock(unit: MHz)
//legal value: 600, 792, 996, 1200
#define M8_CPU_CLK 		    (792)
#define CONFIG_SYS_CPU_CLK	(M8_CPU_CLK)
//----------------------------------------------------------------------


#if   (600 == CONFIG_SYS_CPU_CLK)
	#define	M8_SYS_PLL_N  (1)
	#define	M8_SYS_PLL_M  (50)
	#define M8_SYS_PLL_OD (1)
#elif (792 == CONFIG_SYS_CPU_CLK)
	#define	M8_SYS_PLL_N  (1)
	#define	M8_SYS_PLL_M  (66)
	#define M8_SYS_PLL_OD (1)
#elif (996 == CONFIG_SYS_CPU_CLK)
	#define	M8_SYS_PLL_N  (1)
	#define	M8_SYS_PLL_M  (83)
	#define M8_SYS_PLL_OD (1)
#elif (1200 == CONFIG_SYS_CPU_CLK)
	#define	M8_SYS_PLL_N  (1)
	#define	M8_SYS_PLL_M  (50)
	#define M8_SYS_PLL_OD (0)
#else
	#error "CONFIG_SYS_CPU_CLK is not set! Please set M8 CPU clock first!\n"
#endif

static struct pll_clk_settings __plls
={
	//current test: >=1320MHz  can not work stable@VDD_CPU=1.2V	
	//0x1098[0xc1104260]
	.sys_pll_cntl=	(M8_SYS_PLL_OD << 16) | //OD
					(M8_SYS_PLL_N  << 9 ) | //N
					(M8_SYS_PLL_M  << 0 ) | //M
					(2<<20) | // 4 bits SYS_DPLL_SS_CLK 
					(1<<24) | // 4 bits SYS_DPLL_SS_AMP 
					(0<<28) | // 1 bit SYS_DPLL_SSEN 
					(1<<29) | // 1 bit SYS_DPLL_RESET 
					(1<<30),  // 1 bit SYS_DPLL_EN 
	//A9 clock setting
	//0x1067[0xc110419c]
    .sys_clk_cntl=	(1 << 7) | // 0:oscin 1:scale out
                  	(1 << 5) | // A9_AT CLKEN
                  	(1 << 4) | // A9_APB CLKEN
                  	(0 << 2) | // 0:div1, 1:div2, 2:div3, 3:divn
                  	(1 << 0),  // 0:oscin, 1:sys_pll, 2:ddr_pll, 3:no clock 
    //A9 clock              	
    .sys_clk = CONFIG_SYS_CPU_CLK,//MHz
    .a9_clk  = CONFIG_SYS_CPU_CLK,//MHz

	.mpll_cntl = 0x600009A9,	//2.5G, fixed

	//MPEG clock(clock81) setting
	//[B14:B12]MPEG_CK_SEL 0:XTAL/32KHz 1:ddr_pll 2:mpll_clk_out0
	//                                        3:mpll_clk_out1 4:mpll_clk_out2 5:fclk_div4
	//                                        6:fclk_div3 7:fclk_div5
	//[B8]0:clk81=XTAL/32KHz 1:clk81=pll
	//[B7]enable gating
	//[B9]XTAL/32KHz, 0 : 24MHz, 1:32KHz
	//0x105d [0xc1104174]
    .mpeg_clk_cntl= (5 << 12) |    //[B14,B13,B12] select fclk_div4: 2.55GHz/4=637.5MHz
    				(1 << 8 ) |    //[B8] select pll
    				(1 << 7 ) |    //[B7] cntl_hi_mpeg_div_en, enable gating
                    (3 << 0 ) |    //[B6-B0] div  (n+1)  fclk_div5=2.55G/4=637.5MHz, clk81=637.5MHz/(3+1)=159.375MHz
					(1 << 15),     //[B15] Connect clk81 to the PLL divider output

	.vid_pll_cntl = 0x6001043D,
	.vid2_pll_cntl = 0x61000032,
	.clk81=159375000, //2.55GHz/4/4=159.375MHz

#if defined (CONFIG_VLSI_EMULATOR)
    .spi_setting=0xeb949, //it need fine tune?
#else	
    .spi_setting=0xea949, //it need fine tune?
#endif    
    .nfc_cfg=(((0)&0xf)<<10) | (0<<9) | (0<<5) | 5,
    .sdio_cmd_clk_divide=5,
    .sdio_time_short=(250*180000)/(2*(12)),
};

VOID
ClockInit (
  VOID
  )
{
	UINT8 n_pll_try_times = 0;
	MmioOr32(CBUS_REG_ADDR(AM_ANALOG_TOP_REG1), 1);

	//bandgap enable
	//SYS,MPLL
	MmioOr32(CBUS_REG_ADDR(HHI_MPLL_CNTL6),(1<<26));
	//VID,HDMI
	MmioOr32(CBUS_REG_ADDR(HHI_VID_PLL_CNTL5),(1<<16));
	
	//DDR
	MmioOr32(0xc8000410,(1<<12));

	//MUST	
	//switch a9 clock to  oscillator in the first.  This is sync mux.
	MmioAnd32(CBUS_REG_ADDR(HHI_A9_CLK_CNTL), (~(1<<7)));
	
	MicroSecondDelay(10);
	MmioWrite32(CBUS_REG_ADDR(HHI_A9_CLK_CNTL), 0);
	MicroSecondDelay(10);
	
	//???
	MmioAnd32(CBUS_REG_ADDR(HHI_MPEG_CLK_CNTL), (~(1<<8)));
	MicroSecondDelay(100);

	//PLL setup: bandgap enable -> 1ms delay -> PLL reset + PLL set ->
	//                 PLL enable -> 1ms delay -> PLL release from reset

	//SYS PLL init
	do{
		//BANDGAP reset for SYS_PLL,MPLL lock fail
    MmioAnd32(CBUS_REG_ADDR(HHI_MPLL_CNTL6),~(1<<26));
		MicroSecondDelay(10);
		MmioOr32(CBUS_REG_ADDR(HHI_MPLL_CNTL6),(1<<26));
		MicroSecondDelay(1000); //1ms for bandgap bootup

		PLL_ENTER_RESET(HHI_SYS_PLL_CNTL);
		MmioWrite32(CBUS_REG_ADDR(HHI_SYS_PLL_CNTL2),CFG_SYS_PLL_CNTL_2);
		MmioWrite32(CBUS_REG_ADDR(HHI_SYS_PLL_CNTL3),CFG_SYS_PLL_CNTL_3);
		MmioWrite32(CBUS_REG_ADDR(HHI_SYS_PLL_CNTL4),CFG_SYS_PLL_CNTL_4);
		MmioWrite32(CBUS_REG_ADDR(HHI_SYS_PLL_CNTL5),CFG_SYS_PLL_CNTL_5);
		PLL_SETUP(HHI_SYS_PLL_CNTL, (&__plls)->sys_pll_cntl);
		PLL_RELEASE_RESET(HHI_SYS_PLL_CNTL);

		PLL_LOCK_CHECK(n_pll_try_times,1);

	}while((Rd_cbus(HHI_SYS_PLL_CNTL)&(1<<31))==0);

	//A9 clock setting
	Wr_cbus(HHI_A9_CLK_CNTL,((&__plls)->sys_clk_cntl & (~(1<<7))));
	MicroSecondDelay(1);
	//enable A9 clock
	Wr_cbus(HHI_A9_CLK_CNTL,((&__plls)->sys_clk_cntl | (1<<7)));

	//MPLL init
	//FIXED PLL/Multi-phase PLL, fixed to 2.55GHz
	PLL_ENTER_RESET(HHI_MPLL_CNTL);	//set reset bit to 1
	Wr_cbus(HHI_DPLL_TOP_0, 0x100);
	Wr_cbus(HHI_MPLL_CNTL2, CFG_MPLL_CNTL_2 );
	Wr_cbus(HHI_MPLL_CNTL3, CFG_MPLL_CNTL_3 );
	Wr_cbus(HHI_MPLL_CNTL4, CFG_MPLL_CNTL_4 );
	Wr_cbus(HHI_MPLL_CNTL5, CFG_MPLL_CNTL_5 );
	Wr_cbus(HHI_MPLL_CNTL6, CFG_MPLL_CNTL_6 );
	Wr_cbus(HHI_MPLL_CNTL7, CFG_MPLL_CNTL_7 );
	Wr_cbus(HHI_MPLL_CNTL8, CFG_MPLL_CNTL_8 );
	Wr_cbus(HHI_MPLL_CNTL9, CFG_MPLL_CNTL_9 );
	PLL_SETUP(HHI_MPLL_CNTL, (&__plls)->mpll_cntl);	//2.55G, FIXED
	PLL_RELEASE_RESET(HHI_MPLL_CNTL);	//set reset bit to 0

	//PLL_WAIT_FOR_LOCK(HHI_MPLL_CNTL); //need bandgap reset?
	n_pll_try_times=0;
	do{
		PLL_LOCK_CHECK(n_pll_try_times,5);
		if((Rd_cbus(HHI_MPLL_CNTL)&(1<<31))!=0)
			break;
		Wr_cbus(HHI_MPLL_CNTL,Rd_cbus(HHI_MPLL_CNTL) | (1<<29));
		MicroSecondDelay(1);
		PLL_RELEASE_RESET(HHI_MPLL_CNTL);
		MicroSecondDelay(1);
	}while((Rd_cbus(HHI_MPLL_CNTL)&(1<<31))==0);

	//MPLL is fixed to 2.55GHz
	//clk81=fclk_div4 /2=637.5/4=159.375M
	Wr_cbus(HHI_MPEG_CLK_CNTL, (&__plls)->mpeg_clk_cntl );

	//here need do UART init, print MPLL LOCK CHECK info
	//serial_init(__plls.uart);
	
	n_pll_try_times=0;

	//VID PLL init
	do{
		//BANDGAP reset for VID_PLL, VID2_PLL
		Wr_reg_bits(HHI_VID_PLL_CNTL5,0,16,1);
		MicroSecondDelay(10);
		Wr_reg_bits(HHI_VID_PLL_CNTL5,1,16,1);
		
		MicroSecondDelay(1); //1ms for bandgap bootup

		PLL_ENTER_RESET(HHI_VID_PLL_CNTL);
		Wr_cbus(HHI_VID_PLL_CNTL2,CFG_VID_PLL_CNTL_2);
		Wr_cbus(HHI_VID_PLL_CNTL3,CFG_VID_PLL_CNTL_3);
		Wr_cbus(HHI_VID_PLL_CNTL4,CFG_VID_PLL_CNTL_4);
		Wr_cbus(HHI_VID_PLL_CNTL5,CFG_VID_PLL_CNTL_5);
		PLL_SETUP(HHI_VID_PLL_CNTL, (&__plls)->vid_pll_cntl);
		PLL_RELEASE_RESET(HHI_VID_PLL_CNTL);

		PLL_LOCK_CHECK(n_pll_try_times,2);

	}while((Rd_cbus(HHI_VID_PLL_CNTL)&(1<<31))==0);

 	MicroSecondDelay(100);

}

int init_pctl_ddr3(struct ddr_set *timing_set)
{
	int nRet = -1;
	int loop_whi_count = 0; //all while loop share this number
	int loop_err_count = 0;

#define UPCTL_STAT_MASK (7)
#define UPCTL_STAT_INIT (0)
#define UPCTL_STAT_CONFIG (1)
#define UPCTL_STAT_ACCESS (3)
#define UPCTL_STAT_LOW_POWER (5)

#define UPCTL_CMD_INIT (0)
#define UPCTL_CMD_CONFIG (1)
#define UPCTL_CMD_GO (2)
#define UPCTL_CMD_SLEEP (3)
#define UPCTL_CMD_WAKEUP (4)

//PUB PIR setting
#define PUB_PIR_INIT (1 << 0)
#define PUB_PIR_ZCAL (1 << 1)
#define PUB_PIR_CA (1 << 2)
#define PUB_PIR_PLLINIT (1 << 4)
#define PUB_PIR_DCAL (1 << 5)
#define PUB_PIR_PHYRST (1 << 6)
#define PUB_PIR_DRAMRST (1 << 7)
#define PUB_PIR_DRAMINIT (1 << 8)
#define PUB_PIR_WL (1 << 9)
#define PUB_PIR_QSGATE (1 << 10)
#define PUB_PIR_WLADJ (1 << 11)
#define PUB_PIR_RDDSKW (1 << 12)
#define PUB_PIR_WRDSKW (1 << 13)
#define PUB_PIR_RDEYE (1 << 14)
#define PUB_PIR_WREYE (1 << 15)
#define PUB_PIR_ICPC (1 << 16)
#define PUB_PIR_PLLBYP (1 << 17)
#define PUB_PIR_CTLDINIT (1 << 18)
#define PUB_PIR_RDIMMINIT (1 << 19)
#define PUB_PIR_CLRSR (1 << 27)
#define PUB_PIR_LOCKBYP (1 << 28)
#define PUB_PIR_DCALBYP (1 << 29)
#define PUB_PIR_ZCALBYP (1 << 30)
#define PUB_PIR_INITBYP (1 << 31)

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//HX DDR init code
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//PUB PGSR0
#define PUB_PGSR0_IDONE (1 << 0)
#define PUB_PGSR0_PLDONE (1 << 1)
#define PUB_PGSR0_DCDONE (1 << 2)
#define PUB_PGSR0_ZCDONE (1 << 3)
#define PUB_PGSR0_DIDONE (1 << 4)
#define PUB_PGSR0_WLDONE (1 << 5)
#define PUB_PGSR0_QSGDONE (1 << 6)
#define PUB_PGSR0_WLADONE (1 << 7)
#define PUB_PGSR0_RDDONE (1 << 8)
#define PUB_PGSR0_WDDONE (1 << 9)
#define PUB_PGSR0_REDONE (1 << 10)
#define PUB_PGSR0_WEDONE (1 << 11)
#define PUB_PGSR0_CADONE (1 << 12)

#define PUB_PGSR0_ZCERR (1 << 20)
#define PUB_PGSR0_WLERR (1 << 21)
#define PUB_PGSR0_QSGERR (1 << 22)
#define PUB_PGSR0_WLAERR (1 << 23)
#define PUB_PGSR0_RDERR (1 << 24)
#define PUB_PGSR0_WDERR (1 << 25)
#define PUB_PGSR0_REERR (1 << 26)
#define PUB_PGSR0_WEERR (1 << 27)
#define PUB_PGSR0_CAERR (1 << 28)
#define PUB_PGSR0_CAWERR (1 << 29)
#define PUB_PGSR0_VTDONE (1 << 30)
#define PUB_PGSR0_DTERR (PUB_PGSR0_RDERR | PUB_PGSR0_WDERR | PUB_PGSR0_REERR | PUB_PGSR0_WEERR)

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//HX DDR init code
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define MMC_RESET_CNT_MAX (8)

	int nTempVal = 0;
	int nRetryCnt = 0; //mmc reset try counter; max to try 8 times

mmc_init_all:

	//hx_serial_puts("\nAml log : mmc_init_all\n");
	nRetryCnt++;

	if (nRetryCnt > MMC_RESET_CNT_MAX)
		return nRet;

	//MMC/DMC reset
	MmioWrite32( P_DMC_SOFT_RST,0);
	MmioWrite32(P_DMC_SOFT_RST1,0);
	MicroSecondDelay(10);
	MmioWrite32( P_DMC_SOFT_RST,~0);
	MmioWrite32(P_DMC_SOFT_RST1,~0);

	//unsigned int nMMC_DDR_set = timing_set->t_mmc_ddr_ctrl;
	//int nM8_DDR_CHN_SET = (nMMC_DDR_set >> 24 ) & 3;

/////////////////////////////////////////////////////
pub_init_ddr0:
	loop_whi_count = 0;
	//uboot timming.c setting
	//UPCTL and PUB clock and reset.
	//hx_serial_puts("\nAml log : DDR0 - init start...\n");
	//hx_serial_puts("Aml log : DDR0 - APD,CLK done\n");

	MmioWrite32( P_DDR0_SOFT_RESET,0x0);
	MicroSecondDelay(10);
	MmioWrite32(P_DDR0_SOFT_RESET,0xf);

//#if defined(CONFIG_DDR_LOW_POWER_DISABLE)
	//writel((timing_set->t_ddr_apd_ctrl & (~0xFF)), P_DDR0_APD_CTRL);
//#else
	MmioWrite32( P_DDR0_APD_CTRL,timing_set->t_ddr_apd_ctrl);
//#endif

	MmioWrite32(P_DDR0_CLK_CTRL,timing_set->t_ddr_clk_ctrl);


//#if (defined LPDDR2) || (defined LPDDR3)
	//writel(0x12b, P_DDR0_CLK_CTRL);
//#endif

	MmioWrite32( P_DDR0_PUB_IOVCR0,0x49494949);
	MmioWrite32(P_DDR0_PUB_IOVCR1,0x49494949);

	//DDR0  timing registers
	MmioWrite32(P_DDR0_PCTL_TOGCNT1U,timing_set->t_pctl_1us_pck);	 //1us = nn cycles.
	MmioWrite32(P_DDR0_PCTL_TOGCNT100N,timing_set->t_pctl_100ns_pck); //100ns = nn cycles.
	MmioWrite32(P_DDR0_PCTL_TINIT,timing_set->t_pctl_init_us);		  //200us.
	MmioWrite32(P_DDR0_PCTL_TRSTH,timing_set->t_pctl_rsth_us);		  // 0 for ddr2;  2 for simulation; 500 for ddr3.
	MmioWrite32(P_DDR0_PCTL_TRSTL,timing_set->t_pctl_rstl_us);		  // 0 for ddr2;  2 for simulation; 500 for ddr3.

//#if defined(CONFIG_DDR_LOW_POWER_DISABLE)
	//writel((timing_set->t_pctl_mcfg & (~(0xFF << 8))), P_DDR0_PCTL_MCFG);
//#else
	MmioWrite32(P_DDR0_PCTL_MCFG,timing_set->t_pctl_mcfg);
//#endif

	MmioWrite32( P_DDR0_PCTL_MCFG1,timing_set->t_pctl_mcfg1); //enable hardware c_active_in;

	MmioWrite32(P_DDR0_PUB_DCR,timing_set->t_pub_dcr);

	//PUB MRx registers.
	MmioWrite32(P_DDR0_PUB_MR0,timing_set->t_pub_mr[0]);
	MmioWrite32(P_DDR0_PUB_MR1,timing_set->t_pub_mr[1]);
	MmioWrite32(P_DDR0_PUB_MR2,timing_set->t_pub_mr[2]);
	MmioWrite32( P_DDR0_PUB_MR3,timing_set->t_pub_mr[3]);

	//DDR SDRAM timing parameter.
	MmioWrite32( P_DDR0_PUB_DTPR0,timing_set->t_pub_dtpr[0]);
	MmioWrite32( P_DDR0_PUB_DTPR1,timing_set->t_pub_dtpr[1]);
	MmioWrite32( P_DDR0_PUB_DTPR2,timing_set->t_pub_dtpr[2]);

	//configure auto refresh when data training.
	MmioWrite32(P_DDR0_PUB_PGCR2,(MmioRead32(P_DDR0_PUB_PGCR2) & 0xfffc0000) | 0xc00);

	MmioWrite32(P_DDR0_PUB_DTCR,((MmioRead32(P_DDR0_PUB_DTCR) & 0x00ffffbf) | (1 << 28) | (1 << 24) | (1 << 6) | (1 << 23)));

	//for training gate extended gate
	MmioWrite32(P_DDR0_PUB_DX0GCR3,0x8); //for pdr mode bug //for pdr mode bug this will cause some chip bit deskew  jiaxing add
	MmioWrite32(P_DDR0_PUB_DX1GCR3,0x8);
	MmioWrite32(P_DDR0_PUB_DX2GCR3,0x8);
	MmioWrite32(P_DDR0_PUB_DX3GCR3,0x8);
//#if ((defined LPDDR2) || (defined LPDDR3))
	//writel((readl(P_DDR0_PUB_DSGCR)) | (0x3 << 06), P_DDR0_PUB_DSGCR); //eanble gate extension  dqs gate can help to bit deskew also
																	   //writel((((readl(P_DDR0_PUB_ZQCR))&0xff01fffc)|(1<<24)), P_DDR0_PUB_ZQCR);
//#else
	MmioWrite32(P_DDR0_PUB_DSGCR,(MmioRead32(P_DDR0_PUB_DSGCR)) | (0x1 << 06)); //eanble gate extension  dqs gate can help to bit deskew also
	//writel(((readl(P_DDR0_PUB_ZQCR))&0xff01fffc), P_DDR0_PUB_ZQCR); //for vt bug disable IODLMT
//#endif

	MmioWrite32( P_DDR0_PUB_ZQCR,((MmioRead32(P_DDR0_PUB_ZQCR)) & 0xff01fffc)); //for vt bug disable IODLMT

	MmioWrite32(P_DDR0_PUB_ZQ0PR,(timing_set->t_pub_zq0pr) | ((readl(P_DDR0_PUB_ZQ0PR)) & 0xffffff00));
	MmioWrite32(P_DDR0_PUB_ZQCR,(1 << 1) | ((readl(P_DDR0_PUB_ZQCR)) & 0xfffffffe));

	MmioWrite32(P_DDR0_PUB_DXCCR,timing_set->t_pub_dxccr);

	//writel(((readl(P_DDR0_PUB_ZQCR))&(~(1<<2))), P_DDR0_PUB_ZQCR);

	MmioWrite32(P_DDR0_PUB_ACBDLR0,MmioRead32(P_DDR0_PUB_ACBDLR0) | (timing_set->t_pub_acbdlr0)); //place before write level

	MmioWrite32(P_DDR0_PUB_PTR0,timing_set->t_pub_ptr[0]);
	MmioWrite32(P_DDR0_PUB_PTR1,timing_set->t_pub_ptr[1]);

	MmioWrite32(P_DDR0_PUB_ACIOCR0,MmioRead32(P_DDR0_PUB_ACIOCR0) & 0xdfffffff);

	//configure for phy update request and ack.
	MmioWrite32(P_DDR0_PUB_DSGCR,((MmioRead32(P_DDR0_PUB_DSGCR) & 0xffffffef) | (1 << 6))); //other bits.

	//for simulation to reduce the initial time.
	//real value should be based on the DDR3 SDRAM clock cycles.
	MmioWrite32(P_DDR0_PUB_PTR3,timing_set->t_pub_ptr[3]);
	MmioWrite32(P_DDR0_PUB_PTR4,timing_set->t_pub_ptr[4]);

	while (!(MmioRead32(P_DDR0_PCTL_DFISTSTAT0) & 1))
	{
		DDR_INIT_WHILE_LOOP(loop_whi_count);
	}
	//hx_serial_puts("Aml log : DDR0 - DFI status check done\n");

	MmioWrite32(P_DDR0_PCTL_POWCTL,1);
	while (!(MmioRead32(P_DDR0_PCTL_POWSTAT) & 1))
	{
		DDR_INIT_WHILE_LOOP(loop_whi_count);
	}

	//DDR0 PCTL
	MmioWrite32( P_DDR0_PCTL_TREFI,timing_set->t_pctl_trefi);
	MmioWrite32(P_DDR0_PCTL_TREFI_MEM_DDR3,timing_set->t_pctl_trefi_mem_ddr3);
	MmioWrite32( P_DDR0_PCTL_TREFI,timing_set->t_pctl_trefi | (1 << 31));

	MmioWrite32(P_DDR0_PCTL_TMRD,timing_set->t_pctl_tmrd);
	MmioWrite32( P_DDR0_PCTL_TRFC,timing_set->t_pctl_trfc);
	MmioWrite32(P_DDR0_PCTL_TRP,timing_set->t_pctl_trp);
	MmioWrite32(P_DDR0_PCTL_TAL,timing_set->t_pctl_tal);
	MmioWrite32(P_DDR0_PCTL_TCWL,timing_set->t_pctl_tcwl);

	//DDR0 PCTL
	MmioWrite32(P_DDR0_PCTL_TCL,timing_set->t_pctl_tcl);
	MmioWrite32(P_DDR0_PCTL_TRAS,timing_set->t_pctl_tras);
	MmioWrite32( P_DDR0_PCTL_TRC,timing_set->t_pctl_trc);
	MmioWrite32( P_DDR0_PCTL_TRCD,timing_set->t_pctl_trcd);
	MmioWrite32(P_DDR0_PCTL_TRRD,timing_set->t_pctl_trrd);

	MmioWrite32( P_DDR0_PCTL_TRTP,timing_set->t_pctl_trtp);
	MmioWrite32(P_DDR0_PCTL_TWR,timing_set->t_pctl_twr);
	MmioWrite32(P_DDR0_PCTL_TWTR,timing_set->t_pctl_twtr);
	MmioWrite32(P_DDR0_PCTL_TEXSR,timing_set->t_pctl_texsr);
	MmioWrite32(P_DDR0_PCTL_TXP,timing_set->t_pctl_txp);

	//DDR0 PCTL
	MmioWrite32(P_DDR0_PCTL_TDQS,timing_set->t_pctl_tdqs);
	MmioWrite32(P_DDR0_PCTL_TRTW,timing_set->t_pctl_trtw);
	MmioWrite32(P_DDR0_PCTL_TCKSRE,timing_set->t_pctl_tcksre);
	MmioWrite32(P_DDR0_PCTL_TCKSRX,timing_set->t_pctl_tcksrx);
	MmioWrite32(P_DDR0_PCTL_TMOD,timing_set->t_pctl_tmod);

	MmioWrite32(P_DDR0_PCTL_TCKE,timing_set->t_pctl_tcke);
	MmioWrite32(P_DDR0_PCTL_TZQCS,timing_set->t_pctl_tzqcs);
	MmioWrite32(P_DDR0_PCTL_TZQCL,timing_set->t_pctl_tzqcl);
	MmioWrite32(P_DDR0_PCTL_TXPDLL,timing_set->t_pctl_txpdll);
	MmioWrite32(P_DDR0_PCTL_TZQCSI,timing_set->t_pctl_tzqcsi);

	MmioWrite32(P_DDR0_PCTL_SCFG,timing_set->t_pctl_scfg);

	MmioWrite32(P_DDR0_PCTL_SCTL,UPCTL_CMD_CONFIG); //DDR 0
	while (!(MmioRead32(P_DDR0_PCTL_STAT) & 1))
	{
		DDR_INIT_WHILE_LOOP(loop_whi_count);
	}

	//cfg_ddr_mode == CFG_DDR_32BIT
	
	MmioWrite32(P_DDR0_PCTL_PPCFG,(0xf0 << 1));
	

	//DDR 0 DFI
	MmioWrite32(P_DDR0_PCTL_DFISTCFG0,4);
	MmioWrite32(P_DDR0_PCTL_DFISTCFG1,1);
	MmioWrite32(P_DDR0_PCTL_DFITCTRLDELAY,2);
	MmioWrite32(P_DDR0_PCTL_DFITPHYWRDATA,1);

	//DDR 0 DFI
	nTempVal = timing_set->t_pctl_tcwl + timing_set->t_pctl_tal;
	nTempVal = (nTempVal - ((nTempVal % 2) ? 3 : 4)) / 2;
	MmioWrite32( P_DDR0_PCTL_DFITPHYWRLAT,nTempVal);

	nTempVal = timing_set->t_pctl_tcl + timing_set->t_pctl_tal;
	nTempVal = (nTempVal - ((nTempVal % 2) ? 3 : 4)) / 2;
	MmioWrite32(P_DDR0_PCTL_DFITRDDATAEN,nTempVal);

	//DDR 0 DFI
	MmioWrite32(P_DDR0_PCTL_DFITPHYRDLAT,13);
	MmioWrite32(P_DDR0_PCTL_DFITDRAMCLKDIS,1);
	MmioWrite32(P_DDR0_PCTL_DFITDRAMCLKEN,1);
	MmioWrite32(P_DDR0_PCTL_DFITCTRLUPDMIN,0x4000);
	MmioWrite32(P_DDR0_PCTL_DFILPCFG0,(1 | (3 << 4) | (1 << 8) | (3 << 12) | (7 << 16) | (1 << 24) | (3 << 28)));
	MmioWrite32(P_DDR0_PCTL_DFITPHYUPDTYPE1,0x200);

	MmioWrite32(P_DDR0_PCTL_DFIODTCFG,8);


	MmioWrite32(P_DDR0_PCTL_DFIODTCFG1,(0x0 | (0x6 << 16)));

	//CONFIG_DDR0_DTAR_ADDR//0x8/0x10/0x18
	MmioWrite32(P_DDR0_PUB_DTAR0,(0x0 + timing_set->t_pub0_dtar));
	MmioWrite32(P_DDR0_PUB_DTAR1,(0x08 + timing_set->t_pub0_dtar));
	MmioWrite32(P_DDR0_PUB_DTAR2,(0x10 + timing_set->t_pub0_dtar));
	MmioWrite32(P_DDR0_PUB_DTAR3,(0x18 + timing_set->t_pub0_dtar));

	MmioWrite32(P_DDR0_PCTL_CMDTSTATEN,1);
	while (!(MmioRead32(P_DDR0_PCTL_CMDTSTAT) & 0x1))
	{
		DDR_INIT_WHILE_LOOP(loop_whi_count);
	}

	//===============================================
	//HX PUB INIT
	//hx_serial_puts("Aml log : HX DDR PUB training begin:\n");

	//===============================================
	//PLL,DCAL,PHY RST,ZCAL
	nTempVal = PUB_PIR_ZCAL | PUB_PIR_PLLINIT | PUB_PIR_DCAL | PUB_PIR_PHYRST;
	MmioWrite32(P_DDR0_PUB_PIR,nTempVal);
	MicroSecondDelay(1);
	MmioWrite32(P_DDR0_PUB_PIR,nTempVal | PUB_PIR_INIT);

	while ((MmioRead32(P_DDR0_PUB_PGSR0) != 0x8000000f) &&
		   (MmioRead32(P_DDR0_PUB_PGSR0) != 0xC000000f))
	{
		MicroSecondDelay(10);
		if (MmioRead32(P_DDR0_PUB_PGSR0) & PUB_PGSR0_ZCERR)
		{
			//pctl_serial_puts("Aml log : DDR0 - PUB_PGSR0_ZCERR with [");
			//pctl_serial_put_hex(readl(P_DDR0_PUB_PGSR0), 32);
			//pctl_serial_puts("] retry...\n");
			DDR_INIT_ERROR_LOOP(loop_err_count);
			goto pub_init_ddr0;
		}
		DDR_INIT_WHILE_LOOP(loop_whi_count);
	}
	//hx_serial_puts("Aml log : DDR0 - PLL,DCAL,RST,ZCAL done\n");

	//===============================================
	//DRAM INIT
	nTempVal = PUB_PIR_DRAMRST | PUB_PIR_DRAMINIT;
	MmioWrite32(P_DDR0_PUB_PIR,nTempVal);
	MmioWrite32(P_DDR0_PUB_PIR,nTempVal | PUB_PIR_INIT);

	while ((MmioRead32(P_DDR0_PUB_PGSR0) != 0x8000001f) &&
		   (MmioRead32(P_DDR0_PUB_PGSR0) != 0xC000001f))
	{
		//ddr_udelay(10);
		if (MmioRead32(P_DDR0_PUB_PGSR0) & 1)
			break;
		DDR_INIT_WHILE_LOOP(loop_whi_count);
	}
	//hx_serial_puts("Aml log : DDR0 - DRAM INIT done\n");

#if !defined(CONFIG_PXP_EMULATOR)
#if (!(defined LPDDR2)) && (!(defined LPDDR3))
	//===============================================
	//WL init
	nTempVal = PUB_PIR_WL;
	MmioWrite32(P_DDR0_PUB_PIR,nTempVal);
	MmioWrite32(P_DDR0_PUB_PIR,nTempVal | PUB_PIR_INIT);

	while ((MmioRead32(P_DDR0_PUB_PGSR0) != 0x8000003f) &&
		   (MmioRead32(P_DDR0_PUB_PGSR0) != 0xC000003f))
	{
		//ddr_udelay(10);
		if (MmioRead32(P_DDR0_PUB_PGSR0) & PUB_PGSR0_WLERR)
		{
			//pctl_serial_puts("Aml log : DDR0 - PUB_PGSR0_WLERR with [");
			//pctl_serial_put_hex(readl(P_DDR0_PUB_PGSR0), 32);
			//pctl_serial_puts("] retry...\n");
			DDR_INIT_ERROR_LOOP(loop_err_count);
			goto pub_init_ddr0;
		}
		DDR_INIT_WHILE_LOOP(loop_whi_count);
	}
	//hx_serial_puts("Aml log : DDR0 - WL done\n");
#endif //LPDDR2, LPDDR3
#endif //CONFIG_PXP_EMULATOR

	//===============================================
	//DQS Gate training
#if (!(defined LPDDR2)) && (!(defined LPDDR3))
	nTempVal = PUB_PIR_QSGATE;
	MmioWrite32(P_DDR0_PUB_PIR,nTempVal);
	MmioWrite32(P_DDR0_PUB_PIR,nTempVal | PUB_PIR_INIT);


	while ((MmioRead32(P_DDR0_PUB_PGSR0) != 0x8000007f) &&
		   (MmioRead32(P_DDR0_PUB_PGSR0) != 0xC000007f))

	{
		//ddr_udelay(10);
		if (MmioRead32(P_DDR0_PUB_PGSR0) & PUB_PGSR0_QSGERR)
		{
			//pctl_serial_puts("Aml log : DDR0 - PUB_PGSR0_QSGERR with [");
			//pctl_serial_put_hex(MmioRead32(P_DDR0_PUB_PGSR0), 32);
			//pctl_serial_puts("] retry...\n");
			DDR_INIT_ERROR_LOOP(loop_err_count);
			goto pub_init_ddr0;
		}
		DDR_INIT_WHILE_LOOP(loop_whi_count);
	}
	//hx_serial_puts("Aml log : DDR0 - DQS done\n");
#endif //LPDDR2, LPDDR3

#if !defined(CONFIG_PXP_EMULATOR)
#if (!(defined LPDDR2)) && (!(defined LPDDR3))
	//===============================================
	//Write leveling ADJ
	nTempVal = PUB_PIR_WLADJ;
	writel(nTempVal, P_DDR0_PUB_PIR);
	writel(nTempVal | PUB_PIR_INIT, P_DDR0_PUB_PIR);

	while ((MmioRead32(P_DDR0_PUB_PGSR0) != 0x800000ff) &&
		   (MmioRead32(P_DDR0_PUB_PGSR0) != 0xC00000ff))
	{
		//ddr_udelay(10);
		if (MmioRead32(P_DDR0_PUB_PGSR0) & PUB_PGSR0_WLAERR)
		{
			//pctl_serial_puts("Aml log : DDR0 - PUB_PGSR0_WLAERR with [");
			//pctl_serial_put_hex(readl(P_DDR0_PUB_PGSR0), 32);
			//pctl_serial_puts("] retry...\n");
			DDR_INIT_ERROR_LOOP(loop_err_count);
			goto pub_init_ddr0;
		}
		DDR_INIT_WHILE_LOOP(loop_whi_count);
	}
	//hx_serial_puts("Aml log : DDR0 - WLADJ done\n");
#endif //LPDDR2, LPDDR3

#if (!(defined LPDDR2)) && (!(defined LPDDR3))
	//===============================================
	//Data bit deskew & data eye training
	nTempVal = PUB_PIR_RDDSKW | PUB_PIR_WRDSKW | PUB_PIR_RDEYE | PUB_PIR_WREYE;
	MmioWrite32(P_DDR0_PUB_PIR,nTempVal);
	MmioWrite32(P_DDR0_PUB_PIR,nTempVal | PUB_PIR_INIT);

	while ((MmioRead32(P_DDR0_PUB_PGSR0) != 0x80000fff) &&
		   (MmioRead32(P_DDR0_PUB_PGSR0) != 0xC0000fff))
	{
		MicroSecondDelay(1);
		if (MmioRead32(P_DDR0_PUB_PGSR0) & PUB_PGSR0_DTERR)
		{
			//pctl_serial_puts("Aml log : DDR0 - PUB_PGSR0_DTERR with [");
			//pctl_serial_put_hex(readl(P_DDR0_PUB_PGSR0), 32);
			//pctl_serial_puts("] retry...\n");
			goto pub_init_ddr0;
		}
		DDR_INIT_WHILE_LOOP(loop_whi_count);
	}
	//hx_serial_puts("Aml log : DDR0 - BIT deskew & data eye done\n");
#endif //LPDDR2, LPDDR3
#endif //CONFIG_VLSI_EMULATOR



	MmioWrite32(P_DDR0_PCTL_SCTL,UPCTL_CMD_GO);
	while ((MmioRead32(P_DDR0_PCTL_STAT) & UPCTL_STAT_MASK) != UPCTL_STAT_ACCESS)
	{
		DDR_INIT_WHILE_LOOP(loop_whi_count);
	}
	//hx_serial_puts("Aml log : DDR0 - PCTL enter GO state\n");

	nTempVal = MmioRead32(P_DDR0_PUB_PGSR0);
#ifdef CONFIG_NO_DDR_PUB_VT_CHECK
	if ((((nTempVal >> 20) & 0xfff) != 0xC00) &&
		(((nTempVal >> 20) & 0xfff) != 0x800)
#ifdef CONFIG_DDR_BYPASS_PHY_PLL
		&& (((nTempVal >> 20) & 0xfff) != 0x400) && (((nTempVal >> 20) & 0xfff) != 0x000)
#endif
	)
#else
	if ((((nTempVal >> 20) & 0xfff) != 0xC00)
#ifdef CONFIG_DDR_BYPASS_PHY_PLL
		&& (((nTempVal >> 20) & 0xfff) != 0x400)
#endif
	)
#endif
	{
		//pctl_serial_puts("Aml log : DDR0 - PUB init fail with PGSR0 : 0x");
		//pctl_serial_put_hex(nTempVal, 32);
		//pctl_serial_puts("Aml log : Try again with MMC reset...\n\n");
		DDR_INIT_ERROR_LOOP(loop_err_count);
		goto mmc_init_all;
	}
	else
	{
		//hx_serial_puts("Aml log : DDR0 - init pass with  PGSR0 : 0x");
		//hx_serial_put_hex(readl(P_DDR0_PUB_PGSR0), 32);
		//hx_serial_puts("\n");
		//serial_puts("\n  PGSR1 : 0x");
		//serial_put_hex(readl(P_DDR0_PUB_PGSR1),32);
		//serial_puts("\n");
	}


	nRet = 0;


	MmioWrite32(P_DDR0_PUB_PGCR3,(0x7f << 9) | (MmioRead32(P_DDR0_PUB_PGCR3)));


	MmioWrite32(P_DDR0_PUB_ZQCR,MmioRead32(P_DDR0_PUB_ZQCR) | (0x4));

	MicroSecondDelay(1);



	MmioWrite32(P_DDR0_CLK_CTRL,MmioRead32(P_DDR0_CLK_CTRL) & (~1));

	MicroSecondDelay(1);
	return nRet;
}

static struct ddr_set __ddr_setting={
	.ddr_test		= 0x3,	//full define in ddr.c DDR_TEST_BASEIC = 0x3, DDR_TEST_ALL = 0x7
	.phy_memory_start	= PHYS_MEMORY_START,
	.phy_memory_size	= PHYS_MEMORY_SIZE,
	.t_pub0_dtar	= ((0x0 + CONFIG_DDR0_DTAR_DTCOL)|(CONFIG_DDR0_DTAR_DTROW <<12)|(CONFIG_DDR0_DTAR_DTBANK << 28)),
	.t_pub1_dtar	= 0,

	.t_ddr_apd_ctrl = ( 32 << 0 ) | //[B7..B0]: DMC active latency.  latency to enter LOW POWER state after the the DMC is not actived.
					  ( 6  << 8 ) | //[B15..B8]: Low Power enter latency.when the logic check the PCTL send LP_REQ and ACKed by PUB. after this regsiter bits cycles, we'll gated the PCTL clock and PUB clock if the pub and pctl auto clock gating enabled.
					  ( 0  << 16), 	//[B17,B16]: DDR mode to genreated the refresh ack signal and prcharge signal to DMC for dmc refresh control.
					  			    // 2'b00: DDR3  2'b01.  LPDDR2/3. 2'b10, 2'b11: no refresh ack and precharge signals generated. 
					  			    
	.t_ddr_clk_ctrl = ( 1  << 0 ) | //[B0]: DDR0 PCTL PCLK enable.	before access PCTL regsiter, we need enable PCTL PCLK 24 cycles early.
					    		    //after access PCTL regsiter, we can disable PCTL PCTLK 24 cycles later.
					  ( 1  << 1 ) | //[B1]: DDR0 PUB PCLK auto clock gating enable.
					  ( 0  << 2 ) | //[B2]: Disable DDR0 PUB PCLK clock.
					  ( 1  << 3 ) | //[B3]: DDR0 PCTL n_clk auto clock gating enable.
					  ( 0  << 4 ) | //[B4]: DDR0 PCTL n_clk disable.
					  ( 1  << 5 ) | //[B5]: DDR0 PUB n_clk auto clock gating enable.  (may not be work).
					  ( 0  << 6 ) | //[B6]: DDR0 PUB n_clk disable.
					  ( 0  << 7 ) | //[B7]: NOT used. 
					  ( 1  << 8 ) | //[B8]: DDR0 PHY ctl_clk enable.  
					  ( 0  << 9 ),  //[B9]: DDR0 PHY RF mode.  we can invert the clock to DDR PHY by setting this bit to 1.

	.t_pctl_1us_pck		=  CFG_DDR_CLK/2,
	.t_pctl_100ns_pck	=  CFG_DDR_CLK/20,

#if defined (CONFIG_VLSI_EMULATOR)
	.t_pctl_init_us		=  2,
	.t_pctl_rsth_us		=  2,
	.t_pctl_rstl_us		=  0,
#else
	.t_pctl_init_us		=  512,
	.t_pctl_rsth_us		=  500,
	.t_pctl_rstl_us		=  100,
#endif //#if defined (CONFIG_VLSI_EMULATOR)

	.t_pctl_mcfg =   1 |	//[B0] mem-bl: 0 for 4; 1 for 8
			  (0 << 2) |	//[B2] bl8int_en.   enable bl8 interrupt function.Only valid for DDR2
			  				// and is ignored for mDDR/LPDDR2 and DDR3
	          (1 << 5) |    //[B5] ddr3en: 1: ddr3 protocal; 0 : ddr2 protocal
	          (1 << 3) |    //[B3]two_t_en: DDR 2T mode, default is disable
	          (((((CFG_DDR_FAW+CFG_DDR_RRD-1)/CFG_DDR_RRD)-4)&0x3)<<18) | //[B19,B18] tfaw_cfg
	                          	   //tFAW= (4 + MCFG.tfaw_cfg)*tRRD - MCFG1.tfaw_cfg_offset
	                          	   // 0:tFAW=4*tRRD - mcfg1.tfaw_cfg_offset 
	                          	   // 1:tFAW=5*tRRD - mcfg1.tfaw_cfg_offset 
	                          	   // 2:tFAW=6*tRRD - mcfg1.tfaw_cfg_offset
	          (1 << 17) |    //[B17]pd_exit_mode: 0: slow exit; 1: fast exit. power down exit						
	          (0 << 16) |    //[B16]pd_type: 0: precharge power down, 1 active power down
		      (0x2f << 8)    //[B15-B8]pd_idle: power-down idle in n_clk cycles. 15 cycles empty will entry power down mode.
		      ,
	.t_pctl_mcfg1 = (1<<31)| //[B31]hw_exit_idle_en
			  (((CFG_DDR_FAW%CFG_DDR_RRD)?(CFG_DDR_RRD-(CFG_DDR_FAW%CFG_DDR_RRD)):0)&0x7)<<8 //[B10,B9,B8] tfaw_cfg_offset:
			  //tFAW= (4 + MCFG.tfaw_cfg)*tRRD - tfaw_cfg_offset
			  ,

	.t_pub_zq0pr = 0x19,
	.t_pub_dxccr = 4|(0xf7<<5)|(3<<19),

	//.t_pub_acbdlr0 = 0,
	.t_pub_acbdlr0 = 0x14,
	
	.t_pub_dcr = 0x8b,


	.t_pub_mr={
		[0]=(1 << 12) |   //[B12] 1 fast exit from power down , 0 slow 
			((((CFG_DDR_WR <=8) ? (CFG_DDR_WR - 4) : (CFG_DDR_WR>>1)) & 7) <<  9) | //[B11,B10,B9]WR recovery. It will be calcualted by get_mrs0()@ddr_init_pctl.c 
			(0 <<  8) |   //[B8]DLL reset
			(0 <<  7) |   //[B7]0= Normal 1=Test.
			(((CFG_DDR_CL - 4) & 0x7) <<  4) |   //[B6,B5,B4]CL cas latency high 3 bits (B6,B5,B4).
			(((CFG_DDR_CL - 4) & 0x8) >> 1 ) |   //[B2]CL bit 0
			(0 << 3 ) |   //[B3]burst type,  0:sequential; 1:Interleave.
			(0 << 0 ),    //[B1,B0]burst length	:  00: fixed BL8; 01: 4 or 8 on the fly; 10:fixed BL4; 11: reserved
				                    						      
       	[1]=(0 << 9)|(0<< 6)|(1 << 2)|	//RTT (B9,B6,B2) 000 ODT disable;001:RZQ/4= 60;010: RZQ/2;011:RZQ/6;100:RZQ/12;101:RZQ/8
           	(0 << 5)|(0 << 1) |			//DIC(B5,B1) 00: Reserved for RZQ/6; 01:RZQ/7= 34;10,11 Reserved
		#ifdef CONFIG_ENABLE_WRITE_LEVELING
	            (1 << 7)|     // Write leveling enable
		#endif
	            ((CFG_DDR_AL ? ((CFG_DDR_CL - CFG_DDR_AL)&3): 0) << 3 ),//[B4,B3] AL:
	            			  //00: AL disabled; 01:CL-1;10:CL-2;11:reserved

		//#ifdef CONFIG_ENABLE_WRITE_LEVELING //jiaxing delete
	   	//[2]=0,
		//#else
	   	[2]=(0 << 10)|(1 <<9)|//[B10,B9]TRRWR: 00:Dynamic ODT off , 01:Rzq/4, 10:Rzq/2								
	   		(((CFG_DDR_CWL-5)&0x7)<<3), //[B5,B4,B3] CWL: 
					//000 = 5 (tCK ? 2.5ns)
	        		//001 = 6 (2.5ns > tCK * 1.875ns)
	        		//010 = 7 (1.875ns > tCK * 1.5ns)
    				//011 = 8 (1.5ns > tCK * 1.25ns)
    				//100 = 9 (1.25ns > tCK * 1.07ns)
    				//101 = 10 (1.07ns > tCK * 0.935ns)
    				//110 = 11 (0.935ns > tCK * 0.833ns)
    				//111 = 12 (0.833ns > tCK * 0.75ns)
		//#endif
	   	[3]=0,
	},

	.t_pub_dtpr={
		[0] =  ((CFG_DDR_RTP << 0  )|       //tRTP       //4 TCK,7500ps
				(CFG_DDR_WTR << 4  )|       //tWTR       //4 TCK,7500ps
				(CFG_DDR_RP  << 8  )|       //tRP        //12500ps
				(CFG_DDR_RCD << 12 )|       //tRCD       //12500ps
				(CFG_DDR_RAS << 16 )|       //tRAS       //35000ps
				(CFG_DDR_RRD << 22 )|       //tRRD       //7500ps
				(CFG_DDR_RC  << 26)),
		
		[1] =  ((CFG_DDR_MRD << 0 ) |		//tMRD      //4 TCK
				((CFG_DDR_MOD - 12) << 2 ) | //tMOD      //0: 12 TCK
				(CFG_DDR_FAW  << 5 )  |      //tFAW      //40000ps
				(CFG_DDR_RFC  << 11)  |      //tRFC      //160000~70312500
				(CFG_DDR_WLMRD << 20) |      //tWLMRD    //40 TCK
				(CFG_DDR_WLO   << 26)),      //tWLO      //7500ps

		[2] =  ((CFG_DDR_XS  << 0 ) |    		//tXS        //MAX(tXS,tXSDLL),tXS=170000ps,tXSDLL=512 TCK
				(CFG_DDR_XP  << 10) |    //tXP        //tXP=6000 tXPDLL=24000
				(CFG_DDR_CKE << 15) |    //tCKE       //tCKE=5000ps,tCKE_TCK=3,DDR3,set to tCKESR
				(CFG_DDR_DLLK  << 19)|   //tDLLK      //512
				(CFG_DDR_RTODT << 29)|    //tRTODT
				(1   << 30)|    //tRTW //  CFG_DDR_RTW //jiaxing add
				(0 << 31)),

		[3] = 0,
	},

	.t_pub_ptr={
		[0] =	( 6 |	         //PHY RESET APB clock cycles.
				(320 << 6) |	 //tPLLGS    APB clock cycles. 4us
				(80 << 21)),     //tPLLPD    APB clock cycles. 1us.
				
		[1] =   ( 120 |	         //tPLLRST   APB clock cycles. 9us 
				(1000 << 16)),   //TPLLLOCK  APB clock cycles. 100us. ,
				
		[2] = 0,
		[3] = (20000   |       //tDINIT0  CKE low time with power and lock stable. 500us.
				(186 << 20)),  //tDINIT1. CKE high time to first command(tRFC + 10ns). //jiaxing add
		[4] = (10000 |         //tDINIT2. RESET low time,  200us. 
				(800 << 18)),   //tDINIT3. ZQ initialization command to first command. 1us., //jiaxing add
					
	},

	.t_pctl_trefi  =  CFG_DDR_REFI,
	.t_pctl_trefi_mem_ddr3 = CFG_DDR_REFI_MDDR3,
	.t_pctl_tmrd   = CFG_DDR_MRD,
	.t_pctl_trfc   = CFG_DDR_RFC,
	.t_pctl_trp    = CFG_DDR_RP,
	.t_pctl_tal    = CFG_DDR_AL,
	.t_pctl_tcwl   = CFG_DDR_CWL,
	.t_pctl_tcl    = CFG_DDR_CL,
	.t_pctl_tras   = CFG_DDR_RAS,
	.t_pctl_trc    = CFG_DDR_RC,
	.t_pctl_trcd   = CFG_DDR_RCD,
	.t_pctl_trrd   = CFG_DDR_RRD,
	.t_pctl_trtp   = CFG_DDR_RTP,
	.t_pctl_twr    = CFG_DDR_WR,
	.t_pctl_twtr   = CFG_DDR_WTR,
	.t_pctl_texsr  = CFG_DDR_EXSR,
	.t_pctl_txp    = CFG_DDR_XP,
	.t_pctl_tdqs   = CFG_DDR_DQS,
	.t_pctl_trtw   = CFG_DDR_RTW,
	.t_pctl_tcksre = CFG_DDR_CKSRE,
	.t_pctl_tcksrx = CFG_DDR_CKSRX,
	.t_pctl_tmod   = CFG_DDR_MOD,
	.t_pctl_tcke   = CFG_DDR_CKE,
	.t_pctl_tzqcs  = CFG_DDR_ZQCS,
	.t_pctl_tzqcl  = CFG_DDR_ZQCL,
	.t_pctl_txpdll = CFG_DDR_XPDLL,
	.t_pctl_tzqcsi = CFG_DDR_ZQCSI,
	.t_pctl_scfg   = 0xf01,

	.t_ddr_pll_cntl= (CFG_PLL_OD << 16)|(CFG_PLL_N<<9)|(CFG_PLL_M<<0),
	.t_ddr_clk= CFG_DDR_CLK/2, //DDR PLL output is half of target DDR clock
	
	.t_mmc_ddr_ctrl=(1<<16)	| //bit 16. bank page policy.	
					(CONFIG_M8B_DDR_BIT_MODE_SET<<15) | //bit 15. rank1 bit mode.  0 : 32bits mode. 1: 16bits mode.
					(CONFIG_M8B_DDR_BANK_SET<<13)     | //bit 14:13. rank1 address map bank mode
                                                        // 00 = address switch between 2 banks  bank[0] selection bits [12].    
                                                        // 01 = address switch between 4 banks  bank[1:0] selection bits [13:12]. 
                                                        // 10 = address switch between 2 banks  bank[0] selection bits [8].    
                                                        // 11 = address switch between 4 banks  bank[1:0] selection bits [9:8]. 
					(CONFIG_M8B_DDR_RANK_SET<<12) | //bit 12.  0: one rank.  1 : two ranks.
					(CONFIG_DDR3_ROW_SIZE<<10)    | //bit 11:10. rank1 row size.  2'b01 : A0~A12. 2'b10 : A0~A13.  2'b11 : A0~A14.  2'b00 : A0~A15.
					(CONFIG_DDR3_COL_SIZE<<8)     | //bit 9:8. rank1 col size. 2'b01 : A0~A8, 2'b10 : A0~A9.
					(CONFIG_M8B_DDR_BIT_MODE_SET<<7)| //bit 7. ddr rank0 bit mode.  0: 32bits mode. 1 : 16bits mode.
					(CONFIG_M8B_DDR_BANK_SET<<5) | //bit 6:5.     rank0 address map bank mode
                                                   // 00 = address switch between 2 banks  bank[0] selection bits [12].    
                                                   // 01 = address switch between 4 banks  bank[1:0] selection bits [13:12]. 
                                                   // 10 = address switch between 2 banks  bank[0] selection bits [8].    
                                                   // 11 = address switch between 4 banks  bank[1:0] selection bits [9:8]. 
					(CONFIG_M8B_DDR_RANK_SET<<4)  | //bit 4.  0: one rank.  1 : two ranks.
					(CONFIG_DDR3_ROW_SIZE<<2)     | //bit 3:2. rank0 row size.  2'b01 : A0~A12.   2'b10 : A0~A13.  2'b11 : A0~A14.  2'b00 : A0~A15. 
					(CONFIG_DDR3_COL_SIZE<<0),      //bit 1:0. rank0 col size.  2'b01 : A0~A8,    2'b10 : A0~A9.
	//All following t_mmc_ddr_timmingx unit is half of DDR timming setting
	//e.g t_mmc_ddr_timming0.tCWL = (CFG_DDR_CWL+1)/2-1;
	.t_mmc_ddr_timming0 = (((CFG_DDR_CWL+1)/2-1) << 28 )| // DDR timing register is used for ddr command filter to generate better performace
						  			  // In M6TV we only support HDR mode, that means this timing counnter is half of the real DDR clock cycles.
						              // bit 31:28.  tCWL
					      (((CFG_DDR_RP+1)/2-1)  << 24 )| // bit 27:24.  tRP
						  (((CFG_DDR_RTP+1)/2-1) << 20 )| // bit 23:20.  tRTP
						  (((CFG_DDR_RTW+1)/2-1) << 16 )| // bit 19:16.  tRTW
						  ( 4  << 12 )| // bit 15:12.  tCCD //where is the tCCD, not found in PUB/PCTL spec, 4 from ucode
						  (((CFG_DDR_RCD+1)/2-1) <<  8 )| // bit 11:8.   tRCD
						  (((CFG_DDR_CL+1)/2-1)  <<  4 )| // bit 7:4.    tCL
						  ( 3 <<  0 ) , 					  // bit 3:0.    Burst length.   
						
	.t_mmc_ddr_timming1 = (((CFG_DDR_CL+CFG_DDR_RCD+CFG_DDR_RP+1)/2-1)  << 24 )|  //bit 31:24.  tMISS. the waiting clock cycles for a miss page access ( the same bank different page is actived. = tCL + tRCD + tRP).
						  ( 6 << 20 )| 						   //bit 23:20.  cmd hold number for read command. 
						  ( 0 << 16 )| 						   //bit 19:16.  cmd hold number for write command. 
						  (((CFG_DDR_RFC+1)/2-1) <<  8 )|  //bit 15:8.   tRFC.  refresh to other command time.
						  (((CFG_DDR_WTR+1)/2-1) <<  4 )|  //bit 7:4.    tWTR.
						  (((CFG_DDR_WR+1)/2-1)  <<  0 ) , //bit 3:0.    tWR
						  
	.t_mmc_ddr_timming2 = (((CFG_DDR_ZQCL+1)/2-1) << 24 )|  //bit 31:24  tZQCL
						  ( 16 << 16 )| 				        //bit 23:16  tPVTI
						  (((CFG_DDR_REFI+1)/2-1) <<  8 )|  //bit 15:8    tREFI
						  (((CFG_DDR_CLK/10+1)/2 -1) <<  0 ) ,//bit 7:0  t100ns,
						  
	.t_mmc_arefr_ctrl = ( 0 << 9 ) | //bit 9      hold dmc command after refresh cmd sent to DDR3 SDRAM.
						( 0 << 8 ) | //bit 8      hold dmc command when dmc need to send refresh command to PCTL.
						( 1 << 7 ) | //bit 7      dmc to control auto_refresh enable
						( 0 << 4 ) | //bit 6:4    refresh number per refresh cycle..
						( 1 << 3 ) | //bit 3      pvt enable
						( 1 << 2 ) | //bit 2      zqc enable
						( 1 << 1 ) | //bit 1      ddr1 auto refresh dmc control select.
						( 1 << 0 ),  //bit 0      ddr0 auto refresh dmc control select.,
	.init_pctl=init_pctl_ddr3
};

#define AM_DDR_PLL_CNTL     0xc8000400
#define AM_DDR_PLL_CNTL1    0xc8000404
#define AM_DDR_PLL_CNTL2    0xc8000408
#define AM_DDR_PLL_CNTL3    0xc800040c
#define AM_DDR_PLL_CNTL4    0xc8000410
#define AM_DDR_PLL_STS      0xc8000414
  //bit 31. DDR_PLL lock.
  // bit 6:0.  DDR_PLL analog output for test. 

#define P_DDR_CLK_CNTL     0xc8000418
  //bit 31     ddr_pll_clk enable. enable the clock from DDR_PLL to clock generateion. 
  // whenever change the DDR_PLL frequency, disable the clock, after the DDR_PLL locked, then enable it again. 
  //bit 30.    ddr_pll_prod_test_en.  enable the clock to clock/32 which to clock frequency measurement and production test pin.
  //bit 29.    clock setting update. becasue the register is in PCLK domain. use this pin to update the clock divider setting.
  //bit 28.    clock generation logic soft reset. 0 = reset.
  //bit 15:8   second level divider control.
  //bit 15.    second level divider clock selection. 0 : from first stage clock divider. 1: from second stage clock divider.
  //bit 14.    enable the  first stage clock output to the second stage clock output. 
  //bit 13.    second stage clock counter enable.
  //bit 12.    enable the second stage divider clock ouput. 
  //bit 11.    enable the 4xclk to DDR PHY. 
  //bit 8.     second stage divider selection. 0: /2.  1: /4.
  //bit 7:0.   first stage clock control.   
  //bit 7.     first stage clock selection. 0 : DDR_PLL clock output.  1: divider.
  //bit 6.     pll_clk_en.  enable the pll clock output to the first stage clock output selection.
  //bit 3.     first stage clock counter enable.
  //bit 2.     first stage clock divider output enable.
  //bit 1:0.   00:  /2.  01: /4. 10: /8. 11: /16.
  
#define P_DDR_CLK_STS      0xc800041c
  //not used.
  

#define  P_DDR0_CLK_CTRL    0xc8000800
//bit 9. invert the DDR PHY n_clk.   ( RF mode).
//bit 8. disable the DDR PHY n_clk.
//bit 7. not used. 
//bit 6. force to disable PUB n_clk. 
//bit 5. PUB auto clock gating enable. when the IP detected PCTL enter power down mode, use this bit to gating PUB n_clk.
//bit 4. force to disable PCTL n_clk.
//bit 3.  PCTL n_clk auto clock gating enable.  when the IP detected PCTL enter power down mode, use this bit to gating pctl n_clk.
//bit 2. force to disable PUB PCLK. 
//bit 1. PUB pclk auto clock gating enable.  when the IP detected PCTL enter power down mode,  use this bit to gating pub pclk. 
//bit 0   PCTL PCLK enable.   When never configure PCTL register, we need to enable the APB clock 24 cycles earlier. after finished configure PCTL, 24 cycles later, we should disable this bit to save power.
#define  P_DDR0_SOFT_RESET  0xc8000804
//bit 3. pub n_clk domain soft reset.  1 active.
//bit 2. pub p_clk domain soft reset.
//bit 1. pctl n_clk domain soft reset.
//bit 0. pctl p_clk domain soft reset.
#define  P_DDR0_APD_CTRL    0xc8000808
//bit 15:8.   power down enter latency. when IP checked the dfi_lp_req && dfi_lp_ack,  give PCTL and pub additional latency let them settle down, then gating the clock.
//bit 7:0.  no active latency.  after c_active_in become to low, wait additional latency to check the pctl low power state.  

//DDR PLL > 1G
#define CFG_DDR_PLL_CNTL_2 (0x59C88000)
#define CFG_DDR_PLL_CNTL_3 (0xCA463823)
#define CFG_DDR_PLL_CNTL_4 (0x0286A027)
#define CFG_DDR_PLL_CNTL_5 (0x00003800)
#define  P_DDR1_CLK_CTRL    0xc8002800
#define  P_DDR1_SOFT_RESET  0xc8002804
#define  P_DDR1_APD_CTRL    0xc8002808

#define DMC_REG_BASE       0xc8006000
#define P_DMC_SOFT_RST         DMC_REG_BASE + (0x01 << 2)
#define P_DMC_SOFT_RST1        DMC_REG_BASE + (0x02 << 2)
#define P_DMC_RST_STS          DMC_REG_BASE + (0x03 << 2)
#define P_DMC_RST_STS1         DMC_REG_BASE + (0x04 << 2)
#define P_DMC_VERSION          DMC_REG_BASE + (0x05 << 2)

#define PL310_ST_ADDR	 (volatile unsigned *)0xc4200c00
#define PL310_END_ADDR	(volatile unsigned *)0xc4200c04

VOID
 set_ddr_clock(
   struct ddr_set * timing_reg
   )
{
	int n_pll_try_times = 0;
		
	do {
		//BANDGAP reset for SYS_PLL,MPLL lock fail 
		DEBUG ((EFI_D_INFO, "BANDGAP reset for SYS_PLL,MPLL lock fail\n"));
		MmioWrite32(0xc8000410,MmioRead32(0xc8000410)& (~(1<<12)));
		MmioWrite32(0xc8000410,MmioRead32(0xc8000410)|(1<<12));
		
		MmioWrite32(AM_DDR_PLL_CNTL,(1<<29)); 
		MmioWrite32(AM_DDR_PLL_CNTL1,CFG_DDR_PLL_CNTL_2);
		MmioWrite32(AM_DDR_PLL_CNTL2,CFG_DDR_PLL_CNTL_3);
		MmioWrite32(AM_DDR_PLL_CNTL3,CFG_DDR_PLL_CNTL_4);
		MmioWrite32(AM_DDR_PLL_CNTL4,CFG_DDR_PLL_CNTL_5);
		

		MmioWrite32(AM_DDR_PLL_CNTL,timing_reg->t_ddr_pll_cntl|(1<<29));
		MmioWrite32(AM_DDR_PLL_CNTL,MmioRead32(AM_DDR_PLL_CNTL) & (~(1<<29)));

		PLL_LOCK_CHECK(n_pll_try_times,3);
		DEBUG ((EFI_D_INFO, "PLL_LOCK_CHECK\n"));
	} while((MmioRead32(AM_DDR_PLL_CNTL)&(1<<31))==0);
	DEBUG ((EFI_D_INFO, "DDR Clock initial...\n"));
	MmioWrite32(P_DDR0_SOFT_RESET,0);
	MmioWrite32(P_DDR1_SOFT_RESET,0);
#ifdef CONFIG_DDR_BYPASS_PHY_PLL
	MMC_Wr(P_DDR_CLK_CNTL, 0x80000040);
	MMC_Wr(P_DDR_CLK_CNTL, 0x90000040);
	MMC_Wr(P_DDR_CLK_CNTL, 0xb0000040);
	MMC_Wr(P_DDR_CLK_CNTL, 0x9000a940);
	MMC_Wr(P_DDR_CLK_CNTL, 0xb000a940);
	MMC_Wr(P_DDR_CLK_CNTL, 0x9000b940);
	MMC_Wr(P_DDR_CLK_CNTL, 0xb000b940);
#else
	MMC_Wr(P_DDR_CLK_CNTL, 0x80004040);   // enable DDR PLL CLOCK.
	MMC_Wr(P_DDR_CLK_CNTL, 0x90004040);   // come out of reset.
	MMC_Wr(P_DDR_CLK_CNTL, 0xb0004040);
	MMC_Wr(P_DDR_CLK_CNTL, 0x90004040);
#endif
	//M8 still need keep MMC in reset mode for power saving?
	//relese MMC from reset mode
	MmioWrite32(P_DMC_SOFT_RST,0xffffffff);
	MmioWrite32(P_DMC_SOFT_RST1,0xffffffff);
	//delay_us(100);//No delay need.
}

VOID
 init_dmc(
   struct ddr_set * timing_set
   )
{
	MmioWrite32(P_DMC_SOFT_RST,0xffffffff);
	MmioWrite32(P_DMC_SOFT_RST1,0xffffffff);
	MmioWrite32(P_DMC_REFR_CTRL2,0x20109a27);
	MmioWrite32(P_DMC_REFR_CTRL1,0x80389d);


	MmioWrite32(timing_set->t_mmc_ddr_ctrl, P_DMC_DDR_CTRL);

	MmioWrite32(DMC_SEC_RANGE0_CTRL,0xffff0000);
	MmioWrite32(DMC_SEC_RANGE1_CTRL,0xffffffff);
	MmioWrite32(DMC_SEC_RANGE2_CTRL,0xffffffff);
	MmioWrite32(DMC_SEC_RANGE3_CTRL,0xffffffff);
	MmioWrite32(DMC_SEC_AXI_PORT_CTRL,0xffffffff);
	MmioWrite32(DMC_SEC_AM_PORT_CTRL,0xffffffff);
	MmioWrite32(DMC_DEV_RANGE_CTRL,0xffffffff);
	MmioWrite32(DMC_DEV_RANGE_CTRL1,0xffffffff);
	MmioWrite32(DMC_SEC_CTRL,0x80000000);
	//writel(0x12a, P_DDR0_CLK_CTRL);
	MmioWrite32(P_DMC_REQ_CTRL,0xffff);

	//change PL310 address filtering to allow DRAM reads to go to M1
	MmioWrite32( 0xc4200c04,0xc0000000);
	MmioWrite32(0xc4200c00,0x00000001);
	DEBUG ((EFI_D_INFO, "put some code here to try to stop bus traffic End\n"));
	//put some code here to try to stop bus traffic
	asm("NOP");
	asm("DMB");
	asm("ISB");
}

INT8
 ddr_init_hw(
   struct ddr_set * timing_set
   )
{
	int ret = 0;
	ret = timing_set->init_pctl(timing_set);
	DEBUG ((EFI_D_INFO, "init_pctl End\n"));
	if(ret)
	{
		//serial_puts("\nPUB init fail! Reset...\n");

		MicroSecondDelay(10);
		//AML_WATCH_DOG_START();
	}

	//asm volatile("wfi");
	//while(init_dmc(timing_set);){}
	init_dmc(timing_set);
	DEBUG ((EFI_D_INFO, "init_dmc End\n"));
	return 0;
}
VOID
MemoryInit (
  VOID
  )
{
  int result;

	set_ddr_clock(&__ddr_setting);
	DEBUG ((EFI_D_INFO, "set_ddr_clock End\n"));
	result = ddr_init_hw(&__ddr_setting);
	DEBUG ((EFI_D_INFO, "ddr_init_hw End\n"));
	// assign DDR Memory Space
	if(result == 0) {
		//serial_puts("Assign DDR Memory Space\n");
		*PL310_END_ADDR = 0xc0000000;
		*PL310_ST_ADDR  = 0x00000001;
	}

}
/**
  Return the current Boot Mode

  This function returns the boot reason on the platform

**/
EFI_BOOT_MODE
ArmPlatformGetBootMode (
  VOID
  )
{
  return BOOT_WITH_FULL_CONFIGURATION;
}

/**
  Initialize controllers that must setup in the normal world

  This function is called by the ArmPlatformPkg/PrePi or ArmPlatformPkg/PlatformPei
  in the PEI phase.

**/
RETURN_STATUS
ArmPlatformInitialize (
  IN  UINTN                     MpId
  )
{
  MmioWrite32(0xc1109900,0);
  MmioWrite32(0xc1109904,0);
  TimerConstructor ();
  DEBUG ((EFI_D_INFO, "TimerConstructor End\n"));
  ClockInit ();
  DEBUG ((EFI_D_INFO, "ClockInit End\n"));
  MmioWrite32(0xc8100024,0xbfff3ffb); //red
  UINT32 nPLL = MmioRead32(0xc1104300);
  UINT32 nA9CLK = ((24 / ((nPLL>>9)& 0x1F) ) * (nPLL & 0x1FF))/ (1<<((nPLL>>16) & 0x3));
  DEBUG((DEBUG_INFO,"CPU clock is %d Mhz\n",nA9CLK));
  
  /* Disable WatchDog */
  MmioWrite32(0xc4200108,0x00000222);
  MmioWrite32(0xc420010c,0x00000222);

  /* Reset USB OTG */
  #define P_RESET1_REGISTER 0xc1104408
  MmioWrite32(P_RESET1_REGISTER,(1 << 2));
  
  /* Print CPUID */
  DEBUG((DEBUG_INFO,"Cpu Id is %x\n",MmioRead32(0xc11081a8)));
  
  return RETURN_SUCCESS;
}

EFI_STATUS
PrePeiCoreGetMpCoreInfo (
  OUT UINTN                   *CoreCount,
  OUT ARM_CORE_INFO           **ArmCoreTable
  )
{
  if (ArmIsMpCore()) {
    *CoreCount    = sizeof(mArmPlatformNullMpCoreInfoTable) / sizeof(ARM_CORE_INFO);
    *ArmCoreTable = mArmPlatformNullMpCoreInfoTable;
    return EFI_SUCCESS;
  } else {
    return EFI_UNSUPPORTED;
  }
}

ARM_MP_CORE_INFO_PPI mMpCoreInfoPpi = { PrePeiCoreGetMpCoreInfo };

EFI_PEI_PPI_DESCRIPTOR      gPlatformPpiTable[] = {
  {
    EFI_PEI_PPI_DESCRIPTOR_PPI,
    &gArmMpCoreInfoPpiGuid,
    &mMpCoreInfoPpi
  }
};

VOID
ArmPlatformGetPlatformPpiList (
  OUT UINTN                   *PpiListSize,
  OUT EFI_PEI_PPI_DESCRIPTOR  **PpiList
  )
{
  if (ArmIsMpCore()) {
    *PpiListSize = sizeof(gPlatformPpiTable);
    *PpiList = gPlatformPpiTable;
  } else {
    *PpiListSize = 0;
    *PpiList = NULL;
  }
}

