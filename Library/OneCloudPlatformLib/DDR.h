#define DDR_INIT_WHILE_LOOP_MAX 1000 //all while loop share this number
#define DDR_INIT_WHILE_LOOP(counter)                  \
	NanoSecondDelay(1000);                                      \
	counter++;                                        \
	if (counter > DDR_INIT_WHILE_LOOP_MAX)            \
	{                                                 \
		counter = 0;                                  \
		return -2;                                    \
	}                                                 \
	else                                              \
		continue;

#define DDR_INIT_ERR_LOOP_MAX 10 //all error loop share this number
#define DDR_INIT_ERROR_LOOP(counter)                \
	NanoSecondDelay(1000);                                    \
	counter++;                                      \
	if (counter > DDR_INIT_ERR_LOOP_MAX)            \
	{                                               \
		counter = 0;                                \
		return -1;                                  \
	}


struct ddr_set{
	unsigned       ddr_test;
	unsigned       phy_memory_start;
	unsigned       phy_memory_size;
	unsigned       t_pub0_dtar;
	unsigned       t_pub1_dtar;
	unsigned       t_ddr_apd_ctrl;	
	unsigned       t_ddr_clk_ctrl;	
	
	unsigned short t_pctl_1us_pck;   //PCTL TOGCNT1U
	unsigned short t_pctl_100ns_pck; //PCTL TOGCNT100N
	unsigned short t_pctl_init_us;   //PCTL TINIT
	unsigned short t_pctl_rsth_us;   //PCTL TRSTH
	unsigned short t_pctl_rstl_us;   //PCTL TRSTL

	unsigned short t_pad1;        //padding for 4 bytes alignment
	unsigned       t_pctl_mcfg;   //PCTL MCFG
	unsigned       t_pctl_mcfg1;  //PCTL MCFG1

	unsigned       t_pub_zq0pr;	  //PUB ZQ0PR
	unsigned       t_pub_dxccr;	  //PUB DXCCR

	unsigned       t_pub_acbdlr0; //PUB ACBDLR0

	unsigned       t_pub_dcr;     //PUB DCR
	unsigned short t_pub_mr[4];   //PUB MR0-3		
	unsigned       t_pub_dtpr[4]; //PUB DTPR0-3
	unsigned       t_pub_pgcr2;   //PUB PGCR2
	unsigned       t_pub_dtcr;    //PUB DTCR
	unsigned       t_pub_ptr[5];  //PUB PTR0-3	
	unsigned       t_pub_aciocr;  //PUB ACIOCR		
	unsigned       t_pub_dsgcr;   //PUB DSGCR

	unsigned short t_pctl_trefi;  //PCTL TREFI
	unsigned short t_pctl_trefi_mem_ddr3; //PCTL TREFI MEM DDR3
	unsigned short t_pctl_tmrd;   //PCTL TMRD 2..4
	unsigned short t_pctl_trfc;   //PCTL TRFC 36..374
	unsigned short t_pctl_trp;    //PCTL TRP  0
	unsigned short t_pctl_tal;    //PCTL TAL 0,CL-1,CL-2
	unsigned short t_pctl_tcwl;   //PCTL TCWL 
	unsigned short t_pctl_tcl;    //PCTL TCL
	unsigned short t_pctl_tras;   //PCTL TRAS 15..38
	unsigned short t_pctl_trc;    //PCTL TRC   20..52
	unsigned short t_pctl_trcd;   //PCTL TRCD 5..14
	unsigned short t_pctl_trrd;   //PCTL TRRD 4..8
	unsigned short t_pctl_trtp;   //PCTL TRTP 3..8
	unsigned short t_pctl_twr;    //PCTL TWR  6..16
	unsigned short t_pctl_twtr;   //PCTL TWTR 3..8
	unsigned short t_pctl_texsr;  //PCTL TEXSR 512
	unsigned short t_pctl_txp;    //PCTL TXP  1..7
	unsigned short t_pctl_tdqs;   //PCTL TDQS 1..12
	unsigned short t_pctl_trtw;   //PCTL TRTW 2..10
	unsigned short t_pctl_tcksre; //PCTL TCKSRE 5..15
	unsigned short t_pctl_tcksrx; //PCTL TCKSRX 5..15
	unsigned short t_pctl_tmod;   //PCTL TMOD 0..31
	unsigned short t_pctl_tcke;   //PCTL TCKE 3..6
	unsigned short t_pctl_tzqcs;  //PCTL TZQCS 64
	unsigned short t_pctl_tzqcl;  //PCTL TZQCL 0..1023
	unsigned short t_pctl_txpdll; //PCTL TXPDLL 3..63
	unsigned short t_pctl_tzqcsi; //PCTL TZQCSI 0..4294967295
	unsigned short t_pctl_scfg;   //PCTL 

	unsigned       t_mmc_ddr_ctrl;
	unsigned       t_ddr_pll_cntl;
	unsigned       t_ddr_clk;
	unsigned       t_mmc_ddr_timming0;
	unsigned       t_mmc_ddr_timming1;
	unsigned       t_mmc_ddr_timming2;
	unsigned       t_mmc_arefr_ctrl;
	int            (* init_pctl)(struct ddr_set *);

//#if defined (LPDDR2)||defined (LPDDR3) //for lpddr only
	unsigned short t_pctl_tckesr;   //
	unsigned short t_pctl_tdpd;   //
//#endif
}__attribute__ ((packed));

#define PHYS_MEMORY_START        (0x00000000) // ???
//4Gb x 2pcs(H5TQ4G63CFR-RDC)

#define CONFIG_DDR3_ROW_SIZE (3)
#define CONFIG_DDR3_COL_SIZE (2)
#define CONFIG_DDR_ROW_BITS  (15)
#define PHYS_MEMORY_SIZE     (0x40000000) // 1GB

//DDR mode. Once considering this value stored in efuse, 0=NotSet is necessary
#define CFG_DDR_NOT_SET			0
#define CFG_DDR_32BIT			1
#define CFG_DDR_16BIT_LANE02	2	//DDR lane0+lane2
#define CFG_DDR_16BIT_LANE01	3	//DDR lane0+lane1
#define CFG_DDR_MODE_STO_ADDR	0	//2 //2 bits, store in efuse etc..
#define CFG_DDR_MODE_STO_OFFSET	0	//6	//offset of these 2 bits

#define DDR_RSLR_LEN 6
#define DDR_RDGR_LEN 4

//low power ddr defines
#ifdef CONFIG_LPDDR2
#undef CONFIG_LPDDR3
#define LPDDR2
#endif
#ifdef CONFIG_LPDDR3
#undef CONFIG_LPDDR2
#define LPDDR3
#endif

#if !defined(CONFIG_DDR_COL_BITS)
	#define CONFIG_DDR_COL_BITS  (10)
#endif //CONFIG_DDR_COL_BITS

//DDR training address, DO NOT modify
//DDR0: 0x0F00 - 0x0F7F (128Bytes)
#define CONFIG_M8B_RANK0_DTAR_ADDR (0x3000000)

//M8baby support 16bit and 32bit mode
//#define CONFIG_M8B_DDR_BIT_MODE_SET (CONFIG_M8B_DDR_BIT_MODE_32BIT) //m8b_xxx_xxx.h
#define CONFIG_M8B_DDR_BIT_MODE_32BIT  (0)
#define CONFIG_M8B_DDR_BIT_MODE_16BIT  (1)

//M8baby support two ranks: Rank0 or Rank0+1
//#define CONFIG_M8B_DDR_RANK_SET  (CONFIG_M8B_DDR_RANK0_ONLY) //m8b_xxx_xxx.h.
#define CONFIG_M8B_DDR_RANK0_ONLY      (0)
#define CONFIG_M8B_DDR_RANK0_AND_RANK1 (1)

#define CONFIG_M8B_DDR_RANK0_SIZE_256M  (1)
#define CONFIG_M8B_DDR_RANK0_SIZE_512M  (2)
#define CONFIG_M8B_DDR_RANK0_SIZE_1GB   (3)
#define CONFIG_M8B_DDR_RANK0_SIZE_2GB   (0)

#define CONFIG_M8B_DDR_RANK1_START_ADDR_256M  (256<<20)
#define CONFIG_M8B_DDR_RANK1_START_ADDR_512M  (512<<20)
#define CONFIG_M8B_DDR_RANK1_START_ADDR_1GB   (1<<30)
#define CONFIG_M8B_DDR_RANK1_START_ADDR_2GB   (2<<30)

//M8 DDR0/1 address map bank mode
#define CONFIG_M8B_DDR_BANK_MODE_2_BNK   (0)
#define CONFIG_M8B_DDR_BANK_MODE_4_BNK   (1)

//M8baby support 4 mode to abstract bank from address
//#define CONFIG_M8B_DDR_BANK_SET (CONFIG_M8B_DDR_BANK_SET_S12 ) //m8b_xxx_xxx.h
#define CONFIG_M8B_DDR_BANK_SET_S12	    (0)
#define CONFIG_M8B_DDR_BANK_SET_S13_S12	(1)
#define CONFIG_M8B_DDR_BANK_SET_S8      (2)
#define CONFIG_M8B_DDR_BANK_SET_S9_S8   (3)

#if !defined(CONFIG_M8B_DDR_RANK_SET)
  #define CONFIG_M8B_DDR_RANK_SET  (CONFIG_M8B_DDR_RANK0_ONLY)
#endif

#if !defined(CONFIG_M8B_DDR_BANK_SET)
  #define CONFIG_M8B_DDR_BANK_SET (CONFIG_M8B_DDR_BANK_SET_S12)
#endif

#if (CFG_DDR_MODE > CFG_DDR_32BIT)	//not 32bit mode
	#define CONFIG_M8B_DDR_BIT_MODE_SET (CONFIG_M8B_DDR_BIT_MODE_16BIT)
#else
	#define CONFIG_M8B_DDR_BIT_MODE_SET (CONFIG_M8B_DDR_BIT_MODE_32BIT)
#endif

#if !defined(CONFIG_M8B_DDR_BIT_MODE_SET)
  #define CONFIG_M8B_DDR_BIT_MODE_SET (CONFIG_M8B_DDR_BIT_MODE_32BIT)  
#endif

#define M8BABY_DDR_DTAR_BANK_GET(addr,bit_row,bit_col,bank_set,bit_set) \
	((((addr >> (bit_row+bit_col+(3-bit_set)+(bank_set&1))) & ((bank_set&1) ? 1 : 3)) << ((bank_set & 1)+1) ) |\
		 ((addr >>(((bank_set&2) ? 6 : bit_col)+(2-bit_set))) & ((bank_set&1)?3:1)))

#define M8BABY_DDR_DTAR_DTROW_GET(addr,bit_row,bit_col,bank_set,bit_set) \
	(( (addr) >> (bit_col+((bank_set&1)+(3-bit_set))) & ((1<< (bit_row))-1)))


#define M8BABY_DDR_DTAR_DTCOL_GET(addr,bit_col,bank_set,bit_set) \
	((( (addr) >> (2-bit_set)) & ((1<< (((bank_set) & 2) ? 6 : (bit_col)))-1))| \
		(((bank_set) & 2)? ((((addr) >> (((bank_set) & 1)+(9-bit_set)))&((1<<((bit_col)-6))-1))<<6):(0)))

#define CONFIG_DDR0_DTAR_DTBANK  M8BABY_DDR_DTAR_BANK_GET(CONFIG_M8B_RANK0_DTAR_ADDR,CONFIG_DDR_ROW_BITS,CONFIG_DDR_COL_BITS,CONFIG_M8B_DDR_BANK_SET,CONFIG_M8B_DDR_BIT_MODE_SET)
#define CONFIG_DDR0_DTAR_DTROW   M8BABY_DDR_DTAR_DTROW_GET(CONFIG_M8B_RANK0_DTAR_ADDR,CONFIG_DDR_ROW_BITS,CONFIG_DDR_COL_BITS,CONFIG_M8B_DDR_BANK_SET,CONFIG_M8B_DDR_BIT_MODE_SET)
#define CONFIG_DDR0_DTAR_DTCOL   M8BABY_DDR_DTAR_DTCOL_GET(CONFIG_M8B_RANK0_DTAR_ADDR,CONFIG_DDR_COL_BITS,CONFIG_M8B_DDR_BANK_SET,CONFIG_M8B_DDR_BIT_MODE_SET)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//NOTE: IMPORTANT! DO NOT TRY TO MODIFY FOLLOWING CODE!
//           It is used to get DDR0 training address from PUB_DTAR0.
//          
//How to fetch the DDR training address for M8:
//           1. enable PCTL clock before access
//           2. load DMC DDR setting from P_DMC_DDR_CTRL
//           3. load the DTAR0 value from DDR0 PUB register according to the channel setting from DMC_DDR_CTRL
//           4. disable PCTL clock for power saving
//
//Demo code: 
/*          
	//enable clock
	writel(readl(P_DDR0_CLK_CTRL)|(1), P_DDR0_CLK_CTRL);

	printf("training address: %x\n", M8BABY_GET_DT_ADDR(readl(P_DDR0_PUB_DTAR0), readl(P_DMC_DDR_CTRL)));

	//disable clock
	writel(readl(P_DDR0_CLK_CTRL) & (~1), P_DDR0_CLK_CTRL);  
*/

#define M8BABY_GET_DT_ADDR(dtar, dmc) \
	(((((dtar) >> 28) & 0x7) & (((((dmc) >> 5) & 0x3)&1)?3:1)) << ((((((dmc) >> 5) & 0x3)&2) ? 6 : (((dmc) & 0x3) + 8))+(2-(((dmc) >> 7) & 0x1)))) | \
	(((((((dtar) >> 28) & 0x7) >> (((((dmc) >> 5) & 0x3) & 1)+1))) & (((((dmc) >> 5) & 0x3)&1) ? 1 : 3)) << (((((dmc) >> 2) & 0x3) ? ((((dmc) >> 2) & 0x3)+12) : (16))+(((dmc) & 0x3) + 8)+(3-(((dmc) >> 7) & 0x1))+((((dmc) >> 5) & 0x3)&1))) | \
	(((((dtar)) & 0xfff) & ((1<< ((((((dmc) >> 5) & 0x3)) & 2) ? 6 : ((((dmc) & 0x3) + 8))))-1)) << (2-(((dmc) >> 7) & 0x1))) | \
	((((((dmc) >> 5) & 0x3)) & 2) ? ((((((dtar)) & 0xfff) >> 6) & ((1<<(((((dmc) & 0x3) + 8))-6))-1)) << ((((((dmc) >> 5) & 0x3)) & 1)+(9-(((dmc) >> 7) & 0x1)))):(0)) | \
	((((dtar) >> 12) & 0xffff) << ((((dmc) & 0x3) + 8)+(((((dmc) >> 5) & 0x3)&1)+(3-(((dmc) >> 7) & 0x1)))))

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define CFG_DDR_CLK    636 //696 //768  //792// (636)

#if (CFG_DDR_CLK >= 384) && (CFG_DDR_CLK < 750)
	#define CFG_PLL_OD 2
	#define CFG_PLL_N  1
	#define CFG_PLL_M  (((CFG_DDR_CLK/6)*6)/12)
#elif (CFG_DDR_CLK >= 750) && (CFG_DDR_CLK <= 912)
	#define CFG_PLL_OD 1
	#define CFG_PLL_N  1
	#define CFG_PLL_M  (((CFG_DDR_CLK/12)*12)/24)
#else
	#error "Over PLL range! Please check CFG_DDR_CLK with file m8_skt_v1.h! \n"
#endif

#if (CFG_DDR_CLK >= 384 ) && (CFG_DDR_CLK <533)
	#define DDR3_7_7_7
#elif  (CFG_DDR_CLK >= 533 ) && (CFG_DDR_CLK <667)
	#define DDR3_9_9_9 
#elif  (CFG_DDR_CLK >= 667 ) && (CFG_DDR_CLK <=912)
	#define DDR3_11_11_11
#endif

/////////////////////////////////////////////////////////////////////////////////
//Following setting for board XXXXXXX with DDR K4B4G1646B(SANSUNG)
#ifdef DDR3_7_7_7
	//DTPR0
	#define CFG_DDR_RTP (6)
	#define CFG_DDR_WTR (6)
	#define CFG_DDR_RP  (7)
	#define CFG_DDR_RCD (7)
	#define CFG_DDR_RAS (20)
	#define CFG_DDR_RRD (6)
	#define CFG_DDR_RC  (27)

	//DTPR1
	#define CFG_DDR_MRD (4)
	#define CFG_DDR_MOD (12)
	#define CFG_DDR_FAW (27)
	#define CFG_DDR_RFC (139)
	#define CFG_DDR_WLMRD (40)
	#define CFG_DDR_WLO (6)

	//DTPR2
	#define CFG_DDR_XS   (512)
	#define CFG_DDR_XP   (5)
	#define CFG_DDR_CKE  (4)
	#define CFG_DDR_DLLK (512)
	#define CFG_DDR_RTODT (0)
	#define CFG_DDR_RTW   (4)

	#define CFG_DDR_REFI  (78)
	#define CFG_DDR_REFI_MDDR3  (4)

	#define CFG_DDR_CL    (7)
	#define CFG_DDR_WR    (12)
	#define CFG_DDR_CWL   (8)
	#define CFG_DDR_AL    (0)
	#define CFG_DDR_EXSR  (512)
	#define CFG_DDR_DQS   (4)
	#define CFG_DDR_CKSRE (8)
	#define CFG_DDR_CKSRX (8)
	#define CFG_DDR_ZQCS  (64)
	#define CFG_DDR_ZQCL  (512)
	#define CFG_DDR_XPDLL (20)
	#define CFG_DDR_ZQCSI (1000)
#endif

#ifdef DDR3_9_9_9
	//DTPR0
	#define CFG_DDR_RTP (6)
	#define CFG_DDR_WTR (6)
	#define CFG_DDR_RP  (9)
	#define CFG_DDR_RCD (9)
	#define CFG_DDR_RAS (24)
	#define CFG_DDR_RRD (5)
	#define CFG_DDR_RC  (33)

	//DTPR1
	#define CFG_DDR_MRD (4)
	#define CFG_DDR_MOD (12)
	#define CFG_DDR_FAW (30)
	#define CFG_DDR_RFC (174)
	#define CFG_DDR_WLMRD (40)
	#define CFG_DDR_WLO (6)

	//DTPR2
	#define CFG_DDR_XS   (512)
	#define CFG_DDR_XP   (5)
	#define CFG_DDR_CKE  (4)
	#define CFG_DDR_DLLK (512)
	#define CFG_DDR_RTODT (0)
	#define CFG_DDR_RTW   (4)

	#define CFG_DDR_REFI  (78)
	#define CFG_DDR_REFI_MDDR3  (4)
		
	#define CFG_DDR_CL    (9)
	#define CFG_DDR_WR    (12)
	#define CFG_DDR_CWL   (8)
	#define CFG_DDR_AL    (0)
	#define CFG_DDR_EXSR  (512)
	#define CFG_DDR_DQS   (4)
	#define CFG_DDR_CKSRE (8)
	#define CFG_DDR_CKSRX (8)
	#define CFG_DDR_ZQCS  (64)
	#define CFG_DDR_ZQCL  (136)
	#define CFG_DDR_XPDLL (20)
	#define CFG_DDR_ZQCSI (1000)

#endif

#ifdef DDR3_11_11_11
	//DTPR0
	#define CFG_DDR_RTP (6)
	#define CFG_DDR_WTR (6)
	#define CFG_DDR_RP  (11)
	#define CFG_DDR_RCD (11)
	#define CFG_DDR_RAS (28)
	#define CFG_DDR_RRD (6)
	#define CFG_DDR_RC  (39)

	//DTPR1
	#define CFG_DDR_MRD (4)
	#define CFG_DDR_MOD (12)
	#define CFG_DDR_FAW (32)
	#define CFG_DDR_RFC  208 //(128)
	#define CFG_DDR_WLMRD (40)
	#define CFG_DDR_WLO  7 // (6)  //jiaxing add

	//DTPR2
	#define CFG_DDR_XS   (512)
	#define CFG_DDR_XP   (5)
	#define CFG_DDR_CKE  (4)
	#define CFG_DDR_DLLK (512)
	#define CFG_DDR_RTODT (0)
	#define CFG_DDR_RTW   (4)

	#define CFG_DDR_REFI  (78)
	#define CFG_DDR_REFI_MDDR3  (4)
		
	#define CFG_DDR_CL    (11)
	#define CFG_DDR_WR    (12)
	#define CFG_DDR_CWL   (8)
	#define CFG_DDR_AL    (0)
	#define CFG_DDR_EXSR  (512)
	#define CFG_DDR_DQS   (4)
	#define CFG_DDR_CKSRE (8)
	#define CFG_DDR_CKSRX (8)
	#define CFG_DDR_ZQCS  (64)
	#define CFG_DDR_ZQCL  (512)  //jiaxing add
	#define CFG_DDR_XPDLL (20)
	#define CFG_DDR_ZQCSI (1000)

#endif


/////////////////////////////////////////////////////////////////////////////



