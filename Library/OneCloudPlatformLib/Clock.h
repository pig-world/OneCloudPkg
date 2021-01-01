#define IO_CBUS_BASE            0xC1100000

#define CBUS_REG_OFFSET(reg) ((reg) << 2)
#define CBUS_REG_ADDR(reg)	 (IO_CBUS_BASE + CBUS_REG_OFFSET(reg))

#define AM_ANALOG_TOP_REG0                         0x206e
#define AM_ANALOG_TOP_REG1                         0x206f
#define HHI_MPLL_CNTL6                             0x10a5
#define HHI_VID_PLL_CNTL5                          0x10cc
#define HHI_SYS_CPU_CLK_CNTL                       0x1067
#define HHI_A9_CLK_CNTL                            (HHI_SYS_CPU_CLK_CNTL)
#define HHI_MPEG_CLK_CNTL                          0x105d
#define HHI_SYS_PLL_CNTL                           0x10c0
#define HHI_SYS_PLL_CNTL2                          0x10c1
#define HHI_SYS_PLL_CNTL3                          0x10c2
#define HHI_SYS_PLL_CNTL4                          0x10c3
#define HHI_SYS_PLL_CNTL5                          0x10c4
#define HHI_DPLL_TOP_0                             0x10c6
//SYS PLL	< 2g
#define CFG_SYS_PLL_CNTL_2 (0x59C88000)
#define CFG_SYS_PLL_CNTL_3 (0xCA45B823)
#define CFG_SYS_PLL_CNTL_4 (0x0001D407)
#define CFG_SYS_PLL_CNTL_5 (0x00000870)

#define HHI_MPLL_CNTL                              0x10a0
#define HHI_MPLL_CNTL2                             0x10a1
#define HHI_MPLL_CNTL3                             0x10a2
#define HHI_MPLL_CNTL4                             0x10a3
#define HHI_MPLL_CNTL5                             0x10a4
#define HHI_MPLL_CNTL6                             0x10a5
#define HHI_MPLL_CNTL7                             0x10a6
#define HHI_MPLL_CNTL8                             0x10a7
#define HHI_MPLL_CNTL9                             0x10a8
#define HHI_MPLL_CNTL10                            0x10a9

//FIXED PLL/Multi-phase PLL	= 2.55g(FIXED)
#define CFG_MPLL_CNTL_2 (0x59C80000)
#define CFG_MPLL_CNTL_3 (0xCA45B822)
#define CFG_MPLL_CNTL_4 (0x00014007)
#define CFG_MPLL_CNTL_5 (0xB5500E1A)
#define CFG_MPLL_CNTL_6 (0xF4454545)
#define CFG_MPLL_CNTL_7 (0)
#define CFG_MPLL_CNTL_8 (0)
#define CFG_MPLL_CNTL_9 (0)

#define HHI_VID_PLL_CNTL                           0x10c8
#define HHI_VID_PLL_CNTL2                          0x10c9
#define HHI_VID_PLL_CNTL3                          0x10ca
#define HHI_VID_PLL_CNTL4                          0x10cb
#define HHI_VID_PLL_CNTL5                          0x10cc
#define HHI_VID_PLL_CNTL6                          0x10cd
//VID PLL < 1.7g
#define CFG_VID_PLL_CNTL_2 (0x59C8C000)
#define CFG_VID_PLL_CNTL_3 (0xCA49B022)
#define CFG_VID_PLL_CNTL_4 (0x0023B100)
#define CFG_VID_PLL_CNTL_5 (0x00012385)

#define Is_dos_reg(addr) ((((addr)>>8)==0x09 || ((addr)>>8)==0x0c || ((addr)>>8)==0x0e || ((addr)>>8)==0x03)? 1 : 0)

#define Wr_cbus(addr, data) *(volatile unsigned long *)((Is_dos_reg(addr)? 0xd0050000:0xc1100000)|((addr)<<2))=(data)
#define Rd_cbus(addr) *(volatile unsigned long *)((Is_dos_reg(addr)? 0xd0050000:0xc1100000)|((addr)<<2))
#define Wr_reg_bits(reg, val, start, len) \
  Wr_cbus(reg, ((Rd_cbus(reg) & ~(((1L<<(len))-1)<<(start))) | ((unsigned int)(val) << (start))))

//M8 pll controler use bit 29 as reset bit
#define PLL_ENTER_RESET(pll) \
	Wr_cbus(pll,(1<<29));

#define PLL_RELEASE_RESET(pll) \
	Wr_cbus(pll, Rd_cbus(pll)&(~(1<<29)));

#define PLL_SETUP(pll,set) \
	Wr_cbus(pll,(set) |(1<<29) |(1<<30));\
	MicroSecondDelay(1); //wait 1ms for PLL lock
    
#define M8_PLL_SETUP(set, pll) \
	MmioWrite32((set) |(1<<29) |(1<<30), pll);\
	MicroSecondDelay(1);  //wait 1ms for PLL lock

#define MAX_PLL_TRY_TIMES (5)
//m8 pll init retry check, it will do watch reset 
//after try MAX_PLL_TRY_TIMES times
#define PLL_LOCK_CHECK(counter,type) \
	    NanoSecondDelay(500000); \
		counter++; \
		if(counter > 1){ \
			if(counter>MAX_PLL_TRY_TIMES){ \
			} \
		}
        //Need reboot, should nerver reach this