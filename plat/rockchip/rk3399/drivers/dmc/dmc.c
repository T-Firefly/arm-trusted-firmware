/*
 * Copyright (c) 2016, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <debug.h>

#include <platform_def.h>
#include <plat_private.h>
#include <dmc.h>

/*define for debug*/
#define DMC_CAN_REMOVE_CODE	0
#define DMC_DEBUG_UART_PRINT_CODE 1
#define DMC_PHY_REGS 0

#define DMC_PWRUP_SREFRESH_EXIT 1
#define DMC_DO_DATA_TRAINING	1
#define DMC_DDR3_SUPPORT	0

#if 0
__sramdata struct rk3399_sdram_params sdram_configs[] = {
#include "sdram-lpddr3-hynix-4GB-200.inc"	/* ram_code = 0000 */
};
#else
__sramdata struct rk3399_sdram_params sdram_configs[] = {
#include "sdram_configs.inc"	/* ram_code = 0000 */
};
#endif

#if 0
#define PHY_REGS_GROUP	8
__sramdata u32 phy_regs_map[PHY_REGS_GROUP][3]= {
	{0,0,91},
	{128,91,91},
	{256,182,91},
	{384,273,91},
	{512,364,38},
	{640,402,38},
	{768,440,38},
	{896,478,62},
};
#endif

//#define PHY_REGS_GROUP_SAVE	5
__sramdata u32 phy_regs_map_save[][3]= {
	{0,0,91},
	//{128,91,91},
	//{256,182,91},
	//{384,273,91},
	{512,91,38},
	{640,129,38},
	{768,167,38},
	{896,205,62},
};
#define PHY_REGS_GROUP_SAVE	(sizeof(phy_regs_map_save)/sizeof(phy_regs_map_save[0]))

//#define PHY_REGS_GROUP_RESTORE	8
__sramdata u32 phy_regs_map_restore[][3]= {
	{0,0,91},
	{128,0,91},
	{256,0,91},
	{384,0,91},
	{512,91,38},
	{640,129,38},
	{768,167,38},
	{896,205,62},
};
#define PHY_REGS_GROUP_RESTORE	(sizeof(phy_regs_map_restore)/sizeof(phy_regs_map_restore[0]))

__sramdata u32 sys_reg = 0;
__sramdata u32 cru_con = 0;

__sramdata static struct rk3399_ddr_pctl_regs * rk3399_ddr_pctl[2] = {
	(void *)DDRC0_BASE_ADDR, (void *)DDRC1_BASE_ADDR};
__sramdata static struct rk3399_ddr_pi_regs * rk3399_ddr_pi[2] = {
	(void *)DDRC0_PI_BASE_ADDR, (void *)DDRC1_PI_BASE_ADDR};
__sramdata static struct rk3399_ddr_publ_regs * rk3399_ddr_publ[2] = {
	(void *)DDRC0_PHY_BASE_ADDR, (void *)DDRC1_PHY_BASE_ADDR};
__sramdata static struct rk3399_msch_regs * rk3399_msch[2] = {
	(void *)SERVER_MSCH0_BASE_ADDR, (void *)SERVER_MSCH1_BASE_ADDR};

__sramfunc void dmc_udelay(u32 us)
{
    volatile u32 i;

    i = ((24*us*195+8191)/8192);
    if(i)
    {
        while(i--);
    }
}

/*
 * Copy @len bytes from @src to @dst
 */
__sramfunc void *dmc_memcpy_iram(void *dst, const void *src, u32 len)
{
	const u32 *s = src;
	u32 *d = dst;

	/*copy 4 bytes(u32) one time*/
	len = len/4;
	while (len--)
		*d++ = *s++;

	return dst;
}

/*copy registers to buffer when suspend,can not use IRAM code*/
/*
 * Copy @len bytes from @src to @dst
 */
void *dmc_memcpy_dram(void *dst, const void *src, u32 len)
{
	const u32 *s = src;
	u32 *d = dst;

	/*copy 4 bytes(u32) one time*/
	len = len/4;
	while (len--)
		*d++ = *s++;

	return dst;
}


#if DMC_DEBUG_UART_PRINT_CODE
struct rk3399_uart_regs
{
		u32 uart_rbr;
		u32 uart_dlh;
		u32 uart_iir;
		u32 uart_lcr;
		u32 uart_mcr;
		u32 uart_lsr;
		u32 uart_msr;
		u32 uart_scr;
		u32 reserved1[(0x30-0x20)/4];
		u32 uart_srbr[(0x70-0x30)/4];
		u32 uart_far;
		u32 uart_tfr;
		u32 uart_rfw;
		u32 uart_usr;
		u32 uart_tfl;
		u32 uart_rfl;
		u32 uart_srr;
		u32 uart_srts;
		u32 uart_sbcr;
		u32 uart_sdmam;
		u32 uart_sfe;
		u32 uart_srt;
		u32 uart_stet;
		u32 uart_htx;
		u32 uart_dmasa;
		u32 reserved2[(0xf4-0xac)/4];
		u32 uart_cpr;
		u32 uart_ucv;
		u32 uart_ctr;
};

#define	UART_DBG_BASE_ADDR		0xFF1A0000
#define	UART2_BASE_ADDR			UART_DBG_BASE_ADDR
__sramdata struct rk3399_uart_regs *rk3399_uart_reg = (void *)UART2_BASE_ADDR;

#define  UART_LSR_TEMT                0x40 /* Transmitter empty */

/* GRF REG */
struct rk3399_grf_regs {
	u32 reserved0[(0xe000 - 0x0) / 4];
	u32 grf_gpio2a_iomux;
	u32 grf_gpio2b_iomux;
	u32 grf_gpio2c_iomux;
	u32 grf_gpio2d_iomux;
	u32 grf_gpio3a_iomux;
	u32 grf_gpio3b_iomux;
	u32 grf_gpio3c_iomux;
	u32 grf_gpio3d_iomux;
	u32 grf_gpio4a_iomux;
	u32 grf_gpio4b_iomux;
	u32 grf_gpio4c_iomux;
	u32 grf_gpio4d_iomux;
	u32 reserved1[(0xe200 - 0xe030) / 4];
	u32 grf_soc_con0;
	u32 grf_soc_con1;
	u32 grf_soc_con2;
	u32 grf_soc_con3;
	u32 grf_soc_con4;
	u32 grf_soc_con5;
	u32 reserved_con6;
	u32 grf_soc_con7;
	u32 grf_soc_con8;
	u32 reserved2[(0xe380 - 0xe224) / 4];
	u32 grf_ddrc0_con0;
	u32 grf_ddrc0_con1;
	u32 grf_ddrc1_con0;
	u32 grf_ddrc1_con1;
};
#define GRF_BASE_ADDR			0xFF770000
__sramdata struct rk3399_grf_regs * grf_regs = (void *)GRF_BASE_ADDR;

__sramfunc static  void uart_iomux(void)
{
	u32 guart_iomux=3;
		if (guart_iomux == 1)
			writel_relaxed((0xf << (0 + 16)) | (0xA << 0), &grf_regs->grf_gpio4b_iomux); /* uart2a to sdmmc pin */
		else if (guart_iomux == 2)
			writel_relaxed((0xf << (0 + 16)) | (0xA << 0), &grf_regs->grf_gpio4c_iomux); /* uart2b to hdmii2c pin */
		else if (guart_iomux == 3)
			writel_relaxed((0xf << (6 + 16)) | (0x5 << 6), &grf_regs->grf_gpio4c_iomux); /* uart2c to GPIO pin */
		writel_relaxed((0x3 << (10 + 16)) | ((guart_iomux - 1) << 10), &grf_regs->grf_soc_con7);
}


__sramfunc void uart_init(void)
{
	u32 guart_baud=1500000;
		//uart_sel_clk();
		//enable uart0 clk
		//init uart0
		writel_relaxed(0x83, &rk3399_uart_reg->uart_lcr);
		if (guart_baud == 1500000)
			writel_relaxed(0x1, &rk3399_uart_reg->uart_rbr);
		else
			writel_relaxed(0xd, &rk3399_uart_reg->uart_rbr);
		writel_relaxed(0x3, &rk3399_uart_reg->uart_lcr);
		if(0)
		uart_iomux();
		writel_relaxed(0x1, &rk3399_uart_reg->uart_sfe); /* enable FIFO */
}

__sramfunc void uart_wrtie_byte(unsigned char byte)
{
	return;
		//while((g_uartReg->UART_USR & UART_TRANSMIT_FIFO_NOT_FULL) != UART_TRANSMIT_FIFO_NOT_FULL);
		writel_relaxed(byte, &rk3399_uart_reg->uart_rbr);
		while(!(read32(&rk3399_uart_reg->uart_lsr) & UART_LSR_TEMT));
}

__sramfunc void print(char *s)
{
	while (*s)
	{
		if (*s == '\n')
		{
				uart_wrtie_byte('\r');
		}
			uart_wrtie_byte(*s);
			s++;
	}
}

__sramfunc void _print_hex (u32 n)
{
	if (n & ~0xf) {
		_print_hex(n >> 4);
		n &= 0xf;
	}

	if (n < 10)
		uart_wrtie_byte((unsigned char)(n + '0'));
	else
		uart_wrtie_byte((unsigned char)(n - 10 + 'A'));
}

__sramfunc void print_hex (u32 n)
{
	//port_2_uart();
	_print_hex(n);
	//port_2_usb();
}


__sramfunc void _print_dec (u32 n)
{
	if (n >= 10) {
		_print_dec(n / 10);
		n %= 10;
	}
	uart_wrtie_byte((unsigned char)(n + '0'));
}

__sramfunc void print_dec (u32 n)
{
	//port_2_uart();
	_print_dec(n);
	//port_2_usb();
}


__sramfunc void print_regs(u32 flag, u32 channel, u32 start, u32 size)
{
	struct rk3399_ddr_pctl_regs *ddr_pctl_regs = rk3399_ddr_pctl[channel];
	struct rk3399_ddr_pi_regs *ddr_pi_regs = rk3399_ddr_pi[channel];
	struct rk3399_ddr_publ_regs *ddr_publ_regs = rk3399_ddr_publ[channel];
	u32 i,/*j,*/tmp=0;

	i = start;

	if (size == 999) {
		if (flag == 0) {
			size = 332;
		} else if (flag == 1) {
			size = 200;
		} else if (flag == 2) {
			size = 959;
		}
	}
	//j=size+start;
	for(i=start;i<(size+start);i++) {
		print("{");
		print_dec(channel);
		print("}");
		if (flag == 0) {
			print("denali_ctl");
		} else if (flag == 1) {
			print("denali_pi");
		} else if (flag == 2) {
			print("denali_phy");
		}
		print("[");
		if (i < 10) {
			print("  ");
		} else if (i < 100) {
			print(" ");
		}
		print_dec(i);
		print("]=0x");
		if (flag == 0) {
			tmp=readl_relaxed(&ddr_pctl_regs->denali_ctl[i]);
		} else if (flag == 1) {
			tmp=readl_relaxed(&ddr_pi_regs->denali_pi[i]);
		} else if (flag == 2) {
			tmp=readl_relaxed(&ddr_publ_regs->denali_phy[i]);
		}
		if (tmp<0x10) {
			print("0000000");
		} else if (tmp<0x100) {
			print("000000");
		} else if (tmp<0x1000) {
			print("00000");
		} else if (tmp<0x10000) {
			print("0000");
		} else if (tmp<0x100000) {
			print("000");
		} else if (tmp<0x1000000) {
			print("00");
		} else if (tmp<0x10000000) {
			print("0");
		}
		print_hex(tmp);
		print("\n");
		}
}
#endif



/******SGRF****************/
/* sgrf reg */
struct rk3399_sgrf_regs {
	u32 reserved0[0x40 / 4];
	u32 secure_grf_ddr_rgn_con16;
	u32 reserved1[(0xe00c - 0x44) / 4];
	u32 sgrf_soc_con3;
	u32 sgrf_soc_con4;
	u32 sgrf_soc_con5;
	u32 sgrf_soc_con6;
	u32 sgrf_soc_con7;
};

#define	SGRF_BASE_ADDR			0xFF330000
__sramdata struct rk3399_sgrf_regs * sgrf_regs = (void *)SGRF_BASE_ADDR;
/*************************************************/


/******CRU********************/
typedef enum PLL_ID_Tag
{
		APLL_L=0,
		APLL_B,
		DPLL,
		CPLL,
		GPLL,
		NPLL,
		VPLL,
		PLL_MAX
}PLL_ID;

/* CRU register */
struct rk3399_cru_regs {
		u32 cru_pll_con[PLL_MAX][8];
		u32 reserved0[(0x100 - 0xe0) / 4];
		u32 cru_clksel_con[81];
		u32 reserved1[(0x300 - 0x244) / 4];
		u32 cru_gate_con[35];
		u32 reserved2[(0x400 - 0x38c) / 4];
		u32 cru_softreset_con[21];
		u32 reserved3[(0x500 - 0x454) / 4];
		u32 cru_glb_srst_fst_value;
		u32 cru_glb_srst_snd_value;
		u32 cru_glb_cnt_th;
		u32 cru_misc_con;
		u32 cru_glb_rst_con;
		u32 cru_glb_rst_st;
};

/* CRU_PLL_CON0 */
#define FBDIV(n)      ((0xFFF << 16) | (n))
/* CRU_PLL_CON1 */
#define POSTDIV2(n)   ((0x7 << (12 + 16)) | ((n) << 12))
#define POSTDIV1(n)   ((0x7 << (8 + 16)) | ((n) << 8))
#define REFDIV(n)     ((0x3F << 16) | (n))
/* CRU_PLL_CON2 */
#define PLL_LOCK(n)       (((n) >> 31) & 0x1)
/* CRU_PLL_CON3 */
#define PLL_SLOW_MODE		(0)
#define PLL_NORMAL_MODE		(1)
#define PLL_DP_SLOW_MODE	(2)
#define PLL_MODE(n)			((0x3 << (8 + 16)) | ((n) << 8))
#define FOUT4PHASEPD(n)		((0x1 << (6 + 16)) | ((n) << 6))
#define FOUTVCOPD(n)		((0x1 << (5 + 16)) | ((n) << 5))
#define FOUTPOSTDIVPD(n)	((0x1 << (4 + 16)) | ((n) << 4))
#define DSMPD(n)			((0x1 << (3 + 16)) | ((n) << 3))
#define DACPD(n)			((0x1 << (2 + 16)) | ((n) << 2))
#define PLL_BYPASS(n)		((0x1 << (1 + 16)) | ((n) << 1))
#define PLL_POWER_DOWN(n)	((0x1 << (0 + 16)) | ((n) << 0))

#define CRU_SFTRST_DDR_CTRL(ch, n)	((0x1 << (8 + 16 + (ch) * 4)) | ((n) << (8 + (ch) * 4)))
#define CRU_SFTRST_DDR_PHY(ch, n)	((0x1 << (9 + 16 + (ch) * 4)) | ((n) << (9 + (ch) * 4)))

#define CRU_BASE_ADDR		0xFF760000
__sramdata struct rk3399_cru_regs *rk3399_cru_reg = (void *)CRU_BASE_ADDR;

#define FBDIV_ENC(n)	((n) << 16)
#define FBDIV_DEC(n)	(((n) >> 16)&0xfff)
#define POSTDIV2_ENC(n)	((n) << 12)
#define POSTDIV2_DEC(n)	(((n) >> 12)&0x7)
#define POSTDIV1_ENC(n)	((n) << 8)
#define POSTDIV1_DEC(n)	(((n) >> 8)&0x7)
#define REFDIV_ENC(n)	(n)
#define REFDIV_DEC(n)	((n) &0x3f)

/******************************************************************/

/******PMU*****************************/
/* PMU GRF REG */
struct rk3399_pmu_grf_regs {
	u32 reserved0[(0x180 - 0) / 4];
	u32 pmugrf_soc_con0;
	u32 reserved1[(0x1a8 - 0x184) / 4];
	u32 pmugrf_soc_con10;
	u32 pmugrf_soc_con11;
	u32 reserved2[(0x300 - 0x1b0) / 4];
	u32 pmugrf_os_reg[4];
};

#define	PMU_GRF_BASE_ADDR		0xFF320000
__sramdata struct rk3399_pmu_grf_regs *pmugrf_regs =	(void *)PMU_GRF_BASE_ADDR;
/********************************************/

/*****************************************/
/* PMU CRU */
#define PMU_CRU_BASE_ADDR		0xFF750000

#define PMUCRU_RSTNHOLD_CON0	(0x120)
#define PMUCRU_RSTNHOLD_CON1	(0x124)

#define PRESET_SGRF_HOLD(n)		((0x1 << (6+16)) | ((n) << 6))
#define PRESET_GPIO0_HOLD(n)	((0x1 << (7+16)) | ((n) << 7))
#define PRESET_GPIO1_HOLD(n)	((0x1 << (8+16)) | ((n) << 8))
/********************************************/

/****PMU**********************************/
/* PMU registers */
struct rk3399_pmu_regs {
		u32 pmu_wakeup_cfg[5];
		u32 pmu_pwrdn_con;
		u32 pmu_pwrdn_st;
		u32 pmu_pll_con;
		u32 pmu_pwrmode_con;
		u32 pmu_sft_con;
		u32 pmu_int_con;
		u32 pmu_int_st;
		u32 pmu_gpio0_pos_int_con;
		u32 pmu_gpio0_net_int_con;
		u32 pmu_gpio1_pos_int_con;
		u32 pmu_gpio1_net_int_con;
		u32 pmu_gpio0_pos_int_st;
		u32 pmu_gpio0_net_int_st;
		u32 pmu_gpio1_pos_int_st;
		u32 pmu_gpio1_net_int_st;
		u32 pmu_pwrdn_inten;
		u32 pmu_pwrdn_status;
		u32 pmu_wakeup_status;
		u32 pmu_bus_clr;
		u32 pmu_bus_idle_req;
		u32 pmu_bus_idle_st;
		u32 pmu_bus_idle_ack;
		u32 pmu_cci500_con;
		u32 pmu_adb400_con;
		u32 pmu_adb400_st;
		u32 pmu_power_st;
		u32 pmu_core_pwr_st;
		u32 pmu_osc_cnt;
		u32 pmu_plllock_cnt;
		u32 pmu_pllrst_cnt;
		u32 pmu_stable_cnt;
		u32 pmu_ddrio_pwron_cnt;
		u32 pmu_wakeup_rst_clr_cnt;
		u32 pmu_ddr_sref_st;
		u32 pmu_scu_l_pwrdn_cnt;
		u32 pmu_scu_l_pwrup_cnt;
		u32 pmu_scu_b_pwrdn_cnt;
		u32 pmu_scu_b_pwrup_cnt;
		u32 pmu_gpu_pwrdn_cnt;
		u32 pmu_gpu_pwrup_cnt;
		u32 pmu_center_pwrdn_cnt;
		u32 pmu_center_pwrup_cnt;
		u32 pmu_timeout_cnt;
		u32 pmu_cpu0apm_con;
		u32 pmu_cpu1apm_con;
		u32 pmu_cpu2apm_con;
		u32 pmu_cpu3apm_con;
		u32 pmu_cpu0bpm_con;
		u32 pmu_cpu1bpm_con;
		u32 pmu_noc_auto_ena;
		u32 pmu_pwrdn_con1;
		u32 reserved0[(0xf0 - 0xe0) / 4];
		u32 pmu_sys_reg_reg0;
		u32 pmu_sys_reg_reg1;
		u32 pmu_sys_reg_reg2;
		u32 pmu_sys_reg_reg3;
};

#define	PMU_BASE_ADDR			0xFF310000
__sramdata struct rk3399_pmu_regs *pmu_regs = (void *)PMU_BASE_ADDR;
/*************************************/

__sramfunc void rkclk_configure_ddr(void/*unsigned int hz*/)
{
	u32 refdiv,postdiv1,fbdiv;  //FREF越大，VCO越大，jitter就会小
	int delay = 1000;
	//u32 freq_mhz;

	#if 0
	freq_mhz = hz / MHz;
	refdiv = 1;
	if (freq_mhz <= 300) {
		postdiv1 = 6;
	} else if (freq_mhz <= 400) {
		postdiv1 = 4;
	} else if (freq_mhz <= 600) {
		postdiv1 = 3;
	} else if (freq_mhz <=900) {
		postdiv1 = 2;
	} else {
		postdiv1 = 1;
	}
	fbdiv=(freq_mhz * refdiv * postdiv1) / 24;
	#endif

	fbdiv = FBDIV_DEC(cru_con);
	postdiv1 = POSTDIV1_DEC(cru_con);
	refdiv = REFDIV_DEC(cru_con);
	
	writel_relaxed(PLL_MODE(PLL_SLOW_MODE),
					&rk3399_cru_reg->cru_pll_con[DPLL][3]);
	
	writel_relaxed(FBDIV(fbdiv), &rk3399_cru_reg->cru_pll_con[DPLL][0]);
	
	writel_relaxed(POSTDIV2(1) | POSTDIV1(postdiv1) | REFDIV(refdiv),
					&rk3399_cru_reg->cru_pll_con[DPLL][1]);
	
	while(delay > 0) {
		dmc_udelay(1);
		if (PLL_LOCK(readl_relaxed(&rk3399_cru_reg->cru_pll_con[DPLL][2])))
			break;
		delay --;
	}
	
	writel_relaxed(PLL_MODE(PLL_NORMAL_MODE),
					&rk3399_cru_reg->cru_pll_con[DPLL][3]);
	
	/* IC ECO bug, need to set this register */
	writel_relaxed(0xc000c000, &sgrf_regs->secure_grf_ddr_rgn_con16);
	
}

__sramfunc void rkclk_ddr_reset(u32 channel, u32 ctl, u32 phy)
{
	channel &= 0x1;
	ctl &= 0x1;
	phy &= 0x1;
	writel_relaxed(CRU_SFTRST_DDR_CTRL(channel, ctl) |
					CRU_SFTRST_DDR_PHY(channel, phy),
					&rk3399_cru_reg->cru_softreset_con[4]);
}

__sramfunc void phy_pctrl_reset(struct rk3399_ddr_publ_regs *ddr_publ_regs,
			    u32 channel)
{
	rkclk_ddr_reset(channel, 1, 1);
	dmc_udelay(10);
	rkclk_ddr_reset(channel, 1, 0);
	dmc_udelay(10);
	rkclk_ddr_reset(channel, 0, 0);
	dmc_udelay(10);
}

__sramfunc void phy_dll_bypass_set(u32 channel,
	struct rk3399_ddr_publ_regs *ddr_publ_regs, u32 hz)
{
	if (hz <= 125*MHz) {
		/*phy_sw_master_mode_X*/
		/*PHY_86/214/342/470 4bits offset_8*/
		setbits_le32(&ddr_publ_regs->denali_phy[86],
			     (0x3 << 2) << 8);
		setbits_le32(&ddr_publ_regs->denali_phy[214],
			     (0x3 << 2) << 8);
		setbits_le32(&ddr_publ_regs->denali_phy[342],
			     (0x3 << 2) << 8);
		setbits_le32(&ddr_publ_regs->denali_phy[470],
			     (0x3 << 2) << 8);
		/*phy_adrctl_sw_master_mode*/
		/*PHY_547/675/803 4bits offset_16*/
		setbits_le32(&ddr_publ_regs->denali_phy[547],
			     (0x3 << 2) << 16);
		setbits_le32(&ddr_publ_regs->denali_phy[675],
			     (0x3 << 2) << 16);
		setbits_le32(&ddr_publ_regs->denali_phy[803],
			     (0x3 << 2) << 16);
	} else {
		/*phy_sw_master_mode_X*/
		/*PHY_86/214/342/470 4bits offset_8*/
		clrbits_le32(&ddr_publ_regs->denali_phy[86],
			     (0x3 << 2) << 8);
		clrbits_le32(&ddr_publ_regs->denali_phy[214],
			     (0x3 << 2) << 8);
		clrbits_le32(&ddr_publ_regs->denali_phy[342],
			     (0x3 << 2) << 8);
		clrbits_le32(&ddr_publ_regs->denali_phy[470],
			     (0x3 << 2) << 8);
		/*phy_adrctl_sw_master_mode*/
		/*PHY_547/675/803 4bits offset_16*/
		clrbits_le32(&ddr_publ_regs->denali_phy[547],
			     (0x3 << 2) << 16);
		clrbits_le32(&ddr_publ_regs->denali_phy[675],
			     (0x3 << 2) << 16);
		clrbits_le32(&ddr_publ_regs->denali_phy[803],
			     (0x3 << 2) << 16);
	}
}

__sramfunc void set_memory_map(u32 channel,
		struct rk3399_sdram_params *sdram_params)
{
	const struct rk3399_sdram_channel *sdram_ch =
		&sdram_params->ch[channel];
	u32 *denali_ctl = rk3399_ddr_pctl[channel]->denali_ctl;
	u32 *denali_pi = rk3399_ddr_pi[channel]->denali_pi;
	u32 cs_map;
	u32 reduc;
	u32 row;

	if (sdram_ch->ddrconfig < 2)
		row = 16;
	else if ((sdram_ch->ddrconfig == 3) || (sdram_ch->ddrconfig == 5))
		row = 14;
	else
		row = 15;

	cs_map = (sdram_ch->rank > 1) ? 3 : 1;
	reduc = (sdram_ch->bw == 2) ? 0 : 1;

	clrsetbits_le32(&denali_ctl[191], 0xF, (12 - sdram_ch->col));
	clrsetbits_le32(&denali_ctl[190], (0x3 << 16) | (0x7 << 24),
			((3 - sdram_ch->bk) << 16) | ((16 - row) << 24));

	clrsetbits_le32(&denali_ctl[196], 0x3 | (1 << 16),
			cs_map | (reduc << 16));

	/* PI_199 PI_COL_DIFF:RW:0:4 */
	clrsetbits_le32(&denali_pi[199], 0xF, (12 - sdram_ch->col));

	/* PI_155 PI_ROW_DIFF:RW:24:3 PI_BANK_DIFF:RW:16:2 */
	clrsetbits_le32(&denali_pi[155], (0x3 << 16) | (0x7 << 24),
			((3 - sdram_ch->bk) << 16) | ((16 - row) << 24));
	if (sdram_params->dramtype == LPDDR4) {
		if(cs_map == 1)
			cs_map = 0x5;
		else if (cs_map == 2)
			cs_map = 0xa;
		else
			cs_map = 0xF;
	}
	/* PI_41 PI_CS_MAP:RW:24:4 */
	clrsetbits_le32(&denali_pi[41], 0xf << 24, cs_map << 24);
	if ((sdram_ch->rank == 1) && (sdram_params->dramtype == DDR3))
		write32(&denali_pi[34], 0x2EC7FFFF);
}

/*debug:can not enable dqs odt*/
#define PHY_DRV_ODT_Hi_Z	(0x0)
#define PHY_DRV_ODT_240	(0x1)
#define PHY_DRV_ODT_120	(0x8)
#define PHY_DRV_ODT_80	(0x9)
#define PHY_DRV_ODT_60	(0xc)
#define PHY_DRV_ODT_48	(0xd)
#define PHY_DRV_ODT_40	(0xe)
#define PHY_DRV_ODT_34_3	(0xf)

#if DMC_CAN_REMOVE_CODE
__sramfunc void set_ds_odt(u32 channel, struct rk3399_sdram_params *sdram_params)
{
	struct rk3399_ddr_publ_regs *ddr_publ_regs = rk3399_ddr_publ[channel];

	u32 tsel_idle_en, tsel_wr_en, tsel_rd_en;
	u32 tsel_idle_select, tsel_wr_select, tsel_rd_select;

	tsel_rd_select = PHY_DRV_ODT_240;
	tsel_wr_select = PHY_DRV_ODT_40;
	tsel_idle_select = PHY_DRV_ODT_240;

	if (sdram_params->odt == 1) {
		tsel_rd_en = 1;
		tsel_idle_en = 0;
	} else {
		tsel_rd_en = 0;
		tsel_idle_en = 0;
	}
	tsel_wr_en = 0;

	/*
	 * phy_dq_tsel_select_X 24bits DENALI_PHY_6/134/262/390 offset_0
	 * sets termination values for read/idle cycles and drive strength
	 * for write cycles for DQ/DM
	 */
	clrsetbits_le32(&ddr_publ_regs->denali_phy[6], 0xffffff,
			tsel_rd_select | (tsel_rd_select << 0x4) |
			(tsel_wr_select << 8) | (tsel_wr_select << 12) |
			(tsel_idle_select << 16) | (tsel_idle_select << 20));

	clrsetbits_le32(&ddr_publ_regs->denali_phy[134], 0xffffff,
			tsel_rd_select | (tsel_rd_select << 0x4) |
			(tsel_wr_select << 8) | (tsel_wr_select << 12) |
			(tsel_idle_select << 16) | (tsel_idle_select << 20));

	clrsetbits_le32(&ddr_publ_regs->denali_phy[262], 0xffffff,
			tsel_rd_select | (tsel_rd_select << 0x4) |
			(tsel_wr_select << 8) | (tsel_wr_select << 12) |
			(tsel_idle_select << 16) | (tsel_idle_select << 20));

	clrsetbits_le32(&ddr_publ_regs->denali_phy[390], 0xffffff,
			tsel_rd_select | (tsel_rd_select << 0x4) |
			(tsel_wr_select << 8) | (tsel_wr_select << 12) |
			(tsel_idle_select << 16) | (tsel_idle_select << 20));

	/*
	 * phy_dqs_tsel_select_X 24bits DENALI_PHY_7/135/263/391 offset_0
	 * sets termination values for read/idle cycles and drive strength
	 * for write cycles for DQS
	 */
	clrsetbits_le32(&ddr_publ_regs->denali_phy[7], 0xffffff,
			tsel_rd_select | (tsel_rd_select << 0x4) |
			(tsel_wr_select << 8) | (tsel_wr_select << 12) |
			(tsel_idle_select << 16) | (tsel_idle_select << 20));

	clrsetbits_le32(&ddr_publ_regs->denali_phy[135], 0xffffff,
			tsel_rd_select | (tsel_rd_select << 0x4) |
			(tsel_wr_select << 8) | (tsel_wr_select << 12) |
			(tsel_idle_select << 16) | (tsel_idle_select << 20));

	clrsetbits_le32(&ddr_publ_regs->denali_phy[263], 0xffffff,
			tsel_rd_select | (tsel_rd_select << 0x4) |
			(tsel_wr_select << 8) | (tsel_wr_select << 12) |
			(tsel_idle_select << 16) | (tsel_idle_select << 20));

	clrsetbits_le32(&ddr_publ_regs->denali_phy[391], 0xffffff,
			tsel_rd_select | (tsel_rd_select << 0x4) |
			(tsel_wr_select << 8) | (tsel_wr_select << 12) |
			(tsel_idle_select << 16) | (tsel_idle_select << 20));

	/*phy_adr_tsel_select_ 8bits DENALI_PHY_544/672/800 offset_0*/
	clrsetbits_le32(&ddr_publ_regs->denali_phy[544], 0xff,
			tsel_wr_select | (tsel_wr_select << 0x4));
	clrsetbits_le32(&ddr_publ_regs->denali_phy[672], 0xff,
			tsel_wr_select | (tsel_wr_select << 0x4));
	clrsetbits_le32(&ddr_publ_regs->denali_phy[800], 0xff,
			tsel_wr_select | (tsel_wr_select << 0x4));

	/*phy_pad_addr_drive 29bits DENALI_PHY_928 offset_0*/
	clrsetbits_le32((&ddr_publ_regs->denali_phy[928]), 0xff,
			tsel_wr_select | (tsel_wr_select << 0x4));

	/*phy_pad_rst_drive 8bits DENALI_PHY_937 offset_0*/
	clrsetbits_le32(&ddr_publ_regs->denali_phy[937], 0xff,
			tsel_wr_select | (tsel_wr_select << 0x4));

	/*phy_pad_cke_drive 8bits DENALI_PHY_935 offset_0*/
	clrsetbits_le32(&ddr_publ_regs->denali_phy[935], 0xff,
			tsel_wr_select | (tsel_wr_select << 0x4));

	/*phy_pad_cs_drive 8bits DENALI_PHY_939 offset_0*/
	clrsetbits_le32(&ddr_publ_regs->denali_phy[939], 0xff,
			tsel_wr_select | (tsel_wr_select << 0x4));

	/*phy_pad_clk_drive 8bits DENALI_PHY_929 offset_0*/
	clrsetbits_le32(&ddr_publ_regs->denali_phy[929], 0xff,
			tsel_wr_select | (tsel_wr_select << 0x4));

	/*phy_pad_fdbk_drive 23bit DENALI_PHY_924/925*/
	clrsetbits_le32(&ddr_publ_regs->denali_phy[924], 0xff,
			tsel_wr_select | (tsel_wr_select << 4));
	clrsetbits_le32(&ddr_publ_regs->denali_phy[925], 0xff,
			tsel_rd_select | (tsel_rd_select << 4));

	/*phy_dq_tsel_enable_X 3bits DENALI_PHY_5/133/261/389 offset_16*/
	clrsetbits_le32(&ddr_publ_regs->denali_phy[5], 0x7 << 16,
		(tsel_rd_en | (tsel_wr_en << 1) | (tsel_idle_en << 2)) << 16);
	clrsetbits_le32(&ddr_publ_regs->denali_phy[133], 0x7 << 16,
		(tsel_rd_en | (tsel_wr_en << 1) | (tsel_idle_en << 2)) << 16);
	clrsetbits_le32(&ddr_publ_regs->denali_phy[261], 0x7 << 16,
		(tsel_rd_en | (tsel_wr_en << 1) | (tsel_idle_en << 2)) << 16);
	clrsetbits_le32(&ddr_publ_regs->denali_phy[389], 0x7 << 16,
		(tsel_rd_en | (tsel_wr_en << 1) | (tsel_idle_en << 2)) << 16);

	/*phy_dqs_tsel_enable_X 3bits DENALI_PHY_6/134/262/390 offset_24*/
	clrsetbits_le32(&ddr_publ_regs->denali_phy[6], 0x7 << 24,
			(tsel_rd_en | (tsel_wr_en << 1) |
			(tsel_idle_en << 2)) << 24);
	clrsetbits_le32(&ddr_publ_regs->denali_phy[134], 0x7 << 24,
			(tsel_rd_en | (tsel_wr_en << 1) |
			(tsel_idle_en << 2)) << 24);
	clrsetbits_le32(&ddr_publ_regs->denali_phy[262], 0x7 << 24,
			(tsel_rd_en | (tsel_wr_en << 1) |
			(tsel_idle_en << 2)) << 24);
	clrsetbits_le32(&ddr_publ_regs->denali_phy[390], 0x7 << 24,
			(tsel_rd_en | (tsel_wr_en << 1) |
			(tsel_idle_en << 2)) << 24);

	/* phy_adr_tsel_enable_ 1bit DENALI_PHY_518/646/774 offset_8 */
	clrsetbits_le32(&ddr_publ_regs->denali_phy[518],
			0x1 << 8, tsel_wr_en << 8);
	clrsetbits_le32(&ddr_publ_regs->denali_phy[646],
			0x1 << 8, tsel_wr_en << 8);
	clrsetbits_le32(&ddr_publ_regs->denali_phy[774],
			0x1 << 8, tsel_wr_en << 8);

	/*phy_pad_addr_term tsel 1bit DENALI_PHY_933 offset_17*/
	clrsetbits_le32((&ddr_publ_regs->denali_phy[933]),
			0x1 << 17, tsel_wr_en << 17);

	/*
	 * pad_rst/cke/cs/clk_term tsel 1bits
	 * DENALI_PHY_938/936/940/934 offset_17
	 */
	clrsetbits_le32(&ddr_publ_regs->denali_phy[938],
			0x1 << 17, tsel_wr_en << 17);
	clrsetbits_le32(&ddr_publ_regs->denali_phy[936],
			0x1 << 17, tsel_wr_en << 17);
	clrsetbits_le32(&ddr_publ_regs->denali_phy[940],
			0x1 << 17, tsel_wr_en << 17);
	clrsetbits_le32(&ddr_publ_regs->denali_phy[934],
			0x1 << 17, tsel_wr_en << 17);

	/*phy_pad_fdbk_term 1bit DENALI_PHY_930 offset_17*/
	clrsetbits_le32(&ddr_publ_regs->denali_phy[930],
			0x1 << 17, tsel_wr_en << 17);
}
#endif

#if DMC_DO_DATA_TRAINING
static __sramfunc void select_per_cs_training_index(u32 channel, u32 rank)
{
	u32 *denali_phy = rk3399_ddr_publ[channel]->denali_phy;

	/* PHY_84 PHY_PER_CS_TRAINING_EN_0 1bit offset_16 */
	if ((read32(&denali_phy[84])>>16) & 1) {
		/*
		 * PHY_8/136/264/392
		 * phy_per_cs_training_index_X 1bit offset_24
		 */
		clrsetbits_le32(&denali_phy[8], 0x1 << 24, rank << 24);
		clrsetbits_le32(&denali_phy[136], 0x1 << 24, rank << 24);
		clrsetbits_le32(&denali_phy[264], 0x1 << 24, rank << 24);
		clrsetbits_le32(&denali_phy[392], 0x1 << 24, rank << 24);
	}
}

/*
 * After write leveling for all ranks, check the PHY_CLK_WRDQS_SLAVE_DELAY
 * result, if the two ranks in one slice both met
 * "0x200-PHY_CLK_WRDQS_SLAVE_DELAY < 0x20 or
 * 0x200-PHY_CLK_WRDQS_SLAVE > 0x1E0",
 * enable PHY_WRLVL_EARLY_FORCE_ZERO for this slice, and trigger write
 * leveling again. Else no additional write leveling is required
 */
static __sramfunc void check_write_leveling_value(u32 channel,
	struct rk3399_sdram_params *sdram_params)
{
	u32 *denali_pi = rk3399_ddr_pi[channel]->denali_pi;
	u32 *denali_phy = rk3399_ddr_publ[channel]->denali_phy;
	u32 i, tmp;
	u32 obs_0, obs_1, obs_2, obs_3, obs_err = 0;
	u32 wl_value[2][4];
	u32 rank = sdram_params->ch[channel].rank;

	/* PHY_84 PHY_PER_CS_TRAINING_EN_0 1bit offset_16 */
	if (!((read32(&denali_phy[84])>>16) & 1))
		return;

	for (i = 0; i < rank; i++) {
		/*
		 * PHY_8/136/264/392
		 * phy_per_cs_training_index_X 1bit offset_24
		 */
		clrsetbits_le32(&denali_phy[8], 0x1 << 24, i << 24);
		clrsetbits_le32(&denali_phy[136], 0x1 << 24, i << 24);
		clrsetbits_le32(&denali_phy[264], 0x1 << 24, i << 24);
		clrsetbits_le32(&denali_phy[392], 0x1 << 24, i << 24);
		wl_value[i][0] = (read32(&denali_phy[63]) >> 16) & 0x3ff;
		wl_value[i][1] = (read32(&denali_phy[191]) >> 16) & 0x3ff;
		wl_value[i][2] = (read32(&denali_phy[319]) >> 16) & 0x3ff;
		wl_value[i][3] = (read32(&denali_phy[447]) >> 16) & 0x3ff;
	}

	for (i = 0; i < 4; i++) {
		if (((wl_value[0][i] > 0x1E0) || (wl_value[0][i] < 0x20)) &&
		    ((wl_value[1][i] > 0x1E0) || (wl_value[1][i] < 0x20))) {
			switch (i) {
			case 0:
				clrsetbits_le32(&denali_phy[8], 0x1 << 24, 0 << 24);
				setbits_le32(&denali_phy[79], 0x1 << 16);
				clrsetbits_le32(&denali_phy[8], 0x1 << 24, 1<< 24);
				setbits_le32(&denali_phy[79], 0x1 << 16);
				break;
			case 1:
				clrsetbits_le32(&denali_phy[136], 0x1 << 24, 0 << 24);
				setbits_le32(&denali_phy[207], 0x1 << 16);
				clrsetbits_le32(&denali_phy[136], 0x1 << 24, 1 << 24);
				setbits_le32(&denali_phy[207], 0x1 << 16);
				break;
			case 2:
				clrsetbits_le32(&denali_phy[264], 0x1 << 24, 0 << 24);
				setbits_le32(&denali_phy[335], 0x1 << 16);
				clrsetbits_le32(&denali_phy[264], 0x1 << 24, 1 << 24);
				setbits_le32(&denali_phy[335], 0x1 << 16);
				break;
			case 3:
				clrsetbits_le32(&denali_phy[392], 0x1 << 24, 0 << 24);
				setbits_le32(&denali_phy[463], 0x1 << 16);
				clrsetbits_le32(&denali_phy[392], 0x1 << 24, 1 << 24);
				setbits_le32(&denali_phy[463], 0x1 << 16);
				break;
			default:
				break;
			}
		}
	}

	for (i = 0; i < rank; i++) {
		/* PI_60 PI_WRLVL_EN:RW:8:2 */
		clrsetbits_le32(&denali_pi[60], 0x3 << 8, 0x2 << 8);
		/* PI_59 PI_WRLVL_REQ:WR:8:1,PI_WRLVL_CS:RW:16:2 */
		clrsetbits_le32(&denali_pi[59], (0x1 << 8) | (0x3 << 16),
				(0x1 << 8) | (i << 16));

		select_per_cs_training_index(channel, i);
		while (1) {
			/* PI_174 PI_INT_STATUS:RD:8:25 */
			tmp = read32(&denali_pi[174]) >> 8;

			/*
			 * check status obs,
			 * if error maybe can not get leveling done
			 * PHY_40/168/296/424 phy_wrlvl_status_obs_X:0:13
			 */
			obs_0 = read32(&denali_phy[40]);
			obs_1 = read32(&denali_phy[168]);
			obs_2 = read32(&denali_phy[296]);
			obs_3 = read32(&denali_phy[424]);
			if (((obs_0 >> 12) & 0x1) ||
			    ((obs_1 >> 12) & 0x1) ||
			    ((obs_2 >> 12) & 0x1) ||
			    ((obs_3 >> 12) & 0x1))
				obs_err = 1;
			if ((((tmp >> 10) & 0x1) == 0x1) &&
			    (((tmp >> 13) & 0x1) == 0x1) &&
			    (((tmp >> 4) & 0x1) == 0x0) &&
			    (obs_err == 0))
				break;
			else if ((((tmp >> 4) & 0x1) == 0x1) ||
				 (obs_err == 1))
				 ;
				 //print("check_write_leving_value error!!!\n");
		}
		/* clear interrupt,PI_175 PI_INT_ACK:WR:0:17 */
		write32((&denali_pi[175]), 0x00003f7c);
	}
}

__sramfunc int data_training(u32 channel,
	struct rk3399_sdram_params *sdram_params, u32 training_flag)
{
	uart_wrtie_byte('x');

	u32 *denali_pi = rk3399_ddr_pi[channel]->denali_pi;
	
	uart_wrtie_byte('y');
	u32 *denali_phy = rk3399_ddr_publ[channel]->denali_phy;
	
	uart_wrtie_byte('z');
	u32 i, tmp;
	u32 obs_0, obs_1, obs_2, obs_3, obs_err = 0;
	u32 rank = sdram_params->ch[channel].rank;
	u32 rank_mask;
	uart_wrtie_byte('R');

	if (sdram_params->dramtype == LPDDR4) {
		rank_mask = (rank == 1) ? 0x5 : 0xf;
	} else {
		rank_mask = (rank == 1) ? 0x1 : 0x3;
	}
	
	uart_wrtie_byte('S');
	/* PHY_927 PHY_PAD_DQS_DRIVE  RPULL offset_22 */
	setbits_le32(&denali_phy[927], (1 << 22));
	uart_wrtie_byte('O');

	if (training_flag == PI_FULL_TARINING) {
		if (sdram_params->dramtype == LPDDR4) {
			training_flag = PI_WRITE_LEVELING |
					PI_READ_GATE_TRAINING |
					PI_READ_LEVELING |
					PI_WDQ_LEVELING;
		} else if (sdram_params->dramtype == LPDDR3) {
			training_flag = PI_CA_TRAINING | PI_WRITE_LEVELING |
					PI_READ_GATE_TRAINING |
					PI_READ_LEVELING;
		} else if (sdram_params->dramtype == DDR3) {
			training_flag = PI_WRITE_LEVELING |
					PI_READ_GATE_TRAINING |
					PI_READ_LEVELING;
		}
	}
	uart_wrtie_byte('p');

	/* ca training(LPDDR4,LPDDR3 support) */
	if ((training_flag & PI_CA_TRAINING) == PI_CA_TRAINING) {
		for (i = 0; i < 4; i++) {
			if (!(rank_mask & (1 << i)))
				continue;
			/* PI_100 PI_CALVL_EN:RW:8:2 */
			clrsetbits_le32(&denali_pi[100], 0x3 << 8, 0x2 << 8);
			uart_wrtie_byte('j');

			/* PI_92 PI_CALVL_REQ:WR:16:1,PI_CALVL_CS:RW:24:2 */
			clrsetbits_le32(&denali_pi[92],
					(0x1 << 16) | (0x3 << 24),
					(0x1 << 16) | (i << 24));

			select_per_cs_training_index(channel, i);
			while (1) {
				uart_wrtie_byte('1');
				/* PI_174 PI_INT_STATUS:RD:8:18 */
				tmp = read32(&denali_pi[174]) >> 8;

				/*
				 * check status obs
				 * PHY_532/660/788 phy_adr_calvl_obs1_:0:32
				 */
				obs_0 = read32(&denali_phy[532]);
				obs_1 = read32(&denali_phy[660]);
				obs_2 = read32(&denali_phy[788]);
				if (((obs_0 >> 30) & 0x3) ||
				    ((obs_1 >> 30) & 0x3) ||
				    ((obs_2 >> 30) & 0x3))
					obs_err = 1;
				if ((((tmp >> 11) & 0x1) == 0x1) &&
				    (((tmp >> 13) & 0x1) == 0x1) &&
				    (((tmp >> 5) & 0x1) == 0x0) &&
				    (obs_err == 0))
					break;
				else if ((((tmp >> 5) & 0x1) == 0x1) ||
					 (obs_err == 1))
					return -1;
			}
			/* clear interrupt,PI_175 PI_INT_ACK:WR:0:17 */
			write32((&denali_pi[175]), 0x00003f7c);
		}
	}
	uart_wrtie_byte('k');

	/* write leveling(LPDDR4,LPDDR3,DDR3 support) */
	if ((training_flag & PI_WRITE_LEVELING) == PI_WRITE_LEVELING) {
		
		uart_wrtie_byte('l');
		for (i = 0; i < rank; i++) {
			/* PI_60 PI_WRLVL_EN:RW:8:2 */
			clrsetbits_le32(&denali_pi[60], 0x3 << 8, 0x2 << 8);
			/* PI_59 PI_WRLVL_REQ:WR:8:1,PI_WRLVL_CS:RW:16:2 */
			clrsetbits_le32(&denali_pi[59],
					(0x1 << 8) | (0x3 << 16),
					(0x1 << 8) | (i << 16));

			select_per_cs_training_index(channel, i);
			while (1) {
				uart_wrtie_byte('2');
				/* PI_174 PI_INT_STATUS:RD:8:18 */
				tmp = read32(&denali_pi[174]) >> 8;

				/*
				 * check status obs, if error maybe can not
				 * get leveling done PHY_40/168/296/424
				 * phy_wrlvl_status_obs_X:0:13
				 */
				obs_0 = read32(&denali_phy[40]);
				obs_1 = read32(&denali_phy[168]);
				obs_2 = read32(&denali_phy[296]);
				obs_3 = read32(&denali_phy[424]);
				if (((obs_0 >> 12) & 0x1) ||
				    ((obs_1 >> 12) & 0x1) ||
				    ((obs_2 >> 12) & 0x1) ||
				    ((obs_3 >> 12) & 0x1))
					obs_err = 1;
				if ((((tmp >> 10) & 0x1) == 0x1) &&
				    (((tmp >> 13) & 0x1) == 0x1) &&
				    (((tmp >> 4) & 0x1) == 0x0) &&
				    (obs_err == 0)) {
					if ((rank == 2) && (i == 1))
						check_write_leveling_value
							(channel, sdram_params);
					break;
				} else if ((((tmp >> 4) & 0x1) == 0x1) ||
					 (obs_err == 1))
					return -1;
			}
			/* clear interrupt,PI_175 PI_INT_ACK:WR:0:17 */
			write32((&denali_pi[175]), 0x00003f7c);
		}
	}

	/* read gate training(LPDDR4,LPDDR3,DDR3 support) */
	if ((training_flag & PI_READ_GATE_TRAINING) == PI_READ_GATE_TRAINING) {
		
		uart_wrtie_byte('m');
		for (i = 0; i < rank; i++) {
			/* PI_80 PI_RDLVL_GATE_EN:RW:24:2 */
			clrsetbits_le32(&denali_pi[80], 0x3 << 24, 0x2 << 24);
			/*
			 * PI_74 PI_RDLVL_GATE_REQ:WR:16:1
			 * PI_RDLVL_CS:RW:24:2
			 */
			clrsetbits_le32(&denali_pi[74],
					(0x1 << 16) | (0x3 << 24),
					(0x1 << 16) | (i << 24));

			select_per_cs_training_index(channel, i);
			while (1) {
				uart_wrtie_byte('3');
				/* PI_174 PI_INT_STATUS:RD:8:18 */
				tmp = read32(&denali_pi[174]) >> 8;

				/*
				 * check status obs
				 * PHY_43/171/299/427
				 *     PHY_GTLVL_STATUS_OBS_x:16:8
				 */
				obs_0 = read32(&denali_phy[43]);
				obs_1 = read32(&denali_phy[171]);
				obs_2 = read32(&denali_phy[299]);
				obs_3 = read32(&denali_phy[427]);
				if (((obs_0 >> (16 + 6)) & 0x3) ||
				    ((obs_1 >> (16 + 6)) & 0x3) ||
				    ((obs_2 >> (16 + 6)) & 0x3) ||
				    ((obs_3 >> (16 + 6)) & 0x3))
					obs_err = 1;
				if ((((tmp >> 9) & 0x1) == 0x1) &&
				    (((tmp >> 13) & 0x1) == 0x1) &&
				    (((tmp >> 3) & 0x1) == 0x0) &&
				    (obs_err == 0))
					break;
				else if ((((tmp >> 3) & 0x1) == 0x1) ||
					 (obs_err == 1))
					return -1;
			}
			/* clear interrupt,PI_175 PI_INT_ACK:WR:0:17 */
			write32((&denali_pi[175]), 0x00003f7c);
		}
	}

	/* read leveling(LPDDR4,LPDDR3,DDR3 support) */
	if ((training_flag & PI_READ_LEVELING) == PI_READ_LEVELING) {
		
		uart_wrtie_byte('n');
		for (i = 0; i < rank; i++) {
			/* PI_80 PI_RDLVL_EN:RW:16:2 */
			clrsetbits_le32(&denali_pi[80], 0x3 << 16, 0x2 << 16);
			/* PI_74 PI_RDLVL_REQ:WR:8:1,PI_RDLVL_CS:RW:24:2 */
			clrsetbits_le32(&denali_pi[74],
					(0x1 << 8) | (0x3 << 24),
					(0x1 << 8) | (i << 24));

			select_per_cs_training_index(channel, i);
			while (1) {
				uart_wrtie_byte('4');
				/* PI_174 PI_INT_STATUS:RD:8:18 */
				tmp = read32(&denali_pi[174]) >> 8;

				/*
				 * make sure status obs not report error bit
				 * PHY_46/174/302/430
				 *     phy_rdlvl_status_obs_X:16:8
				 */
				if ((((tmp >> 8) & 0x1) == 0x1) &&
				    (((tmp >> 13) & 0x1) == 0x1) &&
				    (((tmp >> 2) & 0x1) == 0x0))
					break;
				else if (((tmp >> 2) & 0x1) == 0x1)
					return -1;
			}
			/* clear interrupt,PI_175 PI_INT_ACK:WR:0:17 */
			write32((&denali_pi[175]), 0x00003f7c);
		}
	}

	/* wdq leveling(LPDDR4 support) */
	if ((training_flag & PI_WDQ_LEVELING) == PI_WDQ_LEVELING) {
		for (i = 0; i < 4; i++) {
			if (!(rank_mask & (1 << i)))
				continue;
			/*
			 * disable PI_WDQLVL_VREF_EN before wdq leveling?
			 * PI_181 PI_WDQLVL_VREF_EN:RW:8:1
			 */
			clrbits_le32(&denali_pi[181], 0x1 << 8);
			/* PI_124 PI_WDQLVL_EN:RW:16:2 */
			clrsetbits_le32(&denali_pi[124], 0x3 << 16, 0x2 << 16);
			/* PI_121 PI_WDQLVL_REQ:WR:8:1,PI_WDQLVL_CS:RW:16:2 */
			clrsetbits_le32(&denali_pi[121],
					(0x1 << 8) | (0x3 << 16),
					(0x1 << 8) | (i << 16));

			select_per_cs_training_index(channel, i);
			while (1) {
				uart_wrtie_byte('5');
				/* PI_174 PI_INT_STATUS:RD:8:18 */
				tmp = read32(&denali_pi[174]) >> 8;
				if ((((tmp >> 12) & 0x1) == 0x1) &&
				    (((tmp >> 13) & 0x1) == 0x1) &&
				    (((tmp >> 6) & 0x1) == 0x0))
					break;
				else if (((tmp >> 6) & 0x1) == 0x1)
					return -1;
			}
			/* clear interrupt,PI_175 PI_INT_ACK:WR:0:17 */
			write32((&denali_pi[175]), 0x00003f7c);
		}
	}

	/* PHY_927 PHY_PAD_DQS_DRIVE  RPULL offset_22 */
	clrbits_le32(&denali_phy[927], (1 << 22));

	return 0;
}
#endif

__sramfunc void set_ddrconfig(struct rk3399_sdram_params *sdram_params,
			  unsigned char channel, u32 ddrconfig)
{
	/* only need to set ddrconfig */
	struct rk3399_msch_regs *ddr_msch_regs = rk3399_msch[channel];
	u32 *denali_ctl = rk3399_ddr_pctl[channel]->denali_ctl;
	u32 *denali_pi = rk3399_ddr_pi[channel]->denali_pi;
	unsigned int cs0_cap = 0;
	unsigned int cs1_cap = 0;
	unsigned int row;

	cs0_cap = (1 << (sdram_params->ch[channel].cs0_row
			+ sdram_params->ch[channel].col
			+ sdram_params->ch[channel].bk
			+ sdram_params->ch[channel].bw - 20));
	if (sdram_params->ch[channel].rank > 1)
		cs1_cap = cs0_cap >> (sdram_params->ch[channel].cs0_row
				- sdram_params->ch[channel].cs1_row);
	if (sdram_params->ch[channel].row_3_4) {
		cs0_cap = cs0_cap * 3 / 4;
		cs1_cap = cs1_cap * 3 / 4;
	}

	write32(&ddr_msch_regs->ddrconf, ddrconfig | (ddrconfig << 8));
	write32(&ddr_msch_regs->ddrsize, ((cs0_cap / 32) & 0xff) |
					 (((cs1_cap / 32) & 0xff) << 8));

	if (ddrconfig < 2)
		row = 16;
	else if ((ddrconfig == 3) || (ddrconfig == 5))
		row = 14;
	else
		row = 15;

	clrsetbits_le32(&denali_ctl[190], (0x7 << 24), ((16 - row) << 24));
	/* PI_155 PI_ROW_DIFF:RW:24:3 PI_BANK_DIFF:RW:16:2 */
	clrsetbits_le32(&denali_pi[155], (0x7 << 24), ((16 - row) << 24));
}

__sramfunc void ddr_move_to_access_state(u32 channel)
{
}

__sramfunc void dram_all_config(struct rk3399_sdram_params *sdram_params)
{
	u32 sys_reg = 0;
	unsigned int channel;
	unsigned int use;
	struct rk3399_msch_regs *ddr_msch_regs;
	struct rk3399_msch_timings *noc_timing;

	sys_reg |= SYS_REG_ENC_DDRTYPE(sdram_params->dramtype);
	sys_reg |= SYS_REG_ENC_NUM_CH(sdram_params->num_channels);
	for (channel = 0, use = 0;
	     (use < sdram_params->num_channels) && (channel < 2); channel++) {
		struct rk3399_ddr_pctl_regs *ddr_pctl_regs =
		    rk3399_ddr_pctl[channel];
		struct rk3399_sdram_channel *info =
			&sdram_params->ch[channel];
		ddr_msch_regs = rk3399_msch[channel];

		if (sdram_params->ch[channel].col == 0)
			continue;
		use++;
		sys_reg |= SYS_REG_ENC_ROW_3_4(info->row_3_4, channel);
		sys_reg |= SYS_REG_ENC_CHINFO(channel);
		sys_reg |= SYS_REG_ENC_RANK(info->rank, channel);
		sys_reg |= SYS_REG_ENC_COL(info->col, channel);
		sys_reg |= SYS_REG_ENC_BK(info->bk, channel);
		sys_reg |= SYS_REG_ENC_CS0_ROW(info->cs0_row, channel);
		if (sdram_params->ch[channel].rank > 1)
			sys_reg |= SYS_REG_ENC_CS1_ROW(info->cs1_row, channel);
		sys_reg |= SYS_REG_ENC_BW(info->bw, channel);
		sys_reg |= SYS_REG_ENC_DBW(info->dbw, channel);

		noc_timing = &sdram_params->ch[channel].noc_timings;
		write32(&ddr_msch_regs->ddrtiminga0.d32,
			noc_timing->ddrtiminga0.d32);
		write32(&ddr_msch_regs->ddrtimingb0.d32,
			noc_timing->ddrtimingb0.d32);
		write32(&ddr_msch_regs->ddrtimingc0.d32,
			noc_timing->ddrtimingc0.d32);
		write32(&ddr_msch_regs->devtodev0.d32,
			noc_timing->devtodev0.d32);
		write32(&ddr_msch_regs->ddrmode.d32,
			noc_timing->ddrmode.d32);

		/* rank 1 memory clock disable (dfi_dram_clk_disable = 1) */
		if (sdram_params->ch[channel].rank == 1)
			setbits_le32(&ddr_pctl_regs->denali_ctl[276], 1 << 17);
	}

	write32(&(pmugrf_regs->pmugrf_os_reg[2]), sys_reg);
	DDR_STRIDE(sdram_params->stride);

	/* reboot hold register set */
	writel_relaxed(PRESET_SGRF_HOLD(0) | PRESET_GPIO0_HOLD(1) |
					PRESET_GPIO1_HOLD(1),
					PMU_CRU_BASE_ADDR + PMUCRU_RSTNHOLD_CON1);
	clrsetbits_le32(&rk3399_cru_reg->cru_glb_rst_con,
					0x3, 0x3);
}

__sramdata u32 flag=0;

__sramfunc void pctl_cfg(u32 channel,
		     struct rk3399_sdram_params *sdram_params)
{
	struct rk3399_ddr_pctl_regs *ddr_pctl_regs = rk3399_ddr_pctl[channel];
	struct rk3399_ddr_pi_regs *ddr_pi_regs = rk3399_ddr_pi[channel];
	struct rk3399_ddr_publ_regs *ddr_publ_regs = rk3399_ddr_publ[channel];
	u32 tmp, tmp1, tmp2;
	#if !(DMC_PWRUP_SREFRESH_EXIT)
	u32 pwrup_srefresh_exit;
	#endif
	#if DMC_DDR3_SUPPORT
	u32 rank;
	#endif
	u32 i;
	u32 count = 0;
	/* workaround controller bug:
	 * Do not program DRAM_CLASS until NO_PHY_IND_TRAIN_INT is programmed
	 */
	dmc_memcpy_iram(&ddr_pctl_regs->denali_ctl[1],
		(u32 *)&sdram_params->pctl_regs.denali_ctl[1],
		sizeof(struct rk3399_ddr_pctl_regs) - 4);
	write32(&ddr_pctl_regs->denali_ctl[0],
		sdram_params->pctl_regs.denali_ctl[0]);
	dmc_memcpy_iram((u32 *)ddr_pi_regs,
		(u32 *)&sdram_params->pi_regs.denali_pi[0],
		sizeof(struct rk3399_ddr_pi_regs));
	/* rank count need to set for init */
	#if DMC_DDR3_SUPPORT
	/* for temporary fix DDR3 1 cs training loop endless */
	if (sdram_params->dramtype == DDR3) {
		rank = sdram_params->ch[channel].rank;
		sdram_params->ch[channel].rank = 2;
	}
	#endif
	set_memory_map(channel, sdram_params);

	write32(&ddr_publ_regs->denali_phy[910], 0x6400);
	write32(&ddr_publ_regs->denali_phy[911], 0x01221102);
	write32(&ddr_publ_regs->denali_phy[912], 0x0);
	#if !(DMC_PWRUP_SREFRESH_EXIT)
	pwrup_srefresh_exit = read32(&ddr_pctl_regs->denali_ctl[68]) &
			      PWRUP_SREFRESH_EXIT;
	clrbits_le32(&ddr_pctl_regs->denali_ctl[68], PWRUP_SREFRESH_EXIT);
	#endif

	/* PHY_DLL_RST_EN */
	clrsetbits_le32(&ddr_publ_regs->denali_phy[957],
			0x3 << 24, 1 << 24);

	setbits_le32(&ddr_pi_regs->denali_pi[0], START);
	setbits_le32(&ddr_pctl_regs->denali_ctl[0], START);
	/* wait lock */
	while (1) {
		tmp = read32(&ddr_publ_regs->denali_phy[920]);
		tmp1 = read32(&ddr_publ_regs->denali_phy[921]);
		tmp2 = read32(&ddr_publ_regs->denali_phy[922]);
		if ((((tmp >> 16) & 0x1) == 0x1) &&
		     (((tmp1 >> 16) & 0x1) == 0x1) &&
		     (((tmp1 >> 0) & 0x1) == 0x1) &&
		     (((tmp2 >> 0) & 0x1) == 0x1))
			break;
		uart_wrtie_byte('z');
		uart_wrtie_byte('Z');
	}

	#if DMC_PHY_REGS
	dmc_memcpy_iram((u32 *)&ddr_publ_regs->denali_phy[896],
		    (u32 *)&sdram_params->phy_regs.denali_phy[896],
		    (958 - 895) * 4);
	dmc_memcpy_iram((u32 *)&ddr_publ_regs->denali_phy[0],
		    (u32 *)&sdram_params->phy_regs.denali_phy[0],
		    (90 - 0 + 1) * 4);
	dmc_memcpy_iram((u32 *)&ddr_publ_regs->denali_phy[128],
		    (u32 *)&sdram_params->phy_regs.denali_phy[128],
		    (218 - 128 + 1) * 4);
	dmc_memcpy_iram((u32 *)&ddr_publ_regs->denali_phy[256],
		    (u32 *)&sdram_params->phy_regs.denali_phy[256],
		    (346 - 256 + 1) * 4);
	dmc_memcpy_iram((u32 *)&ddr_publ_regs->denali_phy[384],
		    (u32 *)&sdram_params->phy_regs.denali_phy[384],
		    (474 - 384 + 1) * 4);
	dmc_memcpy_iram((u32 *)&ddr_publ_regs->denali_phy[512],
		    (u32 *)&sdram_params->phy_regs.denali_phy[512],
		    (549 - 512 + 1) * 4);
	dmc_memcpy_iram((u32 *)&ddr_publ_regs->denali_phy[640],
		    (u32 *)&sdram_params->phy_regs.denali_phy[640],
		    (677 - 640 + 1) * 4);
	dmc_memcpy_iram((u32 *)&ddr_publ_regs->denali_phy[768],
		    (u32 *)&sdram_params->phy_regs.denali_phy[768],
		    (805 - 768 + 1) * 4);
	#else
	dmc_memcpy_iram((u32 *)&ddr_publ_regs->denali_phy[phy_regs_map_restore[(PHY_REGS_GROUP_RESTORE-1)][0]],
		    (u32 *)&sdram_params->phy_regs.denali_phy[phy_regs_map_restore[(PHY_REGS_GROUP_RESTORE-1)][1]],
		    phy_regs_map_restore[(PHY_REGS_GROUP_RESTORE-1)][2]* 4);

	for (i=0;i<(PHY_REGS_GROUP_RESTORE-1);i++) {
	dmc_memcpy_iram((u32 *)&ddr_publ_regs->denali_phy[phy_regs_map_restore[i][0]],
		(u32 *)&sdram_params->phy_regs.denali_phy[phy_regs_map_restore[i][1]],
		phy_regs_map_restore[i][2]* 4);
	}	

	#endif

	#if DMC_CAN_REMOVE_CODE
	set_ds_odt(channel, sdram_params);
	#endif
	
	#if DMC_CAN_REMOVE_CODE
	/*phy_dqs_tsel_wr_timing_X 8bits DENALI_PHY_84/212/340/468 offset_8*/
	/*dqs_tsel_wr_end[7:4] add Half cycle*/
	tmp = (read32(&ddr_publ_regs->denali_phy[84]) >> 8) & 0xff;
	clrsetbits_le32((&ddr_publ_regs->denali_phy[84]),
			0xff << 8, (tmp + 0x10) << 8);
	tmp = (read32(&ddr_publ_regs->denali_phy[212]) >> 8) & 0xff;
	clrsetbits_le32(&ddr_publ_regs->denali_phy[212],
			0xff << 8, (tmp + 0x10) << 8);
	tmp = (read32(&ddr_publ_regs->denali_phy[340]) >> 8) & 0xff;
	clrsetbits_le32(&ddr_publ_regs->denali_phy[340],
			0xff << 8, (tmp + 0x10) << 8);
	tmp = (read32(&ddr_publ_regs->denali_phy[468]) >> 8) & 0xff;
	clrsetbits_le32(&ddr_publ_regs->denali_phy[468],
			0xff << 8, (tmp + 0x10) << 8);

	/*phy_dqs_tsel_wr_timing_X 8bits DENALI_PHY_83/211/339/467 offset_8*/
	/*dq_tsel_wr_end[7:4] add Half cycle*/
	tmp = (read32(&ddr_publ_regs->denali_phy[83]) >> 16) & 0xff;
	clrsetbits_le32(&ddr_publ_regs->denali_phy[83],
			0xff << 16, (tmp + 0x10) << 16);
	tmp = (read32(&ddr_publ_regs->denali_phy[211]) >> 16) & 0xff;
	clrsetbits_le32(&ddr_publ_regs->denali_phy[211],
			0xff << 16, (tmp + 0x10) << 16);
	tmp = (read32(&ddr_publ_regs->denali_phy[339]) >> 16) & 0xff;
	clrsetbits_le32(&ddr_publ_regs->denali_phy[339],
			0xff << 16, (tmp + 0x10) << 16);
	tmp = (read32(&ddr_publ_regs->denali_phy[467]) >> 16) & 0xff;
	clrsetbits_le32(&ddr_publ_regs->denali_phy[467],
			0xff << 16, (tmp + 0x10) << 16);
	#endif

	/* PHY_DLL_RST_EN */
	clrsetbits_le32(&ddr_publ_regs->denali_phy[957],
			0x3 << 24, 0x2 << 24);

	#if 1 //need de-access IO retention before controller START???
	if (channel == 0) {
		setbits_le32(&pmu_regs->pmu_pwrmode_con, (0x1 << 19));
	}
	else if (channel == 1) {
		setbits_le32(&pmu_regs->pmu_pwrmode_con, (0x1 << 23));
	}
	#endif

	/* fix me
	 * need to care ERROR bit
	 */
	while (!(read32(&ddr_pctl_regs->denali_ctl[203]) & (1 << 3))) {
		if(count < 1000) {	
			uart_wrtie_byte('O');
			uart_wrtie_byte('M');
			uart_wrtie_byte('G');
		}
		count ++;
		}

	#if 0
	if (channel == 0) {
		setbits_le32(&pmu_regs->pmu_pwrmode_con, (0x1 << 19));
	}
	else if (channel == 1) {
		setbits_le32(&pmu_regs->pmu_pwrmode_con, (0x1 << 23));
	}
	/*
	while(read32(&ddr_pctl_regs->denali_ctl[200]) & 0x1);
	clrsetbits_le32(&ddr_pctl_regs->denali_ctl[93], 0xFF << 24,
		0x69 << 24);
	while(((read32(&ddr_pctl_regs->denali_ctl[100]) >> 24) & 0x7f) != 0x40);
	*/
	#endif

	
	#if !(DMC_PWRUP_SREFRESH_EXIT)
	clrsetbits_le32(&ddr_pctl_regs->denali_ctl[68],
			PWRUP_SREFRESH_EXIT,
			pwrup_srefresh_exit);
	#endif

	#if DMC_DDR3_SUPPORT
	/* for temporary fix DDR3 1 cs training loop endless */
	if (sdram_params->dramtype == DDR3) {
		sdram_params->ch[channel].rank = rank;
		set_memory_map(channel, sdram_params);
	}
	#endif
}

__sramfunc struct rk3399_sdram_params *get_sdram_config()
{
	return &sdram_configs[0];
}

struct rk3399_sdram_params *get_sdram_config_dram()
{
	return &sdram_configs[0];
}

void dmc_save(void)
{	
	int channel=0;//save channel0 only
	unsigned int use;
	struct rk3399_sdram_params *sdram_params;
	struct rk3399_ddr_pctl_regs *ddr_pctl_regs;
	struct rk3399_ddr_pi_regs *ddr_pi_regs;
	struct rk3399_ddr_publ_regs *ddr_publ_regs;
	struct rk3399_msch_regs *ddr_msch_regs;
	struct rk3399_msch_timings *noc_timing;
	struct rk3399_cru_regs *rk3399_cru_reg = (void *)CRU_BASE_ADDR;
	u32 refdiv,postdiv2,postdiv1,fbdiv;  //FREF越大，VCO越大，jitter就会小
	u32 i;

	sdram_params = get_sdram_config_dram();
	sys_reg = read32(&(pmugrf_regs->pmugrf_os_reg[2]));

	for (channel = 0, use = 0;
		 (use < sdram_params->num_channels) && (channel < 2); channel++) {			
			//struct rk3399_sdram_channel *info =
				//&sdram_params->ch[channel];
			ddr_msch_regs = rk3399_msch[channel];
			ddr_pctl_regs = rk3399_ddr_pctl[channel];
			ddr_pi_regs = rk3399_ddr_pi[channel];
			ddr_publ_regs = rk3399_ddr_publ[channel];
	
			if (sdram_params->ch[channel].col == 0)
				continue;
			use++;

			sdram_params->ch[channel].rank = SYS_REG_DEC_RANK(sys_reg, channel);
			sdram_params->ch[channel].col = SYS_REG_DEC_COL(sys_reg, channel);
			sdram_params->ch[channel].bk = SYS_REG_DEC_BK(sys_reg, channel);
			sdram_params->ch[channel].bw = SYS_REG_DEC_BW(sys_reg, channel);
			sdram_params->ch[channel].dbw = SYS_REG_DEC_DBW(sys_reg, channel);
			sdram_params->ch[channel].row_3_4 = SYS_REG_DEC_ROW_3_4(sys_reg, channel);
			sdram_params->ch[channel].cs0_row = SYS_REG_DEC_CS0_ROW(sys_reg, channel);
			sdram_params->ch[channel].cs1_row = SYS_REG_DEC_CS1_ROW(sys_reg, channel);
			sdram_params->ch[channel].ddrconfig= read32(&ddr_msch_regs->ddrconf)&0x1f;

			noc_timing = &sdram_params->ch[channel].noc_timings;
			noc_timing->ddrtiminga0.d32 = read32(&ddr_msch_regs->ddrtiminga0.d32);
			noc_timing->ddrtimingb0.d32 = read32(&ddr_msch_regs->ddrtimingb0.d32);
			noc_timing->ddrtimingc0.d32 = read32(&ddr_msch_regs->ddrtimingc0.d32);
			noc_timing->devtodev0.d32 = read32(&ddr_msch_regs->devtodev0.d32);
			noc_timing->ddrmode.d32 = read32(&ddr_msch_regs->ddrmode.d32);
	}

	channel = 0;
	ddr_pctl_regs = rk3399_ddr_pctl[channel];
	ddr_pi_regs = rk3399_ddr_pi[channel];
	ddr_publ_regs = rk3399_ddr_publ[channel];

	fbdiv = readl_relaxed(&rk3399_cru_reg->cru_pll_con[DPLL][0])&0xfff;
	postdiv2 = POSTDIV2_DEC(readl_relaxed(&rk3399_cru_reg->cru_pll_con[DPLL][1]));
	postdiv1 = POSTDIV1_DEC(readl_relaxed(&rk3399_cru_reg->cru_pll_con[DPLL][1]));
	refdiv = REFDIV_DEC(readl_relaxed(&rk3399_cru_reg->cru_pll_con[DPLL][1]));
	cru_con |= FBDIV_ENC(fbdiv);
	cru_con |= POSTDIV2_ENC(postdiv2);
	cru_con |= POSTDIV1_ENC(postdiv1);
	cru_con |= REFDIV_ENC(refdiv);
	sdram_params->ddr_freq= ((fbdiv*24)/(refdiv * postdiv1))*MHz;
	//sdram_params->ddr_freq= 200*MHz;	
	INFO("sdram_params->ddr_freq = %d\n", sdram_params->ddr_freq);
	sdram_params->dramtype = SYS_REG_DEC_DDRTYPE(sys_reg);
	sdram_params->num_channels = SYS_REG_DEC_NUM_CH(sys_reg);
	sdram_params->stride = (read32(&sgrf_regs->sgrf_soc_con4)>>10)&0x1f;
	sdram_params->odt = (((read32(&ddr_publ_regs->denali_phy[5])>>16)&0x7)!=0)?1:0;

	/*copy the registers CTL PI and PHY*/
	dmc_memcpy_dram((u32 *)&sdram_params->pctl_regs.denali_ctl[0],
		(u32 *)ddr_pctl_regs,
		sizeof(struct rk3399_ddr_pctl_regs));
	/*mask DENALI_CTL_00_DATA.START,only copy here,will trigger later*/
	sdram_params->pctl_regs.denali_ctl[0] &=  ~(0x1<<0);
	/*set DENALI_CTL_68_DATA.PWRUP_SREFRESH_EXIT=0x1*/
	sdram_params->pctl_regs.denali_ctl[68] |=  (0x1<<16);

	dmc_memcpy_dram((u32 *)&sdram_params->pi_regs.denali_pi[0],
		(u32 *)ddr_pi_regs,
		sizeof(struct rk3399_ddr_pi_regs));
	/*mask DENALI_PI_00_DATA.START,only copy here,will trigger later*/
	sdram_params->pi_regs.denali_pi[0] &=  ~(0x1<<0);

	#if DMC_PHY_REGS
	dmc_memcpy_dram((u32 *)&sdram_params->phy_regs.denali_phy[0],
		(u32 *)ddr_publ_regs,
		sizeof(struct rk3399_ddr_publ_regs));
	/*set DENALI_PHY_957_DATA.PHY_DLL_RST_EN=0x1*/
	sdram_params->phy_regs.denali_phy[957] =
		(sdram_params->phy_regs.denali_phy[957] & ~(0x3 << 24)) | (1 << 24);
	#else
	for (i=0;i<PHY_REGS_GROUP_SAVE;i++) {
	dmc_memcpy_dram((u32 *)&sdram_params->phy_regs.denali_phy[phy_regs_map_save[i][1]],
		(u32 *)&ddr_publ_regs->denali_phy[phy_regs_map_save[i][0]],
		phy_regs_map_save[i][2]* 4);
	}
	/*set DENALI_PHY_957_DATA.PHY_DLL_RST_EN=0x1*/
	sdram_params->phy_regs.denali_phy[phy_regs_map_save[PHY_REGS_GROUP_SAVE-1][1] + 61] =
		(sdram_params->phy_regs.denali_phy[phy_regs_map_save[PHY_REGS_GROUP_SAVE-1][1] + 61] & ~(0x3 << 24)) | (1 << 24);
	#endif
}

#if 1
__sramfunc void dmc_restore(void)
{
//return;
	int channel=0;
	struct rk3399_sdram_params *sdram_params;
	sdram_params = get_sdram_config();
	//uart_init();

	rkclk_configure_ddr(/*sdram_params->ddr_freq*/);
#if 1
	uart_init();
	uart_wrtie_byte('a');

//	print("sdram_params->ddr_freq=");
//	print_dec(sdram_params->ddr_freq);
//	print("\n");
#endif
	for (channel = 0; channel < 2; channel++) {
		/*struct rk3399_ddr_pctl_regs *ddr_pctl_regs =
			rk3399_ddr_pctl[channel];*/
		struct rk3399_ddr_publ_regs *ddr_publ_regs =
			rk3399_ddr_publ[channel];
		uart_wrtie_byte('b');

		phy_pctrl_reset(ddr_publ_regs, channel);		
		phy_dll_bypass_set(channel, ddr_publ_regs,
				   sdram_params->ddr_freq);
		uart_wrtie_byte('c');

		if (channel >= sdram_params->num_channels) {
			uart_wrtie_byte('x');
			print_hex (channel);
			uart_wrtie_byte('x');
			print_hex (sdram_params->num_channels);
			continue;
		}
		uart_wrtie_byte('Y');
		pctl_cfg(channel, sdram_params);
		uart_wrtie_byte('d');

		/* LPDDR2/LPDDR3 need to wait DAI complete, max 10us */
		if (sdram_params->dramtype == LPDDR3)
			dmc_udelay(10);
		uart_wrtie_byte('F');
		uart_wrtie_byte('W');
		uart_wrtie_byte('W');
		uart_wrtie_byte('K');
		uart_wrtie_byte('K');
		#if DMC_DO_DATA_TRAINING
		if (data_training(channel, sdram_params, PI_FULL_TARINING))	
		{
		}
		#endif
		uart_wrtie_byte('f');

		#if 0
		if (sdram_params->dramtype != DDR3)
			dram_print_all_mr(ddr_pctl_regs,
					sdram_params, channel);
		#endif
		set_ddrconfig(sdram_params, channel,
				  sdram_params->ch[channel].ddrconfig);
		uart_wrtie_byte('g');
		ddr_move_to_access_state(channel);
		uart_wrtie_byte('h');
		//enable_low_power(sdram_params, channel);
	}
	uart_wrtie_byte('j');
	dram_all_config(sdram_params);
	uart_wrtie_byte('k');
	//while(1);
}
#endif
