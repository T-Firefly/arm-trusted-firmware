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
#include <pmu.h>
#include <rk3399_def.h>
#include <soc.h>
#include <dmc.h>

#define PMUGRF_OS_REG0		0x300
#define PMUGRF_OS_REG1		0x304
#define PMUGRF_OS_REG2		0x308
#define PMUGRF_OS_REG3		0x30c

#define CIC_CTRL0		0x0
#define CIC_STATUS0		0x10

#define CRU_SFTRST_DDR_CTRL(ch, n)	((0x1 << (8 + 16 + (ch) * 4)) | \
					 ((n) << (8 + (ch) * 4)))
#define CRU_SFTRST_DDR_PHY(ch, n)	((0x1 << (9 + 16 + (ch) * 4)) | \
					 ((n) << (9 + (ch) * 4)))

#define FBDIV_ENC(n)			((n) << 16)
#define FBDIV_DEC(n)			(((n) >> 16) & 0xfff)
#define POSTDIV2_ENC(n)			((n) << 12)
#define POSTDIV2_DEC(n)			(((n) >> 12) & 0x7)
#define POSTDIV1_ENC(n)			((n) << 8)
#define POSTDIV1_DEC(n)			(((n) >> 8) & 0x7)
#define REFDIV_ENC(n)			(n)
#define REFDIV_DEC(n)			((n) & 0x3f)

/* PMU CRU */
#define PMUCRU_RSTNHOLD_CON0		(0x120)
#define PMUCRU_RSTNHOLD_CON1		(0x124)

#define PRESET_GPIO0_HOLD(n)		(((n) << 7) | WMSK_BIT(7))
#define PRESET_GPIO1_HOLD(n)		(((n) << 8) | WMSK_BIT(8))

__sramdata struct rk3399_sdram_params sdram_config;

static __sramdata uint32_t phy_regs_map[][3] = {
	{0, 0, 91},
	{128, 91, 91},
	{256, 182, 91},
	{384, 273, 91},
	{512, 364, 38},
	{640, 402, 38},
	{768, 440, 38},
	{896, 478, 62},
};

/* To save/restore CRU CON registers */
static __sramdata uint32_t cru_con;

/* To save/restore DDR PHY controller registers */
static __sramdata struct rk3399_ddr_pctl_regs *rk3399_ddr_pctl[2] = {
	(void *)DDRC0_BASE_ADDR, (void *)DDRC1_BASE_ADDR};
static __sramdata struct rk3399_ddr_pi_regs *rk3399_ddr_pi[2] = {
	(void *)DDRC0_PI_BASE_ADDR, (void *)DDRC1_PI_BASE_ADDR};
static __sramdata struct rk3399_ddr_publ_regs *rk3399_ddr_publ[2] = {
	(void *)DDRC0_PHY_BASE_ADDR, (void *)DDRC1_PHY_BASE_ADDR};
static __sramdata struct rk3399_msch_regs *rk3399_msch[2] = {
	(void *)SERVER_MSCH0_BASE_ADDR, (void *)SERVER_MSCH1_BASE_ADDR};

static __sramfunc void dmc_udelay(uint32_t us)
{
	volatile uint32_t i;

	i = ((24 * us * 195 + 8191) / 8192);
	if (i)
		while (i--)
			;
}

/*
 * Copy @len bytes from @src to @dst
 */
static __sramfunc void *dmc_memcpy_iram(void *dst,
					const void *src,
					uint32_t len)
{
	const uint32_t *s = src;
	uint32_t *d = dst;

	/* copy 4 bytes(uint32_t) one time */
	len = len / 4;
	while (len--)
		*d++ = *s++;

	return dst;
}

/*
 * Copy registers to buffer when suspend, can not use IRAM code.
 * Copy @len bytes from @src to @dst
 */
static void *dmc_memcpy_dram(void *dst, const void *src, uint32_t len)
{
	const uint32_t *s = src;
	uint32_t *d = dst;

	/* copy 4 bytes(uint32_t) one time */
	len = len / 4;
	while (len--)
		*d++ = *s++;

	return dst;
}

static __sramfunc void rkclk_configure_ddr(void)
{
	uint32_t refdiv, postdiv1, postdiv2, fbdiv;
	int delay = 1000;

	fbdiv = FBDIV_DEC(cru_con);
	postdiv1 = POSTDIV1_DEC(cru_con);
	postdiv2 = POSTDIV2_DEC(cru_con);
	refdiv = REFDIV_DEC(cru_con);

	mmio_write_32(CRU_BASE + CRU_PLL_CON(DPLL_ID, 3), PLL_SLOW_MODE);

	mmio_write_32(CRU_BASE + CRU_PLL_CON(DPLL_ID, 0), FBDIV(fbdiv));

	mmio_write_32(CRU_BASE + CRU_PLL_CON(DPLL_ID, 1),
		      POSTDIV2(postdiv2) | POSTDIV1(postdiv1) | REFDIV(refdiv));

	while (delay > 0) {
		dmc_udelay(1);
		if (PLL_LOCK(mmio_read_32(CRU_BASE + CRU_PLL_CON(DPLL_ID, 2))))
			break;
		delay--;
	}

	mmio_write_32(CRU_BASE + CRU_PLL_CON(DPLL_ID, 3), PLL_NOMAL_MODE);

	/* IC ECO bug, need to set this register */
	mmio_write_32(SGRF_BASE + SGRF_DDRRGN_CON0_16(16), 0xc000c000);
}

static __sramfunc void rkclk_ddr_reset(uint32_t channel,
				       uint32_t ctl,
				       uint32_t phy)
{
	channel &= 0x1;
	ctl &= 0x1;
	phy &= 0x1;
	mmio_write_32(CRU_BASE + CRU_SOFTRST_CON(4),
		      CRU_SFTRST_DDR_CTRL(channel, ctl) |
		      CRU_SFTRST_DDR_PHY(channel, phy));
}

static __sramfunc void phy_pctrl_reset(
		struct rk3399_ddr_publ_regs *ddr_publ_regs,
		uint32_t channel)
{
	rkclk_ddr_reset(channel, 1, 1);
	dmc_udelay(10);
	rkclk_ddr_reset(channel, 1, 0);
	dmc_udelay(10);
	rkclk_ddr_reset(channel, 0, 0);
	dmc_udelay(10);
}

static __sramfunc void phy_dll_bypass_set(uint32_t channel,
					  struct rk3399_ddr_publ_regs *ddr_publ_regs,
					  uint32_t hz)
{
	uint32_t *denali_phy = ddr_publ_regs->denali_phy;

	if (hz <= 125 * MHz) {
		/* phy_sw_master_mode_X PHY_86/214/342/470 4bits offset_8 */
		mmio_setbits_32((uintptr_t)&denali_phy[86], (0x3 << 2) << 8);
		mmio_setbits_32((uintptr_t)&denali_phy[214], (0x3 << 2) << 8);
		mmio_setbits_32((uintptr_t)&denali_phy[342], (0x3 << 2) << 8);
		mmio_setbits_32((uintptr_t)&denali_phy[470], (0x3 << 2) << 8);
		/* phy_adrctl_sw_master_mode PHY_547/675/803 4bits offset_16 */
		mmio_setbits_32((uintptr_t)&denali_phy[547], (0x3 << 2) << 16);
		mmio_setbits_32((uintptr_t)&denali_phy[675], (0x3 << 2) << 16);
		mmio_setbits_32((uintptr_t)&denali_phy[803], (0x3 << 2) << 16);
	} else {
		/* phy_sw_master_mode_X PHY_86/214/342/470 4bits offset_8 */
		mmio_clrbits_32((uintptr_t)&denali_phy[86], (0x3 << 2) << 8);
		mmio_clrbits_32((uintptr_t)&denali_phy[214], (0x3 << 2) << 8);
		mmio_clrbits_32((uintptr_t)&denali_phy[342], (0x3 << 2) << 8);
		mmio_clrbits_32((uintptr_t)&denali_phy[470], (0x3 << 2) << 8);
		/* phy_adrctl_sw_master_mode PHY_547/675/803 4bits offset_16 */
		mmio_clrbits_32((uintptr_t)&denali_phy[547], (0x3 << 2) << 16);
		mmio_clrbits_32((uintptr_t)&denali_phy[675], (0x3 << 2) << 16);
		mmio_clrbits_32((uintptr_t)&denali_phy[803], (0x3 << 2) << 16);
	}
}

static __sramfunc void set_cs_training_index(uint32_t channel, uint32_t rank)
{
	uint32_t *denali_phy = rk3399_ddr_publ[channel]->denali_phy;

	/* PHY_8/136/264/392 phy_per_cs_training_index_X 1bit offset_24 */
	mmio_clrsetbits_32((uintptr_t)&denali_phy[8], 0x1 << 24, rank << 24);
	mmio_clrsetbits_32((uintptr_t)&denali_phy[136], 0x1 << 24, rank << 24);
	mmio_clrsetbits_32((uintptr_t)&denali_phy[264], 0x1 << 24, rank << 24);
	mmio_clrsetbits_32((uintptr_t)&denali_phy[392], 0x1 << 24, rank << 24);
}

static __sramfunc void select_per_cs_training_index(uint32_t channel,
						    uint32_t rank)
{
	uint32_t *denali_phy = rk3399_ddr_publ[channel]->denali_phy;

	/* PHY_84 PHY_PER_CS_TRAINING_EN_0 1bit offset_16 */
	if ((mmio_read_32((uintptr_t)&denali_phy[84]) >> 16) & 1)
		set_cs_training_index(channel, rank);
}

static void override_write_leveling_value(uint32_t channel)
{
	uint32_t *denali_ctl = rk3399_ddr_pctl[channel]->denali_ctl;
	uint32_t *denali_phy = rk3399_ddr_publ[channel]->denali_phy;
	uint32_t byte;

	/* PHY_896 PHY_FREQ_SEL_MULTICAST_EN 1bit offset_0 */
	mmio_setbits_32((uintptr_t)&denali_phy[896], 1);

	/*
	 * PHY_8/136/264/392
	 * phy_per_cs_training_multicast_en_X 1bit offset_16
	 */
	mmio_clrsetbits_32((uintptr_t)&denali_phy[8], 0x1 << 16, 1 << 16);
	mmio_clrsetbits_32((uintptr_t)&denali_phy[136], 0x1 << 16, 1 << 16);
	mmio_clrsetbits_32((uintptr_t)&denali_phy[264], 0x1 << 16, 1 << 16);
	mmio_clrsetbits_32((uintptr_t)&denali_phy[392], 0x1 << 16, 1 << 16);

	for (byte = 0; byte < 4; byte++)
		mmio_clrsetbits_32((uintptr_t)&denali_phy[63 + (128 * byte)],
				   0xffff << 16,
				   0x200 << 16);

	/* PHY_896 PHY_FREQ_SEL_MULTICAST_EN 1bit offset_0 */
	mmio_clrbits_32((uintptr_t)&denali_phy[896], 1);

	/* CTL_200 ctrlupd_req 1bit offset_8 */
	mmio_clrsetbits_32((uintptr_t)&denali_ctl[200], 0x1 << 8, 0x1 << 8);
}

static __sramfunc int data_training(uint32_t channel,
				    struct rk3399_sdram_params *sdram_params,
				    uint32_t training_flag)
{
	uint32_t *denali_pi = rk3399_ddr_pi[channel]->denali_pi;
	uint32_t *denali_phy = rk3399_ddr_publ[channel]->denali_phy;
	uint32_t obs_0, obs_1, obs_2, obs_3, obs_err = 0;
	uint32_t rank = sdram_params->ch[channel].rank;
	uint32_t rank_mask;
	uint32_t i, tmp;

	if (sdram_params->dramtype == LPDDR4)
		rank_mask = (rank == 1) ? 0x5 : 0xf;
	else
		rank_mask = (rank == 1) ? 0x1 : 0x3;

	/* PHY_927 PHY_PAD_DQS_DRIVE  RPULL offset_22 */
	mmio_setbits_32((uintptr_t)&denali_phy[927], (1 << 22));

	if (training_flag == PI_FULL_TRAINING) {
		if (sdram_params->dramtype == LPDDR4) {
			training_flag = PI_WRITE_LEVELING |
					PI_READ_GATE_TRAINING |
					PI_READ_LEVELING |
					PI_WDQ_LEVELING;
		} else if (sdram_params->dramtype == LPDDR3) {
			training_flag = PI_CA_TRAINING | PI_WRITE_LEVELING |
					PI_READ_GATE_TRAINING;
		} else if (sdram_params->dramtype == DDR3) {
			training_flag = PI_WRITE_LEVELING |
					PI_READ_GATE_TRAINING |
					PI_READ_LEVELING;
		}
	}

	/* ca training(LPDDR4,LPDDR3 support) */
	if ((training_flag & PI_CA_TRAINING) == PI_CA_TRAINING) {
		for (i = 0; i < 4; i++) {
			if (!(rank_mask & (1 << i)))
				continue;

			select_per_cs_training_index(channel, i);
			/* PI_100 PI_CALVL_EN:RW:8:2 */
			mmio_clrsetbits_32((uintptr_t)&denali_pi[100], 0x3 << 8,
					   0x2 << 8);

			/* PI_92 PI_CALVL_REQ:WR:16:1,PI_CALVL_CS:RW:24:2 */
			mmio_clrsetbits_32((uintptr_t)&denali_pi[92],
					   (0x1 << 16) | (0x3 << 24),
					   (0x1 << 16) | (i << 24));
			while (1) {
				/* PI_174 PI_INT_STATUS:RD:8:18 */
				tmp = mmio_read_32((uintptr_t)&denali_pi[174]);
				tmp >>= 8;

				/*
				 * check status obs
				 * PHY_532/660/788 phy_adr_calvl_obs1_:0:32
				 */
				obs_0 = mmio_read_32(
						(uintptr_t)&denali_phy[532]);
				obs_1 = mmio_read_32(
						(uintptr_t)&denali_phy[660]);
				obs_2 = mmio_read_32(
						(uintptr_t)&denali_phy[788]);
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
			mmio_write_32((uintptr_t)(&denali_pi[175]), 0x00003f7c);
		}
		mmio_clrbits_32((uintptr_t)&denali_pi[100], 0x3 << 8);
	}

	/* write leveling(LPDDR4,LPDDR3,DDR3 support) */
	if ((training_flag & PI_WRITE_LEVELING) == PI_WRITE_LEVELING) {
		for (i = 0; i < rank; i++) {
			select_per_cs_training_index(channel, i);
			/* PI_60 PI_WRLVL_EN:RW:8:2 */
			mmio_clrsetbits_32((uintptr_t)&denali_pi[60], 0x3 << 8,
					   0x2 << 8);
			/* PI_59 PI_WRLVL_REQ:WR:8:1,PI_WRLVL_CS:RW:16:2 */
			mmio_clrsetbits_32((uintptr_t)&denali_pi[59],
					   (0x1 << 8) | (0x3 << 16),
					   (0x1 << 8) | (i << 16));

			while (1) {
				/* PI_174 PI_INT_STATUS:RD:8:18 */
				tmp = mmio_read_32((uintptr_t)&denali_pi[174]);
				tmp >>= 8;

				/*
				 * check status obs, if error maybe can not
				 * get leveling done PHY_40/168/296/424
				 * phy_wrlvl_status_obs_X:0:13
				 */
				obs_0 = mmio_read_32(
						(uintptr_t)&denali_phy[40]);
				obs_1 = mmio_read_32(
						(uintptr_t)&denali_phy[168]);
				obs_2 = mmio_read_32(
						(uintptr_t)&denali_phy[296]);
				obs_3 = mmio_read_32(
						(uintptr_t)&denali_phy[424]);
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
					return -1;
			}

			/* clear interrupt,PI_175 PI_INT_ACK:WR:0:17 */
			mmio_write_32((uintptr_t)(&denali_pi[175]), 0x00003f7c);
		}
		if (0)
			override_write_leveling_value(channel);
		mmio_clrbits_32((uintptr_t)&denali_pi[60], 0x3 << 8);
	}

	/* read gate training(LPDDR4,LPDDR3,DDR3 support) */
	if ((training_flag & PI_READ_GATE_TRAINING) == PI_READ_GATE_TRAINING) {
		for (i = 0; i < rank; i++) {
			select_per_cs_training_index(channel, i);
			/* PI_80 PI_RDLVL_GATE_EN:RW:24:2 */
			mmio_clrsetbits_32((uintptr_t)&denali_pi[80], 0x3 << 24,
					   0x2 << 24);
			/*
			 * PI_74 PI_RDLVL_GATE_REQ:WR:16:1
			 * PI_RDLVL_CS:RW:24:2
			 */
			mmio_clrsetbits_32((uintptr_t)&denali_pi[74],
					   (0x1 << 16) | (0x3 << 24),
					   (0x1 << 16) | (i << 24));

			while (1) {
				/* PI_174 PI_INT_STATUS:RD:8:18 */
				tmp = mmio_read_32((uintptr_t)&denali_pi[174]);
				tmp >>= 8;

				/*
				 * check status obs
				 * PHY_43/171/299/427
				 *     PHY_GTLVL_STATUS_OBS_x:16:8
				 */
				obs_0 = mmio_read_32(
						(uintptr_t)&denali_phy[43]);
				obs_1 = mmio_read_32(
						(uintptr_t)&denali_phy[171]);
				obs_2 = mmio_read_32(
						(uintptr_t)&denali_phy[299]);
				obs_3 = mmio_read_32(
						(uintptr_t)&denali_phy[427]);
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
			mmio_write_32((uintptr_t)(&denali_pi[175]), 0x00003f7c);
		}
		mmio_clrbits_32((uintptr_t)&denali_pi[80], 0x3 << 24);
	}

	/* read leveling(LPDDR4,LPDDR3,DDR3 support) */
	if ((training_flag & PI_READ_LEVELING) == PI_READ_LEVELING) {
		for (i = 0; i < rank; i++) {
			select_per_cs_training_index(channel, i);
			/* PI_80 PI_RDLVL_EN:RW:16:2 */
			mmio_clrsetbits_32((uintptr_t)&denali_pi[80], 0x3 << 16,
					   0x2 << 16);
			/* PI_74 PI_RDLVL_REQ:WR:8:1,PI_RDLVL_CS:RW:24:2 */
			mmio_clrsetbits_32((uintptr_t)&denali_pi[74],
					   (0x1 << 8) | (0x3 << 24),
					   (0x1 << 8) | (i << 24));
			while (1) {
				/* PI_174 PI_INT_STATUS:RD:8:18 */
				tmp = mmio_read_32((uintptr_t)&denali_pi[174]);
				tmp >>= 8;

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
			mmio_write_32((uintptr_t)(&denali_pi[175]), 0x00003f7c);
		}
		mmio_clrbits_32((uintptr_t)&denali_pi[80], 0x3 << 16);
	}

	/* wdq leveling(LPDDR4 support) */
	if ((training_flag & PI_WDQ_LEVELING) == PI_WDQ_LEVELING) {
		for (i = 0; i < 4; i++) {
			if (!(rank_mask & (1 << i)))
				continue;

			select_per_cs_training_index(channel, i);
			/*
			 * disable PI_WDQLVL_VREF_EN before wdq leveling?
			 * PI_181 PI_WDQLVL_VREF_EN:RW:8:1
			 */
			mmio_clrbits_32((uintptr_t)&denali_pi[181], 0x1 << 8);
			/* PI_124 PI_WDQLVL_EN:RW:16:2 */
			mmio_clrsetbits_32((uintptr_t)&denali_pi[124],
					   0x3 << 16, 0x2 << 16);
			/* PI_121 PI_WDQLVL_REQ:WR:8:1,PI_WDQLVL_CS:RW:16:2 */
			mmio_clrsetbits_32((uintptr_t)&denali_pi[121],
					   (0x1 << 8) | (0x3 << 16),
					   (0x1 << 8) | (i << 16));
			while (1) {
				/* PI_174 PI_INT_STATUS:RD:8:18 */
				tmp = mmio_read_32((uintptr_t)&denali_pi[174]);
				tmp >>= 8;
				if ((((tmp >> 12) & 0x1) == 0x1) &&
				    (((tmp >> 13) & 0x1) == 0x1) &&
				    (((tmp >> 6) & 0x1) == 0x0))
					break;
				else if (((tmp >> 6) & 0x1) == 0x1)
					return -1;
			}
			/* clear interrupt,PI_175 PI_INT_ACK:WR:0:17 */
			mmio_write_32((uintptr_t)(&denali_pi[175]), 0x00003f7c);
		}
		mmio_clrbits_32((uintptr_t)&denali_pi[124], 0x3 << 16);
	}

	/* PHY_927 PHY_PAD_DQS_DRIVE  RPULL offset_22 */
	mmio_clrbits_32((uintptr_t)&denali_phy[927], (1 << 22));

	return 0;
}

static __sramfunc void set_ddrconfig(struct rk3399_sdram_params *sdram_params,
				     unsigned char channel,
				     uint32_t ddrconfig)
{
	/* only need to set ddrconfig */
	struct rk3399_msch_regs *ddr_msch_regs = rk3399_msch[channel];
	unsigned int cs0_cap = 0;
	unsigned int cs1_cap = 0;

	cs0_cap = (1 << (sdram_params->ch[channel].cs0_row +
			 sdram_params->ch[channel].col +
			 sdram_params->ch[channel].bk +
			 sdram_params->ch[channel].bw - 20));
	if (sdram_params->ch[channel].rank > 1)
		cs1_cap = cs0_cap >> (sdram_params->ch[channel].cs0_row -
				      sdram_params->ch[channel].cs1_row);
	if (sdram_params->ch[channel].row_3_4) {
		cs0_cap = cs0_cap * 3 / 4;
		cs1_cap = cs1_cap * 3 / 4;
	}

	mmio_write_32((uintptr_t)&ddr_msch_regs->ddrconf,
		      ddrconfig | (ddrconfig << 8));
	mmio_write_32((uintptr_t)&ddr_msch_regs->ddrsize,
		      ((cs0_cap / 32) & 0xff) | (((cs1_cap / 32) & 0xff) << 8));
}

static __sramfunc void dram_all_config(struct rk3399_sdram_params *sdram_params)
{
	uint32_t sys_reg = 0;
	unsigned int i;

	sys_reg |= SYS_REG_ENC_DDRTYPE(sdram_params->dramtype);
	sys_reg |= SYS_REG_ENC_NUM_CH(sdram_params->num_channels);

	for (i = 0; i < 2; i++) {
		struct rk3399_sdram_channel *info = &sdram_params->ch[i];
		struct rk3399_msch_regs *ddr_msch_regs = rk3399_msch[i];
		struct rk3399_msch_timings *noc = &info->noc_timings;
		uint32_t *denali_ctl = rk3399_ddr_pctl[i]->denali_ctl;

		if (sdram_params->ch[i].col == 0)
			continue;

		sys_reg |= SYS_REG_ENC_ROW_3_4(info->row_3_4, i);
		sys_reg |= SYS_REG_ENC_CHINFO(i);
		sys_reg |= SYS_REG_ENC_RANK(info->rank, i);
		sys_reg |= SYS_REG_ENC_COL(info->col, i);
		sys_reg |= SYS_REG_ENC_BK(info->bk, i);
		sys_reg |= SYS_REG_ENC_CS0_ROW(info->cs0_row, i);
		if (sdram_params->ch[i].rank > 1)
			sys_reg |= SYS_REG_ENC_CS1_ROW(info->cs1_row, i);
		sys_reg |= SYS_REG_ENC_BW(info->bw, i);
		sys_reg |= SYS_REG_ENC_DBW(info->dbw, i);

		mmio_write_32((uintptr_t)&ddr_msch_regs->ddrtiminga0.d32,
			      noc->ddrtiminga0.d32);
		mmio_write_32((uintptr_t)&ddr_msch_regs->ddrtimingb0.d32,
			      noc->ddrtimingb0.d32);
		mmio_write_32((uintptr_t)&ddr_msch_regs->ddrtimingc0.d32,
			      noc->ddrtimingc0.d32);
		mmio_write_32((uintptr_t)&ddr_msch_regs->devtodev0.d32,
			      noc->devtodev0.d32);
		mmio_write_32((uintptr_t)&ddr_msch_regs->ddrmode.d32,
			      noc->ddrmode.d32);

		/* rank 1 memory clock disable (dfi_dram_clk_disable = 1) */
		if (sdram_params->ch[i].rank == 1)
			mmio_setbits_32((uintptr_t)&denali_ctl[276], 1 << 17);
	}

	DDR_STRIDE(sdram_params->stride);

	/* reboot hold register set */
	mmio_write_32(PMUCRU_BASE + CRU_PMU_RSTHOLD_CON(1),
		      CRU_PMU_SGRF_RST_RLS |
		      PRESET_GPIO0_HOLD(1) |
		      PRESET_GPIO1_HOLD(1));
	mmio_clrsetbits_32(CRU_BASE + CRU_GLB_RST_CON, 0x3, 0x3);
}

static __sramfunc void pctl_cfg(uint32_t channel,
				struct rk3399_sdram_params *sdram_params)
{
	uint32_t *denali_ctl = rk3399_ddr_pctl[channel]->denali_ctl;
	uint32_t *denali_pi = rk3399_ddr_pi[channel]->denali_pi;
	uint32_t *denali_phy = rk3399_ddr_publ[channel]->denali_phy;
	const uint32_t *params_ctl = sdram_params->pctl_regs.denali_ctl;
	const uint32_t *params_phy = sdram_params->phy_regs.denali_phy;
	uint32_t tmp, tmp1, tmp2, i, len;

	/*
	 * Workaround controller bug:
	 * Do not program DRAM_CLASS until NO_PHY_IND_TRAIN_INT is programmed
	 */
	dmc_memcpy_iram(&denali_ctl[1], &params_ctl[1],
			sizeof(struct rk3399_ddr_pctl_regs) - 4);
	mmio_write_32((uintptr_t)&denali_ctl[0], params_ctl[0]);
	dmc_memcpy_iram(denali_pi, &sdram_params->pi_regs.denali_pi[0],
			sizeof(struct rk3399_ddr_pi_regs));

	mmio_write_32((uintptr_t)&denali_phy[910], 0x6400);
	mmio_write_32((uintptr_t)&denali_phy[911], 0x01221102);
	mmio_write_32((uintptr_t)&denali_phy[912], 0x0);

	/* PHY_DLL_RST_EN */
	mmio_clrsetbits_32((uintptr_t)&denali_phy[957], 0x3 << 24, 1 << 24);

	mmio_setbits_32((uintptr_t)&denali_pi[0], START);
	mmio_setbits_32((uintptr_t)&denali_ctl[0], START);

	/* wait lock */
	while (1) {
		tmp = mmio_read_32((uintptr_t)&denali_phy[920]);
		tmp1 = mmio_read_32((uintptr_t)&denali_phy[921]);
		tmp2 = mmio_read_32((uintptr_t)&denali_phy[922]);
		if ((((tmp >> 16) & 0x1) == 0x1) &&
		    (((tmp1 >> 16) & 0x1) == 0x1) &&
		    (((tmp1 >> 0) & 0x1) == 0x1) &&
		    (((tmp2 >> 0) & 0x1) == 0x1))
			break;
		/* if PLL bypass,don't need wait lock */
		if (mmio_read_32((uintptr_t)&denali_phy[911]) & 0x1)
			break;
	}

	len = ARRAY_SIZE(phy_regs_map);
	dmc_memcpy_iram(&denali_phy[phy_regs_map[len - 1][0]],
			&params_phy[phy_regs_map[len - 1][1]],
			phy_regs_map[len - 1][2] * 4);

	for (i = 0; i < len; i++) {
		dmc_memcpy_iram((uint32_t *)&denali_phy[phy_regs_map[i][0]],
				(uint32_t *)&params_phy[phy_regs_map[i][1]],
				phy_regs_map[i][2] * 4);
	}
}

#if 0
extern struct rk3399_ddr_cic_regs *const rk3399_ddr_cic;

static __sramfunc int dram_switch_to_phy_index_n(struct rk3399_sdram_params *sdram_params,
						 uint32_t fn)
{
	uint32_t channel, ch_count;
	uint32_t *denali_phy;

	mmio_write_32((uintptr_t)&rk3399_ddr_cic->cic_ctrl0,
		      (((0x3 << 4) | (1 << 2) | 1) << 16) |
		      (fn << 4) | (1 << 2) | 1);
	while (!(mmio_read_32((uintptr_t)&rk3399_ddr_cic->cic_status0) &
	       (1 << 2)))
		;

	mmio_write_32((uintptr_t)&rk3399_ddr_cic->cic_ctrl0, 0x20002);
	while (!(mmio_read_32((uintptr_t)&rk3399_ddr_cic->cic_status0) &
	       (1 << 0)))
		;

	ch_count = sdram_params->num_channels;

	/* LPDDR4 f2 cann't do training, all training will fail */
	if (!((sdram_params->dramtype == LPDDR4) && (fn == 2))) {
		for (channel = 0; channel < ch_count; channel++) {
			denali_phy = rk3399_ddr_publ[channel]->denali_phy;
			mmio_clrsetbits_32((uintptr_t)&denali_phy[896],
					   (0x3 << 8) | 1, fn << 8);
			if (data_training(channel,
			    sdram_params, PI_FULL_TRAINING)) {
				/* FIXME: dump channel */
			}
		}
	}

	return 0;
}
#endif
/*
 * Needs to be done for both channels at once in case of a shared reset signal
 * between channels.
 */
static __sramfunc int pctl_start(uint32_t channel_mask,
		struct rk3399_sdram_params *sdram_params)
{
	uint32_t *denali_ctl_0 = rk3399_ddr_pctl[0]->denali_ctl;
	uint32_t *denali_phy_0 = rk3399_ddr_publ[0]->denali_phy;
	uint32_t *denali_ctl_1 = rk3399_ddr_pctl[1]->denali_ctl;
	uint32_t *denali_phy_1 = rk3399_ddr_publ[1]->denali_phy;
	uint32_t count;

	/* PHY_DLL_RST_EN */
	if (channel_mask & (1 << 0))
		mmio_clrsetbits_32((uintptr_t)&denali_phy_0[957], 0x3 << 24,
				   0x2 << 24);
	if (channel_mask & (1 << 1))
		mmio_clrsetbits_32((uintptr_t)&denali_phy_1[957], 0x3 << 24,
				   0x2 << 24);

	/* need de-access IO retention before controller START */
	if (channel_mask & (1 << 0))
		mmio_setbits_32(PMU_BASE + PMU_PWRMODE_CON, (1 << 19));
	if (channel_mask & (1 << 1))
		mmio_setbits_32(PMU_BASE + PMU_PWRMODE_CON, (1 << 23));

	/* TODO: check ERROR bit */
	if (channel_mask & (1 << 0)) {
		count = 0;

		while (!(mmio_read_32((uintptr_t)&denali_ctl_0[203]) &
			 (1 << 3))) {
			if (count > 100) {
				/* FIXME: CKE is low?? loop 10ms */
				return -1;
			}

			dmc_udelay(100);
			count++;
		}

		mmio_setbits_32((uintptr_t)&denali_ctl_0[68],
				PWRUP_SREFRESH_EXIT);
	}
	if (channel_mask & (1 << 1)) {
		count = 0;

		while (!(mmio_read_32((uintptr_t)&denali_ctl_1[203]) &
			 (1 << 3))) {
			if (count > 100) {
				/* FIXME: CKE is low?? loop 10ms */
				return -1;
			}

			dmc_udelay(100);
			count++;
		}

		mmio_setbits_32((uintptr_t)&denali_ctl_1[68],
				PWRUP_SREFRESH_EXIT);
	}

	return 0;
}

void dmc_save(void)
{
	struct rk3399_sdram_params *sdram_params = &sdram_config;
	struct rk3399_sdram_channel *channel;
	struct rk3399_msch_regs *ddr_msch_regs;
	struct rk3399_msch_timings *noc;
	uint32_t *denali_ctl;
	uint32_t *denali_pi;
	uint32_t *denali_phy;
	uint32_t *params_ctl;
	uint32_t *params_pi;
	uint32_t *params_phy;
	uint32_t refdiv, postdiv2, postdiv1, fbdiv;
	uint32_t sys_reg, i, tmp;

	sys_reg = mmio_read_32(PMUGRF_BASE + PMUGRF_OS_REG2);

	for (i = 0 ; i < 2; i++) {
		/* channel is not in use */
		if (!(SYS_REG_DEC_CHINFO(sys_reg, i)))
			continue;

		ddr_msch_regs = rk3399_msch[i];
		channel = &sdram_params->ch[i];

		channel->rank = SYS_REG_DEC_RANK(sys_reg, i);
		channel->col = SYS_REG_DEC_COL(sys_reg, i);
		channel->bk = SYS_REG_DEC_BK(sys_reg, i);
		channel->bw = SYS_REG_DEC_BW(sys_reg, i);
		channel->dbw = SYS_REG_DEC_DBW(sys_reg, i);
		channel->row_3_4 = SYS_REG_DEC_ROW_3_4(sys_reg, i);
		channel->cs0_row = SYS_REG_DEC_CS0_ROW(sys_reg, i);
		channel->cs1_row = SYS_REG_DEC_CS1_ROW(sys_reg, i);
		channel->ddrconfig = mmio_read_32(
				(uintptr_t)&ddr_msch_regs->ddrconf) & 0x1f;

		noc = &channel->noc_timings;
		noc->ddrtiminga0.d32 = mmio_read_32(
				(uintptr_t)&ddr_msch_regs->ddrtiminga0.d32);
		noc->ddrtimingb0.d32 = mmio_read_32(
				(uintptr_t)&ddr_msch_regs->ddrtimingb0.d32);
		noc->ddrtimingc0.d32 = mmio_read_32(
				(uintptr_t)&ddr_msch_regs->ddrtimingc0.d32);
		noc->devtodev0.d32 = mmio_read_32(
				(uintptr_t)&ddr_msch_regs->devtodev0.d32);
		noc->ddrmode.d32 = mmio_read_32(
				(uintptr_t)&ddr_msch_regs->ddrmode.d32);
	}

	denali_ctl = rk3399_ddr_pctl[0]->denali_ctl;
	denali_pi = rk3399_ddr_pi[0]->denali_pi;
	denali_phy = rk3399_ddr_publ[0]->denali_phy;
	params_ctl = sdram_params->pctl_regs.denali_ctl;
	params_pi = sdram_params->pi_regs.denali_pi;
	params_phy = sdram_params->phy_regs.denali_phy;

	fbdiv = mmio_read_32(CRU_BASE + CRU_PLL_CON(DPLL_ID, 0)) & 0xfff;
	tmp = mmio_read_32(CRU_BASE + CRU_PLL_CON(DPLL_ID, 1));
	postdiv2 = POSTDIV2_DEC(tmp);
	postdiv1 = POSTDIV1_DEC(tmp);
	refdiv = REFDIV_DEC(tmp);
	cru_con = 0;
	cru_con |= FBDIV_ENC(fbdiv);
	cru_con |= POSTDIV2_ENC(postdiv2);
	cru_con |= POSTDIV1_ENC(postdiv1);
	cru_con |= REFDIV_ENC(refdiv);
	sdram_params->ddr_freq = ((fbdiv * 24) /
				(refdiv * postdiv1 * postdiv2)) * MHz;

	INFO("sdram_params->ddr_freq = %d\n", sdram_params->ddr_freq);
	sdram_params->dramtype = SYS_REG_DEC_DDRTYPE(sys_reg);
	sdram_params->num_channels = SYS_REG_DEC_NUM_CH(sys_reg);
	sdram_params->stride = (mmio_read_32(SGRF_BASE + SGRF_SOC_CON3_7(4)) >>
				10) & 0x1f;
	sdram_params->odt = (((mmio_read_32((uintptr_t)&denali_phy[5]) >> 16) &
			       0x7) != 0) ? 1 : 0;

	/* copy the registers CTL PI and PHY */
	dmc_memcpy_dram(&params_ctl[0], &denali_ctl[0],
			sizeof(struct rk3399_ddr_pctl_regs));

	/* mask DENALI_CTL_00_DATA.START, only copy here, will trigger later */
	params_ctl[0] &= ~(0x1 << 0);
	/* will be set later in the restore stage */
	params_ctl[68] &= ~PWRUP_SREFRESH_EXIT;

	dmc_memcpy_dram(&params_pi[0], &denali_pi[0],
			sizeof(struct rk3399_ddr_pi_regs));

	/* mask DENALI_PI_00_DATA.START, only copy here, will trigger later*/
	params_pi[0] &= ~(0x1 << 0);

	for (i = 0; i < ARRAY_SIZE(phy_regs_map); i++) {
		dmc_memcpy_dram(&params_phy[phy_regs_map[i][1]],
				&denali_phy[phy_regs_map[i][0]],
				phy_regs_map[i][2] * 4);
	}

	/* set DENALI_PHY_957_DATA.PHY_DLL_RST_EN = 0x1 */
	params_phy[phy_regs_map[ARRAY_SIZE(phy_regs_map) - 1][1] + 61] &=
			~(0x3 << 24);
	params_phy[phy_regs_map[ARRAY_SIZE(phy_regs_map) - 1][1] + 61] |= 1 << 24;
	params_phy[phy_regs_map[ARRAY_SIZE(phy_regs_map) - 1][1] + 0] |= 1;
	params_phy[phy_regs_map[ARRAY_SIZE(phy_regs_map) - 1][1] + 0] &=
			~(0x3 << 8);
}

__sramfunc void dmc_restore(void)
{
	struct rk3399_sdram_params *sdram_params = &sdram_config;
	uint32_t channel;
	uint32_t channel_mask = 0;

	rkclk_configure_ddr();

retry:
	for (channel = 0; channel < sdram_params->num_channels; channel++) {
		struct rk3399_ddr_publ_regs *ddr_publ_regs =
			rk3399_ddr_publ[channel];

		phy_pctrl_reset(ddr_publ_regs, channel);
		phy_dll_bypass_set(channel, ddr_publ_regs,
				   sdram_params->ddr_freq);
		if (channel >= sdram_params->num_channels)
			continue;

		pctl_cfg(channel, sdram_params);
	}

	for (channel = 0; channel < 2; channel++) {
		if (sdram_params->ch[channel].col)
			channel_mask |= 1 << channel;
	}

	if (pctl_start(channel_mask, sdram_params) < 0)
		goto retry;

	for (channel = 0; channel < sdram_params->num_channels; channel++) {
		/* LPDDR2/LPDDR3 need to wait DAI complete, max 10us */
		if (sdram_params->dramtype == LPDDR3)
			dmc_udelay(10);

		if (data_training(channel, sdram_params, PI_FULL_TRAINING)) {
			/* TODO: reset the system */
		}

		set_ddrconfig(sdram_params, channel,
			      sdram_params->ch[channel].ddrconfig);
	}

	dram_all_config(sdram_params);
	/* TODO: switch to index 1 after coreboot index 1 patch lands */
}
