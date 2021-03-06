/*
 * Copyright (c) 2017, ARM Limited and Contributors. All rights reserved.
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

#ifndef __SOC_ROCKCHIP_RK3X2X_DFS_H__
#define __SOC_ROCKCHIP_RK3X2X_DFS_H__

struct sdram_default_config {
	unsigned char bl;
	/* 1:auto precharge, 0:never auto precharge */
	unsigned char ap;
	/* dram driver strength */
	unsigned char dramds;
	/* dram ODT, if odt=0, this parameter invalid
	 * DDR3 support: 40,60,120,0
	 * LPDDR3 support: 60,120,240,0
	 */
	unsigned char dramodt;
	/* ca ODT, if odt=0, this parameter invalid
	 * only used by LPDDR4
	 */
	unsigned char caodt;
	unsigned char burst_ref_cnt;
	/* zqcs period, unit(s) */
	unsigned char zqcsi;
	/* rddbi: 1: en 0 :disable for ddr4 only */
	unsigned char rd_dbi;
};

struct  ddr_dts_config_timing {
	unsigned int ddr3_speed_bin;
	unsigned int ddr4_speed_bin;
	unsigned int pd_idle;
	unsigned int sr_idle;
	unsigned int sr_mc_gate_idle;
	unsigned int srpd_lite_idle;
	unsigned int standby_idle;

	unsigned int auto_pd_dis_freq;
	unsigned int auto_sr_dis_freq;
	/* for ddr3 only */
	unsigned int ddr3_dll_dis_freq;
	/* for ddr4 only */
	unsigned int ddr4_dll_dis_freq;
	unsigned int phy_dll_dis_freq;

	unsigned int ddr3_odt_dis_freq;
	unsigned int phy_ddr3_odt_dis_freq;
	unsigned int ddr3_drv;
	unsigned int ddr3_odt;
	unsigned int phy_ddr3_ca_drv;
	unsigned int phy_ddr3_ck_drv;
	unsigned int phy_ddr3_dq_drv;
	unsigned int phy_ddr3_odt;

	unsigned int lpddr3_odt_dis_freq;
	unsigned int phy_lpddr3_odt_dis_freq;
	unsigned int lpddr3_drv;
	unsigned int lpddr3_odt;
	unsigned int phy_lpddr3_ca_drv;
	unsigned int phy_lpddr3_ck_drv;
	unsigned int phy_lpddr3_dq_drv;
	unsigned int phy_lpddr3_odt;

	unsigned int lpddr4_odt_dis_freq;
	unsigned int phy_lpddr4_odt_dis_freq;
	unsigned int lpddr4_drv;
	unsigned int lpddr4_dq_odt;
	unsigned int lpddr4_ca_odt;
	unsigned int phy_lpddr4_ca_drv;
	unsigned int phy_lpddr4_ck_cs_drv;
	unsigned int phy_lpddr4_dq_drv;
	unsigned int phy_lpddr4_odt;

	unsigned int ddr4_odt_dis_freq;
	unsigned int phy_ddr4_odt_dis_freq;
	unsigned int ddr4_drv;
	unsigned int ddr4_odt;
	unsigned int phy_ddr4_ca_drv;
	unsigned int phy_ddr4_ck_drv;
	unsigned int phy_ddr4_dq_drv;
	unsigned int phy_ddr4_odt;

	unsigned int ca_skew[15];
	unsigned int cs0_skew[44];
	unsigned int cs1_skew[44];

	uint32_t available;
};

struct drv_odt_lp_config {
	uint32_t ddr_speed_bin;
	uint32_t pd_idle;
	uint32_t sr_idle;
	uint32_t sr_mc_gate_idle;
	uint32_t srpd_lite_idle;
	uint32_t standby_idle;

	uint32_t auto_pd_dis_freq;
	uint32_t auto_sr_dis_freq;
	uint32_t dram_dll_dis_freq;/* for ddr3, ddr4 only */
	uint32_t phy_dll_dis_freq;
	uint32_t dram_odt_dis_freq;

	uint32_t dram_side_drv;
	uint32_t dram_side_dq_odt;
	uint32_t dram_side_ca_odt;

	uint32_t phy_side_ca_drv;
	uint32_t phy_side_ck_cs_drv;
	uint32_t phy_side_dq_drv;
	uint32_t phy_side_odt;
	uint32_t phy_side_odt_dis_freq;
};

void ddr_dfs_init(uint32_t page_type);
uint32_t ddr_set_rate(uint32_t page_type);
uint32_t ddr_round_rate(uint32_t page_type);
void ddr_set_auto_self_refresh(uint32_t page_type);
uint32_t ddr_get_rate(void);
uint32_t ddr_get_version(void);
void register_fiq_ddr_freq_change(void);

#endif
