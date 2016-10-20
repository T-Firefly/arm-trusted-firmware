/*
 * Copyright (C) 2016, Fuzhou Rockchip Electronics Co., Ltd.
 * All rights reserved.
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

#ifndef __PMU_H__
#define __PMU_H__

/*****************************************************************************
 * The ways of cores power domain contorlling
 *****************************************************************************/
enum cores_pm_ctr_mode {
	core_pwr_pd = 0,
	core_pwr_wfi = 1,
	core_pwr_wfi_int = 2
};

enum pmu_cores_pm_by_wfi {
	core_pm_en = 0,
	core_pm_int_wakeup_en,
	core_pm_dis_int,
	core_pm_sft_wakeup_en
};

extern void *pmu_cpuson_entrypoint_start;
extern void *pmu_cpuson_entrypoint_end;
extern uint64_t cpuson_entry_point[PLATFORM_CORE_COUNT];
extern uint32_t cpuson_flags[PLATFORM_CORE_COUNT];

#define CORES_PM_DISABLE	0x0

/*****************************************************************************
 * pmu con,reg
 *****************************************************************************/
#define PMU_WAKEUP_CFG0		0x00
#define PMU_PWRDN_CON		0x0c
#define PMU_PWRDN_ST		0x10
#define PMU_PWRMD_COM		0x18
#define PMU_SFT_CON		0x1c
#define PMU_INT_CON		0x20
#define PMU_INT_ST		0x24
#define PMU_POWER_ST		0x44
#define PMU_CPUAPM_CON(n)	(0x80 + (n) * 4)
#define PMU_SYS_REG(n)		(0xa0 + (n) * 4)

#define CHECK_CPU_WFIE_BASE		(GRF_BASE + GRF_CPU_STATUS(1))

enum pmu_core_pwrst_shift {
	clst_cpu_wfe = 0,
	clst_cpu_wfi = 4,
};

#define clstl_cpu_wfe (clst_cpu_wfe)
#define clstb_cpu_wfe (clst_cpu_wfe)

enum pmu_pd_id {
	PD_CPU0 = 0,
	PD_CPU1,
	PD_CPU2,
	PD_CPU3,
};

enum pmu_power_mode_common {
	pmu_mode_en = 0,
	sref_enter_en,
	global_int_disable_cfg,
	cpu0_pd_en,
	wait_wakeup_begin_cfg = 4,
	l2_flush_en,
	l2_idle_en,
	ddrio_ret_de_req,
	ddrio_ret_en = 8,
};

enum pmu_sft_con {
	upctl_c_sysreq_cfg = 0,
	l2flushreq_req,
	ddr_io_ret_cfg,
	pmu_sft_ret_cfg,
};


#define CKECK_WFE_MSK		0x1
#define CKECK_WFI_MSK		0x10
#define CKECK_WFEI_MSK		0x11

#define PD_CTR_LOOP		500
#define CHK_CPU_LOOP		500
#define MAX_WAIT_CONUT		1000

#endif /* __PMU_H__ */
