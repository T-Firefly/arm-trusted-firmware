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
#include <arch_helpers.h>
#include <debug.h>
#include <assert.h>
#include <bakery_lock.h>
#include <console.h>
#include <delay_timer.h>
#include <errno.h>
#include <mmio.h>
#include <platform.h>
#include <platform_def.h>
#include <plat_private.h>
#include <pmu.h>
#include <pmu_sram.h>
#include <rk322xh_def.h>
#include <soc.h>
#include <pmu_com.h>

DEFINE_BAKERY_LOCK(rockchip_pd_lock);

#define CPU_PD_CTR_FLG_HW 0

static struct psram_data_t *psram_sleep_cfg =
		(struct psram_data_t *)PSRAM_DT_BASE;

#if !CPU_PD_CTR_FLG_HW
static uint32_t cores_pd_cfg_info[PLATFORM_CORE_COUNT]
#if USE_COHERENT_MEM
__attribute__ ((section("tzfw_coherent_mem")))
#endif
;/* coheront */
#endif

int skip_suspend = 0;
static uint32_t cpu_warm_boot_addr;

#pragma weak rk_register_handler

void rk_register_handler(void)
{
}

void plat_rockchip_pmusram_prepare(void)
{
	uint32_t *sram_dst, *sram_src;
	size_t sram_size = 2;

	/*
	 * pmu sram code and data prepare
	 */
	sram_dst = (uint32_t *)PMUSRAM_BASE;
	sram_src = (uint32_t *)&pmu_cpuson_entrypoint_start;
	sram_size = (uint32_t *)&pmu_cpuson_entrypoint_end -
		    (uint32_t *)sram_src;

	u32_align_cpy(sram_dst, sram_src, sram_size);

	psram_sleep_cfg->sp = PSRAM_DT_BASE;
}

static inline uint32_t get_cpus_pwr_domain_cfg_info(uint32_t cpu_id)
{
#if CPU_PD_CTR_FLG_HW
	uint32_t pd_reg, apm_reg;

	pd_reg = pmu_read32(PMU_PWRDN_CON) & BIT(cpu_id);
	apm_reg = pmu_read32(PMU_CPUAPM_CON(cpu_id)) & BIT(core_pm_en);

	if (pd_reg && !apm_reg)
		return core_pwr_pd;
	else if (!pd_reg && apm_reg)
		return core_pwr_wfi;

	ERROR("%s: 0x%x, 0x%x\n", __func__, pd_reg, apm_reg);
	while (1)
	;
#else
	return cores_pd_cfg_info[cpu_id];
#endif
}

static inline void set_cpus_pwr_domain_cfg_info(uint32_t cpu_id, uint32_t value)
{
#if !CPU_PD_CTR_FLG_HW
	cores_pd_cfg_info[cpu_id] = value;
#if !USE_COHERENT_MEM
	flush_dcache_range((uintptr_t) &cores_pd_cfg_info[cpu_id],
			    sizeof(uint32_t));
#endif
#endif
}

static int cpus_power_domain_on(uint32_t cpu_id)
{
	uint32_t cpu_pd, apm_value, cfg_info, loop = 0;

	cpu_pd = PD_CPU0 + cpu_id;
	cfg_info = get_cpus_pwr_domain_cfg_info(cpu_id);

	if (cfg_info == core_pwr_pd) {
		/* disable apm cfg */
		pmu_write32(CORES_PM_DISABLE, PMU_CPUAPM_CON(cpu_id));
		if (pmu_power_domain_st(cpu_pd) == pmu_pd_on) {
			pmu_write32(CORES_PM_DISABLE, PMU_CPUAPM_CON(cpu_id));
			pmu_power_domain_ctr(cpu_pd, pmu_pd_off);
		}
		pmu_power_domain_ctr(cpu_pd, pmu_pd_on);
	} else {
		/* wait cpu down */
		while (pmu_power_domain_st(cpu_pd) == pmu_pd_on && loop < 100) {
			udelay(2);
			loop++;
		}

		/* return error if can't wait cpu down */
		if (pmu_power_domain_st(cpu_pd) == pmu_pd_on) {
			WARN("%s:can't wait cpu down\n", __func__);
			return -EINVAL;
		}

		/* power up cpu in power down state */
		apm_value = BIT(core_pm_sft_wakeup_en);
		pmu_write32(apm_value, PMU_CPUAPM_CON(cpu_id));
	}

	return 0;
}

static int cpus_power_domain_off(uint32_t cpu_id, uint32_t pd_cfg)
{
	uint32_t cpu_pd, apm_value;

	cpu_pd = PD_CPU0 + cpu_id;
	if (pmu_power_domain_st(cpu_pd) == pmu_pd_off)
		return 0;

	if (pd_cfg == core_pwr_pd) {
		if (check_cpu_wfie(cpu_id, CKECK_WFEI_MSK))
			return -EINVAL;

		/* disable apm cfg */
		pmu_write32(CORES_PM_DISABLE, PMU_CPUAPM_CON(cpu_id));
		set_cpus_pwr_domain_cfg_info(cpu_id, pd_cfg);
		pmu_power_domain_ctr(cpu_pd, pmu_pd_off);
	} else {
		set_cpus_pwr_domain_cfg_info(cpu_id, pd_cfg);
		apm_value = BIT(core_pm_en) | BIT(core_pm_dis_int);
		if (pd_cfg == core_pwr_wfi_int)
			apm_value |= BIT(core_pm_int_wakeup_en);
		pmu_write32(apm_value, PMU_CPUAPM_CON(cpu_id));
	}

	return 0;
}

static void nonboot_cpus_off(void)
{
	uint32_t boot_cpu, cpu;

	/* turn off noboot cpus */
	boot_cpu = plat_my_core_pos();
	for (cpu = 0; cpu < PLATFORM_CORE_COUNT; cpu++) {
		if (cpu == boot_cpu)
			continue;
		cpus_power_domain_off(cpu, core_pwr_pd);
	}
}

static int cores_pwr_domain_on(unsigned long mpidr, uint64_t entrypoint)
{
	uint32_t cpu_id = plat_core_pos_by_mpidr(mpidr);

	assert(cpuson_flags[cpu_id] == 0);
	cpuson_flags[cpu_id] = PMU_CPU_HOTPLUG;
	cpuson_entry_point[cpu_id] = entrypoint;
	dsb();

	cpus_power_domain_on(cpu_id);

	return 0;
}

static int cores_pwr_domain_off(void)
{
	uint32_t cpu_id = plat_my_core_pos();

	cpus_power_domain_off(cpu_id, core_pwr_wfi);

	return 0;
}

static int cores_pwr_domain_suspend(void)
{
	uint32_t cpu_id = plat_my_core_pos();

	assert(cpuson_flags[cpu_id] == 0);
	cpuson_flags[cpu_id] = PMU_CPU_AUTO_PWRDN;
	cpuson_entry_point[cpu_id] = (uintptr_t)plat_get_sec_entrypoint();
	dsb();

	cpus_power_domain_off(cpu_id, core_pwr_wfi_int);

	return 0;
}

static int cores_pwr_domain_on_finish(void)
{
	uint32_t cpu_id = plat_my_core_pos();

	pmu_write32(CORES_PM_DISABLE, PMU_CPUAPM_CON(cpu_id));

	return 0;
}

static int cores_pwr_domain_resume(void)
{
	uint32_t cpu_id = plat_my_core_pos();

	pmu_write32(CORES_PM_DISABLE, PMU_CPUAPM_CON(cpu_id));

	return 0;
}

static void __dead2 soc_sys_global_soft_reset(void)
{
	cru_write32(PLL_SLOW_MODE(CPLL_ID), CRU_CRU_MODE);
	cru_write32(PLL_SLOW_MODE(GPLL_ID), CRU_CRU_MODE);
	cru_write32(PLL_SLOW_MODE(NPLL_ID), CRU_CRU_MODE);
	cru_write32(PLL_SLOW_MODE(APLL_ID), CRU_CRU_MODE);
	isb();

	cru_write32(CRU_GLB_SRST_FST_VALUE, CRU_GLB_SRST_FST);
	isb();

	/*
	 * Maybe the HW needs some times to reset the system,
	 * so we do not hope the core to excute valid codes.
	 */
	while (1)
		;
}

static int sys_pwr_domain_suspend(void)
{
	return 0;
}

static int sys_pwr_domain_resume(void)
{
	return 0;
}

static struct rockchip_pm_ops_cb pm_ops = {
	.cores_pwr_dm_on = cores_pwr_domain_on,
	.cores_pwr_dm_off = cores_pwr_domain_off,
	.cores_pwr_dm_on_finish = cores_pwr_domain_on_finish,
	.cores_pwr_dm_suspend = cores_pwr_domain_suspend,
	.cores_pwr_dm_resume = cores_pwr_domain_resume,
	.sys_pwr_dm_suspend = sys_pwr_domain_suspend,
	.sys_pwr_dm_resume = sys_pwr_domain_resume,
	.sys_gbl_soft_reset = soc_sys_global_soft_reset,
};

void bl31_plat_runtime_setup(void)
{
	/* do nothing */
	return;
}

void plat_rockchip_pmu_init(void)
{
	uint32_t cpu;

	plat_setup_rockchip_pm_ops(&pm_ops);

	for (cpu = 0; cpu < PLATFORM_CORE_COUNT; cpu++)
		cpuson_flags[cpu] = 0;

	cpu_warm_boot_addr = (uint64_t)platform_cpu_warmboot;
	psram_sleep_cfg->ddr_func = 0x00;
	psram_sleep_cfg->ddr_data = 0x00;
	psram_sleep_cfg->ddr_flag = 0x00;
	psram_sleep_cfg->boot_mpidr = read_mpidr_el1() & 0xffff;

	/* boot from ddr  */
	sgrf_write32((cpu_warm_boot_addr >> CPU_BOOT_ADDR_ALIGN) |
		     CPU_BOOT_ADDR_WMASK,
		     SGRF_SOC_CON(1));

	nonboot_cpus_off();

	rk_register_handler();

	INFO("%s: pd status 0x%x\n", __func__, pmu_read32(PMU_PWRDN_ST));
}
