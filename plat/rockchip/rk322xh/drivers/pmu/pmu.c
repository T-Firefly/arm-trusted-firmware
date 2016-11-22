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
#include <bl31.h>
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

#define RK322XH_SUSPEND_DEBUG	0

struct rk322xh_sleep_ddr_data {
	uint32_t pmu_debug_enable;
	uint32_t debug_iomux_save;
	uint32_t pmic_sleep_save;
	uint32_t pmu_wakeup_conf0;
	uint32_t pmu_pwrmd_com;
	uint32_t cru_mode_save;
	uint32_t clk_sel0, clk_sel1, clk_sel18, clk_sel20, clk_sel24;
	uint32_t clk_ungt_save[CRU_CLKGATE_NUMS];
	uint32_t cru_plls_con_save[MAX_PLL][CRU_PLL_CON_NUMS];
};

static struct rk322xh_sleep_ddr_data ddr_data;

struct rk322xh_sleep_sram_data {
	uint32_t pmic_sleep_save;
	uint32_t pmic_sleep_gpio_save[2];
	uint32_t ddr_grf_con0;
	uint32_t dpll_con_save[CRU_PLL_CON_NUMS];
	uint32_t pd_sr_idle_save;
	uint32_t uart2_ier;
};

__sramdata static struct rk322xh_sleep_sram_data sram_data;

__sramfunc static void sram_putchar(char ch)
{
	mmio_write_32(UART2_BASE, ch);
	if (ch == '\n')
		mmio_write_32(UART2_BASE, '\r');

	while (mmio_read_32(UART2_BASE + UART_LSR) & UART_FIFO_EMPTY)
		;
}

static __sramfunc uint64_t arch_counter_get_cntpct(void)
{
	uint64_t cval;

	isb();
	cval = read_cntpct_el0();

	return cval;
}

static __sramfunc void sram_udelay(uint32_t us)
{
	uint64_t orig;
	uint64_t to_wait = 24 * us;

	/* Note: u32 math is way more than enough for our small delays */
	orig = arch_counter_get_cntpct();
	while (arch_counter_get_cntpct() - orig <= to_wait)
	       ;
}

#if RK322XH_SUSPEND_DEBUG
__sramfunc static void sram_print_num(int val)
{
	int i;
	int tmp = val;

	for (i = 1; tmp / 10; tmp /= 10, i *= 10)
		;
	for (; i >= 1; i /= 10) {
		sram_putchar('0' + (char)(val / i));
		val %= i;
	}
	sram_putchar('\n');
}

static inline void print_hex(uint32_t val)
{
	int i;
	unsigned char tmp;

	putchar('0');
	putchar('x');
	for (i = 0; i < 8; val <<= 4, ++i) {
		if ((tmp = (val & 0xf0000000) >> 28) < 10)
			putchar('0' + tmp);
		else
			putchar('a' + tmp - 10);
	}
}

static void regs_dump(uint32_t base, uint32_t start_offset, uint32_t end_offset)
{
	uint32_t i;

	for (i = start_offset; i <= end_offset; i += 4) {
		if (i % 32 == 0)
			tf_printf("\n0x%x: ", base + i);
		print_hex(mmio_read_32(base + i));
		putchar(' ');
		putchar(' ');
		putchar(' ');
		putchar(' ');
	}
}
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

#if 0
static void sys_slp_config(void)
{
	uint32_t slp_mode_cfg = 0;

	/* pmu_debug set */
	ddr_data.debug_iomux_save = grf_read32(GRF_GPIO2A_IOMUX);
	if (ddr_data.pmu_debug_enable) {
		grf_write32(REG_WMSK_BITS(2, 0, 3) |
			    REG_WMSK_BITS(2, 2, 3) |
			    REG_WMSK_BITS(2, 4, 3) |
			    REG_WMSK_BITS(2, 6, 3),
			    GRF_GPIO2A_IOMUX);
	}

	/* sleep wakeup set */
	ddr_data.pmu_wakeup_conf0 = pmu_read32(PMU_WAKEUP_CFG0);
	pmu_write32(WAKEUP_INT_CLUSTER_EN, PMU_WAKEUP_CFG0);

	/* sleep mode set */
	ddr_data.pmu_pwrmd_com = pmu_read32(PMU_PWRMD_COM);
	slp_mode_cfg = BIT(pmu_mode_en) |
			BIT(global_int_disable_cfg) |
			BIT(cpu0_pd_en) |
			BIT(wait_wakeup_begin_cfg)
			;
	pmu_write32(slp_mode_cfg, PMU_PWRMD_COM);

	/* boot address set */
	sgrf_write32((PMUSRAM_BASE >> CPU_BOOT_ADDR_ALIGN) |
		      CPU_BOOT_ADDR_WMASK, SGRF_SOC_CON(1));
}

static void sys_slp_unconfig(void)
{
	/* set boot address */
	sgrf_write32((cpu_warm_boot_addr >> CPU_BOOT_ADDR_ALIGN) |
		     CPU_BOOT_ADDR_WMASK,
		     SGRF_SOC_CON(1));

	/* sleep mode */
	pmu_write32(ddr_data.pmu_pwrmd_com, PMU_PWRMD_COM);

	/* sleep wakeup */
	pmu_write32(ddr_data.pmu_wakeup_conf0, PMU_WAKEUP_CFG0);

	/* pmu_debug set */
	if (ddr_data.pmu_debug_enable)
		grf_write32(ddr_data.debug_iomux_save | REG_W_MSK(0, 0xffff),
			    GRF_GPIO2A_IOMUX);
}
#endif

static uint32_t clk_ungt_msk[CRU_CLKGATE_NUMS] = {
	0x107f, 0x0000, 0x010c, 0x0000, 0x0200,
	0x0000, 0x0000, 0x0017, 0x001f, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0003, 0x0000,
	0xf001, 0x27c0, 0x0011, 0x03ff, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0003, 0x0008
};

static void clks_gating_suspend(uint32_t *ungt_msk)
{
	int i;

	for (i = 0; i < CRU_CLKGATE_NUMS; i++) {
		ddr_data.clk_ungt_save[i] =
					cru_read32(CRU_CLKGATE_CON(i));
#if RK322XH_SUSPEND_DEBUG
		INFO("%d  ", i);
		INFO("save: 0x%x ", cru_read32(CRU_CLKGATE_CON(i)));
		INFO("msk: 0x%x ", ungt_msk[i]);
#endif
		CLK_UNGATING(ungt_msk[i], CRU_CLKGATE_CON(i));
		/* cru_write32(0xffff0000, CRU_CLKGATE_CON(i)); */
#if RK322XH_SUSPEND_DEBUG
		INFO("gating: 0x%x\n", cru_read32(CRU_CLKGATE_CON(i)));
#endif
	}
}

static void clks_gating_resume(void)
{
	int i;

	for (i = 0; i < CRU_CLKGATE_NUMS; i++)
		cru_write32(ddr_data.clk_ungt_save[i] | 0xffff0000,
			    CRU_CLKGATE_CON(i));
}

static inline void pm_pll_wait_lock(uint32_t pll_id)
{
	uint32_t delay = 600000U;

	dsb();

	while (delay > 0) {
		if (IS_PLL_LOKED(pll_id))
			break;
		delay--;
	}
	if (delay == 0)
		ERROR("lock-pll: %d\n", pll_id);
}

static inline void pll_pwr_dwn(uint32_t pll_id, uint32_t pd)
{
	if (pd != IS_PLL_LOKED(pll_id))
		return;

	cru_write32(REG_WMSK_BITS(1, 15, 1), PLL_CONS(pll_id, 1));
	if (pd)
		cru_write32(REG_WMSK_BITS(1, 14, 1), PLL_CONS(pll_id, 1));
	else
		cru_write32(REG_WMSK_BITS(0, 14, 1), PLL_CONS(pll_id, 1));
}

__sramfunc static void dpll_suspend(void)
{
	int i;

	/* slow mode */
	cru_write32(PLL_SLOW_MODE(DPLL_ID), CRU_CRU_MODE);

	/* save pll con */
	for (i = 0; i < CRU_PLL_CON_NUMS; i++)
		sram_data.dpll_con_save[i] =
				cru_read32(PLL_CONS(DPLL_ID, i));

	cru_write32(REG_WMSK_BITS(1, 15, 1), PLL_CONS(DPLL_ID, 1));
	cru_write32(REG_WMSK_BITS(1, 14, 1), PLL_CONS(DPLL_ID, 1));
}

__sramfunc static void dpll_resume(void)
{
	uint32_t delay = 600000U;

	cru_write32(REG_WMSK_BITS(1, 15, 1), PLL_CONS(DPLL_ID, 1));
	cru_write32(REG_WMSK_BITS(0, 14, 1), PLL_CONS(DPLL_ID, 1));

	cru_write32(sram_data.dpll_con_save[1] | 0xc0000000,
		    PLL_CONS(DPLL_ID, 1));

	dsb();

	while (delay > 0) {
		if (IS_PLL_LOKED(DPLL_ID))
			break;
		delay--;
	}
	if (delay == 0)
		while (1)
			;

	cru_write32(PLL_NORM_MODE(DPLL_ID), CRU_CRU_MODE);
}

static inline void pll_suspend(uint32_t pll_id)
{
	int i;

	/* slow mode */
	cru_write32(PLL_SLOW_MODE(pll_id), CRU_CRU_MODE);

	/* save pll con */
	for (i = 0; i < CRU_PLL_CON_NUMS; i++)
		ddr_data.cru_plls_con_save[pll_id][i] =
				cru_read32(PLL_CONS(pll_id, i));

	/* powerdown pll */
	pll_pwr_dwn(pll_id, pmu_pd_off);
}

static inline void pll_resume(uint32_t pll_id)
{
	cru_write32(ddr_data.cru_plls_con_save[pll_id][1] | 0xc0000000,
		    PLL_CONS(pll_id, 1));

	pm_pll_wait_lock(pll_id);

	if (PLL_IS_NORM_MODE(ddr_data.cru_mode_save, pll_id))
		cru_write32(PLL_NORM_MODE(pll_id), CRU_CRU_MODE);
}

static void pm_plls_suspend(void)
{
	ddr_data.cru_mode_save = cru_read32(CRU_CRU_MODE);

	ddr_data.clk_sel0 = cru_read32(CRU_CLKSEL_CON(0));
	ddr_data.clk_sel1 = cru_read32(CRU_CLKSEL_CON(1));
	ddr_data.clk_sel18 = cru_read32(CRU_CLKSEL_CON(18));
	ddr_data.clk_sel20 = cru_read32(CRU_CLKSEL_CON(20));
	ddr_data.clk_sel24 = cru_read32(CRU_CLKSEL_CON(24));

#if RK322XH_SUSPEND_DEBUG
{
	int i, j;

	for (i = 0; i < MAX_PLL; ++i) {
		tf_printf("pll%d:", i);

		for (j = 0; j < CRU_PLL_CON_NUMS; ++j)
			tf_printf("0x%x ", cru_read32(PLL_CONS(i, j)));

		tf_printf("\n");
	}
}
#endif
	pll_suspend(NPLL_ID);
	pll_suspend(CPLL_ID);
	pll_suspend(GPLL_ID);
	pll_suspend(APLL_ID);

	/* core */
	cru_write32(REG_WMSK_BITS(0, 0, 0x1f),
		    CRU_CLKSEL_CON(0));

	/* pclk_dbg */
	cru_write32(REG_WMSK_BITS(0, 0, 0xf),
		    CRU_CLKSEL_CON(1));

	/* crypto */
	cru_write32(REG_WMSK_BITS(0, 0, 0x1f),
		    CRU_CLKSEL_CON(20));

	/* pwm0 */
	cru_write32(REG_WMSK_BITS(0, 8, 0x7f),
		    CRU_CLKSEL_CON(24));

	/* uart2 from 24M */
	cru_write32(REG_WMSK_BITS(2, 8, 0x3),
		    CRU_CLKSEL_CON(18));
}

static void pm_plls_resume(void)
{
	/* uart2 */
	cru_write32(ddr_data.clk_sel18 | REG_W_MSK(8, 0x3),
		    CRU_CLKSEL_CON(18));

	/* pwm0 */
	cru_write32(ddr_data.clk_sel24 | REG_W_MSK(8, 0x7f),
		    CRU_CLKSEL_CON(24));

	/* crypto */
	cru_write32(ddr_data.clk_sel20 | REG_W_MSK(0, 0x1f),
		    CRU_CLKSEL_CON(20));

	/* pclk_dbg */
	cru_write32(ddr_data.clk_sel1 | REG_W_MSK(0, 0xf),
		    CRU_CLKSEL_CON(1));

	/* core */
	cru_write32(ddr_data.clk_sel0 | REG_W_MSK(0, 0x1f),
		    CRU_CLKSEL_CON(0));

	pll_pwr_dwn(APLL_ID, pmu_pd_on);
	pll_pwr_dwn(GPLL_ID, pmu_pd_on);
	pll_pwr_dwn(CPLL_ID, pmu_pd_on);
	pll_pwr_dwn(NPLL_ID, pmu_pd_on);

	pll_resume(APLL_ID);
	pll_resume(GPLL_ID);
	pll_resume(CPLL_ID);
	pll_resume(NPLL_ID);
#if RK322XH_SUSPEND_DEBUG
{
	int i, j;

	for (i = 0; i < MAX_PLL; ++i) {
		tf_printf("pll%d:", i);

		for (j = 0; j < CRU_PLL_CON_NUMS; ++j)
			tf_printf("0x%x ", cru_read32(PLL_CONS(i, j)));

		tf_printf("\n");
	}
}
#endif
}

__sramfunc static void pmic_sleep_config(void)
{
	/* pmic_sleep gpio2_d2 */
	sram_data.pmic_sleep_save = grf_read32(PMIC_SLEEP_REG);
	sram_data.pmic_sleep_gpio_save[1] = mmio_read_32(GPIO2_BASE + 4);
	sram_data.pmic_sleep_gpio_save[0] = mmio_read_32(GPIO2_BASE);
	grf_write32(REG_WMSK_BITS(0, 4, 0x3), PMIC_SLEEP_REG);
	mmio_write_32(GPIO2_BASE + 4,
		      sram_data.pmic_sleep_gpio_save[1] | BIT(26));
	mmio_write_32(GPIO2_BASE,
		      sram_data.pmic_sleep_gpio_save[0] | BIT(26));
}

__sramfunc void pmic_sleep_unconfig(void)
{
	mmio_write_32(GPIO2_BASE, sram_data.pmic_sleep_gpio_save[0]);
	mmio_write_32(GPIO2_BASE + 4, sram_data.pmic_sleep_gpio_save[1]);
	grf_write32(sram_data.pmic_sleep_save | REG_W_MSK(0, 0xffff),
		    PMIC_SLEEP_REG);
}

inline void rockchip_set_sp(uint64_t set_sp)
{
	__asm volatile("mov sp, %0 \n"::"r" (set_sp) : "sp");
}

__sramfunc static void ddr_suspend(void)
{
	sram_data.pd_sr_idle_save = mmio_read_32(DDR_UPCTL_BASE +
						 DDR_PCTL2_PWRCTL) & SELFREF_EN;
	mmio_clrbits_32(DDR_UPCTL_BASE + DDR_PCTL2_PWRCTL, SELFREF_EN);
	sram_data.ddr_grf_con0 = mmio_read_32(DDR_GRF_BASE +
					      DDRGRF_SOC_CON(0));
	mmio_write_32(DDR_GRF_BASE, BIT_WITH_WMSK(14) | WMSK_BIT(15));
	/* override csysreq from ddrc and send valid csysreq signal to
	 * PMU
	 * csysreq is controlled by ddrc only
	 */

	/* in self-refresh */
	mmio_setbits_32(PMU_BASE + PMU_SFT_CON, BIT(0));
	while ((mmio_read_32(DDR_GRF_BASE + DDRGRF_SOC_STATUS(1)) &
		(0x03 << 12)) !=  (0x02 << 12))
		;
	/* ddr retention */
	mmio_setbits_32(PMU_BASE + PMU_SFT_CON, BIT(2));

	/* ddr gating */
	cru_write32(REG_WMSK_BITS(0x7, 4, 0x7), CRU_CLKGATE_CON(0));
	cru_write32(REG_WMSK_BITS(1, 4, 1), CRU_CLKGATE_CON(7));
	cru_write32(REG_WMSK_BITS(0x1ff, 1, 0x1ff), CRU_CLKGATE_CON(18));
	cru_write32(REG_WMSK_BITS(0x3, 0, 0x3), CRU_CLKGATE_CON(27));

	dpll_suspend();
}

__sramfunc static void ddr_resume(void)
{
	dpll_resume();

	/* ddr gating */
	cru_write32(REG_WMSK_BITS(0, 4, 0x7), CRU_CLKGATE_CON(0));
	cru_write32(REG_WMSK_BITS(0, 4, 1), CRU_CLKGATE_CON(7));
	cru_write32(REG_WMSK_BITS(0, 1, 0x1ff), CRU_CLKGATE_CON(18));
	cru_write32(REG_WMSK_BITS(0, 0, 0x3), CRU_CLKGATE_CON(27));

	/* ddr de_retention */
	mmio_clrbits_32(PMU_BASE + PMU_SFT_CON, BIT(2));
	/* exit self-refresh */
	mmio_clrbits_32(PMU_BASE + PMU_SFT_CON, BIT(0));
	while ((mmio_read_32(DDR_GRF_BASE + DDRGRF_SOC_STATUS(1)) &
		(0x03 << 12)) !=  (0x00 << 12))
		;

	mmio_write_32(DDR_GRF_BASE, sram_data.ddr_grf_con0 | 0xc0000000);
	if (sram_data.pd_sr_idle_save)
		mmio_setbits_32(DDR_UPCTL_BASE + DDR_PCTL2_PWRCTL, SELFREF_EN);
}

__sramfunc void sram_suspend(void)
{
	uint32_t cpu_id = plat_my_core_pos();
	unsigned int scr, val;

	/* enable gicc group0, i.e. fiq */
	plat_rockchip_gic_cpuif_enable();

	/* enable gicc group1, i.e. irq. for pwm wakeup */
	val = mmio_read_32(PLAT_RK_GICC_BASE);
	val |= CTLR_ENABLE_G1_BIT;
	mmio_write_32(PLAT_RK_GICC_BASE, val);

	/* disable apm cfg */
	pmu_write32(CORES_PM_DISABLE, PMU_CPUAPM_CON(cpu_id));
	isb();

	/* disable mmu and icache */
	tlbialle3();
	disable_mmu_icache_el3();
	putchar('3');

	/* ddr self-refresh and gating phy */
	ddr_suspend();
	sram_putchar('4');

	/* pmic_sleep pull high */
	pmic_sleep_config();
	sram_putchar('5');

	/* enable PhysicalIRQ bit for NS world to wake the CPU */
	scr = read_scr_el3();
	write_scr_el3(scr | SCR_IRQ_BIT);

	sram_putchar('s');

	/* disable uart clk and disable interrupt */
	sram_data.uart2_ier = mmio_read_32(UART2_BASE + UART_IER);
	mmio_write_32(UART2_BASE + UART_IER, UART_INT_DISABLE);
	cru_write32(0x20002000, CRU_CLKGATE_CON(16));
	cru_write32(0x00040004, CRU_CLKGATE_CON(2));

	/* wfi */
	isb();
	wfi();

	/* restore uart clk and reset fifo */
	cru_write32(0x20000000, CRU_CLKGATE_CON(16));
	cru_write32(0x00040000, CRU_CLKGATE_CON(2));
	mmio_write_32(UART2_BASE + UART_FCR, UART_FIFO_RESET);
	mmio_write_32(UART2_BASE + UART_IER, sram_data.uart2_ier);

	sram_putchar('w');

	/* restore SCR to the original value */
	write_scr_el3(scr);

	/* pmic voltage restore */
	pmic_sleep_unconfig();
	sram_putchar('4');

	/* ddr self-refresh exit */
	sram_udelay(100);
	ddr_resume();
	putchar('3');

	/* exit */
	bl31_warm_entrypoint();
}

void __dead2 rockchip_plat_sys_pd_pwr_dn_wfi(void)
{
	rockchip_set_sp(PSRAM_DT_BASE);

	sram_suspend();

	/* should never reach here */
	psci_power_down_wfi();
}

#define GICD_ISPEND		0x200
#define GPIO_NUMS		4
#define GPIO_INTEN		0x30
#define GPIO_INT_STATUS		0x40

#define RK_GPIO_VIRT(i)		(GPIO0_BASE + (i) * (GPIO1_BASE- GPIO0_BASE))

#define DUMP_GPIO_INTEN(ID) \
	do { \
		uint32_t en = mmio_read_32(RK_GPIO_VIRT(ID) + GPIO_INTEN); \
		if (en) { \
			tf_printf("GPIO%d_INTEN: 0x%x\n", ID, en); \
		} \
	} while (0)

static inline int fls(int x)
{
	int ret;

	__asm volatile("clz\t%0, %1" : "=r" (ret) : "r" (x));

	ret = 64 - ret;
	return ret;
}

static void report_irq(void)
{
	uint32_t irq[4], val, i;
	uint32_t irq_gpio;
	uint32_t pin, gpio_st;
	static const char *bank[] = {"a", "b", "c", "d"};

	/* gpio0~3, irq: 83, 84, 85, 86 */
	val = mmio_read_32((GIC400_BASE + 0x1000) + GICD_ISPEND + 8);
	irq_gpio = (val >> 19) & 0x0f;

	for (i = 0; i < ARRAY_SIZE(irq); i++)
		irq[i] = mmio_read_32((GIC400_BASE + 0x1000) +
				       GICD_ISPEND + (i + 1) * 4);

	for (i = 0; i < ARRAY_SIZE(irq); i++) {
		for (; irq[i]; ) {
			tf_printf(" IRQ: %d", 32 * (i + 1) + fls(irq[i]) - 1);
			irq[i] &= ~BIT(fls(irq[i]) - 1);
		}
	}

	for (i = 0; i < GPIO_NUMS; i++) {
		if (irq_gpio & (1 << i)) {
			gpio_st = mmio_read_32(RK_GPIO_VIRT(i) +
					       GPIO_INT_STATUS);
			for (; gpio_st; ) {
				pin = fls(gpio_st) - 1;
				tf_printf("  gpio%d_%s%d",
					  i, bank[pin / 8], pin % 8);
				gpio_st &= ~BIT(pin);
			}
		}
	}

	putchar('\n');
}

static void rk322x_irq_prepare(void)
{
	DUMP_GPIO_INTEN(0);
	DUMP_GPIO_INTEN(1);
	DUMP_GPIO_INTEN(2);
	DUMP_GPIO_INTEN(3);
	report_irq();
}

static void rk322x_irq_finish(void)
{
	report_irq();
}

static int sys_pwr_domain_suspend(void)
{
	putchar('\n');
	rk322x_irq_prepare();

	putchar('0');

	clks_gating_suspend(clk_ungt_msk);
	putchar('1');

	pm_plls_suspend();
	putchar('2');

	return 0;
}

static int sys_pwr_domain_resume(void)
{
	putchar('2');

	pm_plls_resume();
	putchar('1');

	clks_gating_resume();
	putchar('0');

	rk322x_irq_finish();
	putchar('\n');

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
	psram_sleep_cfg->ddr_func = (uint64_t)ddr_resume;
	psram_sleep_cfg->ddr_data = 0x00;
	psram_sleep_cfg->ddr_flag = 0x01;
	psram_sleep_cfg->boot_mpidr = read_mpidr_el1() & 0xffff;

	/* boot from ddr  */
	sgrf_write32((cpu_warm_boot_addr >> CPU_BOOT_ADDR_ALIGN) |
		     CPU_BOOT_ADDR_WMASK,
		     SGRF_SOC_CON(1));

	nonboot_cpus_off();

	rk_register_handler();

	INFO("%s: pd status 0x%x\n", __func__, pmu_read32(PMU_PWRDN_ST));
}
