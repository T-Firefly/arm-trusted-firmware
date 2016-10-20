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
#include <console.h>
#include <debug.h>
#include <delay_timer.h>
#include <mmio.h>
#include <platform_def.h>
#include <plat_private.h>
#include <rk322xh_def.h>
#include <soc.h>

/* Table of regions to map using the MMU. */
const mmap_region_t plat_rk_mmap[] = {
	MAP_REGION_FLAT(UART2_BASE, UART2_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(PMU_BASE, PMU_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(SGRF_BASE, SGRF_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(GPIO0_BASE, GPIO0_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(GPIO1_BASE, GPIO1_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(GPIO2_BASE, GPIO2_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(GPIO3_BASE, GPIO3_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(CRU_BASE, CRU_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(GRF_BASE, GRF_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(FIREWALL_DDR_BASE, FIREWALL_DDR_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(FIREWALL_CFG_BASE, FIREWALL_CFG_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(STIME_BASE, STIME_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(GIC400_BASE, GIC400_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(INTMEM_BASE, INTMEM_SIZE,
			MT_MEMORY | MT_RW | MT_SECURE),
	{ 0 }
};

/* The RockChip power domain tree descriptor */
const unsigned char rockchip_power_domain_tree_desc[] = {
	/* No of root nodes */
	PLATFORM_SYSTEM_COUNT,
	/* No of children for the root node */
	PLATFORM_CLUSTER_COUNT,
	/* No of children for the first cluster node */
	PLATFORM_CLUSTER0_CORE_COUNT,
};

void secure_timer_init(void)
{
	stimer_write32(CHN(1), 0xffffffff, TIMER_LOADE_COUNT0);
	stimer_write32(CHN(1), 0xffffffff, TIMER_LOADE_COUNT1);

	/* auto reload & enable the timer */
	stimer_write32(CHN(1), TIMER_EN, TIMER_CONTROL_REG);
}

void sgrf_init(void)
{
	/* set ddr rgn0_top and rga0_top as 0 */
	firewall_ddr_write32(0x0, FIREWALL_DDR_FW_DDR_RGN(0));

	/* set all slave ip into no-secure, except stimer */
	firewall_cfg_write32(SGRF_SLV_S_ALL_NS, FIREWALL_CFG_FW_SYS_CON(0));
	firewall_cfg_write32(SGRF_SLV_S_ALL_NS, FIREWALL_CFG_FW_SYS_CON(1));
	firewall_cfg_write32(SGRF_SLV_S_ALL_NS | STIMER_S,
			     FIREWALL_CFG_FW_SYS_CON(2));
	firewall_cfg_write32(SGRF_SLV_S_ALL_NS, FIREWALL_CFG_FW_SYS_CON(3));

	/* set all master ip into no-secure */
	sgrf_write32(0xf0000000, SGRF_SOC_CON(2));
	sgrf_write32(SGRF_MST_S_ALL_NS, SGRF_SOC_CON(3));
	sgrf_write32(SGRF_MST_S_ALL_NS, SGRF_SOC_CON(4));

	/* set DMAC into no-secure */
	sgrf_write32(DMA_IRQ_BOOT_NS, SGRF_DMAC_CON(3));
	sgrf_write32(DMA_PERI_CH_NS_15_0, SGRF_DMAC_CON(4));
	sgrf_write32(DMA_PERI_CH_NS_19_16, SGRF_DMAC_CON(5));
	sgrf_write32(DMA_MANAGER_BOOT_NS, SGRF_DMAC_CON(5));

	/* soft reset dma before use */
	cru_write32(DMA_SOFTRST_REQ, CRU_SOFTRSTS_CON(3));
	udelay(5);
	cru_write32(DMA_SOFTRST_RLS, CRU_SOFTRSTS_CON(3));
}

void plat_rockchip_soc_init(void)
{
	secure_timer_init();
	sgrf_init();

	NOTICE("BL31: Release version: %d.%d\n", MAJOR_VERSION, MINOR_VERSION);
}
