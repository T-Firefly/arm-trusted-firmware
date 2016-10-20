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

#ifndef __SOC_H__
#define __SOC_H__

#ifndef BIT
#define BIT(nr)			(1 << (nr))
#endif

/******************************* stimer ***************************************/
#define TIMER_LOADE_COUNT0	0x00
#define TIMER_LOADE_COUNT1	0x04
#define TIMER_CURRENT_VALUE0	0x08
#define TIMER_CURRENT_VALUE1	0x0C
#define TIMER_CONTROL_REG	0x10
#define TIMER_INTSTATUS		0x18
#define TIMER_EN		0x1

void platform_soc_init(void);
extern const unsigned char rockchip_power_domain_tree_desc[];

/**************************** read/write **************************************/
#define REG_W_MSK(shift, msk)		((msk) << ((shift) + 16))
#define REG_SET_BITS(bits, shift, msk)	(((bits) & (msk)) << (shift))
#define REG_WMSK_BITS(bits, shift, msk)	(REG_W_MSK(shift, msk) |\
						REG_SET_BITS(bits, shift, msk))

#define pmu_read32(offset) mmio_read_32(PMU_BASE + offset)
#define pmu_write32(v, offset)\
	do { mmio_write_32(PMU_BASE + offset, v); dsb(); } while (0)

#define cru_read32(offset) mmio_read_32(CRU_BASE + offset)
#define cru_write32(v, offset)\
	do { mmio_write_32(CRU_BASE + offset, v); dsb(); } while (0)

#define grf_read32(offset) mmio_read_32(GRF_BASE + offset)
#define grf_write32(v, offset)\
	do { mmio_write_32(GRF_BASE + offset, v); dsb(); } while (0)

#define sgrf_read32(offset) mmio_read_32(SGRF_BASE + offset)
#define sgrf_write32(v, offset)\
	do { mmio_write_32(SGRF_BASE + offset, v); dsb(); } while (0)

#define firewall_ddr_read32(offset) mmio_read_32(FIREWALL_DDR_BASE + offset)
#define firewall_ddr_write32(v, offset)\
	do { mmio_write_32(FIREWALL_DDR_BASE + offset, v); dsb(); } while (0)

#define firewall_cfg_read32(offset) mmio_read_32(FIREWALL_CFG_BASE + offset)
#define firewall_cfg_write32(v, offset)\
	do { mmio_write_32(FIREWALL_CFG_BASE + offset, v); dsb(); } while (0)

#define stimer_read32(n, offset) mmio_read_32(STIME_BASE + 0x20 * n + offset)
#define stimer_write32(n, v, offset)\
	do { mmio_write_32(STIME_BASE + 0x20 * n + offset, v); dsb();} while (0)

/**************************** cru *********************************************/
enum plls_id {
	APLL_ID = 0,
	DPLL_ID,
	CPLL_ID,
	GPLL_ID,
	REVERVE,
	NPLL_ID,
	MAX_PLL,
};

#define CRU_CRU_MODE		0x0080
#define CRU_CRU_MISC		0x0084
#define CRU_GLB_SRST_FST	0x009c
#define CRU_GLB_SRST_FST_VALUE	0xfdb9
#define PLL_CONS(id, i)		(0x020 * (id) + ((i) * 4))
#define CRU_CLKSEL_CON(i)	(0x100 + ((i) * 4))
#define CRU_CLKSEL_NUMS		53
#define CRU_CLKGATE_CON(i)	(0x200 + ((i) * 4))
#define CRU_CLKGATE_NUMS	29
#define CRU_SOFTRSTS_CON(n)	(0x300 + ((n) * 4))
#define CRU_SOFTRSTS_NUMS	12

/* PLLn_CON1 */
#define PLL_IS_LOCKED		BIT(10)
/* PLLn_CON0 */
#define PLL_BYPASS		REG_WMSK_BITS(1, 15, 0x1)
#define PLL_NO_BYPASS		REG_WMSK_BITS(0, 15, 0x1)
/* CRU_MODE */
#define PLL_SLOW_MODE(id)			\
		((id) == NPLL_ID) ?		\
		REG_WMSK_BITS(0, 1, 0x1) : REG_WMSK_BITS(0, ((id)*4), 0x1)
#define PLL_NORM_MODE(id)			\
		((id) == NPLL_ID) ?		\
		REG_WMSK_BITS(1, 1, 0x1) : REG_WMSK_BITS(1, ((id)*4), 0x1)

#define CRU_GATEID_CONS(ID)	(0x200 + (ID / 16) * 4)
#define CRU_CONS_GATEID(i)	(16 * (i))
#define GATE_ID(reg, bit)	((reg * 16) + bit)
#define CLK_GATING(msk, reg)	cru_write32((msk << 16) | 0xffff, reg)
#define CLK_UNGATING(msk, reg)	cru_write32(((~msk) << 16) | 0xffff, reg)
#define CRU_UNGATING_OPS(id)	cru_write32(REG_WMSK_BITS(0, (id) % 16, 0x1), \
					    CRU_GATEID_CONS(id))
#define CRU_GATING_OPS(id)	cru_write32(REG_WMSK_BITS(1, (id) % 16, 0x1), \
					    CRU_GATEID_CONS(id))

/************************** config regs ***************************************/
#define CHN(n)				(n)		/* timer channel */
#define FIREWALL_CFG_FW_SYS_CON(n)	(0x000 + (n) * 4)
#define FIREWALL_DDR_FW_DDR_RGN(n)	(0x000 + (n) * 4)
#define FIREWALL_DDR_FW_DDR_MST(n)	(0x020 + (n) * 4)
#define GRF_SOC_CON(n)			(0x400 + (n) * 4)
#define GRF_SOC_STATUS(n)		(0x480 + (n) * 4)
#define GRF_CPU_STATUS(n)		(0x520 + (n) * 4)
#define DDRGRF_SOC_CON(n)		(0x000 + (n) * 4)
#define DDRGRF_SOC_STATUS(n)		(0x100 + (n) * 4)
#define SGRF_SOC_CON(n)			(0x000 + (n) * 4)
#define SGRF_DMAC_CON(n)		(0x100 + (n) * 4)
#define SGRF_HDCP_KEY_CON(n)		(0x280 + (n) * 4)

/************************** regs func *****************************************/
#define STIMER_S			BIT(23)
#define SGRF_SLV_S_ALL_NS		0x0
#define SGRF_MST_S_ALL_NS		0xffffffff
#define DMA_IRQ_BOOT_NS			0xffffffff
#define DMA_MANAGER_BOOT_NS		0x80008000
#define DMA_PERI_CH_NS_15_0		0xffffffff
#define DMA_PERI_CH_NS_19_16		0x000f000f
#define DMA_SOFTRST_REQ			0x01000100
#define DMA_SOFTRST_RLS			0x01000000

/************************** cpu ***********************************************/
#define CPU_BOOT_ADDR_WMASK		0xffff0000
#define CPU_BOOT_ADDR_ALIGN		16

#endif /* __SOC_H__ */
