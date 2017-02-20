/*
 * Copyright (C) 2017, Fuzhou Rockchip Electronics Co., Ltd.
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

#ifndef __FIQ_DFS_H__
#define __FIQ_DFS_H__

#include <stdint.h>
#include <rockchip_sip_svc.h>

/*
 *		How to use these interfaces ?
 *
 * MCU leading ddr freq scaling, sequence(only one step):
 *	1. mcu_dfs_governor_register() to register fiqs;
 *
 * CPUx leading ddr freq scaling, sequence:
 *	1. implement _weak function: dfs_wait_cpus_wfe();
 *	2. cpu_dfs_governor_register() to register fiq;
 *	3. fiq_dfs_stop_cpus() to stop cpus except leader cpu;
 *	4. fiq_dfs_flush_l1(), fiq_dfs_flush_l2(), fiq_dfs_prefetch();
 *	5. ddr drivers does routine: ddr freq scaling;
 *	6. fiq_dfs_active_cpus() to active cpus.
 */

int fiq_dfs_stop_cpus(void);
void fiq_dfs_active_cpus(void);
void fiq_dfs_flush_l1(void);
void fiq_dfs_flush_l2(void);
void fiq_dfs_prefetch(void (*prefetch_fn)(void));

/* weak function, must implement !! */
int dfs_wait_cpus_wfe(void);

/* CPUx dcf */
int cpu_dfs_governor_register(void);

/* MCU dcf */
uint64_t mcu_dfs_governor_register(uint32_t dfs_irq, uint32_t tgt_cpu,
				   struct arm_smccc_res *res);

static inline uint64_t rockchip_get_sp(void)
{
	uint64_t save_sp;

	__asm volatile("mov %0, sp" : "=r" (save_sp) : : "sp");
	return save_sp;
}

static inline void rockchip_set_sp(uint64_t set_sp)
{
	__asm volatile("mov sp, %0" : : "r" (set_sp) : "sp");
}

#endif
