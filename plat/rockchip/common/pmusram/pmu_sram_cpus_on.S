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

#include <arch.h>
#include <asm_macros.S>
#include <platform_def.h>
#include <pmu_sram.h>

	.globl pmu_cpuson_entrypoint_start
	.globl pmu_cpuson_entrypoint_end
	.globl pmu_ddr_test_start
	.globl pmu_ddr_test_end
	.globl pmu_ddr_resume_satrt
	.globl pmu_ddr_resume_end

func pmu_cpuson_entrypoint
pmu_cpuson_entrypoint_start:
	ldr	x5, psram_data
check_wake_cpus:
	mrs	x0, MPIDR_EL1
	and	x1, x0, #MPIDR_CPU_MASK
	and	x0, x0, #MPIDR_CLUSTER_MASK
	orr	x0, x0, x1
	/* primary_cpu */
	ldr	w1, [x5, #PSRAM_DT_MPIDR]
	cmp	w0, w1
	b.eq	sys_wakeup
	/*
	 * If the core is not the primary cpu,
	 * force the core into wfe.
	 */
wfe_loop:
	wfe
	b	wfe_loop
sys_wakeup:
	/* check ddr flag for resume ddr */
	ldr	w2, [x5, #PSRAM_DT_DDRFLAG]
	cmp	w2, #0x0
	b.eq	sys_resume
ddr_resume:
	ldr	x2, [x5, #PSRAM_DT_SP]
	mov	sp, x2
	ldr	x1, [x5, #PSRAM_DT_DDR_FUNC]
	ldr	x0, [x5, #PSRAM_DT_DDR_DATA]
	blr	x1
sys_resume:
	ldr	x1, sys_wakeup_entry
	br	x1

	.align	3
psram_data:
	.quad	PSRAM_DT_BASE
sys_wakeup_entry:
	.quad	psci_entrypoint
pmu_cpuson_entrypoint_end:
	.word	0
endfunc pmu_cpuson_entrypoint

func pmu_ddr_test
pmu_ddr_test_start:
	mov	x5, PMU_BASE
	ldr	w0, [x5, #0x0024]
	bic	w0, w0, #0x1000
	bic	w0, w0, #0x0100
	ldr	w0, [x5, #0x0024]
	orr	w0, w0, #0x1000
	orr	w0, w0, #0x0100
	str	w0, [x5, #0x0024]
loop_1:
	ldr	w1, [x5, #0x0098]
	and	w2, w1, #0x01
	and	w3, w1, #0x04
	orr	w2, w2, w3
	cmp	w2, #0x5
	b.eq	loop_1

	mov	x5, CRU_BASE
	ldr	w0, [x5, #0x0118]
	bic	w0, w0, #0x00000030
	orr	w0, w0, #0x00300000
	orr	w0, w0, #0x00000010
	str	w0, [x5, #0x0118]
	mov	x5, PMU_BASE
	ldr	w0, [x5, #0x0024]
	bic	w0, w0, #0x1000
	bic	w0, w0, #0x0100
	str	w0, [x5, #0x0024]
loop_2:
	ldr	w1, [x5, #0x0098]
	and	w2, w1, #0x01
	and	w3, w1, #0x04
	orr	w2, w2, w3
	cmp	w2, #0x0
	b.eq	loop_2
	ret
pmu_ddr_test_end:
	.word	0
endfunc pmu_ddr_test

func pmu_ddr_resume
pmu_ddr_resume_satrt:
	mov	x5, PMU_BASE
	ldr	w0, [x5, #0x0024]
	bic	w0, w0, #0x1000
	bic	w0, w0, #0x0100
	ldr	w0, [x5, #0x0024]
	orr	w0, w0, #0x1000
	orr	w0, w0, #0x0100
	str	w0, [x5, #0x0024]
loop_3:
	ldr	w1, [x5, #0x0098]
	and	w2, w1, #0x01
	and	w3, w1, #0x04
	orr	w2, w2, w3
	cmp	w2, #0x5
	b.eq	loop_3
	mov	x5, CRU_BASE
	ldr	w0, [x5, #0x0118]
	bic	w0, w0, #0x00000030
	orr	w0, w0, #0x00300000
	orr	w0, w0, #0x00000020
	str	w0, [x5, #0x0118]
	mov	x5, PMU_BASE
	ldr	w0, [x5, #0x0024]
	bic	w0, w0, #0x1000
	bic	w0, w0, #0x0100
	str	w0, [x5, #0x0024]
loop_4:
	ldr	w1, [x5, #0x0098]
	and	w2, w1, #0x01
	and	w3, w1, #0x04
	orr	w2, w2, w3
	cmp	w2, #0x0
	b.eq	loop_4
	ret
pmu_ddr_resume_end:
endfunc pmu_ddr_resume
