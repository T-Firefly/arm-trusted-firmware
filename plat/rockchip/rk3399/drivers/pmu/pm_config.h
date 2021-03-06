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

#ifndef __PM_CONFIG_H__
#define __PM_CONFIG_H__

#define	PM_INVALID_GPIO			0xffff
#define RKPM_APIO0_SUSPEND		BIT(0)
#define RKPM_APIO1_SUSPEND		BIT(1)
#define RKPM_APIO2_SUSPEND		BIT(2)
#define RKPM_APIO3_SUSPEND		BIT(3)
#define RKPM_APIO4_SUSPEND		BIT(4)
#define RKPM_APIO5_SUSPEND		BIT(5)

#define PWM0_REGULATOR_EN		BIT(0)
#define PWM1_REGULATOR_EN		BIT(1)
#define PWM2_REGULATOR_EN		BIT(2)
#define PWM3A_REGULATOR_EN		BIT(3)
#define PWM3B_REGULATOR_EN		BIT(4)

#define RKPM_SLP_WFI			BIT(0)
#define RKPM_SLP_ARMPD			BIT(1)
#define RKPM_SLP_PERILPPD		BIT(2)
#define RKPM_SLP_DDR_RET		BIT(3)
#define RKPM_SLP_PLLPD			BIT(4)
#define RKPM_SLP_OSC_DIS		BIT(5)
#define RKPM_SLP_CENTER_PD		BIT(6)
#define RKPM_SLP_AP_PWROFF		BIT(7)
#define RKPM_SLP_32K_LF			BIT(8)

void pmu_suspend_power(void);
void pmu_resume_power(void);

#endif/* __PM_CONFIG_H__ */
