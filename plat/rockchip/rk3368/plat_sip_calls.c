/*
 * Copyright (c) 2016, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
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

#include <debug.h>
#include <mmio.h>
#include <plat_sip_calls.h>
#include <rockchip_sip_svc.h>
#include <runtime_svc.h>

#define ACCESS_REGS_TBL_CN	1

static unsigned long access_regs_table[ACCESS_REGS_TBL_CN] = {
	0xffb00000, /* efuse */
};

static int regs_access_check(unsigned long val,
			     unsigned long addr,
			     unsigned long ctrl)
{
	uint32_t i;

	addr = addr & 0xfffff000;
	for (i = 0; i < ACCESS_REGS_TBL_CN; i++)
		if (access_regs_table[i] == addr)
			return SIP_RET_SUCCESS;

	return SIP_RET_INVALID_ADDRESS;
}

static uint64_t regs_access(uint64_t val,
			    uint64_t addr_phy,
			    uint64_t ctrl,
			    struct arm_smccc_res *res)
{
	int ret = regs_access_check(val, addr_phy, ctrl);

	if (ret)
		goto exit;

	if (ctrl & SEC_REG_WR)
		mmio_write_32(addr_phy, val);
	else
		res->a1 = mmio_read_32(addr_phy);

exit:
	return ret;
}

uint64_t rockchip_plat_sip_handler(uint32_t smc_fid,
				   uint64_t x1,
				   uint64_t x2,
				   uint64_t x3,
				   uint64_t x4,
				   void *cookie,
				   void *handle,
				   uint64_t flags)
{
	int ret = SIP_RET_DENIED;
	struct arm_smccc_res res = {0};

	switch (smc_fid) {
	case RK_SIP_ACCESS_REG32:
		ret = regs_access(x1, x2, x3, &res);
		SMC_RET2(handle, ret, res.a1);

	default:
		ERROR("%s: unhandled SMC (0x%x)\n", __func__, smc_fid);
		SMC_RET1(handle, SMC_UNK);
	}
}
