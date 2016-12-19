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

#include <assert.h>
#include <debug.h>
#include <mmio.h>
#include <plat_sip_calls.h>
#include <rockchip_sip_svc.h>
#include <runtime_svc.h>
#include <uuid.h>
#include <xlat_tables.h>

/* Rockchip SiP Service UUID */
DEFINE_SVC_UUID(rk_sip_svc_uid,
		0xe86fc7e2, 0x313e, 0x11e6, 0xb7, 0x0d,
		0x8f, 0x88, 0xee, 0x74, 0x7b, 0x72);

#pragma weak rockchip_plat_sip_handler


int sip_version_handler(struct arm_smccc_res *res)
{
	res->a1 = SIP_IMPLEMENT_V2;

	return SIP_RET_SUCCESS;
}

/****************************** share mem smc *********************************/
#pragma weak fiq_debugger_smc_handler
uint64_t fiq_debugger_smc_handler(uint64_t fun_id, void *handle,
                                 uint64_t arg0, uint64_t arg1,
                                 struct arm_smccc_res *res)
{
	ERROR("%s: unhandled SMC (0x%lx)\n", __func__, fun_id);
	return 0;
}

#ifdef SHARE_MEM_BASE

#define SIZE_PAGE(n)	((n) << 12)

static struct share_mem_manage share_mm[SHARE_MEM_PAGE_NUM];

int share_mem_type2page_base(share_page_type_t page_type, uint64_t *out_value)
{
	int i;

	assert(IS_PAGE_ALIGNED(SHARE_MEM_BASE));

	if ((page_type <= SHARE_PAGE_TYPE_INVALID) ||
	    (page_type >= SHARE_PAGE_TYPE_MAX))
		return SIP_RET_INVALID_PARAMS;

	for (i = 0; i < ARRAY_SIZE(share_mm); i++) {
		if (share_mm[i].page_type == page_type) {
			*out_value = share_mm[i].page_base;
			return SIP_RET_SUCCESS;
		}
	}

	return SIP_RET_INVALID_PARAMS;
}

int share_mem_page_get_handler(uint64_t page_num, share_page_type_t page_type,
			       struct arm_smccc_res *res)
{
	uint32_t i;
	uint64_t page_base;
	static uint32_t page_index, mm_index;	/* point to unused one */

	assert(IS_PAGE_ALIGNED(SHARE_MEM_BASE));

	/* invalid page_num or request too many pages */
	if ((page_num <= 0) ||
	    (SHARE_MEM_PAGE_NUM < page_num) ||
	    (SHARE_MEM_PAGE_NUM < page_index + page_num))
		return SIP_RET_INVALID_PARAMS;

	/* invalid page_type */
	if ((page_type <= SHARE_PAGE_TYPE_INVALID) ||
	    (page_type >= SHARE_PAGE_TYPE_MAX))
		return SIP_RET_INVALID_PARAMS;

	/* page_type has been ocuppied */
	for (i = 0; i < ARRAY_SIZE(share_mm); i++) {
		if (share_mm[i].page_type == page_type)
			return SIP_RET_INVALID_PARAMS;
	}

	/* find memory page base */
	page_base = (uint64_t)(SHARE_MEM_BASE + SIZE_PAGE(page_index));
	res->a1 = page_base;

	/* update memory page index */
	page_index += page_num;

	/* record memory page info */
	share_mm[mm_index].page_base = page_base;
	share_mm[mm_index].page_type = page_type;
	share_mm[mm_index].page_num = page_num;
	mm_index++;

	return SIP_RET_SUCCESS;
}

uint64_t share_mem_type2page_size(share_page_type_t page_type)
{
	uint32_t i;

	for (i = 0; i < ARRAY_SIZE(share_mm); i++) {
		if (share_mm[i].page_type == page_type)
			return SIZE_PAGE(share_mm[i].page_num);
	}

	return SIP_RET_INVALID_PARAMS;
}

#else
int share_mem_page_get_handler(uint64_t page_num, share_page_type_t page_type,
			       struct arm_smccc_res *res)
{
	return SIP_RET_NOT_SUPPORTED;
}

int share_mem_type2page_base(share_page_type_t page_type, uint64_t *out_value)
{
	return SIP_RET_NOT_SUPPORTED;
}

uint64_t share_mem_type2page_size(share_page_type_t page_type)
{
	return SIP_RET_NOT_SUPPORTED;
}
#endif

uint64_t rockchip_plat_sip_handler(uint32_t smc_fid,
				   uint64_t x1,
				   uint64_t x2,
				   uint64_t x3,
				   uint64_t x4,
				   void *cookie,
				   void *handle,
				   uint64_t flags)
{
	ERROR("%s: unhandled SMC (0x%x)\n", __func__, smc_fid);
	SMC_RET1(handle, SMC_UNK);
}

/*
 * This function is responsible for handling all SiP calls from the NS world
 */
uint64_t sip_smc_handler(uint32_t smc_fid,
			 uint64_t x1,
			 uint64_t x2,
			 uint64_t x3,
			 uint64_t x4,
			 void *cookie,
			 void *handle,
			 uint64_t flags)
{
	uint32_t ns;
	int ret;
	struct arm_smccc_res res;

	/* Determine which security state this SMC originated from */
	ns = is_caller_non_secure(flags);
	if (!ns)
		SMC_RET1(handle, SMC_UNK);

	switch (smc_fid) {
	case SIP_SVC_CALL_COUNT:
		/* Return the number of Rockchip SiP Service Calls. */
		SMC_RET1(handle,
			 RK_COMMON_SIP_NUM_CALLS + RK_PLAT_SIP_NUM_CALLS);

	case SIP_SVC_UID:
		/* Return UID to the caller */
		SMC_UUID_RET(handle, rk_sip_svc_uid);

	case SIP_SVC_VERSION:
		/* Return the version of current implementation */
		SMC_RET2(handle, RK_SIP_SVC_VERSION_MAJOR,
			RK_SIP_SVC_VERSION_MINOR);

	case RK_SIP_SIP_VERSION32:
		ret = sip_version_handler(&res);
		SMC_RET2(handle, ret, res.a1);

	case RK_SIP_UARTDBG_CFG64:
		ret = fiq_debugger_smc_handler(x3, handle, x1, x2, &res);
		SMC_RET2(handle, ret, res.a1);

	case RK_SIP_SHARE_MEM32:
		ret = share_mem_page_get_handler(x1, x2, &res);
		SMC_RET4(handle, ret, res.a1, res.a2, res.a3);

	default:
		return rockchip_plat_sip_handler(smc_fid, x1, x2, x3, x4,
			cookie, handle, flags);
	}
}

/* Define a runtime service descriptor for fast SMC calls */
DECLARE_RT_SVC(
	rockchip_sip_svc,
	OEN_SIP_START,
	OEN_SIP_END,
	SMC_TYPE_FAST,
	NULL,
	sip_smc_handler
);
