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

#ifndef __DMC_H__
#define __DMC_H__

#include <mmio.h>
#include <rk3399_def.h>
#include <soc.h>

#define KHz (1000)
#define MHz (1000 * KHz)
#define GHz (1000 * MHz)

#define DDRC0_BASE_ADDR			0xFFA80000
#define DDRC1_BASE_ADDR			0xFFA88000
#define SERVER_MSCH0_BASE_ADDR		0xFFA84000
#define SERVER_MSCH1_BASE_ADDR		0xFFA8C000

#define DDR_PI_OFFSET			0x800
#define DDR_PHY_OFFSET			0x2000

#define DDRC0_PI_BASE_ADDR		(DDRC0_BASE_ADDR + DDR_PI_OFFSET)
#define DDRC0_PHY_BASE_ADDR		(DDRC0_BASE_ADDR + DDR_PHY_OFFSET)

#define DDRC1_PI_BASE_ADDR		(DDRC1_BASE_ADDR + DDR_PI_OFFSET)
#define DDRC1_PHY_BASE_ADDR		(DDRC1_BASE_ADDR + DDR_PHY_OFFSET)

enum {
	DDR3 = 3,
	LPDDR2 = 5,
	LPDDR3 = 6,
	LPDDR4 = 7,
	UNUSED = 0xFF
};

struct rk3399_ddr_cic_regs {
	uint32_t cic_ctrl0;
	uint32_t cic_ctrl1;
	uint32_t cic_idle_th;
	uint32_t cic_cg_wait_th;
	uint32_t cic_status0;
	uint32_t cic_status1;
	uint32_t cic_ctrl2;
	uint32_t cic_ctrl3;
	uint32_t cic_ctrl4;
};

struct rk3399_ddr_pctl_regs {
	uint32_t denali_ctl[332];
};

struct rk3399_ddr_publ_regs {
	uint32_t denali_phy[541];
};

struct rk3399_ddr_pi_regs {
	uint32_t denali_pi[200];
};

union noc_ddrtiminga0 {
	uint32_t d32;
	struct {
		unsigned acttoact : 6;
		unsigned reserved0 : 2;
		unsigned rdtomiss : 6;
		unsigned reserved1 : 2;
		unsigned wrtomiss : 6;
		unsigned reserved2 : 2;
		unsigned readlatency : 8;
	} b;
};

union noc_ddrtimingb0 {
	uint32_t d32;
	struct {
		unsigned rdtowr : 5;
		unsigned reserved0 : 3;
		unsigned wrtord : 5;
		unsigned reserved1 : 3;
		unsigned rrd : 4;
		unsigned reserved2 : 4;
		unsigned faw : 6;
		unsigned reserved3 : 2;
	} b;
};

union noc_ddrtimingc0 {
	uint32_t d32;
	struct {
		unsigned burstpenalty : 4;
		unsigned reserved0 : 4;
		unsigned wrtomwr : 6;
		unsigned reserved1 : 18;
	} b;
};

union noc_devtodev0 {
	uint32_t d32;
	struct {
		unsigned busrdtord : 3;
		unsigned reserved0 : 1;
		unsigned busrdtowr : 3;
		unsigned reserved1 : 1;
		unsigned buswrtord : 3;
		unsigned reserved2 : 1;
		unsigned buswrtowr : 3;
		unsigned reserved3 : 17;
	} b;
};

union noc_ddrmode {
	uint32_t d32;
	struct {
		unsigned autoprecharge : 1;
		unsigned bypassfiltering : 1;
		unsigned fawbank : 1;
		unsigned burstsize : 2;
		unsigned mwrsize : 2;
		unsigned reserved2 : 1;
		unsigned forceorder : 8;
		unsigned forceorderstate : 8;
		unsigned reserved3 : 8;
	} b;
};

struct rk3399_msch_regs {
	uint32_t coreid;
	uint32_t revisionid;
	uint32_t ddrconf;
	uint32_t ddrsize;
	union noc_ddrtiminga0 ddrtiminga0;
	union noc_ddrtimingb0 ddrtimingb0;
	union noc_ddrtimingc0 ddrtimingc0;
	union noc_devtodev0 devtodev0;
	uint32_t reserved0[(0x110 - 0x20) / 4];
	union noc_ddrmode ddrmode;
	uint32_t reserved1[(0x1000 - 0x114) / 4];
	uint32_t agingx0;
};

struct rk3399_msch_timings {
	union noc_ddrtiminga0 ddrtiminga0;
	union noc_ddrtimingb0 ddrtimingb0;
	union noc_ddrtimingc0 ddrtimingc0;
	union noc_devtodev0 devtodev0;
	union noc_ddrmode ddrmode;
	uint32_t agingx0;
};

/* DENALI_CTL_00 */
#define START		(1)

/* DENALI_CTL_68 */
#define PWRUP_SREFRESH_EXIT	BIT(1)

/* DENALI_CTL_274 */
#define MEM_RST_VALID	(1)

struct rk3399_sdram_channel {
	unsigned char rank;
	/* col = 0, means this channel is invalid */
	unsigned char col;
	/* 3:8bank, 2:4bank */
	unsigned char bk;
	/* channel buswidth, 2:32bit, 1:16bit, 0:8bit */
	unsigned char bw;
	/* die buswidth, 2:32bit, 1:16bit, 0:8bit */
	unsigned char dbw;
	/* row_3_4 = 1: 6Gb or 12Gb die
	 * row_3_4 = 0: normal die, power of 2
	 */
	unsigned char row_3_4;
	unsigned char cs0_row;
	unsigned char cs1_row;
	unsigned int ddrconfig;
	struct rk3399_msch_timings noc_timings;
};

struct rk3399_sdram_params {
	struct rk3399_sdram_channel ch[2];
	unsigned int ddr_freq;
	unsigned char dramtype;
	unsigned char num_channels;
	unsigned char stride;
	unsigned char odt;
	/* align 8 byte */
	struct rk3399_ddr_pctl_regs pctl_regs;
	/* align 8 byte */
	struct rk3399_ddr_pi_regs pi_regs;
	/* align 8 byte */
	struct rk3399_ddr_publ_regs phy_regs;
	/* used for align 8byte for next struct */
	unsigned int align_8;
};

#define PI_CA_TRAINING		BIT(0)
#define PI_WRITE_LEVELING	BIT(1)
#define PI_READ_GATE_TRAINING	BIT(2)
#define PI_READ_LEVELING	BIT(3)
#define PI_WDQ_LEVELING		BIT(4)
#define PI_FULL_TRAINING	(0xff)

/*
 * sys_reg bitfield struct
 * [31] row_3_4_ch1
 * [30] row_3_4_ch0
 * [29:28] chinfo
 * [27] rank_ch1
 * [26:25] col_ch1
 * [24] bk_ch1
 * [23:22] cs0_row_ch1
 * [21:20] cs1_row_ch1
 * [19:18] bw_ch1
 * [17:16] dbw_ch1;
 * [15:13] ddrtype
 * [12] channelnum
 * [11] rank_ch0
 * [10:9] col_ch0
 * [8] bk_ch0
 * [7:6] cs0_row_ch0
 * [5:4] cs1_row_ch0
 * [3:2] bw_ch0
 * [1:0] dbw_ch0
*/
#define SYS_REG_ENC_ROW_3_4(n, ch)	((n) << (30 + (ch)))
#define SYS_REG_DEC_ROW_3_4(n, ch)	(((n) >> (30 + (ch))) & 0x1)
#define SYS_REG_ENC_CHINFO(ch)		(1 << (28 + (ch)))
#define SYS_REG_DEC_CHINFO(n, ch)	(((n) >> (28 + (ch))) & 0x1)
#define SYS_REG_ENC_DDRTYPE(n)		((n) << 13)
#define SYS_REG_DEC_DDRTYPE(n)		(((n) >> 13) & 0x7)
#define SYS_REG_ENC_NUM_CH(n)		(((n) - 1) << 12)
#define SYS_REG_DEC_NUM_CH(n)		(1 + (((n) >> 12) & 0x1))
#define SYS_REG_ENC_RANK(n, ch)		(((n) - 1) << (11 + (ch) * 16))
#define SYS_REG_DEC_RANK(n, ch)		(1 + (((n) >> (11 + (ch) * 16)) & 0x1))
#define SYS_REG_ENC_COL(n, ch)		(((n) - 9) << (9 + (ch) * 16))
#define SYS_REG_DEC_COL(n, ch)		(9 + (((n) >> (9 + (ch) * 16)) & 0x3))
#define SYS_REG_ENC_BK(n, ch)		(((n) == 3 ? 0 : 1) << (8 + (ch) * 16))
#define SYS_REG_DEC_BK(n, ch)		(3 - (((n) >> (8 + (ch) * 16)) & 0x1))
#define SYS_REG_ENC_CS0_ROW(n, ch)	(((n) - 13) << (6 + (ch) * 16))
#define SYS_REG_DEC_CS0_ROW(n, ch)	(13 + (((n) >> (6 + (ch) * 16)) & 0x3))
#define SYS_REG_ENC_CS1_ROW(n, ch)	(((n) - 13) << (4 + (ch) * 16))
#define SYS_REG_DEC_CS1_ROW(n, ch)	(13 + (((n) >> (4 + (ch) * 16)) & 0x3))
#define SYS_REG_ENC_BW(n, ch)		((2 >> (n)) << (2 + (ch) * 16))
#define SYS_REG_DEC_BW(n, ch)		(2 >> (((n) >> (2 + (ch) * 16)) & 0x3))
#define SYS_REG_ENC_DBW(n, ch)		((2 >> (n)) << (0 + (ch) * 16))
#define SYS_REG_DEC_DBW(n, ch)		(2 >> (((n) >> (0 + (ch) * 16)) & 0x3))
#define DDR_STRIDE(n)	\
	mmio_write_32(SGRF_BASE + SGRF_SOC_CON3_7(4), \
		      (0x1f << (10 + 16)) | ((n) << 10)); \

__sramdata extern struct rk3399_sdram_params sdram_config;

void dmc_save(void);
__sramfunc void dmc_restore(void);

#endif /* __DRAM_H__ */
