/********************************************************************************************
 * arch/arm/src/cxd56xx/audio/drivers/baseband/src/ac_drv_reg.c
 *
 *   Copyright (C) 2014 Sony Corporation. All rights reserved.
 *   Author: Naoya Haneda <Naoya.Haneda@sony.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor Sony nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************************************/
/* Description: Audio Codec register access function */

#include "audio/as_drv_common.h"
#include "audio/ac_drv_reg.h"

#ifdef AC_LOCAL_TEST
#include "debug_print.h"
#define DBG_LogPrintf(...)
uint32_t AC_SWREG[0x10000];
#else
#define AC_SWREG  0
#endif

#ifdef AC_LOCAL_TEST
void init_ac_reg( void )
{
	uint32_t i, offset;

	DEBUG_PRINT("=== %s ===\n", __func__);
	for (i = 0; i < RI_REG_MAX_ENTRY; i++) {
		write_ac_reg( (AC_REG_ID)i, acRegMap[i].init );
	}
	for (offset = DNC1_IRAM_BASE; offset < DNC2_CRAM_BASE+0x800; offset++) {
		write32_ac_reg( offset, 0x00000000 );
	}
}

void read_ac_reg_all( void )
{
	uint32_t i, offset, data;

	DEBUG_PRINT("=== %s ===\n", __func__);
	for (i = 0; i < RI_REG_MAX_ENTRY; i++) {
		offset = acRegMap[i].addr;
		data = read32_ac_reg( offset );
		DEBUG_PRINT("[addr] %04x [data] %08x\n", offset, data);
	}
	for (offset = DNC1_IRAM_BASE; offset < DNC2_CRAM_BASE+0x800; offset++) {
		data = read32_ac_reg( offset );
		DEBUG_PRINT("[addr] %04x [data] %08x\n", offset, data);
	}
}
#endif

uint32_t write_ac_reg( AC_REG_ID regId, uint32_t data )
{
	volatile uint32_t * addr;
	uint32_t mask, curr;

	addr = (volatile uint32_t *)(AC_REG_BASE + AC_SWREG + acRegMap[regId].addr);
	if (acRegMap[regId].len < 32) {
		mask = (1 << acRegMap[regId].len) - 1;
	} else {
		mask = 0xffffffff;
	}

	if( addr != NULL ) {
		curr = *addr & ~(mask << acRegMap[regId].pos);
		*addr = curr | ((data & mask) << acRegMap[regId].pos);
	} else {
		D_ASSERT(0);
	}

	return 0;
}

uint32_t read_ac_reg( AC_REG_ID regId )
{
	volatile uint32_t * addr;
	uint32_t mask, data = 0;

	addr = (volatile uint32_t *)(AC_REG_BASE + AC_SWREG + acRegMap[regId].addr);
	if (acRegMap[regId].len < 32) {
		mask = (1 << acRegMap[regId].len) - 1;
	} else {
		mask = 0xffffffff;
	}

	if( addr != NULL ) {
		data = (*addr >> acRegMap[regId].pos) & mask;
	} else {
		D_ASSERT(0);
	}

	return data;
}

uint32_t write32_ac_reg( uint32_t offset, uint32_t data )
{
	volatile uint32_t * addr;

	addr = (volatile uint32_t *)(AC_REG_BASE + AC_SWREG + offset);
	if( addr != NULL ) {
		*addr = data;
	} else {
		D_ASSERT(0);
	}

	return 0;
}

uint32_t read32_ac_reg( uint32_t offset )
{
	volatile uint32_t * addr;
	uint32_t data = 0;

	addr = (volatile uint32_t *)(AC_REG_BASE + AC_SWREG + offset);
	if( addr != NULL ) {
		data = *addr;
	} else {
		D_ASSERT(0);
	}

	return data;
}

uint32_t read_as_reg( uint32_t reg )
{
	volatile uint32_t *addr;
	uint32_t data = 0;

	if (reg == AS_INT_EN1_REG)
	  {
	  	addr = (volatile uint32_t *)AS_INT_EN1_REG_ADDR;
		data = *addr;
	  }
	else if (reg == AS_INT_IRQ1_REG)
	  {
	  	addr = (volatile uint32_t *)AS_INT_IRQ1_REG_ADDR;
	  	data = *addr;
	  }
	else
	  {
		D_ASSERT(0);
	  }
	return data;
}

uint32_t write_as_reg( uint32_t reg, uint32_t data)
{
	volatile uint32_t *addr;

	if (reg == AS_INT_EN1_REG)
	  {
		addr = (volatile uint32_t *)AS_INT_EN1_REG_ADDR;
		*addr = data;
	  }
	else
	  {
		D_ASSERT(0);
	  }
	return 0;
}
