/****************************************************************************
 * arch/arm/src/cxd56xx/audio/drivers/baseband/src/bca_drv_reg.c
 *
 *   Copyright (C) 2016, 2017 Sony Corporation
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
 ***************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/chip/cxd56_audio.h>

#include "audio/as_drv_common.h"
#include "audio/ac_reg_map.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef AC_LOCAL_TEST
#  include "debug_print.h"
#  define DBG_LogPrintf(...)
uint32_t BCA_SWREG[0x10000];
#else
#  define BCA_SWREG 0
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef AC_LOCAL_TEST
void init_bca_reg(void)
{
  uint32_t i;

  DEBUG_PRINT("=== %s ===\n", __func__);
  for (i = 0; i < BCA_REG_MAX_ENTRY; i++)
    {
      write_bca_reg((BCA_REG_ID)i, bcaRegMap[i].init);
    }
}

void read_bca_reg_all(void)
{
  uint32_t i;
  uint32_t offset;
  uint32_t data;

  DEBUG_PRINT("=== %s ===\n", __func__);
  for (i = 0; i < BCA_REG_MAX_ENTRY; i++)
    {
      offset = bcaRegMap[i].addr;
      data = read32_bca_reg(offset);
      DEBUG_PRINT("[addr] %04x [data] %08x\n", offset, data);
    }
}
#endif

uint32_t write_bca_reg(BCA_REG_ID regId, uint32_t data)
{
  volatile uint32_t *addr;
  uint32_t mask;
  uint32_t curr;

  addr = (volatile uint32_t *)(AC_REG_BASE + BCA_SWREG +
                               bcaRegMap[regId].addr);
  if (bcaRegMap[regId].len < 32)
    {
      mask = (1 << bcaRegMap[regId].len) - 1;
    }
  else
    {
      mask = 0xffffffff;
    }

  if (addr != NULL)
    {
      curr = *addr & ~(mask << bcaRegMap[regId].pos);
      *addr = curr | ((data & mask) << bcaRegMap[regId].pos);
    }
  else
    {
      D_ASSERT(0);
    }

  return 0;
}

uint32_t write_bca_reg_mask(BCA_REG_ID regId)
{
  volatile uint32_t *addr;
  uint32_t mask;

  addr = (volatile uint32_t *)(AC_REG_BASE + BCA_SWREG +
                               bcaRegMap[regId].addr);
  if (bcaRegMap[regId].len < 32)
    {
      mask = (1 << bcaRegMap[regId].len) - 1;
    }
  else
    {
      mask = 0xffffffff;
    }

  if (addr != NULL)
    {
      *addr = mask << bcaRegMap[regId].pos;
    }
  else
    {
      D_ASSERT(0);
    }

  return 0;
}

uint32_t read_bca_reg(BCA_REG_ID regId)
{
  volatile uint32_t *addr;
  uint32_t mask;
  uint32_t data = 0;

  addr = (volatile uint32_t *)(AC_REG_BASE + BCA_SWREG +
                               bcaRegMap[regId].addr);
  if (bcaRegMap[regId].len < 32)
    {
     mask = (1 << bcaRegMap[regId].len) - 1;
    }
  else
    {
      mask = 0xffffffff;
    }

  if (addr != NULL)
    {
      data = (*addr >> bcaRegMap[regId].pos) & mask;
    }
  else
    {
      D_ASSERT(0);
    }

  return data;
}

uint32_t write32_bca_reg(uint32_t offset, uint32_t data)
{
  volatile uint32_t *addr;

  addr = (volatile uint32_t *)(AC_REG_BASE + BCA_SWREG + offset);
  if (addr != NULL)
    {
      *addr = data;
    }
  else
    {
      D_ASSERT(0);
    }

  return 0;
}

uint32_t read32_bca_reg(uint32_t offset)
{
  volatile uint32_t *addr;
  uint32_t data = 0;

  addr = (volatile uint32_t *)(AC_REG_BASE + BCA_SWREG + offset);
  if (addr != NULL)
    {
      data = *addr;
    }
  else
    {
      D_ASSERT(0);
    }

  return data;
}
