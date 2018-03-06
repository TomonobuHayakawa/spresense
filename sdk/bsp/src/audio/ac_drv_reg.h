/****************************************************************************
 * arch/arm/src/cxd56xx/audio/drivers/baseband/include/ac_drv_reg.h
 *
 *   Copyright (C) 2014, 2017 Sony Corporation
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
/* Description: Audio Codec register access function */

#ifndef __SDK_BSP_SRC_AUDIO_AC_DRV_REG_H
#define __SDK_BSP_SRC_AUDIO_AC_DRV_REG_H

/****************************************************************************
 * Included Files
 ***************************************************************************/

#include <arch/chip/cxd56_audio.h>
#include "audio/ac_reg_map.h"

/****************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/****************************************************************************
 * Public Types
 ***************************************************************************/

/****************************************************************************
 * Public Data
 ***************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/****************************************************************************
 * Public Functions
 ***************************************************************************/

uint32_t write_ac_reg(AC_REG_ID regId, uint32_t data);
uint32_t read_ac_reg(AC_REG_ID regId);
uint32_t write32_ac_reg(uint32_t offset, uint32_t data);
uint32_t read32_ac_reg(uint32_t offset);

/* Debug function */
void init_ac_reg(void);
void read_ac_reg_all(void);

#ifdef __cplusplus
} /* end of extern "C" */
#endif /* __cplusplus */

#endif /* __SDK_BSP_SRC_AUDIO_AC_DRV_REG_H */
