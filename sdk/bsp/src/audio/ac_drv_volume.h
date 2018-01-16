/****************************************************************************
 * arch/arm/src/cxd56xx/audio/drivers/baseband/include/ac_drv_volueme.h
 *
 *   Copyright (C) 2016, 2017 Sony Corporation
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
 ***************************************************************************/
/* Description: Audio Codec volume header */

#ifndef __SDK_BSP_SRC_AUDIO_AC_DRV_VOLUME_H
#define __SDK_BSP_SRC_AUDIO_AC_DRV_VOLUME_H

/****************************************************************************
 * Included Files
 ***************************************************************************/

#include "audio/as_drv_common.h"
#include "audio/ac_drv.h"
#include "audio/ac_drv_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

#define MUTE_BIT_API  0x01
#define MUTE_BIT_FADE 0x02
#define MUTE_VOL_REG  0x33
#define VOL_WAIT_TIME 20
#define VOL_TO_REG(vol) (((vol) / 5) & 0xff)
#define VOL_MUTE_TIME(vol, n_cycle) \
  (((VOL_TO_REG(vol) - MUTE_VOL_REG) & 0xff) * (n_cycle + 1 ) * 4 / 48)

/****************************************************************************
 * Public Types
 ***************************************************************************/

typedef struct
{
  AC_REG_ID codecVolId;
  int16_t   hold_vol;
  uint8_t   mute_bit;
} setVolParam;

/****************************************************************************
 * Public Functions
 ***************************************************************************/

uint32_t setMuteVolume(asCodecVolSelId volId, bool waitFlg, uint8_t typeId);
uint32_t setUnMuteVolume(asCodecVolSelId volId,
                         bool waitFlg,
                         uint8_t typeId);
uint32_t setVolume(asCodecVolSelId volId,
                   int16_t volume,
                   bool waitFlg,
                   uint8_t typeId);
uint32_t convBeepFreqToVal(uint32_t beepFreq);

#endif /* __SDK_BSP_SRC_AUDIO_AC_DRV_VOLUME_H */
