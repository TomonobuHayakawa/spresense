/****************************************************************************
 * arch/arm/src/cxd56xx/audio/drivers/baseband/include/audio_io_config.h
 *
 *   Copyright (C) 2015, 2017 Sony Corporation
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
/* Description: Audio BaseBand I/O Settings */

#ifndef __SDK_BSP_SRC_AUDIO_AUDIO_IO_CONFIG_H
#define __SDK_BSP_SRC_AUDIO_AUDIO_IO_CONFIG_H

/****************************************************************************
 * Included Files
 ***************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/****************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

typedef enum
{
  AUDIO_IO_LOWEMI_UNKNOWN  = 0,
  AUDIO_IO_LOWEMI_4MA,
  AUDIO_IO_LOWEMI_2MA,
  AUDIO_IO_LOWEMI_NUM
} audioIoLowemi;

typedef enum
{
  AUDIO_IO_I2S_MODE_UNKNOWN  = 0,
  AUDIO_IO_I2S_MODE_MASTER,
  AUDIO_IO_I2S_MODE_SLAVE,
  AUDIO_IO_I2S_MODE_NUM
} audioIoI2sMode;

typedef enum
{
  AUDIO_IO_I2S_SEL_I2S0  = 0,
  AUDIO_IO_I2S_SEL_I2S1,
  AUDIO_IO_I2S_SEL_NUM
} audioIoI2sSel;

/****************************************************************************
 * Public Types
 ***************************************************************************/

/****************************************************************************
 * Public Functions
 ***************************************************************************/

void setAudioIoMclk(void);
void setAudioIoPdm(audioIoLowemi lowemi);
void setAudioIoI2s(audioIoI2sSel i2s_sel,
                   audioIoI2sMode i2s_mode,
                   audioIoLowemi lowemi);

#ifdef __cplusplus
} /* end of extern "C" */
#endif /* __cplusplus */

#endif /* __SDK_BSP_SRC_AUDIO_AUDIO_IO_CONFIG_H */
