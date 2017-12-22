/****************************************************************************
 * modules/audio/manager/baseband_config.h
 *
 *   Copyright (C) 2015-2017 Sony Corporation. All rights reserved.
 *   Author: Tomonobu Hayakawa <Tomonobu.Hayakawa@sony.com>
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
 ****************************************************************************/

#ifndef __BASEBAND_CONFIG_H
#define __BASEBAND_CONFIG_H

#include "memutils/common_utils/common_types.h"
#include "audio/audio_high_level_api.h"

enum bbPowerId
{
    BB_POWER_INPUT,
    BB_POWER_OUTPUT,
    BB_POWER_BOTH,
    BB_POWER_NUM
};

/* Init command parameter. */

struct BaseBandConfigInit_s
{
    uint32_t       rate[AS_I2S_NUM];             /* I2S data rate.       */
    asBypassModeId bypass_mode_en[AS_I2S_NUM];   /* I2S SRC bypass mode. */
    int32_t        mic_gain[AS_MIC_CHANNEL_MAX]; /* MIC gain.            */
    asOutDeviceId  output_device_sel;            /* Output device ID.    */
    uint8_t        format;                       /* Data format.         */
    int16_t        cs_vol;                       /* Clear stereo volume. */
    uint8_t        cs_en;                        /* Clear stereo mode.   */
    int32_t        beep_vol;                     /* Beep volume.         */
    uint32_t       beep_freq;                    /* Beep frequency.      */
};
typedef struct BaseBandConfigInit_s BaseBandConfigInit_t;

class BasebandConfig
{
public:
  BasebandConfig()
  {
    clearBasebandInitConfig();
  };

  uint32_t initMicGain(AudioCommand &cmd);
  uint32_t setMicGain(AudioCommand &cmd);
  uint32_t initI2SParam(AudioCommand &cmd);
  uint32_t setI2SParam(AudioCommand &cmd);
  uint32_t initOutputSelect(AudioCommand &cmd);
  uint32_t setOutputSelect(AudioCommand &cmd);
  uint32_t initDEQParam(AudioCommand &cmd);
  uint32_t initDNCParam(AudioCommand &cmd);
  uint32_t initClearStereo(AudioCommand &cmd);
  uint32_t setClearStereo(AudioCommand &cmd);
  uint32_t setVolume(AudioCommand &cmd);
  uint32_t setVolumeMute(AudioCommand &cmd);
  uint32_t setBeep(AudioCommand &cmd);

  uint32_t deactivate(bbPowerId power_id);
  uint32_t setActiveBaseband(bbPowerId power_id);

private:
  E_AS  powerOnBaseBand(bbPowerId power_id);
  E_AS  powerOffBaseBand(bbPowerId power_id);
  void  clearBasebandInitConfig(void);
  void  checkErrCode(E_AS, E_AS);

  BaseBandConfigInit_t m_bb_config_init_tbl;
};

#endif /* __BASEBAND_CONFIG_H */
