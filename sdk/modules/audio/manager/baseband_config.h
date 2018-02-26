/****************************************************************************
 * modules/audio/manager/baseband_config.h
 *
 *   Copyright (C) 2015-2017 Sony Corporation. All rights reserved.
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


/** maximum value for above parameter */
#define AS_CS_VOL_MAX           -195
/** minimum value for above parameter */
#define AS_CS_VOL_MIN           -825

/** maximum value for above parameters */
#define AS_VOLUME_MAX           120
/** minimum value for above parameters */
#define AS_VOLUME_MIN           -1020
/** mute setting for above parameters */
#define AS_VOLUME_MUTE          -1025


/** maximum value for above parameter */
#define AS_BEEP_VOL_MAX         0
/** minimum value for above parameter */
#define AS_BEEP_VOL_MIN         -90

/** maximum value for above parameter */
#define AS_BEEP_FREQ_MAX        4085
/** minimum value for above parameter */
#define AS_BEEP_FREQ_MIN        94

#define CHECK_RANGE(val, hold, min, max) \
          (((val) == (hold)) || \
          (((min) <= ((int32_t)val)) && ((val) <= (max))))

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
    cxd56_audio_mic_gain_t mic_gain;             /* MIC gain.            */
    uint8_t        output_device_sel;            /* Output device ID.    */
    int16_t        cs_vol;                       /* Clear stereo volume. */
    bool           cs_en;                        /* Clear stereo mode.   */
    int32_t        beep_vol;                     /* Beep volume.         */
    uint32_t       beep_freq;                    /* Beep frequency.      */
};
typedef struct BaseBandConfigInit_s BaseBandConfigInit_t;

class BasebandConfig
{
public:
  BasebandConfig() :
    m_input_en(false),
    m_output_en(false)
  {
    clearBasebandInitConfig();
  };

  uint32_t initMicGain(AudioCommand &cmd);
  uint32_t setMicGain(AudioCommand &cmd);
  uint32_t initOutputSelect(AudioCommand &cmd);
  uint32_t setOutputSelect(AudioCommand &cmd);
  uint32_t initDEQParam(AudioCommand &cmd);
  uint32_t initDNCParam(AudioCommand &cmd);
  uint32_t initClearStereo(AudioCommand &cmd);
  uint32_t setClearStereo(AudioCommand &cmd);
  uint32_t setVolume(AudioCommand &cmd);
  uint32_t setVolumeMute(AudioCommand &cmd);
  uint32_t setBeep(AudioCommand &cmd);
  uint32_t setRenderingClk(AudioCommand &cmd);
  uint32_t setThroughPath(AudioCommand &cmd);

  uint32_t deactivate(bbPowerId power_id);
  uint32_t setActiveBaseband(bbPowerId power_id);

private:
  CXD56_AUDIO_ECODE  powerOnBaseBand(bbPowerId power_id);
  CXD56_AUDIO_ECODE  powerOffBaseBand(bbPowerId power_id);
  void  clearBasebandInitConfig(void);
  void  checkErrCode(CXD56_AUDIO_ECODE, CXD56_AUDIO_ECODE);
  bool  chkEnableBaseBandInput(void);
  bool  chkEnableBaseBandOutput(void);

  BaseBandConfigInit_t m_bb_config_init_tbl;
  bool m_input_en;
  bool m_output_en;
};

#endif /* __BASEBAND_CONFIG_H */
