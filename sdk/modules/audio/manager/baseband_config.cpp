/****************************************************************************
 * modules/audio/manager/baseband_config.cpp
 *
 *   Copyright (C) 2016-2017 Sony Corporation. All rights reserved.
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
 ****************************************************************************/

#include <debug.h>
#include <arch/chip/cxd56_audio.h>
#include "baseband_config.h"
#include "memutils/common_utils/common_assert.h"

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::deactivate(bbPowerId power_id)
{
  if (E_AS_OK != powerOffBaseBand(power_id))
    {
      return AS_ECODE_AUDIO_POWER_OFF_ERROR;
    }

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::initMicGain(AudioCommand &cmd)
{
  E_AS rtCode = chkMicGainParam(cmd.init_mic_gain_param.mic_gain);
  if (rtCode != E_AS_OK)
    {
      return AS_ECODE_COMMAND_PARAM_MIC_GAIN;
    }

  for (uint8_t micCh = 0; micCh < AS_MIC_CHANNEL_MAX; micCh++)
    {
      if (cmd.init_mic_gain_param.mic_gain[micCh] != AS_MICGAIN_HOLD)
        {
          m_bb_config_init_tbl.mic_gain[micCh] =
            cmd.init_mic_gain_param.mic_gain[micCh];
        }
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::setMicGain(AudioCommand &cmd)
{
  uint32_t rst = initMicGain(cmd);
  if (rst != AS_ECODE_OK)
    {
      return rst;
    }

  if (chkEnableBaseBandInput())
    {
      E_AS rtCode = AS_SetMicGain(&m_bb_config_init_tbl.mic_gain[0]);
      _info("AS_SetMicGain()\t");
      checkErrCode(rtCode, E_AS_OK);
      if (rtCode != E_AS_OK)
        {
          return AS_ECODE_SET_MIC_GAIN_ERROR;
        }
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::initI2SParam(AudioCommand &cmd)
{
  if (cmd.init_i2s_param.i2s_id >= AS_I2S_NUM)
    {
      return AS_ECODE_COMMAND_PARAM_I2S_ID;
    }
  uint8_t i2s_id = cmd.init_i2s_param.i2s_id;

  if (cmd.init_i2s_param.rate > 192000)
    {
      return AS_ECODE_COMMAND_PARAM_SAMPLING_RATE;
    }
  m_bb_config_init_tbl.rate[i2s_id] = cmd.init_i2s_param.rate;

  switch (cmd.init_i2s_param.bypass_mode_en)
    {
      case AS_I2S_BYPASS_MODE_DISABLE:
      case AS_I2S_BYPASS_MODE_ENABLE:
        m_bb_config_init_tbl.bypass_mode_en[i2s_id] =
          (asBypassModeId)cmd.init_i2s_param.bypass_mode_en;
        break;

      default:
        return AS_ECODE_COMMAND_PARAM_BYPASS_MODE;
    }

  E_AS rtCode = chkI2sParam(&m_bb_config_init_tbl.rate[0],
                            &m_bb_config_init_tbl.bypass_mode_en[0]);
  if (rtCode != E_AS_OK)
    {
      return AS_ECODE_COMMAND_PARAM_BYPASS_MODE;
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::setI2SParam(AudioCommand &cmd)
{
  uint32_t rst = initI2SParam(cmd);
  if (rst != AS_ECODE_OK)
    {
      return rst;
    }

  if (chkPowerOnBaseBand())
    {
      E_AS rtCode = AS_SetI2sParam(&m_bb_config_init_tbl.rate[0],
                                   &m_bb_config_init_tbl.bypass_mode_en[0]);
      _info("AS_SetI2sParam()\t");
      checkErrCode(rtCode, E_AS_OK);
      if (rtCode != E_AS_OK)
        {
          return AS_ECODE_SET_I2S_PARAM_ERROR;
        }
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::initOutputSelect(AudioCommand &cmd)
{
  switch (cmd.init_output_select_param.output_device_sel)
    {
      case AS_OUT_OFF:
        m_bb_config_init_tbl.output_device_sel = AS_OUT_OFF;
        break;

      case AS_OUT_SP:
        m_bb_config_init_tbl.output_device_sel = AS_OUT_SP;
        break;

      case AS_OUT_I2S:
        m_bb_config_init_tbl.output_device_sel = AS_OUT_I2S;
        break;

      default:
        return AS_ECODE_COMMAND_PARAM_OUTPUT_DEVICE;
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::setOutputSelect(AudioCommand &cmd)
{
  uint32_t rst = initOutputSelect(cmd);
  if (rst != AS_ECODE_OK)
    {
      return rst;
    }

  if (chkEnableBaseBandOutput())
    {
      E_AS rtCode =
        AS_SetOutputSelect(m_bb_config_init_tbl.output_device_sel);
      _info("AS_SetOutputSelect()\t");
      checkErrCode(rtCode, E_AS_OK);
      if (rtCode != E_AS_OK)
        {
          return AS_ECODE_SET_OUTPUT_SELECT_ERROR;
        }
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::initDEQParam(AudioCommand &cmd)
{
  /* Set DEQ parameters to member variable. Set these parameters when
   * BasebandConfig::setActiveBaseband() is called.
   */

  return AS_ECODE_COMMAND_NOT_SUPPOT;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::initDNCParam(AudioCommand &cmd)
{
  /* Set DNC parameters to member variable. Set these parameters when
   * BasebandConfig::setActiveBaseband() is called.
   */

  return AS_ECODE_COMMAND_NOT_SUPPOT;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::initClearStereo(AudioCommand &cmd)
{
  if (cmd.init_clear_stereo_param.cs_en < AS_CS_MAX_ENTRY)
    {
      m_bb_config_init_tbl.cs_en = cmd.init_clear_stereo_param.cs_en;
    }
  else
    {
      return AS_ECODE_COMMAND_PARAM_FUNCTION_ENABLE;
    }

  if (cmd.init_clear_stereo_param.cs_vol == AS_CS_VOL_HOLD)
    {
      /* Do nothing. */

    }
  else if ((AS_CS_VOL_MIN <= cmd.init_clear_stereo_param.cs_vol) &&
            (cmd.init_clear_stereo_param.cs_vol <= AS_CS_VOL_MAX))
    {
      m_bb_config_init_tbl.cs_vol = cmd.init_clear_stereo_param.cs_vol;
    }
  else
    {
      return AS_ECODE_COMMAND_PARAM_VOLLUME;
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::setClearStereo(AudioCommand &cmd)
{
  E_AS error_code;

  uint32_t rst = initClearStereo(cmd);
  if (rst != AS_ECODE_OK)
    {
      return rst;
    }

  if (chkEnableBaseBandOutput())
    {
      error_code = AS_InitClearStereo((asCsEn)m_bb_config_init_tbl.cs_en,
                                      AS_CS_SIGN_POSITIVE,
                                      m_bb_config_init_tbl.cs_vol);
      _info("AS_InitClearStereo()\t");
      checkErrCode(error_code, E_AS_OK);
      if (error_code != E_AS_OK)
        {
          return AS_ECODE_INIT_CLEAR_STEREO_ERROR;
        }
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::setVolume(AudioCommand &cmd)
{
  if (!chkEnableBaseBandOutput())
    {
      return AS_ECODE_NOT_AUDIO_DATA_PATH;
    }

  if (!CHECK_RANGE(cmd.set_volume_param.input1_db,
                   AS_VOLUME_HOLD,
                   AS_VOLUME_MIN,
                   AS_VOLUME_MAX) &&
       (AS_VOLUME_MUTE != cmd.set_volume_param.input1_db))
    {
      return AS_ECODE_COMMAND_PARAM_INPUT_DB;
    }
  if (!CHECK_RANGE(cmd.set_volume_param.input2_db,
                   AS_VOLUME_HOLD,
                   AS_VOLUME_MIN,
                   AS_VOLUME_MAX) &&
       (AS_VOLUME_MUTE != cmd.set_volume_param.input2_db))
    {
      return AS_ECODE_COMMAND_PARAM_INPUT_DB;
    }
  if (!CHECK_RANGE(cmd.set_volume_param.master_db,
                   AS_VOLUME_HOLD,
                   AS_VOLUME_MIN,
                   AS_VOLUME_MAX) &&
      (AS_VOLUME_MUTE != cmd.set_volume_param.master_db))
    {
      return AS_ECODE_COMMAND_PARAM_MASTER_DB;
    }

  /* CODEC_DSP volume. */

  asCodecVol codec_vol;
  codec_vol.input1_db  = cmd.set_volume_param.input1_db;
  codec_vol.input2_db  = cmd.set_volume_param.input2_db;
  codec_vol.master_db  = cmd.set_volume_param.master_db;

  _info("AS_SetVolume()\t");
  E_AS rtCode = AS_SetVolume(&codec_vol);
  checkErrCode(rtCode, E_AS_OK);
  if (E_AS_OK != rtCode)
    {
      return AS_ECODE_SET_VOLUME_ERROR;
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::setVolumeMute(AudioCommand &cmd)
{
  if (!chkEnableBaseBandOutput())
    {
      return AS_ECODE_NOT_AUDIO_DATA_PATH;
    }

  E_AS rtCode = E_AS_OK;
  switch (cmd.set_volume_mute_param.master_mute)
    {
      case AS_VOLUMEMUTE_HOLD:
        break;

      case AS_VOLUMEMUTE_UNMUTE:
        _info("AS_UnMuteVolume()\t");
        rtCode = AS_UnMuteVolume(AS_VOLUME_MASTER);
        checkErrCode(rtCode, E_AS_OK);
        if (E_AS_OK != rtCode)
          {
            return AS_ECODE_SET_VOLUME_MUTE_ERROR;
          }
        break;

      case AS_VOLUMEMUTE_MUTE:
        _info("AS_MuteVolume()\t");
        rtCode = AS_MuteVolume(AS_VOLUME_MASTER);
        checkErrCode(rtCode, E_AS_OK);
        if (E_AS_OK != rtCode)
          {
            return AS_ECODE_SET_VOLUME_MUTE_ERROR;
          }
        break;

      default:
        return AS_ECODE_COMMAND_PARAM_MASTER_DB;
    }

  switch (cmd.set_volume_mute_param.input1_mute)
    {
      case AS_VOLUMEMUTE_HOLD:
        break;

      case AS_VOLUMEMUTE_UNMUTE:
        _info("AS_UnMuteVolume()\t");
        rtCode = AS_UnMuteVolume(AS_VOLUME_INPUT1);
        checkErrCode(rtCode, E_AS_OK);
        if (E_AS_OK != rtCode)
          {
            return AS_ECODE_SET_VOLUME_MUTE_ERROR;
          }
        break;

      case AS_VOLUMEMUTE_MUTE:
       _info("AS_MuteVolume()\t");
       rtCode = AS_MuteVolume(AS_VOLUME_INPUT1);
       checkErrCode(rtCode, E_AS_OK);
       if (E_AS_OK != rtCode)
        {
          return AS_ECODE_SET_VOLUME_MUTE_ERROR;
        }
        break;

      default:
        return AS_ECODE_COMMAND_PARAM_INPUT_DB;
    }

  switch (cmd.set_volume_mute_param.input2_mute)
    {
      case AS_VOLUMEMUTE_HOLD:
        break;

      case AS_VOLUMEMUTE_UNMUTE:
        _info("AS_UnMuteVolume()\t");
        rtCode = AS_UnMuteVolume(AS_VOLUME_INPUT2);
        checkErrCode(rtCode, E_AS_OK);
        if (E_AS_OK != rtCode)
          {
            return AS_ECODE_SET_VOLUME_MUTE_ERROR;
          }
        break;

      case AS_VOLUMEMUTE_MUTE:
        _info("AS_MuteVolume()\t");
        rtCode = AS_MuteVolume(AS_VOLUME_INPUT2);
        checkErrCode(rtCode, E_AS_OK);
        if (E_AS_OK != rtCode)
          {
            return AS_ECODE_SET_VOLUME_MUTE_ERROR;
          }
        break;
 
      default:
        return AS_ECODE_COMMAND_PARAM_INPUT_DB;
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::setBeep(AudioCommand &cmd)
{
  if (!chkEnableBaseBandOutput())
    {
      return AS_ECODE_NOT_AUDIO_DATA_PATH;
    }

  if (cmd.set_beep_param.beep_en >= AS_BEEPEN_NUM)
    {
      return AS_ECODE_COMMAND_PARAM_FUNCTION_ENABLE;
    }

  if (!CHECK_RANGE(cmd.set_beep_param.beep_vol,
                   AS_BEEP_VOL_HOLD,
                   AS_BEEP_VOL_MIN,
                   AS_BEEP_VOL_MAX))
    {
      return AS_ECODE_COMMAND_PARAM_VOLLUME;
    }

  if (!CHECK_RANGE(cmd.set_beep_param.beep_freq,
                   AS_BEEP_FREQ_HOLD,
                   AS_BEEP_FREQ_MIN,
                   AS_BEEP_FREQ_MAX))
    {
      return AS_ECODE_COMMAND_PARAM_BEEP_FREQ;
    }

  E_AS rtCode = E_AS_OK;
  if (cmd.set_beep_param.beep_en == AS_BEEPEN_DISABLE)
    {
      _info("AS_BeepDisable()\t");
      rtCode = AS_BeepDisable();
      checkErrCode(rtCode, E_AS_OK);
      if (E_AS_OK != rtCode)
        {
          return AS_ECODE_SET_BEEP_ERROR;
        }
    }

  _info("AS_SetBeepParam()\t");
  rtCode = AS_SetBeepParam(cmd.set_beep_param.beep_freq,
                           cmd.set_beep_param.beep_vol);
  checkErrCode(rtCode, E_AS_OK);
  if (E_AS_OK != rtCode)
    {
      return AS_ECODE_SET_BEEP_ERROR;
    }

  if (cmd.set_beep_param.beep_en == AS_BEEPEN_ENABLE)
    {
      _info("AS_BeepEnable()\t");
      rtCode = AS_BeepEnable();
      checkErrCode(rtCode, E_AS_OK);
      if (E_AS_OK != rtCode)
        {
          return AS_ECODE_SET_BEEP_ERROR;
        }
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
inline asPathFromId conv_path_in(uint8_t path)
{
  switch(path)
  {
    case AS_THROUGH_PATH_IN_MIC:
      return AS_PATH_FROM_MIC12;
    case AS_THROUGH_PATH_IN_I2S1:
      return AS_PATH_FROM_I2S1;
    case AS_THROUGH_PATH_IN_I2S2:
      return AS_PATH_FROM_I2S2;
    case AS_THROUGH_PATH_IN_MIXER:
      return AS_PATH_FROM_MIXER;
    default:
      break;
  }
  return AS_PATH_FROM_MIC12;
}

inline asPathToId conv_path_out(uint8_t path)
{
  switch(path)
  {
    case AS_THROUGH_PATH_OUT_MIXER1:
      return AS_PATH_TO_MIXER1;
    case AS_THROUGH_PATH_OUT_MIXER2:
      return AS_PATH_TO_MIXER2;
    case AS_THROUGH_PATH_OUT_I2S1:
      return AS_PATH_TO_I2S1;
    case AS_THROUGH_PATH_OUT_I2S2:
      return AS_PATH_TO_I2S2;
    default:
      break;
  }
  return AS_PATH_TO_MIXER1;
}

uint32_t BasebandConfig::setThroughPath(AudioCommand &cmd)
{
  if (!chkEnableBaseBandInput() || !chkEnableBaseBandOutput())
    {
      return AS_RESPONSE_CODE_NOT_AUDIO_DATA_PATH;
    }

  if (cmd.set_through_path.path1.en && cmd.set_through_path.path2.en)
    {
      if ((cmd.set_through_path.path1.in == cmd.set_through_path.path2.in) ||
          (cmd.set_through_path.path1.out == cmd.set_through_path.path2.out))
        {
          return AS_RESPONSE_CODE_SET_AUDIO_DATA_PATH_ERROR;
        }
    }

  AS_ClearAudioDataPathAll();

  asDmacSelId dmac_id;
  asPathSelParam path_sel_param;
  path_sel_param.mic_dma_channel = 0;

  E_AS rtCode = E_AS_OK;
  if (cmd.set_through_path.path1.en)
    {
      path_sel_param.pathFrom = conv_path_in(cmd.set_through_path.path1.in);
      path_sel_param.pathTo   = conv_path_out(cmd.set_through_path.path1.out);

      rtCode = AS_SetAudioDataPath(&path_sel_param,
                                   &dmac_id,
                                   AS_DMAC_ID_NONE);
      checkErrCode(rtCode, E_AS_OK);
      if (E_AS_OK != rtCode)
        {
          return AS_RESPONSE_CODE_SET_AUDIO_DATA_PATH_ERROR;
        }

    }

  if (cmd.set_through_path.path2.en)
    {
      path_sel_param.pathFrom = conv_path_in(cmd.set_through_path.path2.in);
      path_sel_param.pathTo   = conv_path_out(cmd.set_through_path.path2.out);

      rtCode = AS_SetAudioDataPath(&path_sel_param,
                                   &dmac_id,
                                   AS_DMAC_ID_NONE);
      checkErrCode(rtCode, E_AS_OK);
      if (E_AS_OK != rtCode)
        {
          return AS_RESPONSE_CODE_SET_AUDIO_DATA_PATH_ERROR;
        }
    }

  return AS_RESPONSE_CODE_OK;
}

/*--------------------------------------------------------------------------*/
void BasebandConfig::clearBasebandInitConfig()
{
  m_bb_config_init_tbl.rate[0]           = 48000;
  m_bb_config_init_tbl.rate[1]           = 48000;
  m_bb_config_init_tbl.bypass_mode_en[0] = AS_I2S_BYPASS_MODE_DISABLE;
  m_bb_config_init_tbl.bypass_mode_en[1] = AS_I2S_BYPASS_MODE_DISABLE;
  m_bb_config_init_tbl.mic_gain[0]       = 0;
  m_bb_config_init_tbl.mic_gain[1]       = 0;
  m_bb_config_init_tbl.mic_gain[2]       = 0;
  m_bb_config_init_tbl.mic_gain[3]       = 0;
  m_bb_config_init_tbl.mic_gain[4]       = 0;
  m_bb_config_init_tbl.mic_gain[5]       = 0;
  m_bb_config_init_tbl.mic_gain[6]       = 0;
  m_bb_config_init_tbl.mic_gain[7]       = 0;
  m_bb_config_init_tbl.output_device_sel = AS_OUT_OFF;
  m_bb_config_init_tbl.format            = AS_SAMPLING_FMT_24;
  m_bb_config_init_tbl.cs_vol            = -830;
  m_bb_config_init_tbl.cs_en             = 0;
  m_bb_config_init_tbl.beep_vol          = -12;
  m_bb_config_init_tbl.beep_freq         = 4085;
}

/*--------------------------------------------------------------------------*/
E_AS BasebandConfig::powerOnBaseBand(bbPowerId power_id)
{
  E_AS error_code = E_AS_OK;

  F_ASSERT(power_id < BB_POWER_NUM);

  error_code = AS_PowerOnBaseBand(m_bb_config_init_tbl.rate,
                                  m_bb_config_init_tbl.bypass_mode_en);
  _info("AS_PowerOnBaseBand()\t");
  checkErrCode(error_code, E_AS_OK);
  if (error_code != E_AS_OK)
    {
      return error_code;
    }
  if (power_id != BB_POWER_OUTPUT)
    {
      error_code = AS_BaseBandEnable_input(AS_MICMODE_ACAPULCO,
                                           m_bb_config_init_tbl.mic_gain);
      _info("AS_BaseBandEnable_input()\t");
      checkErrCode(error_code, E_AS_OK);
      if (error_code != E_AS_OK)
        {
          return error_code;
        }
      error_code = AS_InitClearStereo((asCsEn)m_bb_config_init_tbl.cs_en,
                                      AS_CS_SIGN_POSITIVE,
                                      m_bb_config_init_tbl.cs_vol);
      _info("AS_InitClearStereo()\t");
      checkErrCode(error_code, E_AS_OK);
      if (error_code != E_AS_OK)
        {
          return error_code;
        }
    }
  if (power_id != BB_POWER_INPUT)
    {
      error_code =
        AS_BaseBandEnable_output(m_bb_config_init_tbl.output_device_sel);
      _info("AS_BaseBandEnable_output()\t");
      checkErrCode(error_code, E_AS_OK);
      if (error_code != E_AS_OK)
        {
          return error_code;
        }
    }
  return E_AS_OK;
}

/*--------------------------------------------------------------------------*/
E_AS BasebandConfig::powerOffBaseBand(bbPowerId power_id)
{
  E_AS error_code = E_AS_OK;
  bool disable_input = false;
  bool disable_output = false;

  F_ASSERT(power_id < BB_POWER_NUM);

  if (power_id != BB_POWER_OUTPUT)
    {
      error_code = AS_BaseBandDisable_input(AS_MICMODE_ACAPULCO);
      _info("AS_BaseBandDisable_input()\t");
      checkErrCode(error_code, E_AS_OK);
      if (error_code != E_AS_OK)
        {
          return error_code;
        }
      disable_input = true;
    }
  else
    {
      if (!chkEnableBaseBandInput())
        {
          disable_input = true;
        }
    }

  if (power_id != BB_POWER_INPUT)
    {
      error_code = AS_BaseBandDisable_output();
      _info("AS_BaseBandDisable_output()\t");
      checkErrCode(error_code, E_AS_OK);
      if (error_code != E_AS_OK)
        {
          return error_code;
        }
      disable_output = true;
    }
  else
    {
      if (!chkEnableBaseBandOutput())
        {
          disable_output = true;
        }
    }

  if (disable_input && disable_output)
    {
      error_code = AS_PowerOffBaseBand();
      _info("AS_PowerOffBaseBand()\t");
      checkErrCode(error_code, E_AS_OK);
      if (error_code != E_AS_OK)
        {
          return error_code;
        }
    }
  return error_code;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::setActiveBaseband(bbPowerId power_id)
{
  if (E_AS_OK != powerOnBaseBand(power_id))
    {
      return AS_ECODE_AUDIO_POWER_ON_ERROR;
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
void  BasebandConfig::checkErrCode(E_AS rtCode, E_AS okCode)
{
  if (rtCode == okCode)
    {
      _info("\t[OK]\n");
    }
  else
    {
      _err("\t[ERR] (code:%2d)\n", rtCode);
    }
}
