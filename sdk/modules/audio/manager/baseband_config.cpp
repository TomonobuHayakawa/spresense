/****************************************************************************
 * modules/audio/manager/baseband_config.cpp
 *
 *   Copyright (C) 2016-2017 Sony Corporation. All rights reserved.
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
  if (CXD56_AUDIO_ECODE_OK != powerOffBaseBand(power_id))
    {
      return AS_ECODE_AUDIO_POWER_OFF_ERROR;
    }

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::initMicGain(AudioCommand &cmd)
{
  for (uint8_t micCh = 0; micCh < AS_MIC_CHANNEL_MAX; micCh++)
    {
      m_bb_config_init_tbl.mic_gain.gain[micCh] =
        cmd.init_mic_gain_param.mic_gain[micCh];
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

  if (m_input_en)
    {
      CXD56_AUDIO_ECODE error_code = cxd56_audio_set_micgain(&m_bb_config_init_tbl.mic_gain);
      _info("cxd56_audio_set_micgain()\t");
      checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
      if (error_code != CXD56_AUDIO_ECODE_OK)
        {
          return AS_ECODE_SET_MIC_GAIN_ERROR;
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
      case AS_OUT_SP:
      case AS_OUT_I2S:
        m_bb_config_init_tbl.output_device_sel =
          cmd.init_output_select_param.output_device_sel;
        break;

      default:
        return AS_ECODE_COMMAND_PARAM_OUTPUT_DEVICE;
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::setOutputSelect(AudioCommand &cmd)
{
  CXD56_AUDIO_ECODE error_code;
  uint32_t rst = initOutputSelect(cmd);
  if (rst != AS_ECODE_OK)
    {
      return rst;
    }

  if (m_output_en)
    {
      switch (m_bb_config_init_tbl.output_device_sel)
        {
          case AS_OUT_OFF:
            error_code = cxd56_audio_dis_output();
            _info("cxd56_audio_dis_output()\t");
            break;

          case AS_OUT_SP:
            error_code = cxd56_audio_en_output(true);
            _info("cxd56_audio_en_output(true)\t");
            break;

          case AS_OUT_I2S:
            {
              error_code = cxd56_audio_en_output(false);
              _info("cxd56_audio_en_output(false)\t");
              checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
              if (error_code != CXD56_AUDIO_ECODE_OK)
                {
                  return AS_ECODE_SET_OUTPUT_SELECT_ERROR;
                }

              /* Set Path, Mixer to I2S0 */

              cxd56_audio_signal_t sig_id = CXD56_AUDIO_SIG_MIX;
              cxd56_audio_sel_t    sel_info;
              sel_info.au_dat_sel1 = false;
              sel_info.au_dat_sel2 = false;
              sel_info.cod_insel2  = false;
              sel_info.cod_insel3  = false;
              sel_info.src1in_sel  = true;
              sel_info.src2in_sel  = false;

              error_code = cxd56_audio_set_datapath(sig_id, sel_info);
            }
            break;

          default:
            return AS_ECODE_COMMAND_PARAM_OUTPUT_DEVICE;
        }
      checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
      if (error_code != CXD56_AUDIO_ECODE_OK)
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
  m_bb_config_init_tbl.cs_en = (cmd.init_clear_stereo_param.cs_en != 0)
                                ? true : false;

  if ((AS_CS_VOL_MIN <= cmd.init_clear_stereo_param.cs_vol) &&
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
  CXD56_AUDIO_ECODE error_code;

  uint32_t rst = initClearStereo(cmd);
  if (rst != AS_ECODE_OK)
    {
      return rst;
    }

  if (m_output_en)
    {
      if (m_bb_config_init_tbl.cs_en)
        {
          error_code =
            cxd56_audio_en_cstereo(false,
                                   m_bb_config_init_tbl.cs_vol);
          _info("cxd56_audio_en_cstereo()\t");
        }
      else
        {
          error_code = cxd56_audio_dis_cstereo();
          _info("cxd56_audio_dis_cstereo()\t");
        }
      checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
      if (error_code != CXD56_AUDIO_ECODE_OK)
        {
          return AS_ECODE_INIT_CLEAR_STEREO_ERROR;
        }
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::setVolume(AudioCommand &cmd)
{
  CXD56_AUDIO_ECODE error_code = CXD56_AUDIO_ECODE_OK;

  if (!m_output_en)
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

  if (cmd.set_volume_param.input1_db != AS_VOLUME_HOLD)
    {
      _info("cxd56_audio_set_vol(in1)\t");
      error_code = cxd56_audio_set_vol(CXD56_AUDIO_VOLID_MIXER_IN1,
                                       cmd.set_volume_param.input1_db);
      checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
      if (error_code != CXD56_AUDIO_ECODE_OK)
        {
          return AS_ECODE_SET_VOLUME_ERROR;
        }
    }

  if (cmd.set_volume_param.input2_db != AS_VOLUME_HOLD)
    {
      _info("cxd56_audio_set_vol(in2)\t");
      error_code = cxd56_audio_set_vol(CXD56_AUDIO_VOLID_MIXER_IN2,
                                       cmd.set_volume_param.input2_db);
      checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
      if (error_code != CXD56_AUDIO_ECODE_OK)
        {
          return AS_ECODE_SET_VOLUME_ERROR;
        }
    }

  if (cmd.set_volume_param.master_db != AS_VOLUME_HOLD)
    {
      _info("cxd56_audio_set_vol(out)\t");
      error_code = cxd56_audio_set_vol(CXD56_AUDIO_VOLID_MIXER_OUT,
                                       cmd.set_volume_param.master_db);
      checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
      if (error_code != CXD56_AUDIO_ECODE_OK)
        {
          return AS_ECODE_SET_VOLUME_ERROR;
        }
    }

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::setVolumeMute(AudioCommand &cmd)
{
  CXD56_AUDIO_ECODE error_code = CXD56_AUDIO_ECODE_OK;

  if (!m_output_en)
    {
      return AS_ECODE_NOT_AUDIO_DATA_PATH;
    }

  switch (cmd.set_volume_mute_param.master_mute)
    {
      case AS_VOLUMEMUTE_HOLD:
        break;

      case AS_VOLUMEMUTE_UNMUTE:
        _info("cxd56_audio_unmute_vol(master)\t");
        error_code = cxd56_audio_unmute_vol(CXD56_AUDIO_VOLID_MIXER_OUT);
        checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
        if (error_code != CXD56_AUDIO_ECODE_OK)
          {
            return AS_ECODE_SET_VOLUME_MUTE_ERROR;
          }
        break;

      case AS_VOLUMEMUTE_MUTE:
        _info("cxd56_audio_mute_vol(master)\t");
        error_code = cxd56_audio_mute_vol(CXD56_AUDIO_VOLID_MIXER_OUT);
        checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
        if (error_code != CXD56_AUDIO_ECODE_OK)
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
        _info("cxd56_audio_unmute_vol(in1)\t");
        error_code = cxd56_audio_unmute_vol(CXD56_AUDIO_VOLID_MIXER_IN1);
        checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
        if (error_code != CXD56_AUDIO_ECODE_OK)
          {
            return AS_ECODE_SET_VOLUME_MUTE_ERROR;
          }
        break;

      case AS_VOLUMEMUTE_MUTE:
        _info("cxd56_audio_mute_vol(in1)\t");
        error_code = cxd56_audio_mute_vol(CXD56_AUDIO_VOLID_MIXER_IN1);
        checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
        if (error_code != CXD56_AUDIO_ECODE_OK)
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
        _info("cxd56_audio_unmute_vol(in2)\t");
        error_code = cxd56_audio_unmute_vol(CXD56_AUDIO_VOLID_MIXER_IN2);
        checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
        if (error_code != CXD56_AUDIO_ECODE_OK)
          {
            return AS_ECODE_SET_VOLUME_MUTE_ERROR;
          }
        break;

      case AS_VOLUMEMUTE_MUTE:
        _info("cxd56_audio_mute_vol(in2)\t");
        error_code = cxd56_audio_mute_vol(CXD56_AUDIO_VOLID_MIXER_IN2);
        checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
        if (error_code != CXD56_AUDIO_ECODE_OK)
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
  CXD56_AUDIO_ECODE error_code = CXD56_AUDIO_ECODE_OK;

  if (!m_output_en)
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

  if (cmd.set_beep_param.beep_en == AS_BEEPEN_DISABLE)
    {
      _info("cxd56_audio_stop_beep()\t");
      error_code = cxd56_audio_stop_beep();
      checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
      if (error_code != CXD56_AUDIO_ECODE_OK)
        {
          return AS_ECODE_SET_BEEP_ERROR;
        }
    }

  if (AS_BEEP_FREQ_HOLD != cmd.set_beep_param.beep_freq)
    {
      _info("cxd56_audio_set_beep_freq()\t");
      error_code = cxd56_audio_set_beep_freq(cmd.set_beep_param.beep_freq);
      checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
      if (error_code != CXD56_AUDIO_ECODE_OK)
        {
          return AS_ECODE_SET_BEEP_ERROR;
        }
    }

  if (AS_BEEP_VOL_HOLD != cmd.set_beep_param.beep_vol)
    {
      _info("cxd56_audio_set_beep_vol()\t");
      error_code = cxd56_audio_set_beep_vol(cmd.set_beep_param.beep_vol);
      checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
      if (error_code != CXD56_AUDIO_ECODE_OK)
        {
          return AS_ECODE_SET_BEEP_ERROR;
        }
    }

  if (cmd.set_beep_param.beep_en == AS_BEEPEN_ENABLE)
    {
      _info("cxd56_audio_play_beep()\t");
      error_code = cxd56_audio_play_beep();
      checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
      if (error_code != CXD56_AUDIO_ECODE_OK)
        {
          return AS_ECODE_SET_BEEP_ERROR;
        }
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::setRenderingClk(AudioCommand &cmd)
{
  CXD56_AUDIO_ECODE error_code = CXD56_AUDIO_ECODE_OK;
  cxd56_audio_clkmode_t mode;

  if (cmd.set_renderingclk_param.clk_mode != AS_CLKMODE_NORMAL &&
      cmd.set_renderingclk_param.clk_mode != AS_CLKMODE_HIRES)
    {
      return AS_ECODE_COMMAND_PARAM_RENDERINGCLK;
    }

  _info("cxd56_audio_set_clkmode()\t");
  mode = (cmd.set_renderingclk_param.clk_mode == AS_CLKMODE_NORMAL) ?
          CXD56_AUDIO_CLKMODE_NORMAL : CXD56_AUDIO_CLKMODE_HIRES;
  error_code = cxd56_audio_set_clkmode(mode);
  checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      return AS_ECODE_SET_RENDERINGCLK_ERROR;
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
inline cxd56_audio_signal_t conv_path_signal(uint8_t in_path)
{
  switch(in_path)
  {
    case AS_THROUGH_PATH_IN_MIC:
      return CXD56_AUDIO_SIG_MIC1;
    case AS_THROUGH_PATH_IN_I2S1:
      return CXD56_AUDIO_SIG_I2S0;
    case AS_THROUGH_PATH_IN_I2S2:
      return CXD56_AUDIO_SIG_I2S1;
    case AS_THROUGH_PATH_IN_MIXER:
      return CXD56_AUDIO_SIG_MIX;
    default:
      break;
  }
  return CXD56_AUDIO_SIG_MIC1;
}

inline cxd56_audio_sel_t conv_path_sel(uint8_t in_path, uint8_t out_path)
{
  cxd56_audio_sel_t sel_info;
  sel_info.au_dat_sel1 = false;
  sel_info.au_dat_sel2 = false;
  sel_info.cod_insel2  = false;
  sel_info.cod_insel3  = false;
  sel_info.src1in_sel  = false;
  sel_info.src2in_sel  = false;

  switch(in_path)
  {
    case AS_THROUGH_PATH_IN_MIC:
      {
        sel_info.au_dat_sel1 = true;
        if (AS_THROUGH_PATH_OUT_MIXER1 == out_path)
          {
            sel_info.cod_insel2  = true;
          }
        else if (AS_THROUGH_PATH_OUT_MIXER2 == out_path)
          {
            sel_info.cod_insel3  = true;
          }
        else if (AS_THROUGH_PATH_OUT_I2S1 == out_path)
          {
            sel_info.src1in_sel  = true;
          }
        else
          {
            sel_info.src2in_sel  = true;
          }
      }
      break;

    case AS_THROUGH_PATH_IN_I2S1:
      {
        if (AS_THROUGH_PATH_OUT_MIXER1 == out_path)
          {
            sel_info.cod_insel2 = true;
          }
        else if (AS_THROUGH_PATH_OUT_MIXER2 == out_path)
          {
            sel_info.cod_insel3  = true;
          }
      }
      break;

    case AS_THROUGH_PATH_IN_I2S2:
      {
        sel_info.au_dat_sel1 = true;
        if (AS_THROUGH_PATH_OUT_MIXER1 == out_path)
          {
            sel_info.cod_insel2  = true;
          }
        else if (AS_THROUGH_PATH_OUT_MIXER2 == out_path)
          {
            sel_info.cod_insel3  = true;
          }
      }
      break;

    case AS_THROUGH_PATH_IN_MIXER:
      {
        if (AS_THROUGH_PATH_OUT_I2S1 == out_path)
          {
            sel_info.src1in_sel  = true;
          }
        else
          {
            sel_info.src2in_sel  = true;
          }
      }
      break;

    default:
      break;
  }

  return sel_info;
}

uint32_t BasebandConfig::setThroughPath(AudioCommand &cmd)
{
  CXD56_AUDIO_ECODE error_code = CXD56_AUDIO_ECODE_OK;
  cxd56_audio_signal_t sig_id;
  cxd56_audio_sel_t    sel_info;

  if (!m_input_en || !m_output_en)
    {
      return AS_ECODE_NOT_AUDIO_DATA_PATH;
    }

  if (cmd.set_through_path.path1.en && cmd.set_through_path.path2.en)
    {
      if ((cmd.set_through_path.path1.in == cmd.set_through_path.path2.in) ||
          (cmd.set_through_path.path1.out == cmd.set_through_path.path2.out))
        {
          return AS_ECODE_SET_AUDIO_DATA_PATH_ERROR;
        }
    }

  checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      return AS_ECODE_SET_AUDIO_DATA_PATH_ERROR;
    }

  if (cmd.set_through_path.path1.en)
    {
      sig_id   = conv_path_signal(cmd.set_through_path.path1.in);
      sel_info = conv_path_sel(cmd.set_through_path.path1.in,
                               cmd.set_through_path.path1.out);

      error_code = cxd56_audio_set_datapath(sig_id, sel_info);
      checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
      if (error_code != CXD56_AUDIO_ECODE_OK)
        {
          return AS_ECODE_SET_AUDIO_DATA_PATH_ERROR;
        }

    }

  if (cmd.set_through_path.path2.en)
    {
      sig_id   = conv_path_signal(cmd.set_through_path.path2.in);
      sel_info = conv_path_sel(cmd.set_through_path.path2.in,
                               cmd.set_through_path.path2.out);
      error_code = cxd56_audio_set_datapath(sig_id, sel_info);
      checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
      if (error_code != CXD56_AUDIO_ECODE_OK)
        {
          return AS_ECODE_SET_AUDIO_DATA_PATH_ERROR;
        }
    }

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
void BasebandConfig::clearBasebandInitConfig()
{
  for (int i = 0; i < CXD56_AUDIO_MIC_CH_MAX; i++)
    {
      m_bb_config_init_tbl.mic_gain.gain[i]  = 0;
    }
  m_bb_config_init_tbl.output_device_sel = AS_OUT_OFF;
  m_bb_config_init_tbl.cs_vol            = -830;
  m_bb_config_init_tbl.cs_en             = false;
  m_bb_config_init_tbl.beep_vol          = -12;
  m_bb_config_init_tbl.beep_freq         = 4085;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE BasebandConfig::powerOnBaseBand(bbPowerId power_id)
{
  CXD56_AUDIO_ECODE error_code = CXD56_AUDIO_ECODE_OK;

  F_ASSERT(power_id < BB_POWER_NUM);

  error_code = cxd56_audio_poweron();
  _info("cxd56_audio_poweron()\t");
  checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      return error_code;
    }
  if ((power_id != BB_POWER_OUTPUT) && !m_input_en)
    {
      error_code = cxd56_audio_en_input(&m_bb_config_init_tbl.mic_gain);
      _info("cxd56_audio_en_input()\t");
      checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
      if (error_code != CXD56_AUDIO_ECODE_OK)
        {
          return error_code;
        }
      if (m_bb_config_init_tbl.cs_en)
        {
          error_code =
            cxd56_audio_en_cstereo(false,
                                   m_bb_config_init_tbl.cs_vol);
          _info("cxd56_audio_en_cstereo()\t");
        }
      else
        {
          error_code = cxd56_audio_dis_cstereo();
          _info("cxd56_audio_dis_cstereo()\t");
        }
      checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
      if (error_code != CXD56_AUDIO_ECODE_OK)
        {
          return error_code;
        }
      m_input_en = true;
    }
  if ((power_id != BB_POWER_INPUT) && !m_output_en)
    {
       switch (m_bb_config_init_tbl.output_device_sel)
        {
          case AS_OUT_SP:
            error_code= cxd56_audio_en_output(true);
            _info("cxd56_audio_en_output(true)\t");
            break;

          case AS_OUT_I2S:
            {
              error_code = cxd56_audio_en_output(false);
              _info("cxd56_audio_en_output(false)\t");
              checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
              if (error_code != CXD56_AUDIO_ECODE_OK)
                {
                  return AS_ECODE_SET_OUTPUT_SELECT_ERROR;
                }

              /* Set Path, Mixer to I2S0 */

              cxd56_audio_signal_t sig_id = CXD56_AUDIO_SIG_MIX;
              cxd56_audio_sel_t    sel_info;
              sel_info.au_dat_sel1 = false;
              sel_info.au_dat_sel2 = false;
              sel_info.cod_insel2  = false;
              sel_info.cod_insel3  = false;
              sel_info.src1in_sel  = true;
              sel_info.src2in_sel  = false;

              error_code = cxd56_audio_set_datapath(sig_id, sel_info);
            }
            break;

          default:
            error_code= cxd56_audio_dis_output();
            _info("cxd56_audio_dis_output()\t");
            break;
        }
      checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
      if (error_code != CXD56_AUDIO_ECODE_OK)
        {
          return error_code;
        }
      m_output_en = true;
    }
  return CXD56_AUDIO_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE BasebandConfig::powerOffBaseBand(bbPowerId power_id)
{
  CXD56_AUDIO_ECODE error_code = CXD56_AUDIO_ECODE_OK;

  F_ASSERT(power_id < BB_POWER_NUM);

  if (!m_input_en && !m_output_en)
    {
      return error_code;
    }

  if ((power_id != BB_POWER_OUTPUT) && m_input_en)
    {

      error_code = cxd56_audio_dis_input();
      _info("cxd56_audio_dis_input()\t");
      checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
      if (error_code != CXD56_AUDIO_ECODE_OK)
        {
          return error_code;
        }

      m_input_en = false;
    }

  if ((power_id != BB_POWER_INPUT) && m_output_en)
    {
      error_code = cxd56_audio_dis_output();
      _info("cxd56_audio_dis_output()\t");
      checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
      if (error_code != CXD56_AUDIO_ECODE_OK)
        {
          return error_code;
        }
      m_output_en = false;
    }

  if (!m_input_en && !m_output_en)
    {
      error_code = cxd56_audio_poweroff();
      _info("cxd56_audio_poweroff()\t");
      checkErrCode(error_code, CXD56_AUDIO_ECODE_OK);
      if (error_code != CXD56_AUDIO_ECODE_OK)
        {
          return error_code;
        }
    }
  return error_code;
}

/*--------------------------------------------------------------------------*/
uint32_t BasebandConfig::setActiveBaseband(bbPowerId power_id)
{
  if (CXD56_AUDIO_ECODE_OK != powerOnBaseBand(power_id))
    {
      return AS_ECODE_AUDIO_POWER_ON_ERROR;
    }
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
void  BasebandConfig::checkErrCode(CXD56_AUDIO_ECODE rtCode, CXD56_AUDIO_ECODE okCode)
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
