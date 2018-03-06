/****************************************************************************
 * nuttx/arch/arm/src/cxd56xx/audio/drivers/baseband/src/ac_drv_volume.c
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

#include <unistd.h>

#include "audio/aca_drv.h"
#include "audio/ac_drv_volume.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define audioDelayTask(ms) usleep((ms) * 1000)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static setVolParam gVolParam[AS_VOLUME_NUM] =
{
  {RI_SDIN1_VOL, AS_VOLUME_MUTE, MUTE_BIT_API},
  {RI_SDIN2_VOL, AS_VOLUME_MUTE, MUTE_BIT_API},
  {RI_DAC_VOL,   AS_VOLUME_MUTE, MUTE_BIT_API}
};

static bool gBeepEn = false;

const uint16_t beepFreqTable[] =
{
   120,  127,  134,  142,  151,  160,  169,  180,  190,  201,  214,  226,
   240,  254,  269,  285,  302,  320,  339,  360,  381,  403,  428,  453,
   480,  509,  539,  571,  606,  642,  681,  719,  762,  810,  857,  910,
   965, 1021, 1079, 1143, 1215, 1289, 1362, 1444, 1536, 1627, 1714, 1829,
  1939, 2043, 2182, 2313, 2400, 2560, 2704, 2866, 3048, 3200, 3429, 3623,
  3840, 4085,   94
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t setMuteVolume(asCodecVolSelId volId, bool waitFlg, uint8_t typeId)
{
  uint32_t waitTime = 0;

  if (gVolParam[volId].mute_bit == 0)
    {
      waitTime = VOL_MUTE_TIME(gVolParam[volId].hold_vol, 1);

      write_ac_reg(gVolParam[volId].codecVolId, MUTE_VOL_REG);
      if (waitFlg)
        {
          audioDelayTask(waitTime);
        }
    }
  else
    {
      if (waitFlg)
        {
          uint32_t tempWaitTime = VOL_MUTE_TIME(gVolParam[volId].hold_vol, 1);
          audioDelayTask(tempWaitTime);
        }
    }

  gVolParam[volId].mute_bit |= typeId;
  outputDeviceUpdate();

  return waitTime;
}

uint32_t setUnMuteVolume(asCodecVolSelId volId, bool waitFlg, uint8_t typeId)
{
  uint32_t waitTime = 0;

  gVolParam[volId].mute_bit &= ~typeId;
  outputDeviceUpdate();
  if (gVolParam[volId].mute_bit == 0)
    {
      if (typeId == MUTE_BIT_API)
        {
          waitTime = VOL_WAIT_TIME;
        }
      else
        {
          waitTime = VOL_MUTE_TIME(gVolParam[volId].hold_vol, 1);
        }

      uint32_t vol = VOL_TO_REG(gVolParam[volId].hold_vol);
      write_ac_reg(gVolParam[volId].codecVolId, vol);
      if (waitFlg)
        {
          audioDelayTask(waitTime);
        }
    }

  return waitTime;
}

uint32_t setVolume(asCodecVolSelId volId,
                   int16_t volume,
                   bool waitFlg,
                   uint8_t typeId )
{
  uint32_t waitTime = 0;

  gVolParam[volId].hold_vol = volume;

  waitTime = setUnMuteVolume(volId, waitFlg, typeId);

  return waitTime;
}

E_AS AS_SetVolume(FAR asCodecVol *pCodecVol)
{
  E_AS rtCode = E_AS_OK;

  if (!chkEnableBaseBandOutput())
    {
      return E_AS_SETVOL_POWER_ON_OUT_CHK_ERR;
    }
  if (pCodecVol == NULL)
    {
      return E_AS_CODEC_VOL_NULL;
    }
  if (bb_config_add_tbl.output_device_sel == AS_OUT_DEV_OFF)
    {
      return E_AS_SETVOL_OUTDEV_ERR;
    }

  uint32_t vol_wait = 0;
  uint32_t vol_wait_tmp = 0;
  int16_t volume = AS_VOLUME_MUTE;

  for (asCodecVolSelId volId = AS_VOLUME_INPUT1; volId < AS_VOLUME_NUM;
       volId++)
    {

      switch (volId)
        {
          case AS_VOLUME_INPUT1:
            volume = pCodecVol->input1_db;
            break;

          case AS_VOLUME_INPUT2:
            volume = pCodecVol->input2_db;
            break;

          case AS_VOLUME_MASTER:
            volume = pCodecVol->master_db;
            break;

          default:
            /* Do nothing. */
            break;
        }

      if (volume == AS_VOLUME_MUTE)
        {
          vol_wait_tmp = setMuteVolume(volId, false, MUTE_BIT_API);
          if (vol_wait < vol_wait_tmp)
            {
              vol_wait = vol_wait_tmp;
            }
        }
      else if (volume == AS_VOLUME_HOLD)
        {
          /* Do nothing. */
        }
      else if ((AS_VOLUME_MIN <= volume) && (volume <= AS_VOLUME_MAX))
        {
          vol_wait_tmp = setVolume(volId, volume, false, MUTE_BIT_API);
          if (vol_wait < vol_wait_tmp)
            {
              vol_wait = vol_wait_tmp;
            }
        }
      else
        {
          return E_AS_CODEC_VOL_PARAM;
        }
    }

  if (vol_wait != 0)
    {
      audioDelayTask(vol_wait);
    }

  return rtCode;
}

E_AS AS_MuteVolume(asCodecVolSelId volId)
{
  E_AS rtCode = E_AS_OK;

  if (!chkEnableBaseBandOutput())
    {
      return E_AS_MUTE_POWER_ON_OUT_CHK_ERR;
    }

  if (volId >= AS_VOLUME_NUM)
    {
      return E_AS_CODEC_VOL_ID_PARAM;
    }
  if (bb_config_add_tbl.output_device_sel == AS_OUT_DEV_OFF)
    {
      return E_AS_MUTE_OUTDEV_ERR;
    }

  setMuteVolume(volId, true, MUTE_BIT_API);

  return rtCode;
}

E_AS AS_UnMuteVolume(asCodecVolSelId volId)
{
  E_AS rtCode = E_AS_OK;

  if (!chkEnableBaseBandOutput())
    {
      return E_AS_UNMUTE_POWER_ON_OUT_CHK_ERR;
    }
  if (volId >= AS_VOLUME_NUM)
    {
      return E_AS_CODEC_VOL_ID_PARAM;
    }
  if (bb_config_add_tbl.output_device_sel == AS_OUT_DEV_OFF)
    {
      return E_AS_UNMUTE_OUTDEV_ERR;
    }
  setUnMuteVolume(volId, true, MUTE_BIT_API);

  return rtCode;
}

E_AS muteVolumeFade(asCodecVolSelId volId, bool waitFlg)
{
  E_AS rtCode = E_AS_OK;

  if (volId >= AS_VOLUME_NUM)
    {
      return E_AS_CODEC_VOL_ID_PARAM;
    }
  setMuteVolume(volId, waitFlg, MUTE_BIT_FADE);

  return rtCode;
}

E_AS unMuteVolumeFade(asCodecVolSelId volId, bool waitFlg)
{
  E_AS rtCode = E_AS_OK;

  if (volId >= AS_VOLUME_NUM)
    {
      return E_AS_CODEC_VOL_ID_PARAM;
    }
  setUnMuteVolume(volId, waitFlg, MUTE_BIT_FADE);

  return rtCode;
}

E_AS AS_SetBeepParam(uint32_t beepFreq, int32_t beepVol)
{
  E_AS rtCode = E_AS_OK;
  uint32_t val;

  if (!chkEnableBaseBandOutput())
    {
      return E_AS_SETBEEP_POWER_ON_OUT_CHK_ERR;
    }
  if (bb_config_add_tbl.output_device_sel == AS_OUT_DEV_OFF)
    {
      return E_AS_SETBEEP_OUTDEV_ERR;
    }

  if (!CHECK_RANGE(beepVol, AS_BEEP_VOL_HOLD, AS_BEEP_VOL_MIN,
                   AS_BEEP_VOL_MAX))
    {
      return E_AS_BEEP_VOL_PARAM;
    }
  if (!CHECK_RANGE(beepFreq, AS_BEEP_FREQ_HOLD, AS_BEEP_FREQ_MIN,
                   AS_BEEP_FREQ_MAX))
    {
      return E_AS_BEEP_FREQ_PARAM;
    }

  write_ac_reg(RI_BEEP_ON, 0);

  if (beepVol == AS_BEEP_VOL_HOLD)
    {
      /* Do nothing. */
    }
  else
    {
      val = -beepVol / 3;
      write_ac_reg(RI_BEEP_VOL, val);
    }

  if (beepFreq == AS_BEEP_FREQ_HOLD)
    {
      /* Do nothing. */
    }
  else
    {
      val = convBeepFreqToVal(beepFreq);
      write_ac_reg(RI_BEEP_FREQ, val);
    }

  if (gBeepEn == true)
    {
      write_ac_reg(RI_BEEP_ON, 1);
    }

  return rtCode;
}
E_AS AS_BeepDisable(void)
{
  E_AS rtCode = E_AS_OK;

  if (!chkEnableBaseBandOutput())
    {
      return E_AS_BEEPDIS_POWER_ON_OUT_CHK_ERR;
    }
  if (bb_config_add_tbl.output_device_sel == AS_OUT_DEV_OFF)
    {
      return E_AS_BEEPDIS_OUTDEV_ERR;
    }

  gBeepEn = false;
  write_ac_reg(RI_BEEP_ON, 0);

  outputDeviceUpdate();

  return rtCode;
}
E_AS AS_BeepEnable(void)
{
  E_AS rtCode = E_AS_OK;

  if (!chkEnableBaseBandOutput())
    {
      return E_AS_BEEPENA_POWER_ON_OUT_CHK_ERR;
    }
  if (bb_config_add_tbl.output_device_sel == AS_OUT_DEV_OFF)
    {
      return E_AS_BEEPENA_OUTDEV_ERR;
    }

  gBeepEn = true;

  outputDeviceUpdate();

  write_ac_reg(RI_BEEP_ON, 1);
  return rtCode;
}

uint32_t convBeepFreqToVal(uint32_t beepFreq)
{
  uint32_t prev;

  for (uint32_t i = 0; i < sizeof(beepFreqTable) / sizeof(uint16_t); i++)
    {
      prev = (i + 62) % 63;
      if (beepFreq < beepFreqTable[i])
        {
        if (prev == 62)
          {
            break;
          }
        else if(beepFreqTable[prev] <= beepFreq)
          {
            break;
          }
        }
    }

  return prev;

}

void outputDeviceUpdate(void)
{
  if (bb_config_add_tbl.output_device_sel == AS_OUT_DEV_OFF)
    {
      return;
    }

  if (bb_config_add_tbl.sp_offon == 0)
    {
      if ((gBeepEn) || (gVolParam[AS_VOLUME_MASTER].mute_bit == 0))
        {
          bb_config_add_tbl.sp_offon = 1;
#ifndef CONFIG_CXD56_AUDIO_ANALOG_NONE
          AS_AcaControl(AS_ACA_SET_OUTPUT_DEVICE, (uint32_t)AS_OUT_DEV_SP);
#endif
        }
    }
  else
    {
      if ((!gBeepEn) && (gVolParam[AS_VOLUME_MASTER].mute_bit != 0))
        {
          bb_config_add_tbl.sp_offon = 0;
#ifndef CONFIG_CXD56_AUDIO_ANALOG_NONE
          AS_AcaControl(AS_ACA_SET_OUTPUT_DEVICE, (uint32_t)AS_OUT_DEV_OFF);
#endif
        }
    }
}
