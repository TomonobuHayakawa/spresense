/****************************************************************************
 * nuttx/arch/arm/src/cxd56xx/audio/drivers/baseband/src/ac_drv_path.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <unistd.h>
#include <time.h>
#include <debug.h>

#include <arch/chip/cxd56_audio.h>

#include "audio/aca_drv.h"
#include "audio/ac_drv_reg.h"
#include "audio/ac_drv_path.h"
#include "audio/common_assert.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define audioDelayTask(ms) usleep((ms) * 1000)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static asPathFromId fromSel[FROM_SELECTOR_NUM] =
{
  AS_PATH_FROM_NONE,
  AS_PATH_FROM_NONE
};

asPathFromId toSpI2s[TO_SPI2S_NUM] =
{
  AS_PATH_FROM_NONE,
  AS_PATH_FROM_NONE,
  AS_PATH_FROM_NONE,
  AS_PATH_FROM_NONE
};

static const uint8_t dataPathCombiBit[9] =
{
  0x01,  /* AS_PATH_FROM_MIC1_8 */
  0x1E,  /* AS_PATH_FROM_MIC12  */
  0x1E,  /* AS_PATH_FROM_MIC34  */
  0x1E,  /* AS_PATH_FROM_MIC56  */
  0x1E,  /* AS_PATH_FROM_MIC78  */
  0x07,  /* AS_PATH_FROM_I2S1   */
  0x07,  /* AS_PATH_FROM_I2S2   */
  0x1E,  /* AS_PATH_FROM_DMAC   */
  0x18   /* AS_PATH_FROM_MIXER  */
};

/* 0bit:AS_PATH_FROM_MIC1_8, 5bit:AS_PATH_FROM_I2S1, 6bit:AS_PATH_FROM_I2S2 */
static uint8_t dataPathToDmacBit = 0;

uint64_t m_mic_boot_start_time = 0xFFFFFFFFFFFFFFFFull;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

E_AS AS_SetAudioDataPath(FAR asPathSelParam *pPathSelParam,
                         FAR asDmacSelId *getDmacId,
                         asDmacSelId setDmacId)
{
  E_AS rtCode = E_AS_OK;
  uint8_t pathToId;
  asPathFromId dmacid = AS_PATH_FROM_NONE;

  if (!chkPowerOnBaseBand())
    {
      return E_AS_SET_DP_POWER_ON_CHK_ERR;
    }
  if (pPathSelParam == NULL)
    {
      return E_AS_PATH_SEL_NULL;
    }

  if (pPathSelParam->pathFrom >= AS_PATH_FROM_MAX_ENTRY)
    {
      return E_AS_PATH_SEL_FROM_PARAM;
    }
  if (pPathSelParam->pathTo >= AS_PATH_TO_MAX_ENTRY)
    {
      return E_AS_PATH_SEL_TO_PARAM;
    }

  rtCode = chkDataPath(pPathSelParam->pathFrom, pPathSelParam->pathTo);
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  switch (pPathSelParam->pathFrom)
    {
      case AS_PATH_FROM_MIC1_8:
      case AS_PATH_FROM_MIC12:
      case AS_PATH_FROM_MIC34:
      case AS_PATH_FROM_MIC56:
      case AS_PATH_FROM_MIC78:
        if (m_mic_boot_start_time != 0xFFFFFFFFFFFFFFFFull)
          {
            D_ASSERT(bb_config_tblp != NULL);
            struct timespec end;
            int ret = clock_gettime(CLOCK_REALTIME, &end);
            F_ASSERT(ret == 0);
            uint64_t time = (unsigned long long)end.tv_sec * 1000 +
                            (unsigned long long)end.tv_nsec / 1000000 -
                             m_mic_boot_start_time;

            if (time < bb_config_tblp->mic_boot_wait)
              {
                _info("AnalogMic active wait:%dmsec\n",
                       (int)(bb_config_tblp->mic_boot_wait - time));
                audioDelayTask(bb_config_tblp->mic_boot_wait - time);
              }
#ifndef CONFIG_CXD56_AUDIO_ANALOG_NONE
            rtCode = AS_AcaControl(AS_ACA_SET_AMIC_BOOT_DONE, (uint32_t)NULL);
#else
            return E_AS_PATH_SEL_FROM_PARAM;
#endif
          }
        break;

      default:
        /* Do nothing. */
        break;
    }

  switch (pPathSelParam->pathTo)
    {
      case AS_PATH_TO_DMAC:
        if ((dataPathToDmacBit & (1 << pPathSelParam->pathFrom)) == 0)
          {
            if (getDmacId == NULL)
              {
                return E_AS_PATH_SEL_DMACID_NULL;
              }
            switch (pPathSelParam->pathFrom)
              {
                case AS_PATH_FROM_MIC1_8:
                  if ((pPathSelParam->mic_dma_channel > 0) &&
                      (pPathSelParam->mic_dma_channel <= 8))
                    {
                      bb_config_add_tbl.mic_dma_channel =
                                        pPathSelParam->mic_dma_channel;
                      *getDmacId = AS_DMAC_SEL_AC_IN;
                    }
                  else
                    {
                      return E_AS_PATH_SEL_MIC_DMA_CHANNEL_PARAM;
                    }
                  break;

                case AS_PATH_FROM_I2S1:
                  *getDmacId = AS_DMAC_SEL_I2S_IN;
                  break;

                case AS_PATH_FROM_I2S2:
                  *getDmacId = AS_DMAC_SEL_I2S2_IN;
                  break;

                default:
                  D_ASSERT(0);
                  break;
              }
            dataPathToDmacBit |= (1 << pPathSelParam->pathFrom);
          }
        else
          {
            return E_AS_PATH_SEL_USED_ERR;
          }
        break;

      case AS_PATH_TO_MIXER1:
      case AS_PATH_TO_MIXER2:
      case AS_PATH_TO_I2S1:
      case AS_PATH_TO_I2S2:
        if (pPathSelParam->pathTo == AS_PATH_TO_MIXER1)
          {
            pathToId = TO_SPI2S_MIXER1;
          }
        else if (pPathSelParam->pathTo == AS_PATH_TO_MIXER2)
          {
            pathToId = TO_SPI2S_MIXER2;
          }
        else if (pPathSelParam->pathTo == AS_PATH_TO_I2S1)
          {
            pathToId = TO_SPI2S_I2S1;
          }
        else
          {
            pathToId = TO_SPI2S_I2S2;
          }
        if (toSpI2s[pathToId] == pPathSelParam->pathFrom)
          {
            return E_AS_PATH_SEL_USED_ERR;
          }
        else if (toSpI2s[pathToId] == AS_PATH_FROM_NONE)
          {
            /* Do nothing. */
          }
        else
          {
            return E_AS_PATH_SEL_USED_ERR;
          }

        uint8_t pathFromId;
        switch (pPathSelParam->pathFrom)
          {
            case AS_PATH_FROM_I2S1:
            case AS_PATH_FROM_I2S2:
            case AS_PATH_FROM_MIXER:
              toSpI2s[pathToId] = pPathSelParam->pathFrom;
              setDataPathTo(pPathSelParam->pathFrom, pathToId);
              break;

            case AS_PATH_FROM_MIC12:
            case AS_PATH_FROM_MIC34:
            case AS_PATH_FROM_MIC56:
            case AS_PATH_FROM_MIC78:
              if (fromSel[FROM_SELECTOR1] == pPathSelParam->pathFrom)
                {
                  pathFromId = FROM_SELECTOR1;
                }
              else if (fromSel[FROM_SELECTOR2] == pPathSelParam->pathFrom)
                {
                  pathFromId = FROM_SELECTOR2;
                }
              else if (fromSel[FROM_SELECTOR1] == AS_PATH_FROM_NONE)
                {
                  fromSel[FROM_SELECTOR1] = pPathSelParam->pathFrom;
                  pathFromId = FROM_SELECTOR1;
                  setDataPathSel(pathFromId, pPathSelParam->pathFrom);
                }
              else if (fromSel[FROM_SELECTOR2] == AS_PATH_FROM_NONE)
                {
                  fromSel[FROM_SELECTOR2] = pPathSelParam->pathFrom;
                  pathFromId = FROM_SELECTOR2;
                  setDataPathSel(pathFromId, pPathSelParam->pathFrom);
                }
              else
                {
                  return E_AS_PATH_SEL_USED_ERR;
                }
              setDataPathToSel(pathFromId, pathToId);
              toSpI2s[pathToId] = pPathSelParam->pathFrom;
              break;

            case AS_PATH_FROM_DMAC:
              if (getDmacId == NULL)
                {
                  return E_AS_PATH_SEL_DMACID_NULL;
                }
              if (setDmacId == AS_DMAC_ID_NONE)
                {
                  if (fromSel[FROM_SELECTOR1] == AS_PATH_FROM_NONE)
                    {
                      fromSel[FROM_SELECTOR1] = AS_PATH_FROM_DMAC1;
                      dmacid = AS_PATH_FROM_DMAC1;
                      pathFromId = FROM_SELECTOR1;
                      setDataPathSel(pathFromId, pPathSelParam->pathFrom);
                      *getDmacId = AS_DMAC_SEL_I2S_OUT;
                    }
                  else if (fromSel[FROM_SELECTOR2] == AS_PATH_FROM_NONE)
                    {
                      fromSel[FROM_SELECTOR2] = AS_PATH_FROM_DMAC2;
                      dmacid = AS_PATH_FROM_DMAC2;
                      pathFromId = FROM_SELECTOR2;
                      setDataPathSel(pathFromId, pPathSelParam->pathFrom);
                      *getDmacId = AS_DMAC_SEL_I2S2_OUT;
                    }
                  else
                    {
                      return E_AS_PATH_SEL_USED_ERR;
                    }
                }
              else if (setDmacId == AS_DMAC_SEL_I2S_OUT)
                {
                  if (fromSel[FROM_SELECTOR1] == AS_PATH_FROM_DMAC1)
                    {
                      dmacid = AS_PATH_FROM_DMAC1;
                      pathFromId = FROM_SELECTOR1;
                      *getDmacId = AS_DMAC_SEL_I2S_OUT;
                    }
                 else
                   {
                     return E_AS_PATH_SEL_DMACID_ERR;
                   }
                }
              else if (setDmacId == AS_DMAC_SEL_I2S2_OUT)
                {
                  if (fromSel[FROM_SELECTOR2] == AS_PATH_FROM_DMAC2)
                    {
                      dmacid = AS_PATH_FROM_DMAC2;
                      pathFromId = FROM_SELECTOR2;
                      *getDmacId = AS_DMAC_SEL_I2S2_OUT;
                    }
                  else
                    {
                      return E_AS_PATH_SEL_DMACID_ERR;
                    }
                }
              else
                {
                  return E_AS_PATH_SEL_DMACID_PARAM;
                }
              setDataPathToSel(pathFromId, pathToId);
              toSpI2s[pathToId] = dmacid;
              break;

            default:
              D_ASSERT(0);
              break;
           }
         break;

       default:
         D_ASSERT(0);
         break;
    }
  return rtCode;
}

E_AS AS_ClearAudioDataPath(FAR asPathSelParam *pPathSelParam)
{
  E_AS rtCode = E_AS_OK;

  if (!chkPowerOnBaseBand())
    {
      return E_AS_CLR_DP_POWER_ON_CHK_ERR;
    }
  if (pPathSelParam == NULL)
    {
      return E_AS_PATH_SEL_NULL;
    }

  if (pPathSelParam->pathFrom >= AS_PATH_FROM_MAX_ENTRY)
    {
      return E_AS_PATH_SEL_FROM_PARAM;
    }
  if (pPathSelParam->pathTo >= AS_PATH_TO_MAX_ENTRY)
    {
      return E_AS_PATH_SEL_TO_PARAM;
    }

  rtCode = chkDataPath(pPathSelParam->pathFrom, pPathSelParam->pathTo);
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  uint8_t pathToId;
  switch (pPathSelParam->pathTo)
    {
      case AS_PATH_TO_DMAC:
        if ((dataPathToDmacBit & (1 << pPathSelParam->pathFrom)) == 0)
          {
            return E_AS_PATH_SEL_NOUSE_ERR;
          }
        else
          {
            dataPathToDmacBit &= ~(1 << pPathSelParam->pathFrom);
          }
        break;

      case AS_PATH_TO_MIXER1:
      case AS_PATH_TO_MIXER2:
      case AS_PATH_TO_I2S1:
      case AS_PATH_TO_I2S2:
        if (pPathSelParam->pathTo == AS_PATH_TO_MIXER1)
          {
            pathToId = TO_SPI2S_MIXER1;
          }
        else if (pPathSelParam->pathTo == AS_PATH_TO_MIXER2)
          {
            pathToId = TO_SPI2S_MIXER2;
          }
        else if (pPathSelParam->pathTo == AS_PATH_TO_I2S1)
          {
            pathToId = TO_SPI2S_I2S1;
          }
        else
          {
            pathToId = TO_SPI2S_I2S2;
          }
        asPathFromId dmacid = AS_PATH_FROM_NONE;
        if (pPathSelParam->pathFrom == AS_PATH_FROM_DMAC)
          {
            if (toSpI2s[pathToId] == AS_PATH_FROM_DMAC1)
              {
                dmacid = AS_PATH_FROM_DMAC1;
                toSpI2s[pathToId] = AS_PATH_FROM_NONE;
              }
            else if (toSpI2s[pathToId] == AS_PATH_FROM_DMAC2)
              {
                dmacid = AS_PATH_FROM_DMAC2;
                toSpI2s[pathToId] = AS_PATH_FROM_NONE;
              }
            else
              {
                return E_AS_PATH_SEL_NOUSE_ERR;
              }
          }
        else if (toSpI2s[pathToId] != pPathSelParam->pathFrom)
          {
            return E_AS_PATH_SEL_NOUSE_ERR;
          }
        else
          {
            dmacid = pPathSelParam->pathFrom;
            toSpI2s[pathToId] = AS_PATH_FROM_NONE;
          }
        uint8_t reset_flg = 1;
        for (uint8_t i = 0; i < TO_SPI2S_NUM; i++)
          {
            if (toSpI2s[i] == dmacid)
              {
                reset_flg = 0;
              }
          }
        if (reset_flg == 1)
          {
            for (uint8_t i = 0; i < FROM_SELECTOR_NUM; i++)
              {
                if (fromSel[i] == dmacid)
                  {
                    fromSel[i] = AS_PATH_FROM_NONE;
                  }
              }
          }
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS AS_ClearAudioDataPathAll(void)
{
  E_AS rtCode = E_AS_OK;
  uint8_t i = 0;
  if (!chkPowerOnBaseBand())
    {
      return E_AS_CLRALL_DP_POWER_ON_CHK_ERR;
    }

  for (i = 0; i < FROM_SELECTOR_NUM; i++)
    {
      fromSel[i] = AS_PATH_FROM_NONE;
    }
  for (i = 0; i < TO_SPI2S_NUM; i++)
    {
      toSpI2s[i] = AS_PATH_FROM_NONE;
    }
  dataPathToDmacBit = 0;
  initDataPathI2S();
  return rtCode;
}

void setDataPathTo(asPathFromId fromId, uint8_t toId)
{
  AC_REG_ID acRegId = RI_COD_INSEL2;
  uint8_t val = 0;

  switch (toId)
    {
      case TO_SPI2S_MIXER1:
        acRegId = RI_COD_INSEL2;
        break;

      case TO_SPI2S_MIXER2:
        acRegId = RI_COD_INSEL3;
        break;

      case TO_SPI2S_I2S1:
        acRegId = RI_SRC1IN_SEL;
        break;

      case TO_SPI2S_I2S2:
        acRegId = RI_SRC2IN_SEL;
        break;

      default:
        D_ASSERT(0);
        break;
    }

  switch (fromId)
    {
      case AS_PATH_FROM_I2S1:
        val = 0;
        break;

      case AS_PATH_FROM_I2S2:
        val = 1;
        break;

      case AS_PATH_FROM_MIXER:
        val = 3;
        break;

      default:
        D_ASSERT(0);
        break;
    }
  write_ac_reg(acRegId, val);
}

void setDataPathToSel(uint8_t fromId, uint8_t toId)
{
  AC_REG_ID acRegId = RI_COD_INSEL2;
  uint8_t val1 = 0;
  uint8_t val2 = 0;

  switch (toId)
    {
      case TO_SPI2S_MIXER1:
        acRegId = RI_COD_INSEL2;
        break;

      case TO_SPI2S_MIXER2:
        acRegId = RI_COD_INSEL3;
        break;

      case TO_SPI2S_I2S1:
        acRegId = RI_SRC1IN_SEL;
        break;

      case TO_SPI2S_I2S2:
        acRegId = RI_SRC2IN_SEL;
        break;

      default:
        D_ASSERT(0);
        break;
    }

  switch (fromId)
    {
      case FROM_SELECTOR1:
        val1 = 2;
        val2 = 0;
        break;

      case FROM_SELECTOR2:
        val1 = 3;
        val2 = 1;
        break;

      default:
        D_ASSERT(0);
        break;
    }
  if (toId < TO_SPI2S_I2S1)
    {
      write_ac_reg(acRegId, val1);
    }
  else
    {
      write_ac_reg(acRegId, val2);
    }
}

void setDataPathSel(uint8_t selId, asPathFromId fromId)
{
  AC_REG_ID acRegId = RI_AU_DAT_SEL1;
  uint8_t val = 0;

  switch (selId)
    {
      case FROM_SELECTOR1:
        acRegId = RI_AU_DAT_SEL1;
        break;

      case FROM_SELECTOR2:
        acRegId = RI_AU_DAT_SEL2;
        break;

      default:
        D_ASSERT(0);
        break;
    }

  switch (fromId)
    {
      case AS_PATH_FROM_DMAC:
        val = 4;
        break;

      case AS_PATH_FROM_MIC12:
        val = 0;
        break;

      case AS_PATH_FROM_MIC34:
        val = 1;
        break;

      case AS_PATH_FROM_MIC56:
        val = 2;
        break;

      case AS_PATH_FROM_MIC78:
        val = 3;
        break;

      default:
        D_ASSERT(0);
        break;
    }
  write_ac_reg(acRegId, val);
}

void initDataPathI2S(void)
{
  AC_REG_ID acRegId;
  uint8_t val;

  acRegId = RI_SRC1IN_SEL;
  val = 2;
  write_ac_reg(acRegId, val);

  acRegId = RI_SRC2IN_SEL;
  val = 2;
  write_ac_reg(acRegId, val);
}

asDmacMixerOutId GetDmacPathToMixerId(asDmacSelId dmacId)
{
  asDmacMixerOutId mixerOutId = AS_MIXER_OUT_PARAM;
  asPathFromId path_from;

  switch (dmacId)
    {
      case AS_DMAC_SEL_I2S_OUT:
        path_from = AS_PATH_FROM_DMAC1;
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        path_from = AS_PATH_FROM_DMAC2;
        break;

      default:
        return mixerOutId;
    }
  if (toSpI2s[TO_SPI2S_MIXER1] == path_from)
    {
      mixerOutId = AS_MIXER1_OUT;
    }
  else if (toSpI2s[TO_SPI2S_MIXER2] == path_from)
    {
      mixerOutId = AS_MIXER2_OUT;
    }
  else
    {
      mixerOutId = AS_MIXER_OUT_NONE;
    }
  return mixerOutId;
}

E_AS chkDataPath(asPathFromId fromId, asPathToId toId)
{
  if (toId == AS_PATH_TO_NONE)
    {
      return E_AS_PATH_SEL_TO_PARAM;
    }
  if (fromId == AS_PATH_FROM_NONE)
    {
      return E_AS_PATH_SEL_FROM_PARAM;
    }
  if ((dataPathCombiBit[fromId - 1] & (0x01 << (toId - 1))) == 0)
    {
      return E_AS_PATH_SEL_COMBINATION_NG;
    }
  return E_AS_OK;
}


