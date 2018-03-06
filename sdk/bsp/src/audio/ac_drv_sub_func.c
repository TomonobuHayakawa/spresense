/****************************************************************************
 * arch/arm/src/cxd56xx/audio/drivers/baseband/src/ac_drv_sub_func.c
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
/* Description: Audio Codec driver low-level API sub function */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <math.h>
#include <debug.h>

#include "audio/ac_drv_sub_func.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef M_PI
#  define M_PI 3.14159265358979323846
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

E_AS setSrcParam(asSrcSelId srcId, FAR asI2sParam *pI2sParam)
{
  E_AS rtCode = E_AS_OK;

  switch (srcId)
    {
      case AS_SRC_SEL_SRC1:
        rtCode = setI2sMode(srcId, pI2sParam->pSrc1);
        break;

      case AS_SRC_SEL_SRC2:
        rtCode = setI2sMode(srcId, pI2sParam->pSrc2);
        break;

      case AS_SRC_SEL_BOTH:
        rtCode = setI2sMode(AS_SRC_SEL_SRC1, pI2sParam->pSrc1);
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }
        rtCode = setI2sMode(AS_SRC_SEL_SRC2, pI2sParam->pSrc2);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS setI2sMode(asSrcSelId srcId, FAR asSrcParam *pSrc)
{
  E_AS rtCode = E_AS_OK;
  AC_REG_ID acRegSdMaster = RI_SD1MASTER;
  AC_REG_ID acRegDif      = RI_DIF1;
  AC_REG_ID acRegSrc      = RI_SRC1;
  AC_REG_ID acRegBypass   = RI_TEST_OUT_SEL0;

  switch (srcId)
    {
      case AS_SRC_SEL_SRC1:
        acRegSdMaster = RI_SD1MASTER;
        acRegDif = RI_DIF1;
        acRegSrc = RI_SRC1;
        acRegBypass = RI_TEST_OUT_SEL0;
        break;

      case AS_SRC_SEL_SRC2:
        acRegSdMaster = RI_SD2MASTER;
        acRegDif = RI_DIF2;
        acRegSrc = RI_SRC2;
        acRegBypass = RI_TEST_OUT_SEL1;
        break;

      case AS_SRC_SEL_BOTH:
        /* Not support here. */

      default:
        D_ASSERT(0);
        break;
    }

  /* SRC_OUT_CHANNEL */

  switch (pSrc->ch)
    {
      case AS_SRC_CHANNEL_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_SRC_CHANNEL_0CH:
        write_ac_reg(RI_BLF_EN, 0);
        break;

      case AS_SRC_CHANNEL_2CH:
        write_ac_reg(RI_BLF_EN, 1);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  /* I2S_MODE */

  switch (pSrc->i2sMode)
    {
      case AS_SRC_MODE_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_SRC_MODE_MASTER:
        write_ac_reg(acRegSdMaster, 1);
        break;

      case AS_SRC_MODE_SLAVE:
        write_ac_reg(acRegSdMaster, 0);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  /* SRC_FORMAT */

  switch (pSrc->fmt)
    {
      case AS_SRC_FORMAT_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_SRC_FORMAT_I2S:
        write_ac_reg(acRegDif, 0);
        break;

      case AS_SRC_FORMAT_LEFT:
        write_ac_reg(acRegDif, 1);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  rtCode = setI2sChSwap(srcId, pSrc);
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  /* SRC_FS */

  switch (pSrc->fs)
    {
      case AS_SRC_FS_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_SRC_FS_HIGH:
        write_ac_reg(acRegSrc, 3);
        break;

      case AS_SRC_FS_MID:
        write_ac_reg(acRegSrc, 2);
        break;

      case AS_SRC_FS_LOW:
        write_ac_reg(acRegSrc, 1);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  switch (pSrc->bypassEn)
    {
      case AS_I2S_BP_MODE_DISABLE:
        write_ac_reg(acRegBypass, 0);
        break;

      case AS_I2S_BP_MODE_ENABLE:
        write_ac_reg(acRegBypass, 1);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS setI2sChSwap(asSrcSelId srcId, FAR asSrcParam *pSrc)
{
  E_AS rtCode = E_AS_OK;
  AC_REG_ID acRegSwap = RI_LR_SWAP1;

  switch (srcId)
    {
      case AS_SRC_SEL_SRC1:
        acRegSwap = RI_LR_SWAP1;
        break;

      case AS_SRC_SEL_SRC2:
        acRegSwap = RI_LR_SWAP2;
        break;

      case AS_SRC_SEL_BOTH:
      default:
        /* Not support here. */
        D_ASSERT(0);
    }

  switch (pSrc->fmt)
    {
      case AS_SRC_FORMAT_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_SRC_FORMAT_I2S:
        write_ac_reg(acRegSwap, 0);
        break;

      case AS_SRC_FORMAT_LEFT:
        write_ac_reg(acRegSwap, 1);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS setSelCicParam(asCicSelId cicId, FAR asCicParam *pCicParam)
{
  E_AS rtCode = E_AS_OK;
  AC_REG_ID acRegInSel   = RI_CIC1IN_SEL;
  AC_REG_ID acRegAdcFs   = RI_ADC_FS;
  AC_REG_ID acRegHpfMode = RI_HPF1_MODE;

  switch (cicId)
    {
      case AS_CIC_SEL_CIC1:
        acRegInSel = RI_CIC1IN_SEL;
        acRegHpfMode = RI_HPF1_MODE;
        break;

      case AS_CIC_SEL_CIC2:
        acRegInSel = RI_CIC2IN_SEL;
        acRegHpfMode = RI_HPF2_MODE;
        break;

      case AS_CIC_SEL_CIC3:
        acRegInSel = RI_CIC3IN_SEL;
        acRegHpfMode = RI_HPF3_MODE;
        break;

      case AS_CIC_SEL_CIC4:
        acRegInSel = RI_CIC4IN_SEL;
        acRegHpfMode = RI_HPF4_MODE;
        break;

      default:
        D_ASSERT(0);
        break;
    }

  switch (pCicParam->inSel)
    {
      case AS_CIC_IN_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_CIC_IN_ACAPULCO_MIC:
        write_ac_reg(acRegInSel, 0);
        break;

      case AS_CIC_IN_DMIC:
        write_ac_reg(acRegInSel, 1);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  switch (pCicParam->adcFs)
    {
      case AS_CIC_ADC_FS_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_CIC_ADC_FS_64:
        write_ac_reg(acRegAdcFs, 0);
        break;

      case AS_CIC_ADC_FS_128:
        write_ac_reg(acRegAdcFs, 1);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  switch (pCicParam->hpfMode)
    {
      case AS_CIC_HPF_MODE_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_CIC_HPF_MODE_OFF:
        write_ac_reg(acRegHpfMode, 0);
        break;

      case AS_CIC_HPF_MODE_LOW:
        write_ac_reg(acRegHpfMode, 1);
        break;

      case AS_CIC_HPF_MODE_MID:
        write_ac_reg(acRegHpfMode, 2);
        break;

      case AS_CIC_HPF_MODE_HIGH:
        write_ac_reg(acRegHpfMode, 3);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  rtCode = setSelCicGain(cicId, pCicParam);

  return rtCode;
}

E_AS setSelCicGain(asCicSelId cicId, FAR asCicParam *pCicParam)
{
  E_AS rtCode = E_AS_OK;
  uint32_t val;
  uint8_t cicIdx = cicId - AS_CIC_SEL_CIC1;
  uint8_t chIdx = 0;
  const AC_REG_ID cicGainMode[AS_AC_CIC_MIC_CH_NUM] =
  {
    RI_CIC1_GAIN_MODE,
    RI_CIC2_GAIN_MODE
  };
  const AC_REG_ID cicGain[4][AS_AC_CIC_MIC_CH_NUM] =
  {
    { RI_CIC1_LGAIN, RI_CIC1_RGAIN},
    { RI_CIC2_LGAIN, RI_CIC2_RGAIN},
    { RI_CIC3_LGAIN, RI_CIC3_RGAIN},
    { RI_CIC4_LGAIN, RI_CIC4_RGAIN}
  };
  const AC_REG_ID matsubaraGain[2][AS_AC_CIC_MIC_CH_NUM] =
  {
    { RI_ADC1L_VOL, RI_ADC1R_VOL},
    { RI_ADC2L_VOL, RI_ADC2R_VOL},
  };

  /* GAIN_MODE */

  switch (pCicParam->gainMode)
    {
      case AS_CIC_GAIN_MODE_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_CIC_GAIN_MODE_MATSUBARA:
        switch (cicId)
          {
            case AS_CIC_SEL_CIC1:
            case AS_CIC_SEL_CIC2:
              write_ac_reg(cicGainMode[cicIdx], 0);

              for (chIdx = 0; chIdx < AS_AC_CIC_MIC_CH_NUM; chIdx++)
                {
                  if (pCicParam->gain[chIdx] == AS_MTBR_GAIN_HOLD)
                    {
                      /* Do nothing. */
                    }
                  else if (pCicParam->gain[chIdx] == AS_MICGAIN_MUTE)
                    {
                      write_ac_reg(matsubaraGain[cicIdx][chIdx], 0x01);
                    }
                  else if ((AS_MTBR_GAIN_MIN <= pCicParam->gain[chIdx]) &&
                           (pCicParam->gain[chIdx] <= AS_MTBR_GAIN_MAX))
                    {
                      val = (pCicParam->gain[chIdx] / 5) & 0x7f;
                      write_ac_reg(matsubaraGain[cicIdx][chIdx], val);
                    }
                  else
                    {
                      return E_AS_MICGAIN_PARAM;
                    }
                }
              break;

            case AS_CIC_SEL_CIC3:
            case AS_CIC_SEL_CIC4:
              /* Not support here. */

            default:
              D_ASSERT(0);
              break;
          }
        break;

      case AS_CIC_GAIN_MODE_CIC:
        for (chIdx = 0; chIdx < AS_AC_CIC_MIC_CH_NUM; chIdx++)
          {
            if (!CHECK_RANGE(pCicParam->gain[chIdx], AS_MICGAIN_HOLD,
                 AS_CIC_GAIN_MIN, AS_CIC_GAIN_MAX) &&
                (AS_MICGAIN_MUTE != pCicParam->gain[chIdx]))
              {
                return E_AS_MICGAIN_PARAM;
              }
          }

        switch (cicId)
          {
            case AS_CIC_SEL_CIC1:
            case AS_CIC_SEL_CIC2:
              write_ac_reg(cicGainMode[cicIdx], 1);
            case AS_CIC_SEL_CIC3:
            case AS_CIC_SEL_CIC4:
              for (chIdx = 0; chIdx < AS_AC_CIC_MIC_CH_NUM; chIdx++)
                {
                  if (pCicParam->gain[chIdx] == AS_MICGAIN_HOLD)
                    {
                      /* Do nothing. */
                    }
                  else if (pCicParam->gain[chIdx] == AS_MICGAIN_MUTE)
                    {
                      write_ac_reg(cicGain[cicIdx][chIdx], 0x4000);
                    }
                  else
                    {
                      val = (uint32_t)(pow(10.0f,
                                           ((float)pCicParam->gain[chIdx] /
                                            100.0f / 20.0f)) * 0x4000 +
                                            0x4000);
                      write_ac_reg(cicGain[cicIdx][chIdx], val);
                    }
                }
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

E_AS setDecimCommon(FAR asDecimParam *pDcmParam)
{
  E_AS rtCode = E_AS_OK;

  /* DECIM_MODE */

  switch (pDcmParam->mode)
    {
      case AS_DECIM_MODE_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_DECIM_MODE_THROUGH:
        write_ac_reg(RI_SEL_DECIM, 0);
        break;

      case AS_DECIM_MODE_DECIM:
        write_ac_reg(RI_SEL_DECIM, 1);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  /* DECIM_IN_FS */

  switch (pDcmParam->inFs)
    {
      case AS_DECIM_IN_FS_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_DECIM_IN_FS_4:
        write_ac_reg(RI_SEL_INF, 0);
        write_ac_reg(RI_DCMFS, 1);
        write_ac_reg(RI_DCMFS_34, 1);
        break;

      case AS_DECIM_IN_FS_8:
        write_ac_reg(RI_SEL_INF, 1);
        write_ac_reg(RI_DCMFS, 2);
        write_ac_reg(RI_DCMFS_34, 2);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  /* DECIM_OUT_FS */

  switch (pDcmParam->outFs)
    {
      case AS_DECIM_OUT_FS_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_DECIM_OUT_FS_1:
        write_ac_reg(RI_SEL_OUTF, 0);
        break;

      case AS_DECIM_OUT_FS_2:
        write_ac_reg(RI_SEL_OUTF, 1);
        break;

      case AS_DECIM_OUT_FS_4:
        write_ac_reg(RI_SEL_OUTF, 2);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS setDecimOut(asDecimSelId dcmId, FAR asDecimParam *pDcmParam)
{
  E_AS rtCode = E_AS_OK;

  /* DECIM_SEL */

  if ((dcmId == AS_DECIM_SEL_DECIM0) || (dcmId == AS_DECIM_SEL_BOTH))
    {
      D_ASSERT(pDcmParam->outEn0 <= 0x0f);
      write_ac_reg(RI_OUTEN_MIC2R_A, ((pDcmParam->outEn0 >> 3) & 0x01));
      write_ac_reg(RI_OUTEN_MIC2L_A, ((pDcmParam->outEn0 >> 2) & 0x01));
      write_ac_reg(RI_OUTEN_MIC1R_A, ((pDcmParam->outEn0 >> 1) & 0x01));
      write_ac_reg(RI_OUTEN_MIC1L_A, ((pDcmParam->outEn0 >> 0) & 0x01));
    }
  if ((dcmId == AS_DECIM_SEL_DECIM1) || (dcmId == AS_DECIM_SEL_BOTH))
    {
      D_ASSERT(pDcmParam->outEn1 <= 0x0f);
      write_ac_reg(RI_OUTEN_MIC2R_B, ((pDcmParam->outEn1 >> 3) & 0x01));
      write_ac_reg(RI_OUTEN_MIC2L_B, ((pDcmParam->outEn1 >> 2) & 0x01));
      write_ac_reg(RI_OUTEN_MIC1R_B, ((pDcmParam->outEn1 >> 1) & 0x01));
      write_ac_reg(RI_OUTEN_MIC1L_B, ((pDcmParam->outEn1 >> 0) & 0x01));
    }

  return rtCode;
}

uint32_t calc_time_const(AsClkModeId clkMode, uint32_t tc)
{
  uint32_t fs = ((clkMode == AS_CLK_MODE_HIRES) ? 192000 / 4 : 48000 / 1);
  uint32_t val = ((tc == 0) ? (0x400000) :
                  ((1.0 - exp(-1.0 * 1000000 / tc / fs)) * 4194304));
  return val;
}

uint32_t calc_alc_lpf(AsClkModeId clkMode, uint32_t fc)
{
  uint32_t fs;
  uint32_t val;

  if (fc == AS_ALC_LPF_THROUGH)
    {
      val = 0x400000;
    }
  else
    {
      fs = ((clkMode == AS_CLK_MODE_HIRES) ? 192000 : 48000);
      val = (1.0 - exp(-2.0 * M_PI * fc / 1000 / fs)) * 4194304;
    }
  return val;
}

E_AS setAlcParam()
{
  E_AS rtCode = E_AS_OK;
  uint32_t val;

  asAlcParam alcParam;
  alcParam.clkMode    = (AsClkModeId)bb_config_tblp->clk_mode;
  alcParam.alcRec     = AS_ALC_MODE_PLAY;
  alcParam.alcTarget  = bb_config_tblp->alc_target;
  alcParam.alcKnee    = bb_config_tblp->alc_knee;
  alcParam.alcAlg     = AS_ALC_ALG_UNKNOWN;
  alcParam.alcDelay   = AS_ALC_DELAY_HOLD;
  alcParam.alcLpf     = AS_ALC_LPF_HOLD;
  alcParam.alcAttack  = AS_ALCSPC_ATTACK_HOLD;
  alcParam.alcRelease = AS_ALCSPC_RELEASE_HOLD;

  switch (alcParam.alcRec)
    {
      case AS_ALC_MODE_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_ALC_MODE_PLAY:
        write_ac_reg(RI_ALC_REC, 0);
        break;

      case AS_ALC_MODE_RECORD:
        write_ac_reg(RI_ALC_REC, 1);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  if (!CHECK_RANGE(alcParam.alcTarget,
                   AS_ALC_TARGET_HOLD,
                   AS_ALC_TARGET_MIN,
                   AS_ALC_TARGET_MAX))
    {
      return E_AS_ALC_TARGET_PARAM;
    }
  if (alcParam.alcTarget == AS_ALC_TARGET_HOLD)
    {
      /* Do nothing. */
    }
  else
    {
      write_ac_reg(RI_ALCTARGET, -(alcParam.alcTarget));
    }

  if (!CHECK_RANGE(alcParam.alcKnee,
                   AS_ALC_KNEE_HOLD,
                   AS_ALC_KNEE_MIN,
                   AS_ALC_KNEE_MAX))
    {
      return E_AS_ALC_KNEE_PARAM;
    }
  if (alcParam.alcKnee == AS_ALC_KNEE_HOLD)
    {
      /* Do nothing. */
    }
  else
    {
      val = -(alcParam.alcKnee) / 5;
      write_ac_reg(RI_ALC_KNEE, val);
    }

  switch (alcParam.alcAlg)
    {
      case AS_ALC_ALG_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_ALC_ALG_NOT_USE_INSTANT:
        write_ac_reg(RI_ALC_ALG, 0);
        break;

      case AS_ALC_ALG_USE_INSTANT:
        write_ac_reg(RI_ALC_ALG, 1);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  D_ASSERT(CHECK_RANGE(alcParam.alcDelay,
                       AS_ALC_DELAY_HOLD,
                       AS_ALC_DELAY_MIN,
                       AS_ALC_DELAY_MAX));
  if (alcParam.alcDelay == AS_ALC_DELAY_HOLD)
    {
      /* Do nothing. */
    }
  else
    {
      write_ac_reg(RI_ALC_DELAY, alcParam.alcDelay);
    }

  if (alcParam.clkMode < AS_CLK_MODE_NUM)
    {
      if (alcParam.clkMode == AS_CLK_MODE_HIRES)
        {
          D_ASSERT(CHECK_RANGE(alcParam.alcLpf,
                               AS_ALC_LPF_HOLD,
                               AS_ALC_LPF_HIRES_MIN,
                               AS_ALC_LPF_HIRES_MAX));
        }
      else
        {
          D_ASSERT(CHECK_RANGE(alcParam.alcLpf,
                               AS_ALC_LPF_HOLD,
                               AS_ALC_LPF_NORMAL_MIN,
                               AS_ALC_LPF_NORMAL_MAX));
        }
      D_ASSERT(alcParam.alcLpf == AS_ALC_LPF_THROUGH);

      if (alcParam.alcLpf == AS_ALC_LPF_HOLD)
        {
          /* Do nothing. */
        }
      else
        {
          val = calc_alc_lpf(alcParam.clkMode, alcParam.alcLpf);
          write_ac_reg(RI_ALC_LPF, val);
        }

      D_ASSERT(CHECK_RANGE(alcParam.alcAttack,
                           AS_ALCSPC_ATTACK_HOLD,
                           AS_ALCSPC_ATTACK_MIN,
                           AS_ALCSPC_ATTACK_MAX));
      if (alcParam.alcAttack == AS_ALCSPC_ATTACK_HOLD)
        {
          /* Do nothing. */
        }
      else
        {
          val = calc_time_const(alcParam.clkMode, alcParam.alcAttack);
          write_ac_reg(RI_SPC_ALC_ATTACK, val);
        }

      D_ASSERT(CHECK_RANGE(alcParam.alcRelease,
                           AS_ALCSPC_RELEASE_HOLD,
                           AS_ALCSPC_RELEASE_MIN,
                           AS_ALCSPC_RELEASE_MAX));
      if (alcParam.alcRelease == AS_ALCSPC_RELEASE_HOLD)
        {
          /* Do nothing. */
        }
      else
        {
          val = calc_time_const(alcParam.clkMode, alcParam.alcRelease);
          write_ac_reg(RI_SPC_ALC_RELEASE, val);
        }
    }
  else
    {
      return E_AS_CLK_MODE_PARAM;
    }

  return rtCode;
}

E_AS setSpcParam()
{
  E_AS rtCode = E_AS_OK;
  uint32_t val;

  asSpcParam spcParam;
  spcParam.clkMode    = (AsClkModeId)bb_config_tblp->clk_mode;
  spcParam.spcAweight = AS_SPC_FILTER_UNKNOWN;
  spcParam.spcLimit   = bb_config_tblp->spc_limit;
  spcParam.spcAttack  = AS_ALCSPC_ATTACK_HOLD;
  spcParam.spcRelease = AS_ALCSPC_RELEASE_HOLD;

  switch (spcParam.spcAweight)
    {
      case AS_SPC_FILTER_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_SPC_FILTER_THROUGH:
        write_ac_reg(RI_SPC_AWEIGHT, 0);
        break;

      case AS_SPC_FILTER_AWEIGHT:
        write_ac_reg(RI_SPC_AWEIGHT, 1);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  if (!CHECK_RANGE(spcParam.spcLimit,
                   AS_SPC_LIMIT_HOLD,
                   AS_SPC_LIMIT_MIN,
                   AS_SPC_LIMIT_MAX))
    {
      return E_AS_SPC_LIMIT_PARAM;
    }
  if (spcParam.spcLimit == AS_SPC_LIMIT_HOLD)
    {
      /* Do nothing. */
    }
  else
    {
      val = -(spcParam.spcLimit) / 5;
      write_ac_reg(RI_SPC_LIMIT, val);
    }

  if (spcParam.clkMode < AS_CLK_MODE_NUM)
    {
      D_ASSERT(CHECK_RANGE(spcParam.spcAttack,
                           AS_ALCSPC_ATTACK_HOLD,
                           AS_ALCSPC_ATTACK_MIN,
                           AS_ALCSPC_ATTACK_MAX));
      if (spcParam.spcAttack == AS_ALCSPC_ATTACK_HOLD)
        {
          /* Do nothing. */
        }
      else
        {
          val = calc_time_const(spcParam.clkMode, spcParam.spcAttack);
          write_ac_reg(RI_SPC_ALC_ATTACK, val);
        }

    D_ASSERT(CHECK_RANGE(spcParam.spcRelease,
                         AS_ALCSPC_RELEASE_HOLD,
                         AS_ALCSPC_RELEASE_MIN,
                         AS_ALCSPC_RELEASE_MAX));
    if (spcParam.spcRelease == AS_ALCSPC_RELEASE_HOLD)
      {
        /* Do nothing. */
      }
    else
      {
        val = calc_time_const(spcParam.clkMode, spcParam.spcRelease);
        write_ac_reg(RI_SPC_ALC_RELEASE, val);
      }
    }
  else
    {
      return E_AS_CLK_MODE_PARAM;
    }

  return rtCode;
}

E_AS setDeqCoef(AC_REG_ID acRegId, FAR const uint32_t *pCoef, uint32_t len)
{
  E_AS rtCode = E_AS_OK;

  D_ASSERT(pCoef != NULL);

  for (uint32_t i = 0; i < len; i++)
    {
      write32_ac_reg(acRegMap[acRegId].addr + (i * 4), *(pCoef + i));
    }

  return rtCode;
}

E_AS setDncRam(uint32_t offset, FAR const uint32_t *pData, uint32_t len)
{
  E_AS rtCode = E_AS_OK;

  D_ASSERT(pData != NULL);

  for (uint32_t i = 0; i < len; i++)
    {
      write32_ac_reg(offset+(i * 4), *(pData + i));
    }

  return rtCode;
}

E_AS setAcSmstrParam(asSmstrModeId mode)
{
  E_AS rtCode = E_AS_OK;

  /* NSDD */

  write_ac_reg(RI_NSDD, 0x07fb5);

  /* NSX2 */

  switch (mode)
    {
      case AS_SMSTR_MODE_FS_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_SMSTR_MODE_FS_16:
        write_ac_reg(RI_NSX2, 0);
        break;

      case AS_SMSTR_MODE_FS_32:
        write_ac_reg(RI_NSX2, 1);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS setAcSerDesParam(FAR asSerDesParam *pSdesParam)
{
  E_AS rtCode = E_AS_OK;

  switch (pSdesParam->serFs)
    {
      case AS_ACA_SER_FS_UNKNOWN:
        D_ASSERT(pSdesParam->serMode == AS_ACA_SER_MODE_UNKNOWN);
        /* Do nothing. */
        break;

      case AS_ACA_SER_FS_128:
        D_ASSERT(pSdesParam->serMode == AS_ACA_SER_MODE_4CH);
        write_ac_reg(RI_FS_FS, 0);
        write_ac_reg(RI_SER_MODE, 1);
        break;

      case AS_ACA_SER_FS_64:
        D_ASSERT(pSdesParam->serMode == AS_ACA_SER_MODE_8CH);
        write_ac_reg(RI_FS_FS, 1);
        write_ac_reg(RI_SER_MODE, 0);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  rtCode = setAcDesOut(pSdesParam);

  return rtCode;
}

E_AS setAcDesOut(FAR asSerDesParam *pSdesParam)
{
  E_AS rtCode = E_AS_OK;
  AC_REG_ID acRegSelOut;
  uint32_t  i;

  for (i = 0; i < 8; i++)
    {
      switch (i)
        {
          case 0:
            acRegSelOut = RI_SEL_OUT1_L;
            break;

          case 1:
            acRegSelOut = RI_SEL_OUT1_R;
            break;

          case 2:
            acRegSelOut = RI_SEL_OUT2_L;
            break;

          case 3:
            acRegSelOut = RI_SEL_OUT2_R;
            break;

          case 4:
            acRegSelOut = RI_SEL_OUT3_L;
            break;

          case 5:
            acRegSelOut = RI_SEL_OUT3_R;
            break;

          case 6:
            acRegSelOut = RI_SEL_OUT4_L;
            break;

          case 7:
            acRegSelOut = RI_SEL_OUT4_R;
            break;

          default:
            acRegSelOut = RI_SEL_OUT1_L;
            break;
        }

    switch (pSdesParam->selCh.out[i])
      {
        case AS_SDES_DES_SEL_UNKNOWN:
          /* Do nothing. */
          break;

        case AS_SDES_DES_SEL_CH1:
          write_ac_reg(acRegSelOut, 0);
          break;

        case AS_SDES_DES_SEL_CH2:
          write_ac_reg(acRegSelOut, 1);
          break;

        case AS_SDES_DES_SEL_CH3:
          write_ac_reg(acRegSelOut, 2);
          break;

        case AS_SDES_DES_SEL_CH4:
          write_ac_reg(acRegSelOut, 3);
          break;

        case AS_SDES_DES_SEL_CH5:
          write_ac_reg(acRegSelOut, 4);
          break;

        case AS_SDES_DES_SEL_CH6:
          write_ac_reg(acRegSelOut, 5);
          break;

        case AS_SDES_DES_SEL_CH7:
          write_ac_reg(acRegSelOut, 6);
          break;

        case AS_SDES_DES_SEL_CH8:
          write_ac_reg(acRegSelOut, 7);
          break;

        default:
          D_ASSERT(0);
          break;
      }
    }

  return rtCode;
}

void checkErrCode(E_AS rtCode, E_AS okCode)
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

