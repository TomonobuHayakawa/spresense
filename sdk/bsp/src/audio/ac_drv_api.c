/*****************************************************************************
 * arch/arm/src/cxd56xx/audio/drivers/baseband/src/ac_drv_api.c
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
 ****************************************************************************/
/* Description: Audio Codec driver low-level API */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <unistd.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <debug.h>

#include "audio/ac_drv_sub_func.h"
#include "audio/ac_drv_path.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define audioDelayTask(ms) usleep((ms) * 1000)

#define VOL_OFFSET 190

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

E_AS asAc_CheckID(void)
{
  if (read_ac_reg(RI_REVID) != AC_REVID)
    {
      return E_AS_AC_ID_NG;
    }
  if (read_ac_reg(RI_DEVICEID) != AC_DEVICEID)
    {
      return E_AS_AC_ID_NG;
    }

  return E_AS_OK;
}

E_AS asAc_ResetDsp(void)
{
  write_ac_reg(RI_S_RESET, 1);
  write_ac_reg(RI_S_RESET, 0);

  audioDelayTask(20);  /* Wait 20ms. */

  return E_AS_OK;
}

E_AS asAc_InitDsp(void)
{
  write_ac_reg(RI_PDN_DSPB,  0);
  write_ac_reg(RI_PDN_DSPS2, 0);
  write_ac_reg(RI_PDN_DSPS1, 0);
  write_ac_reg(RI_PDN_DSPC,  0);

  write_ac_reg(RI_DSPRAM4_CLR, 1);
  write_ac_reg(RI_DSPRAM2_CLR, 1);
  write_ac_reg(RI_DSPRAM1_CLR, 1);

  audioDelayTask(1);  /* 512 cycle @24.576MHz */

  write_ac_reg(RI_DSPRAM4_CLR, 0);
  write_ac_reg(RI_DSPRAM2_CLR, 0);
  write_ac_reg(RI_DSPRAM1_CLR, 0);

  write_ac_reg(RI_PDN_DSPB,  1);
  write_ac_reg(RI_PDN_DSPS2, 1);
  write_ac_reg(RI_PDN_DSPS1, 1);
  write_ac_reg(RI_PDN_DSPC,  1);

  write_ac_reg(RI_S_RESET, 1);
  write_ac_reg(RI_S_RESET, 0);

  up_enable_irq(AUDCODEC_IRQn);

  return E_AS_OK;
}

E_AS asAc_PowerOnSrc(asSrcSelId srcId, FAR asI2sParam *pI2sParam)
{
  E_AS rtCode = E_AS_OK;

  D_ASSERT(pI2sParam != NULL);
  D_ASSERT((pI2sParam->pSrc1 != NULL) && (pI2sParam->pSrc2 != NULL));

  /* Power on SRC. */

  if ((srcId == AS_SRC_SEL_SRC1) || (srcId == AS_SRC_SEL_BOTH))
    {
      write_ac_reg(RI_PDN_DSPS1, 0);
    }
  if ((srcId == AS_SRC_SEL_SRC2) || (srcId == AS_SRC_SEL_BOTH))
    {
      write_ac_reg(RI_PDN_DSPS2, 0);
    }
  D_ASSERT(srcId < AS_SRC_SEL_MAX_ENTRY);

  asBca_SetSrcParam();

  /* Power on BLF. */

  switch (srcId)
    {
      case AS_SRC_SEL_SRC1:
        if (pI2sParam->pSrc1->ch == AS_SRC_CHANNEL_2CH)
          {
            write_ac_reg(RI_PDN_DSPB, 0);
          }
        break;

      case AS_SRC_SEL_SRC2:
        if (pI2sParam->pSrc2->ch == AS_SRC_CHANNEL_2CH)
          {
            write_ac_reg(RI_PDN_DSPB, 0);
          }
        break;

      case AS_SRC_SEL_BOTH:
        if ((pI2sParam->pSrc1->ch == AS_SRC_CHANNEL_2CH) ||
            (pI2sParam->pSrc2->ch == AS_SRC_CHANNEL_2CH))
          {
            write_ac_reg(RI_PDN_DSPB, 0);
          }
        break;

      default:
        D_ASSERT(0);
        break;
    }

  rtCode = asAc_SetSrcParam(srcId, pI2sParam);

  return rtCode;
}

E_AS asAc_SetSrcParam(asSrcSelId srcId, FAR asI2sParam *pI2sParam)
{
  E_AS rtCode = E_AS_OK;

  D_ASSERT(pI2sParam != NULL);
  D_ASSERT((pI2sParam->pSrc1 != NULL) && (pI2sParam->pSrc2 != NULL));

  /* SRC_RES */

  switch (pI2sParam->resMode)
    {
      case AS_SRC_RES_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_SRC_RES_HIGH:
        write_ac_reg(RI_HI_RES_MODE, 1);
        break;

      case AS_SRC_RES_NORMAL:
        write_ac_reg(RI_HI_RES_MODE, 0);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  /* Set SRC param. */

  rtCode = setSrcParam(srcId, pI2sParam);

  write_ac_reg(RI_ARWPHSET, 0);
  write_ac_reg(RI_HALT_INHIBIT, 0);

  return rtCode;
}

E_AS asAc_EnableSrc(asSrcSelId srcId)
{
  E_AS rtCode = E_AS_OK;

  switch (srcId)
    {
      case AS_SRC_SEL_SRC1:
        write_ac_reg(RI_SDIN1_EN, 1);
        write_ac_reg(RI_SDOUT1_EN, 1);
        if (read_ac_reg(RI_SD1MASTER) == 1)
          {
            if (read_ac_reg(RI_SDCK_OUTENX) == 1)
              {
                write_ac_reg(RI_SDCK_OUTENX, 0);
              }
            else
              {
                /* Do nothing. */
              }
          }
        else
          {
            write_ac_reg(RI_SDCK_OUTENX, 1);
          }
        break;

      case AS_SRC_SEL_SRC2:
        write_ac_reg(RI_SDIN2_EN, 1);
        write_ac_reg(RI_SDOUT2_EN, 1);
        if (read_ac_reg(RI_SD2MASTER) == 1)
          {
            if (read_ac_reg(RI_SDCK_OUTENX) == 1)
              {
                write_ac_reg(RI_SDCK_OUTENX, 0);
              }
            else
              {
                /* Do nothing. */
              }
          }
        else
          {
            write_ac_reg(RI_SDCK_OUTENX, 1);
          }
        break;

      case AS_SRC_SEL_BOTH:
        write_ac_reg(RI_SDIN1_EN, 1);
        write_ac_reg(RI_SDIN2_EN, 1);
        write_ac_reg(RI_SDOUT1_EN, 1);
        write_ac_reg(RI_SDOUT2_EN, 1);
        if ((read_ac_reg(RI_SD1MASTER) == 1) ||
            (read_ac_reg(RI_SD2MASTER) == 1))
          {
            if (read_ac_reg(RI_SDCK_OUTENX) == 1)
              {
                write_ac_reg(RI_SDCK_OUTENX, 0);
              }
            else
              {
                /* Do nothing. */
              }
          }
        else
          {
            write_ac_reg(RI_SDCK_OUTENX, 1);
          }
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS asAc_PowerOnCic(asCicSelId cicId, FAR asCicParam *pCicParam)
{
  E_AS rtCode = E_AS_OK;

  D_ASSERT(pCicParam != NULL);

  /* Power on CIC. */

  switch (pCicParam->inSel)
    {
      case AS_CIC_IN_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_CIC_IN_ACAPULCO_MIC:
        if ((cicId == AS_CIC_SEL_CIC1) || (cicId == AS_CIC_SEL_CIC12) ||
            (cicId == AS_CIC_SEL_CIC_ALL))
          {
            write_ac_reg(RI_PDN_AMIC1, 0);
          }
        if ((cicId == AS_CIC_SEL_CIC2) || (cicId == AS_CIC_SEL_CIC12) ||
            (cicId == AS_CIC_SEL_CIC_ALL))
          {
            write_ac_reg(RI_PDN_AMIC2, 0);
          }
        if ((cicId == AS_CIC_SEL_CIC34) || (cicId == AS_CIC_SEL_CIC_ALL))
          {
            write_ac_reg(RI_PDN_AMICEXT, 0);
          }
        if ((cicId == AS_CIC_SEL_CIC3) || (cicId == AS_CIC_SEL_CIC4))
          {
            if (read_ac_reg(RI_PDN_AMICEXT) == 1)
              {
                write_ac_reg(RI_PDN_AMICEXT, 0);
              }
            else
              {
                /* Do nothing. */
              }
          }
        D_ASSERT(cicId < AS_CIC_SEL_MAX_ENTRY);
        break;

    case AS_CIC_IN_DMIC:
      if (read_ac_reg(RI_PDN_DMIC) == 1)
        {
          write_ac_reg(RI_PDN_DMIC, 0);
        }
      else
        {
          /* Do nothing. */
        }
      break;

    default:
      D_ASSERT(0);
      break;
    }

  rtCode = asAc_SetCicParam(cicId, pCicParam);

  return rtCode;
}

E_AS asAc_SetCicParam(asCicSelId cicId, FAR asCicParam *pCicParam)
{
  E_AS rtCode = E_AS_OK;

  D_ASSERT(pCicParam != NULL);

  /* Set CIC param. */

  switch (cicId)
    {
      case AS_CIC_SEL_CIC1:
        rtCode = setSelCicParam(AS_CIC_SEL_CIC1, pCicParam);
        break;

      case AS_CIC_SEL_CIC2:
        rtCode = setSelCicParam(AS_CIC_SEL_CIC2, pCicParam);
        break;

      case AS_CIC_SEL_CIC3:
        rtCode = setSelCicParam(AS_CIC_SEL_CIC3, pCicParam);
        break;

      case AS_CIC_SEL_CIC4:
        rtCode = setSelCicParam(AS_CIC_SEL_CIC4, pCicParam);
        break;

      case AS_CIC_SEL_CIC12:
        rtCode = setSelCicParam(AS_CIC_SEL_CIC1, pCicParam);
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }
        rtCode = setSelCicParam(AS_CIC_SEL_CIC2, pCicParam);
        break;

      case AS_CIC_SEL_CIC34:
        rtCode = setSelCicParam(AS_CIC_SEL_CIC3, pCicParam);
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }
        rtCode = setSelCicParam(AS_CIC_SEL_CIC4, pCicParam);
        break;

      case AS_CIC_SEL_CIC_ALL:
        rtCode = setSelCicParam(AS_CIC_SEL_CIC1, pCicParam);
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }

        rtCode = setSelCicParam(AS_CIC_SEL_CIC2, pCicParam);
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }

        rtCode = setSelCicParam(AS_CIC_SEL_CIC3, pCicParam);
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }

        rtCode = setSelCicParam(AS_CIC_SEL_CIC4, pCicParam);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS asAc_PowerOnDecim(void)
{
  E_AS rtCode = E_AS_OK;

  asDecimSelId dcmId = AS_DECIM_SEL_BOTH;

  /* Power on DECIM. */

  switch (dcmId)
    {
      case AS_DECIM_SEL_DECIM0:
      case AS_DECIM_SEL_DECIM1:
      case AS_DECIM_SEL_BOTH:
        write_ac_reg(RI_DECIM0_EN, 1);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  write_ac_reg(RI_MCK_AHBMSTR_EN, 1);

  rtCode = asAc_SetDecimParam(dcmId);

  return rtCode;
}

E_AS asAc_SetDecimParam(asDecimSelId dcmId)
{
  E_AS rtCode = E_AS_OK;

  asDecimParam dcmParam;
  rtCode = GetDcmParam(&dcmParam);
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  /* Set DECIM param. */

  rtCode = setDecimCommon(&dcmParam);
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  /* Set DECIM output. */

  rtCode = setDecimOut(dcmId, &dcmParam);

  return rtCode;
}

E_AS asAc_PowerOnCodec(void)
{
  E_AS rtCode = E_AS_OK;

  write_ac_reg(RI_PDN_DSPC, 0);

  write_ac_reg(RI_DSR_RATE, 1);
  write_ac_reg(RI_DIGSFT, 1);

  return rtCode;
}

E_AS asAc_SetDigSft(uint32_t digsft)
{
  E_AS rtCode = E_AS_OK;

  write_ac_reg(RI_DIGSFT, digsft);

  return rtCode;
}

E_AS asAc_SetDsrRate(uint32_t rate)
{
  E_AS rtCode = E_AS_OK;

  write_ac_reg(RI_DSR_RATE, rate);

  return rtCode;
}

E_AS asAc_SetAlcSpcParam(void)
{
  E_AS rtCode = E_AS_OK;

  write_ac_reg(RI_ALC_EN, 0);
  write_ac_reg(RI_SPC_EN, 0);

  switch (bb_config_tblp->alc_spc_sel)
    {
      case AS_ALC_SPC_SEL_ALC:
        rtCode = setAlcParam();
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }
        write_ac_reg(RI_ALC_EN, 1);
        break;

      case AS_ALC_SPC_SEL_SPC:
        rtCode = setSpcParam();
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }
        write_ac_reg(RI_SPC_EN, 1);
        break;

      case AS_ALC_SPC_SEL_OFF:
        /* Do nothing. */
        break;

      default:
        return E_AS_ALC_SPC_SEL_PARAM;
    }

  return rtCode;
}

E_AS asAc_SetClearStereo(asCsSignId csSign, int32_t csVol)
{
  E_AS rtCode = E_AS_OK;

  switch (csSign)
    {
      case AS_CS_SIGN_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_CS_SIGN_POSITIVE:
        write_ac_reg(RI_CS_SIGN, 0);
        break;

      case AS_CS_SIGN_NEGATIVE:
        write_ac_reg(RI_CS_SIGN, 1);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  if (csVol == AS_CS_VOL_INF_ZERO)
    {
      write_ac_reg(RI_CS_VOL, 0x00);
    }
  else if ((AS_CS_VOL_MIN <= csVol) && (csVol <= AS_CS_VOL_MAX))
    {
      uint32_t val = ((csVol + VOL_OFFSET) / 5) & 0x7f;
      write_ac_reg(RI_CS_VOL, val);
    }
  else
    {
      return E_AS_CS_VOL_PARAM;
    }

  return rtCode;
}

E_AS asAc_SetDeqCoef(bool deqEn, FAR asDeqParam *pDeqParam)
{
  E_AS rtCode = E_AS_OK;

  write_ac_reg(RI_DEQ_EN, 0);

  if (pDeqParam != NULL)
    {
      rtCode = setDeqCoef(RI_DEQ_COEF_1B0, pDeqParam->pCoefBand1,
                          DEQ_COEF_NUM);
      if (rtCode != E_AS_OK)
        {
          return rtCode;
        }

      rtCode = setDeqCoef(RI_DEQ_COEF_2B0, pDeqParam->pCoefBand2,
                          DEQ_COEF_NUM);
      if (rtCode != E_AS_OK)
        {
          return rtCode;
        }

      rtCode = setDeqCoef(RI_DEQ_COEF_3B0, pDeqParam->pCoefBand3,
                          DEQ_COEF_NUM);
      if (rtCode != E_AS_OK)
        {
          return rtCode;
        }

      rtCode = setDeqCoef(RI_DEQ_COEF_4B0, pDeqParam->pCoefBand4,
                          DEQ_COEF_NUM);
      if (rtCode != E_AS_OK)
        {
          return rtCode;
        }

      rtCode = setDeqCoef(RI_DEQ_COEF_5B0, pDeqParam->pCoefBand5,
                          DEQ_COEF_NUM);
      if (rtCode != E_AS_OK)
        {
          return rtCode;
        }

      rtCode = setDeqCoef(RI_DEQ_COEF_6B0, pDeqParam->pCoefBand6,
                          DEQ_COEF_NUM);
      if (rtCode != E_AS_OK)
        {
          return rtCode;
        }
    }

  if (deqEn == true)
    {
      write_ac_reg(RI_DEQ_EN, 1);
      audioDelayTask(1);
    }

  return rtCode;
}

E_AS asAc_PowerOnDnc(asDncSelId dncId)
{
  E_AS rtCode = E_AS_OK;

  switch (dncId)
    {
      case AS_DNC_SEL_DNC1:
        write_ac_reg(RI_PDN_DNC1, 0);
        write_ac_reg(RI_PDN_ANC, 0);
        break;

      case AS_DNC_SEL_DNC2:
        write_ac_reg(RI_PDN_DNC2, 0);
        write_ac_reg(RI_PDN_ANC, 0);
        break;

      case AS_DNC_SEL_BOTH:
        write_ac_reg(RI_PDN_DNC1, 0);
        write_ac_reg(RI_PDN_DNC2, 0);
        write_ac_reg(RI_PDN_ANC, 0);
        break;

      default:
        return E_AS_DNC_SEL_PARAM;
    }
  rtCode = asBca_SetDncParam(dncId);

  return rtCode;
}

E_AS asAc_SetDncParam(bool dncEn, asDncSelId dncId,
                      FAR asDncParam *pDncParam1,
                      FAR asDncParam *pDncParam2)
{
  E_AS rtCode = E_AS_OK;

  switch (dncId)
    {
      case AS_DNC_SEL_DNC1:
        write_ac_reg(RI_DNC1_MUTE, 1);
        write_ac_reg(RI_DNC1_START, 0);
        break;

      case AS_DNC_SEL_DNC2:
        write_ac_reg(RI_DNC2_MUTE, 1);
        write_ac_reg(RI_DNC2_START, 0);
        break;

      case AS_DNC_SEL_BOTH:
        write_ac_reg(RI_DNC1_MUTE, 1);
        write_ac_reg(RI_DNC2_MUTE, 1);
        write_ac_reg(RI_DNC1_START, 0);
        write_ac_reg(RI_DNC2_START, 0);
        break;

      default:
        return E_AS_DNC_SEL_PARAM;
    }

  if ((pDncParam1 != NULL) || (pDncParam2 != NULL))
    {
      write_ac_reg(RI_DSPRAM3_CLR, 1);
      audioDelayTask(1);  /* 512 cycle @24.576MHz */
      write_ac_reg(RI_DSPRAM3_CLR, 0);
    }

  if ((dncId == AS_DNC_SEL_DNC1) || (dncId == AS_DNC_SEL_BOTH))
    {
      if (pDncParam1 != NULL)
        {
          write_ac_reg(RI_RAM_RW_EN, 1);
          rtCode = setDncRam(DNC1_IRAM_BASE, pDncParam1->pDncIRam,
                             DNC_IRAM_SIZE);
          if (rtCode != E_AS_OK)
            {
              return rtCode;
            }

          rtCode = setDncRam(DNC1_CRAM_BASE,
                             pDncParam1->pDncCRam,
                             DNC_CRAM_SIZE);
          if (rtCode != E_AS_OK)
            {
              return rtCode;
            }
          write_ac_reg(RI_RAM_RW_EN, 0);
        }
    }

  if ((dncId == AS_DNC_SEL_DNC2) || (dncId == AS_DNC_SEL_BOTH))
    {
      if (pDncParam2 != NULL)
        {
          write_ac_reg(RI_RAM_RW_EN, 1);
          rtCode = setDncRam(DNC2_IRAM_BASE,
                             pDncParam2->pDncIRam,
                             DNC_IRAM_SIZE);
          if (rtCode != E_AS_OK)
            {
              return rtCode;
            }

          rtCode = setDncRam(DNC2_CRAM_BASE,
                             pDncParam2->pDncCRam,
                             DNC_CRAM_SIZE);
          if (rtCode != E_AS_OK)
            {
              return rtCode;
            }
          write_ac_reg(RI_RAM_RW_EN, 0);
        }
    }

  if (dncEn == true)
    {
      switch (dncId)
        {
          case AS_DNC_SEL_DNC1:
            write_ac_reg(RI_DNC1_START, 1);
            write_ac_reg(RI_DNC1_MUTE, 0);
            break;

          case AS_DNC_SEL_DNC2:
            write_ac_reg(RI_DNC2_START, 1);
            write_ac_reg(RI_DNC2_MUTE, 0);
            break;

          case AS_DNC_SEL_BOTH:
            write_ac_reg(RI_DNC1_START, 1);
            write_ac_reg(RI_DNC2_START, 1);
            write_ac_reg(RI_DNC1_MUTE, 0);
            write_ac_reg(RI_DNC2_MUTE, 0);
            break;

          default:
            return E_AS_DNC_SEL_PARAM;
        }
    }

  return rtCode;
}

E_AS asAc_PowerOnSmstr(void)
{
  E_AS rtCode = E_AS_OK;

  /* Power on S-Mster. */

  write_ac_reg(RI_PDN_SMSTR, 0);

  rtCode = asAc_SetSmstrParam();

  return rtCode;
}

E_AS asAc_SetSmstrParam(void)
{
  E_AS rtCode = E_AS_OK;

  asSmstrModeId mode;
  if ((bb_config_tblp->clk_mode == AS_CLK_MODE_HIRES) &&
      (bb_config_tblp->xtal_sel == AS_XTAL_49_152MHZ))
    {
      mode = AS_SMSTR_MODE_FS_32;
    }
  else
    {
      mode = AS_SMSTR_MODE_FS_16;
    }
  rtCode = setAcSmstrParam(mode);

  asBca_SetSmstrParam();

  return rtCode;
}

E_AS asAc_EnableSmstr(void)
{
  E_AS rtCode = E_AS_OK;

  write_ac_reg(RI_NSPMUTE, 0);

  return rtCode;
}

E_AS asAc_PowerOnSerDes(FAR asSerDesParam *pSdesParam)
{
  E_AS rtCode = E_AS_OK;

  D_ASSERT(pSdesParam != NULL);

  write_ac_reg(RI_SDES_EN, 1);

  rtCode = asAc_SetSerDesParam(pSdesParam);

  return rtCode;
}

E_AS asAc_SetSerDesParam(FAR asSerDesParam *pSdesParam)
{
  E_AS rtCode = E_AS_OK;

  D_ASSERT(pSdesParam != NULL);

  rtCode = setAcSerDesParam(pSdesParam);

  return rtCode;
}

E_AS asAc_EnableSer(void)
{
  E_AS rtCode = E_AS_OK;

  write_ac_reg(RI_FS_CLK_EN, 1);
  write_ac_reg(RI_PDM_OUT_EN, 1);

  return rtCode;
}

void asAc_PowerOffAudioCodec()
{
  /* Disable AHBMASTER. */

  write_ac_reg(RI_MCK_AHBMSTR_EN, 0);

  /* Disable CODEC. */

  write_ac_reg(RI_ALC_EN, 0);
  write_ac_reg(RI_SPC_EN, 0);
  write_ac_reg(RI_DEQ_EN, 0);

  /* Disable DNC. */

  write_ac_reg(RI_DNC1_MUTE, 1);
  write_ac_reg(RI_DNC2_MUTE, 1);
  write_ac_reg(RI_DNC1_START, 0);
  write_ac_reg(RI_DNC2_START, 0);

  /* Disable SRC. */

  write_ac_reg(RI_SDIN1_EN, 0);
  write_ac_reg(RI_SDIN2_EN, 0);
  write_ac_reg(RI_SDOUT1_EN, 0);
  write_ac_reg(RI_SDOUT2_EN, 0);
  write_ac_reg(RI_SDCK_OUTENX, 1);
  write_ac_reg(RI_BLF_EN, 0);

  /* Disable SDES. */

  write_ac_reg(RI_PDM_OUT_EN, 0);
  write_ac_reg(RI_FS_CLK_EN, 0);
  write_ac_reg(RI_SDES_EN, 0);

  /* Power off SRC. */

  write_ac_reg(RI_PDN_DSPS1, 1);
  write_ac_reg(RI_PDN_DSPS2, 1);
  write_ac_reg(RI_PDN_DSPB, 1);

  /* Power off CODEC. */

  write_ac_reg(RI_PDN_DSPC, 1);

  /* Power off DNC. */

  write_ac_reg(RI_PDN_DNC1, 1);
  write_ac_reg(RI_PDN_DNC2, 1);
  write_ac_reg(RI_PDN_ANC, 1);
}

void asAc_PowerOffAudioCodecInput()
{
  /* Disable DECIM. */

  write_ac_reg(RI_DECIM0_EN, 0);

  /* Power off CIC. */

  write_ac_reg(RI_PDN_AMIC1, 1);
  write_ac_reg(RI_PDN_AMIC2, 1);
  write_ac_reg(RI_PDN_AMICEXT, 1);
  write_ac_reg(RI_PDN_DMIC, 1);
}

E_AS asAc_PowerOffAudioCodecOutput()
{
  E_AS rtCode = E_AS_OK;
  asCodecVol codecVol;

  if (bb_config_add_tbl.output_device_sel != AS_OUT_DEV_OFF)
    {
      codecVol.input1_db = AS_VOLUME_MUTE;
      codecVol.input2_db = AS_VOLUME_MUTE;
      codecVol.master_db = AS_VOLUME_MUTE;
      rtCode = AS_SetVolume(&codecVol);
      D_ASSERT(rtCode == E_AS_OK);
      if (rtCode != E_AS_OK)
        {
          return rtCode;
        }

      rtCode = AS_BeepDisable();
      D_ASSERT(rtCode == E_AS_OK);
      if (rtCode != E_AS_OK)
      {
        return rtCode;
      }
    }

  /* Power off S-Master. */

  write_ac_reg(RI_NSPMUTE, 1);
  write_ac_reg(RI_PDN_SMSTR, 1);

  return rtCode;
}

void asAc_AhbmasterEnable()
{
  write_ac_reg(RI_MCK_AHBMSTR_EN, 1);
}

E_AS PowerOnAudioCodec(uint32_t rate[AS_I2S_ID_NUM],
                       asBypassModeId bypass_mode_en[AS_I2S_ID_NUM])
{
  E_AS rtCode = E_AS_OK;

  rtCode = GetSdesParam();
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  rtCode = GetI2sParam(rate, bypass_mode_en);
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  rtCode = asAc_CheckID();
  _info("asAc_CheckID(%d)\n", rtCode);
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }

  rtCode = asAc_InitDsp();
  _info("asAc_InitDsp(%d)\n", rtCode);
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }

  rtCode = asAc_PowerOnSerDes(&sSdesParam);
  _info("asAc_PowerOnSerDes(%d)\n", rtCode);
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }

  rtCode = asAc_PowerOnCodec();
  _info("asAc_PowerOnCodec(%d)\n", rtCode);
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }

  rtCode = asAc_PowerOnSrc(gSrcId, &sI2sParam);
  _info("asAc_PowerOnSrc(%d)\n", rtCode);
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }

  rtCode = asAc_ResetDsp();
  _info("asAc_ResetDsp(%d)\n", rtCode);
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }

  rtCode = asAc_EnableSrc(gSrcId);
  _info("asAc_EnableSrc(%d)\n", rtCode);
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }

  rtCode = asAc_EnableSer();
  _info("asAc_EnableSer(%d)\n", rtCode);
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }
  rtCode = asAc_EnableSer();
  _info("asAc_EnableSer(%d)\n", rtCode);

  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }

  if (bb_config_tblp->alc_spc_sel != AS_ALC_SPC_SEL_OFF)
    {
      rtCode = asAc_SetAlcSpcParam();
      _info("asAc_SetAlcSpcParam(%d)\n", rtCode);
    }

  return rtCode;
}

E_AS EnableAudioCodecInput(int32_t micgain[AS_MIC_CHANNEL_MAX])
{
  E_AS rtCode = E_AS_OK;

  uint8_t cicNum = (bb_config_add_tbl.micNum + 1) / AS_AC_CIC_MIC_CH_NUM;
  asCicSelId cicId = AS_CIC_SEL_CIC1;
  asCicParam cicParam;

  for (uint8_t i = 0; i < cicNum; i++)
    {
      cicId = (asCicSelId)i;
      rtCode = GetCicParam(&micgain[i*AS_AC_CIC_MIC_CH_NUM],
                           cicId, &cicParam);
      if (rtCode != E_AS_OK)
        {
          return rtCode;
        }
      switch (cicId)
        {
          case AS_CIC_SEL_CIC1:
          case AS_CIC_SEL_CIC2:
          case AS_CIC_SEL_CIC12:
            rtCode = asAc_PowerOnCic(cicId, &cicParam);
            _info("asAc_PowerOnCic(%d)\n", rtCode);
            if (E_AS_OK != rtCode)
              {
                return rtCode;
              }
            break;

          case AS_CIC_SEL_CIC3:
          case AS_CIC_SEL_CIC4:
          case AS_CIC_SEL_CIC34:
          case AS_CIC_SEL_CIC_ALL:
            if (cicParam.gainMode == AS_CIC_GAIN_MODE_CIC)
              {
                rtCode = asAc_PowerOnCic(cicId, &cicParam);
                _info("asAc_PowerOnCic(%d)\n", rtCode);
                if (E_AS_OK != rtCode)
                  {
                    return rtCode;
                  }
              }
            break;

          default:
            D_ASSERT(0);
            break;
        }
    }
  rtCode = asAc_PowerOnDecim();
  _info("asAc_PowerOnDecim(%d)\n", rtCode);

  return rtCode;
}

E_AS EnableAudioCodecOutput(void)
{
  E_AS rtCode = E_AS_OK;

  rtCode = asAc_PowerOnSmstr();
  _info("asAc_PowerOnSmstr(%d)\n", rtCode);
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }
  rtCode = asAc_EnableSmstr();
  _info("asAc_EnableSmstr(%d)\n", rtCode);

  return rtCode;
}

void setAcOutputI2S(void)
{
  setDataPathTo(AS_PATH_FROM_MIXER, TO_SPI2S_I2S1);
}

