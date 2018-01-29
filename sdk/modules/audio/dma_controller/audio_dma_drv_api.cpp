/****************************************************************************
 * modules/audio/dma_controller/audio_dma_drv_api.cpp
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
 ****************************************************************************/

#include <sys/types.h>
#include <arch/chip/cxd56_audio.h>

#include "audio_bb_drv.h"
#include "audio_dma_drv_api.h"
#include "memutils/common_utils/common_assert.h"
#include <debug.h>
#include "debug/dbg_log.h"

static uint16_t dmacMinimumSize[5] = {0};

/*--------------------------------------------------------------------*/
static E_AS initDmac(asInitDmacParam *pInitDmacParam)
{
  E_AS rtCode = E_AS_OK;
  E_AS_BB rtCodeBB = E_AS_BB_DMA_OK;
  uint32_t dma_err;
  AudioDrvDmaInitParam param;

  rtCode = asDmac_InitDriver(pInitDmacParam->dmacId,
                             pInitDmacParam->format,
                             &param.ch_num);
  if (rtCode != E_AS_OK)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_BASEBAND_ERROR);
      return rtCode;
    }

  _info("dma(%d:%d)\n", pInitDmacParam->dmacId, pInitDmacParam->format);

  clearDmacDoneIntStatus(pInitDmacParam->dmacId);
  clearDmacErrIntStatus(pInitDmacParam->dmacId);
  clearDmacCmbIntStatus(pInitDmacParam->dmacId);

  if (pInitDmacParam->p_dmadone_func != NULL)
    {
      dmacMinimumSize[pInitDmacParam->dmacId] = DMAC_MIN_SIZE_INT;
      param.p_dmadone_func = pInitDmacParam->p_dmadone_func;
      setDmacDoneIntMask(pInitDmacParam->dmacId, false);
      /* TODO: should be false for ES */
      setDmacErrIntMask(pInitDmacParam->dmacId, true);
      setDmacBusIntMask(pInitDmacParam->dmacId, false);
      setDmacCmbIntMask(pInitDmacParam->dmacId, false);
    }
  else
    {
      dmacMinimumSize[pInitDmacParam->dmacId] = DMAC_MIN_SIZE_POL;
      param.p_dmadone_func = NULL;
      setDmacDoneIntMask(pInitDmacParam->dmacId, true);
      setDmacErrIntMask(pInitDmacParam->dmacId, true);
      setDmacBusIntMask(pInitDmacParam->dmacId, false);
      setDmacCmbIntMask(pInitDmacParam->dmacId, false);
    }

  getDmacErrorStatus(pInitDmacParam->dmacId, &dma_err);
  F_ASSERT(dma_err == 0);

  param.dmac_id      = pInitDmacParam->dmacId;
  param.p_error_func = pInitDmacParam->p_error_func;

  if (pInitDmacParam->format == AS_SAMPLING_FMT_24)
    {
      param.dma_byte_len = AS_DMAC_BYTE_WT_24BIT;
    }
  else
    {
      param.dma_byte_len = AS_DMAC_BYTE_WT_16BIT;
    }

  param.fade_en = pInitDmacParam->fade_en;

  rtCodeBB = AS_AudioDrvDmaInit(&param);

  if (rtCodeBB != E_AS_BB_DMA_OK)
    {
      _err("ERR: dma(%d) er(%d)\n", pInitDmacParam->dmacId, rtCodeBB);
      rtCode = E_AS_DMAC_MSG_SEND_ERR;
    }

  return rtCode;
}

/*--------------------------------------------------------------------*/
E_AS AS_InitDmac(asInitDmacParam *pInitDmacParam)
{
  if (pInitDmacParam == NULL)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return E_AS_INITDMAC_NULL;
    }

  if (!chkPowerOnBaseBand())
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_BASEBAND_ERROR);
      return E_AS_INITDMAC_POWER_ON_CHK_ERR;
    }

  asAc_AhbmasterEnable();

  E_AS rtCode = initDmac(pInitDmacParam);

  return rtCode;
}

/*--------------------------------------------------------------------*/
E_AS AS_StartDmac(asDmacSelId dmacId)
{
  E_AS rtCode = E_AS_OK;
  E_AS_BB rtCodeBB = E_AS_BB_DMA_OK;
  uint32_t stat;

  if (!chkPowerOnBaseBand())
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_BASEBAND_ERROR);
      return E_AS_STARTDMAC_POWER_ON_CHK_ERR;
    }

  if ((dmacId == AS_DMAC_ID_NONE)
   || (dmacId >= AS_DMAC_SEL_MAX_ENTRY))
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return E_AS_DMAC_ID_PARAM;
    }

  rtCode = getDmacCmdStatus(dmacId, &stat);

  if (rtCode != E_AS_OK)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_BASEBAND_ERROR);
      return rtCode;
   }

  if (stat != 1)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return E_AS_DMAC_BUSY;
    }

  rtCodeBB = AS_AudioDrvDmaStart(dmacId);

  if (rtCodeBB != E_AS_BB_DMA_OK)
    {
      return E_AS_DMAC_MSG_SEND_ERR;
    }

  return rtCode;
}

/*--------------------------------------------------------------------*/
E_AS AS_ReadDmac(asReadDmacParam *pReadDmacParam)
{
  E_AS rtCode = E_AS_OK;
  E_AS_BB rtCodeBB = E_AS_BB_DMA_OK;

  if (!chkPowerOnBaseBand())
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_BASEBAND_ERROR);
      return E_AS_READDMAC_POWER_ON_CHK_ERR;
    }

  if (pReadDmacParam == NULL)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return E_AS_WRITEDMAC_NULL;
    }

  switch (pReadDmacParam->dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
      case AS_DMAC_SEL_I2S_IN:
      case AS_DMAC_SEL_I2S2_IN:
          break;

      case AS_DMAC_SEL_I2S_OUT:
      case AS_DMAC_SEL_I2S2_OUT:
          DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          _err("ERR: dma(%d) ID error\n", pReadDmacParam->dmacId);
          return E_AS_DMAC_ID_PARAM;

      default:
          DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          return E_AS_DMAC_ID_PARAM;
    }

  /* Error check */

  if ((pReadDmacParam->size + pReadDmacParam->size2) > DMAC_MAX_SIZE)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) size(%d,%d)\n",
             pReadDmacParam->dmacId, pReadDmacParam->size,
             pReadDmacParam->size2);

      return E_AS_DMAC_SIZE_MAX_ERR;
    }

  if (pReadDmacParam->size < dmacMinimumSize[pReadDmacParam->dmacId])
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) size(%d) min(%d)\n",
             pReadDmacParam->dmacId, pReadDmacParam->size,
             dmacMinimumSize[pReadDmacParam->dmacId]);

      return E_AS_DMAC_SIZE_MIN_ERR;
    }

  if ((uint32_t*)(pReadDmacParam->addr) == NULL)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) addr(0x%x)\n",
             pReadDmacParam->dmacId, pReadDmacParam->addr);

      return E_AS_DMAC_TRANS_ADDR_NULL;
    }

  if ((0 < pReadDmacParam->size2)
   && (pReadDmacParam->size2 < dmacMinimumSize[pReadDmacParam->dmacId]))
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) size2(%d) min(%d)\n",
             pReadDmacParam->dmacId, pReadDmacParam->size2,
             dmacMinimumSize[pReadDmacParam->dmacId]);

      return E_AS_DMAC_SIZE_MIN_ERR;
    }

  if ((pReadDmacParam->size2 > 0)
   && ((uint32_t*)(pReadDmacParam->addr2) == NULL))
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) size2(%d) addr2(0x%x)\n",
             pReadDmacParam->dmacId, pReadDmacParam->size2,
             pReadDmacParam->addr2);

      return E_AS_DMAC_TRANS_ADDR_NULL;
    }

  rtCodeBB = AS_AudioDrvDmaRun(pReadDmacParam);

  if (rtCodeBB != E_AS_BB_DMA_OK)
    {
      return E_AS_DMAC_MSG_SEND_ERR;
    }

  return rtCode;
}

/*--------------------------------------------------------------------*/
E_AS AS_WriteDmac(asWriteDmacParam *pWriteDmacParam)
{
  E_AS rtCode = E_AS_OK;
  E_AS_BB rtCodeBB = E_AS_BB_DMA_OK;

  if (!chkPowerOnBaseBand())
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_BASEBAND_ERROR);
      return E_AS_WRITEDMAC_POWER_ON_CHK_ERR;
    }

  if (pWriteDmacParam == NULL)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return E_AS_WRITEDMAC_NULL;
    }

  switch (pWriteDmacParam->dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
      case AS_DMAC_SEL_I2S_IN:
      case AS_DMAC_SEL_I2S2_IN:
          DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          _err("ERR: dma(%d) ID error\n", pWriteDmacParam->dmacId);
          return E_AS_DMAC_ID_PARAM;

      case AS_DMAC_SEL_I2S_OUT:
      case AS_DMAC_SEL_I2S2_OUT:
          break;

      default:
          DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          return E_AS_DMAC_ID_PARAM;
    }

  /* Error check */

  if ((pWriteDmacParam->size + pWriteDmacParam->size2) > DMAC_MAX_SIZE)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) size(%d,%d)\n",
             pWriteDmacParam->dmacId, pWriteDmacParam->size,
             pWriteDmacParam->size2);

      return E_AS_DMAC_SIZE_MAX_ERR;
    }

  if (pWriteDmacParam->size < dmacMinimumSize[pWriteDmacParam->dmacId])
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) size(%d) min(%d)\n",
             pWriteDmacParam->dmacId, pWriteDmacParam->size,
             dmacMinimumSize[pWriteDmacParam->dmacId]);

      return E_AS_DMAC_SIZE_MIN_ERR;
    }

  if ((uint32_t*)(pWriteDmacParam->addr) == NULL)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) addr(0x%x)\n",
             pWriteDmacParam->dmacId, pWriteDmacParam->addr);

      return E_AS_DMAC_TRANS_ADDR_NULL;
    }

  if ((0 < pWriteDmacParam->size2)
   && (pWriteDmacParam->size2 < dmacMinimumSize[pWriteDmacParam->dmacId]))
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) size2(%d) min(%d)\n",
             pWriteDmacParam->dmacId, pWriteDmacParam->size2,
             dmacMinimumSize[pWriteDmacParam->dmacId]);

      return E_AS_DMAC_SIZE_MIN_ERR;
    }

  if ((pWriteDmacParam->size2 > 0)
   && ((uint32_t*)(pWriteDmacParam->addr2) == NULL))
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      _err("ERR: dma(%d) size2(%d) addr2(0x%x)\n",
             pWriteDmacParam->dmacId, pWriteDmacParam->size2,
             pWriteDmacParam->addr2);

      return E_AS_DMAC_TRANS_ADDR_NULL;
    }

  rtCodeBB = AS_AudioDrvDmaRun((asReadDmacParam *)pWriteDmacParam);

  if (rtCodeBB != E_AS_BB_DMA_OK)
    {
      return E_AS_DMAC_MSG_SEND_ERR;
    }

  return rtCode;
}

/*--------------------------------------------------------------------*/
E_AS AS_StopDmac(asDmacSelId dmacId, asDmacStopMode stopMode)
{
  E_AS rtCode = E_AS_OK;
  E_AS_BB rtCodeBB = E_AS_BB_DMA_OK;
  AudioDrvDmaStopParam param;

  if (!chkPowerOnBaseBand())
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_BASEBAND_ERROR);
      return E_AS_STOPDMAC_POWER_ON_CHK_ERR;
    }

  if ((dmacId == AS_DMAC_ID_NONE) || (dmacId >= AS_DMAC_SEL_MAX_ENTRY))
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return E_AS_DMAC_ID_PARAM;
    }

  param.dmac_id   = dmacId;
  param.stop_mode = (AudioDrvDmaStopMode)stopMode;

  rtCodeBB = AS_AudioDrvDmaStop(&param);

  if (rtCodeBB != E_AS_BB_DMA_OK)
    {
      return E_AS_DMAC_MSG_SEND_ERR;
    }

  return rtCode;
}

/*--------------------------------------------------------------------*/
E_AS AS_GetReadyCmdNumDmac(asDmacSelId dmacId, uint32_t *pResult)
{
  E_AS rtCode = E_AS_OK;
  E_AS_BB rtCodeBB = E_AS_BB_DMA_OK;
  AudioDrvDmaInfo dmaInfo;

  if (!chkPowerOnBaseBand())
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_BASEBAND_ERROR);
      return E_AS_GETREADYCMDDMAC_POWER_ON_CHK_ERR;
    }

  if ((dmacId == AS_DMAC_ID_NONE) || (dmacId >= AS_DMAC_SEL_MAX_ENTRY))
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return E_AS_DMAC_ID_PARAM;
    }

  if (pResult == NULL)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return E_AS_GETREADYCMD_RESULT_NULL;
    }

  rtCodeBB = AS_AudioDrvDmaGetInfo(dmacId, &dmaInfo);

  if (rtCodeBB != E_AS_BB_DMA_OK)
    {
      return E_AS_DMAC_MSG_SEND_ERR;
    }

  *pResult = dmaInfo.ready_empty + dmaInfo.running_empty;

  _info("dma(%d:%d,%d)\n", dmacId, dmaInfo.state, *pResult);

  return rtCode;
}

/*--------------------------------------------------------------------*/
E_AS AS_RegistDmaIntCb(asDmacSelId dmacId, AS_DmaIntCb p_dmaIntCb)
{
  E_AS rtCode = E_AS_OK;

  if (!chkPowerOnBaseBand())
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_BASEBAND_ERROR);
      return E_AS_GETREADYCMDDMAC_POWER_ON_CHK_ERR;
    }

  if ((dmacId == AS_DMAC_ID_NONE) || (dmacId >= AS_DMAC_SEL_MAX_ENTRY))
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return E_AS_DMAC_ID_PARAM;
    }

  if (p_dmaIntCb == NULL)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return E_AS_GETREADYCMD_RESULT_NULL;
    }

  rtCode = asDmac_RegsitIntCb(dmacId, p_dmaIntCb);

  return rtCode;
}

/*--------------------------------------------------------------------*/
E_AS AS_NotifyDmaCmplt(asDmacSelId dmacId, E_AS_DMA_INT code)
{
  E_AS rtCode = E_AS_OK;

  if (!chkPowerOnBaseBand())
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_BASEBAND_ERROR);
      return E_AS_GETREADYCMDDMAC_POWER_ON_CHK_ERR;
    }

  if ((dmacId == AS_DMAC_ID_NONE) || (dmacId >= AS_DMAC_SEL_MAX_ENTRY))
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return E_AS_DMAC_ID_PARAM;
    }

  if (E_AS_BB_DMA_OK != AS_AudioDrvDmaNofifyCmplt(dmacId, code))
    {
      return E_AS_DMAC_ID_PARAM;
    }

  return rtCode;
}

