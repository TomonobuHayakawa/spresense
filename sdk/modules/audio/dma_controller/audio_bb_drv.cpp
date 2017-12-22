/****************************************************************************
 * modules/audio/dma_controller/audio_bb_drv.cpp
 *
 *   Copyright (C) 2016-2017 Sony Corporation. All rights reserved.
 *   Author: Naoya Haneda <Naoya.Haneda@sony.com>
 *           Hayakawa Tomonobu <Tomonobu.Hayakawa@sony.com>
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

/****************************************************************************
    Include
 ****************************************************************************/

#include "memutils/message/Message.h"
#include "common/audio_message_types.h"
#include "audio_bb_drv.h"
#include "audio_dma_drv.h"

#include <arch/chip/cxd56_audio.h>

#include <debug.h>

#include "audio/audio_high_level_api.h"/*TODO:Do not want include.*/

/****************************************************************************
    Definitions
 ****************************************************************************/
#define CHECK_DMA_ERR_INT
#define CHECK_BUS_ERR_INT

/****************************************************************************
    Macros
 ****************************************************************************/
extern "C" {

/****************************************************************************/
/*TODO:Do not need all.*/
static AsDmaDrv *s_dma_drv_instance[AS_DMAC_SEL_MAX_ENTRY-1];

/****************************************************************************/
E_AS dmaDrvActive(asDmacSelId dmacId)
{
  s_dma_drv_instance[dmacId] = new AsDmaDrv(dmacId);

  if (s_dma_drv_instance[dmacId] == NULL)
    {
      return E_AS_DMAC_ACTIVATED_ERR;
    }

  return E_AS_OK;
}


/****************************************************************************/
E_AS dmaDrvDeactive(asDmacSelId dmacId)
{
  if (s_dma_drv_instance[dmacId] == NULL)
    {
      return E_AS_DMAC_DEACTIVATED_ERR;
    }

  delete s_dma_drv_instance[dmacId];

  return E_AS_OK;
}

/****************************************************************************/

static bool activateDmac[AS_DMAC_SEL_MAX_ENTRY - 1] =
{
  false, false, false, false, false
};

/*--------------------------------------------------------------------*/
E_AS AS_ActivateDmac(asDmacSelId dmacId)
{
  E_AS rtCode = E_AS_OK;

  /* Check baseband power status. */

  if (!chkPowerOnBaseBand())
    {
      return E_AS_ACTDMAC_POWER_ON_CHK_ERR;
    }

  /* Check DMAC id. */

  if (dmacId >= (AS_DMAC_SEL_MAX_ENTRY - 1))
    {
      return E_AS_DMAC_ID_PARAM;
    }

  /* Get resource for using dmac. */

  if (!activateDmac[dmacId])
    {
      rtCode = dmaDrvActive(dmacId);

      if(rtCode == E_AS_OK)
        {
          activateDmac[dmacId] = true;
        }
    }
  else
    {
      rtCode = E_AS_DMAC_ACTIVATED_ERR;
    }

  _info("dma(%d) er(%d)\n", dmacId, rtCode);

  return rtCode;
}

/*--------------------------------------------------------------------*/
E_AS AS_DeactivateDmac(asDmacSelId dmacId)
{
  E_AS rtCode = E_AS_OK;

  /* Check baseband power status. */

  if (!chkPowerOnBaseBand())
    {
      return E_AS_DEACTDMAC_POWER_ON_CHK_ERR;
    }

  /* Check DMAC id. */

  if (dmacId >= (AS_DMAC_SEL_MAX_ENTRY - 1))
    {
      return E_AS_DMAC_ID_PARAM;
    }

  /* Release resource for using dmac. */

  if (activateDmac[dmacId])
    {
      rtCode = dmaDrvDeactive(dmacId);

      if(rtCode == E_AS_OK)
        {
          activateDmac[dmacId] = false;
        }
    }
  else
    {
      rtCode = E_AS_DMAC_DEACTIVATED_ERR;
    }

  _info("dma(%d) er(%d)\n", dmacId, rtCode);

  return rtCode;
}

} /* extern "C" */

/****************************************************************************/
/* API      */

/*--------------------------------------------------------------------*/
E_AS_BB AS_AudioDrvDmaInit(AudioDrvDmaInitParam *pParam)
{
  if (s_dma_drv_instance[pParam->dmac_id]->parse(AsDmaDrv::EvtInit,
                                                 (void *)pParam))
    {
      return E_AS_BB_DMA_OK;
    }

  return E_AS_BB_DMA_ILLEGAL;
}

/*--------------------------------------------------------------------*/
E_AS_BB AS_AudioDrvDmaRun(asReadDmacParam *pParam)
{
  if (s_dma_drv_instance[pParam->dmacId]->parse(AsDmaDrv::EvtRun,
                                                (void *)pParam))
    {
      return E_AS_BB_DMA_OK;
    }

  return E_AS_BB_DMA_ILLEGAL;
}

/*--------------------------------------------------------------------*/
E_AS_BB AS_AudioDrvDmaStart(asDmacSelId dmac_id)
{
  if (s_dma_drv_instance[dmac_id]->parse(AsDmaDrv::EvtStart,
                                         (void *)&dmac_id))
    {
      return E_AS_BB_DMA_OK;
    }

  return E_AS_BB_DMA_ILLEGAL;
}

/*--------------------------------------------------------------------*/
E_AS_BB AS_AudioDrvDmaStop(AudioDrvDmaStopParam *pParam)
{
  if (s_dma_drv_instance[pParam->dmac_id]->parse(AsDmaDrv::EvtStop,
                                                 (void *)pParam))
    {
      return E_AS_BB_DMA_OK;
    }

  return E_AS_BB_DMA_ILLEGAL;
}

/*--------------------------------------------------------------------*/
E_AS_BB AS_AudioDrvDmaGetInfo(asDmacSelId dmac_id, AudioDrvDmaInfo *pDmaInfo)
{
  if (s_dma_drv_instance[dmac_id]->parse(AsDmaDrv::EvtGetInfo,
                                         (void *)pDmaInfo))
    {
      return E_AS_BB_DMA_OK;
    }

  return E_AS_BB_DMA_ILLEGAL;
}

/*--------------------------------------------------------------------*/
void enable_audio_int(void)
{
  volatile uint32_t int_en;

  int_en = read_as_reg(AS_INT_EN1_REG);

  int_en |= (1 << AS_INT_EN1_BIT_AU0); /* INT_MIC */
  int_en |= (1 << AS_INT_EN1_BIT_AU1); /* INT_I2S1 */
  int_en |= (1 << AS_INT_EN1_BIT_AU2); /* INT_I2S2 */
  int_en |= (1 << AS_INT_EN1_BIT_AU3); /* INT_CODEC */

  write_as_reg(AS_INT_EN1_REG, int_en);

  return;
}

/*--------------------------------------------------------------------*/
void disable_audio_int(void)
{
  volatile uint32_t int_en;

  int_en = read_as_reg(AS_INT_EN1_REG);

  int_en &= ~(1 << AS_INT_EN1_BIT_AU0); /* INT_MIC */
  int_en &= ~(1 << AS_INT_EN1_BIT_AU1); /* INT_I2S1 */
  int_en &= ~(1 << AS_INT_EN1_BIT_AU2); /* INT_I2S2 */
  int_en &= ~(1 << AS_INT_EN1_BIT_AU3); /* INT_CODEC */

  write_as_reg(AS_INT_EN1_REG, int_en);

  return;
}

/*--------------------------------------------------------------------*/
#ifdef CHECK_AC_INT
static void ac_int_callback(uint32_t ac_intr)
{
  if (ac_intr & (1<<bcaRegMap[BCA_Int_hresp_err].pos))
    {
      _info("HRESP_ERR(%x)\n", ac_intr);
    }

  if (ac_intr & (1<<bcaRegMap[BCA_Int_i2s_bck_err1].pos))
    {
      _info("I2S_BCK_ERR1(%x)\n", ac_intr);
    }

  if (ac_intr & (1<<bcaRegMap[BCA_Int_i2s_bck_err2].pos))
    {
      _info("I2S_BCK_ERR2(%x)\n", ac_intr);
    }

  if (ac_intr & (1<<bcaRegMap[BCA_Int_anc_faint].pos))
    {
      _info("ANC_FAINT(%x)\n", ac_intr);
    }

  if (ac_intr & (1<<bcaRegMap[BCA_Int_ovf_smasl].pos))
    {
      _info("OVF_SMASL(%x)\n", ac_intr);
    }

  if (ac_intr & (1<<bcaRegMap[BCA_Int_ovf_smasr].pos))
    {
      _info("OVF_SMASR(%x)\n", ac_intr);
    }

  if (ac_intr & (1<<bcaRegMap[BCA_Int_ovf_dnc1l].pos))
    {
      _info("OVF_DNC1L(%x)\n", ac_intr);
    }

  if (ac_intr & (1<<bcaRegMap[BCA_Int_ovf_dnc1r].pos))
    {
      _info("OVF_DNC1R(%x)\n", ac_intr);
    }

  if (ac_intr & (1<<bcaRegMap[BCA_Int_ovf_dnc2l].pos))
    {
      _info("OVF_DNC2L(%x)\n", ac_intr);
    }

  if (ac_intr & (1<<bcaRegMap[BCA_Int_ovf_dnc2r].pos))
    {
      _info("OVF_DNC2R(%x)\n", ac_intr);
    }
}
#endif

/*--------------------------------------------------------------------*/
E_AS_BB AS_AudioDrvDmaNofifyCmplt(asDmacSelId dmacId, E_AS_DMA_INT code)
{
  bool result = false;

  /* Notify dma excution result. */

  switch(code)
    {
      case E_AS_DMA_INT_CMPLT:
        result = s_dma_drv_instance[dmacId]->parse(AsDmaDrv::EvtCmplt, NULL);
        break;

      case E_AS_DMA_INT_ERR:
        result = s_dma_drv_instance[dmacId]->parse(AsDmaDrv::EvtDmaErr, NULL);
        break;

      case E_AS_DMA_INT_ERR_BUS:
        result = s_dma_drv_instance[dmacId]->parse(AsDmaDrv::EvtBusErr, NULL);
        break;

      default:
        break;
    }

  if (result)
    {
      return E_AS_BB_DMA_OK;
    }

  return E_AS_BB_DMA_ILLEGAL;
}

/*--------------------------------------------------------------------*/
static AS_DmaIntCb s_DmaIntCb[AS_DMAC_SEL_MAX_ENTRY];

E_AS_BB AS_AudioDrvDmaRegsitIntCb(asDmacSelId dmacId, AS_DmaIntCb pDmacCb)
{
  if (AS_DMAC_SEL_MAX_ENTRY < dmacId)
    {
      return E_AS_BB_DMA_ILLEGAL;
    }

  s_DmaIntCb[dmacId] = pDmacCb;

  return E_AS_BB_DMA_OK;
}

extern "C" {

/*--------------------------------------------------------------------*/
void AS_AudioIntHandler(void)
{
  uint32_t int_irq = read_as_reg(AS_INT_IRQ1_REG);

  uint32_t int_ac =
    read32_bca_reg(bcaRegMap[BCA_Mic_Int_Ctrl_done_mic].addr)
    & ~(read32_bca_reg(bcaRegMap[BCA_Mic_Int_Mask_done_mic].addr))
    & 0x0f;

  uint32_t int_i2s =
    read32_bca_reg(bcaRegMap[BCA_I2s1_Int_Ctrl_done_i2so].addr)
    & ~(read32_bca_reg(bcaRegMap[BCA_I2s1_Int_Mask_done_i2so].addr))
    & 0x3f;

  uint32_t int_i2s2 =
    read32_bca_reg(bcaRegMap[BCA_I2s2_Int_Ctrl_done_i2so].addr)
    & ~(read32_bca_reg(bcaRegMap[BCA_I2s2_Int_Mask_done_i2so].addr))
    & 0x3f;

  /* AUDIO_INT_AC : check interruption from mic */

  if (int_irq & (1 << AS_INT_IRQ1_BIT_AU0))
    {
      if (int_ac != 0)
        {
          write32_bca_reg(bcaRegMap[BCA_Mic_Int_Ctrl_done_mic].addr, int_ac);

          if (int_ac & (1<<bcaRegMap[BCA_Mic_Int_Ctrl_done_mic].pos))
            {
              (*s_DmaIntCb[AS_DMAC_SEL_AC_IN])(AS_DMAC_SEL_AC_IN,
                                               E_AS_DMA_INT_CMPLT);
            }

          if (int_ac & (1<<bcaRegMap[BCA_Mic_Int_Ctrl_err_mic].pos))
            {
              setDmacErrIntMask(AS_DMAC_SEL_AC_IN, true);

              clearDmacErrIntStatus(AS_DMAC_SEL_AC_IN);

#ifdef CHECK_DMA_ERR_INT
              (*s_DmaIntCb[AS_DMAC_SEL_AC_IN])(AS_DMAC_SEL_AC_IN,
                                               E_AS_DMA_INT_ERR);
#endif
            }

          if (int_ac & (1<<bcaRegMap[BCA_Mic_Int_Ctrl_cmb_mic].pos))
            {
              setDmacCmbIntMask(AS_DMAC_SEL_AC_IN, true);

              clearDmacCmbIntStatus(AS_DMAC_SEL_AC_IN);

#ifdef CHECK_BUS_ERR_INT
              (*s_DmaIntCb[AS_DMAC_SEL_AC_IN])(AS_DMAC_SEL_AC_IN,
                                               E_AS_DMA_INT_ERR);
#endif
            }
        }
    }

  /* AUDIO_INT_I2S1 : check interruption from I2S-1*/

  if (int_irq & (1 << AS_INT_IRQ1_BIT_AU1))
    {
      if (int_i2s != 0)
        {
          write32_bca_reg(bcaRegMap[BCA_I2s1_Int_Ctrl_done_i2so].addr,
                          int_i2s);

          if (int_i2s & (1<<bcaRegMap[BCA_I2s1_Int_Ctrl_done_i2si].pos))
            {
              (*s_DmaIntCb[AS_DMAC_SEL_I2S_IN])(AS_DMAC_SEL_I2S_IN,
                                                E_AS_DMA_INT_CMPLT);
            }

          if (int_i2s & (1<<bcaRegMap[BCA_I2s1_Int_Ctrl_done_i2so].pos))
            {
              (*s_DmaIntCb[AS_DMAC_SEL_I2S_OUT])(AS_DMAC_SEL_I2S_OUT,
                                                 E_AS_DMA_INT_CMPLT);
            }

          if (int_i2s & (1<<bcaRegMap[BCA_I2s1_Int_Ctrl_err_i2si].pos))
            {
              setDmacErrIntMask(AS_DMAC_SEL_I2S_IN, true);

              clearDmacErrIntStatus(AS_DMAC_SEL_I2S_IN);

#ifdef CHECK_DMA_ERR_INT
              (*s_DmaIntCb[AS_DMAC_SEL_I2S_IN])(AS_DMAC_SEL_I2S_IN,
                                                E_AS_DMA_INT_ERR);
#endif
            }

          if (int_i2s & (1<<bcaRegMap[BCA_I2s1_Int_Ctrl_err_i2so].pos))
            {
              setDmacErrIntMask(AS_DMAC_SEL_I2S_OUT, true);

              clearDmacErrIntStatus(AS_DMAC_SEL_I2S_OUT);

#ifdef CHECK_DMA_ERR_INT
              (*s_DmaIntCb[AS_DMAC_SEL_I2S_OUT])(AS_DMAC_SEL_I2S_OUT,
                                                 E_AS_DMA_INT_ERR);
#endif
            }

          if (int_i2s & (1<<bcaRegMap[BCA_I2s1_Int_Ctrl_cmb_i2s].pos))
            {
              setDmacCmbIntMask(AS_DMAC_SEL_I2S_OUT, true);

              clearDmacCmbIntStatus(AS_DMAC_SEL_I2S_OUT);

#ifdef CHECK_BUS_ERR_INT
              (*s_DmaIntCb[AS_DMAC_SEL_I2S_OUT])(AS_DMAC_SEL_I2S_OUT,
                                                 E_AS_DMA_INT_ERR_BUS);
#endif
            }
        }
    }

  /* AUDIO_INT_I2S2 : check interruption from I2S-2 */

  if (int_irq & (1 << AS_INT_IRQ1_BIT_AU2))
    {
      if (int_i2s2 != 0)
        {
          write32_bca_reg(bcaRegMap[BCA_I2s2_Int_Ctrl_done_i2so].addr,
                          int_i2s2);

          if (int_i2s2 & (1<<bcaRegMap[BCA_I2s2_Int_Ctrl_done_i2si].pos))
            {
              (*s_DmaIntCb[AS_DMAC_SEL_I2S2_IN])(AS_DMAC_SEL_I2S2_IN,
                                                 E_AS_DMA_INT_CMPLT);
            }

          if (int_i2s2 & (1<<bcaRegMap[BCA_I2s2_Int_Ctrl_done_i2so].pos))
            {
              (*s_DmaIntCb[AS_DMAC_SEL_I2S2_OUT])(AS_DMAC_SEL_I2S2_OUT,
                                                  E_AS_DMA_INT_CMPLT);
            }

          if (int_i2s2 & (1<<bcaRegMap[BCA_I2s2_Int_Ctrl_err_i2si].pos))
            {
              setDmacErrIntMask(AS_DMAC_SEL_I2S2_IN, true);

              clearDmacErrIntStatus(AS_DMAC_SEL_I2S2_IN);

#ifdef CHECK_DMA_ERR_INT
              (*s_DmaIntCb[AS_DMAC_SEL_I2S2_IN])(AS_DMAC_SEL_I2S2_IN,
                                                 E_AS_DMA_INT_ERR);
#endif
            }

          if (int_i2s2 & (1<<bcaRegMap[BCA_I2s2_Int_Ctrl_err_i2so].pos))
            {
              setDmacErrIntMask(AS_DMAC_SEL_I2S2_OUT, true);

              clearDmacErrIntStatus(AS_DMAC_SEL_I2S2_OUT);

#ifdef CHECK_DMA_ERR_INT
              (*s_DmaIntCb[AS_DMAC_SEL_I2S2_OUT])(AS_DMAC_SEL_I2S2_OUT,
                                                  E_AS_DMA_INT_ERR);
#endif
            }

          if (int_i2s2 & (1<<bcaRegMap[BCA_I2s2_Int_Ctrl_cmb_i2s].pos))
            {
              setDmacCmbIntMask(AS_DMAC_SEL_I2S2_OUT, true);

              clearDmacCmbIntStatus(AS_DMAC_SEL_I2S2_OUT);

#ifdef CHECK_BUS_ERR_INT
              (*s_DmaIntCb[AS_DMAC_SEL_I2S2_OUT])(AS_DMAC_SEL_I2S2_OUT,
                                                  E_AS_DMA_INT_ERR_BUS);
#endif
            }
        }
    }

  if (int_irq & (1 << AS_INT_IRQ1_BIT_AU3))
    {
      uint32_t int_au  = read32_bca_reg(bcaRegMap[BCA_Int_hresp_err].addr);

      if (int_au != 0)
        {
          write32_bca_reg(bcaRegMap[BCA_Int_clr_hresp_err].addr, int_au);

#ifdef CHECK_AC_INT
          ac_int_callback(int_au);
#endif
        }
    }
}

}
