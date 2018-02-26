/****************************************************************************
 * bsp/src/audio/cxd56_audio.c
 *
 *   Copyright (C) 2016, 2017, 2018 Sony Corporation
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

#include <sdk/config.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <debug.h>

#include <arch/board/board.h>
#include <arch/chip/cxd56_audio.h>

#include "cxd56_clock.h"

#include "audio/cxd56_audio_config.h"
#include "audio/cxd56_audio_power.h"
#include "audio/cxd56_audio_filter.h"
#include "audio/cxd56_audio_mic.h"
#include "audio/cxd56_audio_volume.h"
#include "audio/cxd56_audio_digital.h"
#include "audio/cxd56_audio_beep.h"
#include "audio/cxd56_audio_irq.h"
#include "audio/cxd56_audio_dma.h"
#include "audio/cxd56_audio_pin.h"
#include "audio/cxd56_audio_analog.h"
#include "audio/cxd56_audio_ac_reg.h"
#include "audio/cxd56_audio_bca_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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

CXD56_AUDIO_ECODE cxd56_audio_poweron(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check status */

  /* Set global pin. */

  cxd56_audio_pin_set();

  /* Initialize config. */

  cxd56_audio_config_init();

  /* Power On analog block. */

  ret = cxd56_audio_analog_poweron();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  /* Power On audio codec block. */

  ret = cxd56_audio_power_on();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  /* Enable interrupt. */

  cxd56_audio_irq_attach();
  cxd56_audio_irq_enable();

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_poweroff(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check status */

  /* Power off audio codec block. */

  ret = cxd56_audio_power_off();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  /* Power off analog block. */

  ret = cxd56_audio_analog_poweroff();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  /* Disable interrupt. */

  cxd56_audio_irq_disable();
  cxd56_audio_irq_detach();

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_en_cstereo(bool sign_inv, int16_t vol)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_filter_set_cstereo(true, sign_inv, vol);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_dis_cstereo(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_filter_set_cstereo(false, false, 0);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_poweron_dnc(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check status */

  cxd56_audio_filter_poweron_dnc();

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_poweroff_dnc(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check status */

  cxd56_audio_filter_poweroff_dnc();

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_en_dnc(cxd56_audio_dnc_id_t id,
                                     FAR cxd56_audio_dnc_bin_t *bin)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check status */

  cxd56_audio_filter_set_dnc(id, true, bin);

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_dis_dnc(cxd56_audio_dnc_id_t id)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check status */

  cxd56_audio_filter_set_dnc(id, false, NULL);

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_en_deq(FAR cxd56_audio_deq_coef_t *deq)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check status */

  cxd56_audio_filter_set_deq(true, deq);

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_dis_deq(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check status */

  cxd56_audio_filter_set_deq(false, NULL);

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_en_input(FAR cxd56_audio_mic_gain_t *gain)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

#ifdef CONFIG_CXD56_AUDIO_ANALOG_NONE
  return CXD56_AUDIO_ECODE_MIC_NO_ANA;
#endif

  if (gain == NULL)
    {
      return CXD56_AUDIO_ECODE_MIC_ARG_NULL;
    }

  ret = cxd56_audio_analog_poweron_input(gain);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  ret = cxd56_audio_mic_enable(gain);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_en_output(bool sp_out_en)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

#ifndef CONFIG_CXD56_AUDIO_ANALOG_NONE
  cxd56_audio_clkmode_t clk_mode = cxd56_audio_config_get_clkmode();

  if (sp_out_en)
    {
      ret = cxd56_audio_analog_poweron_output();
    }
  else
    {
      ret = cxd56_audio_analog_poweroff_output();
    }
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  /* Enable S-Master. */

  cxd56_audio_ac_reg_poweron_smaster(clk_mode);
  cxd56_audio_bca_reg_set_smaster();
  cxd56_audio_ac_reg_enable_smaster();
#endif

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_dis_input(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_mic_disable();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  ret = cxd56_audio_analog_poweroff_input();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_dis_output(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;
#ifndef CONFIG_CXD56_AUDIO_ANALOG_NONE
  cxd56_audio_ac_reg_disable_smaster();

  /* Mute output. */

  ret = cxd56_audio_volume_mute(CXD56_AUDIO_VOLID_MIXER_OUT);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  cxd56_audio_ac_reg_poweroff_smaster();

  ret = cxd56_audio_analog_poweroff_output();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }
#endif
  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_set_vol(cxd56_audio_volid_t id, int16_t vol)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_volume_set(id, vol);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_mute_vol(cxd56_audio_volid_t id)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_volume_mute(id);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_unmute_vol(cxd56_audio_volid_t id)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_volume_unmute(id);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_mute_vol_fade(cxd56_audio_volid_t id, bool wait)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_volume_mute_fade(id, wait);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_unmute_vol_fade(cxd56_audio_volid_t id, bool wait)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_volume_unmute_fade(id, wait);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_set_beep_freq(uint16_t freq)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_beep_set_freq(freq);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_set_beep_vol(int16_t vol)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_beep_set_vol(vol);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_play_beep(void)
{
 CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  cxd56_audio_beep_play();

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_stop_beep(void)
{
 CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  cxd56_audio_beep_stop();

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_set_micgain(FAR cxd56_audio_mic_gain_t *gain)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  if (gain == NULL)
    {
      return CXD56_AUDIO_ECODE_MIC_ARG_NULL;
    }

  /* Set analog mic gain. */

  ret = cxd56_audio_analog_set_micgain(gain);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  /* Set digital mic gain. */

  ret = cxd56_audio_mic_set_gain(gain);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_set_deq(bool en, FAR cxd56_audio_deq_coef_t *deq)
{
  cxd56_audio_filter_set_deq(en, deq);

  return CXD56_AUDIO_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_get_dmahandle(cxd56_audio_dma_path_t path,
                                            FAR cxd56_audio_dma_t *handle)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check error of argument, state */

  if (handle == NULL)
    {
      return CXD56_AUDIO_ECODE_DMA_ARG_NULL;
    }

  ret = cxd56_audio_dma_get_handle(path, handle);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  if (CXD56_AUDIO_DMA_PATH_MIC_TO_MEM == path)
    {
      ret = cxd56_audio_analog_wait_input_standby();
      if (CXD56_AUDIO_ECODE_OK != ret)
        {
          return ret;
        }
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_free_dmahandle(FAR cxd56_audio_dma_t handle)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_dma_free_handle(handle);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_set_datapath(cxd56_audio_signal_t sig,
                                           cxd56_audio_sel_t sel)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_ac_reg_set_selector(sig, sel);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  if (CXD56_AUDIO_SIG_MIC1 == sig ||
      CXD56_AUDIO_SIG_MIC2 == sig ||
      CXD56_AUDIO_SIG_MIC3 == sig ||
      CXD56_AUDIO_SIG_MIC4 == sig)
    {
      ret = cxd56_audio_analog_wait_input_standby();
      if (CXD56_AUDIO_ECODE_OK != ret)
        {
          return ret;
        }
    }
  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_init_dma(cxd56_audio_dma_t handle,
                                       cxd56_audio_samp_fmt_t fmt,
                                       FAR uint8_t *ch_num)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  if (ch_num == NULL)
    {
      return CXD56_AUDIO_ECODE_DMA_ARG_NULL;
    }

  ret = cxd56_audio_dma_init(handle, fmt, ch_num);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_set_dmacb(cxd56_audio_dma_t handle,
                                        FAR cxd56_audio_dma_cb_t cb)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  if (cb == NULL)
    {
      return CXD56_AUDIO_ECODE_DMA_ARG_NULL;
    }

  ret = cxd56_audio_dma_set_cb(handle, cb);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_get_dmamstate(cxd56_audio_dma_t handle,
                                            FAR cxd56_audio_dma_mstate_t *state)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  if (state == NULL)
    {
      return CXD56_AUDIO_ECODE_DMA_ARG_NULL;
    }

  ret = cxd56_audio_dma_get_mstate(handle, state);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_en_dmaint(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_dma_en_dmaint();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return CXD56_AUDIO_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_dis_dmaint(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_dma_dis_dmaint();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return CXD56_AUDIO_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_clear_dmaerrint(cxd56_audio_dma_t handle)
{
  cxd56_audio_bca_reg_clear_err_int(handle);

  return CXD56_AUDIO_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_mask_dmaerrint(cxd56_audio_dma_t handle)
{
  cxd56_audio_bca_reg_mask_err_int(handle);

  return CXD56_AUDIO_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_unmask_dmaerrint(cxd56_audio_dma_t handle)
{
  cxd56_audio_bca_reg_unmask_err_int(handle);

  return CXD56_AUDIO_ECODE_OK;
}

/*--------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_start_dma(cxd56_audio_dma_t handle,
                                        uint32_t addr,
                                        uint32_t sample)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_dma_start(handle, addr, sample);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return CXD56_AUDIO_ECODE_OK;
}

/*--------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_stop_dma(cxd56_audio_dma_t handle)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_dma_stop(handle);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return CXD56_AUDIO_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_set_clkmode(cxd56_audio_clkmode_t mode)
{
  if ((CXD56_AUDIO_CFG_MCLK == CXD56_AUDIO_CFG_XTAL_24_576MHZ) &&
      (mode == CXD56_AUDIO_CLKMODE_HIRES))
    {
      return CXD56_AUDIO_ECODE_CFG_CLK_MODE;
    }

  cxd56_audio_config_set_clkmode(mode);

  return CXD56_AUDIO_ECODE_OK;
}

/*--------------------------------------------------------------------------*/
cxd56_audio_clkmode_t cxd56_audio_get_clkmode(void)
{
  return cxd56_audio_config_get_clkmode();
}

/*--------------------------------------------------------------------------*/
cxd56_audio_dmafmt_t cxd56_audio_get_dmafmt(void)
{
  cxd56_audio_dmafmt_t fmt;

  if (CXD56_AUDIO_CFG_DMA_FORMAT == CXD56_AUDIO_CFG_DMA_FORMAT_LR)
    {
      fmt = CXD56_AUDIO_DMA_FMT_LR;
    }
  else
    {
      fmt = CXD56_AUDIO_DMA_FMT_RL;
    }

  return fmt;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_en_digsft(cxd56_audio_dsr_rate_t rate)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  cxd56_audio_ac_reg_set_dsrrate(rate);
  cxd56_audio_ac_reg_enable_digsft();

  return ret;
}

/*--------------------------------------------------------------------------*/
CXD56_AUDIO_ECODE cxd56_audio_dis_digsft(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  cxd56_audio_ac_reg_disable_digsft();

  return ret;
}
