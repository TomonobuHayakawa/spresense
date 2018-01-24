/****************************************************************************
 * nuttx/arch/arm/src/cxd56xx/audio/drivers/baseband/src/cxd56_audio_drv.c
 *
 *   Copyright (C) 2016, 2017 Sony Corporation
 *   Author: Naoya Haneda <Naoya.Haneda@sony.com>
 *           Tomonobu Hayakawa<Tomonobu.Hayakawa@sony.com>
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
#include <debug.h>

#include <arch/board/board.h>
#include <arch/chip/cxd56_audio.h>

#include "cxd56_clock.h"
#include "audio/audio_io_config.h"
#include "audio/ac_drv.h"
#include "audio/aca_drv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool poweron_common = false;
static bool poweron_input  = false;
static bool poweron_output = false;

/****************************************************************************
 * Public Data
 ****************************************************************************/

asAcaPulcoParam     sAcaPulcoParam;
asSerDesParam       sAcaPulcoSdesParam;
asAcaPulcoInParam   sAcaPulcoInParam;
asAcaPulcoOutParam  sAcaPulcoOutParam;
asSerDesParam       sSdesParam;
asI2sParam          sI2sParam;
static asSrcParam   sSrcParam[AS_I2S_ID_NUM];
asSrcSelId          gSrcId;

BaseBandConfigTbl  bb_config_tbl =
{
#if defined(CONFIG_CXD56_AUDIO_MICBIAS_20V)
  0,                                   /* mic_bias_sel */
#else
  1,                                   /* mic_bias_sel */
#endif

  0,                                   /* reserved */

#if defined(CONFIG_CXD56_AUDIO_CLK_MODE_HIRES)
  AS_CLK_MODE_HIRES,                   /* clk_mode */
#else
  AS_CLK_MODE_NORMAL,                  /* clk_mode */
#endif

#if defined(CONFIG_CXD56_AUDIO_XTAL_SEL_49_152MHZ)
  AS_XTAL_49_152MHZ,                   /* xtal_sel */
#else
  AS_XTAL_24_576MHZ,                   /* xtal_sel */
#endif

  CONFIG_CXD56_AUDIO_MIC_CHANNEL_SEL,  /* mic_channel_sel */

#if defined(CONFIG_CXD56_AUDIO_GPO_A_WEAKEST)
  AS_IO_DS_WEAKEST,                    /* gpo_ds */
#elif defined(CONFIG_CXD56_AUDIO_GPO_A_WEAKER)
  AS_IO_DS_WEAKER,                     /* gpo_ds */
#elif defined(CONFIG_CXD56_AUDIO_GPO_A_STRONGER)
  AS_IO_DS_STRONGER,                   /* gpo_ds */
#else
  AS_IO_DS_STRONGEST,                  /* gpo_ds */
#endif

#if defined(CONFIG_CXD56_AUDIO_DA_DATA_WEAKEST)
  AS_IO_DS_WEAKEST,                    /* ad_data_ds */
#elif defined(CONFIG_CXD56_AUDIO_DA_DATA_WEAKER)
  AS_IO_DS_WEAKER,                     /* ad_data_ds */
#elif defined(CONFIG_CXD56_AUDIO_DA_DATA_STRONGER)
  AS_IO_DS_STRONGER,                   /* ad_data_ds */
#else
  AS_IO_DS_STRONGEST,                  /* ad_data_ds */
#endif

#if defined(CONFIG_CXD56_AUDIO_CLKOUT_DMIC_WEAKEST)
  AS_IO_DS_WEAKEST,                    /* dmic_clk_ds */
#elif defined(CONFIG_CXD56_AUDIO_CLKOUT_DMIC_WEAKER)
  AS_IO_DS_WEAKER,                     /* dmic_clk_ds */
#elif defined(CONFIG_CXD56_AUDIO_CLKOUT_DMIC_STRONGER)
  AS_IO_DS_STRONGER,                   /* dmic_clk_ds */
#else
  AS_IO_DS_STRONGEST,                  /* dmic_clk_ds */
#endif

#if defined(CONFIG_CXD56_AUDIO_MCLKOUT_WEAKEST)
  AS_IO_DS_WEAKEST,                    /* mclk_ds */
#elif defined(CONFIG_CXD56_AUDIO_MCLKOUT_WEAKER)
  AS_IO_DS_WEAKER,                     /* mclk_ds */
#elif defined(CONFIG_CXD56_AUDIO_MCLKOUT_STRONGER)
  AS_IO_DS_STRONGER,                   /* mclk_ds */
#else
  AS_IO_DS_STRONGEST,                  /* mclk_ds */
#endif

#if defined(CONFIG_CXD56_AUDIO_I2S_DATA_PATH_NONE)
  AS_I2S_DEVICE_I2S_SLAVE,             /* i2s_device_1 */
  AS_I2S_FORMAT_I2S,                   /* i2s_format_1 */
#else
#  if defined(CONFIG_CXD56_AUDIO_I2S_DEVICE_1_SLAVE)
     AS_I2S_DEVICE_I2S_SLAVE,          /* i2s_device_1 */
#  else
     AS_I2S_DEVICE_I2S_MASTER,         /* i2s_device_1 */
#  endif

#  if defined(CONFIG_CXD56_AUDIO_I2S_FORMAT_1_LEFT)
     AS_I2S_FORMAT_LEFT,               /* i2s_format_1 */
#  else
     AS_I2S_FORMAT_I2S,                /* i2s_format_1 */
#  endif
#endif

  0,                                    /* reserved */

#if defined(CONFIG_CXD56_AUDIO_I2S_DATA_PATH_NONE)
  AS_I2S_DATA_PATH_NONE,               /* i2s_data_path */
#elif defined(CONFIG_CXD56_AUDIO_I2S_DATA_PATH_1)
  AS_I2S_DATA_PATH_1,                  /* i2s_data_path */
#else
  AS_I2S_DATA_PATH_2,                  /* i2s_data_path */
#endif

#if defined(CONFIG_CXD56_AUDIO_I2S_DATA_PATH_2)
#  if defined(CONFIG_CXD56_AUDIO_I2S_DEVICE_2_SLAVE)
     AS_I2S_DEVICE_I2S_SLAVE,          /* i2s_device_2 */
#  else
    AS_I2S_DEVICE_I2S_MASTER,          /* i2s_device_2 */
#  endif

#  if defined(CONFIG_CXD56_AUDIO_I2S_FORMAT_2_LEFT)
     AS_I2S_FORMAT_LEFT,               /* i2s_format_2 */
#  else
     AS_I2S_FORMAT_I2S,                /* i2s_format_2 */
#  endif
#else
  AS_I2S_DEVICE_I2S_SLAVE,             /* i2s_device_2 */
  AS_I2S_FORMAT_I2S,                   /* i2s_format_2 */
#endif

  CONFIG_CXD56_AUDIO_MIC_BOOT_WAIT,    /* mic_boot_wait */

#if defined(CONFIG_CXD56_AUDIO_PDM_LOWEMI_2MA)
  AS_LOWEMI_2MA,                       /* pdm_lowemi */
#else
  AS_LOWEMI_4MA,                       /* pdm_lowemi */
#endif

#if defined(CONFIG_CXD56_AUDIO_I2S_LOWEMI_2MA)
  AS_LOWEMI_2MA,                       /* i2s_lowemi */
#else
  AS_LOWEMI_4MA,                       /* i2s_lowemi */
#endif

#if defined(CONFIG_CXD56_AUDIO_CIC_IN_SEL_CXD)
  AS_CIC_IN_SEL_CXD,                   /* cic_input_sel */
#elif defined (CONFIG_CXD56_AUDIO_CIC_IN_SEL_DMIC)
  AS_CIC_IN_SEL_DMIC,                  /* cic_input_sel */
#else
  AS_CIC_IN_SEL_NONE,                  /* cic_input_sel */
#endif

  0,                                   /* reserved */

#if defined(CONFIG_CXD56_AUDIO_ALC_SPC_SEL_OFF)
  AS_ALC_SPC_SEL_OFF,                  /* alc_spc_sel */
#elif defined(CONFIG_CXD56_AUDIO_ALC_SPC_SEL_ALC)
  AS_ALC_SPC_SEL_ALC,                  /* alc_spc_sel */
#else
  AS_ALC_SPC_SEL_SPC,                  /* alc_spc_sel */
#endif

  0,                                   /* reserved */

#if defined(CONFIG_CXD56_AUDIO_ALC_SPC_SEL_SPC)
  CONFIG_CXD56_AUDIO_SPC_LIMIT,        /* spc_limit */
#else
  0,                                   /* spc_limit */
#endif

#if defined(CONFIG_CXD56_AUDIO_ALC_SPC_SEL_ALC)
  CONFIG_CXD56_AUDIO_ALC_TARGET,       /* alc_target */
  CONFIG_CXD56_AUDIO_ALC_KNEE,         /* alc_knee */
#else
  0,                                   /* alc_target */
  0,                                   /* alc_knee */
#endif

  0,                                   /* reserved */

#if defined(CONFIG_CXD56_AUDIO_DMA_DATA_FORMAT_LR)
  AS_DMA_DATA_FORMAT_LR,               /* dma_data_format */
#else
  AS_DMA_DATA_FORMAT_RL,               /* dma_data_format */
#endif

#if defined(CONFIG_CXD56_AUDIO_HPADC_MIC_BIAS_ON)
  AS_HPADC_MIC_BIAS_ON,                /* hpadc_mic_bias */
#else
  AS_HPADC_MIC_BIAS_OFF,               /* hpadc_mic_bias */
#endif

  0,                                   /* reserved */

  0,                                   /* sp_delay */
  0,                                   /* loop_mode */
  2,                                   /* pwm_mode */
  0,                                   /* sp_delay_free */

#if defined(CONFIG_CXD56_AUDIO_SP_SPLIT_LONGEST)
  AS_ACA_SP_SPLITON_SEL_LONGEST,       /* sp_split */
#elif defined(CONFIG_CXD56_AUDIO_SP_SPLIT_LONG)
  AS_ACA_SP_SPLITON_SEL_LONG,          /* sp_split */
#elif defined(CONFIG_CXD56_AUDIO_SP_SPLIT_SHORT)
  AS_ACA_SP_SPLITON_SEL_SHORT,         /* sp_split */
#else
  AS_ACA_SP_SPLITON_SEL_SHORTEST,      /* sp_split */
#endif

#if defined(CONFIG_CXD56_AUDIO_SP_DRV_LINEOUT)
  AS_ACA_SP_DRV_SEL_LINEOUT,           /* sp_drive */
#elif defined (CONFIG_CXD56_AUDIO_SP_DRV_1DRIVERT)
  AS_ACA_SP_DRV_SEL_1DRIVER,           /* sp_drive */
#elif defined (CONFIG_CXD56_AUDIO_SP_DRV_2DRIVERT)
  AS_ACA_SP_DRV_SEL_2DRIVER,           /* sp_drive */
#else
  AS_ACA_SP_DRV_SEL_4DRIVER,           /* sp_drive */
#endif
  0,                                   /* reserved */
  0,                                   /* reserved */
};

FAR BaseBandConfigTbl *bb_config_tblp   = &bb_config_tbl;
BaseBandConfigAddTbl  bb_config_add_tbl = {0};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void AS_AudioIntHandler(void);

E_AS AS_PowerOnBaseBand(uint32_t rate[AS_I2S_ID_NUM],
                        asBypassModeId bypass_mode_en[AS_I2S_ID_NUM])
{
  E_AS rtCode = E_AS_OK;
  if (poweron_common)
    {
      return E_AS_OK;
    }

  board_aca_power_control(CXD5247_DVDD, true);
  if (!board_aca_power_monitor(CXD5247_DVDD))
    {
      return E_AS_PM_ADONIS_PWON_CHK_ERR;
    }
  cxd56_audio_clock_enable(AUD_MCLK_EXT, 0);
  if (!cxd56_audio_clock_is_enabled())
    {
      return E_AS_PM_AUDIO_PWON_CHK_ERR;
    }
  setAudioIoMclk();

  rtCode = GetBaseBandConfigParam();
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }

#ifndef CONFIG_CXD56_AUDIO_ANALOG_NONE
  rtCode = PowerOnAcaPulco();
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }
#endif

  rtCode = PowerOnAudioCodec(rate, bypass_mode_en);
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }

  irq_attach(CXD56_IRQ_AUDIO_0, (xcpt_t)AS_AudioIntHandler, NULL);
  irq_attach(CXD56_IRQ_AUDIO_1, (xcpt_t)AS_AudioIntHandler, NULL);
  irq_attach(CXD56_IRQ_AUDIO_2, (xcpt_t)AS_AudioIntHandler, NULL);
  irq_attach(CXD56_IRQ_AUDIO_3, (xcpt_t)AS_AudioIntHandler, NULL);

  poweron_common = true;
  return rtCode;
}

E_AS AS_PowerOffBaseBand(void)
{
  E_AS rtCode = E_AS_OK;

  if (!poweron_common)
    {
      return E_AS_OK;
    }
  if (poweron_input || poweron_output)
    {
      return E_AS_POWER_OFF_CHK_ERR;
    }

  asAc_PowerOffAudioCodec();

#ifndef CONFIG_CXD56_AUDIO_ANALOG_NONE
  rtCode = AS_AcaControl(AS_ACA_POWER_OFF_COMMON, (uint32_t)NULL);
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }
#endif

  irq_detach(CXD56_IRQ_AUDIO_0);
  irq_detach(CXD56_IRQ_AUDIO_1);
  irq_detach(CXD56_IRQ_AUDIO_2);
  irq_detach(CXD56_IRQ_AUDIO_3);

  cxd56_audio_clock_disable();
  board_aca_power_control(CXD5247_DVDD, false);

  poweron_common = false;
  return rtCode;
}

E_AS AS_BaseBandEnable_input(asMicMode micMode, int32_t micGain[8])
{
  E_AS rtCode = E_AS_OK;

  if (poweron_input)
    {
      return E_AS_OK;
    }

  if (micMode == AS_MICMODE_ACAPULCO)
    {
      if(!poweron_common)
        {
          return E_AS_IN_POWER_ON_CHK_ERR;
        }
    if (micGain == NULL)
      {
        return E_AS_MICGAIN_NULL;
      }

    if ((bb_config_add_tbl.micDev == AS_ACA_MIC_AMIC) ||
        (bb_config_add_tbl.micDev == AS_ACA_MIC_BOTH))
      {
        board_aca_power_control(CXD5247_AVDD, true);
      }

#ifndef CONFIG_CXD56_AUDIO_ANALOG_NONE
    rtCode = EnableAcaPulcoInput(micGain);
    if (E_AS_OK != rtCode)
      {
        return rtCode;
      }
#else
    return E_AS_ACAPULCO_ID_NG;
#endif

    rtCode = EnableAudioCodecInput(micGain);
    if (E_AS_OK != rtCode)
      {
        return rtCode;
      }
    }
  else if(micMode == AS_MICMODE_HPADC)
    {
      rtCode = GetBaseBandConfigParam();
      if (E_AS_OK != rtCode)
        {
          return rtCode;
        }

      board_aca_power_control(CXD5247_AVDD|CXD5247_DVDD, true);

#ifndef CONFIG_CXD56_AUDIO_ANALOG_NONE
      rtCode = PowerOnHpadcMicBias();
      if (E_AS_OK != rtCode)
        {
          return rtCode;
        }
#else
      return E_AS_ACAPULCO_ID_NG;
#endif
    }
  else
    {
      return E_AS_MICMODE_PARAM;
    }

  poweron_input = true;
  return rtCode;
}

E_AS AS_BaseBandEnable_output(asOutDeviceId devid)
{
  E_AS rtCode = E_AS_OK;
  if (poweron_output)
    {
      return E_AS_OK;
    }
  if (!poweron_common)
    {
      return E_AS_OUT_POWER_ON_CHK_ERR;
    }

  if (devid != AS_OUT_DEV_OFF)
    {
      board_aca_power_control(CXD5247_AVDD, true);
    }
  bb_config_add_tbl.output_device_sel = devid;
  bb_config_add_tbl.sp_offon = 0;

  if (devid == AS_OUT_DEV_SP)
    {
#ifndef CONFIG_CXD56_AUDIO_ANALOG_NONE
      rtCode = EnableAcaPulcoOutput();
      if (E_AS_OK != rtCode)
        {
          return rtCode;
        }
#else
      return E_AS_OUT_DEVICE_SEL_PARAM;
#endif
    }

  rtCode = EnableAudioCodecOutput();
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }

  if (devid == AS_OUT_DEV_I2S)
    {
      setAcOutputI2S();
    }
  poweron_output = true;
  return rtCode;
}

E_AS AS_BaseBandDisable_input(asMicMode micMode)
{
  E_AS rtCode = E_AS_OK;
  if (!poweron_input)
    {
      return E_AS_OK;
    }

  if (micMode == AS_MICMODE_ACAPULCO)
    {
      asAc_PowerOffAudioCodecInput();

      rtCode = AS_AcaControl(AS_ACA_POWER_OFF_INPUT, (uint32_t)NULL);
      if (E_AS_OK != rtCode)
        {
          return rtCode;
        }

      if (!poweron_output)
        {
          board_aca_power_control(CXD5247_AVDD, false);
        }
    }
  else if(micMode == AS_MICMODE_HPADC)
    {
#ifndef CONFIG_CXD56_AUDIO_ANALOG_NONE
      rtCode = PowerOffHpadcMicBias();
      if (E_AS_OK != rtCode)
        {
          return rtCode;
        }
#else
      return E_AS_ACAPULCO_ID_NG;
#endif

      board_aca_power_control(CXD5247_AVDD | CXD5247_DVDD, false);
    }
  else
    {
      return E_AS_MICMODE_PARAM;
    }

  poweron_input = false;
  return rtCode;
}

E_AS AS_BaseBandDisable_output(void)
{
  E_AS rtCode = E_AS_OK;
  if (!poweron_output)
    {
      return E_AS_OK;
    }

  rtCode = asAc_PowerOffAudioCodecOutput();
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }

  rtCode = AS_AcaControl(AS_ACA_POWER_OFF_OUTPUT, (uint32_t)NULL);
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }

  if (!poweron_input)
    {
      board_aca_power_control(CXD5247_AVDD, false);
    }

  poweron_output = false;
  return rtCode;
}

bool chkPowerOnBaseBand(void)
{
  return poweron_common;
}

bool chkEnableBaseBandInput(void)
{
  return poweron_input;
}

bool chkEnableBaseBandOutput(void)
{
  return poweron_output;
}

E_AS AS_InitClearStereo(asCsEn csEn, asCsSignId csSign, int32_t csVol)
{
  E_AS rtCode = E_AS_OK;

  if (!poweron_common)
    {
      return E_AS_CS_POWER_ON_CHK_ERR;
    }

  if ((csSign != AS_CS_SIGN_POSITIVE) && (csSign != AS_CS_SIGN_NEGATIVE))
    {
      return E_AS_CS_SIGN_PARAM;
    }

  if (csEn == AS_CS_ENABLE)
    {
      rtCode = asAc_SetClearStereo(csSign, csVol);
      _info("asAc_SetClearStereo(%d)\n", rtCode);
    }
  else if(csEn == AS_CS_DISABLE)
    {
      rtCode = asAc_SetClearStereo(csSign, AS_CS_VOL_INF_ZERO);
      _info("asAc_SetClearStereo(%d)\n", rtCode);
    }
  else
    {
      return E_AS_CS_EN_PARAM;
    }
  return rtCode;
}

E_AS AS_SetDeqCoef(bool deqEn, FAR asDeqParam *pDeqParam)
{
  if (!poweron_common)
    {
      return E_AS_DEQ_POWER_ON_CHK_ERR;
    }

  return E_AS_DEQ_UNSUPPORT;
}

E_AS AS_SetDncParam(asDncMode dncMode)
{
  if (!poweron_common)
    {
      return E_AS_DNC_POWER_ON_CHK_ERR;
    }

  return E_AS_DNC_UNSUPPORT;
}

E_AS AS_SetMicGain(int32_t micGain[AS_MIC_CHANNEL_MAX])
{
  E_AS rtCode = E_AS_OK;

  if (!poweron_input)
    {
      return E_AS_MIC_POWER_ON_CHK_ERR;
    }

  rtCode = GetAcaPulcoInParam(micGain);
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  if (bb_config_tblp->cic_input_sel == AS_CIC_IN_SEL_CXD)
    {
      if (bb_config_add_tbl.micDev == AS_ACA_MIC_UNKNOWN)
        {
          return E_AS_MIC_CHANNEL_SEL_PARAM;
        }
    }
  else if(bb_config_tblp->cic_input_sel == AS_CIC_IN_SEL_DMIC)
    {
      if(bb_config_add_tbl.micDev != AS_ACA_MIC_DMIC)
        {
          return E_AS_MIC_CHANNEL_SEL_PARAM;
        }
    }
  else
    {
      return E_AS_MIC_CHANNEL_SEL_PARAM;
    }

  rtCode = AS_AcaControl(AS_ACA_INIT_AMIC, (uint32_t)&sAcaPulcoInParam);
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  uint8_t cicNum = (bb_config_add_tbl.micNum + 1) / AS_AC_CIC_MIC_CH_NUM;
  asCicSelId cicId = AS_CIC_SEL_CIC1;
  asCicParam cicParam;

  for (uint8_t cicCnt = 0; cicCnt < cicNum; cicCnt++)
    {
      cicId = (asCicSelId)cicCnt;
      rtCode = GetCicParam(&micGain[cicCnt * AS_AC_CIC_MIC_CH_NUM],
                           cicId,
                           &cicParam);
      if (rtCode != E_AS_OK)
        {
          return rtCode;
        }
      rtCode = setSelCicGain(cicId, &cicParam);
      if (rtCode != E_AS_OK)
        {
          return rtCode;
        }
    }

  return rtCode;
}

E_AS AS_SetOutputSelect(asOutDeviceId devid)
{
  E_AS rtCode = E_AS_OK;
  if (!poweron_output)
    {
      return E_AS_OUTSEL_POWER_ON_CHK_ERR;
    }
  if (devid >= AS_OUT_DEV_NUM)
    {
      return E_AS_OUT_DEVICE_SEL_PARAM;
    }
  if (devid == AS_OUT_DEV_OFF)
    {
      if(bb_config_add_tbl.output_device_sel != AS_OUT_DEV_OFF)
        {
          rtCode = AS_MuteVolume(AS_VOLUME_MASTER);
          D_ASSERT(rtCode == E_AS_OK);
          rtCode = AS_BeepDisable();
          D_ASSERT(rtCode == E_AS_OK);
          board_aca_power_control(CXD5247_AVDD, false);
        }
    }
  else
    {
      board_aca_power_control(CXD5247_AVDD, true);
    }
  bb_config_add_tbl.output_device_sel = devid;
  if (devid == AS_OUT_DEV_I2S)
    {
      setAcOutputI2S();
    }
  return rtCode;
}

E_AS AS_SetI2sParam(uint32_t rate[AS_I2S_ID_NUM],
                    asBypassModeId bypass_mode_en[AS_I2S_ID_NUM])
{
  if (!poweron_common)
    {
      return E_AS_I2S_POWER_ON_CHK_ERR;
    }
  E_AS rtCode = GetI2sParam(rate, bypass_mode_en);
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  if (bb_config_tblp->i2s_data_path == AS_I2S_DATA_PATH_NONE)
    {
      return E_AS_I2S_DATA_PATH_PARAM;
    }

  rtCode = asAc_SetSrcParam(gSrcId, &sI2sParam);

  return rtCode;
}

E_AS setMicChSel(uint8_t micNum, asSampFmt format) {
  E_AS rtCode = E_AS_OK;

  asSerDesParam     sdesParam;

  D_ASSERT(bb_config_tblp != NULL);
  uint8_t micCh = 0;
  if ((format == AS_SAMPLING_FMT_16) &&
      (bb_config_tblp->dma_data_format == AS_DMA_DATA_FORMAT_RL))
    {
      for (micCh = 0; micCh < AS_MIC_CHANNEL_MAX; micCh++)
        {
          if (micCh % 2 == 0)
            {
              sdesParam.selCh.out[micCh] =
                        (asSdesDesSelOutId)(AS_SDES_DES_SEL_CH1 + micCh + 1);
            }
          else
            {
              sdesParam.selCh.out[micCh] =
                        (asSdesDesSelOutId)(AS_SDES_DES_SEL_CH1 + micCh - 1);
            }
        }
    }
  else
    {
      for (micCh = 0; micCh < AS_MIC_CHANNEL_MAX; micCh++)
        {
          sdesParam.selCh.out[micCh] =
                          (asSdesDesSelOutId)(AS_SDES_DES_SEL_CH1 + micCh);
        }
    }
  if ((format == AS_SAMPLING_FMT_16) &&
      ((micNum % 2) == 1))
    {
      if (bb_config_tblp->dma_data_format == AS_DMA_DATA_FORMAT_LR)
        {
          sdesParam.selCh.out[micNum] = sdesParam.selCh.out[micNum - 1];
        }
      else
        {
          sdesParam.selCh.out[micNum - 1] = sdesParam.selCh.out[micNum];
        }
    }

  rtCode = setAcDesOut(&sdesParam);
  return rtCode;
}

E_AS chkMicGainParam(int16_t micGain[AS_MIC_CHANNEL_MAX])
{
  E_AS rtCode = E_AS_OK;

  if (bb_config_tblp == NULL)
    {
      rtCode = GetBaseBandConfigParam();
      if (E_AS_OK != rtCode)
        {
          return rtCode;
        }
    }

  uint8_t mic_sel = 0;
  for (uint8_t micCh = 0; micCh < AS_MIC_CHANNEL_MAX; micCh++)
    {
      mic_sel = (bb_config_tblp->mic_channel_sel >> (micCh * MIC_CH_BITNUM)) &
                MIC_CH_BITMAP;
      if (micGain[micCh] == AS_MICGAIN_MUTE)
        {
          continue;
        }
      if ((mic_sel >= AS_ACA_SER_SEL_AMIC1) &&
          (mic_sel <= AS_ACA_SER_SEL_AMIC4))
        {
          if (!CHECK_RANGE(micGain[micCh],
                           AS_MICGAIN_HOLD,
                           AS_MICGAINA_MIN,
                           AS_MICGAINA_MAX))
            {
              return E_AS_MICGAIN_PARAM;
            }
        }
      else if ((mic_sel >= AS_ACA_SER_SEL_DMIC1) &&
               (mic_sel <= AS_ACA_SER_SEL_DMIC8))
        {
          if (!CHECK_RANGE(micGain[micCh],
                           AS_MICGAIN_HOLD,
                           AS_MICGAIND_MIN,
                           AS_MICGAIND_MAX))
            {
              return E_AS_MICGAIN_PARAM;
            }
        }
      else
        {
          /* Do nothing. */
        }
    }

  return rtCode;
}

E_AS chkI2sParam(uint32_t rate[AS_I2S_ID_NUM],
                 asBypassModeId bypass_mode_en[AS_I2S_ID_NUM])
{
  E_AS rtCode = E_AS_OK;

  if (bb_config_tblp == NULL)
    {
      rtCode = GetBaseBandConfigParam();
      if (E_AS_OK != rtCode)
        {
          return rtCode;
        }
    }

  for (uint8_t i2s_cnt = 0; i2s_cnt < AS_I2S_ID_NUM; i2s_cnt++)
    {
      if((i2s_cnt == 0) &&
         (bb_config_tblp->i2s_data_path == AS_I2S_DATA_PATH_NONE))
        {
          break;
        }
      else if ((i2s_cnt == 1) &&
               (bb_config_tblp->i2s_data_path != AS_I2S_DATA_PATH_2))
        {
          break;
        }
      if (bypass_mode_en[i2s_cnt] == AS_I2S_BP_MODE_ENABLE)
        {
          if (i2s_cnt == 0)
            {
              if ((bb_config_tblp->i2s_device_1 !=
                   AS_I2S_DEVICE_I2S_MASTER) ||
                  (rate[i2s_cnt] <= 96000))
                {
                  return E_AS_SRC_BYPASS_ERR;
                }
            }
          else
            {
              if ((bb_config_tblp->i2s_device_2 !=
                   AS_I2S_DEVICE_I2S_MASTER) ||
                  (rate[i2s_cnt] <= 96000))
                {
                  return E_AS_SRC_BYPASS_ERR;
                }
            }
        }
    }
  return rtCode;
}


/* TODO: Followin GetXX functions can be moved to other files. */

E_AS GetBaseBandConfigParam(void)
{
  uint8_t amic_flg = 0;
  uint8_t dmic_flg = 0;
  uint8_t mic_num  = 0;
  for (uint8_t i = 0; i < 8; i++)
    {
      uint8_t mic_sel;
      mic_sel = (bb_config_tblp->mic_channel_sel >> (i * MIC_CH_BITNUM)) &
                MIC_CH_BITMAP;
      if ((mic_sel >= 1) && (mic_sel <= 4))
        {
          amic_flg = 1;
          mic_num++;
        }
      else if ((mic_sel >= 5) && (mic_sel <= 12))
        {
          dmic_flg = 1;
          mic_num++;
        }
    }
  bb_config_add_tbl.micNum = mic_num;
  if (amic_flg == 1)
    {
      if(dmic_flg == 1)
        {
          bb_config_add_tbl.micDev = AS_ACA_MIC_BOTH;
          bb_config_add_tbl.ser_mode = AS_SER_MODE_64FS;
        }
      else
        {
          bb_config_add_tbl.micDev = AS_ACA_MIC_AMIC;
          bb_config_add_tbl.ser_mode = AS_SER_MODE_128FS;
        }
    }
  else
    {
      if(dmic_flg == 1)
        {
          bb_config_add_tbl.micDev = AS_ACA_MIC_DMIC;
          bb_config_add_tbl.ser_mode = AS_SER_MODE_64FS;
        }
      else
        {
          bb_config_add_tbl.micDev = AS_ACA_MIC_UNKNOWN;
          bb_config_add_tbl.ser_mode = AS_SER_MODE_64FS;
        }
    }
  return E_AS_OK;
}

E_AS GetAcaPulcoParam(void)
{
  switch (bb_config_tblp->xtal_sel)
    {
      case AS_XTAL_24_576MHZ:
        switch(bb_config_tblp->clk_mode)
          {
            case AS_CLK_MODE_NORMAL:
              sAcaPulcoParam.oscMode = AS_ACA_OSC_24_576MHZ;
              break;

            case AS_CLK_MODE_HIRES:
              return E_AS_XTAL_CLKMODE_PARAM;

            default:
              return E_AS_CLK_MODE_PARAM;
          }
        break;

      case AS_XTAL_49_152MHZ:
        switch(bb_config_tblp->clk_mode)
          {
            case AS_CLK_MODE_NORMAL:
              sAcaPulcoParam.oscMode = AS_ACA_OSC_49_152MHZ;
              break;

            case AS_CLK_MODE_HIRES:
              sAcaPulcoParam.oscMode = AS_ACA_OSC_49_152MHZ_HIRES;
              break;

            default:
              return E_AS_CLK_MODE_PARAM;
          }
        break;
      default:
        return E_AS_XTAL_SEL_PARAM;
    }
  sAcaPulcoParam.micDev = (asAcaPulcoMicDeviceId)bb_config_add_tbl.micDev;

  switch (bb_config_tblp->gpo_ds)
    {
      case AS_IO_DS_WEAKEST:
        sAcaPulcoParam.gpoDs = AS_ACA_IO_DS_WEAKEST;
        break;

      case AS_IO_DS_WEAKER:
        sAcaPulcoParam.gpoDs = AS_ACA_IO_DS_WEAKER;
        break;

      case AS_IO_DS_STRONGER:
        sAcaPulcoParam.gpoDs = AS_ACA_IO_DS_STRONGER;
        break;

      case AS_IO_DS_STRONGEST:
        sAcaPulcoParam.gpoDs = AS_ACA_IO_DS_STRONGEST;
        break;

      default:
        return E_AS_GPO_DS_PARAM;
    }
  switch (bb_config_tblp->ad_data_ds)
    {
      case AS_IO_DS_WEAKEST:
        sAcaPulcoParam.adDataDs = AS_ACA_IO_DS_WEAKEST;
        break;

      case AS_IO_DS_WEAKER:
        sAcaPulcoParam.adDataDs = AS_ACA_IO_DS_WEAKER;
        break;

      case AS_IO_DS_STRONGER:
        sAcaPulcoParam.adDataDs = AS_ACA_IO_DS_STRONGER;
        break;

      case AS_IO_DS_STRONGEST:
        sAcaPulcoParam.adDataDs = AS_ACA_IO_DS_STRONGEST;
        break;

      default:
        return E_AS_AD_DATA_DS_PARAM;
    }
  switch (bb_config_tblp->dmic_clk_ds)
    {
      case AS_IO_DS_WEAKEST:
        sAcaPulcoParam.dmicClkDs = AS_ACA_IO_DS_WEAKEST;
        break;

      case AS_IO_DS_WEAKER:
        sAcaPulcoParam.dmicClkDs = AS_ACA_IO_DS_WEAKER;
        break;

      case AS_IO_DS_STRONGER:
        sAcaPulcoParam.dmicClkDs = AS_ACA_IO_DS_STRONGER;
        break;

      case AS_IO_DS_STRONGEST:
        sAcaPulcoParam.dmicClkDs = AS_ACA_IO_DS_STRONGEST;
        break;

      default:
        return E_AS_DMIC_CLK_DS_PARAM;
    }
  switch (bb_config_tblp->mclk_ds)
    {
      case AS_IO_DS_WEAKEST:
        sAcaPulcoParam.mclkDs = AS_ACA_IO_DS_WEAKEST;
        break;

      case AS_IO_DS_WEAKER:
        sAcaPulcoParam.mclkDs = AS_ACA_IO_DS_WEAKER;
        break;

      case AS_IO_DS_STRONGER:
        sAcaPulcoParam.mclkDs = AS_ACA_IO_DS_STRONGER;
        break;

      case AS_IO_DS_STRONGEST:
        sAcaPulcoParam.mclkDs = AS_ACA_IO_DS_STRONGEST;
        break;

      default:
        return E_AS_MCLK_DS_PARAM;
    }
  return E_AS_OK;
}

E_AS GetAcaPulcoSdesParam(void)
{
  if (bb_config_add_tbl.ser_mode == AS_SER_MODE_128FS)
    {
      sAcaPulcoSdesParam.serMode = AS_ACA_SER_MODE_4CH;
      sAcaPulcoSdesParam.serFs = AS_ACA_SER_FS_128;
    }
  else
    {
      sAcaPulcoSdesParam.serMode = AS_ACA_SER_MODE_8CH;
      sAcaPulcoSdesParam.serFs = AS_ACA_SER_FS_64;
    }

  for (uint8_t i = 0; i < AS_MIC_CHANNEL_MAX; i++)
    {
      uint8_t chSel = (bb_config_tblp->mic_channel_sel >>
                      (i * MIC_CH_BITNUM)) &
                      MIC_CH_BITMAP;
      sAcaPulcoSdesParam.selCh.in[i] = (asAcaPulcoSerSelChId)chSel;
    }
  return E_AS_OK;
}

E_AS GetAcaPulcoInParam(int32_t micgain[AS_MIC_CHANNEL_MAX])
{
  sAcaPulcoInParam.micDev = (asAcaPulcoMicDeviceId)bb_config_add_tbl.micDev;
  switch (bb_config_tblp->mic_bias_sel)
    {
      case 0:
        sAcaPulcoInParam.micBiasSel = AS_ACA_MICBIAS_SEL_2_0V;
        break;

      case 1:
        sAcaPulcoInParam.micBiasSel = AS_ACA_MICBIAS_SEL_2_8V;
        break;

      default:
        return E_AS_MIC_BIAS_SEL_PARAM;
    }

  sAcaPulcoInParam.micGain[0] = 0;
  sAcaPulcoInParam.micGain[1] = 0;
  sAcaPulcoInParam.micGain[2] = 0;
  sAcaPulcoInParam.micGain[3] = 0;
  sAcaPulcoInParam.pgaGain[0] = 0;
  sAcaPulcoInParam.pgaGain[1] = 0;
  sAcaPulcoInParam.pgaGain[2] = 0;
  sAcaPulcoInParam.pgaGain[3] = 0;
  sAcaPulcoInParam.vgain[0] = 0;
  sAcaPulcoInParam.vgain[1] = 0;
  sAcaPulcoInParam.vgain[2] = 0;
  sAcaPulcoInParam.vgain[3] = 0;

  uint8_t micId;
  uint8_t micCh;
  uint8_t mic_sel;
  for (micCh = 0; micCh < AS_MIC_CHANNEL_MAX; micCh++)
    {
      mic_sel = (bb_config_tblp->mic_channel_sel >> (micCh * MIC_CH_BITNUM)) &
                MIC_CH_BITMAP;
      if ((mic_sel >= 1) && (mic_sel <= 4))
        {
          if (micgain[micCh] == AS_MICGAIN_MUTE)
            {
              continue;
            }
          micId = mic_sel - 1;
          if (micgain[micCh] >= AS_ADN_MIC_GAIN_MAX)
            {
              sAcaPulcoInParam.micGain[micId] = AS_ADN_MIC_GAIN_MAX;
            }
          else
            {
              sAcaPulcoInParam.micGain[micId] = (micgain[micCh] / 30) * 30;
            }
          sAcaPulcoInParam.pgaGain[micId] = micgain[micCh] -
                                            sAcaPulcoInParam.micGain[micId];
        }
    }
  return E_AS_OK;
}

E_AS GetAcaPulcoSmstrParam(FAR asSmstrParam *pAcaPulcoSmstrParam)
{
  switch (bb_config_tblp->clk_mode)
    {
      case AS_CLK_MODE_NORMAL:
        pAcaPulcoSmstrParam->mode = AS_SMSTR_MODE_FS_16;
        pAcaPulcoSmstrParam->mckFs = AS_SMSTR_MCK_FS_512;
        break;

      case AS_CLK_MODE_HIRES:
        pAcaPulcoSmstrParam->mode = AS_SMSTR_MODE_FS_32;
        pAcaPulcoSmstrParam->mckFs = AS_SMSTR_MCK_FS_1024;
        break;

      default:
        return E_AS_CLK_MODE_PARAM;
    }

  pAcaPulcoSmstrParam->chSel   = AS_SMSTR_CHSEL_NORMAL;
  pAcaPulcoSmstrParam->out2Dly = 0x00;

  switch (bb_config_tblp->pwm_mode)
    {
      case 0:
        pAcaPulcoSmstrParam->pwmMode = AS_SMSTR_PWMMD_BOTH_ALTER;
        break;

      case 1:
        pAcaPulcoSmstrParam->pwmMode = AS_SMSTR_PWMMD_SINGLE;
        break;

      case 2:
        pAcaPulcoSmstrParam->pwmMode = AS_SMSTR_PWMMD_BOTH;
        break;

      case 3:
        pAcaPulcoSmstrParam->pwmMode = AS_SMSTR_PWMMD_SINGLE_ALTER;
        break;

      case 4:
        pAcaPulcoSmstrParam->pwmMode = AS_SMSTR_PWMMD_BOTH_ALTER;
        break;

      default:
        pAcaPulcoSmstrParam->pwmMode = AS_SMSTR_PWMMD_BOTH_ALTER;
        break;
    }

  return E_AS_OK;
}

E_AS GetAcaPulcoOutParam(void)
{
  switch (bb_config_add_tbl.output_device_sel)
    {
      case AS_OUT_DEV_OFF:
        sAcaPulcoOutParam.outDev = AS_ACA_OUT_OFF;
        break;

      case AS_OUT_DEV_SP:

        /* Speaker will be enabled when volume setting is unmuted. */

        sAcaPulcoOutParam.outDev = AS_ACA_OUT_OFF;
        break;

      case AS_OUT_DEV_I2S:
        sAcaPulcoOutParam.outDev = AS_ACA_OUT_OFF;
        break;

      default:
        return E_AS_OUT_DEVICE_SEL_PARAM;
    }
  sAcaPulcoOutParam.pwmOut[0] = AS_ACA_PWMOUT_UNKNOWN;
  sAcaPulcoOutParam.pwmOut[1] = AS_ACA_PWMOUT_UNKNOWN;

  switch (bb_config_tblp->sp_delay)
    {
      case 0:
        sAcaPulcoOutParam.spDelay = AS_ACA_SP_DELAY_SEL_UNKNOWN;
        break;

      case 1:
        sAcaPulcoOutParam.spDelay = AS_ACA_SP_DELAY_SEL_NON;
        break;

      case 2:
        sAcaPulcoOutParam.spDelay = AS_ACA_SP_DELAY_SEL_SHORT;
        break;

      case 3:
        sAcaPulcoOutParam.spDelay = AS_ACA_SP_DELAY_SEL_MIDDLE;
        break;

      case 4:
        sAcaPulcoOutParam.spDelay = AS_ACA_SP_DELAY_SEL_LONG;
        break;

      default:
        sAcaPulcoOutParam.spDelay = AS_ACA_SP_DELAY_SEL_UNKNOWN;
        break;
    }

  switch (bb_config_tblp->loop_mode)
    {
      case 0:
        sAcaPulcoOutParam.loopMode = AS_ACA_SP_LOOP_MODE_UNKNOWN;
        break;

      case 1:
        sAcaPulcoOutParam.loopMode = AS_ACA_SP_LOOP_MODE_ENABLE;
        break;

      case 2:
        sAcaPulcoOutParam.loopMode = AS_ACA_SP_LOOP_MODE_DISABLE;
        break;

      default:
        sAcaPulcoOutParam.loopMode = AS_ACA_SP_LOOP_MODE_UNKNOWN;
        break;
    }

  switch (bb_config_tblp->clk_mode)
    {
      case AS_CLK_MODE_NORMAL:
        sAcaPulcoOutParam.mode = AS_SMSTR_MODE_FS_16;
        break;

      case AS_CLK_MODE_HIRES:
        sAcaPulcoOutParam.mode = AS_SMSTR_MODE_FS_32;
        break;

      default:
        return E_AS_CLK_MODE_PARAM;
    }

  sAcaPulcoOutParam.spDlyFree = bb_config_tblp->sp_delay_free;
  sAcaPulcoOutParam.spSpliton = bb_config_tblp->sp_split;
  sAcaPulcoOutParam.spDrv     = bb_config_tblp->sp_drive;

  return E_AS_OK;
}

E_AS GetSdesParam(void)
{
  uint8_t ch_cnt = 0;

  if (bb_config_add_tbl.ser_mode == AS_SER_MODE_128FS)
    {
      sSdesParam.serFs   = AS_ACA_SER_FS_128;
      sSdesParam.serMode = AS_ACA_SER_MODE_4CH;
    }
  else
    {
      sSdesParam.serFs   = AS_ACA_SER_FS_64;
      sSdesParam.serMode = AS_ACA_SER_MODE_8CH;
    }

  for (ch_cnt = 0; ch_cnt < AS_MIC_CHANNEL_MAX; ch_cnt++)
    {
      sSdesParam.selCh.out[ch_cnt] =
                       (asSdesDesSelOutId)(AS_SDES_DES_SEL_CH1 + ch_cnt);
    }

  switch (bb_config_tblp->pdm_lowemi)
    {
      case AS_LOWEMI_4MA:
        setAudioIoPdm(AUDIO_IO_LOWEMI_4MA);
        break;

      case AS_LOWEMI_2MA:
        setAudioIoPdm(AUDIO_IO_LOWEMI_2MA);
        break;

      default:
        return E_AS_PDM_LOWEMI_PARAM;
    }

  return E_AS_OK;
}

E_AS GetI2sParam(uint32_t rate[AS_I2S_ID_NUM],
                 asBypassModeId bypassEn[AS_I2S_ID_NUM])
{
  uint8_t srcCnt = 0;
  uint8_t srcCntMax = 0;
  uint8_t i2_device[AS_I2S_ID_NUM] = {0};
  uint8_t i2s_format[AS_I2S_ID_NUM] = {0};
  const audioIoI2sSel i2sSel[AS_I2S_ID_NUM] =
    {
      AUDIO_IO_I2S_SEL_I2S0, AUDIO_IO_I2S_SEL_I2S1
    };
  const E_AS i2s_device_err[AS_I2S_ID_NUM] =
    {
      E_AS_I2S_DEVICE_1_PARAM, E_AS_I2S_DEVICE_2_PARAM
    };
  const E_AS i2s_format_err[AS_I2S_ID_NUM] =
    {
      E_AS_I2S_FORMAT_1_PARAM, E_AS_I2S_FORMAT_2_PARAM
    };
  E_AS rtCode = chkI2sParam(rate, bypassEn);
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  if (bb_config_tblp->i2s_data_path == AS_I2S_DATA_PATH_2)
    {
      gSrcId = AS_SRC_SEL_BOTH;
      srcCntMax = 2;
    }
  else if ((bb_config_tblp->i2s_data_path == AS_I2S_DATA_PATH_1) ||
           (bb_config_tblp->i2s_data_path == AS_I2S_DATA_PATH_NONE))
    {
      gSrcId = AS_SRC_SEL_SRC1;
      srcCntMax = 1;
    }
  else
    {
      return E_AS_I2S_DATA_PATH_PARAM;
    }

  sI2sParam.pSrc1 = &sSrcParam[0];
  sI2sParam.pSrc2 = &sSrcParam[1];

  switch (bb_config_tblp->clk_mode)
    {
      case AS_CLK_MODE_NORMAL:
        sI2sParam.resMode = AS_SRC_RES_NORMAL;
        break;

      case AS_CLK_MODE_HIRES:
        sI2sParam.resMode = AS_SRC_RES_HIGH;
        break;

      default:
        return E_AS_CLK_MODE_PARAM;
    }

  audioIoLowemi lowemi;
  switch (bb_config_tblp->i2s_lowemi)
    {
      case AS_LOWEMI_4MA:
        lowemi = AUDIO_IO_LOWEMI_4MA;
        break;

      case AS_LOWEMI_2MA:
        lowemi = AUDIO_IO_LOWEMI_2MA;
        break;

      default:
        return E_AS_I2S_LOWEMI_PARAM;
    }

  if (bb_config_tblp->i2s_data_path == AS_I2S_DATA_PATH_NONE)
    {
      sSrcParam[0].i2sMode = AS_SRC_MODE_UNKNOWN;
      sSrcParam[0].ch = AS_SRC_CHANNEL_UNKNOWN;
      sSrcParam[0].fs = AS_SRC_FS_UNKNOWN;
      sSrcParam[0].fmt = AS_SRC_FORMAT_UNKNOWN;
      sSrcParam[0].bypassEn = bypassEn[0];
    }
  else
    {
      i2_device[0] = bb_config_tblp->i2s_device_1;
      i2_device[1] = bb_config_tblp->i2s_device_2;
      i2s_format[0] = bb_config_tblp->i2s_format_1;
      i2s_format[1] = bb_config_tblp->i2s_format_2;

      for (srcCnt = 0; srcCnt < srcCntMax; srcCnt++)
        {
          if (i2_device[srcCnt] == AS_I2S_DEVICE_I2S_MASTER)
            {
              sSrcParam[srcCnt].i2sMode = AS_SRC_MODE_MASTER;
            }
          else if(i2_device[srcCnt] == AS_I2S_DEVICE_I2S_SLAVE)
            {
              sSrcParam[srcCnt].i2sMode = AS_SRC_MODE_SLAVE;
            }
          else
            {
              return i2s_device_err[srcCnt];
            }

          sSrcParam[srcCnt].ch = AS_SRC_CHANNEL_2CH;
          sSrcParam[srcCnt].bypassEn = bypassEn[srcCnt];
          if (sSrcParam[srcCnt].i2sMode == AS_SRC_MODE_SLAVE)
            {
              setAudioIoI2s(i2sSel[srcCnt], AUDIO_IO_I2S_MODE_SLAVE, lowemi);
            }
          else
            {
              setAudioIoI2s(i2sSel[srcCnt], AUDIO_IO_I2S_MODE_MASTER, lowemi);
            }
          if (rate[srcCnt] <= 48000)
            {
              sSrcParam[srcCnt].fs = AS_SRC_FS_LOW;
            }
          else if (rate[srcCnt] <= 96000)
            {
              sSrcParam[srcCnt].fs = AS_SRC_FS_MID;
            }
          else if (rate[srcCnt] <= 192000)
            {
              sSrcParam[srcCnt].fs = AS_SRC_FS_HIGH;
            }
          else
            {
              return E_AS_I2S_RATE_PARAM;
            }

          if (i2s_format[srcCnt] == AS_I2S_FORMAT_I2S)
            {
              sSrcParam[srcCnt].fmt = AS_SRC_FORMAT_I2S;
            }
          else if (i2s_format[srcCnt] == AS_I2S_FORMAT_LEFT)
            {
              sSrcParam[srcCnt].fmt = AS_SRC_FORMAT_LEFT;
            }
          else
            {
              return i2s_format_err[srcCnt];
            }
        }
    }

  return rtCode;
}

E_AS GetCicParam(int32_t micgain[AS_AC_CIC_MIC_CH_NUM],
                 asCicSelId cicId,
                 FAR asCicParam *pCicParam)
{
  uint8_t micCh    = 0;
  uint8_t cicMicCh = 0;
  uint8_t mic_sel  = 0;
  int32_t setGain  = 0;
  for (cicMicCh = 0; cicMicCh < AS_AC_CIC_MIC_CH_NUM; cicMicCh++)
    {
      micCh = (cicMicCh + (cicId * AS_AC_CIC_MIC_CH_NUM));
      mic_sel = (bb_config_tblp->mic_channel_sel >> (micCh*MIC_CH_BITNUM)) &
                MIC_CH_BITMAP;
      if ((mic_sel >= AS_ACA_SER_SEL_DMIC1) &&
          (mic_sel <= AS_ACA_SER_SEL_DMIC8))
        {
          setGain = micgain[cicMicCh];
        }
      else
        {
          if (micgain[cicMicCh] == AS_MICGAIN_MUTE)
            {
              setGain = micgain[cicMicCh];
            }
          else
            {
              setGain = 0;
            }
        }
      pCicParam->gain[cicMicCh] = setGain;
    }

  /* CIC param */

  switch (bb_config_tblp->cic_input_sel)
    {
      case AS_CIC_IN_SEL_NONE:
        pCicParam->inSel = AS_CIC_IN_UNKNOWN;
        break;

      case AS_CIC_IN_SEL_CXD:
        pCicParam->inSel = AS_CIC_IN_ACAPULCO_MIC;
        break;

      case AS_CIC_IN_SEL_DMIC:
        pCicParam->inSel = AS_CIC_IN_DMIC;
        break;

      default:
        return E_AS_CIC_INPUT_SEL_PARAM;
    }

  switch (bb_config_add_tbl.ser_mode)
    {
      case AS_SER_MODE_128FS:
        pCicParam->adcFs = AS_CIC_ADC_FS_128;
        break;

      case AS_SER_MODE_64FS:
        pCicParam->adcFs = AS_CIC_ADC_FS_64;
        break;

      default:
        D_ASSERT(0);
        break;
    }

  pCicParam->gainMode = AS_CIC_GAIN_MODE_CIC;
  pCicParam->hpfMode = AS_CIC_HPF_MODE_LOW;

  return E_AS_OK;
}

E_AS GetDcmParam(FAR asDecimParam *pDcmParam)
{
  /* DECIM param */

  if ((bb_config_add_tbl.ser_mode == AS_SER_MODE_64FS) &&
      (bb_config_tblp->clk_mode == AS_CLK_MODE_HIRES))
    {
      pDcmParam->mode = AS_DECIM_MODE_THROUGH;
    }
  else
    {
      pDcmParam->mode = AS_DECIM_MODE_DECIM;
    }
  switch (bb_config_add_tbl.ser_mode)
    {
      case AS_SER_MODE_128FS:
        pDcmParam->inFs = AS_DECIM_IN_FS_8;
        break;

      case AS_SER_MODE_64FS:
        pDcmParam->inFs = AS_DECIM_IN_FS_4;
        break;

      default:
        D_ASSERT(0);
        break;
    }
  switch (bb_config_tblp->clk_mode)
    {
      case AS_CLK_MODE_NORMAL:
        pDcmParam->outFs = AS_DECIM_OUT_FS_1;
        break;

      case AS_CLK_MODE_HIRES:
        pDcmParam->outFs = AS_DECIM_OUT_FS_4;
        break;

      default:
        return E_AS_CLK_MODE_PARAM;
    }
  pDcmParam->outEn0 = 0x0f;
  pDcmParam->outEn1 = 0x0f;
  return E_AS_OK;
}

AsClkModeId GetClkMode(void)
{
  return bb_config_tblp->clk_mode;
}
