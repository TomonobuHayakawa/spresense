/****************************************************************************
 * nuttx/arch/arm/src/cxd56xx/audio/drivers/baseband/src/aca_drv_api.c
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

/* If you enable the driver, enable it.
 * #define CONFIG_ACA_DRV_ENABLE
 */

#define CONFIG_BB_API_ENABLE

#ifdef CONFIG_ACA_DRV_ENABLE
#  include "audio/aca_drv_sub_func.h"
#endif
#ifdef CONFIG_BB_API_ENABLE
#  include "audio/as_drv_common.h"
#  include "audio/aca_drv.h"

#  include "up_arch.h"
#  include "chip/cxd5602_topreg.h"
#  include "cxd56_clock.h"
#  include "arch/board/board.h"
#  include "audio/common_assert.h"
#  include <time.h>
#  include <debug.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
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

#ifdef CONFIG_ACA_DRV_ENABLE
E_AS AS_AcaControl(uint8_t type, uint32_t param)
{
  E_AS rtCode = E_AS_OK;
  FAR asAcaPulcoParam    *pAcaPulcoParam;
  FAR asAcaPulcoInParam  *pAcaPulcoInParam;
  FAR asAcaPulcoOutParam *pAcaPulcoOutParam;
  FAR asSerDesParam      *pSdesParam;
  FAR asSmstrParam       *pAcaPulcoSmstrParam;
  asOutDeviceId      outDevId;

  switch ((AsAcaControlType)type)
    {
      case AS_ACA_CHECK_ID:
        rtCode = asAca_CheckID();
        break;

      case AS_ACA_POWER_ON_COMMON:
        pAcaPulcoParam = (FAR asAcaPulcoParam *)param;
        rtCode = asAca_PowerOnAcaPulco(pAcaPulcoParam);
        break;

      case AS_ACA_POWER_ON_INPUT:
        pAcaPulcoInParam = (FAR asAcaPulcoInParam *)param;
        rtCode = asAca_PowerOnAcaPulcoInput(pAcaPulcoInParam);
        break;

      case AS_ACA_POWER_ON_OUTPUT:
        pAcaPulcoOutParam = (FAR asAcaPulcoOutParam *)param;
        rtCode = asAca_PowerOnAcaPulcoOutput(pAcaPulcoOutParam);
        break;

      case AS_ACA_SET_SERDES:
        pSdesParam = (FAR asSerDesParam *)param;
        rtCode = asAca_SetSerDes(pSdesParam);
        break;

      case AS_ACA_SET_SMASTER:
        pAcaPulcoSmstrParam = (FAR asSmstrParam *)param;
        rtCode = asAca_SetSmstr(pAcaPulcoSmstrParam);
        break;

      case AS_ACA_POWER_OFF_COMMON:
        rtCode = asAca_PowerOffAcaPulco();
        break;

      case AS_ACA_POWER_OFF_INPUT:
        rtCode = asAca_PowerOffAcaPulcoInput();
        break;

      case AS_ACA_POWER_OFF_OUTPUT:
        rtCode = asAca_PowerOffAcaPulcoOutput();
        break;

      case AS_ACA_POWER_ON_MICBIAS:
        rtCode = asAca_PowerOnMicBiasA();
        break;

      case AS_ACA_POWER_OFF_MICBIAS:
        rtCode = asAca_PowerOffMicBiasA();
        break;

      case AS_ACA_INIT_AMIC:
        pAcaPulcoInParam = (FAR asAcaPulcoInParam *)param;
        rtCode = initAcaPulcoAmic(pAcaPulcoInParam);
        break;

      case AS_ACA_SET_OUTPUT_DEVICE:
        outDevId = (asOutDeviceId)param;
        asAca_SetOutputDevice(outDevId);
        break;

      case AS_ACA_SET_AMIC_BOOT_DONE:
        rtCode = setAcaPulcoAmicBootDone();
        break;

      default:
        return E_AS_ACAPULCO_ID_NG;
    }
  return rtCode;
}

E_AS asAca_CheckID(void)
{
  uint8_t chipid = read_aca_reg(AU_CHIPID);
  switch (chipid)
    {
      case ACA_CHIPID_ES1:
        _info("AcaPulco ES1(%02Xh)\n", chipid);
        break;

      case ACA_CHIPID_ES2:
        _info("AcaPulco ES2(%02Xh)\n", chipid);
        break;

      case ACA_CHIPID_ES3:
        _info("AcaPulco ES3(%02Xh)\n", chipid);
        break;

      case ACA_CHIPID_ES4:
        _info("AcaPulco ES4(%02Xh)\n", chipid);
        break;

      default:
        _info("%s()[ERR] (code:%2d, ChipID:%02Xh)\n",
               __func__, E_AS_ACAPULCO_ID_NG, chipid);
        return E_AS_ACAPULCO_ID_NG;
    }

  return E_AS_OK;
}

E_AS asAca_PowerOnAcaPulco(FAR asAcaPulcoParam *pAcaPulcoParam)
{
  E_AS rtCode = E_AS_OK;

  D_ASSERT(pAcaPulcoParam != NULL);

  /* Enable OSC. */

  rtCode = enableAcaPulcoOSC(pAcaPulcoParam);
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  /* Set clock frequency. */

  rtCode = setAcaPulcoClkFreq(pAcaPulcoParam);
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  /* Enable clock. */

  write_aca_reg(AU_LOGIC_CLK_EN, 1);
  write_aca_reg(AU_O_MCLK_EN, 1);

  rtCode = asAca_SetAcaPulcoParam(pAcaPulcoParam);

  return rtCode;
}

E_AS asAca_SetAcaPulcoParam(FAR asAcaPulcoParam *pAcaPulcoParam)
{
  E_AS rtCode = E_AS_OK;

  D_ASSERT(pAcaPulcoParam != NULL);

  /* Initialize AcaPulco I/O. */

  rtCode = initAcaPulcoIO(pAcaPulcoParam);

  return rtCode;
}

E_AS asAca_PowerOffAcaPulco()
{
  E_AS rtCode = E_AS_OK;

  rtCode = disableAcaPulcoOSC();
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  /* Disable clock. */

  write_aca_reg(AU_LOGIC_CLK_EN, 0);
  write_aca_reg(AU_O_MCLK_EN, 0);

  return rtCode;
}

E_AS asAca_PowerOffAcaPulcoInput()
{
  E_AS rtCode = E_AS_OK;

  rtCode = powerOffAcaPulcoAdc();
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  rtCode = powerOffAcaPulcoAmic();
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  rtCode = powerOffAcaPulcoDmic();

  return rtCode;
}

E_AS asAca_PowerOffAcaPulcoOutput()
{
  E_AS rtCode = E_AS_OK;

  rtCode = powerOffAcaPulcoEpOut();
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  rtCode = disableAcaPulcoPwmOut();

  return rtCode;
}

E_AS asAca_PowerOnAcaPulcoInput(FAR asAcaPulcoInParam *pAcaPulcoInParam)
{
  E_AS rtCode = E_AS_OK;

  D_ASSERT(pAcaPulcoInParam != NULL);

  switch (pAcaPulcoInParam->micDev)
    {
      case AS_ACA_MIC_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_ACA_MIC_AMIC:
        rtCode = powerOnAcaPulcoAmic(pAcaPulcoInParam);
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }
        break;

      case AS_ACA_MIC_DMIC:
        rtCode = powerOnAcaPulcoDmic();
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }
        break;

      case AS_ACA_MIC_BOTH:
        rtCode = powerOnAcaPulcoAmic(pAcaPulcoInParam);
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }
        rtCode = powerOnAcaPulcoDmic();
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }
        break;

      default:
        D_ASSERT(0);
        break;
    }

  rtCode = asAca_SetAcaPulcoInput(pAcaPulcoInParam);

  return rtCode;
}

E_AS asAca_SetAcaPulcoInput(FAR asAcaPulcoInParam *pAcaPulcoInParam)
{
  E_AS rtCode = E_AS_OK;

  D_ASSERT(pAcaPulcoInParam != NULL);

  switch (pAcaPulcoInParam->micDev)
    {
      case AS_ACA_MIC_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_ACA_MIC_AMIC:
      case AS_ACA_MIC_BOTH:
        rtCode = initAcaPulcoAmic(pAcaPulcoInParam);
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }
        rtCode = initAcaPulcoAdc(pAcaPulcoInParam);
        break;

      case AS_ACA_MIC_DMIC:
        /* Do nothing. */
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS asAca_PowerOnAcaPulcoOutput(FAR asAcaPulcoOutParam *pAcaPulcoOutParam)
{
  E_AS rtCode = E_AS_OK;

  D_ASSERT(pAcaPulcoOutParam != NULL);

  switch (pAcaPulcoOutParam->outDev)
    {
      case AS_ACA_OUT_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_ACA_OUT_HP:
      case AS_ACA_OUT_EP:
        rtCode = powerOnAcaPulcoEpOut(pAcaPulcoOutParam);
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }
        rtCode = enableAcaPulcoEpOut();
        break;

      case AS_ACA_OUT_PWM:
        rtCode = enableAcaPulcoPwmOut(pAcaPulcoOutParam);
        break;

      case AS_ACA_OUT_HP_PWM:
      case AS_ACA_OUT_EP_PWM:
        rtCode = powerOnAcaPulcoEpOut(pAcaPulcoOutParam);
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }
        rtCode = enableAcaPulcoEpOut();
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }
        rtCode = enableAcaPulcoPwmOut(pAcaPulcoOutParam);
        break;

      case AS_ACA_OUT_OFF:
        rtCode = powerOnAcaPulcoEpOut(pAcaPulcoOutParam);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS asAca_SetAcaPulcoOutput(FAR asAcaPulcoOutParam *pAcaPulcoOutParam)
{
  E_AS rtCode = E_AS_OK;

  D_ASSERT(pAcaPulcoOutParam != NULL);

  switch (pAcaPulcoOutParam->outDev)
    {
      case AS_ACA_OUT_UNKNOWN:
        /* Do nothing. */
        break;

      case AS_ACA_OUT_HP:
      case AS_ACA_OUT_EP:
        rtCode = enableAcaPulcoEpOut();
        break;

      case AS_ACA_OUT_PWM:
        rtCode = enableAcaPulcoPwmOut(pAcaPulcoOutParam);
        break;

      case AS_ACA_OUT_HP_PWM:
      case AS_ACA_OUT_EP_PWM:
        rtCode = enableAcaPulcoEpOut();
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }
        rtCode = enableAcaPulcoPwmOut(pAcaPulcoOutParam);
        break;

      case AS_ACA_OUT_OFF:
        rtCode = disableAcaPulcoEpOut();
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }
        rtCode = disableAcaPulcoPwmOut();
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS asAca_SetSerDes(FAR asSerDesParam *pSdesParam)
{
  E_AS rtCode = E_AS_OK;

  D_ASSERT(pSdesParam != NULL);

  rtCode = setAcaPulcoSerDesParam(pSdesParam);

  return rtCode;
}

E_AS asAca_SetSmstr(FAR asSmstrParam *pAcaPulcoSmstrParam)
{
  E_AS rtCode = E_AS_OK;

  rtCode = setAcaPulcoSmstrParam(pAcaPulcoSmstrParam);

  return rtCode;
}

E_AS asAca_PowerOnMicBiasA()
{
  E_AS rtCode = E_AS_OK;

  rtCode = powerOnAcaPulcoAmicBiasA();

  return rtCode;
}

E_AS asAca_PowerOffMicBiasA()
{
  E_AS rtCode = E_AS_OK;

  rtCode = powerOffAcaPulcoAmicBiasA();

  return rtCode;
}

void asAca_SetOutputDevice(asOutDeviceId outDevId)
{
  if (outDevId == AS_OUT_DEV_SP)
    {
      enableAcaPulcoEpOut();
    }
  else
    {
      disableAcaPulcoEpOut();
    }
}
#endif

#ifdef CONFIG_BB_API_ENABLE
E_AS PowerOnHpadcMicBias()
{
  E_AS rtCode = E_AS_OK;

  D_ASSERT(bb_config_tblp != NULL);
  if (bb_config_tblp->hpadc_mic_bias == AS_HPADC_MIC_BIAS_ON)
    {
      if(!board_aca_power_monitor(CXD5247_AVDD | CXD5247_DVDD))
        {
          return E_AS_PM_ADONIS_PWON_CHK_ERR;
        }

      rtCode = AS_AcaControl(AS_ACA_POWER_ON_MICBIAS, (uint32_t)NULL);
    }
  return rtCode;
}

E_AS PowerOffHpadcMicBias()
{
  E_AS rtCode = E_AS_OK;

  D_ASSERT(bb_config_tblp != NULL);
  if (bb_config_tblp->hpadc_mic_bias == AS_HPADC_MIC_BIAS_ON)
    {
      rtCode = AS_AcaControl(AS_ACA_POWER_OFF_MICBIAS, (uint32_t)NULL);
    }
  return rtCode;
}

E_AS PowerOnAcaPulco(void)
{
  E_AS rtCode;

  rtCode = GetAcaPulcoParam();
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }
  rtCode = GetAcaPulcoSdesParam();
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  rtCode = AS_AcaControl(AS_ACA_CHECK_ID, (uint32_t)NULL);
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }
  rtCode = AS_AcaControl(AS_ACA_POWER_ON_COMMON, (uint32_t)&sAcaPulcoParam);
  _info("asAca_PowerOnAcaPulco(%d)\n", rtCode);
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }
  rtCode = AS_AcaControl(AS_ACA_SET_SERDES, (uint32_t)&sAcaPulcoSdesParam);
  _info("asAca_SetSerDes(%d)\n", rtCode);

  return rtCode;
}

E_AS EnableAcaPulcoInput(int32_t micgain[8])
{
  E_AS rtCode;

  rtCode = GetAcaPulcoInParam(micgain);
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  if ((sAcaPulcoInParam.micDev == AS_ACA_MIC_AMIC) ||
      (sAcaPulcoInParam.micDev == AS_ACA_MIC_BOTH))
    {
      rtCode = AS_AcaControl(AS_ACA_POWER_ON_MICBIAS, (uint32_t)NULL);
      if (rtCode != E_AS_OK)
        {
          return rtCode;
        }

      struct timespec start;
      int ret = clock_gettime(CLOCK_REALTIME, &start);
      F_ASSERT(ret == 0);
      m_mic_boot_start_time = (unsigned long long)start.tv_sec * 1000 +
                              (unsigned long long)start.tv_nsec / 1000000;
    }

  rtCode = AS_AcaControl(AS_ACA_POWER_ON_INPUT, (uint32_t)&sAcaPulcoInParam);
  _info("asAca_PowerOnAcaPulcoInput(%d)\n", rtCode);

  return rtCode;
}

E_AS EnableAcaPulcoOutput(void)
{
  E_AS rtCode;

  rtCode = GetAcaPulcoOutParam();
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  asSmstrParam      acaPulcoSmstrParam;
  rtCode = GetAcaPulcoSmstrParam(&acaPulcoSmstrParam);
  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  rtCode = AS_AcaControl(AS_ACA_SET_SMASTER,
                         (uint32_t)&acaPulcoSmstrParam);
  _info("asAca_SetSmstr(%d)\n", rtCode);
  if (E_AS_OK != rtCode)
    {
      return rtCode;
    }
  rtCode = AS_AcaControl(AS_ACA_POWER_ON_OUTPUT,
                         (uint32_t)&sAcaPulcoOutParam);
  _info("asAca_PowerOnAcaPulcoOutput(%d)\n", rtCode);

  return rtCode;
}
#endif
