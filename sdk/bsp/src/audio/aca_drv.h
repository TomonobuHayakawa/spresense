/****************************************************************************
 * nuttx/arch/arm/src/cxd56xx/audio/drivers/baseband/include/aca_drv.h
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

#ifndef __SDK_BSP_SRC_AUDIO_ACA_DRV_H
#define __SDK_BSP_SRC_AUDIO_ACA_DRV_H

/****************************************************************************
 * Included Files
 ***************************************************************************/

#include "audio/as_drv_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

#define AS_MIC_GAIN_HOLD        255
#define AS_MIC_GAIN_MAX         150
#define AS_MIC_GAIN_MIN         0

#define AS_PGA_GAIN_HOLD        255
#define AS_PGA_GAIN_MAX         60
#define AS_PGA_GAIN_MIN         0

#define AS_VGAIN_HOLD           127
#define AS_VGAIN_MAX            60
#define AS_VGAIN_MIN            -95

typedef enum
{
  AS_ACA_OSC_UNKNOWN,
  AS_ACA_OSC_24_576MHZ,        /* 24.576MHz */
  AS_ACA_OSC_24_576MHZ_HIRES,  /* 24.576MHz,Hi-Res */
  AS_ACA_OSC_49_152MHZ,        /* 49.152MHz */
  AS_ACA_OSC_49_152MHZ_HIRES,  /* 49.152MHz,Hi-Res */
  AS_ACA_OSC_MAX_ENTRY
} asAcaPulcoOscModeId;

typedef enum
{
  AS_ACA_MIC_UNKNOWN,
  AS_ACA_MIC_AMIC,             /* Analog MIC */
  AS_ACA_MIC_DMIC,             /* Digital MIC */
  AS_ACA_MIC_BOTH,             /* Analog MIC and Digital MIC */
  AS_ACA_MIC_MAX_ENTRY
} asAcaPulcoMicDeviceId;

typedef enum
{
  AS_ACA_IO_DS_UNKNOWN,
  AS_ACA_IO_DS_WEAKEST,        /* Weakest */
  AS_ACA_IO_DS_WEAKER,         /* Weaker */
  AS_ACA_IO_DS_STRONGER,       /* Stronger */
  AS_ACA_IO_DS_STRONGEST,      /* Strongest */
  AS_ACA_IO_DS_MAX_ENTRY
} asAcaPulcoIoDsId;

typedef enum
{
  AS_ACA_OUT_UNKNOWN,
  AS_ACA_OUT_HP,               /* Headphone output */
  AS_ACA_OUT_EP,               /* Ear Speaker output */
  AS_ACA_OUT_PWM,              /* PWM output */
  AS_ACA_OUT_HP_PWM,           /* Headphone and PWM output */
  AS_ACA_OUT_EP_PWM,           /* Ear Speaker and PWM output */
  AS_ACA_OUT_OFF,              /* Disable output */
  AS_ACA_OUT_MAX_ENTRY
} asAcaPulcoOutDeviceId;

typedef enum
{
  AS_ACA_MICBIAS_SEL_UNKNOWN,
  AS_ACA_MICBIAS_SEL_2_0V,     /* 2.0V */
  AS_ACA_MICBIAS_SEL_2_8V,     /* 2.8V */
  AS_ACA_MICBIAS_SEL_MAX_ENTRY
} asAcaPulcoMicBiasSelId;

typedef enum
{
  AS_ACA_SP_LOOP_MODE_UNKNOWN,
  AS_ACA_SP_LOOP_MODE_ENABLE,
  AS_ACA_SP_LOOP_MODE_DISABLE,
  AS_ACA_SP_LOOP_MODE_MAX_ENTRY
} asAcaSpLoopModeId;

typedef enum
{
  AS_ACA_SP_DELAY_SEL_UNKNOWN,
  AS_ACA_SP_DELAY_SEL_NON,
  AS_ACA_SP_DELAY_SEL_SHORT,
  AS_ACA_SP_DELAY_SEL_MIDDLE,
  AS_ACA_SP_DELAY_SEL_LONG,
  AS_ACA_SP_DELAY_SEL_MAX_ENTRY
} asAcaSpDelaySelId;

typedef enum
{
  AS_ACA_PWMOUT_UNKNOWN,
  AS_ACA_PWMOUT_OFF,           /* Disable */
  AS_ACA_PWMOUT_LN,            /* LN */
  AS_ACA_PWMOUT_LP,            /* LP */
  AS_ACA_PWMOUT_RN,            /* RN */
  AS_ACA_PWMOUT_RP,            /* RP */
  AS_ACA_PWMOUT_MAX_ENTRY
} asAcaPulcoPwmOutId;

typedef enum
{
  AS_ACA_SP_DLY_FREE_UNKNOWN,
  AS_ACA_SP_DLY_FREE_OFF,
  AS_ACA_SP_DLY_FREE_ON,
  AS_ACA_SP_DLY_FREE_MAX_ENTRY
} asAcaSpDlyFreeId;

typedef enum
{
  AS_ACA_SP_SPLITON_SEL_UNKNOWN,
  AS_ACA_SP_SPLITON_SEL_SHORTEST,
  AS_ACA_SP_SPLITON_SEL_SHORT,
  AS_ACA_SP_SPLITON_SEL_LONG,
  AS_ACA_SP_SPLITON_SEL_LONGEST,
  AS_ACA_SP_SPLITON_SEL_MAX_ENTRY
} asAcaSpSplitonSelId;

typedef enum
{
  AS_ACA_SP_DRV_SEL_UNKNOWN,
  AS_ACA_SP_DRV_SEL_4DRIVER,
  AS_ACA_SP_DRV_SEL_2DRIVER,
  AS_ACA_SP_DRV_SEL_1DRIVER,
  AS_ACA_SP_DRV_SEL_LINEOUT,
  AS_ACA_SP_DRV_SEL_MAX_ENTRY
} asAcaSpDrvSelId;

/****************************************************************************
 * Public Types
 ***************************************************************************/

typedef struct
{
  asAcaPulcoOscModeId   oscMode;
  asAcaPulcoMicDeviceId micDev;
  asAcaPulcoIoDsId      gpoDs;
  asAcaPulcoIoDsId      adDataDs;
  asAcaPulcoIoDsId      dmicClkDs;
  asAcaPulcoIoDsId      mclkDs;
} asAcaPulcoParam;

typedef struct
{
  asAcaPulcoMicDeviceId   micDev;
  asAcaPulcoMicBiasSelId  micBiasSel;
  uint32_t                micGain[4];
  uint32_t                pgaGain[4];
  int32_t                 vgain[4];
} asAcaPulcoInParam;

typedef struct
{
  asAcaPulcoOutDeviceId outDev;
  asAcaPulcoPwmOutId    pwmOut[2];
  asAcaSpDelaySelId     spDelay;
  asAcaSpLoopModeId     loopMode;
  asSmstrModeId         mode;
  asAcaSpDlyFreeId      spDlyFree;
  asAcaSpSplitonSelId   spSpliton;
  asAcaSpDrvSelId       spDrv;
} asAcaPulcoOutParam;

/****************************************************************************
 * Public Data
 ***************************************************************************/

extern asAcaPulcoParam     sAcaPulcoParam;
extern asSerDesParam       sAcaPulcoSdesParam;
extern asAcaPulcoInParam   sAcaPulcoInParam;
extern asAcaPulcoOutParam  sAcaPulcoOutParam;

extern uint64_t            m_mic_boot_start_time;

/****************************************************************************
 * Public Functions
 ***************************************************************************/

/* API for AcaPulco */
E_AS asAca_CheckID(void);
E_AS asAca_PowerOnAcaPulco(FAR asAcaPulcoParam *pAcaPulcoParam);
E_AS asAca_SetAcaPulcoParam(FAR asAcaPulcoParam *pAcaPulcoParam);
E_AS asAca_PowerOnAcaPulcoInput(FAR asAcaPulcoInParam *pAcaPulcoInParam);
E_AS asAca_SetAcaPulcoInput(FAR asAcaPulcoInParam *pAcaPulcoInParam);
E_AS asAca_PowerOnAcaPulcoOutput(FAR asAcaPulcoOutParam *pAcaPulcoOutParam);
E_AS asAca_SetAcaPulcoOutput(FAR asAcaPulcoOutParam *pAcaPulcoOutParam);
E_AS asAca_SetSerDes(FAR asSerDesParam *pSdesParam);
E_AS asAca_SetSmstr(FAR asSmstrParam *pAcaPulcoSmstrParam);
E_AS asAca_PowerOffAcaPulco(void);
E_AS asAca_PowerOffAcaPulcoInput(void);
E_AS asAca_PowerOffAcaPulcoOutput(void);
E_AS asAca_PowerOnMicBiasA(void);
E_AS asAca_PowerOffMicBiasA(void);
E_AS initAcaPulcoAmic(FAR asAcaPulcoInParam *pAcaPulcoInParam);
void asAca_SetOutputDevice(asOutDeviceId outDevId);
E_AS PowerOnAcaPulco(void);
E_AS EnableAcaPulcoInput(int32_t micgain[8]);
E_AS EnableAcaPulcoOutput(void);
E_AS PowerOnHpadcMicBias(void);
E_AS PowerOffHpadcMicBias(void);

#endif /* __SDK_BSP_SRC_AUDIO_ACA_DRV_H */
