/****************************************************************************
 * arch/arm/src/cxd56xx/audio/drivers/baseband/include/ac_drv.h
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
/* Description: Audio Codec driver local header */

#ifndef __SDK_BSP_SRC_AUDIO_AC_DRV_H
#define __SDK_BSP_SRC_AUDIO_AC_DRV_H

/****************************************************************************
 * Included Files
 ***************************************************************************/

#include <arch/chip/cxd56_audio.h>

#include "audio/as_drv_common.h"
#include "audio/ac_drv_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

#define AS_MTBR_GAIN_HOLD       255
#define AS_MTBR_GAIN_MAX        0
#define AS_MTBR_GAIN_MIN        -630
#define AS_MTBR_GAIN_MUTE       -635

#define AS_CIC_GAIN_MAX         0
#define AS_CIC_GAIN_MIN         -7850
#define AS_CIC_GAIN_MUTE        -7855

#define AS_ALC_TARGET_HOLD      127
#define AS_ALC_TARGET_MIN       -63
#define AS_ALC_TARGET_MAX       0

#define AS_ALC_KNEE_HOLD        255
#define AS_ALC_KNEE_MIN         -635
#define AS_ALC_KNEE_MAX         0

#define AS_ALC_DELAY_HOLD       255
#define AS_ALC_DELAY_MIN        0
#define AS_ALC_DELAY_MAX        63

#define AS_ALC_LPF_HOLD         0xffff
#define AS_ALC_LPF_HIRES_MIN    8
#define AS_ALC_LPF_HIRES_MAX    21200000
#define AS_ALC_LPF_NORMAL_MIN   2
#define AS_ALC_LPF_NORMAL_MAX   5300000
#define AS_ALC_LPF_THROUGH      0

#define AS_ALCSPC_ATTACK_HOLD   0xffffffff
#define AS_ALCSPC_ATTACK_MIN    0
#define AS_ALCSPC_ATTACK_MAX    87300000

#define AS_ALCSPC_RELEASE_HOLD  0xffffffff
#define AS_ALCSPC_RELEASE_MIN   0
#define AS_ALCSPC_RELEASE_MAX   87300000

#define AS_SPC_LIMIT_HOLD       255
#define AS_SPC_LIMIT_MIN        -250
#define AS_SPC_LIMIT_MAX        0

#define DEQ_BAND_NUM            6
#define DEQ_COEF_NUM            5
#define DNC_IRAM_SIZE           (256 * 2)
#define DNC_CRAM_SIZE           128

#define AS_AC_CIC_NUM           4
#define AS_AC_CIC_MIC_CH_NUM    2

typedef enum
{
  AS_SRC_SEL_SRC1,                /* SRC1 */
  AS_SRC_SEL_SRC2,                /* SRC2 */
  AS_SRC_SEL_BOTH,                /* SRC1 and SRC2 */
  AS_SRC_SEL_MAX_ENTRY
} asSrcSelId;

typedef enum
{
  AS_SRC_RES_UNKNOWN,
  AS_SRC_RES_HIGH,                /* High Resolution */
  AS_SRC_RES_NORMAL,              /* Normal Resolution */
  AS_SRC_RES_MAX_ENTRY
} asI2sResolutionId;

typedef enum
{
  AS_SRC_MODE_UNKNOWN,
  AS_SRC_MODE_MASTER,             /* I2S Master mode */
  AS_SRC_MODE_SLAVE,              /* I2S Slave mode */
  AS_SRC_MODE_MAX_ENTRY
} asI2sModeId;

typedef enum
{
  AS_SRC_FS_UNKNOWN,
  AS_SRC_FS_HIGH,                 /* 96 <  fs <= 192[kHz] */
  AS_SRC_FS_MID,                  /* 48 <  fs <=  96[kHz] */
  AS_SRC_FS_LOW,                  /*  8 <= fs <=  48[kHz] */
  AS_SRC_FS_MAX_ENTRY
} asI2sFsId;

typedef enum
{
  AS_SRC_FORMAT_UNKNOWN,
  AS_SRC_FORMAT_I2S,              /* I2S format */
  AS_SRC_FORMAT_LEFT,             /* Left Justified format */
  AS_SRC_FORMAT_MAX_ENTRY
} asI2sFormatId;

typedef enum
{
  AS_SRC_CHANNEL_UNKNOWN,
  AS_SRC_CHANNEL_0CH,             /* 0 channels */
  AS_SRC_CHANNEL_2CH,             /* 2 channels */
  AS_SRC_CHANNEL_MAX_ENTRY
} asI2sChannelId;

typedef enum
{
  AS_CIC_SEL_CIC1,                /* CIC1 */
  AS_CIC_SEL_CIC2,                /* CIC2 */
  AS_CIC_SEL_CIC3,                /* CIC3 */
  AS_CIC_SEL_CIC4,                /* CIC4 */
  AS_CIC_SEL_CIC12,               /* CIC1 and CIC2 */
  AS_CIC_SEL_CIC34,               /* CIC3 and CIC4 */
  AS_CIC_SEL_CIC_ALL,             /* CIC1,CIC2,CIC3 and CIC4 */
  AS_CIC_SEL_MAX_ENTRY
} asCicSelId;

typedef enum
{
  AS_CIC_IN_UNKNOWN,
  AS_CIC_IN_ACAPULCO_MIC,         /* AcaPulco */
  AS_CIC_IN_DMIC,                 /* DMIC */
  AS_CIC_IN_MAX_ENTRY
} asCicInSelId;

typedef enum
{
  AS_CIC_ADC_FS_UNKNOWN,
  AS_CIC_ADC_FS_64,               /* 64fs */
  AS_CIC_ADC_FS_128,              /* 128fs */
  AS_CIC_ADC_FS_MAX_ENTRY
} asCicAdcFsId;

typedef enum
{
  AS_CIC_GAIN_MODE_UNKNOWN,
  AS_CIC_GAIN_MODE_MATSUBARA,     /* MATSUBARA compatible mode */
  AS_CIC_GAIN_MODE_CIC,           /* CIC_GAIN mode */
  AS_CIC_GAIN_MODE_MAX_ENTRY
} asCicGainModeId;

typedef enum
{
  AS_CIC_HPF_MODE_UNKNOWN,
  AS_CIC_HPF_MODE_OFF,            /* HPF cut-off disable */
  AS_CIC_HPF_MODE_LOW,            /* HPF cut-off frequency low */
  AS_CIC_HPF_MODE_MID,            /* HPF cut-off frequency mid */
  AS_CIC_HPF_MODE_HIGH,           /* HPF cut-off frequency high */
  AS_CIC_HPF_MODE_MAX_ENTRY
} asCicHpfModeId;

typedef enum
{
  AS_DECIM_SEL_DECIM0,            /* DECIM0 */
  AS_DECIM_SEL_DECIM1,            /* DECIM1 */
  AS_DECIM_SEL_BOTH,              /* DECIM0 and DECIM1 */
  AS_DECIM_SEL_MAX_ENTRY
} asDecimSelId;

typedef enum
{
  AS_DECIM_MODE_UNKNOWN,
  AS_DECIM_MODE_THROUGH,          /* Through mode */
  AS_DECIM_MODE_DECIM,            /* Decimation mode */
  AS_DECIM_MODE_MAX_ENTRY
} asDecimModeId;

typedef enum
{
  AS_DECIM_IN_FS_UNKNOWN,
  AS_DECIM_IN_FS_4,               /* 4fs input */
  AS_DECIM_IN_FS_8,               /* 8fs input */
  AS_DECIM_IN_FS_MAX_ENTRY
} asDecimInputFsId;

typedef enum
{
  AS_DECIM_OUT_FS_UNKNOWN,
  AS_DECIM_OUT_FS_1,              /* 1fs output */
  AS_DECIM_OUT_FS_2,              /* 2fs output */
  AS_DECIM_OUT_FS_4,              /* 4fs output */
  AS_DECIM_OUT_FS_MAX_ENTRY
} asDecimOutputFsId;

typedef enum
{
  AS_ALC_MODE_UNKNOWN,
  AS_ALC_MODE_PLAY,
  AS_ALC_MODE_RECORD,
  AS_ALC_MODE_MAX_ENTRY
} asAlcModeId;

typedef enum
{
  AS_ALC_ALG_UNKNOWN,
  AS_ALC_ALG_NOT_USE_INSTANT,
  AS_ALC_ALG_USE_INSTANT,
  AS_ALC_ALG_MAX_ENTRY
} asAlcAlgId;

typedef enum
{
  AS_SPC_FILTER_UNKNOWN,
  AS_SPC_FILTER_THROUGH,
  AS_SPC_FILTER_AWEIGHT,
  AS_SPC_FILTER_MAX_ENTRY
} asSpcFilterId;

typedef enum
{
  AS_DEQ_MODE_A,
  AS_DEQ_MODE_B,
  AS_DEQ_MODE_C,
  AS_DEQ_MODE_MAX_ENTRY
} asDeqModeId;

/****************************************************************************
 * Public Types
 ***************************************************************************/

typedef struct
{
  asDecimModeId       mode;
  asDecimInputFsId    inFs;
  asDecimOutputFsId   outFs;
  uint32_t            outEn0;
  uint32_t            outEn1;
} asDecimParam;

typedef struct
{
  asCicInSelId        inSel;
  asCicAdcFsId        adcFs;
  asCicGainModeId     gainMode;
  asCicHpfModeId      hpfMode;
  int32_t             gain[AS_AC_CIC_MIC_CH_NUM];
} asCicParam;

typedef struct
{
  asI2sModeId         i2sMode;
  asI2sFsId           fs;
  asI2sFormatId       fmt;
  asI2sChannelId      ch;
  asBypassModeId      bypassEn;
} asSrcParam;

typedef struct
{
  asI2sResolutionId   resMode;
  asSrcParam *        pSrc1;
  asSrcParam *        pSrc2;
} asI2sParam;

typedef struct
{
  AsClkModeId         clkMode;
  asAlcModeId         alcRec;
  int32_t             alcTarget;
  int32_t             alcKnee;
  asAlcAlgId          alcAlg;
  uint32_t            alcDelay;
  uint32_t            alcLpf;
  uint32_t            alcAttack;
  uint32_t            alcRelease;
} asAlcParam;

typedef struct
{
  AsClkModeId         clkMode;
  asSpcFilterId       spcAweight;
  int32_t             spcLimit;
  uint32_t            spcAttack;
  uint32_t            spcRelease;
} asSpcParam;

typedef struct
{
  const uint32_t *    pCoefBand1;
  const uint32_t *    pCoefBand2;
  const uint32_t *    pCoefBand3;
  const uint32_t *    pCoefBand4;
  const uint32_t *    pCoefBand5;
  const uint32_t *    pCoefBand6;
} asDeqParam;

typedef struct
{
  const uint32_t *    pDncIRam;
  const uint32_t *    pDncCRam;
} asDncParam;

/****************************************************************************
 * Public Data
 ***************************************************************************/

extern asI2sParam        sI2sParam;
extern asSrcSelId        gSrcId;
extern asSerDesParam     sSdesParam;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/****************************************************************************
 * Public Functions
 ***************************************************************************/

/* API for Audio Codec */
E_AS asAc_CheckID(void);
E_AS asAc_ResetDsp(void);
E_AS asAc_InitDsp(void);
E_AS asAc_PowerOnSrc(asSrcSelId srcId, FAR asI2sParam *pI2sParam);
E_AS asAc_SetSrcParam(asSrcSelId srcId, FAR asI2sParam *pI2sParam);
E_AS asAc_EnableSrc(asSrcSelId srcId);
E_AS asAc_PowerOnCic(asCicSelId cicId, FAR asCicParam *pCicParam);
E_AS asAc_SetCicParam(asCicSelId cicId, FAR asCicParam *pCicParam);
E_AS asAc_PowerOnDecim(void);
E_AS asAc_SetDecimParam(asDecimSelId dcmId);
E_AS asAc_PowerOnCodec(void);
E_AS asAc_SetAlcSpcParam(void);
E_AS asAc_SetClearStereo(asCsSignId csSign, int32_t csVol);
E_AS asAc_SetDeqCoef(bool deqEn, FAR asDeqParam *pDeqParam);
E_AS asAc_PowerOnDnc(asDncSelId dncId);
E_AS asAc_SetDncParam(bool dncEn, asDncSelId dncId,
                      FAR asDncParam *pDncParam1,
                      FAR asDncParam *pDncParam2);
E_AS asAc_PowerOnSmstr(void);
E_AS asAc_SetSmstrParam(void);
E_AS asAc_EnableSmstr(void);
E_AS asAc_PowerOnSerDes(FAR asSerDesParam *pSdesParam);
E_AS asAc_SetSerDesParam(FAR asSerDesParam *pSdesParam);
E_AS asAc_EnableSer(void);
void asAc_PowerOffAudioCodec(void);
void asAc_PowerOffAudioCodecInput(void);
void asAc_PowerOffAudioCodecOutput(void);
E_AS PowerOnAudioCodec(uint32_t rate[AS_I2S_NUM],
                       asBypassModeId bypass_mode_en[AS_I2S_NUM]);
E_AS EnableAudioCodecInput(int32_t micgain[8]);
E_AS EnableAudioCodecOutput(void);

E_AS GetCicParam(int32_t micgain[AS_AC_CIC_MIC_CH_NUM],
                 asCicSelId cicId,
                 FAR asCicParam *pCicParam);
E_AS GetDcmParam(FAR asDecimParam *pDcmParam);
E_AS setSelCicGain(asCicSelId cicId, FAR asCicParam *pCicParam);
E_AS setAcDesOut(FAR asSerDesParam *pSdesParam);
void setAcOutputI2S(void);

#ifdef __cplusplus
} /* end of extern "C" */
#endif /* __cplusplus */

#endif /* __SDK_BSP_SRC_AUDIO_AC_DRV_H */
