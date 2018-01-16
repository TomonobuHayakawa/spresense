/****************************************************************************
 * arch/arm/src/cxd56xx/audio/drivers/baseband/include/ac_drv_func.h
 *
 *   Copyright (C) 2014, 2017 Sony Corporation
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
/* Description: Audio Codec driver low-level API sub function */

#ifndef __SDK_BSP_SRC_AUDIO_AC_DRV_SUB_FUNC_H
#define __SDK_BSP_SRC_AUDIO_AC_DRV_SUB_FUNC_H

/****************************************************************************
 * Included Files
 ***************************************************************************/

#include "audio/as_drv_common.h"
#include "audio/ac_drv.h"
#include "audio/ac_drv_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/****************************************************************************
 * Public Types
 ***************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/****************************************************************************
 * Public Data
 ***************************************************************************/

extern BaseBandConfigTbl *bb_config_tblp;

/****************************************************************************
 * Public Functions
 ***************************************************************************/

E_AS setSrcParam(asSrcSelId srcId, FAR asI2sParam *pI2sParam);
E_AS setI2sMode(asSrcSelId srcId, FAR asSrcParam *pSrc);
E_AS setI2sChSwap(asSrcSelId srcId, FAR asSrcParam *pSrc);
E_AS setSelCicParam(asCicSelId cicId, FAR asCicParam *pCicParam);
E_AS setDecimCommon(FAR asDecimParam *pDcmParam);
E_AS setDecimOut(asDecimSelId dcmId, FAR asDecimParam *pDcmParam);
E_AS setAlcParam(void);
E_AS setSpcParam(void);
E_AS setDeqCoef(AC_REG_ID acRegId, FAR const uint32_t *pCoef, uint32_t len);
E_AS setDncRam(uint32_t offset, FAR const uint32_t *pData, uint32_t len);
E_AS setAcSmstrParam(asSmstrModeId mode);
E_AS setAcSerDesParam(FAR asSerDesParam *pSdesParam);

#ifdef __cplusplus
} /* end of extern "C" */
#endif /* __cplusplus */

#endif /* __SDK_BSP_SRC_AUDIO_AC_DRV_SUB_FUNC_H */
