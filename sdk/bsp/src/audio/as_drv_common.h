/****************************************************************************
 * nuttx/arch/arm/src/cxd56xx/audio/drivers/baseband/include/as_drv_common.h
 *
 *   Copyright (C) 2016-2017 Sony Corporation. All rights reserved.
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
 ****************************************************************************/

#ifndef __SDK_BSP_SRC_AUDIO_AS_DRV_COMMON_H
#define __SDK_BSP_SRC_AUDIO_AS_DRV_COMMON_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <arch/chip/cxd56_audio.h>

#include "audio/common_attention.h"
#include "audio/common_assert.h"
#include "chip/cxd5602_memorymap.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* INT_CAUSE1 */
#define AS_INT_CAUSE1_REG               (CXD56_INTC_BASE+3*4)
#define AS_INT_CAUSE1_BIT_AU0           6 // 102 % 32bit
#define AS_INT_CAUSE1_BIT_AU1           7
#define AS_INT_CAUSE1_BIT_AU2           8
#define AS_INT_CAUSE1_BIT_AU3           9

/* INT_EN1 */
#define AS_INT_EN1_REG_ADDR             (CXD56_INTC_BASE+0x10+3*4)

/* INT_POL */
#define AS_INT_POL_REG                  (CXD56_INTC_BASE+0x20+3*4)
#define AS_INT_POL_BIT_AU0              6
#define AS_INT_POL_BIT_AU1              7
#define AS_INT_POL_BIT_AU2              8
#define AS_INT_POL_BIT_AU3              9

/* INT_IRQ1 */
#define AS_INT_IRQ1_REG_ADDR            (CXD56_INTC_BASE+0x30+3*4)

#define AS_FLG_DMAC_AC_IN_READY    0x00000001
#define AS_FLG_DMAC_AC_IN_DONE     0x00000002
#define AS_FLG_DMAC_I2S_IN_READY   0x00000004
#define AS_FLG_DMAC_I2S_IN_DONE    0x00000008
#define AS_FLG_DMAC_I2S_OUT_READY  0x00000010
#define AS_FLG_DMAC_I2S_OUT_DONE   0x00000020
#define AS_FLG_DMAC_I2S2_IN_READY  0x00000040
#define AS_FLG_DMAC_I2S2_IN_DONE   0x00000080
#define AS_FLG_DMAC_I2S2_OUT_READY 0x00000100
#define AS_FLG_DMAC_I2S2_OUT_DONE  0x00000200

#if defined(AC_LOCAL_TEST) || defined(ADO_LOCAL_TEST) || defined(ACA_LOCAL_TEST)
#define newSYS_DelayTask(n)
#endif

#define MIC_CH_BITNUM             (4)
#define MIC_CH_BITMAP             (0x0f)

/// keep setting for #AdnIOSet.mic_gain_a, #AdnIOSet.mic_gain_b, #AdnIOSet.mic_gain_c, #AdnIOSet.mic_gain_d
#define AS_ADN_MIC_GAIN_HOLD            (255)
/// maximum value for above parameters
#define AS_ADN_MIC_GAIN_MAX             (150)
/// minimun value for above parameters
#define AS_ADN_MIC_GAIN_MIN             (0)

/// keep setting for #AdnIOSet.pga_gain_a, #AdnIOSet.pga_gain_b, #AdnIOSet.pga_gain_c, #AdnIOSet.pga_gain_d
#define AS_ADN_PGA_GAIN_HOLD            (255)
/// maxmium value for above parameters
#define AS_ADN_PGA_GAIN_MAX             (60)
/// minimum value for above parameters
#define AS_ADN_PGA_GAIN_MIN             (0)

/// keep setting for #AdnIOSet.vgain_a, #AdnIOSet.vgain_b, #AdnIOSet.vgain_c, #AdnIOSet.vgain_d
#define AS_ADN_VGAIN_HOLD               (127)
/// maxinum value for above parameters
#define AS_ADN_VGAIN_MAX                (60)
/// minimum value for above parameters
#define AS_ADN_VGAIN_MIN                (-95)


typedef enum {
	AS_SMSTR_MODE_FS_UNKNOWN,
	AS_SMSTR_MODE_FS_16,            // 16fs
	AS_SMSTR_MODE_FS_32,            // 32fs
	AS_SMSTR_MODE_FS_MAX_ENTRY
} asSmstrModeId;

typedef enum {
	AS_SMSTR_MCK_FS_UNKNOWN,
	AS_SMSTR_MCK_FS_512,            // 512fs
	AS_SMSTR_MCK_FS_1024,           // 1024fs
	AS_SMSTR_MCK_FS_MAX_ENTRY
} asSmstrMckId;

typedef enum {
	AS_SMSTR_PWMMD_UNKNOWN,
	AS_SMSTR_PWMMD_SINGLE,          // Single side
	AS_SMSTR_PWMMD_BOTH,            // Both side
	AS_SMSTR_PWMMD_SINGLE_ALTER,    // Single side alternating
	AS_SMSTR_PWMMD_BOTH_ALTER,      // Both side alternating
	AS_SMSTR_PWMMD_MAX_ENTRY
} asSmstrPwmModeId;

typedef enum {
	AS_SMSTR_CHSEL_UNKNOWN,
	AS_SMSTR_CHSEL_NORMAL,          // Normal
	AS_SMSTR_CHSEL_EXCHANGE,        // Exchange L and R
	AS_SMSTR_CHSEL_MAX_ENTRY
} asSmstrChSelId;

/** Select data transfer mode from CXD5247 to Spritzer*/
typedef enum {
	/// keep setting
	AS_SER_MODE_HOLD = 0,
	/// 128fs/4ch (Analog MIC only)
	AS_SER_MODE_128FS,
	/// 64fs/8ch (default)
	AS_SER_MODE_64FS,
	AS_SER_MODE_NUM
} AsSerModeId;

typedef enum {
	AS_DNC_SEL_DNC1,                // DNC1
	AS_DNC_SEL_DNC2,                // DNC2
	AS_DNC_SEL_BOTH,                // DNC1 and DNC2
	AS_DNC_SEL_MAX_ENTRY
} asDncSelId;

typedef struct {
	asSmstrModeId       mode;
	asSmstrMckId        mckFs;
	asSmstrPwmModeId    pwmMode;
	asSmstrChSelId      chSel;
	uint8_t             out2Dly;
} asSmstrParam;


E_AS GetBaseBandConfigParam( void );
E_AS GetAcaPulcoParam( void );
E_AS GetAcaPulcoSdesParam( void );
E_AS GetAcaPulcoInParam( int32_t micgain[AS_MIC_CHANNEL_MAX] );
E_AS GetAcaPulcoSmstrParam( asSmstrParam *pAcaPulcoSmstrParam );
E_AS GetAcaPulcoOutParam( void );
E_AS GetSdesParam( void );
E_AS GetI2sParam( uint32_t rate[AS_I2S_NUM], asBypassModeId bypassEn[AS_I2S_NUM] );


void asBca_SetSmstrParam( void );
E_AS asBca_SetSrcParam( void );
E_AS asBca_SetDncParam(asDncSelId dncId);

#ifdef __cplusplus
} /* end of extern "C" */
#endif /* __cplusplus */

#endif /* __SDK_BSP_SRC_AUDIO_AS_DRV_COMMON_H */
