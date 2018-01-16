/****************************************************************************
 * arch/arm/src/cxd56xx/audio/drivers/baseband/include/ac_drv_path.h
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
/* Description: Audio Codec data path control */

#ifndef __SDK_BSP_SRC_AUDIO_AC_DRV_PATH_H
#define __SDK_BSP_SRC_AUDIO_AC_DRV_PATH_H

/****************************************************************************
 * Included Files
 ***************************************************************************/

#include "audio/as_drv_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/****************************************************************************
 * Public Types
 ***************************************************************************/

typedef struct
{
  bool cis1;  /* COD_INSEL1 */
  bool cis2;  /* COD_INSEL2 */
  bool cis3;  /* COD_INSEL3 */
  bool ads1;  /* AU_DAT_SEL1 */
  bool ads2;  /* AU_DAT_SEL2 */
  bool s1is;  /* SRC1IN_SEL */
  bool s2is;  /* SRC2IN_SEL */
} asPathSelFlag;

typedef enum
{
  TO_SPI2S_MIXER1,  /* Mixer1 */
  TO_SPI2S_MIXER2,  /* Mixer2 */
  TO_SPI2S_I2S1,    /* I2S1 */
  TO_SPI2S_I2S2,    /* I2S2 */
  TO_SPI2S_NUM
} toSpI2sId;

typedef enum
{
  FROM_SELECTOR1,    /* Selector1 */
  FROM_SELECTOR2,    /* Selector2 */
  FROM_SELECTOR_NUM
} fromSelId;

/****************************************************************************
 * Public Data
 ***************************************************************************/

extern asPathFromId toSpI2s[TO_SPI2S_NUM];

/****************************************************************************
 * Public Functions
 ***************************************************************************/

void setDataPathTo(asPathFromId fromId, uint8_t toId);
void setDataPathToSel(uint8_t fromId, uint8_t toId);
void setDataPathSel(uint8_t selId, asPathFromId fromId);
void initDataPathI2S(void);
E_AS chkDataPath(asPathFromId fromId, asPathToId toId);

#endif /* __SDK_BSP_SRC_AUDIO_AC_DRV_PATH_H */
