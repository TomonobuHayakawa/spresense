/****************************************************************************
 * arch/arm/src/cxd56xx/audio/drivers/baseband/include/ac_reg_map.h
 *
 *   Copyright (C) 2014, 2017 Sony Corporation
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
/*
 * This header file is generated as follows.
 * % perl genRegHeader.pl AC_TOP_regmap_20140805.xlsm
 * Output file: ac_reg_map.h, ac_reg_map.c
 * Number of detected registers: 317
 */

#ifndef __SDK_BSP_SRC_AUDIO_AC_REG_MAP_H
#define __SDK_BSP_SRC_AUDIO_AC_REG_MAP_H

/****************************************************************************
 * Included Files
 ***************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* Enable at local test.
 * #define AC_LOCAL_TEST
 */

#ifdef AC_LOCAL_TEST
#  define AC_REG_BASE  0x00001000
#else
#  define AC_REG_BASE  (0x0c000000 + 0x02300000)
#endif

#define AC_REVID     0x20
#define AC_DEVICEID  0x02

#define DNC1_IRAM_BASE  0x3000
#define DNC1_CRAM_BASE  0x3800
#define DNC2_IRAM_BASE  0x3c00
#define DNC2_CRAM_BASE  0x4400

/****************************************************************************
 * Public Types
 ***************************************************************************/

/****************************************************************************
 * Public Data
 ***************************************************************************/

/****************************************************************************
 * Public Functions
 ***************************************************************************/

#endif /* _AC_REG_MAP_H_ */
