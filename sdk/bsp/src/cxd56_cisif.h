/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_cisif.h
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Tetsuro Itabashi <Tetsuro.x.Itabashi@sony.com>
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_CISIF_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_CISIF_H

/************************************************************************************
 * Public Types
 ************************************************************************************/

typedef void (*Notify_callback_t)(uint8_t code, uint32_t size, uint32_t addr);
typedef void (*Comp_callback_t)(uint8_t code, uint8_t last_frame, uint32_t size, uint32_t addr);

typedef enum
{
  E_OK,
  E_SETTING_FAILED,
  E_INVALID_STATE,
  E_INVALID_PARAMETER,
} ResCode;

typedef enum
{
  CisifFormatYuv,
  CisifFormatJpeg,
  CisifFormatInterleave,
  CisifFormatMax,
} CisifFormat;

typedef struct
{
  uint16_t hsize;
  uint16_t vsize;
  uint32_t notify_size;
  Notify_callback_t notify_func;
  Comp_callback_t comp_func;
} CisifInitYuvParam;

typedef struct
{
  uint32_t notify_size;
  Notify_callback_t notify_func;
  Comp_callback_t comp_func;
} CisifInitJpgParam;

typedef struct
{
  CisifFormat format;
  CisifInitYuvParam yuv_param;
  CisifInitJpgParam jpg_param;
} CisifParam_t;

typedef struct
{
  uint8_t *strg_addr;
  uint32_t strg_size;
} StrageArea;

typedef struct
{
  uint8_t *strg_addr_0;
  uint8_t *strg_addr_1;
  uint32_t strg_size;
} BankStrageArea;


#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

ResCode cxd56_cisifinit(CisifParam_t *pCisifPar);
ResCode cxd56_cisifcaptureframe(StrageArea *yuv_area, StrageArea *jpg_area);
ResCode cxd56_cisifstartmonitoring(BankStrageArea *yuv_area, BankStrageArea *jpg_area);
ResCode cxd56_cisifstopmonitoring(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_CISIF_H */
