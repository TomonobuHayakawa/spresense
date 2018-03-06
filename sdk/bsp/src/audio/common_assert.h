/****************************************************************************
 * arch/arm/src/cxd56xx/audio/drivers/baseband/include/common_assert.h
 *
 *   Copyright (C) 2010-2012, 2017 Sony Corporation
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
/* Description: Common assertion macros */

#ifndef __SDK_BSP_SRC_AUDIO_COMMON_ASSERT_H
#define __SDK_BSP_SRC_AUDIO_COMMON_ASSERT_H

/****************************************************************************
 * Included Files
 ***************************************************************************/

#include "audio/common_macro.h"
#include "assert.h"

/****************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/*
 * Static assert. expがfalseと評価されるとコンパイルエラーとなる
 * expはコンパイル時に評価可能な定数式のみ記述可能
 */
#define S_ASSERT(exp) \
  enum \
    { \
      JOIN_MACRO(AssertionFail_, __LINE__) = 1 / (!!(exp)) \
    }

/*
 * Debug assert.
 * NDEBUGマクロ定義時に空文になるので、expやfuncに副作用のある式は記述不可
 */
#ifdef NDEBUG
#  define D_ASSERT(exp)        ((void)0)
#  define D_ASSERT2(exp, func) ((void)0)
#else /* NDEBUG */
#  define D_ASSERT(exp)        F_ASSERT(exp)
#  define D_ASSERT2(exp, func) F_ASSERT2((exp), (func))
#endif /* NDEBUG */

/*
 * Fatal assert.
 * D_ASSERTと違いNDEBUG時も無効にならない。expやfuncに副作用のある式を記述可能
 */
#ifdef ASSERT_USE_RETURN_ADDR
#  define F_ASSERT(exp) ASSERT(exp)
#  define F_ASSERT2(exp, func) ASSERT(exp)
#else  /* ASSERT_USE_RETURN_ADDR */
#  define F_ASSERT(exp) ASSERT(exp)
#  define F_ASSERT2(exp, func) ASSERT(exp)
#endif /* ASSERT_USE_RETURN_ADDR */

#ifdef  __cplusplus
extern "C" {
#endif

#if defined(_MSC_VER)
extern void * _ReturnAddress(void);
# pragma intrinsic(_ReturnAddress)
#  define GET_RETURN_ADDR() _ReturnAddress()
#elif defined(__CC_ARM)
#  define GET_RETURN_ADDR() (void *)__return_address()
#else
/*
 * x86向け以外の大半のgccでは、
 * 引数に0しか指定できないらしい(gcc3.3.6+allegro-2.2.3では0のみ)
 */
#  define GET_RETURN_ADDR() __builtin_return_address(0)
#endif

/*
 * BSPで、_AssertionFail関数を実装すること
 * ISS環境であれば、下記のような実装で良い
 * printf("%s, Assert at %s line:%d\n", exp, filename, line); exit(1);
 */
#ifdef ASSERT_USE_RETURN_ADDR
/* extern void _AssertionFail(const char* exp, FAR const char* filename,
 * int line, void* ret_addr);
 */
#else  /* ASSERT_USE_RETURN_ADDR */
/* extern void _AssertionFail(const char* exp, FAR const char* filename,
 * int line);
 */
#endif /* ASSERT_USE_RETURN_ADDR */

#ifdef  __cplusplus
} /* end of extern "C" */
#endif

/****************************************************************************
 * Public Types
 ***************************************************************************/

/****************************************************************************
 * Public Data
 ***************************************************************************/

/****************************************************************************
 * Public Functions
 ***************************************************************************/

#endif	/* __SDK_BSP_SRC_AUDIO_COMMON_ASSERT_H */
