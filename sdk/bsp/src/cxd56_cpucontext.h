/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_cpu_context.h
 *
 *   Copyright (C) 2016 Sony Corporation. All rights reserved.
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
#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_CPUCONTEXT_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_CPUCONTEXT_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/*------------------------------------------------------------------------------
 * Include files
 *----------------------------------------------------------------------------*/
 
#include <stdint.h>

/*------------------------------------------------------------------------------
 * Pre-processor Definitions
 *----------------------------------------------------------------------------*/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct cpu_state_s {

  /* ARM core register */

  uint32_t r4;
  uint32_t r5;
  uint32_t r6;
  uint32_t r7;
  uint32_t r8;
  uint32_t r9;
  uint32_t r10;
  uint32_t r11;
  uint32_t r12;
  uint32_t r14;
  uint32_t msp;
  uint32_t psp;
  uint32_t xpsr;
  uint32_t primask;
  uint32_t control;
  uint32_t basepri;
  uint32_t faultmask;

  /* FPU register */

  uint32_t s0;
  uint32_t s1;
  uint32_t s2;
  uint32_t s3;
  uint32_t s4;
  uint32_t s5;
  uint32_t s6;
  uint32_t s7;
  uint32_t s8;
  uint32_t s9;
  uint32_t s10;
  uint32_t s11;
  uint32_t s12;
  uint32_t s13;
  uint32_t s14;
  uint32_t s15;
  uint32_t s16;
  uint32_t s17;
  uint32_t s18;
  uint32_t s19;
  uint32_t s20;
  uint32_t s21;
  uint32_t s22;
  uint32_t s23;
  uint32_t s24;
  uint32_t s25;
  uint32_t s26;
  uint32_t s27;
  uint32_t s28;
  uint32_t s29;
  uint32_t s30;
  uint32_t s31;
  uint32_t fpscr;

  /* SCS : System control and ID registers */

  uint32_t vtor;
  uint32_t aircr;
  uint32_t scr;
  uint32_t shpri1;
  uint32_t shpri2;
  uint32_t shpri3;
  uint32_t shcsr;
  uint32_t cpacr;
  uint32_t fpccr;
  uint32_t fpcar;
  uint32_t fpdscr;
  uint32_t actlr;

  /* SCS : SysTick */

  uint32_t syst_csr;
  uint32_t syst_rvr;

  /* SCS : NVIC */

  uint32_t nvic_iser[UP_CPU_CONTEXT_NVIC_ISER_NUM];
  uint32_t nvic_ipr[UP_CPU_CONTEXT_NVIC_IPR_NUM];

  /* SCS : MPU */

};

/*------------------------------------------------------------------------------
 * Public Data
 *----------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 * Public Functions
 *----------------------------------------------------------------------------*/

int cxd56_cpu_context_sleep(void);

int up_cpuctxsavewithwfi(struct cpu_state_s *ctx);
void up_cpuctxload(void);
int up_fpuctxsave(struct cpu_state_s *ctx);
int up_fpuctxload(struct cpu_state_s *ctx);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* __ARCH_ARM_SRC_CXD56XX_CXD56_CPUCONTEXT_H */
