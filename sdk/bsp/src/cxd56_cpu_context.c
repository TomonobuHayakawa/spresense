/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_cpu_context.c
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

/*------------------------------------------------------------------------------
 * include files
 *----------------------------------------------------------------------------*/

#include <nuttx/irq.h>
#include "cxd56_cpu_context.h"
#include "nvic.h"
#include "up_arch.h"

/*------------------------------------------------------------------------------
 * private function define
 *----------------------------------------------------------------------------*/

static int cxd56_cpu_context_scsctxsave(up_context_ctx_t *pV7m);
static int cxd56_cpu_context_scsctxload(up_context_ctx_t *pV7m);
static int cxd56_cpu_context_systickctxsave(up_context_ctx_t *pV7m);
static int cxd56_cpu_context_systickctxload(up_context_ctx_t *pV7m);
static int cxd56_cpu_context_nvicctxsave(up_context_ctx_t *pV7m);
static int cxd56_cpu_context_nvicctxload(up_context_ctx_t *pV7m);

/*------------------------------------------------------------------------------
 * private variable define
 *----------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 * public variable define
 *----------------------------------------------------------------------------*/

up_context_ctx_t g_V7mCtx;

/*------------------------------------------------------------------------------
 * board define
 *----------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/**
 * <function name>
 * PD_V6mScsContextSave / Load
 *
 */
/*---------------------------------------------------------------------------*/

/* System control and ID register */

#define NVIC_FPCAR  (ARMV7M_NVIC_BASE + NVIC_FPCAR_OFFSET)
#define NVIC_FPDSCR  (ARMV7M_NVIC_BASE + NVIC_FPDSCR_OFFSET)
#define NVIC_ACTLR  (ARMV7M_NVIC_BASE + 0x008)

/* ICSR:Interrupt Control State Register */

#define CXD56_CONTEXT_SCB_ICSR_NMIPENDSET  0x80000000
#define CXD56_CONTEXT_SCB_ICSR_PENDSVSET  0x10000000
#define CXD56_CONTEXT_SCB_ICSR_PENDSVCLR  0x08000000
#define CXD56_CONTEXT_SCB_ICSR_PENDSTSET  0x04000000
#define CXD56_CONTEXT_SCB_ICSR_PENDSTCLR  0x02000000
#define CXD56_CONTEXT_SCB_ICSR_ISRPREEMPT  0x00800000
#define CXD56_CONTEXT_SCB_ICSR_ISRPENDING  0x00400000
#define CXD56_CONTEXT_SCB_ICSR_VECTPENDING  0x001FF000
#define CXD56_CONTEXT_SCB_ICSR_RETTOBASE  0x00000800
#define CXD56_CONTEXT_SCB_ICSR_VECTACTIVE  0x0000001F

static int cxd56_cpu_context_scsctxsave(up_context_ctx_t *ctx)
{
  uint32_t icsr;

  ctx->vtor = getreg32(NVIC_VECTAB);
  ctx->aircr = getreg32(NVIC_AIRCR);
  ctx->scr = getreg32(NVIC_SYSCON);
  ctx->shpri1 = getreg32(NVIC_SYSH4_7_PRIORITY);
  ctx->shpri2 = getreg32(NVIC_SYSH8_11_PRIORITY);
  ctx->shpri3 = getreg32(NVIC_SYSH12_15_PRIORITY);
  ctx->shcsr = getreg32(NVIC_SYSHCON);
  ctx->cpacr = getreg32(NVIC_CPACR);
  ctx->fpccr = getreg32(NVIC_FPCCR);
  ctx->fpcar = getreg32(NVIC_FPCAR);
  ctx->fpdscr = getreg32(NVIC_FPDSCR);
  ctx->actlr = getreg32(NVIC_ACTLR);

  /* Clear Pending PendSV and SysTic */

  icsr = (CXD56_CONTEXT_SCB_ICSR_PENDSVCLR | CXD56_CONTEXT_SCB_ICSR_PENDSTCLR);
  putreg32(icsr, NVIC_INTCTRL);

  return 0;
}

static int cxd56_cpu_context_scsctxload(up_context_ctx_t *ctx)
{
  putreg32(ctx->actlr, NVIC_ACTLR);
  putreg32(ctx->vtor, NVIC_VECTAB);
  putreg32(ctx->aircr, NVIC_AIRCR);
  putreg32(ctx->scr, NVIC_SYSCON);
  putreg32(ctx->shpri1, NVIC_SYSH4_7_PRIORITY);
  putreg32(ctx->shpri2, NVIC_SYSH8_11_PRIORITY);
  putreg32(ctx->shpri3, NVIC_SYSH12_15_PRIORITY);
  putreg32(ctx->shcsr, NVIC_SYSHCON);
  putreg32(ctx->cpacr, NVIC_CPACR);
  putreg32(ctx->fpccr, NVIC_FPCCR);
  putreg32(ctx->fpcar, NVIC_FPCAR);
  putreg32(ctx->fpdscr, NVIC_FPDSCR);
  putreg32(ctx->actlr, NVIC_ACTLR);

  return 0;
}

/*---------------------------------------------------------------------------*/
/**
 * <function name>
 * PD_V6mSysTickContextSave / Load
 *
 */
/*---------------------------------------------------------------------------*/

static int cxd56_cpu_context_systickctxsave(up_context_ctx_t *ctx)
{
  ctx->syst_rvr = getreg32(NVIC_SYSTICK_RELOAD);
  ctx->syst_csr = getreg32(NVIC_SYSTICK_CTRL);

  putreg32(0, NVIC_SYSTICK_CTRL);

  return 0;
}

static int cxd56_cpu_context_systickctxload(up_context_ctx_t *ctx)
{
  putreg32(ctx->syst_rvr, NVIC_SYSTICK_RELOAD);
  putreg32(ctx->syst_csr, NVIC_SYSTICK_CTRL);

  return 0;
}

/*---------------------------------------------------------------------------*/
/**
 * <function name>
 * cxd56_context_NvicContextSave / Load
 *
 */
/*---------------------------------------------------------------------------*/

static int cxd56_cpu_context_nvicctxsave(up_context_ctx_t *ctx)
{
  int i;
  volatile uint32_t *pNvic;

  pNvic = (volatile uint32_t *)NVIC_IRQ0_31_ENABLE;
  for ( i=0; i<UP_CPU_CONTEXT_NVIC_ISER_NUM; i++ )
    {
      ctx->nvic_iser[i] = getreg32(pNvic);
      pNvic ++;
    }

  pNvic = (volatile uint32_t *)NVIC_IRQ0_3_PRIORITY;
  for ( i=0; i<UP_CPU_CONTEXT_NVIC_IPR_NUM; i++ )
    {
      ctx->nvic_ipr[i] = getreg32(pNvic);
      pNvic ++;
    }

  /* Dsiable Interrupt, except IPCM */

  pNvic = (volatile uint32_t *)NVIC_IRQ0_31_CLEAR;
  for ( i=0; i<UP_CPU_CONTEXT_NVIC_ISER_NUM; i++ )
    {
      if ( 3 == i )
        {

          /* XXX don't disable bit13:SW_INT,
           *                   bit14:fifo of sending,
           * bit15:fifo of receiving
           */

          putreg32(0xFFFF1FFF, pNvic);
        }
      else
        {
          putreg32(0xFFFFFFFF, pNvic);
        }
        pNvic ++;
    }

  /* Clear Pending Interrupt */

  pNvic = (volatile uint32_t *)NVIC_IRQ0_31_CLRPEND;
  for ( i=0; i<UP_CPU_CONTEXT_NVIC_ISER_NUM; i++ )
    {
      putreg32(0xFFFFFFFF, pNvic);
      pNvic ++;
    }

  return 0;
}

static int cxd56_cpu_context_nvicctxload(up_context_ctx_t *ctx)
{
  int i;
  volatile uint32_t *pNvic;

  pNvic = (volatile uint32_t *)NVIC_IRQ0_3_PRIORITY;
  for ( i=0; i<UP_CPU_CONTEXT_NVIC_IPR_NUM; i++ )
    {
      putreg32(ctx->nvic_ipr[i], pNvic);
      pNvic ++;
    }

  pNvic = (volatile uint32_t *)NVIC_IRQ0_31_ENABLE;
  for ( i=0; i<UP_CPU_CONTEXT_NVIC_ISER_NUM; i++ )
    {
      putreg32(ctx->nvic_iser[i], pNvic);
      pNvic ++;
    }

  return 0;
}

/*---------------------------------------------------------------------------*/
/**
 * <function name>
 * cxd56_context_Sleep
 *
 */
/*---------------------------------------------------------------------------*/
int cxd56_cpu_context_sleep(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  cxd56_cpu_context_nvicctxsave(&g_V7mCtx);
  cxd56_cpu_context_systickctxsave(&g_V7mCtx);
  up_fpuctxsave(&g_V7mCtx);
  cxd56_cpu_context_scsctxsave(&g_V7mCtx);
  up_cpuctxsavewithwfi(&g_V7mCtx);

  cxd56_cpu_context_scsctxload(&g_V7mCtx);
  up_fpuctxload(&g_V7mCtx);
  cxd56_cpu_context_systickctxload(&g_V7mCtx);
  cxd56_cpu_context_nvicctxload(&g_V7mCtx);

  leave_critical_section(flags);
  return 0;
}

