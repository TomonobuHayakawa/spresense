/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_cpucontext.c
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

/* TODO: This source needs more refactoring.
 *
 * 1. cxd56_cpucontext.c is not appropriate. I think cxd56_hotsleep.c is better.
 * 2. NVIC register bits are already defined at nvic.h, use them.
 * 3. I want more comments for the order of saving registers.
 * 4. Function names to be shorter and rethink separation.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>

#include "nvic.h"
#include "up_arch.h"

#include "cxd56_cpucontext.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* System control and ID register */

#define NVIC_FPCAR  (ARMV7M_NVIC_BASE + NVIC_FPCAR_OFFSET)
#define NVIC_FPDSCR (ARMV7M_NVIC_BASE + NVIC_FPDSCR_OFFSET)
#define NVIC_ACTLR  (ARMV7M_NVIC_BASE + 0x008)

/* ICSR:Interrupt Control State Register */

#define CXD56_CONTEXT_SCB_ICSR_NMIPENDSET   0x80000000
#define CXD56_CONTEXT_SCB_ICSR_PENDSVSET    0x10000000
#define CXD56_CONTEXT_SCB_ICSR_PENDSVCLR    0x08000000
#define CXD56_CONTEXT_SCB_ICSR_PENDSTSET    0x04000000
#define CXD56_CONTEXT_SCB_ICSR_PENDSTCLR    0x02000000
#define CXD56_CONTEXT_SCB_ICSR_ISRPREEMPT   0x00800000
#define CXD56_CONTEXT_SCB_ICSR_ISRPENDING   0x00400000
#define CXD56_CONTEXT_SCB_ICSR_VECTPENDING  0x001FF000
#define CXD56_CONTEXT_SCB_ICSR_RETTOBASE    0x00000800
#define CXD56_CONTEXT_SCB_ICSR_VECTACTIVE   0x0000001F

#define NR_ISERS  4
#define NR_IPRS  32

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void cxd56_cpu_context_scsctxsave(void);
static void cxd56_cpu_context_scsctxload(void);
static void cxd56_cpu_context_systickctxsave(void);
static void cxd56_cpu_context_systickctxload(void);
static void cxd56_cpu_context_nvicctxsave(void);
static void cxd56_cpu_context_nvicctxload(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct cpu_state_s g_cpustate;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void cxd56_cpu_context_scsctxsave(void)
{
  struct cpu_state_s *state = &g_cpustate;
  uint32_t icsr;

  state->vtor   = getreg32(NVIC_VECTAB);
  state->aircr  = getreg32(NVIC_AIRCR);
  state->scr    = getreg32(NVIC_SYSCON);
  state->shpri1 = getreg32(NVIC_SYSH4_7_PRIORITY);
  state->shpri2 = getreg32(NVIC_SYSH8_11_PRIORITY);
  state->shpri3 = getreg32(NVIC_SYSH12_15_PRIORITY);
  state->shcsr  = getreg32(NVIC_SYSHCON);
  state->cpacr  = getreg32(NVIC_CPACR);
  state->fpccr  = getreg32(NVIC_FPCCR);
  state->fpcar  = getreg32(NVIC_FPCAR);
  state->fpdscr = getreg32(NVIC_FPDSCR);
  state->actlr  = getreg32(NVIC_ACTLR);

  /* Clear Pending PendSV and SysTic */

  icsr = (CXD56_CONTEXT_SCB_ICSR_PENDSVCLR | CXD56_CONTEXT_SCB_ICSR_PENDSTCLR);
  putreg32(icsr, NVIC_INTCTRL);
}

static void cxd56_cpu_context_scsctxload(void)
{
  struct cpu_state_s *state = &g_cpustate;

  putreg32(state->actlr,  NVIC_ACTLR);
  putreg32(state->vtor,   NVIC_VECTAB);
  putreg32(state->aircr,  NVIC_AIRCR);
  putreg32(state->scr,    NVIC_SYSCON);
  putreg32(state->shpri1, NVIC_SYSH4_7_PRIORITY);
  putreg32(state->shpri2, NVIC_SYSH8_11_PRIORITY);
  putreg32(state->shpri3, NVIC_SYSH12_15_PRIORITY);
  putreg32(state->shcsr,  NVIC_SYSHCON);
  putreg32(state->cpacr,  NVIC_CPACR);
  putreg32(state->fpccr,  NVIC_FPCCR);
  putreg32(state->fpcar,  NVIC_FPCAR);
  putreg32(state->fpdscr, NVIC_FPDSCR);
  putreg32(state->actlr,  NVIC_ACTLR);
}

static void cxd56_cpu_context_systickctxsave(void)
{
  struct cpu_state_s *state = &g_cpustate;

  state->syst_rvr = getreg32(NVIC_SYSTICK_RELOAD);
  state->syst_csr = getreg32(NVIC_SYSTICK_CTRL);

  putreg32(0, NVIC_SYSTICK_CTRL);
}

static void cxd56_cpu_context_systickctxload(void)
{
  struct cpu_state_s *state = &g_cpustate;

  putreg32(state->syst_rvr, NVIC_SYSTICK_RELOAD);
  putreg32(state->syst_csr, NVIC_SYSTICK_CTRL);
}

static void cxd56_cpu_context_nvicctxsave(void)
{
  struct cpu_state_s *state = &g_cpustate;
  volatile uint32_t *addr;
  int i;

  addr = (volatile uint32_t *)NVIC_IRQ0_31_ENABLE;
  for (i = 0; i < NR_ISERS; i++, addr++)
    {
      state->nvic_iser[i] = getreg32(addr);
    }

  addr = (volatile uint32_t *)NVIC_IRQ0_3_PRIORITY;
  for (i = 0; i < NR_IPRS; i++, addr++)
    {
      state->nvic_ipr[i] = getreg32(addr);
    }

  /* Dsiable Interrupt, except IPCM */

  addr = (volatile uint32_t *)NVIC_IRQ0_31_CLEAR;
  for (i = 0; i < NR_ISERS; i++, addr++)
    {
      if (i == 3)
        {

          /* XXX don't disable bit13:SW_INT,
           *                   bit14:fifo of sending,
           * bit15:fifo of receiving
           */

          putreg32(0xFFFF1FFF, addr);
        }
      else
        {
          putreg32(0xFFFFFFFF, addr);
        }
    }

  /* Clear Pending Interrupt */

  addr = (volatile uint32_t *)NVIC_IRQ0_31_CLRPEND;
  for (i = 0; i < NR_ISERS; i++, addr++)
    {
      putreg32(0xFFFFFFFF, addr);
    }
}

static void cxd56_cpu_context_nvicctxload(void)
{
  struct cpu_state_s *state = &g_cpustate;
  volatile uint32_t *addr;
  int i;

  addr = (volatile uint32_t *)NVIC_IRQ0_3_PRIORITY;
  for (i = 0; i < NR_IPRS; i++, addr++)
    {
      putreg32(state->nvic_ipr[i], addr);
    }

  addr = (volatile uint32_t *)NVIC_IRQ0_31_ENABLE;
  for (i = 0; i < NR_ISERS; i++, addr++)
    {
      putreg32(state->nvic_iser[i], addr);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cxd56_cpu_context_sleep(void)
{
  struct cpu_state_s *state = &g_cpustate;
  irqstate_t flags;

  flags = enter_critical_section();

  cxd56_cpu_context_nvicctxsave(state);
  cxd56_cpu_context_systickctxsave(state);
  up_fpuctxsave(state);
  cxd56_cpu_context_scsctxsave(state);
  up_cpuctxsavewithwfi(state);

  cxd56_cpu_context_scsctxload(state);
  up_fpuctxload(state);
  cxd56_cpu_context_systickctxload(state);
  cxd56_cpu_context_nvicctxload(state);

  leave_critical_section(flags);

  return 0;
}

