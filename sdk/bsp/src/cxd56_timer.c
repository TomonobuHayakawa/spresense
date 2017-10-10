/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_timer.c
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Kazuya Hioki <Kazuya.Hioki@sony.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>
#include <nuttx/arch.h>

#include <sys/types.h>

#include <stdint.h>
#include <limits.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/timers/timer.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "cxd56_timer.h"
#include "cxd56_clock.h"

#ifdef CONFIG_TIMER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clock ********************************************************************/

/* Use timer divider of 16 */

#define TIMER_DIVIDER       (16)

/* Timer' max clock is about 10MHz (Divide max 160MHz resolution by 16) and
 * timer has 32bit counter. Therefore, the max counter is the following value
 * to avoid counter wrap around.
 */

#define TIMER_MAXTIMEOUT    (ULONG_MAX / 10)

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * timer_lowerhalf_s structure.
 */

struct cxd56_lowerhalf_s
{
  FAR const struct timer_ops_s  *ops;  /* Lower half operations */

  /* Private data */

  uint32_t base;            /* Base address of the timer */
  tccb_t   handler;         /* Current user interrupt handler */
  uint32_t timeout;         /* The current timeout value (us) */
  uint32_t clkticks;        /* actual clock ticks for current interval */
  bool     started;         /* The timer has been started */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Interrupt handling *******************************************************/

static int      cxd56_timer_interrupt(int irq, FAR void *context);

/* "Lower half" driver methods **********************************************/

static int      cxd56_start(FAR struct timer_lowerhalf_s *lower);
static int      cxd56_stop(FAR struct timer_lowerhalf_s *lower);
static int      cxd56_getstatus(FAR struct timer_lowerhalf_s *lower,
                                FAR struct timer_status_s *status);
static int      cxd56_settimeout(FAR struct timer_lowerhalf_s *lower,
                                 uint32_t timeout);
static tccb_t   cxd56_sethandler(struct timer_lowerhalf_s *lower,
                                 tccb_t handler);
static int      cxd56_ioctl(FAR struct timer_lowerhalf_s *lower, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_tmrops =
{
  .start      = cxd56_start,
  .stop       = cxd56_stop,
  .getstatus  = cxd56_getstatus,
  .settimeout = cxd56_settimeout,
  .sethandler = cxd56_sethandler,
  .ioctl      = cxd56_ioctl,
};

/* "Lower half" driver state */

static struct cxd56_lowerhalf_s g_tmrdevs[2];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_timer_interrupt
 *
 * Description:
 *   TC interrupt
 *
 * Input Parameters:
 *   Usual interrupt callback arguments.
 *
 * Returned Values:
 *   Always returns OK.
 *
 ****************************************************************************/

static int cxd56_timer_interrupt(int irq, FAR void *context)
{
  FAR struct cxd56_lowerhalf_s *priv = &g_tmrdevs[irq - CXD56_IRQ_TIMER0];

  timdbg("Entry\n");
  DEBUGASSERT((irq >= CXD56_IRQ_TIMER0) && (irq <= CXD56_IRQ_TIMER1));

  /* Is there a registered callback?  If the callback has been
   * nullified, the timer will be stopped.
   */

  if (priv->handler)
    {
      priv->handler(&priv->timeout);
    }
  else
    {
      /* No callback or the callback returned false.. stop the timer */

      cxd56_stop((FAR struct timer_lowerhalf_s *)priv);
      timdbg("Stopped\n");
    }

  /* Clear the interrupts */

  putreg32(TIMER_INTERRUPT, priv->base + CXD56_TIMER_INTCLR);

  return OK;
}

/****************************************************************************
 * Name: cxd56_start
 *
 * Description:
 *   Start the timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_start(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct cxd56_lowerhalf_s *priv = (FAR struct cxd56_lowerhalf_s *)lower;

  timvdbg("Entry: started %d\n", priv->started);

  /* Has the timer already been started? */

  if (!priv->started)
    {
      uint32_t ctrl = (TIMERCTRL_ENABLE | TIMERCTRL_DIV_16 |
                       TIMERCTRL_SIZE_32BIT | TIMERCTRL_MODE_WRAP);

      if (priv->timeout)
        {
          ctrl |= (TIMERCTRL_PERIODIC | TIMERCTRL_INTENABLE);
        }
      else
        {
          ctrl |= (TIMERCTRL_FREERUN | TIMERCTRL_INTDISABLE);
        }

      /* Start the timer */

      putreg32(ctrl, priv->base + CXD56_TIMER_CONTROL);

      priv->started = true;
      return OK;
    }

  /* Return EBUSY to indicate that the timer was already running */

  return -EBUSY;
}

/****************************************************************************
 * Name: cxd56_stop
 *
 * Description:
 *   Stop the timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_stop(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct cxd56_lowerhalf_s *priv = (FAR struct cxd56_lowerhalf_s *)lower;

  timvdbg("Entry: started %d\n", priv->started);

  /* Has the timer already been started? */

  if (priv->started)
    {
      /* Stop the timer */

      putreg32(0, priv->base + CXD56_TIMER_CONTROL);

      /* Clear interrupt just in case */

      putreg32(TIMER_INTERRUPT, priv->base + CXD56_TIMER_INTCLR);

      priv->started = false;
      return OK;
    }

  /* Return ENODEV to indicate that the timer was not running */

  return -ENODEV;
}

/****************************************************************************
 * Name: cxd56_getstatus
 *
 * Description:
 *   Get the current timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the "lower-
 *            half" driver state structure.
 *   status - The location to return the status information.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_getstatus(FAR struct timer_lowerhalf_s *lower,
                           FAR struct timer_status_s *status)
{
  FAR struct cxd56_lowerhalf_s *priv = (FAR struct cxd56_lowerhalf_s *)lower;
  uint64_t remaining;

  timvdbg("Entry\n");
  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = 0;
  if (priv->started)
    {
      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->handler)
    {
      status->flags |= TCFLAGS_HANDLER;
    }

  /* Return the actual timeout in microseconds */

  status->timeout = priv->timeout;

  /* Get the time remaining until the timer expires (in microseconds). */

  remaining = (uint64_t)getreg32(priv->base + CXD56_TIMER_VALUE);
  status->timeleft =
    (uint32_t)((remaining * 1000000ULL) / cxd56_get_cpu_baseclk());

  timvdbg("  flags    : %08x\n", status->flags);
  timvdbg("  timeout  : %d\n", status->timeout);
  timvdbg("  timeleft : %d\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: cxd56_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower
 *             half" driver state structure.
 *   timeout - The new timeout value in milliseconds.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_settimeout(FAR struct timer_lowerhalf_s *lower,
                            uint32_t timeout)
{
  FAR struct cxd56_lowerhalf_s *priv = (FAR struct cxd56_lowerhalf_s *)lower;
  uint32_t load;

  DEBUGASSERT(priv);

  if (priv->started)
    {
      return -EPERM;
    }

  timdbg("Entry: timeout=%d\n", timeout);

  /* Can this timeout be represented? */

  if (timeout < 1 || timeout > TIMER_MAXTIMEOUT)
    {
      timdbg("ERROR: Cannot represent timeout=%lu > %lu\n", timeout,
             TIMER_MAXTIMEOUT);
      return -ERANGE;
    }

  /* Intended timeout */

  priv->timeout = timeout;

  /* Actual clock ticks */

  priv->clkticks = cxd56_get_cpu_baseclk();

  load = (((uint64_t)timeout * priv->clkticks) / TIMER_DIVIDER / 1000000);
  putreg32(load, priv->base + CXD56_TIMER_LOAD);
  modifyreg32(priv->base + CXD56_TIMER_CONTROL, 0,
              TIMERCTRL_PERIODIC | TIMERCTRL_INTENABLE);

  timdbg("clkticks=%d timeout=%d load=%d\n", priv->clkticks, priv->timeout,
         load);

  return OK;
}

/****************************************************************************
 * Name: cxd56_sethandler
 *
 * Description:
 *   Call this user provided timeout callback.
 *
 * Input Parameters:
 *   lower      - A pointer the publicly visible representation of the
 *                "lower-half" driver state structure.
 *   newhandler - The new timer expiration function pointer.  If this
 *                function pointer is NULL, then the reset-on-expiration
 *                behavior is restored,
 *
 * Returned Values:
 *   The previous timer expiration function pointer or NULL is there was
 *   no previous function pointer.
 *
 ****************************************************************************/

static tccb_t cxd56_sethandler(struct timer_lowerhalf_s *lower, tccb_t handler)
{
  FAR struct cxd56_lowerhalf_s *priv = (struct cxd56_lowerhalf_s *)lower;
  irqstate_t flags;
  tccb_t oldhandler;

  flags = enter_critical_section();

  DEBUGASSERT(priv);
  timvdbg("Entry: handler=%p\n", handler);

  /* Get the old handler return value */

  oldhandler = priv->handler;

  /* Save the new handler */

  priv->handler = handler;

  leave_critical_section(flags);
  return oldhandler;
}

/****************************************************************************
 * Name: cxd56_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half" driver
 *   are forwarded to the lower half driver through this method.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *   cmd   - The ioctl command value
 *   arg   - The optional argument that accompanies the 'cmd'.  The
 *           interpretation of this argument depends on the particular
 *           command.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_ioctl(FAR struct timer_lowerhalf_s *lower, int cmd,
                       unsigned long arg)
{
  int ret = -ENOTTY;

  timvdbg("Entry: cmd=%d arg=%ld\n", cmd, arg);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_timer_initialize
 *
 * Description:
 *   Initialize the timer.  The timer is initialized and
 *   registers as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the timer.  This should be of the form
 *     /dev/timer0
 *   timer - the timer's number.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void cxd56_timer_initialize(FAR const char *devpath, int timer)
{
  FAR struct cxd56_lowerhalf_s *priv = &g_tmrdevs[timer];
  int irq;

  timdbg("Entry: devpath=%s\n", devpath);
  DEBUGASSERT((timer >= CXD56_TIMER0) && (timer <= CXD56_TIMER1));

  /* Initialize the driver state structure.  Here we assume: (1) the state
   * structure lies in .bss and was zeroed at reset time.  (2) This function
   * is only called once so it is never necessary to re-zero the structure.
   */

  switch (timer)
    {
      case CXD56_TIMER0:
        priv->base = CXD56_TIMER0_BASE;
        irq        = CXD56_IRQ_TIMER0;
        timdbg("Using: Timer 0");
        break;

      case CXD56_TIMER1:
        priv->base = CXD56_TIMER1_BASE;
        irq        = CXD56_IRQ_TIMER1;
        timdbg("Using: Timer 1");
        break;

      default:
        ASSERT(0);
    }

  priv->ops = &g_tmrops;

  (void)irq_attach(irq, cxd56_timer_interrupt);

  /* Enable NVIC interrupt. */

  up_enable_irq(irq);

  /* Register the timer driver as /dev/timerX */

  (void)timer_register(devpath, (FAR struct timer_lowerhalf_s *)priv);
}

#endif /* CONFIG_TIMER */
