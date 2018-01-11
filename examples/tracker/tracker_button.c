/****************************************************************************
 * examples/tracker/tracker_button.c
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Yutaka Miyajima <Yutaka.Miyajima@sony.com>
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
 * NOTE: This test exercises internal button driver interfaces.  As such, it
 * it relies on internal OS interfaces that are not normally available to a
 * user-space program.  As a result, this example cannot be used if a
 * NuttX is built as a protected, supervisor kernel (CONFIG_BUILD_PROTECTED or
 * CONFIG_BUILD_KERNEL).
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_ARCH_BUTTONS
#  error "CONFIG_ARCH_BUTTONS is not defined in the configuration"
#endif

#define BUTTONS_NAME0 "BUTTON0"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef void (* user_handler)(void);

struct button_info_s
{
  FAR const char *name; /* Name for the button */
  xcpt_t handler;       /* Button interrupt handler */
  user_handler usr_hdr; /* User callback handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void show_buttons(uint8_t oldset, uint8_t newset);

static void button_handler(int id, int irq);
static int button0_handler(int irq, FAR void *context);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Button Names */

static struct button_info_s g_buttoninfo =
{
  BUTTONS_NAME0,
  button0_handler,
  NULL
};

/* Last sampled button set */

static uint8_t g_oldset;
static uint8_t g_first = 1;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_buttons(uint8_t oldset, uint8_t newset)
{
  uint8_t chgset = oldset ^ newset;

  /* Show each button state change */

  uint8_t mask = (1 << 0);
  if ((chgset & mask) != 0)
    {
      FAR const char *state;

      /* Get the button state */

      if ((newset & mask) != 0)
        {
          state = "depressed";
        }
      else
        {
          state = "released";
        }

      /* Use lowsyslog() because we make be executing from an
       * interrupt handler.
       */

      lowsyslog(LOG_INFO, "  %s %s\n",
                g_buttoninfo.name, state);
    }
}

static void button_handler(int id, int irq)
{
  uint8_t newset = board_buttons();

  lowsyslog(LOG_INFO, "IRQ:%d Button %d:%s SET:%02x:\n",
            irq, id, g_buttoninfo.name, newset);

  show_buttons(g_oldset, newset);
  g_oldset = newset;
}

static int button0_handler(int irq, FAR void *context)
{
  button_handler(0, irq);
  if (g_buttoninfo.usr_hdr)
    {
      if (g_first == 1)
        {
          g_first = 0;
        }
      else
        {
          g_buttoninfo.usr_hdr();
        }
    }
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * register_button_handler
 ****************************************************************************/

int register_button_handler(int id, void (*user_hdr)(void))
{
  /* Set user callback handler */

  if (g_buttoninfo.usr_hdr == NULL)
    {
      g_buttoninfo.usr_hdr = user_hdr;
    }
  else
    {
      return -1;
    }

  /* Initialize the button GPIOs */

  board_button_initialize();

  /* Register to receive button interrupts */

  xcpt_t oldhandler = board_button_irq(0, g_buttoninfo.handler);

  /* Use lowsyslog() for compatibility with interrupt handler output. */

  lowsyslog(LOG_INFO, "Attached handler at %p to button %d [%s], oldhandler:%p\n",
            g_buttoninfo.handler, 0,
            g_buttoninfo.name, oldhandler);

  /* Some hardware multiplexes different GPIO button sources to the same
   * physical interrupt.  If we register multiple such multiplexed button
   * interrupts, then the second registration will overwrite the first.  In
   * this case, the first button interrupts may be aliased to the second
   * interrupt handler (or worse, could be lost).
   */

  if (oldhandler != NULL)
    {
      lowsyslog(LOG_INFO, "WARNING: oldhandler:%p is not NULL!  "
                "Button events may be lost or aliased!\n",
                oldhandler);
    }

  return 0;
}

