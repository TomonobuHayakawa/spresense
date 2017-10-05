/****************************************************************************
 * configs/collet/src/cxd56_buttons.c
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Hitoshi Fukuda <Hitoshi.Fukuda@sony.com>
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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "collet.h"
#include "cxd56_gpio.h"
#include "cxd56_gpioint.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Pin configuration for each CXD56XX button.  This array is indexed
 * by the GPIO_* definitions in collet.h
 */

static const uint16_t g_buttoncfg[BOARD_NUM_BUTTONS] =
{
  GPIO_BUT1, GPIO_BUT2
};

/* This array defines all of the interrupt handlers current attached to
 * button events.
 */

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_CXD56_GPIO_IRQ)
static xcpt_t g_buttonisr[BOARD_NUM_BUTTONS];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.  After
 *   that, board_buttons() may be called to collect the current state of all
 *   buttons or board_button_irq() may be called to register button interrupt
 *   handlers.
 *
 ****************************************************************************/

void board_button_initialize(void)
{
  cxd56_gpio_config(GPIO_BUT1, true);
  cxd56_gpio_config(GPIO_BUT2, true);
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.  After
 *   that, board_buttons() may be called to collect the current state of all
 *   buttons.
 *
 *   board_buttons() may be called at any time to harvest the state of every
 *   button.  The state of the buttons is returned as a bitset with one
 *   bit corresponding to each button:  If the bit is set, then the button
 *   is pressed.  See the BOARD_BUTTON_*_BIT and BOARD_JOYSTICK_*_BIT
 *   definitions in board.h for the meaning of each bit.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint8_t ret = 0;
  int i;

  /* Check that state of each key */

  for (i = 0; i < BOARD_NUM_BUTTONS; i++)
    {
       /* A low value means that the key is pressed. */

       bool depressed = !cxd56_gpio_read(g_buttoncfg[i]);

       /* Accumulate the set of depressed (not released) keys */

       if (depressed)
         {
            ret |= (1 << i);
         }
    }

  return ret;
}

/****************************************************************************
 * Button support.
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.  After
 *   that, board_button_irq() may be called to register button interrupt handlers.
 *
 *   board_button_irq() may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is a
 *   button enumeration value that uniquely identifies a button resource. See the
 *   BOARD_BUTTON_* and BOARD_JOYSTICK_* definitions in board.h for the meaning
 *   of enumeration values.  The previous interrupt handler address is returned
 *   (so that it may restored, if so desired).
 *
 *   Note that board_button_irq() also enables button interrupts.  Button
 *   interrupts will remain enabled after the interrupt handler is attached.
 *   Interrupts may be disabled (and detached) by calling board_button_irq with
 *   irqhandler equal to NULL.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
#ifdef CONFIG_CXD56_GPIO_IRQ
  irqstate_t flags;

  /* Verify that the button ID is within range */

  if ((unsigned)id < BOARD_NUM_BUTTONS)
    {
      /* Return the current button handler and set the new interrupt handler */

      g_buttonisr[id] = irqhandler;

      /* Disable interrupts until we are done */

      flags = enter_critical_section();

      /* Configure the interrupt.  Either attach and enable the new
       * interrupt or disable and detach the old interrupt handler.
       */

      if (irqhandler)
        {
          /* Attach then enable the new interrupt handler */

          cxd56_gpioint_config(g_buttoncfg[id],
                               GPIOINT_TOGGLE_MODE_MASK |
                               GPIOINT_NOISE_FILTER_ENABLE |
                               GPIOINT_LEVEL_HIGH, irqhandler);

          cxd56_gpioint_enable(g_buttoncfg[id]);
        }
      else
        {
          /* Disable then detach the old interrupt handler */

          cxd56_gpioint_disable(g_buttoncfg[id]);
        }
      leave_critical_section(flags);
    }
#else
  _err("ERROR: Not found gpio interrupt driver.\n");
#endif
  return OK;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
