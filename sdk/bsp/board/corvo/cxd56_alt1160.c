/****************************************************************************
 * configs/corvo/src/cxd56_alt1160.c
 *
 *   Copyright (C) 2016 Sony Corporation. All rights reserved.
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/modem/alt1160.h>
#if defined(CONFIG_MODEM_ALT_1160) && defined(CONFIG_CXD56_GPIO_IRQ)
#  include <arch/board/board.h>
#  include "cxd56_gpio.h"
#  include "cxd56_gpioint.h"
#  include "cxd56_pinconfig.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_MODEM_ALT_1160) 
#  if !defined(CONFIG_CXD56_GPIO_IRQ)
#    error "CONFIG_CXD56_GPIO_IRQ is not defined in the configuration"
#  endif
#  if !defined(CONFIG_CXD56_SPI5)
#    error "CONFIG_CXD56_SPI5 is not defined in the configuration"
#  endif
#endif

#if defined(CONFIG_MODEM_ALT_1160) && defined(CONFIG_CXD56_GPIO_IRQ)

#define ALT1160_SHUTDOWN (PIN_PWM1)
#define MODEM_WAKEUP     (PIN_AP_CLK)
#define MASTER_REQUEST   (PIN_EMMC_DATA2)
#define SLAVE_REQUEST    (PIN_EMMC_DATA3)
#define NUM_OF_PINS      (sizeof(pincfg) / sizeof(struct alt1160_pincfg))

#endif

 /****************************************************************************
 * Private Types
 ****************************************************************************/

#if defined(CONFIG_MODEM_ALT_1160) && defined(CONFIG_CXD56_GPIO_IRQ)

struct alt1160_pincfg
{
  uint32_t pin;
  bool input_enable;
  bool init_val;
};

#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_MODEM_ALT_1160) && defined(CONFIG_CXD56_GPIO_IRQ)

static const struct alt1160_pincfg pincfg[] =
{
  { MODEM_WAKEUP, false, false },     /* out, low */
  { MASTER_REQUEST, false, false },   /* out, low */
  { SLAVE_REQUEST, true, false }      /* in, low */
};

#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_SPI) && defined(CONFIG_CXD56_SPI5) && defined(CONFIG_MODEM_ALT_1160)

/****************************************************************************
 * Name: spi_pincontrol
 *
 * Description:
 *   Configure the SPI pin
 *
 * Input Parameter:
 *   on - true: enable pin, false: disable pin
 *
 ****************************************************************************/

static void spi_pincontrol(bool on)
{
  if (on)
    {
      CXD56_PIN_CONFIGS(PINCONFS_EMMCA_SPI5);
    }
  else
    {
      CXD56_PIN_CONFIGS(PINCONFS_EMMCA_GPIO);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_alt1160initialize
 *
 * Description:
 *   Register device of the alt1160 modem
 *
 ****************************************************************************/

int cxd56_alt1160initialize(FAR const char *devpath, FAR struct spi_dev_s* spi)
{
  int ret;

  dbg("Initializing ALT1160..\n");

  ret = alt1160_register(devpath, spi);
  if (ret < 0)
  {
    dbg("Error registering ALT1160\n");
  }

  /* disable the SPI pin */

  spi_pincontrol(false);

#ifndef CONFIG_ARCH_BOARD_COLLET

  /* alt1160 shutdown (high) */

  cxd56_gpio_config(ALT1160_SHUTDOWN, false);
  cxd56_gpio_write(ALT1160_SHUTDOWN, true);

#endif

  return ret;
}

#endif

#if defined(CONFIG_MODEM_ALT_1160) && defined(CONFIG_CXD56_GPIO_IRQ)

/****************************************************************************
 * Name: board_alt1160_power_control
 *
 * Description:
 *   Power on/off the alt1160 device on the board.
 *
 ****************************************************************************/

void board_alt1160_power_control(bool en)
{
  int i;

  if (en)
    {
      /* power on ALT1160 device */

      cxd56_gpio_write(ALT1160_SHUTDOWN, true);

      board_power_control(POWER_LTE, true);

      cxd56_gpio_write(ALT1160_SHUTDOWN, false);

      for (i = 0; i < NUM_OF_PINS; i++ )
        {
          /* input pin: input enable */

          cxd56_gpio_config(pincfg[i].pin, pincfg[i].input_enable);

          /* if it is an output pin, write a default value */

          if (pincfg[i].input_enable == false)
            {
              cxd56_gpio_write(pincfg[i].pin, pincfg[i].init_val);
            }
        }

      /* Slave request seems to float in Lite Hibernation
       * and becomes HIGH at some times when it should stay LOW.
       */

      cxd56_pin_config(PINCONF_SET(SLAVE_REQUEST,
                                   PINCONF_MODE0,
                                   PINCONF_INPUT_ENABLE,
                                   PINCONF_DRIVE_NORMAL,
                                   PINCONF_PULLDOWN));

      /* enable the SPI pin */

      spi_pincontrol(true);
    }
  else
    {
      for (i = 0; i < NUM_OF_PINS; i++ )
        {
          /* input disable, output disable(Hi-z) */

          cxd56_gpio_config(pincfg[i].pin, false);
        }

      /* disable the SPI pin */

      spi_pincontrol(false);

      /* power off ALT1160 device */

      cxd56_gpio_write(ALT1160_SHUTDOWN, true);

      board_power_control(POWER_LTE, false);

#ifdef CONFIG_ARCH_BOARD_COLLET

      cxd56_gpio_config(ALT1160_SHUTDOWN, false);

#endif
    }
}

/****************************************************************************
 * Name: board_alt1160_gpio_write
 *
 * Description:
 *   Write GPIO pin.
 *
 ****************************************************************************/

void board_alt1160_gpio_write(uint32_t pin, bool value)
{
  if (pin < NUM_OF_PINS)
    {
      if (pincfg[pin].input_enable == false)
        {
          cxd56_gpio_write(pincfg[pin].pin, value);
        }
    }
}

/****************************************************************************
 * Name: board_alt1160_gpio_read
 *
 * Description:
 *   Read GPIO pin.
 *
 ****************************************************************************/

bool board_alt1160_gpio_read(uint32_t pin)
{
  bool val = false;

  if (pin < NUM_OF_PINS)
    {
      if (pincfg[pin].input_enable == true)
        {
          val = cxd56_gpio_read(pincfg[pin].pin);
        }
    }

  return val;
}

/****************************************************************************
 * Name: board_alt1160_gpio_irq
 *
 * Description:
 *   Register GPIO irq.
 *
 ****************************************************************************/

void board_alt1160_gpio_irq(uint32_t pin, uint32_t polarity,
                            uint32_t noise_filter, xcpt_t irqhandler)
{
  uint32_t pol;
  uint32_t nf;

  switch(polarity)
    {
      case ALT1160_GPIOINT_LEVEL_HIGH:
        pol = GPIOINT_LEVEL_HIGH;
        break;

      case ALT1160_GPIOINT_LEVEL_LOW:
        pol = GPIOINT_LEVEL_LOW;
        break;

      case ALT1160_GPIOINT_EDGE_RISE:
        pol = GPIOINT_EDGE_RISE;
        break;

      case ALT1160_GPIOINT_EDGE_FALL:
        pol = GPIOINT_EDGE_FALL;
        break;

      case ALT1160_GPIOINT_EDGE_BOTH:
        pol = GPIOINT_EDGE_BOTH;
        break;

      default:
        return;
        break;
    }
  if (noise_filter == ALT1160_GPIOINT_NOISE_FILTER_ENABLE)
    {
      nf = GPIOINT_NOISE_FILTER_ENABLE;
    }
  else
    {
      nf = GPIOINT_NOISE_FILTER_DISABLE;
    }
  if (pin < NUM_OF_PINS)
    {
      if (pincfg[pin].input_enable == true)
        {
          if (irqhandler)
            {
              /* Attach then enable the new interrupt handler */

              cxd56_gpioint_config(pincfg[pin].pin,
                                   (GPIOINT_TOGGLE_MODE_MASK | nf | pol),
                                   irqhandler);
            }
        }
    }
}

/****************************************************************************
 * Name: board_alt1160_gpio_int_control
 *
 * Description:
 *   Enable or disable GPIO interrupt.
 *
 ****************************************************************************/

void board_alt1160_gpio_int_control(uint32_t pin, bool en)
{
  if (pin < NUM_OF_PINS)
    {
      if (pincfg[pin].input_enable == true)
        {
          if (en)
            {
              /* enable interrupt */

              cxd56_gpioint_enable(pincfg[pin].pin);
            }
          else
            {
              /* disable interrupt */

              cxd56_gpioint_disable(pincfg[pin].pin);
            }
        }
    }
  
}

#endif

