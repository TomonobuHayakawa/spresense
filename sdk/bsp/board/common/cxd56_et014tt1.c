/****************************************************************************
 * configs/cxd56xx/src/cxd56_et014tt1.c
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Kei Yamamoto <Kei.x.Yamamoto@sony.com>
 *           Tomonobu Hayakawa <Tomonobu.Hayakawa@sony.com>
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/time.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"
#include "cxd56_et014tt1.h"

#if defined(CONFIG_EINK_ET014TT1)

FAR struct fb_vtable_s *cxd56_et014tt1_initialize(FAR const char *devpath,
                    FAR struct spi_dev_s *spi);
int EPD_HAL_Init(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct spi_dev_s *g_spi = NULL;
static struct et014tt1_pin_s g_pin =
{
#ifdef CONFIG_ARCH_BOARD_COLLET
  .rst = PIN_SPI2_CS_X,         /* rst */
  .busy = PIN_SPI2_SCK,         /* busy */
  .cs = PIN_SEN_IRQ_IN,         /* cs */
  .oei = -1,                    /* oei */
  .power = -1                   /* power */
#else
   .rst = PIN_PWM2,              /* rst */
   .busy = PIN_SPI3_CS1_X,       /* busy */
   .cs = PIN_PWM3,               /* cs */
   .oei = -1,                    /* oei */
   .power = -1                   /* power */
#endif
};

/****************************************************************************
 * Name: cxd56_et014tt1_initialize
 *
 * Initialize LCD Device
 *
 *******************************************************************************/

FAR struct fb_vtable_s *cxd56_et014tt1_initialize(FAR const char *devpath, FAR struct spi_dev_s *spi)
{
  int ret;
  DEBUGASSERT(spi != 0);

  g_spi = spi;

  FAR struct fb_vtable_s *dev = et014tt1_initialize(g_spi, &g_pin);

#ifdef CONFIG_ARCH_BOARD_COLLET
  board_power_control(POWER_EINK, true);
#endif

  ret = EPD_HAL_Init();
  if (ret < 0)
    {
      dbg("Error EPD_HAL_Init\n");
    }

  ret = et014tt1_register(devpath);
  if (ret < 0)
    {
      dbg("Error et014tt1_register\n");
    }

  return dev;
}

/****************************************************************************
 * Name: EPD_HAL_Init
 *
 * Initialize HAL
 *
 *******************************************************************************/

int EPD_HAL_Init(void)
{
  et014tt1_configspi(g_spi);

  /* Configure GPIO output pin */

  cxd56_gpio_config(g_pin.rst, false);
  cxd56_gpio_config(g_pin.busy, true);
  cxd56_gpio_config(g_pin.cs, false);

  return 0;
}

#endif /* CONFIG_EINK_ET014TT1 */

