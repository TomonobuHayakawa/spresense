/****************************************************************************
 * configs/cxd56xx/src/cxd56_lpm013m091a.c
 *
 *   Copyright (C) 2016 Sony Corporation. All rights reserved.
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

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/lcd/lpm013m091a.h>
#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"

#if defined(CONFIG_LCD_LPM013M091A)

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* Board specific GPIO pin assignment */

#define DISPLAY_RST PIN_SPI2_MOSI /* Reset signal */
#define DISPLAY_DC  PIN_SPI2_MISO /* Data/Command signal */

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

static ResCode lpm013m091a_sendReset(FAR struct lpm013m091a_lcd_s *lcd);
static ResCode lpm013m091a_setTransferMode(FAR struct lpm013m091a_lcd_s *lcd, TransferMode mode);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lpm013m091a_lcd_s g_lcd =
{
  .sendReset = lpm013m091a_sendReset,
  .setTransferMode = lpm013m091a_setTransferMode,
};

/*******************************************************************************
 * Send Reset Command to LCD Driver
 *******************************************************************************/

static ResCode lpm013m091a_sendReset(FAR struct lpm013m091a_lcd_s *lcd)
{
  cxd56_gpio_write(DISPLAY_RST, false);

  up_mdelay(10);

  cxd56_gpio_write(DISPLAY_RST, true);

  up_mdelay(50);

  return ResCodeOk;
}

/*******************************************************************************
 * Select data or command
 * High:=> data, Low:=> command
 *******************************************************************************/

static ResCode lpm013m091a_setTransferMode(FAR struct lpm013m091a_lcd_s *lcd, TransferMode mode)
{
  bool value = (mode == TransferData) ? true : false;

  cxd56_gpio_write(DISPLAY_DC, value);

  return ResCodeOk;
}

/*******************************************************************************
 * Initialize LCD Device
 *******************************************************************************/

#if defined(CONFIG_NX_LCDDRIVER)
FAR struct lcd_dev_s *cxd56_lpm013m091a_initialize(FAR struct spi_dev_s *spi)
{
  DEBUGASSERT(spi != 0);

  FAR struct lcd_dev_s *dev = lpm013m091a_initialize(&g_lcd, spi);

  return dev;
}

#else

extern FAR struct fb_vtable_s *lpm013m091a_fb_initialize(FAR struct lpm013m091a_lcd_s *lcd, struct spi_dev_s *spi);
extern int lpm013m091a_register(FAR const char *devpath, FAR struct lpm013m091a_lcd_s *lcd);

FAR struct fb_vtable_s *cxd56_lpm013m091a_fb_initialize(FAR const char *devpath, FAR struct spi_dev_s *spi)
{
  int ret;
  DEBUGASSERT(spi != 0);

  FAR struct fb_vtable_s *dev = lpm013m091a_fb_initialize(&g_lcd, spi);

  /* Configure GPIO output pin */

  cxd56_gpio_config(DISPLAY_RST, false);
  cxd56_gpio_config(DISPLAY_DC, false);

  ret = lpm013m091a_register(devpath, &g_lcd);
  if (ret < 0)
    {
      dbg("Error registering lpm013m091a\n");
    }

  return dev;
}
#endif /* CONFIG_NX_LCDDRIVER */

#endif /* CONFIG_LCD_LPM013M091A */
