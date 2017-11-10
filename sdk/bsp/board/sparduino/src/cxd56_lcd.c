/****************************************************************************
 * configs/corvo/src/cxd56_lcd.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Modified: Librae <librae8226@gmail.com>
 *   Modified: Tetsuro Itabashi <Tetsuro.x.Itabashi@sony.com>
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
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#if defined(CONFIG_LCD_LPM013M091A)
#include <nuttx/lcd/lpm013m091a.h>
#endif
#if defined(CONFIG_EINK_ET014TT1)
#include <nuttx/lcd/et014tt1.h>
#endif

#include "sparduino.h"
#include "cxd56_spi.h"

/* Debug ********************************************************************/

/* Define CONFIG_DEBUG_LCD to enable detailed LCD debug output. Verbose debug must
 * also be enabled.
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#  undef CONFIG_DEBUG_LCD
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_LCD
#endif

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg(format, ...)  vdbg(format, ##__VA_ARGS__)
#else
#  define lcddbg(x...)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lcd_dev_s *g_lcd;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(CONFIG_SPI) && defined(CONFIG_CXD56_SPI4)

#if defined(CONFIG_NX_LCDDRIVER)
#if defined(CONFIG_LCD_LPM013M091A)
/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is
 *   fully initialized, display memory cleared, and the LCD ready to use,
 *   but with the power setting at 0 (full off).
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  lcddbg("Initializing lcd\n");

  if (g_lcd == NULL)
    {
      /* globally initialize spi bus for peripherals */

      lcddbg("initialize spi 4.\n");
      FAR struct spi_dev_s *spi = cxd56_spibus_initialize(4);
      if (!spi)
        {
          lcddbg("ERROR: Failed to initialize spi bus.\n");
          return -ENODEV;
        }

      g_lcd = cxd56_lpm013m091a_initialize(spi);
    }

  DEBUGASSERT(g_lcd);

  return OK;
}

#endif /* CONFIG_LCD_LPM013M091A */
#else /* CONFIG_NX_LCDDRIVER */
/****************************************************************************
 * Name: board_graphics_setup
 *
 * Description:
 *   Called by NX initialization logic to configure the LCD
 *
 ****************************************************************************/

FAR struct fb_vtable_s *board_graphics_setup(unsigned int devno)
{
  dbg("Initializing lcd\n");

  /* globally initialize spi bus for peripherals */

  dbg("initialize spi 4.\n");
  FAR struct spi_dev_s *spi = cxd56_spibus_initialize(4);
  if (!spi)
    {
      dbg("ERROR: Failed to initialize spi bus.\n");
      return NULL;
    }

#if defined(CONFIG_LCD_LPM013M091A)
  return cxd56_lpm013m091a_fb_initialize("/dev/lcd0", spi);
#endif
#if defined(CONFIG_EINK_ET014TT1)
  return cxd56_et014tt1_initialize("/dev/lcd0", spi);
#endif

}
#endif /* CONFIG_NX_LCDDRIVER */
#endif /* CONFIG_SPI / CONFIG_CXD56_SPI4 */

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This
 *   allows support for multiple LCD devices.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  DEBUGASSERT(lcddev == 0);
  return g_lcd;
}
