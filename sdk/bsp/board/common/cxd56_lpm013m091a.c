/****************************************************************************
 * configs/cxd56xx/src/cxd56_lpm013m091a.c
 *
 *   Copyright (C) 2016 Sony Corporation
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/lpm013m091a.h>
#include "cxd56_gpio.h"
#include "cxd56_spi.h"
#include "cxd56_pinconfig.h"
#include "pinassign.h"

#if defined(CONFIG_LCD_LPM013M091A)

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

static int lpm013m091a_send_reset(FAR struct lpm013m091a_lcd_s *lcd);
static int lpm013m091a_set_transfer_mode(FAR struct lpm013m091a_lcd_s *lcd,
                                         lpm013m091a_trans_mode_t mode);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lcd_dev_s *g_lcd;

static struct lpm013m091a_lcd_s g_lpm013m091a =
{
  .send_reset = lpm013m091a_send_reset,
  .set_transfer_mode = lpm013m091a_set_transfer_mode,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*******************************************************************************
 * Send Reset Command to LCD Driver
 *******************************************************************************/

static int lpm013m091a_send_reset(FAR struct lpm013m091a_lcd_s *lcd)
{
  cxd56_gpio_write(LPM013M091A_RST, false);

  up_mdelay(10);

  cxd56_gpio_write(LPM013M091A_RST, true);

  up_mdelay(50);

  return OK;
}

/*******************************************************************************
 * Select data or command
 * High:=> data, Low:=> command
 *******************************************************************************/

static int lpm013m091a_set_transfer_mode(FAR struct lpm013m091a_lcd_s *lcd, lpm013m091a_trans_mode_t mode)
{
  bool value = (mode == TRANSFER_DATA) ? true : false;

  cxd56_gpio_write(LPM013M091A_DC, value);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern FAR struct fb_vtable_s *lpm013m091a_fb_initialize(FAR struct lpm013m091a_lcd_s *lcd, struct spi_dev_s *spi);
extern int lpm013m091a_register(FAR const char *devpath, FAR struct lpm013m091a_lcd_s *lcd);

#if defined(CONFIG_NX_LCDDRIVER)
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
  lcdinfo("Initializing lcd\n");

  if (g_lcd == NULL)
    {
      /* globally initialize spi bus for peripherals */

      lcdinfo("initialize spi %d.\n", DISPLAY_SPI);
      FAR struct spi_dev_s *spi = cxd56_spibus_initialize(DISPLAY_SPI);
      if (!spi)
        {
          lcderr("ERROR: Failed to initialize spi bus.\n");
          return -ENODEV;
        }

      DEBUGASSERT(spi != 0);
      g_lcd = lpm013m091a_initialize(&g_lpm013m091a, spi);
    }

  DEBUGASSERT(g_lcd);

  return OK;
}

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
  int ret;
  FAR struct fb_vtable_s *dev;

  lcdinfo("Initializing lcd\n");

  /* globally initialize spi bus for peripherals */

  lcdinfo("initialize spi %d.\n", DISPLAY_SPI);
  FAR struct spi_dev_s *spi = cxd56_spibus_initialize(DISPLAY_SPI);
  if (!spi)
    {
      lcderr("ERROR: Failed to initialize spi bus.\n");
      return NULL;
    }

  DEBUGASSERT(spi != 0);
  dev = lpm013m091a_fb_initialize(&g_lpm013m091a, spi);

  /* Configure GPIO output pin */

  cxd56_gpio_config(LPM013M091A_RST, false);
  cxd56_gpio_config(LPM013M091A_DC, false);

  ret = lpm013m091a_register("/dev/lcd0", &g_lpm013m091a);
  if (ret < 0)
    {
      lcderr("Error registering lpm013m091a\n");
    }

  return dev;

}
#endif /* CONFIG_NX_LCDDRIVER */

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


#endif /* CONFIG_LCD_LPM013M091A */
