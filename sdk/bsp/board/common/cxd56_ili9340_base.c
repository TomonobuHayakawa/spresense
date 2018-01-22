/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_ili9340_base.c
 *
 * Device driver base for ILI9340 LCD.
 *
 *   Copyright (C) 2017 Sony Corporation
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/arch.h>

#include "cxd56_ili9340_base.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ili9340_base_s g_ili9340_base;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#define SEND_CMD_WITH_ERR_CHK(lcd, command)      \
  do {                                               \
      lcd->set_transfer_mode(lcd, TRANSFER_COMMAND);    \
      int __ret__ = lcd->send_data(lcd, command); \
      if(__ret__ != OK) {                     \
          return __ret__;                  \
      }                                              \
  } while (0)

#define SEND_DATA_WITH_ERR_CHK(lcd, data)            \
  do {                                               \
      lcd->set_transfer_mode(lcd, TRANSFER_DATA);       \
      int __ret__ = lcd->send_data(lcd, data);    \
      if(__ret__ != OK) {                     \
          return __ret__;                  \
      }                                              \
  } while (0)

static inline void ili9340_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the ILI9340 */

  SPI_SETMODE(spi, SPIDEV_MODE3);
  SPI_SETBITS(spi, 8);
  (void)SPI_HWFEATURES(spi, 0);
  (void)SPI_SETFREQUENCY(spi, _SPI_MAXFREQUENCY);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*******************************************************************************
 * LCD Driver Initialize
 *******************************************************************************/

int ili9340_init(FAR struct ili9340_lcd_s *lcd)
{
  up_mdelay(10);
  lcd->send_reset(lcd);

  SEND_CMD_WITH_ERR_CHK(lcd, 0xEF);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x03);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x80);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x02);

  SEND_CMD_WITH_ERR_CHK(lcd, 0xCF);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x00);
  SEND_DATA_WITH_ERR_CHK(lcd, 0xC1);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x30);

  SEND_CMD_WITH_ERR_CHK(lcd, 0xED);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x64);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x03);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x12);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x81);

  SEND_CMD_WITH_ERR_CHK(lcd, 0xE8);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x85);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x00);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x78);

  SEND_CMD_WITH_ERR_CHK(lcd, 0xCB);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x39);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x2C);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x00);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x34);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x02);

  SEND_CMD_WITH_ERR_CHK(lcd, 0xF7);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x20);

  SEND_CMD_WITH_ERR_CHK(lcd, 0xEA);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x00);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x00);

  SEND_CMD_WITH_ERR_CHK(lcd, ILI9340_PWCTR1); /* Power control */
  SEND_DATA_WITH_ERR_CHK(lcd, 0x23); /* VRH[5:0] */

  SEND_CMD_WITH_ERR_CHK(lcd, ILI9340_PWCTR2); /* Power control */
  SEND_DATA_WITH_ERR_CHK(lcd, 0x10); /* SAP[2:0];BT[3:0] */

  SEND_CMD_WITH_ERR_CHK(lcd, ILI9340_VMCTR1); /* VCM control */
  SEND_DATA_WITH_ERR_CHK(lcd, 0x3e);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x28);

  SEND_CMD_WITH_ERR_CHK(lcd, ILI9340_VMCTR2); /* VCM control2 */
  SEND_DATA_WITH_ERR_CHK(lcd, 0x86);

  SEND_CMD_WITH_ERR_CHK(lcd, ILI9340_TELON); /* Memory Access Control */
  SEND_DATA_WITH_ERR_CHK(lcd, 0x01);

  SEND_CMD_WITH_ERR_CHK(lcd, ILI9340_MADCTL); /* Memory Access Control */
#if ILI9340_LCDWIDTH == 240
  SEND_DATA_WITH_ERR_CHK(lcd, ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR);
#else
  /*  swap x-y for landscape view and set the color order as RGB (ILI9340_MADCTL_BGR seems to set as RGB) */
  SEND_DATA_WITH_ERR_CHK(lcd, ILI9340_MADCTL_MX |  ILI9340_MADCTL_MY | ILI9340_MADCTL_MV | ILI9340_MADCTL_BGR);
#endif
  SEND_CMD_WITH_ERR_CHK(lcd, ILI9340_PIXFMT);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x55); /* 16bit / pixel */

  SEND_CMD_WITH_ERR_CHK(lcd, ILI9340_FRMCTR1);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x00);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x18);

  SEND_CMD_WITH_ERR_CHK(lcd, 0xF2); /* 3Gamma Function Disable */
  SEND_DATA_WITH_ERR_CHK(lcd, 0x00);

  SEND_CMD_WITH_ERR_CHK(lcd, ILI9340_GAMMASET); /* Gamma curve selected */
  SEND_DATA_WITH_ERR_CHK(lcd, 0x01);

  SEND_CMD_WITH_ERR_CHK(lcd, ILI9340_GMCTRP1);  /* Set Gamma */
  SEND_DATA_WITH_ERR_CHK(lcd, 0x0F);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x31);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x2B);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x0C);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x0E);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x08);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x4E);
  SEND_DATA_WITH_ERR_CHK(lcd, 0xF1);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x37);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x07);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x10);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x03);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x0E);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x09);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x00);

  SEND_CMD_WITH_ERR_CHK(lcd, ILI9340_GMCTRN1);  /* Set Gamma */
  SEND_DATA_WITH_ERR_CHK(lcd, 0x00);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x0E);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x14);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x03);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x11);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x07);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x31);
  SEND_DATA_WITH_ERR_CHK(lcd, 0xC1);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x48);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x08);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x0F);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x0C);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x31);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x36);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x0F);

  SEND_CMD_WITH_ERR_CHK(lcd, 0xB7); /* entry mode set */
  SEND_DATA_WITH_ERR_CHK(lcd, 0x07);

  SEND_CMD_WITH_ERR_CHK(lcd, ILI9340_DFUNCTR); /* Display Function Control */
  SEND_DATA_WITH_ERR_CHK(lcd, 0x0A);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x82);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x27);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x00);

  SEND_CMD_WITH_ERR_CHK(lcd, ILI9340_IFCTRL);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x01);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x00);
  SEND_DATA_WITH_ERR_CHK(lcd, 0x00);

  SEND_CMD_WITH_ERR_CHK(lcd, ILI9340_SLPOUT); /* Exit Sleep */
  up_mdelay(120);
  SEND_CMD_WITH_ERR_CHK(lcd, ILI9340_DISPON); /* Display on */

  return OK;
}

/*******************************************************************************
 * Set Draw Address
 *******************************************************************************/

int ili9340_set_draw_area(FAR struct ili9340_lcd_s *lcd,
                                int16_t start_x, int16_t start_y,
                                int16_t end_x, int16_t end_y)
{
  if ((start_x < 0) || (start_x >= ILI9340_XRES) || (start_y < 0)
      || (start_y >= ILI9340_YRES))
    {
      return -EPERM;
    }

  if ((end_x < 0) || (end_x >= ILI9340_XRES) || (end_y < 0)
      || (end_y >= ILI9340_YRES))
    {
      return -EPERM;
    }

  SEND_CMD_WITH_ERR_CHK(lcd, ILI9340_CASET);
  SEND_DATA_WITH_ERR_CHK(lcd, start_x >> 8);
  SEND_DATA_WITH_ERR_CHK(lcd, start_x & 0xFF);
  SEND_DATA_WITH_ERR_CHK(lcd, end_x >> 8);
  SEND_DATA_WITH_ERR_CHK(lcd, end_x & 0xFF);

  SEND_CMD_WITH_ERR_CHK(lcd, ILI9340_PASET);
  SEND_DATA_WITH_ERR_CHK(lcd, start_y >> 8);
  SEND_DATA_WITH_ERR_CHK(lcd, start_y & 0xFF);
  SEND_DATA_WITH_ERR_CHK(lcd, end_y >> 8);
  SEND_DATA_WITH_ERR_CHK(lcd, end_y & 0xFF);

  SEND_CMD_WITH_ERR_CHK(lcd, ILI9340_RAMWR);

  return OK;
}

/*******************************************************************************
 * Send Bitmap Data to LCD Device
 *******************************************************************************/

int ili9340_draw_bitmap(FAR struct ili9340_lcd_s *lcd, FAR const uint16_t *bitmap,
                               int16_t x, int16_t y,
                               int16_t w, int16_t h)
{
  if ((w < 1) || (h < 1))
    {
      return -EPERM;
    }

  int code = ili9340_set_draw_area(lcd, x, y, x + w - 1, y + h - 1);
  if (code != OK)
    {
      return code;
    }

  lcd->set_transfer_mode(lcd, TRANSFER_DATA);

  return ili9340_send_data_burst(lcd, bitmap, w * h * sizeof(uint16_t));

}

/*******************************************************************************
 * Turn On/Off LCD Backlight
 *******************************************************************************/

int ili9340_turn_backlight(FAR struct ili9340_lcd_s *lcd, bool on)
{
  if (on)
    {
      SEND_CMD_WITH_ERR_CHK(lcd, 0x53);
      SEND_DATA_WITH_ERR_CHK(lcd, 0x24);
    }
  else
    {
      SEND_CMD_WITH_ERR_CHK(lcd, 0x53);
      SEND_DATA_WITH_ERR_CHK(lcd, 0x00);
    }

  return OK;
}

/*******************************************************************************
 * Send Data to LCD Device
 *******************************************************************************/

int ili9340_send_data_burst(FAR struct ili9340_lcd_s *lcd,
                                  const void *data, uint32_t size)
{
  FAR struct ili9340_base_s *dev = (FAR struct ili9340_base_s *) &g_ili9340_base;

  if (size < 1)
    {
      return -EPERM;
    }

  /* If SPI bus is shared then lock and configure it */

  (void) SPI_LOCK(dev->spi, true);
  ili9340_configspi(dev->spi);

  /* Select the ILI9340 */

  SPI_SELECT(dev->spi, SPIDEV_DISPLAY(0), true);

  /* Send register to read and get the next byte */

  (void) SPI_SNDBLOCK(dev->spi, data, size);

  /* Deselect the ILI9340 */

  SPI_SELECT(dev->spi, SPIDEV_DISPLAY(0), false);

  /* Unlock bus */

  (void) SPI_LOCK(dev->spi, false);

  return OK;
}

/*******************************************************************************
 * Send Data to LCD Device
 *******************************************************************************/

int ili9340_send_data(FAR struct ili9340_lcd_s *lcd, uint32_t data)
{
  FAR struct ili9340_base_s *dev = (FAR struct ili9340_base_s *) &g_ili9340_base;

  /* If SPI bus is shared then lock and configure it */

  (void) SPI_LOCK(dev->spi, true);
  ili9340_configspi(dev->spi);

  /* Select the ILI9340 */

  SPI_SELECT(dev->spi, SPIDEV_DISPLAY(0), true);

  /* Send register to read and get the next byte */

  (void) SPI_SEND(dev->spi, data);

  /* Deselect the ILI9340 */

  SPI_SELECT(dev->spi, SPIDEV_DISPLAY(0), false);

  /* Unlock bus */

  (void) SPI_LOCK(dev->spi, false);

  return OK;
}

FAR struct ili9340_base_s *ili9340_base_initialize(
    FAR struct ili9340_lcd_s *lcd, struct spi_dev_s *spi)
{
  FAR struct ili9340_base_s *priv = &g_ili9340_base;

  DEBUGASSERT(lcd != 0 && spi != 0);

  priv->lcd = lcd;
  priv->lcd->init = ili9340_init;
  priv->lcd->send_data = ili9340_send_data;
  priv->lcd->send_data_burst = ili9340_send_data_burst;

  priv->spi = spi;

  return priv;
}
