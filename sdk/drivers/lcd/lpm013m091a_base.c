/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_lpm013m091a_base.c
 *
 * Device driver base for LPM013M091A LCD.
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
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

#include <sdk/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/arch.h>

#include "lpm013m091a_base.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lpm013m091a_base_s g_lpm013m091a_base;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#define sendCommandWithErrorCheck(lcd, command)      \
  do {                                               \
      lcd->setTransferMode(lcd, TransferCommand);    \
      ResCode __ret__ = lcd->sendData(lcd, command); \
      if(__ret__ != ResCodeOk) {                     \
          return ResCodeSpiSendErr;                  \
      }                                              \
  } while (0)

#define sendDataWithErrorCheck(lcd, data)            \
  do {                                               \
      lcd->setTransferMode(lcd, TransferData);       \
      ResCode __ret__ = lcd->sendData(lcd, data);    \
      if(__ret__ != ResCodeOk) {                     \
          return ResCodeSpiSendErr;                  \
      }                                              \
  } while (0)

static inline void lpm013m091a_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the LPM013M091A */

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

ResCode lpm013m091a_init(FAR struct lpm013m091a_lcd_s *lcd)
{
  up_mdelay(10);
  ResCode ret = lcd->sendReset(lcd);
  if (ret == ResCodeResetErr)
    {
      return ResCodeResetErr;
    }

  /* soft reset */
  sendCommandWithErrorCheck(lcd, LPM013M091A_SWRESET);
  up_mdelay(10);

  /* Analog mode */

  sendCommandWithErrorCheck(lcd, LPM013M091A_B3);
  sendDataWithErrorCheck(lcd, 0x02);

  /* Set Display Mode */

  sendCommandWithErrorCheck(lcd, LPM013M091A_BB);
  sendDataWithErrorCheck(lcd, 0x10);

  /* SPI GRAM access enable */

  sendCommandWithErrorCheck(lcd, LPM013M091A_F3);
  sendDataWithErrorCheck(lcd, 0x02);

  /* Bright Level Max */

  sendCommandWithErrorCheck(lcd, 0x51);
  sendDataWithErrorCheck(lcd, 0xff);

  /* Backlight ON */

  sendCommandWithErrorCheck(lcd, 0x53);
  sendDataWithErrorCheck(lcd, 0x24);

  /* Frame rate 60Hz */

  sendCommandWithErrorCheck(lcd, LPM013M091A_FF);
  sendDataWithErrorCheck(lcd, 0x24);
  sendCommandWithErrorCheck(lcd, 0xD8);
  sendDataWithErrorCheck(lcd, 0x41);
  sendCommandWithErrorCheck(lcd, 0xD9);
  sendDataWithErrorCheck(lcd, 0x1E);

  sendCommandWithErrorCheck(lcd, LPM013M091A_FF);
  sendDataWithErrorCheck(lcd, 0x10);

  /* Set the color format (18bit:0x06, 16bit:0x05) */

  sendCommandWithErrorCheck(lcd, LPM013M091A_PIXFMT);
  sendDataWithErrorCheck(lcd, 0x05);

  /* sleep out */

  sendCommandWithErrorCheck(lcd, LPM013M091A_SLPOUT);
  up_mdelay(10);

  /* display on */

  sendCommandWithErrorCheck(lcd, LPM013M091A_DISPON);
  up_mdelay(120);

  return ResCodeOk;
}

/*******************************************************************************
 * Set Draw Address
 *******************************************************************************/

ResCode lpm013m091a_setDrawArea(FAR struct lpm013m091a_lcd_s *lcd,
                                int16_t start_x, int16_t start_y,
                                int16_t end_x, int16_t end_y)
{
  if ((start_x < 0) || (start_x >= LPM013M091A_XRES) || (start_y < 0)
      || (start_y >= LPM013M091A_YRES))
    {
      return ResCodeOutOfDisp;
    }

  if ((end_x < 0) || (end_x >= LPM013M091A_XRES) || (end_y < 0)
      || (end_y >= LPM013M091A_YRES))
    {
      return ResCodeOutOfDisp;
    }

  sendCommandWithErrorCheck(lcd, LPM013M091A_CASET);
  sendDataWithErrorCheck(lcd, start_x >> 8);
  sendDataWithErrorCheck(lcd, start_x & 0xFF);
  sendDataWithErrorCheck(lcd, end_x >> 8);
  sendDataWithErrorCheck(lcd, end_x & 0xFF);

  sendCommandWithErrorCheck(lcd, LPM013M091A_PASET);
  sendDataWithErrorCheck(lcd, start_y >> 8);
  sendDataWithErrorCheck(lcd, start_y & 0xFF);
  sendDataWithErrorCheck(lcd, end_y >> 8);
  sendDataWithErrorCheck(lcd, end_y & 0xFF);

  sendCommandWithErrorCheck(lcd, LPM013M091A_RAMWR);

  return ResCodeOk;
}

/*******************************************************************************
 * Send Bitmap Data to LCD Device
 *******************************************************************************/

ResCode lpm013m091a_drawBitmap(FAR struct lpm013m091a_lcd_s *lcd, FAR const uint16_t *bitmap,
                               int16_t x, int16_t y,
                               int16_t w, int16_t h)
{
  if ((w < 1) || (h < 1))
    {
      return ResCodeOutOfDisp;
    }

  ResCode code = lpm013m091a_setDrawArea(lcd, x, y, x + w - 1, y + h - 1);
  if (code != ResCodeOk)
    {
      return code;
    }

  lcd->setTransferMode(lcd, TransferData);

  ResCode ret = lpm013m091a_sendDataBurst(lcd, bitmap, w * h * sizeof(uint16_t));
  if (ret == ResCodeOk)
    {
      return ResCodeOk;
    }
  else
    {
      if (ret == ResCodeSizeErr)
        {
          return ResCodeSizeErr;
        }
      else
        {
          return ResCodeTimeOut;
        }
    }
}

/*******************************************************************************
 * Turn On/Off LCD Backlight
 *******************************************************************************/

ResCode lpm013m091a_turnBacklight(FAR struct lpm013m091a_lcd_s *lcd, bool on)
{
  if (on)
    {
      sendCommandWithErrorCheck(lcd, 0x53);
      sendDataWithErrorCheck(lcd, 0x24);
    }
  else
    {
      sendCommandWithErrorCheck(lcd, 0x53);
      sendDataWithErrorCheck(lcd, 0x00);
    }

  return ResCodeOk;
}

/*******************************************************************************
 * Send Data to LCD Device
 *******************************************************************************/

ResCode lpm013m091a_sendDataBurst(FAR struct lpm013m091a_lcd_s *lcd,
                                  const void *data, uint32_t size)
{
  FAR struct lpm013m091a_base_s *dev = (FAR struct lpm013m091a_base_s *) &g_lpm013m091a_base;

  if (size < 1)
    {
      return ResCodeSizeErr;
    }

  /* If SPI bus is shared then lock and configure it */

  (void) SPI_LOCK(dev->spi, true);
  lpm013m091a_configspi(dev->spi);

  /* Select the LPM013M091A */

  SPI_SELECT(dev->spi, SPIDEV_DISPLAY(0), true);

  /* Send register to read and get the next byte */

  (void) SPI_SNDBLOCK(dev->spi, data, size);

  /* Deselect the LPM013M091A */

  SPI_SELECT(dev->spi, SPIDEV_DISPLAY(0), false);

  /* Unlock bus */

  (void) SPI_LOCK(dev->spi, false);

  return ResCodeOk;
}

/*******************************************************************************
 * Send Data to LCD Device
 *******************************************************************************/

ResCode lpm013m091a_sendData(FAR struct lpm013m091a_lcd_s *lcd, uint32_t data)
{
  FAR struct lpm013m091a_base_s *dev = (FAR struct lpm013m091a_base_s *) &g_lpm013m091a_base;

  /* If SPI bus is shared then lock and configure it */

  (void) SPI_LOCK(dev->spi, true);
  lpm013m091a_configspi(dev->spi);

  /* Select the LPM013M091A */

  SPI_SELECT(dev->spi, SPIDEV_DISPLAY(0), true);

  /* Send register to read and get the next byte */

  (void) SPI_SEND(dev->spi, data);

  /* Deselect the LPM013M091A */

  SPI_SELECT(dev->spi, SPIDEV_DISPLAY(0), false);

  /* Unlock bus */

  (void) SPI_LOCK(dev->spi, false);

  return ResCodeOk;
}

FAR struct lpm013m091a_base_s *lpm013m091a_base_initialize(
    FAR struct lpm013m091a_lcd_s *lcd, struct spi_dev_s *spi)
{
  FAR struct lpm013m091a_base_s *priv = &g_lpm013m091a_base;

  DEBUGASSERT(lcd != 0 && spi != 0);

  priv->lcd = lcd;
  priv->lcd->init = lpm013m091a_init;
  priv->lcd->sendData = lpm013m091a_sendData;
  priv->lcd->sendDataBurst = lpm013m091a_sendDataBurst;

  priv->spi = spi;

  return priv;
}
