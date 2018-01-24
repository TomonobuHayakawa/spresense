/****************************************************************************
 * drivers/lcd/ili9340.c
 *
 * Driver for ILI9340 LCD.
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Librae <librae8226@gmail.com>
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
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ili9340.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Display resolution */

#define ILI9340_XRES        320
#define ILI9340_YRES        240

/* Dolor depth and format */

#define ILI9340_BPP           16
#define ILI9340_COLORFMT      FB_FMT_RGB16_565

/* Bytes per logical row and column */

#define ILI9340_XSTRIDE       (ILI9340_XRES << 1)
#define ILI9340_YSTRIDE       (ILI9340_YRES << 1)

#define ILI9340_FBSIZE        (ILI9340_XSTRIDE*ILI9340_YRES)

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* lcd data transfer methods */

static int ili9340_putrun(fb_coord_t row, fb_coord_t col,
                              FAR const uint8_t * buffer, size_t npixels);
#ifndef CONFIG_LCD_NOGETRUN
static int ili9340_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t * buffer,
                              size_t npixels);
#endif

/* lcd configuration */

static int ili9340_getvideoinfo(FAR struct lcd_dev_s *dev,
                                    FAR struct fb_videoinfo_s *vinfo);
static int ili9340_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                                    FAR struct lcd_planeinfo_s *pinfo);

/* lcd specific controls */

static int ili9340_getpower(FAR struct lcd_dev_s *dev);
static int ili9340_setpower(FAR struct lcd_dev_s *dev, int power);
static int ili9340_getcontrast(FAR struct lcd_dev_s *dev);
static int ili9340_setcontrast(FAR struct lcd_dev_s *dev, unsigned int contrast);

/* system defined functions */

extern int ili9340_draw_bitmap(FAR struct ili9340_lcd_s *lcd,
                                      FAR const uint16_t *bitmap, int16_t x, int16_t y,
                                      int16_t w, int16_t h);
extern int ili9340_turn_backlight(FAR struct ili9340_lcd_s *lcd,
                                         bool on);
extern FAR struct ili9340_base_s *ili9340_base_initialize(
    FAR struct ili9340_lcd_s *lcd, struct spi_dev_s *spi);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_runbuffer[ILI9340_BPP * ILI9340_XRES / 8];

/* This structure describes the overall lcd video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt = ILI9340_COLORFMT,            /* Color format: rgb16-565: rrrr rggg gggb bbbb */
  .xres = ILI9340_XRES,               /* Horizontal resolution in pixel columns */
  .yres = ILI9340_YRES,               /* Vertical resolution in pixel rows */
  .nplanes = 1,                           /* Number of color planes supported */
};

/* This is the standard, nuttx plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  .putrun = ili9340_putrun,           /* Put a run into lcd memory */
#ifndef CONFIG_LCD_NOGETRUN
  .getrun = ili9340_getrun,           /* Get a run from lcd memory */
#endif
  .buffer = (uint8_t *) g_runbuffer,      /* Run scratch buffer */
  .bpp = ILI9340_BPP,                 /* Bits-per-pixel */
};

struct ili9340_dev_s
{
  /* Publically visible device structure */

  struct lcd_dev_s dev;

  /* Private lcd-specific information follows */

  struct ili9340_lcd_s* lcd;
  struct spi_dev_s* spi;

  uint8_t contrast;               /* Current contrast setting */
  uint8_t power;                  /* Current power setting */

#ifndef CONFIG_LCD_NOGETRUN
  uint8_t fb[ILI9340_FBSIZE];
#endif
};

static struct ili9340_dev_s g_ili9340_dev =
{
  .dev =
    {
      /* lcd configuration */

      .getvideoinfo = ili9340_getvideoinfo,
      .getplaneinfo = ili9340_getplaneinfo,

      /* lcd specific controls */

      .getpower = ili9340_getpower,
      .setpower = ili9340_setpower,
      .getcontrast = ili9340_getcontrast,
      .setcontrast = ili9340_setcontrast,
    },
};

/* lcd data transfer methods */

int ili9340_putrun(fb_coord_t row, fb_coord_t col,
                       FAR const uint8_t * buffer, size_t npixels)
{
  int ret;
  FAR struct ili9340_dev_s *dev = (FAR struct ili9340_dev_s *)&g_ili9340_dev;

  ret = ili9340_draw_bitmap(dev->lcd, (uint16_t*)buffer, col, row, npixels, 1);

  return ret;
}

#ifndef CONFIG_LCD_NOGETRUN
int ili9340_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t * buffer,
                       size_t npixels)
{
  lcderr("getrun is not supported for now.\n");
  return OK;
}
#endif

/* lcd configuration */

int ili9340_getvideoinfo(FAR struct lcd_dev_s *dev,
                             FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdinfo("fmt: %d xres: %d yres: %d nplanes: %d\n",
          g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres,
          g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

int ili9340_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                             FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(pinfo && planeno == 0);
  lcdinfo("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/* lcd specific controls */

int ili9340_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct ili9340_dev_s *lcd = (FAR struct ili9340_dev_s *)dev;
  DEBUGASSERT(lcd);
  lcdinfo("%d\n", lcd->power);
  return lcd->power;
}

int ili9340_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct ili9340_dev_s *lcd = (FAR struct ili9340_dev_s *)dev;

  lcdinfo("%d\n", power);
  lcd->power = power;

  if (power > 0)
    {
      ili9340_turn_backlight(lcd->lcd, true);
    }
  else
    {
      ili9340_turn_backlight(lcd->lcd, false);
    }

  return OK;
}

int ili9340_getcontrast(FAR struct lcd_dev_s *dev)
{
  struct ili9340_dev_s *mlcd = (struct ili9340_dev_s *)dev;
  DEBUGASSERT(mlcd);
  lcdinfo("contrast: %d\n", mlcd->contrast);
  return mlcd->contrast;
}

int ili9340_setcontrast(FAR struct lcd_dev_s *dev, unsigned int contrast)
{
  struct ili9340_dev_s *mlcd = (struct ili9340_dev_s *)dev;
  DEBUGASSERT(mlcd);
  lcdinfo("contrast: %d\n", contrast);

  mlcd->contrast = contrast;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Initialize LCD
 ****************************************************************************/

FAR struct lcd_dev_s* ili9340_initialize(FAR struct ili9340_lcd_s *lcd, struct spi_dev_s *spi)
{
  FAR struct ili9340_dev_s *priv = &g_ili9340_dev;

  FAR struct ili9340_base_s *base = ili9340_base_initialize(lcd, spi);
  priv->lcd = base->lcd;
  priv->spi = base->spi;

  priv->lcd->init(priv->lcd);

  return &priv->dev;
}
