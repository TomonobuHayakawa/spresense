/****************************************************************************
 * drivers/lcd/lpm013m091a.c
 *
 * Driver for LPM013M091A LCD.
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Librae <librae8226@gmail.com>
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
#include <nuttx/lcd/lpm013m091a.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Display resolution */

#define LPM013M091A_XRES        320
#define LPM013M091A_YRES        300

/* Dolor depth and format */

#define LPM013M091A_BPP           16
#define LPM013M091A_COLORFMT      FB_FMT_RGB16_565

/* Bytes per logical row and column */

#define LPM013M091A_XSTRIDE       (LPM013M091A_XRES << 1)
#define LPM013M091A_YSTRIDE       (LPM013M091A_YRES << 1)

#define LPM013M091A_FBSIZE        (LPM013M091A_XSTRIDE*LPM013M091A_YRES)

/* Debug */

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg(format, ...)  dbg(format, ##__VA_ARGS__)
#  define lcdvdbg(format, ...) vdbg(format, ##__VA_ARGS__)
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* lcd data transfer methods */

static int lpm013m091a_putrun(fb_coord_t row, fb_coord_t col,
                              FAR const uint8_t * buffer, size_t npixels);
#ifndef CONFIG_LCD_NOGETRUN
static int lpm013m091a_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t * buffer,
                              size_t npixels);
#endif

/* lcd configuration */

static int lpm013m091a_getvideoinfo(FAR struct lcd_dev_s *dev,
                                    FAR struct fb_videoinfo_s *vinfo);
static int lpm013m091a_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                                    FAR struct lcd_planeinfo_s *pinfo);

/* lcd specific controls */

static int lpm013m091a_getpower(FAR struct lcd_dev_s *dev);
static int lpm013m091a_setpower(FAR struct lcd_dev_s *dev, int power);
static int lpm013m091a_getcontrast(FAR struct lcd_dev_s *dev);
static int lpm013m091a_setcontrast(FAR struct lcd_dev_s *dev, unsigned int contrast);

/* system defined functions */

extern int lpm013m091a_draw_bitmap(FAR struct lpm013m091a_lcd_s *lcd,
                                      FAR const uint16_t *bitmap, int16_t x, int16_t y,
                                      int16_t w, int16_t h);
extern int lpm013m091a_turn_backlight(FAR struct lpm013m091a_lcd_s *lcd,
                                         bool on);
extern FAR struct lpm013m091a_base_s *lpm013m091a_base_initialize(
    FAR struct lpm013m091a_lcd_s *lcd, struct spi_dev_s *spi);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_runbuffer[LPM013M091A_BPP * LPM013M091A_XRES / 8];

/* This structure describes the overall lcd video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt = LPM013M091A_COLORFMT,            /* Color format: rgb16-565: rrrr rggg gggb bbbb */
  .xres = LPM013M091A_XRES,               /* Horizontal resolution in pixel columns */
  .yres = LPM013M091A_YRES,               /* Vertical resolution in pixel rows */
  .nplanes = 1,                           /* Number of color planes supported */
};

/* This is the standard, nuttx plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  .putrun = lpm013m091a_putrun,           /* Put a run into lcd memory */
#ifndef CONFIG_LCD_NOGETRUN
  .getrun = lpm013m091a_getrun,           /* Get a run from lcd memory */
#endif
  .buffer = (uint8_t *) g_runbuffer,      /* Run scratch buffer */
  .bpp = LPM013M091A_BPP,                 /* Bits-per-pixel */
};

struct lpm013m091a_dev_s
{
  /* Publically visible device structure */

  struct lcd_dev_s dev;

  /* Private lcd-specific information follows */

  struct lpm013m091a_lcd_s* lcd;
  struct spi_dev_s* spi;

  uint8_t contrast;               /* Current contrast setting */
  uint8_t power;                  /* Current power setting */

#ifndef CONFIG_LCD_NOGETRUN
  uint8_t fb[LPM013M091A_FBSIZE];
#endif
};

static struct lpm013m091a_dev_s g_lpm013m091a_dev =
{
  .dev =
    {
      /* lcd configuration */

      .getvideoinfo = lpm013m091a_getvideoinfo,
      .getplaneinfo = lpm013m091a_getplaneinfo,

      /* lcd specific controls */

      .getpower = lpm013m091a_getpower,
      .setpower = lpm013m091a_setpower,
      .getcontrast = lpm013m091a_getcontrast,
      .setcontrast = lpm013m091a_setcontrast,
    },
};

/* lcd data transfer methods */

int lpm013m091a_putrun(fb_coord_t row, fb_coord_t col,
                       FAR const uint8_t * buffer, size_t npixels)
{
  int ret;
  FAR struct lpm013m091a_dev_s *dev = (FAR struct lpm013m091a_dev_s *)&g_lpm013m091a_dev;

  ret = lpm013m091a_draw_bitmap(dev->lcd, (uint16_t*)buffer, col, row, npixels, 1);

  return ret;
}

#ifndef CONFIG_LCD_NOGETRUN
int lpm013m091a_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t * buffer,
                       size_t npixels)
{
  lcddbg("getrun is not supported for now.\n");
  return OK;
}
#endif

/* lcd configuration */

int lpm013m091a_getvideoinfo(FAR struct lcd_dev_s *dev,
                             FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdvdbg("fmt: %d xres: %d yres: %d nplanes: %d\n",
          g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres,
          g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

int lpm013m091a_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                             FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(pinfo && planeno == 0);
  lcdvdbg("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/* lcd specific controls */

int lpm013m091a_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct lpm013m091a_dev_s *lcd = (FAR struct lpm013m091a_dev_s *)dev;
  DEBUGASSERT(lcd);
  lcddbg("%d\n", lcd->power);
  return lcd->power;
}

int lpm013m091a_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct lpm013m091a_dev_s *lcd = (FAR struct lpm013m091a_dev_s *)dev;

  lcddbg("%d\n", power);
  lcd->power = power;

  if (power > 0)
    {
      lpm013m091a_turn_backlight(lcd->lcd, true);
    }
  else
    {
      lpm013m091a_turn_backlight(lcd->lcd, false);
    }

  return OK;
}

int lpm013m091a_getcontrast(FAR struct lcd_dev_s *dev)
{
  struct lpm013m091a_dev_s *mlcd = (struct lpm013m091a_dev_s *)dev;
  DEBUGASSERT(mlcd);
  lcddbg("contrast: %d\n", mlcd->contrast);
  return mlcd->contrast;
}

int lpm013m091a_setcontrast(FAR struct lcd_dev_s *dev, unsigned int contrast)
{
  struct lpm013m091a_dev_s *mlcd = (struct lpm013m091a_dev_s *)dev;
  DEBUGASSERT(mlcd);
  lcddbg("contrast: %d\n", contrast);

  mlcd->contrast = contrast;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Initialize LCD
 ****************************************************************************/

FAR struct lcd_dev_s* lpm013m091a_initialize(FAR struct lpm013m091a_lcd_s *lcd, struct spi_dev_s *spi)
{
  FAR struct lpm013m091a_dev_s *priv = &g_lpm013m091a_dev;

  FAR struct lpm013m091a_base_s *base = lpm013m091a_base_initialize(lcd, spi);
  priv->lcd = base->lcd;
  priv->spi = base->spi;

  priv->lcd->init(priv->lcd);

  return &priv->dev;
}
