/****************************************************************************
 * drivers/lcd/et014tt1.c
 *
 *   Copyright (C) 2017, 2018 Sony Corporation
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
#include <sdk/config.h>

#include <sys/types.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/et014tt1.h>

#include "et014tt1.h"

/* Include waveform data */

#include "et014tt1_waveformdata.c"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/* ET014TT1 is 2 bpp grey scale device but framebuffer constructed by bytes,
 * this driver reports as 8 bpp pixel device to NX library, but each color
 * bits only LSB 2 bits, so this driver cannot show in 256 grey scale.
 */

#define ET014TT1_PXFMT   FB_FMT_Y8
#define ET014TT1_BPP     8

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct et014tt1_dev_s
{
  struct lcd_dev_s dev;

  FAR struct et014tt1_lcd_s *lcd;

  /* Device resolusion */

  fb_coord_t xres;
  fb_coord_t yres;

  /* width holds actual device width for calculate software rotation */

  uint16_t width;

  uint8_t power;

  uint8_t *runbuffer;
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/****************************************************************************
 * frame buffer info
 ****************************************************************************/

static int et014tt1_getvideoinfo(FAR struct lcd_dev_s *dev,
                                 FAR struct fb_videoinfo_s *vinfo);
static int et014tt1_getplaneinfo(FAR struct lcd_dev_s *dev,
                                 unsigned int planeno,
                                 FAR struct lcd_planeinfo_s *pinfo);

/* lcd specific controls */

static int et014tt1_getpower(struct lcd_dev_s *dev);
static int et014tt1_setpower(struct lcd_dev_s *dev, int power);
static int et014tt1_getcontrast(struct lcd_dev_s *dev);
static int et014tt1_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct et014tt1_dev_s g_lcddev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name:  et014tt1_putrun
 *
 * Description:
 *   Write a partial raster line to the LCD.
 *
 * Parameters:
 *   devno   - Number of lcd device
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be writen to the LCD
 *   npixels - The number of pixels to write to the
 *             (range: 0 < npixels <= xres-col)
 *
 * Returned Value:
 *
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

static int et014tt1_putrun(fb_coord_t row, fb_coord_t col,
                           FAR const uint8_t *buffer, size_t npixels)
{
  FAR struct et014tt1_dev_s *priv = &g_lcddev;
  FAR uint8_t *dest;
  int step;
  size_t i;

  if (!buffer)
    {
      return -EINVAL;
    }
  if (col + npixels > priv->xres || row > priv->yres)
    {
      return -EINVAL;
    }

  dest = EPD_TCON_GetUpdateBuffer();

  DEBUGASSERT(dest);

  /* Calculate starting point and update byte steps */

#if defined CONFIG_LCD_ET014TT1_LANDSCAPE
  /* Rotate 90 degrees, clock wise */

  dest += (((col + 1) * priv->width) - 1) - row;
  step = priv->width;
#elif defined CONFIG_LCD_ET014TT1_RLANDSCAPE
  /* Rotate 90 degrees, counter clock wise */

  dest += ((priv->xres - 1 - col) * priv->width) + row;
  step = -priv->width;
#elif defined CONFIG_LCD_ET014TT1_RPORTRAIT
  /* Rotate 180 degrees */

  dest += (((priv->yres - row) * priv->width) - 1) - col;
  step = -1;
#elif defined CONFIG_LCD_ET014TT1_PORTRAIT
  /* Normal */

  dest += (row * priv->width) + col;
  step = 1;
#endif

  /* Update pixels in specified line at framebuffer.
   *
   * ET014TT1 is grey 4 depths (2bpp) device, but library frame buffer
   * assigned LSB 2 bits to color, and other bits are reserved and not modify.
   * So it masking and only modify color bits.
   *
   * ePaper device updating process is too slow, so application must call
   * et014tt1_update() non standard API to reflect drawing.
   */

  for (i = 0; i < npixels; i++, dest += step)
    {
      *dest = (*dest & 0xfc) | (buffer[i] & 3);
    }

  return OK;
}

/****************************************************************************
 * Name:  et014tt1_getrun
 *
 * Description:
 *   Read a partial raster line from the LCD.
 *
 * Parameter:
 *   devno   - Number of the lcd device
 *   row     - Starting row to read from (range: 0 <= row < yres)
 *   col     - Starting column to read read (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer in which to return the run read from the LCD
 *   npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 *
 * Returned Value:
 *
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static int et014tt1_getrun(fb_coord_t row, fb_coord_t col,
                           FAR uint8_t * buffer, size_t npixels)
{
  FAR struct et014tt1_dev_s *priv = &g_lcddev;
  FAR uint8_t *src;
  size_t i;

  if (!buffer || devno != 0)
    {
      return -EINVAL;
    }
  if (col + npixels > priv->xres || row > priv->yres)
    {
      return -EINVAL;
    }

  src = EPD_TCON_GetUpdateBuffer();

  DEBUGASSERT(src);

#if defined CONFIG_LCD_ET014TT1_LANDSCAPE
  /* Rotate 90 degrees, clock wise */

  src += (((col + 1) * priv->width) - 1) - row;
  step = priv->width;
#elif defined CONFIG_LCD_ET014TT1_RLANDSCAPE
  /* Rotate 90 degrees, counter clock wise */

  src += ((priv->xres - 1 - col) * priv->width) + row;
  step = -priv->width;
#elif defined CONFIG_LCD_ET014TT1_RPORTRAIT
  /* Rotate 180 degrees */

  src += (((priv->yres - row) * priv->width) - 1) - col;
  step = -1;
#elif defined CONFIG_LCD_ET014TT1_PORTRAIT
  /* Normal */

  src += (row * priv->width) + col;
  step = 1;
#endif

  for (i = 0; i < npixels; i++, src += step)
    {
      buffer[i] = *src & 3;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name:  et014tt1_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 * Parameter:
 *   dev - A reference to the driver specific structure
 *   vinfo - A reference to the videoinfo structure
 *
 * Returned Value:
 *
 *  On success - OK
 *  On error   - -EINVAL
 *
 ****************************************************************************/

static int et014tt1_getvideoinfo(FAR struct lcd_dev_s *dev,
                                 FAR struct fb_videoinfo_s *vinfo)
{
  FAR struct et014tt1_dev_s *priv = (FAR struct et014tt1_dev_s *)dev;

  if (dev && vinfo)
    {
      vinfo->fmt     = ET014TT1_PXFMT;
      vinfo->xres    = priv->xres;
      vinfo->yres    = priv->yres;
      vinfo->nplanes = 1;

      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name:  et014tt1_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 * Parameter:
 *   dev     - A reference to the driver specific structure
 *   planeno - The plane number
 *   pinfo   - A reference to the planeinfo structure
 *
 * Returned Value:
 *
 *  On success - OK
 *  On error   - -EINVAL
 *
 ****************************************************************************/

static int et014tt1_getplaneinfo(FAR struct lcd_dev_s *dev,
                                 unsigned int planeno,
                                 FAR struct lcd_planeinfo_s *pinfo)
{
  FAR struct et014tt1_dev_s *priv = (FAR struct et014tt1_dev_s *)dev;

  if (dev && pinfo && planeno == 0)
    {
      pinfo->putrun = et014tt1_putrun;
#ifndef CONFIG_LCD_NOGETRUN
      pinfo->getrun = et014tt1_getrun;
#endif
      pinfo->bpp    = ET014TT1_BPP;
      pinfo->buffer = priv->runbuffer;

      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name:  et014tt1_getpower
 *
 * Description:
 *   ET014TT1 device uses power while updating, this function do nothing.
 *
 * Parameter:
 *   dev     - A reference to the driver specific structure
 *
 * Returned Value:
 *
 *  On success - OK
 *  On error   - -EINVAL
 *
 ****************************************************************************/

static int et014tt1_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct et014tt1_dev_s *priv = (FAR struct et014tt1_dev_s *)dev;
  return priv->power;
}

/****************************************************************************
 * Name:  et014tt1_setpower
 *
 * Description:
 *   ET014TT1 device uses power while updating, this function do nothing.
 *
 * Parameter:
 *   dev   - A reference to the driver specific structure
 *   power - Value of the power
 *
 * Returned Value:
 *
 *  On success - OK
 *  On error   - -EINVAL
 *
 ****************************************************************************/

static int et014tt1_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct et014tt1_dev_s *priv = (FAR struct et014tt1_dev_s *)dev;
  priv->power = power;
  return OK;
}

/****************************************************************************
 * Name:  et014tt1_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 * Parameter:
 *   dev   - A reference to the lcd driver structure
 *
 * Returned Value:
 *
 *  On success - current contrast value
 *  On error   - -ENOSYS, not supported by the et014tt1.
 *
 ****************************************************************************/

static int et014tt1_getcontrast(struct lcd_dev_s *dev)
{
  lcdinfo("Not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name:  et014tt1_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 * Parameter:
 *   dev   - A reference to the lcd driver structure
 *
 * Returned Value:
 *
 *  On success - OK
 *  On error   - -ENOSYS, not supported by the et014tt1.
 *
 ****************************************************************************/

static int et014tt1_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  lcdinfo("contrast: %d\n", contrast);
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: et014tt1_initialize
 *
 * Description:
 *   Initialize LCD
 *
 ****************************************************************************/

FAR struct lcd_dev_s *et014tt1_initialize(struct et014tt1_lcd_s *lcd,
                                          int devno)
{
  FAR struct et014tt1_dev_s *priv = &g_lcddev;
  FAR struct lcd_dev_s *dev = &priv->dev;
  const EPD_TCON_PANEL_INFO* info;
  uint8_t *runbuffer;
  int ret;

  if (!lcd || devno != 0)
    {
      errno = EINVAL;
      return NULL;
    }

  ret = EPD_TCON_Init((const EPD_TCON_DRIVER_HAL *)lcd);
  if (ret != 0)
    {
      lcderr("Error in EPD_TCON_Init: %d\n", ret);
      return NULL;
    }

  EPD_TCON_SetVcom(-2500);

  ret = EPD_TCON_LoadWaveform(g_waveformdata);
  if (ret != 1)
    {
      lcderr("Error in EPD_TCON_LoadWaveform: %d\n", ret);
      return NULL;
    }

  info = EPD_TCON_GetPanelInfo();
  if (info == NULL)
    {
      return NULL;
    }

#if defined(CONFIG_LCD_ET014TT1_LANDSCAPE) || defined(CONFIG_LCD_ET014TT1_RLANDSCAPE)
  priv->xres  = info->Height;
  priv->yres  = info->Width;
#else
  priv->xres  = info->Width;
  priv->yres  = info->Height;
#endif
  priv->width = info->Width;

  runbuffer = (uint8_t *)kmm_malloc(priv->xres);
  if (!runbuffer)
    {
      lcderr("Line buffer allocation failure.\n");
      return NULL;
    }

  dev->getvideoinfo = et014tt1_getvideoinfo;
  dev->getplaneinfo = et014tt1_getplaneinfo;
  dev->getpower     = et014tt1_getpower;
  dev->setpower     = et014tt1_setpower;
  dev->getcontrast  = et014tt1_getcontrast;
  dev->setcontrast  = et014tt1_setcontrast;
  priv->lcd = lcd;
  priv->runbuffer = runbuffer;

  lcdinfo("panel info - width: %d, height: %d\n", priv->xres, priv->yres);

  return dev;
}

/****************************************************************************
 * Name: et014tt1_update
 *
 * This function is non NuttX standard interface.
 *
 * ET014TT1 ePaper device is too slow for displaying. So we provide special
 * update interface to applications for faster updating and lower power
 * comsumption.
 * Thus, application must call this after frame buffer updated to show drawing
 * results.
 *
 ****************************************************************************/

void et014tt1_update(void)
{
  EPD_TCON_PowerOn();
  EPD_TCON_Update(ET014TT1_DISPLAYMODEGC);
  EPD_TCON_PowerOff();
}

/****************************************************************************
 * Name: et014tt1_clear
 *
 * This is a special and non NuttX standard interface.
 *
 ****************************************************************************/

void et014tt1_clear(void)
{
  EPD_TCON_PanelClear();
}

/****************************************************************************
 * Name: __aeabi_memset
 *
 * memset wrapper function for SWT library
 ****************************************************************************/

void * __aeabi_memset(void *block, int c, size_t size)
{
  return memset(block, c, size);
}

/****************************************************************************
 * Name: __aeabi_memclr
 *
 * memclr wrapper function for SWT library
 ****************************************************************************/

void * __aeabi_memclr(void *block, int c, size_t size)
{
  return memset(block, c, size);
}

/****************************************************************************
 * Name: __aeabi_memmove4
 *
 * memmove4 wrapper function for SWT library
 ****************************************************************************/

void * __aeabi_memmove4(void *to, const void *from, size_t size)
{
  return memmove(to, from, size);
}

