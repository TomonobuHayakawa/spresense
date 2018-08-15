/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_lpm013m091a_fb.c
 *
 * Driver for LPM013M091A LCD as a character device using frame buffer.
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

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/lpm013m091a.h>

#include "cxd56_lpm013m091a_base.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* frame buffer info */

int lpm013m091a_getvideoinfo(FAR struct fb_vtable_s *vtable,
                    FAR struct fb_videoinfo_s *vinfo);
int lpm013m091a_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                    FAR struct fb_planeinfo_s *pinfo);

/* Character driver methods */

static int     lpm013m091a_open(FAR struct file *filep);
static int     lpm013m091a_close(FAR struct file *filep);
static ssize_t lpm013m091a_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int     lpm013m091a_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char *g_fb;
static struct fb_vtable_s g_dev;

static const struct file_operations g_lpm013m091afops =
{
  lpm013m091a_open,  /* open */
  lpm013m091a_close, /* close */
  0,                 /* read */
  lpm013m091a_write, /* write */
  0,                 /* seek */
  lpm013m091a_ioctl, /* ioctl */
};

static struct lpm013m091a_base_s *g_lpm013m091a_base;

/****************************************************************************
 * Name: getvideoinfo_watch
 ****************************************************************************/

int lpm013m091a_getvideoinfo(FAR struct fb_vtable_s *vtable,
                    FAR struct fb_videoinfo_s *vinfo)
{
  vinfo->fmt = LPM013M091A_COLORFMT; /* see FB_FMT_*  */
  vinfo->xres = LPM013M091A_XRES;    /* Horizontal resolution in pixel columns */
  vinfo->yres = LPM013M091A_YRES;    /* Vertical resolution in pixel rows */
  vinfo->nplanes = 1;                /* Number of color planes supported */
  return 0;
}

/****************************************************************************
 * Name: getplaneinfo_watch
 ****************************************************************************/

int lpm013m091a_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                    FAR struct fb_planeinfo_s *pinfo)
{
  pinfo->fbmem = g_fb;                 /* Start of frame buffer memory */
  pinfo->fblen = LPM013M091A_FBSIZE;   /* Length of frame buffer memory in bytes */
  pinfo->stride = LPM013M091A_XSTRIDE; /* Length of a line in bytes */
  pinfo->display = 0;                  /* Display number */
  pinfo->bpp = LPM013M091A_BPP;        /* Bits per pixel */
  return 0;
}

/****************************************************************************
 * Name: lpm013m091a_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int lpm013m091a_open(FAR struct file *filep)
{
  g_fb = kmm_malloc(LPM013M091A_FBSIZE);

  if (!g_fb)
    {
      dbg("Failed to malloc frame buffer.\n");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: lpm013m091a_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int lpm013m091a_close(FAR struct file *filep)
{
  if (g_fb)
    {
      kmm_free(g_fb);
    }

  return OK;
}

/****************************************************************************
 * Name: lpm013m091a_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t lpm013m091a_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  ResCode res;

  if (buflen != LPM013M091A_FBSIZE)
    {
      dbg("Expected buffer size is %d\n", LPM013M091A_FBSIZE);
      return -1;
    }

  res = lpm013m091a_drawBitmap(g_lpm013m091a_base->lcd, (FAR const uint16_t *)buffer, 0, 0, LPM013M091A_XRES, LPM013M091A_YRES);
  if (res != ResCodeOk)
    {
      dbg("Failed to write buffer. rescode: %d\n", res);
      return -1;
    }

  return buflen;
}

/****************************************************************************
 * Name: lpm013m091a_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

int lpm013m091a_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      /* Turn ON/OFF backlight. Arg: bool */

      case LCDIOC_BACKLIGHT:
        {
          ResCode res = lpm013m091a_turnBacklight(g_lpm013m091a_base->lcd, arg != 0);
          if (res != ResCodeOk)
            {
              dbg("Failed to ioctl. rescode: %d\n", res);
              ret = -EIO;
            }

          break;
        }

      default:
        dbg("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Initialize LCD
 ****************************************************************************/

FAR struct fb_vtable_s *lpm013m091a_fb_initialize(FAR struct lpm013m091a_lcd_s *lcd, struct spi_dev_s *spi)
{
  g_lpm013m091a_base = lpm013m091a_base_initialize(lcd, spi);

  g_dev.getvideoinfo = lpm013m091a_getvideoinfo;
  g_dev.getplaneinfo = lpm013m091a_getplaneinfo;

  return &g_dev;
}

/****************************************************************************
 * Name: lpm013m091a_register
 *
 * Description:
 *   Register the lpm013m091a character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/lcd0"
 *   dev     - An instance of the SPI interface to use to communicate with
 *             lpm013m091a
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lpm013m091a_register(FAR const char *devpath, FAR struct lpm013m091a_lcd_s *lcd)
{
  FAR struct lpm013m091a_lcd_s *priv;
  int ret;

  priv = lcd;

  ret = lpm013m091a_init(priv);
  if (ret < 0)
    {
      dbg("Failed to initialize lpm013m091a.\n");
      return ret;
    }

  ret = register_driver(devpath, &g_lpm013m091afops, 0666, priv);
  if (ret < 0)
    {
      dbg("Failed to register driver: %d\n", ret);
    }

  return OK;
}
