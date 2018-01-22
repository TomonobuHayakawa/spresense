/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_ili9340_fb.c
 *
 * Driver for ILI9340 LCD as a character device using frame buffer.
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

#include <sdk/config.h>

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
#include <nuttx/lcd/ili9340.h>

#include "cxd56_ili9340_base.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* frame buffer info */

int ili9340_getvideoinfo(FAR struct fb_vtable_s *vtable,
                    FAR struct fb_videoinfo_s *vinfo);
int ili9340_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                    FAR struct fb_planeinfo_s *pinfo);

/* Character driver methods */

static int     ili9340_open(FAR struct file *filep);
static int     ili9340_close(FAR struct file *filep);
static ssize_t ili9340_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int     ili9340_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char *g_fb;
static struct fb_vtable_s g_dev;

static const struct file_operations g_ili9340fops =
{
  ili9340_open,  /* open */
  ili9340_close, /* close */
  0,                 /* read */
  ili9340_write, /* write */
  0,                 /* seek */
  ili9340_ioctl, /* ioctl */
};

static struct ili9340_base_s *g_ili9340_base;

/****************************************************************************
 * Name: getvideoinfo_watch
 ****************************************************************************/

int ili9340_getvideoinfo(FAR struct fb_vtable_s *vtable,
                    FAR struct fb_videoinfo_s *vinfo)
{
  vinfo->fmt = ILI9340_COLORFMT; /* see FB_FMT_*  */
  vinfo->xres = ILI9340_XRES;    /* Horizontal resolution in pixel columns */
  vinfo->yres = ILI9340_YRES;    /* Vertical resolution in pixel rows */
  vinfo->nplanes = 1;                /* Number of color planes supported */
  return 0;
}

/****************************************************************************
 * Name: getplaneinfo_watch
 ****************************************************************************/

int ili9340_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                    FAR struct fb_planeinfo_s *pinfo)
{
  pinfo->fbmem = g_fb;                 /* Start of frame buffer memory */
  pinfo->fblen = ILI9340_FBSIZE;   /* Length of frame buffer memory in bytes */
  pinfo->stride = ILI9340_XSTRIDE; /* Length of a line in bytes */
  pinfo->display = 0;                  /* Display number */
  pinfo->bpp = ILI9340_BPP;        /* Bits per pixel */
  return 0;
}

/****************************************************************************
 * Name: ili9340_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int ili9340_open(FAR struct file *filep)
{
  g_fb = kmm_malloc(ILI9340_FBSIZE);

  if (!g_fb)
    {
      lcderr("Failed to malloc frame buffer.\n");
      return -ENOMEM;
    }

  return OK;
}

/****************************************************************************
 * Name: ili9340_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int ili9340_close(FAR struct file *filep)
{
  if (g_fb)
    {
      kmm_free(g_fb);
    }

  return OK;
}

/****************************************************************************
 * Name: ili9340_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t ili9340_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  int res;

  if (buflen != ILI9340_FBSIZE)
    {
      lcderr("Expected buffer size is %d\n", ILI9340_FBSIZE);
      return -EPERM;
    }

  res = ili9340_draw_bitmap(g_ili9340_base->lcd, (FAR const uint16_t *)buffer, 0, 0, ILI9340_XRES, ILI9340_YRES);
  if (res != OK)
    {
      lcderr("Failed to write buffer. rescode: %d\n", res);
      return -ENOBUFS;
    }

  return buflen;
}

/****************************************************************************
 * Name: ili9340_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

int ili9340_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      /* Turn ON/OFF backlight. Arg: bool */

      case LCDIOC_BACKLIGHT:
        {
          int res = ili9340_turn_backlight(g_ili9340_base->lcd, arg != 0);
          if (res != OK)
            {
              lcderr("Failed to ioctl. rescode: %d\n", res);
              ret = -EIO;
            }

          break;
        }

      default:
        lcderr("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Initialize LCD
 ****************************************************************************/

FAR struct fb_vtable_s *ili9340_fb_initialize(FAR struct ili9340_lcd_s *lcd, struct spi_dev_s *spi)
{
  g_ili9340_base = ili9340_base_initialize(lcd, spi);

  g_dev.getvideoinfo = ili9340_getvideoinfo;
  g_dev.getplaneinfo = ili9340_getplaneinfo;

  return &g_dev;
}

/****************************************************************************
 * Name: ili9340_register
 *
 * Description:
 *   Register the ili9340 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/lcd0"
 *   dev     - An instance of the SPI interface to use to communicate with
 *             ili9340
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ili9340_register(FAR const char *devpath, FAR struct ili9340_lcd_s *lcd)
{
  FAR struct ili9340_lcd_s *priv;
  int ret;

  priv = lcd;

  ret = ili9340_init(priv);
  if (ret < 0)
    {
      lcderr("Failed to initialize ili9340.\n");
      return ret;
    }

  ret = register_driver(devpath, &g_ili9340fops, 0666, priv);
  if (ret < 0)
    {
      lcderr("Failed to register driver: %d\n", ret);
    }

  return ret;
}
