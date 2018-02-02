/****************************************************************************
 * examples/camera/camera_main.c
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

#include <sys/ioctl.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <semaphore.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
#include <nuttx/lcd/lcd.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#endif
#include <nuttx/video/isx012.h>

#include <arch/chip/cisif.h>

#include "nximage.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_CAMERA_LCD_DEVNO
#  define CONFIG_EXAMPLES_CAMERA_LCD_DEVNO 0
#endif

/* Convert 32bit integer to unsigned 8bit with saturated */

#define itou8(v) ((v) < 0 ? 0 : ((v) > 255 ? 255 : (v)))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct uyvy_s
{
  uint8_t u0;
  uint8_t y0;
  uint8_t v0;
  uint8_t y1;
};

struct capture_info_s
{
  uint8_t code;
  uint8_t last_frame;
  uint32_t size;
  uint32_t addr;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
struct nximage_data_s g_nximage =
{
  NULL,          /* hnx */
  NULL,          /* hbkgd */
  0,             /* xres */
  0,             /* yres */
  false,         /* havpos */
  { 0 },         /* sem */
  0              /* exit code */
};
#endif

static sem_t g_capture;

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD

static inline int nximage_initialize(void)
{
  FAR NX_DRIVERTYPE *dev;
  nxgl_mxpixel_t color;
  int ret;

  /* Initialize the LCD device */

  printf("nximage_initialize: Initializing LCD\n");
  ret = board_lcd_initialize();
  if (ret < 0)
    {
      printf("nximage_initialize: board_lcd_initialize failed: %d\n", -ret);
      return ERROR;
    }

  /* Get the device instance */

  dev = board_lcd_getdev(CONFIG_EXAMPLES_CAMERA_LCD_DEVNO);
  if (!dev)
    {
      printf("nximage_initialize: board_lcd_getdev failed, devno=%d\n",
             CONFIG_EXAMPLES_CAMERA_LCD_DEVNO);
      return ERROR;
    }

  /* Turn the LCD on at 75% power */

  (void)dev->setpower(dev, ((3*CONFIG_LCD_MAXPOWER + 3)/4));

  /* Then open NX */

  printf("nximage_initialize: Open NX\n");
  g_nximage.hnx = nx_open(dev);
  if (!g_nximage.hnx)
    {
      printf("nximage_initialize: nx_open failed: %d\n", errno);
      return ERROR;
    }

  /* Set background color to black */

  color = 0;
  nx_setbgcolor(g_nximage.hnx, &color);
  ret = nx_requestbkgd(g_nximage.hnx, &g_nximagecb, NULL);
  if (ret < 0)
    {
      printf("nximage_initialize: nx_requestbkgd failed: %d\n", errno);
      nx_close(g_nximage.hnx);
      return ERROR;
    }

  while (!g_nximage.havepos)
    {
      (void) sem_wait(&g_nximage.sem);
    }
  printf("nximage_initialize: Screen resolution (%d,%d)\n", g_nximage.xres, g_nximage.yres);

  return 0;
}
#endif

volatile struct capture_info_s g_info;

static void complete_capture(uint8_t code, uint8_t last_frame,
                             uint32_t size, uint32_t addr)
{
  /* Save capture image information */

  g_info.code = code;
  g_info.last_frame = last_frame;
  g_info.size = size;
  g_info.addr = addr;

  sem_post(&g_capture);
}

#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD

static inline void ycbcr2rgb(uint8_t y, uint8_t cb, uint8_t cr, uint8_t *r, uint8_t *g, uint8_t *b)
{
  int _r;
  int _g;
  int _b;
  _r = (128 * (y-16) +                  202 * (cr-128) + 64) / 128;
  _g = (128 * (y-16) -  24 * (cb-128) -  60 * (cr-128) + 64) / 128;
  _b = (128 * (y-16) + 238 * (cb-128)                  + 64) / 128;
  *r = itou8(_r);
  *g = itou8(_g);
  *b = itou8(_b);
}

static inline uint16_t ycbcrtorgb565(uint8_t y, uint8_t cb, uint8_t cr)
{
  uint8_t r;
  uint8_t g;
  uint8_t b;

  ycbcr2rgb(y, cb, cr, &r, &g, &b);
  r = (r >> 3) & 0x1f;
  g = (g >> 2) & 0x3f;
  b = (b >> 3) & 0x1f;
  return (uint16_t)(((uint16_t)r << 11) | ((uint16_t)g << 5) | (uint16_t)b);
}

/* Color conversion to show on display devices. */

static void yuv2rgb(void *buf, uint32_t size)
{
  struct uyvy_s *ptr;
  struct uyvy_s uyvy;
  uint16_t *dest;
  uint32_t i;

  ptr = buf;
  dest = buf;
  for (i = 0; i < size / 4; i++)
    {
      /* Save packed YCbCr elements due to it will be replaced with
       * converted color data.
       */

      uyvy = *ptr++;

      /* Convert color format to packed RGB565 */

      *dest++ = ycbcrtorgb565(uyvy.y0, uyvy.u0, uyvy.v0);
      *dest++ = ycbcrtorgb565(uyvy.y1, uyvy.u0, uyvy.v0);
    }
}

#endif

/****************************************************************************
 * camera_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int camera_main(int argc, FAR char *argv[])
#else
int camera_main(int argc, char *argv[])
#endif
{
  cisif_param_t  cis_param;
  cisif_sarea_t  cis_area;
  void *capturebuffer;
  uint32_t capturesize;
  uint32_t hsize;
  uint32_t vsize;
  int fd;
  int ret;

#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
  ret = nximage_initialize();
  if (ret < 0)
    {
      printf("camera_main: Failed to get NX handle: %d\n", errno);
      return 1;
    }
#endif

  hsize = 320;
  vsize = 240;
  capturesize = hsize * vsize * 2;

  /* Allocate image buffer for image sensor. This buffer must be 32 byte
   * aligned because it will be accessed directly from hardware.
   */

  capturebuffer = memalign(32, capturesize);

  fd = open("/dev/imager0", 0);
  if (fd < 0)
    {
      printf("failed to open imager driver : %d:%d\n", fd, errno);
      return 0;
    }
  ret = ioctl(fd, IMGIOC_SETSTATE, STATE_ISX012_ACTIVE);
  if (ret < 0)
    {
      printf("IMGIOC_SETSTATE failed. %d\n", ret);
      return ret;
    }

  sem_init(&g_capture, 1, 0);

  /* TODO: Need description for this configuration */

  cis_param.format                = FORMAT_CISIF_YUV;
  cis_param.yuv_param.hsize       = hsize;
  cis_param.yuv_param.vsize       = vsize;
  cis_param.yuv_param.notify_size = 0;
  cis_param.yuv_param.notify_func = NULL;
  cis_param.yuv_param.comp_func   = complete_capture;
  cis_param.jpg_param.notify_size = 0;
  cis_param.jpg_param.notify_func = NULL;
  cis_param.jpg_param.comp_func   = NULL;
  ret = cxd56_cisifinit(&cis_param);
  if (ret != OK)
    {
      printf("camera_main: cxd56_cisifinit failed: %d\n", ret);
      goto finish;
    }

  cis_area.strg_addr = capturebuffer;
  cis_area.strg_size = capturesize;

#ifdef CONFIG_EXAMPLES_CAMERA_INFINITE
  for (;;)
#endif
    {
      ret = cxd56_cisifcaptureframe(&cis_area, NULL);
      if (ret != OK)
        {
          printf("camera_main: cxd56_cisifcaptureframe failed: %d\n", ret);
          goto finish;
        }

      /* Waiting for capture done */

      sem_wait(&g_capture);

      printf("frame: %d at %08x (size %08x) (code=%d)\n", g_info.last_frame,
             g_info.addr, g_info.size, g_info.code);

#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
      /* Convert YUV color format to RGB565 */

      yuv2rgb(capturebuffer, capturesize);

      nximage_image(g_nximage.hbkgd, capturebuffer);
#else
      /* TODO: Save capture image to file or any other output destination */
#endif
    }

#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
  /* Stop until key input for prevent clear display by nx_releasebkgd() */

  (void) getchar();
#endif

 finish:
#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
  nx_releasebkgd(g_nximage.hbkgd);
  nx_close(g_nximage.hnx);
#endif

  ret = close(fd);
  if (ret < 0)
    {
      printf("failed to close imager driver : %d\n", ret);
    }

  free(capturebuffer);

  return 0;
}
