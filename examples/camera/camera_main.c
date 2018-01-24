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

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/video/isx012.h>

#include <arch/chip/cisif.h>
#include <arch/chip/pm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define COEF_Y   (128) /*            (int)(1.0000 * 128 + 0.5)*/
#define COEF_RCR (202) /*(1.5748) -> (int)(1.5748 * 128 + 0.5)*/
#define COEF_GCB ( 24) /*(0.4681) -> (int)(0.1873 * 128 + 0.5)*/
#define COEF_GCR ( 60) /*(0.1873) -> (int)(0.4681 * 128 + 0.5)*/
#define COEF_BCB (238) /*(1.8556) -> (int)(1.8556 * 128 + 0.5)*/

/* TODO: Remove LCD dependency */

#ifdef CONFIG_LCD_LPM013M091A
#define CAMERA_LCD_SEND_SIZE 0x2ee00 /* 320x300x2 */
#elif defined(CONFIG_LCD_ILI9340)
#define CAMERA_LCD_SEND_SIZE 0x25800 /* 320x240x2 */
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* TODO: freqlock is unnecessary because this is the outside of camera usage.
 * If you want to keep high voltage, then check CONFIG_CPUFREQ_RELEASE_LOCK
 * has been disabled.
 */

static struct pm_cpu_freqlock_s img_lock =
  PM_CPUFREQLOCK_INIT(PM_CPUFREQLOCK_TAG('C','M',0), PM_CPUFREQLOCK_FLAG_HV);

static volatile uint32_t rcv_frame = 1;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void camera_rcv_frame_data(uint8_t code, uint8_t last_frame, uint32_t size, uint32_t addr)
{
/* TODO: must be use semaphore to polling wait */

  rcv_frame = 0;
}

void camera_translate_yuv2rgb(uint8_t *buf, uint32_t hsize, uint32_t vsize)
{
  uint32_t yuv = 0;
  uint32_t y  = 0;
  uint32_t cb = 0;
  uint32_t cr = 0;
  int16_t rgb[3];
  uint32_t n = 0, i, j;

  uint16_t *p_src = (uint16_t *)buf;
  for (int iy = 0; iy < vsize ; iy++)
    {
      for (int ix = 0; ix < hsize; ix++) 
        {
          if (!(yuv & 1)) 
            {
              cb = (p_src[n]   & 0xff);
              cr = (p_src[n+1] & 0xff);
            }

          y = (p_src[n] & 0xff00) >> 8;
          yuv++;
          rgb[0] = (int)( (COEF_Y*(y-16) + COEF_RCR*(cr-128) + 64) / 128);
          rgb[1] = (int)( (COEF_Y*(y-16) - COEF_GCB*(cb-128) - 
                          COEF_GCR*(cr-128) + 64) / 128);
          rgb[2] = (int)( (COEF_Y*(y-16) + COEF_BCB*(cb-128) + 64) / 128);
          for (i = 0; i < 3;i++)
            {
              if (rgb[i] >= 255)
                {
                  rgb[i] = 255;
                }
              else if (rgb[i] <= 0)
                {
                  rgb[i] = 0;
                }

            }

          i =      (rgb[0] & 0xf8);       /* 5bit Red */
          i = i | ((rgb[1] & 0xe0) >> 5); /* 3bit Green */
          j =      (rgb[1] & 0x1c) << 3;  /* 3bit */
          j = j | ((rgb[2] & 0xf8) >> 3); /* 5bit Blue */
          p_src[n] = (i & 0xff) | ((j << 8) & 0xff00);
          n++;
        }
    }
}

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
  static uint8_t *img_buf;
  uint32_t hsize;
  uint32_t vsize;
  int fd1;
  int fd2;
  int ret;

  up_pm_acquire_freqlock(&img_lock);

  img_buf = memalign(32, CAMERA_LCD_SEND_SIZE); /* get buffer to imager and LCD */

  fd1 = open("/dev/imager0", O_CREAT); /* open device of imager */
  if (fd1 < 0)
    {
      printf("failed to open imager driver : %d:%d\n", fd1, errno);
    }
  else
    {
      ret = ioctl(fd1, IMGIOC_SETSTATE, STATE_ISX012_ACTIVE); /* ActiveISX012 */
      if (ret < 0)
        {
          printf("IMGIOC_SETSTATE failed. %d\n", ret);
          return ret;
        }
    }

  board_graphics_setup(0);

  /* TODO: Must be use NuttX LCD interface. */

  fd2 = open("/dev/lcd0", O_WRONLY);
  if (fd2 < 0)
    {
      printf("failed to open display driver : %d\n", fd2);
    }
  else
    {
      ioctl(fd2, 1, 1);
    }

  hsize = 320;
  vsize = 240;
  cis_param.format                = FORMAT_CISIF_YUV;
  cis_param.yuv_param.hsize       = 320;
  cis_param.yuv_param.vsize       = 240;
  cis_param.yuv_param.notify_size = 0;
  cis_param.yuv_param.notify_func = NULL;
  cis_param.yuv_param.comp_func   = camera_rcv_frame_data;
  cis_param.jpg_param.notify_size = 0;
  cis_param.jpg_param.notify_func = NULL;
  cis_param.jpg_param.comp_func   = NULL;
  ret = cxd56_cisifinit(&cis_param); /* initial CISIF */
  if (ret != OK)
    {
      printf("camera_main: cxd56_cisifinit failed: %d\n", ret);
    }

  cis_area.strg_addr = img_buf;
  cis_area.strg_size = 320*240*2;
  while(1)
    {
      ret = cxd56_cisifcaptureframe(&cis_area, NULL); /* get frame */
      if (ret == OK)
        {
          while(rcv_frame); /* wait frame */
          rcv_frame=1;
          if (fd2)
            {
              /* TODO: Remove this #if */
#if 1 /* translate from YUV to RGB565 */
              camera_translate_yuv2rgb(img_buf, hsize, vsize);
#else /* RGB565 */
              /* byte swap */
              uint16_t *p_src = (uint16_t *)img_buf;
              for (int i = 0; i < hsize*vsize; i++) 
                {
                  uint16_t tmp = *p_src;
                  *p_src++ = tmp << 8 | tmp >> 8;
                }
#endif
              ret = write(fd2, img_buf, CAMERA_LCD_SEND_SIZE); /* display to LCD */

              /* TODO: to more meaningful message */

              printf("DIS:%d\n", ret);
            }
        }
      else
        {
          printf("camera_main: cxd56_cisifcaptureframe failed: %d\n", ret);
        }
    }

  ret = close(fd1); /* close device of imager */
  if (ret < 0)
    {
      printf("failed to close imager driver : %d\n", ret);
    }

  ret = close(fd2); /* close device of LCD */
  if (ret < 0)
    {
      printf("failed to close display driver : %d\n", ret);
    }

  return 0;
}
