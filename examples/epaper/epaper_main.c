/****************************************************************************
 * epaper/epaper_main.c
 *
 *   Copyright (C) 2011, 2015-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Copyright (C) 2018 Sony Corporation
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
#include <sys/boardctl.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <sched.h>
#include <pthread.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include <nuttx/lcd/lcd.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>

#include <nuttx/lcd/et014tt1.h>

#include "epaper.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* If not specified, assume that the hardware supports one video plane */

#ifndef CONFIG_EXAMPLES_EPAPER_VPLANE
#  define CONFIG_EXAMPLES_EPAPER_VPLANE 0
#endif

/* If not specified, assume that the hardware supports one LCD device */

#ifndef CONFIG_EXAMPLES_EPAPER_DEVNO
#  define CONFIG_EXAMPLES_EPAPER_DEVNO 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct epaper_data_s g_epaper =
{
  NULL,          /* hnx */
  NULL,          /* hbkgd */
  NULL,          /* hfont */
  0,             /* xres */
  0,             /* yres */
  false,         /* havpos */
  { 0 },         /* sem */
  NXEXIT_SUCCESS /* exit code */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: epaper_initialize
 ****************************************************************************/

static inline int epaper_initialize(void)
{
  FAR NX_DRIVERTYPE *dev;
  int ret;

  /* Initialize the LCD device */

  printf("epaper_initialize: Initializing LCD\n");
  ret = board_lcd_initialize();
  if (ret < 0)
    {
      printf("epaper_initialize: board_lcd_initialize failed: %d\n", -ret);
      g_epaper.code = NXEXIT_LCDINITIALIZE;
      return ERROR;
    }

  /* Get the device instance */

  dev = board_lcd_getdev(CONFIG_EXAMPLES_EPAPER_DEVNO);
  if (!dev)
    {
      printf("epaper_initialize: board_lcd_getdev failed, devno=%d\n",
             CONFIG_EXAMPLES_EPAPER_DEVNO);
      g_epaper.code = NXEXIT_LCDGETDEV;
      return ERROR;
    }

  /* Turn the LCD on at 75% power */

  (void)dev->setpower(dev, ((3*CONFIG_LCD_MAXPOWER + 3)/4));

  /* Then open NX */

  printf("epaper_initialize: Open NX\n");
  g_epaper.hnx = nx_open(dev);
  if (!g_epaper.hnx)
    {
      printf("epaper_initialize: nx_open failed: %d\n", errno);
      g_epaper.code = NXEXIT_NXOPEN;
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: epaper_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int epaper_main(int argc, char *argv[])
#endif
{
  nxgl_mxpixel_t color;
  int ret;

  /* Initialize NX */

  ret = epaper_initialize();
  printf("epaper_main: NX handle=%p\n", g_epaper.hnx);
  if (!g_epaper.hnx || ret < 0)
    {
      printf("epaper_main: Failed to get NX handle: %d\n", errno);
      g_epaper.code = NXEXIT_NXOPEN;
      goto errout;
    }

  /* Get the default font handle */

  g_epaper.hfont = nxf_getfonthandle(CONFIG_EXAMPLES_EPAPER_FONTID);
  if (!g_epaper.hfont)
    {
      printf("epaper_main: Failed to get font handle: %d\n", errno);
      g_epaper.code = NXEXIT_FONTOPEN;
      goto errout;
    }

  /* Set the background to the configured background color */

  printf("epaper_main: Set background color=%d\n",
         CONFIG_EXAMPLES_EPAPER_BGCOLOR);

  color = CONFIG_EXAMPLES_EPAPER_BGCOLOR;
  ret = nx_setbgcolor(g_epaper.hnx, &color);
  if (ret < 0)
    {
      printf("epaper_main: nx_setbgcolor failed: %d\n", errno);
      g_epaper.code = NXEXIT_NXSETBGCOLOR;
      goto errout_with_nx;
    }

  /* Get the background window */

  ret = nx_requestbkgd(g_epaper.hnx, &g_epapercb, NULL);
  if (ret < 0)
    {
      printf("epaper_main: nx_setbgcolor failed: %d\n", errno);
      g_epaper.code = NXEXIT_NXREQUESTBKGD;
      goto errout_with_nx;
    }

  /* Wait until we have the screen resolution.  We'll have this immediately
   * unless we are dealing with the NX server.
   */

  while (!g_epaper.havepos)
    {
      (void)sem_wait(&g_epaper.sem);
    }
  printf("epaper_main: Screen resolution (%d,%d)\n", g_epaper.xres, g_epaper.yres);

  /* Now, say hello and update to show on an ePaper device */

  epaper_hello(g_epaper.hbkgd);

  /* Application must be call et014tt1_update() special API to display */

  et014tt1_update();

  /* Release background */

  (void)nx_releasebkgd(g_epaper.hbkgd);

  /* Close NX */

errout_with_nx:
  printf("epaper_main: Close NX\n");
  nx_close(g_epaper.hnx);
errout:
  return g_epaper.code;
}
