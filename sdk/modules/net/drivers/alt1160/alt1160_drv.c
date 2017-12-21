/****************************************************************************
 * cxd56net/driver/alt1160/alt1160_drv.c
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Yutaka Miyajima <Yutaka.Miyajima@sony.com>
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

#include <sys/ioctl.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/modem/alt1160.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_MODEM_ALT_1160)
#  error "CONFIG_MODEM_ALT_1160 is not defined in the configuration"
#endif

#define ALT_1160_DEVPATH      "/dev/alt1160"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * alt1160_open
 ****************************************************************************/

int alt1160_open(void)
{
  int ret;
  int fd;

  fd = open(ALT_1160_DEVPATH, O_RDWR);
  if (fd < 0)
    {
      _err("Device %s open failure. %d\n", ALT_1160_DEVPATH, fd);
      return fd;
    }

  ret = ioctl(fd, MODEM_IOC_POWERON, 0);
  if (ret != OK)
    {
      _err("modem power on failure. %d\n", ret);
      close(fd);
      fd = -1;
      return fd;
    }

  return fd;
}

/****************************************************************************
 * alt1160_close
 ****************************************************************************/

int alt1160_close(int fd)
{
  int ret;

  ret = ioctl(fd, MODEM_IOC_POWEROFF, 0);
  if (ret != OK)
    {
      _err("modem power off failure. %d\n", ret);
    }
  else
    {
      ret = close(fd);
    }

  return ret;
}

/****************************************************************************
 * alt1160_write
 ****************************************************************************/

ssize_t alt1160_write(int fd, FAR const void *buf, size_t len)
{
  ssize_t size = ERROR;
  int lfd;

  /* Open device file
   * Because the called task may not be open
   */

  lfd = open(ALT_1160_DEVPATH, O_RDWR);
  if (lfd < 0)
    {
      _err("Device %s open failure. %d\n", ALT_1160_DEVPATH, lfd);
    }
  else
    {
      size = write(lfd, buf, len);
      close(lfd);
    }

  return size;
}

/****************************************************************************
 * alt1160_read
 ****************************************************************************/

ssize_t alt1160_read(int fd, FAR void *buf, size_t len)
{
  ssize_t size = 0;

  size = read(fd, buf, len);

  return size;
}

/****************************************************************************
 * alt1160_read_abort
 ****************************************************************************/

int alt1160_read_abort(int fd)
{
  int ret;

  ret = ioctl(fd, MODEM_IOC_READABORT, 0);

  return ret;
}


