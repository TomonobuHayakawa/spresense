/****************************************************************************
 * examples/proximity/proximity_main_lt1pa01_int.c
 *
 *   Copyright (C) 2016 Sony Corporation
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
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/sensors/lt1pa01.h>

#ifdef CONFIG_CXD56_SCU
#include <arch/chip/cxd56_scu.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_PROXIMITY_DEVNAME
#  define CONFIG_EXAMPLES_PROXIMITY_DEVNAME "/dev/proximity0"
#endif

#ifndef CONFIG_EXAMPLES_PROXIMITY_SIGNO
#  define CONFIG_EXAMPLES_PROXIMITY_SIGNO 13
#endif

#define MY_TIMER_SIGNAL 17

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_sensing_data(int fd)
{
  int ret;
  uint8_t ps;
  uint8_t status = 0;
  static uint8_t prev = 0;

  ret = read(fd, &ps, 1);
  if (ret < 0)
    {
      printf("Read error %d\n", errno);
    }

  ret = ioctl(fd, SNIOC_GETINTSTATUS, (unsigned long)((uintptr_t)&status));
  if (ret < 0)
    {
      printf("GETINTSTATUS failed. %d\n", errno);
    }
  else
    {
      if (prev != status)
        {
          if (status & 0x80)
            {
              printf("Near\n");
            }
          else
            {
              printf("Far\n");
            }
          prev = status;
        }
    }
}

static int sensing_main(int fd)
{
  sigset_t           set;
  struct sigevent    sev;
  struct itimerspec  timer;
  timer_t timerid;
  int ret;

  sev.sigev_notify            = SIGEV_SIGNAL;
  sev.sigev_signo             = MY_TIMER_SIGNAL;
  ret = timer_create(CLOCK_REALTIME, &sev, &timerid);
  if (ret != OK)
    {
      printf("timer_create failed. %d\n", ret);
      return -1;
    }

  timer.it_value.tv_sec     = 0;
  timer.it_value.tv_nsec    = 250 * 1000 * 1000;
  timer.it_interval.tv_sec  = timer.it_value.tv_sec;
  timer.it_interval.tv_nsec = timer.it_value.tv_nsec;
  timer_settime(timerid, 0, &timer, NULL);

  sigemptyset(&set);
  sigaddset(&set, MY_TIMER_SIGNAL);

  for (;;)
    {
      ret = sigwaitinfo(&set, NULL);
      if (ret < 0)
        {
          int errcode = errno;
          if (errcode == EINTR)
            {
              continue;
            }
          fprintf(stderr, "sigwaitinfo() failed. %d\n", errcode);
          return -1;
        }

      show_sensing_data(fd);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * sensor_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int proximity_main(int argc, char *argv[])
#endif
{
  int fd;
  int ret;

  printf("Sensing start...\n");

  fd = open(CONFIG_EXAMPLES_PROXIMITY_DEVNAME, O_RDONLY);
  if (fd < 0)
    {
      printf("Device %s open failure. %d\n",
             CONFIG_EXAMPLES_PROXIMITY_DEVNAME, fd);
      return -1;
    }

  ret = ioctl(fd, SNIOC_SETPROXLTHRESHOLD, 0xe0);
  if (ret < 0)
    {
      printf("SETPROXLTHRESHOLD failed. %d\n", ret);
      return -1;
    }

  ret = ioctl(fd, SNIOC_SETPROXHTHRESHOLD, 0xf0);
  if (ret < 0)
    {
      printf("SETPROXHTHRESHOLD failed. %d\n", ret);
      return -1;
    }

  ret = ioctl(fd, SNIOC_STARTPROXMEASUREMENT, 0);
  if (ret < 0)
    {
      printf("STARTPROXMEASUREMENT failed. %d\n", ret);
      return -1;
    }

  sensing_main(fd);

  ret = ioctl(fd, SNIOC_STOPPROXMEASUREMENT, 0);
  if (ret < 0)
    {
      printf("STOPPROXMEASUREMENT failed. %d\n", ret);
      return -1;
    }

  close(fd);

  return 0;
}
