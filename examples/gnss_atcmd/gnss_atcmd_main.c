/****************************************************************************
 * examples/gnss_atcmd/gnss_atcmd_main.c
 *
 *   Copyright (C) 2017 Sony. All rights reserved.
 *   Author: Tomoyuki Takahashi <Tomoyuki.A.Takahashi@sony.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
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
#include <sdk/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <sched.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <arch/chip/gnss.h>
#include "gpsutils/cxd56_gnss_nmea.h"
#include "gnss_usbserial.h"
#include "gnss_atcmd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GNSS_POLL_FD_NUM          2
#define GNSS_POLL_TIMEOUT_FOREVER -1
#define MY_GNSS_SIG0              18
#define MY_GNSS_SIG1              19
#define USB_RBUF_SIZE             128
#define READ_FD                   usbfds[GNSS_ATCMD_READ_FD]
#define WRITE_FD                  usbfds[GNSS_ATCMD_WRITE_FD]

#define _USE_STATIC_NMEA_BUF

#ifdef _DEBUG
#define dbg_printf                printf
#else
#define dbg_printf(FMT, ...)
#endif

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM
#ifndef CONFIG_EXAMPLES_GNSS_ATCMD_SUB_STACKSIZE
#define CONFIG_EXAMPLES_GNSS_ATCMD_SUB_STACKSIZE  512
#endif
#ifndef CONFIG_EXAMPLES_GNSS_ATCMD_SUB_PRIORITY
#define CONFIG_EXAMPLES_GNSS_ATCMD_SUB_PRIORITY   CONFIG_EXAMPLES_GNSS_ATCMD_PRIORITY
#endif
#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM */

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  int8_t   sing;
  uint8_t  degree;
  uint8_t  minute;
  uint32_t frac;
} ST_DMS;

/****************************************************************************
 * Private Data
 ****************************************************************************/
static struct gnss_atcmd_info atcmd_info;
static pthread_t              atcmd_tid;
static struct cxd56_gnss_positiondata_s  posdat;
static int                    usbfds[2];
static char                   usb_rbuf[USB_RBUF_SIZE];
#ifdef _USE_STATIC_NMEA_BUF
static char                   nmea_buf[NMEA_SENTENCE_MAX_LEN];
#endif
#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM
static sem_t                  syncsem;
static pthread_t              atcmd_sub_tid;
static NMEA_SPECTRUM_DATA     spectrumdat;
#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM */

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int print_nmea(int fd)
{
  int    ret;

  ret = lseek(fd, CXD56_GNSS_READ_OFFSET_LAST_GNSS, SEEK_SET);
  if (ret < 0)
    {
      ret = errno;
      printf("lseek error %d\n", ret);
      goto _err1;
    }

  ret = read(fd, &posdat, sizeof(posdat));
  if (ret < 0)
    {
      ret = errno;
      printf("read error %d\n", ret);
      goto _err1;
    }

   NMEA_Output(&posdat);

_err1:
  return ret;
}

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM

static int print_spectrum(int fd)
{
  int    ret;

  do
    {
      ret = lseek(fd, CXD56_GNSS_READ_OFFSET_SPECTRUM, SEEK_SET);
      if (ret < 0)
        {
          ret = errno;
          printf("lseek error %d\n", ret);
          goto _err1;
        }

      ret = read(fd, &spectrumdat, sizeof(spectrumdat));
      if (ret < 0)
        {
          ret = errno;
          printf("read error %d\n", ret);
          goto _err1;
        }
      else if (ret == 0)
        {
          goto _err1;
        }

      NMEA_OutputSpectrum(&spectrumdat);
    }
  while (1);

_err1:
  return ret;
}

#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM */

/* output NMEA */

FAR static char *reqbuf(uint16_t size)
{
#ifdef _USE_STATIC_NMEA_BUF
  if (size > sizeof(nmea_buf))
    {
      printf("reqbuf error: oversize %s\n", size);
      return NULL;
    }
  return nmea_buf;
#else
  return malloc(size);
#endif
}

static void freebuf(FAR char *buf)
{
#ifdef _USE_STATIC_NMEA_BUF
#else
  free(buf);
#endif
}

static int outnmea(FAR char *buf)
{
  return gnss_atcmd_printf(WRITE_FD, "%s", buf);
}

static int outbin(FAR char *buf, uint32_t len)
{
  return write(WRITE_FD, buf, (size_t)len);
}

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM

static int set_signal(int fd, int signo, uint8_t gnsssig, bool enable)
{
  struct cxd56_gnss_signal_setting_s setting;

  setting.fd      = fd;
  setting.enable  = enable;
  setting.gnsssig = gnsssig;
  setting.signo   = signo;
  setting.data    = NULL;

  return ioctl(fd, CXD56_GNSS_IOCTL_SIGNAL_SET, (unsigned long)&setting);
}

static int signal2own(int signo, void *data)
{
  int ret;

#ifdef CONFIG_CAN_PASS_STRUCTS
  union sigval value;
  value.sival_ptr = data;
  ret = sigqueue(getpid(), signo, value);
#else
  ret = sigqueue(getpid(), signo, data);
#endif

  return ret;
}

#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM */

static FAR void atcmd_emulator(FAR void *arg)
{
  int                    fd;
  int                    ret;
  int                    remain;
  struct pollfd          fds[GNSS_POLL_FD_NUM];
  char *                 bufhead;
  NMEA_OUTPUT_CB         funcs;

  /* program start */

  printf("Start GNSS_ATCMD!!\n");

  fd = open("/dev/gps", O_RDONLY);
  if (fd < 0)
    {
      printf("open error:%d,%d\n", fd, errno);
      ret = -ENODEV;
      goto _err0;
    }

  /* Init NMEA library */

  NMEA_InitMask();
  funcs.bufReq  = reqbuf;
  funcs.out     = outnmea;
  funcs.outBin  = outbin;
  funcs.bufFree = freebuf;
  NMEA_RegistOutputFunc(&funcs);

  ret = gnss_usbserial_open(usbfds);
  if (ret < 0)
    {
      printf("usb open error. %d\n", ret);
      goto _err1;
    }

  atcmd_info.gnssfd = fd;
  atcmd_info.wfd    = WRITE_FD;
  atcmd_info.rfd    = READ_FD;

  bufhead       = usb_rbuf;
  remain        = sizeof(usb_rbuf);

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM
  sem_post(&syncsem);
#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM */

  do
    {
      char *p;
      int   sz;

      memset(fds, 0, sizeof(fds));
      fds[0].fd     = READ_FD;
      fds[0].events = POLLIN;
      fds[1].fd     = fd;
      fds[1].events = POLLIN;

      ret = poll(fds, GNSS_POLL_FD_NUM, GNSS_POLL_TIMEOUT_FOREVER);
      if (ret <= 0 && errno != EINTR)
        {
          printf("poll error %d,%d,%x,%x\n", ret, errno, fds[0].events,
                  fds[0].revents);
          break;
        }

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM
      ret = sem_wait(&syncsem);
      if (ret < 0)
        {
          printf("unexpected wait syncsem error%d\n", ret);
          goto _err1;
        }
#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM */

      if (fds[0].revents & POLLIN)
        {
          sz = read(READ_FD, bufhead, remain);
          if (sz < 0)
            {
              continue;
            }
          remain -= sz;
          bufhead += sz;
          if (remain > 0)
            {
              *bufhead = '\0';
              p = strchr(usb_rbuf, '\r');
              if (p == NULL)
                {
                  p = strchr(usb_rbuf, '\n');
                  if (p == NULL)
                    {
                      goto _continue;
                    }
                }
              sz = p - usb_rbuf + 1;
            }
          else
            {
              sz = sizeof(usb_rbuf);
              p  = &usb_rbuf[sz - 1];
            }
          *p = '\0';
          remain  = sizeof(usb_rbuf);
          bufhead = usb_rbuf;
          dbg_printf("# %*s\n", sz, usb_rbuf);
#ifdef _DEBUG
          write(WRITE_FD, usb_rbuf, sz);
#endif
          ret = gnss_atcmd_exec(&atcmd_info, usb_rbuf, sz);
        }

_continue:
      if (fds[1].revents & POLLIN)
        {
          print_nmea(fd);
        }

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM
      sem_post(&syncsem);
#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM */
    }
  while (ret != -ESHUTDOWN);

  gnss_usbserial_close(usbfds);

_err1:
  ret = close(fd);
  if (ret < 0)
    {
      printf("device close error\n");
    }

  /* The pthread has returned */

_err0:
#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM
  signal2own(MY_GNSS_SIG1, NULL);
  sem_destroy(&syncsem);
#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM */

  printf("Stop GNSS_ATCMD!!\n");

  pthread_exit(0);
}

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM

static FAR void spectrum_handler(FAR void *arg)
{
  int      ret;
  int      signo;
  sigset_t mask;

  ret = sem_wait(&syncsem);
  if (ret < 0)
    {
      printf("unexpected wait syncsem error%d\n", ret);
      goto _err;
    }

  /* Init signal */

  ret =
    set_signal(atcmd_info.gnssfd, MY_GNSS_SIG0, CXD56_GNSS_SIG_SPECTRUM, 1);
  if (ret < 0)
    {
      printf("GNSS spectrum signal set error\n");
      goto _err;
    }

  sigemptyset(&mask);
  sigaddset(&mask, MY_GNSS_SIG0);
  sigaddset(&mask, MY_GNSS_SIG1);

  sem_post(&syncsem);

  do
    {
      signo = sigwaitinfo(&mask, NULL);
      if (signo < 0)
        {
          continue;
        }
      else if (signo == MY_GNSS_SIG1)
        {
          goto _exit;
        }

      ret = sem_wait(&syncsem);
      if (ret < 0)
        {
          printf("unexpected wait syncsem error%d\n", ret);
          goto _exit;
        }

      if (signo == MY_GNSS_SIG0)
        {
          print_spectrum(atcmd_info.gnssfd);
        }

      sem_post(&syncsem);
    }
  while (1);

_exit:
  set_signal(atcmd_info.gnssfd, MY_GNSS_SIG0, CXD56_GNSS_SIG_SPECTRUM, 0);
_err:
  printf("GNSS exit spectrum handler\n");
  pthread_exit(0);
}

#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM */

/****************************************************************************
 * gnss_atcmd_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int gnss_atcmd_main(int argc, char *argv[])
#endif
{
  pthread_attr_t           tattr;
  struct sched_param       param;
  int                      ret;

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM

  ret = sem_init(&syncsem, 0, 0);
  if (ret < 0)
    {
      _err("Failed to initialize syncsem!\n");
      goto _err;
    }

#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM */

  pthread_attr_init(&tattr);
  tattr.stacksize      = CONFIG_EXAMPLES_GNSS_ATCMD_STACKSIZE;
  param.sched_priority = CONFIG_EXAMPLES_GNSS_ATCMD_PRIORITY;
  pthread_attr_setschedparam(&tattr, &param);

  ret = pthread_create(&atcmd_tid, &tattr, (pthread_startroutine_t)atcmd_emulator,
                       (pthread_addr_t)NULL);
  if (ret != 0)
    {
      ret = -ret; /* pthread_create does not modify errno. */
      goto _err;
    }

#ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM

  pthread_attr_init(&tattr);
  tattr.stacksize      = CONFIG_EXAMPLES_GNSS_ATCMD_SUB_STACKSIZE;
  param.sched_priority = CONFIG_EXAMPLES_GNSS_ATCMD_SUB_PRIORITY;
  pthread_attr_setschedparam(&tattr, &param);

  ret = pthread_create(&atcmd_sub_tid, &tattr, (pthread_startroutine_t)spectrum_handler,
                       (pthread_addr_t)NULL);
  if (ret != 0)
    {
      ret = -ret; /* pthread_create does not modify errno. */
    }

#endif /* ifdef CONFIG_EXAMPLES_GNSS_ATCMD_SUPPORT_SPECRUM */

_err:
  return ret;
}
