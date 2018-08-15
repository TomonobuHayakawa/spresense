/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_et014tt1.c
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Kei Yamamoto <Kei.x.Yamamoto@sony.com>
 *           Tomonobu Hayakawa <Tomonobu.Hayakawa@sony.com>
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

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <assert.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/video/fb.h>
#include <nuttx/spi/spi.h>
#include <clock/clock.h>
#include <nuttx/timers/rtc.h>
#include <nuttx/time.h>
#include <nuttx/lcd/et014tt1.h>

#include <stdbool.h>
#include <errno.h>

#include <semaphore.h>
#include "gpio.h"
#include "et014tt1.h"

#ifndef ERROR
#  define ERROR -1
#endif
#ifndef OK
#  define OK 0
#endif

extern unsigned char g_WaveformData[];
extern const EPD_TCON_DRIVER_HAL g_TCON_HAL;

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/****************************************************************************
 * frame buffer info
 ****************************************************************************/

static int et014tt1_getvideoinfo(FAR struct fb_vtable_s *vtable,
                    FAR struct fb_videoinfo_s *vinfo);
static int et014tt1_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                    FAR struct fb_planeinfo_s *pinfo);

/****************************************************************************
 * Character driver methods
 ****************************************************************************/

static int     et014tt1_open(FAR struct file *filep);
static int     et014tt1_close(FAR struct file *filep);
static ssize_t et014tt1_write(FAR struct file *filep,
                    FAR const char *buffer, size_t buflen);
static int     et014tt1_ioctl(FAR struct file *filep,
                    int cmd, unsigned long arg);

/****************************************************************************
 * EInk library call backs
 ****************************************************************************/

static void _Spi_SetClock(SPI_SPEED_TYPE speed);
static void _Spi_CS_Enable(void);
static void _Spi_CS_Disable(void);
static void _Spi_Write(const uint8_t *buf, int32_t len);
static uint8_t _Spi_ReadByte(void);
static void _SetResetPin(void);
static void _ClrResetPin(void);
static void _SetPowerOnPin(void);
static void _ClrPowerOnPin(void);
static void _SetOEIPin(void);
static void _ClrOEIPin(void);
static uint32_t _ReadBusyPin(void);
static void _DelayMS(int32_t ms);
static void _OnFrameStartEvent(void);
static void _StartTimer(uint32_t ns);
static TIMER_STATE_TYPE _GetTimerState(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char *g_fb;
static struct fb_vtable_s g_dev;
struct timespec g_stop_time;

static FAR struct spi_dev_s *g_spi;
static struct et014tt1_pin_s *g_pin;

static const struct file_operations g_et014tt1fops =
{
  et014tt1_open,  /* open */
  et014tt1_close, /* close */
  0,              /* read */
  et014tt1_write, /* write */
  0,              /* seek */
  et014tt1_ioctl, /* ioctl */
};

const EPD_TCON_DRIVER_HAL g_TCON_HAL =
{
    _Spi_SetClock,
    _Spi_CS_Enable,
    _Spi_CS_Disable,
    _Spi_Write,
    _Spi_ReadByte,
    _SetResetPin,
    _ClrResetPin,
    _SetPowerOnPin,
    _ClrPowerOnPin,
    _SetOEIPin,
    _ClrOEIPin,
    _ReadBusyPin,
    _DelayMS,
    _OnFrameStartEvent,
    _StartTimer,
    _GetTimerState,
};

/****************************************************************************
 * Name: ET014TT1 Define
 ****************************************************************************/

/* Display resolution */

#define ET014TT1_DEVICE_XRES        128
#define ET014TT1_DEVICE_YRES        296

#ifdef CONFIG_EINK_ET014TT1_ROTATE
#define ET014TT1_XRES        ET014TT1_DEVICE_YRES
#define ET014TT1_YRES        ET014TT1_DEVICE_XRES
#else
#define ET014TT1_XRES        ET014TT1_DEVICE_XRES
#define ET014TT1_YRES        ET014TT1_DEVICE_YRES
#endif

/* Color depth and format */

#define ET014TT1_BPP           8
#define ET014TT1_COLORFMT      FB_FMT_Y8

/* Bytes per logical row and column */

#define ET014TT1_XSTRIDE       ((ET014TT1_XRES * ET014TT1_BPP) >> 3)
#define ET014TT1_FBSIZE        (ET014TT1_XSTRIDE*ET014TT1_YRES)

#define SPI_FREQUENCY20MHz   (20000000)
#define SPI_FREQUENCY40MHz   (40000000)
#define SPI_BITWIDTH         (8)

#define GPIO_LEVEL_HI             (true)
#define GPIO_LEVEL_LO             (false)

#define SPIDEV_EINK  SPIDEV_DISPLAY
#define ONE_SEC 1000000000

typedef enum
{
  DisplayModeInit = 0,
  DisplayModeGC,
  DisplayModeGU,
  DisplayModeA2,
} DisplayMode;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: et014tt1_getvideoinfo
 ****************************************************************************/

static int et014tt1_getvideoinfo(FAR struct fb_vtable_s *vtable,
                    FAR struct fb_videoinfo_s *vinfo)
{
  dbg("%d\n", __LINE__);
  vinfo->fmt = ET014TT1_COLORFMT; /* see FB_FMT_*  */
  vinfo->xres = ET014TT1_XRES;    /* Horizontal resolution in pixel columns */
  vinfo->yres = ET014TT1_YRES;    /* Vertical resolution in pixel rows */
  vinfo->nplanes = 1;             /* Number of color planes supported */
  return 0;
}

/****************************************************************************
 * Name: et014tt1_getplaneinfo
 ****************************************************************************/

static int et014tt1_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                    FAR struct fb_planeinfo_s *pinfo)
{
  dbg("%d\n", __LINE__);
  pinfo->fbmem = g_fb;            /* Start of frame buffer memory */
  pinfo->fblen = ET014TT1_FBSIZE; /* Length of frame buffer memory in bytes */
  pinfo->stride = ET014TT1_XSTRIDE;/* Length of a line in bytes */
  pinfo->display = 0;             /* Display number */
  pinfo->bpp = ET014TT1_BPP;      /* Bits per pixel */
  return 0;
}

/****************************************************************************
 * Name: EPD_Initialize
 ****************************************************************************/

int EPD_Initialize(void)
{
  dbg("%d %s\n", __LINE__, __TIME__);
  int ret = 0;

  ret = EPD_TCON_Init(&g_TCON_HAL);
  if (ret != 0)
    {
      dbg ("Error in EPD_TCON_Init: %d\n", ret);
      return ret;
    }

  EPD_TCON_SetVcom(-2500);

  ret = EPD_TCON_LoadWaveform(g_WaveformData);
  if (ret != 1)
    {
      dbg("Error in EPD_TCON_LoadWaveform: %d\n", ret);
      return ret;
    }

  const EPD_TCON_PANEL_INFO* paneInfo = EPD_TCON_GetPanelInfo();
  printf("panel info - width: %d, height: %d, bpp: %d\n",
                      paneInfo->Width, paneInfo->Height, paneInfo->BPP);

  return 0;
}

/****************************************************************************
 * Name: et014tt1_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int et014tt1_open(FAR struct file *filep)
{
  dbg("%d\n", __LINE__);
  g_fb = kmm_malloc(ET014TT1_FBSIZE);

  if (!g_fb)
    {
      dbg("Failed to malloc frame buffer.\n");
      return ERROR;
    }
  dbg("%d\n", __LINE__);
  int ret = EPD_Initialize();
  if (0 != ret)
    {
      return  ret;
    }

  ret = et014tt1_ioctl(filep, EINKIOC_POWERON, 0);

  return  ret;
}

/****************************************************************************
 * Name: et014tt1_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int et014tt1_close(FAR struct file *filep)
{
  dbg("%d\n", __LINE__);
  int ret = et014tt1_ioctl(filep, EINKIOC_POWEROFF, 0);

  if (g_fb)
    {
      kmm_free(g_fb);
      g_fb = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: et014tt1_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t et014tt1_write(FAR struct file *filep,
                    FAR const char *buffer, size_t buflen)
{
  dbg("%d\n", __LINE__);
  uint8_t* updateBuffer;

  /* for a local coordinate */

  int xi;
  int yi;
  int bytesi;

  /* for a e-Ink coordinate */

  int xo;
  int yo;
  int byteso;
  int strideo =  ((ET014TT1_DEVICE_XRES * ET014TT1_BPP) >> 3);


  if (buflen != ET014TT1_FBSIZE)
    {
      dbg("Expected buffer size is %d\n", ET014TT1_FBSIZE);
      return -1;
    }

  updateBuffer = EPD_TCON_GetUpdateBuffer();

  /* copy from buffer to updateBuffer. */

  for(yi = 0; yi < ET014TT1_YRES; yi++)
    {
      for(xi = 0; xi < ET014TT1_XRES; xi++)
        {
#ifdef CONFIG_EINK_ET014TT1_ROTATE

          /* Swap right side down. (rotate 90 degree right and mirroring) */
          xo = ET014TT1_YRES - yi;
          yo = xi;

          bytesi = yi * ET014TT1_XSTRIDE + xi;
          byteso = yo * strideo + xo;

#else
          xo = xi;
          yo = yi;

          bytesi = yi * ET014TT1_XSTRIDE + xi;
          byteso = bytesi;
#endif

          updateBuffer[byteso] &= 0xfc;
          updateBuffer[byteso] |= buffer[bytesi] & 0x03;
        }
    }

  EPD_TCON_Update(DisplayModeGC);
  dbg("%d ok\n", __LINE__);

  return buflen;
}

/****************************************************************************
 * Name: et014tt1_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int et014tt1_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  switch(cmd)
  {
  case EINKIOC_POWEROFF:
    dbg("EPD_TCON_PowerOff %d\n", __LINE__);
    EPD_TCON_PowerOff();
    break;

  case EINKIOC_POWERON:
    dbg("EPD_TCON_PowerOn %d\n", __LINE__);
    EPD_TCON_PowerOn();
    break;

  case EINKIOC_PANELCLEAR:
    dbg("EPD_TCON_PanelClear %d\n", __LINE__);
    EPD_TCON_PanelClear();
    break;

  default:
    dbg("Unknown command %d\n", __LINE__);
    return ERROR;
    break;

  }
  return OK;
}

/****************************************************************************
 * Name: et014tt1_initialize
 *
 * Description:Initialize LCD
 ****************************************************************************/

FAR struct fb_vtable_s *et014tt1_initialize(FAR struct spi_dev_s *spi,
                    struct et014tt1_pin_s *pin)
{
  dbg("%d\n", __LINE__);
  g_spi = spi;
  g_pin = pin;

  g_dev.getvideoinfo = et014tt1_getvideoinfo;
  g_dev.getplaneinfo = et014tt1_getplaneinfo;
  return &g_dev;
}

/****************************************************************************
 * Name: et014tt1_register
 *
 * Description:
 *   Register the et014tt1 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/eink0"
 *   dev     - An instance of the SPI interface to use to communicate with
 *             et014tt1
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int et014tt1_register(FAR const char *devpath)
{
  dbg("%d devpath %s\n", __LINE__, devpath);
  int ret;

  ret = register_driver(devpath, &g_et014tt1fops, 0666, NULL);
  if (ret < 0)
    {
      dbg("Failed to register driver: register_driver %d %s\n", ret, devpath);
      return ret;
    }

  dbg("%d ok\n", __LINE__);
  return OK;
}

/****************************************************************************
 * Name: et014tt1_configspi
 *
 * SW-TCON HAL Interface
 *
 ****************************************************************************/

void et014tt1_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the ET014TT1 */

  SPI_SETMODE(spi, SPIDEV_MODE3);
  SPI_SETBITS(spi, SPI_BITWIDTH);

  SPI_HWFEATURES(spi, 0);
}

/****************************************************************************
 * Name: _Spi_SetClock
 *
 * SW-TCON HAL Interface
 *
 ****************************************************************************/

void _Spi_SetClock(SPI_SPEED_TYPE speed)
{
  switch (speed)
    {
      case SPI_SPEED_LOW:
        SPI_SETFREQUENCY(g_spi, SPI_FREQUENCY20MHz);
        break;
      case SPI_SPEED_HIGH:
          SPI_SETFREQUENCY(g_spi, SPI_FREQUENCY40MHz);
        break;
      default:
        dbg("error in _Spi_SetClock: %d\n", speed);
        assert(0);
        break;
    }
}

/****************************************************************************
 * Name: _Spi_CS_Enable
 *
 * SW-TCON HAL Interface
 *
 ****************************************************************************/

void _Spi_CS_Enable(void)
{
  cxd56_gpio_write(g_pin->cs, GPIO_LEVEL_LO);
}

/****************************************************************************
 * Name: _Spi_CS_Disable
 *
 * SW-TCON HAL Interface
 *
 ****************************************************************************/

void _Spi_CS_Disable(void)
{
  cxd56_gpio_write(g_pin->cs, GPIO_LEVEL_HI);
}

/****************************************************************************
 * Name: _Spi_Write
 *
 * SW-TCON HAL Interface
 *
 ****************************************************************************/

void _Spi_Write(const uint8_t *buf, int32_t len)
{
  SPI_LOCK(g_spi, true);
  et014tt1_configspi(g_spi);
  SPI_SELECT(g_spi, SPIDEV_EINK, true);

  SPI_SNDBLOCK(g_spi, buf, len);
  SPI_SELECT(g_spi, SPIDEV_EINK, false);

  SPI_LOCK(g_spi, false);
}

/****************************************************************************
 * Name: _Spi_ReadByte
 *
 * SW-TCON HAL Interface
 *
 ****************************************************************************/

uint8_t _Spi_ReadByte(void)
{
  SPI_LOCK(g_spi, true);
  et014tt1_configspi(g_spi);
  SPI_SELECT(g_spi, SPIDEV_EINK, true);

  int ret = 0xFFFFFFFF;
  SPI_RECVBLOCK(g_spi, &ret, 1);

  SPI_SELECT(g_spi, SPIDEV_EINK, false);
  SPI_LOCK(g_spi, false);
  return ret & 0xff;
}

/****************************************************************************
 * Name: _SetResetPin
 *
 * SW-TCON HAL Interface
 *
 ****************************************************************************/

void _SetResetPin(void)
{
  cxd56_gpio_write(g_pin->rst, GPIO_LEVEL_HI);
}

/****************************************************************************
 * Name: _ClrResetPin
 *
 * SW-TCON HAL Interface
 *
 ****************************************************************************/

void _ClrResetPin(void)
{
  cxd56_gpio_write(g_pin->rst, GPIO_LEVEL_LO);
}

/****************************************************************************
 * Name: _SetPowerOnPin
 *
 * SW-TCON HAL Interface
 *
 ****************************************************************************/

void _SetPowerOnPin(void)
{
  if (g_pin->power >= 0)
    {
      cxd56_gpio_write(g_pin->power, GPIO_LEVEL_HI);
    }
}

/****************************************************************************
 * Name: _ClrPowerOnPin
 *
 * SW-TCON HAL Interface
 *
 ****************************************************************************/

void _ClrPowerOnPin(void)
{
  if (g_pin->power >= 0)
    {
      cxd56_gpio_write(g_pin->power, GPIO_LEVEL_LO);
    }
}

/****************************************************************************
 * Name: _SetOEIPin
 *
 * SW-TCON HAL Interface
 *
 ****************************************************************************/

void _SetOEIPin(void)
{
  if (g_pin->oei >= 0)
    {
      cxd56_gpio_write(g_pin->oei, GPIO_LEVEL_HI);
    }
}

/****************************************************************************
 * Name: _ClrOEIPin
 *
 * SW-TCON HAL Interface
 *
 ****************************************************************************/

void _ClrOEIPin(void)
{
  if (g_pin->oei >= 0)
    {
      cxd56_gpio_write(g_pin->oei, GPIO_LEVEL_LO);
    }
}

/****************************************************************************
 * Name: _ReadBusyPin
 *
 * SW-TCON HAL Interface
 *
 ****************************************************************************/

uint32_t _ReadBusyPin(void)
{
  return (uint32_t)cxd56_gpio_read(g_pin->busy);
}

/****************************************************************************
 * Name: _DelayMS
 *
 * SW-TCON HAL Interface
 *
 ****************************************************************************/

void _DelayMS(int32_t ms)
{
  up_mdelay(ms);
}

/****************************************************************************
 * Name: _OnFrameStartEvent
 *
 * SW-TCON HAL Interface
 *
 ****************************************************************************/

void _OnFrameStartEvent(void)
{
  /* nothing */
}

/****************************************************************************
 * Name: _StartTimer
 *
 * SW-TCON HAL Interface
 *
 ****************************************************************************/

void _StartTimer(uint32_t ns)
{
  up_rtc_gettime(&g_stop_time);

  struct timespec ts =
  {
    .tv_sec = ns / ONE_SEC,
    .tv_nsec = ns % ONE_SEC
  };

  clock_timespec_add(&g_stop_time, &ts, &g_stop_time);

}

/****************************************************************************
 * Name: _GetTimerState
 *
 * SW-TCON HAL Interface
 *
 ****************************************************************************/

TIMER_STATE_TYPE _GetTimerState()
{
  TIMER_STATE_TYPE state = TIMER_RUNNING;
  struct timespec cur_time;
  struct timespec rem_time;

  up_rtc_gettime(&cur_time);

  clock_timespec_subtract(&g_stop_time, &cur_time, &rem_time);
  if((0 == rem_time.tv_sec) && (0 == rem_time.tv_nsec))
    {
      state = TIMER_STOP;
    }

  return state;
}

/****************************************************************************
 * Name: __aeabi_memset
 *
 * SW-TCON HAL callback
 *
 ****************************************************************************/

void * __aeabi_memset(void *block, int c, size_t size)
{
  return(memset(block, c, size));
}

/****************************************************************************
 * Name: __aeabi_memclr
 *
 * SW-TCON HAL callback
 *
 ****************************************************************************/

void * __aeabi_memclr(void *block, int c, size_t size)
{
  return(memset(block, c, size));
}

/****************************************************************************
 * Name: __aeabi_memmove4
 *
 * SW-TCON HAL callback
 *
 ****************************************************************************/

void * __aeabi_memmove4(void *to, const void *from, size_t size)
{
  return(memmove(to, from, size));
}

