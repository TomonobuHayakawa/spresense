/****************************************************************************
 * config/spresense/src/cxd56_appinit.c
 *
 *   Copyright (C) 2016-2017 Sony Corporation. All rights reserved.
 *   Author: Nobuto Kobayashi <Nobuto.Kobayashi@sony.com>
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
#include <sdk/config.h>

#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <sys/mount.h>
#include <sys/stat.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "spresense.h"
#include "cxd56_spi.h"
#include "cxd56_i2c.h"
#include "cxd56_sysctl.h"
#include "cxd56_powermgr.h"
#include "cxd56_uart.h"
#include "cxd56_timerisr.h"
#include <arch/chip/pm.h>

#include "cxd56_pinconfig.h"
#include "cxd56_gpio.h"

#ifdef CONFIG_CXD56_RTC
#  include <nuttx/timers/rtc.h>
#  include "cxd56_rtc.h"
#endif

#ifdef CONFIG_TIMER
#  include "cxd56_timer.h"
#endif

#ifdef CONFIG_CXD56_SCU
#include <arch/chip/cxd56_scu.h>
#endif

#ifdef CONFIG_CXD56_ADC
#include <arch/chip/cxd56_adc.h>
#endif

#ifdef CONFIG_CXD56_SFC
#  include <nuttx/mtd/mtd.h>
#  include "cxd56_sfc.h"

#  ifdef CONFIG_FS_NXFFS
#    include <nuttx/fs/nxffs.h>
#  endif
#endif

#ifdef CONFIG_CXD56_SDIO
#  include <nuttx/sdio.h>
#  include <nuttx/mmcsd.h>
#  include "cxd56_sdhci.h"
#endif

#ifdef CONFIG_CXD56_CPUFIFO
#  include "cxd56_cpufifo.h"
#endif

#ifdef CONFIG_CXD56_ICC
#  include "cxd56_icc.h"
#endif

#ifdef CONFIG_CXD56_FARAPI
#  include "cxd56_farapi.h"
#endif

#ifdef CONFIG_ASMP
#  include <asmp/asmp.h>
#endif

#ifdef CONFIG_USBDEV
#  include "cxd56_usbdev.h"
#endif

#ifdef CONFIG_VIDEO_ISX012
#  include "cxd56_gpio.h"
#  include <nuttx/video/isx012.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_SFC_DEVNO
#  define CONFIG_SFC_DEVNO 0
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct pm_cpu_freqlock_s g_hv_lock =
  PM_CPUFREQLOCK_INIT(PM_CPUFREQLOCK_TAG('C','P',0), PM_CPUFREQLOCK_FLAG_HV);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_spifi_initialize
 *
 * Description:
 *
 ****************************************************************************/

#ifdef CONFIG_CXD56_SFC
static int nsh_sfc_initialize(void)
{
  FAR struct mtd_dev_s *mtd;
  int ret;

  mtd = cxd56_sfc_initialize();
  if (!mtd)
    {
      ferr("ERROR: cxd56_spifi_initialize failed\n");
      return -ENODEV;
    }

  /* use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(CONFIG_SFC_DEVNO, mtd);
  if (ret < 0)
    {
      ferr("ERROR: Initializing the FTL layer: %d\n", ret);
      return ret;
    }

#if defined(CONFIG_FS_SMARTFS)
  /* Initialize to provide SMARTFS on the MTD interface */

  ret = smart_initialize(CONFIG_SFC_DEVNO, mtd, NULL);
  if (ret < 0)
    {
      ferr("ERROR: SmartFS initialization failed: %d\n", ret);
      return ret;
    }

  ret = mount("/dev/smart0d1", "/mnt/spif", "smartfs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount the SmartFS volume: %d\n", errno);
      return ret;
    }
#elif defined(CONFIG_FS_NXFFS)
  /* Initialize to provide NXFFS on the MTD interface */

  ret = nxffs_initialize(mtd);
  if (ret < 0)
    {
      ferr("ERROR: NXFFS initialization failed: %d\n", ret);
      return ret;
    }

  ret = mount(NULL, "/mnt/spif", "nxffs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount the NXFFS volume: %d\n", errno);
      return ret;
    }
#endif

  return OK;
}
#else
#  define nsh_sfc_initialize() (OK)
#endif

#ifdef CONFIG_CXD56_CPUFIFO
static int nsh_cpucom_initialize(void)
{
  int ret = OK;

  cxd56_cfinitialize();

#ifdef CONFIG_CXD56_ICC
  cxd56_iccinitialize();
#endif
#ifdef CONFIG_CXD56_FARAPI
  cxd56_farapiinitialize();
#endif

  cxd56_sysctlinitialize();

  return ret;
}
#else
#  define nsh_cpucom_initialize() (OK)
#endif

#ifdef CONFIG_TIMER
static void timer_initialize(void)
{
  int i;
  char devname[16];

  for (i = 0; i < CXD56_TIMER_NUM; i++)
    {
      snprintf(devname, sizeof(devname), "/dev/timer%d", i);
      cxd56_timer_initialize(devname, i);
    }
  return;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
  struct pm_cpu_wakelock_s wlock;

#ifdef CONFIG_CXD56_I2C0
  FAR struct i2c_master_s *i2c0;
#endif
#ifdef CONFIG_CXD56_I2C2
  FAR struct i2c_master_s *i2c2;
#endif
#ifdef CONFIG_CXD56_SPI5
  FAR struct spi_dev_s *spi5;
#endif
#ifdef CONFIG_CXD56_SDIO
  FAR struct sdio_dev_s *sdhci0;
  struct stat stat_sdio;
#endif
  int ret;

  ret = nsh_cpucom_initialize();

  ret = cxd56_pm_initialize();
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize powermgr.\n");
    }

  wlock.info = PM_CPUWAKELOCK_TAG('C', 'A', 0);
  wlock.count = 0;
  up_pm_acquire_wakelock(&wlock);

#ifdef CONFIG_RTC_DRIVER
  rtc_initialize(0, cxd56_rtc_lowerhalf());
#endif

#ifdef CONFIG_TIMER
  timer_initialize();
#endif

  cxd56_uart_initialize();
  cxd56_timerisr_initialize();

#ifdef CONFIG_CXD56_CPUFIFO
  ret = cxd56_pm_bootup();
  if (ret < 0)
    {
      _err("ERROR: Failed to powermgr bootup.\n");
    }
#endif

  up_pm_acquire_freqlock(&g_hv_lock);

#ifdef CONFIG_CXD56_SCU
  scu_initialize();
#endif

#ifdef CONFIG_CXD56_SPI5
  /* globally initialize spi bus for peripherals */
  spi5 = cxd56_spibus_initialize(5);
  if (!spi5)
  {
    _err("ERROR: Failed to initialize spi bus.\n");
    return -ENODEV;
  }
#ifdef HAVE_SPITOOL
  cxd56_spi_register(spi5, 5);
#endif
#endif

#ifdef CONFIG_FS_PROCFS

#ifdef CONFIG_FS_PROCFS_REGISTER
  /* register usbdev procfs */

  (void)cxd56_usbdev_procfs_register();
#endif

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount the procfs. %d\n", errno);
    }
#endif

#ifdef CONFIG_PWM
  ret = board_pwm_setup();
  if (ret < 0)
    {
      _err("ERROR: Failed to initialze pwm. \n");
    }
#endif

#ifdef CONFIG_CXD56_ADC
  ret = cxd56_adcinitialize();
  if (ret < 0)
    {
      _err("ERROR: Failed to initialze adc. \n");
    }
#endif

#ifdef CONFIG_CXD56_GNSS
  ret = cxd56_gnssinitialize("/dev/gps");
  if (ret < 0)
    {
      _err("ERROR: Failed to initialze gnss. \n");
    }
#endif

#ifdef CONFIG_USERLED_LOWER
  ret = cxd56_userled_initialize("/dev/userleds");
  if (ret < 0)
    {
      _err("ERROR: Failed to initialze led. \n");
    }
#endif

#ifdef CONFIG_CXD56_GEOFENCE
  ret = cxd56_geofenceinitialize("/dev/geofence");
  if (ret < 0)
    {
      _err("ERROR: Failed to initialze geofence. \n");
    }
#endif

#ifdef CONFIG_MODEM_ALT_1160
  ret = cxd56_alt1160initialize("/dev/alt1160", spi5);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize Alt1160.\n");
    }
#endif

#ifdef CONFIG_SEDRIVER
  devse_register();
#endif

#ifdef CONFIG_ASMP
  asmp_initialize();
#endif

#ifdef CONFIG_CXD56_I2C0
  /* globally intialize i2c bus for peripherals */

  i2c0 = cxd56_i2cbus_initialize(0);
  if (!i2c0)
  {
    _err("ERROR: Failed to intialize i2c bus0.\n");
    return -ENODEV;
  }
#ifdef HAVE_I2CTOOL
  cxd56_i2c_register(i2c0, 0);
#endif
#endif
#ifdef CONFIG_CXD56_I2C2
  /* globally intialize i2c bus for peripherals */

  i2c2 = cxd56_i2cbus_initialize(2);
  if (!i2c2)
  {
    _err("ERROR: Failed to intialize i2c bus2.\n");
    return -ENODEV;
  }
#ifdef HAVE_I2CTOOL
  cxd56_i2c_register(i2c2, 2);
#endif
#endif

#ifdef CONFIG_BMP280
  ret = cxd56_bmp280initialize(i2c0);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize BMP280.\n");
    }
#endif

#ifdef CONFIG_AK09912
  ret = cxd56_ak09912initialize("/dev/accel", i2c0);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize KX022.\n");
    }
#endif

#ifdef CONFIG_KX022
  ret = cxd56_kx022initialize("/dev/accel", i2c0);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize KX022.\n");
    }
#endif

#ifdef CONFIG_LT1PA01
  ret = cxd56_lt1pa01initialize(i2c0);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize LT1PA01.\n");
    }
#endif

#ifdef CONFIG_VIDEO_ISX012
  cxd56_isx012initialize("/dev/imager", i2c2);
#endif

  ret = nsh_sfc_initialize();

#ifdef CONFIG_CXD56_SDIO
  /* Mount the SDHC-based MMC/SD block driver */
  /* This should be used with 3.3V */
  /* First, get an instance of the SDHC interface */

  finfo("Initializing SDHC slot 0\n");

  sdhci0 = cxd56_sdhci_initialize(0);
  if (!sdhci0)
    {
      _err("ERROR: Failed to initialize SDHC slot 0\n");
      return -ENODEV;
    }

  /* Now bind the SDHC interface to the MMC/SD driver */

  finfo("Bind SDHC to the MMC/SD driver, minor=0\n");

  ret = mmcsd_slotinitialize(0, sdhci0);
  if (ret != OK)
    {
      ferr("ERROR: Failed to bind SDHC to the MMC/SD driver: %d\n", ret);
      return ret;
    }

  finfo("Successfully bound SDHC to the MMC/SD driver\n");

  /* Handle the initial card state */

  cxd56_sdhci_mediachange(sdhci0);

  if (stat("/dev/mmcsd0", &stat_sdio) == 0)
    {
      if (S_ISBLK(stat_sdio.st_mode))
        {
          ret = mount("/dev/mmcsd0", "/mnt/sd0", "vfat", 0, NULL);
          if (ret == 0)
            {
              _err("Successfully mount a SDCARD via the MMC/SD driver\n");
            }
          else
            {
              _err("ERROR: Failed to mount the SDCARD. %d\n", errno);
            }
        }
    }
#endif

#ifdef CONFIG_CPUFREQ_RELEASE_LOCK
  up_pm_release_freqlock(&g_hv_lock);
#endif

  up_pm_release_wakelock(&wlock);

  return 0;
}

#ifdef CONFIG_BOARDCTL_IOCTL
int board_ioctl(unsigned int cmd, uintptr_t arg)
{
  int ret = -1;

  switch (cmd)
    {
#ifdef CONFIG_CXD56_USBDEV
      /* CMD:           BOARDIOC_USBDEV_SETNOTIFYSIG
       * DESCRIPTION:   Set signal id for notify USB device connection status and supply current value.
       * ARG:           None
       * CONFIGURATION: CONFIG_LIB_BOARDCTL
       * DEPENDENCIES:  Board logic must provide board_app_initialization
       */

      case BOARDIOC_USBDEV_SETNOTIFYSIG:
        {
          ret = cxd56_usbdev_setsigno((int)arg);
        }
        break;
#endif
      default:
        break;
    }

  /* Set the errno value on any errors */

  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}
#endif

/* This function is provided outside exported SDK, therefore here is defined
 * as weak symbol to avoid link error.
 */

int weak_function spresense_main(int argc, char *argv[])
{
  return 0;
}
