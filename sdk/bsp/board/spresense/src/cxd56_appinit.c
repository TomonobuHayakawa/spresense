/****************************************************************************
 * config/spresense/src/cxd56_appinit.c
 *
 *   Copyright (C) 2016-2017 Sony Corporation. All rights reserved.
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

#ifdef CONFIG_CXD56_GNSS
#  include "cxd56_gnss.h"
#endif

#ifdef CONFIG_CXD56_GEOFENCE
#  include "cxd56_geofence.h"
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct pm_cpu_freqlock_s g_hv_lock =
  PM_CPUFREQLOCK_INIT(PM_CPUFREQLOCK_TAG('C','P',0), PM_CPUFREQLOCK_FLAG_HV);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

#ifdef CONFIG_USERLED_LOWER
  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      _err("ERROR: Failed to initialze led. \n");
    }
#endif

#ifdef CONFIG_CXD56_GNSS
  ret = cxd56_gnssinitialize("/dev/gps");
  if (ret < 0)
    {
      _err("ERROR: Failed to initialze gnss. \n");
    }
#endif

#ifdef CONFIG_CXD56_GEOFENCE
  ret = cxd56_geofenceinitialize("/dev/geofence");
  if (ret < 0)
    {
      _err("ERROR: Failed to initialze geofence. \n");
    }
#endif

#ifdef CONFIG_SEDRIVER
  devse_register();
#endif

#ifdef CONFIG_ASMP
  asmp_initialize();
#endif

#ifdef CONFIG_SYSTEM_I2CTOOL
  /* Initialize to use system/i2ctool for all of the i2c port */

  for (int bus = 0; bus <= 2; bus++)
    {
      board_i2cdev_initialize(bus);
    }
#endif

#ifdef CONFIG_BMP280
  ret = board_bmp280_initialize(0);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize BMP280.\n");
    }
#endif

#ifdef CONFIG_AK09912
  ret = board_ak09912_initialize("/dev/mag", 0);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize AK09912.\n");
    }
#endif

#ifdef CONFIG_BMI160
  int bmi160_bus;

#  ifdef CONFIG_BMI160_I2C
  bmi160_bus = 0; /* I2C0 */
#  else /*  CONFIG_BMI160_SPI */
  bmi160_bus = 3; /* SPI3 */
#  endif
  ret = board_bmi160_initialize(bmi160_bus);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize BMI160.\n");
    }
#endif

#ifdef CONFIG_KX022
  ret = board_kx022_initialize("/dev/accel", 0);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize KX022.\n");
    }
#endif

#ifdef CONFIG_APDS9930
  ret = board_apds9930_initialize(0);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize APDS9930.\n");
    }
#endif

#ifdef CONFIG_LT1PA01
  ret = board_lt1pa01_initialize(0);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize LT1PA01.\n");
    }
#endif

#ifdef CONFIG_RPR0521RS
  ret = board_rpr0521rs_initialize(0);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize RPR0521RS.\n");
    }
#endif

#ifdef CONFIG_BH1721FVC
  ret = board_bh1721fvc_initialize("/dev/light", 0);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize BH1721FVC.\n");
    }
#endif

#ifdef CONFIG_BH1745NUC
  ret = board_bh1745nuc_initialize("/dev/color", 0);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize BH1745NUC.\n");
    }
#endif

#ifdef CONFIG_VIDEO_ISX012
  board_isx012_initialize("/dev/imager", 2);
#endif

#ifdef CONFIG_CXD56_SFC
  ret = board_flash_initialize();
  if (ret < 0)
    {
      _err("ERROR: Failed to initialze SPI-Flash. %d\n", errno);
    }
#endif

#if defined(CONFIG_CXD56_SDIO) && !defined(CONFIG_CXD56_SPISD)
  ret = board_sdcard_initialize();
  if (ret < 0)
    {
      _err("ERROR: Failed to initialze sdhci. \n");
    }
#endif

#ifdef CONFIG_CXD56_SPISD
  /* Mount the SPI-based MMC/SD block driver */

  ret = board_spisd_initialize(0, 4);
  if (ret < 0)
    {
      ferr("ERROR: Failed to initialize SPI device to MMC/SD: %d\n",
           ret);
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
