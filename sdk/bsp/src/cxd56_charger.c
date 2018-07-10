/****************************************************************************
 * bsp/src/cxd56_battery.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

/* CXD5247GF is Li-Ion Battery Charger with Power-Path Management.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <math.h>

#include <nuttx/kmalloc.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>

#include <arch/chip/battery_ioctl.h>

#include "cxd56_pmic.h"

#ifdef CONFIG_CXD56_CHARGER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_CXD56_CHARGER_DEBUG
#define baterr(fmt, ...) logerr(fmt, ## __VA_ARGS__)
#else
#define baterr(fmt, ...)
#endif

/* Configuration */

#undef USE_FLOAT_CONVERSION

#ifdef CONFIG_CXD56_BATTERY_TEMP_PRECISE
#  if !defined(CONFIG_LIBM)
#    error Temperature conversion in float requires math library.
#  endif
#  define USE_FLOAT_CONVERSION 1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct charger_dev_s
{
  sem_t batsem;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int charger_open(FAR struct file *filep);
static int charger_close(FAR struct file *filep);
static ssize_t charger_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t charger_write(FAR struct file *filep,
                             FAR const char *buffer, size_t buflen);
static int charger_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_batteryops =
{
  charger_open,   /* open */
  charger_close,  /* close */
  charger_read,   /* read */
  charger_write,  /* write */
  0,              /* seek */
  charger_ioctl   /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL          /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL          /* unlink */
#endif
};

static struct charger_dev_s g_chargerdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: charger_therm2temp
 *
 * Description:
 *   Convert temperature register value to degrees Celsius.
 *
 ****************************************************************************/

static int charger_therm2temp(int val)
{
#ifdef USE_FLOAT_CONVERSION
  float f1, f2, f3, f4, f5, f6;

  f1 = (float)val;
  f2 = f1 / 4096.f;
  f3 = f2 * 100.f / (1.f - f2);
  f4 = f3 / 100.f;
  f5 = logf(f4);
  f6 = 1.f / (f5 / 4250.f + 1.f / 298.f) - 273.f;

  return (int)f6;
#else
  static short T[29] =  /* -40,-35,..-20,-15,..,95,100 */
    { 4020, 3986, 3939, 3877, 3759, 3691, 3562, 3405, 3222, 3015, /* -40,.. */
      2787, 2545, 2296, 2048, 1808, 1582, 1374, 1186, 1020,  874, /*  10,.. */
      747,  639 , 546,  467,  400,  343,  295,  254,  220 };      /*  60,..,100 */
  int i;
  int t0 = -45;
  int t1 = -40;
  int tt = -45;

  for (i = 0; i < 29; i++)
    {
      if (val > T[i])
        {
          break;
        }
      t0 += 5;
      t1 += 5;
    }

  if (i > 0)
    {
      int diff = T[i - 1] - T[i];
      tt = t1 - (val - T[i]) * 5 / diff; /* interpolation : not accurate */
    }

  return tt;
#endif
}

static int charger_get_status(FAR enum battery_charger_status_e *status)
{
  uint8_t state;
  int ret;

  if (status == NULL)
    {
      return -EINVAL;
    }

  *status = BATTERY_UNKNOWN;

  ret = cxd56_pmic_getchargestate(&state);
  if (ret < 0)
    {
      return -EIO;
    }

  switch (state)
    {
      /* Is there some fault in the battery? */

      case PMIC_STAT_BAT_UNUSUAL:
        *status = BATTERY_FAULT;
        break;

      /* Is the charging done? */

      case PMIC_STAT_CHG_COMPLETE:
        *status = BATTERY_FULL;
        break;

      /* Is the charging in progress? */

      case PMIC_STAT_GB_QCKCHARGE:
      case PMIC_STAT_GB_LOWCHARGE:
      case PMIC_STAT_GB_HIGHCHARGE:
        *status = BATTERY_CHARGING;
        break;

      /* Is the charging ready? */

      case PMIC_STAT_CHG_STOP:
        *status = BATTERY_DISCHARGING;
        break;

      default:
        *status = BATTERY_IDLE;
        break;
    }

  return OK;
}

static int charger_get_health(FAR enum battery_charger_health_e *health)
{
  FAR struct pmic_gauge_s gauge;
  uint8_t state;
  int temp;
  int ret;

  if (health == NULL)
    {
      return -EINVAL;
    }

  *health = BATTERY_HEALTH_UNKNOWN;

  ret = cxd56_pmic_get_gauge(&gauge);
  if (ret < 0)
    {
      return -EIO;
    }

  ret = cxd56_pmic_getchargestate(&state);
  if (ret < 0)
    {
      return -EIO;
    }

  /* Convert register value to degrees Celsius */

  temp = charger_therm2temp(gauge.temp);

  if (temp < 10)
    {
      *health = BATTERY_HEALTH_COLD;
    }
  else if (temp > 60)
    {
      *health = BATTERY_HEALTH_OVERHEAT;
    }
  else
    {
      *health = BATTERY_HEALTH_GOOD;
    }

  return OK;
}


/****************************************************************************
 * Name: battery_open
 *
 * Description:
 *   This function is called whenever the battery device is opened.
 *
 ****************************************************************************/

static int charger_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: battery_close
 *
 * Description:
 *   This routine is called when the battery device is closed.
 *
 ****************************************************************************/

static int charger_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: battery_read
 ****************************************************************************/

static ssize_t charger_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  /* Return nothing read */

  return 0;
}

/****************************************************************************
 * Name: battery_write
 ****************************************************************************/

static ssize_t charger_write(FAR struct file *filep,
                             FAR const char *buffer, size_t buflen)
{
  /* Return nothing written */

  return 0;
}

/****************************************************************************
 * Name: battery_ioctl
 ****************************************************************************/

static int charger_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct charger_dev_s *priv  = inode->i_private;
  int ret = -ENOTTY;

  sem_wait(&priv->batsem);
  
  switch (cmd)
    {
      case BATIOC_STATE:
        {
          FAR enum battery_charger_status_e *status =
            (FAR enum battery_charger_status_e *)(uintptr_t)arg;
          ret = charger_get_status(status);
        }
        break;

      case BATIOC_HEALTH:
        {
          FAR enum battery_charger_health_e *health =
            (FAR enum battery_charger_health_e *)(uintptr_t)arg;
          ret = charger_get_health(health);
        }
        break;

      case BATIOC_VOLTAGE:
        {
          FAR int *voltage = (FAR int *)(uintptr_t)arg;
        }
        break;

      case BATIOC_GET_CHGVOLTAGE:
        {
          FAR int *voltage = (FAR int *)(uintptr_t)arg;
          ret = cxd56_pmic_getchargevol(voltage);
        }
        break;

      case BATIOC_SET_CHGVOLTAGE:
        {
          int voltage = (int)arg;
          ret = cxd56_pmic_setchargevol(voltage);
        }
        break;

      case BATIOC_GET_CHGCURRENT:
        {
          FAR int *current = (FAR int *)(uintptr_t)arg;
          ret = cxd56_pmic_getchargecurrent(current);
        }
        break;

      case BATIOC_SET_CHGCURRENT:
        {
          int current = (int)arg;
          ret = cxd56_pmic_setchargecurrent(current);
        }
        break;

      case BATIOC_GET_RECHARGEVOL:
        {
          FAR int *voltage = (FAR int *)(uintptr_t)arg;
          ret = cxd56_pmic_getrechargevol(voltage);
        }
        break;

      case BATIOC_SET_RECHARGEVOL:
        {
          int voltage = (int)arg;
          ret = cxd56_pmic_setrechargevol(voltage);
        }
        break;

      case BATIOC_GET_COMPCURRENT:
        {
          FAR int *current = (FAR int *)(uintptr_t)arg;
          ret = cxd56_pmic_getchargecompcurrent(current);
        }
        break;

      case BATIOC_SET_COMPCURRENT:
        {
          int current = (int)arg;
          ret = cxd56_pmic_setchargecompcurrent(current);
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  sem_post(&priv->batsem);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_batinitialize
 *
 * Description:
 *   Initialize the CXD5247 battery driver.
 *
 * Input Parameters:
 *   devpath - Device file path
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int cxd56_charger_initialize(FAR const char *devpath)
{
  FAR struct charger_dev_s *priv = &g_chargerdev;
  int ret;

  /* Initialize the CXD5247 device structure */

  sem_init(&priv->batsem, 0, 1);

  /* Register battery driver */

  ret = register_driver(devpath, &g_batteryops, 0666, priv);
  if (ret < 0)
    {
      baterr("ERROR: register_driver failed: %d\n", ret);
      return -EFAULT;
    }

  return OK;
}

int cxd56_charger_uninitialize(FAR const char *devpath)
{
  (void) unregister_driver(devpath);

  return OK;
}

#endif /* CONFIG_CXD56_BATTERY */
