/****************************************************************************
 * drivers/modem/alt1160.c
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

#include <sdk/config.h>

#include <arch/board/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/modem/alt1160.h>
#include "alt1160_dev.h"
#include "alt1160_spi.h"
#include "alt1160_pm.h"
#include "alt1160_sys.h"

#if defined(CONFIG_MODEM_ALT_1160)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods. */

static int     alt1160_open(FAR struct file *filep);
static int     alt1160_close(FAR struct file *filep);
static ssize_t alt1160_read(FAR struct file *filep, FAR char *buffer,
                            size_t len);
static ssize_t alt1160_write(FAR struct file *filep, FAR const char *buffer,
                             size_t len);
static int     alt1160_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface. */

static const struct file_operations g_alt1160fops =
{
  alt1160_open,                 /* open */
  alt1160_close,                /* close */
  alt1160_read,                 /* read */
  alt1160_write,                /* write */
  0,                            /* seek */
  alt1160_ioctl,                /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: is_poweron
 *
 * Description:
 *   Check ALT1160 power on.
 *
 ****************************************************************************/

static int is_poweron(FAR struct alt1160_dev_s *priv)
{
  int poweron;

  alt1160_sys_lock(&priv->lock);

  poweron = priv->poweron;

  alt1160_sys_unlock(&priv->lock);

  return poweron;
}

/****************************************************************************
 * Name: alt1160_initialize
 *
 * Description:
 *   Initialize ALT1160 driver.
 *
 ****************************************************************************/

static int alt1160_initialize(FAR struct alt1160_dev_s *priv)
{
  int ret;

  ret = alt1160_sys_initlock(&priv->lock);
  if (ret == ERROR)
    {
      m_err("alt1160_sys_initlock() failed:%d\n", ret);
    }

  priv->poweron = 0;

  return ret;
}

/****************************************************************************
 * Name: alt1160_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int alt1160_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: alt1160_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int alt1160_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: alt1160_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t alt1160_read(FAR struct file *filep, FAR char *buffer,
                            size_t len)
{
  FAR struct inode         *inode = filep->f_inode;
  FAR struct alt1160_dev_s *priv  = inode->i_private;
  ssize_t                  rsize  = ERROR;

  if (is_poweron(priv))
    {
      rsize = alt1160_spi_read(priv, buffer, len);
    }

  return rsize;
}

/****************************************************************************
 * Name: alt1160_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t alt1160_write(FAR struct file *filep, FAR const char *buffer,
                             size_t len)
{
  FAR struct inode         *inode = filep->f_inode;
  FAR struct alt1160_dev_s *priv  = inode->i_private;
  ssize_t                  wsize  = ERROR;

  if (is_poweron(priv))
    {
      wsize = alt1160_spi_write(priv, buffer, len);
    }

  return wsize;
}

/****************************************************************************
 * Name: alt1160_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int alt1160_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode         *inode = filep->f_inode;
  FAR struct alt1160_dev_s *priv  = inode->i_private;
  int                      ret    = -EPERM;

  switch (cmd)
    {
      case MODEM_IOC_POWERON:  /* Power on ALT1160. */
        {
          alt1160_sys_lock(&priv->lock);

          if (!priv->poweron)
            {
              board_alt1160_power_control(true);

              /* Wait until the communication is ready to start. */

              sleep(CONFIG_MODEM_ALT_1160_BOOT_DELAY);

              /* Intialize ALT1160 SPI driver. */

              ret = alt1160_spi_init(priv);
              priv->poweron = 1;
            }

          alt1160_sys_unlock(&priv->lock);
        }
        break;

      case MODEM_IOC_POWEROFF:  /* Power off ALT1160. */
        {
          alt1160_sys_lock(&priv->lock);

          if (priv->poweron)
            {
              /* Unintialize ALT1160 SPI driver */

              ret = alt1160_spi_uninit(priv);

              board_alt1160_power_control(false);
              priv->poweron = 0;
            }

          alt1160_sys_unlock(&priv->lock);
        }
      break;

      case MODEM_IOC_READABORT:  /* Abort the read process. */
        {
          if (is_poweron(priv))
            {
              ret = alt1160_spi_readabort(priv);
            }
        }
        break;

      case MODEM_IOC_SLEEP:  /* Make ALT1160 sleep. */
        {
          if (is_poweron(priv))
            {
              ret = alt1160_spi_sleepmodem(priv);
            }
        }
        break;

      case MODEM_IOC_PM_REGISTERCB:  /* Register callback function. */
        {
          if (is_poweron(priv))
            {
              ret = alt1160_pm_registercb((alt1160_pm_cbfunc_t)arg);
            }
        }
        break;

      case MODEM_IOC_PM_DEREGISTERCB:  /* Deregister callback function. */
        {
          if (is_poweron(priv))
            {
              ret = alt1160_pm_deregistercb();
            }
        }
        break;

      case MODEM_IOC_PM_GETSTATE:  /* Get ALT1160 power management state. */
        {
          if (is_poweron(priv))
            {
              *(uint32_t *)arg = alt1160_pm_getstate();
              ret = 0;
            }
        }
        break;

      case MODEM_IOC_PM_INITWAKELOCK:  /* Initialze wakelock resource. */
        {
          if (is_poweron(priv))
            {
              ret = alt1160_pm_initwakelock(
                (struct alt1160_pm_wakelock_s *)arg);
            }
        }
        break;

      case MODEM_IOC_PM_ACQUIREWAKELOCK:  /* Acquire wakelock. */
        {
          if (is_poweron(priv))
            {
              ret = alt1160_pm_acquirewakelock(
                (struct alt1160_pm_wakelock_s *)arg);
            }
        }
        break;

      case MODEM_IOC_PM_RELEASEWAKELOCK:  /* Release wakelock. */
        {
          if (is_poweron(priv))
            {
              ret = alt1160_pm_releasewakelock(
                (struct alt1160_pm_wakelock_s *)arg);
            }
        }
        break;

      case MODEM_IOC_PM_GETNUMOFWAKELOCK:  /* Get number of wakelocks. */
        {
          if (is_poweron(priv))
            {
              ret = alt1160_pm_getnumofwakelock(
                (struct alt1160_pm_wakelock_s *)arg);
            }
        }
        break;

      case MODEM_IOC_PM_GETWAKELOCKSTATE:  /* Get wakelock state. */
        {
          if (is_poweron(priv))
            {
              ret = alt1160_pm_getwakelockstate();
            }
        }
        break;

      default:
        m_err("Unrecognized cmd: 0x%08x\n", cmd);
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: alt1160_register
 *
 * Description:
 *   Register the ALT1160 character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/alt1160".
 *   dev     - An instance of the SPI interface to use to communicate with
 *             ALT1160.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int alt1160_register(FAR const char *devpath, FAR struct spi_dev_s *dev)
{
  FAR struct alt1160_dev_s *priv;
  int                      ret;
  int                      size = sizeof(struct alt1160_dev_s);

  priv = (FAR struct alt1160_dev_s *)kmm_malloc(size);
  if (!priv)
    {
      m_err("Failed to allocate instance.\n");
      return -ENOMEM;
    }

  priv->spi = dev;

  ret = alt1160_initialize(priv);
  if (ret < 0)
    {
      m_err("Failed to initialize ALT1160 driver.\n");
      kmm_free(priv);
      return ret;
    }

  ret = register_driver(devpath, &g_alt1160fops, 0666, priv);
  if (ret < 0)
    {
      m_err("Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  return OK;
}

#endif
