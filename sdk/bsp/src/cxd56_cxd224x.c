/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_cxd224x.c
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
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
#include <sys/time.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "cxd56_i2c.h"
#include "cxd56_pinconfig.h"
#include "cxd56_gpio.h"
#include "cxd56_gpioint.h"

#include <arch/chip/cxd224x.h>
#include "cxd56_cxd224x.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_BUFFER_SIZE 780

#define WAIT_PTRN_CLEAR 0x00000000
#define WAIT_PTRN_READ  0x00000001
#define WAIT_PTRN_CLOSE 0x00000002

#define PACKET_HEADER_SIZE_NCI  (3)
#define MAX_PACKET_SIZE         (PACKET_HEADER_SIZE_NCI + 255)

#if 0
#define DBG_I2C_ERROR_PRINT(format,...) dbg(format, ##__VA_ARGS__)
#define DBG_I2C_DEBUG_PRINT(format,...) vdbg(format, ##__VA_ARGS__)
#define DBG_I2C_WARN_PRINT(format,...)  vdbg(format, ##__VA_ARGS__)
#else
#define DBG_I2C_ERROR_PRINT(format,...)
#define DBG_I2C_DEBUG_PRINT(format,...)
#define DBG_I2C_WARN_PRINT(format,...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int cxd224x_i2c_host_int_hdr(int irq, FAR void *context, void *dummy);

static int     cxd224x_i2c_open(FAR struct file *filep);
static int     cxd224x_i2c_close(FAR struct file *filep);
static ssize_t cxd224x_i2c_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen);
static ssize_t cxd224x_i2c_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t buflen);
static int     cxd224x_i2c_ioctl(FAR struct file *filep, int cmd,
                                 unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int     cxd224x_i2c_poll(FAR struct file *filep, struct pollfd *fds,
                                bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations cxd224x_i2c_fops =
{
  cxd224x_i2c_open,
  cxd224x_i2c_close,
  cxd224x_i2c_read,
  cxd224x_i2c_write,
  0,
  cxd224x_i2c_ioctl,
#ifndef CONFIG_DISABLE_POLL
  cxd224x_i2c_poll,
#endif
  0
};

static unsigned int g_count_irq;

static sem_t exclsem;
static sem_t* p_waitsem;

static uint32_t g_ptn;

/* GPIO */

static int g_host_int = PIN_SPI2_MOSI;
static int g_xrst     = PIN_SPI2_CS_X;
static int g_wake     = PIN_SPI2_MISO;

/* I2C */

static struct i2c_master_s* i2c_dev = NULL;
static const struct i2c_config_s  i2c_config =
{
  .frequency = 400000,          /* 400kHz */
  .address   = 0x28,            /* Slave address */
  .addrlen   = 7,               /* Bit length of Slave address */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int cxd224x_i2c_open(FAR struct file *filep)
{
  int ret;

  DBG_I2C_WARN_PRINT("%s: start\n", __func__);

  DEBUGASSERT(filep && filep->f_inode);
  if (filep->f_inode->i_crefs > 1)
    {
      return ERROR;
    }

  cxd56_gpio_config(g_xrst, false);
  cxd56_gpio_write(g_xrst, false);

  cxd56_gpio_config(g_wake, false);
  cxd56_gpio_write(g_wake, true);

  ret = cxd56_gpioint_config(g_host_int,
                             GPIOINT_NOISE_FILTER_DISABLE |
                             GPIOINT_LEVEL_LOW, cxd224x_i2c_host_int_hdr);
  if (ret < 0)
    {
      return ERROR;
    }

  sem_init(&exclsem, 0, 1);

  g_count_irq = 0;
  filep->f_inode->i_crefs++;

  return OK;
}

static int cxd224x_i2c_close(FAR struct file *filep)
{
  int rtcd = OK;

  DBG_I2C_WARN_PRINT("%s: start\n", __func__);

  /* cxd56_gpioint_wake_disable(g_host_int); */
  cxd56_gpioint_disable(g_host_int);

  sem_destroy(&exclsem);

  /* TODO move to other or keep here?? */
  /* cxd56_i2cbus_uninitialize(i2c_dev); */

  filep->f_inode->i_crefs--;

  return rtcd;
}

static ssize_t cxd224x_i2c_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  unsigned char tmp[MAX_BUFFER_SIZE];
  int total, len, ret;

  total = 0;
  len = 0;

  /* FIXME: This function not processed when no debug */

  DEBUGASSERT(cxd56_gpio_read(g_host_int) == 0);

  ret = sem_wait(&exclsem);
  DEBUGASSERT(ret == 0);

  DBG_I2C_WARN_PRINT("%s: start\n", __func__);

  ret = i2c_read(i2c_dev, &i2c_config, tmp, PACKET_HEADER_SIZE_NCI);
  if( ret < 0 )
    {
      DBG_I2C_ERROR_PRINT("i2c_read failed: %d\n", ret);
      return ret;
    }

  if (tmp[0] != 0xff)
    {
      total = PACKET_HEADER_SIZE_NCI;
      len = tmp[PACKET_HEADER_SIZE_NCI-1];

      if (len > 0 && (len + total) <= buflen)
        {
          int remain = len;
          for(;;)
            {
              int read_sz;

#define RDBLKSZ 32
              if (remain > RDBLKSZ)
                {
                  read_sz = RDBLKSZ;
                }
              else
                {
                  read_sz = remain;
                }

              ret = i2c_read(i2c_dev, &i2c_config, tmp + total, read_sz);
              if (ret < 0)
                {
                  DBG_I2C_ERROR_PRINT("i2c_read failed: %d\n", ret);
                  return ret;
                }
              total += read_sz;
              remain -= read_sz;
              if (remain == 0)
                {
                  break;
                }
            }
        }
    }

#ifdef SPZ_DEBUG
  int i;
  DBG_I2C_WARN_PRINT("read:");
  for(i = 0; i < total; i++)
    {
      DBG_I2C_WARN_PRINT("%02x", tmp[i]);
    }
  DBG_I2C_WARN_PRINT("\n");
#endif /* SPZ_DEBUG */

  ret = sem_post(&exclsem);
  DEBUGASSERT(ret==0);

  if (total > buflen) {
    DBG_I2C_ERROR_PRINT("%s: failed to copy to user space, total = %d\n",
                        __func__, total);
    return ERROR;
  }

  memcpy(buffer, tmp, total);

  DBG_I2C_WARN_PRINT("%s: end\n", __func__);

  return total;
}

static ssize_t cxd224x_i2c_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t buflen)
{
  unsigned char tmp[MAX_BUFFER_SIZE];
  int ret;

  if (buflen > MAX_BUFFER_SIZE) {
    DBG_I2C_ERROR_PRINT("%s: out of memory\n", __func__);
    return ERROR;
  }

  memcpy(tmp, buffer, buflen);

  ret = sem_wait(&exclsem);
  DEBUGASSERT(ret==0);

  DBG_I2C_WARN_PRINT("%s: start\n", __func__);

#ifdef SPZ_DEBUG
  int i;
  DBG_I2C_WARN_PRINT("write:");
  for (i = 0; i < buflen; i++)
    {
      DBG_I2C_WARN_PRINT("%02x", buffer[i]);
    }
  DBG_I2C_WARN_PRINT("\n");
#endif /* SPZ_DEBUG */

  ret = i2c_write(i2c_dev, &i2c_config, tmp, buflen);
  if (ret < 0)
    {
      DBG_I2C_ERROR_PRINT("%s: i2c_write Fail ret = %d\n", __func__, ret);
      return ret;
    }

  ret = sem_post(&exclsem);
  DEBUGASSERT(ret == 0);

  DBG_I2C_WARN_PRINT("%s: end\n", __func__);

  return buflen;
}

#ifndef CONFIG_DISABLE_POLL
static int cxd224x_i2c_poll(FAR struct file *filep, FAR struct pollfd *fds,
                            bool setup)
{
  int ret = 0;
  uint32_t ptn;

  DBG_I2C_WARN_PRINT("%s: start(setup %d)\n", __func__, setup);
  DEBUGASSERT(fds[0].fd >= 0);

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      if ((g_ptn & WAIT_PTRN_CLOSE) != 0)
        {
          ret = -EINTR;
        }
      else
        {
          g_ptn = WAIT_PTRN_CLEAR;
          p_waitsem = fds[0].sem;
          DEBUGASSERT(p_waitsem!=NULL);
          cxd56_gpioint_enable(g_host_int);
          /* cxd56_gpioint_wake_enable(g_host_int); */
          ret = 0;
        }
    }
  else
    {
      p_waitsem = NULL;
      fds[0].revents |= POLLIN;
      ptn = g_ptn;
      g_ptn = WAIT_PTRN_CLEAR;
      if ((ptn & WAIT_PTRN_READ) != 0)
        {
          if (g_count_irq > 0)
            {
              g_count_irq--;
              ret = 0; /* success */
            }
          else
            {
              /* FIXME: */
              /* cxd56_gpioint_enable(g_host_int); */
            }
        }
      else if ((ptn & WAIT_PTRN_CLOSE) != 0) {
        ret = -EINTR;
      }
    }

  DBG_I2C_WARN_PRINT("%s: end(setup %d)\n", __func__, setup);

  return ret;
}
#endif

static int cxd224x_i2c_host_int_hdr(int irq, FAR void *context, void *dummy)
{
  int ret;
  int pin;

  DBG_I2C_WARN_PRINT("%s: HostIntHdr irq %d(%d) start\n", __func__ , irq,
                     g_count_irq );
  cxd56_gpioint_disable(g_host_int);
  g_count_irq++;

  pin = cxd56_gpio_read(g_host_int);
  if (pin != 0)
    {
      DBG_I2C_ERROR_PRINT("%s: g_host_int : %d - False interrupt\n", __func__,
                          pin);
      return OK;
    }

  DEBUGASSERT(p_waitsem!=NULL);
  g_ptn |= WAIT_PTRN_READ;
  ret = sem_post(p_waitsem);
  if (ret < 0)
    {
      DBG_I2C_ERROR_PRINT("%s: sem_post Fail ret = %d\n", __func__, ret);
      return ret;
    }
  p_waitsem = NULL;

  return OK;
}

int cxd224x_i2c_WakeupSig(void)
{
  int ret;

  DBG_I2C_DEBUG_PRINT("%s: i2c_WakeupSig\n", __func__);
  g_ptn |= WAIT_PTRN_CLOSE;
  if (p_waitsem==NULL)
    {
      return OK;
    }

  ret = sem_post(p_waitsem);
  if (ret < 0)
    {
      DBG_I2C_ERROR_PRINT("%s: sem_post Fail ret = %d\n", __func__, ret);
      return ret;
    }

  return OK;
}

int cxd224x_i2c_ResetSig(void)
{
  return OK;
}

static int cxd224x_i2c_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  DBG_I2C_WARN_PRINT("%s: start\n", __func__);

  switch(cmd) {
    case CXDNFC_RST_CTL:
      cxd56_gpio_write(g_xrst, true);
      g_count_irq = 0;
      usleep(10000);
      cxd56_gpio_write(g_xrst, false);
      break;

    case CXDNFC_POWER_CTL:
#if defined(CONFIG_NFC_CXD224X_VEN) || defined(CONFIG_NFC_CXD224X_VEN_MODULE)
      if(arg == 0)
        {
          cxd56_gpio_write(g_pon, true);
        }
      else if (arg == 1)
        {
          cxd56_gpio_write(g_pon, false);
        }
#else
      return 1; /* not support */
#endif
      break;

    case CXDNFC_WAKE_CTL:
      if (arg == 0)
        {
          cxd56_gpio_write(g_wake, true);
        }
      else if (arg == 1)
        {
          cxd56_gpio_write(g_wake, false);
        }
      break;

    default:
      break;
  }

  DBG_I2C_WARN_PRINT("%s: end\n", __func__);

  return OK;
}

void cxd224x_initialize( FAR const char *devpath, int bus)
{
  int ret;

  DBG_I2C_DEBUG_PRINT("%s: Enter cxd224x_initialize()\n", __func__);

  /* Initialize i2c deivce */

  i2c = cxd56_i2cbus_initialize(bus);
  if (!i2c)
    {
      _err("ERROR: Failed to initialize i2c%d.\n", bus);
      return;
    }

  i2c_dev = i2c;

  ret = register_driver(devpath, &cxd224x_i2c_fops, 0666, NULL);
  if (ret < 0)
    {
      _err("register_driver failure.\n");
    }
}
