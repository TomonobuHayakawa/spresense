/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_i2c.c
 *
 *   Copyright (C) 2016 Sony Corporation. All rights reserved.
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
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "cxd56_clock.h"
#include "cxd56_i2c.h"
#include "chip/cxd56_i2c.h"
#include "cxd56_pinconfig.h"

#if defined(CONFIG_CXD56_I2C0_SCUSEQ) || defined(CONFIG_CXD56_I2C1_SCUSEQ)
#include <arch/chip/cxd56_scu.h>
#endif

#ifdef CONFIG_CXD56_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define I2C_TIMEOUT  (20*1000/CONFIG_USEC_PER_TICK) /* 20 mS */

#define I2C_DEFAULT_FREQUENCY 400000

#define I2C_INTR_ENABLE ((INTR_STOP_DET) | \
                         (INTR_TX_ABRT)  | \
                         (INTR_TX_OVER)  | \
                         (INTR_RX_OVER)  | \
                         (INTR_RX_UNDER))

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct cxd56_i2cdev_s
{
  struct i2c_master_s dev;     /* Generic I2C device */
  unsigned int     base;       /* Base address of registers */
  uint16_t         irqid;      /* IRQ for this device */
  int8_t           port;       /* Port number */
  uint32_t         baseFreq;   /* branch frequency */

  sem_t            mutex;      /* Only one thread can access at a time */
  sem_t            wait;       /* Place to wait for transfer completion */
  WDOG_ID          timeout;    /* watchdog to timeout when bus hung */
  uint32_t         frequency;  /* Current I2C frequency */

  struct i2c_msg_s *msgs;

  int              error;      /* Error status of each transfers */
  int              refs;       /* Reference count */
};

/* Channel 0 as SCU_I2C0
 * Channel 1 as SCU_I2C1
 * Channel 2 as I2CM
 */

#ifdef CONFIG_CXD56_I2C0
static struct cxd56_i2cdev_s g_i2c0dev =
{
  .port = 0,
  .base = CXD56_SCU_I2C0_BASE,
  .irqid = CXD56_IRQ_SCU_I2C0,
  .refs = 0,
};
#endif
#ifdef CONFIG_CXD56_I2C1
static struct cxd56_i2cdev_s g_i2c1dev =
{
  .port = 1,
  .base = CXD56_SCU_I2C1_BASE,
  .irqid = CXD56_IRQ_SCU_I2C1,
  .refs = 0,
};
#endif
#ifdef CONFIG_CXD56_I2C2
static struct cxd56_i2cdev_s g_i2c2dev =
{
  .port = 2,
  .base = CXD56_I2CM_BASE,
  .irqid = CXD56_IRQ_I2CM,
  .refs = 0,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t i2c_reg_read(struct cxd56_i2cdev_s *priv,
                                    uint32_t offset);
static inline void i2c_reg_write(struct cxd56_i2cdev_s *priv, uint32_t offset,
                                 uint32_t val);
static inline void i2c_reg_rmw(struct cxd56_i2cdev_s *dev, uint32_t offset,
                               uint32_t val, uint32_t mask);

static int cxd56_i2c_disable(struct cxd56_i2cdev_s *priv);
static void cxd56_i2c_enable(struct cxd56_i2cdev_s *priv);

static int  cxd56_i2c_interrupt(int irq, FAR void *context, FAR void *arg);
static void cxd56_i2c_timeout(int argc, uint32_t arg, ...);
static void cxd56_i2c_setfrequency(struct cxd56_i2cdev_s *priv,
                                   uint32_t frequency);
static int  cxd56_i2c_transfer(FAR struct i2c_master_s *dev,
                               FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int cxd56_i2c_reset(FAR struct i2c_master_s * dev);
#endif
#if defined(CONFIG_CXD56_I2C0_SCUSEQ) || defined(CONFIG_CXD56_I2C1_SCUSEQ)
static int  cxd56_i2c_transfer_scu(FAR struct i2c_master_s *dev,
                                   FAR struct i2c_msg_s *msgs, int count);
#endif

/****************************************************************************
 * Name: cxd56_i2c_pincontrol
 *
 * Description:
 *   Configure the I2C pin
 *
 * Input Parameter:
 *   on - true: enable pin, false: disable pin
 *
 ****************************************************************************/

static void cxd56_i2c_pincontrol(int ch, bool on)
{
  switch (ch)
    {
#ifdef CONFIG_CXD56_I2C0
      case 0:
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_I2C0);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_I2C0_GPIO);
          }
        break;
#endif /* CONFIG_CXD56_I2C0 */

#ifdef CONFIG_CXD56_I2C1
      case 1:
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_PWMB_I2C1);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_PWMB_GPIO);
          }
        break;
#endif /* CONFIG_CXD56_I2C1 */

#ifdef CONFIG_CXD56_I2C2
      case 2:
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI0B_I2C2);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI0B_GPIO);
          }
        break;
#endif /* CONFIG_CXD56_I2C2 */

      default:
        break;
    }
}

/****************************************************************************
 * I2C device operations
 ****************************************************************************/

struct i2c_ops_s cxd56_i2c_ops =
{
  .transfer = cxd56_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset = cxd56_i2c_reset,
#endif
};

#if defined(CONFIG_CXD56_I2C0_SCUSEQ) || defined(CONFIG_CXD56_I2C1_SCUSEQ)
struct i2c_ops_s cxd56_i2c_scu_ops =
{
  .transfer = cxd56_i2c_transfer_scu,
#ifdef CONFIG_I2C_RESET
  .reset = cxd56_i2c_reset,
#endif
};
#endif

/****************************************************************************
 * Name: cxd56_i2c_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/

static void cxd56_i2c_setfrequency(struct cxd56_i2cdev_s *priv,
                                   uint32_t frequency)
{
  int32_t lcnt;
  int32_t hcnt;
  uint64_t lcnt64;
  uint64_t hcnt64;
  uint64_t speed;
  uint64_t tLow;
  uint64_t tHigh;
  uint32_t base = cxd56_get_i2c_baseclock(priv->port);
  uint32_t spklen;

  if ((priv->frequency == frequency) && (priv->baseFreq == base))
    {
      return;
    }

  priv->frequency = frequency;
  priv->baseFreq = base;

  base /= 1000;

  switch (frequency)
    {
      case 100000:
        tLow  = 4700000;
        tHigh = 4000000;
        break;

      case 400000:
        tLow  = 1300000;
        tHigh = 600000;
        break;

      case 1000000:
        tLow  = 500000;
        tHigh = 260000;
        break;

      default:
        return;
    }

  if (frequency > 100000)
    {
      if (base < 20032)
        {
          spklen = 1;
        }
      else if (base < 40064)
        {
          spklen = 2;
        }
      else
        {
          spklen = 3;
        }
    }
  else
    {
      spklen = 1;
    }

  lcnt64 = (tLow + 6500ull / 20000ull) * base;
  lcnt   = ((lcnt64 + 999999999ull) / 1000000000ull) - 1; /* ceil */
  lcnt   = lcnt < 8 ? 8 : lcnt;

  hcnt64 = (tHigh - 6500ull) * base;
  hcnt   = ((hcnt64 + 999999999ull) / 1000000000ull) - 6 - spklen; /* ceil */
  hcnt   = hcnt < 6 ? 6 : hcnt;

  speed =
    1000000000000000000ull /
    (((lcnt + 1) * 1000000000000ull + (hcnt + 6 + spklen) * 1000000000000ull) /
       base +
     20000ull / 1000ull * 1000000ull);

  if (speed > (frequency * 1000ull))
    {
      uint64_t adj;
      adj = ((1000000000000000000ull / (frequency * 1000ull)) -
             (1000000000000000000ull / speed)) *
            base;
      hcnt += (adj + 999999999999ull) / 1000000000000ull;
    }

  /* use FS register in SS and FS mode */

  i2c_reg_write(priv, CXD56_IC_FS_SCL_HCNT, hcnt);
  i2c_reg_write(priv, CXD56_IC_FS_SCL_LCNT, lcnt);
  i2c_reg_rmw(priv, CXD56_IC_CON, IC_SPEED_FS, IC_MAX_SPEED_MODE);

  i2c_reg_write(priv, CXD56_IC_FS_SPKLEN, spklen);
}

/****************************************************************************
 * Name: cxd56_i2c_timeout
 *
 * Description:
 *   Watchdog timer for timeout of I2C operation
 *
 ****************************************************************************/

static void cxd56_i2c_timeout(int argc, uint32_t arg, ...)
{
  struct cxd56_i2cdev_s *priv = (struct cxd56_i2cdev_s *)arg;
  irqstate_t flags            = enter_critical_section();

  priv->error = -ENODEV;
  sem_post(&priv->wait);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: cxd56_i2c_drainrxfifo
 *
 * Description:
 *   Receive I2C data
 *
 ****************************************************************************/

static void cxd56_i2c_drainrxfifo(struct cxd56_i2cdev_s *priv)
{
  struct i2c_msg_s *msg = priv->msgs;
  uint32_t status;
  uint32_t dat;
  ssize_t i;

  DEBUGASSERT(msg != NULL);

  status = i2c_reg_read(priv, CXD56_IC_STATUS);

  for (i = 0; i < msg->length && status & STATUS_RFNE; i++)
    {
      dat            = i2c_reg_read(priv, CXD56_IC_DATA_CMD);
      msg->buffer[i] = dat & 0xff;
      status         = i2c_reg_read(priv, CXD56_IC_STATUS);
    }
}

/****************************************************************************
 * Name: cxd56_i2c_interrupt
 *
 * Description:
 *   The I2C Interrupt Handler
 *
 ****************************************************************************/

static int cxd56_i2c_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct cxd56_i2cdev_s *priv = (FAR struct cxd56_i2cdev_s *)arg;
  uint32_t state;
  int ret;

  state = i2c_reg_read(priv, CXD56_IC_INTR_STAT);

  if (state & INTR_TX_ABRT)
    {
      i2c_reg_read(priv, CXD56_IC_CLR_TX_ABRT);
      priv->error = -ENODEV;
    }

  if (state & INTR_TX_OVER)
    {
      i2c_reg_read(priv, CXD56_IC_CLR_TX_OVER);
      priv->error = -EIO;
    }

  if (state & INTR_RX_OVER)
    {
      i2c_reg_read(priv, CXD56_IC_CLR_RX_OVER);
      priv->error = -EIO;
    }

  if (state & INTR_RX_UNDER)
    {
      i2c_reg_read(priv, CXD56_IC_CLR_RX_UNDER);
      priv->error = -EIO;
    }

  if (state & INTR_TX_EMPTY)
    {
      /* TX_EMPTY is automatically cleared by hardware
       * when the buffer level goes above the threshold.
       */

      i2c_reg_rmw(priv, CXD56_IC_INTR_MASK, 0, INTR_TX_EMPTY);
    }

  if (state & INTR_RX_FULL)
    {
      /* RX_FULL is automatically cleared by hardware
       * when the buffer level goes below the threshold.
       */

      i2c_reg_rmw(priv, CXD56_IC_INTR_MASK, 0, INTR_RX_FULL);
      cxd56_i2c_drainrxfifo(priv);
    }

  if (state & INTR_STOP_DET)
    {
      i2c_reg_read(priv, CXD56_IC_CLR_STOP_DET);
    }

  if ((priv->error) || (state & INTR_TX_EMPTY) || (state & INTR_RX_FULL))
    {
      /* Failure of wd_cancel() means that the timer expired.
       * In this case, sem_post() has already been called.
       * Therefore, call sem_post() only when wd_cancel() succeeds.
       */

      ret = wd_cancel(priv->timeout);
      if (ret == OK)
        {
          sem_post(&priv->wait);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_i2c_receive
 *
 * Description:
 *   Receive data from I2C bus.
 *   Prohibit all interrupt because the STOP condition might happen
 *   if the interrupt occurs when the writing request.
 *   Actual receiving data is in RX_FULL interrupt handler.
 *
 ****************************************************************************/

static int cxd56_i2c_receive(struct cxd56_i2cdev_s *priv, int last)
{
  struct i2c_msg_s *msg = priv->msgs;
  int i;
  irqstate_t flags;

  DEBUGASSERT(msg != NULL);

  /* update threshold value of the receive buffer */

  i2c_reg_write(priv, CXD56_IC_RX_TL, msg->length - 1);

  for (i = 0; i < msg->length - 1; i++)
    {
      i2c_reg_write(priv, CXD56_IC_DATA_CMD, CMD_READ);
    }

  flags = enter_critical_section();
  wd_start(priv->timeout, I2C_TIMEOUT, cxd56_i2c_timeout, 1, (uint32_t)priv);

  /* Set stop flag for indicate the last data */

  i2c_reg_write(priv, CXD56_IC_DATA_CMD, CMD_READ | (last ? CMD_STOP : 0));

  i2c_reg_rmw(priv, CXD56_IC_INTR_MASK, INTR_RX_FULL, INTR_RX_FULL);
  leave_critical_section(flags);

  return 0;
}

/****************************************************************************
 * Name: cxd56_i2c_send
 *
 * Description:
 *   Send data to I2C bus.
 *
 ****************************************************************************/

static int cxd56_i2c_send(struct cxd56_i2cdev_s *priv, int last)
{
  struct i2c_msg_s *msg = priv->msgs;
  ssize_t i;
  irqstate_t flags;

  DEBUGASSERT(msg != NULL);

  for (i = 0; i < msg->length - 1; i++)
    {
      while (!(i2c_reg_read(priv, CXD56_IC_STATUS) & STATUS_TFNF));

      i2c_reg_write(priv, CXD56_IC_DATA_CMD, (uint32_t)msg->buffer[i]);
    }

  while (!(i2c_reg_read(priv, CXD56_IC_STATUS) & STATUS_TFNF));

  flags = enter_critical_section();
  wd_start(priv->timeout, I2C_TIMEOUT, cxd56_i2c_timeout, 1, (uint32_t)priv);
  i2c_reg_write(priv, CXD56_IC_DATA_CMD,
                (uint32_t)msg->buffer[i] | (last ? CMD_STOP : 0));

  /* Enable TX_EMPTY interrupt for determine transfer done. */

  i2c_reg_rmw(priv, CXD56_IC_INTR_MASK, INTR_TX_EMPTY, INTR_TX_EMPTY);
  leave_critical_section(flags);

  return 0;
}

/****************************************************************************
 * Name: cxd56_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers
 *
 ****************************************************************************/

static int cxd56_i2c_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs, int count)
{
  struct cxd56_i2cdev_s *priv = (struct cxd56_i2cdev_s *)dev;
  int i;
  int ret    = 0;
  int semval = 0;
  int addr = -1;

  DEBUGASSERT(dev != NULL);

  /* Get exclusive access to the I2C bus */

  sem_wait(&priv->mutex);

  /* Check wait semaphore value. If the value is not 0, the transfer can not
   * be performed normally.
   */

  ret = sem_getvalue(&priv->wait, &semval);
  DEBUGASSERT(ret == OK && semval == 0);

  /* Disable clock gating (clock enable) */

  cxd56_i2c_clock_gate_disable(priv->port);

  for (i = 0; i < count; i++, msgs++)
    {
      /* Pass msg descriptor via device context */

      priv->msgs  = msgs;
      priv->error = OK;

      if (addr != msgs->addr)
        {
          cxd56_i2c_disable(priv);

          cxd56_i2c_setfrequency(priv, msgs->frequency);

          i2c_reg_rmw(priv, CXD56_IC_CON, IC_RESTART_EN, IC_RESTART_EN);
          i2c_reg_write(priv, CXD56_IC_TAR, msgs->addr & 0x7f);

          cxd56_i2c_enable(priv);
          addr = msgs->addr;
        }

      if (msgs->flags & I2C_M_READ)
        {
          ret = cxd56_i2c_receive(priv, i + 1 == count);
        }
      else
        {
          ret = cxd56_i2c_send(priv, i + 1 == count);
        }

      if (ret < 0)
        {
          break;
        }

      /* Wait for interrupts of each transfer */

      sem_wait(&priv->wait);

      if (priv->error != OK)
        {
          ret = priv->error;
          break;
        }

      /* Clear msg descriptor for prevent illegal access in interrupt */

      priv->msgs = NULL;
    }

  cxd56_i2c_disable(priv);

  /* Enable clock gating (clock disable) */

  cxd56_i2c_clock_gate_enable(priv->port);

  sem_post(&priv->mutex);
  return ret;
}

/************************************************************************************
 * Name: cxd56_i2c_reset
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to break loose stuck I2C devices.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_RESET
static int cxd56_i2c_reset(FAR struct i2c_master_s *dev)
{
  return OK;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Name: cxd56_i2c_transfer_scu
 *
 * Description:
 *   Perform a sequence of I2C transfers with scu oneshot sequencer.
 *
 ****************************************************************************/

#if defined(CONFIG_CXD56_I2C0_SCUSEQ) || defined(CONFIG_CXD56_I2C1_SCUSEQ)
static int cxd56_i2c_transfer_scu(FAR struct i2c_master_s *dev,
                                  FAR struct i2c_msg_s *msgs, int count)
{
  struct cxd56_i2cdev_s *priv = (struct cxd56_i2cdev_s *)dev;
  int i, j;
  int ret = 0;
  uint16_t inst[12];
  int instn            = 0;
  ssize_t len          = 0;
  ssize_t readlen      = 0;
  ssize_t totalreadlen = 0;
  uint8_t *buf         = NULL;
  uint8_t *readbuf     = NULL;
  uint8_t addr       = msgs->addr;

  DEBUGASSERT(dev != NULL);

  /* Get exclusive access to the I2C bus */

  sem_wait(&priv->mutex);

  if(priv->frequency != msgs->frequency)
    {
      cxd56_i2c_clock_gate_disable(priv->port);
      cxd56_i2c_disable(priv);
      cxd56_i2c_setfrequency(priv, msgs->frequency);
      i2c_reg_rmw(priv, CXD56_IC_CON, IC_RESTART_EN, IC_RESTART_EN);
      i2c_reg_write(priv, CXD56_IC_TAR, msgs->addr & 0x7f);
      cxd56_i2c_enable(priv);
      cxd56_i2c_clock_gate_enable(priv->port);

      priv->frequency = msgs->frequency;
    }
  for (i = 0; i < count && instn < 12; i++, msgs++)
    {
      len = msgs->length;
      buf = msgs->buffer;

      /* If the slave address is changed, I2C transfer starts with
       * settings until changed.
       */

      if (addr != msgs->addr)
        {
          break;
        }

      /* If the read requests is duplicated, only the first request is
       * accepted.
       */

      if ((msgs->flags & I2C_M_READ) && (!buf || readbuf))
        {
          break;
        }

      /* Construct SCU oneshot instructions */

      while (len && instn < 12)
        {
          if (msgs->flags & I2C_M_READ)
            {
              readlen       = (len > 8) ? 8 : len;
              inst[instn++] = SCU_INST_RECV(readlen);
              readbuf       = buf;
              totalreadlen += readlen;
              len -= readlen;
            }
          else
            {
              for (j = 0; j < len && instn < 12; j++)
                {
                  inst[instn++] = SCU_INST_SEND(*buf++);
                }
              len = len - j;
            }
        }
    }

  if (instn > 0)
    {
      inst[instn - 1] |= SCU_INST_LAST;

      if (readbuf)
        {
          ret = scu_i2ctransfer(priv->port, addr, inst, instn, readbuf,
                                totalreadlen);
        }
      else
        {
          ret = scu_i2ctransfer(priv->port, addr, inst, instn, NULL, 0);
        }
    }

  sem_post(&priv->mutex);

  return ret;
}
#endif

static inline uint32_t i2c_reg_read(struct cxd56_i2cdev_s *priv,
                                    uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static inline void i2c_reg_write(struct cxd56_i2cdev_s *priv, uint32_t offset,
                                 uint32_t val)
{
  putreg32(val, priv->base + offset);
}

static inline void i2c_reg_rmw(struct cxd56_i2cdev_s *priv, uint32_t offset,
                               uint32_t val, uint32_t mask)
{
  uint32_t regval;
  regval = getreg32(priv->base + offset);
  putreg32((regval & ~mask) | val, priv->base + offset);
}

static int cxd56_i2c_disable(struct cxd56_i2cdev_s *priv)
{
  int retry = 25000;
  uint32_t stat;

  /* disable all interrupt */

  i2c_reg_write(priv, CXD56_IC_INTR_MASK, 0x0);

  /* clear all interrupt status */

  i2c_reg_read(priv, CXD56_IC_CLR_INTR);
  i2c_reg_write(priv, CXD56_IC_ENABLE, 0);

  do
    {
      stat = i2c_reg_read(priv, CXD56_IC_ENABLE_STATUS);
    }
  while (--retry && (stat & ESTATUS_IC_EN));

  if (!retry)
    {
      i2cerr("i2c wait timeout.\n");
      return -EBUSY;
    }

  /* clear all interrupt status again */

  i2c_reg_read(priv, CXD56_IC_CLR_INTR);

  return 0;
}

static void cxd56_i2c_enable(struct cxd56_i2cdev_s *priv)
{
  i2c_reg_write(priv, CXD56_IC_INTR_MASK, I2C_INTR_ENABLE);
  i2c_reg_write(priv, CXD56_IC_ENABLE, 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device
 *
 ****************************************************************************/

struct i2c_master_s *cxd56_i2cbus_initialize(int port)
{
  struct cxd56_i2cdev_s *priv;

  irqstate_t flags;

  flags = enter_critical_section();

#ifdef CONFIG_CXD56_I2C0
  if (port == 0)
    {
      priv        = &g_i2c0dev;
#  ifndef CONFIG_CXD56_I2C0_SCUSEQ
      priv->dev.ops = &cxd56_i2c_ops;
#  else
      priv->dev.ops = &cxd56_i2c_scu_ops;
#  endif
    }
  else
#endif
#ifdef CONFIG_CXD56_I2C1
  if (port == 1)
    {
      priv        = &g_i2c1dev;
#  ifndef CONFIG_CXD56_I2C1_SCUSEQ
      priv->dev.ops = &cxd56_i2c_ops;
#  else
      priv->dev.ops = &cxd56_i2c_scu_ops;
#  endif
    }
  else
#endif
#ifdef CONFIG_CXD56_I2C2
  if (port == 2)
    {
      priv          = &g_i2c2dev;
      priv->dev.ops = &cxd56_i2c_ops;
    }
  else
#endif
    {
      leave_critical_section(flags);
      i2cerr("I2C Only support 0,1,2\n");
      return NULL;
    }

  priv->refs++;

  /* Test if already initialized or not */

  if (1 < priv->refs)
    {
      leave_critical_section(flags);
      return &priv->dev;
    }

  priv->port      = port;
  priv->frequency = 0;

  cxd56_i2c_clock_enable(priv->port);
  priv->baseFreq = cxd56_get_i2c_baseclock(priv->port);

  cxd56_i2c_disable(priv);

  i2c_reg_write(priv, CXD56_IC_INTR_MASK, 0x00);
  i2c_reg_read(priv, CXD56_IC_CLR_INTR);

  /* set threshold level of the Rx/Tx FIFO */

  i2c_reg_write(priv, CXD56_IC_RX_TL, 0xff);
  i2c_reg_write(priv, CXD56_IC_TX_TL, 0);

  /* set hold time for margin */

  i2c_reg_write(priv, CXD56_IC_SDA_HOLD, 1);

  i2c_reg_write(priv, CXD56_IC_CON,
                (IC_SLAVE_DISABLE | IC_MASTER_MODE | IC_TX_EMPTY_CTRL));
  cxd56_i2c_setfrequency(priv, I2C_DEFAULT_FREQUENCY);

  leave_critical_section(flags);

  /* Configure pin */

  cxd56_i2c_pincontrol(port, true);

  sem_init(&priv->mutex, 0, 1);
  sem_init(&priv->wait, 0, 0);

  priv->timeout = wd_create();

  /* Attach Interrupt Handler */

  irq_attach(priv->irqid, cxd56_i2c_interrupt, priv);

  /* Enable Interrupt Handler */

  up_enable_irq(priv->irqid);

  /* Enable Interrupt in SCU */

  if (port == 0 || port == 1)
    {
      putreg32(getreg32(CXD56_SCU_BASE + 0x400) | (1u << (port + 1)),
               CXD56_SCU_BASE + 0x400);
    }

  /* Enable clock gating (clock disable) */

  cxd56_i2c_clock_gate_enable(port);

  return &priv->dev;
}

/****************************************************************************
 * Name: cxd56_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 ****************************************************************************/

int cxd56_i2cbus_uninitialize(FAR struct i2c_master_s *dev)
{
  struct cxd56_i2cdev_s *priv = (struct cxd56_i2cdev_s *)dev;

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  if (--priv->refs)
    {
      return OK;
    }

  /* Configure pin */

  cxd56_i2c_pincontrol(priv->port, false);

  /* Disable clock gating (clock enable) */

  cxd56_i2c_clock_gate_disable(priv->port);

  cxd56_i2c_disable(priv);
  cxd56_i2c_clock_disable(priv->port);

  up_disable_irq(priv->irqid);
  irq_detach(priv->irqid);

  wd_delete(priv->timeout);
  priv->timeout = NULL;
  sem_destroy(&priv->mutex);
  sem_destroy(&priv->wait);

  return OK;
}

#endif /* CONFIG_CXD56_I2C */
