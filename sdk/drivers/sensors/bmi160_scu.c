/****************************************************************************
 * drivers/sensors/bmi160.c
 * Character driver for the Bosch BMI160 Barometer Sensor
 *
 *   Copyright (C) 2016 Sony Corporation. All rights reserved.
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

#include <sdk/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/bmi160.h>
#include <arch/chip/cxd56_scu.h>

#if defined(CONFIG_BMI160) && defined(CONFIG_CXD56_SCU)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#ifdef CONFIG_CXD56_DECI_GYRO
#  define GYRO_SEQ_TYPE SEQ_TYPE_DECI
#else
#  define GYRO_SEQ_TYPE SEQ_TYPE_NORMAL
#endif

#ifdef CONFIG_CXD56_DECI_ACCEL
#  define ACCEL_SEQ_TYPE SEQ_TYPE_DECI
#else
#  define ACCEL_SEQ_TYPE SEQ_TYPE_NORMAL
#endif

#define DEVID               0xd1

/* BMI160 have accel and gyro, XYZ axis respectively in 16 bits.
 */

#define BMI160_BYTESPERSAMPLE 6
#define BMI160_ELEMENTSIZE    2

/* Use reading sensor data via oneshot, for debug use only */

/* #define USE_ONESHOTREAD */

/* BMI160 Registers *************************************************************************/
/* Register Addresses */

#define BMI160_CHIP_ID          (0x00) /* Chip ID */
#define BMI160_ERROR            (0x02) /* Error register */
#define BMI160_PMU_STAT         (0x03) /* Current power mode */
#define BMI160_DATA_0           (0x04) /* MAG X  7:0 (LSB) */
#define BMI160_DATA_1           (0x05) /* MAG X 15:8 (MSB) */
#define BMI160_DATA_2           (0x06) /* MAG Y  7:0 (LSB) */
#define BMI160_DATA_3           (0x07) /* MAG Y 15:8 (MSB) */
#define BMI160_DATA_4           (0x08) /* MAG Z  7:0 (LSB) */
#define BMI160_DATA_5           (0x09) /* MAG Z 15:8 (MSB) */
#define BMI160_DATA_6           (0x0A) /* RHALL  7:0 (LSB) */
#define BMI160_DATA_7           (0x0B) /* RHALL 15:8 (MSB) */
#define BMI160_DATA_8           (0x0C) /* GYR X  7:0 (LSB) */
#define BMI160_DATA_9           (0x0D) /* GYR X 15:8 (MSB) */
#define BMI160_DATA_10          (0x0E) /* GYR Y  7:0 (LSB) */
#define BMI160_DATA_11          (0x0F) /* GYR Y 15:8 (MSB) */
#define BMI160_DATA_12          (0x10) /* GYR Z  7:0 (LSB) */
#define BMI160_DATA_13          (0x11) /* GYR Z 15:8 (MSB) */
#define BMI160_DATA_14          (0x12) /* ACC X  7:0 (LSB) */
#define BMI160_DATA_15          (0x13) /* ACC X 15:8 (MSB) */
#define BMI160_DATA_16          (0x14) /* ACC Y  7:0 (LSB) */
#define BMI160_DATA_17          (0x15) /* ACC Y 15:8 (MSB) */
#define BMI160_DATA_18          (0x16) /* ACC Z  7:0 (LSB) */
#define BMI160_DATA_19          (0x17) /* ACC Z 15:8 (MSB) */
#define BMI160_SENSORTIME_0     (0x18) /* Sensor time 0 */
#define BMI160_SENSORTIME_1     (0x19) /* Sensor time 1 */
#define BMI160_SENSORTIME_2     (0x1A) /* Sensor time 2 */
#define BMI160_STAT             (0x1B) /* Status register */
#define BMI160_INTR_STAT_0      (0x1C) /* Interrupt status */
#define BMI160_INTR_STAT_1      (0x1D)
#define BMI160_INTR_STAT_2      (0x1E)
#define BMI160_INTR_STAT_3      (0x1F)
#define BMI160_TEMPERATURE_0    (0x20) /* Temperature */
#define BMI160_TEMPERATURE_1    (0x21)
#define BMI160_FIFO_LENGTH_0    (0x22) /* FIFO length */
#define BMI160_FIFO_LENGTH_1    (0x23)
#define BMI160_FIFO_DATA        (0x24)
#define BMI160_ACCEL_CONFIG     (0x40) /* ACCEL config for ODR, bandwidth and undersampling */
#define BMI160_ACCEL_RANGE      (0x41) /* ACCEL range */
#define BMI160_GYRO_CONFIG      (0x42) /* GYRO config for ODR and bandwidth */
#define BMI160_GYRO_RANGE       (0x43) /* GYRO range */
#define BMI160_MAG_CONFIG       (0x44) /* MAG config for ODR */
#define BMI160_FIFO_DOWN        (0x45) /* GYRO and ACCEL downsampling rates for FIFO */
#define BMI160_FIFO_CONFIG_0    (0x46) /* FIFO config */
#define BMI160_FIFO_CONFIG_1    (0x47)
#define BMI160_MAG_IF_0         (0x4B) /* MAG interface */
#define BMI160_MAG_IF_1         (0x4C)
#define BMI160_MAG_IF_2         (0x4D)
#define BMI160_MAG_IF_3         (0x4E)
#define BMI160_MAG_IF_4         (0x4F)
#define BMI160_INTR_ENABLE_0    (0x50) /* Interrupt enable */
#define BMI160_INTR_ENABLE_1    (0x51)
#define BMI160_INTR_ENABLE_2    (0x52)
#define BMI160_INTR_OUT_CTRL    (0x53)
#define BMI160_INTR_LATCH       (0x54) /* Latch duration */
#define BMI160_INTR_MAP_0       (0x55) /* Map interrupt */
#define BMI160_INTR_MAP_1       (0x56)
#define BMI160_INTR_MAP_2       (0x57)
#define BMI160_INTR_DATA_0      (0x58) /* Data source */
#define BMI160_INTR_DATA_1      (0x59)
#define BMI160_INTR_LOWHIGH_0   (0x5A) /* Threshold interrupt */
#define BMI160_INTR_LOWHIGH_1   (0x5B)
#define BMI160_INTR_LOWHIGH_2   (0x5C)
#define BMI160_INTR_LOWHIGH_3   (0x5D)
#define BMI160_INTR_LOWHIGH_4   (0x5E)
#define BMI160_INTR_MOTION_0    (0x5F)
#define BMI160_INTR_MOTION_1    (0x60)
#define BMI160_INTR_MOTION_2    (0x61)
#define BMI160_INTR_MOTION_3    (0x62)
#define BMI160_INTR_TAP_0       (0x63)
#define BMI160_INTR_TAP_1       (0x64)
#define BMI160_INTR_ORIENT_0    (0x65)
#define BMI160_INTR_ORIENT_1    (0x66)
#define BMI160_INTR_FLAT_0      (0x67)
#define BMI160_INTR_FLAT_1      (0x68)
#define BMI160_FOC_CONFIG       (0x69) /* Fast offset configuration */
#define BMI160_CONFIG           (0x6A) /* Miscellaneous configuration */
#define BMI160_IF_CONFIG        (0x6B) /* Serial interface configuration */
#define BMI160_PMU_TRIGGER      (0x6C) /* GYRO power mode trigger */
#define BMI160_SELF_TEST        (0x6D) /* Self test */
#define BMI160_NV_CONFIG        (0x70) /* SPI/I2C selection */
#define BMI160_OFFSET_0         (0x71) /* ACCEL and GYRO offset */
#define BMI160_OFFSET_1         (0x72)
#define BMI160_OFFSET_2         (0x73)
#define BMI160_OFFSET_3         (0x74)
#define BMI160_OFFSET_4         (0x75)
#define BMI160_OFFSET_5         (0x76)
#define BMI160_OFFSET_6         (0x77)
#define BMI160_STEP_COUNT_0     (0x78) /* Step counter interrupt */
#define BMI160_STEP_COUNT_1     (0x79)
#define BMI160_STEP_CONFIG_0    (0x7A) /* Step counter configuration */
#define BMI160_STEP_CONFIG_1    (0x7B)
#define BMI160_CMD              (0x7e) /* Command register */

/* Register 0x40 - ACCEL_CONFIG accel bandwidth */

#define ACCEL_OSR4_AVG1   (0 << 4)
#define ACCEL_OSR2_AVG2   (1 << 4)
#define ACCEL_NORMAL_AVG4 (2 << 4)
#define ACCEL_CIC_AVG8    (3 << 4)
#define ACCEL_RES_AVG2    (4 << 4)
#define ACCEL_RES_AVG4    (5 << 4)
#define ACCEL_RES_AVG8    (6 << 4)
#define ACCEL_RES_AVG16   (7 << 4)
#define ACCEL_RES_AVG32   (8 << 4)
#define ACCEL_RES_AVG64   (9 << 4)
#define ACCEL_RES_AVG128  (10 << 4)

/* Register 0x42 - GYRO_CONFIG accel bandwidth */

#define GYRO_OSR4_MODE   (0x00 << 4)
#define GYRO_OSR2_MODE   (0x01 << 4)
#define GYRO_NORMAL_MODE (0x02 << 4)
#define GYRO_CIC_MODE    (0x03 << 4)

/* Register 0x7e - CMD */

#define	ACCEL_PM_SUSPEND      (0X10)
#define ACCEL_PM_NORMAL       (0x11)
#define	ACCEL_PM_LOWPOWER     (0X12)
#define GYRO_PM_SUSPEND       (0x14)
#define GYRO_PM_NORMAL        (0x15)
#define GYRO_PM_FASTSTARTUP   (0x17)
#define MAG_PM_SUSPEND        (0x18)
#define MAG_PM_NORMAL         (0x19)
#define MAG_PM_LOWPOWER       (0x1A)

#ifndef itemsof
#  define itemsof(array) (sizeof(array)/sizeof(array[0]))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Save BMI160 power status */

uint32_t g_pmu_stat;

struct bmi160_dev_s
{
  FAR struct spi_dev_s *spi;
  FAR struct seq_s *seq;
  int fifoid;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t bmi160_getreg8(uint8_t regaddr);
static void bmi160_putreg8(uint8_t regaddr, uint8_t regval);

/* Character driver methods */

static int     bmi160_open_gyro(FAR struct file *filep);
static int     bmi160_open_accel(FAR struct file *filep);
static int     bmi160_close_gyro(FAR struct file *filep);
static int     bmi160_close_accel(FAR struct file *filep);
static ssize_t bmi160_read(FAR struct file *filep, FAR char *buffer,
                           size_t len);
static int     bmi160_ioctl(FAR struct file *filep,int cmd,unsigned long arg);

static int     bmi160_checkid(void);

static int bmi160_devregister(FAR const char *devpath, FAR struct spi_dev_s *dev,
                              int minor, const struct file_operations *fops);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_bmi160gyrofops =
{
  bmi160_open_gyro,    /* open */
  bmi160_close_gyro,   /* close */
  bmi160_read,    /* read */
  0,                   /* write */
  0,                   /* seek */
  bmi160_ioctl,   /* ioctl */
};

static const struct file_operations g_bmi160accelfops =
{
  bmi160_open_accel,    /* open */
  bmi160_close_accel,   /* close */
  bmi160_read,    /* read */
  0,                    /* write */
  0,                    /* seek */
  bmi160_ioctl,   /* ioctl */
};

/* SCU instructions for pick gyro sensing data. */

static const uint16_t g_bmi160gyroinst[] =
{
  SCU_INST_SEND(BMI160_DATA_8 | 0x80),
  SCU_INST_RECV(BMI160_BYTESPERSAMPLE) | SCU_INST_LAST,
};

/* SCU instructions for pick accel sensing data. */

static const uint16_t g_bmi160accelinst[] =
{
  SCU_INST_SEND(BMI160_DATA_14 | 0x80),
  SCU_INST_RECV(BMI160_BYTESPERSAMPLE) | SCU_INST_LAST,
};

/* Sequencer instance */

static FAR struct seq_s *g_seq_gyro = NULL;
static FAR struct seq_s *g_seq_accel = NULL;

static int g_refcnt_gyro = 0;
static int g_refcnt_accel = 0;

/****************************************************************************
 * Name: bmi160_getreg8
 *
 * Description:
 *   Read from an 8-bit BMI160 register
 *
 ****************************************************************************/

static uint8_t bmi160_getreg8(uint8_t regaddr)
{
  uint8_t regval = 0;
  uint16_t inst[2];

  /* Send register to read and get the next byte */

  inst[0] = SCU_INST_SEND(regaddr | 0x80);
  inst[1] = SCU_INST_RECV(1) | SCU_INST_LAST;

  scu_spitransfer(0, inst, 2, &regval, 1);

  return regval;
}

#ifdef USE_ONESHOTREAD
/****************************************************************************
 * Name: bmi160_getregs
 *
 * Description:
 *   Burst read from an BMI160 register
 *
 ****************************************************************************/

static uint8_t bmi160_getregs(uint8_t regaddr, void *buffer, int len)
{
  uint16_t inst[3];
  int ilen;

  /* Send register to read and get the next byte */

  inst[0] = SCU_INST_SEND(regaddr | 0x80);
  if (len > 8)
    {
      inst[1] = SCU_INST_RECV(8);
      inst[2] = SCU_INST_RECV(len - 8) | SCU_INST_LAST;
      ilen = 3;
    }
  else
    {
      inst[1] = SCU_INST_RECV(len) | SCU_INST_LAST;
      ilen = 2;
    }

  scu_spitransfer(0, inst, ilen, buffer, len);

  return OK;
}
#endif

/****************************************************************************
 * Name: bmi160_putreg8
 *
 * Description:
 *   Write a value to an 8-bit BMI160 register
 *
 ****************************************************************************/

static void bmi160_putreg8(uint8_t regaddr, uint8_t regval)
{
  uint16_t inst[2];

  /* Send register address and set the value */

  inst[0] = SCU_INST_SEND(regaddr);
  inst[1] = SCU_INST_SEND(regval) | SCU_INST_LAST;

  scu_spitransfer(0, inst, 2, NULL, 0);
}

/****************************************************************************
 * Name: bmi160_setcommand
 *
 * Description:
 *   Write a value to an 8-bit BMI160 register
 *
 ****************************************************************************/

static void bmi160_setcommand(uint8_t command)
{
  /* Write command register */

  bmi160_putreg8(BMI160_CMD, command);

  /* Interface idle time delay */

  up_mdelay(1);

  /* Save power mode status of Accel and gyro */

  g_pmu_stat = bmi160_getreg8(BMI160_PMU_STAT) & 0x3c;
}

static int bmi160_seqinit_gyro(FAR struct bmi160_dev_s *priv)
{
  DEBUGASSERT(!g_seq_gyro);

  /* Open sequencer */

  g_seq_gyro = seq_open(GYRO_SEQ_TYPE, SCU_BUS_SPI);
  if (!g_seq_gyro)
    {
      return -ENOENT;
    }
  priv->seq = g_seq_gyro;

  seq_setaddress(priv->seq, 0);

  /* Set instruction and sample data information to sequencer */

  seq_setinstruction(priv->seq, g_bmi160gyroinst, itemsof(g_bmi160gyroinst));
  seq_setsample(priv->seq, BMI160_BYTESPERSAMPLE, 0, BMI160_ELEMENTSIZE, false);

  return OK;
}

static int bmi160_seqinit_accel(FAR struct bmi160_dev_s *priv)
{
  DEBUGASSERT(!g_seq_accel);

  /* Open sequencer */

  g_seq_accel = seq_open(ACCEL_SEQ_TYPE, SCU_BUS_SPI);
  if (!g_seq_accel)
    {
      return -ENOENT;
    }
  priv->seq = g_seq_accel;

  seq_setaddress(priv->seq, 0);

  /* Set instruction and sample data information to sequencer */

  seq_setinstruction(priv->seq, g_bmi160accelinst, itemsof(g_bmi160accelinst));
  seq_setsample(priv->seq, BMI160_BYTESPERSAMPLE, 0, BMI160_ELEMENTSIZE, false);

  return OK;
}

/****************************************************************************
 * Name: bmi160_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int bmi160_open_gyro(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi160_dev_s *priv  = inode->i_private;
  int ret;

  if (g_refcnt_gyro == 0)
    {
      /* Open and set sequencer */

      ret = bmi160_seqinit_gyro(priv);
      if (ret)
        {
          return ret;
        }

      /* Change gyroscope to normal mode */

      bmi160_setcommand(GYRO_PM_NORMAL);
      up_mdelay(30);

      /* Set gyro to normal bandwidth and output data rate 100Hz
       * Hz = 100/2^(8-n)
       */

      bmi160_putreg8(BMI160_GYRO_CONFIG, GYRO_NORMAL_MODE | 8);
    }
  else
    {
      /* Set existing sequencer */

      priv->seq = g_seq_gyro;
    }

  g_refcnt_gyro++;

  return OK;
}

static int bmi160_open_accel(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi160_dev_s *priv  = inode->i_private;
  int ret;

  if (g_refcnt_accel == 0)
    {
      /* Open and set sequencer */

      ret = bmi160_seqinit_accel(priv);
      if (ret)
        {
          return ret;
        }

      /* Change accelerometer to normal mode */

      bmi160_setcommand(ACCEL_PM_NORMAL);
      up_mdelay(30);

      /* Set accel to normal bandwidth and output data rate 100Hz
       * Hz = 100/2^(8-n)
       */

      bmi160_putreg8(BMI160_ACCEL_CONFIG, ACCEL_OSR4_AVG1 | 8);
    }
  else
    {
      /* Set existing sequencer */

      priv->seq = g_seq_accel;
    }

  g_refcnt_accel++;

  return OK;
}

/****************************************************************************
 * Name: bmi160_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int bmi160_close_gyro(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi160_dev_s *priv  = inode->i_private;

  g_refcnt_gyro--;
  if (g_refcnt_gyro == 0)
    {
      DEBUGASSERT(g_seq_gyro);

      /* Change gyroscope to suspend */

      bmi160_setcommand(GYRO_PM_SUSPEND);
      up_mdelay(30);

      seq_close(g_seq_gyro);
      g_seq_gyro = NULL;
    }
  else
    {
      (void) seq_ioctl(priv->seq, priv->fifoid, SCUIOC_FREEFIFO, 0);
    }

  return OK;
}

static int bmi160_close_accel(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi160_dev_s *priv  = inode->i_private;

  g_refcnt_accel--;
  if (g_refcnt_accel == 0)
    {
      DEBUGASSERT(g_seq_accel);

      /* Change accelerometer to suspend */

      bmi160_setcommand(ACCEL_PM_SUSPEND);
      up_mdelay(30);

      /* Close sequencer */

      seq_close(g_seq_accel);
      g_seq_accel = NULL;
    }
  else
    {
      (void) seq_ioctl(priv->seq, priv->fifoid, SCUIOC_FREEFIFO, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: bmi160_read_gyro
 *
 * Description:
 *   Standard character driver read method for accel/gyro.
 *
 ****************************************************************************/

static ssize_t bmi160_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi160_dev_s *priv  = inode->i_private;

#ifdef USE_ONESHOTREAD
  bmi160_getregs(BMI160_DATA_14, buffer, 6);
  len = 6;
#else
  len = seq_read(priv->seq, priv->fifoid, buffer, len);
#endif

  return len;
}

/****************************************************************************
 * Name: bmi160_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int bmi160_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi160_dev_s *priv  = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      default:
        {
          if (_SCUIOCVALID(cmd))
            {
              /* Redirect SCU commands */

              ret = seq_ioctl(priv->seq, priv->fifoid, cmd, arg);
            }
          else
            {
              snerr("Unrecognized cmd: %d\n", cmd);
              ret = -ENOTTY;
            }
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: bmi160_checkid
 *
 * Description:
 *   Read and verify the BMI160 chip ID
 *
 ****************************************************************************/

static int bmi160_checkid(void)
{
  uint8_t devid = 0;

  /* Read device ID  */

  devid = bmi160_getreg8(BMI160_CHIP_ID);
  sninfo("devid: %04x\n", devid);

  if (devid != (uint16_t) DEVID)
    {
      /* ID is not Correct */

      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: bmi160_seqregister
 *
 * Description:
 *   Register the BMI160 character device with sequencer
 *
 * Input Parameters:
 *   devpath - The base path to the driver to register. E.g., "/dev/accel"
 *   dev     - An instance of the SPI interface to use to communicate with
 *             BMI160
 *   id      - FIFO ID
 *   fops    - File operations
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int bmi160_devregister(FAR const char *devpath, FAR struct spi_dev_s *dev,
                              int minor, const struct file_operations *fops)
{
  FAR struct bmi160_dev_s *priv;
  char path[12];
  int ret;

  priv = (FAR struct bmi160_dev_s *)kmm_malloc(sizeof(struct bmi160_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi = dev;
  priv->seq = NULL;
  priv->fifoid = minor;

  (void) snprintf(path, sizeof(path), "%s%d", devpath, minor);
  ret = register_driver(path, fops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: bmi160_init
 *
 * Description:
 *   Register the BMI160 character device as 'devpath'
 *
 * Input Parameters:
 *   dev     - An instance of the SPI interface to use to communicate with
 *             BMI160
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmi160_init(FAR struct spi_dev_s *dev)
{
  int ret;

  /* Configure SPI for the BMI160 */

  SPI_SETMODE(dev, SPIDEV_MODE3);
  SPI_SETBITS(dev, 8);
  (void)SPI_HWFEATURES(dev, 0);
  (void)SPI_SETFREQUENCY(dev, BMI160_SPI_MAXFREQUENCY);

  /* BMI160 detects communication bus is SPI by rising edge of CS. */

  bmi160_getreg8(0x7f);
  bmi160_getreg8(0x7f); /* workaround: fail to switch SPI, run twice */

  ret = bmi160_checkid();
  if (ret < 0)
    {
      snerr("Wrong Device ID!\n");
      return ret;
    }

  /* To avoid gyro wakeup it is required to write 0x00 to 0x6C*/

  bmi160_putreg8(BMI160_PMU_TRIGGER, 0);
  up_mdelay(1);

  return OK;
}

/****************************************************************************
 * Name: bmi160gyro_register
 *
 * Description:
 *   Register the BMI160 gyro sensor character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The base path to the driver to register. E.g., "/dev/gyro"
 *   dev     - An instance of the SPI interface to use to communicate with
 *             BMI160
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmi160gyro_register(FAR const char *devname, int minor,
                        FAR struct spi_dev_s *dev)
{
  int ret;

  ret = bmi160_devregister(devname, dev, minor, &g_bmi160gyrofops);
  if (ret < 0)
    {
      snerr("Gyroscope register failed. %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bmi160accel_register
 *
 * Description:
 *   Register the BMI160 accelerometer character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The base path to the driver to register. E.g., "/dev/accel"
 *   dev     - An instance of the SPI interface to use to communicate with
 *             BMI160
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmi160accel_register(FAR const char *devname, int minor,
                         FAR struct spi_dev_s *dev)
{
  int ret;

  ret = bmi160_devregister(devname, dev, minor, &g_bmi160accelfops);
  if (ret < 0)
    {
      snerr("Accelerometer register failed. %d\n", ret);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_BMI160 */
