/****************************************************************************
 * drivers/video/isx012.c
 *
 *   Copyright (C) 2017 Sony Corporation
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
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/isx012.h>
#include <arch/board/board.h>
#include <arch/chip/cisif.h>

#include "isx012_reg.h"

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* The following macro is enabled because it is to make stable startup. (other case) */
/* #define ISX012_NOT_USE_NSTBY */

/* The following macro is disabled because it is to see detailed control. */
/* #define ISX012_CHECK_IN_DETAIL */

/* The following macro is disabled because the AF setting timing is delayed. */
/* #define ISX012_FIRST_SET_AF */

/* The following macro is disabled because the AF is not used. */
/* #define ISX012_AF_EN */

/* The following macro is disabled because to no wait after mode transfer. */
/* #define ISX012_FRAME_SKIP_EN */

#define OUT_YUV_VSIZE_MIN         (64)
#define OUT_YUV_HSIZE_MIN         (96)
#define OUT_JPG_VSIZE_MIN         (64)
#define OUT_JPG_HSIZE_MIN         (96)
#define OUT_YUV_15FPS_VSIZE_MAX  (360)
#define OUT_YUV_15FPS_HSIZE_MAX  (480)
#define OUT_YUV_30FPS_VSIZE_MAX  (360)
#define OUT_YUV_30FPS_HSIZE_MAX  (480)
#define OUT_YUV_60FPS_VSIZE_MAX  (360)
#define OUT_YUV_60FPS_HSIZE_MAX  (480)
#define OUT_YUV_120FPS_VSIZE_MAX (240)
#define OUT_YUV_120FPS_HSIZE_MAX (320)
#define OUT_JPG_15FPS_VSIZE_MAX (1944)
#define OUT_JPG_15FPS_HSIZE_MAX (2592)
#define OUT_JPG_30FPS_VSIZE_MAX  (960)
#define OUT_JPG_30FPS_HSIZE_MAX (1280)
#define OUT_JPG_60FPS_VSIZE_MAX  (480)
#define OUT_JPG_60FPS_HSIZE_MAX  (640)
#define OUT_JPG_120FPS_VSIZE_MAX (240)
#define OUT_JPG_120FPS_HSIZE_MAX (320)

#define OUT_YUVINT_30FPS_VSIZE_MAX  (240)
#define OUT_YUVINT_30FPS_HSIZE_MAX  (400)
#define OUT_JPGINT_30FPS_VSIZE_MAX  (960)
#define OUT_JPGINT_30FPS_HSIZE_MAX (1280)
#define OUT_JPGINT_15FPS_VSIZE_MAX (1224)
#define OUT_JPGINT_15FPS_HSIZE_MAX (1632)

#define AF_EXT_TIMEOUT              (500) /* ms */
#define AF_EXT_WAIT_TIME              (5) /* ms */
#define AF_EXT_DELAY_TIME             (0) /* TODO:ms */
#define VINT_TIMEOUT                (400) /* ms */
#define VINT_WAIT_TIME                (5) /* ms */
#define VINT_DELAY_TIME               (0) /* ms */
#define CAMERA_MODE_TIMEOUT         (800) /* TODO: 2vsync is 400ms.worst:5fps*/
#define CAMERA_MODE_WAIT_TIME        (10) /* ms */
#define CAMERA_MODE_DELAY_TIME        (0) /* ms */
#define DEVICE_STATE_TIMEOUT        (100) /* ms */
#define DEVICE_STATE_WAIT_TIME        (1) /* ms */
#define DEVICE_STATE_DELAY_TIME       (2) /* ms */

#define I2CFREQ_STANDARD         (100000) /* Standard mode : 100kHz */
#define I2CFREQ_FAST             (400000) /* Fast mode     : 400kHz */

/* Debug option */
#ifdef CONFIG_DEBUG_IMAGER_ERROR
#  define imagererr(format, ...)     _err(format, ##__VA_ARGS__)
#else
/* #  define imagererr(x...) */
#  define imagererr                  printf
#endif

#ifdef CONFIG_DEBUG_IMAGER_WARN
#  define imagerwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define imagerwarn(x...)
#endif

#ifdef CONFIG_DEBUG_IMAGER_INFO
#  define imagerinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#ifndef ISX012_AF_EN
#  define imagerinfo(x...)
#else
#  define imagerinfo                 printf
#endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct isx012_maxsize_s {
  uint16_t yuv_vsize_max;
  uint16_t yuv_hsize_max;
  uint16_t jpg_vsize_max;
  uint16_t jpg_hsize_max;
  uint8_t  fps_type;
  uint8_t  sensor_mode;
};

typedef struct isx012_maxsize_s isx012_maxsize_t;

struct isx012_setparam_s {
  uint8_t  fps;
  uint8_t  format;
  uint8_t  sensor_mode;
  uint16_t hsize;
  uint16_t vsize;
  uint16_t int_hsize;
  uint16_t int_vsize;
};

typedef struct isx012_setparam_s isx012_setparam_t;

#define ARRAY_NENTRIES(a) (sizeof(a)/sizeof(isx012_reg_t))

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* register operations */
static uint16_t isx012_getreg(isx012_dev_t *priv,
                              uint16_t regaddr, uint16_t regsize);
static int     isx012_putreg(isx012_dev_t *priv,
                          uint16_t regaddr, uint16_t regval, uint16_t regsize);
static int     isx012_putreglist(isx012_dev_t *priv,
                             FAR const isx012_reg_t *reglist, size_t nentries);
#ifdef ISX012_CHECK_IN_DETAIL
static int     isx012_putregs(isx012_dev_t *priv,
                          uint16_t regaddr, uint8_t *regvals, uint8_t regsize);
static int     isx012_chipid(FAR struct i2c_master_s *i2c);
#endif

static int isx012_chk_int_state(isx012_dev_t *priv,
                                uint8_t  sts,       uint32_t delay_time,
                                uint32_t wait_time, uint32_t timeout);
#ifdef ISX012_AF_EN
static int isx012_chk_reg_bit(isx012_dev_t *priv,
                              uint16_t reg,       uint8_t mask_bit,
                              uint32_t delay_time,
                              uint32_t wait_time, uint32_t timeout);
#endif
static int isx012_chk_param(isx012_param_t *param, isx012_setparam_t *set_param);
static int isx012_set_mode_param(isx012_dev_t *priv,
                                 isx012_setparam_t set_moni_param,
                                 isx012_setparam_t set_cap_param);
static int isx012_change_camera_mode(isx012_dev_t *priv, isx012_mode_t mode);
static int isx012_change_device_state(isx012_dev_t *priv, isx012_state_t state);
static int isx012_write_reg(isx012_dev_t *priv, isx012_reg_t *reg);
static int isx012_read_reg(isx012_dev_t *priv, isx012_reg_t *reg);
static int isx012_change_mode_param(isx012_dev_t *priv, FAR isx012_t *imager);
static int isx012_set_moni_refresh(isx012_dev_t *priv, unsigned long arg);

static int isx012_open(FAR struct file *filep);
static int isx012_close(FAR struct file *filep);
static int isx012_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static isx012_state_t g_state;
static isx012_mode_t  g_mode;
static uint32_t g_i2c_freq = I2CFREQ_STANDARD;
static int g_af_loadsts = 0;

static isx012_maxsize_t g_rate_to_maxsize[RATE_ISX012_MAX] = {
 {OUT_YUV_120FPS_VSIZE_MAX, OUT_YUV_120FPS_HSIZE_MAX, OUT_JPG_120FPS_VSIZE_MAX,
  OUT_JPG_120FPS_HSIZE_MAX, REGVAL_FPSTYPE_120FPS, REGVAL_SENSMODE_1_8},/*120*/

 {OUT_YUV_60FPS_VSIZE_MAX,  OUT_YUV_60FPS_HSIZE_MAX,  OUT_JPG_60FPS_VSIZE_MAX,
  OUT_JPG_60FPS_HSIZE_MAX, REGVAL_FPSTYPE_60FPS, REGVAL_SENSMODE_1_4},   /*60*/

 {OUT_YUV_30FPS_VSIZE_MAX,  OUT_YUV_30FPS_HSIZE_MAX,  OUT_JPG_30FPS_VSIZE_MAX,
  OUT_JPG_30FPS_HSIZE_MAX, REGVAL_FPSTYPE_30FPS, REGVAL_SENSMODE_1_2},   /*30*/

 {OUT_YUV_15FPS_VSIZE_MAX,  OUT_YUV_15FPS_HSIZE_MAX,  OUT_JPG_15FPS_VSIZE_MAX,
  OUT_JPG_15FPS_HSIZE_MAX, REGVAL_FPSTYPE_15FPS, REGVAL_SENSMODE_ALLPIX},/*15*/

 {OUT_YUV_15FPS_VSIZE_MAX,  OUT_YUV_15FPS_HSIZE_MAX,  OUT_JPG_15FPS_VSIZE_MAX,
  OUT_JPG_15FPS_HSIZE_MAX, REGVAL_FPSTYPE_10FPS, REGVAL_SENSMODE_ALLPIX},/*10*/

 {OUT_YUV_15FPS_VSIZE_MAX,  OUT_YUV_15FPS_HSIZE_MAX,  OUT_JPG_15FPS_VSIZE_MAX,
  OUT_JPG_15FPS_HSIZE_MAX, REGVAL_FPSTYPE_7_5FPS, REGVAL_SENSMODE_ALLPIX},/*7*/

 {OUT_YUV_15FPS_VSIZE_MAX,  OUT_YUV_15FPS_HSIZE_MAX,  OUT_JPG_15FPS_VSIZE_MAX,
  OUT_JPG_15FPS_HSIZE_MAX, REGVAL_FPSTYPE_6FPS, REGVAL_SENSMODE_ALLPIX}, /* 6*/

 {OUT_YUV_15FPS_VSIZE_MAX,  OUT_YUV_15FPS_HSIZE_MAX,  OUT_JPG_15FPS_VSIZE_MAX,
  OUT_JPG_15FPS_HSIZE_MAX, REGVAL_FPSTYPE_5FPS, REGVAL_SENSMODE_ALLPIX}, /* 5*/
};

static isx012_maxsize_t g_rate_to_maxsize_interleave[RATE_ISX012_MAX] = {
 {                         0,                          0,
                           0,                          0,
                           0,                   0},    /* Rate120Fps*/
 {                         0,                          0,
                           0,                          0,
                           0,                   0},    /* Rate60Fps */
 {OUT_YUVINT_30FPS_VSIZE_MAX, OUT_YUVINT_30FPS_HSIZE_MAX,
  OUT_JPGINT_30FPS_VSIZE_MAX, OUT_JPGINT_30FPS_HSIZE_MAX,
  REGVAL_FPSTYPE_30FPS,       REGVAL_SENSMODE_1_2},    /* Rate30Fps */
 {OUT_YUVINT_30FPS_VSIZE_MAX, OUT_YUVINT_30FPS_HSIZE_MAX,
  OUT_JPGINT_15FPS_VSIZE_MAX, OUT_JPGINT_15FPS_HSIZE_MAX,
  REGVAL_FPSTYPE_15FPS,       REGVAL_SENSMODE_ALLPIX}, /* Rate15Fps */
 {OUT_YUVINT_30FPS_VSIZE_MAX, OUT_YUVINT_30FPS_HSIZE_MAX,
  OUT_JPGINT_15FPS_VSIZE_MAX, OUT_JPGINT_15FPS_HSIZE_MAX,
  REGVAL_FPSTYPE_10FPS,       REGVAL_SENSMODE_ALLPIX}, /* Rate10Fps */
 {OUT_YUVINT_30FPS_VSIZE_MAX, OUT_YUVINT_30FPS_HSIZE_MAX,
  OUT_JPGINT_15FPS_VSIZE_MAX, OUT_JPGINT_15FPS_HSIZE_MAX,
  REGVAL_FPSTYPE_7_5FPS,      REGVAL_SENSMODE_ALLPIX},/* Rate7_5Fps */
 {OUT_YUVINT_30FPS_VSIZE_MAX, OUT_YUVINT_30FPS_HSIZE_MAX,
  OUT_JPGINT_15FPS_VSIZE_MAX, OUT_JPGINT_15FPS_HSIZE_MAX,
  REGVAL_FPSTYPE_6FPS,        REGVAL_SENSMODE_ALLPIX},  /* Rate6Fps */
 {OUT_YUVINT_30FPS_VSIZE_MAX, OUT_YUVINT_30FPS_HSIZE_MAX,
  OUT_JPGINT_15FPS_VSIZE_MAX, OUT_JPGINT_15FPS_HSIZE_MAX,
  REGVAL_FPSTYPE_5FPS,        REGVAL_SENSMODE_ALLPIX},  /* Rate5Fps */
};

#ifndef ISX012_NOT_USE_NSTBY
static const isx012_reg_t g_isx012_presleep[] = {
  {0x0007, 0x00, 0x01}, /* PLL_CKSEL */
  {0x0008, 0x00, 0x01}, /* SRCCK_DIV */
  {0x0006, 0x17, 0x01}, /* INCK_SET */
};
#define ISX012_PRESLEEP_NENTRIES ARRAY_NENTRIES(g_isx012_presleep)
#endif

static const isx012_reg_t g_isx012_def_init[] = {
#ifdef ISX012_NOT_USE_NSTBY
  {0x0007, 0x00,   0x01}, /* PLL_CKSEL */
  {0x0008, 0x00,   0x01}, /* SRCCK_DIV */
#endif
  {0x000C, 0xAA,   0x01}, /* DRIVABILITY */
  {0x00C2, 0x0200,    2}, /* VIFCONFIG */
  {0x00E0, 0xFF0A,    2}, /* YUVCONFIG_TN */
  {0x013A, 0x00,   0x01}, /* ILCODELEN */
  {0x00B2, 0x01,   0x01}, /* AFMODE_MONI */
  {0x00DE, 0xFF6A,    2}, /* YUVCONFIG */
  {0x6A8C, 0x10FE,    2}, /* VIF_REC601_Y */
  {0x6A8E, 0x10F0,    2}, /* VIF_REC601_C */
  {0x00AF, 0x11,   0x01}, /* HSENS_MODE_SEL */
  {0x00C4, 0x30,   0x01}, /* VIF_CLKCONFIG1 */
  {0x00C5, 0x30,   0x01}, /* VIF_CLKCONFIG2 */
  {0x00C6, 0x30,   0x01}, /* VIF_CLKCONFIG3 */
  {0x00C7, 0x30,   0x01}, /* VIF_CLKCONFIG4 */
  {0x00C8, 0x30,   0x01}, /* VIF_CLKCONFIG5 */
  {0x00C9, 0x30,   0x01}, /* VIF_CLKCONFIG6 */
  {0x00CA, 0x30,   0x01}, /* VIF_CLKCONFIG7 */
  {0x00CB, 0x30,   0x01}, /* VIF_CLKCONFIG8 */
  {0x00CC, 0x30,   0x01}, /* VIF_CLKCONFIG9 */
  {0x00CD, 0x30,   0x01}, /* VIF_CLKCONFIG10 */
  {0x00CE, 0x30,   0x01}, /* VIF_CLKCONFIG11 */
  {0x00CF, 0x30,   0x01}, /* VIF_CLKCONFIG12 */
  {0x6A12, 0x11,   0x01}, /* VIF_CLKCONFIG13 */
  {0x6A13, 0x11,   0x01}, /* VIF_CLKCONFIG14 */
  {0x6A14, 0x11,   0x01}, /* VIF_CLKCONFIG15 */
  {0x6A15, 0x11,   0x01}, /* VIF_CLKCONFIG16 */
#ifdef ISX012_NOT_USE_NSTBY
  {0x0006, 0x17,   0x01}, /* INCK_SET */
#endif
};
#define ISX012_RESET_NENTRIES ARRAY_NENTRIES(g_isx012_def_init)

static const struct file_operations g_isx012fops =
{
  isx012_open,              /* open */
  isx012_close,             /* close */
  0,                        /* read */
  0,                        /* write */
  0,                        /* seek */
  isx012_ioctl,             /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,                        /* poll */
#endif
  0                         /* unlink */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint16_t isx012_getreg(isx012_dev_t *priv,
                              uint16_t regaddr, uint16_t regsize)
{
  struct i2c_config_s config;
  volatile uint16_t regval;
  volatile uint8_t buffer[2];
  int ret;

  /* Set up the I2C configuration */
  config.frequency = g_i2c_freq; /* FREQUENCY */
  config.address   = ISX012_I2C_SLV_ADDR;
  config.addrlen   = 7;
  buffer[0] = regaddr >> 8;
  buffer[1] = regaddr & 0xff;

  /* Write the register address */
  ret = i2c_write(priv->i2c, &config, (uint8_t *)buffer, 2);
  if (ret < 0)
    {
      imagererr("i2c_write failed: %d\n", ret);
      return 0;
    }

  /* Restart and read 16bits from the register */
  ret = i2c_read(priv->i2c, &config, (uint8_t *)buffer, regsize);
  if (ret < 0)
    {
      imagererr("i2c_read failed: %d\n", ret);
      return 0;
    }

  memcpy((uint8_t *)&regval, (uint8_t *)buffer, regsize);

  return regval;
}

static int isx012_putreg(isx012_dev_t *priv,
                         uint16_t regaddr, uint16_t regval, uint16_t regsize)
{
  struct i2c_config_s config;
  volatile uint8_t buffer[4];
  int ret;

  /* Set up the I2C configuration */
  config.frequency = g_i2c_freq; /* FREQUENCY */
  config.address   = ISX012_I2C_SLV_ADDR;
  config.addrlen   = 7;

  /* Set up for the transfer */
  buffer[0] = regaddr >> 8;   /* RegAddr Hi */
  buffer[1] = regaddr & 0xff; /* RegAddr Low*/

  memcpy((uint8_t *)&buffer[2], (uint8_t *)&regval, regsize);

  /* And do it */
  ret = i2c_write(priv->i2c, &config, (uint8_t *)buffer, regsize+2);
  if (ret < 0)
    {
      imagererr("i2c_write failed: %d\n", ret);
    }

  return ret;
}

static int isx012_putreglist(isx012_dev_t *priv,
                             FAR const isx012_reg_t *reglist, size_t nentries)
{
  FAR const isx012_reg_t *entry;
  int ret = OK;

  for (entry = reglist; nentries > 0; nentries--, entry++)
    {
      ret = isx012_putreg(priv, entry->regaddr, entry->regval, entry->regsize);
      if (ret < 0)
        {
          imagererr("isx012_putreg failed: %d\n", ret);
          return ret;
        }

    }
  return ret;
}

#ifdef ISX012_CHECK_IN_DETAIL
static int isx012_putregs(isx012_dev_t *priv,
                          uint16_t regaddr, uint8_t *regvals, uint8_t regsize)
{
  struct i2c_config_s config;
  volatile uint8_t buffer[16];
  int ret;

  /* Set up the I2C configuration */
  config.frequency = g_i2c_freq; /* FREQUENCY */
  config.address   = ISX012_I2C_SLV_ADDR;
  config.addrlen   = 7;

  /* Set up for the transfer */
  buffer[0] = regaddr >> 8;   /* RegAddr Hi */
  buffer[1] = regaddr & 0xff; /* RegAddr Low*/

  memcpy((uint8_t *)&buffer[2], (uint8_t *)regvals, regsize);
  ret = i2c_write(priv->i2c, &config, (uint8_t *)buffer, regsize+2);
  if (ret < 0)
    {
      imagererr("i2c_write failed: %d\n", ret);
    }

  return ret;
}

static int isx012_chipid(isx012_dev_t *priv)
{
  uint8_t inck;

  inck = isx012_getreg(priv, 0x06, 0x01);
  if (inck != 0x06 && inck != 0x17)
    {
      imagererr("Unsupported sensor INCK=%02x\n",inck);
      return -ENODEV;
    }

  imagerinfo("INCK=%02x\n", inck);
  return OK;
}
#endif

static int isx012_chk_int_state(isx012_dev_t *priv,
                                uint8_t sts, uint32_t delay_time,
                                uint32_t wait_time, uint32_t timeout)
{
  int ret = 0;
  volatile uint8_t data;
  uint32_t time = 0;

  usleep(delay_time*1000);
  while(time < timeout)
    {
      data = isx012_getreg(priv, INTSTS0, sizeof(data));
      data = data & sts;
      if (data != 0)
        {
          ret = isx012_putreg(priv, INTCLR0, data, sizeof(data));
          return ret;
        }

      usleep(wait_time*1000);
      time += wait_time;
    }
  return ERROR;
}

#ifdef ISX012_AF_EN
static int isx012_chk_reg_bit(isx012_dev_t *priv,
                              uint16_t reg, uint8_t mask_bit,
                              uint32_t delay_time, uint32_t wait_time,
                              uint32_t timeout)
{
  volatile uint8_t data;
  uint32_t time = 0;

  usleep(delay_time*1000);
  while(time < timeout)
    {
      data = isx012_getreg(priv, reg, sizeof(data));
      imagerinfo("ISX012: AF_EXT_AFRAMDRVFIN: %x\n", data);
      data = data & mask_bit;
      if (data != 0)
        {
          return OK;
        }

      usleep(wait_time*1000);
      time += wait_time;
    }
  return -EPERM;
}
#endif /* ISX012_AF_EN */

static int isx012_chk_param(isx012_param_t *param,
                            isx012_setparam_t *set_param)
{
  if (param->rate >= RATE_ISX012_MAX) {
    return -EPERM;
  }
  set_param->int_hsize = 0;
  set_param->int_vsize = 0;

  switch (param->format)
    {
      case FORMAT_ISX012_YUV:
      case FORMAT_ISX012_RGB565:
        if (param->yuv_hsize < OUT_YUV_HSIZE_MIN ||
            param->yuv_hsize > g_rate_to_maxsize[param->rate].yuv_hsize_max ||
            param->yuv_vsize < OUT_YUV_VSIZE_MIN ||
            param->yuv_vsize > g_rate_to_maxsize[param->rate].yuv_vsize_max)
          {
            return -EPERM;
          }

        if (param->yuv_hsize % 2 != 0 || param->yuv_vsize % 2 != 0)
          {
            return -EPERM;
          }

        if (param->format == FORMAT_ISX012_YUV)
          {
            set_param->format = REGVAL_OUTFMT_YUV;
          }
        else
          {
            set_param->format = REGVAL_OUTFMT_RGB;
          }
 
        set_param->hsize = param->yuv_hsize;
        set_param->vsize = param->yuv_vsize;
        set_param->fps = g_rate_to_maxsize[param->rate].fps_type;
        set_param->sensor_mode = g_rate_to_maxsize[param->rate].sensor_mode;
        break;
      case FORMAT_ISX012_JPEG_MODE1:
        if (param->jpeg_hsize < OUT_JPG_HSIZE_MIN ||
            param->jpeg_hsize > g_rate_to_maxsize[param->rate].jpg_hsize_max ||
            param->jpeg_vsize < OUT_JPG_VSIZE_MIN ||
            param->jpeg_vsize > g_rate_to_maxsize[param->rate].jpg_vsize_max)
          {
            return -EPERM;
          }
 
        if (param->jpeg_hsize % 2 != 0 || param->jpeg_vsize % 2 != 0)
          {
            return -EPERM;
          }
 
        set_param->format = REGVAL_OUTFMT_JPEG;
        set_param->hsize = param->jpeg_hsize;
        set_param->vsize = param->jpeg_vsize;
        set_param->fps = g_rate_to_maxsize[param->rate].fps_type;
        set_param->sensor_mode = g_rate_to_maxsize[param->rate].sensor_mode;
        break;
      case FORMAT_ISX012_JPEG_MODE1_INT:
        if (param->jpeg_hsize < OUT_JPG_HSIZE_MIN ||
            param->jpeg_hsize > g_rate_to_maxsize_interleave[param->rate].jpg_hsize_max ||
            param->jpeg_vsize < OUT_JPG_VSIZE_MIN ||
            param->jpeg_vsize > g_rate_to_maxsize_interleave[param->rate].jpg_vsize_max ||
            param->yuv_hsize < OUT_YUV_HSIZE_MIN ||
            param->yuv_hsize > g_rate_to_maxsize_interleave[param->rate].yuv_hsize_max ||
            param->yuv_vsize < OUT_YUV_VSIZE_MIN ||
            param->yuv_vsize > g_rate_to_maxsize_interleave[param->rate].yuv_vsize_max)
          {
            return -EPERM;
          }

        if (param->jpeg_hsize % 2 != 0 || param->jpeg_vsize % 2 != 0 ||
            param->yuv_hsize % 2 != 0 || param->yuv_vsize % 2 != 0)
          {
            return -EPERM;
          }

        set_param->format = REGVAL_OUTFMT_INTERLEAVE;
        set_param->hsize = param->jpeg_hsize;
        set_param->vsize = param->jpeg_vsize;
        set_param->int_hsize = param->yuv_hsize;
        set_param->int_vsize = param->yuv_vsize;
        set_param->fps = g_rate_to_maxsize_interleave[param->rate].fps_type;
        set_param->sensor_mode = g_rate_to_maxsize_interleave[param->rate].sensor_mode;
        break;
      default:
        return -EPERM;
    }
  return OK;
}

static int isx012_set_mode_param(isx012_dev_t *priv,
                                 isx012_setparam_t set_moni_param,
                                 isx012_setparam_t set_cap_param)
{
  int ret = 0;
  isx012_setparam_t *mparam = &set_moni_param;
  isx012_setparam_t *cparam = &set_cap_param;

  ret = isx012_putreg(priv, FPSTYPE_MONI,  mparam->fps, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, OUTFMT_MONI,   mparam->format, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, SENSMODE_MONI, mparam->sensor_mode, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, HSIZE_MONI,    mparam->hsize, sizeof(uint16_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, VSIZE_MONI,    mparam->vsize, sizeof(uint16_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, FPSTYPE_CAP,   cparam->fps, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, OUTFMT_CAP,    cparam->format, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, SENSMODE_CAP,  cparam->sensor_mode, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, HSIZE_CAP,     cparam->hsize, sizeof(uint16_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, VSIZE_CAP,     cparam->vsize, sizeof(uint16_t));
  if (ret < 0)
    {
      return ret;
    }

  if (mparam->format == REGVAL_OUTFMT_INTERLEAVE)
    {
      ret = isx012_putreg(priv, HSIZE_TN, mparam->int_hsize, sizeof(uint16_t));
      if (ret < 0)
        {
          return ret;
        }

      ret = isx012_putreg(priv, VSIZE_TN, mparam->int_vsize, sizeof(uint16_t));
      if (ret < 0)
        {
          return ret;
        }

    }

  if (cparam->format == REGVAL_OUTFMT_INTERLEAVE)
    {
      ret = isx012_putreg(priv, HSIZE_TN, cparam->int_hsize, sizeof(uint16_t));
      if (ret < 0)
        {
          return ret;
        }

      ret = isx012_putreg(priv, VSIZE_TN, cparam->int_vsize, sizeof(uint16_t));
      if (ret < 0)
        {
          return ret;
        }

    }

  return OK;
}

static int isx012_change_cisif(isx012_dev_t *priv, cisif_param_t *param) //@@@
{
  int ret = 0;
  isx012_format_t format;
  uint16_t        hsize;
  uint16_t        vsize;
  cisif_sarea_t cis_area;
  cisif_param_t cis_param;

  memset(&cis_param, 0, sizeof(cisif_param_t));
  cis_area.strg_addr = (uint8_t *)param->sarea.strg_addr;
  cis_area.strg_size = param->sarea.strg_size;
  cis_area.capnum    = param->sarea.capnum;
  cis_area.interval  = param->sarea.interval;

  if (g_mode == MODE_ISX012_MONITORING)
    {
      format = priv->image.moni_param.format;
      if (format == FORMAT_ISX012_YUV)
        {
          hsize  = priv->image.moni_param.yuv_hsize;
          vsize  = priv->image.moni_param.yuv_vsize;

          cis_param.format = FORMAT_CISIF_YUV;
          cis_param.yuv_param.hsize = hsize;
          cis_param.yuv_param.vsize = vsize;
          cis_param.yuv_param.comp_func = param->yuv_param.comp_func;
        }
      else if (format == FORMAT_ISX012_JPEG_MODE1)
        {
          hsize  = priv->image.moni_param.jpeg_hsize;
          vsize  = priv->image.moni_param.jpeg_vsize;

          cis_param.format = FORMAT_CISIF_JPEG;
          cis_param.jpg_param.comp_func = param->jpg_param.comp_func;
        }
    }
  else if (g_mode == MODE_ISX012_CAPTURE)
    {
      format = priv->image.cap_param.format;
      if (format == FORMAT_ISX012_YUV)
        {
          hsize  = priv->image.cap_param.yuv_hsize;
          vsize  = priv->image.cap_param.yuv_vsize;

          cis_param.format = FORMAT_CISIF_YUV;
          cis_param.yuv_param.hsize = hsize;
          cis_param.yuv_param.vsize = vsize;
          cis_param.yuv_param.comp_func = param->yuv_param.comp_func;
        }
      else if (format == FORMAT_ISX012_JPEG_MODE1)
        {
          hsize  = priv->image.cap_param.jpeg_hsize;
          vsize  = priv->image.cap_param.jpeg_vsize;

          cis_param.format = FORMAT_CISIF_JPEG;
          cis_param.jpg_param.comp_func = param->jpg_param.comp_func;
        }
    }
  else
    {
      format = FORMAT_ISX012_JPEG_MODE1;
    }

  if (param->sarea.capnum > 1)
    {
      ret = cxd56_cisifcontinuouscapture(&cis_param, &cis_area);
    }
  else if (format == FORMAT_ISX012_YUV)
    {
      ret = cxd56_cisifcaptureframe(&cis_param, &cis_area, NULL);
    }
  else
    {
      ret = cxd56_cisifcaptureframe(&cis_param, NULL, &cis_area);
    }

  return ret;
}

static int isx012_change_camera_mode(isx012_dev_t *priv, isx012_mode_t mode)
{
  int ret = 0;
  uint8_t mode_data;
  uint8_t format_data;
  uint32_t vifmode;
#ifdef ISX012_FRAME_SKIP_EN
  uint8_t mask_num;
  int i;
#endif /* ISX012_FRAME_SKIP_EN */

  if (g_state != STATE_ISX012_ACTIVE)
    {
      return -EPERM;
    }

  switch (mode)
    {
      case MODE_ISX012_MONITORING:
        if (g_mode == MODE_ISX012_MONITORING)
          {
            return -EPERM;
          }

        format_data = isx012_getreg(priv, OUTFMT_MONI, 1);
        mode_data = REGVAL_MODESEL_MON;
        break;
      case MODE_ISX012_CAPTURE:
        if (g_mode == MODE_ISX012_CAPTURE)
          {
            return -EPERM;
          }

        format_data = isx012_getreg(priv, OUTFMT_CAP, 1);
        mode_data = REGVAL_MODESEL_CAP;
        break;
      case MODE_ISX012_HALFRELEASE:
        if (g_mode != MODE_ISX012_MONITORING)
          {
            return -EPERM;
          }

        format_data = isx012_getreg(priv, OUTFMT_MONI, 1);
        mode_data = REGVAL_MODESEL_HREL;
        break;
      default:
        return -EPERM;
    }

  switch (format_data) /* mode parallel */
    {
      case REGVAL_OUTFMT_YUV:
        vifmode = REGVAL_VIFMODE_YUV_PARALLEL;
        break;
      case REGVAL_OUTFMT_JPEG:
        vifmode = REGVAL_VIFMODE_JPEG_PARALLEL;
        break;
      case REGVAL_OUTFMT_INTERLEAVE:
        vifmode = REGVAL_VIFMODE_INTERLEAVE_PARALLEL;
        break;
      case REGVAL_OUTFMT_RGB:
        vifmode = REGVAL_VIFMODE_RGB_PARALLEL;
        break;
      default:
        vifmode = REGVAL_VIFMODE_YUV_PARALLEL;
        break;
    }

  ret = isx012_putreg(priv, VIFMODE, vifmode, sizeof(vifmode));
  if (ret < 0)
    {
      return ret;
    }

  isx012_putreg(priv, INTCLR0, CM_CHANGED_STS, 1);

  ret = isx012_putreg(priv, MODESEL, mode_data, sizeof(mode_data));
  if (ret < 0)
    {
      return ret;
    }

  /* Wait CM_CHANGED */
  ret = isx012_chk_int_state(priv, CM_CHANGED_STS,
                                   CAMERA_MODE_DELAY_TIME,
                                   CAMERA_MODE_WAIT_TIME,
                                   CAMERA_MODE_TIMEOUT);
  if (ret != 0)
    {
      return ret;
    }

  g_mode = mode;

#ifdef ISX012_FRAME_SKIP_EN
  if (mode != MODE_ISX012_HALFRELEASE)
    {
      isx012_putreg(priv, INTCLR0, VINT_STS, 1);
      mask_num = isx012_getreg(priv, RO_MASK_NUM, sizeof(mask_num));
      for (i=0; i<mask_num; i++)
        {
          /* Wait Next VINT */
          ret = isx012_chk_int_state(priv, VINT_STS, VINT_DELAY_TIME,
                                           VINT_WAIT_TIME, VINT_TIMEOUT);
          if (ret != 0)
            {
              return ret;
            }
        }
    }
#endif /* ISX012_FRAME_SKIP_EN */

  return OK;
}

static int isx012_change_device_state(isx012_dev_t *priv, isx012_state_t state)
{
  int ret = 0;
#ifdef ISX012_FRAME_SKIP_EN
  int i;
  uint8_t mute_cnt;
#endif /* ISX012_FRAME_SKIP_EN */

  if (g_state == STATE_ISX012_PRESLEEP || g_state == state)
    {
      return -EPERM;
    }

  switch (state)
    {
      case STATE_ISX012_SLEEP:
        isx012_putreg(priv, INTCLR0, OM_CHANGED_STS, 1);
        board_isx012_set_sleep(1);
        break;
      case STATE_ISX012_ACTIVE:
        isx012_putreg(priv, INTCLR0, OM_CHANGED_STS | CM_CHANGED_STS, 1);
        board_isx012_release_sleep();
        break;
      case STATE_ISX012_PRESLEEP:
        return -EPERM;
      default:
        return -EPERM;
    }

  /* Wait OM_CHANGED */
  ret = isx012_chk_int_state(priv, OM_CHANGED_STS,
                                   DEVICE_STATE_DELAY_TIME,
                                   DEVICE_STATE_WAIT_TIME,
                                   DEVICE_STATE_TIMEOUT);
  if (ret != 0)
    {
      return ret;
    }

  g_state = state;

  if (state == STATE_ISX012_ACTIVE)
    {
      /* Wait CM_CHANGED -> Monitoring */
      ret = isx012_chk_int_state(priv, CM_CHANGED_STS,
                                       CAMERA_MODE_DELAY_TIME,
                                       CAMERA_MODE_WAIT_TIME,
                                       CAMERA_MODE_TIMEOUT);
      if (ret != 0)
        {
          return ret;
        }

#ifdef ISX012_FRAME_SKIP_EN
      mute_cnt = isx012_getreg(priv, MUTECNT, sizeof(mute_cnt));
      isx012_putreg(priv, INTCLR0, VINT_STS, 1);
      for (i=0; i<mute_cnt; i++)
        {
          /* Wait Next VINT */
          ret = isx012_chk_int_state(priv, VINT_STS, VINT_DELAY_TIME,
                                           VINT_WAIT_TIME, VINT_TIMEOUT);
          if (ret != 0)
            {
              return ret;
            }
        }
#endif  /* ISX012_FRAME_SKIP_EN */

#ifdef ISX012_AF_EN
#ifndef ISX012_FIRST_SET_AF
      if (!g_af_loadsts)
        {
          imagerinfo("AF Driver load.\n");
          /* AF default driver load */
          ret = isx012_putreg(priv, AF_EXT, 1, 1);
          if (ret < 0)
            {
              board_isx012_set_reset();
              return ret;
            }

          ret = isx012_chk_reg_bit(priv, AF_EXT,
                                        AF_EXT_AFRAMDRVFIN,
                                        AF_EXT_DELAY_TIME,
                                        AF_EXT_WAIT_TIME,
                                        AF_EXT_TIMEOUT);
          if (ret < 0 || ret > 0)
            {
              board_isx012_set_reset();
              return ret;
            }

          g_af_loadsts = 1;
        }

      ret = isx012_putreg(priv, AF_RESTART_F, 1, 1);
      if (ret < 0)
        {
          return ret;
        }
#endif /* ISX012_FIRST_SET_AF */
#endif /* ISX012_AF_EN */
    }

  g_mode = MODE_ISX012_MONITORING;

  return OK;
}

/******************************************************************************
 * isx012_change_mode_param
 *****************************************************************************/
static int isx012_change_mode_param(isx012_dev_t *priv, FAR isx012_t *imager)
{
  int ret;
  isx012_setparam_t set_moni_param;
  isx012_setparam_t set_cap_param;

  ret = isx012_chk_param(&imager->moni_param, &set_moni_param);
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_chk_param(&imager->cap_param, &set_cap_param);
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_set_mode_param(priv, set_moni_param, set_cap_param);
  if (ret < 0)
    {
      return ret;
    }

  priv->image.moni_param.format     = imager->moni_param.format;
  priv->image.moni_param.rate       = imager->moni_param.rate;
  priv->image.cap_param.format      = imager->cap_param.format;
  priv->image.cap_param.rate        = imager->cap_param.rate;
  priv->image.moni_param.yuv_hsize  = imager->moni_param.yuv_hsize;
  priv->image.moni_param.yuv_vsize  = imager->moni_param.yuv_vsize;
  priv->image.moni_param.jpeg_hsize = imager->moni_param.jpeg_hsize;
  priv->image.moni_param.jpeg_vsize = imager->moni_param.jpeg_vsize;
  priv->image.cap_param.yuv_hsize   = imager->cap_param.yuv_hsize;
  priv->image.cap_param.yuv_vsize   = imager->cap_param.yuv_vsize;
  priv->image.cap_param.jpeg_hsize  = imager->cap_param.jpeg_hsize;
  priv->image.cap_param.jpeg_vsize  = imager->cap_param.jpeg_vsize;

  return OK;
}

/******************************************************************************
 * isx012_read_reg
 *****************************************************************************/
static int isx012_read_reg(isx012_dev_t *priv, isx012_reg_t *reg)
{
  reg->regval = isx012_getreg(priv, reg->regaddr, reg->regsize);

  return OK;
}

/******************************************************************************
 * isx012_write_reg
 *****************************************************************************/
static int isx012_write_reg(isx012_dev_t *priv, isx012_reg_t *reg)
{
  int ret;

  ret = isx012_putreg(priv, reg->regaddr, reg->regval, reg->regsize);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/******************************************************************************
 * isx012_set_moni_refresh
 *****************************************************************************/
static int isx012_set_moni_refresh(isx012_dev_t *priv, unsigned long arg)
{
  int ret = 0;
  uint8_t mask_num;
  int i;

  if (g_state != STATE_ISX012_ACTIVE)
    {
      return -EPERM;
    }

  if (g_mode != MODE_ISX012_MONITORING)
    {
      return -EPERM;
    }

  /* Set MONI_REFRESH */
  isx012_putreg(priv, INTCLR0, CM_CHANGED_STS, 1);
  ret = isx012_putreg(priv, MONI_REFRESH, 1, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait CM_CHANGED */
  ret = isx012_chk_int_state(priv, CM_CHANGED_STS,
                                   CAMERA_MODE_DELAY_TIME,
                                   CAMERA_MODE_WAIT_TIME,
                                   CAMERA_MODE_TIMEOUT);
  if (ret != 0)
    {
      return ret;
    }

  /* Invalid frame skip */
  isx012_putreg(priv, INTCLR0, VINT_STS, 1);
  mask_num = isx012_getreg(priv, RO_MASK_NUM, sizeof(mask_num));
  for (i=0; i<mask_num; i++)
    {
      /* Wait Next VINT */
      ret = isx012_chk_int_state(priv, VINT_STS, VINT_DELAY_TIME,
                                       VINT_WAIT_TIME, VINT_TIMEOUT);
      if (ret != 0)
        {
          return ret;
        }
    }

  return OK;
}

int isx012_initialize(isx012_dev_t *priv)
{
  int ret;
  isx012_setparam_t set_moni_param;
  isx012_setparam_t set_cap_param;

  ret = isx012_chk_param(&(priv->image.moni_param), &set_moni_param);
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_chk_param(&(priv->image.cap_param), &set_cap_param);
  if (ret < 0)
    {
      return ret;
    }

#ifdef ISX012_NOT_USE_NSTBY
  board_isx012_release_sleep();
  board_isx012_release_reset();
  usleep(6000);
#else
  board_isx012_release_reset();
  usleep(6000);
#endif

#ifdef ISX012_CHECK_IN_DETAIL
  /* check the chip id*/
  ret = isx012_chipid(priv);
  if (ret < 0)
    {
      imagererr("isx012_chipid failed: %d\n", ret);
      board_isx012_set_reset();
      return ret;
    }
#endif

  /* Wait OM_CHANGED Power OFF -> PreSleep */
  ret = isx012_chk_int_state(priv, OM_CHANGED_STS, DEVICE_STATE_DELAY_TIME,
                             DEVICE_STATE_WAIT_TIME, DEVICE_STATE_TIMEOUT);
  if (ret != OK)
    {
      imagererr("OM_CHANGED_STS(PreSleep) is Not occured: %d\n", ret);
      return ret;
    }

  g_state = STATE_ISX012_PRESLEEP;

#ifndef ISX012_NOT_USE_NSTBY
  /* set the isx012 clock */
  /* Write INCK_SET register ISX012 change state PreSleep -> Sleep */
  ret = isx012_putreglist(priv, g_isx012_presleep, ISX012_PRESLEEP_NENTRIES);
  if (ret != OK)
    {
      imagererr("isx012_putreglist(INCK_SET) failed: %d\n", ret);
      return ret;
    }

  /* Wait OM_CHANGED PreSleep -> Sleep */
  ret = isx012_chk_int_state(priv, OM_CHANGED_STS, DEVICE_STATE_DELAY_TIME,
                             DEVICE_STATE_WAIT_TIME, DEVICE_STATE_TIMEOUT);
  if (ret != OK)
    {
      imagererr("OM_CHANGED_STS(Sleep) is Not occured: %d\n", ret);
      return ret;
    }
#endif

  g_state = STATE_ISX012_SLEEP;
  g_i2c_freq = I2CFREQ_FAST;

  /* initialize the isx012 hardware */
  ret = isx012_putreglist(priv, g_isx012_def_init, ISX012_RESET_NENTRIES);
  if (ret < 0)
    {
      imagererr("isx012_putreglist failed: %d\n", ret);
      board_isx012_set_reset();
      return ret;
    }

#ifdef ISX012_AF_EN
#ifdef ISX012_FIRST_SET_AF
/*  board_isx012_release_sleep(); */

  /* AF default driver load */
  ret = isx012_putreg(priv, AF_EXT, 1, 1);
  if (ret < 0)
    {
      board_isx012_set_reset();
      return ret;
    }

  ret = isx012_chk_reg_bit(priv, AF_EXT,
                                 AF_EXT_AFRAMDRVFIN,
                                 AF_EXT_DELAY_TIME,
                                 AF_EXT_WAIT_TIME,
                                 AF_EXT_TIMEOUT);
  if (ret < 0 || ret > 0)
    {
      board_isx012_set_reset();
      return ret;
    }

  g_af_loadsts = 1;

/*  board_isx012_set_sleep(1); */
/*  g_state = STATE_ISX012_SLEEP; */
#endif
#endif

  ret = isx012_set_mode_param(priv, set_moni_param, set_cap_param);
  if (ret < 0)
    {
      board_isx012_set_reset();
      return ret;
    }

  return OK;
}

static int isx012_open(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct isx012_dev_s *priv = inode->i_private;
  int ret = 0;

  ret = board_isx012_power_on();
  if (ret < 0)
    {
      imagererr("Failed to power on %d\n", ret);
      return ret;
    }

  ret = isx012_initialize(priv);
  if (ret < 0)
    {
      imagererr("Failed to open %d\n", ret);
      board_isx012_set_reset();
      board_isx012_power_off();
      return ret;
    }

  ret = cxd56_cisifinit();
  if (ret < 0)
    {
      imagererr("Fail cxd56_cisifinit %d\n", ret);
    }

  return ret;
}

static int isx012_close(FAR struct file *filep)
{
  int ret = 0;

  if (g_state == STATE_ISX012_ACTIVE)
    {
      board_isx012_set_sleep(1);
    }

  board_isx012_set_reset();

  ret = board_isx012_power_off();
  if (ret < 0)
    {
      imagererr("Failed to power off %d\n", ret);
      return ret;
    }

  ret = cxd56_cisiffinalize();
  if (ret < 0)
    {
      imagererr("Fail cxd56_cisiffinalize %d\n", ret);
      return ret;
    }

  g_i2c_freq = I2CFREQ_STANDARD;
  g_state    = STATE_ISX012_POWEROFF;
  g_af_loadsts = 0;

  return ret;
}

static int isx012_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct isx012_dev_s *priv  = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      case IMGIOC_SETSTATE:
        ret = isx012_change_device_state(priv, arg);
        break;
      case IMGIOC_SETMODE:
        ret = isx012_change_camera_mode(priv, arg);
        break;
      case IMGIOC_SETMODEP:
        ret = isx012_change_mode_param(priv, (isx012_t *)arg);
        break;
      case IMGIOC_SETCISIF: //@@@
        ret = isx012_change_cisif(priv, (cisif_param_t *)arg);
        break;
      case IMGIOC_READREG:
        ret = isx012_read_reg(priv, (isx012_reg_t *)arg);
        break;
      case IMGIOC_WRITEREG:
        ret = isx012_write_reg(priv, (isx012_reg_t *)arg);
        break;
      case IMGIOC_MONIREF:
        ret = isx012_set_moni_refresh(priv, arg);
        break;
      default:
        imagererr("Unrecognized cmd: %d\n", cmd);
        ret = - ENOTTY;
        break;
    }

  return ret;
}

int isx012_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
  FAR struct isx012_dev_s *priv;
  char path[16];
  int ret;

  /* Initialize the ISX012 device structure */

  priv = (FAR struct isx012_dev_s *)kmm_malloc(sizeof(struct isx012_dev_s));
  if (!priv)
    {
      imagererr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  g_i2c_freq = I2CFREQ_STANDARD;
  g_state    = STATE_ISX012_POWEROFF;
  g_af_loadsts = 0;

  priv->i2c  = i2c;
  priv->addr = ISX012_I2C_SLV_ADDR;
  priv->freq = g_i2c_freq;
  /* moni=YUV, cap=JPEG */
  priv->image.moni_param.format     = FORMAT_ISX012_YUV;
  priv->image.moni_param.rate       = RATE_ISX012_30FPS;
  priv->image.cap_param.format      = FORMAT_ISX012_JPEG_MODE1;
  priv->image.cap_param.rate        = RATE_ISX012_15FPS;
  /* moni=QVGA, cap=VGA */
  priv->image.moni_param.yuv_hsize  = 320;
  priv->image.moni_param.yuv_vsize  = 240;
  priv->image.moni_param.jpeg_hsize = 320;
  priv->image.moni_param.jpeg_vsize = 240;
  priv->image.cap_param.yuv_hsize   = 640;
  priv->image.cap_param.yuv_vsize   = 480;
  priv->image.cap_param.jpeg_hsize  = 640;
  priv->image.cap_param.jpeg_vsize  = 480;
  sem_init(&priv->wait, 0, 0);

  /* Register the character driver */

  (void) snprintf(path, sizeof(path), "%s%d", devpath, 0);
  ret = register_driver(path, &g_isx012fops, 0666, priv);
  if (ret < 0)
    {
      imagererr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }
  else
    {
      imagerinfo("ISX012 driver loaded successfully!\n");
    }

  return ret;
}
