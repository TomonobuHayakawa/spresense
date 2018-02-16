/****************************************************************************
 * configs/spresense/src/spresense.h
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Kazuya Hioki <Kazuya.Hioki@sony.com>
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

#ifndef __CONFIGS_SPRESENSE_SRC_SPRESENSE_H
#define __CONFIGS_SPRESENSE_SRC_SPRESENSE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include "cxd56_pinconfig.h"

struct i2c_master_s;
struct spi_dev_s;

/* LED definitions *********************************************************/

#define GPIO_LED1           (PIN_PWM0)
#define GPIO_LED2           (PIN_PWM1)

/* Buttons definitions *****************************************************/

#define GPIO_BUT1           (PIN_SPI2_MOSI)
#define GPIO_BUT2           (PIN_SPI2_MISO)

/****************************************************************************
 * Name: cxd56_gnssinitialize
 *
 * Description:
 *   Called to configure an CXD56xx internal GNSS for spresense board
 *
 ****************************************************************************/

#ifdef CONFIG_CXD56_GNSS
int cxd56_gnssinitialize(FAR const char *devpath);
#endif

/****************************************************************************
 * Name: cxd56_userled_initialize
 *
 * Description:
 *   Called to configure an leds for spresense board
 *
 ****************************************************************************/

#ifdef CONFIG_USERLED_LOWER
int cxd56_userled_initialize(FAR const char *devname);
#endif

/****************************************************************************
 * Name: cxd56_geofenceinitialize
 *
 * Description:
 *   Called to configure an CXD56xx internal GNSS for spresense board
 *
 ****************************************************************************/

#ifdef CONFIG_CXD56_GEOFENCE
int cxd56_geofenceinitialize(FAR const char *devpath);
#endif

/****************************************************************************
 * Name: cxd56_alt1160initialize
 *
 * Description:
 *   Called to configure an SPI and to register ALT1160
 *   for the spresense board.
 *
 ****************************************************************************/

#ifdef CONFIG_MODEM_ALT_1160
int cxd56_alt1160initialize(FAR const char *devpath, FAR struct spi_dev_s* spi);
#endif

/****************************************************************************
 * Name: cxd56_lpm013m091a_initialize
 *
 * Description:
 *   Called to configure an LPM013M091A LCD driver for corvo board
 *
 ****************************************************************************/

#ifdef CONFIG_LCD_LPM013M091A
#  ifdef CONFIG_NX_LCDDRIVER
FAR struct lcd_dev_s *cxd56_lpm013m091a_initialize(FAR struct spi_dev_s *spi);
#  else
FAR struct fb_vtable_s *cxd56_lpm013m091a_fb_initialize(FAR const char *devpath, FAR struct spi_dev_s *spi);
#  endif
#endif

/****************************************************************************
 * Name: cxd56_isx012initialize
 *
 * Description:
 *   Called to configure an I2C and to register ISX012 for the corvo board.
 *
 ****************************************************************************/

#ifdef CONFIG_VIDEO_ISX012
int cxd56_isx012initialize(FAR const char *devpath, FAR struct i2c_master_s* i2c);
#endif

/****************************************************************************
 * Name: cxd56_bmp280initialize
 *
 * Description:
 *   Called to configure an I2C and to register BMP280 for the corvo board.
 *
 ****************************************************************************/

#ifdef CONFIG_BMP280
int cxd56_bmp280initialize(FAR struct i2c_master_s* i2c);
#endif

/****************************************************************************
 * Name: cxd56_ak09912initialize
 *
 * Description:
 *   Called to configure an I2C and to register AKM09912 for corvo board
 *
 ****************************************************************************/

#ifdef CONFIG_AK09912
int cxd56_ak09912initialize(FAR const char *devpath, FAR struct i2c_master_s* i2c);
#endif

/****************************************************************************
 * Name: cxd56_lt1pa01initialize
 *
 * Description:
 *   Called to configure an I2C and to register LT1PA01 for corvo board
 *
 ****************************************************************************/

#ifdef CONFIG_LT1PA01
int cxd56_lt1pa01initialize(FAR struct i2c_master_s* i2c);
#endif

/****************************************************************************
 * Name: cxd56_kx022initialize
 *
 * Description:
 *   Called to configure an I2C and to register KX022 for corvo board
 *
 ****************************************************************************/

#ifdef CONFIG_KX022
int cxd56_kx022initialize(FAR const char *devpath, FAR struct i2c_master_s* i2c);
#endif

/****************************************************************************
 * Name: cxd56_rpr0521rsinitialize
 *
 * Description:
 *   Called to configure an I2C and to register RPR0521RS for collet board
 *
 ****************************************************************************/

#ifdef CONFIG_RPR0521RS
int cxd56_rpr0521rsinitialize(FAR struct i2c_master_s* i2c);
#endif

/****************************************************************************
 * Name: cxd56_bh1745nucinitialize
 *
 * Description:
 *   Called to configure an I2C and to register BH1745NUC for collet board
 *
 ****************************************************************************/

#ifdef CONFIG_BH1745NUC
int cxd56_bh1745nucinitialize(FAR const char *devpath, FAR struct i2c_master_s* i2c);
#endif

#endif /* __CONFIGS_SPRESENSE_SRC_SPRESENSE_H */
