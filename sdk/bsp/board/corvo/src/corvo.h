/****************************************************************************
 * configs/corvo/src/corvo.h
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

#ifndef __CONFIGS_CORVO_SRC_CORVO_H
#define __CONFIGS_CORVO_SRC_CORVO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include "cxd56_pinconfig.h"

struct spi_dev_s;

/* LED definitions *********************************************************/

#define GPIO_LED1           (PIN_PWM0)
#define GPIO_LED2           (PIN_PWM1)

/* Buttons definitions *****************************************************/

#define GPIO_BUT1           (PIN_SPI2_MOSI)
#define GPIO_BUT2           (PIN_SPI2_MISO)

/* Do we need to register SPI drivers on behalf of the SPI tool? */

#define HAVE_SPITOOL 1
#if !defined(CONFIG_SYSTEM_SPITOOL) || !defined(CONFIG_SPI_DRIVER)
#  undef HAVE_SPITOOL
#endif

/****************************************************************************
 * Name: cxd56_gnssinitialize
 *
 * Description:
 *   Called to configure an CXD56xx internal GNSS for corvo board
 *
 ****************************************************************************/

#ifdef CONFIG_CXD56_GNSS
int cxd56_gnssinitialize(FAR const char *devpath);
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
 * Name: cxd56_et014tt1_initialize
 *
 * Description:
 *   Called to configure an ET014TT1 EInk driver for corvo board
 *
 ****************************************************************************/

#ifdef CONFIG_EINK_ET014TT1
FAR struct fb_vtable_s *cxd56_et014tt1_initialize(FAR const char *devpath,
                    FAR struct spi_dev_s *spi);
#endif

/****************************************************************************
 * Name: cxd56_userled_initialize
 *
 * Description:
 *   Called to configure an leds for corvo board
 *
 ****************************************************************************/

#ifdef CONFIG_USERLED_LOWER
int cxd56_userled_initialize(FAR const char *devname);
#endif

/****************************************************************************
 * Name: cxd56_geofenceinitialize
 *
 * Description:
 *   Called to configure an CXD56xx internal GNSS for corvo board
 *
 ****************************************************************************/

#ifdef CONFIG_CXD56_GEOFENCE
int cxd56_geofenceinitialize(FAR const char *devpath);
#endif

#endif /* __CONFIGS_CORVO_SRC_CORVO_H */
