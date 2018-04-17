/****************************************************************************
 * configs/collet/src/collet.h
 *
 *   Copyright (C) 2017 Sony Corporation.
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

#ifndef __CONFIGS_COLLET_SRC_COLLET_H
#define __CONFIGS_COLLET_SRC_COLLET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include "cxd56_pinconfig.h"

/* LED definitions *********************************************************/

#define GPIO_LED1           (PIN_PWM2)

/* Buttons definitions *****************************************************/

#define GPIO_BUT1           (PIN_SPI2_MOSI)
#define GPIO_BUT2           (PIN_SPI2_MISO)

/****************************************************************************
 * Name: cxd56_gnssinitialize
 *
 * Description:
 *   Called to configure an CXD56xx internal GNSS for collet board
 *
 ****************************************************************************/

#ifdef CONFIG_CXD56_GNSS
int cxd56_gnssinitialize(FAR const char *devpath);
#endif

/****************************************************************************
 * Name: cxd56_userled_initialize
 *
 * Description:
 *   Called to configure an leds for collet board
 *
 ****************************************************************************/

#ifdef CONFIG_USERLED_LOWER
int cxd56_userled_initialize(FAR const char *devname);
#endif

/****************************************************************************
 * Name: cxd56_geofenceinitialize
 *
 * Description:
 *   Called to configure an CXD56xx internal GNSS for collet board
 *
 ****************************************************************************/

#ifdef CONFIG_CXD56_GEOFENCE
int cxd56_geofenceinitialize(FAR const char *devpath);
#endif

#endif /* __CONFIGS_COLLET_SRC_COLLET_H */
