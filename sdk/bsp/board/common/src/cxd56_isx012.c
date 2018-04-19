/****************************************************************************
 * bsp/board/common/src/cxd56_isx012.c
 *
 *   Copyright (C) 2016 Sony Corporation
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
#include <debug.h>
#include <errno.h>

#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"
#include "cxd56_i2c.h"

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check if the following are defined in the board.h */

#ifndef IMAGER_RST
#  error "IMAGER_RST must be defined in board.h !!"
#endif
#ifndef IMAGER_SLEEP
#  error "IMAGER_SLEEP must be defined in board.h !!"
#endif

#define STANDBY_TIME                (600*1000) /* TODO: (max100ms/30fps)*/
#define DEVICE_STARTUP_TIME           (6*1000) /* ms */
#define SLEEP_CANCEL_TIME            (13*1000) /* ms */
#define POWER_CHECK_TIME             (1*1000)  /* ms */

#define ALL_POWERON                 (7)
#define ALL_POWEROFF                (0)
#define POWER_CHECK_RETRY           (10)

int board_isx012_power_on(void)
{
  int ret;
  uint32_t stat;
  int i;

  /* 'POWER_IMAGE_SENSOR==PMIC_GPO(4/5/7)' */

  ret = board_power_control(POWER_IMAGE_SENSOR, true);
  if (ret)
    {
      _err("ERROR: Failed to power on ImageSensor. %d\n", ret);
      return -ENODEV;
    }

  ret = -ETIMEDOUT;
  for (i = 0; i < POWER_CHECK_RETRY; i++)
    {
      stat = 0;
      stat |= (uint32_t)board_power_monitor(PMIC_GPO(4)) << 0;
      stat |= (uint32_t)board_power_monitor(PMIC_GPO(5)) << 1;
      stat |= (uint32_t)board_power_monitor(PMIC_GPO(7)) << 2;
      if (stat == ALL_POWERON)
        {
          ret = OK;
          break;
        }

      usleep(POWER_CHECK_TIME);
    }
  return ret;
}

int board_isx012_power_off(void)
{
  int ret;
  uint32_t stat;
  int i;

  /* POWER_IMAGE_SENSOR==PMIC_GPO(4/5/7) */

  ret = board_power_control(POWER_IMAGE_SENSOR, false);
  if (ret)
    {
      _err("ERROR: Failed to power off ImageSensor. %d\n", ret);
      return -ENODEV;
    }

  ret = -ETIMEDOUT;
  for (i = 0; i < POWER_CHECK_RETRY; i++)
    {
      stat = 0;
      stat |= (uint32_t)board_power_monitor(PMIC_GPO(4)) << 0;
      stat |= (uint32_t)board_power_monitor(PMIC_GPO(5)) << 1;
      stat |= (uint32_t)board_power_monitor(PMIC_GPO(7)) << 2;
      if (stat == ALL_POWEROFF)
        {
          ret = OK;
          break;
        }

      usleep(POWER_CHECK_TIME);
    }

  return ret;
}

void board_isx012_set_reset(void)
{
  cxd56_gpio_write(IMAGER_RST, false);
}

void board_isx012_release_reset(void)
{
  cxd56_gpio_write(IMAGER_RST, true);
}

void board_isx012_set_sleep(int kind)
{
  cxd56_gpio_write(IMAGER_SLEEP, false);
  if (kind == 0)
    {
      /* PowerON -> sleep */

      usleep(DEVICE_STARTUP_TIME);
    }
  else
    {
      /* active -> sleep */

      usleep(STANDBY_TIME);
    }
}

void board_isx012_release_sleep(void)
{
  cxd56_gpio_write(IMAGER_SLEEP, true);
  usleep(SLEEP_CANCEL_TIME);
}

int isx012_register(FAR const char *devpath, FAR struct i2c_master_s *i2c);

int board_isx012_initialize(FAR const char *devpath, int bus)
{
  int ret;
  uint32_t pinconf;
  FAR struct i2c_master_s *i2c;

  _info("Initializing ISX012...\n");

#ifdef IMAGER_ALERT
  cxd56_gpio_config(IMAGER_ALERT, true);
#endif
  cxd56_gpio_config(IMAGER_SLEEP,false);
  cxd56_gpio_config(IMAGER_RST,false);
  board_isx012_set_reset();
  cxd56_gpio_write(IMAGER_SLEEP, false);

  pinconf = PINCONF_SET(PIN_IS_CLK,   PINCONF_MODE1, PINCONF_INPUT_ENABLE,
                        PINCONF_DRIVE_NORMAL, PINCONF_FLOAT);
  cxd56_pin_config(pinconf);
  pinconf = PINCONF_SET(PIN_IS_VSYNC, PINCONF_MODE1, PINCONF_INPUT_ENABLE,
                        PINCONF_DRIVE_NORMAL, PINCONF_FLOAT);
  cxd56_pin_config(pinconf);
  pinconf = PINCONF_SET(PIN_IS_HSYNC, PINCONF_MODE1, PINCONF_INPUT_ENABLE,
                        PINCONF_DRIVE_NORMAL, PINCONF_FLOAT);
  cxd56_pin_config(pinconf);
  pinconf = PINCONF_SET(PIN_IS_DATA0, PINCONF_MODE1, PINCONF_INPUT_ENABLE,
                        PINCONF_DRIVE_NORMAL, PINCONF_FLOAT);
  cxd56_pin_config(pinconf);
  pinconf = PINCONF_SET(PIN_IS_DATA1, PINCONF_MODE1, PINCONF_INPUT_ENABLE,
                        PINCONF_DRIVE_NORMAL, PINCONF_FLOAT);
  cxd56_pin_config(pinconf);
  pinconf = PINCONF_SET(PIN_IS_DATA2, PINCONF_MODE1, PINCONF_INPUT_ENABLE,
                        PINCONF_DRIVE_NORMAL, PINCONF_FLOAT);
  cxd56_pin_config(pinconf);
  pinconf = PINCONF_SET(PIN_IS_DATA3, PINCONF_MODE1, PINCONF_INPUT_ENABLE,
                        PINCONF_DRIVE_NORMAL, PINCONF_FLOAT);
  cxd56_pin_config(pinconf);
  pinconf = PINCONF_SET(PIN_IS_DATA4, PINCONF_MODE1, PINCONF_INPUT_ENABLE,
                        PINCONF_DRIVE_NORMAL, PINCONF_FLOAT);
  cxd56_pin_config(pinconf);
  pinconf = PINCONF_SET(PIN_IS_DATA5, PINCONF_MODE1, PINCONF_INPUT_ENABLE,
                        PINCONF_DRIVE_NORMAL, PINCONF_FLOAT);
  cxd56_pin_config(pinconf);
  pinconf = PINCONF_SET(PIN_IS_DATA6, PINCONF_MODE1, PINCONF_INPUT_ENABLE,
                        PINCONF_DRIVE_NORMAL, PINCONF_FLOAT);
  cxd56_pin_config(pinconf);
  pinconf = PINCONF_SET(PIN_IS_DATA7, PINCONF_MODE1, PINCONF_INPUT_ENABLE,
                        PINCONF_DRIVE_NORMAL, PINCONF_FLOAT);
  cxd56_pin_config(pinconf);

  /* Initialize i2c deivce */

  i2c = cxd56_i2cbus_initialize(bus);
  if (!i2c)
    {
      return -ENODEV;
    }

  ret = isx012_register(devpath, i2c);
  if (ret < 0)
    {
      _err("Error registering ISX012.\n");
    }

  return ret;
}
