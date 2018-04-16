/****************************************************************************
 * configs/cxd56xx/src/cxd56_lt1pa01.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>

#include <nuttx/sensors/lt1pa01.h>
#ifdef CONFIG_LT1PA01_SCU
#include <arch/chip/cxd56_scu.h>
#endif

#include "cxd56_i2c.h"

#if defined(CONFIG_CXD56_I2C) && defined(CONFIG_LT1PA01)

#ifdef CONFIG_LT1PA01_SCU
int board_lt1pa01_initialize(int bus)
{
  int ret;
  FAR struct i2c_master_s *i2c;

  sninfo("Initializing LT1PA01...\n");

  /* Initialize i2c deivce */

  i2c = cxd56_i2cbus_initialize(bus);
  if (!i2c)
    {
      snerr("ERROR: Failed to initialize i2c%d.\n", bus);
      return -ENODEV;
    }

  ret = lt1pa01_init(i2c, bus);
  if (ret < 0)
    {
      snerr("Error initialize LT1PA01.\n");
      return ret;
    }

  /* Register devices for each FIFOs at I2C bus */

  ret = lt1pa01als_register("/dev/light", 0, i2c, bus);
  if (ret < 0)
    {
      snerr("Error registering LT1PA01[ALS].\n");
      return ret;
    }

  ret = lt1pa01prox_register("/dev/proximity", 0, i2c, bus);
  if (ret < 0)
    {
      snerr("Error registering LT1PA01[PS].\n");
      return ret;
    }

  return ret;
}
#endif /* CONFIG_LT1PA01_SCU */

#endif /* CONFIG_CXD56_I2C && CONFIG_LT1PA01 */