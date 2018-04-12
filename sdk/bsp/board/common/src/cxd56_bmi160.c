/****************************************************************************
 * configs/cxd56xx/src/cxd56_bmi160.c
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
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/bmi160.h>
#ifdef CONFIG_BMI160_SCU
#include <arch/chip/cxd56_scu.h>
#endif

#ifdef CONFIG_CXD56_DECI_GYRO
#  define GYRO_NR_SEQS 3
#else
#  define GYRO_NR_SEQS 1
#endif

#ifdef CONFIG_CXD56_DECI_ACCEL
#  define ACCEL_NR_SEQS 3
#else
#  define ACCEL_NR_SEQS 1
#endif

#if defined(CONFIG_SPI) && defined(CONFIG_BMI160)

int board_bmi160_initialize(FAR struct spi_dev_s* spi)
{
  int ret;
#ifdef CONFIG_BMI160_SCU
  int i;

  sninfo("Initializing BMI160..\n");

  ret = bmi160_init(spi);
  if (ret < 0)
    {
      snerr("Error initialize BMI160\n");
      return ret;
    }

  /* Create char devices for each FIFOs */

  for (i = 0; i < GYRO_NR_SEQS; i++)
    {
      ret = bmi160gyro_register("/dev/gyro", i, spi);
      if (ret < 0)
        {
          snerr("Error registering gyroscope. %d\n", ret);
          return ret;
        }
    }

  /* Create char devices for each FIFOs */

  for (i = 0; i < ACCEL_NR_SEQS; i++)
    {
      ret = bmi160accel_register("/dev/accel", i, spi);
      if (ret < 0)
        {
          snerr("Error registering accelerometer. %d\n", ret);
          return ret;
        }
    }

#else
  sninfo("Initializing BMI160..\n");

  ret = bmi160_register("/dev/accel0", spi);
  if (ret < 0)
    {
      snerr("Error registering BMI160\n");
    }
#endif
  return ret;
}

#endif

