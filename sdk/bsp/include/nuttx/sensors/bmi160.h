/********************************************************************************************
 * include/nuttx/sensors/bmi160.h
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
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_BMI160_H
#define __INCLUDE_NUTTX_SENSORS_BMI160_H

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#if defined(CONFIG_BMI160)

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

#define BMI160_SPI_MAXFREQUENCY 10000000

/* Configuration ****************************************************************************/
/* Prerequisites:
 *
 * CONFIG_BMI160
 *   Enables support for the BMI160 driver
 */

/* IOCTL Commands ***************************************************************************/

#define SNIOC_ENABLESC     _SNIOC(0x0001) /* Arg: uint8_t value */
#define SNIOC_READSC       _SNIOC(0x0002) /* Arg: int16_t* pointer */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/****************************************************************************
 * struct 6-axis data
 ****************************************************************************/
struct accel_t
{
  int16_t x;
  int16_t y;
  int16_t z;
};

struct gyro_t
{
  int16_t x;
  int16_t y;
  int16_t z;
};

struct accel_gyro_st_s
{
  struct gyro_t  gyro;
  struct accel_t accel;
  uint32_t sensor_time;
};

struct spi_dev_s;

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: bmi160_register
 *
 * Description:
 *   Register the BMI160 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/press0"
 *   dev     - An instance of the SPI interface to use to communicate with
 *             BMI160
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmi160_register(FAR const char *devpath, FAR struct spi_dev_s *dev);

#ifdef CONFIG_BMI160_SCU
int bmi160_init(FAR struct spi_dev_s *dev);

int bmi160gyro_register(FAR const char *devname, int minor,
                        FAR struct spi_dev_s *dev);
int bmi160accel_register(FAR const char *devname, int mirno,
                         FAR struct spi_dev_s *dev);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_BMI160 */
#endif /* __INCLUDE_NUTTX_SENSORS_BMI160_H */
