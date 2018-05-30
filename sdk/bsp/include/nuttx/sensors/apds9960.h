/****************************************************************************
 * bsp/include/nuttx/sensors/apds9960.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_APDS9960_H
#define __INCLUDE_NUTTX_SENSORS_APDS9960_H

#include <sdk/config.h>

#if defined(CONFIG_I2C) && defined(CONFIG_APDS9960)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************
 * Prerequisites:
 *
 * CONFIG_APDS9960
 *   Enables support for the APDS9960 driver
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

struct gesture_data_s
{
  uint8_t up;    /* up raw data    */
  uint8_t down;  /* down raw data  */
  uint8_t left;  /* left raw data  */
  uint8_t right; /* right raw data */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: apds9960_init
 *
 * Description:
 *   Initialize APDS9960 proximity and ambient light sensor device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             APDS9960
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int apds9960_init(FAR struct i2c_master_s *i2c, int port);

/****************************************************************************
 * Name: apds9960als_register
 *
 * Description:
 *   Register the APDS9960 ambient light sensor character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/gesture0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             APDS9960
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int apds9960_register(FAR const char          *devpath,
                      FAR struct i2c_master_s *i2c,
                      int                     port);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_APDS9960 */
#endif /* __INCLUDE_NUTTX_SENSORS_APDS9960_H */
