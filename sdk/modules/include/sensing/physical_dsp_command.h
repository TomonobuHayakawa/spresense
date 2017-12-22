/****************************************************************************
 * include/sensing/physical_dsp_command.h
 *
 *   Copyright (C) 2016-2017 Sony Corporation
 *   Author: Tomonobu Hayakawa <Tomonobu.Hayakawa@sony.com>
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

#ifndef __INCLUDE_SENSING_PHYSICAL_DSP_COMMAND_H
#define __INCLUDE_SENSING_PHYSICAL_DSP_COMMAND_H

/**
 * @defgroup physical DSP API
 * @{
 */

#include <stdint.h>

/**
 * @file physical_dsp_command.h
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* -------------------------------------------------------------------------- */
/**
 * @struct ThreeAxisSample
 * @brief Structure of physical sensor sample data using 3 axis.
 */
typedef struct {

  float                ax;               /**<
                                          * Acceleration sensor:
                                          * X axis standard gravity acceleration. The unit is [G].
                                          * Magnetometer sensor:
                                          * Northward component, X. The unit is [uT].
                                          */
  float                ay;               /**<
                                          * Acceleration sensor:
                                          * Y axis standard gravity acceleration. The unit is [G].
                                          * Magnetometer sensor:
                                          * Northward component, Y. The unit is [uT].
                                          */
  float                az;               /**<
                                          * Acceleration sensor:
                                          * Z axis standard gravity acceleration. The unit is [G].
                                          * Magnetometer sensor:
                                          * Northward component, Z. The unit is [uT].
                                          */

} ThreeAxisSample;

/* -------------------------------------------------------------------------- */
/**
 * @struct ThreeAxesSampleData
 * @brief the frame of 3 axis data.
 */
typedef struct {

  unsigned long        time_stamp;       /**< Time stamp at update.[ms]                       */
  uint16_t             sampling_rate;    /**< Sampling frequency of acceleration data sample. */
  uint16_t             sample_num;       /**< 3 axis table number of samples.                 */
  ThreeAxisSample*     p_data;           /**< 3 axis table sample data.                       */

} ThreeAxisSampleData;

/* -------------------------------------------------------------------------- */
/**
 * @struct BarSampleData
 * @brief the frame of Barometer data.
 */
typedef struct {

  unsigned long        time_stamp;       /**< Time stamp at update.[ms]                    */
  uint16_t             sampling_rate;    /**< Sampling frequency of Barometer data sample. */
  uint16_t             sample_num;       /**< Barometer number of samples.                 */
  uint32_t*            p_data;           /**< Barometer sample data.                       */

} BarSampleData;


/* -------------------------------------------------------------------------- */
/**
 * @struct GnssData
 * @brief the struct of GNSS data for the execute command.
 */
typedef struct {

  double               raw_latitude;     /**< Unfiltered latitude. The unit is [degrees].    */
  double               raw_longitude;    /**< Longitude not filtered. The unit is [degree].  */
  double               latitude;         /**<
                                          * Indicates the filtered latitude.
                                          * The unit is [degrees].
                                          * In SDK 2.0, specify the same value as raw_latitude.
                                          */
  double               longitude;        /**<
                                          * Indicates the filtered longitude.
                                          * The unit is [degrees].
                                          * In SDK 2.0, specify the same value as raw_longitude.
                                          */
  float                direction;        /**< Indicates the direction. The unit is [degree]. */
  float                velocity;         /**< Indicates speed. The unit is [m/s].            */
  uint32_t             time_stamp;       /**< Time stamp at update [ms]. */
  uint8_t              pos_fix_mode;     /**<
                                          * Indicates the GNSS position compensation mode.
                                          * (T.B.D Current status 2 fixed([1:Invalid/2:2D/3:3D]).)
                                          */
  uint8_t              vel_fix_mode;     /**<
                                          * Indicates the GNSS speed compensation mode.
                                          * (T.B.D Current status 2 fixed([1:Invalid/2:2D VZ/3:2D Offset/4:3D/5:1D/6:PRED]).)
                                          */

} GnssSampleData;


#ifdef __cplusplus
};
#endif

/**
 * @}
 */

#endif /*  __INCLUDE_SENSING_PHYSICAL_DSP_COMMAND_H */

