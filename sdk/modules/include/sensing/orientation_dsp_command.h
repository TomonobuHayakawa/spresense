/****************************************************************************
 * include/sensing/orientation_dsp_command.h
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

#ifndef __INCLUDE_SENSING_ORIENTATION_DSP_COMMAND_H
#define __INCLUDE_SENSING_ORIENTATION_DSP_COMMAND_H

/**
 * @defgroup logical_orientation Orientation DSP API
 * @{
 */

#include <stdint.h>

#include "sensing/physical_dsp_command.h"

/**
 * @file orientation_dsp_command.h
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @def ORIENTATION_ACCEL_SAMPLING_MAX
 * @brief Acceleration sensor Indicates the maximum number of data samples.
 */
#define ORIENTATION_ACCEL_SAMPLING_MAX         (16)
/**
 * @def ORIENTATION_MAG_SAMPLING_MAX
 * @brief Magnetometer sensor Indicates the maximum number of data samples.
 */
#define ORIENTATION_MAG_SAMPLING_MAX           (8)

/* --------------------------------------------------------------------------
  Enumerations
   -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/**
 * @enum OrientationCmdType
 * @brief Publishers
 */
typedef enum {
  ORIENTATION_CMD_UPDATE_ACCEL = 0 ,     /**< Used to update acceleration sensor data. */
  ORIENTATION_CMD_UPDATE_MAG ,           /**< Used to update magnetometer sensor data. */
} OrientationCmdType;

/* --------------------------------------------------------------------------
  Command Structures
   -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/**
 * @struct SensorInitOrientation
 * @brief the structure of Accelstep setting for initilaize.
 */
typedef struct {

} SensorInitOrientation;


/* -------------------------------------------------------------------------- */


/* -------------------------------------------------------------------------- */
/**
 * @struct SensorExecOrientation
 * @brief the command of Orientation execute by a frame(a few sample).
 */
typedef struct {

  OrientationCmdType   cmd_type;           /**< Indicates the Orientation command type. */
  ThreeAxisSampleData  update_three_axis;  /**< Frame of 3 axis data.                   */

} SensorExecOrientation;

/* -------------------------------------------------------------------------- */
/**
 * @struct SensorFlushOrientation
 * @brief the command of Orientation terminate.
 */
typedef struct {

} SensorFlushOrientation;


/* --------------------------------------------------------------------------
  the structure of results.
   -------------------------------------------------------------------------- */
/**
 * @struct OrientationData
 * @brief the structure of Orientation results.
 * @brief the result is returned only when ORIENTATION_CMD_UPDATE_MAG is specified as the command type.
 */
typedef struct{

  float azimuth;                         /**<
                                          * Indicates calculation of azimuth from input sensor data.
                                          * The unit is[rad].
                                          * (*)No deviation correction.
                                          */
  float pitch;                           /**<
                                          * Indicates calculation of pitch attitude from input sensor data.
                                          * The unit is[rad].
                                          */
  float roll;                            /**<
                                          * Indicates calculation of roll attitude from input sensor data.
                                          * The unit is[rad].
                                          */
  float decl;                            /**<
                                          * Argument angle.
                                          * The unit is[rad].
                                          * (*)Use Azimuth - Decl
                                          */
  int   acc_azimuth;                     /**<
                                          * Return the azimuth accuracy level. [0 - 3]
                                          */
  int   calib_lv;                        /**<
                                          * Return the calibration accuracy level. [0 - 3]
                                          */

} OrientationData;
/* -------------------------------------------------------------------------- */


#ifdef __cplusplus
};
#endif

/**
 * @}
 */

#endif /*  __INCLUDE_SENSING_ORIENTATION_DSP_COMMAND_H */

