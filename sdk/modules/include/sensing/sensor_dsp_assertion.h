/****************************************************************************
 * modules/include/sensing/sensor_dsp_assertion.h
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

#ifndef __INCLUDE_SENSING_SENSOR_DSP_ASSERTION_H
#define __INCLUDE_SENSING_SENSOR_DSP_ASSERTION_H

/**
 * @defgroup logical_sensor Logical Sensors
 * @{
 */


#include <stdint.h>

/**
 * @file sensor_dsp_assertion.h
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* --------------------------------------------------------------------------
  Enumerations
   -------------------------------------------------------------------------- */
/**
 * @enum SensorErrorCode
 * @brief error codes.
 */
typedef enum {
  SensorNoError ,                          /**< No error.                       */
  SensorInvalidArg ,                       /**< Illegal argument.               */
  SensorInvalidCommandId ,                 /**< Invalid command ID.             */
  SensorInvalidEventType ,                 /**< Invalid event type.             */
  SensorInvalidSensorType ,                /**< Illegal sensor type.            */
  SensorNotUpdate ,                        /**< No update.                      */
  SensorIllegalState ,                     /**< Status abnormality.             */
  SensorTimestampDiscontinuous ,           /**< Time stamps are not continuous. */
  SensorAssertionFail ,                    /**< Abnormal occurrence.            */
} SensorErrorCode;

/* --------------------------------------------------------------------------
  Command Structures.
   -------------------------------------------------------------------------- */
/**
 * @struct SensorAssertionInfo
 * @brief Assert information structure
 */
typedef struct {

  SensorErrorCode code;            /**< Error code See SensorErrorCode in sensor_cmd_defs.h.       */

  uint16_t        src_file_no;     /**< Serial number of the fault detection source file. */
  uint16_t        line;            /**< Line number of the fault detection source file. */

} SensorAssertionInfo;
/* -------------------------------------------------------------------------- */

#ifdef __cplusplus
};
#endif

/**
 * @}
 */

#endif/* __INCLUDE_SENSING_SENSOR_DSP_ASSERTION_H */
