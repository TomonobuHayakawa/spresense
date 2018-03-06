/****************************************************************************
 * include/sensing/sensor_dsp_command.h
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

#ifndef __INCLUDE_SENSING_SENSOR_DSP_COMMAND_H
#define __INCLUDE_SENSING_SENSOR_DSP_COMMAND_H
/**
 * @defgroup logical_sensor Logical Sensors
 * @{
 */


#include <stdint.h>

/**
 * @file sensor_dsp_command.h
 */

/*
  T.B.D
    Currently, header files of each logical sensor are placed in each directory.
    However, it is not preferable for the step counter to refer to the gesture directory.
    For this reason, consider modification.
*/
#include "sensing/sensor_dsp_assertion.h"
#include "sensing/aesm_dsp_command.h"
#include "sensing/arm_gesture_dsp_command.h"
#include "sensing/orientation_dsp_command.h"
#include "sensing/tram_dsp_command.h"
#include "sensing/tramlite_dsp_command.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

inline bool is_async_msg(uint32_t data)
{
  return ((data & 0x80000000) != 0) ? true : false;
}

inline uint8_t get_async_msgtype(uint32_t param)
{
  return static_cast<uint8_t>((param & 0x0000FF00) >> 8);
}

inline uint8_t get_async_msgparam(uint32_t param)
{
  return static_cast<uint8_t>(param & 0x000000FF);
}

/* --------------------------------------------------------------------------
  Enumerations
   -------------------------------------------------------------------------- */
/**
 * @enum SensorEventType
 * @brief codes of command events.
 */
typedef enum {
  InvalidSensorEvent = 0xFF ,              /**< Illegal event type.   */
  InitEvent = 0,                           /**< Initialization event. */
  ExecEvent ,                              /**< Execution event.      */
  FlushEvent ,                             /**< Terminal event.       */
  SensorEventTypeNum ,                     /**< Number of events.     */
} SensorEventType;

/* -------------------------------------------------------------------------- */
/**
 * @enum SensorProcessMode
 * @brief codes of command logical sensor modes.
 * (T.B.D:tentative!!)
 */
typedef enum {
  InvalidSensorProcessMode = 0xFF ,        /**< Illegal process mode. */
  CommonMode = 0 ,                         /**< Common mode.          */
  AesmMode ,                               /**< AESM mode.            */
  GestureProcMode ,                        /**< Gesture mode.         */
  CompassProcMode ,                        /**< Compass mode.         */
  TramProcMode ,                           /**< Tram mode.            */
  TramliteProcMode ,                       /**< Tramlite mode.        */
} SensorProcessMode;

/**
 * @enum SensorType
 * @brief codes of command logical sensor types.
 */
typedef enum {
  InvalidSensorType = 0xFF ,               /**< Illegal sensor type.      */
  StepCounter = 0 ,                        /**< Step counter sensor type. */
  ArmGesture ,                             /**< Arm Gesture sensor type.  */
  Compass ,                                /**< Compass sensor type.      */
  TransportationMode ,                     /**< Tram sensor type.         */
  TransportationModeLite ,                 /**< Tram sensor type.         */
} SensorType;


/* -------------------------------------------------------------------------- */
/**
 * @enum SensorExecResult
 * @brief DSP results.
 */
typedef enum {
  SensorOK ,                               /**< Command execution succeeded.                    */
  SensorError ,                            /**< Command execution failed.                       */
  SensorWarning ,                          /**< Warning is issued when the command is executed. */
  SensorResultData,                        /**< Result Event from DSP.                          */
} SensorExecResult;


/* --------------------------------------------------------------------------
  these structures of sensor dsp commands.
   -------------------------------------------------------------------------- */
/**
 * @struct SensorDspCmdHeader
 * @brief the header of DSP commands.
 */
typedef struct {

  uint8_t    context_id;
  uint8_t    event_type;                   /**<
                                            * Indicates the event type of the command.
                                            * See SensorEventType in sensor_cmd_defs.h.
                                            */
  uint8_t    sensor_type;                  /**<
                                            * Indicates the sensor type of the command.
                                            * See SensorType in sensor_cmd_defs.h.
                                            */

} SensorDspCmdHeader;

/* -------------------------------------------------------------------------- */
/**
 * @struct SensorResult
 * @brief the structure of DSP result on DSP commands.
 * (T.B.D:In the future, use the sensor that you specified in config to isolate.)
 */
typedef struct {

  SensorExecResult      exec_result;       /**< execute resule See SensorExecResult in sensor_cmd_defs.h.  */
  union {
    AesmStepCount       result_steps;      /**< Structure of step count, speed information after updating. */
    GestureArmPosition  result_arm_pos;    /**< Structure showing the acquired arm position.               */
    GestureTypes        result_gesture;    /**< Structure showing the type of recognized gesture.          */
    OrientationData     result_orientation;/**< Structure showing the acquired azimuth angle.              */
    SensorAssertionInfo assert_info;       /**< Assert information structure.                              */
  };

} SensorResult;

/* -------------------------------------------------------------------------- */
/**
 * @struct SensorDspCmd
 * @brief the structure of DSP commands.
 */
typedef struct {

  SensorDspCmdHeader    header;                    /**< DSP command header.                 */

  union {

    SensorInitAesm         init_aesm_cmd;          /**< AESM initialization command.        */
    SensorInitGesture      init_gesture_cmd;       /**< Gesture initialization command.     */
    SensorInitOrientation  init_orientation_cmd;   /**< Orientation initialization command. */
    SensorInitTram         init_tram_cmd;          /**< TRAM initialization command.        */
    SensorInitTramlite     init_tramlite_cmd;      /**< TRAMLITE initialization command.    */
    // If a sensor is added and you need to do an Init, we will add an Init structure.

    SensorExecAesm         exec_aesm_cmd;          /**< AESM execution command.             */
    SensorExecGesture      exec_gesture_cmd;       /**< Gesture execution command.          */
    SensorExecOrientation  exec_orientation_cmd;   /**< Orientation execution command.      */
    SensorExecTram         exec_tram_cmd;          /**< TRAM initialization command.        */
    SensorExecTramlite     exec_tramlite_cmd;      /**< TRAMLITE initialization command.    */
    // If sensor is added and you need to do exec, we will add Exec structure.

    SensorFlushAesm         flush_aesm_cmd;        /**< AESM termination command.           */
    SensorFlushGesture      flush_gesture_cmd;     /**< Gesture termination command.        */
    SensorFlushOrientation  flush_orientation_cmd; /**< Orientation termination command.    */
    SensorFlushTram         flush_tram_cmd;        /**< TRAM termination command.           */
    SensorFlushTramlite     flush_tramlite_cmd;    /**< TRAMLITE termination command.       */
    // If a sensor is added and you need to do a flush, add a Flush structure.

  };

  SensorResult      result;                        /**< Result value
                                                    * (execution result, acquired content)
                                                    */

} SensorDspCmd;


#ifdef __cplusplus
};
#endif

/**
 * @}
 */


#endif /* __INCLUDE_SENSING_SENSOR_DSP_COMMAND_H */

