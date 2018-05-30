/****************************************************************************
 * modules/include/sensing/arm_gesture_dsp_command.h
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

#ifndef __INCLUDE_SENSING_GESTURE_DSP_COMMAND_H
#define __INCLUDE_SENSING_GESTURE_DSP_COMMAND_H

/**
 * @defgroup logical_arm_gesture GESTURE DSP API
 * @{
 */

#include <stdint.h>

/**
 * @file arm_gesture_dsp_command.h
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @def GESTURE_ARM_UP
 * @brief ID for ARM_UP event, also used for setting arm state.
 */
#define GESTURE_ARM_UP    0x00000001
/**
 * @def GESTURE_ARM_DOWN
 * @brief ID for ARM_DOWN event, also used for setting arm state.
 */
#define GESTURE_ARM_DOWN  0x00000002
/**
 * @def GESTURE_TWIST
 * @brief ID for TWIST event.
*/
#define GESTURE_TWIST    0x00000004
/**
 * @def GESTURE_SHAKE
 * @brief ID for SHAKE event.
 */
#define GESTURE_SHAKE    0x00000010

/**
 * @enum GestureArmPosition
 * @brief Arm positions.
 */
typedef enum {
  GESTURE_POS_UP = 0 ,                      /**< Arm position is up. */
  GESTURE_POS_DOWN ,                        /**< Arm position is down. */
} GestureArmPosition;

/**
 * @enum GestureCmdType
 * @brief Publishers
 */
typedef enum {
  GESTURE_CMD_SET_TYPE = 0 ,                /**< Reconfiguration after initialization.                      */
  GESTURE_CMD_GET_TYPE ,                    /**< Get gesture type.                                          */
  GESTURE_CMD_SET_ARM_POSITION ,            /**< Set arm position. Unusable in SDK 2.0.                     */
  GESTURE_CMD_ROTATION ,                    /**< Rotation. Unusable in SDK 2.0.                             */
} GestureCmdType;

/* --------------------------------------------------------------------------
  Command Structures
   -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/**
 * @struct GestureTypes
 * @brief the structure of gesture type.
 * @brief please join the following values with a logical sum.
 *
 * @brief (GESTURE_ARM_UP, GESTURE_ARM_DOWN, GESTURE_TWIST, GESTURE_SHAKE)
 */
typedef struct {

  uint32_t             type;                /**< Gesture type. */

} GestureTypes;

/**
 * @struct SensorInitGesture
 * @brief the structure of gesture type for initilaize.
 */
typedef struct {

  GestureTypes         gesture_types;       /**< Structure showing gesture type. */

} SensorInitGesture;
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/**
 * @struct GestureSetRotation
 * @brief Rotation matrix. Assume x_adjusted, y_adjusted, z_adjusted as adjusted
 * @brief x, y, z acceleration value accordingly. As well, x_sensor, y_sensor,
 * @brief z_sensor are defined as original x, y, z value from sensor accordingly.
 * @brief x_adjusted, y_adjusted and z_adjusted are calculated as follows:
 * @brief x_adjusted = x_sensor * xx + y_sensor * xy + z_sensor * xz;
 * @brief y_adjusted = x_sensor * yx + y_sensor * yy + z_sensor * yz;
 * @brief z_adjusted = x_sensor * zx + y_sensor * zy + z_sensor * zz;
 */
typedef struct {
    float xx, xy, xz;
    float yx, yy, yz;
    float zx, zy, zz;
} GestureSetRotation;

/**
 * @struct SensorExecGesture
 * @brief the struct of gesture for the execute command.
 */
typedef struct {

  GestureCmdType       cmd_type;            /**< Gesture command type.                      */

  union {
    GestureArmPosition  arm_position;       /**< Structure showing arm position.            */
    GestureTypes        gesture_types;      /**< Structure showing gesture type.            */
    ThreeAxisSampleData update_gesture;     /**< Gesture type information update structure. */
    GestureSetRotation  gesture_rotation;   /**< Structure to set arm rotation information. */
  };

} SensorExecGesture;
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/**
 * @struct SensorFlushGesture
 * @brief the command of gesture terminate.
 */
typedef struct {

} SensorFlushGesture;
/* -------------------------------------------------------------------------- */

#ifdef __cplusplus
};
#endif

/**
 * @}
 */

#endif/* __INCLUDE_SENSING_GESTURE_DSP_COMMAND_H */
