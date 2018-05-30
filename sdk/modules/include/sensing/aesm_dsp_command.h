/****************************************************************************
 * modules/include/sensing/aesm_dsp_command.h
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

#ifndef __INCLUDE_SENSING_AESM_DSP_COMMAND_H
#define __INCLUDE_SENSING_AESM_DSP_COMMAND_H

/**
 * @defgroup logical_aesm AESM DSP API
 * @{
 */

#include <stdint.h>

#include "sensing/physical_dsp_command.h"

/**
 * @file aesm_dsp_command.h
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @def AESM_SAMPLING_MAX
 * @brief Indicates the maximum number of samples.
 */
#define AESM_SAMPLING_MAX         (32)

/* --------------------------------------------------------------------------
  Enumerations
   -------------------------------------------------------------------------- */
/**
 * @enum AesmMovementType
 * @brief Activity Class
 */
typedef enum {
  AESM_MOVEMENT_TYPE_OTHER = 0 ,         /**< This means that it can not be recognized. */
  AESM_MOVEMENT_TYPE_STILL ,             /**< This means that it is stopped.            */
  AESM_MOVEMENT_TYPE_WALK ,              /**< This means walking.                       */
  AESM_MOVEMENT_TYPE_RUN ,               /**< This means running.                       */
} AesmMovementType;

/* -------------------------------------------------------------------------- */
/**
 * @enum AesmStepMode
 * @brief Step modes
 */
typedef enum {

  AESM_STEPMODE_USE_FIXED_LENGTH = 0 ,   /**< Using fixed length (not using existing stride table).         */
  AESM_STEPMODE_USE_NEW_TABLE,           /**< Create a new stride table with stepLength, and use the table. */
  AESM_STEPMODE_USE_STEP_TABLE,          /**< Using existing stride table (not using fixed length).         */
} AesmStepMode;

/* -------------------------------------------------------------------------- */
/**
 * @enum AesmCmdType
 * @brief Publishers
 */
typedef enum {
  AESM_CMD_UPDATE_ACCELERATION = 0 ,     /**< Acceleration sensor data is used to update acceleration. */
  AESM_CMD_UPDATE_GPS ,                  /**< Update GPS information using GPS sensor data.            */
  AESM_CMD_STEP_SET                      /**< Set user step setting.                                   */
} AesmCmdType;

/* --------------------------------------------------------------------------
  Command Structures
   -------------------------------------------------------------------------- */
/**
 * @struct AesmStepSetting
 * @brief the structure of step setting for initialize.
 */
typedef struct {

  int32_t              step_length;      /**< Step stride setting value. Min 1[cm]:Use default value, Max 249[cm]. */
  AesmStepMode         step_mode;        /**< Setting of using the stride table. */

} AesmStepSetting;

/* -------------------------------------------------------------------------- */
/**
 * @struct AesmSetting
 * @brief the structure of Accelstep user setting.
 */
struct aesm_setting_s
{
  AesmStepSetting      user_set_walking;  /**< User Setting for walking. */
  AesmStepSetting      user_set_running;  /**< User Setting for running. */
};

typedef struct aesm_setting_s AesmSetting;

/* -------------------------------------------------------------------------- */
/**
 * @struct SensorInitAesm
 * @brief the structure of Accelstep setting for initilaize.
 */
typedef struct {

  AesmStepSetting      setting;           /**< Step user setting structure. */

} SensorInitAesm;

/* -------------------------------------------------------------------------- */
/**
 * @struct SensorExecAesm
 * @brief the command of AESM execute by a frame(a few sample).
 */
typedef struct {

  AesmCmdType           cmd_type;         /**< Indicates the AESM command type.                  */

  union {
    ThreeAxisSampleData update_acc;       /**< Acceleration update command setting structure.    */
    GnssSampleData      update_gps;       /**< GPS information update command setting structure. */
    AesmSetting         setting;          /**< Step user setting structure.                      */
  };

} SensorExecAesm;

/* -------------------------------------------------------------------------- */
/**
 * @struct SensorFlushAesm
 * @brief the command of AESM terminate.
 */
typedef struct {

} SensorFlushAesm;


/* --------------------------------------------------------------------------
  the structure of results.
   -------------------------------------------------------------------------- */
/**
 * @struct AesmStepCount
 * @brief the structure of AESM results.
 */
typedef struct {

  float                tempo;            /**<
                                          * Indicates tempo of walking / jogging calculated from the input acceleration data.
                                          * The unit is [Hz].
                                          */
  float                stride;           /**<
                                          * Indicates stride calculated from input acceleration data.
                                          * The unit is [cm].
                                          */
  float                speed;            /**<
                                          * Indicates speed of walking / jogging calculated from the input acceleration data.
                                          * The unit is [m/s].
                                          */
  float                distance;         /**<
                                          * Indicates cumulative travel distance calculated from the input acceleration data.
                                          * The unit is [m].
                                          */
  uint32_t             step;             /**<
                                          * Indicates the number of steps calculated from the input acceleration data.
                                          * The unit is [step].
                                          */
  AesmMovementType     movement_type;    /**< Indicates the walking type calculated from the input acceleration data. */
  uint64_t             time_stamp;       /**< Indicates latest timestamp of the acceleration sensor data used.(T.B.D) */

} AesmStepCount;
/* -------------------------------------------------------------------------- */


#ifdef __cplusplus
};
#endif

/**
 * @}
 */

#endif /*  __INCLUDE_SENSING_AESM_DSP_COMMAND_H */

