/****************************************************************************
 *
 * include/sensing/tramlite_dsp_command.h
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
 * 3. Neither the name Sony nor the names of its contributors
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

#ifndef __INCLUDE_SENSING_TRAMLITE_DSP_COMMAD_H
#define __INCLUDE_SENSING_TRAMLITE_DSP_COMMAD_H

/**
 * @defgroup logical_tramlite TRAMLITE DSP API
 * @{
 */

#include <stdint.h>

#include "sensing/physical_dsp_command.h"

/**
 * @file tramlite_dsp_command.h
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * Accelerometer sensor sampling frequency
 */
#define TRAMLITE_ACC_SAMPLING  (64)

/* --------------------------------------------------------------------------
  Enumerations
   -------------------------------------------------------------------------- */
/**
 * Command type
 */
typedef enum {
  TramliteCmdTypeResult = 0, /**< Result of command*/
  TramliteCmdTypeTrans,      /**< Notification of state transition */
  TramliteCmdTypeNum
} TramliteCmdType;
    
/**
 * TRAMLITE state
 */
typedef enum {
  TramliteStateMs = 0, /**< MS state */
  TramliteStateCmd,    /**< CMD state */
  TramliteStateTmi,    /**< TMI state */
  TramliteStateNum
} TramliteState;


/**
 * Sensor type
 */
typedef enum {
  TramliteSensorAcc = 0 ,  /**< Accelerometer */
} TramliteSensorType;

/**
 * Result of transportation mode inference
 */
enum
{
  TRAMLITE_CLASS_UNDETERMINED = 0, /**< Undetermined */
  TRAMLITE_CLASS_STAY,             /**< Staying */
  TRAMLITE_CLASS_WALK,             /**< Walking */
  TRAMLITE_CLASS_RUN,              /**< Running */
  TRAMLITE_CLASS_VEHICLE,          /**< Getting on vehicle */
  TRAMLITE_CLASS_BICYCLE,          /**< Riding bicycle */
};
  
/* --------------------------------------------------------------------------
  Command Structures
   -------------------------------------------------------------------------- */
/**
 * Initialization command
 */
typedef struct {

  float* likelihood;

} SensorInitTramlite;


/* -------------------------------------------------------------------------- */
/**
 * Execution command
 */
typedef struct {
  TramliteSensorType         type; /**< Sensor type  */

  union {
    ThreeAxisSampleData  acc_data; /**< Accelerometer data */

  };

} SensorExecTramlite;

/* -------------------------------------------------------------------------- */
/**
 * Finalization command
 */
typedef struct {

/* 要求をクリアするだけなのでパラメータなし？*/

} SensorFlushTramlite;

#ifdef __cplusplus
};
#endif

/**
 * @}
 */

#endif /*  __INCLUDE_SENSING_TRAMLITE_DSP_COMMAD_H */

