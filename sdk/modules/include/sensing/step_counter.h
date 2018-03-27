/****************************************************************************
 * include/sensing/step_counter.h
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

#ifndef __INCLUDE_SENSING_STEP_COUNTER_H
#define __INCLUDE_SENSING_STEP_COUNTER_H

/**
 * @defgroup logical_stepcounter Step Counter API 
 * @{
 */

#include <asmp/mpmq.h>
#include <asmp/mptask.h>
#include "memutils/memory_manager/MemHandle.h"
#include "sensing/sensor_command.h"
#include "sensing/sensor_id.h"
#include "sensing/sensor_ecode.h"
#include "memutils/s_stl/queue.h"

/**
 * @def
 * initial step length of each move type
 */

#define STEP_COUNTER_INITIAL_WALK_STEP_LENGTH 60   /* [cm] */
#define STEP_COUNTER_INITIAL_RUN_STEP_LENGTH  80   /* [cm] */

/* -------------------------------------------------------------------------- */
/**
 * @enum StepCounterStepMode
 * @brief Step modes
 */
typedef enum
{
  STEP_COUNTER_MODE_FIXED_LENGTH = 0,    /**< Using fixed length (not using existing stride table).         */
  STEP_COUNTER_MODE_NEW_TABLE,           /**< Create a new stride table with stepLength, and use the table. */
  STEP_COUNTER_MODE_STEP_TABLE           /**< Using existing stride table (not using fixed length).         */

} StepCounterStepMode;
/* -------------------------------------------------------------------------- */
/**
 * @struct StepCounterSetParam
 * @brief the structure of step setting for initialize.
 */
struct step_counter_param_s
{
  int32_t              step_length;      /**< Step stride setting value. Min 1[cm], Max 249[cm]. */
  StepCounterStepMode  step_mode;        /**< Setting of using the stride table. */
};

typedef struct step_counter_param_s StepCounterSetParam;

/* -------------------------------------------------------------------------- */
/**
 * @struct StepCounterSetting
 * @brief the structure of Accelstep user setting.
 */
struct step_counter_setting_s
{
  StepCounterSetParam      walking;      /**< User Setting for walking. */
  StepCounterSetParam      running;      /**< User Setting for running. */
};

typedef struct step_counter_setting_s StepCounterSetting;

/*--------------------------------------------------------------------
    StepCounter Class
  --------------------------------------------------------------------*/

class StepCounterClass
{
public:

  /* public methods */
  int open(void);
  int close(void);
  int write(sensor_command_data_mh_t*);
  void set_callback(void);
  int receive(void);
  int set(StepCounterSetting *);

  StepCounterClass(MemMgrLite::PoolId cmd_pool_id)
      : m_cmd_pool_id(cmd_pool_id)
  {
  };

  ~StepCounterClass(){};

private:
  #define MAX_EXEC_COUNT 8
  struct exe_mh_s {
    MemMgrLite::MemHandle cmd;
    MemMgrLite::MemHandle data;
  };
  s_std::Queue<struct exe_mh_s, MAX_EXEC_COUNT> m_exe_que;
  
  /* private members */

  MemMgrLite::PoolId m_cmd_pool_id;

  mptask_t    m_mptask;
  mpmq_t      m_mq;

  pthread_t m_thread_id;

  /* private methods */

  int sendInit(void);
};

/*--------------------------------------------------------------------
    External Interface
  --------------------------------------------------------------------*/

/**
 * @brief Create StepCounterClass instance. 
 * return Address for instance of StepCounterClass
 *
 */
StepCounterClass* StepCounterCreate(MemMgrLite::PoolId cmd_pool_id);

/**
 * @brief     Load AESM library and boot up as worker task.
 *            After booted up, send initialize and wait complete.
 * @param[in] ins : instance address of StepCounterClass
 * @return    result of process.
 */
int StepCounterOpen(StepCounterClass* ins);

/**
 * @brief     Destory AESM worker task.
 * @param[in] ins : instance address of StepCounterClass
 * @return    result of process.
 */
int StepCounterClose(StepCounterClass* ins);

/**
 * @brief     Send data to AESM worker task.
 * @param[in] ins : instance address of StepCounterClass
 * @param[in] command : command including data to send
 * @return    result of process
 */
int StepCounterWrite(StepCounterClass* ins, sensor_command_data_mh_t* command);

/**
 * @brief     User step set function.
 * @param[in] ins : instance address of StepCounterClass
 * @param[in] set : user step set pointer
 * @return    result of process
 */
int StepCounterSet(StepCounterClass *ins, StepCounterSetting *set);

/**
 * @}
 */

#endif /* __INCLUDE_SENSING_STEP_COUNTER_H */

