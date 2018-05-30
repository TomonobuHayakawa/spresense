/****************************************************************************
 * modules/include/sensing/voice_detection.h
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

#ifndef __INCLUDE_SENSING_VOICE_DETECTION_H
#define __INCLUDE_SENSING_VOICE_DETECTION_H

/**
 * @defgroup logical_vad Voice Activity Detection API
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <asmp/mpmq.h>
#include <asmp/mptask.h>
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/s_stl/queue.h"
#include "sensing/sensor_dsp_command.h"
#include "sensing/sensor_command.h"
#include "sensing/sensor_id.h"
#include "sensing/sensor_ecode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Voice Activity Detection Class */

class VoiceDetectionClass
{

public:

  /* public mathods */

  int open(void);
  int close(void);
  int write(FAR sensor_command_data_mh_t*);
  void set_callback(FAR SensorDspCmd*);
  int receive(void);


  VoiceDetectionClass(MemMgrLite::PoolId pool_id)
    : m_cmd_pool_id(pool_id)
  {
  };

  ~VoiceDetectionClass(){};

private:
  #define MAX_EXEC_COUNT 8
  struct exe_mh_s {
    MemMgrLite::MemHandle cmd;
    MemMgrLite::MemHandle data;
  };
  s_std::Queue<struct exe_mh_s, MAX_EXEC_COUNT> m_exe_que;

  /* private members */

  MemMgrLite::PoolId  m_cmd_pool_id;

  mptask_t  m_mptask;
  mpmq_t    m_mq;

  pthread_t m_thread_id;

  /* private mathods */

  int sendInit(void);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* External C Interface */

/**
 * @brief Create VoiceDetectionClass instance.
 * return Address for instance of VoiceDetectionClass
 */

FAR VoiceDetectionClass *VoiceDetectionCreate(MemMgrLite::PoolId cmd_pool_id);

/**
 * @brief     Load VoiceDetection library and boot up as worker task.
 *            After booted up, send initialize and wait complete.
 * @param[in] ins : instance address of VoiceDetectionClass
 * @return    result of process.
 */

int VoiceDetectionOpen(FAR VoiceDetectionClass *ins);

/**
 * @brief     Destory VoiceDetection worker task.
 * @param[in] ins : instance address of VoiceDetectionClass
 * @return    result of process.
 */

int VoiceDetectionClose(FAR VoiceDetectionClass *ins);

/**
 * @brief     Send data to VoiceDetection worker task.
 * @param[in] ins : instance address of VoiceDetectionClass
 * @param[in] command : command including data to send
 * @return    result of process
 */

int VoiceDetectionWrite(FAR VoiceDetectionClass *ins,
                        FAR sensor_command_data_mh_t *command);

/**
 * @}
 */

#endif /*__INCLUDE_SENSING_VOICE_DETECTION_H */
