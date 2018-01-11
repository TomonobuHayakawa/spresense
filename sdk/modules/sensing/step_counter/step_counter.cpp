/****************************************************************************
 * sensing/step_counter/step_counter.cpp
 *
 *   Copyright (C) 2017 Sony Corporation
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

#include <sys/time.h>

#include "sensing/step_counter.h"
#include "sensing/sensor_dsp_command.h"
#include <debug.h>
#include "dsp_sensor_version.h"

#define err printf

#define STEPCOUNTER_MQ_ID   1
#define DSP_BOOTED_CMD_ID   1
#define STEPCOUNTER_CMD_ID  2

extern void SF_SendSensorData(sensor_command_data_t* packet);

/*--------------------------------------------------------------------
    StepCounter Data Structure
  --------------------------------------------------------------------*/
typedef struct
{
  unsigned int    type;
  unsigned int    walking_meter;
  unsigned int    running_meter;
} step_counter_initairize_t;

typedef struct
{
  unsigned int    type : 8;
  unsigned int    time : 24;

  unsigned int    num : 16;
  unsigned int    fs : 16;

  unsigned int*   adr;
} step_counter_data_t;

/*--------------------------------------------------------------------
    External Interface  
  --------------------------------------------------------------------*/
StepCounterClass* StepCounterCreate(MemMgrLite::PoolId cmd_pool_id)
{
  return new StepCounterClass(cmd_pool_id);
}

int StepCounterOpen(StepCounterClass* ins)
{
  int ret;

  ret = ins->open();

  return ret;
}

int StepCounterClose(StepCounterClass* ins)
{
  int ret;

  ret = ins->close();
  delete ins;

  return ret; 
}

int StepCounterWrite(StepCounterClass* ins, sensor_command_data_mh_t* command)
{
  int ret;

  ret = ins->write(command);

  return ret;
}

/*--------------------------------------------------------------------
    Internal Functions
  --------------------------------------------------------------------*/

static void *receiver_thread_entry(FAR void *p_instance)
{
  do {
    ((StepCounterClass*)p_instance)->receive();
  } while(1);
  
  return 0;
}

/*--------------------------------------------------------------------
    Step Counter Class
  --------------------------------------------------------------------*/

int StepCounterClass::open(void)
{
  int errout_ret;
  int ret;
  int id;
  uint32_t msgdata;

  /* Initalize Worker as task. */

  ret = mptask_init_secure(&m_mptask, "AESM"/* tentative *//*filename*/);

  if (ret != 0)
    {
      _err("mptask_init() failure. %d\n", ret);
      return SENSOR_DSP_LOAD_ERROR;
    }

  ret = mptask_assign(&m_mptask);

  if (ret != 0)
    {
      _err("mptask_asign() failure. %d\n", ret);
      return SENSOR_DSP_LOAD_ERROR;
    }

  /* Queue for communication between Supervisor and Worker create. */

  ret = mpmq_init(&m_mq, STEPCOUNTER_MQ_ID, mptask_getcpuid(&m_mptask));
  if (ret < 0)
    {
      _err("mpmq_init() failure. %d\n", ret);
      errout_ret = SENSOR_DSP_LOAD_ERROR;
      goto step_counter_errout_with_mptask_destroy;
    }

  /* Release subcore. */

  ret = mptask_exec(&m_mptask);
  if (ret != 0)
    {
      _err("mptask_exec() failure. %d\n", ret);
      errout_ret = SENSOR_DSP_LOAD_ERROR;
      goto step_counter_errout_with_mpmq_destory;
    }

  /* Wait boot response event */

  id = mpmq_receive(&m_mq, &msgdata);
  if (id != DSP_BOOTED_CMD_ID)
    {
      _err("boot error! %d\n", id);
      errout_ret = SENSOR_DSP_BOOT_ERROR;
      goto step_counter_errout_with_mpmq_destory;
    }
  if (msgdata != DSP_AESM_VERSION)
    {
      _err("boot error! [dsp version:0x%x] [sensorutils version:0x%x]\n",
        msgdata, DSP_AESM_VERSION);
      errout_ret = SENSOR_DSP_VERSION_ERROR;
      goto step_counter_errout_with_mpmq_destory;
    }
  
  /* Send InitEvent and wait response. */

  ret = this->sendInit();
  if (ret != SENSOR_OK)
    {
      errout_ret = ret;
      goto step_counter_errout_with_mpmq_destory;
    }

  /* Create receive tread */

  ret = pthread_create(&m_thread_id, NULL,
                       receiver_thread_entry,
                       static_cast<pthread_addr_t>(this));
  if (ret != 0)
    {
      err("Failed to create receiver_thread_entry, error=%d\n", ret);
      errout_ret = SENSOR_TASK_CREATE_ERROR;
    }
  else
    {
      return SENSOR_OK;
    }

step_counter_errout_with_mpmq_destory:
  ret = mpmq_destroy(&m_mq);
  DEBUGASSERT(ret == 0);

step_counter_errout_with_mptask_destroy:
  ret = mptask_destroy(&m_mptask, false, NULL);
  DEBUGASSERT(ret == 0);

  return errout_ret;
}


/*--------------------------------------------------------------------*/
int StepCounterClass::close(void)
{
  int wret = -1;
  int ret = mptask_destroy(&m_mptask, false, &wret);
  if (ret < 0)
    {
      err("mptask_destroy() failure. %d\n", ret);
      return SENSOR_DSP_UNLOAD_ERROR;
    }

  _info("Worker exit status = %d\n", wret);

  pthread_cancel(this->m_thread_id);
  pthread_join(this->m_thread_id, NULL);

  /* Finalize all of MP objects */
  mpmq_destroy(&m_mq);

  return SENSOR_OK;
}

/*--------------------------------------------------------------------*/
int StepCounterClass::sendInit(void)
{
  /* tentative : Not absolutely necessary to use MemHandle. */

  MemMgrLite::MemHandle mh;
  if (mh.allocSeg(m_cmd_pool_id, sizeof(SensorDspCmd)) != ERR_OK)
    {
      return SENSOR_MEMHANDLE_ALLOC_ERROR;
    }

  SensorDspCmd* dsp_cmd = (SensorDspCmd*)mh.getPa();

  dsp_cmd->header.sensor_type = StepCounter;
  dsp_cmd->header.event_type  = InitEvent;

  int ret = mpmq_send(&m_mq, (AesmMode << 4) + (InitEvent << 1), reinterpret_cast<int32_t>(mh.getPa()));
  if (ret < 0)
    {
      _err("mpmq_send() failure. %d\n", ret);
      return SENSOR_DSP_INIT_ERROR;
    }

  /* wait for initialized event */

  uint32_t msgdata;

  int id = mpmq_receive(&m_mq, &msgdata);
  if ((id != STEPCOUNTER_CMD_ID) && (reinterpret_cast<SensorDspCmd*>(msgdata)->result.exec_result != SensorOK))
    {
      _err("init error! %08x : %d\n", id, reinterpret_cast<SensorDspCmd*>(msgdata)->result.exec_result);
      return SENSOR_DSP_INIT_ERROR;
    }

  return SENSOR_OK;
}

/*--------------------------------------------------------------------*/
int StepCounterClass::write(sensor_command_data_mh_t* command)
{
  struct exe_mh_s       exe_mh;

  /* copy memhandle of data */

  exe_mh.data = command->mh;

  /* allocate segment of command */

  if (exe_mh.cmd.allocSeg(m_cmd_pool_id, sizeof(SensorDspCmd)) != ERR_OK)
    {
      _err("allocSeg() failure.¥n");
      return SENSOR_MEMHANDLE_ALLOC_ERROR;
    }

  SensorDspCmd* dsp_cmd = (SensorDspCmd*)exe_mh.cmd.getPa();

  dsp_cmd->header.sensor_type = StepCounter;
  dsp_cmd->header.event_type  = ExecEvent;

  /* Select sensor type by incoming sensor. */

  switch (command->self)
    {
      case accelID:
        {
          dsp_cmd->exec_aesm_cmd.cmd_type                 = AESM_CMD_UPDATE_ACCELERATION;
          dsp_cmd->exec_aesm_cmd.update_acc.time_stamp    = command->time;
          dsp_cmd->exec_aesm_cmd.update_acc.sampling_rate = command->fs;
          dsp_cmd->exec_aesm_cmd.update_acc.sample_num    = command->size;
          dsp_cmd->exec_aesm_cmd.update_acc.p_data        = reinterpret_cast<ThreeAxisSample*>(exe_mh.data.getPa());
        }
        break;

      case gnssID:
        {
          dsp_cmd->exec_aesm_cmd.cmd_type = AESM_CMD_UPDATE_GPS;
          dsp_cmd->exec_aesm_cmd.update_gps = *(reinterpret_cast<GnssSampleData*>(exe_mh.data.getPa()));
        }
        break;

      default:
        {
          dsp_cmd->exec_aesm_cmd.cmd_type = AESM_CMD_UPDATE_ACCELERATION;
        }
        break;
    }

  if (!m_exe_que.push(exe_mh))
    {
      _err("m_exe_que.push() failure.¥n");
      return SENSOR_QUEUE_PUSH_ERROR;
    }

  /* Send sensored data. (* Data which sent to DSP is physical address of command msg.) */

  int ret = mpmq_send(&m_mq, (AesmMode << 4) + (ExecEvent << 1), reinterpret_cast<int32_t>(exe_mh.cmd.getPa()));
  if (ret < 0)
    {
      _err("mpmq_send() failure. %d\n", ret);
      return SENSOR_DSP_EXEC_ERROR;
    }

  return SENSOR_OK;
}

/*--------------------------------------------------------------------*/
void StepCounterClass::set_callback(void)
{
  struct exe_mh_s          exe_mh = m_exe_que.top();
  sensor_command_data_mh_t packet;

  /* Create command. */
  packet.header.code = SendData;
  packet.header.size = 4; /*tentative*/
  packet.self        = stepcounterID;
  packet.time        = 0;
  packet.fs          = 0;
  packet.size        = 3;
  packet.mh          = exe_mh.cmd;

  SF_SendSensorDataMH(&packet);

  /* Pop exec queue (Free segment). */

  m_exe_que.pop();
}

/*--------------------------------------------------------------------*/
/*!
 * @brief receive result from dsp
 */
int StepCounterClass::receive(void)
{
  int      command;
  uint32_t msgdata;
  bool     active = true;

  /* Wait for worker message */

  while (active)
    {
      command = mpmq_receive(&m_mq, &msgdata);
      if (command < 0)
        {
          _err("mpmq_receive() failure. command(%d) < 0\n", command);
          return command;
        }

      this->set_callback();
    }
  
  return 0;
}



