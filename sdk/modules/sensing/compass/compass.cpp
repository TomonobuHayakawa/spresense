/****************************************************************************
 * sensing/compass/compass.cpp
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/time.h>
#include <debug.h>

#include "sensing/compass.h"
#include "sensing/sensor_dsp_command.h"
#include "dsp_sensor_version.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define COMPASS_MQ_ID       1
#define DSP_BOOTED_CMD_ID   1
#define COMPASS_CMD_ID      2

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void *receiver_thread_entry(FAR void *p_instance)
{
  ((CompassClass*)p_instance)->receive();

  pthread_exit(0);
  return 0;
}

/*--------------------------------------------------------------------*/
int CompassClass::open(void)
{
  int errout_ret;
  int ret;
  int id;
  uint32_t msgdata;

  /* Initalize Worker as task. */

  ret = mptask_init_secure(&m_mptask, "ORIENTATION"/* tentative *//*filename*/);
  if (ret != 0)
    {
      _err("mptask_init() failure. %d\n", ret);
      return SS_ECODE_DSP_LOAD_ERROR;
    }

  ret = mptask_assign(&m_mptask);
  if (ret != 0)
    {
      _err("mptask_asign() failure. %d\n", ret);
      return SS_ECODE_DSP_LOAD_ERROR;
    }

  /* Queue for communication between Supervisor and Worker create. */

  ret = mpmq_init(&m_mq, COMPASS_MQ_ID, mptask_getcpuid(&m_mptask));
  if (ret < 0)
    {
      _err("mpmq_init() failure. %d\n", ret);
      errout_ret = SS_ECODE_DSP_LOAD_ERROR;
      goto compass_errout_with_mptask_destroy;
    }

  /* Release subcore. */

  ret = mptask_exec(&m_mptask);
  if (ret != 0)
    {
      _err("mptask_exec() failure. %d\n", ret);
      errout_ret = SS_ECODE_DSP_LOAD_ERROR;
      goto compass_errout_with_mpmq_destory;
    }

  /* Wait boot response event. */

  id = mpmq_receive(&m_mq, &msgdata);
  if (id != DSP_BOOTED_CMD_ID)
    {
      _err("boot error! %d\n", id);
      errout_ret = SS_ECODE_DSP_BOOT_ERROR;
      goto compass_errout_with_mpmq_destory;
    }
  if (msgdata != DSP_ORIENTATION_VERSION)
    {
      _err("boot error! [dsp version:0x%x] [sensing version:0x%x]\n",
          msgdata, DSP_ORIENTATION_VERSION);
      errout_ret = SS_ECODE_DSP_VERSION_ERROR;
      goto compass_errout_with_mpmq_destory;
    }

  /* Send InitEvent and wait response. */

  ret = this->sendInit();
  if (ret != SS_ECODE_OK)
    {
      errout_ret = ret;
      goto compass_errout_with_mpmq_destory;
    }

  /* Create receive tread. */

  ret = pthread_create(&m_thread_id,
                       NULL,
                       receiver_thread_entry,
                       static_cast<pthread_addr_t>(this));
  if (ret != 0)
    {
      _err("Failed to create receiver_thread_entry, error=%d\n", ret);
      errout_ret = SS_ECODE_TASK_CREATE_ERROR;
    }
  else
    {
      return SS_ECODE_OK;
    }

compass_errout_with_mpmq_destory:
  ret = mpmq_destroy(&m_mq);
  DEBUGASSERT(ret == 0);

compass_errout_with_mptask_destroy:
  ret = mptask_destroy(&m_mptask, false, NULL);
  DEBUGASSERT(ret == 0);

  return errout_ret;
}


/*--------------------------------------------------------------------*/
int CompassClass::close(void)
{

  int wret = -1;
  int ret = 0;

  ret = mptask_destroy(&m_mptask, false, &wret);
  if (ret < 0)
    {
      _err("mptask_destroy() failure. %d\n", ret);
      return SS_ECODE_DSP_UNLOAD_ERROR;
    }

  ret = pthread_cancel(this->m_thread_id);
  pthread_join(this->m_thread_id, NULL);

  /* Finalize all of MP objects. */

  mpmq_destroy(&m_mq);

  return SS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
int CompassClass::sendInit(void)
{
  /* Tentative : Not absolutely necessary to use MemHandle. */

  MemMgrLite::MemHandle mh;
  if (mh.allocSeg(m_cmd_pool_id, sizeof(SensorDspCmd)) != ERR_OK)
    {
      return SS_ECODE_MEMHANDLE_ALLOC_ERROR;
    }

  SensorDspCmd* dsp_cmd = (SensorDspCmd*)mh.getVa();

  dsp_cmd->header.sensor_type = Compass;
  dsp_cmd->header.event_type  = InitEvent;

  int ret = mpmq_send(&m_mq,
                      (CompassProcMode << 4) + (InitEvent << 1),
                      reinterpret_cast<int32_t>(mh.getPa()));
  if (ret < 0)
    {
      _err("mpmq_send() failure. %d¥n", ret);
      return SS_ECODE_DSP_INIT_ERROR;
    }

  /* Wait for initialized event. */

  uint32_t msgdata;

  int id = mpmq_receive(&m_mq, &msgdata);
  if ((id != COMPASS_CMD_ID) &&
      (reinterpret_cast<SensorDspCmd*>(msgdata)->result.exec_result != SensorOK))
    {
      _err("init error! %08x : %d\n",
          id, reinterpret_cast<SensorDspCmd*>(msgdata)->result.exec_result);
      return SS_ECODE_DSP_INIT_ERROR;
    }

  return SS_ECODE_OK;
}

int CompassClass::sendFlush(void)
{
  /* Tentative : Not absolutely necessary to use MemHandle. */

  MemMgrLite::MemHandle mh;
  if (mh.allocSeg(m_cmd_pool_id, sizeof(SensorDspCmd)) != ERR_OK)
    {
      return SS_ECODE_MEMHANDLE_ALLOC_ERROR;
    }

  SensorDspCmd* dsp_cmd = (SensorDspCmd*)mh.getVa();

  dsp_cmd->header.sensor_type = Compass;
  dsp_cmd->header.event_type  = FlushEvent;

  int ret = mpmq_send(&m_mq,
                      (CompassProcMode << 4) + (InitEvent << 1),
                      reinterpret_cast<int32_t>(mh.getPa()));
  if (ret < 0)
    {
      _err("mpmq_send() failure. %d¥n", ret);
      return SS_ECODE_DSP_EXEC_ERROR;
    }

  /* Wait for finalize event. */

  uint32_t msgdata;

  int id = mpmq_receive(&m_mq, &msgdata);
  if ((id != COMPASS_CMD_ID) &&
      (reinterpret_cast<SensorDspCmd*>(msgdata)->result.exec_result != SensorOK))
    {
      _err("init error! %08x : %d\n",
          id, reinterpret_cast<SensorDspCmd*>(msgdata)->result.exec_result);
      return SS_ECODE_DSP_EXEC_ERROR;
    }

  return SS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
int CompassClass::write(sensor_command_data_mh_t* command)
{
  ThreeAxisSampleData*  p_three_axis;
  struct exe_mh_s       exe_mh;

  /* Copy memhandle of data. */

  exe_mh.data = command->mh;

  /* Allocate segment of command. */

  if (exe_mh.cmd.allocSeg(m_cmd_pool_id, sizeof(SensorDspCmd)) != ERR_OK)
    {
      return SS_ECODE_MEMHANDLE_ALLOC_ERROR;
    }

  SensorDspCmd* dsp_cmd = (SensorDspCmd*)exe_mh.cmd.getVa();

  dsp_cmd->header.sensor_type = Compass;
  dsp_cmd->header.event_type  = ExecEvent;

  /* Select sensor type by incoming sensor. */

  switch (command->self)
    {
      case accelID:
        {
          dsp_cmd->exec_orientation_cmd.cmd_type = ORIENTATION_CMD_UPDATE_ACCEL;
          if (!m_accel_exe_que.push(exe_mh)) /* Cannot save MHandle due to system error. */
            {
              return SS_ECODE_QUEUE_PUSH_ERROR;
            }
        }
        break;

      case magID:
        {
          dsp_cmd->exec_orientation_cmd.cmd_type = ORIENTATION_CMD_UPDATE_MAG;
          if (!m_mag_exe_que.push(exe_mh)) /* Cannot save MHandle due to system error. */
            {
              return SS_ECODE_QUEUE_PUSH_ERROR;
            }
        }
        break;

      default:
        {
          dsp_cmd->exec_orientation_cmd.cmd_type = ORIENTATION_CMD_UPDATE_ACCEL;
        }
        break;
    }

  p_three_axis                = &dsp_cmd->exec_orientation_cmd.update_three_axis;
  p_three_axis->time_stamp    = command->time;
  p_three_axis->sampling_rate = command->fs;
  p_three_axis->sample_num    = command->size;
  p_three_axis->p_data        = reinterpret_cast<ThreeAxisSample*>(exe_mh.data.getPa());

  /* Send sensored data.
   * (Data which sent to DSP is physical address of command msg.)
   */

  int ret = mpmq_send(&m_mq,
                      (CompassProcMode << 4) + (ExecEvent << 1),
                      reinterpret_cast<int32_t>(exe_mh.cmd.getPa()));
  if (ret < 0)
    {
      _err("mpmq_send() failure. %d¥n", ret);
      return SS_ECODE_DSP_EXEC_ERROR;
    }

  return SS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
void CompassClass::set_callback(uint32_t msgdata)
{
  sensor_command_data_mh_t packet;
  MemMgrLite::MemHandle    mh;
  CompassResult*           p_result = NULL;
  OrientationCmdType       cmdtype =
    (reinterpret_cast<SensorDspCmd*>(msgdata)->exec_orientation_cmd.cmd_type);

  packet.header.code = SendData;
  packet.header.size = 4; /*tentative*/
  packet.self        = compassID;
  packet.time        = 0;
  packet.fs          = 0;
  packet.size        = 3;

  F_ASSERT(mh.allocSeg(m_rst_pool_id, sizeof(CompassResult)) == ERR_OK);

  p_result = reinterpret_cast<CompassResult*>(mh.getVa());

  p_result->resultcmd   = 0;

  switch (cmdtype)
    {
    case ORIENTATION_CMD_UPDATE_ACCEL:
      {
        struct exe_mh_s accel_mh = m_accel_exe_que.top();
        SensorDspCmd* dsp_cmd = reinterpret_cast<SensorDspCmd*>(accel_mh.cmd.getVa());

        p_result->resultcmd   = ORIENTATION_CMD_UPDATE_ACCEL;
        p_result->exec_result = dsp_cmd->result.exec_result;
        p_result->errcode     = dsp_cmd->result.assert_info.code;

        packet.mh          = mh;

        SF_SendSensorDataMH(&packet);

        /* Pop exec queue (Free segment). */
        m_accel_exe_que.pop();

      }
      break;

    case ORIENTATION_CMD_UPDATE_MAG:
      {

        struct exe_mh_s mag_mh = m_mag_exe_que.top();
        SensorDspCmd* dsp_cmd = reinterpret_cast<SensorDspCmd*>(mag_mh.cmd.getVa());

        p_result->resultcmd   = ORIENTATION_CMD_UPDATE_MAG;
        p_result->exec_result = dsp_cmd->result.exec_result;
        p_result->errcode     = dsp_cmd->result.assert_info.code;
        p_result->azimuth     = dsp_cmd->result.result_orientation.azimuth;
        p_result->pitch       = dsp_cmd->result.result_orientation.pitch;
        p_result->roll        = dsp_cmd->result.result_orientation.roll;
        p_result->decl        = dsp_cmd->result.result_orientation.decl;
        p_result->acc_azimuth = dsp_cmd->result.result_orientation.acc_azimuth;
        p_result->calib_lv    = dsp_cmd->result.result_orientation.calib_lv;

        packet.mh          = mh;

        SF_SendSensorDataMH(&packet);

        /* Pop exec queue (Free segment). */

        m_mag_exe_que.pop();

      }
      break;
    default:
      /* Do nothing. */
      break;
    }

}

/*--------------------------------------------------------------------*/
/*!
 * @brief receive result from dsp
 */
int CompassClass::receive(void)
{
  int      command;
  uint32_t msgdata;
  bool     active = true;

  /* Wait for worker message. */

  while (active)
    {
      command = mpmq_receive(&m_mq, &msgdata);
      if (command < 0)
        {
          _err("mpmq_recieve() failure. command(%d) < 0\n", command);
          return command;
        }

      this->set_callback(msgdata);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

CompassClass* CompassCreate(MemMgrLite::PoolId cmd_pool_id, MemMgrLite::PoolId rst_pool_id)
{
  return new CompassClass(cmd_pool_id, rst_pool_id);
}

int CompassOpen(FAR CompassClass *ins)
{
  int ret;

  ret = ins->open();

  return ret;
}

int CompassClose(FAR CompassClass *ins)
{
  int ret;

  ret = ins->close();
  delete ins;

  return ret; 
}

int CompassWrite(FAR CompassClass *ins, FAR sensor_command_data_mh_t *command)
{
  int ret;

  ret = ins->write(command);

  return ret;
}

