/****************************************************************************
 * sensing/voice_detection/voice_detection.cpp
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

#include "sensing/voice_detection.h"
#include "sensing/sensor_dsp_command.h"
#include "apus/apu_cmd.h"            /* TODO remove */
#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VOICEDETECTION_MQ_ID     1

#define DSP_BOOTED_CMD_ID        1
#define VOICEDETECTION_CMD_ID    2

#define err  printf

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Default paramater of VAD. */

uint32_t VADCoef_table[16] =
{
  0x00010007, 0x13330b24, 0x0000030a, 0x73200000,
  0x2800645c, 0x0b245500, 0x030a1333, 0x00000000,
  0x645c7320, 0x55002800, 0x00000000, 0x00000000,
  0x00000000, 0x00000000, 0x00000000, 0x00000000
};
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR void *voice_detection_receive(FAR void *p_instance)
{
  ((FAR VoiceDetectionClass *)p_instance)->receive();

  pthread_exit(0);
  return 0;
}

/* VoiceDetection Class */

/*-------------------------------------------------------------------------*/
int VoiceDetectionClass::open(void)
{
  int errout_ret;
  int ret;
  int id;
  uint32_t msgdata; /* 0 is nomal boot. */

  /* Initalize Worker as task. */

  ret = mptask_init_secure(&m_mptask, "VADWUW");

  if (ret != 0)
    {
      _err(("mptask_init_secure() failure. %d\n", ret);
      return SS_ECODE_DSP_LOAD_ERROR;
    }

  ret = mptask_assign(&m_mptask);

  if (ret != 0)
    {
      _err(("mptask_asign() failure. %d\n", ret);
      return SS_ECODE_DSP_LOAD_ERROR;
    }

  /* Queue for communication between Supervisor and Worker create. */

  ret = mpmq_init(&m_mq, VOICEDETECTION_MQ_ID, mptask_getcpuid(&m_mptask));
  if (ret < 0)
    {
      _err(("mpmq_init() failure. %d\n", ret);
      errout_ret = SS_ECODE_DSP_LOAD_ERROR;
      goto voice_detection_errout_with_mptask_destroy;
    }

  /* Release subcore. */

  ret = mptask_exec(&m_mptask);
  if (ret != 0)
    {
      _err(("mptask_exec() failure. %d\n", ret);
      errout_ret = SS_ECODE_DSP_LOAD_ERROR;
      goto voice_detection_errout_with_mpmq_destory;
    }

  /* Wait boot response event */

  id = mpmq_receive(&m_mq, &msgdata);
  if ((id != DSP_BOOTED_CMD_ID) && (msgdata != 0))
    {
      _err(("boot error! %d\n", ret);
      errout_ret = SS_ECODE_DSP_BOOT_ERROR;
      goto voice_detection_errout_with_mpmq_destory;
    }

  /* Send InitEvent and wait response. */

  ret = this->sendInit();
  if (ret != 0)
    {
      _err(("sendInit() failure. %d\n", ret);
      errout_ret = ret;
      goto voice_detection_errout_with_mpmq_destory;
    }

  /* Create receive tread */

  ret = pthread_create(&m_thread_id,
                       NULL,
                       voice_detection_receive,
                       static_cast<pthread_addr_t>(this));
  if (ret != 0)
    {
      err("Failed to create receiver_thread_entry, error=%d\n", ret);
      errout_ret = SS_ECODE_TASK_CREATE_ERROR;
    }
  else
    {
      return SS_ECODE_OK;
    }

voice_detection_errout_with_mpmq_destory:
  ret = mpmq_destroy(&m_mq);
  DEBUGASSERT(ret == 0);

voice_detection_errout_with_mptask_destroy:
  ret = mptask_destroy(&m_mptask, false, NULL);
  DEBUGASSERT(ret == 0);

  return errout_ret;
}

/*-------------------------------------------------------------------------*/
int VoiceDetectionClass::close(void)
{
  int wret = -1;
  int ret = mptask_destroy(&m_mptask, false, &wret);
  if (ret < 0)
  {
    err("mptask_destroy() failure. %d\n", ret);
    return SS_ECODE_DSP_UNLOAD_ERROR;
  }

  _info("Worker exit status = %d\n", wret);

  pthread_cancel(this->m_thread_id);
  pthread_join(this->m_thread_id, NULL);

  /* Finalize all of MP objects */
  mpmq_destroy(&m_mq);

  return SS_ECODE_OK;
}

/*-------------------------------------------------------------------------*/
int VoiceDetectionClass::sendInit(void)
{
  /* Tentative : Not absolutely necessary to use MemHandle. */

  MemMgrLite::MemHandle mh;

  /* TODO use SensorDspCmd */

  if (mh.allocSeg(m_cmd_pool_id, sizeof(Wien2::Apu::Wien2ApuCmd)) != ERR_OK)
    {
      return SS_ECODE_MEMHANDLE_ALLOC_ERROR;
    }

  Wien2::Apu::Wien2ApuCmd* dsp_cmd = (Wien2::Apu::Wien2ApuCmd*)mh.getPa();

  dsp_cmd->header.process_mode                   = Wien2::Apu::RecognitionMode;
  dsp_cmd->header.event_type                     = Wien2::Apu::InitEvent;
  dsp_cmd->init_recognition_cmd.recognition_type = Wien2::Apu::VadWuwsr;
  dsp_cmd->init_recognition_cmd.sampling_rate    = Wien2::AudFs_16000;
  dsp_cmd->init_recognition_cmd.p_vad_param      = (uint8_t*)VADCoef_table;
  dsp_cmd->init_recognition_cmd.debug_dump_info.addr = NULL;
  dsp_cmd->init_recognition_cmd.debug_dump_info.size = 0;

  int ret = mpmq_send(&m_mq,
                     (Wien2::Apu::RecognitionMode << 4) +
                      (Wien2::Apu::InitEvent << 1),
                      reinterpret_cast<int32_t>(mh.getPa()));
  if (ret < 0)
    {
      _err("mpmq_send() failure. %d¥n", ret);
      return SS_ECODE_DSP_INIT_ERROR;
    }

  /* Wait for initialized event */

  uint32_t msgdata;

  int id = mpmq_receive(&m_mq, &msgdata);
  if ((id != VOICEDETECTION_CMD_ID) &&
      (reinterpret_cast<Wien2::Apu::Wien2ApuCmd *>
        (msgdata)->result.exec_result != Wien2::Apu::ApuExecOK))
    {
      _err("init error! %08x : %d\n",
            id,
            reinterpret_cast<Wien2::Apu::Wien2ApuCmd *>
              (msgdata)->result.exec_result);
      return SS_ECODE_DSP_INIT_ERROR;
    }

  return SS_ECODE_OK;
}

/*-------------------------------------------------------------------------*/
int VoiceDetectionClass::write(FAR sensor_command_data_mh_t *command)
{
  /* Copy memhandle of data */

  struct exe_mh_s exe_mh;
  exe_mh.data = command->mh;

  /* Allocate segment of command */

  if (exe_mh.cmd.allocSeg(
        m_cmd_pool_id,
        sizeof(Wien2::Apu::Wien2ApuCmd)) != ERR_OK)
    {
      return SS_ECODE_MEMHANDLE_ALLOC_ERROR;
    }

  Wien2::Apu::Wien2ApuCmd* dsp_cmd =
    (Wien2::Apu::Wien2ApuCmd*)exe_mh.cmd.getPa();
  static long unsigned int isFound;

  dsp_cmd->header.process_mode = Wien2::Apu::RecognitionMode;
  dsp_cmd->header.event_type   = Wien2::Apu::ExecEvent;
  dsp_cmd->exec_recognition_cmd.input_buffer.size      = command->size;
  dsp_cmd->exec_recognition_cmd.input_buffer.p_buffer  =
    reinterpret_cast<FAR unsigned long*>(exe_mh.data.getPa());
  dsp_cmd->exec_recognition_cmd.output_buffer.size     = 0;
  dsp_cmd->exec_recognition_cmd.output_buffer.p_buffer = &isFound;

  /* Send driver data */

  if (!m_exe_que.push(exe_mh))
    {
      return SS_ECODE_QUEUE_PUSH_ERROR;
    }

  int ret = mpmq_send(&m_mq,
                      (Wien2::Apu::RecognitionMode << 4) +
                        (Wien2::Apu::ExecEvent << 1),
                      reinterpret_cast<int32_t>(exe_mh.cmd.getPa()));
  if (ret < 0)
    {
      _err("mpmq_send() failure. %d¥n", ret);
      return SS_ECODE_DSP_EXEC_ERROR;
    }

  return SS_ECODE_OK;
}

/*-------------------------------------------------------------------------*/
void VoiceDetectionClass::set_callback(FAR SensorDspCmd *p_cmd)
{
  bool detected = false;
 
  struct exe_mh_s exe_mh = m_exe_que.top();

  Wien2::Apu::Wien2ApuCmd* result_data =
    reinterpret_cast<Wien2::Apu::Wien2ApuCmd*>(exe_mh.cmd.getVa());

  if (Wien2::Apu::ApuExecOK == result_data->result.exec_result)
    {
      uint32_t result = *result_data->exec_recognition_cmd.output_buffer.p_buffer;
      detected = (result & 0x100) != 0;
    }
  else
    {
      /* Detailed error code is set for result_data->result.code. */

      err("voice detection error: %d, %d, %d\n",
           result_data->result.exec_result,
           result_data->result.internal_result[0].res_src,
           result_data->result.internal_result[0].code);
      
      /* TODO: Notify result to Application */
    }

  /* Create command. */

  sensor_command_data_t packet;
  packet.header.code = SendData;
  packet.header.size = 4; /* tentative */
  packet.self        = vadID;
  packet.time        = 0;
  packet.fs          = 0;
  packet.size        = 3;
  packet.is_ptr      = 0;
  packet.data        = static_cast<uint32_t>(detected);

  SF_SendSensorData(&packet);

  /* Pop exec queue (Free segment). */

  m_exe_que.pop();
}

/*-------------------------------------------------------------------------*/
/*!
 * @brief receive result from dsp
 */
int VoiceDetectionClass::receive(void)
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

      this->set_callback(reinterpret_cast<FAR SensorDspCmd *>(msgdata));
    }

  return SS_ECODE_OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR VoiceDetectionClass *VoiceDetectionCreate(MemMgrLite::PoolId cmd_pool_id)
{
  return new VoiceDetectionClass(cmd_pool_id);
}

/*-------------------------------------------------------------------------*/
int VoiceDetectionOpen(FAR VoiceDetectionClass *ins)
{
  int ret;

  ret = ins->open();

  return ret;
}

/*-------------------------------------------------------------------------*/
int VoiceDetectionClose(FAR VoiceDetectionClass *ins)
{
  int ret;

  ret = ins->close();
  delete ins;

  return ret;
}

/*-------------------------------------------------------------------------*/
int VoiceDetectionWrite(FAR VoiceDetectionClass *ins,
                        FAR sensor_command_data_mh_t *command)
{
  int ret;

  ret = ins->write(command);

  return ret;
}

