/****************************************************************************
 * modules/audio/objects/media_recorder/audio_rec_sink_task.cpp
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

#include <string.h>
#include "audio_rec_sink_task.h"
#include "wien2_internal_packet.h"
#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pid_t    s_rcd_snk_pid;
static MsgQueId s_self_dtq;
static MsgQueId s_recorder_dtq;
/* 当然クラスに隠蔽したい:TODO */

static AudioRecSinkTask *s_obj = NULL;
static RamSinkForAudio  *p_cur_instance = NULL;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void AudioRecSinkTask::run(void)
{
  err_t        err_code;
  MsgQueBlock* que;
  MsgPacket*   msg;

  err_code = MsgLib::referMsgQueBlock(m_self_dtq, &que);
  F_ASSERT(err_code == ERR_OK);

  while(1)
    {
      err_code = que->recv(TIME_FOREVER, &msg);
      F_ASSERT(err_code == ERR_OK);

      parse(msg);

      err_code = que->pop();
      F_ASSERT(err_code == ERR_OK);
    }
}

/*--------------------------------------------------------------------------*/
AudioRecSinkTask::MsgProc
  AudioRecSinkTask::MsgProcTbl[AUD_SNK_MSG_NUM][AudioRecSinkStateNum] =
{
  /* Message Type: MSG_AUD_SNK_INIT. */

  {                                       /* Sink status: */
    &AudioRecSinkTask::init,              /*   Ready.     */
    &AudioRecSinkTask::illegal,           /*   Active.    */
    &AudioRecSinkTask::illegal            /*   Overflow.  */
  },

  /* Message Type: MSG_AUD_SNK_DATA. */

  {                                       /* Sink status: */
    &AudioRecSinkTask::dataSinkOnReady,   /*   Ready.     */
    &AudioRecSinkTask::dataSinkOnActive,  /*   Active.    */
    &AudioRecSinkTask::dataSinkOnOverflow /*   Overflow.  */
  },

  /* Message Type: MSG_AUD_SNK_STOP. */

  {                                       /* Sink status: */
    &AudioRecSinkTask::illegal,           /*   Ready.     */
    &AudioRecSinkTask::stopOnActive,      /*   Active.    */
    &AudioRecSinkTask::stopOnActive       /*   Overflow.  */
  }
};

/*--------------------------------------------------------------------------*/
void AudioRecSinkTask::parse(MsgPacket *msg)
{
  uint32_t event = MSG_GET_SUBTYPE(msg->getType());
  F_ASSERT((event < AUD_SNK_MSG_NUM));

  (this->*MsgProcTbl[event][m_state])(msg);
}

/*--------------------------------------------------------------------------*/
void AudioRecSinkTask::illegal(MsgPacket *msg)
{
  uint32_t event = MSG_GET_SUBTYPE(msg->getType());
  switch (event)
    {
      case MSG_AUD_SNK_INIT:
        {
          msg->moveParam<InitAudioRecSinkParam_s>();
        }
        break;
      case MSG_AUD_SNK_DATA:
        {
          msg->moveParam<AudioRecSinkData_s>();
        }
        break;
      case MSG_AUD_SNK_STOP:
        break;
    }
  MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST);
  D_ASSERT(0);
}

/*--------------------------------------------------------------------------*/
void AudioRecSinkTask::init(MsgPacket *msg)
{
  InitAudioRecSinkParam_s cmd = msg->moveParam<InitAudioRecSinkParam_s>();
  err_t er;

  bool res = true;
  switch(cmd.output_device)
    {
      case AS_SETRECDR_STS_OUTPUTDEVICE_RAM:
        p_cur_instance = new RamSinkForAudio();
        break;
      default:
        res = false;
        D_ASSERT(0);
        break;
    }

  m_codec_type = cmd.codec_type;

  if (res)
    {
      if (!(p_cur_instance->init(cmd)))
        {
          return;
        }
    }
  er = MsgLib::send<bool>(s_recorder_dtq,
                          MsgPriNormal,
                          MSG_AUD_VRC_RST_RSINK_INIT,
                          s_self_dtq,
                          res);
  if (er)
    {
      F_ASSERT(0);
    }
}

/*--------------------------------------------------------------------------*/
void AudioRecSinkTask::dataSinkOnReady(MsgPacket *msg)
{
  bool res = true;
  err_t er;
  AudioRecSinkData_s cmd = msg->moveParam<AudioRecSinkData_s>();

  if (!(p_cur_instance->write(cmd)))
    {
      m_state = AudioRecSinkStateOverflow;
      res = false;
    }
  else
    {
      m_state = AudioRecSinkStateActive;
    }
  er = MsgLib::send<bool>(s_recorder_dtq,
                          MsgPriNormal,
                          MSG_AUD_VRC_RST_RSINK_DATA,
                          s_self_dtq,
                          res);
  if (er)
    {
      F_ASSERT(0);
    }
}

/*--------------------------------------------------------------------------*/
void AudioRecSinkTask::dataSinkOnActive(MsgPacket *msg)
{
  bool res = true;
  err_t er;
  AudioRecSinkData_s cmd = msg->moveParam<AudioRecSinkData_s>();

  if (!(p_cur_instance->write(cmd)))
    {
      m_state = AudioRecSinkStateOverflow;
      res = false;
    }
  er = MsgLib::send<bool>(s_recorder_dtq,
                          MsgPriNormal,
                          MSG_AUD_VRC_RST_RSINK_DATA,
                          s_self_dtq,
                          res);
  if (er)
    {
      F_ASSERT(0);
    }
}

/*--------------------------------------------------------------------------*/
void AudioRecSinkTask::dataSinkOnOverflow(MsgPacket *msg)
{
  AudioRecSinkData_s cmd = msg->moveParam<AudioRecSinkData_s>();
  err_t er = MsgLib::send<bool>(s_recorder_dtq,
                                MsgPriNormal,
                                MSG_AUD_VRC_RST_RSINK_DATA,
                                s_self_dtq,
                                true);
  if (er)
    {
      F_ASSERT(0);
    }
}

/*--------------------------------------------------------------------------*/
void AudioRecSinkTask::stopOnActive(MsgPacket *msg)
{
  bool res = true;
  if (!(p_cur_instance->finalize())) {
    res = false;
  }
  m_state = AudioRecSinkStateReady;

  err_t er = MsgLib::send<bool>(s_recorder_dtq,
                                MsgPriNormal,
                                MSG_AUD_VRC_RST_RSINK_STOP,
                                s_self_dtq,
                                res);
  if (er)
    {
      F_ASSERT(0);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C"
{
/*--------------------------------------------------------------------------*/
int AS_VoiceRecorderSinkEntry(int argc, char *argv[])
{
  AudioRecSinkTask::create(s_self_dtq);
  return 0;
}

/*--------------------------------------------------------------------------*/
bool AS_ActivateVoiceRecorderSink(FAR AsActRecorderSinkParam_t *param)
{
  s_self_dtq     = param->msgq_id.recorder_sink;
  s_recorder_dtq = param->msgq_id.recorder;

  s_rcd_snk_pid = task_create("REC_SINK",
                              150,
                              1024 * 2,
                              AS_VoiceRecorderSinkEntry,
                              NULL);
  if (s_rcd_snk_pid < 0)
    {
      _err("ERROR AS_ActivateVoiceRecorderSink failed\n");
      return false;
    }
  /*タスクのパラメータは、KConfigへ:TODO */

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeactivateVoiceRecorderSink(void)
{
  task_delete(s_rcd_snk_pid);
  s_obj = NULL;
  return true;
}

} /* extern "C" */

/*--------------------------------------------------------------------------*/
void AudioRecSinkTask::create(MsgQueId self_dtq)
{
  if(s_obj == NULL)
    {
      s_obj = new AudioRecSinkTask(self_dtq);
      s_obj->run();
    }
  else
    {
      F_ASSERT(0);
    }
}

__WIEN2_END_NAMESPACE

