/****************************************************************************
 * modules/audio/objects/media_recorder/audio_rec_sink_task.h
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

#ifndef __MODULES_AUDIO_OBJECTS_MEDIA_RECORDER_AUDIO_REC_SINK_TASK_H
#define __MODULES_AUDIO_OBJECTS_MEDIA_RECORDER_AUDIO_REC_SINK_TASK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include "memutils/os_utils/chateau_osal.h"
#include "memutils/message/Message.h"
#include "ram_sink_for_audio.h"
#include "common/audio_message_types.h"
#include "wien2_common_defs.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/
class AudioRecSinkTask {
public:
  static void create(MsgQueId self_dtq);

private:
  enum AudioRecSinkState
  {
    AudioRecSinkStateReady = 0,
    AudioRecSinkStateActive,
    AudioRecSinkStateOverflow,
    AudioRecSinkStateNum
  };

  MsgQueId          m_self_dtq;
  AudioRecSinkState m_state;
  AudioCodec        m_codec_type;
  AudioRecSinkTask *m_cur_instance;

  typedef void (AudioRecSinkTask::*MsgProc)(MsgPacket*);
  static MsgProc MsgProcTbl[AUD_SNK_MSG_NUM][AudioRecSinkStateNum];

  AudioRecSinkTask(MsgQueId self_dtq):
    m_self_dtq(self_dtq),
    m_state(AudioRecSinkStateReady),
    m_codec_type(InvalidCodecType)
  {}

  ~AudioRecSinkTask() {}
  void run();
  void parse(MsgPacket*);
  void illegal(MsgPacket*);
  void init(MsgPacket*);
  void dataSinkOnReady(MsgPacket*);
  void dataSinkOnActive(MsgPacket*);
  void dataSinkOnOverflow(MsgPacket*);
  void stopOnActive(MsgPacket*);
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

__WIEN2_END_NAMESPACE

#endif /* __MODULES_AUDIO_OBJECTS_MEDIA_RECORDER_AUDIO_REC_SINK_TASK_H */

