/****************************************************************************
 * modules/audio/objects/output_mixer/output_mix_sink_device.h
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

#ifndef __MODULES_AUDIO_OBJECTS_OUTPUT_OUTPUT_MIXER_OUTPUT_MIX_SINK_DEVICE_H
#define __MODULES_AUDIO_OBJECTS_OUTPUT_OUTPUT_MIXER_OUTPUT_MIX_SINK_DEVICE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "memutils/message/Message.h"
#include "memutils/message/MsgPacket.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/s_stl/queue.h"
#include "memutils/simple_fifo/CMN_SimpleFifo.h"
#include "memutils/common_utils/common_assert.h"
#include "common/audio_message_types.h"
#include "wien2_internal_packet.h"
#include "wien2_common_defs.h"
#include "components/renderer/renderer_component.h"
#include "objects/stream_parser/ram_lpcm_data_source.h"
#include "objects/stream_parser/mp3_stream_mng.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

class OutputMixToHPI2S
{
public:
  OutputMixToHPI2S():
    m_state(AS_MODULE_ID_OUTPUT_MIX_OBJ, "", Booted),
    m_adjust_direction(OutputMixNoAdjust),
    m_adjustment_times(0)
    {}

    MsgQueId m_self_dtq, m_player_dtq;
    int m_self_handle;

  ~OutputMixToHPI2S() {}
  void parse(MsgPacket* msg);

  bool is_active(void)
    {
      return ((m_state.get() != Booted) ? true : false);
    }
  void set_self_handle(int handle)
    {
      m_self_handle = handle;
    }
  void set_self_dtq(MsgQueId dtqid)
    {
      m_self_dtq = dtqid;
    }

private:
  enum State
  {
    Booted = 0,
    Ready,
    Active,
    Stopping,
    StateNum
  };

  AudioState<State> m_state;

  typedef void (OutputMixToHPI2S::*MsgProc)(MsgPacket *);
  static MsgProc MsgProcTbl[AUD_MIX_MSG_NUM][StateNum];

  typedef s_std::Queue<OutputMixObjInputDataCmd, 10> RenderDataQueue;
  RenderDataQueue m_render_data_queue;

  RenderComponentHandler m_render_comp_handler;

  int8_t m_adjust_direction;
  int32_t m_adjustment_times;

  void illegal(MsgPacket *msg);
  void act(MsgPacket *msg);
  void deact(MsgPacket *msg);

  void input_data_on_ready(MsgPacket *msg);
  void input_data_on_active(MsgPacket *msg);

  void stop_on_active(MsgPacket *msg);

  void illegal_done(MsgPacket *msg);
  void done_on_active(MsgPacket *msg);
  void done_on_stopping(MsgPacket *msg);

  void clock_recovery(MsgPacket *msg);

  void parseOutputMixRst(MsgPacket *msg);

  int8_t get_period_adjustment(void);
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

#endif /* __MODULES_AUDIO_OBJECTS_OUTPUT_OUTPUT_MIXER_OUTPUT_MIX_SINK_DEVICE_H */
