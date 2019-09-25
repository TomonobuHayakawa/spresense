/****************************************************************************
 * modules/audio/components/oscillator/oscillator_component.h
 *
 *   Copyright 2019 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#ifndef WIEN2_OSCILLATOR_COMPONENT_H
#define WIEN2_OSCILLATOR_COMPONENT_H

#include "memutils/os_utils/chateau_osal.h"

#include "wien2_common_defs.h"
#include "memutils/s_stl/queue.h"
#include "apus/apu_cmd.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"
#include "debug/dbg_log.h"
#include "components/common/component_common.h"

using namespace MemMgrLite;

__WIEN2_BEGIN_NAMESPACE

typedef uint32_t OscllicatorComponentHandler;

struct ExecOscParam
{
  BufferHeader  buffer;
  uint8_t       channel_no;     /**< Channel number of data */ /* 変更したいチャンネル番号 */
  void*         cp;
  uint32_t      size;
};

struct SetOscParam
{
  uint8_t           channel_no;
  uint32_t          frequency;
};

struct OscCmpltParam
{
  uint32_t event_type;
  bool     result;

  union
  {
    ExecOscParam exec_osc_param;
    SetOscParam  set_osc_param;
  };
};

typedef bool (*OscCompCallback)(OscCmpltParam*, void*);

struct InitOscParam
{
  WaveMode          type;
  uint8_t           channel_num;
  AudioPcmBitWidth  bit_length;
  uint32_t          sampling_rate;
  OscCompCallback   callback;
  void             *instance;
};

class OscillatorComponent : public ComponentCommon<Apu::InternalResult>
{
public:
  OscillatorComponent(MsgQueId apu_dtq, PoolId apu_pool_id)
  {
    m_callback    = NULL;
    m_apu_dtq     = apu_dtq;
    m_apu_pool_id = apu_pool_id;
  }
  ~OscillatorComponent() {}

  uint32_t activate(MsgQueId apu_dtq, PoolId apu_pool_id, const char *path, uint32_t *dsp_inf);
  bool     deactivate();
  uint32_t init(const InitOscParam& param, uint32_t *dsp_inf);
  bool     exec(const ExecOscParam& param);
  bool     flush(void);
  bool     set(const SetOscParam& param);
  bool     recv(void *p_param);
  bool     done(void)
  {
    if (!m_apu_cmd_mh_que.pop())
      {
        OSCILLATOR_CMP_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
        return false;
      }
    return true;
  }
  void    *getApuCmdBuf()
  {
    MemMgrLite::MemHandle mh;

    if (mh.allocSeg(m_apu_pool_id, sizeof(Apu::Wien2ApuCmd)) != ERR_OK)
      {
        OSCILLATOR_CMP_WARN(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
        return NULL;
      }

    if (!m_apu_cmd_mh_que.push(mh))
      {
        OSCILLATOR_CMP_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
        return NULL;
      }

    return mh.getPa();
  }
  MsgQueId get_apu_mid(void) { return m_apu_dtq; };

private:
  typedef s_std::Queue<MemMgrLite::MemHandle, APU_COMMAND_QUEUE_SIZE> ApuCmdMhQue;

  ApuCmdMhQue     m_apu_cmd_mh_que;
  MsgQueId        m_apu_dtq;
  PoolId          m_apu_pool_id;
  OscCompCallback m_callback;
  void           *m_instance;
  void           *m_dsp_handler;

  void  send_apu(Apu::Wien2ApuCmd*);
};

__WIEN2_END_NAMESPACE

#endif /* WIEN2_OSCILLATOR_COMPONENT_H */

