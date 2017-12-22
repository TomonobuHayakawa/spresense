/***********************************************************************
 *
 *      File Name: encoder_component.h
 *
 *      Description: Header file of encoder component
 *
 *      Notes: (C) Copyright 2015 Sony Corporation
 *
 *      Author: Hsingying Ho
 *
 ***********************************************************************
 */

#ifndef WIEN2_ENCODER_COMPONENT_H
#define WIEN2_ENCODER_COMPONENT_H

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

typedef bool (*EncCompCallback)(void*);

struct InitEncParam
{
  AudioCodec       codec_type;
  uint32_t         input_sampling_rate;
  uint32_t         output_sampling_rate;
  AudioPcmBitWidth bit_width;
  uint8_t          channel_num;
  uint8_t          complexity;
  uint32_t         bit_rate;
  EncCompCallback  callback;
};

struct ExecEncParam
{
  BufferHeader input_buffer;
  BufferHeader output_buffer;
};

struct StopEncParam
{
  BufferHeader output_buffer;
};

struct EncCmpltParam
{
  Apu::ApuEventType event_type;

  union
  {
    InitEncParam init_enc_cmplt;
    ExecEncParam exec_enc_cmplt;
    StopEncParam stop_enc_cmplt;
  };
};

extern "C" {

uint32_t AS_encode_activate(AudioCodec param,MsgQueId apu_dtq, PoolId apu_pool_id, uint32_t *dsp_inf);
bool AS_encode_deactivate(void);
uint32_t AS_encode_init(const InitEncParam&, uint32_t *dsp_inf);
bool AS_encode_exec(const ExecEncParam&);
bool AS_encode_stop(const StopEncParam&);
bool AS_encode_recv_apu(void *p_param);
bool AS_encode_recv_done(void);

} /* extern "C" */

class EncoderComponent : public ComponentCommon
{
public:
  EncoderComponent(MsgQueId apu_dtq,PoolId apu_pool_id)
  {
    m_apu_dtq = apu_dtq;
    m_apu_pool_id = apu_pool_id;
  }
  ~EncoderComponent() {}

  uint32_t activate_apu(AudioCodec param, uint32_t *dsp_inf);
  bool deactivate_apu();
  uint32_t init_apu(const InitEncParam& param, uint32_t *dsp_inf);
  bool exec_apu(const ExecEncParam& param);
  bool flush_apu(const StopEncParam& param);
  bool recv_apu(void *p_param);
  bool recv_done(void) { return freeApuCmdBuf(); };
  MsgQueId get_apu_mid(void) { return m_apu_dtq; };

private:
  typedef s_std::Queue<MemMgrLite::MemHandle, APU_COMMAND_QUEUE_SIZE> ApuCmdMhQue;
  ApuCmdMhQue m_apu_cmd_mh_que;

  MsgQueId m_apu_dtq;
  PoolId m_apu_pool_id;

  void send_apu(Apu::Wien2ApuCmd*);
  EncCompCallback m_callback;

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  DebugLogInfo m_debug_log_info;
#endif

  void* getApuCmdBuf()
  {
    MemMgrLite::MemHandle mh;

    if (mh.allocSeg(m_apu_pool_id, sizeof(Apu::Wien2ApuCmd)) != ERR_OK)
      {
        ENCODER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
        return NULL;
      }

    if (!m_apu_cmd_mh_que.push(mh))
      {
        ENCODER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
        return NULL;
      }

    return mh.getPa();
  }

  bool freeApuCmdBuf()
  {
    if (!m_apu_cmd_mh_que.pop())
      {
        FILTER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
        return false;
      }

    return true;
  }

  void *m_dsp_handler;
};

__WIEN2_END_NAMESPACE

#endif /* WIEN2_ENCODER_COMPONENT_H */

