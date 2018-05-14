/********************************************************************
 *
 *  File Name: decoder_cmpt.h
 *
 *  Description: Decoder Componet Header
 *
 *  Notes: (C) Copyright 2015 Sony Corporation
 *
 *  Author: Tomonobu Hayakawa
 *
 ********************************************************************
 */

#ifndef _DECODER_COMPONENT_H_
#define _DECODER_COMPONENT_H_

#include "wien2_common_defs.h"
#include "apus/apu_cmd.h"
#include "memutils/os_utils/chateau_osal.h"
#include "memutils/s_stl/queue.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"
#include "debug/dbg_log.h"
#include "components/common/component_common.h"

__WIEN2_BEGIN_NAMESPACE

#define FALT_HANDLE_ID 0xFF

typedef bool (*DecCompCallback)(void*, void*);

struct InitDecCompParam
{
  AudioCodec       codec_type;
  uint32_t         input_sampling_rate;
  AudioChannelNum  channel_num;
  AudioPcmBitWidth bit_width;
  uint32_t         frame_sample_num;
  DecCompCallback  callback;
  void             *p_requester;
};

struct ExecDecCompParam
{
  BufferHeader input_buffer;
  BufferHeader output_buffer;
  uint8_t      num_of_au;
  bool         is_valid_frame;
};

struct StopDecCompParam
{
  BufferHeader output_buffer;
  bool         is_valid_frame;
};

struct SetDecCompParam
{
  uint8_t l_gain; /**< audio level gain of L ch (0 - 200%) */
  uint8_t r_gain; /**< audio level gain of R ch (0 - 200%) */
};

struct DecCmpltParam
{
  Apu::ApuEventType event_type;

  union
  {
    InitDecCompParam init_dec_cmplt;
    ExecDecCompParam exec_dec_cmplt;
    StopDecCompParam stop_dec_cmplt;
    SetDecCompParam  setparam_dec_cmplt;
  };
};

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
struct DecDebugLogInfo
{
  DebugLogInfo info;
  uint32_t     namemap_ps;
};
#endif

extern "C" {

uint32_t AS_decode_init(const InitDecCompParam& param,
                        void *p_instance,
                        uint32_t *dsp_inf);

bool AS_decode_exec(const ExecDecCompParam& param, void *p_instance);

bool AS_decode_stop(const StopDecCompParam& param, void *p_instance);

bool AS_decode_setparam(const SetDecCompParam& param, void *p_instance);

bool AS_decode_recv_apu(void *p_param, void *p_instance);

bool AS_decode_recv_done(void *p_instance);

uint32_t AS_decode_activate(AudioCodec param,
                            const char *path,
                            void **p_instance,
                            MemMgrLite::PoolId apu_pool_id,
                            MsgQueId apu_mid,
                            uint32_t *dsp_inf);

bool AS_decode_deactivate(void *p_instance);

} /* extern "C" */


class DecoderComponent : public ComponentCommon
{
public:
  DecoderComponent(MemMgrLite::PoolId apu_pool_id,MsgQueId apu_mid)
  {
    m_apu_pool_id = apu_pool_id;
    m_apu_mid = apu_mid;
  }
  ~DecoderComponent() {}

  uint32_t init_apu(const InitDecCompParam& param, uint32_t *dsp_inf);
  bool exec_apu(const ExecDecCompParam& param);
  bool flush_apu(const StopDecCompParam& param);
  bool setparam_apu(const SetDecCompParam& param);
  bool recv_apu(void *p_param);
  bool recv_done(void) { return freeApuCmdBuf(); };
  uint32_t activate(AudioCodec param, const char *path, uint32_t *dsp_inf);
  bool deactivate();
  MsgQueId get_apu_mid(void) { return m_apu_mid; };

  void *m_dsp_handler;

private:
  MemMgrLite::PoolId m_apu_pool_id;

  MsgQueId m_apu_mid;

  DecCompCallback m_callback;

  #define APU_DEC_QUEUE_SIZE 5

  typedef s_std::Queue<MemMgrLite::MemHandle, APU_DEC_QUEUE_SIZE> ApuCmdMhQue;
  ApuCmdMhQue m_apu_cmd_mh_que;

  void *m_p_requester;

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  DecDebugLogInfo m_debug_log_info;
#endif

  void send_apu(Apu::Wien2ApuCmd*);

  void* getApuCmdBuf()
  {
    MemMgrLite::MemHandle mh;

    if (mh.allocSeg(m_apu_pool_id, sizeof(Apu::Wien2ApuCmd)) != ERR_OK)
      {
        DECODER_WARN(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
        return NULL;
      }

    if (!m_apu_cmd_mh_que.push(mh))
      {
        DECODER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
        return NULL;
      }

    return mh.getPa();
  }

  bool freeApuCmdBuf()
  {
    if (!m_apu_cmd_mh_que.pop())
      {
        DECODER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
        return false;
      }

    return true;
  }
};

__WIEN2_END_NAMESPACE

#endif /* _DECODER_COMPONENT_H_ */

