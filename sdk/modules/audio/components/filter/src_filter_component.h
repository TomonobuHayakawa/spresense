/***********************************************************************
 *
 *      File Name: src_filter_component.h
 *
 *      Description: Header file of SRC filter component
 *
 *      Notes: (C) Copyright 2016 Sony Corporation
 *
 *      Author: Hsingying Ho
 *
 ***********************************************************************
 */

#ifndef SRC_FILTER_COMPONENT_H
#define SRC_FILTER_COMPONENT_H

#include "memutils/os_utils/chateau_osal.h"

#include "wien2_common_defs.h"
#include "memutils/s_stl/queue.h"
#include "apus/apu_cmd.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"

#include "dsp_driver/include/dsp_drv.h"
#include "components/common/component_common.h"

#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE
using namespace MemMgrLite;

/*--------------------------------------------------------------------*/
/* TODO: A command path HighLevelAPI->Spu->Apu has
 *       too many paramter conversion.
 *       It should be decreased.
 */

struct InitSRCParam
{
  int32_t  sample_num;
  uint32_t input_sampling_rate;
  uint32_t output_sampling_rate;
  uint16_t input_pcm_byte_length;
  uint16_t output_pcm_byte_length;
  uint8_t  channel_num;
};

struct ExecSRCParam
{
  BufferHeader input_buffer;
  BufferHeader output_buffer;
};

struct StopSRCParam
{
  BufferHeader output_buffer;
};

/*--------------------------------------------------------------------*/
class SRCComponent : public ComponentCommon
{
private:
  typedef bool (*MppCompCallback)(DspDrvComPrm_t*);
  MppCompCallback m_callback;

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  DebugLogInfo m_debug_log_info;
#endif

public:
  SRCComponent(MsgQueId apu_dtq,PoolId apu_pool_id)
    : m_apu_dtq(apu_dtq)
    , m_apu_pool_id(apu_pool_id)
    ,m_dsp_handler(NULL) {}
  ~SRCComponent() {}

  uint32_t activate_apu(SRCComponent *p_component, const char *path, uint32_t *dsp_inf);
  bool deactivate_apu();
  uint32_t init_apu(InitSRCParam param, uint32_t *dsp_inf);
  bool exec_apu(ExecSRCParam param);
  bool flush_apu(StopSRCParam param);

  bool setCallBack(MppCompCallback func) { m_callback = func; return true; };
  bool recv_apu(DspDrvComPrm_t*);
  bool recv_done(void) { return freeApuCmdBuf(); };
  MsgQueId get_apu_mid(void) { return m_apu_dtq; };

private:
  typedef s_std::Queue<MemMgrLite::MemHandle, APU_COMMAND_QUEUE_SIZE> ApuQue;
  ApuQue m_apu_cmd_mh_que;

  MsgQueId m_apu_dtq;
  PoolId m_apu_pool_id;

  void send_apu(Apu::Wien2ApuCmd*);

  void* getApuCmdBuf()
  {
    MemMgrLite::MemHandle mh;

    if (mh.allocSeg(m_apu_pool_id, sizeof(Apu::Wien2ApuCmd)) != ERR_OK)
      {
        FILTER_WARN(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
        return NULL;
      }

    if (!m_apu_cmd_mh_que.push(mh))
      {
        FILTER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
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

#endif /* SRC_FILTER_COMPONENT_H */

