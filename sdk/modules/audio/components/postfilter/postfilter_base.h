/********************************************************************
 *
 *  File Name: postfilter_base.h
 *
 *  Description: PostProcess Componet Header
 *
 *  Notes: (C) Copyright 2018 Sony Corporation
 *
 *  Author: -
 *
 ********************************************************************
 */

#ifndef _POSTFILTER_BASE_H_
#define _POSTFILTER_BASE_H_

#include "apus/apu_cmd.h"
#include "audio/audio_common_defs.h"
#include "memutils/os_utils/chateau_osal.h"
#include "memutils/memory_manager/MemHandle.h"

struct PostfilterCbParam
{
  Wien2::Apu::ApuEventType event_type;
};

typedef bool (*PostFilterCallback)(PostfilterCbParam*, void*);

struct InitPostfilterParam
{
  uint32_t           channel_num;
  uint32_t           bit_width;
  uint32_t           sample_num;
  PostFilterCallback callback;
  void               *p_requester;
};

struct ExecPostfilterParam
{
  AsPcmDataParam        input;
  MemMgrLite::MemHandle output_mh;
};

struct FlushPostfilterParam
{
  MemMgrLite::MemHandle output_mh;
};

struct PostfilterCmpltParam
{
  bool           result;
  AsPcmDataParam output;
};

class PostfilterBase
{
public:
  PostfilterBase() {}
  virtual ~PostfilterBase() {}

  virtual uint32_t init_apu(const InitPostfilterParam& param, uint32_t *dsp_inf) = 0;
  virtual bool exec_apu(const ExecPostfilterParam& param) = 0;
  virtual bool flush_apu(const FlushPostfilterParam& param) = 0;
  virtual bool recv_done(PostfilterCmpltParam *cmplt) = 0;
  virtual bool recv_done(void) = 0;
  virtual uint32_t activate(uint32_t *dsp_inf) = 0;
  virtual bool deactivate() = 0;

protected:
  PostFilterCallback m_callback;

  void *m_p_requester;
};

#endif /* _POSTFILTER_BASE_H_ */

