/********************************************************************
 *
 *  File Name: postfilter_through.h
 *
 *  Description: PostProcess Componet Header
 *
 *  Notes: (C) Copyright 2018 Sony Corporation
 *
 *  Author: -
 *
 ********************************************************************
 */

#ifndef _POSTFILTER_THROUGH_H_
#define _POSTFILTER_THROUGH_H_

#include "audio/audio_high_level_api.h"
#include "memutils/s_stl/queue.h"
#include "postfilter_base.h"

class PostfilterThrough : public PostfilterBase
{
public:
  PostfilterThrough() {}
  ~PostfilterThrough() {}

  virtual uint32_t init_apu(const InitPostfilterParam& param, uint32_t *dsp_inf);
  virtual bool exec_apu(const ExecPostfilterParam& param);
  virtual bool flush_apu(const FlushPostfilterParam& param);
  virtual bool recv_done(PostfilterCmpltParam *cmplt);
  virtual bool recv_done(void) { return true; };
  virtual uint32_t activate(uint32_t *dsp_inf);
  virtual bool deactivate();

private:
  #define REQ_QUEUE_SIZE 7 
  typedef s_std::Queue<AsPcmDataParam, REQ_QUEUE_SIZE> ReqQue;
  ReqQue m_req_que;
};

#endif /* _POSTFILTER_THROUGH_H_ */

