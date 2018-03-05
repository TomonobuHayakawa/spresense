/********************************************************************
 *
 *  File Name: postfilter_through.cpp
 *
 *  Description: PostProcess Componet
 *
 *  Notes: (C) Copyright 2018 Sony Corporation
 *
 *  Author: - 
 *
 ********************************************************************
 */

#include "postfilter_through.h"

/*--------------------------------------------------------------------
    Class Methods
  --------------------------------------------------------------------*/
uint32_t PostfilterThrough::init_apu(const InitPostfilterParam& param,
                                     uint32_t *dsp_inf)
{
  m_callback = param.callback;
  m_p_requester = param.p_requester;

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool PostfilterThrough::exec_apu(const ExecPostfilterParam& param)
{
  m_req_que.push(param.input);

  PostfilterCbParam cbpram;

  cbpram.event_type = Wien2::Apu::ExecEvent;

  m_callback(&cbpram, m_p_requester);

  return true;
}

/*--------------------------------------------------------------------*/
bool PostfilterThrough::flush_apu(const FlushPostfilterParam& param)
{
  AsPcmDataParam fls = { 0 };

  fls.mh       = param.output_mh;
  fls.is_valid = true;

  m_req_que.push(fls);

  PostfilterCbParam cbpram;

  cbpram.event_type = Wien2::Apu::FlushEvent;

  m_callback(&cbpram, m_p_requester);

  return true;
}

/*--------------------------------------------------------------------*/
bool PostfilterThrough::recv_done(PostfilterCmpltParam *cmplt)
{
  cmplt->output = m_req_que.top();
  cmplt->result = Wien2::Apu::ApuExecOK;

  m_req_que.pop();

  return true;
};

/*--------------------------------------------------------------------*/
uint32_t PostfilterThrough::activate(uint32_t *dsp_inf)
{
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool PostfilterThrough::deactivate(void)
{
  return true;
}

