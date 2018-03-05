/********************************************************************
 *
 *  File Name: postfilter_api.h
 *
 *  Description: PostProcess Componet Header
 *
 *  Notes: (C) Copyright 2018 Sony Corporation
 *
 *  Author: -
 *
 ********************************************************************
 */

#ifndef _POSTFILTER_API_H_
#define _POSTFILTER_API_H_

#include "postfilter_component.h"
#include "postfilter_through.h"

extern "C" {

uint32_t AS_postfilter_init(const InitPostfilterParam *param,
                            void *p_instance,
                            uint32_t *dsp_inf);

bool AS_postfilter_exec(const ExecPostfilterParam *param, void *p_instance);

bool AS_postfilter_flush(const FlushPostfilterParam *param, void *p_instance);

bool AS_postfilter_recv_done(void *p_instance, PostfilterCmpltParam *cmplt);

uint32_t AS_postfilter_activate(void **p_instance,
                                MemMgrLite::PoolId apu_pool_id,
                                MsgQueId apu_mid,
                                uint32_t *dsp_inf,
                                bool through);

bool AS_postfilter_deactivate(void *p_instance);

} /* extern "C" */

#endif /* _POSTFILTER_API_H_ */

