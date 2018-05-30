/****************************************************************************
 * modules/audio/components/filter/postfilter_component.h
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
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

