/****************************************************************************
 * examples/streaming/include/app_memutil.h
 *
 *   Copyright (C) 2017 Sony Corporation.
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

#ifndef _APP_MEMUTIL_H_
#define _APP_MEMUTIL_H_

void ActivateAudioSubSystem(void);
bool ActivatePlayer(void);
bool ActivateSubPlayer(void);
bool ActivateOutputMix(void);
bool ActivateRenderer(void);

bool AS_DeactivatePlayer(void);
bool AS_DeactivateOutputMix(void);
bool AS_DeactivateSubPlayer(void);

void MsgLib_initFirst(uint32_t num_pools, uint32_t top_drm);
void MsgLib_initPerCpu(void);
void* MemMgr_translatePoolAddrToVa(uint32_t addr);
void MemMgr_createStaticPools(uint32_t layout_no, void* va, uint32_t work_size);
void MemMgr_initFirst(void* manager_area, uint32_t area_size);
void MemMgr_initPerCpu(void* manager_area, uint32_t pool_num);

#endif /* _APP_MEMUTIL_H_ */
