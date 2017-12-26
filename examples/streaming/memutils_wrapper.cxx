/****************************************************************************
 * examples/streaming/memutils_wrapper.cxx
 *
 *   Copyright (C) 2017 Sony Corporation
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "memutils/message/Message.h"
#include "memutils/memory_manager/MemHandle.h"

#include <arch/chip/cxd56_audio.h>

#include "msgq_id.h"

#include "mem_layout.h"
#include "msgq_pool.h"
#include "pool_layout.h"
#include "fixed_fence.h"

#include "audio/audio_high_level_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
extern "C" {

void ActivateAudioSubSystem(void)
{
    AudioSubSystemIDs ids;

    ids.app = MSGQ_AUD_APP;
    ids.mng = MSGQ_AUD_MGR;
    ids.player_main = MSGQ_AUD_PLY;
    ids.player_sub = 0xFF;
    ids.mixer = MSGQ_AUD_OUTPUT_MIX;
    ids.recorder = 0xFF;
    ids.effector = 0xFF;
    ids.recognizer = 0xFF;

    AS_ActivateAudioSubSystem(ids);
}
    
bool ActivatePlayer(void)
{
  AsActPlayerParam_t player_act_param;

  player_act_param.msgq_id.player = MSGQ_AUD_PLY;
  player_act_param.msgq_id.mng    = MSGQ_AUD_MGR;
  player_act_param.msgq_id.mixer  = MSGQ_AUD_OUTPUT_MIX;
  player_act_param.msgq_id.dsp    = MSGQ_AUD_DSP;
  player_act_param.pool_id.es     = DEC_ES_MAIN_BUF_POOL;
  player_act_param.pool_id.pcm    = REND_PCM_BUF_POOL;
  player_act_param.pool_id.dsp    = DEC_APU_CMD_POOL;
  
  return AS_ActivatePlayer(&player_act_param);
}

bool ActivateSubPlayer(void)
{
  AsActPlayerParam_t sub_player_act_param;

  sub_player_act_param.msgq_id.player = MSGQ_AUD_SUB_PLY;
  sub_player_act_param.msgq_id.mng    = MSGQ_AUD_MGR;
  sub_player_act_param.msgq_id.mixer  = MSGQ_AUD_OUTPUT_MIX;
  sub_player_act_param.msgq_id.dsp    = MSGQ_AUD_DSP;
  sub_player_act_param.pool_id.es     = DEC_ES_SUB_BUF_POOL;
  sub_player_act_param.pool_id.pcm    = REND_PCM_SUB_BUF_POOL;
  sub_player_act_param.pool_id.dsp    = DEC_APU_CMD_POOL;

  return AS_ActivateSubPlayer(&sub_player_act_param);
}

bool ActivateOutputMix(void)
{
  AsActOutputMixParam_t output_mix_act_param;

  output_mix_act_param.msgq_id.mixer = MSGQ_AUD_OUTPUT_MIX;
  output_mix_act_param.msgq_id.mng   = MSGQ_AUD_MGR;

  return AS_ActivateOutputMix(&output_mix_act_param);
}

bool ActivateRenderer(void)
{
  AsActRendererParam_t renderer_act_param;

  renderer_act_param.msgq_id.dev0_req  = MSGQ_AUD_RND_PLY;
  renderer_act_param.msgq_id.dev0_sync = MSGQ_AUD_RND_PLY_SYNC;
  renderer_act_param.msgq_id.dev1_req  = MSGQ_AUD_RND_SUB;
  renderer_act_param.msgq_id.dev1_sync = MSGQ_AUD_RND_SUB_SYNC;

  return AS_ActivateRenderer(&renderer_act_param);
}

void MsgLib_initFirst(uint32_t num_pools, uint32_t top_drm)
{
  MsgLib::initFirst(num_pools, top_drm);
  return;
}

void MsgLib_initPerCpu(void)
{
  MsgLib::initPerCpu();
  return;
}

void* MemMgr_translatePoolAddrToVa(uint32_t addr)
{
  return MemMgrLite::translatePoolAddrToVa(static_cast<MemMgrLite::PoolAddr>(addr));
}

void MemMgr_createStaticPools(uint32_t layout_no, void* va, uint32_t work_size)
{
  MemMgrLite::Manager::createStaticPools(layout_no, va, work_size, MemMgrLite::MemoryPoolLayouts[layout_no]);
  return;
}

void MemMgr_initFirst(void* manager_area, uint32_t area_size)
{
  MemMgrLite::Manager::initFirst(manager_area, area_size);
  return;
}

void MemMgr_initPerCpu(void* manager_area, uint32_t pool_num)
{
  MemMgrLite::Manager::initPerCpu(manager_area, pool_num);
  return;
}

} /* extern "C" */

