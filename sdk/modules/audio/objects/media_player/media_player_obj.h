/****************************************************************************
 * modules/audio/objects/media_player/media_player_obj.h
 *
 *   Copyright (C) 2017 Sony Corporation
 *   Author: Tomonobu Hayakawa <Tomonobu.Hayakawa@sony.com>
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

#ifndef __MODULES_AUDIO_OBJECTS_MEDIA_PLAYER_MEDIA_PLAYER_OBJ_H
#define __MODULES_AUDIO_OBJECTS_MEDIA_PLAYER_MEDIA_PLAYER_OBJ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "audio/audio_high_level_api.h"
#include "memutils/os_utils/chateau_osal.h"
#include "memutils/message/Message.h"
#include "memutils/s_stl/queue.h"
#include "memutils/memory_manager/MemHandle.h"
#include "wien2_common_defs.h"
#include "audio_state.h"
#include "common/audio_message_types.h"
#include "player_input_device_handler.h"
#include "wien2_internal_packet.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

class PlayerObj
{
public:
  static void create(void **obj,
                     MsgQueId self_dtq,
                     MsgQueId manager_dtq,
                     MsgQueId output_mix_dtq,
                     MsgQueId apu_dtq,
                     MemMgrLite::PoolId es_pool_id,
                     MemMgrLite::PoolId pcm_pool_id,
                     MemMgrLite::PoolId apu_pool_id);

  MsgQueId get_selfId()
    {
      return m_self_dtq;
    }
  MsgQueId get_apuId()
    {
      return m_apu_dtq;
    }

private:
  PlayerObj(MsgQueId self_dtq,
            MsgQueId manager_dtq,
            MsgQueId output_mix_dtq,
            MsgQueId apu_dtq,
            MemMgrLite::PoolId es_pool_id,
            MemMgrLite::PoolId pcm_pool_id,
            MemMgrLite::PoolId apu_pool_id);

  MsgQueId m_self_dtq;
  MsgQueId m_manager_dtq;
  MsgQueId m_output_mix_dtq;
  MsgQueId m_apu_dtq;

  MemMgrLite::PoolId m_es_pool_id;
  MemMgrLite::PoolId m_pcm_pool_id;
  MemMgrLite::PoolId m_apu_pool_id; /* そもそも、Apuのコマンドは、Componentsで取得じゃないの？:TODO */

  int m_outmix_handle;

  enum PlayerState
  {
    BootedState = 0,
    ReadyState,
    PrePlayParentState,
    PlayState,
    StoppingState,
    WaitEsEndState,
    UnderflowState,
    WaitStopState,
    PlayerStateNum
  };

  enum PlayerSubState
  {
    InvalidSubState = 0xffffffff,
    SubStatePrePlay = 0,
    SubStatePrePlayStopping,
    SubStatePrePlayWaitEsEnd,
    SubStatePrePlayUnderflow,
    SubStateNum
  };

  AudioState<PlayerState>    m_state;
  AudioState<PlayerSubState> m_sub_state;

  PlayerInputDeviceHandler *m_input_device_handler;
  InputHandlerOfRAM         m_in_ram_device_handler;
  void*                     m_p_dec_instance;

  uint32_t  m_max_es_buff_size;
  uint32_t  m_max_pcm_buff_size;
  AudioCodec  m_codec_type;

  #define  MAX_EXEC_COUNT    4  /* Number of audio frames to be prior introduced. */
  #define  MAX_OUT_BUFF_NUM  9  /* Number of PCM buffer. */

  typedef s_std::Queue<MemMgrLite::MemHandle, MAX_EXEC_COUNT + 1> EsMhQueue;
  EsMhQueue m_es_buf_mh_que;

  typedef s_std::Queue<MemMgrLite::MemHandle, MAX_OUT_BUFF_NUM> PcmMhQueue;
  PcmMhQueue m_pcm_buf_mh_que;

  typedef s_std::Queue<OutputMixObjInputDataCmd, MAX_OUT_BUFF_NUM + 1> DecodecPcmMhQueue;
  DecodecPcmMhQueue m_decoded_pcm_mh_que;

  s_std::Queue<AudioCommand, 1> m_external_cmd_que;

  void run(void);
  void parse(MsgPacket *);
  void parseSubState(MsgPacket *);

  typedef void (PlayerObj::*MsgProc)(MsgPacket *);
  static MsgProc MsgProcTbl[AUD_PLY_MSG_NUM][PlayerStateNum];
  static MsgProc PlayerSubStateTbl[AUD_PLY_MSG_NUM][SubStateNum];

  void illegalEvt(MsgPacket *);

  void activate(MsgPacket *);
  void deactivate(MsgPacket *);

  void init(MsgPacket *);

  void playOnReady(MsgPacket *);

  void stopOnPlay(MsgPacket *);
  void stopOnWait(MsgPacket *);
  void stopOnUnderflow(MsgPacket *);
  void stopOnWaitEsEnd(MsgPacket *);
  void stopOnPrePlay(MsgPacket *);
  void stopOnPrePlayUnderflow(MsgPacket *);
  void stopOnPrePlayWaitEsEnd(MsgPacket *);

  void illegalSinkDone(MsgPacket *);
  void sinkDoneOnBoot(MsgPacket *);
  void sinkDoneOnReady(MsgPacket *);
  void sinkDoneOnPlay(MsgPacket *);
  void sinkDoneOnStopping(MsgPacket *);
  void sinkDoneOnUnderflow(MsgPacket *);
  void sinkDoneOnWaitEsEnd(MsgPacket *);

  void illegalDecDone(MsgPacket *);
  void decDoneOnPlay(MsgPacket *);
  void decDoneOnWaitStop(MsgPacket *);
  void decDoneOnWaitEsEnd(MsgPacket *);
  void decDoneOnPrePlay(MsgPacket *);
  void decDoneOnPrePlayStopping(MsgPacket *);
  void decDoneOnPrePlayUnderflow(MsgPacket *);
  void decDoneOnPrePlayWaitEsEnd(MsgPacket *);

  void setClkRecovery(MsgPacket *);

  uint32_t loadCodec(AudioCodec codec, uint32_t* dsp_inf);
  bool unloadCodec();

  uint32_t startPlay(uint32_t* dsp_inf);
  void stopPlay(void);

  void sendPcmToOutputMix(const OutputMixObjInputDataCmd& data);
  void stopOutputMix(void);
  void sendPcmToOutputMixOnDecStopping();

  void decode(void* p_es, uint32_t es_size);

  void *allocPcmBuf(uint32_t size);
  bool  freePcmBuf() {
  if (!m_pcm_buf_mh_que.pop())
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
      return false;
    }
    return true;
  }

  void *getEs(uint32_t* size);
  bool freeEsBuf()
    {
      if (!m_es_buf_mh_que.pop())
        {
        MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
        return false;
      }
      return true;
    }

  void sendAudioCmdCmplt(const AudioCommand& cmd,
                         uint32_t result,
                         uint32_t sub_result = 0)
    {
      AudioMngCmdCmpltResult cmplt(cmd.header.command_code, cmd.header.sub_code, result, AS_MODULE_ID_PLAYER_OBJ, sub_result);
      err_t er = MsgLib::send<AudioMngCmdCmpltResult>(m_manager_dtq, MsgPriNormal, MSG_TYPE_AUD_RES, m_self_dtq, cmplt);
      if (ERR_OK != er)
        {
          F_ASSERT(0);
        }
    }

  void finalize();
  bool checkAndSetMemPool();
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

__WIEN2_END_NAMESPACE

#endif /* __MODULES_AUDIO_OBJECTS_MEDIA_PLAYER_MEDIA_PLAYER_OBJ_H */
