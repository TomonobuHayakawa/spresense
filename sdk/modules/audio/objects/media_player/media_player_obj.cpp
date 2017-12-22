/****************************************************************************
 * modules/audio/objects/media_player/media_player_obj.cpp
 *
 *   Copyright (C)  2017 Sony Corporation
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <nuttx/arch.h>
#include <string.h>
#include <stdlib.h>
#include "memutils/os_utils/os_wrapper.h"
#include "memutils/common_utils/common_assert.h"
#include "media_player_obj.h"
#include "components/decoder/decoder_component.h"
#include "dsp_driver/include/dsp_drv.h"
#include "debug/dbg_log.h"

__USING_WIEN2
using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef SUPPORT_SBC_PLAYER
#  define NUM_OF_AU 8
#else
/* Note: Codectypes other than SBC do not adopt the concept of super AU. */
#  define NUM_OF_AU 1
#endif /* SUPPORT_SBC_PLAYER */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pid_t    s_ply_pid = -1;
static MsgQueId s_self_dtq;
static MsgQueId s_manager_dtq;
static MsgQueId s_omix_dtq;
static MsgQueId s_dsp_dtq;
static PoolId   s_es_pool_id;
static PoolId   s_pcm_pool_id;
static PoolId   s_apu_pool_id;
static pid_t    s_sub_ply_pid;
static MsgQueId s_sub_self_dtq;
static MsgQueId s_sub_manager_dtq;
static MsgQueId s_sub_omix_dtq;
static MsgQueId s_sub_dsp_dtq;
static PoolId   s_sub_es_pool_id;
static PoolId   s_sub_pcm_pool_id;
static PoolId   s_sub_apu_pool_id;

static void *s_play_obj = NULL;
static void *s_sub_play_obj = NULL;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool decoder_comp_done_callback(void *p_response, FAR void *p_requester)
{
  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;
  DecCmpltParam cmplt;

  if (DSP_COM_DATA_TYPE_STRUCT_ADDRESS != p_param->type)
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_DSP_ILLEGAL_REPLY);
      return false;
    }

  Apu::Wien2ApuCmd *packet = reinterpret_cast<Apu::Wien2ApuCmd *>
    (p_param->data.pParam);

  cmplt.event_type = static_cast<Wien2::Apu::ApuEventType>
    (packet->header.event_type);

  switch (packet->header.event_type)
    {
      case Apu::ExecEvent:
        {
          cmplt.exec_dec_cmplt.input_buffer   = packet->exec_dec_cmd.input_buffer;
          cmplt.exec_dec_cmplt.output_buffer  = packet->exec_dec_cmd.output_buffer;
          cmplt.exec_dec_cmplt.is_valid_frame =
            ((packet->result.exec_result == Apu::ApuExecOK) ? true : false);

          if (!cmplt.exec_dec_cmplt.is_valid_frame)
            {
              MEDIA_PLAYER_WARN(AS_ATTENTION_SUB_CODE_DSP_RESULT_ERROR);
            }

          MEDIA_PLAYER_VDBG("Dec s %d v %d\n",
                            cmplt.exec_dec_cmplt.output_buffer.size,
                            cmplt.exec_dec_cmplt.is_valid_frame);

          err_t er = MsgLib::send<DecCmpltParam>((static_cast<FAR PlayerObj *>
                                                  (p_requester))->get_selfId(),
                                                 MsgPriNormal,
                                                 MSG_AUD_PLY_CMD_DEC_DONE,
                                                 (static_cast<FAR PlayerObj *>
                                                  (p_requester))->get_selfId(),
                                                 cmplt);
          F_ASSERT(er == ERR_OK);
        }
        break;

    case Apu::FlushEvent:
      {
        cmplt.stop_dec_cmplt.output_buffer  = packet->flush_dec_cmd.output_buffer;
        cmplt.stop_dec_cmplt.is_valid_frame =
          ((packet->result.exec_result == Apu::ApuExecOK) ? true : false);

        if (!cmplt.stop_dec_cmplt.is_valid_frame)
          {
            MEDIA_PLAYER_WARN(AS_ATTENTION_SUB_CODE_DSP_RESULT_ERROR);
          }

        MEDIA_PLAYER_VDBG("FlsDec s %d v %d\n",
                          cmplt.stop_dec_cmplt.output_buffer.size,
                          cmplt.stop_dec_cmplt.is_valid_frame);

        err_t er = MsgLib::send<DecCmpltParam>((static_cast<FAR PlayerObj *>
                                                (p_requester))->get_selfId(),
                                               MsgPriNormal,
                                               MSG_AUD_PLY_CMD_DEC_DONE,
                                               (static_cast<FAR PlayerObj *>
                                                (p_requester))->get_selfId(),
                                               cmplt);
        F_ASSERT(er == ERR_OK);
      }
      break;

    default:
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_DSP_ILLEGAL_REPLY);
      return false;
  }
  return true;
}

/*--------------------------------------------------------------------------*/
PlayerObj::PlayerObj(MsgQueId self_dtq,
                     MsgQueId manager_dtq,
                     MsgQueId output_mix_dtq,
                     MsgQueId apu_dtq,
                     MemMgrLite::PoolId es_pool_id,
                     MemMgrLite::PoolId pcm_pool_id,
                     MemMgrLite::PoolId apu_pool_id):
  m_self_dtq(self_dtq),
  m_manager_dtq(manager_dtq),
  m_output_mix_dtq(output_mix_dtq),
  m_apu_dtq(apu_dtq),
  m_es_pool_id(es_pool_id),
  m_pcm_pool_id(pcm_pool_id),
  m_apu_pool_id(apu_pool_id),
  m_outmix_handle(0),
  m_state(AS_MODULE_ID_PLAYER_OBJ, "main", BootedState),
  m_sub_state(AS_MODULE_ID_PLAYER_OBJ, "sub", InvalidSubState),
  m_input_device_handler(NULL),
  m_codec_type(InvalidCodecType)
{
}

/*--------------------------------------------------------------------------*/
void PlayerObj::run(void)
{
  err_t        err_code;
  MsgQueBlock *que;
  MsgPacket   *msg;

  err_code = MsgLib::referMsgQueBlock(m_self_dtq, &que);
  F_ASSERT(err_code == ERR_OK);

  while(1)
    {
      err_code = que->recv(TIME_FOREVER, &msg);
      F_ASSERT(err_code == ERR_OK);

      parse(msg);

      err_code = que->pop();
      F_ASSERT(err_code == ERR_OK);
    }
}

/*--------------------------------------------------------------------------*/
PlayerObj::MsgProc PlayerObj::MsgProcTbl[AUD_PLY_MSG_NUM][PlayerStateNum] =
{
  /* Message type: MSG_AUD_PLY_CMD_ACT. */

  {                                  /* Player status:        */
    &PlayerObj::activate,            /*   BootedState.        */
    &PlayerObj::init,                /*   ReadyState.         */
    &PlayerObj::parseSubState,       /*   PrePlayParentState. */
    &PlayerObj::illegalEvt,          /*   PlayState.          */
    &PlayerObj::illegalEvt,          /*   StoppingState.      */
    &PlayerObj::illegalEvt,          /*   WaitEsEndState.     */
    &PlayerObj::illegalEvt,          /*   UnderflowState.     */
    &PlayerObj::illegalEvt           /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_PLY_CMD_INIT. */

  {                                  /* Player status:        */
    &PlayerObj::illegalEvt,          /*   BootedState.        */
    &PlayerObj::init,                /*   ReadyState.         */
    &PlayerObj::parseSubState,       /*   PrePlayParentState. */
    &PlayerObj::illegalEvt,          /*   PlayState.          */
    &PlayerObj::illegalEvt,          /*   StoppingState.      */
    &PlayerObj::illegalEvt,          /*   WaitEsEndState.     */
    &PlayerObj::illegalEvt,          /*   UnderflowState.     */
    &PlayerObj::illegalEvt           /*   WaitStopState.      */
  },

  /* Message type:  MSG_AUD_PLY_CMD_PLAY. */

  {                                  /* Player status:        */
    &PlayerObj::illegalEvt,          /*   BootedState.        */
    &PlayerObj::playOnReady,         /*   ReadyState.         */
    &PlayerObj::parseSubState,       /*   PrePlayParentState. */
    &PlayerObj::illegalEvt,          /*   PlayState.          */
    &PlayerObj::illegalEvt,          /*   StoppingState.      */
    &PlayerObj::illegalEvt,          /*   WaitEsEndState.     */
    &PlayerObj::illegalEvt,          /*   UnderflowState.     */
    &PlayerObj::illegalEvt           /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_PLY_CMD_STOP. */

  {                                  /* Player status:        */
    &PlayerObj::illegalEvt,          /*   BootedState.        */
    &PlayerObj::illegalEvt,          /*   ReadyState.         */
    &PlayerObj::parseSubState,       /*   PrePlayParentState. */
    &PlayerObj::stopOnPlay,          /*   PlayState.          */
    &PlayerObj::illegalEvt,          /*   StoppingState.      */
    &PlayerObj::stopOnWaitEsEnd,     /*   WaitEsEndState.     */
    &PlayerObj::stopOnUnderflow,     /*   UnderflowState.     */
    &PlayerObj::stopOnWait           /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_PLY_CMD_DEACT. */

  {                                  /* Player status:        */
    &PlayerObj::illegalEvt,          /*   BootedState.        */
    &PlayerObj::deactivate,          /*   ReadyState.         */
    &PlayerObj::parseSubState,       /*   PrePlayParentState. */
    &PlayerObj::illegalEvt,          /*   PlayState.          */
    &PlayerObj::illegalEvt,          /*   StoppingState.      */
    &PlayerObj::illegalEvt,          /*   WaitEsEndState.     */
    &PlayerObj::illegalEvt,          /*   UnderflowState.     */
    &PlayerObj::illegalEvt           /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_PLY_CMD_OUTPUT_MIX_DONE. */

  {                                  /* Player status:        */
    &PlayerObj::sinkDoneOnBoot,      /*   BootedState.        */
    &PlayerObj::sinkDoneOnReady,     /*   ReadyState.         */
    &PlayerObj::parseSubState,       /*   PrePlayParentState. */
    &PlayerObj::sinkDoneOnPlay,      /*   PlayState.          */
    &PlayerObj::sinkDoneOnStopping,  /*   StoppingState.      */
    &PlayerObj::sinkDoneOnWaitEsEnd, /*   WaitEsEndState.     */
    &PlayerObj::sinkDoneOnUnderflow, /*   UnderflowState.     */
    &PlayerObj::illegalSinkDone      /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_PLY_CMD_DEC_DONE. */

  {                                  /* Player status:        */
    &PlayerObj::illegalDecDone,      /*   BootedState.        */
    &PlayerObj::illegalDecDone,      /*   ReadyState.         */
    &PlayerObj::parseSubState,       /*   PrePlayParentState. */
    &PlayerObj::decDoneOnPlay,       /*   PlayState.          */
    &PlayerObj::decDoneOnWaitStop,   /*   StoppingState.      */
    &PlayerObj::decDoneOnWaitEsEnd,  /*   WaitEsEndState.     */
    &PlayerObj::decDoneOnWaitStop,   /*   UnderflowState.     */
    &PlayerObj::illegalDecDone       /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_PLY_CMD_CLKRECOVERY. */

  {                                  /* Player status:        */
    &PlayerObj::illegalEvt,          /*   BootedState.        */
    &PlayerObj::setClkRecovery,      /*   ReadyState.         */
    &PlayerObj::parseSubState,       /*   PrePlayParentState. */
    &PlayerObj::setClkRecovery,      /*   PlayState.          */
    &PlayerObj::setClkRecovery,      /*   StoppingState.      */
    &PlayerObj::setClkRecovery,      /*   WaitEsEndState.     */
    &PlayerObj::setClkRecovery,      /*   UnderflowState.     */
    &PlayerObj::setClkRecovery       /*   WaitStopState.      */
  }
};

/*--------------------------------------------------------------------------*/
PlayerObj::MsgProc PlayerObj::PlayerSubStateTbl[AUD_PLY_MSG_NUM][SubStateNum] =
{
  /* Message type: MSG_AUD_PLY_CMD_ACT. */

  {                                        /* Player sub status:          */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlay.          */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayStopping.  */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayWaitEsEnd. */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayUnderflow. */
  },

  /* Message type: MSG_AUD_PLY_CMD_INIT. */

  {                                        /* Player sub status:          */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlay.          */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayStopping.  */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayWaitEsEnd. */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayUnderflow. */
  },

  /* Message type:  MSG_AUD_PLY_CMD_PLAY. */

  {                                        /* Player sub status:          */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlay.          */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayStopping.  */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayWaitEsEnd. */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayUnderflow. */
  },

  /* Message type: MSG_AUD_PLY_CMD_STOP. */

  {                                        /* Player sub status:          */
    &PlayerObj::stopOnPrePlay,             /*   SubStatePrePlay.          */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayStopping.  */
    &PlayerObj::stopOnPrePlayWaitEsEnd,    /*   SubStatePrePlayWaitEsEnd. */
    &PlayerObj::stopOnPrePlayUnderflow,    /*   SubStatePrePlayUnderflow. */
  },

  /* Message type: MSG_AUD_PLY_CMD_DEACT. */

  {                                        /* Player sub status:          */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlay.          */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayStopping.  */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayWaitEsEnd. */
    &PlayerObj::illegalEvt,                /*   SubStatePrePlayUnderflow. */
  },

  /* Message type: MSG_AUD_PLY_CMD_OUTPUT_MIX_DONE. */

  {                                        /* Player sub status:          */
    &PlayerObj::illegalSinkDone,           /*   SubStatePrePlay.          */
    &PlayerObj::illegalSinkDone,           /*   SubStatePrePlayStopping.  */
    &PlayerObj::illegalSinkDone,           /*   SubStatePrePlayWaitEsEnd. */
    &PlayerObj::illegalSinkDone,           /*   SubStatePrePlayUnderflow. */
  },

  /* Message type: MSG_AUD_PLY_CMD_DEC_DONE. */

  {                                        /* Player sub status:          */
    &PlayerObj::decDoneOnPrePlay,          /*   SubStatePrePlay.          */
    &PlayerObj::decDoneOnPrePlayStopping,  /*   SubStatePrePlayStopping.  */
    &PlayerObj::decDoneOnPrePlayWaitEsEnd, /*   SubStatePrePlayWaitEsEnd. */
    &PlayerObj::decDoneOnPrePlayUnderflow, /*   SubStatePrePlayUnderflow. */
  },

  /* Message type: MSG_AUD_PLY_CMD_CLKRECOVERY. */

  {                                        /* Player sub status:          */
    &PlayerObj::setClkRecovery,            /*   SubStatePrePlay.          */
    &PlayerObj::setClkRecovery,            /*   SubStatePrePlayStopping.  */
    &PlayerObj::setClkRecovery,            /*   SubStatePrePlayWaitEsEnd. */
    &PlayerObj::setClkRecovery,            /*   SubStatePrePlayUnderflow. */
  }
};

/*--------------------------------------------------------------------------*/
void PlayerObj::parse(MsgPacket *msg)
{
  uint event = MSG_GET_SUBTYPE(msg->getType());
  F_ASSERT(event < AUD_PLY_MSG_NUM);

  (this->*MsgProcTbl[event][m_state.get()])(msg);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::illegalEvt(MsgPacket *msg)
{
  AudioCommand cmd = msg->moveParam<AudioCommand>();

  sendAudioCmdCmplt(cmd, AS_RESPONSE_CODE_STATE_VIOLATION);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::activate(MsgPacket *msg)
{
  AudioCommand cmd = msg->moveParam<AudioCommand>();
  PlayerInputDeviceHandler::PlayerInHandle in_device_handle;
  bool result;

  MEDIA_PLAYER_DBG("ACT: act %d, indev %d, indev sub %d, "
                   "outdev %d, outdev sub %d\n",
                   cmd.set_player_sts_param.active_player,
                   cmd.set_player_sts_param.input_device,
                   cmd.set_player_sts_param.input_device_sub,
                   cmd.set_player_sts_param.output_device,
                   cmd.set_player_sts_param.output_device_sub);

  if (!checkAndSetMemPool())
    {
      sendAudioCmdCmplt(cmd, AS_RESPONSE_CODE_CHECK_MEMORY_POOL_ERROR);
      return;
    }

  switch (cmd.set_player_sts_param.input_device)
    {
      case AS_SETPLAYER_INPUTDEVICE_RAM:
        {
          m_input_device_handler = &m_in_ram_device_handler;
          in_device_handle.p_ram_device_handle =
            cmd.set_player_sts_param.ram_handler;
        }
        break;

    default:
      sendAudioCmdCmplt(cmd, AS_RESPONSE_CODE_COMMAND_PARAM_INPUT_DEVICE);
      return;
  }

  result = m_input_device_handler->initialize(&in_device_handle);
  if (result)
    {
      err_t er = ERR_OK;
      OutputMixObjActCmd init_output_mix_param;
      OutputMixObjParam  outmix_param;

      init_output_mix_param.mixer_type = MainOnly;
      init_output_mix_param.output_device = HPOutputDevice;

      outmix_param.handle    = 0;
      outmix_param.act_param = init_output_mix_param;

      er = MsgLib::send<OutputMixObjParam>(m_output_mix_dtq,
                                           MsgPriNormal,
                                           MSG_AUD_MIX_CMD_ACT,
                                           m_self_dtq,
                                           outmix_param);
      F_ASSERT(er == ERR_OK);

      if (!m_external_cmd_que.push(cmd))
        {
          sendAudioCmdCmplt(cmd, AS_RESPONSE_CODE_QUEUE_OPERATION_ERROR);
          return;
        }
    }
  else
    {
      sendAudioCmdCmplt(cmd, AS_RESPONSE_CODE_COMMAND_PARAM_INPUT_HANDLER);
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::deactivate(MsgPacket *msg)
{
  AudioCommand cmd = msg->moveParam<AudioCommand>();
  OutputMixObjParam outmix_param;
  err_t er;

  MEDIA_PLAYER_DBG("DEACT:\n");

  if (!unloadCodec())
    {
      sendAudioCmdCmplt(cmd, AS_RESPONSE_CODE_DSP_UNLOAD_ERROR);
      return;
    }

  if (!m_external_cmd_que.push(cmd))
    {
      sendAudioCmdCmplt(cmd, AS_RESPONSE_CODE_QUEUE_OPERATION_ERROR);
      return;
    }

  outmix_param.handle = m_outmix_handle;

  er = MsgLib::send<OutputMixObjParam>(m_output_mix_dtq,
                                       MsgPriNormal,
                                       MSG_AUD_MIX_CMD_DEACT,
                                       m_self_dtq,
                                       outmix_param);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::init(MsgPacket *msg)
{
  AudioCommand cmd = msg->moveParam<AudioCommand>();
  uint8_t result;

  MEDIA_PLAYER_DBG("INIT: ch num %d, bit len %d, codec %d, fs %d\n",
           cmd.init_player_param.channel_number,
           cmd.init_player_param.bit_length,
           cmd.init_player_param.codec_type,
           cmd.init_player_param.sampling_rate);

  result = m_input_device_handler->setParam(cmd.init_player_param);
  if (result == AS_RESPONSE_CODE_OK)
    {
      sendAudioCmdCmplt(cmd, AS_RESPONSE_CODE_OK);
    }
  else
    {
      sendAudioCmdCmplt(cmd, result);
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::playOnReady(MsgPacket *msg)
{
  AudioCommand cmd = msg->moveParam<AudioCommand>();
  uint32_t dsp_inf = 0;
  uint32_t rst     = AS_RESPONSE_CODE_OK;

  MEDIA_PLAYER_DBG("PLAY:\n");

  if ((rst = startPlay(&dsp_inf)) == AS_RESPONSE_CODE_OK)
    {
      m_sub_state = SubStatePrePlay;
      m_state = PrePlayParentState;

      sendAudioCmdCmplt(cmd, AS_RESPONSE_CODE_OK);
    }
  else
    {
      sendAudioCmdCmplt(cmd, rst, dsp_inf);
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::stopOnPlay(MsgPacket *msg)
{
  AudioCommand cmd = msg->moveParam<AudioCommand>();

  MEDIA_PLAYER_DBG("STOP:\n");

  /* Note: Since the completion of current command will be notified
   * by other event trigger, queue external command.
   */

  if (!m_external_cmd_que.push(cmd))
    {
      sendAudioCmdCmplt(cmd, AS_RESPONSE_CODE_QUEUE_OPERATION_ERROR);
      return;
    }

  if (cmd.stop_player_param.stop_mode == AS_STOPPLAYER_NORMAL)
    {
      stopPlay();
      m_state = StoppingState;
    }
  else
    {
      m_state = WaitEsEndState;
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::stopOnWait(MsgPacket *msg)
{
  AudioCommand cmd = msg->moveParam<AudioCommand>();

  MEDIA_PLAYER_DBG("STOP:\n");

  sendAudioCmdCmplt(cmd, AS_RESPONSE_CODE_OK);

  m_state = ReadyState;
}

/*--------------------------------------------------------------------------*/
void PlayerObj::stopOnWaitEsEnd(MsgPacket *msg)
{
  stopOnPlay(msg);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::stopOnUnderflow(MsgPacket *msg)
{
  AudioCommand cmd = msg->moveParam<AudioCommand>();

  MEDIA_PLAYER_DBG("STOP:\n");

  if (!m_external_cmd_que.push(cmd))
    {
      sendAudioCmdCmplt(cmd, AS_RESPONSE_CODE_QUEUE_OPERATION_ERROR);
      return;
    }

  m_state = StoppingState;
}

/*--------------------------------------------------------------------------*/
void PlayerObj::stopOnPrePlay(MsgPacket *msg)
{
  AudioCommand cmd = msg->moveParam<AudioCommand>();

  MEDIA_PLAYER_DBG("STOP:\n");

  /* Note: Since the completion of current command will be notified
   * by other event trigger, queue external command.
   */

  if (!m_external_cmd_que.push(cmd))
    {
      sendAudioCmdCmplt(cmd, AS_RESPONSE_CODE_QUEUE_OPERATION_ERROR);
      return;
    }

  if (cmd.stop_player_param.stop_mode == AS_STOPPLAYER_NORMAL)
    {
      stopPlay();
      m_sub_state = SubStatePrePlayStopping;
    }
  else
    {
      m_sub_state = SubStatePrePlayWaitEsEnd;
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::stopOnPrePlayWaitEsEnd(MsgPacket *msg)
{
  stopOnPrePlay(msg);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::stopOnPrePlayUnderflow(MsgPacket *msg)
{
  AudioCommand cmd = msg->moveParam<AudioCommand>();

  MEDIA_PLAYER_DBG("STOP:\n");

  /* Note: Since the completion of current command will be notified
   * by other event trigger, queue external command.
   */

  if (!m_external_cmd_que.push(cmd))
    {
      sendAudioCmdCmplt(cmd, AS_RESPONSE_CODE_QUEUE_OPERATION_ERROR);
      return;
    }

  m_sub_state = SubStatePrePlayStopping;
}

/*--------------------------------------------------------------------------*/
void PlayerObj::illegalSinkDone(MsgPacket *msg)
{
  msg->moveParam<OutputMixDoneParam>();
  MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::sinkDoneOnBoot(MsgPacket *msg)
{
  if (m_external_cmd_que.empty())
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
      return;
    }

  AudioCommand ext_cmd = m_external_cmd_que.top();
  if (!m_external_cmd_que.pop())
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
    }

  OutputMixDoneParam done_param = msg->moveParam<OutputMixDoneParam>();
  if (OutputMixActDone != done_param.done_type)
    {
      sendAudioCmdCmplt(ext_cmd,
                        AS_RESPONSE_CODE_INTERNAL_COMMAND_CODE_ERROR);
      return;
    }
  else
    {
      sendAudioCmdCmplt(ext_cmd, AS_RESPONSE_CODE_OK);
      m_outmix_handle = done_param.handle;
      m_state = ReadyState;
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::sinkDoneOnReady(MsgPacket *msg)
{
  if (m_external_cmd_que.empty())
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
      return;
    }

  AudioCommand ext_cmd = m_external_cmd_que.top();
  if (!m_external_cmd_que.pop())
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
    }

  OutputMixDoneParam done_param = msg->moveParam<OutputMixDoneParam>();
  if (OutputMixDeactDone != done_param.done_type)
    {
      sendAudioCmdCmplt(ext_cmd,
                        AS_RESPONSE_CODE_INTERNAL_COMMAND_CODE_ERROR);
      return;
    }
  else
    {
      sendAudioCmdCmplt(ext_cmd, AS_RESPONSE_CODE_OK);
      m_state = BootedState;
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::sinkDoneOnPlay(MsgPacket *msg)
{
  OutputMixDoneParam done_param = msg->moveParam<OutputMixDoneParam>();
  if (OutputMixExecDone != done_param.done_type)
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return;
    }

  if (!m_decoded_pcm_mh_que.empty())
    {
      sendPcmToOutputMix(m_decoded_pcm_mh_que.top());
      if (!m_decoded_pcm_mh_que.pop())
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
        }
    }

  if ((MemMgrLite::Manager::getPoolNumAvailSegs(m_es_pool_id) > 0) &&
        (MemMgrLite::Manager::getPoolNumAvailSegs(m_pcm_pool_id) > 1))
    {
      /* Do next decoding process. */

      uint32_t es_size = m_max_es_buff_size;
      void* es_addr = getEs(&es_size);

      if (es_addr != NULL)
        {
          decode(es_addr, es_size);
        }
      else
        {
          stopPlay();
          if (m_state == PlayState)
            {
              m_state = UnderflowState;
              MEDIA_PLAYER_WARN(AS_ATTENTION_SUB_CODE_SIMPLE_FIFO_UNDERFLOW);
            }
          else
            {
              m_state = StoppingState;
            }
        }
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::sinkDoneOnStopping(MsgPacket *msg)
{
  OutputMixDoneParam done_param = msg->moveParam<OutputMixDoneParam>();

  if (OutputMixStopDone == done_param.done_type)
    {
      finalize();

      if (m_external_cmd_que.empty())
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
          return;
        }

      AudioCommand ext_cmd = m_external_cmd_que.top();
      if (!m_external_cmd_que.pop())
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
        }
      sendAudioCmdCmplt(ext_cmd, AS_RESPONSE_CODE_OK);

      m_state = ReadyState;
    }
  else if (OutputMixExecDone == done_param.done_type)
    {
      sendPcmToOutputMixOnDecStopping();
    }
  else
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::sinkDoneOnWaitEsEnd(MsgPacket *msg)
{
  sinkDoneOnPlay(msg);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::sinkDoneOnUnderflow(MsgPacket *msg)
{
  OutputMixDoneParam done_param = msg->moveParam<OutputMixDoneParam>();

  if (OutputMixStopDone == done_param.done_type)
    {
      finalize();

      m_state = WaitStopState;
    }
  else if (OutputMixExecDone == done_param.done_type)
    {
      sendPcmToOutputMixOnDecStopping();
    }
  else
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::illegalDecDone(MsgPacket *msg)
{
  msg->moveParam<DecCmpltParam>();

  MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::decDoneOnPlay(MsgPacket *msg)
{
  DecCmpltParam cmplt = msg->moveParam<DecCmpltParam>();
  OutputMixObjInputDataCmd data;

  AS_decode_recv_done(m_p_dec_instance);
  freeEsBuf();

  data.handle   = m_outmix_handle;
  data.mh       = m_pcm_buf_mh_que.top();
  data.size     = cmplt.exec_dec_cmplt.output_buffer.size;
  data.is_valid = ((data.size == 0) ?
                    false : cmplt.exec_dec_cmplt.is_valid_frame);

  if (!m_decoded_pcm_mh_que.push(data))
    {
      MEDIA_PLAYER_FATAL(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return;
    }

  freePcmBuf();

  if ((MemMgrLite::Manager::getPoolNumAvailSegs(m_es_pool_id) > 0) &&
        (MemMgrLite::Manager::getPoolNumAvailSegs(m_pcm_pool_id) > 1))
    {
      /* Do next decoding process. */

      uint32_t es_size = m_max_es_buff_size;
      void    *es_addr = getEs(&es_size);

    if (es_addr != NULL)
      {
        decode(es_addr, es_size);
      }
    else
      {
        /* There is no stream data. */

        stopPlay();
        if (m_state == PlayState)
          {
            m_state = UnderflowState;
            MEDIA_PLAYER_WARN(AS_ATTENTION_SUB_CODE_SIMPLE_FIFO_UNDERFLOW);
          }
        else
          {
            m_state = StoppingState;
          }
      }
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::decDoneOnWaitEsEnd(MsgPacket *msg)
{
  decDoneOnPlay(msg);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::decDoneOnWaitStop(MsgPacket *msg)
{
  DecCmpltParam cmplt = msg->moveParam<DecCmpltParam>();
  OutputMixObjInputDataCmd data;

  AS_decode_recv_done(m_p_dec_instance);

  data.handle = m_outmix_handle;
  data.mh     = m_pcm_buf_mh_que.top();

  if (Apu::ExecEvent == cmplt.event_type)
    {
      freeEsBuf();

      data.size     = cmplt.exec_dec_cmplt.output_buffer.size;
      data.is_valid = ((data.size == 0) ?
                       false : cmplt.exec_dec_cmplt.is_valid_frame);
    }
  else if (Apu::FlushEvent == cmplt.event_type)
    {
      data.size = cmplt.stop_dec_cmplt.output_buffer.size;
      data.is_valid = cmplt.stop_dec_cmplt.is_valid_frame;
      data.is_es_end = true;
    }

  if (!m_decoded_pcm_mh_que.push(data))
    {
      MEDIA_PLAYER_FATAL(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return;
    }

    freePcmBuf();
}

/*--------------------------------------------------------------------------*/
void PlayerObj::decDoneOnPrePlay(MsgPacket *msg)
{
  DecCmpltParam cmplt = msg->moveParam<DecCmpltParam>();
  OutputMixObjInputDataCmd data;

  AS_decode_recv_done(m_p_dec_instance);
  freeEsBuf();

  data.handle   = m_outmix_handle;
  data.mh       = m_pcm_buf_mh_que.top();
  data.size     = cmplt.exec_dec_cmplt.output_buffer.size;
  data.is_valid = ((data.size == 0) ?
                   false : cmplt.exec_dec_cmplt.is_valid_frame);

  if (data.size != 0)
    {
      if (!m_decoded_pcm_mh_que.push(data))
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
          return;
        }
    }

  freePcmBuf();

  if (m_decoded_pcm_mh_que.size() < MAX_EXEC_COUNT)
    {
      uint32_t es_size = m_max_es_buff_size;
      void    *es_addr = getEs(&es_size);

    if (es_addr != NULL)
      {
        decode(es_addr, es_size);
      }
    else
      {
        stopPlay();
        if (m_sub_state == SubStatePrePlay)
          {
            m_sub_state = SubStatePrePlayUnderflow;
            MEDIA_PLAYER_WARN(AS_ATTENTION_SUB_CODE_SIMPLE_FIFO_UNDERFLOW);
          }
        else
          {
            m_sub_state = SubStatePrePlayStopping;
          }
      }
    }
  else
    {
      while(!m_decoded_pcm_mh_que.empty())
        {
          sendPcmToOutputMix(m_decoded_pcm_mh_que.top());
          if (!m_decoded_pcm_mh_que.pop())
            {
              MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
              break;
            }
        }

      uint32_t es_size = m_max_es_buff_size;
      void    *es_addr = getEs(&es_size);

      if (es_addr != NULL)
        {
          decode(es_addr, es_size);

          if (m_sub_state == SubStatePrePlay)
            {
              m_state = PlayState;
            }
          else
            {
              m_state = WaitEsEndState;
            }

            m_sub_state = InvalidSubState;
        }
      else
        {
          stopPlay();
          if (m_sub_state == SubStatePrePlay)
            {
              m_state = UnderflowState;
              MEDIA_PLAYER_WARN(AS_ATTENTION_SUB_CODE_SIMPLE_FIFO_UNDERFLOW);
            }
          else
            {
              m_state = StoppingState;
            }
        }
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::decDoneOnPrePlayStopping(MsgPacket *msg)
{
  /* Free all resource and send stop completion. */

  DecCmpltParam cmplt = msg->moveParam<DecCmpltParam>();
  AS_decode_recv_done(m_p_dec_instance);
  freePcmBuf();

  if (Apu::ExecEvent == cmplt.event_type)
    {
      freeEsBuf();
    }
  else if (Apu::FlushEvent == cmplt.event_type)
    {
      /* Free all decoded pcm MHandle. */

      while (!m_decoded_pcm_mh_que.empty())
        {
          if (!m_decoded_pcm_mh_que.pop())
            {
              MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
              break;
            }
        }

      finalize();

      if (m_external_cmd_que.empty())
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
          return;
        }

      AudioCommand ext_cmd = m_external_cmd_que.top();
      if (!m_external_cmd_que.pop())
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
        }
      sendAudioCmdCmplt(ext_cmd, AS_RESPONSE_CODE_OK);

      m_sub_state = InvalidSubState;
      m_state = ReadyState;
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::decDoneOnPrePlayWaitEsEnd(MsgPacket *msg)
{
  decDoneOnPrePlay(msg);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::decDoneOnPrePlayUnderflow(MsgPacket *msg)
{
  /* Free all resource. */

  DecCmpltParam cmplt = msg->moveParam<DecCmpltParam>();
  AS_decode_recv_done(m_p_dec_instance);
  freePcmBuf();

  if (Apu::ExecEvent == cmplt.event_type)
    {
      freeEsBuf();
    }
  else if (Apu::FlushEvent == cmplt.event_type)
    {
      /* Free all decoded pcm MHandle. */

      while (!m_decoded_pcm_mh_que.empty())
        {
          if (!m_decoded_pcm_mh_que.pop())
            {
              MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
              break;
            }
        }
      finalize();

      m_sub_state = InvalidSubState;
      m_state = WaitStopState;
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::setClkRecovery(MsgPacket *msg)
{
  AudioCommand cmd = msg->moveParam<AudioCommand>();
  OutputMixObjParam outmix_param;
  err_t er;

  MEDIA_PLAYER_DBG("SET CLOCK RECOVERY: dir %d, times %d\n",
                   cmd.clk_recovery_param.direction,
                   cmd.clk_recovery_param.times);

  /* Set paramete to OutputMixer. */

  outmix_param.handle                    = m_outmix_handle;
  outmix_param.adjust_param.direction    = cmd.clk_recovery_param.direction;
  outmix_param.adjust_param.adjust_times = cmd.clk_recovery_param.times;

  er = MsgLib::send<OutputMixObjParam>(m_output_mix_dtq,
                                       MsgPriNormal,
                                       MSG_AUD_MIX_CMD_CLKRECOVERY,
                                       m_self_dtq,
                                       outmix_param);
  F_ASSERT(er == ERR_OK);

  /* Send result. */

  sendAudioCmdCmplt(cmd, AS_RESPONSE_CODE_OK);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::parseSubState(MsgPacket *msg)
{
  uint event  = MSG_GET_SUBTYPE(msg->getType());

  if (PlayerObj::InvalidSubState == m_sub_state.get())
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_INTERNAL_STATE_ERROR);
      return;
    }

  (this->*PlayerSubStateTbl[event][m_sub_state.get()])(msg);
}

/*--------------------------------------------------------------------------*/
uint32_t PlayerObj::loadCodec(AudioCodec codec, uint32_t* dsp_inf)
{
  uint32_t rst;

  switch  (codec)
    {
      case AudCodecMP3:
      case AudCodecXAVCLPCM:
      case AudCodecAAC:
      case AudCodecOPUS:
        break;
      default:
        return AS_RESPONSE_CODE_COMMAND_PARAM_CODEC_TYPE;
    }

  if (m_codec_type != InvalidCodecType)
    {
      return AS_RESPONSE_CODE_COMMAND_PARAM_CODEC_TYPE;
    }

  rst = AS_decode_activate(codec,
                           &m_p_dec_instance,
                           m_apu_pool_id,
                           m_apu_dtq,
                           dsp_inf);
  if (rst != AS_RESPONSE_CODE_OK)
    {
      return rst;
    }

  m_codec_type = codec;

  return rst;
}

/*--------------------------------------------------------------------------*/
bool PlayerObj::unloadCodec(void)
{
  /* Only unload when codec is loaded. */

  if (m_codec_type != InvalidCodecType)
    {
      if (!AS_decode_deactivate(m_p_dec_instance))
        {
          return false;
        }
      m_codec_type = InvalidCodecType;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
uint32_t PlayerObj::startPlay(uint32_t* dsp_inf)
{
  uint32_t   rst = AS_RESPONSE_CODE_OK;
  AudioCodec codec_type_of_next_track;
  InitDecCompParam init_dec_comp_param;
  AsClkModeId      clock_mode;

  rst = m_input_device_handler->start();
  if (rst != AS_RESPONSE_CODE_OK)
    {
      return rst;
    }

  /* Update codec type per track. */

  codec_type_of_next_track = m_input_device_handler->getCodecType();
  if (m_codec_type != codec_type_of_next_track)
    {
      if(!unloadCodec())
        {
          return AS_RESPONSE_CODE_DSP_UNLOAD_ERROR;
        }

      rst = loadCodec(codec_type_of_next_track, dsp_inf);
      if(rst != AS_RESPONSE_CODE_OK)
        {
          return rst;
        }
    }

  init_dec_comp_param.codec_type          = m_codec_type;
  init_dec_comp_param.input_sampling_rate =
      m_input_device_handler->getSamplingRate();
  init_dec_comp_param.channel_num         =
      ((m_input_device_handler->getChannelNum() == AS_INITPLAYER_CHNL_STEREO) ?
      (TwoChannels) : (MonoChannels));
  init_dec_comp_param.frame_sample_num    =
      m_input_device_handler->getSampleNumPerFrame();
  init_dec_comp_param.callback            = &decoder_comp_done_callback;
  init_dec_comp_param.p_requester         = static_cast<void*>(this);

  /* TODO: delete fixed value. */

  clock_mode = GetClkMode();
  if (AS_CLK_MODE_HIRES == clock_mode)
    {
      init_dec_comp_param.bit_width = AudPcm24Bit;
    }
  else
    {
      init_dec_comp_param.bit_width = AudPcm16Bit;
    }

  rst = AS_decode_init(init_dec_comp_param, m_p_dec_instance, dsp_inf);
  if (rst != AS_RESPONSE_CODE_OK)
    {
      return rst;
    }

  if (!AS_decode_recv_done(m_p_dec_instance))
    {
      return AS_RESPONSE_CODE_QUEUE_OPERATION_ERROR;
    }

  for (int i=0; i < MAX_EXEC_COUNT; i++)
    {
      /* Get ES data. */

      uint32_t es_size = m_max_es_buff_size;
      void    *es_addr = getEs(&es_size);

      if (es_addr == NULL)
        {
          return  AS_RESPONSE_CODE_SIMPLE_FIFO_UNDERFLOW;
        }

    decode(es_addr, es_size);
  }
  return AS_RESPONSE_CODE_OK;
}

/*--------------------------------------------------------------------------*/
void PlayerObj::stopPlay(void)
{
  StopDecCompParam param;
  param.output_buffer.size = m_max_pcm_buff_size;
  param.output_buffer.p_buffer = reinterpret_cast<unsigned long*>
    (allocPcmBuf(m_max_pcm_buff_size));

  if (param.output_buffer.p_buffer != NULL)
    {
      if (AS_decode_stop(param, m_p_dec_instance) == false)
        {
          /* Do nothing. */
        }
    }

  m_input_device_handler->stop();
}

/*--------------------------------------------------------------------------*/
void PlayerObj::sendPcmToOutputMix(const OutputMixObjInputDataCmd& data)
{
  err_t er;

  if (data.size == 0)
    {
      MEDIA_PLAYER_INF(AS_ATTENTION_SUB_CODE_DECODED_SIZE_ZERO);
      return;
    }

  er = MsgLib::send<OutputMixObjInputDataCmd>(m_output_mix_dtq,
                                              MsgPriNormal,
                                              MSG_AUD_MIX_CMD_DATA,
                                              m_self_dtq,
                                              data);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::stopOutputMix(void)
{
  OutputMixObjParam outmix_param;
  err_t er;

  outmix_param.handle = m_outmix_handle;

  MsgLib::send<OutputMixObjParam>(m_output_mix_dtq,
                                  MsgPriNormal,
                                  MSG_AUD_MIX_CMD_STOP,
                                  m_self_dtq,
                                  outmix_param);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------------*/
void PlayerObj::decode(void* p_es, uint32_t es_size)
{
  void *p_pcm = allocPcmBuf(m_max_pcm_buff_size);
  ExecDecCompParam param;

  if (p_pcm == NULL)
    {
      return;
    }

  param.input_buffer.p_buffer  = reinterpret_cast<unsigned long *>(p_es);
  param.input_buffer.size      = es_size;
  param.output_buffer.p_buffer = reinterpret_cast<unsigned long *>(p_pcm);
  param.output_buffer.size     = m_max_pcm_buff_size;
  param.num_of_au              = NUM_OF_AU;

  if (AS_decode_exec(param, m_p_dec_instance) == false)
    {
      /* Do nothing. */
    }
}

/*--------------------------------------------------------------------------*/
void* PlayerObj::allocPcmBuf(uint32_t size)
{
  MemMgrLite::MemHandle mh;
  if (mh.allocSeg(m_pcm_pool_id, size) != ERR_OK)
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return NULL;
    }
  if (!m_pcm_buf_mh_que.push(mh))
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return NULL;
    }

  return mh.getPa();
}

/*--------------------------------------------------------------------------*/
void* PlayerObj::getEs(uint32_t* size)
{
  MemMgrLite::MemHandle mh;

  if (mh.allocSeg(m_es_pool_id, *size) != ERR_OK)
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return NULL;
    }

  if (m_input_device_handler->getEs(mh.getVa(), size))
    {
      if (!m_es_buf_mh_que.push(mh))
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
          return NULL;
        }
    return mh.getPa();
  }

  return NULL;
}

/*--------------------------------------------------------------------------*/
void PlayerObj::sendPcmToOutputMixOnDecStopping()
{
  if (!m_decoded_pcm_mh_que.empty())
    {
      sendPcmToOutputMix(m_decoded_pcm_mh_que.top());
      if (m_decoded_pcm_mh_que.top().is_es_end)
        {
          stopOutputMix();
        }
    if (!m_decoded_pcm_mh_que.pop())
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
        }
    }
}

/*--------------------------------------------------------------------------*/
void PlayerObj::finalize()
{
  /* Note:
   *   This queues should be EMPTY. If not, there exist any bug.
   *   (error debug log will be showed as following)
   *   If it happens, clear the queues to prevent system crash.
   */

  while (!m_decoded_pcm_mh_que.empty())
    {
      MEDIA_PLAYER_INF(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
      if (!m_decoded_pcm_mh_que.pop())
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
          break;
        }
    }

  while (!m_es_buf_mh_que.empty())
    {
      MEDIA_PLAYER_INF(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
      if (!m_es_buf_mh_que.pop())
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
          break;
        }
    }

  while (!m_pcm_buf_mh_que.empty())
    {
      MEDIA_PLAYER_INF(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
      if (!m_pcm_buf_mh_que.pop())
        {
          MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
          break;
        }
    }
}

/*--------------------------------------------------------------------------*/
bool PlayerObj::checkAndSetMemPool()
{
  if (!MemMgrLite::Manager::isPoolAvailable(m_es_pool_id))
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }
  m_max_es_buff_size = (MemMgrLite::Manager::getPoolSize(m_es_pool_id)) /
    (MemMgrLite::Manager::getPoolNumSegs(m_es_pool_id));

  if (!MemMgrLite::Manager::isPoolAvailable(m_pcm_pool_id))
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }
  m_max_pcm_buff_size = (MemMgrLite::Manager::getPoolSize(m_pcm_pool_id)) /
    (MemMgrLite::Manager::getPoolNumSegs(m_pcm_pool_id));

  if (!MemMgrLite::Manager::isPoolAvailable(m_apu_pool_id))
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }
  if ((int)(sizeof(Apu::Wien2ApuCmd)) >
      (MemMgrLite::Manager::getPoolSize(m_apu_pool_id))/
      (MemMgrLite::Manager::getPoolNumSegs(m_apu_pool_id)))
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return false;
    }
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C"
{
/*--------------------------------------------------------------------------*/
int AS_PlayerObjEntry(int argc, char *argv[])
{
  PlayerObj::create(&s_play_obj,
                    s_self_dtq,
                    s_manager_dtq,
                    s_omix_dtq,
                    s_dsp_dtq,
                    s_es_pool_id,
                    s_pcm_pool_id,
                    s_apu_pool_id);
  return 0;
}

/*--------------------------------------------------------------------------*/
int AS_SubPlayerObjEntry(int argc, char *argv[])
{
  PlayerObj::create(&s_sub_play_obj,
                    s_sub_self_dtq,
                    s_sub_manager_dtq,
                    s_sub_omix_dtq,
                    s_sub_dsp_dtq,
                    s_sub_es_pool_id,
                    s_sub_pcm_pool_id,
                    s_sub_apu_pool_id);
  return 0;
}

/*--------------------------------------------------------------------------*/
bool AS_ActivatePlayer(FAR AsActPlayerParam_t *param)
{
  s_self_dtq    = param->msgq_id.player;
  s_manager_dtq = param->msgq_id.mng;
  s_omix_dtq    = param->msgq_id.mixer;
  s_dsp_dtq     = param->msgq_id.dsp;
  s_es_pool_id  = param->pool_id.es;
  s_pcm_pool_id = param->pool_id.pcm;
  s_apu_pool_id = param->pool_id.dsp;

  s_ply_pid = task_create("PLY_OBJ",
                          150, 1024 * 3,
                          AS_PlayerObjEntry,
                          NULL);
  if (s_ply_pid < 0)
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_ActivateSubPlayer(FAR AsActPlayerParam_t *param)
{
  s_sub_self_dtq    = param->msgq_id.player;
  s_sub_manager_dtq = param->msgq_id.mng;
  s_sub_omix_dtq    = param->msgq_id.mixer;
  s_sub_dsp_dtq     = param->msgq_id.dsp;
  s_sub_es_pool_id  = param->pool_id.es;
  s_sub_pcm_pool_id = param->pool_id.pcm;
  s_sub_apu_pool_id = param->pool_id.dsp;

  s_sub_ply_pid = task_create("SUB_PLY_OBJ",
                              150, 1024 * 3,
                              AS_SubPlayerObjEntry,
                              NULL);
  if (s_sub_ply_pid < 0)
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeactivatePlayer(void)
{
  if (s_ply_pid < 0)
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return false;
    }

  task_delete(s_ply_pid);

  if (s_play_obj != NULL)
    {
      delete ((PlayerObj *)s_play_obj);
      s_play_obj = NULL;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeactivateSubPlayer(void)
{
  if (s_sub_play_obj == NULL)
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return false;
    }

  task_delete(s_sub_ply_pid);
  delete ((PlayerObj *)s_sub_play_obj);
  s_sub_play_obj = NULL;
  return true;
}
} /* extern "C" */

void PlayerObj::create(FAR void **obj,
                       MsgQueId self_dtq,
                       MsgQueId manager_dtq,
                       MsgQueId output_mix_dtq,
                       MsgQueId apu_dtq,
                       MemMgrLite::PoolId es_pool_id,
                       MemMgrLite::PoolId pcm_pool_id,
                       MemMgrLite::PoolId apu_pool_id)
{
  FAR PlayerObj *player_obj = new PlayerObj(self_dtq,
                                            manager_dtq,
                                            output_mix_dtq,
                                            apu_dtq,
                                            es_pool_id,
                                            pcm_pool_id,
                                            apu_pool_id);

  if (player_obj != NULL)
    {
      *obj = reinterpret_cast<void*>(player_obj);
      player_obj->run();
    }
  else
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
    }
}
