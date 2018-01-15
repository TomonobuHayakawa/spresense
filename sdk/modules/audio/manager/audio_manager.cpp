/****************************************************************************
 * modules/audio/manager/audio_manager.cpp
 *
 *   Copyright (C) 2015-2017 Sony Corporation. All rights reserved.
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

#include "memutils/common_utils/common_types.h"
#include "memutils/common_utils/common_attention.h"
#include "audio_manager.h"
#ifdef AS_FEATURE_RECOGNIZER_ENABLE
#include "objects/sound_recognizer/voice_recognition_object.h"
#endif
#include "memutils/os_utils/chateau_osal.h"
#include <arch/chip/cxd56_audio.h>
#include "debug/dbg_log.h"

#define DBG_MODULE DBG_MODULE_AS

#include <nuttx/arch.h>

__USING_WIEN2

static MsgQueId s_selfMid    = 0xFF;  /* Invalid ID. */
static MsgQueId s_appMid     = 0xFF;  /* Invalid ID. */
static MsgQueId s_plyMainMid = 0xFF;  /* Invalid ID. */
static MsgQueId s_plySubMid  = 0xFF;  /* Invalid ID. */
static MsgQueId s_mixSubMid  = 0xFF;  /* Invalid ID. */
static MsgQueId s_rcdSubMid  = 0xFF;  /* Invalid ID. */
static MsgQueId s_effectMid  = 0xFF;  /* Invalid ID. */
static MsgQueId s_rcgMid     = 0xFF;  /* Invalid ID. */

static AudioManager *s_mng = NULL;

/*
 * External Interface.
 */

extern "C" {

/*--------------------------------------------------------------------------*/
void AS_SendAudioCommand(FAR AudioCommand *packet)
{
  MSG_TYPE msg_type;

  switch (packet->getCode())
    {
      case AUDCMD_POWERON:
        msg_type = MSG_AUD_MGR_CMD_POWERON;
        break;

      case AUDCMD_SETPOWEROFFSTATUS:
        msg_type = MSG_AUD_MGR_CMD_POWEROFF;
        break;

      case AUDCMD_INITMICGAIN:
        msg_type = MSG_AUD_MGR_CMD_INITMICGAIN;
        break;

      case AUDCMD_INITI2SPARAM:
        msg_type = MSG_AUD_MGR_CMD_INITI2SPARAM;
        break;

      case AUDCMD_INITDEQPARAM:
        msg_type = MSG_AUD_MGR_CMD_INITDEQPARAM;
        break;

      case AUDCMD_INITOUTPUTSELECT:
        msg_type = MSG_AUD_MGR_CMD_INITOUTPUTSELECT;
        break;

      case AUDCMD_INITDNCPARAM:
        msg_type = MSG_AUD_MGR_CMD_INITDNCPARAM;
        break;

      case AUDCMD_INITCLEARSTEREO:
        msg_type = MSG_AUD_MGR_CMD_INITCLEARSTEREO;
        break;

      case AUDCMD_SETVOLUME:
        msg_type = MSG_AUD_MGR_CMD_SETVOLUME;
        break;

      case AUDCMD_SETVOLUMEMUTE:
        msg_type = MSG_AUD_MGR_CMD_SETVOLUMEMUTE;
        break;

      case AUDCMD_SETBEEPPARAM:
        msg_type = MSG_AUD_MGR_CMD_SETBEEP;
        break;

      case AUDCMD_GETSTATUS:
        msg_type = MSG_AUD_MGR_CMD_GETSTATUS;
        break;

#ifdef AS_FEATURE_PLAYER_ENABLE
      case AUDCMD_INITPLAYER:
      case AUDCMD_PLAYPLAYER:
      case AUDCMD_STOPPLAYER:
      case AUDCMD_CLKRECOVERY:
        msg_type = MSG_AUD_MGR_CMD_PLAYER;
        break;

      case AUDCMD_INITSUBPLAYER:
      case AUDCMD_PLAYSUBPLAYER:
      case AUDCMD_STOPSUBPLAYER:
      case AUDCMD_CLKRECOVERYSUB:
        msg_type = MSG_AUD_MGR_CMD_SUBPLAYER;
        break;

#endif  /* AS_FEATURE_PLAYER_ENABLE */
#ifdef AS_FEATURE_RECORDER_ENABLE
      case AUDCMD_INITREC:
      case AUDCMD_STARTREC:
      case AUDCMD_STOPREC:
        msg_type = MSG_AUD_MGR_CMD_RECORDER;
        break;

#endif  /* AS_FEATURE_RECORDER_ENABLE */
#ifdef AS_FEATURE_RECOGNIZER_ENABLE
      case AUDCMD_STARTVOICECOMMAND:
      case AUDCMD_STOPVOICECOMMAND:
        msg_type = MSG_AUD_MGR_CMD_VOICECOMMAND;
        break;

#endif  /* AS_FEATURE_RECOGNIZER_ENABLE */
#ifdef AS_FEATURE_EFFECTOR_ENABLE
      case AUDCMD_STARTBB:
      case AUDCMD_STOPBB:
        msg_type = MSG_AUD_MGR_CMD_SOUNDFX;
        break;

      case AUDCMD_INITMFE:
      case AUDCMD_DEBUGMFEPARAM:
        msg_type = MSG_AUD_MGR_CMD_MFE;
        break;

      case AUDCMD_INITMPP:
      case AUDCMD_SETMPPPARAM:
      case AUDCMD_DEBUGMPPPARAM:
        msg_type = MSG_AUD_MGR_CMD_MPP;
        break;

#endif  /* AS_FEATURE_EFFECTOR_ENABLE */

      case AUDCMD_SETREADYSTATUS:
        msg_type = MSG_AUD_MGR_CMD_SETREADY;
        break;

      case AUDCMD_SETBBACTIVESTATUS:
        msg_type = MSG_AUD_MGR_CMD_SETACTIVE;
        break;

      case AUDCMD_SETPLAYERSTATUS:
        msg_type = MSG_AUD_MGR_CMD_SETPLAYER;
        break;

      case AUDCMD_SETRECORDERSTATUS:
        msg_type = MSG_AUD_MGR_CMD_SETRECORDER;
        break;

      case AUDCMD_SETTHROUGHSTATUS:
        msg_type = MSG_AUD_MGR_CMD_SETTHROUGH;
        break;

      case AUDCMD_SETTHROUGHPATH:
        msg_type = MSG_AUD_MGR_CMD_SETTHROUGHPATH;
        break;

      case AUDCMD_INITATTENTIONS:
        msg_type = MSG_AUD_MGR_CMD_INITATTENTIONS;
        break;

      default:
        msg_type = MSG_AUD_MGR_CMD_INVALID;
        break;
    }

  err_t er = MsgLib::send<AudioCommand>(s_selfMid,
                                        MsgPriNormal,
                                        msg_type,
                                        s_selfMid,
                                        *packet);
  if (er)
    {
      F_ASSERT(0);
    }
}

static pid_t s_amng_pid = -1;

#define AUDIO_TASK_PRIORITY 100
#define AUDIO_TASK_MANAGER_STACK_SIZE (1024 * 2)

/*--------------------------------------------------------------------------*/
int AS_AudioManagerEntry(void)
{
  AudioManager::create(s_selfMid, s_plyMainMid, s_plySubMid);
  return 0;
}

/*--------------------------------------------------------------------------*/
void AS_ActivateAudioSubSystem(AudioSubSystemIDs ids)
{
  s_selfMid    = (MsgQueId)ids.mng;
  s_appMid     = (MsgQueId)ids.app;
  s_plyMainMid = (MsgQueId)ids.player_main;
  s_plySubMid  = (MsgQueId)ids.player_sub;
  s_mixSubMid  = (MsgQueId)ids.mixer;
  s_rcdSubMid  = (MsgQueId)ids.recorder;
  s_effectMid  = (MsgQueId)ids.effector;
  s_rcgMid     = (MsgQueId)ids.recognizer;

  s_amng_pid = task_create("AMNG",
                           AUDIO_TASK_PRIORITY,
                           AUDIO_TASK_MANAGER_STACK_SIZE,
                           (main_t)AS_AudioManagerEntry,
                           0);
  if (s_amng_pid < 0)
    {
      _err("ERROR AS_ActivateAudioSubSystem failed\n");
      return;
    }
}

/*--------------------------------------------------------------------------*/
void AS_DeactivateAudioSubSystem(void)
{
  if (s_amng_pid < 0)
    {
      MANAGER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
    }

  task_delete(s_amng_pid);

  if (s_mng != NULL)
    {
      delete s_mng;
      s_mng = NULL;
    }
}

/*--------------------------------------------------------------------------*/
void AS_ReceiveAudioResult(FAR AudioResult *packet)
{
  err_t           err_code;
  FAR MsgQueBlock *que;
  FAR MsgPacket   *msg;

  err_code = MsgLib::referMsgQueBlock(s_appMid, &que);
  F_ASSERT(err_code == ERR_OK);

  err_code = que->recv(TIME_FOREVER, &msg);
  F_ASSERT(err_code == ERR_OK);
  F_ASSERT(msg->getType() == MSG_AUD_MGR_RST);

  *packet = msg->moveParam<AudioResult>();
  err_code = que->pop();
  F_ASSERT(err_code == ERR_OK);
}

/*--------------------------------------------------------------------------*/
MsgQueId AS_GetSelfDtq(void)
{
  return s_selfMid;
}

} /* extern "C" */

/*--------------------------------------------------------------------------*/
void AudioManager::create(MsgQueId selfDtq,
                          MsgQueId playerDtq,
                          MsgQueId subplayerDtq)
{
  if (s_mng == NULL)
    {
      s_mng = new AudioManager(selfDtq, playerDtq, subplayerDtq);
      if (s_mng == NULL)
        {
          MANAGER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
          return;
        }
      s_selfMid = s_mng->m_selfDtq;
      s_mng->run();
    }
  else
    {
      F_ASSERT(0);
    }
}

/*--------------------------------------------------------------------------*/
void AudioManager::run(void)
{
  err_t           err_code;
  FAR MsgQueBlock *que;
  FAR MsgPacket   *msg;

  err_code = MsgLib::referMsgQueBlock(m_selfDtq, &que);
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
AudioManager::MsgProc
  AudioManager::MsgProcTbl[AUD_MGR_MSG_NUM][MNG_ALLSTATE_NUM] =
{
  /* Player command. */

  {                                    /* AudioManager all status: */
    &AudioManager::illegal,            /*   Ready state.           */
    &AudioManager::player,             /*   PlayerReady state.     */
    &AudioManager::player,             /*   PlayerActive state.    */
    &AudioManager::player,             /*   PlayerPause state.     */
    &AudioManager::illegal,            /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::illegal,            /*   BasebandReady state.   */
    &AudioManager::illegal,            /*   BasebandActive state.  */
    &AudioManager::illegal,            /*   WaitCommandWord state. */
    &AudioManager::illegal,            /*   PowerOff state.        */
    &AudioManager::illegal             /*   Through state.         */
  },

  /* SetReady command. */

  {                                    /* AudioManager all status: */
    &AudioManager::illegal,            /*   Ready state.           */
    &AudioManager::setRdyOnPlay,       /*   PlayerReady state.     */
    &AudioManager::setRdyOnPlay,       /*   PlayerActive state.    */
    &AudioManager::setRdyOnPlay,       /*   PlayerPause state.     */
    &AudioManager::setRdyOnRecorder,   /*   RecorderReady state.   */
    &AudioManager::setRdyOnRecorder,   /*   RecorderActive state.  */
    &AudioManager::setRdyOnAct,        /*   BasebandReady state.   */
    &AudioManager::setRdyOnAct,        /*   BasebandActive state.  */
    &AudioManager::setRdyOnAct,        /*   WaitCommandWord state. */
    &AudioManager::illegal,            /*   PowerOff state.        */
    &AudioManager::setRdyOnThrough     /*   Through state.         */
  },

  /* SetActive command. */

  {                                    /* AudioManager all status: */
    &AudioManager::setActive,          /*   Ready state.           */
    &AudioManager::illegal,            /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::illegal,            /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::illegal,            /*   BasebandReady state.   */
    &AudioManager::illegal,            /*   BasebandActive state.  */
    &AudioManager::illegal,            /*   WaitCommandWord state. */
    &AudioManager::illegal,            /*   PowerOff state.        */
    &AudioManager::illegal             /*   Through state.         */
  },

  /* SetPlayer command. */

  {                                    /* AudioManager all status: */
    &AudioManager::setPlayerStatus,    /*   Ready state.           */
    &AudioManager::illegal,            /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::illegal,            /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::illegal,            /*   BasebandReady state.   */
    &AudioManager::illegal,            /*   BasebandActive state.  */
    &AudioManager::illegal,            /*   WaitCommandWord state. */
    &AudioManager::illegal,            /*   PowerOff state.        */
    &AudioManager::illegal             /*   Through state.         */
  },

  /* Voice command. */

  {                                    /* AudioManager all status: */
    &AudioManager::illegal,            /*   Ready state.           */
    &AudioManager::illegal,            /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::illegal,            /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::voiceCommand,       /*   BasebandReady state.   */
    &AudioManager::voiceCommand,       /*   BasebandActive state.  */
    &AudioManager::voiceCommand,       /*   WaitCommandWord state. */
    &AudioManager::illegal,            /*   PowerOff state.        */
    &AudioManager::illegal             /*   Through state.         */
  },

  /* SetRecorder command. */

  {                                    /* AudioManager all status: */
    &AudioManager::setRecorder,        /*   Ready state.           */
    &AudioManager::illegal,            /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::illegal,            /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::illegal,            /*   BasebandReady state.   */
    &AudioManager::illegal,            /*   BasebandActive state.  */
    &AudioManager::illegal,            /*   WaitCommandWord state. */
    &AudioManager::illegal,            /*   PowerOff state.        */
    &AudioManager::illegal             /*   Through state.         */
  },

  /* Recorder command. */

  {                                    /* AudioManager all status: */
    &AudioManager::illegal,            /*   Ready state.           */
    &AudioManager::illegal,            /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::recorder,           /*   RecorderReady state.   */
    &AudioManager::recorder,           /*   RecorderActive state.  */
    &AudioManager::illegal,            /*   BasebandReady state.   */
    &AudioManager::illegal,            /*   BasebandActive state.  */
    &AudioManager::illegal,            /*   WaitCommandWord state. */
    &AudioManager::illegal,            /*   PowerOff state.        */
    &AudioManager::illegal             /*   Through state.         */
  },

  /* SoundEffect command. */

  {                                    /* AudioManager all status: */
    &AudioManager::illegal,            /*   Ready state.           */
    &AudioManager::illegal,            /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::illegal,            /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::soundFx,            /*   BasebandReady state.   */
    &AudioManager::soundFx,            /*   BasebandActive state.  */
    &AudioManager::soundFx,            /*   WaitCommandWord state. */
    &AudioManager::illegal,            /*   PowerOff state.        */
    &AudioManager::illegal             /*   Through state.         */
  },

  /* MicrophoneFrontEnd command. */

  {                                    /* AudioManager all status: */
    &AudioManager::illegal,            /*   Ready state.           */
    &AudioManager::illegal,            /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::illegal,            /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::mfe,                /*   BasebandReady state.   */
    &AudioManager::mfe,                /*   BasebandActive state.  */
    &AudioManager::mfe,                /*   WaitCommandWord state. */
    &AudioManager::illegal,            /*   PowerOff state.        */
    &AudioManager::illegal             /*   Through state.         */
  },

  /* MediaPlayerPost command. */

  {                                    /* AudioManager all status: */
    &AudioManager::illegal,            /*   Ready state.           */
    &AudioManager::mpp,                /*   PlayerReady state.     */
    &AudioManager::mpp,                /*   PlayerActive state.    */
    &AudioManager::mpp,                /*   PlayerPause state.     */
    &AudioManager::illegal,            /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::mpp,                /*   BasebandReady state.   */
    &AudioManager::mpp,                /*   BasebandActive state.  */
    &AudioManager::mpp,                /*   WaitCommandWord state. */
    &AudioManager::illegal,            /*   PowerOff state.        */
    &AudioManager::illegal             /*   Through state.         */
  },

  /* GetStaus command. */

  {                                    /* AudioManager all status: */
    &AudioManager::getstatus,          /*   Ready state.           */
    &AudioManager::getstatus,          /*   PlayerReady state.     */
    &AudioManager::getstatus,          /*   PlayerActive state.    */
    &AudioManager::getstatus,          /*   PlayerPause state.     */
    &AudioManager::getstatus,          /*   RecorderReady state.   */
    &AudioManager::getstatus,          /*   RecorderActive state.  */
    &AudioManager::getstatus,          /*   BasebandReady state.   */
    &AudioManager::getstatus,          /*   BasebandActive state.  */
    &AudioManager::getstatus,          /*   WaitCommandWord state. */
    &AudioManager::getstatus,          /*   PowerOff state.        */
    &AudioManager::getstatus           /*   Through state.         */
  },

  /* InitAttentions command. */

  {                                    /* AudioManager all status: */
    &AudioManager::initAttentions,     /*   Ready state.           */
    &AudioManager::illegal,            /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::illegal,            /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::illegal,            /*   BasebandReady state.   */
    &AudioManager::illegal,            /*   BasebandActive state.  */
    &AudioManager::illegal,            /*   WaitCommandWord state. */
    &AudioManager::initAttentions,     /*   PowerOff state.        */
    &AudioManager::illegal             /*   Through state.         */
  },

  /* InitMicGain command. */

  {                                    /* AudioManager all status: */
    &AudioManager::setMicGain,         /*   Ready state.           */
    &AudioManager::setMicGain,         /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::setMicGain,         /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::setMicGain,         /*   BasebandReady state.   */
    &AudioManager::illegal,            /*   BasebandActive state.  */
    &AudioManager::illegal,            /*   WaitCommandWord state. */
    &AudioManager::initMicGain,        /*   PowerOff state.        */
    &AudioManager::setMicGain          /*   Through state.         */
  },

  /* InitI2SPARaram command. */

  {                                    /* AudioManager all status: */
    &AudioManager::setI2SParam,        /*   Ready state.           */
    &AudioManager::setI2SParam,        /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::setI2SParam,        /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::setI2SParam,        /*   BasebandReady state.   */
    &AudioManager::illegal,            /*   BasebandActive state.  */
    &AudioManager::illegal,            /*   WaitCommandWord state. */
    &AudioManager::initI2SParam,       /*   PowerOff state.        */
    &AudioManager::setI2SParam         /*   Through state.         */
  },

  /* InitDEQParam command. */

  {                                    /* AudioManager all status: */
    &AudioManager::initDEQParam,       /*   Ready state.           */
    &AudioManager::initDEQParam,       /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::initDEQParam,       /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::initDEQParam,       /*   BasebandReady state.   */
    &AudioManager::illegal,            /*   BasebandActive state.  */
    &AudioManager::illegal,            /*   WaitCommandWord state. */
    &AudioManager::initDEQParam,       /*   PowerOff state.        */
    &AudioManager::illegal             /*   Through state.         */
  },

  /* InitOutputSelect command. */

  {                                    /* AudioManager all status: */
    &AudioManager::setOutputSelect,    /*   Ready state.           */
    &AudioManager::setOutputSelect,    /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::setOutputSelect,    /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::setOutputSelect,    /*   BasebandReady state.   */
    &AudioManager::illegal,            /*   BasebandActive state.  */
    &AudioManager::illegal,            /*   WaitCommandWord state. */
    &AudioManager::initOutputSelect,   /*   PowerOff state.        */
    &AudioManager::setOutputSelect     /*   Through state.         */
  },

  /* InitDNCParam command. */

  {                                    /* AudioManager all status: */
    &AudioManager::initDNCParam,       /*   Ready state.           */
    &AudioManager::initDNCParam,       /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::initDNCParam,       /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::initDNCParam,       /*   BasebandReady state.   */
    &AudioManager::illegal,            /*   BasebandActive state.  */
    &AudioManager::illegal,            /*   WaitCommandWord state. */
    &AudioManager::initDNCParam,       /*   PowerOff state.        */
    &AudioManager::illegal             /*   Through state.         */
  },

  /* InitClearStereo command. */

  {                                    /* AudioManager all status: */
    &AudioManager::setClearStereo,     /*   Ready state.           */
    &AudioManager::setClearStereo,     /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::setClearStereo,     /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::setClearStereo,     /*   BasebandReady state.   */
    &AudioManager::illegal,            /*   BasebandActive state.  */
    &AudioManager::illegal,            /*   WaitCommandWord state. */
    &AudioManager::initClearStereo,    /*   PowerOff state.        */
    &AudioManager::setClearStereo      /*   Through state.         */
  },

  /* SetVolume command. */

  {                                    /* AudioManager all status: */
    &AudioManager::setVolume,          /*   Ready state.           */
    &AudioManager::setVolume,          /*   PlayerReady state.     */
    &AudioManager::setVolume,          /*   PlayerActive state.    */
    &AudioManager::setVolume,          /*   PlayerPause state.     */
    &AudioManager::setVolume,          /*   RecorderReady state.   */
    &AudioManager::setVolume,          /*   RecorderActive state.  */
    &AudioManager::setVolume,          /*   BasebandReady state.   */
    &AudioManager::setVolume,          /*   BasebandActive state.  */
    &AudioManager::setVolume,          /*   WaitCommandWord state. */
    &AudioManager::illegal,            /*   PowerOff state.        */
    &AudioManager::setVolume           /*   Through state.         */
  },

  /* SetVolumeMute command. */

  {                                    /* AudioManager all status: */
    &AudioManager::setVolumeMute,      /*   Ready state.           */
    &AudioManager::setVolumeMute,      /*   PlayerReady state.     */
    &AudioManager::setVolumeMute,      /*   PlayerActive state.    */
    &AudioManager::setVolumeMute,      /*   PlayerPause state.     */
    &AudioManager::setVolumeMute,      /*   RecorderReady state.   */
    &AudioManager::setVolumeMute,      /*   RecorderActive state.  */
    &AudioManager::setVolumeMute,      /*   BasebandReady state.   */
    &AudioManager::setVolumeMute,      /*   BasebandActive state.  */
    &AudioManager::setVolumeMute,      /*   WaitCommandWord state. */
    &AudioManager::illegal,            /*   PowerOff state.        */
    &AudioManager::setVolumeMute       /*   Through state.         */
  },

  /* SetBeep command. */

  {                                    /* AudioManager all status: */
    &AudioManager::setBeep,            /*   Ready state.           */
    &AudioManager::setBeep,            /*   PlayerReady state.     */
    &AudioManager::setBeep,            /*   PlayerActive state.    */
    &AudioManager::setBeep,            /*   PlayerPause state.     */
    &AudioManager::setBeep,            /*   RecorderReady state.   */
    &AudioManager::setBeep,            /*   RecorderActive state.  */
    &AudioManager::setBeep,            /*   BasebandReady state.   */
    &AudioManager::setBeep,            /*   BasebandActive state.  */
    &AudioManager::setBeep,            /*   WaitCommandWord state. */
    &AudioManager::illegal,            /*   PowerOff state.        */
    &AudioManager::setBeep             /*   Through state.         */
  },

  /* PowerOn command. */

  {                                    /* AudioManager all status: */
    &AudioManager::illegal,            /*   Ready state.           */
    &AudioManager::illegal,            /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::illegal,            /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::illegal,            /*   BasebandReady state.   */
    &AudioManager::illegal,            /*   BasebandActive state.  */
    &AudioManager::illegal,            /*   WaitCommandWord state. */
    &AudioManager::powerOn,            /*   PowerOff state.        */
    &AudioManager::illegal             /*   Through state.         */
  },

  /* PowerOff command. */

  {                                    /* AudioManager all status: */
    &AudioManager::powerOff,           /*   Ready state.           */
    &AudioManager::illegal,            /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::illegal,            /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::illegal,            /*   BasebandReady state.   */
    &AudioManager::illegal,            /*   BasebandActive state.  */
    &AudioManager::illegal,            /*   WaitCommandWord state. */
    &AudioManager::illegal,            /*   PowerOff state.        */
    &AudioManager::illegal             /*   Through state.         */
  },

  /* SubPlayer command. */

  {                                    /* AudioManager all status: */
    &AudioManager::illegal,            /*   Ready state.           */
    &AudioManager::subPlayer,          /*   PlayerReady state.     */
    &AudioManager::subPlayer,          /*   PlayerActive state.    */
    &AudioManager::subPlayer,          /*   PlayerPause state.     */
    &AudioManager::illegal,            /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::illegal,            /*   BasebandReady state.   */
    &AudioManager::illegal,            /*   BasebandActive state.  */
    &AudioManager::illegal,            /*   WaitCommandWord state. */
    &AudioManager::illegal,            /*   PowerOff state.        */
    &AudioManager::illegal             /*   Through state.         */
  },

  /* SetThroughStatus command. */

  {                                    /* AudioManager all status: */
    &AudioManager::setThroughStatus,   /*   Ready state.           */
    &AudioManager::illegal,            /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::illegal,            /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::illegal,            /*   BasebandReady state.   */
    &AudioManager::illegal,            /*   BasebandActive state.  */
    &AudioManager::illegal,            /*   WaitCommandWord state. */
    &AudioManager::illegal,            /*   PowerOff state.        */
    &AudioManager::illegal             /*   Through state.         */
  },

  /* SetThroughPath command. */

  {                                    /* AudioManager all status: */
    &AudioManager::illegal,            /*   Ready state.           */
    &AudioManager::illegal,            /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::illegal,            /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::illegal,            /*   BasebandReady state.   */
    &AudioManager::illegal,            /*   BasebandActive state.  */
    &AudioManager::illegal,            /*   WaitCommandWord state. */
    &AudioManager::illegal,            /*   PowerOff state.        */
    &AudioManager::setThroughPath      /*   Through state.         */
  },

  /* Invalid command. */

  {                                    /* AudioManager all status: */
    &AudioManager::illegal,            /*   Ready state.           */
    &AudioManager::illegal,            /*   PlayerReady state.     */
    &AudioManager::illegal,            /*   PlayerActive state.    */
    &AudioManager::illegal,            /*   PlayerPause state.     */
    &AudioManager::illegal,            /*   RecorderReady state.   */
    &AudioManager::illegal,            /*   RecorderActive state.  */
    &AudioManager::illegal,            /*   BasebandReady state.   */
    &AudioManager::illegal,            /*   BasebandActive state.  */
    &AudioManager::illegal,            /*   WaitCommandWord state. */
    &AudioManager::illegal,            /*   PowerOff state.        */
    &AudioManager::illegal             /*   Through state.         */
  }
};

AudioManager::RstProc AudioManager::RstProcTbl[1][AS_MNG_STATUS_NUM] =
{
  /* Result command. */

  {                                  /* AudioManager status: */
    &AudioManager::cmpltOnReady,     /*   Ready state.       */
    &AudioManager::cmpltOnSoundFx,   /*   Baseband state.    */
    &AudioManager::cmpltOnPlayer,    /*   Player state.      */
    &AudioManager::cmpltOnRecorder,  /*   Recorder state.    */
    &AudioManager::cmpltOnPowerOff   /*   PowerOff state.    */
  },
};

/*--------------------------------------------------------------------------*/
void AudioManager::parse(FAR MsgPacket *msg)
{
  if (MSG_IS_REQUEST(msg->getType()))
    {
      /* External event. */

      uint event = MSG_GET_SUBTYPE(msg->getType());
      F_ASSERT((event < AUD_MGR_MSG_NUM));
      AudioCommand cmd = msg->moveParam<AudioCommand>();
      int allstate = getAllState();
      (this->*MsgProcTbl[event][allstate])(cmd);
    }
  else
    {
      /* Internal event. */

      if (msg->getType() == MSG_AUD_MGR_RST)
        {
          uint event = MSG_GET_SUBTYPE(msg->getType());
          const AudioMngCmdCmpltResult& rst =
            msg->peekParam<AudioMngCmdCmpltResult>();
          (this->*RstProcTbl[event][m_State])(rst);
          msg->popParam<AudioMngCmdCmpltResult>();
        }
      else if (msg->getType() == MSG_AUD_MGR_CALL_ATTENTION)
        {
          const AttentionInfo& info = msg->peekParam<AttentionInfo>();
          execAttentions(info);
          msg->popParam<AttentionInfo>();
#ifdef AS_FEATURE_RECOGNIZER_ENABLE
        }
      else if (msg->getType() == MSG_AUD_MGR_FIND_COMMAND)
        {
          const AudioFindCommandInfo& info =
            msg->peekParam<AudioFindCommandInfo>();
          execFindCommandCallback(info.keyword, info.status);
          msg->popParam<AudioFindCommandInfo>();
#endif  /* AS_FEATURE_RECOGNIZER_ENABLE */
        }
    }
}

/*--------------------------------------------------------------------------*/
void AudioManager::illegal(AudioCommand &cmd)
{
  sendErrRespResult(cmd.header.sub_code,
                    AS_MODULE_ID_AUDIO_MANAGER,
                    AS_RESPONSE_CODE_STATE_VIOLATION);
}

/*--------------------------------------------------------------------------*/
void AudioManager::ignore(AudioCommand &cmd)
{
  F_ASSERT(0);
}

/*--------------------------------------------------------------------------*/
void AudioManager::getstatus(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_GETSTATUS, AUDCMD_GETSTATUS, cmd);
  if (!check)
    {
      return;
    }

  AudioResult packet;
  packet.header.packet_length = LENGTH_AUDRLT;
  packet.header.result_code   = AUDRLT_NOTIFYSTATUS;
  packet.header.sub_code      = 0;

  packet.notify_status.status_info     = m_State;
  packet.notify_status.sub_status_info = m_SubState;
  packet.notify_status.vad_status      =
    ((m_SubState == AS_MNG_SUB_STATUS_WAITCMDWORD) ? m_VadState : 0);

  err_t er = MsgLib::send<AudioResult>(s_appMid,
                                       MsgPriNormal,
                                       MSG_AUD_MGR_RST,
                                       m_selfDtq,
                                       packet);
  if (er)
    {
      F_ASSERT(0);
    }
}

/*--------------------------------------------------------------------------*/
void AudioManager::initAttentions(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_INITATTENTIONS, AUDCMD_INITATTENTIONS, cmd);
  if (!check)
    {
      return;
    }

  if (cmd.init_attentions_param.attention_callback_function != NULL)
    {
      m_attentionCBFunc =
        cmd.init_attentions_param.attention_callback_function;
      sendResult(AUDRLT_INITATTENTIONSCMPLT);
    }
  else
    {
      sendErrRespResult(cmd.header.sub_code,
                        AS_MODULE_ID_AUDIO_MANAGER,
                        AS_RESPONSE_CODE_COMMAND_PARAM_CALLBACK);
    }
}

/*--------------------------------------------------------------------------*/
void AudioManager::execAttentions(const AttentionInfo& info)
{
  if (m_attentionCBFunc != NULL)
    {
#ifndef ATTENTION_USE_FILENAME_LINE
      (m_attentionCBFunc)(info.module_id,
                          info.attention_id,
                          info.sub_code);
#else
      (m_attentionCBFunc)(info.module_id,
                          info.attention_id,
                          info.sub_code,
                          info.file_name,
                          info.line);
#endif  /* ATTENTION_USE_FILENAME_LINE */
    }
}

/*--------------------------------------------------------------------------*/
void AudioManager::powerOn(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_POWERON, AUDCMD_POWERON, cmd);
  if (!check)
    {
      return;
    }

  m_command_code = 0;
      m_State    = AS_MNG_STATUS_READY;
      m_SubState = AS_MNG_SUB_STATUS_NONE;
      sendResult(AUDRLT_STATUSCHANGED);
    }

/*--------------------------------------------------------------------------*/
void AudioManager::powerOff(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_SET_POWEROFF_STATUS, AUDCMD_SETPOWEROFFSTATUS, cmd);
  if (!check)
    {
      return;
    }

      uint8_t rst = AS_RESPONSE_CODE_OK;
      rst = bbConfig.deactivate(BB_POWER_BOTH);
      if (rst != AS_RESPONSE_CODE_OK)
        {
          sendErrRespResult(cmd.header.sub_code,
                            AS_MODULE_ID_AUDIO_DRIVER,
                            rst);
          return;
        }

      m_State    = AS_MNG_STATUS_POWEROFF;
      m_SubState = AS_MNG_SUB_STATUS_NONE;
      sendResult(AUDRLT_STATUSCHANGED);
    }

/*--------------------------------------------------------------------------*/
void AudioManager::soundFx(AudioCommand &cmd)
{
#ifdef AS_FEATURE_EFFECTOR_ENABLE
  MSG_TYPE msg_type;
  bool check = false;

  switch (cmd.header.command_code)
    {
      case AUDCMD_STARTBB:
        check = packetCheck(LENGTH_STARTBB, AUDCMD_STARTBB, cmd);
        if (!check)
          {
            return;
          }
        msg_type = MSG_AUD_SEF_CMD_START;
        break;

      case AUDCMD_STOPBB:
        check = packetCheck(LENGTH_STOPBB, AUDCMD_STOPBB, cmd);
        if (!check)
          {
            return;
          }
        msg_type = MSG_AUD_SEF_CMD_STOP;
        break;

      default:
        sendErrRespResult(cmd.header.sub_code,
                          AS_MODULE_ID_AUDIO_MANAGER,
                          AS_RESPONSE_CODE_COMMAND_CODE_ERROR);
        return;
  }

  err_t er = MsgLib::send<AudioCommand>(s_effectMid,
                                        MsgPriNormal,
                                        msg_type,
                                        m_selfDtq,
                                        cmd);
  if (er)
    {
      F_ASSERT(0);
    }
#else
  sendErrRespResult(cmd.header.sub_code,
                    AS_MODULE_ID_AUDIO_MANAGER,
                    AS_RESPONSE_CODE_COMMAND_NOT_SUPPOT);
#endif /* AS_FEATURE_EFFECTOR_ENABLE */
}

/*--------------------------------------------------------------------------*/
void AudioManager::mfe(AudioCommand &cmd)
{
#ifdef AS_FEATURE_EFFECTOR_ENABLE
  MSG_TYPE msg_type;
  bool check = false;

  switch (cmd.header.command_code)
    {
      case AUDCMD_INITMFE:
        check = packetCheck(LENGTH_INITMFE, AUDCMD_INITMFE, cmd);
        if (!check)
          {
            return;
          }
        if (cmd.init_mfe_param.config_table == 0)
          {
            sendErrRespResult(cmd.header.sub_code,
                              AS_MODULE_ID_AUDIO_MANAGER,
                              AS_RESPONSE_CODE_COMMAND_PARAM_CONFIG_TABLE);
            return;
          }
        msg_type = MSG_AUD_SEF_CMD_INIT;
        break;

      case AUDCMD_DEBUGMFEPARAM:
        check = packetCheck(LENGTH_DEBUGMFEPARAM, AUDCMD_DEBUGMFEPARAM, cmd);
        if (!check)
          {
            return;
          }
        if (cmd.debug_mfe_param.mfe_config_table == 0)
          {
            sendErrRespResult(cmd.header.sub_code,
                              AS_MODULE_ID_AUDIO_MANAGER,
                              AS_RESPONSE_CODE_COMMAND_PARAM_CONFIG_TABLE);
            return;
          }
        msg_type = MSG_AUD_SEF_CMD_DEBUG;
        break;

      default:
        sendErrRespResult(cmd.header.sub_code,
                          AS_MODULE_ID_AUDIO_MANAGER,
                          AS_RESPONSE_CODE_COMMAND_CODE_ERROR);
        return;
    }

  err_t er = MsgLib::send<AudioCommand>(s_effectMid,
                                        MsgPriNormal,
                                        msg_type,
                                        m_selfDtq,
                                        cmd);
  if (er)
    {
      F_ASSERT(0);
    }
#else
  sendErrRespResult(cmd.header.sub_code,
                    AS_MODULE_ID_AUDIO_MANAGER,
                    AS_RESPONSE_CODE_COMMAND_NOT_SUPPOT);
#endif /* AS_FEATURE_EFFECTOR_ENABLE */
}

/*--------------------------------------------------------------------------*/
void AudioManager::mpp(AudioCommand &cmd)
{
#ifdef AS_FEATURE_EFFECTOR_ENABLE
  MSG_TYPE msg_type;
  bool check = false;

  switch (cmd.header.command_code)
    {
      case AUDCMD_INITMPP:
        check = packetCheck(LENGTH_INITMPP, AUDCMD_INITMPP, cmd);
        if (!check)
          {
            return;
          }
        if (AS_MPP_XLOUD_MODE_DISABLE != cmd.init_mpp_param.xloud_mode &&
             cmd.init_mpp_param.xloud_coef_table == 0)
          {
            sendErrRespResult(cmd.header.sub_code,
                              AS_MODULE_ID_AUDIO_MANAGER,
                              AS_RESPONSE_CODE_COMMAND_PARAM_CONFIG_TABLE);
            return;
          }
        if (AS_MPP_EAX_DISABLE != cmd.init_mpp_param.eax_mode &&
             cmd.init_mpp_param.eax_coef_table == 0)
          {
            sendErrRespResult(cmd.header.sub_code,
                              AS_MODULE_ID_AUDIO_MANAGER,
                              AS_RESPONSE_CODE_COMMAND_PARAM_CONFIG_TABLE);
            return;
          }
        msg_type = MSG_AUD_SEF_CMD_INIT;
        break;

      case AUDCMD_SETMPPPARAM:
        check = packetCheck(LENGTH_SUB_SETMPP_COMMON,
                            AUDCMD_SETMPPPARAM,
                            cmd);
        if (!check)
          {
            return;
          }
        msg_type = MSG_AUD_SEF_CMD_SETPARAM;
        break;

      case SUB_SETMPP_XLOUD:
        check = packetCheck(LENGTH_SUB_SETMPP_XLOUD, SUB_SETMPP_XLOUD, cmd);
        if (!check)
          {
            return;
          }
        msg_type = MSG_AUD_SEF_CMD_SETPARAM;
        break;

      case AUDCMD_DEBUGMPPPARAM:
        check = packetCheck(LENGTH_DEBUGMPPPARAM, AUDCMD_DEBUGMPPPARAM, cmd);
        if (!check)
          {
            return;
          }
        msg_type = MSG_AUD_SEF_CMD_DEBUG;
        break;

      default:
        sendErrRespResult(cmd.header.sub_code,
                          AS_MODULE_ID_AUDIO_MANAGER,
                          AS_RESPONSE_CODE_COMMAND_CODE_ERROR);
        return;
    }

  err_t er = MsgLib::send<AudioCommand>(s_effectMid,
                                        MsgPriNormal,
                                        msg_type,
                                        m_selfDtq, cmd);
  if (er)
    {
      F_ASSERT(0);
    }
#else
  sendErrRespResult(cmd.header.sub_code,
                    AS_MODULE_ID_AUDIO_MANAGER,
                    AS_RESPONSE_CODE_COMMAND_NOT_SUPPOT);
#endif /* AS_FEATURE_EFFECTOR_ENABLE */
}

/*--------------------------------------------------------------------------*/
void AudioManager::player(AudioCommand &cmd)
{
#ifdef AS_FEATURE_PLAYER_ENABLE
  MSG_TYPE msg_type;
  bool check = false;

  switch (cmd.header.command_code)
    {
      case AUDCMD_INITPLAYER:
        check = packetCheck(LENGTH_INIT_PLAYER, AUDCMD_INITPLAYER, cmd);
        if (!check)
          {
            return;
          }
        msg_type = MSG_AUD_PLY_CMD_INIT;
        break;

      case AUDCMD_PLAYPLAYER:
        check = packetCheck(LENGTH_PLAY_PLAYER, AUDCMD_PLAYPLAYER, cmd);
        if (!check)
          {
            return;
          }
        msg_type = MSG_AUD_PLY_CMD_PLAY;
        break;

      case AUDCMD_STOPPLAYER:
        check = packetCheck(LENGTH_STOP_PLAYER, AUDCMD_STOPPLAYER, cmd);
        if (!check)
          {
            return;
          }
        msg_type = MSG_AUD_PLY_CMD_STOP;
        break;

      case AUDCMD_CLKRECOVERY:
        check = packetCheck(LENGTH_CLK_RECOVERY, AUDCMD_CLKRECOVERY, cmd);
        if (!check)
          {
            return;
          }
        msg_type = MSG_AUD_PLY_CMD_CLKRECOVERY;
        break;

      default:
        sendErrRespResult(cmd.header.sub_code,
                          AS_MODULE_ID_AUDIO_MANAGER,
                          AS_RESPONSE_CODE_COMMAND_CODE_ERROR);
        return;
    }

  err_t er = MsgLib::send<AudioCommand>(m_playerDtq,
                                        MsgPriNormal,
                                        msg_type,
                                        m_selfDtq,
                                        cmd);
  if (er)
    {
      F_ASSERT(0);
    }
#else
  sendErrRespResult(cmd.header.sub_code,
                    AS_MODULE_ID_AUDIO_MANAGER,
                    AS_RESPONSE_CODE_COMMAND_NOT_SUPPOT);
#endif /* AS_FEATURE_PLAYER_ENABLE */
}

/*--------------------------------------------------------------------------*/
void AudioManager::subPlayer(AudioCommand &cmd)
{
#ifdef AS_FEATURE_PLAYER_ENABLE
  MSG_TYPE msg_type;
  bool check = false;

  switch (cmd.header.command_code)
    {
      case AUDCMD_INITSUBPLAYER:
        check = packetCheck(LENGTH_INIT_PLAYER, AUDCMD_INITSUBPLAYER, cmd);
        if (!check)
          {
            return;
          }
        msg_type = MSG_AUD_PLY_CMD_INIT;
        break;

      case AUDCMD_PLAYSUBPLAYER:
        check = packetCheck(LENGTH_PLAY_PLAYER, AUDCMD_PLAYSUBPLAYER, cmd);
        if (!check)
          {
            return;
          }
        msg_type = MSG_AUD_PLY_CMD_PLAY;
        break;

      case AUDCMD_STOPSUBPLAYER:
        check = packetCheck(LENGTH_STOP_PLAYER, AUDCMD_STOPSUBPLAYER, cmd);
        if (!check)
          {
            return;
          }
        msg_type = MSG_AUD_PLY_CMD_STOP;
        break;

      case AUDCMD_CLKRECOVERYSUB:
        check = packetCheck(LENGTH_CLK_RECOVERY, AUDCMD_CLKRECOVERYSUB, cmd);
        if (!check)
          {
            return;
          }
        msg_type = MSG_AUD_PLY_CMD_CLKRECOVERY;
        break;

      default:
        sendErrRespResult(cmd.header.sub_code,
                          AS_MODULE_ID_AUDIO_MANAGER,
                          AS_RESPONSE_CODE_COMMAND_CODE_ERROR);
        return;
    }

  err_t er = MsgLib::send<AudioCommand>(m_subplayerDtq,
                                        MsgPriNormal,
                                        msg_type,
                                        m_selfDtq,
                                        cmd);
  if (er)
    {
      F_ASSERT(0);
    }
#else
  sendErrRespResult(cmd.header.sub_code,
                    AS_MODULE_ID_AUDIO_MANAGER,
                    AS_RESPONSE_CODE_COMMAND_NOT_SUPPOT);
#endif /* AS_FEATURE_PLAYER_ENABLE */
}

/*--------------------------------------------------------------------------*/
void AudioManager::recorder(AudioCommand &cmd)
{
#ifdef AS_FEATURE_RECORDER_ENABLE
  MSG_TYPE msg_type;
  bool check = false;

  switch (cmd.header.command_code)
    {
      case AUDCMD_INITREC:
        check = packetCheck(LENGTH_INIT_RECORDER, AUDCMD_INITREC, cmd);
        if (!check)
          {
            return;
          }
        msg_type = MSG_AUD_VRC_CMD_INIT;
        break;

      case AUDCMD_STARTREC:
        check = packetCheck(LENGTH_START_RECORDER, AUDCMD_STARTREC, cmd);
        if (!check)
          {
            return;
          }
        msg_type = MSG_AUD_VRC_CMD_PLAY;
        break;

      case AUDCMD_STOPREC:
        check = packetCheck(LENGTH_STOP_RECORDER, AUDCMD_STOPREC, cmd);
        if (!check)
          {
            return;
          }
        msg_type = MSG_AUD_VRC_CMD_STOP;
        break;

      default:
        sendErrRespResult(cmd.header.sub_code,
                          AS_MODULE_ID_AUDIO_MANAGER,
                          AS_RESPONSE_CODE_COMMAND_CODE_ERROR);
        return;
    }

  err_t er = MsgLib::send<AudioCommand>(s_rcdSubMid,
                                        MsgPriNormal,
                                        msg_type,
                                        m_selfDtq,
                                        cmd);
  if (er)
    {
      F_ASSERT(0);
    }
#else
  sendErrRespResult(cmd.header.sub_code,
                    AS_MODULE_ID_AUDIO_MANAGER,
                    AS_RESPONSE_CODE_COMMAND_NOT_SUPPOT);
#endif /* AS_FEATURE_RECORDER_ENABLE */
}

/*--------------------------------------------------------------------------*/
void AudioManager::setThroughStatus(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_SET_THROUGH_STATUS, AUDCMD_SETTHROUGHSTATUS, cmd);
  if (!check)
    {
      return;
    }

  uint32_t rst = bbConfig.setActiveBaseband(BB_POWER_BOTH);
  if (rst != AS_RESPONSE_CODE_OK)
    {
      sendErrRespResult(cmd.header.sub_code,
                        AS_MODULE_ID_AUDIO_DRIVER,
                        rst);
      return;
    }

  m_State    = AS_MNG_STATUS_THROUGH;
  m_SubState = AS_MNG_SUB_STATUS_NONE;

  sendResult(AUDRLT_STATUSCHANGED);
}

/*--------------------------------------------------------------------------*/
void AudioManager::setRdyOnAct(AudioCommand &cmd)
{
#ifdef AS_FEATURE_EFFECTOR_ENABLE
  /* Before transfer to ready status, check parameters first. */

  bool check =
    packetCheck(LENGTH_SET_READY_STATUS, AUDCMD_SETREADYSTATUS, cmd);
  if (!check)
    {
      return;
    }

  err_t er = MsgLib::send(s_rcgMid,
                          MsgPriNormal,
                          MSG_AUD_RCG_DEACT,
                          m_selfDtq);

  /* No error check because there are cases where voice_command is not used.
   * error checking is done in RecognitonObject.
   */

  er = MsgLib::send<AudioCommand>(s_effectMid,
                                  MsgPriNormal,
                                  MSG_AUD_SEF_CMD_DEACT,
                                  m_selfDtq,
                                  cmd);
  if (er)
    {
      F_ASSERT(0);
    }
#else
  sendErrRespResult(cmd.header.sub_code,
                    AS_MODULE_ID_AUDIO_MANAGER,
                    AS_RESPONSE_CODE_COMMAND_NOT_SUPPOT);
#endif /* AS_FEATURE_EFFECTOR_ENABLE */
}

/*--------------------------------------------------------------------------*/
void AudioManager::setRdyOnPlay(AudioCommand &cmd)
{
#ifdef AS_FEATURE_PLAYER_ENABLE
  bool check =
    packetCheck(LENGTH_SET_READY_STATUS, AUDCMD_SETREADYSTATUS, cmd);
  if (!check)
    {
      return;
    }

  err_t er = ERR_OK;

  if (m_active_player & AS_ACTPLAYER_MAIN)
    {
      er = MsgLib::send<AudioCommand>(m_playerDtq,
                                      MsgPriNormal,
                                      MSG_AUD_PLY_CMD_DEACT,
                                      m_selfDtq,
                                      cmd);
      if (er)
        {
          F_ASSERT(0);
        }
      m_player_transition_que.push(er);
      m_active_player &= ~AS_ACTPLAYER_MAIN;
    }

  if (m_active_player & AS_ACTPLAYER_SUB)
    {
      er = MsgLib::send<AudioCommand>(m_subplayerDtq,
                                      MsgPriNormal,
                                      MSG_AUD_PLY_CMD_DEACT,
                                      m_selfDtq,
                                      cmd);
      if (er)
        {
          F_ASSERT(0);
        }
      m_player_transition_que.push(er);
      m_active_player &= ~AS_ACTPLAYER_SUB;
    }
#else
  sendErrRespResult(cmd.header.sub_code,
                    AS_MODULE_ID_AUDIO_MANAGER,
                    AS_RESPONSE_CODE_COMMAND_NOT_SUPPOT);
#endif /* AS_FEATURE_PLAYER_ENABLE */
}

/*--------------------------------------------------------------------------*/
void AudioManager::setRdyOnRecorder(AudioCommand &cmd)
{
#ifdef AS_FEATURE_RECORDER_ENABLE
  bool check =
    packetCheck(LENGTH_SET_READY_STATUS, AUDCMD_SETREADYSTATUS, cmd);
  if (!check)
    {
      return;
    }

  err_t er = ERR_OK;

  er = MsgLib::send<AudioCommand>(s_rcdSubMid,
                                  MsgPriNormal,
                                  MSG_AUD_VRC_CMD_DEACTIVATE,
                                  m_selfDtq,
                                  cmd);
  if (er)
    {
      F_ASSERT(0);
    }
#else
  sendErrRespResult(cmd.header.sub_code,
                    AS_MODULE_ID_AUDIO_MANAGER,
                    AS_RESPONSE_CODE_COMMAND_NOT_SUPPOT);
#endif /* AS_FEATURE_RECORDER_ENABLE */
}

/*--------------------------------------------------------------------------*/
void AudioManager::setRdyOnThrough(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_SET_READY_STATUS, AUDCMD_SETREADYSTATUS, cmd);
  if (!check)
    {
      return;
    }

  uint32_t rst = bbConfig.deactivate(BB_POWER_BOTH);
  if (rst != AS_RESPONSE_CODE_OK)
    {
      sendErrRespResult(cmd.header.sub_code,
                        AS_MODULE_ID_AUDIO_DRIVER,
                        rst);
      return;
    }

  m_State    = AS_MNG_STATUS_READY;
  m_SubState = AS_MNG_SUB_STATUS_NONE;

  sendResult(AUDRLT_STATUSCHANGED);
}

/*--------------------------------------------------------------------------*/
void AudioManager::setActive(AudioCommand &cmd)
{
#ifdef AS_FEATURE_EFFECTOR_ENABLE
  /* Before state transition, check parameters first. */

  bool check =
    packetCheck(LENGTH_SET_BBACTIVE_STATUS, AUDCMD_SETBBACTIVESTATUS, cmd);
  if (!check)
    {
      return;
    }

  uint32_t rst = AS_RESPONSE_CODE_OK;

  rst = bbConfig.setActiveBaseband(BB_POWER_BOTH);
  if (rst != AS_RESPONSE_CODE_OK)
    {
      sendErrRespResult(cmd.header.sub_code,
                        AS_MODULE_ID_AUDIO_DRIVER,
                        rst);
      return;
    }

  if (AS_SET_BBSTS_WITH_VCMD_ACTIVE ==
       cmd.set_baseband_status_param.with_Voice_Command)
    {
      if (AS_SET_BBSTS_WITH_MFE_NONE ==
           cmd.set_baseband_status_param.with_MFE)
        {
          MemMgrLite::Manager::destroyStaticPools();
          rst = bbConfig.deactivate(BB_POWER_BOTH);
          if (rst != AS_RESPONSE_CODE_OK)
            {
              sendErrRespResult(cmd.header.sub_code,
                                AS_MODULE_ID_AUDIO_DRIVER,
                                rst);
              return;
            }
          sendErrRespResult(cmd.header.sub_code,
                            AS_MODULE_ID_AUDIO_MANAGER,
                            AS_RESPONSE_CODE_COMMAND_PARAM_WITH_MFE);
          return;
        }
      else
        {
          err_t er = MsgLib::send(s_rcgMid,
                                  MsgPriNormal,
                                  MSG_AUD_RCG_ACT,
                                  m_selfDtq);
          if (er)
            {
              F_ASSERT(0);
            }
        }
    }

  err_t er = MsgLib::send<AudioCommand>(s_effectMid,
                                        MsgPriNormal,
                                        MSG_AUD_SEF_CMD_ACT,
                                        m_selfDtq,
                                        cmd);
  if (er)
    {
      F_ASSERT(0);
    }
#else
  sendErrRespResult(cmd.header.sub_code,
                    AS_MODULE_ID_AUDIO_MANAGER,
                    AS_RESPONSE_CODE_COMMAND_NOT_SUPPOT);
#endif /* AS_FEATURE_EFFECTOR_ENABLE */
}

/*--------------------------------------------------------------------------*/
#ifdef AS_FEATURE_PLAYER_ENABLE
uint32_t AudioManager::setPlayerStatusParamCheck(uint8_t  input_dev,
                                                 uint8_t  output_dev,
                                                 FAR void *input_handler)
{
  switch (input_dev)
    {
      case AS_SETPLAYER_INPUTDEVICE_RAM:
        break;

      default:
        MANAGER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_RESPONSE_CODE_COMMAND_PARAM_INPUT_DEVICE;
    }

  switch (output_dev)
    {
      case AS_SETPLAYER_OUTPUTDEVICE_SPHP:
      case AS_SETPLAYER_OUTPUTDEVICE_I2SOUTPUT:
        break;

      default:
        MANAGER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        return AS_RESPONSE_CODE_COMMAND_PARAM_OUTPUT_DEVICE;
    }

  if (input_handler == NULL)
    {
      return AS_RESPONSE_CODE_COMMAND_PARAM_INPUT_HANDLER;
    }
  return AS_RESPONSE_CODE_OK;
}
#endif /* AS_FEATURE_PLAYER_ENABLE */

/*--------------------------------------------------------------------------*/
void AudioManager::setPlayerStatus(AudioCommand &cmd)
{
#ifdef AS_FEATURE_PLAYER_ENABLE
  /* Before state transition, check parameters first. */

  bool check =
    packetCheck(LENGTH_SET_PLAYER_STATUS, AUDCMD_SETPLAYERSTATUS, cmd);
  if (!check)
    {
      return;
    }
  if ((AS_ACTPLAYER_MAIN > cmd.set_player_sts_param.active_player) ||
       (AS_ACTPLAYER_BOTH < cmd.set_player_sts_param.active_player))
    {
      sendErrRespResult(cmd.header.sub_code,
                        AS_MODULE_ID_AUDIO_MANAGER,
                        AS_RESPONSE_CODE_COMMAND_PARAM_ACTIVE_PLAYER);
      return;
    }

  m_active_player = cmd.set_player_sts_param.active_player;
  uint32_t check_param;
  if (m_active_player & AS_ACTPLAYER_MAIN)
    {
      check_param =
        setPlayerStatusParamCheck(cmd.set_player_sts_param.input_device,
                                  cmd.set_player_sts_param.output_device,
                                  cmd.set_player_sts_param.ram_handler);
      if (check_param != AS_RESPONSE_CODE_OK)
        {
          sendErrRespResult(cmd.header.sub_code,
                            AS_MODULE_ID_AUDIO_MANAGER,
                            check_param);
          return;
        }
    }

  if (m_active_player & AS_ACTPLAYER_SUB)
    {
      check_param =
        setPlayerStatusParamCheck(cmd.set_player_sts_param.input_device_sub,
                                  cmd.set_player_sts_param.output_device_sub,
                                  cmd.set_player_sts_param.ram_handler_sub);
      if (check_param != AS_RESPONSE_CODE_OK)
        {
          sendErrRespResult(cmd.header.sub_code,
                            AS_MODULE_ID_AUDIO_MANAGER,
                            check_param);
          return;
        }
    }

  uint32_t rst = AS_RESPONSE_CODE_OK;

  rst = bbConfig.setActiveBaseband(BB_POWER_OUTPUT);
  if (rst != AS_RESPONSE_CODE_OK)
    {
      sendErrRespResult(cmd.header.sub_code,
                        AS_MODULE_ID_AUDIO_DRIVER,
                        rst);
      return;
    }

  if (m_active_player & AS_ACTPLAYER_MAIN)
    {
      err_t er = MsgLib::send<AudioCommand>(m_playerDtq,
                                            MsgPriNormal,
                                            MSG_AUD_PLY_CMD_ACT,
                                            m_selfDtq,
                                            cmd);
      if (er)
        {
          F_ASSERT(0);
        }
      m_player_transition_que.push(er);
    }

  if (m_active_player & AS_ACTPLAYER_SUB)
    {
      /* Reload parameters for PlayerObjct(sub). */

      cmd.set_player_sts_param.input_device =
        cmd.set_player_sts_param.input_device_sub;
      cmd.set_player_sts_param.ram_handler =
        cmd.set_player_sts_param.ram_handler_sub;
      cmd.set_player_sts_param.output_device =
        cmd.set_player_sts_param.output_device_sub;
      cmd.set_player_sts_param.output_device_handler =
        cmd.set_player_sts_param.output_device_handler_sub;

      err_t er = MsgLib::send<AudioCommand>(m_subplayerDtq,
                                            MsgPriNormal,
                                            MSG_AUD_PLY_CMD_ACT,
                                            m_selfDtq,
                                            cmd);
      if (er)
        {
          F_ASSERT(0);
        }
      m_player_transition_que.push(er);
    }
#else
  sendErrRespResult(cmd.header.sub_code,
                    AS_MODULE_ID_AUDIO_MANAGER,
                    AS_RESPONSE_CODE_COMMAND_NOT_SUPPOT);
#endif /* AS_FEATURE_PLAYER_ENABLE */
}

/*--------------------------------------------------------------------------*/
void AudioManager::setRecorder(AudioCommand &cmd)
{
#ifdef AS_FEATURE_RECORDER_ENABLE
  /* Before transfer to recorder status, check parameters first. */

  bool check =
    packetCheck(LENGTH_SET_RECORDER_STATUS, AUDCMD_SETRECORDERSTATUS, cmd);
  if (!check)
    {
      return;
    }

  switch (cmd.set_recorder_status_param.input_device)
    {
      case AS_SETRECDR_STS_INPUTDEVICE_MIC_A:
      case AS_SETRECDR_STS_INPUTDEVICE_MIC_D:
      case AS_SETRECDR_STS_INPUTDEVICE_I2S_IN:
        break;

      default:
        MANAGER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        sendErrRespResult(cmd.header.sub_code,
                          AS_MODULE_ID_AUDIO_MANAGER,
                          AS_RESPONSE_CODE_COMMAND_PARAM_INPUT_DEVICE);
        return;
    }

  switch (cmd.set_recorder_status_param.output_device)
    {
      case AS_SETRECDR_STS_OUTPUTDEVICE_EMMC:
      case AS_SETRECDR_STS_OUTPUTDEVICE_RAM:
        break;

      default:
        MANAGER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        sendErrRespResult(cmd.header.sub_code,
                          AS_MODULE_ID_AUDIO_MANAGER,
                          AS_RESPONSE_CODE_COMMAND_PARAM_OUTPUT_DEVICE);
        return;
    }

  uint32_t rst = AS_RESPONSE_CODE_OK;

  rst = bbConfig.setActiveBaseband(BB_POWER_INPUT);
  if (rst != AS_RESPONSE_CODE_OK)
    {
      sendErrRespResult(cmd.header.sub_code,
                        AS_MODULE_ID_AUDIO_DRIVER,
                        rst);
      return;
    }

  err_t er = MsgLib::send<AudioCommand>(s_rcdSubMid,
                                        MsgPriNormal,
                                        MSG_AUD_VRC_CMD_ACTIVATE,
                                        m_selfDtq,
                                        cmd);
  if (er)
    {
      F_ASSERT(0);
    }
#else
  sendErrRespResult(cmd.header.sub_code,
                    AS_MODULE_ID_AUDIO_MANAGER,
                    AS_RESPONSE_CODE_COMMAND_NOT_SUPPOT);
#endif /* AS_FEATURE_RECORDER_ENABLE */
}

/*--------------------------------------------------------------------------*/
void AudioManager::voiceCommand(AudioCommand &cmd)
{
#ifdef AS_FEATURE_RECOGNIZER_ENABLE
  bool check = false;

  switch (cmd.header.command_code)
    {
      case AUDCMD_STARTVOICECOMMAND:
        check = packetCheck(LENGTH_START_VOICE_COMMAND,
                            AUDCMD_STARTVOICECOMMAND,
                            cmd);
        if (!check)
          {
            return;
          }

        if (m_SubState != AS_MNG_SUB_STATUS_BASEBANDACTIVE)
          {
            sendErrRespResult(cmd.header.sub_code,
                              AS_MODULE_ID_AUDIO_MANAGER,
                              AS_RESPONSE_CODE_STATE_VIOLATION);
            return;
          }
        else
          {
            if (cmd.start_voice_command_param.callback_function == NULL)
              {
                sendErrRespResult(cmd.header.sub_code,
                                  AS_MODULE_ID_AUDIO_MANAGER,
                                  AS_RESPONSE_CODE_COMMAND_PARAM_CALLBACK);
                return;
              }
            m_findCommandCBFunc =
              cmd.start_voice_command_param.callback_function;

            VoiceRecognitionCommandObject::CommandSetParam_t param;
            param.key_word = cmd.start_voice_command_param.keyword;
            param.vad_only = cmd.start_voice_command_param.vad_only;
            param.p_vad_param = (uint8_t *)VADCoef_table;
            err_t err =
              MsgLib::send<VoiceRecognitionCommandObject::CommandSetParam_t>(
                s_rcgMid,
                MsgPriNormal,
                MSG_AUD_RCG_START,
                m_selfDtq,
                param);
            if (err)
              {
                F_ASSERT(0);
              }
          }
        break;

      case AUDCMD_STOPVOICECOMMAND:
        {
          check = packetCheck(LENGTH_STOP_VOICE_COMMAND,
                              AUDCMD_STOPVOICECOMMAND,
                              cmd);
          if (!check)
            {
              return;
            }

          if (m_SubState != AS_MNG_SUB_STATUS_WAITCMDWORD)
            {
              sendErrRespResult(cmd.header.sub_code,
                                AS_MODULE_ID_AUDIO_MANAGER,
                                AS_RESPONSE_CODE_STATE_VIOLATION);
              return;
            }

          err_t err = MsgLib::send(s_rcgMid,
                                   MsgPriNormal,
                                   MSG_AUD_RCG_STOP,
                                   m_selfDtq);
          if (err)
            {
              F_ASSERT(0);
            }
        }
        break;

      default:
        {
          sendErrRespResult(cmd.header.sub_code,
                            AS_MODULE_ID_AUDIO_MANAGER,
                            AS_RESPONSE_CODE_COMMAND_CODE_ERROR);
        }
        break;
    }
#else
  sendErrRespResult(cmd.header.sub_code,
                    AS_MODULE_ID_AUDIO_MANAGER,
                    AS_RESPONSE_CODE_COMMAND_NOT_SUPPOT);
#endif /* AS_FEATURE_RECOGNIZER_ENABLE */
}


/*--------------------------------------------------------------------------*/
void AudioManager::illegalCmplt(const AudioMngCmdCmpltResult &cmd)
{
  sendErrRespResult(cmd.sub_code,
                    AS_MODULE_ID_AUDIO_MANAGER,
                    AS_RESPONSE_CODE_STATE_VIOLATION);
}

/*--------------------------------------------------------------------------*/
void AudioManager::cmpltOnReady(const AudioMngCmdCmpltResult &cmd)
{
  uint8_t result_code = AUDRLT_ERRORRESPONSE;

  if (cmd.result != AS_RESPONSE_CODE_OK)
    {
      switch (cmd.command_code)
        {
          case AUDCMD_SETPLAYERSTATUS:
            deactivatePlayer();
            break;

          case AUDCMD_SETRECORDERSTATUS:
            deactivateRecorder();
            break;

          case AUDCMD_SETBBACTIVESTATUS:
            deactivateSoundFx();
            break;
        }
      sendErrRespResult(cmd.sub_code, cmd.module_id, cmd.result);
      return;
    }

  switch (cmd.command_code)
    {
      case AUDCMD_SETPLAYERSTATUS:
        m_command_code = AUDCMD_SETPLAYERSTATUS;
            m_player_transition_que.pop();
            if (m_player_transition_que.empty())
              {
                result_code = AUDRLT_STATUSCHANGED;
                m_State    = AS_MNG_STATUS_PLAYER;
                m_SubState = AS_MNG_SUB_STATUS_PLAYREADY;
              }
            else
              {
                return;
              }
        break;

      case AUDCMD_SETRECORDERSTATUS:
        m_command_code = AUDCMD_SETRECORDERSTATUS;
            result_code = AUDRLT_STATUSCHANGED;
            m_State    = AS_MNG_STATUS_RECORDER;
            m_SubState = AS_MNG_SUB_STATUS_RECORDERREADY;
        break;

      case AUDCMD_SETBBACTIVESTATUS:
        result_code = AUDRLT_STATUSCHANGED;
        m_State    = AS_MNG_STATUS_BASEBAND;
        m_SubState = AS_MNG_SUB_STATUS_BASEBANDREADY;
        break;


      case AUDCMD_POWERON:
        if (m_command_code == AUDCMD_SETPLAYERSTATUS)
          {
            result_code = AUDRLT_STATUSCHANGED;
            m_State    = AS_MNG_STATUS_PLAYER;
            m_SubState = AS_MNG_SUB_STATUS_PLAYREADY;
            m_command_code = 0;
          }
        else if (m_command_code == AUDCMD_SETRECORDERSTATUS)
          {
            result_code = AUDRLT_STATUSCHANGED;
            m_State    = AS_MNG_STATUS_RECORDER;
            m_SubState = AS_MNG_SUB_STATUS_RECORDERREADY;
            m_command_code = 0;
          }
        break;

      case AUDCMD_SETPOWEROFFSTATUS:
        if (cmd.sub_code == 1)
          {
            deactivateOutputMix();

            m_State    = AS_MNG_STATUS_POWEROFF;
            m_SubState = AS_MNG_SUB_STATUS_NONE;
            result_code = AUDRLT_STATUSCHANGED;
            break;
          }
        else
          {
            return;
          }

      default:
        sendErrRespResult(cmd.sub_code,
                          AS_MODULE_ID_AUDIO_MANAGER,
                          AS_RESPONSE_CODE_COMMAND_CODE_ERROR);
        return;
    }
  sendResult(result_code);
}

/*--------------------------------------------------------------------------*/
void AudioManager::cmpltOnSoundFx(const AudioMngCmdCmpltResult &cmd)
{
  uint8_t result_code = AUDRLT_ERRORRESPONSE;

  if (cmd.result != AS_RESPONSE_CODE_OK)
    {
      sendErrRespResult(cmd.sub_code, cmd.module_id, cmd.result);
      return;
    }
  switch (cmd.command_code)
    {
      case AUDCMD_SETREADYSTATUS:
        if (deactivateSoundFx())
          {
                result_code = AUDRLT_STATUSCHANGED;
              }
        break;

#ifdef AS_FEATURE_EFFECTOR_ENABLE
      case AUDCMD_STARTBB:
        result_code = AUDRLT_STARTBBCMPLT;
        m_SubState = AS_MNG_SUB_STATUS_BASEBANDACTIVE;
        break;

      case AUDCMD_STOPBB:
        m_SubState = AS_MNG_SUB_STATUS_BASEBANDREADY;
        result_code = AUDRLT_STOPBBCMPLT;
        break;

      case AUDCMD_INITMFE:
        result_code = AUDRLT_INITMFECMPLT;
        break;

      case AUDCMD_DEBUGMFEPARAM:
        result_code = AUDRLT_DEBUGMFECMPLT;
        break;

      case AUDCMD_INITMPP:
        result_code = AUDRLT_INITMPPCMPLT;
        break;

      case AUDCMD_SETMPPPARAM:
        switch (cmd.sub_code)
          {
            case SUB_SETMPP_COMMON:
              result_code = AUDRLT_SETMPPCMPLT;
              break;

            case SUB_SETMPP_XLOUD:
              result_code = AUDRLT_SETMPPCMPLT;
              break;

            default:
              sendResult(result_code, cmd.command_code);
              return;
          }
        break;

      case AUDCMD_DEBUGMPPPARAM:
        result_code = AUDRLT_DEBUGMPPCMPLT;
        break;

#endif  /* AS_FEATURE_EFFECTOR_ENABLE */
#ifdef AS_FEATURE_RECOGNIZER_ENABLE
      case AUDCMD_STARTVOICECOMMAND:
        result_code = AUDRLT_STARTVOICECOMMANDCMPLT;
        m_SubState = AS_MNG_SUB_STATUS_WAITCMDWORD;
        break;

      case AUDCMD_STOPVOICECOMMAND:
        result_code = AUDRLT_STOPVOICECOMMANDCMPLT;
        m_SubState = AS_MNG_SUB_STATUS_BASEBANDACTIVE;
        break;

#endif  /* AS_FEATURE_RECOGNIZER_ENABLE */
      case AUDCMD_POWERON:
        m_State    = AS_MNG_STATUS_READY;
        m_SubState = AS_MNG_SUB_STATUS_NONE;
        result_code = AUDRLT_STATUSCHANGED;
        break;

      default:
        sendErrRespResult(cmd.sub_code,
                          AS_MODULE_ID_AUDIO_MANAGER,
                          AS_RESPONSE_CODE_COMMAND_CODE_ERROR);
        return;
    }

  sendResult(result_code);
}

/*--------------------------------------------------------------------------*/
void AudioManager::cmpltOnPlayer(const AudioMngCmdCmpltResult &cmd)
{
#ifdef AS_FEATURE_PLAYER_ENABLE
  if (cmd.result != AS_RESPONSE_CODE_OK)
    {
      sendErrRespResult(cmd.sub_code,
                        cmd.module_id,
                        cmd.result,
                        cmd.sub_result);
      return;
    }

  uint8_t result_code = AUDRLT_ERRORRESPONSE;

  switch (cmd.command_code)
    {
      case AUDCMD_INITPLAYER:
        result_code = AUDRLT_INITPLAYERCMPLT;
        break;

      case AUDCMD_PLAYPLAYER:
        result_code = AUDRLT_PLAYCMPLT;
        m_SubState = AS_MNG_SUB_STATUS_PLAYACTIVE; /* tentative!! */
        break;

      case AUDCMD_STOPPLAYER:
        result_code = AUDRLT_STOPCMPLT;
        m_SubState = AS_MNG_SUB_STATUS_PLAYREADY; /* tentative!! */
        break;

      case AUDCMD_CLKRECOVERY:
        result_code = AUDRLT_CLKRECOVERY_CMPLT;
        break;

      case AUDCMD_SETREADYSTATUS:
        m_player_transition_que.pop();
        if (m_player_transition_que.empty())
          {
            if (deactivatePlayer())
              {
                    result_code = AUDRLT_STATUSCHANGED;
                  }
              }
        else
          {
            return;
          }
        break;

      case AUDCMD_INITSUBPLAYER:
        result_code = AUDRLT_SUBINITPLAYERCMPLT;
        break;

      case AUDCMD_PLAYSUBPLAYER:
        result_code = AUDRLT_SUBPLAYCMPLT;
        break;

      case AUDCMD_STOPSUBPLAYER:
        result_code = AUDRLT_SUBSTOPCMPLT;
        break;

      case AUDCMD_CLKRECOVERYSUB:
        result_code = AUDRLT_CLKRECOVERYSUB_CMPLT;
        break;


      case AUDCMD_POWERON:
        m_State    = AS_MNG_STATUS_READY;
        m_SubState = AS_MNG_SUB_STATUS_NONE;
        result_code = AUDRLT_STATUSCHANGED;
        break;

      case AUDCMD_SETPOWEROFFSTATUS:
        return;

      default:
        sendErrRespResult(cmd.sub_code,
                          AS_MODULE_ID_AUDIO_MANAGER,
                          AS_RESPONSE_CODE_COMMAND_CODE_ERROR);
        return;
    }

  sendResult(result_code);
#else
  sendErrRespResult(cmd.sub_code,
                    AS_MODULE_ID_AUDIO_MANAGER,
                    AS_RESPONSE_CODE_COMMAND_NOT_SUPPOT);
#endif /* AS_FEATURE_PLAYER_ENABLE */
}

/*--------------------------------------------------------------------------*/
void AudioManager::cmpltOnRecorder(const AudioMngCmdCmpltResult &cmd)
{
#ifdef AS_FEATURE_RECORDER_ENABLE
  if (cmd.result != AS_RESPONSE_CODE_OK)
    {
      sendErrRespResult(cmd.sub_code,
                        cmd.module_id,
                        cmd.result,
                        cmd.sub_result);
      return;
    }

  uint8_t result_code = AUDRLT_ERRORRESPONSE;

  switch (cmd.command_code)
    {
      case AUDCMD_INITREC:
        result_code = AUDRLT_INITRECCMPLT;
        break;

      case AUDCMD_STARTREC:
        result_code = AUDRLT_RECCMPLT;
        m_SubState = AS_MNG_SUB_STATUS_RECORDERACTIVE; /* tentative!! */
        break;

      case AUDCMD_STOPREC:
        result_code = AUDRLT_STOPRECCMPLT;
        m_SubState = AS_MNG_SUB_STATUS_RECORDERREADY; /* tentative!! */
        break;

      case AUDCMD_SETREADYSTATUS:
        if (deactivateRecorder())
          {
                result_code = AUDRLT_STATUSCHANGED;
              }
        break;

      case AUDCMD_POWERON:
        m_State    = AS_MNG_STATUS_READY;
        m_SubState = AS_MNG_SUB_STATUS_NONE;
        result_code = AUDRLT_STATUSCHANGED;
        break;

      case AUDCMD_SETPOWEROFFSTATUS:
        return;

      default:
        sendErrRespResult(cmd.sub_code,
                          AS_MODULE_ID_AUDIO_MANAGER,
                          AS_RESPONSE_CODE_COMMAND_CODE_ERROR);
        return;
    }

  sendResult(result_code);
#else
  sendErrRespResult(cmd.sub_code,
                    AS_MODULE_ID_AUDIO_MANAGER,
                    AS_RESPONSE_CODE_COMMAND_NOT_SUPPOT);
#endif /* AS_FEATURE_RECORDER_ENABLE */
}

/*--------------------------------------------------------------------------*/
void AudioManager::cmpltOnPowerOff(const AudioMngCmdCmpltResult &cmd)
{
  uint8_t result_code = AUDRLT_ERRORRESPONSE;

  if (cmd.result != AS_RESPONSE_CODE_OK)
    {
      sendErrRespResult(cmd.sub_code, cmd.module_id, cmd.result);
      return;
    }

  switch (cmd.command_code)
    {
      case AUDCMD_POWERON:
        m_State    = AS_MNG_STATUS_READY;
        m_SubState = AS_MNG_SUB_STATUS_NONE;
        sendResult(AUDRLT_STATUSCHANGED);
        return;

      default:
        sendErrRespResult(cmd.sub_code,
                          AS_MODULE_ID_AUDIO_MANAGER,
                          AS_RESPONSE_CODE_COMMAND_CODE_ERROR);
        return;
    }
  sendResult(result_code);
}

/*--------------------------------------------------------------------------*/
AsVadStatus AudioManager::m_VadState = AS_VAD_STATUS_OUT_OF_VOICE_SECTION;
#ifdef AS_FEATURE_RECOGNIZER_ENABLE
AudioFindCommandCallbackFunction  AudioManager::m_findCommandCBFunc = NULL;

void AudioManager::execFindCommandCallback(uint16_t key_word, uint8_t status)
{
  m_VadState =
    ((status != 0) ? AS_VAD_STATUS_INSIDE_VOICE_SECTION : AS_VAD_STATUS_OUT_OF_VOICE_SECTION);
  if (m_findCommandCBFunc != NULL)
    {
      (*m_findCommandCBFunc)(key_word, status);
    }
}
#endif  /* AS_FEATURE_RECOGNIZER_ENABLE */

/*--------------------------------------------------------------------------*/
void AudioManager::sendResult(uint8_t code, uint8_t sub_code)
{
  AudioResult packet;
  packet.header.packet_length = LENGTH_AUDRLT;
  packet.header.result_code   = code;
  packet.header.sub_code      = sub_code;

  if (code == AUDRLT_STATUSCHANGED)
    {
      packet.status_changed_param.changed_status = m_State;
    }

  err_t er = MsgLib::send<AudioResult>(s_appMid,
                                       MsgPriNormal,
                                       MSG_AUD_MGR_RST,
                                       m_selfDtq,
                                       packet);
  if (er)
    {
      F_ASSERT(0);
    }
}

/*--------------------------------------------------------------------------*/
void AudioManager::sendErrRespResult(uint8_t  sub_code,
                                     uint8_t  module_id,
                                     uint32_t error_code,
                                     uint32_t error_sub_code)
{
  AudioResult packet;
  packet.header.packet_length = LENGTH_AUDRLT_ERRORRESPONSE_MAX;
  packet.header.result_code   = AUDRLT_ERRORRESPONSE;
  packet.header.sub_code      = sub_code;

  packet.error_response_param.module_id      = module_id;
  packet.error_response_param.sub_module_id  = 0;
  packet.error_response_param.error_code     = error_code;
  packet.error_response_param.error_sub_code = error_sub_code;

  err_t er = MsgLib::send<AudioResult>(s_appMid,
                                       MsgPriNormal,
                                       MSG_AUD_MGR_RST,
                                       m_selfDtq,
                                       packet);
  if (er)
    {
      F_ASSERT(0);
    }
}

/*--------------------------------------------------------------------------*/
bool AudioManager::deactivatePlayer()
{
  uint32_t rst = AS_RESPONSE_CODE_OK;

      rst = bbConfig.deactivate(BB_POWER_OUTPUT);
      if (rst != AS_RESPONSE_CODE_OK)
        {
          return false;
        }

      m_State    = AS_MNG_STATUS_READY;
      m_SubState = AS_MNG_SUB_STATUS_NONE;
      return true;
    }

/*--------------------------------------------------------------------------*/
bool AudioManager::deactivateRecorder()
{
  uint32_t rst = AS_RESPONSE_CODE_OK;

  rst = bbConfig.deactivate(BB_POWER_INPUT);
  if (rst != AS_RESPONSE_CODE_OK)
    {
      return false;
    }

      m_State    = AS_MNG_STATUS_READY;
      m_SubState = AS_MNG_SUB_STATUS_NONE;

  return true;
}

/*--------------------------------------------------------------------------*/
bool AudioManager::deactivateSoundFx()
{
  uint32_t rst = AS_RESPONSE_CODE_OK;

  rst = bbConfig.deactivate(BB_POWER_OUTPUT);
  if (rst != AS_RESPONSE_CODE_OK)
    {
      return false;
    }

      m_State    = AS_MNG_STATUS_READY;
      m_SubState = AS_MNG_SUB_STATUS_NONE;

  return true;
}

/*--------------------------------------------------------------------------*/
bool AudioManager::deactivateOutputMix()
{
  uint32_t rst = AS_RESPONSE_CODE_OK;

  rst = bbConfig.deactivate(BB_POWER_OUTPUT);
  if (rst != AS_RESPONSE_CODE_OK)
    {
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
int AudioManager::getAllState(void)
{
  int allstate = MNG_ALLSTATE_READY;
  switch (m_SubState)
    {
      case AS_MNG_SUB_STATUS_NONE:
        switch (m_State)
          {
            case AS_MNG_STATUS_READY:
              break;

            case AS_MNG_STATUS_POWEROFF:
              allstate = MNG_ALLSTATE_POWEROFF;
              break;

            case AS_MNG_STATUS_THROUGH:
              allstate = MNG_ALLSTATE_THROUGH;
              break;

            default:
              F_ASSERT(0);
          }
          break;

      case AS_MNG_SUB_STATUS_PLAYREADY:
        allstate = MNG_ALLSTATE_PLAYREADY;
        break;

      case AS_MNG_SUB_STATUS_PLAYACTIVE:
        allstate = MNG_ALLSTATE_PLAYACTIVE;
        break;

      case AS_MNG_SUB_STATUS_PLAYPAUSE:
        allstate = MNG_ALLSTATE_PLAYPAUSE;
        break;

      case AS_MNG_SUB_STATUS_RECORDERREADY:
        allstate = MNG_ALLSTATE_RECODERREADY;
        break;

      case AS_MNG_SUB_STATUS_RECORDERACTIVE:
        allstate = MNG_ALLSTATE_RECODERACTIVE;
        break;

      case AS_MNG_SUB_STATUS_BASEBANDREADY:
        allstate = MNG_ALLSTATE_BBREADY;
        break;

      case AS_MNG_SUB_STATUS_BASEBANDACTIVE:
        allstate = MNG_ALLSTATE_BBACTIVE;
        break;

      case AS_MNG_SUB_STATUS_WAITCMDWORD:
        allstate = MNG_ALLSTATE_WAITCMDWORD;
        break;

      default:
        F_ASSERT(0);
    }
  return allstate;
}

/*--------------------------------------------------------------------------*/
void AudioManager::initMicGain(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_INITMICGAIN, AUDCMD_INITMICGAIN, cmd);
  if (!check)
    {
      return;
    }

  uint32_t rst = bbConfig.initMicGain(cmd);
  if (rst == AS_RESPONSE_CODE_OK)
    {
      sendResult(AUDRLT_INITMICGAINCMPLT, cmd.header.sub_code);
    }
  else
    {
      sendErrRespResult(cmd.header.sub_code, AS_MODULE_ID_AUDIO_DRIVER, rst);
    }
}

/*--------------------------------------------------------------------------*/
void AudioManager::setMicGain(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_INITMICGAIN, AUDCMD_INITMICGAIN, cmd);
  if (!check)
    {
      return;
    }

  uint32_t rst = bbConfig.setMicGain(cmd);
  if (rst == AS_RESPONSE_CODE_OK)
    {
      sendResult(AUDRLT_INITMICGAINCMPLT, cmd.header.sub_code);
    }
  else
    {
      sendErrRespResult(cmd.header.sub_code, AS_MODULE_ID_AUDIO_DRIVER, rst);
    }
}

/*--------------------------------------------------------------------------*/
void AudioManager::initI2SParam(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_INITI2SPARAM, AUDCMD_INITI2SPARAM, cmd);
  if (!check)
    {
      return;
    }

  uint32_t rst = bbConfig.initI2SParam(cmd);
  if (rst == AS_RESPONSE_CODE_OK)
    {
      sendResult(AUDRLT_INITI2SPARAMCMPLT, cmd.header.sub_code);
    }
  else
    {
      sendErrRespResult(cmd.header.sub_code, AS_MODULE_ID_AUDIO_DRIVER, rst);
    }
}

/*--------------------------------------------------------------------------*/
void AudioManager::setI2SParam(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_INITI2SPARAM, AUDCMD_INITI2SPARAM, cmd);
  if (!check)
    {
      return;
    }

  uint32_t rst = bbConfig.setI2SParam(cmd);
  if (rst == AS_RESPONSE_CODE_OK)
    {
      sendResult(AUDRLT_INITI2SPARAMCMPLT, cmd.header.sub_code);
    }
  else
    {
      sendErrRespResult(cmd.header.sub_code, AS_MODULE_ID_AUDIO_DRIVER, rst);
    }
}

/*--------------------------------------------------------------------------*/
void AudioManager::initDEQParam(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_INITDEQPARAM, AUDCMD_INITDEQPARAM, cmd);
  if (!check)
    {
      return;
    }

  uint32_t rst = bbConfig.initDEQParam(cmd);
  if (rst == AS_RESPONSE_CODE_OK)
    {
      sendResult(AUDRLT_INITDEQPARAMCMPLT, cmd.header.sub_code);
    }
  else
    {
      sendErrRespResult(cmd.header.sub_code, AS_MODULE_ID_AUDIO_DRIVER, rst);
    }
}

/*--------------------------------------------------------------------------*/
void AudioManager::initOutputSelect(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_INITOUTPUTSELECT, AUDCMD_INITOUTPUTSELECT, cmd);
  if (!check)
    {
      return;
    }

  uint32_t rst = bbConfig.initOutputSelect(cmd);
  if (rst == AS_RESPONSE_CODE_OK)
    {
      sendResult(AUDRLT_INITOUTPUTSELECTCMPLT, cmd.header.sub_code);
    }
  else
    {
      sendErrRespResult(cmd.header.sub_code, AS_MODULE_ID_AUDIO_DRIVER, rst);
    }
}

/*--------------------------------------------------------------------------*/
void AudioManager::setOutputSelect(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_INITOUTPUTSELECT, AUDCMD_INITOUTPUTSELECT, cmd);
  if (!check)
    {
      return;
    }

  uint32_t rst = bbConfig.setOutputSelect(cmd);
  if (rst == AS_RESPONSE_CODE_OK)
    {
      sendResult(AUDRLT_INITOUTPUTSELECTCMPLT, cmd.header.sub_code);
    }
  else
    {
      sendErrRespResult(cmd.header.sub_code, AS_MODULE_ID_AUDIO_DRIVER, rst);
    }
}

/*--------------------------------------------------------------------------*/
void AudioManager::initDNCParam(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_INITDNCPARAM, AUDCMD_INITDNCPARAM, cmd);
  if (!check)
    {
      return;
    }

  uint32_t rst = bbConfig.initDNCParam(cmd);
  if (rst == AS_RESPONSE_CODE_OK)
    {
      sendResult(AUDRLT_INITDNCPARAMCMPLT, cmd.header.sub_code);
    }
  else
    {
      sendErrRespResult(cmd.header.sub_code, AS_MODULE_ID_AUDIO_DRIVER, rst);
    }
}

/*--------------------------------------------------------------------------*/
void AudioManager::initClearStereo(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_INITCLEARSTEREO, AUDCMD_INITCLEARSTEREO, cmd);
  if (!check)
    {
      return;
    }

  uint32_t rst = bbConfig.initClearStereo(cmd);
  if (rst == AS_RESPONSE_CODE_OK)
    {
      sendResult(AUDRLT_INITCLEARSTEREOCMPLT, cmd.header.sub_code);
    }
  else
    {
      sendErrRespResult(cmd.header.sub_code, AS_MODULE_ID_AUDIO_DRIVER, rst);
    }
}

/*--------------------------------------------------------------------------*/
void AudioManager::setClearStereo(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_INITCLEARSTEREO, AUDCMD_INITCLEARSTEREO, cmd);
  if (!check)
    {
      return;
    }

  uint32_t rst = bbConfig.setClearStereo(cmd);
  if (rst == AS_RESPONSE_CODE_OK)
    {
      sendResult(AUDRLT_INITCLEARSTEREOCMPLT, cmd.header.sub_code);
    }
  else
    {
      sendErrRespResult(cmd.header.sub_code, AS_MODULE_ID_AUDIO_DRIVER, rst);
    }
}

/*--------------------------------------------------------------------------*/
void AudioManager::setVolume(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_SETVOLUME, AUDCMD_SETVOLUME, cmd);
  if (!check)
    {
      return;
    }

  uint32_t rst = bbConfig.setVolume(cmd);
  if (rst == AS_RESPONSE_CODE_OK)
    {
      sendResult(AUDRLT_SETVOLUMECMPLT, cmd.header.sub_code);
    }
  else
    {
      sendErrRespResult(cmd.header.sub_code, AS_MODULE_ID_AUDIO_DRIVER, rst);
    }
}

/*--------------------------------------------------------------------------*/
void AudioManager::setVolumeMute(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_SETVOLUMEMUTE, AUDCMD_SETVOLUMEMUTE, cmd);
  if (!check)
    {
      return;
    }

  uint32_t rst = bbConfig.setVolumeMute(cmd);
  if (rst == AS_RESPONSE_CODE_OK)
    {
      sendResult(AUDRLT_SETVOLUMEMUTECMPLT, cmd.header.sub_code);
    }
  else
    {
      sendErrRespResult(cmd.header.sub_code, AS_MODULE_ID_AUDIO_DRIVER, rst);
    }
}

/*--------------------------------------------------------------------------*/
void AudioManager::setBeep(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_SETBEEPPARAM, AUDCMD_SETBEEPPARAM, cmd);
  if (!check)
    {
      return;
    }

  uint32_t rst = bbConfig.setBeep(cmd);
  if (rst == AS_RESPONSE_CODE_OK)
    {
      sendResult(AUDRLT_SETBEEPCMPLT, cmd.header.sub_code);
    }
  else
    {
      sendErrRespResult(cmd.header.sub_code, AS_MODULE_ID_AUDIO_DRIVER, rst);
    }
}


/*--------------------------------------------------------------------------*/
void AudioManager::setThroughPath(AudioCommand &cmd)
{
  bool check =
    packetCheck(LENGTH_SET_THROUGH_PATH, AUDCMD_SETTHROUGHPATH, cmd);
  if (!check)
    {
      return;
    }

  uint32_t rst = bbConfig.setThroughPath(cmd);
  if (rst != AS_RESPONSE_CODE_OK)
    {
      sendErrRespResult(cmd.header.sub_code,
                        AS_MODULE_ID_AUDIO_DRIVER,
                        rst);
      return;
    }

  sendResult(AUDRLT_SETTHROUGHPATHCMPLT);
}

/*--------------------------------------------------------------------------*/
bool AudioManager::packetCheck(uint8_t      length,
                               uint8_t      command_code,
                               AudioCommand &cmd)
{
  if (length != cmd.header.packet_length)
    {
      sendErrRespResult(cmd.header.sub_code,
                        AS_MODULE_ID_AUDIO_MANAGER,
                        AS_RESPONSE_CODE_PACKET_LENGTH_ERROR);
      return false;
    }
  if (command_code != cmd.header.command_code)
    {
      sendErrRespResult(cmd.header.sub_code,
                        AS_MODULE_ID_AUDIO_MANAGER,
                        AS_RESPONSE_CODE_COMMAND_CODE_ERROR);
      return false;
    }
  return true;
}

/*--------------------------------------------------------------------------*/
/* packet length consistency check. */

/* GetStatus command (AUDCMD_GETSTATUS) packet length. */

S_ASSERT((LENGTH_GETSTATUS << 2) ==
  (sizeof(AudioCommandHeader) + 4));

#ifdef AS_FEATURE_EFFECTOR_ENABLE

/* InitMFE command (AUDCMD_INITMFE) packet length. */

S_ASSERT((LENGTH_INITMFE << 2) ==
  (sizeof(AudioCommandHeader) + sizeof(InitMFEParam)));

/* StartBB command (AUDCMD_STARTBB) packet length. */

S_ASSERT((LENGTH_STARTBB << 2) ==
  (sizeof(AudioCommandHeader) + sizeof(StartBBParam)));

/* StopBB command (AUDCMD_STOPBB) packet length. */

S_ASSERT((LENGTH_STOPBB << 2) ==
  (sizeof(AudioCommandHeader) + sizeof(StopBBParam)));

/* DebugMFEParam command (AUDCMD_DEBUGMFEPARAM) packet length. */
S_ASSERT((LENGTH_DEBUGMFEPARAM << 2) ==
  (sizeof(AudioCommandHeader) + sizeof(DebugMFEParam)));

/* InitMPP command (AUDCMD_INITMPP) packet length. */
S_ASSERT((LENGTH_INITMPP << 2) ==
  (sizeof(AudioCommandHeader) + sizeof(InitMPPParam)));

/* SetMPP command (AUDCMD_SETMPPPARAM) packet length. */

S_ASSERT((LENGTH_SUB_SETMPP_COMMON << 2) ==
  (sizeof(AudioCommandHeader) + sizeof(MppCommonSet)));

/* SetMPP command (AUDCMD_SETMPPPARAM) packet length. */
S_ASSERT((LENGTH_SUB_SETMPP_XLOUD << 2) ==
  (sizeof(AudioCommandHeader) + sizeof(MppXloudSet)));

/* DebugMPPParam command (AUDCMD_DEBUGMPPPARAM) packet length. */
S_ASSERT((LENGTH_DEBUGMPPPARAM << 2) ==
  (sizeof(AudioCommandHeader) + sizeof(DebugMPPParam)));

/* DebugMPPParam command (AUDCMD_DEBUGMPPPARAM) packet length. */

S_ASSERT((LENGTH_SUB_DEBUGMPP_COMMON << 2) ==
  (sizeof(AudioCommandHeader) + sizeof(MppCommonDebug)));

/* DebugMPPParam command (AUDCMD_DEBUGMPPPARAM) packet length. */

S_ASSERT((LENGTH_SUB_DEBUGMPP_XLOUD << 2) ==
  (sizeof(AudioCommandHeader) + sizeof(MppXloudDebug)));

/* SetBaseBandStatus command (AUDCMD_SETBASEBANDSTATUS) packet length. */

S_ASSERT((LENGTH_SET_BASEBAND_STATUS << 2) ==
  (sizeof(AudioCommandHeader) + sizeof(SetBaseBandStatusParam)));
#endif /* #ifdef AS_FEATURE_EFFECTOR_ENABLE */

#ifdef AS_FEATURE_PLAYER_ENABLE

/* SetPlayerStaus command (AUDCMD_SETPLAYERSTATUS) packet length. */

S_ASSERT((LENGTH_SET_PLAYER_STATUS << 2) ==
  (sizeof(AudioCommandHeader) + sizeof(SetPlayerStsParam)));

/* SetPlayerStaus command (AUDCMD_INITPLAYER) packet length.
 * SetPlayerStaus command (AUDCMD_INITSUBPLAYER) packet length.
 */

S_ASSERT((LENGTH_INIT_PLAYER << 2) ==
  (sizeof(AudioCommandHeader) + sizeof(AsInitPlayerParam)));
S_ASSERT((LENGTH_INIT_SUBPLAYER << 2) ==
  (sizeof(AudioCommandHeader) + sizeof(AsInitPlayerParam)));

/* PlayPlayer command (AUDCMD_PLAYPLAYER) packet length.
 * PlayPlayer command (AUDCMD_PLAYSUBPLAYER) packet length.
 */

S_ASSERT((LENGTH_PLAY_PLAYER << 2) ==
  (sizeof(AudioCommandHeader) + 4));
S_ASSERT((LENGTH_PLAY_SUBPLAYER << 2) ==
  (sizeof(AudioCommandHeader) + 4));

/* StopPlayer command (AUDCMD_STOPPLAYER) packet length.
 * StopPlayer command (AUDCMD_STOPSUBPLAYER) packet length.
 */

S_ASSERT((LENGTH_STOP_PLAYER << 2) ==
  (sizeof(AudioCommandHeader) + 4));
S_ASSERT((LENGTH_STOP_SUBPLAYER << 2) ==
  (sizeof(AudioCommandHeader) + 4));
#endif /* #ifdef AS_FEATURE_PLAYER_ENABLE */

#ifdef AS_FEATURE_RECOGNIZER_ENABLE

/* StartVoiceComamnd command (AUDCMD_STARTVOICECOMMAND) packet length. */

S_ASSERT((LENGTH_START_VOICE_COMMAND << 2) ==
  (sizeof(AudioCommandHeader) + sizeof(StartVoiceCommandParam)));

/* StopVoiceCommand command (AUDCMD_STOPVOICECOMMAND) packet length. */

S_ASSERT((LENGTH_STOP_VOICE_COMMAND << 2) ==
  (sizeof(AudioCommandHeader) + 4));
#endif /* #ifdef AS_FEATURE_RECOGNIZER_ENABLE */

#ifdef AS_FEATURE_RECORDER_ENABLE

/* SetRecoderStatus command (AUDCMD_SETRECORDERSTATUS) packet length. */

S_ASSERT((LENGTH_SET_RECORDER_STATUS << 2) ==
  (sizeof(AudioCommandHeader) + sizeof(AsSetRecorderStatusParam)));

/* InitRecorder command (AUDCMD_INITREC) packet length. */
S_ASSERT((LENGTH_INIT_RECORDER << 2) ==
  (sizeof(AudioCommandHeader) + sizeof(AsInitRecorderParam)));

/* StartRec command (AUDCMD_STARTREC) packet length. */

S_ASSERT((LENGTH_START_RECORDER << 2) ==
  (sizeof(AudioCommandHeader) + 4));

/* StopRec command (AUDCMD_STOPREC) packet length. */

S_ASSERT((LENGTH_STOP_RECORDER << 2) ==
  (sizeof(AudioCommandHeader) + 4));
#endif /* #ifdef AS_FEATURE_RECORDER_ENABLE */

/* SetReadyStatus command (AUDCMD_SETREADYSTATUS) packet length. */

S_ASSERT((LENGTH_SET_READY_STATUS << 2) ==
  (sizeof(AudioCommandHeader) + 4));

/* Reslt Code Packet length. */

S_ASSERT((LENGTH_AUDRLT << 2) ==
  (sizeof(AudioResultHeader) + 4));
S_ASSERT((LENGTH_AUDRLT << 2) ==
  (sizeof(AudioResultHeader) + sizeof(NotifyStatus)));
S_ASSERT((LENGTH_AUDRLT << 2) ==
  (sizeof(AudioResultHeader) + sizeof(StatusChangedParam)));
S_ASSERT((LENGTH_AUDRLT_ERRORRESPONSE_MAX << 2) ==
  (sizeof(AudioResultHeader) + sizeof(ErrorResponseParam)));
