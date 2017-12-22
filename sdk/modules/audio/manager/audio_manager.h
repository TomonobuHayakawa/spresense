/****************************************************************************
 * modules/audio/manager/audio_manager.h
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

#ifndef __AUDIO_MANAGER_H
#define __AUDIO_MANAGER_H

#include <audio/audio_high_level_api.h>
#include <arch/chip/cxd56_audio.h>
#include "attention.h"
#include "wien2_common_defs.h"
#include "audio_manager_message_types.h"
#include "memutils/s_stl/queue.h"
#include "wien2_internal_packet.h"
#include "baseband_config.h"

#ifndef __ASSEMBLY__

#  undef EXTERN
#  if defined(__cplusplus)
#    define EXTERN extern "C"
extern "C"
{
#  else
#    define EXTERN extern
#  endif

int cxd56_audio_bb_register(FAR const char *devpath);
int cxd56_audio_bb_unregister(FAR const char *devpath);

#  undef EXTERN
#  if defined(__cplusplus)
}
#  endif

#endif /* __ASSEMBLY__ */


__WIEN2_BEGIN_NAMESPACE

/*--------------------------------------------------------------------*/
class AudioManager
{
public:
  static void create(MsgQueId selfDtq,
                     MsgQueId playerDtq,
                     MsgQueId subplayerDtq);

  ~AudioManager()
  {
    cxd56_audio_bb_unregister("/dev/audio/baseband");
  };

private:
  AudioManager(MsgQueId selfDtq,
               MsgQueId playerDtq,
               MsgQueId subplayerDtq) :
    m_selfDtq(selfDtq),
    m_playerDtq(playerDtq),
    m_subplayerDtq(subplayerDtq),
    m_State(AS_MNG_STATUS_POWEROFF),
    m_SubState(AS_MNG_SUB_STATUS_NONE),
    m_attentionCBFunc(NULL),
    m_active_player(0),
    m_enable_sound_effect(AS_DISABLE_SOUNDEFFECT)
  {
    cxd56_audio_bb_register("/dev/audio/baseband");
  };

  MsgQueId m_selfDtq;
  MsgQueId m_playerDtq;
  MsgQueId m_subplayerDtq;

  AsMngStatus    m_State;
  AsMngSubStatus m_SubState;

  static AsVadStatus m_VadState;

  enum MngAllState
  {
      MNG_ALLSTATE_READY = 0,
      MNG_ALLSTATE_PLAYREADY,
      MNG_ALLSTATE_PLAYACTIVE,
      MNG_ALLSTATE_PLAYPAUSE,
      MNG_ALLSTATE_RECODERREADY,
      MNG_ALLSTATE_RECODERACTIVE,
      MNG_ALLSTATE_BBREADY,
      MNG_ALLSTATE_BBACTIVE,
      MNG_ALLSTATE_WAITCMDWORD,
      MNG_ALLSTATE_WAITKEY,
      MNG_ALLSTATE_POWEROFF,
      MNG_ALLSTATE_NUM /*11*/
  };

  AudioAttentionCallbackFunction          m_attentionCBFunc;
#ifdef CONFIG_AUDIOUTILS_VOICE_COMMAND
  static AudioFindCommandCallbackFunction m_findCommandCBFunc;
#endif
  BasebandConfig bbConfig;
  uint32_t m_active_player;
  uint32_t m_enable_sound_effect;
  uint32_t m_command_code;
  static const int   ActivePlayerNum = 2;
  s_std::Queue<uint8_t, ActivePlayerNum> m_player_transition_que;

  typedef void (AudioManager::*MsgProc)(AudioCommand &cmd);
  typedef void (AudioManager::*RstProc)(const AudioMngCmdCmpltResult &result);
  static MsgProc MsgProcTbl[AUD_MGR_MSG_NUM][MNG_ALLSTATE_NUM];
  static RstProc RstProcTbl[1][AS_MNG_STATUS_NUM];

  void run(void);
  void parse(FAR MsgPacket *);

  void illegal(AudioCommand &cmd);
  void ignore(AudioCommand &cmd);
  void powerOn(AudioCommand &cmd);
  void powerOff(AudioCommand &cmd);
  void setPowerOffOnWait(AudioCommand &cmd);
  void soundFx(AudioCommand &cmd);
  void mfe(AudioCommand &cmd);
  void mpp(AudioCommand &cmd);
  void player(AudioCommand &cmd);
  void recorder(AudioCommand &cmd);
  void initMicGain(AudioCommand &cmd);
  void setMicGain(AudioCommand &cmd);
  void initI2SParam(AudioCommand &cmd);
  void setI2SParam(AudioCommand &cmd);
  void initDEQParam(AudioCommand &cmd);
  void initOutputSelect(AudioCommand &cmd);
  void setOutputSelect(AudioCommand &cmd);
  void initDNCParam(AudioCommand &cmd);
  void initClearStereo(AudioCommand &cmd);
  void setClearStereo(AudioCommand &cmd);
  void setVolume(AudioCommand &cmd);
  void setVolumeMute(AudioCommand &cmd);
  void setBeep(AudioCommand &cmd);
  void setWait(AudioCommand &cmd);
  void setRdyOnAct(AudioCommand &cmd);
  void setRdyOnWait(AudioCommand &cmd);
  void setRdyOnPlay(AudioCommand &cmd);
  void setRdyOnRecorder(AudioCommand &cmd);
  void setActive(AudioCommand &cmd);
  void setPlayerStatus(AudioCommand &cmd);
  void setRecorder(AudioCommand &cmd);
  void outputMixSoundFx(AudioCommand &cmd);
  void subPlayer(AudioCommand &cmd);
  void voiceCommand(AudioCommand &cmd);
  void getstatus(AudioCommand &cmd);
  void initAttentions(AudioCommand &cmd);

  void illegalCmplt(const AudioMngCmdCmpltResult &cmd);
  void cmpltOnReady(const AudioMngCmdCmpltResult &cmd);
  void cmpltOnSoundFx(const AudioMngCmdCmpltResult &cmd);
  void cmpltOnWaitkey(const AudioMngCmdCmpltResult &cmd);
  void cmpltOnPlayer(const AudioMngCmdCmpltResult &cmd);
  void cmpltOnRecorder(const AudioMngCmdCmpltResult &cmd);
  void cmpltOnPowerOff(const AudioMngCmdCmpltResult &cmd);

  int getAllState(void);

  static void execFindTriggerCallback(void);
  static void execFindCommandCallback(uint16_t key_word, uint8_t status);

  void execAttentions(const AttentionInfo&);
  static void execAttentionsCallback(FAR void *);

  void sendResult(uint8_t code, uint8_t sub_code = 0);
  void sendErrRespResult(uint8_t  sub_code,
                         uint8_t  module_id,
                         uint32_t error_code,
                         uint32_t error_sub_code = 0);
  bool deactivatePlayer();
  bool deactivateRecorder();
  bool deactivateSoundFx();
  bool deactivateOutputMix();

  uint32_t setPlayerStatusParamCheck(uint8_t  input_dev,
                                     uint8_t  output_dev,
                                     FAR void *input_handler);
  bool packetCheck(uint8_t length, uint8_t command_code, AudioCommand &cmd);
};

__WIEN2_END_NAMESPACE

#endif /* __AUDIO_MANAGER_H */
