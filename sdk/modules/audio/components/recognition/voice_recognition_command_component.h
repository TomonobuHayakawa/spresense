/********************************************************************
 *
 *  File Name: voice_recognition_command_component.h
 *
 *  Description: Component of Voice Recognition Command
 *
 *  Notes: (C) Copyright 2015 Sony Corporation
 *
 *  Author: Suzunosuke Hida
 *
 ********************************************************************
 */

#ifndef _VOICE_RECOGNITION_COMMAND_COMPONENT_H_
#define _VOICE_RECOGNITION_COMMAND_COMPONENT_H_

#include <stdint.h>

#ifdef __cplusplus

#include <string.h>
#include "memutils/message/Message.h"
#include "apus/apu_cmd.h"

#include "dsp_driver/include/dsp_drv.h"
#include "components/common/component_common.h"

class VoiceCmdComponent : public ComponentCommon
{
public:
  VoiceCmdComponent(MsgQueId recognizer_dtq, MsgQueId dsp_dtq)
    : m_is_found(0)
  {
    m_recognizer_dtq = recognizer_dtq;
    m_dsp_dtq = dsp_dtq;
    m_dsp_handler = NULL;
  }

  ~VoiceCmdComponent(){}

  /*-structure----------------------*/
  /* message parameter */

  typedef struct
  {
    uint8_t vad_only;     /* Is VAD process onlu ? */
    uint8_t *p_vad_param; /* Cotrol parameter for VAD */
  } InitReqParam_t;

  typedef struct
  {
    uint32_t address;
    uint32_t sample_num;
  } ExecReqParam_t;

  uint32_t act(uint32_t *dsp_inf);
  bool deact();
  int  init(InitReqParam_t *p_param);
  int  exec(ExecReqParam_t *p_param);
  int  flush();
  void recv_apu(DspDrvComPrm_t *p_dsp_param);
  MsgQueId get_apu_mid(void) { return m_dsp_dtq; };

private:
  /*-function----------------------*/

  void sendApu(Apu::Wien2ApuCmd* p_cmd);

  /*-variable-----------------------*/

  uint32_t m_recognition_type; /* VAD/VAD+WUWSR */
  uint32_t m_is_found;

  void *m_dsp_handler;

  MsgQueId m_recognizer_dtq;
  MsgQueId m_dsp_dtq;

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  DebugLogInfo m_debug_log_info;
#endif

};
#endif

#ifdef __cplusplus
extern "C" {
#endif
/* OBJECTから呼び出されるインタフェース */
extern uint32_t AS_voiceCmdCmpActivate(MsgQueId recognizer_dtq,
                                       MsgQueId dsp_dtq,
                                       uint32_t *dsp_inf);

extern bool AS_voiceCmdCmpDeactivate(void);
extern int  AS_voiceCmdCmpInit(VoiceCmdComponent::InitReqParam_t *pCmdInit);
extern int  AS_voiceCmdCmpExec(VoiceCmdComponent::ExecReqParam_t *pCmdExec);
extern int  AS_voiceCmdCmpFlush(void);
#ifdef __cplusplus
}
#endif

#endif /* _VOICE_RECOGNITION_COMMAND_COMPONENT_H_ */
