/***********************************************************************
 *
 *      File Name: capture_component.h
 *
 *      Description: Header file of capture component
 *
 *      Notes: (C) Copyright 2015 Sony Corporation
 *
 *      Author: Hsingying Ho
 *
 ***********************************************************************
 */

#ifndef CAPTURE_COMPONENT_H
#define CAPTURE_COMPONENT_H

#include "wien2_common_defs.h"
#include "wien2_internal_packet.h"
#include "audio_state.h"

#include "memutils/os_utils/chateau_osal.h"

#include "memutils/s_stl/queue.h"

#include "memutils/message/Message.h"
#include "memutils/memory_manager/MemHandle.h"

#include "common/audio_message_types.h"

#include "dma_controller/audio_dma_drv_api.h"
#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE

#define MAX_CAPTURE_QUE_NUM (9)
#define MAX_AC_IN_CH_NUM    (4)
#define MAX_I2S_IN_CH_NUM   (2)

#ifndef CONFIG_AUDIOUTILS_CAPTURE_CH_NUM
#define CONFIG_AUDIOUTILS_CAPTURE_CH_NUM 2
#endif
/* Equals to Max number of DMAC resource */
#define MAX_CAPTURE_COMP_INSTANCE_NUM  CONFIG_AUDIOUTILS_CAPTURE_CH_NUM

enum CaptureDevice
{
  CaptureDeviceAnalogMic = 0,
  CaptureDeviceDigitalMic,
  CaptureDeviceI2S,
  CaptureDeviceTypeNum
};

struct ExecCaptureComponentParam
{
  void    *p_pcm;
  int32_t pcm_sample;
};


struct CaptureComponentCmpltParam
{
  CaptureDevice             output_device;
  bool                      end_flag;
  ExecCaptureComponentParam exec_capture_comp_param;
};

typedef void (* CaptureDoneCB)(CaptureComponentCmpltParam p_param);

struct InitCaptureComponentParam
{
  uint8_t          capture_ch_num;
  AudioPcmBitWidth capture_bit_width;
  CaptureDoneCB    callback;
};

struct ActCaptureComponentParam
{
  asPathSelParam path_sel_param;
  CaptureDevice  output_device;
};

struct StopCaptureComponentParam
{
  asDmacStopMode mode;
};

typedef uint32_t CaptureComponentHandler;

enum NotifyType
{
  NtfDmaCmplt = 0,
  NotifyTypeNum
};

struct NotifyCaptureComponentParam
{
  NotifyType   type;
  E_AS_DMA_INT code;
};

struct CaptureComponentParam
{
  CaptureComponentHandler handle;

  union
  {
    ActCaptureComponentParam    act_capture_comp_param;
    InitCaptureComponentParam   init_capture_comp_param;
    ExecCaptureComponentParam   exec_capture_comp_param;
    StopCaptureComponentParam   stop_capture_comp_param;
    NotifyCaptureComponentParam notify_capture_comp_param;
  };
};

extern "C" {

bool AS_get_capture_comp_handler(CaptureComponentHandler *p_handle,
                                 CaptureDevice device_type,
                                 uint8_t mic_channel);

bool AS_release_capture_comp_handler(CaptureComponentHandler p_handle);

bool AS_init_capture(const CaptureComponentParam& param);

bool AS_exec_capture(const CaptureComponentParam& param);

bool AS_stop_capture(const CaptureComponentParam& param);

} /* extern "C" */

class CaptureComponent
{
public:
  CaptureComponent()
    : m_dmac_id(AS_DMAC_ID_NONE)
    , m_output_device(CaptureDeviceTypeNum)
    , m_state(AS_MODULE_ID_CAPTURE_CMP, "", Booted)
  {}

  ~CaptureComponent() {}

  asDmacSelId m_dmac_id;

  CaptureDoneCB m_callback;

  CaptureDevice m_output_device;

  typedef s_std::Queue<CaptureComponentParam, MAX_CAPTURE_QUE_NUM> CapDataQue;
  CapDataQue m_cap_data_que;

  void create(AS_DmaDoneCb, AS_ErrorCb, MsgQueId, MsgQueId);

private:
  enum State
  {
    Booted = 0,
    Ready,
    PreAct,
    Act,
    StateNum
  };

  AS_ErrorCb   m_dma_err_cb;
  AS_DmaDoneCb m_dma_done_cb;

  MsgQueId m_self_dtq;
  MsgQueId m_self_sync_dtq;

  asPathSelParam m_path_sel_param;

  AudioState<State> m_state;

  typedef s_std::Queue<asReadDmacParam, 2> ReadDmacCmdQue;
  ReadDmacCmdQue m_read_dmac_cmd_que;

  typedef bool (CaptureComponent::*EvtProc)(const CaptureComponentParam&);
  static EvtProc EvetProcTbl[AUD_BB_MSG_NUM][StateNum];

  void run();
  bool parse(MsgPacket *msg);

  bool act(const CaptureComponentParam& param);
  bool deact(const CaptureComponentParam& param);
  bool illegal(const CaptureComponentParam& param);
  bool init(const CaptureComponentParam& param);
  bool execOnRdy(const CaptureComponentParam& param);
  bool execOnPreAct(const CaptureComponentParam& param);
  bool execOnAct(const CaptureComponentParam& param);
  bool stopOnPreAct(const CaptureComponentParam& param);
  bool stopOnAct(const CaptureComponentParam& param);
  bool notify(const CaptureComponentParam& param);
};


__WIEN2_END_NAMESPACE

#endif /* CAPTURE_COMPONENT_H */

