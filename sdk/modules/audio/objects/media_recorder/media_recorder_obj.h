/****************************************************************************
 * modules/audio/objects/media_recorder/media_recorder_obj.h
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

#ifndef __MODULES_AUDIO_OBJECTS_MEDIA_RECORDER_MEDIA_RECORDER_OBJ_H
#define __MODULES_AUDIO_OBJECTS_MEDIA_RECORDER_MEDIA_RECORDER_OBJ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "memutils/os_utils/chateau_osal.h"
#include "memutils/message/Message.h"
#include "memutils/s_stl/queue.h"
#include "memutils/memory_manager/MemHandle.h"
#include "audio/audio_high_level_api.h"
#include "common/audio_message_types.h"
#include "audio_recorder_sink.h"
#include "components/capture/capture_component.h"
#include "wien2_common_defs.h"
#include "audio_state.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CAPTURE_DELAY_STAGE_NUM   3  /* TODO:
                                      * baseband仕様依存
                                      * ここで定義するのは適切ではない
                                      */
#define CAPTURE_PCM_BUF_QUE_SIZE  7
#define FALT_HANDLE_ID            0xFF

/****************************************************************************
 * Public Types
 ****************************************************************************/

class VoiceRecorderObjectTask {
public:
  static void create(MsgQueId self_dtq,
                     MsgQueId manager_dtq);

private:
  VoiceRecorderObjectTask(MsgQueId self_dtq,
                          MsgQueId manager_dtq):
    m_self_dtq(self_dtq),
    m_manager_dtq(manager_dtq),
    m_state(AS_MODULE_ID_MEDIA_RECORDER_OBJ, "", RecorderStateInactive),
    m_channel_num(2),
    m_pcm_bit_width(AudPcm16Bit),
    m_pcm_byte_len(2),  /* This value depends on the value of
                         * m_pcm_bit_width.
                         */
    m_sampling_rate(48000),
    m_codec_type(InvalidCodecType),
    m_output_device(AS_SETRECDR_STS_OUTPUTDEVICE_EMMC),
    m_p_output_device_handler(NULL),
    m_capture_from_mic_hdlr(MAX_CAPTURE_COMP_INSTANCE_NUM)
  {}

  enum RecorderState_e
  {
    RecorderStateInactive = 0,
    RecorderStateReady,
    RecorderStateRecording,
    RecorderStateStopping,
    RecorderStateOverflow,
    RecorderStateWaitStop,
    RecorderStateNum
  };

  MsgQueId m_self_dtq;
  MsgQueId m_manager_dtq;
  AudioState<RecorderState_e> m_state;
  int8_t  m_channel_num;
  AudioPcmBitWidth m_pcm_bit_width;
  int8_t  m_pcm_byte_len;
  int32_t m_sampling_rate;
  int32_t m_max_capture_pcm_size;
  int32_t m_max_output_pcm_size;
  AudioCodec m_codec_type;
  AsSetRecorderStsOutputDevice m_output_device;
  AsRecorderOutputDeviceHdlr* m_p_output_device_handler;
  CaptureDevice m_input_device;
  int8_t  m_complexity;
  int32_t m_bit_rate;
  AudioRecorderSink m_rec_sink;
  bool m_fifo_overflow;

  CaptureComponentHandler m_capture_from_mic_hdlr;

  typedef void (VoiceRecorderObjectTask::*MsgProc)(MsgPacket *);
  static MsgProc MsgProcTbl[AUD_VRC_MSG_NUM][RecorderStateNum];
  static MsgProc RstProcTbl[AUD_VRC_RST_MSG_NUM][RecorderStateNum];

  typedef s_std::Queue<MemMgrLite::MemHandle, APU_COMMAND_QUEUE_SIZE>
    OutputBufMhQueue;
  OutputBufMhQueue m_output_buf_mh_que;

  s_std::Queue<AudioCommand, 1> m_external_cmd_que;

  struct mic_in_data_s
  {
    MemMgrLite::MemHandle mh;
    uint32_t pcm_sample;
  };

  typedef s_std::Queue<mic_in_data_s, CAPTURE_PCM_BUF_QUE_SIZE> MicInMhQueue;
  MicInMhQueue m_mic_in_buf_mh_que;

  static AudioCodec asCodecTypeDef2AudioCodecDef[AS_INITREC_CODECTYPE_NUM];
  static int32_t chNumDef2Value[AS_INITREC_CHNL_NUM];
  static AudioPcmBitWidth bitLenDef2WienDef[AS_INITREC_BITLENGTH_NUM];
  static int32_t samplingRateDef2Value[AS_INITREC_SAMPLING_NUM];

  void run();
  void parse(MsgPacket *);

  void illegal(MsgPacket *);
  void activate(MsgPacket *);
  void deactivate(MsgPacket *);

  void init(MsgPacket *);
  void startOnReady(MsgPacket *);
  void stopOnRec(MsgPacket *);
  void stopOnOverflow(MsgPacket *);
  void stopOnWait(MsgPacket *);

  void illegalFilterDone(MsgPacket *);
  void filterDoneOnRec(MsgPacket *);
  void filterDoneOnStop(MsgPacket *);
  void filterDoneOnOverflow(MsgPacket *);

  void illegalEncDone(MsgPacket *);
  void encDoneOnRec(MsgPacket *);
  void encDoneOnStop(MsgPacket *);
  void encDoneOnOverflow(MsgPacket *);

  void illegalCaptureDone(MsgPacket *);
  void captureDoneOnRec(MsgPacket *);
  void captureDoneOnStop(MsgPacket *);

  bool startCapture();
  void execEnc(void *p_pcm, uint32_t pcm_size);
  void stopEnc(void);

  void* getMicInBufAddr();
  void* getOutputBufAddr();

  uint32_t loadCodec(AudioCodec, int32_t, uint32_t *);
  bool unloadCodec(void);

  bool freeMicInBuf()
    {
      if (!m_mic_in_buf_mh_que.pop())
        {
          MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
          return false;
        }

      return true;
    }
  bool freeOutputBuf()
    {
      if (!m_output_buf_mh_que.pop())
        {
          MEDIA_RECORDER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
          return false;
        }

      return true;
    }

  /* MP3Encの場合、16000Hz, 22050Hz, 24000Hz のfsは、1au処理単位は
   * MP2規格仕様になり 1152/2=576サンプルとなるため
   * SampleNumPerFrame[m_codec_type] の値を 2で割算する。
   * さらに48000->録音fsへのSRC変換後が 576サンプルになるように
   * 48000/m_sampling_rateの値を掛算した結果を返す。
   * 以下の変換処理は、fsが、48000Hz, 16000hzのみ対応した処理です。
   * 32000Hz, 44100Hz等に対応する場合は、その場合もSRC変換後が
   * 1152サンプルになるように変換する処理が必要になる。
   */
  uint32_t getPcmCaptureSample()
    {
      if (m_codec_type == AudCodecMP3 && m_sampling_rate < 32000)
        {
          return (SampleNumPerFrame[m_codec_type] / 2 * 48000 /
            m_sampling_rate);
        }
      else if (m_codec_type == AudCodecOPUS)
        {
          /* 20ms. */

          return ((m_sampling_rate / 50) * (48000 / m_sampling_rate));
        }
        return SampleNumPerFrame[m_codec_type];
    }

  void sendAudioCmdCmplt(const AudioCommand &cmd,
                         uint32_t result,
                         uint32_t sub_result = 0)
    {
      AudioMngCmdCmpltResult cmplt(cmd.header.command_code,
                                   cmd.header.sub_code,
                                   result, AS_MODULE_ID_MEDIA_RECORDER_OBJ,
                                   sub_result);
      err_t er = MsgLib::send<AudioMngCmdCmpltResult>(m_manager_dtq,
                                                      MsgPriNormal,
                                                      MSG_TYPE_AUD_RES,
                                                      m_self_dtq,
                                                      cmplt);
      if (ERR_OK != er)
        {
          F_ASSERT(0);
        }
    }

  uint32_t isValidActivateParam(const AudioCommand& cmd);
  uint32_t isValidInitParam(const AudioCommand& cmd);
  uint32_t isValidInitParamMP3(const AudioCommand& cmd);
  uint32_t isValidInitParamWAV(const AudioCommand& cmd);
  uint32_t isValidInitParamOPUS(const AudioCommand& cmd);
  void writeToDataSinker(const MemMgrLite::MemHandle& mh, uint32_t byte_size);

  bool getInputDeviceHdlr(void);
  bool delInputDeviceHdlr(void);

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

extern "C" {

bool AS_deactivateVoiceRecorder(void);

}

__WIEN2_END_NAMESPACE

#endif /* __MODULES_AUDIO_OBJECTS_MEDIA_RECORDER_MEDIA_RECORDER_OBJ_H */

