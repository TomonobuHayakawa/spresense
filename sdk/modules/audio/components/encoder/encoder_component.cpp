/***********************************************************************
 *
 *      File Name: encoder_component.cpp
 *
 *      Description: Encoder component
 *
 *      Notes: (C) Copyright 2015 Sony Corporation
 *
 *      Author: Hsingying Ho
 *
 ***********************************************************************
 */

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
#include <arch/chip/log.h>
#endif

#include "encoder_component.h"
#include "apus/cpuif_cmd.h"

#include "memutils/message/Message.h"
#include "memutils/message/MsgPacket.h"
#include "common/audio_message_types.h"
#include "dsp_driver/include/dsp_drv.h"
#include "apus/dsp_audio_version.h"
#include "wien2_internal_packet.h"

#define DBG_MODULE DBG_MODULE_AS

__WIEN2_BEGIN_NAMESPACE

static EncoderComponent *s_instance = NULL;

extern "C" {
/*--------------------------------------------------------------------
  C Interface
  --------------------------------------------------------------------*/
uint32_t AS_encode_activate(AudioCodec param,MsgQueId apu_dtq, PoolId apu_pool_id, uint32_t *dsp_inf)
{
  if (s_instance == NULL)
    {
      s_instance = new EncoderComponent(apu_dtq,apu_pool_id);
    }

  return s_instance->activate_apu(param, dsp_inf);
}

/*--------------------------------------------------------------------*/
bool AS_encode_deactivate()
{
  bool result = s_instance->deactivate_apu();

  delete s_instance;
  s_instance = NULL;

  return result;
}

/*--------------------------------------------------------------------*/
uint32_t AS_encode_init(const InitEncParam& param, uint32_t *dsp_inf)
{
  return s_instance->init_apu(param, dsp_inf);
}

/*--------------------------------------------------------------------*/
bool AS_encode_exec(const ExecEncParam& param)
{
  return s_instance->exec_apu(param);
}

/*--------------------------------------------------------------------*/
bool AS_encode_stop(const StopEncParam& param)
{
  return s_instance->flush_apu(param);
}

/*--------------------------------------------------------------------*/
bool AS_encode_recv_done(void)
{
  return s_instance->recv_done();
}

/*--------------------------------------------------------------------*/
bool AS_encode_recv_apu(void *p_param)
{
  return s_instance->recv_apu(p_param);
}

/*--------------------------------------------------------------------*/

} /* extern "C" */

/*--------------------------------------------------------------------*/
/* callback function for DSP Driver */
/*--------------------------------------------------------------------*/
void enc_dsp_done_callback(void *p_response, void *p_instance)
{
  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;

  switch (p_param->process_mode)
    {
      case Apu::CommonMode:
          if (p_param->event_type == Apu::BootEvent)
            {
              err_t er = MsgLib::send<uint32_t>(((EncoderComponent*)p_instance)->get_apu_mid(),
                                                MsgPriNormal,
                                                MSG_ISR_APU0,
                                                0,
                                                p_param->data.value);
              F_ASSERT(er == ERR_OK);
            }
          else if (p_param->event_type == Apu::ErrorEvent)
            {
              ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_ASSETION_FAIL);
            }
          break;

      case Apu::EncMode:
          AS_encode_recv_apu(p_response);
          break;

      default:
          ENCODER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          break;
    }
}

#if 0
/*--------------------------------------------------------------------*/
void enc_dsp_error_callback(DspDrvErrPrm_t *p_param)
{
  ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
  F_ASSERT(0);
}

/*--------------------------------------------------------------------*/
void enc_dsp_unload_cmplt_callback(DspDrvUnloadCmpltPrm_t *p_param)
{
  Chateau_SignalSemaphore(AS_SEM_ENC_CMP);
}
#endif

/*--------------------------------------------------------------------
    Class Methods
  --------------------------------------------------------------------*/
uint32_t EncoderComponent::activate_apu(AudioCodec param, uint32_t *dsp_inf)
{
  char filename[10];
  uint32_t encoder_dsp_version;

  ENCODER_DBG("ACT: codec %d\n", param);

  switch (param)
    {
      case AudCodecMP3:
          strncpy(filename, "MP3ENC", 10);
          encoder_dsp_version = DSP_MP3ENC_VERSION;
          break;

      case AudCodecOPUS:
          strncpy(filename, "OPUSENC", 10);
          encoder_dsp_version = DSP_OPUSENC_VERSION;
          break;

      default:
          ENCODER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          return AS_ECODE_COMMAND_PARAM_CODEC_TYPE;
    }

  /* load DSP */

  if ((m_dsp_handler = DD_Load(filename, enc_dsp_done_callback, (void *)this)) == NULL)
    {
      ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_LOAD_ERROR);
      return AS_ECODE_DSP_LOAD_ERROR;
    }

  if (!dsp_boot_check(m_apu_dtq, encoder_dsp_version, dsp_inf))
    {
      ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_VERSION_ERROR);

      if (DD_Unload(m_dsp_handler) != 0)
        {
          ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_ERROR);
        }

      return AS_ECODE_DSP_VERSION_ERROR;
    }

  ENCODER_INF(AS_ATTENTION_SUB_CODE_DSP_LOAD_DONE);

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  memset(&m_debug_log_info, 0, sizeof(m_debug_log_info));
#endif

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool EncoderComponent::deactivate_apu(void)
{
  ENCODER_DBG("DEACT:\n");

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  if (m_debug_log_info.addr)
    {
      up_log_free(m_debug_log_info.name);
    }
#endif

  if (DD_Unload(m_dsp_handler) != 0)
    {
      ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_ERROR);
      return false;
    }
  ENCODER_INF(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_DONE);

  return true;
}

/*--------------------------------------------------------------------*/
uint32_t EncoderComponent::init_apu(const InitEncParam& param, uint32_t *dsp_inf)
{
  ENCODER_DBG("INIT: codec %d, infs %d, outfs %d, bit len %d, ch num %d, complexity %d, bit rate %d, cb %08x\n",
              param.codec_type, param.input_sampling_rate, param.output_sampling_rate, param.bit_width,
              param.channel_num, param.complexity, param.bit_rate, param.callback);

  m_callback = param.callback;

  Apu::Wien2ApuCmd* p_apu_cmd = static_cast<Apu::Wien2ApuCmd*>(getApuCmdBuf());

  if (p_apu_cmd == NULL)
    {
      return AS_ECODE_ENCODER_LIB_INITIALIZE_ERROR;
    }

  memset(p_apu_cmd, 0x00, sizeof(Apu::Wien2ApuCmd));

  p_apu_cmd->header.process_mode = Apu::EncMode;
  p_apu_cmd->header.event_type   = Apu::InitEvent;

  /* When MP3ENC, channel_config and bit_rate is not referenced */

  p_apu_cmd->init_enc_cmd.codec_type          = param.codec_type;
  p_apu_cmd->init_enc_cmd.channel_num         = param.channel_num;
  p_apu_cmd->init_enc_cmd.input_sampling_rate = param.input_sampling_rate;

  if (param.channel_num == 2)
    {
      p_apu_cmd->init_enc_cmd.channel_config = AUD_PCM_CH_CONFIG_2_0;
    }
  else
    {
      p_apu_cmd->init_enc_cmd.channel_config = AUD_PCM_CH_CONFIG_1_0;
    }

  p_apu_cmd->init_enc_cmd.bit_length           = param.bit_width;
  p_apu_cmd->init_enc_cmd.output_sampling_rate = param.output_sampling_rate;

  if (param.codec_type == AudCodecMP3)
    {
      p_apu_cmd->init_enc_cmd.bit_rate = param.bit_rate;
    }

  if (param.codec_type == AudCodecSBC)
    {
      p_apu_cmd->init_enc_cmd.init_sbc_enc_param.block_len   = 16;
      p_apu_cmd->init_enc_cmd.init_sbc_enc_param.subband_num = 8;
      p_apu_cmd->init_enc_cmd.init_sbc_enc_param.enc_type    = 1; /* type SNR */
    }

  if (param.codec_type == AudCodecOPUS)
    {
      p_apu_cmd->init_enc_cmd.bit_rate                                = param.bit_rate;
      p_apu_cmd->init_enc_cmd.init_opus_enc_param.complexity          = param.complexity;
#ifdef CONFIG_AUDIOUTILS_UNUSE_ORIGINAL_OPUS_FORMAT
      p_apu_cmd->init_enc_cmd.init_opus_enc_param.use_original_format = false;
#else
      p_apu_cmd->init_enc_cmd.init_opus_enc_param.use_original_format = true;
#endif /* CONFIG_AUDIOUTILS_UNUSE_ORIGINAL_OPUS_FORMAT */
    }

  p_apu_cmd->init_enc_cmd.debug_dump_info.addr = NULL;
  p_apu_cmd->init_enc_cmd.debug_dump_info.size = 0;

#ifdef CONFIG_AUDIOUTILS_DSP_DEBUG_DUMP
  /* initialization of DSP debug dump */

  if (m_debug_log_info.addr == NULL)
    {
      if (param.codec_type == AudCodecMP3)
        {
          strncpy(m_debug_log_info.name, "MP3ENC", sizeof(m_debug_log_info.name));
        }
      else
        {
          strncpy(m_debug_log_info.name, "OPUSENC", sizeof(m_debug_log_info.name));
        }

      m_debug_log_info.addr = up_log_alloc(m_debug_log_info.name,
                                           AUDIOUTILS_DSP_DEBUG_DUMP_SIZE);

      if (m_debug_log_info.addr == NULL)
        {
          ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_LOG_ALLOC_ERROR);
          return AS_ATTENTION_SUB_CODE_DSP_LOG_ALLOC_ERROR;
        }
    }

  if (m_debug_log_info.addr != NULL)
    {
      p_apu_cmd->init_enc_cmd.debug_dump_info.addr = m_debug_log_info.addr;
      p_apu_cmd->init_enc_cmd.debug_dump_info.size = AUDIOUTILS_DSP_DEBUG_DUMP_SIZE;
    }
  else
    {
      memset(m_debug_log_info.name, 0, sizeof(m_debug_log_info.name));
    }
#endif

  send_apu(p_apu_cmd);

  uint32_t rst = dsp_init_check(m_apu_dtq, dsp_inf);
  return rst;
}

/*--------------------------------------------------------------------*/
bool EncoderComponent::exec_apu(const ExecEncParam& param)
{
  Apu::Wien2ApuCmd* p_apu_cmd = static_cast<Apu::Wien2ApuCmd*>(getApuCmdBuf());

  if (p_apu_cmd == NULL)
    {
      return false;
    }

  memset(p_apu_cmd, 0x00, sizeof(Apu::Wien2ApuCmd));

  p_apu_cmd->header.process_mode = Apu::EncMode;
  p_apu_cmd->header.event_type   = Apu::ExecEvent;

  p_apu_cmd->exec_enc_cmd.input_buffer  = param.input_buffer;
  p_apu_cmd->exec_enc_cmd.output_buffer = param.output_buffer;

  send_apu(p_apu_cmd);

  return true;
}

/*--------------------------------------------------------------------*/
bool EncoderComponent::flush_apu(const StopEncParam& param)
{
  Apu::Wien2ApuCmd* p_apu_cmd = static_cast<Apu::Wien2ApuCmd*>(getApuCmdBuf());

  if (p_apu_cmd == NULL)
    {
      return false;
    }

  memset(p_apu_cmd, 0x00, sizeof(Apu::Wien2ApuCmd));

  p_apu_cmd->header.process_mode = Apu::EncMode;
  p_apu_cmd->header.event_type   = Apu::FlushEvent;

  p_apu_cmd->flush_enc_cmd.output_buffer = param.output_buffer;

  send_apu(p_apu_cmd);

  return true;
}

/*--------------------------------------------------------------------*/
bool EncoderComponent::recv_apu(void *p_response)
{
  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;
  Apu::Wien2ApuCmd *packet = static_cast<Apu::Wien2ApuCmd*>(p_param->data.pParam);

  if (Apu::ApuExecOK != packet->result.exec_result)
    {
      ENCODER_WARN(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
    }

  if (Apu::InitEvent == packet->header.event_type)
    {
      dsp_init_complete(m_apu_dtq, packet);
      return true;
    }

  return m_callback(p_param);
}

/*--------------------------------------------------------------------*/
void EncoderComponent::send_apu(Apu::Wien2ApuCmd* p_cmd)
{
  DspDrvComPrm_t com_param;
  com_param.event_type = p_cmd->header.event_type;
  com_param.process_mode = p_cmd->header.process_mode;
  com_param.type = DSP_COM_DATA_TYPE_STRUCT_ADDRESS;
  com_param.data.pParam = reinterpret_cast<void*>(p_cmd);

  DD_SendCommand(m_dsp_handler, &com_param);
}

__WIEN2_END_NAMESPACE

