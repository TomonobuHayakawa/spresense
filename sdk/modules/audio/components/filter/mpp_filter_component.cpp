/***********************************************************************
 *
 *      File Name: mpp_filter_component.cpp
 *
 *      Description: MFE filter component
 *
 *      Notes: (C) Copyright 2016 Sony Corporation
 *
 *      Author: Hsingying Ho
 *
 ***********************************************************************
 */

#include "components/filter/mpp_filter_component.h"
#include "apus/cpuif_cmd.h"

#include "memutils/message/Message.h"
#include "memutils/message/MsgPacket.h"
#include "common/audio_message_types.h"
#include "apus/apu_context_ids.h"
#include "debug/dbg_log.h"
#include "apus/dsp_audio_version.h"

#define DBG_MODULE DBG_MODULE_AS

__WIEN2_BEGIN_NAMESPACE

#if defined(ENABLE_FLASH_BOOT)
#define SEMAPHORE_WAIT_TIME 3000
#else
#define SEMAPHORE_WAIT_TIME 30000
#endif /* ENABLE_FLASH_BOOT */

static MPPComponent *sp_mpp_component = NULL;

#define DSP_CORE_ID_DUMMY 3 /* dummy */

/*--------------------------------------------------------------------*/
void mpp_filter_dsp_done_callback(void *p_response, void *p_instance)
{
  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;
  Apu::Wien2ApuCmd *packet =
    reinterpret_cast<Apu::Wien2ApuCmd*>(p_param->data.pParam);

  if (sp_mpp_component == NULL)
  {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
  };

  switch (p_param->process_mode)
    {
      case Apu::CommonMode:
        switch (p_param->event_type)
          {
            case Apu::BootEvent:
              {
                err_t er = MsgLib::send<uint32_t>
                        (((MPPComponent*)p_instance)->get_apu_mid(),
                         MsgPriNormal,
                         MSG_ISR_APU0,
                         0,
                         p_param->data.value);

                F_ASSERT(er == ERR_OK);
              }
              break;

            case Apu::ErrorEvent:
                FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_ASSETION_FAIL);
                break;

            default:
                break;
          }
        break;

      case Apu::FilterMode:
        switch (packet->init_filter_cmd.filter_type)
          {
            case Apu::XLOUD:
                if (!sp_mpp_component->recv_apu(p_param))
                  {
                    F_ASSERT(0);
                  }
                break;

            default:
                FILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
                break;
          }
        break;

      default:
        FILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
        F_ASSERT(0);
        break;
    }
}

/*--------------------------------------------------------------------*/
/* Methods of MPPComponent class */
/*--------------------------------------------------------------------*/
uint32_t MPPComponent::activate_apu(MPPComponent *p_component,
                                    uint32_t *dsp_inf)
{
  FILTER_DBG("ACT MPP:\n");

  sp_mpp_component = p_component;

#ifdef ENABLE_FLASH_BOOT
  /* TODO: To be able to configure DSP imange file name by Config menu */

  if (NULL == (m_dsp_handler = DD_Load("MPPEAX",
                                       mpp_filter_dsp_done_callback,
                                       (void *)this)))
    {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_LOAD_ERROR);
      return AS_RESPONSE_CODE_DSP_LOAD_ERROR;
    }
#endif

  /* wait for DSP boot up... */

  if (!dsp_boot_check(m_apu_dtq, DSP_MPPEAX_VERSION, dsp_inf))
    {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_VERSION_ERROR);
      if (DD_Unload(m_dsp_handler) != 0)
        {
          FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_ERROR);
        }

      return AS_RESPONSE_CODE_DSP_VERSION_ERROR;
    }

  FILTER_INF(AS_ATTENTION_SUB_CODE_DSP_LOAD_DONE);

  return AS_RESPONSE_CODE_OK;
}

/*--------------------------------------------------------------------*/
bool MPPComponent::deactivate_apu(void)
{
  FILTER_DBG("DEACT MPP:\n");

#ifdef ENABLE_FLASH_BOOT
  if (DD_Unload(m_dsp_handler) != 0)
    {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_ERROR);
      return false;
    }
#endif

  FILTER_INF(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_DONE);
  return true;
}

/*--------------------------------------------------------------------*/
uint32_t MPPComponent::init_apu(InitXLOUDParam param, uint32_t* dsp_inf)
{
  FILTER_DBG("INIT MPP: ch num %d, sample num %d, infs %d, mode %d, "
             "bit len <in %d/out %d>, xloud coef <tbl %08x/size %d>, "
             "eax coef <tbl %08x/size %d>, sel out %08x\n",
             param.channel_num, param.sample, param.input_sampling_rate,
             param.mode, param.in_pcm_bit_len, param.out_pcm_bit_len,
             param.p_xloud_coef_image, param.xloud_coef_size,
             param.p_eax_coef_image, param.eax_coef_size,
             param.p_sel_out_param);

  m_exec_queue.clear();
  m_buf_idx = 0;

  Apu::Wien2ApuCmd *p_apu_cmd = &m_apu_cmd_buf[m_buf_idx];

  p_apu_cmd->header.core_id      = DSP_CORE_ID_DUMMY;
  p_apu_cmd->header.context_id   = DSP_MPPEAX_CONTEXT_MPPEAX;
  p_apu_cmd->header.process_mode = Apu::FilterMode;
  p_apu_cmd->header.event_type   = Apu::InitEvent;

  p_apu_cmd->init_filter_cmd.filter_type = Apu::XLOUD;

  p_apu_cmd->init_filter_cmd.channel_num = param.channel_num;
  p_apu_cmd->init_filter_cmd.sample      = param.sample;

  p_apu_cmd->init_filter_cmd.init_xloud_param.input_sampling_rate =
    param.input_sampling_rate;

  p_apu_cmd->init_filter_cmd.init_xloud_param.mode =
    param.mode;

  p_apu_cmd->init_filter_cmd.init_xloud_param.in_pcm_bit_len =
    param.in_pcm_bit_len;

  p_apu_cmd->init_filter_cmd.init_xloud_param.out_pcm_bit_len =
    param.out_pcm_bit_len;

  p_apu_cmd->init_filter_cmd.init_xloud_param.p_coef_image =
    reinterpret_cast<uint8_t*>(param.p_xloud_coef_image);

  p_apu_cmd->init_filter_cmd.init_xloud_param.coef_size =
    param.xloud_coef_size;

  p_apu_cmd->init_filter_cmd.init_xloud_param.eax_input_sampling_rate =
    AudioFs2ApuValue[AudFs_16000];

  p_apu_cmd->init_filter_cmd.init_xloud_param.eax_mic_channel_num =
    AudioIOValidChannelNum[TwoChannels];

  p_apu_cmd->init_filter_cmd.init_xloud_param.eax_sample = 80;

  p_apu_cmd->init_filter_cmd.init_xloud_param.eax_mode =
    static_cast<Apu::AudioEaxMode>(param.mode);

  p_apu_cmd->init_filter_cmd.init_xloud_param.eax_enable_external_analysis =
    false;

  p_apu_cmd->init_filter_cmd.init_xloud_param.eax_in_pcm_bit_len =
    Apu::AudPcmFormatInt16;

  p_apu_cmd->init_filter_cmd.init_xloud_param.eax_out_pcm_bit_len =
    Apu::AudPcmFormatInt16;

  p_apu_cmd->init_filter_cmd.init_xloud_param.p_eax_coef_image =
    reinterpret_cast<uint8_t*>(param.p_eax_coef_image);

  p_apu_cmd->init_filter_cmd.init_xloud_param.eax_coef_size =
    param.eax_coef_size;

  p_apu_cmd->init_filter_cmd.init_xloud_param.p_dma_info = NULL;

  p_apu_cmd->init_filter_cmd.debug_dump_info.addr = NULL;
  p_apu_cmd->init_filter_cmd.debug_dump_info.size = 0;

  send_apu(m_apu_cmd_buf[m_buf_idx]);
  m_buf_idx = (m_buf_idx + 1) % APU_COMMAND_QUEUE_SIZE;

  uint32_t rst = dsp_init_check(m_apu_dtq, dsp_inf);

  return rst;
}

/*--------------------------------------------------------------------*/
bool MPPComponent::exec_apu(ExecXLOUDParam param)
{
  if (m_buf_idx >= APU_COMMAND_QUEUE_SIZE)
    {
      return false;
    }

  m_apu_cmd_buf[m_buf_idx].header.core_id      = DSP_CORE_ID_DUMMY;
  m_apu_cmd_buf[m_buf_idx].header.context_id   = DSP_MPPEAX_CONTEXT_MPPEAX;
  m_apu_cmd_buf[m_buf_idx].header.process_mode = Apu::FilterMode;
  m_apu_cmd_buf[m_buf_idx].header.event_type   = Apu::ExecEvent;

  m_apu_cmd_buf[m_buf_idx].exec_filter_cmd.filter_type   = Apu::XLOUD;
  m_apu_cmd_buf[m_buf_idx].exec_filter_cmd.input_buffer  = param.input_buffer;
  m_apu_cmd_buf[m_buf_idx].exec_filter_cmd.output_buffer =
    param.output_buffer;

  /* TODO: tetative, fixed value operation */
  m_apu_cmd_buf[m_buf_idx].exec_filter_cmd.exec_xloud_cmd.level = 30;
  m_apu_cmd_buf[m_buf_idx].exec_filter_cmd.exec_xloud_cmd.headroom_in = 0;
  m_apu_cmd_buf[m_buf_idx].exec_filter_cmd.exec_xloud_cmd.headroom_out = 0;
  m_apu_cmd_buf[m_buf_idx].exec_filter_cmd.exec_xloud_cmd.enable_mute = false;
  m_apu_cmd_buf[m_buf_idx].exec_filter_cmd.exec_xloud_cmd.enable_hybrid_gain_mode =
    true;
  m_apu_cmd_buf[m_buf_idx].exec_filter_cmd.exec_xloud_cmd.eax_level = 30;

  send_apu(m_apu_cmd_buf[m_buf_idx]);
  m_buf_idx = (m_buf_idx + 1) % APU_COMMAND_QUEUE_SIZE;

  return true;
}

/*--------------------------------------------------------------------*/
bool MPPComponent::flush_apu()
{
  if (m_buf_idx >= APU_COMMAND_QUEUE_SIZE)
    {
      return false;
    }

  m_apu_cmd_buf[m_buf_idx].header.core_id      = DSP_CORE_ID_DUMMY;
  m_apu_cmd_buf[m_buf_idx].header.context_id   = DSP_MPPEAX_CONTEXT_MPPEAX;
  m_apu_cmd_buf[m_buf_idx].header.process_mode = Apu::FilterMode;
  m_apu_cmd_buf[m_buf_idx].header.event_type   = Apu::FlushEvent;

  m_apu_cmd_buf[m_buf_idx].flush_filter_cmd.filter_type = Apu::XLOUD;

  send_apu(m_apu_cmd_buf[m_buf_idx]);
  m_buf_idx = (m_buf_idx + 1) % APU_COMMAND_QUEUE_SIZE;

  return true;
}

/*--------------------------------------------------------------------*/
bool MPPComponent::setparam_apu(Apu::ApuSetParamFilterCmd param)
{
  FILTER_DBG("SET MPP: filter type %d, param idx %d, xloud vol %d\n",
             param.filter_type, param.set_mpp.param_idx,
             param.set_mpp.xloud_vol);

  if (m_buf_idx >= APU_COMMAND_QUEUE_SIZE)
    {
      return false;
    }

  m_apu_cmd_buf[m_buf_idx].header.core_id      = DSP_CORE_ID_DUMMY;
  m_apu_cmd_buf[m_buf_idx].header.context_id   = DSP_MPPEAX_CONTEXT_MPPEAX;
  m_apu_cmd_buf[m_buf_idx].header.process_mode = Apu::FilterMode;
  m_apu_cmd_buf[m_buf_idx].header.event_type   = Apu::SetParamEvent;

  m_apu_cmd_buf[m_buf_idx].setparam_filter_cmd.filter_type       = Apu::XLOUD;
  m_apu_cmd_buf[m_buf_idx].setparam_filter_cmd.set_mpp.param_idx =
    param.set_mpp.param_idx;

  if (param.set_mpp.param_idx == Apu::AudXloudSetIndividual)
    {
      m_apu_cmd_buf[m_buf_idx].setparam_filter_cmd.set_mpp.xloud_vol =
        param.set_mpp.xloud_vol;
    }

  send_apu(m_apu_cmd_buf[m_buf_idx]);
  m_buf_idx = (m_buf_idx + 1) % APU_COMMAND_QUEUE_SIZE;

  return true;
}

/*--------------------------------------------------------------------*/
bool MPPComponent::tuning_apu(Apu::ApuTuningFilterCmd param)
{
  FILTER_DBG("TUNING MPP: filter type %d, param idx %d, "
             "xloud tbl <conf %08x/param %08x>, "
             "eax tbl <conf %08x/param %08x>\n",
             param.filter_type, param.tuning_mpp.param_idx,
             param.tuning_mpp.tuning_xloud.xloud_config_table,
             param.tuning_mpp.tuning_xloud.xloud_param_table,
             param.tuning_mpp.tuning_xloud.eax_config_table,
             param.tuning_mpp.tuning_xloud.eax_param_table);

  if (m_buf_idx >= APU_COMMAND_QUEUE_SIZE)
    {
      return false;
    }

  Apu::Wien2ApuCmd *apu_cmd = &m_apu_cmd_buf[m_buf_idx];

  apu_cmd->header.core_id      = DSP_CORE_ID_DUMMY;
  apu_cmd->header.context_id   = DSP_MPPEAX_CONTEXT_MPPEAX;
  apu_cmd->header.process_mode = Apu::FilterMode;
  apu_cmd->header.event_type   = Apu::TuningEvent;

  apu_cmd->tuning_filter_cmd.filter_type          = Apu::XLOUD;
  apu_cmd->tuning_filter_cmd.tuning_mpp.param_idx = param.tuning_mpp.param_idx;

  if (param.tuning_mpp.param_idx == Apu::AudXloudSetIndividual)
    {
      apu_cmd->tuning_filter_cmd.tuning_mpp.tuning_xloud.xloud_config_table =
        param.tuning_mpp.tuning_xloud.xloud_config_table;

      apu_cmd->tuning_filter_cmd.tuning_mpp.tuning_xloud.xloud_param_table =
        param.tuning_mpp.tuning_xloud.xloud_param_table;

      apu_cmd->tuning_filter_cmd.tuning_mpp.tuning_xloud.eax_config_table =
        param.tuning_mpp.tuning_xloud.eax_config_table;

      apu_cmd->tuning_filter_cmd.tuning_mpp.tuning_xloud.eax_param_table =
        param.tuning_mpp.tuning_xloud.eax_param_table;
    }

  send_apu(m_apu_cmd_buf[m_buf_idx]);
  m_buf_idx = (m_buf_idx + 1) % APU_COMMAND_QUEUE_SIZE;

  return true;
}

/*--------------------------------------------------------------------*/
bool MPPComponent::recv_apu(DspDrvComPrm_t *p_param)
{
  Apu::Wien2ApuCmd* packet =
    reinterpret_cast<Apu::Wien2ApuCmd*>(p_param->data.pParam);

  if (Apu::ApuExecOK != packet->result.exec_result)
    {
      FILTER_WARN(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
    }

  if (Apu::InitEvent == packet->header.event_type)
    {
      dsp_init_complete(m_apu_dtq, packet);
    }
  else
    {
      if(!m_callback(p_param))
        {
          FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
          return false;
        }
    }

  if (!m_exec_queue.pop())
    {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------*/
void MPPComponent::send_apu(Apu::Wien2ApuCmd& cmd)
{
  F_ASSERT(!m_exec_queue.full());

  if (!m_exec_queue.push(&cmd))
    {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return;
    }

  DspDrvComPrm_t com_param;

  com_param.process_mode  = cmd.header.process_mode;
  com_param.event_type    = cmd.header.event_type;
  com_param.type          = DSP_COM_DATA_TYPE_STRUCT_ADDRESS;
  com_param.data.pParam   = reinterpret_cast<void*>(&cmd);

  if (0 != DD_SendCommand(m_dsp_handler, &com_param))
    {
      FILTER_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
      return;
    }
}

__WIEN2_END_NAMESPACE

