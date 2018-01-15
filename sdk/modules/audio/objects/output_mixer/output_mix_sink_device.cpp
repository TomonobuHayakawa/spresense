/****************************************************************************
 * modules/audio/objects/output_mixer/output_mix_sink_device.cpp
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/chip/cxd56_audio.h>
#include "output_mix_sink_device.h"
#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BYTE_SIZE_PER_SAMPLE_HIGHRES 8    /* Number of bytes per sample that
                                           * case of high res.
                                           * 8 == (sizeof(int32_t)x2ch)
                                           */
#define BYTE_SIZE_PER_SAMPLE         4    /* Number of bytes per sample
                                           * 4 == (sizeof(int16_t)x2ch)
                                           */
#define DMA_MIN_SAMPLE               240  /* DMA minimum Samples. */
#define DMA_MAX_SAMPLE               1024 /* DMA maximum Samples. */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void send_renderer(RenderComponentHandler handle,
                          void *p_addr,
                          uint32_t byte_size,
                          int8_t adjust,
                          bool is_valid);
static bool check_sample(OutputMixObjInputDataCmd* input);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void render_done_callback(FAR AudioDrvDmaResult *p_param,
                                 void* p_requester)
{
  err_t er;
  OutputMixObjParam outmix_param;
  outmix_param.handle =
    (static_cast<OutputMixToHPI2S*>(p_requester))->m_self_handle;
  outmix_param.renderdone_param.end_flag = p_param->endflg;

  er = MsgLib::send<OutputMixObjParam>((static_cast<OutputMixToHPI2S*>
                                        (p_requester))->m_self_dtq,
                                       MsgPriNormal,
                                       MSG_AUD_MIX_CMD_RENDER_DONE,
                                       (static_cast<OutputMixToHPI2S*>
                                        (p_requester))->m_self_dtq,
                                       outmix_param);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------------*/
/* Methods of OutputMixToHPI2S class */
/*--------------------------------------------------------------------------*/
OutputMixToHPI2S::MsgProc OutputMixToHPI2S::MsgProcTbl[AUD_MIX_MSG_NUM][StateNum] =
{
/*            Booted                          Ready                                  Active                                  Stopping */
/* ACT   */ {&OutputMixToHPI2S::act,         &OutputMixToHPI2S::illegal,            &OutputMixToHPI2S::illegal,             &OutputMixToHPI2S::illegal},
/* DATA  */ {&OutputMixToHPI2S::illegal,     &OutputMixToHPI2S::input_data_on_ready,&OutputMixToHPI2S::input_data_on_active,&OutputMixToHPI2S::illegal},
/* STOP */  {&OutputMixToHPI2S::illegal,     &OutputMixToHPI2S::illegal,            &OutputMixToHPI2S::stop_on_active,      &OutputMixToHPI2S::illegal},
/* DEACT */ {&OutputMixToHPI2S::illegal,     &OutputMixToHPI2S::deact,              &OutputMixToHPI2S::illegal,             &OutputMixToHPI2S::illegal},
/* DONE */  {&OutputMixToHPI2S::illegal_done,&OutputMixToHPI2S::illegal_done,       &OutputMixToHPI2S::done_on_active,      &OutputMixToHPI2S::done_on_stopping},
/* ADJ */   {&OutputMixToHPI2S::illegal,     &OutputMixToHPI2S::clock_recovery,     &OutputMixToHPI2S::clock_recovery,      &OutputMixToHPI2S::clock_recovery},
};

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::parse(MsgPacket* msg)
{
  uint event = MSG_GET_SUBTYPE(msg->getType());
  F_ASSERT(event < AUD_MIX_MSG_NUM);

  (this->*MsgProcTbl[event][m_state.get()])(msg);
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::illegal(MsgPacket* msg)
{
  OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST);
  return;
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::act(MsgPacket* msg)
{
  OutputMixDoneParam done_param;
  err_t er;
  m_player_dtq = msg->getReply();
  OutputMixObjActCmd act_output_mix_param =
    msg->moveParam<OutputMixObjParam>().act_param;

  OUTPUT_MIX_DBG("ACT: dev %d, type %d\n",
                 act_output_mix_param.output_device,
                 act_output_mix_param.mixer_type);

  switch(act_output_mix_param.mixer_type)
    {
      case MainOnly:
        /* Create render component for rendering main sound. */

        {
          RenderDevice render_device;
          if (act_output_mix_param.output_device == HPOutputDevice)
            {
              render_device = RenderDeviceHPSP;
            }
          else
            {
              render_device = RenderDeviceI2S;
            }
          if (!AS_get_render_comp_handler(&m_render_comp_handler,
                                          render_device))
            {
              return;
            }
        }
        break;

      case SoundEffectOnly:
        /* Create render component for rendering sound effect. */

      case MainSoundEffectMix:
        /* Create render components for rendering the mix of
         * main sound and sound effect.
         */

      default:
        /* Send error message. */

        return;
    }

  done_param.handle    = m_self_handle;
  done_param.done_type = OutputMixActDone;

  er = MsgLib::send<OutputMixDoneParam>(m_player_dtq,
                                        MsgPriNormal,
                                        MSG_AUD_PLY_CMD_OUTPUT_MIX_DONE,
                                        m_self_dtq,
                                        done_param);
  F_ASSERT(er == ERR_OK);
  m_state = Ready;
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::deact(MsgPacket* msg)
{
  err_t er;
  OutputMixDoneParam done_param;
  msg->moveParam<OutputMixObjParam>();

  OUTPUT_MIX_DBG("DEACT:\n");

  if (!AS_release_render_comp_handler(m_render_comp_handler))
    {
      return;
    }

  done_param.handle    = m_self_handle;
  done_param.done_type = OutputMixDeactDone;

  er = MsgLib::send<OutputMixDoneParam>(m_player_dtq,
                                        MsgPriNormal,
                                        MSG_AUD_PLY_CMD_OUTPUT_MIX_DONE,
                                        m_self_dtq,
                                        done_param);
  F_ASSERT(er == ERR_OK);
  m_self_handle = 0;
  m_state       = Booted;
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::input_data_on_ready(MsgPacket* msg)
{
  OutputMixObjInputDataCmd input =
    msg->moveParam<OutputMixObjInputDataCmd>();

  /* Keep the resource until render done has been received. */

  if (!m_render_data_queue.push(input))
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return;
    }

  if (!AS_init_renderer(m_render_comp_handler,
                        &render_done_callback,
                        static_cast<void*>(this)))
    {
      return;
    }

  send_renderer(m_render_comp_handler,
                input.mh.getPa(),
                input.size,
                get_period_adjustment(),
                input.is_valid);

  m_state = Active;
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::input_data_on_active(MsgPacket* msg)
{
  OutputMixObjInputDataCmd input =
    msg->moveParam<OutputMixObjInputDataCmd>();

  if(check_sample(&input))
    {
      /* Keep the resource until render done has been received. */

      if (!m_render_data_queue.push(input))
        {
          OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
          return;
        }

      send_renderer(m_render_comp_handler,
                  input.mh.getPa(),
                  input.size,
                  get_period_adjustment(),
                  input.is_valid);
    }
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::stop_on_active(MsgPacket* msg)
{
  OUTPUT_MIX_DBG("STOP:\n");

  msg->moveParam<OutputMixObjParam>();

  if (!AS_stop_renderer(m_render_comp_handler, AS_DMASTOPMODE_NORMAL))
    {
      return;
    }

  m_state = Stopping;
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::illegal_done(MsgPacket* msg)
{
  msg->moveParam<OutputMixObjParam>();
  return;
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::done_on_active(MsgPacket* msg)
{
  err_t er;
  OutputMixDoneParam done_param;
  bool end_flag =
    msg->moveParam<OutputMixObjParam>().renderdone_param.end_flag;
  if (end_flag)
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_INTERNAL_STATE_ERROR);
      return;
    }

  if (m_render_data_queue.empty())
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
      return;
    }
  if (!m_render_data_queue.pop())
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
      return;
    }

  done_param.handle    = m_self_handle;
  done_param.done_type = OutputMixExecDone;

  er = MsgLib::send<OutputMixDoneParam>(m_player_dtq,
                                        MsgPriNormal,
                                        MSG_AUD_PLY_CMD_OUTPUT_MIX_DONE,
                                        m_self_dtq,
                                        done_param);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::done_on_stopping(MsgPacket* msg)
{
  err_t er = ERR_OK;
  OutputMixDoneParam done_param;
  bool end_flag =
    msg->moveParam<OutputMixObjParam>().renderdone_param.end_flag;

  if (m_render_data_queue.empty())
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR);
      return;
    }
  if (!m_render_data_queue.pop())
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
      return;
    }

  done_param.handle    = m_self_handle;

  if (m_render_data_queue.empty())
    {
      if (end_flag == false)
        {
          OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          done_param.done_type = OutputMixExecDone;
          er = MsgLib::send<OutputMixDoneParam>(m_player_dtq,
                                                MsgPriNormal,
                                                MSG_AUD_PLY_CMD_OUTPUT_MIX_DONE,
                                                m_self_dtq, done_param);
          F_ASSERT(er == ERR_OK);
        }
      else
        {
          done_param.done_type = OutputMixStopDone;
          er = MsgLib::send<OutputMixDoneParam>(m_player_dtq,
                                                MsgPriNormal,
                                                MSG_AUD_PLY_CMD_OUTPUT_MIX_DONE,
                                                m_self_dtq,
                                                done_param);
          F_ASSERT(er == ERR_OK);
          m_state = Ready;
        }
    }
  else
    {
      done_param.done_type = OutputMixExecDone;
      er = MsgLib::send<OutputMixDoneParam>(m_player_dtq,
                                            MsgPriNormal,
                                            MSG_AUD_PLY_CMD_OUTPUT_MIX_DONE,
                                            m_self_dtq,
                                            done_param);
      F_ASSERT(er == ERR_OK);
    }
}

/*--------------------------------------------------------------------------*/
void OutputMixToHPI2S::clock_recovery(MsgPacket* msg)
{
  OutputMixObjSoundPeriodAdjust cmd =
    msg->moveParam<OutputMixObjParam>().adjust_param;

  OUTPUT_MIX_DBG("CLOCK RECOVERY: dir %d, times %d\n",
                 cmd.direction, cmd.adjust_times);

  /* Check Paramete. */

  if (cmd.direction < OutputMixAdvance || OutputMixDelay < cmd.direction)
    {
      return;
    }

  /* Set recovery parameters. */

  m_adjust_direction = cmd.direction;
  m_adjustment_times = cmd.adjust_times;

  return;
}

/*--------------------------------------------------------------------------*/
int8_t OutputMixToHPI2S::get_period_adjustment(void)
{
  bool do_adjust = false;
  int8_t adjust_sample = 0;

  /* Determine adjust or not. */

  if (m_adjustment_times > 0)
    {
      do_adjust = true;
      m_adjustment_times--;
    }
  else if (m_adjustment_times < 0)
    {
      do_adjust = true;
    }
  else
    {
      do_adjust = false;
    }

  /* Set adjust samples. */

  if (do_adjust)
    {
      if (m_adjust_direction > 0)
        {
          adjust_sample = 1;
        }
      else if (m_adjust_direction < 0)
        {
          adjust_sample = -1;
        }
      else
        {
          /* No adjust work. */
        }
    }

  return adjust_sample;
}

/*--------------------------------------------------------------------------*/
static void send_renderer(RenderComponentHandler handle,
                          void *p_addr,
                          uint32_t byte_size,
                          int8_t adjust,
                          bool is_valid)
{
  uint32_t byte_size_per_sample = 0;
  AsClkModeId clock_mode = GetClkMode();
  if (AS_CLK_MODE_HIRES == clock_mode)
    {
      byte_size_per_sample = BYTE_SIZE_PER_SAMPLE_HIGHRES;
    }
  else
    {
      byte_size_per_sample = BYTE_SIZE_PER_SAMPLE;
    }

  /* Insert dummy data. */

  if (adjust > 0)
    {
      memcpy((char*)p_addr + byte_size,
             (char*)p_addr + byte_size - (byte_size_per_sample * adjust),
             byte_size_per_sample * adjust);
    }

  /* Adjust sound period.
   * (Adjust transfer samples of DMA out for clock recovery.)
   */

  byte_size += (adjust * byte_size_per_sample);

  /* Send to renderer. */

  if (!AS_exec_renderer(handle,
                        p_addr,
                        byte_size / byte_size_per_sample,
                        is_valid))
    {
      return;
    }
}

/*--------------------------------------------------------------------------*/
static bool check_sample(OutputMixObjInputDataCmd *input)
{
  bool res = true;
  if (input->is_es_end == true)
    {
      if (input->size < DMA_MIN_SAMPLE*BYTE_SIZE_PER_SAMPLE)
        {
          res = false;
        }
    }
  else
    {
      if (input->size < DMA_MIN_SAMPLE*BYTE_SIZE_PER_SAMPLE)
        {
          input->size = DMA_MIN_SAMPLE*BYTE_SIZE_PER_SAMPLE;
        }
    }
  return res;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

__WIEN2_END_NAMESPACE

