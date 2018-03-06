/****************************************************************************
 * modules/audio/objects/output_mixer/output_mix_obj.cpp
 *
 *   Copyright (C) 2017 Sony Corporation
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

#include <string.h>
#include <stdlib.h>
#include <nuttx/arch.h>
#include "output_mix_obj.h"
#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE
using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pid_t    s_omix_pid = -1;
static MsgQueId s_self_dtq;
static MsgQueId s_manager_dtq;
static OutputMixObjectTask *s_omix_ojb = NULL;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

OutputMixObjectTask::OutputMixObjectTask(MsgQueId self_dtq,
                                         MsgQueId manager_dtq):
  m_self_dtq(self_dtq),
  m_output_device(HPOutputDevice)
  {
    for (int i = 0; i < HPI2SoutChNum; i++)
      {
        m_output_mix_to_hpi2s[i].set_self_dtq(self_dtq);
      }
  }

/*--------------------------------------------------------------------------*/
int OutputMixObjectTask::getHpI2sHandle(MsgPacket* msg)
{
  int idx;
  MsgType msgtype = msg->getType();

  if (msgtype == MSG_AUD_MIX_CMD_ACT)
    {
      for (idx = 0; idx < HPI2SoutChNum; idx++)
        {
          if (false == m_output_mix_to_hpi2s[idx].is_active())
            {
              m_output_mix_to_hpi2s[idx].set_self_handle(idx);
              break;
            }
        }

      if (idx >= HPI2SoutChNum)
        {
          OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
        }
    }
  else if (msgtype == MSG_AUD_MIX_CMD_DATA)
    {
      idx = msg->peekParam<OutputMixObjInputDataCmd>().handle;
    }
  else
    {
      idx = msg->peekParam<OutputMixObjParam>().handle;
    }

  return idx;
}

/*--------------------------------------------------------------------------*/
void OutputMixObjectTask::run()
{
  err_t        err_code;
  MsgQueBlock* que;
  MsgPacket*   msg;

  err_code = MsgLib::referMsgQueBlock(m_self_dtq, &que);
  F_ASSERT(err_code == ERR_OK);

  while (1)
    {
      err_code = que->recv(TIME_FOREVER, &msg);
      F_ASSERT(err_code == ERR_OK);

      parse(msg);

      err_code = que->pop();
      F_ASSERT(err_code == ERR_OK);
    }
}

/*--------------------------------------------------------------------------*/
void OutputMixObjectTask::parse(MsgPacket* msg)
{
  if (MSG_GET_CATEGORY(msg->getType()) == MSG_CAT_AUD_MIX)
    {
      if (msg->getType() == MSG_AUD_MIX_CMD_ACT)
        {
          m_output_device =
            msg->peekParam<OutputMixObjParam>().act_param.output_device;
        }
      switch(m_output_device)
        {
          case HPOutputDevice:
          case I2SOutputDevice:
            {
              int idx = getHpI2sHandle(msg);
              if (idx < HPI2SoutChNum)
                {
                  m_output_mix_to_hpi2s[idx].parse(msg);
                }
            }
            break;

          case A2dpSrcOutputDevice:
            break;

          default:
            OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
            break;
        }
    }
  else
    {
      F_ASSERT(0);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C"
{
/*--------------------------------------------------------------------------*/
int AS_OutputMixObjEntry(int argc, char *argv[])
{
  OutputMixObjectTask::create(s_self_dtq, s_manager_dtq);
  return 0;
}

/*--------------------------------------------------------------------------*/
bool AS_ActivateOutputMix(FAR AsActOutputMixParam_t *param)
{
  s_self_dtq    = param->msgq_id.mixer;
  s_manager_dtq = param->msgq_id.mng;

  s_omix_pid = task_create("OMIX_OBJ",
                           150, 1024 * 3,
                           AS_OutputMixObjEntry,
                           NULL);
  if (s_omix_pid < 0)
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeactivateOutputMix(void)
{
  if (s_omix_pid < 0)
    {
      return false;
    }

  task_delete(s_omix_pid);

  if (s_omix_ojb != NULL)
    {
      delete s_omix_ojb;
      s_omix_ojb = NULL;
    }
  return true;
}

} /* extern "C" */

/*--------------------------------------------------------------------------*/
void OutputMixObjectTask::create(MsgQueId self_dtq, MsgQueId manager_dtq)
{
  if (s_omix_ojb == NULL)
    {
      s_omix_ojb = new OutputMixObjectTask(self_dtq, manager_dtq);
      if (s_omix_ojb == NULL)
        {
          OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
          return;
        }
      s_omix_ojb->run();
    }
  else
    {
      OUTPUT_MIX_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
    }
}

__WIEN2_END_NAMESPACE

