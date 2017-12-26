/****************************************************************************
 * examples/streaming/streaming_lte.c
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
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

#include "lte/lte_if.h"
#include "lte/lte_if_system.h"
#include "app_lte.h"
#include "app_audio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
static int com_status = 0;
static int com_id = 0;

static LTECommand command;
static LTECommand_System command_sys;
static LTECommand_System response_sys;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void app_callback_sys(void* out)
{
  LTECommand_System *data = (LTECommand_System *)out;

  com_status = data->header.result;
  printf("Response[%04X][%d][%d][%04X]\n",
         data->header.code,
         data->header.id,
         data->header.result,
         data->header.type);

  switch (data->header.type)
    {
      case LTE_STRUCT_TYPE_MESSAGE:
        printf("data:%s\n", data->message.text.data);
        break;

      case LTE_STRUCT_TYPE_ERROR:
        printf("[ERROR]:%s(code:%d)\n",
               data->message.error.group, data->message.error.code);
        break;

      default:
        if (data->header.result == 0)
          {
            printf("success\n");
          }
        else
          {
            printf("fail\n");
          }
        break;
    }
}


static int wait_recv(void)
{
  int count;

  /* Wait receive */
  for (count=0; count<3000; count++)
    {
      if (com_status != 1)
        {
          return com_status;
        }
      usleep(10*000); /* 10msec sleep */
    }

  /* Timeout */
  return 0;
}


static int send_recv_sys(LTECommand_System* cmd)
{
  /* Initialize status */
  com_status = 1;

  /* Send and return code check */
  printf("LT_SendSystem : %04X(%d)\n", cmd->header.code, cmd->header.id);
  LT_SendSystem(cmd);
  if (cmd->header.result < 0)
    {
      printf("LT_SendSystem error : %d\n", cmd->header.result);
      return cmd->header.result;
    }

  /* Wait receive */
  return wait_recv();
}


static int app_streaming_lte_connect(void)
{
  /* Send system connect command */
  command_sys.header.code = LTE_COMMAND_SYSTEM_CONNECT;
  command_sys.header.id   = com_id++;
  command_sys.header.type = LTE_STRUCT_TYPE_CONFIG;
  command_sys.message.config.IMSregist = 0;
  return send_recv_sys(&command_sys);
}

int app_streaming_lte_init(void)
{
  while (1)
    {
      /* Initialize task */
      command.debug_level = LTE_DEBUG_INFO;
      LT_Start(&command);
      if (command.result == 0)
        {
          break;
        }
      printf("LT_Start() error : %d\n", command.result);

      /* Delay for link detection */
      sleep(3);
    }

  printf("LT_Start() done\n");

  /* Set callback */
  LT_SetCallbackSystem(&app_callback_sys, &response_sys);

  sleep(3);

  if(app_streaming_lte_connect() != 0)
    {
      printf("LTE connection establishment failed\n");
      return 1;
    }

  printf("LTE connection establishment done\n");

  app_streaming_audio_ctrl(D_APP_START_BEEP);
  sleep(1);
  app_streaming_audio_ctrl(D_APP_STOP_BEEP);

  return 0;
}

