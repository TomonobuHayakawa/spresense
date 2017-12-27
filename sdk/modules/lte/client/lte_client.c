/****************************************************************************
 * modules/lte/client/lte_client.c
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Yutaka Miyajima <Yutaka.Miyajima@sony.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
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
#include <string.h>
#include <stdlib.h>
#include <mqueue.h>
#include <fcntl.h>
#include <errno.h>

#include <lte/lte_if.h>
#include <lte/lte_if_system.h>
#include <lte/lte_if_util.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define COMMAND_ID_CONNECT     0x0001
#define COMMAND_ID_DISCONNECT  0x0002
#define COMMAND_ID_SLEEP       0x0003
#define COMMAND_ID_NUMBER      0x0004
#define LT_START_RETRY_OVER    30

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int lte_is_started = 0;
static int com_status = 0;
static LTECommand command;
static LTECommand_System command_sys;
static LTECommand_System response_sys;
static LTECommand_Util command_utl;
static LTECommand_Util response_utl;

static char g_number[LTE_SIZE_COMMAND_UTIL_NUMBER] = {0};

void lte_start(void);
void lte_stop(void);
void lte_connect(void);
void lte_disconnect(void);
void lte_sleep(void);
void lte_get_number(char* number);

static void app_callback_sys(void* out)
{
  LTECommand_System *data = (LTECommand_System *)out;

  com_status = data->header.result;

  switch (data->header.type)
    {
      case LTE_STRUCT_TYPE_MESSAGE:
        printf("data:%s\n", data->message.text.data);
        break;
      case LTE_STRUCT_TYPE_ERROR:
        printf("[ERROR]:%s(code:%d)\n", data->message.error.group, data->message.error.code);
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

static void app_callback_utl(void* out)
{
  LTECommand_Util *data = (LTECommand_Util *)out;

  com_status = data->header.result;
  printf("Response[%04X][%d][%d][%04X]\n", data->header.code, data->header.id, data->header.result, data->header.type);

  switch (data->header.type)
    {
      case LTE_STRUCT_TYPE_MESSAGE:
        printf("data:%s\n", data->message.text.data);
        break;
      case LTE_STRUCT_TYPE_ERROR:
        printf("[ERROR]:%s(code:%d)\n", data->message.error.group, data->message.error.code);
        break;
      case LTE_STRUCT_TYPE_UTIL_NUMBER:
        printf("number:%s(%d)\n", data->message.number.number, data->message.number.type);
        strncpy(g_number, data->message.number.number, sizeof(g_number));
        break;
      case LTE_STRUCT_TYPE_UTIL_QUALITY:
        if (data->message.quality.flag == 0)
          {
            printf("RSSI/BER:%d/%d\n", data->message.quality.rssi, data->message.quality.ber);
          }
        else
          {
            printf("RSSI/BER/RSRP/RSRQ/SINR/RSSNR/CQI:%d/%d/%d/%d/%d/%d/%d\n", data->message.quality.rssi, data->message.quality.ber, data->message.quality.rsrp, data->message.quality.rsrq, data->message.quality.sinr, data->message.quality.rssnr, data->message.quality.cqi);
          }
        break;
      case LTE_STRUCT_TYPE_UTIL_REGISTER:
        printf("reg:%d\narea:%d\ncell:%d\ntechnology:%d\nmode:%d\nformat:%d\noperate:%s\nims:%d\n", data->message.reg.reg, data->message.reg.area, data->message.reg.cell, data->message.reg.technology, data->message.reg.mode, data->message.reg.format, data->message.reg.operate, data->message.reg.ims);
        break;
      case LTE_STRUCT_TYPE_UTIL_CLOCK:
        printf("clock:%s\n", data->message.clock.time);
        break;
      case LTE_STRUCT_TYPE_UTIL_DATA:
        printf("disable_data:%s,disable_data_roaming:%d,time_sync:%d\n", data->message.data.disable_data,  data->message.data.disable_data_roaming, data->message.data.time_sync);
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
  for (count=0; count<1000; count++)
    {
      if (com_status != 1)
        {
          return com_status;
        }
      sleep(1);
    }

  /* Timeout */
  return 0;
}


static int send_recv_sys(LTECommand_System* cmd)
{
  /* Status init */
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

static int send_recv_utl(LTECommand_Util* cmd)
{
  /* Status init */
  com_status = 1;

  /* Send and return code check */
  printf("LT_SendUtil : %04X(%d)\n", cmd->header.code, cmd->header.id);
  LT_SendUtil(cmd);
  if (cmd->header.result < 0)
    {
      printf("LT_SendUtil error : %d\n", cmd->header.result);
      return cmd->header.result;
    }

  /* Wait receive */
  return wait_recv();
}

static int is_started(void)
{
  return lte_is_started;
}

void lte_start(void)
{
  int count;

  if (is_started())
    {
      return;
    }

  while(1)
    {
      for (count = 0; count < LT_START_RETRY_OVER; count++)
        {
          /* Boot the modem and setup internal network */

          command.debug_level = LTE_DEBUG_INFO;
          LT_Start(&command);
          if (command.result == 0)
            {
              break;
            }
          if (command.result != -EAGAIN)
            {
              printf("LT_Start() Error:%d\n", command.result);
              break;
            }

          /* Waiting for link detection and try again */

          if (count == 0)
            {
              printf("LT_Start() waiting .");
              fflush(stdout);
            }
          else
            {
              printf(".");
              fflush(stdout);
            }
          sleep(1);
        }
      printf("\n");

      if (command.result == 0)
        {
          break;
        }

      /* Shutdown the modem because unexpected time has passed. */

      lte_stop();
    }
  lte_is_started = 1;

  printf("LT_Start() done.\n");

  /* Set callback */

  LT_SetCallbackSystem(&app_callback_sys, &response_sys);
  LT_SetCallbackUtil(&app_callback_utl, &response_utl);
}

void lte_stop(void)
{
  LT_Stop(&command);
  if (command.result != 0)
    {
      printf("LT_Stop() Error:%d", command.result);
    }
  lte_is_started = 0;
}

void lte_connect(void)
{
  /* Send system connect command */
  command_sys.header.code = LTE_COMMAND_SYSTEM_CONNECT;
  command_sys.header.id = COMMAND_ID_CONNECT;
  send_recv_sys(&command_sys);

  printf("LTE Network is connected.\n");
}

void lte_disconnect(void)
{
  /* Send system connect command */
  command_sys.header.code = LTE_COMMAND_SYSTEM_DISCONNECT;
  command_sys.header.id = COMMAND_ID_DISCONNECT;
  send_recv_sys(&command_sys);

  printf("LTE Network is disconnected.\n");
}

void lte_sleep(void)
{
  /* Send system sleep command */
  command_sys.header.code = LTE_COMMAND_SYSTEM_SLEEP;
  command_sys.header.id = COMMAND_ID_SLEEP;
  send_recv_sys(&command_sys);

  printf("LTE modem sleep.\n");
}

void lte_get_number(char* number)
{
  /* Send system number command */
  printf("LT_Send() : LTE_COMMAND_UTIL_NUMBER\n");
  command_utl.header.code = LTE_COMMAND_UTIL_NUMBER;
  command_utl.header.id = COMMAND_ID_NUMBER;
  send_recv_utl(&command_utl);

  printf("Get number:%s.\n", g_number);

  strncpy(number, g_number, sizeof(g_number));
}
