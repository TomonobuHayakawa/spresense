/****************************************************************************
 * examples/volte/volte_main.c
 *
 *   Copyright (C) 2017 Sony Corporation.
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
#include "lte/lte_if_util.h"
#include "lte/lte_if_sms.h"
#include "lte/lte_if_volte.h"
#include <net/lwip/netdb.h>
#include <net/lwip/sockets.h>

#include "volte_audio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define MAX_NUMBER_OF_LTE_EXAMPLE_PARAM  10
#define MAX_LENGTH_OF_PARAM              16
#define LTE_NET_BUFFER_SIZE             128

#define D_APP_VOLTE_LTE_CONTINUE   1 /* continue */
#define D_APP_VOLTE_LTE_HOLD       2 /* hold */
#define D_APP_VOLTE_LTE_RESUME     3 /* resume */
#define D_APP_VOLTE_LTE_DISCONNECT 4 /* disconnect */
#define D_APP_VOLTE_LTE_ANSWER     5 /* answer */
#define D_APP_VOLTE_LTE_REJECT     6 /* reject */


/****************************************************************************
 * Private Data
 ****************************************************************************/


static int com_status = 0;
static int com_id = 0;

static LTECommand command;
static LTECommand_System command_sys;
static LTECommand_System response_sys;
#ifdef ENABLE_LTE_UTILITY_MODULE
  static LTECommand_Util command_utl;
  static LTECommand_Util response_utl;
#endif
#ifdef ENABLE_LTE_SMS_MODULE
  static LTECommand_Sms command_sms;
  static LTECommand_Sms response_sms;
#endif
static LTECommand_Volte command_volte;
static LTECommand_Volte response_volte;


typedef struct st_app_ltecommand
{
  char argc;
  char *argv[MAX_NUMBER_OF_LTE_EXAMPLE_PARAM];
} t_st_app_ltecommand;


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void app_callback_sys(void* out)
{
  LTECommand_System *data = (LTECommand_System *)out;

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
      case LTE_STRUCT_TYPE_PIN_INFO:
        printf("status:%s\n", data->message.pin.status);
        break;
      case LTE_STRUCT_TYPE_STATE:
        printf("state:%d\n", data->message.state.state);
        break;
      case LTE_STRUCT_TYPE_POWER_STATE:
        printf("power state:%d\n", data->message.pwrstate.state);
        break;
      case LTE_STRUCT_TYPE_WAKELOCK_STATE:
        printf("wakelock state:%d\n", data->message.lockstate.state);
        break;
      case LTE_STRUCT_TYPE_EVENT:
        printf("event:%s\nvalue:%d\nparam:%s\n", data->message.event.event, data->message.event.value, data->message.event.param);
        break;
      case LTE_STRUCT_TYPE_GPSNOTIFY:
        printf("handle:%d\nnotify:%d\nlocation:%d\nid:%s\nname:%s\nplane:%d\n", data->message.gpsnotify.handle, data->message.gpsnotify.notify, data->message.gpsnotify.location, data->message.gpsnotify.id, data->message.gpsnotify.name, data->message.gpsnotify.plane);
        break;
      case LTE_STRUCT_TYPE_GPSAUTO:
        printf("mode:%d\n", data->message.gpsauto.mode);
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


#ifdef ENABLE_LTE_UTILITY_MODULE
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
#endif


#ifdef ENABLE_LTE_SMS_MODULE
static void app_callback_sms(void* out)
{
  LTECommand_Sms *data = (LTECommand_Sms *)out;

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
      case LTE_STRUCT_TYPE_SMS_MEMORY:
        printf("memory:%s\nused:%d\ntotal:%d\n", data->message.memory.memory, data->message.memory.used, data->message.memory.total);
        break;
      case LTE_STRUCT_TYPE_SMS_SEND:
        printf("index:%d\ntime:%s\n", data->message.send.index, data->message.send.time);
        break;
      case LTE_STRUCT_TYPE_SMS_RECV:
        printf("index:%d\nstatus:%s\nnumber:%s\nctime:%s\ndtime:%s\n", data->message.recv.index, data->message.recv.status, data->message.recv.number, data->message.recv.ctime,  data->message.recv.dtime);
        if (data->message.recv.message_length > 0)
          {
            printf("%s\n", data->message.recv.message);
          }
        break;
      case LTE_STRUCT_TYPE_SMS_REPORT:
        printf("index:%d\nmemory:%s\n", data->message.report.index, data->message.report.memory);
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
#endif


static void app_callback_volte(void* out)
{
  LTECommand_Volte *data = (LTECommand_Volte *)out;

  com_status = data->header.result;
  printf("Response[%04X][%d][%d][%04X]\n", data->header.code, data->header.id, data->header.result, data->header.type);

  switch (data->header.type)
    {
      case LTE_STRUCT_TYPE_NONE:
        if( data->header.result !=0 )
          {
            printf("fail\n");
            return;
          }

        switch (data->header.code)
          {
            case LTE_COMMAND_VOLTE_ANSWER:
              /* receive OK(ATA) */
              com_status = 0;
              app_volte_audio_ctrl( APP_VOLTE_AUDIO_CTRL_BB_START );
              break;
            case LTE_COMMAND_VOLTE_HOOK:
              /* receive OK(ATH) */
              com_status = 0;
              app_volte_audio_ctrl( APP_VOLTE_AUDIO_CTRL_STOP );

              break;
            case LTE_COMMAND_VOLTE_SERVICE:
              /* receive OK(AT+CHLD) */
              com_status = 0;

              break;
            default:
              break;
          }

        break;
      case LTE_STRUCT_TYPE_VOLTE_RESPONSE:

        if( strcmp( data->message.response.status, "RING" ) == 0 )
          {
            printf("receive RING. \n");

            app_volte_audio_ctrl( APP_VOLTE_AUDIO_CTRL_RING_START );
          }
        else if( (strcmp( data->message.response.status, "NO CARRIER" ) == 0) ||
                 (strcmp( data->message.response.status, "CONNECTION TERMINATED" ) == 0) )
          {
            printf("receive NO CARRIER or +CDUU(CONNECTION TERMINATED)\n");

            app_volte_audio_ctrl( APP_VOLTE_AUDIO_CTRL_STOP );

          }
        else if( strcmp( data->message.response.status, "ANSWERED" ) == 0 )
          {
            printf("receive +CDUU(ANSWERED)\n");

          }
        else
          {
            printf("receive +CDUU( NO ANSWER or BUSY)\n");

            app_volte_audio_ctrl( APP_VOLTE_AUDIO_CTRL_STOP );
          }

        break;
      case LTE_STRUCT_TYPE_VOLTE_CALL:

        /* receive OK(+CDU). notified call index( In single call case, call index is not needed) */
        printf("receive +CDU\n");
        com_status = 0;

        app_volte_audio_ctrl( APP_VOLTE_AUDIO_CTRL_BB_START );

        break;

      case LTE_STRUCT_TYPE_VOLTE_NUMBER:
        /* receive +CCWA( for multi ) or +CLIP */
        printf("+CCWA or +CLIP:%s(%d)\n", data->message.number.number, data->message.number.type);
        break;

      case LTE_STRUCT_TYPE_VOLTE_SERVICE:
        printf("receive +CSSU or +CSSI: %d\n", data->message.service.code);
        break;

      case LTE_STRUCT_TYPE_VOLTE_CURRENT:
        printf("+CLCC [%x][%x][%x][%x][%x][%x]\n", data->message.current.index, data->message.current.dir, data->message.current.status, data->message.current.mode, data->message.current.multiparty,data->message.current.length);
        break;

      case LTE_STRUCT_TYPE_MESSAGE:
        printf("data:%s\n", data->message.text.data);
        break;

      default:
        printf("fail\n");
        return;
    }
}


int wait_recv(void)
{
  int count;

  /* Wait receive */
  for (count=0; count<3000; count++)
    {
      if (com_status != 1)
        {
          return com_status;
        }
      usleep(10000);
    }

  /* Timeout */
  return 0;
}


int send_recv_sys(LTECommand_System* cmd)
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


#ifdef ENABLE_LTE_UTILITY_MODULE
int send_recv_utl(LTECommand_Util* cmd)
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
#endif


#ifdef ENABLE_LTE_SMS_MODULE
int send_recv_sms(LTECommand_Sms* cmd)
{
  /* Status init */
  com_status = 1;

  /* Send and return code check */
  printf("LT_SendSms : %04X(%d)\n", cmd->header.code, cmd->header.id);
  LT_SendSms(cmd);
  if (cmd->header.result < 0)
    {
      printf("LT_SendSms error : %d\n", cmd->header.result);
      return cmd->header.result;
    }

  /* Wait receive */
  return wait_recv();
}
#endif


int send_recv_volte(LTECommand_Volte* cmd)
{
  /* Status init */
  com_status = 1;

  /* Send and return code check */
  printf("LT_SendVolte : %04X(%d)\n", cmd->header.code, cmd->header.id);
  LT_SendVolte(cmd);
  if (cmd->header.result < 0)
    {
      printf("LT_SendVolte error : %d\n", cmd->header.result);
      return cmd->header.result;
    }

  /* Wait receive */
  return wait_recv();
}


int send_recv_get(int cmd, int argc, const char *args[])
{
  int ret;
  int sock;
  int count;
  struct hostent *serv_ent;
  struct sockaddr_in ser_addr;
  char buf[LTE_NET_BUFFER_SIZE];
  char data[LTE_NET_BUFFER_SIZE];
  const char *host = args[3]; 
  int port = atoi(args[4]); 
  const char *path = args[5]; 

  /* Argument check */
  if ((host == NULL) || (port <= 0) || (path == NULL))
    {
      printf("Get routine : illegal argument error\n");
      return -1;
    }

  /* Create socket point */
  sock = NT_Socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0)
    {
      printf("NT_Socket error : %d\n", sock);
      return sock;
    }
  /* Get network host entry */
  serv_ent = gethostbyname((char *)host);
  if (serv_ent == NULL)
    {
      printf("NT_GetHostByName error\n");
      return -1;
    }

  /* Create socket address (server) */
  memset((char *)&ser_addr, 0, sizeof(ser_addr));
  ser_addr.sin_family = AF_INET;
  /*
  memcpy((char *)&ser_addr.sin_addr.s_addr, (char *)serv_ent->h_addr, serv_ent->h_length);
  */
  memcpy((char *)&ser_addr.sin_addr.s_addr, (char *)serv_ent->h_addr, sizeof(ser_addr.sin_addr.s_addr));
  ser_addr.sin_port = NT_Htons(port);

  /* Connect to server */
  if ((ret = NT_Connect(sock, (struct sockaddr*)&ser_addr, sizeof(ser_addr))) < 0)
    {
      printf("NT_Connect error : %d\n", ret);
      return ret;
    }

  /* Send and return code check */
  if (cmd == 0)
    {
      snprintf(buf, LTE_NET_BUFFER_SIZE, "GET %s HTTP/1.0\r\nHOST: %s\r\nConnection: close\r\n\r\n", path, host);
      printf("Network send : (%s)\n", buf);
      ret = NT_Send(sock, buf, strlen(buf), 0);
    }
  else
    {
      memset(data, 0, LTE_NET_BUFFER_SIZE);
      for (count=6; count<argc; count++)
        {
          strcat(data, args[count]);
        }
      strcat(data, "\r\n");
      snprintf(buf, LTE_NET_BUFFER_SIZE, "POST %s HTTP/1.0\r\nHOST: %s\r\nConnection: close\r\nContent-Length: %d\r\n\r\n", path, host, strlen(data));
      strcat(buf, data);
      printf("Network send : (%s)\n", buf);
      ret = NT_Send(sock, buf, strlen(buf), 0);
    }
  if (ret < 0)
    {
      printf("NT_Send error : %d\n", ret);
      NT_Close(sock);
      return ret;
    }

  while (1)
    {
      ret = NT_Recv(sock, buf, LTE_NET_BUFFER_SIZE, 0);
      if (ret < 0)
        {
          printf("NT_Recv error : %d\n", ret);
          NT_Close(sock);
          return ret;
        }
      if (ret == 0)
        {
          break;
        }
      printf("%s", buf);
    }

  NT_Close(sock);
  printf("NT_Close ()\n");
  return 0;
}


void app_lteSystemCommand(int argc, const char *args[], void *userData)
{
  int count;

  if ((argc >= 2) && (strcmp(args[2], "connect") == 0))
    {
      /* Send system connect command */
      printf("LT_Send() : LTE_COMMAND_SYSTEM_CONNECT\n");
      command_sys.header.code = LTE_COMMAND_SYSTEM_CONNECT;
      command_sys.header.id = com_id++;
      if (strlen(args[3]) > 0)
        {
          command_sys.header.type = LTE_STRUCT_TYPE_CONFIG;
          command_sys.message.config.IMSregist = atoi(args[3]);
        }
      send_recv_sys(&command_sys);
    }
  else if ((argc >= 2) && (strcmp(args[2], "confirm") == 0))
    {
      /* Send system confirm command */
      printf("LT_Send() : LTE_COMMAND_SYSTEM_CONFIRM\n");
      command_sys.header.code = LTE_COMMAND_SYSTEM_CONFIRM;
      command_sys.header.id = com_id++;
      send_recv_sys(&command_sys);
    }
  else if ((argc >= 2) && (strcmp(args[2], "disconnect") == 0)) {
       /* Send system disconenct command */
       printf("LT_Send() : LTE_COMMAND_SYSTEM_DISCONNECT\n");
       command_sys.header.code = LTE_COMMAND_SYSTEM_DISCONNECT;
       command_sys.header.id = com_id++;
       send_recv_sys(&command_sys);
    }
  else if ((argc >= 2) && (strcmp(args[2], "pin") == 0)) {
       /* Send system pin command */
       printf("LT_Send() : LTE_COMMAND_SYSTEM_PIN\n");
       command_sys.header.code = LTE_COMMAND_SYSTEM_PIN;
       command_sys.header.id = com_id++;
       send_recv_sys(&command_sys);
    }
  else if ((argc >= 2) && (strcmp(args[2], "state") == 0)) {
       /* Send system state command */
       printf("LT_Send() : LTE_COMMAND_SYSTEM_STATE\n");
       command_sys.header.code = LTE_COMMAND_SYSTEM_STATE;
       command_sys.header.id = com_id++;
       send_recv_sys(&command_sys);
    }
  else if ((argc >= 2) && (strcmp(args[2], "power") == 0))
    {
      /* Send system state command */
      printf("LT_Send() : LTE_COMMAND_SYSTEM_POWER_STATE\n");
      command_sys.header.code = LTE_COMMAND_SYSTEM_POWER_STATE;
      command_sys.header.id = com_id++;
      send_recv_sys(&command_sys);
    }
  else if ((argc >= 2) && (strcmp(args[2], "event") == 0))
    {
      if (strlen(args[3]) > 0)
        {
          /* Send system set event command */
          printf("LT_Send() : LTE_COMMAND_SYSTEM_SET_EVENT\n");
          command_sys.header.code = LTE_COMMAND_SYSTEM_SET_EVENT;
          command_sys.header.id = com_id++;
          command_sys.header.type = LTE_STRUCT_TYPE_EVENT;
          command_sys.message.event.flag = atoi(args[3]);
          send_recv_sys(&command_sys);
        }
      else
        {
          /* Send system get event command */
          printf("LT_Send() : LTE_COMMAND_SYSTEM_GET_EVENT\n");
          command_sys.header.code = LTE_COMMAND_SYSTEM_GET_EVENT;
          command_sys.header.id = com_id++;
          send_recv_sys(&command_sys);
        }
    }
  else if ((argc >= 2) && (strcmp(args[2], "sleep") == 0))
    {
      /* Send system sleep command */
      printf("LT_Send() : LTE_COMMAND_SYSTEM_SLEEP\n");
      command_sys.header.code = LTE_COMMAND_SYSTEM_SLEEP;
      command_sys.header.id = com_id++;
      send_recv_sys(&command_sys);
    }
  else if ((argc >= 2) && (strcmp(args[2], "wakelock") == 0))
    {
      /* Send system sleep command */
      printf("LT_Send() : LTE_COMMAND_SYSTEM_ACQUIRE_WAKELOCK\n");
      command_sys.header.code = LTE_COMMAND_SYSTEM_ACQUIRE_WAKELOCK;
      command_sys.header.id = com_id++;
      send_recv_sys(&command_sys);
    }
  else if ((argc >= 2) && (strcmp(args[2], "unlock") == 0))
    {
      /* Send system sleep command */
      printf("LT_Send() : LTE_COMMAND_SYSTEM_RELEASE_WAKELOCK\n");
      command_sys.header.code = LTE_COMMAND_SYSTEM_RELEASE_WAKELOCK;
      command_sys.header.id = com_id++;
      send_recv_sys(&command_sys);
    }
  else if ((argc >= 2) && (strcmp(args[2], "lockstate") == 0))
    {
      /* Send system sleep command */
      printf("LT_Send() : LTE_COMMAND_SYSTEM_WAKELOCK_STATE\n");
      command_sys.header.code = LTE_COMMAND_SYSTEM_WAKELOCK_STATE;
      command_sys.header.id = com_id++;
      send_recv_sys(&command_sys);
    }
  else if ((argc >= 2) && (strcmp(args[2], "notify") == 0))
    {
      /* Send system gpsnotify command */
      printf("LT_Send() : LTE_COMMAND_SYSTEM_GPSNOTIFY\n");
      command_sys.header.code = LTE_COMMAND_SYSTEM_GPSNOTIFY;
      command_sys.header.id = com_id++;
      command_sys.header.type = LTE_STRUCT_TYPE_GPSNOTIFY;
      command_sys.message.gpsnotify.flag = atoi(args[3]);
      send_recv_sys(&command_sys);
  }
  else if ((argc >= 2) && (strcmp(args[2], "allow") == 0))
    {
      /* Send system gpsallow command */
      printf("LT_Send() : LTE_COMMAND_SYSTEM_GPSALLOW\n");
      command_sys.header.code = LTE_COMMAND_SYSTEM_GPSALLOW;
      command_sys.header.id = com_id++;
      command_sys.header.type = LTE_STRUCT_TYPE_GPSALLOW;
      command_sys.message.gpsallow.allow = atoi(args[3]);
      command_sys.message.gpsallow.handle = atoi(args[4]);
      send_recv_sys(&command_sys);
  }
  else if ((argc >= 2) && (strcmp(args[2], "auto") == 0))
    {
      if (strlen(args[3]) > 0)
        {
          /* Send system set gpsauto command */
          printf("LT_Send() : LTE_COMMAND_SYSTEM_SET_GPSAUTO\n");
          command_sys.header.code = LTE_COMMAND_SYSTEM_SET_GPSAUTO;
          command_sys.header.id = com_id++;
          command_sys.header.type = LTE_STRUCT_TYPE_GPSAUTO;
          command_sys.message.gpsauto.mode = atoi(args[3]);
          send_recv_sys(&command_sys);
        }
      else
        {
          /* Send system get gpsauto command */
          printf("LT_Send() : LTE_COMMAND_SYSTEM_GET_GPSAUTO\n");
          command_sys.header.code = LTE_COMMAND_SYSTEM_GET_GPSAUTO;
          command_sys.header.id = com_id++;
          send_recv_sys(&command_sys);
        }
    }
  else if ((argc >= 2) && (strcmp(args[2], "start") == 0)) {
      /* Send LT_Start command */
      command.debug_level = LTE_DEBUG_INFO;
      LT_Start(&command);
      if (command.result == 0)
        {
          printf("LT_Start() done\n");
  
          /* Set callback */
          LT_SetCallbackSystem(&app_callback_sys, &response_sys);
#ifdef ENABLE_LTE_UTILITY_MODULE
          LT_SetCallbackUtil(&app_callback_utl, &response_utl);
#endif
#ifdef ENABLE_LTE_SMS_MODULE
          LT_SetCallbackSms(&app_callback_sms, &response_sms);
#endif
        }
      else
        {
          printf("LT_Start() error : %d\n", command.result);
        }
    }
  else if ((argc >= 2) && (strcmp(args[2], "stop") == 0))
    {
      /* Send LT_Stop command */
      LT_Stop(&command);
      if (command.result == 0) {
        printf("LT_Stop() done\n");
      }
      else
        {
          printf("LT_Stop() error : %d\n", command.result);
        }
    }
  else
    {
      printf("Can't execute command :");
      for (count=0; count<argc; count++)
        {
          printf(" %s", args[count]);
        }
      printf("\n");
    }
}


void app_lteUtilityCommand(int argc, const char *args[], void *userData)
{
  int count;

#ifdef ENABLE_LTE_UTILITY_MODULE
  if ((argc >= 2) && (strcmp(args[2], "version") == 0))
    {
    /* Send system version command */
    printf("LT_Send() : LTE_COMMAND_UTIL_VERSION\n");
    command_utl.header.code = LTE_COMMAND_UTIL_VERSION;
    command_utl.header.id = com_id++;
    send_recv_utl(&command_utl);
  }
  else if ((argc >= 2) && (strcmp(args[2], "number") == 0))
    {
      /* Send system number command */
      printf("LT_Send() : LTE_COMMAND_UTIL_NUMBER\n");
      command_utl.header.code = LTE_COMMAND_UTIL_NUMBER;
      command_utl.header.id = com_id++;
      send_recv_utl(&command_utl);
    }
  else if ((argc >= 2) && (strcmp(args[2], "report") == 0))
    {
      /* Send system error report command */
      printf("LT_Send() : LTE_COMMAND_UTIL_REPORT\n");
      command_utl.header.code = LTE_COMMAND_UTIL_REPORT;
      command_utl.header.id = com_id++;
      send_recv_utl(&command_utl);
    }
  else if ((argc >= 2) && (strcmp(args[2], "quality") == 0))
    {
      /* Send system quality command */
      printf("LT_Send() : LTE_COMMAND_UTIL_QUALITY\n");
      command_utl.header.code = LTE_COMMAND_UTIL_QUALITY;
      command_utl.header.id = com_id++;
      command_utl.header.type = LTE_STRUCT_TYPE_UTIL_QUALITY;
      command_utl.message.quality.flag = atoi(args[3]);
      send_recv_utl(&command_utl);
    }
  else if ((argc >= 2) && (strcmp(args[2], "register") == 0))
    {
      /* Send system register command */
      printf("LT_Send() : LTE_COMMAND_UTIL_REGISTER\n");
      command_utl.header.code = LTE_COMMAND_UTIL_REGISTER;
      command_utl.header.id = com_id++;
      send_recv_utl(&command_utl);
    }
  else if ((argc >= 2) && (strcmp(args[2], "clock") == 0))
    {
      /* Send system clock command */
      printf("LT_Send() : LTE_COMMAND_UTIL_CLOCK\n");
      command_utl.header.code = LTE_COMMAND_UTIL_CLOCK;
      command_utl.header.id = com_id++;
      send_recv_utl(&command_utl);
    }
  else if ((argc >= 2) && (strcmp(args[2], "data") == 0))
    {
      if ((strlen(args[3]) > 0) && (strlen(args[4]) > 0) && (strlen(args[5]) > 0))
        {
          /* Send system set data command */
          printf("LT_Send() : LTE_COMMAND_UTIL_SET_DATA\n");
          command_utl.header.code = LTE_COMMAND_UTIL_SET_DATA;
          command_utl.header.id = com_id++;
          command_utl.header.type = LTE_STRUCT_TYPE_UTIL_DATA;
          command_utl.message.data.disable_data = atoi(args[3]);
          command_utl.message.data.disable_data_roaming = atoi(args[4]);
          command_utl.message.data.time_sync = atoi(args[5]);
          send_recv_utl(&command_utl);
        }
      else
        {
          /* Send system get data command */
          printf("LT_Send() : LTE_COMMAND_UTIL_GET_DATA\n");
          command_utl.header.code = LTE_COMMAND_UTIL_GET_DATA;
          command_utl.header.id = com_id++;
          send_recv_utl(&command_utl);
        }
    }
  else if ((argc >= 2) && (strncmp(args[2], "AT", 2) == 0))
    {
      /* Send AT command */
      printf("LT_Send() : LTE_COMMAND_UTIL_ATCOMMAND\n");
      strncpy(command_utl.message.ATcommand.data, args[2], LTE_SIZE_COMMAND_BUFFER-1);
      command_utl.header.code = LTE_COMMAND_UTIL_ATCOMMAND;
      command_utl.header.id = com_id++;
      command_utl.header.type = LTE_STRUCT_TYPE_MESSAGE;
      command_utl.message.ATcommand.length = strlen(command_utl.message.ATcommand.data);
      send_recv_utl(&command_utl);
    }
  else
    {
      printf("Can't execute command :");
      for (count=0; count<argc; count++)
        {
          printf(" %s", args[count]);
        }
      printf("\n");
    }
#else
  printf("Can't execute command :");
  for (count=0; count<argc; count++)
    {
      printf(" %s", args[count]);
    }
  printf("\n");
#endif
}

void app_lteSmsCommand(int argc, const char *args[], void *userData)
{
  int count;

#ifdef ENABLE_LTE_SMS_MODULE
  if ((argc >= 2) && (strcmp(args[2], "config") == 0))
    {
      /* Send sms config command */
      printf("LT_Send() : LTE_COMMAND_SMS_CONFIG\n");
      command_sms.header.code = LTE_COMMAND_SMS_CONFIG;
      command_sms.header.id = com_id++;
      send_recv_sms(&command_sms);
    }
  else if ((argc >= 2) && (strcmp(args[2], "memory") == 0)) {
      /* Send sms memory command */
      if (strlen(args[3]) == 2)
        {
          printf("LT_Send() : LTE_COMMAND_SMS_SET_MEMORY\n");
          strncpy(command_sms.message.memory.memory, args[3], LTE_SIZE_COMMAND_SMS_MEMORY-1);
          command_sms.header.code = LTE_COMMAND_SMS_SET_MEMORY;
          command_sms.header.id = com_id++;
          command_sms.header.type = LTE_STRUCT_TYPE_SMS_MEMORY;
          command_sms.message.memory.memory_length = strlen(command_sms.message.memory.memory);
          send_recv_sms(&command_sms);
        }
      else if (strlen(args[3]) > 0)
        {
          printf("This command can use \"ME\" and \"SM\" only\n");
        }
      else
        {
          printf("LT_Send() : LTE_COMMAND_SMS_GET_MEMORY\n");
          command_sms.header.code = LTE_COMMAND_SMS_GET_MEMORY;
          command_sms.header.id = com_id++;
          send_recv_sms(&command_sms);
        }
    }
  else if ((argc >= 2) && (strcmp(args[2], "list") == 0))
    {
      /* Send sms list command */
      printf("LT_Send() : LTE_COMMAND_SMS_LIST\n");
      command_sms.header.code = LTE_COMMAND_SMS_LIST;
      command_sms.header.id = com_id++;
      command_sms.header.type = LTE_STRUCT_TYPE_SMS_RECV;
      if (strlen(args[3]) > 0)
        {
          strncpy(command_sms.message.recv.status, args[3], LTE_SIZE_COMMAND_SMS_STATUS-1);
        }
      else
        {
          strncpy(command_sms.message.recv.status, "ALL", LTE_SIZE_COMMAND_SMS_STATUS-1);
        }
      command_sms.message.recv.status_length = strlen(command_sms.message.recv.status);
      send_recv_sms(&command_sms);
    }
  else if ((argc >= 2) && (strcmp(args[2], "read") == 0))
    {
      /* Send sms read command */
      printf("LT_Send() : LTE_COMMAND_SMS_READ\n");
      command_sms.header.code = LTE_COMMAND_SMS_READ;
      command_sms.header.id = com_id++;
      command_sms.header.type = LTE_STRUCT_TYPE_SMS_RECV;
      if (strlen(args[3]) > 0)
        {
          command_sms.message.recv.index = atoi(args[3]);
        }
      else
        {
          command_sms.message.recv.index = 1;
        }
      send_recv_sms(&command_sms);
  }
  else if ((argc >= 2) && (strcmp(args[2], "send") == 0))
    {
      /* Send sms send command */
      printf("LT_Send() : LTE_COMMAND_SMS_SEND\n");
      command_sms.header.code = LTE_COMMAND_SMS_SEND;
      command_sms.header.id = com_id++;
      command_sms.header.type = LTE_STRUCT_TYPE_SMS_SEND;
      if (strlen(args[3]) > 0)
        {
          command_sms.message.send.index = atoi(args[3]);
        }
      else
        {
          command_sms.message.send.index = 1;
        }
      send_recv_sms(&command_sms);
    }
  else if ((argc >= 2) && (strcmp(args[2], "write") == 0))
    {
      /* Send sms write command */
      printf("LT_Send() : LTE_COMMAND_SMS_WRITE\n");
      command_sms.header.code = LTE_COMMAND_SMS_WRITE;
      command_sms.header.id = com_id++;
      command_sms.header.type = LTE_STRUCT_TYPE_SMS_SEND;
      command_sms.message.send.ucs2_flag = 0;
      strncpy(command_sms.message.send.number, args[3], LTE_SIZE_COMMAND_SMS_NUMBER-1);
      command_sms.message.send.number_length = strlen(command_sms.message.send.number);
      strncpy(command_sms.message.send.message, args[4], LTE_SIZE_COMMAND_SMS_MESSAGE-1);
      count = 5;
      while ((argc > count) && (strlen(args[count]) > 0))
        {
          strcat(command_sms.message.send.message, " ");
          strcat(command_sms.message.send.message, args[count]);
          count++;
        }
      command_sms.message.send.message_length = strlen(command_sms.message.send.message);
      send_recv_sms(&command_sms);
    }
  else if ((argc >= 2) && (strcmp(args[2], "direct") == 0))
    {
      /* Send sms direct command */
      printf("LT_Send() : LTE_COMMAND_SMS_DIRECT\n");
      command_sms.header.code = LTE_COMMAND_SMS_DIRECT;
      command_sms.header.id = com_id++;
      command_sms.header.type = LTE_STRUCT_TYPE_SMS_SEND;
      command_sms.message.send.ucs2_flag = 0;
      strncpy(command_sms.message.send.number, args[3], LTE_SIZE_COMMAND_SMS_NUMBER-1);
      command_sms.message.send.number_length = strlen(command_sms.message.send.number);
      strncpy(command_sms.message.send.message, args[4], LTE_SIZE_COMMAND_SMS_MESSAGE-1);
      count = 5;
      while ((argc > count) && (strlen(args[count]) > 0))
        {
          strcat(command_sms.message.send.message, " ");
          strcat(command_sms.message.send.message, args[count]);
          count++;
        }
      command_sms.message.send.message_length = strlen(command_sms.message.send.message);
      send_recv_sms(&command_sms);
    }
  else if ((argc >= 2) && (strcmp(args[2], "delete") == 0))
    {
      /* Send sms delete command */
      printf("LT_Send() : LTE_COMMAND_SMS_DELETE\n");
      command_sms.header.code = LTE_COMMAND_SMS_DELETE;
      command_sms.header.id = com_id++;
      command_sms.header.type = LTE_STRUCT_TYPE_SMS_DELETE;
      if (strncmp(args[3], "all", 3) == 0)
        {
          command_sms.message.del.flag = 1;
          command_sms.message.del.index = 0;
        }
      else
        {
          command_sms.message.del.flag = 0;
          command_sms.message.del.index = atoi(args[3]);
        }
      send_recv_sms(&command_sms);
    }
  else
    {
      printf("Can't execute command :");
      for (count=0; count<argc; count++)
        {
          printf(" %s", args[count]);
        }
      printf("\n");
    }
#else
  printf("Can't execute command :");
  for (count=0; count<argc; count++)
    {
      printf(" %s", args[count]);
    }
  printf("\n");
#endif
}

static void app_volte_lte_send_at_chld( int code )
{
  command_volte.header.code = LTE_COMMAND_VOLTE_SERVICE;
  command_volte.header.id = com_id++;
  command_volte.header.type = LTE_STRUCT_TYPE_VOLTE_SERVICE;
  command_volte.message.service.code = code;
  send_recv_volte(&command_volte);

  if( code !=2 ) /* code=2 means hold/resume */
    {
      app_volte_audio_ctrl( APP_VOLTE_AUDIO_CTRL_STOP );
    }

  if( code == 4) /* code=4 means explicit call transfer */
    {
      app_volte_audio_ctrl( APP_VOLTE_AUDIO_CTRL_STOP );
    }
}

static int app_volte_lte_get_muticall_ctrl_code( const char *ctrl )
{
  if( strcmp( ctrl, "continue" )==0 )
    {
      return D_APP_VOLTE_LTE_CONTINUE;
    }
  else if( strcmp( ctrl, "hold" )==0 )
    {
      return D_APP_VOLTE_LTE_HOLD;
    }
  else if( strcmp( ctrl, "resume")==0 )
    {
      return D_APP_VOLTE_LTE_RESUME;
    }
  else if( strcmp( ctrl, "disconnect")==0 )
    {
      return D_APP_VOLTE_LTE_DISCONNECT;
    }
  else if( strcmp( ctrl, "answer")==0 )
    {
      return D_APP_VOLTE_LTE_ANSWER;
    }
  else if( strcmp( ctrl, "reject")==0 )
    {
      return D_APP_VOLTE_LTE_REJECT;
    }
  else
    {
      return 0;
    }
}

void app_lteVolteCommand(int argc, const char *args[], void *userData)
{
  int count;

  memset( &command_volte, 0, sizeof( command ));

  if ((argc >= 2) && (strcmp(args[2], "config") == 0))
    {
      command_volte.header.code = LTE_COMMAND_VOLTE_CONFIG;
      command_volte.header.id = com_id++;
      send_recv_volte(&command_volte);

    }
  else if ((argc >= 2) && (strcmp(args[2], "call") == 0))
    {
      /* Send AT+CDU request */
      printf("mo call start\n");

      command_volte.header.code = LTE_COMMAND_VOLTE_CALL;
      command_volte.header.id = com_id++;
      command_volte.header.type = LTE_STRUCT_TYPE_VOLTE_CALL;

      if( strcmp( args[3], "tel" ) == 0 )
        {
          command_volte.message.call.flag = LTE_FLAG_VOLTE_TEL;
          strncpy( command_volte.message.call.tel, args[4], LTE_SIZE_COMMAND_VOLTE_TEL);

          command_volte.message.call.tel_length = strlen(args[4]);
        }
      else if( strcmp( args[3], "sip" ) == 0 )
        {
          command_volte.message.call.flag = LTE_FLAG_VOLTE_SIP;
          strncpy( command_volte.message.call.sip, args[4], LTE_SIZE_COMMAND_VOLTE_SIP);
          command_volte.message.call.sip_length = strlen(args[4]);
        }
      else
        {
          printf("Parameter Error!!\n");
          return;
        }

      send_recv_volte(&command_volte);

    }
  else if((argc >= 2) && (strcmp(args[2], "callinfo") == 0))
    {
      printf("get call information()\n");
      command_volte.header.code = LTE_COMMAND_VOLTE_STATUS;
      command_volte.header.id = com_id++;
      send_recv_volte(&command_volte);
    }
  else if ((argc >= 2) && (strcmp(args[2], "answer") == 0))
    {
      /* Send ATA */
      printf("answer MT call(send ATA)\n");
      command_volte.header.code = LTE_COMMAND_VOLTE_ANSWER;
      command_volte.header.id = com_id++;
      send_recv_volte(&command_volte);
    }
  else if ((argc >= 2) && (strcmp(args[2], "reject") == 0))
    {
      printf("Reject MT call(send ATH)\n");
      command_volte.header.code = LTE_COMMAND_VOLTE_HOOK;
      command_volte.header.id = com_id++;
      send_recv_volte(&command_volte);
    }
  else if ((argc >= 2) && (strcmp(args[2], "end") == 0))
    {
      printf("end call\n");
      app_volte_lte_send_at_chld(1);
    }
  else if ((argc >= 2) && (strcmp(args[2], "hold") == 0))
    {
      printf("hold on single call(send AT+CHLD=2)\n");
      app_volte_lte_send_at_chld(2);
    }
  else if ((argc >= 2) && (strcmp(args[2], "resume") == 0))
    {
      printf("resume on single call(send AT+CHLD=2)\n");
      app_volte_lte_send_at_chld(2);
    }
  else if((argc >= 2) && (strcmp(args[2], "multicall") == 0))
    {
      printf("control multi call(send AT+CHLD=0,1,2)\n");
      /* arg[3] and arg[4] are control for current call and for incoming call/call in hold  */

      int current_code;
      int another_code;
      int code;

      current_code = app_volte_lte_get_muticall_ctrl_code( args[3] );
      another_code = app_volte_lte_get_muticall_ctrl_code( args[4] );

      switch ( another_code )
        {
          case D_APP_VOLTE_LTE_RESUME:
          case D_APP_VOLTE_LTE_ANSWER:
            switch ( current_code )
              {
                case D_APP_VOLTE_LTE_DISCONNECT:
                  /* send AT+CHLD=1 */
                  code = 1;

                  break;
                case D_APP_VOLTE_LTE_HOLD:
                  /* send AT+CHLD=2 */
                  code = 2;
                  break;
                default:
                  return;
              }
            break;

          case D_APP_VOLTE_LTE_DISCONNECT:
          case D_APP_VOLTE_LTE_REJECT:

            switch ( current_code )
              {
                case D_APP_VOLTE_LTE_CONTINUE:
                  /* send AT+CHLD=0 */
                  code = 0;
                  break;
                default:
                  return;
              }
            break;

          default:
            return;
        }
      app_volte_lte_send_at_chld(code);
    }
  else if ((argc >= 2) && (strcmp(args[2], "ect") == 0))
    {
      printf("Explicit Call Transfer (send AT+CHLD=4)\n");
      app_volte_lte_send_at_chld(4);
    }
  else if ((argc >= 2) && (strcmp(args[2], "3waycall") == 0))
    {
      printf("Start 3way call (send AT+CHLD=3)\n");
      app_volte_lte_send_at_chld(3);
    }
  else
    {
      printf("Can't execute command :");
      for (count=0; count<argc; count++)
        {
          printf(" %s", args[count]);
        }
      printf("\n");
    }
}


void app_lteNetCommand(int argc, const char *args[], void *userData)
{
  int count;

  if ((argc >= 5) && (strcmp(args[2], "GET") == 0))
    {
      printf("GET with send/recv()\n");
      send_recv_get(0, argc, args);
    }
  else if ((argc >= 5) && (strcmp(args[2], "POST") == 0))
    {
      printf("POST with send/recv()\n");
      send_recv_get(1, argc, args);
    }
  else
   {
      printf("Can't execute command :");
      for (count=0; count<argc; count++)
        {
          printf(" %s", args[count]);
        }
      printf("\n");
    }
}

void lte_init(void)
{
  while (1)
    {
      /* Delay for link detection */
      sleep(3);

      /* Initialize task */
      command.debug_level = LTE_DEBUG_INFO;
      LT_Start(&command);
      if (command.result == 0)
        {
          break;
        }
      printf("LT_Start() error : %d\n", command.result);
    }

  printf("LT_Start() done\n");

  /* Set callback */
  LT_SetCallbackSystem(&app_callback_sys, &response_sys);
#ifdef ENABLE_LTE_UTILITY_MODULE
  LT_SetCallbackUtil(&app_callback_utl, &response_utl);
#endif
#ifdef ENABLE_LTE_SMS_MODULE
  LT_SetCallbackSms(&app_callback_sms, &response_sms);
#endif
  LT_SetCallbackVolte(&app_callback_volte, &response_volte);
}


void lte_wait_main( void )
{
  int cnt;
  mqd_t mqd;
  struct mq_attr mq_attr;
  t_st_app_ltecommand *msg;
  int lte_task_end_flg=0;

  mq_attr.mq_maxmsg  = 5;
  mq_attr.mq_msgsize = sizeof(t_st_app_ltecommand *);
  mq_attr.mq_flags   = 0;

  lte_init();

  mqd = mq_open( "lte_example", O_RDONLY | O_CREAT, 0666, &mq_attr);

  while (1)
    {

      if( mq_receive( mqd, (char *)(&msg), sizeof(t_st_app_ltecommand *), NULL ) )
        {
          printf("Procedure start\n");

          if( strncmp( msg->argv[1], "end", 3 ) == 0 )
            {
              lte_task_end_flg=1;
            }
          else if( strncmp( msg->argv[1], "system", 6 ) == 0 )
            {
              app_lteSystemCommand( msg->argc, (const char **)(msg->argv), NULL );
            }
          else if( strncmp( msg->argv[1], "utility", 6 ) == 0 )
            {
              app_lteUtilityCommand( msg->argc, (const char **)(msg->argv), NULL );
            }
          else if( strncmp( msg->argv[1], "sms", 3 ) == 0 )
            {
              app_lteSmsCommand( msg->argc, (const char **)(msg->argv), NULL );
            }
          else if( strncmp( msg->argv[1], "net", 3 ) == 0 )
            {
              app_lteNetCommand( msg->argc, (const char **)(msg->argv), NULL );
            }
          else if( strncmp( msg->argv[1], "volte", 5 ) == 0 )
            {
              app_lteVolteCommand( msg->argc, (const char **)(msg->argv), NULL );
            }
          else
            {
              printf("Parameter Error\n");
            }

          for( cnt=0; cnt<MAX_NUMBER_OF_LTE_EXAMPLE_PARAM; cnt++)
            {
              free(msg->argv[cnt]);
            }
          free(msg);


          if(lte_task_end_flg == 1)
            {
              printf("LTE task finish!!\n");
              break;
            }

          printf("Procedure Finished. Input next command.\n");
        }
      else
        {
          sleep(1);
        }
    }
}


#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int volte_main(int argc, char *argv[])
#endif
{
  int cnt;
  mqd_t mqd;
  t_st_app_ltecommand *msg;

  if( argc < 2 )
    {
       printf( "Parameter is few.\n");
       return 1;
    }

  if( strncmp(argv[1], "init",4 )==0 )
    {
      task_create( "lte_ex_task", 100, 2048, (main_t)lte_wait_main, NULL );

      if (app_volte_audio_init() != 0)
        {
          return 1;
        }
    }
  else
    {
      if( argc > MAX_NUMBER_OF_LTE_EXAMPLE_PARAM )
        {
          printf(" the number of parameter is very large!!\n");
          return 0;
        }

      msg = malloc( sizeof(t_st_app_ltecommand) );
      memset( msg, 0, sizeof(t_st_app_ltecommand));

      msg->argc=argc;
      for( cnt=0; cnt<argc; cnt++ )
        {
          msg->argv[cnt] = malloc( strlen( argv[cnt] ) + 1);
          strncpy( msg->argv[cnt], argv[cnt],  strlen( argv[cnt] ) + 1);
        }
      for( cnt=argc; cnt<MAX_NUMBER_OF_LTE_EXAMPLE_PARAM; cnt++ )
        {
          msg->argv[cnt] = malloc(1);
          msg->argv[cnt][0] = '\0';
        }


      mqd = mq_open( "lte_example", O_WRONLY, 0666, NULL);

      if( mq_send( mqd, (const char *)(&msg), sizeof(t_st_app_ltecommand *), 0 ) != 0 )
        {
          printf("mq_send to lte_task_main Failed!!\n");
          for( cnt=0; cnt<MAX_NUMBER_OF_LTE_EXAMPLE_PARAM; cnt++)
            {
              free(msg->argv[cnt]);
            }

          free(msg);
        }
    }

  return 0;
}

