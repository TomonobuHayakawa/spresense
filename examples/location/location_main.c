/****************************************************************************
 * examples/location/location_main.c
 *
 *   Copyright (C) 2016 Sony Corporation. All rights reserved.
 *   Author: Mitsuo Hiragane <xMitsuo.Hiragane@jp.sony.com>
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

#ifdef CONFIG_LTE
#include <lte/lte_if.h>
#include <lte/lte_if_system.h>
#endif
#include <location/location_manager.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define MAX_NUMBER_OF_LOCATION_EXAMPLE_PARAM  10
#define MIN_NUMBER_OF_LOCATION_EXAMPLE_PARAM  2

/****************************************************************************
 * Private Data
 ****************************************************************************/
typedef struct st_app_location_cmd
{
  char argc;
  char *argv[MAX_NUMBER_OF_LOCATION_EXAMPLE_PARAM];
} t_st_app_location_cmd;

static int  g_ctl_id  = 0;
static bool g_is_init = false;

#ifdef CONFIG_LTE
static int               g_com_status = 0;
static int               g_com_id = 0;
static LTECommand        g_command      = {0};
static LTECommand_System g_command_sys  = {0};
static LTECommand_System g_response_sys = {0};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_LTE
static void app_callback_sys(void* out)
{
  LTECommand_System *data = (LTECommand_System *)out;

  g_com_status = data->header.result;
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

static int wait_recv(void)
{
  int count;

  /* Wait receive */
  for (count=0; count<500; count++)
    {
      if (g_com_status != 1)
        {
          return g_com_status;
        }
      usleep(10000);
    }

  /* Timeout */
  return 0;
}


static int send_recv_sys(LTECommand_System* cmd)
{
  /* Status init */
  g_com_status = 1;

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
#endif

static void lte_pos_data_cb(struct loc_mng_pos_data *pos_data)
{
  printf("[ctl_id:%d]%04d/%02d/%02d %02d:%02d:%02d.%d LAT:%.06f LONG:%.06f ALT:%.06f\n"
          ,pos_data->ctl_id
          ,pos_data->date.year
          ,pos_data->date.month
          ,pos_data->date.day
          ,pos_data->time.hour
          ,pos_data->time.minute
          ,pos_data->time.sec
          ,pos_data->time.usec
          ,pos_data->latitude
          ,pos_data->longitude
          ,pos_data->altitude
        );
  return;
}

static void lte_init(void)
{
#ifdef CONFIG_LTE
  while (1)
    {
      /* Initialize task */
      g_command.debug_level = LTE_DEBUG_ERROR;
      LT_Start(&g_command);
      if (g_command.result == 0)
        {
          break;
        }
      printf("LT_Start() error : %d\n", g_command.result);
      sleep(2);
    }

  printf("LT_Start() done\n");

  /* Set callback */
  LT_SetCallbackSystem(&app_callback_sys, &g_response_sys);
#endif

  LocMngInit();
}

static void lte_Fin(void)
{
  LocMngFin();

#ifdef CONFIG_LTE
  while (1)
    {
      /* Delay for link detection */
      sleep(3);

      /* Initialize task */
      LT_Stop(&g_command);
      if (g_command.result == 0)
        {
          break;
        }
      printf("LT_Stop() error : %d\n", g_command.result);
    }

  printf("LT_Stop() done\n");
#endif
}

static void location_wait_main( void )
{
  int cnt = 0;;
  int ret = 0;
  int ctl_id = g_ctl_id;
  mqd_t mqd;
  struct mq_attr mq_attr;
  t_st_app_location_cmd *msg;
  int location_task_end_flg=0;
  struct loc_mng_start_param sta_prm = {LOC_MNG_NUM_OF_FIX_INFNITE, 1};

  mq_attr.mq_maxmsg  = 5;
  mq_attr.mq_msgsize = sizeof(t_st_app_location_cmd *);
  mq_attr.mq_flags   = 0;

  lte_init();

  mqd = mq_open( "location_example", O_RDONLY | O_CREAT, 0666, &mq_attr);

  while (1)
    {

      if( mq_receive( mqd, (char *)(&msg), sizeof(t_st_app_location_cmd *), NULL ) )
        {
          printf("Procedure start\n");
         if( strncmp( msg->argv[1], "fin", 3 ) == 0 )
            {
              lte_Fin();
              location_task_end_flg = 1;
              printf("LocMngFin() ret = %d\n", ret);
            }
#ifdef CONFIG_LTE
          if( strncmp( msg->argv[1], "connect", 7 ) == 0 )
            {
              /* Send system connect command */
              printf("LT_Send() : LTE_COMMAND_SYSTEM_CONNECT\n");
              g_command_sys.header.code = LTE_COMMAND_SYSTEM_CONNECT;
              g_command_sys.header.id = g_com_id++;
              if (strlen(msg->argv[2]) > 0)
                {
                  g_command_sys.header.type = LTE_STRUCT_TYPE_CONFIG;
                  g_command_sys.message.config.IMSregist = atoi(msg->argv[2]);
                }
              send_recv_sys(&g_command_sys);
            }
#endif
          else if( strncmp( msg->argv[1], "start", 5 ) == 0 )
            {
               if (strlen(msg->argv[2]) > 0)
                {
                  sta_prm.num_of_fixes = atoi(msg->argv[2]);
                }

               if (strlen(msg->argv[3]) > 0)
                {
                  sta_prm.cycle = atoi(msg->argv[3]);
                }
               sta_prm.cbs = lte_pos_data_cb;
               ret = LocMngStart(&sta_prm, &g_ctl_id);
               printf("LocMngStart() ret = %d ctl_id = %d\n", ret, g_ctl_id);
            }
          else if( strncmp( msg->argv[1], "stop", 4 ) == 0 )
            {
              if (strlen(msg->argv[2]) > 0)
                {
                  ctl_id = atoi(msg->argv[2]);
                }

              ret = LocMngStop(ctl_id);
              printf("LocMngStop() ret = %d\n", ret);
            }
          else
            {
              printf("Invalid argument = %s\n", msg->argv[1]);
            }

          for( cnt=0; cnt<MAX_NUMBER_OF_LOCATION_EXAMPLE_PARAM; cnt++)
            {
              free(msg->argv[cnt]);
            }
          free(msg);


          if(location_task_end_flg == 1)
            {
              printf("location task finish!!\n");
              break;
            }

          printf("Procedure Finished. Input next command.\n");
        }
      else
        {
          sleep(1);
        }
    }

  mq_close(mqd);
  g_is_init = false;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int location_main(int argc, char *argv[])
#endif
{
  int                   cnt = 0;
  mqd_t                 mqd = 0;
  t_st_app_location_cmd *msg = NULL;

  if(0 == strncmp(argv[1], "init",4 ))
    {
      task_create( "location_ex_task", 100, 2048, (main_t)location_wait_main, NULL );
      g_is_init = true;
    }
  else
    {
      if(argc > MAX_NUMBER_OF_LOCATION_EXAMPLE_PARAM)
        {
          printf("the number of parameter is very large!!\n");
          return 0;
        }

      if (argc < MIN_NUMBER_OF_LOCATION_EXAMPLE_PARAM)
        {
          printf("command not found\n");
          return 0;
        }

      if (true != g_is_init)
        {
          printf("Please initialize\n");
          return 0;
        }

      msg = malloc( sizeof(t_st_app_location_cmd) );
      memset( msg, 0, sizeof(t_st_app_location_cmd));

      msg->argc=argc;
      for(cnt=0; cnt<argc; cnt++)
        {
          msg->argv[cnt] = malloc( strlen( argv[cnt] ) + 1);
          strncpy( msg->argv[cnt], argv[cnt],  strlen( argv[cnt] ) + 1);
        }
      for(cnt=argc; cnt<MAX_NUMBER_OF_LOCATION_EXAMPLE_PARAM; cnt++)
        {
          msg->argv[cnt] = malloc(1);
          msg->argv[cnt][0] = '\0';
        }

      mqd = mq_open("location_example", O_WRONLY, 0666, NULL);

      if(0 != mq_send( mqd, (const char *)(&msg), sizeof(t_st_app_location_cmd *), 0 ))
        {
          printf("mq_send to location_task_main Failed!!\n");
          for(cnt=0; cnt<MAX_NUMBER_OF_LOCATION_EXAMPLE_PARAM; cnt++)
            {
              free(msg->argv[cnt]);
            }

          free(msg);
        }

      mq_close(mqd);
    }

  return 0;
}
