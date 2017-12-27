/****************************************************************************
 * examples/mqtt/mqtt_main.c
 *
 *   Copyright (C) 2016 Sony Corporation. All rights reserved.
 *   Author: Mitsuo Hiragane <Mitsuo.Hiragane@sony.com>
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
#include <string.h>
#include <stdlib.h>
#include <mqueue.h>
#include <fcntl.h>

#include <lte/lte_if.h>
#include <lte/lte_if_system.h>
#include <lte/lte_if_util.h>

#include "netutils/mqtt/MQTTClient.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/*------------------
 * Topic
 *----------------*/
#define PAYLOAD_SW_SIZE		160
#define BDKCONNECT_BUF_SIZE	400
#define BDKCONNECT_CLIENT_ID	"iottest"
#define BDKCONNECT_DEVICE_ID	"iottest"
#define BDKCONNECT_BROKER       "mqtt_broker.com"
#define BDKCONNECT_ROOTCA       "/mnt/spif/VeriSign.pem"
#define BDKCONNECT_DEV_CERT     "/mnt/spif/cli_cert.crt"
#define BDKCONNECT_PRI_KEY      "/mnt/spif/private.key"


/*------------------
 * Command
 *----------------*/
#define MQTT_COMMAND_CONNECT             1
#define MQTT_COMMAND_SUB                 2
#define MQTT_COMMAND_PUB                 3
#define MQTT_COMMAND_UNSUB               4
#define MQTT_COMMAND_END                 5


/*------------------
 * Message queue
 *----------------*/
#define MQTT_MAIN_QUEUE                "mqtt/main"
#define MQTT_MAIN_RESP_QUEUE           "mqtt/main_resp"


/*------------------
 * Task setting
 *----------------*/
#define MAIN_TASK_PRIORITY              100
#define BUF_SIZE                        2048


/****************************************************************************
 * Private Data
 ****************************************************************************/
typedef struct {
  int command;
  int num_params;
  char *params[16];
} command_params_t;


/* For MQTT valiables */
unsigned char sendbuf[BDKCONNECT_BUF_SIZE];
unsigned char recvbuf[BDKCONNECT_BUF_SIZE];
MQTTSocket sock;
MQTTClient cli;

char buf[BUF_SIZE];

static int com_status = 0;
static int com_id = 0;
static LTECommand command;
static LTECommand_System command_sys;
static LTECommand_System response_sys;


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
      usleep(60*1000);
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

static void lte_setup(void)
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
  printf("LT_Start() done.\n");

  /* Set callback */
  LT_SetCallbackSystem(&app_callback_sys, &response_sys);

  sleep(3);
  /* Send system connect command */
  command_sys.header.code = LTE_COMMAND_SYSTEM_CONNECT;
  command_sys.header.id = com_id++;
  send_recv_sys(&command_sys);
  printf("LTE Network connection started.\n");
}


static void messageArrived(MessageData* md)
{
  MQTTString* topic = md->topicName;

  printf("callback function in sample application\n");

  printf("topic_length : %d\n", topic->lenstring.len);
  printf("receive_data : %s\n", topic->lenstring.data);
}


void mqtt_main_task(void)
{
  MQTTPacket_connectData option = MQTTPacket_connectData_initializer;
  MQTTMessage msg;

  command_params_t *p;
  unsigned long value = 0;
  struct mq_attr mq_attr_main, mq_attr_resp;
  mode_t mode = 0666;
  mqd_t mqd_main, mqd_main_resp;

  printf("mqtt_main_task starts\n");

  lte_setup();

  mq_attr_main.mq_maxmsg  = 5;
  mq_attr_main.mq_msgsize = sizeof(command_params_t *);
  mq_attr_main.mq_flags   = 0;

  mqd_main=mq_open( MQTT_MAIN_QUEUE     , O_RDONLY | O_CREAT, mode, &mq_attr_main);

  mq_attr_resp.mq_maxmsg  = 1;
  mq_attr_resp.mq_msgsize = sizeof(value);
  mq_attr_resp.mq_flags   = 0;

  mqd_main_resp=mq_open( MQTT_MAIN_RESP_QUEUE, O_WRONLY | O_CREAT, mode, &mq_attr_resp);

  /* Network socket connect */
  int ret = -1;

  for (;;)
    {
      mq_receive( mqd_main, (char *)&p, sizeof(p), NULL);
      if (p != NULL)
        {
          switch (p->command)
            {
              case MQTT_COMMAND_CONNECT:
                printf("COMMAND_CONNECT\n");

                /* Setting MQTT */
                if( atoi(p->params[2]) == 1 )
                  {
                    sock.pRootCALocation = BDKCONNECT_ROOTCA;
                    sock.pDeviceCertLocation = BDKCONNECT_DEV_CERT;
                    sock.pDevicePrivateKeyLocation = BDKCONNECT_PRI_KEY;
                  }

                NT_MQTTSocketInit(&sock, atoi(p->params[2]));

                NT_MQTTClientInit(&cli, &sock, 500, sendbuf, BDKCONNECT_BUF_SIZE, recvbuf, BDKCONNECT_BUF_SIZE);
                printf("MQTT initialized.\n");

                /* Network socket connect */
                ret = -1;
                while (ret < 0)
                  {
                    ret = NT_MQTTSocketConnect(&sock, BDKCONNECT_BROKER, atoi(p->params[3]));

                    if (ret < 0)
                      {
                        printf("NT_MQTTSocketConnect error : %d\n", ret);
                        usleep(500*1000);
                      }
                  }

                printf("Network socket connected.\n");

                /* MQTT connect */
                option.willFlag = 0;
                option.MQTTVersion = 3;
                option.clientID.cstring = BDKCONNECT_CLIENT_ID;
                option.keepAliveInterval = 0;
                option.cleansession = 1;
                ret = NT_MQTTConnect(&cli, &option);
                if (ret < 0)
                  {
                    printf("NT_MQTTConnect error : %d\n", ret);
                    NT_MQTTSocketDisconnect(&sock);
                  }

                break;

              case MQTT_COMMAND_SUB:
                printf("COMMAND_SUB\n");
                if( p->num_params >= 4 )
                  {
                    ret=NT_MQTTSubscribe(&cli, p->params[2], atoi(p->params[3]), messageArrived );
                  }
                else
                  {
                    ret=NT_MQTTSubscribe(&cli, p->params[2], QOS0, messageArrived );

                  }

                if(ret < 0)
                  {
                    printf("NT_MQTTSubscribe error : %d\n", ret);
                  }
                break;

              case MQTT_COMMAND_PUB:
                printf("COMMAND_PUB\n");

                /* Initialize message area */
                memset( &msg, 0, sizeof(msg));

                ret = NT_MQTTYield(&cli, 1000);
                if (ret < 0)
                  {
                    printf("MQTT yield failed : %d\n", ret);
                    break;
                  }

                if( p->num_params >=5 )
                  {
                    msg.qos = atoi( p->params[4] );
                  }
                else
                  {
                    msg.qos = QOS0;
                  }
                msg.payload = (void *)p->params[3];
                msg.payloadlen = strlen(p->params[3]) + 1;


                ret=NT_MQTTPublish(&cli, p->params[2], &msg );
                if(ret < 0)
                  {
                    printf("NT_MQTTPublish error : %d\n", ret);
                  }

                ret = NT_MQTTYield(&cli, 1000);
                if (ret < 0)
                  {
                    printf("MQTT yield failed : %d\n", ret);
                    break;
                  }

                break;

              case MQTT_COMMAND_UNSUB:
                printf("COMMAND_UNSUB\n");
                ret=NT_MQTTUnsubscribe(&cli, p->params[2] );
                if(ret < 0)
                  {
                    printf("NT_MQTTUnSubscribe error : %d\n", ret);
                  }
                break;

              case MQTT_COMMAND_END:
                printf("COMMAND_END\n");
                /* MQTT/socket disconnect */
                NT_MQTTDisconnect(&cli);
                NT_MQTTSocketDisconnect(&sock);

                mq_close(mqd_main);
                mq_close(mqd_main_resp);
                break;

              default:
                printf("Unknown command\n");
                continue;
            }
            mq_send( mqd_main_resp, (const char *)&value, sizeof(value), 0U);
        }
    }
}


static void post_to_main_task(int cmd, int argc, const char *argv[])
{
  command_params_t *p = malloc(sizeof(command_params_t));
  unsigned long value;
  int i;
  mqd_t mqd_main;
  mqd_t mqd_main_resp;

  p->command = cmd;
  p->num_params = (argc < 16) ? argc : 16;
  for (i = 0; i < 16; i++)
    {
      if (i < argc)
        {
          p->params[i] = malloc(strlen(argv[i]) + 1);
          strcpy(p->params[i] ,argv[i]);
        }
      else
        {
          p->params[i] = NULL;
        }
    }

  mqd_main=mq_open( MQTT_MAIN_QUEUE     , O_WRONLY);
  mqd_main_resp=mq_open( MQTT_MAIN_RESP_QUEUE, O_RDONLY);


  mq_send( mqd_main, (const char *)&p, sizeof(p), 0U);

  // Wait until command has finished
  mq_receive( mqd_main_resp, (char *)&value, sizeof(value), NULL);

  for (i = 0; i < 16; i++)
    {
      if (p->params[i] != NULL)
        {
          free(p->params[i]);
        }
    }
  free(p);
  mq_close(mqd_main);
  mq_close(mqd_main_resp);
}


#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int mqtt_main(int argc, char *argv[])
#endif
{
  if( strncmp(argv[1], "init",4 )==0 )
    {

      task_create("mqtt_main_task", MAIN_TASK_PRIORITY, 5000, (main_t)mqtt_main_task, NULL );
    }
  else
    {
      if( strncmp( argv[1], "connect", 7 ) == 0 )
        {
          printf("Connect\n");
          post_to_main_task(MQTT_COMMAND_CONNECT, argc, (const char **)argv);
        }
      else if( strncmp( argv[1], "sub", 3 ) == 0 )
        {
          printf("Subscribe\n");
          post_to_main_task(MQTT_COMMAND_SUB, argc, (const char **)argv);
        }
      else if( strncmp( argv[1], "pub", 3 ) == 0 )
        {
          printf("Publish\n");
          post_to_main_task(MQTT_COMMAND_PUB, argc, (const char **)argv);
        }
      else if( strncmp( argv[1], "unsub", 5 ) == 0 )
        {
          printf("Unsubscribe\n");
          post_to_main_task(MQTT_COMMAND_UNSUB, argc, (const char **)argv);
        }
      else if( strncmp( argv[1], "end", 3 ) == 0 )
        {
          printf("MQTT test finished\n");
          post_to_main_task(MQTT_COMMAND_END, argc, (const char **)argv);
        }
      else
        {
          printf("Parameter Error!!\n");
        }
    }

  return 0;
}
