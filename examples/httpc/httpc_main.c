/****************************************************************************
 * examples/httpc/httpc_main.c
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

#include "netutils/httpc/http_socket.h"
#include "netutils/httpc/tls_socket.h"

#include "netutils/mbedtls/ssl.h"
#include "netutils/mbedtls/certs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define COMMAND_WGET                    1
#define COMMAND_WGETC                   2
#define COMMAND_MEMTEST                 3
#define COMMAND_CLOSE                   4
#define COMMAND_ENABLE                  5
#define COMMAND_DISABLE                 6
#define COMMAND_CERT                    7
#define COMMAND_END                     8

#define HTTPC_MAIN_QUEUE                "httpc/main"
#define HTTPC_MAIN_RESP_QUEUE           "httpc/main_resp"

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
      usleep(10*1000);
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

static char line[128];

static size_t print_lines(const char *buffer)
{
  const char *p = buffer;
  const char *s;
  size_t n;

  while(1)
    {
      s = strchr(p, '\n');
      if (s == NULL)
        {
          n = strlen(p);
          memcpy(line, p, n);
          line[n] = '\0';
          printf("%s", line);
          break;
        }
      n = s - p;
      memcpy(line, p, n);
      line[n] = '\0';
      p = s + 1;
      printf("%s", line);
    }
  return (p - buffer);
}


static int http_socket = -1;
struct http_chunk_client chunk_data_static;

void chunked_recv(struct http_chunk_client *chunk_data)
{
  if (chunk_data != NULL)
    {
      chunk_data->chunk_buf[chunk_data->chunk_readlen] = '\0';

      printf("%s", chunk_data->chunk_buf);
      if(chunk_data->chunk_size == 0)
        {
          NT_http_close(http_socket);
          http_socket = -1;
        }
    }
}


static void http_request(int show_content, int use_https, char *domain, int port, const char *path)
{
  int r;
  int status;
  size_t n = 0;

  if (http_socket < 0)
    {
      http_socket = NT_http_create(use_https);
      printf("HTTP socket created: %d\n", http_socket);
      if (http_socket < 0)
        {
          return;
        }

      if (NT_http_connect(http_socket, domain, port) < 0)
        {
          printf("HTTP socket connect failed\n");
          NT_http_close(http_socket);
          http_socket = -1;
          return;
        }
    }
  else
    {
      printf("HTTP running process: %d\n", http_socket);
      return;
    }

  NT_http_socket_recvtimeout(http_socket, 10);

  r = NT_http_get(http_socket, path, NULL, 0, &status);
  printf("HTTP socket get: %d\n", r);
  if (r < 0)
    {
      NT_http_close(http_socket);
      http_socket = -1;
      return;
    }

  printf("Response status code = %d\n", status);
  if(show_content)
    {
      chunk_data_static.chunk_buf = buf;
      chunk_data_static.chunk_buflen = BUF_SIZE;
      NT_http_add_listener(http_socket, &chunk_data_static, chunked_recv);
    }
  else
    {
      char *p = buf;
      for (;;) {
          r = NT_http_read_response(http_socket, p, BUF_SIZE);
          if (r <= 0) {
        break;
      }

      p[r] = '\0';
      if ((status >= 200) && (status < 300)) {
        printf("Received %4d/%d bytes\n", r, n);
      } else {
        print_lines(buf);
      }
      n += r;
    }
  }
  if (show_content)
    {
      printf("Received %d bytes\n", n);
    }
  else
    {
       NT_http_close(http_socket);
       http_socket = -1;
    }
}

static void do_close(void)
{
  if (http_socket >= 0)
    {
      NT_http_close(http_socket);
      http_socket = -1;
    }
}

static void do_wget(int show_content, int argc, char *argv[])
{
  int use_https = 0;
  char *domain;
  char *path;
  int port = 0;

  if (argc < 6)
    {
      printf("wget: missing parameter\n");
      return;
    }

  if (strcmp(argv[2], "https") == 0)
    {
      use_https = 1;
    }
  else if (strcmp(argv[2], "http"))
    {
      printf("wget: first parameter must be scheme (http or https)\n");
      return;
    }

  int domain_len = strlen(argv[3]);
  int path_len = strlen(argv[5]);;

  domain = malloc(domain_len + 1);
  strcpy(domain, argv[3]);

  port = atoi(&argv[4][0]);

  path = malloc(path_len + 1);
  strcpy(path, argv[5]);

  printf("URL = %s://%s:%d%s\n", (use_https ? "https" : "http"), domain, port, path);

  http_request(show_content, use_https, domain, port, path);

  free(domain);
  free(path);
}

static mbedtls_x509_crt cacert;
static void do_cert(int enable, int argc, char *argv[])
{
  int ret;
  int cnt;

  if (argc < 3)
    {
      printf("Missing parameter\n");
      return;
    }

  mbedtls_x509_crt_init( &cacert );

  for( cnt=2; cnt<argc; cnt++ )
    {
      ret = mbedtls_x509_crt_parse_file( &cacert, argv[cnt] );
      if( ret != 0 )
        {
          printf("Parse fail: %s, ret=%d\n", argv[cnt], ret);
          return;
        }
    }

  NT_tls_set_crt_info(&cacert);
}

static void do_memtest(void)
{
  void *p[100];
  size_t n = 0;
  size_t i,j;

  for (i = 0; i < 100; i++)
    {
      p[i] = malloc(1024);
      if (p[i] != NULL)
        {
          n += 1024;
          printf("allocated %d bytes\n", n);
        }
      else
        {
          printf("failed\n");
          for (j = 0; j < i; j++)
          free(p[j]);
          printf("Could allocate %d kB\n", i);
          break;
        }
    }
}

static void do_enable(int enable, int argc, char *argv[])
{
  if (argc < 3)
    {
      printf("Missing parameter\n");
      return;
    }
  if (strcmp(argv[2], "dnscache") == 0)
    {
    }
  else if (strcmp(argv[2], "sessioncache") == 0)
    {
      NT_tls_socket_session_cache_enable(enable);
    }
  else
    {
      printf("Unkown parameter: %s\n", argv[2]);
    }
}

void httpc_main_task(void)
{
  command_params_t *p;
  unsigned long value = 0;
  struct mq_attr mq_attr_main, mq_attr_resp;
  mode_t mode = 0666;
  mqd_t mqd_main, mqd_main_resp;

  printf("httpc_main_task starts\n");

  lte_setup();

  mq_attr_main.mq_maxmsg  = 5;
  mq_attr_main.mq_msgsize = sizeof(command_params_t *);
  mq_attr_main.mq_flags   = 0;

  mqd_main=mq_open( HTTPC_MAIN_QUEUE     , O_RDONLY | O_CREAT, mode, &mq_attr_main);

  mq_attr_resp.mq_maxmsg  = 1;
  mq_attr_resp.mq_msgsize = sizeof(value);
  mq_attr_resp.mq_flags   = 0;

  mqd_main_resp=mq_open( HTTPC_MAIN_RESP_QUEUE, O_WRONLY | O_CREAT, mode, &mq_attr_resp);

  NT_tls_socket_init();
  NT_http_init();

  for (;;)
    {
      mq_receive( mqd_main, (char *)&p, sizeof(p), NULL);
      if (p != NULL)
        {
          switch (p->command)
            {
              case COMMAND_WGET:
                printf("COMMAND_WGET\n");
                do_wget(0, p->num_params, p->params);
                break;

              case COMMAND_WGETC:
                printf("COMMAND_WGETC\n");
                do_wget(1, p->num_params, p->params);
                break;
              case COMMAND_CERT:
                printf("COMMAND_CERT\n");
                do_cert(1, p->num_params, p->params);
                break;

              case COMMAND_MEMTEST:
                printf("COMMAND_MEMTEST\n");
                do_memtest();
                break;

              case COMMAND_CLOSE:
                printf("COMMAND_CLOSE\n");
                do_close();
                break;

              case COMMAND_ENABLE:
                printf("COMMAND_ENABLE\n");
                do_enable(1, p->num_params, p->params);
                break;

              case COMMAND_DISABLE:
                printf("COMMAND_DISABLE\n");
                do_enable(0, p->num_params, p->params);
                break;

              case COMMAND_END:
                printf("COMMAND_END\n");
                mq_close(mqd_main);
                mq_close(mqd_main_resp);

                return;
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

  mqd_main=mq_open( HTTPC_MAIN_QUEUE     , O_WRONLY);
  mqd_main_resp=mq_open( HTTPC_MAIN_RESP_QUEUE, O_RDONLY);


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
int httpc_main(int argc, char *argv[])
#endif
{
  if( strncmp(argv[1], "init",4 )==0 )
    {

      task_create("httpc_main_task", MAIN_TASK_PRIORITY, 5000, (main_t)httpc_main_task, NULL );
    }
  else
    {
      if( strncmp( argv[1], "wget", 4 ) == 0 )
        {
          printf("Perform a http(s) GET\n");
          post_to_main_task(COMMAND_WGET, argc, (const char **)argv);
        }
      else if( strncmp( argv[1], "wgetc", 5 ) == 0 )
        {
          printf("Perform a http(s) GET\n");
          post_to_main_task(COMMAND_WGETC, argc, (const char **)argv);
        }
      else if( strncmp( argv[1], "scert", 5 ) == 0 )
        {
          printf("Change CA cert\n");
          post_to_main_task(COMMAND_CERT, argc, (const char **)argv);
        }
      else if( strncmp( argv[1], "close", 5 ) == 0 )
        {
          printf("Close HTTP socket\n");
          post_to_main_task(COMMAND_CLOSE, argc, (const char **)argv);
        }
      else if( strncmp( argv[1], "enable", 6 ) == 0 )
        {
          printf("Enable DNS/TLS cache\n");
          post_to_main_task(COMMAND_ENABLE, argc, (const char **)argv);
        }
      else if( strncmp( argv[1], "disable", 7 ) == 0 )
        {
          printf("Disable DNS/TLS cache\n");
          post_to_main_task(COMMAND_DISABLE, argc, (const char **)argv);
        }
      else if( strncmp( argv[1], "memtest", 7 ) == 0 )
        {
          printf("Test memory allocation\n");
          post_to_main_task(COMMAND_MEMTEST, argc, (const char **)argv);
        }
      else if( strncmp( argv[1], "end", 3 ) == 0 )
        {
          printf("HTTPC application finish\n");
          post_to_main_task(COMMAND_END, argc, (const char **)argv);
        }
      else
        {
          printf("Parameter Error!!\n");
        }
    }

  return 0;
}
