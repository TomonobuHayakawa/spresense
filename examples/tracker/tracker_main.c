/****************************************************************************
 * examples/tracker/tracker_main.c
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

#include <sdk/config.h>

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <fcntl.h>
#include <errno.h>

#include <netutils/httpc/http_socket.h>
#include <netutils/httpc/tls_socket.h>

#include <netutils/json/cJSON.h>

#include "tracker_net_client.h"
#include "tracker_location.h"
#include "tracker_button.h"
#include "tracker_sensing_tap.h"
#ifdef CONFIG_EXAMPLES_TRACKER_USING_NXFB

#include "tracker_nxfb.h"

#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_TRACKER_USING_EVENT_TRIGGER
#  if (!defined(CONFIG_EXAMPLES_TRACKER_TAP_TRIGGER)) && (!defined(CONFIG_EXAMPLES_TRACKER_BUTTON_TRIGGER))
#    error "CONFIG_EXAMPLES_TRACKER_TAP_TRIGGER or CONFIG_EXAMPLES_TRACKER_BUTTON_TRIGGER is not defined in the configuration"
#  endif
#endif

#define DOUBLE_STR_LEN         64
#define STR_MAXLEN             64
#define LOCATION_PUB_CYCLE     1    /* sec */
#define LOCK(lock) \
  do \
    { \
      int r; \
      r = sem_wait(&lock); \
      if (r == 0) \
        { \
          break; \
        } \
    } \
   while(1)
#define UNLOCK(lock)           do { sem_post(&lock); } while(0)
#define POSI_LOCK()            LOCK(g_tracker_posi_lock)
#define POSI_UNLOCK()          UNLOCK(g_tracker_posi_lock)

#ifdef CONFIG_EXAMPLES_TRACKER_USING_NXFB
#  define write_display(format, ...) \
  do \
    { \
        int r; \
        r = tracker_nxfb_initialize(); \
        if (r < 0) \
          { \
            printf("nxfb initialzie error:%d,%d\n", r, errno); \
          } \
        else \
          { \
            tracker_nxfb_printf(format, ##__VA_ARGS__); \
            tracker_nxfb_flash(); \
            tracker_nxfb_terminate(); \
          } \
    } \
  while(0)
#else
#  define write_display(format, ...)
#endif /* CONFIG_EXAMPLES_TRACKER_USING_NXFB */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_use_https = 0;
static bool pos_fix_flag = false;
static struct loc_mng_pos_data g_pos_data = {0};
static char g_phoneNo[16];
static int g_http_header_num = 0;
static const char *g_http_headers[CONFIG_EXAMPLES_TRACKER_HTTP_CUSTOM_HEADER_NUM];
static sem_t g_tracker_posi_lock;

#ifdef CONFIG_EXAMPLES_TRACKER_USING_EVENT_TRIGGER

static sem_t g_sem;

#endif /* CONFIG_EXAMPLES_TRACKER_USING_EVENT_TRIGGER */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(FAR const char *progname, int errcode)
{
  fprintf(stderr, "USAGE: %s [OPTIONS] <host> <port> <path>\n", progname);
  fprintf(stderr, "\nWhere:\n");
  fprintf(stderr, "\t<host> is the destination server address\n");
  fprintf(stderr, "\t<port> is the destination port number\n");
  fprintf(stderr, "\t<path> is the file path\n");
  fprintf(stderr, "\nand OPTIONS include the following:\n");
  fprintf(stderr, "\t-s: Using HTTPS.  Default: HTTP\n");
  fprintf(stderr, "\t-H <LINE>: Pass custom header LINE to server\n");
  fprintf(stderr, "\t-h: Show this text and exit\n");
  exit(errcode);
}

#ifdef CONFIG_EXAMPLES_TRACKER_TAP_TRIGGER

static void tap_data_cb(int tap_cnt)
{
  bool flag;

  printf("tap_cnt = %d\n", tap_cnt);

  if (tap_cnt > 1)
    {
      POSI_LOCK();
      flag = pos_fix_flag;
      POSI_UNLOCK();

      if (flag == true)
        {
          sem_post(&g_sem);
        }
    }
}

#endif /* CONFIG_EXAMPLES_TRACKER_TAP_TRIGGER */

#ifdef CONFIG_EXAMPLES_TRACKER_BUTTON_TRIGGER

static void button_handler(void)
{
  bool flag;

  POSI_LOCK();
  flag = pos_fix_flag;
  POSI_UNLOCK();

  if (flag == true)
    {
      sem_post(&g_sem);
    }
}

#endif /* CONFIG_EXAMPLES_TRACKER_BUTTON_TRIGGER */

static void location_publish(struct loc_mng_pos_data *pos_data)
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
          ,pos_data->altitude);

  write_display("LAT:%.06f\nLONG:%.06f\nALT:%.06f"
                ,pos_data->latitude
                ,pos_data->longitude
                ,pos_data->altitude);

  POSI_LOCK();

  pos_fix_flag = true;
  memcpy(&g_pos_data, pos_data, sizeof(g_pos_data));

  POSI_UNLOCK();
}

static void do_tracker(char* domain, int port, char* path)
{
  cJSON *root;
  char *out = NULL;
  char double_str[DOUBLE_STR_LEN];
  int ret;
#ifndef CONFIG_EXAMPLES_TRACKER_USING_EVENT_TRIGGER
  bool flag;
#endif

  ret = sem_init(&g_tracker_posi_lock ,0 ,1);
  if(ret == ERROR)
    {
      perror("sem_init()");

      goto _err;
    }

#ifdef CONFIG_EXAMPLES_TRACKER_USING_EVENT_TRIGGER

  ret = sem_init(&g_sem, 0, 0);
  if(ret == ERROR)
    {
      perror("sem_init()");
      sem_destroy(&g_tracker_posi_lock);

      goto _err;
    }

#endif /* CONFIG_EXAMPLES_TRACKER_USING_EVENT_TRIGGER */

#ifdef CONFIG_EXAMPLES_TRACKER_BUTTON_TRIGGER

  /* register_button_handler */

  register_button_handler(0, button_handler);

#endif /* CONFIG_EXAMPLES_TRACKER_BUTTON_TRIGGER */

#ifdef CONFIG_EXAMPLES_TRACKER_TAP_TRIGGER

  /* Tap start */

  sensing_tap_start(tap_data_cb);

#endif /* CONFIG_EXAMPLES_TRACKER_TAP_TRIGGER */

  write_display("Waiting for\nConnect LTE");

  /* Connect to the LTE network */

  lte_setup();

  write_display("\nLTE Connected");

  lte_get_phone_number(g_phoneNo);

  NT_tls_socket_init();
  NT_http_init();

  write_display("Waiting for\nfix position");

  /* Location start */

  location_start(LOCATION_PUB_CYCLE, location_publish);

  for(;;)
    {
#ifdef CONFIG_EXAMPLES_TRACKER_USING_EVENT_TRIGGER

      /* Wait event trigger */

      ret = sem_wait(&g_sem);
      if(ret == ERROR)
        {
          if(errno != EINTR)
            {
              perror("sem_wait()");
            }
          continue;
        }

#else /* CONFIG_EXAMPLES_TRACKER_USING_EVENT_TRIGGER */

      sleep(10);

      POSI_LOCK();
      flag = pos_fix_flag;
      POSI_UNLOCK();

      if (flag == false)
        {
          continue;
        }

#endif /* CONFIG_EXAMPLES_TRACKER_USING_EVENT_TRIGGER */

      POSI_LOCK();

      /* Construct data using JSON format */

      root = cJSON_CreateObject();
      memset(double_str, 0x00, DOUBLE_STR_LEN);
      snprintf(double_str, DOUBLE_STR_LEN, "%.06f", g_pos_data.latitude);
      cJSON_AddStringToObject(root, "latitude", double_str);
      memset(double_str, 0x00, DOUBLE_STR_LEN);
      snprintf(double_str, DOUBLE_STR_LEN, "%.06f", g_pos_data.longitude);
      cJSON_AddStringToObject(root, "longitude", double_str);
      cJSON_AddStringToObject(root, "phoneNo", (const char*)g_phoneNo);
      out = cJSON_Print(root);

      POSI_UNLOCK();

      printf("%s\n", out);

      /* Send HTTP request to server using POST method */

      http_post_request(g_use_https, domain, port, path,
                        g_http_headers, g_http_header_num,
                        out, strlen(out));

      /* Delete JSON object */

      cJSON_Delete(root);
      free(out);
    }

  sem_destroy(&g_tracker_posi_lock);

#ifdef CONFIG_EXAMPLES_TRACKER_USING_EVENT_TRIGGER

  sem_destroy(&g_sem);

#endif

_err:
  return;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int tracker_main(int argc, char *argv[])
#endif
{
  char *host;
  char *path;
  int port;
  int option;

  /* Parse input parameters */

  while ((option = getopt(argc, argv, "shH:")) != ERROR)
    {
      switch (option)
        {
          case 's':
            g_use_https = 1;
            break;

          case 'h':
            show_usage(argv[0], EXIT_SUCCESS);
            break;

          case 'H':
            if ((g_http_header_num + 1) >= CONFIG_EXAMPLES_TRACKER_HTTP_CUSTOM_HEADER_NUM)
              {
                fprintf(stderr, "ERROR: Too many headers\n");
                show_usage(argv[0], EXIT_FAILURE);
              }
            g_http_headers[g_http_header_num] = optarg;
            g_http_header_num++;
            break;

          case ':':
            fprintf(stderr, "ERROR: Missing required argument\n");
            show_usage(argv[0], EXIT_FAILURE);
            break;

          default:
          case '?':
            fprintf(stderr, "ERROR: Unrecognized option\n");
            show_usage(argv[0], EXIT_FAILURE);
            break;
        }
    }

  /* There should be three final parameters remaining on the command line */

  if ((optind + 2) >= argc)
    {
      printf("ERROR: Missing required 'host' or 'port' or 'path' argument\n");
      show_usage(argv[0], EXIT_FAILURE);
    }

  host = argv[optind++];
  port = atoi(argv[optind++]);
  path = argv[optind++];

  /* Start tracker */

  do_tracker(host, port, path);

  return 0;
}
