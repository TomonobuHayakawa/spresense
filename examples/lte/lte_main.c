/****************************************************************************
 * lte/lte_main.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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
#include <mqueue.h>
#include <fcntl.h>
#include <errno.h>

#include "lte/lte_api.h"
#include "netutils/webclient.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APP_MQUEUE_NAME    "lte_sample_queue"
#define APP_MAX_MQUEUE_MSG 1
#define APP_MQUEUE_MODE    0666
#define APP_IOBUFFER_LEN   512

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_app_iobuffer[APP_IOBUFFER_LEN];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: app_init
 ****************************************************************************/

static int app_init(void)
{
  int errcode;
  mqd_t mqd;
  struct mq_attr mq_attr;

  mq_attr.mq_maxmsg  = APP_MAX_MQUEUE_MSG;
  mq_attr.mq_msgsize = sizeof(int);
  mq_attr.mq_flags   = 0;

  /* Create message queue resource */

  mqd = mq_open(APP_MQUEUE_NAME, (O_RDWR | O_CREAT), APP_MQUEUE_MODE,
                &mq_attr);
  if (mqd < 0)
    {
      errcode = errno;
      printf("mq_open() failed: %d\n", errcode);
      return -1;
    }
  mq_close(mqd);

  return 0;
}

/****************************************************************************
 * Name: app_fin
 ****************************************************************************/

static void app_fin(void)
{
  mq_unlink(APP_MQUEUE_NAME);
}

/****************************************************************************
 * Name: app_notify_response
 ****************************************************************************/

static void app_notify_response(int response)
{
  int ret;
  mqd_t mqd;
  int errcode;
  int buffer = response;

  /* Open message queue for send */

  mqd = mq_open(APP_MQUEUE_NAME, O_WRONLY);
  if (mqd < 0)
    {
      errcode = errno;
      printf("mq_open() failed: %d\n", errcode);
      return;
    }

  /* Send response */

  ret = mq_send(mqd, (FAR const char*)&buffer, sizeof(buffer), 0);
  if (ret < 0)
    {
      errcode = errno;
      printf("mq_send() failed: %d\n", errcode);
      mq_close(mqd);
      return;
    }
  mq_close(mqd);
}

/****************************************************************************
 * Name: app_wait_response
 ****************************************************************************/

static int app_wait_response(int *response)
{
  int ret;
  mqd_t mqd;
  int errcode;
  int resp;

  /* Open message queue for receive */

  mqd = mq_open(APP_MQUEUE_NAME, O_RDONLY);
  if (mqd < 0)
    {
      errcode = errno;
      printf("mq_open() failed: %d\n", errcode);
      return -1;
    }

  /* Receive response */

  ret = mq_receive(mqd, (FAR char*)&resp, sizeof(resp), 0);
  if (ret < 0)
    {
      errcode = errno;
      printf("mq_send() failed: %d\n", errcode);
      mq_close(mqd);
      return -1;
    }
  mq_close(mqd);

  *response = resp;

  return 0;
}

/****************************************************************************
 * Name: app_poweron_cb
 ****************************************************************************/

static void app_poweron_cb(uint32_t result)
{
  printf("%s called\n", __func__);

  app_notify_response(result);
}

/****************************************************************************
 * Name: app_poweroff_cb
 ****************************************************************************/

static void app_poweroff_cb(uint32_t result)
{
  printf("%s called\n", __func__);

  app_notify_response(result);
}

/****************************************************************************
 * Name: app_attach_net_cb
 ****************************************************************************/

static void app_attach_net_cb(uint32_t result, uint32_t errcause)
{
  if (LTE_RESULT_ERROR == result)
    {
      printf("%s called: result:%d errorcause:%d\n",
             __func__, result, errcause);
    }
  else
    {
      printf("%s called: result:%d\n", __func__, result);
    }

  app_notify_response(result);
}

/****************************************************************************
 * Name: app_detach_net_cb
 ****************************************************************************/

static void app_detach_net_cb(uint32_t result)
{
  printf("%s called: result:%d\n", __func__, result);

  app_notify_response(result);
}

/****************************************************************************
 * Name: app_wget_cb
 ****************************************************************************/

static void app_wget_cb(FAR char **buffer, int offset, int datend,
                        FAR int *buflen, FAR void *arg)
{
  (void)write(1, &((*buffer)[offset]), datend - offset);
}

/****************************************************************************
 * Name: app_lte_init
 ****************************************************************************/

static int app_lte_init(void)
{
  int ret;
  int response = LTE_RESULT_OK;

  /* Initialize the LTE library */

  ret = lte_initialize();
  if (ret < 0)
    {
      printf("Failed to initialize LTE library :%d\n", ret);
      goto errout;
    }

  /* Power on the modem
   * If it succeeds, it will be able to accept requests from all APIs */

  ret = lte_power_control(LTE_POWERON, app_poweron_cb);
  if (ret < 0)
    {
      printf("Failed to power on the modem :%d\n", ret);
      goto errout_with_fin;
    }

  /* Wait until the modem startup completed */

  ret = app_wait_response(&response);
  if ((ret < 0) || (response != LTE_RESULT_OK))
    {
      goto errout_with_fin;
    }

  return 0;

errout_with_fin:
  lte_finalize();

errout:
  return ret;
}

/****************************************************************************
 * Name: app_lte_fin
 ****************************************************************************/

static int app_lte_fin(void)
{
  int ret;
  int response = LTE_RESULT_OK;

  /* Power off the modem
   * If it succeeds, it will be not able to accept requests from all APIs */

  ret = lte_power_control(LTE_POWEROFF, app_poweroff_cb);
  if (ret < 0)
    {
      printf("Failed to power off the modem :%d\n", ret);
      goto errout_with_fin;
    }

  /* Wait until the modem shutdown completed */

  ret = app_wait_response(&response);
  if ((ret < 0) || (response != LTE_RESULT_OK))
    {
      goto errout_with_fin;
    }

  /* Finalize LTE library */

  ret = lte_finalize();
  if (ret < 0)
    {
      printf("Failed to finalize LTE library :%d\n", ret);
      goto errout;
    }

  return 0;

errout_with_fin:
  lte_finalize();

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * lte_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int lte_main(int argc, char *argv[])
#endif
{
  int  ret;
  int  response = LTE_RESULT_OK;
  char *url     = NULL;

  if (argc > 1)
    {
      /* The URL of the file to get */

      url = argv[1];
    }

  /* Initialize the this application */

  ret = app_init();
  if (ret < 0)
    {
      goto errout;
    }

  /* Initialize the LTE */

  ret = app_lte_init();
  if (ret < 0)
    {
      goto errout_with_fin;
    }

  /* Attach to LTE network */

  ret = lte_attach_network(app_attach_net_cb);
  if (ret < 0)
    {
      printf("Failed to attach to LTE network :%d\n", ret);
      goto errout_with_lte_fin;
    }

  /* Wait until attach to LTE network to be completed */

  ret = app_wait_response(&response);
  if ((ret < 0) || (response == LTE_RESULT_ERROR))
    {
      goto errout_with_lte_fin;
    }

  if (url != NULL)
    {
      wget(url, g_app_iobuffer, APP_IOBUFFER_LEN, app_wget_cb, NULL);
    }

  /* Detach to LTE network */

  ret = lte_detach_network(app_detach_net_cb);
  if (ret < 0)
    {
      printf("Failed to detach from LTE network :%d\n", ret);
      goto errout_with_lte_fin;
    }

  /* Wait until detach from LTE network to be completed */

  ret = app_wait_response(&response);
  if ((ret < 0) || (response == LTE_RESULT_ERROR))
    {
      goto errout_with_lte_fin;
    }

  /* Finalize the LTE */

  ret = app_lte_fin();
  if (ret < 0)
    {
      goto errout_with_fin;
    }

  /* Finalize the this application */

  app_fin();

  return 0;

errout_with_lte_fin:
  app_lte_fin();

errout_with_fin:
  app_fin();

errout:
  return -1;
}
