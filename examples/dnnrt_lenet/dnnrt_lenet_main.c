/****************************************************************************
 * dnnrt_lenet/dnnrt_lenet_main.c
 *
 *   Copyright 2018 Sony Corporation
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
 * 3. Neither the name of Sony Corporation nor the names of its contributors
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
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <nuttx/config.h>
#include <dnnrt/runtime.h>
#include "loader_nnb.h"
#include "pnm_util.h"

/****************************************************************************
 * Type Definition
 ****************************************************************************/
typedef struct
  {
    char *nnb_path;
    char *pnm_path;
  } my_setting_t;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define DNN_PNM_PATH    "/mnt/sd0/lenet-5/data/0.pgm"
#define DNN_NNB_PATH    "/mnt/sd0/lenet-5/model/lenet-5.nnb"
#define MNIST_SIZE_PX (28*28)

/****************************************************************************
 * Private Data
 ****************************************************************************/
static float s_img_buffer[MNIST_SIZE_PX];

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void convert_datatype(dnn_runtime_t * rt)
{
  nn_variable_t *var = dnn_runtime_input_variable(rt, 0);
  float coefficient = (float)(1 << var->fp_pos);

  if (var->type == NN_DATA_TYPE_FLOAT)
    {
      // do nothing
    }
  else if (var->type == NN_DATA_TYPE_INT16)
    {
      int16_t *int16_buffer = (int16_t *) s_img_buffer;
      for (uint16_t px = 0u; px < MNIST_SIZE_PX; px++)
        {
          int16_buffer[px] = (int16_t) (coefficient * s_img_buffer[px]);
        }
    }
  else
    {
      int8_t *int8_buffer = (int8_t *) s_img_buffer;
      for (uint16_t px = 0u; px < MNIST_SIZE_PX; px++)
        {
          int8_buffer[px] = (int8_t) (coefficient * s_img_buffer[px]);
        }
    }
}

static void parse_args(int argc, char *argv[], my_setting_t * setting)
{
  /* set my_setting_t::{nnb_path,pnm_path} to argv[] if necessary */
  setting->nnb_path = (argc >= 2) ? argv[1] : DNN_NNB_PATH;
  setting->pnm_path = (argc >= 3) ? argv[2] : DNN_PNM_PATH;
}

/****************************************************************************
 * dnnrt_lenet_main
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int dnnrt_lenet_main(int argc, char *argv[])
#endif
{
  int ret;
  unsigned char i;
  float *output_buffer, proc_time;
  const void *inputs[1] = { s_img_buffer };
  dnn_runtime_t rt;
  nn_network_t *network;
  my_setting_t setting = { 0 };
  struct timeval begin, end;

  parse_args(argc, argv, &setting);

  /* load an MNIST image into s_img_buffer and divide the pixels by 255 */
  printf("load pnm image: %s\n", setting.pnm_path);
  ret = pnm_load(setting.pnm_path, 255.0f, s_img_buffer);
  if (ret)
    {
      printf("load pnm image failed due to %d\n", ret);
      goto pnm_error;
    }

  /* load an nnb file, which holds a network structure and weight values, into
   * a heap memory */
  printf("load nnb file: %s\n", setting.nnb_path);
  network = alloc_nnb_network(setting.nnb_path);
  if (network == NULL)
    {
      printf("load nnb file failed\n");
      goto pnm_error;
    }

  /* initialize the dnnrt subsystem */
  ret = dnn_initialize(NULL);
  if (ret)
    {
      printf("dnn_initialize() failed due to %d", ret);
      goto dnn_error;
    }

  /* instantiate a runtime object, dnn_runtime_t, based on the above nnb file */
  ret = dnn_runtime_initialize(&rt, network);
  if (ret)
    {
      printf("dnn_runtime_initialize() failed due to %d\n", ret);
      goto rt_error;
    }

  /* convert the MNIST image's datatype in-place on s_img_buffer */
  convert_datatype(&rt);

  /* feed the MNIST image into dnn_runtime_t and classify it */
  printf("start dnn_runtime_forward()\n");
  gettimeofday(&begin, 0);
  ret = dnn_runtime_forward(&rt, inputs);
  gettimeofday(&end, 0);
  if (ret)
    {
      printf("dnn_runtime_forward() failed due to %d\n", ret);
      goto fin;
    }

  /* show the classification result and its processing time */
  output_buffer = dnn_runtime_output_buffer(&rt, 0u);
  for (i = 0u; i < 10u; i++)
    {
      printf("output[%u]=%.6f\n", i, output_buffer[i]);
    }
  proc_time = (float)end.tv_sec + (float)end.tv_usec / 1.0e6;
  proc_time -= (float)begin.tv_sec + (float)begin.tv_usec / 1.0e6;
  printf("inference time=%.3f\n", proc_time);

fin:
  dnn_runtime_finalize(&rt);
rt_error:
  dnn_finalize();
dnn_error:
  destroy_nnb_network(network);
pnm_error:
  return ret;
}
