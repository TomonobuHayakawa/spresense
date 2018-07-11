/****************************************************************************
 * examples/fft/dsp_rpc.c
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

#include <sdk/config.h>

#include <stdio.h>

#include <asmp/mptask.h>
#include <asmp/mpmq.h>

#include "arm_math.h"
#include "dsp_rpc.h"
#include "resource.h"

#ifndef CONFIG_EXAMPLES_FFT_DISABLE_MATH_OFFLOAD

#define errmsg(x, ...) fprintf(stderr, x, ## __VA_ARGS__)

#define ARGVAL(v) (uint32_t)(v)
#define ARGPTR(p) ((uint32_t)(uintptr_t)(p))

static mptask_t g_dspmath = {0};
static mpmq_t   g_dspmq = {0};
static uint32_t g_buffer[16];

/* Interface for DSP math library is defined like this:
 *
 * args[0] = Function ID (hash)
 * args[1-15] = Arguments for each functions
 *
 * And return value will be sent at message data directory.
 *
 * DSP RPC is not thread safe, but this example use this in single task.
 * So user must be add exclusive processing if user want to use from multiple
 * tasks.
 */

static int dsp_rpc(void *args)
{
  int ret;
  uint32_t data;

  data = (uint32_t)(uintptr_t)args;

  /* Send RPC message to DSP */

  ret = mpmq_send(&g_dspmq, DSP_RPC_MSG, data);
  if (ret < 0)
    {
      errmsg("mpmq_send() failure. %d\n", ret);
      return ret;
    }

  /* Wait for DSP math function is done */

  ret = mpmq_receive(&g_dspmq, &data);
  if (ret < 0)
    {
      errmsg("mpmq_recieve() failure. %d\n", ret);
      return ret;
    }

  return (int)data;
}

/* Following APIs are wrapper for CMSIS DSP Math library.
 * CMSIS DSP library is huge (about 800KB), so we need to choose them
 * actually used, for binary size into small.
 */

void arm_fill_f32(float32_t value, float32_t * pDst, uint32_t blockSize)
{
  uint32_t *args = g_buffer;

  args[0] = DSP_FILL_F32;
  args[1] = ARGVAL(value);
  args[2] = ARGPTR(pDst);
  args[3] = ARGVAL(blockSize);

  (void)dsp_rpc(args);
}

void arm_copy_f32(float32_t * pSrc, float32_t * pDst, uint32_t blockSize)
{
  uint32_t *args = g_buffer;

  args[0] = DSP_COPY_F32;
  args[1] = ARGPTR(pSrc);
  args[2] = ARGPTR(pDst);
  args[3] = ARGVAL(blockSize);

  (void)dsp_rpc(args);
}

arm_status arm_cfft_radix4_init_f32(arm_cfft_radix4_instance_f32 * S,
                                    uint16_t fftLen, uint8_t ifftFlag,
                                    uint8_t bitReverseFlag)
{
  uint32_t *args = g_buffer;

  args[0] = DSP_CFFT_RADIX4_INIT_F32;
  args[1] = ARGPTR(S);
  args[2] = ARGVAL(fftLen);
  args[3] = ARGVAL(ifftFlag);
  args[4] = ARGVAL(bitReverseFlag);

  return (arm_status)dsp_rpc(args);
}

void arm_cfft_radix4_f32(const arm_cfft_radix4_instance_f32 * S,
                                   float32_t * pSrc)
{
  uint32_t *args = g_buffer;

  args[0] = DSP_CFFT_RADIX4_F32;
  args[1] = ARGPTR(S);
  args[2] = ARGPTR(pSrc);

  (void)dsp_rpc(args);
}

void arm_cmplx_mult_cmplx_f32(float32_t * pSrcA, float32_t * pSrcB,
                              float32_t * pDst, uint32_t numSamples)
{
  uint32_t *args = g_buffer;

  args[0] = DSP_CMPLX_MULT_CMPLX_F32;
  args[1] = ARGPTR(pSrcA);
  args[2] = ARGPTR(pSrcB);
  args[3] = ARGPTR(pDst);
  args[4] = ARGVAL(numSamples);

  (void)dsp_rpc(args);
}

int load_library(const char *filename)
{
  int ret;

  memset(g_buffer, 0, sizeof(g_buffer));

  /* Initialize DSP Math library */

  ret = mptask_init(&g_dspmath, filename);
  if (ret != 0)
    {
      errmsg("mptask_init() failure. %d\n", ret);
      return ret;
    }

  ret = mptask_assign(&g_dspmath);
  if (ret != 0)
    {
      errmsg("mptask_asign() failure. %d\n", ret);
      return ret;
    }

  /* Initialize MP message queue with asigned CPU ID, and bind it to MP task */

  ret = mpmq_init(&g_dspmq, DSP_MQID, mptask_getcpuid(&g_dspmath));
  if (ret < 0)
    {
      errmsg("mpmq_init() failure. %d\n", ret);
      return ret;
    }
  ret = mptask_bindobj(&g_dspmath, &g_dspmq);
  if (ret < 0)
    {
      errmsg("mptask_bindobj(mq) failure. %d\n", ret);
      return ret;
    }

  ret = mptask_exec(&g_dspmath);
  if (ret < 0)
    {
      errmsg("mptask_exec() failure. %d\n", ret);
      return ret;
    }

  return 0;
}

void unload_library(void)
{
  /* Send quit request to DSPMATH */

  mpmq_send(&g_dspmq, DSP_RPC_UNLOAD, 0);

  /* Destroy DSPMATH and successfully done. */

  (void) mptask_destroy(&g_dspmath, false, NULL);
  mpmq_destroy(&g_dspmq);
}

#endif /* CONFIG_DISABLE_MATH_OFFLOAD */
