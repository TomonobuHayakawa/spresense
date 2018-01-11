/****************************************************************************
 * examples/tracker/tracker_sensing_tap.cxx
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
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>

#include <sensing/tap_manager.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_ctl_id = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void tap_init(void)
{
  TapMngInit();
}

static int tap_start(tap_mng_out_cbs tap_detect_cb)
{
  int ret = 0;
  struct tap_mng_start_param sta_prm = {0};

  /* Set start parameter */

  sta_prm.tap_prm.tap_period = 500000;  /* Validity period to detect the tap series(usec) */
  sta_prm.tap_prm.peak_thres = 1.10;    /* The minimum value of vibration
                                         * tapping shall be deemed. range 0.0 - 4.0 (G)
                                         */
  sta_prm.tap_prm.long_thres = 0.3;     /* The maximum vibration indicates 
                                         * that vibration of the tap. range 0.0 - 4.0 (G)
                                         */
  sta_prm.tap_prm.stab_frame = 5;       /* time of PEAK_THRES -> LONG_THRES. 
                                         * range 0 - 32 (64Hz frame num)
                                         */
  sta_prm.cbs = (tap_mng_out_cbs)tap_detect_cb;

  /* start sensing tap */

  ret = TapMngStart(&sta_prm, &g_ctl_id);

  return ret;
}

extern "C"
{
  int sensing_tap_start(void (*tap_detect_cb)(int))
  {
    int ret;

    tap_init();

    ret = tap_start(tap_detect_cb);
    if (ret < 0)
      {
        printf("sensing tap failed:%d\n", ret);
      }

    return ret;
  }
}
