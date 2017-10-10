/****************************************************************************
 * configs/cxd56xx/src/cxd56_bm1383glv.c
 *
 *   Copyright (C) 2016 Sony Corporation. All rights reserved.
 *   Author: Makoto Kabe <Makoto.Kabe@sony.com>
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
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>

#include <nuttx/sensors/bm1383glv.h>
#ifdef CONFIG_CXD56_SCU
#include <arch/chip/cxd56_scu.h>
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_BM1383GLV)

#ifdef CONFIG_CXD56_SCU
int cxd56_bm1383glvinitialize(FAR const char *devpath,
                              FAR struct i2c_master_s* i2c)
{
  int ret;

  sninfo("Initializing BM1383GLV...\n");

  /* Initialize deivce at I2C port 0 */

  ret = bm1383glv_init(i2c, 0);
  if (ret < 0)
    {
      snerr("Error initialize BM1383GLV.\n");
      return ret;
    }

  /* Register devices for each FIFOs at I2C port 0 */

  ret = bm1383glv_register(devpath, 0, i2c, 0);
  if (ret < 0)
    {
      snerr("Error registering BM1383GLV.\n");
      return ret;
    }

  return ret;
}
#endif /* CONFIG_CXD56_SCU */

#endif /* CONFIG_I2C && CONFIG_CXD56_I2C0 && CONFIG_BM1383GLV */
