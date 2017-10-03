/****************************************************************************
 * configs/cxd56xx/src/cxd56_bh1745nuc.c
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

#include <nuttx/config.h>

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>

#include <nuttx/sensors/bh1745nuc.h>
#ifdef CONFIG_CXD56_SCU
#include <arch/chip/cxd56_scu.h>
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_CXD56_I2C0) && defined(CONFIG_BH1745NUC)

#ifdef CONFIG_CXD56_SCU
int cxd56_bh1745nucinitialize(FAR const char *devpath,
                              FAR struct i2c_master_s* i2c)
{
  int ret;

  sndbg("Initializing BH1745NUC...\n");

  /* Initialize deivce at I2C port 0 */

  ret = bh1745nuc_init(i2c, 0);
  if (ret < 0)
    {
      sndbg("Error initialize BH1745NUC.\n");
      return ret;
    }

  /* Register devices for each FIFOs at I2C port 0 */

  ret = bh1745nuc_register(devpath, 0, i2c, 0);
  if (ret < 0)
    {
      sndbg("Error registering BH1745NUC.\n");
      return ret;
    }

  return ret;
}
#endif /* CONFIG_CXD56_SCU */

#endif /* CONFIG_I2C && CONFIG_CXD56_I2C0 && CONFIG_BH1745NUC */
