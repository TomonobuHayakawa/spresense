/****************************************************************************
 * bsp/board/spresense/src/cxd56_spisd.c
 *
 *   Copyright (C) 2018 Sony Corporation.
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

#include <debug.h>

#include <nuttx/mmcsd.h>

#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"
#include "cxd56_spisd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_CXD56_SPISD_SLOT_NO
#  define CONFIG_CXD56_SPISD_SLOT_NO 0
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56xx_spisdinitialize
 *
 * Description:
 *   Initialize the SPI-based SD card.
 *
 ****************************************************************************/

int cxd56xx_spisdinitialize(int minor, FAR struct spi_dev_s *spi)
{
  int ret;

  /* Pin Configuration(AP_CLK = SD card detect pin) */

  CXD56_PIN_CONFIGS(PINCONFS_AP_CLK_GPIO);

  /* Input enable */

  cxd56_gpio_config(PIN_AP_CLK, true);

  /* Get the SPI driver instance for the SD chip select */

  finfo("Initializing bit bang SPI for the MMC/SD slot\n");

  ret = mmcsd_spislotinitialize(minor, CONFIG_CXD56_SPISD_SLOT_NO, spi);
  if (ret < 0)
    {
      ferr("ERROR: Failed to bind  bit bang SPI device to MMC/SD slot %d: %d\n",
            0, ret);
      return ret;
    }

  finfo("Successfuly bound  bit bang SPI device to MMC/SD slot %d\n",
        CONFIG_CXD56_SPISD_SLOT_NO);

  return OK;
}
