/****************************************************************************
 * config/sparduino/src/cxd56_sdcard.c
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Kazuya Hioki <Kazuya.Hioki@sony.com>
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "chip.h"
#include "up_arch.h"

#include <arch/board/board.h>
#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IP4855CX25: SD memory card integrated dual voltage level translator */

#define SDCARD_VOLTAGE_LEVEL_TRANS_SEL PIN_GNSS_1PPS_OUT
#define SDCARD_VOLTAGE_LEVEL_TRANS_EN  PIN_HIF_IRQ_OUT

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_sdcard_initialize
 *
 * Description:
 *   Initialize SD Card on the board.
 *
 ****************************************************************************/

void board_sdcard_initialize(void)
{
  /* Initialize pin configuration (input and output disabled yet) */

  cxd56_gpio_config(SDCARD_VOLTAGE_LEVEL_TRANS_SEL, false);
  cxd56_gpio_config(SDCARD_VOLTAGE_LEVEL_TRANS_EN, false);
}

/****************************************************************************
 * Name: board_sdcard_finalize
 *
 * Description:
 *   Finalize SD Card on the board.
 *
 ****************************************************************************/

void board_sdcard_finalize(void)
{
  /* Disable level translater */

  cxd56_gpio_write(SDCARD_VOLTAGE_LEVEL_TRANS_EN, false);

  /* Return back to pin configuration */

  cxd56_gpio_config(SDCARD_VOLTAGE_LEVEL_TRANS_SEL, false);
  cxd56_gpio_config(SDCARD_VOLTAGE_LEVEL_TRANS_EN, false);
}

/****************************************************************************
 * Name: board_sdcard_enable
 *
 * Description:
 *   Enable SD Card on the board.
 *
 ****************************************************************************/

void board_sdcard_enable(void)
{
  /* Enable level translater with 3.3V */

  cxd56_gpio_write(SDCARD_VOLTAGE_LEVEL_TRANS_SEL, false);
  up_mdelay(100);
  cxd56_gpio_write(SDCARD_VOLTAGE_LEVEL_TRANS_EN, true);
  up_mdelay(100);
}

/****************************************************************************
 * Name: board_sdcard_disable
 *
 * Description:
 *   Disable SD Card on the board.
 *
 ****************************************************************************/

void board_sdcard_disable(void)
{
  /* Disable level translater with 3.3V */

  cxd56_gpio_write(SDCARD_VOLTAGE_LEVEL_TRANS_SEL, false);
  up_mdelay(100);
  cxd56_gpio_write(SDCARD_VOLTAGE_LEVEL_TRANS_EN, false);
  up_mdelay(100);
}

/****************************************************************************
 * Name: board_sdcard_set_high_voltage
 *
 * Description:
 *   Set SD Card IO voltage to 3.3V
 *
 ****************************************************************************/

void board_sdcard_set_high_voltage(void)
{
  /* Switch 3.3V */

  cxd56_gpio_write(SDCARD_VOLTAGE_LEVEL_TRANS_SEL, false);
  up_mdelay(100);
}

/****************************************************************************
 * Name: board_sdcard_set_low_voltage
 *
 * Description:
 *   Set SD Card IO voltage to 1.8V
 *
 ****************************************************************************/

void board_sdcard_set_low_voltage(void)
{
  /* Switch 1.8V */

  cxd56_gpio_write(SDCARD_VOLTAGE_LEVEL_TRANS_SEL, true);
  up_mdelay(100);
}
