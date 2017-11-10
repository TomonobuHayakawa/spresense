/****************************************************************************
 * configs/corvo/src/cxd56_bcm20706.c
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Yuchi.Wen <Yuchi.Wen@sony.com>
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
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <arch/board/board.h>

#include "cxd56_gpio.h"
#include "cxd56_sysctl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define BCM20707_RST_N    PIN_HIF_IRQ_OUT
#define BCM20707_DEV_WAKE PIN_SPI2_CS_X
#define NUM_OF_PINS (sizeof(pin_cfg) / sizeof(pin_cfg[0]))

#define BCM20707_RST_DELAY  (50 * 1000)  /* ms */

/****************************************************************************
 * Private Data
 ****************************************************************************/
static const struct {
  uint32_t pin;
  bool input_enable;
  bool init_val;
} pin_cfg[] = {
  {BCM20707_RST_N,    false, false}, /* out, low */
  {BCM20707_DEV_WAKE, false, true},  /* out, high */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
int board_bcm20707_pin_cfg(void)
{
  int ret = 0;
  int i = 0;
  for (i = 0; i < NUM_OF_PINS; ++i) {
    ret = cxd56_gpio_config(pin_cfg[i].pin, pin_cfg[i].input_enable);
    if (!pin_cfg[i].input_enable) {
      cxd56_gpio_write(pin_cfg[i].pin, pin_cfg[i].init_val);
    }
  }
  return ret;
}

void board_bcm20707_reset(void)
{
  cxd56_gpio_write(BCM20707_RST_N, false);
  usleep(BCM20707_RST_DELAY);
  cxd56_gpio_write(BCM20707_RST_N, true);
  usleep(BCM20707_RST_DELAY);
}

int board_bcm20707_power_control(bool en)
{
  int ret = 0;
  ret = board_power_control(POWER_BTBLE, en);
  return ret;
}

void board_bcm20707_enable_sleep(bool en)
{
  cxd56_gpio_write(BCM20707_DEV_WAKE, en);
}

int board_bcm20707_get_firmware_size_in_flash(const char* firmwareName, int* firmware_size)
{
  sysctl_getfwsize_t fwsize = {{0}};
  strncpy(fwsize.filename, firmwareName, (sizeof(fwsize.filename) - 1));
  *firmware_size = cxd56_sysctlcmd(SYSCTL_GETFWSIZE, (uint32_t)(uintptr_t)&fwsize);
  if ((*firmware_size) <= 0) {
    return -1;
  }
  return 0;
}

int board_bcm20707_load_firmware_from_flash(const char* firmwareName, uint32_t* loadbuf)
{
  int ret = 0;
  sysctl_loadfw_t fwload = {0};
  fwload.cpuid = 2;
  fwload.addr = (uint32_t)loadbuf;
  strncpy(fwload.filename, firmwareName, (sizeof(fwload.filename) -1));
  ret = cxd56_sysctlcmd(SYSCTL_LOADFW, (uint32_t)(uintptr_t)&fwload);
  return ret;
}
