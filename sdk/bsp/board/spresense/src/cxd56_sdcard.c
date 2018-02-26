/****************************************************************************
 * config/spresense/src/cxd56_sdcard.c
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
#include <sdk/config.h>

#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/mmcsd.h>
#include <nuttx/sdio.h>

#include "chip.h"
#include "up_arch.h"

#include <arch/board/board.h>
#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"
#include "cxd56_sdhci.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TXS02612RTWR: SDIO port expander with voltage level translation */

#define SDCARD_TXS02612_SEL PIN_AP_CLK

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

int board_sdcard_initialize(void)
{
  FAR struct sdio_dev_s *sdhci0;
  struct stat stat_sdio;
  int ret = OK;

  /* Mount the SDHC-based MMC/SD block driver */
  /* This should be used with 3.3V */
  /* First, get an instance of the SDHC interface */

  finfo("Initializing SDHC slot 0\n");

  sdhci0 = cxd56_sdhci_initialize(0);
  if (!sdhci0)
    {
      _err("ERROR: Failed to initialize SDHC slot 0\n");
      return -ENODEV;
    }

  /* Now bind the SDHC interface to the MMC/SD driver */

  finfo("Bind SDHC to the MMC/SD driver, minor=0\n");

  ret = mmcsd_slotinitialize(0, sdhci0);
  if (ret != OK)
    {
      _err("ERROR: Failed to bind SDHC to the MMC/SD driver: %d\n", ret);
      return ret;
    }

  finfo("Successfully bound SDHC to the MMC/SD driver\n");

  /* Handle the initial card state */

  cxd56_sdhci_mediachange(sdhci0);

  if (stat("/dev/mmcsd0", &stat_sdio) == 0)
    {
      if (S_ISBLK(stat_sdio.st_mode))
        {
          ret = mount("/dev/mmcsd0", "/mnt/sd0", "vfat", 0, NULL);
          if (ret == 0)
            {
              _info("Successfully mount a SDCARD via the MMC/SD driver\n");
            }
          else
            {
              _err("ERROR: Failed to mount the SDCARD. %d\n", errno);
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: board_sdcard_pin_initialize
 *
 * Description:
 *   Initialize SD Card pins on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_initialize(void)
{
  cxd56_gpio_config(SDCARD_TXS02612_SEL, false);

#ifdef CONFIG_SDCARD_TXS02612_PORT0
  /* Select port0 for SD-Card */

  cxd56_gpio_write(SDCARD_TXS02612_SEL, false);
#else
  /* Select port1 for SDIO other than SD-Card */

  cxd56_gpio_write(SDCARD_TXS02612_SEL, true);
#endif
}

/****************************************************************************
 * Name: board_sdcard_pin_finalize
 *
 * Description:
 *   Finalize SD Card pins on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_finalize(void)
{
#ifdef CONFIG_SDCARD_TXS02612_PORT0
  /* Disable SDIO pin configuration */

  CXD56_PIN_CONFIGS(PINCONFS_SDIOA_GPIO);
  CXD56_PIN_CONFIGS(PINCONFS_SDIOB_GPIO);
#else
  /* Disable SDIO pin configuration */

  CXD56_PIN_CONFIGS(PINCONFS_SDIOA_GPIO);
#endif
  /* Disable pin configuration */

  cxd56_gpio_config(SDCARD_TXS02612_SEL, false);
}

/****************************************************************************
 * Name: board_sdcard_pin_configuraton
 *
 * Description:
 *   Configure SD Card pins on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_configuraton(void)
{
  /* SDIO configuration */

  modifyreg32(CXD56_SDHCI_USERDEF1CTL, SDHCI_UDEF1_SDCLKI_SEL,
              SDHCI_UDEF1_SDCLKI_SEL_INT);
  modifyreg32(CXD56_SDHCI_USERDEF2CTL, SDHCI_UDEF2_CMD_SEL,
              SDHCI_UDEF2_CMD_SEL_INT);

#ifdef CONFIG_SDCARD_TXS02612_PORT0
  /* SDIO pin configuration */

  CXD56_PIN_CONFIGS(PINCONFS_SDIOA_SDIO);
  CXD56_PIN_CONFIGS(PINCONFS_SDIOB_SDCARD);
#else
  /* SDIO pin configuration with CD, WP pin disabled */

  putreg32(0, CXD56_TOPREG_IOFIX_APP);
  CXD56_PIN_CONFIGS(PINCONFS_SDIOA_SDIO);
#endif
}

/****************************************************************************
 * Name: board_sdcard_pin_enable
 *
 * Description:
 *   Enable SD Card on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_enable(void)
{
}

/****************************************************************************
 * Name: board_sdcard_pin_disable
 *
 * Description:
 *   Disable SD Card pins on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_disable(void)
{
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
}
