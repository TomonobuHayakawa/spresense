/****************************************************************************
 * config/spresense/src/cxd56_power.c
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <semaphore.h>

#include <nuttx/arch.h>

#include "chip.h"
#include "up_arch.h"

#include <arch/chip/pm.h>
#include <arch/board/board.h>
#include "cxd56_pmic.h"
#include "cxd56_pinconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_ltsem = SEM_INITIALIZER(1);
static bool g_used_lna = true;
static bool g_used_tcxo = true;
#ifdef CONFIG_BOARDCTL_RESET
static struct pm_cpu_freqlock_s g_hv_lock =
  PM_CPUFREQLOCK_INIT(PM_CPUFREQLOCK_TAG('B','P',0), PM_CPUFREQLOCK_FLAG_HV);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_power_control
 *
 * Description:
 *   Power on/off the device on the board.
 *
 ****************************************************************************/

int board_power_control(int target, bool en)
{
  int ret = 0;
  int (*pfunc)(uint8_t chset, bool en) = NULL;

  switch (PMIC_GET_TYPE(target))
    {
#ifdef CONFIG_CXD56_PMIC
    case PMIC_TYPE_LSW:
      pfunc = cxd56_pmic_set_loadswitch;
      break;
    case PMIC_TYPE_GPO:
      pfunc = cxd56_pmic_set_gpo;
      break;
    case PMIC_TYPE_DDCLDO:
      pfunc = cxd56_pmic_set_ddc_ldo;
      break;
#endif /* CONFIG_CXD56_PMIC */
    default:
      break;
    }

  if (pfunc)
    {
      ret = pfunc(PMIC_GET_CH(target), en);
    }

  return ret;
}

/****************************************************************************
 * Name: board_power_monitor
 *
 * Description:
 *   Get status of Power on/off the device on the board.
 *
 ****************************************************************************/

bool board_power_monitor(int target)
{
  bool ret = false;
  bool (*pfunc)(uint8_t chset) = NULL;

  switch (PMIC_GET_TYPE(target))
    {
#ifdef CONFIG_CXD56_PMIC
    case PMIC_TYPE_LSW:
      pfunc = cxd56_pmic_get_loadswitch;
      break;
    case PMIC_TYPE_GPO:
      pfunc = cxd56_pmic_get_gpo;
      break;
    case PMIC_TYPE_DDCLDO:
      pfunc = cxd56_pmic_get_ddc_ldo;
      break;
#endif /* CONFIG_CXD56_PMIC */
    default:
      break;
    }

  if (pfunc)
    {
      ret = pfunc(PMIC_GET_CH(target));
    }

  return ret;
}

/****************************************************************************
 * Name: board_flash_power_control
 *
 * Description:
 *   Power on/off the flash device on the board.
 *
 ****************************************************************************/

int board_flash_power_control(bool en)
{
  int ret = 0;

  if (en)
    {
      /* power on */

      board_power_control(POWER_FLASH, true);

      /* pin enable */

      CXD56_PIN_CONFIGS(PINCONFS_SPI1);

    }
  else
    {
      /* pin disable */

      CXD56_PIN_CONFIGS(PINCONFS_SPI1_GPIO);

      /* power off */

      board_power_control(POWER_FLASH, false);
    }
  return ret;
}

/****************************************************************************
 * Name: board_flash_power_monitor
 *
 * Description:
 *   Get status of Power on/off the flash device on the board.
 *
 ****************************************************************************/

bool board_flash_power_monitor(void)
{
  return board_power_monitor(POWER_FLASH);
}

/****************************************************************************
 * Name: board_xtal_power_control
 *
 * Description:
 *   Power on/off the Xtal device on the board.
 *
 ****************************************************************************/

int board_xtal_power_control(bool en)
{
  int ret = 0;

  /* Get exclusive access to the lna / tcxo power control */

  sem_wait(&g_ltsem);

  if (en)
    {
      /* power on */

      board_power_control(POWER_TCXO, true);

      /* set used flag */

      g_used_tcxo = true;
    }
  else
    {
      /* power off */

      if (!g_used_lna)
        {
          board_power_control(POWER_TCXO, false);
        }

      /* unset used flag */

      g_used_tcxo = false;
    }

  sem_post(&g_ltsem);

  return ret;
}

/****************************************************************************
 * Name: board_xtal_power_monitor
 *
 * Description:
 *   Get status of Power on/off the Xtal device on the board.
 *
 ****************************************************************************/

bool board_xtal_power_monitor(void)
{
  return board_power_monitor(POWER_TCXO);
}

/****************************************************************************
 * Name: board_lna_power_control
 *
 * Description:
 *   Power on/off the LNA device on the board.
 *
 ****************************************************************************/

int board_lna_power_control(bool en)
{
  int ret = 0;

  /* Get exclusive access to the lna / tcxo power control */

  sem_wait(&g_ltsem);

  if (en)
    {
      /* power on */

      board_power_control(POWER_LNA, true);

      /* set used flag */

      g_used_lna = true;
    }
  else
    {
      /* power off */

      if (!g_used_tcxo)
        {
          board_power_control(POWER_LNA, false);
        }

      /* unset used flag */

      g_used_lna = false;
    }

  sem_post(&g_ltsem);

  return ret;
}

/****************************************************************************
 * Name: board_reset
 *
 * Description:
 *   Reset board.  This function may or may not be supported by a
 *   particular board architecture.
 *
 * Input Parameters:
 *   status - Status information provided with the reset event.  This
 *     meaning of this status information is board-specific.  If not used by
 *     a board, the value zero may be provided in calls to board_reset.
 *
 * Returned Value:
 *   If this function returns, then it was not possible to power-off the
 *   board due to some constraints.  The return value int this case is a
 *   board-specific reason for the failure to shutdown.
 *
 * Assumptions:
 *   Must not compile up_systemreset.c to avoid duplication symbol definition
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_RESET
int board_reset(int status)
{
  /* Restore the original state for bootup after power cycle  */

  board_xtal_power_control(true);
  board_flash_power_control(true);
  up_pm_acquire_freqlock(&g_hv_lock);

  /* System reboot */

  up_pm_reboot(); /* this function never returns */

  return 0;
}
#endif

/****************************************************************************
 * Name: board_power_off
 *
 * Description:
 *   Power off the board.
 *
 *   If this function returns, then it was not possible to power-off the
 *   board due to some other constraints.
 *
 * Input Parameters:
 *   status - Status information provided with the power off event.
 *
 * Returned Value:
 *   If this function returns, then it was not possible to power-off the
 *   board due to some constraints.  The return value int this case is a
 *   board-specific reason for the failure to shutdown.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_POWEROFF
int board_power_off(int status)
{
  /* Power off explicitly because GPOs are kept during deep sleeping */

  board_power_control(PMIC_GPO(0) | PMIC_GPO(1) | PMIC_GPO(2) | PMIC_GPO(3) |
                      PMIC_GPO(4) | PMIC_GPO(5) | PMIC_GPO(6) | PMIC_GPO(7),
                      false);

  /* Enter deep sleep mode */

  up_pm_sleep(PM_SLEEP_DEEP); /* this function never returns */

  return 0;
}
#endif

