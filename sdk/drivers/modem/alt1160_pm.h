/****************************************************************************
 * drivers/modem/alt1160_pm.h
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

#ifndef __DRIVERS_MODEM_ALT1160_PM_H
#define __DRIVERS_MODEM_ALT1160_PM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "alt1160_dev.h"
#include "alt1160_sys.h"
#include "alt1160_pm_state.h"

#if defined(CONFIG_MODEM_ALT_1160)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MODEM_PM_WAKEUP_DONE    (0)
#define MODEM_PM_WAKEUP_ALREADY (1)
#define MODEM_PM_WAKEUP_FAIL    (2)

#define MODEM_PM_CB_SLEEP       (0)
#define MODEM_PM_CB_WAKE        (1)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: alt1160_pm_init
 *
 * Description:
 *   Initialize the ALT1160 power manager driver.
 *
 ****************************************************************************/

int alt1160_pm_init(FAR struct alt1160_dev_s *priv);

/****************************************************************************
 * Name: alt1160_pm_uninit
 *
 * Description:
 *   Uninitialize the ALT1160 power manager driver.
 *
 ****************************************************************************/

int alt1160_pm_uninit(FAR struct alt1160_dev_s *priv);

/****************************************************************************
 * Name: alt1160_pm_wakeup
 *
 * Description:
 *   Make modem wake up.
 *
 ****************************************************************************/

int alt1160_pm_wakeup(FAR struct alt1160_dev_s *priv);

#ifdef CONFIG_MODEM_ALT_1160_PROTCOL_V2_1

/****************************************************************************
 * Name: alt1160_pm_callgpiohandler
 *
 * Description:
 *   Call Device to Host GPIO interrupt handler.
 *
 ****************************************************************************/

int alt1160_pm_callgpiohandler(FAR struct alt1160_dev_s *priv);
#endif

/****************************************************************************
 * Name: alt1160_pm_registercb
 *
 * Description:
 *   Register callback for ALT1160 power manager driver.
 *
 ****************************************************************************/

int alt1160_pm_registercb(alt1160_pm_cbfunc_t cb);

/****************************************************************************
 * Name: alt1160_pm_deregistercb
 *
 * Description:
 *   Deregister callback for ALT1160 power manager driver.
 *
 ****************************************************************************/

int alt1160_pm_deregistercb(void);

/****************************************************************************
 * Name: alt1160_pm_sleepmodem
 *
 * Description:
 *   Make modem sleep.
 *
 ****************************************************************************/

int alt1160_pm_sleepmodem(FAR struct alt1160_dev_s *priv);

/****************************************************************************
 * Name: alt1160_pm_cansleep
 *
 * Description:
 *   Check if modem can sleep.
 *
 ****************************************************************************/

int alt1160_pm_cansleep(FAR struct alt1160_dev_s *priv);

/****************************************************************************
 * Name: alt1160_pm_initwakelock
 *
 * Description:
 *   Initialize the modem wakelock resource.
 *
 ****************************************************************************/

int alt1160_pm_initwakelock(FAR struct alt1160_pm_wakelock_s *lock);

/****************************************************************************
 * Name: alt1160_pm_acquirewakelock
 *
 * Description:
 *   Acquire the modem wakelock.
 *
 ****************************************************************************/

int alt1160_pm_acquirewakelock(FAR struct alt1160_pm_wakelock_s *lock);

/****************************************************************************
 * Name: alt1160_pm_releasewakelock
 *
 * Description:
 *   Release the modem wakelock.
 *
 ****************************************************************************/

int alt1160_pm_releasewakelock(FAR struct alt1160_pm_wakelock_s *lock);

/****************************************************************************
 * Name: alt1160_pm_getnumofwakelock
 *
 * Description:
 *   Get the lock count of the specified wakelock.
 *
 ****************************************************************************/

int alt1160_pm_getnumofwakelock(FAR struct alt1160_pm_wakelock_s *lock);

/****************************************************************************
 * Name: alt1160_pm_getwakelockstate
 *
 * Description:
 *   Get the wakelock status. If the return value is 0, it means that it is
 *   not locked. Otherwise it means that someone is locking.
 *
 ****************************************************************************/

int alt1160_pm_getwakelockstate(void);

#endif
#endif /* __DRIVERS_MODEM_ALT1160_PM_H */
