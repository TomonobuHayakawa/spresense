/****************************************************************************
 * drivers/modem/alt1160_pm_state.h
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

#ifndef __DRIVERS_MODEM_ALT1160_PM_STATE_H
#define __DRIVERS_MODEM_ALT1160_PM_STATE_H

#if defined(CONFIG_MODEM_ALT_1160)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MODEM_PM_INTERNAL_STATE_SLEEP          (0)
#define MODEM_PM_INTERNAL_STATE_GOING_TO_WAKE  (1)
#define MODEM_PM_INTERNAL_STATE_WAKE           (2)
#define MODEM_PM_INTERNAL_STATE_GOING_TO_SLEEP (3)
#define MODEM_PM_INTERNAL_STATE_MAX            (4)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: alt1160_pm_getstate
 *
 * Description:
 *   Get current modem state.
 *
 ****************************************************************************/

uint32_t alt1160_pm_getstate(void);

/****************************************************************************
 * Name: alt1160_pm_getinternalstate
 *
 * Description:
 *   Get internal modem state.
 *
 ****************************************************************************/

uint32_t alt1160_pm_getinternalstate(void);

/****************************************************************************
 * Name: alt1160_pm_setinternalstate
 *
 * Description:
 *   Set internal modem state.
 *
 ****************************************************************************/

void alt1160_pm_setinternalstate(uint32_t state);

#endif
#endif /* __DRIVERS_MODEM_ALT1160_SPI_H */
