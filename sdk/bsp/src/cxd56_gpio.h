/********************************************************************************************
 * arch/arm/src/cxd56xx/cxd56_gpio.h
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Hitoshi Fukuda <Hitoshi.Fukuda@sony.com>
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_GPIO_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_GPIO_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#include "cxd56_pinconfig.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

/********************************************************************************************
 * Name: cxd56_gpio_config
 *
 * Description:
 *   Configure a GPIO which input is enabled or not.
 *   Output is enabled when cxd56_gpio_write() is called.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ********************************************************************************************/

int cxd56_gpio_config(uint32_t pin, bool input_enable);

/********************************************************************************************
 * Name: cxd56_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 * Returned Value:
 *   None
 *
 ********************************************************************************************/

void cxd56_gpio_write(uint32_t pin, bool value);

/********************************************************************************************
 * Name: cxd56_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 * Returned Value:
 *   The boolean state of the input pin
 *
 ********************************************************************************************/

bool cxd56_gpio_read(uint32_t pin);

/********************************************************************************************
 * Function:  cxd56_gpio_dump
 *
 * Description:
 *   Dump a gpio pin configuration
 *
 ********************************************************************************************/

#ifdef CONFIG_DEBUG
int cxd56_gpio_dump(uint32_t pin, const char *msg);
#else
#  define cxd56_gpio_dump(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_GPIO_H */
