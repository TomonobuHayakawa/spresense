/****************************************************************************
 * bsp/board/spresense/include/board_pinconfig.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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

#ifndef __BSP_BOARD_SPRESENSE_INCLUDE_BOARD_PINCONFIG_H
#define __BSP_BOARD_SPRESENSE_INCLUDE_BOARD_PINCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Customize from default to the board specific pin configuration
 * The default pin configurations are defined in sdk/bsp/src/chip/cxd5602_pinconfig.h.
 *  Mode: shared pin function mode
 *  ENZI: 1=Input Enable, 0=Input Disable
 *  4mA : Drive Current 1=4mA, 0=2mA
 *  Pull: 0=HiZ floating, PINCONF_PULLUP, PINCONF_PULLDOWN
 *                                                                      M  E     P
 *                                                  P                   o  N  4  u
 *                                                  i                   d  Z  m  l
 *                                                  n                   e  I  A  l
 */
#undef PINCONF_UART2_CTS
#define PINCONF_UART2_CTS                   PINCONF(PIN_UART2_CTS,      1, 1, 0, PINCONF_PULLDOWN)

#undef PINCONF_SPI4_MOSI
#define PINCONF_SPI4_MOSI                   PINCONF(PIN_SPI4_MOSI,      1, 0, 1, 0)

#endif  /* __BSP_BOARD_SPRESENSE_INCLUDE_BOARD_PINCONFIG_H */
