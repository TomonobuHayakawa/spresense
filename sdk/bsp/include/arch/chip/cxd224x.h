/****************************************************************************
 * arch/arm/include/cxd56xx/cxd224x.h
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Tsutomu Ito <Tsutomu.Ito@sony.com>
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

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_CXD56_CXD224X_H
#define __ARCH_ARM_INCLUDE_CXD56XX_CXD56_CXD224X_H

#define CXDNFC_MAGIC (_WLIOCBASE) //Take care of conflict.

/*
 * CXDNFC power control via ioctl
 * CXDNFC_POWER_CTL(0): power off
 * CXDNFC_POWER_CTL(1): power on
 * CXDNFC_WAKE_CTL(0): PON HIGH (normal power mode)
 * CXDNFC_WAKE_CTL(1): PON LOW (low power mode)
 * CXDNFC_WAKE_RST():  assert XRST
 */

#define CXDNFC_POWER_CTL		_IOC(CXDNFC_MAGIC, 0x01)
#define CXDNFC_WAKE_CTL			_IOC(CXDNFC_MAGIC, 0x02)
#define CXDNFC_RST_CTL			_IOC(CXDNFC_MAGIC, 0x03)

#define CXDNFC_RST_ACTIVE 1            /* ActiveHi = 1, ActiveLow = 0 */

#endif
