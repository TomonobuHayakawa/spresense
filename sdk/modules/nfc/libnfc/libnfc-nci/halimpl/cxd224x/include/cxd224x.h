/******************************************************************************
 *
 *  Copyright (C) 2012 Broadcom Corporation.
 *  Copyright (C) 2013 Sony Corporation.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#ifndef _CXD224X_H
#define _CXD224X_H

#ifdef SPZ2_IMPL
#include <sys/ioctl.h>             //Added for _IO() macro
#define CXDNFC_MAGIC (_WLIOCBASE)  //Take care of conflict.
#else
#define CXDNFC_MAGIC 'S'
#endif

/*
 * CXDNFC power control via ioctl
 * CXDNFC_POWER_CTL(0): power off
 * CXDNFC_POWER_CTL(1): power on
 * CXDNFC_WAKE_CTL(0): PON HIGH (normal power mode)
 * CXDNFC_WAKE_CTL(1): PON LOW (low power mode)
 * CXDNFC_WAKE_RST():  assert XRST
 */

#ifdef SPZ2_IMPL
#define CXDNFC_POWER_CTL		_IOC(CXDNFC_MAGIC, 0x01)
#define CXDNFC_WAKE_CTL			_IOC(CXDNFC_MAGIC, 0x02)
#define CXDNFC_RST_CTL			_IOC(CXDNFC_MAGIC, 0x03)
#else
#define CXDNFC_POWER_CTL		_IO(CXDNFC_MAGIC, 0x01)
#define CXDNFC_WAKE_CTL			_IO(CXDNFC_MAGIC, 0x02)
#define CXDNFC_RST_CTL			_IO(CXDNFC_MAGIC, 0x03)
#endif //SPZ2_IMPL

#define CXDNFC_RST_ACTIVE 1            /* ActiveHi = 1, ActiveLow = 0 */

struct cxd224x_platform_data {
	unsigned int irq_gpio;
	unsigned int en_gpio;
	unsigned int wake_gpio;
	unsigned int rst_gpio;
};

#endif
