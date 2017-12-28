/******************************************************************************
 *
 *  Copyright (C) 2013 Sony Corporation
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

#include "nfc_types.h"

#ifndef NFC_TYPES_EXTRA_H
#define NFC_TYPES_EXTRA_H

/****************************************************************************
** NFC_HAL_TASK  definitions(extra)
*****************************************************************************/

/* NFC_HAL_TASK event messages */
/* TODO: if add extra NFC_HAL_EVT_xx at end of nfc_types.h, must update NFC_HAL_EVT_LAST */
#define NFC_HAL_EVT_LAST                         NFC_HAL_EVT_CONTROL_GRANTED
#define NFC_HAL_EVT_TO_PREDISCOVER_CMD_NFC_NCI  (NFC_HAL_EVT_LAST + 0x0100)   /* send registered pre discover cmd */
#define NFC_HAL_EVT_TO_PREDISCOVER_CMD_REG      (NFC_HAL_EVT_LAST + 0x0200)   /* register pre discover cmd */
#define NFC_HAL_EVT_POST_TERMINATE              (NFC_HAL_EVT_LAST + 0x0300)
#define NFC_HAL_EVT_TO_NFC_HALCC                (NFC_HAL_EVT_LAST + 0x0400)   /* HAL control message for sending to HAL */
#endif /* NFC_TYPES_EXTRA_H */
