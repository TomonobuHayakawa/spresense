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


/******************************************************************************
 *
 *  NFC Hardware Abstraction Layer API(extra)
 *
 ******************************************************************************/
#ifndef NFC_HAL_API_EXTRA_H
#define NFC_HAL_API_EXTRA_H
#include "nfc_hal_api.h"

/*******************************************************************************
** HAL API Function Prototypes(extra)
*******************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

EXPORT_HAL_API INT32 HAL_NfcSetoptionalparm (UINT32 option_no, UINT32 new_option);

/*******************************************************************************
**
** Function         HAL_NfcPreDiscoverCmdReg
**
** Description      Register any vendor-specific pre-discovery NCI message
**
** Returns          void
**
*******************************************************************************/
EXPORT_HAL_API BOOLEAN HAL_NfcPreDiscoverCmdReg ( UINT16 data_len, UINT8 *p_data );


#ifdef __cplusplus
}
#endif

#endif /* NFC_HAL_API_EXTRA_H  */
