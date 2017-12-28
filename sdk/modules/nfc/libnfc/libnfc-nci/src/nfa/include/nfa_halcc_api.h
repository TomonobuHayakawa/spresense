/******************************************************************************
 *
 *  Copyright (C) 2014 Sony Corporation
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

#ifndef NFA_HALCC_API_H
#define NFA_HALCC_API_H

#include "nfc_target.h"
#include "nfa_api.h"

/*****************************************************************************
**  Constants and data types
*****************************************************************************/

/*****************************************************************************
**  External Function Declarations
*****************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
**
** Function         NFA_SendHalControlCommand
**
** Description      This function is called to send a HAL control command to the HAL.
**
**                  id              - The command id
**                  cmd_params_len  - The command parameter len
**                  p_cmd_params    - The command parameter
**                  p_cback         - The callback function to receive the command
**                                    status
**
** Returns          NFA_STATUS_OK if successfully initiated
**                  NFA_STATUS_FAILED otherwise
**
*******************************************************************************/
NFC_API extern tNFA_STATUS NFA_SendHalControlCommand ( UINT8 id,
                                                       UINT8 cmd_param_len,
                                                       UINT8 *pp_cmd_params,
                                                       tNFA_HALCC_CBACK *p_cback);

#ifdef __cplusplus
}
#endif

#endif /* NFA_HALCC_API_H */
