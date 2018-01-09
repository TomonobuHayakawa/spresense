/****************************************************************************
 * modules/cmdfw/bt_ble_wrapper.h
 * Spritzer Command Framework Bluetooth Wrapper
 *
 *   Copyright (C) 2017 Sony Corpration. All rights reserved.
 *   Author: Daisuke Sonoda <Daisuke.xA.Sonoda@Sony.com>
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

#ifndef BT_BLE_WRAPPER_H
#define BT_BLE_WRAPPER_H

#include <stdio.h>
#include <bt/bt_comm.h>
#include <bt/bt_spp.h>
#include <ble/ble_comm.h>
#include <ble/ble_gap.h>
#include <ble/ble_gatts.h>
#include <ble/ble_gattc.h>

/********************************************************************************************
 * Public Types
 ********************************************************************************************/
#define BT_BLE_UUID128_LEN 16

typedef struct
{
  uint8_t uuid128[BT_BLE_UUID128_LEN];
}BT_BLE_UUID;

/**
 * @brief callback
 */
typedef void (*RxCallBack)(uint8_t* data, uint16_t len);

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/
/****************************************************************************
 * Name: BTBLE_Initialize
 *
 * Description:
 *   Start spritzer command framework service.
 *
 ****************************************************************************/
int BTBLE_Initialize(char* name, BT_ADDR addr, BT_BLE_UUID uuid, RxCallBack callback);

/****************************************************************************
 * Name: BTBLE_Finalize
 *
 * Description:
 *   Start spritzer command framework service.
 *
 ****************************************************************************/
void BTBLE_Finalize(void);

/********************************************************************************************
 * Name: BTBLE_StartService
 *
 * Description:
 *   Start spritzer command framework service.
 *
 ********************************************************************************************/
int BTBLE_StartService(void);

/********************************************************************************************
 * Name: BTBLE_StopService
 *
 * Description:
 *   Stop spritzer command framework service.
 *
 ********************************************************************************************/
int BTBLE_StopService(void);

/********************************************************************************************
 * Name: BTBLE_SendData
 *
 * Description:
 *   Send spritzer command framework data.
 *
 ********************************************************************************************/
int BTBLE_SendData(uint8_t* data, uint16_t len);

#endif /* BT_BLE_WRAPPER_H */

