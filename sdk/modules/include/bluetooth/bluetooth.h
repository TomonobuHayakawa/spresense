/****************************************************************************
 * modules/include/bluetooth/bluetooth.h
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#ifndef __MODULES_INCLUDE_BLUETOOTH_BLUETOOTH_H
#define __MODULES_INCLUDE_BLUETOOTH_BLUETOOTH_H

#define BLUETOOTH_DEBUG

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#ifdef BLUETOOTH_DEBUG
#include <stdio.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/**
 * @defgroup bt_defs Defines
 * @{
 */

#ifdef BLUETOOTH_DEBUG
#undef _err
#define _err(format, ...)	printf(format, ##__VA_ARGS__)
#undef _info
#define _info(format, ...)	printf(format, ##__VA_ARGS__)
#endif

/**
 *@name BT success code
 *@{
 */
#define BT_SUCCESS 0
/** @} */

/**
 *@name BT fail code
 *@{
 */
#define BT_FAIL    -127
/** @} */

/**
 *@name BT Address Length
 *@{
 */
#define BT_ADDR_LEN 6
/** @} */

/**
 *@name BT Name Length
 *@{
 */
#define BT_NAME_LEN 28
/** @} */

/**
 *@name BT Address Length
 *@{
 */
#define BT_UUID128_LEN 16
/** @} */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @defgroup bt_datatypes Data types
 * @{
 */

/**@brief BT address structure
 */
typedef struct
{
	uint8_t address[BT_ADDR_LEN];
} BT_ADDR;

/**@brief 128-bit UUID types
 */
typedef struct
{
	uint8_t uuid128[BT_UUID128_LEN];
} BT_UUID;

/**@brief BT hci command status
 */
typedef enum
{
	BT_COMMAND_STATUS_SUCCESS = 0,
	BT_COMMAND_STATUS_IN_PROGRESS,
	BT_COMMAND_STATUS_ALREADY_CONNECTED,
	BT_COMMAND_STATUS_NOT_CONNECTED,
	BT_COMMAND_STATUS_BAD_HANDLE,
	BT_COMMAND_STATUS_WRONG_STATE,
	BT_COMMAND_STATUS_INVALID_ARGS,
	BT_COMMAND_STATUS_FAILED,
	BT_COMMAND_STATUS_UNKNOWN_GROUP,
	BT_COMMAND_STATUS_UNKNOWN_COMMAND,
	BT_COMMAND_STATUS_CLIENT_NOT_REGISTERED,
	BT_COMMAND_STATUS_OUT_OF_MEMORY,
	BT_COMMAND_STATUS_DISALLOWED
} BT_CMD_STATUS;

/**@brief BT pairing result
 */
typedef enum
{
	BT_PAIR_SUCCESS,
	BT_PAIR_PASSKEY_ENTRY_FAILURE,
	BT_PAIR_OOB_FAILURE,
	BT_PAIR_PAIRING_AUTHENTICATION_FAILURE,
	BT_PAIR_CONFIRM_VALUE_FAILURE,
	BT_PAIR_PAIRING_NOT_SUPPORTED,
	BT_PAIR_ENCRYPTION_KEY_SIZE_FAILURE,
	BT_PAIR_INVALID_COMMAND,
	BT_PAIR_PAIRING_FAILURE_UNKNOWN,
	BT_PAIR_REPEATED_ATTEMPTS,
	BT_PAIR_INTERNAL_PAIRING_ERROR,
	BT_PAIR_UNKNOWN_IO_CAPABILITIES,
	BT_PAIR_SMP_INITIALIZATION_FAILURE,
	BT_PAIR_CONFIRMATION_FAILRUE,
	BT_PAIR_SMP_BUSY,
	BT_PAIR_ENCRYPTION_FAILURE,
	BT_PAIR_BONDING_STARTED,
	BT_PAIR_RESPONSE_TIMEOUT,
	BT_PAIR_GENERIC_FAILURE,
	BT_PAIR_CONNECTION_TIMEOUT,
} BT_PAIR_STATUS;

/**@brief BT profile connection result
 */
typedef enum
{
	BT_CONNECT_NO_SERVICE = 0,
	BT_CONNECT_NO_DEVICE,
	BT_CONNECT_TIMEOUT,
	BT_CONNECT_OTHER,
} BT_CONNECT_FAIL_REASON_ID;

/**@brief BT visibility ID
 */
typedef enum
{
  BT_VIS_NO_DISCOVERY_NO_CONNECTABLE = 0,
  BT_VIS_DISCOVERY_NO_CONNECTABLE    = 1,
  BT_VIS_NO_DISCOVERY_CONNECTABLE    = 2,
  BT_VIS_DISCOVERY_CONNECTABLE       = 3
} BT_VISIBILITY;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __MODULES_INCLUDE_BLUETOOTH_BLUETOOTH_H */
