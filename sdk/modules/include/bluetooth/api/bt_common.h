/****************************************************************************
 * modules/include/bluetooth/api/bt_common.h
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

#ifndef __MODULES_INCLUDE_BLUETOOTH_API_BT_COMMON_H
#define __MODULES_INCLUDE_BLUETOOTH_API_BT_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <bluetooth/bluetooth.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**@brief BT profile connection status
 */
typedef enum
{
	BT_DISCONNECTED  = 0,
	BT_DISCONNECTING = 1,
	BT_CONNECTING    = 2,
	BT_CONNECTED     = 3
} BT_CONNECT_STATUS;

/** Bluetooth base context
 */
struct bt_common_state_s
{
  struct bt_hal_common_ops_s *bt_hal_common_ops;    /* BT common HAL interfaces */
  struct bt_ble_common_ops_s *bt_ble_common_ops;    /* BT status callbacks */
  BT_ADDR                    bt_addr;               /* BT local device address */
  BT_ADDR                    ble_addr;              /* BLE local device address */
  char                       bt_name[BT_NAME_LEN];  /* BT local device name */
  char                       ble_name[BT_NAME_LEN]; /* BLE local device name */
};

/** Bluetooth ACL context
 */
struct bt_acl_state_s
{
  BT_CONNECT_STATUS          bt_acl_connection;            /* Status of ACL connection */
  struct bt_common_state_s   *bt_common_state;            /* BT base context */
  BT_ADDR                    bt_target_addr;              /* BT target device address */
  char                       bt_target_name[BT_NAME_LEN]; /* BT target device name */
};

/** Bluetooth Common application callbacks
 */
struct bt_ble_common_ops_s
{
  void (*command_status)(BT_CMD_STATUS status);
  void (*pairing_complete)(BT_ADDR addr, BT_PAIR_STATUS status);
  void (*inquiry_result)(BT_ADDR addr, char *name);
  void (*inquiry_complete)(void);
  void (*connect_status_changed)(struct bt_acl_state_s *bt_acl_state, bool connected, int status);
  void (*connected_device_name)(const char *name);
  void (*bond_info)(BT_ADDR addr);
};

/** Rx Data Callback function
 * @param[out] pdata: Rx data body
 * @param[out] len  : Rx data length
 */
typedef void (*rx_callback)(uint8_t *pdata, int len);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Bluetooth module initialize
 *        For initialize a pin config, NV storage, UART, etc.
 *
 * @retval error code
 */

int bt_init(void);

/**
 * @brief Bluetooth module finalize
 *
 * @retval error code
 */

int bt_finalize(void);

/**
 * @brief Set Bluetooth module address
 *        This is Spresense side address and should be call before bt_enable.
 *
 * @param[in] addr: BT_ADDR* Bluetooth device address
 *
 * @retval error code
 */

int bt_set_address(BT_ADDR *addr);

/**
 * @brief Get Bluetooth module address
 *
 * @param[out] addr: BT_ADDR* Bluetooth device address
 *
 * @retval error code
 */

int bt_get_address(BT_ADDR *addr);

/**
 * @brief Set Bluetooth module name
 *        This name visible for other devices and should be call before bt_enable.
 *
 * @param[in] name: char* Bluetooth device name
 *
 * @retval error code
 */

int bt_set_name(char *name);

/**
 * @brief Get Bluetooth module name
 *
 * @param[out] name: char* Bluetooth device name
 *
 * @retval error code
 */

int bt_get_name(char *name);

/**
 * @brief Bluetooth module enable
 *        Bluetooth set power on(and download firmware).
 *
 * @retval error code
 */

int bt_enable(void);

/**
 * @brief Bluetooth module disable
 *        Bluetooth set power off.
 *
 * @retval error code
 */

int bt_disable(void);

/**
 * @brief Bluetooth pairing enable
 *        Entering bluetooth pairing mode.
 *
 * @retval error code
 */

int bt_pairing_enable(void);

/**
 * @brief Bluetooth pairing disable
 *        Escaping bluetooth pairing mode.
 *
 * @retval error code
 */

int bt_paring_disable(void);

#if 0
/* TBD */
/**
 * @brief Bluetooth set paring mode
 *        Paring mode setting.
 *
 * @retval error code
 */

int btSetParingMode(mode);
#endif

/**
 * @brief Bluetooth get bond device list
 *        Get bond devices list with BD_ADDR.
 *
 * @param[in] addr: BT_ADDR* BD_ADDR list
 * @param[in] num: int* Number of BD_ADDR
 *
 * @retval error code
 */

int bt_get_bond_list(BT_ADDR *addr, int *num);

/**
 * @brief Bluetooth unbond by BD_ADDR
 *        Unbond device by BD_ADDR.
 *
 * @param[out] addr: BT_ADDR* Unbond device BD_ADDR
 *
 * @retval error code
 */

int bt_unbond(BT_ADDR *addr);

/**
 * @brief Bluetooth set visible
 *        Visible this device from others.
 *
 * @retval error code
 */

int bt_set_visibility(BT_VISIBILITY visibility);

/**
 * @brief Bluetooth inquiry start
 *        Start to inquiry for connect peer device.
 *
 * @retval error code
 */

int bt_start_inquiry(void);

/**
 * @brief Bluetooth inquiry cancel
 *        Cancel to inquiry.
 *
 * @retval error code
 */

int bt_cancel_inquiry(void);

/**
 * @brief Bluetooth register commin callbacks
 *        Register Connect/Pairing/Inquiry callback
 *
 * @retval error code
 */

int bt_register_common_cb(struct bt_ble_common_ops_s *bt_ble_common_ops);

#endif /* __MODULES_INCLUDE_BLUETOOTH_API_BT_COMMON_H */
