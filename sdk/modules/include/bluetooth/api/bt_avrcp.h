/****************************************************************************
 * modules/include/bluetooth/api/bt_avrcp.h
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

#ifndef __MODULES_INCLUDE_BLUETOOTH_API_BT_AVRCP_H
#define __MODULES_INCLUDE_BLUETOOTH_API_BT_AVRCP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/bluetooth_avrcp_cmds.h>
#include <bluetooth/api/bt_common.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @defgroup bt_datatypes Data types
 * @{
 */

/**@brief BT profile connection status
 */
typedef enum
{
	BT_AVRCP_CONTROLLER  = 0,
	BT_AVRCP_TARGET      = 1
} BT_AVRCP_ROLE;

/**@brief BT avrc track info type TODO: Need to confirm official specification
*/
typedef struct avrcTrackInfo {
	BT_ADDR addr;
	uint8_t attrId;
	uint16_t attrLen;
	uint8_t attrValue[48];
} BT_AVRC_TRACK_INFO;

/** Bluetooth AVRCP application callbacks
 */
/* TODO: correct argument types*/
struct bt_avrcp_notify_ops_s
{
  void (*playStatusChange)(BT_AVRC_TRACK_INFO *trackInfo);
  void (*trackChange)(BT_AVRC_TRACK_INFO *trackInfo);
  void (*trackReachedEnd)(uint8_t *pdata, int len);
  void (*trackReachedStart)(uint8_t *pdata, int len);
  void (*playPosChanged)(uint8_t *pdata, int len);
  void (*batteryStatusChange)(uint8_t *pdata, int len);
  void (*systemStatusChange)(uint8_t *pdata, int len);
  void (*appSettingChange)(uint8_t *pdata, int len);
  void (*nowPlayingChange)(uint8_t *pdata, int len);
  void (*avalPlayerChange)(uint8_t *pdata, int len);
  void (*addrPlayChange)(uint8_t *pdata, int len);
  void (*uidsChange)(uint8_t *pdata, int len);
  void (*volumeChange)(uint8_t *pdata, int len);
};

/** Bluetooth AVRCP application callbacks
 */
struct bt_avrcp_ops_s
{
  void (*command_status)(BT_CMD_STATUS status);
  void (*connect)(struct bt_acl_state_s *bt_acl_state, BT_AVRCP_ROLE role);
  void (*disconnect)(struct bt_acl_state_s *bt_acl_state, BT_AVRCP_ROLE role);
};

/** Bluetooth AVRCP context
 */
struct bt_avrcp_state_s
{
  BT_CONNECT_STATUS            bt_avrcc_connection;  /* Status of AVRCP(controller) connection */
  BT_CONNECT_STATUS            bt_avrct_connection;  /* Status of AVRCP(target) connection */
  struct bt_acl_state_s        *bt_acl_state;        /* Bluetooth ACL context */
  struct bt_hal_avrcp_ops_s    *bt_hal_avrcp_ops;    /* AVRCP HAL interfaces */
  struct bt_avrcp_ops_s        *bt_avrcp_ops;        /* AVRCP connection callbacks */
  struct bt_avrcp_notify_ops_s *bt_avrcp_notify_ops; /* AVRCP Notification callbacks */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Get AVRCP support or not support
 *
 * @retval Suppot or Not support
 */

bool bt_avrcp_is_supported(void);

/**
 * @brief Bluetooth AVRCP connect
 *        Connect to peer device with AVRCP.
 *
 * @param[in] addr: BT_ADDR* device BD_ADDR to connect
 *
 * @retval error code
 */

int bt_avrcp_connect(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth AVRCP disconnect
 *        Disconnect to peer device with AVRCP.
 *
 * @param[in] addr: BT_ADDR* device BD_ADDR to disconnect
 *
 * @retval error code
 */

int bt_avrcp_disconnect(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth AVRCP disconnect
 *        Disconnect to peer device with AVRCP.
 *
 * @param[in] addr: BT_ADDR* device BD_ADDR to disconnect
 * @param[in] eventCode: int Event ID.(TBD. Might contain data)
 *
 * @retval error code
 */

int bt_avrcp_send_command(struct bt_acl_state_s *bt_acl_state, BT_AVRCP_CMD_ID cmd_id, bool press);

/**
 * @brief Bluetooth AVRCP Register notification
 *        Set callback about AVRCP notification.
 *
 * @param[in] notificationFlag: int Notification Flags
 * @param[in] notifycb: notificationCallback call back function for notification.
 *
 * @retval error code
 */

int bt_register_notification(struct bt_avrcp_notify_ops_s *bt_avrcp_notify_ops);

/**
 * @brief Bluetooth AVRCP Register notification
 *        Set callback about AVRCP notification.
 *
 * @param[in] bt_avrcp_ops: bt_avrcp_ops_s callback funcs
 *
 * @retval error code
 */

int bt_avrcp_register_cb(struct bt_avrcp_ops_s *bt_avrcp_ops);

#endif /* __MODULES_INCLUDE_BLUETOOTH_API_BT_AVRCP_H */
