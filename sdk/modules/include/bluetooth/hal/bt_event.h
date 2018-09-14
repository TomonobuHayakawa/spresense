/****************************************************************************
 * modules/include/bluetooth/hal/bt_event.h
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

#ifndef __MODULES_INCLUDE_BLUETOOTH_HAL_BT_EVENT_H
#define __MODULES_INCLUDE_BLUETOOTH_HAL_BT_EVENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <bluetooth/bluetooth.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
 *@name BT Address Length
 *@{
 */
#define BT_MAX_EVENT_DATA_LEN 1024
/** @} */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/** Bluetooth profile and protocol ID
 */
typedef enum
{
	BT_GROUP_COMMON = 0,
	BT_GROUP_A2DP,
	BT_GROUP_AVRCP,
	BT_GROUP_HFP,
	BT_GROUP_SPP,
	BT_GROUP_RFCOMM,
} BT_GROUP_ID;

/** Bluetooth event ID for common function
 */
typedef enum
{
	BT_COMMON_EVENT_CMD_STATUS = 0,
	BT_COMMON_EVENT_PAIRING_COMPLETE,
	BT_COMMON_EVENT_INQUIRY_RESULT,
	BT_COMMON_EVENT_INQUIRY_COMPLETE,
	BT_COMMON_EVENT_CONN_STAT_CHANGE,
	BT_COMMON_EVENT_CONN_DEV_NAME,
	BT_COMMON_EVENT_BOND_INFO,
} BT_COMMON_EVENT_ID;

/** Bluetooth event ID for A2DP
 */
typedef enum
{
	BT_A2DP_EVENT_CMD_STATUS = 0,
	BT_A2DP_EVENT_CONNECT,
	BT_A2DP_EVENT_DISCONNECT,
	BT_A2DP_EVENT_MEDIA_PACKET,
} BT_A2DP_EVENT_ID;

/** Bluetooth event ID for A2DP
 */
typedef enum
{
	BT_AVRCP_EVENT_CMD_STATUS = 0,
	BT_AVRCC_EVENT_CONNECT,
	BT_AVRCC_EVENT_DISCONNECT,
	BT_AVRCT_EVENT_CONNECT,
	BT_AVRCT_EVENT_DISCONNECT,
	BT_AVRCP_EVENT_PLAY_STAT_CHANGE,
	BT_AVRCP_EVENT_TRACK_CHANGE,
	BT_AVRCP_EVENT_TRACK_REACH_END,
	BT_AVRCP_EVENT_TRACK_REACH_START,
	BT_AVRCP_EVENT_PLAY_POS_CHANGE,
	BT_AVRCP_EVENT_BATT_STAT_CHANGE,
	BT_AVRCP_EVENT_SYS_STATUS_CHANGE,
	BT_AVRCP_EVENT_APP_SETT_CHANGE,
	BT_AVRCP_EVENT_NOW_PLAY_CHANGE,
	BT_AVRCP_EVENT_AVAI_PLAYER_CHANGE,
	BT_AVRCP_EVENT_ADDR_PLAYER_CHANGE,
	BT_AVRCP_EVENT_UIDS_CHANGE,
	BT_AVRCP_EVENT_VOLUME_CHANGE,
} BT_AVRCP_EVENT_ID;

/** Bluetooth event ID for HFP
 */
typedef enum
{
	BT_HFP_EVENT_CMD_STATUS = 0,
	BT_HFP_EVENT_HF_CONNECT,
	BT_HFP_EVENT_HF_DISCONNECT,
	BT_HFP_EVENT_AUDIO_CONNECT,
	BT_HFP_EVENT_AUDIO_DISCONNECT,
	BT_HFP_EVENT_AG_FEATURE_RESP,
	BT_HFP_EVENT_AT_CMD_RESP,
} BT_HFP_EVENT_ID;

/** Bluetooth event ID for SPP
 */
typedef enum
{
	BT_SPP_EVENT_CONNECT = 0,
	BT_SPP_EVENT_DISCONNECT,
	BT_SPP_EVENT_CONNECT_FAIL,
	BT_SPP_EVENT_RX_DATA,
} BT_SPP_EVENT_ID;

/** Bluetooth general event data type
 */
struct bt_event_t
{
  uint8_t group_id;
  uint8_t event_id;
  uint8_t data[BT_MAX_EVENT_DATA_LEN];
};

/** Bluetooth command status event data type
 */
struct bt_event_cmd_stat_t
{
  uint8_t group_id;
  uint8_t event_id;
  BT_CMD_STATUS cmd_status;
};

/** Bluetooth pairing complete event data type
 */
struct bt_event_pair_cmplt_t
{
  uint8_t group_id;
  uint8_t event_id;
  BT_ADDR addr;
  BT_PAIR_STATUS status;
};

/** Bluetooth inquiry result event data type
 */
struct bt_event_inquiry_rslt_t
{
  uint8_t group_id;
  uint8_t event_id;
  BT_ADDR addr;
  char name[BT_NAME_LEN];
};

/** Bluetooth connection status change event data type
 */
struct bt_event_conn_stat_t
{
  uint8_t group_id;
  uint8_t event_id;
  BT_ADDR addr;
  bool connected;
  uint8_t status;
};

/** Bluetooth change device name event data type
 */
struct bt_event_dev_name_t
{
  uint8_t group_id;
  uint8_t event_id;
  char name[BT_NAME_LEN];
};

/** Bluetooth change device name event data type
 */
struct bt_event_bond_info_t
{
  uint8_t group_id;
  uint8_t event_id;
  BT_ADDR addr;
};

/** Bluetooth A2DP connection event data type
 */
struct bt_a2dp_event_connect_t
{
  uint8_t group_id;
  uint8_t event_id;
  BT_ADDR addr;
  BT_AUDIO_CODEC_INFO codecInfo;
};

/** Bluetooth A2DP receive data event data type
 */
struct bt_a2dp_event_recv_t
{
  uint8_t group_id;
  uint8_t event_id;
  BT_ADDR addr;
  uint8_t data[BT_MAX_EVENT_DATA_LEN];
  int len;
};

/** Bluetooth AVRCP connection event data type
 */
struct bt_avrcp_event_connect_t
{
  uint8_t group_id;
  uint8_t event_id;
  BT_ADDR addr;
};

/** Bluetooth HFP connection event data type
 */
struct bt_hfp_event_connect_t
{
  uint8_t group_id;
  uint8_t event_id;
  BT_ADDR addr;
  BT_PROFILE_TYPE hfp_type;
};

/** Bluetooth HFP ag feature event data type
 */
struct bt_hfp_event_ag_feature_t
{
  uint8_t group_id;
  uint8_t event_id;
  BT_ADDR addr;
  BT_HFP_AG_FEATURE_FLAG ag_flag;
};

/** Bluetooth HFP at command event data type
 */
struct bt_hfp_event_at_cmd_t
{
  uint8_t group_id;
  uint8_t event_id;
  BT_ADDR addr;
  char at_resp[BT_MAX_EVENT_DATA_LEN];
};

/** Bluetooth SPP connection event data type
 */
struct bt_spp_event_connect_t
{
  uint8_t group_id;
  uint8_t event_id;
  BT_ADDR addr;
  BT_CONNECT_FAIL_REASON_ID reason;
};

/** Bluetooth SPP Rx event data type
 */
struct bt_spp_event_recv_data_t
{
  uint8_t group_id;
  uint8_t event_id;
  BT_ADDR addr;
  uint8_t data[BT_MAX_EVENT_DATA_LEN];
  int len;
};

/**@brief BT avrc supporter notify event
*/
typedef struct {
	bool playStatusChange;          /**< Playback Status Changed */
	bool trackChange;               /**< Track Changed */
	bool trackReachedEnd;           /**< Track End Reached */
	bool trackReachedStart;         /**< Track Reached Start */
	bool playPosChanged;            /**< Playback position changed */
	bool batteryStatusChange;       /**< Battery status changed */
	bool systemStatusChange;        /**< System status changed */
	bool appSettingChange;          /**< Player application settings changed */
	bool nowPlayingChange;          /**< Now Playing Content Changed (AVRCP 1.4) */
	bool avalPlayerChange;          /**< Available Players Changed Notification (AVRCP 1.4) */
	bool addrPlayChange;            /**< Addressed Player Changed Notification (AVRCP 1.4) */
	bool uidsChange;                /**< UIDs Changed Notification (AVRCP 1.4) */
	bool volumeChange;              /**< Notify Volume Change (AVRCP 1.4) */
} BT_AVRC_SUPPORT_NOTIFY_EVENT;

#endif /* __MODULES_INCLUDE_BLUETOOTH_HAL_BT_EVENT_H */
