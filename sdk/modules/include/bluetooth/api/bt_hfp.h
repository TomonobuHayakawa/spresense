/****************************************************************************
 * modules/include/bluetooth/bluetooth_hfp.h
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

#ifndef __MODULES_INCLUDE_BLUETOOTH_API_BT_HFP_H
#define __MODULES_INCLUDE_BLUETOOTH_API_BT_HFP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/bluetooth_hfp_features.h>
#include <bluetooth/api/bt_common.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/
/**
 * @defgroup bt_datatypes Data types
 * @{
 */

/** Bluetooth HFP application callbacks
 */
struct bt_hfp_ops_s
{
  void (*command_status)(BT_CMD_STATUS status);
  void (*connect)(struct bt_acl_state_s *bt_acl_state, BT_PROFILE_TYPE btProfileType);
  void (*disconnect)(struct bt_acl_state_s *bt_acl_state);
  void (*audio_connect)(struct bt_acl_state_s *bt_acl_state);
  void (*audio_disconnect)(struct bt_acl_state_s *bt_acl_state);
  void (*ag_feature)(struct bt_acl_state_s *bt_acl_state, BT_HFP_AG_FEATURE_FLAG feature);
  void (*hf_at_response)(struct bt_acl_state_s *bt_acl_state, char *at_resp);
};

/** Bluetooth HFP context
 */
struct bt_hfp_state_s
{
  BT_CONNECT_STATUS        bt_hfp_connection;       /* Status of HFP connection */
  BT_CONNECT_STATUS        bt_hfp_audio_connection; /* Status of HFP audio connection */
  struct bt_acl_state_s    *bt_acl_state;           /* Bluetooth ACL context */
  struct bt_hal_hfp_ops_s  *bt_hal_hfp_ops;         /* HFP HAL interfaces */
  struct bt_hfp_ops_s      *bt_hfp_ops;             /* HFP connection callbacks */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Get HFP support or not support
 *
 * @retval Suppot or Not support
 */

bool bt_hfp_is_supported(void);

/**
 * @brief Bluetooth HFP connect
 *        Connect to peer device with HFP.
 *
 * @param[in] addr: BT_ADDR* device BD_ADDR to connect
 *
 * @retval error code
 */

int bt_hfp_connect(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth HFP disconnect
 *        Disconnect to peer device with HFP.
 *
 * @param[in] addr: BT_ADDR* device BD_ADDR to disconnect
 *
 * @retval error code
 */

int bt_hfp_disconnect(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth HFP Audio connect
 *        Connect to peer device with HFP Audio.
 *
 * @param[in] addr: BT_ADDR* device BD_ADDR to connect
 *
 * @retval error code
 */

int bt_hfp_audio_connect(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth HFP Audio disconnect
 *        Disconnect to peer device with HFP Audio.
 *
 * @param[in] addr: BT_ADDR* device BD_ADDR to disconnect
 *
 * @retval error code
 */

int bt_hfp_audio_disconnect(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth send HFP command
 *        Send HFP command.
 *
 * @param[in] addr: BT_ADDR* device BD_ADDR to send command
 * @param[in] commandID: int command ID(TBD).
 *
 * @retval error code
 */

int bt_hfp_send_at_command(struct bt_acl_state_s *bt_acl_state, char *at_cmd_str);

/**
 * @brief Bluetooth HFP Register callbacks
 *        Set callback about HFP.
 *
 * @param[in] bt_hfp_ops: bt_hfp_ops_s HFP callbacks
 *
 * @retval error code
 */

int bt_hfp_register_cb(struct bt_hfp_ops_s *bt_hfp_ops);

#endif /* __MODULES_INCLUDE_BLUETOOTH_API_BT_HFP_H */
