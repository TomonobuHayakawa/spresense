/****************************************************************************
 * modules/include/bluetooth/api/bt_a2dp.h
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

#ifndef __MODULES_INCLUDE_BLUETOOTH_API_BT_A2DP_H
#define __MODULES_INCLUDE_BLUETOOTH_API_BT_A2DP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/bluetooth_a2dp_codecs.h>
#include <bluetooth/api/bt_common.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/
/**
 * @defgroup bt_defs Defines
 * @{
 */

/** Bluetooth A2DP application callbacks
 */
struct bt_a2dp_ops_s
{
  void (*command_status)(BT_CMD_STATUS status);
  void (*connect)(struct bt_acl_state_s *bt_acl_state, BT_AUDIO_CODEC_INFO codecInfo);
  void (*disconnect)(struct bt_acl_state_s *bt_acl_state);
  void (*receive_media_pkt)(struct bt_acl_state_s *bt_acl_state, uint8_t *data, int len);
};

/** Bluetooth A2DP context
 */
struct bt_a2dp_state_s
{
  BT_CONNECT_STATUS        bt_a2dp_connection; /* Status of A2DP connection */
  struct bt_acl_state_s    *bt_acl_state;      /* Bluetooth ACL context */
  struct bt_hal_a2dp_ops_s *bt_hal_a2dp_ops;   /* A2DP HAL interfaces */
  struct bt_a2dp_ops_s     *bt_a2dp_ops;       /* A2DP connection callbacks */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Get A2DP support or not support
 *
 * @retval Suppot or Not support
 */

bool bt_a2dp_is_supported(void);

/**
 * @brief Bluetooth A2DP connect
 *        Connect to peer device with A2DP.
 *
 * @param[in] addr: BT_ADDR* device BD_ADDR to connect
 *
 * @retval error code
 */

int bt_a2dp_connect(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth A2DP disconnect
 *        Disconnect to peer device with A2DP.
 *
 * @param[in] addr: BT_ADDR* device BD_ADDR to disconnect
 *
 * @retval error code
 */

int bt_a2dp_disconnect(struct bt_acl_state_s *bt_acl_state);

/**
 * @brief Bluetooth A2DP set codec capability
 *        Set capability of audio codecs
 *
 * @param[in] codec: BT_A2DP_CODEC_TYPE Codec type list(SBC/AAC).
 *
 * @retval error code
 */

int bt_a2dp_set_codec_capability(BT_AUDIO_CODEC_INFO *codec_capabilities, uint8_t num);

/**
 * @brief Register media packet callback
 *
 * @param[in] a2dp_cb: rxCallback A2DP media packet callback function.
 *
 * @retval error code
 */

int bt_register_media_packet_callback(rx_callback a2dp_cb);

/**
 * @brief Register A2DP event packet callback
 *
 * @param[in] a2dpCb: rxCallback A2DP media packet callback function.
 *
 * @retval error code
 */

int bt_a2dp_register_callback(struct bt_a2dp_ops_s *bt_a2dp_ops);

#endif /* __MODULES_INCLUDE_BLUETOOTH_API_BT_A2DP_H */
