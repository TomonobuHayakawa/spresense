/****************************************************************************
 * modules/include/bluetooth/hal/bt_if.h
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

#ifndef __MODULES_INCLUDE_BLUETOOTH_HAL_BT_IF_H
#define __MODULES_INCLUDE_BLUETOOTH_HAL_BT_IF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/bluetooth_a2dp_codecs.h>
#include <bluetooth/bluetooth_avrcp_cmds.h>
#include <bluetooth/bluetooth_hfp_features.h>
#include <bluetooth/hal/bt_event.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/
/**
 * @defgroup bt_datatypes Data types
 * @{
 */

/** Bluetooth Common HAL callbacks
 */
struct bt_hal_common_ops_s
{
  int (*init)(void);
  int (*finalize)(void);
  int (*enable)(bool enable);
  int (*setDevAddr)(BT_ADDR *addr);
  int (*getDevAddr)(BT_ADDR *addr);
  int (*setDevName)(char *name);
  int (*getDevName)(char *name);
  int (*paringEnable)(bool enable);
  int (*getBondList)(BT_ADDR *addrs, int *num);
  int (*unBond)(BT_ADDR *addr);
  int (*setVisibility)(BT_VISIBILITY visibility);
  int (*inquiryStart)(void);
  int (*inquiryCancel)(void);
};

/** Bluetooth A2DP HAL callbacks
 */
struct bt_hal_a2dp_ops_s
{
  int (*connect)(BT_ADDR *addr, bool connect);
  int (*aacEnable)(bool enable);
  int (*vendorCodecEnable)(bool enable);
  int (*set_codec)(BT_AUDIO_CODEC_INFO *codec_info);
};

/** Bluetooth AVRCP HAL callbacks
 */
struct bt_hal_avrcp_ops_s
{
  int (*avrcc_connect)(BT_ADDR *addr, bool connect);
  int (*avrct_connect)(BT_ADDR *addr, bool connect);
  int (*send_avrcp_command)(BT_ADDR *addr, BT_AVRCP_CMD_ID cmd_id, bool press);
  int (*configure_notification)(BT_AVRC_SUPPORT_NOTIFY_EVENT *notification_list);
};

/** Bluetooth HFP HAL callbacks
 */
struct bt_hal_hfp_ops_s
{
  int (*connect)(BT_ADDR *addr, bool connect);
  int (*audio_connect)(BT_ADDR *addr, bool connect);
  int (*set_hf_feature)(BT_HFP_HF_FEATURE_FLAG hf_heature);
  int (*send_at_command)(BT_ADDR *addr, char *at_str);
};

/** Bluetooth SPP HAL callbacks
 */
struct bt_hal_spp_ops_s
{
  int (*connect)(BT_ADDR *addr, bool connect);
  int (*setUuid)(BT_UUID *uuid);
  int (*sendTxData)(BT_ADDR *addr, uint8_t *data, int len);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Bluetooth HAL register
 *
 * @retval error code
 */

int bt_common_register_hal(struct bt_hal_common_ops_s *bt_hal_common_ops);

/**
 * @brief Bluetooth HAL register
 *
 * @retval error code
 */

int bt_common_event_handler(struct bt_event_t *bt_event);

/**
 * @brief Bluetooth HAL register
 *
 * @retval error code
 */

int bt_a2dp_register_hal(struct bt_hal_a2dp_ops_s *bt_hal_a2dp_ops);

/**
 * @brief Bluetooth HAL register
 *
 * @retval error code
 */

int bt_a2dp_event_handler(struct bt_event_t *bt_event);

/**
 * @brief Bluetooth HAL register
 *
 * @retval error code
 */

int bt_avrcp_register_hal(struct bt_hal_avrcp_ops_s *bt_hal_avrcp_ops);

/**
 * @brief Bluetooth HAL register
 *
 * @retval error code
 */

int bt_avrcp_event_handler(struct bt_event_t *bt_event);

/**
 * @brief Bluetooth HAL register
 *
 * @retval error code
 */

int bt_hfp_register_hal(struct bt_hal_hfp_ops_s *bt_hal_hfp_ops);

/**
 * @brief Bluetooth HAL register
 *
 * @retval error code
 */

int bt_hfp_event_handler(struct bt_event_t *bt_event);

/**
 * @brief Bluetooth HAL register
 *
 * @retval error code
 */

int bt_spp_register_hal(struct bt_hal_spp_ops_s *bt_hal_spp_ops);

/**
 * @brief Bluetooth HAL register
 *
 * @retval error code
 */

int bt_spp_event_handler(struct bt_event_t *bt_event);

#endif /* __MODULES_INCLUDE_BLUETOOTH_HAL_BT_IF_H */
