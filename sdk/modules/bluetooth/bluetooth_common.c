/****************************************************************************
 * modules/bluetooth/bluetooth_common.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>
#include <bluetooth/bt_common.h>
#include <bluetooth/hal/bt_if.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bt_common_state_s g_bt_common_state =
{
  .bt_name  = "SONY-BT-CLASSIC",
  .ble_name = "SONY-BLE-CLASSIC",
  .bt_addr  = {{0x20, 0x70, 0x3A, 0x10, 0x00, 0x01}},
  .ble_addr = {{0x20, 0x70, 0x3A, 0x10, 0x00, 0x01}}
};

static struct bt_acl_state_s g_bt_acl_state =
{
  .bt_acl_connection = BT_DISCONNECTED,
  .bt_common_state   = &g_bt_common_state
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int event_cmd_status(struct bt_event_cmd_stat_t *cmd_stat_evt)
{
  int ret = BT_SUCCESS;
  struct bt_ble_common_ops_s *bt_ble_common_ops = g_bt_common_state.bt_ble_common_ops;

  if (bt_ble_common_ops && bt_ble_common_ops->command_status)
    {
      bt_ble_common_ops->command_status(cmd_stat_evt->cmd_status);
    }
  else
    {
      _err("%s [BT][Common] Command status callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_pairing_complete(struct bt_event_pair_cmplt_t *pair_cmplt)
{
  int ret = BT_SUCCESS;
  struct bt_ble_common_ops_s *bt_ble_common_ops = g_bt_common_state.bt_ble_common_ops;

  if (bt_ble_common_ops && bt_ble_common_ops->pairing_complete)
    {
      bt_ble_common_ops->pairing_complete(pair_cmplt->addr, pair_cmplt->status);
    }
  else
    {
      _err("%s [BT][Common] Command status callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_inquiry_result(struct bt_event_inquiry_rslt_t *inq_result_evt)
{
  int ret = BT_SUCCESS;
  struct bt_ble_common_ops_s *bt_ble_common_ops = g_bt_common_state.bt_ble_common_ops;

  if (bt_ble_common_ops && bt_ble_common_ops->inquiry_result)
    {
      bt_ble_common_ops->inquiry_result(inq_result_evt->addr, inq_result_evt->name);
    }
  else
    {
      _err("%s [BT][Common] Command status callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_inquiry_complete(void)
{
  int ret = BT_SUCCESS;
  struct bt_ble_common_ops_s *bt_ble_common_ops = g_bt_common_state.bt_ble_common_ops;

  if (bt_ble_common_ops && bt_ble_common_ops->inquiry_complete)
    {
      bt_ble_common_ops->inquiry_complete();
    }
  else
    {
      _err("%s [BT][Common] Command status callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_conn_stat_change(struct bt_event_conn_stat_t *conn_stat_evt)
{
  int ret = BT_SUCCESS;
  struct bt_ble_common_ops_s *bt_ble_common_ops = g_bt_common_state.bt_ble_common_ops;

  if (conn_stat_evt->connected)
    {
      g_bt_acl_state.bt_target_addr = conn_stat_evt->addr;
      g_bt_acl_state.bt_acl_connection = BT_CONNECTED;
    }
  else
    {
      g_bt_acl_state.bt_acl_connection = BT_DISCONNECTED;
    }

  if (bt_ble_common_ops && bt_ble_common_ops->connect_status_changed)
    {
      bt_ble_common_ops->connect_status_changed(&g_bt_acl_state, conn_stat_evt->connected, conn_stat_evt->status);
    }
  else
    {
      _err("%s [BT][Common] Connect status callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_conn_dev_name(struct bt_event_dev_name_t *dev_name_evt)
{
  int ret = BT_SUCCESS;
  struct bt_ble_common_ops_s *bt_ble_common_ops = g_bt_common_state.bt_ble_common_ops;

  if (bt_ble_common_ops && bt_ble_common_ops->connected_device_name)
    {
      memcpy(g_bt_acl_state.bt_target_name, dev_name_evt->name, BT_NAME_LEN);
      bt_ble_common_ops->connected_device_name(dev_name_evt->name);
    }
  else
    {
      _err("%s [BT][Common] Command status callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

static int event_bond_info(struct bt_event_bond_info_t *bond_info_evt)
{
  int ret = BT_SUCCESS;
  struct bt_ble_common_ops_s *bt_ble_common_ops = g_bt_common_state.bt_ble_common_ops;

  if (bt_ble_common_ops && bt_ble_common_ops->bond_info)
    {
      g_bt_acl_state.bt_target_addr = bond_info_evt->addr;
      bt_ble_common_ops->bond_info(bond_info_evt->addr);
    }
  else
    {
      _err("%s [BT][Common] Bonding information callback failed(CB not registered).\n", __func__);
      return BT_FAIL;
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_init
 *
 * Description:
 *   Initialize BT module(File system, Pin config, UART config, etc).
 *
 ****************************************************************************/

int bt_init(void)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->init)
    {
      ret = bt_hal_common_ops->init();
    }
  else
    {
      _err("%s [BT][Common] Initialization failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_finalize
 *
 * Description:
 *   Finalize BT module.
 *
 ****************************************************************************/

int bt_finalize(void)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->finalize)
    {
      ret = bt_hal_common_ops->finalize();
    }
  else
    {
      _err("%s [BT][Common] Connect failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_set_address
 *
 * Description:
 *   Set bluetooth device address.
 *   If not set address, use default address .
 *
 ****************************************************************************/

int bt_set_address(BT_ADDR *addr)
{
  int ret = BT_SUCCESS;

  if (!addr)
    {
      _err("%s [BT][Common] Set local BT address failed(addr not set).\n", __func__);
      return BT_FAIL;
    }

  memcpy(&g_bt_common_state.bt_addr, addr, sizeof(BT_ADDR));
  return ret;
}

/****************************************************************************
 * Name: bt_get_address
 *
 * Description:
 *   Get current bluetooth device address.
 *
 ****************************************************************************/

int bt_get_address(BT_ADDR *addr)
{
  int ret = BT_SUCCESS;
  BT_ADDR target_addr;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (!addr)
    {
      _err("%s [BT][Common] addr not allocated.\n", __func__);
      return BT_FAIL;
    }

  if (bt_hal_common_ops && bt_hal_common_ops->getDevAddr)
    {
      ret = bt_hal_common_ops->getDevAddr(&target_addr);

      memcpy(addr, &target_addr, sizeof(BT_ADDR));
    }
  else
    {
      _err("%s [BT][Common] Get local address failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_set_name
 *
 * Description:
 *   Set bluetooth device name.
 *
 ****************************************************************************/

int bt_set_name(char *name)
{
  int ret = BT_SUCCESS;

  if (!name)
    {
      _err("%s [BT][Common] Set BT name failed(name not set).\n", __func__);
      return BT_FAIL;
    }

  memcpy(g_bt_common_state.bt_name, name, sizeof(char) * BT_NAME_LEN);
  return ret;
}

/****************************************************************************
 * Name: bt_get_name
 *
 * Description:
 *   Get current bluetooth device name.
 *
 ****************************************************************************/

int bt_get_name(char *name)
{
  int ret = BT_SUCCESS;
  char target_name[BT_NAME_LEN];
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (!name)
    {
      _err("%s [BT][Common] name not allocated.\n", __func__);
      return BT_FAIL;
    }

  if (bt_hal_common_ops && bt_hal_common_ops->getDevName)
    {
      ret = bt_hal_common_ops->getDevName(target_name);
      memcpy(name, target_name, sizeof(char) * BT_NAME_LEN);
    }
  else
    {
      _err("%s [BT][Common] Get local name failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_enable
 *
 * Description:
 *   Enable bluetooth module(Power ON, Reset, Firmware load, set device name, etc).
 *
 ****************************************************************************/

int bt_enable(void)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->enable)
    {
      ret = bt_hal_common_ops->enable(true);
    }
  else
    {
      _err("%s [BT][Common] Enabling failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  if (bt_hal_common_ops && bt_hal_common_ops->setDevAddr)
    {
      ret = bt_hal_common_ops->setDevAddr(&g_bt_common_state.bt_addr);
    }
  else
    {
      _err("%s [BT][Common] Set local address failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  if (bt_hal_common_ops && bt_hal_common_ops->setDevName)
    {
      ret = bt_hal_common_ops->setDevName(g_bt_common_state.bt_name);
    }
  else
    {
      _err("%s [BT][Common] Set local name failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_disable
 *
 * Description:
 *   Disable bluetooth module(Power OFF).
 *
 ****************************************************************************/

int bt_disable(void)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->enable)
    {
      ret = bt_hal_common_ops->enable(false);
    }
  else
    {
      _err("%s [BT][Common] Disabling failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_pairing_enable
 *
 * Description:
 *   Start pairing mode with pairing .
 *
 ****************************************************************************/

int bt_pairing_enable(void)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->paringEnable)
    {
      ret = bt_hal_common_ops->paringEnable(true);
    }
  else
    {
      _err("%s [BT][Common] Paring enable failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_paring_disable
 *
 * Description:
 *   Cancel pairing mode.
 *
 ****************************************************************************/

int bt_paring_disable(void)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->paringEnable)
    {
      ret = bt_hal_common_ops->paringEnable(false);
    }
  else
    {
      _err("%s [BT][Common] Paring disable failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_get_bond_list
 *
 * Description:
 *   Get bonding device BT_ADDR list.
 *
 ****************************************************************************/

int bt_get_bond_list(BT_ADDR *addr, int *num)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->getBondList)
    {
      ret = bt_hal_common_ops->getBondList(addr, num);
    }
  else
    {
      _err("%s [BT][Common] Get bonding list failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_unbond
 *
 * Description:
 *   Unbond device by BT_ADDR.
 *
 ****************************************************************************/

int bt_unbond(BT_ADDR *addr)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->unBond)
    {
      ret = bt_hal_common_ops->unBond(addr);
    }
  else
    {
      _err("%s [BT][Common] Get bonding list failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_set_visible
 *
 * Description:
 *   Set bluetooth device to visible for connect from peer device.
 *
 ****************************************************************************/

int bt_set_visibility(BT_VISIBILITY visibility)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->setVisibility)
    {
      ret = bt_hal_common_ops->setVisibility(visibility);
    }
  else
    {
      _err("%s [BT][Common] Set visibility failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_start_inquiry
 *
 * Description:
 *   Start inquiry to device by name.
 *
 ****************************************************************************/

int bt_start_inquiry(void)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->inquiryStart)
    {
      ret = bt_hal_common_ops->inquiryStart();
    }
  else
    {
      _err("%s [BT][Common] Inquiry start(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_cancel_inquiry
 *
 * Description:
 *   Cancel to current inquiry.
 *
 ****************************************************************************/

int bt_cancel_inquiry(void)
{
  int ret = BT_SUCCESS;
  struct bt_hal_common_ops_s *bt_hal_common_ops = g_bt_common_state.bt_hal_common_ops;

  if (bt_hal_common_ops && bt_hal_common_ops->inquiryCancel)
    {
      ret = bt_hal_common_ops->inquiryCancel();
    }
  else
    {
      _err("%s [BT][Common] Inquiry cancel failed(HAL not registered).\n", __func__);
      return BT_FAIL;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_register_common_cb
 *
 * Description:
 *   Register A2DP generic BT operation callback for Application.
 *
 ****************************************************************************/

int bt_register_common_cb(struct bt_ble_common_ops_s *bt_ble_common_ops)
{
  if (!bt_ble_common_ops)
    {
      _err("%s [BT][Common] Set application callback failed.\n", __func__);
      return BT_FAIL;
    }

  g_bt_common_state.bt_ble_common_ops = bt_ble_common_ops;

  return BT_SUCCESS;
}

/****************************************************************************
 * Name: bt_common_register_hal
 *
 * Description:
 *   Register common HAl interface.
 *
 ****************************************************************************/

int bt_common_register_hal(struct bt_hal_common_ops_s *bt_hal_common_ops)
{
  if (!bt_hal_common_ops)
    {
      _err("%s [BT][Common] Set HAL callback failed.\n", __func__);
      return BT_FAIL;
    }

  g_bt_common_state.bt_hal_common_ops = bt_hal_common_ops;

  return BT_SUCCESS;
}

/****************************************************************************
 * Name: bt_common_event_handler
 *
 * Description:
 *   Handler of generic event.
 *   Receive generic event from HAL and dispatch event for application.
 *
 ****************************************************************************/

int bt_common_event_handler(struct bt_event_t *bt_event)
{
  switch (bt_event->event_id)
    {
      case BT_COMMON_EVENT_CMD_STATUS:
        return event_cmd_status((struct bt_event_cmd_stat_t *) bt_event);

      case BT_COMMON_EVENT_PAIRING_COMPLETE:
        return event_pairing_complete((struct bt_event_pair_cmplt_t *) bt_event);

      case BT_COMMON_EVENT_INQUIRY_RESULT:
        return event_inquiry_result((struct bt_event_inquiry_rslt_t *) bt_event);

      case BT_COMMON_EVENT_INQUIRY_COMPLETE:
        return event_inquiry_complete();

      case BT_COMMON_EVENT_CONN_STAT_CHANGE:
        return event_conn_stat_change((struct bt_event_conn_stat_t *) bt_event);

      case BT_COMMON_EVENT_CONN_DEV_NAME:
        return event_conn_dev_name((struct bt_event_dev_name_t *) bt_event);

      case BT_COMMON_EVENT_BOND_INFO:
        return event_bond_info((struct bt_event_bond_info_t *) bt_event);

      default:
        break;
    }
  return BT_SUCCESS;
}
