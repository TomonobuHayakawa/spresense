/****************************************************************************
 * bluetooth_spp/bluetooth_main.c
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

#include <stdio.h>
#include <bluetooth/bt_spp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void paringComplete(BT_ADDR addr, BT_PAIR_STATUS status);
static void connectStatusChanged(struct bt_acl_state_s *bt_acl_state, bool connected, int status);
static void bondInfo(BT_ADDR addr);
static void onSppConnect(struct bt_acl_state_s *bt_acl_state);
static int connectSPP(struct bt_acl_state_s *bt_acl_state);
static void receiveData(struct bt_acl_state_s *bt_acl_state, uint8_t *pdata, int len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bt_ble_common_ops_s bt_ble_common_ops =
  {
    .pairing_complete = paringComplete,
    .connect_status_changed = connectStatusChanged,
    .bond_info = bondInfo
  };

static struct bt_spp_ops_s bt_spp_ops =
  {
    .connect = onSppConnect,
    .receive_data = receiveData
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void paringComplete(BT_ADDR addr, BT_PAIR_STATUS status)
{
  /* If pairing task complete, this function will call.
   * Print receive event data.
   */

  printf("[BT_SPP] Pairing complete ADDR:%02X:02X:02X:02X:02X:02X, status=%d\n",
          addr.address[0], addr.address[1], addr.address[2],
          addr.address[3], addr.address[4], addr.address[5],
          status);
}

static void connectStatusChanged(struct bt_acl_state_s *bt_acl_state, bool connected, int status)
{
  /* If ACL is connected, SPP can start connect */

  if (connected)
    {
      /* Start to connect SPP */

      connectSPP(bt_acl_state);
    }
}

static void bondInfo(BT_ADDR addr)
{
  /* If new bonding is comming, this function will call.
   * Print new bonding information.
   */

  printf("[BT_SPP] Bonding information ADDR:%02X:02X:02X:02X:02X:02X\n",
          addr.address[0], addr.address[1], addr.address[2],
          addr.address[3], addr.address[4], addr.address[5]);
}

static void onSppConnect(struct bt_acl_state_s *bt_acl_state)
{
  /* If SPP connection is finished, this function will call. */

  printf("%s [BT] SPP connected\n", __func__);
}

static void receiveData(struct bt_acl_state_s *bt_acl_state, uint8_t *pdata, int len)
{
  /* If receive SPP data, this function will call. */

  printf("%s [BT] Receive Data data[0] = 0x%02X, Length = %d \n", __func__, pdata[0], len);
}

static int connectSPP(struct bt_acl_state_s *bt_acl_state)
{
  int ret = BT_SUCCESS;

  /* Check HAL supported SPP */

  if (bt_spp_is_supported())
    {
      /* Register SPP callback function */

      ret = bt_spp_register_cb(&bt_spp_ops);
      if (ret != BT_SUCCESS)
        {
          printf("%s [BT] Register callback failed. ret = %d\n", __func__, ret);
        }

      /* Start to connect SPP */

      ret = bt_spp_connect(bt_acl_state);
      if (ret != BT_SUCCESS)
        {
          printf("%s [BT] Connection request failed. ret = %d\n", __func__, ret);
        }
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * bt_spp_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int bt_spp_main(int argc, char *argv[])
#endif
{
  int ret = 0;

  /* Register BT event callback function */

  ret = bt_register_common_cb(&bt_ble_common_ops);
  if (ret != BT_SUCCESS)
  {
    printf("%s [BT] Register common call back failed. ret = %d\n", __func__, ret);
    goto error;
  }

  /* Initialize BT HAL */

  ret = bt_init();
  if (ret != BT_SUCCESS)
  {
    printf("%s [BT] Initialization failed. ret = %d\n", __func__, ret);
    goto error;
  }

  /* Turn ON BT */

  ret = bt_enable();
  if (ret != BT_SUCCESS)
  {
    printf("%s [BT] Enabling failed. ret = %d\n", __func__, ret);
    goto error;
  }

  /* Start pairing mode */

  ret = bt_pairing_enable();
  if (ret != BT_SUCCESS)
  {
    printf("%s [BT] Pairing enable failed. ret = %d\n", __func__, ret);
    goto error;
  }

  /* Set visible from other devices */

  ret = bt_set_visibility(BT_VIS_DISCOVERY_CONNECTABLE);
  if (ret != BT_SUCCESS)
  {
    printf("%s [BT] Set visible failed. ret = %d\n", __func__, ret);
    goto error;
  }

error:
  return ret;
}
