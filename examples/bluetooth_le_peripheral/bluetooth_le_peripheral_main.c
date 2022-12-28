/****************************************************************************
 * bluetooth_le_peripheral/bluetooth_le_peripheral_main.c
 *
 *   Copyright 2018, 2022 Sony Semiconductor Solutions Corporation
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
#include <string.h>
#include <stdlib.h>
#include <bluetooth/ble_gatt.h>

#include "system/readline.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BLE_MAX_TX_DATA_SIZE     1024

#define BLE_UUID_SDS_SERVICE_IN  0x3802
#define BLE_UUID_SDS_CHAR_IN     0x4a02

#define BONDINFO_FILENAME "/mnt/spif/BONDINFO"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* BLE common callbacks */

/* Connection status change */

static void onLeConnectStatusChanged(struct ble_state_s *ble_state,
                                     bool connected);

/* Device name change */

static void onConnectedDeviceNameResp(const char *name);

/* Save bonding information */

static void onSaveBondInfo(int num, struct ble_bondinfo_s *bond);

/* Load bonding information */

static int onLoadBondInfo(int num, struct ble_bondinfo_s *bond);

/* BLE GATT callbacks */

/* Write request */

static void onWrite(struct ble_gatt_char_s *ble_gatt_char);

/* Read request */

static void onRead(struct ble_gatt_char_s *ble_gatt_char);

/* Notify request */

static void onNotify(struct ble_gatt_char_s *ble_gatt_char, bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ble_common_ops_s ble_common_ops =
  {
    .connect_status_changed     = onLeConnectStatusChanged,
    .connected_device_name_resp = onConnectedDeviceNameResp,
    .save_bondinfo              = onSaveBondInfo,
    .load_bondinfo              = onLoadBondInfo,
  };

static struct ble_gatt_peripheral_ops_s ble_gatt_peripheral_ops =
  {
    .write  = onWrite,
    .read   = onRead,
    .notify = onNotify
  };

static BT_ADDR local_addr               = {{0x19, 0x84, 0x06, 0x14, 0xAB, 0xCD}};

static char local_ble_name[BT_NAME_LEN] = "SONY_BLE";

static bool ble_is_connected = false;

static struct ble_gatt_service_s *g_ble_gatt_service;

static BLE_UUID128 service_in_uuid = {{0xfb, 0x34, 0x9b, 0x5f,  \
                                       0x80, 0x00, 0x00, 0x80,  \
                                       0x00, 0x10, 0x00, 0x00,  \
                                       0x00, 0x00, 0x00, 0x00}};

static BLE_UUID128 char_in_uuid = {{0x00, 0x34, 0x9b, 0x5f,  \
                                    0x80, 0x00, 0x00, 0x80,  \
                                    0x00, 0x10, 0x00, 0x00,  \
                                    0x00, 0x00, 0x00, 0x00}};

static BLE_ATTR_PERM attr_param =
  {
    .readPerm  = BLE_SEC_MODE1LV2_NO_MITM_ENC,
    .writePerm = BLE_SEC_MODE1LV2_NO_MITM_ENC
  };

static uint8_t char_data[BLE_MAX_CHAR_SIZE];

static BLE_CHAR_VALUE char_value =
  {
    .length = BLE_MAX_CHAR_SIZE
  };

static BLE_CHAR_PROP char_property =
  {
    .read   = 1,
    .write  = 1,
    .notify = 1
  };

static struct ble_gatt_char_s g_ble_gatt_char =
  {
    .handle = 0,
    .ble_gatt_peripheral_ops = &ble_gatt_peripheral_ops
  };

static int g_ble_bonded_device_num;
static struct ble_cccd_s **g_cccd = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void onLeConnectStatusChanged(struct ble_state_s *ble_state,
                                      bool connected)
{
  BT_ADDR addr = ble_state->bt_target_addr;

  /* If receive connected status data, this function will call. */

  printf("[BLE_GATT] Connect status ADDR:%02X:%02X:%02X:%02X:%02X:%02X, status:%s\n",
          addr.address[0], addr.address[1], addr.address[2],
          addr.address[3], addr.address[4], addr.address[5],
          connected ? "Connected" : "Disconnected");

  ble_is_connected = connected;
}

static void onConnectedDeviceNameResp(const char *name)
{
  /* If receive connected device name data, this function will call. */

  printf("%s [BLE] Receive connected device name = %s\n", __func__, name);
}

static void onSaveBondInfo(int num, struct ble_bondinfo_s *bond)
{
  int i;
  FILE *fp;
  int sz;

  /* In this example, save the parameter `num` and each members of
   * the parameter `bond` in order to the file.
   */

  fp = fopen(BONDINFO_FILENAME, "wb");
  if (fp == NULL)
    {
      printf("Error: could not create file %s\n", BONDINFO_FILENAME);
      return;
    }

  fwrite(&num, 1, sizeof(int), fp);

  for (i = 0; i < num; i++)
    {
      fwrite(&bond[i], 1, sizeof(struct ble_bondinfo_s), fp);

      /* Because only cccd is pointer member, save it individually. */

      sz = bond[i].cccd_num * sizeof(struct ble_cccd_s);
      fwrite(bond[i].cccd, 1, sz, fp);
    }

  fclose(fp);
}

static int onLoadBondInfo(int num, struct ble_bondinfo_s *bond)
{
  int i;
  FILE *fp;
  int stored_num;
  int sz;

  fp = fopen(BONDINFO_FILENAME, "rb");
  if (fp == NULL)
    {
      return 0;
    }

  fread(&stored_num, 1, sizeof(int), fp);
  g_ble_bonded_device_num = (stored_num < num) ? stored_num : num;
  sz = g_ble_bonded_device_num * sizeof(struct ble_cccd_s *);
  g_cccd = (struct ble_cccd_s **)malloc(sz);
  if (g_cccd == NULL)
    {
      printf("Error: could not load due to malloc error.\n");
      g_ble_bonded_device_num = 0;
    }

  for (i = 0; i < g_ble_bonded_device_num; i++)
    {
      fread(&bond[i], 1, sizeof(struct ble_bondinfo_s), fp);

      /* Because only cccd is pointer member, load it individually. */

      sz = bond[i].cccd_num * sizeof(struct ble_cccd_s);
      g_cccd[i] = (struct ble_cccd_s *)malloc(sz);

      if (g_cccd[i] == NULL)
        {
          printf("Error: could not load all data due to malloc error.");
          printf("The number of loaded device is %d\n", i);

          g_ble_bonded_device_num = i;
          break;
        }

      bond[i].cccd = g_cccd[i];
      fread(bond[i].cccd, 1, sz, fp);
    }

  fclose(fp);

  return g_ble_bonded_device_num;
}

static void free_cccd(void)
{
  int i;

  if (g_cccd)
    {
      for (i = 0; i < g_ble_bonded_device_num; i++)
        {
          if (g_cccd[i])
            {
              free(g_cccd[i]);
            }
        }

      free(g_cccd);
      g_cccd = NULL;
    }
}

static void show_uuid(BLE_UUID *uuid)
{
  int i;

  printf("uuid : ");

  switch (uuid->type)
    {
      case BLE_UUID_TYPE_UUID128:

        /* UUID format YYYYYYYY-YYYY-YYYY-YYYY-YYYYYYYYYYYY */

        for (i = 0; i < BT_UUID128_LEN; i++)
          {
            printf("%02x", uuid->value.uuid128.uuid128[BT_UUID128_LEN - i - 1]);
            if ((i == 3) || (i == 5) || (i == 7) || (i == 9))
              {
                printf("-");
              }
          }

        printf("\n");

        break;

      case BLE_UUID_TYPE_BASEALIAS_BTSIG:
      case BLE_UUID_TYPE_BASEALIAS_VENDOR:

        /* UUID format 0000XXXX-YYYY-YYYY-YYYY-YYYYYYYYYYYY (XXXX : alias) */

        printf("0000%04x-", uuid->value.alias.uuidAlias);

        for (i = 4 ; i < BT_UUID128_LEN; i++)
          {
            printf("%02x", uuid->value.alias.uuidBase.uuid128[BT_UUID128_LEN - i - 1]);
            if ((i == 5) || (i == 7) || (i == 9))
              {
                printf("-");
              }
          }

        printf("\n");

        break;

      default:
        printf("Irregular UUID type.\n");
        break;
    }
}

static void onWrite(struct ble_gatt_char_s *ble_gatt_char)
{
  int i;

  /* If receive connected device name data, this function will call. */

  printf("%s [BLE] start\n", __func__);
  printf("handle : %d\n", ble_gatt_char->handle);
  show_uuid(&ble_gatt_char->uuid);
  printf("value_len : %d\n", ble_gatt_char->value.length);
  printf("value : ");
  for (i = 0; i < ble_gatt_char->value.length; i++)
    {
      printf("%02x ", ble_gatt_char->value.data[i]);
    }

  printf("\n");

  printf("%s [BLE] end\n", __func__);
}

static void onRead(struct ble_gatt_char_s *ble_gatt_char)
{
  /* If receive connected device name data, this function will call. */

  printf("%s [BLE] \n", __func__);
}

static void onNotify(struct ble_gatt_char_s *ble_gatt_char, bool enable)
{
  /* If receive connected device name data, this function will call. */

  printf("%s [BLE] start \n", __func__);
  printf("handle : %d\n", ble_gatt_char->handle);
  show_uuid(&ble_gatt_char->uuid);

  if (enable)
    {
      printf("notification enabled\n");
    }
  else
    {
      printf("notification disabled\n");
    }

  printf("%s [BLE] end \n", __func__);
}

static void ble_peripheral_exit(void)
{
  int ret;

  /* Update connection status */

  ble_is_connected = false;

  /* Turn OFF BT */

  ret = bt_disable();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] BT disable failed. ret = %d\n", __func__, ret);
    }

  /* Finalize BT */

  ret = bt_finalize();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] BT finalize failed. ret = %d\n", __func__, ret);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * BLE_GATT_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret = 0;
  int len = 0;
  char buffer[BLE_MAX_TX_DATA_SIZE] = {0};
  BLE_UUID *s_uuid;
  BLE_UUID *c_uuid;

  /* Initialize BT HAL */

  ret = bt_init();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Initialization failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Register BLE common callbacks */

  ret = ble_register_common_cb(&ble_common_ops);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Register common call back failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Turn ON BT */

  ret = bt_enable();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BT] Enabling failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Free memory that is allocated in onLoadBond() callback function. */

  free_cccd();

  /* BLE set name */

  ret = ble_set_name(local_ble_name);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Set name failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* BLE set address */

  ret = ble_set_address(&local_addr);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Set address failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* BLE enable */

  ret = ble_enable();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Enable failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* BLE create GATT service instance */

  ret = ble_create_service(&g_ble_gatt_service);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Create GATT service failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Setup Service */

  /* Get Service UUID pointer */

  s_uuid = &g_ble_gatt_service->uuid;

  /* Setup Service UUID */

  s_uuid->type                  = BLE_UUID_TYPE_BASEALIAS_BTSIG;
  s_uuid->value.alias.uuidAlias = BLE_UUID_SDS_SERVICE_IN;
  memcpy(&s_uuid->value.alias.uuidBase, &service_in_uuid, sizeof(BLE_UUID128));

  /* Setup Characteristic */

  /* Get Characteristic UUID pointer */

  c_uuid = &g_ble_gatt_char.uuid;

  /* Setup Characteristic UUID */

  c_uuid->type =BLE_UUID_TYPE_BASEALIAS_VENDOR;
  c_uuid->value.alias.uuidAlias = BLE_UUID_SDS_CHAR_IN;
  memcpy(&c_uuid->value.alias.uuidBase, &char_in_uuid, sizeof(BLE_UUID128));

  /* Set data point */

  char_value.data = char_data;

  /* Setup Characteristic BLE_ATTR_PERM */

  memcpy(&char_value.attrPerm, &attr_param, sizeof(BLE_ATTR_PERM));

  /* Setup Characteristic BLE_CHAR_VALUE */

  memcpy(&g_ble_gatt_char.value, &char_value, sizeof(BLE_CHAR_VALUE));

  /* Setup Characteristic BLE_CHAR_PROP */

  memcpy(&g_ble_gatt_char.property, &char_property, sizeof(BLE_CHAR_PROP));

  /* BLE add GATT characteristic into service */

  ret = ble_add_characteristic(g_ble_gatt_service, &g_ble_gatt_char);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Add GATT characteristic failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* BLE register GATT service */

  ret = ble_register_servce(g_ble_gatt_service);
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Register GATT service failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* BLE start advertise */

  ret = ble_start_advertise();
  if (ret != BT_SUCCESS)
    {
      printf("%s [BLE] Start advertise failed. ret = %d\n", __func__, ret);
      goto error;
    }

  /* Send Tx data by using readline */

  while(1)
    {
      printf("ble_peripheral>");
      fflush(stdout);

      len = readline(buffer, sizeof(buffer) - 1, stdin, stdout);

      if (ble_is_connected)
        {
          ret = ble_characteristic_notify(&g_ble_gatt_char, (uint8_t *) buffer, len);
          if (ret != BT_SUCCESS)
            {
              printf("%s [BLE] Send data failed. ret = %d\n", __func__, ret);
            }
        }

      if (!strcmp(buffer, "quit\n"))
        {
          printf("Quit.\n");
          break;
        }
    }

  /* Quit this application */

  ble_peripheral_exit();

error:
  return ret;
}
