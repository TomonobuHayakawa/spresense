/****************************************************************************
 * modules/bluetooth/bluetooth_le_gatt.c
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

#include <bluetooth/ble_gatt.h>
#include <bluetooth/hal/bt_if.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ble_gatt_is_supported
 *
 * Description:
 *   Get Bluetooth Low Ennergy GATT support or not support
 *
 ****************************************************************************/

bool ble_gatt_is_supported(void)
{
  return true;
}

/****************************************************************************
 * Name: ble_create_service
 *
 * Description:
 *   BLE Create GATT Service
 *   Create GATT Service instance and return instance pointer via *service.
 *
 ****************************************************************************/

int ble_create_service(struct ble_gatt_service_s *service)
{
  return BT_SUCCESS;
}

/****************************************************************************
 * Name: ble_register_servce
 *
 * Description:
 *   BLE Register GATT Service
 *   Register GATT Service to HAL.
 *
 ****************************************************************************/

int ble_register_servce(struct ble_gatt_service_s *service)
{
  return BT_SUCCESS;
}

/****************************************************************************
 * Name: ble_add_characteristic
 *
 * Description:
 *   BLE add Characteristic to service
 *
 ****************************************************************************/

int ble_add_characteristic(struct ble_gatt_service_s *service, struct ble_gatt_char_s *charc)
{
  return BT_SUCCESS;
}

/****************************************************************************
 * Name: ble_characteristic_notify
 *
 * Description:
 *   BLE Notify Characteristic value
 *   Notify characteristic value to Central (For Peripheral role)
 *
 ****************************************************************************/

int ble_characteristic_notify(struct ble_gatt_char_s *charc, uint8_t *data, int len)
{
  return BT_SUCCESS;
}

/****************************************************************************
 * Name: ble_characteristic_read
 *
 * Description:
 *   BLE Read Characteristic value
 *   Send read characteristic request to peripheral (For Central role)
 *
 ****************************************************************************/

int ble_characteristic_read(struct ble_gatt_char_s *charc)
{
  return BT_SUCCESS;
}

/****************************************************************************
 * Name: ble_characteristic_write
 *
 * Description:
 *   BLE Write Characteristic value
 *   Send write characteristic request to peripheral (For Central role)
 *
 ****************************************************************************/

int ble_characteristic_write(struct ble_gatt_char_s *charc, uint8_t *data, int len)
{
  return BT_SUCCESS;
}

/****************************************************************************
 * Name: ble_gatt_register_hal
 *
 * Description:
 *   Bluetooth LE GATT function HAL register
 *
 ****************************************************************************/

int ble_gatt_register_hal(struct ble_hal_gatt_ops_s *ble_hal_gatt_ops)
{
  return BT_SUCCESS;
}

/****************************************************************************
 * Name: ble_gatt_event_handler
 *
 * Description:
 *   BLE GATT event handler
 *   HAL should call this function if receive BLE GATT event(@ref BLE_GATT_EVENT_ID).
 *
 ****************************************************************************/

int ble_gatt_event_handler(struct bt_event_t *bt_event)
{
  return BT_SUCCESS;
}

