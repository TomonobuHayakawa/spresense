/****************************************************************************
 * modules/cmdfw/bt_ble_wrapper.c
 * Spritzer Command Framework Bluetooth Wrapper
 *
 *   Copyright (C) 2017 Sony Corpration. All rights reserved.
 *   Author: Daisuke Sonoda <Daisuke.xA.Sonoda@Sony.com>
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
 * 3. Neither the name NuttX nor Sony nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
#include <sdk/config.h>
#include <stdio.h>
#include <unistd.h>
#include <debug.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <mqueue.h>
#include <semaphore.h>
#include "system/readline.h"

#include <bt/bt_comm.h>
#include <bt/bt_spp.h>
#include <ble/ble_comm.h>
#include <ble/ble_gap.h>
#include <ble/ble_gatts.h>
#include <ble/ble_gattc.h>
#include <memutils/simple_fifo/CMN_SimpleFifo.h>
#include "bt_ble_wrapper.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#ifdef CONFIG_DEBUG_COMMANDFW
#  define cmdfwdbg      dbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define cmdfwvdbg     vdbg
#  else
#    define cmdfwvdbg(x...)
#  endif
#else
#  define cmdfwdbg(x...)
#  define cmdfwvdbg(x...)
#endif

#define SEND_SYNC_WAIT_TS         (5)
#define BUF_LEN_MAX               32
#define CONN_HANDLE_INVALED       0xffff
#define BLE_UUID_SDS_SERVICE_IN   0xa000
#define BLE_UUID_SDS_CHAR_IN      0xa002
#define BLE_UUID_SDS_SERVICE_OUT  0xa001
#define BLE_UUID_SDS_CHAR_OUT     0xa003

#define CHAR_BUF_MAX MAX_RCV_DATA_LENGTH
#define CONN_HANDLE_INVALED       0xffff

#define BT_UART_CHNAGE_BAUD_RATE  115200

/****************************************************************************
 * Private Data
 ****************************************************************************/
typedef enum
{
  MsgDataSend = 0,
  MsgSendDone,
  MsgExit,
} MsgType;

typedef struct
{
  MsgType  type;
  uint8_t* data;
  int      len;
  int      xfrd;
} MessageEvent;

typedef enum {
  IDLE = 0,
  INIT,
  ADVERTISING,
  CONNECTED,
  UNINITING
} gap_state_t;

/* bt/ble target device address */

static BT_ADDR g_dest_addr = {{0}};

static uint8_t g_manufacturer_adv_data[] = {
  0xf3,
  0x10, 0x02,
  0x12,
  0xe4, 0x62, 0x6b, 0xb7, 0x0c, 0xe4 /* same with g_addr[] */
};

static BLE_Uuid128 service_uuid = {{0x2d, 0x0a, 0xde, 0xec,
                                       0xe3, 0x20, 0x43, 0xa0,
                                       0x12, 0x49, 0x2c, 0x76,
                                       0x00, 0x00, 0xBF, 0xCF}};
/* bt/ble wrapper deamon process id */

static pid_t g_daemon_pid = -1;
static mqd_t g_mq_dsc;

/* bt/ble send data pointer */

uint8_t* g_data;

RxCallBack g_callback = NULL;

typedef struct ble_srv_sds
{
  bool                 notify_enabled;
  BLE_GapConnHandle    conn_handle;
  uint16_t             srv_handle;
  uint8_t              char_rw_buf[CHAR_BUF_MAX]; /* read / write */
  BLE_GattsCharHandles char_rw_handle;
  uint8_t              char_ri_buf[CHAR_BUF_MAX]; /* read / indication */
  BLE_GattsCharHandles char_ri_handle;
} BLE_SrvSds;

/* ble event context must golobal scope */

BLE_EvtCtx g_ble_evt_ctx;
BLE_SrvSds g_srv_sds;
BLE_GapPairingFeature g_pairing_feature;
static BT_EVT g_bt_evt;

bool is_ble_connection = false;
bool is_ble_adv_continue = false;

static gap_state_t gap_state = IDLE;

static sem_t g_send_sync_sem;
static sem_t g_txcomp_sync_sem;
static sem_t state_lock_sem;
static sem_t disconn_sync_sem;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void bt_event_handler(BT_EVT *btEvt);
static void bt_device_event_handler(BT_EVT *pEvt);
static void bt_spp_event_handler(BT_EVT* pEvt);

static void bt_numericcomparison(BT_EVT_CONFIRMATION_REQ *pData);
static void bt_paircomplete(BT_PAIR_RESULT *pResult);
static void bt_encchanged(BT_EVT_ENC_STATE *pState);
static void bt_bondinfo(BT_BondInfo *pInfo);

static void bt_spp_callback(BT_SESSION_EVT *pEvt);
static void bt_spp_connected(BT_EVT_SPP_CONNECTED *spp);
static void bt_spp_disconnected(uint8_t *pData);
static void bt_spp_rxdata(BT_SPP_DATA *ptr);
static void bt_spp_txcomplete(void);

int bt_init(char* name, BT_ADDR addr, BT_BLE_UUID uuid);

static int ble_event_handler(void *user_data);
static void ble_gap_connected(const BLE_EvtConnected *evt);
static void ble_gap_disconnected(const BLE_EvtDisconnected *evt);
static void ble_gap_disp_passkey(BLE_EvtDisplayPasskey *disp_passkey);
static void ble_gap_exchange_feature(const BLE_EvtExchangeFeature *exchange_feature);
static void ble_gap_auth_status(BLE_EvtAuthStatus *auth_status);
static int32_t ble_gatts_register_service(BLE_SrvSds* srv_sds,
                            BLE_GattsUuidType srv_uuid_type, BLE_GattsUuidType char_uuid_type,
                            BLE_Uuid128 srv_uuid128, BLE_Uuid128 char_uuid128,
                            uint16_t srv_uuid_alias, uint16_t char_uuid_alias,
                            uint8_t read, uint8_t write, uint8_t notify,
                            uint8_t* char_buf, BLE_GattsCharHandles* handle);
static void ble_gap_timeout(BLE_EvtTimeout *evtTimeout);
static void ble_gatts_evt_write(BLE_EvtGattsWrite* gatts_write);
static void ble_gatts_evt_confirm(BLE_EvtGattsIndConfirm* gatts_confirm);
static void ble_gap_set_state(gap_state_t state);

int ble_init(char* name, BT_ADDR addr, BT_BLE_UUID uuid);

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void bt_event_handler(BT_EVT *btEvt)
{
  switch (btEvt->group) {
  case BT_CONTROL_GROUP_DEVICE:
    bt_device_event_handler(btEvt);
    break;
  case BT_CONTROL_GROUP_SPP:
    bt_spp_event_handler(btEvt);
    break;
  default:
    break;
  }
}

static void bt_device_event_handler(BT_EVT *pEvt)
{
  switch (pEvt->opcode) {
  case BT_EVT_DEV_BOND_INFO:
    bt_bondinfo((BT_BondInfo*)pEvt->evtData);
    break;
  case BT_EVT_DEV_INQUIRY_RESULT:
    break;
  case BT_EVT_DEV_INQUIRY_COMPLETE:
    break;
  case BT_EVT_DEV_PAIRING_COMPLETE:
    bt_paircomplete((BT_PAIR_RESULT *)pEvt->evtData);
    break;
  case BT_EVT_DEV_ENCRYPTION_CHANGED:
    bt_encchanged((BT_EVT_ENC_STATE *)pEvt->evtData);
    break;
  case BT_EVT_DEV_CONFIRMATION_REQUEST:
    bt_numericcomparison((BT_EVT_CONFIRMATION_REQ *)pEvt->evtData);
    break;
  default:
    break;
  }
}

static void bt_numericcomparison(BT_EVT_CONFIRMATION_REQ *pData)
{
  BT_REPLY_CONFIRM btReplyConfirm;
  uint8_t* ptr = NULL;

  memcpy(&btReplyConfirm.addr, &pData->addr, BT_ADDR_LEN);
  btReplyConfirm.btAccept = BT_TRUE;
  BT_ReplyConfirmation(btReplyConfirm);
  ptr = (uint8_t*)pData;
  (void)ptr;
  cmdfwdbg("BT address %02x:%02x:%02x:%02x:%02x:%02x\n", ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5]);
  cmdfwdbg("Numeric Comparison code %d\n", pData->numeric);
}

static void bt_paircomplete(BT_PAIR_RESULT *pResult)
{
  uint8_t *ptr = pResult->btAddr.address;
  (void)ptr;
  cmdfwdbg("Pairing complete, status = %d.\n", pResult->btPairStatus);
  cmdfwdbg("Pairing complete, addr %02x:%02x:%02x:%02x:%02x:%02x.\n",
      ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5]);
}

static void bt_encchanged(BT_EVT_ENC_STATE *pState)
{
  uint8_t *ptr = pState->addr.address;
  (void)ptr;
  cmdfwdbg("Enc Changed, status = %d.\n", pState->status);
  cmdfwdbg("Enc Changed, addr %02x:%02x:%02x:%02x:%02x:%02x.\n",
      ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5]);
}

static void bt_bondinfo(BT_BondInfo *pInfo)
{
  int ret = 0;
  uint8_t *ptr = pInfo->btAddr.address;
  (void)ptr;
  cmdfwdbg("Receive bond info, addr %02x:%02x:%02x:%02x:%02x:%02x.\n",
        ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5]);
  ret = BT_SaveBondInfo(pInfo);
  if (ret != BT_SUCCESS)
    {
      cmdfwdbg("[BT][ERROR]BT_SaveBondInfo failed, ret=%d\n", ret);
    }
  else
    {
      cmdfwdbg("BT_SaveBondInfo.\n");
    }
}

static void bt_spp_event_handler(BT_EVT* pEvt)
{
  switch (pEvt->opcode) {
  case BT_EVT_SPP_CONNECT:
    bt_spp_connected((BT_EVT_SPP_CONNECTED *)pEvt->evtData);
    break;
  case BT_EVT_SPP_DISCONNECT:
    bt_spp_disconnected((uint8_t *)pEvt->evtData);
    break;
  default:
    break;
  }
}

static void bt_spp_connected(BT_EVT_SPP_CONNECTED *spp)
{
  uint8_t pData[BT_ADDR_LEN] = {0};
  int ret = BT_SUCCESS;

  memcpy(g_dest_addr.address, spp->addr.address, BT_ADDR_LEN);
  ret = BT_SppSetRxCallBack(&spp->addr, bt_spp_callback);
  if (ret != BT_SUCCESS)
    {
      cmdfwdbg("BT_SppSetRxCallBack failed [%d].\n", ret);
    }
  else
    {
      is_ble_connection = false;
      memcpy(pData, spp->addr.address, BT_ADDR_LEN);
      cmdfwdbg("Spp connected address %02x:%02x:%02x:%02x:%02x:%02x.\n",
            pData[0], pData[1], pData[2],
            pData[3], pData[4], pData[5]);
    }
}

static void bt_spp_disconnected(uint8_t *pData)
{
  (void)pData;
  cmdfwdbg("Spp disconnected.\n");
}

static void bt_spp_callback(BT_SESSION_EVT *pEvt)
{
  BT_SESSION_EVT *btSessEvt = (BT_SESSION_EVT *)pEvt;

  switch (btSessEvt->opcode) {
  case BT_EVT_SPP_SERVICE_NOT_FOUND:
    cmdfwdbg("spp service not found.\n");
    break;
  case BT_EVT_SPP_CONNECTION_FAILED:
    cmdfwdbg("spp connection failed.\n");
    break;
  case BT_EVT_SPP_RX_DATA:
    cmdfwdbg("spp rx data.\n");
    bt_spp_rxdata((BT_SPP_DATA*)btSessEvt->evtData);
    break;
  case BT_EVT_SPP_TX_COMPLETE:
    cmdfwdbg("spp tx complete.\n");
    bt_spp_txcomplete();
    break;
  default:
    break;
  }
}

static void bt_spp_rxdata(BT_SPP_DATA *ptr)
{
  /* Receive spp data */

  if (g_callback != NULL)
    {
      g_callback(ptr->data, ptr->len);
    }
}

static void bt_spp_txcomplete(void)
{
  /* Queued sending spp data */

  sem_post(&g_txcomp_sync_sem);
}


int bt_init(char* name, BT_ADDR addr, BT_BLE_UUID uuid)
{
  int ret = BT_SUCCESS;
  BT_FIRMWARE_INFO info = {0};
  BT_SPP_UUID spp_uuid;

  memcpy(spp_uuid.uuid128, uuid.uuid128, BT_BLE_UUID128_LEN);

  ret = BT_SppSetUuid(&spp_uuid);
  if (ret != BT_SUCCESS)
    {
      cmdfwdbg("BT_SppSetUuid failed, ret=%d\n", ret);
      return ret;
    }

  cmdfwdbg("BT_CommonInitializeWithBtBinary\n");
  info.fileName = FIRMWARE_NAME_BT;
  ret = BT_CommonInitializeWithBtBinary(info);
  if (ret != BT_SUCCESS)
    {
      cmdfwdbg("BT_CommonInitializeWithBtBinary failed, ret=%d\n", ret);
      return ret;
    }

  /* if allready registered callback function exists,
   * BT_SetEvtCallBack function return EEXIST error.
   * if EEXIST error occurs, allowed and continue process.
   */
  cmdfwdbg("BT_SetEvtCallBack\n");
  ret = BT_SetEvtCallBack(bt_event_handler, &g_bt_evt);
  if (ret != BT_SUCCESS && ret != -EEXIST)
    {
      cmdfwdbg("BT_SetEvtCallBack failed, ret=%d\n", ret);
      return ret;
    }

  cmdfwdbg("BT_SetBtAddress\n");
  ret = BT_SetBtAddress(&addr);
  if (ret != BT_SUCCESS)
    {
      cmdfwdbg("BT_SetBtAddress failed, ret=%d\n", ret);
      return ret;
    }

  ret = BT_SetBtName(name);
  if (ret != BT_SUCCESS)
    {
      cmdfwdbg("BT_SetBtName failed, ret=%d\n", ret);
      return ret;
    }

  return ret;
}

/*--------------------------------------------------------------------*/

static int ble_event_handler(void *user_data)
{
  BLE_EvtCtx *evt_ctx = (BLE_EvtCtx *)user_data;
  BLE_Evt *evt        = &evt_ctx->evt;

  switch (evt->evtHeader)
    {
      case BLE_EVENT_TX_COMPLETE:
        cmdfwdbg("BLE_EVENT_TX_COMPLETE, count=%u\n", ((BLE_EvtTxComplete *) evt->evtData)->count);
        break;
      case BLE_GAP_EVENT_CONNECTED:
        ble_gap_connected((BLE_EvtConnected *)evt->evtData);
        break;
      case BLE_GAP_EVENT_DISCONNECTED:
        ble_gap_disconnected((BLE_EvtDisconnected *)evt->evtData);
        break;
      case BLE_GAP_EVENT_DISPLAY_PASSKEY:
        ble_gap_disp_passkey((BLE_EvtDisplayPasskey *)evt->evtData);
        break;
      case BLE_GAP_EVENT_AUTH_STATUS:
        ble_gap_auth_status((BLE_EvtAuthStatus *)evt->evtData);
        break;
      case BLE_GAP_EVENT_AUTH_KEY_REQUEST:
        cmdfwdbg("BLE_GAP_EVENT_AUTH_KEY_REQUEST\n");
        break;
      case BLE_GAP_EVENT_CONN_SEC_UPDATE:
        cmdfwdbg("BLE_GAP_EVENT_CONN_SEC_UPDATE\n");
        break;
      case BLE_GAP_EVENT_EXCHANGE_FEATURE:
        ble_gap_exchange_feature((BLE_EvtExchangeFeature *)evt->evtData);
        break;
      case BLE_GAP_EVENT_TIMEOUT:
        ble_gap_timeout((BLE_EvtTimeout *)evt->evtData);
        break;
      case BLE_GATTS_EVENT_WRITE:
        ble_gatts_evt_write((BLE_EvtGattsWrite*)evt->evtData);
        break;
      case BLE_GATTS_EVENT_CFM:
        ble_gatts_evt_confirm((BLE_EvtGattsIndConfirm *)evt->evtData);
        break;
      default:
        cmdfwdbg("event=%d\n", evt->evtHeader);
        break;
    }

  return 0;
}

static void ble_gap_connected(const BLE_EvtConnected *evt)
{
  g_srv_sds.conn_handle = evt->handle;
  is_ble_connection = true;
  ble_gap_set_state(CONNECTED);

  cmdfwdbg("BLE_GAP_EVENT_CONNECTED\n");
}

static void ble_gap_disconnected(const BLE_EvtDisconnected *evt)
{
  int ret = BLE_SUCCESS;

  g_srv_sds.conn_handle = CONN_HANDLE_INVALED;
  is_ble_connection = false;

  if (gap_state == UNINITING)
    {
      sem_post(&disconn_sync_sem);
      return;
    }

  ble_gap_set_state(INIT);

  ret = BLE_GapStartAdv();
  if (ret != BLE_SUCCESS)
    {
      cmdfwdbg("BLE_GapStartAdv failed, ret=%d\n", ret);
    }
  ble_gap_set_state(ADVERTISING);

  cmdfwdbg("BLE_GAP_EVENT_DISCONNECTED reason:%u\n", evt->reason);
}

static void ble_gap_disp_passkey(BLE_EvtDisplayPasskey *disp_passkey)
{
  cmdfwdbg("Passkey: %s\n", disp_passkey->passkey);
}

static void ble_gap_exchange_feature(const BLE_EvtExchangeFeature *exchange_feature)
{
  BLE_GapPairingFeature* pf = &g_pairing_feature;
  int ret = BLE_GapExchangePairingFeature(exchange_feature->handle, pf);

  if (ret != BLE_SUCCESS)
    {
      cmdfwdbg("exchange pairing feature failed, ret=%d\n", ret);
    }
}

static void ble_gap_auth_status(BLE_EvtAuthStatus *auth_status)
{
  cmdfwdbg("BLE_GAP_EVENT_AUTH_STATUS, status:%02x\n", auth_status->status);

  if (auth_status->status == BLE_GAP_SM_STATUS_SUCCESS)
    {
      cmdfwdbg("BLE bonding SUCCESS, conn-handle:%u, addr-type:%u, addr:%02x:%02x:%02x:%02x:%02x:%02x\n",
               auth_status->handle, auth_status->bondInfo.addrType,
               auth_status->bondInfo.addr[0], auth_status->bondInfo.addr[1],
               auth_status->bondInfo.addr[2], auth_status->bondInfo.addr[3],
               auth_status->bondInfo.addr[4], auth_status->bondInfo.addr[5]);

      int ret = BLE_GapSaveBondInfo(&auth_status->bondInfo);
      if (ret != BLE_SUCCESS)
        {
          cmdfwdbg("BLE_GapSaveBondInfo error:%d\n", ret);
        }
    }
}

static int32_t ble_gatts_register_service(BLE_SrvSds* srv_sds,
                            BLE_GattsUuidType srv_uuid_type, BLE_GattsUuidType char_uuid_type,
                            BLE_Uuid128 srv_uuid128, BLE_Uuid128 char_uuid128,
                            uint16_t srv_uuid_alias, uint16_t char_uuid_alias,
                            uint8_t read, uint8_t write, uint8_t notify,
                            uint8_t* char_buf, BLE_GattsCharHandles* handle)
{
  int32_t       ret = 0;
  BLE_Uuid      serv_uuid;
  BLE_Uuid      char_uuid;
  BLE_CharMeta  char_meta;
  BLE_GattsAttr char_value;

  memset(&serv_uuid,  0, sizeof(BLE_Uuid));
  memset(&char_uuid,  0, sizeof(BLE_Uuid));
  memset(&char_meta,  0, sizeof(BLE_CharMeta));
  memset(&char_value, 0, sizeof(BLE_GattsAttr));

  serv_uuid.type = srv_uuid_type;
  memcpy(&serv_uuid.value.baseAlias.uuidBase, &srv_uuid128, sizeof(BLE_Uuid128));
  serv_uuid.value.baseAlias.uuidAlias = srv_uuid_alias;

  char_uuid.type = char_uuid_type;
  memcpy(&char_uuid.value.baseAlias.uuidBase, &char_uuid128, sizeof(BLE_Uuid128));
  char_uuid.value.baseAlias.uuidAlias = char_uuid_alias;

  ret = BLE_GattsAddService(BLE_GATTS_SRVTYP_PRIMARY,
                            &serv_uuid,
                            &srv_sds->srv_handle);

  if(ret != BLE_SUCCESS)
    {
      cmdfwdbg("init service failed, ret=%d\n", ret);
      return ret;
    }

  char_meta.charPrope.read   = read;
  char_meta.charPrope.write  = write;
  char_meta.charPrope.notify = notify;

  memcpy(&char_value.valueUuid, &char_uuid, sizeof(BLE_Uuid));
  char_value.attrValue          = char_buf;
  char_value.valueLen           = CHAR_BUF_MAX;
  char_value.attrPerm.readPerm  = BLE_SEC_MODE1LV1_NO_SEC;
  char_value.attrPerm.writePerm = BLE_SEC_MODE1LV1_NO_SEC;

  ret = BLE_GattsAddCharacteristic(srv_sds->srv_handle,
                                   &char_meta,
                                   &char_value,
                                   handle);

  if(ret != BLE_SUCCESS)
    {
      cmdfwdbg("init service characteristic failed, ret=%d\n", ret);
      return ret;
    }

  return ret;
}

static void ble_gap_timeout(BLE_EvtTimeout *evtTimeout)
{
  int ret = BLE_SUCCESS;

  switch (evtTimeout->timeoutSrc)
    {
      case BLE_GAP_TIMEOUT_ADVERTISING:
        cmdfwdbg("BLE_GAP_TIMEOUT_ADVERTISING\n");
        ble_gap_set_state(INIT);
        if (is_ble_adv_continue)
          {
            ret = BLE_GapStartAdv();
            if (ret != BLE_SUCCESS)
              {
                cmdfwdbg("BLE_GapStartAdv failed, ret=%d\n", ret);
              }
            ble_gap_set_state(ADVERTISING);
          }
        break;
      case BLE_GAP_TIMEOUT_CONN:
        cmdfwdbg("BLE_GAP_TIMEOUT_CONN\n");
        break;
      case BLE_GAP_TIMEOUT_SCAN:
        cmdfwdbg("BLE_GAP_TIMEOUT_SCAN\n");
        break;
      case BLE_GAP_TIMEOUT_SECURITY_REQUEST:
        cmdfwdbg("BLE_GAP_TIMEOUT_SECURITY_REQUEST\n");
        break;
      default:
        cmdfwdbg("BLE_GAP_TIMEOUT unknown source:%d\n",
                evtTimeout->timeoutSrc);
    }
}

static void ble_gatts_evt_write(BLE_EvtGattsWrite* gatts_write)
{
  BLE_SrvSds* srv_sds = &g_srv_sds;

  if (gatts_write->handle == srv_sds->char_rw_handle.dprHandle.cccdHandle)
    {
      /* client characteristic configuration.bit 0: Notification bit 1: Indication.other reserved.
         see core spec 4.1 Vol3,PartG,3,3,3,3 client characteristic
         configuration.Table 3.11
      */
      srv_sds->notify_enabled = (bool)gatts_write->data[0] & 0x01;
      if (srv_sds->notify_enabled)
        {
          cmdfwdbg("notification enabled!\n");
        }
      else
        {
          cmdfwdbg("notification disabled!\n");
        }
    }
  else
    {
      /* receive spp data */

      if (g_callback != NULL)
        {
          g_callback(gatts_write->data, gatts_write->dataLen);
        }
    }
}

static void ble_gatts_evt_confirm(BLE_EvtGattsIndConfirm* gatts_confirm)
{
  cmdfwdbg("gatts confirm event handle:%u\n", gatts_confirm->handle);
}

static void ble_gap_set_state(gap_state_t state)
{
  sem_wait(&state_lock_sem);
  gap_state = state;
  sem_post(&state_lock_sem);

  cmdfwdbg("state change %d\n", state);
}

static int32_t ble_init_pairing_mode(BLE_GapIoCap io_cap)
{
  BLE_GapPairingFeature* pf = &g_pairing_feature;
  int                    ret = 0;
  BLE_GapOOB             oob = BLE_GAP_OOB_AUTH_DATA_NOT_PRESENT;
  BLE_GapAuth            auth = BLE_GAP_AUTH_REQ_NO_MITM_BOND;

  pf->oob        = oob;
  pf->ioCap      = io_cap;
  pf->authReq    = auth;
  pf->minKeySize = BLE_GAP_MIN_KEY_SIZE;
  pf->maxKeySize = BLE_GAP_MAX_KEY_SIZE;

  return ret;
}

int ble_init(char* name, BT_ADDR addr, BT_BLE_UUID uuid)
{
  int ret = BLE_SUCCESS;
  BLE_InitializeParams params = {0};
  BLE_GapDeviceConfig ble_dev_cfg;
  BLE_GapAddr ble_addr;
  BLE_GapName ble_name;
  BLE_GapAdvData adv_data = {0};

  params.role = BLE_ROLE_PERIPHERAL;
  ret = BLE_CommonInitializeStack(&params);
  if (ret != BLE_SUCCESS)
    {
      cmdfwdbg("BLE_CommonInitializeStack failed, ret=%d\n", ret);
      return ret;
    }

  /* if allready registered callback function exists,
   * BLE_CommonSetBleEvtCallback function return EEXIST error.
   * if EEXIST error occurs, allowed and continue process.
   */
  ret = BLE_CommonSetBleEvtCallback((BLE_EfCb)ble_event_handler, &g_ble_evt_ctx);
  if (ret != BLE_SUCCESS && ret != -EEXIST)
    {
      cmdfwdbg("BLE_CommonSetBleEvtCallback failed, ret=%d\n", ret);
      return ret;
    }

  ble_addr.mode = BLE_GAP_ADDR_MODE_MANUAL;
  memcpy(ble_addr.addr, addr.address, sizeof(addr.address));
  ble_dev_cfg.type = BLE_GAP_DEVICE_CONFIG_ADDR;
  ble_dev_cfg.data = (void*)&ble_addr;
  ret = BLE_GapSetDeviceConfig(&ble_dev_cfg);
  if (ret != BLE_SUCCESS)
    {
      cmdfwdbg("BLE_GapSetDeviceConfig failed, ret=%d\n", ret);
      return ret;
    }

  ble_name.name    = (uint8_t*)name;
  ble_name.size    = strnlen((char*)ble_name.name, BUF_LEN_MAX);
  ble_dev_cfg.type = BLE_GAP_DEVICE_CONFIG_NAME;
  ble_dev_cfg.data = (void*)&ble_name;
  ret = BLE_GapSetDeviceConfig(&ble_dev_cfg);
  if (ret != BLE_SUCCESS)
    {
      cmdfwdbg("BLE_GapSetDeviceConfig failed, ret=%d\n", ret);
      return ret;
    }

  adv_data.flags = BLE_GAP_ADV_LE_GENERAL_DISC_MODE | BLE_GAP_ADV_BR_EDR_NOT_SUPPORTED;
  adv_data.complete32Uuid                     = 0;
  adv_data.completeLocalName.advData          = (uint8_t*)name;
  adv_data.completeLocalName.advLength        = strnlen((char*)adv_data.completeLocalName.advData, BUF_LEN_MAX);
  adv_data.manufacturerSpecificData.advData   = g_manufacturer_adv_data;
  adv_data.manufacturerSpecificData.advLength = sizeof(g_manufacturer_adv_data)/sizeof(uint8_t);
  ret = BLE_GapSetAdvData(&adv_data);
  if (ret != BLE_SUCCESS)
    {
      cmdfwdbg("BLE_GapSetAdvData failed, ret=%d\n", ret);
      return ret;
    }

  ret = ble_gatts_register_service(&g_srv_sds,
                      BLE_UUID_TYPE_BASEALIAS_VENDOR, BLE_UUID_TYPE_BASEALIAS_VENDOR,
                      service_uuid, service_uuid,
                      BLE_UUID_SDS_SERVICE_IN, BLE_UUID_SDS_CHAR_IN,
                      1, 1, 1,
                      g_srv_sds.char_rw_buf, &g_srv_sds.char_rw_handle);
  if (ret != BLE_SUCCESS)
    {
      return ret;
    }
  cmdfwdbg("connection handle for rcv attr %x\n", g_srv_sds.char_rw_handle.charHandle);

  ret = ble_gatts_register_service(&g_srv_sds,
                      BLE_UUID_TYPE_BASEALIAS_VENDOR, BLE_UUID_TYPE_BASEALIAS_VENDOR,
                      service_uuid, service_uuid,
                      BLE_UUID_SDS_SERVICE_OUT, BLE_UUID_SDS_CHAR_OUT,
                      1, 1, 1,
                      g_srv_sds.char_ri_buf, &g_srv_sds.char_ri_handle);
  if (ret != BLE_SUCCESS)
    {
      return ret;
    }
  cmdfwdbg("connection handle for ntfy attr %x\n", g_srv_sds.char_ri_handle.charHandle);

  ble_init_pairing_mode(BLE_GAP_IO_CAP_NO_INPUT_NO_OUTPUT);

  sem_init(&disconn_sync_sem, 0, 0);
  sem_init(&state_lock_sem, 0, 1);
  ble_gap_set_state(INIT);

  return ret;
}

static int bt_ble_sender_task(void)
{
  MessageEvent msg;
  size_t div_size;
  int mq_ret;
  int ret;
  BLE_SrvSds* srv_sds = &g_srv_sds;
  BLE_GattsHandleValueNfyIndParams param = {0};

  while(true)
    {
      /* receive spp send data event from main task */

      mq_ret = mq_receive(g_mq_dsc, (char*)&msg, sizeof(msg), NULL);
      if (mq_ret != ERROR)
        {
          if (msg.type == MsgExit)
            {
              cmdfwdbg("exit event!\n");
              break;
            }
          else if (msg.type == MsgDataSend)
            {
              if (is_ble_connection)
                {
                  /* adjust to send buffer size */

                  div_size = msg.len - msg.xfrd;
                  if (div_size > CHAR_BUF_MAX)
                    {
                      div_size = CHAR_BUF_MAX;
                    }

                  cmdfwdbg("stored size=%d, xfrd=%d, divide=%d\n", msg.len, msg.xfrd, div_size);

                  /* send data by ble gatts service */
                  if (!srv_sds->notify_enabled)
                    {
                      cmdfwdbg("notification disabled!\n");
                    }

                  param.type        = BLE_GATT_NOTIFICATION;
                  param.attrHandle  = srv_sds->char_ri_handle.charHandle;
                  param.attrValData = (uint8_t*)(msg.data + msg.xfrd);
                  param.attrValLen  = div_size;
                  ret = BLE_GattsHandleValueNfyInd(srv_sds->conn_handle, &param);
                  if (ret != BLE_SUCCESS)
                    {
                      cmdfwdbg("gatts handle value nfy/ind failed, ret=%d\n", ret);
                    }
                  else
                    {
                      /* send message to self and check remaining bytes in fifo */

                      msg.xfrd += div_size;
                      if (msg.xfrd >= msg.len)
                        {
                          msg.type = MsgSendDone;
                        }
                      mq_send(g_mq_dsc, (const char*)&msg, sizeof(msg), 0);
                    }
                }
              else
                {
                  /* adjust to send buffer size */

                  div_size = msg.len - msg.xfrd;
                  if (div_size > CONFIG_UART2_TXBUFSIZE)
                    {
                      div_size = CONFIG_UART2_TXBUFSIZE;
                    }

                  cmdfwdbg("stored size=%d, xfrd=%d, divide=%d\n", msg.len, msg.xfrd, div_size);

                  /* send data by spp profile */

                  ret = BT_SppSendData(&g_dest_addr, (msg.data + msg.xfrd), div_size);
                  if (ret != BT_SUCCESS)
                    {
                      cmdfwdbg("BT_SppSendData error, ret:%d\n", ret);
                    }
                  else
                    {
                      sem_wait(&g_txcomp_sync_sem);

                      /* send message to self and check remaining bytes in fifo */

                      msg.xfrd += div_size;
                      if (msg.xfrd >= msg.len)
                        {
                          msg.type = MsgSendDone;
                        }
                      mq_send(g_mq_dsc, (const char*)&msg, sizeof(msg), 0);
                    }
                }
            }
          else if (msg.type == MsgSendDone)
            {
              cmdfwdbg("send data complete\n");
              sem_post(&g_send_sync_sem);
            }
        }
    }

  cmdfwdbg("exited.\n");

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: BTBLE_Initialize
 *
 * Description:
 *   BT/BLE wrapper layer initilize.
 *   do bluetooth and ble stack initialize and simple fifo initialize.
 *
 ****************************************************************************/
int BTBLE_Initialize(char* name, BT_ADDR addr, BT_BLE_UUID uuid, RxCallBack callback)
{
  int ret;

  /* BT initalize */

  ret = bt_init(name, addr, uuid);
  if (ret != BT_SUCCESS)
    {
      cmdfwdbg("bt stack initialize failed, ret=%d\n", ret);
      return -EINVAL;
    }

  /* BLE initalize */

  ret = ble_init(name, addr, uuid);
  if (ret != BLE_SUCCESS)
    {
      cmdfwdbg("ble stack initialize failed, ret=%d\n", ret);
      return -EINVAL;
    }

  g_callback = callback;

  return OK;
}

/****************************************************************************
 * Name: BTBLE_Finalize
 *
 * Description:
 *   Start spritzer command framework service.
 *
 ****************************************************************************/
void BTBLE_Finalize(void)
{
  int ret = BLE_SUCCESS;

  switch (gap_state)
    {
      case CONNECTED:
        cmdfwdbg("current state: CONNECTED\n");
        ble_gap_set_state(UNINITING);
        BLE_GapDisconnectLink(g_srv_sds.conn_handle);
        sem_wait(&disconn_sync_sem);
        break;
      case ADVERTISING:
        cmdfwdbg("current state: ADVERTISING\n");
        is_ble_adv_continue = false;
        ret = BLE_GapStopAdv();
        if (ret != BLE_SUCCESS)
          {
            cmdfwdbg("BLE_GapStopAdv failed, ret=%d\n", ret);
          }
        break;
      case INIT:
        cmdfwdbg("current state: INIT\n");
        break;
      case UNINITING:
        cmdfwdbg("BLE is uninitializing now\n");
        break;
      case IDLE:
        cmdfwdbg("BLE already is uninitialized\n");
        break;
      default:
        cmdfwdbg("tsutil_ble_gap_uninit, unknown state: %d\n", gap_state);
        ASSERT(0);
    }

  sem_destroy(&disconn_sync_sem);
  sem_destroy(&state_lock_sem);

  g_callback = NULL;

  /* bt stack finalize */

  BT_CommonFinalize();

  /* ble stack finalize */

  BLE_CommonFinalizeStack();

  ret = BT_SetBaudrate(BT_UART_CHNAGE_BAUD_RATE);
  if (ret != BT_SUCCESS) {
    cmdfwdbg("BT_SetBaudrate failed, ret=%d\n", ret);
  }
}

/****************************************************************************
 * Name: SPCMD_StartService
 *
 * Description:
 *   Stop spritzer command framework service.
 *
 ****************************************************************************/
int BTBLE_StartService(void)
{
  int ret = BT_SUCCESS;
  struct mq_attr que_attr;

  sem_init(&g_send_sync_sem, 0, 0);
  sem_init(&g_txcomp_sync_sem, 0, 0);

  /* que and task for process command event */

  que_attr.mq_maxmsg  = 10;
  que_attr.mq_msgsize = sizeof(MessageEvent);
  que_attr.mq_flags   = 0;

  g_mq_dsc = mq_open("bt_ble_event_que", O_RDWR | O_CREAT, 0666, &que_attr);

  g_daemon_pid = task_create("bt_ble_sender_task",  150, (1024 * 4), (main_t)bt_ble_sender_task, NULL);
  if (g_daemon_pid < 0)
    {
      cmdfwdbg("start bt_ble_sender_task failed\n");
      return -EINVAL;
    }

  ret = BT_SetPairingEnable(BT_TRUE);
  if (ret != BT_SUCCESS)
    {
      cmdfwdbg("BT_SetPairingEnable failed, ret=%d\n", ret);
      return ret;
    }

  ret = BT_SetVisibility(BT_VIS_DISCOVERY_CONNECTABLE);
  if (ret != BT_SUCCESS)
    {
      cmdfwdbg("BT_SetVisibility failed, ret=%d\n", ret);
      return ret;
    }

  ret = BT_StartInquiry();
  if (ret != BT_SUCCESS)
    {
      cmdfwdbg("BT_StartInquiry failed, ret=%d\n", ret);
    }

  is_ble_adv_continue = true;
  ret = BLE_GapStartAdv();
  if (ret != BLE_SUCCESS)
    {
      cmdfwdbg("BLE_GapStartAdv failed, ret=%d\n", ret);
    }
  ble_gap_set_state(ADVERTISING);

  return ret;
}

/****************************************************************************
 * Name: SPCMD_StopService
 *
 * Description:
 *   Stop spritzer command framework service.
 *
 ****************************************************************************/
int BTBLE_StopService(void)
{
  int ret = BT_SUCCESS;
  MessageEvent msg = {0};

  /* send to exit message to bt_ble_sender_task */

  msg.type = MsgExit;
  mq_send(g_mq_dsc, (const char*)&msg, sizeof(msg), 0);

  sem_destroy(&g_send_sync_sem);
  sem_destroy(&g_txcomp_sync_sem);

  g_daemon_pid = -1;

  return ret;
}

/****************************************************************************
 * Name: SPCMD_SendData
 *
 * Description:
 *   Send spritzer command framework data.
 *
 ****************************************************************************/
int BTBLE_SendData(uint8_t* data, uint16_t len)
{
  struct timespec ts = {0};
  MessageEvent msg = {0};
  int ret;

  /* data check */

  if ((data == NULL) || (len == 0))
    {
      cmdfwdbg("invalid data: addr %08x size %d\n", data, len);
      return -EINVAL;
    }

  msg.type = MsgDataSend;
  msg.data = data;
  msg.len  = len;
  msg.xfrd = 0;
  mq_send(g_mq_dsc, (const char*)&msg, sizeof(msg), 0);

  if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
    {
      cmdfwdbg("clock_gettime error\n");
      return -EINVAL;
    }
  ts.tv_sec += SEND_SYNC_WAIT_TS;

  /* wait data send complete */

  ret = sem_timedwait(&g_send_sync_sem, &ts);
  if (ret != OK)
    {
      if (errno == ETIMEDOUT)
        {
          cmdfwdbg("timeout error\n");
          return -EBUSY;
        }
      else
        {
          cmdfwdbg("error\n");
          return -EIO;
        }
    }

  return 0;
}
