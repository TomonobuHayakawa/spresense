/*
 * Copyright (C) 2012 The Android Open Source Project
 * Copyright (C) 2013 Sony Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <semaphore.h>
#include <errno.h>
#include "OverrideLog.h"
#include "NfcJniUtil.h"
#include "NfcAdaptation.h"
#include "SyncEvent.h"
//#include "PeerToPeer.h"
//#include "RoutingManager.h"
#include "NfcTag.h"
#include "config.h"
#include "PowerSwitch.h"

extern "C"
{
    #include "nfa_ce_api.h"
    #include "ce_api.h"
    #include "nfa_halcc_api.h"
    #include <signal.h> /* kill */
    #include <stdlib.h> /* abort */
#ifdef SPZ1_IMPL
#include "syscall_wrapper.h"
#include "pthread_wrapper.h"
#endif /* SPZ1_IMPL */
}

extern UINT8 *p_nfa_dm_start_up_cfg;
extern const UINT8 nfca_version_string [];
extern const UINT8 nfa_version_string [];
extern tNFA_DM_DISC_FREQ_CFG* p_nfa_dm_rf_disc_freq_cfg; //defined in stack
extern bool gIsTagDeactivating;
extern bool gIsSelectingRfInterface;
extern void nativeNfcTag_notifyRfTimeout ();
extern void nativeNfcTag_doConnectStatus (bool is_connect_ok);
extern void nativeNfcTag_doDeactivateStatus (int status);
extern void nativeNfcTag_doCheckNdefResult (tNFA_STATUS status, uint32_t max_size, uint32_t current_size, uint8_t flags);
extern void nativeNfcTag_doPresenceCheckResult (tNFA_STATUS status);
extern void nativeNfcTag_formatStatus (bool is_ok);
extern void nativeNfcTag_resetPresenceCheck ();
extern void nativeNfcTag_doReadCompleted (tNFA_STATUS status);
extern void nativeNfcTag_abortWaits ();
extern void nativeNfcTag_registerNdefTypeHandler ();

/* notify for application */
extern sem_t g_nfa_activated;
extern int g_event;

#define D_EVENT_TAG_READ 0
#define D_EVENT_HCE      1
#define D_EVENT_ERROR    2 

#ifdef SPZ2_IMPL
#define SIGTERM SIGUSR1 /* TODO Not defined at nuttx's <signal.h>. Temporaly solution */
#endif
/*****************************************************************************
**
** public variables and functions
**
*****************************************************************************/
bool      gActivated = false;
SyncEvent gDeactivatedEvent;
int       sDiscovery_duration;


void   doStartupConfig ();
void   startStopPolling (bool isStartPolling);
void   startRfDiscovery (bool isStart);

static tNFA_STATUS stopPolling_rfDiscoveryDisabled();
static tNFA_STATUS startPolling_rfDiscoveryDisabled(tNFA_TECHNOLOGY_MASK tech_mask);

/*****************************************************************************
**
** private variables and functions
**
*****************************************************************************/
static SyncEvent    sNfaEnableEvent;  //event for NFA_Enable()
static SyncEvent    sNfaDisableEvent;  //event for NFA_Disable()
static SyncEvent    sNfaEnableDisablePollingEvent;  //event for NFA_EnablePolling(), NFA_DisablePolling()
static SyncEvent    sNfaSetConfigEvent;  // event for Set_Config....
static SyncEvent    sNfaGetConfigEvent;  // event for Get_Config....
static SyncEvent    sNfaCeRegFelicaSystemCodeOnDH;  // event for NFA_CE_REGISTERED_EVT
static SyncEvent    sNfaCeConfigureLocalTag;  // event for NFA_CE_LOCAL_TAG_CONFIGURED_EVT
static bool         sIsNfaEnabled = false;
static bool         sDiscoveryEnabled = false;
static bool         sPollingEnabled = false;  //is polling for tag?
static bool         sIsDisabling = false;
static bool         sRfEnabled = false; // whether RF discovery is enabled
static bool         sReaderModeEnabled = false; // whether we're only reading tags, not allowing P2p/card emu
static unsigned int sNfaCheckInternal=0; /* CXD224x workaround: GEN_ERROR */
static bool         sP2pActive = false; // whether p2p was last active
static bool         sAbortConnlessWait = false;
static bool         sIsSecElemSelected = false;  //has NFC service selected a sec elem
#define DEFAULT_TECH_MASK           (NFA_TECHNOLOGY_MASK_A \
                                     | NFA_TECHNOLOGY_MASK_B \
                                     | NFA_TECHNOLOGY_MASK_F )
#define DEFAULT_TECH_MASK_LISTEN    (NFA_TECHNOLOGY_MASK_A \
                                     | NFA_TECHNOLOGY_MASK_B \
                                     | NFA_TECHNOLOGY_MASK_F )
#define DEFAULT_DISCOVERY_DURATION       500
#define READER_MODE_DISCOVERY_DURATION   200

static void nfaConnectionCallback (UINT8 event, tNFA_CONN_EVT_DATA *eventData);
static void nfaDeviceManagementCallback (UINT8 event, tNFA_DM_CBACK_DATA *eventData);
static bool isPeerToPeer (tNFA_ACTIVATED& activated);

static UINT16 sCurrentConfigLen;
static UINT8 sConfig[256];

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

/*******************************************************************************
**
** Function:        handleRfDiscoveryEvent
**
** Description:     Handle RF-discovery events from the stack.
**                  discoveredDevice: Discovered device.
**
** Returns:         None
**
*******************************************************************************/
static void handleRfDiscoveryEvent (tNFC_RESULT_DEVT* discoveredDevice)
{
    if (discoveredDevice->more)
    {
        //there is more discovery notification coming
        return;
    }

    bool isP2p = NfcTag::getInstance ().isP2pDiscovered ();
    if (!sReaderModeEnabled && isP2p)
    {
        //select the peer that supports P2P
        NfcTag::getInstance ().selectP2p();
    }
    else
    {
        //select the first of multiple tags that is discovered
        NfcTag::getInstance ().selectFirstTag();
    }
}


/*******************************************************************************
**
** Function:        nfaConnectionCallback
**
** Description:     Receive connection-related events from stack.
**                  connEvent: Event code.
**                  eventData: Event data.
**
** Returns:         None
**
*******************************************************************************/
static void nfaConnectionCallback (UINT8 connEvent, tNFA_CONN_EVT_DATA* eventData)
{
    tNFA_STATUS status = NFA_STATUS_FAILED;
    ALOGD("%s: event= %u", __FUNCTION__, connEvent);

    switch (connEvent)
    {
    case NFA_POLL_ENABLED_EVT: // whether polling successfully started
        {
            ALOGD("%s: NFA_POLL_ENABLED_EVT: status = %u", __FUNCTION__, eventData->status);

            SyncEventGuard guard (sNfaEnableDisablePollingEvent);
            sNfaEnableDisablePollingEvent.notifyOne ();
        }
        break;

    case NFA_POLL_DISABLED_EVT: // Listening/Polling stopped
        {
            ALOGD("%s: NFA_POLL_DISABLED_EVT: status = %u", __FUNCTION__, eventData->status);

            SyncEventGuard guard (sNfaEnableDisablePollingEvent);
            sNfaEnableDisablePollingEvent.notifyOne ();
        }
        break;

    case NFA_RF_DISCOVERY_STARTED_EVT: // RF Discovery started
        {
            ALOGD("%s: NFA_RF_DISCOVERY_STARTED_EVT: status = %u", __FUNCTION__, eventData->status);

            SyncEventGuard guard (sNfaEnableDisablePollingEvent);
            sNfaEnableDisablePollingEvent.notifyOne ();
        }
        break;

    case NFA_RF_DISCOVERY_STOPPED_EVT: // RF Discovery stopped event
        {
            ALOGD("%s: NFA_RF_DISCOVERY_STOPPED_EVT: status = %u", __FUNCTION__, eventData->status);

            SyncEventGuard guard (sNfaEnableDisablePollingEvent);
            sNfaEnableDisablePollingEvent.notifyOne ();
        }
        break;

    case NFA_DISC_RESULT_EVT: // NFC link/protocol discovery notificaiton
        status = eventData->disc_result.status;
        ALOGD("%s: NFA_DISC_RESULT_EVT: status = %d", __FUNCTION__, status);
        if (status != NFA_STATUS_OK)
        {
            ALOGE("%s: NFA_DISC_RESULT_EVT error: status = %d", __FUNCTION__, status);
        }
        else
        {
            NfcTag::getInstance().connectionEventHandler(connEvent, eventData);
            handleRfDiscoveryEvent(&eventData->disc_result.discovery_ntf);
        }
        break;

    case NFA_SELECT_RESULT_EVT: // NFC link/protocol discovery select response
        ALOGD("%s: NFA_SELECT_RESULT_EVT: status = %d, gIsSelectingRfInterface = %d, sIsDisabling=%d", __FUNCTION__, eventData->status, gIsSelectingRfInterface, sIsDisabling);

        if (sIsDisabling)
            break;

#ifndef SPZ_IMPL
        if (eventData->status != NFA_STATUS_OK)
        {
            if (gIsSelectingRfInterface)
            {
                nativeNfcTag_doConnectStatus(false);
            }

            ALOGE("%s: NFA_SELECT_RESULT_EVT error: status = %d", __FUNCTION__, eventData->status);
            NFA_Deactivate (false);
        }
#else /* SPZ_IMPL */
            NFA_Deactivate (false);
#endif /* SPZ_IMPL */
        break;

    case NFA_DEACTIVATE_FAIL_EVT:
        ALOGD("%s: NFA_DEACTIVATE_FAIL_EVT: status = %d", __FUNCTION__, eventData->status);
        break;

    case NFA_ACTIVATED_EVT: // NFC link/protocol activated
        ALOGD("%s: NFA_ACTIVATED_EVT: gIsSelectingRfInterface=%d, sIsDisabling=%d", __FUNCTION__, gIsSelectingRfInterface, sIsDisabling);
        NfcTag::getInstance().setActive(true);
        if (sIsDisabling || !sIsNfaEnabled)
            break;
        gActivated = true;

        NfcTag::getInstance().setActivationState ();
#if 0 
        if (gIsSelectingRfInterface)
        {
            nativeNfcTag_doConnectStatus(true);
            break;
        }
#endif

        nativeNfcTag_resetPresenceCheck();
        if (isPeerToPeer(eventData->activated))
        {
            if (sReaderModeEnabled)
            {
                ALOGD("%s: ignoring peer target in reader mode.", __FUNCTION__);
                NFA_Deactivate (false);
                break;
            }
            sP2pActive = true;
            ALOGD("%s: NFA_ACTIVATED_EVT; is p2p", __FUNCTION__);
            // Disable RF field events in case of p2p
            UINT8  nfa_disable_rf_events[] = { 0x00 };
            ALOGD ("%s: Disabling RF field events", __FUNCTION__);
            status = NFA_SetConfig(NCI_PARAM_ID_RF_FIELD_INFO, sizeof(nfa_disable_rf_events),
                    &nfa_disable_rf_events[0]);
            if (status == NFA_STATUS_OK) {
                ALOGD ("%s: Disabled RF field events", __FUNCTION__);
            } else {
                ALOGE ("%s: Failed to disable RF field events", __FUNCTION__);
            }
        }
        else
        {
            NfcTag::getInstance().connectionEventHandler (connEvent, eventData);
        }
        break;

    case NFA_DEACTIVATED_EVT: // NFC link/protocol deactivated
        ALOGD("%s: NFA_DEACTIVATED_EVT   Type: %u, gIsTagDeactivating: %d", __FUNCTION__, eventData->deactivated.type,gIsTagDeactivating);
        NfcTag::getInstance().setDeactivationState (eventData->deactivated);
        if (eventData->deactivated.type != NFA_DEACTIVATE_TYPE_SLEEP)
        {
            {
                SyncEventGuard g (gDeactivatedEvent);
                gActivated = false; //guard this variable from multi-threaded access
                gDeactivatedEvent.notifyOne ();
            }
            nativeNfcTag_resetPresenceCheck();
            NfcTag::getInstance().connectionEventHandler (connEvent, eventData);
            nativeNfcTag_abortWaits();
            NfcTag::getInstance().abort ();
        }
        else if (gIsTagDeactivating)
        {
            NfcTag::getInstance().setActive(false);
            nativeNfcTag_doDeactivateStatus(0);
        }

        // If RF is activated for what we think is a Secure Element transaction
        // and it is deactivated to either IDLE or DISCOVERY mode, notify w/event.
        if ((eventData->deactivated.type == NFA_DEACTIVATE_TYPE_IDLE)
                || (eventData->deactivated.type == NFA_DEACTIVATE_TYPE_DISCOVERY))
        {
            if (sP2pActive) {
                sP2pActive = false;
                // Make sure RF field events are re-enabled
                ALOGD("%s: NFA_DEACTIVATED_EVT; is p2p", __FUNCTION__);
                // Disable RF field events in case of p2p
                UINT8  nfa_enable_rf_events[] = { 0x01 };

                if (!sIsDisabling && sIsNfaEnabled)
                {
                    ALOGD ("%s: Enabling RF field events", __FUNCTION__);
                    status = NFA_SetConfig(NCI_PARAM_ID_RF_FIELD_INFO, sizeof(nfa_enable_rf_events),
                            &nfa_enable_rf_events[0]);
                    if (status == NFA_STATUS_OK) {
                        ALOGD ("%s: Enabled RF field events", __FUNCTION__);
                    } else {
                        ALOGE ("%s: Failed to enable RF field events", __FUNCTION__);
                    }
                }
            }
        }
        break;

    case NFA_TLV_DETECT_EVT: // TLV Detection complete
        status = eventData->tlv_detect.status;
        ALOGD("%s: NFA_TLV_DETECT_EVT: status = %d, protocol = %d, num_tlvs = %d, num_bytes = %d",
             __FUNCTION__, status, eventData->tlv_detect.protocol,
             eventData->tlv_detect.num_tlvs, eventData->tlv_detect.num_bytes);
        if (status != NFA_STATUS_OK)
        {
            ALOGE("%s: NFA_TLV_DETECT_EVT error: status = %d", __FUNCTION__, status);
        }
        break;

    case NFA_NDEF_DETECT_EVT: // NDEF Detection complete;
        //if status is failure, it means the tag does not contain any or valid NDEF data;
        //pass the failure status to the NFC Service;
        status = eventData->ndef_detect.status;
        ALOGD("%s: NFA_NDEF_DETECT_EVT: status = 0x%X, protocol = %u, "
             "max_size = %lu, cur_size = %lu, flags = 0x%X", __FUNCTION__,
             status,
             eventData->ndef_detect.protocol, eventData->ndef_detect.max_size,
             eventData->ndef_detect.cur_size, eventData->ndef_detect.flags);
        NfcTag::getInstance().connectionEventHandler (connEvent, eventData);
        nativeNfcTag_doCheckNdefResult(status,
            eventData->ndef_detect.max_size, eventData->ndef_detect.cur_size,
            eventData->ndef_detect.flags);
        break;

    case NFA_DATA_EVT: // Data message received (for non-NDEF reads)
        ALOGD("%s: NFA_DATA_EVT:  len = %d", __FUNCTION__, eventData->data.len);
        break;
    case NFA_RW_INTF_ERROR_EVT:
        ALOGD("%s: NFC_RW_INTF_ERROR_EVT", __FUNCTION__);
        nativeNfcTag_notifyRfTimeout();
        break;
    case NFA_SELECT_CPLT_EVT: // Select completed
        status = eventData->status;
        ALOGD("%s: NFA_SELECT_CPLT_EVT: status = %d", __FUNCTION__, status);
        if (status != NFA_STATUS_OK)
        {
            ALOGE("%s: NFA_SELECT_CPLT_EVT error: status = %d", __FUNCTION__, status);
        }
        break;

    case NFA_READ_CPLT_EVT: // NDEF-read or tag-specific-read completed
        ALOGD("%s: NFA_READ_CPLT_EVT: status = 0x%X", __FUNCTION__, eventData->status);
        nativeNfcTag_doReadCompleted (eventData->status);
        NfcTag::getInstance().connectionEventHandler (connEvent, eventData);
        break;

    case NFA_PRESENCE_CHECK_EVT:
        ALOGD("%s: NFA_PRESENCE_CHECK_EVT", __FUNCTION__);
        nativeNfcTag_doPresenceCheckResult (eventData->status);
        break;

    case NFA_CE_UICC_LISTEN_CONFIGURED_EVT :
        ALOGD("%s: NFA_CE_UICC_LISTEN_CONFIGURED_EVT : status=0x%X", __FUNCTION__, eventData->status);
        break;

        //ListenOnly start
    case NFA_EXCLUSIVE_RF_CONTROL_STARTED_EVT:
        {
            ALOGD("%s: NFA_EXCLUSIVE_RF_CONTROL_STARTED_EVT: status = %u", __FUNCTION__, eventData->status);

            SyncEventGuard guard (sNfaEnableDisablePollingEvent);
            sNfaEnableDisablePollingEvent.notifyOne ();
        }
        break;
    case NFA_EXCLUSIVE_RF_CONTROL_STOPPED_EVT:
        {
            ALOGD("%s: NFA_EXCLUSIVE_RF_CONTROL_STOPPED_EVT: status = %u", __FUNCTION__, eventData->status);

            SyncEventGuard guard (sNfaEnableDisablePollingEvent);
            sNfaEnableDisablePollingEvent.notifyOne ();
        }
        break;
        //ListenOnly end

    case NFA_SET_P2P_LISTEN_TECH_EVT:
        {
            ALOGD("%s: NFA_SET_P2P_LISTEN_TECH_EVT", __FUNCTION__);
//            PeerToPeer::getInstance().connectionEventHandler (connEvent, eventData);
        }
        break;

    case NFA_CE_LOCAL_TAG_CONFIGURED_EVT:
        {
            ALOGD("%s: NFA_CE_LOCAL_TAG_CONFIGURED_EVT", __FUNCTION__);
            SyncEventGuard guard (sNfaCeConfigureLocalTag);
            sNfaCeConfigureLocalTag.notifyOne ();
        }
        break;

    default:
        ALOGE("%s: unknown event ????", __FUNCTION__);
        break;
    }
}


/*********************************************************************
 * crash the NFC service process
 *********************************************************************/
#define NFC_HAL_DEVICE_RESET                  0x84
void nfaHalMsgCallback(UINT8 event, UINT16 param_len, UINT8 *p_param)
{
    static const char fn [] = "nfaHalMsgCallback";
    tNFA_STATUS  status = param_len && p_param ? p_param[0] : NFA_STATUS_FAILED;
    ALOGD ("%s: event=0x%X; len=%d status=%d", fn, event, param_len, status );
    kill (getpid(), SIGTERM);
    ALOGD ("%s: exit", fn);
}
static void nfaCrashNfcService()
{
    static const char fn [] = "nfaCrashNfcService";
    tNFA_STATUS  status;
    int ret;
    unsigned long num=0;
    if (GetNumValue (NAME_SELECT_PANIC_ACTION, &num, sizeof(num)) && num)
    {
        switch (num){
        case NFC_SELECT_PANIC_ACTION_RSTKILL:
            ALOGE ("%s: Reset Device and killing the NFC process", fn);
            status = NFA_SendHalControlCommand(NFC_HAL_DEVICE_RESET, 0, NULL, nfaHalMsgCallback);
            if (status == NFA_STATUS_OK) return;
            /* continue if failed */
        case NFC_SELECT_PANIC_ACTION_KILL:
            ALOGE ("%s: killing the NFC process", fn);
            ret = kill(getpid(), SIGTERM);
            if (!ret) return;
            break;
        case NFC_SELECT_PANIC_ACTION_RSTEVENT:
            g_event = D_EVENT_ERROR;
            sem_post(&g_nfa_activated);
            return;
        default:
            ;
        }
    }


    /* XXX maco since this failure is unrecoverable, abort the process */
    abort();
}

/* CXD224x workaround: GEN_ERROR start*/
#ifdef NFA_DEBUG_DUMP
/*******************************************************************************
**
** Device Memory Dump for DEBUG
**
*******************************************************************************/
typedef struct {
    unsigned int addr;
    unsigned int size;
} tNFA_DEBUG_DUMP_ENTRY;

static bool                sDumpDeviceMemoryEn = false;
static bool                sDumpDeviceMemoryAbortEn= true;
static int                 sDumpDeviceMemoryCnt = 0;
const tNFA_DEBUG_DUMP_ENTRY sDumpDeviceMemory[] = {
    {0x010074c0,  12},    /* g_ErrBit_HCI, g_ErrBit_CLUITR, g_ErrBit_CLUTGT */
//  {0x04006000, 4096},
};
static void nfaVscCallback(UINT8 event, UINT16 param_len, UINT8 *p_param);
static void DumpDeviceMemoryOne(void);
#endif

/* check internal entry */
typedef struct {
    unsigned int addr;
    unsigned int value;
} tNFA_CHECK_ENTRY;

/*******************************************************************************
**
** Function:        nfaVscCheckInternalCallback
**
** Description:     Callback function for nfaVscCheckInternal
**
*******************************************************************************/
static void nfaVscCheckInternalCallback(UINT8 event, UINT16 param_len, UINT8 *p_param)
{
    tNFA_STATUS status = NFA_STATUS_FAILED;
    unsigned int value = 1;
    if (param_len == 8)
    {
        status = (tNFA_STATUS )p_param[3];
        value = (unsigned int )(p_param[4] | (p_param[5]<<8) | (p_param[6]<<16) | (p_param[7]<<24));
    }
    ALOGD ("%s: event=0x%X param_len=%d value=0x%x;", __FUNCTION__, event, param_len, value);

    if ((status != NFA_STATUS_OK) || (sNfaCheckInternal != value))
    {
#ifdef NFA_DEBUG_DUMP
        /* internal buffer dump */
        sDumpDeviceMemoryEn=true;
        sDumpDeviceMemoryCnt = 0;
        /* disable log */
        UINT8 disable_log[] = {0xa0, 0x75, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
        NFA_SendVsCommand(0x34, 0x08, disable_log, nfaVscCallback);
#else
        nfaCrashNfcService();
#endif
    }
}
/*******************************************************************************
**
** Function:        nfaVscCheckInternal
**
** Description:     Check firmware internal value
**
*******************************************************************************/
static void nfaVscCheckInternal(tNFA_CHECK_ENTRY *chk)
{
    tNFA_STATUS status;
    if (chk && chk->addr)
    {
        UINT8 adrp[4];
        adrp[0] = (UINT8 )((chk->addr   ) & 0xff);
        adrp[1] = (UINT8 )((chk->addr>>8) & 0xff);
        adrp[2] = (UINT8 )((chk->addr>>16) & 0xff);
        adrp[3] = (UINT8 )((chk->addr>>24) & 0xff);
        sNfaCheckInternal = chk->value;
        status = NFA_SendVsCommand (0x33, 0x04, adrp, nfaVscCheckInternalCallback);
        if (status == NFA_STATUS_OK)
        {
            return;
        }
    }
    /* error or skip resoon check */
    nfaVscCheckInternalCallback(0, 0, NULL);
}

#ifdef NFA_DEBUG_DUMP
/*******************************************************************************
**
** Device Memory Dump for DEBUG
**
*******************************************************************************/

/*******************************************************************************
**
** Function:        nfaVscCallback
**
** Description:     Callback function for DumpDeviceMemoryOne
**
*******************************************************************************/
static void nfaVscCallback(UINT8 event, UINT16 param_len, UINT8 *p_param)
{
    ALOGD ("%s: event=0x%X param_len=%d p_param=0x%u;", __FUNCTION__, event, param_len, (uintptr_t )p_param );

    if (sDumpDeviceMemoryEn)
    {
        DumpDeviceMemoryOne();
    }
}
/*******************************************************************************
**
** Function:        DumpDeviceMemoryOne
**
** Description:     Send Vs Command for read 1 word from Device Memory
**
*******************************************************************************/
static void DumpDeviceMemoryOne(void)
{
    tNFA_STATUS status;
    if (sDumpDeviceMemoryEn)
    {
        int cnt = 0;
        for(unsigned int i=0; i < sizeof(sDumpDeviceMemory)/sizeof(tNFA_DEBUG_DUMP_ENTRY); i++)
        {
            unsigned int addr = sDumpDeviceMemory[i].addr & ~0x3;
            for(unsigned int j=0; j < sDumpDeviceMemory[i].size; j +=4 )
            {
                if (cnt == sDumpDeviceMemoryCnt)
                {
                    UINT8 adrp[4];
                    adrp[0] = (UINT8 )((addr   ) & 0xff);
                    adrp[1] = (UINT8 )((addr>>8) & 0xff);
                    adrp[2] = (UINT8 )((addr>>16) & 0xff);
                    adrp[3] = (UINT8 )((addr>>24) & 0xff);

                    status = NFA_SendVsCommand (0x33, 0x04, adrp, nfaVscCallback);
                    if (status == NFA_STATUS_OK)
                    {
                        sDumpDeviceMemoryCnt++;
                        return;
                    }
                }
                cnt++;
                addr += 4;
            }
        }
        sDumpDeviceMemoryCnt = 0;
        sDumpDeviceMemoryEn = false;

        if (sDumpDeviceMemoryAbortEn)
        {
            ALOGE ("%s: NFA_DM_GEN_ERROR_REVT; internal error abort;", __FUNCTION__ );
            abort();
        }
    }
}
#endif
/* CXD224x workaround: GEN_ERROR end */

/*******************************************************************************
**
** Function:        nfaDeviceManagementCallback
**
** Description:     Receive device management events from stack.
**                  dmEvent: Device-management event ID.
**                  eventData: Data associated with event ID.
**
** Returns:         None
**
*******************************************************************************/
static void nfaDeviceManagementCallback (UINT8 dmEvent, tNFA_DM_CBACK_DATA* eventData)
{
    ALOGD ("%s: enter; event=0x%X", __FUNCTION__, dmEvent);

    switch (dmEvent)
    {
    case NFA_DM_ENABLE_EVT: /* Result of NFA_Enable */
        {
            SyncEventGuard guard (sNfaEnableEvent);
            ALOGD ("%s: NFA_DM_ENABLE_EVT; status=0x%X",
                    __FUNCTION__, eventData->status);
            sIsNfaEnabled = eventData->status == NFA_STATUS_OK;
            sIsDisabling = false;
            sNfaEnableEvent.notifyOne ();
            if (eventData->status == NCI_STATUS_FAILED)
            {
                ALOGD ("%s: NFA_DM_ENABLE_EVT; Unrecoverable Error",
                       __FUNCTION__);
                nfaCrashNfcService();
            }
        }
        break;

    case NFA_DM_DISABLE_EVT: /* Result of NFA_Disable */
        {
            SyncEventGuard guard (sNfaDisableEvent);
            ALOGD ("%s: NFA_DM_DISABLE_EVT", __FUNCTION__);
            sIsNfaEnabled = false;
            sIsDisabling = false;
            sNfaDisableEvent.notifyOne ();
        }
        break;

    case NFA_DM_SET_CONFIG_EVT: //result of NFA_SetConfig
        ALOGD ("%s: NFA_DM_SET_CONFIG_EVT", __FUNCTION__);
        {
            SyncEventGuard guard (sNfaSetConfigEvent);
            sNfaSetConfigEvent.notifyOne();
        }
        break;

    case NFA_DM_GET_CONFIG_EVT: /* Result of NFA_GetConfig */
        ALOGD ("%s: NFA_DM_GET_CONFIG_EVT", __FUNCTION__);
        {
            SyncEventGuard guard (sNfaGetConfigEvent);
            if (eventData->status == NFA_STATUS_OK &&
                    eventData->get_config.tlv_size <= sizeof(sConfig))
            {
                sCurrentConfigLen = eventData->get_config.tlv_size;
                memcpy(sConfig, eventData->get_config.param_tlvs, eventData->get_config.tlv_size);
            }
            else
            {
                ALOGE("%s: NFA_DM_GET_CONFIG failed", __FUNCTION__);
                sCurrentConfigLen = 0;
            }
            sNfaGetConfigEvent.notifyOne();
        }
        break;

    case NFA_DM_RF_FIELD_EVT:
        ALOGD ("%s: NFA_DM_RF_FIELD_EVT; status=0x%X; field status=%u", __FUNCTION__,
              eventData->rf_field.status, eventData->rf_field.rf_field_status);
        if (sIsDisabling || !sIsNfaEnabled)
            break;

        if (!sP2pActive && eventData->rf_field.status == NFA_STATUS_OK)
        break;

    case NFA_DM_NFCC_TRANSPORT_ERR_EVT:
    case NFA_DM_NFCC_TIMEOUT_EVT:
    case NFA_DM_CORE_GEN_ERROR_REVT:
        {
            if (dmEvent == NFA_DM_CORE_GEN_ERROR_REVT)
            {
                if (eventData->status == 0xf0 /*|| eventData->status == 0xa1*/) /* internal error dump */
                {
                    ALOGE ("%s: NFA_DM_NFCC_CORE_ERROR_EVT; 0xf0 abort", __FUNCTION__);
                    tNFA_CHECK_ENTRY chk = {0x010074c8, 0x00006bf};
                    nfaVscCheckInternal(&chk);
                    break;
                }
            }

            if ((dmEvent == NFA_DM_NFCC_TIMEOUT_EVT) || (dmEvent == NFA_DM_NFCC_TRANSPORT_ERR_EVT)
            || ((dmEvent == NFA_DM_CORE_GEN_ERROR_REVT) && (eventData->status == NFA_STATUS_HW_TIMEOUT)))
            {
                if (dmEvent == NFA_DM_NFCC_TIMEOUT_EVT)
                    ALOGE ("%s: NFA_DM_NFCC_TIMEOUT_EVT; abort", __FUNCTION__);
                else if (dmEvent == NFA_DM_NFCC_TRANSPORT_ERR_EVT)
                    ALOGE ("%s: NFA_DM_NFCC_TRANSPORT_ERR_EVT; abort", __FUNCTION__);
                else if (dmEvent == NFA_DM_CORE_GEN_ERROR_REVT)
                    ALOGE ("%s: NFA_DM_NFCC_CORE_ERROR_EVT; abort", __FUNCTION__);

                nativeNfcTag_abortWaits();
                NfcTag::getInstance().abort ();
                sAbortConnlessWait = true;
                {
                    ALOGD ("%s: aborting  sNfaEnableDisablePollingEvent", __FUNCTION__);
                    SyncEventGuard guard (sNfaEnableDisablePollingEvent);
                    sNfaEnableDisablePollingEvent.notifyOne();
                }
                {
                    ALOGD ("%s: aborting  sNfaEnableEvent", __FUNCTION__);
                    SyncEventGuard guard (sNfaEnableEvent);
                    sNfaEnableEvent.notifyOne();
                }
                {
                    ALOGD ("%s: aborting  sNfaDisableEvent", __FUNCTION__);
                    SyncEventGuard guard (sNfaDisableEvent);
                    sNfaDisableEvent.notifyOne();
                }
                sDiscoveryEnabled = false;
                PowerSwitch::getInstance ().abort ();

                if (!sIsDisabling && sIsNfaEnabled)
                {
                    NFA_Disable(false);
                    sIsDisabling = true;
                }
                else
                {
                    sIsNfaEnabled = false;
                    sIsDisabling = false;
                }
                PowerSwitch::getInstance ().initialize (PowerSwitch::UNKNOWN_LEVEL);
                ALOGE ("%s: crash NFC service", __FUNCTION__);
                //////////////////////////////////////////////
                //crash the NFC service process so it can restart automatically
                nfaVscCheckInternal(NULL);
                //////////////////////////////////////////////
            }
        }
        break;

    case NFA_DM_PWR_MODE_CHANGE_EVT:
        PowerSwitch::getInstance ().deviceManagementCallback (dmEvent, eventData);
        break;

#if 0 /* delete */
        /* CXD224x workaround: GEN_ERROR start*/
    case NFA_DM_CORE_GEN_ERROR_REVT:
        ALOGD ("%s: NFA_DM_GEN_ERROR_REVT; status=0x%X;", __FUNCTION__, eventData->status );

        if (eventData->status == 0xf0 /*|| eventData->status == 0xa1*/) /* internal error dump */
        {
            tNFA_CHECK_ENTRY chk = {0x010074c8, 0x00006bf};
            nfaVscCheckInternal(&chk);
        }
        else if (eventData->status == NFA_STATUS_HW_TIMEOUT)
        {
            nfaVscCheckInternal(NULL);
        }
        break;
        /* CXD224x workaround: GEN_ERROR end */
#endif /* delete */

    case NFA_DM_CORE_RESET_NTF_REVT:
        nfaCrashNfcService();
        break;
    default:
#ifdef NFA_FUNCTION_TEST
        /* TEST extra NFA_xx funtion */
        if( !NfaFunctionTest::getInstance().nfaDeviceManagementCallback (dmEvent, eventData) )
        {
            ALOGD ("%s: exit(NFA_FUNCTION_TEST)", __FUNCTION__);
            return ;
        }
#endif /* NFA_FUNCTION_TEST */
        ALOGD ("%s: unhandled event", __FUNCTION__);
        break;
    }
}

/*******************************************************************************
**
** Function:        nfcManager_doInitialize
**
** Description:     Turn on NFC.
**
** Returns:         True if ok.
**
*******************************************************************************/
bool nfcManager_doInitialize (void)
{
    tNFA_STATUS stat = NFA_STATUS_OK;
    ALOGD ("%s: enter; ver=%s nfa=%s NCI_VERSION=0x%02X",
        __FUNCTION__, nfca_version_string, nfa_version_string, NCI_VERSION);

    PowerSwitch & powerSwitch = PowerSwitch::getInstance ();

    if (sIsNfaEnabled)
    {
        ALOGD ("%s: already enabled", __FUNCTION__);
        goto TheEnd;
    }

    powerSwitch.initialize (PowerSwitch::FULL_POWER);

    {
        unsigned long num = 0;

        NfcAdaptation& theInstance = NfcAdaptation::GetInstance();
        theInstance.Initialize(); //start GKI, NCI task, NFC task

        {
            SyncEventGuard guard (sNfaEnableEvent);
            tHAL_NFC_ENTRY* halFuncEntries = theInstance.GetHalEntryFuncs ();

            NFA_Init (halFuncEntries);

            stat = NFA_Enable (nfaDeviceManagementCallback, nfaConnectionCallback);
            if (stat == NFA_STATUS_OK)
            {
                num = initializeGlobalAppLogLevel ();
                CE_SetTraceLevel (num);
                NFC_SetTraceLevel (num);
                RW_SetTraceLevel (num);
                NFA_SetTraceLevel (num);
                sNfaEnableEvent.wait(); //wait for NFA command to finish
            }
        }

        if (stat == NFA_STATUS_OK)
        {
            //sIsNfaEnabled indicates whether stack started successfully
            if (sIsNfaEnabled)
            {
                nativeNfcTag_registerNdefTypeHandler ();
                NfcTag::getInstance().initialize ();

                // if this value exists, set polling interval.
                if (GetNumValue(NAME_NFA_DM_DISC_DURATION_POLL, &num, sizeof(num)))
                    sDiscovery_duration = num;
                else
                    sDiscovery_duration = DEFAULT_DISCOVERY_DURATION;

                NFA_SetRfDiscoveryDuration(sDiscovery_duration);

                // Do custom NFCA startup configuration.
                doStartupConfig();
                goto TheEnd;
            }
        }

        ALOGE ("%s: fail nfa enable; error=0x%X", __FUNCTION__, stat);

        if (sIsNfaEnabled)
            stat = NFA_Disable (false /* ungraceful */);

        theInstance.Finalize();
    }

TheEnd:
    if (sIsNfaEnabled)
        PowerSwitch::getInstance ().setLevel (PowerSwitch::LOW_POWER);
    ALOGD ("%s: exit", __FUNCTION__);
    return sIsNfaEnabled ? true : false;
}


/*******************************************************************************
**
** Function:        nfcManager_enableDiscovery
**
** Description:     Start polling and listening for devices.
**                  mode: Not used.
**
** Returns:         None
**
*******************************************************************************/
void nfcManager_enableDiscovery (tNFA_TECHNOLOGY_MASK t_mask, bool reader_mode, bool restart)
{
    tNFA_TECHNOLOGY_MASK tech_mask = DEFAULT_TECH_MASK;

    tech_mask = (tNFA_TECHNOLOGY_MASK) t_mask;
    ALOGD ("%s: enter; tech_mask = %02x", __FUNCTION__, tech_mask);

    if (sDiscoveryEnabled && !restart)
    {
        ALOGE ("%s: already polling", __FUNCTION__);
        return;
    }

    PowerSwitch::getInstance ().setLevel (PowerSwitch::FULL_POWER);

    if (sRfEnabled) {
        // Stop RF discovery to reconfigure
        startRfDiscovery(false);
    }

    // Check polling configuration
    if (tech_mask != 0)
    {
        stopPolling_rfDiscoveryDisabled();
        startPolling_rfDiscoveryDisabled(tech_mask);

        if (sPollingEnabled)
        {
            ALOGD ("%s: Enable p2pListening", __FUNCTION__);
//            PeerToPeer::getInstance().enableP2pListening (!reader_mode);

            sReaderModeEnabled = true;
            NFA_PauseP2p();
            NFA_SetRfDiscoveryDuration(sDiscovery_duration);
        }
    }
    else
    {
        // No technologies configured, stop polling
        stopPolling_rfDiscoveryDisabled();
    }

    // Actually start discovery.
    startRfDiscovery (true);
    sDiscoveryEnabled = true;

    PowerSwitch::getInstance ().setModeOn (PowerSwitch::DISCOVERY);

    ALOGD ("%s: exit", __FUNCTION__);
}

/*******************************************************************************
**
** Function:        nfcManager_disableDiscovery
**
** Description:     Stop polling and listening for devices.
**
** Returns:         None
**
*******************************************************************************/
bool nfcManager_disableDiscovery (void)
{
    tNFA_STATUS status = NFA_STATUS_OK;
    ALOGD ("%s: enter;", __FUNCTION__);
    int isAlreadyDisable = false;

    if (sDiscoveryEnabled == false)
    {
        ALOGD ("%s: already disabled", __FUNCTION__);
        isAlreadyDisable = true;
        goto TheEnd;
    }

    // Stop RF Discovery.
    startRfDiscovery (false);

    if (sDiscoveryEnabled)
    {
        SyncEventGuard guard (sNfaEnableDisablePollingEvent);
        status = NFA_DisablePolling ();
        if (status == NFA_STATUS_OK)
        {
            sDiscoveryEnabled = false;
            sNfaEnableDisablePollingEvent.wait (); //wait for NFA_POLL_DISABLED_EVT
        }
        else
            ALOGE ("%s: Failed to disable polling; error=0x%X", __FUNCTION__, status);
    }

    //if nothing is active after this, then tell the controller to power down
    if (! PowerSwitch::getInstance ().setModeOff (PowerSwitch::DISCOVERY))
        PowerSwitch::getInstance ().setLevel (PowerSwitch::LOW_POWER);

TheEnd:
    ALOGD ("%s: exit", __FUNCTION__);
    return isAlreadyDisable ? false : true;
}


/*******************************************************************************
**
** Function:        nfcManager_doDeinitialize
**
** Description:     Turn off NFC.
**
** Returns:         True if ok.
**
*******************************************************************************/
bool nfcManager_doDeinitialize (void)
{
    ALOGD ("%s: enter", __FUNCTION__);

#ifdef NFA_FUNCTION_TEST
    /* TEST extra NFA_xx funtion */
    if( !NfaFunctionTest::getInstance().doDeinitialize () )
    {
        ALOGD ("%s: exit(NFA_FUNCTION_TEST)", __FUNCTION__);
        return true;
    }
#endif /* NFA_FUNCTION_TEST */

    sIsDisabling = true;
    PowerSwitch::getInstance ().initialize (PowerSwitch::UNKNOWN_LEVEL);

    if (sIsNfaEnabled)
    {
        SyncEventGuard guard (sNfaDisableEvent);
        tNFA_STATUS stat = NFA_Disable (true /* graceful */);
        if (stat == NFA_STATUS_OK)
        {
            ALOGD ("%s: wait for completion", __FUNCTION__);
            sNfaDisableEvent.wait (); //wait for NFA command to finish
        }
        else
        {
            ALOGE ("%s: fail disable; error=0x%X", __FUNCTION__, stat);
        }
    }
    nativeNfcTag_abortWaits();
    NfcTag::getInstance().abort ();
    sAbortConnlessWait = true;
    sIsNfaEnabled = false;
    sDiscoveryEnabled = false;
    sIsDisabling = false;
    sIsSecElemSelected = false;
    gActivated = false;

    {
        //unblock NFA_EnablePolling() and NFA_DisablePolling()
        SyncEventGuard guard (sNfaEnableDisablePollingEvent);
        sNfaEnableDisablePollingEvent.notifyOne ();
    }

    NfcAdaptation& theInstance = NfcAdaptation::GetInstance();
    theInstance.Finalize();

    NFA_Destroy();

    ALOGD ("%s: exit", __FUNCTION__);
    return true;
}


/*******************************************************************************
**
** Function:        isPeerToPeer
**
** Description:     Whether the activation data indicates the peer supports NFC-DEP.
**                  activated: Activation data.
**
** Returns:         True if the peer supports NFC-DEP.
**
*******************************************************************************/
static bool isPeerToPeer (tNFA_ACTIVATED& activated)
{
    return activated.activate_ntf.protocol == NFA_PROTOCOL_NFC_DEP;
}


/*******************************************************************************
**
** Function:        startRfDiscovery
**
** Description:     Ask stack to start polling and listening for devices.
**                  isStart: Whether to start.
**
** Returns:         None
**
*******************************************************************************/
void startRfDiscovery(bool isStart)
{
    tNFA_STATUS status = NFA_STATUS_FAILED;

    ALOGD ("%s: is start=%d", __FUNCTION__, isStart);
    SyncEventGuard guard (sNfaEnableDisablePollingEvent);
    status  = isStart ? NFA_StartRfDiscovery () : NFA_StopRfDiscovery ();
    if (status == NFA_STATUS_OK)
    {
        sNfaEnableDisablePollingEvent.wait (); //wait for NFA_RF_DISCOVERY_xxxx_EVT
        sRfEnabled = isStart;
    }
    else
    {
        ALOGE ("%s: Failed to start/stop RF discovery; error=0x%X", __FUNCTION__, status);
    }
}


/*******************************************************************************
**
** Function:        doStartupConfig
**
** Description:     Configure the NFC controller.
**
** Returns:         None
**
*******************************************************************************/
void doStartupConfig()
{
    int actualLen = 0;

#if 0 /* not support Active mode */
    tNFA_STATUS stat = NFA_STATUS_FAILED;

    // If polling for Active mode, set the ordering so that we choose Active over Passive mode first.
    if (nat && (nat->tech_mask & (NFA_TECHNOLOGY_MASK_A_ACTIVE | NFA_TECHNOLOGY_MASK_F_ACTIVE)))
    {
        //SkipBRCM start
        /* Skip BRCM proprietary Set_/Get_config */
        unsigned int num=0;
        if (GetNumValue("SKIP_PROP_CONFIG_WORKAROUND", &num, sizeof(num)) && num){
            ;
        }else{
        UINT8  act_mode_order_param[] = { 0x01 };
        SyncEventGuard guard (sNfaSetConfigEvent);
        stat = NFA_SetConfig(NCI_PARAM_ID_ACT_ORDER, sizeof(act_mode_order_param), &act_mode_order_param[0]);
        if (stat == NFA_STATUS_OK)
            sNfaSetConfigEvent.wait ();
        }
        //SkipBRCM end
    }
#endif /* not support Active mode */

    //configure RF polling frequency for each technology
    static tNFA_DM_DISC_FREQ_CFG nfa_dm_disc_freq_cfg;
    //values in the polling_frequency[] map to members of nfa_dm_disc_freq_cfg
    UINT8 polling_frequency [8] = {1, 1, 1, 1, 1, 1, 1, 1};
    actualLen = GetStrValue(NAME_POLL_FREQUENCY, (char*)polling_frequency, 8);
    if (actualLen == 8)
    {
        ALOGD ("%s: polling frequency", __FUNCTION__);
        memset (&nfa_dm_disc_freq_cfg, 0, sizeof(nfa_dm_disc_freq_cfg));
        nfa_dm_disc_freq_cfg.pa = polling_frequency [0];
        nfa_dm_disc_freq_cfg.pb = polling_frequency [1];
        nfa_dm_disc_freq_cfg.pf = polling_frequency [2];
        nfa_dm_disc_freq_cfg.pi93 = polling_frequency [3];
        nfa_dm_disc_freq_cfg.pbp = polling_frequency [4];
        nfa_dm_disc_freq_cfg.pk = polling_frequency [5];
        nfa_dm_disc_freq_cfg.paa = polling_frequency [6];
        nfa_dm_disc_freq_cfg.pfa = polling_frequency [7];
        p_nfa_dm_rf_disc_freq_cfg = &nfa_dm_disc_freq_cfg;
    }
}


/*******************************************************************************
**
** Function:        nfcManager_isNfcActive
**
** Description:     Used externaly to determine if NFC is active or not.
**
** Returns:         'true' if the NFC stack is running, else 'false'.
**
*******************************************************************************/
bool nfcManager_isNfcActive()
{
    return sIsNfaEnabled;
}


static tNFA_STATUS startPolling_rfDiscoveryDisabled (tNFA_TECHNOLOGY_MASK tech_mask)
{
    tNFA_STATUS stat = NFA_STATUS_FAILED;

    unsigned long num = 0;

    if (tech_mask == 0 && GetNumValue(NAME_POLLING_TECH_MASK, &num, sizeof(num)))
        tech_mask = num;
    else if (tech_mask == 0) tech_mask = DEFAULT_TECH_MASK;

    SyncEventGuard guard (sNfaEnableDisablePollingEvent);
    ALOGD ("%s: enable polling", __FUNCTION__);
    stat = NFA_EnablePolling (tech_mask);
    if (stat == NFA_STATUS_OK)
    {
        ALOGD ("%s: wait for enable event", __FUNCTION__);
        sPollingEnabled = true;
        sNfaEnableDisablePollingEvent.wait (); //wait for NFA_POLL_ENABLED_EVT
    }
    else
    {
        ALOGE ("%s: fail enable polling; error=0x%X", __FUNCTION__, stat);
    }

    return stat;
}

static tNFA_STATUS stopPolling_rfDiscoveryDisabled (void)
{
    tNFA_STATUS stat = NFA_STATUS_FAILED;

    SyncEventGuard guard (sNfaEnableDisablePollingEvent);
    ALOGD ("%s: disable polling", __FUNCTION__);
    stat = NFA_DisablePolling ();
    if (stat == NFA_STATUS_OK) {
        sPollingEnabled = false;
        sNfaEnableDisablePollingEvent.wait (); //wait for NFA_POLL_DISABLED_EVT
    } else {
        ALOGE ("%s: fail disable polling; error=0x%X", __FUNCTION__, stat);
    }

    return stat;
}


#if (defined (NFA_CE_INCLUDED) && (NFA_CE_INCLUDED==TRUE))
void nfcHce_settingTag(tNFA_TECHNOLOGY_MASK t_mask_listen, UINT8 *data, UINT16 cur_size, UINT16 max_size, bool read_only)
{
    tNFA_PROTOCOL_MASK proto = 0;

    if(t_mask_listen == 0) return;

    if(((t_mask_listen & NFA_TECHNOLOGY_MASK_A) == NFA_TECHNOLOGY_MASK_A)
    || ((t_mask_listen & NFA_TECHNOLOGY_MASK_B) == NFA_TECHNOLOGY_MASK_B)) {
        proto |= NFA_PROTOCOL_MASK_ISO_DEP;
    }

    if((t_mask_listen & NFA_TECHNOLOGY_MASK_F) == NFA_TECHNOLOGY_MASK_F) {
        proto |= NFA_PROTOCOL_MASK_T3T;
    }

    {
        SyncEventGuard guard (sNfaCeConfigureLocalTag);
        NFA_CeConfigureLocalTag(proto, data, cur_size, max_size, read_only, 0, NULL);
        sNfaCeConfigureLocalTag.wait (); //wait for NFA_CE_LOCAL_TAG_CONFIGURED_EVT  
    }

    return;
}
#endif /*(defined (NFA_CE_INCLUDED) && (NFA_CE_INCLUDED==TRUE))*/
