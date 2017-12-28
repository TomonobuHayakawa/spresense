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
#include <time.h>
#include <signal.h>
#include "OverrideLog.h"
//#include "NfcJniUtil.h"
#include "NfcTag.h"
#include "config.h"
#include "Mutex.h"
//#include "IntervalTimer.h"
#ifndef SPZ2_IMPL
// Need C++ standard library
#include <iostream>
#endif
#include <string.h>
#include <stdlib.h>

extern "C"
{
    #include "nfa_api.h"
    #include "nfa_rw_api.h"
    #include "ndef_utils.h"
    #include "rw_api.h"
#ifdef SPZ1_IMPL
#include <kernel.h>
#include <os_wrapper.h>
#endif /* SPZ1_IMPL */
}

#define D_NFC_NDEF_SIZE_OVER 0xE0
#define LOCAL_NDEF_MAX 511

extern bool nfcManager_isNfcActive();

extern bool         gActivated;
extern SyncEvent    gDeactivatedEvent;
int ndef[2];

/*****************************************************************************
**
** public variables and functions
**
*****************************************************************************/
    bool    gIsTagDeactivating = false;    // flag for nfa callback indicating we are deactivating for RF interface switch
    bool    gIsSelectingRfInterface = false; // flag for nfa callback indicating we are selecting for RF interface switch


/*****************************************************************************
**
** private variables and functions
**
*****************************************************************************/
// Pre-defined tag type values. These must match the values in
// framework Ndef.java for Google public NFC API.
#define NDEF_UNKNOWN_TYPE          -1
#define NDEF_TYPE1_TAG             1
#define NDEF_TYPE2_TAG             2
#define NDEF_TYPE3_TAG             3
#define NDEF_TYPE4_TAG             4

#define NDEF_MIFARE_CLASSIC_TAG    101

#define STATUS_CODE_TARGET_LOST    146	// this error code comes from the service

#define NDEF_MODE_READ_ONLY              1
#define NDEF_MODE_READ_WRITE             2
#define NDEF_MODE_UNKNOWN                3

static uint32_t     sCheckNdefCurrentSize = 0;
static tNFA_STATUS  sCheckNdefStatus = 0; //whether tag already contains a NDEF message
static bool         sCheckNdefCapable = false; //whether tag has NDEF capability
static tNFA_HANDLE  sNdefTypeHandlerHandle = NFA_HANDLE_INVALID;
static tNFA_INTF_TYPE   sCurrentRfInterface = NFA_INTERFACE_ISO_DEP;
static bool         sWaitingForTransceive = false;
static bool         sTransceiveRfTimeout = false;
static Mutex        sRfInterfaceMutex;
static uint32_t     sReadDataLen = 0;
static uint8_t*     sReadData = NULL;
static bool         sIsReadingNdefMessage = false;
static SyncEvent    sReadEvent;
static sem_t        sWriteSem;
static sem_t        sFormatSem;
static SyncEvent    sTransceiveEvent;
static SyncEvent    sReconnectEvent;
static sem_t        sCheckNdefSem;
static SyncEvent    sPresenceCheckEvent;
static sem_t        sMakeReadonlySem;
static bool     sConnectOk = false;
static bool     sConnectWaitingForComplete = false;
static bool         sGotDeactivate = false;
static uint32_t     sCheckNdefMaxSize = 0;
static bool         sCheckNdefCardReadOnly = false;
static bool         sCheckNdefWaitingForComplete = false;
static bool         sIsTagPresent = true;
static int          sCurrentConnectedTargetType = TARGET_TYPE_UNKNOWN;

/*******************************************************************************
**
** Function:        nativeNfcTag_abortWaits
**
** Description:     Unblock all thread synchronization objects.
**
** Returns:         None
**
*******************************************************************************/
void nativeNfcTag_abortWaits ()
{
    ALOGD ("%s", __FUNCTION__);
    {
        SyncEventGuard g (sReadEvent);
        sReadEvent.notifyOne ();
    }
    sem_post (&sWriteSem);
    sem_post (&sFormatSem);
    {
        SyncEventGuard g (sTransceiveEvent);
        sTransceiveEvent.notifyOne ();
    }
    {
        SyncEventGuard g (sReconnectEvent);
        sReconnectEvent.notifyOne ();
    }

    sem_post (&sCheckNdefSem);
    {
        SyncEventGuard guard (sPresenceCheckEvent);
        sPresenceCheckEvent.notifyOne ();
    }
    sem_post (&sMakeReadonlySem);
    sCurrentRfInterface = NFA_INTERFACE_ISO_DEP;
    sCurrentConnectedTargetType = TARGET_TYPE_UNKNOWN;
}


/*******************************************************************************
**
** Function:        nativeNfcTag_doReadCompleted
**
** Description:     Receive the completion status of read operation.  Called by
**                  NFA_READ_CPLT_EVT.
**                  status: Status of operation.
**
** Returns:         None
**
*******************************************************************************/
void nativeNfcTag_doReadCompleted (tNFA_STATUS status)
{
    ALOGD ("%s: status=0x%X; is reading=%u", __FUNCTION__, status, sIsReadingNdefMessage);

    if (sIsReadingNdefMessage == false)
        return; //not reading NDEF message right now, so just return

    if (status != NFA_STATUS_OK)
    {
        sReadDataLen = 0;
        if (sReadData)
            free (sReadData);
        sReadData = NULL;
    }
    SyncEventGuard g (sReadEvent);
    sReadEvent.notifyOne ();
}


/*******************************************************************************
**
** Function:        ndefHandlerCallback
**
** Description:     Receive NDEF-message related events from stack.
**                  event: Event code.
**                  p_data: Event data.
**
** Returns:         None
**
*******************************************************************************/
static void ndefHandlerCallback (tNFA_NDEF_EVT event, tNFA_NDEF_EVT_DATA *eventData)
{
    ALOGD ("%s: event=%u, eventData=%p", __FUNCTION__, event, eventData);

    switch (event)
    {
    case NFA_NDEF_REGISTER_EVT:
        {
            tNFA_NDEF_REGISTER& ndef_reg = eventData->ndef_reg;
            ALOGD ("%s: NFA_NDEF_REGISTER_EVT; status=0x%X; h=0x%X", __FUNCTION__, ndef_reg.status, ndef_reg.ndef_type_handle);
            sNdefTypeHandlerHandle = ndef_reg.ndef_type_handle;
        }
        break;

    case NFA_NDEF_DATA_EVT:
        {
            ALOGD ("%s: NFA_NDEF_DATA_EVT; data_len = %lu", __FUNCTION__, eventData->ndef_data.len);
            sReadDataLen = eventData->ndef_data.len;
            sReadData = (uint8_t*) malloc (sReadDataLen);
            memcpy (sReadData, eventData->ndef_data.p_data, eventData->ndef_data.len);
        }
        break;

    default:
        ALOGE ("%s: Unknown event %u ????", __FUNCTION__, event);
        break;
    }
}


/*******************************************************************************
**
** Function:        nativeNfcTag_doRead
**
** Description:     Read the NDEF message on the tag.
**
** Returns:         NDEF message.
**
*******************************************************************************/
tNFA_STATUS nativeNfcTag_doRead (unsigned char *buf, unsigned short buf_size, int *size)
{
    ALOGD ("%s: enter", __FUNCTION__);
    tNFA_STATUS status = NFA_STATUS_FAILED;

    sReadDataLen = 0;
    if (sReadData != NULL)
    {
        free (sReadData);
        sReadData = NULL;
    }

    if (sCheckNdefCurrentSize > 0)
    {
        {
            SyncEventGuard g (sReadEvent);
            sIsReadingNdefMessage = true;
            status = NFA_RwReadNDef ();
            sReadEvent.wait (); //wait for NFA_READ_CPLT_EVT
        }
        sIsReadingNdefMessage = false;

        if (sReadDataLen > buf_size) {
            sReadDataLen = buf_size;
            status = NCI_STATUS_MSG_SIZE_TOO_BIG;
        }

        if (sReadDataLen > 0) //if stack actually read data from the tag
        {
            ALOGD ("%s: read %u bytes", __FUNCTION__, sReadDataLen);
            *size = (int)sReadDataLen;
            memcpy (buf, sReadData, sReadDataLen);
        }
    }
    else
    {
        ALOGD ("%s: create emtpy buffer", __FUNCTION__);
        sReadDataLen = 0;
        sReadData = NULL;
        *size = 0;
    }

    if (sReadData)
    {
        free (sReadData);
        sReadData = NULL;
    }
    sReadDataLen = 0;

    ALOGD ("%s: exit", __FUNCTION__);
    return status;
}


/*******************************************************************************
**
** Function:        nativeNfcTag_doConnectStatus
**
** Description:     Receive the completion status of connect operation.
**                  isConnectOk: Status of the operation.
**
** Returns:         None
**
*******************************************************************************/
void nativeNfcTag_doConnectStatus (bool isConnectOk)
{
    if (sConnectWaitingForComplete != false)
    {
        sConnectWaitingForComplete = false;
        sConnectOk = isConnectOk;
        SyncEventGuard g (sReconnectEvent);
        sReconnectEvent.notifyOne ();
    }
}


/*******************************************************************************
**
** Function:        nativeNfcTag_doDeactivateStatus
**
** Description:     Receive the completion status of deactivate operation.
**
** Returns:         None
**
*******************************************************************************/
void nativeNfcTag_doDeactivateStatus (int status)
{
    sGotDeactivate = (status == 0);

    SyncEventGuard g (sReconnectEvent);
    sReconnectEvent.notifyOne ();
}



void nativeNfcTag_notifyRfTimeout ()
{
    SyncEventGuard g (sTransceiveEvent);
    ALOGD ("%s: waiting for transceive: %d", __FUNCTION__, sWaitingForTransceive);
    if (!sWaitingForTransceive)
        return;

    sTransceiveRfTimeout = true;

    sTransceiveEvent.notifyOne ();
}


/*******************************************************************************
**
** Function:        nativeNfcTag_doCheckNdefResult
**
** Description:     Receive the result of checking whether the tag contains a NDEF
**                  message.  Called by the NFA_NDEF_DETECT_EVT.
**                  status: Status of the operation.
**                  maxSize: Maximum size of NDEF message.
**                  currentSize: Current size of NDEF message.
**                  flags: Indicate various states.
**
** Returns:         None
**
*******************************************************************************/
void nativeNfcTag_doCheckNdefResult (tNFA_STATUS status, uint32_t maxSize, uint32_t currentSize, uint8_t flags)
{
    //this function's flags parameter is defined using the following macros
    //in nfc/include/rw_api.h;
    //#define RW_NDEF_FL_READ_ONLY  0x01    /* Tag is read only              */
    //#define RW_NDEF_FL_FORMATED   0x02    /* Tag formated for NDEF         */
    //#define RW_NDEF_FL_SUPPORTED  0x04    /* NDEF supported by the tag     */
    //#define RW_NDEF_FL_UNKNOWN    0x08    /* Unable to find if tag is ndef capable/formated/read only */
    //#define RW_NDEF_FL_FORMATABLE 0x10    /* Tag supports format operation */

    if (status == NFC_STATUS_BUSY)
    {
        ALOGE ("%s: stack is busy", __FUNCTION__);
        return;
    }

    if (!sCheckNdefWaitingForComplete)
    {
        ALOGE ("%s: not waiting", __FUNCTION__);
        return;
    }

    if (flags & RW_NDEF_FL_READ_ONLY)
        ALOGD ("%s: flag read-only", __FUNCTION__);
    if (flags & RW_NDEF_FL_FORMATED)
        ALOGD ("%s: flag formatted for ndef", __FUNCTION__);
    if (flags & RW_NDEF_FL_SUPPORTED)
        ALOGD ("%s: flag ndef supported", __FUNCTION__);
    if (flags & RW_NDEF_FL_UNKNOWN)
        ALOGD ("%s: flag all unknown", __FUNCTION__);
    if (flags & RW_NDEF_FL_FORMATABLE)
        ALOGD ("%s: flag formattable", __FUNCTION__);

    sCheckNdefWaitingForComplete = false;
    sCheckNdefStatus = status;
    sCheckNdefCapable = false; //assume tag is NOT ndef capable
    if (sCheckNdefStatus == NFA_STATUS_OK)
    {
        //NDEF content is on the tag
        sCheckNdefMaxSize = maxSize;
        sCheckNdefCurrentSize = currentSize;
        sCheckNdefCardReadOnly = flags & RW_NDEF_FL_READ_ONLY;
        sCheckNdefCapable = true;
    }
    else if (sCheckNdefStatus == NFA_STATUS_FAILED)
    {
        //no NDEF content on the tag
        sCheckNdefMaxSize = 0;
        sCheckNdefCurrentSize = 0;
        sCheckNdefCardReadOnly = flags & RW_NDEF_FL_READ_ONLY;
        if ((flags & RW_NDEF_FL_UNKNOWN) == 0) //if stack understands the tag
        {
            if (flags & RW_NDEF_FL_SUPPORTED) //if tag is ndef capable
                sCheckNdefCapable = true;
        }
    }
    else
    {
        ALOGE ("%s: unknown status=0x%X", __FUNCTION__, status);
        sCheckNdefMaxSize = 0;
        sCheckNdefCurrentSize = 0;
        sCheckNdefCardReadOnly = false;
    }
    sem_post (&sCheckNdefSem);
}


/*******************************************************************************
**
** Function:        nativeNfcTag_doCheckNdef
**
** Description:     Does the tag contain a NDEF message?
**                  ndefInfo: NDEF info.
**
** Returns:         Status code; 0 is success.
**
*******************************************************************************/
int nativeNfcTag_doCheckNdef(void)
{
    tNFA_STATUS status = NFA_STATUS_FAILED;

    ALOGD ("%s: enter", __FUNCTION__);

    /* Create the write semaphore */
    if (sem_init (&sCheckNdefSem, 0, 0) == -1)
    {
        ALOGE ("%s: Check NDEF semaphore creation failed (errno=0x%08x)", __FUNCTION__, errno);
        return false;
    }

    if (NfcTag::getInstance ().getActivationState () != NfcTag::Active)
    {
        ALOGE ("%s: tag already deactivated", __FUNCTION__);
        goto TheEnd;
    }

    ALOGD ("%s: try NFA_RwDetectNDef", __FUNCTION__);
    sCheckNdefWaitingForComplete = true;
    status = NFA_RwDetectNDef ();

    if (status != NFA_STATUS_OK)
    {
        ALOGE ("%s: NFA_RwDetectNDef failed, status = 0x%X", __FUNCTION__, status);
        goto TheEnd;
    }

    /* Wait for check NDEF completion status */
    if (sem_wait (&sCheckNdefSem))
    {
        ALOGE ("%s: Failed to wait for check NDEF semaphore (errno=0x%08x)", __FUNCTION__, errno);
        goto TheEnd;
    }

    if (sCheckNdefStatus == NFA_STATUS_OK)
    {
        //stack found a NDEF message on the tag
        if (NfcTag::getInstance ().getProtocol () == NFA_PROTOCOL_T1T)
            ndef[0] = NfcTag::getInstance ().getT1tMaxMessageSize ();
        else
            ndef[0] = sCheckNdefMaxSize;
        if (sCheckNdefCardReadOnly)
            ndef[1] = NDEF_MODE_READ_ONLY;
        else
            ndef[1] = NDEF_MODE_READ_WRITE;
        status = NFA_STATUS_OK;
    }
    else if (sCheckNdefStatus == NFA_STATUS_FAILED)
    {
        //stack did not find a NDEF message on the tag;
        if (NfcTag::getInstance ().getProtocol () == NFA_PROTOCOL_T1T)
            ndef[0] = NfcTag::getInstance ().getT1tMaxMessageSize ();
        else
            ndef[0] = sCheckNdefMaxSize;
        if (sCheckNdefCardReadOnly)
            ndef[1] = NDEF_MODE_READ_ONLY;
        else
            ndef[1] = NDEF_MODE_READ_WRITE;
        status = NFA_STATUS_FAILED;
    }
    else if (sCheckNdefStatus == NFA_STATUS_TIMEOUT)
    {
        status = sCheckNdefStatus;
    }
    else
    {
        ALOGD ("%s: unknown status 0x%X", __FUNCTION__, sCheckNdefStatus);
        status = sCheckNdefStatus;
    }

TheEnd:
    /* Destroy semaphore */
    if (sem_destroy (&sCheckNdefSem))
    {
        ALOGE ("%s: Failed to destroy check NDEF semaphore (errno=0x%08x)", __FUNCTION__, errno);
    }
    sCheckNdefWaitingForComplete = false;
    ALOGD ("%s: exit; status=0x%X", __FUNCTION__, status);
    return status;
}


/*******************************************************************************
**
** Function:        nativeNfcTag_resetPresenceCheck
**
** Description:     Reset variables related to presence-check.
**
** Returns:         None
**
*******************************************************************************/
void nativeNfcTag_resetPresenceCheck ()
{
    sIsTagPresent = true;
}


/*******************************************************************************
**
** Function:        nativeNfcTag_doPresenceCheckResult
**
** Description:     Receive the result of presence-check.
**                  status: Result of presence-check.
**
** Returns:         None
**
*******************************************************************************/
void nativeNfcTag_doPresenceCheckResult (tNFA_STATUS status)
{
    SyncEventGuard guard (sPresenceCheckEvent);
    sIsTagPresent = status == NFA_STATUS_OK;
    sPresenceCheckEvent.notifyOne ();
}


/*******************************************************************************
**
** Function:        nativeNfcTag_doPresenceCheck
**
** Description:     Check if the tag is in the RF field.
**                  e: JVM environment.
**                  o: Java object.
**
** Returns:         True if tag is in RF field.
**
*******************************************************************************/
bool nativeNfcTag_doPresenceCheck (void)
{
    ALOGD ("%s", __FUNCTION__);
    tNFA_STATUS status = NFA_STATUS_OK;
    bool isPresent = false;

    // Special case for Kovio.  The deactivation would have already occurred
    // but was ignored so that normal tag opertions could complete.  Now we
    // want to process as if the deactivate just happened.
    if (NfcTag::getInstance ().mTechList [0] == TARGET_TYPE_KOVIO_BARCODE)
    {
        ALOGD ("%s: Kovio, force deactivate handling", __FUNCTION__);
        tNFA_DEACTIVATED deactivated = {NFA_DEACTIVATE_TYPE_IDLE};
        {
            SyncEventGuard g (gDeactivatedEvent);
            gActivated = false; //guard this variable from multi-threaded access
            gDeactivatedEvent.notifyOne ();
        }

        NfcTag::getInstance().setDeactivationState (deactivated);
        nativeNfcTag_resetPresenceCheck();
        NfcTag::getInstance().connectionEventHandler (NFA_DEACTIVATED_EVT, NULL);
        nativeNfcTag_abortWaits();
        NfcTag::getInstance().abort ();

        return false;
    }

    if (nfcManager_isNfcActive() == false)
    {
        ALOGD ("%s: NFC is no longer active.", __FUNCTION__);
        return false;
    }

    if (!sRfInterfaceMutex.tryLock())
    {
        ALOGD ("%s: tag is being reSelected assume it is present", __FUNCTION__);
        return true;
    }

    sRfInterfaceMutex.unlock();

    if (NfcTag::getInstance ().isActivated () == false)
    {
        ALOGD ("%s: tag already deactivated", __FUNCTION__);
        return false;
    }

    {
        SyncEventGuard guard (sPresenceCheckEvent);
        status = NFA_RwPresenceCheck (NfcTag::getInstance().getPresenceCheckAlgorithm());
        if (status == NFA_STATUS_OK)
        {
            sPresenceCheckEvent.wait ();
            isPresent = sIsTagPresent ? true : false;
        }
    }

    if (isPresent == false)
        ALOGD ("%s: tag absent", __FUNCTION__);
    return isPresent;
}


/*******************************************************************************
**
** Function:        nativeNfcTag_registerNdefTypeHandler
**
** Description:     Register a callback to receive NDEF message from the tag
**                  from the NFA_NDEF_DATA_EVT.
**
** Returns:         None
**
*******************************************************************************/
//register a callback to receive NDEF message from the tag
//from the NFA_NDEF_DATA_EVT;
void nativeNfcTag_registerNdefTypeHandler ()
{
    ALOGD ("%s", __FUNCTION__);
    sNdefTypeHandlerHandle = NFA_HANDLE_INVALID;
    NFA_RegisterNDefTypeHandler (TRUE, NFA_TNF_DEFAULT, (UINT8 *) "", 0, ndefHandlerCallback);
}


/*******************************************************************************
**
** Function:        nativeNfcTag_deregisterNdefTypeHandler
**
** Description:     No longer need to receive NDEF message from the tag.
**
** Returns:         None
**
*******************************************************************************/
void nativeNfcTag_deregisterNdefTypeHandler ()
{
    ALOGD ("%s", __FUNCTION__);
    NFA_DeregisterNDefTypeHandler (sNdefTypeHandlerHandle);
    sNdefTypeHandlerHandle = NFA_HANDLE_INVALID;
}

