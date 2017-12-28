/******************************************************************************
 *
 *  Copyright (C) 2012 Broadcom Corporation
 *  Copyright (C) 2013 Sony Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/******************************************************************************
 *
 *  HAL Adaptation Interface (HAI). This interface regulates the interaction
 *  between standard Android HAL and Broadcom-specific HAL.  It adapts
 *  Broadcom-specific features to the Android framework.
 *
 ******************************************************************************/
#define LOG_TAG "NfcNciHal"
#include "OverrideLog.h"
#include "HalAdaptation.h"
#include "SyncEvent.h"
#include "config.h"
#include "nfc_hal_int_extra.h"
#include "nfc_hal_api_extra.h"
#include "nfc_hal_post_reset.h"
#include <errno.h>
#include <pthread.h>
#include "buildcfg.h"

#if (defined(NFC_HAL_HCI_INCLUDED) && (NFC_HAL_HCI_INCLUDED == TRUE))
extern void delete_hal_non_volatile_store (bool forceDelete);
extern int verify_hal_non_volatile_store ();
#else
#define verify_hal_non_volatile_store() (0)
#define delete_hal_non_volatile_store(bool)
#endif

extern "C"
{
#include "userial.h"
#ifdef SPZ1_IMPL
#include "pthread_wrapper.h"
#endif /* SPZ1_IMPL */
}


///////////////////////////////////////
// private declaration, definition


static nfc_stack_callback_t* gAndroidHalCallback = NULL;
static nfc_stack_data_callback_t* gAndroidHalDataCallback = NULL;
static SyncEvent gOpenCompletedEvent;
static SyncEvent gPostInitCompletedEvent;
static SyncEvent gCloseCompletedEvent;

UINT32 ScrProtocolTraceFlag_Hal = SCR_PROTO_TRACE_ALL; //0x017F00;

static void BroadcomHalCallback (UINT8 event, tHAL_NFC_STATUS status);
static void BroadcomHalDataCallback (UINT16 data_len, UINT8* p_data);

extern tNFC_HAL_CFG *p_nfc_hal_cfg;
extern const UINT8  nfca_version_string [];
extern const UINT8  nfa_version_string [];

///////////////////////////////////////

#define HAI_INIT_MAX_CMPLEN 20
int HaiInitializeLibrary (const cxd224x_dev_t* device)
{
    ALOGD ("%s: ver=%s nfa=%s", __FUNCTION__, nfca_version_string, nfa_version_string);
    int retval = EACCES;
    unsigned long num = 0;
    char temp[120];
    UINT8 logLevel = 0;

    logLevel = InitializeGlobalAppLogLevel ();

/*
    if ( GetNumValue ( NAME_PRESERVE_STORAGE, (char*)&num, sizeof ( num ) ) &&
            (num == 1) )
        ALOGD ("%s: preserve HAL NV store", __FUNCTION__);
    else
    {
        delete_hal_non_volatile_store (false);
    }
*/

    // Initialize protocol logging level
    InitializeProtocolLogLevel ();

    tUSERIAL_OPEN_CFG cfg;
    struct tUART_CONFIG  uart;
    memset( &uart, 0, sizeof(struct tUART_CONFIG));

    if ( GetStrValue ( NAME_UART_PARITY, temp, sizeof ( temp ) ) )
    {
        if ( strncmp ( temp, "even", HAI_INIT_MAX_CMPLEN ) == 0 )
            uart.m_iParity = USERIAL_PARITY_EVEN;
        else if ( strncmp ( temp, "odd", HAI_INIT_MAX_CMPLEN ) == 0 )
            uart.m_iParity = USERIAL_PARITY_ODD;
        else if ( strncmp ( temp, "none", HAI_INIT_MAX_CMPLEN ) == 0 )
            uart.m_iParity = USERIAL_PARITY_NONE;
    }
    else
        uart.m_iParity = USERIAL_PARITY_NONE;

    if ( GetStrValue ( NAME_UART_STOPBITS, temp, sizeof ( temp ) ) )
    {
        if ( strncmp ( temp, "1", HAI_INIT_MAX_CMPLEN ) == 0 )
            uart.m_iStopbits = USERIAL_STOPBITS_1;
        else if ( strncmp ( temp, "2", HAI_INIT_MAX_CMPLEN ) == 0 )
            uart.m_iStopbits = USERIAL_STOPBITS_2;
        else if ( strncmp ( temp, "1.5", HAI_INIT_MAX_CMPLEN ) == 0 )
            uart.m_iStopbits = USERIAL_STOPBITS_1_5;
    }
    else if ( GetNumValue ( NAME_UART_STOPBITS, &num, sizeof ( num ) ) )
    {
        if ( num == 1 )
            uart.m_iStopbits = USERIAL_STOPBITS_1;
        else if ( num == 2 )
            uart.m_iStopbits = USERIAL_STOPBITS_2;
    }
    else
        uart.m_iStopbits = USERIAL_STOPBITS_1;

    if ( GetNumValue ( NAME_UART_DATABITS, &num, sizeof ( num ) ) )
    {
        if ( 5 <= num && num <= 8 )
            uart.m_iDatabits = ( 1 << ( num + 1 ) );
    }
    else
        uart.m_iDatabits = USERIAL_DATABITS_8;

    if ( GetNumValue ( NAME_UART_BAUD, &num, sizeof ( num ) ) )
    {
        if ( num == 300 ) uart.m_iBaudrate = USERIAL_BAUD_300;
        else if ( num == 600 ) uart.m_iBaudrate = USERIAL_BAUD_600;
        else if ( num == 1200 ) uart.m_iBaudrate = USERIAL_BAUD_1200;
        else if ( num == 2400 ) uart.m_iBaudrate = USERIAL_BAUD_2400;
        else if ( num == 9600 ) uart.m_iBaudrate = USERIAL_BAUD_9600;
        else if ( num == 19200 ) uart.m_iBaudrate = USERIAL_BAUD_19200;
        else if ( num == 57600 ) uart.m_iBaudrate = USERIAL_BAUD_57600;
        else if ( num == 115200 ) uart.m_iBaudrate = USERIAL_BAUD_115200;
        else if ( num == 230400 ) uart.m_iBaudrate = USERIAL_BAUD_230400;
        else if ( num == 460800 ) uart.m_iBaudrate = USERIAL_BAUD_460800;
        else if ( num == 921600 ) uart.m_iBaudrate = USERIAL_BAUD_921600;
    }
    else if ( GetStrValue ( NAME_UART_BAUD, temp, sizeof ( temp ) ) )
    {
        if ( strncmp ( temp, "auto", HAI_INIT_MAX_CMPLEN ) == 0 )
            uart.m_iBaudrate = USERIAL_BAUD_AUTO;
    }
    else
        uart.m_iBaudrate = USERIAL_BAUD_115200;

    memset (&cfg, 0, sizeof(tUSERIAL_OPEN_CFG));
    cfg.fmt = uart.m_iDatabits | uart.m_iParity | uart.m_iStopbits;
    cfg.baud = uart.m_iBaudrate;

    ALOGD ("%s: uart config=0x%04x, %d\n", __func__, cfg.fmt, cfg.baud);
    USERIAL_Init(&cfg);

    if ( GetNumValue ( NAME_NFCC_ENABLE_TIMEOUT, &num, sizeof ( num ) ) )
    {
        p_nfc_hal_cfg->nfc_hal_nfcc_enable_timeout = num;
    }

    HAL_NfcInitialize ();
    HAL_NfcSetTraceLevel (logLevel); // Initialize HAL's logging level

    /* add: NFC NCI header,footer and prop packet ctrl */
    if ( GetNumValue ( NAME_HAL_INIT_CTRL, &num,  sizeof ( num ) ) )
        HAL_NfcSetoptionalparm ( NFC_OPTPARM_HAL_INIT_CTRL, num );
    else
#ifndef SPZ_IMPL
        HAL_NfcSetoptionalparm ( NFC_OPTPARM_HAL_INIT_CTRL, 0 );
#else
        num = 1;
        HAL_NfcSetoptionalparm ( NFC_OPTPARM_HAL_INIT_CTRL, num );
#endif /* SPZ_IMPL */

#if (defined(NFC_HAL_HCI_INCLUDED) && (NFC_HAL_HCI_INCLUDED == TRUE))
    /* verify nfaStrage.bin1 and if invalid, mark to run NvErase */
    HAL_NfcSetoptionalparm( NFC_OPTPARM_HAL_NVERASE, verify_hal_non_volatile_store ());
#endif

    retval = 0;
    ALOGD ("%s: exit %d", __FUNCTION__, retval);
    return retval;
}


int HaiTerminateLibrary ()
{
    int retval = EACCES;
    ALOGD ("%s: enter", __FUNCTION__);

    HAL_NfcTerminate ();
    gAndroidHalCallback = NULL;
    gAndroidHalDataCallback = NULL;
    GKIH_shutdown ();
    retval = 0;
    ALOGD ("%s: exit %d", __FUNCTION__, retval);
    return retval;
}


void HaiHalCallback (UINT8 event, tHAL_NFC_STATUS status)
{
    ALOGD ("%s: enter; event=0x%X", __FUNCTION__, event);
    switch (event)
    {
    case HAL_NFC_OPEN_CPLT_EVT:
        {
            ALOGD ("%s: HAL_NFC_OPEN_CPLT_EVT; status=0x%X", __FUNCTION__, status);
            SyncEventGuard guard (gOpenCompletedEvent);
            gOpenCompletedEvent.notifyOne ();
            break;
        }

    case HAL_NFC_POST_INIT_CPLT_EVT:
        {
            ALOGD ("%s: HAL_NFC_POST_INIT_CPLT_EVT", __FUNCTION__);
            SyncEventGuard guard (gPostInitCompletedEvent);
            gPostInitCompletedEvent.notifyOne ();
            break;
        }

    case HAL_NFC_CLOSE_CPLT_EVT:
        {
            ALOGD ("%s: HAL_NFC_CLOSE_CPLT_EVT", __FUNCTION__);
            SyncEventGuard guard (gCloseCompletedEvent);
            gCloseCompletedEvent.notifyOne ();
            break;
        }

    case HAL_NFC_ERROR_EVT:
        {
            ALOGD ("%s: HAL_NFC_ERROR_EVT", __FUNCTION__);
            {
                SyncEventGuard guard (gOpenCompletedEvent);
                gOpenCompletedEvent.notifyOne ();
            }
            {
                SyncEventGuard guard (gPostInitCompletedEvent);
                gPostInitCompletedEvent.notifyOne ();
            }
            {
                SyncEventGuard guard (gCloseCompletedEvent);
                gCloseCompletedEvent.notifyOne ();
            }
            break;
        }
    default:
        ;
    }
    gAndroidHalCallback (event, status);
    ALOGD ("%s: exit; event=0x%X", __FUNCTION__, event);
}


void HaiHalDataCallback (UINT16 data_len, UINT8* p_data)
{
    ALOGD ("%s: enter; len=%u", __FUNCTION__, data_len);
    gAndroidHalDataCallback (data_len, p_data);
}

int HaiOpen (const cxd224x_dev_t* device, nfc_stack_callback_t* halCallbackFunc, nfc_stack_data_callback_t* halDataCallbackFunc)
{
    ALOGD ("%s: enter", __FUNCTION__);
    int retval = EACCES;

    gAndroidHalCallback = halCallbackFunc;
    gAndroidHalDataCallback = halDataCallbackFunc;

    SyncEventGuard guard (gOpenCompletedEvent);
    HAL_NfcOpen (HaiHalCallback, HaiHalDataCallback);
    gOpenCompletedEvent.wait ();

    retval = 0;
    ALOGD ("%s: exit %d", __FUNCTION__, retval);
    return retval;
}

int HaiClose (const cxd224x_dev_t* device)
{
    ALOGD ("%s: enter", __FUNCTION__);
    int retval = EACCES;

    SyncEventGuard guard (gCloseCompletedEvent);
    HAL_NfcClose ();
    gCloseCompletedEvent.wait ();
    retval = 0;
    ALOGD ("%s: exit %d", __FUNCTION__, retval);
    return retval;
}


int HaiCoreInitialized (const cxd224x_dev_t* device, uint8_t* coreInitResponseParams)
{
    ALOGD ("%s: enter", __FUNCTION__);
    int retval = EACCES;

    SyncEventGuard guard (gPostInitCompletedEvent);
    HAL_NfcCoreInitialized (coreInitResponseParams);
    gPostInitCompletedEvent.wait ();
    retval = 0;
    ALOGD ("%s: exit %d", __FUNCTION__, retval);
    return retval;
}


int HaiWrite (const cxd224x_dev_t* dev, uint16_t dataLen, const uint8_t* data)
{
    ALOGD ("%s: enter; len=%u", __FUNCTION__, dataLen);
    int retval = EACCES;

    HAL_NfcWrite (dataLen, const_cast<UINT8*> (data));
    retval = 0;
    ALOGD ("%s: exit %d", __FUNCTION__, retval);
    return retval;
}


int HaiPreDiscover (const cxd224x_dev_t* device)
{
    ALOGD ("%s: enter", __FUNCTION__);
    int retval = EACCES;

    retval = HAL_NfcPreDiscover () ? 1 : 0;
    ALOGD ("%s: exit %d", __FUNCTION__, retval);
    return retval;
}


int HaiControlGranted (const cxd224x_dev_t* device)
{
    ALOGD ("%s: enter", __FUNCTION__);
    int retval = EACCES;

    HAL_NfcControlGranted ();
    retval = 0;
    ALOGD ("%s: exit %d", __FUNCTION__, retval);
    return retval;
}


int HaiPowerCycle (const cxd224x_dev_t* device)
{
    ALOGD ("%s: enter", __FUNCTION__);
    int retval = EACCES;

    HAL_NfcPowerCycle ();
    retval = 0;
    ALOGD ("%s: exit %d", __FUNCTION__, retval);
    return retval;
}


int HaiGetMaxNfcee (const cxd224x_dev_t* device, uint8_t* maxNfcee)
{
    ALOGD ("%s: enter", __FUNCTION__);
    int retval = EACCES;

    if ( maxNfcee )
    {
        unsigned long num;

        // At this point we can see if there is a chip-specific value for max ee.
        if ( GetNumValue ( NAME_NFA_MAX_EE_SUPPORTED, &num, sizeof ( num ) ) )
        {
            *maxNfcee = num;
        }
        else
            *maxNfcee = HAL_NfcGetMaxNfcee ();

        ALOGD("%s: max_ee from HAL to use %d", __FUNCTION__, *maxNfcee);
        retval = 0;
    }
    ALOGD ("%s: exit %d", __FUNCTION__, retval);
    return retval;
}
