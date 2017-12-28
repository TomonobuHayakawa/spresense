/******************************************************************************
 *
 *  Copyright (C) 2013 Sony Corporation
 *  Copyright (C) 1999-2012 Broadcom Corporation
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
#define LOG_TAG "NfcNciHal"

#include "OverrideLog.h"
#include "config.h"
#include "nfc_hal_int.h"
#include "userial.h"
extern "C"
{
    #include "nfc_hal_post_reset.h"
}

#ifndef SPZ2_IMPL
//Including stdlib.h should be enough
#include <malloc.h> 
#include <string>
#endif

#include <stdlib.h>
#include <cutils/properties.h>
#include "spdhelper.h"
#include "StartupConfig.h"
#include "nfc_sony_defs.h"

#ifdef SPZ1_IMPL
#include "syscall_wrapper.h"
#endif /* SPZ1_IMPL */

#define FW_PRE_PATCH                        "FW_PRE_PATCH"
#define FW_PATCH                            "FW_PATCH"
#define MAX_RF_DATA_CREDITS                 "MAX_RF_DATA_CREDITS"
#define DBG_CMD_FILE                        "DBG_CMD_FILE"
#define DBG_RSP_FILE                        "DBG_RSP_FILE"

#define MAX_BUFFER      (512)
static char sPatchFn[MAX_BUFFER+1];
static void * sPrmBuf = NULL;

#define CONFIG_MAX_LEN 256
static UINT8 sConfig [CONFIG_MAX_LEN];

#ifndef SPZ2_IMPL
//TODO check reason of comment out
static StartupConfig sStartupConfig;
static StartupConfig sLptdConfig;
static StartupConfig sPreDiscoveryConfig;
#endif

/* defined in the HAL */
extern UINT8 *p_nfc_hal_dm_start_up_cfg;
static UINT8 nfa_dm_lmr_cfg[CONFIG_MAX_LEN];
extern UINT8 *p_nfc_hal_dm_lmr_cfg;
static UINT8 nfa_dm_start_up_rmw_cfg[CONFIG_MAX_LEN];
static UINT8 nfa_dm_start_up_vsc_cfg[CONFIG_MAX_LEN];
static UINT8 nfa_dm_closing_vsc_cfg[CONFIG_MAX_LEN];
extern UINT8 *p_nfc_hal_dm_start_up_rmw_cfg;
extern UINT8 *p_nfc_hal_dm_start_up_vsc_cfg;
extern UINT8 *p_nfc_hal_dm_closing_vsc_cfg;
extern UINT8 *p_nfc_hal_dm_lptd_cfg;
extern UINT8 *p_nfc_hal_dm_start_up_debug_cfg;
extern FILE  *p_nfc_hal_dm_start_up_debug_outfd;
extern UINT8 *p_nfc_hal_pre_discover_cfg;

extern tSNOOZE_MODE_CONFIG gSnoozeModeCfg;
extern tNFC_HAL_CFG *p_nfc_hal_cfg;

/* Default patchfile (in NCD format) */
#ifndef NFA_APP_DEFAULT_PATCHFILE_NAME
#define NFA_APP_DEFAULT_PATCHFILE_NAME      "\0"
#endif

/*******************************************************************************
**
** Function         getFileLength
**
** Description      return the size of a file
**
** Returns          file size in number of bytes
**
*******************************************************************************/
static long getFileLength(FILE* fp)
{
    long sz;
    fseek(fp, 0L, SEEK_END);
    sz = ftell(fp);
    fseek(fp, 0L, SEEK_SET);

    return (sz > 0) ? sz : 0;
}

/*******************************************************************************
**
** Function         isFileExist
**
** Description      Check if file name exists (android does not support fexists)
**
** Returns          TRUE if file exists
**
*******************************************************************************/
static BOOLEAN isFileExist(const char *pFilename)
{
    FILE *pf;

    if ((pf = fopen(pFilename, "r")) != NULL)
    {
        fclose(pf);
        return TRUE;
    }
    return FALSE;
}

/*******************************************************************************
**
** Function         findPatchramFile
**
** Description      Find the patchram file name specified in the .conf
**
** Returns          pointer to the file name
**
*******************************************************************************/
static const char* findPatchramFile(const char * pConfigName, char * pBuffer, int bufferLen)
{
    ALOGD("%s: config=%s", __FUNCTION__, pConfigName);

    if (pConfigName == NULL)
    {
        ALOGD("%s No patchfile defined\n", __FUNCTION__);
        return NULL;
    }

    if (GetStrValue(pConfigName, &pBuffer[0], bufferLen))
    {
        ALOGD("%s found patchfile %s\n", __FUNCTION__, pBuffer);
        return (pBuffer[0] == '\0') ? NULL : pBuffer;
    }

    ALOGD("%s Cannot find patchfile '%s'\n", __FUNCTION__, pConfigName);
    return NULL;
}

/*******************************************************************************
**
** Function:    continueAfterSetSnoozeMode
**
** Description: Called after Snooze Mode is enabled.
**
** Returns:     none
**
*******************************************************************************/
static void continueAfterSetSnoozeMode(tHAL_NFC_STATUS status)
{
    ALOGD("%s: status=%u", __FUNCTION__, status);
    if (status == NCI_STATUS_OK)
        HAL_NfcPreInitDone (HAL_NFC_STATUS_OK);
    else
        HAL_NfcPreInitDone (HAL_NFC_STATUS_FAILED);
}

/*******************************************************************************
**
** Function:    postDownloadPatchram
**
** Description: Called after patch download
**
** Returns:     none
**
*******************************************************************************/
static void postDownloadPatchram(tHAL_NFC_STATUS status)
{
        ALOGD("%s: Not using Snooze Mode", __FUNCTION__);
        HAL_NfcPreInitDone(HAL_NFC_STATUS_OK);

#if 0  //@@@
    ALOGD("%s: status=%i", __FUNCTION__, status);
    if (sPrmBuf)
    {
        free(sPrmBuf);
        sPrmBuf = NULL;
    }
    int glen = GetStrValue (NAME_SNOOZE_MODE_CFG, (char*)&gSnoozeModeCfg, sizeof(gSnoozeModeCfg));
#ifdef SPZ_IMPL
    if(glen == 0) {
        glen = 5;
        gSnoozeModeCfg.snooze_mode          = 0x08;
        gSnoozeModeCfg.idle_threshold_dh    = 0x00;
        gSnoozeModeCfg.idle_threshold_nfcc  = 0x00;
        gSnoozeModeCfg.nfc_wake_active_mode = 0x00;
        gSnoozeModeCfg.dh_wake_active_mode  = 0x01;
    }
#endif
    if (status != HAL_NFC_STATUS_OK)
    {
        ALOGE("%s: Patch download failed", __FUNCTION__);
        if (status == HAL_NFC_STATUS_REFUSED)
        {
            SpdHelper::setPatchAsBad();
        }
        else
            SpdHelper::incErrorCount();

        /* If in SPD Debug mode, fail immediately and obviously */
        if (SpdHelper::isSpdDebug())
            HAL_NfcPreInitDone (HAL_NFC_STATUS_FAILED);
        else
        {
            /* otherwise, power cycle the chip and let the stack startup normally */
            ALOGD("%s: re-init; don't download firmware", __FUNCTION__);
            USERIAL_PowerupDevice(0);
            HAL_NfcReInit ();
        }
    }
    /* Set snooze mode here */
    else if (glen && gSnoozeModeCfg.snooze_mode != NFC_HAL_LP_SNOOZE_MODE_NONE)
    {
        status = HAL_NfcSetSnoozeMode(gSnoozeModeCfg.snooze_mode,
                                       gSnoozeModeCfg.idle_threshold_dh,
                                       gSnoozeModeCfg.idle_threshold_nfcc,
                                       gSnoozeModeCfg.nfc_wake_active_mode,
                                       gSnoozeModeCfg.dh_wake_active_mode,
                                       continueAfterSetSnoozeMode);
        if (status != NCI_STATUS_OK)
        {
            ALOGE("%s: Setting snooze mode failed, status=%i", __FUNCTION__, status);
            HAL_NfcPreInitDone(HAL_NFC_STATUS_FAILED);
        }
    }
    else
    {
        ALOGD("%s: Not using Snooze Mode", __FUNCTION__);
        HAL_NfcPreInitDone(HAL_NFC_STATUS_OK);
    }
#endif //@@@
}


/*******************************************************************************
**
** Function:    prmCallback
**
** Description: Patchram callback (for static patchram mode)
**
** Returns:     none
**
*******************************************************************************/
void prmCallback(UINT8 event)
{
    ALOGD("%s: event=0x%x", __FUNCTION__, event);
    switch (event)
    {
    case NFC_HAL_PRM_CONTINUE_EVT:
        /* This event does not occur if static patchram buf is used */
        break;

    case NFC_HAL_PRM_COMPLETE_EVT:
        postDownloadPatchram(HAL_NFC_STATUS_OK);
        break;

    case NFC_HAL_PRM_ABORT_EVT:
        postDownloadPatchram(HAL_NFC_STATUS_FAILED);
        break;

    case NFC_HAL_PRM_ABORT_INVALID_PATCH_EVT:
        ALOGD("%s: invalid patch...skipping patch download", __FUNCTION__);
        postDownloadPatchram(HAL_NFC_STATUS_REFUSED);
        break;

    case NFC_HAL_PRM_ABORT_BAD_SIGNATURE_EVT:
        ALOGD("%s: patch authentication failed", __FUNCTION__);
        postDownloadPatchram(HAL_NFC_STATUS_REFUSED);
        break;

    case NFC_HAL_PRM_ABORT_NO_NVM_EVT:
        ALOGD("%s: No NVM detected", __FUNCTION__);
        HAL_NfcPreInitDone(HAL_NFC_STATUS_FAILED);
        break;

    default:
        ALOGD("%s: not handled event=0x%x", __FUNCTION__, event);
        break;
    }
}

/*******************************************************************************
**
** Function         getNfaValues
**
** Description      Get configuration values needed by NFA layer
**
** Returns:         None
**
*******************************************************************************/
static void getNfaValues()
{
    int actualLen = 0;


        UINT8 l_sConfig[] = {0x09,0xD1,0x01,0x19,0xD2,0x01,0xff,0xD3,0x01,0x00};
        actualLen = sizeof(l_sConfig);
        memcpy(sConfig, l_sConfig, actualLen);
        ALOGD ( "START_UP_CFG[0] = %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
                sConfig[0],
                sConfig[1],
                sConfig[2],
                sConfig[3],
                sConfig[4],
                sConfig[5],
                sConfig[6],
                sConfig[7] );
p_nfc_hal_dm_start_up_cfg = &sConfig[0];

        UINT8 l_nfa_dm_lmr_cfg[] = {0x19,0x00,0x03,0x00,0x01,0x00,0x00,0x03,0x00,0x01,0x01,0x00,0x03,0x00,0x01,0x02,0x01,0x03,0x00,0x01,0x04,0x01,0x03,0x00,0x01,0x05};
        actualLen = sizeof(l_nfa_dm_lmr_cfg);
        memcpy(nfa_dm_lmr_cfg, l_nfa_dm_lmr_cfg, actualLen);
        ALOGD ( "LISTEN_MODE_ROUTING_CFG[0] = %02x:%02x:%02x:%02x:%02x:%02x...\n",
                nfa_dm_lmr_cfg[0],
                nfa_dm_lmr_cfg[1],
                nfa_dm_lmr_cfg[2],
                nfa_dm_lmr_cfg[3],
                nfa_dm_lmr_cfg[4],
                nfa_dm_lmr_cfg[5]);


        UINT8 l_nfa_dm_start_up_vsc_cfg[] = {0x0b,0x2f,0x34,0x08,0xf8,0x74,0x00,0x01,0x00,0x02,0x00,0x00};
        actualLen = sizeof(l_nfa_dm_start_up_vsc_cfg);
        memcpy(nfa_dm_start_up_vsc_cfg, l_nfa_dm_start_up_vsc_cfg, actualLen);
        p_nfc_hal_dm_start_up_vsc_cfg = &nfa_dm_start_up_vsc_cfg[0];
        ALOGD ( "START_UP_VSC_CFG[0] = %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x...\n",
                nfa_dm_start_up_vsc_cfg[0],
                nfa_dm_start_up_vsc_cfg[1],
                nfa_dm_start_up_vsc_cfg[2],
                nfa_dm_start_up_vsc_cfg[3],
                nfa_dm_start_up_vsc_cfg[4],
                nfa_dm_start_up_vsc_cfg[5],
                nfa_dm_start_up_vsc_cfg[6],
                nfa_dm_start_up_vsc_cfg[7] );



}

/*******************************************************************************
 **
 ** Function         StartPatchDownload
 **
 ** Description      Reads configuration settings, and begins the download
 **                  process if patch files are configured.
 **
 ** Returns:         None
 **
 *******************************************************************************/
static void StartPatchDownload(UINT32 chipid)
{
    getNfaValues();                 // Get NFA configuration values into variables

            ALOGE("%s: No patchfile specified or disabled. Proceeding to post-download procedure...", __FUNCTION__);
            postDownloadPatchram(HAL_NFC_STATUS_OK);

#ifndef SPZ2_IMPL //TODO enable patch function    
    char chipID[30];
    snprintf(chipID, sizeof(chipID), "224x%08lx", chipid);
    ALOGD ("%s: chidId=%s", __FUNCTION__, chipID);

    readOptionalConfig(chipID);     // Read optional chip specific settings
#if (NFCNCI_DEBUGFUNC == TRUE)
    readOptionalDebugConfig("debug");    // Read optional DEBUG specific settings
#endif
    getNfaValues();                 // Get NFA configuration values into variables

#if (NFCNCI_DEBUGFUNC == TRUE)
    if( GetStrValue(DBG_CMD_FILE, sPatchFn, sizeof(sPatchFn)) )
    {
        FILE *fd;

        /* If DBG_CMD_FILE is specified, then download it now */
        if (sPatchFn[0] != '\0')
        {
            /* open DBG_CMD_FILE, read it into a buffer */
            if ((fd = fopen(sPatchFn, "rb")) != NULL)
            {
                UINT32 lenPrmBuffer = getFileLength(fd);

                if( p_nfc_hal_dm_start_up_debug_cfg )
                {
                    ALOGW("%s already allocated p_nfc_hal_dm_start_up_debug_cfg", __FUNCTION__);
                }
                if ( lenPrmBuffer <= NFC_HAL_DBGCMD_MAX_DOWNLOADFILE_SZ
                     && (p_nfc_hal_dm_start_up_debug_cfg = (UINT8 *)malloc(lenPrmBuffer)) != NULL)
                {
                    size_t rlen;
                    ALOGD("%s Downloading DBG_CMD_FILE %s (size: %lu)", __FUNCTION__, sPatchFn, lenPrmBuffer);
                    rlen = fread(p_nfc_hal_dm_start_up_debug_cfg, lenPrmBuffer, 1, fd);
                    UINT32 len_cfg;
                    len_cfg = (UINT32 )((UINT16 )(p_nfc_hal_dm_start_up_debug_cfg[0] | (p_nfc_hal_dm_start_up_debug_cfg[1]<<8))) ;
                    if (rlen == 0 || lenPrmBuffer != len_cfg + 2)
                    {
                        ALOGD("%s illegal format DBG_CMD_FILE", __FUNCTION__);
                        free(p_nfc_hal_dm_start_up_debug_cfg);
                        p_nfc_hal_dm_start_up_debug_cfg = NULL;
                    }
                }
                else
                {
                    ALOGE("%s Downloading DBG_CMD_FILE %s is too big(size: %lu > %d) or malloc fail",
                          __FUNCTION__, sPatchFn, lenPrmBuffer, NFC_HAL_DBGCMD_MAX_DOWNLOADFILE_SZ);
                }
                fclose(fd);

                /* if read done and DBG_RSP_FILE is specified, open it for writing rsp packets */
                if( p_nfc_hal_dm_start_up_debug_cfg ){
                    p_nfc_hal_dm_start_up_debug_outfd = NULL;
                    if( GetStrValue(DBG_RSP_FILE, sPatchFn, sizeof(sPatchFn) ))
                    {
                        p_nfc_hal_dm_start_up_debug_outfd = NULL;
                        if ((sPatchFn[0] != '\0' && (fd = fopen(sPatchFn, "wb")) != NULL))
                        {
                            p_nfc_hal_dm_start_up_debug_outfd = fd;
                        }
                        else
                        {
                            ALOGE("%s Unable to open DBG_RSP_FILE %s", __FUNCTION__, sPatchFn);
                        }
                    }
                }
            }
            else
            {
                ALOGE("%s Unable to open DBG_CMD_FILE %s", __FUNCTION__, sPatchFn);
            }
        }
    }
    else
    {
        ALOGD ("%s: %s is not specified", __FUNCTION__, DBG_CMD_FILE);
    }
#endif

    findPatchramFile(FW_PATCH, sPatchFn, sizeof(sPatchFn));
    {
        FILE *fd;
        sPrmBuf = NULL;
        /* If a patch file was specified, then download it now */
        if (sPatchFn[0] != '\0')
        {
            UINT32 bDownloadStarted = false;

            /* open patchfile, read it into a buffer */
            if ((fd = fopen(sPatchFn, "rb")) != NULL)
            {
                UINT32 lenPrmBuffer = getFileLength(fd);
                ALOGD("%s Downloading patchfile %s (size: %lu)", __FUNCTION__, sPatchFn, lenPrmBuffer);
                if ( lenPrmBuffer <= NFC_HAL_PRM_MAX_DOWNLOADFILE_SZ && (sPrmBuf = malloc(lenPrmBuffer)) != NULL)
                {
                    size_t actualLen = fread(sPrmBuf, 1, lenPrmBuffer, fd);
                    if (actualLen == lenPrmBuffer)
                    {
                        if (!SpdHelper::isPatchBad((UINT8*)sPrmBuf, lenPrmBuffer))
                        {
                            /* Download patch using static memeory mode */
                            HAL_NfcPrmDownloadStart(NFC_HAL_PRM_FORMAT_NCD, 0, (UINT8*)sPrmBuf, lenPrmBuffer, 0, prmCallback);
                            bDownloadStarted = true;
                        }
                    }
                    else
                        ALOGE("%s fail reading patchram", __FUNCTION__);
                }
                else
                    ALOGE("%s Unable to buffer to hold patchram (%lu bytes)", __FUNCTION__, lenPrmBuffer);

                fclose(fd);
            }
            else
                ALOGE("%s Unable to open patchfile %s", __FUNCTION__, sPatchFn);

            /* If the download never got started */
            if (!bDownloadStarted)
            {
                /* If debug mode, fail in an obvious way, otherwise try to start stack */
                postDownloadPatchram(SpdHelper::isSpdDebug() ? HAL_NFC_STATUS_FAILED :
                                     HAL_NFC_STATUS_OK);
            }
        }
        else
        {
            ALOGE("%s: No patchfile specified or disabled. Proceeding to post-download procedure...", __FUNCTION__);
            postDownloadPatchram(HAL_NFC_STATUS_OK);
        }
    }

    ALOGD ("%s: exit", __FUNCTION__);
#endif //SPZ2_IMPL   
}

/*******************************************************************************
 **
 ** Function:    nfc_hal_post_reset_init
 **
 ** Description: Called by the NFC HAL after controller has been reset.
 **              Begin to download firmware patch files.
 **
 ** Returns:     none
 **
 *******************************************************************************/
void nfc_hal_post_reset_init (UINT32 hw_id, UINT8 nvm_type)
{
    ALOGD("%s: fw_id=0x%lx, nvm_type=%d", __FUNCTION__, hw_id, nvm_type);
    UINT8 max_credits = 1;

    if (nvm_type != NCI_SPD_NVM_TYPE_FLASH)
    {
        if( !hw_id )
        {
            /* only read config */
            getNfaValues();      // Get NFA configuration values into variables
        }
        else
        {
            ALOGD("%s: No NVM detected, FAIL the init stage to force a retry", __FUNCTION__);
            USERIAL_PowerupDevice (0);
            HAL_NfcReInit ();
        }
    }
    else
    {
        /* Start downloading the patch files */
        StartPatchDownload(hw_id);

        if (GetNumValue(MAX_RF_DATA_CREDITS, &max_credits, sizeof(max_credits)) && (max_credits > 0))
        {
            ALOGD("%s : max_credits=%d", __FUNCTION__, max_credits);
            HAL_NfcSetMaxRfDataCredits(max_credits);
        }
    }
}
