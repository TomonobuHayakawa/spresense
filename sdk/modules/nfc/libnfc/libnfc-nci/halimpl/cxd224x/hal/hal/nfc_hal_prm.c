/******************************************************************************
 *
 *  Copyright (C) 2012-2014 Broadcom Corporation
 *  Copyright (C) 2014 Sony Corporation
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

#include <string.h>
#include "nfc_hal_int_extra.h"
#include "nfc_sony_defs.h"
#include "userial.h"

/*****************************************************************************
* Definitions
*****************************************************************************/

/* Internal flags */
#define NFC_HAL_PRM_FLAGS_USE_PATCHRAM_BUF  0x01    /* Application provided patchram in a single buffer */
#define NFC_HAL_PRM_FLAGS_RFU               0x02    /* Reserved for future use */

/* Secure patch download definitions */
#define NFC_HAL_PRM_NCD_PATCHFILE_HDR_LEN    64      /* PRJID + HWVer + FWVer + PatchVer + COUNT */
#define NFC_HAL_PRM_NCD_PATCHFILE_DW_UNIT_SZ 240
#define NFC_HAL_PRM_NCD_PATCHFILE_MAC_SZ     16

#define NFC_HAL_W4_REST_NTF_TOUT_DEF         (3000)  /*Timeout: wait for CORE_RESET_NTF after PATCH_END_CMD */
#define NFC_HAL_W4_REST_NTF_TOUT                                        \
    (((NFC_HAL_W4_REST_NTF_TOUT_DEF) > (NFC_HAL_CMD_TOUT)) ? (NFC_HAL_W4_REST_NTF_TOUT_DEF) : (NFC_HAL_CMD_TOUT))

#ifndef NFC_HAL_W4_NVM_UPDATE
#define NFC_HAL_W4_NVM_UPDATE                (900)  /* restart device after Nvm Erase and Update */
#endif

#if (NFC_HAL_PRM_DEBUG == TRUE)
#define NFC_HAL_PRM_STATE(str)  HAL_TRACE_DEBUG2 ("%s st: %d", str, nfc_hal_cb2.prm.state)
#else
#define NFC_HAL_PRM_STATE(str)
#endif

void nfc_hal_prm_post_baud_update (tHAL_NFC_STATUS status);

/*****************************************************************************
** Extern variable from nfc_hal_dm_cfg.c
*****************************************************************************/
extern tNFC_HAL_CFG *p_nfc_hal_cfg;

static const UINT8 nfc_hal_dm_patch_version_cmd [NCI_MSG_HDR_SIZE] =
{
    NCI_MTS_CMD|NCI_GID_PROP,
    NCI_MSG_OPTION_PATCH_VERSION,
    0x00
};

#define NCI_MSG_OPTION_PATCH_START_LEN 0x5
#define NCI_MSG_OPTION_PATCH_START_PATCH_TYPE_NVM 0x0
static const UINT8 nfc_hal_dm_patch_start_cmd [NCI_MSG_HDR_SIZE] =
{
    NCI_MTS_CMD|NCI_GID_PROP,
    NCI_MSG_OPTION_PATCH_START,
    NCI_MSG_OPTION_PATCH_START_LEN
};
static const UINT8 nfc_hal_dm_patch_header_cmd [NCI_MSG_HDR_SIZE] =
{
    NCI_MTS_CMD|NCI_GID_PROP,
    NCI_MSG_OPTION_PATCH_HEADER,
    0x00
};
static const UINT8 nfc_hal_dm_patch_data_cmd [NCI_MSG_HDR_SIZE] =
{
    NCI_MTS_CMD|NCI_GID_PROP,
    NCI_MSG_OPTION_PATCH_DATA,
    0x00
};
static const UINT8 nfc_hal_dm_patch_end_cmd [NCI_MSG_HDR_SIZE] =
{
    NCI_MTS_CMD|NCI_GID_PROP,
    NCI_MSG_OPTION_PATCH_END,
    0x00
};

/*******************************************************************************
**
** Function         nfc_hal_prm_spd_handle_download_complete
**
** Description      Patch download complete (for secure patch download)
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_prm_spd_handle_download_complete (UINT8 event)
{
    nfc_hal_cb2.prm.state = NFC_HAL_PRM_ST2_IDLE;

    /* Notify application now */
    if (nfc_hal_cb2.prm.p_cback)
        (nfc_hal_cb2.prm.p_cback) (event);
}

/*******************************************************************************
**
** Function         nfc_hal_prm_spd_send_next_segment
**
** Description      Send next patch segment (for secure patch download)
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_prm_spd_send_next_segment (void)
{
    UINT8   buf_cmd[NFC_HAL_PRM_NCD_PATCHFILE_DW_UNIT_SZ+NCI_MSG_HDR_SIZE]; /* max size */
    UINT8   *p;
    const UINT8   *p_src;
    UINT16  offset = nfc_hal_cb2.prm.cur_patch_offset;
    UINT8   len;
    const UINT8 *msg_hdr;

    HAL_TRACE_DEBUG1 ("nfc_hal_prm_spd_send_next_segment() offset=%i", nfc_hal_cb2.prm.cur_patch_offset);
    p = buf_cmd;
    p_src = (const UINT8 *)(nfc_hal_cb2.prm.p_cur_patch_data + offset);

    /* Validate that segment should have 1byte */
    if (nfc_hal_cb2.prm.cur_patch_len_remaining < NFC_HAL_PRM_NCD_PATCHFILE_MAC_SZ)
    {
        HAL_TRACE_ERROR2 ("Unexpected end of patch. remain(%dB) < size(%dB)",
                          nfc_hal_cb2.prm.cur_patch_len_remaining, NFC_HAL_PRM_NCD_PATCHFILE_MAC_SZ );
        nfc_hal_prm_spd_handle_download_complete (NFC_HAL_PRM_ABORT_INVALID_PATCH_EVT);
        return;
    }

    if( nfc_hal_cb2.prm.cur_patch_offset < NFC_HAL_PRM_NCD_PATCHFILE_HDR_LEN )
    {
        msg_hdr = nfc_hal_dm_patch_header_cmd;
        len = (UINT8 )NFC_HAL_PRM_NCD_PATCHFILE_HDR_LEN;
        nfc_hal_cb2.prm.state = NFC_HAL_PRM_ST2_SPD_W4_HEADER_RSP;
    }
    else if( nfc_hal_cb2.prm.cur_patch_len_remaining <= NFC_HAL_PRM_NCD_PATCHFILE_MAC_SZ )
    {
        /* cur_patch_len_remaining must equ to NFC_HAL_PRM_NCD_PATCHFILE_MAC_SZ,
           other cases, nci_cmd will be recv error rsp */
        msg_hdr = nfc_hal_dm_patch_end_cmd;
        len = (UINT8 )nfc_hal_cb2.prm.cur_patch_len_remaining;
        nfc_hal_cb2.prm.state = NFC_HAL_PRM_ST2_SPD_AUTHENTICATING;
        HAL_TRACE_DEBUG1 ("Patch downloaded and authenticated. Waiting %i ms for RESET NTF...", NFC_HAL_W4_REST_NTF_TOUT);
        nfc_hal_cb2.prm.timer.p_cback = nfc_hal_prm_process_timeout;
        nfc_hal_main_start_quick_timer (&nfc_hal_cb2.prm.timer, (UINT16)(NFC_HAL_TTYPE_NCI_WAIT_RSP),
                                        ((UINT32) NFC_HAL_W4_REST_NTF_TOUT) * QUICK_TIMER_TICKS_PER_SEC / 1000);
    }
    else
    {
        UINT32 remain_sz = nfc_hal_cb2.prm.cur_patch_len_remaining - NFC_HAL_PRM_NCD_PATCHFILE_MAC_SZ;
        len = remain_sz > NFC_HAL_PRM_NCD_PATCHFILE_DW_UNIT_SZ
            ? (UINT8 )NFC_HAL_PRM_NCD_PATCHFILE_DW_UNIT_SZ : (UINT8 )remain_sz;
        msg_hdr = nfc_hal_dm_patch_data_cmd;
        nfc_hal_cb2.prm.state = NFC_HAL_PRM_ST2_SPD_DOWNLOADING;
    }

    /* build command header in front of body */
    memcpy( p, msg_hdr,  NCI_MSG_HDR_SIZE -1  );
    p += NCI_MSG_HDR_SIZE -1;
    UINT8_TO_STREAM ( p, len );
    ARRAY_TO_STREAM ( p, p_src, len );

    /* Update number of bytes comsumed */
    nfc_hal_cb2.prm.cur_patch_offset += len;
    nfc_hal_cb2.prm.cur_patch_len_remaining -= len ;

    /* Send the command  */
    nfc_hal_dm_send_nci_cmd ( buf_cmd, (UINT8) (len + NCI_MSG_HDR_SIZE),
                              nfc_hal_prm_nci_command_complete_cback);
}

/*******************************************************************************
**
** Function         nfc_hal_prm_spd_handle_next_patch_start
**
** Description      Handle start of next patch (for send patch_start_cmd)
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_prm_spd_handle_next_patch_start (void)
{
    UINT8   *p;
    UINT8   p_cmd[NCI_MSG_HDR_SIZE + NCI_MSG_OPTION_PATCH_START_LEN];

    /* Validate that patch data should have > NFC_HAL_PRM_NCD_PATCHFILE_HDR_SZ */
    if (nfc_hal_cb2.prm.cur_patch_len_remaining < NFC_HAL_PRM_NCD_PATCHFILE_HDR_LEN)
    {
        HAL_TRACE_ERROR0 ("Too small patch size.");
        nfc_hal_prm_spd_handle_download_complete (NFC_HAL_PRM_ABORT_INVALID_PATCH_EVT);
        return;
    }
    p = p_cmd;
    memcpy( p, nfc_hal_dm_patch_start_cmd, NCI_MSG_HDR_SIZE);
    p += NCI_MSG_HDR_SIZE;
    UINT8_TO_STREAM ( p, NCI_MSG_OPTION_PATCH_START_PATCH_TYPE_NVM );
    UINT8_TO_STREAM ( p, nfc_hal_cb2.prm.cur_patch_len_remaining );

    nfc_hal_cb2.prm.state = NFC_HAL_PRM_ST2_SPD_W4_START_RSP;

    /* Send the command (not including HCIT here) */
    nfc_hal_dm_send_nci_cmd (p_cmd, (UINT8) (NCI_MSG_HDR_SIZE + NCI_MSG_OPTION_PATCH_START_LEN),
                             nfc_hal_prm_nci_command_complete_cback);
}

/*******************************************************************************
**
** Function         nfc_hal_prm_spd_check_version
**
** Description      Check patchfile version with current downloaded version
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_prm_spd_check_version (void)
{
    UINT8 *p, *p_start;
    UINT32 patchfile_patch_present_mask=0;
    UINT32 patchfile_project_id = 0;
    UINT32 patchfile_hw_ver = 0;
    UINT32 patchfile_fw_ver = 0;
    UINT16 patchfile_ver_major = 0;
    UINT16 patchfile_ver_minor = 0;

    UINT8  return_code = NFC_HAL_PRM_COMPLETE_EVT;

    /* Initialize patchfile offset pointers */
    p = p_start = NULL;

    /* Get patchfile version */
    if (nfc_hal_cb2.prm.cur_patch_len_remaining >= NFC_HAL_PRM_NCD_PATCHFILE_HDR_LEN)
    {
        /* Parse patchfile header */
        p       = (UINT8 *) nfc_hal_cb2.prm.p_cur_patch_data;
        p_start = p;
        BE_STREAM_TO_UINT32 (patchfile_project_id, p);
        BE_STREAM_TO_UINT32 (patchfile_hw_ver, p);
        BE_STREAM_TO_UINT32 (patchfile_fw_ver, p);
        BE_STREAM_TO_UINT16 (patchfile_ver_major, p);
        BE_STREAM_TO_UINT16 (patchfile_ver_minor, p);

        /* DATE */
        p+=8;

        /* Data len */
        p+=4;

        /* RFU */
        p++;

        /* Check how many patches are in the patch file */
        STREAM_TO_UINT8 (nfc_hal_cb2.prm.spd_patch_count, p);
        if (!nfc_hal_cb2.prm.spd_patch_count)
        {
            HAL_TRACE_ERROR1 ("Unsupported patchfile (number of patches (%i))",
                               nfc_hal_cb2.prm.spd_patch_count);
        }
        else
        {
            patchfile_patch_present_mask = 1;
        }

        HAL_TRACE_DEBUG5 ("Patchfile info: ProjID=0x%08x, ver_major=0x%04x ver_minor=0x%04x, Num_patches=%i, PatchSize=%i",
                          patchfile_project_id, patchfile_ver_major, patchfile_ver_minor,
                          nfc_hal_cb2.prm.spd_patch_count, nfc_hal_cb2.prm.cur_patch_len_remaining);

        /*********************************************************************
        * Version check of patchfile against NVM
        *********************************************************************/

        /* Check HW/FW version */
        if ((NFC_HAL_PRM_HW_VERSION_MASK(nfc_hal_cb2.nvm_cb.ver_hw) != NFC_HAL_PRM_HW_VERSION_MASK(patchfile_hw_ver)))
        {
            HAL_TRACE_DEBUG2 ("Unsupported patchfile( HW version missmach ) hw_ver(masked)=0x%08x file_ver(masked)=0x%08x",
                              NFC_HAL_PRM_HW_VERSION_MASK(nfc_hal_cb2.nvm_cb.ver_hw),
                              NFC_HAL_PRM_HW_VERSION_MASK(patchfile_hw_ver));
            return_code = NFC_HAL_PRM_ABORT_INVALID_PATCH_EVT;
        }
        else if ((NFC_HAL_PRM_FW_VERSION_MASK(nfc_hal_cb2.nvm_cb.ver_fw) != NFC_HAL_PRM_FW_VERSION_MASK(patchfile_fw_ver)))
        {
            HAL_TRACE_DEBUG2 ("Unsupported patchfile( FW version missmach ) fw_ver(masked)=0x%08x file_ver(masked)=0x%08x",
                              NFC_HAL_PRM_FW_VERSION_MASK(nfc_hal_cb2.nvm_cb.ver_fw),
                              NFC_HAL_PRM_FW_VERSION_MASK(patchfile_fw_ver));
            return_code = NFC_HAL_PRM_ABORT_INVALID_PATCH_EVT;
        }
        /* Download the patchfile if no patches in NVM */
        else if ((nfc_hal_cb2.nvm_cb.project_id == 0) || !(nfc_hal_cb2.nvm_cb.flags & NFC_HAL_NVM_FLAGS_PATCH_PRESENT))
        {
            /* No patch in NVM, need to download all */
            nfc_hal_cb2.prm.spd_patch_needed_mask = patchfile_patch_present_mask;

            HAL_TRACE_DEBUG2 ("No previous patch detected. Downloading patch %04x.%04x",
                              patchfile_ver_major, patchfile_ver_minor);
        }
        /* Skip download if project ID of patchfile does not match NVM */
        else if (nfc_hal_cb2.nvm_cb.project_id != patchfile_project_id)
        {
            /* Project IDs mismatch */
            HAL_TRACE_DEBUG2 ("Patch download skipped: Mismatched Project ID (NVM ProjId: 0x%04x, Patchfile ProjId: 0x%04x)",
                              nfc_hal_cb2.nvm_cb.project_id, patchfile_project_id);

            return_code = NFC_HAL_PRM_ABORT_INVALID_PATCH_EVT;
        }
        /* Skip download if version of patchfile is equal to version in NVM */
        /*      f/w may check the major version, but this code dont care */
        else if ( (nfc_hal_cb2.nvm_cb.ver_major == patchfile_ver_major)
                &&(nfc_hal_cb2.nvm_cb.ver_minor == patchfile_ver_minor))
        {
            /* NVM version is newer than patchfile */
            HAL_TRACE_DEBUG2 ("Patch download skipped. NVM patch (version %x.%04x) is the same than the patchfile ",
                              nfc_hal_cb2.nvm_cb.ver_major, nfc_hal_cb2.nvm_cb.ver_minor);

            return_code = NFC_HAL_PRM_COMPLETE_EVT;
        }
        /* Remaining cases: Download all patches in the patchfile */
        else
        {
            nfc_hal_cb2.prm.spd_patch_needed_mask = patchfile_patch_present_mask;
        }

    }
    else
    {
        /* Invalid patch file header */
        HAL_TRACE_ERROR0 ("Invalid patch file header.");

        return_code = NFC_HAL_PRM_ABORT_INVALID_PATCH_EVT;
    }

    /* If we need to download anything, get the first patch to download */
    if (nfc_hal_cb2.prm.spd_patch_needed_mask)
    {
        HAL_TRACE_ERROR4 ("Downloading patch version: %x.%04x (previous version in NVM: %x.%04x)...",
                          patchfile_ver_major, patchfile_ver_minor,
                          nfc_hal_cb2.nvm_cb.ver_major, nfc_hal_cb2.nvm_cb.ver_minor);
        /* Download first segment */
        nfc_hal_cb2.prm.state = NFC_HAL_PRM_ST2_SPD_GET_PATCH_HEADER;
        if (!(nfc_hal_cb2.prm.flags & NFC_HAL_PRM_FLAGS_USE_PATCHRAM_BUF))
        {
            /* Notify adaptation layer to call HAL_NfcPrmDownloadContinue with the next patch segment */
            (nfc_hal_cb2.prm.p_cback) (NFC_HAL_PRM_SPD_GET_NEXT_PATCH);
        }
        else
        {
            nfc_hal_prm_spd_handle_next_patch_start ();
        }
    }
    else
    {
        static BOOLEAN firstTime = TRUE;
        if (firstTime)
        {
            HAL_TRACE_ERROR2 ("NVM patch version is %x.%04x",
                              nfc_hal_cb2.nvm_cb.ver_major, nfc_hal_cb2.nvm_cb.ver_minor);
            firstTime = FALSE;
        }
        /* Download complete */
        nfc_hal_prm_spd_handle_download_complete (return_code);
    }
}

#if (NFC_HAL_TRACE_VERBOSE == TRUE)
/*******************************************************************************
**
** Function         nfc_hal_prm_spd_status_str
**
** Description      Return status string for a given spd status code
**
** Returns          Status string
**
*******************************************************************************/
const UINT8 *nfc_hal_prm_spd_status_str (UINT8 spd_status_code)
{
    const char *p_str;

    switch (spd_status_code)
    {

    case NCI_STATUS_SPD_ERROR2_INVALID_PARAM:
        p_str = "SPD_ERROR_INVALID_PARAM";
        break;

    case NCI_STATUS_SPD_ERROR2_MSG_LEN:
        p_str = "SPD_ERROR_MSG_LEN";
        break;

    case NCI_STATUS_SPD_ERROR2_STATE:
        p_str = "SPD_ERROR_STATE";
        break;

    case NCI_STATUS_SPD_ERROR2_NVM_CORRUPTED:
        p_str = "SPD_ERROR_NVM_CORRUPTED";
        break;

    case NCI_STATUS_SPD_ERROR2_NVM_CORRUPTED_ALL:
        p_str = "SPD_ERROR_NVM_CORRUPTED_ALL";
        break;

   case NCI_STATUS_SPD_ERROR2_INVALID_SIG:
        p_str = "SPD_ERROR_INVALID_SIG";
        break;

    default:
        p_str = "Unspecified Error";
        break;

    }

    return ((UINT8*) p_str);
}
#endif  /* (NFC_HAL_TRACE_VERBOSE == TRUE) */

/*******************************************************************************
**
** Function         nfc_hal_prm_nci_command_complete_cback
**
** Description      Callback for NCI vendor specific command complete
**                  (for secure patch download)
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_prm_nci_command_complete_cback (tNFC_HAL_NCI_EVT event, UINT16 data_len, UINT8 *p_data)
{
    UINT8 status;
    UINT8 len;
    UINT8 *p;

    NFC_HAL_PRM_STATE ("nfc_hal_prm_nci_command_complete_cback");

    /* Stop the command-timeout timer */
    nfc_hal_main_stop_quick_timer (&nfc_hal_cb2.prm.timer);

    /* Skip over NCI header */
    len   = *(p_data + 2);
    p = p_data + NCI_MSG_HDR_SIZE;

    /* Handle OPTION_PATCH_xxx Rsp */
    if (event == NFC_VS_OPTION_PATCH_START || event == NFC_VS_OPTION_PATCH_HEADER || event == NFC_VS_OPTION_PATCH_DATA)
    {
        HAL_TRACE_DEBUG1 ("nfc_hal_prm_nci_command_complete_cback() even=0x%x", event);

        /* Status and error code */
        STREAM_TO_UINT8 (status, p);

        if (status != NCI_STATUS_OK)
        {
            HAL_TRACE_ERROR1 ("Patch_cmd failed, reason code=0x%X", status);

            /* Notify application */
            nfc_hal_prm_spd_handle_download_complete (NFC_HAL_PRM_ABORT_INVALID_PATCH_EVT);
            return;
        }

        if (nfc_hal_cb2.prm.flags & NFC_HAL_PRM_FLAGS_USE_PATCHRAM_BUF)
        {
            /* If patch is in a buffer, get next patch from buffer */
            nfc_hal_prm_spd_send_next_segment ();
        }
        else
        {
            /* Notify adaptation layer to get next patch segment (via HAL_NfcPrmDownloadContinue) */
            (nfc_hal_cb2.prm.p_cback) (NFC_HAL_PRM_CONTINUE_EVT);
        }
    }
    /* Handle OPTION_PATCH_HEADER Rsp */
    else if (event == NFC_VS_OPTION_PATCH_END && nfc_hal_cb2.prm.state == NFC_HAL_PRM_ST2_SPD_AUTHENTICATING)
    {
        UINT32 post_signature_delay = NFC_HAL_PRM_COMMIT_DELAY;
        /* Status and error code */
        STREAM_TO_UINT8 (status, p);

        if (status != NCI_STATUS_OK)
        {
            HAL_TRACE_ERROR1 ("Patch_end_cmd failed, reason code=0x%X", status);

            /* Notify application */
            nfc_hal_prm_spd_handle_download_complete (NFC_HAL_PRM_ABORT_INVALID_PATCH_EVT);
            return;
        }
    }
    /* Handle NCI_MSG_OPTION_PATCH_VERSION RSP */
    else if (event == NFC_VS_OPTION_PATCH_VERSION)
    {
        UINT8 nvm_type, status=1;
        if (len > 1 && data_len > NCI_MSG_HDR_SIZE)
        {
            STREAM_TO_UINT8( status, p );
        }
        if( status != NCI_STATUS_OK )
        {
            HAL_TRACE_DEBUG0 ("nfc_hal_prm_nci_command_complete_cback(): OPTION_PATCH_VERSION fail");
        }
        else
        {
            STREAM_TO_UINT8( nvm_type, p );
            STREAM_TO_UINT16( nfc_hal_cb2.nvm_cb.ver_minor, p );
            STREAM_TO_UINT16( nfc_hal_cb2.nvm_cb.ver_major, p );
            HAL_TRACE_DEBUG0 ("nfc_hal_prm_nci_command_complete_cback(): OPTION_PATCH_VERSION ");
        }
        nfc_hal_prm_spd_handle_download_complete (NFC_HAL_PRM_COMPLETE_EVT);
    }
    else
    {
        /* Invalid response from NFCC during patch download */
        HAL_TRACE_ERROR1 ("Invalid response from NFCC during patch download (opcode=0x%02X)", event);
        nfc_hal_prm_spd_handle_download_complete (NFC_HAL_PRM_ABORT_INVALID_PATCH_EVT);
    }

    NFC_HAL_PRM_STATE ("prm_nci_command_complete_cback");
}

/*******************************************************************************
**
** Function         nfc_hal_prm_nfcc_ready_to_continue
**
** Description      Continue to download patch or notify application completition
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_prm_nfcc_ready_to_continue (void)
{
    /* Clear the bit for the patch we just downloaded */
    nfc_hal_cb2.prm.spd_patch_needed_mask = 0;

    /* Done downloading */
    HAL_TRACE_DEBUG0 ("Patch downloaded and authenticated. Get new patch version.");
    /* add get patch info again to verify the effective FW version */
    nfc_hal_dm_send_nci_cmd (nfc_hal_dm_patch_version_cmd, NCI_MSG_HDR_SIZE, nfc_hal_prm_nci_command_complete_cback);
    nfc_hal_cb2.prm.state = NFC_HAL_PRM_ST2_W4_GET_VERSION;
}

/*******************************************************************************
**
** Function         nfc_hal_prm_spd_reset_ntf
**
** Description      Received RESET NTF from NFCC, indicating it has completed
**                  reset after patch download.
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_prm_spd_reset_ntf (UINT8 reset_reason, UINT8 reset_type)
{
    /* Check if we were expecting a RESET NTF */
    if (nfc_hal_cb2.prm.state == NFC_HAL_PRM_ST2_SPD_AUTHENTICATING)
    {
        HAL_TRACE_DEBUG2 ("Received RESET NTF after patch download (reset_reason=%i, reset_type=%i)", reset_reason, reset_type);
        nfc_hal_cb2.prm.state = NFC_HAL_PRM_ST2_SPD_AUTH_DONE;

        /* Stop waiting for nci_wait_rsp_timer */
        nfc_hal_main_stop_quick_timer (&nfc_hal_cb2.ncit_cb.nci_wait_rsp_timer);
        nfc_hal_cb2.ncit_cb.nci_wait_rsp = NFC_HAL_WAIT_RSP_NONE;

        /* Stop waiting for RESET_NTF timer */
        nfc_hal_main_stop_quick_timer (&nfc_hal_cb2.prm.timer);
        {
            /* wait for Nvm Update */
            GKIH_delay(NFC_HAL_W4_NVM_UPDATE);
            /* Continue with patch download */
            nfc_hal_prm_nfcc_ready_to_continue ();
        }
    }
    else
    {
        HAL_TRACE_ERROR2 ("Received unexpected RESET NTF (reset_reason=%i, reset_type=%i)", reset_reason, reset_type);
    }
}

/*******************************************************************************
**
** Function         nfc_hal_prm_process_timeout
**
** Description      Process timer expireation for patch download
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_prm_process_timeout (void *p_tle)
{
    NFC_HAL_PRM_STATE ("nfc_hal_prm_process_timeout");
    if (nfc_hal_cb2.prm.state == NFC_HAL_PRM_ST2_SPD_AUTHENTICATING)
    {
        nfc_hal_prm_nfcc_ready_to_continue ();
    }
    else if (nfc_hal_cb2.prm.state == NFC_HAL_PRM_ST2_SPD_DOWNLOADING)
    {
        HAL_TRACE_DEBUG0 ("Delay...proceeding to download firmware patch");
        nfc_hal_prm_spd_handle_next_patch_start ();
    }
    else
    {
        HAL_TRACE_ERROR1 ("Patch download: command timeout (state=%i)", nfc_hal_cb2.prm.state);
        nfc_hal_prm_spd_handle_download_complete (NFC_HAL_PRM_ABORT_EVT);
    }

}


/*******************************************************************************
**
** Function         HAL_NfcPrmDownloadStart
**
** Description      Initiate patch download
**
** Input Params
**                  format_type     patch format type
**                                  (NFC_HAL_PRM_FORMAT_BIN, NFC_HAL_PRM_FORMAT_HCD, or
**                                   NFC_HAL_PRM_FORMAT_NCD)
**
**                  dest_address    destination adderess (needed for BIN format only)
**
**                  p_patchram_buf  pointer to patchram buffer. If NULL,
**                                  then app must call HAL_NfcPrmDownloadContinue when
**                                  NFC_HAL_PRM_CONTINUE_EVT is received, to send the next
**                                  segment of patchram
**
**                  patchram_len    size of p_patchram_buf (if non-NULL)
**
**                  patchram_delay  The delay after each patch.
**                                  If the given value is less than the size of the patchram,
**                                  the size of patchram is used instead.
**
**                  p_cback         callback for download status
**
**
** Returns          TRUE if successful, otherwise FALSE
**
**
*******************************************************************************/
BOOLEAN HAL_NfcPrmDownloadStart (tNFC_HAL_PRM_FORMAT format_type,
                                 UINT32              dest_address,
                                 UINT8               *p_patchram_buf,
                                 UINT32              patchram_len,
                                 UINT32              patchram_delay,
                                 tNFC_HAL_PRM_CBACK  *p_cback)
{
    HAL_TRACE_API0 ("HAL_NfcPrmDownloadStart ()");

    memset (&nfc_hal_cb2.prm, 0, sizeof (tNFC_HAL_PRM_CB2));

    if (p_patchram_buf)
    {
        nfc_hal_cb2.prm.p_cur_patch_data = p_patchram_buf;
        nfc_hal_cb2.prm.cur_patch_offset = 0;
        nfc_hal_cb2.prm.cur_patch_len_remaining =  patchram_len;
        nfc_hal_cb2.prm.flags |= NFC_HAL_PRM_FLAGS_USE_PATCHRAM_BUF;

        if (patchram_len == 0)
            return FALSE;
    }

    nfc_hal_cb2.prm.p_cback          = p_cback;
    nfc_hal_cb2.prm.dest_ram         = dest_address;
    nfc_hal_cb2.prm.format           = format_type;
    nfc_hal_cb2.prm.patchram_delay   = patchram_delay;

    nfc_hal_cb2.prm.timer.p_cback = nfc_hal_prm_process_timeout;

    if (format_type == NFC_HAL_PRM_FORMAT_NCD)
    {
        /* If patch download is required, but no NVM is available, then abort */
        if ((p_nfc_hal_cfg->nfc_hal_prm_nvm_required) && (nfc_hal_cb2.nvm_cb.flags & NFC_HAL_NVM_FLAGS_NO_NVM))
        {
            HAL_TRACE_ERROR0 ("This platform requires NVM and the NVM is not available - Abort");
            nfc_hal_prm_spd_handle_download_complete (NFC_HAL_PRM_ABORT_NO_NVM_EVT);
            return FALSE;
        }

        /* Compare patch version in NVM with version in patchfile */
        nfc_hal_cb2.prm.state = NFC_HAL_PRM_ST2_SPD_COMPARE_VERSION;
        if (nfc_hal_cb2.prm.flags & NFC_HAL_PRM_FLAGS_USE_PATCHRAM_BUF)
        {
            /* If patchfile is in a buffer, get patch version from buffer */
            nfc_hal_prm_spd_check_version ();
        }
        else
        {
            /* If patchfile is not in a buffer, then request patchfile header from adaptation layer. */
            (nfc_hal_cb2.prm.p_cback) (NFC_HAL_PRM_SPD_GET_PATCHFILE_HDR_EVT);
        }
    }
    else
    {
        HAL_TRACE_ERROR0 ("Unexpected patch format.");
        return FALSE;
    }

    return TRUE;
}
