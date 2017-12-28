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

/******************************************************************************
 *
 *  Vendor-specific handler for DM events
 *
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "nfc_hal_int_extra.h"
#include "nfc_sony_defs.h"
#include "nfc_hal_post_reset.h"
#include "userial.h"
#include "upio.h"
#include "config.h"
#include "halimpl_version.h"
#include <cutils/properties.h>
#include "nfc_types_extra.h"
/*****************************************************************************
** Constants and types
*****************************************************************************/

static const UINT8 nfc_hal_dm_core_reset_cmd[NCI_MSG_HDR_SIZE + NCI_CORE_PARAM_SIZE_RESET] =
{
    NCI_MTS_CMD|NCI_GID_CORE,
    NCI_MSG_CORE_RESET,
    NCI_CORE_PARAM_SIZE_RESET,
    NCI_RESET_TYPE_RESET_CFG
};

/* NCI command: CXD224x proprietary */
static const UINT8 nfc_hal_dm_get_patch_version_cmd [NCI_MSG_HDR_SIZE] =
{
    NCI_MTS_CMD|NCI_GID_PROP,
    NCI_MSG_OPTION_PATCH_VERSION,
    0x00
};
#define NCI_PATCH_INFO_OFFSET_PATCH_VERSION  2  /* PATCH_VERSION offset in patch info RSP */
static const UINT8 nfc_hal_dm_get_hw_version_cmd [NCI_MSG_HDR_SIZE] =
{
    NCI_MTS_CMD|NCI_GID_PROP,
    NCI_MSG_OPTION_GET_VERSION,
    0x00
};
static const UINT8 nfc_hal_dm_get_int_version_cmd [NCI_MSG_HDR_SIZE] =
{
    NCI_MTS_CMD|NCI_GID_PROP,
    NCI_MSG_OPTION_GET_INTERNAL_VERSION,
    0x00
};

#if (defined(NFC_HAL_HCI_INCLUDED) && (NFC_HAL_HCI_INCLUDED == TRUE))
#define NFA_NVSTORE_NVADDR 0x30000
static const UINT8 nfc_hal_dm_flash_read [NCI_MSG_HDR_SIZE + 0x8] =
{
    NCI_MTS_CMD|NCI_GID_PROP,
    NCI_MSG_OPTION_FLASH_ACCESS,
    0x08,
    0x2, /* read */
    0x0,
    0x4, /* sz */
    0x0,
    ((NFA_NVSTORE_NVADDR   )  & 0xff),
    ((NFA_NVSTORE_NVADDR>>8)  & 0xff),
    ((NFA_NVSTORE_NVADDR>>16) & 0xff),
    ((NFA_NVSTORE_NVADDR>>24) & 0xff),
};
static const UINT8 nfc_hal_dm_flash_erase [NCI_MSG_HDR_SIZE + 0x8] =
{
    NCI_MTS_CMD|NCI_GID_PROP,
    NCI_MSG_OPTION_FLASH_ACCESS,
    0x08,
    0x0, /* erase */
    0x0,
    0x10, /* sz */
    0x0,
    ((NFA_NVSTORE_NVADDR   )  & 0xff),
    ((NFA_NVSTORE_NVADDR>>8)  & 0xff),
    ((NFA_NVSTORE_NVADDR>>16) & 0xff),
    ((NFA_NVSTORE_NVADDR>>24) & 0xff),
};
#endif

/*****************************************************************************
** Extern function prototypes
*****************************************************************************/
extern UINT8 *p_nfc_hal_dm_lptd_cfg;
extern UINT8 *p_nfc_hal_dm_start_up_cfg;
extern tNFC_HAL_CFG *p_nfc_hal_cfg;
extern UINT8 *p_nfc_hal_dm_lmr_cfg;
extern UINT8 *p_nfc_hal_dm_start_up_rmw_cfg;
extern UINT8 *p_nfc_hal_dm_start_up_vsc_cfg;
extern UINT8 *p_nfc_hal_dm_closing_vsc_cfg;
extern UINT8 *p_nfc_hal_dm_start_up_debug_cfg;
extern FILE *p_nfc_hal_dm_start_up_debug_outfd;

/*******************************************************************************
 **
 ** Function         nfc_hal_dm_send_lmr_cmd
 **
 ** Description      Send RF_SET_LISTEN_MODE_ROUTING_CMD to NFCC (for CXD224X)
 **
 ** Returns          tHAL_NFC_STATUS
 **
 *******************************************************************************/
tHAL_NFC_STATUS nfc_hal_dm_send_lmr_cmd (UINT8 tlv_size,
                                         UINT8 *p_param_tlvs,
                                         tNFC_HAL_NCI_CBACK *p_cback)
{
    UINT8  *p_buff, *p;
    UINT8  num_param = 0, *p_tlv;
    int param_len, rem_len;
    UINT16 cmd_len = (UINT16 )(NCI_MSG_HDR_SIZE + tlv_size + 2);
    tHAL_NFC_STATUS status = HAL_NFC_STATUS_FAILED;

    if ((tlv_size == 0)||(p_param_tlvs == NULL))
    {
        return status;
    }
    if ((p_buff = (UINT8 *) GKIH_getbuf ((UINT16)(NCI_MSG_HDR_SIZE + tlv_size+2))) != NULL)
    {
        p = p_buff;

        NCI_MSG_BLD_HDR0 (p, NCI_MT_CMD, NCI_GID_RF_MANAGE);
        NCI_MSG_BLD_HDR1 (p, NCI_MSG_RF_SET_ROUTING);
        UINT8_TO_STREAM  (p, (UINT8) (tlv_size + 2));

        rem_len = (int )tlv_size;
        p_tlv   = p_param_tlvs;
        while (rem_len > 1)
        {
            num_param++;                /* number of params */

            p_tlv ++;                   /* param type   */
            param_len = (int )(*p_tlv++);       /* param length */

            rem_len -= 2;               /* param type and length */
            if (rem_len >= param_len)
            {
                rem_len -= param_len;
                p_tlv   += param_len;   /* next param_type */

                if (rem_len == 0)
                {
                    status = HAL_NFC_STATUS_OK;
                    break;
                }
            }
            else
            {
                /* error found */
                break;
            }
        }

        if (status == HAL_NFC_STATUS_OK)
        {
            UINT8 more=0;
            UINT8_TO_STREAM (p, more);
            UINT8_TO_STREAM (p, num_param);
            ARRAY_TO_STREAM (p, p_param_tlvs, tlv_size);
            nfc_hal_dm_send_nci_cmd (p_buff, cmd_len, p_cback);
        }
        else
        {
            HAL_TRACE_ERROR0 ("nfc_hal_dm_send_lmr_cmd ():Bad TLV");
        }

        GKIH_freebuf (p_buff);
    }

    return status;
}
/*******************************************************************************
 **
 ** Function         nfc_hal_dm_set_config
 **
 ** Description      Send NCI config items to NFCC
 **
 ** Returns          tHAL_NFC_STATUS
 **
 *******************************************************************************/
tHAL_NFC_STATUS nfc_hal_dm_set_config (UINT8 tlv_size,
                                       UINT8 *p_param_tlvs,
                                       tNFC_HAL_NCI_CBACK *p_cback)
{
    UINT8  *p_buff, *p;
    UINT8  num_param = 0, *p_tlv;
    int param_len, rem_len;
    UINT16 cmd_len = (UINT16 )(NCI_MSG_HDR_SIZE + tlv_size + 1);
    tHAL_NFC_STATUS status = HAL_NFC_STATUS_FAILED;

    if ((tlv_size == 0)||(p_param_tlvs == NULL))
    {
        return status;
    }

    if ((p_buff = (UINT8 *) GKIH_getbuf ((UINT16)(NCI_MSG_HDR_SIZE + tlv_size))) != NULL)
    {
        p = p_buff;

        NCI_MSG_BLD_HDR0 (p, NCI_MT_CMD, NCI_GID_CORE);
        NCI_MSG_BLD_HDR1 (p, NCI_MSG_CORE_SET_CONFIG);
        UINT8_TO_STREAM  (p, (UINT8) (tlv_size + 1));

        rem_len = (int )tlv_size;
        p_tlv   = p_param_tlvs;
        while (rem_len > 1)
        {
            num_param++;                /* number of params */

            p_tlv ++;                   /* param type   */
            param_len = (int )(*p_tlv++);       /* param length */

            rem_len -= 2;               /* param type and length */
            if (rem_len >= param_len)
            {
                rem_len -= param_len;
                p_tlv   += param_len;   /* next param_type */

                if (rem_len == 0)
                {
                    status = HAL_NFC_STATUS_OK;
                    break;
                }
            }
            else
            {
                /* error found */
                break;
            }
        }

        if (status == HAL_NFC_STATUS_OK)
        {
            UINT8_TO_STREAM (p, num_param);
            ARRAY_TO_STREAM (p, p_param_tlvs, tlv_size);

            nfc_hal_dm_send_nci_cmd (p_buff, cmd_len, p_cback);
        }
        else
        {
            HAL_TRACE_ERROR0 ("nfc_hal_dm_set_config ():Bad TLV");
        }

        GKIH_freebuf (p_buff);
    }

    return status;
}

/*******************************************************************************
 **
 ** Function         nfc_hal_dm_config_nfcc_cback
 **
 ** Description      Callback for NCI vendor specific command complete
 **
 ** Returns          void
 **
 *******************************************************************************/
void nfc_hal_dm_config_nfcc_cback (tNFC_HAL_NCI_EVT event, UINT16 data_len, UINT8 *p_data)
{
    if (nfc_hal_cb2.dev_cb.next_dm_config == NFC_HAL_DM_CONFIG2_NONE)
    {
        NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_IDLE);
        nfc_hal_cb2.p_stack_cback (HAL_NFC_POST_INIT_CPLT_EVT, HAL_NFC_STATUS_OK);
    }
    else
    {
        nfc_hal_dm_config_nfcc ();
    }
}
/*******************************************************************************
 **
 ** Function         nfc_hal_dm_config_nfcc_closing_cback
 **
 ** Description      Callback for NCI vendor specific command complete
 **
 ** Returns          void
 **
 *******************************************************************************/
void nfc_hal_dm_config_nfcc_closing_cback (tNFC_HAL_NCI_EVT event, UINT16 data_len, UINT8 *p_data)
{
    if (nfc_hal_cb2.dev_cb.next_dm_config == NFC_HAL_DM_CONFIG2_NONE)
    {
        NFC_HDR  *p_msg;
        /* post event to post terminate in NCIT task */
        if ((p_msg = (NFC_HDR *) GKIH_getbuf (NFC_HDR_SIZE)) != NULL)
        {
            p_msg->event = NFC_HAL_EVT_POST_TERMINATE;
            GKIH_send_msg (NFC_HAL_TASK, NFC_HAL_TASK_MBOX, p_msg);
        }
    }
    else
    {
        nfc_hal_dm_config_nfcc_closing ();
    }
}
static const char *dm_config_str( UINT8 type )
{
    switch(type){
    case NFC_HAL_DM_CONFIG2_LPTD:
        return "LPTD";
    case NFC_HAL_DM_CONFIG2_START_UP:
        return "START_UP";
    case NFC_HAL_DM_CONFIG2_LMR:
        return "LMR";
    case NFC_HAL_DM_CONFIG2_START_UP_RMWPRE:
        return "START_UP_RMWPRE";
    case NFC_HAL_DM_CONFIG2_START_UP_RMW:
        return "START_UP_RMW";
    case NFC_HAL_DM_CONFIG2_START_UP_RMWPOST:
        return "START_UP_DEBUGPOST";
    case NFC_HAL_DM_CONFIG2_START_UP_DEBUGPRE:
        return "START_UP_DEBUGPRE";
    case NFC_HAL_DM_CONFIG2_START_UP_DEBUG:
        return "START_UP_DEBUG";
    case NFC_HAL_DM_CONFIG2_START_UP_DEBUGPOST:
        return "START_UP_DEBUGPOST";
    case NFC_HAL_DM_CONFIG2_START_UP_VSC:
        return "START_UP_VSC";
    case NFC_HAL_DM_CONFIG2_NONE:
        return "NONE";
    default:
        ;
    }
    return "Ukn";
}

/*******************************************************************************
**
** Function         nfc_hal_dm_send_startup_vsc
**
** Description      Send VS command before NFA start-up
**
** Returns          None
**
*******************************************************************************/
void nfc_hal_dm_send_startup_vsc (void)
{
    UINT8  *p, *p_end;
    UINT16 len;

    HAL_TRACE_DEBUG0 ("nfc_hal_dm_send_startup_vsc ()");

    /* VSC must have NCI header at least */
    if (nfc_hal_cb2.dev_cb.next_startup_vsc + NCI_MSG_HDR_SIZE - 1 <= *p_nfc_hal_dm_start_up_vsc_cfg)
    {
        p     = p_nfc_hal_dm_start_up_vsc_cfg + nfc_hal_cb2.dev_cb.next_startup_vsc;
        len   = *(p + 2);
        p_end = p + NCI_MSG_HDR_SIZE - 1 + len;

        if (p_end <= p_nfc_hal_dm_start_up_vsc_cfg + *p_nfc_hal_dm_start_up_vsc_cfg)
        {
            /* move to next VSC */
            nfc_hal_cb2.dev_cb.next_startup_vsc += NCI_MSG_HDR_SIZE + len;

            /* if this is last VSC */
            if (p_end == p_nfc_hal_dm_start_up_vsc_cfg + *p_nfc_hal_dm_start_up_vsc_cfg)
                nfc_hal_cb2.dev_cb.next_dm_config = NFC_HAL_DM_CONFIG2_NONE;

            nfc_hal_dm_send_nci_cmd (p, (UINT16)(NCI_MSG_HDR_SIZE + len), nfc_hal_dm_config_nfcc_cback);
            return;
        }
    }

    HAL_TRACE_ERROR0 ("nfc_hal_dm_send_startup_vsc (): Bad start-up VSC");

    NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_IDLE);
    nfc_hal_cb2.p_stack_cback (HAL_NFC_POST_INIT_CPLT_EVT, HAL_NFC_STATUS_FAILED);
}

/*******************************************************************************
**
** Function         nfc_hal_dm_send_startup_rmw
**
** Description      Read modified Write device memory before NFA start-up
**
** Returns          None
**
*******************************************************************************/
void nfc_hal_dm_send_startup_rmw (void)
{
    UINT8  *p, *pp;
    UINT32 len;
    UINT8 len_cfg;
    len_cfg = p_nfc_hal_dm_start_up_rmw_cfg[0];

    HAL_TRACE_DEBUG2 ("nfc_hal_dm_send_startup_rmw(): rmw_state=%d remain_len=%i",
                      nfc_hal_cb2.rmw.state, len_cfg - nfc_hal_cb2.rmw.cur);

    if (nfc_hal_cb2.rmw.cur + 4*3 <= len_cfg && nfc_hal_cb2.rmw.status == HAL_NFC_STATUS_OK)
    {
        UINT8 cmd[NCI_MSG_HDR_SIZE+8];
        UINT32 wmask, adr, wdata;
        cmd[0] = NCI_MTS_CMD|NCI_GID_PROP;
        p      = p_nfc_hal_dm_start_up_rmw_cfg + nfc_hal_cb2.rmw.cur + 1;
        pp     = &cmd[3];
        STREAM_TO_UINT32 (adr, p);
        UINT32_TO_STREAM (pp, adr); /* address */
        switch (nfc_hal_cb2.rmw.state){
        case NFC_HAL_DM_RMW_STATE_READ:
            cmd[1] = NCI_MSG_OPTION_READ_ADR;
            cmd[2] = 4;
            break;
        case NFC_HAL_DM_RMW_STATE_WRITE:
            cmd[1] = NCI_MSG_OPTION_WRITE_ADR;
            cmd[2] = 8;
            STREAM_TO_UINT32 (wmask, p);
            STREAM_TO_UINT32 (wdata, p);
            UINT32_TO_STREAM (pp, (nfc_hal_cb2.rmw.data & ~wmask)|(wdata & wmask));
            HAL_TRACE_DEBUG5 ("nfc_hal_dm_send_startup_rmw(): (rd:0x%x & ~msk:0x%08x)|(wd:0x%x & msk:0x%08x)=0x%x",
                              nfc_hal_cb2.rmw.data, ~wmask, wdata, wmask, (nfc_hal_cb2.rmw.data & ~wmask)|(wdata & wmask));
            break;
        default:
            cmd[2] = 0;
            HAL_TRACE_ERROR4 ("nfc_hal_dm_send_startup_rmw(): status=%d cur=%d state=%d data=0x%04x",
                              nfc_hal_cb2.rmw.status, nfc_hal_cb2.rmw.cur, nfc_hal_cb2.rmw.state, nfc_hal_cb2.rmw.data);
        }
        nfc_hal_cb2.dev_cb.next_dm_config = NFC_HAL_DM_CONFIG2_START_UP_RMW;
        if (nfc_hal_cb2.rmw.state == NFC_HAL_DM_RMW_STATE_WRITE)
        {
            nfc_hal_cb2.rmw.cur += 4*3;
            nfc_hal_cb2.rmw.state = NFC_HAL_DM_RMW_STATE_READ;
            if (len_cfg <= nfc_hal_cb2.rmw.cur)
            {
                nfc_hal_cb2.dev_cb.next_dm_config = NFC_HAL_DM_CONFIG2_START_UP_RMWPOST;
            }
        }
        else
        {
            nfc_hal_cb2.rmw.state = NFC_HAL_DM_RMW_STATE_WRITE;
        }
        if (cmd[2] > 0)
        {
            nfc_hal_dm_send_nci_cmd (cmd, (UINT16)(NCI_MSG_HDR_SIZE + cmd[2]), nfc_hal_dm_config_nfcc_cback);
            return;
        }
    }
    HAL_TRACE_ERROR0 ("nfc_hal_dm_send_startup_rmw(): Bad start-up rmw");

    nfc_hal_cb2.dev_cb.next_dm_config = NFC_HAL_DM_CONFIG2_START_UP_RMWPOST;
    nfc_hal_dm_config_nfcc ();
}

/*******************************************************************************
**
** Function         nfc_hal_dm_send_startup_debug
**
** Description      Send command list before NFA start-up(debug useonly)
**
** Returns          None
**
*******************************************************************************/
void nfc_hal_dm_send_startup_debug (void)
{
    UINT8  *p, *p_end;
    UINT32 len;
    INT32 len_cfg;
    len_cfg = (INT32 )((UINT16 )(p_nfc_hal_dm_start_up_debug_cfg[0] | (p_nfc_hal_dm_start_up_debug_cfg[1]<<8))) + 1; /* len=>pointer*/

    HAL_TRACE_DEBUG1 ("nfc_hal_dm_send_startup_debug() remain_len=%i", len_cfg - nfc_hal_cb2.dev_cb.next_startup_debug + 1);

    /* debug command must have NCI header at least */
    if (nfc_hal_cb2.dev_cb.next_startup_debug + NCI_MSG_HDR_SIZE - 1 <= len_cfg)
    {
        p     = p_nfc_hal_dm_start_up_debug_cfg + nfc_hal_cb2.dev_cb.next_startup_debug;
        len   = *(p + 2);
        p_end = p + NCI_MSG_HDR_SIZE - 1 + len;

        if (p_end <= p_nfc_hal_dm_start_up_debug_cfg + len_cfg)
        {
            /* move to next debug command */
            nfc_hal_cb2.dev_cb.next_startup_debug += NCI_MSG_HDR_SIZE + len;

            /* if this is last debug command */
            if (p_end == p_nfc_hal_dm_start_up_debug_cfg + len_cfg )
                nfc_hal_cb2.dev_cb.next_dm_config = NFC_HAL_DM_CONFIG2_START_UP_DEBUGPOST;
            else
                nfc_hal_cb2.dev_cb.next_dm_config = NFC_HAL_DM_CONFIG2_START_UP_DEBUG;

            nfc_hal_dm_send_nci_cmd (p, (UINT16)(NCI_MSG_HDR_SIZE + len), nfc_hal_dm_config_nfcc_cback);
            return;
        }
    }

    HAL_TRACE_ERROR0 ("nfc_hal_dm_send_startup_debug(): Bad start-up debug command");

    NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_IDLE);
    nfc_hal_cb2.p_stack_cback (HAL_NFC_POST_INIT_CPLT_EVT, HAL_NFC_STATUS_FAILED);
}

/*******************************************************************************
**
** Function         nfc_hal_dm_send_closing_vsc
**
** Description      Send VS command before NFA closing
**
** Returns          None
**
*******************************************************************************/
void nfc_hal_dm_send_closing_vsc (void)
{
    UINT8  *p, *p_end;
    UINT16 len;
    NFC_HDR  *p_msg;

    HAL_TRACE_DEBUG0 ("nfc_hal_dm_send_closing_vsc ()");

    /* VSC must have NCI header at least */
    if (nfc_hal_cb2.dev_cb.next_closing_vsc + NCI_MSG_HDR_SIZE - 1 <= *p_nfc_hal_dm_closing_vsc_cfg)
    {
        p     = p_nfc_hal_dm_closing_vsc_cfg + nfc_hal_cb2.dev_cb.next_closing_vsc;
        len   = *(p + 2);
        p_end = p + NCI_MSG_HDR_SIZE - 1 + len;

        if (p_end <= p_nfc_hal_dm_closing_vsc_cfg + *p_nfc_hal_dm_closing_vsc_cfg)
        {
            /* move to next VSC */
            nfc_hal_cb2.dev_cb.next_closing_vsc += NCI_MSG_HDR_SIZE + len;

            /* if this is last VSC */
            if (p_end == p_nfc_hal_dm_closing_vsc_cfg + *p_nfc_hal_dm_closing_vsc_cfg)
                nfc_hal_cb2.dev_cb.next_dm_config = NFC_HAL_DM_CONFIG2_NONE;

            nfc_hal_dm_send_nci_cmd (p, (UINT16)(NCI_MSG_HDR_SIZE + len), nfc_hal_dm_config_nfcc_closing_cback);
            return;
        }
    }

    HAL_TRACE_ERROR0 ("nfc_hal_dm_send_closing_vsc (): Bad closing VSC");

    NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_CLOSING);

    /* post event to post terminate in NCIT task */
    if ((p_msg = (NFC_HDR *) GKIH_getbuf (NFC_HDR_SIZE)) != NULL)
    {
        p_msg->event = NFC_HAL_EVT_POST_TERMINATE;
        GKIH_send_msg (NFC_HAL_TASK, NFC_HAL_TASK_MBOX, p_msg);
    }
}

/*******************************************************************************
**
** Function         nfc_hal_dm_config_nfcc
**
** Description      Send VS config before NFA start-up
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_dm_config_nfcc (void)
{
    HAL_TRACE_DEBUG2 ("nfc_hal_dm_config_nfcc (): next_dm_config = %d(%s)", nfc_hal_cb2.dev_cb.next_dm_config,
		      dm_config_str(nfc_hal_cb2.dev_cb.next_dm_config));

    if ((p_nfc_hal_dm_lptd_cfg[0]) && (nfc_hal_cb2.dev_cb.next_dm_config <= NFC_HAL_DM_CONFIG2_LPTD))
    {
        nfc_hal_cb2.dev_cb.next_dm_config = NFC_HAL_DM_CONFIG2_START_UP;

        if (nfc_hal_dm_set_config (p_nfc_hal_dm_lptd_cfg[0],
                                   &p_nfc_hal_dm_lptd_cfg[1],
                                   nfc_hal_dm_config_nfcc_cback) == HAL_NFC_STATUS_OK)
        {
            return;
        }
        else
        {
            NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_IDLE);
            nfc_hal_cb2.p_stack_cback (HAL_NFC_POST_INIT_CPLT_EVT, HAL_NFC_STATUS_FAILED);
            return;
        }
    }
    HAL_TRACE_DEBUG2 ("nfc_hal_dm_config_nfcc () start_up_cfg[0][1]=%i:%i", p_nfc_hal_dm_start_up_cfg[0], p_nfc_hal_dm_start_up_cfg[1]);
    if ((p_nfc_hal_dm_start_up_cfg[0]) && (nfc_hal_cb2.dev_cb.next_dm_config <= NFC_HAL_DM_CONFIG2_START_UP))
    {
        nfc_hal_cb2.dev_cb.next_dm_config = NFC_HAL_DM_CONFIG2_LMR;

        if (nfc_hal_dm_set_config (p_nfc_hal_dm_start_up_cfg[0],
                                   &p_nfc_hal_dm_start_up_cfg[1],
                                   nfc_hal_dm_config_nfcc_cback) == HAL_NFC_STATUS_OK)
        {
            return;
        }
        else
        {
            NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_IDLE);
            nfc_hal_cb2.p_stack_cback (HAL_NFC_POST_INIT_CPLT_EVT, HAL_NFC_STATUS_FAILED);
            return;
        }
    }

    /*  send RF_SET_LISTEN_MODE_ROUTING_CMD  */
    if ((p_nfc_hal_dm_lmr_cfg[0]) && (nfc_hal_cb2.dev_cb.next_dm_config <= NFC_HAL_DM_CONFIG2_LMR) )
    {
        nfc_hal_cb2.dev_cb.next_dm_config = NFC_HAL_DM_CONFIG2_START_UP_RMWPRE;
        if (nfc_hal_dm_send_lmr_cmd (p_nfc_hal_dm_lmr_cfg[0],
                                     &p_nfc_hal_dm_lmr_cfg[1],
                                     nfc_hal_dm_config_nfcc_cback) == HAL_NFC_STATUS_OK)
        {
            return;
        }
        else
        {
            NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_IDLE);
            nfc_hal_cb2.p_stack_cback (HAL_NFC_POST_INIT_CPLT_EVT, HAL_NFC_STATUS_FAILED);
            return;
        }
    }
    /* Read modified Write Device memory */
    if (nfc_hal_cb2.dev_cb.next_dm_config <= NFC_HAL_DM_CONFIG2_START_UP_RMWPOST )
    {
        if ( p_nfc_hal_dm_start_up_rmw_cfg[0] )
        {
            UINT8 len_cfg;
            if (nfc_hal_cb2.dev_cb.next_dm_config <= NFC_HAL_DM_CONFIG2_START_UP_RMWPRE)
            {
                nfc_hal_cb2.rmw.status = NCI_STATUS_OK;
                nfc_hal_cb2.rmw.cur = 0;
                nfc_hal_cb2.rmw.state = NFC_HAL_DM_RMW_STATE_READ;
            }

            len_cfg = p_nfc_hal_dm_start_up_rmw_cfg[0];
            if (len_cfg && nfc_hal_cb2.dev_cb.next_dm_config <= NFC_HAL_DM_CONFIG2_START_UP_RMW)
            {
                nfc_hal_dm_send_startup_rmw ();
                return;
            }
        }
        nfc_hal_cb2.dev_cb.next_dm_config = NFC_HAL_DM_CONFIG2_START_UP_DEBUGPRE;
    }

#if (NFCNCI_DEBUGFUNC == TRUE)
    if (nfc_hal_cb2.dev_cb.next_dm_config <= NFC_HAL_DM_CONFIG2_START_UP_DEBUGPOST )
    {
        char valueStr [PROPERTY_VALUE_MAX] = {0};
        int len = property_get("nfc.debug.cmdlist_en", valueStr, "");
        unsigned long num;
        if( len > 0 && sscanf (valueStr, "%lu", &num) == 1 )
        {
            if ( num == 1 && p_nfc_hal_dm_start_up_debug_cfg )
            {
                INT32 len_cfg;
                len_cfg = (INT32 )((UINT16 )(p_nfc_hal_dm_start_up_debug_cfg[0] | (p_nfc_hal_dm_start_up_debug_cfg[1]<<8)));
                if (len_cfg && nfc_hal_cb2.dev_cb.next_dm_config <= NFC_HAL_DM_CONFIG2_START_UP_DEBUG)
                {
                    nfc_hal_dm_send_startup_debug ();
                    return;
                }
                free(p_nfc_hal_dm_start_up_debug_cfg);
                p_nfc_hal_dm_start_up_debug_cfg = NULL;

                /* close output rsp file */
                if( p_nfc_hal_dm_start_up_debug_outfd )
                    fclose(p_nfc_hal_dm_start_up_debug_outfd);
                p_nfc_hal_dm_start_up_debug_outfd = NULL;

                /* init: sys_prop: permission denied uid:1027 */
                if( property_set("nfc.debug.cmdlist_en", "2") < 0 )
                    HAL_TRACE_ERROR0 ("nfc_hal_dm_config_nfcc () Failed to set property");
            }
        }
        nfc_hal_cb2.dev_cb.next_dm_config = NFC_HAL_DM_CONFIG2_START_UP_VSC;
    }
#endif

    if (nfc_hal_cb2.dev_cb.next_dm_config <= NFC_HAL_DM_CONFIG2_START_UP_VSC)
    {
        HAL_TRACE_DEBUG1 ("nfc_hal_dm_config_nfcc (): vsc_cfg=0x%x",*p_nfc_hal_dm_start_up_vsc_cfg );
        if (p_nfc_hal_dm_start_up_vsc_cfg && *p_nfc_hal_dm_start_up_vsc_cfg)
        {
            nfc_hal_dm_send_startup_vsc ();
            return;
        }
    }

    /* nothing to config */
    HAL_TRACE_DEBUG0 ("nfc_hal_dm_config_nfcc (): nothing to do");
    nfc_hal_cb2.dev_cb.next_dm_config = NFC_HAL_DM_CONFIG2_NONE;
    nfc_hal_dm_config_nfcc_cback (0, 0, NULL);
}

/*******************************************************************************
**
** Function         nfc_hal_dm_config_nfcc_closing
**
** Description      Send VS config before closing thread
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_dm_config_nfcc_closing (void)
{
    if (nfc_hal_cb2.dev_cb.next_dm_config <= NFC_HAL_DM_CONFIG2_CLOSING_VSC)
    {
        HAL_TRACE_DEBUG1 ("nfc_hal_dm_config_nfcc_closing (): vsc_cfg=0x%x",*p_nfc_hal_dm_closing_vsc_cfg );
        if (p_nfc_hal_dm_closing_vsc_cfg && *p_nfc_hal_dm_closing_vsc_cfg)
        {
            nfc_hal_dm_send_closing_vsc ();
            return;
        }
    }

    /* nothing to config */
    HAL_TRACE_DEBUG0 ("nfc_hal_dm_config_nfcc_closing (): nothing to do");
    nfc_hal_cb2.dev_cb.next_dm_config = NFC_HAL_DM_CONFIG2_NONE;
    nfc_hal_dm_config_nfcc_closing_cback (0, 0, NULL);
}
/*******************************************************************************
**
** Function         nfc_hal_dm_send_reset_cmd
**
** Description      Send CORE RESET CMD
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_dm_send_reset_cmd (void)
{
    NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_W4_RESET);

    HAL_TRACE_DEBUG1 ("nfc_hal_dm_send_reset_cmd(): state:%d",
                      nfc_hal_cb2.dev_cb.initializing_state);

    nfc_hal_dm_send_nci_cmd (nfc_hal_dm_core_reset_cmd, NCI_MSG_HDR_SIZE + NCI_CORE_PARAM_SIZE_RESET, NULL);
}

/*******************************************************************************
**
** Function         nfc_hal_dm_proc_msg_during_init
**
** Description      Process NCI message while initializing NFCC
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_dm_proc_msg_during_init (NFC_HDR *p_msg)
{
    UINT8 *p;
    UINT8 reset_reason, reset_type;
    UINT8 mt, pbf, gid, op_code;
    UINT8 *p_old, old_gid, old_oid, old_mt;
    tNFC_HAL_NCI_CBACK *p_cback = NULL;

    HAL_TRACE_DEBUG2 ("nfc_hal_dm_proc_msg_during_init(): init_state:%d dm_config:%d",
                      nfc_hal_cb2.dev_cb.initializing_state, nfc_hal_cb2.dev_cb.next_dm_config);

    p = (UINT8 *) (p_msg + 1) + p_msg->offset;

#if (NFCNCI_DEBUGFUNC == TRUE)
    if( (nfc_hal_cb2.dev_cb.next_dm_config >= NFC_HAL_DM_CONFIG2_START_UP_DEBUG
         && nfc_hal_cb2.dev_cb.next_dm_config <= NFC_HAL_DM_CONFIG2_START_UP_DEBUGPOST )
        && p_nfc_hal_dm_start_up_debug_outfd )
    {
        HAL_TRACE_DEBUG1 ("nfc_hal_dm_proc_msg_during_init(): rsp_write len=%d", (size_t )p[2] + NCI_MSG_HDR_SIZE);
        fwrite( p, (size_t )p[2] + NCI_MSG_HDR_SIZE, 1, p_nfc_hal_dm_start_up_debug_outfd );
    }
#endif

    NCI_MSG_PRS_HDR0 (p, mt, pbf, gid);
    NCI_MSG_PRS_HDR1 (p, op_code);

    /* check if waiting for this response */
    if (  (nfc_hal_cb2.ncit_cb.nci_wait_rsp == NFC_HAL_WAIT_RSP_CMD)
        ||(nfc_hal_cb2.ncit_cb.nci_wait_rsp == NFC_HAL_WAIT_RSP_VSC)  )
    {
        if (mt == NCI_MT_RSP)
        {
            p_old = nfc_hal_cb2.ncit_cb.last_hdr;
            NCI_MSG_PRS_HDR0 (p_old, old_mt, pbf, old_gid);
            old_oid = ((*p_old) & NCI_OID_MASK);
            /* make sure this is the RSP we are waiting for before updating the command window */
            if ((old_gid == gid) && (old_oid == op_code))
            {
                nfc_hal_cb2.ncit_cb.nci_wait_rsp = NFC_HAL_WAIT_RSP_NONE;
                p_cback = (tNFC_HAL_NCI_CBACK *)nfc_hal_cb2.ncit_cb.p_vsc_cback;
                nfc_hal_cb2.ncit_cb.p_vsc_cback  = NULL;
                nfc_hal_main_stop_quick_timer (&nfc_hal_cb2.ncit_cb.nci_wait_rsp_timer);
            }
        }
    }

    if (gid == NCI_GID_CORE
        || ((gid == NCI_GID_RF_MANAGE) && (op_code == NCI_MSG_RF_SET_ROUTING)))
    {
        if (op_code == NCI_MSG_CORE_RESET)
        {
            HAL_TRACE_DEBUG2 ("nfc_hal_dm_proc_msg_during_init(): RESET mt=%x state=%x", mt, nfc_hal_cb2.dev_cb.initializing_state );
            if (mt == NCI_MT_RSP)
            {
                if (nfc_hal_cb2.dev_cb.initializing_state == NFC_HAL_INIT_STATE2_W4_RESET)
                {
#if (defined(NFC_HAL_HCI_INCLUDED) && (NFC_HAL_HCI_INCLUDED == TRUE))
                    HAL_TRACE_DEBUG1 ("nfc_hal_dm_proc_msg_during_init(): nverase=%d", nfc_hal_cb2.nverase);
                    if (nfc_hal_cb2.nverase == NFC_HAL_NVERASE_ERASE
                        && nfc_hal_cb2.hal_init_ctrl & NFC_NFASTORAGE_NVERASE)
                    {
                        NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_W4_NVERASE);
                        nfc_hal_dm_send_nci_cmd (nfc_hal_dm_flash_read, sizeof(nfc_hal_dm_flash_read), NULL);
                    }
                    else
#endif
                    {
                        NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_W4_HW_INFO);
                        nfc_hal_dm_send_nci_cmd (nfc_hal_dm_get_hw_version_cmd, NCI_MSG_HDR_SIZE, NULL);
                    }
                }
                else if( nfc_hal_cb2.dev_cb.initializing_state == NFC_HAL_INIT_STATE2_W4_NFCC_ENABLE)
                {
                    ; /* ignore unexpected CORE_RESET_RSP during preinit */
                }
                else
                {
                    NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_IDLE);
                    nfc_hal_main_pre_init_done (HAL_NFC_STATUS_OK);
                }
            }
            else
            {
                if( nfc_hal_cb2.dev_cb.initializing_state == NFC_HAL_INIT_STATE2_W4_NFCC_ENABLE)
                {
                    nfc_hal_main_stop_quick_timer (&nfc_hal_cb2.timer);
                    nfc_hal_dm_pre_init_nfcc ();
                }
                else
                {
                    /* Call reset notification callback */
                    p++;                                /* Skip over param len */
                    STREAM_TO_UINT8 (reset_reason, p);
                    STREAM_TO_UINT8 (reset_type, p);
                    nfc_hal_prm_spd_reset_ntf (reset_reason, reset_type);
                }
            }
        }
        else if (p_cback)
        {
            (*p_cback) ((tNFC_HAL_NCI_EVT) (op_code),
                        p_msg->len,
                        (UINT8 *) (p_msg + 1) + p_msg->offset);
        }
    }
    else if (gid == NCI_GID_PROP) /* this is for download patch */
    {
        if (mt == NCI_MT_NTF)
            op_code |= NCI_NTF_BIT;
        else
            op_code |= NCI_RSP_BIT;

        if ( op_code == (NCI_RSP_BIT|NCI_MSG_OPTION_FLASH_ACCESS)
             && (nfc_hal_cb2.dev_cb.initializing_state == NFC_HAL_INIT_STATE2_W4_NVERASE))
        {   /* erase done */
#if (defined(NFC_HAL_HCI_INCLUDED) && (NFC_HAL_HCI_INCLUDED == TRUE))
            UINT8 state;
            UINT8 len;
            STREAM_TO_UINT8 (len, p);
            STREAM_TO_UINT8 (state, p);
            if ( !state && len == 5 )
            {
                UINT32 type;
                BE_STREAM_TO_UINT32(type, p);
                if (type == NFC_HAL_FOURCC('A', 'N', 'Y', 'd'))
                    nfc_hal_dm_send_nci_cmd (nfc_hal_dm_flash_erase, sizeof(nfc_hal_dm_flash_erase), NULL);
                else
                {
                    HAL_TRACE_DEBUG0 ("NvErase not found");
                    NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_W4_HW_INFO);
                    nfc_hal_dm_send_nci_cmd (nfc_hal_dm_get_hw_version_cmd, NCI_MSG_HDR_SIZE, NULL);
                }
            }
            else if ( !state && len == 1 )
            {
                HAL_TRACE_DEBUG0 ("NvErase done; assert  DeviceReset");
                nfc_hal_cb2.nverase = NFC_HAL_NVERASE_PRESERVE;
                if (nfc_hal_cb2.hal_init_ctrl & NFC_DEVICE_RESET)
                    USERIAL_DeviceReset();
                /* Wait for NFCC to enable - Core reset notification */
                NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_W4_NFCC_ENABLE);

                /* NFCC Enable timeout */
                nfc_hal_main_start_quick_timer (&nfc_hal_cb2.timer, NFC_HAL_TTYPE_NFCC_ENABLE,
                                                ((p_nfc_hal_cfg->nfc_hal_nfcc_enable_timeout)*QUICK_TIMER_TICKS_PER_SEC)/1000);
            }
            else
            {
                HAL_TRACE_DEBUG0 ("NvErase failed");
                NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_W4_HW_INFO);
                nfc_hal_dm_send_nci_cmd (nfc_hal_dm_get_hw_version_cmd, NCI_MSG_HDR_SIZE, NULL);
            }
#endif
        }
        else if( op_code == (NCI_RSP_BIT|NCI_MSG_OPTION_GET_VERSION) )
        {
            /* version info */
            UINT8 state;
            UINT8 fw_ver[3];
            UINT8 hw_ver[3];
            UINT32 fw_ver_u32;
            UINT32 hw_ver_u32;
            p++;
            STREAM_TO_UINT8 (state, p);
            STREAM_TO_ARRAY ( fw_ver, p, 3 );
            STREAM_TO_ARRAY ( hw_ver, p, 3 );

            fw_ver_u32 = ((UINT32 )fw_ver[2]<<16) | ((UINT32 )fw_ver[1]<<8) | (UINT32 )fw_ver[0];
            hw_ver_u32 = ((UINT32 )hw_ver[2]<<24) | ((UINT32 )hw_ver[1]<<16) | ((UINT32 )hw_ver[0]<<8);

            HAL_TRACE_DEBUG2("nfc_hal_dm_proc_msg_during_init(): %s/%s", NFC_LIB_REV, NFC_SVN_REV);

            if( !state )
            {
                HAL_TRACE_DEBUG3 ("OPTION_GET_VERSION: FW=%d(0x%06x)  HW[31:8]=0x%06x",
                                  fw_ver_u32, fw_ver_u32, hw_ver_u32>>8);
                nfc_hal_cb2.dev_cb.dev_hw_id = (hw_ver_u32 & 0xFFFFFF00);
            }
            else
            {
                nfc_hal_cb2.dev_cb.dev_hw_id = 0;
                HAL_TRACE_DEBUG1 ("OPTION_GET_VERSION: fail=0x%x", state);
            }
            if ( nfc_hal_cb2.dev_cb.initializing_state == NFC_HAL_INIT_STATE2_W4_HW_INFO )
            {
                NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_W4_BUILD_INFO);
                nfc_hal_dm_send_nci_cmd (nfc_hal_dm_get_int_version_cmd, NCI_MSG_HDR_SIZE, NULL);
            }
            else if (p_cback)
            {
                (*p_cback) ((tNFC_HAL_NCI_EVT) (op_code),
                            p_msg->len,
                            (UINT8 *) (p_msg + 1) + p_msg->offset);
            }
        }
        else if ( op_code == (NCI_RSP_BIT|NCI_MSG_OPTION_GET_INTERNAL_VERSION) )
        {
            /* version info */
            UINT8 state;
            UINT16 fw_int_ver;
            UINT8 hw_int_ver;
            p++;
            STREAM_TO_UINT8 (state, p);
            STREAM_TO_UINT16 ( fw_int_ver, p);
            STREAM_TO_UINT8 ( hw_int_ver, p );

            if( !state )
            {
                HAL_TRACE_DEBUG3 ("OPTION_GET_INTERANL_VERSION: FW_build=%d(0x%04x) HW[7:0]=0x%02x",
                                  fw_int_ver, fw_int_ver, hw_int_ver);
                nfc_hal_cb2.dev_cb.dev_hw_id &= (UINT32 )0xffffff00;
                nfc_hal_cb2.dev_cb.dev_hw_id |= ((UINT32 )hw_int_ver & 0xff);
                nfc_hal_cb2.dev_cb.dev_fw_build = fw_int_ver;
            }
            else
            {
                nfc_hal_cb2.dev_cb.dev_fw_build = 0;
                HAL_TRACE_DEBUG1 ("OPTION_GET_INTERANL_VERSION: fail=0x%x", state);
            }

            if ( nfc_hal_cb2.dev_cb.initializing_state == NFC_HAL_INIT_STATE2_W4_BUILD_INFO )
            {
                NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_W4_PATCH_INFO);
                nfc_hal_dm_send_nci_cmd (nfc_hal_dm_get_patch_version_cmd, NCI_MSG_HDR_SIZE, NULL);

                nfc_hal_cb2.nvm_cb.ver_hw = nfc_hal_cb2.dev_cb.dev_hw_id;
                nfc_hal_cb2.nvm_cb.ver_fw = fw_int_ver;
            }
            else if (p_cback)
            {
                /* update state */
                (*p_cback) ((tNFC_HAL_NCI_EVT) (op_code),
                            p_msg->len,
                            (UINT8 *) (p_msg + 1) + p_msg->offset);
            }
        }
        else if ( (op_code == (NCI_RSP_BIT|NCI_MSG_OPTION_PATCH_VERSION))
                 &&(nfc_hal_cb2.dev_cb.initializing_state == NFC_HAL_INIT_STATE2_W4_PATCH_INFO) )
        {
            UINT8 nvm_type, len, status=1;
            STREAM_TO_UINT8( len, p );
            if ( len > 0 )
                STREAM_TO_UINT8( status, p );
            if( status != NCI_STATUS_OK )
            {
                HAL_TRACE_DEBUG0 ("nfc_hal_dm_proc_msg_during_init(): OPTION_PATCH_VERSION failed");
                NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_IDLE);
                nfc_hal_main_pre_init_done (HAL_NFC_STATUS_OK);
                return;
            }

            STREAM_TO_UINT8( nvm_type, p );
            STREAM_TO_UINT16( nfc_hal_cb2.nvm_cb.ver_minor, p );
            STREAM_TO_UINT16( nfc_hal_cb2.nvm_cb.ver_major, p );
            if( nfc_hal_cb2.nvm_cb.ver_major == 0 && nfc_hal_cb2.nvm_cb.ver_minor == 0 )
            {
                /* no patch data in NVM */
                nfc_hal_cb2.nvm_cb.project_id = 0;
            }
            else
            {
                nfc_hal_cb2.nvm_cb.project_id = NFC_HAL_PROJECTID;
                nfc_hal_cb2.nvm_cb.flags |= NFC_HAL_NVM_FLAGS_PATCH_PRESENT;
            }
            NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_W4_APP_COMPLETE);
            /* let platform update baudrate or download patch */
            nfc_hal_post_reset_init (nfc_hal_cb2.dev_cb.dev_hw_id, nvm_type );
        }
        else if (nfc_hal_cb2.dev_cb.next_dm_config >= NFC_HAL_DM_CONFIG2_START_UP_RMW
                 && nfc_hal_cb2.dev_cb.next_dm_config <= NFC_HAL_DM_CONFIG2_START_UP_RMWPOST
                 && ((op_code == (NCI_RSP_BIT|NCI_MSG_OPTION_READ_ADR)) || (op_code == (NCI_RSP_BIT|NCI_MSG_OPTION_WRITE_ADR))))
        {
            UINT8 len, status=1;
            HAL_TRACE_DEBUG1 ("nfc_hal_dm_proc_msg_during_init(): rmw_rsp len=%d", (size_t )p[2]);
            STREAM_TO_UINT8( len, p );
            if (len > 0)
                STREAM_TO_UINT8( status, p );
            nfc_hal_cb2.rmw.status = status;
            if (len >= 5)
            {
                STREAM_TO_UINT32( nfc_hal_cb2.rmw.data, p );
            }

            if (p_cback)
            {
                (*p_cback) ((tNFC_HAL_NCI_EVT) (op_code),
                            p_msg->len,
                            (UINT8 *) (p_msg + 1) + p_msg->offset);
            }
            else
            {
                NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_IDLE);
                nfc_hal_cb2.p_stack_cback (HAL_NFC_POST_INIT_CPLT_EVT, HAL_NFC_STATUS_FAILED);
                return;
            }
        }
        else if (p_cback)
        {
            (*p_cback) ((tNFC_HAL_NCI_EVT) (op_code),
                        p_msg->len,
                        (UINT8 *) (p_msg + 1) + p_msg->offset);
        }
    }
#if (defined (NFC_HAL_DELAYED_NTF_INCLUDED) && NFC_HAL_DELAYED_NTF_INCLUDED==TRUE)
    else if ((gid == NCI_GID_RF_MANAGE) && (op_code == NCI_MSG_RF_EE_DISCOVERY_REQ) && (mt == NCI_MT_NTF) )
    {
        /* save EE_DISC_REQ_NTF */
        nfc_hal_main_delayed_pkt_queuing (p_msg);
    }
#endif /*(defined (NFC_HAL_DELAYED_NTF_INCLUDED) && NFC_HAL_DELAYED_NTF_INCLUDED==TRUE) */
}

/*******************************************************************************
**
** Function         nfc_hal_dm_send_nci_cmd
**
** Description      Send NCI command to NFCC while initializing NFCC
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_dm_send_nci_cmd (const UINT8 *p_data, UINT16 len, tNFC_HAL_NCI_CBACK *p_cback)
{
    NFC_HDR *p_buf;
    UINT8  *ps;

    HAL_TRACE_DEBUG1 ("nfc_hal_dm_send_nci_cmd (): nci_wait_rsp = 0x%x", nfc_hal_cb2.ncit_cb.nci_wait_rsp);

    if (nfc_hal_cb2.ncit_cb.nci_wait_rsp != NFC_HAL_WAIT_RSP_NONE)
    {
        HAL_TRACE_ERROR0 ("nfc_hal_dm_send_nci_cmd(): no command window");
        return;
    }

    if ((p_buf = (NFC_HDR *)GKIH_getpoolbuf (NFC_HAL_NCI_POOL_ID)) != NULL)
    {
        nfc_hal_cb2.ncit_cb.nci_wait_rsp = NFC_HAL_WAIT_RSP_VSC;

        p_buf->offset = NFC_HAL_NCI_MSG_OFFSET_SIZE;
        p_buf->event  = NFC_HAL_EVT_TO_NFC_NCI;
        p_buf->len    = len;

        memcpy ((UINT8*) (p_buf + 1) + p_buf->offset, p_data, len);

        /* Keep a copy of the command and send to NCI transport */

        /* save the message header to double check the response */
        ps   = (UINT8 *)(p_buf + 1) + p_buf->offset;
        memcpy(nfc_hal_cb2.ncit_cb.last_hdr, ps, NFC_HAL_SAVED_HDR_SIZE);
        memcpy(nfc_hal_cb2.ncit_cb.last_cmd, ps + NCI_MSG_HDR_SIZE, NFC_HAL_SAVED_CMD_SIZE);

        /* save the callback for NCI VSCs */
        nfc_hal_cb2.ncit_cb.p_vsc_cback = (void *)p_cback;

        nfc_hal_nci_send_cmd (p_buf);

        /* start NFC command-timeout timer */
	HAL_TRACE_DEBUG1 ("start timer: time=%d", ((UINT32) NFC_HAL_CMD_TOUT) * QUICK_TIMER_TICKS_PER_SEC / 1000);
        nfc_hal_main_start_quick_timer (&nfc_hal_cb2.ncit_cb.nci_wait_rsp_timer, (UINT16)(NFC_HAL_TTYPE_NCI_WAIT_RSP),
                                        ((UINT32) NFC_HAL_CMD_TOUT) * QUICK_TIMER_TICKS_PER_SEC / 1000);
    }
}

/*******************************************************************************
**
** Function         nfc_hal_dm_send_pend_cmd
**
** Description      Send a command to NFCC
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_dm_send_pend_cmd (void)
{
    NFC_HDR *p_buf = nfc_hal_cb2.ncit_cb.p_pend_cmd;
    UINT8  *p;

    if (p_buf == NULL)
        return;

    /* check low power mode state */
    if (!nfc_hal_dm_power_mode_execute (NFC_HAL_LP_TX_DATA_EVT2))
    {
        return;
    }

    if (nfc_hal_cb2.ncit_cb.nci_wait_rsp == NFC_HAL_WAIT_RSP_PROP)
    {
#if (NFC_HAL_TRACE_PROTOCOL == TRUE)
        DispHciCmd_Hal (p_buf);
#endif

        /* save the message header to double check the response */
        p = (UINT8 *)(p_buf + 1) + p_buf->offset;
        memcpy(nfc_hal_cb2.ncit_cb.last_hdr, p, NFC_HAL_SAVED_HDR_SIZE);

        /* add packet type for BT message */
        p_buf->offset--;
        p_buf->len++;

        p  = (UINT8 *) (p_buf + 1) + p_buf->offset;
        *p = HCIT_TYPE_COMMAND;

        USERIAL_Write (USERIAL_NFC_PORT, p, p_buf->len);

        GKIH_freebuf (p_buf);
        nfc_hal_cb2.ncit_cb.p_pend_cmd = NULL;

        /* start NFC command-timeout timer */
        nfc_hal_main_start_quick_timer (&nfc_hal_cb2.ncit_cb.nci_wait_rsp_timer, (UINT16)(NFC_HAL_TTYPE_NCI_WAIT_RSP),
                                        ((UINT32) NFC_HAL_CMD_TOUT) * QUICK_TIMER_TICKS_PER_SEC / 1000);

    }
}

/*******************************************************************************
**
** Function         nfc_hal_dm_set_nfc_wake
**
** Description      Set NFC_WAKE line
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_dm_set_nfc_wake (UINT8 cmd)
{
    HAL_TRACE_DEBUG1 ("nfc_hal_dm_set_nfc_wake () %s",
                      (cmd == NFC_HAL_ASSERT_NFC_WAKE ? "ASSERT" : "DEASSERT"));

    /*
    **  nfc_wake_active_mode             cmd              result of voltage on NFC_WAKE
    **
    **  NFC_HAL_LP_ACTIVE_LOW (0)    NFC_HAL_ASSERT_NFC_WAKE (0)    pull down NFC_WAKE (GND)
    **  NFC_HAL_LP_ACTIVE_LOW (0)    NFC_HAL_DEASSERT_NFC_WAKE (1)  pull up NFC_WAKE (VCC)
    **  NFC_HAL_LP_ACTIVE_HIGH (1)   NFC_HAL_ASSERT_NFC_WAKE (0)    pull up NFC_WAKE (VCC)
    **  NFC_HAL_LP_ACTIVE_HIGH (1)   NFC_HAL_DEASSERT_NFC_WAKE (1)  pull down NFC_WAKE (GND)
    */

    if (cmd == nfc_hal_cb2.dev_cb.nfc_wake_active_mode)
        UPIO_Set (UPIO_GENERAL, NFC_HAL_LP_NFC_WAKE_GPIO, UPIO_OFF); /* pull down NFC_WAKE */
    else
        UPIO_Set (UPIO_GENERAL, NFC_HAL_LP_NFC_WAKE_GPIO, UPIO_ON);  /* pull up NFC_WAKE */
}

/*******************************************************************************
**
** Function         nfc_hal_dm_power_mode_execute
**
** Description      If snooze mode is enabled in full power mode,
**                     Assert NFC_WAKE before sending data
**                     Deassert NFC_WAKE when idle timer expires
**
** Returns          TRUE if DH can send data to NFCC
**
*******************************************************************************/
BOOLEAN nfc_hal_dm_power_mode_execute (tNFC_HAL_LP_EVT event)
{
    BOOLEAN send_to_nfcc = FALSE;

    HAL_TRACE_DEBUG4 ("nfc_hal_dm_power_mode_execute () event = %d p=%d sz=%d kw=%d", event, nfc_hal_cb2.dev_cb.power_mode, nfc_hal_cb2.dev_cb.snooze_mode, nfc_hal_cb2.dev_cb.lp_keep_wake);

    if (nfc_hal_cb2.dev_cb.power_mode == NFC_HAL_POWER_MODE_FULL)
    {
        if (nfc_hal_cb2.dev_cb.snooze_mode != NFC_HAL_LP_SNOOZE_MODE_NONE)
        {
            /* SnoozeMode Ctrl from upper layer start */
            BOOLEAN is_transport_evt = FALSE;
            switch( event ){
            case NFC_HAL_LP_DEFAULT_EVT2 :
                nfc_hal_cb2.dev_cb.lp_keep_wake = FALSE;
                nfc_hal_cb2.dev_cb.lp_idle_timeout = nfc_hal_cb2.dev_cb.lp_idle_short_timeout;
                /* start or extend idle timer */
                nfc_hal_main_start_quick_timer (&nfc_hal_cb2.dev_cb.lp_timer, 0x00,
                                                nfc_hal_cb2.dev_cb.lp_idle_timeout * QUICK_TIMER_TICKS_PER_SEC / 1000);
                break;
            case NFC_HAL_LP_KEEP_WAKE_EVT2:
                nfc_hal_cb2.dev_cb.lp_keep_wake = TRUE;
                nfc_hal_dm_set_nfc_wake (NFC_HAL_ASSERT_NFC_WAKE);
                break;
            case NFC_HAL_LP_TIMER_FORCE_EXPIRE_EVT2:
                nfc_hal_cb2.dev_cb.lp_keep_wake = FALSE;
                nfc_hal_main_stop_quick_timer (&nfc_hal_cb2.dev_cb.lp_timer);
                nfc_hal_dm_set_nfc_wake (NFC_HAL_DEASSERT_NFC_WAKE);
                break;
            case NFC_HAL_LP_LONG_TIMER_EVT2 :
                nfc_hal_cb2.dev_cb.lp_keep_wake = FALSE;
                nfc_hal_cb2.dev_cb.lp_idle_timeout = nfc_hal_cb2.dev_cb.lp_idle_long_timeout;
                /* start or extend idle timer */
                nfc_hal_main_start_quick_timer (&nfc_hal_cb2.dev_cb.lp_timer, 0x00,
                                                nfc_hal_cb2.dev_cb.lp_idle_timeout * QUICK_TIMER_TICKS_PER_SEC / 1000);
                break;

            default:
                is_transport_evt = TRUE;
            }
            if( !is_transport_evt )
                return send_to_nfcc;
            /* SnoozeMode Ctrl from upper layer end */

            /* if any transport activity */
            if (  (event == NFC_HAL_LP_TX_DATA_EVT2)
                ||(event == NFC_HAL_LP_RX_DATA_EVT2)  )
            {
                /* if idle timer is not running */
                if (nfc_hal_cb2.dev_cb.lp_timer.in_use == FALSE)
                {
#if (NFC_HAL_UPIO_SET_READ == TRUE)
                    if(event == NFC_HAL_LP_TX_DATA_EVT2)
#endif
                    {
                        nfc_hal_dm_set_nfc_wake (NFC_HAL_ASSERT_NFC_WAKE);
                    }
                }

                /* start or extend idle timer */
                nfc_hal_main_start_quick_timer (&nfc_hal_cb2.dev_cb.lp_timer, 0x00,
                                                nfc_hal_cb2.dev_cb.lp_idle_timeout * QUICK_TIMER_TICKS_PER_SEC / 1000);
            }
            else if (event == NFC_HAL_LP_TIMEOUT_EVT2 && !nfc_hal_cb2.dev_cb.lp_keep_wake)
            {
                /* let NFCC go to snooze mode */
                nfc_hal_dm_set_nfc_wake (NFC_HAL_DEASSERT_NFC_WAKE);
            }
        }

        send_to_nfcc = TRUE;
    }

    return (send_to_nfcc);
}

/*******************************************************************************
**
** Function         nci_hal_lp_timeout_cback
**
** Description      callback function for low power timeout
**
** Returns          void
**
*******************************************************************************/
static void nci_hal_lp_timeout_cback (void *p_tle)
{
    HAL_TRACE_DEBUG0 ("nci_hal_lp_timeout_cback ()");

    nfc_hal_dm_power_mode_execute (NFC_HAL_LP_TIMEOUT_EVT2);
}

/*******************************************************************************
**
** Function         nfc_hal_dm_pre_init_nfcc
**
** Description      This function initializes Broadcom specific control blocks for
**                  NCI transport
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_dm_pre_init_nfcc (void)
{
    HAL_TRACE_DEBUG0 ("nfc_hal_dm_pre_init_nfcc ()");

    /* Send RESET CMD if application registered callback for device initialization */
    nfc_hal_dm_send_reset_cmd ();
}

/*******************************************************************************
**
** Function         nfc_hal_dm_shutting_down_nfcc
**
** Description      This function initializes Broadcom specific control blocks for
**                  NCI transport
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_dm_shutting_down_nfcc (void)
{
    HAL_TRACE_DEBUG0 ("nfc_hal_dm_shutting_down_nfcc ()");

    nfc_hal_cb2.dev_cb.initializing_state = NFC_HAL_INIT_STATE2_CLOSING;

    /* reset low power mode variables */
    if (  (nfc_hal_cb2.dev_cb.power_mode  == NFC_HAL_POWER_MODE_FULL)
        &&(nfc_hal_cb2.dev_cb.snooze_mode != NFC_HAL_LP_SNOOZE_MODE_NONE)  )
    {
        nfc_hal_dm_set_nfc_wake (NFC_HAL_ASSERT_NFC_WAKE);
    }

    nfc_hal_cb2.ncit_cb.nci_wait_rsp = NFC_HAL_WAIT_RSP_NONE;
    nfc_hal_cb2.dev_cb.power_mode  = NFC_HAL_POWER_MODE_FULL;
    nfc_hal_cb2.dev_cb.snooze_mode = NFC_HAL_LP_SNOOZE_MODE_NONE;

    /* Stop all timers */
    nfc_hal_main_stop_quick_timer (&nfc_hal_cb2.ncit_cb.nci_wait_rsp_timer);
    nfc_hal_main_stop_quick_timer (&nfc_hal_cb2.dev_cb.lp_timer);
    nfc_hal_main_stop_quick_timer (&nfc_hal_cb2.prm.timer);
}

/*******************************************************************************
**
** Function         nfc_hal_dm_init
**
** Description      This function initializes Broadcom specific control blocks for
**                  NCI transport
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_dm_init (void)
{
    HAL_TRACE_DEBUG0 ("nfc_hal_dm_init ()");

    nfc_hal_cb2.dev_cb.lp_timer.p_cback = nci_hal_lp_timeout_cback;

    nfc_hal_cb2.ncit_cb.nci_wait_rsp_timer.p_cback = nfc_hal_nci_cmd_timeout_cback;
}

/*******************************************************************************
**
** Function         HAL_NfcDevInitDone
**
** Description      Notify that pre-initialization of NFCC is complete
**
** Returns          void
**
*******************************************************************************/
void HAL_NfcPreInitDone (tHAL_NFC_STATUS status)
{
    HAL_TRACE_DEBUG1 ("HAL_NfcPreInitDone () status=%d", status);

    if (nfc_hal_cb2.dev_cb.initializing_state == NFC_HAL_INIT_STATE2_W4_APP_COMPLETE)
    {
        NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_IDLE);
        nfc_hal_main_send_devinfo(); /* report HW/FW/PATCH info to upper */

        nfc_hal_main_pre_init_done (status);
    }
}
/*******************************************************************************
**
** Function         HAL_NfcReInit
**
** Description      This function is called to restart initialization after REG_PU
**                  toggled because of failure to detect NVM type or download patchram.
**
** Note             This function should be called only during the HAL init process
**
** Returns          HAL_NFC_STATUS_OK if successfully initiated
**                  HAL_NFC_STATUS_FAILED otherwise
**
*******************************************************************************/
tHAL_NFC_STATUS HAL_NfcReInit (void)
{
    tHAL_NFC_STATUS status = HAL_NFC_STATUS_FAILED;

    HAL_TRACE_DEBUG1 ("HAL_NfcReInit () init st=0x%x", nfc_hal_cb2.dev_cb.initializing_state);
    if (nfc_hal_cb2.dev_cb.initializing_state == NFC_HAL_INIT_STATE2_W4_APP_COMPLETE)
    {
        {
            /* Wait for NFCC to enable - Core reset notification */
            NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_W4_NFCC_ENABLE);

            /* NFCC Enable timeout */
            nfc_hal_main_start_quick_timer (&nfc_hal_cb2.timer, NFC_HAL_TTYPE_NFCC_ENABLE,
                                            ((p_nfc_hal_cfg->nfc_hal_nfcc_enable_timeout)*QUICK_TIMER_TICKS_PER_SEC)/1000);
        }

        status = HAL_NFC_STATUS_OK;
    }
    return status;
}

/*******************************************************************************
**
** Function         nfc_hal_dm_set_snooze_mode_cback
**
** Description      This is snooze update complete callback.
**
** Returns          void
**
*******************************************************************************/
static void nfc_hal_dm_set_snooze_mode_cback (tNFC_HAL_BTVSC_CPLT *pData)
{
    UINT8             status = pData->p_param_buf[0];
    tHAL_NFC_STATUS   hal_status;
    tHAL_NFC_STATUS_CBACK *p_cback;

    /* if it is completed */
    if (status == HCI_SUCCESS)
    {
        /* update snooze mode */
        nfc_hal_cb2.dev_cb.snooze_mode = nfc_hal_cb2.dev_cb.new_snooze_mode;

        nfc_hal_dm_set_nfc_wake (NFC_HAL_ASSERT_NFC_WAKE);

        if ( nfc_hal_cb2.dev_cb.snooze_mode != NFC_HAL_LP_SNOOZE_MODE_NONE)
        {
            /* start idle timer */
            nfc_hal_main_start_quick_timer (&nfc_hal_cb2.dev_cb.lp_timer, 0x00,
                                            nfc_hal_cb2.dev_cb.lp_idle_timeout * QUICK_TIMER_TICKS_PER_SEC / 1000);
        }
        else
        {
            nfc_hal_main_stop_quick_timer (&nfc_hal_cb2.dev_cb.lp_timer);
        }
        hal_status = HAL_NFC_STATUS_OK;
    }
    else
    {
        hal_status = HAL_NFC_STATUS_FAILED;
    }

    if (nfc_hal_cb2.dev_cb.p_prop_cback)
    {
        p_cback = nfc_hal_cb2.dev_cb.p_prop_cback;
        nfc_hal_cb2.dev_cb.p_prop_cback = NULL;
        (*p_cback) (hal_status);
    }
}

/*******************************************************************************
**
** Function         HAL_NfcSetSnoozeMode
**
** Description      Set snooze mode
**                  snooze_mode
**                      NFC_HAL_LP_SNOOZE_MODE_NONE - Snooze mode disabled
**                      NFC_HAL_LP_SNOOZE_MODE_UART - Snooze mode for UART
**                      NFC_HAL_LP_SNOOZE_MODE_SPI_I2C - Snooze mode for SPI/I2C
**
**                  idle_threshold_dh/idle_threshold_nfcc
**                      Idle Threshold Host in 100ms unit
**
**                  nfc_wake_active_mode/dh_wake_active_mode
**                      NFC_HAL_LP_ACTIVE_LOW - high to low voltage is asserting
**                      NFC_HAL_LP_ACTIVE_HIGH - low to high voltage is asserting
**
**                  p_snooze_cback
**                      Notify status of operation
**
** Returns          tHAL_NFC_STATUS
**
*******************************************************************************/
tHAL_NFC_STATUS HAL_NfcSetSnoozeMode (UINT8 snooze_mode,
                                      UINT8 idle_threshold_dh,
                                      UINT8 idle_threshold_nfcc,
                                      UINT8 nfc_wake_active_mode,
                                      UINT8 dh_wake_active_mode,
                                      tHAL_NFC_STATUS_CBACK *p_snooze_cback)
{
    tNFC_HAL_BTVSC_CPLT data;
    UINT8 buf[1];
    unsigned int num;

    /* Note:
     * No argument changes form orignal code,
     * but in CXD224x implementation,
     * no need to send NCI command,
     * don't use idle_threshold_dh, idle_threshold_nfcc, dh_wake_active_mode
     * call dummy completion callback for compatiblity
     */

    HAL_TRACE_API1 ("HAL_NfcSetSnoozeMode (): snooze_mode = %d", snooze_mode);

    nfc_hal_cb2.dev_cb.new_snooze_mode      = snooze_mode;
    nfc_hal_cb2.dev_cb.nfc_wake_active_mode = nfc_wake_active_mode;
    nfc_hal_cb2.dev_cb.p_prop_cback         = p_snooze_cback;

    if ( GetNumValue ( NAME_HAL_LP_IDLE_TIMEOUT, &num, sizeof ( num ) ) && num != 0 )
    {
        nfc_hal_cb2.dev_cb.lp_idle_short_timeout = num;
    }
    else
    {
#ifndef SPZ_IMPL
        nfc_hal_cb2.dev_cb.lp_idle_short_timeout = NFC_HAL_LP_IDLE_TIMEOUT ;
#else
        nfc_hal_cb2.dev_cb.lp_idle_short_timeout = 200;
#endif  /* SPZ_IMPL */
    }

    if ( GetNumValue ( NAME_HAL_LP_IDLE_LONG_TIMEOUT, &num, sizeof ( num ) ) && num != 0 )
    {
        nfc_hal_cb2.dev_cb.lp_idle_long_timeout = num;
    }
    else
    {
#ifndef SPZ_IMPL
        nfc_hal_cb2.dev_cb.lp_idle_long_timeout = NFC_HAL_LP_IDLE_LONG_TIMEOUT;
#else
        nfc_hal_cb2.dev_cb.lp_idle_long_timeout = 2000;
#endif /* SPZ_IMPL */
    }

    nfc_hal_cb2.dev_cb.lp_idle_timeout = nfc_hal_cb2.dev_cb.lp_idle_short_timeout;

    HAL_TRACE_API3 ("HAL_NfcSetSnoozeMode () short=%d long=%d actual=%d",
        nfc_hal_cb2.dev_cb.lp_idle_short_timeout, nfc_hal_cb2.dev_cb.lp_idle_long_timeout, nfc_hal_cb2.dev_cb.lp_idle_timeout);

    /* do not send NCI commnad, dummy cback */
    buf[0] = HCI_SUCCESS;
    data.p_param_buf = &buf[0];

    nfc_hal_dm_set_snooze_mode_cback (&data);

    return (NCI_STATUS_OK);
}
