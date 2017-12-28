/******************************************************************************
 *
 *  Copyright (C) 2010-2013 Broadcom Corporation
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
 *  Functions for handling NFC HAL NCI Transport events
 *
 ******************************************************************************/
#include <string.h>
#include "nfc_hal_api.h"
#include "nfc_hal_int_extra.h"
#include "nfc_hal_post_reset.h"
#include "userial.h"
#include "upio.h"
#include "config.h"
#include "nfc_types_extra.h"
/****************************************************************************
** Definitions
****************************************************************************/

/* Default NFC HAL NCI port configuration  */
NFC_HAL_TRANS_CFG_QUALIFIER tNFC_HAL_TRANS_CFG nfc_hal_trans_cfg =
{
    NFC_HAL_SHARED_TRANSPORT_ENABLED,   /* bSharedTransport */
    USERIAL_BAUD_115200,                /* Baud rate */
    USERIAL_FC_HW                       /* Flow control */
};

/* Control block for NFC HAL NCI transport */
#if NFC_DYNAMIC_MEMORY == FALSE
tNFC_HAL_CB2 nfc_hal_cb2;
#else
#include <stdlib.h>
tNFC_HAL_CB2 *nfc_hal_cb2_ptr=NULL;
#endif

extern tNFC_HAL_CFG *p_nfc_hal_cfg;
/****************************************************************************
** Internal function prototypes
****************************************************************************/
static void nfc_hal_main_userial_cback (tUSERIAL_PORT port, tUSERIAL_EVT evt, tUSERIAL_EVT_DATA *p_data);
static void nfc_hal_main_handle_terminate (void);
static void nfc_hal_main_timeout_cback (void *p_tle);

#if (NFC_HAL_DEBUG == TRUE)
const char * const nfc_hal_init_state_str[] =
{
    "IDLE",             /* Initialization is done                */
    "W4_RESET",         /* Waiting for reset rsp                 */
    "W4_HW_INFO",       /* Waiting for hardware info rsp         */
    "W4_NFCC_ENABLE",   /* Waiting for reset ntf atter REG_PU up */
    "W4_BUILD_INFO",    /* Waiting for build info rsp            */
    "W4_PATCH_INFO",    /* Waiting for patch info rsp            */
    "W4_APP_COMPL",     /* Waiting for complete from application */
    "W4_POST_INIT",     /* Waiting for complete of post init     */
    "W4_CONTROL",       /* Waiting for control release           */
    "W4_PREDISC",       /* Waiting for complete of prediscover   */
    "W4_PRECLOSE",      /* Waiting for pre closing               */
    "CLOSING",          /* Shutting down                         */
    "W4_NVERASE",       /* Waiting for NvErase */
};
#endif

/*******************************************************************************
**
** Function         nfc_hal_main_init
**
** Description      This function initializes control block for NFC HAL
**
** Returns          nothing
**
*******************************************************************************/
void nfc_hal_main_init (void)
{

#if NFC_DYNAMIC_MEMORY == TRUE
#include <assert.h>
    nfc_hal_cb2_ptr = (tNFC_HAL_CB2 *)GKIH_os_malloc( sizeof(tNFC_HAL_CB2) );
    assert (nfc_hal_cb2_ptr);
#endif

    /* Clear control block */
    memset (&nfc_hal_cb2, 0, sizeof (tNFC_HAL_CB2));

    nfc_hal_cb2.ncit_cb.nci_ctrl_size   = NFC_HAL_NCI_INIT_CTRL_PAYLOAD_SIZE;
    nfc_hal_cb2.trace_level             = NFC_HAL_INITIAL_TRACE_LEVEL;
    nfc_hal_cb2.timer.p_cback           = nfc_hal_main_timeout_cback;
}

/*******************************************************************************
**
** Function         nfc_hal_main_destroy
**
** Description      This function destroys allocated memory of control block for NFC HAL
**
** Returns          nothing
**
*******************************************************************************/
void nfc_hal_main_destroy (void)
{
#if NFC_DYNAMIC_MEMORY == TRUE
    if (nfc_hal_cb2_ptr)
        GKIH_os_free (nfc_hal_cb2_ptr);
    nfc_hal_cb2_ptr = NULL;
#endif
}

/*******************************************************************************
**
** Function         nfc_hal_main_open_transport
**
** Description      Open transport and prepare for new incoming message;
**
** Returns          nothing
**
*******************************************************************************/
static void nfc_hal_main_open_transport (void)
{
    tUSERIAL_OPEN_CFG open_cfg;

    /* Initialize control block */
    nfc_hal_cb2.ncit_cb.rcv_state = NFC_HAL_RCV_IDLE_ST; /* to process packet type */

    if (nfc_hal_cb2.ncit_cb.p_rcv_msg)
    {
        GKIH_freebuf (nfc_hal_cb2.ncit_cb.p_rcv_msg);
        nfc_hal_cb2.ncit_cb.p_rcv_msg = NULL;
    }

    /* open transport */
    open_cfg.fmt    = (USERIAL_DATABITS_8 | USERIAL_PARITY_NONE | USERIAL_STOPBITS_1);
    open_cfg.baud   = nfc_hal_trans_cfg.userial_baud;
    open_cfg.fc     = nfc_hal_trans_cfg.userial_fc;
    open_cfg.buf    = USERIAL_BUF_BYTE;

    USERIAL_Open (USERIAL_NFC_PORT, &open_cfg, nfc_hal_main_userial_cback);

    {
        /* Wait for NFCC to enable - Core reset notification */
        NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_W4_NFCC_ENABLE);

        /* NFCC Enable timeout */
        nfc_hal_main_start_quick_timer (&nfc_hal_cb2.timer, NFC_HAL_TTYPE_NFCC_ENABLE,
                                        ((p_nfc_hal_cfg->nfc_hal_nfcc_enable_timeout)*QUICK_TIMER_TICKS_PER_SEC)/1000);
    }
}

/*******************************************************************************
**
** Function         nfa_hal_pre_discover_done_cback
**
** Description      Pre-discovery CFG is sent.
**
** Returns          nothing
**
*******************************************************************************/
void nfa_hal_pre_discover_done_cback (tNFC_HAL_NCI_EVT event, UINT16 data_len, UINT8 *p_data)
{
    NFC_HAL_SET_INIT_STATE2(NFC_HAL_INIT_STATE2_IDLE);
    nfc_hal_main_stop_quick_timer (&nfc_hal_cb2.ncit_cb.nci_wait_rsp_timer);
    nfc_hal_cb2.p_stack_cback (HAL_NFC_PRE_DISCOVER_CPLT_EVT, HAL_NFC_STATUS_OK);
}

/*******************************************************************************
**
** Function         nfa_hal_send_pre_discover_cfg
**
** Description      sending Pre-discovery CFG
**
** Returns          nothing
**
*******************************************************************************/
void nfa_hal_send_pre_discover_cfg (void)
{
    if (nfc_hal_dm_set_config (p_nfc_hal_pre_discover_cfg [0],
                               &p_nfc_hal_pre_discover_cfg[1],
                                nfa_hal_pre_discover_done_cback) != HAL_NFC_STATUS_OK)
    {
        nfa_hal_pre_discover_done_cback(0, 0, NULL);
    }
}

/*******************************************************************************
**
** Function         nfc_hal_main_send_error
**
** Description      send an Error event to NFC stack
**
** Returns          nothing
**
*******************************************************************************/
void nfc_hal_main_send_error (tHAL_NFC_STATUS status)
{
    /* Notify stack */
    nfc_hal_cb2.p_stack_cback(HAL_NFC_ERROR_EVT, status);
}

/*******************************************************************************
**
** Function         nfc_hal_main_userial_cback
**
** Description      USERIAL callback for NCI transport
**
** Returns          nothing
**
*******************************************************************************/
static void nfc_hal_main_userial_cback (tUSERIAL_PORT port, tUSERIAL_EVT evt, tUSERIAL_EVT_DATA *p_data)
{
    if (evt == USERIAL_RX_READY_EVT)
    {
       HAL_TRACE_DEBUG0 ("nfc_hal_main_userial_cback: USERIAL_RX_READY_EVT");

        /* Notify transport task of serial port event */
        GKIH_send_event (NFC_HAL_TASK, NFC_HAL_TASK_EVT_DATA_RDY);
    }
    else if (evt == USERIAL_TX_DONE_EVT)
    {
        /* Serial driver has finshed sending data from USERIAL_Write */
        /* Currently, no action is needed for this event */
    }
    else if (evt == USERIAL_ERR_EVT)
    {
        HAL_TRACE_ERROR0 ("nfc_hal_main_userial_cback: USERIAL_ERR_EVT. Notifying NFC_TASK of transport error");
        if (nfc_hal_cb2.ncit_cb.nci_wait_rsp != NFC_HAL_WAIT_RSP_NONE)
        {
            nfc_hal_main_stop_quick_timer (&nfc_hal_cb2.ncit_cb.nci_wait_rsp_timer);
            nfc_hal_nci_cmd_timeout_cback ((void *)&nfc_hal_cb2.ncit_cb.nci_wait_rsp_timer);
        }
        else
        {
            nfc_hal_main_send_error (HAL_NFC_STATUS_ERR_TRANSPORT);
        }
    }
    else if (evt == USERIAL_WAKEUP_EVT)
    {
        HAL_TRACE_DEBUG1 ("nfc_hal_main_userial_cback: USERIAL_WAKEUP_EVT: %d", p_data->sigs);
    }
    else
    {
        HAL_TRACE_DEBUG1 ("nfc_hal_main_userial_cback: unhandled userial evt: %i", evt);
    }
}

/*******************************************************************************
**
** Function         nfc_hal_main_pre_init_done
**
** Description      notify complete of pre-initialization
**
** Returns          nothing
**
*******************************************************************************/
void nfc_hal_main_pre_init_done (tHAL_NFC_STATUS status)
{
    HAL_TRACE_DEBUG1 ("nfc_hal_main_pre_init_done () status = %d", status);

    if (status != HAL_NFC_STATUS_OK)
    {
        nfc_hal_main_handle_terminate ();

        /* Close uart */
        USERIAL_Close (USERIAL_NFC_PORT);
    }

    /* Notify NFC Task the status of initialization */
    nfc_hal_cb2.p_stack_cback (HAL_NFC_OPEN_CPLT_EVT, status);
}

/*******************************************************************************
**
** Function         nfc_hal_main_timeout_cback
**
** Description      callback function for timeout
**
** Returns          void
**
*******************************************************************************/
static void nfc_hal_main_timeout_cback (void *p_tle)
{
    TIMER_LIST_ENT  *p_tlent = (TIMER_LIST_ENT *) p_tle;

    HAL_TRACE_DEBUG0 ("nfc_hal_main_timeout_cback ()");

    switch (p_tlent->event)
    {
    case NFC_HAL_TTYPE_POWER_CYCLE:
        nfc_hal_main_open_transport ();
        break;

    case NFC_HAL_TTYPE_NFCC_ENABLE:
        /* NFCC should have enabled now, notify transport openned */
        nfc_hal_dm_pre_init_nfcc ();
        break;

    default:
        HAL_TRACE_DEBUG1 ("nfc_hal_main_timeout_cback: unhandled timer event (0x%04x)", p_tlent->event);
        break;
    }
}

/*******************************************************************************
**
** Function         nfc_hal_main_handle_terminate
**
** Description      Handle NFI transport shutdown
**
** Returns          nothing
**
*******************************************************************************/
static void nfc_hal_main_handle_terminate (void)
{
    NFC_HDR *p_msg;

    /* dequeue and free buffer */
    if (nfc_hal_cb2.ncit_cb.p_pend_cmd != NULL)
    {
        GKIH_freebuf (nfc_hal_cb2.ncit_cb.p_pend_cmd);
        nfc_hal_cb2.ncit_cb.p_pend_cmd = NULL;
    }

    /* Free unsent nfc rx buffer */
    if (nfc_hal_cb2.ncit_cb.p_rcv_msg)
    {
        GKIH_freebuf (nfc_hal_cb2.ncit_cb.p_rcv_msg);
        nfc_hal_cb2.ncit_cb.p_rcv_msg  = NULL;
    }

    /* Free buffer for pending fragmented response/notification */
    if (nfc_hal_cb2.ncit_cb.p_frag_msg)
    {
        GKIH_freebuf (nfc_hal_cb2.ncit_cb.p_frag_msg);
        nfc_hal_cb2.ncit_cb.p_frag_msg = NULL;
    }

    /* Free buffer for pending prediscover NCI message */
    if (nfc_hal_cb2.prediscover_msg)
    {
        GKIH_freebuf (nfc_hal_cb2.prediscover_msg);
        nfc_hal_cb2.prediscover_msg = NULL;
    }

    /* Free buffers in the tx mbox */
    while ((p_msg = (NFC_HDR *) GKIH_read_mbox (NFC_HAL_TASK_MBOX)) != NULL)
    {
        GKIH_freebuf (p_msg);
    }

    /* notify closing transport */
    nfc_hal_dm_shutting_down_nfcc ();
}

/*******************************************************************************
**
** Function         nfc_hal_main_start_quick_timer
**
** Description      Start a timer for the specified amount of time.
**                  NOTE: The timeout resolution depends on including modules.
**                  QUICK_TIMER_TICKS_PER_SEC should be used to convert from
**                  time to ticks.
**
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_main_start_quick_timer (TIMER_LIST_ENT *p_tle, UINT16 type, UINT32 timeout)
{
    NFC_HDR *p_msg;

    /* if timer list is currently empty, start periodic GKI timer */
    if (nfc_hal_cb2.quick_timer_queue.p_first == NULL)
    {
        /* if timer starts on other than NCIT task (script wrapper) */
        if(GKIH_get_taskid () != NFC_HAL_TASK)
        {
            /* post event to start timer in NCIT task */
            if ((p_msg = (NFC_HDR *) GKIH_getbuf (NFC_HDR_SIZE)) != NULL)
            {
                p_msg->event = NFC_HAL_EVT_TO_START_QUICK_TIMER;
                GKIH_send_msg (NFC_HAL_TASK, NFC_HAL_TASK_MBOX, p_msg);
            }
        }
        else
        {
            GKIH_start_timer (NFC_HAL_QUICK_TIMER_ID, ((GKI_SECS_TO_TICKS (1) / QUICK_TIMER_TICKS_PER_SEC)), TRUE);
        }
    }

    GKIH_remove_from_timer_list (&nfc_hal_cb2.quick_timer_queue, p_tle);

    p_tle->event = type;
    p_tle->ticks = timeout; /* Save the number of ticks for the timer */

    GKIH_add_to_timer_list (&nfc_hal_cb2.quick_timer_queue, p_tle);
}

/*******************************************************************************
**
** Function         nfc_hal_main_stop_quick_timer
**
** Description      Stop a timer.
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_main_stop_quick_timer (TIMER_LIST_ENT *p_tle)
{
    GKIH_remove_from_timer_list (&nfc_hal_cb2.quick_timer_queue, p_tle);

    /* if timer list is empty stop periodic GKI timer */
    if (nfc_hal_cb2.quick_timer_queue.p_first == NULL)
    {
        GKIH_stop_timer (NFC_HAL_QUICK_TIMER_ID);
    }
}

/*******************************************************************************
**
** Function         nfc_hal_main_process_quick_timer_evt
**
** Description      Process quick timer event
**
** Returns          void
**
*******************************************************************************/
static void nfc_hal_main_process_quick_timer_evt (void)
{
    TIMER_LIST_ENT  *p_tle;

    GKIH_update_timer_list (&nfc_hal_cb2.quick_timer_queue, 1);

    while ((nfc_hal_cb2.quick_timer_queue.p_first) && (!nfc_hal_cb2.quick_timer_queue.p_first->ticks))
    {
        p_tle = nfc_hal_cb2.quick_timer_queue.p_first;
        GKIH_remove_from_timer_list (&nfc_hal_cb2.quick_timer_queue, p_tle);

        if (p_tle->p_cback)
        {
            (*p_tle->p_cback) (p_tle);
        }
    }

    /* if timer list is empty stop periodic GKI timer */
    if (nfc_hal_cb2.quick_timer_queue.p_first == NULL)
    {
        GKIH_stop_timer (NFC_HAL_QUICK_TIMER_ID);
    }
}

/*******************************************************************************
**
** Function         nfc_hal_send_nci_msg_to_nfc_task
**
** Description      This function is called to send nci message to nfc task
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_send_nci_msg_to_nfc_task (NFC_HDR * p_msg)
{
#ifdef NFC_HAL_SHARED_GKI
    /* Using shared NFC/HAL GKI resources - send message buffer directly to NFC_TASK for processing */
    p_msg->event = BT_EVT_TO_NFC_NCI;
    GKIH_send_msg (NFC_TASK, NFC_MBOX_ID, p_msg);
#else
    /* Send NCI message to the stack */
    nfc_hal_cb2.p_data_cback (p_msg->len, (UINT8 *) ((p_msg + 1)
                                 + p_msg->offset));
    GKIH_freebuf(p_msg);
#endif
}

/*******************************************************************************
**
** Function         nfc_hal_send_credit_ntf_for_cid
**
** Description      This function is called to send credit ntf
**                  for the specified connection id to nfc task
**
** Returns          void
**
*******************************************************************************/
static void nfc_hal_send_credit_ntf_for_cid (UINT8 cid)
{
    NFC_HDR  *p_msg;
    UINT8    *p, *ps;

    /* Start of new message. Allocate a buffer for message */
    if ((p_msg = (NFC_HDR *) GKIH_getpoolbuf (NFC_HAL_NCI_POOL_ID)) != NULL)
    {
        /* Initialize NFC_HDR */
        p_msg->len    = NCI_DATA_HDR_SIZE + 0x03;
        p_msg->event  = 0;
        p_msg->offset = 0;
        p_msg->layer_specific = 0;

        p = (UINT8 *) (p_msg + 1) + p_msg->offset;
        ps = p;
        NCI_MSG_BLD_HDR0(p, NCI_MT_NTF, NCI_GID_CORE);
        NCI_MSG_BLD_HDR1(p, NCI_MSG_CORE_CONN_CREDITS);
        UINT8_TO_STREAM (p, 0x03);

        /* Number of credit entries */
        *p++ = 0x01;
        /* Connection id of the credit ntf */
        *p++ = cid;
        /* Number of credits */
        *p = 0x01;
#ifdef DISP_NCI
        DISP_NCI (ps, (UINT16) p_msg->len, TRUE);
#endif
        nfc_hal_send_nci_msg_to_nfc_task (p_msg);
    }
    else
    {
        HAL_TRACE_ERROR0 ("Unable to allocate buffer for Sending credit ntf to stack");
    }
}

/*******************************************************************************
**
** Function         nfc_hal_main_send_message
**
** Description      This function is calledto send an NCI message.
**
** Returns          void
**
*******************************************************************************/
static void nfc_hal_main_send_message (NFC_HDR *p_msg)
{
    UINT8   *ps, *pp, cid, pbf;
    UINT16  len = p_msg->len;
    UINT16  data_len;
#ifdef DISP_NCI
    UINT8   delta;
#endif

    HAL_TRACE_DEBUG1 ("nfc_hal_main_send_message() ls:0x%x", p_msg->layer_specific);
    if (  (p_msg->layer_specific == NFC_HAL_WAIT_RSP_CMD)
        ||(p_msg->layer_specific == NFC_HAL_WAIT_RSP_VSC)  )
    {
        nfc_hal_nci_send_cmd (p_msg);
    }
    else
    {
        /* NFC task has fragmented the data packet to the appropriate size
         * and data credit is available; just send it */

        /* send this packet to transport */
        ps = (UINT8 *) (p_msg + 1) + p_msg->offset;
        pp = ps + 1;
#ifdef DISP_NCI
        delta = p_msg->len - len;
        DISP_NCI (ps + delta, (UINT16) (p_msg->len - delta), FALSE);
#endif

        /* check low power mode state */
        if (nfc_hal_dm_power_mode_execute (NFC_HAL_LP_TX_DATA_EVT2))
        {
            USERIAL_Write (USERIAL_NFC_PORT, ps, p_msg->len);
        }
        else
        {
            HAL_TRACE_ERROR0 ("nfc_hal_main_send_message(): drop data in low power mode");
        }
        GKIH_freebuf (p_msg);
    }
}

/*******************************************************************************
**
** Function         nfc_hal_main_reg_prediscover_cmd
**
** Description      Register Pre Discover command, this registered cmd will be remove
**                  after command send.
**
** Returns          fail none zero
**
*******************************************************************************/
static INT32 nfc_hal_main_reg_prediscover_cmd( UINT16 data_len, UINT8 *p_data )
{
    NFC_HDR *p_msg;
    UINT8 mt;

    if( data_len == 0 || p_data == NULL )
    {
        return -1;
    }
    HAL_TRACE_DEBUG2 ("nfc_hal_main_reg_prediscover_cmd (): data_len=%d p_data[]={0x%x,..}", data_len, *p_data);

    if (!nfc_hal_cb2.prediscover_msg)
    {
        if( (nfc_hal_cb2.prediscover_msg = (NFC_HDR *)GKIH_getpoolbuf (NFC_HAL_NCI_POOL_ID)) == NULL)
            return -1;
    }
    p_msg = nfc_hal_cb2.prediscover_msg;

    p_msg->event  = NFC_HAL_EVT_TO_NFC_NCI;
    p_msg->offset = NFC_HAL_NCI_MSG_OFFSET_SIZE;
    p_msg->len    = data_len;
    memcpy ((UINT8 *)(p_msg+1) + p_msg->offset, p_data, data_len);

    /* Check if message is a command or data */
    mt = (*(p_data) & NCI_MT_MASK) >> NCI_MT_SHIFT;
    p_msg->layer_specific = (mt == NCI_MT_CMD) ? NFC_HAL_WAIT_RSP_CMD : 0;

    return 0;
}

static UINT8 *nfc_hal_main_add_devinfo (UINT8 *msg, UINT8 id, UINT8 sz, UINT32 data)
{
    if (sz != 1 && sz != 2 && sz != 4) return msg;

    *msg++ = id;
    while (sz-- > 0)
    {
        *msg++ = (UINT8 )(data & 0xFF);
        data = data >> 8;
    }
    return msg;
}

#define NFC_HAL_DEVINFO_MAX   64
/******************************************************************************
***
** Function         nfc_hal_main_send_devinfo
**
** Description      This function is called to send device info to nfc task
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_main_send_devinfo(void)
{
    UINT8 msg_data[NFC_HAL_DEVINFO_MAX];
    UINT8 *msg = &msg_data[0];
    UINT8 len=0;

    *msg++ = (NCI_MTS_HALMSG|NCI_GID_PROP);
    *msg++ = NFC_HAL_GET_DEVICE_INFO;
    msg++; /* len */

    len += HAL_NFC_GET_DEVICE_INFO_ID_HWID_SZ + 1;
    msg = nfc_hal_main_add_devinfo (msg,
                                    HAL_NFC_GET_DEVICE_INFO_ID_HWID,
                                    HAL_NFC_GET_DEVICE_INFO_ID_HWID_SZ,
                                    nfc_hal_cb2.dev_cb.dev_hw_id);

    len += HAL_NFC_GET_DEVICE_INFO_ID_FWVER_SZ + 1;
    msg = nfc_hal_main_add_devinfo (msg,
                                    HAL_NFC_GET_DEVICE_INFO_ID_FWVER,
                                    HAL_NFC_GET_DEVICE_INFO_ID_FWVER_SZ,
                                    nfc_hal_cb2.dev_cb.dev_fw_build);

    len += HAL_NFC_GET_DEVICE_INFO_ID_PATCHVER_SZ + 1;
    msg = nfc_hal_main_add_devinfo (msg,
                                    HAL_NFC_GET_DEVICE_INFO_ID_PATCHVER,
                                    HAL_NFC_GET_DEVICE_INFO_ID_PATCHVER_SZ,
                                    (((UINT32 )nfc_hal_cb2.nvm_cb.ver_major)<<16)|((UINT32 )nfc_hal_cb2.nvm_cb.ver_minor));

    len++; /* status */

    if (nfc_hal_cb2.dev_cb.dev_hw_id && len < NFC_HAL_DEVINFO_MAX)
    {
        msg_data[2] = len;
        *msg = HAL_NFC_STATUS_OK;
    }
    else
    {
        len = 1;
        msg_data[3] = HAL_NFC_STATUS_FAILED; /* not set dev_hw_id */
    }

    if (nfc_hal_cb2.p_data_cback)
    {
        nfc_hal_cb2.p_data_cback (len + 3, msg_data);
    }
}

#if (defined (NFC_HAL_DELAYED_NTF_INCLUDED) && NFC_HAL_DELAYED_NTF_INCLUDED==TRUE)
/******************************************************************************
***
** Function         nfc_hal_main_delayed_pkt_init
**
** Description      This function is called to clear delayed NTF struct
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_main_delayed_pkt_init(void)
{
    nfc_hal_cb2.dly_ntf.cur = 0;
    nfc_hal_cb2.dly_ntf.state = NFC_HAL_DM_DLYNTF_STATE_QUEUING;
}

/******************************************************************************
***
** Function         nfc_hal_main_delayed_pkt_drain
**
** Description      This function is called to drain saved pakets
**                  to upper layer
**
** Returns          void
**
*******************************************************************************/
static INT32 nfc_hal_main_delayed_pkt_drain()
{
    NFC_HDR  *p_msg;
    UINT8 *p_pkt=&nfc_hal_cb2.dly_ntf.pkt[0];
    UINT8 total_len=0;

    HAL_TRACE_DEBUG2 ("nfc_hal_main_delayed_pkt_drain (): state:%d len:%d",
                      nfc_hal_cb2.dly_ntf.state, nfc_hal_cb2.dly_ntf.cur);

/* NFC_HAL_REQ_DELAYED_PKTS_DRAIN */
    if (nfc_hal_cb2.dly_ntf.state == NFC_HAL_DM_DLYNTF_STATE_CLEARED)
        return HAL_NFC_STATUS_FAILED;

    nfc_hal_cb2.dly_ntf.state = NFC_HAL_DM_DLYNTF_STATE_CLEARED;
    /* Start of new message. Allocate a buffer for message */
    while ((UINT8 *)(p_pkt + 1) < &nfc_hal_cb2.dly_ntf.pkt[nfc_hal_cb2.dly_ntf.cur])
    {
        UINT8 pkt_len;
        UINT8 *p;
        pkt_len = *p_pkt++;
        if ((p_msg = (NFC_HDR *) GKIH_getpoolbuf (NFC_HAL_NCI_POOL_ID)) != NULL)
        {
            p_msg->len    = pkt_len;
            p_msg->event  = 0;
            p_msg->offset = 0;
            p_msg->layer_specific = 0;

            p = (UINT8 *) (p_msg + 1) + p_msg->offset;
            memcpy( p, p_pkt, pkt_len );
            p_pkt += pkt_len;
#ifdef DISP_NCI
            DISP_NCI (p, (UINT16) p_msg->len, TRUE);
#endif
            nfc_hal_send_nci_msg_to_nfc_task (p_msg);
        }
        else
        {
            return HAL_NFC_STATUS_FAILED;
        }
    }
    return HAL_NFC_STATUS_OK;
}
/******************************************************************************
***
** Function         nfc_hal_main_delayed_pkt_drain
**
** Description      This function is called to drain saved pakets
**                  to upper layer
**
** Returns          void
**
*******************************************************************************/
static INT32 nfc_hal_main_delayed_pkt_ctrl(UINT8 cmd)
{
    INT32 ret=HAL_NFC_STATUS_FAILED ;
    switch(cmd)
    {
    case NFC_HAL_REQ_DELAYED_PKTS_CLEAR:
        nfc_hal_cb2.dly_ntf.state = NFC_HAL_DM_DLYNTF_STATE_CLEARED;
        ret = HAL_NFC_STATUS_OK;
        break;
    case NFC_HAL_REQ_DELAYED_PKTS_STOP:
        if (nfc_hal_cb2.dly_ntf.state == NFC_HAL_DM_DLYNTF_STATE_QUEUING)
        {
            nfc_hal_cb2.dly_ntf.state = NFC_HAL_DM_DLYNTF_STATE_STOPED;
            ret = HAL_NFC_STATUS_OK;
        }
        break;
    case NFC_HAL_REQ_DELAYED_PKTS_DRAIN:
        return nfc_hal_main_delayed_pkt_drain();
        break;
    default:
        ; /* error */
    }
    return ret;
}

/*******************************************************************************
**
** Function         nfc_hal_main_delayed_pkt_queuing
**
** Description      Save NCI NTF message for deleyed NTF report
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_main_delayed_pkt_queuing (NFC_HDR *p_msg)
{
    UINT8 *p, *p_dst;
    UINT16 len;
    HAL_TRACE_DEBUG2 ("nfc_hal_delayed_pkt_queuing(): state:%d cur:%d",
                      nfc_hal_cb2.dly_ntf.state, nfc_hal_cb2.dly_ntf.cur );

    if (nfc_hal_cb2.dly_ntf.state != NFC_HAL_DM_DLYNTF_STATE_QUEUING)
        return;

    p = (UINT8 *) (p_msg + 1) + p_msg->offset;

    len = (p_msg->len - p_msg->offset);
    if (nfc_hal_cb2.dly_ntf.cur + len + 1 > NFC_HAL_DLYNTF_PKT_LEN)
    {
        nfc_hal_cb2.dly_ntf.state = NFC_HAL_DM_DLYNTF_STATE_STOPED;
        return;
    }
    if (len < NCI_MSG_HDR_SIZE)
        return;

    p_dst = &nfc_hal_cb2.dly_ntf.pkt[nfc_hal_cb2.dly_ntf.cur];
    *p_dst++ = len;
    memcpy( p_dst, p, len );
    nfc_hal_cb2.dly_ntf.cur += len + 1;
    HAL_TRACE_DEBUG2 ("nfc_hal_delayed_pkt_queuing(): queuing_len:%d cur:%d",
                      len + 1, nfc_hal_cb2.dly_ntf.cur);
}
#endif /*(defined (NFC_HAL_DELAYED_NTF_INCLUDED) && NFC_HAL_DELAYED_NTF_INCLUDED==TRUE) */

/******************************************************************************
***
** Function         nfc_hal_main_handle_halcc
**
** Description      This function is called to handle exhal message
**
** Returns          void
**
*******************************************************************************/
static void nfc_hal_main_handle_halcc (NFC_HDR *p_msg)
{
    UINT8   *ps, *pp, cid, pbf;
    UINT16  len = p_msg->len;
    UINT8 msg_data[5] = {(NCI_MTS_HALMSG|NCI_GID_PROP), 0, 0, 0, HAL_NFC_STATUS_OK};
    pp = (UINT8 *)(p_msg + 1) + p_msg->offset ;

    HAL_TRACE_DEBUG0 ("nfc_hal_main_handle_halcc ()");

    msg_data[1] = pp[1];
    switch( pp[1] ){
    case NFC_HAL_SENDCMD_SNOOZE_ID:
        /* SNOOZE CTRL */
        if( pp[2] == 1 )
        {
            msg_data[4] = HAL_NFC_STATUS_OK;
            switch( pp[3] )
            {
            case NFC_HAL_SENDCMD_SNOOZE_OP_DEFAULT:
                HAL_TRACE_DEBUG0 ("nfc_hal_main_handle_halcc (): SNOOZE_DEFAULT");
                nfc_hal_dm_power_mode_execute (NFC_HAL_LP_DEFAULT_EVT2);
                break;
            case NFC_HAL_SENDCMD_SNOOZE_OP_KEEP_WAKE:
                HAL_TRACE_DEBUG0 ("nfc_hal_main_handle_halcc (): SNOOZE_KEEP_WAKE");
                nfc_hal_dm_power_mode_execute (NFC_HAL_LP_KEEP_WAKE_EVT2);
                break;
            case NFC_HAL_SENDCMD_SNOOZE_OP_LP_TIMER_FORCE_EXPIRE:
                HAL_TRACE_DEBUG0 ("nfc_hal_main_handle_halcc (): SNOOZE_LP_TIMER_FORCE_EXPIRE");
                nfc_hal_dm_power_mode_execute (NFC_HAL_LP_TIMER_FORCE_EXPIRE_EVT2);
                break;
            case NFC_HAL_SENDCMD_SNOOZE_OP_LP_LONG_TIMER:
                HAL_TRACE_DEBUG0 ("nfc_hal_main_handle_halcc (): SNOOZE_LP_LONG_TIMER");
                nfc_hal_dm_power_mode_execute (NFC_HAL_LP_LONG_TIMER_EVT2);
                break;
            default:
                HAL_TRACE_DEBUG0 ("nfc_hal_main_handle_halcc (): Unknown");
                msg_data[4] = HAL_NFC_STATUS_FAILED;
            }
            msg_data[2] = 2; /* len */
            msg_data[3] = pp[3];

            nfc_hal_cb2.p_data_cback (sizeof(msg_data), msg_data); /* 5B */
            return;
        }
        break;
    case NFC_HAL_PREDISCOVER_CMD_REG:
        /* allocate nfc_hal_cb2.prediscover_msg */
        HAL_TRACE_DEBUG0 ("nfc_hal_main_handle_halcc (): PREDISCOVER_CMD_REG");
        /* remove 3-byte NCI header */
        if (!nfc_hal_main_reg_prediscover_cmd (p_msg->len-NCI_MSG_HDR_SIZE,
                                               (UINT8 *)(p_msg + 1) + p_msg->offset + NCI_MSG_HDR_SIZE))
        {
            msg_data[2] = 1; /* len */
            msg_data[3] = HAL_NFC_STATUS_OK;
            nfc_hal_cb2.p_data_cback (sizeof(msg_data)-1, msg_data); /* 4B */
            return;
        }
        break;
    case NFC_HAL_GET_DEVICE_INFO:
        HAL_TRACE_DEBUG0 ("nfc_hal_main_handle_halcc (): GET_DEVICE_INFO");
        nfc_hal_main_send_devinfo();
        return;
    case NFC_HAL_DEVICE_RESET:
        HAL_TRACE_DEBUG0 ("nfc_hal_main_handle_halcc (): DEVICE_RESET");
        msg_data[2] = 1; /* len */
        if (USERIAL_DeviceReset() == 0)
            msg_data[3] = HAL_NFC_STATUS_OK;
        else
            msg_data[3] = HAL_NFC_STATUS_FAILED;
        nfc_hal_cb2.p_data_cback (sizeof(msg_data)-1, msg_data); /* 4B */
        return;
    case NFC_HAL_REQ_DELAYED_PKTS:
        HAL_TRACE_DEBUG0 ("nfc_hal_main_handle_halcc (): REQ_DELAYED_PKTS");
#if (defined (NFC_HAL_DELAYED_NTF_INCLUDED) && NFC_HAL_DELAYED_NTF_INCLUDED==TRUE)
        /* drain saved NTF packet by upper layer request */
        msg_data[2] = 2; /* len */
        msg_data[3] = pp[3];
        if (nfc_hal_main_delayed_pkt_ctrl(pp[3]))
        {
            msg_data[4] = HAL_NFC_STATUS_FAILED;
        }
        nfc_hal_cb2.p_data_cback (sizeof(msg_data), msg_data); /* 5B */
        return;
#endif /*(defined (NFC_HAL_DELAYED_NTF_INCLUDED) && NFC_HAL_DELAYED_NTF_INCLUDED==TRUE) */
    default:
        ;
    }
    {   /* Unknown HalMsg */
        msg_data[2] = 1;
        msg_data[3] = HAL_NFC_STATUS_FAILED;
        nfc_hal_cb2.p_data_cback (sizeof(msg_data)-1, msg_data);
    }
}

/*******************************************************************************
**
** Function         nfc_hal_main_task
**
** Description      NFC HAL NCI transport event processing task
**
** Returns          0
**
*******************************************************************************/
UINT32 nfc_hal_main_task (UINT32 param)
{
    UINT16   event;
    UINT8    byte;
    UINT8    num_interfaces;
    UINT8    *p;
    NFC_HDR  *p_msg;
    BOOLEAN  free_msg;

    HAL_TRACE_DEBUG0 ("NFC_HAL_TASK started");

    /* Main loop */
    while (TRUE)
    {
        event = GKIH_wait (0xFFFF, 0);

        /* Handle NFC_HAL_TASK_EVT_INITIALIZE (for initializing NCI transport) */
        if (event & NFC_HAL_TASK_EVT_INITIALIZE)
        {
            HAL_TRACE_DEBUG0 ("NFC_HAL_TASK got NFC_HAL_TASK_EVT_INITIALIZE signal. Opening NFC transport...");

            nfc_hal_main_open_transport ();
        }

        /* Check for terminate event */
        if (event & NFC_HAL_TASK_EVT_TERMINATE)
        {
#if (NFC_HAL_CLOSE_CONFIG == TRUE)
            HAL_TRACE_DEBUG0 ("NFC_HAL_TASK got NFC_HAL_TASK_EVT_TERMINATE(closing_vsc)");

            /* start pre closing config */
            nfc_hal_cb2.dev_cb.next_dm_config = NFC_HAL_DM_CONFIG2_CLOSING_VSC;
            nfc_hal_cb2.dev_cb.next_closing_vsc = 1; /* init: pointer */
            NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_W4_PRE_CLOSING_DONE);

            nfc_hal_dm_config_nfcc_closing ();

            continue;
#else
            HAL_TRACE_DEBUG0 ("NFC_HAL_TASK got NFC_HAL_TASK_EVT_TERMINATE");
            nfc_hal_main_handle_terminate ();

            /* Close uart */
            USERIAL_Close (USERIAL_NFC_PORT);

            if (nfc_hal_cb2.p_stack_cback)
            {
                nfc_hal_cb2.p_stack_cback (HAL_NFC_CLOSE_CPLT_EVT, HAL_NFC_STATUS_OK);
                nfc_hal_cb2.p_stack_cback = NULL;
            }
            continue;
#endif
        }

        /* Check for power cycle event */
        if (event & NFC_HAL_TASK_EVT_POWER_CYCLE)
        {
            HAL_TRACE_DEBUG0 ("NFC_HAL_TASK got NFC_HAL_TASK_EVT_POWER_CYCLE");
            nfc_hal_main_handle_terminate ();

            /* Close uart */
            USERIAL_Close (USERIAL_NFC_PORT);

            /* power cycle timeout */
            nfc_hal_main_start_quick_timer (&nfc_hal_cb2.timer, NFC_HAL_TTYPE_POWER_CYCLE,
                                            (NFC_HAL_POWER_CYCLE_DELAY*QUICK_TIMER_TICKS_PER_SEC)/1000);
            continue;
        }

        /* NCI message ready to be sent to NFCC */
        if (event & NFC_HAL_TASK_EVT_MBOX)
        {
            while ((p_msg = (NFC_HDR *) GKIH_read_mbox (NFC_HAL_TASK_MBOX)) != NULL)
            {
                free_msg = TRUE;
                switch (p_msg->event & NFC_EVT_MASK)
                {
                case NFC_HAL_EVT_TO_NFC_NCI:
                    nfc_hal_main_send_message (p_msg);
                    /* do not free buffer. NCI VS code may keep it for processing later */
                    free_msg = FALSE;
                    break;

                case NFC_HAL_EVT_TO_PREDISCOVER_CMD_NFC_NCI:
                    /* Send PreDiscover Cmd if regsitered */
                    if( nfc_hal_cb2.prediscover_msg )
                    {
                        nfc_hal_main_send_message (nfc_hal_cb2.prediscover_msg);
                        nfc_hal_cb2.prediscover_msg = NULL;
                        GKIH_delay (1); /* Not wait PREDISCOVER_CMD rsp, insert small bubble for yeild thread */
                    }
                    break;
                case NFC_HAL_EVT_TO_PREDISCOVER_CMD_REG:
                    /* allocate nfc_hal_cb2.prediscover_msg */
                    nfc_hal_main_reg_prediscover_cmd( p_msg->len, (UINT8 *)(p_msg + 1) + p_msg->offset );
                    break;

                case NFC_HAL_EVT_POST_CORE_RESET:
                    NFC_HAL_SET_INIT_STATE2 (NFC_HAL_INIT_STATE2_W4_POST_INIT_DONE);

                    /* set NCI Control packet size from CORE_INIT_RSP */
                    p = (UINT8 *) (p_msg + 1) + p_msg->offset + NCI_MSG_HDR_SIZE;
                    p += 5;
                    STREAM_TO_UINT8 (num_interfaces, p);
                    p += (num_interfaces + 3);
                    nfc_hal_cb2.ncit_cb.nci_ctrl_size = *p;

                    /* start post initialization */
                    nfc_hal_cb2.dev_cb.next_dm_config = NFC_HAL_DM_CONFIG2_LPTD;
                    nfc_hal_cb2.dev_cb.next_startup_vsc = 1;
                    nfc_hal_cb2.dev_cb.next_startup_debug = 2;

                    nfc_hal_dm_config_nfcc ();
                    break;

                case NFC_HAL_EVT_TO_START_QUICK_TIMER:
                    GKIH_start_timer (NFC_HAL_QUICK_TIMER_ID, ((GKI_SECS_TO_TICKS (1) / QUICK_TIMER_TICKS_PER_SEC)), TRUE);
                    break;

                case NFC_HAL_EVT_PRE_DISCOVER:
                    NFC_HAL_SET_INIT_STATE2(NFC_HAL_INIT_STATE2_W4_PREDISCOVER_DONE);
                    nfa_hal_send_pre_discover_cfg ();
                    break;

                case NFC_HAL_EVT_CONTROL_GRANTED:
                    nfc_hal_dm_send_pend_cmd ();
                    break;

                case  NFC_HAL_EVT_POST_TERMINATE:
                    HAL_TRACE_DEBUG0 ("NFC_HAL_TASK got NFC_HAL_EVT_POST_TERMINATE");
                    nfc_hal_main_handle_terminate ();

                    /* Close uart */
                    USERIAL_Close (USERIAL_NFC_PORT);

                    if (nfc_hal_cb2.p_stack_cback)
                    {
                        nfc_hal_cb2.p_stack_cback (HAL_NFC_CLOSE_CPLT_EVT, HAL_NFC_STATUS_OK);
                        nfc_hal_cb2.p_stack_cback = NULL;
                    }
                    break;

                case NFC_HAL_EVT_TO_NFC_HALCC:
                    HAL_TRACE_DEBUG0 ("NFC_HAL_TASK got NFC_HAL_EVT_TO_NFC_HALCC");
                    nfc_hal_main_handle_halcc(p_msg);
                    break;

                default:
                    break;
                }

                if (free_msg)
                    GKIH_freebuf (p_msg);
            }
        }

        /* Data waiting to be read from serial port */
        if (event & NFC_HAL_TASK_EVT_DATA_RDY)
        {
            while (TRUE)
            {
                /* Read one byte to see if there is anything waiting to be read */
                if (USERIAL_Read (USERIAL_NFC_PORT, &byte, 1) == 0)
                {
                    break;
                }
                if (nfc_hal_nci_receive_msg (byte))
                {
                    /* complete of receiving NCI message */
                    nfc_hal_nci_assemble_nci_msg ();
                    if (nfc_hal_cb2.ncit_cb.p_rcv_msg)
                    {
                        if (nfc_hal_nci_preproc_rx_nci_msg (nfc_hal_cb2.ncit_cb.p_rcv_msg))
                        {
                            /* Send NCI message to the stack */
                            nfc_hal_send_nci_msg_to_nfc_task (nfc_hal_cb2.ncit_cb.p_rcv_msg);
                        }
                        else
                        {
                            if (nfc_hal_cb2.ncit_cb.p_rcv_msg)
                                GKIH_freebuf(nfc_hal_cb2.ncit_cb.p_rcv_msg);
                        }
                        nfc_hal_cb2.ncit_cb.p_rcv_msg = NULL;
                    }
                }
            } /* while (TRUE) */
        }

        /* Process quick timer tick */
        if (event & NFC_HAL_QUICK_TIMER_EVT_MASK)
        {
            nfc_hal_main_process_quick_timer_evt ();
        }
    }

    HAL_TRACE_DEBUG0 ("nfc_hal_main_task terminated");

    GKIH_exit_task (GKIH_get_taskid ());
    return 0;
}

/*******************************************************************************
**
** Function         HAL_NfcSetTraceLevel
**
** Description      This function sets the trace level for HAL.  If called with
**                  a value of 0xFF, it simply returns the current trace level.
**
** Returns          The new or current trace level
**
*******************************************************************************/
UINT8 HAL_NfcSetTraceLevel (UINT8 new_level)
{
    if (new_level != 0xFF)
        nfc_hal_cb2.trace_level = new_level;

    return (nfc_hal_cb2.trace_level);
}
/*******************************************************************************
**
** Function         HAL_NfcSetoptionalparm
**
** Description      This function sets the optional param for HAL.
**
** Returns          fail none zero
**
*******************************************************************************/
INT32 HAL_NfcSetoptionalparm (UINT32 option_no, UINT32 new_option)
{
    INT32 ret=0;
    switch( option_no ){
    case NFC_OPTPARM_HAL_INIT_CTRL:
        nfc_hal_cb2.hal_init_ctrl = new_option;
        break;
    case NFC_OPTPARM_HAL_NVERASE:
        nfc_hal_cb2.nverase = new_option ? NFC_HAL_NVERASE_ERASE : NFC_HAL_NVERASE_PRESERVE;
        HAL_TRACE_DEBUG1 ("HAL_NfcSetoptionalparm NFC_OPTPARM_HAL_NVERASE=%d", nfc_hal_cb2.nverase);
        break;
    default:
        ret=-1;
    }
    return ret;
}
