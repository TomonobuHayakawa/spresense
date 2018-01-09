/*
 * Sony Advanced Instrument of Libraries
 *
 * This program is subject to copyright protection in accordance with the
 * applicable law. It must not, except where allowed by law, by any means or
 * in any form be reproduced, distributed or lent. Moreover, no part of the
 * program may be used, viewed, printed, disassembled or otherwise interfered
 * with in any form, except where allowed by law, without the express written
 * consent of the copyright holder.
 *
 * Copyright 2013,2014 Sony Corporation
 */

#include <stdlib.h>
#include "nfc/nfc_fe_api.h"

#include "nf_common.h"

typedef struct {
    FUNC_FE_CB  cb;
    RECV_INFO   *recv_info;
} CB_TH_PARAM;

static pthread_t cbth;
static CB_TH_PARAM cb_th_param;

tNF_STATUS g_nf_status;
sem_t sem_cb_wait;
bool err_ntf_flg = false;

extern struct msgbuf recv_buf;

/*******************************************************************************
**
** Function         callback_task
**
** Description      This function is call a user callback function.
**
** Parameters:      *arg - (input) pointer to thread parameters.
**
** Returns          void
**
*******************************************************************************/
static void *callback_task(void *arg)
{
    int recv_len;
    unsigned char recv_data[255];
    int status;
    int type = D_TECH_MASK_TYPE_F;
    CB_TH_PARAM *p_task_para = (CB_TH_PARAM *)arg;

    DBG_LOGF_DEBUG("callback_task start\n");

    while(1)
    {
        DBG_LOGF_DEBUG("callback_task wait\n");
        sem_wait(&sem_cb_wait);

        if((g_nf_status == NF_ST_END) || (g_nf_status == NF_ST_IDLE))
        {
            break;
        }

        if(err_ntf_flg == false)
        {
            status = D_NFC_SUCCESS;
            type = D_TECH_MASK_TYPE_F;
            recv_len = hostif_get_fe_data(recv_data, 255);
        }
        else
        {
            err_ntf_flg = false;
            status = D_NFC_FAILED;
            recv_len = 0;
        }

        DBG_LOGF_DEBUG("call callback func\n");
        (p_task_para->cb)(status, type, recv_data, recv_len);
    }

    DBG_LOGF_DEBUG("callback_task end\n");

    pthread_exit(NULL);
    return (void *)NULL;
}


/*******************************************************************************
**
** Function         NF_Listen_Start
**
** Description      This function is start listen RF.
**
** Parameters:      mode - (input) indicate rf or wire.
**                  nf_event_cb -(input) pointer to user call back function.
**                  type -(input) indicate rf type.
**
** Returns          D_NF_SUCCESS if process succeeded
**                  D_NF_DUPL_ERR if calling this function twice.
**                  D_NF_PROC_ERR if calling this function at wrong state.
**
*******************************************************************************/
int NF_Listen_Start(int mode, FUNC_FE_CB nf_event_cb, int type)
{
    int ret;

    DBG_LOGF_DEBUG("%s: start mode=%d, nf_event_cb=0x%x, type=%d\n", __func__, mode, nf_event_cb, type);

    /* check NF Status */
    if(g_nf_status != NF_ST_IDLE)
    {
        DBG_LOGF_ERROR("%s: status err status=%d\n", __func__, g_nf_status);
        if((g_nf_status == NF_ST_STARTING) || (g_nf_status == NF_ST_ENABLE) || (g_nf_status == NF_ST_FE_OPEN))
        {
            return(D_NF_DUPL_ERR);
        }
        else if(g_nf_status == NF_ST_END)
        {
            return(D_NF_PROC_ERR);
        }
    }

    /* change status */
    g_nf_status = NF_ST_STARTING;

    /* check parameter */
    if(mode != (NF_START_MODE_RF | NF_START_MODE_WIRE))
    {
        g_nf_status = NF_ST_IDLE;
        DBG_LOGF_ERROR("%s: param err mode=%d, nf_event_cb=0x%x, type=%d\n", __func__, mode, nf_event_cb, type);
        return(D_NF_PARAM_ERR);
    }
    if(nf_event_cb == NULL)
    {
        g_nf_status = NF_ST_IDLE;
        DBG_LOGF_ERROR("%s: param err mode=%d, nf_event_cb=0x%x, type=%d\n", __func__, mode, nf_event_cb, type);
        return(D_NF_PARAM_ERR);
    }
    if(type & (D_TECH_MASK_TYPE_A | D_TECH_MASK_TYPE_B))
    {
        g_nf_status = NF_ST_IDLE;
        DBG_LOGF_ERROR("%s: param err mode=%d, nf_event_cb=0x%x, type=%d\n", __func__, mode, nf_event_cb, type);
        return(D_NF_UNSUPPORT);
    }
    if(type != D_TECH_MASK_TYPE_F)
    {
        g_nf_status = NF_ST_IDLE;
        DBG_LOGF_ERROR("%s: param err mode=%d, nf_event_cb=0x%x, type=%d\n", __func__, mode, nf_event_cb, type);
        return(D_NF_PARAM_ERR);
    }

    sem_init(&sem_cb_wait, 0, 0);

    /* create callback thread */
    cb_th_param.cb        = nf_event_cb;
    cb_th_param.recv_info = NULL;
    if(pthread_create(&cbth, NULL, callback_task, &cb_th_param) != 0)
    {
        DBG_LOGF_ERROR("%s: pthread_create() failed\n", __FUNCTION__);
        return(D_NF_PROC_ERR);
    }

    /* I2C Open */
    if(hostif_open(HOSTIF_I2C, NULL) != E_OK)
    {
        g_nf_status = NF_ST_IDLE;
        DBG_LOGF_ERROR("%s: hostif_open() failed\n", __func__);
        return(D_NF_PROC_ERR);
    }

    /* Listen Start */
    ret = nf_listen_start();
    if(ret != D_NF_SUCCESS)
    {
        hostif_close();
        g_nf_status = NF_ST_IDLE;

        DBG_LOGF_ERROR("%s: nf_listen_start() failed\n", __func__);
        return(D_NF_PROC_ERR);
    }

    /* change status */
    g_nf_status = NF_ST_ENABLE;

    DBG_LOGF_DEBUG("%s: end\n", __func__);

    return(D_NF_SUCCESS);
}


/*******************************************************************************
**
** Function         NF_Listen_End
**
** Description      This function is finish listen RF.
**
** Parameters:      none
**
** Returns          D_NF_SUCCESS if process succeeded.
**                  D_NF_NOT_START if calling this function before start.
**                  D_NF_PROC_ERR if calling this function at wrong state.
**
*******************************************************************************/
int NF_Listen_End(void)
{
    int ret;

    DBG_LOGF_DEBUG("%s: start\n", __func__);

    /* check NF Status */
    if(g_nf_status != NF_ST_ENABLE)
    {
        if(g_nf_status == NF_ST_IDLE)
        {
            return(D_NF_SUCCESS);
        }
        else if((g_nf_status == NF_ST_STARTING) || (g_nf_status == NF_ST_END))
        {
            DBG_LOGF_ERROR("%s: status error status=%d\n", __func__, g_nf_status);
            return(D_NF_PROC_ERR);
        }
    }

    /* change status */
    g_nf_status = NF_ST_END;

    ret = nf_listen_end();
    if(ret != D_NF_SUCCESS)
    {
        DBG_LOGF_ERROR("%s: nf_listen_end() failed\n", __func__);
    }

    /* I2C Close */
    hostif_close();

    sem_destroy(&sem_cb_wait);

    /* change status */
    g_nf_status = NF_ST_IDLE;

    DBG_LOGF_DEBUG("%s: end\n", __func__);
    return(D_NF_SUCCESS);
}


/*******************************************************************************
**
** Function         NF_Felica_Open
**
** Description      This function is open wire communication.
**
** Parameters:      none
**
** Returns          D_NF_SUCCESS if process succeeded.
**                  D_NF_NOT_START if calling this function before start.
**                  D_NF_PROC_ERR if calling this function at wrong state.
**
*******************************************************************************/
int NF_Felica_Open(void)
{
    int ret;

    DBG_LOGF_DEBUG("%s: start\n", __func__);

    /* check NF Status */
    if(g_nf_status != NF_ST_ENABLE)
    {
        DBG_LOGF_ERROR("%s: status error status=%d\n", __func__, g_nf_status);
        if(g_nf_status == NF_ST_IDLE)
        {
            return(D_NF_NOT_START);
        }
        else if((g_nf_status == NF_ST_STARTING) || (g_nf_status == NF_ST_END) || (g_nf_status == NF_ST_FE_OPEN))
        {
            return(D_NF_PROC_ERR);
        }
    }

    /* change status */
    g_nf_status = NF_ST_FE_OPEN;

    ret = nf_felica_open();
    if(ret != D_NF_SUCCESS)
    {
        DBG_LOGF_ERROR("%s: nf_felica_open() failed\n", __func__);
        g_nf_status = NF_ST_ENABLE;
        return(D_NF_PROC_ERR);
    }

    DBG_LOGF_DEBUG("%s: end\n", __func__);
    return(D_NF_SUCCESS);
}


/*******************************************************************************
**
** Function         NF_Felica_Close
**
** Description      This function is close wire communication.
**
** Parameters:      none
**
** Returns          D_NF_SUCCESS if process succeeded.
**                  D_NF_NOT_START if calling this function before start.
**                  D_NF_PROC_ERR if calling this function at wrong state.
**
*******************************************************************************/
int NF_Felica_Close(void)
{
    int ret;

    DBG_LOGF_DEBUG("%s: start\n", __func__);

    /* check NF Status */
    if(g_nf_status != NF_ST_FE_OPEN)
    {
        DBG_LOGF_ERROR("%s: status error status=%d\n", __func__, g_nf_status);
        if(g_nf_status == NF_ST_IDLE)
        {
            return(D_NF_NOT_START);
        }
        else if((g_nf_status == NF_ST_STARTING) || (g_nf_status == NF_ST_ENABLE) || (g_nf_status == NF_ST_END))
        {
            return(D_NF_PROC_ERR);
        }
    }

    /* change status */
    g_nf_status = NF_ST_ENABLE;

    ret = nf_felica_close();
    if(ret != D_NF_SUCCESS)
    {
        DBG_LOGF_ERROR("%s: nf_felica_close() failed \n", __func__);
        return(D_NF_PROC_ERR);
    }

    DBG_LOGF_DEBUG("%s: end\n", __func__);
    return(D_NF_SUCCESS);
}


/*******************************************************************************
**
** Function         NF_Felica_Close
**
** Description      This function is send data in wire communication.
**
** Parameters:      send_data - (input) pointer to send_data.
**                  send_size - (input) size of send_data.
**
** Returns          D_NF_SUCCESS if process succeeded.
**                  D_NF_NOT_START if calling this function before start.
**                  D_NF_PROC_ERR if calling this function at wrong state.
**                  D_NF_PARAM_ERR if parameter is wrong.
**
*******************************************************************************/
int NF_Felica_Send(unsigned char *send_data, unsigned short send_size)
{
    int ret;

    DBG_LOGF_DEBUG("%s: start\n", __func__);

    /* check NF Status */
    if(g_nf_status != NF_ST_FE_OPEN)
    {
        DBG_LOGF_ERROR("%s: status error status=%d\n", __func__, g_nf_status);
        if(g_nf_status == NF_ST_IDLE)
        {
            return(D_NF_NOT_START);
        }
        else if((g_nf_status == NF_ST_STARTING) || (g_nf_status == NF_ST_ENABLE) || (g_nf_status == NF_ST_END))
        {
            return(D_NF_PROC_ERR);
        }
    }

    /* check parameter */
    if((send_data == NULL) || (send_size == 0))
    {
        DBG_LOGF_ERROR("%s: param err data=0x%x, size=%d\n", __func__, send_data, send_size);
        return(D_NF_PARAM_ERR);
    }

    ret = nf_felica_send(send_data, send_size);
    if(ret != D_NF_SUCCESS)
    {
        DBG_LOGF_DEBUG("%s: nf_felica_send() failed\n", __func__);
        return(D_NF_PROC_ERR);
    }

    DBG_LOGF_DEBUG("%s: end\n", __func__);
    return(D_NF_SUCCESS);
}

/*******************************************************************************
**
** Function         NF_Fw_Update
**
** Description      This function is 
**
** Parameters:      void
**
** Returns          D_NF_SUCCESS if process succeeded.
**                  D_NF_NOT_START if calling this function before start.
**                  D_NF_PROC_ERR if calling this function at wrong state.
**
*******************************************************************************/
int NF_Fw_Update(void)
{
    int ret;

    DBG_LOGF_DEBUG("%s: start \n", __func__);

    /* check NF Status */
    if(g_nf_status != NF_ST_IDLE)
    {
        DBG_LOGF_ERROR("%s: status err status=%d\n", __func__, g_nf_status);
        if((g_nf_status == NF_ST_STARTING) || (g_nf_status == NF_ST_ENABLE) || (g_nf_status == NF_ST_FE_OPEN))
        {
            return(D_NF_DUPL_ERR);
        }
        else if(g_nf_status == NF_ST_END)
        {
            return(D_NF_PROC_ERR);
        }
    }

    /* change status */
    g_nf_status = NF_ST_UPDATING;

    /* get patch file */
    if(update_get_patch() != E_OK)
    {
        g_nf_status = NF_ST_IDLE;
        DBG_LOGF_ERROR("%s: hostif_get_patch() failed\n", __func__);
        return(D_NF_PROC_ERR);
    }

    /* I2C Open */
    if(hostif_open(HOSTIF_I2C, NULL) != E_OK)
    {
        /* delete patch file */
        update_delete_patch();

        g_nf_status = NF_ST_IDLE;
        DBG_LOGF_ERROR("%s: hostif_open() failed\n", __func__);
        return(D_NF_PROC_ERR);
    }

    /* Listen Start */
    ret = nf_fw_update();
    if(ret != D_NF_SUCCESS)
    {
        /* delete patch file */
        update_delete_patch();
        g_nf_status = NF_ST_IDLE;

        hostif_close();

        DBG_LOGF_ERROR("%s: nf_fw_update() failed\n", __func__);
        return(D_NF_PROC_ERR);
    }

    /* delete patch file */
    update_delete_patch();

    /* change status */
    g_nf_status = NF_ST_IDLE;

    /* I2C Close */
    hostif_close();

    DBG_LOGF_DEBUG("%s: end\n", __func__);

    return(D_NF_SUCCESS);
}
