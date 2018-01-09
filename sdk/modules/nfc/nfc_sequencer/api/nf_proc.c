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

#include <string.h>

#include "nf_common.h"

#define LOOP_OUT 0
#define LOOP_IN  1

#define NCI_HEADER_SZ   3
#define COMMAND_MAX_LEN 258
#define RSP_TIME 100

typedef enum {
    SEND, /* send command */
    RECV, /* receive response or notification */
    NON,  /* end of command */
}TYPE;

typedef struct{
    TYPE           type;                    /* next proccess */
    unsigned char  data[COMMAND_MAX_LEN];   /* command */
    bool           mask_flg;                /* use or unuse exp_mask */
    unsigned char  mask[COMMAND_MAX_LEN];   /* mask expected value */
}DATA_FORMAT;

DATA_FORMAT rf_listen[] = {
#ifdef CXD224X_I2C
    {RECV, {0x60, 0x00, 0x02, 0x00, 0x01},                   false, {0x00}}, /* CORE_RESET_NTF */
#endif /* CXD224X_I2C */
    {SEND, {0x2F, 0x20, 0x00},                               false, {0x00}}, /* GET_VERSION */
    {RECV, {0x4F, 0x20, 0x07, 0x00, 0x20, 0x01, 0x00, 0x00,
            0x31, 0x02},                                     false, {0x00}},
    {SEND, {0x2F, 0x24, 0x00},                               false, {0x00}}, /* GET_INTER_VERSION */
    {RECV, {0x4F, 0x24, 0x04, 0x00, 0x83, 0x30, 0x11},       false, {0x00}},
    {SEND, {0x2F, 0x1B, 0x00},                               false, {0x00}}, /* GET_PATCH_VERSION */
    {RECV, {0x4F, 0x1B, 0x06, 0x00, 0x00, 0x0B, 0x00, 0x01,
            0x00},                                           true,
           {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00}},
    {SEND, {0x20, 0x00, 0x01, 0x00},                         false, {0x00}}, /* CORE_RESET_CMD */
    {RECV, {0x40, 0x00, 0x03, 0x00, 0x10, 0x01},             false, {0x00}}, /* CORE_RESET_RSP */
    {SEND, {0x20, 0x01, 0x00},                               false, {0x00}}, /* CORE_INIT_CMD */
    {RECV, {0x40, 0x01, 0x15, 0x00, 0x01, 0x0E, 0x02, 0x00,
            0x04, 0x00, 0x01, 0x02, 0x03, 0x03, 0xF8, 0x02,
            0xFF, 0xF0, 0x01, 0x1A, 0x00, 0x00, 0x00, 0x00}, false, {0x00}}, /* CORE_INIT_RSP */
    {SEND, {0x22, 0x00, 0x01, 0x01},                         false, {0x00}}, /* NFCEE_DISCOVER_CMD */
    {RECV, {0x42, 0x00, 0x02, 0x00, 0x02},                   false, {0x00}}, /* NFCEE_DISCOVER_RSP */
    {RECV, {0x62, 0x00, 0x05, 0x01, 0x01, 0x01, 0x01, 0x00}, false, {0x00}}, /* NFCEE_DISCOVER_NTF(SIM disable) */
    {RECV, {0x62, 0x00, 0x05, 0x02, 0x01, 0x01, 0x02, 0x00}, false, {0x00}}, /* NFCEE_DISCOVER_NTF(SAM disable) */
    {SEND, {0x22, 0x01, 0x02, 0x02, 0x01},                   false, {0x00}}, /* NFCEE_MODE_SET_CMD(for SAM) */
    {RECV, {0x42, 0x01, 0x01, 0x00},                         false, {0x00}}, /* NFCEE_MODE_SET_RSP */
    {RECV, {0x62, 0x00, 0x05, 0x02, 0x00, 0x01, 0x02, 0x00}, false, {0x00}}, /* NFCEE_DISCOVER_NTF(SAM enable) */
    {RECV, {0x61, 0x0A, 0x06, 0x01, 0x00, 0x03, 0x02, 0x82, 
            0x03},                                           false, {0x00}}, /* RF_NFCEE_DISCOVER_REQ_NTF */
    {SEND, {0x22, 0x00, 0x01, 0x00},                         false, {0x00}}, /* NFCEE_DISCOVER_CMD(Disable */
    {RECV, {0x42, 0x00, 0x02, 0x00, 0x02},                   false, {0x00}}, /* NFCEE_DISCOVER_RSP */
    {SEND, {0x21, 0x01, 0x07, 0x00, 0x01, 0x01, 0x03, 0x02,
            0x01, 0x03},                                     false, {0x00}}, /* RF_SET_LISTEN_MODE_ROUTING_CMD */
    {RECV, {0x41, 0x01, 0x01, 0x00},                         false, {0x00}}, /* RF_SET_LISTEN_MODE_ROUTING_RSP */
    {SEND, {0x21, 0x03, 0x03, 0x01, 0x82, 0x01},             false, {0x00}}, /* RF_DISCOVER_CMD */
    {RECV, {0x41, 0x03, 0x01, 0x00},                         false, {0x00}}, /* RF_DISCOVER_RSP */
    {NON,  {0x00},                                           false, {0x00}}
};

DATA_FORMAT rf_deactivate[] = {
    {SEND, {0x21, 0x06, 0x01, 0x00}, false, {0x00}}, /* RF_DEACTIVATE_CMD */
    {RECV, {0x41, 0x06, 0x01, 0x00}, false, {0x00}}, /* RF_DEACTIVATE_RSP */
    {NON,  {0x00},                   false, {0x00}}
};

DATA_FORMAT felica_open[] = {  
    {SEND, {0x20, 0x04, 0x06, 0x03, 0x01, 0x01, 0x02, 0x02, 
            0x02},                                          false, {0x00}}, /* CORE_CONN_CREATE_CMD(for SAM) */
    {RECV, {0x40, 0x04, 0x04, 0x00, 0xFF, 0x01, 0x02},      false, {0x00}}, /* CORE_CONN_CREATE_RSP */
    {NON,  {0x00},                                          false, {0x00}}
};

DATA_FORMAT felica_close[] = {
    {SEND, {0x20,0x05,0x01,0x02}, false, {0x00}}, /* CORE_CONN_CLOSE_CMD */
    {RECV, {0x40,0x05,0x01,0x00}, false, {0x00}}, /* CORE_CONN_CLOSE_RSP */
    {NON,  {0x00},                false, {0x00}}
};

DATA_FORMAT fw_patch_update[] = {
#ifdef CXD224X_I2C
    {RECV, {0x60, 0x00, 0x02, 0x00, 0x01},                   false, {0x00}}, /* CORE_RESET_NTF */
#endif /* CXD224X_I2C */
    {SEND, {0x20, 0x00, 0x01, 0x00},                         false, {0x00}}, /* CORE_RESET_CMD */
    {RECV, {0x40, 0x00, 0x03, 0x00, 0x10, 0x01},             false, {0x00}}, /* CORE_RESET_RSP */
    {SEND, {0x2F, 0x20, 0x00},                               false, {0x00}}, /* PROP(GET_VERSION_CMD) */
    {RECV, {0x4F, 0x20, 0x07, 0x00, 0x20, 0x01, 0x00, 0x00,
            0x31, 0x02},                                     false, {0x00}},
    {SEND, {0x2F, 0x24, 0x00},                               false, {0x00}}, /* PROP(GET_INTER_VERSION_CMD) */
    {RECV, {0x4F, 0x24, 0x04, 0x00, 0x83, 0x30, 0x11},       false, {0x00}},
    {SEND, {0x2f,0x1b,0x00},                                 false, {0x00}}, /* PROP(OPTION_PATCH_VERSION_CMD) */
    {RECV, {0x4f,0x1b,0x06,0x00,0x00,0x0b,0x00,0x01,0x00},   true, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00}},
    {SEND, {0x2f,0x1c,0x05,0x00,0x38,0x00,0x00,0x00},        false, {0x00}}, /* PROP(OPTION_PATCH_START_CMD) */
    {RECV, {0x4f,0x1c,0x01,0x00},                            false, {0x00}},
    {SEND, {0x2f,0x1d,0x00},                                 false, {0x00}}, /* PROP(OPTION_PATCH_HEADER_CMD) */
    {RECV, {0x4f,0x1d,0x01,0x00},                            false, {0x00}},
    {SEND, {0x2f,0x1e,0x00},                                 false, {0x00}}, /* PROP(OPTION_PATCH_DATA_CMD) */
    {RECV, {0x4f,0x1e,0x01,0x00},                            false, {0x00}},
    {SEND, {0x2f,0x1f,0x00},                                 false, {0x00}}, /* PROP(OPTION_PATCH_END_CMD) */
    {RECV, {0x60,0x00,0x02,0x00,0x01},                       false, {0x00}}, /* CORE_RESET_NTF */
    {NON,  {0x00},                                           false, {0x00}}
};

static unsigned char nci_data[COMMAND_MAX_LEN];

bool wait_rsp_flg = false;

/*******************************************************************************
**
** Function         proc_com_send
**
** Description      This function is controler sending according 
**                  to command format.
**
** Parameters:      pdata - (input) command data pointer.
**                  posi  - (input/output) next command data position.
**
** Returns          E_OK
**                  E_FAIL
**
*******************************************************************************/
static int proc_com_send(DATA_FORMAT *pd, int *pos)
{
    unsigned short len;

    len = ((pd + (*pos))->data[2]) + NCI_HEADER_SZ;

    if(hostif_send((pd + (*pos))->data, len) != E_OK)
    {
        DBG_LOG_ERROR("hostif_send() failed\n");
        return(E_FAIL);
    }

    (*pos)++;

    return (E_OK);
}

/*******************************************************************************
**
** Function         proc_com_recv
**
** Description      This function is controler receiving according 
**                  to command format.
**
** Parameters:      pdata - (input) command data pointer.
**                  posi  - (input/output) next command data position.
**
** Returns          E_OK
**                  E_FAIL
**
*******************************************************************************/
static int proc_com_recv(DATA_FORMAT *pd, int *pos)
{
    unsigned short len;
    unsigned char *mask = NULL;

    wait_rsp_flg = true;

    if((pd + (*pos))->mask_flg == true)
    {
        mask = (pd + (*pos))->mask;
    }

    len = ((pd + (*pos))->data[2]) + NCI_HEADER_SZ;

    if(hostif_recv_and_cmp((pd + (*pos))->data, len, mask, RSP_TIME) != E_OK)
    {
        DBG_LOG_ERROR("hostif_recv_and_cmp() failed\n");
        return(E_FAIL);
    }

    wait_rsp_flg = false;

    (*pos)++;

    return (E_OK);
}

/*******************************************************************************
**
** Function         common_if
**
** Description      This function is controler sending or receiving according 
**                  to command format.
**
** Parameters:      pdata - (input) next command data.
**
** Returns          E_OK
**                  E_FAIL
**
*******************************************************************************/
int common_if(DATA_FORMAT *pdata)
{
    int tbl_posi = 0;
    int tbl_loop = 1;

    DBG_LOG_DEBUG("common_if() start\n");


    while(tbl_loop == LOOP_IN)
    {
        switch((pdata + tbl_posi)->type)
        {
        case SEND:
            if(proc_com_send(pdata, &tbl_posi) != E_OK)
                return(E_FAIL);

            break;

        case RECV:
            if(proc_com_recv(pdata, &tbl_posi) != E_OK)
                return(E_FAIL);

            break;

        case NON:
            tbl_loop = LOOP_OUT;
            break;

        default:
            DBG_LOG_ERROR("common_if() type empty\n");
            tbl_loop = LOOP_OUT;
            break;
        }
    }

    DBG_LOG_DEBUG("common_if() end\n");
    return (E_OK);
}


/*******************************************************************************
**
** Function         nf_listen_start
**
** Description      This function is start listen RF.
**
** Parameters:      none
**
** Returns          common_if function return value.
**
*******************************************************************************/
int nf_listen_start(void)
{
    DATA_FORMAT *p_data = rf_listen;
    int ret;

    ret = common_if(p_data);

    return ret;
}


/*******************************************************************************
**
** Function         nf_listen_end
**
** Description      This function is finish listen RF.
**
** Parameters:      none
**
** Returns          common_if function return value.
**
*******************************************************************************/
int nf_listen_end(void)
{
    DATA_FORMAT *p_data = rf_deactivate;
    int ret;

    ret = common_if(p_data);

    return ret;
}


/*******************************************************************************
**
** Function         nf_felica_open
**
** Description      This function is open wire communication.
**
** Parameters:      none
**
** Returns          common_if function return value.
**
*******************************************************************************/
int nf_felica_open(void)
{
    DATA_FORMAT *p_data = felica_open;
    int ret;

    ret = common_if(p_data);

    return ret;
}


/*******************************************************************************
**
** Function         nf_felica_close
**
** Description      This function is close wire communication.
**
** Parameters:      none
**
** Returns          common_if function return value.
**
*******************************************************************************/
int nf_felica_close(void)
{
    DATA_FORMAT *p_data = felica_close;
    int ret;

    ret = common_if(p_data);

    return ret;

}


/*******************************************************************************
**
** Function         create_send_data
**
** Description      This function is create nci command.
**
** Parameters:      send_data - (input) pointer to send_data.
**                  send_size - (input) size of send_data.
**
** Returns          nci command data size
**
*******************************************************************************/
static int create_send_data(unsigned char *send_data, unsigned short send_size)
{
    int len = 0;

    if(send_size > 255)
    {
        DBG_LOGF_ERROR("create_send_data send_size over\n");
        return 0;
    }

    len = send_size + 3;

    nci_data[0] = 0x02;
    nci_data[1] = 0x00;
    nci_data[2] = send_size;
    memcpy(&nci_data[3], send_data, send_size);

    DBG_LOGF_WARN("create_send_data send_size=%d, len=%d\n", send_size, len);

    return len;
}


/*******************************************************************************
**
** Function         nf_felica_send
**
** Description      This function is send nci command in wire communication.
**
** Parameters:      send_data - (input) pointer to send_data.
**                  send_size - (input) size of send_data.
**
** Returns          E_OK
**                  E_FAIL
**
*******************************************************************************/
int nf_felica_send(unsigned char *send_data, unsigned short send_size)
{
    int ret;
    unsigned short nci_len;

    nci_len = create_send_data(send_data, send_size);
    if(nci_len == 0)
    {
        DBG_LOG_ERROR("create_send_data() failed\n");
        return(E_FAIL);
    }

    DBG_LOGF_DEBUG("nf_felica_send() nci_len=%d\n", nci_len);

    ret = hostif_send(nci_data, nci_len);
    if(ret != E_OK)
    {
        DBG_LOG_ERROR("hostif_send() failed\n");
        return(E_FAIL);
    }

    return(E_OK);
}

/*******************************************************************************
**
** Function         nf_fw_update
**
** Description      
**
** Parameters:      
**
** Returns          
**
*******************************************************************************/
int nf_fw_update(void)
{
    DATA_FORMAT *pdata = fw_patch_update;
    int tbl_loop = LOOP_IN;
    int tbl_posi = 0;
    unsigned char file_loop_flg = 0;
    NVM_VER nvm_ver;

    DBG_LOG_DEBUG("nf_fwupdate() start\n");

#ifdef CXD224X_I2C
    /* receive CORE_RESET_NTF */
    if(proc_com_recv(pdata, &tbl_posi) != E_OK)
        return(E_FAIL);
#endif /* CXD224X_I2C */

    /* CORE RESET */
    if(proc_com_send(pdata, &tbl_posi) != E_OK)
        return(E_FAIL);
    if(proc_com_recv(pdata, &tbl_posi) != E_OK)
        return(E_FAIL);
    hostif_get_version(&nvm_ver, GET_HW_VER);

    /* Get HW VERSION */
    if(proc_com_send(pdata, &tbl_posi) != E_OK)
        return(E_FAIL);
    if(proc_com_recv(pdata, &tbl_posi) != E_OK)
        return(E_FAIL);
    hostif_get_version(&nvm_ver, GET_HW_VER);

    /* Get INTERNAL VER */
    if(proc_com_send(pdata, &tbl_posi) != E_OK)
        return(E_FAIL);
    if(proc_com_recv(pdata, &tbl_posi) != E_OK)
        return(E_FAIL);
    hostif_get_version(&nvm_ver, GET_FW_VER);

    /* Get PATCH VER */
    if(proc_com_send(pdata, &tbl_posi) != E_OK)
        return(E_FAIL);
    if(proc_com_recv(pdata, &tbl_posi) != E_OK)
        return(E_FAIL);
    hostif_get_version(&nvm_ver, GET_PATCH_VER);

    /* compare patch */
    if(update_chk_need_update(&nvm_ver) != E_OK)
    {
        DBG_LOG_WARN("chk_need_update() failed\n");
        return(E_FAIL);
    }

    /* PATCH UPDATE */
    while(tbl_loop == LOOP_IN)
    {
        switch((pdata + tbl_posi)->type)
        {
        case SEND:
            if(update_create_nci_patch_cmd((pdata + tbl_posi)->data, &file_loop_flg) != E_OK)
            {
                DBG_LOG_ERROR("create_nci_patch_cmd() failed\n");
                return(E_FAIL);
            }

            if(proc_com_send(pdata, &tbl_posi) != E_OK)
                return(E_FAIL);
            break;

        case RECV:
            if(proc_com_recv(pdata, &tbl_posi) != E_OK)
                return(E_FAIL);

            if(file_loop_flg == REPEAT_CMD)
            {
                tbl_posi -= 2;
            }
            break;

        case NON:
            tbl_loop = LOOP_OUT;
            break;

        default:
            tbl_loop = LOOP_OUT;
            break;
        }
    }

    DBG_LOG_DEBUG("nf_fwupdate() end\n");
    return (E_OK);
}

