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
#include <poll.h>
#include <sys/types.h>
#include <sys/socket.h>

#include "nf_common.h"

#define POLL_TIMEOUT (-1)
#define MSG_TYPE (1)
#define MSG_LEN (260)


typedef struct s_th_params
{
    int active;
} TH_PARAMS;

struct msgbuf
{
    long mtype;
    unsigned char mtext[MSG_LEN];
};

struct msgbuf recv_buf;

sem_t sem_recv;
static pthread_t _th = -1;
static TH_PARAMS _th_params;
pthread_mutex_t _mutex_buf;
pthread_mutex_t _mutex_i2c;
uchar g_rsp_buff[NCI_RSP_MAX_LEN];

static HOSTIF_TYPE _type = HOSTIF_UNKNOWN;

static bool core_reset_ntf_init_flg = false;

extern tNF_STATUS g_nf_status;

extern ssize_t send(int sockfd, const void* buf, size_t len, int flags);
extern ssize_t recv(int sockfd, void *buf, size_t len, int flags);

static int signal_fds[2];
static inline int create_signal(struct pollfd* set)
{
#ifndef SPZ2_IMPL
    if (signal_fds[0] == 0 && socketpair(AF_UNIX, SOCK_STREAM, 0, signal_fds) < 0)
    {
        DBG_LOGF_DEBUG("%s create_signal_sockets:socketpair failed\n", __func__);
        return -1;
    }
#else
    signal_fds[0] = -1;
    signal_fds[1] = -2;
#endif /* SPZ2_IMPL */

    set->fd = signal_fds[0];
    return signal_fds[0];
}
    
static inline void close_signal(void)
{
#ifndef SPZ2_IMPL
    int stat = 0;

    stat = close(signal_fds[0]);
    if (stat == -1)
        DBG_LOGF_DEBUG("%s, fail close index 0;\n", __FUNCTION__);
    signal_fds[0] = 0;

    stat = close(signal_fds[1]);
    if (stat == -1)
        DBG_LOGF_DEBUG("%s, fail close index 1;\n", __FUNCTION__);
    signal_fds[1] = 0;
#else
    signal_fds[0] = 0;
    signal_fds[1] = 0;
#endif /* SPZ2_IMPL */
}

static inline int send_signal(void)
{
    char sig_on = 1;

    DBG_LOGF_DEBUG("%s: Sending signal to %d\n", __func__, signal_fds[1]);
    return send(signal_fds[1], &sig_on, sizeof(sig_on), 0);
}

static inline int reset_signal(void)
{
    char sig_recv = 0;
    DBG_LOGF_DEBUG("%s: Receiving signal from %d\n", __func__, signal_fds[0]);
    recv(signal_fds[0], &sig_recv, sizeof(sig_recv), MSG_WAITALL);
    return (int)sig_recv;
}

static inline int is_signaled(struct pollfd* set)
{
    return ((set->revents & POLLIN) == POLLIN) || ((set->revents & POLLRDNORM) == POLLRDNORM) ;
}
/*******************************************************************************
**
** Function         hostif_chk_err_ntf
**
** Description      This function is
**
** Parameters:      recv_data - (input) pointer to notification data.
**
** Returns          none
**
*******************************************************************************/
static void hostif_chk_err_ntf(const uchar *ntf)
{
#ifdef NF_INVALID_ERROR_NTF
    if(((ntf[0] & 0xE0) >> 5) == NCI_MT_NTF)
    {
        if((ntf[0] & 0x0F) == 0)
        {
            /* CORE_GENERIC_ERROR_NTF */
            if((ntf[1] & 0x3F) == 0x07)
            {
                if(ntf[3] == 0xf0)
                {
                    DBG_LOG_ERROR("recv CORE_GENERIC_ERROR_NTF.\n");
                    err_ntf_flg = true;
                }
            }
            /* CORE_RESET_NTF */
            else if((ntf[1] & 0x3F) == 0x00)
            {
                if(core_reset_ntf_init_flg == true)
                {
                    DBG_LOG_ERROR("recv CORE_RESET_NTF.\n");
                    err_ntf_flg = true;
                }
                else
                {
                    core_reset_ntf_init_flg = true;
                }
            }
        }
    }
#endif /* NF_INVALID_ERROR_NTF */
}

/*******************************************************************************
**
** Function         hostif_chk_recv_msg
**
** Description      This function is check receive data.
**
** Parameters:      recv_data - (input) pointer to receive data.
**
** Returns          0 ignore
**                  1 notify
**                  2 notify to user
**
*******************************************************************************/
static int hostif_chk_recv_msg(unsigned char *recv_data)
{
    int ret = 0;
    NCI_MT mt = NCI_MT_UNKNOWN;

    DBG_LOGF_DEBUG("hostif_chk_recv_msg() start.\n");

    /* get mt */
    mt = hostif_get_nci_msg_type(recv_data);
    hostif_chk_err_ntf(recv_data);

    if((g_nf_status == NF_ST_STARTING) || (g_nf_status == NF_ST_END))
    {
        if(err_ntf_flg == true)
        {
            /* when recieve ERROR_NTF, notify to user. ignore CREDIT_NTF*/
            DBG_LOGF_ERROR("hostif_chk_recv_msg() status=%d, mt=%d.\n", g_nf_status, mt);
            ret = 2;/* notify data */
        }
        else
        {
            DBG_LOGF_DEBUG("hostif_chk_recv_msg() status=%d, mt=%d.\n", g_nf_status, mt);
            ret = 1; /* notify data */
        }
    }
    else if(g_nf_status == NF_ST_ENABLE)
    {
        if(mt == NCI_MT_RSP)
        {
            DBG_LOGF_DEBUG("hostif_chk_recv_msg() status=%d, mt=%d.\n", g_nf_status, mt);
            ret = 1; /* notify data */
        }
        else if(err_ntf_flg == true)
        {
            /* when recieve ERROR_NTF, notify to user. ignore CREDIT_NTF*/
            DBG_LOGF_ERROR("hostif_chk_recv_msg() status=%d, mt=%d.\n", g_nf_status, mt);
            ret = 2;/* notify data */
        }
        else
        {
            DBG_LOGF_DEBUG("hostif_chk_recv_msg() status=%d, mt=%d.\n", g_nf_status, mt);
            ret = 0; /* ignore data */
        }
    }
    else if(g_nf_status == NF_ST_FE_OPEN)
    {
        if(mt == NCI_MT_DAT)
        {
            /* when recieve DATA, notify to user */
            DBG_LOGF_DEBUG("hostif_chk_recv_msg() status=%d, mt=%d.\n", g_nf_status, mt);
            ret = 2; /* notify data */
        }
        else if(mt == NCI_MT_RSP)
        {
            DBG_LOGF_DEBUG("hostif_chk_recv_msg() status=%d, mt=%d.\n", g_nf_status, mt);
            ret = 1; /* notify data */
        }
        else if(err_ntf_flg == true)
        {
            /* when recieve ERROR_NTF, notify to user. ignore CREDIT_NTF*/
            DBG_LOGF_ERROR("hostif_chk_recv_msg() status=%d, mt=%d.\n", g_nf_status, mt);
            ret = 2;/* notify data */
        }
        else
        {
            DBG_LOGF_DEBUG("hostif_chk_recv_msg() status=%d, mt=%d.\n", g_nf_status, mt);
            ret = 0; /* ignore data */
        }
    }
    else if(g_nf_status == NF_ST_UPDATING)
    {
        if(mt == NCI_MT_RSP)
        {
            DBG_LOGF_DEBUG("hostif_chk_recv_msg() status=%d, mt=%d.\n", g_nf_status, mt);
            ret = 1; /* notify data */
        }
        else
        {
            DBG_LOGF_DEBUG("hostif_chk_recv_msg() status=%d, mt=%d.\n", g_nf_status, mt);
            ret = 1; /* notify data */
        }
    }

    DBG_LOGF_DEBUG("hostif_chk_recv_msg() end.\n");
    return ret; /* notify data */
}

/*******************************************************************************
**
** Function         recv_task
**
** Description      This function is task receiving data.
**
** Parameters:      thdata - (input) point to thread parameters.
**
** Returns          none
**
*******************************************************************************/
static void *recv_task(void *thdata)
{

    struct pollfd pfd[2];
    TH_PARAMS *params = (TH_PARAMS *)thdata;
    int recv_len;
    int rc;
    int divide_flg; /* divide next process */
    int cnt;        /* loop count of recv data for print */
    int count = 0;
    int offset = 0;

    DBG_LOG_DEBUG("recv_task() start.\n");

#ifdef CXD224X_I2C
    if((NULL == thdata) || (nfc_i2c_get_fd() < 0))
#else
    if((NULL == thdata) || (uart_get_fd() < 0))
#endif /* CXD224X_I2C */
    {
        DBG_LOG_ERROR("Error: recv_task(), Condition is will not fit.\n");
        return (void *)NULL;
    }

#ifdef CXD224X_I2C
    pfd[0].fd = nfc_i2c_get_fd();
#else
    pfd[0].fd = uart_get_fd();
#endif /* CXD224X_I2C */
    pfd[0].events = POLLIN | POLLERR | POLLRDNORM;
    pfd[0].revents = 0;
  
    create_signal(&pfd[1]);
    pfd[1].events = POLLIN | POLLERR | POLLRDNORM;
    pfd[1].revents = 0;

    while(0 != params->active)
    {
        rc = poll(pfd, 2, POLL_TIMEOUT);
        if(0 < rc)
        {
            // for Debug
            DBG_LOGF_DEBUG("recv_task(), poll(), pfd.revents=0x%08X\n", pfd[0].revents);

            if(is_signaled(&pfd[1]))
            {
                DBG_LOGF_DEBUG("reset\n");
                reset_signal();
                continue;
            }

            recv_buf.mtype = MSG_TYPE;

            rc = pthread_mutex_lock(&_mutex_i2c);
            if (0 != rc)
            {
                DBG_LOGF_ERROR("Error: recv_task(), pthread_mutex_lock(I2C)\n");
                continue;
            }

#ifdef CXD224X_I2C
            nfc_i2c_ctrl_io(I2C_IO_WAKE_CTL, I2C_IO_LOW);
#endif /* CXD224X_I2C */

            //if timeout, set pon low.
            nf_start_timer();

            count = 3;
            do {
                if(count < 0)
                    break;

#ifdef CXD224X_I2C
                recv_len = nfc_i2c_read(recv_buf.mtext + offset, 255);
#else
                recv_len = uart_read(recv_buf.mtext + offset, count);
#endif /* CXD224X_I2C */
                if(recv_len <= 0)
                { 
                    DBG_LOGF_ERROR("ERROR: read() recv_len worng =%d\n", recv_len);
                    break;
                }

                if(offset < 3)
                {
                    if(recv_len  + offset == 3)
                    {
                        count = recv_buf.mtext[2];
                    }
                    else if(recv_len + offset > 3)
                    {
                        count = 3 + recv_buf.mtext[2] - recv_len;
                    }
                    else
                    {
                        count -= recv_len;
                    }
                    offset += recv_len;
                }
                else
                {
                    offset += recv_len;
                    count -= recv_len;
                }
                if(count == 0)
                {
                    recv_len = offset;
                    offset = 0;
                    break;
                }
            } while(count > 0);

            DBG_LOGF_DEBUG("read() size=%d ", recv_len);
            for(cnt = 0; cnt < recv_len; cnt++) {
                DBG_LOGF_DEBUG("0x%02x ", recv_buf.mtext[cnt]);
            }
            DBG_LOGF_DEBUG("\n");

            rc = pthread_mutex_unlock(&_mutex_i2c);
            if (0 != rc)
            {
                DBG_LOGF_ERROR("Error: recv_task(), pthread_mutex_unlock(I2C)\n");
            }

            if (0 < recv_len)
            {
                divide_flg = hostif_chk_recv_msg(recv_buf.mtext);
                if(divide_flg == 0)
                {
                    /* nothing */
                    DBG_LOGF_DEBUG("ignore msg\n");
                }
                else if(divide_flg == 1)
                {
                    pthread_mutex_lock(&_mutex_buf);
                    enqueue(recv_buf.mtext);
                    pthread_mutex_unlock(&_mutex_buf);

                    DBG_LOGF_DEBUG("sem_post sem_recv\n");
                    sem_post(&sem_recv);
                }
                else if(divide_flg == 2)
                {
                    DBG_LOGF_DEBUG("sem_post sem_cb_wait\n");
                    sem_post(&sem_cb_wait);
                }
            }
            else
            {
                DBG_LOGF_DEBUG("sem_post sem_recv\n");
                sem_post(&sem_recv);
            }
        }
        else if (rc == 0)
        {
            // for Debug
            DBG_LOGF_DEBUG("recv_task(), poll(), timeout!\n");
        }
        else
        {
            DBG_LOGF_ERROR("Error: recv_task(), poll()\n");
        }
    }

    DBG_LOGF_DEBUG("recv_task() end.\n");

    pthread_exit(NULL);
    return (void *)NULL;
}

/*******************************************************************************
**
** Function         hostif_open
**
** Description      This function is open host interface.
**
** Parameters:      type - (input) host interface type
**                  param - (input) not use
**
** Returns          E_OK
**                  E_FAIL
**
*******************************************************************************/
int hostif_open(HOSTIF_TYPE type, void *param)
{
    int result = E_FAIL;
    int rc;
    // "param" <-- Ignored

#ifdef CXD224X_I2C
    result = nfc_i2c_open();
#else
    result = uart_open();
#endif /* CXD224X_I2C */
    if (E_OK != result) goto HOSTIF_OPEN_END;

    //Create Queue
    create_queue();

    // Mutex
    pthread_mutex_init(&_mutex_buf, NULL);
    pthread_mutex_init(&_mutex_i2c, NULL);

    sem_init(&sem_recv, 0, 0);

    nf_create_timer();

    // Thread
    _th_params.active = 1;

    rc = pthread_create(&_th, NULL, recv_task, (void *)&_th_params);
    if (rc != 0)
    {
        DBG_LOGF_ERROR("Error: hostif_open(), pthread_create()\n");
        goto HOSTIF_OPEN_END;
    }

#ifdef CXD224X_I2C
    nfc_i2c_ctrl_io(I2C_IO_RST_CTL, I2C_IO_LOW);
#endif /* CXD224X_I2C */

    result = E_OK;
    _type = type;

HOSTIF_OPEN_END:
    return result;
}

/*******************************************************************************
**
** Function         hostif_close
**
** Description      This function is clse host interface.
**
** Parameters:      none
**
** Returns          none
**
*******************************************************************************/
void hostif_close(void)
{
    send_signal();
        
   if (0 <= _th)
    {
        _th_params.active = 0;
        pthread_join(_th, NULL);
        usleep(100 * 1000);
    }
    _th = -1;

    nf_delete_timer();
#ifdef CXD224X_I2C
    nfc_i2c_ctrl_io(I2C_IO_WAKE_CTL, I2C_IO_HI);
#endif /* CXD224X_I2C */

    sem_destroy(&sem_recv);

    //Delete Queue
    delete_queue();

    close_signal();

    pthread_mutex_destroy(&_mutex_buf);
    pthread_mutex_destroy(&_mutex_i2c);

#ifdef CXD224X_I2C
    nfc_i2c_close();
#else
    uart_close();
#endif /* CXD224X_I2C */
    core_reset_ntf_init_flg = false;

    return;
}


/*******************************************************************************
**
** Function         hostif_send
**
** Description      This function is send host interface.
**
** Parameters:      data - (input) point to send data.
**                  len - (input) size of send data.
**
** Returns          E_OK
**                  E_FAIL
**
*******************************************************************************/
int hostif_send(const uchar *data, int len)
{
    int result = E_FAIL;
    int rc;  /* return of function */
    int cnt; /* loop count of send data for print */

    if (NULL == data || len <= 0)
    {
        return E_PARAM;
    }

    rc = pthread_mutex_lock(&_mutex_i2c);
    if (0 != rc)
    {
        DBG_LOGF_ERROR("Error: hostif_send(), pthread_mutex_lock()\n");
    }

#ifdef CXD224X_I2C
    nfc_i2c_ctrl_io(I2C_IO_WAKE_CTL, I2C_IO_LOW);
#endif /* CXD224X_I2C */

    //if timeout, set pon low.
    nf_start_timer();

#ifdef CXD224X_I2C
    result = nfc_i2c_write(data, len);
#else
    result = uart_write(data, len);
#endif /* CXD224X_I2C */

    rc = pthread_mutex_unlock(&_mutex_i2c);
    if (0 != rc)
    {
        DBG_LOGF_ERROR("Error: hostif_send(), pthread_mutex_unlock()\n");
        goto HOSTIF_SEND_END;
    }

    DBG_LOGF_DEBUG("write() size=%d ", len);
    for(cnt = 0; cnt < len; cnt++){
       DBG_LOGF_DEBUG("0x%02x ", data[cnt]);
    }
    DBG_LOGF_DEBUG("\n");

HOSTIF_SEND_END:
    return result;
}

/*******************************************************************************
**
** Function         hostif_get_data
**
** Description      This function is get data.
**
** Parameters:      buff - (input) pointer to recieved data
**                  len - (input) length of recieved data
**
** Returns          recieved data length
**
*******************************************************************************/
int hostif_get_data(uchar *buff, int len)
{
    int result = 0;
    int recv_len = 0;
    int payload_len = 0;
    bool err;
    unsigned char data[258];
    int rc;
 
    rc = pthread_mutex_lock(&_mutex_buf);
    if(0 != rc)
    {
        DBG_LOGF_ERROR("Error: hostif_recv(), pthread_mutex_lock()\n");
        goto HOSTIF_RECV_END;
    }

    if(!is_empty())
    {
        dequeue(data, &err);
    }

    rc = pthread_mutex_unlock(&_mutex_buf);
    if(0 != rc)
    {
        DBG_LOGF_ERROR("Error: hostif_recv(), pthread_mutex_lock()\n");
        goto HOSTIF_RECV_END;
    }
    
    payload_len = (int)data[NCI_RSP_HEADER_SIZE - 1];
    recv_len = (NCI_RSP_HEADER_SIZE + payload_len);
    if (recv_len <= len)
    {
        result = recv_len;
    }
    else
    {
        DBG_LOGF_ERROR("Error: hostif_get_data(), Receive buffer size is too small.\n");
        result = len;
    }
    memcpy((void *)buff, (const void *)data, result);


HOSTIF_RECV_END:
    return result;
}

/*******************************************************************************
**
** Function         hostif_get_fe_data
**
** Description      This function is get data.
**
** Parameters:      buff - (input) pointer to recieved data
**                  len - (input) length of recieved data
**
** Returns          recieved data length
**
*******************************************************************************/
int hostif_get_fe_data(uchar *buff, int len)
{
    int result = 0;
    int payload_len = 0;

    payload_len = (int)recv_buf.mtext[NCI_RSP_HEADER_SIZE - 1];
    if (payload_len <= len)
    {
        result = payload_len;
    }
    else
    {
        DBG_LOGF_ERROR("Error: hostif_get_data(), Receive buffer size is too small.\n");
        result = len;
    }
    memcpy((void *)buff, (const void *)&recv_buf.mtext[NCI_RSP_HEADER_SIZE], result);

    return result;
}

/*******************************************************************************
**
** Function         hostif_recv
**
** Description      This function is wait for receive data.
**
** Parameters:      buff - (output) pointer to receive data
**                  len - (output) size of receive data
**
** Returns          hostif_get_data return vlue
**                  E_PARAM if parameter wrong
**
*******************************************************************************/
int hostif_recv(uchar *buff, int len)
{
    int result = 0;

    if (NULL == buff || len <= 0)
    {
        return E_PARAM;
    }

    sem_wait(&sem_recv);

    result = hostif_get_data(buff, len);

    return result;
}

/*******************************************************************************
**
** Function         hostif_cmp_rsp
**
** Description      This function is compare a received data and a expected data.
**
** Parameters:      rsp
**                  rsp_len
**                  exp
**                  exp_len
**                  mask
**
** Returns          0:match, <>0:unmatch
**
*******************************************************************************/
int hostif_cmp_rsp(
    const uchar *rsp,
    int   rsp_len,
    const uchar *exp,
    int   exp_len,
    const uchar *mask)
{
    int result = E_OK;

    uchar mask_value = 0;
    int i = 0;

    if (NULL == rsp || rsp_len <= 0 || NULL == exp || exp_len <= 0)
    {
        return E_PARAM;
    }

    // for Debug
    //printf("rsp: %s\n", byte_array_to_string(rsp, rsp_len));
    //printf("exp: %s\n", byte_array_to_string(exp, exp_len));
    //printf("msk: %s\n", byte_array_to_string(mask, exp_len));

    if (rsp_len != exp_len)
    {
        DBG_LOGF_ERROR("rsp_len = %d exp_len = %d.\n",rsp_len,exp_len);
        result = E_FAIL;
        goto HOSTIF_CMP_RSP_END;
    }

    for (i = 0; i < rsp_len; i++)
    {
        if (mask != NULL)
        {
            mask_value = mask[i];
        }
        else
        {
            mask_value = 0xFF;
        }

        if ( (rsp[i] & mask_value) != (exp[i] & mask_value) )
        {
            DBG_LOGF_ERROR("rsp = %ld exp = %ld. i= %d\n", (long int)rsp[i] & mask_value, (long int)exp[i] & mask_value, i);
            result = E_FAIL;
            break;
        }
    }

HOSTIF_CMP_RSP_END:
    return result;
}

/*******************************************************************************
**
** Function         hostif_recv_and_cmp
**
** Description      This function is compare a received data and a expected data.
**
** Parameters:      exp
**                  len
**                  mask
**                  wait
**
** Returns          E_OK
**                  E_FAIL
**
*******************************************************************************/
int hostif_recv_and_cmp(
    const uchar *exp,
    int len,
    const uchar *mask,
    int wait)
{
    int result = E_OK;

    int recv_len = 0;
    NCI_MT mt = NCI_MT_UNKNOWN;
    NCI_MT target_mt = NCI_MT_UNKNOWN;

    if (NULL == exp || len <= 0)
    {
        return E_PARAM;
    }

    target_mt = hostif_get_nci_msg_type(exp);
    recv_len = hostif_recv(g_rsp_buff, sizeof(g_rsp_buff));
    if (0 < recv_len)
    {
        mt = hostif_get_nci_msg_type(g_rsp_buff);
        if (mt == target_mt)
        {
            result = hostif_cmp_rsp(g_rsp_buff, recv_len, exp, len, mask);
            if (E_OK != result)
            {
                DBG_LOGF_ERROR("Mismatch\n");
                result = E_FAIL;
            }
        }
        else
        {
            DBG_LOGF_ERROR("*Recv* mt=%d, t_mt=%d\n", mt, target_mt);
            result = E_FAIL;
        }
    }

    return result;
}

/*******************************************************************************
**
** Function         hostif_get_nci_msg_type
**
** Description      This function is judge command type
**
** Parameters:      header - (input) pointer to command header.
**
** Returns          NCI_MT_UNKNOWN
**                  NCI_MT_DAT
**                  NCI_MT_CMD
**                  NCI_MT_RSP
**                  NCI_MT_NTF
**
*******************************************************************************/
NCI_MT hostif_get_nci_msg_type(const uchar *header)
{
    NCI_MT result = NCI_MT_UNKNOWN;
    uchar mt = (uchar)0x00;

    if (NULL == header)
    {
        return NCI_MT_UNKNOWN;
    }

    mt = ( (header[0] & 0xE0) >> 5 );
    result = (NCI_MT)mt;

    // for Debug
    DBG_LOGF_DEBUG("hostif_get_nci_msg_type: HDR[0]=0x%02x, mt=%d\n", header[0], (int)result);

    return result;
}

void hostif_get_version(NVM_VER *nvm_ver, GET_NVM_VER type)
{
    if(type == GET_HW_VER)
    {
        nvm_ver->hw_ver[0] = g_rsp_buff[9];
        nvm_ver->hw_ver[1] = g_rsp_buff[8];

        DBG_LOGF_DEBUG("hostif_get_version: HW[0]=0x%02x, [1]=0x%02x\n", nvm_ver->hw_ver[0], nvm_ver->hw_ver[1]);

    }
    else if(type == GET_FW_VER)
    {
        nvm_ver->fw_ver[0] = g_rsp_buff[5];
        nvm_ver->fw_ver[1] = g_rsp_buff[4];

        DBG_LOGF_DEBUG("hostif_get_version: FW[0]=0x%02x, [1]=0x%02x\n", nvm_ver->fw_ver[0], nvm_ver->fw_ver[1]);

    }
    else if(type == GET_PATCH_VER)
    {
        nvm_ver->patch_ver[0] = g_rsp_buff[8];
        nvm_ver->patch_ver[1] = g_rsp_buff[7];
        nvm_ver->patch_ver[2] = g_rsp_buff[6];
        nvm_ver->patch_ver[3] = g_rsp_buff[5];

        DBG_LOGF_DEBUG("hostif_get_version: PATCH[0]=0x%02x, [1]=0x%02x, [2]=0x%02x, [3]=0x%02x\n",
            nvm_ver->patch_ver[0], nvm_ver->patch_ver[1], nvm_ver->patch_ver[2], nvm_ver->patch_ver[3]);

    }
    return;
}


