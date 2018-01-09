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

#ifndef __NF_COMMON_H__
#define __NF_COMMON_H__

#include <stdio.h>
#include <pthread.h>
#include <debug.h>

// my type(s)
typedef unsigned char uchar;
typedef unsigned int  uint;
typedef unsigned long ulong;

#define NCI_RSP_HEADER_SIZE (3)	// Response header Size
#define NCI_RSP_MAX_LEN (258)	// CMD(3) + BODY(255)
#define NCI_RSP_LEN_INDEX (2)	// Index of "Length"

//#define RECV_INTERVAL_TIME (1000)	// [uSec] <-- NOT USED
#define RECV_RETRY_COUNT (10)

#define E_OK    (0)
#define E_FAIL  (-1)
#define E_PARAM (-2)

#define TYPEA
//#define TYPEB

#define REPEAT_CMD 0
#define NEXT_CMD   1

#define DBG_LOGF_ERROR printf
#define DBG_LOG_ERROR  printf
#define DBG_LOGF_WARN  printf
#define DBG_LOG_WARN   printf
#define DBG_LOGF_DEBUG printf
#define DBG_LOG_DEBUG  printf

enum {
    NF_ST_IDLE,
    NF_ST_STARTING,
    NF_ST_ENABLE,
    NF_ST_END,
    NF_ST_FE_OPEN,
    NF_ST_UPDATING
};
typedef int tNF_STATUS;

typedef enum
{
    HOSTIF_UNKNOWN = 0,
    HOSTIF_I2C,
    HOSTIF_UART,	// Reserved
} HOSTIF_TYPE;

typedef enum
{
    I2C_IO_RST_CTL = 0,
    I2C_IO_WAKE_CTL,
    I2C_IO_POWER_CTL,	// Reserved
} I2C_IO_TYPE;

typedef enum
{
    I2C_IO_LOW = 0,
    I2C_IO_HI,
} I2C_IO_CTRL;

typedef enum
{
    NCI_MT_UNKNOWN = -1,
    NCI_MT_DAT = 0,
    NCI_MT_CMD = 1,
    NCI_MT_RSP = 2,
    NCI_MT_NTF = 3,
} NCI_MT;

typedef struct{
    unsigned char  hw_ver[2];
    unsigned char  fw_ver[2];
    unsigned char  patch_ver[4];
}NVM_VER;

typedef enum
{
    GET_HW_VER = 0,
    GET_FW_VER,
    GET_PATCH_VER,
} GET_NVM_VER;

typedef struct buf_list{
    unsigned char item[255];
    struct buf_list *next;
} BUF_LIST;

typedef struct {
    BUF_LIST *front;
    BUF_LIST *rear;
} QUEUE;

// Global Variable(s)
//
extern uchar g_rsp_buff[NCI_RSP_MAX_LEN];
extern bool err_ntf_flg;
extern sem_t sem_cb_wait;
extern pthread_mutex_t _mutex_i2c;
extern bool wait_rsp_flg;
extern sem_t sem_recv;

//
// Function Prototype(s)
//
int  hostif_open(HOSTIF_TYPE type, void *param);
void hostif_close(void);
void hostif_reset(void);
int  hostif_send(const uchar *data, int len);
int  hostif_recv(uchar *buff, int len);
int  hostif_cmp_rsp(const uchar *rsp, int rsp_len, const uchar *exp, int exp_len, const uchar *mask);
int  hostif_recv_and_cmp(const uchar *exp, int len, const uchar *mask, int wait);
NCI_MT hostif_get_nci_msg_type(const uchar *header);
int hostif_get_data(uchar *buff, int len);
int hostif_get_fe_data(uchar *buff, int len);
void hostif_get_version(NVM_VER *nvm_ver, GET_NVM_VER type);

int update_get_patch(void);
void update_delete_patch(void);
int update_chk_need_update(NVM_VER *nvm_ver);
int update_create_nci_patch_cmd(unsigned char *pdata, unsigned char *flg);

void ret_check(int *timeout, int ret);

int nf_listen_start(void);
int nf_listen_end(void);
int nf_felica_open(void);
int nf_felica_close(void);
int nf_felica_send(unsigned char *send_data, unsigned short send_size);
int nf_fw_update(void);

int nf_create_timer(void);
int nf_start_timer(void);
int nf_stop_timer(void);
int nf_delete_timer(void);

BUF_LIST *create_list(unsigned char *in_dat, BUF_LIST *next);
void create_queue(void);
void delete_list(BUF_LIST *list);
void delete_queue(void);
bool is_empty(void);
bool enqueue(unsigned char *in_dat);
unsigned char * dequeue(unsigned char *out_dat, bool *err);

int nfc_i2c_open(void);
void nfc_i2c_close(void);
int nfc_i2c_write(const uchar *data, int len);
int nfc_i2c_read(uchar *buff, int len);
int nfc_i2c_get_fd(void);
void nfc_i2c_ctrl_io(I2C_IO_TYPE io_type, I2C_IO_CTRL ctrl);

#endif /* __NF_COMMON_H__ */
