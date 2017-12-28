/******************************************************************************
 *
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
#ifndef SPZ1_IMPL
#include <pthread.h>
#include <semaphore.h>
#include <stdlib.h>
#endif /* SPZ1_IMPL */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "config.h"

#ifdef SPZ1_IMPL
#include "pthread_wrapper.h"
#include "syscall_wrapper.h"
#endif /* SPZ1_IMPL */

#ifndef SPZ_IMPL
#include "libnfc_api.h"
#else
#include "nfc/libnfc_api.h"
#ifdef SPZ1_IMPL
#include "../sdk_include/libnfc_api.h"
#endif // SPZ1_IMPL
#endif /* SPZ_IMPL */

#include "NativeNfcManager.h"

#ifdef SPZ1_IMPL
#include "spz_nfc_log.h"
DBG_DEFINE_MODULE(NF);
#define DBG_MODULE NF
#endif //SPZ1_IMPL

#define DEFAULT_TECH_MASK (NFA_TECHNOLOGY_MASK_A \
                           | NFA_TECHNOLOGY_MASK_B \
                           | NFA_TECHNOLOGY_MASK_F )
#define DEFAULT_TECH_MASK_LISTEN    (NFA_TECHNOLOGY_MASK_A \
                           | NFA_TECHNOLOGY_MASK_B \
                          | NFA_TECHNOLOGY_MASK_F )

#define PRESENCE_CHECK_DELAY 100000

typedef struct {
    FUNC_CB cb;
    unsigned char  *buff;
    unsigned short buff_size;
    RESULT_INFO    *result_info;
} TASK_INFO;

static TASK_INFO     task_info;
static pthread_t     cbTask_info;
static bool          g_cb_tsk_stop = true;
#ifdef SPZ2_IMPL
static unsigned char *g_ndef_buf;
static unsigned short g_ndef_size;
#endif //SPZ2_IMPL
sem_t g_nfa_activated;
int   g_event;

#ifdef SPZ1_IMPL
SYS_ApiCallbackId cb_id; /* callback API ID */
#endif

#ifndef SPZ1_IMPL
#define PRINT_ERR(...) printf(__VA_ARGS__)
#define PRINT_WARN(...) printf(__VA_ARGS__)
#define PRINT_DEBUG(...) printf(__VA_ARGS__)
#else
#define PRINT_ERR(...) DBG_LOGF_ERROR(__VA_ARGS__)
#define PRINT_WARN(...) DBG_LOGF_WARN(__VA_ARGS__)
#define PRINT_DEBUG(...) DBG_LOGF_DEBUG(__VA_ARGS__)
#endif /* SPZ1_IMPL */

/*******************************************************************************
**
** Function:        callbackTask
**
** Description:     
**
** Returns:         None
**
*******************************************************************************/
void *callbackTask(void *arg)
{
    TASK_INFO     *p_task_info = (TASK_INFO *)arg;
    bool          isPresent;
    int           event;
    tNFA_STATUS   ret       = NFA_STATUS_OK;
    int           status    = D_NFC_SUCCESS;
#ifdef SPZ1_IMPL
    unsigned short read_ndef_size = 0;
#endif /* SPZ1_IMPL */
#ifdef SPZ2_IMPL
    RESULT_INFO   result;
#endif
    
#ifndef SPZ1_IMPL
    g_ndef_buf = (unsigned char *)malloc(LOCAL_NDEF_MAX);
    if(g_ndef_buf == NULL) {
        PRINT_ERR("%s: malloc error\n", __FUNCTION__);
        return NULL;
    }
#endif /* SPZ1_IMPL */

    g_cb_tsk_stop = false;
    while(1)
    {
        status = D_NFC_SUCCESS;

        sem_wait(&g_nfa_activated);
        if(g_cb_tsk_stop) {
#ifndef SPZ1_IMPL
            free(g_ndef_buf);
#else
            pthread_exit(NULL);
#endif /* SPZ1_IMPL */
            return NULL;
        }

        event = g_event;
        if(event == D_EVENT_TAG_READ) {
            ret = nativeNfcTag_doCheckNdef();
            if(ret == NFA_STATUS_OK) {
#if defined(SPZ2_IMPL)                
                ret = nativeNfcTag_doRead(g_ndef_buf, p_task_info->buff_size, (int*)&g_ndef_size);
#elif defined(SPZ1_IMPL)                
                ret = nativeNfcTag_doRead(p_task_info->buff, p_task_info->buff_size, (int*)&read_ndef_size);
#else
                ret = nativeNfcTag_doRead(g_ndef_buf, &g_ndef_size);
#endif /* SPZ2_IMPL */
            }

            if(ret == NFA_STATUS_OK) {
                status = D_NFC_SUCCESS;
            }
            else if(ret == NFA_STATUS_TIMEOUT) {
#ifdef SPZ1_IMPL
                read_ndef_size = 0;
#endif /* SPZ1_IMPL */
                status = D_NFC_TIMEOUT;
            }
            else if(ret == NCI_STATUS_MSG_SIZE_TOO_BIG) {
                status = D_NFC_NDEF_SIZE_OVER;
            }
            else {
#ifdef SPZ1_IMPL
                read_ndef_size = 0;
#endif /* SPZ1_IMPL */
                status = D_NFC_FAILED;
            }

            if((status == D_NFC_SUCCESS) || (status == D_NFC_NDEF_SIZE_OVER)
            || (status == D_NFC_TIMEOUT)) {
#ifndef SPZ1_IMPL
                result.event     = event;
                //result.ndef_data = g_ndef_buf;
                //result.ndef_size = g_ndef_size;
                result.read_ndef = g_ndef_buf;
                result.read_ndef_size = g_ndef_size;
                result.status    = status;

                if(p_task_info->cb != NULL)
                    (p_task_info->cb)(&result);
#else
                   p_task_info->result_info->event          = event;
                   p_task_info->result_info->read_ndef      = p_task_info->buff;
                   p_task_info->result_info->read_ndef_size = read_ndef_size;
                   p_task_info->result_info->status         = status;
                   SYS_CallApiCallback(cb_id, (void *)p_task_info->result_info);
#endif /* SPZ1_IMPL */
            }

            while(1)
            {
                usleep(PRESENCE_CHECK_DELAY);

                isPresent = nativeNfcTag_doPresenceCheck();
                if((isPresent == FALSE) || (g_cb_tsk_stop == true)) {
                    break;
                }
            }
        }
        else if(g_event == D_EVENT_HCE) {
#ifndef SPZ1_IMPL
            result.event     = event;
            // result.ndef_data = NULL;
            // result.ndef_size = 0;
            result.read_ndef = NULL;
            result.read_ndef_size = 0;
            result.status    = D_NFC_SUCCESS;

            if(p_task_info->cb != NULL)
                (p_task_info->cb)(&result);
#else
             p_task_info->result_info->event          = event;
             p_task_info->result_info->read_ndef      = NULL;
             p_task_info->result_info->read_ndef_size = 0;
             p_task_info->result_info->status         = D_NFC_SUCCESS;
             SYS_CallApiCallback(cb_id, (void *)p_task_info->result_info);
#endif /* SPZ1_IMPL */            
        }
        else if(g_event == D_EVENT_ERROR) {
#ifndef SPZ1_IMPL
            result.event     = event;
            // result.ndef_data = NULL;
            // result.ndef_size = 0;
            result.read_ndef = NULL;
            result.read_ndef_size = 0;
            result.status    = D_NFC_FAILED;

            if(p_task_info->cb != NULL)
                (p_task_info->cb)(&result);
#else
             p_task_info->result_info->event          = event;
             p_task_info->result_info->read_ndef      = NULL;
             p_task_info->result_info->read_ndef_size = 0;
             p_task_info->result_info->status         = D_NFC_FAILED;
             SYS_CallApiCallback(cb_id, (void *)p_task_info->result_info);
#endif /* SPZ1_IMPL */
        } 


    }

#ifndef SPZ1_IMPL
    free(g_ndef_buf);
#else
    pthread_exit(NULL);
#endif /* SPZ1_IMPL */
    return NULL;
}

/*******************************************************************************
**
** Function:        NF_Start
**
** Description:     start reader and/or HCE.
**
** Returns:         D_NF_SUCCESS if ok.
**                  D_NF_PARAM_ERR if parameter error.
**                  D_NF_DUPL_ERR if duplicate nfc start.
**                  D_NF_PROC_ERR if unexpected error.
**
*******************************************************************************/
int NF_Start(unsigned char *ndef_data, unsigned short ndef_size, int reader_hce_mode, unsigned char *buff, unsigned short buff_size, FUNC_CB funcCb, RESULT_INFO *result_info)
{
    bool ret;
    bool readonly = true;
    bool reader_mode = false;
    tNFA_TECHNOLOGY_MASK tech_mask = 0x07;
    tNFA_TECHNOLOGY_MASK tech_mask_listen = 0x03;
    unsigned long num;

    /*
     * parameter check
     */
    if(result_info == NULL) {
        PRINT_ERR("%s: result_info NULL\n", __FUNCTION__);
        return D_NF_PARAM_ERR;
    }

    if(reader_hce_mode == READ_AND_HCE ) {
        if((ndef_data == NULL) || (ndef_size == 0)) {
            PRINT_ERR("%s: R and H ndef_data NULL or ndef_size 0\n", __FUNCTION__);
            return D_NF_PARAM_ERR;
        }
        if((buff == NULL) || (buff_size == 0)) {
            PRINT_ERR("%s: R and H buff NULL or buff_size 0\n", __FUNCTION__);
            return D_NF_PARAM_ERR;
        }
    }
    else if(reader_hce_mode == READ_ONLY) {
        if((buff == NULL) || (buff_size == 0)) {
            PRINT_ERR("%s: R ONLY buff NULL or buff_size 0\n", __FUNCTION__);
            return D_NF_PARAM_ERR;
        }
    }
    else if(reader_hce_mode == HCE_ONLY) {
        if((ndef_data == NULL) || (ndef_size == 0)) {
            PRINT_ERR("%s: H ONLY ndef_data NULL or ndef_size 0\n", __FUNCTION__);
            return D_NF_PARAM_ERR;
        }
    }
    else {
        PRINT_ERR("%s: wrong mode\n", __FUNCTION__);
        return D_NF_PARAM_ERR;
    }

    /*
     * duplicated check
     */
    if(g_cb_tsk_stop == false)
    {
        PRINT_ERR("%s: Duplicate error\n", __FUNCTION__);
        return D_NF_DUPL_ERR;
    }
    g_cb_tsk_stop = false;

    /*
     * get poll tech mask
     */
    num = 0;
    if(GetNumValue(NAME_POLLING_TECH_MASK, &num, sizeof(num))) {
        tech_mask = num;
    }
    else {
        tech_mask = DEFAULT_TECH_MASK;
    }
    PRINT_DEBUG ("%s: tag polling tech mask=0x%X\n", __FUNCTION__, tech_mask);

    /*
     * get listen tech mask
     */
    num = 0;
    if(GetNumValue(NAME_UICC_LISTEN_TECH_MASK, &num, sizeof(num))) {
        tech_mask_listen = num;
    }
    else {
        tech_mask_listen = DEFAULT_TECH_MASK_LISTEN;
    }
    PRINT_DEBUG ("%s: listening tech mask=0x%X\n", __FUNCTION__, tech_mask_listen);

    sem_init(&g_nfa_activated, 0, 0);
#ifdef SPZ1_IMPL
    cb_id = SYS_GetApiCallbackId(funcCb);
#endif
    task_info.cb = funcCb;
    task_info.buff = buff;
    task_info.buff_size = buff_size;
    task_info.result_info = result_info;

    if(pthread_create(&cbTask_info, NULL, callbackTask, &task_info) != 0) {
        PRINT_ERR("%s: pthread_create failed\n", __FUNCTION__);
        return D_NF_PROC_ERR;
    }

    ret = nfcManager_doInitialize();
    if(ret) {
        if(reader_hce_mode == READ_AND_HCE) {
            reader_mode = true;
            nfcHce_settingTag(tech_mask_listen, 
                              (UINT8*)ndef_data,
                              ndef_size,
                              LOCAL_NDEF_MAX,
                              readonly);

            nfcManager_enableDiscovery(tech_mask, reader_mode, false);
        }

        if(reader_hce_mode == READ_ONLY) {
            reader_mode      = true;
            tech_mask_listen = 0;
            nfcManager_enableDiscovery(tech_mask, reader_mode, false);
        }

        if(reader_hce_mode == HCE_ONLY) {
            reader_mode = false;
            tech_mask   = 0;
            nfcHce_settingTag(tech_mask_listen, 
                              (UINT8*)ndef_data,
                              ndef_size,
                              LOCAL_NDEF_MAX,
                              readonly);

            nfcManager_enableDiscovery(tech_mask, reader_mode, false);
       }
    }
    else {
        PRINT_ERR("%s: nfcManager_doInitialize failed(%d)\n", __FUNCTION__, ret);
        g_cb_tsk_stop = true;
        sem_post(&g_nfa_activated);
        pthread_join(cbTask_info, NULL);
        sem_destroy(&g_nfa_activated);
        return D_NF_PROC_ERR;
    }

    return D_NF_SUCCESS;
}


/*******************************************************************************
**
** Function:        NF_End
**
** Description:     end reader and/or HCE.
**
** Returns:         D_NF_SUCCESS if ok.
**
*******************************************************************************/
int NF_End(void)
{
    bool ret;

    ret = nfcManager_disableDiscovery();
    if(ret) {
        nfcManager_doDeinitialize();

        g_cb_tsk_stop = true;
        sem_post(&g_nfa_activated);
        pthread_join(cbTask_info, NULL);
        sem_destroy(&g_nfa_activated);
    }

    g_cb_tsk_stop = true;
    return D_NF_SUCCESS;
}

