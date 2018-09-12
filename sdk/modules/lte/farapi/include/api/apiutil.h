/****************************************************************************
 * modules/lte/farapi/include/api/apiutil.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __MODULES_LTE_FARAPI_INCLUDE_API_APIUTIL_H
#define __MODULES_LTE_FARAPI_INCLUDE_API_APIUTIL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>

#include "osal.h"
#include "thrdpool.h"
#include "dbg_if.h"
#include "apicmdgw.h"
#include "farapi_errno.h"
#include "farapi_seterrno.h"
#include "buffpoolwrapper.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define apiutil_lock()            do { sys_disable_dispatch(); } while(0)
#define apiutil_unlock()          do { sys_enable_dispatch(); } while(0)

#define APIUTIL_CHECK_INITIALIZED_AND_SET(ret) \
  { \
    apiutil_lock(); \
    if (g_lte_initialized) \
      { \
        ret = -EBUSY; \
      } \
    else \
      { \
        g_lte_initialized = true; \
        apiutil_callback_createlock(); \
        ret = 0; \
      } \
    apiutil_unlock(); \
  }

#define APIUTIL_SET_INITIALIZED() \
  { \
    apiutil_lock(); \
    g_lte_initialized = true; \
    apiutil_callback_createlock(); \
    apiutil_unlock(); \
  }

#define APIUTIL_CHECK_FINALIZED_AND_SET(ret) \
  { \
    apiutil_lock(); \
    if (!g_lte_initialized) \
      { \
        ret = -EPERM; \
      } \
    else \
      { \
        g_lte_initialized = false; \
        apiutil_callback_deletelock(); \
        ret = 0; \
      } \
    apiutil_unlock(); \
  }

#define APIUTIL_SET_FINALIZED() \
  { \
    apiutil_lock(); \
    g_lte_initialized = false; \
    apiutil_callback_deletelock(); \
    apiutil_unlock(); \
  }

#define APIUTIL_ISINIT(is_init) \
  { \
    apiutil_lock(); \
    is_init = g_lte_initialized; \
    apiutil_unlock(); \
  }

#define APIUTIL_REG_CALLBACK(ret, tgt_callback, new_callback) \
  { \
    apiutil_callback_lock(); \
    if (tgt_callback) \
      { \
        ret = -EBUSY; \
      } \
    else \
      { \
        tgt_callback = new_callback; \
        ret = 0; \
      } \
    apiutil_callback_unlock(); \
  }

#define APIUTIL_GET_AND_CLR_CALLBACK(ret, tgt_callback, old_callback) \
  { \
    apiutil_callback_lock(); \
    if (!tgt_callback) \
      { \
        ret = -EPERM; \
      } \
    else \
      { \
        old_callback = tgt_callback; \
        tgt_callback = NULL; \
        ret = 0; \
      } \
    apiutil_callback_unlock(); \
  }

#define APIUTIL_CLR_CALLBACK(tgt_callback) \
  { \
    apiutil_callback_lock(); \
    tgt_callback = NULL; \
    apiutil_callback_unlock(); \
  }

#define APIUTIL_IS_ARG_NULL(arg) \
  { \
    if (!arg) \
      { \
        DBGIF_LOG_ERROR("Input argument is NULL.\n"); \
        return -EINVAL; \
      } \
  }

#define APIUTIL_FREE_CMD(dat) \
  do \
    { \
      const int32_t freeret = apicmdgw_freebuff(dat); \
      if (freeret < 0) \
        { \
          DBGIF_LOG1_ERROR("apicmdgw_freebuff() failure. ret:%d\n", freeret); \
        } \
    } \
  while (0)

#define APIUTIL_CHK_AND_LOCK_PROC(flg) \
  do \
    { \
      if (flg) \
        { \
          return -EBUSY; \
        } \
      flg = true; \
    } \
  while (0)

#define APIUTIL_UNLOCK_PROC(flg) \
  do \
    { \
      flg = false; \
    } \
  while (0)

#define APIUTIL_SOCK_ALLOC_CMDBUFF(buff, id ,len) \
  ((buff = apiutil_alloc_cmdbuff(id, len)) != NULL)

#define APIUTIL_SOCK_ALLOC_RESBUFF(buff, len) \
  ((buff = apiutil_alloc_resbuff(len)) != NULL)

#define APIUTIL_SOCK_ALLOC_CMDANDRESBUFF(buff, id, bufflen, res, reslen) \
  do \
    { \
      if (!APIUTIL_SOCK_ALLOC_CMDBUFF(buff, id, bufflen)) \
        { \
          farapi_seterrno((int32_t)FARAPI_ENOMEM); \
          return -1; \
        } \
      if (!APIUTIL_SOCK_ALLOC_RESBUFF(res, reslen)) \
        { \
          APIUTIL_FREE_CMD((FAR uint8_t *)buff); \
          farapi_seterrno((int32_t)FARAPI_ENOMEM); \
          return -1; \
        } \
    } \
  while (0) \

#define APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmdbuff, resbuff) \
  do \
    { \
      if (cmdbuff) \
        { \
          APIUTIL_FREE_CMD((FAR uint8_t *)cmdbuff); \
        }  \
      if (resbuff) \
        { \
          (void)BUFFPOOL_FREE(resbuff); \
        } \
    } \
  while (0) \

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern bool        g_lte_initialized;
extern sys_mutex_t g_lte_apicallback_mtx;

/****************************************************************************
 * Inline functions
 ****************************************************************************/

static inline int32_t APIUTIL_SEND_AND_FREE(FAR uint8_t *dat)
{
  int32_t ret = APICMDGW_SEND_ONLY(dat);
  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("Failed to send API command. err=[%d]\n", ret);
    }

  APIUTIL_FREE_CMD(dat);

  return ret;
}

FAR static inline void *apiutil_alloc_cmdbuff(int32_t cmdid, uint16_t len)
{
  FAR void *buff = NULL;

  buff = apicmdgw_cmd_allocbuff(cmdid, len);
  if (!buff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
    }
  return buff;
}

FAR static inline void *apiutil_alloc_resbuff(uint16_t len)
{
  FAR void *res = NULL;

  res = BUFFPOOL_ALLOC(len);
  if (!res)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      farapi_seterrno((int32_t)FARAPI_ENOMEM);
    }

  return res;
}

/****************************************************************************
 * Name: apiutil_callback_createlock
 *
 * Description:
 *   Create the lock for access to the API callback.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void apiutil_callback_createlock(void)
{
  int32_t      ret;
  sys_cremtx_s param = {0};

  ret = sys_create_mutex(&g_lte_apicallback_mtx, &param);
  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("Failed to create mutex :%d\n", ret);
    }
}

/****************************************************************************
 * Name: apiutil_callback_deletelock
 *
 * Description:
 *   Delete the lock for access to the API callback.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void apiutil_callback_deletelock(void)
{
  int32_t      ret;

  ret = sys_delete_mutex(&g_lte_apicallback_mtx);
  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("Failed to delete mutex :%d\n", ret);
    }
}

/****************************************************************************
 * Name: apiutil_callback_lock
 *
 * Description:
 *   Lock access to the API callback.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void apiutil_callback_lock(void)
{
  sys_lock_mutex(&g_lte_apicallback_mtx);
}

/****************************************************************************
 * Name: apiutil_callback_unlock
 *
 * Description:
 *   Unock access to the API callback.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void apiutil_callback_unlock(void)
{
  sys_unlock_mutex(&g_lte_apicallback_mtx);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: apiutil_runjob
 *
 * Description:
 *  run job to the worker.
 *
 * Input Parameters:
 *  id   workerid
 *  job  workin job pointer.
 *  arg  job parameters pointer.
 *
 * Returned Value:
 *   If the process succeeds, it returns APIUTIL_SUCCESS.
 *   Otherwise APIUTIL_FAILURE is returned.
 *
 ****************************************************************************/

int32_t apiutil_runjob(
  int8_t id,  CODE thrdpool_jobif_t job, FAR void *arg);

#endif /* __MODULES_LTE_FARAPI_INCLUDE_API_APIUTIL_H */
