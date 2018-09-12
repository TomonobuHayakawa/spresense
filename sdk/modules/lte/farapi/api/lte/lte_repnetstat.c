/****************************************************************************
 * modules/lte/farapi/api/lte/lte_setrep_netstat.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <errno.h>

#include "lte/lte_api.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "apicmd_setrepnetstat.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SETREP_NETSTAT_DATA_LEN (sizeof(struct apicmd_cmddat_setrepnetstat_s))
#define SETREP_NETSTAT_RES_DATA_LEN \
  (sizeof(struct apicmd_cmddat_setrepnetstatres_s))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_lte_setnetstat_isproc = false;

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern netstat_report_cb_t g_netstat_report_callback;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_set_report_netstat
 *
 * Description:
 *   Change the report setting of the LTE network state and data
 *   communication state. The default report setting is disable.
 *
 * Input Parameters:
 *   netstat_callback Callback function to notify that LTE network state.
 *                    If NULL is set, the report setting is disabled.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_set_report_netstat(netstat_report_cb_t netstat_callback)
{
  int32_t                                     ret        = 0;
  bool                                        is_init    = false;
  FAR struct apicmd_cmddat_setrepnetstat_s    *cmdbuff   = NULL;
  FAR struct apicmd_cmddat_setrepnetstatres_s *resbuff   = NULL;
  uint16_t                                    resbufflen =
                                                SETREP_NETSTAT_RES_DATA_LEN;
  uint16_t                                    reslen     = 0;

  /* Check if the library is initialized */

  APIUTIL_ISINIT(is_init);
  if (!is_init)
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      return -EPERM;
    }

  /* Check this process runnning. */

  APIUTIL_CHK_AND_LOCK_PROC(g_lte_setnetstat_isproc);

  /* Accept the API */
  /* Allocate API command buffer to send */

  cmdbuff = (FAR struct apicmd_cmddat_setrepnetstat_s *)
    apicmdgw_cmd_allocbuff(APICMDID_SET_REP_NETSTAT,
    SETREP_NETSTAT_DATA_LEN);
  if (!cmdbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      APIUTIL_UNLOCK_PROC(g_lte_setnetstat_isproc);
      return -ENOMEM;
    }
  else
    {
      resbuff = (FAR struct apicmd_cmddat_setrepnetstatres_s *)
        BUFFPOOL_ALLOC(resbufflen);
      if (!resbuff)
        {
          DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
          APIUTIL_FREE_CMD((FAR uint8_t *)cmdbuff);
          APIUTIL_UNLOCK_PROC(g_lte_setnetstat_isproc);
          return -ENOMEM;
        }
      
      /* Set command parameter. */

      cmdbuff->enability = !netstat_callback ?
        APICMD_SETREP_NETSTAT_DISABLE: APICMD_SETREP_NETSTAT_ENABLE;

      /* Send API command to modem */

      ret = apicmdgw_send((FAR uint8_t *)cmdbuff, (FAR uint8_t *)resbuff,
        resbufflen, &reslen, SYS_TIMEO_FEVR);
    }

  if (0 <= ret)
    {
      /* Register API callback */

      if (APICMD_SETREP_NETSTAT_RES_OK == resbuff->result)
        {
          APIUTIL_CLR_CALLBACK(g_netstat_report_callback);
          if (netstat_callback)
            {
              APIUTIL_REG_CALLBACK(
                ret, g_netstat_report_callback, netstat_callback);
            }
        }
      else
        {
          DBGIF_LOG_ERROR("API command response is err.\n");
          ret = -EIO;
        }
    }

  if (0 <= ret)
    {
      ret = 0;
    }

  APIUTIL_FREE_CMD((FAR uint8_t *)cmdbuff);
  (void)BUFFPOOL_FREE(resbuff);
  APIUTIL_UNLOCK_PROC(g_lte_setnetstat_isproc);

  return ret;
}
