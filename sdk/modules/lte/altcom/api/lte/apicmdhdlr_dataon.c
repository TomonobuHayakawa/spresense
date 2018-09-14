/****************************************************************************
 * modules/lte/altcom/api/lte/apicmdhdlr_dataon.c
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

#include <string.h>

#include "lte/lte_api.h"
#include "buffpoolwrapper.h"
#include "apicmd_dataon.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern data_on_cb_t g_dataon_callback;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: attachnet_job
 *
 * Description:
 *   This function is an API callback for data connect.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void dataon_job(FAR void *arg)
{
  int32_t                              ret;
  FAR struct apicmd_cmddat_dataonres_s *data;
  data_on_cb_t                         callback;
  uint32_t                             result;
  uint32_t                             errcause = 0;

  data = (FAR struct apicmd_cmddat_dataonres_s *)arg;

  ALTCOM_GET_AND_CLR_CALLBACK(ret, g_dataon_callback, callback);

  if ((ret == 0) && (callback))
    {
      if (APICMD_DATAON_RES_OK == data->result)
        {
          result = (uint32_t)LTE_RESULT_OK;
        }
      else if (APICMD_DATAON_RES_ERR == data->result)
        {
          result = (uint32_t)LTE_RESULT_ERROR;
          switch(data->errorcause)
            {
              case APICMD_DATAON_RES_ERRCAUSE_WAITENTERPIN:
                {
                  errcause = (uint32_t)LTE_ERR_WAITENTERPIN;
                }
              break;
              case APICMD_DATAON_RES_ERRCAUSE_REJECT:
                {
                  errcause = (uint32_t)LTE_ERR_REJECT;
                }
              break;
              case APICMD_DATAON_RES_ERRCAUSE_MAXRETRY:
                {
                  errcause = (uint32_t)LTE_ERR_MAXRETRY;
                }
              break;
              case APICMD_DATAON_RES_ERRCAUSE_BARRING:
                {
                  errcause = (uint32_t)LTE_ERR_BARRING;
                }
              break;
              case APICMD_DATAON_RES_ERRCAUSE_DETACHED:
                {
                  errcause = (uint32_t)LTE_ERR_DETACHED;
                }
              break;
              default:
                  errcause = (uint32_t)LTE_ERR_UNEXPECTED;
              break;
            }
        }
      else
        {
          result = (uint32_t)LTE_RESULT_CANCEL;
        }

      callback(result, errcause);
    }
  else
    {
      DBGIF_LOG_ERROR("Unexpected!! callback is NULL.\n");
    }

  /* In order to reduce the number of copies of the receive buffer,
   * bring a pointer to the receive buffer to the worker thread.
   * Therefore, the receive buffer needs to be released here. */

  altcom_free_cmd((FAR uint8_t *)arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: apicmdhdlr_dataon
 *
 * Description:
 *   This function is an API command handler for data connection on result.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_DATAON_RES,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_dataon(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt,
    APICMDID_CONVERT_RES(APICMDID_DATAON), dataon_job);
}
