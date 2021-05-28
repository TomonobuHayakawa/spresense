/****************************************************************************
 * modules/lte/altcom/api/mbedtls/cipher_info_from_string.c
 *
 *   Copyright 2018 Sony Corporation
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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
#include "dbg_if.h"
#include "altcom_errno.h"
#include "altcom_seterrno.h"
#include "apicmd_cipher_info_from_string.h"
#include "apiutil.h"
#include "mbedtls/cipher.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CIPHER_INFO_FROM_STR_REQ_DATALEN (sizeof(struct apicmd_cipher_info_from_string_s))
#define CIPHER_INFO_FROM_STR_RES_DATALEN (sizeof(struct apicmd_cipher_info_from_stringres_s))

#define CIPHER_INFO_FROM_STR_SUCCESS 0
#define CIPHER_INFO_FROM_STR_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cipher_info_from_string_req_s
{
  const char *cipher_name;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mbedtls_cipher_info_t g_cipher_info = {0};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t cipher_info_from_string_request(FAR struct cipher_info_from_string_req_s *req)
{
  int32_t                                        ret;
  uint16_t                                       reslen = 0;
  FAR struct apicmd_cipher_info_from_string_s    *cmd = NULL;
  FAR struct apicmd_cipher_info_from_stringres_s *res = NULL;

  /* Check ALTCOM protocol version */

  if (apicmdgw_get_protocolversion() != APICMD_VER_V1)
    {
      return CIPHER_INFO_FROM_STR_FAILURE;
    }
  /* Allocate send and response command buffer */
  if (req->cipher_name == NULL)
    {
      return CIPHER_INFO_FROM_STR_FAILURE;
    }

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_TLS_CIPHER_INFO_FROM_STRING,
    CIPHER_INFO_FROM_STR_REQ_DATALEN,
    (FAR void **)&res, CIPHER_INFO_FROM_STR_RES_DATALEN))
    {
      return CIPHER_INFO_FROM_STR_FAILURE;
    }

  /* Fill the data */
  memset(cmd->cipher_name, '\0', APICMD_CIPHER_INFO_NAME_LEN);
  strncpy((char*) cmd->cipher_name, req->cipher_name, APICMD_CIPHER_INFO_NAME_LEN-1);

  DBGIF_LOG1_DEBUG("[cipher_info_from_string]cipher name: %s\n", cmd->cipher_name);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      CIPHER_INFO_FROM_STR_RES_DATALEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      goto errout_with_cmdfree;
    }

  if (reslen != CIPHER_INFO_FROM_STR_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->cipher_info);

  DBGIF_LOG1_DEBUG("[cipher_info_from_string res]ret: %d\n", ret);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return CIPHER_INFO_FROM_STR_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/


const mbedtls_cipher_info_t *mbedtls_cipher_info_from_string(const char *cipher_name)
{
  int32_t               result;
  struct cipher_info_from_string_req_s req;

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return NULL;
    }

  req.cipher_name = cipher_name;

  result = cipher_info_from_string_request(&req);

  if (result == CIPHER_INFO_FROM_STR_FAILURE)
    {
      return NULL;
    }
  else
    {
      g_cipher_info.id = result;
      return &g_cipher_info;
    }
}

