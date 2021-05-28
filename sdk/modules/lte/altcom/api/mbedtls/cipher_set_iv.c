/****************************************************************************
 * modules/lte/altcom/api/mbedtls/cipher_set_iv.c
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
#include "apicmd_cipher_set_iv.h"
#include "apiutil.h"
#include "mbedtls/cipher.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CIPHER_SET_IV_REQ_DATALEN (sizeof(struct apicmd_cipher_set_iv_s))
#define CIPHER_SET_IV_RES_DATALEN (sizeof(struct apicmd_cipher_set_ivres_s))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cipher_set_iv_req_s
{
  uint32_t            id;
  const unsigned char *iv;
  size_t              iv_len;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t cipher_set_iv_request(FAR struct cipher_set_iv_req_s *req)
{
  int32_t                              ret;
  uint16_t                             reslen = 0;
  FAR struct apicmd_cipher_set_iv_s    *cmd = NULL;
  FAR struct apicmd_cipher_set_ivres_s *res = NULL;

  /* Check ALTCOM protocol version */

  if (apicmdgw_get_protocolversion() != APICMD_VER_V1)
    {
      return MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA;
    }

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_TLS_CIPHER_SET_IV, CIPHER_SET_IV_REQ_DATALEN,
    (FAR void **)&res, CIPHER_SET_IV_RES_DATALEN))
    {
      return MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA;
    }

  /* Fill the data */

  cmd->ctx = htonl(req->id);
  memset(cmd->iv, 0, APICMD_CIPHER_SET_IV_LEN);
  memcpy(cmd->iv, req->iv, req->iv_len);
  cmd->iv_len = htonl(req->iv_len);

  DBGIF_LOG1_DEBUG("[cipher_set_iv]ctx id: %d\n", req->id);
  DBGIF_LOG1_DEBUG("[cipher_set_iv]iv_len: %zu\n", req->iv_len);


  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      CIPHER_SET_IV_RES_DATALEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      goto errout_with_cmdfree;
    }

  if (reslen != CIPHER_SET_IV_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);

  DBGIF_LOG1_DEBUG("[cipher_set_iv res]ret: %d\n", ret);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mbedtls_cipher_set_iv(mbedtls_cipher_context_t *ctx,
                          const unsigned char *iv, size_t iv_len)
{
  int32_t                    result;
  struct cipher_set_iv_req_s req;

  if(ctx == NULL || iv == NULL || MBEDTLS_MAX_IV_LENGTH < iv_len)
    {
      return MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA;
    }

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return MBEDTLS_ERR_CIPHER_BAD_INPUT_DATA;
    }

  req.id = ctx->id;
  req.iv = iv;
  req.iv_len = iv_len;

  result = cipher_set_iv_request(&req);

  return result;
}

