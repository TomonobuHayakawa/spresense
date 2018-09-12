/****************************************************************************
 * modules/lte/farapi/api/socket/farapi_close.c
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

#include <stdbool.h>

#include "dbg_if.h"
#include "farapi_sock.h"
#include "farapi_socket.h"
#include "farapi_seterrno.h"
#include "apicmd_close.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "bswap.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CLOSE_REQ_DATALEN (sizeof(struct apicmd_close_s))
#define CLOSE_RES_DATALEN (sizeof(struct apicmd_closeres_s))

#define CLOSE_REQ_SUCCESS 0
#define CLOSE_REQ_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct close_req_s
{
  int sockfd;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: close_request
 *
 * Description:
 *   Send FARAPI_CLOSE_REQ.
 *
 ****************************************************************************/

static int32_t close_request(FAR struct farapi_socket_s *fsock,
                             FAR struct close_req_s *req)
{
  int32_t                      ret;
  int32_t                      err;
  uint16_t                     reslen = 0;
  FAR struct apicmd_close_s    *cmd = NULL;
  FAR struct apicmd_closeres_s *res = NULL;

  /* Allocate send and response command buffer */

  APIUTIL_SOCK_ALLOC_CMDANDRESBUFF(cmd, APICMDID_SOCK_CLOSE,
                                   CLOSE_REQ_DATALEN, res,
                                   CLOSE_RES_DATALEN);

  /* Fill the data */

  cmd->sockfd   = bswap32(req->sockfd);

  DBGIF_LOG1_DEBUG("[close-req]sockfd: %d\n", req->sockfd);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      CLOSE_RES_DATALEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      err = -ret;
      goto errout_with_cmdfree;
    }

  if (reslen != CLOSE_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      err = FARAPI_EFAULT;
      goto errout_with_cmdfree;
    }

  ret = bswap32(res->ret_code);
  err = bswap32(res->err_code);

  DBGIF_LOG2_DEBUG("[close-res]ret: %d, err: %d\n", ret, err);

  if (APICMD_CLOSE_RES_RET_CODE_ERR == ret)
    {
      DBGIF_LOG1_ERROR("API command response is err :%d.\n", err);
      goto errout_with_cmdfree;
    }

  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);

  return CLOSE_REQ_SUCCESS;

errout_with_cmdfree:
  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);
  farapi_seterrno(err);
  return CLOSE_REQ_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: farapi_close
 *
 * Description:
 *   farapi_close() closes a file descriptor of the socket.
 *
 * Parameters:
 *   sockfd   Socket descriptor to close
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately.
 *
 ****************************************************************************/

int farapi_close(int sockfd)
{
  int32_t                    result;
  bool                       is_init;
  FAR struct farapi_socket_s *fsock;
  struct close_req_s         req;

  APIUTIL_ISINIT(is_init);
  if (!is_init)
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      farapi_seterrno(FARAPI_EPERM);
      return -1;
    }

  fsock = farapi_sockfd_socket(sockfd);
  if (!fsock)
    {
      farapi_seterrno(FARAPI_EINVAL);
      return -1;
    }

  memset(fsock, 0, sizeof(struct farapi_socket_s));

  req.sockfd = sockfd;

  result = close_request(fsock, &req);

  if (result == CLOSE_REQ_FAILURE)
   {
     return -1;
   }

  return 0;
}
