/****************************************************************************
 * modules/lte/farapi/api/socket/farapi_fcntl.c
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
#include "apicmd_fcntl.h"
#include "farapi_errno.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "bswap.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FCNTL_REQ_DATALEN (sizeof(struct apicmd_fcntl_s))
#define FCNTL_RES_DATALEN (sizeof(struct apicmd_fcntlres_s))

#define FCNTL_REQ_SUCCESS 0
#define FCNTL_REQ_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fcntl_req_s
{
  int sockfd;
  int cmd;
  int val;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fcntl_request
 *
 * Description:
 *   Send FARAPI_FCNTL_REQ.
 *
 ****************************************************************************/

static int32_t fcntl_request(FAR struct farapi_socket_s *fsock,
                             FAR struct fcntl_req_s *req)
{
  int32_t                      ret;
  int32_t                      err;
  uint16_t                     reslen = 0;
  FAR struct apicmd_fcntl_s    *cmd = NULL;
  FAR struct apicmd_fcntlres_s *res = NULL;

  /* Allocate send and response command buffer */

  APIUTIL_SOCK_ALLOC_CMDANDRESBUFF(cmd, APICMDID_SOCK_FCNTL,
                                   FCNTL_REQ_DATALEN, res,
                                   FCNTL_RES_DATALEN);

  /* Fill the data */

  cmd->sockfd = bswap32(req->sockfd);
  cmd->cmd    = bswap32(req->cmd);
  cmd->val    = bswap32(req->val);

  DBGIF_LOG3_DEBUG("[fcntl-req]sockfd: %d, cmd:%d, val: %d\n", req->sockfd, req->cmd, req->val);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      FCNTL_RES_DATALEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      err = -ret;
      goto errout_with_cmdfree;
    }

  if (reslen != FCNTL_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      err = FARAPI_EFAULT;
      goto errout_with_cmdfree;
    }

  ret = bswap32(res->ret_code);
  err = bswap32(res->err_code);

  DBGIF_LOG2_DEBUG("[fcntl-res]ret: %d, err: %d\n", ret, err);

  if (APICMD_FCNTL_RES_RET_CODE_ERR == ret)
    {
      DBGIF_LOG1_ERROR("API command response is err :%d.\n", err);
      goto errout_with_cmdfree;
    }

  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);

  return FCNTL_REQ_SUCCESS;

errout_with_cmdfree:
  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);
  farapi_seterrno(err);
  return FCNTL_REQ_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: farapi_fcntl
 *
 * Description:
 *   Performs fcntl operations on socket
 *
 * Input Parameters:
 *   sockfd - Socket descriptor of the socket to operate on
 *   cmd    - The fcntl command.
 *   val    - Command-specific arguments
 *
 * Returned Value:
 *   0 is returned on success; -1 is returned on failure and
 *   the errno value is set appropriately.
 *
 ****************************************************************************/

int farapi_fcntl(int sockfd, int cmd, int val)
{
  int                        ret = -1;
  bool                       is_init;
  FAR struct farapi_socket_s *fsock;
  struct fcntl_req_s         req;

  APIUTIL_ISINIT(is_init);
  if (!is_init)
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      farapi_seterrno(FARAPI_ENETDOWN);
      return -1;
    }

  fsock = farapi_sockfd_socket(sockfd);
  if (!fsock)
    {
      farapi_seterrno(FARAPI_EINVAL);
      return -1;
    }

  switch(cmd)
    {
      case FARAPI_GETFL:
        ret = fsock->flags;
        break;

      case FARAPI_SETFL:
        if ((val & ~FARAPI_O_NONBLOCK) == 0)
          {
            if (val & FARAPI_O_NONBLOCK)
              {
                fsock->flags |= FARAPI_O_NONBLOCK;
              }
            else
              {
                fsock->flags &= ~FARAPI_O_NONBLOCK;
              }

            req.sockfd = sockfd;
            req.cmd    = APICMD_FCNTL_REQUEST_SETFL;
            req.val    = val;

            fcntl_request(fsock, &req);
            ret = 0;
          }
        else
          {
            farapi_seterrno(FARAPI_ENOSYS);
          }
        break;

      default:
        farapi_seterrno(FARAPI_ENOSYS);
        break;
    }

  return ret;
}
