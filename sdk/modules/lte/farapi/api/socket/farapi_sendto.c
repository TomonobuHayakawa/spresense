/****************************************************************************
 * modules/lte/farapi/api/socket/farapi_sendto.c
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
#include <stdbool.h>

#include "dbg_if.h"
#include "farapi_socket.h"
#include "farapi_select.h"
#include "farapi_sock.h"
#include "farapi_seterrno.h"
#include "apicmd_sendto.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "bswap.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SENDTO_REQ_DATALEN (sizeof(struct apicmd_sendto_s))
#define SENDTO_RES_DATALEN (sizeof(struct apicmd_sendtores_s))
#define SENDTO_REQ_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sendto_req_s
{
  int                              sockfd;
  FAR const void                   *buf;
  size_t                           len;
  int                              flags;
  FAR const struct farapi_sockaddr *to;
  farapi_socklen_t                 tolen;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sendto_request
 *
 * Description:
 *   Send FARAPI_SENDTO_REQ.
 *
 ****************************************************************************/

static int32_t sendto_request(FAR struct farapi_socket_s *fsock,
                              FAR struct sendto_req_s *req)
{
  int32_t                       ret;
  int32_t                       err;
  uint32_t                      sendlen;
  uint16_t                      reslen = 0;
  FAR struct apicmd_sendto_s    *cmd = NULL;
  FAR struct apicmd_sendtores_s *res = NULL;

  /* Calculate the request command size */

  sendlen = SENDTO_REQ_DATALEN + req->len - sizeof(cmd->senddata);

  /* Allocate send and response command buffer */

  APIUTIL_SOCK_ALLOC_CMDANDRESBUFF(cmd, APICMDID_SOCK_SENDTO, sendlen, res,
                                   SENDTO_RES_DATALEN);

  /* Fill the data */

  cmd->sockfd  = bswap32(req->sockfd);
  cmd->flags   = bswap32(req->flags);
  cmd->datalen = bswap32(req->len);
  memcpy(&cmd->senddata, req->buf, req->len);
  if (req->to)
    {
      farapi_sockaddr_to_sockstorage(req->to, &cmd->to);
      cmd->tolen = bswap32(req->tolen);
    }
  else
    {
      cmd->tolen = bswap32(0);
    }

  DBGIF_LOG3_DEBUG("[sendto-req]sockfd: %d, flags: %d, len: %d\n", req->sockfd, req->flags, req->len);
  DBGIF_LOG1_DEBUG("[sendto-req]tolen: %d\n", req->tolen);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                       SENDTO_RES_DATALEN, &reslen, SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      err = -ret;
      goto errout_with_cmdfree;
    }

  if (reslen != SENDTO_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      err = FARAPI_EFAULT;
      goto errout_with_cmdfree;
    }

  ret = bswap32(res->ret_code);
  err = bswap32(res->err_code);

  DBGIF_LOG2_DEBUG("[sendto-res]ret: %d, err: %d\n", ret, err);

  if (APICMD_SENDTO_RES_RET_CODE_ERR == ret)
    {
      DBGIF_LOG1_ERROR("API command response is err :%d.\n", err);
      goto errout_with_cmdfree;
    }

  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);

  return ret;

errout_with_cmdfree:
  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);
  farapi_seterrno(err);
  return SENDTO_REQ_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: farapi_sendto
 *
 * Description:
 *   If farapi_sendto() is used on a connection-mode (FARAPI_SOCK_STREAM)
 *   socket, the parameters to and 'tolen' are ignored (and the error EISCONN
 *   may be returned when they are not NULL and 0), and the error ENOTCONN is
 *   returned when the socket was not actually connected.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent. On error,
 *   -1 is returned, and errno is set appropriately.
 *
 ****************************************************************************/

int farapi_sendto(int sockfd, const void *buf, size_t len, int flags,
                  const struct farapi_sockaddr *to, farapi_socklen_t tolen)
{
  int32_t                     ret;
  int32_t                     result;
  bool                        is_init;
  struct farapi_fd_set_s      writeset;
  FAR struct farapi_socket_s  *fsock;
  struct sendto_req_s         req;
  FAR struct farapi_timeval   *sendtimeo;

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

  /* Check length of data to send */

  if (len > APICMD_SENDTO_SENDDATA_LENGTH)
    {
      DBGIF_LOG1_ERROR("There is not enough buffer available. len:%d\n", len);
      farapi_seterrno(FARAPI_EMSGSIZE);
      return -1;
    }

  if (!buf)
    {
      DBGIF_LOG_ERROR("buf is NULL\n");
      farapi_seterrno(FARAPI_EINVAL);
      return -1;
    }

  if (to && (!tolen))
    {
      DBGIF_LOG_ERROR("tolen is NULL\n");
      farapi_seterrno(FARAPI_EINVAL);
      return -1;
    }

  req.sockfd = sockfd;
  req.buf    = buf;
  req.len    = len;
  req.flags  = flags;
  req.to     = to;
  req.tolen  = tolen;

  if (fsock->flags & FARAPI_O_NONBLOCK)
    {
      /* Check send buffer is available */

      FARAPI_FD_ZERO(&writeset);
      FARAPI_FD_SET(sockfd, &writeset);

      ret = farapi_select_nonblock((sockfd + 1), NULL, &writeset, NULL);
      if (ret <= 0)
        {
          DBGIF_LOG1_ERROR("select failed: %d\n", farapi_errno());
          return -1;
        }

      if (!FARAPI_FD_ISSET(sockfd, &writeset))
        {
          farapi_seterrno(FARAPI_EFAULT);
          DBGIF_LOG1_ERROR("select failed: %d\n", farapi_errno());
          return -1;
        }

      result = sendto_request(fsock, &req);

      if (result == SENDTO_REQ_FAILURE)
       {
         return -1;
       }
    }
  else
    {
      /* Wait until send buffer is available */

      FARAPI_FD_ZERO(&writeset);
      FARAPI_FD_SET(sockfd, &writeset);

      sendtimeo = &fsock->sendtimeo;
      if ((fsock->sendtimeo.tv_sec == 0) && (fsock->sendtimeo.tv_usec == 0))
      {
        sendtimeo = NULL;
      }

      ret = farapi_select_block((sockfd + 1), NULL, &writeset, NULL, sendtimeo);
      if (ret <= 0)
        {
          if (ret == 0)
            {
              farapi_seterrno(FARAPI_EFAULT);
            }

          DBGIF_LOG1_ERROR("select failed: %d\n", farapi_errno());
          return -1;
        }

      if (!FARAPI_FD_ISSET(sockfd, &writeset))
        {
          farapi_seterrno(FARAPI_EFAULT);
          DBGIF_LOG1_ERROR("select failed: %d\n", farapi_errno());
          return -1;
        }

      result = sendto_request(fsock, &req);

      if (result == SENDTO_REQ_FAILURE)
       {
         return -1;
       }
    }

  return result;
}
