/****************************************************************************
 * modules/lte/farapi/api/socket/farapi_recvfrom.c
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
#include "apicmd_recvfrom.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "bswap.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RECVFROM_REQ_DATALEN (sizeof(struct apicmd_recvfrom_s))
#define RECVFROM_RES_DATALEN (sizeof(struct apicmd_recvfromres_s))

#define RECVFROM_REQ_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct recvfrom_req_s
{
  int                        sockfd;
  FAR void                   *buf;
  size_t                     len;
  int                        flags;
  FAR struct farapi_sockaddr *from;
  FAR farapi_socklen_t       *fromlen;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: recvfrom_request
 *
 * Description:
 *   Send FARAPI_RECVFROM_REQ.
 *
 ****************************************************************************/

static int32_t recvfrom_request(FAR struct farapi_socket_s *fsock,
                                FAR struct recvfrom_req_s *req)
{
  int32_t                         ret;
  int32_t                         err;
  uint32_t                        resplen;
  uint16_t                        reslen = 0;
  FAR struct apicmd_recvfrom_s    *cmd = NULL;
  FAR struct apicmd_recvfromres_s *res = NULL;

  /* Calculate the request command size */

  resplen = RECVFROM_RES_DATALEN + req->len - sizeof(res->recvdata);

  /* Allocate send and response command buffer */

  APIUTIL_SOCK_ALLOC_CMDANDRESBUFF(cmd, APICMDID_SOCK_RECVFROM,
                                   RECVFROM_REQ_DATALEN,
                                   res, resplen);

  /* Fill the data */

  cmd->sockfd  = bswap32(req->sockfd);
  cmd->flags   = bswap32(req->flags);
  cmd->recvlen = bswap32(req->len);
  if (req->fromlen)
    {
      cmd->fromlen = bswap32(*req->fromlen);
    }
  else
    {
      cmd->fromlen = bswap32(0);
    }

  DBGIF_LOG3_DEBUG("[recvfrom-req]sockfd: %d, flags: %d, recvlen: %d\n", req->sockfd, req->flags, req->len);
  if (req->fromlen)
    {
      DBGIF_LOG1_DEBUG("[recvfrom-req]fromlen: %d\n", *req->fromlen);
    }

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      resplen, &reslen, SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      err = -ret;
      goto errout_with_cmdfree;
    }

  if (reslen != resplen)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      err = FARAPI_EFAULT;
      goto errout_with_cmdfree;
    }

  ret = bswap32(res->ret_code);
  err = bswap32(res->err_code);

  DBGIF_LOG2_DEBUG("[recvfrom-res]ret: %d, err: %d\n", ret, err);

  if (APICMD_RECVFROM_RES_RET_CODE_ERR == ret)
    {
      DBGIF_LOG1_ERROR("API command response is err :%d.\n", err);
      goto errout_with_cmdfree;
    }
  else
    {
      if (req->len < ret)
        {
          DBGIF_LOG1_ERROR("Unexpected recv data length: %d\n", ret);
          err = FARAPI_EFAULT;
          goto errout_with_cmdfree;
        }
      memcpy(req->buf, res->recvdata, ret);

      DBGIF_LOG1_DEBUG("[recvfrom-res]fromlen: %d\n", bswap32(res->fromlen));

      if (req->from)
        {
          if (req->fromlen)
            {
              if (*req->fromlen < bswap32(res->fromlen))
                {
                  DBGIF_LOG2_INFO("Input fromlen: %d, Output fromlen: %d\n", *req->fromlen, bswap32(res->fromlen));
                }
              memcpy(req->from, &res->from, *req->fromlen);
            }
          else
            {
              DBGIF_LOG_ERROR("Unexpected. fromlen is NULL.\n");
            }
        }
      if (req->fromlen)
        {
          *req->fromlen = bswap32(res->fromlen);
        }
    }

  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);

  return ret;

errout_with_cmdfree:
  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);
  farapi_seterrno(err);
  return RECVFROM_REQ_FAILURE;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: farapi_recvfrom
 *
 * Description:
 *   farapi_recvfrom() receives messages from a socket, and may be used to
 *   receive data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags
 *   from     Address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters received. On error,
 *   -1 is returned, and errno is set appropriately.
 *
 ****************************************************************************/

int farapi_recvfrom(int sockfd, void *buf, size_t len, int flags,
                    struct farapi_sockaddr *from, farapi_socklen_t *fromlen)
{
  int32_t                    ret;
  int32_t                    result;
  bool                       is_init;
  struct farapi_fd_set_s     readset;
  FAR struct farapi_socket_s *fsock;
  struct recvfrom_req_s      req;
  FAR struct farapi_timeval  *recvtimeo;

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

  if (len > APICMD_RECVFROM_RES_RECVDATA_LENGTH)
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

  if (from && (!fromlen))
    {
      DBGIF_LOG_ERROR("fromlen is NULL\n");
      farapi_seterrno(FARAPI_EINVAL);
      return -1;
    }

  req.sockfd  = sockfd;
  req.buf     = buf;
  req.len     = len;
  req.flags   = flags;
  req.from    = from;
  req.fromlen = fromlen;

  if (fsock->flags & FARAPI_O_NONBLOCK)
    {
      /* Check recv buffer is available */

      FARAPI_FD_ZERO(&readset);
      FARAPI_FD_SET(sockfd, &readset);

      ret = farapi_select_nonblock((sockfd + 1), &readset, NULL, NULL);
      if (ret <= 0)
        {
          DBGIF_LOG1_ERROR("select failed: %d\n", farapi_errno());
          return -1;
        }

      if (!FARAPI_FD_ISSET(sockfd, &readset))
        {
          farapi_seterrno(FARAPI_EFAULT);
          DBGIF_LOG1_ERROR("select failed: %d\n", farapi_errno());
          return -1;
        }

      result = recvfrom_request(fsock, &req);

      if (result == RECVFROM_REQ_FAILURE)
       {
         return -1;
       }
    }
  else
    {
      /* Wait until recv buffer is available */

      FARAPI_FD_ZERO(&readset);
      FARAPI_FD_SET(sockfd, &readset);

      recvtimeo = &fsock->recvtimeo;
      if ((fsock->recvtimeo.tv_sec == 0) && (fsock->recvtimeo.tv_usec == 0))
      {
        recvtimeo = NULL;
      }

      ret = farapi_select_block((sockfd + 1), &readset, NULL, NULL, recvtimeo);
      if (ret <= 0)
        {
          if (ret == 0)
            {
              farapi_seterrno(FARAPI_EFAULT);
            }

          DBGIF_LOG1_ERROR("select failed: %d\n", farapi_errno());
          return -1;
        }

      if (!FARAPI_FD_ISSET(sockfd, &readset))
        {
          farapi_seterrno(FARAPI_EFAULT);
          DBGIF_LOG1_ERROR("select failed: %d\n", farapi_errno());
          return -1;
        }

      result = recvfrom_request(fsock, &req);

      if (result == RECVFROM_REQ_FAILURE)
       {
         return -1;
       }
    }

  return result;
}
