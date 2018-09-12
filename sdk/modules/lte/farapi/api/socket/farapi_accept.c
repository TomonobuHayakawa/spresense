/****************************************************************************
 * modules/lte/farapi/api/socket/farapi_accept.c
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
#include "apicmd_accept.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "bswap.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ACCEPT_REQ_DATALEN (sizeof(struct apicmd_accept_s))
#define ACCEPT_RES_DATALEN (sizeof(struct apicmd_acceptres_s))

#define ACCEPT_REQ_SUCCESS 0
#define ACCEPT_REQ_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct accept_req_s
{
  int                        sockfd;
  FAR struct farapi_sockaddr *addr;
  FAR farapi_socklen_t       *addrlen;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: accept_request
 *
 * Description:
 *   Send FARAPI_ACCEPT_REQ.
 *
 ****************************************************************************/

static int32_t accept_request(FAR struct farapi_socket_s *fsock,
                              FAR struct accept_req_s *req)
{
  int32_t                       ret;
  int32_t                       err;
  int32_t                       result;
  uint16_t                      reslen = 0;
  FAR struct apicmd_accept_s    *cmd = NULL;
  FAR struct apicmd_acceptres_s *res = NULL;

  /* Allocate send and response command buffer */

  APIUTIL_SOCK_ALLOC_CMDANDRESBUFF(cmd, APICMDID_SOCK_ACCEPT,
                                   ACCEPT_REQ_DATALEN, res,
                                   ACCEPT_RES_DATALEN);

  /* Fill the data */

  cmd->sockfd  = bswap32(req->sockfd);
  if (req->addrlen)
    {
      cmd->addrlen = bswap32(*req->addrlen);
    }
  else
    {
      cmd->addrlen = bswap32(0);
    }

  DBGIF_LOG2_DEBUG("[accept-req]sockfd: %d, addrlen: %d\n", req->sockfd, *req->addrlen);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      ACCEPT_RES_DATALEN,
                      &reslen, SYS_TIMEO_FEVR);
  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      err = -ret;
      goto errout_with_cmdfree;
    }

  if (reslen != ACCEPT_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      err = FARAPI_EFAULT;
      goto errout_with_cmdfree;
    }

  ret = bswap32(res->ret_code);
  err = bswap32(res->err_code);

  DBGIF_LOG2_DEBUG("[accept-res]ret: %d, err: %d\n", ret, err);

  if (APICMD_ACCEPT_RES_RET_CODE_ERR == ret)
    {
      DBGIF_LOG1_ERROR("API command response is err :%d.\n", err);
      goto errout_with_cmdfree;
    }
  else
    {
      DBGIF_LOG1_DEBUG("[accept-res]addrlen: %d\n", bswap32(res->addrlen));

      if (req->addr)
        {
          if (req->addrlen)
            {
              if (*req->addrlen < bswap32(res->addrlen))
                {
                  DBGIF_LOG2_INFO("Input addrlen: %d, Output addrlen: %d\n", *req->addrlen, bswap32(res->addrlen));
                }
              memcpy(req->addr, &res->address, *req->addrlen);
            }
          else
            {
              DBGIF_LOG_ERROR("Unexpected. addrlen is NULL.\n");
            }
        }
      if (req->addrlen)
        {
          *req->addrlen = bswap32(res->addrlen);
        }
      result = ret;
    }

  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);

  return result;

errout_with_cmdfree:
  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);
  farapi_seterrno(err);
  return ACCEPT_REQ_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: farapi_accept
 *
 * Description:
 *   The farapi_accept function is used with connection-based socket types
 *   (FARAPI_SOCK_STREAM). It extracts the first
 *   connection request on the queue of pending connections, creates a new
 *   connected socket with mostly the same properties as 'sockfd', and
 *   allocates a new socket descriptor for the socket, which is returned. The
 *   newly created socket is no longer in the listening state. The original
 *   socket 'sockfd' is unaffected by this call.  Per file descriptor flags
 *   are not inherited across an accept.
 *
 *   The 'sockfd' argument is a socket descriptor that has been created with
 *   farapi_socket(), bound to a local address with farapi_bind(), and is
 *   listening for connections after a call to farapi_listen().
 *
 *   On return, the 'addr' structure is filled in with the address of the
 *   connecting entity. The 'addrlen' argument initially contains the size
 *   of the structure pointed to by 'addr'; on return it will contain the
 *   actual length of the address returned.
 *
 *   If no pending connections are present on the queue, and the socket is
 *   not marked as non-blocking, accept blocks the caller until a connection
 *   is present. If the socket is marked non-blocking and no pending
 *   connections are present on the queue, accept returns EAGAIN.
 *
 * Parameters:
 *   sockfd   The listening socket descriptor
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr', Return: returned size of 'addr'
 *
 * Returned Value:
 *  Returns -1 on error. If it succeeds, it returns a non-negative integer
 *  that is a descriptor for the accepted socket.
 *
 ****************************************************************************/

int farapi_accept(int sockfd, struct farapi_sockaddr *addr,
                  farapi_socklen_t *addrlen)
{
  int32_t                    ret;
  int32_t                    result;
  bool                       is_init;
  FAR struct farapi_socket_s *fsock;
  struct farapi_fd_set_s     readset;
  struct accept_req_s        req;

  APIUTIL_ISINIT(is_init);
  if (!is_init)
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      farapi_seterrno(FARAPI_ENETDOWN);
      return -1;
    }

  if (addr && (!addrlen))
    {
      DBGIF_LOG_ERROR("addrlen is NULL\n");
      farapi_seterrno(FARAPI_EINVAL);
      return -1;
    }

  fsock = farapi_sockfd_socket(sockfd);
  if (!fsock)
    {
      farapi_seterrno(FARAPI_EINVAL);
      return -1;
    }

  req.sockfd  = sockfd;
  req.addr    = addr;
  req.addrlen = addrlen;

  if (fsock->flags & FARAPI_O_NONBLOCK)
    {
      FARAPI_FD_ZERO(&readset);
      FARAPI_FD_SET(sockfd, &readset);

      ret = farapi_select_nonblock((sockfd + 1), &readset, NULL, NULL);
      if (ret > 0)
        {
          if (!FARAPI_FD_ISSET(sockfd, &readset))
            {
              farapi_seterrno(FARAPI_EFAULT);
              DBGIF_LOG1_ERROR("select failed: %d\n", farapi_errno());
              return -1;
            }

          /* Send accept request */

          result = accept_request(fsock, &req);
          if (result == ACCEPT_REQ_FAILURE)
            {
              return -1;
            }
        }
      else
        {
          if (ret == 0)
            {
              farapi_seterrno(FARAPI_EAGAIN);
            }
          else
            {
              DBGIF_LOG1_ERROR("select failed: %d\n", farapi_errno());
            }
          return -1;
        }
    }
  else
    {
      /* Wait until a connection is present */

      FARAPI_FD_ZERO(&readset);
      FARAPI_FD_SET(sockfd, &readset);

      ret = farapi_select_block((sockfd + 1), &readset, NULL, NULL, NULL);
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

      /* Send accept request */

      result = accept_request(fsock, &req);
      if (result == ACCEPT_REQ_FAILURE)
        {
          return -1;
        }
    }

  return result;
}
