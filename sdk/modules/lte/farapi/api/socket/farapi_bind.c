/****************************************************************************
 * modules/lte/farapi/api/socket/farapi_bind.c
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
#include "apicmd_bind.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "bswap.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BIND_REQ_DATALEN (sizeof(struct apicmd_bind_s))
#define BIND_RES_DATALEN (sizeof(struct apicmd_bindres_s))

#define BIND_REQ_SUCCESS 0
#define BIND_REQ_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bind_req_s
{
  int                              sockfd;
  FAR const struct farapi_sockaddr *addr;
  farapi_socklen_t                 addrlen;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bind_request
 *
 * Description:
 *   Send FARAPI_BIND_REQ.
 *
 ****************************************************************************/

static int32_t bind_request(FAR struct farapi_socket_s *fsock,
                            FAR struct bind_req_s *req)
{
  int32_t                     ret;
  int32_t                     err;
  int32_t                     result;
  uint16_t                    reslen = 0;
  FAR struct apicmd_bind_s    *cmd = NULL;
  FAR struct apicmd_bindres_s *res = NULL;

  /* Allocate send and response command buffer */

  APIUTIL_SOCK_ALLOC_CMDANDRESBUFF(cmd, APICMDID_SOCK_BIND,
                                   BIND_REQ_DATALEN, res,
                                   BIND_RES_DATALEN);

  /* Fill the data */

  farapi_sockaddr_to_sockstorage(req->addr, &cmd->name);

  cmd->sockfd  = bswap32(req->sockfd);
  cmd->namelen = bswap32(req->addrlen);

  DBGIF_LOG2_DEBUG("[bind-req]sockfd: %d, namelen: %d\n", req->sockfd, req->addrlen);
  DBGIF_LOG3_DEBUG("[bind-req]s2_len: %d, ss_family: %d, port: %d\n", cmd->name.s2_len, cmd->name.ss_family, *(FAR int16_t*)(cmd->name.s2_data1));

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      BIND_RES_DATALEN,
                      &reslen, SYS_TIMEO_FEVR);
  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      err = -ret;
      goto errout_with_cmdfree;
    }

  if (reslen != BIND_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      err = FARAPI_EFAULT;
      goto errout_with_cmdfree;
    }

  ret = bswap32(res->ret_code);
  err = bswap32(res->err_code);

  DBGIF_LOG2_DEBUG("[bind-res]ret: %d, err: %d\n", ret, err);

  if (APICMD_BIND_RES_RET_CODE_ERR == ret)
    {
      DBGIF_LOG1_ERROR("API command response is err :%d.\n", err);
      goto errout_with_cmdfree;
    }
  else
    {
      result = BIND_REQ_SUCCESS;
    }

  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);

  return result;

errout_with_cmdfree:
  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);
  farapi_seterrno(err);
  return BIND_REQ_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: farapi_bind
 *
 * Description:
 *   farapi_bind() gives the socket 'sockfd' the local address 'addr'.
 *   'addr' is 'addrlen' bytes long. Traditionally, this is called 
 *   "assigning a name to a socket." When a socket is created with socket, 
 *   it exists in a name space (address family) but has no name assigned.
 *
 * Parameters:
 *   sockfd   Socket descriptor of the socket to bind
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 ****************************************************************************/

int farapi_bind(int sockfd, const struct farapi_sockaddr *addr,
                farapi_socklen_t addrlen)
{
  int32_t                    result;
  bool                       is_init;
  FAR struct farapi_socket_s *fsock;
  struct bind_req_s          req;

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

  req.sockfd  = sockfd;
  req.addr    = addr;
  req.addrlen = addrlen;

  /* Send bind request */

  result = bind_request(fsock, &req);

  if (result == BIND_REQ_FAILURE)
    {
      return -1;
    }

  return 0;
}
