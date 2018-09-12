/****************************************************************************
 * modules/lte/farapi/api/socket/farapi_getsockname.c
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
#include "apicmd_getsockname.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "bswap.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GETSOCKNAME_REQ_DATALEN (sizeof(struct apicmd_getsockname_s))
#define GETSOCKNAME_RES_DATALEN (sizeof(struct apicmd_getsocknameres_s))

#define GETSOCKNAME_REQ_SUCCESS 0
#define GETSOCKNAME_REQ_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct getsockname_req_s
{
  int                        sockfd;
  FAR struct farapi_sockaddr *addr;
  FAR farapi_socklen_t       *addrlen;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getsockname_request
 *
 * Description:
 *   Send FARAPI_GETSOCKNAME_REQ.
 *
 ****************************************************************************/

static int32_t getsockname_request(FAR struct farapi_socket_s *fsock,
                                   FAR struct getsockname_req_s *req)
{
  int32_t                            ret;
  int32_t                            err;
  int32_t                            result;
  uint16_t                           reslen = 0;
  FAR struct apicmd_getsockname_s    *cmd = NULL;
  FAR struct apicmd_getsocknameres_s *res = NULL;

  /* Allocate send and response command buffer */

  APIUTIL_SOCK_ALLOC_CMDANDRESBUFF(cmd, APICMDID_SOCK_GETSOCKNAME,
                                   GETSOCKNAME_REQ_DATALEN, res,
                                   GETSOCKNAME_RES_DATALEN);

  /* Fill the data */

  cmd->sockfd  = bswap32(req->sockfd);
  cmd->namelen = bswap32(*req->addrlen);

  DBGIF_LOG2_DEBUG("[getsockname-req]sockfd: %d, addrlen: %d\n", req->sockfd, *req->addrlen);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      GETSOCKNAME_RES_DATALEN,
                      &reslen, SYS_TIMEO_FEVR);
  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      err = -ret;
      goto errout_with_cmdfree;
    }

  if (reslen != GETSOCKNAME_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      err = FARAPI_EFAULT;
      goto errout_with_cmdfree;
    }

  ret = bswap32(res->ret_code);
  err = bswap32(res->err_code);

  DBGIF_LOG2_DEBUG("[getsockname-res]ret: %d, err: %d\n", ret, err);

  if (APICMD_GETSOCKNAME_RES_RET_CODE_ERR == ret)
    {
      DBGIF_LOG1_ERROR("API command response is err :%d.\n", err);
      goto errout_with_cmdfree;
    }
  else
    {
      DBGIF_LOG1_DEBUG("[getsockname-res]addrlen: %d\n", bswap32(res->namelen));

      if (*req->addrlen < bswap32(res->namelen))
        {
          DBGIF_LOG2_INFO("Input addrlen: %d, Output addrlen: %d\n", *req->addrlen, bswap32(res->namelen));
        }
      memcpy(req->addr, &res->name, *req->addrlen);
      *req->addrlen = bswap32(res->namelen);
      result = GETSOCKNAME_REQ_SUCCESS;
    }

  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);

  return result;

errout_with_cmdfree:
  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);
  farapi_seterrno(err);
  return GETSOCKNAME_REQ_FAILURE;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: farapi_getsockname
 *
 * Description:
 *   The farapi_getsockname() function retrieves the locally-bound name of the
 *   specified socket, stores this address in the sockaddr structure pointed
 *   to by the 'addr' argument, and stores the length of this address in the
 *   object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket [in]
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address. Otherwise, -1 is returned and errno is set to indicate the error.
 *
 ****************************************************************************/

int farapi_getsockname(int sockfd, struct farapi_sockaddr *addr,
                       farapi_socklen_t *addrlen)
{
  int32_t                    result;
  bool                       is_init;
  FAR struct farapi_socket_s *fsock;
  struct getsockname_req_s   req;

  APIUTIL_ISINIT(is_init);
  if (!is_init)
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      farapi_seterrno(FARAPI_ENETDOWN);
      return -1;
    }

  if ((!addr) || (!addrlen))
    {
      DBGIF_LOG_ERROR("addr or addrlenis NULL\n");
      farapi_seterrno(FARAPI_EINVAL);
      return -1;
    }

  if (*addrlen <= 0)
    {
      DBGIF_LOG1_ERROR("Invalide addrlen: %d\n", *addrlen);
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

  /* Send getsockname request */

  result = getsockname_request(fsock, &req);
  if (result == GETSOCKNAME_REQ_FAILURE)
    {
      return -1;
    }

  return 0;
}
