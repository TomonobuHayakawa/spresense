/****************************************************************************
 * modules/lte/farapi/api/socket/farapi_gethostbyname_r.c
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
#include "farapi_netdb.h"
#include "bswap.h"
#include "apiutil.h"
#include "farapi_inet.h"
#include "farapi_netdb.h"
#include "apicmd_gethostbynamer.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GETHOSTBYNAMER_REQ_DATALEN (sizeof(struct apicmd_gethostbynamer_s))
#define GETHOSTBYNAMER_RES_DATALEN (sizeof(struct apicmd_gethostbynamer_res_s))

#define GETHOSTBYNAMER_REQ_SUCCESS 0
#define GETHOSTBYNAMER_REQ_FAILURE -1

/****************************************************************************
 * Private Type
 ****************************************************************************/

struct gethostbyname_r_helper {
  FAR struct farapi_in6_addr *addr_list[2];
  struct farapi_in6_addr addr;
  FAR char *aliases;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gethostbynamer_request
 *
 * Description:
 *   Send APICMDID_SOCK_GETHOSTBYNAME_R_REQ.
 *
 ****************************************************************************/

static int32_t gethostbynamer_request(FAR const char *name, int32_t namelen,
                                      FAR struct farapi_hostent *ret,
                                      FAR char *buf, size_t buflen,
                                      FAR struct farapi_hostent **result,
                                      FAR int *h_errnop)
{
  int32_t                                cmdret;
  int32_t                                err;
  uint16_t                               reslen = 0;
  int32_t                                alslen = 0;
  FAR char                               *hostname;
  FAR struct apicmd_gethostbynamer_s     *cmd   = NULL;
  FAR struct apicmd_gethostbynamer_res_s *res   = NULL;
  FAR struct gethostbyname_r_helper      *h     = NULL;

  APIUTIL_SOCK_ALLOC_CMDANDRESBUFF(cmd, APICMDID_SOCK_GETHOSTBYNAMER,
                                   GETHOSTBYNAMER_REQ_DATALEN, res,
                                   GETHOSTBYNAMER_RES_DATALEN);
  cmd->namelen = bswap32(namelen);
  memcpy(cmd->name, name, namelen);
  cmd->buflen  = bswap32(buflen);
  cmdret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                         GETHOSTBYNAMER_RES_DATALEN,
                         &reslen, SYS_TIMEO_FEVR);
  if (0 > cmdret)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", cmdret);
      err = -cmdret;
      goto errout_with_cmdfree;
    }

  if (GETHOSTBYNAMER_RES_DATALEN != reslen)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      err = FARAPI_EFAULT;
      goto errout_with_cmdfree;
    }

  cmdret = bswap32(res->ret_code);
  err    = bswap32(res->err_code);

  if (APICMD_GETHOSTBYNAMER_RES_RET_CODE_OK != cmdret)
    {
      DBGIF_LOG1_ERROR("API command response is err :%d.\n", err);
      goto errout_with_cmdfree;
    }

  /* Fill command result */

  h = (struct gethostbyname_r_helper *)buf;
  if (res->h_name)
    {
      hostname = ((char*)h) + sizeof(struct gethostbyname_r_helper);
      strncpy(hostname, (FAR const char *)res->h_name, namelen+1);
      ret->h_name = hostname;
    }

  if (res->h_aliases)
    {
      alslen = strlen((FAR char *)res->h_aliases);
      ret->h_aliases[0] = (FAR char *)BUFFPOOL_ALLOC(alslen);
      strncpy(ret->h_aliases[0], (FAR const char *)res->h_aliases, alslen);
    }

  ret->h_addrtype = bswap32(res->h_addrtype);
  ret->h_length = FARAPI_AF_INET == ret->h_addrtype ?
    sizeof(struct farapi_in_addr) : sizeof(struct farapi_in6_addr);
  if (res->h_addr_list)
    {
      memcpy(&h->addr, res->h_addr_list, ret->h_length);
      h->addr_list[0] = &h->addr;
      h->addr_list[1] = NULL;
      ret->h_addr_list = (FAR char**)&h->addr_list;
    }

  *result = ret;
  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);
  return GETHOSTBYNAMER_REQ_SUCCESS;

errout_with_cmdfree:
  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);
  if (h_errnop)
    {
      *h_errnop = err;
    }
  else
    {
      h_errnop = &err;
    }

  return GETHOSTBYNAMER_REQ_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: farapi_gethostbyname_r
 *
 * Description:
 *   The farapi_gethostbyname_r() function returns a structure of type
 *   hostent for the given host name. Here name is either a hostname, or an
 *   IPv4 address in standard dot notation (as for inet_addr(3)), or an IPv6
 *   address in colon (and possibly dot) notation.
 *
 *   If name is an IPv4 or IPv6 address, no lookup is performed and
 *   farapi_gethostbyname_r() simply copies name into the h_name field
 *   and its struct in_addr equivalent into the h_addr_list[0] field of the
 *   returned hostent structure.
 *
 * Input Parameters:
 *   name - The name of the host to find.
 *   ret - Caller provided location to return the host data.
 *   buf - Caller provided buffer to hold string data associated with the
 *     host data.
 *   buflen - The size of the caller-provided buffer
 *   result - Point to the result on success.
 *   h_errnop - There h_errno value returned in the event of a failure.
 *
 * Returned Value:
 *   0 is returned on success, -1 is returned on a failure
 *   with the returned h_errno value provided the reason for the failure.
 *
 ****************************************************************************/

int farapi_gethostbyname_r(const char *name, struct farapi_hostent *ret,
                           char *buf, size_t buflen,
                           struct farapi_hostent **result, int *h_errnop)
{
  bool    is_init;
  int32_t namelen = 0;

  APIUTIL_ISINIT(is_init);
  if (!is_init)
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      return -1;
    }

  if (!name || !ret || !buf || !buflen || !result)
    {
      DBGIF_LOG_ERROR("Invalid paramaeter\n");
      if (h_errnop)
        {
          *h_errnop = FARAPI_EINVAL;
        }

      return -1;
    }

  namelen = strlen(name);
  if (GETHOSTBYNAMER_REQ_SUCCESS !=
    gethostbynamer_request(name, namelen,
      ret, buf, buflen, result, h_errnop))
    {
      return -1;
    }

  return 0;
}
