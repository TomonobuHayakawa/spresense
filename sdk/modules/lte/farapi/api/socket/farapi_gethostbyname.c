/****************************************************************************
 * modules/lte/farapi/api/socket/farapi_gethostbyname.c
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
#include "buffpoolwrapper.h"
#include "bswap.h"
#include "apiutil.h"
#include "farapi_netdb.h"
#include "farapi_inet.h"
#include "farapi_netdb.h"
#include "farapi_seterrno.h"
#include "apicmd_gethostbyname.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GETHOSTBYNAME_REQ_DATALEN (sizeof(struct apicmd_gethostbyname_s))
#define GETHOSTBYNAME_RES_DATALEN (sizeof(struct apicmd_gethostbynameres_s))

#define NAME_LENGTH (APICMD_GETHOSTBYNAME_NAME_MAX_LENGTH + 1)

#define GETHOSTBYNAME_REQ_SUCCESS 0
#define GETHOSTBYNAME_REQ_FAILURE -1

/****************************************************************************
 * Public Data
 ****************************************************************************/

int farapi_h_errno;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gethostbyname_request
 *
 * Description:
 *   Send APICMDID_SOCK_GETHOSTBYNAME_REQ.
 *
 ****************************************************************************/

static int32_t gethostbyname_request(FAR const char *name, int32_t namelen,
                                     FAR struct farapi_hostent *result)
{
  int32_t                              ret;
  int32_t                              err    = 0;
  uint16_t                             reslen = 0;
  FAR struct apicmd_gethostbyname_s    *cmd   = NULL;
  FAR struct apicmd_gethostbynameres_s *res   = NULL;
  static struct farapi_in6_addr        s_hostent_addr;
  static char                          *s_phostent_addr[2];
  static char                          s_hostname[NAME_LENGTH];
  int32_t                              l_namelen;
  int32_t                              aliaslen;

  APIUTIL_SOCK_ALLOC_CMDANDRESBUFF(cmd, APICMDID_SOCK_GETHOSTBYNAME,
                                   GETHOSTBYNAME_REQ_DATALEN, res,
                                   GETHOSTBYNAME_RES_DATALEN);
  cmd->namelen = bswap32(namelen);
  memcpy(cmd->name, name, namelen);
  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                     GETHOSTBYNAME_RES_DATALEN, &reslen,
                     SYS_TIMEO_FEVR);
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      err = -ret;
      DBGIF_LOG1_ERROR("apicmdgw_send() :%d.\n", err);
      goto errout_with_cmdfree;
    }

  if (GETHOSTBYNAME_RES_DATALEN != reslen)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      ret = FARAPI_EFAULT;
      goto errout_with_cmdfree;
    }

  ret = bswap32(res->ret_code);
  if (APICMD_GETHOSTBYNAME_RES_RET_CODE_OK != ret)
    {
      farapi_h_errno = ret;
      err = GETHOSTBYNAME_REQ_FAILURE;
      DBGIF_LOG1_ERROR("API command response is err :%d.\n", err);
      goto errout_with_cmdfree;
    }

  /* Fill command result */

  result->h_addrtype = bswap32(res->h_addrtype);
  l_namelen = strlen((FAR char *)res->h_name);
  if (0 < l_namelen)
    {
      memset(s_hostname, 0, sizeof(s_hostname));
      memcpy(s_hostname, res->h_name, l_namelen);
      result->h_name = s_hostname;
    }

  result->h_length = FARAPI_AF_INET == result->h_addrtype ?
    sizeof(struct farapi_in_addr) : sizeof(struct farapi_in6_addr);
  memset(&s_hostent_addr, 0, sizeof(s_hostent_addr));

  memcpy(&s_hostent_addr, res->h_addr_list, result->h_length);

  s_phostent_addr[0] = (FAR char *)&s_hostent_addr;

  s_phostent_addr[1] = NULL;
  result->h_addr_list = (FAR char**)&s_phostent_addr;

  aliaslen = strnlen((FAR char *)res->h_aliases,
                     APICMD_GETHOSTBYNAME_RES_H_ALIASES_LENGTH);
  DBGIF_LOG1_ERROR("h_aliases len =%d\n", aliaslen);

  if (aliaslen > 0)
    {
      memcpy(result->h_aliases,
             res->h_aliases, aliaslen);
    }
  else
    {
      result->h_aliases = NULL;
    }

  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);

  return GETHOSTBYNAME_REQ_SUCCESS;

errout_with_cmdfree:
  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);
  return err;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: farapi_gethostbyname
 *
 * Description:
 *   The farapi_gethostbyname() function returns a structure of type hostent
 *   for the given host name. Here name is either a hostname, or an IPv4
 *   address in standard dot notation (as for inet_addr(3)), or an IPv6
 *   address in colon (and possibly dot) notation.
 *
 *   If name is an IPv4 or IPv6 address, no lookup is performed and
 *   farapi_gethostbyname_r() simply copies name into the h_name field
 *   and its struct in_addr equivalent into the h_addr_list[0] field of the
 *   returned hostent structure.
 *
 * Input Parameters:
 *   name - The name of the host to find.
 *
 * Returned Value:
 *   Upon successful completion, this function will return a pointer to a
 *   hostent structure if the requested entry was found, and a null pointer
 *   if the end of the database was reached or the requested entry was not
 *   found.
 *
 *   Upon unsuccessful completion, farapi_gethostbyname() will set h_errno to
 *   indicate the error
 *
 ****************************************************************************/

struct farapi_hostent *farapi_gethostbyname(const char *name)
{
  bool                         is_init;
  int32_t                      ret;
  uint32_t                     namelen = 0;
  static struct farapi_hostent s_hostent;

  APIUTIL_ISINIT(is_init);
  if (!is_init)
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      return NULL;
    }

  if (!name)
    {
      DBGIF_LOG_ERROR("Invalid param\n");
      return NULL;
    }

  namelen = strlen(name);
  if (!namelen || APICMD_GETHOSTBYNAME_NAME_MAX_LENGTH < namelen)
    {
      DBGIF_LOG1_ERROR("Invalid param. namelen = [%d]\n", namelen);
      return NULL;
    }

  ret = gethostbyname_request(name, namelen, &s_hostent);
  if (GETHOSTBYNAME_REQ_SUCCESS != ret)
    {
      return NULL;
    }

  return &s_hostent;
}
