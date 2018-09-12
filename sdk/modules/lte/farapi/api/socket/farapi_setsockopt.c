/****************************************************************************
 * modules/lte/farapi/api/socket/farapi_setsockopt.c
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
#include "farapi_sock.h"
#include "farapi_socket.h"
#include "farapi_in.h"
#include "farapi_errno.h"
#include "farapi_seterrno.h"
#include "apicmd_setsockopt.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "bswap.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SETSOCKOPT_REQ_DATALEN (sizeof(struct apicmd_setsockopt_s))
#define SETSOCKOPT_RES_DATALEN (sizeof(struct apicmd_setsockoptres_s))

#define SET_MODE_8BIT          1
#define SET_MODE_32BIT         2
#define SET_MODE_LINGER        3
#define SET_MODE_INADDR        4
#define SET_MODE_IPMREQ        5

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: farapi_setsockopt
 *
 * Description:
 *   farapi_setsockopt() sets the option specified by the 'option' argument,
 *   at the protocol level specified by the 'level' argument, to the value
 *   pointed to by the 'value' argument for the socket associated with the
 *   file descriptor specified by the 'sockfd' argument.
 *
 *   The 'level' argument specifies the protocol level of the option. To set
 *   options at the socket level, specify the level argument
 *   as FARAPI_SOL_SOCKET.
 *
 * Parameters:
 *   sockfd    Socket descriptor of socket
 *   level     Protocol level to set the option
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *  0 on success; -1 on failure
 *
 ****************************************************************************/

int farapi_setsockopt(int sockfd, int level, int option, const void *value,
                      farapi_socklen_t value_len)
{
  int32_t                           ret;
  int32_t                           err;
  bool                              is_init;
  FAR struct farapi_socket_s        *fsock;
  int32_t                           set_mode = 0;
  uint8_t                           optval[APICMD_SETSOCKOPT_OPTVAL_LENGTH];
  uint16_t                          reslen = 0;
  FAR struct apicmd_setsockopt_s    *cmd = NULL;
  FAR struct apicmd_setsockoptres_s *res = NULL;

  APIUTIL_ISINIT(is_init);
  if (!is_init)
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      farapi_seterrno(FARAPI_ENETDOWN);
      return -1;
    }

  if (!value)
    {
      DBGIF_LOG_ERROR("Invalid parameter\n");
      farapi_seterrno(FARAPI_EINVAL);
      return -1;
    }

  fsock = farapi_sockfd_socket(sockfd);
  if (!fsock)
    {
      farapi_seterrno(FARAPI_EINVAL);
      return -1;
    }

  switch(level)
    {
      /* Level: FARAPI_SOL_SOCKET */

      case FARAPI_SOL_SOCKET:
        switch(option)
          {
            case FARAPI_SO_BROADCAST:
            case FARAPI_SO_KEEPALIVE:
            case FARAPI_SO_REUSEADDR:
            case FARAPI_SO_RCVBUF:
            case FARAPI_SO_NO_CHECK:
              FARAPI_CHECK_VALUELEN(value_len, int32_t);
              set_mode = SET_MODE_32BIT;
              break;

            case FARAPI_SO_SNDTIMEO:
              FARAPI_CHECK_VALUELEN(value_len, struct farapi_timeval);
              memcpy(&fsock->sendtimeo, value,
                     sizeof(struct farapi_timeval));
              return 0;
              break;

            case FARAPI_SO_RCVTIMEO:
              FARAPI_CHECK_VALUELEN(value_len, struct farapi_timeval);
              memcpy(&fsock->recvtimeo, value,
                     sizeof(struct farapi_timeval));
              return 0;
              break;

            case FARAPI_SO_LINGER:
              FARAPI_CHECK_VALUELEN(value_len, struct farapi_linger);
              set_mode = SET_MODE_LINGER;
              break;

            default:
              DBGIF_LOG1_ERROR("Not support option: %d\n", option);
              farapi_seterrno(FARAPI_EINVAL);
              return -1;
              break;
          }
        break;

      /* Level: FARAPI_IPPROTO_IP */

      case FARAPI_IPPROTO_IP:
        switch(option)
          {
            case FARAPI_IP_TTL:
            case FARAPI_IP_TOS:
              FARAPI_CHECK_VALUELEN(value_len, int32_t);
              set_mode = SET_MODE_32BIT;
              break;

            case FARAPI_IP_MULTICAST_TTL:
            case FARAPI_IP_MULTICAST_LOOP:
              FARAPI_CHECK_VALUELEN(value_len, uint8_t);
              set_mode = SET_MODE_8BIT;
              break;

            case FARAPI_IP_MULTICAST_IF:
              FARAPI_CHECK_VALUELEN(value_len, struct farapi_in_addr);
              set_mode = SET_MODE_INADDR;
              break;

            case FARAPI_IP_ADD_MEMBERSHIP:
            case FARAPI_IP_DROP_MEMBERSHIP:
              FARAPI_CHECK_VALUELEN(value_len, struct farapi_ip_mreq);
              set_mode = SET_MODE_IPMREQ;
              break;

            default:
              DBGIF_LOG1_ERROR("Not support option: %d\n", option);
              farapi_seterrno(FARAPI_EINVAL);
              return -1;
              break;
          }
        break;

      /* Level: FARAPI_IPPROTO_TCP */

      case FARAPI_IPPROTO_TCP:
        switch(option)
          {
            case FARAPI_TCP_NODELAY:
            case FARAPI_TCP_KEEPALIVE:
            case FARAPI_TCP_KEEPIDLE:
            case FARAPI_TCP_KEEPINTVL:
            case FARAPI_TCP_KEEPCNT:
              FARAPI_CHECK_VALUELEN(value_len, int32_t);
              set_mode = SET_MODE_32BIT;
              break;

            default:
              DBGIF_LOG1_ERROR("Not support option: %d\n", option);
              farapi_seterrno(FARAPI_EINVAL);
              return -1;
              break;
          }
        break;

      /* Level: FARAPI_IPPROTO_IPV6 */

      case FARAPI_IPPROTO_IPV6:
        switch(option)
          {
            case FARAPI_IPV6_V6ONLY:
              FARAPI_CHECK_VALUELEN(value_len, int32_t);
              set_mode = SET_MODE_32BIT;
              break;

            default:
              DBGIF_LOG1_ERROR("Not support option: %d\n", option);
              farapi_seterrno(FARAPI_EINVAL);
              return -1;
              break;
          }
        break;

      /* Level: FARAPI_IPPROTO_UDPLITE */

      case FARAPI_IPPROTO_UDPLITE:
        switch(option)
          {
            case FARAPI_UDPLITE_SEND_CSCOV:
            case FARAPI_UDPLITE_RECV_CSCOV:
              FARAPI_CHECK_VALUELEN(value_len, int32_t);
              set_mode = SET_MODE_32BIT;
              break;

            default:
              DBGIF_LOG1_ERROR("Not support option: %d\n", option);
              farapi_seterrno(FARAPI_EINVAL);
              return -1;
              break;
          }
        break;

      /* Level: FARAPI_IPPROTO_RAW */

      case FARAPI_IPPROTO_RAW:
        switch(option)
          {
            case FARAPI_IPV6_CHECKSUM:
              FARAPI_CHECK_VALUELEN(value_len, int32_t);
              set_mode = SET_MODE_32BIT;
              break;

            default:
              DBGIF_LOG1_ERROR("Not support option: %d\n", option);
              farapi_seterrno(FARAPI_EINVAL);
              return -1;
              break;
          }
        break;

      default:
        DBGIF_LOG1_ERROR("Not support level: %d\n", level);
        farapi_seterrno(FARAPI_EINVAL);
        return -1;
        break;
    }

  memset(optval, 0, APICMD_SETSOCKOPT_OPTVAL_LENGTH);

  switch(set_mode)
    {
      case SET_MODE_8BIT:
        optval[0] = *(FAR uint8_t*)value;
        break;

      case SET_MODE_32BIT:
        *((FAR int32_t*)optval) = bswap32(*(FAR int32_t*)value);
        break;

      case SET_MODE_LINGER:
        ((FAR struct farapi_linger*)optval)->l_onoff =
          bswap32(((FAR struct farapi_linger*)value)->l_onoff);
        ((FAR struct farapi_linger*)optval)->l_linger =
          bswap32(((FAR struct farapi_linger*)value)->l_linger);
        break;

      case SET_MODE_INADDR:
        ((FAR struct farapi_in_addr*)optval)->s_addr =
          bswap32(((FAR struct farapi_in_addr*)value)->s_addr);
        break;

      case SET_MODE_IPMREQ:
        ((FAR struct farapi_ip_mreq*)optval)->imr_multiaddr.s_addr =
          bswap32(((FAR struct farapi_ip_mreq*)value)->imr_multiaddr.s_addr);
        ((FAR struct farapi_ip_mreq*)optval)->imr_interface.s_addr =
          bswap32(((FAR struct farapi_ip_mreq*)value)->imr_interface.s_addr);
        break;

      default:
        DBGIF_LOG1_ERROR("Unsupported mode: %d.\n", set_mode);
        farapi_seterrno(FARAPI_EFAULT);
        return -1;
        break;
    }

  /* Allocate send and response command buffer */

  APIUTIL_SOCK_ALLOC_CMDANDRESBUFF(cmd, APICMDID_SOCK_SETSOCKOPT,
                                   SETSOCKOPT_REQ_DATALEN, res,
                                   SETSOCKOPT_RES_DATALEN);

  /* Fill the data */

  cmd->sockfd  = bswap32(sockfd);
  cmd->level   = bswap32(level);
  cmd->optname = bswap32(option);
  cmd->optlen  = bswap32(value_len);
  memcpy(cmd->optval, optval, value_len);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      SETSOCKOPT_RES_DATALEN,
                      &reslen, SYS_TIMEO_FEVR);
  if (0 <= ret)
    {
      ret = bswap32(res->ret_code);
      err = bswap32(res->err_code);

      if (APICMD_SETSOCKOPT_RES_RET_CODE_ERR == ret)
        {
          DBGIF_LOG_ERROR("API command response is err.\n");
          farapi_seterrno(err);
        }
    }
  else
    {
      farapi_seterrno(-ret);
      ret = -1;
    }

  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);

  return 0;
}
