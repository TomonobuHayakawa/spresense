/****************************************************************************
 * modules/lte/farapi/api/socket/farapi_getsockopt.c
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
#include "farapi_inet.h"
#include "farapi_seterrno.h"
#include "apicmd_getsockopt.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "bswap.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GETSOCKOPT_REQ_DATALEN (sizeof(struct apicmd_getsockopt_s))
#define GETSOCKOPT_RES_DATALEN (sizeof(struct apicmd_getsockoptres_s))

#define SET_MODE_8BIT          1
#define SET_MODE_32BIT         2
#define SET_MODE_LINGER        3
#define SET_MODE_INADDR        4

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: farapi_getsockopt
 *
 * Description:
 *   farapi_getsockopt() retrieve thse value for the option specified by the
 *   'option' argument for the socket specified by the 'sockfd' argument. If
 *   the size of the option value is greater than 'value_len', the value
 *   stored in the object pointed to by the 'value' argument will be silently
 *   truncated. Otherwise, the length pointed to by the 'value_len' argument
 *   will be modified to indicate the actual length of the'value'.
 *
 *   The 'level' argument specifies the protocol level of the option. To
 *   retrieve options at the socket level, specify the level argument as
 *   FARAPI_SOL_SOCKET.
 *
 * Parameters:
 *   sockfd    Socket descriptor of socket
 *   level     Protocol level to set the option
 *   option    identifies the option to get
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 ****************************************************************************/

int farapi_getsockopt(int sockfd, int level, int option, void *value,
                      farapi_socklen_t *value_len)
{
  int32_t                           ret;
  int32_t                           err;
  int32_t                           optlen;
  int32_t                           set_mode = 0;
  FAR int32_t                       *pint32_val;
  bool                              is_init;
  uint16_t                          reslen = 0;
  FAR struct apicmd_getsockopt_s    *cmd = NULL;
  FAR struct apicmd_getsockoptres_s *res = NULL;
  FAR struct farapi_socket_s        *fsock;
  FAR struct farapi_linger          *plinger;
  FAR struct farapi_in_addr         *pinaddr;

  APIUTIL_ISINIT(is_init);
  if (!is_init)
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      farapi_seterrno(FARAPI_ENETDOWN);
      return -1;
    }

  if ((!value) || (!value_len))
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
      /* Level: FARAPI_SOL_SOCKET*/

      case FARAPI_SOL_SOCKET:
        switch(option)
          {
            case FARAPI_SO_ACCEPTCONN:
            case FARAPI_SO_ERROR:
            case FARAPI_SO_BROADCAST:
            case FARAPI_SO_KEEPALIVE:
            case FARAPI_SO_REUSEADDR:
            case FARAPI_SO_TYPE:
            case FARAPI_SO_RCVBUF:
            case FARAPI_SO_NO_CHECK:
              FARAPI_CHECK_VALUELEN(*(FAR int32_t*)value_len, int32_t);
              set_mode = SET_MODE_32BIT;
              break;


            case FARAPI_SO_SNDTIMEO:
              FARAPI_CHECK_VALUELEN(*(FAR int32_t*)value_len,
                                    struct farapi_timeval);
              memcpy(value, &fsock->sendtimeo,
                     sizeof(struct farapi_timeval));
              *value_len = sizeof(struct farapi_timeval);
              return 0;
              break;

            case FARAPI_SO_RCVTIMEO:
              FARAPI_CHECK_VALUELEN(*(FAR int32_t*)value_len,
                                    struct farapi_timeval);
              memcpy(value, &fsock->recvtimeo,
                     sizeof(struct farapi_timeval));
              *value_len = sizeof(struct farapi_timeval);
              return 0;
              break;

            case FARAPI_SO_LINGER:
              FARAPI_CHECK_VALUELEN(*(FAR int32_t*)value_len,
                                    struct farapi_linger);
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
              FARAPI_CHECK_VALUELEN(*(FAR int32_t*)value_len, int32_t);
              set_mode = SET_MODE_32BIT;
              break;

            case FARAPI_IP_MULTICAST_TTL:
            case FARAPI_IP_MULTICAST_LOOP:
              FARAPI_CHECK_VALUELEN(*(FAR int32_t*)value_len, uint8_t);
              set_mode = SET_MODE_8BIT;
              break;

            case FARAPI_IP_MULTICAST_IF:
              FARAPI_CHECK_VALUELEN(*(FAR int32_t*)value_len,
                                    struct farapi_in_addr);
              set_mode = SET_MODE_INADDR;
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
              FARAPI_CHECK_VALUELEN(*(FAR int32_t*)value_len, int32_t);
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
              FARAPI_CHECK_VALUELEN(*(FAR int32_t*)value_len, int32_t);
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
              FARAPI_CHECK_VALUELEN(*(FAR int32_t*)value_len, int32_t);
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
              FARAPI_CHECK_VALUELEN(*(FAR int32_t*)value_len, int32_t);
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

  /* Allocate send and response command buffer */

  APIUTIL_SOCK_ALLOC_CMDANDRESBUFF(cmd, APICMDID_SOCK_GETSOCKOPT,
                                   GETSOCKOPT_REQ_DATALEN, res,
                                   GETSOCKOPT_RES_DATALEN);

  /* Fill the data */

  cmd->sockfd  = bswap32(sockfd);
  cmd->level   = bswap32(level);
  cmd->optname = bswap32(option);
  cmd->optlen  = bswap32(*value_len);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      GETSOCKOPT_RES_DATALEN,
                      &reslen, SYS_TIMEO_FEVR);
  if (0 <= ret)
    {
      ret    = bswap32(res->ret_code);
      err    = bswap32(res->err_code);
      optlen = bswap32(res->optlen);

      if (APICMD_GETSOCKOPT_RES_RET_CODE_ERR == ret)
        {
          DBGIF_LOG_ERROR("API command response is err.\n");
          farapi_seterrno(err);
        }
      else
        {
          switch(set_mode)
            {
              case SET_MODE_8BIT:
                if (optlen == sizeof(uint8_t))
                  {
                    *(FAR uint8_t*)value = res->optval[0];
                    ret = 0;
                  }
                else
                  {
                    DBGIF_LOG1_ERROR("Unexpected option len: %d.\n", optlen);
                    farapi_seterrno(FARAPI_EFAULT);
                    ret = -1;
                  }
                break;

              case SET_MODE_32BIT:
                if (optlen == sizeof(int32_t))
                  {
                    pint32_val = (FAR int32_t*)&res->optval[0];
                    *(FAR int32_t*)value = bswap32(*pint32_val);
                    ret = 0;
                  }
                else
                  {
                    DBGIF_LOG1_ERROR("Unexpected option len: %d.\n", optlen);
                    farapi_seterrno(FARAPI_EFAULT);
                    ret = -1;
                  }
                break;

              case SET_MODE_LINGER:
                if (optlen == sizeof(struct farapi_linger))
                  {
                    plinger = (FAR struct farapi_linger*)&res->optval[0];
                    ((FAR struct farapi_linger*)value)->l_onoff =
                      bswap32(plinger->l_onoff);
                    ((FAR struct farapi_linger*)value)->l_linger =
                      bswap32(plinger->l_linger);
                    ret = 0;
                  }
                else
                  {
                    DBGIF_LOG1_ERROR("Unexpected option len: %d.\n", optlen);
                    farapi_seterrno(FARAPI_EFAULT);
                    ret = -1;
                  }
                break;

              case SET_MODE_INADDR:
                if (optlen == sizeof(struct farapi_in_addr))
                  {
                    pinaddr = (FAR struct farapi_in_addr*)&res->optval[0];
                    ((FAR struct farapi_in_addr*)value)->s_addr =
                      bswap32(pinaddr->s_addr);
                    ret = 0;
                  }
                else
                  {
                    DBGIF_LOG1_ERROR("Unexpected option len: %d.\n", optlen);
                    farapi_seterrno(FARAPI_EFAULT);
                    ret = -1;
                  }
                break;

              default:
                DBGIF_LOG1_ERROR("Unexpected set mode: %d.\n", set_mode);
                farapi_seterrno(FARAPI_EFAULT);
                ret = -1;
                break;
            }
        }
    }
  else
    {
      farapi_seterrno(-ret);
      ret = -1;
    }

  APIUTIL_SOCK_FREE_CMDANDRESBUFF(cmd, res);

  return ret;
}
