/****************************************************************************
 * modules/lte/farapi/include/api/socket/apicmd_getsockopt.h
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

#ifndef __MODULES_LTE_FARAPI_INCLUDE_API_SOCKET_APICMD_GETSOCKOPT_H
#define __MODULES_LTE_FARAPI_INCLUDE_API_SOCKET_APICMD_GETSOCKOPT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "farapi_socket.h"
#include "apicmd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_GETSOCKOPT_RES_OPTVAL_LENGTH (16)

#define APICMD_GETSOCKOPT_RES_LEVEL_SOL_SOCKET (FARAPI_SOL_SOCKET)

#define APICMD_GETSOCKOPT_RES_OPTNAME_SO_ACCEPTCONN (FARAPI_SO_ACCEPTCONN)
#define APICMD_GETSOCKOPT_RES_OPTNAME_SO_BROADCAST  (FARAPI_SO_BROADCAST)
#define APICMD_GETSOCKOPT_RES_OPTNAME_SO_KEEPALIVE  (FARAPI_SO_KEEPALIVE)
#define APICMD_GETSOCKOPT_RES_OPTNAME_SO_REUSEADDR  (FARAPI_SO_REUSEADDR)
#define APICMD_GETSOCKOPT_RES_OPTNAME_SO_TYPE       (FARAPI_SO_TYPE)
#define APICMD_GETSOCKOPT_RES_OPTNAME_SO_ERROR      (FARAPI_SO_ERROR)
#define APICMD_GETSOCKOPT_RES_OPTNAME_SO_SNDTIMEO   (FARAPI_SO_SNDTIMEO)
#define APICMD_GETSOCKOPT_RES_OPTNAME_SO_RCVTIMEO   (FARAPI_SO_RCVTIMEO)
#define APICMD_GETSOCKOPT_RES_OPTNAME_SO_RCVBUF     (FARAPI_SO_RCVBUF)
#define APICMD_GETSOCKOPT_RES_OPTNAME_SO_LINGER     (FARAPI_SO_LINGER)
#define APICMD_GETSOCKOPT_RES_OPTNAME_SO_NO_CHECK   (FARAPI_SO_NO_CHECK)
#define APICMD_GETSOCKOPT_RES_OPTNAME_IP_TTL        (FARAPI_IP_TTL)
#define APICMD_GETSOCKOPT_RES_OPTNAME_IP_TOS        (FARAPI_IP_TOS)
#define APICMD_GETSOCKOPT_RES_OPTNAME_IP_MULTICAST_TTL \
  (FARAPI_IP_MULTICAST_TTL)
#define APICMD_GETSOCKOPT_RES_OPTNAME_IP_MULTICAST_IF \
  (FARAPI_IP_MULTICAST_IF)
#define APICMD_GETSOCKOPT_RES_OPTNAME_IP_MULTICAST_LOOP \
  (FARAPI_IP_MULTICAST_LOOP)
#define APICMD_GETSOCKOPT_RES_OPTNAME_IP_ADD_MEMBERSHIP \
  (FARAPI_IP_ADD_MEMBERSHIP)
#define APICMD_GETSOCKOPT_RES_OPTNAME_IP_DROP_MEMBERSHIP \
  (FARAPI_IP_DROP_MEMBERSHIP)
#define APICMD_GETSOCKOPT_RES_OPTNAME_TCP_NODELAY   (FARAPI_TCP_NODELAY)
#define APICMD_GETSOCKOPT_RES_OPTNAME_TCP_KEEPALIVE (FARAPI_TCP_KEEPALIVE)
#define APICMD_GETSOCKOPT_RES_OPTNAME_TCP_KEEPIDLE  (FARAPI_TCP_KEEPIDLE)
#define APICMD_GETSOCKOPT_RES_OPTNAME_TCP_KEEPINTVL (FARAPI_TCP_KEEPINTVL)
#define APICMD_GETSOCKOPT_RES_OPTNAME_TCP_KEEPCNT   (FARAPI_TCP_KEEPCNT)
#define APICMD_GETSOCKOPT_RES_OPTNAME_IPV6_CHECKSUM (FARAPI_IPV6_CHECKSUM)
#define APICMD_GETSOCKOPT_RES_OPTNAME_IPV6_V6ONLY   (FARAPI_IPV6_V6ONLY)
#define APICMD_GETSOCKOPT_RES_OPTNAME_UDPLITE_SEND_CSCOV \
  (FARAPI_UDPLITE_SEND_CSCOV)
#define APICMD_GETSOCKOPT_RES_OPTNAME_UDPLITE_RECV_CSCOV \
  (FARAPI_UDPLITE_RECV_CSCOV)

#define APICMD_GETSOCKOPT_RES_RET_CODE_OK  (0)
#define APICMD_GETSOCKOPT_RES_RET_CODE_ERR (-1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

PACK_STRUCT_BEGIN
struct apicmd_getsockopt_s
{
  PACK_STRUCT_FIELD(int32_t sockfd);
  PACK_STRUCT_FIELD(int32_t level);
  PACK_STRUCT_FIELD(int32_t optname);
  PACK_STRUCT_FIELD(int32_t optlen);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END

PACK_STRUCT_BEGIN
struct apicmd_getsockoptres_s
{
  PACK_STRUCT_FIELD(int32_t ret_code);
  PACK_STRUCT_FIELD(int32_t err_code);
  PACK_STRUCT_FIELD(int32_t optlen);
  PACK_STRUCT_FIELD(int8_t optval[APICMD_GETSOCKOPT_RES_OPTVAL_LENGTH]);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END

#endif /* __MODULES_LTE_FARAPI_INCLUDE_API_SOCKET_APICMD_GETSOCKOPT_H */
