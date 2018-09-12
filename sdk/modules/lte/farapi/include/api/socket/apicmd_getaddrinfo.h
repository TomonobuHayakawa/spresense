/****************************************************************************
 * modules/lte/farapi/include/api/socket/apicmd_getaddrinfo.h
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

#ifndef __MODULES_LTE_FARAPI_INCLUDE_API_SOCKET_APICMD_GETADDRINFO_H
#define __MODULES_LTE_FARAPI_INCLUDE_API_SOCKET_APICMD_GETADDRINFO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "farapi_socket.h"
#include "farapi_netdb.h"
#include "apicmd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_GETADDRINFO_NODENAME_MAX_LENGTH (256)
#define APICMD_GETADDRINFO_SERVNAME_MAX_LENGTH (32)
#define APICMD_GETADDRINFO_HINTS_FLAG_ENABLE   (1)
#define APICMD_GETADDRINFO_HINTS_FLAG_DISABLE  (0)
#define APICMD_GETADDRINFO_RES_AI_COUNT        (2)

#define APICMD_GETADDRINFO_AIFLAG_PASSIVE     (FARAPI_AI_PASSIVE)
#define APICMD_GETADDRINFO_AIFLAG_CANONNAME   (FARAPI_AI_CANONNAME)
#define APICMD_GETADDRINFO_AIFLAG_NUMERICHOST (FARAPI_AI_NUMERICHOST)
#define APICMD_GETADDRINFO_AIFLAG_NUMERICSERV (FARAPI_AI_NUMERICSERV)
#define APICMD_GETADDRINFO_AIFLAG_V4MAPPED    (FARAPI_AI_V4MAPPED)
#define APICMD_GETADDRINFO_AIFLAG_ALL         (FARAPI_AI_ALL)
#define APICMD_GETADDRINFO_AIFLAG_ADDRCONFIG  (FARAPI_AI_ADDRCONFIG)

#define APICMD_GETADDRINFO_ADDRFAMILY_UNSPEC (FARAPI_AF_UNSPEC)
#define APICMD_GETADDRINFO_ADDRFAMILY_INET   (FARAPI_AF_INET)
#define APICMD_GETADDRINFO_ADDRFAMILY_INET6  (FARAPI_AF_INET6)

#define APICMD_GETADDRINFO_PROTOCLTYPE_SOCK_STREAM (FARAPI_SOCK_STREAM)
#define APICMD_GETADDRINFO_PROTOCLTYPE_SOCK_DGRAM  (FARAPI_SOCK_DGRAM)
#define APICMD_GETADDRINFO_PROTOCLTYPE_SOCK_RAW    (FARAPI_SOCK_RAW)

#define APICMD_GETADDRINFO_AIPROTOCOL_IP      (FARAPI_IPPROTO_IP)
#define APICMD_GETADDRINFO_AIPROTOCOL_ICMP    (FARAPI_IPPROTO_ICMP)
#define APICMD_GETADDRINFO_AIPROTOCOL_TCP     (FARAPI_IPPROTO_TCP)
#define APICMD_GETADDRINFO_AIPROTOCOL_UDP     (FARAPI_IPPROTO_UDP)
#define APICMD_GETADDRINFO_AIPROTOCOL_IPV6    (FARAPI_IPPROTO_IPV6)
#define APICMD_GETADDRINFO_AIPROTOCOL_ICMPV6  (FARAPI_IPPROTO_ICMPV6)
#define APICMD_GETADDRINFO_AIPROTOCOL_UDPLITE (FARAPI_IPPROTO_UDPLITE)
#define APICMD_GETADDRINFO_AIPROTOCOL_RAW     (FARAPI_IPPROTO_RAW)

#define APICMD_GETADDRINFO_AI_CANONNAME_LENGTH (256)

#define APICMD_GETADDRINFO_RES_RET_CODE_OK  (0)
#define APICMD_GETADDRINFO_RES_RET_CODE_ERR (-1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

PACK_STRUCT_BEGIN
struct apicmd_getaddrinfo_ai_s
{
  PACK_STRUCT_FIELD(int32_t ai_flags);
  PACK_STRUCT_FIELD(int32_t ai_family);
  PACK_STRUCT_FIELD(int32_t ai_socktype);
  PACK_STRUCT_FIELD(int32_t ai_protocol);
  PACK_STRUCT_FIELD(farapi_socklen_t ai_addrlen);
  PACK_STRUCT_FIELD(struct farapi_sockaddr_storage ai_addr);
  PACK_STRUCT_FIELD(uint32_t ai_cnamelen);
  PACK_STRUCT_FIELD(
    int8_t ai_canonname[APICMD_GETADDRINFO_AI_CANONNAME_LENGTH]);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END


PACK_STRUCT_BEGIN
struct apicmd_getaddrinfo_s
{
  PACK_STRUCT_FIELD(uint32_t nodenamelen);
  PACK_STRUCT_FIELD(int8_t nodename[APICMD_GETADDRINFO_NODENAME_MAX_LENGTH]);
  PACK_STRUCT_FIELD(uint32_t servnamelen);
  PACK_STRUCT_FIELD(int8_t servname[APICMD_GETADDRINFO_SERVNAME_MAX_LENGTH]);
  PACK_STRUCT_FIELD(int32_t hints_flag);
  PACK_STRUCT_FIELD(int32_t ai_flags);
  PACK_STRUCT_FIELD(int32_t ai_family);
  PACK_STRUCT_FIELD(int32_t ai_socktype);
  PACK_STRUCT_FIELD(int32_t ai_protocol);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END

PACK_STRUCT_BEGIN
struct apicmd_getaddrinfores_s
{
  PACK_STRUCT_FIELD(int32_t ret_code);
  PACK_STRUCT_FIELD(uint32_t ai_num);
  PACK_STRUCT_FIELD(
    struct apicmd_getaddrinfo_ai_s ai[APICMD_GETADDRINFO_RES_AI_COUNT]);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END

#endif /* __MODULES_LTE_FARAPI_INCLUDE_API_SOCKET_APICMD_GETADDRINFO_H */
