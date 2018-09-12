/****************************************************************************
 * modules/lte/farapi/include/api/socket/apicmd_recvfrom.h
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

#ifndef __MODULES_LTE_FARAPI_INCLUDE_API_SOCKET_APICMD_RECVFROM_H
#define __MODULES_LTE_FARAPI_INCLUDE_API_SOCKET_APICMD_RECVFROM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "farapi_socket.h"
#include "apicmd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_RECVFROM_FLAGS_MSG_PEEK      (FARAPI_MSG_PEEK)
#define APICMD_RECVFROM_FLAGS_MSG_WAITALL   (FARAPI_MSG_WAITALL)
#define APICMD_RECVFROM_FLAGS_MSG_OOB       (FARAPI_MSG_OOB)
#define APICMD_RECVFROM_FLAGS_MSG_DONTWAIT  (FARAPI_MSG_DONTWAIT)
#define APICMD_RECVFROM_FLAGS_MSG_MORE      (FARAPI_MSG_MORE)
#define APICMD_RECVFROM_RES_RECVDATA_LENGTH (1500)

#define APICMD_RECVFROM_RES_RET_CODE_OK  (0)
#define APICMD_RECVFROM_RES_RET_CODE_ERR (-1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

PACK_STRUCT_BEGIN
struct apicmd_recvfrom_s
{
  PACK_STRUCT_FIELD(int32_t sockfd);
  PACK_STRUCT_FIELD(int32_t recvlen);
  PACK_STRUCT_FIELD(int32_t flags);
  PACK_STRUCT_FIELD(uint32_t fromlen);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END

PACK_STRUCT_BEGIN
struct apicmd_recvfromres_s
{
  PACK_STRUCT_FIELD(int32_t ret_code);
  PACK_STRUCT_FIELD(int32_t err_code);
  PACK_STRUCT_FIELD(uint32_t fromlen);
  PACK_STRUCT_FIELD(struct farapi_sockaddr_storage from);
  PACK_STRUCT_FIELD(int8_t recvdata[APICMD_RECVFROM_RES_RECVDATA_LENGTH]);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END

#endif /* __MODULES_LTE_FARAPI_INCLUDE_API_SOCKET_APICMD_RECVFROM_H */
