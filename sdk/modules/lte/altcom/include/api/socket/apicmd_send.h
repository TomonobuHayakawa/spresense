/****************************************************************************
 * modules/lte/altcom/include/api/socket/apicmd_send.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_SOCKET_APICMD_SEND_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_SOCKET_APICMD_SEND_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "altcom_socket.h"
#include "apicmd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_SEND_FLAGS_MSG_PEEK         (ALTCOM_MSG_PEEK)
#define APICMD_SEND_FLAGS_MSG_MSG_WAITALL  (ALTCOM_MSG_WAITALL)
#define APICMD_SEND_FLAGS_MSG_MSG_OOB      (ALTCOM_MSG_OOB)
#define APICMD_SEND_FLAGS_MSG_MSG_DONTWAIT (ALTCOM_MSG_DONTWAIT)
#define APICMD_SEND_FLAGS_MSG_MSG_MORE     (ALTCOM_MSG_MORE)
#define APICMD_SEND_SENDDATA_LENGTH        (1500)

#define APICMD_SEND_RES_RET_CODE_OK  (0)
#define APICMD_SEND_RES_RET_CODE_ERR (-1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

PACK_STRUCT_BEGIN
struct apicmd_send_s
{
  PACK_STRUCT_FIELD(int32_t sockfd);
  PACK_STRUCT_FIELD(int32_t flags);
  PACK_STRUCT_FIELD(int32_t datalen);
  PACK_STRUCT_FIELD(int8_t senddata[APICMD_SEND_SENDDATA_LENGTH]);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END

PACK_STRUCT_BEGIN
struct apicmd_sendres_s
{
  PACK_STRUCT_FIELD(int32_t ret_code);
  PACK_STRUCT_FIELD(int32_t err_code);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_SOCKET_APICMD_SEND_H */
