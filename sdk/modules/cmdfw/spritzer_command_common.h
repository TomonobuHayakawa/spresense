/****************************************************************************
 * modules/cmdfw/spritzer_command_common.h
 *
 *   Copyright (C) 2017 Sony Corpration. All rights reserved.
 *   Author: Ryuuta Sakane <Ryuuta.Sakane@Sony.com>
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
 * 3. Neither the name NuttX nor Sony nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#ifndef SPRITZER_COMMAND_COMMON_H
#define SPRITZER_COMMAND_COMMON_H

typedef enum
{
    Ready = 0x00,
    Working = 0x01,
    SyncViolationInt = 0x01 << 1,
    ReplyInt = 0x01 << 2,
} SpCmdState;

typedef enum
{
    SPCMD_ERROR = 0,
    SPCMD_REPLY_REQUEST = 4,
    SPCMD_REPLY,
} SpCmdPrmBit;

typedef struct
{
    uint16_t delimiter;
    uint16_t command_id:10;
    uint16_t param:6;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
    uint16_t sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */
    uint16_t payload_size;
    uint16_t check_sum;
} spcommand_header_t;

#define COMMAND_DELIMITER   0xAA55

#ifdef CONFIG_SPCOMMAND_ENABLE_CRCCHECK
#define SPCOMMAND_CRC_SIZE (2)
#else
#define SPCOMMAND_CRC_SIZE (0)
#endif /* CONFIG_SPCOMMAND_ENABLE_CRCCHECK */

#define PAYLOAD_BUFFER_SIZE  (2048 + SPCOMMAND_CRC_SIZE)

#endif /* SPRITZER_COMMAND_COMMON_H */

