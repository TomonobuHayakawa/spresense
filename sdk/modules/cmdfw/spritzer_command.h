/****************************************************************************
 * modules/cmdfw/spritzer_command.h
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

#ifndef SPRITZER_COMMAND_IF_H
#define SPRITZER_COMMAND_IF_H

#include <stdio.h>
#include <sdk/config.h>

typedef enum
{
    NoErr = 0,          /* No error */
    CheckSumErr,        /* check sum error */
#ifdef CONFIG_SPCOMMAND_ENABLE_CRCCHECK
    CrcErr,             /* crc error */
#endif /* CONFIG_SPCOMMAND_ENABLE_CRCCHECK */
    EventProcErr,       /* event proc result error */
    EventNotSupportErr, /* Not support event error */
    ParamErr = 0x0100,  /* parameter error */
    InvalidHandle,      /* invalid handle error */
    DataShortage,       /* packet data shortage */
    CmdBusy,            /* command proc busy */
} SpCmdResult;

typedef struct
{
    bool     req_reply;    /* reply requested */
    bool     is_reply;     /* reply command or not */
    bool     is_error;     /* error packet or not */
    uint8_t* payload_addr; /* payload address */
    uint16_t payload_size; /* payload size */
    uint16_t command_id;   /* command id */
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
    uint16_t sequence_no;  /* sequence No */
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */
} spcommand_packet_info;

typedef spcommand_packet_info spcommand_create_param;
typedef spcommand_packet_info event_param;

typedef uint32_t SpCmdFwHandle;

/**
 * @brief     Activate Spritzer Command Framework.
 *            This API must be called Before using Framework.
 * @param[out] p_handle : Handle for Framework
 * @param[in]  pccb     : Callback function for notify packet created
 * @return    true : Activation success
 */
bool SpritzerCommand_activate(SpCmdFwHandle* p_handle);

/**
 * @brief     Deactivate Spritzer Command Framework.
 * @param[in] handle : Handle for Framework
 * @return    true : Deactivation success
 */
bool SpritzerCommand_deactivate(SpCmdFwHandle handle);

/**
 * @brief     Put command(packet) data into internal buffer of framework.
 *            The packet must be put by this API before parsing.
 * @param[in] handle : Handle for Framework
 * @param[in] p_data : pointer to data that wants to put
 * @param[in] size   : data size
 * @return    NoErr, InvalidHandle
 */
SpCmdResult SpritzerCommand_putData(SpCmdFwHandle handle, uint8_t* p_data, uint32_t size);

/**
 * @brief     Excute command(packet) which is stored in internal buffer of framework.
 *            If reply command(includeing error reply) is created in this API, application
 *            can get packet by SpritzerCommand_createPacket API.
 * @param[in] handle : Handle for Framework
 * @return    NoErr, InvalidHandle, CmdBusy, DataShortage, ChecksumErr, CrcErr, EventProcErr, EventNotSupport
 */
SpCmdResult SpritzerCommand_execute(SpCmdFwHandle handle);

/**
 * @brief     Parse command(packet) data which is stored in internal buffer of framework.
 *            This API works only parse packet, not excute other proc and not create packet.
 * @param[in]  handle : Handle for Framework
 * @param[out] info   : parsed result (packet information)
 * @return    NoErr, InvalidHandle, DataShortage, ChecksumErr, CrcErr
 */
SpCmdResult SpritzerCommand_parse(SpCmdFwHandle handle, spcommand_packet_info* info);

/**
 * @brief     Process event work according to command_id and event parameter.
 *            If reply command(includeing error reply) is required, created packet.
 *            Application can get created packet data by SpritzerCommand_getPacket API.
 * @param[in]  handle : Handle for Framework
 * @param[in]  evparam : event process parameter
 * @return    NoErr, InvalidHandle, EventProcErr, EventNotSupport
 */
SpCmdResult SpritzerCommand_eventProc(SpCmdFwHandle handle, event_param* evparam);

/**
 * @brief     Create packet data on rule of Spritzer Command Framework
 *            Application can get created packet data by SpritzerCommand_getPacket API.
 * @param[in] handle : Handle for Framework
 * @param[in] cparam : Parameter for packet creation
 * @return    NoErr, InvalidHandle, CmdBusy, DataShortage, ChecksumErr, CrcErr, EventProcErr, EventNotSupport
 */
bool SpritzerCommand_createPacket(SpCmdFwHandle handle, spcommand_create_param* cparam);

/**
 * @brief     Get packet information.
 * @param[out] addr : Pointer to data
 * @param[out] size : size of data
 * @return    true : packet exists, false : packet not exists
 */
bool SpritzerCommand_getPacket(uint8_t** addr, uint16_t* size);

/**
 * @brief     Parse user required area packet information.
 *            Application can indeicate any buffer which including command packet, and this API
 *            replys packet information. Note, this API does "not" parse internal fifo of framework.
 * @param[in] addr : buffer address
 * @param[in] size : buffer size
 * @param[out] info : packet information
 * @return
 */
SpCmdResult SpritzerCommand_parseIndicatedArea(uint8_t* addr, uint16_t size, spcommand_packet_info* info);

/**
 * @brief     Notify completion of read and free buffer to spcommandfw.
 *            After read data from framework buffer, application should notify read size by this API.
 *            This API outputs remain data size and data addr. And, if current packet is all read,
 *            check succeding packet is required or not and create if required.
 *            Application can get created packet data by SpritzerCommand_getPacket API.
 * @param[in] handle       : Handle for Framework
 * @param[in] read_size    : A size which is application read
 * @param[out] next_addr   : Pointer to succeding data
 * @param[out] remain_size : Remaining size of data
 * @return    true : read succedss, false : indicated read_size couldn't read
 */
bool SpritzerCommand_freeData(SpCmdFwHandle handle, uint16_t read_size, uint8_t** next_addr, uint16_t* remain_size);

/**
 * @brief     Check evnt registered state
 * @param[in] command_id : Command id which wants to check
 * @return    true : Indicated command_id is registered
 */
bool SpritzerCommand_isEventRegistered(uint16_t command_id);

#endif /* SPRITZER_COMMAND_IF_H */

