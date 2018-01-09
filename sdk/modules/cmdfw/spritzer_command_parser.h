/****************************************************************************
 * modules/cmdfw/spritzer_command_parser.h
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

#ifndef SPRITZER_COMMAND_PARSER_H
#define SPRITZER_COMMAND_PARSER_H

#include <stdio.h>
#include "spritzer_command.h"
#include "spritzer_command_common.h"

#define NUM_ADSET_REGION 2

typedef enum
{
    DELIMITER_NOTFOUND = 0,
    DELIMITER_1STBYTE,
    DELIMITER_2NDBYTE,
} DelimiterState;

typedef struct
{
    struct addr_set_t
    {
        uint8_t* addr;
        uint16_t size;
    } adset[NUM_ADSET_REGION];
} spcommand_adset_t;

bool SpritzerCommandParser_activate(SpCmdFwHandle* p_handle);
bool SpritzerCommandParser_deactivate(void);
SpCmdResult SpritzerCommandParser_putData(uint8_t* p_data, uint32_t size);
SpCmdResult SpritzerCommandParser_parse(spcommand_header_t* header, uint8_t* payload);
SpCmdResult SpritzerCommandParser_parseIndicatedArea(uint8_t* target_area, uint16_t target_size, spcommand_packet_info* info);

#endif /* SPRITZER_COMMAND_PARSER_H */

