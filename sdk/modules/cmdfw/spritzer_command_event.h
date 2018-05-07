/****************************************************************************
 * modules/cmdfw/spritzer_command_event.h
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

#ifndef SPRITZER_COMMAND_EVENT_H
#define SPRITZER_COMMAND_EVENT_H

#include <sdk/config.h>
#include <stdio.h>
#include "spritzer_command.h"
#include "spritzer_command_common.h"

/* Command IDs */

#define COMMAND_ID_CATEGORY_SYSTEM          (0x00)
#define COMMAND_ID_CATEGORY_SENSOR          (0x20)

#define COMMAND_ID_GET_FIRMWARE_VERSION     (COMMAND_ID_CATEGORY_SYSTEM | 0x00)
#ifdef CONFIG_BOARDCTL_RESET
#define COMMAND_ID_RESET_SYSTEM             (COMMAND_ID_CATEGORY_SYSTEM | 0x01)
#endif /* CONFIG_BOARDCTL_RESET */
#define COMMAND_ID_RESISTER_EVENT           (COMMAND_ID_CATEGORY_SYSTEM | 0x02)
#define COMMAND_ID_UNRESISTER_EVENT         (COMMAND_ID_CATEGORY_SYSTEM | 0x03)
#ifdef CONFIG_BOARDCTL_POWEROFF
#define COMMAND_ID_POWER_OFF                (COMMAND_ID_CATEGORY_SYSTEM | 0x04)
#endif /* CONFIG_BOARDCTL_POWEROFF */
#define COMMAND_ID_LOWBATTERY_ALERT         (COMMAND_ID_CATEGORY_SYSTEM | 0x06)
#define COMMAND_ID_FIRMUP_EVENT             (COMMAND_ID_CATEGORY_SYSTEM | 0x11)
#define COMMAND_ID_WRITEFILE_EVENT          (COMMAND_ID_CATEGORY_SYSTEM | 0x14)
#define COMMAND_ID_READFILE_EVENT           (COMMAND_ID_CATEGORY_SYSTEM | 0x15)

#define COMMAND_ID_ENABLE_SENSORS           (COMMAND_ID_CATEGORY_SENSOR | 0x01)
#define COMMAND_ID_DISABLE_SENSORS          (COMMAND_ID_CATEGORY_SENSOR | 0x02)
#define COMMAND_ID_SENSOR_DATA              (COMMAND_ID_CATEGORY_SENSOR | 0x03)
#define COMMAND_ID_ENABLE_ACT_RECOGNTION    (COMMAND_ID_CATEGORY_SENSOR | 0x05)
#define COMMAND_ID_DISABLE_ACT_RECOGNTION   (COMMAND_ID_CATEGORY_SENSOR | 0x06)

#define COMMAND_ID_USER_EVENTS              (0x200)

/* Battery Level */

#define SPCOMMAND_BATTERY_LEVEL_FULL  (0xff)
#define SPCOMMAND_BATTERY_LEVEL_LOW   (0x10)
#define SPCOMMAND_BATTERY_LEVEL_EMPTY (0x00)

/* Spontaneout event identification bit define */

typedef enum
{
    SPCOMMAND_BIT_BATTERY_ALERT = 0,
    SPCOMMAND_BIT_USER_DEFINED1 = 16,
    SPCOMMAND_BIT_USER_DEFINED2,
    SPCOMMAND_BIT_USER_DEFINED3,
    SPCOMMAND_BIT_USER_DEFINED4,
    SPCOMMAND_BIT_USER_DEFINED5,
    SPCOMMAND_BIT_USER_DEFINED6,
    SPCOMMAND_BIT_USER_DEFINED7,
    SPCOMMAND_BIT_USER_DEFINED8,
    SPCOMMAND_BIT_USER_DEFINED9,
    SPCOMMAND_BIT_USER_DEFINED10,
    SPCOMMAND_BIT_USER_DEFINED11,
    SPCOMMAND_BIT_USER_DEFINED12,
    SPCOMMAND_BIT_USER_DEFINED13,
    SPCOMMAND_BIT_USER_DEFINED14,
    SPCOMMAND_BIT_USER_DEFINED15,
    SPCOMMAND_BIT_USER_DEFINED16,
} SpontaneousEventBit;

/* Sensor ids for spcommandfw */

typedef enum
{
    SPCOMMAND_SENSOR_ACCELE = 0,
    SPCOMMAND_SENSOR_PRESSURE = 8,
    SPCOMMAND_SENSOR_TEMPERTURE,
    SPCOMMAND_SENSOR_MAGNE = 15,
    SPCOMMAND_SENSOR_GNSS = 20,
    SPCOMMAND_SENSOR_BAROMETER = 24,
    SPCOMMAND_SENSOR_COMPASS,
} SpcmdSensorId;

/* Activity Recognizer ids for spcommandfw */

typedef enum
{
    SPCOMMAND_ACTRECOG_GESTURE = 0,
    SPCOMMAND_ACTRECOG_STEPCOUNTER,
    SPCOMMAND_ACTRECOG_TRAM,
} SpcmdActRecogId;

/* State of iteration work */

typedef enum
{
    SPCOMMAND_ITERATE_NONE = 0,
    SPCOMMAND_ITERATE_READFILE,
    SPCOMMAND_ITERATE_STATENUM,
} IterationState;

typedef int err_t;
typedef err_t (*evevt_func_t)(event_param* param);

typedef struct
{
    ushort  id;
    evevt_func_t event;
} event_element;

typedef struct
{
    ushort  id;
    SpontaneousEventBit bit;
} spontaneous_event_table;

typedef bool (*sensor_callback)(SpcmdSensorId id, bool act/* true = enable */);
typedef bool (*act_recog_callback)(SpcmdActRecogId id, bool act/* ture = enable */);

/* public functions */

bool SpritzerCommandEvent_activate(void);
bool SpritzerCommandEvent_deactivate(void);
SpCmdResult SpritzerCommandEvent_proc(event_param* evparam);
bool SpritzerCommandEvent_isEventRegistered(uint16_t command_id);
bool SpritzerCommandEvent_nextPacket(void);
bool SpritzerCommandEvent_registerSensorCb(sensor_callback func);
bool SpritzerCommandEvent_registerActRecogCb(act_recog_callback func);

#endif /* SPRITZER_COMMAND_EVENT_H */

