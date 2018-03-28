/****************************************************************************
 * modules/audio/include/audio_manager_message_types.h
 *
 *   Copyright (C) 2017 Sony Corporation
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

#ifndef __MODULES_AUDIO_INCLUDE_AUDIO_MANAGER_MESSAGE_TYPE_H
#define __MODULES_AUDIO_INCLUDE_AUDIO_MANAGER_MESSAGE_TYPE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "memutils/message/Message.h"
#include "common/audio_message_types.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/************************************************************************
 *
 *  request / response
 *
 ************************************************************************
 */

#define MSG_TYPE_AUTIL_RES  (MSG_TYPE_RESPONSE | MSG_TYPE_USER_AUDIO_UTIL)
#define MSG_TYPE_AUTIL_REQ  (MSG_TYPE_REQUEST  | MSG_TYPE_USER_AUDIO_UTIL)

/************************************************************************
 *
 *  category
 *
 ************************************************************************
 */

#define MSG_CAT_AUTIL_MNG  (MSG_SET_CATEGORY(0x0))

/************************************************************************
 *
 *    MSG_CAT_AUTIL_MNG: Audio Manager Command/Result(bi-directional)
 *
 *   D15 D14 D13 D12 D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *  |REQ|USER_AUTIL |  MSG_CAT_MNG  | MSG_SUB_TYPE                  |
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *
 ************************************************************************
 */
#define MSG_AUD_MNG_REQ    (MSG_TYPE_AUTIL_REQ | MSG_CAT_AUTIL_MNG)
#define MSG_AUD_MNG_RES    (MSG_TYPE_AUTIL_RES | MSG_CAT_AUTIL_MNG)

#define MSG_AUD_MGR_CMD_PLAYER           (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x00))
#define MSG_AUD_MGR_CMD_SETREADY         (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x01))
#define MSG_AUD_MGR_CMD_SETACTIVE        (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x02))
#define MSG_AUD_MGR_CMD_SETPLAYER        (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x03))
#define MSG_AUD_MGR_CMD_VOICECOMMAND     (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x04))
#define MSG_AUD_MGR_CMD_SETRECORDER      (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x05))
#define MSG_AUD_MGR_CMD_RECORDER         (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x06))
#define MSG_AUD_MGR_CMD_SOUNDFX          (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x07))
#define MSG_AUD_MGR_CMD_MFE              (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x08))
#define MSG_AUD_MGR_CMD_MPP              (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x09))
#define MSG_AUD_MGR_CMD_GETSTATUS        (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x0a))
#define MSG_AUD_MGR_CMD_INITATTENTIONS   (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x0b))
#define MSG_AUD_MGR_CMD_INITMICGAIN      (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x0c))
#define MSG_AUD_MGR_CMD_INITI2SPARAM     (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x0d))
#define MSG_AUD_MGR_CMD_INITDEQPARAM     (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x0e))
#define MSG_AUD_MGR_CMD_INITOUTPUTSELECT (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x0f))
#define MSG_AUD_MGR_CMD_INITDNCPARAM     (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x10))
#define MSG_AUD_MGR_CMD_INITCLEARSTEREO  (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x11))
#define MSG_AUD_MGR_CMD_SETVOLUME        (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x12))
#define MSG_AUD_MGR_CMD_SETVOLUMEMUTE    (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x13))
#define MSG_AUD_MGR_CMD_SETBEEP          (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x14))
#define MSG_AUD_MGR_CMD_POWERON          (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x15))
#define MSG_AUD_MGR_CMD_POWEROFF         (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x16))
#define MSG_AUD_MGR_CMD_SUBPLAYER        (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x17))
#define MSG_AUD_MGR_CMD_SETTHROUGH       (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x18))
#define MSG_AUD_MGR_CMD_SETTHROUGHPATH   (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x19))
#define MSG_AUD_MGR_CMD_SETRENDERINGCLK  (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x1a))
#define MSG_AUD_MGR_CMD_INVALID          (MSG_AUD_MNG_REQ | MSG_SET_SUBTYPE(0x1b))

#define LAST_AUD_MGR_MSG     (MSG_AUD_MGR_CMD_INVALID + 1)
#define AUD_MGR_MSG_NUM      (LAST_AUD_MGR_MSG & MSG_TYPE_SUBTYPE)

#define MSG_AUD_MGR_CALL_ATTENTION  (MSG_AUD_MNG_RES | MSG_SET_SUBTYPE(0x01))

/************************************************************************
 *
 *  responce from AudioSubSystem
 *
 ************************************************************************
 */

#define MSG_AUD_MGR_RST             (MSG_TYPE_AUD_RES)
#define MSG_AUD_MGR_FIND_TRIGGER    (MSG_AUD_RCG_FIND_TRIGGER)
#define MSG_AUD_MGR_FIND_COMMAND    (MSG_AUD_RCG_FIND_COMMAND)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __MODULES_AUDIO_INCLUDE_AUDIO_MANAGER_MESSAGE_TYPE_H */
