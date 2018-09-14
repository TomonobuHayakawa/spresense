/****************************************************************************
 * modules/include/bluetooth/bluetooth_avrcp_cmds.h
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

#ifndef __MODULES_INCLUDE_BLUETOOTH_BLUETOOTH_AVRCP_CMDS_H
#define __MODULES_INCLUDE_BLUETOOTH_BLUETOOTH_AVRCP_CMDS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @defgroup bt_datatypes Data types
 * @{
 */


/**@brief AVRCP command list
 */
typedef enum
{
  BT_AVRCP_CMD_SELECT     = 0x00,
  BT_AVRCP_CMD_UP,
  BT_AVRCP_CMD_DOWN,
  BT_AVRCP_CMD_LEFT,
  BT_AVRCP_CMD_RIGHT,
  BT_AVRCP_CMD_RIGHT_UP,
  BT_AVRCP_CMD_RIGHT_DOWN,
  BT_AVRCP_CMD_LEFT_UP,
  BT_AVRCP_CMD_LEFT_DOWN,
  BT_AVRCP_CMD_ROOT_MENU,
  BT_AVRCP_CMD_SETUP_MENU,
  BT_AVRCP_CMD_CONT_MENU,
  BT_AVRCP_CMD_FAV_MENU,
  BT_AVRCP_CMD_EXIT,
  BT_AVRCP_CMD_0          = 0x20,
  BT_AVRCP_CMD_1,
  BT_AVRCP_CMD_2,
  BT_AVRCP_CMD_3,
  BT_AVRCP_CMD_4,
  BT_AVRCP_CMD_5,
  BT_AVRCP_CMD_6,
  BT_AVRCP_CMD_7,
  BT_AVRCP_CMD_8,
  BT_AVRCP_CMD_9,
  BT_AVRCP_CMD_DOT,
  BT_AVRCP_CMD_ENTER,
  BT_AVRCP_CMD_CLEAR,
  BT_AVRCP_CMD_CHAN_UP    = 0x30,
  BT_AVRCP_CMD_CHAN_DOWN,
  BT_AVRCP_CMD_PREV_CHAN,
  BT_AVRCP_CMD_SOUND_SEL,
  BT_AVRCP_CMD_INPUT_SEL,
  BT_AVRCP_CMD_DISP_INFO,
  BT_AVRCP_CMD_HELP,
  BT_AVRCP_CMD_PAGE_UP,
  BT_AVRCP_CMD_PAGE_DOWN,
  BT_AVRCP_CMD_POWER      = 0x40,
  BT_AVRCP_CMD_VOL_UP,
  BT_AVRCP_CMD_VOL_DOWN,
  BT_AVRCP_CMD_MUTE,
  BT_AVRCP_CMD_PLAY,
  BT_AVRCP_CMD_STOP,
  BT_AVRCP_CMD_PAUSE,
  BT_AVRCP_CMD_RECORD,
  BT_AVRCP_CMD_REWIND,
  BT_AVRCP_CMD_FAST_FOR,
  BT_AVRCP_CMD_EJECT,
  BT_AVRCP_CMD_FORWARD,
  BT_AVRCP_CMD_BACKWARD,
  BT_AVRCP_CMD_ANGLE      = 0x50,
  BT_AVRCP_CMD_SUBPICT,
  BT_AVRCP_CMD_F1         = 0x71,
  BT_AVRCP_CMD_F2,
  BT_AVRCP_CMD_F3,
  BT_AVRCP_CMD_F4,
  BT_AVRCP_CMD_F5,
  BT_AVRCP_CMD_VENDOR     = 0x7E
} BT_AVRCP_CMD_ID;

#endif /* __MODULES_INCLUDE_BLUETOOTH_BLUETOOTH_AVRCP_CMDS_H */
