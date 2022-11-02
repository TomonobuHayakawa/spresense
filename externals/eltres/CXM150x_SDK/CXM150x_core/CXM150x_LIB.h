// ==========================================================================
/*!
* @file     CXM150x_LIB.h
* @brief    UART communication related driver
* @date     2021/12/27
*
* Copyright 2021 Sony Semiconductor Solutions Corporation
* 
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
* 
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation and/or
* other materials provided with the distribution.
* 
* 3. Neither the name of Sony Semiconductor Solutions Corporation nor the names of
* its contributors may be used to endorse or promote products derived from this
* software without specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/
// =========================================================================

#ifndef __CXM150x_LIB_H
#define __CXM150x_LIB_H

#include "CXM150x_APITypeDef.h"

// Communication wait time with CXM (unit is msec)
#define MAX_TIME_OUT_TICK_COUNT     (5000)
// Timeout period for CXM150x power ON message wait
#define MAX_POWER_ON_TIME_OUT_TICK_COUNT     (10000)
// Profile changing timeout time
#define MAX_SET_PROFILE_TIME_OUT_TICK_COUNT     (50000)
// Timeout period from receiving the first character of UART communication to receiving CR + LF
#define MAX_UART_LINE_TIME_OUT_TICK_COUNT     (3000)
// Mode setting timeout time
#define MAX_SET_MODE_TIME_OUT_TICK_COUNT     (10000)

// Flag ON, OFF
#define UART_DRIVER_FLAG_ON         (1)
#define UART_DRIVER_FLAG_OFF        (0)

// Command receive result
#define COMMAND_RESULT_RESPONSE_WAIT        (0)
#define COMMAND_RESULT_OK                   (1)
#define COMMAND_RESULT_ERROR                (2)
#define COMMAND_RESULT_BUSY                 (3)
#define COMMAND_RESULT_TIME_OUT             (4)
#define COMMAND_RESULT_COMMAND_INVALID      (5)

// Command transmission result
#define COMMAND_SEND_RESULT_OK              (0)
#define COMMAND_SEND_RESULT_BUSY            (1)
#define COMMAND_SEND_RESULT_NG              (2)
#define COMMAND_SEND_RESULT_TIME_OUT        (3)

/* Command string definition */
// first character of command
#define CXM150x_COMMAND_PREFIX_CHAR    "<"

// command string (group + target string)
#define CXM150x_COMMAND_SYS_VER "SYS VER"
#define CXM150x_COMMAND_SYS_MODE "SYS MODE"
#define CXM150x_COMMAND_SYS_STT "SYS STT"
#define CXM150x_COMMAND_SYS_RESET "SYS RESET"
#define CXM150x_COMMAND_SYS_TO_FETCHING "SYS TO_FETCHING"
#define CXM150x_COMMAND_SYS_TO_WAIT_FETCHING "SYS TO_WAIT_FETCHING"
#define CXM150x_COMMAND_SYS_EEPROM "SYS EEPROM"
#define CXM150x_COMMAND_SYS_BTVER "SYS BTVER"
#define CXM150x_COMMAND_SYS_TO_GNSS_BACKUP "SYS TO_GNSS_BACKUP"
#define CXM150x_COMMAND_SYS_TO_DSLP "SYS TO_DSLP"
#define CXM150x_COMMAND_TX_PLD "TX PLD"
#define CXM150x_COMMAND_TX_CUR_FRM_TYPE "TX CUR_FRM_TYPE"
#define CXM150x_COMMAND_TX_PREV_PLD "TX PREV_PLD"
#define CXM150x_COMMAND_TX_PREV_STT "TX PREV_STT"
#define CXM150x_COMMAND_TX_PREV_TIME "TX PREV_TIME"
#define CXM150x_COMMAND_TX_NEXT_TIME "TX NEXT_TIME"
#define CXM150x_COMMAND_TX_POC_EN "TX POC_EN"
#define CXM150x_COMMAND_TX_DUTY "TX DUTY"

#define CXM150x_COMMAND_GNSS_VER "GNSS VER"
#define CXM150x_COMMAND_GNSS_STT "GNSS STT"
#define CXM150x_COMMAND_GNSS_HOST_FW_SENT "GNSS HOST_FW_SENT"
#define CXM150x_COMMAND_GNSS_GPOE "GNSS GPOE"
#define CXM150x_COMMAND_GNSS_GTIM "GNSS GTIM"
#define CXM150x_COMMAND_TIME_PREM "TIME PREM"
#define CXM150x_COMMAND_TIME_DREM "TIME DREM"
#define CXM150x_COMMAND_TIME_TREM "TIME TREM"
#define CXM150x_COMMAND_TIME_CTIME "TIME CTIME"


// action string
#define CXM150x_COMMAND_SET "SET"
#define CXM150x_COMMAND_GET "GET"
#define CXM150x_COMMAND_SET_EVT "SET_EVT"
#define CXM150x_COMMAND_GET_EVT "GET_EVT"

#define CXM150x_COMMAND_ON "ON"
#define CXM150x_COMMAND_OFF "OFF"

// Transmission profile change command string(for profile specification)
#define CXM150x_COMMAND_TX_CUR_FRM_TYPE_MODE_STR_P1 "P,1"
#define CXM150x_COMMAND_TX_CUR_FRM_TYPE_MODE_STR_P2 "P,2"
#define CXM150x_COMMAND_TX_CUR_FRM_TYPE_MODE_STR_E  "E"
#define CXM150x_COMMAND_TX_CUR_FRM_TYPE_MODE_STR_P  "P"

// Response data structure
typedef struct {
    uint32_t m_result_code;                        // OK, NG (NG in response from CXM150x), timeout, BUSY
    int32_t m_option_num;                         // Numeric response (XX seconds, etc.)
    uint8_t m_option_str[RECEIVE_BUF_SIZE];      // On / Off, version information, previous payload, GPS time, etc.
}CommandResponseInfo;

return_code wait_power_on_message(void);
uint32_t init_uart_driver(void);
void trigger_analyse(void);
void uart_receive_to_buffer_callback(uint32_t type_from,uint32_t rcv_cnt);
return_code send_and_wait_command_response(uint8_t *cmd,uint8_t *respons_message);
return_code send_and_wait_command_response_long_wait(uint8_t *cmd,uint8_t *respons_message,uint32_t max_wait);
return_code send_and_register_callback(uint8_t *cmd,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER res_func,CXM150x_RES_PARSE_CALLBACK_FUNC_POINTER parse_func,void *ret_struct);
return_code send_and_register_callback_long_wait(uint8_t *cmd,CXM150x_CALLBACK_RESPONSE_FUNC_POINTER res_func,CXM150x_RES_PARSE_CALLBACK_FUNC_POINTER parse_func,void *ret_struct,uint32_t max_wait);
return_code prep_wait_power_on_message(CXM150x_CALLBACK_RESPONSE_FUNC_POINTER res_func,CXM150x_RES_PARSE_CALLBACK_FUNC_POINTER parse_func,void *ret_struct);
uint32_t get_CXM150x_Rx_message_count(void);
return_code prep_wait_power_on_message_reset(void);

// Definition for test mode
#if CXM150x_TEST_MODE_API_USE
#define CXM150x_COMMAND_TEST_TX_CH "TEST TX_CH"
#define CXM150x_COMMAND_TEST_TX_MODE "TEST TX_MODE"
#define CXM150x_COMMAND_TEST_TX_RUN "TEST TX_RUN"
#define CXM150x_COMMAND_TEST_GPI "TEST GPI"
#define CXM150x_COMMAND_TEST_GPO "TEST GPO"

#define CXM150x_COMMAND_GPIO_PORT_H "H"
#define CXM150x_COMMAND_GPIO_PORT_L "L"
#endif


#endif // __CXM150x_LIB_H
