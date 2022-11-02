// ==========================================================================
/*!
* @file     CXM150x_CTRL_FW_UPDATE_Port.c
* @brief    Define HAL wrapper functions for CONTROL FW UPDATE
* @date     2021/08/16
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

#include <stdio.h>
#include "CXM150x_APITypeDef.h"
#include "CXM150x_CTRL_FW_UPDATE_Port.h"
#include "CXM150x_Port.h"
#include "stm32l0xx_hal.h"
#include "usart.h"

#define FOR_STM32_HAL_DRIVER_CONTROL_FW_UPDATE

// Compile only if certain symbols are defined in CXM150x_APITypedef.h
#if CXM150x_CTRL_FW_UPDATE_API_USE


// ===========================================================================
//! Check the checksum
/*!
 *
 * @param [in] rx_buf: Received message
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return confirmation result
*/
// ===========================================================================
uint32_t sum_check(uint8_t *rx_buf){
    uint32_t sum = 0;
    uint8_t data_len = rx_buf[CXM150x_CTRL_FW_UPDATE_API_POS_LE0];
    uint8_t calc_sum;
    uint8_t rx_sum;

    // Checksum calculation
    sum = rx_buf[CXM150x_CTRL_FW_UPDATE_API_POS_RB]
         + rx_buf[CXM150x_CTRL_FW_UPDATE_API_POS_NUL]
         + rx_buf[CXM150x_CTRL_FW_UPDATE_API_POS_CMD];
    
    for(uint32_t i=0;i<data_len;i++){
        sum += rx_buf[CXM150x_CTRL_FW_UPDATE_API_POS_DATA+i];
    }
    
    calc_sum = (uint8_t)(sum & 0x000000FF);
    rx_sum = rx_buf[CXM150x_CTRL_FW_UPDATE_API_POS_DATA + data_len];
    
    if(calc_sum != rx_sum){
        printf("sum check error(0x%02X)\r\n",calc_sum);
        return CXM150x_RESPONSE_NG;
    }
    
    return CXM150x_RESPONSE_OK;
    
}

// ===========================================================================
//! Perform UART reception in CONTROL FW UPDATE mode
/*!
 *
 * @param [in] wait_cnt: Maximum message reception wait count (specified in milliseconds)
 * @param [out] rx_buf: Receive buffer
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return receive result
*/
// ===========================================================================
return_code wrapper_CXM150x_ctrl_fw_update_rx_message(uint8_t *rx_buf,uint32_t wait_cnt){
    uint8_t c = 0;
    uint32_t rcv_cnt = 0;
    uint32_t dt_n = 0;
    int32_t set_result;
    uint32_t st_tick = wrapper_CXM150x_get_tick();
    uint32_t c_tick;
    uint8_t ed_val;

    printf("rcv: ");

    // Receive the 1st byte and check if it is CONTROL FW UPDATE communication format
    while(rcv_cnt <= CXM150x_CTRL_FW_UPDATE_API_POS_SD0){
#ifdef FOR_STM32_HAL_DRIVER_CONTROL_FW_UPDATE
        set_result = HAL_UART_Receive(&huart1,&c,1,wait_cnt);
        if(set_result == HAL_OK){
            printf("%02X ",c);
            rx_buf[rcv_cnt++] = c;
        }
#endif
        // timeout check
        c_tick = wrapper_CXM150x_get_tick();
        if(c_tick - st_tick > wait_cnt){
            printf("\r\ntimeout command.(%dmsec)\r\n",wait_cnt);
            return RETURN_TIMEOUT;
        }
    }
    
    // Switches reception processing depending on whether the communication format is CONTROL FW UPDATE or not
    if(rx_buf[CXM150x_CTRL_FW_UPDATE_API_POS_SD0] == CXM150x_CTRL_FW_UPDATE_API_SD){
        // In case of CONTROL FW UPDATE communication format
        
        // Receive data up to LE
        while(rcv_cnt <= CXM150x_CTRL_FW_UPDATE_API_POS_LE1){
#ifdef FOR_STM32_HAL_DRIVER_CONTROL_FW_UPDATE
            set_result = HAL_UART_Receive(&huart1,&c,1,wait_cnt);
            if(set_result == HAL_OK){
                printf("%02X ",c);
                rx_buf[rcv_cnt++] = c;
            }
#endif
            // timeout check
            c_tick = wrapper_CXM150x_get_tick();
            if(c_tick - st_tick > wait_cnt){
                printf("\r\ntimeout command.(%dmsec)\r\n",wait_cnt);
                return RETURN_TIMEOUT;
            }
        }
        
        // error if first LE and second LE are different
        if(rx_buf[CXM150x_CTRL_FW_UPDATE_API_POS_LE0] != rx_buf[CXM150x_CTRL_FW_UPDATE_API_POS_LE1]){
            printf("LE0-LE1 invalid.\r\n");
            return RETURN_NG;
        }
        // Get the length of the DATA section
        dt_n = rx_buf[CXM150x_CTRL_FW_UPDATE_API_POS_LE0];
        
        // Length check of DATA section (maximum 240 bytes)
        if(dt_n > CTRL_FW_BINARY_IMAGE_MAX_LEN){
            printf("dt_n invalid.\r\n");
            return RETURN_NG;
        }
        
        // Receive after DATA (DATA length + SUM, ED)
        while(rcv_cnt < (CXM150x_CTRL_FW_UPDATE_API_MSG_INF_LEN + dt_n)){
#ifdef FOR_STM32_HAL_DRIVER_CONTROL_FW_UPDATE
            set_result = HAL_UART_Receive(&huart1,&c,1,wait_cnt);
            if(set_result == HAL_OK){
                printf("%02X ",c);
                rx_buf[rcv_cnt++] = c;
            }
#endif
            // timeout check
            c_tick = wrapper_CXM150x_get_tick();
            if(c_tick - st_tick > wait_cnt){
                printf("\r\ntimeout command.(%dmsec)\r\n",wait_cnt);
                return RETURN_TIMEOUT;
            }
        }
        printf("\r\n");

    } else {
        // Case other than CONTROL FW UPDATE communication format
        // If UART communication starts other than 0x68, receive until line feed code is received
        while(1){
#ifdef FOR_STM32_HAL_DRIVER_CONTROL_FW_UPDATE
            set_result = HAL_UART_Receive(&huart1,&c,1,wait_cnt);
            if(set_result == HAL_OK){
                printf("%02X ",c);
                if(rcv_cnt >= CXM150x_CTRL_FW_UPDATE_MAX_COMMAND_LEN - 1){     // Subtract one because '\0' requires 1 byte
                    rx_buf[rcv_cnt] = '\0';
                    printf("rcv_cnt over.");
                    return RETURN_NG;
                }
                rx_buf[rcv_cnt++] = c;
                if(c == '\n'){
                    rx_buf[rcv_cnt] = '\0';
                    printf("\r\n");
                    return RETURN_OK;
                }
            }
#endif
            // timeout check
            c_tick = wrapper_CXM150x_get_tick();
            if(c_tick - st_tick > wait_cnt){
                printf("\r\ntimeout command.(%dmsec)\r\n",wait_cnt);
                return RETURN_TIMEOUT;
            }
        }
    }
    
    // Check checksum
    if(sum_check(rx_buf) != CXM150x_RESPONSE_OK){
        return RETURN_NG;
    }
    
    // Check that ED is 0x16
    ed_val = rx_buf[CXM150x_CTRL_FW_UPDATE_API_POS_DATA + dt_n + 1];
    if(ed_val != CXM150x_CTRL_FW_UPDATE_API_ED){
        printf("ED chek error.\r\n");
        return RETURN_NG;
    }
    return RETURN_OK;
}

// ===========================================================================
//! Send UART in CONTROL FW UPDATE mode
/*!
 *
 * @param [in] snd_buf: Send message buffer
 * @param [in] snd_cnt: Number of transmit bytes
 * @param [in] wait_cnt: Maximum message reception wait count (specified in milliseconds)
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return transmission result
*/
// ===========================================================================
return_code wrapper_CXM150x_ctrl_fw_update_tx_message(uint8_t *snd_buf,uint8_t snd_cnt,uint32_t wait_cnt){
    // display sent message
    printf("snd: ");
    for(uint32_t i=0;i<snd_cnt;i++){
        printf("%02X ",snd_buf[i]);
    }
    printf("\r\n");

#ifdef FOR_STM32_HAL_DRIVER_CONTROL_FW_UPDATE
    HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart1,snd_buf,snd_cnt,wait_cnt);
    if(ret == HAL_TIMEOUT){
        printf("transmit timeout.\r\n");
        return RETURN_TIMEOUT;
    } else if(ret == HAL_BUSY || ret == HAL_ERROR){
        printf("transmit error.(%d)\r\n",ret);
        return RETURN_NG;
    }
#endif

    return RETURN_OK;
}

// ===========================================================================
//! Abort UART IT
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 *
 * @return none
*/
// ===========================================================================
return_code wrapper_CXM150x_ctrl_fw_update_uart_abort_IT(void){
#ifdef FOR_STM32_HAL_DRIVER_CONTROL_FW_UPDATE
    HAL_StatusTypeDef ret = HAL_UART_Abort_IT(&huart1);
    if(ret == HAL_TIMEOUT){
        printf("abort timeout.\r\n");
        return RETURN_TIMEOUT;
    } else if(ret == HAL_BUSY || ret == HAL_ERROR){
        printf("abort error.(%d)\r\n",ret);
        return RETURN_NG;
    }
#endif
    return RETURN_OK;
}

#endif  //CXM150x_CTRL_FW_UPDATE_API_USE





