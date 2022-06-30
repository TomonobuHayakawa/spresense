// ==========================================================================
/*!
* @file     main_standalone_mode_sample_app.h
* @brief    header file of main_standalone_mode_sample_app.c
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_STANDALONE_MODE_SAMPLE_APP_H
#define __MAIN_STANDALONE_MODE_SAMPLE_APP_H
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "stm32l0xx_hal.h"
#include "CXM150x_typedef.h"
#include "CXM150x_Port.h"
/* Includes ------------------------------------------------------------------*/
int main_standalone_mode_sample_app(void);
void uart_receive_to_buffer_callback(uint32_t type_from,uint32_t rcv_cnt);
void int2_callback(void);

/* Private define ------------------------------------------------------------*/

#endif /* __MAIN_STANDALONE_MODE_SAMPLE_APP_H */




