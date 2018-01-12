/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#ifndef __ARCH_SYS_ARCH_H__
#define __ARCH_SYS_ARCH_H__

#include "net/lwip/opt.h"

#if NO_SYS == 0
#if defined(CONFIG_OS_MERLOT)
#include <FreeRTOS.h>
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define TRUE 1
#define FALSE 0

typedef SYS_Task sys_thread_t;

typedef SYS_Id sys_sem_t;
typedef SYS_Id sys_mutex_t;
typedef SYS_Id sys_mbox_t;

typedef void * xQueueHandle;
typedef void * xTaskHandle;
typedef void * xQueueSetHandle;
typedef void * xQueueSetMemberHandle;

typedef xQueueHandle xSemaphoreHandle;

#define SYS_MBOX_NULL					( ( xQueueHandle ) NULL )
#define SYS_SEM_NULL					( ( xSemaphoreHandle ) NULL )
#define SYS_DEFAULT_THREAD_STACK_DEPTH	configMINIMAL_STACK_SIZE

typedef uint32_t sys_prot_t;

#define sys_mbox_valid( x ) ( ( ( *x ) == 0) ? FALSE : TRUE )
#define sys_mbox_set_invalid( x ) ( ( *x ) = 0 )
#define sys_sem_valid( x ) ( ( ( *x ) == 0) ? FALSE : TRUE )
#define sys_sem_set_invalid( x ) ( ( *x ) = 0 )

#elif defined(CONFIG_OS_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define SYS_MBOX_NULL					( ( xQueueHandle ) NULL )
#define SYS_SEM_NULL					( ( xSemaphoreHandle ) NULL )
#define SYS_DEFAULT_THREAD_STACK_DEPTH	configMINIMAL_STACK_SIZE

typedef xSemaphoreHandle sys_sem_t;
typedef xSemaphoreHandle sys_mutex_t;
typedef xQueueHandle sys_mbox_t;
typedef xTaskHandle sys_thread_t;
typedef int sys_prot_t;

#define sys_mbox_valid( x ) ( ( ( *x ) == NULL) ? pdFALSE : pdTRUE )
#define sys_mbox_set_invalid( x ) ( ( *x ) = NULL )
#define sys_sem_valid( x ) ( ( ( *x ) == NULL) ? pdFALSE : pdTRUE )
#define sys_sem_set_invalid( x ) ( ( *x ) = NULL )

#elif defined(CONFIG_OS_NUTTX)
#include <stdio.h>
#include <string.h>
#include <semaphore.h>
#include <mqueue.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/stat.h>

typedef struct {
    int valid;
    sem_t sem;
} sem_impl_t;

typedef struct {
    int valid;
    mqd_t mq;
    char mq_name[16];
} mbox_impl_t;

typedef mbox_impl_t		SYS_Id;
typedef unsigned int	SYS_Time;

typedef struct t_msg {
    struct t_msg *next;
} SYS_Message;


typedef void * xSemaphoreHandle;
typedef void * xQueueHandle;

#define SYS_MBOX_NULL					( ( xQueueHandle ) NULL )
#define SYS_SEM_NULL					( ( xSemaphoreHandle ) NULL )
#define SYS_DEFAULT_THREAD_STACK_DEPTH	configMINIMAL_STACK_SIZE

typedef sem_impl_t  sys_sem_t;
typedef sem_t       sys_mutex_t;
typedef mbox_impl_t sys_mbox_t;

#define sys_mbox_valid( x )       ( ( ( (x)->valid ) == 0) ? FALSE : TRUE )
#define sys_mbox_set_invalid( x ) ( ( (x)->valid ) = 0 )
#define sys_sem_valid( x )        ( ( ( (x)->valid ) == 0) ? FALSE : TRUE )
#define sys_sem_set_invalid( x )  ( ( (x)->valid ) = 0 )

typedef uint32_t sys_prot_t;

typedef pthread_t sys_thread_t;

#endif

#endif /* NO_SYS */

#endif /* __ARCH_SYS_ARCH_H__ */
