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
/* lwIP includes. */

#include "net/lwip/opt.h"

#include "net/lwip/debug.h"
#include "net/lwip/def.h"
#include "lwip/sys.h"
#include "lwip/mem.h"

#include <stdio.h>
#include <string.h>

 #if NO_SYS==0

/* ------------------------ lwIP includes --------------------------------- */
#include "lwip/stats.h"

/*---------------------------------------------------------------------------*
 * Routine:  sys_mbox_new
 *---------------------------------------------------------------------------*
 * Description:
 *      Creates a new mailbox
 * Inputs:
 *      int size                -- Size of elements in the mailbox
 * Outputs:
 *      sys_mbox_t              -- Handle to new mailbox
 *---------------------------------------------------------------------------*/
err_t sys_mbox_new( sys_mbox_t *pxMailBox, int iSize )
{
	err_t xReturn = ERR_MEM;

#if defined(CONFIG_OS_MERLOT)
	SYS_CreateMailboxParams params;
	
	params.attribute = 0;

	*pxMailBox = SYS_CreateMailbox(0, &params);
	
	if( *pxMailBox > 0 )
#elif defined(CONFIG_OS_FREERTOS)
	*pxMailBox = xQueueCreate( iSize, sizeof( void * ) );
	if( *pxMailBox != NULL )
#elif defined(CONFIG_OS_NUTTX)
	mode_t mode = (S_IWOTH | S_IROTH | S_IWGRP | S_IRGRP | S_IWUSR | S_IRUSR);
	static u32_t suffix_num = 0;
	irqstate_t irqflags;

	memset(pxMailBox->mq_name, 0, sizeof(pxMailBox->mq_name));
	irqflags = enter_critical_section();
	snprintf(pxMailBox->mq_name, sizeof(pxMailBox->mq_name), "net_lwip%d", suffix_num);
	suffix_num++;
	leave_critical_section(irqflags);

	pxMailBox->mq = mq_open( pxMailBox->mq_name, O_RDWR | O_CREAT, mode, NULL );
	if( (pxMailBox->mq) != (mqd_t)ERROR )
	{
		pxMailBox->valid = 1;
	}
	else
	{
		pxMailBox->valid = 0;
	}

	if( (pxMailBox->mq) != (mqd_t)ERROR )
#endif
	{
		
		xReturn = ERR_OK;
		SYS_STATS_INC_USED( mbox );
	}
	return xReturn;

}


/*---------------------------------------------------------------------------*
 * Routine:  sys_mbox_free
 *---------------------------------------------------------------------------*
 * Description:
 *      Deallocates a mailbox. If there are messages still present in the
 *      mailbox when the mailbox is deallocated, it is an indication of a
 *      programming error in lwIP and the developer should be notified.
 * Inputs:
 *      sys_mbox_t mbox         -- Handle of mailbox
 * Outputs:
 *      sys_mbox_t              -- Handle to new mailbox
 *---------------------------------------------------------------------------*/
void sys_mbox_free( sys_mbox_t *pxMailBox )
{
#if defined(CONFIG_OS_MERLOT)
	SYS_DeleteMailbox(*pxMailBox);
#elif defined(CONFIG_OS_FREERTOS)
	unsigned long ulMessagesWaiting;

	ulMessagesWaiting = uxQueueMessagesWaiting( *pxMailBox );
	configASSERT( ( ulMessagesWaiting == 0 ) );

	#if SYS_STATS
	{
		if( ulMessagesWaiting != 0UL )
		{
			SYS_STATS_INC( mbox.err );
		}

		SYS_STATS_DEC( mbox.used );
	}
	#endif /* SYS_STATS */

	vQueueDelete( *pxMailBox );
#elif defined(CONFIG_OS_NUTTX)
	mq_close( pxMailBox->mq );
	mq_unlink( pxMailBox->mq_name );
	pxMailBox->valid = 0;
#endif

}

struct lwip_sms{
	SYS_Message msg;
	void *data;
};

/*---------------------------------------------------------------------------*
 * Routine:  sys_mbox_post
 *---------------------------------------------------------------------------*
 * Description:
 *      Post the "msg" to the mailbox.
 * Inputs:
 *      sys_mbox_t mbox         -- Handle of mailbox
 *      void *data              -- Pointer to data to post
 *---------------------------------------------------------------------------*/
void sys_mbox_post( sys_mbox_t *pxMailBox, void *pxMessageToPost )
{
#if defined(CONFIG_OS_MERLOT)
	struct lwip_sms *head = NULL;

	while ( head == NULL)
	{
		head = (struct lwip_sms *)memp_malloc(MEMP_MBOX_MSG);
		if ( head == NULL)
			SYS_SleepTask(10);
	}
	memset(head, 0, sizeof(struct lwip_sms));
	head->data = pxMessageToPost;

	while(SYS_SendMailbox(*pxMailBox, (SYS_Message *)head) != 0);
#elif defined(CONFIG_OS_FREERTOS)
	while( xQueueSendToBack( *pxMailBox, &pxMessageToPost, portMAX_DELAY ) != pdTRUE );
#elif defined(CONFIG_OS_NUTTX)
	u32_t addr = (u32_t)pxMessageToPost;
	int ret;

	while(1)
	{
		ret = mq_send( pxMailBox->mq, (char*)&addr, sizeof(addr), 0 );
		if (ret == 0)
		{
			break;
		}
	}
#endif
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_mbox_trypost
 *---------------------------------------------------------------------------*
 * Description:
 *      Try to post the "msg" to the mailbox.  Returns immediately with
 *      error if cannot.
 * Inputs:
 *      sys_mbox_t mbox         -- Handle of mailbox
 *      void *msg               -- Pointer to data to post
 * Outputs:
 *      err_t                   -- ERR_OK if message posted, else ERR_MEM
 *                                  if not.
 *---------------------------------------------------------------------------*/
err_t sys_mbox_trypost( sys_mbox_t *pxMailBox, void *pxMessageToPost )
{
	err_t xReturn;

#if defined(CONFIG_OS_MERLOT)
	struct lwip_sms *head;

	head = (struct lwip_sms *)memp_malloc(MEMP_MBOX_MSG);
	if ( head != NULL)
	{
		memset(head, 0, sizeof(struct lwip_sms));
		head->data = pxMessageToPost;
		if ( SYS_SendMailbox(*pxMailBox, (SYS_Message *)head) == 0)
		{
			xReturn = ERR_OK;
		}
		else
		{
			/* The queue was already full. */
			memp_free(MEMP_MBOX_MSG, head);

			xReturn = ERR_MEM;
			SYS_STATS_INC( mbox.err );
		}
	}
	else
	{
		xReturn = ERR_MEM;
		SYS_STATS_INC( mbox.err );
	}

#elif defined(CONFIG_OS_FREERTOS)
	if( xQueueSend( *pxMailBox, &pxMessageToPost, 0UL ) == pdPASS )
	{
		xReturn = ERR_OK;
	}
	else
	{
		/* The queue was already full. */
		xReturn = ERR_MEM;
		SYS_STATS_INC( mbox.err );
	}
#elif defined(CONFIG_OS_NUTTX)
	u32_t addr = (u32_t)pxMessageToPost;

	while(1)
	{
		if( mq_send( pxMailBox->mq, (char*)&addr, sizeof(addr), 0 ) == 0 )
		{
			xReturn = ERR_OK;
		}
		else
		{
			if(errno == EINTR)
			{
				continue;
			}
			xReturn = ERR_MEM;
		}
		break;
	}
#endif

	return xReturn;

}

/*---------------------------------------------------------------------------*
 * Routine:  sys_arch_mbox_fetch
 *---------------------------------------------------------------------------*
 * Description:
 *      Blocks the thread until a message arrives in the mailbox, but does
 *      not block the thread longer than "timeout" milliseconds (similar to
 *      the sys_arch_sem_wait() function). The "msg" argument is a result
 *      parameter that is set by the function (i.e., by doing "*msg =
 *      ptr"). The "msg" parameter maybe NULL to indicate that the message
 *      should be dropped.
 *
 *      The return values are the same as for the sys_arch_sem_wait() function:
 *      Number of milliseconds spent waiting or SYS_ARCH_TIMEOUT if there was a
 *      timeout.
 *
 *      Note that a function with a similar name, sys_mbox_fetch(), is
 *      implemented by lwIP.
 * Inputs:
 *      sys_mbox_t mbox         -- Handle of mailbox
 *      void **msg              -- Pointer to pointer to msg received
 *      u32_t timeout           -- Number of milliseconds until timeout
 * Outputs:
 *      u32_t                   -- SYS_ARCH_TIMEOUT if timeout, else number
 *                                  of milliseconds until received.
 *---------------------------------------------------------------------------*/
u32_t sys_arch_mbox_fetch( sys_mbox_t *pxMailBox, void **ppvBuffer, u32_t ulTimeOut )
{
	portTickType xElapsed;
	unsigned long ulReturn = 0;

#if defined(CONFIG_OS_MERLOT)
	portTickType xStartTime, xEndTime;

	struct lwip_sms *head = NULL;

	xStartTime = xTaskGetTickCount();	

	if( ulTimeOut != 0UL )
	{
		xEndTime = xStartTime;
		*ppvBuffer = NULL;

		while((xEndTime - xStartTime) < ulTimeOut/portTICK_RATE_MS )
		{
			if(SYS_ReceiveMailbox(*pxMailBox, (SYS_Message **)&head, TMO_POL) == 0)
			{
				*ppvBuffer = head->data;
				memp_free(MEMP_MBOX_MSG, head);

				xEndTime = xTaskGetTickCount();
				xElapsed = ( xEndTime - xStartTime ) * portTICK_RATE_MS;
				ulReturn = xElapsed;
				break;
			}
			else
			{
	            SYS_SleepTask(10);
				ulReturn = SYS_ARCH_TIMEOUT;
			}
			xEndTime = xTaskGetTickCount();
		}
	}
	else
	{
		while(SYS_ReceiveMailbox(*pxMailBox, (SYS_Message **)&head, TMO_FEVR) != 0);

		*ppvBuffer = head->data;
		memp_free(MEMP_MBOX_MSG, head);

		xEndTime = xTaskGetTickCount();
		xElapsed = ( xEndTime - xStartTime ) * portTICK_RATE_MS;

		if( xElapsed == 0UL )
		{
			xElapsed = 1UL;
		}

		ulReturn = xElapsed;
	}	
	
#elif defined(CONFIG_OS_FREERTOS)
	portTickType xStartTime, xEndTime;
	void *pvDummy;
	xStartTime = xTaskGetTickCount();

	if( NULL == ppvBuffer )
	{
		ppvBuffer = &pvDummy;
	}

	if( ulTimeOut != 0UL )
	{
		if( pdTRUE == xQueueReceive( *pxMailBox, &( *ppvBuffer ), ulTimeOut/ portTICK_RATE_MS ) )
		{
			xEndTime = xTaskGetTickCount();
			xElapsed = ( xEndTime - xStartTime ) * portTICK_RATE_MS;

			ulReturn = xElapsed;
		}
		else
		{
			/* Timed out. */
			*ppvBuffer = NULL;
			ulReturn = SYS_ARCH_TIMEOUT;
		}
	}
	else
	{
		while( pdTRUE != xQueueReceive( *pxMailBox, &( *ppvBuffer ), portMAX_DELAY ) );
		xEndTime = xTaskGetTickCount();
		xElapsed = ( xEndTime - xStartTime ) * portTICK_RATE_MS;

		if( xElapsed == 0UL )
		{
			xElapsed = 1UL;
		}

		ulReturn = xElapsed;
	}

#elif defined(CONFIG_OS_NUTTX)
	u32_t addr[8];
	struct timespec curr_time, end_time;
	clock_gettime( CLOCK_REALTIME , &curr_time );
	if( ulTimeOut != 0UL )
	{
		struct timespec to_time;
		to_time.tv_sec  = curr_time.tv_sec + (ulTimeOut/1000);
		to_time.tv_nsec = curr_time.tv_nsec + ((ulTimeOut%1000)*1000*1000);
		if( to_time.tv_nsec >= 1000*1000*1000 )
		{
			to_time.tv_sec += 1;
			to_time.tv_nsec -= 1000*1000*1000;
		}

		while(1)
		{
			if(mq_timedreceive( pxMailBox->mq, (char*)&addr[0], sizeof(addr), NULL, &to_time) > 0)
			{
				clock_gettime( CLOCK_REALTIME, &end_time );
				if( end_time.tv_nsec - curr_time.tv_nsec < 0)
				{
					xElapsed = (end_time.tv_sec - curr_time.tv_sec - 1)*1000
	    	                   + ( end_time.tv_nsec - curr_time.tv_nsec + 1000*1000*1000 )/(1000*1000);

				}
				else
				{
					xElapsed = (end_time.tv_sec - curr_time.tv_sec)*1000
	    	                   + ( end_time.tv_nsec - curr_time.tv_nsec )/(1000*1000);
				}
				*ppvBuffer = (void*)addr[0];
				ulReturn = xElapsed;
			}
			else
			{
				if (errno == EINTR)
				{
					continue;
				}
				*ppvBuffer = NULL;
				ulReturn = SYS_ARCH_TIMEOUT;
			}
			break;
		}
	}
	else
	{
		ssize_t size;

		while(1)
		{
			size = mq_receive( pxMailBox->mq, (char*)&addr[0], sizeof(addr), NULL);
			if (size > 0)
			{
				break;
			}
		}

		clock_gettime( CLOCK_REALTIME, &end_time );
		if( end_time.tv_nsec - curr_time.tv_nsec < 0)
		{
			xElapsed = (end_time.tv_sec - curr_time.tv_sec - 1)*1000
   	                   + ( end_time.tv_nsec - curr_time.tv_nsec + 1000*1000*1000 )/(1000*1000);
		}
		else
		{
			xElapsed = (end_time.tv_sec - curr_time.tv_sec)*1000
   	                   + ( end_time.tv_nsec - curr_time.tv_nsec )/(1000*1000);
		}

		*ppvBuffer = (void*)addr[0];
		ulReturn = xElapsed;
	}
#endif
	return ulReturn;	

}

/*---------------------------------------------------------------------------*
 * Routine:  sys_arch_mbox_tryfetch
 *---------------------------------------------------------------------------*
 * Description:
 *      Similar to sys_arch_mbox_fetch, but if message is not ready
 *      immediately, we'll return with SYS_MBOX_EMPTY.  On success, 0 is
 *      returned.
 * Inputs:
 *      sys_mbox_t mbox         -- Handle of mailbox
 *      void **msg              -- Pointer to pointer to msg received
 * Outputs:
 *      u32_t                   -- SYS_MBOX_EMPTY if no messages.  Otherwise,
 *                                  return ERR_OK.
 *---------------------------------------------------------------------------*/
u32_t sys_arch_mbox_tryfetch( sys_mbox_t *pxMailBox, void **ppvBuffer )
{
	unsigned long ulReturn;
#if defined(CONFIG_OS_MERLOT)
	struct lwip_sms *head = NULL;

	if(SYS_ReceiveMailbox( *pxMailBox, (SYS_Message **)&head, TMO_POL) == 0)
	{
		*ppvBuffer = head->data;
		memp_free(MEMP_MBOX_MSG, head);
		ulReturn = ERR_OK;
	}
#elif defined(CONFIG_OS_FREERTOS)
	void *pvDummy;

	if( ppvBuffer== NULL )
	{
		ppvBuffer = &pvDummy;
	}

	if( pdTRUE == xQueueReceive( *pxMailBox,   &( *ppvBuffer ), 0UL ) )
	{
		ulReturn = ERR_OK;
	}
#elif defined(CONFIG_OS_NUTTX)
	u32_t addr[8];
	struct timespec curr_time;
	clock_gettime( CLOCK_REALTIME , &curr_time );

	if( mq_timedreceive( pxMailBox->mq,  (char*)&addr[0] , sizeof(addr), NULL, &curr_time ) > 0 )
	{
		*ppvBuffer = (void*)addr[0];
		ulReturn = ERR_OK;
	}
#endif
	else
	{
		ulReturn = SYS_MBOX_EMPTY;
	}

	return ulReturn;
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_sem_new
 *---------------------------------------------------------------------------*
 * Description:
 *      Creates and returns a new semaphore. The "ucCount" argument specifies
 *      the initial state of the semaphore.
 *      NOTE: Currently this routine only creates counts of 1 or 0
 * Inputs:
 *      sys_mbox_t mbox         -- Handle of mailbox
 *      u8_t ucCount              -- Initial ucCount of semaphore (1 or 0)
 * Outputs:
 *      sys_sem_t               -- Created semaphore or 0 if could not create.
 *---------------------------------------------------------------------------*/
err_t sys_sem_new( sys_sem_t *pxSemaphore, u8_t ucCount )
{
	err_t xReturn = ERR_MEM;

#if defined(CONFIG_OS_MERLOT)
	SYS_CreateSemaphoreParams params;
	
	params.attribute = 0;
	params.initialCount = ucCount;
	params.maxCount = 1;
	
	*pxSemaphore = SYS_CreateSemaphore(0, &params);
	if( *pxSemaphore > 0 )
	{
		xReturn = ERR_OK;
		SYS_STATS_INC_USED( sem );
	}
	else
	{
		SYS_STATS_INC( sem.err );
	}
#elif defined(CONFIG_OS_FREERTOS)

	vSemaphoreCreateBinary( ( *pxSemaphore ) );

	if( *pxSemaphore != NULL )
	{
		if( ucCount == 0U )
		{
			xSemaphoreTake( *pxSemaphore, 1UL );
		}

		xReturn = ERR_OK;
		SYS_STATS_INC_USED( sem );
	}
	else
	{
		SYS_STATS_INC( sem.err );
	}
#elif defined(CONFIG_OS_NUTTX)
	if( sem_init( &(pxSemaphore->sem), 0, ucCount ) == 0 )
	{
		pxSemaphore->valid = 1;
		xReturn = ERR_OK;
	}
	else
	{
		pxSemaphore->valid = 0;
	}
#endif
	return xReturn;
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_arch_sem_wait
 *---------------------------------------------------------------------------*
 * Description:
 *      Blocks the thread while waiting for the semaphore to be
 *      signaled. If the "timeout" argument is non-zero, the thread should
 *      only be blocked for the specified time (measured in
 *      milliseconds).
 *
 *      If the timeout argument is non-zero, the return value is the number of
 *      milliseconds spent waiting for the semaphore to be signaled. If the
 *      semaphore wasn't signaled within the specified time, the return value is
 *      SYS_ARCH_TIMEOUT. If the thread didn't have to wait for the semaphore
 *      (i.e., it was already signaled), the function may return zero.
 *
 *      Notice that lwIP implements a function with a similar name,
 *      sys_sem_wait(), that uses the sys_arch_sem_wait() function.
 * Inputs:
 *      sys_sem_t sem           -- Semaphore to wait on
 *      u32_t timeout           -- Number of milliseconds until timeout
 * Outputs:
 *      u32_t                   -- Time elapsed or SYS_ARCH_TIMEOUT.
 *---------------------------------------------------------------------------*/
u32_t sys_arch_sem_wait( sys_sem_t *pxSemaphore, u32_t ulTimeout )
{
	portTickType xElapsed;
	unsigned long ulReturn;

#if defined(CONFIG_OS_MERLOT)
	portTickType xStartTime, xEndTime;
	xStartTime = xTaskGetTickCount();
	if( ulTimeout != 0UL )
	{
		if(SYS_WaitSemaphore( *pxSemaphore, ulTimeout / portTICK_RATE_MS ) == 0)
		{
			xEndTime = xTaskGetTickCount();
			xElapsed = ( xEndTime - xStartTime ) * portTICK_RATE_MS;
			ulReturn = xElapsed;
		}
		else
		{
			/* Timed out. */
			ulReturn = SYS_ARCH_TIMEOUT;
		}
	}
	else
	{
		SYS_WaitSemaphore( *pxSemaphore,  TMO_FEVR);

		xEndTime = xTaskGetTickCount();
		xElapsed = ( xEndTime - xStartTime ) * portTICK_RATE_MS;

		if( xElapsed == 0UL )
		{
			xElapsed = 1UL;
		}

		ulReturn = xElapsed;
	}

#elif defined(CONFIG_OS_FREERTOS)
	portTickType xStartTime, xEndTime;
	xStartTime = xTaskGetTickCount();

	if( ulTimeout != 0UL )
	{
		if( xSemaphoreTake( *pxSemaphore, ulTimeout / portTICK_RATE_MS ) == pdTRUE )
		{
			xEndTime = xTaskGetTickCount();
			xElapsed = (xEndTime - xStartTime) * portTICK_RATE_MS;
			ulReturn = xElapsed;
		}
		else
		{
			ulReturn = SYS_ARCH_TIMEOUT;
		}
	}
	else
	{
		while( xSemaphoreTake( *pxSemaphore, portMAX_DELAY ) != pdTRUE );
		xEndTime = xTaskGetTickCount();
		xElapsed = ( xEndTime - xStartTime ) * portTICK_RATE_MS;

		if( xElapsed == 0UL )
		{
			xElapsed = 1UL;
		}

		ulReturn = xElapsed;
	}

#elif defined(CONFIG_OS_NUTTX)
	struct timespec curr_time, end_time;
	clock_gettime( CLOCK_REALTIME , &curr_time );

	if( ulTimeout != 0UL )
	{
		struct timespec to_time;
		to_time.tv_sec  = curr_time.tv_sec + (ulTimeout/1000);
		to_time.tv_nsec = curr_time.tv_nsec + ((ulTimeout%1000)*1000*1000);
		if( to_time.tv_nsec >= 1000*1000*1000 )
		{
			to_time.tv_sec += 1;
			to_time.tv_nsec -= 1000*1000*1000;
		}
		
		while(1)
		{
			if( sem_timedwait( &(pxSemaphore->sem), &to_time ) == 0 )
			{
				clock_gettime( CLOCK_REALTIME, &end_time );
				if( end_time.tv_nsec - curr_time.tv_nsec < 0)
				{
					xElapsed = (end_time.tv_sec - curr_time.tv_sec - 1)*1000
	    	                   + ( end_time.tv_nsec - curr_time.tv_nsec + 1000*1000*1000 )/(1000*1000);

				}
				else
				{
					xElapsed = (end_time.tv_sec - curr_time.tv_sec)*1000
	    	                   + ( end_time.tv_nsec - curr_time.tv_nsec )/(1000*1000);
				}

				ulReturn = xElapsed;
			}
			else
			{
				if (errno == EINTR)
				{
					continue;
				}
				ulReturn = SYS_ARCH_TIMEOUT;
			}
			break;
		}
	}
	else
	{
		while( sem_wait( &(pxSemaphore->sem) ) != 0 );
		clock_gettime( CLOCK_REALTIME, &end_time );
		if( end_time.tv_nsec - curr_time.tv_nsec < 0)
		{
			xElapsed = (end_time.tv_sec - curr_time.tv_sec - 1)*1000
   	                   + ( end_time.tv_nsec - curr_time.tv_nsec + 1000*1000*1000 )/(1000*1000);
		}
		else
		{
			xElapsed = (end_time.tv_sec - curr_time.tv_sec)*1000
   	                   + ( end_time.tv_nsec - curr_time.tv_nsec )/(1000*1000);
		}

		if( xElapsed == 0UL )
		{
			xElapsed = 1UL;
		}

		ulReturn = xElapsed;
	}
#endif
	return ulReturn;
}

/**
 * @brief	Create a new mutex
 * @param	pxMutex pointer to the mutex to create
 * @return	a new mutex
 */
err_t sys_mutex_new( sys_mutex_t *pxMutex )
{
	err_t xReturn = ERR_MEM;
#if defined(CONFIG_OS_MERLOT)
	SYS_CreateSemaphoreParams params;
	
	params.attribute = 0;
	params.initialCount = 1;
	params.maxCount = 1;
	
	*pxMutex = SYS_CreateSemaphore(0, &params);

	if( *pxMutex > 0 )
#elif defined(CONFIG_OS_FREERTOS)
	*pxMutex = xSemaphoreCreateMutex();

	if( *pxMutex != NULL )
#elif defined(CONFIG_OS_NUTTX)
	if( sem_init( pxMutex, 0, 1 ) == 0 )
#endif
	{
		xReturn = ERR_OK;
		SYS_STATS_INC_USED( mutex );
	}
	else
	{
		SYS_STATS_INC( mutex.err );
	}

	return xReturn;
}

/** Lock a mutex
 * @param pxMutex the mutex to lock */
void sys_mutex_lock( sys_mutex_t *pxMutex )
{
#if defined(CONFIG_OS_MERLOT)
	SYS_WaitSemaphore(*pxMutex, TMO_FEVR);
#elif defined(CONFIG_OS_FREERTOS)
	while( xSemaphoreTake( *pxMutex, portMAX_DELAY ) != pdPASS );
#elif defined(CONFIG_OS_NUTTX)
	while(sem_wait( pxMutex ) != 0);
#endif

}

/** Unlock a mutex
 * @param pxMutex the mutex to unlock */
void sys_mutex_unlock(sys_mutex_t *pxMutex )
{
#if defined(CONFIG_OS_MERLOT)
	SYS_SignalSemaphore(*pxMutex);
#elif defined(CONFIG_OS_FREERTOS)
	xSemaphoreGive( *pxMutex );
#elif defined(CONFIG_OS_NUTTX)
	sem_post( pxMutex );
#endif
}


/** Delete a semaphore
 * @param pxMutex the mutex to delete */
void sys_mutex_free( sys_mutex_t *pxMutex )
{
	SYS_STATS_DEC( mutex.used );
#if defined(CONFIG_OS_MERLOT)
	SYS_DeleteSemaphore(*pxMutex);
#elif defined(CONFIG_OS_FREERTOS)
	vQueueDelete( *pxMutex );
#elif defined(CONFIG_OS_NUTTX)
	sem_destroy( pxMutex );
#endif
}


/*---------------------------------------------------------------------------*
 * Routine:  sys_sem_signal
 *---------------------------------------------------------------------------*
 * Description:
 *      Signals (releases) a semaphore
 * Inputs:
 *      sys_sem_t sem           -- Semaphore to signal
 *---------------------------------------------------------------------------*/
void sys_sem_signal( sys_sem_t *pxSemaphore )
{
#if defined(CONFIG_OS_MERLOT)
	SYS_SignalSemaphore(*pxSemaphore);
#elif defined(CONFIG_OS_FREERTOS)
	xSemaphoreGive( *pxSemaphore );
#elif defined(CONFIG_OS_NUTTX)
	sem_post( &(pxSemaphore->sem) );
#endif
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_sem_free
 *---------------------------------------------------------------------------*
 * Description:
 *      Deallocates a semaphore
 * Inputs:
 *      sys_sem_t sem           -- Semaphore to free
 *---------------------------------------------------------------------------*/
void sys_sem_free( sys_sem_t *pxSemaphore )
{
	SYS_STATS_DEC(sem.used);
#if defined(CONFIG_OS_MERLOT)
	SYS_DeleteSemaphore(*pxSemaphore);
#elif defined(CONFIG_OS_FREERTOS)
	vQueueDelete( *pxSemaphore );
#elif defined(CONFIG_OS_NUTTX)
	sem_destroy( &pxSemaphore->sem );
	pxSemaphore->valid = 0;
#endif
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_init
 *---------------------------------------------------------------------------*
 * Description:
 *      Initialize sys arch
 *---------------------------------------------------------------------------*/

void sys_init(void)
{
}

u32_t sys_now(void)
{
#if defined(CONFIG_OS_NUTTX)
	struct timespec tp;
	clock_gettime( CLOCK_MONOTONIC, &tp  );
	return (tp.tv_sec*1000)+(tp.tv_nsec/(1000*1000));
#else
	SYS_Time time = 0;
	SYS_GetTime(&time);
	return time;
#endif
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_thread_new
 *---------------------------------------------------------------------------*
 * Description:
 *      Starts a new thread with priority "prio" that will begin its
 *      execution in the function "thread()". The "arg" argument will be
 *      passed as an argument to the thread() function. The id of the new
 *      thread is returned. Both the id and the priority are system
 *      dependent.
 * Inputs:
 *      char *name              -- Name of thread
 *      void (* thread)(void *arg) -- Pointer to function to run.
 *      void *arg               -- Argument passed into function
 *      int stacksize           -- Required stack amount in bytes
 *      int prio                -- Thread priority
 * Outputs:
 *      sys_thread_t            -- Pointer to per-thread timeouts.
 *---------------------------------------------------------------------------*/
sys_thread_t sys_thread_new( const char *pcName, void( *pxThread )( void *pvParameters ), void *pvArg, int iStackSize, int iPriority )
{
	sys_thread_t xReturn;
#if defined(CONFIG_OS_MERLOT)
	SYS_Task desc;
	SYS_CreateTaskParams params;

	params.arg = (void *)pvArg;
	params.function = pxThread;
	params.name = pcName;
	params.priority = iPriority;
	params.stackSize = iStackSize;

	int ercd = SYS_CreateTask(&desc, &params);
	if( ercd == 0 )
	{
		xReturn = (sys_thread_t)desc;

	}
	else
	{
		xReturn = NULL;
	}
	
#elif defined(CONFIG_OS_FREERTOS)
	xTaskHandle xCreatedTask;
	portBASE_TYPE xResult;

	xResult = xTaskCreate( pxThread, ( signed char * ) pcName, iStackSize, pvArg, iPriority, &xCreatedTask );

	if( xResult == pdPASS )
	{
		xReturn = xCreatedTask;
	}
	else
	{
		xReturn = NULL;
	}

#elif defined(CONFIG_OS_NUTTX)
	pthread_t thread_id;
	pthread_attr_t attr;
	struct sched_param param;
	sigset_t new_set;
	sigset_t old_set;

	/* set signal mask */
	sigfillset(&new_set);
	pthread_sigmask(SIG_SETMASK, &new_set, &old_set);

	pthread_attr_init(&attr);
	param.sched_priority = iPriority;
	pthread_attr_setschedparam(&attr, &param);
	pthread_attr_setstacksize(&attr, iStackSize);

	if( pthread_create( &thread_id, &attr, (pthread_startroutine_t)pxThread, pvArg ) == 0 )
	{
		pthread_attr_destroy(&attr);

		xReturn = thread_id;
	}
	else
	{
		xReturn = 0;
	}

	/* reset signal mask */
	pthread_sigmask(SIG_SETMASK, &old_set, NULL);
#endif
	return xReturn;
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_thread_delete
 *---------------------------------------------------------------------------*
 * Description:
 *      Delete thread.
 * Inputs:
 *      sys_thread_t            -- thread identifier
 * Outputs:
 *      result                  -- result of deletion.
 *---------------------------------------------------------------------------*/
int sys_thread_delete( sys_thread_t thread_t )
{
#if defined(CONFIG_OS_MERLOT)
	return SYS_DeleteTask( thread_t );
#elif defined(CONFIG_OS_NUTTX)
	pthread_cancel( thread_t );
	return pthread_join( thread_t, NULL );
#else
	return 0;
#endif
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_thread_join
 *---------------------------------------------------------------------------*
 * Description:
 *       Await termination of another thread.
 *---------------------------------------------------------------------------*/
s32_t sys_thread_join( sys_thread_t thread_t, void** thread_return )
{
	s32_t ret = 0;
#if defined(CONFIG_OS_MERLOT)

#elif defined(CONFIG_OS_FREERTOS)

#elif defined(CONFIG_OS_NUTTX)
	ret = pthread_join(thread_t, thread_return);
#endif
	return ret;
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_thread_exit
 *---------------------------------------------------------------------------*
 * Description:
 *      Exit self thread.
 *---------------------------------------------------------------------------*/
void sys_thread_exit( void )
{
#if defined(CONFIG_OS_MERLOT)

#elif defined(CONFIG_OS_FREERTOS)

#elif defined(CONFIG_OS_NUTTX)

	pthread_exit(0);
#endif
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_arch_protect
 *---------------------------------------------------------------------------*
 * Description:
 *      This optional function does a "fast" critical region protection and
 *      returns the previous protection level. This function is only called
 *      during very short critical regions. An embedded system which supports
 *      ISR-based drivers might want to implement this function by disabling
 *      interrupts. Task-based systems might want to implement this by using
 *      a mutex or disabling tasking. This function should support recursive
 *      calls from the same task or interrupt. In other words,
 *      sys_arch_protect() could be called while already protected. In
 *      that case the return value indicates that it is already protected.
 *
 *      sys_arch_protect() is only required if your port is supporting an
 *      operating system.
 * Outputs:
 *      sys_prot_t              -- Previous protection level (not used here)
 *---------------------------------------------------------------------------*/

sys_prot_t sys_arch_protect( void )
{
#if defined(CONFIG_OS_NUTTX)
	sched_lock();
#else
	vPortEnterCritical();
#endif
	return ( sys_prot_t ) 1;
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_arch_unprotect
 *---------------------------------------------------------------------------*
 * Description:
 *      This optional function does a "fast" set of critical region
 *      protection to the value specified by pval. See the documentation for
 *      sys_arch_protect() for more information. This function is only
 *      required if your port is supporting an operating system.
 * Inputs:
 *      sys_prot_t              -- Previous protection level (not used here)
 *---------------------------------------------------------------------------*/
void sys_arch_unprotect( sys_prot_t xValue )
{
	(void) xValue;
#if defined(CONFIG_OS_NUTTX)
	sched_unlock();
#else
	taskEXIT_CRITICAL();
#endif
}

/*
 * Prints an assertion messages and aborts execution.
 */
void sys_assert( const char *pcMessage )
{
	(void) pcMessage;

	for (;;)
	{
	}
}

/*---------------------------------------------------------------------------*
 * Routine:  sys_jiffies
 *---------------------------------------------------------------------------*
 * Description:
 *      Used by PPP as a timestamp-ish value
 *---------------------------------------------------------------------------*/
u32_t sys_jiffies(void) {
    static u32_t jiffies = 0;
	int ercd;
#if defined(CONFIG_OS_NUTTX)
	u32_t time;
	struct timespec tp;
	ercd = clock_gettime( CLOCK_MONOTONIC, &tp  );
	time = (tp.tv_sec*1000)+(tp.tv_nsec/(1000*1000));
	if(ercd != 0)
#else
	SYS_Time time;
	ercd = SYS_GetTime(&time);
	if(ercd < 0)
#endif
		return -1;
	else
	    jiffies += 1 + (time/10000);
    return jiffies;
}

/*-------------------------------------------------------------------------*
 * End of File:  sys_arch.c
 *-------------------------------------------------------------------------*/

#endif

/*-----------------------------------------------------------------------------------*/
