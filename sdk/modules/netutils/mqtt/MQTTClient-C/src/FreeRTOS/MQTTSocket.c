/*******************************************************************************
 * Copyright (c) 2014, 2015 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Allan Stockdill-Mander - initial API and implementation and/or initial documentation
 *    Ian Craggs - convert to FreeRTOS
 *******************************************************************************/

#include "MQTTSocket.h"

#if defined(CONFIG_OS_MERLOT)
int ThreadStart(const char *pcName, void( *pxThread )( void *pvParameters ), void *pvArg, int iStackSize, int iPriority)
{
	SYS_Task desc;
	SYS_CreateTaskParams params;

	params.arg = (void *)pvArg;
	params.function = pxThread;
	params.name = pcName;
	params.priority = iPriority;
	params.stackSize = iStackSize;
	SYS_CreateTask(&desc, &params);
	return 0;
}
#elif defined(CONFIG_OS_FREERTOS)
int ThreadStart(Thread* thread, void (*fn)(void*), void* arg)
{
	int rc = 0;
	uint16_t usTaskStackSize = (configMINIMAL_STACK_SIZE * 5);
	UBaseType_t uxTaskPriority = uxTaskPriorityGet(NULL); /* set the priority as the same as the calling task*/

	rc = xTaskCreate(fn,	/* The function that implements the task. */
		"MQTTTask",			/* Just a text name for the task to aid debugging. */
		usTaskStackSize,	/* The stack size is defined in FreeRTOSIPConfig.h. */
		arg,				/* The task parameter, not used in this case. */
		uxTaskPriority,		/* The priority assigned to the task is defined in FreeRTOSConfig.h. */
		&thread->task);		/* The task handle is not used. */

	return rc;

}
#elif defined(CONFIG_OS_NUTTX)
int ThreadStart(const char *pcName, void( *pxThread )( void *pvParameters ), void *pvArg, int iStackSize, int iPriority)
{
	pthread_attr_t attr;
	pthread_t      thread;
	int rc = 0;

	pthread_attr_init(&attr);

	attr.priority = iPriority;
	attr.stacksize = iStackSize;

	if( (rc=pthread_create( &thread, &attr, (pthread_startroutine_t)pxThread, pvArg )) == 0 )
	{
		pthread_detach(thread);
		return 0;
	}
	else
	{
		return rc;
	}
}
#endif


void MutexInit(Mutex* mutex)
{
#if defined(CONFIG_OS_MERLOT)
	SYS_CreateSemaphoreParams params;
	
	params.attribute = 0;
	params.initialCount = 1;
	params.maxCount = 1;
	
	mutex->sem = SYS_CreateSemaphore(0, &params);

#elif defined(CONFIG_OS_FREERTOS)
	mutex->sem = xSemaphoreCreateMutex();
#elif defined(CONFIG_OS_NUTTX)
	sem_init( (sem_t *)&(mutex->sem), 0, 1 );
#endif
}

int MutexLock(Mutex* mutex)
{
#if defined(CONFIG_OS_MERLOT)
	return SYS_WaitSemaphore(mutex->sem, TMO_FEVR);
#elif defined(CONFIG_OS_FREERTOS)
	return xSemaphoreTake(mutex->sem, portMAX_DELAY);
#elif defined(CONFIG_OS_NUTTX)
	return sem_wait((sem_t *)&(mutex->sem));
#endif
}

int MutexUnlock(Mutex* mutex)
{
#if defined(CONFIG_OS_MERLOT)
	return SYS_SignalSemaphore(mutex->sem);
#elif defined(CONFIG_OS_FREERTOS)
	return xSemaphoreGive(mutex->sem);
#elif defined(CONFIG_OS_NUTTX)
	return sem_destroy((sem_t *)&(mutex->sem));
#endif
}

void TimerCountdownMS(Timer* timer, unsigned int timeout_ms)
{
#if defined(CONFIG_OS_NUTTX)
	timer->TimeOut = timeout_ms;
	clock_gettime( CLOCK_MONOTONIC, &timer->start_time); /* Record the time at which this function was entered. */
#else
	timer->xTicksToWait = timeout_ms / portTICK_PERIOD_MS; /* convert milliseconds to ticks */
	vTaskSetTimeOutState(&timer->xTimeOut); /* Record the time at which this function was entered. */
#endif
}

void TimerCountdown(Timer* timer, unsigned int timeout) 
{
	TimerCountdownMS(timer, timeout * 1000);
}

int TimerLeftMS(Timer* timer) 
{
#if defined(CONFIG_OS_NUTTX)
	struct timespec current_time;
	struct timespec diff_time;
	int difftime_msec = 0;
	clock_gettime( CLOCK_MONOTONIC, &current_time);

	diff_time.tv_sec  = current_time.tv_sec - timer->start_time.tv_sec;
	diff_time.tv_nsec = current_time.tv_nsec - timer->start_time.tv_nsec;

	if(diff_time.tv_nsec < 0)
	{
		diff_time.tv_sec -= 1;
		diff_time.tv_nsec += 1000*1000*1000;
	}

	difftime_msec = diff_time.tv_sec*1000 + diff_time.tv_nsec/(1000*1000);

	if(difftime_msec < timer->TimeOut)
	{
		return timer->TimeOut - difftime_msec;
	}
	else
	{
		return 0;
	}
#else
	xTaskCheckForTimeOut(&timer->xTimeOut, &timer->xTicksToWait); /* updates xTicksToWait to the number left */
	return (timer->xTicksToWait * portTICK_PERIOD_MS);
#endif
}

char TimerIsExpired(Timer* timer)
{
#if defined(CONFIG_OS_NUTTX)
	if( TimerLeftMS(timer) )
	{
		return 0;
	}
	else
	{
		return 1;
	}
#else
	return xTaskCheckForTimeOut(&timer->xTimeOut, &timer->xTicksToWait) == pdTRUE;
#endif
}

void TimerInit(Timer* timer)
{
	memset(timer, 0, sizeof(Timer));
}

int MQTTSocket_read(MQTTSocket* n, unsigned char* buffer, int len, int timeout_ms)
{
#if defined(CONFIG_OS_NUTTX)
	Timer timer;
	struct timeval tv;
#else
	TickType_t xTicksToWait = timeout_ms / portTICK_PERIOD_MS; /* convert milliseconds to ticks */
	struct timeval tv;
	TimeOut_t xTimeOut;
#endif

	int recvLen = 0;

#if defined(CONFIG_OS_NUTTX)
	TimerInit(&timer);
	TimerCountdownMS( &timer, timeout_ms );
#else
	vTaskSetTimeOutState(&xTimeOut); /* Record the time at which this function was entered. */
#endif
	do
	{
		int rc = 0;

		tv.tv_sec = timeout_ms / 1000;  /* 1 second Timeout */
		tv.tv_usec = (timeout_ms % 1000) * 1000;  
		NT_SetSockOpt(n->my_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
		rc = NT_Recv(n->my_socket, buffer + recvLen, len - recvLen, 0);
		if (rc > 0)
			recvLen += rc;
		else if (rc < 0)
		{
			recvLen = rc;
			break;
		}
#if defined(CONFIG_OS_NUTTX)
	} while ((recvLen < len) && (TimerIsExpired(&timer) == 0));
#else
	} while ((recvLen < len) && (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) == pdFALSE));
#endif

	return recvLen;
}


int MQTTSocket_write(MQTTSocket* n, unsigned char* buffer, int len, int timeout_ms)
{
#if defined(CONFIG_OS_NUTTX)
	Timer timer;
	struct timeval tv;
#else
	TickType_t xTicksToWait = timeout_ms / portTICK_PERIOD_MS; /* convert milliseconds to ticks */
	struct timeval tv;
	TimeOut_t xTimeOut;
#endif

	int sentLen = 0;

#if defined(CONFIG_OS_NUTTX)
	TimerInit(&timer);
	TimerCountdownMS( &timer, timeout_ms );
#else
	vTaskSetTimeOutState(&xTimeOut); /* Record the time at which this function was entered. */
#endif
	do
	{
		int rc = 0;

		tv.tv_sec = timeout_ms / 1000;  /* 1 second Timeout */
		tv.tv_usec = (timeout_ms % 1000) * 1000;  

		NT_SetSockOpt(n->my_socket, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
		rc = NT_Send(n->my_socket, buffer + sentLen, len - sentLen, 0);
		if (rc > 0)
		{
			sentLen += rc;
		}
		else if (rc < 0)
		{
			sentLen = rc;
			break;
		}
#if defined(CONFIG_OS_NUTTX)
	} while ((sentLen < len) && (TimerIsExpired(&timer) == 0));
#else
	} while ((sentLen < len) && (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) == pdFALSE));
#endif

	return sentLen;
}


void NT_MQTTSocketDisconnect(MQTTSocket* n)
{
	NT_Close(n->my_socket);
}


void NT_MQTTSocketInit(MQTTSocket* n, int use_ssl)
{
	n->my_socket = 0;
	n->use_ssl = use_ssl;
	if(use_ssl == 0) {
		n->mqttread = MQTTSocket_read;
		n->mqttwrite = MQTTSocket_write;
		n->disconnect = NT_MQTTSocketDisconnect;
	} else {
		MQTTSslInit(n);
	}
}


int NT_MQTTSocketConnect(MQTTSocket* n, char* addr, int port)
{
int* sock = &n->my_socket;
	int type = SOCK_STREAM;
	struct sockaddr_in address;
#if defined(AF_INET6)
	struct sockaddr_in6 address6;
#endif
	int rc = -1;
#if defined(WIN32)
	short family;
#else
	int family = AF_INET;
#endif
	struct addrinfo *result = NULL;
	struct addrinfo hints = {0, AF_UNSPEC, SOCK_STREAM, IPPROTO_TCP, 0, NULL, NULL, NULL};
	static struct timeval tv;

	*sock = -1;
	if (addr[0] == '[')
	  ++addr;

	if ((rc = getaddrinfo(addr, NULL, &hints, &result)) == 0)
	{
		struct addrinfo* res = result;

		/* prefer ip4 addresses */
		while (res)
		{
			if (res->ai_family == AF_INET)
			{
				result = res;
				break;
			}
			res = res->ai_next;
		}

#if defined(AF_INET6)
		if (result->ai_family == AF_INET6)
		{
			address6.sin6_port = htons(port);
			address6.sin6_family = family = AF_INET6;
			address6.sin6_addr = ((struct sockaddr_in6*)(result->ai_addr))->sin6_addr;
		}
		else
#endif
		if (result->ai_family == AF_INET)
		{
			address.sin_port = htons(port);
			address.sin_family = family = AF_INET;
			address.sin_addr = ((struct sockaddr_in*)(result->ai_addr))->sin_addr;
		}
		else
			rc = -1;

		freeaddrinfo(result);
	}

	if (rc == 0)
	{
		*sock =	socket(family, type, 0);
		if (*sock != -1)
		{
#if defined(NOSIGPIPE)
			int opt = 1;

			if (setsockopt(*sock, SOL_SOCKET, SO_NOSIGPIPE, (void*)&opt, sizeof(opt)) != 0)
				Log(TRACE_MIN, -1, "Could not set SO_NOSIGPIPE for socket %d", *sock);
#endif
			if (n->use_ssl){
				if (family == AF_INET)
					rc = connect(*sock, (struct sockaddr*)&address, sizeof(address));
#if defined(AF_INET6)
				else
					rc = connect(*sock, (struct sockaddr*)&address6, sizeof(address6));

#endif
				if (rc == 0){
					n->mqtt_net_context.fd = *sock; 
					rc = MQTTSslConnect(n, addr);
					if (rc != 0) {
						NT_Close(*sock);
						*sock = -1;
					}
				}
				else {
					NT_Close(*sock);
					*sock = -1;
				}
				
			}
			else {
				if (family == AF_INET)
					rc = connect(*sock, (struct sockaddr*)&address, sizeof(address));
#if defined(AF_INET6)
				else
					rc = connect(*sock, (struct sockaddr*)&address6, sizeof(address6));
#endif
				if (rc != 0) {
					NT_Close(*sock);
					*sock = -1;
				}
			}
		}
		else {
			rc = -1;
		}
	}
	if (n->my_socket == -1 || rc == -1)
		return rc;

	tv.tv_sec = 1;  /* 1 second Timeout */
	tv.tv_usec = 0;  
	setsockopt(n->my_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
	return n->my_socket;
}

