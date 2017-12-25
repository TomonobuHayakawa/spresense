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
 *******************************************************************************/

#if !defined(MQTTSocket_H)
#define MQTTSocket_H

#if defined(CONFIG_OS_NUTTX)
#include <pthread.h>
#include <mqueue.h>
#include <semaphore.h>
#include <time.h>
#include "lwip/netdb.h"
#else
#include <FreeRTOS.h>
#include "task.h"
#include "queue.h"
#include "semphr.h"
#endif

#include "socket.h"

#define USE_SSL 1

#ifdef USE_SSL
#include "netutils/mbedtls/config.h"
#include "netutils/mbedtls/ctr_drbg.h"
#include "netutils/mbedtls/entropy.h"
#include "netutils/mbedtls/net.h"
#include "netutils/mbedtls/platform.h"
#include "netutils/mbedtls/ssl.h"
#endif

typedef struct Timer Timer;

/**
 * definition of the Timer struct. Platform specific
 */

struct Timer {
#if defined(CONFIG_OS_NUTTX)
	struct timespec start_time;
	unsigned int TimeOut;
#else
	TickType_t xTicksToWait;
	TimeOut_t xTimeOut;
#endif
};

/**
 * \addtogroup MQTT
 * \{
 */

/**
 * \defgroup Mqtt_socket_if MQTT socket interface
 * MQTT socket interface
 * \{
 */
/**
 * \brief           MQTT socket strutcture
 */
typedef struct MQTTSocket  MQTTSocket ;
struct MQTTSocket 
{
	int my_socket;
#ifdef USE_SSL
	int use_ssl;
	mbedtls_ssl_context *mqtt_context;
	mbedtls_net_context mqtt_net_context;
	char *pRootCALocation;					/**< The Root CA file path in using mbedTLS (full file, not path) */
	char *pDeviceCertLocation;				/**< The device identity certificate file path in using mbedTLS (full file, not path) */
	char *pDevicePrivateKeyLocation;		/**< The device private key file in using mbedTLS (full file, not path) */
#endif
	int (*mqttread) (MQTTSocket *, unsigned char*, int, int);
	int (*mqttwrite) (MQTTSocket *, unsigned char*, int, int);
	void (*disconnect) (MQTTSocket *);
};


void TimerInit(Timer*);
char TimerIsExpired(Timer*);
void TimerCountdownMS(Timer*, unsigned int);
void TimerCountdown(Timer*, unsigned int);
int TimerLeftMS(Timer*);

typedef struct Mutex
{
	unsigned char sem;
} Mutex;

void MutexInit(Mutex*);
int MutexLock(Mutex*);
int MutexUnlock(Mutex*);

typedef struct Thread
{
#if defined(CONFIG_OS_NUTTX)
	char         dummy;
#else
	TaskHandle_t task;
#endif
} Thread;

int ThreadStart(const char *pcName, void( *pxThread )( void *pvParameters ), void *pvArg, int iStackSize, int iPriority);
int MQTTSocket_read(MQTTSocket*, unsigned char*, int, int);
int MQTTSocket_write(MQTTSocket*, unsigned char*, int, int);


/**
 * Initialize  MQTT socket
 * @param socket  MQTT socket
 * @param use_tls 0:not use mbedTLS / 1:use mbedTLS
 */
void NT_MQTTSocketInit(MQTTSocket *socket, int use_tls);

/**
 *  Connect MQTT socket 
 * @param socket  MQTT socket to be used
 * @param broker  MQTT broker address/hostname to be connected
 * @param port    port that MQTT broker uses
 */
int NT_MQTTSocketConnect(MQTTSocket *socket, char *broker, int port);

/**
 *  Connect MQTT socket 
 * @param socket  MQTT socket to be disconnected
 */
void NT_MQTTSocketDisconnect(MQTTSocket *socket);
/* \} */


/* \} */

void MQTTSslInit(MQTTSocket* n);
int MQTTSslConnect(MQTTSocket* n, char* hostname);
void MQTTSslDisconnect(MQTTSocket* n);
int MQTTSslWrite(MQTTSocket* n, unsigned char *buf, int len, int);
int MQTTSslRead(MQTTSocket* n, unsigned char *buf, int len, int);
#endif
