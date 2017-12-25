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

#if defined(CONFIG_OS_NUTTX)
#define DEBUG_PRINTF _info

#else
#include <debug/dbg_log.h>
DBG_DEFINE_MODULE(MQTT);
#undef DBG_MODULE
#define DBG_MODULE MQTT
#define DEBUG_PRINTF(...)     DBG_LOGF_SYNC_DEBUG(__VA_ARGS__)
#endif

#define MBEDTLS_CALLOC        calloc
#define MBEDTLS_FREE          free

#define	MQTT_MAX_SESSIONS 5

void MQTTSslInit(MQTTSocket* n)
{
  DEBUG_PRINTF("mbedTLS is not supported\n");

  n->my_socket = 0;

  n->mqttread = MQTTSslRead;
  n->mqttwrite = MQTTSslWrite;
  n->disconnect = NT_MQTTSocketDisconnect;

  return;
}

int MQTTSslConnect(MQTTSocket* n, char* hostname)
{
  DEBUG_PRINTF("mbedTLS is not supported\n");
  return 1;
}

void
MQTTSslDisconnect(MQTTSocket* n)
{
  DEBUG_PRINTF("mbedTLS is not supported\n");
  return;
}

int
MQTTSslRead(MQTTSocket* n, unsigned char *buf, int len, int timeout_ms)
{
  DEBUG_PRINTF("mbedTLS is not supported\n");
  return 0;
}

int MQTTSslWrite(MQTTSocket* n, unsigned char *buf, int len, int timeout_ms)
{
  DEBUG_PRINTF("mbedTLS is not supported\n");
  return 0;
}

