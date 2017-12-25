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

	
/* Static TLS data that is common to all TLS connections */
static mbedtls_entropy_context g_entropy;
static mbedtls_ctr_drbg_context g_ctr_drbg;
static mbedtls_ssl_config g_ssl_conf;
static mbedtls_x509_crt g_ssl_ca;
static mbedtls_x509_crt clicert;
static mbedtls_pk_context pkey;

void MQTTSslInit(MQTTSocket* n)
{
  static const char *pers = "mqtt_test";
  int r;

  n->my_socket = 0;

  n->mqttread = MQTTSslRead;
  n->mqttwrite = MQTTSslWrite;
  n->disconnect = NT_MQTTSocketDisconnect;

  // Initialize state variables
  mbedtls_ctr_drbg_init(&g_ctr_drbg);
  mbedtls_ssl_config_init(&g_ssl_conf);
  mbedtls_entropy_init(&g_entropy);
  mbedtls_x509_crt_init(&g_ssl_ca);

  // Set up for random bits generation
  if ((r = mbedtls_ctr_drbg_seed(&g_ctr_drbg, &g_entropy,
                                 (const unsigned char *)pers, strlen(pers))) != 0) {
    goto exit;
  }
  mbedtls_ssl_conf_rng(&g_ssl_conf, &g_ctr_drbg);

  // General configuration
  if ((r = mbedtls_ssl_config_defaults(&g_ssl_conf, MBEDTLS_SSL_IS_CLIENT,
                                       MBEDTLS_SSL_TRANSPORT_STREAM,
                                       MBEDTLS_SSL_PRESET_DEFAULT)) != 0) {
    goto exit;
  }

#if 0
  // Install the root certificate(s), and set up for server authentication
  size_t num_certs = rootca_number_of_certificates();
  for (size_t i = 0; i < num_certs; i++) {
    if (mbedtls_x509_crt_parse_der(&g_ssl_ca, rootca_get_certificate(i), rootca_get_certificate_size(i)) != 0) {
      goto exit;
    }
  }
#endif
  r = mbedtls_x509_crt_parse_file(&g_ssl_ca, n->pRootCALocation);
  if (r < 0) {
    goto exit;
  }

  r = mbedtls_x509_crt_parse_file(&clicert, n->pDeviceCertLocation);
  if (r == 0) {
    r = mbedtls_pk_parse_keyfile(&pkey, n->pDevicePrivateKeyLocation, "");
    if (r != 0) {
      goto exit;
    } 
  }


  mbedtls_ssl_conf_ca_chain(&g_ssl_conf, &g_ssl_ca, NULL);
  mbedtls_ssl_conf_authmode(&g_ssl_conf, MBEDTLS_SSL_VERIFY_REQUIRED);

#if 0
  for (size_t i = 0; i < MQTT_MAX_SESSIONS; i++) {
    g_mqtt_sessions[i].session = NULL;
  }
#endif
  return;

exit:
  mbedtls_ssl_config_free(&g_ssl_conf);
  mbedtls_ctr_drbg_free(&g_ctr_drbg);
  mbedtls_entropy_free(&g_entropy);

}

int MQTTSslConnect(MQTTSocket* n, char* hostname)
{
  int ret;

  if (n == NULL)
    return -1;

  mbedtls_ssl_context *mqtt_context = MBEDTLS_CALLOC(1, sizeof(mbedtls_ssl_context));
  n->mqtt_context = mqtt_context;

  mbedtls_ssl_init(mqtt_context);

  mbedtls_ssl_conf_own_cert(&g_ssl_conf, &clicert, &pkey);
  if (mbedtls_ssl_setup(mqtt_context, &g_ssl_conf) != 0) {
    MBEDTLS_FREE(n->mqtt_context);
    n->mqtt_context = NULL;
    return -1;
  }

  if (hostname != NULL)
    mbedtls_ssl_set_hostname(mqtt_context, hostname);
  mbedtls_ssl_set_bio(mqtt_context, &n->mqtt_net_context.fd);

#if !defined(CONFIG_OS_NUTTX)
  uint32_t tt1 = portTICK_PERIOD_MS * xTaskGetTickCount();
#endif
  if ((ret = mbedtls_ssl_handshake(mqtt_context)) != 0) {
    DEBUG_PRINTF("TLS handshake failed\n");
  mbedtls_printf( " failed\n  ! mbedtls_ssl_handshake returned -0x%x\n\n", -ret );
    MBEDTLS_FREE(n->mqtt_context);
    n->mqtt_context = NULL;
    return -1;
  }
#if !defined(CONFIG_OS_NUTTX)
  uint32_t tt2 = portTICK_PERIOD_MS * xTaskGetTickCount();
  DEBUG_PRINTF("Time elapsed = %u msecs\n", tt2 - tt1);
#endif
  DEBUG_PRINTF("TLS handshake succeeded\n");

  return 0;
}

void
MQTTSslDisconnect(MQTTSocket* n)
{
  if (n == NULL)
    return ;

  if (n->mqtt_context != NULL) {
    mbedtls_ssl_free(n->mqtt_context);
    MBEDTLS_FREE(n->mqtt_context);
    n->mqtt_context = NULL;
  }

  NT_Close(n->my_socket);
  n->my_socket = -1;
}

int
MQTTSslRead(MQTTSocket* n, unsigned char *buf, int len, int timeout_ms)
{
#if defined(CONFIG_OS_NUTTX)
	Timer timer;
#else
  TickType_t xTicksToWait = timeout_ms / portTICK_PERIOD_MS; /* convert milliseconds to ticks */
  TimeOut_t xTimeOut;
#endif

  int recvLen = 0;

  if (n == NULL)
    return -1;

#if defined(CONFIG_OS_NUTTX)
	TimerInit(&timer);
	TimerCountdownMS( &timer,  timeout_ms );
#else
  vTaskSetTimeOutState(&xTimeOut); /* Record the time at which this function was entered. */
#endif
  do
  {
    int rc = 0;

    mbedtls_ssl_conf_read_timeout(&g_ssl_conf, timeout_ms);
    rc = mbedtls_ssl_read(n->mqtt_context, (unsigned char *)buf, len);
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

int MQTTSslWrite(MQTTSocket* n, unsigned char *buf, int len, int timeout_ms)
{
#if defined(CONFIG_OS_NUTTX)
	Timer timer;
#else
  TickType_t xTicksToWait = timeout_ms / portTICK_PERIOD_MS; /* convert milliseconds to ticks */
  TimeOut_t xTimeOut;
#endif

  int sentLen = 0;

  if (n == NULL)
    return -1;

#if defined(CONFIG_OS_NUTTX)
	TimerInit(&timer);
	TimerCountdownMS( &timer, timeout_ms );
#else
  vTaskSetTimeOutState(&xTimeOut); /* Record the time at which this function was entered. */
#endif
  do
  {
    int rc = 0;

    mbedtls_ssl_conf_read_timeout(&g_ssl_conf, timeout_ms);
    rc = mbedtls_ssl_write(n->mqtt_context, (unsigned char *)buf, len);
    if (rc > 0)
      sentLen += rc;
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

#if 0
static mbedtls_ssl_session *
mqtt_session_find(const struct sockaddr *addr)
{
  if (g_mqtt_session_cache_enabled) {
    for (size_t i = 0; i < MQTT_MAX_SESSIONS; i++) {
      if (g_mqtt_sessions[i].session != NULL &&
          memcmp(addr, &g_mqtt_sessions[i].address, sizeof(struct sockaddr)) == 0) {
        return g_mqtt_sessions[i].session;
      }
    }
  }

  return NULL;
}

static void
mqtt_session_update(mbedtls_ssl_context *ctx, const struct sockaddr *addr)
{
  int idx = -1;
  mbedtls_ssl_session *session;

  if (!g_mqtt_session_cache_enabled)
    return;

  // Find an empty slot or a matching entry
  for (size_t i = 0; i < MQTT_MAX_SESSIONS; i++) {
    if (g_mqtt_sessions[i].session == NULL) {
      if (idx < 0)
        idx = i;
    } else if (memcmp(addr, &g_mqtt_sessions[i].address, sizeof(struct sockaddr)) == 0) {
      idx = i;
      break;
    }
  }
  if (idx < 0) {
    // Table is full, and no entry matches. Delete the oldest entry (at index 0).
    session = g_mqtt_sessions[0].session;
    mbedtls_ssl_session_free(session);
    MBEDTLS_FREE(session);
    for (size_t i = 0; i < MQTT_MAX_SESSIONS - 1; i++)
      g_mqtt_sessions[i] = g_mqtt_sessions[i + 1];
    idx = MQTT_MAX_SESSIONS - 1;
    g_mqtt_sessions[idx].session = NULL;
  }

  session = g_mqtt_sessions[idx].session;
  if (session == NULL)
    session = MBEDTLS_CALLOC(1, sizeof(mbedtls_ssl_session));
  else
    mbedtls_ssl_session_free(session);
  mbedtls_ssl_session_init(session);
  if (mbedtls_ssl_get_session(ctx, session)) {
    mbedtls_ssl_session_free(session);
    MBEDTLS_FREE(session);
    g_mqtt_sessions[idx].session = NULL;
    return;
  }
  g_mqtt_sessions[idx].session = session;
  memcpy(&g_mqtt_sessions[idx].address, addr, sizeof(struct sockaddr));
}
#endif
