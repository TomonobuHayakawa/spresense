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
 *    Allan Stockdill-Mander/Ian Craggs - initial API and implementation and/or initial documentation
 *    Ian Craggs - documentation and platform specific header
 *******************************************************************************/

#if !defined(__MQTT_CLIENT_C_)
#define __MQTT_CLIENT_C_

#if defined(__cplusplus)
 extern "C" {
#endif

#if defined(WIN32_DLL) || defined(WIN64_DLL)
  #define DLLImport __declspec(dllimport)
  #define DLLExport __declspec(dllexport)
#elif defined(LINUX_SO)
  #define DLLImport extern
  #define DLLExport  __attribute__ ((visibility ("default")))
#else
  #define DLLImport
  #define DLLExport
#endif
 
#include "MQTTSocket.h"
#include "MQTTPacket.h"
#include "stdio.h"

/** 
 * @ingroup lteiftop
 * @defgroup MQTT
 * MQTT interface
 * @{
 */


#if defined(MQTTCLIENT_PLATFORM_HEADER)
/* The following sequence of macros converts the MQTTCLIENT_PLATFORM_HEADER value
 * into a string constant suitable for use with include.
 */
#define xstr(s) str(s)
#define str(s) #s
#include xstr(MQTTCLIENT_PLATFORM_HEADER)
#endif

#define MAX_PACKET_ID 65535 /* according to the MQTT specification - do not change! */


#if !defined(MAX_MESSAGE_HANDLERS)
#define MAX_MESSAGE_HANDLERS 5 /* redefinable - how many subscriptions do you want? */
#endif


/**
 * \defgroup Mqtt_client_if MQTT client interface
 * MQTT client interface
 * \{
 */
/**
 * \brief          MQTT QOS
 */
enum QoS {
  QOS0,  /**< QOS0 */
  QOS1,  /**< QOS1 */
  QOS2   /**< QOS2 */
 };


/**
 * \brief          return code
 */
enum returnCode {
  BUFFER_OVERFLOW = -2, /**< buffer overflow */
  FAILURE = -1,         /**< failure */
  SUCCESS = 0           /**< success */
};

/* The Timer structure must be defined in the platform specific header,
 * and have the following functions to operate on it.  */
extern void TimerInit(Timer*);
extern char TimerIsExpired(Timer*);
extern void TimerCountdownMS(Timer*, unsigned int);
extern void TimerCountdown(Timer*, unsigned int);
extern int TimerLeftMS(Timer*);


/**
 * \brief           MQTT message strutcture
 */
typedef struct MQTTMessage
{
    enum QoS qos;             /**< QoS in MQTT header */
    unsigned char retained;   /**< RETAIN flag in MQTT header */
    unsigned char dup;        /**< DUP flag in MQTT header*/
    unsigned short id;        /**< message ID in MQTT header */
    void *payload;            /**< MQTT message payload */
    size_t payloadlen;        /**< length of MQTT message payload */
} MQTTMessage;

/**
 * \brief           MQTT message strutcture
 */
typedef struct MessageData
{
    MQTTMessage* message;     /**< MQTT message */
    MQTTString* topicName;    /**< MQTT topic name */
} MessageData;

typedef void (*messageHandler)(MessageData*);


/**
 * \brief           MQTT client object structure
 * User has not to care about the detai of this structure.
 */
typedef struct MQTTClient
{
    unsigned int next_packetid,
    command_timeout_ms;
    size_t buf_size,
    readbuf_size;
    unsigned char *buf,
    *readbuf;
    unsigned int keepAliveInterval;
    char ping_outstanding;
    int isconnected;
    char *pRootCALocation;
    char *pDeviceCertLocation;
    char *pDevicePrivateKeyLocation;
    MessageData md;
    MQTTMessage msg;
    MQTTString topicName;

    struct MessageHandlers
    {
        const char* topicFilter;
#if defined(CONFIG_OS_NUTTX)
        messageHandler callback;
#else
        uint32_t MessageHandler_id;
#endif
    } messageHandlers[MAX_MESSAGE_HANDLERS];

    void (*defaultMessageHandler) (MessageData*);

    MQTTSocket* ipstack;
    Timer ping_timer;
#if defined(MQTT_TASK)
	Mutex mutex;
	Thread thread;
#endif 
} MQTTClient;

#define DefaultClient {0, 0, 0, 0, NULL, NULL, 0, 0, 0}


/**
 * Create an MQTT client object
 * @param client   MQTT client object
 * @param network  MQTT socket
 * @param command_timeout_ms  timeout time( unit:ms )
 * @param sendbuf address of application prepare area
 * @param sendbuf_size size of sendbuf
 * @param readbuf address of application prepare area
 * @param readbuf_size size of readbuf
 */
DLLExport void NT_MQTTClientInit(MQTTClient* client, MQTTSocket* network, unsigned int command_timeout_ms,
		unsigned char* sendbuf, size_t sendbuf_size, unsigned char* readbuf, size_t readbuf_size);

/** MQTT Connect - send an MQTT connect packet down the network and wait for a Connack
 *  The nework object must be connected to the network endpoint before calling this
 *  @param client  MQTT client object
 *  @param options  connect options
 *  @return success code
 */
DLLExport int NT_MQTTConnect(MQTTClient* client, MQTTPacket_connectData* options);

/** MQTT Publish - send an MQTT publish packet and wait for all acks to complete for all QoSs
 *  @param client  the client object to use
 *  @param topic  the topic to publish to
 *  @param message  the message to send
 *  @return success code
 */
DLLExport int NT_MQTTPublish(MQTTClient* client, const char *topic, MQTTMessage *message);

/** MQTT Subscribe - send an MQTT subscribe packet and wait for suback before returning.
 *  @param client  the client object to use
 *  @param topicFilter  the topic filter to subscribe to
 *  @param QoS  MQTT QoS in subscribing
 *  @param messageHandler  Callback function address that will be called when receiving MQTT PUBLISH message about topicFilter
 *  @return success code
 */
DLLExport int NT_MQTTSubscribe(MQTTClient* client, const char* topicFilter, enum QoS, messageHandler messageHandler);

/** MQTT UnSubscribe - send an MQTT unsubscribe packet and wait for unsuback before returning.
 *  @param client - the client object to be unsubsribed
 *  @param topicFilter - the topic filter to be unsubscribed from
 *  @return success code
 */
DLLExport int NT_MQTTUnsubscribe(MQTTClient* client, const char* topicFilter);

/** MQTT Disconnect - send an MQTT disconnect packet and close the connection
 *  @param client  the client object to be disconnected
 *  @return success code
 */
DLLExport int NT_MQTTDisconnect(MQTTClient* client);

/** MQTT Yield - MQTT background
 *  @param client  the client object to use
 *  @param time  the time, in milliseconds, to yield for 
 *  @return success code
 */
DLLExport int NT_MQTTYield(MQTTClient* client, int time);

#if defined(MQTT_TASK)
/** MQTT start background thread for a client.  After this, MQTTYield should not be called.
*  @param client  the client object to use
*  @return success code
*/
DLLExport int MQTTStartTask(MQTTClient* client);
#endif
/* \} */

/**@}*/


#if defined(__cplusplus)
     }
#endif

#endif
