examples/mqtt
^^^^^^^^^^^^^^
  This sample is an example of using a MQTT utility.

  Supported MQTT functions are:
  - Connect to MQTT broker
  - Subscribe ( support all QOS )
  - Publish   ( supoort all QOS )
  - Unsubscribe


  Configuration Pre-requisites:
    ( By executing "./configure.sh corvo/lte", the following settings is applied automatically. )

    CONFIG_NETUTILS_MQTT          - MQTT utility
    CONFIG_NETUTILS_MBEDTLS       - mbedTLS utility ( If not using TLS, not needed.)

  Example Configuration:

    CONFIG_EXAMPLES_MQTT           - Enable MQTT example
    CONFIG_EXAMPLES_MQTT_PROGNAME  - Program name.
    CONFIG_EXAMPLES_MQTT_PRIORITY  - Example priority. Default: 100
    CONFIG_EXAMPLES_MQTT_STACKSIZE - Example stack size. Default: 2048

  Operation:
    MQTT operates by command input.

  Command:

    mqtt init
      Initial processing of MQTT is done(include LTE connection establishment).

    mqtt connect [useTLS]  [MQTTbroker_port]
      Send MQTT CONNECT message to MQTT broker.

      Connection informations that is set are:
        MQTT client ID                   : set by definition in source code.(BDKCONNECT_CLIENT_ID in mqtt_mainc.c)
        MQTT device ID                   : set by definition in source code.(BDKCONNECT_DEVICE_ID in mqtt_mainc.c)
        MQTT broker host name            : set by definition in source code.(BDKCONNECT_BROKER in mqtt_mainc.c)
        MQTT broker port                 : set by command line parameter
        useTLS                           : set by command line parameter
          The following 3 parameters are needed only in using TLS.
            RootCA certificate path      : set by definition in source code.(BDKCONNECT_ROOTCA in mqtt_mainc.c)
            Client certificate path      : set by definition in source code.(BDKCONNECT_DEV_CERT in mqtt_mainc.c)
            Private key path             : set by definition in source code.(BDKCONNECT_PRI_KEY in mqtt_mainc.c)

      - useTLS ( 0 or 1)
          0 means "do not use TLS".
          1 means "use TLS"

      - MQTTbroker_port
          Depends on MQTT broker setting. ( for example, 1883 or 8883 )


    mqtt sub [topic] [qos]
      Send MQTT Publish message to MQTT broker.
      - topic
          Specify MQTT topic that is subscribed. ( arbitrary character string.)
      - qos ( 0Å`2 )
          Specify MQTT QOS.


    mqtt pub [topic] [data] [qos]
      Send MQTT Publish message to MQTT broker.
      - topic
          Specify MQTT topic that is published. ( arbitrary character string.)

      - data
          Specify MQTT data that you want to publish.
          
      - qos ( 0Å`2 )
          Specify MQTT QOS.
          
     mqtt unsub [topic]
      - topic
          Specify MQTT topic that is unsubscribed. ( arbitrary character string.)


     mqtt end
      End application.


  Pre-requisites for MQTT on TLS
    Put rootCA certificate,  Client certificate, and Private key on /mnt/spif. 
    (The filename is arbitrary, but same as filename in the program(BDKCONNECT_ROOTCA/BDKCONNECT_DEV_CERT/BDKCONNECT_PRI_KEY.)
     These files are used on TLS certification on MQTT Broker.)
