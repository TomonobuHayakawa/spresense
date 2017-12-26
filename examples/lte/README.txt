examples/lte
^^^^^^^^^^^^

  This sample can resistration to operator's LTE network via modem.

  Supported LTE functions are:

  - Data communication.
  - Sending and receiving SMS message.
  - Confirm connection status of LTE network.

  Supported modem is:

  - ALT1160

  Configuration Pre-requisites:

    CONFIG_CXD56_EMMC - Disable CXD56xx eMMC.
    CONFIG_CXD56_SPI5 - Enable CXD56xx SPI5.
    CONFIG_CXD56_SPI5_PINMAP_EMMC - SPI5 assigns to the shared pins with eMMC.
    CONFIG_CXD56_GPIO_IRQ - Enable CXD56xx GPIO interrupt.
    CONFIG_CXD56_DMAC - Enable CXD56xx DMAC.
    CONFIG_CXD56_DMAC_SPI5_MAXSIZE - Max DMA transfer size. Default: 1516
    CONFIG_ARCH_DMA - Enale DMA architecture.
    CONFIG_MODEM - Enable modem support.
    CONFIG_MODEM_ALT_1160 - Enable ALT1160 modem support.
    CONFIG_MODEM_ALT_1160_PROTCOL_V2_1 - Enable ALT1160 modem driver protocol version 2.1
    CONFIG_NET_LWIP - Enable network stack of LwIP.
    CONFIG_LIBM - Enable Standard Math library support.
    CONFIG_GPSUTILS_CXD56NMEA_LIB - Enable CXD56xx NMEA library support.
    CONFIG_RIL - Enable radio interface library support.

  Configuration in this example:

    CONFIG_EXAMPLES_LTE - Enable this example
    CONFIG_EXAMPLES_LTE_PROGNAME - Program name
    CONFIG_EXAMPLES_LTE_PRIORITY - Example priority. Default: 100
    CONFIG_EXAMPLES_LTE_STACKSIZE - Example stack size. Default: 2048

  It is possible to set it automatically by using "corvo/lte" configuration.

  Operation:

    In this application the following commands are available on the console.

    lte init
      - Initialization command for this application program.
      - After executing this command, the following commands can be executed.
      - No need to execute this command until "lte end" command is execute.

    lte end
      - Termination command for this application program.
      - After executing this command, the following commands cannot be executed.

    lte system connect
      - Connect to LTE network.

    lte system disconnect
      - Disconnect from LTE network.

    lte system confirm
      - Confirm LTE network connection.

    lte system state
      - Get LTE connection state of LTE manager.
        -------------------------------------------------------------------------
        | Value | State      | Description                                      |
        -------------------------------------------------------------------------
        |   5   | Limited    | PIN confirmation failed and services are limited |
        -------------------------------------------------------------------------
        |   4   | Connect    | Connected                                        |
        -------------------------------------------------------------------------
        |   3   | Establish  | Connection established and wait for setup        |
        -------------------------------------------------------------------------
        |   2   | Pin        | PIN confirmed                                    |
        -------------------------------------------------------------------------
        |   1   | Init       | Initialized and not connected                    |
        -------------------------------------------------------------------------
        |   0   | Disconnect | Disconnected (default)                           |
        -------------------------------------------------------------------------

    lte system pin
      - Get PIN information from SIM. 

    lte system power
      - Get modem power state. 
        -----------------------
        | Value | State       |
        -----------------------
        |   1   | Wake state  |
        -----------------------
        |   0   | Sleep state |
        -----------------------

    lte system event (value)
      - Get / set enable mask of event (default is 0x0000).
      - Get if "value" is null.
        ---------------------------------
        | Value  |  Description         |
        ---------------------------------
        | 0x0001 |  System event        |
        ---------------------------------
        | 0x0002 |  Network state event |
        ---------------------------------
        | 0x0004 |  IMS state event     |
        ---------------------------------
        | 0x0008 |  PDN state event     |
        ---------------------------------
        | 0x0010 |  EPS state event     |
        ---------------------------------

    lte system sleep
      - Sleep LTE modem.

    lte system wakelock
      - Acquire LTE modem wakelock.

    lte system unlock
      - Release LTE modem wakelock.

    lte system lockstate
      - Get LTE modem wakelock state.

    lte system notify (value)
      - A-GPS location request notification.
        ------------------------------------------------------------------------------
        | Value  |  Description                                                      |
        ------------------------------------------------------------------------------
        |   0    |  Disables reporting and positioning                               |
        ------------------------------------------------------------------------------
        |   1    |  Subscribe for notifications of MT-LR over control plane          |
        ------------------------------------------------------------------------------
        |   2    |  Subscribe for notifications of MT-LR over SUPL                   |
        ------------------------------------------------------------------------------
        |   3    |  Subscribe for notifications of MT-LR over control plane and SUPL |
        ------------------------------------------------------------------------------

    lte system allow (allow) (handle)
      - A-GPS location request allowance.
        ------------------------------------------------------------------------------------
        | Parameter      | Value | Description                                             |
        ------------------------------------------------------------------------------------
        | allow          |   0   | Location disclosure allowed                             |
        |                -------------------------------------------------------------------
        |                |   1   | Location disclosure not allowed                         |
        ------------------------------------------------------------------------------------
        | handle         |   x   | The value given in A-GPS location request notification  |
        ------------------------------------------------------------------------------------

    lte system auto (value)
      - Get / set A-GPS location request auto allowance.
      - Get if "value" is null.
        ---------------------------------
        | Value  |  Description         |
        ---------------------------------
        |   0    |  Manual mode         |
        ---------------------------------
        |   1    |  Auto mode (OK)      |
        ---------------------------------
        |   2    |  Auto mode (NG)      |
        ---------------------------------

    lte system start
      - Execute LT_Start().

    lte system stop
      - Execute LT_Stop().

    lte system reset
      - Reset modem and status.

    lte utility version
      - Get version information of LTE manager and ALT1160 FW.

    lte utility number
      - Get telephone number of SIM card.

    lte utility report
      - Get error report.

    lte utility quality
      - Get RF signal quality information.

    lte utility register
      - Get network and IMS registration status and information.

    lte utility clock
      - Get time information [yy/MM/dd,hh:mm:ssÅ}zz].

    lte utility data (home) (roaming) (sync)
      - Get / Set data disable on home/roaming and time synchronization flag.
      - Get if "home" is null.
        -------------------------------------------------------------------------
        | Parameter      | Value | Description                                  |
        -------------------------------------------------------------------------
        | home / roaming | 0 / 0 | Enable data transaction (default)            |
        |                |-------------------------------------------------------
        |                | 0 / 1 | Disable data transaction on roaming          |
        |                |-------------------------------------------------------
        |                | 1 / x | Disable data transaction                     |
        -------------------------------------------------------------------------
        | sync           | 0     | Ignore time packet from LTE network          |
        |                |-------------------------------------------------------
        |                | 1     | Sync time packet from LTE network (default)  |
        -------------------------------------------------------------------------

    lte utility (ATcommand)
      - Send direct AT command to Alt1160, and receive response and "OK".

    lte sms config
      - Configure SMS default setting.

    lte sms memory (mem)
      - Get / Set SMS access memory.
      - Get if "mem" is null, and only "ME" and "SM" are available for "mem".

    lte sms list
      - Get SMS message list.

    lte sms read (num)
      - Get 1 SMS message of "num" in memory.

    lte sms write (tel) (message)
      - Write 1 SMS message to memory.
      - Set "tel" for telephone number, and "message" for message.

    lte sms send (num)
      - Send 1 SMS message of "num" in memory.

    lte sms direct (tel) (message)
      - Send 1 SMS message directly.
      - Set "tel" for telephone number, and "message" for message.

    lte sms delete (num)
      - Delete 1 SMS message of "num" in memory.
      - If "num" is "all", all messages can be deleted. 

    lte net GET (host) (port) (directory)
      - HTTP/1.0 GET command.
      - host : host name (ex. www.sony.com)
      - port : port number (ex. 80)
      - directory : URL directory (ex. / or /index.htm) need "/" for start charactor.

    lte net POST (host) (port) (directory) (data)
      - HTTP/1.0 POST command.
      - host : host name (ex. www.sony.com)
      - port : port number (ex. 80)
      - directory : URL directory (ex. / or /index.htm) need "/" for start charactor.
      - data : POST data

