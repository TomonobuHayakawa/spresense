examples/tracker
^^^^^^^^^^^^^^^^
  This sample is an example that send current GNSS position to the server 
  using HTTP POST method via LTE network.
  This application send current GNSS position periodically after FIX position.
  It is possible to change it event driven method. Either TAP detection or 
  button detection can be selected.

  Configuration Pre-requisites:

    CONFIG_NETUTILS_HTTPC
    CONFIG_NETUTILS_MBEDTLS ( If not using TLS, not needed.)
    CONFIG_NETUTILS_JSON
    not select CONFIG_CXD56_EMMC
    CONFIG_CXD56_DMAC
    CONFIG_CXD56_SPI5
    CONFIG_CXD56_SPI5_PINMAP_EMMC
    CONFIG_CXD56_DMAC_SPI5_TX
    CONFIG_CXD56_DMAC_SPI5_TX_CH=4
    CONFIG_CXD56_DMAC_SPI5_TX_MAXSIZE=1516
    CONFIG_CXD56_DMAC_SPI5_RX
    CONFIG_CXD56_DMAC_SPI5_RX_CH=5
    CONFIG_CXD56_DMAC_SPI5_RX_MAXSIZE=1516
    CONFIG_CXD56_GPIO_IRQ
    CONFIG_CXD56_GNSS
    CONFIG_MODEM
    CONFIG_MODEM_ALT_1160
    CONFIG_MODEM_ALT_1160_PROTOCOL_V2_1
    CONFIG_MODEM_ALT_1160_BOOT_DELAY=10
    CONFIG_ARCH_DMA
    CONFIG_NET_LWIP
    CONFIG_RIL

  Example Configuration:

    CONFIG_EXAMPLES_TRACKER                        - Enable tracker example
    CONFIG_EXAMPLES_TRACKER_PROGNAME               - Program name.
    CONFIG_EXAMPLES_TRACKER_PRIORITY               - Example priority. Default: 100
    CONFIG_EXAMPLES_TRACKER_STACKSIZE              - Example stack size. Default: 5120
    CONFIG_EXAMPLES_TRACKER_HTTP_CUSTOM_HEADER_NUM - Max number of custom headers
    CONFIG_EXAMPLES_TRACKER_USING_EVENT_TRIGGER    - Using event driven method
    CONFIG_EXAMPLES_TRACKER_TAP_TRIGGER            - Triggered by TAP detection
    CONFIG_EXAMPLES_TRACKER_BUTTON_TRIGGER         - Triggered by button detection

  Operation:
    Tracker operates by command input.

  Command:

    USAGE: tracker [OPTIONS] <host> <port> <path>

    Where:
            <host> is the destination server address
            <port> is the destination port number
            <path> is the file path

    and OPTIONS include the following:
            -s: Using HTTPS.  Default: HTTP
            -H <LINE>: Pass custom header LINE to server
            -h: Show this text and exit

     ( For example, if you execute 
       tracker -H "Content-Type: application/json; charset=utf-8" httpbin.org 80 /post
       destination server's URL is "http://httpbin.org:80/post".)
