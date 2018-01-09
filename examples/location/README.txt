examples/location
^^^^^^^^^^^^^^
  This sample is an example of using a location manager.
  Location is fixed using measurement data acquired
  from gnss sensor or network positioning measurement.
  When location is fixed, the Control ID, timestamp(UTC)
  and positioning data(Latitude, Longitude, Altitude) is output.

  Supported modem is:

  - ALT1160

  This application depends on its device.

  Configuration Pre-requisites:

    CONFIG_GPSUTILS_LOCATION_MANAGER - Enable Location manager library
    CONFIG_LIBC_FLOATINGPOINT        - Enable floating point
    CONFIG_CXD56_GNSS                - Enable GNSS device

    Assisted GPS used requisites:

      CONFIG_CXD56_EMMC                  - Disable CXD56xx eMMC.
      CONFIG_CXD56_SPI5                  - Enable CXD56xx SPI5.
      CONFIG_CXD56_SPI5_PINMAP_EMMC      - SPI5 assigns to the shared pins with eMMC.
      CONFIG_CXD56_GPIO_IRQ              - Enable CXD56xx GPIO interrupt.
      CONFIG_CXD56_DMAC                  - Enable CXD56xx DMAC.
      CONFIG_CXD56_DMAC_SPI5_MAXSIZE     - Max DMA transfer size. Default: 1516
      CONFIG_ARCH_DMA                    - Enale DMA architecture.
      CONFIG_MODEM                       - Enable modem support.
      CONFIG_MODEM_ALT_1160              - Enable ALT1160 modem support.
      CONFIG_MODEM_ALT_1160_PROTCOL_V2_1 - Enable ALT1160 modem driver protocol version 2.1
      CONFIG_NET_LWIP                    - Enable network stack of LwIP.
      CONFIG_LIBM                        - Enable Standard Math library support.
      CONFIG_GPSUTILS_CXD56NMEA_LIB      - Enable CXD56xx NMEA library support.
      CONFIG_RIL                         - Enable radio interface library support.

      It is possible to set it automatically by using "corvo/lte" configuration.

  Example Configuration:

    CONFIG_EXAMPLES_LOCATION           - Enable Location manager example
    CONFIG_EXAMPLES_LOCATION_PROGNAME  - Program name.
    CONFIG_EXAMPLES_LOCATION_PRIORITY  - Example priority. Default: 100
    CONFIG_EXAMPLES_LOCATION_STACKSIZE - Example stack size. Default: 2048

  Operation:
    Location manager operates by command input.

  Command:

    location init
      Initial processing of location manager is done.

    location start
      Location measurement starts.
      It is returned control ID when location measurement starts success.

    location stop [control_id]
      Location measurement stops with the specified coefficient.
      The coefficient is an optional parameter,
      and if not set it will operate with the returned value
      when location measurement started of last.
      
      - control_id
         It is returned value when location measurement started.

    location fin
      Location manager the end processing.

