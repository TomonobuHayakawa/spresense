LTE {#lte_doc}
============================
[TOC]

# General # {#spritzer_lte_general}

Spritzer LTE Overview is as follows.

\image html lte_architecture_overview.png
<div class="figure_annot">Fig. Spritzer LTE Architechure Overview</div>
\n

\image html lte_network_structure_overview.png
<div class="figure_annot">Fig. Spritzer LTE Network Structure Overview</div>
\n


--------------------------------------
# Key features:

 - TCP/IP Basic Protocol Stacks (LwIP base)
  - And also DHCP client and DNS client
 - LTE functionalities
  - Data communication
  - Short Message Service
  - VoLTE
  - A-GPS
 - Support network libraries
   - HTTP and HTTPS
   - SSL/TLS (mbedTLS)
   - MQTT
<br>


--------------------------------------
# Network Layer # {#spritzer_lte_network_layer}

User applications can use TCP/IP functions on LTE network.
- Support TCP and UDP.
- Can use TCP/IP with applications and RIL simultaneously.
- Can use general BSD (Berkeley) socket API
<br>

## Network API
- Socket API(@ref netlwip)
- MQTT library(@ref MQTT)
- mbed_tls library(@ref Net_mbed_tls)
<br>


--------------------------------------
# Radio Interface Layer # {#spritzer_lte_radio_interface_layer}

RIL layer for CXD5602 and Alt1160 system is named "LTE Manager".
"LTE Manager" has original functions API.
- C language API
- Provide minimum functions for LTE (not exist for GSM/CDMA etc.).
- Modem power management

"LTE Manager" is divided into 5 library for application use.
- System manager library
 - This library manages connection between CXD5602 and Alt1160 and configure for LTE connection.
 - This library can control modem power management.
- System utility library
 - This library is a utility toolbox for LTE connection of Alt1160.
- SMS manager library
 - This library manages SMS functions of Alt1160.
- A-GPS manager library
 - This library manages A-GPS functions of Alt1160.
 - This library manages positionig functions of CXD5602.
- VoLTE manager library
 - This library manages VoLTE functions of Alt1160.
 - This library manages audio functions of CXD5602.
<br>

## LTE API
- LTE Common API(@ref lteifcom)
- LTE System API(@ref lteifsys)
- LTE Utility API(@ref lteifutl)
- LTE SMS API(@ref lteifsms)
- LTE A-GPS API(@ref lteifgps)
- LTE VoLTE API(@ref lteifvol)
<br>

--------------------------------------
## Configurations #     {#spritzer_lte_configurations}
There is a configuration script in the sony_apps/tools/ directory that makes this easier. <br>
It is used as follows: <br>
- $ cd sony_apps <br>
- $ ./tools/configure.sh <board-name>/<config-dir> <br>

An example: <br>
- $ cd sony_apps <br>
- $ ./tools/configure.sh corvo/lte <br>

Select options in below.
~~~
[System Type]
  [CXD56xx Peripheral Support]
    [eMMC] <= N
    [SPI5] <= Y
      [SPI5 pin assign to eMMC] <= Y
    [GPIO interrupt support] <= Y
    [DMAC support] <= Y
      [DMAC settings for SPI5] <= 1516
[Device Drivers]
  [Modem Support] <= Y
    [Enable alt1160 modem driver]  <= Y
      [Enable alt1160 modem driver protocol version 2.1]  <= Y
[Networking Support]
  [LwIP Networking support] <= Y
[Library Routines]
  [Standard Math library] <= Y
[Application Configuration]
  [GPS Utilities]
    [Support CXD56xx gnss NMEA convert library] <= Y
  [Radio Interface Library]
    [Support Radio Interface Library] <= Y
~~~

These options are for using basic LTE functions.<br>
For additional options for using functions such as VoLTE and A-GPS, refer to the README.txt of each example.

<br>

--------------------------------------
## Examples #   {#spritzer_lte_example}

Some samples are shown below.
For details, please refer to the README.txt of each example.

### LTE(Data communication / Short Message Service) example
- [sony_apps/examples/lte/lte_main.c](\ref examples/lte/lte_main.c)

### A-GPS example
- [sony_apps/examples/location/location_main.c](\ref examples/location/location_main.c)

### VoLTE example
- [sony_apps/examples/volte/volte_main.c](\ref examples/volte/volte_main.c)

### HTTP client example
- [sony_apps/examples/httpc/httpc_main.c](\ref examples/httpc/httpc_main.c)

### MQTT example
- [sony_apps/examples/mqtt/mqtt_main.c](\ref examples/mqtt/mqtt_main.c)

\example examples/lte/lte_main.c
\example examples/location/location_main.c
\example examples/volte/volte_main.c
\example examples/httpc/httpc_main.c
\example examples/mqtt/mqtt_main.c
