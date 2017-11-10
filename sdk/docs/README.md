Spritzer SDK Documentation
=====================================
[TOC]

# General #    {#spritzer_overview_general}

**Spritzer** is the code name of the product **CXD5602** of Sony Semiconductor Solutions Corporation.\n
The CXD5602 is a semiconductor device. And it is developped as a SoC for wearable and IoT devices.\n
\n
Spritzer SDK is based on NuttX. \n
So please refer to [original NuttX site](http://www.nuttx.org/) to see basic kernel information.

# The CXD5602 HW Overview #	{#spritzer_overview_hw_overview}

- [What is the Spritzer] (@ref what_is_spritzer)


# Spritzer SDK Additional Functions #    {#spritzer_overview_additionals}

Spritzer SDK Overview is as follows.

\image html sdk_arch_overview_sml.png
<div class="figure_annot">Fig. Spritzer SDK Overview</div>
\n

The following links are describing details for each additional functions.

- [ASMP Framework] (@ref spritzer_asmp_framework_general)
- [Sensor Control Unit] (@ref scu_doc)
- [Audio] (@ref audio_doc)
- [GNSS] (@ref gnss_doc)
- [LTE] (@ref lte_doc)
- [BLE] (@ref ble_doc)
- [NFC] (@ref nfc_doc)
- [Board Specific Information] (@ref boardspec_doc)
- [Power Management] (@ref powermnagement_doc)
- [Battery] (@ref battery_doc)

# OSS Licenses #    {#spritzer_overview_license}

- Spritzer SDK Additional Functions
  + Generic
  + LwIP
  + Nordic BLE
  + NFC
- NuttX
- NuttX sony_apps
- Binary Firmwares

**License of Spritzer SDK Additional Functions**

Some functions are added on NuttX for this Spritzer.\n
And those are granted under a license as follows.\n
\n
Basically, Spritzer SDK is licensed by BSD-style License.\n
But SDK uses some other Open Source Codes. So we follows the licenses for those OSS.\n
The List of Open Source Codes the SDK uses are as follows.\n
\n

- **LwIP**
- **Nordic BLE**
- **NFC**

\n
Spritzer basic license.

\include Spritzer_License
\n
LwIP license.\n

\include nuttx/net/lwip/lwip_license.txt
\n
Nordic BLE license.\n

Will be added soon....\n

\n
NFC license.\n

Will be added soon....\n

\n
**License of NuttX**

\include nuttx/COPYING

\n
**License of NuttX apps**

\include sony_apps/COPYING

**License of Binary Firmwares**

\include proprietary/LICENSES.txt
