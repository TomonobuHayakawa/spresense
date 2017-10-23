Welcome to SPRITZER SDK

# Overview

Spritzer is the code name of the product CXD5602 produced by Sony Semiconductor
Solutions Corporation.  
The CXD5602 is a ARM arhitecture SoC, including many peripherals for IoT and
wearable solutions.
  
SPRITZER SDK is based on NuttX.
So please refer to original NuttX site to see basic kernel information.

This SDK constructed by series of libraies, drivers and architecture specific
code, these helps to you getting started and create faster for your products.


## Directory structure

```
sdk
|-- bsp            - Board support package
|   |-- board      - Board specific code
|   |-- include    - SDK headers
|   |-- scripts    - Linker scripts
|   `-- src        - Common source
|-- configs        - Default configuration files
|   `-- kernel
|-- drivers        - External device drivers
|   |-- lcd
|   `-- sensors
|-- lib            - Kernel and SDK library output directory
|-- modules
|   |-- asmp       - ASMP framework
|   |-- audio      - Audio library
|   |-- bluetooth  - Bluetooth
|   |-- include    - Library headers
|   |-- lte        - LTE library
|   |-- nfc        - NFC library
|   `-- sensing    - Smart sensing library
|-- system         - System commands
`-- tools          - Build utilities
```
