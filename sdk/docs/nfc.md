NFC  {#nfc_doc}
--------------------------------------
[TOC]

This SDK supports NFC(Near Field Communication) function.

# Supported device {#nfc_supported_device}
- CXD224x(Sony Semiconductor Solutions)

____________________________________
# Key features {#nfc_key_features}

Following lists the features of the NFC block. For the details of each function, see the corresponding section.

|Feature|Description|
|:------|:----------|
|Discovery|Initializes the NFC block and CXD224x, starts Discovery operation (Poll/Listen), and waits for connection with an opposing NFC device.|
|Reader|Reads the NFC tag|
|Host Card Emulation(HCE)|Performs Host card emulation to operate as an NFC tag|
|Bluetooth Handover Utility|This utility is used by the application when performing handover from NFC to Bluetooth. It extracts pairing information from read NDEF messages, and embeds pairing information in the NDEF messages to be sent.|

____________________________________
# Detail description for features {#nfc_detail_description}

## Discovery

- This function initializes the NFC block and CXD224x, starts Discovery operation (Poll/Listen), and waits for connection with an opposing NFC device.
- When Activation to an opposing NFC device is completed and that complete notification is received from CXD224x during Discovery operation, this software starts Reader operation.
- When this software is activated from an opposing NFC device and that notification is received from CXD224x during Discovery operation, this software starts HCE operation.

## Reader

- This function operates CXD224x as an NFC reader, reads the NDEF message of the opposing NFC tag, and transfers it to the application. The protocol process is performed inside the NFC block in accordance with the NCI packet protocol process and tag type received via the I2C, and the NDEF message is reconfigured.
- The readable tag types are as follows.
  * NFC Forum Type 1 Tag
  * NFC Forum Type 2 Tag
  * NFC Forum Type 3 Tag
  * NFC Forum Type 4 Tag

## Host Card Emulation(HCE)
- This function operates CXD224x as an NFC tag, and transmits the NDEF message designated by the application to the opposing NFC device. Formatting is performed inside the NFC block in accordance with the tag type, and then NCI packets are generated and transferred to CXD224x via the I2C.
- The tag types that can be emulated are as follows.
  * NFC Forum Type 3 Tag
  * NFC Forum Type 4 Tag

## Bluetooth Handover Utility
This utility is used by the application to perform static handover to Bluetooth.
- Received NDEF messages are interpreted and the out-of-band (OOB) data is extracted. This OOB data includes the pairing information (BD_ADDR, etc.).
- The OOB data transferred from the application is embedded in the NDEF messages to be transmitted.

For the details of static handover and OOB, see the reference document of "NFC Forum Bluetooth Secure Simple Pairing Using NFC" published by NFC Forum and Bluetooth SIG.

____________________________________
# Software block diagram {#nfc_block_diagram}

The software block diagram of NFC function is shown below.

![Block diagram](nfc_libnfc_block_diagram.png)

NFC features consists of modules colored in red in the above diagram. Followings describe overview of each module
The NFC block consists of the modules indicated by the green boxes in the figure above, and operates on the DSP processor core to control CXD224x as the device Host.
The Host  interface uses an I2C port and a GPIO port.
Each module is described below.

* API : This module is the API provided to the user application.
This provides an abstracted interface that enables the application to easily use the Reader and HCE functions of libnfc.
* libnfc : This module processes the NFC protocol.
This mainly performs NDEF(NFC Data Exchange Format) handling and the issue and response processes of NCI(NFC Controller Interface ) commands used to control CXD224x.
* cxd224x drv : This module abstracts the host-device communication required for CXD224x control. It uses an I2C(master operation) and GPIO to detect interrupt as Host interfaces, and performs control unique to CXD224x via these Host interfaces. This module calls the low level drivers (I2C, GPIO) to achieve this.

____________________________________
# API Sequence {#nfc_libnfc_api_sequence}

The following sections show the sequence for each function. The NFC API Layer part in the sequence charts is executed by the user application context. The arrows in the sequence charts have the following meanings.

![Legend](nfc_libnfc_legend.png)

## Initialization

![Initialization sequence](nfc_libnfc_initialize.png)

## Reader function

![Reader function sequence](nfc_libnfc_reader.png)

## HCE function

![HCE function sequence](nfc_libnfc_hce.png)

## Finalization

![Finalization sequence](nfc_libnfc_finalize.png)

____________________________________
# State Transitions {#nfc_state_transtions}
As shown below, NFC block performs the state transitions by calling API.

![State transition diagram](nfc_libnfc_state.png)

Meaning of each states is listed below.

|State|Description|
|:----|:----------|
|Discovery|State in which Discovery operation is performed and operation waits for communication with an opposing NFC device.|
|Reader|State in which Reader operation is performed by communication with an opposing NFC device.|
|HCE|State in which HCE operation is performed by communication with an opposing NFC device.|


____________________________________
# Configuration {#nfc_configuration}

Followings are configuration required for NFC block.

## API, libnfc, cxd224x drv

~~~
CONFIG_CXD56_I2C1=y
CONFIG_CXD56_GPIO_IRQ=y

CONFIG_CLOCK_MONOTONIC=y
CONFIG_SCHED_WAITPID=y
CONFIG_MUTEX_TYPES=y
CONFIG_USERMAIN_STACKSIZE=8192
CONFIG_PTHREAD_STACK_DEFAULT=8192

CONFIG_HAVE_CXX=y
CONFIG_HAVE_CXXINITIALIZE=y

CONFIG_I2C_DRIVER=y
CONFIG_DRIVERS_WIRELESS=y

CONFIG_LIBNFC=y
~~~

## Example application

~~~
CONFIG_SERIAL_TERMIOS=y
CONFIG_SCHED_WAITPID=y

CONFIG_EXAMPLES_NFC_TOP=y
CONFIG_EXAMPLES_NFC_READER_HCE=y
CONFIG_EXAMPLES_NFC_READER_HCE_PRIORITY=100
CONFIG_EXAMPLES_NFC_READER_HCE_STACKSIZE=2048
~~~
____________________________________
# Examples  {#nfc_examples}

## Application examples  {#nfc_examples_apps}
- [sony_apps/examples/nfc/reader_hce/reader_hce.c](\ref examples/nfc/reader_hce/reader_hce.c)

\example examples/nfc/reader_hce/reader_hce.c

____________________________________
