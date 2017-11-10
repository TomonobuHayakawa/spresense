Bluetooth Low Energy  {#ble_doc}
-------------------------------------
[TOC]

This SDK supports Bluetooth low energy function.

# Supported device: {#Supported_device}
- nRF51822(Nordic)

---

# Key features: {#key_features}

- Create a BLE role
    - Supports the below GAP role
        * Central
        * Peripheral
        * Broadcaster
        * Observer

- Scan
    - Discover a new BLE device (central role only)
- Advertise
    - Advertise data to the central device
- Set/Get configuration
    - Set/get local device configuration
- Bond
    - A procedure to create trusted relationship between two connected devices
- Connect device 
    - Establish a connection between two devices
- Add services
    - Add a service to attribute database
- Send notification
    - Send notification/indication to GATT client device
- Discover services
    - Discover services form GATT server
- Receive notification
    - Receive notification/indication from GATT server
- Read characteristic
    - Read a characteristic value from GATT server
- Write characteristic
    - Write a characteristic value to GATT server

---

# Detail description for features {#Detail_description}

## Create BLE role

The BLE block provides the functionalities that can create a GAP role.

|Role   |Description|
|:------|:-------------------------|
|Central|Central role initiates the establishment of a physical.Connection and shall have both a transmitter and a receiver.|
|Peripheral|Peripheral role accepts the establishment of a physical link and shall have both a transmitter and a receiver.|
|Broadcaster|Broadcaster role sends advertising events, shall have a transmitter and may have a receiver.|
|Observer|Observer role receives advertising events, shall have a receiver and may have a transmitter.|


## Scan

BLE block provides the functions that central BLE device discover new peripheral BLE devices.
You can control the scan procedure by 
- Start scan
- Stop scan

## Advertise

BLE block provides the functions that peripheral BLE devices can be discovered by central device.
You can control the advertise procedure by

- Start advertising
- Stop advertising
- Set advertising data

## Set/Get configuration

Set and get local device configuration. You can set and get local configuration like:
- Device name
- Address
- Appearance

## Bond

Purpose of bonding is to create a relation between two Bluetooth devices based on a common link key (a bond). 
The link key is created and exchanged (pairing) during the bonding procedure and is expected to be stored by both Bluetooth devices, to be used for future authentication.

## Connect device

Establish a connection between two devices. Include sub-functions as follows:
- Create a Connection
- Disconnect a link
- Manage Connection parameter

## Add services

|Role       |Description|
|:----------|:-------------------------|
|GATT Client|This is the device that initiates commands and requests towards the GATT server. It can receive responses, indications and notifications sent by the GATT server.|
|GATT Server|This is a device that accepts incoming commands and requests from the GATT client. It can send responses, indications and notifications to the GATT client.|

A service is a collection of data and associated behaviors to accomplish a particular function or feature.
Add services include:
- Add service
- Add characteristic
- Add descriptor
Only support GATT Server role.

## Send notification

An attribute value can be notified or indicated to the GATT client.
- Notification: would not expect attribute protocol layer acknowledgement.
- Indication: would expect a confirmation from GATT client.

## Discover services

According to the BLE core specification, Service discovery includes:
- Primary service discovery
- Relationship discovery
- Characteristic discovery
- Characteristic descriptor discovery

## Receive notification

Receive notification/indication from GATT server. 
If it is an indication, a confirmation is sent to GATT server.

## Read characteristic

Read a characteristic value from GATT server.
There are four sub-functions:
- Read Characteristic Value
- Read Using Characteristic UUID
- Read Long Characteristic Values
- Read Multiple Characteristic Values

## Write characteristic

Write a characteristic value to GATT server.
There are five sub-functions:
- Write Without Response
- Signed Write Without Response
- Write Characteristic Value
- Write Long Characteristic Values
- Reliable Writes

---

# API Sequence {#API_sequence}

## Initialization
Before your application can communicate over BLE, you need to initialize the BLE module. This operation is accomplished in two steps:
1.	Initialization protocol stack;
2.	Set the event callback function.

![Initialization sequence](ble_initialization.png)

##Configuration

In your application, may need to configure the BLE address and name.

![Configuration sequence](ble_configuration.png)

##Advertising

When the BLE device is working as peripheral role, it needs to send advertising packets.

![Advertising sequence](ble_advertising.png)

##Connection(Peripheral role)

When central role device receives peripheral device’s advertising packet, the central device can send connection requests to the peripheral device.

![Connection(Peripheral role) sequence](ble_connection.png)

##Finalization

If you don’t need to use the BLE module anymore, you could finalize the module.

![Finalization sequence](ble_finalization.png)

##Paring justworks

If the central device initials a pair request, According to your IO capability, It would be justworks

![Paring justworks sequence](ble_paring_justworks.png)

##Paring passkey

If the central device initials a pair request, According to your IO capability, It would be passkey. You can set static or random passkey by calling GapSetSecParam function.

![Paring passkey sequence](ble_paring_passkey.png)

##Gatts Add service

As a GATT server, you can add a service declaration, characteristic declaration to the attribute table.

![Add service sequence](ble_gatts_add_service.png)

##Gatts write characteristic

When a central role device writes a characteristic, application can be informed by BLE_GATTS_EVENT_WRITE event. 

![Write characteristic sequnece](ble_gatts_write_characteristic.png)

##Gatts handle value notification/indication

As a GATT server, you can handle value notification or indication.

![Handle value noticiation/indication sequence](ble_gatts_handle_value_notification.png)

##Gatts update attribute value

As a GATT server, you can change the attribute value.

![Update attribute value sequence](ble_gatts_update_attribute_value.png)

##Evt display passkey

If the central device initials a pair request, According to your IO capability, It would be passkey pairing. You can set static or random passkey by calling GapSetSecParam function. When passkey pairing, an event BLE_EvtDisplayPasskey is sent to user.

![EvtDisplayPasskey sequence](ble_evtDisplayPasskey.png)

##Scanning

When the BLE device is working as central role, it needs to receive advertising packets.

![Scanning sequence](ble_scanning.png)

##Connection(Central role)

When central role device receives peripheral device’s advertising packet, the central device can send connection requests to the peripheral device.

![Connection(Central role) sequence](ble_connection_central.png)

##Pairing justworks(Central role)

When central role and peripheral role are connected, you might need to start the pairing process. According to your IO capability, It would be justworks.

![Pairing justworks(Central role) sequence](ble_paring_justworks_central.png)

##Pairing passkey(Central role)

When central role and peripheral role are connected, you might need to start the pairing process. According to your IO capability, It would be passkey.

![Pairing passkey(Central role) sequence](ble_paring_passkey_central.png)

##Db discovery

If the connection is established. Central role(usually GATT client) device can initial database discovery in order to find attribute database information in GATT server. 

![Db discovery sequence](ble_db_discovery.png)

##GattC read characteristic

Application of a central role device can read a characteristic from GATT server.

![Gattc read characteristic sequence](ble_gattc_read_characteristic.png)

##Gattc write characteristic

Application of a central role device can write a characteristic from GATT server.

![Gattc write characteristic sequence](ble_gattc_write_characteristic.png)

##Gattc handle value notification/indication

GATT client can handle value notification/indication when GATT server sends notification/indication.

![Gattc handle value notification/indication sequence](ble_gattc_handle_value_notification.png)

##Save bonded information

When the application is bonded with peer device, the application can save the bonded information.

![Save bonded information sequence](ble_save_bonded_information.png)

##Get bonded devices list

The application can get the bonded devices list.

![Get bonded devices list sequence](ble_get_bonded_devices_list.png)

##Clear bonded information

The application can clear the saved bond information.

![Clear bonded information sequence](ble_clear_bonded_information.png)

##Sequre connection establishment bu using save bond info(Central role)

The central role application encrypts connection link by using saved bond information.

![Secure conenction establishment by using saved bond info(central role)](ble_secure_connection_central.png)

##Sequre connection establishment bu using save bond info(Peripheral role)

The Peripheral role application encrypts connection link by using saved bond information.

![Secure conenction establishment by using saved bond info(peripheral role)](ble_secure_connection_peripheral.png)

---

# Examples  {#ble_examples}

## Application examples  {#ble_sample_apps}
- [sony_apps/examples/ble_peripheral/ble_peripheral_main.c](\ref examples/ble_peripheral/ble_peripheral_main.c)

\example examples/ble_peripheral/ble_peripheral_main.c
\example examples/ble_beacon/ble_beacon_main.c
\example examples/ble_beacon_scanner/ble_beacon_scanner_main.c
\example examples/ble_central/ble_central_main.c

---
