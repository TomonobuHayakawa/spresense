Bluetooth {#bt_doc}
-------------------------------------
[TOC]

This SDK supports Bluetooth function.

# Supported device: {#bt_Supported_device}
- BCM20706(Cypress)

---

# Key features: {#bt_key_features}

- Discoverable and connectable
    - Make it discoverable  and connectablefrom the remote device
- Device inquiry
    - Search for surrounding Bluetooth devices
- Local Device setting
    - Include friendly name, local address and so on
- Pairing
    - Pin code and different pairing modes of SSP
- Authenticated device manager
    - Management authenticated devices (bond information)
- Sniff mode
    - Control sniff mode
- Uart baudrate
    - Control the uart baudrate
- I2S role
    - Set the I2S role
- Supported profile
    * SPP
        * Support the following operation
            * Send SPP data
    * A2DP src
        * Support SBC codec
        * Support the following operation
            * Start play
            * Stop play
    * A2DP sink
        * Support SBC and  AAC codec
        * Support the following operation
            * Set the codec
            * Enable/disable AAC
    * AVRCP(CT)
        * Support the following operation
            * Play
            * Stop
            * Pause
            * Fast forward
            * Rewind
            * Next track
            * Previous track
            * Volume up
            * volume down
            * Set repeat mode
            * Set shuffule mode
            * Notify absolute volume
            * Get the track information
    * AVRCP(TG)
        * Support the following operation
            * Volume up
            * Volume down
            * Set absolute volume
            * Send player status
    * HFP
        * Support the following operation
            * Create audio connection
            * Distroy audio connection
            * Send supported AT command
                * BT_HF_AT_COMMAND_SPK
                * BT_HF_AT_COMMAND_MIC
                * BT_HF_AT_COMMAND_A
                * BT_HF_AT_COMMAND_BINP
                * BT_HF_AT_COMMAND_BVRA
                * BT_HF_AT_COMMAND_BLDN
                * BT_HF_AT_COMMAND_CHLD
                * BT_HF_AT_COMMAND_CHUP
                * BT_HF_AT_COMMAND_CIND
                * BT_HF_AT_COMMAND_CNUM
                * BT_HF_AT_COMMAND_D
                * BT_HF_AT_COMMAND_NREC
                * BT_HF_AT_COMMAND_VTS
                * BT_HF_AT_COMMAND_CMEE
                * BT_HF_AT_COMMAND_CLCC
                * BT_HF_AT_COMMAND_UNAT
            * Receive command response
                * HF_INCOMING_CALL
                * HF_SPEAKER_GAIN
                * HF_MICROPHONE_GAIN
                * HF_AG_INDICATORS
                * HF_VOICE_RECOGNITION_STATUS
                * HF_CODEC_SELECTION
    * HSP
        * Support the following operation
            * Send button press command
            * Send AT command
                * BT_HF_AT_COMMAND_SPK
                * BT_HF_AT_COMMAND_MIC
            * Receive command response
                * HF_INCOMING_CALL
    * AG
        * Support the following operation
            * Create audio connection
            * Distroy audio connection


---

# Detail description for features {#bt_Detail_description}

## Discoverable and Connectable

Set visibility of BT devices. If BT device is set to discoverable, it can be inquiried by nearby devices.
If BT device is set to connectable, it can be connected by nearby devices.

## Device inquiry

We do inquiry to discover surrounding Bluetooth devices. Inquiry will be automatically stopped in the specified time duration or the specified the number of remote devices are found. It is also possible to cancel manually.
Below is a list of remote device information obtained by inquiry.
- BD_ADDR
- RSSI
- Enhanced Inquiry Response
the BT_ADDR endian of inquiry is differnt with other events and commands in SDK2.0, we will modify this in SDK2.1.

## Local Device setting

Set and get local BT include friendly name, local address and so on.

## Pairing

If device doesn’t have the link key, pairing is required when connecting to a remote device .The way to pair is performed automatically according to the reported status of the other party.
For the pairing method, the following is supported.
- Pin code
- SSP Just Works
- SSP Numeric Comparison
- SSP Passkey Entry (Input)
- SSP Passkey Entry (Display)

## Authenticated device manager

Store and restore the bond information.

## Sniff mode

- Control the sniff mode paramter
- Enable/disable the sniff mode

## Uart baudrate

Set the UAR baudrate between Cxd56xx and BCM20706

## I2S role

Set I2S role of BCM20706

## Supported profile
- SPP
    * Connect and disconnect SPP with specify device by peer address.
    * Send and receive SPP data.
- A2DP src
    * Connect and disconnect A2DP SRC with specify device by peer address.
    * Start and stop play with specify codec type.
- A2DP sink
    * Connect and disconnect A2DP SINK with specify device by peer address.
    * Start and stop play with specify codec type.
- AVRCP(CT)
    * Connect and disconnect AVRCP with specify device by peer address.
    * Control audio connection.
- AVRCP(TG)
    * Connect and disconnect AVRCP with specify device by peer address.
- HFP
    * Connect and disconnect HFP with specify device by peer address.
    * Control audio connection.
- HSP
    * Connect and disconnect HSP with specify device by peer address.
- AG
    * Connect and disconnect AG with specify device by peer address.
    * Control audio connection.


---

# API Sequence {#bt_API_sequence}

## Initialization
Before your application can communicate over Bluetooth, you need to initialize the BT module. This operation is accomplished in two steps:
1.	Initialization protocol stack;
2.	Set the event callback function.

![Initialization sequence](bt/bt_init.png)

##Configuration

In your application, may need to configure the BT address and name.

![Configuration sequence](bt/bt_config.png)

##Visibility

BT devices can switch the visibility mode.
If the mode is discoverable and connectable, local device can be found and be connected by nearby devices.
If the mode is connectable, local device can be connected by nearby devices.
If the mode is none, local device can’t be found and be connected by nearby devices.
The mode discoverable only can’t be used.


![Visibility sequence](bt/bt_set_visibility.png)

##Finalization

If you don’t need to use the BT module anymore, you could finalize the module.

![Finalization sequence](bt/bt_finalization.png)

##Inquiry

BT device can inquiry to notice the nearby devices until timeout. The process also can be cancelled by user.

![Inquiry sequence](bt/bt_inquiry.png)

##Bond

BT device can bond with other devices, it means just pairing only. The pairing process will also be established.

![Bond sequence](bt/bt_bond.png)

##Unbond

BT device also can delete bond information, it means two devices should pairing again.

![Unbond sequence](bt/bt_unbond.png)

##Sniff mode

Enable/disable the sniff mode and set the sniff mode parameter

![Sniff mode sequence](bt/bt_set_sniff_mode.png)

##Uart baudrate

You can change the UART baud rate between Cxd56xx and BCM20706

![Uart baudrate sequence](bt/bt_set_uart_baudrate.png)

##I2S role

You can change the I2S role of BCM20706

![Unbond sequence](bt/bt_set_pcm_i2s_role.png)

##SPP connect

SPP connection can be established by local device and peer device.

![SPP connect sequence](bt/bt_sppconnect.png)

##SPP disconnect

SPP connection can be destroyed by local device and peer device.

![SPP disconnect sequnece](bt/bt_sppdisconnect.png)

##SPP is connected

SPP connection can be established by peer device.

![SPP is connected sequence](bt/bt_spp_is_connected.png)

##SPP is disconnected

SPP disconnection can be removed by peer device.

![SPP is disconnected sequence](bt/bt_spp_is_disconnected.png)

##SPP send data

If SPP continues send data, user should send data until receive SPP_TX_COMPLETE event.

![SPP send data sequence](bt/bt_sppsenddata.png)

##SPP receive data

SPP receive data from peer device.

![SPP receive data sequence](bt/bt_sppreceivedata.png)

##A2DP SRC connect

A2DP SRC connection can be established by local device and peer device.

![A2DP SRC connect sequence](bt/bt_a2dp_src_connect.png)

##A2DP SRC disconnect

A2DP SRC connection can be destroyed by local device and peer device.

![A2DP SRC disconnect sequence](bt/bt_a2dp_src_disconnect.png)

##A2DP SRC is connected

A2DP SRC connection can be established by peer device.

![A2DP SRC is connected sequence](bt/bt_a2dp_src_is_connected.png)

##A2DP SRC is disconnected

A2DP SRC disconnection can be destroyed by peer device.

![A2DP SRC is disconnected sequence](bt/bt_a2dp_src_is_disconnected.png)

##A2DP SRC start play

Before send audio data, A2DP should start play. If codec type is not SBC, please use other APIs.

![A2DP SRC start play sequence](bt/bt_a2dp_src_play.png)

##A2DP SRC stop play

![A2DP sRC stop play sequence](bt/bt_a2dp_src_stop.png)

##A2DP SRC is started play

A2DP audio connection is established by peer device

![A2DP SRC is started play sequence](bt/bt_a2dp_src_is_played.png)

##A2DP SRC is stopped play

![A2DP sRC is stopped play sequence](bt/bt_a2dp_src_is_stopped.png)

##A2DP SINK connect

A2DP SINK connection can be established by local device and peer device.

![A2DP SINK connect sequence](bt/bt_a2dp_sink_connect.png)

##A2DP SINK disconnect

A2DP SINK connection can be destroyed by local device and peer device.

![A2DP SINK disconnect sequence](bt/bt_a2dp_sink_disconnect.png)

##A2DP SINK is connected

A2DP SINK connection can be established by peer device.

![A2DP SINK is connected sequence](bt/bt_a2dp_sink_is_connected.png)

##A2DP SINK is disconnected

A2DP SINK connection can be destroyed by peer device.

![A2DP SINK disconnect sequence](bt/bt_a2dp_sink_is_disconnected.png)

##A2DP SINK start

A2DP SINK audio can be started by local device.

![A2DP SINK audio start sequence](bt/bt_a2dp_sink_start.png)

##A2DP SINK stop

A2DP SINK audio can be stopped by local device.

![A2DP SINK audio stop sequence](bt/bt_a2dp_sink_stop.png)

##A2DP SINK is started

A2DP SINK audio can be started by peer device.

![A2DP SINK is started sequence](bt/bt_a2dp_sink_is_started.png)

##A2DP SINK is stopped

A2DP SINK audio can be stopped by peer device.

![A2DP SINK is stopped sequence](bt/bt_a2dp_sink_is_stopped.png)

##A2DP SINK enable AAC

A2DP SINK AAC can be enabled or disabled

![A2DP SINK enable AAC sequence](bt/bt_a2dp_sink_enable_aac.png)

##A2DP SINK set codec

A2DP SINK codec can be set.

![A2DP SINK set codec sequence](bt/bt_a2dp_sink_set_codec.png)

##AVRCP control connect

AVRC control connection can be established by local device and peer device.

![AVRCP control connect sequence](bt/bt_avrcp_ctrl_connect.png)

##AVRCP control disconnect

AVRC control connection can be destroyed by local device and peer device.

![AVRCP control disconnect sequence](bt/bt_avrcp_ctrl_disconnect.png)

##AVRCP control is connected

AVRC control connection can be established by peer device.

![AVRCP control is connected sequence](bt/bt_avrcp_ctrl_is_connected.png)

##AVRCP control is disconnected

AVRC control connection can be destroyed by peer device.

![AVRCP control is disconnected sequence](bt/bt_avrcp_ctrl_is_disconnected.png)

##AVRCP control send command

![AVRCP control send command sequence](bt/bt_avrcp_ctrl_send_command.png)

##AVRCP control get track info

![AVRCP control get track info sequence](bt/bt_avrcp_ctrl_get_track.png)

##AVRCP control set volume

![AVRCP control set volume sequence](bt/bt_avrcp_ctrl_setvolume.png)

##AVRCP control notify volume

![AVRCP control notify volume sequence](bt/bt_avrcp_ctrl_notify_volume.png)

##AVRCP control receive volume up/down command

![AVRCP control receive volume up/down  sequence](bt/bt_avrcp_ctrl_receive_volume_up_down.png)

##AVRCP control receive absolute volume

![AVRCP control receive absolute volume sequence](bt/bt_avrcp_ctrl_receive_absolute_volume.png)

##AVRCP target connect

AVRC target connection can be established by local device and peer device.

![AVRCP target connect sequence](bt/bt_avrcp_target_connect.png)

##AVRCP target disconnect

AVRC target connection can be destroyed by local device and peer device.

![AVRCP target disconnection sequence](bt/bt_avrcp_target_disconnect.png)

##AVRCP target is connected

AVRC target connection can be established by peer device.

![AVRCP target is connected sequence](bt/bt_avrcp_target_is_connected.png)

##AVRCP target is disconnected

AVRC target connection can be destroyed by peer device.

![AVRCP target is disconnected sequence](bt/bt_avrcp_target_is_disconnected.png)

##AVRCP target send play status

![AVRCP target send play status sequence](bt/bt_avrcp_target_send_play_status.png)

##AVRCP target set volume

![AVRCP target set volume sequence](bt/bt_avrcp_target_set_volume.png)

##AVRCP target volume up/down

![AVRCP target volume up/down  sequence](bt/bt_avrcp_target_volume_up_down.png)

##HFP control connect

![HFP control connect sequence](bt/bt_hfp_connect.png)

##HFP control disconnect

![HFP control disconnect sequence](bt/bt_hfp_disconnect.png)

##HFP control is connected

![HFP control is connected sequence](bt/bt_hfp_is_connected.png)

##HFP control is disconnected

![HFP control is disconnected sequence](bt/bt_hfp_is_disconnected.png)

##HFP audio connect

![HFP audio connect sequence](bt/bt_hfp_connect_audio.png)

##HFP audio disconnect

![HFP audio disconnect sequence](bt/bt_hfp_disconnect_audio.png)

##HFP audio is connected

![HFP audio is connected sequence](bt/bt_hfp_audio_is_connected.png)

##HFP audio is disconnected

![HFP audio is disconnected sequence](bt/bt_hfp_audio_is_disconnected.png)

##HFP send AT command

![HFP send AT command sequence](bt/bt_hfp_send_at_command.png)

##HFP recevie response

![HFP receive response sequence](bt/bt_hfp_receive_response.png)

##HFP set supported features

![HFP set supported features sequence](bt/bt_hfp_set_features.png)

##HSP control connect

![HSP control connect sequence](bt/bt_hsp_connect.png)

##HSP control disconnect

![HSP control disconnect sequence](bt/bt_hsp_disconnect.png)

##HSP control is connected

![HSP control is connected sequence](bt/bt_hsp_is_connected.png)

##HSP control is disconnected

![HSP control is disconnected sequence](bt/bt_hsp_is_disconnected.png)

##HSP button press

![HSP button press sequence](bt/bt_hsp_button_press.png)

##HSP audio is connected

![HSP audio is connected sequence](bt/bt_hsp_audio_is_connected.png)

##HSP audio is disconnected

![HSP audio is disconnected sequence](bt/bt_hsp_audio_is_disconnected.png)

##HSP send AT command

![HSP send AT command sequence](bt/bt_hsp_send_at_command.png)

##HSP recevie response

![HSP receive response sequence](bt/bt_hsp_receive_response.png)

##AG control connect

![AG control connect sequence](bt/bt_ag_connect.png)

##AG control disconnect

![AG control disconnect sequence](bt/bt_ag_disconnect.png)

##AG control is connected

![AG control is connected sequence](bt/bt_ag_is_connected.png)

##AG control is disconnected

![AG control is disconnected sequence](bt/bt_ag_is_disconnected.png)

##AG audio connect

![AG audio connect sequence](bt/bt_ag_connect_audio.png)

##AG audio disconnect

![AG audio disconnect sequence](bt/bt_ag_disconnect_audio.png)

##AG audio is connected

![AG audio is connected sequence](bt/bt_ag_audio_is_connected.png)

##AG audio is disconnected

![AG audio is disconnected sequence](bt/bt_ag_audio_is_disconnected.png)

##AG receive AT command

![AG receive AT command sequence](bt/bt_ag_receive_at_command.png)

---

# Examples  {#bt_examples}

## Application examples  {#bt_sample_apps}
- [sony_apps/examples/bt_spp/bt_spp_main.c](\ref examples/bt_spp/bt_spp_main.c)

\example examples/bt_spp/bt_spp_main.c

## Application examples  {#bt_sample_apps}
- [sony_apps/examples/bt_spp_spcmd/bt_spp_spcmd_main.c](\ref examples/bt_spp_spcmd/bt_spp_spcmd_main.c)

\example examples/bt_spp_spcmd/bt_spp_spcmd_main.c

## Application examples  {#bt_sample_apps}
- [sony_apps/examples/bt_a2dp_snk/bt_a2dp_snk_main.c](\ref examples/bt_spp/bt_spp_main.c)

\example examples/bt_a2dp_snk/bt_a2dp_snk_main.c

## Application examples  {#bt_sample_apps}
- [sony_apps/examples/bt_a2dp_snk_with_decode/bt_a2dp_snk_with_decode_main.c](\ref examples/bt_a2dp_snk_with_decode/bt_a2dp_snk_with_decode_main.c)

\example examples/bt_a2dp_snk_with_decode/bt_a2dp_snk_with_decode_main.c

## Application examples  {#bt_sample_apps}
- [sony_apps/examples/bt_a2dp_src/bt_a2dp_src_main.c](\ref examples/bt_a2dp_src/bt_a2dp_src_main.c)

\example examples/bt_a2dp_src/bt_a2dp_src_main.c

## Application examples  {#bt_sample_apps}
- [sony_apps/examples/bt_hfp_hf/bt_hfp_hf_main.c](\ref examples/bt_hfp_hf/bt_hfp_hf_main.c)

\example examples/bt_hfp_hf/bt_hfp_hf_main.c

## Application examples  {#bt_sample_apps}
- [sony_apps/examples/bt_hfp_ag/bt_hfp_ag_main.c](\ref examples/bt_hfp_ag/bt_hfp_ag_main.c)

\example examples/bt_hfp_ag/bt_hfp_ag_main.c

---
