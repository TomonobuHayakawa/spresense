Audio Sub-System {#audio_doc}
============================
[TOC]

# General # {#spritzer_audio_general}

CXD5602 has audio sub system for audio feature.

The roughly list of Audio Sub-System block functions are described below.

 - Audio Codec hardware (AD/DA, DNC, DEQ, etc.) control <br>
 - Audio Player function <br>
 - Audio Recorder function <br>
 - Bluetooth related function(for BT-A2DP) <br>
 - Voice Call function <br>
 - Voice recognition function <br>
( - Sound quality enhancement process) <br>
( - Environment sound sensing) <br>

This document is the specification describes the software specifications related to Spritzer Audio block control. <br>
The Audio block control API has the two accessible layers of High Level specifications and Low Level specifications. <br>

High Level specifications is provided by "Audio Manager" on Audio Utility Libraries. <br>
Low Level specifications can be called by driver layer is provided from nuttX`s API like "/dev/audio".. <br>
(But, Low Level specifications is under constructions.) <br>


This document describes the specifications for both layers.

--------------------------------------
# High Level API # {#spritzer_audio_high_level_api}

High Level API can use from Audio Manager in Audio Utitity.


Below is a stack diagram of Audio Subsystem.

![Audio Stack diagram.](AudioStack.png)

Audio Subsystem can be controlled by following High-Level API Command System.\n
To send a command to Audio SubSystem, use [AS_SendAudioCommand](@ref AS_SendAudioCommand) and command format is [AudioCommand](@ref AudioCommand).\n
Audio SubSystem will return result. To receive them, use [AS_ReceiveAudioResult](@ref AS_ReceiveAudioResult), result format is [AudioResult](@ref AudioResult).\n

**To use Audio SubSystem via audio_manager, keep send-receive sequene per 1 command.**

![Audio Hi-Level API Command System.](Audio_HighLevel_Command_System.png)


- @ref audioutils_audio_high_level_api

- refer to <a href="../CXD5602-SDK-AudioSubSystem-spec-v2.27.pdf">"CXD5602-SDK-AudioSubSystem-spec-v2.27.pdf"</a>


--------------------------------------
## Each Functions #     {#spritzer_audio_functions}

### Audio Player Functions#     {#spritzer_audio_player_functions}

We provide Audio Player Function with effect sounds.

A simple data flow on Audio Player is shown below.

![Audio Player Dataflow.](Audio_player_dataflow.png)

#### How to use # {#spritzer_audio_player_functions_howtouse}

Here discribe how to use audio player function in typical case.

##### Preparation

Software component which called AudioManager, playerObject, OutputmixObject, RenderComponent in Audio SubSystem are designed to work as "task".<br>
So, to use them by application, call activate function of them.<br>

- Activate AudioManager.<br>
   Activate by caling AS_ActivateAudioSubSystem(AudioSubSystemIDs).<br>
   In the structure AudioSubSystemIDs of the argument, specify the MsgQueID defined in the Message Library Configuration.<br>
   Specify 0xFF for Object which is not used.<br>

~~~
AudioSubSystemIDs ids;

ids.app         = MSGQ_AUD_APP;
ids.mng         = MSGQ_AUD_MGR;
ids.player_main = MSGQ_AUD_PLY;
ids.player_sub  = MSGQ_AUD_SFX;
ids.mixer       = MSGQ_AUD_OUTPUT_MIX;
ids.recoder     = 0xFF;
ids.effector    = 0xFF;
ids.recognizer  = 0xFF;

AS_ActivateAudioSubSystem(ids);
~~~

- Activate PlayerObject.<br>
   Activate by caling AS_ActivatePlayer().<br>
   Specify the MsgQueID defined in the Message Library Configuration and the PoolId defined int the Memory Manager Configuration as arguments.<br>
   
~~~
bool act_rst = AS_ActivatePlayer(
        MSGQ_AUD_PLY,
        MSGQ_AUD_MGR,
        MSGQ_AUD_OUTPUT_MIX,
        MSGQ_AUD_DSP,
        DEC_ES_MAIN_BUF_POOL,
        REND_PCM_BUF_POOL,
        DEC_APU_CMD_POOL
      );
~~~

- Activate PlayerObject for SoundFx.<br>
   Activate by caling AS_ActivateSfxPlayer().<br>
   Specify the MsgQueID defined in the Message Library Configuration and the PoolId defined int the Memory Manager Configuration as arguments.<br>
   
~~~
bool act_rst = AS_ActivateSfxPlayer(
        MSGQ_AUD_SFX,
        MSGQ_AUD_MGR,
        MSGQ_AUD_OUTPUT_MIX,
        MSGQ_AUD_DSP,
        DEC_ES_SUB_BUF_POOL,
        REND_PCM_SUB_BUF_POOL,
        DEC_APU_CMD_POOL
      );
~~~

- Activate OutputmixObject.<br>
   Activate by caling AS_ActivateOutputMix().<br>
   Specify the MsgQueID defined in the Message Library Configuration and the PoolId defined int the Memory Manager Configuration as arguments.<br>

~~~
bool act_rst = AS_ActivateOutputMix(
        MSGQ_AUD_OUTPUT_MIX,
        MSGQ_AUD_MGR
        );
~~~

- Activate RenderComponent.<br>
   Activate by caling AS_ActivateRenderer().<br>
   Specify the MsgQueID defined in the Message Library Configuration and the PoolId defined int the Memory Manager Configuration as arguments.<br>
   if application don't use SoundFX, specify 0xFF for the 3rd and 4th arguments.<br>

~~~
bool act_rst = AS_ActivateRenderer(
        MSGQ_AUD_RND_PLY,
        MSGQ_AUD_RND_PLY_SYNC,
        MSGQ_AUD_RND_SFX,
        MSGQ_AUD_RND_SFX_SYNC
        );
~~~

##### Initialize

1. Power on Audio SubSystem.<br>
  [AUDCMD_POWERON](@ref AUDCMD_POWERON)<br>
  <br> 
2. Set output.<br> 
  If you would like to output to Speaker, send [AUDCMD_INITOUTPUTSELECT](@ref AUDCMD_INITOUTPUTSELECT), [InitOutputSelectParam](@ref InitOutputSelectParam) command.<br>
  If I2S output is reuired, send [AUDCMD_INITI2SPARAM](@ref AUDCMD_INITI2SPARAM), [InitI2SParam](@ref InitI2SParam) insted of above.<br>
  <br> 
3. Set AudioSubSystem transition to Player Status.<br> 
  Select a player(Main, Sub, or both of them) which you'd like to actvite.<br>
  If you would like to use SimpleFIFO, the handle of them should be set.<br>
  [AUDCMD_SETPLAYERSTATUS](@ref AUDCMD_SETPLAYERSTATUS), [SetPlayerStsParam](@ref SetPlayerStsParam)<br>
  <br>
4. Set volume.<br> 
  If you would like to output to Speaker, set volume by [AUDCMD_SETVOLUME](@ref AUDCMD_SETVOLUME), [SetVolumeParam](@ref SetVolumeParam).<br>
  <br> 

![Initial sequence](audio_player_function_typical_sequence_initial.png)

##### Start play

1. Init (sub)player information.<br>
  Set information(codec, bit length, etc...) of music, soundeffet file.
  [AUDCMD_INITPLAYER](@ref AUDCMD_INITPLAYER)/[AUDCMD_INITSUBPLAYER](@ref AUDCMD_INITSUBPLAYER), [AsInitPlayerParam](@ref AsInitPlayerParam)<br>
  <br>
2. Start player.<br>
  [AUDCMD_PLAYPLAYER](@ref AUDCMD_PLAYPLAYER)/[AUDCMD_PLAYSUBPLAYER](@ref AUDCMD_PLAYSUBPLAYER)<br>
  <br>

![Playing sequence](audio_player_function_typical_sequence_playing.png)

##### Stop play

1. Stop player.<br>
  [AUDCMD_STOPPLAYER](@ref AUDCMD_STOPPLAYER)/[AUDCMD_STOPSUBPLAYER](@ref AUDCMD_STOPSUBPLAYER)<br>

![Stop sequence](audio_player_function_typical_sequence_stop.png)

#### Build Configurations #     {#spritzer_audio_player_highlevel_configurations}

Select options in below:
~~~
:(Select audio player application)
[System Type]
  [CXD56xx Peripheral Support]
    [Audio]                      <= Y
      [I2S0]                     <= Y

  RAM size for kernel            <= 0xA0000
  RAM size for ASMP              <= 0xE0000

[RTOS Features]
  [Tasks and Scheduling]
    [Enable waitpid() API]       <= Y
  [Stack and heap information]
    [Idle thread stack size]     <= 4096

[Memory Management]
  [Number of memory regions]     <= 2
  [Enable Tile Allocator]        <= Y

[Library Routines]
  [Standard Math library]        <= Y
  [Have C++ compiler]            <= Y
  [Have C++ initialization]      <= Y
  [ASMP support library]         <= Y

[Application Configuration]
  [Audio Utilities]
    [Audio manager]              <= Y
    [Playlist manager]           <= Y
      [playlist file mount path] <= /mnt/vfat/PLAYLIST
      [play file mount path]     <= /mnt/vfat/AUDIO
    [Audio Feature Support]
      [Audio Feature]            <= Y
      [Audio Player]             <= Y
         (Attention: If anything other than the above is selected, set 'N' to these items)
      [DSP image mount path]     <= /mnt/vfat/BIN
  [Memory Utilities]
    [Memory manager]             <= Y
    [Message]                    <= Y
    [Simple FIFO]                <= Y
    

~~~

#### Memory Utility Configurations and Layout#     {#spritzer_audio_player_memutil_configurations}

##### Message Library Configuration

~~~
MsgQuePool
 # ID,                           n_size  n_num  h_size  h_num
  ["MSGQ_AUD_MGR",               88,     3,     0,      0],
  ["MSGQ_AUD_APP",               40,     2,     0,      0],
  ["MSGQ_AUD_DSP",               20,     5,     0,      0],
  ["MSGQ_AUD_PLY",               48,     5,     0,      0],
  ["MSGQ_AUD_SUB_PLY",           48,     5,     0,      0],
  ["MSGQ_AUD_OUTPUT_MIX",        48,     8,     0,      0],
  ["MSGQ_AUD_RND_PLY",           32,    16,     0,      0],
  ["MSGQ_AUD_RND_PLY_SYNC",      16,     8,     0,      0],
  ["MSGQ_AUD_RND_SUB",           32,    16,     0,      0],
  ["MSGQ_AUD_RND_SUB_SYNC",      16,     8,     0,      0],

See "sony_apps/examples/audio/audio_player/config/msgq_layout.conf" for each definition.

If configration is changed, use tool to generate header files.
Ex) ruby -Isony_apps/memutils/message/tool sony_apps/examples/audio/audio_player/config/msgq_layout.conf 0x000fc000 0x3140 msgq_id.h msgq_pool.h

~~~

##### Memory Manager (Intelligent Fix Pool) Configuration

~~~
FixedAreas
 # name,                  device,     align,        size,         fence
  ["AUDIO_WORK_AREA",     "AUD_SRAM", U_TILE_ALIGN, 0x0003c000,   false],
  ["MSG_QUE_AREA",        "AUD_SRAM", U_MSGQ_ALIGN, 0x00003140,   false],
  ["MEMMGR_WORK_AREA",    "AUD_SRAM", U_STD_ALIGN,  0x00000200,   false],
  ["MEMMGR_DATA_AREA",    "AUD_SRAM", U_STD_ALIGN,  0x00000100,   false],

CAUTION: Fixed Areas can not be customized

PoolAreas
 # name,                    area,              type,  align,        pool-size,  seg, fence,  spinlock
  ["DEC_ES_MAIN_BUF_POOL",  "AUDIO_WORK_AREA", Basic, U_STD_ALIGN,  0x00006000, 4,   true,   ""],
  ["REND_PCM_BUF_POOL",     "AUDIO_WORK_AREA", Basic, U_STD_ALIGN,  0x00012000, 9,   true,   ""],
  ["REND_PCM_SUB_BUF_POOL", "AUDIO_WORK_AREA", Basic, U_STD_ALIGN,  0x00012000, 9,   true,   ""],
  ["DEC_APU_CMD_POOL",      "AUDIO_WORK_AREA", Basic, U_STD_ALIGN,  0x00000384, 10,  true,   ""],
  ["DEC_ES_SUB_BUF_POOL",   "AUDIO_WORK_AREA", Basic, U_STD_ALIGN,  0x00003000, 4,   true,   ""],

See "sony_apps/examples/audio/audio_player/config/mem_layout.conf" for each definition.

If configration is changed, use tool to generate header files.
Ex) ruby -Isony_apps/memutils/memory_manager/tool sony_apps/examples/audio/audio_player/config/mem_layout.conf mem_layout.h fixed_fence.h pool_layout.h

~~~

#### Error Attentions and Approche#     {#spritzer_audio_player_errors}

If you receive the following warning during audio playback, please pay attention to the following.

+ Notification content: <br>
 - module_id: AS_MODULE_ID_PLAYER_OBJ
 - error_code: AS_ERROR_CODE_WARNING
 - sub_code: AS_ATTENTION_SUB_CODE_SIMPLE_FIFO_UNDERFLOW

This warning is caused by the following.

If data on the player side can not be read from the SimpleFIFO, an "Underflow! No more data." message is displayed and attensions is notified to the application side. <br>
In this state, the player side is in a stop wait state and the application side always needs to issue a stop instruction. <br>
If the application receives a notification at an unintended timing, it is assumed that writing of data to the FIFO is slow. <br>
In order to receive attensions notification, you need to implement the InitAttentions interface of the Audio high level API. <br>


#### Builds & install #     {#spritzer_audio_player_example_build}

Type 'make' to build NuttX. <br>

You can see 'nuttx' file on top of the NuttX source tree. <br>
Then copy nuttx.spk into Corvo USB MSC. <br>
(see README.txt for more information) <br>

CAUTION: <br>
sony_apps build system cannot build automatically by configuration or/and example source modification. <br>
Please 'make clean' first. <br>

DSP binary image install: <br>
Create "BIN/" directory on the root directory of eMMC filesystem and copy DSP binary image to this directory. <br>
The DSP binary image to copy is in "sony_sony_apps/audioutils/dsp" according to configuration. <br>

- Binary image required for audio player according to configuration: <br>
  | Image   | size     |
  |:--------|:---------|
  | MP3DEC  | 118kbyte |
  | WAVDEC  | 60kbyte  |
  | AACDEC  | 223kbtye |
  | OPUSDEC | 120kbyte |

#### Audio Player Examples #   {#spritzer_audio_player_example}

We provide Audio Player example Code.

##### Build Configurations #     {#spritzer_audio_player_example_configurations}

To use Audio Player example, select configs as follow.

~~~
(audio player:)
[Application Configuration]
  [Examples]
    [Audio player example] <= Y

~~~

Attention!! Audio & Logical sensor example can select only on examle!
If you select multipule examples, it will causes build error.

- Only support the following format:
  |      | sampling rate                 | PCM bit length | channel number | CPU frequency lock |
  |:-----|:------------------------------|:---------------|:---------------|:-------------------|
  | mp3  | 16kHz / 32kHz 44.1kHz / 48kHz | 16bit          | 1ch / 2ch      | High voltage       |
  | wav  | 16kHz / 32kHz 44.1kHz / 48kHz | 16bit          | 2ch            | High voltage       |
  | aac  | 24kHz / 32kHz 44.1kHz / 48kHz | 16bit          | 1ch / 2ch      | Low voltage        |
  | opus | 8kHz  / 16kHz                 | 16bit          | 1ch            | High voltage       |

##### How to execute

- Preparation <br>
 + Music file<br>
   For the player application, you will need a music file. <br>
   Create "AUDIO/" directory on the root directory of eMMC filesystem and copy music files to this directory. <br>
   <br>

 + Playlist <br>
   A playlist is required to play music files. <br>
   In this file, we create track database with using csv data format. You can add new tracks to this file or delete some tracks from this file in specified format(*). <br>
   Create "PLAYLIST/" directory on the root directory of eMMC filesystem and copy "track_db.csv" files to this directory. <br>
    
   (*)Format for "track_db.csv" <br>
   [filename],[artist],[album],[channel number],[bit lentgh],[sampling rate],[file format],[play info] <br>
   ~~~
   ABC.mp3,artist1,album1,2,16,44100,mp3,0
   DEF.mp3,artist1,album1,2,16,48000,mp3,0
   GHI.wav,artist2,album2,2,16,16000,wav,0
   ZXC.aac,artist3,album3,2,16,48000,aac,0
   VBN.opus,artist4,album4,1,16,8000,opus,0
   .....
   ~~~
   > **notice**<br>
   > If you would not like to use playlist, deactivate playlist by config and type track info when you play tracks.<br>
   > (Playlist function is automatically enabled when audio player example is selected.)
   > ~~~
   > [Application Configration]
   >   [Audio Utilities]
   >     [Playlist Manager]     <= n
   > ~~~
   <br>

 + Soundeffect file<br>
   In addition, this application requires a file for sounding soundeffect.<br>
   Place a file which follows format below to "AUDIO/" directory.<br>
   | item          | value        |
   |:--------------|:-------------| 
   | file name     | Sfxsound.mp3 |
   | bit length    | 16bit        |
   | ch num        | Stereo       |
   | Sampling rate | 44.1kHz      |
   <br>

- Notice<br>
 > **Using SPI-Flash instead of EMMC**
 > <br>
 > <br>
 >  If you want to use SPI-Flash, for example, in case you don't have EMMC,<br>
 >  you need copy to SPI-Flash via ram disk. Therefore, please do following procedure.<br>
 >   
 >  1. Enable ramdisk function (set menu config) and build.  
 >   ~~~
 >   (menuconfig:)
 >   [Device Drivers]
 >     [USB Device Driver Support]
 >       [USB Mass storage class device]
 >         [Mass storage Product ID]        <= Change to other than default value.(ex, 0x1234)
 >
 >   [Application Configuration]
 >     [System Libraries and NSH Add-Ons]
 >       [USB Mass Storage Device Commands]
 >         [LUN1 Device Path]               <= /dev/ram1
 >     [NSH Library]
 >       [Disable Individual commands]
 >         [Disable mkfatfs]                <= n
 >   ~~~
 >   <br>
 >
 >  2. Copy files to SPI-Flash via ramdisk.<br>
 >   Create ramdisk. The size could be set by last argument.
 >   ~~~
 >   nsh>mkrd -m 1 -s 512 1024
 >   ~~~
 >   Create fat filesystem.
 >   ~~~
 >   nsh>mkfatfs /dev/ram1
 >   ~~~
 >   Connet to PC by Mass Storate.
 >   ~~~
 >   nsh>msconn
 >   ~~~
 >   <br>
 >   _Copy files(ex. MYFILE.bin) to USB device. (PC operation)_<br>
 >   <br>
 >   Mount ramdisk.
 >   ~~~
 >   nsh>mount -t vfat /dev/ram1 /tmp
 >   ~~~
 >   Copy a file to SPI-Flash.
 >   ~~~
 >   nsh>cp /tmp/MYFILE.bin /mnt/spfi/MYFILE.bin
 >   ~~~
 >   <br>
 >
 >  3. Set files(DSPs, audio files, playlist file...) path.
 >   ~~~
 >   (menuconfig:)
 >   [Application Configuration]
 >     [Audio Utilities]
 >       [Audio Feature Support]
 >         [DSP image mount path] <= /mnt/spif/??? (Any directory under spif)
 >     [playlist file mount path] <= /mnt/spif/??? (Any directory under spif)
 >     [play file mount path]     <= /mnt/spif/??? (Any directory under spif)
 >     [Examples]
 >       [Audio player menu]
 >         [play file mount path] <= /mnt/spif/??? (Any directory under spif)
 >   ~~~
 <br>

- Now, let's run the audio player application. <br>
  Turn on and boot Nuttx board. <br>
  When board startup is completed. <br>
  Please activate the audio player application with command shown in blow.
  ~~~
  nsh> player
  ~~~
  Then, player could be accept player commands.<br>

 + Type
   ~~~
   player> outDevice SPHP
   ~~~
   command, select output device. (SPHP for SP/HP device; I2S for I2S device) <br>
 + Type
   ~~~
   player> play
   ~~~
   command, then the application will start to play music files. <br>
   > **notice**<br>
   > If playlist function is disabled, track info is needed to play and it should be written in specified format as shown below.<br>
   > "[filename],[ch num],[bit length],[sampling rate],[codec]"<br>
   > Please type like below.
   > ~~~
   > player> play maymusic.mp3,2,16,44100,mp3
   > ~~~
 + Type
   ~~~
   player> stop
   ~~~
   command, then the application will stop playing the music files. <br>
 + Type
   ~~~
   player> sfxplay
   ~~~
   command, then the application will start to play soundeffect files. <br>
 + Type
   ~~~
   player> sfxstop
   ~~~
   command, then the application will stop playing the soundeffect files. <br>
 + Type
   ~~~
   player> qui
   ~~~
   command, then the application will exit audio player application. <br>


### Audio Recorder Functions#     {#spritzer_audio_recorder_functions}

We provide Audio Recorder Function.

A simple data flow on Audio Recorder is shown below.

![Audio Recorder Dataflow.](TBD)

#### How to use # {#spritzer_audio_recorder_functions_howtouse}

Here discribe how to use audio recorder function in typical case.

##### Preparation

Software component which called AudioManager, RecorderObject, CaptureComponent in Audio SubSystem are designed to work as "task".<br>
So, to use them by application, call activate function of them.<br>

- Activate AudioManager.<br>
   Activate by caling AS_ActivateAudioSubSystem(AudioSubSystemIDs).<br>
   In the structure AudioSubSystemIDs of the argument, specify the MsgQueID defined in the Message Library Configuration.<br>
   Specify 0xFF for Object which is not used.<br>

~~~
AudioSubSystemIDs ids;

ids.app         = MSGQ_AUD_APP;
ids.mng         = MSGQ_AUD_MGR;
ids.player_main = 0xFF;
ids.player_sub  = 0xFF;
ids.mixer       = 0xFF;
ids.recoder     = MSGQ_AUD_RECORDER;
ids.effector    = 0xFF;
ids.recognizer  = 0xFF;

AS_ActivateAudioSubSystem(ids);
~~~

- Activate RecorderObject.<br>
   Activate by caling AS_ActivatePlayer().<br>
   Specify the MsgQueID defined in the Message Library Configuration and the PoolId defined int the Memory Manager Configuration as arguments.<br>
   
~~~
bool act_rst = AS_ActivateVoiceRecorder(
        MSGQ_AUD_RECORDER,
        MSGQ_AUD_MEDIA_REC_SINK,
        MSGQ_AUD_MGR,
        MSGQ_AUD_DSP,
        ENC_APU_CMD_POOL,
        MIC_IN_BUF_POOL,
        OUTPUT_BUF_POOL
      );
~~~

- Activate sink feature of RecorderObject.<br>
   Activate by caling AS_ActivateVoiceRecorderSink().<br>
   Specify the MsgQueID defined in the Message Library Configuration and the PoolId defined int the Memory Manager Configuration as arguments.<br>
   
~~~
bool act_rst = AS_ActivateVoiceRecorderSink(
        MSGQ_AUD_MEDIA_REC_SINK,
        MSGQ_AUD_RECORDER
      );
~~~

- Activate CaptureComponent.<br>
   Activate by caling AS_ActivateCapture().<br>
   Specify the MsgQueID defined in the Message Library Configuration and the PoolId defined int the Memory Manager Configuration as arguments.<br>
   if application don't use 2nd input channel, specify 0xFF for the 3rd and 4th arguments.<br>

~~~
bool act_rst = AS_ActivateRenderer(
        MSGQ_AUD_CAP,
        MSGQ_AUD_CAP_SYNC,
        0xFF,
        0xFF
        );
~~~

#### Build Configurations #     {#spritzer_audio_recorder_highlevel_configurations}

Select options in below:
~~~
:(Select audio recorder application)
[System Type]
  [CXD56xx Peripheral Support]
    [Audio] <= Y
      [I2S0] <= Y
[RTOS Features]
  [Tasks and Scheduling]
    [Enable waitpid() API]  <= Y
  [Stack and heap information]
    [Idle thread stack size] <= 4096
[Memory Management]
  [Number of memory regions] <= 2
  [Enable Tile Allocator]    <= Y
[Library Routines]
  [Standard Math library] <= Y
  [Have C++ compiler] <= Y
  [Have C++ initialization] <= Y
  [ASMP support library] <= Y
[Application Configuration]
  [Audio Utilities]
    [Audio manager] <= Y
    [Audio Feature Support]
      [Audio Feature] <= Y
      [Audio Recorder] <= Y
      [Sampling Rate Converter] <= Y
         (Attention: If anything other than the above is selected, set 'N' to these items)
      [DSP image mount path] <= /mnt/vfat/BIN
  [Memory Utilities]
    [Memory manager] <= Y
    [Message] <= Y
    [Simple FIFO] <= Y

~~~

#### Memory Utility Configurations and Layout#     {#spritzer_audio_recorder_memutil_configurations}

##### Message Library Configuration

~~~
MsgQuePool
 # ID,                           n_size  n_num  h_size  h_num
  ["MSGQ_AUD_MGR",               88,     3,     0,      0],
  ["MSGQ_AUD_APP",               40,     2,     0,      0],
  ["MSGQ_AUD_DSP",               20,     5,     0,      0],
  ["MSGQ_AUD_RECORDER",          48,     5,     0,      0],
  ["MSGQ_AUD_MEDIA_REC_SINK",    36,    17,     0,      0],
  ["MSGQ_AUD_CAP",               24,    16,     0,      0],
  ["MSGQ_AUD_CAP_SYNC",          16,     8,     0,      0],

See "sony_apps/examples/audio/audio_recorder/config/msgq_layout.conf" for each definition.

If configration is changed, use tool to generate header files.
Ex) ruby -Isony_apps/memutils/message/tool sony_apps/examples/audio/audio_recorder/config/msgq_layout.conf 0x000fc000 0x3140 msgq_id.h msgq_pool.h

~~~

##### Memory Manager (Intelligent Fix Pool) Configuration

~~~
FixedAreas
 # name,                  device,     align,        size,         fence
  ["AUDIO_WORK_AREA",     "AUD_SRAM", U_TILE_ALIGN, 0x0003c000,   false],
  ["MSG_QUE_AREA",        "AUD_SRAM", U_MSGQ_ALIGN, 0x00003140,   false],
  ["MEMMGR_WORK_AREA",    "AUD_SRAM", U_STD_ALIGN,  0x00000200,   false],
  ["MEMMGR_DATA_AREA",    "AUD_SRAM", U_STD_ALIGN,  0x00000100,   false],

CAUTION: Fixed Areas can not be customized

PoolAreas
 # name,                    area,              type,  align,        pool-size,  seg, fence,  spinlock
  ["OUTPUT_BUF_POOL",       "AUDIO_WORK_AREA", Basic, U_STD_ALIGN,  0x00008700, 5,   true,   ""],
  ["MIC_IN_BUF_POOL",       "AUDIO_WORK_AREA", Basic, U_STD_ALIGN,  0x00003000, 2,   true,   ""],
  ["ENC_APU_CMD_POOL",      "AUDIO_WORK_AREA", Basic, U_STD_ALIGN,  0x000000FC, 3,   true,   ""],
  ["SRC_APU_CMD_POOL",      "AUDIO_WORK_AREA", Basic, U_STD_ALIGN,  0x000000FC, 3,   true,   ""],

See "sony_apps/examples/audio/audio_recorder/config/mem_layout.conf" for each definition.

If configration is changed, use tool to generate header files.
Ex) ruby -Isony_apps/memutils/memory_manager/tool sony_apps/examples/audio/audio_recorder/config/mem_layout.conf mem_layout.h fixed_fence.h pool_layout.h

~~~

#### Error Attentions and Approche#     {#spritzer_audio_recorder_errors}

If you receive the following warning during audio playback, please pay attention to the following.

+ Notification content: <br>
 - module_id: AS_MODULE_ID_MEDIA_RECORDER_OBJ
 - error_code: AS_ERROR_CODE_WARNING
 - sub_code: AS_ATTENTION_SUB_CODE_SIMPLE_FIFO_OVERFLOW

This warning is caused by the following.

If data can not be written to the SimpleFIFO on the recorder side, "SimpleFIFO overflow" message is displayed and attensions is notified to the application side. <br>
In this state, the recorder side is in a stop wait state and the application side always needs to issue a stop instruction. <br>
If the application receives a notification at an unintended timing, it is assumed that reading of data from the SimpleFIFO is slow. <br>
In order to receive attensions notification, you need to implement the InitAttentions interface of the Audio high level API. <br>


#### Builds & install #     {#spritzer_audio_recorder_example_build}

Type 'make' to build NuttX. <br>

You can see 'nuttx' file on top of the NuttX source tree. <br>
Then copy nuttx.spk into Corvo USB MSC. <br>
(see README.txt for more information) <br>

CAUTION: <br>
sony_apps build system cannot build automatically by configuration or/and example source modification. <br>
Please 'make clean' first. <br>

DSP binary image install: <br>
Create "BIN/" directory on the root directory of eMMC filesystem and copy DSP binary image to this directory. <br>
The DSP binary image to copy is in "sony_sony_apps/audioutils/dsp" according to configuration. <br>

- Binary image required for audio recorder according to configuration: <br>
 + "MP3ENC"
 + "SRC"
 + "OPUSENC"

- Binary image size loaded <br>
 + MP3ENC  : 264kbyte
 + SRC     : 84kbyte
 + OPUSENC : 249kbtye

#### Audio Recorder Examples #   {#spritzer_audio_recorder_example}

We provide Audio Recorder example Code.

##### Build Configurations #     {#spritzer_audio_recorder_example_configurations}

You will select as flow.

~~~
(audio recorder:)
[Application Configuration]
  [Examples]
    [Audio recorder example] <= Y

~~~
Attention!! Audio & Logcal sensor example can select only on examle!
If you select multipule examples, you cannot build.

- Only support the following format:
 + Input device: Analog-mic / I2S <br>
 + Input channel: <br>
     - Analog-mic  : 1ch / 2ch <br>
     - I2S         : 2ch <br>
 + Sampling rate of recording data: <br>
     - mp3  : 16kHz / 48kHz <br>
     - wav  : 16kHz / 48kHz <br>
     - opus : 8kHz <br>
 + File format: mp3 / wav / opus <br>
 + File name: <br>
     - mp3  : "YYYYMMDD_HHMMSS.mp3" <br>
     - wav  : "YYYYMMDD_HHMMSS.wav" <br>
     - opus : "YYYYMMDD_HHMMSS.opus" <br>

How to execute
- Now, let's run the audio recorder application. <br>
  Turn on and boot Nuttx board. <br>
  Please activate the audio recorder application when board startup is completed. <br>
 nsh>recorder <br>

 + Type <br>
        recorder> inDevice AMIC <br>
        command, select input device. (AMIC for Analog-Mic device; I2S for I2S device) <br>
 + Type <br>
        recorder> setRecInfo mp3,16,2 <br>
        command, set recording information. <br>
        file format(mp3 for mp3 format; wav for wav format),sampling rate(16 for 16kHz; 48 for 48kHz),input channel(1 for 1ch mic; 2 for 2ch mic or i2s) <br>
 + Type <br>
        recorder> rec <br>
        command, then the application will start to record data from eMMC filesystem. <br>
 + Type <br>
        recorder> stop <br>
        command, then the application will stop recording. <br>
 + Type <br>
        recorder> quit <br>
        command, then the application will exit audio recorder application. <br>
 nsh>

- Recording data. <br>
  The "REC/" directory is created and stored in the root of the eMMC filesystem as the recording data. <br>


### Audio Voice Call Functions#     {#spritzer_audio_voice_call_functions}

We provide Audio Voice Call Function.

A simple data flow on Audio Voice Call is shown below.

![Audio Voice Call Dataflow.](TBD)

#### How to use # {#spritzer_audio_voice_call_functions_howtouse}

Here discribe how to use voice call function in typical case.

##### Preparation

Software component which called AudioManager, EffectorObject, RenderComponent, CaptureComponent in Audio SubSystem are designed to work as "task".<br>
So, to use them by application, call activate function of them.<br>

- Activate AudioManager.<br>
   Activate by caling AS_ActivateAudioSubSystem(AudioSubSystemIDs).<br>
   In the structure AudioSubSystemIDs of the argument, specify the MsgQueID defined in the Message Library Configuration.<br>
   Specify 0xFF for Object which is not used.<br>

~~~
AudioSubSystemIDs ids;

ids.app         = MSGQ_AUD_APP;
ids.mng         = MSGQ_AUD_MGR;
ids.player_main = 0xFF;
ids.player_sub  = 0xFF;
ids.mixer       = MSGQ_AUD_OUTPUT_MIX;
ids.recoder     = 0xFF;
ids.effector    = MSGQ_AUD_SOUND_EFFECT;
ids.recognizer  = 0xFF;

AS_ActivateAudioSubSystem(ids);
~~~

- Activate EffectorObject.<br>
   Activate by caling AS_ActivateEffector().<br>
   Specify the MsgQueID defined in the Message Library Configuration and the PoolId defined int the Memory Manager Configuration as arguments.<br>
   
~~~
bool act_rst = AS_ActivateEffector(
        MSGQ_AUD_SOUND_EFFECT,
        MSGQ_AUD_MGR,
        0xFF,
        MSGQ_AUD_DSP,
        MIC_IN_BUF_POOL,
        I2S_IN_BUF_POOL,
        HP_OUT_BUF_POOL,
        I2S_OUT_BUF_POOL,
        MFE_OUT_BUF_POOL
      );
~~~

- Activate RenderComponent.<br>
   Activate by caling AS_ActivateRenderer().<br>
   Specify the MsgQueID defined in the Message Library Configuration and the PoolId defined int the Memory Manager Configuration as arguments.<br>

~~~
bool act_rst = AS_ActivateRenderer(
        MSGQ_AUD_RND_SPHP,
        MSGQ_AUD_RND_SPHP_SYNC,
        MSGQ_AUD_RND_I2S,
        MSGQ_AUD_RND_I2S_SYNC
        );
~~~

- Activate CaptureComponent.<br>
   Activate by caling AS_ActivateCapture().<br>
   Specify the MsgQueID defined in the Message Library Configuration and the PoolId defined int the Memory Manager Configuration as arguments.<br>

~~~
bool act_rst = AS_ActivateRenderer(
        MSGQ_AUD_CAP_MIC,
        MSGQ_AUD_CAP_MIC_SYNC,
        MSGQ_AUD_CAP_I2S,
        MSGQ_AUD_CAP_I2S_SYNC
        );
~~~

#### Build Configurations #     {#spritzer_audio_voice_call_highlevel_configurations}

Select options in below:
~~~
:(Select audio voice call application)
[System Type]
  [CXD56xx Peripheral Support]
    [Audio] <= Y
      [I2S0] <= Y
[RTOS Features]
  [Tasks and Scheduling]
    [Enable waitpid() API]  <= Y
  [Stack and heap information]
    [Idle thread stack size] <= 4096
[Memory Management]
  [Number of memory regions] <= 2
  [Enable Tile Allocator]    <= Y
[Library Routines]
  [Standard Math library] <= Y
  [Have C++ compiler] <= Y
  [Have C++ initialization] <= Y
  [ASMP support library] <= Y
[Application Configuration]
  [Audio Utilities]
    [Audio manager] <= Y
    [Audio Feature Support]
      [Audio Feature] <= Y
      [Voice Call] <= Y
      [Mic Front End] <= Y
      [Media Player Post] <= Y
         (Attention: If anything other than the above is selected, set 'N' to these items)
  [Memory Utilities]
    [Memory manager] <= Y
    [Message] <= Y
    [Simple FIFO] <= Y

~~~

#### Memory Utility Configurations and Layout#     {#spritzer_audio_voice_call_memutil_configurations}

##### Message Library Configuration

~~~
MsgQuePool
 # ID,                           n_size  n_num  h_size  h_num
  ["MSGQ_AUD_MGR",               88,     3,     0,      0],
  ["MSGQ_AUD_APP",               40,     2,     0,      0],
  ["MSGQ_AUD_DSP",               20,     5,     0,      0],
  ["MSGQ_AUD_OUTPUT_MIX",        48,     8,     0,      0],
  ["MSGQ_AUD_SOUND_EFFECT",      52,     5,     0,      0],
  ["MSGQ_AUD_RCG_CMD",           20,     5,     0,      0],
  ["MSGQ_AUD_CAP_MIC",           24,    16,     0,      0],
  ["MSGQ_AUD_CAP_MIC_SYNC",      16,     8,     0,      0],
  ["MSGQ_AUD_CAP_I2S",           24,    16,     0,      0],
  ["MSGQ_AUD_CAP_I2S_SYNC",      16,     8,     0,      0],
  ["MSGQ_AUD_RND_SPHP",          32,    16,     0,      0],
  ["MSGQ_AUD_RND_SPHP_SYNC",     16,     8,     0,      0],
  ["MSGQ_AUD_RND_I2S",           32,    16,     0,      0],
  ["MSGQ_AUD_RND_I2S_SYNC",      16,     8,     0,      0],

See "sony_apps/examples/audio/voice_call/config/msgq_layout.conf" for each definition.

If configration is changed, use tool to generate header files.
Ex) ruby -Isony_apps/memutils/message/tool sony_apps/examples/audio/voice_call/config/msgq_layout.conf 0x000fc000 0x3140 msgq_id.h msgq_pool.h

~~~

##### Memory Manager (Intelligent Fix Pool) Configuration

~~~
FixedAreas
 # name,                  device,     align,        size,         fence
  ["AUDIO_WORK_AREA",     "AUD_SRAM", U_TILE_ALIGN, 0x0003c000,   false],
  ["MSG_QUE_AREA",        "AUD_SRAM", U_MSGQ_ALIGN, 0x00003140,   false],
  ["MEMMGR_WORK_AREA",    "AUD_SRAM", U_STD_ALIGN,  0x00000200,   false],
  ["MEMMGR_DATA_AREA",    "AUD_SRAM", U_STD_ALIGN,  0x00000100,   false],

CAUTION: Fixed Areas can not be customized

PoolAreas
 # name,               area,              type,  align,       pool-size,   seg, fence,  spinlock
  ["MIC_IN_BUF_POOL",  "AUDIO_WORK_AREA", Basic, U_STD_ALIGN, 0x00000960,  5,   true,   ""],
  ["I2S_IN_BUF_POOL",  "AUDIO_WORK_AREA", Basic, U_STD_ALIGN, 0x000012C0,  5,   true,   ""],
  ["HP_OUT_BUF_POOL",  "AUDIO_WORK_AREA", Basic, U_STD_ALIGN, 0x000012C0,  5,   true,   ""],
  ["I2S_OUT_BUF_POOL", "AUDIO_WORK_AREA", Basic, U_STD_ALIGN, 0x000012C0,  5,   true,   ""],
  ["MFE_OUT_BUF_POOL", "AUDIO_WORK_AREA", Basic, U_STD_ALIGN, 0x00000500,  8,   true,   ""],

See "sony_apps/examples/audio/voice_call/config/mem_layout.conf" for each definition.

If configration is changed, use tool to generate header files.
Ex) ruby -Isony_apps/memutils/memory_manager/tool sony_apps/examples/audio/voice_call/config/mem_layout.conf mem_layout.h fixed_fence.h pool_layout.h

~~~

#### Builds & install #     {#spritzer_audio_voice_call_example_build}

Type 'make' to build NuttX. <br>

You can see 'nuttx' file on top of the NuttX source tree. <br>
Then copy nuttx.spk into Corvo USB MSC. <br>
(see README.txt for more information) <br>

CAUTION: <br>
sony_apps build system cannot build automatically by configuration or/and example source modification. <br>
Please 'make clean' first. <br>

DSP binary image install: <br>
Install DSP binary image to FlashROM. <br>
The DSP binary image to copy is in "proprietary/<board-name>/bin" according to configuration. <br>

- Binary image required for audio voice call according to configuration: <br>
 + "MFESRC.spk" <br>
 or
 + "MFESRC.espk"

- Binary image size loaded <br>
 + MFESRC : 174kbyte

#### Audio Voice Call Examples #   {#spritzer_audio_voice_call_example}

We provide Audio Voice Call example Code.

##### Build Configurations #     {#spritzer_audio_voice_call_example_configurations}

You will select as flow.

~~~
(audio voice call:)
[Application Configuration]
  [Examples]
    [Audio voice_call example] <= Y
~~~
Attention!! Audio & Logcal sensor example can select only on examle!
If you select multipule examples, you cannot build.

- Only support the following format:
 + Input channel: 1ch analog mic <br>

How to execute
- preparation <br>
  Connect the microphone and speaker to the board. Then I2S signal is inputted. <br>

- Now, let's run the voice call application. <br>
  Turn on and boot Nuttx board. <br>
  When board startup is completed. <br>
  Please activate the voice call application with command shown in blow.

  nsh>voice_call <br>

  Then, player could be accept player commands.<br>

 + start
   ~~~
   voice_call> start
   ~~~
   command, then the application will start voice call. <br>

 + stop
   ~~~
   voice_call> stop
   ~~~
   command, then the application will stop voice call. <br>

 + quit
   ~~~
   voice_call> quit
   ~~~
   command, then the application will exit voice call application. <br>

- call feature <br>
  The voice you input to the microphone is output I2S. <br>
  The I2S input is output speaker. <br>


### Audio Voice Command Functions#     {#spritzer_audio_voice_command_functions}

We provide Audio Voice Command Function.

A simple data flow on Audio Voice Command is shown below.

![Audio Voice Command Dataflow.] (TBD)

#### How to use # {#spritzer_audio_voice_command_functions_howtouse}

Here discribe how to use voice command function in typical case.

##### Preparation

Software component which called AudioManager, EffectorObject, RecognizerObject, RenderComponent, CaptureComponent in Audio SubSystem are designed to work as "task".<br>
So, to use them by application, call activate function of them.<br>

- Activate AudioManager.<br>
   Activate by caling AS_ActivateAudioSubSystem(AudioSubSystemIDs).<br>
   In the structure AudioSubSystemIDs of the argument, specify the MsgQueID defined in the Message Library Configuration.<br>
   Specify 0xFF for Object which is not used.<br>

~~~
AudioSubSystemIDs ids;

ids.app         = MSGQ_AUD_APP;
ids.mng         = MSGQ_AUD_MGR;
ids.player_main = 0xFF;
ids.player_sub  = 0xFF;
ids.mixer       = MSGQ_AUD_OUTPUT_MIX;
ids.recoder     = 0xFF;
ids.effector    = MSGQ_AUD_SOUND_EFFECT;
ids.recognizer  = MSGQ_AUD_RCG_CMD;

AS_ActivateAudioSubSystem(ids);
~~~

- Activate EffectorObject.<br>
   Activate by caling AS_ActivateEffector().<br>
   Specify the MsgQueID defined in the Message Library Configuration and the PoolId defined int the Memory Manager Configuration as arguments.<br>
   
~~~
bool act_rst = AS_ActivateEffector(
        MSGQ_AUD_SOUND_EFFECT,
        MSGQ_AUD_MGR,
        0xFF,
        MSGQ_AUD_DSP,
        MIC_IN_BUF_POOL,
        I2S_IN_BUF_POOL,
        HP_OUT_BUF_POOL,
        I2S_OUT_BUF_POOL,
        MFE_OUT_BUF_POOL
      );
~~~

- Activate RecognizerObject.<br>
   Activate by caling AS_activateRecognizer().<br>
   Specify the MsgQueID defined in the Message Library Configuration and the PoolId defined int the Memory Manager Configuration as arguments.<br>
   
~~~
bool act_rst = AS_activateRecognizer(
        MSGQ_AUD_RCG_CMD,
        MSGQ_AUD_MGR,
        WUWSR_IN_BUF_POOL
      );
~~~

- Activate RenderComponent.<br>
   Activate by caling AS_ActivateRenderer().<br>
   Specify the MsgQueID defined in the Message Library Configuration and the PoolId defined int the Memory Manager Configuration as arguments.<br>

~~~
bool act_rst = AS_ActivateRenderer(
        MSGQ_AUD_RND_SPHP,
        MSGQ_AUD_RND_SPHP_SYNC,
        MSGQ_AUD_RND_I2S,
        MSGQ_AUD_RND_I2S_SYNC
        );
~~~

- Activate CaptureComponent.<br>
   Activate by caling AS_ActivateCapture().<br>
   Specify the MsgQueID defined in the Message Library Configuration and the PoolId defined int the Memory Manager Configuration as arguments.<br>

~~~
bool act_rst = AS_ActivateRenderer(
        MSGQ_AUD_CAP_MIC,
        MSGQ_AUD_CAP_MIC_SYNC,
        MSGQ_AUD_CAP_I2S,
        MSGQ_AUD_CAP_I2S_SYNC
        );
~~~

#### Build Configurations #     {#spritzer_audio_voice_command_highlevel_configurations}

Select options in below:
~~~
:(Select audio voice command application)
[System Type]
  [CXD56xx Peripheral Support]
    [Audio] <= Y
      [I2S0] <= Y
[RTOS Features]
  [Tasks and Scheduling]
    [Enable waitpid() API]  <= Y
  [Stack and heap information]
    [Idle thread stack size] <= 4096
[Memory Management]
  [Number of memory regions] <= 2
  [Enable Tile Allocator]    <= Y
[Library Routines]
  [Standard Math library] <= Y
  [Have C++ compiler] <= Y
  [Have C++ initialization] <= Y
  [ASMP support library] <= Y
[Application Configuration]
  [Audio Utilities]
    [Audio manager] <= Y
    [Audio Feature Support]
      [Audio Feature] <= Y
      [Voice Command] <= Y
      [Mic Front End] <= Y
      [Media Player Post] <= Y
         (Attention: If anything other than the above is selected, set 'N' to these items)
  [Memory Utilities]
    [Memory manager] <= Y
    [Message] <= Y
    [Simple FIFO] <= Y

~~~
#### Memory Utility Configurations and Layout#     {#spritzer_voice_command_memutil_configurations}

##### Message Library Configuration

~~~
MsgQuePool
 # ID,                           n_size  n_num  h_size  h_num
  ["MSGQ_AUD_MGR",               88,     3,     0,      0],
  ["MSGQ_AUD_APP",               40,     2,     0,      0],
  ["MSGQ_AUD_DSP",               20,     5,     0,      0],
  ["MSGQ_AUD_OUTPUT_MIX",        48,     8,     0,      0],
  ["MSGQ_AUD_SOUND_EFFECT",      52,     5,     0,      0],
  ["MSGQ_AUD_RCG_CMD",           20,     5,     0,      0],
  ["MSGQ_AUD_CAP_MIC",           24,    16,     0,      0],
  ["MSGQ_AUD_CAP_MIC_SYNC",      16,     8,     0,      0],
  ["MSGQ_AUD_CAP_I2S",           24,    16,     0,      0],
  ["MSGQ_AUD_CAP_I2S_SYNC",      16,     8,     0,      0],
  ["MSGQ_AUD_RND_SPHP",          32,    16,     0,      0],
  ["MSGQ_AUD_RND_SPHP_SYNC",     16,     8,     0,      0],
  ["MSGQ_AUD_RND_I2S",           32,    16,     0,      0],
  ["MSGQ_AUD_RND_I2S_SYNC",      16,     8,     0,      0],

See "sony_apps/examples/audio/voice_command/config/msgq_layout.conf" for each definition.

If configration is changed, use tool to generate header files.
Ex) ruby -Isony_apps/memutils/message/tool sony_apps/examples/audio/voice_command/config/msgq_layout.conf 0x000fc000 0x3140 msgq_id.h msgq_pool.h

~~~

##### Memory Manager (Intelligent Fix Pool) Configuration

~~~
FixedAreas
 # name,                  device,     align,        size,         fence
  ["AUDIO_WORK_AREA",     "AUD_SRAM", U_TILE_ALIGN, 0x0003c000,   false],
  ["MSG_QUE_AREA",        "AUD_SRAM", U_MSGQ_ALIGN, 0x00003140,   false],
  ["MEMMGR_WORK_AREA",    "AUD_SRAM", U_STD_ALIGN,  0x00000200,   false],
  ["MEMMGR_DATA_AREA",    "AUD_SRAM", U_STD_ALIGN,  0x00000100,   false],

CAUTION: Fixed Areas can not be customized

PoolAreas
 # name,               area,              type,  align,       pool-size,   seg,  fence,  spinlock
  ["MIC_IN_BUF_POOL",  "AUDIO_WORK_AREA", Basic, U_STD_ALIGN, 0x00000960,  5,    true,   ""],
  ["I2S_IN_BUF_POOL",  "AUDIO_WORK_AREA", Basic, U_STD_ALIGN, 0x000012C0,  5,    true,   ""],
  ["HP_OUT_BUF_POOL",  "AUDIO_WORK_AREA", Basic, U_STD_ALIGN, 0x000012C0,  5,    true,   ""],
  ["I2S_OUT_BUF_POOL", "AUDIO_WORK_AREA", Basic, U_STD_ALIGN, 0x000012C0,  5,    true,   ""],
  ["MFE_OUT_BUF_POOL", "AUDIO_WORK_AREA", Basic, U_STD_ALIGN, 0x00000500,  8,    true,   ""],
  ["WUWSR_IN_BUF_POOL","AUDIO_WORK_AREA", Basic, U_STD_ALIGN, 0x00002A80,  17,   true,   ""],

See "sony_apps/examples/audio/voice_command/config/mem_layout.conf" for each definition.

If configration is changed, use tool to generate header files.
Ex) ruby -Isony_apps/memutils/memory_manager/tool sony_apps/examples/audio/voice_command/config/mem_layout.conf mem_layout.h fixed_fence.h pool_layout.h

~~~

#### Builds & install #     {#spritzer_audio_voice_command_example_build}

Type 'make' to build NuttX. <br>

You can see 'nuttx' file on top of the NuttX source tree. <br>
Then copy nuttx.spk into Corvo USB MSC. <br>
(see README.txt for more information) <br>

CAUTION: <br>
sony_apps build system cannot build automatically by configuration or/and example source modification. <br>
Please 'make clean' first. <br>

DSP binary image install: <br>
Install DSP binary image to FlashROM. <br>
The DSP binary image to copy is in "proprietary/<board-name>/bin" according to configuration. <br>

- Binary image required for audio voice call according to configuration: <br>
 + "MFESRC.spk"
 + "VADWUW.spk" <br>
 or
 + "MFESRC.espk"
 + "VADWUW.espk"

- Binary image size loaded <br>
 + MFESRC : 174kbyte
 + VADWUW : 179kbyte

#### Audio Voice Command Examples #   {#spritzer_audio_voice_command_example}

We provide Audio Voice Command example Code.

##### Build Configurations #     {#spritzer_audio_voice_command_example_configurations}

You will select as flow.

~~~
(audio voice command:)
[Application Configuration]
  [Examples]
    [Audio voice_command example] <= Y
~~~
Attention!! Audio & Logcal sensor example can select only on examle!
If you select multipule examples, you cannot build.

- Only support the following format:
 + Input channel: 1ch analog mic <br>

How to execute
- preparation <br>
  Connect the microphone and speaker to the board. Then I2S signal is inputted. <br>

- Now, let's run the voice command application. <br>
  Turn on and boot Nuttx board. <br>
  Please activate the voice command application when board startup is completed. <br>
  nsh>voice_command <br>
 
- recognition word <br>
  You say "hallo sony" to the microphone. <br>
  + If your voice is recognitioned, "VAD RISE->" and "Found Command" are displayed on consol. <br>
  + If your voice is not recognitioned, "VAD RISE->" is displayed on consol. <br>
  + If there is no voice input for 2 seconds, "->VAD FALL" is displayed on consol. <br>


--------------------------------------
# Low Level API  {#spritzer_audio_low_level_api}

- Now Under constructions.


--------------------------------------
# Used Libraries  {#spritzer_audio_used_libraries}

Audio Sub-System  assumes that you use the following libraries.

- @ref spritzer_memutil

