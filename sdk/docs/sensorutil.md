Sensor Fusion Utility Libraries on Spritzer{#sensorutil_doc}
============================
[TOC]

# General # {#spritzer_sensorutil_general}

We provide utiliy libraries for sensor fusion.\n
These libraries are composed of "Sensor Manager", and some logical sensors.


- "Sensor Manager" provide the framework to distribute some sensor data based on publish-subscribe architecture.
- Logical sensors provide some some algorithms of sensor fusion.

The publish / subscribe architecture diagram provided by Sensor Manager is shown below.

![Sensor Manager publish / subscribe diagram](sensor_manager_pub_sub_diagram.png)

By using the mechanism of this sensor framework, you can easily add your own logical sensors.

---------------------

# Sensor Manager # {#spritzer_sensor_manager}

## General # {#spritzer_sensor_manager_general}

"Sensor Manager" is the framework for logical sensors(sensor fusion).\n
A sensor is resist sensor manager for other sensors.\n


## APIs # {#spritzer_sensor_manager_api}

"Sensor Manager" provides SF_SendSensor* API.\n
It has the following 5 APIs.\n
\n
It issues commands such as various operations and settings for the logical sensor.\n
The command is controlled by the data format called packet.\n

| APIs         | Description                                               | Corresponding API            | Corresponding packet      |
|:-------------|:----------------------------------------------------------|:-----------------------------|:--------------------------|
| Register     | Resister a sensor client to Sensor Manager as subscriber. | (@ref SF_SendSensorResister) | sensor_command_register_t |
| Release      | Unresister the sensor client from Sensor Manager.         | (@ref SF_SendSensorRelease)  | sensor_command_release_t  |
| SendData     | Sender function to Sensor Manager without MemHandle.      | (@ref SF_SendSensorData)     | sensor_command_data_t     |
| SendData(MH) | Sender function to Sensor Manager with MemHandle.         | (@ref SF_SendSensorDataMH)   | sensor_command_data_mh_t  |
| SendSetPower | Set power status of sensors.                              | (@ref SF_SendSensorSetPower) | sensor_command_power_t    |

**Sensor manager API using.**\n
An example of a packet set by each API is shown below.

- Register\n
   In this example, StepCounter is regiterd to SensorManager as client, and it's subscribing Accel and Gnss sensor data.\n
   When Accel or Gnss publishes a any data, callback function "receive_data_(mh_)callback" will be called.\n
   If power control operattion excuted to StepCounter, then "power_ctrl_callback" will be called.\n
 ~~~{.c}
  sensor_command_register_t reg;

  reg.header.size   = 0;
  reg.header.code   = ResisterClient;
  reg.self          = stepcounterID;
  reg.subscriptions = (0x01 << accelID) | (0x01 << gnssID);
  reg.callback      = receive_data_callback;
  reg.callback_mh   = receive_data_mh_callback;
  reg.callback_pw   = power_ctrl_callback;

  SF_SendSensorResister(&reg);
 ~~~

- Release\n
   Unregister StepCounter from SensorManager. After this, StepCounter will not receive any publish data and power control request.\n
 ~~~{.c}
  sensor_command_release_t rel;

  rel.header.size = 0;
  rel.header.code = ReleaseClient;
  rel.self        = stepcounterID;

  SF_SendSensorRelease(&rel);
 ~~~

- SendData\n
   Send a data address from Accel to SensorManager.\n
   The data will be published to subscribers of Accel.\n
 ~~~{.c}
  static const uint16_t ACCEL_SAMPLE_MAX_NUM = 32; /* Sample number of acceleration data. */

  static int accel_data_send(uint32_t context, AccelDOF* acc_dof)
  {
    typedef struct
    {
      unsigned long time_stamp; /* Time stamp at update.[ms]                */
      float         ax;         /* X axis standard gravity acceleration.[G] */
      float         ay;         /* Y axis standard gravity acceleration.[G] */
      float         az;         /* Z axis standard gravity acceleration.[G] */
    } AccData;

    static uint8_t  rcv_cnt = 0;
    AccData         read_data;
    static AccData  send_data[ACCEL_SAMPLE_MAX_NUM];

    send_data[rcv_cnt].time_stamp = acc_dof->sensor_time;
    send_data[rcv_cnt].ax         = acc_dof->accel_x;
    send_data[rcv_cnt].ay         = acc_dof->accel_y;
    send_data[rcv_cnt].az         = acc_dof->accel_z;

    rcv_cnt++;

    if (rcv_cnt >= ACCEL_SAMPLE_MAX_NUM)
      {
        sensor_command_data_mh_t packet;
        packet.header.size = 4;
        packet.header.code = SendData;
        packet.self        = accelID;
        packet.time        = 0;
        packet.fs          = ACCEL_SAMPLING_FREQUENCY;
        packet.size        = ACCEL_SAMPLE_MAX_NUM;
        packet.adr         = (void*)&send_data[0];

        SF_SendSensorData(&packet);

        rcv_cnt = 0;
      }
    return 0;
  }
 ~~~

- SendData(MH)\n
   Send a data with MemHandle from Accel to SensorManager.\n
   The data will be published to subscribers of Accel.\n
 ~~~{.c}
  static const uint16_t ACCEL_SAMPLE_MAX_NUM = 32; /* Sample number of acceleration data. */

  static int accel_read_callback(uint32_t context, AccelDOF* acc_dof)
  {
    /* Accel data fromat for AESM */

    typedef struct
    {
      unsigned long time_stamp; /* Time stamp at update.[ms]                */
      float         ax;         /* X axis standard gravity acceleration.[G] */
      float         ay;         /* Y axis standard gravity acceleration.[G] */
      float         az;         /* Z axis standard gravity acceleration.[G] */
    } AccData;

    static uint8_t  rcv_cnt = 0;
    static AccData* data_top = NULL;

    rcv_cnt++;

    if (rcv_cnt == 1)
      {
        MemMgrLite::MemHandle mh;
        if (mh.allocSeg(SENSOR_DATA_BUF_POOL, 0x200) != MemMgrLite::NoError)
          {
            ASSERT(0);
          }

        data_top = static_cast<AccData*>(mh.getVa());

        if (!p_accel_que->push(mh))
          {
            ASSERT(0); /* Cannot save MHandle due to system error. */
          }
      }

    data_top->time_stamp = acc_dof->sensor_time;
    data_top->ax         = acc_dof->accel_x;
    data_top->ay         = acc_dof->accel_y;
    data_top->az         = acc_dof->accel_z;
    data_top++;

    if (rcv_cnt >= ACCEL_SAMPLE_MAX_NUM)
      {
        MemMgrLite::MemHandle mh = p_accel_que->top();

        sensor_command_data_mh_t packet;
        packet.header.size = 0;
        packet.header.code = SendData;
        packet.self        = accelID;
        packet.time        = 0;
        packet.fs          = ACCEL_SAMPLING_FREQUENCY;
        packet.size        = rcv_cnt;
        packet.mh          = mh;

        SF_SendSensorDataMH(&packet);

        rcv_cnt = 0;
      }
    return 0;
  }
 ~~~

*See also.*

- @ref sensor_manger

## Configurations & build  # {#spritzer_sensor_manager_config}

To use StepCounter, please set configs like below by "make menuconfig".
~~~
$ make menuconfig
~~~

~~~
[Library Routines]
  [Standard Math library]   <= Y
  [Have C++ compiler]       <= Y
  [Have C++ initialization] <= Y

[Application Configuration]
  [Memory Utilities]
    [MEMORY_MANAGER]        <= Y
    [MEMUTILS_MESSAGE]      <= Y
  [Sensor Utilities]
    [SENSOR_MANAGER]        <= Y
~~~

When you are done, please enter make and build the sample.
~~~
$ make
~~~

## Sequence # {#spritzer_sensor_manager_sequence}

---------------------
# Logical sensors # {#spritzer_logical_sensors}

## General # {#spritzer_logical_sensors_general}

Some logical sensors can be created by logical sensor framework.\n
A logical sensor can be implemented in various ways.\n
 - A logical sensor task on NuttX.
 - A logical sensor DSP on asmp.
 - A logical sensor DSP from A or other vendors with encryption.
 - A logical sensor Without Sensor Manager.


### A logical sensor task on NuttX.
 
| Contents                       | Service provider |
|:-------------------------------|:-----------------|
| TAP Gesture (Tapping Detector) | from SSS         |

 [T.B.D]


### A logical sensor DSP on asmp.


| Contents                       | Service provider |
|:-------------------------------|:-----------------|
| TAP Gesture (Tapping Detector) | from SSS         |

[not implemented.]

A logical sensor uses the ASMP framework.\n
The ASMP framework has Supervisor tasks and MP tasks.\n
In addition, the logical sensor uses the Worker task for the MP task.\n

![Supervisor / Worker configuration diagram.](spritzer_sensor_supervisor_worker_diagram.png)

For the ASMP framework, see the following.\n
- [Spritzer ASMP Framework](@ref spritzer_asmp_framework_general)



### A logical sensor DSP from A or other vendors with encryption.

We already provides several logical sensors from SONY R&D and SSS as follow.\n

| Contents                                                           | Service provider |
|:-------------------------------------------------------------------|:-----------------|
| AESM (Activity Engine and Step Meter)                              | from SONY R&D    |
| Arm Gesture (Gesture recognition on an arm for like a smart watch) | from SONY R&D    |
| Orientation (Electronic compass library)                           | from SONY R&D    |
| TRAM (TRAnsportation Mode recognition)                             | from SONY R&D    |



The worker part of the provided logical sensor is encrypted.\n
The figure is shown below.\n

![Worker is encrypted diagram.](encryption_sensor_client_proc.png)


### A logical sensor Without Sensor Manager.
 
| Contents                       | Service provider |
|:-------------------------------|:-----------------|
| TAP Gesture (Tapping Detector) | from SSS         |

---------------------
## Logical sensor implementation on Supervisor

- Create\n
   Create a class instance to communicate with workers.

- Open\n
   Load library and boot up as worker task.

- Write\n
   Send data to worker task.

- Close\n
   Destroy worker task.

These requests are defined as the following commands and are used for transmission and reception.\n
If you will create new logical sensor, you will modify this command.\n
For details, refer to each supervisor.

---------------------
## Logical sensor implementation on DSP(Worker)

We provide the framework for a logical sensor on worker(DSP).
It has 3 event for worker.

- init\n
  Initialize for logical sensor.

- exec\n
  Execute calculate on sensor data.

- flush\n
  End execute process of sensor data.

These requests are defined as the following commands and are used for transmission and reception.\n
If you will create new logical sensor, you will modify this command.

### DSP Command Framework APIs

*See also.*

- @ref logical_sensor


---------------------
## Pre-Provided logical sensors # {#spritzer_sony_logical_sensor}
This section discribes about Pre-Provided logical sensors.\n
There are designed on sensor framework, and it can be reffered as a example of design.\n

---------------------
## AESM (Activity Engine and Step Meter)  # {#spritzer_logical_sensor_stepcounter}
The configuration diagram of the logic sensor is shown below.\n
- Configuration of AESM\n
  ![AESM structural diagram](sensor_aesm_configuration_diagram.png)

### Supervisor # {#spritzer_logical_sensor_stepcounter_supervisor}
"Supervisor" is the framework for logical sensors(sensor fusion).\n
The Supervisor provides several APIs for controlling [Workers](@ref spritzer_logical_sensor_stepcounter_worker).\n

#### APIs # {#spritzer_logical_sensor_stepcounter_supervisor_apis}
AESM Supervisor provides StepCounter* API.\n
It has the following 4 APIs.

| APIs   | Description                                          | Corresponding APIs                          |
|:-------|:-----------------------------------------------------|:--------------------------------------------|
| Create | Create a class instance to communicate with workers. | [StepCounterCreate](@ref StepCounterCreate) |
| Open   | Load AESM library and boot up as worker task.        | [StepCounterOpen](@ref StepCounterOpen)     |
| Write  | Send data to AESM worker task.                       | [StepCounterWrite](@ref StepCounterWrite)   |
| Close  | Destroy AESM worker task.                            | [StepCounterClose](@ref StepCounterClose)   |

*See also.*

- @ref logical_stepcounter

##### Data format for StepCounter
Logical sensor StepCounter requires Accel and Gnss data by specified format as below.\n
It have to be written by [StepCounterWrite](@ref StepCounterWrite).\n

![Accel data format for StepCounter](step_counter/Stepcounter_require_data_format_accel.png)\n

![Gnss data format for StepCounter](step_counter/Stepcounter_require_data_format_gnss.png)\n

#### Sequence # {#spritzer_logical_sensor_stepcounter_supervisor_sequence}
Call APIs according to the following sequence diagram.

 - Initialize\n
   StepCounter Initialize sequence.
   ![Initial Sequence](step_counter/Stepcounter_Initial_sequence.png)

 - Sensing(for Accel)\n
   StepCounter Sensing sequence. (In case of Accel data received.)
   ![Sensing Sequence(Accel)](step_counter/Stepcounter_Sensing_sequence_Accel.png)

 - Sensing(for Gnss)\n
   StepCounter Sensing sequence. (In case of Gnss data received.)
   ![Sensing Sequence(Gnss)](step_counter/Stepcounter_Sensing_sequence_Gnss.png)


#### Configurations & build # {#spritzer_logical_sensor_stepcounter_supervisor_config}

To use StepCounter, please set configs like below by "make menuconfig".
~~~
$ make menuconfig
~~~

~~~
[System Type]
  [CXD56xx Peripheral Support]
    [SPI3]                        <= Y
    [Sensor Control Unit Support] <= Y (*1)
    [GNSS device]                 <= Y

[RTOS Features]
  [Stack and heap information]
    [Idle thread stack size]      <= 4096

[Device Drivers]
  [Sensor Device Support]         <= Y
    [Bosch BMI160 Sensor support] <= Y

[Memory Management]
  [Number of memory regions]      <= 2
  [Enable Tile Allocator]         <= Y

[Library Routines]
  [Standard Math library]         <= Y
  [Have C++ compiler]             <= Y
  [Have C++ initialization]       <= Y
  [ASMP support library]          <= Y

[Application Configuration]
  [Examples]
    [AESM sensor example]         <= Y (*2)
  [Sensor Utilities]
    [Sensor manager]              <= Y
      [Logical Sensors]
        [Stepcounter]             <= Y
  [Memory Utilities]
    [MEMORY_MANAGER]              <= Y
    [MEMUTILS_MESSAGE]            <= Y
  [GPS Utilities]
    [Support CXD56xx gnss NMEA convert library] <= Y
~~~

(\*1):If you'd like to use Sensor Control Unit, it's necessary to activate this config.\n
(\*2):To build the StepCounter sample, activate this config.(!!Cannnot activate other Examples at the same time.!!)

Alternatively, you can set it by entering the following command.
~~~
$ cd sony_apps
$ ./tools/configure.sh corvo/aesm
~~~

When you are done, please enter make and build the sample.
~~~
$ make
~~~

#### Memory Utirity Library Configurations & Layout # {#spritzer_logical_sensor_stepcounter_supervisor_memutil_config}

##### Message Library Configuration

~~~
MsgQuePool
 # ID,              n_size  n_num  h_size  h_num
  ["MSGQ_SEN_MGR",  40,     8,     0,      0],

See "sony_apps/examples/logical_sensor/aesm/config/msgq_layout.conf" for each definition.
~~~

##### Memory Manager (Intelligent Fix Pool) Configuration

~~~
FixedAreas
 # name,                  device,     align,        size,         fence
  ["SENSOR_WORK_AREA",    "SHM_SRAM", U_TILE_ALIGN, 0x0001c400,   false],
  ["MSG_QUE_AREA",        "SHM_SRAM", U_MSGQ_ALIGN, 0x00003240,   false],
  ["MEMMGR_WORK_AREA",    "SHM_SRAM", U_STD_ALIGN,  0x00000200,   false],
  ["MEMMGR_DATA_AREA",    "SHM_SRAM", U_STD_ALIGN,  0x00000100,   false],

CAUTION: Fixed Areas can not be customized

PoolAreas
 # name,                      area,               type,  align,       pool-size,  seg, fence,  spinlock
  ["SENSOR_DSP_CMD_BUF_POOL", "SENSOR_WORK_AREA", Basic, U_STD_ALIGN, 0x70,       8,   false,  ""],
  ["ACCEL_DATA_BUF_POOL",     "SENSOR_WORK_AREA", Basic, U_STD_ALIGN, 0x180,      8,   false,  ""],
  ["GNSS_DATA_BUF_POOL",      "SENSOR_WORK_AREA", Basic, U_STD_ALIGN, 0x30,       8,   false,  ""],

See "sony_apps/examples/logical_sensor/aesm/config/mem_layout.conf" for each definition.
~~~

### Worker # {#spritzer_logical_sensor_stepcounter_worker}
The Worker runs on other core and analys sensor data.\n
And returns the processing result(step count, state, etc...) of various requested commands.

AESM worker requires data which shown below for analysis.
* Accel data (32Hz, 32samples/1sec)
* Gnss data (1Hz, 1sample/1sec)

#### APIs # {#spritzer_logical_sensor_stepcounter_worker_apis}
We provide the framework for a AESM on worker.
It has 3 event for worker.

| Event | Description                       | Corresponding command |
|:------|:----------------------------------|:----------------------|
| init  | Initialize for AESM.              | SensorInitAesm        |
| exec  | Execute calculate on sensor data. | SensorExecAesm        |
| flush | End execute process of AESM.      | SensorFlushAesm       |

In execute, the following are prepared as AESM execution commands.
| Command                                          | Description                    |
|:-------------------------------------------------|:-------------------------------|
| [AESM_CMD_UPDATE_ACCELERATION](@ref AesmCmdType) | Update by acceleration sensor. |
| [AESM_CMD_UPDATE_GPS](@ref AesmCmdType)          | Update by GPS information.     |

**DSP command setting method.**\n
An example of a packet set for each command is shown below.

- init\n
  ~~~{.c}
    SensorDspCmd dsp_cmd;

    dsp_cmd.header.sensor_type     = StepCounter;
    dsp_cmd.header.event_type      = InitEvent;
  ~~~

- exec(for Accel)\n
  ~~~{.c}
    SensorDspCmd dsp_cmd;

    dsp_cmd.header.sensor_type     = StepCounter;
    dsp_cmd.header.event_type      = ExecEvent;
    dsp_cmd.exec_aesm_cmd.cmd_type = AESM_CMD_UPDATE_ACCELERATION;
  ~~~

- exec(for GPS)\n
  ~~~{.c}
    SensorDspCmd dsp_cmd;

    dsp_cmd.header.sensor_type     = StepCounter;
    dsp_cmd.header.event_type      = ExecEvent;
    dsp_cmd.exec_aesm_cmd.cmd_type = AESM_CMD_UPDATE_GPS;
  ~~~

- flush\n
  ~~~{.c}
    SensorDspCmd dsp_cmd;

    dsp_cmd.header.sensor_type     = StepCounter;
    dsp_cmd.header.event_type      = FlushEvent;
  ~~~

*See also.*

- @ref logical_aesm

#### Sequence # {#spritzer_logical_sensor_stepcounter_worker_sequence}
Call APIs according to the following sequence diagram.

- Initialize\n
  ![AESM Initialize](DSP_AESM_Init.png)

- Execute\n
  ![AESM Execute](DSP_AESM_Exec.png)

- Terminate\n
  ![AESM Terminate](DSP_AESM_Flush.png)

---------------------
## Arm Gesture # {#spritzer_logical_sensor_arm_gesture}
The configuration diagram of the logical sensor provided below is shown.\n
- Configuration of gesture\n
  ![Arm Gesture structural diagram](sensor_arm_gesture_configuration_diagram.png)

### Supervisor # {#spritzer_logical_sensor_arm_gesture_supervisor}
"Supervisor" is the framework for logical sensors(sensor fusion).\n
The Supervisor provides several APIs for controlling [Workers](@ref spritzer_logical_sensor_arm_gesture_worker).\n


#### APIs # {#spritzer_logical_sensor_arm_gesture_supervisor_apis}
Arm Gesture Supervisor provides Gesture* API.\n
It has the following 5 APIs.

|APIs         |Description                                              |Corresponding APIs                               |
|:------------|:--------------------------------------------------------|:------------------------------------------------|
|Create       |Create a class instance to communicate with workers.     |[GestureCreate](@ref GestureCreate)              |
|Open         |Load Arm Gesture library and boot up as worker task.     |[GestureOpen](@ref GestureOpen)                  |
|Write        |Send data to Arm Gesture worker task.                    |[GestureWrite](@ref GestureWrite)                |
|Close        |Destroy Arm Gesture worker task.                         |[GestureClose](@ref GestureClose)                |
|SetCoordinate|Set rotation for coordinate transformation of the device.|[GestureSetCoordinate](@ref GestureSetCoordinate)|


*See also.*

- @ref logical_gesture

##### Data format for ArmGesture
Logical sensor ArmGesture requires Accel data by specified format as below.\n
It have to be written by [GestureWrite](@ref GestureWrite).\n

![Accel data format for ArmGesture](gesture/arm_gesture_require_data_format_accel.png)\n

##### Specification of rotation for coordinate transformation
The relationship between data and rotation parameter is as follows.\n

~~~
x_adjusted = data_x * rotation_xx + data_y * rotation_xy + data_z * rotation_xz;
y_adjusted = data_x * rotation_yx + data_y * rotation_yy + data_z * rotation_yz;
z_adjusted = data_x * rotation_zx + data_y * rotation_zy + data_z * rotation_zz;

~~~

The relationship between default rotation parameter value and coordinate is as follows.

~~~
rotation_xx =  1; rotation_xy =  0; rotation_xz =  0;
rotation_yx =  0; rotation_yy =  1; rotation_yz =  0;
rotation_zx =  0; rotation_zy =  0; rotation_zz =  1;
~~~

![Default coordinate](gesture/arm_gesture_default_coordinate.png)\n


#### Sequence # {#spritzer_logical_sensor_arm_gesture_supervisor_sequence}
Call APIs according to the following sequence diagram.

 - Initialize\n
   Arm Gesture Initialize sequence.
   ![Initial Sequence](gesture/arm_gesture_initial_seqence.png)

 - Sensing\n
   Arm Gesture Sensing sequence. (In case of Accel data received.)
   ![Sensing Sequence(arm gesture)](gesture/arm_gesture_execute_seqence.png)

#### Configurations & build # {#spritzer_logical_sensor_arm_gesture_supervisor_config}

To use Arm Gesture, please set configs like below by "make menuconfig".
~~~
$ make menuconfig
~~~

~~~
[System Type]
  [CXD56xx Peripheral Support]
    [SPI3] <= Y
    [Sensor Control Unit Support] <= Y (*1)

[RTOS Features]
  [Stack and heap information]
    [Idle thread stack size]      <= 4096

[Device Drivers]
  [Sensor Device Support]         <= Y
    [Bosch BMI160 Sensor support] <= Y

[Memory Management]
  [Number of memory regions]      <= 2
  [Enable Tile Allocator]         <= Y

[Library Routines]
  [Standard Math library]         <= Y
  [Have C++ compiler]             <= Y
  [Have C++ initialization]       <= Y
  [ASMP support library]          <= Y

[Application Configuration]
  [Examples]
    [Arm gesture sensor example]  <= Y(*2)
  [Sensor Utilities]
    [Sensor manager]              <= Y
      [Logical Sensors]
        [Arm gesture]             <= Y
  [Memory Utilities]
    [MEMORY_MANAGER]              <= Y
    [MEMUTILS_MESSAGE]            <= Y
~~~

(\*1):If you'd like to use Sensor Control Unit, it's necessary to activate this config.\n
(\*2):To build the Arm Gesture sample, activate this config.(!!Cannnot activate other Examples at the same time.!!)

- Arm Gesture
~~~
 arm_gesture  --->
  [arm gesture sensor example] <= Y
~~~

Alternatively, you can set it by entering the following command.
~~~
$ cd sony_apps
$ ./tools/configure.sh corvo/arm_gesture
~~~

When you are done, please enter make and build the sample.
~~~
$ make
~~~

#### Memory Utirity Library Configurations & Layout # {#spritzer_logical_sensor_arm_gesture_supervisor_memutil_config}

##### Message Library Configuration

~~~
MsgQuePool
 # ID,              n_size  n_num  h_size  h_num
  ["MSGQ_SEN_MGR",  40,     8,     0,      0],

See "sony_apps/examples/logical_sensor/arm_gesture/config/msgq_layout.conf" for each definition.
~~~

##### Memory Manager (Intelligent Fix Pool) Configuration

~~~
FixedAreas
 # name,                  device,     align,        size,         fence
  ["SENSOR_WORK_AREA",    "SHM_SRAM", U_TILE_ALIGN, 0x0001c400,   false],
  ["MSG_QUE_AREA",        "SHM_SRAM", U_MSGQ_ALIGN, 0x00003240,   false],
  ["MEMMGR_WORK_AREA",    "SHM_SRAM", U_STD_ALIGN,  0x00000200,   false],
  ["MEMMGR_DATA_AREA",    "SHM_SRAM", U_STD_ALIGN,  0x00000100,   false],

CAUTION: Fixed Areas can not be customized

PoolAreas
 # name,                      area,               type,  align,       pool-size,  seg, fence,  spinlock
  ["SENSOR_DSP_CMD_BUF_POOL", "SENSOR_WORK_AREA", Basic, U_STD_ALIGN, 0x70,       8,   false,  ""],
  ["ACCEL_DATA_BUF_POOL",     "SENSOR_WORK_AREA", Basic, U_STD_ALIGN, 0xC0,       8,   false,  ""],

See "sony_apps/examples/logical_sensor/arm_gesture/config/mem_layout.conf" for each definition.
~~~

### Worker # {#spritzer_logical_sensor_arm_gesture_worker}
The Worker runs on other core and analys sensor data.\n
And returns the processing result of various requested commands.

ArmGesture worker requires data which shown below for analysis.
* Accel data (8Hz, 16samples/2sec)

#### APIs # {#spritzer_logical_sensor_arm_gesture_worker_apis}

We provide the framework for a Arm Gesture on worker.
It has 3 event for worker.

|Event |Description                         |Corresponding command |
|:-----|:-----------------------------------|:---------------------|
|init  |Initialize for Arm Gesture.(*)      |SensorInitGesture     |
|exec  |Execute calculate on sensor data.   |SensorExecGesture     |
|flush |End execute process of Arm Gesture. |SensorFlushGesture    |

(\*):For initialize please set the gesture type you want to recognize.\n
     Please join the following values with a logical sum.\n
    (This is also the case when changing the type while using set commands.)

 - [Arm up motion.](@ref GESTURE_ARM_UP)
 - [Arm down motion.](@ref GESTURE_ARM_DOWN)
 - [Arm twist motion.](@ref GESTURE_TWIST)
 - [Arm shake motion.](@ref GESTURE_SHAKE)

In execute, the following are prepared as Arm Gesture execution commands.
|Command                                      |Description                                  |
|:--------------------------------------------|:--------------------------------------------|
| [GESTURE_CMD_SET_TYPE](@ref GestureCmdType) | Reconfiguration after initialization.       |
| [GESTURE_CMD_GET_TYPE](@ref GestureCmdType) | Get gesture type.                           |
| [GESTURE_CMD_ROTATION](@ref GestureCmdType) | Set rotation for coordinate transformation. |

**DSP command setting method.**\n
An example of a packet set for each command is shown below.

- init\n
  ~~~{.c}
    SensorDspCmd dsp_cmd;

    dsp_cmd.header.sensor_type        = ArmGesture;
    dsp_cmd.header.event_type         = InitEvent;
    dsp_cmd.init_gesture_cmd.gesture_types.type
                                      = (GESTURE_ARM_UP | GESTURE_ARM_DOWN | GESTURE_TWIST | GESTURE_SHAKE);
  ~~~

- exec(for get type)\n
  ~~~{.c}
    SensorDspCmd dsp_cmd;

    dsp_cmd.header.sensor_type        = ArmGesture;
    dsp_cmd.header.event_type         = ExecEvent;
    dsp_cmd.exec_gesture_cmd.cmd_type = GESTURE_CMD_GET_TYPE;
  ~~~

- exec(for set type)\n
  ~~~{.c}
    SensorDspCmd dsp_cmd;

    dsp_cmd.header.sensor_type        = ArmGesture;
    dsp_cmd.header.event_type         = ExecEvent;
    dsp_cmd.exec_gesture_cmd.cmd_type = GESTURE_CMD_SET_TYPE;
    dsp_cmd.exec_gesture_cmd.gesture_types.type
                                      = (GESTURE_ARM_UP | GESTURE_ARM_DOWN);
  ~~~

- exec(for rotation)\n
  ~~~{.c}
    GestureSetRotation rotation;
    rotation.xx = xx; rotation.xy = xy; rotation.xz = xz;
    rotation.yx = yx; rotation.yy = yy; rotation.yz = yz;
    rotation.zx = zx; rotation.zy = zy; rotation.zz = zz;
  
    SensorDspCmd dsp_cmd;

    dsp_cmd.header.sensor_type        = ArmGesture;
    dsp_cmd.header.event_type         = ExecEvent;
    dsp_cmd.exec_gesture_cmd.cmd_type = GESTURE_CMD_ROTATION;
    dsp_cmd.exec_gesture_cmd.gesture_rotation = rotation;
  ~~~

- flush\n
  ~~~{.c}
    SensorDspCmd dsp_cmd;

    dsp_cmd.header.sensor_type        = ArmGesture;
    dsp_cmd.header.event_type         = FlushEvent;
  ~~~

*See also.*

- @ref logical_arm_gesture

#### Sequence # {#spritzer_logical_sensor_arm_gesture_worker_sequence}
Call APIs according to the following sequence diagram.

- Initialize\n
  ![Arm Gesture Initialize](DSP_Gesture_Init.png)

- Execute\n
  ![Arm Gesture Execute](DSP_Gesture_Exec.png)

- Terminate\n
  ![Arm Gesture Terminate](DSP_Gesture_Flush.png)


---------------------
## TAP Gesture (Tapping Detector) # {#spritzer_logical_sensor_tap}

### Tap Library # {#spritzer_logical_sensor_tap_tap_library}
Tap Library is an algorithm to detect tap from accel sensor data.\n
The configuration diagram of the logic sensor shown below is shown.
- Configuration of Tap Library\n
  ![Tap Library structural diagram](tap_library_configuration_diagram.png)

#### APIs # {#spritzer_logical_sensor_tap_tap_library_api}
Tap Library provides Tap Library* API.\n
It has the following 5 APIs.

|APIs   |Description                                                                |Corresponding APIs                            |
|:------|:--------------------------------------------------------------------------|:---------------------------------------------|
|Create |Create TapClass instance.                                                  |[TapCreate](@ref TapCreate)                   |
|Open   |Set coefficients necessary for parameter  initialization and tap detection.|[TapOpen](@ref TapOpen)                       |
|Write  |(with Timestamp) Detect tap.                                               |[TapWrite](@ref TapWrite)                     |
|Write  |(without Timestamp) get Timestamp and Detect tap.                          |[TapWrite_timestamp](@ref TapWrite_timestamp) |
|Close  |none                                                                       |[TapClose](@ref TapClose)                     |

*See also.*

- @ref logical_tap_library

#### Configurations & build # {#spritzer_logical_sensor_tap_tap_library_config}
To use Tap Library, please set configs like below by "make menuconfig".
~~~
$ make menuconfig
~~~

~~~
[RTOS Features]
  [Clocks and Scheduling]
    [Support CLOCK_MONOTONIC] <= Y

[Library Routines]
  [Standard Math library] <= Y
  [Have C++ compiler] <= Y
  [Have C++ initialization] <= Y

[Application Configuration]
  [Sensor Utilities]
    [Logical Sensors]
      [Tap]
        [Support tap library] <= Y
~~~

When you are done, please enter make and build the Tap Library.
~~~
$ make
~~~

#### Sequence # {#spritzer_logical_sensor_tap_tap_library_sequence}
Call APIs according to the following sequence diagram.

 - Tap sequence.
   Do not use Tap Manager, call Tap Library directly from example.\n
   (!!There is no sample code for this sequence.!!)
   ![Tap Library Sequence](tap/Tap_Library_sequence.png)


### Tap Manager # {#spritzer_logical_sensor_tap_tap_manager}
Tap Manager activates the accel sensor, passes the accel data to Tap Library and receives the tapcnt.\n

The configuration diagram of the logic sensor shown below is shown.
- Configuration of Tap Manager\n
  ![Tap Manager structural diagram](tap_manager_configuration_diagram.png)


#### APIs # {#spritzer_logical_sensor_tap_tap_manager_api}
Tap Manager provides Tap Manager* API.\n
It has the following 4 APIs.

|APIs   |Description                                          |Corresponding APIs                  |
|:------|:----------------------------------------------------|:-----------------------------------|
|Init   |Initial processing. Create task.                     |[TapMngInit](@ref TapMngInit)       |
|Start  |Start Accel sensor and detect tap.                   |[TapMngStart](@ref TapMngStart)     |
|Stop   |Stop the Accel sensor and stop tap detection.        |[TapMngStop](@ref TapMngStop)       |
|Fin    |Finalize processing.                                 |[TapMngFin](@ref TapMngFin)         |

*See also.*

- @ref logical_tap_manager

#### Configurations & build # {#spritzer_logical_sensor_tap_tap_manager_config}
To use Tap Manager, please set configs like below by "make menuconfig".
~~~
$ make menuconfig
~~~

~~~
[System Type]
  [CXD56xx Peripheral Support]
    [SPI3] <= Y
    [Sensor Control Unit Support] <= Y

[RTOS Features]
  [Clocks and Scheduling]
    [Support CLOCK_MONOTONIC] <= Y

[Device Drivers]
  [Sensor Device Support] <= Y
    [Bosch BMI160 Sensor support] <= Y

[Library Routines]
  [Have C++ compiler] <= Y
  [Have C++ initialization] <= Y

[Application Configuration]
  [Examples]
    [Tap manager example] <= Y (*1)
  [Sensor Utilities]
    [Logical Sensors]
      [Tap]
        [Support tap library] <= Y
        [Support tap manager library] <= Y
~~~
(\*1):To build the Tap sample, activate this config.(!!Cannnot activate other Examples at sametime.!!)

- Tap
~~~
  [Examples] --->
    [Tap manager example] <= Y
~~~

When you are done, please enter make and build the sample.
~~~
$ make
~~~

#### Sequence # {#spritzer_logical_sensor_tap_tap_manager_sequence}
Call APIs according to the following sequence diagram.
   
 - Initialize\n
   Tap Manager Initialize sequence.
   ![Initial Sequence](tap/tap_manager_Initial_sequence.png)

 - Start\n
   Tap Manager Start sequence.
   ![Start Sequence](tap/tap_manager_Start_sequence.png)

 - Stop\n
   Tap Manager Stop sequence.
   ![Stop Sequence](tap/tap_manager_Stop_sequence.png)

 - Finalize\n
   Tap Manager Finalize sequence.
   ![Finalize Sequence](tap/tap_manager_Finalize_sequence.png)

---------------------
## Orientation (Electronic compass library)  # {#spritzer_logical_sensor_orientation}
The configuration diagram of the logic sensor shown below is shown.\n
- Configuration of Orientation\n
  ![Orientation structural diagram](sensor_compass_configuration_diagram.png)

### Supervisor # {#spritzer_logical_sensor_orientation_supervisor}
"Supervisor" is the framework for logical sensors(sensor fusion).\n
The Supervisor provides several APIs for controlling [Workers](@ref spritzer_logical_sensor_orientation_worker).\n


#### APIs # {#spritzer_logical_sensor_orientation_supervisor_apis}
Orientation Supervisor provides Compass* API.\n
It has the following 4 APIs.

| APIs   | Description                                          | Corresponding APIs                  |
|:-------|:-----------------------------------------------------|:------------------------------------|
| Create | Create a class instance to communicate with workers. | [CompassCreate](@ref CompassCreate) |
| Open   | Load Orientation library and boot up as worker task. | [CompassOpen](@ref CompassOpen)     |
| Write  | Send data to Orientation worker task.                | [CompassWrite](@ref CompassWrite)   |
| Close  | Destory Orientation worker task.                     | [CompassClose](@ref CompassClose)   |

*See also.*

- @ref logical_compass

##### Data format for Compass
Logical sensor Compass requires Accel and Magnetometer data by specified format as below.\n
It have to be written by [CompassWrite](@ref CompassWrite).\n

![Accel data format for Compass](compass/compass_require_data_format_accel.png)\n

*Magnetometer data format is same as Accel data, but the "Unit" is "micro Tesla(uT)".*\n

#### Sequence # {#spritzer_logical_sensor_orientation_supervisor_sequence}
Call APIs according to the following sequence diagram.

 - Initialize\n
   Compass Initialize sequence.
   ![Initial Sequence](compass/Compass_Initial_sequence.png)

 - Sensing(for Accel)\n
   Compass Sensing sequence. (In case of Accel data recieved.)
   ![Sensing Sequence(Accel)](compass/Compass_Sensing_sequence_Accel.png)

 - Sensing(for Magnetometer)\n
   Compass Sensing sequence. (In case of Magnetometer data recieved.)
   ![Sensing Sequence(Magnetometer)](compass/Compass_Sensing_sequence_Mag.png)


#### Configurations & build # {#spritzer_logical_sensor_orientation_supervisor_config}

To use Orientation, please set configs like below by "make menuconfig".
~~~
$ make menuconfig
~~~

~~~
[System Type]
  [CXD56xx Peripheral Support]
    [SPI3]                                 <= Y
    [Sensor Control Unit Support]          <= Y (*1)

[RTOS Features]
  [Stack and heap information]
    [Idle thread stack size]               <= 4096

[Device Drivers]
  [Sensor Device Support]                  <= Y
    [Asahi AK09912 Compass Sensor support] <= Y
    [Bosch BMI160 Sensor support]          <= Y

[Memory Management]
  [Number of memory regions]               <= 2
  [Enable Tile Allocator]                  <= Y

[Library Routines]
  [Standard Math library]                  <= Y
  [Have C++ compiler]                      <= Y
  [Have C++ initialization]                <= Y
  [ASMP support library]                   <= Y

[Application Configuration]
  [Examples]
    [Compass sensor example]               <= Y(*2)
  [Sensor Utilities]
    [Sensor manager]                       <= Y
      [Logical Sensors]
        [Compass]                          <= Y
  [Memory Utilities]
    [MEMORY_MANAGER]                       <= Y
    [MEMUTILS_MESSAGE]                     <= Y
~~~

(\*1):If you'd like to use Sencor Control Unit, it's neccesary to activate this config.\n
(\*2):To build the Compass sample, activate this consig.(!!Cannnot activate other Examples at sametime.!!)

Alternatively, you can set it by entering the following command.
~~~
$ cd sony_apps
$ ./tools/configure.sh corvo/compass
~~~

When you are done, please enter make and build the sample.
~~~
$ make
~~~

#### Memory Utirity Library Configurations & Layout # {#spritzer_logical_sensor_orientation_supervisor_memutil_config}

##### Message Library Configuration

~~~
MsgQuePool
 # ID,              n_size  n_num  h_size  h_num
  ["MSGQ_SEN_MGR",  40,     8,     0,      0],

See "sony_apps/examples/logical_sensor/compass/config/msgq_layout.conf" for each definition.
~~~

##### Memory Manager (Intelligent Fix Pool) Configuration

~~~
FixedAreas
 # name,                  device,     align,        size,         fence
  ["SENSOR_WORK_AREA",    "SHM_SRAM", U_TILE_ALIGN, 0x0001c400,   false],
  ["MSG_QUE_AREA",        "SHM_SRAM", U_MSGQ_ALIGN, 0x00003240,   false],
  ["MEMMGR_WORK_AREA",    "SHM_SRAM", U_STD_ALIGN,  0x00000200,   false],
  ["MEMMGR_DATA_AREA",    "SHM_SRAM", U_STD_ALIGN,  0x00000100,   false],

CAUTION: Fixed Areas can not be customized

PoolAreas
 # name,                       area,               type,  align,       pool-size,  seg, fence,  spinlock
  ["SENSOR_DSP_CMD_BUF_POOL",  "SENSOR_WORK_AREA", Basic, U_STD_ALIGN, 0x70,       8,   false,  ""],
  ["COMPASS_RST_CMD_BUF_POOL", "SENSOR_WORK_AREA", Basic, U_STD_ALIGN, 0x24,       4,   false,  ""],
  ["ACCEL_DATA_BUF_POOL",      "SENSOR_WORK_AREA", Basic, U_STD_ALIGN, 0xC0,       8,   false,  ""],
  ["MAG_DATA_BUF_POOL",        "SENSOR_WORK_AREA", Basic, U_STD_ALIGN, 0x60,       8,   false,  ""],

See "sony_apps/examples/logical_sensor/compass/config/mem_layout.conf" for each definition.
~~~

### Worker # {#spritzer_logical_sensor_orientation_worker}
The Worker runs on other core and analys sensor data.\n
And returns the processing result of various requested commands.

Compass worker requires data which shown below for analysis.
* Accel data (16Hz, 16samples/1sec)
* Magnetometer data (8Hz, 8samples/1sec)

#### APIs # {#spritzer_logical_sensor_orientation_worker_apis}
We provide the framework for a Orientation on worker.
It has 3 event for worker.

| Event | Description                         | Corresponding command  |
|:------|:------------------------------------|:-----------------------|
| init  | Initialize for Orientation.         | SensorInitOrientation  |
| exec  | Execute calculate on sensor data.   | SensorExecOrientation  |
| flush | End execute process of Orientation. | SensorFlushOrientation |

In execute, the following are prepared as Orientation execution commands.
| Command                                                 | Description                         |
|:--------------------------------------------------------|:------------------------------------|
| [ORIENTATION_CMD_UPDATE_ACCEL](@ref OrientationCmdType) | Update by acceleration sensor.      |
| [ORIENTATION_CMD_UPDATE_MAG](@ref OrientationCmdType)   | Update by magnetometer information. |

**DSP command setting method.**\n
An example of a packet set for each command is shown below.

- init\n
  ~~~{.c}
    SensorDspCmd dsp_cmd;

    dsp_cmd.header.sensor_type     = Compass;
    dsp_cmd.header.event_type      = InitEvent;
  ~~~

- exec(for Accel)\n
  ~~~{.c}
    SensorDspCmd dsp_cmd;

    dsp_cmd.header.sensor_type     = Compass;
    dsp_cmd.header.event_type      = ExecEvent;
    dsp_cmd.exec_aesm_cmd.cmd_type = ORIENTATION_CMD_UPDATE_ACCEL;
  ~~~

- exec(for Magnetometer)\n
  ~~~{.c}
    SensorDspCmd dsp_cmd;

    dsp_cmd.header.sensor_type     = Compass;
    dsp_cmd.header.event_type      = ExecEvent;
    dsp_cmd.exec_aesm_cmd.cmd_type = ORIENTATION_CMD_UPDATE_MAG;
  ~~~

- flush\n
  ~~~{.c}
    SensorDspCmd dsp_cmd;

    dsp_cmd.header.sensor_type     = Compass;
    dsp_cmd.header.event_type      = FlushEvent;
  ~~~

*See also.*

- @ref logical_compass

#### Sequence # {#spritzer_logical_sensor_orientation_worker_sequence}
Call APIs according to the following sequence diagram.

- Initialize\n
  ![Orientation Initialize](DSP_Compass_Init.png)

- Execute\n
  ![Orientation Execute](DSP_Compass_Exec.png)

- Terminate\n
  ![Orientation Terminate](DSP_Compass_Flush.png)


---------------------
## TRAM (TRAnsportation Mode recognition)  # {#spritzer_logical_sensor_transportation_mode}
The configuration diagram of the logic sensor shown below is shown.\n
- Configuration of TRAM\n
  ![TRAM structural diagram](sensor_transport_mode_configuration_diagram.png)

### Supervisor # {#spritzer_logical_sensor_transportation_mode_supervisor}
"Supervisor" is the framework for logical sensors(sensor fusion).\n
The Supervisor provides several APIs for controlling [Workers](@ref spritzer_logical_sensor_transportation_mode_worker).\n


#### APIs # {#spritzer_logical_sensor_transportation_mode_supervisor_apis}
TRAM Supervisor provides Transportation Mode Recognition* API.\n
It has the following 4 APIs.

| APIs   | Description                                          | Corresponding APIs                          |
|:-------|:-----------------------------------------------------|:--------------------------------------------|
| Create | Create a class instance to communicate with workers. | [TramCreate](@ref TramCreate)               |
| Open   | Load TRAM library and boot up as worker task.        | [TramOpen](@ref TramOpen)                   |
| Write  | Send data to TRAM worker task.                       | [TramWrite](@ref TramWrite)                 |
| Close  | Destroy TRAM worker task.                            | [TramClose](@ref TramClose)                 |

*See also.*

- @ref logical_transport

##### Data format for Transport mode 
Logical sensor Transport mode requires Accel and Magnetometer, Barometer data by specified format as below.\n
It have to be written by [TramWrite](@ref TramWrite).\n

![Accel data format for TramWrite](tram/Tram_require_data_format_accel.png)\n

*Magnetometer data format is same as Accel data, but the "Unit" is "micro Tesla(uT)".*\n

![Barometer data format for TramWrite](tram/Tram_require_data_format_barometer.png)\n

#### Sequence # {#spritzer_logical_sensor_tram_supervisor_sequence}
Call APIs according to the following sequence diagram.

 - Initialize\n
   Tram Initialize sequence.
   ![Initial Sequence](tram/Tram_Initial_sequence.png)

 - Sensing(for Accel)\n
   Tram Sensing sequence. (In case of Accel data received.)
   ![Sensing Sequence(Accel)](tram/Tram_Sensing_sequence_Accel.png)

 - Sensing(for Magnetometer)\n
   StepCounter Sensing sequence. (In case of Magnetometer data received.)
   ![Sensing Sequence(Magnetometer)](tram/Tram_Sensing_sequence_Magnetmeter.png)

 - Sensing(for Barometer)\n
   StepCounter Sensing sequence. (In case of Barometer data received.)
   ![Sensing Sequence(Barometer)](tram/Tram_Sensing_sequence_Barometer.png)

#### Configurations & build # {#spritzer_logical_sensor_transportation_mode_supervisor_config}

To use StepCounter, please set configs like below by "make menuconfig".
~~~
$ make menuconfig
~~~

~~~
[System Type]
  [CXD56xx Peripheral Support]
    [SPI3]                        <= Y
    [Sensor Control Unit Support] <= Y

[RTOS Features]
  [Stack and heap information]
    [Idle thread stack size]      <= 4096

[Device Drivers]
  [Sensor Device Support]                            <= Y
    [Bosch BMI280 Barometic Pressure Sensor support] <= Y
    [Asahi AK09912 Compass Sensor support]           <= Y
    [Bosch BMI160 Sensor support]                    <= Y

[Memory Management]
  [Number of memory regions]      <= 2
  [Enable Tile Allocator]         <= Y

[Library Routines]
  [Standard Math library]         <= Y
  [Have C++ compiler]             <= Y
  [Have C++ initialization]       <= Y
  [ASMP support library]          <= Y

[Application Configuration]
  [Examples]
    [TRAM sensor example]         <= Y(*1)

  [Sensor Utilities]
    [Sensor manager]              <= Y
      [Logical Sensors]
        [transport mode]          <= Y
  [Memory Utilities]
    [MEMORY_MANAGER]              <= Y
    [MEMUTILS_MESSAGE]            <= Y
~~~

(\*1):To build the TRAM sensor sample, activate this config.(!!Cannnot activate other Examples at the same time.!!)

Alternatively, you can set it by entering the following command.
~~~
$ cd sony_apps
$ ./tools/configure.sh corvo/tram
~~~

When you are done, please enter make and build the sample.
~~~
$ make
~~~

#### Memory Utirity Library Configurations & Layout # {#spritzer_logical_sensor_tram_supervisor_memutil_config}

##### Message Library Configuration

~~~
MsgQuePool
 # ID,              n_size  n_num  h_size  h_num
  ["MSGQ_SEN_MGR",  40,     8,     0,      0],

See "sony_apps/examples/logical_sensor/tram/config/msgq_layout.conf" for each definition.
~~~

##### Memory Manager (Intelligent Fix Pool) Configuration

~~~
FixedAreas
 # name,                  device,     align,        size,         fence
  ["SENSOR_WORK_AREA",    "SHM_SRAM", U_TILE_ALIGN, 0x0001c400,   false],
  ["MSG_QUE_AREA",        "SHM_SRAM", U_MSGQ_ALIGN, 0x00003240,   false],
  ["MEMMGR_WORK_AREA",    "SHM_SRAM", U_STD_ALIGN,  0x00000200,   false],
  ["MEMMGR_DATA_AREA",    "SHM_SRAM", U_STD_ALIGN,  0x00000100,   false],

CAUTION: Fixed Areas can not be customized

PoolAreas
 # name,                      area,               type,  align,       pool-size,  seg, fence,  spinlock
  ["SENSOR_DSP_CMD_BUF_POOL", "SENSOR_WORK_AREA", Basic, U_STD_ALIGN, 0x70,       8,   false,  ""],
  ["ACCEL_DATA_BUF_POOL",     "SENSOR_WORK_AREA", Basic, U_STD_ALIGN, 0xF00,      8,   false,  ""],
  ["MAG_DATA_BUF_POOL",       "SENSOR_WORK_AREA", Basic, U_STD_ALIGN, 0x1E0,      8,   false,  ""],
  ["PRESS_DATA_BUF_POOL",     "SENSOR_WORK_AREA", Basic, U_STD_ALIGN, 0xA0,       8,   false,  ""],
  ["TEMP_DATA_BUF_POOL",      "SENSOR_WORK_AREA", Basic, U_STD_ALIGN, 0xA0,       8,   false,  ""],

See "sony_apps/examples/logical_sensor/tram/config/mem_layout.conf" for each definition.
~~~

### Worker # {#spritzer_logical_sensor_transport_mode_worker}
The Worker runs on other core and analys sensor data.\n
And returns the processing result of various requested commands.\n

Tram worker requires data which shown below for analysis.
* Accel data (64Hz, 320samples/5sec)
* Magnetometer data (8Hz, 40samples/1sec)
* Barometer data (8Hz, 40samples/1sec)

#### APIs # {#spritzer_logical_sensor_transport_mode_worker_apis}
We provide the framework for a TRAM on worker.
It has 3 event for worker.

| Event | Description                       | Corresponding command |
|:------|:----------------------------------|:----------------------|
| init  | Initialize for TRAM.              | SensorInitTram        |
| exec  | Execute calculate on sensor data. | SensorExecTram        |
| flush | End execute process of TRAM.      | SensorFlushTram       |

In execute, the following are prepared as TRAM execution commands.
| Command       | Description                    |
|:--------------|:-------------------------------|
| TramSensorAcc | Update accel data.             |
| TramSensorMag | Update magnetometerda          |
| TramSensorBar | Update barometer data          |

**DSP command setting method.**\n
An example of a packet set for each command is shown below.

- init\n
  ~~~{.c}
    SensorDspCmd dsp_cmd;

    dsp_cmd.header.sensor_type     = TransportationMode;
    dsp_cmd.header.event_type      = InitEvent;
  ~~~

- exec(for Accel)\n
  ~~~{.c}
    SensorDspCmd dsp_cmd;

    dsp_cmd.header.sensor_type     = TransportationMode;
    dsp_cmd.header.event_type      = ExecEvent;
  ~~~

- exec(for Mag)\n
  ~~~{.c}
    SensorDspCmd dsp_cmd;

    dsp_cmd.header.sensor_type     = TransportationMode;
    dsp_cmd.header.event_type      = ExecEvent;
  ~~~

- flush\n
  ~~~{.c}
    SensorDspCmd dsp_cmd;

    dsp_cmd.header.sensor_type     = TransportationMode;
    dsp_cmd.header.event_type      = FlushEvent;
  ~~~

*See also.*

- @ref logical_tram

**Sample data setting method.**\n

  Please set up to 320 samples as below.\n

  Please set 40 sample as below.\n

  Please set 40 sample as below.\n

#### Sequence # {#spritzer_logical_sensor_transport_mode_worker_sequence}
Call APIs according to the following sequence diagram.

- Initialize\n

- Execute\n

- Terminate\n
  
---------------------
# Code Examples # {#spritzer_sensor_util_example}

## AESM sample

- [sony_apps/examples/logical_sensor/aesm/aesm_main.cxx](\ref examples/logical_sensor/aesm/aesm_main.cxx)
- [sony_apps/examples/logical_sensor/aesm/accel_sensor.c](\ref examples/logical_sensor/aesm/accel_sensor.c)
- [sony_apps/examples/logical_sensor/aesm/gnss_sensor.c](\ref examples/logical_sensor/aesm/gnss_sensor.c)

## Arm Gesture sample

- [sony_apps/examples/logical_sensor/arm_gesture/arm_gesture_main.cxx](\ref examples/logical_sensor/arm_gesture/arm_gesture_main.cxx)
- [sony_apps/examples/logical_sensor/arm_gesture/accel_sensor.c](\ref examples/logical_sensor/arm_gesture/accel_sensor.c)

## TAP sample
- [sony_apps/examples/tap/tap_main.cxx](\ref examples/tap/tap_main.cxx)

## Orientation sample

- [sony_apps/examples/logical_sensor/compass/compass_main.cxx](\ref examples/logical_sensor/compass/compass_main.cxx)
- [sony_apps/examples/logical_sensor/compass/accel_sensor.c](\ref examples/logical_sensor/compass/accel_sensor.c)
- [sony_apps/examples/logical_sensor/compass/magnetometer_sensor.c](\ref examples/logical_sensor/compass/magnetometer_sensor.c)

## TRAM sample



\example examples/logical_sensor/aesm/aesm_main.cxx
\example examples/logical_sensor/aesm/accel_sensor.c
\example examples/logical_sensor/aesm/gnss_sensor.c
\example examples/logical_sensor/arm_gesture/arm_gesture_main.cxx
\example examples/logical_sensor/arm_gesture/accel_sensor.c
\example examples/logical_sensor/compass/compass_main.cxx
\example examples/logical_sensor/compass/accel_sensor.c
\example examples/logical_sensor/compass/magnetometer_sensor.c
\example examples/tap/tap_main.cxx


