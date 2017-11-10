GNSS  {#gnss_doc}
========

[TOC]

GNSS is a function to measure and calculate the current position.
To use this function, you need to connect the antenna for GNSS to the system.

# Key features:

- Supports the multi satellite types of GPS, GLONASS with embedded GNSS hardware.
- Asynchronous positioning processing and notification function with application.
- Geofence sub device.

These features will be controlled by application through POSIX file manipulation
functions as open, close, read, seek, and ioctl.
Application open GNSS device file "/dev/gps", next starts by ioctl command,
and get position data by read function.

GNSS has multiple commands of ioctl described in this document.
NuttX ioctl function have three argments. For GNSS, 2nd "req" is GNSS
command, 3rd "arg" is paramter as in/out data.

---

# Configuration {#gnss_configuration}

The configuration related to GNSS is shown below.

When using GNSS, set CONFIG_CXD56_GNSS to y.

~~~
[System Type]
  [CXD56xx Peripheral Support]
    [GNSS device] (CXD56_GNSS)  = Y
~~~

When using NMEA convert library /ref gnss_utilities_nmea, set CONFIG_CXD56_GNSS, CONFIG_LIBM and own config CONFIG_GPSUTILS_CXD56NMEA_LIB to y.

~~~
[System Type]
  [CXD56xx Peripheral Support]
    [GNSS device] (CXD56_GNSS) = Y
[Library Routines]
  [Standard Math library] (LIBM) = Y
[Application Configuration]
  [GPS Utilities]
    [Support CXD56xx gnss NMEA convert library] (GPSUTILS_CXD56NMEA_LIB) = Y
~~~

It is necessary to set for CONFIG_CXD56_GNSS and own config CONFIG_EXAMPLES_GNSS of Example "gnss".

~~~
[System Type]
  [CXD56xx Peripheral Support]
    [GNSS device] (CXD56_GNSS) = Y
[Application Configuration]
  [Examples]
    [GNSS positioning example] (EXAMPLES_GNSS) = Y
~~~

It is necessary to set configs for NMEA convert library and own config CONFIG_EXAMPLES_GNSS_ATCMD of Example "gnss".

~~~
[System Type]
  [CXD56xx Peripheral Support]
    [GNSS device] (CXD56_GNSS) = Y
[Library Routines]
  [Standard Math library] (LIBM) = Y
[Device Drivers]
  [USB Device Driver Support]
    [USB Modem (CDC/ACM) support] = Y
[Application Configuration]
  [GPS Utilities]
    [Support CXD56xx gnss NMEA convert library] (GPSUTILS_CXD56NMEA_LIB) = Y
[Application Configuration]
  [Examples]
    [GNSS CXD5603 @command emulator example] (EXAMPLES_GNSS_ATCMD) = Y

~~~

It is necessary to set for CONFIG_CXD56_GNSS and own config CONFIG_EXAMPLES_GNSS_FACTORY of Example "gnss_factory".
Set EXAMPLES_GNSS_FACTORY_SVID according to test environment.
To run the factory test directly, change [Application entry point].
The value of the execution result (cn and doppler) is multiplied by 1000000 and it is output.

~~~
[System Type]
  [CXD56xx Peripheral Support]
    [GNSS device] (CXD56_GNSS) = Y
[Application Configuration]
  [Examples]
    [GNSS FACTORY test] (EXAMPLES_GNSS_FACTORY) = Y
    [FACTORY TEST svid] (EXAMPLES_GNSS_FACTORY_SVID) = 1
[RTOS Features]
  [Tasks and Scheduling]
    [Application entry point]
      set 'gnss_factory_test'
~~~

For the configuration of the example gnss_display, please show REAME file.
- [sony_apps/examples/gnss_display/README.txt](\ref examples/gnss_display/README.txt)

In any case the following values are configurable.\n
'Disable power control of the LNA device from GNSS device driver' is a setting
about the LNA power control through BSP layer. When there is a mechanism of LNA power
control in BSP, please set it to N.
'Enable GNSS HOT Sleep' is an option to enable the GNSS CPU stop function when GNSS is not working,
contributing to lower power consumption.
'GNSS backup file name' and 'GNSS CEP file name' are files used by GNSS, \n
and it is necessary to change the file name according to the file-system of the SDK-system.

~~~
[System Type]
  [CXD56xx Peripheral Support]
    [GNSS setting]
      [Disable power control of the LNA device from GNSS device driver] = N
      [Enable GNSS HOT Sleep] = N
      [GNSS backup file name] = '/mnt/spif/gnss_backup.bin'
      [GNSS CEP file name] = '/mnt/vfat/gnss_cep.bin'
~~~

GNSS device driver notifies APP via POSIX 's poll or signal mechanism every positioning output.\n
The maximum number of waits can be independently configured with the following settings. The default value is four for poll and three for signal.\n
For details on how to notify, see ["Position calculation notification"](#gnss_position_calculation_notification).

~~~
[System Type]
  [CXD56xx Peripheral Support]
    [GNSS setting]
      [GNSS max poll waiters] = 4
      [GNSS max signal receivers] = 3
~~~

---

# Device control #  {#gnss_device_control}

This section describes the control commands for positioning with GNSS.

IOCTL commands that control GNSS are listed in [IOCTL commands](/ref gnss_ioctl).
A GNSS device can be opened as a file from multiple applications at the same time,
but because there is only one GNSS device itself, IOTCL commands will be accepted in order.
In other words, regardless of the issued application, the commands are executed
in turn in the GNSS device. As a result, GNSS may operate with unintended settings
unless it arbitrates issuing commands between applications.

## Startup #  {#gnss_startup}

Select the type of satellite used for start positioning before by #CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM.
The positioning cycle(period) can also be specified by #CXD56_GNSS_IOCTL_SET_OPE_MODE.

When performing Hot Start, it is necessary to specify the current position and time.
Refer to ["Using GNSS backup data"](#gnss_backupram) for the setting for Warm Start and Hot Start.

## Start positioning #  {#gnss_start_positioning}

You can specify the start mode by #CXD56_GNSS_IOCTL_START.
The table below shows the difference in start mode.\n
Note) These values ​​are general reference values ​​and do not guarantee performance.

Start mode | IOCTL command parameter | TTFF     | Information to use
---------- | ----------------------- | -------- | ------------------
Cold Start | #CXD56_GNSS_STMOD_COLD  | > 45 sec | Discard all current position, time and sattelite orbital information, and start positioning from the beginning.
Warm Start | #CXD56_GNSS_STMOD_WARM  | > 20 sec | Use current position, time and only almanac. Do not use Ephemeris.
Hot Start  | #CXD56_GNSS_STMOD_HOT   | >= 1 sec | Use current position, time, almanac and ephemeris.

If there is luck of necessary information for Warm or Hot Start,
GNSS automatically starts cold start using positioning other than Cold start.
So it is usual to specify Hot Start.

## Stop positioning #  {#gnss_stop_positioning}

Execute the stop command #CXD56_GNSS_IOCTL_STOP.
It takes a few hundred milliseconds to stop the positioning from issuing a command.

## Position calculation notification # {#gnss_position_calculation_notification}

There are two types of notifications, poll method and signal method.

In the case of poll, you can be notified with general poll usage.
It is possible to wait at poll with the number of tasks specified by the configuration value CONFIG_CXD56_GNSS_NPOLLWAITERS.

On the other hand, in order to receive notification of position calculation with signal,
use the IOCTL command #CXD56_GNSS_IOCTL_SIGNAL_SET to set it after opening the GNSS device.
The signal specified by this command is issued at each event such as positioning,
the application can wait it with sigwaitinfo or handle it with the handler set by sigaction.
If it is unnecessary to receive signal, this setting can be cleared
by setting the parameter enable field to 0 with #CXD56_GNSS_IOCTL_SIGNAL_SET.
It is possible to receive signals with the number of tasks specified
by the configuration value CONFIG_CXD56_GNSS_NSIGNALRECEIVERS.

In the [application exsample program](\ref examples/gnss/gnss_main.c),
an example is shown by switching the notification method
with CONFIG_EXAMPLES_GNSS_USE_SIGNAL.

---

# For faster positioning #  {#gnss_fast_positioning}

This section explains the backup data and other informations to be fixed the position quickly
using the HotStart mode.

The figure below shows the standard HotStart flow to issue ioctl commands.

\image html gnss_hotstart_saving_backupdata.png
<div class="figure_annot">Fig. Standard flow for hot start using backupdata</div>

## Using GNSS backup data ## {#gnss_backupram}

The receiver position, ephemeris, almanac, TCXO offset and other information required
for hot start are included in the backup data. Even if the main power supply is disconnected,
the backup RAM is held by the backup power supply.
In addition, ephemeris, almanac and TCXO offset can save to the file on the flash memory
or other file system by the ioctl command. If saving the backup data, restore it to RAM automatically
when booting system from power OFF, the GNSS subsystem can start positioning using a hot start.

### Power condition and backup data

In system power ON state, the GPS positioning operation is underway and the receiver position,
ephemeris, almanac, TCXO offset value, etc. are stored.
In sleep state, the power is kept suppling to the backup RAM and real-time clock.
It is remained in a condition required for hot start in this state.
In deep sleep state, the backup data is lost. The real-time clock is kept in PMIC.
In power OFF state, the backup data and the real-time clock operation are lost.

### Saving backup data

To save backup data, a SDK application will send the ioctl command: #CXD56_GNSS_IOCTL_SAVE_BACKUP_DATA
Then GNSS deivce will save to the file as the name CONFIG_CXD56_GNSS_BACKUP_FILENAME.

Please be note about the life of the flash memory if using this function many times to save data to the flash ROM.
It is recommend that data is saved only before the system power OFF
or deep sleep because it is minimized saving frequency to avoid damage about flash write.

### Invalid backup data

Saved backup data is corrupted in rare case.
The system rejects invalid data by checksum calculation and initializes data.
Then the subsystem will start positioning using cold start instead of hot start automatically.

### Expired data

The ephemeris and almanac have expiration. Expired data will be ignored
and it spent many seconds to fix the position as cold start.
Ignored data will be overwritten by new data while positioning.

### Using assist data

With the CEP assist data, HotStart is possible at the initial start when no ephemeris is accumulated or even when it has expired.
To use CEP data, a SDK application will place the file as the name CONFIG_CXD56_GNSS_CEP_FILENAME,
and send the ioctl command: #CXD56_GNSS_IOCTL_OPEN_CEP_DATA and #CXD56_GNSS_IOCTL_CLOSE_CEP_DATA
The CEP file can be downloaded from the dedicated server. Please tell SDK support if necessary.\n
CEP setting is possible while positioning is stop.

The following figure shows the standard CEP flow.
\image html gnss_CEP_standard_flow.png
<div class="figure_annot">Fig. Standard CEP flow</div>

The following figure shows the CEP DATA update flow.
\image html gnss_CEP_data_update_flow.png
<div class="figure_annot">Fig. CEP DATA update flow</div>

## GPS time ## {#gnss_gpstime}

In order to do HotStart, it is necessary that the correct current time with an error within 60 seconds
is set in the GNSS device. It is set either by using the value automatically restored
from RTC_GPS where GPS time acquired at the previous positioning is held or by ioctl command from APP.

### Accuracy of RTC_GPS

When positioning stops, RTC_GPS is updated to GPS time.
The error of GPS time in RTC_GPS is the error of the RTC clock itself.
Therefore, GPS time counting by RTC_GPS will be shifted while stopping positioning.
When the period from positioning stop to restart is short,
GPS time with less error can be used for Hot Start.

Immediately after turning on the system power supply, RTC_GPS indicates "0h 6 - Jan - 1980".
When positioning is done even once, RTC_GPS counts up based on the GPS time.
When no time setting is made, positioning calculation is performed based on RTC_GPS.

### Set time by the command

To set the current time to the GNSS device a SDK application will send the ioctl command: #CXD56_GNSS_IOCTL_SET_TIME

## Current location ## {#gnss_currentlocation}

In order to do HotStart, it is nessary to set the current location.
If APP dose not know current location and no set it to the GNSS device,
GNSS device will cacluate based on last position in do HotStart.

### Set location by the command

To set the current location to the GNSS device a SDK application will send
the ioctl command: #CXD56_GNSS_IOCTL_SET_RECEIVER_POSITION_ELLIPSOIDAL
or #CXD56_GNSS_IOCTL_SET_RECEIVER_POSITION_ORTHOGONAL

# For acurate positioning # {#gnss_acurate_positioning}

In this section it is described about the field in the positioning data
to be known in order to perform accurate positioning.

## Indication in receiver positioning data ## {#gnss_receiver_accuracy}

It is the explanations about fields of the GNSS receiver data.

### Field posDop, velIdx of struct #cxd56_gnss_receiver_s

- Type: struct #cxd56_gnss_dop_s

- Sub fields: pDop, hDop, vDop, ewDop, nsDop as float

- Unit: none

posDop is DOP for position. velIdx is DOP of velocity.
Dilution of precision (DOP) is specified the effect of satellite geometry.
velIdx is calculated by multiplying the velocity DOP by the weight coefficients.
Prefix of each fields, 'p' is overall DOP, 'h' is horizontal, 'v' is vertical,
'ew' is East-West and 'ns' is North-Sourth one.

The smaller the value of DOP, the better the geometry is.
When the sky is narrow like the downtown, or when the number of satellites operating is small
or the positioning in the area, the direction of the satellite that appears in the sky may be biased.
When pDOP is 2 or less, it is considered that the satellite is uniformly present enough
and geometrically correct position can be calculated.
On the other hand, when pDOP exceeds 5, it is difficult to calculate the correct position.

In case of continuing to use positioning with a large DOP,
it is necessary to design a use case, assuming that the positioning result is low precision.

### Field posAcc of struct #cxd56_gnss_receiver_s

- Type: struct #cxd56_gnss_var_s

- Sub fields: hVar, vVar as float

- Unit: meter

These values are square root of error covariance of calculation of position and velocity.
These show effect of noise and other inaccuracies to position filter.
Prefix of each fields, 'h' is horizontal accuracy and 'v' is vertical.

PosAcc shows the standard deviation of the position error.
Satellite signal noise, mutipath effect, and DOP affect this position error accuracy.
If the accuracy of the error is higher than the system requirement,
in order to improve this value it is necessary to change to a more sensitive antenna
and wait for the use of the positioning results until the satellite number increases,
so it is necessary to improve the reception performance.

## Indication in satellite positioning data ##  {#gnss_satellite_accuracy}

It is the explanations about fields of the GNSS receiver data.

### Field sigLevel of struct #cxd56_gnss_sv_s

- Type: float

- Sub fields: none

- Unit: dBHz

This value represents the C/N ratio(CN, CNR, Carrier-to-noise ratio) of the GNSS signal,
which shows the signal reception status.
The larger the value, the more stable GNSS positioning can be done.
On the other hand, if there is a noise source near the receiver or there is an obstacle around it,
the signal value will be small and stable positioning will not be possible.
CN is different for each satellite.

In general it is desirable that there are five or more satellites receiving signals
at 30 dBHz or higher for stable positioning.

____________________________________

# Utilities # {#gnss_utilities}

## NMEA converter # {#gnss_utilities_nmea}

As a gpsutils we prepared a library to convert binary format positioning data read from CXD56xx GNSS device to NMEA sentence.

First register the sentence storage buffer management
and output three callback functions in the library.
Next, when passing struct #cxd56_gnss_sv_s type positioning data read from the GNSS device
to the conversion function, it converts it to a sentence.
Please set CONFIG_MLIB to y for this.

For details, please show \ref #gnss_nmea .

Please also refer to the application example \ref gnss_example_nmea.

____________________________________

# Geofence # {#gnss_geofence}

## Key features ## {#geofence_keyfeatures}

Geofence is the function to be used to send a notification regarding a pre-defined geofence Region. 
- Each Region can be defined by Latitude, Longitude, and a Radius. 
- The user can define up to twenty Regions.

\image html geofence_notification.png
<div class="figure_annot">Fig. Geofence transition notification</div>

There are three types of notifications: "ENTER", "DWELL", and "EXIT".
The distance between the device and the center of each region is calculated every position update.
Users can define dwelling period.<br>

After adding the Region, the GNSS core continue to monitor the transition.
When GNSS core recognizes a state change, it sends a notification to APP.

## Configuration ## {#geofence_configuration}


When using Geofence, set GNSS device and Geofence Support to y.

~~~
[System Type]
  CXD56xx Peripheral Support -->
    [*] GNSS device
    [*]   Geofence Support
    
~~~

## Add Region and setup options ## {#geofence_addregion}
Applications open /dev/geofence to use the Geofence feature.<br>
To add a Region and set the Geofence options, use the ioctl command of /dev/geofence.

### CXD56_GEOFENCE_IOCTL_ADD
This is the command to add Region. <br>
The user can define up to twenty Regions. 
Each Region can be defined by Latitude, Longitude, and a Radius.<br>
The north latitude and east longitude directions are "+" values 
so when specifying the receiver position using a south latitude and 
west longitude, add a "-" (minus) sign in front to the values.<br>

### CXD56_GEOFENCE_IOCTL_SET_MODE
This is the command to define dead zone and dwelling period. <br>
Dwelling period means that the DWELL state determination period. <br>

## State transition ## {#geofence_transition}

\image html geofence_transition.png
<div class="figure_annot">Fig. Geofence state transition</div>

## Dead zone ## {#geofence_deadzone}
Users are allowed to define dead zone at a circle boundary in order to reduce excess of notification by position fluctuation.<br>
When dead zone is defined <br>
- "ENTER" will be notified by inner circle <br>
- "EXIT" will be notified by outer circle <br>
State will not change when a device locates inside dead zone

\image html geofence_deadzone.png
<div class="figure_annot">Fig. Geofence Dead zone</div>


## Read Geofence transition data ## {#geofence_dataread}
The state change can be known by using poll method.<br>
The first judgment after adding the Region will always be notified.
After that, it will be notified only when a state is updated.<br>
#cxd56_geofence_status_s is transition data struct.

Please also refer to the application example \ref gnss_example_geofence.

____________________________________

# PVTLog # {#gnss_pvtlog}

## Key features ## {#pvtlog_keyfeatures}
PVTLog is a function to log Position, Velocity and Time information in the GNSS core. (PVT is an acronym for Position, Velocity and Time.)<br>
Up to 170 logs can be stored in the GNSS core.
When the number of logs is stored up to the prescribed number, APP will be notified using Signal method.<br>
For example, if settting it every 15 minutes, it is able to log store for 42 hours continuously independent of APP<br>
Please refer to struct #cxd56_pvtlog_s structure for logging data.

## Configuration ## {#pvtlog_configuration}

When using PVTLog, set GNSS device Support to y.

~~~
[System Type]
  CXD56xx Peripheral Support -->
    [*] GNSS device

~~~

## Start and stop log store ## {#pvtlog_startstop}
To start log store, use the #CXD56_GNSS_IOCTL_PVTLOG_START ioctl command.<br>
Use this command to set the log store interval the prescribed number of notifications.
The log store interval can be larger than the positioning interval.<br>
Automatically logs when positioning start.<br>
To stop log store, use the #CXD56_GNSS_IOCTL_PVTLOG_STOP ioctl command.

## Notification## {#pvtlog_notification}
When the log is saved up to the prescribed number, Signal method is used for notification to APP.
#CXD56_GNSS_SIG_PVTLOG is used for the signal setting.<br>
After notification, it is necessary to read log data by the next log preservation timing.
The data stored in the GNSS core is deleted when the first log writing after notification.<br>
Log data can be read at offset #CXD56_GNSS_READ_OFFSET_PVTLOG

## Notice ## {#pvtlog_notice}
The log data in the GNSS core is located in the backup RAM.
If stored data exists at the start of logging, it will be added without being deleted.
To start new log store, please start logging after executing delete command #CXD56_GNSS_IOCTL_PVTLOG_DELETE_LOG.<br>

Please also refer to the application example \ref gnss_example_pvtlog.
____________________________________

# Examples #  {#gnss_examples}

## Application examples ##  {#gnss_sample_apps}

### Notify positioning {#gnss_example_notify}

GNSS positioning notification.\n
Notify by poll or signal method switchable by Kconfig CONFIG_EXAMPLES_GNSS_USE_SIGNAL
or CONFIG_EXAMPLES_GNSS_USE_POLL.

- [sony_apps/examples/gnss/gnss_main.c](\ref examples/gnss/gnss_main.c)

### NMEA outpupt {#gnss_example_nmea}

Convert serial commands to IOCTL commands and output NMEA sentence.\n
Input serial commands starting with '@' character used in CXD5600 and CXD5602 from CDC/USB serial.
Output NMEA sentences from CDC/USB serial.

- [sony_apps/examples/gnss_atcmd/gnss_atcmd_main.c](\ref examples/gnss_atcmd/gnss_atcmd_main.c)
- [sony_apps/examples/gnss_atcmd/gnss_atcmd_parser.c](\ref examples/gnss_atcmd/gnss_atcmd_parser.c)
- [sony_apps/examples/gnss_atcmd/gnss_usbserial.c](\ref examples/gnss_atcmd/gnss_usbserial.c)

### Geofence transitions {#gnss_example_geofence}
- [sony_apps/examples/geofence/geofence_main.c](\ref examples/geofence/geofence_main.c)

### PVTLog {#gnss_example_pvtlog}
- [sony_apps/examples/gnss_pvtlog/gnss_pvtlog_main.c](\ref examples/gnss_pvtlog/gnss_pvtlog_main.c)

### FACTORY TEST {#gnss_example_factory}
- [sony_apps/examples/gnss_factory/gnss_factory_main.c](\ref examples/gnss_factory/gnss_factory_main.c)

### GNSS status on e-ink display {#gnss_example_display}
- [sony_apps/examples/gnss_display/gnss_display_main.c](\ref examples/gnss_display/gnss_display_main.c)

\example examples/gnss/gnss_main.c
\example examples/gnss_atcmd/gnss_atcmd_main.c
\example examples/gnss_atcmd/gnss_atcmd_parser.c
\example examples/gnss_atcmd/gnss_usbserial.c
\example examples/geofence/geofence_main.c
\example examples/gnss_pvtlog/gnss_pvtlog_main.c
\example examples/gnss_factory/gnss_factory_main.c
\example examples/gnss_display/README.txt
\example examples/gnss_display/gnss_display_main.c
