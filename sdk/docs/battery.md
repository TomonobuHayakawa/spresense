Battery charger and fuel gauge {#battery_doc}
============================
[TOC]

# Battery Charger and fuel gauge {#battery_function}

SDK support CXD5247 charging and monitoring battery function

 - Initial parameter
 - Battery Charger
 - Battery fuel gauge
 - Power monitor

Battery Charger and fuel gauge feature are activated with Kconfig
~~~
  Device Drivers  --->   
    [*] Power Management Support  --->  
    [*]   Battery Charger support     
    [*]     CXD5247 Battery charger support    
    [*]   Battery Fuel Gauge support   
    [*]     CXD5247 Battery fuel gauge support    
~~~

--------------------------------------
## Initial parameter (SBL)

Boot Loader (SBL) initialize battery charger parameters in boot seaquence.

There are 10 parameters in SBL.

| Parameter | Selection |
|:-----|:--------|
| charger         | OFF/ON|
| charge_voltage  | 4000/4050/4100/4150/4200/4250/4300/4350/4400|
| charge_current  | 100/500|
| charge_ifin     | 10/20/30/40/50|
| boot_voltage    | 3000-3800|
| system_voltage  | 3000-3700|
| recharge_voltage| 400/350/300/250|
| low_temp_charge | OFF/WEAK/ON|
| high_temp_charge| OFF/WEAK/ON|
| charger_reset   | OFF/ON|

- charger<br>
This parameter enable charger function.<br>
In default charger=OFF<br>
SBL don't access charger in default.<br>
If the system use battery charger function, set parameter charger=ON.<br>
<br>
If charger parameter is ON, user can change other charging parameters.<br>
<br>
- charge_voltage<br>
This parameter set the max voltage of charging.<br>
In default charge_voltage=4200(mv)<br>
Change this parameter in accordance with battery specifications.<br><br>

- charge_current<br>
This parameter set the charging current in boot sequence.<br>
In default charge_current=100(mA)<br>
100 means 0.2C charging and 500 means 1C charging.<br>
If the max charging current determined by the external resistor value is smaller than 100mA, or USB-Bus power always supply 500mA then set this parameter 500 (1C).<br>
SBL use this parameter for charging before FW boot (load and start).<br><br>

- charge_ifin<br>
While charging, charger supplys power until the battery voltage become more than specified voltage and do not exeed the maximum current. If the battery voltage increase to the specifed voltage, charger reduce the supply current. If the reduced current is less than charge_ifin value, charging complete.<br><br>
Set ifin parameter sufficiently smaller than max charging current, otherwise charging will stop immediately and battery voltage does not reach specivied voltage.<br>
If ifin parameter is small, charging time will be longer.<br><br>

- boot_voltage<br>
This parameter is battery threshold voltage to start loading and executing FW.<br>
In default boot_voltage=3401(mV)<br>
In low battery voltage case, dropping voltage may cause system down before starting FW.<br>
SBL will wait until the battery voltage exceeds the boot_voltage.<br>
If the system consume much power in boot sequence, set this parameter large, according to characteristics of the battery and consuming power.<br><br>

- system_voltage<br>
This parameter is battery threshold voltage to SYSTEM go down/up.<br>
In default system_voltage=3402(mV)<br>
If the battery voltage is lower than system_voltage-60mV, system RESET will assert, and if the battery voltage is higher than system_voltage, system RESET deassert.<br>
Change this parameter in accordance with the characteristics of the battery and peripheral devices.<br>
If the system consumes low power and battery capacity in low voltage is large, and peripheral devices doesn't require 3300mV, change this parameter lower and extend operating time.<br><br>

- recharge_voltage<br>
This parameter is theshold voltage to restart charging.<br>
After completion of charging, charger doesn't restart charging until battery voltage drops below the charge_voltage-recharge_voltage.<br>
In default recharge_voltage=400(mV)<br>
The charger charge upto 4200mV and re-charge from 3800mV.<br>
If the USB connection is out, and re-connect, charger can start charging again while the battery voltage is over re-charge voltage.<br><br>

- low_temp_charge<br>
This parameter change charger action in Low (0-10 degree) temperature<br>
| low_temp_charge | Description |
|:-----|:--------|
| OFF  | Do not charge in 0-10 degree |
| WEAK | Charge with half of specified current in 0-10 degree |
| ON   | Charge with specified current and voltage in 0-10 degree |

If the battery doesn't support  0-10 degree charging, set this parameter "OFF".<br>
Regardless of this parameter charger stop charging under 0 degree.<br><br>

- high_temp_charge<br>
This parameter change charger action in High (45-60 degree) temperature<br>
| high_temp_charge | Description |
|:-----|:--------|
| OFF  | Do not charge in 45-60 degree |
| WEAK | Charge with -0.15V than specified voltage in 45-60 degree |
| ON   | Charge with specified current and voltage in 45-60 degree |
If the battery doesn't support 45-60 degree charging, set this parameter "OFF".<br>
Regardless of this parameter charger stop charging over 60 degree.<br><br>

- charger_reset   : (OFF/ON)<br>
Charger HW takes in the parameter when USB cable connected.<br>
So in some condition, updated charger parameter doesn't effect in next charging action. <br>
If this parameter is "ON", charger is forcely reset in boot seaquence, and parameter is forcely updated regardless of USB connection.<br>
This causes boot timer longer (about 1 second).<br><br>
In default charger_reset=OFF<br>


--------------------------------------
## Battery Charger  {#battery_charger}

User can control charger with /dev/charger device file.

There are these ioctl functions

| function | IOCTL |  Description | value| type|
|:-----|:--------|:-----|:-----|:--------|
| voltage      | BATIOC_VOLTAGE|Set max voltage for charging| 4000/../4400mV(1)| int |
| current      | BATIOC_CURRENT| Set charging current| 0mA/100mA/500mA/Reset(2)| int |
| temperature  | BATIOC_TEMPERATURE| Set/Get charger action in High or Low degree| ON/OFF/WEAK(3)| int*|
| recharge     | BATIOC_RECHARGE| Set/Get Charging restart threshold| Vmax-0.4V/../Vmax-0.25V(4)| int*|
| ifin         | BATIOC_CHG_IFIN| Set/Get Charging finish current| 50/40/30/20/10mA(5)| int* |
| temperature table| BATIOC_CHG_TEMPTABLE| Set/Get Temperature table| (6)| cxd5247_temperature_table_t* |
| get_voltage  | BATIOC_GET_VOLTAGE| Get max voltage setting| (1)| int* |
| get_current  | BATIOC_GET_CURRENT| Get charging current setting| (2)| int* |
| get_porttype | BATIOC_GET_PORTTYPE| Get USB port type| SDP/DCP| int* |

See Example program "charger".

### parameters

parameters are defined in cxd5247_charger.h

- (1) voltage (BATIOC_VOLTAGE)<br>
CXD5247_VOLT_4400 : 4.4V<br>
CXD5247_VOLT_4350 : 4.35V<br>
CXD5247_VOLT_4300 : 4.3V<br>
CXD5247_VOLT_4250 : 4.25V<br>
CXD5247_VOLT_4200 : 4.2V<br>
CXD5247_VOLT_4150 : 4.15V<br>
CXD5247_VOLT_4100 : 4.1V<br>
CXD5247_VOLT_4050 : 4.05V<br>
CXD5247_VOLT_4000 : 4.0V<br>
<br>
- (2) current (BATIOC_CURRENT)<br>
CXD5247_CURR_LIM_2MA   : no charge<br>
CXD5247_CURR_LIM_100MA : charge up to 100mA (0.2C)<br>
CXD5247_CURR_LIM_500MA : charge up to 500mA (1C)<br>
CXD5247_CURR_RESET     : Reset charger (Stop functions)<br>
<br>
- (3) temperature (BATIOC_TEMPERATURE)<br>
CXD5247_TEMP_HIGH_ON   : Continue charge in the condition of over 45 degree<br>
CXD5247_TEMP_HIGH_OFF  : Stop charge in the condition of over 45 degree<br>
CXD5247_TEMP_HIGH_WEAK : Charge VMAX-0.15V in the condition of over 45 degree<br>
CXD5247_TEMP_LOW_ON    : Continue charge in the condition of under 10 degree<br>
CXD5247_TEMP_LOW_OFF   : Stop charge in the condition of under 10 degree<br>
CXD5247_TEMP_LOW_WEAK  : Charge 0.5C in the condition of under 10 degree<br>
<br>
- (4) recharge (BATIOC_RECHARGE)<br>
CXD5247_RECH_250 : Restart charge if battery voltage become under VMAX-0.25V<br>
CXD5247_RECH_300 : Restart charge if battery voltage become under VMAX-0.30V<br>
CXD5247_RECH_350 : Restart charge if battery voltage become under VMAX-0.35V<br>
CXD5247_RECH_400 : Restart charge if battery voltage become under VMAX-0.40V<br>
<br>
- (5) ifin (BATIOC_CHG_IFIN)<br>
CXD5247_IFIN_50MA : Charging finish current = 50mA<br>
CXD5247_IFIN_40MA : Charging finish current = 40mA<br>
CXD5247_IFIN_30MA : Charging finish current = 30mA<br>
CXD5247_IFIN_20MA : Charging finish current = 20mA<br>
CXD5247_IFIN_10MA : Charging finish current = 10mA<br>
<br>
- (6) temperature table (BATIOC_CHG_TEMPTABLE)<br>
~~~{.c}
typedef struct cxd5247_temperature_table_s
{
  int T60; /* 60 degrees */
  int T45; /* 45 degrees */
  int T10; /* 10 degrees */
  int T00; /*  0 degrees */
} cxd5247_temperature_table_t;
~~~
Thermister parameter in 4 points.<br>
| variable | degree | default value |
|:-----|:--------|:----|
|T60|60 degrees| 744|
|T45|45 degrees|1188|
|T10|10 degrees|2773|
|T00| 0 degrees|3199|
<br>
To get current table, set T60 value to -1.<br>
<br>

### Sample code to set charging current

~~~{.c}
#include <sys/ioctl.h>
#include <fcntl.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/power/cxd5247_charger.h>

/* Charge with 100mA limit */
{
  int ret;
  int current = CXD5247_CURR_LIM_100MA;
  int fd = open("/dev/charger", O_RDOK | O_WROK);
  if(fd < 0)
    {
      return -1;
    }
  ret = ioctl(fd, BATIOC_CURRENT, (uintptr_t)&current);
  close(fd);
  return ret;
}
~~~

After first connection of USB cable, charging will start by 100mA/500mA current setting.
After start charging, charging will stop with 0mA current setting.

If connected USB port is DCP, CXD5247 start charging automatically without software control.

In default setting, charging voltage is 4.2V and re-charge voltage is -0.40V.
So after start charging, charging will stop if battery voltage increase over 4.2V,
and doesn't restart charging until battery voltage drop under 3.80V.

After USB disconnect and connection, charger return to initial state.

Max charging current is determined by resistance value on board.
Please establish it with the specification of the battery. <br>
-The Current mode 500MA, charge up to specified current determined by the resistance.<br>
-The Current mode 100MA, charge up to 1/5 of specified current.<br>
Charging current must more than 40mA, otherwise charger can't charge to full charge.

If charger detect temperature is under 0 degree or over 60 degree, it stop charging automatically.
User can set charger action between 0 degree to 10 degree and between 45 degree to 60 degree with temperature ioctl.

--------------------------------------
## Battery Fuel Gauge {#battery_gauge}

User can get fuel gauge information with /dev/gauge device file.

There are these ioctl functions

| function | IOCTL | Description | type| unit/value|
|:-----|:--------|:--------|:-----|:-----|
| state        | BATIOC_STATE      | Get charger status    | int| IDLE/CHARGING/DISCHARGING|
| health       | BATIOC_HEALTH     | Get battery status    | int| GOOD/DISCON/COLD/OVERHEAT/UNUSUAL|
| voltage      | BATIOC_VOLTAGE    | Get voltage of battery| b16_t| mV|
| current      | BATIOC_CURRENT    | Get current of battery| b16_t| 0.1mA|
| temperature  | BATIOC_TEMPERATURE| Get temperature       | b16_t| ADC value(see example code)|

See Example program "charger".

### Sample code to read battery voltage

~~~{.c}
#include <sys/ioctl.h>
#include <fcntl.h>
#include <nuttx/power/battery_gauge.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/power/cxd5247_charger.h>

/* Read Voltage */
{
  int ret;
  short vol;
  int fd = open("/dev/gauge", O_RDOK | O_WROK);
  if (fd < 0)
    {
      return -1;
    }
  ret = ioctl(fd, BATIOC_VOLTAGE, (uintptr_t)&vol);
  close(fd);
}
~~~

The power chip CXD5427 will Reset System if battery voltage is under 3.4V.
So please charge the battery if the battery voltage decrease near 3.4V.
High clock execution can't be done in low voltage, because power consuming processing cause instantaneous drop of voltage and reset may be happen.

--------------------------------------
## Sample program for Battery Charger and Fuel Gauge  {#charger_example}

How to build and use it, see sony_apps/examples/charger/README.txt

~~~
 Application Configuration  --->     
    Examples  --->   
   [*] Charger example    
~~~

Charger example program monitor battery voltage and current every second.

Simply execute charger with no parameter.
~~~
nsh> charger
Hello, charger + gauge start!!
  0:00:09.410 4168 mV,    -7.0 mA,   -  (1898: 29) : DISCHARGING
  0:00:10.470 4168 mV,    -5.8 mA,   -  (1898: 29) : DISCHARGING
  <time>     <voltage>   <current>  <%><temperature> <status>
~~~

Charger example program can automatically start and stop charging with option parameters.

charger [measure_low_voltage] [log_size] [measure_interval]

| Parameter | Description |
|:-----|:--------|
| measure_low_voltage | start charging if battery voltage become less than measure_low_voltage|
| log_size | record charging/discharging voltage/current pair logs in RAM and show summary if charging complete or start charging.|
| measure_interval | measuring interval in seconds (in default: 1 second)|

For example.
~~~
nsh> charger 3350 180000 2
~~~

Charging start if battery voltage become less than 3350mV and charging will stop if charging complete, repeatedly.
Charging current is 1C in default. Press '1' key to make charging current 0.2C or press '5' key to make charging current  1C.<br>
If charge with 1C mode, connect USB bus which can supply 500mA.

Log size should greater than charging and dis-charging time.
With this example, program store 360000 seconds (100 hour) changes.

If charging complete the following message appear.

~~~
----------------------------------------
Log=5250, total=30749033
  0 : 3544, 199.2 :       7059 /   30749033
  1 : 3671, 201.1 :     313169 /   30749033
  2 : 3750, 201.3 :     618931 /   30749033
....
 98 : 4174,  38.1 :   30134353 /   30749033
 99 : 4174,  33.0 :   30442810 /   30749033
100 : 4175,  27.2 :   30749033 /   30749033
----------------------------------------
<%>   <mV>   <mA> <sub total Watt> <total Watt>
~~~

--------------------------------------
## Power Monitor {#power_monitor}

fuel gauge driver support intermittent measurement of voltage and current

It records up to 127 event in reserved SRAM.
Measurement interval is from 120ms to 60000ms(1 min).

There are 4 ioctl functions<br>
<br>
| function | Description |
|:-----|:--------|
| BATIOC_MONITOR_ENABLE | Enable and disable measurement |
| BATIOC_MONITOR_STATUS | Get status of power monitor |
| BATIOC_MONITOR_SET    | Clear data in SRAM and integrated capacity|
| BATIOC_MONITOR_GET    | Get data from SRAM |
<br>
- Record data format
~~~{.c}
typedef struct {
    uint16_t index;
    uint16_t timestamp;
    uint16_t voltage;
    int16_t  current;
} power_monitor_log_t;
~~~
| Parameter | Range | Description |
|:-----|:--------|:-----|
|index     | 0..32767 |measurement index, increment and wrap|
|timestamp | 0..65535 |measurement time, 1/8sec unit|
|voltage   | 0..4586  |mV|
|current   | -31250..31250|16uA unit:-500mA..500mA|
<br>
index starts from 0 and wrap 32767 to 0.<br>
timestamp is RTC counter, so it isn't cleared after reboot. It counts up in 1/8sec, so it wrap in 8192 second (about 136 min)<br>
<br>
- parameter of BATIOC_MONITOR_ENABLE
~~~{.c}
typedef struct {
    int on;
    int interval;
    int threshold_volt;
    int threshold_current;
} power_monitor_enable_arg_t;
~~~
| Parameter | Range | unit | Description |
|:-----|:--------|:-----|:-----|
|on               | 0/1 |disable/enable| measurement enable/disable|
|interval         | 120..60000 |ms|measurement interval|
|threshold_volt   | 0..5000  |mV|measure cumulative power threshold voltage|
|threshold_current| 0..500000|uA|measure charging time threshold of current|
<br>
(1) enable monitor without integrated capacity<br>
~~~{.c}
 on=1
 interval=120
 threshold_volt=5000
~~~
(2) enable monitor with integrated capacity<br>
~~~{.c}
 on=1
 interval=120
 threshold_volt=3700
~~~
(3) disable monitor<br>
~~~{.c}
 on=0
 interval=don't care
 threshold=don't care
~~~
<br>
If power_monitor enabled, it record measured data repeatedly until power_monitor disabled. If the system go to cold sleep, measurement stops while sleep, but measurement continues after wakeup.
If the system go to deep sleep, all recoreded data will vanish and measurement doesn't continue after wakeup.
<br>
<br>
- parameter of BATIOC_MONITOR_STATUS
~~~{.c}
typedef struct {
    int bRun;
    int index;
    int latest;
    int total_watt;
    int total_time;
} power_monitor_status_arg_t;
~~~
MONITOR_STATUS return error before starting monitor.<br>
| Parameter | Range | unit | Description |
|:-----|:--------|:-----|:-----|
|bRun      | 0/1 |-| measurement task is running |
|index     | 0-0x7FFFFFFF|-|latest index of measured data(start from 0)|
|latest    | 0..65535  |1/8sec|latest timestamp of measured data|
|total_watt| signed 32bit|uA|integrated capacity (sum of i*v while voltage greater than threshold_volt)|
|total_time| signed 32bit|1/8sec|charging time while current greater than threshold_current and voltage greater than threshold_volt|
<br>
- parameter of BATIOC_MONITOR_SET
~~~{.c}
typedef struct {
    int clearBuf;
    int clearSum;
} power_monitor_set_arg_t;
~~~
(1) clear all reacorded data<br>
~~~{.c}
 clearBuf=1
 clearSum=0
~~~
(2) clear integrated capacity and charging time<br>
~~~{.c}
 clearBuf=0
 clearSum=1
~~~
<br>
- parameter of BATIOC_MONITOR_GET
~~~{.c}
typedef struct {
    power_monitor_log_t *ptr;
    int index;
    int size;
} power_monitor_get_arg_t;
~~~
(1) get 4 data from index=1 <br>
~~~{.c}
ptr = pointer of recieve data
index=1
size=4
~~~
<br>
Check current index by calling MONITOR_STATUS before calling MONITOR_GET<br>
If specified index when calling MONITOR_GET isn't in recorded data, return error<br>
<br>
See power_monitor example program.<br>

## Sample program for power_monitor {#power_monitor_example}

How to build and use it, see sony_apps/examples/power_monitor/README.txt

~~~
 Application Configuration  --->
    Examples  --->
   [*] "Power Monitor" example
~~~

(1) start monitoring
~~~
nsh> power_monitor start 120 3700
~~~
(2) stop monitoring
~~~
nsh> power_monitor stop
~~~
(3) get status
~~~
nsh> power_monitor status
~~~
(4) clear data
~~~
nsh> power_monitor set 1 1
~~~
(5) get data (from index=1 to 4)
~~~
nsh> power_monitor get 1 4
~~~
(6) get all data
~~~
nsh> power_monitor get
~~~
(7) monitoring
~~~
nsh> power_monitor monitor 1 10
~~~
Example of display (discharge) : (power_monitor start 120 3700 100000)
~~~
 6118.750 : 4090mV(4090 .. 4091) -  1.7mA( -2640 ..  -1184) :     -1019,       0
 6119.750 : 4089mV(4088 .. 4090) -  2.2mA( -3520 ..  -1184) :     -1791,       0
 6120.750 : 4088mV(4086 .. 4090) -  2.9mA( -5280 ..  -1472) :     -2755,       0
<timestamp> <volt><-   range  -> <current><-   range    ->   <total_watt>
~~~
Example of display (charge) : (power_monitor start 120 3700 100000)
~~~
 6148.250 : 4090mV(4090 .. 4091) -  2.1mA( -2352 ..  -1472) :    -30571,       0
 6149.250 : 4178mV(4091 .. 4212) + 89.6mA( -2352 .. 130080) :      7333,       0
 6150.375 : 4211mV(4211 .. 4212) +120.0mA(119248 .. 121296) :     47669,       1
 6151.375 : 4211mV(4211 .. 4212) +118.2mA(117200 .. 119536) :     87424,       2
 6152.375 : 4212mV(4212 .. 4212) +116.7mA(116016 .. 117200) :    126715,       3
<timestamp> <volt><-   range  -> <current><-   range    ->   <total_watt><total_time>
~~~
