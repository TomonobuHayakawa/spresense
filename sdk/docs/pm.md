Power Management {#powermnagement_doc}
============================
[TOC]

# Power State # {#power_state}

--------------------------------------
## Overview {#power_state_overview}

Spritzer SDK has been supported the following Power State.
To realize the lower power consumption, there are three sleeping state, Hot, Cold and Deep Sleep.
Deep Sleep is the lowest power consumption, but it takes the longest time to resume from the state.
It also takes time to resume from Cold Sleep, but Cold Sleep can have more wakeup triggers than Deep Sleep.
Hot Sleep can keep the contents of memory with SRAM retention and it's unnecessary to load programs from storage like as SPI-Flash.
Therefore Hot Sleep has the shortest startup time. Each Detail is described in the following chapter.
- Run : Normal running state
- Idle: OS idle task state and WFI (Wait for Interrupt)
- Hot Sleep: Sleeping state with SRAM retentioned
- Cold Sleep: Sleeping state with SRAM powered off
- Deep Sleep: Only PMIC is powered on
- Power Off : Battery Detached

### Power State Chart Diagram

\image html pm_state.png
\n

### Power Domain Hierarchical Structure

\image html pm_domain.png

--------------------------------------
## Power State related API {#power_state_api}

### up_pm_get_bootcause() {#up_pm_get_bootcause}
- **Function Prototype:**
 - uint32_t up_pm_get_bootcause(void);
- **Description:**
 - Get the system boot cause.
 - Returned *boot cause* indicates a cause why the system launched from Power Off, Deep Sleep or Cold Sleep state.
 - This *boot cause* has a no connection with Hot Sleep.
  - In case of Hot Boot (wakeup from Hot Sleep), a interrupt handler of wakeup cause will be generally executed after boot-up.
- **Boot Cause / Boot Mask definitions:**
|Boot Cause / Boot Mask  |Boot Type|Maskable|Description                                                 |
|:-----------------------|:--------|:-------|:-----------------------------------------------------------|
|PM_BOOT_POR_NORMAL      |POR Boot |No      |Power On Reset when battery was attached for the first time |
|PM_BOOT_POR_DEADBATT    |POR Boot |No      |Battery was charged for over 3.4V from DeadBattery state    |
|PM_BOOT_WDT_REBOOT      |Reboot   |No      |Explicitly Self Reboot or System Watchdog expired           |
|PM_BOOT_WDT_RESET       |Reboot   |No      |Reset only Spritzer[*2]                                     |
|PM_BOOT_DEEP_WKUPL      |Deep Boot|Yes     |Detected WKUPL signal                                       |
|PM_BOOT_DEEP_WKUPS      |Deep Boot|Yes[*1] |Detected WKUPS signal                                       |
|PM_BOOT_DEEP_RTC        |Deep Boot|Yes[*1] |RTC Alarm expired                                           |
|PM_BOOT_DEEP_USB_ATTACH |Deep Boot|No      |USB Connected                                               |
|PM_BOOT_DEEP_OTHERS     |Deep Boot|No      |Reserved others cause occurred                              |
|PM_BOOT_COLD_SCU_INT    |Cold Boot|Yes     |Detected SCU Interrupt                                      |
|PM_BOOT_COLD_RTC        |Cold Boot|Yes     |RTC Alarm expired                                           |
|PM_BOOT_COLD_RTC_ALM0   |Cold Boot|Yes     |RTC Alarm0 expired(NuttX generally uses Alarm0)             |
|PM_BOOT_COLD_RTC_ALM1   |Cold Boot|Yes     |RTC Alarm1 expired                                          |
|PM_BOOT_COLD_RTC_ALM2   |Cold Boot|Yes     |RTC Alarm2 expired                                          |
|PM_BOOT_COLD_RTC_ALMERR |Cold Boot|Yes     |RTC Alarm Error occurred                                    |
|PM_BOOT_COLD_GPIO       |Cold Boot|Yes     |Detected GPIO interrupt                                     |
|PM_BOOT_COLD_SEN_INT    |Cold Boot|Yes     |Detected Sensor Interrupt (SEN_INT)                         |
|PM_BOOT_COLD_PMIC_INT   |Cold Boot|Yes     |Detected PMIC Interrupt                                     |
|PM_BOOT_COLD_USB_DETACH |Cold Boot|Yes     |USB Disconnected                                            |
|PM_BOOT_COLD_USB_ATTACH |Cold Boot|Yes     |USB Connected                                               |
 [*1]: Make it impossible to be disable both PM_BOOT_DEEP_WKUPS and PM_BOOT_DEEP_RTC<BR>
 [*2]: Basically not used, or might be used only in the HV(High Voltage)-only system or the system without CXD5427(AcaPulco)


### up_pm_get_bootmask() {#up_pm_get_bootmask}
- **Function Prototype:**
 - uint32_t up_pm_get_bootmask(void);
- **Description:**
 - Get the system boot mask.
 - Definitions of *boot mask* is the same as *boot cause* as described above.
 - Returned *boot mask* indicates whether each *boot cause* is enabled or not.
 - By default, all of the *boot cause* are enabled.
 - This *boot mask* is retained during Hot Sleeping or Cold Sleeping, but is reset in Deep Boot or POR Boot.
 - If you would like to prohibit the specified *boot cause*, clear *boot mask* by up_pm_clr_bootmask() before calling up_pm_sleep().


### up_pm_set_bootmask() {#up_pm_set_bootmask}
- **Function Prototype:**
 - uint32_t up_pm_set_bootmask(uint32_t mask);
- **Description:**
 - Enable the specified *boot cause*.
 - Return the updated *boot mask*.


### up_pm_clr_bootmask() {#up_pm_clr_bootmask}
- **Function Prototype:**
 - uint32_t up_pm_clr_bootmask(uint32_t mask);
- **Description:**
 - Disable the specified *boot cause*.
 - Return the updated *boot mask*.
- **Example:**
~~~~~~~~~~~~~~~{.c}
#include <arch/chip/pm.h>

  uint32_t bootmask;

  bootmask = up_pm_get_bootmask(); // Get the current bootmask
  printf("bootmask=0x%08x\n", bootmask); // Display the current bootmask

  bootmask = up_pm_clr_bootmask(PM_BOOT_COLD_USB_DETACH); // Disable wakeup by USB detached
  printf("bootmask=0x%08x\n", bootmask); // Display the updated bootmask

~~~~~~~~~~~~~~~


### up_pm_sleep() {#up_pm_sleep}
- **Function Prototype:**
 - int up_pm_sleep(enum pm_sleepmode_e mode);
- **Description:**
 - Enter sleep mode.
 - This function never returns.
- **Sleep Mode definitions:**
|Sleep Mode    |Description      |
|:-------------|:----------------|
|PM_SLEEP_DEEP |Enter Deep Sleep |
|PM_SLEEP_COLD |Enter Cold Sleep |

- **Example:**
 - If CONFIG_BOARDCTL_POWEROFF=y, board_power_off() API is activated.<BR>
   As a example of up_pm_sleep(), you can call up_pm_sleep(PM_SLEEP_DEEP) in board_power_off() function.<BR>
   So then 'poweroff' or 'shutdown' command on NutShell would be possible to enter Deep Sleep state instead of poweroff.<BR>
   Please see configs/covro/cxd56_power.c for details.


### up_pm_reboot() {#up_pm_reboot}
- **Function Prototype:**
 - int up_pm_reboot(void);
- **Description:**
 - System reboot.
 - This function never returns.
- **Example:**
 - If CONFIG_BOARDCTL_RESET=y, board_reset() API is activated.<BR>
   As a example of up_pm_reboot(), you can call up_pm_reboot() in board_reset() function.<BR>
   So then 'reboot' or 'shutdown' command on NutShell would be possible to reboot the system.<BR>
   Please see configs/covro/cxd56_power.c for details.


--------------------------------------
## Deep Sleep / Deep Boot {#deepsleep}

### Features:
 - CXD5602(Spritzer) is completely powered off.
 - CXD5427(AcaPulco) is deep sleep mode.
  - RTC time is retained if your system has a RTC XTAL.
  - GPO signal is retained.
   - If it's unnecessary to keep GPO on, GPO should set off for power saving before entering Deep Sleep state.
   - It must be care for current flow to powered off CXD5602 I/O pin.
  - Load Switch is off.

#### Power Consumption
 - Battery consumption current is about 2 uA on the reference board.

#### Sleep Conditions
 - Call up_pm_sleep(PM_SLEEP_DEEP)
 - WKUPL signal was asserted more than 3 seconds.

#### Wakeup Conditions
 - WKUPL signal was asserted more than 3 seconds.
 - WKUPS signal was asserted.
 - RTC alarm expired.
 - USB was connected.
  - If USB has been already connected, it's impossible to enter Deep Sleep state and reboot immediately.<BR>
    At this time, the *boot cause* is not PM_BOOT_DEEP_USB_ATTACH but PM_BOOT_POR_NORMAL.
    It's better not to call up_pm_sleep(PM_SLEEP_DEEP) when USB is connected.


--------------------------------------
## Cold Sleep / Cold Boot {#coldsleep}

### Features:
 - CXD5602(Spritzer)
  - PMU power domain is on. (Must)
  - SCU power domain may be on. (Option)
  - CXD5602 I/O pin is activated.
  - BackupSRAM is retained.
  - PWD_APP is powered off.
   - PWD_APP SRAM is powered off.
   - PWD_APP_DSP(all of the application cores) is powered off.
   - PWD_APP_SUB is powered off.
   - PWD_APP_AUD is powered off.
 - CXD5427(AcaPulco) is normal running.
  - RTC time is retained if your system has a RTC XTAL.
  - GPO signal is retained.
  - Load Switch is off by software control.

#### Power Consumption
 - Battery consumption current is about 100-200 uA on the reference board.

#### Sleep Conditions
 - Call up_pm_sleep(PM_SLEEP_COLD)

#### Wakeup Conditions
 - SCU interrupt was asserted.
 - Sensor interrupt was asserted.
 - PMIC interrupt was asserted.
  - WKUPS signal was asserted.
  - Low Battery notification was notified.
 - Any GPIO signal was asserted.
 - RTC alarm expired.
 - USB was connected or disconnected.


--------------------------------------
## Hot Sleep / Hot Boot {#hotsleep}

### Features:
 - CXD5602(Spritzer)
  - NuttX CPU is powered off
   - SRAM that NuttX CPU was using is retention
  - PWD_APP_DSP domain (DSPs) is controlled by NuttX CPU
  - PWD_APP_SUB domain is controlled by NuttX CPU
  - PWD_APP_AUD domain is controlled by NuttX CPU
 - CXD5427(AcaPulco) is normal running.
  - RTC time is retained if your system has a RTC XTAL.
  - GPO signal is retained.
  - Load Switch is retained.

#### Power Consumption
 - Battery consumption current is more than about 400-500 uA on the reference board.

#### Sleep Conditions
 - Enable CONFIG_CXD56_HOT_SLEEP
  - By below parameters, you can control the transition conditions.
   - CXD56_HOT_SLEEP_WAIT_COUNT (milli-seconds)
   - CXD56_HOT_SLEEP_THRESHOLD (milli-seconds)
  - Acquire/Release wakelock
   - If at least one wakelock has been acquired, the system can't enter Hot Sleep state.
 1. When NuttX OS is idle task, power manager counts the spending time on idle task.
  - If any interrupt occurrs, the above counter is cleared and OS dispatches any interrupt handler or task from idle task.
 2. If the above counter reachs longer than CXD56_HOT_SLEEP_WAIT_COUNT, power manager calculates the expected idle time by next OS time event.
 3. If expected idle time > CXD56_HOT_SLEEP_THRESHOLD, then the system enters to Hot Sleep
  - If any wakelock is acquired, then it prevents the system from entering Hot Sleep.

#### Wakeup Conditions
 - Any interrupt to NuttX CPU was asserted.
 - For example, the type of interrupt are:
  - UART1 (Debug Terminal) Rx interrupt
  - UART2 (Communication port with the externel device like as Bluetooth chip) Rx interrupt
  - SCU (Sensor Control Unit) Watermark interrupt
  - USB VBUS (Connected) interrupt
  - USB VBUS Negative (Disconnected) interrupt
  - PMIC (Low Battery Notification or WKUPS) interrupt
  - GPIO interrupt(s) that have been registered and used before hot sleeping
  - Inter-CPU communication from other CPUs to NuttX CPU
  - NuttX OS timer event interrupt


--------------------------------------
# Power saving control # {#power_saving_control}

SDK provides following two Power saving control feature.

 - CPU working frequency control
 - CPU power down control


--------------------------------------
## CPU working frequency control #  {#power_saving_control_freqlock}

SDK provides following three frequency states.

 - High voltage state
   - CPU frequency is about PLL 160MHz.
   - CPU will consume a lot of power.

 - Low voltage state
   - CPU frequency is about PLL 32MHz.
   - CPU will consume less power.

 - RCOSC state
   - CPU frequency is about RCOSC 8MHz.
   - CPU will consume very little power.

SDK's default setting is High voltage state, can not transfer to other state. <br>
Transition will be possible with the following configuration settings. <br>
In this case frequency states after boot up is RCOSC state.
~~~
 [System Type]
   [Release CPU frequency lock] <= Y
~~~

State transition is requested by following API call.

 - Transition to High voltage state
~~~{.c}
  struct pm_cpu_freqlock_s lock;

  lock.flag = PM_CPUFREQLOCK_FLAG_HV;
  up_pm_acquire_freqlock(&lock);

~~~
 <br>
 - Leave High voltage state
~~~{.c}
  up_pm_release_freqlock(&lock);
~~~
   - Please specify same pointer as parameter used for transition to High voltage state.
   - If no request from other users, will transition to RCOSC state.
   - If another user's request, will transition to request state. <br>
 <br>
 - Transition to Low voltage state
~~~{.c}
  struct pm_cpu_freqlock_s lock;

  lock.flag = PM_CPUFREQLOCK_FLAG_LV;
  up_pm_acquire_freqlock(&lock);
~~~
  - If other user request High voltage, stay High voltage until all user release the lock. <br>
 <br>
 - Leave Low voltage state
~~~{.c}
  up_pm_release_freqlock(&lock);
~~~
   - Please specify same pointer as parameter used for transition to Low voltage state.
   - If no request from other users, will transition to RCOSC state.
   - If another user's request, will transition to request state. <br>


--------------------------------------
## CPU power down control #  {#power_saving_control_sleep}

SDK provide ability to Application CPU(Main M4F) and GNSS CPU(M4F) power down when each CPU IDLE state. <br>
If timer expire or external interrupt occurs, CPU power supply restart, then memory state is retained.

### Application CPU(Main M4F) #  {#power_saving_control_sleep_app}

SDK's default setting is not Application CPU power down when CPU IDLE state. <br>
Application CPU power down will be possible with the following configuration settings. <br>
~~~
 [System Type]
   [Enable HOT Sleep] <= Y
~~~

Application CPU power down can be regulated by calling following API.
~~~{.c}
struct pm_cpu_wakelock_s lock;

lock.count = 0;
up_pm_acquire_wakelock(&lock);
~~~
Release regulation of Application CPU power down calls following API. <br>
Please specify same pointer as parameter used for regulate.
~~~{.c}
up_pm_release_wakelock(&lock);
~~~

### GNSS CPU(M4F) #  {#power_saving_control_sleep_gps}

This feature can be set when GNSS feature is enabled.

SDK's default setting is not GNSS CPU power down when CPU IDLE state. <br>
GNSS CPU power down will be possible with the following configuration settings. <br>
~~~
 [System Type]
   [CXD56xx Peripheral Support]
     [GNSS device] <= Y
       [GNSS settings]
         [Enable GNSS HOT Sleep] <= Y
~~~

