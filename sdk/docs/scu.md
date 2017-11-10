SCU (Sensor Control Unit)  {#scu_doc}
-------------------------------------
[TOC]

SCU (Sensor Control Unit) is pick up the sensing data from any sensor device
connected via SPI and I2C bus in SCU device without CPU.

Key features:
- Take sensor data automatically by sequencer
- Decimation processing for taken sensor data (decimator)
- IIR filter application
- Event notifier

Above features are controlled by application.

SCU has SPI and two I2C buses, and 8 sequencers and 2 Decimators.

![SCU block diagram](scu_block.png)

____________________________________
# Configuration {#configuration}

~~~
[System Type]
  [CXD56xx Peripheral Support]
    [Sensor Control Unit Support] (CXD56_SCU)  = Y
      [SCU Decimator assignments]                (optional)
        ...
        (Decimator supported drivers)
        ...
  [uDMAC support] (CXD56_UDMAC)                = Y
  [Sequencer Sampling Predivider] (CXD56_SCU_PREDIV) = 64 (default)
  [SCU clock mode]                             = RCOSC (default)
  [SCU32K clock source]                        = RTC   (default)
  [SCU Debug] (CXD56_SCU_DEBUG)                = N     (default)
~~~

SCU Decimator assignments option can be assigned decimator supported sensor. User can be enabled 3 or above sensors but open() failed if 2 decimators already in use.

Sequencer Sampling Predivider affects sequencer sampling rate.

SCU clock mode selects its running clock.

SCU32K clock source selects source of SCU sampling base clock 32768Hz.

____________________________________
# Sequencer     {#sequencer}

SCU has two types of sequencer.
Normal sequencer takes sensor data periodically and store it to owned FIFO.
SCU support device driver can be read from its FIFO.

![Sequencer block diagram](scu_seq_block.png)

Each sequencer's FIFOs are separated into configurable size by #SCUIOC_SETFIFO.
That FIFO memory is total 40KB. Application must be set before use SCU supported driver.

\dontinclude mag/mag_main.c
\skip SCUIOC_SETFIFO
\until ;

Sequencer picks the sensor data by sampling rate itself, different from sensor
device. So application must be set sequencer sampling rate by #SCUIOC_SETSAMPLE.

\skip SCUIOC_SETSAMPLE
\until ;

The argument of #SCUIOC_SETSAMPLE is base clock divider in exponentation of 2.

`Sequencer sampling rate = 32768 / CONFIG_CXD56_SCU_PREDIV / (2 ^ n)`

For example, CONFIG_CXD56_SCU_PREDIV = 64 (default) and pass 2 to #SCUIOC_SETSAMPLE,
sequencer sampling rate becomes 128 Hz.

Application must be careful to sequencer sampling rate, it is different from sensor
sampling rate. So application must set sampling rate both of sensor and SCU, and their
combination.

Application can be taken a signal specified by #SCUIOC_SETWATERMARK.

SCU driver send a signal to application when number of sampling data in sequencer FIFO is reached to watermark value. For example, application set sampling rate to 128 Hz and set watermark to 128, then SCU signaling each 1 second period.

\dontinclude mag/mag_main.c
\skip wm.signo
\until SCUIOC_SETWATERMARK

When watermark signal received, the timestamp of the first sampling data is given by siginfo.

# Decimator     {#decimator}

Decimator extends a sequencer feature, add decimation processing. Additionally,
Decimator has 3 FIFOs for storing sensor data, so application takes one sensor device
data to give to different applications.

![Decimator block diagram](scu_deci_block.png)

## Decimation processing

Decimation process is applied for sample data by sequencer sampling rate, and decimation process applies CIC filter.
Application can be changed the decimation parameter with #SCUIOC_SETDECIMATION command with struct decimation_s.

\dontinclude decimator/decimator_main.c
\skip dec.ratio
\until SCUIOC_SETDECIMATION

`ratio` member specifies pick up ratio against sampling rate in exponentation of 2. If sampling rate is 64 Hz and `ratio` set to 1 ( 1 / (2 ^ 1) ), actual sampling rate becomes 32 Hz.
Note that set 0 to `ratio` become same rate with sampling rate, but decimation process will be applied CIC filter logic to sample data. So if you don't want to use CIC filter, set `forcethrough` member to 1.

## Amplification

Decimation process can be amplified sampling data to use `leveladj` member.
Amplified sampling data will be saturated to its data width.

For `leveladj` can be used constants listed in below:

- #SCU_LEVELADJ_X1 (x1)
- #SCU_LEVELADJ_X2 (x2)
- #SCU_LEVELADJ_X4 (x4)
- #SCU_LEVELADJ_X8 (x8)

## Supported devices

For using decimator, it needs a special support by sensor drivers.
Currently decimator supported sensors listed in below.

- BMI160 (gyro, accel)
- AK09912
- KX022
- BMI1422GMV

# Sensor data processing  {#sensor_data_processing}

SCU has a sensor data processing for taken sampling data.
Sensor data processing unit structured by IIR filter and Event detector. Application can be set up to 3 units. This unit is connected to each FIFO.

## IIR filter    {#iir_filter}

SCU can be applied two IIR filters for sampling data.

![IIR filter path](scu_IIR_filter_path.png)

- A = Apply to ALL
- F = Apply only FIFO
- E = Apply only Event detector

Application can be set two IIR filters to any combination of the positions.\n
FIFO is sequencer FIFO (also decimator). Event detector is special function of SCU, described in later.

Available combinations are:
- 2 to All
- All and FIFO
- All and Event detector
- 2 to FIFO
- FIFO and Event detector
- 2 to Event detector

IIR filter position settings are already defined by enum #filter_pos_e.

IIR filter can be configured by its coefficiencies.

![IIR filter block diagram](scu_IIR_filter.png)

Each coefficiencies are 34 bit fixed point number, s2.31 format. So IIR filter coefficiencies are structured in 32 bit (h) and 8 bit (l) values.

![struct iir_coeff_s h, l](scu_IIR_coeff.png)

31 bit of member @c h is S, it is a signed bit, 0 = plus, 1 = minus.

30 to 29 bits of member @c h are integer part.

28 to 0 of member @c h and 7 to 6 bits of member @c l are fractional part.

## Event detector    {#event_detector}

Event detector is a monitoring feature of sensor data. Application can be known the timing of sensor data to be taken.

Event detector counts input sensor data is high or low against configured threshold.
Event detector can handle 16 bits per sample and 1, 2 or 3 axis data. If 2 or 3 axis data is input, then normalize these data and count them.
And if counted value reached to configured value, event detector raise a interrupt and application take this event by signal with timestamp.

The normalize calculation is different when number of axis.

- For 1 axis (X axis):
<pre>
  norm = abs(x)
</pre>

- For 2 axis (X and Y axis):
<pre>
 norm = max(abs(x), abs(y)) * 123 / 128 + min(abs(x), abs(y)) * 51 / 128
</pre>

- For 3 axis (X, Y, Z):
<pre>
  L1   = max(abs(x), abs(y)) * 120 / 128 + min(abs(x), abs(y)) * 49 / 128
  norm = max(L1, abs(z)) + min(L1, abs(z)) * 44 / 128
</pre>

Event detector configured by struct scuev_notify_s. There is two detect configuration @a rise and @a fall, and each members have @a threshold, @a count0 and @a count1. SCU will be notified to application when input data counts reached to @a count0 + @a count1.

Event detector needs IIR filter output, so user must be configure IIR filter first.

![Event detection](scu_event.png)

Event detector processes:

- Counting higher value than threshold continuously
- If counter reached to @a count0, start actual couning
- If input data fall less than threshold before reached to @a count0, then stop and reset counts
- If total count is reached to @a count0 + @a count1, then raise rise event
- If count1 is zero, then notify when count reached to @a count0 immediately
- If threshold or count0 is zero, configuration is ignored

# Restrictions {#restrictions}

Decimator, IIR Filter and Event detector can be processed only for following data formats.

- 16 bit data per sample
- 1, 2 and 3 elements

Decimator can be take any data format, but processed only data duplication when data other than above format.

____________________________________
# For Driver developers {#developer}

SCU supported driver developers must be needs things in below:

- Understanding sequencer instruction
- Configure sensor device by special transfer API
- Open sequencer and connect with sensor driver
- Read sampling data from sequencer
- Pass ioctl commands to sequencer

## Understanding sequencer instruction  {#seq_instruction}

SCU sequencer has unique instruction, and send/receive the data via SPI/I2C buses by its instruction.
This instruction is 16 bit long, and commonly used to SPI and I2C buses.

Driver developer can be used #SCU_INST_SEND() and #SCU_INST_RECV() macros.

Some SCU API wants these instructions by array of uint16_t. This series of instructions indicates to sequencer how transfer with sensor device.
Additionally, special termination indicator (#SCU_INST_LAST) is needed at the last of instruction array.

Here is a short examples of instruction arrays.

\code
inst[0] = SCU_INST_SEND(0x00);                  // Send register address 0x00
inst[1] = SCU_INST_RECV(2) | SCU_INST_LAST;     // Read 0x00 and 0x01 address data, and indicate the last of instructions
\endcode

## Configure sensor device by special transfer API  {#oneshot}

First, driver initialize for target device. SCU supported driver must use scu_spitransfer() or scu_i2ctransfer() for accessing device registers.
scu_spitransfer() and scu_i2ctransfer() function needs to specified by sequencer instruction.

Ex. Register write access function

\dontinclude drivers/sensors/ak09912_scu.c
\skip void ak09912_putreg8
\until }

Actually, SPI and I2C bus contolled by SCU are can be access directly, but if accessing from SCU and sensor driver to the same bus and the same time, it is cause of bus confliction. So driver developer must use this APIs.

## Open/Close sequencer  {#open_close_sequencer}

SCU driver handles the sequencer object for all operations. So each sensor driver must hold this instance.

\dontinclude drivers/sensors/ak09912_scu.c
\skip seq_open
\until = g_seq;

In this sample, created sequencer instance stored into device data.

For decimator, it can be read from 3 FIFOs, so create 3 device file to read the same data from different applications. In this case, seq_open() must be called once, because decimator duplicates sensor data to 3 FIFOs. So developer must be count the number of references for avoid multiple initialization.

\dontinclude drivers/sensors/ak09912_scu.c
\skip g_refcnt == 0
\until g_refcnt++;

Additionally, SCU can be sleep when no sequencers running. seq_open() and seq_close() APIs are already supported to SCU sleep function. So, developer should call these APIs from open() and close() driver interfaces respectively, and also control device power mode. As a result, the driver can be reduce power consumption.

## Read sampling data from sequencer    {#seq_read}

For actual pick up sensor data by sequencer, also needs receive instructions to set by seq_setinstruction().

\dontinclude drivers/sensors/ak09912_scu.c
\skip seq_setinstruction
\until ;

\c g_ak09912inst is defined like this:
\dontinclude drivers/sensors/ak09912_scu.c
\skip g_ak09912inst
\until ;

Sequencer takes sensor data by this instruction set periodically. And its data stored to sequencer owned FIFO.

Driver can be read from its FIFO, use seq_read().

\dontinclude drivers/sensors/ak09912_scu.c
\skip seq_read
\until ;

This driver read sensor data and store to application buffer directly.

## Pass ioctl commands to sequencer  {#seq_ioctl}

SCU ioctl commands passing from each sensor driver is a simple.
Call seq_ioctl() from each drivers ioctl() if command is SCU IO commands. For determining which SCU IO command, use _SCUIOCVALID() macro.

e.g.
\dontinclude drivers/sensors/ak09912_scu.c
\skipline _SCUIOCVALID(cmd)
\until }

____________________________________

# Examples  {#examples}
## Driver initializer examples for SCU supported driver {#initializer}

- [configs/cxd56xx/src/cxd56_ak09912.c](\ref configs/cxd56xx/src/cxd56_ak09912.c)
- [configs/cxd56xx/src/cxd56_bmi160.c](\ref configs/cxd56xx/src/cxd56_bmi160.c)

## Driver examples for SCU supported driver  {#drivers}
- [drivers/sensors/ak09912_scu.c](\ref drivers/sensors/ak09912_scu.c)
- [drivers/sensors/bmi160_scu.c](\ref drivers/sensors/bmi160_scu.c)

## Application examples  {#sample_apps}
- [sony_apps/examples/mag/mag_main.c](\ref examples/mag/mag_main.c)
- [sony_apps/examples/gyro/gyro_main.c](\ref examples/gyro/gyro_main.c)
- [sony_apps/examples/decimator/decimator_main.c](\ref examples/decimator/decimator_main.c)

\example configs/cxd56xx/src/cxd56_ak09912.c
\example configs/cxd56xx/src/cxd56_bmi160.c
\example drivers/sensors/ak09912_scu.c
\example drivers/sensors/bmi160_scu.c
\example examples/mag/mag_main.c
\example examples/gyro/gyro_main.c
\example examples/decimator/decimator_main.c
