What is the Spritzer	{#what_is_spritzer}
========================================
[TOC]

# General #	{#what_is_spritzer_general}

Spritzer is a SoC developped for warable and IoT devices.\n
The semiconductor process is FD-SOI 28nm for Low Power.\n

Main features of the Spritzer are as follows.

- System Micro Control Unit. (ARM Coretex-M0+)
- Subsystem for independent GNSS.
  + GNSS supports GPS, GLONASS, SBAS, BeiDou and Galileo.
- Total 6 Application Micro Control Units. (ARM Coretex-M4Fs)
- Audio Codec Accelerator.
- Sensing Engine
  + This engine is a dedicated HW created to aquire data from externally connected sensors.  This is called **Sensor Control Unit**. This unit polls sensor data via connected BUSes like I2C, SPI etc, without MPUs. With this dedicated HW, lower power consumption is realized at all times during sensing.
  + This engine can control one SPI and two I2C interfaces.
  + 40KB FIFO for sensory data.
- A dedicated IC for Power Management Control, USB Charging, RTC and Audio functions, corded CXD5247.
\n
\n

\image html spritzer_main_features_sml.png
<div class="figure_annot">Fig. Spritzer Main Features</div>
\n
\n

# Spritzer Internal #	{#what_is_spritzer_internal}

Spritzer Internal Block diagram as follows.

\image html spritzer_internal_sml.png
<div class="figure_annot">Fig. Spritzer Internal</div>
\n

