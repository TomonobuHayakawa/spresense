Board Specific Information {#boardspec_doc}
-------------------------------------------
[TOC]

# Pin Configurations  {#pinconfig_doc}

Pin Configurations are controlled by pinconfig driver under chip-specific layer.
- arch/arm/src/cxd56xx/cxd56_pinconfig.c
- arch/arm/src/cxd56xx/cxd56_pinconfig.h
- arch/arm/src/cxd56xx/chip/cxd5602_pinconfig.h

The pinconfig driver provides:
- I/O setting (as input enable, drive current strength or weak pull-up/pull-down) for each pin
- Select/Switch the function mode for each pin group

Basically the pin should be configured via the pin-related driver or board-specific layer.
So it's not possible to control directly from  application layer.

## Pin specification {#pinconfig_spec}

A pin has the multiple functions, and a function is decided by changing the ModeX(0-3).
Because the pins are categorized into groups, the mode is selected for each group.

- For example, regarding Group = I2C4,
 - When Mode0 is seleted, both I2C4_BCK and I2C4_BDT pins are GPIO.
 - When Mode1 is seleted, both I2C4_BCK and I2C4_BDT pins are I2C4(PMIC).

   _Note: It's not possible that only I2C4\_BCK selects GPIO without the change of I2C4_BDT._

For all of the pins, the initial mode at reset is Mode0.
Mode0 behaves as GPIO, but the input is disabled by default.
If you want to use as input pin of GPIO, then call cxd56_gpio_config(pin, true).
See the GPIO manual for details.

## Pin List {#pinconfig_list}

- **Pin** is defined as PIN_XXX in arch/arm/src/cxd56xx/cxd56_pinconfig.h.
- **WLCSP** is 100-pin package, and some pins are removed.
- **FCBGA** is 185-pin full package.
- **ModeX** describes the role of pin.
- **OD** means Open-Drain.

* Below is the list of pins that SDK can control.
|Group           |Pin            |WLCSP|FCBGA|Mode0  |Mode1          |Mode2          |Mode3          |
|:---------------|:--------------|:---:|:---:|:------|:--------------|:--------------|:--------------|
|I2C4            |I2C4_BCK       |○   |○   |GPIO   |I2C4(PMIC)     |―             |―             |
|　↑            |I2C4_BDT       |○   |○   |　↑   |　↑           |―             |―             |
|PMIC_INT        |PMIC_INT       |○   |○   |GPIO   |PMIC_INT       |PMIC_INT(OD)   |―             |
|RTC_IRQ_OUT     |RTC_IRQ_OUT    |―   |○   |GPIO   |RTC_IRQ_OUT    |RTC_IRQ_OUT(OD)|―             |
|AP_CLK          |AP_CLK         |○   |○   |GPIO   |AP_CLK         |PMU_WDT        |PMU_WDT(OD)    |
|GNSS_1PPS_OUT   |GNSS_1PPS_OUT  |―   |○   |GPIO   |GNSS_1PPS_OUT  |CPU_WDT        |CPU_WDT(OD)    |
|SPI0A           |SPI0_CS_X      |○   |○   |GPIO   |UART1(DBG)     |SPI0(CFG)      |SYS_MONOUT0    |
|　↑            |SPI0_SCK       |○   |○   |　↑   |　↑           |　↑           |SYS_MONOUT1    |
|SPI0B           |SPI0_MOSI      |―   |○   |GPIO   |I2C2(CFG)      |　↑           |SYS_MONOUT2    |
|　↑            |SPI0_MISO      |―   |○   |　↑   |　↑           |　↑           |SYS_MONOUT3    |
|SPI1A           |SPI1_CS_X      |○   |○   |GPIO   |SPI1(Flash)    |SPI0(CFG)      |SYS_MONOUT4    |
|　↑            |SPI1_SCK       |○   |○   |　↑   |　↑           |　↑           |SYS_MONOUT5    |
|　↑            |SPI1_IO0       |○   |○   |　↑   |　↑           |　↑           |SYS_MONOUT6    |
|　↑            |SPI1_IO1       |○   |○   |　↑   |　↑           |　↑           |SYS_MONOUT7    |
|SPI1B           |SPI1_IO2       |○   |○   |GPIO   |　↑           |―             |SYS_MONOUT8    |
|　↑            |SPI1_IO3       |○   |○   |　↑   |　↑           |―             |SYS_MONOUT9    |
|SPI2A           |SPI2_CS_X      |○   |○   |GPIO   |SPI2(HostIF)   |UART0(HostIF)  |I2C3(HostIF)   |
|　↑            |SPI2_SCK       |○   |○   |　↑   |　↑           |　↑           |　↑           |
|SPI2B           |SPI2_MOSI      |○   |○   |GPIO   |　↑           |UART0(HostIF)  |―             |
|　↑            |SPI2_MISO      |○   |○   |　↑   |　↑           |　↑           |―             |
|HIFIRQ          |HIF_IRQ_OUT    |○   |○   |GPIO   |HIF_IRQ_OUT    |HIF_IRQ_OUT(OD)|GNSS_1PPS_OUT  |
|HIFEXT          |HIF_GPIO0      |―   |○   |GPIO   |―             |―             |GPS_EXTLD      |
|SEN_IRQ_IN      |SEN_IRQ_IN     |○   |○   |GPIO   |SEN_IRQ_IN     |SYS_MONOUT0    |―             |
|SPI3_CS0_X      |SPI3_CS0_X     |○   |○   |GPIO   |SPI3_CS0_X     |SYS_MONOUT1    |―             |
|SPI3_CS1_X      |SPI3_CS1_X     |○   |○   |GPIO   |SPI3_CS1_X     |SYS_MONOUT2    |―             |
|SPI3_CS2_X      |SPI3_CS2_X     |○   |○   |GPIO   |SPI3_CS2_X     |SYS_MONOUT3    |―             |
|SPI3            |SPI3_SCK       |○   |○   |GPIO   |SPI3(Sensor)   |SYS_MONOUT4    |―             |
|　↑            |SPI3_MOSI      |○   |○   |　↑   |　↑           |SYS_MONOUT5    |―             |
|　↑            |SPI3_MISO      |○   |○   |　↑   |　↑           |SYS_MONOUT6    |―             |
|I2C0            |I2C0_BCK       |○   |○   |GPIO   |I2C0(Sensor)   |SYS_MONOUT7    |―             |
|　↑            |I2C0_BDT       |○   |○   |　↑   |　↑           |SYS_MONOUT8    |―             |
|PWMA            |PWM0           |○   |○   |GPIO   |PWMA           |SYS_MONOUT9    |―             |
|　↑            |PWM1           |○   |○   |　↑   |　↑           |GPIO           |―             |
|PWMB            |PWM2           |○   |○   |GPIO   |PWMB           |I2C1(Sensor)   |―             |
|　↑            |PWM3           |○   |○   |　↑   |　↑           |　↑           |―             |
|IS              |IS_CLK         |―   |○   |GPIO   |IS             |―             |―             |
|　↑            |IS_VSYNC       |―   |○   |　↑   |　↑           |―             |―             |
|　↑            |IS_HSYNC       |―   |○   |　↑   |　↑           |―             |―             |
|　↑            |IS_DATA0       |―   |○   |　↑   |　↑           |―             |―             |
|　↑            |IS_DATA1       |―   |○   |　↑   |　↑           |―             |―             |
|　↑            |IS_DATA2       |―   |○   |　↑   |　↑           |―             |―             |
|　↑            |IS_DATA3       |―   |○   |　↑   |　↑           |―             |―             |
|　↑            |IS_DATA4       |―   |○   |　↑   |　↑           |―             |―             |
|　↑            |IS_DATA5       |―   |○   |　↑   |　↑           |―             |―             |
|　↑            |IS_DATA6       |―   |○   |　↑   |　↑           |―             |―             |
|　↑            |IS_DATA7       |―   |○   |　↑   |　↑           |―             |―             |
|UART2           |UART2_TXD      |○   |○   |GPIO   |UART2(APP)     |APP_MONOUT0    |―             |
|　↑            |UART2_RXD      |○   |○   |　↑   |　↑           |APP_MONOUT1    |―             |
|　↑            |UART2_CTS      |○   |○   |　↑   |　↑           |APP_MONOUT2    |―             |
|　↑            |UART2_RTS      |○   |○   |　↑   |　↑           |APP_MONOUT3    |―             |
|SPI4            |SPI4_CS_X      |○   |○   |GPIO   |SPI4(APP)      |APP_MONOUT4    |―             |
|　↑            |SPI4_SCK       |○   |○   |　↑   |　↑           |APP_MONOUT5    |―             |
|　↑            |SPI4_MOSI      |○   |○   |　↑   |　↑           |APP_MONOUT6    |―             |
|　↑            |SPI4_MISO      |○   |○   |　↑   |　↑           |APP_MONOUT7    |―             |
|EMMCA           |EMMC_CLK       |○   |○   |GPIO   |EMMC           |SPI5(APP)      |―             |
|　↑            |EMMC_CMD       |○   |○   |　↑   |　↑           |　↑           |―             |
|　↑            |EMMC_DATA0     |○   |○   |　↑   |　↑           |　↑           |―             |
|　↑            |EMMC_DATA1     |○   |○   |　↑   |　↑           |　↑           |―             |
|EMMCB           |EMMC_DATA2     |○   |○   |GPIO   |　↑           |APP_MONOUT8    |―             |
|　↑            |EMMC_DATA3     |○   |○   |　↑   |　↑           |APP_MONOUT9    |―             |
|SDIOA           |SDIO_CLK       |―   |○   |GPIO   |SDIO           |SPI5(APP)      |―             |
|　↑            |SDIO_CMD       |―   |○   |　↑   |　↑           |　↑           |―             |
|　↑            |SDIO_DATA0     |―   |○   |　↑   |　↑           |　↑           |―             |
|　↑            |SDIO_DATA1     |―   |○   |　↑   |　↑           |　↑           |―             |
|　↑            |SDIO_DATA2     |―   |○   |　↑   |　↑           |GPIO           |―             |
|　↑            |SDIO_DATA3     |―   |○   |　↑   |　↑           |　↑           |―             |
|SDIOB           |SDIO_CD        |―   |○   |GPIO   |SDIO(Card)     |―             |―             |
|　↑            |SDIO_WP        |―   |○   |　↑   |　↑           |―             |―             |
|SDIOC           |SDIO_CMDDIR    |―   |○   |GPIO   |SDIO           |―             |―             |
|　↑            |SDIO_DIR0      |―   |○   |　↑   |　↑           |―             |―             |
|　↑            |SDIO_DIR1_3    |―   |○   |　↑   |　↑           |―             |―             |
|SDIOD           |SDIO_CLKI      |―   |○   |GPIO   |SDIO(Card)     |―             |―             |
|I2S0            |I2S0_BCK       |○   |○   |GPIO   |I2S0           |APP_MONOUT0    |―             |
|　↑            |I2S0_LRCK      |○   |○   |　↑   |　↑           |APP_MONOUT1    |―             |
|　↑            |I2S0_DATA_IN   |○   |○   |　↑   |　↑           |APP_MONOUT2    |―             |
|　↑            |I2S0_DATA_OUT  |○   |○   |　↑   |　↑           |APP_MONOUT3    |―             |
|I2S1            |I2S1_BCK       |―   |○   |GPIO   |I2S1           |APP_MONOUT4    |―             |
|　↑            |I2S1_LRCK      |―   |○   |　↑   |　↑           |APP_MONOUT5    |―             |
|　↑            |I2S1_DATA_IN   |―   |○   |　↑   |　↑           |APP_MONOUT6    |―             |
|　↑            |I2S1_DATA_OUT  |―   |○   |　↑   |　↑           |APP_MONOUT7    |―             |
|MCLK            |MCLK           |○   |○   |GPIO   |MCLK           |APP_MONOUT8    |―             |
|PDM             |PDM_CLK        |○   |○   |GPIO   |PDM            |APP_MONOUT9    |―             |
|　↑            |PDM_IN         |○   |○   |　↑   |　↑           |GPIO           |―             |
|　↑            |PDM_OUT        |○   |○   |　↑   |　↑           |GPIO           |―             |
|USBVBUS         |USB_VBUSINT    |○   |○   |GPIO   |USB_VBUSINT    |―             |―             |

## Usage of pinconfig driver {#pinconfig_usage}

A example code of pin configuration:

~~~~~~~~~~~~~~~{.c}
#include "cxd56_pinconfig.h"
{
  CXD56_PIN_CONFIGS(PINCONFS_UART2);      // Use UART2
  CXD56_PIN_CONFIGS(PINCONFS_UART2_GPIO); // Return back to GPIO that is default Hi-Z setting (disabled both input and output)
}

#include "cxd56_pinconfig.h"
#include "cxd56_gpio.h"
{
  cxd56_gpio_config(PIN_AP_CLK, true);  // Use PIN_AP_CLK as GPIO input pin (input-enabled)
  value = cxd56_gpio_read(PIN_AP_CLK);  // It's possible to read value from PIN_AP_CLK
}

#include "cxd56_pinconfig.h"
#include "cxd56_gpio.h"
{
  cxd56_gpio_config(PIN_AP_CLK, false); // Use PIN_AP_CLK as GPIO output pin
                                        // Here is output-disabled yet
  cxd56_gpio_write(PIN_AP_CLK, false);  // output-enabled and output LOW
          or
  cxd56_gpio_write(PIN_AP_CLK, true);   // output-enabled and output HIGH
}
~~~~~~~~~~~~~~~

The reference set of pin configuration has been already defined as named PINCONFS_XXX
in arch/arm/src/cxd56xx/chip/cxd5602_pinconfig.h.

* Below is the reference definitions of pin configuration.
|Group           |Pin            |WLCSP|FCBGA|Mode0 Definitions             |Mode1 Definitions              |Mode2 Definitions              |Mode3 Definitions                  |
|:---------------|:--------------|:---:|:---:|:-----------------------------|:------------------------------|:------------------------------|:----------------------------------|
|I2C4            |I2C4_BCK       |○   |○   |PINCONFS_I2C4_GPIO            |PINCONFS_I2C4                  |                               |                                   |
|　↑            |I2C4_BDT       |○   |○   |　↑                          |　↑                           |                               |                                   |
|PMIC_INT        |PMIC_INT       |○   |○   |PINCONFS_PMIC_INT_GPIO        |PINCONFS_PMIC_INT              |PINCONFS_PMIC_INT_OD           |                                   |
|RTC_IRQ_OUT     |RTC_IRQ_OUT    |―   |○   |PINCONFS_RTC_IRQ_OUT_GPIO     |PINCONFS_RTC_IRQ_OUT           |PINCONFS_RTC_IRQ_OUT_OD        |                                   |
|AP_CLK          |AP_CLK         |○   |○   |PINCONFS_AP_CLK_GPIO          |PINCONFS_AP_CLK                |PINCONFS_AP_CLK_PMU_WDT        |PINCONFS_AP_CLK_PMU_WDT_OD         |
|GNSS_1PPS_OUT   |GNSS_1PPS_OUT  |―   |○   |PINCONFS_GNSS_1PPS_OUT_GPIO   |PINCONFS_GNSS_1PPS_OUT         |PINCONFS_GNSS_1PPS_OUT_CPU_WDT |PINCONFS_GNSS_1PPS_OUT_CPU_WDT_OD  |
|SPI0A           |SPI0_CS_X      |○   |○   |PINCONFS_SPI0(A)_GPIO         |PINCONFS_SPI0A_UART1           |PINCONFS_SPI0                  |                                   |
|　↑            |SPI0_SCK       |○   |○   |　↑                          |　↑                           |　↑                           |                                   |
|SPI0B           |SPI0_MOSI      |―   |○   |PINCONFS_SPI0(B)_GPIO         |PINCONFS_SPI0B_I2C2            |　↑                           |                                   |
|　↑            |SPI0_MISO      |―   |○   |　↑                          |　↑                           |　↑                           |                                   |
|SPI1A           |SPI1_CS_X      |○   |○   |PINCONFS_SPI1(A)_GPIO         |PINCONFS_SPI1                  |PINCONFS_SPI1A_SPI0            |                                   |
|　↑            |SPI1_SCK       |○   |○   |　↑                          |　↑                           |　↑                           |                                   |
|　↑            |SPI1_IO0       |○   |○   |　↑                          |　↑                           |　↑                           |                                   |
|　↑            |SPI1_IO1       |○   |○   |　↑                          |　↑                           |　↑                           |                                   |
|SPI1B           |SPI1_IO2       |○   |○   |PINCONFS_SPI1(B)_GPIO         |　↑                           |                               |                                   |
|　↑            |SPI1_IO3       |○   |○   |　↑                          |　↑                           |                               |                                   |
|SPI2A           |SPI2_CS_X      |○   |○   |PINCONFS_SPI2(A)_GPIO         |PINCONFS_SPI2                  |PINCONFS_SPI2A_UART0           |PINCONFS_SPI2A_I2C3                |
|　↑            |SPI2_SCK       |○   |○   |　↑                          |　↑                           |　↑                           |　↑                               |
|SPI2B           |SPI2_MOSI      |○   |○   |PINCONFS_SPI2(B)_GPIO         |　↑                           |PINCONFS_SPI2B_UART0           |                                   |
|　↑            |SPI2_MISO      |○   |○   |　↑                          |　↑                           |　↑                           |                                   |
|HIFIRQ          |HIF_IRQ_OUT    |○   |○   |PINCONFS_HIF_IRQ_OUT_GPIO     |PINCONFS_HIF_IRQ_OUT           |PINCONFS_HIF_IRQ_OUT_OD        |PINCONFS_HIF_IRQ_OUT_GNSS_1PPS_OUT |
|HIFEXT          |HIF_GPIO0      |―   |○   |PINCONFS_HIF_GPIO0_GPIO       |                               |                               |PINCONFS_HIF_GPIO0_GPS_EXTLD       |
|SEN_IRQ_IN      |SEN_IRQ_IN     |○   |○   |PINCONFS_SEN_IRQ_IN_GPIO      |PINCONFS_SEN_IRQ_IN            |                               |                                   |
|SPI3_CS0_X      |SPI3_CS0_X     |○   |○   |PINCONFS_SPI3_CS0_X_GPIO      |PINCONFS_SPI3_CS0_X            |                               |                                   |
|SPI3_CS1_X      |SPI3_CS1_X     |○   |○   |PINCONFS_SPI3_CS1_X_GPIO      |PINCONFS_SPI3_CS1_X            |                               |                                   |
|SPI3_CS2_X      |SPI3_CS2_X     |○   |○   |PINCONFS_SPI3_CS2_X_GPIO      |PINCONFS_SPI3_CS2_X            |                               |                                   |
|SPI3            |SPI3_SCK       |○   |○   |PINCONFS_SPI3_GPIO            |PINCONFS_SPI3                  |                               |                                   |
|　↑            |SPI3_MOSI      |○   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |SPI3_MISO      |○   |○   |　↑                          |　↑                           |                               |                                   |
|I2C0            |I2C0_BCK       |○   |○   |PINCONFS_I2C0_GPIO            |PINCONFS_I2C0                  |                               |                                   |
|　↑            |I2C0_BDT       |○   |○   |　↑                          |　↑                           |                               |                                   |
|PWMA            |PWM0           |○   |○   |PINCONFS_PWMA_GPIO            |PINCONFS_PWMA                  |                               |                                   |
|　↑            |PWM1           |○   |○   |　↑                          |　↑                           |                               |                                   |
|PWMB            |PWM2           |○   |○   |PINCONFS_PWMB_GPIO            |PINCONFS_PWMB                  |PINCONFS_PWMB_I2C1             |                                   |
|　↑            |PWM3           |○   |○   |　↑                          |　↑                           |　↑                           |                                   |
|IS              |IS_CLK         |―   |○   |PINCONFS_IS_GPIO              |PINCONFS_IS                    |                               |                                   |
|　↑            |IS_VSYNC       |―   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |IS_HSYNC       |―   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |IS_DATA0       |―   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |IS_DATA1       |―   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |IS_DATA2       |―   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |IS_DATA3       |―   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |IS_DATA4       |―   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |IS_DATA5       |―   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |IS_DATA6       |―   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |IS_DATA7       |―   |○   |　↑                          |　↑                           |                               |                                   |
|UART2           |UART2_TXD      |○   |○   |PINCONFS_UART2_GPIO           |PINCONFS_UART2                 |                               |                                   |
|　↑            |UART2_RXD      |○   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |UART2_CTS      |○   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |UART2_RTS      |○   |○   |　↑                          |　↑                           |                               |                                   |
|SPI4            |SPI4_CS_X      |○   |○   |PINCONFS_SPI4_GPIO            |PINCONFS_SPI4                  |                               |                                   |
|　↑            |SPI4_SCK       |○   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |SPI4_MOSI      |○   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |SPI4_MISO      |○   |○   |　↑                          |　↑                           |                               |                                   |
|EMMCA           |EMMC_CLK       |○   |○   |PINCONFS_EMMC(A)_GPIO         |PINCONFS_EMMC                  |PINCONFS_EMMCA_SPI5            |                                   |
|　↑            |EMMC_CMD       |○   |○   |　↑                          |　↑                           |　↑                           |                                   |
|　↑            |EMMC_DATA0     |○   |○   |　↑                          |　↑                           |　↑                           |                                   |
|　↑            |EMMC_DATA1     |○   |○   |　↑                          |　↑                           |　↑                           |                                   |
|EMMCB           |EMMC_DATA2     |○   |○   |PINCONFS_EMMC(B)_GPIO         |　↑                           |                               |                                   |
|　↑            |EMMC_DATA3     |○   |○   |　↑                          |　↑                           |                               |                                   |
|SDIOA           |SDIO_CLK       |―   |○   |PINCONFS_SDIOA_GPIO           |PINCONFS_SDIOA_{SDIO,SDCARD}   |PINCONFS_SDIOA_SPI5            |                                   |
|　↑            |SDIO_CMD       |―   |○   |　↑                          |　↑                           |　↑                           |                                   |
|　↑            |SDIO_DATA0     |―   |○   |　↑                          |　↑                           |　↑                           |                                   |
|　↑            |SDIO_DATA1     |―   |○   |　↑                          |　↑                           |　↑                           |                                   |
|　↑            |SDIO_DATA2     |―   |○   |　↑                          |　↑                           |　↑                           |                                   |
|　↑            |SDIO_DATA3     |―   |○   |　↑                          |　↑                           |　↑                           |                                   |
|SDIOB           |SDIO_CD        |―   |○   |PINCONFS_SDIOB_GPIO           |PINCONFS_SDIOB_SDCARD          |                               |                                   |
|　↑            |SDIO_WP        |―   |○   |　↑                          |　↑                           |                               |                                   |
|SDIOC           |SDIO_CMDDIR    |―   |○   |PINCONFS_SDIOC_GPIO           |PINCONFS_SDIOC_SDIO            |                               |                                   |
|　↑            |SDIO_DIR0      |―   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |SDIO_DIR1_3    |―   |○   |　↑                          |　↑                           |                               |                                   |
|SDIOD           |SDIO_CLKI      |―   |○   |PINCONFS_SDIOD_GPIO           |PINCONFS_SDIOD_SDIO            |                               |                                   |
|I2S0            |I2S0_BCK       |○   |○   |PINCONFS_I2S0_GPIO            |PINCONFS_I2S0_{M,S}            |                               |                                   |
|　↑            |I2S0_LRCK      |○   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |I2S0_DATA_IN   |○   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |I2S0_DATA_OUT  |○   |○   |　↑                          |　↑                           |                               |                                   |
|I2S1            |I2S1_BCK       |―   |○   |PINCONFS_I2S1_GPIO            |PINCONFS_I2S1_{M,S}            |                               |                                   |
|　↑            |I2S1_LRCK      |―   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |I2S1_DATA_IN   |―   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |I2S1_DATA_OUT  |―   |○   |　↑                          |　↑                           |                               |                                   |
|MCLK            |MCLK           |○   |○   |PINCONFS_MCLK_GPIO            |PINCONFS_MCLK                  |                               |                                   |
|PDM             |PDM_CLK        |○   |○   |PINCONFS_PDM_GPIO             |PINCONFS_PDM                   |                               |                                   |
|　↑            |PDM_IN         |○   |○   |　↑                          |　↑                           |                               |                                   |
|　↑            |PDM_OUT        |○   |○   |　↑                          |　↑                           |                               |                                   |
|USBVBUS         |USB_VBUSINT    |○   |○   |PINCONFS_USB_VBUSINT_GPIO     |PINCONFS_USB_VBUSINT           |                               |                                   |

____________________________________________________________________________________________________________

# LED porting guide  {#led_doc}

In the case of supporting LED, porting to control LED is needed.
To support LED, porting does the following thing.

- Board common header

  Add a definition necessary to LED control.

- Board Configuration

  A configuration of GPIO is performed to LED port.

- Interface of LED driver and GPIO driver

  LED ON/OFF is controlled to LED port.

Please refer to the [LED_Support](http://www.nuttx.org/Documentation/NuttxPortingGuide.html#ledsupport) porting guide for details of LED porting.

## Examples  {#led_examples}

### Board common header examples {#led_boardheader}

- [configs/corvo/include/board.h](\ref configs/corvo/include/board.h)

### Board Configuration and LED controller examples {#led_config_control}

- [configs/corvo/src/cxd56_autoleds.c](\ref configs/corvo/src/cxd56_autoleds.c)
- [configs/corvo/src/cxd56_userleds.c](\ref configs/corvo/src/cxd56_userleds.c)
- [configs/corvo/src/corvo.h](\ref configs/corvo/src/corvo.h)

### Driver examples for LED supported driver {#led_drivers}

- [drivers/leds/userled_lower.c](\ref drivers/leds/userled_lower.c)
- [drivers/leds/userled_upper.c](\ref drivers/leds/userled_upper.c)

### Application examples {#led_sample_apps}

- [sony_apps/examples/leds/leds_main.c](\ref examples/leds/leds_main.c)

____________________________________________________________________________________________________________

# Button porting guide  {#button_doc}

In the case of supporting button, porting to control button is needed.
To support button, porting does the following thing.

- Board common header

  Add a definition necessary to button control.

- Board Configuration

  A configuration of GPIO is performed to button port.

- Interface of application and GPIO driver

  Button ON/OFF is controlled to button port.


## Examples  {#button_examples}

### Board common header examples {#button_boardheader}

- [configs/corvo/include/board.h](\ref configs/corvo/include/board.h)

### Board Configuration and button controller examples {#button_config_control}

- [configs/corvo/src/cxd56_buttons.c](\ref configs/corvo/src/cxd56_buttons.c)
- [configs/corvo/src/corvo.h](\ref configs/corvo/src/corvo.h)

### Application examples {#button_sample_sony_apps}

- [sony_apps/examples/buttons/buttons_main.c](\ref examples/buttons/buttons_main.c)

\example configs/corvo/include/board.h
\example configs/corvo/src/cxd56_autoleds.c
\example configs/corvo/src/cxd56_userleds.c
\example configs/corvo/src/corvo.h
\example drivers/leds/userled_lower.c
\example drivers/leds/userled_upper.c
\example examples/leds/leds_main.c

\example configs/corvo/src/cxd56_buttons.c
\example examples/buttons/buttons_main.c

