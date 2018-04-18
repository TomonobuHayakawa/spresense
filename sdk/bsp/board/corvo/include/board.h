/****************************************************************************
 * configs/corvo/include/board.h
 *
 *   Copyright (C) 2016,2018 Sony Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor Sony nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef _CONFIGS_CORVO_INCLUDE_BOARD_H
#define _CONFIGS_CORVO_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>
#include <sys/boardctl.h>
#include <stdbool.h>

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_GPIO_IRQ)
#  include <nuttx/irq.h>
#endif

#if defined(CONFIG_BOARDCTL_IOCTL) && defined(CONFIG_USBDEV)
#  include <arch/chip/usbdev.h>
#endif

#include <arch/board/common/cxd56_flash.h>
#include <arch/board/common/cxd56_emmcdev.h>

#include <arch/board/common/cxd56_bmp280.h>
#include <arch/board/common/cxd56_bm1383glv.h>
#include <arch/board/common/cxd56_bmi160.h>
#include <arch/board/common/cxd56_kx022.h>
#include <arch/board/common/cxd56_ak09912.h>
#include <arch/board/common/cxd56_bm1422gmv.h>
#include <arch/board/common/cxd56_bh1745nuc.h>
#include <arch/board/common/cxd56_apds9930.h>
#include <arch/board/common/cxd56_lt1pa01.h>
#include <arch/board/common/cxd56_bh1721fvc.h>
#include <arch/board/common/cxd56_rpr0521rs.h>
#include <arch/board/common/cxd56_i2cdev.h>

#include <arch/board/common/cxd56_isx012.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking ****************************************************************/

#define BOARD_XTAL_FREQUENCY        (26000000)  /* XTAL oscillator frequency (Y3) */
#define BOARD_RTCCLK_FREQUENCY      (32768)     /* RTC oscillator frequency (Y1) */
#define BOARD_INTRCOSC_FREQUENCY    (8192000)   /* Internal RC oscillator frequency */

#ifdef CONFIG_CXD56_80MHz
#  define BOARD_FCLKOUT_FREQUENCY   (80000000)
#else
#  define BOARD_FCLKOUT_FREQUENCY   (100000000)
#endif

/* UART clocking ***********************************************************/
/* Configure all UARTs to use the XTAL input frequency */

#define BOARD_UART0_BASEFREQ        BOARD_XTAL_FREQUENCY
#define BOARD_UART1_BASEFREQ        BOARD_FCLKOUT_FREQUENCY
#define BOARD_UART2_BASEFREQ        BOARD_XTAL_FREQUENCY

/* LED definitions *********************************************************/

#define BOARD_LED1          (0)
#define BOARD_LED2          (1)
#define BOARD_NLEDS         (2)

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT      (1 << BOARD_LED1)
#define BOARD_LED2_BIT      (1 << BOARD_LED2)

/* LED pattern for use with board_autoled_on() and board_autoled_off()
 *           ON            OFF
 *       LED1   LED2   LED1   LED2
 * PTN0: OFF    OFF     -      -
 * PTN1: ON     OFF     -      -
 * PTN2: -      ON      -      OFF
 *
 */

#define LED_AUTOLED_PTN0    (0)
#define LED_AUTOLED_PTN1    (1)
#define LED_AUTOLED_PTN2    (2)

#define LED_STARTED         (LED_AUTOLED_PTN0)
#define LED_HEAPALLOCATE    (LED_AUTOLED_PTN1)
#define LED_IRQSENABLED     (LED_AUTOLED_PTN1)
#define LED_STACKCREATED    (LED_AUTOLED_PTN1)
#define LED_INIRQ           (LED_AUTOLED_PTN2)
#define LED_SIGNAL          (LED_AUTOLED_PTN2)
#define LED_ASSERTION       (LED_AUTOLED_PTN2)
#define LED_PANIC           (LED_AUTOLED_PTN2)

/* Buttons definitions *****************************************************/

#define BOARD_NUM_BUTTONS   (2)

/* Power Control definitions ***********************************************/

/*
 *     Switch    Device
 *     --------- -------------------------------
 *     LSW2      AcaPulco Audio Digital VDD
 *     LSW3      SPI-Flash & TCXO
 *     LSW4      GNSS LNA
 *     GPO0      AcaPulco Audio Analog VDD
 *     GPO1      Sensor 1.8V
 *     GPO2      Sensor 3.3V
 *     GPO3      Bluetooth/Bluetooth Low Energy
 *     GPO4      Image Sensor 1.2V
 *     GPO5      Image Sensor 3.3V
 *     GPO6      eMMC 3.3V/1.8V
 *     GPO7      Image Sensor 1.8V
 *
 */

#define PMIC_NONE           (0)
#define PMIC_TYPE_LSW       (1u << 8)
#define PMIC_TYPE_GPO       (1u << 9)
#define PMIC_TYPE_DDCLDO    (1u << 10)
#define PMIC_GET_TYPE(v)    ((v) & 0xff00)
#define PMIC_GET_CH(v)      ((v) & 0x00ff)
#define PMIC_LSW(n)         (PMIC_TYPE_LSW | (1u << (n)))
#define PMIC_GPO(n)         (PMIC_TYPE_GPO | (1u << (n)))
#define PMIC_DDCLDO(n)      (PMIC_TYPE_DDCLDO | (1u << (n)))

enum board_power_device {

  /* DDC/LDO */

  POWER_DDC_IO          = PMIC_DDCLDO(0),
  POWER_LDO_EMMC        = PMIC_DDCLDO(1),
  POWER_DDC_ANA         = PMIC_DDCLDO(2),
  POWER_LDO_ANA         = PMIC_DDCLDO(3),
  POWER_DDC_CORE        = PMIC_DDCLDO(4),
  POWER_LDO_PERI        = PMIC_DDCLDO(5),

  /* Load Switch */

  POWER_AUDIO_DVDD      = PMIC_LSW(2),
  POWER_FLASH           = PMIC_LSW(3),
  POWER_TCXO            = PMIC_LSW(3),
  POWER_LNA             = PMIC_LSW(4),

  /* GPO */

  POWER_AUDIO_AVDD      = PMIC_GPO(0),
  POWER_SENSOR_18V      = PMIC_GPO(1),
  POWER_SENSOR_33V      = PMIC_GPO(2),
  POWER_BMI160          = POWER_SENSOR_18V,
  POWER_SENSOR          = POWER_SENSOR_18V | POWER_SENSOR_33V,
  POWER_BTBLE           = PMIC_GPO(3),
  POWER_EINK            = PMIC_NONE,
  POWER_EMMC            = PMIC_GPO(6),
  POWER_LFOUR           = PMIC_NONE,
  POWER_LTE             = PMIC_NONE,
  POWER_IMAGE_SENSOR    = PMIC_GPO(4) | PMIC_GPO(5) | PMIC_GPO(7),

};

/* CXD5247 power control definitions *******************************************/

#define CXD5247_AVDD  (0x01)
#define CXD5247_DVDD  (0x02)

/* Display device pin definitions *******************************************/

#define DISPLAY_RST     PIN_SPI2_MOSI
#define DISPLAY_DC      PIN_SPI2_MISO

#define DISPLAY_SPI     4

/* External pin definitions for EINK deivce */

#define EINK_RST    PIN_PWM2
#define EINK_BUSY   PIN_SPI3_CS1_X
#define EINK_CS     PIN_PWM3
#define EINK_OEI    -1
#define EINK_POWER  -1

/* Imager device pin definitions *******************************************/

#define IMAGER_RST      PIN_SPI0_MISO
#define IMAGER_SLEEP    PIN_SPI0_MOSI
#define IMAGER_ALERT    PIN_SDIO_CLKI

/*
 * Set signal id for notify USB device connection status and supply current value.
 * signal returns "usbdev_notify_s" struct pointer in sival_ptr.
 *
 * Arg: Value of sinal number
 */

#define BOARDIOC_USBDEV_SETNOTIFYSIG      (BOARDIOC_USER+0x0001)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_boardinitialize
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_boardinitialize(void);

/****************************************************************************
 * Name: board_power_control
 *
 * Description:
 *   Power on/off the device on the board.
 *
 ****************************************************************************/

int board_power_control(int target, bool en);

/****************************************************************************
 * Name: board_power_monitor
 *
 * Description:
 *   Get status of Power on/off the device on the board.
 *
 ****************************************************************************/

bool board_power_monitor(int target);

/****************************************************************************
 * Name: board_flash_power_control
 *
 * Description:
 *   Power on/off the flash device on the board.
 *
 ****************************************************************************/

int board_flash_power_control(bool en);

/****************************************************************************
 * Name: board_flash_power_monitor
 *
 * Description:
 *   Get status of Power on/off the flash device on the board.
 *
 ****************************************************************************/

bool board_flash_power_monitor(void);

/****************************************************************************
 * Name: board_xtal_power_control
 *
 * Description:
 *   Power on/off the Xtal device on the board.
 *
 ****************************************************************************/

int board_xtal_power_control(bool en);

/****************************************************************************
 * Name: board_xtal_power_monitor
 *
 * Description:
 *   Get status of Power on/off the Xtal device on the board.
 *
 ****************************************************************************/

bool board_xtal_power_monitor(void);

/****************************************************************************
 * Name: board_lna_power_control
 *
 * Description:
 *   Power on/off the LNA device on the board.
 *
 ****************************************************************************/

int board_lna_power_control(bool en);

/****************************************************************************
 * Name: board_aca_power_control
 *
 * Description:
 *   Power on/off the Aca device on the board.
 *
 ****************************************************************************/

int board_aca_power_control(int target, bool en);

/****************************************************************************
 * Name: board_aca_power_monitor
 *
 * Description:
 *   Get status of Power on/off the Aca device on the board.
 *
 ****************************************************************************/

bool board_aca_power_monitor(int target);

/****************************************************************************
 * Name: board_external_amp_mute_control
 *
 * Description:
 *   External Amp. Mute on/off.
 *    true:  Mute on
 *    false: Mute off
 *
 ****************************************************************************/

#define board_external_amp_mute_control(en) (OK)

/****************************************************************************
 * Name: board_external_amp_mute_monitor
 *
 * Description:
 *   Get External Amp. Mute status.
 *    true:  Mute on
 *    false: Mute off
 *
 ****************************************************************************/

#define board_external_amp_mute_monitor() (false)

#ifdef CONFIG_BCM20707

/****************************************************************************
 * Name: board_bcm20707_pin_cfg
 *
 * Description:
 *   Initialize bcm20707 control pins, it must be called before any operation
 *   to do power control, wake up and reset.
 *
 ****************************************************************************/

int board_bcm20707_pin_cfg(void);

/****************************************************************************
 * Name: board_bcm20707_reset
 *
 * Description:
 *   Reset bcm20707 chip
 *
 ****************************************************************************/

void board_bcm20707_reset(void);

/****************************************************************************
 * Name: board_bcm20707_power_control
 *
 * Description:
 *   Power on/off bcm20707 chip
 *
 ****************************************************************************/

int board_bcm20707_power_control(bool en);

/****************************************************************************
 * Name: board_bcm20707_enable_sleep
 *
 * Description:
 *   Enable/disable bcm20707 enters sleep mode
 *
 ****************************************************************************/

void board_bcm20707_enable_sleep(bool en);

/****************************************************************************
 * Name: board_bcm20707_get_firmware_size_in_flash
 *
 * Description:
 *   Get bcm20707 size of firmware in SPI Flash
 *
 ****************************************************************************/

int board_bcm20707_get_firmware_size_in_flash(const char* firmware, int* firmware_size);
/****************************************************************************
 * Name: board_bcm20707_load_firmware_from_flash
 *
 * Description:
 *   Load bcm20707 firmware from SPI Flash
 *
 ****************************************************************************/

int board_bcm20707_load_firmware_from_flash(const char* firmwareName, uint32_t* loadbuf);
#endif /* CONFIG_BCM20707 */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* _CONFIGS_CORVO_INCLUDE_BOARD_H */
