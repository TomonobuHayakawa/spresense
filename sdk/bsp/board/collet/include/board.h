/****************************************************************************
 * configs/collet/include/board.h
 *
 *   Copyright (C) 2016 Sony Corporation. All rights reserved.
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

#ifndef _CONFIGS_COLLET_INCLUDE_BOARD_H
#define _CONFIGS_COLLET_INCLUDE_BOARD_H

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

#include <arch/board/common/cxd56_power.h>
#include <arch/board/common/cxd56_audio.h>

#include <arch/board/common/cxd56_flash.h>

#include <arch/board/common/cxd56_bmp280.h>
#include <arch/board/common/cxd56_bm1383glv.h>
#include <arch/board/common/cxd56_bmi160.h>
#include <arch/board/common/cxd56_kx022.h>
#include <arch/board/common/cxd56_ak09912.h>
#include <arch/board/common/cxd56_bm1422gmv.h>
#include <arch/board/common/cxd56_bh1745nuc.h>
#include <arch/board/common/cxd56_apds9930.h>
#include <arch/board/common/cxd56_apds9960.h>
#include <arch/board/common/cxd56_lt1pa01.h>
#include <arch/board/common/cxd56_bh1721fvc.h>
#include <arch/board/common/cxd56_rpr0521rs.h>
#include <arch/board/common/cxd56_i2cdev.h>

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
#define BOARD_NLEDS         (1)

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT      (1 << BOARD_LED1)

/* Auto LED definitions ****************************************************/
/* The COLLET board has a single green LED (there are additional LEDs on the
 * base board not considered here).
 */
                                /* ON      OFF                 */
#define LED_STARTED         (0) /* OFF     ON  (never happens) */
#define LED_HEAPALLOCATE    (0) /* OFF     ON  (never happens) */
#define LED_IRQSENABLED     (0) /* OFF     ON  (never happens) */
#define LED_STACKCREATED    (1) /* ON      ON  (never happens) */
#define LED_INIRQ           (2) /* OFF     NC  (momentary) */
#define LED_SIGNAL          (2) /* OFF     NC  (momentary) */
#define LED_ASSERTION       (2) /* OFF     NC  (momentary) */
#define LED_PANIC           (0) /* OFF     ON  (1Hz flashing) */

/* Buttons definitions *****************************************************/

#define BOARD_NUM_BUTTONS   (2)

/* Power Control definitions ***********************************************/

/*
 *   For SPARK + COLLET board:
 *
 *     Switch    Device
 *     --------- -------------------------------
 *     LSW2      AcaPulco Audio Digital VDD
 *     LSW3      SPI-Flash & TCXO
 *     LSW4      GNSS LNA
 *     GPO0      AcaPulco Audio Analog VDD
 *     GPO1      BMI160 1.8V
 *     GPO2      Sensor 1.8V
 *     GPO3      Sensor 3.3V
 *     GPO4      Bluetooth/Bluetooth Low Energy
 *     GPO5      Lfour
 *     GPO6      LTE
 *     GPO7      E-Ink
 */

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
  POWER_SENSOR_18V      = PMIC_GPO(2),
  POWER_SENSOR_33V      = PMIC_GPO(3),
  POWER_BMI160          = PMIC_GPO(1),
  POWER_SENSOR          = POWER_BMI160 | POWER_SENSOR_18V | POWER_SENSOR_33V,
  POWER_BTBLE           = PMIC_GPO(4),
  POWER_EINK            = PMIC_GPO(7),
  POWER_EMMC            = PMIC_NONE,
  POWER_LFOUR           = PMIC_GPO(5),
  POWER_LTE             = PMIC_GPO(6),
  POWER_IMAGE_SENSOR    = PMIC_NONE,
};

/* CXD5247 power control definitions *******************************************/

#define CXD5247_AVDD  (0x01)
#define CXD5247_DVDD  (0x02)

/* Display device pin definitions *******************************************/

#define DISPLAY_RST     PIN_SPI2_MOSI
#define DISPLAY_DC      PIN_SPI2_MISO

#define DISPLAY_SPI     4

/* External pin definitions for EINK deivce */

#define EINK_RST    PIN_SPI2_CS_X
#define EINK_BUSY   PIN_SPI2_SCK
#define EINK_CS     PIN_SEN_IRQ_IN
#define EINK_OEI    (-1)
#define EINK_POWER  (-1)

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* _CONFIGS_COLLET_INCLUDE_BOARD_H */
