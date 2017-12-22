/********************************************************************************************
 * arch/arm/src/cxd56xx/cxd56_pinconfig.h
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_PINCONFIG_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_PINCONFIG_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <sdk/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "chip/cxd5602_pinconfig.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* 32-bit encoded pinconf value
 *
 * 3322 2222 2222 1111 1111 1100 0000 0000
 * 1098 7654 3210 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ---- ---- ---- ----
 * PPPP PPP. .... .... .... .... .... .... Pin number
 * .... ...D .... .... .... .... .... .... Drive strength
 * .... .... .... ...U .... ...U .... .... Pull-up/down/off
 * .... .... .... .... .... .... .... ...I Input enable
 * .... .... .... .... .... .... .... .MM. Alternate mode number
 */

/* Pin number Definitions */

#define PINCONF_PIN_SHIFT       (25)
#define PINCONF_PIN_MASK        (0x7Fu << PINCONF_PIN_SHIFT)
#define PINCONF_GET_PIN(p)      (((p) & PINCONF_PIN_MASK) >> PINCONF_PIN_SHIFT)
#define PINCONF_SET_PIN(p)      (((p) << PINCONF_PIN_SHIFT) & PINCONF_PIN_MASK)

#define PIN_RTC_CLK_IN          (0)
#define PIN_I2C4_BCK            (1)
#define PIN_I2C4_BDT            (2)
#define PIN_PMIC_INT            (3)
#define PIN_RTC_IRQ_OUT         (4)
#define PIN_AP_CLK              (5)
#define PIN_GNSS_1PPS_OUT       (6)
#define PIN_SPI0_CS_X           (17)
#define PIN_SPI0_SCK            (18)
#define PIN_SPI0_MOSI           (19)
#define PIN_SPI0_MISO           (20)
#define PIN_SPI1_CS_X           (21)
#define PIN_SPI1_SCK            (22)
#define PIN_SPI1_IO0            (23)
#define PIN_SPI1_IO1            (24)
#define PIN_SPI1_IO2            (25)
#define PIN_SPI1_IO3            (26)
#define PIN_SPI2_CS_X           (27)
#define PIN_SPI2_SCK            (28)
#define PIN_SPI2_MOSI           (29)
#define PIN_SPI2_MISO           (30)
#define PIN_HIF_IRQ_OUT         (31)
#define PIN_HIF_GPIO0           (32)
#define PIN_SEN_IRQ_IN          (37)
#define PIN_SPI3_CS0_X          (38)
#define PIN_SPI3_CS1_X          (39)
#define PIN_SPI3_CS2_X          (40)
#define PIN_SPI3_SCK            (41)
#define PIN_SPI3_MOSI           (42)
#define PIN_SPI3_MISO           (43)
#define PIN_I2C0_BCK            (44)
#define PIN_I2C0_BDT            (45)
#define PIN_PWM0                (46)
#define PIN_PWM1                (47)
#define PIN_PWM2                (48)
#define PIN_PWM3                (49)
#define PIN_IS_CLK              (56)
#define PIN_IS_VSYNC            (57)
#define PIN_IS_HSYNC            (58)
#define PIN_IS_DATA0            (59)
#define PIN_IS_DATA1            (60)
#define PIN_IS_DATA2            (61)
#define PIN_IS_DATA3            (62)
#define PIN_IS_DATA4            (63)
#define PIN_IS_DATA5            (64)
#define PIN_IS_DATA6            (65)
#define PIN_IS_DATA7            (66)
#define PIN_UART2_TXD           (67)
#define PIN_UART2_RXD           (68)
#define PIN_UART2_CTS           (69)
#define PIN_UART2_RTS           (70)
#define PIN_SPI4_CS_X           (71)
#define PIN_SPI4_SCK            (72)
#define PIN_SPI4_MOSI           (73)
#define PIN_SPI4_MISO           (74)
#define PIN_EMMC_CLK            (75)
#define PIN_EMMC_CMD            (76)
#define PIN_EMMC_DATA0          (77)
#define PIN_EMMC_DATA1          (78)
#define PIN_EMMC_DATA2          (79)
#define PIN_EMMC_DATA3          (80)
#define PIN_SDIO_CLK            (81)
#define PIN_SDIO_CMD            (82)
#define PIN_SDIO_DATA0          (83)
#define PIN_SDIO_DATA1          (84)
#define PIN_SDIO_DATA2          (85)
#define PIN_SDIO_DATA3          (86)
#define PIN_SDIO_CD             (87)
#define PIN_SDIO_WP             (88)
#define PIN_SDIO_CMDDIR         (89)
#define PIN_SDIO_DIR0           (90)
#define PIN_SDIO_DIR1_3         (91)
#define PIN_SDIO_CLKI           (92)
#define PIN_I2S0_BCK            (93)
#define PIN_I2S0_LRCK           (94)
#define PIN_I2S0_DATA_IN        (95)
#define PIN_I2S0_DATA_OUT       (96)
#define PIN_I2S1_BCK            (97)
#define PIN_I2S1_LRCK           (98)
#define PIN_I2S1_DATA_IN        (99)
#define PIN_I2S1_DATA_OUT       (100)
#define PIN_MCLK                (101)
#define PIN_PDM_CLK             (102)
#define PIN_PDM_IN              (103)
#define PIN_PDM_OUT             (104)
#define PIN_USB_VBUSINT         (105)

/* Drive strength Definitions */

#define PINCONF_DRIVE_SHIFT     (24)
#define PINCONF_DRIVE_MASK      (1u << PINCONF_DRIVE_SHIFT)

#define PINCONF_DRIVE_NORMAL    (1u << PINCONF_DRIVE_SHIFT) /* 2mA */
#define PINCONF_DRIVE_HIGH      (0u << PINCONF_DRIVE_SHIFT) /* 4mA */

#define PINCONF_IS_DRIVE_NORM(p) (((p) & PINCONF_DRIVE_MASK) == PINCONF_DRIVE_NORMAL)
#define PINCONF_IS_DRIVE_HIGH(p) (((p) & PINCONF_DRIVE_MASK) == PINCONF_DRIVE_HIGH)

/* Pull-up/down/off Definitions */

#define PINCONF_PULL_MASK       ((1u << 16) | (1u << 8))

#define PINCONF_FLOAT           ((1u << 16) | (1u << 8))
#define PINCONF_PULLUP          ((1u << 16) | (0u << 8))
#define PINCONF_PULLDOWN        ((0u << 16) | (1u << 8))

#define PINCONF_IS_FLOAT(p)     (((p) & PINCONF_PULL_MASK) == PINCONF_FLOAT)
#define PINCONF_IS_PULLUP(p)    (((p) & PINCONF_PULL_MASK) == PINCONF_PULLUP)
#define PINCONF_IS_PULLDOWN(p)  (((p) & PINCONF_PULL_MASK) == PINCONF_PULLDOWN)

/* Input enable Definitions */

#define PINCONF_IN_EN_SHIFT     (0)
#define PINCONF_IN_EN_MASK      (1u << PINCONF_IN_EN_SHIFT)

#define PINCONF_INPUT_ENABLE    (1u << PINCONF_IN_EN_SHIFT)
#define PINCONF_INPUT_DISABLE   (0u << PINCONF_IN_EN_SHIFT)

#define PINCONF_INPUT_ENABLED(p) (((p) & PINCONF_IN_EN_MASK) == PINCONF_INPUT_ENABLE)

/* Alternate mode number Definitions */

#define PINCONF_MODE_SHIFT      (1)
#define PINCONF_MODE_MASK       (3u << PINCONF_MODE_SHIFT)

#define PINCONF_GET_MODE(p)     (((p) & PINCONF_MODE_MASK) >> PINCONF_MODE_SHIFT)
#define PINCONF_SET_MODE(p)     (((p) << PINCONF_MODE_SHIFT) & PINCONF_MODE_MASK)

#define PINCONF_MODE0           (0) /* GPIO */
#define PINCONF_MODE1           (1) /* Function */
#define PINCONF_MODE2           (2) /* Function */
#define PINCONF_MODE3           (3) /* Function */

/* Set pinconf macro Definitions */

#define PINCONF_SET(pin, mode, input, drive, pull) \
  ( \
    (PINCONF_SET_PIN(pin)) | \
    (PINCONF_SET_MODE(mode)) | \
    (input) | (drive) | (pull) \
    )

#define PINCONF_SET_GPIO(pin, input) \
  PINCONF_SET((pin), PINCONF_MODE0, (input), PINCONF_DRIVE_NORMAL, PINCONF_FLOAT)

#define CXD56_PIN_CONFIGS(pin) do { \
  uint32_t p[] = pin; \
  cxd56_pin_configs((p), sizeof(p) / sizeof((p)[0])); \
} while (0)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

struct cxd56_pin_status_s
{
  uint32_t mode;     /* alternate pin function mode */
  uint32_t input_en; /* input enable or disable */
  uint32_t drive;    /* strength of drive current */
  uint32_t pull;     /* internal pull-up, pull-down or floating */
};

typedef struct cxd56_pin_status_s cxd56_pin_status_t;

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

/********************************************************************************************
 * Name: cxd56_pin_config
 *
 * Description:
 *   Configure a pin based on bit-encoded description of the pin.
 *
 * Input Value:
 *   32-bit encoded value describing the pin.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ********************************************************************************************/

int cxd56_pin_config(uint32_t pinconf);

/********************************************************************************************
 * Name: cxd56_pin_configs
 *
 * Description:
 *   Configure multiple pins based on bit-encoded description of the pin.
 *
 * Input Value:
 *   Array of 32-bit encoded value describing the pin.
 *   Number of elements in the array.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ********************************************************************************************/

int cxd56_pin_configs(uint32_t pinconfs[], size_t n);

/********************************************************************************************
 * Name: cxd56_pin_status
 *
 * Description:
 *   Get a pin status.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ********************************************************************************************/

int cxd56_pin_status(uint32_t pin, cxd56_pin_status_t *stat);

/********************************************************************************************
 * Function:  cxd56_pin_dump
 *
 * Description:
 *   Dump a pin configuration
 *
 ********************************************************************************************/

#ifdef CONFIG_DEBUG
int cxd56_pin_dump(uint32_t pin, const char *msg);
#else
#  define cxd56_pin_dump(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_PINCONFIG_H */
