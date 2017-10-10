/********************************************************************************************
 * include/nuttx/lcd/lpm013m091a.h
 *
 *   Copyright (C) 2016 Sony Corporation. All rights reserved.
 *   Author: Tetsuro Itabashi <Tetsuro.x.Itabashi@sony.com>
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

#ifndef __INCLUDE_NUTTX_LCD_LPM013M091A_H
#define __INCLUDE_NUTTX_LCD_LPM013M091A_H

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>

#include <nuttx/fs/ioctl.h>

/* LCD driver ioctl definitions **********************************************/

#define _LCDBASE        (0x2100) /* LCD driver commands */
#define _LCDIOCVALID(c) (_IOC_TYPE(c)==_LCDBASE)
#define _LCDIOC(nr)     _IOC(_LCDBASE,nr)

/* IOCTL Commands ***************************************************************************/

#define LCDIOC_BACKLIGHT _LCDIOC(0x0001) /* Arg: bool value */

#define LPM013M091A_LCDWIDTH  320
#define LPM013M091A_LCDHEIGHT 300

#define LPM013M091A_SWRESET 0x01

#define LPM013M091A_SLPIN   0x10
#define LPM013M091A_SLPOUT  0x11

#define LPM013M091A_DISPOFF 0x28
#define LPM013M091A_DISPON  0x29
#define LPM013M091A_CASET 0x2A
#define LPM013M091A_PASET 0x2B
#define LPM013M091A_RAMWR 0x2C

#define LPM013M091A_PIXFMT 0x3A

#define LPM013M091A_B3 0xB3
#define LPM013M091A_BB 0xBB

#define LPM013M091A_F3 0xF3
#define LPM013M091A_FB 0xFB
#define LPM013M091A_FF 0xFF

/* color info for 16bit RGB */
#define	LPM013M091A_BLACK   0x0000
#define	LPM013M091A_BLUE    0x001F
#define	LPM013M091A_RED     0xF800
#define	LPM013M091A_GREEN   0x07E0
#define LPM013M091A_CYAN    0x07FF
#define LPM013M091A_MAGENTA 0xF81F
#define LPM013M091A_YELLOW  0xFFE0
#define LPM013M091A_WHITE   0xFFFF

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/*******************************************************************************
 * Response CODE
 *******************************************************************************/

typedef enum
{
  ResCodeOk = 0x00000000,
  ResCodeErr,
  ResCodeOutOfDisp,
  ResCodeSpiSendErr,
  ResCodeTimeOut,
  ResCodeInitializeErr,
  ResCodeResetErr,
  ResCodeSizeErr,
} ResCode;

typedef enum
{
  TransferCommand = 0,
  TransferData = 1,
} TransferMode;

struct lpm013m091a_lcd_s
{
  ResCode (*init)(FAR struct lpm013m091a_lcd_s *lcd);
  ResCode (*sendReset)(FAR struct lpm013m091a_lcd_s *lcd);
  ResCode (*setTransferMode)(FAR struct lpm013m091a_lcd_s *lcd, TransferMode mode);
  ResCode (*sendData)(FAR struct lpm013m091a_lcd_s *lcd, uint32_t data);
  ResCode (*sendDataBurst)(FAR struct lpm013m091a_lcd_s *lcd, const void *data, uint32_t size);
};

struct lpm013m091a_base_s
{
  struct lpm013m091a_lcd_s* lcd;
  struct spi_dev_s* spi;
};

FAR struct lcd_dev_s* lpm013m091a_initialize(struct lpm013m091a_lcd_s *lcd, struct spi_dev_s *spi);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LCD_LPM013M091A_H */
