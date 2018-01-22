/********************************************************************************************
 * include/nuttx/lcd/ili9340.h
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

#ifndef __INCLUDE_NUTTX_LCD_ILI9340_H
#define __INCLUDE_NUTTX_LCD_ILI9340_H

#include <sdk/config.h>
#include <nuttx/spi/spi.h>

#include <nuttx/fs/ioctl.h>

/* LCD driver ioctl definitions **********************************************/

#define _LCDBASE        (0x2100) /* LCD driver commands */
#define _LCDIOCVALID(c) (_IOC_TYPE(c)==_LCDBASE)
#define _LCDIOC(nr)     _IOC(_LCDBASE,nr)

/* IOCTL Commands ***************************************************************************/

#define LCDIOC_BACKLIGHT _LCDIOC(0x0001) /* Arg: bool value */

#define ILI9340_LCDWIDTH  320
#define ILI9340_LCDHEIGHT 240

#define ILI9340_NOP     0x00
#define ILI9340_SWRESET 0x01
#define ILI9340_RDDID   0x04
#define ILI9340_RDDST   0x09

#define ILI9340_SLPIN   0x10
#define ILI9340_SLPOUT  0x11
#define ILI9340_PTLON   0x12
#define ILI9340_NORON   0x13

#define ILI9340_RDMODE  0x0A
#define ILI9340_RDMADCTL  0x0B
#define ILI9340_RDPIXFMT  0x0C
#define ILI9340_RDIMGFMT  0x0A
#define ILI9340_RDSELFDIAG  0x0F

#define ILI9340_INVOFF  0x20
#define ILI9340_INVON   0x21
#define ILI9340_GAMMASET 0x26
#define ILI9340_DISPOFF 0x28
#define ILI9340_DISPON  0x29

#define ILI9340_CASET   0x2A
#define ILI9340_PASET   0x2B
#define ILI9340_RAMWR   0x2C
#define ILI9340_RAMRD   0x2E

#define ILI9340_PTLAR   0x30
#define ILI9340_TELON  0x35
#define ILI9340_MADCTL  0x36

#define ILI9340_IDLEON  0x38
#define ILI9340_IDLEOFF  0x39

#define ILI9340_MADCTL_MY  0x80
#define ILI9340_MADCTL_MX  0x40
#define ILI9340_MADCTL_MV  0x20
#define ILI9340_MADCTL_ML  0x10
#define ILI9340_MADCTL_RGB 0x00
#define ILI9340_MADCTL_BGR 0x08
#define ILI9340_MADCTL_MH  0x04

#define ILI9340_PIXFMT  0x3A

#define ILI9340_FRMCTR1 0xB1
#define ILI9340_FRMCTR2 0xB2
#define ILI9340_FRMCTR3 0xB3
#define ILI9340_INVCTR  0xB4
#define ILI9340_DFUNCTR 0xB6

#define ILI9340_PWCTR1  0xC0
#define ILI9340_PWCTR2  0xC1
#define ILI9340_PWCTR3  0xC2
#define ILI9340_PWCTR4  0xC3
#define ILI9340_PWCTR5  0xC4
#define ILI9340_VMCTR1  0xC5
#define ILI9340_VMCTR2  0xC7

#define ILI9340_RDID1   0xDA
#define ILI9340_RDID2   0xDB
#define ILI9340_RDID3   0xDC
#define ILI9340_RDID4   0xDD

#define ILI9340_GMCTRP1 0xE0
#define ILI9340_GMCTRN1 0xE1

#define ILI9340_IFCTRL 0xF6

#define	ILI9340_BLACK   0x0000
#define	ILI9340_BLUE    0x001F
#define	ILI9340_RED     0xF800
#define	ILI9340_GREEN   0x07E0
#define ILI9340_CYAN    0x07FF
#define ILI9340_MAGENTA 0xF81F
#define ILI9340_YELLOW  0xFFE0
#define ILI9340_WHITE   0xFFFF

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

enum ili9340_trans_mode_e
{
  TRANSFER_COMMAND = 0,
  TRANSFER_DATA,
};

typedef enum ili9340_trans_mode_e ili9340_trans_mode_t;

struct ili9340_lcd_s
{
  int (*init)(FAR struct ili9340_lcd_s *lcd);
  int (*send_reset)(FAR struct ili9340_lcd_s *lcd);
  int (*set_transfer_mode)(FAR struct ili9340_lcd_s *lcd, ili9340_trans_mode_t mode);
  int (*send_data)(FAR struct ili9340_lcd_s *lcd, uint32_t data);
  int (*send_data_burst)(FAR struct ili9340_lcd_s *lcd, const void *data, uint32_t size);
};

struct ili9340_base_s
{
  struct ili9340_lcd_s* lcd;
  struct spi_dev_s* spi;
};

FAR struct lcd_dev_s* ili9340_initialize(struct ili9340_lcd_s *lcd, struct spi_dev_s *spi);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LCD_ILI9340_H */
