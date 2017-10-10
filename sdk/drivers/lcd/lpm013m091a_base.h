/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_lpm013m091a_base.h
 *
 * Device driver base for LPM013M091A LCD.
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
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
 ****************************************************************************/

#ifndef __DRIVERS_LCD_LPM013M091A_BASE_H
#define __DRIVERS_LCD_LPM013M091A_BASE_H

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/lcd/lpm013m091a.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration */

#define _SPI_MAXFREQUENCY 20000000

/* Display resolution */

#define LPM013M091A_XRES        320
#define LPM013M091A_YRES        300

/* Dolor depth and format */

#define LPM013M091A_BPP           16
#define LPM013M091A_COLORFMT      FB_FMT_RGB16_565

/* Bytes per logical row and column */

#define LPM013M091A_XSTRIDE       (LPM013M091A_XRES << 1)
#define LPM013M091A_YSTRIDE       (LPM013M091A_YRES << 1)

#define LPM013M091A_FBSIZE        (LPM013M091A_XSTRIDE*LPM013M091A_YRES)

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

ResCode lpm013m091a_setDrawArea(FAR struct lpm013m091a_lcd_s *lcd,
                                int16_t start_x, int16_t start_y, int16_t end_x,
                                int16_t end_y);
ResCode lpm013m091a_drawBitmap(FAR struct lpm013m091a_lcd_s *lcd,
                               FAR const uint16_t *bitmap, int16_t x, int16_t y,
                               int16_t w, int16_t h);
ResCode lpm013m091a_turnBacklight(FAR struct lpm013m091a_lcd_s *lcd,
                                  bool on);
ResCode lpm013m091a_init(FAR struct lpm013m091a_lcd_s *lcd);
ResCode lpm013m091a_sendData(FAR struct lpm013m091a_lcd_s *lcd, uint32_t data);
ResCode lpm013m091a_sendDataBurst(FAR struct lpm013m091a_lcd_s *lcd,
                                  const void *data, uint32_t size);

FAR struct lpm013m091a_base_s *lpm013m091a_base_initialize(
    FAR struct lpm013m091a_lcd_s *lcd, struct spi_dev_s *spi);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_LCD_LPM013M091A_BASE_H */
