/****************************************************************************
 * bsp/include/nuttx/lcd/et014tt1.h
 *
 *   Copyright (C) 2017 Sony Corporation
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

#ifndef __INCLUDE_NUTTX_LCD_ET014TT1_H
#define __INCLUDE_NUTTX_LCD_ET014TT1_H

/*****************************************************************************
 * Public Types
 ****************************************************************************/

#define ET014TT1_SPI_SPEED_LOW  0
#define ET014TT1_SPI_SPEED_HIGH 1

#define ET014TT1_TIMER_STOP    0
#define ET014TT1_TIMER_RUNNING 1

struct et014tt1_lcd_s
{
  /* This interface structure defined as EPD_TCON_DRIVER_HAL in SWT.
   *
   */

  void     (*spisetclock)(int speed);
  void     (*spicsenable)(void);
  void     (*spicsdisable)(void);
  void     (*spiwrite)(const uint8_t *buf, int32_t len);
  uint8_t  (*spireadbyte)(void);

  void     (*setresetpin)(void);
  void     (*clrresetpin)(void);
  void     (*setpoweronpin)(void);
  void     (*clrpoweronpin)(void);
  void     (*setoeipin)(void);
  void     (*clroeipin)(void);
  uint32_t (*readbusypin)(void);
  void     (*delayms)(int32_t ms);

  void     (*onframestartevent)(void);

  void     (*starttimer)(uint32_t ns);
  int      (*gettimerstate)(void);
};

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/**************************************************************************************
 * Public Function Prototypes
 **************************************************************************************/

/****************************************************************************
 * Name: et014tt1_initialize
 *
 * Description:
 *   Initialize LCD
 *
 ****************************************************************************/

FAR struct lcd_dev_s *et014tt1_initialize(struct et014tt1_lcd_s *lcd, int devno);

/****************************************************************************
 * Name: et014tt1_update
 *
 * This function is non NuttX standard interface.
 *
 * ET014TT1 ePaper device is too slow for displaying. So we provide special
 * update interface to applications for faster updating and lower power
 * comsumption.
 * Thus, application must call this after frame buffer updated to show drawing
 * results.
 *
 ****************************************************************************/

void et014tt1_update(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif
