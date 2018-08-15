/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_et014tt1.h
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Kei Yamamoto <Kei.x.Yamamoto@sony.com>
 *           Tomonobu Hayakawa <Tomonobu.Hayakawa@sony.com>
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
#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_ET014TT1_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_ET014TT1_H

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

typedef struct
{
    int32_t Width;
    int32_t Height;
    int32_t BPP;
}EPD_TCON_PANEL_INFO;

typedef enum
{
    SPI_SPEED_LOW = 0,
    SPI_SPEED_HIGH = 1
}SPI_SPEED_TYPE;

typedef enum
{
    TIMER_RUNNING = 1,
    TIMER_STOP = 0,
}TIMER_STATE_TYPE;

typedef enum
{
    EPD_BORDER_OFF   = 0,
    EPD_BORDER_BLACK = 1,
    EPD_BORDER_WHITE = 2
}EPD_BORDER_TYPE;

typedef struct
{
    /* SPI HAL */
    void (*Spi_SetClock)(SPI_SPEED_TYPE speed);
    void (*Spi_CS_Enable)(void);
    void (*Spi_CS_Disable)(void);
    void (*Spi_Write)(const uint8_t *buf, int32_t len);
    uint8_t (*Spi_ReadByte)(void);

    /* GPIO HAL */
    void (*SetResetPin)(void);
    void (*ClrResetPin)(void);
    void (*SetPowerOnPin)(void);
    void (*ClrPowerOnPin)(void);
    void (*SetOEIPin)(void);
    void (*ClrOEIPin)(void);
    uint32_t (*ReadBusyPin)(void);
    void (*DelayMS)(int32_t ms);

    void (*OnFrameStartEvent)(void);

    void (*StartTimer)(uint32_t ns);
    TIMER_STATE_TYPE (*GetTimerState)(void);
}EPD_TCON_DRIVER_HAL;

const EPD_TCON_PANEL_INFO *EPD_TCON_GetPanelInfo(void);
void     EPD_TCON_Update(uint8_t mode);
void     EPD_TCON_UpdateWithReagl(uint8_t mode);
int32_t  EPD_TCON_Init(const EPD_TCON_DRIVER_HAL *drvHal);
void     EPD_TCON_PowerOff(void);
void     EPD_TCON_PowerOn(void);
void     EPD_TCON_PanelClear(void);
void     EPD_TCON_SetVcom(int32_t mV);
uint8_t *EPD_TCON_GetUpdateBuffer(void);
uint32_t EPD_TCON_LoadWaveform(const uint8_t *data);
void     EPD_TCON_SetFrameRate(uint16_t fr);
uint16_t EPD_TCON_GetFrameRate(void);
void     EPD_TCON_Set_Border(EPD_BORDER_TYPE border,
                    uint16_t frame, uint16_t delayms);
int16_t  EPD_TCON_ReadTemperature(void);


struct et014tt1_pin_s
{
    int16_t rst;
    int16_t busy;
    int16_t cs;
    int16_t oei;
    int16_t power;
};

FAR struct fb_vtable_s *et014tt1_initialize(FAR struct spi_dev_s *spi,
                    struct et014tt1_pin_s *pin);
int et014tt1_register(FAR const char *devpath);
void et014tt1_configspi(FAR struct spi_dev_s *spi);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */

#endif

