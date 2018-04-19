/****************************************************************************
 * configs/cxd56xx/src/cxd56_et014tt1.c
 *
 *   Copyright (C) 2018 Sony Corporation
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/time.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/et014tt1.h>
#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include "cxd56_gpio.h"
#include "cxd56_spi.h"
#include "cxd56_pinconfig.h"

#if defined(CONFIG_LCD_ET014TT1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check if the following are defined in the board.h */

#ifndef DISPLAY_SPI
#  error "DISPLAY_SPI must be defined in board.h !!"
#endif
#ifndef EINK_RST
#  error "EINK_RST must be defined in board.h !!"
#endif
#ifndef EINK_BUSY
#  error "EINK_BUSY must be defined in board.h !!"
#endif
#ifndef EINK_CS
#  error "EINK_CS must be defined in board.h !!"
#endif
#ifndef EINK_OEI
#  error "EINK_OEI must be defined in board.h !!"
#endif
#ifndef EINK_POWER
#  error "EINK_POWER must be defined in board.h !!"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#define SPI_FREQUENCY20MHz   (20000000)
#define SPI_FREQUENCY40MHz   (40000000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct et014tt1_pin_s
{
    int16_t rst;
    int16_t busy;
    int16_t cs;
    int16_t oei;
    int16_t power;
};

struct cxd56_et014tt1_lcd_s
{
  struct et014tt1_lcd_s lcd;
  FAR struct spi_dev_s *spi;

  /* Pin configuration. Each pins are defined at board.h */

  struct et014tt1_pin_s pin;

  /* Store starting time for timer emulation */

  int timerstate;
  struct timespec expiredtime;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct cxd56_et014tt1_lcd_s g_lcddev;
static FAR struct lcd_dev_s *g_lcd = NULL;

/****************************************************************************
 * Private functions
 ****************************************************************************/

static inline void cxd56_et014tt1_pininitialize(void)
{
  FAR struct cxd56_et014tt1_lcd_s *priv = &g_lcddev;
  FAR struct et014tt1_pin_s *pin = &priv->pin;

  pin->rst   = EINK_RST;
  pin->busy  = EINK_BUSY;
  pin->cs    = EINK_CS;
  pin->oei   = EINK_OEI;
  pin->power = EINK_POWER;

  cxd56_gpio_config(pin->rst, false);
  cxd56_gpio_config(pin->busy, true);
  cxd56_gpio_config(pin->cs, false);
}

/****************************************************************************
 * Name: et014tt1_configspi
 ****************************************************************************/

static void cxd56_et014tt1_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for ET014TT1 */

  SPI_SETMODE(spi, SPIDEV_MODE3);
  SPI_SETBITS(spi, 8);

  SPI_HWFEATURES(spi, 0);
}

/****************************************************************************
 * Name: cxd56_et014tt1_spisetclock
 ****************************************************************************/

static void cxd56_et014tt1_spisetclock(int speed)
{
  struct cxd56_et014tt1_lcd_s *priv = &g_lcddev;

  switch (speed)
    {
      case ET014TT1_SPI_SPEED_LOW:
        SPI_SETFREQUENCY(priv->spi, SPI_FREQUENCY20MHz);
        break;

      case ET014TT1_SPI_SPEED_HIGH:
        SPI_SETFREQUENCY(priv->spi, SPI_FREQUENCY40MHz);
        break;

      default:
        lcderr("Unsupported speed: %d\n", speed);
        break;
    }
}

/****************************************************************************
 * Name: cxd56_et014tt1_spicsenable
 ****************************************************************************/

static void cxd56_et014tt1_spicsenable(void)
{
  struct cxd56_et014tt1_lcd_s *priv = &g_lcddev;
  cxd56_gpio_write(priv->pin.cs, false);
}

/****************************************************************************
 * Name: cxd56_et014tt1_spicsdisable
 ****************************************************************************/

static void cxd56_et014tt1_spicsdisable(void)
{
  struct cxd56_et014tt1_lcd_s *priv = &g_lcddev;
  cxd56_gpio_write(priv->pin.cs, true);
}

/****************************************************************************
 * Name: cxd56_et014tt1_spiwrite
 ****************************************************************************/

static void cxd56_et014tt1_spiwrite(const uint8_t *buf, int32_t len)
{
  struct cxd56_et014tt1_lcd_s *priv = &g_lcddev;

  SPI_LOCK(priv->spi, true);

  cxd56_et014tt1_configspi(priv->spi);

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);
  SPI_SNDBLOCK(priv->spi, buf, len);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: cxd56_et014tt1_spireadbyte
 ****************************************************************************/

static uint8_t cxd56_et014tt1_spireadbyte(void)
{
  struct cxd56_et014tt1_lcd_s *priv = &g_lcddev;
  int ret = -1;

  SPI_LOCK(priv->spi, true);

  cxd56_et014tt1_configspi(priv->spi);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);

  SPI_RECVBLOCK(priv->spi, &ret, 1);

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);
  SPI_LOCK(priv->spi, false);

  return ret & 0xff;
}

/****************************************************************************
 * Name: cxd56_et014tt1_setresetpin
 ****************************************************************************/

static void cxd56_et014tt1_setresetpin(void)
{
  struct cxd56_et014tt1_lcd_s *priv = &g_lcddev;
  cxd56_gpio_write(priv->pin.rst, true);
}

/****************************************************************************
 * Name: cxd56_et014tt1_clrresetpin
 ****************************************************************************/

static void cxd56_et014tt1_clrresetpin(void)
{
  struct cxd56_et014tt1_lcd_s *priv = &g_lcddev;
  cxd56_gpio_write(priv->pin.rst, false);
}

/****************************************************************************
 * Name: cxd56_et014tt1_setpoweronpin
 ****************************************************************************/

static void cxd56_et014tt1_setpoweronpin(void)
{
  struct cxd56_et014tt1_lcd_s *priv = &g_lcddev;
  if (priv->pin.power >= 0)
    {
      cxd56_gpio_write(priv->pin.power, true);
    }
}

/****************************************************************************
 * Name: cxd56_et014tt1_clrpoweronpin
 ****************************************************************************/

static void cxd56_et014tt1_clrpoweronpin(void)
{
  struct cxd56_et014tt1_lcd_s *priv = &g_lcddev;
  if (priv->pin.power >= 0)
    {
      cxd56_gpio_write(priv->pin.power, false);
    }
}

/****************************************************************************
 * Name: cxd56_et014tt1_setoeipin
 ****************************************************************************/

static void cxd56_et014tt1_setoeipin(void)
{
  struct cxd56_et014tt1_lcd_s *priv = &g_lcddev;
  if (priv->pin.oei >= 0)
    {
      cxd56_gpio_write(priv->pin.oei, true);
    }
}

/****************************************************************************
 * Name: cxd56_et014tt1_clroeipin
 ****************************************************************************/

static void cxd56_et014tt1_clroeipin(void)
{
  struct cxd56_et014tt1_lcd_s *priv = &g_lcddev;
  if (priv->pin.oei >= 0)
    {
      cxd56_gpio_write(priv->pin.oei, false);
    }
}

/****************************************************************************
 * Name: cxd56_et014tt1_readbusypin
 ****************************************************************************/

static uint32_t cxd56_et014tt1_readbusypin(void)
{
  struct cxd56_et014tt1_lcd_s *priv = &g_lcddev;
  return (uint32_t)cxd56_gpio_read(priv->pin.busy);
}

/****************************************************************************
 * Name: cxd56_et014tt1_delayms
 ****************************************************************************/

static void cxd56_et014tt1_delayms(int32_t ms)
{
  up_mdelay(ms);
}

/****************************************************************************
 * Name: cxd56_et014tt1_onframestartevent
 ****************************************************************************/

static void cxd56_et014tt1_onframestartevent(void)
{
  /* Do nothing */
}

/****************************************************************************
 * Name: cxd56_et014tt1_starttimer
 ****************************************************************************/

static void cxd56_et014tt1_starttimer(uint32_t ns)
{
  FAR struct cxd56_et014tt1_lcd_s *priv = &g_lcddev;
  FAR struct timespec *exp = &priv->expiredtime;
  struct timespec ts;

  clock_gettime(CLOCK_REALTIME, &ts);

  exp->tv_sec = ts.tv_sec + (ns / 1000000000);
  exp->tv_nsec = ts.tv_nsec + (ns % 1000000000);

  priv->timerstate = ET014TT1_TIMER_RUNNING;
}

/****************************************************************************
 * Name: cxd56_et014tt1_gettimerstate
 ****************************************************************************/

static int cxd56_et014tt1_gettimerstate(void)
{
  FAR struct cxd56_et014tt1_lcd_s *priv = &g_lcddev;
  FAR struct timespec *exp = &priv->expiredtime;
  struct timespec now;

  if (priv->timerstate == ET014TT1_TIMER_RUNNING)
    {
      clock_gettime(CLOCK_REALTIME, &now);

      if (now.tv_sec > exp->tv_sec)
        {
          priv->timerstate = ET014TT1_TIMER_STOP;
        }
      else if (now.tv_sec == exp->tv_sec && now.tv_nsec > exp->tv_nsec)
        {
          priv->timerstate = ET014TT1_TIMER_STOP;
        }
    }

  return priv->timerstate;
}

/****************************************************************************
 * Public functions
 ****************************************************************************/
/****************************************************************************
 * Name: board_lcd_initialize
 *
 * Description:
 *   Called by NX initialization logic to configure the LCD
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  FAR struct cxd56_et014tt1_lcd_s *priv = &g_lcddev;
  FAR struct et014tt1_lcd_s *lcd = &priv->lcd;
  FAR struct spi_dev_s *spi;

  lcdinfo("Initializing lcd\n");

  if (g_lcd)
    {
      /* Display already initialized */

      return OK;
    }

  memset(priv, 0, sizeof(struct cxd56_et014tt1_lcd_s));

  /* Initialize private data */

  priv->spi = NULL;

  lcdinfo("initialize spi %d.\n", DISPLAY_SPI);
  spi = cxd56_spibus_initialize(DISPLAY_SPI);
  if (!spi)
    {
      lcderr("ERROR: Failed to initialize spi bus.\n");
      return -EINVAL;
    }

  cxd56_et014tt1_configspi(spi);

  /* Configure GPIO output pin */

  cxd56_et014tt1_pininitialize();

  board_power_control(POWER_EINK, true);

  priv->spi = spi;

  lcd->spisetclock       = cxd56_et014tt1_spisetclock;
  lcd->spicsenable       = cxd56_et014tt1_spicsenable;
  lcd->spicsdisable      = cxd56_et014tt1_spicsdisable;
  lcd->spiwrite          = cxd56_et014tt1_spiwrite;
  lcd->spireadbyte       = cxd56_et014tt1_spireadbyte;
  lcd->setresetpin       = cxd56_et014tt1_setresetpin;
  lcd->clrresetpin       = cxd56_et014tt1_clrresetpin;
  lcd->setpoweronpin     = cxd56_et014tt1_setpoweronpin;
  lcd->clrpoweronpin     = cxd56_et014tt1_clrpoweronpin;
  lcd->setoeipin         = cxd56_et014tt1_setoeipin;
  lcd->clroeipin         = cxd56_et014tt1_clroeipin;
  lcd->readbusypin       = cxd56_et014tt1_readbusypin;
  lcd->delayms           = cxd56_et014tt1_delayms;
  lcd->onframestartevent = cxd56_et014tt1_onframestartevent;
  lcd->starttimer        = cxd56_et014tt1_starttimer;
  lcd->gettimerstate     = cxd56_et014tt1_gettimerstate;

  g_lcd = et014tt1_initialize(lcd, 0);

  return OK;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This
 *   allows support for multiple LCD devices.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  if (lcddev == 0)
    {
      return g_lcd;
    }
  return NULL;
}

#endif /* CONFIG_EINK_ET014TT1 */
