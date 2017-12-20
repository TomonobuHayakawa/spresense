/****************************************************************************
 * drivers/modem/alt1160_dev.h
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Yutaka Miyajima <Yutaka.Miyajima@sony.com>
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

#ifndef __DRIVERS_MODEM_ALT1160_DEV_H
#define __DRIVERS_MODEM_ALT1160_DEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/modem/alt1160.h>

#include "alt1160_spi.h"
#include "alt1160_sys.h"

#if defined(CONFIG_MODEM_ALT_1160)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct alt1160_dev_s
{
  FAR struct spi_dev_s      *spi;
  struct alt1160_spi_dev_s  spidev;
  struct alt1160_sys_lock_s lock;
  int                       poweron;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: alt1160_spi_init
 *
 * Description:
 *   Initialize ALT1160 driver.
 *
 ****************************************************************************/

int alt1160_spi_init(FAR struct alt1160_dev_s *priv);

/****************************************************************************
 * Name: alt1160_spi_uninit
 *
 * Description:
 *   Uninitialize ALT1160 driver.
 *
 ****************************************************************************/

int alt1160_spi_uninit(FAR struct alt1160_dev_s *priv);

/****************************************************************************
 * Name: alt1160_spi_read
 *
 * Description:
 *   ALT1160 SPI driver read method.
 *
 ****************************************************************************/

ssize_t alt1160_spi_read(FAR struct alt1160_dev_s *priv,
                         FAR const char *buffer, size_t readlen);

/****************************************************************************
 * Name: alt1160_spi_write
 *
 * Description:
 *   ALT1160 SPI driver write method.
 *
 ****************************************************************************/

ssize_t alt1160_spi_write(FAR struct alt1160_dev_s *priv,
                          FAR const char *buffer, size_t witelen);

/****************************************************************************
 * Name: alt1160_spi_readabort
 *
 * Description:
 *   Abort the read process.
 *
 ****************************************************************************/

int alt1160_spi_readabort(FAR struct alt1160_dev_s *priv);


/****************************************************************************
 * Name: alt1160_spi_sleepmodem
 *
 * Description:
 *   Make ALT1160 sleep.
 *
 ****************************************************************************/

int alt1160_spi_sleepmodem(FAR struct alt1160_dev_s *priv);

#ifdef CONFIG_MODEM_ALT_1160_PROTCOL_V2_1

/****************************************************************************
 * Name: alt1160_spi_setreceiverready
 *
 * Description:
 *   Set receiver ready notification.
 *
 ****************************************************************************/

int alt1160_spi_setreceiverready(FAR struct alt1160_dev_s *priv);

/****************************************************************************
 * Name: alt1160_spi_isreceiverready
 *
 * Description:
 *   Check already notified or not by alt1160_spi_setreceiverready.
 *
 ****************************************************************************/

int alt1160_spi_isreceiverready(FAR struct alt1160_dev_s *priv);

/****************************************************************************
 * Name: alt1160_spi_clearreceiverready
 *
 * Description:
 *   Clear receiver ready notification.
 *
 ****************************************************************************/

int alt1160_spi_clearreceiverready(FAR struct alt1160_dev_s *priv);

#endif

/****************************************************************************
 * Name: alt1160_spi_gpioreadyisr
 *
 * Description:
 *   Interrupt handler for SLAVE_REQUEST GPIO line.
 *
 ****************************************************************************/

int alt1160_spi_gpioreadyisr(int irq, FAR void *context, FAR void *arg);

#endif
#endif /* __DRIVERS_MODEM_ALT1160_DEV_H */
