/****************************************************************************
 * drivers/modem/alt1160_spi.h
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

#ifndef __DRIVERS_MODEM_ALT1160_SPI_H
#define __DRIVERS_MODEM_ALT1160_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>
#include <semaphore.h>
#include "alt1160_dev.h"
#include "alt1160_sys.h"

#if defined(CONFIG_MODEM_ALT_1160)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure describes the transfer header. */

struct alt1160_spi_xferhdr_s
{
  uint8_t header[4]; /* Transfer header. */
};

/* This structure describes the buffer for data receive. */

struct alt1160_spi_rxbuff_s
{
  char                        *buff_addr; /* Receive buffer address. */
  uint32_t                    buff_size;  /* Size of this buffer. */
  uint32_t                    rx_size;    /* Received data size. */
  struct alt1160_spi_rxbuff_s *next;      /* Link for next buffer. */
};

/* This structure describes the fifo for received buffer. */

struct alt1160_spi_rxbufffifo_s
{
  struct alt1160_spi_rxbuff_s *head; /* Point to the head of fifo */
  struct alt1160_spi_rxbuff_s *tail; /* Point to the tail of fifo */
  struct alt1160_sys_csem_s   csem;  /* It is used for notification
                                      * when data is put in fifo. */
};

/* This structure describes the parameters for receive buffer information. */

struct alt1160_spi_rxbuffinfo_s
{
  struct alt1160_spi_rxbuff_s     *free_buff;  /* Free receive buffer
                                                * address. */
  struct alt1160_spi_rxbufffifo_s fifo;        /* Receive buffer fifo. */
};

/* This structure describes the parameters for send data. */

struct alt1160_spi_tx_s
{
  struct alt1160_sys_lock_s    lock;        /* Lock on accessing the
                                             * following parameters. */
  struct alt1160_sys_flag_s    done_flag;   /* Notify that tx request
                                             * has been completed. */
  struct alt1160_spi_xferhdr_s header;      /* Tx header. */
  char                         *buff_addr;  /* Buffer address for data
                                             * transmission speceified by
                                             * the user. */
  int32_t                      actual_size; /* Actual data size. */
  int32_t                      total_size;  /* Data size of 4byte
                                             * alignment. */
  int32_t                      result;      /* Result of transfer. */
};

/* This structure describes the parameters for receive data. */

struct alt1160_spi_rx_s
{
  struct alt1160_sys_lock_s       lock;        /* Lock on accessing the
                                                * following parameters. */
  struct alt1160_spi_xferhdr_s    header;      /* Rx header. */
  int32_t                         actual_size; /* Actual data size */
  int32_t                         total_size;  /* Data size of 4byte
                                                * alignment. */
  struct alt1160_spi_rxbuff_s     *rxbuff;     /* Current recieve beffer. */
  bool                            rxabort;     /* Indicates whether the
                                                * rx process is aborted. */
};

/* This structure describes the parameters for sleep modem. */

struct alt1160_spi_sleepmodem_s
{
  struct alt1160_sys_lock_s lock;       /* Lock on accessing the
                                         * following parameters. */
  struct alt1160_sys_flag_s done_flag;  /* Notify that sleep request
                                         * has been completed. */
  int32_t                   result;     /* Result of sleep request. */
  bool                      requested;  /* Indicates that sleep request
                                         * has been requested. */
  timer_t                   sv_timerid; /* Superviser timer. */
};

/* This structure describes the resource of the ALT1160 spi driver */

struct alt1160_spi_dev_s
{
  /* Common fields */

  bool                             is_not_run;    /* Indicates xfer task
                                                   * is not run. */
  int32_t                          task_id;       /* xfer task ID. */
  struct alt1160_sys_flag_s        xfer_flag;     /* Used for event handling
                                                   * of xfer task. */
  struct alt1160_sys_flag_s        dma_done_flag; /* Notify that DMA transfer
                                                   * has been completed. */

  /* Parameter for recieve buffer */

  struct alt1160_spi_rxbuffinfo_s  rxbuffinfo;

  /* Parameter for send data */

  struct alt1160_spi_tx_s          tx_param;

  /* Parameter for recieve data */

  struct alt1160_spi_rx_s          rx_param;

  /* Parameters for sleep modem */

  struct alt1160_spi_sleepmodem_s sleep_param;
};

#endif
#endif /* __DRIVERS_MODEM_ALT1160_SPI_H */
