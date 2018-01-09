/****************************************************************************
 * modules/cmdfw/spritzer_command_creator.c
 *
 *   Copyright (C) 2017 Sony Corpration. All rights reserved.
 *   Author: Ryuuta Sakane <Ryuuta.Sakane@Sony.com>
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

#include <string.h>
#ifdef CONFIG_SPCOMMAND_ENABLE_CRCCHECK
#include <crc16.h>
#endif /* CONFIG_SPCOMMAND_ENABLE_CRCCHECK */
#include "memutils/simple_fifo/CMN_SimpleFifo.h"
#include "spritzer_command_creator.h"

#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_COMMANDFW
#  define cmdfwdbg      dbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define cmdfwvdbg     vdbg
#  else
#    define cmdfwvdbg(x...)
#  endif
#else
#  define cmdfwdbg(x...)
#  define cmdfwvdbg(x...)
#endif

static CMN_SimpleFifoHandle s_fifo_handle;
static uint32_t s_fifo_buff[sizeof(spcommand_packet_t)/sizeof(uint32_t) + 1];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static uint16_t calc_checksum(uint8_t* data, uint32_t size);

/****************************************************************************
 * Public Functions
 ****************************************************************************/
bool SpritzerCommandCreator_activate(void)
{
  /* initialize buffer for created packet data */

  if (CMN_SimpleFifoInitialize(&s_fifo_handle, s_fifo_buff, sizeof(s_fifo_buff), NULL) != 0)
    {
      printf("cannot initialize simple fifo.\n");
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool SpritzerCommandCreator_deactivate(void)
{
  /* clear buffer for created packet data */

  CMN_SimpleFifoClear(&s_fifo_handle);

  return true;
}

/*--------------------------------------------------------------------*/
bool SpritzerCommandCreator_create(spcommand_create_param* param)
{
  spcommand_header_t header;
#ifdef CONFIG_SPCOMMAND_ENABLE_CRCCHECK
  uint16_t crc_code = 0;
#endif /* CONFIG_SPCOMMAND_ENABLE_CRCCHECK */

  /* param check */

  if ((param->payload_size != 0 && param->payload_addr == NULL) || param->payload_size > 2048)
    {
      cmdfwdbg("parameter error! size = %d(<=2048), payload = %08x(!NULL)\n",
               param->payload_size, param->payload_addr);
      return false;
    }

  /* at first, clear buffer. allways create at top of buffer */

  CMN_SimpleFifoClear(&s_fifo_handle);

  /* create and set command header */

  header.delimiter   = COMMAND_DELIMITER;
  header.param       = (param->is_reply << SPCMD_REPLY | param->is_error << SPCMD_ERROR);
  header.command_id  = (param->command_id & 0x03ff);
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
  header.sequence_no = param->sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */
  header.payload_size = param->payload_size;
  header.check_sum   = calc_checksum((uint8_t*)&(header), sizeof(header) - sizeof(uint16_t));

  if (CMN_SimpleFifoOffer(&s_fifo_handle, (void*)&header, sizeof(header)) != sizeof(header))
    {
      printf("simple fifo offer error\n");
    }

  /* for caluculate sum and crc */

  if (param->payload_size & 1)
    {
      param->payload_size += 1;
    }

  /* set payload */

  if (param->payload_size != 0)
    {
      if (CMN_SimpleFifoOffer(&s_fifo_handle,
                              (void*)param->payload_addr,
                              param->payload_size)
          != param->payload_size)
        {
          printf("simple fifo offer error %d %d\n",
                 CMN_SimpleFifoGetVacantSize(&s_fifo_handle), param->payload_size);
        }
    }

#ifdef CONFIG_SPCOMMAND_ENABLE_CRCCHECK
  /* set crc pattern */

  crc_code = (param->payload_size != 0) ? crc16(param->payload_addr, param->payload_size) : 0;

  if (CMN_SimpleFifoOffer(&s_fifo_handle, (void*)&crc_code, sizeof(crc_code)) != sizeof(crc_code))
    {
      printf("simple fifo offer error\n");
    }
#endif /* CONFIG_SPCOMMAND_ENABLE_CRCCHECK */

  return true;
}

/*--------------------------------------------------------------------*/
void SpritzerCommandCreator_getPacketInfo(uint8_t** addr, uint16_t* size)
{
  CMN_SimpleFifoPeekHandle peek_handle;

  /* peek and get addr of remain packet data */

  if (CMN_SimpleFifoPeekWithOffset(&s_fifo_handle, &peek_handle, 1, 0) != 0)
    {
      *addr = peek_handle.m_pChunk[0];
    }
  else
    {
      *addr = NULL;
    }

  /* get size of packet data*/

  *size = CMN_SimpleFifoGetOccupiedSize(&s_fifo_handle);
}

/*--------------------------------------------------------------------*/
bool SpritzerCommandCreator_readSkipData(uint32_t size, uint8_t** addr, uint16_t* remain_size)
{
  bool rtcd = true;

  /* dispose data (not copy to any buffers) */

  if (CMN_SimpleFifoPoll(&s_fifo_handle, NULL, size) != size)
    {
      printf("simple fifo poll error\n");

      rtcd = false;
    }

  /* return remain size */

  SpritzerCommandCreator_getPacketInfo(addr, remain_size);

  return rtcd;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static uint16_t calc_checksum(uint8_t* data, uint32_t size)
{
  uint16_t sum = 0;
  uint32_t i = 0;

  for (i = 0; i < size; i++)
    {
      sum += *data;
      data++;
    }

  return sum;
}


