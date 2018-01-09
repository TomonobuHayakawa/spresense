/****************************************************************************
 * modules/cmdfw/spritzer_command_parser.c
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
#include "spritzer_command_parser.h"
#include "spritzer_command_event.h"

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

#define SIMPLE_FIFO_BUF_SIZE CONFIG_SPCOMMAND_PACKET_BUFFER_SIZE

static uint32_t s_fifo_buff[SIMPLE_FIFO_BUF_SIZE/sizeof(uint32_t)];
static CMN_SimpleFifoHandle s_fifo_handle;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
static int32_t recent_sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static SpCmdResult detect_packet(uint32_t* packet_size);
static bool search_delimiter(spcommand_adset_t* data, uint16_t* offset);
static SpCmdResult check_validity(spcommand_header_t* header, spcommand_adset_t* data);
static uint16_t calc_checksum(uint8_t* data, uint32_t size);

/****************************************************************************
 * Public Functions
 ****************************************************************************/
bool SpritzerCommandParser_activate(SpCmdFwHandle* p_handle)
{
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
  /* initialize sequence number */

  recent_sequence_no = -1;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

  /* initialize buffer for received data */

  if (CMN_SimpleFifoInitialize(&s_fifo_handle, s_fifo_buff, sizeof(s_fifo_buff), NULL) != 0)
    {
      printf("cannot initialize simple fifo.\n");
      return false;
    }

  /* set handle */

  *p_handle = (SpCmdFwHandle)&s_fifo_handle;

  return true;
}

/*--------------------------------------------------------------------*/
bool SpritzerCommandParser_deactivate(void)
{
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
  /* clear sequence number */

  recent_sequence_no = 0;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

  /* clear buffer for received data */

  CMN_SimpleFifoClear(&s_fifo_handle);

  return true;
}

/*--------------------------------------------------------------------*/
SpCmdResult SpritzerCommandParser_putData(uint8_t* p_data, uint32_t size)
{
  /* data check */

  if ((p_data == NULL) || (size == 0))
    {
      printf("invalid data: addr %08x size %d\n");
      return ParamErr;
    }

  /* store data to fifo which will be retrieved by command parser */

  if (CMN_SimpleFifoOffer(&s_fifo_handle, (void*)p_data, size) != size)
    {
      printf("simple fifo offer error\n");
    }

  return NoErr;
}

/*--------------------------------------------------------------------*/
SpCmdResult SpritzerCommandParser_parse(spcommand_header_t* header, uint8_t* payload)
{
  SpCmdResult result;
  uint32_t packet_size = 0;

  /* check having whole packet ? (sum check inside) */

  result = detect_packet(&packet_size);

  if (result == DataShortage)
    {
      return result;
    }

  /* hold recieved data (header) */

  CMN_SimpleFifoPoll(&s_fifo_handle, (void*)header, sizeof(spcommand_header_t));

  if (result == CheckSumErr)
    {
      /* even if checksum error, return the header data to send error reply packet */

      return result;
    }

  /* hold recieved data (payload) */

  CMN_SimpleFifoPoll(&s_fifo_handle, (void*)payload, packet_size - sizeof(spcommand_header_t));

#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
  /* check sequencial no */

  recent_sequence_no = (recent_sequence_no + 1) & 0xffff;
  if (recent_sequence_no != header->sequence_no)
    {
      cmdfwvdbg("sequence number unmatch. exp %d != rcv %d\n", recent_sequence_no, header->sequence_no);
    }
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

  return result;
}

/*--------------------------------------------------------------------*/
SpCmdResult SpritzerCommandParser_parseIndicatedArea(uint8_t* target_area, uint16_t target_size, spcommand_packet_info* info)
{
  spcommand_adset_t target = { { { 0 } } };
  uint16_t offset_to_delimiter = 0;
  uint8_t* packet_addr = NULL;
  uint16_t packet_size = 0;
  bool found = false;
  SpCmdResult result = NoErr;

  /* set target addr and size */

  target.adset[0].addr = target_area;
  target.adset[0].size = target_size;

  /* search delimiter */

  found = search_delimiter(&target, &offset_to_delimiter);

  packet_addr = target_area + offset_to_delimiter;
  packet_size = target_size - offset_to_delimiter;

  /* header exists ? */

  if (found && (packet_size >= sizeof(spcommand_header_t)))
    {
      /* set payload addres and size */

      target.adset[0].addr = packet_addr + sizeof(spcommand_header_t);
      target.adset[0].size = packet_size - sizeof(spcommand_header_t);

      /* check packet validity */

      result = check_validity((spcommand_header_t*)packet_addr, &target);

      if (result == NoErr)
        {
          /* set parse result */

          spcommand_header_t* header = (spcommand_header_t*)packet_addr;

          info->req_reply    = (header->param & (0x01 << SPCMD_REPLY_REQUEST)) ? true : false;
          info->is_reply     = (header->param & (0x01 << SPCMD_REPLY)) ? true : false;
          info->is_error     = (header->param & (0x01 << SPCMD_ERROR)) ? true : false;
          info->payload_addr = target.adset[0].addr;
          info->payload_size = header->payload_size;
          info->command_id   = header->command_id;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
          info->sequence_no  = header->sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */
        }
    }
  else
    {
      result = DataShortage;
    }

  return result;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static SpCmdResult detect_packet(uint32_t* packet_size)
{
  CMN_SimpleFifoPeekHandle peek_handle = { { 0 } };
  size_t stored_size = CMN_SimpleFifoGetOccupiedSize(&s_fifo_handle);
  spcommand_header_t peeked_header;
  spcommand_adset_t target = { { { 0 } } };
  uint16_t offset_to_delimiter = 0;
  uint8_t region = 0;
  bool found = false;
  SpCmdResult result = NoErr;

  *packet_size = 0;

  /* peek all data in fifo */

  CMN_SimpleFifoPeekWithOffset(&s_fifo_handle, &peek_handle, stored_size, 0);

  for (region = 0; region < NUM_ADSET_REGION; region++)
    {
      target.adset[region].addr = peek_handle.m_pChunk[region];
      target.adset[region].size = peek_handle.m_szChunk[region];
    }

  /* search delimiter */

  found = search_delimiter(&target, &offset_to_delimiter);
  cmdfwvdbg("found ? [%d]. Discard %d bytes.\n", found, offset_to_delimiter);

  if (offset_to_delimiter)
    {
      /* dispose data which is stored between rp and delimiter (NULL means take as trashbox)*/

      CMN_SimpleFifoPoll(&s_fifo_handle, NULL, offset_to_delimiter);

      /* peek again */

      stored_size = CMN_SimpleFifoGetOccupiedSize(&s_fifo_handle);
      CMN_SimpleFifoPeekWithOffset(&s_fifo_handle, &peek_handle, stored_size, 0);
    }

  /* header received ? */

  if (found && (stored_size >= sizeof(spcommand_header_t)))
    {
      /* copy header from fifo */

      CMN_SimpleFifoCopyFromPeekHandle(&peek_handle, (void*)&peeked_header, sizeof(spcommand_header_t));

      /* peek payload */

      CMN_SimpleFifoPeekWithOffset(&s_fifo_handle, &peek_handle,
                                   stored_size - sizeof(spcommand_header_t),
                                   sizeof(spcommand_header_t));

      for (region = 0; region < NUM_ADSET_REGION; region++)
        {
          target.adset[region].addr = peek_handle.m_pChunk[region];
          target.adset[region].size = peek_handle.m_szChunk[region];
        }

      /* check header validity */

      result = check_validity(&peeked_header, &target);

      if (result == NoErr)
        {
          *packet_size = sizeof(spcommand_header_t) + peeked_header.payload_size + SPCOMMAND_CRC_SIZE;
          cmdfwvdbg("Receive valid packet. size[%d]\n", *packet_size);
        }
    }
  else
    {
      result = DataShortage;
    }

  return result;
}

/*--------------------------------------------------------------------*/
static bool  search_delimiter(spcommand_adset_t* data, uint16_t* offset)
{
  DelimiterState statecnt = DELIMITER_NOTFOUND;
  bool found = false;
  uint8_t region = 0;

  *offset = 0;

  for (region = 0; region < NUM_ADSET_REGION; region++)
    {
      uint8_t* ptr = data->adset[region].addr;
      uint16_t sz = data->adset[region].size;
      uint32_t check_cnt = 0;

      while ((check_cnt < sz) && !found)
        {
          if (statecnt == DELIMITER_NOTFOUND)
            {
              if (*ptr == (COMMAND_DELIMITER & 0xff))
                {
                  statecnt = DELIMITER_1STBYTE;
                }
            }
          else if (statecnt == DELIMITER_1STBYTE)
            {
              if (*ptr == ((COMMAND_DELIMITER >> 8) & 0xff))
                {
                  statecnt = DELIMITER_2NDBYTE;
                  found = true;
                }
              else if (*ptr == (COMMAND_DELIMITER & 0xff))
                {
                  /* keep statecnt */
                }
              else
                {
                  statecnt = DELIMITER_NOTFOUND;
                }
            }
          else
            {
              /* keep statecnt */
            }

          check_cnt++;
          ptr++;
        }

      *offset += check_cnt;

      if (found)
        {
          break;
        }
    }

  *offset -= statecnt;

  return found;
}

/*--------------------------------------------------------------------*/
static SpCmdResult check_validity(spcommand_header_t* header, spcommand_adset_t* payload)
{
  uint16_t check_sum = 0;
  SpCmdResult result = NoErr;

  /* check header validity */

  check_sum = calc_checksum((uint8_t*)header, sizeof(spcommand_header_t) - sizeof(uint16_t));

  if (header->check_sum == check_sum)
    {
      /* whole packet exists ? */

      if (payload->adset[0].size + payload->adset[1].size < header->payload_size + SPCOMMAND_CRC_SIZE)
        {
          result = DataShortage;
        }
#ifdef CONFIG_SPCOMMAND_ENABLE_CRCCHECK
      else
        {
          /* check crc */

          if (header->payload_size != 0)
            {
              int region;
              uint16_t calc_crc = 0;
              uint16_t recv_crc = 0;
              int ofst = 0;
              uint8_t* addr = NULL;
              int sht = 8;
              int size = (payload->adset[0].size < header->payload_size) ? payload->adset[0].size : header->payload_size;

              /* calc crc code from payload */

              for (region = 0; region < NUM_ADSET_REGION; region++)
                {
                  calc_crc = crc16part(payload->adset[region].addr, size, calc_crc);
                  size = header->payload_size - size;
                }

              /* get received crc code */

              for (region = NUM_ADSET_REGION - 1; region >= 0; region--)
                {
                  addr = payload->adset[region].addr;
                  ofst = payload->adset[region].size;

                  while(--ofst >= 0)
                    {
                      recv_crc |= (uint16_t)*(addr + ofst) << sht;
                      sht -= 8;

                      if (sht < 0)
                        {
                          break;
                        }
                    }
                }

              if (recv_crc != calc_crc)
                {
                  cmdfwdbg("CommandParser_parse: crc error! crc = 0x%x(calc result = %x)\n", recv_crc, calc_crc);
                  result = CrcErr;
                }
            }
        }
#endif /* CONFIG_SPCOMMAND_ENABLE_CRCCHECK */
    }
  else
    {
      cmdfwdbg("checksum error. calc [%02x] != stored [%02x]\n", check_sum, header->check_sum);
      result = CheckSumErr;
    }

  return result;
}

/*--------------------------------------------------------------------*/
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


