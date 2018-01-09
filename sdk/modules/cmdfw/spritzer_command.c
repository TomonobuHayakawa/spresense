/****************************************************************************
 * modules/cmdfw/spritzer_command.c
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
#include "spritzer_command.h"
#include "spritzer_command_common.h"
#include "spritzer_command_parser.h"
#include "spritzer_command_creator.h"
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

static bool s_spcmd_active = false;

static SpCmdFwHandle s_spcmd_handle;
static SpCmdState s_state = Ready;
static spcommand_header_t s_violation_reply;
static spcommand_header_t s_reply_int;
static uint8_t __attribute__((aligned(2))) s_payload[PAYLOAD_BUFFER_SIZE];
static spcommand_header_t s_header;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
static bool create_error_reply(uint16_t command_id, uint16_t sequence_no);
#else
static bool create_error_reply(uint16_t command_id);
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */
static void proc_reserved_reply(void);

/****************************************************************************
 * Public Functions
 ****************************************************************************/
bool SpritzerCommand_activate(SpCmdFwHandle* p_handle)
{
  cmdfwdbg("Activate:\n");

  /* check activation state */

  if (s_spcmd_active)
    {
      printf("sp command fw is already initialized.\n");
      return false;
    }

  /* set active */

  s_spcmd_active = true;

  /* internal state */

  s_state = Ready;

  /* activate sub blocks */

  SpritzerCommandParser_activate(&s_spcmd_handle);
  SpritzerCommandCreator_activate();
  SpritzerCommandEvent_activate();

  /* set handle */

  *p_handle = s_spcmd_handle;

  return true;
}

/*--------------------------------------------------------------------*/
bool SpritzerCommand_deactivate(SpCmdFwHandle handle)
{
  cmdfwdbg("Deactivate:\n");

  /* handle validity check */

  if (handle != s_spcmd_handle)
    {
      printf("invalid handle.\n");
      return false;
    }

  /* clear activation state */

  s_spcmd_active = false;

  /* deactivate sub blocks */

  SpritzerCommandParser_deactivate();
  SpritzerCommandCreator_deactivate();
  SpritzerCommandEvent_deactivate();

  /* internal state */

  s_state = Ready;

  /* clear handle */

  s_spcmd_handle = 0;

  return true;
}

/*--------------------------------------------------------------------*/
SpCmdResult SpritzerCommand_putData(SpCmdFwHandle handle, uint8_t* p_data, uint32_t size)
{
  SpCmdResult result = NoErr;

  cmdfwdbg("Put data: from[%08x (%d byte)]\n", p_data, size);

  /* handle validity check */

  if (handle != s_spcmd_handle)
    {
      printf("invalid handle\n");
      return InvalidHandle;
    }

  /* put data into internal buffer of spcommandfw */

  result = SpritzerCommandParser_putData(p_data, size);

  return result;
}

/*--------------------------------------------------------------------*/
SpCmdResult SpritzerCommand_execute(SpCmdFwHandle handle)
{
  spcommand_header_t* header;
  uint8_t* payload;
  SpCmdResult result = NoErr;

  cmdfwdbg("Excute:\n");

  /* handle validity check */

  if (handle != s_spcmd_handle)
    {
      printf("invalid handle\n");
      return InvalidHandle;
    }

  /* parse packet */

  header  = &s_header;
  payload = (s_state & ReplyInt) ? NULL : s_payload;

  result = SpritzerCommandParser_parse(header, payload);

  /* whole packet received ? */

  if (result == DataShortage)
    {
      return DataShortage;
    }

  /* state check */

  if (s_state != Ready)
    {
      /* is reply packet ? */

      if (header->param & (0x01 << SPCMD_REPLY))
        {
          s_state |= ReplyInt;
          memcpy(&s_reply_int, header, sizeof(s_reply_int));

          printf("Reply int: id %04x\n", header->command_id);
        }
      else
        {
          /* if sync violation, skip proc, and reply error packet after current packet is sent */

          s_state |= SyncViolationInt;
          memcpy(&s_violation_reply, header, sizeof(s_violation_reply));

          printf("Sync violation: id %04x\n", header->command_id);
        }

       return CmdBusy;
    }

  /* command excute */

  if (result == NoErr)
    {
      /* do event work */

      event_param evparam;
      uint8_t* addr;
      uint16_t size;

      evparam.req_reply    = (header->param & (0x01 << SPCMD_REPLY_REQUEST)) ? true : false;
      evparam.is_reply     = (header->param & (0x01 << SPCMD_REPLY)) ? true : false;
      evparam.is_error     = (header->param & (0x01 << SPCMD_ERROR)) ? true : false;
      evparam.payload_addr = payload;
      evparam.payload_size = header->payload_size;
      evparam.command_id = header->command_id;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
      evparam.sequence_no  = header->sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

      result = SpritzerCommandEvent_proc(&evparam);

      if (SpritzerCommand_getPacket(&addr, &size))
        {
          cmdfwvdbg("packet created at[%08x (%d byte)]\n", addr, size);
          s_state = Working;
        }
    }
#ifdef CONFIG_SPCOMMAND_ENABLE_CRCCHECK
  else if (result == CheckSumErr || result == CrcErr)
#else
  else if (result == CheckSumErr)
#endif /* CONFIG_SPCOMMAND_ENABLE_CRCCHECK */
    {
      /* create error reply */

      bool rtcd;

#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
      rtcd = create_error_reply(header->command_id, header->sequence_no);
#else
      rtcd = create_error_reply(header->command_id);
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

      if (rtcd)
        {
          s_state = Working;
        }
    }
  else
    {
      /* do nothing */
    }

  return result;
}

/*--------------------------------------------------------------------*/
SpCmdResult SpritzerCommand_parse(SpCmdFwHandle handle, spcommand_packet_info* info)
{
  spcommand_header_t* header;
  uint8_t* payload;
  SpCmdResult result = NoErr;

  cmdfwdbg("Parse:\n");

  /* state check */

  if (s_state != Ready)
    {
      printf("previous command active\n");
      return CmdBusy;
    }

  /* handle validity check */

  if (handle != s_spcmd_handle)
    {
      printf("invalid handle\n");
      return InvalidHandle;
    }

  /* parse packet */

  header  = &s_header;
  payload = s_payload;

  result = SpritzerCommandParser_parse(header, payload);

  /* whole packet received ? */

  if (result == DataShortage)
    {
      return DataShortage;
    }

  /* set parse info */

  info->req_reply    = (header->param & (0x01 << SPCMD_REPLY_REQUEST)) ? true : false;
  info->is_reply     = (header->param & (0x01 << SPCMD_REPLY)) ? true : false;
  info->is_error     = (header->param & (0x01 << SPCMD_ERROR)) ? true : false;
  info->payload_addr = payload;
  info->payload_size = header->payload_size;
  info->command_id   = header->command_id;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
  info->sequence_no  = header->sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

  return result;
}

/*--------------------------------------------------------------------*/
SpCmdResult SpritzerCommand_eventProc(SpCmdFwHandle handle, event_param* evparam)
{
  SpCmdResult result = NoErr;
  uint8_t* addr;
  uint16_t size;

  cmdfwdbg("Event proc: command_id[%d]\n", evparam->command_id);

  /* state check */

  if (s_state != Ready)
    {
      printf("previous command active\n");
      return CmdBusy;
    }

  /* handle validity check */

  if (handle != s_spcmd_handle)
    {
      printf("invalid handle\n");
      return InvalidHandle;
    }

  /* do event work */

  result = SpritzerCommandEvent_proc(evparam);

  if (SpritzerCommand_getPacket(&addr, &size))
    {
      cmdfwvdbg("packet created %08x %d\n", addr, size);
      s_state = Working;
    }

  return result;
}

/*--------------------------------------------------------------------*/
bool SpritzerCommand_createPacket(SpCmdFwHandle handle, spcommand_create_param* cparam)
{
  bool rtcd = false;

  cmdfwdbg("Create packet: command_id[%d]\n", cparam->command_id);

  /* state check */

  if (s_state != Ready)
    {
      printf("previous command active\n");
      return rtcd;
    }

  /* handle validity check */

  if (handle != s_spcmd_handle)
   {
     printf("invalid handle\n");
     return rtcd;
   }

  /* create packet */

  rtcd = SpritzerCommandCreator_create(cparam);

  if (rtcd)
    {
      s_state = Working;
    }

  return rtcd;
}

/*--------------------------------------------------------------------*/
bool SpritzerCommand_freeData(SpCmdFwHandle handle, uint16_t read_size, uint8_t** next_addr, uint16_t* remain_size)
{
  bool rtcd = false;
  uint8_t* addr;
  uint16_t size;

  cmdfwdbg("Free data: size[%d]\n", read_size);

  /* handle validity check */

  if (handle != s_spcmd_handle)
   {
     printf("invalid handle\n");
     return 0;
   }

  /* dispose data which is already read */

  rtcd = SpritzerCommandCreator_readSkipData(read_size, next_addr, remain_size);

  if (*remain_size != 0)
    {
      /* current packet data is still remaining */
    }
  else
    {
      /* current packet data is all read
       * then next packet check (is succeeding packet exists ?)
       */

      if (!SpritzerCommandEvent_nextPacket())
        {
          /* if there is not succeding packet, check error reply for sync violation */

          if (s_state & SyncViolationInt)
            {
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
              create_error_reply(s_violation_reply.command_id, s_violation_reply.sequence_no);
#else
              create_error_reply(s_violation_reply.command_id);
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

              s_state = (SpritzerCommand_getPacket(&addr, &size)) ? (s_state & ~SyncViolationInt) : Ready;
              cmdfwvdbg("sync violation reply : id %04x, %08x %d\n", s_violation_reply.command_id, addr, size);
            }
          else if (s_state & ReplyInt)
            {
              proc_reserved_reply();

              s_state = (SpritzerCommand_getPacket(&addr, &size)) ? (s_state & ~ReplyInt) : Ready;
              cmdfwvdbg("reserved reply : %08x %d\n", addr, size);
            }
          else
            {
              s_state = Ready;
            }
        }
    }

  return rtcd;
}

/*--------------------------------------------------------------------*/
bool SpritzerCommand_getPacket(uint8_t** addr, uint16_t* size)
{
  bool is_packet_exist = false;

  cmdfwdbg("Get packet:\n");

  /* check current packet */

  SpritzerCommandCreator_getPacketInfo(addr, size);

  if (*size != 0 && *addr != NULL)
    {
      is_packet_exist = true;
    }

  return is_packet_exist;
}

/*--------------------------------------------------------------------*/
SpCmdResult SpritzerCommand_parseIndicatedArea(uint8_t* addr, uint16_t size, spcommand_packet_info* info)
{
  SpCmdResult result;

  cmdfwdbg("Parse indicated area: area[%08x (%d byte)]\n", addr, size);

  result = SpritzerCommandParser_parseIndicatedArea(addr, size, info);

  return result;
}

/*--------------------------------------------------------------------*/
bool SpritzerCommand_isEventRegistered(uint16_t command_id)
{
  cmdfwdbg("Is event registerd: command_id[%d]\n", command_id);

  return SpritzerCommandEvent_isEventRegistered(command_id);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
static bool create_error_reply(uint16_t command_id, uint16_t sequence_no)
#else
static bool create_error_reply(uint16_t command_id)
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */
{
  bool rtcd = true;

  spcommand_create_param cparam;

  cparam.req_reply    = false;
  cparam.is_reply     = true;
  cparam.is_error     = true;
  cparam.payload_addr = NULL;
  cparam.payload_size = 0;
  cparam.command_id   = command_id;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
  cparam.sequence_no  = sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

  rtcd = SpritzerCommandCreator_create(&cparam);

  return rtcd;
}

/*--------------------------------------------------------------------*/
static void proc_reserved_reply(void)
{
  event_param evparam;

  evparam.req_reply    = false;
  evparam.is_reply     = true;
  evparam.is_error     = (s_reply_int.param & (0x01 << SPCMD_ERROR)) ? true : false;
  evparam.payload_addr = s_payload;
  evparam.payload_size = s_reply_int.payload_size;
  evparam.command_id   = s_reply_int.command_id;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
  evparam.sequence_no  = s_reply_int.sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

  SpritzerCommandEvent_proc(&evparam);

  return;
}

