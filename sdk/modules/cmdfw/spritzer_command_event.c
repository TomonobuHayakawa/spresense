/****************************************************************************
 * modules/cmdfw/spritzer_command_event.c
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include "fwuputils/fwup_client.h"
#include "spritzer_command_event.h"
#include "spritzer_command_creator.h"

#include <errno.h>
#include <sys/stat.h>
#include <sys/utsname.h>
#include <debug.h>
#include <nuttx/board.h>

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

static sensor_callback s_sensor_cb;
static act_recog_callback s_act_recog_cb;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static bool  illegal_event(event_param* param);
static bool  readfile_proc(bool* is_eof);

static err_t reply_receive(event_param* param);
static err_t get_firmware_version(event_param* param);
#ifdef CONFIG_BOARDCTL_RESET
static err_t reset_system(event_param* param);
#endif /* CONFIG_BOARDCTL_RESET */
static err_t register_event(event_param* param);
static err_t unregister_event(event_param* param);
#ifdef CONFIG_BOARDCTL_POWEROFF
static err_t power_off(event_param* param);
#endif /* CONFIG_BOARDCTL_POWEROFF */
static err_t fwupdate_event(event_param* param);
static err_t readfile_event(event_param* param);
static err_t writefile_event(event_param* param);
static err_t enable_sensors(event_param* param);
static err_t disable_sensors(event_param* param);
static err_t enable_activity_recognition(event_param* param);
static err_t disable_activity_recognition(event_param* param);

static event_element reply_table[] =
{
  { COMMAND_ID_GET_FIRMWARE_VERSION, reply_receive },
#ifdef CONFIG_BOARDCTL_RESET
  { COMMAND_ID_RESET_SYSTEM,         reply_receive },
#endif /* CONFIG_BOARDCTL_RESET */
  { COMMAND_ID_RESISTER_EVENT,       reply_receive },
  { COMMAND_ID_UNRESISTER_EVENT,     reply_receive },
#ifdef CONFIG_BOARDCTL_POWEROFF
  { COMMAND_ID_POWER_OFF,            reply_receive },
#endif /* CONFIG_BOARDCTL_POWEROFF */
  { COMMAND_ID_FIRMUP_EVENT,         reply_receive },
  { COMMAND_ID_READFILE_EVENT,       reply_receive },
  { COMMAND_ID_WRITEFILE_EVENT,      reply_receive },
};

static event_element event_table[] =
{
  { COMMAND_ID_GET_FIRMWARE_VERSION,   get_firmware_version },
#ifdef CONFIG_BOARDCTL_RESET
  { COMMAND_ID_RESET_SYSTEM,           reset_system         },
#endif /* CONFIG_BOARDCTL_RESET */
  { COMMAND_ID_RESISTER_EVENT,         register_event       },
  { COMMAND_ID_UNRESISTER_EVENT,       unregister_event     },
#ifdef CONFIG_BOARDCTL_POWEROFF
  { COMMAND_ID_POWER_OFF,              power_off            },
#endif /* CONFIG_BOARDCTL_POWEROFF */
  { COMMAND_ID_FIRMUP_EVENT,           fwupdate_event       },
  { COMMAND_ID_READFILE_EVENT,         readfile_event       },
  { COMMAND_ID_WRITEFILE_EVENT,        writefile_event      },

  { COMMAND_ID_ENABLE_SENSORS,         enable_sensors       },
  { COMMAND_ID_DISABLE_SENSORS,        disable_sensors      },
  { COMMAND_ID_ENABLE_ACT_RECOGNTION,  enable_activity_recognition  },
  { COMMAND_ID_DISABLE_ACT_RECOGNTION, disable_activity_recognition },
};

static spontaneous_event_table event_bit_table[] =
{
  { COMMAND_ID_LOWBATTERY_ALERT, SPCOMMAND_BIT_BATTERY_ALERT },
};

static uint32_t s_registered_event = 0;
static uint32_t s_enabled_sensors = 0;
static uint32_t s_enabled_act_recognizer = 0;
static int32_t s_readfile_remain_size = 0;

/****************************************************************************
 * Public Functions
 ****************************************************************************/
bool SpritzerCommandEvent_activate(void)
{
  return true;
}

/*--------------------------------------------------------------------*/
bool SpritzerCommandEvent_deactivate(void)
{
  s_registered_event = 0;
  s_readfile_remain_size = 0;

  return true;
}

/*--------------------------------------------------------------------*/
SpCmdResult SpritzerCommandEvent_proc(event_param* evparam)
{
  event_element* table_ptr;
  uint32_t       table_length;
  err_t          ret_status;
  uint32_t       i;

  if (evparam->is_reply)
    {
      /* reply events */

      cmdfwvdbg("Reply Event %x\n", evparam->command_id);

      table_ptr    = reply_table;
      table_length = sizeof(reply_table) / sizeof(event_element);
    }
  else
    {
      if (evparam->command_id & COMMAND_ID_USER_EVENTS)
        {
          /* user defined events */

          cmdfwvdbg("User Event %x\n", evparam->command_id);

          table_ptr    = NULL;
          table_length = 0;
        }
      else
        {
          /* system events */

          cmdfwvdbg("System Event %x\n", evparam->command_id);

          table_ptr    = event_table;
          table_length = sizeof(event_table) / sizeof(event_element);
        }
    }

  /* scan event function table */

  for (i = 0; i < table_length; i++)
    {
      if (table_ptr->id == evparam->command_id)
        {
          ret_status = table_ptr->event(evparam);
          cmdfwvdbg("event ret [%d]\n", ret_status);
          return (ret_status == 0) ? NoErr : EventProcErr;
        }

      table_ptr++;
    }

  /* if not found on table, it is illegal command */

  illegal_event(evparam);

  return EventNotSupportErr;
}

/*--------------------------------------------------------------------*/
bool SpritzerCommandEvent_isEventRegistered(uint16_t command_id)
{
  uint32_t table_length = 0;
  bool rtcd = false;
  uint32_t i = 0;

  table_length = sizeof(event_bit_table) / sizeof(event_bit_table);

  /* scan event bit table */

  for (i = 0; i < table_length; i++)
    {
      if (command_id == event_bit_table[i].id)
        {
          rtcd = ((s_registered_event & (1 << event_bit_table[i].bit)) != 0 ) ? true : false;
          break;
        }
    }

  return rtcd;
}

/*--------------------------------------------------------------------*/
bool SpritzerCommandEvent_nextPacket(void)
{
  bool is_next = true;

  if (s_readfile_remain_size != 0)
    {
      /* read file remains */

      bool is_eof = false;
      readfile_proc(&is_eof);
    }
  else if (0)
    {
      /* emergency info packet (not implemented yet) */
    }
  else
    {
      /* there is not next packet */

      is_next = false;
    }

  cmdfwvdbg("Next packet ? [%d]\n", is_next);

  return is_next;
}

/*--------------------------------------------------------------------*/
bool SpritzerCommandEvent_registerSensorCb(sensor_callback func)
{
  bool rtcd = true;

  s_sensor_cb = func;

  return rtcd;
}

/*--------------------------------------------------------------------*/
bool SpritzerCommandEvent_registerActRecogCb(act_recog_callback func)
{
  bool rtcd = true;

  s_act_recog_cb = func;

  return rtcd;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/
bool illegal_event(event_param* param)
{
  spcommand_create_param cparam;

#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
  cmdfwvdbg("rcv : illegal command [id:%04x][seq_no:%d][size:%d]\n",
            param->command_id, param->sequence_no, param->payload_size);
#else
  cmdfwvdbg("rcv : illegal command [id:%04x][size:%d]\n",
            param->command_id, param->payload_size);
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

  cparam.req_reply    = false;
  cparam.is_reply     = true;
  cparam.is_error     = true;
  cparam.payload_addr = NULL;
  cparam.payload_size = 0;
  cparam.command_id   = param->command_id;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
  cparam.sequence_no  = param->sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

  return SpritzerCommandCreator_create(&cparam);
}

/*--------------------------------------------------------------------*/
err_t reply_receive(event_param* param)
{
  err_t ercd = 0;

  cmdfwvdbg("reply rcv: id %d, seq No %d\n", param->command_id, param->sequence_no);

  return ercd;
}

/*--------------------------------------------------------------------*/
/* Get Firmware Version                                               */
/*--------------------------------------------------------------------*/
err_t get_firmware_version(event_param* param)
{
  err_t ercd = 0;
  uint8_t fw_version[] = "2.1.1.2 test";
  spcommand_create_param cparam;

  cmdfwvdbg("rcv : get fw version\n");

  if (param->req_reply)
    {
      cparam.req_reply    = false;
      cparam.is_reply     = true;
      cparam.is_error     = false;
      cparam.payload_addr = fw_version;
      cparam.payload_size = sizeof(fw_version);
      cparam.command_id   = COMMAND_ID_GET_FIRMWARE_VERSION;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
      cparam.sequence_no  = param->sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

      ercd = (SpritzerCommandCreator_create(&cparam) == true) ? 0 : 1;

      cmdfwvdbg("Create Packet! %s\n", fw_version);
    }

  return ercd;
}

#ifdef CONFIG_BOARDCTL_RESET
/*--------------------------------------------------------------------*/
/* Reset System Event                                                 */
/*--------------------------------------------------------------------*/
err_t reset_system(event_param* param)
{
  err_t ercd = 0;

  cmdfwvdbg("rcv : reset system\n");

  board_reset(0);

  return ercd;
}
#endif /* CONFIG_BOARDCTL_RESET */

/*--------------------------------------------------------------------*/
/* Register Event                                                     */
/*--------------------------------------------------------------------*/
err_t register_event(event_param* param)
{
  err_t ercd = 0;
  spcommand_create_param cparam;

  cmdfwvdbg("rcv : register event\n");

  /* save registered event */

  s_registered_event |= (uint32_t)((*(param->payload_addr++)      ) & 0x000000ff);
  s_registered_event |= (uint32_t)((*(param->payload_addr++) <<  8) & 0x0000ff00);
  s_registered_event |= (uint32_t)((*(param->payload_addr++) << 16) & 0x00ff0000);
  s_registered_event |= (uint32_t)((*(param->payload_addr)   << 24) & 0xff000000);

  cmdfwvdbg("regsitered event [%08x]\n", s_registered_event);

  /* create reply packet */

  if (param->req_reply)
    {
      cparam.req_reply    = false;
      cparam.is_reply     = true;
      cparam.is_error     = false;
      cparam.payload_addr = NULL;
      cparam.payload_size = 0;
      cparam.command_id   = COMMAND_ID_RESISTER_EVENT;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
      cparam.sequence_no  = param->sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

      ercd = (SpritzerCommandCreator_create(&cparam) == true) ? 0 : 1;
    }

  return ercd;
}

/*--------------------------------------------------------------------*/
/* Unregister Event                                                   */
/*--------------------------------------------------------------------*/
err_t unregister_event(event_param* param)
{
  err_t ercd = 0;
  spcommand_create_param cparam;
  uint32_t unregister_event_req = 0;

  cmdfwvdbg("rcv : unregister event\n");

  /* save unregistered event */

  unregister_event_req |= (uint32_t)((*(param->payload_addr++)      ) & 0x000000ff);
  unregister_event_req |= (uint32_t)((*(param->payload_addr++) <<  8) & 0x0000ff00);
  unregister_event_req |= (uint32_t)((*(param->payload_addr++) << 16) & 0x00ff0000);
  unregister_event_req |= (uint32_t)((*(param->payload_addr)   << 24) & 0xff000000);

  s_registered_event &= ~unregister_event_req;
  cmdfwvdbg("regsitered event [%08x]\n", s_registered_event);

  /* create reply packet */

  if (param->req_reply)
    {
      cparam.req_reply    = false;
      cparam.is_reply     = true;
      cparam.is_error     = false;
      cparam.payload_addr = NULL;
      cparam.payload_size = 0;
      cparam.command_id   = COMMAND_ID_UNRESISTER_EVENT;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
      cparam.sequence_no  = param->sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

      ercd = (SpritzerCommandCreator_create(&cparam) == true) ? 0 : 1;
    }

  return ercd;
}

#ifdef CONFIG_BOARDCTL_POWEROFF
/*--------------------------------------------------------------------*/
/* Power Off Event                                                    */
/*--------------------------------------------------------------------*/
err_t power_off(event_param* param)
{
  err_t ercd = 0;

  cmdfwvdbg("rcv : power off\n");

  board_power_off(0);

  return ercd;
}
#endif /* CONFIG_BOARDCTL_POWEROFF */

/*--------------------------------------------------------------------*/
/* FW update Event                                                    */
/*--------------------------------------------------------------------*/
#ifndef MIN
#  define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#if defined(CONFIG_DEBUG_VERBOSE) && defined(CONFIG_DEBUG_COMMANDFW)
static const char *get_file_type_string(enum fw_type_e fwtype)
{
  switch (fwtype)
    {
    case FW_APP: return "FW_APP";
    case FW_SYS: return "FW_SYS";
    case FW_UPDATER: return "FW_UPDATER";
    case FW_SBL: return "FW_SBL";
    default: return "";
    }
}
#endif

/*--------------------------------------------------------------------*/
static int do_partial_download(struct fwup_client_s *fwup, FILE *fp,
                               enum fw_type_e fwtype, uint32_t fwsize)
{
  int ret = OK;
  uint32_t *buf = NULL;
  uint32_t size;
  uint32_t remain = fwsize;

  do
    {
      size = MIN(remain, CONFIG_SPCOMMAND_FWUPDATE_WORK_AREA_SIZE);
      buf = (uint32_t*)malloc(size); /* buf must be 4byte alignment */
      if (buf == NULL)
        {
          ret = -ENOMEM;
          break;
        }

      ret = fread(buf, 1, size, fp);
      if (size != ret)
        {
          /* sanity check */

          free(buf);
          ret = -ENODATA;
          break;
        }

      ret = fwup->download(fwtype, fwsize, buf, size);

      remain -= size;

      cmdfwvdbg("->dl(0x%08x, %d / %d): ret=%d\n",
             (uint32_t)buf, fwsize - remain, fwsize, ret);

      fwup->msgsync();
      free(buf);

    }
  while (0 < remain);

  return ret;
}

/*--------------------------------------------------------------------*/
static int do_package_download(struct fwup_client_s *fwup, char* pathname)
{
  struct header_s
  {
    enum fw_type_e fwtype;
    uint32_t       fwsize;
  };

  int ret = OK;
  enum fw_type_e fwtype;
  uint32_t       fwsize;
  FILE *fp;
  int fwnum = 0;

  struct header_s header;

  fp = fopen(pathname, "rb");
  if (fp == NULL)
    {
      return -ENOENT;
    }

  while (1)
    {
      /* get package header */

      ret = fread(&header, 1, sizeof(struct header_s), fp);
      if (sizeof(struct header_s) != ret)
        {
          break; /* end of byte stream */
        }

      fwtype = header.fwtype;
      fwsize = header.fwsize;

      /* debug information */

      cmdfwvdbg("File: %s(%d)\n", pathname, fwnum++);
      cmdfwvdbg("Size: %d\n", fwsize);
      cmdfwvdbg("Type: %s\n", get_file_type_string(fwtype));
      if (fwsize <= 0)
        {
          ret = -ENOENT;
          break;
        }

      ret = do_partial_download(fwup, fp, fwtype, fwsize);
      if (ret)
        {
          break;
        }
    }

  fclose(fp);

  (void)fwnum;

  return ret;
}

/*--------------------------------------------------------------------*/
err_t fwupdate_event(event_param* param)
{
  err_t ercd = 0;
  char path[64] = { 0 };

  cmdfwvdbg("rcv : fw update event\n");

  /* Setup for FW Update */

  struct fwup_client_s *fwup = fwup_client_setup();

  /* FW Update Sequence Initialization */

  fwup->init();

  /* FW Download into SPI-Flash */

  snprintf(path, sizeof(path), "%s/sp_package.bin", CONFIG_SPCOMMAND_FWUPDATE_PACKAGE_DIR);
  ercd = do_package_download(fwup, path);
  if (ercd < 0)
    {
      spcommand_create_param cparam = { 0 };

      cparam.req_reply    = false;
      cparam.is_reply     = true;
      cparam.is_error     = true;
      cparam.payload_addr = NULL;
      cparam.payload_size = 0;
      cparam.command_id   = COMMAND_ID_FIRMUP_EVENT;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
      cparam.sequence_no  = param->sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

      ercd = (SpritzerCommandCreator_create(&cparam) == true) ? 0 : 1;

      return ercd;
    }

  /* FW Update Sequence Start after reboot */

  ercd = fwup->update();
  cmdfwvdbg("->update: ercd=%d\n", ercd);

  return ercd;

}

/*--------------------------------------------------------------------*/
/* Read File Event                                                    */
/*--------------------------------------------------------------------*/
#define READFILE_NAME_MAX 32
#define READFILE_SIZE_MAX 2016

typedef struct
{
  uint8_t name[READFILE_NAME_MAX];
  uint8_t data[READFILE_SIZE_MAX];
} ReadFileData;

static FILE* s_readfp;
static ReadFileData s_read_file;

/*--------------------------------------------------------------------*/
static bool readfile_proc(bool* is_eof)
{
  uint32_t               read_size = 0;
  spcommand_create_param cparam = { 0 };
  bool                   rtcd = false;

  /* read file */

  memset(s_read_file.data, 0, sizeof(s_read_file.data));
  read_size = fread(s_read_file.data, 1, READFILE_SIZE_MAX, s_readfp);
  s_readfile_remain_size -= read_size;

  cmdfwvdbg("read file name = %s, fp = %x, read size = %d, remain = %d\n",
            s_read_file.name, s_readfp, read_size, s_readfile_remain_size);

  if ((read_size < READFILE_SIZE_MAX) ||
      (s_readfile_remain_size <= 0))
    {
      *is_eof = true;

      /* when read to eof, close file */

      fclose(s_readfp);
      s_readfp = NULL;

      s_readfile_remain_size = 0;
    }
  else
    {
      *is_eof = false;
    }

  cparam.req_reply    = false;
  cparam.is_reply     = true;
  cparam.is_error     = (read_size < 0) ? true : false;
  cparam.payload_addr = (uint8_t*)&s_read_file;
  cparam.payload_size = READFILE_NAME_MAX + read_size;
  cparam.command_id   = COMMAND_ID_READFILE_EVENT;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
  cparam.sequence_no   = 0xffff;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

  rtcd = SpritzerCommandCreator_create(&cparam);

  return rtcd;
}

/*--------------------------------------------------------------------*/
err_t readfile_event(event_param* param)
{
  typedef struct
  {
    uint8_t  name[READFILE_NAME_MAX];
    uint32_t size;
  } FileInfo;

  err_t                                ercd = 0;
  FileInfo __attribute__((aligned(4))) file_info;
  struct stat                          file_stat = { 0 };
  spcommand_create_param               cparam;

  char path[64] = { 0 };
  int  ferr = 0;

  cmdfwvdbg("rcv : readfile event\n");

  /* open file */

  snprintf(path, sizeof(path), "%s/%s", CONFIG_SPCOMMAND_FWUPDATE_PACKAGE_DIR, param->payload_addr);
  s_readfp = fopen(path, "r");
  if (s_readfp == NULL)
    {
      cmdfwdbg("open error %s\n", path);
      ferr = get_errno();
    }

  /* get file size */

  if (stat(path, &file_stat) != 0)
    {
      fclose(s_readfp);
      ferr = get_errno();
    }

  strncpy((char*)file_info.name, (char*)param->payload_addr, sizeof(file_info.name));
  file_info.size = file_stat.st_size;
  cmdfwvdbg("read file : %s [size %d]\n", file_info.name, file_info.size);

  /* enter iteration work */

  s_readfile_remain_size = file_stat.st_size;

  memset(s_read_file.name, 0, sizeof(s_read_file.name));
  strncpy((char*)s_read_file.name, (char*)file_info.name, sizeof(s_read_file.name));

  /* reply parameters are "file name" and "file size". */

  if (param->req_reply)
    {
      cparam.req_reply    = false;
      cparam.is_reply     = true;
      cparam.is_error     = (ferr != 0) ? true : false;
      cparam.payload_addr = (uint8_t*)&file_info;
      cparam.payload_size = sizeof(file_info);
      cparam.command_id   = COMMAND_ID_READFILE_EVENT;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
      cparam.sequence_no  = param->sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

      ercd = (SpritzerCommandCreator_create(&cparam) == true) ? 0 : 1;
    }

  return ercd;
}

/*--------------------------------------------------------------------*/
/* Write File Event                                                   */
/*--------------------------------------------------------------------*/
err_t writefile_event(event_param* param)
{
  typedef struct
  {
    uint8_t attribute;
    uint8_t name[31]; // Premise : NULL code inside
  } FileInfo;

  err_t ercd = 0;
  FileInfo* file_info = (FileInfo*)param->payload_addr;
  uint8_t* data = param->payload_addr + sizeof(FileInfo);
  uint32_t data_size = param->payload_size - sizeof(FileInfo);
  spcommand_create_param cparam;

  char* mode;
  static FILE* fd;
  char path[64] = { 0 };

  int ferr = 0;
  static int total_size = 0;

  if (file_info->attribute == 0)
    {
      mode = "wb";
      total_size = 0;
    }
  else
    {
      mode = "ab";
    }

  cmdfwvdbg("rcv : writefile event mode [%s]\n", mode);

  snprintf(path, sizeof(path), "%s/%s", CONFIG_SPCOMMAND_FWUPDATE_PACKAGE_DIR, file_info->name);
  fd = fopen(path, mode);

  if ((int)fd != 0)
    {
      fwrite(data, 1,data_size,fd);
    }
  else
    {
      cmdfwdbg("open error! %x\n",fd);
      ferr  = get_errno();
      cmdfwdbg(" error no! %x\n",ferr);
    }

  if (fclose(fd) != 0)
    {
      cmdfwdbg("close error\n");
    }

  total_size = total_size + data_size;
  cmdfwvdbg("total size=%d\n", total_size);

  if (param->req_reply)
    {
      cparam.req_reply    = false;
      cparam.is_reply     = true;
      cparam.is_error     = (ferr != 0) ? true : false;
      cparam.payload_addr = (uint8_t*)&data_size;
      cparam.payload_size = 4;
      cparam.command_id   = COMMAND_ID_WRITEFILE_EVENT;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
      cparam.sequence_no  = param->sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

      ercd = (SpritzerCommandCreator_create(&cparam) == true) ? 0 : 1;
    }

  (void)ferr;

  return ercd;
}

/*--------------------------------------------------------------------*/
/* Enable Sensors Event                                               */
/*--------------------------------------------------------------------*/
err_t enable_sensors(event_param* param)
{
  err_t ercd = 0;
  uint8_t id;
  uint32_t enable_sensors_bit = 0;
  spcommand_create_param cparam;

  cmdfwvdbg("rcv : enable sensors event\n");

  /* save enabled sensors */

  enable_sensors_bit |= (uint32_t)((*(param->payload_addr++)      ) & 0x000000ff);
  enable_sensors_bit |= (uint32_t)((*(param->payload_addr++) <<  8) & 0x0000ff00);
  enable_sensors_bit |= (uint32_t)((*(param->payload_addr++) << 16) & 0x00ff0000);
  enable_sensors_bit |= (uint32_t)((*(param->payload_addr)   << 24) & 0xff000000);

  for (id = 0; id < 32/* bit length */; id++)
    {
      if ((id == SPCOMMAND_SENSOR_ACCELE) ||
          (id == SPCOMMAND_SENSOR_PRESSURE) ||
          (id == SPCOMMAND_SENSOR_TEMPERTURE) ||
          (id == SPCOMMAND_SENSOR_MAGNE) ||
          (id == SPCOMMAND_SENSOR_GNSS) ||
          (id == SPCOMMAND_SENSOR_BAROMETER) ||
          (id == SPCOMMAND_SENSOR_COMPASS))
        {
          if (enable_sensors_bit & (0x01 << id))
            {
              s_sensor_cb(id, true);
            }
        }
    }

  /* save sensor state */

  s_enabled_sensors |= enable_sensors_bit;

  /* create reply packet */

  cparam.req_reply    = true;
  cparam.is_reply     = true;
  cparam.is_error     = false;
  cparam.payload_addr = (uint8_t*)&s_enabled_sensors;
  cparam.payload_size = sizeof(s_enabled_sensors);
  cparam.command_id   = COMMAND_ID_ENABLE_SENSORS;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
  cparam.sequence_no  = param->sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

  ercd = (SpritzerCommandCreator_create(&cparam) == true) ? 0 : 1;

  return ercd;
}

/*--------------------------------------------------------------------*/
/* Disable Sensors Event                                               */
/*--------------------------------------------------------------------*/
err_t disable_sensors(event_param* param)
{
  err_t ercd = 0;
  uint8_t id;
  uint32_t disable_sensors_bit = 0;
  spcommand_create_param cparam;

  cmdfwvdbg("rcv : disable sensors event\n");

  /* save disabled sensors */

  disable_sensors_bit |= (uint32_t)((*(param->payload_addr++)      ) & 0x000000ff);
  disable_sensors_bit |= (uint32_t)((*(param->payload_addr++) <<  8) & 0x0000ff00);
  disable_sensors_bit |= (uint32_t)((*(param->payload_addr++) << 16) & 0x00ff0000);
  disable_sensors_bit |= (uint32_t)((*(param->payload_addr)   << 24) & 0xff000000);

  for (id = 0; id < 32/* bit length */; id++)
    {
      if ((id == SPCOMMAND_SENSOR_ACCELE) ||
          (id == SPCOMMAND_SENSOR_PRESSURE) ||
          (id == SPCOMMAND_SENSOR_TEMPERTURE) ||
          (id == SPCOMMAND_SENSOR_MAGNE) ||
          (id == SPCOMMAND_SENSOR_GNSS) ||
          (id == SPCOMMAND_SENSOR_BAROMETER) ||
          (id == SPCOMMAND_SENSOR_COMPASS))
        {
          if (disable_sensors_bit & (0x01 << id))
            {
              s_sensor_cb(id, false);
            }
        }
    }

  /* save sensor state */

  s_enabled_sensors &= ~disable_sensors_bit;

  /* create reply packet */

  cparam.req_reply    = true;
  cparam.is_reply     = true;
  cparam.is_error     = false;
  cparam.payload_addr = (uint8_t*)&s_enabled_sensors;
  cparam.payload_size = sizeof(s_enabled_sensors);
  cparam.command_id   = COMMAND_ID_DISABLE_SENSORS;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
  cparam.sequence_no  = param->sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

  ercd = (SpritzerCommandCreator_create(&cparam) == true) ? 0 : 1;

  return ercd;
}

/*--------------------------------------------------------------------*/
/* Enable Activity Recognition Event                                  */
/*--------------------------------------------------------------------*/
err_t enable_activity_recognition(event_param* param)
{
  err_t ercd = 0;
  uint32_t enable_activity_recognizer = 0;

  spcommand_create_param cparam;

  cmdfwvdbg("rcv : enable activity recognition event\n");

  /* save disabled sensors */

  enable_activity_recognizer |= (uint32_t)((*(param->payload_addr++)      ) & 0x000000ff);
  enable_activity_recognizer |= (uint32_t)((*(param->payload_addr++) <<  8) & 0x0000ff00);
  enable_activity_recognizer |= (uint32_t)((*(param->payload_addr++) << 16) & 0x00ff0000);
  enable_activity_recognizer |= (uint32_t)((*(param->payload_addr)   << 24) & 0xff000000);

  if (enable_activity_recognizer & 0x000000ff)
    {
      /* gesture */
      s_act_recog_cb(SPCOMMAND_ACTRECOG_GESTURE, true);
    }

  if (enable_activity_recognizer & 0x00ff0000)
    {
      /* step counter */
      s_act_recog_cb(SPCOMMAND_ACTRECOG_STEPCOUNTER, true);
    }

  if (enable_activity_recognizer & 0xff000000)
    {
      /* tram */
      s_act_recog_cb(SPCOMMAND_ACTRECOG_TRAM, true);
    }

  /* save enabled recognizer */

  s_enabled_act_recognizer |= enable_activity_recognizer;

  /* create reply packet */

  cparam.req_reply    = true;
  cparam.is_reply     = true;
  cparam.is_error     = false;
  cparam.payload_addr = (uint8_t*)&s_enabled_act_recognizer;
  cparam.payload_size = sizeof(s_enabled_act_recognizer);
  cparam.command_id   = COMMAND_ID_ENABLE_ACT_RECOGNTION;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
  cparam.sequence_no  = param->sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

  ercd = (SpritzerCommandCreator_create(&cparam) == true) ? 0 : 1;

  return ercd;
}

/*--------------------------------------------------------------------*/
/* Disable Activity Recognition Event                                 */
/*--------------------------------------------------------------------*/
err_t disable_activity_recognition(event_param* param)
{
  err_t ercd = 0;
  uint32_t disable_activity_recognizer = 0;

  spcommand_create_param cparam;

  cmdfwvdbg("rcv : disable activity recognition event\n");

  /* save disabled sensors */

  disable_activity_recognizer |= (uint32_t)((*(param->payload_addr++)      ) & 0x000000ff);
  disable_activity_recognizer |= (uint32_t)((*(param->payload_addr++) <<  8) & 0x0000ff00);
  disable_activity_recognizer |= (uint32_t)((*(param->payload_addr++) << 16) & 0x00ff0000);
  disable_activity_recognizer |= (uint32_t)((*(param->payload_addr)   << 24) & 0xff000000);

  if (disable_activity_recognizer & 0x000000ff)
    {
      /* gesture */
      s_act_recog_cb(SPCOMMAND_ACTRECOG_GESTURE, false);
    }

  if (disable_activity_recognizer & 0x00ff0000)
    {
      /* step counter */
      s_act_recog_cb(SPCOMMAND_ACTRECOG_STEPCOUNTER, false);
    }

  if (disable_activity_recognizer & 0xff000000)
    {
      /* tram */
      s_act_recog_cb(SPCOMMAND_ACTRECOG_TRAM, false);
    }

  /* save enabled recognizer */

  s_enabled_act_recognizer &= ~disable_activity_recognizer;

  /* create reply packet */

  cparam.req_reply    = true;
  cparam.is_reply     = true;
  cparam.is_error     = false;
  cparam.payload_addr = (uint8_t*)&s_enabled_act_recognizer;
  cparam.payload_size = sizeof(s_enabled_act_recognizer);
  cparam.command_id   = COMMAND_ID_DISABLE_ACT_RECOGNTION;
#ifdef CONFIG_SPCOMMAND_ENABLE_SEQNO
  cparam.sequence_no  = param->sequence_no;
#endif /* CONFIG_SPCOMMAND_ENABLE_SEQNO */

  ercd = (SpritzerCommandCreator_create(&cparam) == true) ? 0 : 1;

  return ercd;
}

