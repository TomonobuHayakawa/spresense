/********************************************************************
 *
 *  File Name: main.cpp
 *
 *  Description: Top Process on Apu
 *
 *  Notes: (C) Copyright 2014 Sony Corporation
 *
 *  Author: shuji Biwa
 *
 ********************************************************************
 */

#include <errno.h>

#include <stdlib.h>
#include <string.h>

extern "C"
{
#include <asmp/types.h>
#include <asmp/mpshm.h>
#include <asmp/mpmutex.h>
#include <asmp/mpmq.h>

#include "asmp.h"
}

#include "dsp_audio_version.h"
#include "postfilter_ctrl.h"

#define KEY_MQ 2
#define KEY_SHM   1
#define COMMAND_DATATYPE_ADDRESS 0
#define COMMAND_DATATYPE_VALUE 1 

#define ASSERT(cond) if (!(cond)) wk_abort()

static PostFilterCtrl s_ins;
static mpmq_t s_mq;

extern "C" {

/*--------------------------------------------------------------------*/
static void reply_to_spu(void *addr)
{
  /* Reply to SPU */

  uint8_t msg_id = 0;
  uint32_t msg_data = 0;

  /* Create message ID */

  msg_id |= (Wien2::Apu::FilterMode << 4);
  msg_id |= (Wien2::Apu::ExecEvent  << 1);
  msg_id |= COMMAND_DATATYPE_ADDRESS;

  /* Message data is address of APU command */

  msg_data = reinterpret_cast<uint32_t>(addr);

  /* Send */

  int ret = mpmq_send(&s_mq, msg_id, msg_data);

  if (ret != 0)
    {
      /* error */
    }
}

/*--------------------------------------------------------------------*/
};

int main()
{
  
  int ret = 0;

  /* Initialize MP message queue,
   * On the worker side, 3rd argument is ignored.
   */

  ret = mpmq_init(&s_mq, KEY_MQ, 0);
  ASSERT(ret == 0);

  /* Reply "boot complete"
   * MsgID is taken as part of message parameters.
   */

  uint8_t msg_id = 0;

  msg_id |= (Wien2::Apu::CommonMode << 4);
  msg_id |= (Wien2::Apu::BootEvent  << 1);
  msg_id |= COMMAND_DATATYPE_VALUE;

  ret = mpmq_send(&s_mq, msg_id, DSP_POSTFLTR_VERSION);

  /* Excution loop */

  while (true)
    {
      /* Receive message from SPU
       * Process is blocked until receive a message
       * (mpmq_receive() is polling message internally)
       */

      int command = 0;
      uint32_t msgdata = 0;

      command = mpmq_receive(&s_mq, &msgdata);

      //uint8_t process_mode = (command >> 4) & 0xf;
      //uint8_t event_type   = (command >> 1) & 0x7;
      uint8_t type         = (command >> 0) & 0x1;

      /* Parse and execute message */

      if (type == COMMAND_DATATYPE_ADDRESS)
        {
          s_ins.parse(reinterpret_cast<Wien2::Apu::Wien2ApuCmd *>(msgdata));
        }
      else
        {
          /* do nothing */
        }

      /* Send Reply */

      reply_to_spu(reinterpret_cast<void *>(msgdata));
    }
}

