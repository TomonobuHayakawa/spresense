/****************************************************************************
 * examples/volte/volte_audio.cxx
 *
 *   Copyright (C) 2017 Sony Corporation.
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
#include <sys/types.h>
#include <sched.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <asmp/mpshm.h>

#include "memutils/os_utils/chateau_osal.h"
#include "audio/audio_high_level_api.h"
#include "memutils/simple_fifo/CMN_SimpleFifo.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"
#include "msgq_id.h"
#include "mem_layout.h"
#include "memory_layout.h"
#include "msgq_pool.h"
#include "pool_layout.h"
#include "fixed_fence.h"

#include <arch/chip/cxd56_audio.h>

#include "volte_audio.h"

using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* task setting */
#define AUDIO_TASK_PRIORITY           (150)
#define AUDIO_TASK_CTRL_PRIORITY      (100)
#define AUDIO_TASK_PLAYER_STACK_SIZE  (1024 * 3)
#define AUDIO_TASK_OUTMIX_STACK_SIZE  (1024 * 3)
#define AUDIO_TASK_CTRL_STACK_SIZE    (1024 * 3)

/* for Audio command execution/response reception */
static AudioCommand command;
static AudioResult result;


/* for simpleFifo */
#define WRITE_SIMPLE_FIFO_SIZE	(2560)
#define SIMPLE_FIFO_FRAME_NUM	(30)
#define SIMPLE_FIFO_BUF_SIZE	(WRITE_SIMPLE_FIFO_SIZE * SIMPLE_FIFO_FRAME_NUM)

static uint8_t appVolteAudio_simple_fifo_buf[SIMPLE_FIFO_BUF_SIZE];
static uint8_t appVolteAudio_ram_buf[WRITE_SIMPLE_FIFO_SIZE];
static uint8_t appVolteAudio_ringtoneData[SIMPLE_FIFO_BUF_SIZE];
static int     appVolteAudio_ringtoneSize;
static int     appVolteAudio_playedSize;

static CMN_SimpleFifoHandle appVolteAudio_simple_fifo_handle;
static AsPlayerInputDeviceHdlrForRAM appVolteAudioRamHandler;

static unsigned int appVolteAudio_user;

extern "C" {
int app_volte_audio_ctrl( char audio_ctrl )
{
  mqd_t mqd;

  printf( "audio_ctrl=%d\n", audio_ctrl );

  if( audio_ctrl == APP_VOLTE_AUDIO_CTRL_NONE )
    {
      return 0;
    }

  mqd = mq_open( "volte_audio_example", O_WRONLY, 0666, NULL);

  if( mq_send( mqd, &audio_ctrl, sizeof(audio_ctrl), 0 ) != 0 )
    {
      printf("mq_send to volte_audio_example queue\n");
    }

  mq_close(mqd);

  return 0;
}
}

/* static function */
static int appVolteAudioResultCheck( const char *function_name, uint8_t check_code )
{
  AS_ReceiveAudioResult(&result);
  if (result.header.result_code != check_code)
    {
      printf( "%s ERROR: Result code=0x%x\n", function_name, result.header.result_code);
      return 1;
    }

  return 0;
}

static int appVolteAudioSetVolume(void)
{
  command.header.packet_length = LENGTH_SETVOLUME;
  command.header.command_code  = AUDCMD_SETVOLUME;
  command.header.sub_code      = 0;

  command.set_volume_param.input1_db = -250;
  command.set_volume_param.input2_db = AS_VOLUME_HOLD;
  command.set_volume_param.master_db = AS_VOLUME_DAC;

  AS_SendAudioCommand(&command);

  return appVolteAudioResultCheck( "appVolteAudioSetVolume", AUDRLT_SETVOLUMECMPLT );
}

static int appVolteAudioPowerOn(void)
{
  command.header.packet_length = LENGTH_POWERON;
  command.header.command_code  = AUDCMD_POWERON;
  command.header.sub_code      = 0x00;

  command.power_on_param.enable_sound_effect = AS_DISABLE_SOUNDEFFECT;

  AS_SendAudioCommand(&command);

  return appVolteAudioResultCheck( "appVolteAudioPowerOn", AUDRLT_STATUSCHANGED );
}


static int appVolteAudioOutputSelect(void)
{
  command.header.packet_length = LENGTH_INITOUTPUTSELECT;
  command.header.command_code  = AUDCMD_INITOUTPUTSELECT;
  command.header.sub_code      = 0;

  command.init_output_select_param.output_device_sel = AS_OUT_SP;

  AS_SendAudioCommand(&command);

  return appVolteAudioResultCheck( "appVolteAudioOutputSelect", AUDRLT_INITOUTPUTSELECTCMPLT );
}

static int appVolteAudioPushSimpleFifo( void )
{
  int cnt;

  appVolteAudio_playedSize = 0;

  for( cnt = 0 ; cnt < appVolteAudio_ringtoneSize/WRITE_SIMPLE_FIFO_SIZE; cnt++ )
    {
      if (CMN_SimpleFifoOffer(&appVolteAudio_simple_fifo_handle, 
                              (const void*)&appVolteAudio_ringtoneData[cnt*WRITE_SIMPLE_FIFO_SIZE], 
                              WRITE_SIMPLE_FIFO_SIZE) == 0)
        {
          printf( "Simple FIFO is full!\n" );
          return 1;
        }
    }

  if (CMN_SimpleFifoOffer(&appVolteAudio_simple_fifo_handle,
                          (const void*)&appVolteAudio_ringtoneData[cnt*WRITE_SIMPLE_FIFO_SIZE],
                          appVolteAudio_ringtoneSize%WRITE_SIMPLE_FIFO_SIZE) == 0)
    {
      printf( "Simple FIFO is full!\n" );
      return 1;
    }

  return 0;
}


static void appVolteAudioCallback(uint32_t size)
{
  appVolteAudio_playedSize += size;
  if( appVolteAudio_playedSize >= appVolteAudio_ringtoneSize )
    {
      appVolteAudioPushSimpleFifo();
    }

  return;
}

static int appVolteAudioInitSimpleFifo(void)
{
  if (CMN_SimpleFifoInitialize(&appVolteAudio_simple_fifo_handle, appVolteAudio_simple_fifo_buf, SIMPLE_FIFO_BUF_SIZE, NULL) != 0)
    {
      printf( "Fail to initialize simple FIFO." );
      return 1;
    }

  CMN_SimpleFifoClear(&appVolteAudio_simple_fifo_handle);

  appVolteAudioRamHandler.simple_fifo_handler = (void*)(&appVolteAudio_simple_fifo_handle);
  appVolteAudioRamHandler.callback_function = appVolteAudioCallback;

  return 0;
}


static int appVolteAudioSetPlayerStatus(void)
{
  command.header.packet_length  = LENGTH_SET_PLAYER_STATUS;
  command.header.command_code   = AUDCMD_SETPLAYERSTATUS;
  command.header.sub_code       = 0;

  command.set_player_sts_param.input_device  = AS_SETPLAYER_INPUTDEVICE_RAM;
  command.set_player_sts_param.ram_handler   = &appVolteAudioRamHandler;
  command.set_player_sts_param.output_device = AS_SETPLAYER_OUTPUTDEVICE_SPHP;
  command.set_player_sts_param.output_device_handler = 0x00;

  AS_SendAudioCommand( &command );

  return appVolteAudioResultCheck( "appVolteAudioSetPlayerStatus", AUDRLT_STATUSCHANGED );
}



static int appVolteAudioInitPlayer(void)
{
  command.header.packet_length  = LENGTH_INIT_PLAYER;
  command.header.command_code   = AUDCMD_INITPLAYER;
  command.header.sub_code       = 0;

  command.init_player_param.codec_type     = AS_CODECTYPE_MP3;
  command.init_player_param.bit_length     = AS_BITLENGTH_16;
  command.init_player_param.channel_number = AS_CHANNEL_STEREO;
  command.init_player_param.sampling_rate  = AS_SAMPLINGRATE_16000;

  AS_SendAudioCommand( &command );

  return appVolteAudioResultCheck( "appVolteAudioInitPlayer", AUDRLT_INITPLAYERCMPLT );
}

static int appVolteAudioPlayPlayer(void)
{
  command.header.packet_length  = LENGTH_PLAY_PLAYER;
  command.header.command_code   = AUDCMD_PLAYPLAYER;
  command.header.sub_code       = 0;

  AS_SendAudioCommand( &command );

  return appVolteAudioResultCheck( "appVolteAudioPlayPlayer", AUDRLT_PLAYCMPLT );
}

static int appVolteAudioStopPlayer(void)
{
  command.header.packet_length = LENGTH_STOP_PLAYER;
  command.header.command_code  = AUDCMD_STOPPLAYER;
  command.header.sub_code      = 0;

  AS_SendAudioCommand( &command );

  return appVolteAudioResultCheck( "appVolteAudioStopPlayer", AUDRLT_STOPCMPLT );
}


static int appVolteAudioInitI2sParam(void)
{
  command.header.packet_length  = LENGTH_INITI2SPARAM;
  command.header.command_code   = AUDCMD_INITI2SPARAM;
  command.header.sub_code       = 0;

  command.init_i2s_param.i2s_id         = AS_I2S1;
  command.init_i2s_param.rate           = 48000;
  command.init_i2s_param.bypass_mode_en = AS_I2S_BYPASS_MODE_DISABLE;

  AS_SendAudioCommand( &command );

  return appVolteAudioResultCheck( "appVolteAudioInitI2sParam", AUDRLT_INITI2SPARAMCMPLT );
}

static int appVolteAudioInitMicGain(void)
{
  command.header.packet_length = LENGTH_INITMICGAIN;
  command.header.command_code  = AUDCMD_INITMICGAIN;
  command.header.sub_code      = 0;

  command.init_mic_gain_param.mic_gain[0] = 210;
  command.init_mic_gain_param.mic_gain[1] = 210;
  command.init_mic_gain_param.mic_gain[2] = 210;
  command.init_mic_gain_param.mic_gain[3] = 210;
  command.init_mic_gain_param.mic_gain[4] = AS_MICGAIN_HOLD;
  command.init_mic_gain_param.mic_gain[5] = AS_MICGAIN_HOLD;
  command.init_mic_gain_param.mic_gain[6] = AS_MICGAIN_HOLD;
  command.init_mic_gain_param.mic_gain[7] = AS_MICGAIN_HOLD;

  AS_SendAudioCommand( &command );

  return appVolteAudioResultCheck( "appVolteAudioInitMicGain", AUDRLT_INITMICGAINCMPLT );
}


static int appVolteAudioSetBasebandStatus(void)
{
  command.header.packet_length = LENGTH_SET_BASEBAND_STATUS;
  command.header.command_code  = AUDCMD_SETBASEBANDSTATUS;
  command.header.sub_code      = 0;

  command.set_baseband_status_param.with_MFE           = AS_SET_BBSTS_WITH_MFE_ACTIVE;
  command.set_baseband_status_param.with_Voice_Command = AS_SET_BBSTS_WITH_VCMD_NONE;
  command.set_baseband_status_param.with_MPP           = AS_SET_BBSTS_WITH_MPP_NONE;
  command.set_baseband_status_param.input_device       = AS_INPUT_DEVICE_AMIC1CH_I2S2CH;
  command.set_baseband_status_param.output_device      = AS_OUTPUT_DEVICE_SP2CH_I2S2CH;

  AS_SendAudioCommand( &command );

  return appVolteAudioResultCheck( "appVolteAudioSetBbActiveStatus", AUDRLT_STATUSCHANGED );
}

/* parameter for command_code=AUDCMD_INITMFE */
/* TODO: Stop keeping data locally */
static char mfe_coef_table[1024] __attribute__((aligned(4))) = {
	0x81, 0x02, 0x01, 0x00, 0x90, 0x01, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x40, 0x9A, 0x99, 0x19, 0x3F,
	0xCD, 0xCC, 0x4C, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0xCD, 0xCC, 0xCC, 0x3D, 0x9A, 0x99, 0x19, 0x3F,
	0x9A, 0x99, 0x99, 0x3F, 0x8F, 0xC2, 0xF5, 0x3D, 0x9A, 0x99, 0x19, 0x3E, 0x0A, 0xD7, 0x23, 0x3E,
	0xB8, 0x1E, 0x05, 0x3E, 0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D,
	0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D,
	0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D,
	0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D, 0x0A, 0xD7, 0x23, 0x3D,
	0x0A, 0xD7, 0x23, 0x3D, 0x9A, 0x99, 0x19, 0x3D, 0x29, 0x5C, 0x0F, 0x3D, 0xB8, 0x1E, 0x05, 0x3D,
	0x8F, 0xC2, 0xF5, 0x3C, 0xAE, 0x47, 0xE1, 0x3C, 0xCD, 0xCC, 0xCC, 0x3C, 0xEC, 0x51, 0xB8, 0x3C,
	0x0A, 0xD7, 0xA3, 0x3C, 0x0A, 0xD7, 0xA3, 0x3C, 0x0A, 0xD7, 0xA3, 0x3C, 0x0A, 0xD7, 0xA3, 0x3C,
	0x0A, 0xD7, 0xA3, 0x3C, 0x0A, 0xD7, 0xA3, 0x3C, 0x0A, 0xD7, 0xA3, 0x3C, 0x0A, 0xD7, 0xA3, 0x3C,
	0x0A, 0xD7, 0xA3, 0x3C, 0x0A, 0xD7, 0x23, 0x3D, 0x63, 0xEE, 0x2A, 0x3F, 0x63, 0xEE, 0x2A, 0x3F,
	0xB2, 0x9D, 0x87, 0x3F, 0x63, 0xEE, 0x2A, 0x3F, 0x63, 0xEE, 0x2A, 0x3F, 0xB2, 0x9D, 0x87, 0x3F,
	0x63, 0xEE, 0x2A, 0x3F, 0x63, 0xEE, 0x2A, 0x3F, 0xB2, 0x9D, 0x87, 0x3F, 0x4C, 0xA6, 0x1A, 0x3F,
	0x4C, 0xA6, 0x1A, 0x3F, 0xCC, 0x5D, 0x8B, 0x3F, 0xA7, 0x79, 0x17, 0x3F, 0xA7, 0x79, 0x17, 0x3F,
	0x7A, 0xC7, 0x89, 0x3F, 0xEE, 0x5A, 0x12, 0x3F, 0xEE, 0x5A, 0x12, 0x3F, 0xBD, 0xE3, 0x94, 0x3F,
	0x7C, 0x61, 0x02, 0x3F, 0x7C, 0x61, 0x02, 0x3F, 0xAC, 0x8B, 0xA3, 0x3F, 0x5A, 0xF5, 0x29, 0x3F,
	0x5A, 0xF5, 0x29, 0x3F, 0x86, 0x5A, 0xA3, 0x3F, 0xF5, 0xB9, 0x6A, 0x3F, 0xF5, 0xB9, 0x6A, 0x3F,
	0xC4, 0xB1, 0xBE, 0x3F, 0x2B, 0xF6, 0x9F, 0x3F, 0x2B, 0xF6, 0x9F, 0x3F, 0x31, 0x99, 0xF2, 0x3F,
	0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x90, 0x01, 0x00, 0x00,
	0xE8, 0x03, 0x00, 0x00, 0xE2, 0x04, 0x00, 0x00, 0xDC, 0x05, 0x00, 0x00, 0x08, 0x07, 0x00, 0x00,
	0xD0, 0x07, 0x00, 0x00, 0xC4, 0x09, 0x00, 0x00, 0xB8, 0x0B, 0x00, 0x00, 0xA0, 0x0F, 0x00, 0x00,
	0x40, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD8, 0x47, 0x27, 0xBC,
	0x5B, 0xEB, 0x0B, 0xBC, 0xCD, 0xCC, 0x4C, 0xBC, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x00, 0x00, 0x00,
	0x33, 0x33, 0x33, 0x3F, 0xCD, 0xCC, 0xCC, 0x3D, 0x00, 0x00, 0xA0, 0x41, 0x66, 0x66, 0x66, 0x3F,
	0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x41, 0x00, 0x00, 0x00, 0x3F, 0x58, 0x02, 0x00, 0x00,
	0xC8, 0x00, 0x00, 0x00, 0x90, 0x01, 0x00, 0x00, 0x90, 0x01, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00,
	0xE8, 0x03, 0x00, 0x00, 0x33, 0x33, 0x33, 0x3F, 0x9A, 0x99, 0x99, 0x3F, 0x0A, 0xD7, 0x23, 0x3C,
	0xCD, 0xCC, 0x4C, 0x3E, 0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x80, 0x3F, 0xA3, 0xE8, 0xA1, 0x3E,
	0xEC, 0x51, 0x38, 0x3E, 0x8F, 0xC2, 0xF5, 0x3D, 0x00, 0x00, 0x40, 0x3F, 0x33, 0x33, 0x33, 0x3F,
	0x9A, 0x99, 0x59, 0x3F, 0x9A, 0x99, 0x59, 0x3F, 0x00, 0x00, 0x80, 0x3E, 0x52, 0xB8, 0x7E, 0x3F,
	0x33, 0x33, 0x73, 0x3F, 0x52, 0xB8, 0x7E, 0x3F, 0x00, 0x00, 0x00, 0x3F, 0xB9, 0xFC, 0x7F, 0x3F,
	0x3B, 0xDF, 0x7F, 0x3F, 0x9A, 0x99, 0x99, 0x3F, 0x8F, 0xC2, 0x75, 0x3C, 0x6F, 0x12, 0x03, 0x3B,
	0x6F, 0x12, 0x83, 0x3A, 0xCD, 0xCC, 0x4C, 0x3E, 0x0A, 0xD7, 0x23, 0x3C, 0x0A, 0xD7, 0x23, 0x3C,
	0x66, 0x66, 0x66, 0x3F, 0x52, 0xB8, 0x7E, 0x3F, 0x77, 0xBE, 0x7F, 0x3F, 0x1C, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x30, 0x90, 0xA9, 0x42, 0x4B, 0xC0, 0x30, 0x43, 0x4E, 0x42, 0x8A, 0x43,
	0x23, 0x5F, 0xC0, 0x43, 0x70, 0x12, 0xFB, 0x43, 0xE3, 0x5F, 0x1D, 0x44, 0x95, 0xE9, 0x3F, 0x44,
	0xE4, 0x60, 0x65, 0x44, 0xAF, 0x02, 0x87, 0x44, 0xF9, 0x0D, 0x9D, 0x44, 0xB6, 0xF7, 0xB4, 0x44,
	0x76, 0xE8, 0xCE, 0x44, 0x3A, 0x0C, 0xEB, 0x44, 0x5E, 0xC9, 0x04, 0x45, 0xE3, 0x57, 0x15, 0x45,
	0xC1, 0x4D, 0x27, 0x45, 0x6F, 0xC9, 0x3A, 0x45, 0xFB, 0xEB, 0x4F, 0x45, 0x3D, 0xD9, 0x66, 0x45,
	0x1B, 0xB8, 0x7F, 0x45, 0x62, 0x59, 0x8D, 0x45, 0x7D, 0xFB, 0x9B, 0x45, 0x33, 0xDB, 0xAB, 0x45,
	0x6F, 0x13, 0xBD, 0x45, 0x69, 0xC1, 0xCF, 0x45, 0xCE, 0x04, 0xE4, 0x45, 0x00, 0x00, 0xFA, 0x45,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x05, 0x00, 0x00, 0x00, 0x77, 0xBE, 0x7F, 0x3F, 0x66, 0x66, 0x66, 0x3F, 0x00, 0x00, 0x00, 0x00,
	0x6F, 0x12, 0x83, 0x3A, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F,
	0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F,
	0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F,
	0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F,
	0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x6F, 0x12, 0x03, 0x3B, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


static int appVolteAudioInitMfe(void)
{
  command.header.packet_length = LENGTH_INITMFE;
  command.header.command_code  = AUDCMD_INITMFE;
  command.header.sub_code      = 0;

  command.init_mfe_param.input_fs           = AS_SAMPLINGRATE_16000;
  command.init_mfe_param.ref_channel_num    = AS_CHANNEL_STEREO;
  command.init_mfe_param.mic_channel_num    = AS_CHANNEL_MONO;
  command.init_mfe_param.enable_echocancel  = AS_ENABLE_ECHOCANCEL;
  command.init_mfe_param.include_echocancel = AS_INCLUDE_ECHOCANCEL;
  command.init_mfe_param.mfe_mode           = AS_MFE_MODE_SPEAKING;
  command.init_mfe_param.config_table       = (uint32_t)(mfe_coef_table);

  AS_SendAudioCommand( &command );

  return appVolteAudioResultCheck( "appVolteAudioInitMfe", AUDRLT_INITMFECMPLT );
}


static int appVolteAudioInitMpp(void)
{
  command.header.packet_length = LENGTH_INITMPP;
  command.header.command_code  = AUDCMD_INITMPP;
  command.header.sub_code      = 0;

  command.init_mpp_param.output_fs          = AS_SAMPLINGRATE_48000;
  command.init_mpp_param.output_channel_num = AS_CHANNEL_STEREO;
  command.init_mpp_param.mpp_mode           = AS_MPP_MODE_XLOUD_ONLY;
  command.init_mpp_param.xloud_mode         = AS_MPP_XLOUD_MODE_DISABLE;
  command.init_mpp_param.coef_mode          = AS_MPP_COEF_SPEAKER;
  command.init_mpp_param.eax_mode           = AS_MPP_EAX_DISABLE;
  command.init_mpp_param.xloud_coef_table   = 0;
  command.init_mpp_param.eax_coef_table     = 0;

  AS_SendAudioCommand( &command );

  return appVolteAudioResultCheck( "appVolteAudioInitMpp", AUDRLT_INITMPPCMPLT );
}

static int appVolteAudioStartBb(void)
{
  command.header.packet_length = LENGTH_STARTBB;
  command.header.command_code  = AUDCMD_STARTBB;
  command.header.sub_code      = 0;

  command.start_bb_param.output_device     = AS_OUTPUT_DEVICE_SP2CH_I2S2CH;
  command.start_bb_param.input_device      = AS_INPUT_DEVICE_AMIC1CH_I2S2CH;
  command.start_bb_param.select_output_mic = AS_SELECT_MIC0_OR_MIC3;
  command.start_bb_param.I2S_output_data   = AS_MFE_OUTPUT_MICSIN;
  command.start_bb_param.SP_output_data    = AS_MPP_OUTPUT_I2SIN;

  AS_SendAudioCommand( &command );

  return appVolteAudioResultCheck( "appVolteAudioStartBb", AUDRLT_STARTBBCMPLT );
}


static int appVolteAudioStopBb(void)
{
  command.header.packet_length = LENGTH_STOPBB;
  command.header.command_code  = AUDCMD_STOPBB;
  command.header.sub_code      = 0;

  command.stop_bb_param.stop_device       = 0;

  AS_SendAudioCommand( &command );

  return appVolteAudioResultCheck( "appVolteAudioStopBb", AUDRLT_STOPBBCMPLT );
}



static int appVolteAudioSetReadyStatus(void)
{
  command.header.packet_length = LENGTH_SET_READY_STATUS;
  command.header.command_code  = AUDCMD_SETREADYSTATUS;
  command.header.sub_code      = 0x00;
  AS_SendAudioCommand(&command);

  return appVolteAudioResultCheck( "appVolteAudioSetReadyStatus", AUDRLT_STATUSCHANGED );
}

static int appVolteAudioSetPowerOffStatus(void)
{
  command.header.packet_length = LENGTH_SET_POWEROFF_STATUS;
  command.header.command_code  = AUDCMD_SETPOWEROFFSTATUS;
  command.header.sub_code      = 0x00;
  AS_SendAudioCommand(&command);

  return appVolteAudioResultCheck( "appVolteAudioSetPowerOffStatus", AUDRLT_STATUSCHANGED );
}

static void init_libraries(void)
{
  int ret;
  uint32_t addr = AUD_SRAM_ADDR;
  
  mpshm_t s_shm;
  ret = mpshm_init(&s_shm, 1, 1024 * 128 * 2);
  if (ret < 0)
    {
  	  _err("mpshm_init() failure. %d\n", ret);
  	  return;
    }
  ret = mpshm_remap(&s_shm, (void *)addr);
  if (ret < 0)
    {
      _err("mpshm_remap() failure. %d\n", ret);
      return;
    }
  
  /* Initalize MessageLib */
  MsgLib::initFirst(NUM_MSGQ_POOLS,MSGQ_TOP_DRM);
  MsgLib::initPerCpu();

  void* mml_data_area = translatePoolAddrToVa(MEMMGR_DATA_AREA_ADDR);
  Manager::initFirst(mml_data_area, MEMMGR_DATA_AREA_SIZE);
  Manager::initPerCpu(mml_data_area, NUM_MEM_POOLS);
}

static int app_volte_audio_ring_start( void )
{

  /* Create static memory pool of Layout 0 */
  const NumLayout layout_no = MEM_LAYOUT_PLAYER;
  void* work_va = translatePoolAddrToVa(MEMMGR_WORK_AREA_ADDR);
  Manager::createStaticPools(layout_no, work_va, MEMMGR_MAX_WORK_SIZE, MemoryPoolLayouts[layout_no]);

  if( appVolteAudioPowerOn() )
    {
      return 1;
    }

  /* Init Output Select */
  if( appVolteAudioOutputSelect() )
    {
      return 1;
    }

  /* Init Simple Fifo */

  if( appVolteAudioInitSimpleFifo() )
    {
      return 1;
    }

  /* Set Player Status */
  if( appVolteAudioSetPlayerStatus() )
    {
      return 1;
    }

  /* Push data to simple fifo */
  if( appVolteAudioPushSimpleFifo() )
    {
      return 1;
    }

  /* Set Volume */
  if( appVolteAudioSetVolume() )
    {
      return 1;
    }

  /* Init Player */
  if( appVolteAudioInitPlayer() )
    {
      return 1;
    }

  /* Play Player */
  if( appVolteAudioPlayPlayer() )
    {
      return 1;
    }

  return 0;
}


static int app_volte_audio_ring_stop(void)
{
  /* stop Baseband */
  if( appVolteAudioStopPlayer() )
    {
      return 1;
    }

  /* Set ready status */
  if( appVolteAudioSetReadyStatus() )
    {
      return 1;
    }

  /* Power off */
  if( appVolteAudioSetPowerOffStatus() )
    {
      return 1;
    }

  /* destroy static memory pool */
  Manager::destroyStaticPools();

  return 0;
}


static int app_volte_audio_bb_start(void)
{
  if( appVolteAudio_user > 0)
    {
      appVolteAudio_user++;
      return 0;
    }

  /* Create static memory pool of Layout 1 */
  const NumLayout layout_no = MEM_LAYOUT_SOUNDEFFECT;
  void* work_va = translatePoolAddrToVa(MEMMGR_WORK_AREA_ADDR);
  Manager::createStaticPools(layout_no, work_va, MEMMGR_MAX_WORK_SIZE, MemoryPoolLayouts[layout_no]);

  /* Power On */
  if( appVolteAudioPowerOn() )
    {
      return 1;
    }

  /* Init Output Select */
  if( appVolteAudioOutputSelect() )
    {
      return 1;
    }

  /* Init I2S Param */
  if( appVolteAudioInitI2sParam() )
    {
      return 1;
    }

  /* Init Mic Gain */
  if( appVolteAudioInitMicGain() )
    {
      return 1;
    }

  /* Set BBActive Status */
  if( appVolteAudioSetBasebandStatus() )
    {
      return 1;
    }

  /* Init MFE */
  if( appVolteAudioInitMfe() )
    {
      return 1;
    }

  /* Init MPP */
  if( appVolteAudioInitMpp() )
    {
      return 1;
    }

  /* Set Volume */
  if( appVolteAudioSetVolume() )
    {
      return 1;
    }

  /* start Baseband */
  if( appVolteAudioStartBb() )
    {
      return 1;
    }

  appVolteAudio_user++;

  return 0;
}


static int app_volte_audio_bb_stop(void)
{
  if( appVolteAudio_user > 1)
    {
      appVolteAudio_user--;
      return 0;
    }

  /* stop Baseband */
  if( appVolteAudioStopBb() )
    {
      return 1;
    }

  /* Set ready status */
  if( appVolteAudioSetReadyStatus() )
    {
      return 1;
    }

  /* Power off */
  if( appVolteAudioSetPowerOffStatus() )
    {
      return 1;
    }

  /* destroy static memory pool */
  Manager::destroyStaticPools();

  appVolteAudio_user--;
  return 0;
}


static int app_volte_audio_read_ringtone( void )
{
  int fd;
  int ret;
  appVolteAudio_ringtoneSize=0;

  fd = open( "/mnt/spif/ring_tone.mp3", O_RDONLY );

  do
    {
      ret = read(fd, &appVolteAudio_ram_buf, WRITE_SIMPLE_FIFO_SIZE);

      if (ret < 0)
        {
          printf( "Fail to read file\n" );
          close(fd);
          return 1;
        }

      if (ret == 0)
        {
          break;
        }

      memcpy( &appVolteAudio_ringtoneData[appVolteAudio_ringtoneSize], appVolteAudio_ram_buf, ret );
      appVolteAudio_ringtoneSize += ret;

    } while( ret > 0);

  close(fd);

  return 0;
}


extern "C" void app_volte_audio_main( void )
{
  mqd_t mqd;
  struct mq_attr mq_attr;
  char  ctrl;
  int app_volte_ring_state = 0;

  mq_attr.mq_maxmsg  = 10;
  mq_attr.mq_msgsize = sizeof(char);
  mq_attr.mq_flags   = 0;

  mqd = mq_open( "volte_audio_example", O_RDONLY | O_CREAT, 0666, &mq_attr);

  app_volte_audio_read_ringtone();

  while (1)
    {
      if( mq_receive( mqd, &ctrl, sizeof(ctrl), NULL ) )
        {
          switch( ctrl )
            {
              case APP_VOLTE_AUDIO_CTRL_RING_START:
                if( app_volte_audio_ring_start() )
                  {
                    printf("ring tone start fail\n");
                    return;
                  }
                app_volte_ring_state = 1;
                break;

              case APP_VOLTE_AUDIO_CTRL_BB_START:
                if( app_volte_ring_state )
                  {
                    if( app_volte_audio_ring_stop() )
                      {
                        printf("ring tone stop fail\n");
                        return;
                      }
                    app_volte_ring_state = 0;
                  }

                if( app_volte_audio_bb_start())
                  {
                    printf("baseband start fail\n");
                    return;
                  }

                break;

              case APP_VOLTE_AUDIO_CTRL_STOP:
                if( app_volte_ring_state )
                  {
                    if( app_volte_audio_ring_stop() )
                      {
                        printf("ring tone stop fail\n");
                        return;
                      }
                    app_volte_ring_state = 0;
                  }
                else
                  {
                    if( app_volte_audio_bb_stop() )
                      {
                        printf("baseband stop fail\n");
                        return;
                      }
                  }
                break;
              default:
                break;
            }
        }
    }

  return;
}


extern "C" {
int app_volte_audio_init(void)
{
  bool act_rst;
  AudioSubSystemIDs ids;
  pid_t volte_audio_pid = -1;

  init_libraries();

  /* activate audio utils */

  ids.app = MSGQ_AUD_APP;
  ids.mng = MSGQ_AUD_MGR;
  ids.player_main = MSGQ_AUD_PLY;
  ids.player_sub = 0xFF;
  ids.mixer = MSGQ_AUD_OUTPUT_MIX;
  ids.recorder = 0xFF;
  ids.effector = MSGQ_AUD_SOUND_EFFECT;
  ids.recognizer = MSGQ_AUD_RCG_CMD;

  AS_ActivateAudioSubSystem(ids);

  AsActPlayerParam_t player_act_param;
  player_act_param.msgq_id.player = MSGQ_AUD_PLY;
  player_act_param.msgq_id.mng    = MSGQ_AUD_MGR;
  player_act_param.msgq_id.mixer  = MSGQ_AUD_OUTPUT_MIX;
  player_act_param.msgq_id.dsp    = MSGQ_AUD_DSP;
  player_act_param.pool_id.es     = DEC_ES_MAIN_BUF_POOL;
  player_act_param.pool_id.pcm    = REND_PCM_BUF_POOL;
  player_act_param.pool_id.dsp    = DEC_APU_CMD_POOL;

  act_rst = AS_ActivatePlayer(&player_act_param);
  if (!act_rst)
    {
      _err("ERROR AS_ActivatePlayer() failed\n");
      return 1;
    }

  AsActOutputMixParam_t output_mix_act_param;
  output_mix_act_param.msgq_id.mixer = MSGQ_AUD_OUTPUT_MIX;
  output_mix_act_param.msgq_id.mng   = MSGQ_AUD_MGR;

  act_rst = AS_ActivateOutputMix(&output_mix_act_param);
  if (!act_rst)
    {
      _err("ERROR AS_ActivateOutputMix() failed\n");
      return 1;
    }

  AsActEffectorParam_t effector_act_param;
  effector_act_param.msgq_id.effector   = MSGQ_AUD_SOUND_EFFECT;
  effector_act_param.msgq_id.mng        = MSGQ_AUD_MGR;
  effector_act_param.msgq_id.recognizer = MSGQ_AUD_RCG_CMD;
  effector_act_param.msgq_id.dsp        = MSGQ_AUD_DSP;
  effector_act_param.pool_id.mic_in     = MIC_IN_BUF_POOL;
  effector_act_param.pool_id.i2s_in     = I2S_IN_BUF_POOL;
  effector_act_param.pool_id.sphp_out   = HP_OUT_BUF_POOL;
  effector_act_param.pool_id.i2s_out    = I2S_OUT_BUF_POOL;
  effector_act_param.pool_id.mfe_out    = MFE_OUT_BUF_POOL;

  act_rst = AS_ActivateEffector(&effector_act_param);
  if (!act_rst)
    {
      _err("ERROR AS_ActivateEffector() failed\n");
      return 1;
    }

  AsActRendererParam_t renderer_act_param;
  renderer_act_param.msgq_id.dev0_req  = MSGQ_AUD_RND_SPHP;
  renderer_act_param.msgq_id.dev0_sync = MSGQ_AUD_RND_SPHP_SYNC;
  renderer_act_param.msgq_id.dev1_req  = MSGQ_AUD_RND_I2S;
  renderer_act_param.msgq_id.dev1_sync = MSGQ_AUD_RND_I2S_SYNC;

  act_rst = AS_ActivateRenderer(&renderer_act_param);
  if (!act_rst)
    {
      _err("ERROR AS_ActivateRenderer() failed\n");
      return 1;
    }

  AsActCaptureParam_t capture_act_param;
  capture_act_param.msgq_id.dev0_req  = MSGQ_AUD_CAP_MIC;
  capture_act_param.msgq_id.dev0_sync = MSGQ_AUD_CAP_MIC_SYNC;
  capture_act_param.msgq_id.dev1_req  = MSGQ_AUD_CAP_I2S;
  capture_act_param.msgq_id.dev1_sync = MSGQ_AUD_CAP_I2S_SYNC;

  act_rst = AS_ActivateCapture(&capture_act_param);
  if (!act_rst)
    {
      _err("ERROR AS_ActivateCapture() failed\n");
      return 1;
    }

  volte_audio_pid = task_create("audio_ctrl_task", AUDIO_TASK_CTRL_PRIORITY, AUDIO_TASK_CTRL_STACK_SIZE, (main_t)app_volte_audio_main, NULL );
  if (volte_audio_pid < 0)
    {
      printf("ERROR task_create(audio_ctrl_task) failed\n");
      return 1;
    }

  return 0;
}
}
