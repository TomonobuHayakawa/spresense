/****************************************************************************
 * examples/streaming/streaming_audio.c
 *
 *   Copyright (C) 2017 Sony Corporation
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

#include <arch/chip/cxd56_audio.h>

#include "memutils/os_utils/chateau_osal.h"
#include "audio/audio_high_level_api.h"
#include "memutils/simple_fifo/CMN_SimpleFifo.h"
#include "msgq_id.h"
#include "mem_layout.h"
#include "memory_layout.h"
#include "netutils/httpc/http_socket.h"
#include "app_audio.h"
#include "app_playlist.h"
#include "app_memutil.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* task setting */
#define AUDIO_TASK_PRIORITY    (150)
#define AUDIO_TASK_MANAGER_STACK_SIZE (1024 * 2)
#define AUDIO_TASK_PLAYER_STACK_SIZE  (1024 * 3)
#define AUDIO_TASK_OUTMIX_STACK_SIZE  (1024 * 3)
#define AUDIO_TASK_SEFFECT_STACK_SIZE (1024 * 2)

/* for simpleFifo */
#define FIRST_WRITE_NUM			(24)
#define WRITE_SIMPLE_FIFO_SIZE	(2560)
#define SIMPLE_FIFO_FRAME_NUM	(36)
#define SIMPLE_FIFO_BUF_SIZE	(WRITE_SIMPLE_FIFO_SIZE*SIMPLE_FIFO_FRAME_NUM)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* for Audio command execution/response reception */
static AudioCommand command;
static AudioResult result;

static uint8_t audio_simple_fifo_buf[SIMPLE_FIFO_BUF_SIZE];
static uint8_t audio_ram_buf[WRITE_SIMPLE_FIFO_SIZE];

static CMN_SimpleFifoHandle audio_simple_fifo_handle;
static AsPlayerInputDeviceHdlrForRAM audioRamHandler;

/* for streaming */
static int http_socket;
static int stream_status;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int audioResultCheck(char *function_name, uint8_t check_code)
{
  AS_ReceiveAudioResult(&result);
  if (result.header.result_code != check_code)
    {
      printf("%s ERROR: Result code=0x%x\n", function_name, result.header.result_code);
      return 1;
    }

  return 0;
}

static int audioSetVolume(void)
{
  command.header.packet_length = LENGTH_SETVOLUME;
  command.header.command_code  = AUDCMD_SETVOLUME;
  command.header.sub_code      = 0;

  command.set_volume_param.input1_db = -250;
  command.set_volume_param.input2_db = AS_VOLUME_HOLD;
  command.set_volume_param.master_db = AS_VOLUME_DAC;

  AS_SendAudioCommand(&command);

  return audioResultCheck("audioSetVolume", AUDRLT_SETVOLUMECMPLT);
}

static int audioPowerOn(void)
{
  command.header.packet_length = LENGTH_POWERON;
  command.header.command_code  = AUDCMD_POWERON;
  command.header.sub_code      = 0x00;

  command.power_on_param.enable_sound_effect = AS_DISABLE_SOUNDEFFECT;

  AS_SendAudioCommand(&command);

  return audioResultCheck("audioPowerOn", AUDRLT_STATUSCHANGED);
}

static int audioOutputSelect(void)
{
  command.header.packet_length = LENGTH_INITOUTPUTSELECT;
  command.header.command_code  = AUDCMD_INITOUTPUTSELECT;
  command.header.sub_code      = 0;

  command.init_output_select_param.output_device_sel = AS_OUT_SP;

  AS_SendAudioCommand(&command);

  return audioResultCheck("audioOutputSelect", AUDRLT_INITOUTPUTSELECTCMPLT);
}

static int audioOpenStream(int is_https, const char *domain, const char *path)
{
  int rtcd;
  int status;

  NT_http_init();

  http_socket = NT_http_create(is_https);
  if (http_socket < 0)
    {
      printf("HTTP socket creation failed.ret=%d \n", http_socket);
      return 1;
    }

  rtcd = NT_http_connect(http_socket, (char *)domain, (is_https == 1) ? 443 : 80);
  if (rtcd < 0)
    {
      printf("HTTP socket connect failed. ret=%d \n", rtcd);
      return false;
    }

  rtcd = NT_http_get(http_socket,
                     path,
                     NULL,
                     0,
                     &status);
  if (rtcd < 0)
    {
      printf("HTTP GET failed. ret=%d\n", rtcd);
      return 1;
    }

  return 0;
}

static int audioReadStream(uint32_t* read_size)
{
  int rtcd;

  rtcd = NT_http_read_response(http_socket, (char *)audio_ram_buf, *read_size);
  if (rtcd < 0)
    {
      *read_size = 0;
    }
  else
    {
      *read_size = rtcd;
    }

  return 0;
}

static int audioWriteSimpleFifo(uint32_t size)
{
  if (CMN_SimpleFifoOffer(&audio_simple_fifo_handle, audio_ram_buf, size) == 0)
    {
      printf("write simpleFifo failed.\n");
      return 1;
    }

  return 0;
}

static void audioCloseStream(void)
{
  NT_http_close(http_socket);
  http_socket = -1;

  return;
}

static uint32_t audioOfferEsData(void)
{
  uint32_t read_size = WRITE_SIMPLE_FIFO_SIZE;
  size_t vacant_size = CMN_SimpleFifoGetVacantSize(&audio_simple_fifo_handle);

  if (vacant_size < read_size)
    {
      read_size = vacant_size;
    }

  if (audioReadStream(&read_size) < 0)
    {
      return -1;
    }

  if (read_size != 0)
    {
      audioWriteSimpleFifo(read_size);
    }

  return read_size;
}

static int audioOfferEsDataInAdvance(void)
{
  uint32_t frame_num = FIRST_WRITE_NUM;
  uint32_t read_size = WRITE_SIMPLE_FIFO_SIZE;
  size_t vacant_size = 0;

  struct TrackInfo track;

  if( app_streaming_playlist_getTrack(&track) )
    {
      printf("error\n");
    }

  audioOpenStream(track.is_https, track.domain, track.path);

  CMN_SimpleFifoClear(&audio_simple_fifo_handle);

  while (frame_num > 0)
    {
      read_size = WRITE_SIMPLE_FIFO_SIZE;
      vacant_size = CMN_SimpleFifoGetVacantSize(&audio_simple_fifo_handle);

      if (vacant_size < read_size)
        {
          read_size = vacant_size;
        }

      if (audioReadStream(&read_size))
        {
          return 1;
        }

      if (read_size == 0)
        {
          break;
        }

      if (audioWriteSimpleFifo(read_size))
        {
          return 1;
        }

      frame_num--;
    }

  return 0;
}

static void audioCallback(uint32_t size)
{
  return;
}

static int audioInitSimpleFifo(void)
{
  if (CMN_SimpleFifoInitialize(&audio_simple_fifo_handle,
                               audio_simple_fifo_buf,
                               SIMPLE_FIFO_BUF_SIZE, NULL) != 0)
    {
      printf("Fail to initialize simple FIFO.");
      return 1;
    }

  audioRamHandler.simple_fifo_handler = (void*)(&audio_simple_fifo_handle);
  audioRamHandler.callback_function = audioCallback;

  return 0;
}

static int audioSetPlayerStatus(void)
{
  command.header.packet_length  = LENGTH_SET_PLAYER_STATUS;
  command.header.command_code   = AUDCMD_SETPLAYERSTATUS;
  command.header.sub_code       = 0;

  command.set_player_sts_param.input_device  = AS_SETPLAYER_INPUTDEVICE_RAM;
  command.set_player_sts_param.ram_handler   = &audioRamHandler;
  command.set_player_sts_param.output_device = AS_SETPLAYER_OUTPUTDEVICE_SPHP;
  command.set_player_sts_param.output_device_handler = 0x00;

  AS_SendAudioCommand(&command);

  return audioResultCheck("audioSetPlayerStatus", AUDRLT_STATUSCHANGED);
}

static int audioInitPlayer(void)
{
  command.header.packet_length  = LENGTH_INIT_PLAYER;
  command.header.command_code   = AUDCMD_INITPLAYER;
  command.header.sub_code       = 0;

  command.init_player_param.codec_type     = AS_CODECTYPE_MP3;
  command.init_player_param.bit_length     = AS_BITLENGTH_16;
  command.init_player_param.channel_number = AS_CHANNEL_STEREO;
  command.init_player_param.sampling_rate  = AS_SAMPLINGRATE_44100;

  AS_SendAudioCommand(&command);

  return audioResultCheck("audioInitPlayer", AUDRLT_INITPLAYERCMPLT);
}

static int audioPlayPlayer(void)
{
  command.header.packet_length  = LENGTH_PLAY_PLAYER;
  command.header.command_code   = AUDCMD_PLAYPLAYER;
  command.header.sub_code       = 0;

  AS_SendAudioCommand(&command);

  return audioResultCheck("audioPlayPlayer", AUDRLT_PLAYCMPLT);
}

static int audioStopPlayer(void)
{
  command.header.packet_length = LENGTH_STOP_PLAYER;
  command.header.command_code  = AUDCMD_STOPPLAYER;
  command.header.sub_code      = 0;

  AS_SendAudioCommand(&command);

  return audioResultCheck("audioStopPlayer", AUDRLT_STOPCMPLT);
}

static int audioStartBeep(void)
{
  command.header.packet_length = LENGTH_SETBEEPPARAM;
  command.header.command_code  = AUDCMD_SETBEEPPARAM;
  command.header.sub_code      = 0;
  command.set_beep_param.beep_en = 1;
  command.set_beep_param.beep_vol = -45;
  command.set_beep_param.beep_freq = 660;

  AS_SendAudioCommand(&command);

  return audioResultCheck("audioStartBeep", AUDRLT_SETBEEPCMPLT);
}

static int audioStopBeep(void)
{
  command.header.packet_length = LENGTH_SETBEEPPARAM;
  command.header.command_code  = AUDCMD_SETBEEPPARAM;
  command.header.sub_code      = 0;
  command.set_beep_param.beep_en   = 0;
  command.set_beep_param.beep_vol  = AS_BEEP_VOL_HOLD;
  command.set_beep_param.beep_freq = AS_BEEP_FREQ_HOLD;

  AS_SendAudioCommand(&command);

  return audioResultCheck("audioStopBeep", AUDRLT_SETBEEPCMPLT);
}

static void init_libraries(void)
{
  int ret;
  uint32_t addr = AUD_SRAM_ADDR;
  
  mpshm_t s_shm;
  ret = mpshm_init(&s_shm, 1, 1024 * 128 * 2);
  if (ret < 0)
    {
  	  printf("mpshm_init() failure. %d\n", ret);
  	  return;
    }
  ret = mpshm_remap(&s_shm, (void *)addr);
  if (ret < 0)
    {
      printf("mpshm_remap() failure. %d\n", ret);
      return;
    }
  
  /* Initalize MessageLib */
  MsgLib_initFirst(NUM_MSGQ_POOLS, MSGQ_TOP_DRM);
  MsgLib_initPerCpu();

  void* mml_data_area = MemMgr_translatePoolAddrToVa(MEMMGR_DATA_AREA_ADDR);
  MemMgr_initFirst(mml_data_area, MEMMGR_DATA_AREA_SIZE);
  MemMgr_initPerCpu(mml_data_area, NUM_MEM_POOLS);
  
  /* Create static memory pool of Layout 0 */
  void* work_va = MemMgr_translatePoolAddrToVa(MEMMGR_WORK_AREA_ADDR);
  MemMgr_createStaticPools(MEM_LAYOUT_PLAYER, work_va, MEMMGR_MAX_WORK_SIZE);
}

static int app_streaming_audio_playerInit(void)
{
  /* Power on */
  if (audioPowerOn())
    {
      return 1;
    }

  /* Init Output Select */
  if (audioOutputSelect())
    {
      return 1;
    }

  /* Init Simple Fifo */
  if (audioInitSimpleFifo())
    {
      return 1;
    }

  /* Set Player Status */
  if (audioSetPlayerStatus())
    {
      return 1;
    }

  /* Set Volume */
  if (audioSetVolume())
    {
      return 1;
    }

  return 0;
}

static int app_streaming_audio_playMusic(void)
{
  /* Init Player */
  if (audioInitPlayer())
    {
      return 1;
    }

  /* Push data to simple fifo */
  if (audioOfferEsDataInAdvance())
    {
      return 1;
    }

  /* Play Player */
  if (audioPlayPlayer())
    {
      return 1;
    }

  stream_status = D_APP_STS_ACTIVE;

  return 0;
}

static int app_streaming_audio_stopMusic(void)
{
  /* Close socket */
  audioCloseStream();

  /* stop Player */
  if (audioStopPlayer())
    {
      return 1;
    }

  stream_status = D_APP_STS_INACTIVE;

  return 0;
}

static int app_streaming_audio_startBeep( void )
{
  /* start Beep */
  if (audioStartBeep())
    {
      return 1;
    }

  return 0;
}

static int app_streaming_audio_stopBeep(void)
{
  /* stop Beep */
  if (audioStopBeep())
    {
      return 1;
    }

  return 0;
}

static int app_streaming_audio_offerEsData(void)
{
  uint32_t offer_size;

  switch (stream_status)
    {
      case D_APP_STS_INACTIVE:
        break;

      case D_APP_STS_ACTIVE_EOF:
        if (CMN_SimpleFifoGetOccupiedSize(&audio_simple_fifo_handle) != 0)
          {
            break;
          }

        printf("Play next music\n");
        if (app_streaming_audio_stopMusic())
          {
            printf("stop music failed\n");
            return 1;
          }
        if (app_streaming_audio_playMusic())
          {
            return 1;
          }
        break;

      default:  /* D_APP_STS_ACTIVE */
        /* Offer Es Data */
        offer_size = audioOfferEsData();

        if (offer_size < 0)
          {
            printf("Offer Es Data error\n");
            return 1;
          }

        if (offer_size == 0)
          {
            printf("Reach EOF of input file.\n");
            stream_status = D_APP_STS_ACTIVE_EOF;
          }


        break;
    }

  return 0;
}

static void app_streaming_audio_main(void)
{
  mqd_t mqd;
  struct mq_attr mq_attr;
  char  ctrl;


  struct timespec curr_time;

  memset(&curr_time, 0, sizeof(curr_time));

  if (app_streaming_audio_playerInit())
    {
      printf("audio player initialization failed\n");
      return;
    }

  app_streaming_playlist_open();

  mq_attr.mq_maxmsg  = 10;
  mq_attr.mq_msgsize = sizeof(char);
  mq_attr.mq_flags   = 0;

  mqd = mq_open("stream_ctrl", O_RDONLY | O_CREAT, 0666, &mq_attr);

  stream_status = D_APP_STS_INACTIVE;

  while (1)
    {
      usleep(5*1000); /* sleep 5 miliseconds */

      if (CMN_SimpleFifoGetVacantSize(&audio_simple_fifo_handle) > WRITE_SIMPLE_FIFO_SIZE)
        {
          app_streaming_audio_ctrl(D_APP_OFFER_ESDATA);
        }

      while (1)
        {
          if (stream_status == D_APP_STS_INACTIVE)
            {
              if (mq_receive(mqd, &ctrl, sizeof(ctrl), NULL ) <= 0)
                {
                  break;
                }
            }
          else
            {
              if (mq_timedreceive(mqd, &ctrl, sizeof(ctrl), NULL, &curr_time ) <= 0)
                {
                  break;
                }
            }

          switch (ctrl)
            {
              case D_APP_START_STREAMING:
                if (app_streaming_audio_playMusic())
                  {
                    printf("streaming start fail\n");
                    return;
                  }
                break;

              case D_APP_STOP_STREAMING:
                if (app_streaming_audio_stopMusic())
                  {
                    printf("streaming stop fail\n");
                    return;
                  }
                break;

              case D_APP_START_BEEP:
                if (app_streaming_audio_startBeep())
                  {
                    printf("Beep start fail\n");
                    return;
                  }
                break;

              case D_APP_STOP_BEEP:
                if (app_streaming_audio_stopBeep())
                  {
                    printf("Beep stop fail\n");
                    return;
                  }
                break;

              case D_APP_OFFER_ESDATA:
                if (app_streaming_audio_offerEsData())
                  {
                    printf("Offer ES Data  fail\n");
                    return;
                  }
                break;

              default:
                break;
            }
        }
    }

  return;
}

int app_streaming_audio_init(void)
{
  pid_t stream_pid      = -1;

  init_libraries();

  ActivateAudioSubSystem();

  if (!ActivatePlayer())
    {
      printf("ERROR ActivatePlayer(PLY_OBJ) failed\n");
      return 1;
    }

  if (!ActivateSubPlayer())
    {
      printf("ERROR ActivateSubPlayer() failed\n");
      return 1;
    }

  if (!ActivateOutputMix())
    {
      printf("ERROR ActivateOutputMix() failed\n");
      return 1;
    }

  if (!ActivateRenderer())
    {
      printf("ERROR ActivateRenderer() failed\n");
      return 1;
    }

  stream_pid = task_create("stream_task", 100, 2048,
                           (main_t)app_streaming_audio_main, NULL );
  if (stream_pid < 0)
    {
      printf("ERROR task_create(stream_task) failed\n");
      return 1;
    }

  return 0;
}

void app_streaming_audio_ctrl(char ctrl)
{
  mqd_t mqd;
  mqd = mq_open("stream_ctrl", O_WRONLY, 0666, NULL);

  if (mq_send(mqd, &ctrl, sizeof(ctrl), 0 ) != 0)
    {
      printf("mq_send to stream_ctrl queue falied\n");
    }

  mq_close(mqd);

  return;
}
