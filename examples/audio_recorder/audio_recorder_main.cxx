/****************************************************************************
 * examples/audio_recorder/audio_recorder_main.c
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Tomonobu Hayakawa<Tomonobu.Hayakawa@sony.com>
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

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <errno.h>
#include <asmp/mpshm.h>
#include <sys/stat.h>

#include "memutils/os_utils/chateau_osal.h"
#include "memutils/simple_fifo/CMN_SimpleFifo.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"
#include "audio/audio_high_level_api.h"
#include "include/msgq_id.h"
#include "include/mem_layout.h"
#include "include/memory_layout.h"
#include "include/msgq_pool.h"
#include "include/pool_layout.h"
#include "include/fixed_fence.h"

#include <arch/chip/cxd56_audio.h>

using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_AUDIO_RECORDER_FILE_MOUNTPT
#  define CONFIG_EXAMPLES_AUDIO_RECORDER_FILE_MOUNTPT "/mnt/vfat/REC"
#endif

/* For FIFO. */

#define READ_SIMPLE_FIFO_SIZE 3072
#define SIMPLE_FIFO_FRAME_NUM 10
#define SIMPLE_FIFO_BUF_SIZE  (READ_SIMPLE_FIFO_SIZE * SIMPLE_FIFO_FRAME_NUM)

/* For wave header. */

#define CHUNKID_RIFF      "RIFF"
#define FORMAT_WAVE       "WAVE"
#define SUBCHUNKID_FMT    "fmt "
#define SUBCHUNKID_DATA   "data"
#define AUDIO_FORMAT_PCM  0x0001
#define FMT_SIZE          0x10

/* Length of recording file name */

#define MAX_PATH_LENGTH 128

/* Recording time(sec). */

#define RECORDER_REC_TIME 10

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

struct wav_format_s
{
  uint8_t  riff[4];    /* "RIFF" */
  uint32_t total_size;
  uint8_t  wave[4];    /* "WAVE" */
  uint8_t  fmt[4];     /* "fmt " */
  uint32_t fmt_size;   /* fmt chunk size */
  uint16_t format;     /* format type */
  uint16_t channel;
  uint32_t rate;       /* sampling rate */
  uint32_t avgbyte;    /* rate * block */
  uint16_t block;      /* channels * bit / 8 */
  uint16_t bit;        /* bit length */
  uint8_t  data[4];    /* "data" */
  uint32_t data_size;
};
typedef struct wav_format_s WAVEFORMAT;

/* For FIFO */

struct recorder_fifo_info_s
{
  CMN_SimpleFifoHandle        handle;
  AsRecorderOutputDeviceHdlr  output_device;
  uint32_t fifo_area[SIMPLE_FIFO_BUF_SIZE/sizeof(uint32_t)];
  uint8_t  write_buf[READ_SIMPLE_FIFO_SIZE];
};

struct recorder_file_info_s
{
  uint32_t  sampling_rate;
  uint8_t   channel_number;
  uint8_t   codec_type;
  uint32_t  size;
  DIR      *dirp;
  FILE     *fd;
};

struct recorder_info_s
{
  struct recorder_fifo_info_s  fifo;
  struct recorder_file_info_s  file;
  WAVEFORMAT wav_header;
};

/* Recording parameter. */

enum codec_type_e
{
  CODEC_TYPE_MP3 = 0,
  CODEC_TYPE_WAV,
  CODEC_TYPE_OPUS,
  CODEC_TYPE_NUM
};

enum sampling_rate_e
{
  SAMPLING_RATE_8K = 0,
  SAMPLING_RATE_16K,
  SAMPLING_RATE_48K,
  SAMPLING_RATE_NUM
};

enum channel_type_e
{
  CHAN_TYPE_MONO = 0,
  CHAN_TYPE_STEREO,
  CHAN_TYPE_NUM
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static recorder_info_s s_recorder_info;

/* For share memory. */

static mpshm_t s_shm;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void outputDeviceCallback(uint32_t size)
{
    /* do nothing */
}

static void app_update_wav_file_size(void)
{
  fseek(s_recorder_info.file.fd, 0, SEEK_SET);
  s_recorder_info.wav_header.total_size =
    s_recorder_info.file.size + sizeof(WAVEFORMAT) - 8;
  s_recorder_info.wav_header.data_size  =
    s_recorder_info.file.size;
}

static bool app_write_wav_header(void)
{
  size_t ret = fwrite((const void *)&s_recorder_info.wav_header,
                      1,
                      sizeof(WAVEFORMAT),
                      s_recorder_info.file.fd);
  if (ret != sizeof(WAVEFORMAT))
    {
      printf("Fail to write file(wav header)\n");
      return false;
    }
  return true;
}

static bool app_open_output_file(void)
{
  static char fname[MAX_PATH_LENGTH];

  struct tm *cur_time;
  struct timespec cur_sec;

  clock_gettime(CLOCK_REALTIME, &cur_sec);
  cur_time = gmtime(&cur_sec.tv_sec);

  if (s_recorder_info.file.codec_type == AS_INITREC_MP3)
    {
      snprintf(fname, MAX_PATH_LENGTH, "%s/%04d%02d%02d_%02d%02d%02d.mp3",
        CONFIG_EXAMPLES_AUDIO_RECORDER_FILE_MOUNTPT,
        cur_time->tm_year,
        cur_time->tm_mon,
        cur_time->tm_mday,
        cur_time->tm_hour,
        cur_time->tm_min,
        cur_time->tm_sec);
    }
  else if (s_recorder_info.file.codec_type == AS_INITREC_WAV)
    {
      snprintf(fname, MAX_PATH_LENGTH, "%s/%04d%02d%02d_%02d%02d%02d.wav",
        CONFIG_EXAMPLES_AUDIO_RECORDER_FILE_MOUNTPT,
        cur_time->tm_year,
        cur_time->tm_mon,
        cur_time->tm_mday,
        cur_time->tm_hour,
        cur_time->tm_min,
        cur_time->tm_sec);
    }
  else
    {
      snprintf(fname, MAX_PATH_LENGTH, "%s/%04d%02d%02d_%02d%02d%02d.opus",
        CONFIG_EXAMPLES_AUDIO_RECORDER_FILE_MOUNTPT,
        cur_time->tm_year,
        cur_time->tm_mon,
        cur_time->tm_mday,
        cur_time->tm_hour,
        cur_time->tm_min,
        cur_time->tm_sec);
    }

  s_recorder_info.file.fd = fopen(fname, "w");
  if (s_recorder_info.file.fd == 0)
    {
      printf("open err(%s)\n", fname);
      return false;
    }
  printf("Record data to %s.\n", &fname[0]);

  if (s_recorder_info.file.codec_type == AS_INITREC_WAV)
    {
      if (!app_write_wav_header())
      {
        printf("Error: app_write_wav_header() failure.\n");
        return false;
      }
  }
  return true;
}

static void app_close_output_file(void)
{
  fclose(s_recorder_info.file.fd);
}

static void app_write_output_file(uint32_t size)
{
  ssize_t ret;

  if (size == 0 || CMN_SimpleFifoGetOccupiedSize(&s_recorder_info.fifo.handle) == 0)
    {
      return;
    }

  if (CMN_SimpleFifoPoll(&s_recorder_info.fifo.handle,
                        (void*)s_recorder_info.fifo.write_buf,
                        size) == 0)
    {
      printf("ERROR: Fail to get data from simple FIFO.\n");
      return;
    }

  ret = fwrite(s_recorder_info.fifo.write_buf, 1, size, s_recorder_info.file.fd);
  if (ret < 0) {
    printf("ERROR: Cannot write recorded data to output file.\n");
    app_close_output_file();
    return;
  }
  s_recorder_info.file.size += size;
}

static bool app_init_simple_fifo(void)
{
  if (CMN_SimpleFifoInitialize(&s_recorder_info.fifo.handle,
                               s_recorder_info.fifo.fifo_area,
                               SIMPLE_FIFO_BUF_SIZE, NULL) != 0)
    {
      printf("Error: Fail to initialize simple FIFO.");
      return false;
    }
  CMN_SimpleFifoClear(&s_recorder_info.fifo.handle);

  s_recorder_info.fifo.output_device.simple_fifo_handler =
    (void*)(&s_recorder_info.fifo.handle);
  s_recorder_info.fifo.output_device.callback_function = outputDeviceCallback;

  return true;
}

static void app_pop_simple_fifo(void)
{
  size_t occupied_simple_fifo_size =
    CMN_SimpleFifoGetOccupiedSize(&s_recorder_info.fifo.handle);
  uint32_t output_size = 0;

  while (occupied_simple_fifo_size > 0)
    {
      output_size = (occupied_simple_fifo_size > READ_SIMPLE_FIFO_SIZE) ?
        READ_SIMPLE_FIFO_SIZE : occupied_simple_fifo_size;
      app_write_output_file(output_size);
      occupied_simple_fifo_size -= output_size;
    }
}

static bool printAudCmdResult(uint8_t command_code, AudioResult& result)
{
  if (AUDRLT_ERRORRESPONSE == result.header.result_code) {
    printf("Command code(0x%x): AUDRLT_ERRORRESPONSE:"
           "Module id(0x%x): Error code(0x%x)\n",
            command_code,
            result.error_response_param.module_id,
            result.error_response_param.error_code);
    return false;
  }
  else if (AUDRLT_ERRORATTENTION == result.header.result_code) {
    printf("Command code(0x%x): AUDRLT_ERRORATTENTION\n", command_code);
    return false;
  }
  return true;
}

static void app_attention_callback(uint8_t module_id,
                                   uint8_t error_code,
                                   uint8_t sub_code,
                                   const char* file_name,
                                   uint32_t line)
{
  printf("Attention!! %s L%d ecode %d subcode %d\n", file_name, line, error_code, sub_code);
}

static bool app_init_attention(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_INITATTENTIONS;
  command.header.command_code  = AUDCMD_INITATTENTIONS;
  command.header.sub_code      = 0x00;
  command.init_attentions_param.attention_callback_function = app_attention_callback;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_act_audio_sub_system(void)
{
  bool result = false;

  /* Activate manager of AudioSubSystem. */

  AudioSubSystemIDs ids;
  ids.app         = MSGQ_AUD_APP;
  ids.mng         = MSGQ_AUD_MGR;
  ids.player_main = 0xFF;
  ids.player_sub  = 0xFF;
  ids.mixer       = 0xFF;
  ids.recorder    = MSGQ_AUD_RECORDER;
  ids.effector    = 0xFF;
  ids.recognizer  = 0xFF;

  AS_ActivateAudioSubSystem(ids);

  /* Set callback function of attention message */

  if (!app_init_attention())
    {
      printf("Error: app_init_attention() failure.\n");
      return 1;
    }


  AsActRecorderParam_t recorder_act_param;
  recorder_act_param.msgq_id.recorder      = MSGQ_AUD_RECORDER;
  recorder_act_param.msgq_id.recorder_sink = MSGQ_AUD_MEDIA_REC_SINK;
  recorder_act_param.msgq_id.mng           = MSGQ_AUD_MGR;
  recorder_act_param.msgq_id.dsp           = MSGQ_AUD_DSP;
  recorder_act_param.pool_id.input         = MIC_IN_BUF_POOL;
  recorder_act_param.pool_id.output        = OUTPUT_BUF_POOL;
  recorder_act_param.pool_id.dsp           = ENC_APU_CMD_POOL;

  result = AS_ActivateVoiceRecorder(&recorder_act_param);
  if (!result)
    {
      printf("Error: AS_ActivateVoiceRecorder() failure. system memory insufficient!\n");
      return false;
    }

  /* Activate recorder sink feature. */

  AsActRecorderSinkParam_t recorder_sink_act_param;
  recorder_sink_act_param.msgq_id.recorder      = MSGQ_AUD_RECORDER;
  recorder_sink_act_param.msgq_id.recorder_sink = MSGQ_AUD_MEDIA_REC_SINK;

  result = AS_ActivateVoiceRecorderSink(&recorder_sink_act_param);
  if (!result)
    {
      printf("Error: AS_ActivateVoiceRecorderSink() failed. system memory insufficient!\n");
      return false;
    }

  /* Activate renderer feature. */

  AsActCaptureParam_t capture_act_param;
  capture_act_param.msgq_id.dev0_req  = MSGQ_AUD_CAP;
  capture_act_param.msgq_id.dev0_sync = MSGQ_AUD_CAP_SYNC;
  capture_act_param.msgq_id.dev1_req  = 0xFF;
  capture_act_param.msgq_id.dev1_sync = 0xFF;

  result = AS_ActivateCapture(&capture_act_param);
  if (!result)
    {
      printf("Error: AS_ActivateCapture() failure. system memory insufficient!\n");
      return false;
    }

  return true;
}

static void app_deact_audio_sub_system(void)
{
  AS_DeactivateAudioSubSystem();
  AS_DeactivateVoiceRecorder();
  AS_DeactivateVoiceRecorderSink();
  AS_DeactivateCapture();
}

static bool app_power_on(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_POWERON;
  command.header.command_code  = AUDCMD_POWERON;
  command.header.sub_code      = 0x00;
  command.power_on_param.enable_sound_effect = AS_DISABLE_SOUNDEFFECT;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_power_off(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SET_POWEROFF_STATUS;
  command.header.command_code  = AUDCMD_SETPOWEROFFSTATUS;
  command.header.sub_code      = 0x00;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_set_ready(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SET_READY_STATUS;
  command.header.command_code  = AUDCMD_SETREADYSTATUS;
  command.header.sub_code      = 0x00;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_init_mic_gain(void)
{
  AudioCommand command;
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
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_set_recording_param(codec_type_e codec_type,
                                    sampling_rate_e sampling_rate,
                                    channel_type_e ch_type)
{
  switch(codec_type)
    {
      case CODEC_TYPE_MP3:
        s_recorder_info.file.codec_type = AS_INITREC_MP3;
        break;
      case CODEC_TYPE_WAV:
        s_recorder_info.file.codec_type = AS_INITREC_WAV;
        break;
      case CODEC_TYPE_OPUS:
        s_recorder_info.file.codec_type = AS_INITREC_OPUS;
        break;
      default:
        printf("Error: Invalid codec type(%d)\n", codec_type);
        return false;
    }

  switch(sampling_rate)
    {
      case SAMPLING_RATE_8K:
        s_recorder_info.file.sampling_rate = AS_INITREC_SAMPLING_8K;
        break;
      case SAMPLING_RATE_16K:
        s_recorder_info.file.sampling_rate = AS_INITREC_SAMPLING_16K;
        break;
      case SAMPLING_RATE_48K:
        s_recorder_info.file.sampling_rate = AS_INITREC_SAMPLING_48K;
        break;
      default:
        printf("Error: Invalid sampling rate(%d)\n", sampling_rate);
        return false;
    }

  switch(ch_type)
    {
      case CHAN_TYPE_MONO:
        s_recorder_info.file.channel_number = AS_INITREC_CHNL_MONO;
        break;
      case CHAN_TYPE_STEREO:
        s_recorder_info.file.channel_number = AS_INITREC_CHNL_STEREO;
        break;
      default:
        printf("Error: Invalid channel type(%d)\n", ch_type);
        return false;
    }

  return true;
}

static bool app_set_recorder_status(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SET_RECORDER_STATUS;
  command.header.command_code  = AUDCMD_SETRECORDERSTATUS;
  command.header.sub_code      = 0x00;
  command.set_recorder_status_param.input_device          = AS_SETRECDR_STS_INPUTDEVICE_MIC_A;
  command.set_recorder_status_param.input_device_handler  = 0x00;
  command.set_recorder_status_param.output_device         = AS_SETRECDR_STS_OUTPUTDEVICE_RAM;
  command.set_recorder_status_param.output_device_handler = &s_recorder_info.fifo.output_device;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static void app_init_recorder_wav(AudioCommand* command)
{
  /* Update wave header */

  memcpy(s_recorder_info.wav_header.riff, CHUNKID_RIFF,    strlen(CHUNKID_RIFF));
  memcpy(s_recorder_info.wav_header.wave, FORMAT_WAVE,     strlen(FORMAT_WAVE));
  memcpy(s_recorder_info.wav_header.fmt,  SUBCHUNKID_FMT,  strlen(SUBCHUNKID_FMT));
  memcpy(s_recorder_info.wav_header.data, SUBCHUNKID_DATA, strlen(SUBCHUNKID_DATA));
  s_recorder_info.wav_header.total_size = 0;
  s_recorder_info.wav_header.fmt_size   = FMT_SIZE;
  s_recorder_info.wav_header.format     = AUDIO_FORMAT_PCM;
  s_recorder_info.wav_header.channel    = s_recorder_info.file.channel_number;
  s_recorder_info.wav_header.rate       = s_recorder_info.file.sampling_rate;
  s_recorder_info.wav_header.avgbyte    =
    s_recorder_info.file.sampling_rate * s_recorder_info.file.channel_number * 2;
  s_recorder_info.wav_header.block      = s_recorder_info.file.channel_number * 2;
  s_recorder_info.wav_header.bit        = 2 * 8;
  s_recorder_info.wav_header.data_size  = 0;

  command->init_recorder_param.codec_type = AS_INITREC_WAV;
}

static void app_init_recorder_mp3(AudioCommand* command)
{
  command->init_recorder_param.codec_type = AS_INITREC_MP3;
  command->init_recorder_param.bitrate    = AS_INITREC_BITRATE_96KBPS;
}

static void app_init_recorder_opus(AudioCommand* command)
{
  command->init_recorder_param.codec_type = AS_INITREC_OPUS;
  command->init_recorder_param.bitrate    = AS_INITREC_BITRATE_8KBPS;
  command->init_recorder_param.computational_complexity = AS_INITREC_COMPLEXITY_0;
}

static bool app_init_recorder(codec_type_e codec_type,
                              sampling_rate_e sampling_rate,
                              channel_type_e ch_type)
{
  if (!app_set_recording_param(CODEC_TYPE_WAV,
                               SAMPLING_RATE_16K,
                               CHAN_TYPE_MONO))
    {
      printf("Error: app_set_recording_param() failure.\n");
      return false;
    }

  AudioCommand command;
  command.header.packet_length = LENGTH_INIT_RECORDER;
  command.header.command_code  = AUDCMD_INITREC;
  command.header.sub_code      = 0x00;
  command.init_recorder_param.sampling_rate  = s_recorder_info.file.sampling_rate;
  command.init_recorder_param.channel_number = s_recorder_info.file.channel_number;
  command.init_recorder_param.bit_length     = AS_INITREC_BITLENGTH_16;

  switch (s_recorder_info.file.codec_type)
    {
      case AS_INITREC_WAV:
        app_init_recorder_wav(&command);
        break;
      case AS_INITREC_MP3:
        app_init_recorder_mp3(&command);
        break;
      case AS_INITREC_OPUS:
        app_init_recorder_opus(&command);
        break;
      default:
        break;
  }

  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);

}

static bool app_start_recorder(void)
{
  s_recorder_info.file.size = 0;
  if (!app_open_output_file())
    {
      return false;
    }
  CMN_SimpleFifoClear(&s_recorder_info.fifo.handle);

  AudioCommand command;
  command.header.packet_length = LENGTH_START_RECORDER;
  command.header.command_code  = AUDCMD_STARTREC;
  command.header.sub_code      = 0x00;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_stop_recorder(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_STOP_RECORDER;
  command.header.command_code  = AUDCMD_STOPREC;
  command.header.sub_code      = 0x00;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  if (!printAudCmdResult(command.header.command_code, result))
    {
      return false;
    }

  size_t occupied_simple_fifo_size = CMN_SimpleFifoGetOccupiedSize(&s_recorder_info.fifo.handle);
  uint32_t output_size = 0;

  while (occupied_simple_fifo_size > 0)
    {
      output_size = (occupied_simple_fifo_size > READ_SIMPLE_FIFO_SIZE) ?
        READ_SIMPLE_FIFO_SIZE : occupied_simple_fifo_size;
      app_write_output_file(output_size);
      occupied_simple_fifo_size = CMN_SimpleFifoGetOccupiedSize(&s_recorder_info.fifo.handle);
    }

  if (s_recorder_info.file.codec_type == AS_INITREC_WAV)
    {
      app_update_wav_file_size();
      if (!app_write_wav_header())
        {
          printf("Error: app_write_wav_header() failure.\n");
        }
    }

  app_close_output_file();
  return true;
}

static bool app_open_file_dir(void)
{
  DIR *dirp;
  int ret;
  char *name = CONFIG_EXAMPLES_AUDIO_RECORDER_FILE_MOUNTPT;

  dirp = opendir("/mnt");
  if (!dirp)
    {
      printf("opendir err(errno:%d)\n",errno);
      return false;
    }
  ret = mkdir(name, 0777);
  if (ret != 0)
    {
      if(errno != EEXIST)
        {
          printf("mkdir err(errno:%d)\n",errno);
          return false;
        }
    }

  s_recorder_info.file.dirp = dirp;
  return true;
}

static bool app_close_file_dir(void)
{
  closedir(s_recorder_info.file.dirp);

  return true;
}

static bool app_init_libraries(void)
{
  int ret;
  uint32_t addr = AUD_SRAM_ADDR;

  /* Initialize shared memory.*/

  ret = mpshm_init(&s_shm, 1, 1024 * 128 * 2);
  if (ret < 0)
    {
      printf("Error: mpshm_init() failure. %d\n", ret);
      return false;
    }

  ret = mpshm_remap(&s_shm, (void *)addr);
  if (ret < 0)
    {
      printf("Error: mpshm_remap() failure. %d\n", ret);
      return false;
    }

  /* Initalize MessageLib. */

  err_t err = MsgLib::initFirst(NUM_MSGQ_POOLS, MSGQ_TOP_DRM);
  if (err != ERR_OK)
    {
      printf("Error: MsgLib::initFirst() failure. 0x%x\n", err);
      return false;
    }

  err = MsgLib::initPerCpu();
  if (err != ERR_OK)
    {
      printf("Error: MsgLib::initPerCpu() failure. 0x%x\n", err);
      return false;
    }

  void* mml_data_area = translatePoolAddrToVa(MEMMGR_DATA_AREA_ADDR);
  err = Manager::initFirst(mml_data_area, MEMMGR_DATA_AREA_SIZE);
  if (err != ERR_OK)
    {
      printf("Error: Manager::initFirst() failure. 0x%x\n", err);
      return false;
    }

  err = Manager::initPerCpu(mml_data_area, NUM_MEM_POOLS);
  if (err != ERR_OK)
    {
      printf("Error: Manager::initPerCpu() failure. 0x%x\n", err);
      return false;
    }

  /* Create static memory pool of VoiceCall. */

  const NumLayout layout_no = MEM_LAYOUT_RECORDER;
  void* work_va = translatePoolAddrToVa(MEMMGR_WORK_AREA_ADDR);
  err = Manager::createStaticPools(layout_no,
                             work_va,
                             MEMMGR_MAX_WORK_SIZE,
                             MemoryPoolLayouts[layout_no]);
  if (err != ERR_OK)
    {
      printf("Error: Manager::initPerCpu() failure. %d\n", err);
      return false;
    }

  return true;
}

static bool app_finalize_libraries(void)
{
  /* Finalize MessageLib. */

  MsgLib::finalize();

  /* Destroy static pools. */

  MemMgrLite::Manager::destroyStaticPools();

  /* Finalize memory manager. */

  MemMgrLite::Manager::finalize();

  /* Destroy shared memory. */

  int ret;
  ret = mpshm_detach(&s_shm);
  if (ret < 0)
    {
      printf("Error: mpshm_detach() failure. %d\n", ret);
      return false;
    }

  ret = mpshm_destroy(&s_shm);
  if (ret < 0)
    {
      printf("Error: mpshm_destroy() failure. %d\n", ret);
      return false;
    }

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
extern "C" int main(int argc, FAR char *argv[])
#else
extern "C" int recorder_main(int argc, char *argv[])
#endif
{
  printf("Start AudioRecorder example\n");

  
  /* First, initialize the shared memory and memory utility used by AudioSubSystem. */

  if (!app_init_libraries())
    {
      printf("Error: init_libraries() failure.\n");
      return 1;
    }

  /* Next, Activate the features used by AudioSubSystem. */

  if (!app_act_audio_sub_system())
    {
      printf("Error: act_audiosubsystem() failure.\n");
      return 1;
    }

  /* Open directory of recording file. */

  if (!app_open_file_dir())
    {
      printf("Error: app_open_file_dir() failure.\n");
      return 1;
    }
  
   /* On and after this point, AudioSubSystem must be active.
   * Register the callback function to be notified when a problem occurs.
   */

  /* Change AudioSubsystem to Ready state so that I/O parameters can be changed. */

  if (!app_power_on())
    {
      printf("Error: app_power_on() failure.\n");
      return 1;
    }

  /* Initialize simple fifo. */

  if (!app_init_simple_fifo())
    {
      printf("Error: app_init_simple_fifo() failure.\n");
      return false;
    }

  /* Set the initial gain of the microphone to be used. */

  if (!app_init_mic_gain())
    {
      printf("Error: app_init_mic_gain() failure.\n");
      return 1;
    }

  /* Set recorder operation mode. */

  if (!app_set_recorder_status())
    {
      printf("Error: app_set_recorder_status() failure.\n");
      return 1;
    }

  /* Initialize recorder. */

  if (!app_init_recorder(CODEC_TYPE_WAV,
                         SAMPLING_RATE_16K,
                         CHAN_TYPE_MONO))
    {
      printf("Error: app_init_recorder() failure.\n");
      return 1;
    }

  /* Start recorder operation. */

  if (!app_start_recorder())
    {
      printf("Error: app_start_recorder() failure.\n");
      return 1;
    }

  /* Running... */

  printf("Running time is %d sec\n", RECORDER_REC_TIME);
  for(int i = 0; i < RECORDER_REC_TIME * 100; i++)
    {
      /* Check the FIFO every 10 ms and pop if there is data. */

      usleep(10 * 1000);
      app_pop_simple_fifo();
    }

  /* Stop recorder operation. */

  if (!app_stop_recorder())
    {
      printf("Error: app_stop_recorder() failure.\n");
      return 1;
    }

  /* Close directory of recording file. */

  if (!app_close_file_dir())
    {
      printf("Error: app_close_contents_dir() failure.\n");
      return 1;
    }

  /* Return the state of AudioSubSystem before voice_call operation. */

  if (!app_set_ready())
    {
      printf("Error: app_set_ready() failure.\n");
      return 1;
    }

  /* Change AudioSubsystem to PowerOff state. */

  if (!app_power_off())
    {
      printf("Error: app_power_off() failure.\n");
      return 1;
    }

  /* Deactivate the features used by AudioSubSystem. */

  app_deact_audio_sub_system();

  /* finalize the shared memory and memory utility used by AudioSubSystem. */

  if (!app_finalize_libraries())
    {
      printf("Error: finalize_libraries() failure.\n");
      return 1;
    }

  printf("Exit AudioRecorder example\n");

  return 0;
}
