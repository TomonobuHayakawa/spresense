/****************************************************************************
 * examples/audio_player/audio_player_main.c
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

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <errno.h>
#include <asmp/mpshm.h>
#include <arch/chip/pm.h>
#include <arch/board/board.h>
#include <sys/stat.h>
#include "memutils/os_utils/chateau_osal.h"
#include "audio/audio_high_level_api.h"
#include "memutils/simple_fifo/CMN_SimpleFifo.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"
#include "include/msgq_id.h"
#include "include/mem_layout.h"
#include "include/memory_layout.h"
#include "include/msgq_pool.h"
#include "include/pool_layout.h"
#include "include/fixed_fence.h"
#include "playlist/playlist.h"

using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_AUDIOUTILS_PLAYLIST_PLAYFILE_MOUNTPT
#  define CONFIG_AUDIOUTILS_PLAYLIST_PLAYFILE_MOUNTPT "/mnt/sd0/AUDIO"
#endif

/* PlayList file name */

#define PLAY_LIST_NAME "TRACK_DB.CSV"

/* For FIFO */

#define WRITE_SIMPLE_FIFO_SIZE  2560
#define SIMPLE_FIFO_FRAME_NUM   9
#define SIMPLE_FIFO_BUF_SIZE    WRITE_SIMPLE_FIFO_SIZE * SIMPLE_FIFO_FRAME_NUM

#define FIFO_RESULT_OK  0
#define FIFO_RESULT_ERR 1
#define FIFO_RESULT_EOF 2
#define FIFO_RESULT_FUL 3

/* Default Volume. -20dB */

#define PLAYER_DEF_VOLUME -200

/* Play time(sec) */

#define PLAYER_PLAY_TIME 10

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

/* For FIFO */

struct player_fifo_info_s
{
  CMN_SimpleFifoHandle          handle;
  AsPlayerInputDeviceHdlrForRAM input_device;
  uint32_t fifo_area[SIMPLE_FIFO_BUF_SIZE/sizeof(uint32_t)];
  uint8_t  read_buf[WRITE_SIMPLE_FIFO_SIZE];
};

/* For play file */

struct player_file_info_s
{
  int32_t size;
  DIR    *dirp;
  int     fd;
};

/* Player info */

struct player_info_s
{
  struct player_fifo_info_s   fifo;
  struct player_file_info_s   file;
#ifdef CONFIG_AUDIOUTILS_PLAYLIST
  Playlist *playlist_ins = NULL;
#else
error "AUDIOUTILS_PLAYLIST is not enable"
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* For control player */

static struct player_info_s  s_player_info;

/* For share memory. */

static mpshm_t s_shm;

/* For frequency lock. */

static struct pm_cpu_freqlock_s s_player_lock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void app_init_freq_lock(void)
{
#ifndef CONFIG_PM_DISABLE_FREQLOCK_COUNT
  s_player_lock.count = 0;
#endif
  s_player_lock.info = PM_CPUFREQLOCK_TAG('A', 'P', 0);
  s_player_lock.flag = PM_CPUFREQLOCK_FLAG_HV;
}

static void app_freq_lock(void)
{
  up_pm_acquire_freqlock(&s_player_lock);
}

static void app_freq_release(void)
{
  up_pm_release_freqlock(&s_player_lock);
}

static bool app_open_contents_dir(void)
{
  DIR *dirp;
  const char *name = CONFIG_AUDIOUTILS_PLAYLIST_PLAYFILE_MOUNTPT;
  
  dirp = opendir(name);

  if (!dirp)
    {
      printf("Error: %s directory path error. check the path!\n", name);
      return false;
    }

  s_player_info.file.dirp = dirp;

  return true;
}

static bool app_close_contents_dir(void)
{
  closedir(s_player_info.file.dirp);

  return true;
}


static bool app_open_playlist(void)
{
  bool result = false;

  if (s_player_info.playlist_ins != NULL)
    {
      printf("Error: Open playlist failure. Playlist is already open\n");
      return false;
    }

  s_player_info.playlist_ins = new Playlist(PLAY_LIST_NAME);
  
  result = s_player_info.playlist_ins->init();
  if (!result)
    {
      printf("Error: Playlist::init() failure.\n");
      return false;
    }

  s_player_info.playlist_ins->setPlayMode(Playlist::PlayModeNormal);
  if (!result)
    {
      printf("Error: Playlist::setPlayMode() failure.\n");
      return false;
    }

  s_player_info.playlist_ins->setRepeatMode(Playlist::RepeatModeOff);
  if (!result)
    {
      printf("Error: Playlist::setRepeatMode() failure.\n");
      return false;
    }

  s_player_info.playlist_ins->select(Playlist::ListTypeAllTrack, NULL);
  if (!result)
    {
      printf("Error: Playlist::select() failure.\n");
      return false;
    }

  return true;
}

static bool app_close_playlist(void)
{
  if (s_player_info.playlist_ins == NULL)
    {
      printf("Error: Close playlist failure. Playlist is not open\n");
      return false;
    }

  delete s_player_info.playlist_ins;
  s_player_info.playlist_ins = NULL;

  return true;
}

static bool app_get_next_track(Track* track)
{
  bool ret;

  if (s_player_info.playlist_ins == NULL)
    {
      printf("Error: Get next track failure. Playlist is not open\n");
      return false;
    }

  ret = s_player_info.playlist_ins->getNextTrack(track);

  return ret;
}

static void app_input_device_callback(uint32_t size)
{
    /* do nothing */
}

static bool app_init_simple_fifo(void)
{
  if (CMN_SimpleFifoInitialize(&s_player_info.fifo.handle,
                               s_player_info.fifo.fifo_area,
                               SIMPLE_FIFO_BUF_SIZE, NULL) != 0)
    {
      printf("Error: Fail to initialize simple FIFO.");
      return false;
    }
  CMN_SimpleFifoClear(&s_player_info.fifo.handle);

  s_player_info.fifo.input_device.simple_fifo_handler = (void*)(&s_player_info.fifo.handle);
  s_player_info.fifo.input_device.callback_function = app_input_device_callback;

  return true;
}

static int app_push_simple_fifo(int fd)
{
  int ret;

  ret = read(fd, &s_player_info.fifo.read_buf, WRITE_SIMPLE_FIFO_SIZE);
  if (ret < 0)
    {
      printf("Error: Fail to read file. errno:%d\n", get_errno());
      return FIFO_RESULT_ERR;
    }

  if (CMN_SimpleFifoOffer(&s_player_info.fifo.handle, (const void*)(s_player_info.fifo.read_buf), ret) == 0)
    {
      return FIFO_RESULT_FUL;
    }
  s_player_info.file.size = (s_player_info.file.size - ret);
  if (s_player_info.file.size == 0)
    {
      return FIFO_RESULT_EOF;
    }
  return FIFO_RESULT_OK;
}

static bool app_first_push_simple_fifo(int fd)
{
  int i;
  int ret = 0;

  for(i = 0; i < SIMPLE_FIFO_FRAME_NUM-1; i++)
    {
      if ((ret = app_push_simple_fifo(fd)) != FIFO_RESULT_OK)
        {
          break;
        }
    }

  return (ret != FIFO_RESULT_ERR) ? true : false;
}

static bool app_refill_simple_fifo(int fd)
{
  int32_t ret;
  size_t  vacant_size;

  vacant_size = CMN_SimpleFifoGetVacantSize(&s_player_info.fifo.handle);

  if ((vacant_size != 0) && (vacant_size > WRITE_SIMPLE_FIFO_SIZE))
    {
      int cnt = 1;
      if (vacant_size > WRITE_SIMPLE_FIFO_SIZE*3)
        {
          cnt = 3;
        }
      else if (vacant_size > WRITE_SIMPLE_FIFO_SIZE*2)
        {
          cnt = 2;
        }
      
      for (int i = 0; i < cnt; i++)
        {
          if ((ret = app_push_simple_fifo(fd)) != FIFO_RESULT_OK)
            {
              break;
            }
        }
    }

  return (ret != FIFO_RESULT_ERR) ? true : false;
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
  ids.player_main = MSGQ_AUD_PLY;
  ids.player_sub  = 0xFF;
  ids.mixer       = MSGQ_AUD_OUTPUT_MIX;
  ids.recorder    = 0xFF;
  ids.effector    = 0xFF;
  ids.recognizer  = 0xFF;

  AS_ActivateAudioSubSystem(ids);

  /* Set callback function of attention message */

  if (!app_init_attention())
    {
      printf("Error: app_init_attention() failure.\n");
      return 1;
    }


  AsActPlayerParam_t player_act_param;
  player_act_param.msgq_id.player = MSGQ_AUD_PLY;
  player_act_param.msgq_id.mng    = MSGQ_AUD_MGR;
  player_act_param.msgq_id.mixer  = MSGQ_AUD_OUTPUT_MIX;
  player_act_param.msgq_id.dsp    = MSGQ_AUD_DSP;
  player_act_param.pool_id.es     = DEC_ES_MAIN_BUF_POOL;
  player_act_param.pool_id.pcm    = REND_PCM_BUF_POOL;
  player_act_param.pool_id.dsp    = DEC_APU_CMD_POOL;

  result = AS_ActivatePlayer(&player_act_param);

  if (!result)
    {
      printf("Error: AS_ActivatePlayer() failure. system memory insufficient!\n");
      return false;
    }

  /* Activate mixer feature. */

  AsActOutputMixParam_t output_mix_act_param;
  output_mix_act_param.msgq_id.mixer = MSGQ_AUD_OUTPUT_MIX;
  output_mix_act_param.msgq_id.mng   = MSGQ_AUD_MGR;

  result = AS_ActivateOutputMix(&output_mix_act_param);
  if (!result)
    {
      printf("Error: AS_ActivateOutputMix() failed. system memory insufficient!\n");
      return false;
    }

  /* Activate renderer feature. */

  AsActRendererParam_t renderer_act_param;
  renderer_act_param.msgq_id.dev0_req  = MSGQ_AUD_RND_PLY;
  renderer_act_param.msgq_id.dev0_sync = MSGQ_AUD_RND_PLY_SYNC;
  renderer_act_param.msgq_id.dev1_req  = 0xFF;
  renderer_act_param.msgq_id.dev1_sync = 0xFF;

  result = AS_ActivateRenderer(&renderer_act_param);
  if (!result)
    {
      printf("Error: AS_ActivateRenderer() failure. system memory insufficient!\n");
      return false;
    }

  return true;
}

static void app_deact_audio_sub_system(void)
{
  AS_DeactivateAudioSubSystem();
  AS_DeactivatePlayer();
  AS_DeactivateOutputMix();
  AS_DeactivateRenderer();
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

static bool app_init_output_select(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_INITOUTPUTSELECT;
  command.header.command_code  = AUDCMD_INITOUTPUTSELECT;
  command.header.sub_code      = 0;
  command.init_output_select_param.output_device_sel = AS_OUT_SP;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_set_volume(int master_db)
{
    AudioCommand command;
    command.header.packet_length = LENGTH_SETVOLUME;
    command.header.command_code  = AUDCMD_SETVOLUME;
    command.header.sub_code      = 0;
    command.set_volume_param.input1_db = 0;
    command.set_volume_param.input2_db = 0;
    command.set_volume_param.master_db = master_db;
    AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_set_player_status(void)
{
    AudioCommand command;
    command.header.packet_length = LENGTH_SET_PLAYER_STATUS;
    command.header.command_code = AUDCMD_SETPLAYERSTATUS;
    command.header.sub_code = 0x00;
    command.set_player_sts_param.active_player  = AS_ACTPLAYER_MAIN;
    command.set_player_sts_param.input_device   = AS_SETPLAYER_INPUTDEVICE_RAM;
    command.set_player_sts_param.ram_handler    = &s_player_info.fifo.input_device;
    command.set_player_sts_param.output_device  = AS_SETPLAYER_OUTPUTDEVICE_SPHP;
    command.set_player_sts_param.output_device_handler     = 0x00;
    command.set_player_sts_param.input_device_sub          = 0x00;
    command.set_player_sts_param.ram_handler_sub           = NULL;
    command.set_player_sts_param.output_device_sub         = 0x00;
    command.set_player_sts_param.output_device_handler_sub = 0x00;
    AS_SendAudioCommand(&command);

    AudioResult result;
    AS_ReceiveAudioResult(&result);
    return printAudCmdResult(command.header.command_code, result);
}

static int app_init_player(uint8_t codec_type,
                           uint32_t sampling_rate,
                           uint8_t channel_number)
{
    AudioCommand command;
    command.header.packet_length = LENGTH_INIT_PLAYER;
    command.header.command_code  = AUDCMD_INITPLAYER;
    command.header.sub_code      = 0x00;
    command.init_player_param.codec_type     = codec_type;
    command.init_player_param.bit_length     = AS_BITLENGTH_16;
    command.init_player_param.channel_number = channel_number;
    command.init_player_param.sampling_rate  = sampling_rate;
    AS_SendAudioCommand(&command);

    AudioResult result;
    AS_ReceiveAudioResult(&result);
    return printAudCmdResult(command.header.command_code, result);
}

static int app_play_player(void)
{
    AudioCommand command;
    command.header.packet_length = LENGTH_PLAY_PLAYER;
    command.header.command_code  = AUDCMD_PLAYPLAYER;
    command.header.sub_code      = 0x00;
    AS_SendAudioCommand(&command);

    AudioResult result;
    AS_ReceiveAudioResult(&result);
    return printAudCmdResult(command.header.command_code, result);
}

static bool app_stop_player(void)
{
    AudioCommand command;
    command.header.packet_length = LENGTH_STOP_PLAYER;
    command.header.command_code  = AUDCMD_STOPPLAYER;
    command.header.sub_code      = 0x00;
    command.stop_player_param.stop_mode = AS_STOPPLAYER_NORMAL; // todo: comment
    AS_SendAudioCommand(&command);

    AudioResult result;
    AS_ReceiveAudioResult(&result);
    return printAudCmdResult(command.header.command_code, result);
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

  const NumLayout layout_no = MEM_LAYOUT_PLAYER_MAIN_ONLY;
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

static int app_play_file_open(FAR const char *file_path, FAR int32_t *file_size)
{
  int fd = open(file_path, O_RDONLY);

  *file_size = 0;
  if (fd >= 0)
    {
      struct stat stat_buf;
      if (stat(file_path, &stat_buf) == OK)
        {
          *file_size = stat_buf.st_size;
        }
    }

  return fd;
}

static bool app_start(void)
{
  Track track;

  /* Get next track */

  if (!app_get_next_track(&track))
    {
      printf("Error: No more tracks to play.\n");
      app_freq_release();
      return false;
    }

  char full_path[128];
  snprintf(full_path, sizeof(full_path), "%s/%s", CONFIG_AUDIOUTILS_PLAYLIST_PLAYFILE_MOUNTPT, track.title);

  s_player_info.file.fd = app_play_file_open(full_path, &s_player_info.file.size);
  if (s_player_info.file.fd < 0)
    {
      printf("Error: %s open error. check paths and files!\n", full_path);
      app_freq_release();
      return false;
    }
  if (s_player_info.file.size == 0)
    {
      close(s_player_info.file.fd);
      app_freq_release();
      printf("Error: %s file size is abnormal. check files!\n",full_path);
      return false;
    }

  /* Push data to simple fifo */

  if (!app_first_push_simple_fifo(s_player_info.file.fd))
    {
      printf("Error: app_first_push_simple_fifo() failure.\n");
      CMN_SimpleFifoClear(&s_player_info.fifo.handle);
      close(s_player_info.file.fd);
      app_freq_release();
      return false;
    }

  /* Init Player */

  if (!app_init_player(track.codec_type, track.sampling_rate, track.channel_number))
    {
      printf("Error: app_init_player() failure.\n");
      CMN_SimpleFifoClear(&s_player_info.fifo.handle);
      close(s_player_info.file.fd);
      app_freq_release();
      return false;
    }

  /* Play Player */

  if (!app_play_player())
    {
      printf("Error: app_play_player() failure.\n");
      CMN_SimpleFifoClear(&s_player_info.fifo.handle);
      close(s_player_info.file.fd);
      app_freq_release();
      return false;
    }

  return true;
}

static bool app_stop(void)
{
  bool result = true;

  if (!app_stop_player())
    {
      printf("Error: app_stop_player() failure.\n");
      result = false;
    }

  if (close(s_player_info.file.fd) != 0)
    {
      printf("Error: close() failure.\n");
      result = false;
    }

  return result;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
extern "C" int main(int argc, FAR char *argv[])
#else
extern "C" int player_main(int argc, char *argv[])
#endif
{
  printf("Start AudioPlayer example\n");

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

  /* On and after this point, AudioSubSystem must be active.
   * Register the callback function to be notified when a problem occurs.
   */

  /* Open directory of play contents. */

  if (!app_open_contents_dir())
    {
      printf("Error: app_open_contents_dir() failure.\n");
      return 1;
    }

  /* Change AudioSubsystem to Ready state so that I/O parameters can be changed. */

  if (!app_power_on())
    {
      printf("Error: app_power_on() failure.\n");
      return 1;
    }

  /* Open playlist. */

  if (!app_open_playlist())
    {
      printf("Error: app_open_playlist() failure.\n");
      return 1;
    }

  /* Initialize simple fifo. */

  if (!app_init_simple_fifo())
    {
      printf("Error: app_init_simple_fifo() failure.\n");
      return false;
    }

  /* Set the device to output the mixed audio. */

  if (!app_init_output_select())
    {
      printf("Error: app_init_output_select() failure.\n");
      return 1;
    }

  /* Set player operation mode. */

  if (!app_set_player_status())
    {
      printf("Error: app_set_player_status() failure.\n");
      return 1;
    }

  /* Cancel output mute. */

  app_set_volume(PLAYER_DEF_VOLUME);

  if (board_external_amp_mute_control(false) != OK)
    {
      printf("Error: board_external_amp_mute_control(false) failuer.\n");
      return 1;
    }

  /* Initialize frequency lock parameter. */

  app_init_freq_lock();

  /* Lock cpu frequency to high. */

  app_freq_lock();

  /* Start player operation. */

  if (!app_start())
    {
      printf("Error: app_start_player() failure.\n");
      return 1;
    }

  /* Running... */

  printf("Running time is %d sec\n", PLAYER_PLAY_TIME);
  for(int i = 0; i < PLAYER_PLAY_TIME * 100; i++)
    {
      /* Check the FIFO every 10 ms and fill if there is space. */
      usleep(10 * 1000);
      if (!app_refill_simple_fifo(s_player_info.file.fd))
        {
          break;
        }
    }

  /* Stop player operation. */

  if (!app_stop())
    {
      printf("Error: app_stop() failure.\n");
      return 1;
    }

  /* Unlock cpu frequency. */

  app_freq_release();

  /* Set output mute. */

  if (board_external_amp_mute_control(true) != OK)
    {
      printf("Error: board_external_amp_mute_control(true) failuer.\n");
      return 1;
    }

  /* Close playlist. */

  if (!app_close_playlist())
    {
      printf("Error: app_close_playlist() failure.\n");
      return 1;
    }

  if (!app_close_contents_dir())
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

  printf("Exit AudioPlayer example\n");

  return 0;
}
