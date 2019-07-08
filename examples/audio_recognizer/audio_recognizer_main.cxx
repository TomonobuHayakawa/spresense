/****************************************************************************
 * audio_recognizer/audio_recognizer_main.cxx
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#include <sdk/config.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>
#include <asmp/mpshm.h>
#include <sys/stat.h>

#include "memutils/os_utils/chateau_osal.h"
#include "memutils/simple_fifo/CMN_SimpleFifo.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"
#include "audio/audio_high_level_api.h"
#include <audio/utilities/wav_containerformat.h>
#include "include/msgq_id.h"
#include "include/mem_layout.h"
#include "include/memory_layout.h"
#include "include/msgq_pool.h"
#include "include/pool_layout.h"
#include "include/fixed_fence.h"
#include "rcgproc_command.h"
#ifdef CONFIG_EXAMPLES_AUDIO_RECOGNIZER_USEPREPROC
#include "userproc_command.h"
#endif /* CONFIG_EXAMPLES_AUDIO_RECOGNIZER_USEPREPROC */

#include <arch/chip/cxd56_audio.h>

using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RECFILE_ROOTPATH "/mnt/sd0/REC"
#define DSPBIN_PATH "/mnt/sd0/BIN"

/* Use microphone channel number.
 *   [Analog microphone]
 *       Maximum number: 4
 *       The channel number is 1/2/4
 *   [Digital microphone]
 *       Maximum number: 8
 *       The channel number is 1/2/4/6/8
 */

/* Recognizing time(sec). */

#define RECOGNIZER_EXEC_TIME 10

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/* Recognizing parameter. */

enum sampling_rate_e
{
  SAMPLING_RATE_8K = 0,
  SAMPLING_RATE_16K,
  SAMPLING_RATE_48K,
  SAMPLING_RATE_192K,
  SAMPLING_RATE_NUM
};

enum channel_type_e
{
  CHAN_TYPE_MONO = 0,
  CHAN_TYPE_STEREO,
  CHAN_TYPE_4CH,
  CHAN_TYPE_6CH,
  CHAN_TYPE_8CH,
  CHAN_TYPE_NUM
};

enum bitwidth_e
{
  BITWIDTH_16BIT = 0,
  BITWIDTH_24BIT,
  BITWIDTH_32BIT
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* For share memory. */

static mpshm_t s_shm;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
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

static void app_attention_callback(const ErrorAttentionParam *attparam)
{
  printf("Attention!! %s L%d ecode %d subcode %d\n",
          attparam->error_filename,
          attparam->line_number,
          attparam->error_code,
          attparam->error_att_sub_code);
}

static void recognizer_find_callback(const uint8_t param[7])
{
  printf("app:recognizer_find_callback %d %d %d %d %d %d %d\n", param[0], param[1], param[2], param[3], param[4], param[5], param[6]);
}

static bool app_create_audio_sub_system(void)
{
  bool result = false;

  /* Create manager of AudioSubSystem. */

  AudioSubSystemIDs ids;
  ids.app         = MSGQ_AUD_APP;
  ids.mng         = MSGQ_AUD_MNG;
  ids.player_main = 0xFF;
  ids.player_sub  = 0xFF;
  ids.micfrontend = MSGQ_AUD_FRONTEND;
  ids.mixer       = 0xFF;
  ids.recorder    = 0xFF;
  ids.effector    = 0xFF;
  ids.recognizer  = MSGQ_AUD_RECOGNIZER;

  AS_CreateAudioManager(ids, app_attention_callback);

  /* Create Frontend. */

  AsCreateMicFrontendParams_t frontend_create_param;
  frontend_create_param.msgq_id.micfrontend = MSGQ_AUD_FRONTEND;
  frontend_create_param.msgq_id.mng         = MSGQ_AUD_MNG;
  frontend_create_param.msgq_id.dsp         = MSGQ_AUD_PREDSP;
  frontend_create_param.pool_id.input       = S0_MIC_IN_BUF_POOL;
#ifdef CONFIG_EXAMPLES_AUDIO_RECOGNIZER_USEPREPROC
  frontend_create_param.pool_id.output      = S0_PREPROC_BUF_POOL;
#else
  frontend_create_param.pool_id.output      = S0_NULL_POOL;
#endif
  frontend_create_param.pool_id.dsp         = S0_PRE_APU_CMD_POOL;

  AS_CreateMicFrontend(&frontend_create_param, NULL);

  /* Create Recognizer. */

  AsCreateRecognizerParam_t recognizer_create_param;
  recognizer_create_param.msgq_id.recognizer = MSGQ_AUD_RECOGNIZER;
  recognizer_create_param.msgq_id.mng        = MSGQ_AUD_MNG;
  recognizer_create_param.msgq_id.dsp        = MSGQ_AUD_RCGDSP;
  recognizer_create_param.pool_id.wuwsr_in   = S0_OUTPUT_BUF_POOL;
  recognizer_create_param.pool_id.dsp        = S0_RCG_APU_CMD_POOL;

  result = AS_CreateRecognizer(&recognizer_create_param, NULL);
  if (!result)
    {
      printf("Error: AS_CreateRecognizer() failure. system memory insufficient!\n");
      return false;
    }

  /* Create Capture feature. */

  AsCreateCaptureParam_t capture_create_param;
  capture_create_param.msgq_id.dev0_req  = MSGQ_AUD_CAP;
  capture_create_param.msgq_id.dev0_sync = MSGQ_AUD_CAP_SYNC;
  capture_create_param.msgq_id.dev1_req  = 0xFF;
  capture_create_param.msgq_id.dev1_sync = 0xFF;

  result = AS_CreateCapture(&capture_create_param);
  if (!result)
    {
      printf("Error: As_CreateCapture() failure. system memory insufficient!\n");
      return false;
    }

  return true;
}

static void app_deact_audio_sub_system(void)
{
  AS_DeleteAudioManager();
  AS_DeleteRecognizer();
  AS_DeleteMicFrontend();
  AS_DeleteCapture();
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
  command.init_mic_gain_param.mic_gain[0] = 0;
  command.init_mic_gain_param.mic_gain[1] = 0;
  command.init_mic_gain_param.mic_gain[2] = 0;
  command.init_mic_gain_param.mic_gain[3] = 0;
  command.init_mic_gain_param.mic_gain[4] = 0;
  command.init_mic_gain_param.mic_gain[5] = 0;
  command.init_mic_gain_param.mic_gain[6] = 0;
  command.init_mic_gain_param.mic_gain[7] = 0;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

#ifdef CONFIG_EXAMPLES_AUDIO_RECOGNIZER_USEPREPROC
static bool app_set_preprocess_type(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SETMFETYPE;
  command.header.command_code  = AUDCMD_SETMFETYPE;
  command.header.sub_code      = 0x00;
  command.set_mfetype_param.preproc_type = AsMicFrontendPreProcUserCustom;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}
#endif /* CONFIG_EXAMPLES_AUDIO_RECOGNIZER_USEPREPROC */

static bool app_set_recognizer_type(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SET_RECOGNIZERTYPE;
  command.header.command_code  = AUDCMD_SETRECOGNIZETYPE;
  command.header.sub_code      = 0x00;
  command.set_recognizer_type.recognizer_type = AsRecognizerTypeUserCustom;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_set_recognizer_status(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_SET_RECOGNIZER_STATUS;
  command.header.command_code  = AUDCMD_SETRECOGNIZERSTATUS;
  command.header.sub_code      = 0x00;
  command.set_recognizer_status_param.input_device = AsMicFrontendDeviceMic;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_init_recognizer(sampling_rate_e sampling_rate,
                                channel_type_e ch_type,
                                bitwidth_e bitwidth)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_INIT_RECOGNIZER;
  command.header.command_code  = AUDCMD_INITRECOGNIZER;
  command.header.sub_code      = 0x00;
  command.init_recognizer.ch_num     = AS_CHANNEL_MONO;
  command.init_recognizer.bit_length = AS_BITLENGTH_16;
  command.init_recognizer.samples    = 320;
  command.init_recognizer.fcb        = recognizer_find_callback;

  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_start_recognizer(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_START_RECOGNIZER;
  command.header.command_code  = AUDCMD_STARTRECOGNIZER;
  command.header.sub_code      = 0x00;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_stop_recognizer(void)
{
  AudioCommand command;
  command.header.packet_length = LENGTH_STOP_RECOGNIZER;
  command.header.command_code  = AUDCMD_STOPRECOGNIZER;
  command.header.sub_code      = 0x00;
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  if (!printAudCmdResult(command.header.command_code, result))
    {
      return false;
    }

  return true;
}

#ifdef CONFIG_EXAMPLES_AUDIO_RECOGNIZER_USEPREPROC
static bool app_init_mfe(void)
{
  static InitParam s_initparam;

  AudioCommand command;
  command.header.packet_length = LENGTH_INITMFE;
  command.header.command_code  = AUDCMD_INITMFE;
  command.header.sub_code      = 0x00;
  command.init_mfe_param.initpre_param.packet_addr = reinterpret_cast<uint8_t *>(&s_initparam);
  command.init_mfe_param.initpre_param.packet_size = sizeof(s_initparam);
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_set_mfe(void)
{
  static SetParam s_setparam;

  s_setparam.enable = true;
  s_setparam.coef   = 99;

  AudioCommand command;
  command.header.packet_length = LENGTH_SETMFE;
  command.header.command_code  = AUDCMD_SETMFE;
  command.header.sub_code      = 0x00;
  command.init_mfe_param.initpre_param.packet_addr = reinterpret_cast<uint8_t *>(&s_setparam);
  command.init_mfe_param.initpre_param.packet_size = sizeof(s_setparam);
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}
#endif /* CONFIG_EXAMPLES_AUDIO_RECOGNIZER_USEPREPROC */

static bool app_init_rcgproc(void)
{
  static InitRcgParam s_initrcgparam;

  AudioCommand command;
  command.header.packet_length = LENGTH_INIT_RECOGNIZERPROC;
  command.header.command_code  = AUDCMD_INITRECOGNIZERPROC;
  command.header.sub_code      = 0x00;
  command.init_rcg_param.packet_addr = reinterpret_cast<uint8_t *>(&s_initrcgparam);
  command.init_rcg_param.packet_size = sizeof(s_initrcgparam);
  AS_SendAudioCommand(&command);

  AudioResult result;
  AS_ReceiveAudioResult(&result);
  return printAudCmdResult(command.header.command_code, result);
}

static bool app_set_rcgproc(void)
{
  static SetRcgParam s_setrcgparam;

  AudioCommand command;
  command.header.packet_length = LENGTH_SET_RECOGNIZERPROC;
  command.header.command_code  = AUDCMD_SETRECOGNIZERPROC;
  command.header.sub_code      = 0x00;
  command.init_rcg_param.packet_addr = reinterpret_cast<uint8_t *>(&s_setrcgparam);
  command.init_rcg_param.packet_size = sizeof(s_setrcgparam);
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

  const NumLayout layout_no = MEM_LAYOUT_RECOGNIZER;
  void* work_va = translatePoolAddrToVa(S0_MEMMGR_WORK_AREA_ADDR);
  const PoolSectionAttr *ptr  = &MemoryPoolLayouts[SECTION_NO0][layout_no][0];
  err = Manager::createStaticPools(SECTION_NO0,
                                   layout_no,
                                   work_va,
                                   S0_MEMMGR_WORK_AREA_SIZE,
                                   ptr);
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

void app_recognizer_process(uint32_t rec_time)
{
  /* Timer Start */
  time_t start_time;
  time_t cur_time;

  time(&start_time);

  do
    {
      usleep(500 * 1000);
    } while((time(&cur_time) - start_time) < rec_time);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
extern "C" int main(int argc, FAR char *argv[])
#else
extern "C" int recognizer_main(int argc, char *argv[])
#endif
{
  printf("Start AudioRecognizer example\n");

  
  /* First, initialize the shared memory and memory utility used by AudioSubSystem. */

  if (!app_init_libraries())
    {
      printf("Error: init_libraries() failure.\n");
      return 1;
    }

  /* Next, Create the features used by AudioSubSystem. */

  if (!app_create_audio_sub_system())
    {
      printf("Error: act_audiosubsystem() failure.\n");
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

  /* Set the initial gain of the microphone to be used. */

  if (!app_init_mic_gain())
    {
      printf("Error: app_init_mic_gain() failure.\n");
      return 1;
    }

#ifdef CONFIG_EXAMPLES_AUDIO_RECOGNIZER_USEPREPROC
  /* Set preprocess type. */

  if (!app_set_preprocess_type())
    {
      printf("Error: app_set_preprocess_type() failure.\n");
      return 1;
    }
#endif /* CONFIG_EXAMPLES_AUDIO_RECOGNIZER_USEPREPROC */

  /* Set recognizer type. */

  if (!app_set_recognizer_type())
    {
      printf("Error: app_set_recognizer_type() failure.\n");
      return 1;
    }

  /* Set recognizer operation mode. */

  if (!app_set_recognizer_status())
    {
      printf("Error: app_set_recognizer_status() failure.\n");
      return 1;
    }

  /* Initialize recognizer. */

  if (!app_init_recognizer(SAMPLING_RATE_48K,
                           CHAN_TYPE_MONO,
                           BITWIDTH_16BIT))
    {
      printf("Error: app_init_recognizer() failure.\n");
      return 1;
    }

#ifdef CONFIG_EXAMPLES_AUDIO_RECOGNIZER_USEPREPROC
  /* Init PREPROC */

  if (!app_init_mfe())
    {
      printf("Error: app_init_mfe() failure.\n");
      return 1;
    }

  /* Set PREPROC */

  if (!app_set_mfe())
    {
      printf("Error: app_set_mfe() failure.\n");
      return 1;
    }
#endif /* CONFIG_EXAMPLES_AUDIO_RECOGNIZER_USEPREPROC */

  /* Init RCGPROC */

  if (!app_init_rcgproc())
    {
      printf("Error: app_init_rcgproc() failure.\n");
      return 1;
    }

  /* Set RCGPROC */

  if (!app_set_rcgproc())
    {
      printf("Error: app_set_rcgproc() failure.\n");
      return 1;
    }

  /* Start recognizer operation. */

  if (!app_start_recognizer())
    {
      printf("Error: app_start_recognizer() failure.\n");
      return 1;
    }

  /* Running... */

  printf("Running time is %d sec\n", RECOGNIZER_EXEC_TIME);

  app_recognizer_process(RECOGNIZER_EXEC_TIME);

  /* Stop recognizer operation. */

  if (!app_stop_recognizer())
    {
      printf("Error: app_stop_recognizer() failure.\n");
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

  printf("Exit AudioRecognizer example\n");

  return 0;
}