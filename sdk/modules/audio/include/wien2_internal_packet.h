/****************************************************************************
 * modules/audio/include/wien2_internal_packet.h
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

#ifndef __MODULES_AUDIO_INCLUDE_WIEN2_INTERNAL_PACKET_H
#define __MODULES_AUDIO_INCLUDE_WIEN2_INTERNAL_PACKET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "wien2_common_defs.h"
#include "memutils/memory_manager/MemHandle.h"
#include "audio/audio_high_level_api.h"
#include "apus/apu_cmd.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct AudioMngCmdCmpltResult
{
  /* TODO: 直接にリザルトコードを返すかどうか、迷い中 */
  /* Sub-stateも一緒にobject layer->audio managerに返してあげたほうがよいかも */
  uint8_t    command_code;  /**< command code of high level API */
  uint8_t    sub_code;      /**< sub code of high level API */
  uint8_t    module_id;     /**< module id of high level API */
  uint8_t    reserved;
  uint32_t   result;        /**< result code of high level API */
  uint32_t   sub_result;

  AudioMngCmdCmpltResult():
    command_code(0xFF),
    sub_code(0xFF),
    module_id(0xFF),
    result(0xFFFFFFFF),
    sub_result(0xFFFFFFFF)
  {}

  AudioMngCmdCmpltResult(uint8_t arg_command_code,
                         uint8_t arg_sub_code,
                         uint32_t arg_result,
                         uint8_t arg_module_id,
                         uint32_t arg_sub_result = 0):
    command_code(arg_command_code),
    sub_code(arg_sub_code),
    module_id(arg_module_id),
    result(arg_result),
    sub_result(arg_sub_result)
  {}
};

/*--------------------------------------------------------------------*/
/* Internal parameters between output-mix object and its user.        */
/*--------------------------------------------------------------------*/
/**< Output device type of output-mix object. */
enum OutputMixDevice
{
  HPOutputDevice = 0,
  I2SOutputDevice,
  A2dpSrcOutputDevice,
  OutputMixDeviceNum
};

/**< Mixer type of output-mix object. */
enum OutputMixerType
{
  MainOnly = 0,
  SoundEffectOnly,
  MainSoundEffectMix,
  OutputMixerTypeNum
};

/**< Parameters for initializing output-mix object. */
struct OutputMixObjActCmd
{
  OutputMixDevice output_device;
  OutputMixerType mixer_type;

  union
  {
    InitI2SParam as_init_i2s_param; /* Only when output_device equals
                                     * HPOutputDevice or I2SOutputDevice
                                     * need to set this field.
                                     */
  };
};

/**< Parameters for deactivation output-mix object. */
struct OutputMixObjDeactCmd
{
};

/**< Input data for output-mix object task. */
struct OutputMixObjInputDataCmd
{
  int handle;

  MemMgrLite::MemHandle mh; /**< MHandle for saving input data. */
  uint32_t sample;          /**< Sample of input data. */
  uint32_t size;            /**< Byte size of input data. */
  bool is_es_end;           /**< True: is last data of current session. */
                            /**< Otherwise, is not last data of current session. */
  bool is_valid;            /**< True: Valid PCM frame. */

  OutputMixObjInputDataCmd():
    sample(0),
    size(0),
    is_es_end(false),
    is_valid(true)
  {}
};

/**< Parameters for stop output-mix object. */
struct OutputMixObjStopCmd
{
};

/**< Parameters for render done notify to output-mix object. */
struct OutputMixObjRenderDoneCmd
{
  bool end_flag;
};

/**< Setting of sounding period adjustment. */
struct OutputMixObjSoundPeriodAdjust
{
  int8_t   direction;
  uint32_t adjust_times;
};

struct OutputMixObjParam
{
  int handle;

  union
  {
    OutputMixObjActCmd            act_param;
    OutputMixObjDeactCmd          deact_param;
    OutputMixObjStopCmd           stop_param;
    OutputMixObjRenderDoneCmd     renderdone_param;
    OutputMixObjSoundPeriodAdjust adjust_param;
  };
};

/**< Completion of output-mix object task. */
enum OutputMixDoneCmdType
{
  OutputMixActDone = 0,
  OutputMixDeactDone,
  OutputMixExecDone,
  OutputMixStopDone,
  OutputMixDoneCmdTypeNum
};

/**< Completion data from output-mix object task. */
struct OutputMixDoneParam
{
  int handle;
  OutputMixDoneCmdType done_type;
};

struct DspResult
{
  Apu::ExecResult  exec_result;
  Apu::InternalResult  internal_result;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

__WIEN2_END_NAMESPACE

#endif /* __MODULES_AUDIO_INCLUDE_WIEN2_INTERNAL_PACKET_H */

