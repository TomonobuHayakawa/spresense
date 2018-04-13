/****************************************************************************
 * modules/include/audio/audio_outputmix_api.h
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
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

#ifndef __SONY_APPS_INCLUDE_AUDIOUTIL_AUDIO_OUTPUTMIX_API_H
#define __SONY_APPS_INCLUDE_AUDIOUTIL_AUDIO_OUTPUTMIX_API_H

/**
 * @defgroup audioutils Audio Utility
 * @{
 */

/**
 * @defgroup audioutils_audio_outputmix_api Audio OutputMix API
 * @{
 *
 * @file       audio_outputmix_api.h
 * @brief      Spritzer Audio OutputMix API
 * @author     Spritzer Audio SW Team
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "audio/audio_common_defs.h"

#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/MsgPacket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/
enum AsOutputMixerHandle
{
  /*! \brief OutputMixer No.0 */

  OutputMixer0 = 0,

  /*! \brief OutputMixer No.1 */

  OutputMixer1,
};

enum AsOutputMixDevice
{
  /*! \brief Speaker out */

  HPOutputDevice = 0,

  /*! \brief I2S out */

  I2SOutputDevice,

  /*! \brief A2DP out */

  A2dpSrcOutputDevice,

  OutputMixDeviceNum
};

/**< Mixer type of output-mix object. */
enum AsOutputMixerType
{
  /*! \brief Main */

  MainOnly = 0,

  /*! \brief SoundEffet */

  SoundEffectOnly,

  /*! \brief Main & SoundEffet */

  MainSoundEffectMix,

  OutputMixerTypeNum
};

/**< Completion of output-mix object task. */
enum AsOutputMixDoneCmdType
{
  /*! \brief Activation done */

  OutputMixActDone = 0,

  /*! \brief Deactivation done */

  OutputMixDeactDone,

  /*! \brief Set Clock recovery done */

  OutputMixSetClkRcvDone,

  OutputMixDoneCmdTypeNum
};

/** Message queue ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Message queue id of output mixer */

  uint8_t mixer;

} AsOutputMixMsgQueId_t;

/** Activate function parameter */

typedef struct
{
  /*! \brief [in] ID for sending messages to each function */

  AsOutputMixMsgQueId_t msgq_id;

} AsCreateOutputMixParam_t;

/** Activate function parameter */

struct AsOutputMixDoneParam
{
  int handle;

  AsOutputMixDoneCmdType done_type;

};

typedef void (*OutputMixerCallback)(MsgQueId requester_dtq, MsgType msgtype, AsOutputMixDoneParam *param);
typedef void (*OutputMixerErrorCallback)(uint8_t handle);

typedef struct
{
  /*! \brief [in] Output device type
   *
   * Use #AsOutputMixDevice enum type
   */

  uint8_t output_device;

  /*! \brief [in] Mixer typ
   *
   * Use #AsOutputMixerType enum type
   */

  uint8_t mixer_type;

  /*! \brief [in] Done callback */

  OutputMixerCallback cb;

  /*! \brief [in] error callback */

  OutputMixerErrorCallback error_cb;

} AsActivateOutputMixer;

/** Deactivate function parameter */

typedef struct
{
  uint8_t reserve;

} AsDeactivateOutputMixer;

/** Data send function parameter */

typedef struct
{
  /*! \brief [in] Handle of OutputMixer */

  uint8_t handle;

  /*! \brief [in] Send done callback */

  PcmProcDoneCallback callback;

  /*! \brief [in] PCM data parameter */

  AsPcmDataParam pcm;

} AsSendDataOutputMixer;

/** Clock recovery function parameter */

typedef struct
{
  /*! \brief [in] Recovery direction (advance or delay) */

  int8_t   direction;

  /*! \brief [in] Recovery term */

  uint32_t times;

} AsFrameTermFineControl;

/** Clock recovery function parameter */

typedef struct 
{
  uint8_t handle;

  union
  {
    AsActivateOutputMixer   act_param;
    AsDeactivateOutputMixer deact_param;
    AsFrameTermFineControl  fterm_param;
  };

} OutputMixerCommand;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif
/**
 * @brief Create audio output mixer
 *
 * @param[in] param: Parameters of resources used by output mixer
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_CreateOutputMixer(FAR AsCreateOutputMixParam_t *param);

/**
 * @brief Activate audio output mixer
 *
 * @param[in] param: Activation parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_ActivateOutputMixer(uint8_t handle, FAR AsActivateOutputMixer &actparam);

/**
 * @brief Send audio data via outputmixer
 *
 * @param[in] param: Send data parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_SendDataOutputMixer(FAR AsSendDataOutputMixer &sendparam);

/**
 * @brief Set clock recovery parameters
 *
 * @param[in] param: clock recovery parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_FrameTermFineControlOutputMixer(uint8_t handle, FAR AsFrameTermFineControl &ftermparam);

/**
 * @brief Deactivate audio output mixer
 *
 * @param[in] param: Deactivation parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeactivateOutputMixer(uint8_t handle, FAR AsDeactivateOutputMixer &deactparam);

/**
 * @brief Delete output mixer
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeleteOutputMix(void);

#ifdef __cplusplus
}
#endif

#endif  /* __SONY_APPS_INCLUDE_AUDIOUTIL_AUDIO_OUTPUTMIX_API_H */
/**
 * @}
 */

/**
 * @}
 */
