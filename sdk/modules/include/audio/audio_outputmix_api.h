/****************************************************************************
 * modules/include/audio/audio_outputmix_api.h
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/** Message queue ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Message queue id of output mixer */

  uint8_t mixer;

  /*! \brief [in] Message queue id of audio_manager */

  uint8_t mng;
} AsOutputMixMsgQueId_t;

/** Activate function parameter */

typedef struct
{
  /*! \brief [in] ID for sending messages to each function */

  AsOutputMixMsgQueId_t msgq_id;
} AsActOutputMixParam_t;

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
 * @brief Activate audio output mixer
 *
 * @param[in] param: Parameters of resources used by output mixer
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_ActivateOutputMix(FAR AsActOutputMixParam_t *param);

/**
 * @brief Deactivate output mixer
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeactivateOutputMix(void);

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
