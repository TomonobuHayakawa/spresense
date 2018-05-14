/****************************************************************************
 * modules/include/audio/audio_recorder_api.h
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

#ifndef __SONY_APPS_INCLUDE_AUDIOUTIL_AUDIO_RECORDER_API_H
#define __SONY_APPS_INCLUDE_AUDIOUTIL_AUDIO_RECORDER_API_H

/**
 * @defgroup audioutils Audio Utility
 * @{
 */

/**
 * @defgroup audioutils_audio_recorder_api Audio Recorder API
 * @{
 *
 * @file       audio_recorder_api.h
 * @brief      Spritzer Audio Recorder API
 * @author     Spritzer Audio SW Team
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AS_FEATURE_RECORDER_ENABLE

/** @name Packet length of command*/
/** @{ */

/*! \brief StartRec command (#AUDCMD_STARTREC) packet length */

#define LENGTH_START_RECORDER    2

/*! \brief StopRec command (#AUDCMD_STOPREC) packet length */

#define LENGTH_STOP_RECORDER     2

/*! \brief InitRecorder command (#AUDCMD_INITREC) packet length */

#define LENGTH_INIT_RECORDER    10 

/** @} */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* SetRecorderStatus */

/** Select Recorder input device */

typedef enum
{
  /*! \brief CXD5247 AMIC */

  AS_SETRECDR_STS_INPUTDEVICE_MIC_A = 0,

  /*! \brief CXD5247 DMIC */

  AS_SETRECDR_STS_INPUTDEVICE_MIC_D,

  /*! \brief I2S Input */

  AS_SETRECDR_STS_INPUTDEVICE_I2S_IN,
  AS_SETRECDR_STS_INPUTDEVICE_NUM
} AsSetRecorderStsInputDevice;

/** Select Recorder output device */

typedef enum
{
  /*! \brief eMMC FileSystem (__not supported__) */

  AS_SETRECDR_STS_OUTPUTDEVICE_EMMC = 0,

  /*! \brief RAM */

  AS_SETRECDR_STS_OUTPUTDEVICE_RAM,
  AS_SETRECDR_STS_OUTPUTDEVICE_NUM
} AsSetRecorderStsOutputDevice;

/* InitRecorder */

/** Select InitRecorder computational complexity(Opus Only) */

typedef enum
{
  /*! \brief Complexity = 0(lowest complexity) */

  AS_INITREC_COMPLEXITY_0 = 0,

  /*! \brief Complexity = 1 */

  AS_INITREC_COMPLEXITY_1,

  /*! \brief Complexity = 2 */

  AS_INITREC_COMPLEXITY_2,

  /*! \brief Complexity = 2 */

  AS_INITREC_COMPLEXITY_3,

  /*! \brief Complexity = 2 */

  AS_INITREC_COMPLEXITY_4,

  /*! \brief Complexity = 2 */

  AS_INITREC_COMPLEXITY_5,

  /*! \brief Complexity = 2 */

  AS_INITREC_COMPLEXITY_6,

  /*! \brief Complexity = 2 */

  AS_INITREC_COMPLEXITY_7,

  /*! \brief Complexity = 2 */

  AS_INITREC_COMPLEXITY_8,

  /*! \brief Complexity = 2 */

  AS_INITREC_COMPLEXITY_9,

  /*! \brief Complexity = 10 (highest complexity) */

  AS_INITREC_COMPLEXITY_10,
  AS_INITREC_COMPLEXITY_NUM
} AsInitRecorderComputationalComplexity;

/* for AsRecorderOutputDeviceHdlr */

/** SimpliFifo Callback function
 * @param[in] size :  Set write size after writing data to SimpleFifo
 */

typedef void (*AudioSimpleFifoWriteDoneCallbackFunction)(uint32_t size);

/** internal of output_device_handler
 * (used in AsSetRecorderStatusParam) parameter
 */

typedef struct
{
  /*! \brief [in] Set SimpleFifo handler
   *
   * Use CMN_SimpleFifoHandle (refer to include file)
   */

  void *simple_fifo_handler;

  /*! \brief [in] Set callback function
   *
   * Call this function when SimpleFifo was read
   */

  AudioSimpleFifoWriteDoneCallbackFunction callback_function;
} AsRecorderOutputDeviceHdlr;

/** SetRecorderStatus Command (#AUDCMD_SETRECORDERSTATUS) parameter */

typedef struct
{
  /*! \brief [in] Select Recorder input device
   *
   * Use #AsSetRecorderStsInputDevice enum type
   */

  uint8_t  input_device;

  /*! \brief [in] Select Recorder output device
   *
   * Use #AsSetRecorderStsOutputDevice enum type
   */

  uint8_t  output_device;

  /*! \brief [in] reserved */

  uint8_t  reserved1;

  /*! \brief [in] reserved */

  uint8_t  reserved2;

  /*! \brief [in] Set Recorder input device handler, T.B.D. */

  uint32_t input_device_handler;

  /*! \brief [in] Set Recorder output device handler, refer following. */

  AsRecorderOutputDeviceHdlr*  output_device_handler;
} AsSetRecorderStatusParam;

/** InitRecorder Command (#AUDCMD_INITREC) parameter */

typedef struct
{
  /*! \brief [in] Select sampling rate for recorded data
   *
   * Use #AsInitRecorderSamplingRateIndex enum type
   */

  uint32_t sampling_rate;

  /*! \brief [in] Select InitRecorder input channels
   *
   * Use #AsInitRecorderChannelNumberIndex enum type
   */

  uint8_t  channel_number;

  /*! \brief [in] Select InitRecorder input bit length
   *
   * Use #AsInitRecorderBitLength enum type
   */

  uint8_t  bit_length;

  /*! \brief [in] Select InitRecorder codec type
   *
   * Use #AsInitRecorderCodecType enum type
   */

  uint8_t  codec_type;

  /*! \brief [in] Select InitRecorder computational complexity
   *
   * Use #AsInitRecorderComputationalComplexity enum type
   */

  uint8_t  computational_complexity;

  /*! \brief [in] Select InitRecorder bitrate
   *
   * Use #AsInitRecorderBitrate enum type
   */

  uint32_t bitrate;

  /*! \brief [in] DSP path 
   */

  char dsp_path[AS_AUDIO_DSP_PATH_LEN];

} AsInitRecorderParam;

/** Message queue ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Message queue id of recorder */

  uint8_t recorder;

  /*! \brief [in] Message queue id of audio_manager */

  uint8_t mng;

  /*! \brief [in] Message queue id of DSP */

  uint8_t dsp;
} AsRecorderMsgQueId_t;

/** Pool ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Memory pool id of input data */

  uint8_t input;

  /*! \brief [in] Memory pool id of output data */

  uint8_t output;

  /*! \brief [in] Memory pool id of dsp command data */

  uint8_t dsp;
} AsRecorderPoolId_t;

/** Activate function parameter */

typedef struct
{
  /*! \brief [in] ID for sending messages to each function */

  AsRecorderMsgQueId_t msgq_id;

  /*! \brief [in] ID of memory pool for processing data */

  AsRecorderPoolId_t   pool_id;
} AsActRecorderParam_t;

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
 * @brief Activate audio recorder
 *
 * @param[in] param: Parameters of resources used by audio recorder
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_CreateVoiceRecorder(FAR AsActRecorderParam_t *param);

/**
 * @brief Deactivate audio recorder
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeleteVoiceRecorder(void);

#ifdef __cplusplus
}
#endif

#endif  /* __SONY_APPS_INCLUDE_AUDIOUTIL_AUDIO_RECORDER_API_H */
/**
 * @}
 */

/**
 * @}
 */
