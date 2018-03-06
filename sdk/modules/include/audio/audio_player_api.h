/****************************************************************************
 * modules/include/audio/audio_player_api.h
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

#ifndef __SONY_APPS_INCLUDE_AUDIOUTIL_AUDIO_PLAYER_API_H
#define __SONY_APPS_INCLUDE_AUDIOUTIL_AUDIO_PLAYER_API_H

/**
 * @defgroup audioutils Audio Utility
 * @{
 */

/**
 * @defgroup audioutils_audio_player_api Audio Player API
 * @{
 *
 * @file       audio_player_api.h
 * @brief      Spritzer Audio Player API
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

#define AS_FEATURE_PLAYER_ENABLE

/** @name Packet length of player command*/
/** @{ */

/*! \brief InitPlayer command (#AUDCMD_INITPLAYER) packet length */

#define LENGTH_INIT_PLAYER     (3)

/*! \brief InitSubPlayer command (#AUDCMD_INITSUBPLAYER) packet length */

#define LENGTH_INIT_SUBPLAYER  (LENGTH_INIT_PLAYER)

/*! \brief PlayPlayer command (#AUDCMD_PLAYPLAYER) packet length */

#define LENGTH_PLAY_PLAYER     (2)

/*! \brief PlaySubPlayer command (#AUDCMD_PLAYSUBPLAYER) packet length */

#define LENGTH_PLAY_SUBPLAYER  (LENGTH_PLAY_PLAYER)

/*! \brief StopPlayer command (#AUDCMD_STOPPLAYER) packet length */

#define LENGTH_STOP_PLAYER     (2)

/*! \brief StopSubPlayer command (#AUDCMD_STOPSUBPLAYER) packet length */

#define LENGTH_STOP_SUBPLAYER  (LENGTH_STOP_PLAYER)

/*! \brief ClkRecovery command ("AUDCMD_CLKRECOVERY)packet length */

#define LENGTH_CLK_RECOVERY  (2)

/*! \brief Set audio gain leve command ("AUDCMD_SETGAIN)packet length */

#define LENGTH_SET_GAIN (2)

/** @} */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/** Select activate player */

typedef enum
{
  /*! \brief Activate main player */

  AS_ACTPLAYER_MAIN = 1,

  /*! \brief Activate sub player */

  AS_ACTPLAYER_SUB,

  /*! \brief Activate main & sub player */

  AS_ACTPLAYER_BOTH,
  AS_ACTPLAYER_NUM
} AsSetActivatePlayer;

/** Select Player Input device */

typedef enum
{
  /*! \brief eMMC FileSystem (__not supported__) */

  AS_SETPLAYER_INPUTDEVICE_EMMC = 0,

  /*! \brief A2DP Media Packet FIFO (__not supported__) */

  AS_SETPLAYER_INPUTDEVICE_A2DPFIFO,

  /*! \brief I2S input (__not supported__) */

  AS_SETPLAYER_INPUTDEVICE_I2SINPUT,

  /*! \brief RAM */

  AS_SETPLAYER_INPUTDEVICE_RAM,
  AS_SETPLAYER_INPUTDEVICE_NUM

} AsSetPlayerInputDevice;

/** Select Player Output device */

typedef enum
{
  /*! \brief CXD5247 SP/HP */

  AS_SETPLAYER_OUTPUTDEVICE_SPHP = 0,

  /*! \brief I2S Output */

  AS_SETPLAYER_OUTPUTDEVICE_I2SOUTPUT,

  /*! \brief A2DP Media Packet FIFO (__not supported__) */

  AS_SETPLAYER_OUTPUTDEVICE_A2DPFIFO,
  AS_SETPLAYER_OUTPUTDEVICE_NUM
} AsSetPlayerOutputDevice;

/** Select stop mode */

typedef enum
{
  /*! \brief Normal stop (immediately stop) */

  AS_STOPPLAYER_NORMAL = 0,

  /*! \brief Wait end of es */

  AS_STOPPLAYER_ESEND
} AsStopPlayerStopMode;

/**< Output sound period adjustment direction */

typedef enum
{
  /*! \brief Adjust to the + direction */

  OutputMixAdvance = -1,

  /*! \brief No adjust */

  OutputMixNoAdjust = 0,

  /*! \brief Adjust to the - direction */

  OutputMixDelay = 1,
} AsClkRecoveryDirection;

/* for AsPlayerInputDeviceHdlrForRAM */

/** SimpliFifo Callback function
 * @param[in] size : Set read size after reading the SimpleFifo
 */

typedef void (*AudioSimpleFifoReadDoneCallbackFunction)(uint32_t size);

/** internal of ram_handler (used in AsPlayerInputDeviceHdlr) parameter */

typedef struct
{
  /*! \brief [in] Set SimpleFifo handler
   *
   * Use CMN_SimpleFifoHandle (refer to include file)
   */

  void *simple_fifo_handler;

  /*! \brief [in] Set callback function,
   * Call this function when SimpleFifo was read
   */

  AudioSimpleFifoReadDoneCallbackFunction callback_function;

  /*! \brief [in] Read size notification threshold */

  uint32_t  notification_threshold_size;
} AsPlayerInputDeviceHdlrForRAM;

/** SetPlayerStatus Command (#AUDCMD_SETPLAYERSTATUS) parameter */

#if defined(__CC_ARM)
#pragma anon_unions
#endif

typedef struct
{
  /*! \brief [in] Select activate player
   *
   * Use #AsSetActivatePlayer enum type
   */

  uint8_t  active_player;

  /*! \brief [in] Select Player Input device
   *
   *  Use #AsSetPlayerInputDevice enum type
   */

  uint8_t  input_device;

  /*! \brief [in] Select SubPlayer Input device, same as above. */

  uint8_t  input_device_sub;

  /*! \brief [in] reserved */

  uint8_t  reserved3;

  /*! \brief [in] Set Player Input device handler, refer following. */

  AsPlayerInputDeviceHdlrForRAM* ram_handler;

  /*! \brief [in] Set SubPlayer Input device handler, refer following. */

  AsPlayerInputDeviceHdlrForRAM* ram_handler_sub;

  /*! \brief [in] Select Player Output device
   *
   * Use #AsSetPlayerOutputDevice enum type
   */

  uint8_t  output_device;

  /*! \brief [in] Select SubPlayer Output device, same as above.  */

  uint8_t  output_device_sub;

  /*! \brief [in] reserved */

  uint8_t  reserved5;

  /*! \brief [in] reserved */

  uint8_t  reserved6;

  /*! \brief [in] Set Player Output device handler, T.B.D. */

  uint32_t output_device_handler;

  /*! \brief [in] Set Player Output device sub_handler, T.B.D. */

  uint32_t output_device_handler_sub;

#if !defined(__CC_ARM)
} SetPlayerStsParam ;
#else
} SetPlayerStsParam __attribute__((transparent_union));
#endif

/** InitPlayer Command (#AUDCMD_INITPLAYER, AUDCMD_INITSUBPLAYER) parameter */

typedef struct
{
  /*! \brief [in] Select InitPlayer input channels
   *
   * Use #AsInitPlayerChannelNumberIndex enum type
   */

  uint8_t  channel_number;

  /*! \brief [in] Select InitPlayer input bit length
   *
   * Use #AsInitPlayerBitLength enum type
   */

  uint8_t  bit_length;

  /*! \brief [in] Select InitPlayer codec type
   *
   * Use #AsInitPlayerCodecType enum type
   */

  uint8_t  codec_type;

  /*! \brief [in] reserved */

  uint8_t  reserved1;

  /*! \brief [in] Select sampling rate of es data
   *
   * Use #AsInitPlayerSamplingRateIndex enum type
   */

  uint32_t sampling_rate;
} AsInitPlayerParam;

/** StopPlayer Command (#AUDCMD_STOPPLAYER, #AUDCMD_STOPSUBPLAYER) parameter */

typedef struct
{
  /*! \brief [in] Stop mode which indicates immediate or wait end of es
   *
   * Use #AsStopPlayerStopMode enum type
   */

  uint8_t stop_mode;
} AsStopPlayerParam;

/** Adjust Sound Period Command (#AUDCMD_ADJUST_SOUNDPERIOD) parameter */

typedef struct
{
  /*! \brief [in] Set adjust directoin
   *
   * Use #AsClkRecoveryDirection enum type
   */

  int8_t  direction;

  /*! \brief [in] Set how many times do adjust
   *
   * Use #AsInitPlayerBitLength enum type
   */

  uint32_t times;
} AsClkRecoveryParam;

/** Set Audio gain level Command (#AUDCMD_SETGAIN) parameter */

typedef struct
{
  /*! \brief [in] Gain level Lch
   * Percentage 0 - 200 %
   */

  uint8_t l_gain;

  /*! \brief [in] Gain level Rch
   * Percentage 0 - 200 %
   */

  uint8_t r_gain;
} AsSetGainParam;

/** Message queue ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Message queue id of player */

  uint8_t player;

  /*! \brief [in] Message queue id of audio_manager */

  uint8_t mng;

  /*! \brief [in] Message queue id of output mixer */

  uint8_t mixer;

  /*! \brief [in] Message queue id of DSP */

  uint8_t dsp;
} AsPlayerMsgQueId_t;

/** Pool ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Memory pool id of es data */

  uint8_t es;

  /*! \brief [in] Memory pool id of pcm data */

  uint8_t pcm;

  /*! \brief [in] Memory pool id of dsp command data */

  uint8_t dsp;
} AsPlayerPoolId_t;

/** Activate function parameter */

typedef struct
{
  /*! \brief [in] ID for sending messages to each function */

  AsPlayerMsgQueId_t msgq_id;

  /*! \brief [in] ID of memory pool for processing data */

  AsPlayerPoolId_t   pool_id;
} AsActPlayerParam_t;

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
 * @brief Activate audio main player
 *
 * @param[in] param: Parameters of resources used by audio main player
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_ActivatePlayer(FAR AsActPlayerParam_t *param);

/**
 * @brief Activate audio sub player
 *
 * @param[in] param: Parameters of resources used by audio sub player
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_ActivateSubPlayer(FAR AsActPlayerParam_t *param);

/**
 * @brief Deactivate audio main player
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeactivatePlayer(void);

/**
 * @brief Deactivate audio sub player
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeactivateSubPlayer(void);

#ifdef __cplusplus
}
#endif

#endif  /* __SONY_APPS_INCLUDE_AUDIOUTIL_AUDIO_PLAYER_API_H */
/**
 * @}
 */

/**
 * @}
 */
