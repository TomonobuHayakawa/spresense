/****************************************************************************
 * modules/audio/include/apus/dsp_audio_version.h
 *
 *   Copyright (C) 2017 Sony Corporation
 *   Author: Tomonobu Hayakawa <Tomonobu.Hayakawa@sony.com>
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

#ifndef __MODULES_AUDIO_INCLUDE_APUS_DSP_AUDIO_VERSION_H
#define __MODULES_AUDIO_INCLUDE_APUS_DSP_AUDIO_VERSION_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Version rule:
 * (change library).(change of DSP interface).(change of internal processing)
 */

/* Decoder Version. */

#define DSP_AACDEC_VERSION    0x010203    /* 01.02.03 */
#define DSP_MP3DEC_VERSION    0x010203    /* 01.02.03 */
#define DSP_OPUSDEC_VERSION   0x010203    /* 01.02.03 */
#define DSP_WAVDEC_VERSION    0x010203    /* 01.02.03 */

/* Encoder Version. */

#define DSP_MP3ENC_VERSION    0x010203    /* 01.02.03 */
#define DSP_OPUSENC_VERSION   0x010304    /* 01.03.04 */

/* Filter Version. */

#define DSP_MFESRC_VERSION    0x010202    /* 01.02.02 */
#define DSP_SRC_VERSION       0x010203    /* 01.02.03 */
#define DSP_MPPEAX_VERSION    0x010202    /* 01.02.02 */

/* Recognizer Version. */

#define DSP_FREQDET_VERSION   0x010202    /* 01.02.02 */
#define DSP_MEASURE_VERSION   0x010202    /* 01.02.02 */
#define DSP_VADWUW_VERSION    0x010202    /* 01.02.02 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __MODULES_AUDIO_INCLUDE_APUS_DSP_AUDIO_VERSION_H */

