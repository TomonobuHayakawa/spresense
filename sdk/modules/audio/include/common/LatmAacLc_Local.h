/****************************************************************************
 * modules/audio/include/common/LatmAacLc_Local.h
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

#ifndef __MODULES_AUDIO_INCLUDE_COMMON_LATMAACLC_LOCAL_H
#define __MODULES_AUDIO_INCLUDE_COMMON_LATMAACLC_LOCAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LATM/LOASで使用するsyncword(11bit値→long値で取得後に比較すること) */

#define LATM_LENGTH_OF_SYNCWORD  11
#define LATM_LENGTH_OF_FRAME     13
#define LATM_SYNCWORD_LOAS       0x2B7        /* -010 1011 0111 */
#define LATM_SYNCWORD_EXT_LOAS   LATM_SYNCWORD_LOAS
#define LATM_SYNCWORD_EXT_PS     0x548        /* -101 0100 1000 */


/* Channel_Configuration)ISO準拠) */

#define LATM_CC_IDX_PCE    0        /* call program_config_element() */

/* SamplingFrequencyIndex)ISO準拠) */

#define LATM_FS_IDX_ESC    0xf      /* ESC value */

/*--------------- 以下、ローカル用 ---------------*/

#define LOCAL_CHECK_NG    -1

/* bit長を定義 */

#define LATM_BIT_OF_BYTE  8
#define LATM_BIT_OF_LONG  32
#define LATM_VAL_OF_5BIT  0x1F

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* AudioObjectType)ISO準拠) */

enum latm_aot_e
{
  AotNull = 0,
  AotAacMain,                     /*  1:AAC-MAIN */
  AotAacLc,                       /*  2:AAC-LC */
  AotAacSsr,                      /*  3:AAC-SSR */
  AotAacLtp,                      /*  4:AAC-LTP */
  AotSbr,                         /*  5:AAC-SBR */
  AotAacScalable,                 /*  6:AAC scalable */
  AotTwinVq,                      /*  7:TwinVQ */
  AotCelp,                        /*  8:CELP */
  AotHvxc,                        /*  9:HVXC */
  AotReserved10,
  AotReserved11,
  AotTtsi,                        /* 12:TTSI */
  AotMainSynthesis,               /* 13:Main synthesis */
  AotWavetableSynthesis,          /* 14:Wavetable synthesis */
  AotGeneralMidi,                 /* 15:General MIDI */
  AotAlgorithmicSynthesisAudioFx, /* 16:Algorithmic synthesis and Audio FX */
  AotErAacLc,                     /* 17:ER AAC-LC */
  AotReserved18,
  AotErAacLtp,                    /* 19:ER AAC-LTP */
  AotErAacScalable,               /* 20:ER AAC scalable */
  AotErTwinVq,                    /* 21:ER TwinVQ */
  AotErBsac,                      /* 22:ER BSAC */
  AotErAacLd,                     /* 23:ER AAC-LD */
  AotErCelp,                      /* 24:ER CELP */
  AotErHvxc,                      /* 25:ER HVXC */
  AotErHiln,                      /* 26:ER HILN */
  AotErParametric,                /* 27:ER Parametric */
  AotSsc,                         /* 28:SSC */
  AotPs,                          /* 29:PS */
  AotMpegSurround,                /* 30:MPEG Surround */
  AotEscape,                      /* 31:(escape) */
  AotLayer1,                      /* 32:Layer-1 */
  AotLayer2,                      /* 33:Layer-2 */
  AotLayer3,                      /* 34:Layer-3 */
  AotDst,                         /* 35:DST */
  AotAls,                         /* 36:ALS */
  AotSls,                         /* 37:SLS */
  AotSlsNonCore,                  /* 38:SLS non-core */
  AotErAacEld,                    /* 39:ER AAC-ELD */
  AotSmrSimple,                   /* 40:SMR Simple */
  AotSmrMain,                     /* 41:SMR Main */
  AotMax
};
typedef enum latm_aot_e LatmAot;

/* framLengthType(ISO準拠) */

enum latm_flt_e
{
  FltVariablePayload = 0, /* 0:Payload with variable frame length */
  FltFixedPayload,  /* 1:Payload with fixed frame length */
  FltReserved2,     /* 2:(reserved) */
  FltCelp1of2,      /* 3:Payload CELP with one of 2 kinds of frame length */
  FltCelpFixed,     /* 4:Payload CELP or ER CELP with fixed length */
  FltErCelp1of4,    /* 5:Payload ER CELP with one of 4 kinds frame length */
  FltHvxcFixed,     /* 6:Payload HVXC or ER HVXC with fixed frame length */
  FltHvxc1of4,      /* 7:Payload HVXC or ER HVXC with one of
                     *    4 kinds frame length
                     */
  FltMax
};
typedef enum latm_flt_e LatmFlt;

/* 本ツール内の各関数内で使い回す共有情報 */

struct latm_local_info_s
{
  uint8_t  *ptr_check_latm;    /* 現在ポインタ */
  uint32_t  total_bit_length;  /* 読み込み済みの累計bit長 */
  uint32_t  stream_cnt;        /* = StreamID */

  /* データ受け渡し用の目的で一時的に使用する */

  uint32_t  temp_long;        /* long値 */
};
typedef struct latm_local_info_s LatmLocalInfo;

/* Payloadでchunk使用時にのみ必要な情報 */

struct use_chunk_info_s
{
  uint8_t num_chunk;
  uint8_t reserve;
  int8_t stream_cnt_chunk[LATM_MAX_STREAM_ID]; /* 添字=chunk番号
                                                * (streamID - 1)
                                                */
};
typedef struct use_chunk_info_s UseChunkInfo;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __MODULES_AUDIO_INCLUDE_COMMON_LATMAACLC_LOCAL_H */
