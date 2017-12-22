/****************************************************************************
 * modules/audio/include/common/LatmAacLc.h
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

#ifndef __MODULES_AUDIO_INCLUDE_COMMON_LATMAACLC_H_
#define __MODULES_AUDIO_INCLUDE_COMMON_LATMAACLC_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
 
/* ユ－ザー側でconfig_lengthをセットする場合、
 * config_length_flagにも以下のdefineをセットすること
 */

#define LATM_ENABLE_CONFIG_LENGTH  1 /* config_length_flag有効 */

/* 後述する構造体メンバ内の定義 */

#define LATM_VAL_OF_4BIT    0xF
#define LATM_MAX_STREAM_ID  16       /* ストリームIDの最大値 */
#define LATM_MIN_STREAM_ID  0        /* ストリームIDの最小値 */

/* 情報テーブルの配列では、ストリームID(1～16)を添字に使用するので
 * 最大数は+1する
 */

#define LATM_MAX_STREAM_ARRAY    (LATM_MAX_STREAM_ID + 1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/*
 * AudioSpecificConfig情報
 *
 * StreamMuxConfig内の、StreamID毎に保持する
 * ※「use_same_configフラグ=1」時には、
 * 1つ前のAudioSpecificConfig保持情報をcopyする
 */

struct info_audio_specific_config_s /* 添え字はstreamID([0]は未使用) */
{
  /* 上記useSameConfig=1の時は、その前のAudioSpecificConfig内容をcopy */

  /* audioObjectType(構文上の最大値=95) */

  uint8_t audio_object_type;

  /* channelConfiguration(4-bit(有効値0～0xf)) */

  uint8_t channel_configuration;

  /* samplingFrequencyIndex(4-bit(有効値0～0xf)) */

  uint8_t sampling_frequency_index;

  /* program_config_element使用時に、とりあえず以下の項目だけは保持しておく
   * ～PCE情報全部だと大きすぎるので
   */

  /* chanelConfigurationが0時のprogram_config_element内に設定された
   * object type
   */

  uint8_t pce_object_type;

  /* chanelConfigurationが0時のprogram_config_element内に設定された
   * samplingFrequencyIndex
   */

  uint8_t pce_sampling_frequency_index;

  /* 以下は拡張用 */

  int8_t   sbr_present_flag;     /* sbrPresentFlag */
  int8_t   ps_present_flag;      /* psPresentFlag */
  uint8_t  extension_audio_object_type; /* extensionAudioObjectType */

  /* extensionChannelConfiguration(ER-BSAC時のみ) */

  uint8_t  extension_channel_configuration;

  /* extensionSamplingFrequencyIndex */

  uint8_t  extension_sampling_frequency_index;

  /* extensionSamplingFrequency
   * (24-bit extensionSamplingFrequencyIndex=0xF(escape value)時に使用)
   */

  uint32_t extension_sampling_frequency;

  /* FS値は拡張用ではないが、構造体のアライメントを考慮して最後尾にセット */

  /* samplingFrequency
   * (24-bit samplingFrequencyIndex=0xF(escape value)時に使用)
   */

  uint32_t sampling_frequency;

  /* config_lengthは、ユーザーが設定(未設定時は0として扱う) */

  /* config_length値の有効無効フラグ(1=有効 1≠無効) */

  uint8_t config_length_flag;
  uint8_t reserved;

  /* AudioSpecificConfigのbitサイズ(上記config_length_flag=1時にのみ有効) */

  int config_length;

  /*----- 以下SpecificConfig情報(サイズが大きくなるのでコメントアウトしておく) -----*/

#ifdef LATMTEST_DBG_COMMENT
  union
  {
    /* channelConfiguration=0時は
     * GASpecificConfig内のprogram_config_element()が頼りだが、
     * parserには不要
     */

    struct GASpecificConfig  ga;  /* AAC */
    struct SSCSpecificConfig ssc; /* AudioObjectType=28(SSC) */
  } spConfig;
#endif /* LATMTEST_DBG_COMMENT */
};
typedef struct info_audio_specific_config_s InfoAudioSpecificConfig;

struct info_stream_id_s /* 添え字はstreamID([0]は未使用) */
{
  /* streamIDは、1～16の値(infoStreamID[]の添え字)。
   * 元はカウンタなので、未使用(=0)になったら以降のIDは全て未使用
   */

  int8_t stream_id;     /* streamID */

  /*----- 以降はstreamID≠0の時に使用 -----*/

  /* 逆引きstreamID */

  uint8_t prog;         /* streamIDに対応するprogram番号 */
  uint8_t lay;          /* streamIDに対応するlayer番号 */

  /* 以下はstreamIDに対応する項目 */

  /* frameLengthType(ペイロードタイプ) */

  uint8_t frame_length_type;

  /* frameLength(9-bit frameLengthType=1の時に使用) */

  uint32_t frame_length;

  /* latmBufferFullness(frameLengthType==0時のみ) */

  uint8_t latm_buffer_fullness;

  /* useSameConfig(=1の場合、ストリーム上ではAudioSpecificConfig省略される) */

  uint8_t use_same_config;

  /* useSameConfig値にかかわらず、AudioSpecificConfigを用意
   * (論理的には、StreamMuxConfigが存在してもAudioSpecificConfigが
   * 存在しないケースがあるため)
   */

  InfoAudioSpecificConfig asc;

  /* LATM先頭からのoffset
   * 対応するpayloadのoffset値(StreamMuxConfigが存在する場合のみ)
   */

  uint32_t payload_offset;
};
typedef struct info_stream_id_s InfomationStreamID;

struct info_stream_frame_s /* 添え字はstreamID([0]は未使用) */
{
    /* 以下はstreamIDに対応する項目 */

  /* frameLengthType(ペイロードタイプ) */

  uint8_t frame_length_type;

  /* frameLength(9-bit frameLengthType=1の時に使用) */

  uint32_t frame_length;

  /* LATM先頭からのoffset
   * 対応するpayloadのoffset値(StreamMuxConfigが存在する場合のみ)
   */

  uint32_t payload_offset;
};
typedef struct info_stream_frame_s InfomationStreamFrame;

/* StreamMuxConfig情報のうち、
 * 分岐判定のためユーザー側で保持しておいてほしい構造体情報
 *
 * [使用方法]
 * 1. 提供するAPI関数の1回目の使用前に、本構造体サイズのバッファを確保
 * 2. API関数をコールする際の引数2を「構造体バッファの先頭」にする
 * 3. 以降、API関数を連続使用する間は、同バッファを解放しない
 *   (LATMフレームを連続して読み出す間は解放しない)
 *
 * ※構造体メンバの「info_stream_id[].asc.config_length」は、ユーザー側で設定
 *   ・・・AudioConfigSpecific()のサイズがわかっている場合、
 *         そのサイズをbit長でセット
 *         不明な場合、0をセット
 */

struct info_stream_mux_config_s
{
  /* 使用するstreamIDの最大値(有効値1～16)－ 0は未使用のため、他項目の参照不可 */

  uint8_t max_stream_id;

  /* 1-bit audioMuxVersion */

  uint8_t audio_muxversion;

  /* 1-bit audioMuxVersionA */

  uint8_t audio_muxversion_a;

  /* 1-bit allStreamsSameTimeFraming */

  uint8_t all_streams_sametime_framing;

  /* 1-bit otherDataPresent */

  uint8_t other_data_present;
  uint8_t reserved;

  /* 6-bit(有効値0～63) numSubFrames */

  uint8_t num_sub_frames;

  /* 4-bit(有効値0～15) numProgram */

  uint8_t num_program;

  /* 3-bit(有効値0～7)(添え字はprogram番号) numLayer */

  uint8_t num_layer[(LATM_VAL_OF_4BIT + 1)];

  /* otherDataLenBits(otherDataPresent=1の時に使用) */

  uint32_t other_data_len_bits;

  /* 以下の2つは、streamID値-1でセットされるため、別に扱う */

  /* progSIndx(allStreamsSameTimeFraming=0の時に使用) */

  uint8_t   prog_stream_indx[LATM_MAX_STREAM_ID];

  /* laySIndx(allStreamsSameTimeFraming=0の時に使用) */

  uint8_t   lay_stream_indx[LATM_MAX_STREAM_ID];

  /* 構文上は最大「Program×Layer」だが、streamIdx=streamCnt=streamIDであり、
   * 実質の最大は16のはず
   */

  /* 添え字はstreamID([0]は未使用) */

  InfomationStreamID info_stream_id[LATM_MAX_STREAM_ARRAY];
  InfomationStreamFrame info_stream_frame[64];
};
typedef struct info_stream_mux_config_s InfoStreamMuxConfig;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/*
 * AACLC_getNextLatm()
 *
 * 次LATM先頭を取得するAPI
 *
 * 引数1 : LOAS/LATMの先頭(例えば、ペイロードの先頭)
 * 引数2 : 上記で確保した構造体バッファの先頭
 *
 * 戻り値 : 引数1で始まるLATMフレームの次LATM先頭アドレス
 *          0=NG(LATMヘッダ内データにある、AudioObjectTypeが「未サポート」)
 */
FAR uint8_t *AACLC_getNextLatm(FAR uint8_t *ptr_readbuff,
                               FAR InfoStreamMuxConfig *ptr_stream_mux_config);

#endif /* __MODULES_AUDIO_INCLUDE_COMMON_LATMAACLC_H_ */
