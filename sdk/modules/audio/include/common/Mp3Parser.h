/****************************************************************************
 * modules/audio/include/common/Mp3Parser.h
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

#ifndef __MODULES_AUDIO_INCLUDE_COMMON_MP3PARSER_H
#define __MODULES_AUDIO_INCLUDE_COMMON_MP3PARSER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include "memutils/simple_fifo/CMN_SimpleFifo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*----- 戻り値定義 -----*/

#define MP3PARSER_SUCCESS           0  /* 成功 */
#define MP3PARSER_NO_FRAME_HEADER   -1 /* フレームヘッダが無い */
#define MP3PARSER_NO_OUTPUT_REGION  -2 /* フレーム取得用バッファサイズ不足 */
#define MP3PARSER_NO_CAPABILITY     -3 /* 使用不可 */
#define MP3PARSER_FILE_ERROR        -4 /* 指定ファイルが無い */
#define MP3PARSER_PARAMETER_ERROR   -5 /* パラメータエラー */

/* (ready_to_extract_frames) */

#define MP3PARSER_NEXT_SYNC_UNFOUND  0 /* 次のsyncwordが見つからない */
#define MP3PARSER_NEXT_SYNC_FOUND    1 /* 次のsyncwordが見つかった */

/*----- Misc値デフォルト定義(未設定時にデフォルト設定します) -----*/

/* 1つ目のsyncword検索上限デフォルト値(byte) */

#define MP3PARSER_DEFAULT_1ST_SYNC_SEARCH_MAX 0xFFFFFFFF

/* 2つ目のsyncword検索上限デフォルト値(byte) */

#define MP3PARSER_DEFAULT_2ND_SYNC_SEARCH_MAX 256

/* 切り出しモードのデフォルト値 */

#define MP3PARSER_DEFAULT_EXTRACTION_MODE  (Mp3ParserExtractFrameOnly)

#define MP3PARSER_DEFAULT_RAM_OFFSET  0xFFFFFFFF

#define MP3PARSER_SYNCWORD_1    0xFF
#define MP3PARSER_SYNCWORD_2    0xF0

/* 各アイテム値チェック用(reservedは使用されないはず) */

#define MP3PARSER_FS_RESERVED        3    /* '11' */
#define MP3PARSER_BITRATE_FREE       0    /* '0000' */
#define MP3PARSER_BITRATE_UNUSED     15   /* '1111' */
#define MP3PARSER_PRIVATEBIT_ISOUSED 1    /* '1' */
#define MP3PARSER_EMPHASIS_RESERVED  2    /* '10' */

/* 2byte目のメンバ */

#define MP3PARSER_GET_ID(byte)          ((byte & 0x08) >> 3)
#define MP3PARSER_GET_LAYER(byte)       ((byte & 0x06) >> 1)
#define MP3PARSER_GET_PROTECTION(byte)   (byte & 0x01)
/* 3byte目のメンバ */
#define MP3PARSER_GET_BR(byte)          ((byte & 0xF0) >> 4)
#define MP3PARSER_GET_FS(byte)          ((byte & 0x0C) >> 2)
#define MP3PARSER_GET_PADDING(byte)     ((byte & 0x02) >> 1)
#define MP3PARSER_GET_PRIVATE(byte)      (byte & 0x01)
/* 4byte目のメンバ */
#define MP3PARSER_GET_MODE(byte)        ((byte & 0xC0) >> 6)
#define MP3PARSER_GET_MODEEXT(byte)     ((byte & 0x30) >> 4)
#define MP3PARSER_GET_CPRIGHT(byte)     ((byte & 0x08) >> 3)
#define MP3PARSER_GET_ORGHOME(byte)     ((byte & 0x04) >> 2)
#define MP3PARSER_GET_EMPHAS(byte)       (byte & 0x02)

#define MP3PARSER_BITLENGTH_BYTE     8      /* 1byteのbit長 */
#define MP3PARSER_SLOTLENGTH_LAYER1  4      /* LAYER1の1-slot長 */
#define MP3PARSER_SLOTLENGTH_LAYER23 1      /* LAYER1の2/3-slot長 */

/* サンプル数(byteあたり)取得マクロ
 * ※ISO記述の「slot長算出式」に登場するマジックナンバー(定数144とか12とか)
 * が得られる
 */

#define MP3PARSER_CONST_BYTE_SAMPLES_V1(layer) \
          (mp3_parser_v1_num_samples_frame[layer] / MP3PARSER_BITLENGTH_BYTE)
#define MP3PARSER_CONST_BYTE_SAMPLES_V1_LAYER1 \
          (mp3_parser_v1_num_samples_frame[Mp3ParserLayer1] / \
            MP3PARSER_BITLENGTH_BYTE / MP3PARSER_SLOTLENGTH_LAYER1)
#define MP3PARSER_CONST_BYTE_SAMPLES_V2(layer) \
          (mp3_parser_v2_num_samples_frame[layer] / \
            MP3PARSER_BITLENGTH_BYTE)
#define MP3PARSER_CONST_BYTE_SAMPLES_V2_LAYER1 \
          (mp3_parser_v2_num_samples_frame[Mp3ParserLayer1] / \
            MP3PARSER_BITLENGTH_BYTE / MP3PARSER_SLOTLENGTH_LAYER1)

/* フレームバイト長算出マクロ
 * ・・・paddingがある場合は、この値に加算する
 * (一次元目の添え字=sampling_frequency)
 * (二次元目の添え字=bitrate_index)
 *
 *  値は、(byteあたりサンプル数 × bitrate ÷ sampling_frequency) ＋ padding長
 */

/* padding追加用(返す値の単位はslot数) */

#define MP3PARSER_CALC_PADDING(padding)    ((padding == 1) ? 1 : 0)

/* Layer1用・・・slot=4byte */

#define MP3PARSER_CALC_LAYER1_V1_FRAME_SIZE(br_idx,fs_idx,padding) \
          ((((MP3PARSER_CONST_BYTE_SAMPLES_V1_LAYER1 * \
            mp3_parser_v1_bitrate[Mp3ParserLayer1][br_idx] / \
            mp3_parser_v1_sampling_frequency[fs_idx])) + \
            MP3PARSER_CALC_PADDING(padding)) * MP3PARSER_SLOTLENGTH_LAYER1)
#define MP3PARSER_CALC_LAYER1_V2_FRAME_SIZE(br_idx,fs_idx,padding) \
          ((((MP3PARSER_CONST_BYTE_SAMPLES_V2_LAYER1 * \
            mp3_parser_v2_bitrate[Mp3ParserLayer1][br_idx] / \
            mp3_parser_v2_sampling_frequency[fs_idx])) + \
            MP3PARSER_CALC_PADDING(padding)) * MP3PARSER_SLOTLENGTH_LAYER1)

/* Layer3/Layer2用 */

#define MP3PARSER_CALC_LAYER3_V1_FRAME_SIZE(layer,br_idx,fs_idx,padding) \
          ((MP3PARSER_CONST_BYTE_SAMPLES_V1(layer) * \
            mp3_parser_v1_bitrate[layer][br_idx] / \
            mp3_parser_v1_sampling_frequency[fs_idx]) + \
            MP3PARSER_CALC_PADDING(padding))
#define MP3PARSER_CALC_LAYER3_V2_FRAME_SIZE(layer,br_idx,fs_idx,padding) \
          ((MP3PARSER_CONST_BYTE_SAMPLES_V2(layer) * \
            mp3_parser_v2_bitrate[layer][br_idx] / \
            mp3_parser_v2_sampling_frequency[fs_idx]) + \
            MP3PARSER_CALC_PADDING(padding))

/* Layer共通(mpeg1/2別) */

#define MP3PARSER_CALC_V1_FRAME_SIZE(layer,br_idx,fs_idx,padding)  \
          ((layer == Mp3ParserLayer1) ? \
            MP3PARSER_CALC_LAYER1_V1_FRAME_SIZE(br_idx,fs_idx,padding) : \
            MP3PARSER_CALC_LAYER3_V1_FRAME_SIZE(layer,br_idx,fs_idx,padding))
#define MP3PARSER_CALC_V2_FRAME_SIZE(layer,br_idx,fs_idx,padding)  \
          ((layer == Mp3ParserLayer1) ? \
            MP3PARSER_CALC_LAYER1_V2_FRAME_SIZE(br_idx,fs_idx,padding) : \
            MP3PARSER_CALC_LAYER3_V2_FRAME_SIZE(layer,br_idx,fs_idx,padding))

/* Layer共通 */

#define MP3PARSER_CALC_FRAME_SIZE(id,layer,br_idx,fs_idx,padding)  \
          ((id == Mp3ParserMpeg1) ? \
            MP3PARSER_CALC_V1_FRAME_SIZE(layer,br_idx,fs_idx,padding) : \
            MP3PARSER_CALC_V2_FRAME_SIZE(layer,br_idx,fs_idx,padding))

/* MP3フレームのうち、最小のもの(MPEG2 Layer3 fs=24kHz br=8k) */

#define MP3PARSER_MIN_MP3FRAME_LENGTH  (24 + 4)

#define MP3PARSER_NULL          0
#define MP3PARSER_HEADSIZE      4

/* 切り出し対象がファイルの場合用 */

/* 内部関数内ファイル読み込み用ローカルバッファサイズ */

#define MP3PARSER_LOCAL_READFILE_BUFFERSIZE  32

/* 内部関数内バッファ読み込み用ローカルバッファサイズ */

#define MP3PARSER_LOCAL_POLL_BUFFERSIZE      1024

#ifndef O_BINARY
#define O_BINARY  0
#endif

#ifdef WINDOWS
/* ファイルseek時の「origin指定＝ファイル先頭」 */

#define PA3PARSER_FILESEEK_ORIGIN_TOP  0

/* ファイルseek時の「origin指定＝ファイル最後尾」 */

#define PA3PARSER_FILESEEK_ORIGIN_END  2
#else
/*--------------------- for Spritzer --------------------*/

/* ファイルseek時の「origin指定＝ファイル先頭」 */

#define PA3PARSER_FILESEEK_ORIGIN_TOP   1  /* FS_FSEEK_SET */

/* ファイルseek時の「origin指定＝ファイル最後尾」 */

#define PA3PARSER_FILESEEK_ORIGIN_END   2  /* FS_FSEEK_END */
#endif

/* ID3v2タグ用 */

#define MP3PARSER_ID3V2_ID1      0x49  /* 'I' */
#define MP3PARSER_ID3V2_ID2      0x44  /* 'D' */
#define MP3PARSER_ID3V2_ID3      0x33  /* '3' */

/* ID3v2タグ長取得マクロ(各byteの最上位bitは無効) */

#define MP3PARSER_ID3v2_GET_LENGTH(len1,len2,len3,len4) \
          (uint32_t)(((len1 & 0x7F) << 21) | ((len2 & 0x7F) << 14) | \
            ((len3 & 0x7F) << 7) | (len4 & 0x7F))

/* ID3v1タグ長(固定長) */

#define MP3PARSER_ID3v1_FIXED_LENGTH    128
#define MP3PARSER_ID3v1_2_FIXED_LENGTH  227

/* ID3v1タグ用 */

#define MP3PARSER_ID3V1_ID1     0x54  /* 'T' */
#define MP3PARSER_ID3V1_ID2     0x41  /* 'A' */
#define MP3PARSER_ID3V1_ID3     0x47  /* 'G' */
#define MP3PARSER_ID3V1_ID4     0x2B  /* '+' */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* MP3切り出しモード(ライブラリ開始APIで指定) */

enum mp3parser_extraction_mode_e
{
  Mp3ParserExtractFrameOnly = 0,          /* 算出フレーム長で切り出す */
  Mp3ParserExtractFrameAndTrailingPadding /* 次syncwordまでをフレームとして
                                           * 切り出す
                                           */
};
typedef enum mp3parser_extraction_mode_e MP3PARSER_ExtractionMode;

/* MP3切り出し対象種別 */

enum mp3parser_src_type_e
{
  Mp3ParserSrcBuffer = 0,   /* 切り出し対象がバッファ */
  Mp3ParserSrcFile          /* 切り出し対象がファイル */
};
typedef enum mp3parser_src_type_e MP3PARSER_SrcType;

/* ライブラリ機能チェックAPI用 */

enum mp3parser_capability_e
{
  Mp3ParserCapabilityPeekable = 0,  /* PEKK可能か */
};
typedef enum mp3parser_capability_e MP3PARSER_Capability;

/** ライブラリ開始APIコール時の引数用の「その他パラメータ情報」
 *  (パラメータ情報のバッファはAPI呼び元で確保してください)
 *
 *  ・本パラメータ情報は、切り出し対象「バッファ／ファイル」で兼用です
 */

struct mp3parser_config_s
{
  uint32_t search_max_1st_sync; /* 1つ目syncwordの検索上限サイズ(byte長) */
  uint32_t search_max_2nd_sync; /* 2つ目syncwordの検索上限サイズ(byte長) */
  uint8_t  extraction_mode;     /* 切り出しモード */
  uint8_t  reserved[3];
};
typedef struct mp3parser_config_s MP3PARSER_Config;

/** APIコール時の引数用のハンドル情報
 *  (ハンドル情報のバッファはAPI呼び元で確保してください)
 *
 *  ・ライブラリ開始時に内容を初期設定し、一部情報は「切り出し毎」に更新します
 *
 */

struct mp3parser_handle_s
{
  int32_t  counter_of_extracted_frame; /* 取得済みフレーム数 */
  MP3PARSER_SrcType src_type;          /* 切り出し対象種別 */
  uint8_t  extraction_mode;            /* 切り出しモード */
  uint32_t search_max_1st_sync;        /* 1つ目syncwordの検索上限サイズ */
  uint32_t search_max_2nd_sync;        /* 2つ目syncwordの検索上限サイズ */
  union
  {
    /* 切り出し対象のSimpleFIFOハンドル情報(切り出し対象種別＝バッファ時) */

    FAR CMN_SimpleFifoHandle *simple_fifo_handler;

    /* 切り出し対象バッファの先頭(切り出し対象種別＝バッファ時) */

    FAR uint8_t *top_buff;
  } src;

  /* 切り出し対象のサイズ(バッファ／ファイル共通) */

  uint32_t size_of_src;

  /* 切り出し対象内の現在オフセット位置(バッファ／ファイル共通) */

  uint32_t current_offset;
  FAR MP3PARSER_Config  *pConfig;      /* 「その他パラメータ情報」 */
};
typedef struct mp3parser_handle_s MP3PARSER_Handle;

/*--------------------------------------------------------------------------*/
#if 0  /* BitField構造体 － 未使用だが、ヘッダ内メンバ情報としてコメント代わりに残す */
#ifdef WINDOWS
/** MP3 header - bit field
 *  (for little endian)
 */

struct mp3parser_header_s
{
  uint8_t  syncword1 : 8;             /* [0] bit0-7 */
  uint8_t  protection_bit : 1;        /* [1] bit7 */
  uint8_t  layer : 2;                 /* [1] bit5-6 */
  uint8_t  id : 1;                    /* [1] bit4 */
  uint8_t  syncword2 : 4;             /* [1] bit0-3 */
  uint8_t  private_bit : 1;           /* [2] bit7 */
  uint8_t  padding_bit : 1;           /* [2] bit6 */
  uint8_t  sampling_frequency : 2;    /* [2] bit4-5 */
  uint8_t  bitrate_index : 4;         /* [2] bit0-3 */
  uint8_t  emphasis : 2;              /* [3] bit6-7 */
  uint8_t  original_home : 1;         /* [3] bit5 */
  uint8_t  copyright : 1;             /* [3] bit4 */
  uint8_t  mode_extension : 2;        /* [3] bit2-3 */
  uint8_t  mode : 2;                  /* [3] bit0-1 */
};
typedef struct mp3parser_header_s Mp3ParserHeader;
#else
/** MP3 header - bit field
 *  (for big endian)
 */

struct mp3parser_header_s
{
  unsigned int  syncword : 12;           /* [0] bit0-7 [1] bit0-3 */
  unsigned int  id : 1;                  /* [1] bit4 */
  unsigned int  layer : 2;               /* [1] bit5-6 */
  unsigned int  protection_bit : 1;      /* [1] bit7 */
  unsigned int  bitrate_index : 4;       /* [2] bit0-3 */
  unsigned int  sampling_frequency : 2;  /* [2] bit4-5 */
  unsigned int  padding_bit : 1;         /* [2] bit6 */
  unsigned int  private_bit : 1;         /* [2] bit7 */
  unsigned int  mode : 2;                /* [3] bit0-1 */
  unsigned int  mode_extension : 2;      /* [3] bit2-3 */
  unsigned int  copyright : 1;           /* [3] bit4 */
  unsigned int  original_home : 1;       /* [3] bit5 */
  unsigned int  emphasis : 2;            /* [3] bit6-7 */
};
typedef struct mp3parser_header_s Mp3ParserHeader;
#endif
#endif

/* MP3ヘッダ情報 */

struct mp3parser_union_head_s
{
  uint8_t copy_byte[4];
};
typedef struct mp3parser_union_head_s Mp3ParserUnionHead;

/* Layer種別 */

enum mp3parser_header_layer_e
{
  Mp3ParserLayerReserved = 0,
  Mp3ParserLayer3,
  Mp3ParserLayer2,
  Mp3ParserLayer1,
};
typedef enum mp3parser_header_layer_e Mp3ParserHeaderLayer;

/* ID(version)種別 */

enum mp3parser_header_id_e
{
  Mp3ParserMpeg2 = 0,      /* MPEG2 (version2) */
  Mp3ParserMpeg1,          /* MPEG1 (version1) */
};
typedef enum mp3parser_header_id_e Mp3ParserHeaderId;

/* 内部関数間のI/F用 */

struct mp3parser_local_info_s
{
  Mp3ParserUnionHead  uhd;   /* MP3ヘッダ(固定長4byte) */
  FAR uint8_t *ptr_start;    /* (テンポラリ用) */
  uint32_t max_search_byte;  /* (テンポラリ用) */
  uint32_t search_offset;    /* (テンポラリ用) */
  uint32_t found_offset;     /* (テンポラリ用) */
  uint32_t sync_offset_1;    /* 1つ目のsyncword位置(先頭からのオフセット) */
  uint32_t sync_offset_2;    /* 2つ目のsyncword位置(先頭からのオフセット) */
  uint32_t frame_length_1;   /* 1つ目の算出フレーム長 */
  uint32_t frame_length_2;   /* 2つ目の算出フレーム長 */
  uint32_t remain_length;    /* 切り出し対象の残サイズ */
};
typedef struct mp3parser_local_info_s Mp3ParserLocalInfo;

/* ファイルアクセス関数の戻り値 */

enum mp3parser_return_value_of_file_e
{
  Mp3ParserReturnNoFilename = (-3),     /* filename異常 */
  Mp3ParserReturnFileOpenError = (-2),  /* ファイルエラー */
  Mp3ParserReturnFileAccesError = (-1), /* ファイルエラー */
  Mp3ParserReturnFileFavorable = 0,     /* 正常 */
};
typedef enum mp3parser_return_value_of_file_e Mp3ParserReturnValueOfFile;

/* ID3v2タグヘッダの「byteテーブル」時のインデックス */

enum mp3parser_id3v2_header_index_e
{
  Mp3ParserID3v2HeadIndexID1 = 0,
  Mp3ParserID3v2HeadIndexID2,
  Mp3ParserID3v2HeadIndexID3,
  Mp3ParserID3v2HeadIndexVer1,
  Mp3ParserID3v2HeadIndexVer2,
  Mp3ParserID3v2HeadIndexFlag,
  Mp3ParserID3v2HeadIndexLen1,
  Mp3ParserID3v2HeadIndexLen2,
  Mp3ParserID3v2HeadIndexLen3,
  Mp3ParserID3v2HeadIndexLen4,
  Mp3ParserID3v2HeaderLength     /* 10byte目(ID3v2タグヘッダ長として使用) */
};
typedef enum mp3parser_id3v2_header_index_e Mp3ParserID3v2HeaderIndex;

/* ID3v1タグヘッダの「byteテーブル」時のインデックス */

enum mp3parser_id3v1_header_index_e
{
  Mp3ParserID3v1HeadIndexID1 = 0,
  Mp3ParserID3v1HeadIndexID2,
  Mp3ParserID3v1HeadIndexID3,
  Mp3ParserID3v1HeadIndexID4,
};
typedef enum mp3parser_id3v1_header_index_e Mp3ParserID3v1HeaderIndex;

/* syncword検索関数からの戻り値 */

enum mp3parser_return_value_of_sync_search_e
{
  Mp3ParserReturnNoSyncword = (-1), /* syncword未検出 */
  Mp3ParserReturnPendding = 0,      /* 保留(指定サイズ終了直前にsyncword
                                     * かもしれないものがあるが、ヘッダ情報
                                     * は指定サイズ外にあるため確認できない)
                                     */
  Mp3ParserReturnFoundSyncword = 1, /* syncword検出 */
  Mp3ParserReturnFound1stOnly,      /* 1つ目syncwordのみ検出 */
  Mp3ParserReturnSearchContinue,    /* 偽syncをskipして処理続行 */
};
typedef enum mp3parser_return_value_of_sync_search_e \
               Mp3ParserReturnValueOfSyncSearch;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* 1フレーム内のサンプル数 順番は、上記Mp3ParserHeaderLayerと同じ) */

static const uint32_t mp3_parser_v1_num_samples_frame[] =
{
  0,
  1152,
  1152,
  384
};

static const uint32_t mp3_parser_v2_num_samples_frame[] =
{
  0,
  576,
  1152,
  384
};

/* Layer別ビットレートテーブル
 * (一次元目の添え字=上記Mp3ParserHeaderLayer)
 * (二次元目の添え字=bitrate_index)
 */

static const int32_t mp3_parser_v1_bitrate[4][16] =
{
  /* mp3_parser_layer_reserved */

  {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0
  },

  /* mp3_parser_layer_3 */

  {
    0,
    32000,
    40000,
    48000,
    56000,
    64000,
    80000,
    96000,
    112000,
    128000,
    160000,
    192000,
    224000,
    256000,
    320000,
    -1
  },

  /* mp3_parser_layer_2 */

  {
    0,
    32000,
    48000,
    56000,
    64000,
    80000,
    96000,
    112000,
    128000,
    160000,
    192000,
    224000,
    256000,
    320000,
    384000,
    -1
  },

  /* mp3_parser_layer_1 */

  {
    0,
    32000,
    64000,
    96000,
    128000,
    160000,
    192000,
    224000,
    256000,
    288000,
    320000,
    352000,
    384000,
    416000,
    448000,
    -1
  }
};

static const int32_t mp3_parser_v2_bitrate[4][16] =
{
  /* mp3_parser_layer_reserved */

  {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0
  },

  /* mp3_parser_layer_3 */

  {
    0,
    8000,
    16000,
    24000,
    32000,
    40000,
    48000,
    56000,
    64000,
    80000,
    96000,
    112000,
    128000,
    144000,
    160000,
    -1
  },

  /* mp3_parser_layer_2 */

  {
    0,
    8000,
    16000,
    24000,
    32000,
    40000,
    48000,
    56000,
    64000,
    80000,
    96000,
    112000,
    128000,
    144000,
    160000,
    -1
  },

  /* mp3_parser_layer_1 */

  {
    0,
    32000,
    48000,
    56000,
    64000,
    80000,
    96000,
    112000,
    128000,
    144000,
    160000,
    176000,
    192000,
    224000,
    256000,
    -1
  }
};

/* サンプリング周波数テーブル(添え字=sampling_frequency) */

static const uint32_t mp3_parser_v1_sampling_frequency[4] =
{
  44100,
  48000,
  32000,
  0
};

static const uint32_t mp3_parser_v2_sampling_frequency[4] =
{
  22500,
  24000,
  16000,
  0
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* 外部API宣言 */

int32_t Mp3Parser_initialize(FAR MP3PARSER_Handle *ptr_hndl,
                             FAR CMN_SimpleFifoHandle *simple_fifo_handler,
                             FAR MP3PARSER_Config *config);
int32_t Mp3Parser_pollSingleFrame(FAR MP3PARSER_Handle *ptr_hndl,
                                  FAR uint8_t *out_buffer,
                                  uint32_t out_buffer_size,
                                  FAR uint32_t *out_frame_size,
                                  FAR int32_t *ready_to_extract_frames);
int32_t Mp3Parser_finalize(FAR MP3PARSER_Handle *ptr_hndl);
int32_t Mp3Parser_getSamplingRate(FAR MP3PARSER_Handle *ptr_hndl,
                                  FAR uint32_t *ptr_sampling_rate);

/* ローカル関数宣言 */

uint32_t mp3parser_extract_frame(FAR MP3PARSER_Handle *ptr_hndl,
                                 FAR Mp3ParserLocalInfo *ptr_info,
                                 FAR uint8_t *out_buffer);
Mp3ParserReturnValueOfSyncSearch mp3parser_distribute_processing( \
                                   FAR MP3PARSER_Handle *ptr_hndl,
                                   FAR Mp3ParserLocalInfo *ptr_info);
int32_t mp3parser_get_frameheader(FAR MP3PARSER_Handle *ptr_hndl,
                                  FAR Mp3ParserLocalInfo *ptr_info,
                                  FAR uint8_t *ptr_local_buff);
Mp3ParserReturnValueOfFile mp3parser_buffer_check_tag( \
                             FAR MP3PARSER_Handle *ptr_hndl,
                             FAR Mp3ParserLocalInfo *ptr_info);
Mp3ParserReturnValueOfFile mp3parser_buffer_check_tag_v1( \
                             FAR MP3PARSER_Handle *ptr_hndl,
                             FAR Mp3ParserLocalInfo *ptr_info);

#endif /* __MODULES_AUDIO_INCLUDE_COMMON_MP3PARSER_H */
