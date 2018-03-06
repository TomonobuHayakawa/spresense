/****************************************************************************
 * modules/audio/include/common/RamAdtsParser.h
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

#ifndef __MODULES_AUDIO_INCLUDE_COMMON_RAMADTSPARSER_H
#define __MODULES_AUDIO_INCLUDE_COMMON_RAMADTSPARSER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "memutils/common_utils/common_types.h"
#include "memutils/simple_fifo/CMN_SimpleFifo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ヘッダ情報の妥当性チェック結果 */

#define  HDR_OK            0x0000  /* No Error */
#define  HDR_SYNCWORD_NG   0x0001  /* Error: syncword not found */
#define  HDR_PROFILE_NG    0x0002  /* Error: profile NG (not AAC-LC) */
#define  HDR_SAMLERATE_NG  0x0004  /* Error: Invalid Sample Rate */
#define  HDR_FRAMESIZE_NG  0x0008  /* Error: Frame Size */
#define  HDR_ERROR         0x8000  /* Undefined Error */

#define PARSER_LOCAL_POLL_BUFFERSIZE 1024

/* ADTS-API戻り値 */

#define  ADTS_OK    0    /* 成功 */
#define  ADTS_ERR   1    /* 失敗 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* ADTS-DATAアクセス用ハンドル(Parser内部で使用) */

struct adts_handle_s
{
  FAR CMN_SimpleFifoHandle  *pSimpleFifoHandler;
  uint32_t        current_pos;      /* 現在位置(先頭からのoffset) */
  uint32_t        search_pos;       /* サーチ位置(先頭からのoffset) */
  uint32_t        parse_size;       /* パースデータサイズ */
};
typedef struct adts_handle_s AdtsHandle;

enum adts_parser_error_detail_e
{
  AdtsParserNormal = 0,       /* 正常(エラーなし) */
  AdtsParserAbnormalArg,      /* 引数エラー(戻り値=ADTS_ERR時) */
  AdtsParserConnotDataAccess, /* ファイルアクセスエラー(戻り値=ADTS_ERR時) */
  AdtsParserCannotGetHeader,  /* ヘッダ取得不可(戻り値=ADTS_ERR時) */
  AdtsParserAbnormalHeader,   /* ヘッダ情報異常(戻り値=ADTS_ERR時) */
  AdtsParserShortageBuffer,   /* バッファサイズ不足(戻り値=ADTS_ERR時) */
};
typedef enum adts_parser_error_detail_e AdtsParserErrorDetail;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/*!
 * @brief ADTS Parserの初期化
 *
 * @param[in] pHandle ADTS-DATAアクセス用ハンドルのポインタ
 *
 * @param[in] simple_fifo_handler SimpleFIFOハンドル情報のポインタ
 *
 * @param[out] uipErrDetail エラー詳細
 *
 * @return Function return code
 */

int32_t AdtsParser_Initialize(FAR AdtsHandle *pHandle,
                              FAR CMN_SimpleFifoHandle *pSimpleFifoHandler,
                              FAR AdtsParserErrorDetail *uipErrDetail);

/*!
 * @brief ADTS Dataからのフレーム取得(1フレーム)
 *
 * @param[in] pHandle ADTS-DATAアクセス用ハンドルのポインタ
 *
 * @param[in] buff 取得用バッファの先頭ポインタ
 *
 * @param[in/out] uiSize - [in] 取得バッファのサイズ  [out] 取得フレームのサイズ(Byte数)
 *
 * @param[out] usResult ヘッダ情報の妥当性チェック結果
 *
 * @param[out] uipErrDetail エラー詳細
 *
 * @return Function return code
 */

int32_t AdtsParser_ReadFrame(FAR AdtsHandle *pHandle,
                             FAR int8_t *buff,
                             FAR uint32_t *uiSize,
                             FAR uint16_t *usResult,
                             FAR AdtsParserErrorDetail *uipErrDetail);

/*!
 * @brief ADTS Parserの終了
 *
 * @param[in] pHandle ADTS-DATAアクセス用ハンドルのポインタ
 *
 * @param[out] uipErrDetail エラー詳細
 *
 * @return Function return code
 */

int32_t AdtsParser_Finalize(FAR AdtsHandle *pHandle,
                            FAR AdtsParserErrorDetail *uipErrDetail);

/*!
 * @brief サンプリング周波数取得(ADTSヘッダーからの簡易読み出し)
 *
 * @param[in] pHandle ADTS-DATAアクセス用ハンドルのポインタ
 *
 * @param[in] uipSmplingRate 取得したサンプリング周波数値
 *
 * @param[out] uipErrDetail エラー詳細
 *
 * @return Function return code
 */

int32_t AdtsParser_GetSamplingRate(FAR AdtsHandle *pHandle,
                                   FAR uint32_t *uipSmplingRate,
                                   FAR AdtsParserErrorDetail *uipErrDetail);

#endif /* __MODULES_AUDIO_INCLUDE_COMMON_RAMADTSPARSER_H */
