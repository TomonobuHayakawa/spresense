/****************************************************************************
 * modules/audio/include/apus/apu_cmd.h
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

#ifndef __MODULES_AUDIO_INCLUDE_APUS_APU_CMD_H
#define __MODULES_AUDIO_INCLUDE_APUS_APU_CMD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <limits.h>
#include "memutils/common_utils/common_types.h"
#include "wien2_common_defs.h"
#include "apus/apu_cmd_defs.h"

__WIEN2_BEGIN_NAMESPACE
__APU_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#if defined(__CC_ARM)
#pragma anon_unions
#endif

/**
 * Apuへの制御コマンド結果
 */

enum exec_result_e
{
  ApuExecOK,     /**< 実行成功 */
  ApuExecError,  /**< Errorが発生 */
  ApuWarning     /**< Warningが発生 */

};
typedef enum exec_result_e ExecResult;

/**
 * Error 発生元
 */

enum apu_internal_err_src_e
{
  FromDrv = 0,  /**< Driver 起因のエラー */
  FromLib       /**< Library 起因のエラー */
};
typedef enum apu_internal_err_src_e ApuInternalErrorSource;

/**
 * TODO(Hsingying, 2015/4/22):
 *   今まで使用した定義をそのままWien2 namespaceに持ってきただけ。
 *   直近、精査 & 定義をし直す必要がある。
 */

enum apu_internal_err_code_e
{
  ApuSuccess         = 0x00,  /**< OK */
  ApuStateErr        = 0x01,  /**< State violation */
  ApuDumpInitError   = 0x10,  /**< Initialization error of dump function */
  ApuCodecTypeError,          /**< Codec type value error */
  ApuStaticAreaInsufficient,  /**< Library static area insufficient */
  ApuChannelFormatError,      /**< Channel format value  error */
  ApuSamplingRateError,       /**< Sampling rate value error */
  ApuBitRateError,            /**< Bit rate value error */
  ApuByteLenError,            /**< Byte length value error */
  ApuComplexityError,         /**< Complexity value error */
  ApuQueueFull,               /**< Queue is full */
  ApuInitErr         = 0x20,  /**< Library initialization error */
  ApuDecErr          = 0x30,  /**< Decoder library error */
  ApuEncErr          = 0x40,  /**< Encoder library error */
  ApuSRCErr          = 0x50,  /**< Sampling rate converter library error */
  ApuMFEErr          = 0x60   /**< Mic front end library error */
};
typedef enum apu_internal_err_code_e ApuInternalErrorCode;

/* PCM format の型 */
enum audio_pcm_format_type_e
{
  AudPcmFormatInt16 = 0,  /**< 符号付16bit整数型 */
  AudPcmFormatInt24,      /**< 符号付24bit整数型 */
  AudPcmFormatInt32,      /**< 符号付32bit整数型 */
  AudPcmFormatFloat32     /**< 符号付32bit浮動小数点型 */
};
typedef enum audio_pcm_format_type_e AudioPcmFormatType;

/* xLOUD処理モード */
enum audio_xloud_mode_e
{
  AudXloudModeNormal = 0,  /**< 通常モード */
  AudXloudModeCinema,      /**< 映画モード */
  AudXloudModeDisable      /**< Disable, パスするー動作 */
};
typedef enum audio_xloud_mode_e AudioXloudMode;

/* EAX 環境適応処理モード */
enum audio_eax_mode_e
{
  AudEaxModeNormal = 0,  /**< 標準モード */
  AudEaxModeCall,        /**< 通話モード */
  AudEaxModeDisable      /**< Disable, xLOUDの効果調整量 0 固定 */
};
typedef enum audio_eax_mode_e AudioEaxMode;

/* xLOUD処理モード */
enum audio_xloud_set_type_e
{
  AudXloudSetCommon = 0,  /**< 共通 */
  AudXloudSetIndividual,  /**< 個別 */
  AudXloudSetTypeNum
};
typedef enum audio_xloud_set_type_e AudioXloudSetType;

/****************************************************************************/
/**
 * Apu command header
 */

struct apu_cmd_header_s
{
  uint8_t core_id;       /**< コマンド受信先(Apuから見た場合)のcore id */
  uint8_t context_id;    /**< 制御したいcore内のcontextのid */
  uint8_t process_mode;  /**< Apuコマンドのprocess mode */
  uint8_t event_type;    /**< Apuコマンドのevent種別 */

  apu_cmd_header_s():
    core_id(0xFF),
    context_id(0xFF),
    process_mode(InvalidApuProcessMode),
    event_type(InvalidApuEvent)
    {}
};
typedef struct apu_cmd_header_s ApuCmdHeader;

/****************************************************************************/
/**
 * Debug Dump Information
 */

struct debug_dump_info_s
{
public:
  FAR void *addr;  /**< Debug dump area address */
  size_t size;    /**< Debug dump area size */
};
typedef struct debug_dump_info_s DebugDumpInfo;

/****************************************************************************/
/**
 * Apu result command
 */

struct internal_result_s
{
public:
  ApuInternalErrorSource res_src;  /**< Error source */
  ApuInternalErrorCode   code;     /**< Error code */
  uint32_t               value;    /**< Error value */
};
typedef struct internal_result_s InternalResult;

struct apu_result_s
{
public:
  ExecResult     exec_result;        /**< Execute resule */
  InternalResult internal_result[3]; /**< Information about internal result */

  apu_result_s():
    exec_result(ApuExecOK)
    {}
};
typedef struct apu_result_s ApuResult;

/****************************************************************************/
/**
 *    Apu command for decoder control
 */
/**
 * Initializing processing
 */

struct apu_pcm_param_s
{
public:
  AudioPcmBitWidth   bit_length;        /**< Bit length of output data */
  AudioChannelFormat channel_format;    /**< Channel format of output data */
  uint32_t           sampling_rate;     /**< Sampling rate of output data */
  AudioChannelConfig channel_config;    /**< Channel config of output data */
  int32_t            decoder_output_sample; /**< Sample number of output data */
};
typedef struct apu_pcm_param_s ApuPcmParam;

struct apu_init_dec_cmd_s
{
public:
  AudioCodec         codec_type;      /**< Codec type of input data */
  uint8_t            channel_num;     /**< Channel number of input data */
  uint32_t           bit_rate;        /**< Bit rate of input data */
  uint32_t           sampling_rate;   /**< Sampling rate of input data */
  AudioChannelConfig channel_config;  /**< Channel config of input data */
  ApuPcmParam        out_pcm_param;   /**< Output data information */
  DebugDumpInfo      debug_dump_info; /**< Debug dump information */
};
typedef struct apu_init_dec_cmd_s ApuInitDecCmd;

/**
 * Executing processing
 */

struct ApuExecDecCmd
{
public:
  BufferHeader  input_buffer;   /**< Input buffer information */
                                /**<  (including at least buffer address */
                                /**<  or pointer and buffer size) */
  BufferHeader  output_buffer;  /**< Output buffer information */
                                /**<  (including at least buffer address */
                                /**<  or pointer and buffer size) */
  uint8_t       num_of_au;      /**< Number of AU */
};

/**
 * Flushing processing
 */

struct ApuFlushDecCmd
{
public:
  BufferHeader  output_buffer; /**< Output buffer information */
                               /**<  (including at least buffer address */
                               /**<  or pointer and buffer size) */
};


/****************************************************************************/
/**
 *    Apu command for filter control
 */
/**
 * Initializing processing
 */
/* Type of src */

struct ApuInitSRCParam
{
public:
  uint32_t  input_sampling_rate;   /**< Sampling rate of input data */
  uint32_t  output_sampling_rate;  /**< Sampling rate of output data */
  float     cut_off;               /**< Cut off */
  uint32_t  attenuation;           /**< Attenuation[dB] */
  uint16_t  in_word_len;           /**< Word length of input data[Byte] */
  uint16_t  out_word_len;          /**< Word length of output data[Byte] */

};

/* Type of mfe */

struct ApuInitMFEParam
{
public:
  int32_t   proc_mode;        /**< MFE mode */
  int32_t   ref_channel_num;  /**< Channel number of reference data */
  int32_t   sampling_rate;    /**< Sampling rate of input data */
  bool      use_aec;          /**< Include echo cancellr */
  bool      enable_mfe_aec;   /**< Echo cancellr */
  uint32_t  config_table;     /**< MFE configuration table */
};

/* Type of mpp */

struct ApuInitXLOUDParam
{
public:
  uint32_t            input_sampling_rate;     /**< Sampling rate of input data */
  AudioXloudMode      mode;                    /**< xLOUD mode */
  AudioPcmFormatType  in_pcm_bit_len;          /**< Bit length of input data */
  AudioPcmFormatType  out_pcm_bit_len;         /**< Bit length of input data */
  FAR uint8_t         *p_coef_image;           /**< Pointer of optimization coefficient table */
  uint32_t            coef_size;               /**< Size of optimization coefficient table */
  uint32_t            eax_input_sampling_rate; /**< EAX sampling rate of intput data */
  uint8_t             eax_mic_channel_num;     /**< EAX mic channel number of input data */
  uint32_t            eax_sample;              /**< EAX Sample number of input data */
  AudioEaxMode        eax_mode;                /**< EAX mode */
  AudioPcmFormatType  eax_in_pcm_bit_len;      /**< EAX bit length of input data */
  AudioPcmFormatType  eax_out_pcm_bit_len;     /**< EAX bit length of input data */
  bool                eax_enable_external_analysis; /**< EAX setting flag to use external library */
  FAR uint8_t        *p_eax_coef_image;       /**< EAX pointer of optimization coefficient table */
  uint32_t            eax_coef_size;          /**< EAX size of optimization coefficient table */
  FAR void           *p_dma_info;             /**< Dma info 変数のアドレス */
};

struct ApuInitFilterCmd
{
public:
  ApuFilterType  filter_type;  /**< Type of filter */
  uint8_t        channel_num;  /**< Channel number of input data */
  uint32_t       sample;       /**< Sample number of input data */
  union
  {
    ApuInitSRCParam   init_src_param;   /**< SRC parameter information */
    ApuInitMFEParam   init_mfe_param;   /**< MFE parameter information */
    ApuInitXLOUDParam init_xloud_param; /**< MPP parameter information */
  };
  DebugDumpInfo       debug_dump_info;  /**< Debug dump information */
#if !defined(__CC_ARM)
};
#else
} __attribute__((transparent_union));
#endif

/**
 * Executing processing
 */
/* Type of src */

struct ApuExecSRCCmd
{
public:
};

/* Type of mfe */

struct Wien2ApuCmd;
struct ApuExecMFENotifcation
{
public:
  FAR Wien2ApuCmd *p_apu_cmd;     /**< Apu command pointer */
  BufferHeader     output_buffer; /**< Output buffer information */
                                  /**<  (including at least buffer address */
                                  /**<  or pointer and buffer size) */
};

struct ApuExecMFECmd
{
public:
  BufferHeader           ref_input_buffer;  /**< Reference input buffer information */
                                            /**<  (including at least buffer address */
                                            /**<  or pointer and buffer size) */
  ApuExecMFENotifcation  notification;      /**< MFE notification */
  FAR void              *p_eax_handle;      /**< EAX handle pointer */
};

/* type of mpp */

struct ApuExecXLOUDCmd
{
public:
  uint32_t  level;                    /**< xLOUD emphasis level */
  uint32_t  headroom_in;              /**< Head room of intput data */
  uint32_t  headroom_out;             /**< Head room of output data */
  uint32_t  volume_id;                /**< Volume */
  bool      enable_mute;              /**< Setting flag of mute state */
  bool      enable_hybrid_gain_mode;  /**< Gain and hybrid mode setting flag */
  uint32_t  eax_level;                /**< EAX adjustment level */
};

struct ApuExecFilterCmd
{
public:
  ApuFilterType  filter_type;    /**< Type of filter */
  BufferHeader   input_buffer;   /**< Input buffer information */
                                 /**<  (including at least buffer address */
                                 /**<  or pointer and buffer size) */
  BufferHeader   output_buffer;  /**< Output buffer information */
                                 /**<  (including at least buffer address */
                                 /**<  or pointer and buffer size) */
  union {
  ApuExecSRCCmd    exec_src_cmd;    /**< SRC parameter information */
  ApuExecMFECmd    exec_mfe_cmd;    /**< MFE parameter information */
  ApuExecXLOUDCmd  exec_xloud_cmd;  /**< MPP parameter information */
  };
#if !defined(__CC_ARM)
};
#else
} __attribute__((transparent_union));
#endif

/**
 * Flushing processing
 */

struct ApuFlushSRCCmd
{
  BufferHeader  output_buffer; /**< Output buffer information */
                               /**<  (including at least buffer address */
                               /**<  or pointer and buffer size) */
};

struct ApuFlushFilterCmd
{
public:
  ApuFilterType  filter_type;  /**< Type of filter */
  union {
  ApuFlushSRCCmd  flush_src_cmd;  /**< SRC parameter information */
  };
#if !defined(__CC_ARM)
};
#else
} __attribute__((transparent_union));
#endif

/**
 * Setting param processing
 */
/* Type of mfe */

struct SetMFEParam
{
public:
  int16_t  param_idx;  /**< Index of parameter */
  union
  {
    FAR void  *p_eax_handle;  /**< EAX handle pointer */
  };
#if !defined(__CC_ARM)
};
#else
} __attribute__((transparent_union));
#endif

/* Type of mpp */

struct SetMPPParam
{
public:
  int16_t  param_idx;  /**< Index of parameter */
  union {
  int16_t  xloud_vol;  /**< XLOUD volume */
  };
#if !defined(__CC_ARM)
};
#else
} __attribute__((transparent_union));
#endif

struct ApuSetParamFilterCmd
{
public:
  ApuFilterType  filter_type;  /**< Type of filter */
  union
  {
    SetMFEParam  set_mfe;  /**< MFE parameter information */
    SetMPPParam  set_mpp;  /**< MPP parameter information */
  };
#if !defined(__CC_ARM)
};
#else
} __attribute__((transparent_union));
#endif

/**
 * Tuning param processing
 */
/* Type of mfe */

struct TuningMFEParam
{
public:
  uint32_t  mic_delay;         /**< Mmic delay[ms] */
  uint32_t  ref_delay;         /**< Reference delay[ms] */
  uint32_t  mfe_config_table;  /**< MFE configuration address */
};

struct TuningXLOUDParam
{
public:
  uint32_t  xloud_config_table;  /**< XLOUD configuration address */
  uint32_t  xloud_param_table;   /**< XLOUD parameter table address */
  uint32_t  eax_config_table;    /**< EAX configuration address */
  uint32_t  eax_param_table;     /**< EAX parameter table address */
};

/* Type of mpp */

struct TuningMPPParam
{
public:
  int16_t  param_idx;  /**< Index of parameter */
  union
  {
    TuningXLOUDParam  tuning_xloud;
  };
#if !defined(__CC_ARM)
};
#else
} __attribute__((transparent_union));
#endif

struct ApuTuningFilterCmd
{
public:
  ApuFilterType  filter_type;  /**< Type of filter */
  union
  {
    TuningMFEParam  tuning_mfe;  /**< MFE parameter information */
    TuningMPPParam  tuning_mpp;  /**< MPP parameter information */
  };
#if !defined(__CC_ARM)
};
#else
} __attribute__((transparent_union));
#endif


/****************************************************************************/
/**
 *    Apu command for encoder control
 */
/**
 * Initializing processing
 */

struct ApuInitSBCEncParam
{
public:
  uint8_t  block_len;    /**< Block length */
  uint8_t  subband_num;  /**< Subband number */
  uint8_t  enc_type;     /**< Encoder type */
  uint8_t  reserved;     /**< Reserved */
};

struct ApuInitOPUSEncParam
{
public:
  uint32_t complexity;    /**< Computational complexity */
  bool      use_original_format; /**< Use original format */
};

struct ApuInitEncCmd
{
public:
  AudioCodec          codec_type;            /**< Codec type */
  uint8_t             channel_num;           /**< Channel number */
  uint32_t            bit_rate;              /**< Bit rate */
  uint32_t            input_sampling_rate;   /**< Sampling rate of input data */
  uint32_t            output_sampling_rate;  /**< Sampling rate of output data */
  AudioChannelConfig  channel_config;        /**< Channel config */
  AudioPcmBitWidth    bit_length;            /**< Bit length */
  union
  {
    ApuInitSBCEncParam  init_sbc_enc_param;  /**< SBC parameter information */
    ApuInitOPUSEncParam init_opus_enc_param; /**< OPUS parameter information */
  };
  DebugDumpInfo       debug_dump_info;       /**< Debug dump information */
#if !defined(__CC_ARM)
};
#else
} __attribute__((transparent_union));
#endif

/**
 * Executing processing
 */
struct ApuExecEncCmd
{
public:
  BufferHeader  input_buffer;  /**< Input buffer information */
                               /**<  (including at least buffer address */
                               /**<  or pointer and buffer size) */
  BufferHeader  output_buffer; /**< Output buffer information */
                               /**<  (including at least buffer address */
                               /**<  or pointer and buffer size) */
};

/**
 * Flushing processing
 */

struct ApuFlushEncCmd
{
public:
  BufferHeader  output_buffer; /**< Output buffer information */
                               /**<  (including at least buffer address */
                               /**<  or pointer and buffer size) */
};


/****************************************************************************/
/**
 *    Apu command for recognition control
 */
/**
 * Initializing processing
 */

struct ApuInitRecognitionCmd
{
public:
  ApuRecognitionType  recognition_type;  /**< Type of recognition process */
  AudioSamplingRate   sampling_rate;     /**< Sampling rate of input data */
  FAR uint8_t         *p_vad_param;      /**< VAD parameter buffer pointer */
  DebugDumpInfo       debug_dump_info;   /**< Debug dump information */
};

/**
 * Executing processing
 */

struct ApuExecRecognitionCmd
{
public:
  BufferHeader  input_buffer;  /**< Input buffer information */
                               /**<  (including at least buffer address */
                               /**<  or pointer and buffer size) */
  BufferHeader  output_buffer; /**< Output buffer information */
                               /**<  (including at least buffer address */
                               /**<  or pointer and buffer size) */
};

/**
 * Flushing processing
 */
struct ApuFlushRecognitionCmd
{
public:
  ApuFlushType  flush_type;    /**< Type of flush process */
  BufferHeader  output_buffer; /**< Output buffer information */
                               /**<  (including at least buffer address */
                               /**<  or pointer and buffer size) */
};


/****************************************************************************/
/**
 * Apuへの制御コマンド
 */

struct Wien2ApuCmd
{
  ApuCmdHeader  header;  /**< Apu command header */
  union
  {
    ApuInitDecCmd           init_dec_cmd;          /**< Parameters for initializing */
                                                   /**<  decoder processing */
    ApuInitFilterCmd        init_filter_cmd;       /**< Parameters for initializing */
                                                   /**<  filter processing */
    ApuInitEncCmd           init_enc_cmd;          /**< Parameters for initializing */
                                                   /**<  encoder processing */
    ApuInitRecognitionCmd   init_recognition_cmd;  /**< Parameters for initializing */
                                                   /**<  recognition processing */

    ApuExecDecCmd           exec_dec_cmd;          /**< Parameters for executing */
                                                   /**<  decoder processing */
    ApuExecFilterCmd        exec_filter_cmd;       /**< Parameters for executing */
                                                   /**<  filter processing */
    ApuExecEncCmd           exec_enc_cmd;          /**< Parameters for executing */
                                                   /**<  encoder processing */
    ApuExecRecognitionCmd   exec_recognition_cmd;  /**< Parameters for executing */
                                                   /**<  recognition processing */

    ApuFlushDecCmd          flush_dec_cmd;         /**< Parameters for flushing */
                                                   /**<  decoder processing */
    ApuFlushFilterCmd       flush_filter_cmd;      /**< Parameters for flushing */
                                                   /**<  filter processing */
    ApuFlushEncCmd          flush_enc_cmd;         /**< Parameters for flushing */
                                                   /**<  encoder processing */
    ApuFlushRecognitionCmd  flush_recognition_cmd; /**< Parameters for flushing */
                                                   /**<  recognition processing */

    ApuSetParamFilterCmd    setparam_filter_cmd;   /**< Parameters for setting */
                                                   /**<  filter processing */

    ApuTuningFilterCmd      tuning_filter_cmd;     /**< Parameters for tuning */
                                                   /**<  filter processing */
  };
  ApuResult  result;  /**< Retrun value of current command */
#if !defined(__CC_ARM)
};
#else
} __attribute__((transparent_union));
#endif

/****************************************************************************/
class SetErrorInfo
{

public:
  SetErrorInfo() {}
  ~SetErrorInfo() {}

  void set_error_info(FAR Wien2ApuCmd *cmd,
                      ApuInternalErrorSource res_src,
                      ApuInternalErrorCode code,
                      uint32_t debug_value)
    {
      cmd->result.exec_result = ApuExecError;
      cmd->result.internal_result[0].res_src = res_src;
      cmd->result.internal_result[0].code    = code;
      cmd->result.internal_result[0].value   = debug_value;
    }
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

__APU_END_NAMESPACE /* Namespace of Apu */
__WIEN2_END_NAMESPACE /* Namespace of Wien2 */

#endif /* __MODULES_AUDIO_INCLUDE_APUS_APU_CMD_H */

