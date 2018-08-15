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
 
/* ���|�U�[����config_length���Z�b�g����ꍇ�A
 * config_length_flag�ɂ��ȉ���define���Z�b�g���邱��
 */

#define LATM_ENABLE_CONFIG_LENGTH  1 /* config_length_flag�L�� */

/* ��q����\���̃����o���̒�` */

#define LATM_VAL_OF_4BIT    0xF
#define LATM_MAX_STREAM_ID  16       /* �X�g���[��ID�̍ő�l */
#define LATM_MIN_STREAM_ID  0        /* �X�g���[��ID�̍ŏ��l */

/* ���e�[�u���̔z��ł́A�X�g���[��ID(1�`16)��Y���Ɏg�p����̂�
 * �ő吔��+1����
 */

#define LATM_MAX_STREAM_ARRAY    (LATM_MAX_STREAM_ID + 1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/*
 * AudioSpecificConfig���
 *
 * StreamMuxConfig���́AStreamID���ɕێ�����
 * ���uuse_same_config�t���O=1�v���ɂ́A
 * 1�O��AudioSpecificConfig�ێ�����copy����
 */

struct info_audio_specific_config_s /* �Y������streamID([0]�͖��g�p) */
{
  /* ��LuseSameConfig=1�̎��́A���̑O��AudioSpecificConfig���e��copy */

  /* audioObjectType(�\����̍ő�l=95) */

  uint8_t audio_object_type;

  /* channelConfiguration(4-bit(�L���l0�`0xf)) */

  uint8_t channel_configuration;

  /* samplingFrequencyIndex(4-bit(�L���l0�`0xf)) */

  uint8_t sampling_frequency_index;

  /* program_config_element�g�p���ɁA�Ƃ肠�����ȉ��̍��ڂ����͕ێ����Ă���
   * �`PCE���S�����Ƒ傫������̂�
   */

  /* chanelConfiguration��0����program_config_element���ɐݒ肳�ꂽ
   * object type
   */

  uint8_t pce_object_type;

  /* chanelConfiguration��0����program_config_element���ɐݒ肳�ꂽ
   * samplingFrequencyIndex
   */

  uint8_t pce_sampling_frequency_index;

  /* �ȉ��͊g���p */

  int8_t   sbr_present_flag;     /* sbrPresentFlag */
  int8_t   ps_present_flag;      /* psPresentFlag */
  uint8_t  extension_audio_object_type; /* extensionAudioObjectType */

  /* extensionChannelConfiguration(ER-BSAC���̂�) */

  uint8_t  extension_channel_configuration;

  /* extensionSamplingFrequencyIndex */

  uint8_t  extension_sampling_frequency_index;

  /* extensionSamplingFrequency
   * (24-bit extensionSamplingFrequencyIndex=0xF(escape value)���Ɏg�p)
   */

  uint32_t extension_sampling_frequency;

  /* FS�l�͊g���p�ł͂Ȃ����A�\���̂̃A���C�����g���l�����čŌ���ɃZ�b�g */

  /* samplingFrequency
   * (24-bit samplingFrequencyIndex=0xF(escape value)���Ɏg�p)
   */

  uint32_t sampling_frequency;

  /* config_length�́A���[�U�[���ݒ�(���ݒ莞��0�Ƃ��Ĉ���) */

  /* config_length�l�̗L�������t���O(1=�L�� 1������) */

  uint8_t config_length_flag;
  uint8_t reserved;

  /* AudioSpecificConfig��bit�T�C�Y(��Lconfig_length_flag=1���ɂ̂ݗL��) */

  int config_length;

  /*----- �ȉ�SpecificConfig���(�T�C�Y���傫���Ȃ�̂ŃR�����g�A�E�g���Ă���) -----*/

#ifdef LATMTEST_DBG_COMMENT
  union
  {
    /* channelConfiguration=0����
     * GASpecificConfig����program_config_element()�����肾���A
     * parser�ɂ͕s�v
     */

    struct GASpecificConfig  ga;  /* AAC */
    struct SSCSpecificConfig ssc; /* AudioObjectType=28(SSC) */
  } spConfig;
#endif /* LATMTEST_DBG_COMMENT */
};
typedef struct info_audio_specific_config_s InfoAudioSpecificConfig;

struct info_stream_id_s /* �Y������streamID([0]�͖��g�p) */
{
  /* streamID�́A1�`16�̒l(infoStreamID[]�̓Y����)�B
   * ���̓J�E���^�Ȃ̂ŁA���g�p(=0)�ɂȂ�����ȍ~��ID�͑S�Ė��g�p
   */

  int8_t stream_id;     /* streamID */

  /*----- �ȍ~��streamID��0�̎��Ɏg�p -----*/

  /* �t����streamID */

  uint8_t prog;         /* streamID�ɑΉ�����program�ԍ� */
  uint8_t lay;          /* streamID�ɑΉ�����layer�ԍ� */

  /* �ȉ���streamID�ɑΉ����鍀�� */

  /* frameLengthType(�y�C���[�h�^�C�v) */

  uint8_t frame_length_type;

  /* frameLength(9-bit frameLengthType=1�̎��Ɏg�p) */

  uint32_t frame_length;

  /* latmBufferFullness(frameLengthType==0���̂�) */

  uint8_t latm_buffer_fullness;

  /* useSameConfig(=1�̏ꍇ�A�X�g���[����ł�AudioSpecificConfig�ȗ������) */

  uint8_t use_same_config;

  /* useSameConfig�l�ɂ�����炸�AAudioSpecificConfig��p��
   * (�_���I�ɂ́AStreamMuxConfig�����݂��Ă�AudioSpecificConfig��
   * ���݂��Ȃ��P�[�X�����邽��)
   */

  InfoAudioSpecificConfig asc;

  /* LATM�擪�����offset
   * �Ή�����payload��offset�l(StreamMuxConfig�����݂���ꍇ�̂�)
   */

  uint32_t payload_offset;
};
typedef struct info_stream_id_s InfomationStreamID;

struct info_stream_frame_s /* �Y������streamID([0]�͖��g�p) */
{
    /* �ȉ���streamID�ɑΉ����鍀�� */

  /* frameLengthType(�y�C���[�h�^�C�v) */

  uint8_t frame_length_type;

  /* frameLength(9-bit frameLengthType=1�̎��Ɏg�p) */

  uint32_t frame_length;

  /* LATM�擪�����offset
   * �Ή�����payload��offset�l(StreamMuxConfig�����݂���ꍇ�̂�)
   */

  uint32_t payload_offset;
};
typedef struct info_stream_frame_s InfomationStreamFrame;

/* StreamMuxConfig���̂����A
 * ���򔻒�̂��߃��[�U�[���ŕێ����Ă����Ăق����\���̏��
 *
 * [�g�p���@]
 * 1. �񋟂���API�֐���1��ڂ̎g�p�O�ɁA�{�\���̃T�C�Y�̃o�b�t�@���m��
 * 2. API�֐����R�[������ۂ̈���2���u�\���̃o�b�t�@�̐擪�v�ɂ���
 * 3. �ȍ~�AAPI�֐���A���g�p����Ԃ́A���o�b�t�@��������Ȃ�
 *   (LATM�t���[����A�����ēǂݏo���Ԃ͉�����Ȃ�)
 *
 * ���\���̃����o�́uinfo_stream_id[].asc.config_length�v�́A���[�U�[���Őݒ�
 *   �E�E�EAudioConfigSpecific()�̃T�C�Y���킩���Ă���ꍇ�A
 *         ���̃T�C�Y��bit���ŃZ�b�g
 *         �s���ȏꍇ�A0���Z�b�g
 */

struct info_stream_mux_config_s
{
  /* �g�p����streamID�̍ő�l(�L���l1�`16)�| 0�͖��g�p�̂��߁A�����ڂ̎Q�ƕs�� */

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

  /* 6-bit(�L���l0�`63) numSubFrames */

  uint8_t num_sub_frames;

  /* 4-bit(�L���l0�`15) numProgram */

  uint8_t num_program;

  /* 3-bit(�L���l0�`7)(�Y������program�ԍ�) numLayer */

  uint8_t num_layer[(LATM_VAL_OF_4BIT + 1)];

  /* otherDataLenBits(otherDataPresent=1�̎��Ɏg�p) */

  uint32_t other_data_len_bits;

  /* �ȉ���2�́AstreamID�l-1�ŃZ�b�g����邽�߁A�ʂɈ��� */

  /* progSIndx(allStreamsSameTimeFraming=0�̎��Ɏg�p) */

  uint8_t   prog_stream_indx[LATM_MAX_STREAM_ID];

  /* laySIndx(allStreamsSameTimeFraming=0�̎��Ɏg�p) */

  uint8_t   lay_stream_indx[LATM_MAX_STREAM_ID];

  /* �\����͍ő�uProgram�~Layer�v�����AstreamIdx=streamCnt=streamID�ł���A
   * �����̍ő��16�̂͂�
   */

  /* �Y������streamID([0]�͖��g�p) */

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
 * ��LATM�擪���擾����API
 *
 * ����1 : LOAS/LATM�̐擪(�Ⴆ�΁A�y�C���[�h�̐擪)
 * ����2 : ��L�Ŋm�ۂ����\���̃o�b�t�@�̐擪
 *
 * �߂�l : ����1�Ŏn�܂�LATM�t���[���̎�LATM�擪�A�h���X
 *          0=NG(LATM�w�b�_���f�[�^�ɂ���AAudioObjectType���u���T�|�[�g�v)
 */
FAR uint8_t *AACLC_getNextLatm(FAR uint8_t *ptr_readbuff,
                               FAR InfoStreamMuxConfig *ptr_stream_mux_config);

#endif /* __MODULES_AUDIO_INCLUDE_COMMON_LATMAACLC_H_ */
