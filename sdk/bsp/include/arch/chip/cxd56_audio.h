/****************************************************************************
 * bsp/include/arch/chip/cxd56_audio.h
 *
 *   Copyright (C) 2016, 2017 Sony Corporation
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
 ***************************************************************************/
/**
 * @defgroup audioutils Audio Utility
 * @{
 */

#ifndef __BSP_INCLUDE_ARCH_CHIP_CXD56_AUDIO_H
#define __BSP_INCLUDE_ARCH_CHIP_CXD56_AUDIO_H

/* API Documents creater with Doxgen */

/**
 * @defgroup audioutils_audio_common_api Audio Common API
 * @{
 *
 * @file       cxd56_audio.h
 * @brief      Spritzer Audio Common API
 * @author     Spritzer Audio SW Team
 */

/****************************************************************************
 * Included Files
 ***************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include <arch/chip/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

#define CXD56_AUDIO_IOCTL_BB_UNDEF              (0x00 << 8)
#define CXD56_AUDIO_IOCTL_BB_POWER              (0x01 << 8)
#define CXD56_AUDIO_IOCTL_BB_COMMON             (0x02 << 8)
#define CXD56_AUDIO_IOCTL_BB_INPUT              (0x03 << 8)
#define CXD56_AUDIO_IOCTL_BB_OUTPUT             (0x04 << 8)
#define CXD56_AUDIO_IOCTL_BB_MICGAIN            (0x05 << 8)
#define CXD56_AUDIO_IOCTL_BB_VOLUME             (0x06 << 8)
#define CXD56_AUDIO_IOCTL_BB_CLEARSTEREO        (0x07 << 8)
#define CXD56_AUDIO_IOCTL_BB_BEEP               (0x08 << 8)
#define CXD56_AUDIO_IOCTL_BB_DATAPATH           (0x09 << 8)

#define AUDIO_IOCTL_TYPE_UNDEF                  0x00
#define AUDIO_IOCTL_TYPE_RESET                  0x01
#define AUDIO_IOCTL_TYPE_INIT                   0x02
#define AUDIO_IOCTL_TYPE_SET                    0x03
#define AUDIO_IOCTL_TYPE_CLEAR                  0x04
#define AUDIO_IOCTL_TYPE_ENABLE                 0x05
#define AUDIO_IOCTL_TYPE_DISABLE                0x06
#define AUDIO_IOCTL_TYPE_MUTE                   0x07
#define AUDIO_IOCTL_TYPE_UNMUTE                 0x08

#define AUDIO_CXD5247_AVDD  0x01
#define AUDIO_CXD5247_DVDD  0x02
#define AUDIO_CXD5247       (AUDIO_CXD5247_AVDD | AUDIO_CXD5247_DVDD)
#define AUDIO_CXD56xx       0x04

typedef enum
{
  RI_REVID,
  RI_DEVICEID,
  RI_PDN_AMICEXT,
  RI_PDN_AMIC1,
  RI_PDN_AMIC2,
  RI_PDN_DAC,
  RI_PDN_LINEIN,
  RI_PDN_MIC,
  RI_PDN_DMIC,
  RI_PDN_DSPB,
  RI_PDN_ANC,
  RI_PDN_DNC1,
  RI_PDN_DNC2,
  RI_PDN_SMSTR,
  RI_PDN_DSPS2,
  RI_PDN_DSPS1,
  RI_PDN_DSPC,
  RI_FS_FS,
  RI_DECIM0_EN,
  RI_DECIM1_EN,
  RI_SDES_EN,
  RI_MCK_AHBMSTR_EN,
  RI_AU_DAT_SEL2,
  RI_AU_DAT_SEL1,
  RI_COD_INSEL3,
  RI_COD_INSEL2,
  RI_COD_INSEL1,
  RI_DSR_RATE,
  RI_DIGSFT,
  RI_SRC1,
  RI_SRC1IN_SEL,
  RI_SRC2,
  RI_SRC2IN_SEL,
  RI_DIF2,
  RI_DIF1,
  RI_SD2MASTER,
  RI_SD1MASTER,
  RI_SDCK_OUTENX,
  RI_HI_RES_MODE,
  RI_HPF2_MODE,
  RI_CIC2IN_SWAP,
  RI_INV_DMIC2L,
  RI_INV_DMIC2R,
  RI_CIC2_GAIN_MODE,
  RI_CIC2IN_SEL,
  RI_ADC2_BOOST,
  RI_INV_AMIC2L,
  RI_INV_AMIC2R,
  RI_HPF1_MODE,
  RI_CIC1IN_SWAP,
  RI_INV_DMIC1L,
  RI_INV_DMIC1R,
  RI_CIC1_GAIN_MODE,
  RI_CIC1IN_SEL,
  RI_ADC1_BOOST,
  RI_ADC_FS,
  RI_INV_AMIC1L,
  RI_INV_AMIC1R,
  RI_HPF4_MODE,
  RI_CIC4IN_SWAP,
  RI_INV_DMIC4L,
  RI_INV_DMIC4R,
  RI_CIC4IN_SEL,
  RI_ADC4_BOOST,
  RI_INV_AMIC4L,
  RI_INV_AMIC4R,
  RI_HPF3_MODE,
  RI_CIC3IN_SWAP,
  RI_INV_DMIC3L,
  RI_INV_DMIC3R,
  RI_CIC3IN_SEL,
  RI_ADC_3_BOOST,
  RI_INV_AMIC3L,
  RI_INV_AMIC3R,
  RI_CIC1_RGAIN,
  RI_CIC1_LGAIN,
  RI_CIC2_RGAIN,
  RI_CIC2_LGAIN,
  RI_CIC3_RGAIN,
  RI_CIC3_LGAIN,
  RI_CIC4_RGAIN,
  RI_CIC4_LGAIN,
  RI_SPC_LIMIT,
  RI_SPC_EN,
  RI_ALC_KNEE,
  RI_ALCTARGET,
  RI_ALC_REC,
  RI_ALC_EN,
  RI_INV_ASP2R,
  RI_INV_ASP2L,
  RI_INV_ASP1R,
  RI_INV_ASP1L,
  RI_ARC,
  RI_ARC_VOL,
  RI_CS_VOL,
  RI_CS_SIGN,
  RI_SDOUT_VOL,
  RI_SDIN2_VOL,
  RI_SDIN1_VOL,
  RI_SDIN1_EN,
  RI_SDIN2_EN,
  RI_SDOUT1_EN,
  RI_SDOUT2_EN,
  RI_MUTE_B,
  RI_BLF_EN,
  RI_TRANS_MODE,
  RI_DAC_VOL,
  RI_LINEIN_VOL,
  RI_BEEP_VOL,
  RI_BEEP_FREQ,
  RI_BEEP_ON,
  RI_M_SPCLKERR1,
  RI_M_SPCLKERR2,
  RI_ADC1L_VOL,
  RI_ADC1R_VOL,
  RI_ADC2L_VOL,
  RI_ADC2R_VOL,
  RI_SMS_INTIM,
  RI_DNC2_AVF,
  RI_DNC2_MONION1,
  RI_DNC2_MONIEN1,
  RI_DNC2_MONION0,
  RI_DNC2_MONIEN0,
  RI_DNC1_AVF,
  RI_DNC1_MONION1,
  RI_DNC1_MONIEN1,
  RI_DNC1_MONION0,
  RI_DNC1_MONIEN0,
  RI_DNC2_CFMD,
  RI_DNC2_ESS,
  RI_DNC2_ZWR,
  RI_DNC2_MUTE,
  RI_DNC2_START,
  RI_DNC1_CFMD,
  RI_DNC1_ESS,
  RI_DNC1_ZWR,
  RI_DNC1_MUTE,
  RI_DNC1_START,
  RI_DNC_STB,
  RI_DCMFS_34,
  RI_DNC_512,
  RI_DCMFS,
  RI_DNC1_CANVOL1,
  RI_DNC1_CANVOL0,
  RI_DNC2_CANVOL1,
  RI_DNC2_CANVOL0,
  RI_DNC1_MONVOL1,
  RI_DNC1_MONVOL0,
  RI_DNC2_MONVOL1,
  RI_DNC2_MONVOL0,
  RI_DNC1_ALGAIN1,
  RI_DNC1_ALGAIN0,
  RI_DNC2_ALGAIN1,
  RI_DNC2_ALGAIN0,
  RI_DNC_PHD,
  RI_DNC1_LIMIYT,
  RI_DNC1_LMTON0,
  RI_DNC1_LIMITR,
  RI_DNC1_LIMITA,
  RI_DNC1_INSTMD,
  RI_DNC2_LIMIYT,
  RI_DNC2_LMTON0,
  RI_DNC2_LIMITR,
  RI_DNC2_LIMITA,
  RI_DNC2_INSTMD,
  RI_ANC_FALVL,
  RI_ANC_TST,
  RI_ANC_FATST,
  RI_ENVREG_RESET,
  RI_ANC_CHSEL,
  RI_ANC_TR,
  RI_ANC_TA,
  RI_ANC_SOUT,
  RI_ANC_FASPN,
  RI_ANC_ZWR,
  RI_ANC_MUTE,
  RI_ANC_START,
  RI_ANC_FASTART,
  RI_ANC_FAWTB,
  RI_ANC_FAWTA,
  RI_ANC_ENV1,
  RI_ANC_ENV0,
  RI_ANC_CURST,
  RI_ANC_FAST,
  RI_ANC_ENV2,
  RI_NS_AMMD,
  RI_BPGAIN,
  RI_BPSEL,
  RI_NSDI,
  RI_NSII,
  RI_BPON,
  RI_NSMS,
  RI_CHSEL,
  RI_NSADJON,
  RI_NSX2,
  RI_NSPMUTE,
  RI_NSDD,
  RI_OUT2DLY,
  RI_NSAD,
  RI_PWMMD,
  RI_NSAS,
  RI_NSADJ,
  RI_VCONT,
  RI_TEST_OUT,
  RI_TEST_OUT_SEL0,
  RI_TEST_OUT_SEL1,
  RI_TEST_IN,
  RI_S_RESET,
  RI_HALT_INHIBIT,
  RI_FSRDBGMD,
  RI_BEEP_TEST,
  RI_ARWPHSET,
  RI_DSPRAM4_CLR,
  RI_DSPRAM3_CLR,
  RI_DSPRAM2_CLR,
  RI_DSPRAM1_CLR,
  RI_ALC_DELAY,
  RI_ALC_ALG,
  RI_ARC_TIMER,
  RI_ARC_DLY,
  RI_SPC_AWEIGHT,
  RI_SPC_ALC_ATTACK,
  RI_SPC_ALC_RELEASE,
  RI_ALC_LPF,
  RI_SPC_ENERGY,
  RI_W_RSRV,
  RI_R_RSRV,
  RI_SER_MODE,
  RI_PDM_OUT_EN,
  RI_FS_CLK_EN,
  RI_SEL_OUT4_R,
  RI_SEL_OUT4_L,
  RI_SEL_OUT3_R,
  RI_SEL_OUT3_L,
  RI_SEL_OUT2_R,
  RI_SEL_OUT2_L,
  RI_SEL_OUT1_R,
  RI_SEL_OUT1_L,
  RI_OUTEN_MIC1L_B,
  RI_OUTEN_MIC1R_B,
  RI_OUTEN_MIC2L_B,
  RI_OUTEN_MIC2R_B,
  RI_OUTEN_MIC1L_A,
  RI_OUTEN_MIC1R_A,
  RI_OUTEN_MIC2L_A,
  RI_OUTEN_MIC2R_A,
  RI_SEL_OUTF,
  RI_SEL_INF,
  RI_SEL_DECIM,
  RI_DEQ_COEF_1B0,
  RI_DEQ_EN,
  RI_DEQ_COEF_1B1,
  RI_DEQ_COEF_1B2,
  RI_DEQ_COEF_1A1,
  RI_DEQ_COEF_1A2,
  RI_DEQ_COEF_2B0,
  RI_DEQ_COEF_2B1,
  RI_DEQ_COEF_2B2,
  RI_DEQ_COEF_2A1,
  RI_DEQ_COEF_2A2,
  RI_DEQ_COEF_3B0,
  RI_DEQ_COEF_3B1,
  RI_DEQ_COEF_3B2,
  RI_DEQ_COEF_3A1,
  RI_DEQ_COEF_3A2,
  RI_DEQ_COEF_4B0,
  RI_DEQ_COEF_4B1,
  RI_DEQ_COEF_4B2,
  RI_DEQ_COEF_4A1,
  RI_DEQ_COEF_4A2,
  RI_DEQ_COEF_5B0,
  RI_DEQ_COEF_5B1,
  RI_DEQ_COEF_5B2,
  RI_DEQ_COEF_5A1,
  RI_DEQ_COEF_5A2,
  RI_DEQ_COEF_6B0,
  RI_DEQ_COEF_6B1,
  RI_DEQ_COEF_6B2,
  RI_DEQ_COEF_6A1,
  RI_DEQ_COEF_6A2,
  RI_LR_SWAP1,
  RI_LR_SWAP2,
  RI_DUMMY_0,
  RI_DUMMY_1,
  RI_DUMMY_2,
  RI_DUMMY_3,
  RI_DUMMY_4,
  RI_DUMMY_5,
  RI_DUMMY_6,
  RI_DUMMY_7,
  RI_DUMMY_8,
  RI_DUMMY_9,
  RI_DUMMY_10,
  RI_DUMMY_11,
  RI_DUMMY_12,
  RI_DUMMY_13,
  RI_DUMMY_14,
  RI_DUMMY_15,
  RI_DUMMY_16,
  RI_DUMMY_17,
  RI_DUMMY_18,
  RI_DUMMY_19,
  RI_DUMMY_20,
  RI_DUMMY_21,
  RI_DUMMY_22,
  RI_DUMMY_23,
  RI_DUMMY_24,
  RI_DUMMY_25,
  RI_DUMMY_26,
  RI_DUMMY_27,
  RI_DUMMY_28,
  RI_DUMMY_29,
  RI_DUMMY_30,
  RI_DUMMY_31,
  RI_RAM_RW_EN,
  RI_REG_MAX_ENTRY,
} AC_REG_ID;

typedef enum
{
  BCA_Mic_In_start_adr,
  BCA_Mic_In_sample_no,
  BCA_Mic_In_rtd_trg,
  BCA_Mic_In_nointr,
  BCA_Mic_In_bitwt,
  BCA_Mic_In_ch8_sel,
  BCA_Mic_In_ch7_sel,
  BCA_Mic_In_ch6_sel,
  BCA_Mic_In_ch5_sel,
  BCA_Mic_In_ch4_sel,
  BCA_Mic_In_ch3_sel,
  BCA_Mic_In_ch2_sel,
  BCA_Mic_In_ch1_sel,
  BCA_Mic_In_start,
  BCA_Mic_In_error_setting,
  BCA_Mic_In_monbuf,

  BCA_I2s1_In_start_adr,
  BCA_I2s1_In_sample_no,
  BCA_I2s1_In_rtd_trg,
  BCA_I2s1_In_nointr,
  BCA_I2s1_In_bitwt,
  BCA_I2s1_In_ch2_sel,
  BCA_I2s1_In_ch1_sel,
  BCA_I2s1_In_Mon_start,
  BCA_I2s1_In_Mon_error_setting,
  BCA_I2s1_In_Mon_monbuf,
  BCA_I2s2_In_start_adr,
  BCA_I2s2_In_sample_no,
  BCA_I2s2_In_rtd_trg,
  BCA_I2s2_In_nointr,
  BCA_I2s2_In_bitwt,
  BCA_I2s2_In_ch2_sel,
  BCA_I2s2_In_ch1_sel,
  BCA_I2s2_In_Mon_start,
  BCA_I2s2_In_Mon_error_setting,
  BCA_I2s2_In_Mon_monbuf,

  BCA_I2s1_Out_start_adr,
  BCA_I2s1_Out_sample_no,
  BCA_I2s1_Out_rtd_trg,
  BCA_I2s1_Out_nointr,
  BCA_I2s1_Out_bitwt,
  BCA_I2s1_Out_sd1_r_sel,
  BCA_I2s1_Out_sd1_l_sel,
  BCA_I2s1_Out_Mon_start,
  BCA_I2s1_Out_Mon_error_setting,
  BCA_I2s1_Out_Mon_monbuf,
  BCA_I2s2_Out_start_adr,
  BCA_I2s2_Out_sample_no,
  BCA_I2s2_Out_rtd_trg,
  BCA_I2s2_Out_nointr,
  BCA_I2s2_Out_bitwt,
  BCA_I2s2_Out_sd1_r_sel,
  BCA_I2s2_Out_sd1_l_sel,
  BCA_I2s2_Out_Mon_start,
  BCA_I2s2_Out_Mon_error_setting,
  BCA_I2s2_Out_Mon_monbuf,

  BCA_I2s_ensel,
  BCA_Mic_In_prdat_u,
  BCA_I2s1_In_prdat_u,
  BCA_I2s2_In_prdat_u,
  BCA_I2s1_Out_prdat_d,
  BCA_I2s2_Out_prdat_d,

  BCA_Mic_Int_Ctrl_done_mic,
  BCA_Mic_Int_Ctrl_err_mic,
  BCA_Mic_Int_Ctrl_smp_mic,
  BCA_Mic_Int_Ctrl_cmb_mic,
  BCA_I2s1_Int_Ctrl_done_i2so,
  BCA_I2s1_Int_Ctrl_err_i2so,
  BCA_I2s1_Int_Ctrl_done_i2si,
  BCA_I2s1_Int_Ctrl_err_i2si,
  BCA_I2s1_Int_Ctrl_smp_i2s,
  BCA_I2s1_Int_Ctrl_cmb_i2s,
  BCA_I2s2_Int_Ctrl_done_i2so,
  BCA_I2s2_Int_Ctrl_err_i2so,
  BCA_I2s2_Int_Ctrl_done_i2si,
  BCA_I2s2_Int_Ctrl_err_i2si,
  BCA_I2s2_Int_Ctrl_smp_i2s,
  BCA_I2s2_Int_Ctrl_cmb_i2s,

  BCA_Mic_Int_Mask_done_mic,
  BCA_Mic_Int_Mask_err_mic,
  BCA_Mic_Int_Mask_smp_mic,
  BCA_Mic_Int_Mask_cmb_mic,
  BCA_Mic_Int_Mask_nostpmsk,
  BCA_Mic_Int_Mask_srst_mic,
  BCA_I2s1_Int_Mask_done_i2so,
  BCA_I2s1_Int_Mask_err_i2so,
  BCA_I2s1_Int_Mask_done_i2si,
  BCA_I2s1_Int_Mask_err_i2si,
  BCA_I2s1_Int_Mask_smp_i2s,
  BCA_I2s1_Int_Mask_cmb_i2s,
  BCA_I2s1_Int_Mask_nostpmsk,
  BCA_I2s1_Int_Mask_srst_i2s,
  BCA_I2s2_Int_Mask_done_i2so,
  BCA_I2s2_Int_Mask_err_i2so,
  BCA_I2s2_Int_Mask_done_i2si,
  BCA_I2s2_Int_Mask_err_i2si,
  BCA_I2s2_Int_Mask_smp_i2s,
  BCA_I2s2_Int_Mask_cmb_i2s,
  BCA_I2s2_Int_Mask_nostpmsk,
  BCA_I2s2_Int_Mask_srst_i2s,

  BCA_Int_m_hresp_err,
  BCA_Int_m_i2s1_bck_err1,
  BCA_Int_m_i2s1_bck_err2,
  BCA_Int_m_anc_faint,
  BCA_Int_m_ovf_smasl,
  BCA_Int_m_ovf_smasr,
  BCA_Int_m_ovf_dnc1l,
  BCA_Int_m_ovf_dnc1r,
  BCA_Int_m_ovf_dnc2l,
  BCA_Int_m_ovf_dnc2r,
  BCA_Int_clr_hresp_err,
  BCA_Int_clr_i2s1_bck_err1,
  BCA_Int_clr_i2S1_bck_err2,
  BCA_Int_clr_anc_faint,
  BCA_Int_clr_ovf_smasl,
  BCA_Int_clr_ovf_smasr,
  BCA_Int_clr_ovf_dnc1l,
  BCA_Int_clr_ovf_dnc1r,
  BCA_Int_clr_ovf_dnc2l,
  BCA_Int_clr_ovf_dnc2r,
  BCA_Int_hresp_err,
  BCA_Int_i2s_bck_err1,
  BCA_Int_i2s_bck_err2,
  BCA_Int_anc_faint,
  BCA_Int_ovf_smasl,
  BCA_Int_ovf_smasr,
  BCA_Int_ovf_dnc1l,
  BCA_Int_ovf_dnc1r,
  BCA_Int_ovf_dnc2l,
  BCA_Int_ovf_dnc2r,

  BCA_Dbg_Mic_ch1_data,
  BCA_Dbg_Mic_ch2_data,
  BCA_Dbg_Mic_ch3_data,
  BCA_Dbg_Mic_ch4_data,
  BCA_Dbg_Mic_ch5_data,
  BCA_Dbg_Mic_ch6_data,
  BCA_Dbg_Mic_ch7_data,
  BCA_Dbg_Mic_ch8_data,
  BCA_Dbg_I2s1_u_ch1_data,
  BCA_Dbg_I2s1_u_ch2_data,
  BCA_Dbg_I2s1_d_ch1_data,
  BCA_Dbg_I2s1_d_ch2_data,
  BCA_Dbg_I2s2_u_ch1_data,
  BCA_Dbg_I2s2_u_ch2_data,
  BCA_Dbg_I2s2_d_ch1_data,
  BCA_Dbg_I2s2_d_ch2_data,
  BCA_Dbg_Ctrl_mic_dbg_en,
  BCA_Dbg_Ctrl_I2s1_dbg_u_en,
  BCA_Dbg_Ctrl_I2s1_dbg_d_en,
  BCA_Dbg_Ctrl_I2s2_dbg_u_en,
  BCA_Dbg_Ctrl_I2s2_dbg_d_en,

  BCA_Clk_En_ahbmstr_mic_en,
  BCA_Clk_En_ahbmstr_I2s1_en,
  BCA_Clk_En_ahbmstr_I2s2_en,
  BCA_Mclk_Mon_thresh,

  AHB_Master_Mic_Mask,
  AHB_Master_I2s1_Mask,
  AHB_Master_I2s2_Mask,

  BCA_REG_MAX_ENTRY
} BCA_REG_ID;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** keep setting for #InitMicGainParam.mic_gain */
#define AS_MICGAIN_HOLD         215
/** maxinum value for above parameters */
#define AS_MICGAINA_MAX         210
/** minimum value for above parameters */
#define AS_MICGAINA_MIN         0
/** mute setting for above parameters */
#define AS_MICGAIN_MUTE         -7855
/** maxinum value for above parameters */
#define AS_MICGAIND_MAX         0
/** minimum value for above parameters */
#define AS_MICGAIND_MIN         -7850

/** keep setting for #InitClearStereoParam.cs_vol */
#define AS_CS_VOL_HOLD          255
/** maximum value for above parameter */
#define AS_CS_VOL_MAX           -195
/** minimum value for above parameter */
#define AS_CS_VOL_MIN           -825
/** inf zero setting for above parameter */
#define AS_CS_VOL_INF_ZERO      -830

/** keep setting for #SetVolumeParam.input1_db,
 * #SetVolumeParam.input2_db,
 * #SetVolumeParam.master_db
 */
#define AS_VOLUME_HOLD          255
/** maximum value for above parameters */
#define AS_VOLUME_MAX           120
/** minimum value for above parameters */
#define AS_VOLUME_MIN           -1020
/** mute setting for above parameters */
#define AS_VOLUME_MUTE          -1025

/** keep setting for #SetBeepParam.beep_vol */
#define AS_BEEP_VOL_HOLD        255
/** maximum value for above parameter */
#define AS_BEEP_VOL_MAX         0
/** minimum value for above parameter */
#define AS_BEEP_VOL_MIN         -90

/** keep setting for #SetBeepParam.beep_freq */
#define AS_BEEP_FREQ_HOLD       0
/** maximum value for above parameter */
#define AS_BEEP_FREQ_MAX        4085
/** minimum value for above parameter */
#define AS_BEEP_FREQ_MIN        94

/** mic channel max */
#define AS_MIC_CHANNEL_MAX      8

/* INT_EN1 */

#define AS_INT_EN1_REG          0
#define AS_INT_EN1_BIT_AU0      6
#define AS_INT_EN1_BIT_AU1      7
#define AS_INT_EN1_BIT_AU2      8
#define AS_INT_EN1_BIT_AU3      9

/* INT_IRQ1 */

#define AS_INT_IRQ1_REG         1
#define AS_INT_IRQ1_BIT_AU0     6
#define AS_INT_IRQ1_BIT_AU1     7
#define AS_INT_IRQ1_BIT_AU2     8
#define AS_INT_IRQ1_BIT_AU3     9

#define AUDMIC_IRQn    CXD56_IRQ_AUDIO_0
#define AUDI2S1_IRQn   CXD56_IRQ_AUDIO_1
#define AUDI2S2_IRQn   CXD56_IRQ_AUDIO_2
#define AUDCODEC_IRQn  CXD56_IRQ_AUDIO_3

/** Get DMAC Speaker Out */
typedef enum
{
  AS_MIXER1_OUT,                /* Mixer1 */
  AS_MIXER2_OUT,                /* Mixer2 */
  AS_MIXER_OUT_NONE,            /* Not Mixer out */
  AS_MIXER_OUT_PARAM            /* Param Error */
} asDmacMixerOutId;

/** Select numbers of the I2S */
typedef enum
{
  /** I2S1 */
  AS_I2S_ID_1,
  /** I2S2 */
  AS_I2S_ID_2,
  AS_I2S_ID_NUM
} asI2sId;

/** Select I2S SRC bypass mode */
typedef enum
{
  /** disable */
  AS_I2S_BP_MODE_DISABLE,
  /** enable */
  AS_I2S_BP_MODE_ENABLE,
  AS_I2S_BP_MODE_NUM
} asBypassModeId;

/** Select output device ID */
typedef enum
{
  /** output device none */
  AS_OUT_DEV_OFF,
  /** output device speaker */
  AS_OUT_DEV_SP,
  /** output device i2s */
  AS_OUT_DEV_I2S,
  AS_OUT_DEV_NUM
} asOutDeviceId;

/** Select DNC mode */
typedef enum
{
  /** DNC off */
  AS_DNC_OFF,
  /** DNC mode A */
  AS_DNC_A,
  /** DNC mode B */
  AS_DNC_B,
  /** DNC mode C */
  AS_DNC_C,
  AS_DNC_NUM
} asDncMode;

/** Select sampling data format */
typedef enum
{
  /** 24bit */
  AS_SAMPLING_FMT_24,
  /** 16bit */
  AS_SAMPLING_FMT_16,
  AS_SAMPLING_FMT_NUM
} asSampFmt;

/** Select CXD5247's X'tal frequency */
typedef enum
{
  /** 24.576MHz (default) */
  AS_XTAL_24_576MHZ,
  /** 49.152MHz */
  AS_XTAL_49_152MHZ,
  AS_XTAL_NUM
} AsXtalSelId;

/** Select Clock mode */
typedef enum
{
  /** Normal mode (default) */
  AS_CLK_MODE_NORMAL,
  /** Hi-Res mode */
  AS_CLK_MODE_HIRES,
  AS_CLK_MODE_NUM
} AsClkModeId;

/** Select the intensity of the output drive */
typedef enum
{
  /** Weakest */
  AS_IO_DS_WEAKEST,
  /** Weaker */
  AS_IO_DS_WEAKER,
  /** Stronger */
  AS_IO_DS_STRONGER,
  /** Strongest */
  AS_IO_DS_STRONGEST,
  AS_IO_DS_MAX_ENTRY
} asIoDsId;

/** Select PCM data path */
typedef enum
{
  AS_I2S_DATA_PATH_NONE,
  AS_I2S_DATA_PATH_1,
  AS_I2S_DATA_PATH_2,
  AS_I2S_DATA_PATH_NUM
} AsI2sDataPath;

/** Select PCM input mode/port */
typedef enum
{
  /** I2S Master mode */
  AS_I2S_DEVICE_I2S_MASTER,
  /** I2S Slave mode (default) */
  AS_I2S_DEVICE_I2S_SLAVE,
  /** SLIMbus Master mode */
  AS_I2S_DEVICE_NUM
} AsI2sDeviceId;

/** Select PCM format */
typedef enum
{
  /** I2S format */
  AS_I2S_FORMAT_I2S,
  /** Left Justified format (default) */
  AS_I2S_FORMAT_LEFT,
  AS_I2S_FORMAT_NUM
} AsI2sFormatId;

/** Select Audio Codec CIC input source */
typedef enum
{
  /** CIC input none */
  AS_CIC_IN_SEL_NONE,
  /** from CXD5247 (default) */
  AS_CIC_IN_SEL_CXD,
  /** from DMIC */
  AS_CIC_IN_SEL_DMIC,
  AS_CIC_IN_SEL_NUM
} AsCicInputSelId;

/** Select drive current */
typedef enum
{
  /** 4mA */
  AS_LOWEMI_4MA,
  /** 2mA (default) */
  AS_LOWEMI_2MA,
  AS_LOWEMI_NUM
} AsLowemi;

/** Select ALC/SPC mode */
typedef enum
{
  /** ALC */
  AS_ALC_SPC_SEL_ALC,
  /** SPC */
  AS_ALC_SPC_SEL_SPC,
  /** OFF */
  AS_ALC_SPC_SEL_OFF,
  AS_ALC_SPC_SEL_MAX_ENTRY
} asAlcSpcSelId;

/** Select DMAC data format */
typedef enum
{
  /** Data format LR */
  AS_DMA_DATA_FORMAT_LR,
  /** Data format RL */
  AS_DMA_DATA_FORMAT_RL,
  AS_DMA_DATA_FORMAT_MAX_ENTRY
} AsDmaDataFormat;

/** Select HPADC MIC bias mode */
typedef enum
{
  /** ON */
  AS_HPADC_MIC_BIAS_ON,
  /** OFF */
  AS_HPADC_MIC_BIAS_OFF,
  AS_HPADC_MIC_BIAS_MAX_ENTRY
} AsHpadcMicBias;

/** Select Audio block clock */
typedef enum {
  /** Audio Block Clock 24M */
  AS_AUDIO_BLOCK_CLOCK_24M = 0,
  /** Audio Block Clock 49M */
  AS_AUDIO_BLOCK_CLOCK_49M = 1,
} AsAudioBlockClock;

/** BaseBand driver common return code */
typedef enum
{
  E_AS_OK,                                /* 0 */
  /* Power OFF/ON */
  E_AS_PWON_NG,                           /* 1 */
  /* Power OFF/ON chk */
  E_AS_POWER_OFF_CHK_ERR,                 /* 2 */
  E_AS_IN_POWER_ON_CHK_ERR,               /* 3 */
  E_AS_OUT_POWER_ON_CHK_ERR,              /* 4 */
  E_AS_CS_POWER_ON_CHK_ERR,               /* 5 */
  E_AS_DEQ_POWER_ON_CHK_ERR,              /* 6 */
  E_AS_DNC_POWER_ON_CHK_ERR,              /* 7 */
  E_AS_MIC_POWER_ON_CHK_ERR,              /* 8 */
  E_AS_OUTSEL_POWER_ON_CHK_ERR,           /* 9 */
  E_AS_I2S_POWER_ON_CHK_ERR,              /* 10 */
  E_AS_SETVOL_POWER_ON_OUT_CHK_ERR,       /* 11 */
  E_AS_MUTE_POWER_ON_OUT_CHK_ERR,         /* 12 */
  E_AS_UNMUTE_POWER_ON_OUT_CHK_ERR,       /* 13 */
  E_AS_SETBEEP_POWER_ON_OUT_CHK_ERR,      /* 14 */
  E_AS_BEEPDIS_POWER_ON_OUT_CHK_ERR,      /* 15 */
  E_AS_BEEPENA_POWER_ON_OUT_CHK_ERR,      /* 16 */
  E_AS_SET_DP_POWER_ON_CHK_ERR,           /* 17 */
  E_AS_CLR_DP_POWER_ON_CHK_ERR,           /* 18 */
  E_AS_CLRALL_DP_POWER_ON_CHK_ERR,        /* 19 */
  E_AS_ACTDMAC_POWER_ON_CHK_ERR,          /* 20 */
  E_AS_INITDMAC_POWER_ON_CHK_ERR,         /* 21 */
  E_AS_STARTDMAC_POWER_ON_CHK_ERR,        /* 22 */
  E_AS_READDMAC_POWER_ON_CHK_ERR,         /* 23 */
  E_AS_WRITEDMAC_POWER_ON_CHK_ERR,        /* 24 */
  E_AS_STOPDMAC_POWER_ON_CHK_ERR,         /* 25 */
  E_AS_GETREADYCMDDMAC_POWER_ON_CHK_ERR,  /* 26 */
  E_AS_DEACTDMAC_POWER_ON_CHK_ERR,        /* 27 */

  /* PowerManager ON chk */
  E_AS_PM_ADONIS_PWON_CHK_ERR,            /* 28 */
  E_AS_PM_AUDIO_PWON_CHK_ERR,             /* 29 */
  /* Audio Codec ID */
  E_AS_AC_ID_NG,                          /* 30 */
  /* AcaPulco ID */
  E_AS_ACAPULCO_ID_NG,                    /* 31 */
  /* BaseBandConfig */
  E_AS_AD_DATA_DS_PARAM,                  /* 32 */
  E_AS_ALC_SPC_SEL_PARAM,                 /* 33 */
  E_AS_ALC_KNEE_PARAM,                    /* 34 */
  E_AS_ALC_TARGET_PARAM,                  /* 35 */
  E_AS_SPC_LIMIT_PARAM,                   /* 36 */
  E_AS_CIC_INPUT_SEL_PARAM,               /* 37 */
  E_AS_CLK_MODE_PARAM,                    /* 38 */
  E_AS_DMIC_CLK_DS_PARAM,                 /* 39 */
  E_AS_GPO_DS_PARAM,                      /* 40 */
  E_AS_I2S_DATA_PATH_PARAM,               /* 41 */
  E_AS_I2S_DEVICE_1_PARAM,                /* 42 */
  E_AS_I2S_FORMAT_1_PARAM,                /* 43 */
  E_AS_I2S_DEVICE_2_PARAM,                /* 44 */
  E_AS_I2S_FORMAT_2_PARAM,                /* 45 */
  E_AS_I2S_LOWEMI_PARAM,                  /* 46 */
  E_AS_LOAD_NG,                           /* 47 */
  E_AS_MCLK_DS_PARAM,                     /* 48 */
  E_AS_MIC_BIAS_SEL_PARAM,                /* 49 */
  E_AS_MIC_CHANNEL_SEL_PARAM,             /* 50 */
  E_AS_PDM_LOWEMI_PARAM,                  /* 51 */
  E_AS_XTAL_CLKMODE_PARAM,                /* 52 */
  E_AS_XTAL_SEL_PARAM,                    /* 53 */
  E_AS_BBENABLE_NG,                       /* 54 */
  /* BEEP */
  E_AS_SETBEEP_OUTDEV_ERR,                /* 55 */
  E_AS_BEEPENA_OUTDEV_ERR,                /* 56 */
  E_AS_BEEPDIS_OUTDEV_ERR,                /* 57 */
  E_AS_BEEP_FREQ_PARAM,                   /* 58 */
  E_AS_BEEP_VOL_PARAM,                    /* 59 */
  /* I2S */
  E_AS_I2S_RATE_PARAM,                    /* 60 */
  E_AS_SRC_BYPASS_ERR,                    /* 61 */
  /* Codec Volume */
  E_AS_SETVOL_OUTDEV_ERR,                 /* 62 */
  E_AS_MUTE_OUTDEV_ERR,                   /* 63 */
  E_AS_UNMUTE_OUTDEV_ERR,                 /* 64 */
  E_AS_CODEC_VOL_NULL,                    /* 65 */
  E_AS_CODEC_VOL_ID_PARAM,                /* 66 */
  E_AS_CODEC_VOL_PARAM,                   /* 67 */
  /* Audio Codec ClearStereo */
  E_AS_CS_EN_PARAM,                       /* 68 */
  E_AS_CS_SIGN_PARAM,                     /* 69 */
  E_AS_CS_VOL_PARAM,                      /* 70 */
  /* DMAC IF */
  E_AS_DMAC_ACTIVATED_ERR,                /* 71 */
  E_AS_DMAC_BUSY,                         /* 72 */
  E_AS_DMAC_CRE_TASK_ERR,                 /* 73 */
  E_AS_DMAC_DEACTIVATED_ERR,              /* 74 */
  E_AS_DMAC_DEL_TASK_ERR,                 /* 75 */
  E_AS_DMAC_ERR_START,                    /* 76 */
  E_AS_DMAC_ID_PARAM,                     /* 77 */
  E_AS_DMAC_MSG_SEND_ERR,                 /* 78 */
  E_AS_DMAC_SAMPLING_FMT_PARAM,           /* 79 */
  E_AS_DMAC_SIZE_MAX_ERR,                 /* 80 */
  E_AS_DMAC_SIZE_MIN_ERR,                 /* 81 */
  E_AS_DMAC_TRANS_ADDR_NULL,              /* 82 */
  E_AS_INITDMAC_NULL,                     /* 83 */
  E_AS_READDMAC_NULL,                     /* 84 */
  E_AS_WRITEDMAC_NULL,                    /* 85 */
  E_AS_GETREADYCMD_RESULT_NULL,           /* 86 */

  /* DEQ */
  E_AS_DEQ_UNSUPPORT,                     /* 87 */
  /* DNC */
  E_AS_DNC_UNSUPPORT,                     /* 88 */
  E_AS_DNC_SEL_PARAM,                     /* 89 */
  /* MIC */
  E_AS_MICGAIN_PARAM,                     /* 90 */
  E_AS_MICMODE_PARAM,                     /* 91 */
  E_AS_MICGAIN_NULL,                      /* 92 */
  /* Output Device */
  E_AS_OUT_DEVICE_SEL_PARAM,              /* 93 */
  /* Audio Codec data path */
  E_AS_PATH_SEL_COMBINATION_NG,           /* 94 */
  E_AS_PATH_SEL_DMACID_ERR,               /* 95 */
  E_AS_PATH_SEL_DMACID_NULL,              /* 96 */
  E_AS_PATH_SEL_DMACID_PARAM,             /* 97 */
  E_AS_PATH_SEL_FROM_PARAM,               /* 98 */
  E_AS_PATH_SEL_TO_PARAM,                 /* 99 */
  E_AS_PATH_SEL_MIC_DMA_CHANNEL_PARAM,    /* 100 */
  E_AS_PATH_SEL_NOUSE_ERR,                /* 101 */
  E_AS_PATH_SEL_NULL,                     /* 102 */
  E_AS_PATH_SEL_USED_ERR                  /* 103 */
} E_AS;

/** DMAC ID to get in AS_SetAudioDataPath function */
typedef enum
{
  /** Aci */
  AS_DMAC_SEL_AC_IN,
  /** I2S_IN */
  AS_DMAC_SEL_I2S_IN,
  /** I2S_OUT */
  AS_DMAC_SEL_I2S_OUT,
  /** I2S2_IN */
  AS_DMAC_SEL_I2S2_IN,
  /** I2S2_OUT */
  AS_DMAC_SEL_I2S2_OUT,
  AS_DMAC_ID_NONE,
  AS_DMAC_SEL_MAX_ENTRY
} asDmacSelId;

/** Select MIC mode */
typedef enum
{
  /** AcaPulco MIC */
  AS_MICMODE_ACAPULCO,
  /** HPADC MIC */
  AS_MICMODE_HPADC,
  AS_MICMODE_MAX_ENTRY
} asMicMode;

/** Select volume ID */
typedef enum
{
  /** Mixer input1 volume, -1020:-102.0dB, ... 120:12.0dB,
   * #AS_VOLUME_HOLD:keep setting,
   * AS_VOLUME_MUTE(default):Mute
   */
  AS_VOLUME_INPUT1,
  /** Mixer input2 volume, -1020:-102.0dB, ... 120:12.0dB,
   * #AS_VOLUME_HOLD:keep setting,
   * AS_VOLUME_MUTE(default):Mute
   */
  AS_VOLUME_INPUT2,
  /** Mixer output volume, -1020:-102.0dB, ... 120:12.0dB,
   * #AS_VOLUME_HOLD:keep setting,
   * AS_VOLUME_MUTE(default):Mute
   */
  AS_VOLUME_MASTER,
  AS_VOLUME_NUM
} asCodecVolSelId;

/** Select Clear Stereo mode */
typedef enum
{
  /** Off */
  AS_CS_DISABLE,
  /** On */
  AS_CS_ENABLE,
  AS_CS_MAX_ENTRY
} asCsEn;

/** Select Clear Stereo sign */
typedef enum
{
  AS_CS_SIGN_UNKNOWN,
  /** positive */
  AS_CS_SIGN_POSITIVE,
  /** negative */
  AS_CS_SIGN_NEGATIVE,
  AS_CS_SIGN_MAX_ENTRY
} asCsSignId;

/** Select data path from */
typedef enum
{
  AS_PATH_FROM_NONE,
  /** MIC CH1-CH8 output */
  AS_PATH_FROM_MIC1_8,
  /** MIC CH1,CH2 output */
  AS_PATH_FROM_MIC12,
  /** MIC CH3,CH4 output */
  AS_PATH_FROM_MIC34,
  /** MIC CH5,CH6 output */
  AS_PATH_FROM_MIC56,
  /** MIC CH7,CH8 output */
  AS_PATH_FROM_MIC78,
  /** I2S1 output */
  AS_PATH_FROM_I2S1,
  /** I2S2 output */
  AS_PATH_FROM_I2S2,
  /** DMAC output */
  AS_PATH_FROM_DMAC,
  /** Mixer output */
  AS_PATH_FROM_MIXER,
  /** DMAC1 output */
  AS_PATH_FROM_DMAC1,
  /** DMAC2 output */
  AS_PATH_FROM_DMAC2,
  AS_PATH_FROM_MAX_ENTRY
} asPathFromId;

/** Select data path to */
typedef enum
{
  AS_PATH_TO_NONE,
  /** DMAC input */
  AS_PATH_TO_DMAC,
  /** Mixer1 input */
  AS_PATH_TO_MIXER1,
  /** Mixer2 input */
  AS_PATH_TO_MIXER2,
  /** I2S1 input */
  AS_PATH_TO_I2S1,
  /** I2S2 input */
  AS_PATH_TO_I2S2,
  AS_PATH_TO_MAX_ENTRY
} asPathToId;

typedef enum
{
  AS_ACA_SER_MODE_UNKNOWN,
  AS_ACA_SER_MODE_8CH,         /* 8ch */
  AS_ACA_SER_MODE_4CH,         /* 4ch */
  AS_ACA_SER_MODE_MAX_ENTRY
} asAcaPulcoSerModeId;

typedef enum
{
  AS_ACA_SER_FS_UNKNOWN,
  AS_ACA_SER_FS_128,           /* 128fs */
  AS_ACA_SER_FS_64,            /* 64fs */
  AS_ACA_SER_FS_MAX_ENTRY
} asAcaPulcoSerFsId;

typedef enum
{
  AS_ACA_SER_SEL_FIX0 = 0,
  AS_ACA_SER_SEL_AMIC1 = 1,
  AS_ACA_SER_SEL_AMIC2 = 2,
  AS_ACA_SER_SEL_AMIC3 = 3,
  AS_ACA_SER_SEL_AMIC4 = 4,
  AS_ACA_SER_SEL_DMIC1 = 5,
  AS_ACA_SER_SEL_DMIC2 = 6,
  AS_ACA_SER_SEL_DMIC3 = 7,
  AS_ACA_SER_SEL_DMIC4 = 8,
  AS_ACA_SER_SEL_DMIC5 = 9,
  AS_ACA_SER_SEL_DMIC6 = 10,
  AS_ACA_SER_SEL_DMIC7 = 11,
  AS_ACA_SER_SEL_DMIC8 = 12,
  AS_ACA_SER_SEL_UNKNOWN = 15,
  AS_ACA_SER_SEL_MAX_ENTRY = 16
} asAcaPulcoSerSelChId;

typedef enum
{
  AS_SDES_DES_SEL_UNKNOWN,
  AS_SDES_DES_SEL_CH1,
  AS_SDES_DES_SEL_CH2,
  AS_SDES_DES_SEL_CH3,
  AS_SDES_DES_SEL_CH4,
  AS_SDES_DES_SEL_CH5,
  AS_SDES_DES_SEL_CH6,
  AS_SDES_DES_SEL_CH7,
  AS_SDES_DES_SEL_CH8,
  AS_SDES_DES_SEL_MAX_ENTRY
} asSdesDesSelOutId;

typedef enum
{
  AS_ACA_CHECK_ID,
  AS_ACA_POWER_ON_COMMON,
  AS_ACA_POWER_ON_INPUT,
  AS_ACA_POWER_ON_OUTPUT,
  AS_ACA_SET_SERDES,
  AS_ACA_SET_SMASTER,
  AS_ACA_POWER_OFF_COMMON,
  AS_ACA_POWER_OFF_INPUT,
  AS_ACA_POWER_OFF_OUTPUT,
  AS_ACA_POWER_ON_MICBIAS,
  AS_ACA_POWER_OFF_MICBIAS,
  AS_ACA_INIT_AMIC,
  AS_ACA_SET_AMIC_BOOT_DONE,
  AS_ACA_SET_OUTPUT_DEVICE,
  AS_ACA_CONTROL_TYPE_NUM
} AsAcaControlType;

#define CHECK_RANGE(val, hold, min, max) \
          (((val) == (hold)) || \
          (((min) <= ((int32_t)val)) && ((val) <= (max))))

/** DMAC interrupt notify code */
typedef enum
{
  E_AS_DMA_INT_CMPLT,  /* nomal end */
  E_AS_DMA_INT_ERR,    /* illegal end */
  E_AS_DMA_INT_ERR_BUS /* bus error */
} E_AS_DMA_INT;

/** DMAC transfer error callback function */
typedef void (* AS_DmaIntCb)(asDmacSelId dmacId, E_AS_DMA_INT code);

  
typedef enum
{
  AS_DMAC_AC_IN_SEL_MIC1L = 0,
  AS_DMAC_AC_IN_SEL_MIC1R = 1,
  AS_DMAC_AC_IN_SEL_MIC2L = 2,
  AS_DMAC_AC_IN_SEL_MIC2R = 3,
  AS_DMAC_AC_IN_SEL_MIC3L = 4,
  AS_DMAC_AC_IN_SEL_MIC3R = 5,
  AS_DMAC_AC_IN_SEL_MIC4L = 6,
  AS_DMAC_AC_IN_SEL_MIC4R = 7,
  AS_DMAC_AC_IN_SEL_UNUSE = 8
} asDmacAcInSelId;

typedef enum
{
  AS_DMAC_I2S_IN_SEL_SRC1L = 0,
  AS_DMAC_I2S_IN_SEL_SRC1R = 1,
  AS_DMAC_I2S_IN_SEL_UNUSE
} asDmacI2sInSelId;

typedef enum
{
  AS_DMAC_I2S_OUT_SEL_SD1L  = 0,
  AS_DMAC_I2S_OUT_SEL_SD1R  = 1,
  AS_DMAC_I2S_OUT_SEL_UNUSE
} asDmacI2sOutSelId;

/****************************************************************************
 * Public Types
 ***************************************************************************/

typedef struct
{
  uint32_t addr;
  uint32_t pos;
  uint32_t len;
  uint32_t init;
} acReg;

/** BaseBand Config Table */
typedef struct
{
  /** [in] Select MIC bias, 0:2.0V(default) or 1:2.8V */
  uint8_t    mic_bias_sel;
  /** [in] reserved */
  uint8_t    reserved1;
  /** [in] Select Clock mode, Use #AsClkModeId enum type. */
  uint8_t    clk_mode;
  /** [in] Select CXD5247's X'tal frequency, Use #AsXtalSelId enum type. */
  uint8_t    xtal_sel;

  /** [in] Select CXD5247 MIC input */
  uint32_t  mic_channel_sel;

  /** [in] Select drive strength for GPO_A pin, Use #asIoDsId enum type. */
  uint8_t    gpo_ds;
  /** [in] Select drive strength for DA_DATA pin, Use #asIoDsId enum type. */
  uint8_t    ad_data_ds;
  /** [in] Select drive strength for CLKOUT_DMIC pin,
   * Use #asIoDsId enum type.
   */
  uint8_t    dmic_clk_ds;
  /** [in] Select drive strength for MCLKOUT pin, Use #asIoDsId enum type. */
  uint8_t    mclk_ds;

  /** [in] Select PCM input mode/port, Use #AsI2sDeviceId enum type. */
  uint8_t    i2s_device_1;
  /** [in] Select PCM format, Use #AsI2sFormatId enum type. */
  uint8_t    i2s_format_1;
  /** [in] reserved */
  uint8_t    reserved2;
  /** [in] Select PCM data path, Use #AsI2sDataPath enum type. */
  uint8_t    i2s_data_path;

  /** [in] Select PCM input mode/port, Use #AsI2sDeviceId enum type. */
  uint8_t    i2s_device_2;
  /** [in] Select PCM format, Use #AsI2sFormatId enum type. */
  uint8_t    i2s_format_2;
  /** [in] Set wait time for MIC booting. [msec] */
  uint16_t  mic_boot_wait;

  /** [in] Select PDM drive current, Use #AsLowemi enum type. */
  uint8_t    pdm_lowemi;
  /** [in] Select I2S drive current, Use #AsLowemi enum type. */
  uint8_t    i2s_lowemi;
  /** [in] Select Audio Codec CIC input source,
   * Use #AsCicInputSelId enum type.
   */
  uint8_t    cic_input_sel;
  /** [in] reserved */
  uint8_t    reserved4;

  /** [in] Select Audio Code ALC/SPC mode, Use #asAlcSpcSelId enum type. */
  uint8_t    alc_spc_sel;
  uint8_t    reserved5;
  /** [in] Set SPC limit level,
   * 0:0dB(default),
   * -5:-0.5dB,
   * ... ,
   * -245:-24.5dB,
   * -250:-25.0dB
   */
  int16_t    spc_limit;

  /** [in] Set ALC target level,
   * 0:0dB(default),
   * -1:-1dB,
   * ... ,
   * -62:-62dB,
   * -63:-63dB
   */
  int16_t    alc_target;
  /** [in] Set ALC knee level,
   * 0:0dB,
   * -5:-0.5dB,
   * ... ,
   * -40:-4.0dB(default),
   * ... ,
   * -630:-63.0dB,
   * -635:-63.5dB
   */
  int16_t    alc_knee;

  /** [in] reserved */
  uint8_t    reserved6;
  /** [in] Select DMA data format, Use #AsDmaDataFormat enum type. */
  uint8_t    dma_data_format;
  /** [in] Select HPADC MIC bias enable/disable,
   * Use #AsHpadcMicBias enum type.
   */
  uint8_t    hpadc_mic_bias;
  /** [in] reserved */
  uint8_t    reserved7;

  /** [in] Select dead time, Use #asAcaSpDelaySelId enum type. */
  uint8_t    sp_delay;
  /** [in] Select FB loop, Use #asAcaSpLoopModeId enum type. */
  uint8_t    loop_mode;
  /** [in] Select PWM motion mode, Use #asSmstrPwmModeId enum type. */
  uint8_t    pwm_mode;
  /** [in] Select free mode of delay input,
   * Use #asAcaSpDlyFreeId enum type.
   */
  uint8_t    sp_delay_free;

  /** [in] Select split drive mode, Use #asAcaSpSplitonSelId enum type. */
  uint8_t    sp_split;
  /** [in] Select drive motion mode, Use #asAcaSpDrvSelId enum type. */
  uint8_t    sp_drive;
  uint8_t    reserved8;
  uint8_t    reserved9;
} BaseBandConfigTbl;

/** #AS_SetVolume function parameter */
typedef struct
{
  /** [in] Mixer input1 volume */
  int16_t             input1_db;
  /** [in] Mixer input2 volume */
  int16_t             input2_db;
  /** [in] Mixer output volume */
  int16_t             master_db;
} asCodecVol;

/** #AS_SetAudioDataPath function parameter */
typedef struct
{
  /** [in] Select data path from, Use #asPathFromId enum type */
  asPathFromId        pathFrom;
  /** [in] Select data path to, Use #asPathToId enum type */
  asPathToId          pathTo;
  /** [in] set dma channels of MIC */
  uint8_t             mic_dma_channel;
} asPathSelParam;

/** BaseBand Config Add Table */
typedef struct
{
  uint8_t    micNum;
  uint8_t    micDev;
  uint8_t    ser_mode;
  uint8_t    output_device_sel;

  uint8_t    sp_offon;
  uint8_t    mic_dma_channel;
} BaseBandConfigAddTbl;

typedef struct
{
  asAcaPulcoSerModeId   serMode;
  asAcaPulcoSerFsId     serFs;
  union
  {
    asAcaPulcoSerSelChId  in[AS_MIC_CHANNEL_MAX];
    asSdesDesSelOutId     out[AS_MIC_CHANNEL_MAX];
  } selCh;
} asSerDesParam;

#undef EXTERN
#ifdef __cplusplus
}
#endif

struct audio_bb_power_param_s
{
  FAR uint32_t        *rate;
  FAR asBypassModeId  *bypass_mode;
};

struct audio_bb_input_param_s
{
  asMicMode     micMode;
  FAR int32_t   *micGain;
};

struct audio_bb_clearstereo_param_s
{
  asCsEn      csEn;
  asCsSignId  csSign;
  int32_t     csVol;
};

struct audio_bb_beep_param_s
{
  uint32_t  beepFreq;
  int32_t   beepVol;
};

struct audio_bb_datapath_param_s
{
  FAR asPathSelParam  *pPathSelParam;
  asDmacSelId*        getDmacId;
  asDmacSelId         setDmacId;
};

/****************************************************************************
 * Public Data
 ***************************************************************************/

extern const acReg acRegMap[RI_REG_MAX_ENTRY];

extern const acReg bcaRegMap[BCA_REG_MAX_ENTRY];

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ***************************************************************************/

/**
 * @brief Power on BaseBand driver
 *
 * @param[in] uint32_t I2S data rate
 * @param[in] asBypassModeId I2S SRC bypass mode
 *
 * @retval E_AS return code
 */
E_AS AS_PowerOnBaseBand(uint32_t rate[AS_I2S_ID_NUM],
                        asBypassModeId bypass_mode_en[AS_I2S_ID_NUM]);

/**
 * @brief Power off BaseBand driver
 *
 * @param none
 *
 * @retval E_AS return code
 */
E_AS AS_PowerOffBaseBand(void);

/**
 * @brief Enable BaseBand driver input
 *
 * @param[in] asMicMode MIC mode
 * @param[in] int32_t MIC gain
 *
 * @retval E_AS return code
 */
E_AS AS_BaseBandEnable_input(asMicMode micMode,
                             int32_t micGain[AS_MIC_CHANNEL_MAX]);

/**
 * @brief Enable BaseBand driver output
 *
 * @param[in] asOutDeviceId Out device ID
 *
 * @retval E_AS return code
 */
E_AS AS_BaseBandEnable_output(asOutDeviceId devid);

/**
 * @brief Disable BaseBand driver input
 *
 * @param[in] asMicMode MIC mode
 *
 * @retval E_AS return code
 */
E_AS AS_BaseBandDisable_input(asMicMode micMode);

/**
 * @brief Disable BaseBand driver output
 *
 * @param none
 *
 * @retval E_AS return code
 */
E_AS AS_BaseBandDisable_output(void);

/**
 * @brief Set volume
 *
 * @param[in] asCodecVol* Codec volume parameter
 *
 * @retval E_AS return code
 */
E_AS AS_SetVolume(FAR asCodecVol *pCodecVol);

/**
 * @brief Set volume mute
 *
 * @param[in] asCodecVolSelId Volume type ID
 *
 * @retval E_AS return code
 */
E_AS AS_MuteVolume(asCodecVolSelId volId);

/**
 * @brief Set volume unmute
 *
 * @param[in] asCodecVolSelId Volume type ID
 *
 * @retval E_AS return code
 */
E_AS AS_UnMuteVolume(asCodecVolSelId volId);

/**
 * @brief Init clear stereo
 *
 * @param[in] asCsEn Clear stereo mode
 * @param[in] asCsSignId Clear stereo sign
 * @param[in] int32_t Clear stereo volume, -825:-82.5dB, ... -195:-19.5dB,
 * step:5
 *
 * @retval E_AS return code
 */
E_AS AS_InitClearStereo(asCsEn csEn, asCsSignId csSign, int32_t csVol);

/**
 * @brief Set DNC parameter
 *
 * @param[in] asDncMode DNC mode
 *
 * @retval E_AS return code
 */
E_AS AS_SetDncParam(asDncMode dncMode);

/**
 * @brief Set beep parameter
 *
 * @param[in] uint32_t beep frequency, 94:94Hz, ... 4085:4085Hz, step:1
 * @param[in] uint32_t beep volume, -90:-90dB, ... 0:0dB, step:3, default:-12,
 * #AS_BEEP_VOL_HOLD:keep setting
 *
 * @retval E_AS return code
 */
E_AS AS_SetBeepParam(uint32_t beepFreq, int32_t beepVol);

/**
 * @brief Disable beep
 *
 * @param none
 *
 * @retval E_AS return code
 */
E_AS AS_BeepDisable(void);

/**
 * @brief Enable beep
 *
 * @param none
 *
 * @retval E_AS return code
 */
E_AS AS_BeepEnable(void);

/**
 * @brief Set audio data path
 *
 * @param[in] asPathSelParam* Data path parameter
 * @param[out] asDmacSelId* DMAC of the setting data path
 * @param[in] asDmacSelId To set the data path in the same DMAC ID
 *
 * @retval E_AS return code
 */
E_AS AS_SetAudioDataPath(FAR asPathSelParam *pPathSelParam,
                         FAR asDmacSelId *getDmacId,
                         asDmacSelId setDmacId);

/**
 * @brief Clear audio data path
 *
 * @param[in] asPathSelParam* Data path parameter
 *
 * @retval E_AS return code
 */
E_AS AS_ClearAudioDataPath(FAR asPathSelParam *pPathSelParam);

/**
 * @brief Clear all audio data path
 *
 * @param noner
 *
 * @retval E_AS return code
 */
E_AS AS_ClearAudioDataPathAll(void);

/**
 * @brief Set MIC gain
 *
 * @param[in] int32_t MIC gain
 *
 * @retval E_AS return code
 */
E_AS AS_SetMicGain(int32_t micGain[AS_MIC_CHANNEL_MAX]);

E_AS setMicChSel(uint8_t micNum, asSampFmt format);
E_AS chkMicGainParam(int16_t micGain[AS_MIC_CHANNEL_MAX]);
E_AS chkI2sParam(uint32_t rate[AS_I2S_ID_NUM],
                 asBypassModeId bypass_mode_en[AS_I2S_ID_NUM]);
void outputDeviceUpdate(void);
bool chkPowerOnBaseBand(void);
bool chkEnableBaseBandInput(void);
bool chkEnableBaseBandOutput(void);
E_AS AS_SetOutputSelect(asOutDeviceId devid);
E_AS AS_SetI2sParam(uint32_t rate[AS_I2S_ID_NUM],
                    asBypassModeId bypass_mode_en[AS_I2S_ID_NUM]);
AsClkModeId GetClkMode(void);
AsDmaDataFormat GetDmaDataFormat(void);

uint32_t write_bca_reg(BCA_REG_ID regId, uint32_t data);
uint32_t write_bca_reg_mask(BCA_REG_ID regId);
uint32_t read_bca_reg(BCA_REG_ID regId);
uint32_t write32_bca_reg(uint32_t offset, uint32_t data);
uint32_t read32_bca_reg(uint32_t offset);

E_AS AS_SetRenderingClk(uint8_t clk_mode);

/* Debug function */

void init_bca_reg(void);
void read_bca_reg_all(void);

/* bca_drv_sub_func */

E_AS getDmacCmdStatus(asDmacSelId dmacId, FAR uint32_t *pResult);
E_AS getDmacErrorStatus(asDmacSelId dmacId, FAR uint32_t *pResult);
E_AS setDmacAddr(asDmacSelId dmacId, uint32_t addr);
E_AS setDmacSample(asDmacSelId dmacId, uint32_t sample);
extern E_AS setDmacTrg(asDmacSelId dmacId, bool nointr);
E_AS setDmacDoneIntMask(asDmacSelId dmacId, bool mask);
E_AS getDmacDoneIntStatus(asDmacSelId dmacId, FAR uint32_t *pResult);
E_AS clearDmacDoneIntStatus(asDmacSelId dmacId);
E_AS setDmacErrIntMask(asDmacSelId dmacId, bool mask);
E_AS getDmacErrIntStatus(asDmacSelId dmacId, FAR uint32_t *pResult);
E_AS clearDmacErrIntStatus(asDmacSelId dmacId);

E_AS setDmacSmpIntMask(asDmacSelId dmacId, bool mask);
E_AS getDmacSmpIntStatus(asDmacSelId dmacId, FAR uint32_t *pResult);
E_AS clearDmacSmpIntStatus(asDmacSelId dmacId);
E_AS getDmacChannel(asDmacSelId dmacId, FAR uint32_t *pResult);
E_AS setDmacTrgWithChSel(asDmacSelId dmacId, bool nointr, uint32_t chsel);
E_AS getDmacMonbufStatus(asDmacSelId dmacId, FAR uint32_t *pResult);
E_AS getDmacMonStartStatus(asDmacSelId dmacId, FAR uint32_t *pResult);
E_AS setDmacStop(asDmacSelId dmacId);

E_AS setDmacCmbIntMask(asDmacSelId dmacId, bool mask);
E_AS clearDmacCmbIntStatus(asDmacSelId dmacId);
E_AS setDmacBusIntMask(asDmacSelId dmacId, bool mask);

uint32_t read_as_reg(uint32_t reg);
uint32_t write_as_reg(uint32_t reg, uint32_t data);

asDmacMixerOutId GetDmacPathToMixerId(asDmacSelId dmacId);
void checkErrCode(E_AS rtCode, E_AS okCode);

E_AS muteVolumeFade(asCodecVolSelId volId, bool waitFlg);
E_AS unMuteVolumeFade(asCodecVolSelId volId, bool waitFlg);
void asAc_AhbmasterEnable(void);
E_AS asBca_StopDmac(asDmacSelId dmacId);
E_AS AS_AcaControl(uint8_t type, uint32_t param);

extern BaseBandConfigTbl *bb_config_tblp;
extern BaseBandConfigAddTbl bb_config_add_tbl;

/* ac_drv */

E_AS asAc_SetDigSft(uint32_t digsft);
E_AS asAc_SetDsrRate(uint32_t rate);

int cxd56_audio_bb_register(FAR const char *devpath);
int cxd56_audio_bb_unregister(FAR const char *devpath);

/* audio_dma */

E_AS asDmac_InitDriver(asDmacSelId dmac_id, asSampFmt format, uint8_t *p_ch_num);
E_AS asDmac_RegsitIntCb(asDmacSelId, AS_DmaIntCb);
void asDmac_MonitorStatus(asDmacSelId);
void asDmac_EnableInt(void);
void asDmac_DisableInt(void);

#ifdef __cplusplus
} /* end of extern "C" */
#endif /* __cplusplus */

#endif /* __BSP_INCLUDE_ARCH_CHIP_CXD56_AUDIO_H */
/**
 * @}
 */
/**
 * @}
 */
