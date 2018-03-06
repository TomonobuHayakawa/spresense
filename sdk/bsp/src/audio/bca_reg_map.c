/****************************************************************************
 * arch/arm/src/cxd56xx/audio/drivers/baseband/src/bca_reg_map.c
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

/*
 * This header file is generated as follows.
 * % perl genRegHeader.pl BUS_CTRL_AHB_regmap_20140728.xls
 * Output file: bca_reg_map.h, bca_reg_map.c
 * Number of detected registers: 95
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/chip/cxd56_audio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

const acReg bcaRegMap[BCA_REG_MAX_ENTRY] =
{
  {0x1000,  2, 30, 0x00000000},  /* Mic_In_start_adr (0x00000000)     */
  {0x1004,  0, 32, 0x00000000},  /* Mic_In_sample_no (0x00000000)     */
  {0x1008,  0,  2, 0x00000000},  /* Mic_In_rtd_trg (0x00)             */
  {0x1008,  2,  1, 0x00000000},  /* Mic_In_nointr (0x00)              */
  {0x100C,  0,  1, 0x00000000},  /* Mic_In_bitwt (0x00)               */
  {0x1010,  0,  4, 0x00000000},  /* Mic_In_ch8_sel (0x00)             */
  {0x1010,  4,  4, 0x00000000},  /* Mic_In_ch7_sel (0x00)             */
  {0x1010,  8,  4, 0x00000000},  /* Mic_In_ch6_sel (0x00)             */
  {0x1010, 12,  4, 0x00000000},  /* Mic_In_ch5_sel (0x00)             */
  {0x1010, 16,  4, 0x00000000},  /* Mic_In_ch4_sel (0x00)             */
  {0x1010, 20,  4, 0x00000000},  /* Mic_In_ch3_sel (0x00)             */
  {0x1010, 24,  4, 0x00000000},  /* Mic_In_ch2_sel (0x00)             */
  {0x1010, 28,  4, 0x00000000},  /* Mic_In_ch1_sel (0x00)             */
  {0x1014,  0,  1, 0x00000000},  /* Mic_In_start (0x00)               */
  {0x1014,  8,  8, 0x00000000},  /* Mic_In_error_setting (0x00)       */
  {0x1014, 16,  4, 0x00000000},  /* Mic_In_monbuf (0x00)              */

  {0x1080,  2, 30, 0x00000000},  /* I2s1_In_start_adr (0x00000000)    */
  {0x1084,  0, 32, 0x00000000},  /* I2s1_In_sample_no (0x00000000)    */
  {0x1088,  0,  2, 0x00000000},  /* I2s1_In_rtd_trg (0x00)            */
  {0x1088,  2,  1, 0x00000000},  /* I2s1_In_nointr (0x00)             */
  {0x108C,  0,  1, 0x00000000},  /* I2s1_In_bitwt (0x00)              */
  {0x1090,  0,  2, 0x00000000},  /* I2s1_In_ch2_sel (0x00)            */
  {0x1090,  4,  2, 0x00000000},  /* I2s1_In_ch1_sel (0x00)            */
  {0x1094,  0,  1, 0x00000000},  /* I2s1_In_Mon_start (0x00)          */
  {0x1094,  8,  8, 0x00000000},  /* I2s1_In_Mon_error_setting (0x00)  */
  {0x1094, 16,  4, 0x00000000},  /* I2s1_In_Mon_monbuf (0x00)         */
  {0x10a0,  2, 30, 0x00000000},  /* I2s2_In_start_adr (0x00000000)    */
  {0x10a4,  0, 32, 0x00000000},  /* I2s2_In_sample_no (0x00000000)    */
  {0x10a8,  0,  2, 0x00000000},  /* I2s2_In_rtd_trg (0x00)            */
  {0x10a8,  2,  1, 0x00000000},  /* I2s2_In_nointr (0x00)             */
  {0x10ac,  0,  1, 0x00000000},  /* I2s2_In_bitwt (0x00)              */
  {0x10b0,  0,  2, 0x00000000},  /* I2s2_In_ch2_sel (0x00)            */
  {0x10b0,  4,  2, 0x00000000},  /* I2s2_In_ch1_sel (0x00)            */
  {0x10b4,  0,  1, 0x00000000},  /* I2s2_In_Mon_start (0x00)          */
  {0x10b4,  8,  8, 0x00000000},  /* I2s2_In_Mon_error_setting (0x00)  */
  {0x10b4, 16,  4, 0x00000000},  /* I2s2_In_Mon_monbuf (0x00)         */

  {0x10c0,  2, 30, 0x00000000},  /* I2s1_Out_start_adr (0x00000000)   */
  {0x10c4,  0, 32, 0x00000000},  /* I2s1_Out_sample_no (0x00000000)   */
  {0x10c8,  0,  2, 0x00000000},  /* I2s1_Out_rtd_trg (0x00)           */
  {0x10c8,  2,  1, 0x00000000},  /* I2s1_Out_nointr (0x00)            */
  {0x10cc,  0,  1, 0x00000000},  /* I2s1_Out_bitwt (0x00)             */
  {0x10d0,  0,  2, 0x00000000},  /* I2s1_Out_sd1_r_sel (0x00)         */
  {0x10d0,  4,  2, 0x00000000},  /* I2s1_Out_sd1_l_sel (0x00)         */
  {0x10d4,  0,  1, 0x00000000},  /* I2s1_Out_Mon_start (0x00)         */
  {0x10d4,  8,  8, 0x00000000},  /* I2s1_Out_Mon_error_setting (0x00) */
  {0x10d4, 16,  4, 0x00000000},  /* I2s1_Out_Mon_monbuf (0x00)        */
  {0x10e0,  2, 30, 0x00000000},  /* I2s2_Out_start_adr (0x00000000)   */
  {0x10e4,  0, 32, 0x00000000},  /* I2s2_Out_sample_no (0x00000000)   */
  {0x10e8,  0,  2, 0x00000000},  /* I2s2_Out_rtd_trg (0x00)           */
  {0x10e8,  2,  1, 0x00000000},  /* I2s2_Out_nointr (0x00)            */
  {0x10ec,  0,  1, 0x00000000},  /* I2s2_Out_bitwt (0x00)             */
  {0x10f0,  0,  2, 0x00000000},  /* I2s2_Out_sd1_r_sel (0x00)         */
  {0x10f0,  4,  2, 0x00000000},  /* I2s2_Out_sd1_l_sel (0x00)         */
  {0x10f4,  0,  1, 0x00000000},  /* I2s2_Out_Mon_start (0x00)         */
  {0x10f4,  8,  8, 0x00000000},  /* I2s2_Out_Mon_error_setting (0x00) */
  {0x10f4, 16,  4, 0x00000000},  /* I2s2_Out_Mon_monbuf (0x00)        */

  {0x1110,  0,  1, 0x00000000},  /* I2s_ensel (0x00)                  */
  {0x1120,  0, 32, 0x00000000},  /* Mici_prdat_u (0x00000000)         */
  {0x1130,  0, 32, 0x00000000},  /* I2s1_In_prdat_u (0x00000000)      */
  {0x1134,  0, 32, 0x00000000},  /* I2s2_In_prdat_u (0x00000000)      */
  {0x1138,  0, 32, 0x00000000},  /* I2s1_Out_prdat_d (0x00000000)     */
  {0x113c,  0, 32, 0x00000000},  /* I2s2_Out_prdat_d (0x00000000)     */

  {0x1140,  0,  1, 0x00000000},  /* Mic_Int_Ctrl_done_mic (0x00)      */
  {0x1140,  1,  1, 0x00000000},  /* Mic_Int_Ctrl_err_mic (0x00)       */
  {0x1140,  2,  1, 0x00000000},  /* Mic_Int_Ctrl_smp_mic (0x00)       */
  {0x1140,  3,  1, 0x00000000},  /* Mic_Int_Ctrl_cmb_mic (0x00)       */
  {0x1144,  0,  1, 0x00000000},  /* I2s1_Int_Ctrl_done_i2so (0x00)    */
  {0x1144,  1,  1, 0x00000000},  /* I2s1_Int_Ctrl_err_i2so (0x00)     */
  {0x1144,  2,  1, 0x00000000},  /* I2s1_Int_Ctrl_done_i2si (0x00)    */
  {0x1144,  3,  1, 0x00000000},  /* I2s1_Int_Ctrl_err_i2si (0x00)     */
  {0x1144,  4,  1, 0x00000000},  /* I2s1_Int_Ctrl_smp_i2s (0x00)      */
  {0x1144,  5,  1, 0x00000000},  /* I2s1_Int_Ctrl_cmb_i2s (0x00)      */
  {0x1148,  0,  1, 0x00000000},  /* I2s2_Int_Ctrl_done_i2so (0x00)    */
  {0x1148,  1,  1, 0x00000000},  /* I2s2_Int_Ctrl_err_i2so (0x00)     */
  {0x1148,  2,  1, 0x00000000},  /* I2s2_Int_Ctrl_done_i2si (0x00)    */
  {0x1148,  3,  1, 0x00000000},  /* I2s2_Int_Ctrl_err_i2si (0x00)     */
  {0x1148,  4,  1, 0x00000000},  /* I2s2_Int_Ctrl_smp_i2s (0x00)      */
  {0x1148,  5,  1, 0x00000000},  /* I2s2_Int_Ctrl_cmb_i2s (0x00)      */

  {0x114c,  0,  1, 0x00000001},  /* Mic_Int_Mask_done_mic (0x00)      */
  {0x114c,  1,  1, 0x00000001},  /* Mic_Int_Mask_err_mic (0x00)       */
  {0x114c,  2,  1, 0x00000001},  /* Mic_Int_Mask_smp_mic (0x00)       */
  {0x114c,  3,  1, 0x00000001},  /* Mic_Int_Mask_cmb_mic (0x00)       */
  {0x114c, 30,  1, 0x00000000},  /* Mic_Int_Mask_nostpmsk (0x00)      */
  {0x114c, 31,  1, 0x00000000},  /* Mic_Int_Mask_srst_mic (0x00)      */
  {0x1150,  0,  1, 0x00000001},  /* I2s1_Int_Mask_done_i2so (0x00)    */
  {0x1150,  1,  1, 0x00000001},  /* I2s1_Int_Mask_err_i2so (0x00)     */
  {0x1150,  2,  1, 0x00000001},  /* I2s1_Int_Mask_done_i2si (0x00)    */
  {0x1150,  3,  1, 0x00000001},  /* I2s1_Int_Mask_err_i2si (0x00)     */
  {0x1150,  4,  1, 0x00000001},  /* I2s1_Int_Mask_smp_i2s (0x00)      */
  {0x1150,  5,  1, 0x00000001},  /* I2s1_Int_Mask_cmb_i2s (0x00)      */
  {0x1150, 30,  1, 0x00000000},  /* I2s1_Int_Mask_nostpmsk_i2s (0x00) */
  {0x1150, 31,  1, 0x00000000},  /* I2s1_Int_Mask_srst_i2s (0x00)     */
  {0x1154,  0,  1, 0x00000001},  /* I2s2_Int_Mask_done_i2so (0x00)    */
  {0x1154,  1,  1, 0x00000001},  /* I2s2_Int_Mask_err_i2so (0x00)     */
  {0x1154,  2,  1, 0x00000001},  /* I2s2_Int_Mask_done_i2si (0x00)    */
  {0x1154,  3,  1, 0x00000001},  /* I2s2_Int_Mask_err_i2si (0x00)     */
  {0x1154,  4,  1, 0x00000001},  /* I2s2_Int_Mask_smp_i2s (0x00)      */
  {0x1154,  5,  1, 0x00000001},  /* I2s2_Int_Mask_cmb_i2s (0x00)      */
  {0x1154, 30,  1, 0x00000000},  /* I2s2_Int_Mask_nostpmsk_i2s (0x00) */
  {0x1154, 31,  1, 0x00000000},  /* I2s2_Int_Mask_srst_i2s (0x00)     */

  {0x1158,  0,  1, 0x00000001},  /* Int_m_hresp_err (0x01)            */
  {0x1158,  8,  1, 0x00000001},  /* Int_m_I2s1_bak_err1 (0x01)        */
  {0x1158,  9,  1, 0x00000001},  /* Int_m_I2s1_bck_err2 (0x01)        */
  {0x1158, 10,  1, 0x00000001},  /* Int_m_anc_faint (0x01)            */
  {0x1158, 17,  1, 0x00000001},  /* Int_m_ovf_smasl (0x01)            */
  {0x1158, 18,  1, 0x00000001},  /* Int_m_ovf_smasr (0x01)            */
  {0x1158, 21,  1, 0x00000001},  /* Int_m_ovf_dnc1l (0x01)            */
  {0x1158, 22,  1, 0x00000001},  /* Int_m_ovf_dnc1r (0x01)            */
  {0x1158, 23,  1, 0x00000001},  /* Int_m_ovf_dnc2l (0x01)            */
  {0x1158, 24,  1, 0x00000001},  /* Int_m_ovf_dnc2r (0x01)            */
  {0x115c,  0,  1, 0x00000000},  /* Int_clr_hresp_err (0x00)          */
  {0x115c,  8,  1, 0x00000000},  /* Int_clr_I2s1_bck_err1 (0x00)      */
  {0x115c,  9,  1, 0x00000000},  /* Int_clr_I2s1_bck_err2 (0x00)      */
  {0x115c, 10,  1, 0x00000000},  /* Int_clr_anc_faint (0x00)          */
  {0x115c, 17,  1, 0x00000000},  /* Int_clr_ovf_smasl (0x00)          */
  {0x115c, 18,  1, 0x00000000},  /* Int_clr_ovf_smasr (0x00)          */
  {0x115c, 21,  1, 0x00000000},  /* Int_clr_ovf_dnc1l (0x00)          */
  {0x115c, 22,  1, 0x00000000},  /* Int_clr_ovf_dnc1r (0x00)          */
  {0x115c, 23,  1, 0x00000000},  /* Int_clr_ovf_dnc2l (0x00)          */
  {0x115c, 24,  1, 0x00000000},  /* Int_clr_ovf_dnc2r (0x00)          */
  {0x1160,  0,  1, 0x00000000},  /* Int_hresp_err (0x00)              */
  {0x1160,  8,  1, 0x00000000},  /* Int_i2s_bck_err1 (0x00)           */
  {0x1160,  9,  1, 0x00000000},  /* Int_i2s_bck_err2 (0x00)           */
  {0x1160, 10,  1, 0x00000000},  /* Int_anc_faint (0x00)              */
  {0x1160, 17,  1, 0x00000000},  /* Int_ovf_smasl (0x00)              */
  {0x1160, 18,  1, 0x00000000},  /* Int_ovf_smasr (0x00)              */
  {0x1160, 21,  1, 0x00000000},  /* Int_ovf_dnc1l (0x00)              */
  {0x1160, 22,  1, 0x00000000},  /* Int_ovf_dnc1r (0x00)              */
  {0x1160, 23,  1, 0x00000000},  /* Int_ovf_dnc2l (0x00)              */
  {0x1160, 24,  1, 0x00000000},  /* Int_ovf_dnc2r (0x00)              */

  {0x1180,  8, 24, 0x00000000},  /* Dbg_Mic_ch1_data (0x00)           */
  {0x1184,  8, 24, 0x00000000},  /* Dbg_Mic_ch2_data (0x00)           */
  {0x1188,  8, 24, 0x00000000},  /* Dbg_Mic_ch3_data (0x00)           */
  {0x118c,  8, 24, 0x00000000},  /* Dbg_Mic_ch4_data (0x00)           */
  {0x1190,  8, 24, 0x00000000},  /* Dbg_Mic_ch5_data (0x00)           */
  {0x1194,  8, 24, 0x00000000},  /* Dbg_Mic_ch6_data (0x00)           */
  {0x1198,  8, 24, 0x00000000},  /* Dbg_Mic_ch7_data (0x00)           */
  {0x119c,  8, 24, 0x00000000},  /* Dbg_Mic_ch8_data (0x00)           */
  {0x11a0,  8, 24, 0x00000000},  /* Dbg_I2s1_u_ch1_data (0x00)        */
  {0x11a4,  8, 24, 0x00000000},  /* Dbg_I2s1_u_ch2_data (0x00)        */
  {0x11a8,  8, 24, 0x00000000},  /* Dbg_I2s1_d_ch1_data (0x00)        */
  {0x11ac,  8, 24, 0x00000000},  /* Dbg_I2s1_d_ch2_data (0x00)        */
  {0x11b0,  8, 24, 0x00000000},  /* Dbg_I2s2_u_ch1_data (0x00)        */
  {0x11b4,  8, 24, 0x00000000},  /* Dbg_I2s2_u_ch2_data (0x00)        */
  {0x11b8,  8, 24, 0x00000000},  /* Dbg_I2s2_d_ch1_data (0x00)        */
  {0x11bc,  8, 24, 0x00000000},  /* Dbg_I2s2_d_ch2_data (0x00)        */
  {0x11c0,  0,  1, 0x00000000},  /* Dbg_Ctrl_mic_dbg_en (0x00)        */
  {0x11c0,  1,  1, 0x00000000},  /* Dbg_Ctrl_I2s1_dbg_u_en (0x00)     */
  {0x11c0,  2,  1, 0x00000000},  /* Dbg_Ctrl_I2s1_dbg_d_en (0x00)     */
  {0x11c0,  3,  1, 0x00000000},  /* Dbg_Ctrl_I2s2_dbg_u_en (0x00)     */
  {0x11c0,  4,  1, 0x00000000},  /* Dbg_Ctrl_I2s2_dbg_d_en (0x00)     */

  {0x11f0,  0,  1, 0x00000000},  /* Clk_En_ahbmstr_mic_en (0x00)      */
  {0x11f0,  1,  1, 0x00000000},  /* Clk_En_ahbmstr_I2s1_en (0x00)     */
  {0x11f0,  2,  1, 0x00000000},  /* Clk_En_ahbmstr_I2s2_en (0x00)     */
  {0x11fc,  0,  8, 0x00000064},  /* Mclk_Mon_thresh (0x64)            */

  {0x1730,  0, 32, 0x00000000},  /* AHB MASTER MIC MASK (0x00)        */
  {0x1F30,  0, 32, 0x00000000},  /* AHB MASTER I2S1 MASK (0x00)       */
  {0x2730,  0, 32, 0x00000000}   /* AHB MASTER I2S2 MASK (0x00)       */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

