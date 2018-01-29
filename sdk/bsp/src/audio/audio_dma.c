/****************************************************************************
 * bsp/src/audio/audio_dma.cpp
 *
 *   Copyright (C) 2018 Sony Corporation
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/chip/cxd56_audio.h>
#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static AS_DmaIntCb s_DmaIntCb[AS_DMAC_SEL_MAX_ENTRY];

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CHECK_AC_INT
static void ac_int_callback(uint32_t ac_intr)
{
  if (ac_intr & (1<<bcaRegMap[BCA_Int_hresp_err].pos))
    {
      _info("HRESP_ERR(%x)\n", ac_intr);
    }

  if (ac_intr & (1<<bcaRegMap[BCA_Int_i2s_bck_err1].pos))
    {
      _info("I2S_BCK_ERR1(%x)\n", ac_intr);
    }

  if (ac_intr & (1<<bcaRegMap[BCA_Int_i2s_bck_err2].pos))
    {
      _info("I2S_BCK_ERR2(%x)\n", ac_intr);
    }

  if (ac_intr & (1<<bcaRegMap[BCA_Int_anc_faint].pos))
    {
      _info("ANC_FAINT(%x)\n", ac_intr);
    }

  if (ac_intr & (1<<bcaRegMap[BCA_Int_ovf_smasl].pos))
    {
      _info("OVF_SMASL(%x)\n", ac_intr);
    }

  if (ac_intr & (1<<bcaRegMap[BCA_Int_ovf_smasr].pos))
    {
      _info("OVF_SMASR(%x)\n", ac_intr);
    }

  if (ac_intr & (1<<bcaRegMap[BCA_Int_ovf_dnc1l].pos))
    {
      _info("OVF_DNC1L(%x)\n", ac_intr);
    }

  if (ac_intr & (1<<bcaRegMap[BCA_Int_ovf_dnc1r].pos))
    {
      _info("OVF_DNC1R(%x)\n", ac_intr);
    }

  if (ac_intr & (1<<bcaRegMap[BCA_Int_ovf_dnc2l].pos))
    {
      _info("OVF_DNC2L(%x)\n", ac_intr);
    }

  if (ac_intr & (1<<bcaRegMap[BCA_Int_ovf_dnc2r].pos))
    {
      _info("OVF_DNC2R(%x)\n", ac_intr);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

E_AS asDmac_InitDriver(asDmacSelId dmac_id,
                       asSampFmt format,
                       uint8_t *p_ch_num)
{
  E_AS rtCode = E_AS_OK;
  uint8_t  ch_num;

  BCA_REG_ID micChSel[AS_MIC_CHANNEL_MAX] =
  {
    BCA_Mic_In_ch1_sel,
    BCA_Mic_In_ch2_sel,
    BCA_Mic_In_ch3_sel,
    BCA_Mic_In_ch4_sel,
    BCA_Mic_In_ch5_sel,
    BCA_Mic_In_ch6_sel,
    BCA_Mic_In_ch7_sel,
    BCA_Mic_In_ch8_sel
  };

  asDmacAcInSelId micSelId[AS_MIC_CHANNEL_MAX] =
  {
    AS_DMAC_AC_IN_SEL_MIC1L,
    AS_DMAC_AC_IN_SEL_MIC1R,
    AS_DMAC_AC_IN_SEL_MIC2L,
    AS_DMAC_AC_IN_SEL_MIC2R,
    AS_DMAC_AC_IN_SEL_MIC3L,
    AS_DMAC_AC_IN_SEL_MIC3R,
    AS_DMAC_AC_IN_SEL_MIC4L,
    AS_DMAC_AC_IN_SEL_MIC4R
  };

  switch (bb_config_tblp->clk_mode)
    {
      case AS_CLK_MODE_NORMAL:
        write_bca_reg(BCA_I2s_ensel, 0);
        break;

      case AS_CLK_MODE_HIRES:
        write_bca_reg(BCA_I2s_ensel, 1);
        break;

      default:
        return E_AS_CLK_MODE_PARAM;
    }

  switch (dmac_id)
    {
      case AS_DMAC_SEL_AC_IN:
        
        ch_num = bb_config_add_tbl.mic_dma_channel;
        if (ch_num > AS_MIC_CHANNEL_MAX)
          {
            return E_AS_MIC_CHANNEL_SEL_PARAM;
          }

        rtCode = setMicChSel(ch_num, format);
        if (rtCode != E_AS_OK)
          {
            return rtCode;
          }

        switch (format)
          {
            case AS_SAMPLING_FMT_24:
              write_bca_reg(BCA_Mic_In_bitwt, 0);

              for (uint8_t ch = 0; ch < AS_MIC_CHANNEL_MAX; ch++)
                {
                  if (ch_num > ch)
                    {
                      write_bca_reg(micChSel[ch],
                                    micSelId[ch]);
                    }
                  else
                    {
                      write_bca_reg(micChSel[ch],
                                    AS_DMAC_AC_IN_SEL_UNUSE);
                    }
                }
              break;

            case AS_SAMPLING_FMT_16:
              write_bca_reg(BCA_Mic_In_bitwt, 1);

              for (uint8_t ch = 0; ch < (AS_MIC_CHANNEL_MAX / 2); ch++)
                {
                  if(ch_num > (ch * 2))
                    {
                      write_bca_reg(micChSel[ch],
                                    micSelId[ch]);
                    }
                  else
                    {
                      write_bca_reg(micChSel[ch],
                                    AS_DMAC_AC_IN_SEL_UNUSE);
                    }
                }

              for (uint8_t ch = (AS_MIC_CHANNEL_MAX / 2);
                   ch < AS_MIC_CHANNEL_MAX;
                   ch++)
                {
                  write_bca_reg(micChSel[ch],
                                AS_DMAC_AC_IN_SEL_UNUSE);
                }
              break;

            default:
              return E_AS_DMAC_SAMPLING_FMT_PARAM;
          }

        write_bca_reg(BCA_Clk_En_ahbmstr_mic_en, 1);
        write_bca_reg(BCA_Mic_In_start_adr, 0x00000000);
        write_bca_reg(BCA_Mic_In_sample_no, 0);
        break;

      case AS_DMAC_SEL_I2S_IN:
        /* BUS IF (I2s_In) */
        ch_num = 2;

        switch (format)
          {
            case AS_SAMPLING_FMT_24:
                write_bca_reg(BCA_I2s1_In_bitwt, 0);
                write_bca_reg(BCA_I2s1_In_ch2_sel,
                              AS_DMAC_I2S_IN_SEL_SRC1R);
                write_bca_reg(BCA_I2s1_In_ch1_sel,
                              AS_DMAC_I2S_IN_SEL_SRC1L);
                break;

            case AS_SAMPLING_FMT_16:
                write_bca_reg(BCA_I2s1_In_bitwt, 1);
                write_bca_reg(BCA_I2s1_In_ch2_sel,
                              AS_DMAC_I2S_IN_SEL_UNUSE);
                write_bca_reg(BCA_I2s1_In_ch1_sel,
                              AS_DMAC_I2S_IN_SEL_SRC1L);
                break;

            default:
                return E_AS_DMAC_SAMPLING_FMT_PARAM;
          }
        write_bca_reg(BCA_Clk_En_ahbmstr_I2s1_en, 1);
        write_bca_reg(BCA_I2s1_In_start_adr, 0x00000000);
        write_bca_reg(BCA_I2s1_In_sample_no, 0);
        break;

      case AS_DMAC_SEL_I2S_OUT:
        /* BUS IF (I2s_Out) */

        ch_num = 2;

        write_bca_reg(BCA_I2s1_Out_sd1_l_sel, AS_DMAC_I2S_OUT_SEL_SD1L);
        write_bca_reg(BCA_I2s1_Out_sd1_r_sel, AS_DMAC_I2S_OUT_SEL_SD1R);

        switch (format)
          {
            case AS_SAMPLING_FMT_24:
                write_bca_reg(BCA_I2s1_Out_bitwt, 0);
                break;

            case AS_SAMPLING_FMT_16:
                write_bca_reg(BCA_I2s1_Out_bitwt, 1);
                break;

            default:
                return E_AS_DMAC_SAMPLING_FMT_PARAM;
          }

        write_bca_reg(BCA_Clk_En_ahbmstr_I2s1_en, 1);
        write_bca_reg(BCA_I2s1_Out_start_adr, 0x00000000);
        write_bca_reg(BCA_I2s1_Out_sample_no, 0);
        break;

      case AS_DMAC_SEL_I2S2_IN:
        /* BUS IF (I2s2_In) */

        ch_num = 2;

        switch (format)
          {
            case AS_SAMPLING_FMT_24:
                write_bca_reg(BCA_I2s2_In_bitwt, 0);
                write_bca_reg(BCA_I2s2_In_ch2_sel,
                              AS_DMAC_I2S_IN_SEL_SRC1R);
                write_bca_reg(BCA_I2s2_In_ch1_sel,
                              AS_DMAC_I2S_IN_SEL_SRC1L);
                break;

            case AS_SAMPLING_FMT_16:
                write_bca_reg(BCA_I2s2_In_bitwt, 1);
                write_bca_reg(BCA_I2s2_In_ch2_sel,
                              AS_DMAC_I2S_IN_SEL_UNUSE);
                write_bca_reg(BCA_I2s2_In_ch1_sel,
                              AS_DMAC_I2S_IN_SEL_SRC1L);
                break;

            default:
                return E_AS_DMAC_SAMPLING_FMT_PARAM;
          }

        write_bca_reg(BCA_Clk_En_ahbmstr_I2s2_en, 1);
        write_bca_reg(BCA_I2s2_In_start_adr, 0x00000000);
        write_bca_reg(BCA_I2s2_In_sample_no, 0);
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        /* BUS IF (I2s2_Out) */

        ch_num = 2;

        write_bca_reg(BCA_I2s2_Out_sd1_l_sel, AS_DMAC_I2S_OUT_SEL_SD1L);
        write_bca_reg(BCA_I2s2_Out_sd1_r_sel, AS_DMAC_I2S_OUT_SEL_SD1R);

        switch (format)
          {
            case AS_SAMPLING_FMT_24:
              write_bca_reg(BCA_I2s2_Out_bitwt, 0);
              break;

            case AS_SAMPLING_FMT_16:
              write_bca_reg(BCA_I2s2_Out_bitwt, 1);
              break;

            default:
              return E_AS_DMAC_SAMPLING_FMT_PARAM;
          }

        write_bca_reg(BCA_Clk_En_ahbmstr_I2s2_en, 1);
        write_bca_reg(BCA_I2s2_Out_start_adr, 0x00000000);
        write_bca_reg(BCA_I2s2_Out_sample_no, 0);
        break;

      default:
        return E_AS_DMAC_ID_PARAM;
    }

  *p_ch_num = ch_num;

  return rtCode;
}

/*--------------------------------------------------------------------*/
void asDmac_MonitorStatus(asDmacSelId dmac_id)
{
  switch (dmac_id)
    {
      case AS_DMAC_SEL_AC_IN:
        _info("dmac_id(%d): start(%d),error(%d),monbuf(%d)\n", dmac_id,
            read_bca_reg(BCA_Mic_In_start),
            read_bca_reg(BCA_Mic_In_error_setting),
            read_bca_reg(BCA_Mic_In_monbuf));
        break;

      case AS_DMAC_SEL_I2S_IN:
        _info("dmac_id(%d): start(%d),error(%d),monbuf(%d)\n", dmac_id,
            read_bca_reg(BCA_I2s1_In_Mon_start),
            read_bca_reg(BCA_I2s1_In_Mon_error_setting),
            read_bca_reg(BCA_I2s1_In_Mon_monbuf));
        break;

      case AS_DMAC_SEL_I2S2_IN:
        _info("dmac_id(%d): start(%d),error(%d),monbuf(%d)\n", dmac_id,
            read_bca_reg(BCA_I2s2_In_Mon_start),
            read_bca_reg(BCA_I2s2_In_Mon_error_setting),
            read_bca_reg(BCA_I2s2_In_Mon_monbuf));
        break;

      case AS_DMAC_SEL_I2S_OUT:
        _info("dmac_id(%d): start(%d),error(%d),monbuf(%d)\n", dmac_id,
            read_bca_reg(BCA_I2s1_Out_Mon_start),
            read_bca_reg(BCA_I2s1_Out_Mon_error_setting),
            read_bca_reg(BCA_I2s1_Out_Mon_monbuf));
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        _info("dmac_id(%d): start(%d),error(%d),monbuf(%d)\n", dmac_id,
            read_bca_reg(BCA_I2s2_Out_Mon_start),
            read_bca_reg(BCA_I2s2_Out_Mon_error_setting),
            read_bca_reg(BCA_I2s2_Out_Mon_monbuf));
        break;

      default:
        _info("Error: dmac_id(%d)\n", dmac_id);
        break;
    }
}

/*--------------------------------------------------------------------*/
void asDmac_EnableInt(void)
{
  volatile uint32_t int_en;

  int_en = read_as_reg(AS_INT_EN1_REG);

  int_en |= (1 << AS_INT_EN1_BIT_AU0); /* INT_MIC */
  int_en |= (1 << AS_INT_EN1_BIT_AU1); /* INT_I2S1 */
  int_en |= (1 << AS_INT_EN1_BIT_AU2); /* INT_I2S2 */
  int_en |= (1 << AS_INT_EN1_BIT_AU3); /* INT_CODEC */

  write_as_reg(AS_INT_EN1_REG, int_en);

  /* Enalbe bus error interrupt. */

  write_bca_reg(BCA_Int_m_hresp_err, 0);

  return;
}

/*--------------------------------------------------------------------*/
void asDmac_DisableInt(void)
{
  volatile uint32_t int_en;

  int_en = read_as_reg(AS_INT_EN1_REG);

  int_en &= ~(1 << AS_INT_EN1_BIT_AU0); /* INT_MIC */
  int_en &= ~(1 << AS_INT_EN1_BIT_AU1); /* INT_I2S1 */
  int_en &= ~(1 << AS_INT_EN1_BIT_AU2); /* INT_I2S2 */
  int_en &= ~(1 << AS_INT_EN1_BIT_AU3); /* INT_CODEC */

  write_as_reg(AS_INT_EN1_REG, int_en);

  /* Disalbe bus error interrupt. */

  write_bca_reg(BCA_Int_m_hresp_err, 1);

  return;
}

/*--------------------------------------------------------------------*/
E_AS asDmac_RegsitIntCb(asDmacSelId dmacId, AS_DmaIntCb pDmacCb)
{
  if (AS_DMAC_SEL_MAX_ENTRY < dmacId)
    {
      return E_AS_DMAC_ID_PARAM;
    }

  s_DmaIntCb[dmacId] = pDmacCb;

  return E_AS_OK;
}

/*--------------------------------------------------------------------*/
void setDmacBusInt(void)
{
  /* Enable interrupt of bus error */

  write_bca_reg(BCA_Int_m_hresp_err, 0);
}

/*--------------------------------------------------------------------*/
void AS_AudioIntHandler(void)
{
  uint32_t int_irq = read_as_reg(AS_INT_IRQ1_REG);

  uint32_t int_ac =
    read32_bca_reg(bcaRegMap[BCA_Mic_Int_Ctrl_done_mic].addr)
    & ~(read32_bca_reg(bcaRegMap[BCA_Mic_Int_Mask_done_mic].addr))
    & 0x0f;

  uint32_t int_i2s =
    read32_bca_reg(bcaRegMap[BCA_I2s1_Int_Ctrl_done_i2so].addr)
    & ~(read32_bca_reg(bcaRegMap[BCA_I2s1_Int_Mask_done_i2so].addr))
    & 0x3f;

  uint32_t int_i2s2 =
    read32_bca_reg(bcaRegMap[BCA_I2s2_Int_Ctrl_done_i2so].addr)
    & ~(read32_bca_reg(bcaRegMap[BCA_I2s2_Int_Mask_done_i2so].addr))
    & 0x3f;

  /* AUDIO_INT_AC : check interruption from mic */

  if (int_irq & (1 << AS_INT_IRQ1_BIT_AU0))
    {
      if (int_ac != 0)
        {
          write32_bca_reg(bcaRegMap[BCA_Mic_Int_Ctrl_done_mic].addr, int_ac);

          if (int_ac & (1<<bcaRegMap[BCA_Mic_Int_Ctrl_done_mic].pos))
            {
              (*s_DmaIntCb[AS_DMAC_SEL_AC_IN])(AS_DMAC_SEL_AC_IN,
                                               E_AS_DMA_INT_CMPLT);
            }

          if (int_ac & (1<<bcaRegMap[BCA_Mic_Int_Ctrl_err_mic].pos))
            {
              setDmacErrIntMask(AS_DMAC_SEL_AC_IN, true);

              clearDmacErrIntStatus(AS_DMAC_SEL_AC_IN);

#ifdef CHECK_DMA_ERR_INT
              (*s_DmaIntCb[AS_DMAC_SEL_AC_IN])(AS_DMAC_SEL_AC_IN,
                                               E_AS_DMA_INT_ERR);
#endif
            }

          if (int_ac & (1<<bcaRegMap[BCA_Mic_Int_Ctrl_cmb_mic].pos))
            {
              setDmacCmbIntMask(AS_DMAC_SEL_AC_IN, true);

              clearDmacCmbIntStatus(AS_DMAC_SEL_AC_IN);

#ifdef CHECK_BUS_ERR_INT
              (*s_DmaIntCb[AS_DMAC_SEL_AC_IN])(AS_DMAC_SEL_AC_IN,
                                               E_AS_DMA_INT_ERR);
#endif
            }
        }
    }

  /* AUDIO_INT_I2S1 : check interruption from I2S-1*/

  if (int_irq & (1 << AS_INT_IRQ1_BIT_AU1))
    {
      if (int_i2s != 0)
        {
          write32_bca_reg(bcaRegMap[BCA_I2s1_Int_Ctrl_done_i2so].addr,
                          int_i2s);

          if (int_i2s & (1<<bcaRegMap[BCA_I2s1_Int_Ctrl_done_i2si].pos))
            {
              (*s_DmaIntCb[AS_DMAC_SEL_I2S_IN])(AS_DMAC_SEL_I2S_IN,
                                                E_AS_DMA_INT_CMPLT);
            }

          if (int_i2s & (1<<bcaRegMap[BCA_I2s1_Int_Ctrl_done_i2so].pos))
            {
              (*s_DmaIntCb[AS_DMAC_SEL_I2S_OUT])(AS_DMAC_SEL_I2S_OUT,
                                                 E_AS_DMA_INT_CMPLT);
            }

          if (int_i2s & (1<<bcaRegMap[BCA_I2s1_Int_Ctrl_err_i2si].pos))
            {
              setDmacErrIntMask(AS_DMAC_SEL_I2S_IN, true);

              clearDmacErrIntStatus(AS_DMAC_SEL_I2S_IN);

#ifdef CHECK_DMA_ERR_INT
              (*s_DmaIntCb[AS_DMAC_SEL_I2S_IN])(AS_DMAC_SEL_I2S_IN,
                                                E_AS_DMA_INT_ERR);
#endif
            }

          if (int_i2s & (1<<bcaRegMap[BCA_I2s1_Int_Ctrl_err_i2so].pos))
            {
              setDmacErrIntMask(AS_DMAC_SEL_I2S_OUT, true);

              clearDmacErrIntStatus(AS_DMAC_SEL_I2S_OUT);

#ifdef CHECK_DMA_ERR_INT
              (*s_DmaIntCb[AS_DMAC_SEL_I2S_OUT])(AS_DMAC_SEL_I2S_OUT,
                                                 E_AS_DMA_INT_ERR);
#endif
            }

          if (int_i2s & (1<<bcaRegMap[BCA_I2s1_Int_Ctrl_cmb_i2s].pos))
            {
              setDmacCmbIntMask(AS_DMAC_SEL_I2S_OUT, true);

              clearDmacCmbIntStatus(AS_DMAC_SEL_I2S_OUT);

#ifdef CHECK_BUS_ERR_INT
              (*s_DmaIntCb[AS_DMAC_SEL_I2S_OUT])(AS_DMAC_SEL_I2S_OUT,
                                                 E_AS_DMA_INT_ERR_BUS);
#endif
            }
        }
    }

  /* AUDIO_INT_I2S2 : check interruption from I2S-2 */

  if (int_irq & (1 << AS_INT_IRQ1_BIT_AU2))
    {
      if (int_i2s2 != 0)
        {
          write32_bca_reg(bcaRegMap[BCA_I2s2_Int_Ctrl_done_i2so].addr,
                          int_i2s2);

          if (int_i2s2 & (1<<bcaRegMap[BCA_I2s2_Int_Ctrl_done_i2si].pos))
            {
              (*s_DmaIntCb[AS_DMAC_SEL_I2S2_IN])(AS_DMAC_SEL_I2S2_IN,
                                                 E_AS_DMA_INT_CMPLT);
            }

          if (int_i2s2 & (1<<bcaRegMap[BCA_I2s2_Int_Ctrl_done_i2so].pos))
            {
              (*s_DmaIntCb[AS_DMAC_SEL_I2S2_OUT])(AS_DMAC_SEL_I2S2_OUT,
                                                  E_AS_DMA_INT_CMPLT);
            }

          if (int_i2s2 & (1<<bcaRegMap[BCA_I2s2_Int_Ctrl_err_i2si].pos))
            {
              setDmacErrIntMask(AS_DMAC_SEL_I2S2_IN, true);

              clearDmacErrIntStatus(AS_DMAC_SEL_I2S2_IN);

#ifdef CHECK_DMA_ERR_INT
              (*s_DmaIntCb[AS_DMAC_SEL_I2S2_IN])(AS_DMAC_SEL_I2S2_IN,
                                                 E_AS_DMA_INT_ERR);
#endif
            }

          if (int_i2s2 & (1<<bcaRegMap[BCA_I2s2_Int_Ctrl_err_i2so].pos))
            {
              setDmacErrIntMask(AS_DMAC_SEL_I2S2_OUT, true);

              clearDmacErrIntStatus(AS_DMAC_SEL_I2S2_OUT);

#ifdef CHECK_DMA_ERR_INT
              (*s_DmaIntCb[AS_DMAC_SEL_I2S2_OUT])(AS_DMAC_SEL_I2S2_OUT,
                                                  E_AS_DMA_INT_ERR);
#endif
            }

          if (int_i2s2 & (1<<bcaRegMap[BCA_I2s2_Int_Ctrl_cmb_i2s].pos))
            {
              setDmacCmbIntMask(AS_DMAC_SEL_I2S2_OUT, true);

              clearDmacCmbIntStatus(AS_DMAC_SEL_I2S2_OUT);

#ifdef CHECK_BUS_ERR_INT
              (*s_DmaIntCb[AS_DMAC_SEL_I2S2_OUT])(AS_DMAC_SEL_I2S2_OUT,
                                                  E_AS_DMA_INT_ERR_BUS);
#endif
            }
        }
    }

  if (int_irq & (1 << AS_INT_IRQ1_BIT_AU3))
    {
      uint32_t int_au  = read32_bca_reg(bcaRegMap[BCA_Int_hresp_err].addr);

      if (int_au != 0)
        {
          write32_bca_reg(bcaRegMap[BCA_Int_clr_hresp_err].addr, int_au);

#ifdef CHECK_AC_INT
          ac_int_callback(int_au);
#endif
        }
    }
}
