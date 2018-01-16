/****************************************************************************
 * arch/arm/src/cxd56xx/audio/drivers/baseband/src/bca_drv_sub_func.c
 *
 *   Copyright (C) 2016, 2017 Sony Corporation
 *   Author: Naoya Haneda <Naoya.Haneda@sony.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/chip/cxd56_audio.h>
#include "audio/common_assert.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

E_AS getDmacCmdStatus(asDmacSelId dmacId, FAR uint32_t *pResult)
{
  E_AS rtCode = E_AS_OK;

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        *pResult = read_bca_reg(BCA_Mic_In_rtd_trg);
        break;

      case AS_DMAC_SEL_I2S_IN:
        *pResult = read_bca_reg(BCA_I2s1_In_rtd_trg);
        break;

      case AS_DMAC_SEL_I2S_OUT:
        *pResult = read_bca_reg(BCA_I2s1_Out_rtd_trg);
        break;

      case AS_DMAC_SEL_I2S2_IN:
        *pResult = read_bca_reg(BCA_I2s2_In_rtd_trg);
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        *pResult = read_bca_reg(BCA_I2s2_Out_rtd_trg);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS getDmacErrorStatus(asDmacSelId dmacId, FAR uint32_t *pResult)
{
  E_AS rtCode = E_AS_OK;

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        *pResult = read_bca_reg(BCA_Mic_In_error_setting);
        break;

      case AS_DMAC_SEL_I2S_IN:
        *pResult = read_bca_reg(BCA_I2s1_In_Mon_error_setting);
        break;

      case AS_DMAC_SEL_I2S_OUT:
        *pResult = read_bca_reg(BCA_I2s1_Out_Mon_error_setting);
        break;

      case AS_DMAC_SEL_I2S2_IN:
        *pResult = read_bca_reg(BCA_I2s2_In_Mon_error_setting);
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        *pResult = read_bca_reg(BCA_I2s2_Out_Mon_error_setting);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS setDmacAddr(asDmacSelId dmacId, uint32_t addr)
{
  E_AS rtCode = E_AS_OK;

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        write_bca_reg(BCA_Mic_In_start_adr, addr >> 2);
        break;

      case AS_DMAC_SEL_I2S_IN:
        write_bca_reg(BCA_I2s1_In_start_adr, addr >> 2);
        break;

      case AS_DMAC_SEL_I2S_OUT:
        write_bca_reg(BCA_I2s1_Out_start_adr, addr >> 2);
        break;

      case AS_DMAC_SEL_I2S2_IN:
        write_bca_reg(BCA_I2s2_In_start_adr, addr >> 2);
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        write_bca_reg(BCA_I2s2_Out_start_adr, addr >> 2);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS setDmacSample(asDmacSelId dmacId, uint32_t sample)
{
  E_AS rtCode = E_AS_OK;

  D_ASSERT((1 <= sample) && (sample <= 1024));

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        write_bca_reg(BCA_Mic_In_sample_no, sample - 1);
        break;

      case AS_DMAC_SEL_I2S_IN:
        write_bca_reg(BCA_I2s1_In_sample_no, sample - 1);
        break;

      case AS_DMAC_SEL_I2S_OUT:
        write_bca_reg(BCA_I2s1_Out_sample_no, sample - 1);
        break;

      case AS_DMAC_SEL_I2S2_IN:
        write_bca_reg(BCA_I2s2_In_sample_no, sample - 1);
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        write_bca_reg(BCA_I2s2_Out_sample_no, sample - 1);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS setDmacTrg(asDmacSelId dmacId, bool nointr)
{
  E_AS rtCode = E_AS_OK;

  uint32_t val = ((nointr == true) ? 0x05 : 0x01);

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        write32_bca_reg(bcaRegMap[BCA_Mic_In_rtd_trg].addr, val);
        break;

      case AS_DMAC_SEL_I2S_IN:
        write32_bca_reg(bcaRegMap[BCA_I2s1_In_rtd_trg].addr, val);
        break;

      case AS_DMAC_SEL_I2S_OUT:
        write32_bca_reg(bcaRegMap[BCA_I2s1_Out_rtd_trg].addr, val);
        break;

      case AS_DMAC_SEL_I2S2_IN:
        write32_bca_reg(bcaRegMap[BCA_I2s2_In_rtd_trg].addr, val);
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        write32_bca_reg(bcaRegMap[BCA_I2s2_Out_rtd_trg].addr, val);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}


E_AS setDmacDoneIntMask(asDmacSelId dmacId, bool mask)
{
  E_AS rtCode = E_AS_OK;

  uint32_t val = ((mask == true) ? 1 : 0);

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        write_bca_reg(BCA_Mic_Int_Mask_done_mic, val);
        break;

      case AS_DMAC_SEL_I2S_IN:
        write_bca_reg(BCA_I2s1_Int_Mask_done_i2si, val);
        break;

      case AS_DMAC_SEL_I2S_OUT:
        write_bca_reg(BCA_I2s1_Int_Mask_done_i2so, val);
        break;

      case AS_DMAC_SEL_I2S2_IN:
        write_bca_reg(BCA_I2s2_Int_Mask_done_i2si, val);
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        write_bca_reg(BCA_I2s2_Int_Mask_done_i2so, val);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS getDmacDoneIntStatus(asDmacSelId dmacId, FAR uint32_t *pResult)
{
  E_AS rtCode = E_AS_OK;

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        *pResult = read_bca_reg(BCA_Mic_Int_Ctrl_done_mic);
        break;

      case AS_DMAC_SEL_I2S_IN:
        *pResult = read_bca_reg(BCA_I2s1_Int_Ctrl_done_i2si);
        break;

      case AS_DMAC_SEL_I2S_OUT:
        *pResult = read_bca_reg(BCA_I2s1_Int_Ctrl_done_i2so);
        break;

      case AS_DMAC_SEL_I2S2_IN:
        *pResult = read_bca_reg(BCA_I2s2_Int_Ctrl_done_i2si);
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        *pResult = read_bca_reg(BCA_I2s2_Int_Ctrl_done_i2so);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS clearDmacDoneIntStatus(asDmacSelId dmacId)
{
  E_AS rtCode = E_AS_OK;

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        write_bca_reg_mask(BCA_Mic_Int_Ctrl_done_mic);
        break;

      case AS_DMAC_SEL_I2S_IN:
        write_bca_reg_mask(BCA_I2s1_Int_Ctrl_done_i2si);
        break;

      case AS_DMAC_SEL_I2S_OUT:
        write_bca_reg_mask(BCA_I2s1_Int_Ctrl_done_i2so);
        break;

      case AS_DMAC_SEL_I2S2_IN:
        write_bca_reg_mask(BCA_I2s2_Int_Ctrl_done_i2si);
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        write_bca_reg_mask(BCA_I2s2_Int_Ctrl_done_i2so);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS setDmacErrIntMask(asDmacSelId dmacId, bool mask)
{
  E_AS rtCode = E_AS_OK;

  uint32_t val = ((mask == true) ? 1 : 0);
  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        write_bca_reg(BCA_Mic_Int_Mask_err_mic, val);
        break;

      case AS_DMAC_SEL_I2S_IN:
        write_bca_reg(BCA_I2s1_Int_Mask_err_i2si, val);
        break;

      case AS_DMAC_SEL_I2S_OUT:
        write_bca_reg(BCA_I2s1_Int_Mask_err_i2so, val);
        break;

      case AS_DMAC_SEL_I2S2_IN:
        write_bca_reg(BCA_I2s2_Int_Mask_err_i2si, val);
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        write_bca_reg(BCA_I2s2_Int_Mask_err_i2so, val);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS getDmacErrIntStatus(asDmacSelId dmacId, FAR uint32_t *pResult)
{
  E_AS rtCode = E_AS_OK;

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        *pResult = read_bca_reg(BCA_Mic_Int_Ctrl_err_mic);
        break;

      case AS_DMAC_SEL_I2S_IN:
        *pResult = read_bca_reg(BCA_I2s1_Int_Ctrl_err_i2si);
        break;

      case AS_DMAC_SEL_I2S_OUT:
        *pResult = read_bca_reg(BCA_I2s1_Int_Ctrl_err_i2so);
        break;

      case AS_DMAC_SEL_I2S2_IN:
        *pResult = read_bca_reg(BCA_I2s2_Int_Ctrl_err_i2si);
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        *pResult = read_bca_reg(BCA_I2s2_Int_Ctrl_err_i2so);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS clearDmacErrIntStatus(asDmacSelId dmacId)
{
  E_AS rtCode = E_AS_OK;

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        write_bca_reg_mask(BCA_Mic_Int_Ctrl_err_mic);
        break;

      case AS_DMAC_SEL_I2S_IN:
        write_bca_reg_mask(BCA_I2s1_Int_Ctrl_err_i2si);
        break;

      case AS_DMAC_SEL_I2S_OUT:
        write_bca_reg_mask(BCA_I2s1_Int_Ctrl_err_i2so);
        break;

      case AS_DMAC_SEL_I2S2_IN:
        write_bca_reg_mask(BCA_I2s2_Int_Ctrl_err_i2si);
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        write_bca_reg_mask(BCA_I2s2_Int_Ctrl_err_i2so);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS setDmacSmpIntMask(asDmacSelId dmacId, bool mask)
{
  E_AS rtCode = E_AS_OK;

  uint32_t val = ((mask == true) ? 1 : 0);

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        write_bca_reg(BCA_Mic_Int_Mask_smp_mic, val);
        break;

      case AS_DMAC_SEL_I2S_IN:
      case AS_DMAC_SEL_I2S_OUT:
        write_bca_reg(BCA_I2s1_Int_Mask_smp_i2s, val);
        break;

      case AS_DMAC_SEL_I2S2_IN:
      case AS_DMAC_SEL_I2S2_OUT:
        write_bca_reg(BCA_I2s2_Int_Mask_smp_i2s, val);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS getDmacSmpIntStatus(asDmacSelId dmacId, FAR uint32_t *pResult)
{
  E_AS rtCode = E_AS_OK;

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        *pResult = read_bca_reg(BCA_Mic_Int_Ctrl_smp_mic);
        break;

      case AS_DMAC_SEL_I2S_IN:
      case AS_DMAC_SEL_I2S_OUT:
        *pResult = read_bca_reg(BCA_I2s1_Int_Ctrl_smp_i2s);
        break;

      case AS_DMAC_SEL_I2S2_IN:
      case AS_DMAC_SEL_I2S2_OUT:
        *pResult = read_bca_reg(BCA_I2s2_Int_Ctrl_smp_i2s);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS clearDmacSmpIntStatus(asDmacSelId dmacId)
{
  E_AS rtCode = E_AS_OK;

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        write_bca_reg_mask(BCA_Mic_Int_Ctrl_smp_mic);
        break;

      case AS_DMAC_SEL_I2S_IN:
      case AS_DMAC_SEL_I2S_OUT:
        write_bca_reg_mask(BCA_I2s1_Int_Ctrl_smp_i2s);
        break;

      case AS_DMAC_SEL_I2S2_IN:
      case AS_DMAC_SEL_I2S2_OUT:
        write_bca_reg_mask(BCA_I2s2_Int_Ctrl_smp_i2s);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS getDmacChannel(asDmacSelId dmacId, FAR uint32_t *pResult)
{
  E_AS rtCode = E_AS_OK;

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        *pResult = read32_bca_reg(bcaRegMap[BCA_Mic_In_ch8_sel].addr);
        break;

      case AS_DMAC_SEL_I2S_IN:
        *pResult = read32_bca_reg(bcaRegMap[BCA_I2s1_In_ch2_sel].addr);
        break;

      case AS_DMAC_SEL_I2S_OUT:
        *pResult = read32_bca_reg(bcaRegMap[BCA_I2s1_Out_sd1_r_sel].addr);
        break;

      case AS_DMAC_SEL_I2S2_IN:
        *pResult = read32_bca_reg(bcaRegMap[BCA_I2s2_In_ch2_sel].addr);
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        *pResult = read32_bca_reg(bcaRegMap[BCA_I2s2_Out_sd1_r_sel].addr);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS setDmacTrgWithChSel(asDmacSelId dmacId, bool nointr, uint32_t chsel)
{
  E_AS rtCode = E_AS_OK;
  uint32_t val = ((nointr == true) ? 0x05 : 0x01);

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        write32_bca_reg(bcaRegMap[BCA_Mic_In_ch8_sel].addr, 0xffffffff);
        write32_bca_reg(bcaRegMap[BCA_Mic_In_ch8_sel].addr, chsel);
        write32_bca_reg(bcaRegMap[BCA_Mic_In_rtd_trg].addr, val);
        break;

      case AS_DMAC_SEL_I2S_IN:
        write32_bca_reg(bcaRegMap[BCA_I2s1_In_ch2_sel].addr, 0xffffffff);
        write32_bca_reg(bcaRegMap[BCA_I2s1_In_ch2_sel].addr, chsel);
        write32_bca_reg(bcaRegMap[BCA_I2s1_In_rtd_trg].addr, val);
        break;

      case AS_DMAC_SEL_I2S_OUT:
        write32_bca_reg(bcaRegMap[BCA_I2s1_Out_sd1_r_sel].addr, 0xffffffff);
        write32_bca_reg(bcaRegMap[BCA_I2s1_Out_sd1_r_sel].addr, chsel);
        write32_bca_reg(bcaRegMap[BCA_I2s1_Out_rtd_trg].addr, val);
        break;

      case AS_DMAC_SEL_I2S2_IN:
        write32_bca_reg(bcaRegMap[BCA_I2s2_In_ch2_sel].addr, 0xffffffff);
        write32_bca_reg(bcaRegMap[BCA_I2s2_In_ch2_sel].addr, chsel);
        write32_bca_reg(bcaRegMap[BCA_I2s2_In_rtd_trg].addr, val);
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        write32_bca_reg(bcaRegMap[BCA_I2s2_Out_sd1_r_sel].addr, 0xffffffff);
        write32_bca_reg(bcaRegMap[BCA_I2s2_Out_sd1_r_sel].addr, chsel);
        write32_bca_reg(bcaRegMap[BCA_I2s2_Out_rtd_trg].addr, val);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}


E_AS getDmacMonbufStatus(asDmacSelId dmacId, FAR uint32_t *pResult)
{
  E_AS rtCode = E_AS_OK;

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        *pResult = read_bca_reg(BCA_Mic_In_monbuf);
        break;

      case AS_DMAC_SEL_I2S_IN:
        *pResult = read_bca_reg(BCA_I2s1_In_Mon_monbuf);
        break;

      case AS_DMAC_SEL_I2S_OUT:
        *pResult = read_bca_reg(BCA_I2s1_Out_Mon_monbuf);
        break;

      case AS_DMAC_SEL_I2S2_IN:
        *pResult = read_bca_reg(BCA_I2s2_In_Mon_monbuf);
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        *pResult = read_bca_reg(BCA_I2s2_Out_Mon_monbuf);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS getDmacMonStartStatus(asDmacSelId dmacId, FAR uint32_t *pResult)
{
  E_AS rtCode = E_AS_OK;

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        *pResult = read_bca_reg(BCA_Mic_In_start);
        break;

      case AS_DMAC_SEL_I2S_IN:
        *pResult = read_bca_reg(BCA_I2s1_In_Mon_start);
        break;

      case AS_DMAC_SEL_I2S_OUT:
        *pResult = read_bca_reg(BCA_I2s1_Out_Mon_start);
        break;

      case AS_DMAC_SEL_I2S2_IN:
        *pResult = read_bca_reg(BCA_I2s2_In_Mon_start);
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        *pResult = read_bca_reg(BCA_I2s2_Out_Mon_start);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS setDmacStop(asDmacSelId dmacId)
{
  E_AS rtCode = E_AS_OK;

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        write32_bca_reg(bcaRegMap[BCA_Mic_In_rtd_trg].addr, 0x04);
        break;

      case AS_DMAC_SEL_I2S_IN:
        write32_bca_reg(bcaRegMap[BCA_I2s1_In_rtd_trg].addr, 0x04);
        break;

      case AS_DMAC_SEL_I2S_OUT:
        write32_bca_reg(bcaRegMap[BCA_I2s1_Out_rtd_trg].addr, 0x04);
        break;

      case AS_DMAC_SEL_I2S2_IN:
        write32_bca_reg(bcaRegMap[BCA_I2s2_In_rtd_trg].addr, 0x04);
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        write32_bca_reg(bcaRegMap[BCA_I2s2_Out_rtd_trg].addr, 0x04);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}


/* Bus error */

E_AS setDmacCmbIntMask(asDmacSelId dmacId, bool mask)
{
  E_AS rtCode = E_AS_OK;

  uint32_t val = ((mask == true) ? 1 : 0);

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        write_bca_reg(BCA_Mic_Int_Mask_cmb_mic, val);
        break;

      case AS_DMAC_SEL_I2S_IN:
      case AS_DMAC_SEL_I2S_OUT:
        write_bca_reg(BCA_I2s1_Int_Mask_cmb_i2s, val);
        break;

      case AS_DMAC_SEL_I2S2_IN:
      case AS_DMAC_SEL_I2S2_OUT:
        write_bca_reg(BCA_I2s2_Int_Mask_cmb_i2s, val);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS clearDmacCmbIntStatus(asDmacSelId dmacId)
{
  E_AS rtCode = E_AS_OK;

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        write_bca_reg_mask(BCA_Mic_Int_Ctrl_cmb_mic);
        break;

      case AS_DMAC_SEL_I2S_IN:
      case AS_DMAC_SEL_I2S_OUT:
        write_bca_reg_mask(BCA_I2s1_Int_Ctrl_cmb_i2s);
        break;

      case AS_DMAC_SEL_I2S2_IN:
      case AS_DMAC_SEL_I2S2_OUT:
        write_bca_reg_mask(BCA_I2s2_Int_Ctrl_cmb_i2s);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}

E_AS setDmacBusIntMask(asDmacSelId dmacId, bool mask)
{
  E_AS rtCode = E_AS_OK;
  uint32_t data;
  uint32_t val = ((mask == true) ? 0x00000000 : 0x00000fff);

  switch (dmacId)
    {
      case AS_DMAC_SEL_AC_IN:
        data = (0x00000303 & val);
        write_bca_reg(AHB_Master_Mic_Mask, data);
        break;

      case AS_DMAC_SEL_I2S_IN:
        data = read_bca_reg(AHB_Master_I2s1_Mask);
        data = data | (0x00000101 & val) ;
        write_bca_reg(AHB_Master_I2s1_Mask, data);
        break;

      case AS_DMAC_SEL_I2S_OUT:
        data = read_bca_reg(AHB_Master_I2s1_Mask);
        data = data | (0x00000202 & val);
        write_bca_reg(AHB_Master_I2s1_Mask, data);
        break;

      case AS_DMAC_SEL_I2S2_IN:
        data = read_bca_reg(AHB_Master_I2s2_Mask);
        data = data | (0x00000101 & val);
        write_bca_reg(AHB_Master_I2s2_Mask, data);
        break;

      case AS_DMAC_SEL_I2S2_OUT:
        data = read_bca_reg(AHB_Master_I2s2_Mask);
        data = data | (0x00000202 & val);
        write_bca_reg(AHB_Master_I2s2_Mask, data);
        break;

      default:
        D_ASSERT(0);
        break;
    }

  return rtCode;
}
