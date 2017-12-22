/****************************************************************************
 * nuttx/arch/arm/src/cxd56xx/audio/drivers/baseband/src/bca_drv_api.c
 *
 *   Copyright (C) 2016-2017 Sony Corporation. All rights reserved.
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
 ****************************************************************************/

#include <arch/chip/cxd56_audio.h>

#include "audio/as_drv_common.h"

E_AS asBca_StopDmac( asDmacSelId dmacId )
{
	E_AS rtCode = E_AS_OK;
	uint32_t stat = 0;

	rtCode = getDmacCmdStatus( dmacId, &stat );
	if (rtCode != E_AS_OK) return rtCode;
	if (stat != 1) {
		return E_AS_DMAC_BUSY;
	}

	switch (dmacId) {
	case AS_DMAC_SEL_AC_IN:
		write32_bca_reg( bcaRegMap[BCA_Mic_In_rtd_trg].addr, 0x04 );
		break;
	case AS_DMAC_SEL_I2S_IN:
		write32_bca_reg( bcaRegMap[BCA_I2s1_In_rtd_trg].addr, 0x04 );
		break;
	case AS_DMAC_SEL_I2S_OUT:
		write32_bca_reg( bcaRegMap[BCA_I2s1_Out_rtd_trg].addr, 0x04 );
		break;
	case AS_DMAC_SEL_I2S2_IN:
		write32_bca_reg( bcaRegMap[BCA_I2s2_In_rtd_trg].addr, 0x04 );
		break;
	case AS_DMAC_SEL_I2S2_OUT:
		write32_bca_reg( bcaRegMap[BCA_I2s2_Out_rtd_trg].addr, 0x04 );
		break;
	default:
		D_ASSERT(0);
		break;
	}

	return rtCode;
}

void asBca_SetSmstrParam()
{
	write_bca_reg( BCA_Int_m_ovf_smasl, 0 );
	write_bca_reg( BCA_Int_m_ovf_smasr, 0 );
}

E_AS asBca_SetSrcParam()
{
	E_AS rtCode = E_AS_OK;

	switch(bb_config_tblp->i2s_data_path) {
	case AS_I2S_DATA_PATH_1:
		write_bca_reg( BCA_Int_m_i2s1_bck_err1, 0 );
		break;
	case AS_I2S_DATA_PATH_2:
		write_bca_reg( BCA_Int_m_i2s1_bck_err1, 0 );
		write_bca_reg( BCA_Int_m_i2s1_bck_err2, 0 );
		break;
	default:
		/* do nothing */
		break;
	}
	return rtCode;
}

E_AS asBca_SetDncParam(asDncSelId dncId)
{
	E_AS rtCode = E_AS_OK;

	switch( dncId ){
	case AS_DNC_SEL_DNC1:
		write_bca_reg( BCA_Int_m_anc_faint, 0 );
		write_bca_reg( BCA_Int_m_ovf_dnc1l, 0 );
		write_bca_reg( BCA_Int_m_ovf_dnc1r, 0 );
		break;
	case AS_DNC_SEL_DNC2:
		write_bca_reg( BCA_Int_m_anc_faint, 0 );
		write_bca_reg( BCA_Int_m_ovf_dnc2l, 0 );
		write_bca_reg( BCA_Int_m_ovf_dnc2r, 0 );
		break;
	case AS_DNC_SEL_BOTH:
		write_bca_reg( BCA_Int_m_anc_faint, 0 );
		write_bca_reg( BCA_Int_m_ovf_dnc1l, 0 );
		write_bca_reg( BCA_Int_m_ovf_dnc1r, 0 );
		write_bca_reg( BCA_Int_m_ovf_dnc2l, 0 );
		write_bca_reg( BCA_Int_m_ovf_dnc2r, 0 );
		break;
	default:
		return E_AS_DNC_SEL_PARAM;
	}

	return rtCode;
}

