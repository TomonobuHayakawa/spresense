/********************************************************************************************
 * arch/arm/src/cxd56xx/audio/drivers/baseband/src/audio_io_config.c
 *
 *   Copyright (C) 2015 Sony Corporation. All rights reserved.
 *   Author: Tomonobu Hayakawa<Tomonobu.Hayakawa@sony.com>
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
 ********************************************************************************************/
/* Description: Audio BaseBand I/O Settings */

/*******************************************************
    Include
*******************************************************/

#include <nuttx/config.h>

#include "cxd56_pinconfig.h"
#include "audio/common_assert.h"
#include "audio/audio_io_config.h"

/* IO Setting for Audio MCLK */
void setAudioIoMclk(void)
{
	CXD56_PIN_CONFIGS(PINCONFS_MCLK);
}

/* IO Setting for Audio PDM */
void setAudioIoPdm(audioIoLowemi lowemi)
{
	switch(lowemi) {
	case AUDIO_IO_LOWEMI_UNKNOWN:
		/* Do Nothing. */
		break;
	case AUDIO_IO_LOWEMI_4MA:
		CXD56_PIN_CONFIGS(PINCONFS_PDM_HIGH);
		break;
	case AUDIO_IO_LOWEMI_2MA:
		CXD56_PIN_CONFIGS(PINCONFS_PDM_NORM);
		break;
	default:
		F_ASSERT(0);
	}
}

#ifdef CONFIG_CXD56_I2S0
/* IO Setting for Audio I2S0 */
void setAudioIoI2s0(audioIoI2sMode i2s_mode, audioIoLowemi lowemi)
{
	switch(i2s_mode) {
	case AUDIO_IO_I2S_MODE_UNKNOWN:
		/* Do Nothing. */
		break;
	case AUDIO_IO_I2S_MODE_MASTER:
		if (AUDIO_IO_LOWEMI_4MA == lowemi) {
			CXD56_PIN_CONFIGS(PINCONFS_I2S0_M_HIGH);
		} else if (AUDIO_IO_LOWEMI_2MA == lowemi) {
			CXD56_PIN_CONFIGS(PINCONFS_I2S0_M_NORM);
		} else {
			/* Do Nothing. */
		}
		break;
	case AUDIO_IO_I2S_MODE_SLAVE:
		if (AUDIO_IO_LOWEMI_4MA == lowemi) {
			CXD56_PIN_CONFIGS(PINCONFS_I2S0_S_HIGH);
		} else if (AUDIO_IO_LOWEMI_2MA == lowemi) {
			CXD56_PIN_CONFIGS(PINCONFS_I2S0_S_NORM);
		} else {
			/* Do Nothing. */
		}
		break;
	default:
		F_ASSERT(0);
	}
}
#endif /* CONFIG_CXD56_I2S0 */

#ifdef CONFIG_CXD56_I2S1
/* IO Setting for Audio I2S1 */
void setAudioIoI2s1(audioIoI2sMode i2s_mode, audioIoLowemi lowemi)
{
	switch(i2s_mode) {
	case AUDIO_IO_I2S_MODE_UNKNOWN:
		/* Do Nothing. */
		break;
	case AUDIO_IO_I2S_MODE_MASTER:
		if (AUDIO_IO_LOWEMI_4MA == lowemi) {
			CXD56_PIN_CONFIGS(PINCONFS_I2S1_M_HIGH);
		} else if (AUDIO_IO_LOWEMI_2MA == lowemi) {
			CXD56_PIN_CONFIGS(PINCONFS_I2S1_M_NORM);
		} else {
			/* Do Nothing. */
		}
		break;
	case AUDIO_IO_I2S_MODE_SLAVE:
		if (AUDIO_IO_LOWEMI_4MA == lowemi) {
			CXD56_PIN_CONFIGS(PINCONFS_I2S1_S_HIGH);
		} else if (AUDIO_IO_LOWEMI_2MA == lowemi) {
			CXD56_PIN_CONFIGS(PINCONFS_I2S1_S_NORM);
		} else {
			/* Do Nothing. */
		}
		break;
	default:
		F_ASSERT(0);
	}
}
#endif /* CONFIG_CXD56_I2S1 */

/* IO Setting for Audio I2S */
void setAudioIoI2s(audioIoI2sSel i2s_sel, audioIoI2sMode i2s_mode, audioIoLowemi lowemi)
{
	switch (i2s_sel) {
#ifdef CONFIG_CXD56_I2S0
	case AUDIO_IO_I2S_SEL_I2S0:
		setAudioIoI2s0(i2s_mode, lowemi);
		break;
#endif /* CONFIG_CXD56_I2S0 */
#ifdef CONFIG_CXD56_I2S1
	case AUDIO_IO_I2S_SEL_I2S1:
		setAudioIoI2s1(i2s_mode, lowemi);
		break;
#endif /* CONFIG_CXD56_I2S1 */
	default:
		F_ASSERT(0);
	}
}
