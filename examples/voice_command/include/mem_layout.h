/****************************************************************************
 * voice_command/include/mem_layout.h
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

#ifndef MEM_LAYOUT_H_INCLUDED
#define MEM_LAYOUT_H_INCLUDED

/*
 * Memory devices
 */
/* AUD_SRAM: type=RAM, use=0x0003f300, remainder=0x00000d00 */
#define AUD_SRAM_ADDR  0x000c0000
#define AUD_SRAM_SIZE  0x00040000

/*
 * Fixed areas
 */
#define AUDIO_WORK_AREA_ALIGN   0x00000008
#define AUDIO_WORK_AREA_ADDR    0x000c0000
#define AUDIO_WORK_AREA_DRM     0x000c0000 /* _DRM is obsolete macro. to use _ADDR */
#define AUDIO_WORK_AREA_SIZE    0x0003e000

#define MSG_QUE_AREA_ALIGN   0x00000008
#define MSG_QUE_AREA_ADDR    0x000fe000
#define MSG_QUE_AREA_DRM     0x000fe000 /* _DRM is obsolete macro. to use _ADDR */
#define MSG_QUE_AREA_SIZE    0x00001000

#define MEMMGR_WORK_AREA_ALIGN   0x00000008
#define MEMMGR_WORK_AREA_ADDR    0x000ff000
#define MEMMGR_WORK_AREA_DRM     0x000ff000 /* _DRM is obsolete macro. to use _ADDR */
#define MEMMGR_WORK_AREA_SIZE    0x00000200

#define MEMMGR_DATA_AREA_ALIGN   0x00000008
#define MEMMGR_DATA_AREA_ADDR    0x000ff200
#define MEMMGR_DATA_AREA_DRM     0x000ff200 /* _DRM is obsolete macro. to use _ADDR */
#define MEMMGR_DATA_AREA_SIZE    0x00000100

/*
 * Memory Manager max work area size
 */
#define MEMMGR_MAX_WORK_SIZE  0x000000c4

/*
 * Pool IDs
 */
#define NULL_POOL  0
#define MIC_IN_BUF_POOL  1
#define I2S_IN_BUF_POOL  2
#define HP_OUT_BUF_POOL  3
#define I2S_OUT_BUF_POOL  4
#define MFE_OUT_BUF_POOL  5
#define WUWSR_IN_BUF_POOL  6

#define NUM_MEM_LAYOUTS  1
#define NUM_MEM_POOLS  7


/*
 * Pool areas
 */
/* Layout0: */
#define MEMMGR_L0_WORK_SIZE   0x000000c4

/* Skip 0x0004 bytes for alignment. */
#define L0_MIC_IN_BUF_POOL_ALIGN    0x00000008
#define L0_MIC_IN_BUF_POOL_L_FENCE  0x000c0004
#define L0_MIC_IN_BUF_POOL_ADDR     0x000c0008
#define L0_MIC_IN_BUF_POOL_SIZE     0x00000960
#define L0_MIC_IN_BUF_POOL_U_FENCE  0x000c0968
#define L0_MIC_IN_BUF_POOL_NUM_SEG  0x00000005
#define L0_MIC_IN_BUF_POOL_SEG_SIZE 0x000001e0

#define L0_I2S_IN_BUF_POOL_ALIGN    0x00000008
#define L0_I2S_IN_BUF_POOL_L_FENCE  0x000c096c
#define L0_I2S_IN_BUF_POOL_ADDR     0x000c0970
#define L0_I2S_IN_BUF_POOL_SIZE     0x000012c0
#define L0_I2S_IN_BUF_POOL_U_FENCE  0x000c1c30
#define L0_I2S_IN_BUF_POOL_NUM_SEG  0x00000005
#define L0_I2S_IN_BUF_POOL_SEG_SIZE 0x000003c0

#define L0_HP_OUT_BUF_POOL_ALIGN    0x00000008
#define L0_HP_OUT_BUF_POOL_L_FENCE  0x000c1c34
#define L0_HP_OUT_BUF_POOL_ADDR     0x000c1c38
#define L0_HP_OUT_BUF_POOL_SIZE     0x000012c0
#define L0_HP_OUT_BUF_POOL_U_FENCE  0x000c2ef8
#define L0_HP_OUT_BUF_POOL_NUM_SEG  0x00000005
#define L0_HP_OUT_BUF_POOL_SEG_SIZE 0x000003c0

#define L0_I2S_OUT_BUF_POOL_ALIGN    0x00000008
#define L0_I2S_OUT_BUF_POOL_L_FENCE  0x000c2efc
#define L0_I2S_OUT_BUF_POOL_ADDR     0x000c2f00
#define L0_I2S_OUT_BUF_POOL_SIZE     0x000012c0
#define L0_I2S_OUT_BUF_POOL_U_FENCE  0x000c41c0
#define L0_I2S_OUT_BUF_POOL_NUM_SEG  0x00000005
#define L0_I2S_OUT_BUF_POOL_SEG_SIZE 0x000003c0

#define L0_MFE_OUT_BUF_POOL_ALIGN    0x00000008
#define L0_MFE_OUT_BUF_POOL_L_FENCE  0x000c41c4
#define L0_MFE_OUT_BUF_POOL_ADDR     0x000c41c8
#define L0_MFE_OUT_BUF_POOL_SIZE     0x00000500
#define L0_MFE_OUT_BUF_POOL_U_FENCE  0x000c46c8
#define L0_MFE_OUT_BUF_POOL_NUM_SEG  0x00000008
#define L0_MFE_OUT_BUF_POOL_SEG_SIZE 0x000000a0

#define L0_WUWSR_IN_BUF_POOL_ALIGN    0x00000008
#define L0_WUWSR_IN_BUF_POOL_L_FENCE  0x000c46cc
#define L0_WUWSR_IN_BUF_POOL_ADDR     0x000c46d0
#define L0_WUWSR_IN_BUF_POOL_SIZE     0x00002a80
#define L0_WUWSR_IN_BUF_POOL_U_FENCE  0x000c7150
#define L0_WUWSR_IN_BUF_POOL_NUM_SEG  0x00000011
#define L0_WUWSR_IN_BUF_POOL_SEG_SIZE 0x00000280

/* Remainder AUDIO_WORK_AREA=0x00036eac */

#endif /* MEM_LAYOUT_H_INCLUDED */
