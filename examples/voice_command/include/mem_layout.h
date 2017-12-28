/*
 * mem_layout.h -- MemMgrLite layout definition.
 *
 * This file was created by mem_layout.conf
 * !!! CAUTION! don't edit this file manually !!!
 *
 *   Notes: (C) Copyright 2014 Sony Corporation
 */
#ifndef MEM_LAYOUT_H_INCLUDED
#define MEM_LAYOUT_H_INCLUDED

/*
 * Memory Manager Configurations
 */
//#define USE_MEMMGR_FENCE /* TODO This define macro should be set by Kconfig. */

/*
 * User defined constants
 */

/*
 * Memory devices
 */
/* AUD_SRAM: type=RAM, use=0x0003f440, remainder=0x00000bc0 */
#define AUD_SRAM_ADDR	0x000c0000
#define AUD_SRAM_SIZE	0x00040000

/* RESERVED: type=RAM, use=0x00000080, remainder=0x0003ff80 */
#define RESERVED_ADDR	0x0e000000
#define RESERVED_SIZE	0x00040000

/*
 * Fixed areas
 */
#define AUDIO_WORK_AREA_ALIGN   0x00020000
#define AUDIO_WORK_AREA_ADDR    0x000c0000
#define AUDIO_WORK_AREA_DRM     0x000c0000 /* _DRM is obsolete macro. to use _ADDR */
#define AUDIO_WORK_AREA_SIZE    0x0003c000

#define MSG_QUE_AREA_ALIGN   0x00000040
#define MSG_QUE_AREA_ADDR    0x000fc000
#define MSG_QUE_AREA_DRM     0x000fc000 /* _DRM is obsolete macro. to use _ADDR */
#define MSG_QUE_AREA_SIZE    0x00003140

#define MEMMGR_WORK_AREA_ALIGN   0x00000008
#define MEMMGR_WORK_AREA_ADDR    0x000ff140
#define MEMMGR_WORK_AREA_DRM     0x000ff140 /* _DRM is obsolete macro. to use _ADDR */
#define MEMMGR_WORK_AREA_SIZE    0x00000200

#define MEMMGR_DATA_AREA_ALIGN   0x00000008
#define MEMMGR_DATA_AREA_ADDR    0x000ff340
#define MEMMGR_DATA_AREA_DRM     0x000ff340 /* _DRM is obsolete macro. to use _ADDR */
#define MEMMGR_DATA_AREA_SIZE    0x00000100

#define SPL_MGR_AREA_ALIGN   0x00000008
#define SPL_MGR_AREA_ADDR    0x0e000000
#define SPL_MGR_AREA_DRM     0x0e000000 /* _DRM is obsolete macro. to use _ADDR */
#define SPL_MGR_AREA_SIZE    0x00000040

#define APU_LOG_AREA_ALIGN   0x00000008
#define APU_LOG_AREA_ADDR    0x0e000040
#define APU_LOG_AREA_DRM     0x0e000040 /* _DRM is obsolete macro. to use _ADDR */
#define APU_LOG_AREA_SIZE    0x00000040

/*
 * Memory Manager max work area size
 */
#define MEMMGR_MAX_WORK_SIZE  0x000000c4

/*
 * Pool IDs
 */
#define NULL_POOL	0
#define MIC_IN_BUF_POOL	1
#define I2S_IN_BUF_POOL	2
#define HP_OUT_BUF_POOL	3
#define I2S_OUT_BUF_POOL	4
#define MFE_OUT_BUF_POOL	5
#define WUWSR_IN_BUF_POOL	6

#define NUM_MEM_LAYOUTS	1
#define NUM_MEM_POOLS	7


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

/* Remainder AUDIO_WORK_AREA=0x00034eac */

#endif /* MEM_LAYOUT_H_INCLUDED */
